// FK detour. Hot path: atomics only, no logging, no allocation, no VirtualQuery, no locks.
// v0.2.0: motion matching pose blend (ApplyMotionMatching) before the Test A rotation.
//
// v0.1.1: in-game test of 0.1.0 showed tag matching gave matchedCalls=0, so the detour now counts
// three criteria (tag, D == *(animObj+0x18), R == metaRig) and applies according to matchMode.
// Player identification by desc[3] (qword at D+0x18) == trackA/trackB follows the approach of
// cyberpunk-vr-port (MIT, (c) 2026 dariulone), src/Hooks/AnimPose.cpp and
// src/Natives/VrikArming.cpp. Applying it to FK's D is an ASSUMPTION made observable through the
// matchedCalls counter.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>

#include "CommonUtil.hpp"
#include "FkHook.hpp"
#include "QuatMath.hpp"
#include "State.hpp"
#include "PoseHook.hpp"
#include "EarlyPoseRuntime.hpp"
#include "mm/AimOffsetRuntime.hpp"
#include "mm/MeleeAimRuntime.hpp"
#include "mm/MeleeBaseRuntime.hpp"
#include "mm/MeleePoseRuntime.hpp"
#include "mm/HeadLookRuntime.hpp"
#include "mm/GamePoseCapture.hpp"
#include "mm/PoseTrace.hpp"
#include "mm/BodyDeformerTrace.hpp"
#include "mm/CoreWriteRecord.hpp"
#include "BodyHelperRuntime.hpp"
#include "mm/NativeReloadContactsRuntime.hpp"
#include "mm/VaultTorso.hpp"
#include "mm/VehicleTransition.hpp"
#include "ragdoll/RagdollEpisode.hpp"
#include "ragdoll/RagdollSlotProbe.hpp"
#include "ragdoll/RdPoseTable.hpp"

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;

uint32_t ApplyPoseSnapshot(void*, uint8_t* aDesc, uint8_t* bones, uint32_t aCount,
                           uint64_t aMergedRig, const mm::integ::PoseSnapshot& snap, int64_t aNow)
{
    if (!aDesc || !bones || !aCount || !g_hot.mmActive.load(std::memory_order_acquire))
        return 0;
    // Ragdoll episode (ragdoll/RagdollEpisode.hpp): the engine owns V's pose; no MM blend, reload contacts or AO on any path.
    if (ragdoll::GameOwned())
        return 0;
    // Positive fade weight is not proof of TPP. Preserve the original FPP
    // records as soon as the camera withdraws permission, including AO.
    if (!g_hot.perspectiveWrite.Allows(aNow,
            g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed)))
    {
        // A denial (even a short one) ends the reload episode: an old correction must not be replayed after it. When another
        // animation job owns the state right now the reset is left as a request that the next owner honours first.
        mm::reload_contacts_runtime::ResetContinuityOnDenial();
        return 0;
    }
    if (!(snap.weight > 0.0f))
        return 0;
    if (snap.rig != aMergedRig)
    {
        g_hot.mmRigMismatch.value.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    const int64_t stale = g_hot.mmStaleTicks.load(std::memory_order_relaxed);
    if (aNow < snap.publishTicks || (stale > 0 && (aNow - snap.publishTicks) > stale))
    {
        g_hot.mmStaleSkips.value.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    // From here on the published pose and the layers on top of it (aim offset, melee, head look, reload contacts) write core bones; a
    // fully masked blend (n == 0 below) can still be followed by those layers, so the writer is recorded here.
    mm::corewrite::Mark(snap.source == mm::integ::kPoseSourceDriver ? mm::corewrite::kDriver : mm::corewrite::kHost);
    mm::integ::CaptureGamePoseSample(bones, aCount, aMergedRig, aNow);
    mm::reload_contacts_runtime::Prepared reloadConfig;
    mm::reload_contacts_runtime::Cycle reloadCycle;
    // Admission (mm::reload_contacts_runtime::Admit): the Mapping owner (opt-in, early-pose.txt) corrects every reload overlay; the default
    // FK owner corrects only the overlays listed in FkOwnerOverlay and never a purpose-built (authored) reload.
    const auto reloadOwner = earlyruntime::CurrentOwner() == earlyruntime::Owner::Mapping ?
        mm::reload_contacts_runtime::HookOwner::Mapping : mm::reload_contacts_runtime::HookOwner::FK;
    const bool reloadEligible = mm::reload_contacts_runtime::Admit(snap, reloadOwner);
    if (snap.reloadContactsEligible)
    {
        g_hot.rcSnapEligible.value.fetch_add(1, std::memory_order_relaxed);
        if (mm::reload_contacts_runtime::ReloadContactsOverlay(snap.traceOverlay))
        {
            g_hot.rcOverlayOk.value.fetch_add(1, std::memory_order_relaxed);
            if (reloadEligible)
                g_hot.rcOwnerOk.value.fetch_add(1, std::memory_order_relaxed);
        }
    }
    const bool reloadHaveConfig = reloadEligible && mm::reload_contacts_runtime::g_config.Read(reloadConfig);
    if (reloadHaveConfig)
        g_hot.rcHaveConfig.value.fetch_add(1, std::memory_order_relaxed);
    const bool reloadCaptured = reloadHaveConfig &&
        mm::reload_contacts_runtime::Capture(reloadConfig, snap, bones, aCount, true, reloadCycle);
    if (reloadCaptured)
        g_hot.rcCaptured.value.fetch_add(1, std::memory_order_relaxed);
    mm::posetrace::Frame trace;
    const bool tracing = mm::posetrace::g_dump.Enabled();
    bool captured = false;
    if (tracing)
    {
        trace.ticks = aNow; trace.desc = reinterpret_cast<uint64_t>(aDesc);
        trace.thread = GetCurrentThreadId(); trace.count = aCount; trace.snap = snap;
        captured = mm::posetrace::Capture(trace, 0, bones, aCount);
    }
    // melee reference pose guard (mm/MeleePoseGuard.hpp): a frame where the graph left the upper body on the skeleton's reference pose
    // gets the last good engine pose back before any layer reads it
    utpp::meleepose::GuardEngine(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.meleeAimWeight, aNow);
    // melee full body: when the engine pose of Hips is not where MM's Hips are (the kneeling engine base of Male V) the lower body stays on MM
    const float keepLowerOnMM = snap.lowerScale < 1.0f ? utpp::meleebase::LowerBodyKeepWeight(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap, aNow)
                                                       : utpp::meleebase::LowerBodyReset();
    // melee chest base correction: the key bones of the game pose, before the blend mixes them with the MM pose (see MeleeBaseRuntime.hpp)
    utpp::meleebase::CaptureEngineKeys(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.meleeAimWeight);
    const uint32_t n = mm::integ::BlendIntoRecords(bones, aCount, snap, snap.weight, keepLowerOnMM);
    if (captured) captured = mm::posetrace::Capture(trace, 1, bones, aCount);
    // Melee full pose (mm/MeleePose.hpp): while the melee full body hands the lower body over (standing strikes), the matched clip's
    // own Hips / Spine / legs, time-synchronised with it. After the blend (it replaces the engine base and the kept MM legs alike),
    // before the chest correction, which it scales down by its share.
    utpp::meleepose::Apply(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.lowerScale, aNow);
    if (n)
    {
        g_hot.mmAppliedCalls.value.fetch_add(1, std::memory_order_relaxed);
        g_hot.mmBonesWritten.value.fetch_add(n, std::memory_order_relaxed);
    }
    // 0.6.3 aim offset: on the final local pose (game upper body blended in by the mask), before FK. Guards inside.
    aimofs::Apply(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.weight, snap.upperScale, aNow);
    // Melee chest base correction (2026-11-13, mm/MeleeBase.hpp): the chest of a recognised fists clip frame goes back to the authored
    // orientation on the hips / spine the game shows. Before the tilt, which composes on top of it.
    utpp::meleebase::Apply(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.weight, snap.upperScale, snap.meleeAimWeight, snap.meleeBase, aNow);
    // Melee aim tilt (2026-11-09, mm/MeleeAim.hpp): spine pitch / yaw toward the camera aim point while the melee mask reason
    // is on (snap.meleeAimWeight 0 = no-op). Its own layer: the gun aim offset above stays off in melee.
    utpp::meleeaim::Apply(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.weight, snap.upperScale, snap.meleeAimWeight,
                          aNow);
    // Head look (mm/HeadLook.hpp): Spine3 (small share, no weapon in hand), Neck / Neck1 / Head and the eyes toward the target the CET
    // module sends (UTPPSetHeadLook). After every other layer so that it measures the head the pose really has; before the reload
    // contacts, which stay the last core write. Gate = MM ownership x (1 - melee fade); idle: two relaxed loads, no write.
    utpp::headlook::Apply(bones, aCount, reinterpret_cast<void*>(aMergedRig), snap.weight, snap.upperScale, snap.meleeAimWeight,
                          snap.traceOverlay, aNow);
    if (reloadCaptured)
    {
        // g_continuity belongs to one animation job at a time. A job that loses the lock keeps the ORDINARY baseline records
        // for this call (the pose the game shows without the contact solve) on purpose: repeating the last output would
        // read state another thread is writing, and a stale repeat is not better than the baseline. The state itself stays
        // consistent (the owner's fade / limiter continue from its own last output); a perspective denial that finds the
        // state busy is kept as a reset request (ResetContinuityOnDenial).
        mm::reload_contacts_runtime::ContinuityLock lock;
        if (lock)
        {
            LARGE_INTEGER commitNow{};
            QueryPerformanceCounter(&commitNow);
            mm::native_reload_contacts::Result reloadResult;
            const bool permitted = g_hot.armed.load(std::memory_order_acquire) &&
                g_hot.perspectiveWrite.Allows(commitNow.QuadPart,
                    g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed));
            // Last core operation before helper evaluation: no later AO may move the preserved contacts.
            const int64_t qpcFrequency = util::QpcFrequency();
            if (mm::reload_contacts_runtime::Finish(reloadConfig, snap, bones, aCount,
                permitted, reloadCycle, reloadResult, snap.reloadContactsWeight,
                mm::reload_contacts_runtime::kAuthoredContactMinReach,
                &mm::reload_contacts_runtime::g_continuity, int64_t(commitNow.QuadPart), qpcFrequency))
                g_hot.rcFinishWritten.value.fetch_add(1, std::memory_order_relaxed);
        }
        else
            g_hot.rcLockLost.value.fetch_add(1, std::memory_order_relaxed);
    }
    else if (reloadHaveConfig)
    {
        g_hot.rcHoldOnly.value.fetch_add(1, std::memory_order_relaxed);
        mm::reload_contacts_runtime::ContinuityLock lock;
        if (lock) // same single-owner rule as above: a lost lock leaves the baseline
        {
            // Eligible reload but no native cycle this frame: fade the last accepted correction out (no-op without one).
            LARGE_INTEGER holdNow{};
            QueryPerformanceCounter(&holdNow);
            const int64_t holdFrequency = util::QpcFrequency();
            const bool permitted = g_hot.armed.load(std::memory_order_acquire) &&
                g_hot.perspectiveWrite.Allows(holdNow.QuadPart,
                    g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed));
            mm::reload_contacts_runtime::HoldOnly(reloadConfig, snap, bones, aCount, permitted,
                mm::reload_contacts_runtime::g_continuity, int64_t(holdNow.QuadPart), holdFrequency);
        }
    }
    else if (!reloadEligible && mm::reload_contacts_runtime::ObservesRifle(snap.traceOverlay, reloadOwner) &&
        mm::reload_contacts_runtime::g_config.Read(reloadConfig))
    {
        // Rifle out, no correction this frame: remember the published weapon / left hand (read-only) so that the first frames of the
        // next reload start from the pose the player saw, not from the game's envelope jump (NativeReloadContactsRuntime PublishedStart).
        mm::reload_contacts_runtime::ContinuityLock lock;
        if (lock)
        {
            LARGE_INTEGER obsNow{};
            QueryPerformanceCounter(&obsNow);
            mm::reload_contacts_runtime::ObservePublished(reloadConfig, snap, bones, aCount,
                mm::reload_contacts_runtime::g_continuity, int64_t(obsNow.QuadPart));
        }
    }

    if (captured && mm::posetrace::Capture(trace, 2, bones, aCount)) mm::posetrace::g_dump.Push(trace);
    return n;
}

namespace
{
FK_t g_originalFK = nullptr;

// Ragdoll R1o probe (ragdoll/RdPoseTable.hpp): fk row per player-tagged / player-rig descriptor, model positions after FK.
// No C++ objects with destructors; a bad descriptor faults into the handler and the sample is skipped.
// Hips translation before FK (local), read for every descriptor; the capture below decides whether it is V's.
__declspec(noinline) void RdFkPre(const uint8_t* aDesc, float (&aOut)[4])
{
    __try
    {
        const uint8_t* bones = *reinterpret_cast<uint8_t* const*>(aDesc + kDescBonesOff);
        const uint32_t count = *reinterpret_cast<const uint32_t*>(aDesc + kDescCountOff);
        const int32_t hips = rdpose::g_idx[0].load(std::memory_order_relaxed);
        if (!bones || hips < 0 || static_cast<uint32_t>(hips) >= count)
            return;
        const auto* b = reinterpret_cast<const float*>(bones + static_cast<size_t>(hips) * kBoneStride);
        for (int j = 0; j < 4; ++j)
            aOut[j] = b[j];
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
}

__declspec(noinline) void RdFkCapture(const uint8_t* aDesc, void* aRig, bool aMatched, const float (&aPre)[4])
{
    __try
    {
        const uint64_t tag = *reinterpret_cast<const uint64_t*>(aDesc + kDescTagOff);
        const uint64_t ta = g_hot.trackA.load(std::memory_order_relaxed);
        const uint64_t tb = g_hot.trackB.load(std::memory_order_relaxed);
        const uint64_t pr = g_hot.playerRig.load(std::memory_order_relaxed);
        const bool player = (tag != 0 && (tag == ta || tag == tb)) || (pr != 0 && reinterpret_cast<uint64_t>(aRig) == pr);
        if (!player)
            return;
        rdpose::Row* r = rdpose::Find(rdpose::g_fk, reinterpret_cast<uint64_t>(aDesc));
        if (!r)
            return;
        const uint8_t* bones = *reinterpret_cast<uint8_t* const*>(aDesc + kDescBonesOff);
        const uint32_t count = *reinterpret_cast<const uint32_t*>(aDesc + kDescCountOff);
        r->a.store(reinterpret_cast<uint64_t>(aRig), std::memory_order_relaxed);
        r->b.store(tag, std::memory_order_relaxed);
        r->count.store(count, std::memory_order_relaxed);
        if (aMatched)
            r->aux.store(1, std::memory_order_relaxed);
        r->calls.fetch_add(1, std::memory_order_relaxed);
        r->seq.store(rdpose::g_seq.fetch_add(1, std::memory_order_relaxed), std::memory_order_relaxed);
        rdpose::Store4(r->pre, aPre);
        if (!bones)
            return;
        for (int i = 0; i < 3; ++i)
        {
            const int32_t k = rdpose::g_idx[i].load(std::memory_order_relaxed);
            if (k >= 0 && static_cast<uint32_t>(k) < count)
                rdpose::Store4(r->v[i], reinterpret_cast<const float*>(bones + static_cast<size_t>(k) * kBoneStride));
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
}

void ApplyMotionMatching(uint8_t* aDesc, void* aRig, uint32_t aCount, int64_t aNow)
{
    // Ragdoll episode: first line, before the seated neck cut (it runs for every owner) and the FK-side vehicle transition / MM blend.
    if (ragdoll::GameOwned())
        return;
    // Skin helper recompute on the frames of the weight-0 writers below (BodyHelperRuntime.hpp SetZeroWriterRecompute): the window of
    // this call, opened only while one of them can write and the body is shown in third person - by the vehicle camera (mounted
    // third-person flag of the vehicle layer) for the vehicle transition, the vehicle guards and the neck cut; by the camera's
    // pose-write permission for the vault torso.
    if (bodyhelperruntime::Enabled() && bodyhelperruntime::ZeroWriterRecompute())
    {
        const bool vehicleThirdPerson = mm::vehtrans::Wanted() && mm::vehtrans::g_mountTpp.load(std::memory_order_acquire);
        if (vehicleThirdPerson ||
            (mm::vaulttorso::rt::Active(aNow, util::QpcFrequency()) &&
             g_hot.perspectiveWrite.Allows(aNow, g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed))))
            mm::corewrite::MarkZeroWindow(vehicleThirdPerson);
    }
    // Seated neck cut (mm/VehicleTransition.hpp SEATED NECK CUT): the game adds the seating-edge upper-body pop after the early
    // mapping job, so the guard runs on the pose the FK pass receives, for every write owner (armed only by a mount).
    if (mm::vehtrans::Wanted() &&
        mm::vehtrans::NeckGuard(*reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff), aCount, reinterpret_cast<uint64_t>(aRig), aNow))
        mm::corewrite::Mark(mm::corewrite::kNeckGuard);
    // Vault torso limiter (mm/VaultTorso.hpp): a released vault over a vehicle with a ranged weapon in hand shows the game's own pose with the
    // weapon layer's rigid spine (MM weight 0, so no MM / aim offset layer runs). Same stage and rules as the seated neck cut: the pose the FK
    // pass receives, every write owner, TPP only. NOT gated by mmActive: the vault is the game's pose and the MM weight is 0 there (hard-out
    // within ~0.12 s of the release). One relaxed load while idle (the weight is 0 outside the episode).
    {
        const int64_t vaultTps = util::QpcFrequency();
        if (mm::vaulttorso::rt::Active(aNow, vaultTps) &&
            mm::vaulttorso::rt::FkPass(*reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff), aCount, aRig, aNow, vaultTps,
                                       g_hot.perspectiveWrite.Allows(aNow, g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed))))
            mm::corewrite::Mark(mm::corewrite::kVaultTorso);
    }
    // Ownership is session-scoped, not a per-frame fallback. A rejected early
    // mapping pass must never reopen the late FK write path.
    if (earlyruntime::CurrentOwner() != earlyruntime::Owner::FK)
        return;
    // Vehicle mount / exit transition (mm/VehicleTransition.hpp). Wanted() is false unless MMDriver called BeginMount or
    // SetVehicleCapture, so the on-foot path below is unchanged. It runs at MM weight 0, keeps the perspective gate and the
    // rig check (inside); a transition frame skips the MM blend, reload contacts, AO and CaptureGamePoseSample.
    if (mm::vehtrans::Wanted())
    {
        auto* vehBones = *reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff);
        const bool vehAllows = g_hot.perspectiveWrite.Allows(aNow,
            g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed));
        if (mm::vehtrans::FkSide(vehBones, aCount, reinterpret_cast<uint64_t>(aRig), aNow, vehAllows))
            return;
    }
    if (!g_hot.mmActive.load(std::memory_order_acquire) ||
        !g_hot.perspectiveWrite.Allows(aNow, g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed)))
        return;
    mm::integ::PoseSnapshot snap;
    if (!g_mmPose.Read(snap))
    {
        g_hot.mmTornReads.value.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    auto* bones = *reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff);
    ApplyPoseSnapshot(nullptr, aDesc, bones, aCount, reinterpret_cast<uint64_t>(aRig), snap, aNow);
    // The game filled the shadow skeleton from the main bones before this write; set it again from the written pose. The published
    // writer mark is set once the pose passed every gate (third person, weight > 0, rig, age), also when the blend itself was fully
    // masked and only the layers on top of it (melee, aim, head look, reload contacts) wrote.
    if (snap.weight > 0.0f && (mm::corewrite::Current() & mm::corewrite::kPublishedMask))
        shadowcopy::Apply(bones, aCount, reinterpret_cast<uint64_t>(aRig));
}

// No C++ objects with destructors here; may fault on a bad descriptor, caught by the caller.
void ApplyUnguarded(uint8_t* aDesc, void* aRig, int64_t aNow)
{
    const uint64_t tag = *reinterpret_cast<const uint64_t*>(aDesc + kDescTagOff);
    const uint32_t count = *reinterpret_cast<const uint32_t*>(aDesc + kDescCountOff);
    const uint64_t a = g_hot.trackA.load(std::memory_order_relaxed);
    const uint64_t b = g_hot.trackB.load(std::memory_order_relaxed);
    const uint64_t pd = g_hot.playerDesc.load(std::memory_order_relaxed);
    const uint64_t pr = g_hot.playerRig.load(std::memory_order_relaxed);

    // Diagnostics: which identification criterion recognises the player's FK call.
    const bool byTag = tag != 0 && (tag == a || tag == b);
    const bool byDesc = pd != 0 && reinterpret_cast<uint64_t>(aDesc) == pd;
    const bool byRig = pr != 0 && reinterpret_cast<uint64_t>(aRig) == pr;
    if (byTag)
        g_hot.matchTag.value.fetch_add(1, std::memory_order_relaxed);
    if (byDesc)
        g_hot.matchDesc.value.fetch_add(1, std::memory_order_relaxed);
    if (byRig)
        g_hot.matchRig.value.fetch_add(1, std::memory_order_relaxed);
    const uint32_t playerBones = g_hot.playerBones.load(std::memory_order_relaxed);
    if (playerBones != 0 && count == playerBones)
    {
        g_hot.sameCountCalls.value.fetch_add(1, std::memory_order_relaxed);
        g_hot.sampleDesc.store(reinterpret_cast<uint64_t>(aDesc), std::memory_order_relaxed);
        g_hot.sampleRig.store(reinterpret_cast<uint64_t>(aRig), std::memory_order_relaxed);
        g_hot.sampleTag.store(tag, std::memory_order_relaxed);
    }

    bool match = false;
    switch (g_hot.matchMode.load(std::memory_order_relaxed))
    {
    case 0: match = byTag; break;
    case 1: match = byDesc; break;
    case 2: match = byRig; break;
    default: match = byTag || byDesc || byRig; break;
    }
    if (!match)
        return;

    g_hot.matchedCalls.value.fetch_add(1, std::memory_order_relaxed);
    g_hot.lastMatchedCount.store(count, std::memory_order_relaxed);
    mm::corewrite::MarkPlayerCall();

    // Motion matching first; the Test A rotation below (if enabled) applies on top of it.
    // Explicit helper capture also works with MM disabled; this is not pose ownership.
    if (mm::bodytrace::g_capture.Active())
    {
        mm::bodytrace::Frame helperFrame;
        uint8_t* helperBones = *reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff);
        const bool helperCaptured = mm::bodytrace::g_capture.Begin(helperFrame, helperBones, count,
            reinterpret_cast<uint64_t>(aRig), reinterpret_cast<uint64_t>(aDesc), aNow, GetCurrentThreadId());
        ApplyMotionMatching(aDesc, aRig, count, aNow);
        if (helperCaptured)
            mm::bodytrace::g_capture.End(helperFrame, helperBones, count,
                reinterpret_cast<uint64_t>(aRig), reinterpret_cast<uint64_t>(aDesc));
    }
    else ApplyMotionMatching(aDesc, aRig, count, aNow);

    if (earlyruntime::CurrentOwner() != earlyruntime::Owner::FK || !g_hot.enabled.load(std::memory_order_relaxed) ||
        ragdoll::GameOwned())
        return;
    const float angle = g_hot.angleDeg.load(std::memory_order_relaxed);
    if (angle == 0.0f)
        return;
    const int32_t bone = g_hot.boneIndex.load(std::memory_order_relaxed);
    if (bone < 0 || static_cast<uint32_t>(bone) >= count)
    {
        g_hot.badIndexCalls.value.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    uint8_t* bones = *reinterpret_cast<uint8_t**>(aDesc + kDescBonesOff);
    if (!bones)
        return;

    float* rot = reinterpret_cast<float*>(bones + static_cast<size_t>(bone) * kBoneStride + kBoneRotOff);
    if (quat::ApplyLocalRotation(rot, g_hot.axis.load(std::memory_order_relaxed), angle))
        g_hot.appliedCalls.value.fetch_add(1, std::memory_order_relaxed);
}

int FaultFilter(unsigned long aCode)
{
    g_hot.faultCode.store(static_cast<uint32_t>(aCode), std::memory_order_relaxed);
    return EXCEPTION_EXECUTE_HANDLER;
}

__declspec(noinline) void ApplyGuarded(uint8_t* aDesc, void* aRig, int64_t aNow)
{
    __try
    {
        ApplyUnguarded(aDesc, aRig, aNow);
    }
    __except (FaultFilter(GetExceptionCode()))
    {
        g_hot.enabled.store(false, std::memory_order_relaxed);
        g_hot.mmActive.store(false, std::memory_order_relaxed);
        g_hot.fault.store(true, std::memory_order_release);
    }
}
} // namespace

void* FkDetour(void* aDesc, void* aRig)
{
    LARGE_INTEGER t0, t1{};
    QueryPerformanceCounter(&t0);
    mm::corewrite::Begin(); // core write record of this call (mm/CoreWriteRecord.hpp): marked by the layers, read by the helper recompute
    // Own-cost timing (two more QPC readings and shared counters per call) only while the detailed diagnostics are on.
    const bool timing = g_hot.timing.load(std::memory_order_relaxed);
    g_hot.totalCalls.value.fetch_add(1, std::memory_order_relaxed);
    const uint64_t matchedBefore = g_hot.matchedCalls.value.load(std::memory_order_relaxed);
    if (aDesc && g_hot.armed.load(std::memory_order_acquire) && !g_hot.fault.load(std::memory_order_acquire))
        ApplyGuarded(static_cast<uint8_t*>(aDesc), aRig, t0.QuadPart);
    // Final LOCAL observation/candidate evaluation, after any permitted core writes.
    posediag::NoteBeforeFK(aDesc, aRig, t0.QuadPart);
    // A player call the helper recompute did not see (no mapping-cycle policy for this rig, ragdoll, helpers disabled) is still counted.
    if (const uint32_t record = mm::corewrite::Current();
        (record & mm::corewrite::kPlayerCall) && !(record & mm::corewrite::kEvaluated))
        bodyhelperruntime::NoteNotEvaluated(record);
    if (timing)
    {
        QueryPerformanceCounter(&t1);
        const uint64_t own = static_cast<uint64_t>(t1.QuadPart - t0.QuadPart);
        g_hot.ownTicks.value.fetch_add(own, std::memory_order_relaxed);
        uint64_t prevMax = g_hot.ownTicksMax.load(std::memory_order_relaxed);
        while (own > prevMax && !g_hot.ownTicksMax.compare_exchange_weak(prevMax, own, std::memory_order_relaxed))
        {
        }
    }
    // matchedCalls is shared across threads, so this attribution is approximate under contention.
    const bool playerCall = g_hot.matchedCalls.value.load(std::memory_order_relaxed) != matchedBefore;
    // R1o / R1p probe tables only while a ragdoll episode runs: idle they cost nothing.
    const bool owned = ragdoll::GameOwned();
    const bool rdOn = owned && aDesc && rdpose::g_idx[0].load(std::memory_order_relaxed) >= 0;
    float rdPre[4] = {};
    if (rdOn)
        RdFkPre(static_cast<const uint8_t*>(aDesc), rdPre);
    void* result = g_originalFK(aDesc, aRig);
    // FK of a descriptor ends its merge evaluation: the keep store must not outlive it (RdKeep.hpp). Deliberately outside
    // ApplyGuarded (which skips when disarmed or faulted).
    if (owned && aDesc)
        ragdollslot::Keep().CloseEvaluation(reinterpret_cast<uint64_t>(aDesc));
    if (rdOn)
        RdFkCapture(static_cast<const uint8_t*>(aDesc), aRig, playerCall, rdPre);
    // Pelvis telemetry for the ragdoll natives (read-only; idle: 4 Hz standing sample, cheap reject for every other call).
    ragdollslot::OnPlayerFk(static_cast<const uint8_t*>(aDesc), aRig, t0.QuadPart);
    if (playerCall && timing)
    {
        LARGE_INTEGER t2;
        QueryPerformanceCounter(&t2);
        g_hot.playerFkTicks.value.fetch_add(static_cast<uint64_t>(t2.QuadPart - t1.QuadPart), std::memory_order_relaxed);
    }
    return result;
}

FK_t* FkOriginalSlot()
{
    return &g_originalFK;
}

void RdFkTraceSetBones(int32_t aHips, int32_t aHead, int32_t aFoot)
{
    rdpose::g_idx[1].store(aHead, std::memory_order_relaxed);
    rdpose::g_idx[2].store(aFoot, std::memory_order_relaxed);
    rdpose::g_idx[0].store(aHips, std::memory_order_relaxed); // gate last
}

namespace shadowcopy
{
namespace
{
constexpr uint32_t kPairs = 24, kToes = 2;
constexpr const char* kMainNames[kPairs] = {
    "Hips", "Spine", "Spine1", "Spine2", "Spine3", "LeftUpLeg", "RightUpLeg", "LeftLeg", "RightLeg", "LeftFoot", "RightFoot",
    "LeftShoulder", "RightShoulder", "Neck", "Neck1", "Head", "LeftArm", "RightArm", "LeftForeArm", "RightForeArm", "LeftHand",
    "RightHand", "WeaponLeft", "WeaponRight"};
constexpr const char* kToeNames[kToes][3] = {{"LeftToeBase", "LeftHeel", "LeftFoot"}, {"RightToeBase", "RightHeel", "RightFoot"}};
struct Table
{
    uint64_t rig = 0;             // 0 = off
    uint32_t count = 0, pad = 0;
    std::array<int32_t, kPairs> dst{}, src{};
    std::array<int32_t, kToes> toeDst{}, toeHeel{}, toeSrc{};
};
// One consistent table per read (a FK call on another thread may run while the arm path replaces it).
mm::integ::SeqlockBuffer<Table> g_table;
std::atomic<ArmStatus> g_status{ArmStatus::Off};

int32_t IndexOf(std::span<const uint64_t> aNames, uint64_t aHash)
{
    for (size_t i = 0; i < aNames.size(); ++i)
        if (aNames[i] == aHash) return static_cast<int32_t>(i);
    return -1;
}
uint64_t ShadowHash(const char* aMain) { return mm::integ::CNameHash(std::string("shadow_") + aMain); }
uint32_t Fail(ArmStatus aStatus) { g_status.store(aStatus, std::memory_order_release); return 0; }

void Rotate(const quat::Q& q, const float* v, float* out)
{
    // v' = v + 2w (u x v) + 2 u x (u x v)
    const float ux = q.x, uy = q.y, uz = q.z;
    const float cx = uy * v[2] - uz * v[1], cy = uz * v[0] - ux * v[2], cz = ux * v[1] - uy * v[0];
    const float dx = uy * cz - uz * cy, dy = uz * cx - ux * cz, dz = ux * cy - uy * cx;
    out[0] = v[0] + 2.f * (q.w * cx + dx);
    out[1] = v[1] + 2.f * (q.w * cy + dy);
    out[2] = v[2] + 2.f * (q.w * cz + dz);
}
} // namespace

uint32_t Arm(uint64_t aRig, std::span<const uint64_t> aNames, std::span<const int16_t> aParents)
{
    Disarm();
    if (!aRig || aNames.empty() || aNames.size() != aParents.size()) return Fail(ArmStatus::Off);
    Table t;
    t.rig = aRig;
    t.count = static_cast<uint32_t>(aNames.size());
    uint32_t shadowFound = 0, missing = 0;
    for (uint32_t i = 0; i < kPairs; ++i)
    {
        t.src[i] = IndexOf(aNames, mm::integ::CNameHash(kMainNames[i]));
        t.dst[i] = IndexOf(aNames, ShadowHash(kMainNames[i]));
        shadowFound += t.dst[i] >= 0;
        missing += t.src[i] < 0 || t.dst[i] < 0;
    }
    for (uint32_t k = 0; k < kToes; ++k)
    {
        t.toeDst[k] = IndexOf(aNames, ShadowHash(kToeNames[k][0]));
        t.toeSrc[k] = IndexOf(aNames, mm::integ::CNameHash(kToeNames[k][0]));
        t.toeHeel[k] = IndexOf(aNames, mm::integ::CNameHash(kToeNames[k][1]));
        shadowFound += t.toeDst[k] >= 0;
        missing += t.toeDst[k] < 0 || t.toeSrc[k] < 0 || t.toeHeel[k] < 0;
    }
    if (!shadowFound) return Fail(ArmStatus::NoShadowBones);
    if (missing) return Fail(ArmStatus::MissingBone);
    // Same shape: the parent of shadow_X is shadow_(parent of X) for a paired parent, else the same bone (Root for the hips).
    auto shadowOf = [&t](int32_t aMain) -> int32_t {
        for (uint32_t i = 0; i < kPairs; ++i)
            if (t.src[i] == aMain) return t.dst[i];
        return aMain;
    };
    for (uint32_t i = 0; i < kPairs; ++i)
        if (aParents[static_cast<size_t>(t.dst[i])] != shadowOf(aParents[static_cast<size_t>(t.src[i])]))
            return Fail(ArmStatus::ShapeMismatch);
    for (uint32_t k = 0; k < kToes; ++k)
    {
        const int32_t foot = IndexOf(aNames, mm::integ::CNameHash(kToeNames[k][2]));
        if (aParents[static_cast<size_t>(t.toeSrc[k])] != t.toeHeel[k] || aParents[static_cast<size_t>(t.toeHeel[k])] != foot ||
            aParents[static_cast<size_t>(t.toeDst[k])] != shadowOf(foot))
            return Fail(ArmStatus::ShapeMismatch);
    }
    g_table.Publish(t);
    g_status.store(ArmStatus::Ready, std::memory_order_release);
    return kPairs + kToes;
}

void Disarm()
{
    g_table.Publish(Table{});
    g_status.store(ArmStatus::Off, std::memory_order_release);
}

ArmStatus Status() { return g_status.load(std::memory_order_acquire); }
const char* StatusName()
{
    switch (Status())
    {
    case ArmStatus::NoShadowBones: return "no-shadow-bones";
    case ArmStatus::Ready: return "ready";
    case ArmStatus::MissingBone: return "missing-bone";
    case ArmStatus::ShapeMismatch: return "shape-mismatch";
    default: return "off";
    }
}

uint32_t Apply(uint8_t* aBones, uint32_t aCount, uint64_t aRig)
{
    Table t;
    if (!aBones || !aRig || !g_table.Read(t) || t.rig != aRig || aCount != t.count) return 0;
    // indices come from a rig of t.count bones, so they are inside a pose of aCount == t.count records
    for (uint32_t i = 0; i < kPairs; ++i)
        std::memcpy(aBones + static_cast<size_t>(t.dst[i]) * kBoneStride,
                    aBones + static_cast<size_t>(t.src[i]) * kBoneStride, kBoneStride);
    for (uint32_t k = 0; k < kToes; ++k)
    {
        // local(shadow_ToeBase) = local(Heel) * local(ToeBase): translation, rotation quat, scale (float4 each)
        const float* h = reinterpret_cast<const float*>(aBones + static_cast<size_t>(t.toeHeel[k]) * kBoneStride);
        const float* s = reinterpret_cast<const float*>(aBones + static_cast<size_t>(t.toeSrc[k]) * kBoneStride);
        float out[12];
        const quat::Q hq{h[4], h[5], h[6], h[7]}, sq{s[4], s[5], s[6], s[7]};
        const float scaled[3] = {h[8] * s[0], h[9] * s[1], h[10] * s[2]};
        Rotate(hq, scaled, out);
        out[0] += h[0]; out[1] += h[1]; out[2] += h[2]; out[3] = s[3];
        const quat::Q q = quat::Mul(hq, sq);
        out[4] = q.x; out[5] = q.y; out[6] = q.z; out[7] = q.w;
        out[8] = h[8] * s[8]; out[9] = h[9] * s[9]; out[10] = h[10] * s[10]; out[11] = s[11];
        std::memcpy(aBones + static_cast<size_t>(t.toeDst[k]) * kBoneStride, out, sizeof out);
    }
    return kPairs + kToes;
}
} // namespace shadowcopy
} // namespace utpp

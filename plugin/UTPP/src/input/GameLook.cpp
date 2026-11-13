// UTPP 0.6.5 game look write: the game's FPP look = the UTPP world camera direction.
//
// Evidence (2.31, exe 3.0.80.51928 SHA-256 a7de8294...0991, Ghidra decompile;
// scripts Source\tools\ghidra\DecompileAt / RefsTo / StringXrefs / DumpPtrs /
// FuncsByDisps.java):
//  1. gameFPPCameraComponent ctor FUN_14069cff0 (vtable 0x142b031d0), param set apply FUN_14069d4e0 (headingLocked +0x3F6,
//     maintainWorldSpaceHeading +0x409, sensitivityMultX/Y +0x498/+0x49C, heading reference quaternion +0x3B0). A change of
//     headingLocked calls RotateEntityToCamera FUN_14080404c (unlock) or FUN_140804320 (lock, "OnHeadingLocked" CName).
//  2. The look is NOT stored in the component: the per-tick input update FUN_1404e1638(comp, dt, inX, inY, extX, extY, flag)
//     reads the animation graph outputs "YawInput" -> +0x42C and "PitchInput" -> +0x430 (FUN_1404e2968 via the graph
//     variable source at +0x380; 999.0 when missing), scales the input by sensitivityMult, applies
//     maintainWorldSpaceHeading (subtracts the entity's turn since +0x3B0) and calls FUN_1404e0d88(comp, dt, yawDelta,
//     pitchDelta, extYaw, extPitch), which writes the deltas into the AnimFeature_FPPCamera at +0x360 (deltaYaw +0x44,
//     deltaYawExternal +0x48, deltaPitch +0x60, deltaPitchExternal +0x64, yawMultiplier +0xC4 = rubber band factor; the
//     clamp there is "yawMultiplier * yawDelta + YawInput" against the yaw limits -> the graph integrates
//     YawInput += yawMultiplier * deltaYaw). The graph node (animAnimNode_FPPCamera) drives the camera bones.
//  3. RotateEntityToCamera: new entity yaw = quatYaw(entity world orientation) + YawInput (FUN_1402d057c = atan2(2(wz-xy),
//     1-2(x^2+z^2)) in degrees, the CET convention atan2(-x, y)); entity world orientation = *(entity+0xB0)+0xF0
//     (FUN_140520f84). So world look yaw = entity yaw + YawInput; pitch up positive (ForceDownwardPitch: -70/-70).
//  4. Why headingLocked / VehicleCombat did not hold the look: the lock only makes YawInput relative to the entity (it
//     turns with V); maintainWorldSpaceHeading subtracts the entity turn scaled by sensitivityMultX and only on the
//     mouse-input branch, and the whole update is skipped while +0x4B8 (unlock blend) > 0.
//  5. (round 2) The apply itself can change the delta: FUN_1404e32c8 reads the
//     graph variables PitchInput / PitchRef / YawInput / YawRef; when they exist, the rubber band FUN_1404e12f0 (params
//     +0x444..+0x460, limits +0x3E8 / +0x3EC / +0x3E0 / +0x3E4) returns the multiplier (+0xC4 / +0xC8) and a delta that
//     REPLACES deltaYaw / deltaPitch when it is non-zero; yawMultiplier * delta + YawInput is clamped to the yaw limits.
//     FUN_1404e1638 writes resetYawSpeed / resetPitchSpeed (+0x78 / +0x7C) before the apply (non-zero only with external
//     input). Which of these (or a graph-side blend) overrides the look while adjustTransform turns V is not known: the
//     status counters featRepl / resetTicks / overrideTicks and rawErr show it in game.
// Fix: detour FUN_1404e0d88. While a target is set (UTPPSetGameLook, game thread, expires after 0.25 s) and the heading is
// locked, the yaw / pitch delta of the player's FPP camera component is replaced by the delta that brings YawInput /
// PitchInput to (targetYaw - entityYaw, targetPitch) (0.6.5b: feed-forward + low-gain feedback, stable for a graph delay
// of 0 or 1 tick, divergence trip on unexplained error growth: GameLookCore.hpp); the external deltas are zeroed.
// Otherwise the call passes through unchanged. Hot path: atomics, a try-lock and SEH reads.
// 0.6.5b: UTPPSetGameLook(yaw, pitch, entityYawAheadDeg): the entity turn utpp_bodyturn requested for the coming tick (its
// adjustTransform lands after this camera update in the same tick - measured, GameLookCore.hpp "ahead order"); used once by
// the next write (one-shot), so the look does not turn with V for a tick and then get pulled back (in game: up to 18 deg /
// frame at 60 fps while aim align ran). Round 2 (2026-10-13 review): a call with ahead 0 no longer clears a pending,
// unconsumed ahead (the camera's own call later in the same CET frame used to drop utpp_bodyturn's step); non-zero
// aheads add up until the next write consumes them (stale after the target TTL). The ticks around an ahead are override
// ticks for the controller (the game overrides the FPP look while adjustTransform turns V: log 01:05 / 01:25).
// Round 2 diagnostics (read only, after the game's apply): the feature delta / multipliers the graph integrates (fed back to
// the controller's prediction: Controller::NoteApplied), how often the apply changed our delta (rubber band / yaw clamp),
// resetYawSpeed / resetPitchSpeed (+0x78 / +0x7C, set by the input update FUN_1404e1638 before the apply) and the override
// weight (+0xDC) - to identify the mechanism of the override in game. A trip turns the write off for kTripCooldown s, then
// SetTarget re-arms it (at most kMaxTrips per game session; was: off for the session).
// Install checks (fail closed): the prologue of FUN_1404e0d88, its only call site in FUN_1404e1638 (feature +0x360), the
// YawInput / PitchInput refresh (+0x42C / +0x430 -> FUN_1404e2968), FUN_1404e2968 (+0x380) and FUN_140520f84 (+0xB0 / +0xF0).
// 2026-10-28 vehicle edge zeroing (OnFeed, GameLookCore.hpp EdgeZero): after the PSM vehicle bit clears in third person the
// hook drives YawInput to 0 for 0.3 s (no heading lock or target needed; pitch untouched), so the on-foot controller has no
// look offset to turn V onto (unmount swivel, feed yaw +10.9 -> -8.3 in 0.2 s). The component is resolved at the edge.
// Runtime checks (per SetTarget): RTTI sizes / property offsets, the component of the player (GetFPPCameraComponent, owner
// +0x50), the feature type at +0x360, the graph source at +0x380, and the raw entity yaw against GetWorldOrientation.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include <intrin.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/CProperty.hpp>
#include <RED4ext/Scripting/Natives/Quaternion.hpp>
#include <RED4ext/Scripting/Utils.hpp>

#include "../CommonUtil.hpp"
#include "../PluginLog.hpp"
#include "../Arming.hpp"
#include "GameLook.hpp"
#include "GameLookCore.hpp"

namespace utpp::gamelook
{
namespace
{
// gameFPPCameraComponent (size 0x510)
constexpr uint32_t kCompSize = 0x510;
constexpr uint32_t kOffOwner = 0x50;          // entIComponent owner (RED4ext.SDK entIComponent.hpp)
constexpr uint32_t kOffFeature = 0x360;       // AnimFeature_FPPCamera* (unreflected)
constexpr uint32_t kOffVarSource = 0x380;     // graph variable source (unreflected)
constexpr uint32_t kOffPitchMin = 0x3E0;      // reflected
constexpr uint32_t kOffPitchMax = 0x3E4;      // reflected
constexpr uint32_t kOffYawMaxLeft = 0x3E8;    // reflected
constexpr uint32_t kOffYawMaxRight = 0x3EC;   // reflected
constexpr uint32_t kOffHeadingLocked = 0x3F6; // reflected
constexpr uint32_t kOffYawInput = 0x42C;      // unreflected: graph output YawInput
constexpr uint32_t kOffPitchInput = 0x430;    // unreflected: graph output PitchInput
constexpr uint32_t kOffSensX = 0x498;         // reflected
constexpr uint32_t kOffUnlockBlend = 0x4B8;   // unreflected: unlock blend (the apply skips the update while > 0)
// entEntity / entIPlacedComponent
constexpr uint32_t kOffTransformComp = 0xB0; // entEntity transformComponent (SDK entEntity.hpp)
constexpr uint32_t kOffWorldRot = 0xF0;      // IPlacedComponent world orientation (FUN_140520f84)
// animAnimFeature_FPPCamera
constexpr uint32_t kFeatSize = 0x100;
constexpr uint32_t kOffFeatDeltaYaw = 0x44;
constexpr uint32_t kOffFeatDeltaPitch = 0x60;
constexpr uint32_t kOffFeatResetYaw = 0x78;   // resetYawSpeed (written by FUN_1404e1638 before the apply)
constexpr uint32_t kOffFeatResetPitch = 0x7C; // resetPitchSpeed
constexpr uint32_t kOffFeatYawMult = 0xC4;
constexpr uint32_t kOffFeatPitchMult = 0xC8;
constexpr uint32_t kOffFeatOverride = 0xDC;   // override (weight)

constexpr int64_t kTtlNs = 250'000'000; // a target (and a pending ahead) older than this is not written
constexpr float kEntityYawTolerance = 1.0f;

using Apply_t = void (*)(void*, float, float, float, float, float);
using LookVars_t = bool (*)(void*, float*); // round 3: graph look read (comp, out[4])

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;
Apply_t g_origApply = nullptr;
// round 3: RotateEntityToCamera's look read (GameLookCore.hpp kRvaLookVars / kRvaRotateReadRet)
LookVars_t g_origLookVars = nullptr;
void* g_targetLookVars = nullptr;
uintptr_t g_rotRet = 0; // absolute return address of the read call inside RotateEntityToCamera
std::atomic<bool> g_rotHook{false};
std::atomic<uint64_t> g_rotCalls{0}, g_rotZeroed{0}, g_rotPassed{0}, g_rotNotValue{0};
std::atomic<float> g_rotLastYaw{0}, g_rotLastEntYaw{0};
std::atomic<int64_t> g_rotLastNs{0};
std::atomic<bool> g_rotLastZeroed{false};
void* g_target = nullptr;

std::atomic<bool> g_installed{false};
std::atomic<void*> g_comp{nullptr};
std::atomic<void*> g_owner{nullptr};
std::atomic<bool> g_active{false};
std::atomic<float> g_targetYaw{0.0f};
std::atomic<float> g_targetPitch{0.0f};
std::atomic<float> g_ahead{0.0f}; // 0.6.5b: entity yaw turn expected in the next tick (consumed by a write; round 2: summed)
std::atomic<int64_t> g_aheadStampNs{0}; // round 2: time of the last non-zero ahead (a pending one older than kTtlNs is dropped)
std::atomic<int64_t> g_stampNs{0};
std::atomic<bool> g_resetReq{false};
std::atomic<bool> g_rearmReq{false}; // round 2: SetTarget re-armed the write after a trip (Controller::Rearm on the next write)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
Controller g_ctl; // only inside g_busy

// counters / last values (relaxed)
std::atomic<uint64_t> g_calls{0}, g_callsComp{0}, g_writes{0}, g_stale{0}, g_busyCount{0}, g_faults{0}, g_ownerMismatch{0},
    g_skipNotLocked{0}, g_skipSentinel{0}, g_skipNonFinite{0}, g_skipTripped{0}, g_extZeroed{0}, g_featFaults{0},
    g_aheadUsed{0}, g_aheadN{0}, g_aheadDiscN{0}, g_aheadStale{0}, g_ovrTicks{0}, g_releases{0}, g_featYawRepl{0},
    g_featPitchRepl{0}, g_featReplOvr{0}, g_resetTicks{0}, g_overrideTicks{0};
std::atomic<float> g_lastEntYaw{0}, g_lastYawIn{0}, g_lastPitchIn{0}, g_lastYawDelta{0}, g_lastPitchDelta{0},
    g_lastDesiredYaw{0}, g_lastDesiredPitch{0}, g_lastFeatYaw{0}, g_lastFeatPitch{0}, g_lastYawMult{1}, g_lastPitchMult{1},
    g_maxAbsYawDelta{0}, g_lastErrYaw{0}, g_lastErrPitch{0}, g_lastRawErrYaw{0}, g_lastRawErrPitch{0}, g_lastFfYaw{0},
    g_lastFfPitch{0}, g_lastAhead{0}, g_lastAheadEff{0}, g_maxAbsErr{0}, g_lastRelease{0}, g_maxRelease{0},
    g_lastResetYaw{0}, g_lastResetPitch{0}, g_lastOverride{0}, g_maxFeatYawDiff{0}, g_maxFeatPitchDiff{0},
    g_yawMaxLeft{0}, g_yawMaxRight{0};
std::atomic<double> g_aheadAbs{0}, g_aheadNoFeed{0}, g_resAfter{0}, g_resBefore{0}, g_resLate{0};
std::atomic<int> g_lag{0}; // diagnostic hint only (0.6.5b: never used for the command)
std::atomic<int> g_aheadOrder{0};
std::atomic<uint32_t> g_lagSwitches{0}, g_aheadOrderSwitches{0};
std::atomic<bool> g_tripped{false};
std::atomic<float> g_tripErr{0}, g_tripFrom{0}, g_tripChurn{0};
std::atomic<int> g_tripKind{0}, g_tripCount{0};
std::atomic<int64_t> g_tripAtNs{0};

// 2026-10-28 vehicle edge zeroing (GameLookCore.hpp EdgeZero). Published by OnFeed (game thread): the edge values, then
// the sequence, then the arm time (0 = no window). Hook state only inside g_busy.
std::atomic<int64_t> g_edgeArmNs{0};
std::atomic<int64_t> g_exitArmNs{0}; // round 2: PSM exit start in third person (0 = no exit phase)
std::atomic<uint64_t> g_exitExpired{0};
std::atomic<uint32_t> g_edgeSeq{0};
std::atomic<float> g_edgeEntYaw0{0};
EdgeZero g_edge;            // only inside g_busy
uint32_t g_edgeSeenSeq = 0; // only inside g_busy
bool g_edgeEnded = false;   // only inside g_busy
// window results (written by the hook, read by the report on the game thread)
std::atomic<int> g_edgeTicks{0}, g_edgeWrites{0}, g_edgeCorr{0}, g_edgeLocked{0}, g_edgeSentinel{0}, g_edgeNonFinite{0},
    g_edgeBlend{0}, g_edgeFeatRepl{0}, g_edgeOwnerMiss{0}, g_edgeBusy{0};
std::atomic<float> g_edgeFirstYawIn{0}, g_edgeLastYawIn{0}, g_edgeLastEntYaw{0}, g_edgeMaxDrift{0}, g_edgeFirstMs{-1},
    g_edgeMaxDelta{0};
std::atomic<uint64_t> g_edgeArms{0}, g_edgeWritesTotal{0};

// cold (game thread)
std::mutex g_coldMutex;
std::string g_installWhy = "not installed (Install not called)";
std::string g_lastWhy = "-";        // last SetTarget failure (or "ok")
uint64_t g_sets = 0, g_setFails = 0, g_stops = 0, g_onLogs = 0, g_aheadSets = 0, g_aheadBad = 0, g_lagLogs = 0,
         g_rearms = 0;
bool g_rttiChecked = false;
bool g_rttiOk = false;
std::string g_rttiWhy;
RED4ext::CClass* g_fppCls = nullptr;
uint64_t g_featHash = 0;
float g_lastEntYawCheck = 0.0f; // |raw - RTTI| of the last SetTarget
int g_loggedLag = 0;
int g_loggedOrder = 0;
bool g_loggedTrip = false;
VehicleEdge g_vehEdge;        // feed edge detection
uint64_t g_edgeCallsAtArm = 0; // g_callsComp at the arm (hook calls for the component during the window)
uint64_t g_edgeNoTpp = 0, g_edgeCancels = 0, g_edgeArmFails = 0, g_edgeLogs = 0;
std::string g_edgeLast = "-"; // last edge result (status)
uint64_t g_exitStarts = 0, g_exitNoTpp = 0;       // round 2: exit starts seen / not in third person
float g_exitEntYaw0 = 0.0f, g_exitYawIn0 = 0.0f; // at the exit start

using pluginlog::LogInfo;
using pluginlog::LogWarn;

int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct CompRead
{
    float yawIn, pitchIn, pitchMin, pitchMax, yawMaxLeft, yawMaxRight, unlockBlend;
    bool locked;
    void* owner;
    float q[4];
    bool hasRot;
};

// No C++ objects with destructors in the SEH helpers.
bool SehReadComp(const uint8_t* aComp, CompRead* aOut)
{
    __try
    {
        aOut->yawIn = *reinterpret_cast<const float*>(aComp + kOffYawInput);
        aOut->pitchIn = *reinterpret_cast<const float*>(aComp + kOffPitchInput);
        aOut->pitchMin = *reinterpret_cast<const float*>(aComp + kOffPitchMin);
        aOut->pitchMax = *reinterpret_cast<const float*>(aComp + kOffPitchMax);
        aOut->yawMaxLeft = *reinterpret_cast<const float*>(aComp + kOffYawMaxLeft);
        aOut->yawMaxRight = *reinterpret_cast<const float*>(aComp + kOffYawMaxRight);
        aOut->unlockBlend = *reinterpret_cast<const float*>(aComp + kOffUnlockBlend);
        aOut->locked = *reinterpret_cast<const uint8_t*>(aComp + kOffHeadingLocked) != 0;
        aOut->owner = *reinterpret_cast<void* const*>(aComp + kOffOwner);
        aOut->hasRot = false;
        if (aOut->owner)
        {
            const uint8_t* tc = *reinterpret_cast<const uint8_t* const*>(static_cast<const uint8_t*>(aOut->owner) +
                                                                         kOffTransformComp);
            if (tc)
            {
                std::memcpy(aOut->q, tc + kOffWorldRot, sizeof(aOut->q));
                aOut->hasRot = true;
            }
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

struct FeatRead
{
    float yaw, pitch, yawMult, pitchMult, resetYaw, resetPitch, ovr;
};

bool SehReadFeature(const uint8_t* aComp, FeatRead* aOut)
{
    __try
    {
        const uint8_t* feat = *reinterpret_cast<const uint8_t* const*>(aComp + kOffFeature);
        if (!feat)
            return false;
        aOut->yaw = *reinterpret_cast<const float*>(feat + kOffFeatDeltaYaw);
        aOut->pitch = *reinterpret_cast<const float*>(feat + kOffFeatDeltaPitch);
        aOut->yawMult = *reinterpret_cast<const float*>(feat + kOffFeatYawMult);
        aOut->pitchMult = *reinterpret_cast<const float*>(feat + kOffFeatPitchMult);
        aOut->resetYaw = *reinterpret_cast<const float*>(feat + kOffFeatResetYaw);
        aOut->resetPitch = *reinterpret_cast<const float*>(feat + kOffFeatResetPitch);
        aOut->ovr = *reinterpret_cast<const float*>(feat + kOffFeatOverride);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// 0 ok, 1 owner != player, 2 no feature, 3 feature type, 4 no graph source, 5 no entity transform, -1 fault
int SehCheckComp(uint8_t* aComp, void* aPlayer, uint64_t aFeatHash, float* aRawYaw)
{
    __try
    {
        if (*reinterpret_cast<void**>(aComp + kOffOwner) != aPlayer)
            return 1;
        auto* feat = *reinterpret_cast<RED4ext::IScriptable**>(aComp + kOffFeature);
        if (!feat)
            return 2;
        RED4ext::CClass* t = feat->GetType();
        if (!t || t->name.hash != aFeatHash)
            return 3;
        if (!*reinterpret_cast<void**>(aComp + kOffVarSource))
            return 4;
        const uint8_t* tc = *reinterpret_cast<const uint8_t* const*>(static_cast<const uint8_t*>(aPlayer) + kOffTransformComp);
        if (!tc)
            return 5;
        float q[4];
        std::memcpy(q, tc + kOffWorldRot, sizeof(q));
        *aRawYaw = YawFromQuat(q[0], q[1], q[2], q[3]);
        return 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}

using util::HexAt;
using util::SehBytesEqual;
using util::SehCopy;

void StoreMax(std::atomic<float>& aMax, float aV)
{
    float cur = aMax.load(std::memory_order_relaxed);
    while (aV > cur && !aMax.compare_exchange_weak(cur, aV, std::memory_order_relaxed))
    {
    }
}

void DetourApply(void* aComp, float aDt, float aYaw, float aPitch, float aExtYaw, float aExtPitch)
{
    g_calls.fetch_add(1, std::memory_order_relaxed);
    bool wrote = false;
    bool locked = false; // g_busy held (round 2: until the feature after the game's apply went to the controller)
    bool wroteOvr = false;
    float wroteYaw = 0.0f, wrotePitch = 0.0f;
    void* comp = g_comp.load(std::memory_order_acquire);
    if (comp && aComp == comp)
    {
        g_callsComp.fetch_add(1, std::memory_order_relaxed);
        if (g_active.load(std::memory_order_acquire))
        {
            const int64_t now = NowNs();
            if (now - g_stampNs.load(std::memory_order_relaxed) > kTtlNs)
            {
                g_stale.fetch_add(1, std::memory_order_relaxed);
                g_resetReq.store(true, std::memory_order_relaxed);
                g_ahead.store(0.0f, std::memory_order_relaxed);
            }
            else if (!g_busy.test_and_set(std::memory_order_acquire))
            {
                locked = true;
                if (g_rearmReq.exchange(false, std::memory_order_relaxed))
                    g_ctl.Rearm(); // round 2: re-armed by SetTarget after the trip cooldown (history reset too)
                if (g_resetReq.exchange(false, std::memory_order_relaxed))
                    g_ctl.ResetHistory();
                CompRead r{};
                if (!SehReadComp(static_cast<const uint8_t*>(aComp), &r))
                    g_faults.fetch_add(1, std::memory_order_relaxed);
                else if (r.owner != g_owner.load(std::memory_order_relaxed) || !r.hasRot)
                    g_ownerMismatch.fetch_add(1, std::memory_order_relaxed);
                else
                {
                    Inputs in;
                    in.targetYaw = g_targetYaw.load(std::memory_order_relaxed);
                    in.targetPitch = g_targetPitch.load(std::memory_order_relaxed);
                    in.entityYaw = YawFromQuat(r.q[0], r.q[1], r.q[2], r.q[3]);
                    in.yawInput = r.yawIn;
                    in.pitchInput = r.pitchIn;
                    in.pitchMin = r.pitchMin;
                    in.pitchMax = r.pitchMax;
                    in.headingLocked = r.locked;
                    // 0.6.5b: the entity turn requested for this tick (consumed; round 2: dropped when older than the
                    // target TTL - no write consumed it) and the tick time (divergence window)
                    in.aheadYaw = g_ahead.exchange(0.0f, std::memory_order_relaxed);
                    if (in.aheadYaw != 0.0f && now - g_aheadStampNs.load(std::memory_order_relaxed) > kTtlNs)
                    {
                        g_aheadStale.fetch_add(1, std::memory_order_relaxed);
                        in.aheadYaw = 0.0f;
                    }
                    in.dt = aDt;
                    // round 2: the rubber band factors of the last apply (compensated by the controller)
                    in.yawMult = g_lastYawMult.load(std::memory_order_relaxed);
                    in.pitchMult = g_lastPitchMult.load(std::memory_order_relaxed);
                    g_yawMaxLeft.store(r.yawMaxLeft, std::memory_order_relaxed);
                    g_yawMaxRight.store(r.yawMaxRight, std::memory_order_relaxed);
                    const Output o = g_ctl.Step(in);
                    g_lastEntYaw.store(in.entityYaw, std::memory_order_relaxed);
                    g_lastYawIn.store(r.yawIn, std::memory_order_relaxed);
                    g_lastPitchIn.store(r.pitchIn, std::memory_order_relaxed);
                    g_lag.store(g_ctl.lag, std::memory_order_relaxed);
                    g_lagSwitches.store(g_ctl.lagSwitches, std::memory_order_relaxed);
                    g_aheadN.store(g_ctl.aheadN, std::memory_order_relaxed);
                    g_aheadDiscN.store(g_ctl.aheadDiscN, std::memory_order_relaxed);
                    g_aheadAbs.store(g_ctl.aheadAbs, std::memory_order_relaxed);
                    g_aheadNoFeed.store(g_ctl.aheadNoFeed, std::memory_order_relaxed);
                    g_resAfter.store(g_ctl.resAfter, std::memory_order_relaxed);
                    g_resBefore.store(g_ctl.resBefore, std::memory_order_relaxed);
                    g_resLate.store(g_ctl.resLate, std::memory_order_relaxed);
                    g_aheadOrder.store(static_cast<int>(g_ctl.aheadOrder), std::memory_order_relaxed);
                    g_aheadOrderSwitches.store(g_ctl.aheadOrderSwitches, std::memory_order_relaxed);
                    g_ovrTicks.store(g_ctl.ovrTicks, std::memory_order_relaxed);
                    g_releases.store(g_ctl.releases, std::memory_order_relaxed);
                    g_lastRelease.store(g_ctl.lastRelease, std::memory_order_relaxed);
                    g_maxRelease.store(g_ctl.maxRelease, std::memory_order_relaxed);
                    StoreMax(g_maxAbsErr, g_ctl.TakeMaxErr());
                    if (o.trippedNow)
                    {
                        g_tripErr.store(g_ctl.tripErr, std::memory_order_relaxed);
                        g_tripFrom.store(g_ctl.tripFrom, std::memory_order_relaxed);
                        g_tripChurn.store(g_ctl.tripChurn, std::memory_order_relaxed);
                        g_tripKind.store(g_ctl.tripKind, std::memory_order_relaxed);
                        g_tripCount.store(g_ctl.tripCount, std::memory_order_relaxed);
                        g_tripAtNs.store(now, std::memory_order_relaxed);
                        g_tripped.store(true, std::memory_order_relaxed);
                    }
                    if (o.write)
                    {
                        aYaw = o.yawDelta;
                        aPitch = o.pitchDelta;
                        if (aExtYaw != 0.0f || aExtPitch != 0.0f)
                            g_extZeroed.fetch_add(1, std::memory_order_relaxed);
                        aExtYaw = 0.0f;
                        aExtPitch = 0.0f;
                        g_lastYawDelta.store(o.yawDelta, std::memory_order_relaxed);
                        g_lastPitchDelta.store(o.pitchDelta, std::memory_order_relaxed);
                        g_lastDesiredYaw.store(o.desiredYaw, std::memory_order_relaxed);
                        g_lastDesiredPitch.store(o.desiredPitch, std::memory_order_relaxed);
                        g_lastErrYaw.store(o.errYaw, std::memory_order_relaxed);
                        g_lastErrPitch.store(o.errPitch, std::memory_order_relaxed);
                        g_lastRawErrYaw.store(o.rawErrYaw, std::memory_order_relaxed);
                        g_lastRawErrPitch.store(o.rawErrPitch, std::memory_order_relaxed);
                        g_lastFfYaw.store(o.ffYaw, std::memory_order_relaxed);
                        g_lastFfPitch.store(o.ffPitch, std::memory_order_relaxed);
                        g_lastAhead.store(in.aheadYaw, std::memory_order_relaxed);
                        g_lastAheadEff.store(o.aheadEff, std::memory_order_relaxed);
                        if (in.aheadYaw != 0.0f)
                            g_aheadUsed.fetch_add(1, std::memory_order_relaxed);
                        StoreMax(g_maxAbsYawDelta, std::fabs(o.yawDelta));
                        g_writes.fetch_add(1, std::memory_order_relaxed);
                        wrote = true;
                        wroteOvr = o.ovr;
                        wroteYaw = o.yawDelta;
                        wrotePitch = o.pitchDelta;
                    }
                    else
                    {
                        switch (o.skip)
                        {
                        case Skip::NotLocked:
                            g_skipNotLocked.fetch_add(1, std::memory_order_relaxed);
                            break;
                        case Skip::Sentinel:
                            g_skipSentinel.fetch_add(1, std::memory_order_relaxed);
                            break;
                        case Skip::NonFinite:
                            g_skipNonFinite.fetch_add(1, std::memory_order_relaxed);
                            break;
                        default:
                            g_skipTripped.fetch_add(1, std::memory_order_relaxed);
                            break;
                        }
                    }
                }
            }
            else
                g_busyCount.fetch_add(1, std::memory_order_relaxed);
        }
    }
    // 2026-10-28 vehicle edge zeroing: from the PSM exit start (VehicleExitStart, third person) through the exit and for
    // kEdgeZeroSeconds after the vehicle edge the yaw delta drives YawInput to 0 (no write when the normal write already
    // wrote this tick), so RotateEntityToCamera at the unmount finds no look offset; the entity yaw drift is tracked from
    // the edge until the report (kEdgeReportSeconds, OnFeed clears the edge arm time). Pitch and the external pitch pass
    // unchanged.
    bool edgeWrote = false;
    float edgeYaw = 0.0f;
    const int64_t edgeArm = g_edgeArmNs.load(std::memory_order_acquire);
    int64_t exitArm = g_exitArmNs.load(std::memory_order_acquire);
    if ((edgeArm != 0 || exitArm != 0) && comp && aComp == comp)
    {
        if (!locked)
        {
            if (!g_busy.test_and_set(std::memory_order_acquire))
                locked = true;
            else
                g_edgeBusy.fetch_add(1, std::memory_order_relaxed);
        }
        if (locked)
        {
            const uint32_t seq = g_edgeSeq.load(std::memory_order_acquire);
            if (seq != g_edgeSeenSeq)
            {
                g_edgeSeenSeq = seq;
                g_edge.Reset();
                g_edgeEnded = false;
            }
            const int64_t now = NowNs();
            const double exitAge = exitArm != 0 ? static_cast<double>(now - exitArm) * 1e-9 : -1.0;
            const double edgeAge = edgeArm != 0 ? static_cast<double>(now - edgeArm) * 1e-9 : -1.0;
            const bool inWindow = EdgeZeroWanted(exitAge, edgeAge);
            if (exitArm != 0 && exitAge > static_cast<double>(kExitMaxSeconds) &&
                g_exitArmNs.compare_exchange_strong(exitArm, 0, std::memory_order_acq_rel))
                g_exitExpired.fetch_add(1, std::memory_order_relaxed); // an exit that never reached the edge
            const int64_t age = now - (edgeArm != 0 ? edgeArm : exitArm); // since the arm of this window
            CompRead r{};
            if (!SehReadComp(static_cast<const uint8_t*>(aComp), &r) ||
                r.owner != g_owner.load(std::memory_order_relaxed) || !r.hasRot)
                g_edgeOwnerMiss.fetch_add(1, std::memory_order_relaxed);
            else
            {
                const float entYaw = YawFromQuat(r.q[0], r.q[1], r.q[2], r.q[3]);
                if (edgeArm != 0)
                {
                    const float drift = std::fabs(Wrap180(entYaw - g_edgeEntYaw0.load(std::memory_order_relaxed)));
                    if (Finite(drift))
                        StoreMax(g_edgeMaxDrift, drift);
                }
                g_edgeLastEntYaw.store(entYaw, std::memory_order_relaxed);
                if (inWindow && !wrote)
                {
                    if (r.unlockBlend > 0.0f)
                        g_edgeBlend.fetch_add(1, std::memory_order_relaxed);
                    const EdgeZeroOut o = g_edge.Step(r.yawIn, r.locked, g_lastYawMult.load(std::memory_order_relaxed));
                    if (o.write)
                    {
                        aYaw = o.yawDelta;
                        aExtYaw = 0.0f;
                        edgeWrote = true;
                        edgeYaw = o.yawDelta;
                        g_edgeWritesTotal.fetch_add(1, std::memory_order_relaxed);
                        StoreMax(g_edgeMaxDelta, std::fabs(o.yawDelta));
                        if (g_edge.writes == 1)
                            g_edgeFirstMs.store(static_cast<float>(static_cast<double>(age) * 1e-6), std::memory_order_relaxed);
                    }
                    g_edgeTicks.store(g_edge.ticks, std::memory_order_relaxed);
                    g_edgeWrites.store(g_edge.writes, std::memory_order_relaxed);
                    g_edgeCorr.store(g_edge.corrections, std::memory_order_relaxed);
                    g_edgeLocked.store(g_edge.locked, std::memory_order_relaxed);
                    g_edgeSentinel.store(g_edge.sentinel, std::memory_order_relaxed);
                    g_edgeNonFinite.store(g_edge.nonFinite, std::memory_order_relaxed);
                    g_edgeFirstYawIn.store(g_edge.firstYawIn, std::memory_order_relaxed);
                    g_edgeLastYawIn.store(r.yawIn, std::memory_order_relaxed);
                }
            }
            if (!inWindow && !g_edgeEnded && g_edge.writes > 0)
            {
                // hand back after a window that wrote (not before the exit phase delay): the normal write (if any)
                // starts again as a first write
                g_edgeEnded = true;
                g_resetReq.store(true, std::memory_order_relaxed);
            }
        }
    }
    g_origApply(aComp, aDt, aYaw, aPitch, aExtYaw, aExtPitch);
    if (edgeWrote)
    {
        // the game's rubber band factor for the next correction; did the apply replace the yaw delta (diagnostic)
        FeatRead f{0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f};
        if (SehReadFeature(static_cast<const uint8_t*>(aComp), &f))
        {
            if (Finite(f.yawMult) && f.yawMult > 0.0f && f.yawMult <= 2.0f)
                g_lastYawMult.store(f.yawMult, std::memory_order_relaxed);
            const float dy = std::fabs(f.yaw - edgeYaw);
            if (Finite(dy) && dy > 1e-3f)
                g_edgeFeatRepl.fetch_add(1, std::memory_order_relaxed);
        }
    }
    if (wrote)
    {
        FeatRead f{0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f};
        if (SehReadFeature(static_cast<const uint8_t*>(aComp), &f))
        {
            g_lastFeatYaw.store(f.yaw, std::memory_order_relaxed);
            g_lastFeatPitch.store(f.pitch, std::memory_order_relaxed);
            // the multipliers feed the next step's compensation (sane values only)
            const bool ym = Finite(f.yawMult) && f.yawMult > 0.0f && f.yawMult <= 2.0f;
            const bool pm = Finite(f.pitchMult) && f.pitchMult > 0.0f && f.pitchMult <= 2.0f;
            g_lastYawMult.store(ym ? f.yawMult : 1.0f, std::memory_order_relaxed);
            g_lastPitchMult.store(pm ? f.pitchMult : 1.0f, std::memory_order_relaxed);
            // round 2: what the graph integrates goes to the controller's prediction (override ticks)
            g_ctl.NoteApplied(f.yaw * (ym ? f.yawMult : 1.0f), f.pitch * (pm ? f.pitchMult : 1.0f));
            // round 2 diagnostics: the game's apply changed our delta (rubber band return / yaw clamp), reset speeds and
            // the override weight of this tick
            const float dy = std::fabs(f.yaw - wroteYaw), dp = std::fabs(f.pitch - wrotePitch);
            if (Finite(dy) && dy > 1e-3f)
            {
                g_featYawRepl.fetch_add(1, std::memory_order_relaxed);
                StoreMax(g_maxFeatYawDiff, dy);
            }
            if (Finite(dp) && dp > 1e-3f)
            {
                g_featPitchRepl.fetch_add(1, std::memory_order_relaxed);
                StoreMax(g_maxFeatPitchDiff, dp);
            }
            if (wroteOvr && ((Finite(dy) && dy > 1e-3f) || (Finite(dp) && dp > 1e-3f)))
                g_featReplOvr.fetch_add(1, std::memory_order_relaxed);
            if (f.resetYaw != 0.0f || f.resetPitch != 0.0f)
            {
                g_resetTicks.fetch_add(1, std::memory_order_relaxed);
                g_lastResetYaw.store(f.resetYaw, std::memory_order_relaxed);
                g_lastResetPitch.store(f.resetPitch, std::memory_order_relaxed);
            }
            if (f.ovr != 0.0f)
            {
                g_overrideTicks.fetch_add(1, std::memory_order_relaxed);
                g_lastOverride.store(f.ovr, std::memory_order_relaxed);
            }
        }
        else
            g_featFaults.fetch_add(1, std::memory_order_relaxed);
    }
    if (locked)
        g_busy.clear(std::memory_order_release);
}

// Round 3: the graph look read (2.50 @0x5FD84C, called every camera tick and by RotateEntityToCamera). Only the call from
// RotateEntityToCamera (return address kRvaRotateReadRet) is looked at: inside RotateZeroGate the YawInput it reads is
// replaced by 0, so the game does not turn V onto its look at the unmount and its unlock blend starts at V's heading.
// Hot path: one compare for every other caller.
__declspec(noinline) bool DetourLookVars(void* aComp, float* aOut)
{
    const bool r = g_origLookVars(aComp, aOut);
    if (reinterpret_cast<uintptr_t>(_ReturnAddress()) != g_rotRet || !aOut)
        return r;
    g_rotCalls.fetch_add(1, std::memory_order_relaxed);
    const int64_t now = NowNs();
    const int64_t exitArm = g_exitArmNs.load(std::memory_order_acquire);
    const int64_t edgeArm = g_edgeArmNs.load(std::memory_order_acquire);
    const double exitAge = exitArm != 0 ? static_cast<double>(now - exitArm) * 1e-9 : -1.0;
    const double edgeAge = edgeArm != 0 ? static_cast<double>(now - edgeArm) * 1e-9 : -1.0;
    const bool playerComp = aComp != nullptr && aComp == g_comp.load(std::memory_order_acquire);
    const float yaw = aOut[kLookVarsYawOffset / sizeof(float)];
    CompRead c{};
    if (playerComp && SehReadComp(static_cast<const uint8_t*>(aComp), &c) && c.hasRot)
        g_rotLastEntYaw.store(YawFromQuat(c.q[0], c.q[1], c.q[2], c.q[3]), std::memory_order_relaxed);
    g_rotLastYaw.store(yaw, std::memory_order_relaxed);
    g_rotLastNs.store(now, std::memory_order_relaxed);
    bool zeroed = false;
    if (!RotateZeroGate(playerComp, exitAge, edgeAge))
        g_rotPassed.fetch_add(1, std::memory_order_relaxed);
    else if (!RotateYawZeroable(yaw))
        g_rotNotValue.fetch_add(1, std::memory_order_relaxed);
    else
    {
        aOut[kLookVarsYawOffset / sizeof(float)] = 0.0f;
        g_rotZeroed.fetch_add(1, std::memory_order_relaxed);
        zeroed = true;
    }
    g_rotLastZeroed.store(zeroed, std::memory_order_relaxed);
    return r;
}

// Game thread, g_coldMutex held.
bool CheckRttiLocked(std::string& aWhy)
{
    if (g_rttiChecked)
    {
        aWhy = g_rttiWhy;
        return g_rttiOk;
    }
    g_rttiChecked = true;
    g_rttiOk = false;
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* cls = rtti ? rtti->GetClass("gameFPPCameraComponent") : nullptr;
    RED4ext::CClass* feat = rtti ? rtti->GetClass("animAnimFeature_FPPCamera") : nullptr;
    char buf[160];
    if (!cls || !feat)
    {
        g_rttiWhy = "RTTI class gameFPPCameraComponent / animAnimFeature_FPPCamera not found";
    }
    else if (cls->GetSize() != kCompSize || feat->GetSize() != kFeatSize)
    {
        std::snprintf(buf, sizeof(buf), "RTTI size: gameFPPCameraComponent 0x%X (want 0x%X), feature 0x%X (want 0x%X)",
                      cls->GetSize(), kCompSize, feat->GetSize(), kFeatSize);
        g_rttiWhy = buf;
    }
    else
    {
        struct P
        {
            RED4ext::CClass* cls;
            const char* name;
            uint32_t off;
        };
        const P props[] = {{cls, "headingLocked", kOffHeadingLocked},     {cls, "pitchMin", kOffPitchMin},
                           {cls, "pitchMax", kOffPitchMax},               {cls, "yawMaxLeft", kOffYawMaxLeft},
                           {cls, "yawMaxRight", kOffYawMaxRight},         {cls, "sensitivityMultX", kOffSensX},
                           {feat, "deltaYaw", kOffFeatDeltaYaw},          {feat, "deltaPitch", kOffFeatDeltaPitch},
                           {feat, "yawMultiplier", kOffFeatYawMult},      {feat, "pitchMultiplier", kOffFeatPitchMult},
                           {feat, "resetYawSpeed", kOffFeatResetYaw},     {feat, "resetPitchSpeed", kOffFeatResetPitch},
                           {feat, "override", kOffFeatOverride}};
        g_rttiWhy.clear();
        for (const P& p : props)
        {
            RED4ext::CProperty* prop = p.cls->GetProperty(p.name);
            if (!prop || prop->valueOffset != p.off)
            {
                std::snprintf(buf, sizeof(buf), "RTTI property %s: %s0x%X (want 0x%X)", p.name, prop ? "" : "missing ",
                              prop ? prop->valueOffset : 0u, p.off);
                g_rttiWhy = buf;
                break;
            }
        }
        if (g_rttiWhy.empty())
        {
            g_rttiOk = true;
            g_rttiWhy = "ok";
            g_fppCls = cls;
            g_featHash = feat->name.hash;
        }
    }
    aWhy = g_rttiWhy;
    return g_rttiOk;
}

// Game thread, g_coldMutex held. Resolves and validates the player's FPP camera component; returns it or nullptr.
void* ResolveLocked(void*& aPlayerOut, std::string& aWhy)
{
    aPlayerOut = nullptr;
    auto* player = static_cast<RED4ext::IScriptable*>(GetPlayerEntityRaw());
    if (!player)
    {
        aWhy = "no player";
        return nullptr;
    }
    RED4ext::CClass* pcls = player->GetType();
    RED4ext::CClassFunction* getFpp = pcls ? pcls->GetFunction("GetFPPCameraComponent") : nullptr;
    RED4ext::CClassFunction* getRot = pcls ? pcls->GetFunction("GetWorldOrientation") : nullptr;
    if (!getFpp || !getRot)
    {
        aWhy = "player class without GetFPPCameraComponent / GetWorldOrientation";
        return nullptr;
    }
    RED4ext::Handle<RED4ext::IScriptable> h;
    if (!RED4ext::ExecuteFunction(player, getFpp, &h) || !h)
    {
        aWhy = "GetFPPCameraComponent returned nothing";
        return nullptr;
    }
    RED4ext::CClass* ccls = h->GetType();
    if (!ccls || !ccls->IsA(g_fppCls))
    {
        aWhy = "GetFPPCameraComponent: not a gameFPPCameraComponent";
        return nullptr;
    }
    auto* comp = reinterpret_cast<uint8_t*>(h.instance);
    float rawYaw = 0.0f;
    const int rc = SehCheckComp(comp, player, g_featHash, &rawYaw);
    if (rc != 0)
    {
        static const char* kWhy[] = {"ok", "component owner +0x50 is not the player", "no AnimFeature_FPPCamera at +0x360",
                                     "+0x360 is not an animAnimFeature_FPPCamera", "no graph variable source at +0x380",
                                     "player without transform component (+0xB0)"};
        aWhy = rc < 0 ? std::string("fault while reading the component") : std::string(kWhy[rc]);
        return nullptr;
    }
    RED4ext::Quaternion q{};
    if (!RED4ext::ExecuteFunction(player, getRot, &q))
    {
        aWhy = "GetWorldOrientation failed";
        return nullptr;
    }
    const float rttiYaw = YawFromQuat(q.i, q.j, q.k, q.r);
    g_lastEntYawCheck = std::fabs(Wrap180(rawYaw - rttiYaw));
    if (!Finite(g_lastEntYawCheck) || g_lastEntYawCheck > kEntityYawTolerance)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "entity yaw +0xB0/+0xF0 %.2f != GetWorldOrientation %.2f", static_cast<double>(rawYaw),
                      static_cast<double>(rttiYaw));
        aWhy = buf;
        return nullptr;
    }
    aPlayerOut = player;
    return comp;
}

uint64_t g_failLogs = 0;
void NoteWhyLocked(const std::string& aWhy, bool aFail)
{
    if (aWhy == g_lastWhy)
        return;
    g_lastWhy = aWhy;
    // once per change of the reason (the first 30 changes, then every 500th)
    if (aFail && (g_failLogs++ < 30 || (g_failLogs % 500) == 0))
        LogWarn("UTPP gamelook: write not possible: %s (no write)", aWhy.c_str());
}
} // namespace

void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    std::lock_guard lock(g_coldMutex);
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !g_sdk || !g_sdk->hooking)
    {
        g_installWhy = "no image base / hooking API";
        LogWarn("UTPP gamelook: hook not installed (%s); game look write inert", g_installWhy.c_str());
        return;
    }
    struct Site
    {
        uint32_t rva;
        const uint8_t* bytes;
        size_t len;
        const char* what;
    };
    const Site sites[] = {
        {kRvaApply, kApplyPrologue, sizeof(kApplyPrologue), "apply prologue @0x39CD5C"},
        {kRvaCallSite, kCallSite, sizeof(kCallSite), "apply call site @0x39CBD7"},
        {kRvaRefresh, kRefreshSite, sizeof(kRefreshSite), "YawInput/PitchInput refresh @0x39CAD2"},
        {kRvaReadVars, kReadVarsPrologue, sizeof(kReadVarsPrologue), "graph variable read @0x5FD7CC"},
        {kRvaEntityRot, kEntityRotHead, sizeof(kEntityRotHead), "entity orientation @0x7C0C28"},
        {kRvaEntityRot + 0x12, kEntityRotRead, sizeof(kEntityRotRead), "entity orientation read @0x7C0C3A"},
    };
    for (const Site& s : sites)
    {
        if (!SehBytesEqual(base + s.rva, s.bytes, s.len))
        {
            g_installWhy = std::string(s.what) + " mismatch [" + HexAt(base + s.rva, s.len) + "]";
            LogWarn("UTPP gamelook: hook not installed (%s); game look write inert", g_installWhy.c_str());
            return;
        }
    }
    uint8_t* target = base + kRvaApply;
    if (!g_sdk->hooking->Attach(g_handle, target, reinterpret_cast<void*>(&DetourApply),
                                reinterpret_cast<void**>(&g_origApply)) ||
        !g_origApply)
    {
        g_origApply = nullptr;
        g_installWhy = "hooking->Attach failed on @0x39CD5C";
        LogWarn("UTPP gamelook: %s; game look write inert", g_installWhy.c_str());
        return;
    }
    g_target = target;
    g_installed.store(true, std::memory_order_release);
    g_installWhy = "installed (FPP look delta apply @0x39CD5C)";
    LogInfo("UTPP gamelook: hook attached (FPP look delta apply @%p); no write until UTPPSetGameLook",
            static_cast<void*>(target));
    // round 3: the look read of RotateEntityToCamera (fail closed: every site checked, the read call must target it)
    const Site rotSites[] = {
        {kRvaRotate, kRotateHead, sizeof(kRotateHead), "RotateEntityToCamera head @0x7E60E0"},
        {kRvaRotateReadSite, kRotateReadSite, sizeof(kRotateReadSite), "rotate look read call @0x7E6130"},
        {kRvaRotateUse, kRotateUse, sizeof(kRotateUse), "rotate YawInput use @0x7E615E"},
        {kRvaRotateBlend, kRotateBlend, sizeof(kRotateBlend), "rotate unlock blend 0.3 @0x7E6315"},
        {kRvaLookVars, kLookVarsHead, sizeof(kLookVarsHead), "graph look read head @0x5FD84C"},
        {kRvaLookVarsYaw, kLookVarsYaw, sizeof(kLookVarsYaw), "graph look read YawInput -> out+8 @0x5FD8D2"},
    };
    std::string rotWhy;
    for (const Site& s : rotSites)
    {
        if (!SehBytesEqual(base + s.rva, s.bytes, s.len))
        {
            rotWhy = std::string(s.what) + " mismatch [" + HexAt(base + s.rva, s.len) + "]";
            break;
        }
    }
    if (rotWhy.empty())
    {
        int32_t rel = 0;
        const uint8_t* callAt = base + kRvaRotateReadSite + sizeof(kRotateReadSite) - 1; // E8
        if (!SehCopy(callAt + 1, reinterpret_cast<uint8_t*>(&rel), sizeof(rel)) ||
            callAt + 5 + rel != base + kRvaLookVars)
            rotWhy = "rotate look read call does not target @0x5FD84C";
    }
    if (rotWhy.empty())
    {
        g_rotRet = reinterpret_cast<uintptr_t>(base + kRvaRotateReadRet);
        uint8_t* rt = base + kRvaLookVars;
        if (g_sdk->hooking->Attach(g_handle, rt, reinterpret_cast<void*>(&DetourLookVars),
                                   reinterpret_cast<void**>(&g_origLookVars)) &&
            g_origLookVars)
        {
            g_targetLookVars = rt;
            g_rotHook.store(true, std::memory_order_release);
            LogInfo("UTPP gamelook: rotate-to-camera guard attached (look read @%p, caller RotateEntityToCamera @%p): the "
                    "unmount turn of V onto the look is zeroed in the third-person vehicle exit only",
                    static_cast<void*>(rt), static_cast<void*>(base + kRvaRotate));
        }
        else
        {
            g_origLookVars = nullptr;
            rotWhy = "hooking->Attach failed on @0x5FD84C";
        }
    }
    if (!rotWhy.empty())
        LogWarn("UTPP gamelook: rotate-to-camera guard not installed (%s); the unmount may turn V onto the look",
                rotWhy.c_str());
}

void Shutdown()
{
    std::lock_guard lock(g_coldMutex);
    g_active.store(false);
    g_comp.store(nullptr);
    if (g_sdk && g_sdk->hooking && g_targetLookVars)
        LogInfo("UTPP gamelook unload: rotate guard detach %s",
                g_sdk->hooking->Detach(g_handle, g_targetLookVars) ? "ok" : "FAILED");
    g_targetLookVars = nullptr;
    g_rotHook.store(false);
    if (g_sdk && g_sdk->hooking && g_target)
        LogInfo("UTPP gamelook unload: detach %s", g_sdk->hooking->Detach(g_handle, g_target) ? "ok" : "FAILED");
    g_target = nullptr;
    g_installed.store(false);
}

bool SetTarget(float aWorldYawDeg, float aPitchDeg, float aEntityYawAheadDeg)
{
    std::lock_guard lock(g_coldMutex);
    ++g_sets;
    std::string why;
    bool ok = false;
    bool blocked = false;
    if (!g_installed.load(std::memory_order_acquire))
    {
        why = "hook not installed: " + g_installWhy;
        blocked = true;
    }
    else if (g_tripped.load(std::memory_order_relaxed))
    {
        // round 2: off for kTripCooldown s after a trip, then re-armed (at most kMaxTrips per game session)
        const int count = g_tripCount.load(std::memory_order_relaxed);
        const float off = static_cast<float>(static_cast<double>(NowNs() - g_tripAtNs.load(std::memory_order_relaxed)) * 1e-9);
        if (RearmDue(off, count))
        {
            g_tripped.store(false, std::memory_order_relaxed);
            g_rearmReq.store(true, std::memory_order_relaxed);
            g_loggedTrip = false;
            ++g_rearms;
            LogInfo("UTPP gamelook: re-armed after a trip (%.1f s off, trip %d of %d)", static_cast<double>(off), count,
                    kMaxTrips);
        }
        else
        {
            const bool forever = count >= kMaxTrips;
            char buf[400];
            std::snprintf(buf, sizeof(buf), "tripped: the look loop diverged (%s: |error| %.1f -> %.1f deg, read-back beyond "
                          "the target %.0f deg in the last %.1f s, target < %.0f deg/s; trip %d of %d) -> %s",
                          g_tripKind.load() == 1 ? "the error grew in 2 windows" : "the read-back spun without converging",
                          static_cast<double>(g_tripFrom.load()), static_cast<double>(g_tripErr.load()),
                          static_cast<double>(g_tripChurn.load()), static_cast<double>(kDivergeWindow),
                          static_cast<double>(kDivergeSlowRate), count, kMaxTrips,
                          forever ? "off for this session" : "off, re-armed after the cooldown");
            why = buf;
            if (!g_loggedTrip)
            {
                g_loggedTrip = true;
                LogWarn("UTPP gamelook: %s (writes=%llu lagHint=%d cooldown=%.0f s)", why.c_str(),
                        static_cast<unsigned long long>(g_writes.load()), g_lag.load(), static_cast<double>(kTripCooldown));
            }
            blocked = true;
        }
    }
    if (!blocked && (!Finite(aWorldYawDeg) || !Finite(aPitchDeg)))
        why = "non-finite target";
    else if (!blocked && !CheckRttiLocked(why))
        why = "RTTI layout: " + why;
    else if (!blocked)
    {
        void* player = nullptr;
        void* comp = ResolveLocked(player, why);
        if (comp)
        {
            if (comp != g_comp.load(std::memory_order_relaxed) || player != g_owner.load(std::memory_order_relaxed))
            {
                g_owner.store(player, std::memory_order_relaxed);
                g_resetReq.store(true, std::memory_order_relaxed);
                g_comp.store(comp, std::memory_order_release);
            }
            // 0.6.5b: the entity turn expected in the coming tick (a bad value is ignored, never written). Round 2: added
            // to a pending one (consumed by the next write); a 0 never clears it - the camera's own call (ahead 0) may come
            // after utpp_bodyturn's call (its step) in the same CET frame
            float ahead = 0.0f;
            if (Finite(aEntityYawAheadDeg) && std::fabs(aEntityYawAheadDeg) <= 180.0f)
                ahead = aEntityYawAheadDeg;
            else
                ++g_aheadBad;
            const int64_t now = NowNs();
            if (ahead != 0.0f)
            {
                ++g_aheadSets;
                // a pending ahead older than the TTL was never consumed: replaced, not added to (CAS: the hook may
                // consume it meanwhile)
                const bool old = now - g_aheadStampNs.load(std::memory_order_relaxed) > kTtlNs;
                float cur = g_ahead.load(std::memory_order_relaxed);
                while (!g_ahead.compare_exchange_weak(cur, ClampAbs((old ? 0.0f : cur) + ahead, 180.0f),
                                                      std::memory_order_relaxed))
                {
                }
                g_aheadStampNs.store(now, std::memory_order_relaxed);
            }
            g_targetYaw.store(Wrap180(aWorldYawDeg), std::memory_order_relaxed);
            g_targetPitch.store(aPitchDeg, std::memory_order_relaxed);
            g_stampNs.store(now, std::memory_order_relaxed);
            if (!g_active.exchange(true, std::memory_order_acq_rel))
            {
                g_resetReq.store(true, std::memory_order_relaxed);
                if (g_onLogs++ < 20 || (g_onLogs % 100) == 0)
                    LogInfo("UTPP gamelook: write ON (comp=%p yaw=%.1f pitch=%.1f ahead=%.2f entYawCheck=%.2f writes=%llu; "
                            "feed-forward + feedback k=%.2f, override ticks on the prediction + release, ahead order %s)",
                            comp, static_cast<double>(aWorldYawDeg), static_cast<double>(aPitchDeg),
                            static_cast<double>(ahead), static_cast<double>(g_lastEntYawCheck),
                            static_cast<unsigned long long>(g_writes.load()), static_cast<double>(kGain),
                            AheadOrderName(static_cast<AheadOrder>(g_aheadOrder.load())));
            }
            why = "ok";
            ok = true;
        }
    }
    // 0.6.5b: the lag is a diagnostic hint only (the controller does not use it); logged on change, rate limited
    const int lag = g_lag.load(std::memory_order_relaxed);
    if (lag != g_loggedLag)
    {
        g_loggedLag = lag;
        if (g_lagLogs++ < 20 || (g_lagLogs % 100) == 0)
            LogInfo("UTPP gamelook: graph lag hint -> %d tick(s) (switches=%u; diagnostic only, the command does not use it)",
                    lag, g_lagSwitches.load());
    }
    // round 2: the measured order of V's turn and the camera update (used for the ahead) - logged on change
    const int order = g_aheadOrder.load(std::memory_order_relaxed);
    if (order != g_loggedOrder)
    {
        g_loggedOrder = order;
        const uint64_t dn = g_aheadDiscN.load();
        const double div = dn ? static_cast<double>(dn) : 1.0;
        LogInfo("UTPP gamelook: ahead order -> %s (switches=%u, %llu samples: mean |turn - ahead| after %.3f before %.3f "
                "late %.3f deg)", AheadOrderName(static_cast<AheadOrder>(order)), g_aheadOrderSwitches.load(),
                static_cast<unsigned long long>(dn), g_resAfter.load() / div, g_resBefore.load() / div,
                g_resLate.load() / div);
    }
    if (!ok)
    {
        ++g_setFails;
        g_ahead.store(0.0f, std::memory_order_relaxed);
        if (g_active.exchange(false, std::memory_order_acq_rel))
            g_resetReq.store(true, std::memory_order_relaxed);
    }
    NoteWhyLocked(why, !ok);
    return ok;
}

bool Stop()
{
    std::lock_guard lock(g_coldMutex);
    const bool was = g_active.exchange(false, std::memory_order_acq_rel);
    g_ahead.store(0.0f, std::memory_order_relaxed);
    if (was)
    {
        ++g_stops;
        g_resetReq.store(true, std::memory_order_relaxed);
    }
    return was;
}

namespace
{
// Game thread, g_coldMutex held: the player's FPP camera component for the zeroing (g_comp / g_owner updated) and its
// read now; nullptr with aWhy on failure.
void* ResolveForZeroLocked(CompRead& aRead, std::string& aWhy)
{
    void* player = nullptr;
    void* comp = nullptr;
    if (!g_installed.load(std::memory_order_acquire))
        aWhy = "hook not installed: " + g_installWhy;
    else if (!CheckRttiLocked(aWhy))
        aWhy = "RTTI layout: " + aWhy;
    else
        comp = ResolveLocked(player, aWhy);
    if (comp && (!SehReadComp(static_cast<const uint8_t*>(comp), &aRead) || !aRead.hasRot))
    {
        aWhy = "component read failed";
        comp = nullptr;
    }
    if (comp && (comp != g_comp.load(std::memory_order_relaxed) || player != g_owner.load(std::memory_order_relaxed)))
    {
        g_owner.store(player, std::memory_order_relaxed);
        g_resetReq.store(true, std::memory_order_relaxed);
        g_comp.store(comp, std::memory_order_release);
    }
    return comp;
}

// Game thread, g_coldMutex held: window results reset before a new window is published (sequence bump).
void ResetWindowStatsLocked(float aEntYaw, float aYawIn)
{
    g_edgeEntYaw0.store(aEntYaw, std::memory_order_relaxed);
    g_edgeLastEntYaw.store(aEntYaw, std::memory_order_relaxed);
    g_edgeTicks.store(0, std::memory_order_relaxed);
    g_edgeWrites.store(0, std::memory_order_relaxed);
    g_edgeCorr.store(0, std::memory_order_relaxed);
    g_edgeLocked.store(0, std::memory_order_relaxed);
    g_edgeSentinel.store(0, std::memory_order_relaxed);
    g_edgeNonFinite.store(0, std::memory_order_relaxed);
    g_edgeBlend.store(0, std::memory_order_relaxed);
    g_edgeFeatRepl.store(0, std::memory_order_relaxed);
    g_edgeOwnerMiss.store(0, std::memory_order_relaxed);
    g_edgeBusy.store(0, std::memory_order_relaxed);
    g_edgeFirstYawIn.store(0.0f, std::memory_order_relaxed);
    g_edgeLastYawIn.store(aYawIn, std::memory_order_relaxed);
    g_edgeMaxDrift.store(0.0f, std::memory_order_relaxed);
    g_edgeMaxDelta.store(0.0f, std::memory_order_relaxed);
    g_edgeFirstMs.store(-1.0f, std::memory_order_relaxed);
    g_edgeCallsAtArm = g_callsComp.load(std::memory_order_relaxed);
}
} // namespace

bool VehicleExitStart(bool aUtppTpp, int aRideTpp)
{
    std::lock_guard lock(g_coldMutex);
    ++g_exitStarts;
    if (!ExitZeroAllowed(aUtppTpp, aRideTpp))
    {
        ++g_exitNoTpp;
        g_exitArmNs.store(0, std::memory_order_release);
        g_edgeLast = "exit start not in third person (no exit zeroing)";
        if (g_edgeLogs++ < 50)
            LogInfo("UTPP gamelook: vehicle exit start -> no look zeroing (utppTpp=%d rideCamera=%s)", aUtppTpp ? 1 : 0,
                    aRideTpp == 1 ? "tpp" : (aRideTpp == 0 ? "fpp" : "unknown"));
        return false;
    }
    std::string why;
    CompRead r{};
    if (!ResolveForZeroLocked(r, why))
    {
        ++g_edgeArmFails;
        g_exitArmNs.store(0, std::memory_order_release);
        g_edgeLast = "exit arm failed: " + why;
        if (g_edgeLogs++ < 50)
            LogWarn("UTPP gamelook: vehicle exit start -> look zeroing not possible: %s", why.c_str());
        return false;
    }
    const int64_t now = NowNs();
    const float entYaw = YawFromQuat(r.q[0], r.q[1], r.q[2], r.q[3]);
    ResetWindowStatsLocked(entYaw, r.yawIn);
    g_exitEntYaw0 = entYaw;
    g_exitYawIn0 = r.yawIn;
    g_edgeSeq.fetch_add(1, std::memory_order_acq_rel);
    g_exitArmNs.store(now, std::memory_order_release);
    g_edgeLast = "exit phase armed";
    if (g_edgeLogs++ < 50)
        LogInfo("UTPP gamelook: vehicle exit start -> look zeroing through the exit (entYaw=%.2f yawIn=%.2f lookYaw=%.2f "
                "headingLocked=%d unlockBlend=%.3f lookWrite=%d)",
                static_cast<double>(entYaw), static_cast<double>(r.yawIn), static_cast<double>(Wrap180(entYaw + r.yawIn)),
                r.locked ? 1 : 0, static_cast<double>(r.unlockBlend), g_active.load() ? 1 : 0);
    return true;
}

void OnFeed(bool aVehicleBit, bool aTpp)
{
    std::lock_guard lock(g_coldMutex);
    const EdgeEvent ev = g_vehEdge.Feed(aVehicleBit, aTpp);
    const int64_t now = NowNs();
    if (ev == EdgeEvent::Cancel)
    {
        const int64_t arm = g_edgeArmNs.load(std::memory_order_relaxed);
        if (arm != 0)
        {
            g_edgeArmNs.store(0, std::memory_order_release);
            ++g_edgeCancels;
            g_edgeLast = "cancelled (vehicle bit back)";
            LogInfo("UTPP gamelook: vehicle edge zeroing cancelled after %.3f s (vehicle bit back; writes=%d)",
                    static_cast<double>(now - arm) * 1e-9, g_edgeWrites.load());
        }
    }
    else if (ev == EdgeEvent::NoTpp)
    {
        // round 2: an exit phase armed in third person ends here too (no zeroing on foot in first person)
        const int64_t exitArm = g_exitArmNs.exchange(0, std::memory_order_acq_rel);
        ++g_edgeNoTpp;
        g_edgeLast = "edge in first person (no zeroing)";
        if (g_edgeLogs++ < 50)
            LogInfo("UTPP gamelook: vehicle edge in first person -> no look zeroing (the game look is the view; exit phase "
                    "%s, writes=%d)", exitArm != 0 ? "ended" : "none", g_edgeWrites.load());
    }
    else if (ev == EdgeEvent::Arm)
    {
        // round 2: the exit phase (if any) ends here; its result goes into the edge line
        const int64_t exitArm = g_exitArmNs.exchange(0, std::memory_order_acq_rel);
        char exitBuf[400] = "exit phase: none (no exit start seen in third person)";
        if (exitArm != 0)
            std::snprintf(exitBuf, sizeof(exitBuf),
                          "exit phase %.2f s: entYaw %.2f -> %.2f, yawIn %.2f -> %.2f, ticks=%d writes=%d corrections=%d "
                          "maxDelta=%.2f locked=%d sentinel=%d unlockBlendTicks=%d featRepl=%d ownerMiss=%d",
                          static_cast<double>(now - exitArm) * 1e-9, static_cast<double>(g_exitEntYaw0),
                          static_cast<double>(g_edgeLastEntYaw.load()), static_cast<double>(g_exitYawIn0),
                          static_cast<double>(g_edgeLastYawIn.load()), g_edgeTicks.load(), g_edgeWrites.load(),
                          g_edgeCorr.load(), static_cast<double>(g_edgeMaxDelta.load()), g_edgeLocked.load(),
                          g_edgeSentinel.load(), g_edgeBlend.load(), g_edgeFeatRepl.load(), g_edgeOwnerMiss.load());
        // round 3: the last RotateEntityToCamera look read (normally ~0.13 s before this edge)
        char rotBuf[240];
        {
            const int64_t rn = g_rotLastNs.load(std::memory_order_relaxed);
            if (!g_rotHook.load(std::memory_order_relaxed))
                std::snprintf(rotBuf, sizeof(rotBuf), "rotate guard: not installed");
            else if (rn == 0)
                std::snprintf(rotBuf, sizeof(rotBuf), "rotate guard: no RotateEntityToCamera call yet");
            else
                std::snprintf(rotBuf, sizeof(rotBuf),
                              "rotate guard: last call %.0f ms before the edge, yawIn read %.2f -> %s (entYaw %.2f), "
                              "zeroed=%llu passed=%llu notValue=%llu",
                              static_cast<double>(now - rn) * 1e-6, static_cast<double>(g_rotLastYaw.load()),
                              g_rotLastZeroed.load() ? "0 (zeroed)" : "kept", static_cast<double>(g_rotLastEntYaw.load()),
                              static_cast<unsigned long long>(g_rotZeroed.load()),
                              static_cast<unsigned long long>(g_rotPassed.load()),
                              static_cast<unsigned long long>(g_rotNotValue.load()));
        }
        std::string why;
        CompRead r{};
        if (!ResolveForZeroLocked(r, why))
        {
            ++g_edgeArmFails;
            g_edgeLast = "arm failed: " + why;
            if (g_edgeLogs++ < 50)
                LogWarn("UTPP gamelook: vehicle edge -> look zeroing not possible: %s (the game may turn V onto its look; %s; "
                        "%s)", why.c_str(), exitBuf, rotBuf);
        }
        else
        {
            const float entYaw = YawFromQuat(r.q[0], r.q[1], r.q[2], r.q[3]);
            ResetWindowStatsLocked(entYaw, r.yawIn);
            g_edgeSeq.fetch_add(1, std::memory_order_acq_rel);
            g_edgeArmNs.store(now, std::memory_order_release);
            g_edgeArms.fetch_add(1, std::memory_order_relaxed);
            g_edgeLast = "armed";
            if (g_edgeLogs++ < 50)
                LogInfo("UTPP gamelook: vehicle edge -> look zeroing for %.2f s (entYaw=%.2f yawIn=%.2f lookYaw=%.2f "
                        "headingLocked=%d unlockBlend=%.3f lookWrite=%d; %s; %s)",
                        static_cast<double>(kEdgeZeroSeconds), static_cast<double>(entYaw), static_cast<double>(r.yawIn),
                        static_cast<double>(Wrap180(entYaw + r.yawIn)), r.locked ? 1 : 0,
                        static_cast<double>(r.unlockBlend), g_active.load() ? 1 : 0, exitBuf, rotBuf);
        }
    }
    // summary once the drift window is over (game thread; the hook only stores values)
    const int64_t arm = g_edgeArmNs.load(std::memory_order_relaxed);
    if (arm != 0 && static_cast<double>(now - arm) * 1e-9 >= static_cast<double>(kEdgeReportSeconds))
    {
        g_edgeArmNs.store(0, std::memory_order_release);
        const float drift = g_edgeMaxDrift.load();
        const int ticks = g_edgeTicks.load();
        char buf[160];
        std::snprintf(buf, sizeof(buf), "done: drift %.2f deg, writes %d", static_cast<double>(drift), g_edgeWrites.load());
        g_edgeLast = buf;
        if (g_edgeLogs++ < 50)
            LogInfo("UTPP gamelook: vehicle edge zeroing done (+%.2f s): entity yaw drift max %.2f deg (%s 2 deg; edge %.2f -> "
                    "%.2f), yawIn first %.2f last %.2f, first write %.1f ms after the edge, ticks=%d writes=%d corrections=%d "
                    "maxDelta=%.2f locked=%d sentinel=%d nonFinite=%d unlockBlendTicks=%d featRepl=%d ownerMiss=%d busy=%d "
                    "componentCalls=%llu, entYaw at the rotate call %.2f%s",
                    static_cast<double>(now - arm) * 1e-9, static_cast<double>(drift), drift < 2.0f ? "<" : ">=",
                    static_cast<double>(g_edgeEntYaw0.load()), static_cast<double>(g_edgeLastEntYaw.load()),
                    static_cast<double>(g_edgeFirstYawIn.load()), static_cast<double>(g_edgeLastYawIn.load()),
                    static_cast<double>(g_edgeFirstMs.load()), ticks, g_edgeWrites.load(), g_edgeCorr.load(),
                    static_cast<double>(g_edgeMaxDelta.load()), g_edgeLocked.load(), g_edgeSentinel.load(),
                    g_edgeNonFinite.load(), g_edgeBlend.load(), g_edgeFeatRepl.load(), g_edgeOwnerMiss.load(),
                    g_edgeBusy.load(), static_cast<unsigned long long>(g_callsComp.load() - g_edgeCallsAtArm),
                    static_cast<double>(g_rotLastEntYaw.load()), ticks == 0 ? " (no hook tick in the window: the component update did not run)" : "");
    }
}

std::string Status()
{
    std::string why, inst, edgeLast;
    uint64_t sets, fails, stops, aheadSets, aheadBad, rearms, edgeNoTpp, edgeCancels, edgeArmFails, exitStarts, exitNoTpp;
    float entCheck;
    {
        std::lock_guard lock(g_coldMutex);
        edgeLast = g_edgeLast;
        edgeNoTpp = g_edgeNoTpp;
        edgeCancels = g_edgeCancels;
        edgeArmFails = g_edgeArmFails;
        exitStarts = g_exitStarts;
        exitNoTpp = g_exitNoTpp;
        why = g_lastWhy;
        inst = g_installWhy;
        sets = g_sets;
        fails = g_setFails;
        stops = g_stops;
        aheadSets = g_aheadSets;
        aheadBad = g_aheadBad;
        rearms = g_rearms;
        entCheck = g_lastEntYawCheck;
    }
    auto u = [](const std::atomic<uint64_t>& a) { return static_cast<unsigned long long>(a.load()); };
    auto d = [](const std::atomic<float>& a) { return static_cast<double>(a.load()); };
    // 0.6.5b ahead check, round 2: three order hypotheses (mean |entity turn between two reads - ahead|): after (the
    // ahead of the last tick: the turn lands after the camera update, assumed), before (this tick's: it landed before the
    // read), late (two ticks ago). The true order has ~0 and is clearly below the other two (aheadOrder = the one used).
    const unsigned long long an = u(g_aheadN), dn = u(g_aheadDiscN);
    const double aDiv = an ? static_cast<double>(an) : 1.0;
    const double dDiv = dn ? static_cast<double>(dn) : 1.0;
    const bool tripped = g_tripped.load();
    const int tripCount = g_tripCount.load();
    const double tripOff = tripped ? static_cast<double>(NowNs() - g_tripAtNs.load()) * 1e-9 : 0.0;
    std::string s;
    s.reserve(3000);
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "gamelook hook=%d active=%d tripped=%d tripCount=%d tripPermanent=%d tripOff=%.1f rearms=%llu comp=%p "
                  "target=%.1f/%.1f sets=%llu setFails=%llu stops=%llu calls=%llu callsComp=%llu writes=%llu stale=%llu "
                  "busy=%llu faults=%llu ownerMismatch=%llu notLocked=%llu sentinel=%llu nonFinite=%llu trippedSkips=%llu "
                  "extZeroed=%llu featFaults=%llu ",
                  g_installed.load() ? 1 : 0, g_active.load() ? 1 : 0, tripped ? 1 : 0, tripCount,
                  (tripped && tripCount >= kMaxTrips) ? 1 : 0, tripOff, static_cast<unsigned long long>(rearms),
                  g_comp.load(), d(g_targetYaw), d(g_targetPitch), static_cast<unsigned long long>(sets),
                  static_cast<unsigned long long>(fails), static_cast<unsigned long long>(stops), u(g_calls),
                  u(g_callsComp), u(g_writes), u(g_stale), u(g_busyCount), u(g_faults), u(g_ownerMismatch),
                  u(g_skipNotLocked), u(g_skipSentinel), u(g_skipNonFinite), u(g_skipTripped), u(g_extZeroed),
                  u(g_featFaults));
    s += buf;
    std::snprintf(buf, sizeof(buf),
                  "k=%.2f err=%.2f/%.2f rawErr=%.2f/%.2f maxAbsErr=%.2f ff=%.3f/%.3f ovrTicks=%llu releases=%llu "
                  "lastRelease=%.2f maxRelease=%.2f ",
                  static_cast<double>(kGain), d(g_lastErrYaw), d(g_lastErrPitch), d(g_lastRawErrYaw), d(g_lastRawErrPitch),
                  d(g_maxAbsErr), d(g_lastFfYaw), d(g_lastFfPitch), u(g_ovrTicks), u(g_releases), d(g_lastRelease),
                  d(g_maxRelease));
    s += buf;
    std::snprintf(buf, sizeof(buf),
                  "aheadSets=%llu aheadBad=%llu aheadStale=%llu aheadUsed=%llu ahead=%.2f aheadEff=%.2f aheadN=%llu "
                  "aheadMean=%.2f aheadNoFeed=%.3f aheadDiscN=%llu aheadAfter=%.3f aheadBefore=%.3f aheadLate=%.3f "
                  "aheadOrder=%s aheadOrderSwitches=%u lagHint=%d lagHintSwitches=%u ",
                  static_cast<unsigned long long>(aheadSets), static_cast<unsigned long long>(aheadBad), u(g_aheadStale),
                  u(g_aheadUsed), d(g_lastAhead), d(g_lastAheadEff), an, g_aheadAbs.load() / aDiv,
                  g_aheadNoFeed.load() / aDiv, dn, g_resAfter.load() / dDiv, g_resBefore.load() / dDiv,
                  g_resLate.load() / dDiv, AheadOrderName(static_cast<AheadOrder>(g_aheadOrder.load())),
                  g_aheadOrderSwitches.load(), g_lag.load(), g_lagSwitches.load());
    s += buf;
    std::snprintf(buf, sizeof(buf),
                  "entYaw=%.2f yawIn=%.2f pitchIn=%.2f yawLimits=%.1f/%.1f desired=%.2f/%.2f delta=%.3f/%.3f "
                  "feat=%.3f/%.3f featRepl=%llu/%llu featReplOvr=%llu maxFeatDiff=%.2f/%.2f yawMult=%.3f pitchMult=%.3f "
                  "resetTicks=%llu reset=%.2f/%.2f overrideTicks=%llu override=%.3f maxAbsDelta=%.2f entYawCheck=%.3f ",
                  d(g_lastEntYaw), d(g_lastYawIn), d(g_lastPitchIn), d(g_yawMaxLeft), d(g_yawMaxRight),
                  d(g_lastDesiredYaw), d(g_lastDesiredPitch), d(g_lastYawDelta), d(g_lastPitchDelta), d(g_lastFeatYaw),
                  d(g_lastFeatPitch), u(g_featYawRepl), u(g_featPitchRepl), u(g_featReplOvr), d(g_maxFeatYawDiff),
                  d(g_maxFeatPitchDiff), d(g_lastYawMult), d(g_lastPitchMult), u(g_resetTicks), d(g_lastResetYaw),
                  d(g_lastResetPitch), u(g_overrideTicks), d(g_lastOverride), d(g_maxAbsYawDelta),
                  static_cast<double>(entCheck));
    s += buf;
    std::snprintf(buf, sizeof(buf),
                  "edgeArms=%llu edgeWrites=%llu edgeNoTpp=%llu edgeCancels=%llu edgeArmFails=%llu exitStarts=%llu "
                  "exitNoTpp=%llu exitExpired=%llu exitOn=%d rotHook=%d rotCalls=%llu rotZeroed=%llu rotPassed=%llu "
                  "rotNotValue=%llu ",
                  u(g_edgeArms), u(g_edgeWritesTotal), static_cast<unsigned long long>(edgeNoTpp),
                  static_cast<unsigned long long>(edgeCancels), static_cast<unsigned long long>(edgeArmFails),
                  static_cast<unsigned long long>(exitStarts), static_cast<unsigned long long>(exitNoTpp),
                  u(g_exitExpired), g_exitArmNs.load() != 0 ? 1 : 0, g_rotHook.load() ? 1 : 0, u(g_rotCalls),
                  u(g_rotZeroed), u(g_rotPassed), u(g_rotNotValue));
    s += buf;
    s += "edge=\"" + edgeLast + "\" last=\"" + why + "\" install=\"" + inst + "\"";
    g_maxAbsYawDelta.store(0.0f, std::memory_order_relaxed); // per status interval
    g_maxAbsErr.store(0.0f, std::memory_order_relaxed);
    g_maxFeatYawDiff.store(0.0f, std::memory_order_relaxed);
    g_maxFeatPitchDiff.store(0.0f, std::memory_order_relaxed);
    return s;
}
} // namespace utpp::gamelook

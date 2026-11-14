// Read-only mapping census. Nothing here changes animation records, mapping
// tables or FK ownership. The mapping ABI/layout are corroborated by
// cyberpunk-vr-port, src/Hooks/AnimPose.cpp (MIT, dariulone, 2026),
// a decompile of the pose chain and the exact validated EXE prologue.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/animMetaRig.hpp>
#include <RED4ext/Scripting/Natives/animRig.hpp>

#include "CommonUtil.hpp"
#include "PoseHook.hpp"
#include "State.hpp"
#include "FkHook.hpp"
#include "EarlyPoseRuntime.hpp"
#include "BodyHelperRuntime.hpp"
#include "ragdoll/RdPoseTable.hpp"
#include "ragdoll/RagdollEpisode.hpp"
#include "ragdoll/RdLastPose.hpp"
#include "ragdoll/RagdollSlotProbe.hpp"
#include "mm/VehicleTransition.hpp" // vehicle batch W1 P2: mount / exit transition on the Mapping owner path
#include "mm/CoreWriteRecord.hpp"

#include <atomic>
#include <cstring>
#include <cstddef>

namespace utpp
{
namespace
{
Mapping_t g_originalMapping = nullptr;
std::atomic<bool> g_enabled{false};
std::atomic<uint64_t> g_calls{0}, g_player{0}, g_root{0}, g_fk{0};
std::atomic<uint64_t> g_next{0}, g_stored{0}, g_dropped{0}, g_faults{0};
std::atomic<uint32_t> g_faultCode{0};
mm::semanticpose::Capture g_semantic;
std::atomic<uint64_t> g_semanticGeneration{0}, g_semanticStored{0}, g_semanticDuplicate{0};
std::atomic<uint64_t> g_semanticInvalid{0}, g_semanticBusy{0}, g_semanticFull{0}, g_semanticStale{0};
std::atomic<uint64_t> g_utcFileTime{0};
std::atomic<int64_t> g_startTicks{0}, g_qpcHz{0};
static_assert(sizeof(RED4ext::CName) == sizeof(uint64_t));
static_assert(offsetof(RED4ext::CName, hash) == 0);
static_assert(offsetof(RED4ext::anim::MetaRig, boneNames) == 0x20);
static_assert(offsetof(RED4ext::anim::Rig, boneNames) == 0x50);
struct Slot
{
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    posediag::Event event;
};
Slot g_ring[posediag::kCensusCapacity];
struct CyclePolicy { mm::mappingcycle::Plan plan; uint64_t rig = 0; };
mm::integ::SeqlockBuffer<CyclePolicy> g_cyclePolicy;
std::atomic<uint64_t> g_cycleVersion{0};
std::atomic<bool> g_cycleReady{false};
struct CycleWorker
{
    CyclePolicy policy;
    mm::mappingcycle::State state;
    uint64_t session = 0;
};
thread_local CycleWorker g_cycleWorker;
// Mapping owner: core writers (and the weight-0 writer window) of the mapping jobs of the pending cycle of this thread
// (mm/CoreWriteRecord.hpp). Kept only while the cycle the main job started is pending; handed to the FK call that consumes that cycle
// and cleared by every FK call, so the writers of one cycle never reach the FK call of another one.
struct EarlyWriters { uint64_t cycle = 0; uint32_t writers = 0; };
thread_local EarlyWriters g_earlyWriters;
void KeepEarlyWriters(uint32_t aRecord)
{
    const auto& state = g_cycleWorker.state;
    if (!state.active || !state.cycle) { g_earlyWriters = {}; return; }
    if (g_earlyWriters.cycle != state.cycle) g_earlyWriters = {state.cycle, 0};
    g_earlyWriters.writers |= mm::corewrite::EarlyPart(aRecord); // the helper jobs of the cycle add nothing and remove nothing
}
// Reset of the pending cycle evidence for the per-FK-call path. A state that is not active is already the reset state (only Observe
// writes it, setting `active` together with every other field; Reset clears all of them), so the ~2 KB clear is skipped for it:
// with the diagnostics off this ran on every FK call of every skeleton.
inline void ResetCycleIfActive(mm::mappingcycle::State& aState)
{
    if (aState.active)
        mm::mappingcycle::Reset(aState);
}
bool RefreshCyclePolicy()
{
    auto& worker = g_cycleWorker;
    const uint64_t version = g_cycleVersion.load(std::memory_order_acquire);
    const uint64_t session = g_semanticGeneration.load(std::memory_order_acquire);
    if (!g_cycleReady.load(std::memory_order_acquire))
    { mm::mappingcycle::Reset(worker.state); return false; }
    if (worker.policy.plan.generation != version || worker.session != session)
    {
        mm::mappingcycle::Reset(worker.state);
        worker.session = session;
        if (!g_cyclePolicy.Read(worker.policy) || worker.policy.plan.generation != version) return false;
    }
    return worker.policy.plan.valid;
}

bool PlayerTag(uint64_t aTag)
{
    return aTag && (aTag == g_hot.trackA.load(std::memory_order_relaxed) ||
                   aTag == g_hot.trackB.load(std::memory_order_relaxed));
}

// Ragdoll R1o probe (ragdoll/RdPoseTable.hpp): map row per source rig merged into a player-tagged descriptor, read right
// after the original merge and before any early MM write. No C++ objects with destructors inside the SEH boundary.
// Hips translation of the player-tagged output descriptor before the merge; false when it is not V's or unreadable.
__declspec(noinline) bool RdMapPre(void* aContext, float (&aOut)[4])
{
    __try
    {
        auto* desc = *reinterpret_cast<uint8_t**>(static_cast<uint8_t*>(aContext) + 0x38);
        if (!desc || !PlayerTag(*reinterpret_cast<const uint64_t*>(desc + kDescTagOff)))
            return false;
        const uint8_t* bones = *reinterpret_cast<uint8_t* const*>(desc + kDescBonesOff);
        const uint32_t count = *reinterpret_cast<const uint32_t*>(desc + kDescCountOff);
        const int32_t hips = rdpose::g_idx[0].load(std::memory_order_relaxed);
        if (!bones || hips < 0 || static_cast<uint32_t>(hips) >= count)
            return false;
        const auto* b = reinterpret_cast<const float*>(bones + static_cast<size_t>(hips) * kBoneStride);
        for (int j = 0; j < 4; ++j)
            aOut[j] = b[j];
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

__declspec(noinline) void RdMapCapture(void* aGraph, void* aContext, const float (&aPre)[4])
{
    __try
    {
        auto* desc = *reinterpret_cast<uint8_t**>(static_cast<uint8_t*>(aContext) + 0x38);
        if (!desc || !PlayerTag(*reinterpret_cast<const uint64_t*>(desc + kDescTagOff)))
            return;
        auto* g = static_cast<uint8_t*>(aGraph);
        auto* sourceRig = *reinterpret_cast<const RED4ext::anim::Rig* const*>(g + 0x40);
        rdpose::Row* r = rdpose::Find(rdpose::g_map, reinterpret_cast<uint64_t>(sourceRig));
        if (!r)
            return;
        const uint8_t* bones = *reinterpret_cast<uint8_t* const*>(desc + kDescBonesOff);
        const uint32_t count = *reinterpret_cast<const uint32_t*>(desc + kDescCountOff);
        r->a.store(reinterpret_cast<uint64_t>(desc), std::memory_order_relaxed);
        r->b.store(reinterpret_cast<uint64_t>(aGraph), std::memory_order_relaxed);
        r->count.store(sourceRig ? sourceRig->boneNames.size() : 0, std::memory_order_relaxed);
        r->aux.store(count, std::memory_order_relaxed);
        r->calls.fetch_add(1, std::memory_order_relaxed);
        r->seq.store(rdpose::g_seq.fetch_add(1, std::memory_order_relaxed), std::memory_order_relaxed);
        rdpose::Store4(r->pre, aPre);
        if (bones)
        {
            const int32_t hips = rdpose::g_idx[0].load(std::memory_order_relaxed);
            if (hips >= 0 && static_cast<uint32_t>(hips) < count)
            {
                const auto* b = reinterpret_cast<const float*>(bones + static_cast<size_t>(hips) * kBoneStride);
                rdpose::Store4(r->v[0], b);
                rdpose::Store4(r->v[1], b + 4);
            }
            for (int i = 1; i < 3; ++i)
            {
                const int32_t k = rdpose::g_idx[i].load(std::memory_order_relaxed);
                if (k >= 0 && static_cast<uint32_t>(k) < count)
                    rdpose::Store4(r->v[i + 1], reinterpret_cast<const float*>(bones + static_cast<size_t>(k) * kBoneStride) + 4);
            }
        }
        r->f[0].store(*reinterpret_cast<const uint64_t*>(g + 0x18), std::memory_order_relaxed);
        r->f[1].store(*reinterpret_cast<const uint64_t*>(g + 0x30), std::memory_order_relaxed);
        const auto* h = *reinterpret_cast<const uint8_t* const*>(g + 0x20);
        r->f[5].store(h ? *reinterpret_cast<const uint64_t*>(h + 0x30) : 0, std::memory_order_relaxed);
        // Last: the analysed entry layout may not hold for aGraph; a fault here leaves the earlier fields intact.
        r->f[2].store(*reinterpret_cast<const uint64_t*>(g + 0x25a0), std::memory_order_relaxed);
        r->f[3].store(*(g + 0x25b1), std::memory_order_relaxed);
        r->f[4].store(*reinterpret_cast<const uint64_t*>(g + 0x25c8), std::memory_order_relaxed);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
}

// Ragdoll production core (ragdoll/RdKeep.hpp, RagdollEpisode.hpp): later merges (deformations, shadow, face, ...) re-emit V's core
// bones from poses that never carry the ragdoll overlay. While the episode owns the body (the overlay gate was seen, plus the engine's
// 0.5 s blend-out) the bones the player_base merge wrote (its own (source, destination) pairs for this LOD) are saved per output
// descriptor and written back after each later merge into the same descriptor, so FK sees the ragdoll. Gather / scatter run under
// SEH in small POD-only functions; the try-lock of the store is taken and released OUTSIDE them, so a fault never leaks it.
constexpr uint64_t kPlayerBaseGraphHash = 0x7C8EF3AC7AF95509; // FNV-1a64 of base\gameplay\anim_graphs\player_base.animgraph

struct RdKeepTarget
{
    uint8_t* desc = nullptr;
    uint8_t* bones = nullptr;
    uint32_t count = 0;
    uint64_t graphHash = 0;
    const uint32_t* pairs = nullptr; // player_base merge only: (source, destination) uint32 pairs of this LOD
    uint32_t pairCount = 0;
};

__declspec(noinline) bool RdKeepReadTarget(void* aGraph, void* aContext, unsigned int aLod, RdKeepTarget& aOut)
{
    __try
    {
        auto* desc = *reinterpret_cast<uint8_t**>(static_cast<uint8_t*>(aContext) + 0x38);
        if (!desc || aLod > 7 || !PlayerTag(*reinterpret_cast<const uint64_t*>(desc + kDescTagOff)))
            return false;
        uint8_t* bones = *reinterpret_cast<uint8_t**>(desc + kDescBonesOff);
        const uint32_t count = *reinterpret_cast<const uint32_t*>(desc + kDescCountOff);
        if (!bones || !count)
            return false;
        const auto* h = *reinterpret_cast<const uint8_t* const*>(static_cast<uint8_t*>(aGraph) + 0x20);
        aOut.desc = desc;
        aOut.bones = bones;
        aOut.count = count;
        aOut.graphHash = h ? *reinterpret_cast<const uint64_t*>(h + 0x30) : 0;
        aOut.pairs = nullptr;
        aOut.pairCount = 0;
        if (aOut.graphHash == kPlayerBaseGraphHash)
        {
            const auto* mapping = *reinterpret_cast<const uint8_t* const*>(static_cast<uint8_t*>(aContext) + 0x30);
            if (!mapping)
                return false;
            const auto* counts = *reinterpret_cast<const uint16_t* const*>(mapping + 0x50);
            const auto* pairs = *reinterpret_cast<const uint32_t* const*>(mapping + 0x40);
            if (!counts || !pairs)
                return false;
            aOut.pairs = pairs;
            aOut.pairCount = counts[aLod];
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Copies the destination records of the root merge into the locked slot. false = fault (nothing may be kept).
__declspec(noinline) bool RdKeepGather(ragdoll::RdKeep::Slot* aSlot, const RdKeepTarget& aT, uint32_t& aN)
{
    __try
    {
        uint32_t n = 0;
        for (uint32_t i = 0; i < aT.pairCount && n < ragdoll::RdKeep::kMaxBones; ++i)
        {
            const uint32_t dst = aT.pairs[2 * i + 1];
            if (dst >= aT.count)
                continue;
            aSlot->dst[n] = dst;
            std::memcpy(aSlot->data[n], aT.bones + static_cast<size_t>(dst) * kBoneStride, kBoneStride);
            ++n;
        }
        aN = n;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

__declspec(noinline) bool RdKeepScatter(const ragdoll::RdKeep::Slot* aSlot, const RdKeepTarget& aT)
{
    __try
    {
        for (uint32_t i = 0; i < aSlot->n; ++i)
            if (aSlot->dst[i] < aT.count)
                std::memcpy(aT.bones + static_cast<size_t>(aSlot->dst[i]) * kBoneStride, aSlot->data[i], kBoneStride);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

void RdKeepApply(void* aGraph, void* aContext, unsigned int aLod)
{
    RdKeepTarget t;
    if (!RdKeepReadTarget(aGraph, aContext, aLod, t))
        return;
    ragdoll::RdKeep& keep = ragdollslot::Keep();
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    const uint32_t thread = GetCurrentThreadId();
    const uint64_t desc = reinterpret_cast<uint64_t>(t.desc);
    if (t.graphHash == kPlayerBaseGraphHash)
    {
        ragdoll::RdKeep::Slot* slot = keep.BeginSave(desc);
        if (!slot)
            return;
        uint32_t n = 0;
        if (!RdKeepGather(slot, t, n))
        {
            keep.AbortSave(slot);
            return;
        }
        // get-up: a copy of the latest ragdoll pose for the game thread (RdLastPose.hpp; skipped while frozen, allocation-free)
        ragdoll::LastPose().Write(desc, slot->dst, slot->data, n, now.QuadPart, t.count);
        keep.CommitSave(slot, n, now.QuadPart, thread);
        return;
    }
    ragdoll::RdKeep::Slot* slot = keep.BeginRestore(desc, now.QuadPart, thread);
    if (!slot)
        return;
    if (!RdKeepScatter(slot, t))
    {
        keep.AbortRestore(slot);
        return;
    }
    keep.EndRestore(slot);
}

void Store(posediag::Event& aEvent)
{
    const uint64_t ticket = g_next.fetch_add(1, std::memory_order_relaxed);
    Slot& slot = g_ring[ticket % posediag::kCensusCapacity];
    // Never wait on an animation worker or on the game-thread reader.
    if (slot.busy.test_and_set(std::memory_order_acquire))
    {
        g_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    aEvent.sequence = ticket + 1;
    slot.event = aEvent;
    slot.busy.clear(std::memory_order_release);
    g_stored.fetch_add(1, std::memory_order_relaxed);
}

void RecordSemanticResult(mm::semanticpose::Result aResult)
{
    using mm::semanticpose::Result;
    switch (aResult)
    {
    case Result::Stored: g_semanticStored.fetch_add(1, std::memory_order_relaxed); break;
    case Result::Duplicate: g_semanticDuplicate.fetch_add(1, std::memory_order_relaxed); break;
    case Result::Invalid: g_semanticInvalid.fetch_add(1, std::memory_order_relaxed); break;
    case Result::Busy: g_semanticBusy.fetch_add(1, std::memory_order_relaxed); break;
    case Result::Full: g_semanticFull.fetch_add(1, std::memory_order_relaxed); break;
    case Result::StaleGeneration: g_semanticStale.fetch_add(1, std::memory_order_relaxed); break;
    default: break;
    }
}

// POD only: MSVC SEH must not unwind C++ objects. Reads only established fields.
void CensusMapping(void* aGraph, void* aContext, void* aSource, unsigned int aLod)
{
    __try
    {
        if (!aGraph || !aContext)
            { mm::mappingcycle::Reset(g_cycleWorker.state); return; }
        // Mapping context+0x38 points to the full output descriptor; graph+0x40
        // points to the source animRig RESOURCE (names@0x50), not the merged
        // animMetaRig (names@0x20). Source+0x30 is the incoming local bone buffer.
        auto* desc = *reinterpret_cast<uint8_t**>(static_cast<uint8_t*>(aContext) + 0x38);
        if (!desc)
            { mm::mappingcycle::Reset(g_cycleWorker.state); return; }
        const uint64_t tag = *reinterpret_cast<uint64_t*>(desc + kDescTagOff);
        if (!PlayerTag(tag))
            { mm::mappingcycle::Reset(g_cycleWorker.state); return; }
        g_player.fetch_add(1, std::memory_order_relaxed);
        posediag::Event event;
        event.desc = reinterpret_cast<uint64_t>(desc);
        event.bones = *reinterpret_cast<uint64_t*>(desc + kDescBonesOff);
        event.tag = tag;
        event.count = *reinterpret_cast<uint32_t*>(desc + kDescCountOff);
        event.rig = *reinterpret_cast<uint64_t*>(static_cast<uint8_t*>(aGraph) + 0x40);
        event.graph = reinterpret_cast<uint64_t>(aGraph);
        event.source = reinterpret_cast<uint64_t>(aSource);
        if (aSource)
            event.sourceBones = *reinterpret_cast<uint64_t*>(static_cast<uint8_t*>(aSource) + 0x30);
        event.lod = aLod;
        // Proven mapping-pair table layout: decompile-pose-chain.txt lines
        // 34-48. The per-LOD uint16 value counts uint32(source,destination)
        // pairs; it does NOT identify the source graph or its total bones.
        if (aLod <= 7)
        {
            const auto* mapping = *reinterpret_cast<const uint8_t**>(
                static_cast<uint8_t*>(aContext) + 0x30);
            if (mapping)
            {
                const auto* counts = *reinterpret_cast<const uint16_t* const*>(mapping + 0x50);
                const auto* pairs = *reinterpret_cast<const uint32_t* const*>(mapping + 0x40);
                if (counts)
                {
                    const uint32_t count = counts[aLod];
                    if (count <= 8192 && (pairs || count == 0))
                    {
                        // FNV-1a over the little-endian bytes (x64) of each source,destination uint32;
                        // bounded and allocation-free.
                        const uint64_t hash = util::Fnv1a64(reinterpret_cast<const uint8_t*>(pairs), size_t(count) * 2u * sizeof(uint32_t));
                        event.mappedBonePairsCount = count;
                        event.mappedBonePairsFingerprint = hash;
                        event.mappedBonePairsKnown = true;
                    }
                }
            }
        }
        event.rootRig = event.rig && event.rig == g_hot.playerRig.load(std::memory_order_relaxed);
        if (event.rootRig)
            g_root.fetch_add(1, std::memory_order_relaxed);
        if (!event.bones || !event.count || event.count > 8192)
            { mm::mappingcycle::Reset(g_cycleWorker.state); return; }
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        event.ticks = now.QuadPart;
        event.thread = GetCurrentThreadId();
        if ((g_semantic.Enabled() || bodyhelperruntime::Enabled()) && event.mappedBonePairsKnown)
        {
            // SDK animRig.hpp and VR WeaponRig.cpp agree on the resource
            // layout. Only the separately armed destination uses MetaRig.
            auto* sourceRig = reinterpret_cast<const RED4ext::anim::Rig*>(event.rig);
            const uint64_t destinationRigAddress = g_hot.playerRig.load(std::memory_order_acquire);
            auto* destinationRig = reinterpret_cast<const RED4ext::anim::MetaRig*>(destinationRigAddress);
            const uint32_t sourceCount = sourceRig ? sourceRig->boneNames.size() : 0;
            const uint32_t destinationCount = destinationRig ? destinationRig->boneNames.size() : 0;
            if (!sourceCount || sourceCount > mm::semanticpose::kMaxBones ||
                !destinationCount || destinationCount > mm::semanticpose::kMaxBones ||
                destinationCount != event.count || event.mappedBonePairsCount > mm::semanticpose::kMaxPairs ||
                !sourceRig->boneNames.Data() || !destinationRig->boneNames.Data())
                g_semanticInvalid.fetch_add(1, std::memory_order_relaxed);
            else
            {
                const auto* mapping = *reinterpret_cast<const uint8_t**>(static_cast<uint8_t*>(aContext) + 0x30);
                const auto* pairs = *reinterpret_cast<const uint32_t* const*>(mapping + 0x40);
                mm::semanticpose::Identity identity;
                identity.generation = g_semanticGeneration.load(std::memory_order_acquire);
                identity.sourceRig = event.rig; identity.destinationRig = destinationRigAddress;
                identity.graph = event.graph; identity.desc = event.desc;
                identity.bones = event.bones; identity.tag = event.tag;
                identity.ticks = event.ticks; identity.lod = event.lod; identity.thread = event.thread;
                // All live pointer dereferences/copies occur inside this SEH
                // callback. The control path only receives owned hash values.
                const auto semanticResult = g_semantic.Observe(identity,
                    {reinterpret_cast<const uint64_t*>(sourceRig->boneNames.Data()), sourceCount},
                    {reinterpret_cast<const uint64_t*>(destinationRig->boneNames.Data()), destinationCount},
                    {pairs, event.mappedBonePairsCount * 2u});
                RecordSemanticResult(semanticResult);
                event.sourceCount = sourceCount;
                event.sourceCountKnown = true;
                if (event.sourceBones)
                    event.nativeBytes = mm::mappingreceipt::Compare(
                        {reinterpret_cast<const std::byte*>(event.sourceBones), sourceCount * kBoneStride},
                        {reinterpret_cast<const std::byte*>(event.bones), event.count * kBoneStride},
                        {pairs, event.mappedBonePairsCount * 2u});
                if (semanticResult != mm::semanticpose::Result::Invalid &&
                    semanticResult != mm::semanticpose::Result::StaleGeneration && RefreshCyclePolicy())
                {
                    const auto& policy = g_cycleWorker.policy;
                    mm::mappingcycle::Identity id{policy.plan.generation, event.desc, event.bones,
                        event.tag, destinationRigAddress, event.count, event.thread, event.lod};
                    event.cycleConfigured = policy.rig == destinationRigAddress;
                    if (event.cycleConfigured)
                    {
                        event.cycleObservation = static_cast<uint32_t>(mm::mappingcycle::Observe(
                            policy.plan, g_cycleWorker.state, id, event.ticks,
                            g_qpcHz.load(std::memory_order_relaxed) / 2,
                            {reinterpret_cast<const uint64_t*>(sourceRig->boneNames.Data()), sourceCount},
                            {pairs, event.mappedBonePairsCount * 2u}, event.nativeBytes));
                        event.cycle.generation = policy.plan.generation;
                        event.cycle.cycle = g_cycleWorker.state.cycle;
                    }
                    else mm::mappingcycle::Reset(g_cycleWorker.state);
                }
            }
        }
        if (!event.cycleConfigured) mm::mappingcycle::Reset(g_cycleWorker.state);
        if (g_enabled.load(std::memory_order_relaxed)) Store(event);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        mm::mappingcycle::Reset(g_cycleWorker.state);
        bodyhelperruntime::Disable();
        g_faultCode.store(GetExceptionCode(), std::memory_order_relaxed);
        g_faults.fetch_add(1, std::memory_order_relaxed);
        g_enabled.store(false, std::memory_order_release);
        g_semantic.Stop();
    }
}

// Independent of diagnostic census. All game-owned views and the shared apply
// callback stay inside this SEH boundary. No source/destination rig conflation.
void ApplyEarlyMapping(void* aGraph, void* aContext, unsigned int aLod)
{
    __try
    {
        // vehicle batch W1 P2: the mount / exit transition writes at MM weight 0 (mmActive false), so a running transition or
        // capture (vehtrans::Wanted) also passes this gate; without it the path is the one before the batch.
        // Ragdoll episode: the engine owns V's pose (RagdollEpisode.hpp); no MM / vehicle transition / AO write on the mapping path.
        if (!aGraph || !aContext || aLod > 7 || ragdoll::GameOwned() ||
            !g_hot.armed.load(std::memory_order_acquire) ||
            !(g_hot.mmActive.load(std::memory_order_acquire) || mm::vehtrans::Wanted()) ||
            g_hot.fault.load(std::memory_order_acquire)) return;
        auto* desc = *reinterpret_cast<uint8_t**>(static_cast<uint8_t*>(aContext) + 0x38);
        if (!desc) return;
        const uint64_t tag = *reinterpret_cast<const uint64_t*>(desc + kDescTagOff);
        if (!PlayerTag(tag)) return;
        auto* sourceRig = *reinterpret_cast<const RED4ext::anim::Rig* const*>(
            static_cast<uint8_t*>(aGraph) + 0x40);
        const auto* mapping = *reinterpret_cast<const uint8_t**>(static_cast<uint8_t*>(aContext) + 0x30);
        if (!sourceRig || !mapping) return;
        const uint32_t sourceCount = sourceRig->boneNames.size();
        const auto* names = sourceRig->boneNames.Data();
        const auto* counts = *reinterpret_cast<const uint16_t* const*>(mapping + 0x50);
        const auto* pairs = *reinterpret_cast<const uint32_t* const*>(mapping + 0x40);
        if (!names || !sourceCount || sourceCount > earlyruntime::kMaxRigBones || !counts || !pairs) return;
        const uint32_t pairCount = counts[aLod];
        if (!pairCount || pairCount > earlyruntime::kMaxRigBones) return;
        LARGE_INTEGER now{};
        QueryPerformanceCounter(&now);
        // vehicle batch W1 P2 (CAR-5 / MOTO-3 mount, MOTO-6 exit capture): this path runs only when the Mapping owner writes the pose
        // (opt-in experiment, early-pose.txt; the release owner is FK, see EarlyPoseRuntime.cpp), so
        // the transition runs here (the FK path does the same for the default FK owner). The perspective gate and the rig check stay
        // inside FkSide (a denial ends the transition and captures nothing); only the player's full output (count ==
        // playerBones) of the MAIN mapping job (FkSideMapping: main anchors + every transition bone covered, as EvaluateMapping);
        // the helper jobs of the same frame find the core records already written and must not apply the transition again.
        // Non-zero = a transition frame: no MM blend, reload contacts, AO or game pose sample on it.
        if (mm::vehtrans::Wanted())
        {
            // The weight-0 writer window of this cycle opens here, before the transition writes, as the FK owner opens it before
            // FkSide (FkHook.cpp ApplyMotionMatching): a write that ends the transition still reaches the helper recompute inside it.
            if (bodyhelperruntime::Enabled() && bodyhelperruntime::ZeroWriterRecompute() &&
                mm::vehtrans::g_mountTpp.load(std::memory_order_acquire))
                mm::corewrite::MarkZeroWindow(true);
            const bool vehAllows = g_hot.perspectiveWrite.Allows(now.QuadPart,
                g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed));
            auto* vehBones = *reinterpret_cast<uint8_t**>(desc + kDescBonesOff);
            const uint32_t vehCount = *reinterpret_cast<const uint32_t*>(desc + kDescCountOff);
            const uint32_t playerBones = g_hot.playerBones.load(std::memory_order_relaxed);
            if (vehBones && playerBones != 0 && vehCount == playerBones &&
                mm::vehtrans::FkSideMapping(vehBones, vehCount, g_hot.playerRig.load(std::memory_order_acquire), now.QuadPart,
                                            vehAllows, {reinterpret_cast<const uint64_t*>(names), sourceCount},
                                            {pairs, pairCount * 2u}) != 0)
                return;
            if (!g_hot.mmActive.load(std::memory_order_acquire)) return; // transition / capture only: no MM pose to write
        }
        if (!g_hot.perspectiveWrite.Allows(now.QuadPart,
                g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed))) return;
        mm::integ::PoseSnapshot snapshot;
        if (!g_mmPose.Read(snapshot))
        {
            g_hot.mmTornReads.value.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        earlyruntime::MappingView view;
        view.desc = desc;
        view.bones = *reinterpret_cast<uint8_t**>(desc + kDescBonesOff);
        view.tag = tag;
        view.sourceRig = reinterpret_cast<uint64_t>(sourceRig);
        view.outputCount = *reinterpret_cast<const uint32_t*>(desc + kDescCountOff);
        view.sourceNames = {reinterpret_cast<const uint64_t*>(names), sourceCount};
        view.pairs = {pairs, pairCount * 2u};
        earlyruntime::EvaluateMapping(view, snapshot, now.QuadPart, &ApplyPoseSnapshot);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        // Keep the selected Mapping owner. Never recover by applying the late
        // FK pose after a partial early write; latch MM off for this session.
        g_hot.faultCode.store(GetExceptionCode(), std::memory_order_relaxed);
        g_hot.enabled.store(false, std::memory_order_relaxed);
        g_hot.mmActive.store(false, std::memory_order_relaxed);
        g_hot.perspectiveWrite.Reset();
        g_hot.fault.store(true, std::memory_order_release);
        earlyruntime::Disarm();
    }
}
} // namespace

void* MappingDetour(void* aGraph, void* aContext, void* aSource, unsigned int aLod)
{
    float rdPre[4] = {};
    // R1o / R1p probe tables only while an episode runs: idle they cost nothing.
    const bool rdOn = ragdoll::GameOwned() && aGraph && aContext && rdpose::g_idx[0].load(std::memory_order_relaxed) >= 0 &&
                      RdMapPre(aContext, rdPre);
    void* result = g_originalMapping(aGraph, aContext, aSource, aLod);
    // Observe native bytes before our early core write contaminates this evidence.
    if (g_enabled.load(std::memory_order_relaxed) || bodyhelperruntime::Enabled())
    {
        g_calls.fetch_add(1, std::memory_order_relaxed);
        if (g_hot.armed.load(std::memory_order_acquire))
            CensusMapping(aGraph, aContext, aSource, aLod);
    }
    if (aGraph && aContext && ragdoll::GameOwned())
        RdKeepApply(aGraph, aContext, aLod);
    if (rdOn)
        RdMapCapture(aGraph, aContext, rdPre);
    if (earlyruntime::CurrentOwner() == earlyruntime::Owner::Mapping)
    {
        // The writers of this job go to the FK call of its cycle (KeepEarlyWriters), not to whatever record this thread holds.
        const uint32_t record = mm::corewrite::Current();
        mm::corewrite::Begin();
        ApplyEarlyMapping(aGraph, aContext, aLod);
        KeepEarlyWriters(mm::corewrite::Current());
        mm::corewrite::Restore(record);
    }
    return result;
}

Mapping_t* MappingOriginalSlot() { return &g_originalMapping; }

namespace posediag
{
bool PrepareCycleCensus(uint64_t rig, std::span<const uint64_t> names,
                        std::span<const uint64_t> helpers)
{
    g_cycleReady.store(false, std::memory_order_release);
    const uint64_t version = g_cycleVersion.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (g_hot.armed.load(std::memory_order_acquire) || !rig) return false;
    const std::array<uint64_t,5> anchors{
        mm::integ::CNameHash("reference_joint"), mm::integ::CNameHash("WeaponLeft"),
        mm::integ::CNameHash("WeaponRight"), mm::integ::CNameHash("LeftEye"), mm::integ::CNameHash("RightEye")};
    CyclePolicy policy;
    policy.rig = rig;
    if (mm::mappingcycle::Prepare(version, names, anchors, helpers, policy.plan) !=
        mm::mappingcycle::Error::None) return false;
    LARGE_INTEGER hz{}; QueryPerformanceFrequency(&hz);
    g_qpcHz.store(hz.QuadPart, std::memory_order_relaxed);
    g_cyclePolicy.Publish(policy);
    g_cycleReady.store(true, std::memory_order_release);
    return true;
}

void SetEnabled(bool aEnabled)
{
    g_enabled.store(false, std::memory_order_release);
    g_semantic.Stop();
    if (!aEnabled) return;
    const uint64_t generation = g_semanticGeneration.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (!g_semantic.Start(generation)) return;
    LARGE_INTEGER ticks{}, hz{};
    FILETIME utc{};
    QueryPerformanceCounter(&ticks); QueryPerformanceFrequency(&hz); GetSystemTimeAsFileTime(&utc);
    g_startTicks.store(ticks.QuadPart, std::memory_order_relaxed);
    g_qpcHz.store(hz.QuadPart, std::memory_order_relaxed);
    g_utcFileTime.store((uint64_t{utc.dwHighDateTime} << 32u) | utc.dwLowDateTime, std::memory_order_relaxed);
    g_semanticStored.store(0, std::memory_order_relaxed); g_semanticDuplicate.store(0, std::memory_order_relaxed);
    g_semanticInvalid.store(0, std::memory_order_relaxed); g_semanticBusy.store(0, std::memory_order_relaxed);
    g_semanticFull.store(0, std::memory_order_relaxed); g_semanticStale.store(0, std::memory_order_relaxed);
    g_enabled.store(true, std::memory_order_release);
}

Counters ReadCounters()
{
    Counters c;
    c.enabled = g_enabled.load(std::memory_order_acquire);
    c.mappingCalls = g_calls.load(std::memory_order_relaxed);
    c.playerPasses = g_player.load(std::memory_order_relaxed);
    c.rootPasses = g_root.load(std::memory_order_relaxed);
    c.fkPasses = g_fk.load(std::memory_order_relaxed);
    c.stored = g_stored.load(std::memory_order_relaxed);
    c.contentionDrops = g_dropped.load(std::memory_order_relaxed);
    c.readFaults = g_faults.load(std::memory_order_relaxed);
    c.faultCode = g_faultCode.load(std::memory_order_relaxed);
    c.semanticStored = g_semanticStored.load(std::memory_order_relaxed);
    c.semanticDuplicate = g_semanticDuplicate.load(std::memory_order_relaxed);
    c.semanticInvalid = g_semanticInvalid.load(std::memory_order_relaxed);
    c.semanticBusy = g_semanticBusy.load(std::memory_order_relaxed);
    c.semanticFull = g_semanticFull.load(std::memory_order_relaxed);
    c.semanticStale = g_semanticStale.load(std::memory_order_relaxed);
    c.semanticGeneration = g_semanticGeneration.load(std::memory_order_relaxed);
    c.utcFileTime = g_utcFileTime.load(std::memory_order_relaxed);
    c.startTicks = g_startTicks.load(std::memory_order_relaxed);
    c.qpcHz = g_qpcHz.load(std::memory_order_relaxed);
    return c;
}

bool CopySemantic(uint32_t aIndex, mm::semanticpose::Record& aOut) { return g_semantic.Copy(aIndex, aOut); }
bool SemanticCount(uint32_t& aOut) { return g_semantic.Count(aOut); }

size_t CopyCensus(Event* aOut, size_t aCapacity)
{
    if (!aOut || !aCapacity)
        return 0;
    size_t n = 0;
    for (Slot& slot : g_ring)
    {
        if (n == aCapacity)
            break;
        if (slot.busy.test_and_set(std::memory_order_acquire))
            continue;
        if (slot.event.sequence)
            aOut[n++] = slot.event;
        slot.busy.clear(std::memory_order_release);
    }
    return n;
}

void NoteBeforeFK(void* aDesc, void* aRig, int64_t aTicks)
{
    // Every FK call takes the early writers of this thread, delivered below only to the FK call of their own cycle.
    const EarlyWriters early = g_earlyWriters;
    if (early.writers) g_earlyWriters = {};
    if ((!g_enabled.load(std::memory_order_relaxed) && !bodyhelperruntime::Enabled()) || !aDesc ||
        !g_hot.armed.load(std::memory_order_acquire))
        { ResetCycleIfActive(g_cycleWorker.state); return; }
    __try
    {
        auto* desc = static_cast<uint8_t*>(aDesc);
        const uint64_t tag = *reinterpret_cast<uint64_t*>(desc + kDescTagOff);
        if (!PlayerTag(tag))
            { ResetCycleIfActive(g_cycleWorker.state); return; }
        Event event;
        event.kind = EventKind::BeforeFK;
        event.desc = reinterpret_cast<uint64_t>(aDesc);
        event.bones = *reinterpret_cast<uint64_t*>(desc + kDescBonesOff);
        event.tag = tag;
        event.rig = reinterpret_cast<uint64_t>(aRig);
        event.count = *reinterpret_cast<uint32_t*>(desc + kDescCountOff);
        event.rootRig = event.rig && event.rig == g_hot.playerRig.load(std::memory_order_relaxed);
        event.ticks = aTicks;
        event.thread = GetCurrentThreadId();
        if (RefreshCyclePolicy())
        {
            const auto& policy = g_cycleWorker.policy;
            mm::mappingcycle::FKIdentity id{policy.plan.generation, event.desc, event.bones,
                event.tag, event.rig, event.count, event.thread};
            event.cycleConfigured = policy.rig == event.rig;
            // Always consume, including FPP denial or identity mismatch. No pose writes.
            event.cycle = mm::mappingcycle::ConsumeBeforeFK(g_cycleWorker.state, id, aTicks,
                g_qpcHz.load(std::memory_order_relaxed) / 2,
                g_hot.perspectiveWrite.Allows(aTicks,
                    g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed)));
            // Mapping owner: the core writers of the main mapping job of this cycle belong to this call's record.
            if (early.writers && early.cycle == event.cycle.cycle &&
                event.cycle.status != mm::mappingcycle::ConsumeStatus::MissingMain &&
                event.cycle.status != mm::mappingcycle::ConsumeStatus::IdentityMismatch &&
                earlyruntime::CurrentOwner() == earlyruntime::Owner::Mapping)
                mm::corewrite::MarkEarly(early.writers);
        }
        if (event.cycleConfigured && bodyhelperruntime::Enabled() && !ragdoll::GameOwned())
            bodyhelperruntime::Evaluate(desc, reinterpret_cast<uint8_t*>(event.bones), event.count,
                event.rig, event.cycle, aTicks);
        g_fk.fetch_add(1, std::memory_order_relaxed);
        if (g_enabled.load(std::memory_order_relaxed)) Store(event);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        mm::mappingcycle::Reset(g_cycleWorker.state);
        bodyhelperruntime::Disable();
        g_faultCode.store(GetExceptionCode(), std::memory_order_relaxed);
        g_faults.fetch_add(1, std::memory_order_relaxed);
        g_enabled.store(false, std::memory_order_release);
        g_semantic.Stop();
    }
}
} // namespace posediag
} // namespace utpp

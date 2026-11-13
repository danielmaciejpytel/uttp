#pragma once

#include <cstddef>
#include <cstdint>
#include "mm/SemanticPoseDiscovery.hpp"
#include "mm/MappingByteReceipt.hpp"
#include "mm/MappingCycleReceipt.hpp"

namespace utpp
{
// Cyberpunk 2077 2.50 RVA 0x1B3C08. Win64 arguments established by the local
// decompile and the MIT cyberpunk-vr-port AnimPose.cpp; preserve RAX unchanged.
using Mapping_t = void* (*)(void*, void*, void*, unsigned int);

void* MappingDetour(void* aGraph, void* aContext, void* aSource, unsigned int aLod);
// Mapping owner (EarlyPoseRuntime) can perform an explicitly configured early
// write after original mapping; diagnostic census remains a separate opt-in.
Mapping_t* MappingOriginalSlot();

namespace posediag
{
constexpr size_t kCensusCapacity = 128;
enum class EventKind : uint32_t { MappingAfter = 1, BeforeFK = 2 };

struct Event
{
    uint64_t sequence = 0;
    int64_t ticks = 0;
    // MappingAfter: rig is source animRig resource; BeforeFK: merged animMetaRig.
    uint64_t desc = 0, bones = 0, tag = 0, rig = 0;
    uint64_t graph = 0, source = 0, sourceBones = 0;
    uint32_t count = 0, lod = 0, thread = 0;
    EventKind kind = EventKind::MappingAfter;
    bool rootRig = false;
    // Offsets for these fields have NOT been independently established locally.
    // Zero plus known=false is intentional, not a claimed graph identity.
    uint64_t graphHash = 0;
    uint32_t sourceCount = 0;
    bool graphHashKnown = false, sourceCountKnown = false;
    // Mapping table length for this LOD, NOT the source rig bone count.
    uint32_t mappedBonePairsCount = 0;
    uint64_t mappedBonePairsFingerprint = 0;
    bool mappedBonePairsKnown = false;
    // Pure native observation, before early MM writes; not a frame/freshness token.
    mm::mappingreceipt::Result nativeBytes;
    mm::mappingcycle::Receipt cycle;
    uint32_t cycleObservation = 0;
    bool cycleConfigured = false;
};

struct Counters
{
    bool enabled = false;
    uint64_t mappingCalls = 0, playerPasses = 0, rootPasses = 0;
    uint64_t fkPasses = 0, stored = 0, contentionDrops = 0, readFaults = 0;
    uint32_t faultCode = 0;
    uint64_t semanticStored = 0, semanticDuplicate = 0, semanticInvalid = 0;
    uint64_t semanticBusy = 0, semanticFull = 0, semanticStale = 0;
    uint64_t semanticGeneration = 0, utcFileTime = 0;
    int64_t startTicks = 0, qpcHz = 0;
};

// Diagnostic only: default OFF; no pose or snapshot mutation. Counters are
// cumulative. The ring retains the latest events, including across toggles.
void SetEnabled(bool aEnabled);
// Cold arm path, while disarmed. Full helper catalog; diagnostic only.
bool PrepareCycleCensus(uint64_t mergedRig, std::span<const uint64_t> destinationNames,
                        std::span<const uint64_t> requiredHelpers);
Counters ReadCounters();
size_t CopyCensus(Event* aOut, size_t aCapacity); // game thread; sort by sequence outside callback
bool CopySemantic(uint32_t aIndex, mm::semanticpose::Record& aOut);
bool SemanticCount(uint32_t& aOut);
void NoteBeforeFK(void* aDesc, void* aRig, int64_t aTicks);
} // namespace posediag
} // namespace utpp

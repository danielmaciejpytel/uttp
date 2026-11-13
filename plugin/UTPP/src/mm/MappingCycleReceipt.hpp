#pragma once

#include "MappingByteReceipt.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// Read-only evidence that a main mapping was followed by native mapping writes
// for every required destination helper. This is not an engine frame token and
// never grants permission to modify the pose.
namespace utpp::mm::mappingcycle
{
inline constexpr uint32_t kMaxBones = 1024;
inline constexpr uint32_t kMaxAnchors = 5;

enum class Error : uint32_t { None, Bounds, Catalog, Duplicate, MissingAnchor };
struct Plan
{
    uint64_t generation = 0;
    uint32_t count = 0, requiredCount = 0;
    std::array<uint64_t, kMaxBones> destinationNames{};
    std::array<uint64_t, kMaxBones> requiredNames{};
    std::array<uint16_t, kMaxBones> requiredDestinations{};
    std::array<uint64_t, kMaxAnchors> mainAnchors{};
    bool valid = false;
};

inline Error Prepare(uint64_t generation, std::span<const uint64_t> destinationNames,
                     std::span<const uint64_t> mainAnchors,
                     std::span<const uint64_t> requiredHelpers, Plan& out)
{
    out = Plan{};
    if (!generation || destinationNames.empty() || destinationNames.size() > kMaxBones ||
        mainAnchors.size() != kMaxAnchors || requiredHelpers.empty() ||
        requiredHelpers.size() > kMaxBones)
        return Error::Bounds;

    Plan next;
    next.generation = generation;
    next.count = static_cast<uint32_t>(destinationNames.size());
    next.requiredCount = static_cast<uint32_t>(requiredHelpers.size());
    for (uint32_t i = 0; i < next.count; ++i)
    {
        const uint64_t name = destinationNames[i];
        if (!name) return Error::Catalog;
        if (std::find(destinationNames.begin(), destinationNames.begin() + i, name) !=
            destinationNames.begin() + i)
            return Error::Duplicate;
        next.destinationNames[i] = name;
    }
    for (uint32_t i = 0; i < kMaxAnchors; ++i)
    {
        const uint64_t anchor = mainAnchors[i];
        if (!anchor || std::find(mainAnchors.begin(), mainAnchors.begin() + i, anchor) !=
            mainAnchors.begin() + i)
            return Error::Catalog;
        if (std::find(destinationNames.begin(), destinationNames.end(), anchor) == destinationNames.end())
            return Error::MissingAnchor;
        next.mainAnchors[i] = anchor;
    }
    for (uint32_t i = 0; i < next.requiredCount; ++i)
    {
        const uint64_t helper = requiredHelpers[i];
        if (!helper || std::find(requiredHelpers.begin(), requiredHelpers.begin() + i, helper) !=
            requiredHelpers.begin() + i)
            return Error::Duplicate;
        if (std::find(mainAnchors.begin(), mainAnchors.end(), helper) != mainAnchors.end())
            return Error::Catalog;
        const auto it = std::find(destinationNames.begin(), destinationNames.end(), helper);
        if (it == destinationNames.end()) return Error::Catalog;
        next.requiredNames[i] = helper;
        next.requiredDestinations[i] = static_cast<uint16_t>(it - destinationNames.begin());
    }
    next.valid = true;
    out = next;
    return Error::None;
}

struct Identity
{
    uint64_t generation = 0, desc = 0, bones = 0, tag = 0, mergedRig = 0;
    uint32_t count = 0, thread = 0, lod = 0;
};
struct FKIdentity
{
    uint64_t generation = 0, desc = 0, bones = 0, tag = 0, mergedRig = 0;
    uint32_t count = 0, thread = 0;
};

enum class ObserveResult : uint32_t { MainStarted, HelpersUpdated, Ignored, Rejected };
enum class ConsumeStatus : uint32_t { Complete, Denied, Incomplete, MissingMain, IdentityMismatch, Stale };
struct Receipt
{
    ConsumeStatus status = ConsumeStatus::MissingMain;
    uint64_t generation = 0, cycle = 0;
    int64_t startedTicks = 0, lastTicks = 0;
    uint32_t required = 0, covered = 0, equal = 0;
    bool complete = false;
};
struct State
{
    Identity identity{};
    int64_t startedTicks = 0, lastTicks = 0;
    uint64_t observerSequence = 0, cycle = 0;
    uint32_t requiredCount = 0;
    std::array<bool, kMaxBones> observed{}, equal{};
    bool active = false;
};

inline bool Same(const Identity& a, const Identity& b)
{
    return a.generation == b.generation && a.desc == b.desc && a.bones == b.bones &&
        a.tag == b.tag && a.mergedRig == b.mergedRig && a.count == b.count &&
        a.thread == b.thread && a.lod == b.lod;
}
inline bool Same(const Identity& a, const FKIdentity& b)
{
    return a.generation == b.generation && a.desc == b.desc && a.bones == b.bones &&
        a.tag == b.tag && a.mergedRig == b.mergedRig && a.count == b.count &&
        a.thread == b.thread;
}
inline bool Has(std::span<const uint64_t> names, uint64_t value)
{
    return std::find(names.begin(), names.end(), value) != names.end();
}
inline void Reset(State& state)
{
    const uint64_t sequence = state.observerSequence;
    state = State{};
    state.observerSequence = sequence;
}

// Call after the native Mapping returns and before any mod pose write. The
// caller owns the SEH boundary for spans that reference live game memory.
inline ObserveResult Observe(const Plan& plan, State& state, const Identity& identity,
                             int64_t ticks, int64_t maxAge,
                             std::span<const uint64_t> sourceNames,
                             std::span<const uint32_t> pairs,
                             const mappingreceipt::Result& bytes)
{
    if (!plan.valid || identity.generation != plan.generation || !identity.desc || !identity.bones ||
        !identity.tag || !identity.mergedRig || identity.count != plan.count || !identity.thread ||
        identity.lod > 7 || ticks < 0 || sourceNames.empty() || sourceNames.size() > kMaxBones ||
        pairs.empty() || (pairs.size() & 1u) || pairs.size() / 2 > kMaxBones ||
        !bytes.valid || bytes.overlap || bytes.compared != pairs.size() / 2 || bytes.equal > bytes.compared)
    {
        Reset(state);
        return ObserveResult::Rejected;
    }
    for (size_t i = 0; i < pairs.size(); i += 2)
        if (pairs[i] >= sourceNames.size() || pairs[i + 1] >= plan.count)
        {
            Reset(state);
            return ObserveResult::Rejected;
        }

    bool hasMain = true;
    for (uint64_t anchor : plan.mainAnchors)
        hasMain = hasMain && Has(sourceNames, anchor);
    bool hasHelper = false;
    for (uint32_t h = 0; h < plan.requiredCount; ++h)
        hasHelper = hasHelper || Has(sourceNames, plan.requiredNames[h]);
    bool touchesRequiredDestination = false;
    for (size_t p = 0; p < pairs.size(); p += 2)
        for (uint32_t h = 0; h < plan.requiredCount; ++h)
            touchesRequiredDestination = touchesRequiredDestination ||
                pairs[p + 1] == plan.requiredDestinations[h];
    if (hasMain && (hasHelper || touchesRequiredDestination))
    {
        Reset(state);
        return ObserveResult::Rejected;
    }
    if (hasMain)
    {
        for (uint64_t anchor : plan.mainAnchors)
        {
            bool mapped = false;
            for (size_t p = 0; p < pairs.size(); p += 2)
                mapped = mapped || (sourceNames[pairs[p]] == anchor &&
                                    plan.destinationNames[pairs[p + 1]] == anchor);
            if (!mapped)
            {
                Reset(state);
                return ObserveResult::Rejected;
            }
        }
        Reset(state);
        ++state.observerSequence;
        if (!state.observerSequence) ++state.observerSequence;
        state.cycle = state.observerSequence;
        state.identity = identity;
        state.startedTicks = state.lastTicks = ticks;
        state.requiredCount = plan.requiredCount;
        state.active = true;
        return ObserveResult::MainStarted;
    }
    // Once a main pass has begun, an interleaved mapping for a different
    // descriptor/buffer/thread or an out-of-order/stale event invalidates the
    // pending evidence, even if that mapping is not a helper pass.
    if (state.active && (!Same(state.identity, identity) || ticks < state.lastTicks ||
        (maxAge > 0 && (ticks < state.startedTicks || ticks - state.startedTicks > maxAge))))
    {
        Reset(state);
        return ObserveResult::Rejected;
    }
    if (!hasHelper && !touchesRequiredDestination) return ObserveResult::Ignored;
    if (!state.active) return ObserveResult::Rejected;

    // Process in mapping-table order: repeated destinations use last-writer
    // semantics, so a later unequal/wrong-name write clears earlier equality.
    for (size_t p = 0; p < pairs.size(); p += 2)
    {
        const uint32_t src = pairs[p], dst = pairs[p + 1];
        for (uint32_t h = 0; h < plan.requiredCount; ++h)
        {
            if (dst != plan.requiredDestinations[h]) continue;
            state.observed[h] = true;
            const bool namesMatch = sourceNames[src] == plan.destinationNames[dst];
            const bool equal = (bytes.equalPairs[(p / 2) / 64] & (uint64_t{1} << ((p / 2) % 64))) != 0;
            state.equal[h] = namesMatch && equal;
        }
    }
    state.lastTicks = ticks;
    return ObserveResult::HelpersUpdated;
}

// Every FK call consumes and clears this thread's pending evidence, including
// denied, incomplete, stale and identity-mismatched calls. This is telemetry,
// not proof of an engine frame boundary and not permission to write.
inline Receipt ConsumeBeforeFK(State& state, const FKIdentity& identity,
                               int64_t ticks, int64_t maxAge, bool perspectiveAllowed)
{
    Receipt result;
    result.generation = state.active ? state.identity.generation : 0;
    result.cycle = state.cycle;
    result.startedTicks = state.startedTicks;
    result.lastTicks = state.lastTicks;
    if (state.active)
    {
        for (uint32_t i = 0; i < kMaxBones; ++i)
        {
            result.covered += state.observed[i] ? 1u : 0u;
            result.equal += state.equal[i] ? 1u : 0u;
        }
        result.required = state.requiredCount;
    }
    result.complete = state.active && result.covered == result.required && result.equal == result.required;
    if (!state.active) result.status = ConsumeStatus::MissingMain;
    else if (!Same(state.identity, identity)) result.status = ConsumeStatus::IdentityMismatch;
    else if (ticks < state.startedTicks || ticks < state.lastTicks ||
             (maxAge > 0 && ticks - state.startedTicks > maxAge)) result.status = ConsumeStatus::Stale;
    else if (!perspectiveAllowed) result.status = ConsumeStatus::Denied;
    else if (!result.complete) result.status = ConsumeStatus::Incomplete;
    else result.status = ConsumeStatus::Complete;
    Reset(state);
    return result;
}
} // namespace utpp::mm::mappingcycle

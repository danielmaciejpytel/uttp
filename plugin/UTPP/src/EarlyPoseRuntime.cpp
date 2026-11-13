// Experimental early owner, opt-in and fail-closed. This file does not install
// a hook, change its owner automatically, mutate helper bones, or write poses
// without the caller's explicit apply callback.
#include "EarlyPoseRuntime.hpp"
#include "State.hpp"

#include <algorithm>
#include <array>
#include <atomic>

namespace utpp::earlyruntime
{
namespace
{
struct Policy
{
    uint64_t generation = 0, mergedRig = 0, trackA = 0, trackB = 0;
    uint32_t count = 0, helpers = 0;
    std::array<uint64_t, kMaxRigBones> destinationNames{};
    std::array<uint64_t, kMaxRigBones> forbiddenHelpers{};
};
mm::integ::SeqlockBuffer<Policy> g_policy;
std::atomic<Owner> g_owner{Owner::FK};
std::atomic<bool> g_ready{false};
std::atomic<uint64_t> g_generation{0};
std::atomic<uint64_t> g_calls{0}, g_notReady{0}, g_identity{0}, g_pose{0};
std::atomic<uint64_t> g_semantic{0}, g_perspective{0}, g_applied{0}, g_callbackNoWrite{0};

bool TagMatches(uint64_t tag, uint64_t a, uint64_t b) { return tag && (tag == a || tag == b); }
Result Reject(std::atomic<uint64_t>& counter, Result result)
{
    counter.fetch_add(1, std::memory_order_relaxed);
    return result;
}
bool HasHash(std::span<const uint64_t> names, uint64_t hash)
{
    return std::find(names.begin(), names.end(), hash) != names.end();
}

// These five mapped core names are present in the proven main source, absent
// from the 67-common-bone deformation source. No source bone-count constant is
// used: live-source-name-proof.json established the name sets, not graph paths.
const std::array<uint64_t, 5> kMainAnchors{
    mm::integ::CNameHash("reference_joint"), mm::integ::CNameHash("WeaponLeft"),
    mm::integ::CNameHash("WeaponRight"), mm::integ::CNameHash("LeftEye"), mm::integ::CNameHash("RightEye")};
} // namespace

bool SelectOwner(Owner owner)
{
    if (g_hot.armed.load(std::memory_order_acquire) || (owner != Owner::FK && owner != Owner::Mapping)) return false;
    g_owner.store(owner, std::memory_order_release);
    return true;
}
Owner CurrentOwner() { return g_owner.load(std::memory_order_acquire); }

bool PrepareArm(uint64_t mergedRig, uint32_t count, uint64_t trackA, uint64_t trackB,
                std::span<const uint64_t> destinationNames, std::span<const uint64_t> forbiddenHelpers)
{
    if (g_hot.armed.load(std::memory_order_acquire)) return false;
    g_ready.store(false, std::memory_order_release);
    if (!mergedRig || !count || count > kMaxRigBones || destinationNames.size() != count ||
        !(trackA || trackB) || forbiddenHelpers.empty() || forbiddenHelpers.size() > kMaxRigBones)
        return false;
    Policy policy;
    policy.generation = g_generation.fetch_add(1, std::memory_order_acq_rel) + 1;
    policy.mergedRig = mergedRig; policy.count = count;
    policy.trackA = trackA; policy.trackB = trackB;
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint64_t hash = destinationNames[i];
        if (!hash || std::find(destinationNames.begin(), destinationNames.begin() + i, hash) != destinationNames.begin() + i)
            return false; // ambiguous destination names cannot define a write policy
        policy.destinationNames[i] = hash;
    }
    for (uint64_t anchor : kMainAnchors)
        if (!HasHash(destinationNames, anchor)) return false;
    // The complete destination-name table above is unique. Requiring each
    // helper to occur there proves exactly one destination association and
    // prevents an unrelated catalog from silently passing source exclusion.
    for (uint64_t helper : forbiddenHelpers)
        if (!helper || !HasHash(destinationNames, helper)) return false;
    std::copy(forbiddenHelpers.begin(), forbiddenHelpers.end(), policy.forbiddenHelpers.begin());
    policy.helpers = static_cast<uint32_t>(forbiddenHelpers.size());
    std::sort(policy.forbiddenHelpers.begin(), policy.forbiddenHelpers.begin() + policy.helpers);
    if (policy.forbiddenHelpers[0] == 0) return false;
    if (std::adjacent_find(policy.forbiddenHelpers.begin(), policy.forbiddenHelpers.begin() + policy.helpers) !=
        policy.forbiddenHelpers.begin() + policy.helpers) return false;
    for (uint64_t anchor : kMainAnchors)
        if (std::binary_search(policy.forbiddenHelpers.begin(), policy.forbiddenHelpers.begin() + policy.helpers, anchor))
            return false;
    g_policy.Publish(policy);
    g_ready.store(true, std::memory_order_release);
    return true;
}

void Disarm()
{
    g_ready.store(false, std::memory_order_release);
    g_generation.fetch_add(1, std::memory_order_acq_rel);
}

Result EvaluateMapping(const MappingView& mapping, const mm::integ::PoseSnapshot& snap,
                       int64_t now, Apply apply, void* user)
{
    if (CurrentOwner() != Owner::Mapping) return Result::WrongOwner;
    g_calls.fetch_add(1, std::memory_order_relaxed);
    if (!g_ready.load(std::memory_order_acquire) || !g_hot.armed.load(std::memory_order_acquire))
        return Reject(g_notReady, Result::NotReady);
    Policy policy;
    if (!g_policy.Read(policy) || policy.generation != g_generation.load(std::memory_order_acquire))
        return Reject(g_notReady, Result::NotReady);
    if (!mapping.desc || !mapping.bones || !mapping.sourceRig || mapping.outputCount != policy.count ||
        g_hot.playerRig.load(std::memory_order_acquire) != policy.mergedRig ||
        g_hot.playerBones.load(std::memory_order_acquire) != policy.count ||
        !TagMatches(mapping.tag, policy.trackA, policy.trackB) ||
        !TagMatches(mapping.tag, g_hot.trackA.load(std::memory_order_relaxed), g_hot.trackB.load(std::memory_order_relaxed)))
        return Reject(g_identity, Result::IdentityRejected);
    if (!g_hot.mmActive.load(std::memory_order_acquire) || !apply || !(snap.weight > 0.0f) ||
        snap.rig != policy.mergedRig || snap.numBones > mm::integ::kMaxMMBones ||
        snap.firstBone >= snap.numBones || now < snap.publishTicks)
        return Reject(g_pose, Result::PoseRejected);
    const int64_t maxAge = g_hot.mmStaleTicks.load(std::memory_order_relaxed);
    if (maxAge > 0 && now - snap.publishTicks > maxAge) return Reject(g_pose, Result::PoseRejected);
    if (!g_hot.perspectiveWrite.Allows(now, g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed)))
        return Reject(g_perspective, Result::PerspectiveRejected);
    if (mapping.sourceNames.empty() || mapping.sourceNames.size() > kMaxRigBones ||
        mapping.pairs.empty() || (mapping.pairs.size() & 1u) || mapping.pairs.size() > kMaxRigBones * 2u)
        return Reject(g_semantic, Result::SemanticRejected);
    for (uint64_t hash : mapping.sourceNames)
        if (std::binary_search(policy.forbiddenHelpers.begin(), policy.forbiddenHelpers.begin() + policy.helpers, hash))
            return Reject(g_semantic, Result::SemanticRejected);
    for (uint64_t anchor : kMainAnchors)
        if (!HasHash(mapping.sourceNames, anchor)) return Reject(g_semantic, Result::SemanticRejected);

    std::array<int16_t, kMaxRigBones> coreAtDestination;
    coreAtDestination.fill(-1);
    std::array<bool, mm::integ::kMaxMMBones> covered{};
    for (uint32_t b = snap.firstBone; b < snap.numBones; ++b)
    {
        const int32_t dst = snap.gameIndex[b];
        if (dst < 0 || static_cast<uint32_t>(dst) >= policy.count || coreAtDestination[static_cast<size_t>(dst)] >= 0)
            return Reject(g_semantic, Result::SemanticRejected);
        const uint64_t name = policy.destinationNames[static_cast<size_t>(dst)];
        if (std::binary_search(policy.forbiddenHelpers.begin(), policy.forbiddenHelpers.begin() + policy.helpers, name))
            return Reject(g_semantic, Result::SemanticRejected);
        coreAtDestination[static_cast<size_t>(dst)] = static_cast<int16_t>(b);
    }
    std::array<uint64_t, kMaxRigBones / 64> destinationsSeen{};
    for (size_t i = 0; i < mapping.pairs.size(); i += 2)
    {
        const uint32_t src = mapping.pairs[i], dst = mapping.pairs[i + 1];
        if (src >= mapping.sourceNames.size() || dst >= policy.count ||
            mapping.sourceNames[src] != policy.destinationNames[dst])
            return Reject(g_semantic, Result::SemanticRejected);
        const uint64_t bit = uint64_t{1} << (dst & 63u);
        if (destinationsSeen[dst >> 6u] & bit) return Reject(g_semantic, Result::SemanticRejected);
        destinationsSeen[dst >> 6u] |= bit;
        if (coreAtDestination[dst] >= 0) covered[static_cast<size_t>(coreAtDestination[dst])] = true;
    }
    for (uint32_t b = snap.firstBone; b < snap.numBones; ++b)
        if (!covered[b]) return Reject(g_semantic, Result::SemanticRejected);

    // Recheck camera/arming after semantic reads. Every admitted invocation is
    // post-original-mapping and that original has just overwritten ALL core
    // records, so repeated jobs start from fresh game data instead of blending
    // our previous result a second time. FK owner must stay disabled throughout.
    if (!g_ready.load(std::memory_order_acquire) || policy.generation != g_generation.load(std::memory_order_acquire) ||
        !g_hot.armed.load(std::memory_order_acquire)) return Reject(g_notReady, Result::NotReady);
    if (!g_hot.perspectiveWrite.Allows(now, g_hot.perspectiveWriteMaxAge.load(std::memory_order_relaxed)))
        return Reject(g_perspective, Result::PerspectiveRejected);
    const uint32_t written = apply(user, mapping.desc, mapping.bones, mapping.outputCount, policy.mergedRig, snap, now);
    if (!written) return Reject(g_callbackNoWrite, Result::CallbackNoWrite);
    g_applied.fetch_add(1, std::memory_order_relaxed);
    return Result::Applied;
}

Counters ReadCounters()
{
    return {g_calls.load(), g_notReady.load(), g_identity.load(), g_pose.load(),
            g_semantic.load(), g_perspective.load(), g_applied.load(), g_callbackNoWrite.load()};
}
} // namespace utpp::earlyruntime

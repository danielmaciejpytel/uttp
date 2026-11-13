// UTPP ragdoll production core: pure decisions of the slot probe / pelvis telemetry (plain C++, no Windows / RED4ext, unit-tested in
// tests/RagdollEpisodeTest.cpp). RagdollSlotProbe.cpp only does the game-memory reads (SEH) and calls these.
//
//  - CanBindSlotCache: the production slot cache (index + component of V's pose slot, which feeds the episode gate) may only ever be
//    bound to a component whose owner is the KNOWN player. The research probe UTPPRagdollSlotDump used to bind any component the
//    script passed, so an NPC's ragdoll could become "V's gate" while the player id was still unknown.
//  - ResolvePelvisIndices / PelvisRigCache: the pelvis telemetry reads Hips / Spine / LeftUpLeg / RightUpLeg by bone index. V has
//    several output descriptors with different rigs (619 / 645 bones in game); the indices resolved on the merged rig are not valid
//    for another rig, so each new rig is resolved by bone name once and cached.
#pragma once

#include <atomic>
#include <cstdint>

#ifndef UTPP_RAGDOLL_EPISODE_MUTATION
#define UTPP_RAGDOLL_EPISODE_MUTATION 0
#endif

namespace utpp::ragdoll
{
// aPlayerHash = the player's EntityID hash (0 = unknown), aOwnerHash = owner entity hash of the candidate component (0 = unreadable).
inline bool CanBindSlotCache(uint64_t aPlayerHash, uint64_t aOwnerHash)
{
#if UTPP_RAGDOLL_EPISODE_MUTATION == 6
    (void)aPlayerHash; (void)aOwnerHash;
    return true; // mutation 6: any component the probe is handed becomes the production gate
#else
    return aPlayerHash != 0 && aOwnerHash == aPlayerHash;
#endif
}

constexpr uint32_t kPelvisBones = 4; // Hips, Spine, LeftUpLeg, RightUpLeg
constexpr uint32_t kMaxRigNames = 8192;

// First index of each wanted name hash in the array aNames[aCount]; false (aOut = -1 where missing) when any of the four is absent.
inline bool ResolvePelvisIndices(const uint64_t* aNames, uint32_t aCount, const uint64_t (&aWanted)[kPelvisBones],
                                 int32_t (&aOut)[kPelvisBones])
{
    bool all = aNames && aCount > 0 && aCount <= kMaxRigNames;
    for (uint32_t w = 0; w < kPelvisBones; ++w)
    {
        aOut[w] = -1;
        if (!all)
            continue;
        for (uint32_t i = 0; i < aCount; ++i)
        {
            if (aNames[i] == aWanted[w] && aWanted[w] != 0)
            {
                aOut[w] = static_cast<int32_t>(i);
                break;
            }
        }
        if (aOut[w] < 0)
            all = false;
    }
    return all;
}

// Small cache of the per-rig result, keyed by (rig pointer, descriptor bone count). Unresolved rigs are cached too (negative), so a
// rig that cannot be resolved costs one attempt, not one per frame. Any thread; a contended lookup returns Busy (the caller drops that
// sample), nothing ever waits. Clear() on every arm / disarm (rig pointers are reused after a rig is freed).
class PelvisRigCache
{
public:
    static constexpr uint32_t kEntries = 4;
    enum class Result { Ok, Unresolved, Busy };

    // aResolve(aRigKey, aCount, int32_t (&aOut)[4]) -> bool, called under the lock once per new (rig, count).
    template<typename Resolve>
    Result Lookup(uint64_t aRig, uint32_t aCount, Resolve&& aResolve, int32_t (&aOut)[kPelvisBones])
    {
        if (!aRig)
            return Result::Unresolved;
        if (m_lock.test_and_set(std::memory_order_acquire))
            return Result::Busy;
#if UTPP_RAGDOLL_EPISODE_MUTATION == 7
        // mutation 7: the first resolved rig's indices are reused for every other rig (the pre-fix global indices)
        for (uint32_t i = 0; i < kEntries; ++i)
        {
            if (m_e[i].used && m_e[i].ok)
            {
                for (uint32_t k = 0; k < kPelvisBones; ++k)
                    aOut[k] = m_e[i].idx[k];
                m_lock.clear(std::memory_order_release);
                return Result::Ok;
            }
        }
#endif
        Entry* hit = nullptr;
        for (uint32_t i = 0; i < kEntries; ++i)
            if (m_e[i].used && m_e[i].rig == aRig && m_e[i].count == aCount)
                hit = &m_e[i];
        if (!hit)
        {
            hit = &m_e[m_next++ % kEntries];
            hit->used = true;
            hit->rig = aRig;
            hit->count = aCount;
            int32_t idx[kPelvisBones];
            hit->ok = aResolve(aRig, aCount, idx);
            for (uint32_t k = 0; k < kPelvisBones; ++k)
                hit->idx[k] = hit->ok ? idx[k] : -1;
            m_resolutions.fetch_add(1, std::memory_order_relaxed);
        }
        const bool ok = hit->ok;
        if (ok)
            for (uint32_t k = 0; k < kPelvisBones; ++k)
                aOut[k] = hit->idx[k];
        m_lock.clear(std::memory_order_release);
        return ok ? Result::Ok : Result::Unresolved;
    }

    void Clear()
    {
        while (m_lock.test_and_set(std::memory_order_acquire))
        {
        } // the holder is a bounded lookup; Clear runs on the game thread at arm time, never in a hot path
        for (Entry& e : m_e)
            e = Entry{};
        m_next = 0;
        m_lock.clear(std::memory_order_release);
    }
    uint64_t Resolutions() const { return m_resolutions.load(std::memory_order_relaxed); }

private:
    struct Entry
    {
        bool used = false, ok = false;
        uint64_t rig = 0;
        uint32_t count = 0;
        int32_t idx[kPelvisBones] = {-1, -1, -1, -1};
    };
    std::atomic_flag m_lock = {};
    Entry m_e[kEntries];
    uint32_t m_next = 0;
    std::atomic<uint64_t> m_resolutions{0};
};
} // namespace utpp::ragdoll

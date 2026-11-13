// UTPP ragdoll R1o probe (read-only): where V's rendered pose loses the ragdoll overlay. Two small tables filled from the
// hot paths with atomics only (no locks, no allocation) and logged on the game thread by UTPPRagdollSlotDump, one line per row:
//   map - one row per source rig the mapping function (RVA 0x1B3C08) merges into a player-tagged output descriptor: the
//         merged buffer's Hips translation and Hips / Head / LeftFoot local rotation right after that merge, plus the graph
//         entry fields named by the analysis notes (+0x18, +0x30, +0x25a0,
//         +0x25b1, +0x25c8 and the graph hash at *(+0x20)+0x30; whether aGraph is that entry is itself under test);
//   fk  - one row per descriptor FK (RVA 0x582B3C) receives with the player tag or the player's merged rig: count, rig,
//         Hips / Head / LeftFoot model-space position after FK.
// R1p: every row also keeps the descriptor's Hips translation just before the call and a global call sequence number.
#pragma once

#include <atomic>
#include <cstdint>

namespace utpp::rdpose
{
constexpr int kRows = 6;

struct Row
{
    std::atomic<uint64_t> key{0}; // map: source rig; fk: descriptor
    std::atomic<uint64_t> a{0};   // map: output descriptor; fk: rig
    std::atomic<uint64_t> b{0};   // map: graph (aGraph); fk: tag
    std::atomic<uint32_t> count{0}; // map: source rig bones; fk: descriptor count
    std::atomic<uint32_t> aux{0};   // map: output count; fk: 1 once seen as the matched player call
    std::atomic<uint64_t> calls{0};
    std::atomic<uint64_t> seq{0};   // g_seq at the latest captured call (orders merges and FK within a frame)
    std::atomic<float> pre[4];      // Hips translation in the descriptor just before the call
    std::atomic<float> v[4][4];     // map: Hips pos, Hips / Head / LeftFoot rot; fk: Hips / Head / LeftFoot pos (v[3] unused)
    std::atomic<uint64_t> f[6];     // map only: +0x18, +0x30, +0x25a0, +0x25b1 (byte), +0x25c8, graph hash
};

inline std::atomic<uint64_t> g_seq{0};
inline Row g_map[kRows];
inline Row g_fk[kRows];
inline std::atomic<int32_t> g_idx[3] = {{-1}, {-1}, {-1}}; // merged-rig Hips, Head, LeftFoot (set at arm time)

// (The R1q keep counters moved to ragdoll/RdKeep.hpp.)

// Row for aKey, claiming a free one; nullptr when the table is full.
inline Row* Find(Row (&aTable)[kRows], uint64_t aKey)
{
    for (auto& r : aTable)
    {
        uint64_t k = r.key.load(std::memory_order_relaxed);
        if (k == aKey)
            return &r;
        if (k == 0)
        {
            if (r.key.compare_exchange_strong(k, aKey, std::memory_order_relaxed) || k == aKey)
                return &r;
        }
    }
    return nullptr;
}

inline void Store4(std::atomic<float> (&aOut)[4], const float* aIn)
{
    for (int j = 0; j < 4; ++j)
        aOut[j].store(aIn[j], std::memory_order_relaxed);
}
} // namespace utpp::rdpose

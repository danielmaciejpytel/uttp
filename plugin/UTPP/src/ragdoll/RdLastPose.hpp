// UTPP ragdoll get-up: the last ragdoll pose of V (plain C++, no Windows / RED4ext, unit-tested in tests/RagdollGetUpTest.cpp).
//
// The animation worker saves the destination records of the player_base merge while the ragdoll overlay is on (PoseHook.cpp
// RdKeepApply, RdKeep.hpp). This store keeps a copy of the latest save so that the game thread can read it when the ragdoll is
// released (the get-up starts from that pose). RdKeep slots are only valid inside one evaluation and cannot be read from the game
// thread; this copy can.
//
// Concurrency: one writer at a time (try-lock, a contended write is dropped, nothing ever waits), any number of readers (seqlock,
// at most 3 attempts, a torn read fails). Write and Read never allocate. The payload is a fixed array (about 13 KB).
//   - full-body rule: the descriptor with the largest record count wins; a smaller save never overwrites it until Clear().
//   - Freeze(true): Write becomes a no-op (the pre-release pose is kept while the disable event is still on its way).
//   - Clear(): empties the store, resets the largest-count rule and lifts the freeze (a new episode starts empty).
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

#ifndef UTPP_GETUP_MUTATION
#define UTPP_GETUP_MUTATION 0
#endif

namespace utpp::ragdoll
{
class RdLastPose
{
public:
    static constexpr uint32_t kMaxBones = 256; // = RdKeep::kMaxBones
    static constexpr uint32_t kStride = 48;    // bytes per descriptor bone record (pos float4, quat, scale float4)

    struct Snapshot
    {
        uint64_t desc = 0;       // output descriptor pointer the records came from
        uint32_t n = 0;          // records
        uint32_t descCount = 0;  // bones of that descriptor (0 = not recorded); informational
        int64_t ticks = 0;       // QPC time of the save
        uint32_t dst[kMaxBones];
        float rec[kMaxBones][12]; // pos xyzw, quat xyzw, scale xyzw
    };

    // aData: n records of kStride bytes. Returns true when the copy was stored.
    bool Write(uint64_t aDesc, const uint32_t* aDst, const uint8_t (*aData)[kStride], uint32_t aN, int64_t aTicks,
               uint32_t aDescCount = 0)
    {
        if (aN == 0 || aN > kMaxBones || aDst == nullptr || aData == nullptr)
            return false;
        if (m_frozen.load(std::memory_order_acquire))
            return false;
#if UTPP_GETUP_MUTATION == 6
        // mutation 6: a smaller descriptor overwrites the full-body one
#else
        if (aN < m_bestN.load(std::memory_order_relaxed))
            return false;
#endif
        if (m_writer.test_and_set(std::memory_order_acquire))
            return false; // contended: dropped, never waited for
        const uint32_t epoch = m_epoch.load(std::memory_order_acquire); // a Clear() during this write discards it
        m_seq.fetch_add(1, std::memory_order_acq_rel); // odd: writing
        m_snap.desc = aDesc;
        m_snap.n = aN;
        m_snap.descCount = aDescCount;
        m_snap.ticks = aTicks;
        std::memcpy(m_snap.dst, aDst, size_t(aN) * sizeof(uint32_t));
        for (uint32_t i = 0; i < aN; ++i)
            std::memcpy(m_snap.rec[i], aData[i], kStride);
        m_seq.fetch_add(1, std::memory_order_release); // even: done
        if (m_epoch.load(std::memory_order_acquire) != epoch)
        {
            m_writer.clear(std::memory_order_release); // cleared while writing: nothing of this write is published
            return false;
        }
        m_ticks.store(aTicks, std::memory_order_relaxed);
        m_bestN.store(aN, std::memory_order_relaxed);
        m_have.store(true, std::memory_order_release);
        m_writer.clear(std::memory_order_release);
        return true;
    }

    void Freeze(bool aOn) { m_frozen.store(aOn, std::memory_order_release); }
    bool Frozen() const { return m_frozen.load(std::memory_order_acquire); }

    // Cheap check without a copy: a save exists and is not older than aMaxAgeS. aN = its record count.
    bool Peek(int64_t aNowTicks, int64_t aHz, double aMaxAgeS, uint32_t& aN) const
    {
        if (!m_have.load(std::memory_order_acquire) || aHz <= 0)
            return false;
        const int64_t t = m_ticks.load(std::memory_order_relaxed);
        const double age = aNowTicks > t ? static_cast<double>(aNowTicks - t) / static_cast<double>(aHz) : 0.0;
        if (!(age <= aMaxAgeS))
            return false;
        aN = m_bestN.load(std::memory_order_relaxed);
        return true;
    }

    // false: nothing stored / older than aMaxAgeS / still torn after a bounded wait. The wait (at most ~200 us, yielding) only happens
    // while a write is in flight (a write copies about 7 KB, a few microseconds); the game thread never blocks longer than that.
    bool Read(Snapshot& aOut, int64_t aNowTicks, int64_t aHz, double aMaxAgeS) const
    {
        if (!m_have.load(std::memory_order_acquire) || aHz <= 0)
            return false;
        std::chrono::steady_clock::time_point deadline{};
        bool haveDeadline = false;
        for (;;)
        {
            const uint32_t s0 = m_seq.load(std::memory_order_acquire);
            if (!(s0 & 1u))
            {
                // header first, then only the n used records (a torn n is clamped; the seq check below rejects the copy anyway)
                aOut.desc = m_snap.desc;
                aOut.descCount = m_snap.descCount;
                aOut.ticks = m_snap.ticks;
                const uint32_t n = std::min(m_snap.n, kMaxBones);
                aOut.n = n;
                std::memcpy(aOut.dst, m_snap.dst, size_t(n) * sizeof(uint32_t));
                std::memcpy(aOut.rec, m_snap.rec, size_t(n) * sizeof(aOut.rec[0]));
                std::atomic_thread_fence(std::memory_order_acquire);
                if (m_seq.load(std::memory_order_relaxed) == s0)
                {
                    if (aOut.n == 0)
                        return false;
                    const double age =
                        aNowTicks > aOut.ticks ? static_cast<double>(aNowTicks - aOut.ticks) / static_cast<double>(aHz) : 0.0;
                    return age <= aMaxAgeS;
                }
            }
            if (!haveDeadline)
            {
                deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(200);
                haveDeadline = true;
            }
            else if (std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::yield();
        }
    }

    // Empties the store (a new episode starts empty), resets the largest-n rule and lifts the freeze. Serialized with an in-flight Write:
    // the epoch makes that write discard itself, and the writer lock (bounded wait) keeps it from publishing over the cleared state.
    void Clear()
    {
        m_epoch.fetch_add(1, std::memory_order_acq_rel);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(200);
        bool locked = !m_writer.test_and_set(std::memory_order_acquire);
        while (!locked && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::yield();
            locked = !m_writer.test_and_set(std::memory_order_acquire);
        }
        m_have.store(false, std::memory_order_release);
        m_bestN.store(0, std::memory_order_relaxed);
        m_frozen.store(false, std::memory_order_release);
        if (locked)
            m_writer.clear(std::memory_order_release);
    }

private:
    std::atomic_flag m_writer = {};
    std::atomic<uint32_t> m_seq{0};
    std::atomic<bool> m_have{false}, m_frozen{false};
    std::atomic<uint32_t> m_bestN{0};
    std::atomic<uint32_t> m_epoch{0}; // bumped by Clear()
    std::atomic<int64_t> m_ticks{0};
    Snapshot m_snap;
};

// The one process-wide store (written by PoseHook on the animation worker, read by motiondriver on the game thread).
inline RdLastPose& LastPose()
{
    static RdLastPose s_lastPose;
    return s_lastPose;
}
} // namespace utpp::ragdoll

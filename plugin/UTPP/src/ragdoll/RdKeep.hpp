// UTPP ragdoll production core: keep store for V's ragdoll pose (plain C++, no Windows / RED4ext, unit-tested in
// tests/RagdollKeepTest.cpp).
//
// Why: while V's RagdollControl overlay (animSys pose slot +0x29) is on, the player_base merge writes the ragdoll into the
// output descriptor, but the later merges into the SAME descriptor (deformations, shadow, face, the 28-bone rig) re-emit V's
// core bones from poses that never carry the overlay. The save stores the destination bones of the player_base merge; every
// later merge into the same descriptor writes them back, so FK sees the ragdoll (experiment R1q, now production).
//
// Rules of the store (all hot-path functions are allocation-free and never wait):
//  - A fixed table of kSlots entries keyed by the output descriptor pointer; V has two tagged tracks (trackA / trackB), they
//    land in different slots and never touch each other.
//  - Each slot has a try-lock (atomic_flag). A hot path that cannot take it skips and counts; it never spins. The lock is held
//    only around the copy, the CALLER does the (SEH-guarded) game-memory copy between Begin* and Commit/End/Abort, so a fault
//    can be turned into Abort* by the caller and the lock is never leaked (header stays free of SEH).
//  - A save opens an "evaluation" for its descriptor: serial = ++counter, valid = true, stamped with the save time and thread.
//    The evaluation is closed by FK of that descriptor (CloseEvaluation), by the next save into the slot, by a fault, or by an
//    age cap (a descriptor whose FK never comes). A restore applies only to a valid, fresh slot of the matching descriptor and,
//    when the caller passes a serial, only to exactly that evaluation. A torn / stale / contended save is never written back:
//    contention poisons the slot until the next committed save.
//  - Instrumentation: thread id of the save and of each restore; restores whose thread differs from the save thread are counted
//    (threadDiff), so the in-game status line shows whether the merges of one descriptor stay on one thread.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#ifndef UTPP_RAGDOLL_KEEP_MUTATION
#define UTPP_RAGDOLL_KEEP_MUTATION 0
#endif

namespace utpp::ragdoll
{
class RdKeep
{
public:
    static constexpr uint32_t kSlots = 4;
    static constexpr uint32_t kMaxBones = 256;
    static constexpr uint32_t kStride = 48; // bytes per descriptor bone record (pos float4, rot quat, scale float4)

    struct Slot
    {
        Slot() { busy.clear(); }
        std::atomic_flag busy;
        std::atomic<uint64_t> key{0};      // output descriptor pointer, 0 = free
        std::atomic<bool> valid{false};    // an open, committed evaluation
        std::atomic<bool> poisoned{false}; // a save or an FK clear lost the try-lock: nothing may be restored until the next commit
        uint64_t serial = 0;
        std::atomic<int64_t> ticks{0}; // save time; read unlocked by BeginSave's reclaim pre-check
        uint32_t thread = 0;
        uint32_t n = 0;
        uint32_t dst[kMaxBones];
        uint8_t data[kMaxBones][kStride];
    };

    struct Counters
    {
        uint64_t saves = 0, restores = 0, skips = 0, faults = 0;
        uint64_t skipNoSlot = 0, skipStale = 0, skipSerial = 0, skipBusy = 0, skipPoison = 0, skipFull = 0;
        uint64_t threadSame = 0, threadDiff = 0;
        uint32_t lastBones = 0, lastSaveThread = 0, lastRestoreThread = 0;
        uint64_t lastSerial = 0;
    };

    // Evaluations older than this (same unit as the 'now' arguments, QPC ticks in the game) are never restored.
    void SetMaxAge(int64_t aTicks) { m_maxAge.store(aTicks, std::memory_order_relaxed); }

    // ---- save side (the player_base merge) ----
    // Locks and returns the slot for aDesc (claiming a free / invalid one), already marked not valid. nullptr: contended or full.
    // A slot of ANOTHER descriptor that is still valid / poisoned is reclaimed once its evaluation is older than the age cap: its FK
    // never came (freed / replaced descriptor, FK not through the detour) or the FK close lost the try-lock, and nothing would ever
    // restore or close it, so it would otherwise hold one of the few slots for the rest of the episode. aNow is the caller's clock
    // (QPC ticks in the game); the one-argument form uses the latest time this store has seen (saves and restores).
    Slot* BeginSave(uint64_t aDesc) { return BeginSave(aDesc, m_now.load(std::memory_order_relaxed)); }
    Slot* BeginSave(uint64_t aDesc, int64_t aNow)
    {
        if (!aDesc)
            return nullptr;
        NoteTime(aNow);
        for (uint32_t i = 0; i < kSlots; ++i) // existing slot of this descriptor first
        {
            Slot& s = m_slots[i];
            if (s.key.load(std::memory_order_acquire) != aDesc)
                continue;
            if (s.busy.test_and_set(std::memory_order_acquire))
            {
                s.poisoned.store(true, std::memory_order_release);
                Skip(m_skipBusy);
                return nullptr;
            }
            if (s.key.load(std::memory_order_relaxed) != aDesc) // stolen between the check and the lock
            {
                s.busy.clear(std::memory_order_release);
                continue;
            }
            s.valid.store(false, std::memory_order_release);
            return &s;
        }
        for (uint32_t i = 0; i < kSlots; ++i) // free, invalid or aged-out slot of another descriptor
        {
            Slot& s = m_slots[i];
            const uint64_t key = s.key.load(std::memory_order_acquire);
            if (key != 0 && s.valid.load(std::memory_order_acquire) && !Aged(s, aNow))
                continue;
            if (s.busy.test_and_set(std::memory_order_acquire))
                continue;
            if (s.valid.load(std::memory_order_relaxed) && s.key.load(std::memory_order_relaxed) != 0 && !Aged(s, aNow))
            {
                s.busy.clear(std::memory_order_release); // (re-checked under the lock: a save committed meanwhile)
                continue;
            }
            s.key.store(aDesc, std::memory_order_release);
            s.valid.store(false, std::memory_order_release);
            s.n = 0;
            return &s;
        }
        Skip(m_skipFull);
        return nullptr;
    }
    // Publishes aN gathered records (aSlot->dst / data filled by the caller), opens the evaluation, unlocks. Returns the serial.
    uint64_t CommitSave(Slot* aSlot, uint32_t aN, int64_t aNow, uint32_t aThread)
    {
        aSlot->n = aN > kMaxBones ? kMaxBones : aN;
        aSlot->ticks.store(aNow, std::memory_order_relaxed);
        NoteTime(aNow);
        aSlot->thread = aThread;
        aSlot->serial = m_serial.fetch_add(1, std::memory_order_relaxed) + 1;
        aSlot->poisoned.store(false, std::memory_order_relaxed);
        aSlot->valid.store(aSlot->n != 0, std::memory_order_release);
        const uint64_t serial = aSlot->serial;
        m_saves.fetch_add(1, std::memory_order_relaxed);
        m_lastBones.store(aSlot->n, std::memory_order_relaxed);
        m_lastSaveThread.store(aThread, std::memory_order_relaxed);
        m_lastSerial.store(serial, std::memory_order_relaxed);
        aSlot->busy.clear(std::memory_order_release);
        return serial;
    }
    // A fault while gathering: nothing is kept.
    void AbortSave(Slot* aSlot)
    {
        aSlot->valid.store(false, std::memory_order_release);
        aSlot->n = 0;
        m_faults.fetch_add(1, std::memory_order_relaxed);
        aSlot->busy.clear(std::memory_order_release);
    }

    // ---- restore side (every later merge into the same descriptor) ----
    // Locks and returns the slot when aDesc has a valid, fresh evaluation (and, with aExpectSerial != 0, exactly that one);
    // nullptr otherwise (counted by reason). Records the thread instrumentation on success.
    Slot* BeginRestore(uint64_t aDesc, int64_t aNow, uint32_t aThread, uint64_t aExpectSerial = 0)
    {
        NoteTime(aNow); // restores keep arriving while the table is full, so the age reclaim of BeginSave(aDesc) stays current
        for (uint32_t i = 0; i < kSlots; ++i)
        {
            Slot& s = m_slots[i];
            if (!aDesc || s.key.load(std::memory_order_acquire) != aDesc)
                continue;
            if (!s.valid.load(std::memory_order_acquire))
            {
                Skip(m_skipNoSlot);
                return nullptr;
            }
            if (s.poisoned.load(std::memory_order_acquire))
            {
                Skip(m_skipPoison);
                return nullptr;
            }
            if (s.busy.test_and_set(std::memory_order_acquire))
            {
                Skip(m_skipBusy);
                return nullptr;
            }
            const int64_t maxAge = m_maxAge.load(std::memory_order_relaxed);
#if UTPP_RAGDOLL_KEEP_MUTATION == 3
            const bool tooOld = ((void)maxAge, (void)aNow, false); // mutation 3: no age cap
#else
            const bool tooOld = maxAge > 0 && (aNow - s.ticks.load(std::memory_order_relaxed) > maxAge || aNow < s.ticks.load(std::memory_order_relaxed));
#endif
#if UTPP_RAGDOLL_KEEP_MUTATION == 1
            const bool serialOk = ((void)aExpectSerial, true); // mutation 1: the serial check is gone
#else
            const bool serialOk = aExpectSerial == 0 || aExpectSerial == s.serial;
#endif
            if (s.key.load(std::memory_order_relaxed) != aDesc || !s.valid.load(std::memory_order_relaxed) ||
                s.poisoned.load(std::memory_order_relaxed) || s.n == 0)
            {
                s.busy.clear(std::memory_order_release);
                Skip(m_skipNoSlot);
                return nullptr;
            }
            if (tooOld)
            {
                s.valid.store(false, std::memory_order_release);
                s.busy.clear(std::memory_order_release);
                Skip(m_skipStale);
                return nullptr;
            }
            if (!serialOk)
            {
                s.busy.clear(std::memory_order_release);
                Skip(m_skipSerial);
                return nullptr;
            }
            if (s.thread == aThread)
                m_threadSame.fetch_add(1, std::memory_order_relaxed);
            else
                m_threadDiff.fetch_add(1, std::memory_order_relaxed);
            m_lastRestoreThread.store(aThread, std::memory_order_relaxed);
            return &s;
        }
        Skip(m_skipNoSlot);
        return nullptr;
    }
    void EndRestore(Slot* aSlot)
    {
        m_restores.fetch_add(1, std::memory_order_relaxed);
        aSlot->busy.clear(std::memory_order_release);
    }
    // A fault while writing back: the evaluation is dropped, the remaining merges of this frame stay unkept.
    void AbortRestore(Slot* aSlot)
    {
        aSlot->valid.store(false, std::memory_order_release);
        m_faults.fetch_add(1, std::memory_order_relaxed);
        aSlot->busy.clear(std::memory_order_release);
    }

    // ---- evaluation end ----
    // FK of aDesc: the evaluation is over. A busy slot is poisoned instead (a restore in flight must not outlive its frame).
    void CloseEvaluation(uint64_t aDesc)
    {
#if UTPP_RAGDOLL_KEEP_MUTATION == 2
        (void)aDesc; // mutation 2: FK never closes the evaluation
        return;
#else
        if (!aDesc)
            return;
        for (uint32_t i = 0; i < kSlots; ++i)
        {
            Slot& s = m_slots[i];
            if (s.key.load(std::memory_order_acquire) != aDesc || !s.valid.load(std::memory_order_acquire))
                continue;
            if (s.busy.test_and_set(std::memory_order_acquire))
            {
                s.poisoned.store(true, std::memory_order_release);
                // The evaluation is over either way: the holder is a restore (it finishes with the data it already has) or a save
                // (CommitSave reopens the slot). Leaving valid set would park the slot until the age cap.
#if UTPP_RAGDOLL_KEEP_MUTATION != 5 // mutation 5: the poisoned slot stays valid and parks until the age cap
                s.valid.store(false, std::memory_order_release);
#endif
                continue;
            }
            s.valid.store(false, std::memory_order_release);
            s.busy.clear(std::memory_order_release);
        }
#endif
    }
    // Disarm / episode end: nothing may be restored any more.
    void InvalidateAll()
    {
        for (uint32_t i = 0; i < kSlots; ++i)
        {
            m_slots[i].valid.store(false, std::memory_order_release);
            m_slots[i].poisoned.store(true, std::memory_order_release); // cleared by the next committed save
        }
    }

    // ---- convenience wrappers (tests / non-SEH callers); an exception thrown by the callback counts as a fault ----
    template<typename Fill>
    bool Save(uint64_t aDesc, int64_t aNow, uint32_t aThread, Fill&& aFill, uint64_t* aSerial = nullptr)
    {
        Slot* s = BeginSave(aDesc, aNow);
        if (!s)
            return false;
        uint32_t n = 0;
        try
        {
            n = aFill(s->dst, s->data);
        }
        catch (...)
        {
            AbortSave(s);
            return false;
        }
        const uint64_t serial = CommitSave(s, n, aNow, aThread);
        if (aSerial)
            *aSerial = serial;
        return n != 0;
    }
    template<typename Apply>
    bool Restore(uint64_t aDesc, int64_t aNow, uint32_t aThread, Apply&& aApply, uint64_t aExpectSerial = 0)
    {
        Slot* s = BeginRestore(aDesc, aNow, aThread, aExpectSerial);
        if (!s)
            return false;
        try
        {
            aApply(s->n, s->dst, s->data);
        }
        catch (...)
        {
            AbortRestore(s);
            return false;
        }
        EndRestore(s);
        return true;
    }

    Counters Snapshot() const
    {
        Counters c;
        c.saves = m_saves.load(std::memory_order_relaxed);
        c.restores = m_restores.load(std::memory_order_relaxed);
        c.faults = m_faults.load(std::memory_order_relaxed);
        c.skipNoSlot = m_skipNoSlot.load(std::memory_order_relaxed);
        c.skipStale = m_skipStale.load(std::memory_order_relaxed);
        c.skipSerial = m_skipSerial.load(std::memory_order_relaxed);
        c.skipBusy = m_skipBusy.load(std::memory_order_relaxed);
        c.skipPoison = m_skipPoison.load(std::memory_order_relaxed);
        c.skipFull = m_skipFull.load(std::memory_order_relaxed);
        c.skips = c.skipNoSlot + c.skipStale + c.skipSerial + c.skipBusy + c.skipPoison + c.skipFull;
        c.threadSame = m_threadSame.load(std::memory_order_relaxed);
        c.threadDiff = m_threadDiff.load(std::memory_order_relaxed);
        c.lastBones = m_lastBones.load(std::memory_order_relaxed);
        c.lastSaveThread = m_lastSaveThread.load(std::memory_order_relaxed);
        c.lastRestoreThread = m_lastRestoreThread.load(std::memory_order_relaxed);
        c.lastSerial = m_lastSerial.load(std::memory_order_relaxed);
        return c;
    }

private:
    void Skip(std::atomic<uint64_t>& aReason) { aReason.fetch_add(1, std::memory_order_relaxed); }
    void NoteTime(int64_t aNow)
    {
        if (aNow > m_now.load(std::memory_order_relaxed))
            m_now.store(aNow, std::memory_order_relaxed);
    }
    // An open evaluation older than the age cap (never restored, never closed). Unlocked pre-check; callers re-check under the lock.
    bool Aged(const Slot& aSlot, int64_t aNow) const
    {
#if UTPP_RAGDOLL_KEEP_MUTATION == 4
        (void)aSlot; (void)aNow;
        return false; // mutation 4: a stale open slot is never reclaimed
#else
        const int64_t maxAge = m_maxAge.load(std::memory_order_relaxed);
        const int64_t t = aSlot.ticks.load(std::memory_order_relaxed);
        return maxAge > 0 && aNow > t && aNow - t > maxAge;
#endif
    }

    Slot m_slots[kSlots];
    std::atomic<int64_t> m_maxAge{0}; // 0 = no age cap (tests); the game sets ~50 ms
    std::atomic<int64_t> m_now{0};    // latest time passed to a save / restore (clock of the one-argument BeginSave)
    std::atomic<uint64_t> m_serial{0};
    std::atomic<uint64_t> m_saves{0}, m_restores{0}, m_faults{0};
    std::atomic<uint64_t> m_skipNoSlot{0}, m_skipStale{0}, m_skipSerial{0}, m_skipBusy{0}, m_skipPoison{0}, m_skipFull{0};
    std::atomic<uint64_t> m_threadSame{0}, m_threadDiff{0};
    std::atomic<uint32_t> m_lastBones{0}, m_lastSaveThread{0}, m_lastRestoreThread{0};
    std::atomic<uint64_t> m_lastSerial{0};
};
} // namespace utpp::ragdoll

// UTPP item render plane, pure core (2026-11-01; round 2: anim-param handler gate, agreement-aware outsideLast, size peak): the
// plane policy, the per-call decision of the plane setter detour and the
// bounded registry of components whose plane the animation-parameter broadcast drives. No game types, no locks, no allocation:
// ItemPlane.cpp wraps it (lock, weak references, the real setter); tests/ItemPlaneCoreTest.cpp drives it directly.
//
// Why a registry: the broadcast detour alone latches. Components set by the FPP graph (renderPlane / renderPlaneLeftArm /
// renderPlaneInspect -> 2 = RPl_Weapon) before the TPP override turns on keep RPl_Weapon until the parameter changes again, and
// components rewritten to RPl_Scene keep it after the override turns off (FPP arms / sleeves would clip into walls). The registry
// remembers, per component, the plane vanilla last requested INSIDE a broadcast; on every policy change the game thread re-applies
// PlaneFor(policy, requested) to every live entry: force -> Scene for RPl_Weapon, vanilla -> exactly the requested value.
//
// Rules (each one is a check in ItemPlaneCoreTest, with a mutation below):
//  - PlaneFor: only RPl_Weapon (2) becomes RPl_Scene (0) and only while forcing; 0, 1 (Background), 3 and 4 (the default sentinel
//    that the setter resolves through vfunc +0x278) pass unchanged. Background is only counted (bgInBc), not rewritten (design C).
//  - The setter detour applies PlaneFor only to an animation-parameter write ("inside a broadcast", InAnimParam): the broadcast
//    frame itself OR one of the two anim-param event handlers (Skinned 0x20A240 / MorphTarget 0x209000, which also run later from
//    the owner entity's queued entAnimParamsEvent with no broadcast frame, audit NG1). Outside (equip start plane, item events
//    incl. UTPP's own scene / weapon events, quests) the requested plane is written unchanged.
//  - Registration happens only inside a broadcast. An outside-broadcast write to an already registered component that DISAGREES
//    with the policy (written != PlaneFor(override, requested)) marks the entry "outsideLast"; the re-apply leaves such an entry
//    alone (the outside write survives, e.g. a stale event landing after an edge or a vanilla / quest override). An outside write
//    that AGREES with the policy (UTPP's own Correct(): 0 in TPP, 2 in FPP; audit SPECIAL-2) clears the mark, so the re-apply owns
//    the next edge. The next broadcast write clears the mark too. The requested plane is never overwritten by an outside write.
//  - The registry keeps the REQUESTED plane, never the written one, so force -> vanilla restores the original value exactly.
//  - Entries are keyed by (instance, reference-count block), so a reused address with a new reference block is a different entry.
//  - Bounded: kMaxEntries = 3/4 of the slots (open addressing, linear probe); a new component over the cap is dropped and counted.
//    Dead entries (strong count 0, decided by the caller) are removed by Prune (edges, re-apply, dumps; rebuilds the table) and,
//    since 2026-11-01 (review: a long TPP session without edges filled the table with dead entries and dropped V's components),
//    by a reclaim run from RecordInside itself when a NEW component would hit the cap: the caller's aReclaim (an allocation-free
//    Prune of a bounded batch, under the caller's lock) frees room first and the component is dropped only if the table is still
//    full. The reclaim is rate-limited (kReclaimGap drops) after one that freed nothing, so a table full of live entries is not
//    rescanned for every new component; a reclaim that freed room re-arms it at once.
#pragma once

#include <cstddef>
#include <cstdint>

// UTPP_ITEMPLANE_MUTATION=N (test builds only; undefined / 0 = production), each must fail its own ItemPlaneCoreTest check:
//   1 = PlaneFor ignores the policy (always forces);            2 = the detour ignores the inside-broadcast gate;
//   3 = forcing rewrites every plane, not only RPl_Weapon;      4 = the re-apply rewrites entries whose last write was outside;
//   5 = Prune keeps dead entries;                               6 = the cap admits one entry too many;
//   7 = the registry keeps the written plane, not the requested one;
//   8 = a broadcast write does not clear the outsideLast mark;  9 = an outside-broadcast write registers a new component;
//  10 = entries are keyed by the instance only (a reused address aliases a dead entry);
//  11 = InAnimParam ignores the handler depth (the queued anim-param event path counts as outside, audit NG1);
//  12 = NoteOutside drops the agreement check (every outside write marks outsideLast, audit SPECIAL-2);
//  13 = no reclaim at the cap (a table full of dead entries drops the new component);
//  14 = the reclaim ignores the rate limit (a full table of live entries is rescanned for every new component).
#ifndef UTPP_ITEMPLANE_MUTATION
#define UTPP_ITEMPLANE_MUTATION 0
#endif

namespace utpp::itemplane::core
{
constexpr int32_t kPlaneScene = 0;
constexpr int32_t kPlaneBackground = 1;
constexpr int32_t kPlaneWeapon = 2;

// The one policy mapping, used by the detour (inside a broadcast) and by the re-apply.
constexpr int32_t PlaneFor(bool aForce, int32_t aRequested)
{
#if UTPP_ITEMPLANE_MUTATION == 1
    aForce = true;
#endif
#if UTPP_ITEMPLANE_MUTATION == 3
    return aForce ? kPlaneScene : aRequested;
#else
    return (aForce && aRequested == kPlaneWeapon) ? kPlaneScene : aRequested;
#endif
}

// Whether a setter call carries an animation-parameter plane: inside the broadcast frame (aBcDepth) or inside one of the two
// anim-param event handlers (aHandlerDepth; they run both from the broadcast and from the owner's queued entAnimParamsEvent).
// The detour passes the result as aInBroadcast to OnSetterCall.
constexpr bool InAnimParam(int aBcDepth, int aHandlerDepth)
{
#if UTPP_ITEMPLANE_MUTATION == 11
    (void)aHandlerDepth;
    return aBcDepth > 0;
#else
    return aBcDepth > 0 || aHandlerDepth > 0;
#endif
}

// The plane the setter detour passes on to the real setter.
constexpr int32_t DetourPlane(bool aInBroadcast, bool aOverride, int32_t aRequested)
{
#if UTPP_ITEMPLANE_MUTATION == 2
    aInBroadcast = true;
#endif
    return aInBroadcast ? PlaneFor(aOverride, aRequested) : aRequested;
}

struct Entry
{
    uintptr_t instance = 0; // component pointer (0 = free slot)
    uintptr_t refCnt = 0;   // its reference-count block (ISerializable::ref.refCount)
    int32_t requested = 0;  // plane last requested inside a broadcast (raw value, 0..4)
    bool outsideLast = false; // the last write came from outside a broadcast: the re-apply leaves the component alone
};

enum class RecordResult : uint8_t
{
    Inserted, // new entry (the caller takes a weak reference)
    Updated,  // existing entry, requested plane refreshed
    Dropped,  // registry full: not recorded (counted)
};

template<size_t kSlots>
class Registry
{
    static_assert(kSlots >= 8 && (kSlots & (kSlots - 1)) == 0, "slot count must be a power of two");

public:
#if UTPP_ITEMPLANE_MUTATION == 6
    static constexpr size_t kMaxEntries = kSlots / 4 * 3 + 1;
#else
    static constexpr size_t kMaxEntries = kSlots / 4 * 3;
#endif
    static constexpr size_t kCapacity = kSlots;
    // Drops between two reclaim attempts after an attempt that freed nothing (the table is really full of live entries).
    static constexpr size_t kReclaimGap = 64;

    size_t Size() const
    {
        return m_size;
    }
    // High-water mark of Size() since construction / Forget (Prune does not lower it): registry health (cap 3/4 of the slots).
    size_t Peak() const
    {
        return m_peak;
    }

    // Inside a broadcast. aOnInsert() runs (under the caller's lock) only for a new entry. aReclaim() runs (under the same lock,
    // at most once per call) only when a new entry would exceed the cap: it removes dead entries (through Prune) and the new
    // entry is dropped only if the table is still full.
    template<typename OnInsert, typename OnReclaim>
    RecordResult RecordInside(uintptr_t aInstance, uintptr_t aRefCnt, int32_t aRequested, OnInsert&& aOnInsert,
                              OnReclaim&& aReclaim)
    {
        Entry* e = Probe(aInstance, aRefCnt);
        if (e->instance != 0)
        {
#if UTPP_ITEMPLANE_MUTATION == 10
            e->refCnt = aRefCnt;
#endif
            e->requested = aRequested;
#if UTPP_ITEMPLANE_MUTATION != 8
            e->outsideLast = false;
#endif
            return RecordResult::Updated;
        }
        if (m_size >= kMaxEntries)
        {
            if (!TryReclaim(aReclaim))
                return RecordResult::Dropped;
            e = Probe(aInstance, aRefCnt); // the reclaim rebuilt the table: the first probe result is stale
        }
        e->instance = aInstance;
        e->refCnt = aRefCnt;
        e->requested = aRequested;
        e->outsideLast = false;
        ++m_size;
        if (m_size > m_peak)
            m_peak = m_size;
        aOnInsert();
        return RecordResult::Inserted;
    }

    // Outside a broadcast: never registers. For a registered component, outsideLast = the write disagrees with the policy
    // (aRequested != PlaneFor(aOverride, entry.requested)): a disagreeing write is protected from the re-apply, an agreeing one
    // (UTPP's own Correct() events) hands the component back to it. Returns whether the component is registered.
    bool NoteOutside(uintptr_t aInstance, uintptr_t aRefCnt, int32_t aRequested, bool aOverride)
    {
        Entry* e = Probe(aInstance, aRefCnt);
        if (e->instance == 0)
        {
#if UTPP_ITEMPLANE_MUTATION == 9
            if (m_size < kMaxEntries)
            {
                e->instance = aInstance;
                e->refCnt = aRefCnt;
                e->requested = aRequested;
                e->outsideLast = false;
                ++m_size;
            }
#endif
            (void)aRequested;
            return false;
        }
#if UTPP_ITEMPLANE_MUTATION == 12
        (void)aOverride;
        e->outsideLast = true;
#else
        e->outsideLast = aRequested != PlaneFor(aOverride, e->requested);
#endif
        return true;
    }

    const Entry* Find(uintptr_t aInstance, uintptr_t aRefCnt) const
    {
        const Entry* e = const_cast<Registry*>(this)->Probe(aInstance, aRefCnt);
        return e->instance != 0 ? e : nullptr;
    }

    template<typename F>
    void ForEach(F&& aFn) const
    {
        for (size_t i = 0; i < kSlots; ++i)
            if (m_slots[i].instance != 0)
                aFn(m_slots[i]);
    }

    // Removes every entry for which aIsDead(entry) is true; aOnRemove(entry) runs for each removed one (the caller releases its
    // weak reference, later and outside its lock). Rebuilds the table (no tombstones). No allocation, no game calls: safe under
    // the caller's registry lock from any thread, provided aIsDead / aOnRemove are (the setter-detour reclaim passes a bounded
    // aIsDead and collects the removed entries into a fixed array). Returns the number removed.
    template<typename IsDead, typename OnRemove>
    size_t Prune(IsDead&& aIsDead, OnRemove&& aOnRemove)
    {
        size_t kept = 0, removed = 0;
        for (size_t i = 0; i < kSlots; ++i)
        {
            const Entry& e = m_slots[i];
            if (e.instance == 0)
                continue;
#if UTPP_ITEMPLANE_MUTATION == 5
            const bool dead = false;
            (void)aIsDead;
#else
            const bool dead = aIsDead(e);
#endif
            if (dead)
            {
                aOnRemove(e);
                ++removed;
            }
            else
                m_scratch[kept++] = e;
        }
        if (removed == 0)
            return 0;
        for (size_t i = 0; i < kSlots; ++i)
            m_slots[i] = Entry{};
        m_size = 0;
        for (size_t i = 0; i < kept; ++i)
        {
            Entry* e = Probe(m_scratch[i].instance, m_scratch[i].refCnt);
            *e = m_scratch[i];
            ++m_size;
        }
        return removed;
    }

    // Forget every entry without callbacks (plugin unload: the game's allocators may already be gone, the weak references leak).
    void Forget()
    {
        for (size_t i = 0; i < kSlots; ++i)
            m_slots[i] = Entry{};
        m_size = 0;
        m_peak = 0;
    }

private:
    // Called when a new entry would exceed the cap. True when the reclaim freed room (rate limit: see kReclaimGap).
    template<typename OnReclaim>
    bool TryReclaim(OnReclaim&& aReclaim)
    {
#if UTPP_ITEMPLANE_MUTATION == 13
        (void)aReclaim;
        return false;
#else
#if UTPP_ITEMPLANE_MUTATION != 14
        if (m_dropsSinceReclaim < kReclaimGap)
        {
            ++m_dropsSinceReclaim;
            return false;
        }
#endif
        aReclaim();
        if (m_size < kMaxEntries)
        {
            m_dropsSinceReclaim = kReclaimGap; // freed room: the next full table may reclaim at once
            return true;
        }
        m_dropsSinceReclaim = 0;
        return false;
#endif
    }

    static size_t Hash(uintptr_t aInstance, uintptr_t aRefCnt)
    {
        uint64_t h = static_cast<uint64_t>(aInstance) ^ (static_cast<uint64_t>(aRefCnt) * 0x9E3779B97F4A7C15ull);
        h ^= h >> 29;
        h *= 0xBF58476D1CE4E5B9ull;
        h ^= h >> 32;
        return static_cast<size_t>(h);
    }

    static bool SameKey(const Entry& aEntry, uintptr_t aInstance, uintptr_t aRefCnt)
    {
#if UTPP_ITEMPLANE_MUTATION == 10
        (void)aRefCnt;
        return aEntry.instance == aInstance;
#else
        return aEntry.instance == aInstance && aEntry.refCnt == aRefCnt;
#endif
    }

    // The matching entry, else the free slot where it would go. The table never fills (cap 3/4), so the probe terminates.
    Entry* Probe(uintptr_t aInstance, uintptr_t aRefCnt)
    {
#if UTPP_ITEMPLANE_MUTATION == 10
        size_t i = Hash(aInstance, 0) & (kSlots - 1);
#else
        size_t i = Hash(aInstance, aRefCnt) & (kSlots - 1);
#endif
        for (;;)
        {
            Entry& e = m_slots[i];
            if (e.instance == 0 || SameKey(e, aInstance, aRefCnt))
                return &e;
            i = (i + 1) & (kSlots - 1);
        }
    }

    Entry m_slots[kSlots] = {};
    Entry m_scratch[kSlots] = {};
    size_t m_size = 0;
    size_t m_peak = 0;
    size_t m_dropsSinceReclaim = kReclaimGap; // armed: the first full table reclaims at once
};

// What one setter call did (the native layer turns the flags into its counters).
struct SetterOutcome
{
    int32_t plane = 0;        // the plane passed on to the real setter
    bool weaponInBc = false;  // RPl_Weapon requested inside a broadcast (any override state)
    bool forced = false;      // ... and rewritten to RPl_Scene
    bool bgInBc = false;      // RPl_Background requested inside a broadcast (counted, not rewritten)
    bool weaponOutBc = false; // RPl_Weapon requested outside a broadcast (never rewritten)
    bool noRef = false;       // no usable reference (aInstance == 0): not registered
    RecordResult record = RecordResult::Updated;
    bool recorded = false;    // inside a broadcast with a reference: RecordInside ran (see record)
    bool notedOutside = false; // outside a broadcast, the component is registered: outsideLast set or cleared (NoteOutside)
};

// The complete bookkeeping of one plane setter call (the caller holds the registry lock). aInBroadcast = InAnimParam(...) of the
// caller. aInstance / aRefCnt = 0: no usable reference (the component is still written exactly as without the registry).
// aOnInsert / aReclaim: see Registry::RecordInside (the overload without aReclaim never reclaims).
template<size_t kSlots, typename OnInsert, typename OnReclaim>
SetterOutcome OnSetterCall(Registry<kSlots>& aReg, bool aInBroadcast, bool aOverride, uintptr_t aInstance, uintptr_t aRefCnt,
                           int32_t aRequested, OnInsert&& aOnInsert, OnReclaim&& aReclaim)
{
    SetterOutcome o;
    o.plane = DetourPlane(aInBroadcast, aOverride, aRequested);
    o.noRef = aInstance == 0 || aRefCnt == 0;
    if (aInBroadcast)
    {
        o.weaponInBc = aRequested == kPlaneWeapon;
        o.forced = o.plane != aRequested;
        o.bgInBc = aRequested == kPlaneBackground;
        if (!o.noRef)
        {
#if UTPP_ITEMPLANE_MUTATION == 7
            o.record = aReg.RecordInside(aInstance, aRefCnt, o.plane, aOnInsert, aReclaim);
#else
            o.record = aReg.RecordInside(aInstance, aRefCnt, aRequested, aOnInsert, aReclaim);
#endif
            o.recorded = true;
        }
    }
    else
    {
        o.weaponOutBc = aRequested == kPlaneWeapon;
        if (!o.noRef)
            o.notedOutside = aReg.NoteOutside(aInstance, aRefCnt, aRequested, aOverride);
    }
    return o;
}

template<size_t kSlots, typename OnInsert>
SetterOutcome OnSetterCall(Registry<kSlots>& aReg, bool aInBroadcast, bool aOverride, uintptr_t aInstance, uintptr_t aRefCnt,
                           int32_t aRequested, OnInsert&& aOnInsert)
{
    return OnSetterCall(aReg, aInBroadcast, aOverride, aInstance, aRefCnt, aRequested, aOnInsert, [] { return size_t(0); });
}

// One re-apply step for a live entry whose current plane (component +0x138) is aCurrent. write = false: leave it (the last write
// came from outside a broadcast and disagreed with the policy then, or the plane already matches; the real setter would not notify on an equal value either).
struct ReapplyStep
{
    bool write = false;
    int32_t plane = 0;
    bool orphan = false; // the write was due but the component has no owner (detached): skipped, see PlanReapplyAttached
};
constexpr ReapplyStep PlanReapply(bool aForce, const Entry& aEntry, int32_t aCurrent)
{
#if UTPP_ITEMPLANE_MUTATION != 4
    if (aEntry.outsideLast)
        return {false, aCurrent};
#endif
    const int32_t target = PlaneFor(aForce, aEntry.requested);
    return {target != aCurrent, target};
}

// PlanReapply for a component whose owner (entIComponent +0x50) was read right before the call: aOwnerAttached = owner != nullptr.
// The game's plane setter dereferences the owner (propagation, vfunc +0x280), so a component without an owner (still referenced but
// detached from its entity, e.g. an FPP garment part after the return from first-person aim) must never reach it: the entry is
// skipped (write = false, orphan = true) and stays in the registry (it may be attached again when the representation returns).
// Only a write that would have been made counts as an orphan; an entry that needs no write is not reported.
constexpr ReapplyStep PlanReapplyAttached(bool aForce, const Entry& aEntry, int32_t aCurrent, bool aOwnerAttached)
{
    const ReapplyStep step = PlanReapply(aForce, aEntry, aCurrent);
    if (step.write && !aOwnerAttached)
        return {false, aCurrent, true};
    return step;
}
} // namespace utpp::itemplane::core

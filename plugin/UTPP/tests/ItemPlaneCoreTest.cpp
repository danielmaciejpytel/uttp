// UTPP item render plane core (src/render/ItemPlaneCore.hpp): the policy table (policy x requested plane x inside / outside a
// broadcast), force -> vanilla restoring the requested plane exactly, outside-broadcast writes never rewritten (neither by the
// detour nor by the re-apply), the outsideLast mark cleared by the next broadcast write, pruning of dead entries, the size cap and
// the reclaim of dead entries when a new component hits the cap (+ its rate limit) and the (instance, reference block) key. No data files, no game types: a fake component world drives the same core calls as
// ItemPlane.cpp (OnSetterCall in the detour; Prune + PlanReapply on the game thread).
// Mutations (itemplane.cmake, each must fail its own check): see the list in ItemPlaneCore.hpp (1..14).
#include <cstdio>
#include <cstdint>
#include <cstdarg>
#include <string>
#include <vector>

#include "../src/render/ItemPlaneCore.hpp"
#include "TestFailFast.hpp"

using namespace utpp::itemplane::core;
static int passed = 0, failed = 0;
// FAIL text of a CHECK line, for the mutation early exit (TestFailFast.hpp); same text as the printed line.
static std::string FailText(const char* aFmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, aFmt);
    std::vsnprintf(buf, sizeof buf, aFmt, ap);
    va_end(ap);
    return buf;
}
#define CHECK(c, ...) do { if (c) ++passed; else { ++failed; const std::string utppFailLine = FailText("FAIL " __VA_ARGS__); std::printf("%s\n", utppFailLine.c_str()); UtppFailFastNote(utppFailLine); } } while (0)

namespace
{
constexpr size_t kSlots = 64; // kMaxEntries = 48

struct FakeComp
{
    int32_t plane = 0; // component +0x138 (the real setter stores the raw value)
    bool alive = true; // strong count > 0
    bool attached = true; // owner (entIComponent +0x50) != nullptr; the real setter dereferences it
    int setterCalls = 0;  // calls the re-apply made to the real setter (a detached component must get none)
    uintptr_t refCnt = 0;
    int weakRefs = 0;  // weak references the registry holds
};

// The fake game: components addressed by index; instance = index + 1 (0 is "no reference").
struct World
{
    Registry<kSlots> reg;
    std::vector<FakeComp> comps;
    uint64_t weaponInBc = 0, forced = 0, bgInBc = 0, weaponOutBc = 0, dropped = 0, released = 0;
    int reclaimCalls = 0;      // reclaim attempts made by RecordInside (cap hit by a new component)
    size_t reclaimBatch = 16;  // bounded batch, as the detour reclaim (ItemPlane.cpp kReclaimBatch = 64)

    explicit World(size_t aCount) : comps(aCount)
    {
        for (size_t i = 0; i < aCount; ++i)
            comps[i].refCnt = 0x1000 + i * 0x10;
    }
    uintptr_t Inst(size_t aIdx) const
    {
        return static_cast<uintptr_t>(aIdx + 1);
    }
    size_t Idx(uintptr_t aInst) const
    {
        return static_cast<size_t>(aInst - 1);
    }
    // One plane setter call through the detour bookkeeping (as DetourSetPlane does).
    SetterOutcome Set(size_t aIdx, bool aInBc, bool aOverride, int32_t aPlane)
    {
        FakeComp& c = comps[aIdx];
        const SetterOutcome o = OnSetterCall(reg, aInBc, aOverride, Inst(aIdx), c.refCnt, aPlane, [&] { ++c.weakRefs; },
                                         [&] { return Reclaim(); });
        weaponInBc += o.weaponInBc;
        forced += o.forced;
        bgInBc += o.bgInBc;
        weaponOutBc += o.weaponOutBc;
        dropped += o.recorded && o.record == RecordResult::Dropped;
        c.plane = o.plane;
        return o;
    }
    // The detour-side reclaim: dead entries only, at most reclaimBatch per call (as ItemPlane.cpp ReclaimInDetour).
    size_t Reclaim()
    {
        ++reclaimCalls;
        size_t taken = 0;
        return reg.Prune([&](const Entry& e) { return !comps[Idx(e.instance)].alive && taken < reclaimBatch; },
                         [&](const Entry& e) {
                             ++taken;
                             --comps[Idx(e.instance)].weakRefs;
                             ++released;
                         });
    }
    size_t Prune()
    {
        return reg.Prune([&](const Entry& e) { return !comps[Idx(e.instance)].alive; },
                         [&](const Entry& e) {
                             --comps[Idx(e.instance)].weakRefs;
                             ++released;
                         });
    }
    // The game-thread re-apply (as ItemPlane.cpp Reapply): prune, then PlanReapply for every live entry. Returns the writes.
    // The owner is read right before each setter call (PlanReapplyAttached); a detached component is skipped and counted.
    int orphans = 0;
    int Reapply(bool aForce)
    {
        Prune();
        int written = 0;
        reg.ForEach([&](const Entry& e) {
            FakeComp& c = comps[Idx(e.instance)];
            if (!c.alive)
                return;
            const ReapplyStep s = PlanReapplyAttached(aForce, e, c.plane, c.attached);
            if (s.orphan)
            {
                ++orphans;
                return;
            }
            if (s.write)
            {
                ++c.setterCalls;
                c.plane = s.plane;
                ++written;
            }
        });
        return written;
    }
};

const char* Yn(bool b)
{
    return b ? "on" : "off";
}
} // namespace

int main()
{
    // 1. Policy table: policy x requested plane (0..4, 4 = the default sentinel) x inside / outside a broadcast.
    for (int ov = 0; ov < 2; ++ov)
        for (int bc = 0; bc < 2; ++bc)
            for (int32_t req = 0; req <= 4; ++req)
            {
                World w(1);
                const SetterOutcome o = w.Set(0, bc != 0, ov != 0, req);
                const int32_t expect = (ov && bc && req == kPlaneWeapon) ? kPlaneScene : req;
                if (!ov)
                    CHECK(o.plane == expect, "policy off: broadcast %s, requested %d -> wrote %d (expected %d)", Yn(bc), req,
                          o.plane, expect);
                else if (!bc)
                    CHECK(o.plane == expect, "outside broadcast: override on, requested %d -> wrote %d (expected %d)", req,
                          o.plane, expect);
                else
                    CHECK(o.plane == expect, "policy force: requested plane %d -> wrote %d (expected %d)", req, o.plane, expect);
                CHECK(PlaneFor(ov != 0, req) == ((ov && req == kPlaneWeapon) ? kPlaneScene : req),
                      "PlaneFor(%d, %d) = %d", ov, req, PlaneFor(ov != 0, req));
                CHECK(o.weaponInBc == (bc && req == kPlaneWeapon), "weaponInBc flag (ov %d bc %d req %d)", ov, bc, req);
                CHECK(o.forced == (ov && bc && req == kPlaneWeapon), "forced flag (ov %d bc %d req %d)", ov, bc, req);
                CHECK(o.bgInBc == (bc && req == kPlaneBackground), "bgInBc flag (ov %d bc %d req %d)", ov, bc, req);
                CHECK(o.weaponOutBc == (!bc && req == kPlaneWeapon), "weaponOutBc flag (ov %d bc %d req %d)", ov, bc, req);
                // Registration only inside a broadcast.
                CHECK(w.reg.Size() == (bc ? 1u : 0u), "outside write registered a component (ov %d bc %d req %d): size %zu", ov,
                      bc, req, w.reg.Size());
            }
    // No usable reference: written exactly as without the registry, nothing registered.
    {
        Registry<kSlots> reg;
        int inserts = 0;
        const SetterOutcome o = OnSetterCall(reg, true, true, 0, 0, kPlaneWeapon, [&] { ++inserts; });
        CHECK(o.plane == kPlaneScene && o.noRef && !o.recorded && reg.Size() == 0 && inserts == 0,
              "no reference: plane %d noRef %d recorded %d size %zu", o.plane, o.noRef, o.recorded, reg.Size());
    }

    // 2. Latching removed: the FPP graph set RPl_Weapon before the override turned on; force -> vanilla restores exactly.
    {
        World w(4);
        w.Set(0, true, false, kPlaneWeapon);     // arms: FPP graph drives 2 (override off)
        w.Set(1, true, false, kPlaneBackground); // something on Background
        w.Set(2, true, false, 4);                // default sentinel
        w.Set(3, true, false, kPlaneScene);
        CHECK(w.comps[0].plane == kPlaneWeapon, "override off writes 2 unchanged (got %d)", w.comps[0].plane);
        const int f = w.Reapply(true);
        CHECK(w.comps[0].plane == kPlaneScene && f == 1, "force re-apply: arms plane %d, writes %d (expected 0, 1)",
              w.comps[0].plane, f);
        CHECK(w.comps[1].plane == kPlaneBackground && w.comps[2].plane == 4 && w.comps[3].plane == kPlaneScene,
              "force re-apply touched non-weapon planes: %d %d %d", w.comps[1].plane, w.comps[2].plane, w.comps[3].plane);
        const int v = w.Reapply(false);
        CHECK(w.comps[0].plane == kPlaneWeapon && v == 1, "restore: vanilla re-apply gives arms plane %d, writes %d (expected 2, 1)",
              w.comps[0].plane, v);
        CHECK(w.Reapply(false) == 0, "second vanilla re-apply wrote again");
    }
    {
        // Recorded while the override was on (written 0): the vanilla re-apply must give back the REQUESTED 2.
        World w(1);
        w.Set(0, true, true, kPlaneWeapon);
        CHECK(w.comps[0].plane == kPlaneScene, "override on, broadcast 2 -> %d", w.comps[0].plane);
        w.Reapply(false);
        CHECK(w.comps[0].plane == kPlaneWeapon, "restore after a forced broadcast write: plane %d (expected 2)", w.comps[0].plane);
        w.Reapply(true);
        CHECK(w.comps[0].plane == kPlaneScene, "force again after restore: plane %d", w.comps[0].plane);
    }

    // 3. Outside-broadcast writes are never rewritten (FPP restore event, equip start plane, quests).
    {
        World w(1);
        w.Set(0, true, false, kPlaneWeapon);             // registered by the FPP graph
        const SetterOutcome o = w.Set(0, false, true, kPlaneWeapon); // outside write while the override is on
        CHECK(o.plane == kPlaneWeapon && o.notedOutside, "outside write with override on: wrote %d noted %d", o.plane,
              o.notedOutside);
        const int f = w.Reapply(true);
        CHECK(w.comps[0].plane == kPlaneWeapon && f == 0, "outside-written entry rewritten by the force re-apply: plane %d, writes %d",
              w.comps[0].plane, f);
        // The next broadcast write clears the mark: the re-apply owns the component again.
        w.Set(0, true, true, kPlaneWeapon); // -> 0
        w.Reapply(false);
        CHECK(w.comps[0].plane == kPlaneWeapon, "broadcast write clears the outside mark: vanilla re-apply gives %d (expected 2)",
              w.comps[0].plane);
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(e && e->requested == kPlaneWeapon, "outside write changed the requested plane: %d", e ? e->requested : -1);
    }
    {
        // UTPP's own scene event (outside) on a registered component must not become the "vanilla" value. Round 2 (SPECIAL-2):
        // with the override on it agrees with the policy (PlaneFor(on, 2) = 0), so it does not mark the entry outsideLast.
        World w(1);
        w.Set(0, true, false, kPlaneWeapon);
        w.Set(0, false, true, kPlaneScene); // UTPPSetItemRenderPlane(0)
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(e && e->requested == kPlaneWeapon, "scene event recorded as vanilla: requested %d", e ? e->requested : -1);
        CHECK(e && !e->outsideLast, "agree rows: scene event with the override on marked the entry outside");
    }

    // 4. Pruning: dead entries removed on the game thread, weak references released once, survivors still found.
    {
        World w(5);
        for (size_t i = 0; i < 5; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        int weak = 0;
        for (auto& c : w.comps)
            weak += c.weakRefs;
        CHECK(weak == 5, "weak references taken %d (expected 5, one per new component)", weak);
        w.Set(2, true, false, kPlaneWeapon); // update: no second weak reference
        CHECK(w.comps[2].weakRefs == 1, "update took another weak reference (%d)", w.comps[2].weakRefs);
        w.comps[1].alive = false;
        w.comps[3].alive = false;
        const int f = w.Reapply(true);
        CHECK(w.reg.Size() == 3 && w.released == 2, "prune: size %zu, released %llu (expected 3, 2)", w.reg.Size(),
              static_cast<unsigned long long>(w.released));
        CHECK(f == 3 && w.comps[1].plane == kPlaneWeapon && w.comps[3].plane == kPlaneWeapon,
              "dead components written by the re-apply (writes %d)", f);
        CHECK(w.comps[1].weakRefs == 0 && w.comps[3].weakRefs == 0 && w.comps[0].weakRefs == 1, "weak references after prune");
        for (size_t i : {size_t(0), size_t(2), size_t(4)})
            CHECK(w.reg.Find(w.Inst(i), w.comps[i].refCnt) != nullptr, "survivor %zu lost by the prune rebuild", i);
        CHECK(w.Prune() == 0, "second prune removed something");
    }

    // 5. Cap: 3/4 of the slots; new components over the cap are dropped (no weak reference), updates still work, prune frees room.
    {
        World w(60);
        for (size_t i = 0; i < 60; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        const size_t cap = Registry<kSlots>::kMaxEntries;
        int weak = 0;
        for (auto& c : w.comps)
            weak += c.weakRefs;
        CHECK(w.reg.Size() == 48 && w.dropped == 12 && weak == 48, "cap: max %zu size %zu dropped %llu weak %d",
              cap, w.reg.Size(), static_cast<unsigned long long>(w.dropped), weak);
        CHECK(w.comps[59].plane == kPlaneWeapon, "dropped component not written normally (%d)", w.comps[59].plane);
        const SetterOutcome u = w.Set(0, true, true, kPlaneWeapon);
        CHECK(u.record == RecordResult::Updated && u.plane == kPlaneScene, "update at the cap: record %d plane %d",
              static_cast<int>(u.record), u.plane);
        for (size_t i = 0; i < 10; ++i)
            w.comps[i].alive = false;
        w.Prune();
        for (size_t i = 48; i < 58; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        CHECK(w.reg.Size() == 48, "after prune of 10 the dropped components register (size %zu)", w.reg.Size());
        // Size high-water mark (Status registryPeak=): the cap, not lowered by the prune.
        w.comps[48].alive = false;
        w.Prune();
        CHECK(w.reg.Size() == 47 && w.reg.Peak() == 48, "registry peak: size %zu peak %zu (expected 47, 48)", w.reg.Size(),
              w.reg.Peak());
    }

    // 5b. Reclaim at the cap (review 2026-11-01): a long session without prune edges leaves the table full of dead entries; a NEW
    //     component must then register (the reclaim frees a bounded batch first) instead of being dropped. No explicit Prune.
    {
        World w(120);
        for (size_t i = 0; i < 48; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        for (size_t i = 0; i < 48; ++i)
            w.comps[i].alive = false;
        const SetterOutcome o = w.Set(100, true, false, kPlaneWeapon);
        CHECK(o.record == RecordResult::Inserted && w.dropped == 0 && w.comps[100].weakRefs == 1 &&
                  w.reg.Find(w.Inst(100), w.comps[100].refCnt) != nullptr,
              "reclaim at the cap: record %d dropped %llu weak %d (expected Inserted, 0, 1)", static_cast<int>(o.record),
              static_cast<unsigned long long>(w.dropped), w.comps[100].weakRefs);
        CHECK(w.reclaimCalls == 1 && w.released == 16 && w.reg.Size() == 33,
              "reclaim at the cap: calls %d released %llu size %zu (expected 1, 16, 33)", w.reclaimCalls,
              static_cast<unsigned long long>(w.released), w.reg.Size());
        // The survivors of the rebuild are still found, the reclaimed ones are gone and their weak references released once.
        int found = 0, weak = 0;
        for (size_t i = 0; i < 48; ++i)
        {
            found += w.reg.Find(w.Inst(i), w.comps[i].refCnt) != nullptr;
            weak += w.comps[i].weakRefs;
        }
        CHECK(found == 32 && weak == 32, "reclaim at the cap: survivors found %d, weak references left %d (expected 32, 32)", found,
              weak);
        // Further new components keep registering until the freed room is used up, then the next reclaim refills it.
        for (size_t i = 101; i < 120; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        CHECK(w.dropped == 0 && w.reg.Size() == 36,
              "reclaim at the cap: refill dropped %llu size %zu (expected 0, 36)", static_cast<unsigned long long>(w.dropped), w.reg.Size());
    }

    // 5c. Reclaim rate limit: a table really full of live entries is not rescanned for every new component (one attempt, then one
    //     per kReclaimGap + 1 drops); the live entries survive and every extra component is dropped and counted.
    {
        World w(260);
        for (size_t i = 0; i < 48; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        for (size_t i = 100; i < 260; ++i)
            w.Set(i, true, false, kPlaneWeapon);
        CHECK(w.reclaimCalls >= 1 && w.reclaimCalls <= 3 && w.dropped == 160 && w.reg.Size() == 48,
              "reclaim rate limit: %d attempts for 160 dropped components (dropped %llu size %zu; expected 1..3, 160, 48)",
              w.reclaimCalls, static_cast<unsigned long long>(w.dropped), w.reg.Size());
    }

    // 6. Key = (instance, reference block): a reused address with a new block is a different component.
    {
        Registry<kSlots> reg;
        int inserts = 0;
        const auto a = OnSetterCall(reg, true, false, 0x40, 0x9000, kPlaneWeapon, [&] { ++inserts; });
        const auto b = OnSetterCall(reg, true, false, 0x40, 0x9100, kPlaneScene, [&] { ++inserts; });
        CHECK(a.record == RecordResult::Inserted && b.record == RecordResult::Inserted && reg.Size() == 2 && inserts == 2,
              "reused address aliases the old entry: size %zu inserts %d", reg.Size(), inserts);
        const Entry* old = reg.Find(0x40, 0x9000);
        CHECK(old && old->requested == kPlaneWeapon, "old entry overwritten by the reused address");
    }

    // 7. Round 2, NG1: the anim-param gate = broadcast depth OR handler depth (Skinned 0x20A240 / MorphTarget 0x209000). The
    //    owner's queued entAnimParamsEvent runs those handlers with no broadcast frame (bc 0, handler 1).
    for (int bc = 0; bc < 2; ++bc)
        for (int hd = 0; hd < 2; ++hd)
            CHECK(InAnimParam(bc, hd) == (bc != 0 || hd != 0), "handler path: InAnimParam(%d, %d) = %d", bc, hd,
                  InAnimParam(bc, hd));
    {
        // Queued handler write, override on, requested 2: forced and recorded like a broadcast write, not marked outside.
        World w(1);
        const SetterOutcome o = w.Set(0, InAnimParam(0, 1), true, kPlaneWeapon);
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(o.plane == kPlaneScene && o.recorded && e && !e->outsideLast && e->requested == kPlaneWeapon,
              "handler path: queued write plane %d recorded %d outside %d requested %d (expected 0, 1, 0, 2)", o.plane,
              o.recorded, e ? e->outsideLast : -1, e ? e->requested : -1);
    }
    {
        // The NG1 sequence: registered by the broadcast while the override was off (2), the override turns on (re-apply -> 0),
        // then the owner's queued event lands with 2. It must not overwrite the forced 0 or take the entry away from the re-apply.
        World w(1);
        w.Set(0, InAnimParam(1, 1), false, kPlaneWeapon); // listener path: broadcast frame calling the handler
        w.Reapply(true);
        const SetterOutcome q = w.Set(0, InAnimParam(0, 1), true, kPlaneWeapon); // queued event, override on
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(q.plane == kPlaneScene && w.comps[0].plane == kPlaneScene && e && !e->outsideLast,
              "handler path: queued write after the edge wrote %d, component %d, outside %d (expected 0, 0, 0)", q.plane,
              w.comps[0].plane, e ? e->outsideLast : -1);
        // The re-apply still owns it: with the component poked back to 2, the force step writes 0.
        w.comps[0].plane = kPlaneWeapon;
        const ReapplyStep s = e ? PlanReapply(true, *e, w.comps[0].plane) : ReapplyStep{};
        CHECK(s.write && s.plane == kPlaneScene, "handler path: re-apply step write %d plane %d (expected 1, 0)", s.write, s.plane);
    }

    // 8. Round 2, SPECIAL-2: an outside write that agrees with the policy clears outsideLast, a disagreeing one sets it.
    {
        // Agree while on (Correct(true) writes 0 in TPP) after a disagreeing write: cleared; the next edge is re-applied.
        World w(1);
        w.Set(0, true, false, kPlaneWeapon);  // registered, requested 2
        w.Set(0, false, true, kPlaneWeapon);  // outside 2 while forcing: disagrees -> marked
        w.Set(0, false, true, kPlaneScene);   // Correct(true): 0 while forcing -> agrees -> cleared
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(e && !e->outsideLast, "agree rows: agree-while-on kept the outside mark");
        const int v = w.Reapply(false); // edge to FPP
        CHECK(v == 1 && w.comps[0].plane == kPlaneWeapon, "agree rows: edge after agree-while-on wrote %d, plane %d (expected 1, 2)",
              v, w.comps[0].plane);
    }
    {
        // Agree while off (Correct(false) writes 2 in FPP) after a disagreeing write: cleared; the next edge is re-applied.
        World w(1);
        w.Set(0, true, false, kPlaneWeapon);
        w.Set(0, false, false, kPlaneScene);  // stale 0 while off: disagrees -> marked
        w.Set(0, false, false, kPlaneWeapon); // Correct(false): 2 while off -> agrees -> cleared
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(e && !e->outsideLast, "agree rows: agree-while-off kept the outside mark");
        const int f = w.Reapply(true); // edge to TPP
        CHECK(f == 1 && w.comps[0].plane == kPlaneScene, "agree rows: edge after agree-while-off wrote %d, plane %d (expected 1, 0)",
              f, w.comps[0].plane);
    }
    {
        // Stale 0 while off (an event queued before the edge lands after it): disagrees, marked, the re-apply leaves it alone.
        World w(1);
        w.Set(0, true, false, kPlaneWeapon);
        w.Set(0, false, false, kPlaneScene);
        const Entry* e = w.reg.Find(w.Inst(0), w.comps[0].refCnt);
        CHECK(e && e->outsideLast && e->requested == kPlaneWeapon, "stale 0 while off: outside %d requested %d (expected 1, 2)",
              e ? e->outsideLast : -1, e ? e->requested : -1);
        const int v = w.Reapply(false);
        CHECK(v == 0 && w.comps[0].plane == kPlaneScene, "stale 0 while off rewritten by the re-apply: writes %d plane %d", v,
              w.comps[0].plane);
    }
    {
        // Scoped ADS round trip (aimingScoped=fpp): TPP -> FPP -> TPP with UTPP's own Correct() events between the edges. Every
        // edge is re-applied immediately (no RPl_Weapon frame in TPP after the exit).
        World w(1);
        w.Set(0, true, true, kPlaneWeapon);   // TPP: broadcast 2 -> 0, registered
        w.Set(0, false, true, kPlaneScene);   // Correct(true)
        w.Reapply(false);                     // ADS enter: FPP edge
        const int32_t fpp = w.comps[0].plane;
        w.Set(0, false, false, kPlaneWeapon); // Correct(false): restore event
        w.Reapply(true);                      // ADS exit: TPP edge
        CHECK(fpp == kPlaneWeapon && w.comps[0].plane == kPlaneScene,
              "agree rows: scoped ADS round trip planes FPP %d, TPP %d (expected 2, 0)", fpp, w.comps[0].plane);
    }

    // 9. Detached component (owner == nullptr; crash 2026-11-12: an FPP garment part still referenced but detached after the return
    //    from scoped aim). The re-apply must not call the real setter for it, must count it, must keep the entry (it may be
    //    attached again) and must still re-apply the live components.
    {
        World w(3);
        w.Set(0, true, false, kPlaneWeapon); // live
        w.Set(1, true, false, kPlaneWeapon); // detached
        w.Set(2, true, false, kPlaneWeapon); // live
        w.comps[1].attached = false;
        const int f = w.Reapply(true);
        CHECK(w.comps[1].setterCalls == 0 && w.comps[1].plane == kPlaneWeapon,
              "detached component reached the setter: calls %d plane %d (expected 0, 2)", w.comps[1].setterCalls, w.comps[1].plane);
        CHECK(w.orphans == 1, "detached component not counted: orphans %d (expected 1)", w.orphans);
        CHECK(f == 2 && w.comps[0].plane == kPlaneScene && w.comps[2].plane == kPlaneScene && w.comps[0].setterCalls == 1 &&
                  w.comps[2].setterCalls == 1,
              "live components not re-applied next to a detached one: writes %d planes %d %d (expected 2, 0 0)", f,
              w.comps[0].plane, w.comps[2].plane);
        CHECK(w.reg.Find(w.Inst(1), w.comps[1].refCnt) != nullptr, "detached entry dropped from the registry");
        // Attached again (representation back): the kept entry is re-applied on the next pass; the others already match.
        w.comps[1].attached = true;
        const int v = w.Reapply(true);
        CHECK(v == 1 && w.comps[1].plane == kPlaneScene && w.comps[1].setterCalls == 1 && w.orphans == 1,
              "re-attached entry not re-applied: writes %d plane %d calls %d orphans %d (expected 1, 0, 1, 1)", v,
              w.comps[1].plane, w.comps[1].setterCalls, w.orphans);
    }
    {
        // A detached component whose plane already matches needs no write: not reported as skipped (nothing was prevented).
        World w(1);
        w.Set(0, true, true, kPlaneWeapon); // written 0 already
        w.comps[0].attached = false;
        w.Reapply(true);
        CHECK(w.orphans == 0 && w.comps[0].setterCalls == 0, "detached component without a due write counted: orphans %d calls %d",
              w.orphans, w.comps[0].setterCalls);
        // Pure step: attached or not, the decision is PlanReapply's; the orphan flag only marks a prevented write.
        Entry e;
        e.requested = kPlaneWeapon;
        const ReapplyStep a = PlanReapplyAttached(true, e, kPlaneWeapon, true);
        const ReapplyStep d = PlanReapplyAttached(true, e, kPlaneWeapon, false);
        CHECK(a.write && !a.orphan && a.plane == kPlaneScene, "attached step: write %d orphan %d plane %d (expected 1, 0, 0)", a.write,
              a.orphan, a.plane);
        CHECK(!d.write && d.orphan && d.plane == kPlaneWeapon, "detached step: write %d orphan %d plane %d (expected 0, 1, 2)", d.write,
              d.orphan, d.plane);
    }

    std::printf("ItemPlaneCore: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

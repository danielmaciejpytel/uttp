// ragdoll production core: episode state machine, owner flag, settle detection, pelvis telemetry and the MM hand-over feed
// (src/ragdoll/RagdollEpisode.hpp). Offline, plain C++, injected time.
// Mutations (build with -DUTPP_RAGDOLL_EPISODE_MUTATION=k, CMake utpp_ragdoll_episode_mut1/2/3):
//   1 the blend-out timer starts at Release()   -> "FAIL blend-out timer starts at the gate edge"
//   2 Abort() leaves the owner flag set         -> "FAIL abort clears the owner flag"
//   3 a late impact does not re-arm the settle  -> "FAIL late impact re-arms the settle hold"
//   4 the forced blend-out (stuck release) bounces back to Active while the gate stays up -> "FAIL forced blend-out ends"
//   5 a missing slot reading clears the Abort lockout -> "FAIL abort lockout survives an unknown gate"
//   6 any component can become the production slot cache -> "FAIL slot cache binds only the known player"
//   7 the pelvis indices of one rig are reused for every rig -> "FAIL pelvis indices are per rig"
//   8 HandToGetUp does not arm the stuck-gate lockout -> "FAIL hand-off to the get-up keeps the stuck gate from restarting the episode"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "../src/ragdoll/RagdollEpisode.hpp"
#include "../src/ragdoll/RagdollGuards.hpp"

using namespace utpp::ragdoll;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
}

bool Is(const Episode& e, EpState s) { return e.State() == s; }

void TestIdleAndGate()
{
    Episode e;
    Check(Is(e, EpState::Idle) && !e.Owns(), "starts idle, owner flag clear");
    e.Tick(false, 0.0);
    Check(Is(e, EpState::Idle) && !e.Owns(), "idle without the gate stays idle");
    e.Tick(true, 1.0);
    Check(Is(e, EpState::Active) && e.Owns(), "the gate alone starts an episode (probe-driven ragdoll)");
    e.Tick(true, 2.0);
    Check(Is(e, EpState::Active) && e.Owns(), "gate held: stays active");
    e.Tick(false, 3.0);
    Check(Is(e, EpState::BlendOut) && e.Owns(), "the gate falling enters the blend-out; the owner flag stays set");
    e.Tick(false, 3.0 + 0.65);
    Check(Is(e, EpState::Idle) && !e.Owns(), "the blend-out timer ends the episode and clears the flag");
    Check(e.Counters().episodes == 1, "one episode counted");
}

void TestBegin()
{
    Episode e;
    Check(!e.Begin(0.0, false) && Is(e, EpState::Idle), "Begin refused without hook / player");
    Check(e.Begin(0.0) && Is(e, EpState::Active), "Begin arms the episode (state 1)");
    Check(!e.Owns(), "an unconfirmed episode does not own the body yet");
    e.Tick(false, 0.5);
    Check(Is(e, EpState::Active) && !e.Owns(), "no gate yet inside the confirm window");
    e.Tick(true, 0.6);
    Check(Is(e, EpState::Active) && e.Owns(), "the gate confirms the episode and sets the owner flag");
    Check(e.Begin(0.7) && Is(e, EpState::Active), "Begin while active is idempotent");

    Episode t;
    t.Begin(0.0);
    t.Tick(false, 1.2);
    Check(Is(t, EpState::Idle) && !t.Owns() && t.Counters().confirmTimeouts == 1, "Begin without a gate rise is abandoned after 1 s");
    t.Tick(false, 1.3);
    Check(Is(t, EpState::Idle), "stays idle after the confirm timeout");
}

void TestBlendOut()
{
    Episode e;
    e.Begin(0.0);
    e.Tick(true, 0.1);
    e.Release(1.0);
    Check(Is(e, EpState::Active) && e.Owns(), "Release only records the intent while the gate is up");
    e.Tick(false, 1.2); // the real 1 -> 0 edge, one tick after the disable event
    Check(Is(e, EpState::BlendOut), "the gate edge enters the blend-out");
    e.Tick(false, 1.2 + 0.6);
    const bool stillOwned = e.Owns() && Is(e, EpState::BlendOut);
    Check(stillOwned, "blend-out timer starts at the gate edge", "owned at edge+0.6=" + std::to_string(stillOwned));
    e.Tick(false, 1.2 + 0.66);
    Check(Is(e, EpState::Idle) && !e.Owns(), "blend-out ends 0.65 s after the edge");

    // the gate rises again during the blend-out: a new activation, back to active, no gap in ownership
    Episode r;
    r.Tick(true, 0.0);
    r.Tick(false, 1.0);
    r.Tick(true, 1.3);
    Check(Is(r, EpState::Active) && r.Owns() && r.Counters().reactivations == 1, "gate rising in the blend-out returns to active");

    // a Begin during the blend-out keeps ownership until the gate rises
    Episode b;
    b.Tick(true, 0.0);
    b.Tick(false, 1.0);
    Check(b.Begin(1.2) && Is(b, EpState::BlendOut) && b.Owns(), "Begin during the blend-out keeps ownership");
    b.Tick(false, 1.8);
    Check(Is(b, EpState::BlendOut) && b.Owns(), "the blend-out is extended for the new request");
    b.Tick(true, 2.0);
    Check(Is(b, EpState::Active) && b.Owns(), "the new activation takes over");

    // Release before the gate ever rose cancels the episode
    Episode c;
    c.Begin(0.0);
    c.Release(0.2);
    Check(Is(c, EpState::Idle) && !c.Owns(), "Release before activation cancels");

    // Release but the gate never falls: blend out anyway
    Episode s;
    s.Tick(true, 0.0);
    s.Release(1.0);
    s.Tick(true, 2.0);
    Check(Is(s, EpState::Active), "stuck gate inside the release grace");
    s.Tick(true, 2.6);
    Check(Is(s, EpState::BlendOut) && s.Counters().releaseStuck == 1, "a gate that never falls after Release blends out anyway");

    // ... and the still-raised gate must not bounce the forced blend-out back to Active (ownership would persist to the 30 s maximum)
    s.Tick(true, 2.7);
    const bool bounced = Is(s, EpState::Active);
    Check(Is(s, EpState::BlendOut) && s.Owns() && s.Counters().reactivations == 0, "forced blend-out ignores the raised gate",
          "state=" + std::to_string(static_cast<int>(s.State())));
    s.Tick(true, 3.0);
    Check(Is(s, EpState::BlendOut) && s.Owns(), "forced blend-out keeps blending while the gate stays up");
    s.Tick(true, 2.6 + 0.66);
    const bool released = Is(s, EpState::Idle) && !s.Owns();
    Check(released, "forced blend-out ends", "bounced=" + std::to_string(bounced) + " idle=" + std::to_string(released));
    s.Tick(true, 4.0);
    s.Tick(true, 10.0);
    Check(Is(s, EpState::Idle) && !s.Owns() && s.Counters().episodes == 1, "a gate that stays up after the forced end does not restart it");
    s.Tick(false, 11.0);
    s.Tick(true, 12.0);
    Check(Is(s, EpState::Active) && s.Owns() && s.Counters().episodes == 2, "after the gate fell a new activation starts a new episode");

    // an unknown reading (cache dropped) is not "the gate fell": the forced blend-out keeps ignoring the gate afterwards
    Episode u;
    u.Tick(true, 0.0);
    u.Release(1.0);
    u.Tick(true, 2.6);
    u.Tick(false, 2.65, false);
    u.Tick(true, 2.7);
    Check(Is(u, EpState::BlendOut) && u.Counters().reactivations == 0, "an unknown reading does not end the forced blend-out's gate-ignore");

    // a gate seen low during the forced blend-out ends the stuck state: a later rise is a genuine new activation
    Episode f;
    f.Tick(true, 0.0);
    f.Release(1.0);
    f.Tick(true, 2.6);
    f.Tick(false, 2.7);
    f.Tick(true, 2.8);
    Check(Is(f, EpState::Active) && f.Counters().reactivations == 1, "gate low then high in the forced blend-out reactivates");

    // a Begin during the forced blend-out is a new request: the gate may reactivate
    Episode fb;
    fb.Tick(true, 0.0);
    fb.Release(1.0);
    fb.Tick(true, 2.6);
    fb.Begin(2.7);
    fb.Tick(true, 2.8);
    Check(Is(fb, EpState::Active) && fb.Owns(), "Begin during the forced blend-out re-enables the reactivation");
}

void TestMaxAndAbort()
{
    Episode e;
    e.Tick(true, 0.0);
    e.Tick(true, 29.0);
    Check(Is(e, EpState::Active), "inside the hard maximum");
    e.Tick(true, 31.0);
    Check(Is(e, EpState::Idle) && !e.Owns() && e.Counters().maxTimeouts == 1, "hard maximum ends the episode");
    e.Tick(true, 32.0);
    Check(Is(e, EpState::Idle), "a gate that is still up does not restart it (lockout)");
    e.Tick(false, 33.0);
    e.Tick(true, 34.0);
    Check(Is(e, EpState::Active), "after the gate fell a new activation starts a new episode");

    Episode a;
    a.Tick(true, 0.0);
    a.Abort(1.0);
    const bool owned = a.Owns();
    Check(!owned && Is(a, EpState::Idle), "abort clears the owner flag", "owner=" + std::to_string(owned));
    Check(a.Counters().aborts == 1, "abort counted");
    a.Abort(1.1);
    Check(a.Counters().aborts == 1, "abort while idle is a no-op");

    Episode g;
    g.Tick(true, 0.0);
    g.Abort(1.0); // the disable event has not taken effect yet: the gate is still up
    g.Tick(true, 1.016);
    g.Tick(true, 1.032);
    Check(Is(g, EpState::Idle) && !g.Owns(), "abort sticks while the gate is still up");
    g.Tick(false, 1.5);
    g.Tick(true, 2.0);
    Check(Is(g, EpState::Active) && g.Owns(), "a new activation after the gate fell starts a new episode");
    g.Abort(2.1);
    Check(g.Begin(2.2) && Is(g, EpState::Active), "Begin after an abort is accepted (clears the lockout)");

    Episode d;
    d.Tick(true, 0.0);
    d.Tick(false, 1.0);
    d.Abort(1.1);
    Check(Is(d, EpState::Idle) && !d.Owns(), "abort during the blend-out ends it at once");
    d.Tick(false, 2.0);
    Check(Is(d, EpState::Idle), "and it stays idle");

    // OnDisarm / SetPlayerHash: Abort, then the slot cache is dropped and the next reading is "unknown" (not "gate low")
    Episode k;
    k.Tick(true, 0.0);
    k.Abort(1.0);
    k.Tick(false, 1.016, false); // OnFrame right after DropSlotCache
    k.Tick(true, 1.032);         // the job tick found the slot again; the gate is still up
    const bool restarted = !Is(k, EpState::Idle);
    Check(!restarted && !k.Owns(), "abort lockout survives an unknown gate", "restarted=" + std::to_string(restarted));
    k.Tick(false, 1.5); // a real low reading
    k.Tick(true, 2.0);
    Check(Is(k, EpState::Active) && k.Owns(), "a known low gate clears the lockout");
}

void TestSlotBinding()
{
    constexpr uint64_t kPlayer = 0x1234, kNpc = 0x9999;
    Check(CanBindSlotCache(kPlayer, kPlayer), "the known player's component is bound");
    const bool npc = CanBindSlotCache(kPlayer, kNpc);
    Check(!npc, "slot cache binds only the known player", "npc=" + std::to_string(npc));
    Check(!CanBindSlotCache(0, kNpc), "an NPC component is not bound while the player id is unknown");
    Check(!CanBindSlotCache(0, 0), "nothing is bound without any id");
    Check(!CanBindSlotCache(kPlayer, 0), "an unreadable owner is not bound");
}

void TestPelvisRigs()
{
    using utpp::ragdoll::PelvisRigCache;
    const uint64_t wanted[4] = {11, 22, 33, 44}; // Hips, Spine, LeftUpLeg, RightUpLeg
    // the merged rig (619) and the larger one (645) order the bones differently
    const uint64_t rigA[] = {5, 11, 22, 33, 44, 6};
    const uint64_t rigB[] = {44, 33, 7, 8, 22, 11, 9};
    int32_t out[4];
    Check(utpp::ragdoll::ResolvePelvisIndices(rigA, 6, wanted, out) && out[0] == 1 && out[1] == 2 && out[2] == 3 && out[3] == 4,
          "bone names resolve to indices");
    const uint64_t missing[] = {11, 22, 33};
    Check(!utpp::ragdoll::ResolvePelvisIndices(missing, 3, wanted, out) && out[3] == -1, "a rig without one of the bones does not resolve");

    PelvisRigCache c;
    const auto resolver = [&](uint64_t aRig, uint32_t aCount, int32_t (&aOut)[4]) {
        const uint64_t* names = aRig == 0xA ? rigA : rigB;
        return utpp::ragdoll::ResolvePelvisIndices(names, aCount, wanted, aOut);
    };
    int32_t a[4], b[4];
    Check(c.Lookup(0xA, 6, resolver, a) == PelvisRigCache::Result::Ok && a[0] == 1, "first rig resolved");
    Check(c.Lookup(0xB, 7, resolver, b) == PelvisRigCache::Result::Ok, "second rig resolved");
    Check(b[0] == 5 && b[1] == 4 && b[2] == 1 && b[3] == 0, "pelvis indices are per rig",
          "b=" + std::to_string(b[0]) + "," + std::to_string(b[1]) + "," + std::to_string(b[2]) + "," + std::to_string(b[3]));
    int32_t again[4];
    c.Lookup(0xA, 6, resolver, again);
    Check(again[0] == 1 && c.Resolutions() == 2, "a known rig is served from the cache", "resolutions=" + std::to_string(c.Resolutions()));
    Check(c.Lookup(0xA, 5, resolver, again) != PelvisRigCache::Result::Busy && c.Resolutions() == 3,
          "the same pointer with another bone count is a new rig");

    PelvisRigCache n;
    const auto none = [&](uint64_t, uint32_t, int32_t (&)[4]) { return false; };
    Check(n.Lookup(0xC, 3, none, a) == PelvisRigCache::Result::Unresolved, "an unresolvable rig is reported");
    n.Lookup(0xC, 3, none, a);
    n.Lookup(0xC, 3, none, a);
    Check(n.Resolutions() == 1, "an unresolvable rig is cached (one attempt)");
    n.Clear();
    n.Lookup(0xC, 3, none, a);
    Check(n.Resolutions() == 2, "Clear forgets the cache (re-arm / pointer reuse)");
    Check(n.Lookup(0, 3, none, a) == PelvisRigCache::Result::Unresolved, "a null rig never resolves");
}

void TestGameOwnedFeed()
{
    using namespace utpp::mm::integ;
    const uint32_t raw = kPsmReaction | kPsmMeleePose | (static_cast<uint32_t>(5 + 1) << kPsmDetailShift) | kPsmWeaponKnown;
    const uint32_t g = GameOwnedPsmBlock(raw);
    Check(PsmDetailOf(g) == kGameOwnedDetail && (g & kPsmDetailed) != 0, "the feed carries LocomotionDetailed 29 with the detailed bit");
    Check(PsmDetailReleases(PsmDetailOf(g)), "that detail is the existing release-to-game state of knockdown");
    Check((g & kPsmReaction) != 0 && (g & kPsmMeleePose) != 0 && (g & kPsmWeaponKnown) != 0, "the other feed bits are preserved");
    Check(PsmDetailOf(GameOwnedPsmBlock(0)) == 29, "also from an empty block");
    Check(!GameOwned(), "the process-wide flag is clear in the test");
}

void TestSmoothSpeed()
{
    SmoothSpeed s;
    float p[3] = {0, 0, 0};
    float v = 0;
    for (int i = 0; i < 120; ++i) // 1 m/s along X for 2 s at 60 Hz
    {
        p[0] = static_cast<float>(i) / 60.0f;
        v = s.Step(p, i / 60.0);
    }
    Check(std::fabs(v - 1.0f) < 0.02f, "constant velocity converges", std::to_string(v));
    for (int i = 120; i < 180; ++i)
        v = s.Step(p, i / 60.0);
    Check(v < 0.02f, "standing still decays to zero", std::to_string(v));
    p[0] += 10.0f;
    v = s.Step(p, 100.0); // long gap: restart, no spike
    Check(v == 0.0f, "a long gap restarts the filter without a spike");
}

void TestSettle()
{
    SettleDetector d;
    bool settled = false;
    for (int i = 0; i <= 9; ++i)
        settled = d.Update(i * 0.1, 0.0f);
    Check(!settled, "not settled before the minimum time");
    settled = d.Update(1.0, 0.0f);
    Check(settled, "settled once calm for the hold time after the minimum");

    SettleDetector late;
    bool s2 = false;
    for (int i = 0; i <= 8; ++i)
        s2 = late.Update(i * 0.1, 0.0f);
    late.Update(0.9, 0.5f); // a late impact
    s2 = late.Update(1.0, 0.0f);
    const bool atOne = s2;
    s2 = late.Update(1.4, 0.0f);
    const bool atFourTenths = s2;
    Check(!atOne && !atFourTenths, "late impact re-arms the settle hold", "at1.0=" + std::to_string(atOne) + " at1.4=" + std::to_string(atFourTenths));
    Check(late.Update(1.5, 0.0f), "settles 0.5 s after the late impact calmed down");

    SettleDetector begun;
    begun.Begin(10.0); // the episode started at t = 10; the first script call comes late
    Check(!begun.Update(10.4, 0.0f), "settle clock runs from the episode start (not before the minimum time)");
    Check(begun.Update(11.0, 0.0f), "settled 1 s after the episode start when calm");
    Episode ep;
    ep.Tick(true, 7.5);
    Check(ep.StartedAt() == 7.5, "episode start time is exposed");

    SettleDetector forced;
    bool f = false;
    for (int i = 0; i <= 50; ++i)
        f = forced.Update(i * 0.1, 1.0f);
    Check(f, "a body that never calms down is forced settled at the maximum");
}

void SetPose(float (&h)[3], float (&s)[3], float (&l)[3], float (&r)[3], const float (&u)[3], const float (&lr)[3], float aHipZ)
{
    h[0] = 0; h[1] = 0; h[2] = aHipZ;
    for (int i = 0; i < 3; ++i)
    {
        s[i] = h[i] + u[i];
        l[i] = h[i] - 0.5f * lr[i];
        r[i] = h[i] + 0.5f * lr[i];
    }
}

void TestFaceUp()
{
    float h[3], s[3], l[3], r[3];
    const float right[3] = {0.2f, 0, 0};
    const float standUp[3] = {0, 0, 0.2f};
    SetPose(h, s, l, r, standUp, right, 1.0f);
    PelvisTracker t;
    Check(!t.HasReference(), "no standing reference at first");
    t.SetReference(h, s, l, r);
    Check(t.HasReference(), "the standing sample sets the reference");

    auto classify = [&](const float (&up)[3], const float (&lr)[3]) {
        float hh[3], ss[3], ll[3], rr[3];
        SetPose(hh, ss, ll, rr, up, lr, 0.2f);
        t.Feed(hh, ss, ll, rr, 1000, 1000);
        PelvisSnapshot p;
        if (!t.Read(p, 1100, 1000))
            return -2;
        return t.FaceUp(p);
    };
    const float headBack[3] = {0, -0.2f, 0};
    const float headFwd[3] = {0, 0.2f, 0};
    const float left[3] = {-0.2f, 0, 0};
    const float sideUp[3] = {0.2f, 0, 0};
    const float sideRight[3] = {0, 0, -0.2f};
    Check(classify(headBack, right) == 1, "lying on the back is face up");
    Check(classify(headFwd, right) == 0, "lying on the belly is face down");
    Check(classify(headFwd, left) == 1, "head forward with the legs swapped: belly up again");
    Check(classify(sideUp, sideRight) == -1, "lying on the side is unknown");
    Check(classify(standUp, right) == -1, "standing is not face up / down");

    // a rig whose leg order is mirrored: the standing reference fixes the sign
    PelvisTracker m;
    SetPose(h, s, l, r, standUp, left, 1.0f);
    m.SetReference(h, s, l, r);
    float hh[3], ss[3], ll[3], rr[3];
    SetPose(hh, ss, ll, rr, headBack, left, 0.2f);
    m.Feed(hh, ss, ll, rr, 1000, 1000);
    PelvisSnapshot p;
    Check(m.Read(p, 1050, 1000) && m.FaceUp(p) == 1, "a mirrored rig is signed against its own standing pose");
    Check(!m.Read(p, 1600, 1000), "a sample older than 0.5 s is not fresh");

    // no reference -> unknown
    PelvisTracker n;
    SetPose(hh, ss, ll, rr, headBack, right, 0.2f);
    n.Feed(hh, ss, ll, rr, 1000, 1000);
    Check(n.Read(p, 1010, 1000) && n.FaceUp(p) == -1, "without a standing reference the face is unknown");
    Check(ClassifyFaceUp(0.49f) == -1 && ClassifyFaceUp(0.5f) == 1 && ClassifyFaceUp(-0.5f) == 0, "dead zone of the classifier");
}

void TestPelvisSpeed()
{
    PelvisTracker t;
    float h[3] = {0, 0, 0.5f}, s[3] = {0, 0, 0.7f}, l[3] = {-0.1f, 0, 0.5f}, r[3] = {0.1f, 0, 0.5f};
    float v = 0;
    PelvisSnapshot p;
    for (int i = 0; i < 90; ++i) // 2 m/s along Y at 60 Hz (ticks in 1/1000 s)
    {
        const float dy = 2.0f * static_cast<float>(i) / 60.0f;
        h[1] = dy; s[1] = dy; l[1] = dy; r[1] = dy;
        t.Feed(h, s, l, r, static_cast<int64_t>(i * 1000000.0 / 60.0), 1000000); // ticks in microseconds
        if (t.Read(p, static_cast<int64_t>(i * 1000000.0 / 60.0), 1000000))
            v = p.speed;
    }
    Check(std::fabs(v - 2.0f) < 0.05f, "pelvis speed converges to the true speed", std::to_string(v));
    Check(std::fabs(p.pos[1] - 2.0f * 89.0f / 60.0f) < 1e-3f, "pelvis position published");
}
// native get-up: the episode is handed over, the still-raised gate must not restart it; the owner flag is withheld while the get-up runs
void TestGetUpHandOff()
{
    Episode e;
    e.Tick(true, 1.0);
    e.Tick(true, 1.2);
    Check(Is(e, EpState::Active) && e.Owns(), "get-up: setup - a confirmed episode");
    e.HandToGetUp(1.3);
    Check(Is(e, EpState::Idle) && !e.Owns(), "get-up: HandToGetUp ends the episode at once (Idle, owner flag clear)");
    e.Tick(true, 1.31);
    e.Tick(true, 1.40);
    Check(Is(e, EpState::Idle) && !e.Owns(), "hand-off to the get-up keeps the stuck gate from restarting the episode");
    e.Tick(false, 1.5); // the disable event is taken: the gate was seen low
    e.Tick(true, 2.0);  // a new knockdown
    Check(Is(e, EpState::Active) && e.Owns(), "a new gate rise after the gate was seen low starts a new episode");
    // from BlendOut too
    Episode b;
    b.Tick(true, 0.0);
    b.Tick(false, 0.1);
    Check(Is(b, EpState::BlendOut) && b.Owns(), "get-up: setup - blend-out");
    b.HandToGetUp(0.2);
    b.Tick(true, 0.25);
    Check(Is(b, EpState::Idle) && !b.Owns(), "get-up: HandToGetUp from BlendOut also ends it and ignores the raised gate");
    Check(e.Counters().getUpHandOffs == 1 && b.Counters().getUpHandOffs == 1, "get-up: the hand-off is counted");
    // the global owner flag: a running get-up withholds the writers although the episode owns the body
    Check(!GetUpOwns() && !GameOwned(), "get-up: the process-wide flags are clear in the test");
    g_episode.Tick(true, 10.0);
    Check(GameOwned(), "get-up: the episode alone owns the body (writers withheld)");
    g_getUpOwns.store(true);
    Check(g_episode.Owns() && !GameOwned(), "get-up: GetUpOwns() makes GameOwned() false even with a confirmed episode (writers resume)");
    g_getUpOwns.store(false);
    g_episode.Abort(11.0);
    // the feed carries no knockdown detail during the get-up
    using namespace utpp::mm::integ;
    const uint32_t raw = kPsmVehicle | kPsmWeaponKnown | (static_cast<uint32_t>(29 + 1) << kPsmDetailShift) | kPsmDetailed;
    const uint32_t g = GetUpPsmBlock(raw);
    Check(PsmDetailOf(g) == -1 && (g & kPsmDetailed) == 0 && (g & kPsmVehicle) != 0 && (g & kPsmWeaponKnown) != 0,
          "get-up: GetUpPsmBlock removes the injected detail and keeps every other bit (vehicle still closes the gate)");
    Check(GetUpPsmBlock(GameOwnedPsmBlock(0)) == 0, "get-up: GetUpPsmBlock undoes GameOwnedPsmBlock");
    // v1 -> v2 abort request: one consumer
    RequestGetUpAbort(kGetUpAbortBaseGate);
    Check(TakeGetUpAbortRequest() == kGetUpAbortBaseGate && TakeGetUpAbortRequest() == 0, "get-up: the base abort request is consumed once");
}

// section 5: the post-ragdoll window opens when the knockdown detail ends
void TestRecoverWindow()
{
    RecoverWindow w;
    for (int i = 0; i < 10; ++i)
        w.Step(1, 0.0166f);
    Check(!w.Active(), "recover window: closed while the detail never was the knockdown");
    for (int i = 0; i < 10; ++i)
        w.Step(29, 0.0166f);
    Check(!w.Active(), "recover window: closed during the knockdown");
    w.Step(1, 0.0166f);
    Check(w.Active(), "recover window: opens when the detail leaves 29");
    float t = 0;
    int n = 0;
    while (w.Active() && n < 1000)
    {
        w.Step(1, 0.0166f);
        t += 0.0166f;
        ++n;
    }
    Check(std::fabs(t - 1.5f) < 0.05f, "recover window: lasts 1.5 s of feed time", std::to_string(t));
    w.Step(29, 0.0166f);
    w.Step(1, 0.0166f);
    w.Step(29, 0.0166f);
    Check(!w.Active(), "recover window: a new knockdown closes it");
    Check(!RagdollRecoverActive(), "recover window: the process-wide state is idle in the test");
}
} // namespace

int main()
{
    TestIdleAndGate();
    TestBegin();
    TestBlendOut();
    TestMaxAndAbort();
    TestSlotBinding();
    TestPelvisRigs();
    TestGameOwnedFeed();
    TestSmoothSpeed();
    TestSettle();
    TestFaceUp();
    TestPelvisSpeed();
    TestGetUpHandOff();
    TestRecoverWindow();
    std::printf("RagdollEpisodeTest: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

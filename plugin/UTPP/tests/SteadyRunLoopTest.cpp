// G10: the steady Run gait between the walk cap and the run loops (analog stick 0.7-0.82: chooser speed 247 / 320 cm/s) on the real v2 database
// plays Walk_Loop_* (up to the walk-loop pace) and Run_Loop_* clips with no pose reset, not the non-loop Walk_to_Run transition clips
// (MatcherConfig::steadyRunLoops). Evidence: the recorded log (game feed 15:44, analog ramp), replayed through the installed bake.
// Usage: utpp_mm_steadyrun_test <utpp_motion_woman*.utppmm>   (chooser.json next to it)
// Path: synthetic feed -> adapter::MapToChooser (production MapConfig defaults) -> BuildQueryTrajectory -> ChooserMatcher::Update.
// Mutants of the fix are the config switches themselves (a rule off must fail the check that pins it).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "../src/mm/MotionAdapter.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
static int passed = 0, failed = 0;
#define CHECK(c, ...) do { if (c) ++passed; else { ++failed; std::printf("FAIL " __VA_ARGS__); std::printf("\n"); } } while (0)

struct Step1
{
    std::string clip;
    float rate = 1.f;
    float clipTime = 0.f;
    bool jumped = false;
    int gait = 0;
    float speedCmS = 0.f;
};

struct Leg
{
    float speedMs, stick, seconds;
};

struct Rig
{
    MotionDatabase db;
    chooser::Table table;
    ChooserMatcher matcher;
    adapter::MapConfig mc;
    adapter::MapState ms;
    adapter::QueryTimes qtimes;
    std::vector<TrajSample> traj;
    float dt = 1.f / 60.f;
    bool Load(const char* p)
    {
        std::string err, chooserPath(p);
        chooserPath.replace(chooserPath.find_last_of('.'), std::string::npos, ".chooser.json");
        if (!db.LoadFromFile(p, kRigIdentityWoman, err) || !table.LoadFile(chooserPath.c_str(), err))
        { std::printf("LOAD %s\n", err.c_str()); return false; }
        qtimes = adapter::BuildQueryTimes(db);
        return true;
    }
    bool Reset(const MatcherConfig& cfg, float togglePoint = 0.f)
    {
        std::string err;
        ms = adapter::MapState{};
        mc = adapter::MapConfig{};
        mc.stanceFilterS = adapter::kStanceFilterSeconds;
        mc.stanceHoldInDodge = true;
        mc.stanceCleanEdge = true;
        mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
        mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
        mc.gameWalkTogglePoint = togglePoint;
        return matcher.Init(&db, &table, cfg, err);
    }
    Step1 Step(float speedMs, float stick, bool walk, int psm = adapter::kPsmStand)
    {
        const float vel[3] = {0, speedMs, 0}, mv[2] = {0, stick};
        adapter::MotionFeed f = adapter::MakeMotionFeed(vel, mv, dt, 0u, 0, adapter::AirFeed{}, 0.f, 0.f);
        f.wantsToWalk = walk;
        f.psmDetailed = psm;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = 0.f;
        const ChooserState cs = adapter::MapToChooser(f, mc, ms, ctx);
        traj.resize(adapter::kQueryTrajMax);
        const uint32_t nq = adapter::BuildQueryTrajectory(ms, qtimes, traj.data(), adapter::kQueryTrajMax);
        traj.resize(nq);
        const MatchResult& r = matcher.Update(dt, traj.data(), nq, cs);
        Step1 o;
        if (r.clip >= 0)
            o.clip = std::string(db.ClipName(uint32_t(r.clip)));
        o.rate = r.playRate;
        o.clipTime = r.clipTime;
        o.jumped = r.jumped;
        o.gait = cs.gait;
        o.speedCmS = cs.speed2dCmS;
        return o;
    }
    // 0.5 s of rest, then every leg: a 0.4 s ramp from the previous speed (the game steps its speed with the stick), then the steady speed
    std::vector<Step1> Run(const std::vector<Leg>& legs, bool walk = false, int psm = adapter::kPsmStand)
    {
        std::vector<Step1> out;
        for (int i = 0; i < int(std::lround(0.5f / dt)); ++i)
            out.push_back(Step(0.f, 0.f, walk, psm));
        float prev = 0.f;
        for (const Leg& l : legs)
        {
            const int n = int(std::lround(l.seconds / dt));
            for (int i = 0; i < n; ++i)
                out.push_back(Step(prev + (l.speedMs - prev) * std::fmin(1.f, float(i) * dt / 0.4f), l.stick, walk, psm));
            prev = l.speedMs;
        }
        return out;
    }
};

static bool IsLoop(const std::string& c) { return c.rfind("M_Neutral_Walk_Loop_", 0) == 0 || c.rfind("M_Neutral_Run_Loop_", 0) == 0; }
static bool IsWalkLoop(const std::string& c) { return c.rfind("M_Neutral_Walk_Loop_", 0) == 0; }
static bool IsRunLoop(const std::string& c) { return c.rfind("M_Neutral_Run_Loop_", 0) == 0; }
static bool IsWalkToRun(const std::string& c) { return c.find("Transition_Walk_to_Run") != std::string::npos; }

struct Tail
{
    bool allLoops = true, allWalk = true, allRun = true;
    int jumps = 0, transitionUpdates = 0, headPicks = 0;
    float minRate = 1e9f, maxRate = -1e9f;
};
// the last aTailS seconds of v
static Tail TailOf(const std::vector<Step1>& v, float aTailS, float dt)
{
    Tail t;
    for (size_t i = v.size() - size_t(aTailS / dt); i < v.size(); ++i)
    {
        t.allLoops = t.allLoops && IsLoop(v[i].clip);
        t.allWalk = t.allWalk && IsWalkLoop(v[i].clip);
        t.allRun = t.allRun && IsRunLoop(v[i].clip);
        t.jumps += v[i].jumped ? 1 : 0;
        t.transitionUpdates += IsWalkToRun(v[i].clip) ? 1 : 0;
        t.minRate = std::min(t.minRate, v[i].rate);
        t.maxRate = std::max(t.maxRate, v[i].rate);
    }
    return t;
}
// new picks of a Walk_to_Run clip at its head (clip time < 0.1 s) above the walk pace: the pose reset of the game feed
static int HeadPicks(const std::vector<Step1>& v)
{
    int n = 0;
    for (const Step1& s : v)
        n += (s.jumped && IsWalkToRun(s.clip) && s.clipTime < 0.1f && s.speedCmS > 215.f) ? 1 : 0; // a run start from rest (< 210 cm/s) keeps the motion pick
    return n;
}
static int Jumps(const std::vector<Step1>& v, size_t from)
{
    int n = 0;
    for (size_t i = from; i < v.size(); ++i)
        n += v[i].jumped ? 1 : 0;
    return n;
}
static bool SameSeq(const std::vector<Step1>& a, const std::vector<Step1>& b, size_t from = 0)
{
    if (a.size() != b.size()) return false;
    for (size_t i = from; i < a.size(); ++i)
        if (a[i].clip != b[i].clip || a[i].rate != b[i].rate || a[i].jumped != b[i].jumped) return false;
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: utpp_mm_steadyrun_test <motion .utppmm>\n"); return 2; }
    Rig rig;
    if (!rig.Load(argv[1])) return 2;
    const MatcherConfig on;                 // production defaults
    MatcherConfig off = on;                 // motion behaviour
    off.steadyRunLoops = false;
    CHECK(on.steadyRunLoops, "steadyRunLoops is on by default (the game builds MatcherConfig{})");
    // one rule off each: it must fail the check that pins it
    MatcherConfig noWindow = on, noHead = on, noWalk = on, noFloor = on, noHold = on;
    noWindow.runTransitionWindowS = 0.f;
    noHead.runTransitionNoHeadRow = false;
    noWalk.runWalkLoopsMaxCmS = 0.f;
    noFloor.runMinPlayRate = 0.f;
    noHold.runWalkLoopsHoldCmS = 1.0e6f;

    // the game speed steps of the analog ramp (feed 2026-10-31): stick 0.64 -> 1.50 m/s (187 cm/s, Walk), 0.73 -> 1.95 (247), 0.82 -> 2.50 (320), 1.0 -> 3.85 (500)
    const Leg walk{1.50f, 0.64f, 3.0f}, s247{1.95f, 0.73f, 4.0f}, s320{2.50f, 0.82f, 4.0f}, full{3.85f, 1.0f, 3.0f};

    for (const int hz : {30, 60, 144})
    {
        rig.dt = 1.f / float(hz);

        // ---- A: walk -> stick 0.73 (247 cm/s, Run gait): the Walk_Loop clip plays on at rate > 1, no transition clip, no jump after the step
        rig.Reset(on);
        auto a = rig.Run({walk, s247});
        const size_t a0 = a.size() - size_t(4.0f / rig.dt);
        Tail ta = TailOf(a, 3.0f, rig.dt);
        CHECK(a.back().gait == 1 && std::fabs(a.back().speedCmS - 247.f) < 6.f, "Hz=%d A queries ~247 cm/s in the Run gait (%d, %.0f)", hz, a.back().gait, a.back().speedCmS);
        CHECK(ta.allWalk && ta.transitionUpdates == 0 && ta.jumps == 0, "Hz=%d A 247 cm/s after a walk: only Walk_Loop_*, no jump in the last 3 s (jumps %d, transitions %d)", hz, ta.jumps, ta.transitionUpdates);
        CHECK(ta.minRate > 1.0f && ta.maxRate <= 1.15f + 1e-4f, "Hz=%d A rate %.3f..%.3f within (1.0, 1.15]", hz, ta.minRate, ta.maxRate);
        CHECK(Jumps(a, a0) <= 2, "Hz=%d A at most 2 jumps from the step on (%d)", hz, Jumps(a, a0));

        // ---- B: the same from rest (no walk before): no Walk_to_Run head pick, steady loops in the last 3 s
        rig.Reset(on);
        auto b = rig.Run({s247});
        Tail tb = TailOf(b, 3.0f, rig.dt);
        CHECK(tb.allLoops && tb.transitionUpdates == 0 && tb.jumps == 0, "Hz=%d B 247 cm/s from rest: loops only, no jump in the last 3 s (jumps %d, transitions %d)", hz, tb.jumps, tb.transitionUpdates);
        CHECK(HeadPicks(b) == 0, "Hz=%d B no Walk_to_Run pick at clip time 0 (%d)", hz, HeadPicks(b));

        // ---- C: 2.50 m/s (320 cm/s) after 247: Run_Loop_* at the rate floor, no pose reset
        rig.Reset(on);
        auto c = rig.Run({walk, s247, s320});
        Tail tc = TailOf(c, 2.5f, rig.dt);
        CHECK(tc.allRun && tc.transitionUpdates == 0 && tc.jumps == 0, "Hz=%d C 320 cm/s: Run_Loop_* only, no jump in the last 2.5 s (jumps %d, transitions %d)", hz, tc.jumps, tc.transitionUpdates);
        CHECK(tc.minRate >= 0.5f - 1e-4f && tc.maxRate < 0.7f && std::fabs(c.back().speedCmS - 320.f) < 6.f,
              "Hz=%d C rate %.3f..%.3f in [0.5, 0.7) at %.0f cm/s", hz, tc.minRate, tc.maxRate, c.back().speedCmS);
        CHECK(HeadPicks(c) == 0, "Hz=%d C no Walk_to_Run pick at clip time 0 (%d)", hz, HeadPicks(c));

        // ---- D: the old failure, flag off: the pose resets of the game feed (a Walk_to_Run pick at clip time 0 every ~0.17 s) at 247 cm/s
        rig.Reset(off);
        auto d = rig.Run({walk, s247, s320});
        CHECK(HeadPicks(d) >= 4, "Hz=%d D flag off: Walk_to_Run head picks present (%d), the negative control of A / B / C", hz, HeadPicks(d));
        Tail td = TailOf(d, 2.5f, rig.dt);
        CHECK(td.transitionUpdates > 0 && !td.allRun, "Hz=%d D flag off: the 320 cm/s tail plays a Walk_to_Run clip (%d updates)", hz, td.transitionUpdates);

        // ---- each rule alone does only its own half
        rig.Reset(noWalk);
        auto cn = rig.Run({walk, s247});
        Tail tn = TailOf(cn, 3.0f, rig.dt);
        CHECK(!tn.allWalk, "Hz=%d no Walk_Loops in the Run gait: 247 cm/s is not played by Walk_Loop_* (walk %d)", hz, int(tn.allWalk));
        rig.Reset(noFloor);
        auto cf = rig.Run({walk, s247, s320});
        Tail tf = TailOf(cf, 2.5f, rig.dt);
        CHECK(tf.minRate >= 0.85f - 1e-3f, "Hz=%d no run rate floor: the 320 cm/s loops play at the ordinary 0.85 (%.3f)", hz, tf.minRate);
        rig.Reset(noWindow);
        auto cw = rig.Run({walk, s247, s320});
        Tail tw = TailOf(cw, 2.5f, rig.dt);
        CHECK(tw.transitionUpdates > 0, "Hz=%d no transition window: Walk_to_Run is a new candidate at 320 cm/s again (%d updates)", hz, tw.transitionUpdates);
        rig.Reset(noHold);
        auto ck = rig.Run({walk, s247, s320});
        Tail tk = TailOf(ck, 2.5f, rig.dt);
        CHECK(tk.allWalk, "Hz=%d no hold limit: the walk loop is held at 320 cm/s (%d)", hz, int(tk.allWalk));

        // ---- identity outside the Run gait rules: walk (Walk gait), crouch walk, full run tail (500 cm/s: the loops at their ordinary rate)
        rig.Reset(on);
        const auto x1 = rig.Run({walk});
        rig.Reset(off);
        const auto y1 = rig.Run({walk});
        CHECK(SameSeq(x1, y1), "Hz=%d walk 187 cm/s: flag on == flag off", hz);
        rig.Reset(on);
        const auto x2 = rig.Run({Leg{1.2f, 0.6f, 3.0f}}, false, adapter::kPsmCrouch);
        rig.Reset(off);
        const auto y2 = rig.Run({Leg{1.2f, 0.6f, 3.0f}}, false, adapter::kPsmCrouch);
        CHECK(SameSeq(x2, y2), "Hz=%d crouch walk: flag on == flag off", hz);
        rig.Reset(on);
        const auto x3 = rig.Run({walk, s247, s320, full});
        rig.Reset(off);
        const auto y3 = rig.Run({walk, s247, s320, full});
        Tail tx = TailOf(x3, 1.5f, rig.dt);
        CHECK(tx.allRun && tx.maxRate < 1.0f && tx.minRate > 0.85f, "Hz=%d full run 500 cm/s tail: Run_Loop_* at %.3f..%.3f (above the floor, unchanged bound)", hz, tx.minRate, tx.maxRate);
        Tail ty = TailOf(y3, 1.5f, rig.dt);
        // the flag-off tail may still be finishing a Walk_to_Run transition (what G10 removes); where both play the run loop the rate is the same
        CHECK(!ty.allRun || (std::fabs(tx.minRate - ty.minRate) < 1e-3f && std::fabs(tx.maxRate - ty.maxRate) < 1e-3f),
              "Hz=%d full run 500 cm/s tail: run-loop rate flag on == flag off (%.3f..%.3f vs %.3f..%.3f)", hz, tx.minRate, tx.maxRate, ty.minRate, ty.maxRate);

        // ---- a real walk -> full run acceleration still takes the transition (what it is for) when it follows a walk within the window
        rig.Reset(on);
        auto acc = rig.Run({walk, Leg{3.85f, 1.0f, 2.0f}});
        int picks = 0;
        for (const Step1& s : acc) picks += (s.jumped && IsWalkToRun(s.clip)) ? 1 : 0;
        CHECK(picks > 0, "Hz=%d walk -> full run acceleration still picks Walk_to_Run (window after the walk)", hz);
        std::printf("Hz=%d walk -> full run (flag on): Walk_to_Run picks %d\n", hz, picks);

        // ---- a run -> walk deceleration (stick back to 0.64: Walk gait at once) is not touched by G10: flag on == flag off, and it still takes the
        // Run_to_Walk transition (G1 window)
        rig.Reset(on);
        const auto dec = rig.Run({full, Leg{1.50f, 0.64f, 3.0f}});
        rig.Reset(off);
        const auto decOff = rig.Run({full, Leg{1.50f, 0.64f, 3.0f}});
        CHECK(SameSeq(dec, decOff), "Hz=%d run -> walk: flag on == flag off", hz);
        int rw = 0;
        for (const Step1& st : dec) rw += (st.jumped && st.clip.find("Transition_Run_to_Walk") != std::string::npos) ? 1 : 0;
        CHECK(rw > 0, "Hz=%d run -> walk still picks Run_to_Walk (%d)", hz, rw);
    }

    // ---- E: walk -> a faster steady speed at once (inside the window the Walk_to_Run clips are legal): the head row (clip time 0) is picked only
    // without runTransitionNoHeadRow; the pick depends on the update rate and the target speed, so the evidence is the sum over rates and speeds
    int headOn = 0, headOff = 0, runs = 0;
    for (const int hz : {30, 60, 144})
        for (const float v : {2.2f, 2.35f, 2.5f, 2.65f, 2.8f, 3.0f})
        {
            rig.dt = 1.f / float(hz);
            const Leg tgt{v, 0.82f, 4.0f};
            rig.Reset(on);
            headOn += HeadPicks(rig.Run({walk, tgt}));
            rig.Reset(noHead);
            headOff += HeadPicks(rig.Run({walk, tgt}));
            ++runs;
        }
    std::printf("E walk -> faster steady speed, %d runs: Walk_to_Run head picks %d (rule on), %d (rule off)\n", runs, headOn, headOff);
    CHECK(headOn == 0, "E walk -> faster: no Walk_to_Run pick at clip time 0 (%d)", headOn);
    CHECK(headOff > 0, "E no head-row rule: Walk_to_Run is picked at clip time 0 inside the window (%d)", headOff);

    // ---- F: steady-speed sweep over the whole band, 1.9-3.3 m/s in 0.1 steps (the stick from the measured stick / speed curve, kGameStickSpeedMotion),
    // from rest and from a walk, at three update rates: no Walk_to_Run pick at clip time 0, loops only in the last 2 s, at most one jump there
    int sweepBad = 0, sweepRuns = 0;
    for (const int hz : {30, 60, 144})
        for (int i = 0; i <= 14; ++i)
        {
            rig.dt = 1.f / float(hz);
            const float v = 1.9f + 0.1f * float(i);
            const float stick = v <= 1.94f ? 0.73f : (v <= 2.48f ? 0.73f + (v - 1.94f) / 0.54f * 0.09f : 0.82f + (v - 2.48f) / 1.36f * 0.18f);
            for (const bool fromWalk : {false, true})
            {
                rig.Reset(on);
                const Leg tgt{v, stick, 8.0f};
                const auto q = fromWalk ? rig.Run({walk, tgt}) : rig.Run({tgt});
                const Tail tq = TailOf(q, 2.0f, rig.dt);
                const bool ok = HeadPicks(q) == 0 && tq.allLoops && tq.jumps <= 1 && tq.minRate >= 0.5f - 1e-4f && tq.maxRate <= 1.15f + 1e-4f;
                ++sweepRuns;
                if (!ok)
                {
                    ++sweepBad;
                    std::printf("FAIL sweep Hz=%d v=%.1f stick=%.3f %s: head %d loops %d jumps %d trans %d rate %.2f..%.2f last %s\n", hz, v, stick,
                                fromWalk ? "from walk" : "from rest", HeadPicks(q), int(tq.allLoops), tq.jumps, tq.transitionUpdates, tq.minRate, tq.maxRate, q.back().clip.c_str());
                }
            }
        }
    CHECK(sweepBad == 0, "F steady sweep 1.9-3.3 m/s: %d of %d runs not clean", sweepBad, sweepRuns);

    // ---- G: a deceleration that keeps the stick >= 0.7 (Run gait all the way down to 247 cm/s: the walk-loop injection on the way down)
    for (const int hz : {30, 60, 144})
    {
        rig.dt = 1.f / float(hz);
        rig.Reset(on);
        const auto dn = rig.Run({full, s247});
        const Tail tg = TailOf(dn, 2.5f, rig.dt);
        int rw = 0;
        for (const Step1& st : dn) rw += st.clip.find("Transition_Run_to_Walk") != std::string::npos ? 1 : 0;
        CHECK(tg.allLoops && tg.jumps == 0 && tg.transitionUpdates == 0, "Hz=%d G full run -> stick 0.73: loops only, no jump in the last 2.5 s (jumps %d, trans %d)", hz, tg.jumps, tg.transitionUpdates);
        CHECK(rw == 0, "Hz=%d G no Run_to_Walk clip in the Run gait (%d updates)", hz, rw);
    }

    std::printf("SteadyRunLoop: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

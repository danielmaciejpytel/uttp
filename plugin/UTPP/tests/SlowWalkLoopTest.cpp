// G1: the established slow walk (analog half stick, walk toggle at the walk point) plays the Walk_Loop_* clips of the real v2 database, not the
// non-loop Run_to_Walk transition clips, and its loop tempo may go below the ordinary 0.85 bound (MatcherConfig::slowWalkLoops).
// Usage: utpp_mm_slowwalk_test <utpp_motion_woman*.utppmm> [--dump]   (chooser.json next to it; --dump prints the cheapest clips of the
// walk-loop database per channel group at 1.7 s of the analog case, the evidence of MatcherConfig::slowWalkLoops)
// Path: synthetic feed -> adapter::MapToChooser (production MapConfig defaults) -> BuildQueryTrajectory -> ChooserMatcher::Update.
// Mutants of the fix are the config switches themselves (a rule off must fail the check that pins it, see ExpectFail below).
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
    bool jumped = false;
    int gait = 0;
    float speedCmS = 0.f;
    std::string dbName;   // database of the playing pose
    int movementState = 0; // ChooserState::movementState (0 Idle, 1 Moving)
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
    std::string path;
    bool Load(const char* p)
    {
        path = p;
        std::string err, chooserPath(p);
        chooserPath.replace(chooserPath.find_last_of('.'), std::string::npos, ".chooser.json");
        if (!db.LoadFromFile(p, kRigIdentityWoman, err) || !table.LoadFile(chooserPath.c_str(), err))
        { std::printf("LOAD %s\n", err.c_str()); return false; }
        qtimes = adapter::BuildQueryTimes(db);
        return true;
    }
    // new matcher + map state with the production stance / filter settings (MotionDriver.cpp)
    // analogFloorMs: the minimum analog walk setting (CET mm.minWalkSpeed; MotionDriver maps it with adapter::SetAnalogFloor), 0 = off
    bool Reset(const MatcherConfig& cfg, float togglePoint = 0.f, float analogFloorMs = 0.f)
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
        adapter::SetAnalogFloor(mc, analogFloorMs);
        return matcher.Init(&db, &table, cfg, err);
    }
    // one update: capsule speed along +Y (m/s), stick |MoveY|, walk toggle, PSM detail (Stand / Crouch)
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
        o.jumped = r.jumped;
        o.gait = cs.gait;
        o.speedCmS = cs.speed2dCmS;
        o.movementState = cs.movementState;
        if (r.database >= 0 && uint32_t(r.database) < db.NumDatabases())
            o.dbName = std::string(db.DbName(uint32_t(r.database)));
        return o;
    }
    // 0.5 s of rest, a 0.4 s ramp to the steady speed, then aSeconds of steady walk
    std::vector<Step1> Steady(float speedMs, float stick, bool walk, float aSeconds, int psm = adapter::kPsmStand)
    {
        std::vector<Step1> out;
        const int n = int(std::lround((0.5f + aSeconds) / dt)), rest = int(std::lround(0.5f / dt));
        for (int i = 0; i < n; ++i)
        {
            const float t = (i - rest) * dt;
            const bool moving = i >= rest;
            out.push_back(Step(moving ? speedMs * std::fmin(1.f, t / 0.4f) : 0.f, moving ? stick : 0.f, walk, psm));
        }
        return out;
    }
};

static bool IsLoop(const std::string& c) { return c.rfind("M_Neutral_Walk_Loop_", 0) == 0; }
static bool IsRunToWalk(const std::string& c) { return c.find("Transition_Run_to_Walk") != std::string::npos; }

// the last aTailS seconds of the steady part: every update plays a Walk_Loop_* clip, none re-picks a transition
static bool TailLoops(const std::vector<Step1>& v, float aTailS, float dt, int& transitions, float& minRate, float& maxRate)
{
    transitions = 0;
    minRate = 1e9f;
    maxRate = -1e9f;
    bool all = true;
    const size_t from = v.size() - size_t(aTailS / dt);
    for (size_t i = from; i < v.size(); ++i)
    {
        all = all && IsLoop(v[i].clip);
        transitions += IsRunToWalk(v[i].clip) ? 1 : 0;
        minRate = std::min(minRate, v[i].rate);
        maxRate = std::max(maxRate, v[i].rate);
    }
    return all;
}

static bool SameSeq(const std::vector<Step1>& a, const std::vector<Step1>& b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i].clip != b[i].clip || a[i].rate != b[i].rate || a[i].jumped != b[i].jumped) return false;
    return true;
}

// cheapest clips of the walk-loop database for the query of the current state (clip minima, trajectory part split from the pose part)
static void Dump(Rig& rig)
{
    const MotionDatabase& db = rig.db;
    uint32_t aDb = 0;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        if (db.DbName(d) == "PSD_Dense_Stand_Walk_Loops") aDb = d;
    rig.matcher.BuildQuery(aDb, true, true);
    const DatabaseRecord& D = db.Db(aDb);
    const SchemaRecord& S = db.Schema(D.schema);
    struct Best { std::string clip; float cost = 1e9f, traj = 0, pose = 0, add = 0; };
    std::vector<Best> v;
    for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
    {
        const EntryRecord& en = db.Entry(e);
        Best b;
        b.clip = std::string(db.ClipName(en.clip));
        for (uint32_t row = en.firstRow - D.firstRow; row < en.firstRow - D.firstRow + en.numRows; ++row)
        {
            const RowRecord& rr = db.Row(D.firstRow + row);
            if (rr.flags & kRowBlock) continue;
            const float c = db.RowCost(aDb, row, rig.matcher.NormalizedQuery()) + rr.costAddend;
            if (c >= b.cost) continue;
            b.cost = c;
            b.add = rr.costAddend;
            b.traj = b.pose = 0;
            for (uint32_t ci = S.firstChannel; ci < S.firstChannel + S.numChannels; ++ci)
            {
                const ChannelRecord& ch = db.Channel(ci);
                for (uint32_t k = 0; k < ch.cardinality; ++k)
                {
                    const float t = rig.matcher.NormalizedQuery()[ch.dataOffset + k] - db.Features(aDb, row)[ch.dataOffset + k];
                    ((ch.flags & kChanTrajectory) ? b.traj : b.pose) += t * t;
                }
            }
        }
        v.push_back(b);
    }
    std::sort(v.begin(), v.end(), [](const Best& a, const Best& b) { return a.cost < b.cost; });
    for (size_t i = 0; i < v.size() && i < 6; ++i)
        std::printf("    %-44s cost=%.3f (trajectory %.4f pose %.4f addend %.3f)\n", v[i].clip.c_str(), v[i].cost, v[i].traj, v[i].pose, v[i].add);
}

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: utpp_mm_slowwalk_test <motion .utppmm> [--dump]\n"); return 2; }
    const bool dump = argc > 2 && std::strcmp(argv[2], "--dump") == 0;
    Rig rig;
    if (!rig.Load(argv[1])) return 2;
    const MatcherConfig on;                 // production defaults
    MatcherConfig off = on;                 // motion behaviour
    off.slowWalkLoops = false;
    MatcherConfig windowOnly = on, floorOnly = on; // one rule off each: they must fail the check that pins the other rule
    windowOnly.walkMinPlayRate = 0.f;
    floorOnly.walkTransitionWindowS = 0.f;
    CHECK(on.slowWalkLoops, "slowWalkLoops is on by default (the game builds MatcherConfig{})");

    for (const int hz : {30, 60, 144})
    {
        rig.dt = 1.f / float(hz);
        int tr = 0;
        float lo = 0, hi = 0;

        // ---- analog half stick: 0.82 m/s at |mv| 0.455 (game logs 2026-10-30: speed 0.32 -> 0.84 at stick 0.17 -> 0.45)
        rig.Reset(on);
        auto a = rig.Steady(0.82f, 0.455f, false, 8.0f);
        CHECK(TailLoops(a, 3.0f, rig.dt, tr, lo, hi) && tr == 0, "Hz=%d analog 0.82 m/s: only Walk_Loop_* in the last 3 s (transitions %d)", hz, tr);
        CHECK(a.back().gait == 0 && std::fabs(a.back().speedCmS - 102.f) < 5.f, "Hz=%d analog 0.82 m/s queries ~102 cm/s (%.0f) in the Walk gait", hz, a.back().speedCmS);
        CHECK(lo >= 0.5f - 1e-4f && hi < 0.85f - 1e-3f, "Hz=%d analog rate in [0.5, 0.85) (%.3f..%.3f)", hz, lo, hi);

        // ---- walk toggle at the walk point (CET mm.walkSpeed 1.6 m/s -> 200 cm/s)
        rig.Reset(on, 1.6f);
        auto g = rig.Steady(1.6f, 1.0f, true, 8.0f);
        CHECK(TailLoops(g, 3.0f, rig.dt, tr, lo, hi) && tr == 0, "Hz=%d G walk 1.6 m/s: only Walk_Loop_* in the last 3 s (transitions %d)", hz, tr);
        CHECK(std::fabs(g.back().speedCmS - 200.f) < 5.f && lo > 0.8f && hi < 1.0f, "Hz=%d G walk queries 200 cm/s (%.0f), rate %.3f..%.3f", hz, g.back().speedCmS, lo, hi);

        // ---- motion walk range: 1.2 m/s at |mv| 0.6 -> 150 cm/s, rate below the ordinary 0.85
        rig.Reset(on);
        auto m = rig.Steady(1.2f, 0.6f, false, 8.0f);
        CHECK(TailLoops(m, 3.0f, rig.dt, tr, lo, hi) && tr == 0 && hi < 0.85f - 1e-3f && lo >= 0.5f - 1e-4f,
              "Hz=%d analog 1.2 m/s: Walk_Loop_*, rate %.3f..%.3f", hz, lo, hi);

        // ---- a fixed rate (minPlayRate == maxPlayRate, the debug / test configuration) is not lowered by the walk floor
        MatcherConfig fixedCfg = on;
        fixedCfg.minPlayRate = fixedCfg.maxPlayRate = 1.0f;
        rig.Reset(fixedCfg);
        auto fx = rig.Steady(0.82f, 0.455f, false, 4.0f);
        TailLoops(fx, 2.0f, rig.dt, tr, lo, hi);
        CHECK(lo == 1.0f && hi == 1.0f, "Hz=%d fixed rate 1.0 stays 1.0 under the walk floor (%.3f..%.3f)", hz, lo, hi);

        // ---- flag off = today's behaviour: the transition clip at the 0.85 bound (negative control of the three checks above)
        rig.Reset(off);
        auto o = rig.Steady(0.82f, 0.455f, false, 8.0f);
        bool anyTr = false;
        for (size_t i = o.size() - size_t(3.0f / rig.dt); i < o.size(); ++i) anyTr = anyTr || IsRunToWalk(o[i].clip);
        TailLoops(o, 3.0f, rig.dt, tr, lo, hi);
        CHECK(anyTr && std::fabs(lo - 0.85f) < 1e-3f && std::fabs(hi - 0.85f) < 1e-3f,
              "Hz=%d flag off: the Run_to_Walk transition plays at 0.85 as before (%d updates, rate %.3f..%.3f)", hz, tr, lo, hi);
        // each rule alone does only its own half
        rig.Reset(windowOnly);
        auto w = rig.Steady(0.82f, 0.455f, false, 8.0f);
        CHECK(TailLoops(w, 3.0f, rig.dt, tr, lo, hi) && std::fabs(lo - 0.85f) < 1e-3f && std::fabs(hi - 0.85f) < 1e-3f,
              "Hz=%d transition window alone: loops at the ordinary 0.85 (%.3f..%.3f)", hz, lo, hi);
        rig.Reset(floorOnly);
        auto fl = rig.Steady(0.82f, 0.455f, false, 8.0f);
        TailLoops(fl, 3.0f, rig.dt, tr, lo, hi);
        CHECK(tr > 0 && std::fabs(lo - 0.85f) < 1e-3f, "Hz=%d rate floor alone: the transition (non-loop) keeps the 0.85 bound (%d updates)", hz, tr);

        // ---- a real run -> walk deceleration still takes the transition clip (what it is for)
        for (const MatcherConfig* cfg : std::vector<const MatcherConfig*>{&on, &off})
        {
            rig.Reset(*cfg);
            rig.Steady(3.85f, 1.0f, false, 3.0f); // run (stick 1 -> Run gait, 500 cm/s)
            int transitionPicks = 0;
            for (int i = 0; i < int(2.0f / rig.dt); ++i)
            {
                const float t = float(i) * rig.dt;
                const Step1 s = rig.Step(std::fmax(0.82f, 3.85f - (3.85f - 0.82f) * t / 0.6f), 0.455f, false);
                transitionPicks += s.jumped && IsRunToWalk(s.clip);
            }
            std::printf("Hz=%d run -> walk (%s): Run_to_Walk picks %d\n", hz, cfg == &on ? "flag on" : "flag off", transitionPicks);
            if (cfg == &on) {
                CHECK(transitionPicks > 0, "Hz=%d run -> walk deceleration still picks Run_to_Walk with the flag on", hz);
            }
        }

        // ---- identity outside the stand walk loops: run, sprint-less crouch walk are bit-identical with the flag on and off
        for (const char* what : {"run", "crouch walk"})
        {
            const bool crouch = what[0] == 'c';
            rig.Reset(on);
            const auto x = crouch ? rig.Steady(1.2f, 0.6f, false, 3.0f, adapter::kPsmCrouch) : rig.Steady(3.85f, 1.0f, false, 3.0f);
            rig.Reset(off);
            const auto y = crouch ? rig.Steady(1.2f, 0.6f, false, 3.0f, adapter::kPsmCrouch) : rig.Steady(3.85f, 1.0f, false, 3.0f);
            CHECK(SameSeq(x, y), "Hz=%d %s: flag on == flag off", hz, what);
        }

        // ---- minimum analog walk speed (mm.minWalkSpeed 0.82): a small tilt |mv| 0.09 while the script lifts the capsule from 0 to
        // 0.82 m/s leaves the Idles database within 0.25 s (feed 2026-10-31 17:18: Idles for ~3.2 s while the capsule moved at 0.24 m/s)
        for (const int psm : {adapter::kPsmStand, adapter::kPsmCrouch})
        {
            rig.Reset(on, 0.f, 0.82f);
            const float liftedMs = psm == adapter::kPsmCrouch ? 0.82f * 2.7f / 3.84f : 0.82f;
            auto s = rig.Steady(liftedMs, 0.09f, false, 3.0f, psm);
            const size_t rest = size_t(std::lround(0.5f / rig.dt));
            int leave = -1;
            for (size_t i = rest; i < s.size() && leave < 0; ++i)
                if (s[i].movementState == 1 && !s[i].dbName.empty() && s[i].dbName.find("Idles") == std::string::npos)
                    leave = int(i - rest);
            const float leaveS = leave < 0 ? -1.f : float(leave) * rig.dt;
            std::printf("Hz=%d %s |mv| 0.09 small tilt: leaves Idles after %.3f s (%s), after 0.25 s %s, end %s\n", hz,
                        psm == adapter::kPsmCrouch ? "crouch" : "stand", leaveS, leave >= 0 ? s[rest + size_t(leave)].dbName.c_str() : "-",
                        s[rest + size_t(std::lround(0.25f / rig.dt))].dbName.c_str(), s.back().dbName.c_str());
            CHECK(leave >= 0 && leaveS <= 0.25f, "Hz=%d %s |mv| 0.09 lifted to %.2f m/s: Moving and not Idles after %.3f s (<= 0.25)", hz,
                  psm == adapter::kPsmCrouch ? "crouch" : "stand", liftedMs, leaveS);
            // the start is the one of a half-stick start (the input acceleration of analogFloorStick): the same databases as |mv| 0.455 at the
            // same capsule speed (stand: Walk_Starts; with the raw |mv| 0.09 acceleration IsStarting stayed false and the first frame picked
            // Walk_Pivots for ~0.3 s)
            rig.Reset(on, 0.f, 0.82f);
            const auto h = rig.Steady(liftedMs, 0.455f, false, 3.0f, psm);
            bool sameDbs = true;
            for (size_t i = rest; i < s.size(); ++i)
                sameDbs = sameDbs && s[i].dbName == h[i].dbName;
            std::printf("Hz=%d %s |mv| 0.455 reference: first pick %s, end %s\n", hz, psm == adapter::kPsmCrouch ? "crouch" : "stand",
                        h[rest].dbName.c_str(), h.back().dbName.c_str());
            CHECK(sameDbs, "Hz=%d %s |mv| 0.09 lifted: the same databases as the |mv| 0.455 start at the same capsule speed", hz,
                  psm == adapter::kPsmCrouch ? "crouch" : "stand");
            if (psm == adapter::kPsmStand)
                CHECK(leave >= 0 && s[rest + size_t(leave)].dbName.find("Starts") != std::string::npos,
                      "Hz=%d stand |mv| 0.09: the first pick is Walk_Starts (%s)", hz, leave >= 0 ? s[rest + size_t(leave)].dbName.c_str() : "-");
            bool idleTail = false;
            for (size_t i = s.size() - size_t(2.0f / rig.dt); i < s.size(); ++i)
                idleTail = idleTail || s[i].movementState != 1 || s[i].dbName.find("Idles") != std::string::npos;
            CHECK(!idleTail, "Hz=%d %s |mv| 0.09: no Idles / Idle state in the last 2 s of the steady small tilt", hz,
                  psm == adapter::kPsmCrouch ? "crouch" : "stand");
            // release: the stick goes to exactly 0 (the game reports 0 at rest) and the capsule brakes to 0 in 0.3 s -> stop
            if (psm == adapter::kPsmStand)
            {
                bool idleAtRelease = false, stopped = false;
                for (int i = 0; i < int(1.5f / rig.dt); ++i)
                {
                    const float t = float(i) * rig.dt;
                    const Step1 r = rig.Step(std::fmax(0.f, liftedMs * (1.f - t / 0.3f)), 0.f, false, psm);
                    if (i == 0)
                        idleAtRelease = r.movementState == 0;
                    stopped = stopped || r.dbName.find("Stops") != std::string::npos || r.dbName.find("Idles") != std::string::npos;
                }
                CHECK(idleAtRelease && stopped, "Hz=%d release after the small tilt: Idle state at once (%d), a Stops / Idles pick within 1.5 s (%d)",
                      hz, idleAtRelease ? 1 : 0, stopped ? 1 : 0);
            }
        }

        // ---- identity: at and above the floor stick (|mv| 0.455 .. 1.0) the floor (analogFloorStick 0.455) changes nothing, bit for bit
        for (const float stick : {0.455f, 0.55f, 0.64f, 0.73f, 0.82f, 1.0f})
        {
            const float speed = adapter::GameSteadySpeed(adapter::MapConfig{}, 0, false, stick);
            rig.Reset(on);
            const auto x = rig.Steady(speed, stick, false, 3.0f);
            rig.Reset(on, 0.f, 0.82f);
            const auto y = rig.Steady(speed, stick, false, 3.0f);
            bool same = SameSeq(x, y);
            for (size_t i = 0; same && i < x.size(); ++i)
                same = x[i].speedCmS == y[i].speedCmS && x[i].gait == y[i].gait && x[i].dbName == y[i].dbName &&
                       x[i].movementState == y[i].movementState;
            CHECK(same, "Hz=%d |mv| %.3f (%.2f m/s): floor on == floor off", hz, stick, speed);
            if (stick == 0.455f) // reference for the small tilt above: the first pick of a half-stick start from rest
                std::printf("Hz=%d |mv| 0.455 start: first pick %s, after 0.25 s %s\n", hz, y[size_t(std::lround(0.5f / rig.dt))].dbName.c_str(),
                            y[size_t(std::lround(0.75f / rig.dt))].dbName.c_str());
        }
    }

    if (dump)
    {
        rig.dt = 1.f / 60.f;
        rig.Reset(off);
        rig.Steady(0.82f, 0.455f, false, 1.2f);
        std::printf("walk-loop database, query of the analog 0.82 m/s case (flag off, 1.7 s):\n");
        Dump(rig);
    }
    std::printf("SlowWalkLoop: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

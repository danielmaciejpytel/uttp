// Selection churn guard regression (2026-11-11): clip changes per second of the REAL v1 MMDriver + MotionDriver (no RED4ext / game) on slices of the
// 31-column game feed dump of the 2026-11-11 10:29 harness run (Female V), the windows of Source\tools\gametest that failed
// motion_transitions_per_s <= 3.0 (thresholds.json, unchanged): slalom 3.875, flick_180_run / flick 4.0, flick_180_sprint / accel 3.334,
// sprint_dash / dash_left, dash_right, dash_back 4.0; sprint_dash / sprint_again (8 in the game, 9 = exactly 3.0 per second
// after the first guards) is pinned with margin.
// Usage: utpp_mm_loco_churn_test <data dir> <v1 database.utppmm> <slice dir (tests\data)>
// The slices keep the original feed time; each starts a few seconds before its window so the driver warms up. The full-feed replay
// (utpp_motion_driver_decisiondump --full-feed) reproduces the counts of the game report for these windows; the slices reproduce the full-feed
// counts for the windows of the first guards (checked when those budgets were set; the start upshift guard shifted dash_left to 4 here against 5
// in the full-feed replay, its budget stays 5), so a window count here is the number the harness would report within one change. The authoritative
// table of all 63 windows is the full-feed replay (Build\work\mm-churn-20261111\report.txt).
// Checks: A every window stays within its budget = the count with the guards (slalom 14, flick 7, flick_180_sprint / accel 4, dash_left 5,
//   dash_right 4, dash_back 4, sprint_again 8); the counts without the guards are 31 / 10 / 5 / 9 / 8 / 5 / 9 (a bound of 3.0 per second is 24 / 7 / 4 /
//   6 / 6 / 6 / 9 changes). flick_180_sprint / accel (the gait edges Walk -> Run -> Sprint of a full-stick start with a 180 deg reversal, the same
//   count as the game report, 5 changes in 1.5 s = 3.33 per second) and sprint_again (the same start chain) are held by the start upshift guard
//   (MatcherConfig::startUpshiftHoldS, startUpshiftRefaceWalkHoldS / RunHoldS): a gait upshift during a young Walk start / reface start clip is not
//   an interrupting edge (counts 4 and 8 with the brief reface hold; 3 and 7 with the 0.35 s one). The windows counted like the harness since the metric
//   fix of gt_log.py (a relabel of the same clip is no change). churn_reface_sprint / churn_reface_whip: slices of the 2026-11-11 16:04 Female dump.
//   C a Walk reface start replaced by another reface start (opposite turn side, Run variant) is replaced <= 0.13 s after its pick (no late swap into the
//   turn: footlock_corr_max_cm 16.1 cm with the 0.35 s hold on reface starts, Male camera_whip_180 / accel 2026-11-11 16:25; 6.7 cm before the hold).
//   D (churn guard 7, MatcherConfig::startRestartDwellS, churn_stalled_start.feed = the Female 2026-11-11 17:37 weapon_aim / walk_armed, aim_walk and
//   weapon_walk / walk_unarmed windows, the stick held against a blocked capsule): no start clip is restarted from its first frame by another start of
//   the same speed class more than once per window (game: Run_Start_F_Lfoot <-> Run_Start_F_Rfoot every 0.217 s at ct 0, 13 changes in 3 s = 4.3 per
//   second; the same with every guard off). churn_sprint_flick_man.feed (Male 17:19, needs the Male bundle) pins its 9 changes: NOT fixed.
//   E (gait return hold, adapter::MapToChooser, no data files): a slow rising stick 0.3 -> 0.6 after a short run keeps the Run gait for 0.3 s from the last
//   real Run frame, once, without flicker (RunGaitReturnHold; mutation 27 removes the hold: "keeps Run" fails too).
//   B an ordinary release at speed still picks its Stop clip on the release frame (no stick latency): none of the guards may delay a stick edge
//   (the start upshift guard only defers the swap of a young start clip to the faster gait's start; the first start of a press and every release / stop are not held).
// Mutations (UTPP_MUTATION_JUMPPICK, run-time, src/MutationSelect.hpp): 24 no loop dwell, 25 no young event clip bias, 26 no dodge hold,
// 27 no gait return hold, 28 no start upshift hold, 29 reface starts held 0.35 s like plain starts (check C), 30 no start restart / stalled dwell
// (check D). Each must fail its check
// (tests/motiondriverinteg.cmake).
#define UTPP_DECISIONDUMP_NO_MAIN
#include "MotionDriverDecisionDump.cpp"

namespace
{
struct Window
{
    const char* name;
    float t0, t1; // feed time (s), as written in the slice
    int budget;   // clip / database changes allowed in [t0, t1)
    bool reface = false; // check C applies: the Walk reface start swaps of this window
    bool restarts = false; // check D applies: no start clip restarted from its first frame over and over
};

// check C: a Walk reface start that is replaced by another reface start (the opposite turn side) is replaced by the gait edge, not later: 0.1 s after
// the pick at most (the swap before the start-upshift hold came 50-67 ms after the pick; the 0.35 s hold deferred it to 183 ms, where the foot lock
// corrected 16.1 cm in the Male 2026-11-11 16:25 camera_whip_180 / accel run, 6.7 cm without the hold). The brief reface hold gives ~100 ms.
constexpr float kMaxRefaceSwapS = 0.13f;

struct Slice
{
    const char* file;
    std::vector<Window> windows;
    bool man = false; // Male V bundle (body slot 1)
};

// the clip / database changes of the frames the v2 driver stepped (what the MOTION trace lines show), counted like gt_log.py _motion_metrics: the same
// clip under another database label whose clip time advanced by the elapsed time (within 0.05 s, MOTION_RELABEL_CT_TOL_S) is no change
int CountChanges(const FeedFile& aFeed, const std::vector<FrameOut>& aRun, float aT0, float aT1)
{
    int n = 0;
    bool have = false;
    int32_t clip = -1, dbi = -1;
    float ct = 0.0f, tPrev = 0.0f;
    for (size_t i = 0; i < aRun.size() && i < aFeed.fr.size(); ++i)
    {
        if (!aRun[i].stepped)
            continue;
        const float t = aFeed.fr[i].t;
        const bool differs = aRun[i].clip != clip || aRun[i].db != dbi;
        const bool relabel = differs && aRun[i].clip == clip && std::fabs((aRun[i].clipTime - ct) - (t - tPrev)) <= 0.05f;
        if (have && t >= aT0 && t < aT1 && differs && !relabel)
            ++n;
        clip = aRun[i].clip;
        dbi = aRun[i].db;
        ct = aRun[i].clipTime;
        tPrev = t;
        have = true;
    }
    return n;
}

// reface start swaps: every replacement of a Walk_Reface_Start clip by another reface start (the opposite turn side, the Run variant, ...) whose pick
// lies in [aT0 - 0.3, aT1); aDelay = seconds between the pick of the replaced clip and the replacement (relabels of the same clip do not count)
struct RefaceSwaps
{
    int count = 0;
    float worstDelayS = 0.0f;
};
RefaceSwaps FindRefaceSwaps(const mm::motion::MotionDatabase& aDb, const FeedFile& aFeed, const std::vector<FrameOut>& aRun, float aT0, float aT1)
{
    RefaceSwaps r;
    int32_t clip = -1;
    float pickT = 0.0f;
    bool have = false;
    for (size_t i = 0; i < aRun.size() && i < aFeed.fr.size(); ++i)
    {
        if (!aRun[i].stepped || aRun[i].clip < 0)
            continue;
        const float t = aFeed.fr[i].t;
        if (have && aRun[i].clip != clip && pickT >= aT0 - 0.3f && pickT < aT1)
        {
            const std::string_view from = aDb.ClipName(static_cast<uint32_t>(clip)), to = aDb.ClipName(static_cast<uint32_t>(aRun[i].clip));
            if (from.find("Walk_Reface_Start_") != std::string_view::npos && to.find("Reface_Start_") != std::string_view::npos)
            {
                ++r.count;
                r.worstDelayS = std::fmax(r.worstDelayS, t - pickT);
            }
        }
        if (!have || aRun[i].clip != clip)
            pickT = t;
        clip = aRun[i].clip;
        have = true;
    }
    return r;
}

// check D: a start clip (Walk / Run / Sprint start, no reface) replaced by another start clip of the same speed class that starts from its first frame (clip time <= 0.05 s): the
// restart of the Female 2026-11-11 17:37 run (Run_Start_F_Lfoot <-> Run_Start_F_Rfoot every 0.217 s at ct 0, the stick held against a blocked capsule)
int CountStartRestarts(const mm::motion::MotionDatabase& aDb, const FeedFile& aFeed, const std::vector<FrameOut>& aRun, float aT0, float aT1)
{
    int n = 0;
    int32_t clip = -1;
    for (size_t i = 0; i < aRun.size() && i < aFeed.fr.size(); ++i)
    {
        if (!aRun[i].stepped || aRun[i].clip < 0)
            continue;
        const float t = aFeed.fr[i].t;
        if (clip >= 0 && aRun[i].clip != clip && t >= aT0 && t < aT1 && aRun[i].clipTime <= 0.05f)
        {
            const std::string_view from = aDb.ClipName(static_cast<uint32_t>(clip)), to = aDb.ClipName(static_cast<uint32_t>(aRun[i].clip));
            const auto isStart = [](std::string_view a) { return a.find("_Start_") != std::string_view::npos && a.find("Reface") == std::string_view::npos; };
            if (isStart(from) && isStart(to) && utpp::mm::steer::SpeedClassOfName(from) == utpp::mm::steer::SpeedClassOfName(to))
                ++n;
        }
        clip = aRun[i].clip;
    }
    return n;
}
constexpr int kMaxStartRestarts = 1;

// check E (gait return hold, MapConfig::gaitReturnHoldS, no data files): a slow RISING stick (0.3 -> 0.6, below the 0.7 Walk / Run threshold) shortly after a
// short run, the capsule still running, keeps the Run gait for gaitReturnHoldS from the last real Run frame and then returns to Walk ONCE (2 gait
// changes in the ramp: Walk -> Run when the hold starts, Run -> Walk when it expires; 30 before the fix). Before the fix the
// forced gait was stored in st.gait (the next frame saw st.gait == 1 and fell back to Walk) and the forced frame reset the hold clock, so the gait flickered
// 1 / 0 on alternate frames and the hold never expired while the stick kept rising.
constexpr int kGaitHoldDipFrames = 3; // 0.05 s at the dip stick before the ramp
struct GaitHoldRun
{
    int forcedFrames = 0; // ramp frames with gait Run (the hold)
    int flips = 0;        // gait changes in the ramp (1 -> 0, 0 -> 1)
    float lastRunT = 0.0f; // seconds since the ramp start of the last Run frame
    int runsAfterWalk = 0; // Run frames after the first Walk frame of the ramp (the gait must not come back)
};
GaitHoldRun RunGaitReturnHold(float aHoldS)
{
    namespace ad = utpp::mm::adapter;
    const float dt = 1.0f / 60.0f;
    ad::MapConfig mc;
    mc.gaitReturnHoldS = aHoldS;
    ad::MapState ms;
    GaitHoldRun out;
    // 0.5 s full-stick run (3.84 m/s), then the stick dips to 0.3 for 0.1 s (the Walk gait at once, the capsule still runs), then a 0.5 s ramp 0.3 -> 0.6
    // (0.01 per frame) with the capsule at 2.5 m/s, then 0.5 s at 0.6
    const int nRun = 30, nDip = kGaitHoldDipFrames, nRamp = 30, nTail = 30;
    int prevGait = 1;
    bool sawWalk = false;
    for (int i = 0; i < nRun + nDip + nRamp + nTail; ++i)
    {
        float stick = 1.0f, speed = 3.84f;
        if (i >= nRun)
            stick = 0.3f, speed = 3.0f;
        if (i >= nRun + nDip)
            stick = std::fmin(0.6f, 0.3f + 0.01f * static_cast<float>(i - nRun - nDip + 1)), speed = 2.5f;
        const float vel[3] = {0.0f, speed, 0.0f}, mv[2] = {0.0f, stick};
        ad::MotionFeed f = ad::MakeMotionFeed(vel, mv, dt, 0u, 0, ad::AirFeed{}, 0.0f, 0.0f);
        ad::MapContext ctx;
        ctx.rootYawValid = true;
        const utpp::mm::motion::ChooserState cs = ad::MapToChooser(f, mc, ms, ctx);
        if (i < nRun + nDip)
        {
            prevGait = cs.gait;
            continue;
        }
        const float t = static_cast<float>(i - nRun - nDip) * dt;
        if (cs.gait != prevGait)
            ++out.flips;
        sawWalk = sawWalk || cs.gait == 0;
        if (cs.gait >= 1)
        {
            ++out.forcedFrames;
            out.lastRunT = t;
            out.runsAfterWalk += sawWalk ? 1 : 0;
        }
        prevGait = cs.gait;
    }
    return out;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::printf("usage: utpp_mm_loco_churn_test <data dir> <v1 database> <slice dir>\n");
        return 2;
    }
    const std::string data = argv[1], sliceDir = argv[3];
    // E: the gait return hold on a slow rising stick after a short run (no data files)
    {
        const GaitHoldRun e = RunGaitReturnHold(0.3f);
        char ed[200];
        std::snprintf(ed, sizeof(ed), "%d Run frames in the ramp, %d gait changes, last Run frame %.3f s after the ramp start, %d Run frames after the first Walk frame",
                      e.forcedFrames, e.flips, static_cast<double>(e.lastRunT), e.runsAfterWalk);
        Check(e.forcedFrames >= 3, "gait return hold keeps Run on the slow rising stick after a short run", ed);
        Check(e.flips <= 2 && e.runsAfterWalk == 0,"gait return hold does not flicker the gait on a slow rising stick", ed);
        Check(e.lastRunT <= 0.3f - static_cast<float>(kGaitHoldDipFrames) / 60.0f + 0.02f, "gait return hold expires 0.3 s after the last real Run frame", ed);
        const GaitHoldRun off = RunGaitReturnHold(0.0f);
        Check(off.forcedFrames == 0, "gait return hold off (knob 0) = the gait of the stick", std::to_string(off.forcedFrames) + " Run frames");
    }
    g_baseDb.assign(argv[2], argv[2] + std::strlen(argv[2]));
    g_fullFeed = true;
    const motiondriver::Paths pw = DataPaths(data, "woman");
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(pw.files.db, rig, &db))
    {
        std::printf("FAIL v2 rig of %s\n", pw.files.db.c_str());
        return 3;
    }
    motiondriver::SetPathsOverride(0, pw);
    const motiondriver::Paths pm = DataPaths(data, "man");
    Rig rigM;
    std::unique_ptr<mm::motion::MotionDatabase> dbM;
    if (!RigFromDb(pm.files.db, rigM, &dbM))
    {
        std::printf("FAIL v2 rig of %s\n", pm.files.db.c_str());
        return 3;
    }
    motiondriver::SetPathsOverride(1, pm);
    motionhost::SetMotionHooks(&kHooks);

    const std::vector<Slice> slices = {
        {"churn_slalom.feed", {{"slalom", 118.203f, 126.203f, 16}}},
        {"churn_flick_run.feed", {{"flick_180_run flick", 151.308f, 153.808f, 7}}},
        {"churn_flick_sprint.feed", {{"flick_180_sprint accel", 162.384f, 163.884f, 4, true}}},
        {"churn_reface_sprint.feed", {{"flick_180_sprint accel (second run)", 169.529f, 171.029f, 4, true}}},
        {"churn_reface_whip.feed", {{"camera_whip_180 accel", 185.111f, 187.111f, 3, true}}},
        // Female 2026-11-11 17:37: the stick held against a blocked capsule (weapon armed / unarmed walk): the start clips restarted every 0.2 s (13 changes in 3 s)
        {"churn_stalled_start.feed", {{"weapon_aim walk_armed", 307.026f, 310.026f, 6, false, true}, {"weapon_aim aim_walk", 310.026f, 313.026f, 3, false, true},
                                       {"weapon_walk walk_unarmed", 318.44f, 321.442f, 6, false, true}}},
        // Male 2026-11-11 17:19: sprint flick; NOT fixed: 9 changes in 2.5 s (3.6 per second, game 9), the budget pins the count
        {"churn_sprint_flick_man.feed", {{"flick_180_sprint flick (Male, not fixed)", 181.397f, 183.897f, 9}}, true},
        {"churn_dash.feed", {{"sprint_dash dash_left", 258.284f, 260.284f, 5}, {"sprint_dash dash_right", 260.284f, 262.284f, 4}, {"sprint_dash dash_back", 262.284f, 264.284f, 4}, {"sprint_dash sprint_again", 264.284f, 267.284f, 8}}},
    };
    for (const Slice& s : slices)
    {
        FeedFile f;
        if (!LoadFeedAny(sliceDir + "\\" + s.file, f))
        {
            std::printf("FAIL slice %s missing / unreadable\n", s.file);
            return 3;
        }
        g_dumpBody = s.man ? 1 : 0;
        const mm::motion::MotionDatabase& sdb = s.man ? *dbM : *db;
        const std::vector<FrameOut> run = RunDriver(s.man ? rigM : rig, f);
        size_t stepped = 0;
        for (const FrameOut& o : run)
            stepped += o.stepped;
        Check(stepped > run.size() / 2, std::string("v2 drives ") + s.file, std::to_string(stepped) + " of " + std::to_string(run.size()) + " feeds");
        for (const Window& w : s.windows)
        {
            const int n = CountChanges(f, run, w.t0, w.t1);
            char detail[160];
            std::snprintf(detail, sizeof(detail), "%d changes in %.2f s = %.2f per second, budget %d (bound 3.0 per second)", n, static_cast<double>(w.t1 - w.t0),
                          static_cast<double>(n) / static_cast<double>(w.t1 - w.t0), w.budget);
            Check(n <= w.budget, std::string("churn ") + w.name, detail);
            if (w.reface)
            {
                const RefaceSwaps rs = FindRefaceSwaps(sdb, f, run, w.t0, w.t1);
                char sw[160];
                std::snprintf(sw, sizeof(sw), "%d reface start swaps, slowest %.0f ms after the pick (limit %.0f ms)", rs.count, static_cast<double>(rs.worstDelayS * 1000.0f),
                              static_cast<double>(kMaxRefaceSwapS * 1000.0f));
                Check(rs.count >= 1, std::string("reface swap present ") + w.name, sw);
                Check(rs.worstDelayS <= kMaxRefaceSwapS, std::string("reface swap not delayed ") + w.name, sw);
            }
            if (w.restarts)
            {
                const int nr = CountStartRestarts(sdb, f, run, w.t0, w.t1);
                char rd[120];
                std::snprintf(rd, sizeof(rd), "%d start clips restarted from their first frame by another start (limit %d)", nr, kMaxStartRestarts);
                Check(nr <= kMaxStartRestarts, std::string("start restart ") + w.name, rd);
            }
        }
    }

    g_dumpBody = 0;
    // B: an ordinary release at speed (run_arc_left / stop of the same session): the Stop clip is picked on the release frame
    {
        FeedFile f;
        if (!LoadFeedAny(sliceDir + "\\churn_stop.feed", f))
        {
            std::printf("FAIL slice churn_stop.feed missing / unreadable\n");
            return 3;
        }
        const std::vector<FrameOut> run = RunDriver(rig, f);
        const float tStop0 = 89.092f, tStop1 = 91.591f; // the report window of the 'stop' step
        size_t rel = f.fr.size();
        bool high = false;
        for (size_t i = 0; i < f.fr.size(); ++i)
        {
            const float mag = std::hypot(f.fr[i].mvx, f.fr[i].mvy);
            high = high || (mag > 0.5f && f.fr[i].t >= tStop0 - 0.5f);
            if (high && mag < 0.05f && f.fr[i].t >= tStop0 - 0.5f)
            {
                rel = i;
                break;
            }
        }
        Check(rel < f.fr.size(), "stop slice: the stick release is in the feed");
        int stopFrame = -1;
        int32_t last = -1;
        for (size_t i = rel < f.fr.size() ? rel : 0; i < run.size() && f.fr[i].t < tStop1; ++i)
        {
            if (!run[i].stepped || run[i].clip < 0 || run[i].clip == last)
                continue;
            last = run[i].clip;
            if (stopFrame < 0 && db->ClipName(static_cast<uint32_t>(run[i].clip)).find("_Stop_") != std::string_view::npos)
                stopFrame = static_cast<int>(i - rel);
        }
        Check(stopFrame >= 0 && stopFrame <= 1, "a stick release at speed picks its Stop clip on the release frame (no stick latency)",
              "stop clip " + std::to_string(stopFrame) + " feeds after the release");
    }
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAIL" : "OK", g_pass, g_fail);
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return g_fail ? 1 : 0;
}

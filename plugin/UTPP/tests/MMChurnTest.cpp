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
// Checks: A every window stays within its budget = the count with the guards (slalom 16 since the loop dwell override 0.2, 14 with 0.3; flick 7, flick_180_sprint / accel 4, dash_left 5,
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
//   second; the same with every guard off). churn_sprint_flick_man.feed (Male 17:19, needs the Male bundle): 6 changes (9 without the turn commit).
//   E (gait return hold, adapter::MapToChooser, no data files): a slow rising stick 0.3 -> 0.6 after a short run keeps the Run gait for 0.3 s from the last
//   real Run frame, once, without flicker (RunGaitReturnHold; mutation 27 removes the hold: "keeps Run" fails too).
//   B an ordinary release at speed still picks its Stop clip on the release frame (no stick latency): none of the guards may delay a stick edge
//   (the start upshift guard only defers the swap of a young start clip to the faster gait's start; the first start of a press and every release / stop are not held).
//   F (gait return hold, no data files): the hold of check E driven by time with uneven frame times (1/43, 1/60, 1/120 s in turn) lasts as long and
//   changes the gait as often as at a fixed 60 feeds per second (the stick's rise is a rate per second, MapConfig::gaitReturnRiseRate).
//   G (turn commit, MatcherConfig::turnCommit; the flick windows): the turn clip picked on the reversal plays until its authored root turn is done
//   (without the commit the Male sprint flick dropped Run_Turn_L_180 at clip time 0.80 for Run_Loop_B, then a loop / reface chain followed).
//   H (turn commit and release, churn_turn_release.feed = Female keyboard-like strafe taps): the keys released while a committed turn clip
//   plays end the commit at once and the Stop clip is picked on the release frame, as in check B.
//   K (heading catch-up, MatcherConfig::headingCatchUpS; churn_kbd_*.feed = Female 2026-11-14 03:53, full-deflection direction keys): after each
//   heading step (the wanted direction turns >= 35 deg between two feeds) at most two clip changes until the next step for a 45 / 90 deg
//   change and at most one per flip of the left / right taps (0.35 s apart); the windows also keep 3.0 changes per second.
// Mutations (UTPP_MUTATION_JUMPPICK, run-time, src/MutationSelect.hpp): 24 no loop dwell, 25 no young event clip bias, 26 no dodge hold,
// 27 no gait return hold, 28 no start upshift hold, 29 reface starts held 0.35 s (check C), 30 no start restart / stalled dwell
// (check D), 31 the gait return hold's rise test per feed instead of per second (check F), 32 no turn commit (check G and the flick budgets),
// 33 no heading catch-up (check K).
// Each must fail its check
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
    bool turnHeld = false; // check G applies: a turn clip picked on the reversal plays through its authored root turn
    int edgeMax = -1;      // check K applies: at most this many clip changes after each digital heading step (-1 = not checked)
};

// check G (MatcherConfig::turnCommit): pivot / moving-turn clips (steer::IsPivot / IsMoveTurn) picked in [aT0, aT1) that were replaced before their authored
// root turn was done (more than 1 deg of root yaw left); aPicks = how many such clips were picked
struct TurnDrops
{
    int picks = 0, drops = 0;
    std::string first; // the first early replacement: "clip ct -> clip"
};
float TurnEndS(const mm::motion::MotionDatabase& aDb, uint32_t aClip)
{
    const auto& cl = aDb.Clip(aClip);
    double total = 0.0, done = 0.0;
    for (uint32_t f = 0; f < cl.numFrames; ++f)
        total += std::fabs(aDb.Root(cl.firstFrame + f).dyaw);
    for (uint32_t f = 0; f < cl.numFrames; ++f)
    {
        done += std::fabs(aDb.Root(cl.firstFrame + f).dyaw);
        if (total - done <= 3.14159265358979 / 180.0)
            return static_cast<float>(f) / aDb.Header().fps;
    }
    return 0.0f;
}
TurnDrops FindTurnDrops(const mm::motion::MotionDatabase& aDb, const FeedFile& aFeed, const std::vector<FrameOut>& aRun, float aT0, float aT1)
{
    TurnDrops r;
    int32_t clip = -1;
    float ct = 0.0f, pickT = 0.0f;
    bool turn = false;
    for (size_t i = 0; i < aRun.size() && i < aFeed.fr.size(); ++i)
    {
        if (!aRun[i].stepped || aRun[i].clip < 0)
            continue;
        const float t = aFeed.fr[i].t;
        if (aRun[i].clip != clip)
        {
            // the clip time shown last is one update before the replacement: 0.05 s of tolerance (a 20 Hz update)
            if (turn && pickT >= aT0 && pickT < aT1 && ct + 0.05f < TurnEndS(aDb, static_cast<uint32_t>(clip)))
            {
                if (!r.drops++)
                    r.first = std::string(aDb.ClipName(static_cast<uint32_t>(clip))) + " ct " + std::to_string(ct) + " -> " +
                              std::string(aDb.ClipName(static_cast<uint32_t>(aRun[i].clip)));
            }
            const uint8_t cls = mm::steer::ClassifyClip(aDb.ClipName(static_cast<uint32_t>(aRun[i].clip)));
            turn = mm::steer::IsPivot(cls) || mm::steer::IsMoveTurn(cls);
            pickT = t;
            r.picks += turn && t >= aT0 && t < aT1;
        }
        clip = aRun[i].clip;
        ct = aRun[i].clipTime;
    }
    return r;
}

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

// check K (MatcherConfig::headingCatchUpS): clip changes after each digital heading step of the window - a feed whose wanted direction (the feed's
// Locomotion target yaw) turned by >= 35 deg from the previous feed - counted until the next step or the key release (target gone)
struct EdgeChanges
{
    int edges = 0, worst = 0;
    float worstT = 0.0f;
};
EdgeChanges ChangesPerHeadingStep(const FeedFile& aFeed, const std::vector<FrameOut>& aRun, float aT0, float aT1)
{
    EdgeChanges r;
    int cur = -1; // changes since the last step (-1 = no step yet)
    float curT = 0.0f;
    const auto close = [&] {
        if (cur > r.worst)
            r.worst = cur, r.worstT = curT;
        cur = -1;
    };
    int32_t clip = -1, dbi = -1;
    float ct = 0.0f, tPrev = 0.0f;
    bool have = false;
    for (size_t i = 1; i < aRun.size() && i < aFeed.fr.size(); ++i)
    {
        const Frame& p = aFeed.fr[i - 1];
        const Frame& f = aFeed.fr[i];
        const float t = f.t;
        if (t >= aT1)
            break;
        if (!f.haveTarget)
            close();
        else if (t >= aT0 && p.haveTarget && std::fabs(std::remainder(f.targetYawDeg - p.targetYawDeg, 360.0f)) >= 35.0f)
        {
            close();
            cur = 0;
            curT = t;
            ++r.edges;
        }
        if (!aRun[i].stepped)
            continue;
        const bool differs = aRun[i].clip != clip || aRun[i].db != dbi;
        const bool relabel = differs && aRun[i].clip == clip && std::fabs((aRun[i].clipTime - ct) - (t - tPrev)) <= 0.05f;
        if (have && differs && !relabel && cur >= 0)
            ++cur;
        clip = aRun[i].clip;
        dbi = aRun[i].db;
        ct = aRun[i].clipTime;
        tPrev = t;
        have = true;
    }
    close();
    return r;
}

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
// check F (gait return hold under uneven frame times): the same short run, dip and slow rising stick as check E, driven by time instead of by feed
// count. The stick rises at aRampPerS (stick per second) from 0.3 to 0.6; aUneven cycles the feed time through 1/43, 1/60 and 1/120 s (a game that
// does not hold a fixed frame rate), else every feed is 1/60 s. "Rising" is a rate (MapConfig::gaitReturnRiseRate), so the hold must last as long in
// seconds and change the gait as often at any frame time; a fixed rise per feed (mutation 31) drops the hold on every short frame.
GaitHoldRun RunGaitReturnHoldTimed(float aRampPerS, bool aUneven)
{
    namespace ad = utpp::mm::adapter;
    ad::MapConfig mc;
    ad::MapState ms;
    GaitHoldRun out;
    const float dts[3] = {1.0f / 43.0f, 1.0f / 60.0f, 1.0f / 120.0f};
    const float tRun = 0.5f, tDip = 0.05f, tRamp = 0.3f / aRampPerS, tTail = 0.5f;
    float t = 0.0f;
    int prevGait = 1, k = 0;
    bool sawWalk = false;
    while (t < tRun + tDip + tRamp + tTail)
    {
        const float dt = aUneven ? dts[k++ % 3] : 1.0f / 60.0f;
        t += dt;
        float stick = 1.0f, speed = 3.84f;
        if (t > tRun)
            stick = 0.3f, speed = 3.0f;
        if (t > tRun + tDip)
            stick = std::fmin(0.6f, 0.3f + aRampPerS * (t - tRun - tDip)), speed = 2.5f;
        const float vel[3] = {0.0f, speed, 0.0f}, mv[2] = {0.0f, stick};
        ad::MotionFeed f = ad::MakeMotionFeed(vel, mv, dt, 0u, 0, ad::AirFeed{}, 0.0f, 0.0f);
        ad::MapContext ctx;
        ctx.rootYawValid = true;
        const utpp::mm::motion::ChooserState cs = ad::MapToChooser(f, mc, ms, ctx);
        if (t <= tRun + tDip)
        {
            prevGait = cs.gait;
            continue;
        }
        if (cs.gait != prevGait)
            ++out.flips;
        // the first ramp feed may still be Walk (its rise since the dip is below the rate threshold): count Walk only once the hold has begun
        sawWalk = sawWalk || (cs.gait == 0 && out.forcedFrames > 0);
        if (cs.gait >= 1)
        {
            ++out.forcedFrames;
            out.lastRunT = t - tRun - tDip;
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
    // F: the gait return hold under uneven frame times (1/43, 1/60, 1/120 s in turn) lasts as long and changes the gait as often as at a fixed 60 per second
    {
        const float ramp = 0.36f; // stick per second: above the 0.3 per second rise rate, 0.003 per feed at 120 per second
        const GaitHoldRun fixed = RunGaitReturnHoldTimed(ramp, false), uneven = RunGaitReturnHoldTimed(ramp, true);
        char fd[240];
        std::snprintf(fd, sizeof(fd), "fixed 60/s: %d gait changes, last Run %.3f s; uneven: %d gait changes, last Run %.3f s, %d Run frames after the first Walk frame",
                      fixed.flips, static_cast<double>(fixed.lastRunT), uneven.flips, static_cast<double>(uneven.lastRunT), uneven.runsAfterWalk);
        Check(fixed.forcedFrames >= 3 && uneven.forcedFrames >= 3, "gait return hold holds under fixed and uneven frame times", fd);
        Check(uneven.flips <= 2 && uneven.runsAfterWalk == 0, "gait return hold does not flicker under uneven frame times", fd);
        Check(std::fabs(uneven.lastRunT - fixed.lastRunT) <= 1.0f / 43.0f + 1e-3f, "gait return hold lasts as long under uneven frame times", fd);
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
        {"churn_flick_run.feed", {{"flick_180_run flick", 151.308f, 153.808f, 6, false, false, true}}},
        {"churn_flick_sprint.feed", {{"flick_180_sprint accel", 162.384f, 163.884f, 4, true}}},
        {"churn_reface_sprint.feed", {{"flick_180_sprint accel (second run)", 169.529f, 171.029f, 4, true}}},
        {"churn_reface_whip.feed", {{"camera_whip_180 accel", 185.111f, 187.111f, 3, true}}},
        // Female 2026-11-11 17:37: the stick held against a blocked capsule (weapon armed / unarmed walk): the start clips restarted every 0.2 s (13 changes in 3 s)
        {"churn_stalled_start.feed", {{"weapon_aim walk_armed", 307.026f, 310.026f, 6, false, true}, {"weapon_aim aim_walk", 310.026f, 313.026f, 3, false, true},
                                       {"weapon_walk walk_unarmed", 318.44f, 321.442f, 6, false, true}}},
        // Male 2026-11-11 17:19: sprint flick; 9 changes in 2.5 s (3.6 per second, game 9) before the turn commit, 6 with it
        {"churn_sprint_flick_man.feed", {{"flick_180_sprint flick (Male)", 181.397f, 183.897f, 6, false, false, true}}, true},
        {"churn_dash.feed", {{"sprint_dash dash_left", 258.284f, 260.284f, 5}, {"sprint_dash dash_right", 260.284f, 262.284f, 4}, {"sprint_dash dash_back", 262.284f, 264.284f, 3}, {"sprint_dash sprint_again", 264.284f, 267.284f, 7}}},
        // Female 2026-11-14 03:53, keyboard-like input (full-deflection direction keys): the harness step windows keep 3.0 changes per second; check K
        // runs over the key edges of the whole sequence (the key edge of a step reaches the feed ~0.13 s before the harness step window starts)
        {"churn_kbd_switch90.feed", {{"kbd_switch_90 right", 36.10f, 37.50f, 4}, {"kbd_switch_90 back", 37.50f, 38.90f, 4}, {"kbd_switch_90 left", 38.90f, 40.30f, 4},
                                     {"kbd_switch_90 fwd_again", 40.30f, 41.70f, 4}, {"kbd_switch_90 key edges", 35.9f, 41.6f, 99, false, false, false, 2}}},
        {"churn_kbd_diagonals.feed", {{"kbd_diagonals fwd_right", 80.16f, 81.06f, 2}, {"kbd_diagonals right", 81.06f, 81.96f, 2}, {"kbd_diagonals back_right", 81.96f, 82.86f, 2},
                                      {"kbd_diagonals back", 82.86f, 83.76f, 2}, {"kbd_diagonals back_left", 83.76f, 84.66f, 2}, {"kbd_diagonals left", 84.66f, 85.56f, 2},
                                      {"kbd_diagonals fwd_left", 85.56f, 86.46f, 2}, {"kbd_diagonals key edges", 79.9f, 87.2f, 99, false, false, false, 2}}},
        {"churn_kbd_strafe.feed", {{"kbd_strafe_weave weave", 100.56f, 103.76f, 9}, {"kbd_strafe_weave strafe_taps", 103.76f, 106.56f, 8},
                                   {"kbd_strafe_weave weave key edges", 100.4f, 103.6f, 99, false, false, false, 2},
                                   {"kbd_strafe_weave strafe_taps key edges", 103.6f, 106.5f, 99, false, false, false, 1}}},
        {"churn_kbd_switch180.feed", {{"kbd_switch_180 back", 56.77f, 58.37f, 4}, {"kbd_switch_180 fwd_again", 58.37f, 59.97f, 4}, {"kbd_switch_180 right", 62.77f, 64.37f, 4},
                                      {"kbd_switch_180 left_again", 64.37f, 65.97f, 4}}},
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
            if (w.turnHeld)
            {
                const TurnDrops td = FindTurnDrops(sdb, f, run, w.t0, w.t1);
                const std::string tdd = std::to_string(td.picks) + " turn clips picked, " + std::to_string(td.drops) + " replaced before their turn was done" +
                                        (td.first.empty() ? std::string() : " (" + td.first + ")");
                Check(td.picks >= 1 && td.drops == 0, std::string("turn played through ") + w.name, tdd);
            }
            if (w.edgeMax >= 0)
            {
                const EdgeChanges ec = ChangesPerHeadingStep(f, run, w.t0, w.t1);
                char ed[160];
                std::snprintf(ed, sizeof(ed), "%d heading steps, at most %d clip changes after one (the step at %.3f s), limit %d", ec.edges, ec.worst,
                              static_cast<double>(ec.worstT), w.edgeMax);
                Check(ec.edges >= 1 && ec.worst <= w.edgeMax, std::string("changes per heading step ") + w.name, ed);
            }
        }
    }

    g_dumpBody = 0;
    // the first stick release (move input > 0.5, then < 0.05) at or after aFrom: feeds from it to the first Stop clip picked before aTo (-1 = none)
    const auto stopAfterRelease = [&](const char* aFile, float aFrom, float aTo, int& aStopFrame) {
        FeedFile f;
        if (!LoadFeedAny(sliceDir + "\\" + aFile, f))
            return false;
        const std::vector<FrameOut> run = RunDriver(rig, f);
        size_t rel = f.fr.size();
        bool high = false;
        for (size_t i = 0; i < f.fr.size(); ++i)
        {
            const float mag = std::hypot(f.fr[i].mvx, f.fr[i].mvy);
            high = high || (mag > 0.5f && f.fr[i].t >= aFrom);
            if (high && mag < 0.05f && f.fr[i].t >= aFrom)
            {
                rel = i;
                break;
            }
        }
        Check(rel < f.fr.size(), std::string(aFile) + ": the stick release is in the feed");
        aStopFrame = -1;
        int32_t last = -1;
        for (size_t i = rel < f.fr.size() ? rel : 0; i < run.size() && f.fr[i].t < aTo; ++i)
        {
            if (!run[i].stepped || run[i].clip < 0 || run[i].clip == last)
                continue;
            last = run[i].clip;
            if (aStopFrame < 0 && db->ClipName(static_cast<uint32_t>(run[i].clip)).find("_Stop_") != std::string_view::npos)
                aStopFrame = static_cast<int>(i - rel);
        }
        return true;
    };
    // B: an ordinary release at speed (run_arc_left / stop of the same session): the Stop clip is picked on the release frame
    {
        const float tStop0 = 89.092f, tStop1 = 91.591f; // the report window of the 'stop' step
        int stopFrame = -1;
        if (!stopAfterRelease("churn_stop.feed", tStop0 - 0.5f, tStop1, stopFrame))
        {
            std::printf("FAIL slice churn_stop.feed missing / unreadable\n");
            return 3;
        }
        Check(stopFrame >= 0 && stopFrame <= 1, "a stick release at speed picks its Stop clip on the release frame (no stick latency)",
              "stop clip " + std::to_string(stopFrame) + " feeds after the release");
    }
    // H (turn commit and release, churn_turn_release.feed = Female keyboard-like strafe taps, the keys released while Run_Turn_L_135 plays):
    // a release ends the turn commit at once; the Stop clip is picked on the release frame (with a held turn: no Stop, idle 0.2 s later)
    {
        int stopFrame = -1;
        if (!stopAfterRelease("churn_turn_release.feed", 107.0f, 109.41f, stopFrame))
        {
            std::printf("FAIL slice churn_turn_release.feed missing / unreadable\n");
            return 3;
        }
        Check(stopFrame >= 0 && stopFrame <= 1, "a stick release during a turn picks its Stop clip on the release frame (turn commit released)",
              "stop clip " + std::to_string(stopFrame) + " feeds after the release");
    }
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAIL" : "OK", g_pass, g_fail);
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return g_fail ? 1 : 0;
}

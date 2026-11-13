// Slide EXIT offline replay (branch fix/slide-exit): real game feeds (tests/data/slide_exit_feeds.txt, mm.feedDump of the
// 2026-10-21 03:37 and 2026-10-20 10:11 sessions, +-3 s around every slide exit) go through the production stack minus the game:
// adapter::MapToChooser -> BuildQueryTrajectory -> ChooserMatcher::Update -> motionstack::BlendStack (the live outgoing-clip crossfade
// PoseEvaluator uses for the base pose, motion FastFeet+Root duration factors) on the installed slide bundle
// (utpp_motion_woman_knee_twistfix_slide). The ROOT / orientation-warp graph and the overlay layers are not part of it (graph
// disabled); the metric is the model-space rotation of Hips / Spine3 of the blended base pose, which is where the 2026-10-21
// 03:37 capture showed the exit pops (stage 1 "after Blend" already carried the 50-74 deg / 0.1 s step).
//
// Usage: utpp_mm_slide_exit_test <slide_exit_feeds.txt> <bundle path without extension> [--report] [--csv out.csv]
//                                [--set <knob> <value>]...   (experiments; knobs: see SetKnob)
//
// Replay configuration = MotionDriver.cpp StepLocked (2752-2768): slide on with the slide spec of the bundle and
// slideDeceleration kSlideDecelerationMs2, stanceSpeedBlendTime of the matcher, the learned gravity. NOT in the feed dump and
// therefore a stated gap: the native capsuleBraking/Acceleration stats (game 24 / 3.91 m/s^2, fresh in ~15 % of the 03:37 feeds)
// and visualStrafe (hip-fire pose; needs the weapon state). The same asserts are repeated for both of them (variants below).
//
// Measured per exit (first frame with MovementMode_LastFrame == Sliding):
//  - ground exits (OnGround on the first frame): max rotation of Hips / Spine3 over 0.1 s in [exit - 0.1 s, exit + kRotWindowS],
//    the clip phase the exit clip is entered at, how long it stays selected, Jump_* picks within 1 s without a PSM air state;
//  - jump exits (InAir with a PSM air state, Jump 18 + vz +5.4 .. +5.7): must still pick a Jump_* take-off clip;
//  - air frames the PSM does not confirm (the 0.08-0.10 s slope / step drops of 03:37 at PSM Slide 5 / Crouch 3);
//  - stance: fed stance runs shorter than 0.15 s, and the stance during a Dodge from the crouch.
// Each fix has a parameter (MatcherConfig::slideExitMaxEntryS / slideExitFamily / slideExitMinHoldS / slideExitMinPlayRate /
// blendSlideExit, MapConfig::slideAirGraceS / stanceFilterS / stanceHoldInDodge) whose pre-fix value is the "off" setting: the same
// replay with one fix off must violate its bound (mutation), and with all of them off the replay shows the pre-fix numbers
// (60-90 deg per 0.1 s, exit clip entered at 0.5 s, 3.47 s hold, 4 false drops, 27 short stance runs). Exits of slides shorter than
// 0.4 s are not bound by the rotation / entry checks: the displayed pose is still the 0.2 s entry fade there, which no slide pose row
// describes (MatcherConfig::slideExitSettledS keeps their pre-fix behaviour); they are printed.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../src/mm/MotionBlendStack.hpp"

namespace adapter = utpp::mm::adapter;
namespace integ = utpp::mm::integ;
namespace motion = utpp::mm::motion;
namespace motionstack = utpp::mm::motionstack;
namespace math = utpp::mm::math;
namespace chooser = utpp::mm::chooser;
using utpp::mm::BoneTransform;

namespace
{
int g_fail = 0, g_checks = 0;
bool g_report = false;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
        ++g_fail;
    if (!aOk || g_report)
        std::printf("%s %s (%.3f, %.3f)\n", aOk ? "PASS" : "FAIL", aWhat, aA, aB);
}

// ---- bounds the production configuration must meet (the pre-fix replay: 60-90 deg, 0.00-0.53 s, 24 short runs, 4 false drops)
constexpr double kMaxRotDeg = 25.0;       // task bound: Hips / Spine3 rotation per 0.1 s on a ground exit
constexpr double kMaxRotRunDeg = 32.0;    // Hips / Spine3 of the Run / Sprint exits: the authored clips peak at 28.7 / 34.7 deg per 0.1 s at rate 1 (pre-fix 70-83)
constexpr double kRotWindowS = 0.8;       // exit window [exit - 0.1, exit + 0.8] s at most, see Analyse
constexpr double kMinSlideForRotationS = 0.4; // shorter slides are still inside their 0.2 s entry blend at the exit
constexpr double kMaxHandoverDeg = 50.0;  // regression guard of the blend into ordinary locomotion after the exit clip
constexpr double kMaxEntryClipTimeS = 0.25; // the exit clip is entered at its start (pre-fix 0.30 .. 0.53 s)
constexpr double kYieldToleranceS = 0.07; // the hold ends within this long after a chooser state change
constexpr double kHoldToleranceS = 0.06;  // the exit clip stays selected at least slideExitMinHoldS - this
constexpr double kMaxExitHoldS = 1.6;     // and (task 2) not much longer than its rising part: pre-fix replay 3.47 s
constexpr double kShortStanceRunS = 0.15; // stance changes shorter than this are filtered

struct Row
{
    double t = 0;
    float dt = 0, vel[3] = {0, 0, 0}, mv[2] = {0, 0}, yaw = 0, cam = 0, target = 0;
    int loco = 0;
    uint32_t psm = 0;
    bool haveTarget = false;
    int onGround = -1;
    float dist = -1;
    int fall = 0, walk = 0;
};
struct Section
{
    std::string name;
    std::vector<Row> rows;
};

float Num(const std::string& aTok, float aDef = 0.0f) { return aTok == "-" ? aDef : std::strtof(aTok.c_str(), nullptr); }

bool LoadFixture(const char* aPath, std::vector<Section>& aOut)
{
    std::ifstream f(aPath);
    if (!f)
        return false;
    std::string line;
    std::vector<std::string> cols;
    while (std::getline(f, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind("# cols", 0) == 0)
        {
            std::istringstream is(line.substr(6));
            std::string c;
            cols.clear();
            while (is >> c)
                cols.push_back(c);
            continue;
        }
        if (line.empty() || line[0] == '#')
            continue;
        if (line.rfind("@slide ", 0) == 0)
        {
            std::string name = line.substr(7);
            name = name.substr(0, name.find(' '));
            aOut.push_back({name, {}});
            continue;
        }
        if (aOut.empty() || cols.empty())
            return false;
        std::istringstream is(line);
        std::vector<std::string> tok;
        std::string t;
        while (is >> t)
            tok.push_back(t);
        if (tok.size() != cols.size())
            return false;
        auto at = [&](const char* aName) -> const std::string& {
            for (size_t i = 0; i < cols.size(); ++i)
                if (cols[i] == aName)
                    return tok[i];
            static const std::string kDash = "-";
            return kDash;
        };
        Row r;
        r.t = std::strtod(at("t").c_str(), nullptr);
        r.dt = Num(at("dt"));
        r.vel[0] = Num(at("vx"));
        r.vel[1] = Num(at("vy"));
        r.vel[2] = Num(at("vz"));
        r.mv[0] = Num(at("mvx"));
        r.mv[1] = Num(at("mvy"));
        r.yaw = Num(at("yawDeg"));
        r.cam = Num(at("camDeg"));
        r.loco = static_cast<int>(Num(at("loco")));
        r.psm = static_cast<uint32_t>(std::strtoull(at("psm").c_str(), nullptr, 10));
        r.haveTarget = at("haveTarget") == "1";
        r.target = Num(at("targetYawDeg"));
        r.onGround = at("onGround") == "-" ? -1 : static_cast<int>(Num(at("onGround")));
        r.dist = Num(at("distGround"), -1.0f);
        r.fall = static_cast<int>(Num(at("psmFall")));
        r.walk = static_cast<int>(Num(at("walk")));
        aOut.back().rows.push_back(r);
    }
    return !aOut.empty();
}

// ---- one replay configuration: the knobs under test (production defaults) and the fidelity gaps
struct Variant
{
    std::string name = "prod";
    adapter::MapConfig mc;
    motion::MatcherConfig cfg;
    // body turn during the slide (synthetic): the PSM Slide rows get yaw / camera / target + yawShiftDeg, afterwards the shift fades
    // out over yawShiftRampS. > 45 deg off the capsule velocity = MovementDirection Other = the KneesOut slide family (the game
    // feeds hold only two real KneesOut slides; the real slides turn up to 38 deg off the body).
    float yawShiftDeg = 0.0f, yawShiftRampS = 0.3f;
    // aim / motion changes after the slide (synthetic): aimMode 1 = RMB held from the slide end (stationaryAimTurn while stopped),
    // 2 = a shot turn 0.3 .. 0.6 s after the slide end; reverseAfterS > 0 = the stick and the capsule reverse that long after the slide.
    int aimMode = 0;
    float reverseAfterS = 0.0f, reverseFirstS = 0.0f; // reverseFirstS > 0 = reversed during the first reverseFirstS after the slide too
    Variant()
    {
        mc.slide = true;
        mc.slideDeceleration = adapter::kSlideDecelerationMs2; // MotionDriver.cpp 2757
        mc.slideAirGraceS = adapter::kSlideAirGraceSeconds;    // MotionDriver.cpp: the runtime enables the fix-slide-exit MapConfig fields
        mc.stanceFilterS = adapter::kStanceFilterSeconds;
        mc.stanceHoldInDodge = true;
        mc.stanceCleanEdge = true; // P1 (fix batch 2): the runtime lets a clean Stand <-> Crouch edge bypass the filter
        mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
        mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    }
};

bool SetKnob(Variant& v, const std::string& aName, float aValue)
{
    if (aName == "slideExitYieldS")
        v.cfg.slideExitYieldS = aValue;
    else if (aName == "slideExitMinHoldS")
        v.cfg.slideExitMinHoldS = aValue;
    else if (aName == "slideExitFamily")
        v.cfg.slideExitFamily = aValue != 0.0f;
    else if (aName == "slideExitMaxEntryS")
        v.cfg.slideExitMaxEntryS = aValue;
    else if (aName == "slideExitSettledS")
        v.cfg.slideExitSettledS = aValue;
    else if (aName == "slideExitBiasS")
        v.cfg.slideExitBiasS = aValue;
    else if (aName == "slideExitMinPlayRate")
        v.cfg.slideExitMinPlayRate = aValue;
    else if (aName == "blendSlideExit")
        v.cfg.blendSlideExit = aValue;
    else if (aName == "slideAirGraceS")
        v.mc.slideAirGraceS = aValue;
    else if (aName == "stanceFilterS")
        v.mc.stanceFilterS = aValue;
    else if (aName == "stanceHoldInDodge")
        v.mc.stanceHoldInDodge = aValue != 0.0f;
    else if (aName == "blendSlide")
        v.cfg.blendSlide = aValue;
    else if (aName == "slideEntryMaxS") // RC2 slide entry cap (0 = off)
        v.cfg.slideEntryMaxS = aValue;
    else if (aName == "slideIntoHandover") // RC3 Into -> Loop handover mid-slide
        v.cfg.slideIntoHandover = aValue != 0.0f;
    else if (aName == "braking") // native capsule braking stat, m/s^2 (fresh game value 24)
        v.mc.capsuleBrakingDeceleration = aValue;
    else if (aName == "accel") // native capsule acceleration, m/s^2 (fresh game values 3.91 / 6.80)
        v.mc.capsuleAcceleration = aValue;
    else if (aName == "visualStrafe")
        v.mc.visualStrafe = aValue != 0.0f;
    else
        return false;
    return true;
}

struct Frame
{
    double t = 0;
    int detail = 0;
    int stanceRaw = 0; // PSM stance
    int stance = 0;    // stance fed to the matcher
    int mode = 0, modeLast = 0, gait = 0, moveState = 0, flags = 0;
    bool stanceRawOk = true; // ChooserState::stanceRaw == the PSM stance of that feed
    std::string clip, db;
    bool jumped = false;
    float blend = 0, cost = 0, contCost = 0, clipTime = 0, rate = 1;
    float rotHips = 0, rotSpine3 = 0; // rotation over the next 0.1 s (filled after the run)
    math::Quat qHips, qSpine3;
};

struct Model
{
    motion::MotionDatabase db;
    chooser::Table table;
    adapter::SlideSpec slide;
    adapter::QueryTimes qtimes;
    std::vector<float> factors;
    int boneHips = -1, boneSpine3 = -1;
    bool Load(const std::string& aStem, std::string& aErr)
    {
        if (!db.LoadFromFile((aStem + ".utppmm").c_str(), motion::kRigIdentityWoman, aErr))
            return false;
        if (!table.LoadFile((aStem + ".chooser.json").c_str(), aErr))
            return false;
        std::ifstream f(aStem + ".chooser.json", std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        if (!adapter::ParseSlideSpec(ss.str(), slide))
        {
            aErr = "no slide section";
            return false;
        }
        qtimes = adapter::BuildQueryTimes(db);
        factors.assign(db.NumBones(), 1.0f);
        for (uint32_t b = 0; b < db.NumBones(); ++b)
        {
            const auto name = db.BoneName(b);
            if (name == "Root" || name == "Trajectory")
                factors[b] = 0.1f;
            for (const char* leg : {"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"})
                if (name == leg)
                    factors[b] = 1.0f / 1.5f;
            if (name == "Hips")
                boneHips = int(b);
            if (name == "Spine3")
                boneSpine3 = int(b);
        }
        return boneHips >= 0 && boneSpine3 >= 0;
    }
};

math::Quat ModelRot(const motion::MotionDatabase& aDb, const BoneTransform* aPose, int aBone)
{
    math::Quat q{0, 0, 0, 1};
    std::vector<int> chain;
    for (int b = aBone; b >= 0; b = aDb.BoneParent(uint32_t(b)))
        chain.push_back(b);
    for (size_t i = chain.size(); i-- > 0;)
        q = math::QuatNormalize(math::QuatMul(q, aPose[chain[i]].q));
    return q;
}

float AngleDeg(const math::Quat& a, const math::Quat& b)
{
    const float d = std::fmin(1.0f, std::fabs(math::QuatDot(a, b)));
    return 2.0f * std::acos(d) * 57.29577951f;
}

std::vector<Frame> Replay(Model& m, const Section& s, const Variant& aVar)
{
    std::vector<Frame> out;
    motion::ChooserMatcher matcher;
    std::string err;
    if (!matcher.Init(&m.db, &m.table, aVar.cfg, err))
    {
        std::printf("FAIL: matcher init %s\n", err.c_str());
        ++g_fail;
        return out;
    }
    motionstack::BlendStack stack;
    stack.Init(m.db, m.factors);
    adapter::MapState map;
    std::vector<motion::TrajSample> traj(adapter::kQueryTrajMax);
    int32_t prevDb = -1;
    bool havePrev = false;
    double lastSlideT = -1.0e30, lastSlideRowT = -1.0e30;
    for (const Row& r0 : s.rows)
    {
        Row r = r0;
        if (aVar.yawShiftDeg != 0.0f)
        {
            if (adapter::DetailFromFeed(r.psm, r.loco) == adapter::kPsmSlide)
                lastSlideT = r.t;
            const double k = lastSlideT == r.t ? 1.0 : std::fmax(0.0, 1.0 - (r.t - lastSlideT) / double(aVar.yawShiftRampS));
            const float shift = aVar.yawShiftDeg * float(k);
            r.yaw += shift;
            r.cam += shift;
            r.target += shift;
        }
        const bool slideRow = adapter::DetailFromFeed(r.psm, r.loco) == adapter::kPsmSlide;
        if (slideRow)
            lastSlideRowT = r.t;
        const double sinceSlide = lastSlideRowT > -1.0e29 ? r.t - lastSlideRowT : -1.0;
        const bool afterSlide = !slideRow && sinceSlide > 0.0;
        const bool aimNow = aVar.aimMode == 1 && afterSlide;
        const bool shotNow = aVar.aimMode == 2 && afterSlide && sinceSlide >= 0.3 && sinceSlide < 0.6;
        if (afterSlide && ((aVar.reverseAfterS > 0.0f && sinceSlide >= double(aVar.reverseAfterS)) ||
                           (aVar.reverseFirstS > 0.0f && sinceSlide < double(aVar.reverseFirstS))))
        {
            r.vel[0] = -r.vel[0];
            r.vel[1] = -r.vel[1];
            r.mv[0] = -r.mv[0];
            r.mv[1] = -r.mv[1];
        }
        const float dt = std::fmin(r.dt > 0 ? r.dt : 0.0f, 0.25f);
        adapter::AirFeed air;
        air.valid = r.onGround >= 0;
        air.onGround = r.onGround != 0;
        air.distanceToGround = r.dist;
        air.psmFall = r.fall;
        adapter::MotionFeed f = adapter::MakeMotionFeed(r.vel, r.mv, dt, r.psm, r.loco, air, r.cam, r.yaw);
        f.wantsToWalk = r.walk != 0;
        f.haveTarget = r.haveTarget && std::isfinite(r.target);
        f.targetYawDeg = f.haveTarget ? r.target : 0.0f;
        f.aiming = aimNow || shotNow;
        adapter::MapContext ctx;
        ctx.aimFacing = aimNow || shotNow;
        ctx.shotTurn = shotNow;
        ctx.shotTargetYawDeg = r.yaw + 90.0f;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = r.yaw; // the visual root follows the actor (no turn-in-place driven by a root lag)
        if (prevDb >= 0)
            ctx.currentDbPivots = adapter::DatabaseHasPivotsTag(m.db.DbName(uint32_t(prevDb)));
        adapter::MapConfig mc = aVar.mc;
        mc.slideSpec = m.slide;
        mc.stanceSpeedBlendTime = matcher.BlendTimeFor(motion::ChooserState{}, motion::ChooserState{});
        mc.gravity = 16.0f;
        const motion::ChooserState cs = adapter::MapToChooser(f, mc, map, ctx);
        const uint32_t nt = adapter::BuildQueryTrajectory(map, m.qtimes, traj.data(), uint32_t(traj.size()));
        if (nt == 0)
            continue;
        motion::MatchResult res = matcher.Update(dt, traj.data(), nt, cs, nullptr);
        if (res.clip < 0)
            continue;
        if (!havePrev || (res.jumped && !res.blend.valid))
            stack.Reset();
        if (!stack.Step(res, dt))
            continue;
        havePrev = true;
        prevDb = res.database;
        Frame fr;
        fr.t = r.t;
        fr.detail = adapter::DetailFromFeed(r.psm, r.loco);
        fr.stanceRaw = adapter::PsmCrouchStance(fr.detail) ? 1 : 0;
        fr.stance = cs.stance;
        fr.mode = cs.movementMode;
        fr.modeLast = cs.movementModeLast;
        fr.gait = cs.gait;
        fr.moveState = cs.movementState;
        fr.stanceRawOk = cs.stanceRaw == (adapter::PsmCrouchStance(fr.detail) ? 1 : 0);
        fr.flags = motion::ChooserMatcher::HoldFlags(cs);
        fr.clip = std::string(m.db.ClipName(uint32_t(res.clip)));
        fr.db = std::string(m.db.DbName(uint32_t(res.database)));
        fr.jumped = res.jumped;
        fr.blend = res.jumped ? res.blend.blendTime : 0.0f;
        fr.cost = res.cost;
        fr.contCost = res.continuingCost;
        fr.clipTime = res.clipTime;
        fr.rate = res.playRate;
        fr.qHips = ModelRot(m.db, stack.Output(), m.boneHips);
        fr.qSpine3 = ModelRot(m.db, stack.Output(), m.boneSpine3);
        out.push_back(std::move(fr));
    }
    for (size_t i = 0; i < out.size(); ++i) // rotation over the next 0.1 s
    {
        size_t j = i;
        while (j + 1 < out.size() && out[j].t - out[i].t < 0.1 - 1e-6)
            ++j;
        if (out[j].t - out[i].t < 0.085)
            continue;
        out[i].rotHips = AngleDeg(out[i].qHips, out[j].qHips);
        out[i].rotSpine3 = AngleDeg(out[i].qSpine3, out[j].qSpine3);
    }
    return out;
}

struct Exit
{
    std::string section;
    double t = 0;
    bool ground = false;   // OnGround on the first frame after the slide
    bool psmAir = false;   // a PSM air state (Fall / Jump / SlideFall ...) on that frame
    bool run = false;      // exit into the Run / Sprint databases (authored Spine3 peak 35 deg per 0.1 s)
    float maxRot = 0;      // max of Hips / Spine3 rotation per 0.1 s in the exit window
    float rotHips = 0, rotSpine3 = 0;
    float handoverRot = 0; // same measure after the exit clip was left (ordinary locomotion blend; reported, not part of the exit bound)
    double slideDur = 0;   // seconds the slide lasted before this exit (a 0.1 s slide still carries its entry blend)
    float entryClipTime = 0;
    double hold = 0;       // seconds the first exit clip stays selected
    double groundS = 1.0e30; // seconds from the exit to the first frame that is not OnGround any more (take-off / new slide)
    int jumpPicks = 0;     // Jump_* picks within 1 s of a ground exit while the PSM never reported air
    double changeS = 1.0e30; // seconds from the exit to the first RELEASE of the exit hold: a stance change, a gait / movementState change
                             // that lasted slideExitYieldS, or a started aim / shot turn / pivot / spin / reface
    double flagS = 1.0e30;   // ... the part of it that is a started aim / shot turn / pivot / spin / reface
    int flags0 = 0;          // HoldFlags on the first exit feed (set at the selection)
    bool toStand = false;    // the first stance change is Crouch -> Stand
    double standS = 1.0e30;  // seconds to that stance change
    std::string changeDb;    // database picked 2 feeds after that stance change
    std::string slideDb;     // database of the last slide frame
    float intoPhase = 1.0e9f; // clip time of the last slide frame when it plays an Into clip (still lowering below 0.95 s)
    std::string firstClip, firstDb;
};

bool IsJumpClip(const std::string& aClip) { return aClip.find("Jump_") != std::string::npos; }

struct Summary
{
    std::vector<Exit> exits;
    int falseAirFrames = 0;   // InAir frames without a PSM air state while a slide context is on (chooser mode 1, PSM 5 / 3 / 1 ...)
    int falseJumpFrames = 0;  // frames playing a Jump_* clip with the PSM never airborne in the section
    int stanceRawBad = 0;     // frames whose ChooserState::stanceRaw differs from the PSM stance
    int shortStanceRuns = 0;  // fed stance runs < kShortStanceRunS (section ends excluded)
    int shortStanceRunsRaw = 0;
    int dodgesFromCrouch = 0, dodgeStanceChanges = 0; // fed stance changes during a Dodge entered from the crouch
    size_t frames = 0;
};

void Analyse(const Section& s, const std::vector<Frame>& fr, Summary& out, double aYieldS)
{
    out.frames += fr.size();
    for (size_t i = 1; i < fr.size(); ++i)
    {
        if (!(fr[i].modeLast == motion::kMovementSliding && fr[i].mode != motion::kMovementSliding))
            continue;
        Exit e;
        e.section = s.name;
        e.t = fr[i].t;
        e.ground = fr[i].mode == 0;
        e.psmAir = adapter::PsmAir(fr[i].detail);
        e.firstClip = fr[i].clip;
        e.firstDb = fr[i].db;
        e.run = fr[i].db.find("ExitToRun") != std::string::npos || fr[i].db.find("ExitToSprint") != std::string::npos;
        e.entryClipTime = fr[i].clipTime;
        size_t s0 = i;
        while (s0 > 0 && fr[s0 - 1].mode == motion::kMovementSliding)
            --s0;
        e.slideDur = fr[i].t - fr[s0].t;
        e.slideDb = fr[i - 1].db;
        if (fr[i - 1].clip.find("_Into_") != std::string::npos)
            e.intoPhase = fr[i - 1].clipTime;
        double runStart = -1.0, gaitS = 1.0e30;
        int armed = fr[i].flags; // flags set at the selection stay masked until they have dropped once (a start, not a level)
        e.flags0 = fr[i].flags;
        for (size_t q = i + 1; q < fr.size() && fr[q].mode == 0; ++q)
        {
            const double t = fr[q].t - fr[i].t;
            if (fr[q].stance != fr[i].stance && e.standS > 1.0e29)
            {
                e.standS = t;
                e.toStand = fr[i].stance == 1 && fr[q].stance == 0;
                e.changeDb = fr[std::min(q + 2, fr.size() - 1)].db;
            }
            if (fr[q].gait != fr[i].gait || fr[q].moveState != fr[i].moveState)
            {
                if (runStart < 0.0)
                    runStart = fr[q].t;
                if (fr[q].t - runStart >= aYieldS - 1.0e-6 && gaitS > 1.0e29)
                    gaitS = t;
            }
            else
                runStart = -1.0;
            if ((fr[q].flags & ~armed) != 0 && e.flagS > 1.0e29)
                e.flagS = t;
            armed &= fr[q].flags;
        }
        e.changeS = std::fmin(std::fmin(e.standS, gaitS), e.flagS);
        size_t k = i;
        while (k + 1 < fr.size() && fr[k + 1].clip == fr[i].clip)
            ++k;
        e.hold = fr[k].t - fr[i].t;
        // Rotation window: the 0.1 s windows that start in [exit - 0.1 s, end of the exit clip - 0.1 s] (at least up to exit + 0.1 s,
        // so a clip dropped at once is judged on its pop), cut where the character leaves the ground / enters a new slide
        // (a take-off or a second slide is its own blend). Everything after the exit clip is the handover to ordinary locomotion.
        double tGround = 1.0e30;
        for (size_t q = i; q < fr.size(); ++q)
            if (fr[q].mode != 0)
            {
                tGround = fr[q].t;
                break;
            }
        e.groundS = tGround - fr[i].t;
        // ... and windows that end before a chooser stance / gait / movementState change (that change starts the next state's own blend)
        const double hi = std::fmax(fr[i].t - 0.1, std::fmin(std::fmin(std::fmin(std::fmax(fr[i].t + 0.1, fr[k].t - 0.1), fr[i].t + kRotWindowS),
                                                                        tGround - 0.1), fr[i].t + e.changeS - 0.1));
        for (size_t q = i >= 8 ? i - 8 : 0; q < fr.size() && fr[q].t <= hi; ++q)
            if (fr[q].t >= fr[i].t - 0.1)
            {
                e.rotHips = std::fmax(e.rotHips, fr[q].rotHips);
                e.rotSpine3 = std::fmax(e.rotSpine3, fr[q].rotSpine3);
            }
        e.maxRot = std::fmax(e.rotHips, e.rotSpine3);
        for (size_t q = k; q < fr.size() && fr[q].t <= std::fmin(fr[k].t + kRotWindowS, tGround - 0.1); ++q)
            if (fr[q].t >= fr[k].t - 0.1)
                e.handoverRot = std::fmax(e.handoverRot, std::fmax(fr[q].rotHips, fr[q].rotSpine3));
        bool air = false;
        for (size_t q = i; q < fr.size() && fr[q].t - fr[i].t < 0.6; ++q) // the slide context, as the false air frames below
        {
            air = air || adapter::PsmAir(fr[q].detail);
            if (e.ground && IsJumpClip(fr[q].clip) && fr[q].jumped && !air)
                ++e.jumpPicks;
        }
        out.exits.push_back(std::move(e));
    }
    bool psmAirSeen = false; // whole-section: no PSM air state at all -> every InAir frame / Jump clip is unconfirmed
    for (const Frame& f : fr)
        psmAirSeen = psmAirSeen || adapter::PsmAir(f.detail);
    for (const Frame& f : fr)
        out.stanceRawBad += !f.stanceRawOk;
    double lastSlideT = -1.0e30; // air / Jump frames count only in the slide context (during the slide and 0.6 s = slideExitMapS after it)
    for (const Frame& f : fr)
    {
        if (f.mode == motion::kMovementSliding)
            lastSlideT = f.t;
        if (f.t - lastSlideT > 0.6)
            continue;
        if (f.mode == 1 && !adapter::PsmAir(f.detail))
            ++out.falseAirFrames;
        if (!psmAirSeen && IsJumpClip(f.clip))
            ++out.falseJumpFrames;
    }
    for (int pass = 0; pass < 2 && !fr.empty(); ++pass) // stance runs of the fed stance (pass 0) and the PSM stance (pass 1)
    {
        size_t st = 0;
        for (size_t i = 1; i <= fr.size(); ++i)
        {
            const int a = pass ? fr[st].stanceRaw : fr[st].stance;
            const int b = i < fr.size() ? (pass ? fr[i].stanceRaw : fr[i].stance) : -1;
            if (i == fr.size() || a != b)
            {
                const double dur = (i < fr.size() ? fr[i].t : fr[i - 1].t) - fr[st].t;
                if (st > 0 && i < fr.size() && dur < kShortStanceRunS)
                    (pass ? out.shortStanceRunsRaw : out.shortStanceRuns)++;
                st = i;
            }
        }
    }
    for (size_t i = 1; i < fr.size(); ++i) // Dodge 7 entered from the crouch: the chooser stance must not change inside it
        if (fr[i].detail == 7 && fr[i - 1].detail != 7 && fr[i - 1].stance == 1)
        {
            ++out.dodgesFromCrouch;
            for (size_t k = i; k < fr.size() && fr[k].detail == 7; ++k)
                if (fr[k].stance != 1)
                {
                    ++out.dodgeStanceChanges;
                    break;
                }
        }
}

Summary RunAll(Model& m, const std::vector<Section>& aSecs, const Variant& aVar, FILE* aCsv, bool aPrint)
{
    Summary sum;
    for (const Section& s : aSecs)
    {
        const std::vector<Frame> fr = Replay(m, s, aVar);
        if (aPrint)
            std::printf("== %s [%s]: %zu frames\n", s.name.c_str(), aVar.name.c_str(), fr.size());
        if (aCsv)
            for (const Frame& f : fr)
                std::fprintf(aCsv, "%s,%.4f,%d,%d,%d,%d,%d,%s,%s,%d,%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f,%d,%d\n", s.name.c_str(), f.t, f.detail, f.stanceRaw,
                             f.stance, f.mode, f.modeLast, f.db.c_str(), f.clip.c_str(), f.jumped ? 1 : 0, f.blend, f.cost, f.contCost,
                             f.clipTime, f.rotHips, f.rotSpine3, f.rate, f.gait, f.moveState);
        const size_t first = sum.exits.size();
        Analyse(s, fr, sum, double(aVar.cfg.slideExitYieldS));
        if (aPrint)
            for (size_t i = first; i < sum.exits.size(); ++i)
            {
                const Exit& e = sum.exits[i];
                std::printf("  exit t=%.2f %s psmAir %d slide %.2fs first=%s enteredAt %.2fs rot0.1s hips %.1f spine3 %.1f (handover %.1f) hold "
                            "%.2fs jumpPicks %d\n",
                            e.t, e.ground ? "ground" : "air", e.psmAir ? 1 : 0, e.slideDur, e.firstClip.c_str(), double(e.entryClipTime),
                            double(e.rotHips), double(e.rotSpine3), double(e.handoverRot), e.hold, e.jumpPicks);
            }
    }
    if (aPrint)
        std::printf("  [%s] false air frames %d, false jump frames %d, fed stance runs < %.2fs: %d (PSM %d), dodges from crouch %d changed %d\n",
                    aVar.name.c_str(), sum.falseAirFrames, sum.falseJumpFrames, kShortStanceRunS, sum.shortStanceRuns, sum.shortStanceRunsRaw,
                    sum.dodgesFromCrouch, sum.dodgeStanceChanges);
    return sum;
}

// ---- the bounds. Each returns true when the replay MEETS the bound.
struct Verdict
{
    bool rotation = true, entry = true, hold = true, holdMax = true, noFalseJump = true, noFalseAir = true, jumpsWork = true, stance = true,
         dodge = true, yields = true, standConform = true;
    int kneesExits = 0, idleExits = 0, runExits = 0, standChanges = 0, crouchIdleExits = 0, standIdleExits = 0, flagReleases = 0, flagsAtSelect = 0;
    int groundExits = 0, jumpExits = 0, rotationExits = 0;
    double worstHips = 0, worstSpine3 = 0, worstHandover = 0;
    double worstRot = 0, worstEntry = 0, shortestHold = 1e9, longestHold = 0;
};

Verdict Judge(const Summary& s, const Variant&)
{
    Verdict v;
    const double minHoldRef = double(motion::MatcherConfig{}.slideExitMinHoldS);
    std::vector<std::pair<long, std::string>> seen; // the same exit can sit in two overlapping sections
    for (const Exit& e : s.exits)
    {
        const std::pair<long, std::string> key{std::lround(e.t * 10.0), e.firstClip};
        if (std::find(seen.begin(), seen.end(), key) != seen.end())
            continue;
        seen.push_back(key);
        if (e.ground)
        {
            ++v.groundExits;
            v.shortestHold = std::fmin(v.shortestHold, std::fmin(e.hold, e.groundS));
            v.longestHold = std::fmax(v.longestHold, e.hold);
            // a slide shorter than 0.4 s still shows its entry fade, a slide cancelled while the Into clip is still lowering the body
            // (clip time < 0.95 s) has no lying pose: both keep the pre-fix behaviour (MatcherConfig::slideExitSettledS) and are printed only
            const bool settled = e.slideDur >= kMinSlideForRotationS && e.intoPhase >= 0.95f;
            if (settled)
            {
                ++v.rotationExits;
                v.worstEntry = std::fmax(v.worstEntry, double(e.entryClipTime));
                v.entry = v.entry && e.entryClipTime <= kMaxEntryClipTimeS;
                v.rotation = v.rotation && e.rotHips < (e.run ? kMaxRotRunDeg : kMaxRotDeg) && e.rotSpine3 < (e.run ? kMaxRotRunDeg : kMaxRotDeg);
                v.worstRot = std::fmax(v.worstRot, double(e.maxRot));
                v.worstHips = std::fmax(v.worstHips, double(e.rotHips));
                v.worstSpine3 = std::fmax(v.worstSpine3, double(e.rotSpine3));
            }
            v.worstHandover = std::fmax(v.worstHandover, double(e.handoverRot));
            v.flagReleases += e.flagS < minHoldRef && e.flagS <= e.changeS && e.flagS < e.groundS;
            v.flagsAtSelect += e.flags0 != 0;
            const double minHold = minHoldRef;
            v.hold = v.hold && e.hold >= std::fmin(std::fmin(minHold, e.groundS), e.changeS) - kHoldToleranceS;
            // a stance / gait / movementState change that reached the chooser ends the hold at once (not 0.6 s later)
            if (e.changeS < minHold && e.changeS < e.groundS)
                v.yields = v.yields && e.hold <= e.changeS + kYieldToleranceS;
            if (e.toStand && e.standS < 1.0)
            {
                ++v.standChanges;
                v.standConform = v.standConform && e.changeDb.find("Stand") != std::string::npos; // body follows the PSM: a Stand database
            }
            if (settled)
            {
                v.kneesExits += e.firstClip.find("KneesOut_Out") != std::string::npos;
                v.idleExits += e.firstDb.find("Idle") != std::string::npos;
                v.runExits += e.run;
                v.crouchIdleExits += e.firstDb.find("ExitToCrouchIdle") != std::string::npos;
                v.standIdleExits += e.firstDb.find("ExitToStandIdle") != std::string::npos;
            }
            if (e.firstDb.find("Idle") == std::string::npos) // the idle exit clips (6 s) run on into the idle by design
                v.holdMax = v.holdMax && e.hold <= kMaxExitHoldS;
            v.noFalseJump = v.noFalseJump && e.jumpPicks == 0;
        }
        else if (e.psmAir)
        {
            ++v.jumpExits;
            v.jumpsWork = v.jumpsWork && IsJumpClip(e.firstClip);
        }
    }
    v.noFalseJump = v.noFalseJump && s.falseJumpFrames == 0;
    v.noFalseAir = s.falseAirFrames == 0;
    v.stance = s.shortStanceRuns == 0;
    v.dodge = s.dodgesFromCrouch > 0 && s.dodgeStanceChanges == 0;
    return v;
}

void CheckVerdict(const char* aTag, const Verdict& v, const Summary& s)
{
    char buf[200];
    auto name = [&](const char* w) {
        std::snprintf(buf, sizeof buf, "%s: %s", aTag, w);
        return buf;
    };
    Check(v.groundExits >= 9, name("fixture holds >= 9 ground slide exits"), v.groundExits);
    Check(v.rotationExits >= 20, name("fixture holds >= 20 ground exits of slides >= 0.4 s"), v.rotationExits);
    // KneesOut >= 1: the second KneesOut exit (x1015-0659-161) came from the mid-slide Into re-jump chain that RC3 (MatcherConfig::slideIntoHandover)
    // removes (FootOut Into -> mirrored Into re-jump -> KneesOut Into); with the handover the slide stays FootOut. The KneesOut variant below
    // (yaw +60) keeps the KneesOut coverage (>= 15 exits).
    Check(v.kneesExits >= 1 && v.crouchIdleExits >= 2 && v.standIdleExits >= 2 && v.runExits >= 4,
          name("fixture covers KneesOut slides (>= 1), CrouchIdle (>= 2) / StandIdle (>= 2) and Run (>= 4) exits"), v.kneesExits, v.runExits);
    Check(v.worstHandover < kMaxHandoverDeg, name("handover from the exit clip to ordinary locomotion (worst deg, bound)"), v.worstHandover,
          kMaxHandoverDeg);
    Check(v.jumpExits >= 4, name("fixture holds >= 4 real jumps out of a slide (PSM Jump 18)"), v.jumpExits);
    Check(v.rotation, name("ground exits: Hips / Spine3 rotation per 0.1 s below the bound (worst deg, bound)"), v.worstRot, kMaxRotDeg);
    if (g_report)
        std::printf("   worst Hips %.1f, worst Spine3 %.1f deg per 0.1 s\n", v.worstHips, v.worstSpine3);
    Check(v.entry, name("ground exits: exit clip entered at its start (worst clip time s, bound)"), v.worstEntry, kMaxEntryClipTimeS);
    Check(v.hold, name("ground exits: exit clip kept for its minimum hold (shortest s)"), v.shortestHold);
    Check(v.holdMax, name("ground exits: exit clip not held longer than its rise (longest s, bound)"), v.longestHold, kMaxExitHoldS);
    Check(v.yields, name("ground exits: a chooser stance / gait / movementState change ends the exit hold at once"));
    Check(v.standChanges >= 1 && v.standConform, name("ground exits: a Crouch -> Stand change (CrouchSprint after the slide) picks a Stand database"),
          v.standChanges);
    Check(v.noFalseJump, name("no Jump_* clip after a slide without a PSM air state (picks)"), s.falseJumpFrames);
    Check(v.noFalseAir, name("no InAir frame without a PSM air state (frames)"), s.falseAirFrames);
    Check(v.jumpsWork, name("real jumps out of a slide still pick a Jump_* take-off clip"), v.jumpExits);
    Check(v.stance, name("fed stance runs shorter than 0.15 s (count)"), s.shortStanceRuns, s.shortStanceRunsRaw);
    Check(s.stanceRawBad == 0, name("ChooserState::stanceRaw = the PSM stance on every feed (bad feeds)"), s.stanceRawBad);
    Check(v.dodge, name("stance does not change inside a Dodge from the crouch (dodges, changed)"), s.dodgesFromCrouch, s.dodgeStanceChanges);
}

// ---- synthetic unit checks of the two MapToChooser changes (no database)
adapter::MotionFeed SynthFeed(float aDt, int aDetail, bool aOnGround, float aVz = 0.0f, float aSpeed = 3.0f)
{
    const float vel[3] = {0.0f, aSpeed, aVz};
    const float mv[2] = {0.0f, 1.0f};
    adapter::AirFeed air;
    air.valid = true;
    air.onGround = aOnGround;
    air.distanceToGround = aOnGround ? 0.0f : 0.12f;
    // psm block: detail + 1 in bits 16..23 together with kPsmDetailed (bit 6), as scripts/utpp.reds packs it
    const uint32_t psm = (uint32_t(aDetail + 1) << 16) | (1u << 6);
    return adapter::MakeMotionFeed(vel, mv, aDt, psm, 0, air, 0.0f, 0.0f);
}

void SyntheticChecks()
{
    adapter::MapConfig c = Variant{}.mc; // the runtime configuration
    c.slideSpec.declared = true;
    adapter::MapContext ctx;
    const float dt = 1.0f / 60.0f;
    auto run = [&](const adapter::MapConfig& aCfg, const std::vector<std::pair<int, bool>>& aSeq, float aSpeed = 3.0f) {
        adapter::MapState m;
        std::vector<std::pair<int, int>> out; // stance, mode
        for (const auto& [detail, ground] : aSeq)
        {
            const motion::ChooserState cs = adapter::MapToChooser(SynthFeed(dt, detail, ground, ground ? 0.0f : -1.5f, aSpeed), aCfg, m, ctx);
            out.push_back({cs.stance, cs.movementMode});
        }
        return out;
    };
    auto rep = [](int aDetail, int aN, bool aGround = true) { return std::vector<std::pair<int, bool>>(size_t(aN), {aDetail, aGround}); };
    auto cat = [](std::vector<std::pair<int, bool>> a, const std::vector<std::pair<int, bool>>& b) {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    };

    // --- stance filter: single-feed CrouchSprint 30 spikes inside the crouch never reach the chooser
    {
        auto seq = rep(3, 30);
        for (int i = 0; i < 3; ++i)
            seq = cat(cat(seq, rep(30, 1)), rep(3, 7));
        const auto on = run(c, seq);
        bool allCrouch = true;
        for (const auto& p : on)
            allCrouch = allCrouch && p.first == 1;
        Check(allCrouch, "synthetic: 1-feed CrouchSprint spikes inside the crouch are filtered");
        adapter::MapConfig off = c;
        off.stanceFilterS = 0.0f;
        int flips = 0;
        const auto o = run(off, seq);
        for (size_t i = 1; i < o.size(); ++i)
            flips += o[i].first != o[i - 1].first;
        Check(flips == 6, "synthetic (mutation): without the filter every spike flips the stance twice", flips);
    }
    // --- P1 (fix batch 2): a CLEAN crouch -> stand edge (Crouch 3 -> Stand 1) reaches the chooser on its own feed; a change that comes from
    // CrouchSprint 30 is still accepted after the filter time, not before (this was one assertion: "a real crouch -> stand change is delayed
    // by the filter time only", which pinned the 0.15 s lag of every stand-up)
    {
        auto firstStand = [](const std::vector<std::pair<int, int>>& aOut) {
            for (size_t i = 0; i < aOut.size(); ++i)
                if (aOut[i].first == 0)
                    return int(i);
            return -1;
        };
        const auto clean = run(c, cat(rep(3, 30), rep(1, 30)), 0.0f);
        Check(firstStand(clean) == 30, "synthetic: a clean crouch -> stand edge reaches the chooser on its own feed (feed, expected)", firstStand(clean), 30);
        const auto sprint = run(c, cat(rep(3, 30), rep(30, 30)));
        const int fs = firstStand(sprint);
        const double delay = (fs - 30 + 1) * double(dt); // the accepting feed is the filter-th feed of the new value
        Check(fs >= 30 && delay >= double(c.stanceFilterS) - 0.001 && delay <= double(c.stanceFilterS) + double(dt) + 0.001,
              "synthetic: a crouch -> CrouchSprint 30 change is delayed by the filter time only (s)", delay, c.stanceFilterS);
    }
    // --- Dodge from the crouch keeps the crouch; from the stand it stays stand
    {
        const auto on = run(c, cat(cat(rep(3, 30), rep(7, 12)), rep(3, 30)));
        bool crouch = true;
        for (const auto& p : on)
            crouch = crouch && p.first == 1;
        Check(crouch, "synthetic: Dodge 7 from the crouch keeps the crouch stance");
        adapter::MapConfig off = c;
        off.stanceHoldInDodge = false;
        off.stanceFilterS = 0.0f;
        const auto o = run(off, cat(cat(rep(3, 30), rep(7, 12)), rep(3, 30)));
        Check(o[35].first == 0, "synthetic (mutation): without the hold the Dodge reads as Stand", o[35].first);
        const auto st = run(c, cat(rep(1, 30), rep(7, 12)));
        Check(st.back().first == 0, "synthetic: Dodge 7 from the stand stays Stand", st.back().first);
    }
    // --- review 2026-10-21: a Dodge shortly after a stance change keeps the RAW stance before it, the pending change keeps ageing
    {
        auto firstStance = [](const std::vector<std::pair<int, int>>& aOut, int aStance, size_t aFrom) {
            for (size_t i = aFrom; i < aOut.size(); ++i)
                if (aOut[i].first == aStance)
                    return int(i);
            return -1;
        };
        // Crouch -> CrouchSprint 0.1 s -> Dodge 0.2 s -> Stand: the stand change reaches the chooser 0.15 s after the PSM change, inside the dodge
        // (from CrouchSprint 30 the edge is not a clean one: it keeps the filter, so there is a pending change for the Dodge to keep ageing)
        const auto up = run(c, cat(cat(rep(3, 30), rep(30, 6)), cat(rep(7, 12), rep(1, 30))));
        const int stood = firstStance(up, 0, 30);
        Check(stood >= 30 && stood <= 30 + 9 && stood < 30 + 6 + 12,
              "synthetic: Crouch -> CrouchSprint -> Dodge: Stand arrives 0.15 s after the PSM change, inside the dodge", stood - 30, 9);
        // CrouchSprint (stand) -> Crouch 0.1 s -> Dodge -> Crouch
        const auto down = run(c, cat(cat(rep(30, 30), rep(3, 6)), cat(rep(7, 12), rep(3, 30))));
        const int crouched = firstStance(down, 1, 30);
        Check(crouched >= 30 && crouched <= 30 + 9 && crouched < 30 + 6 + 12,
              "synthetic: CrouchSprint -> Crouch -> Dodge: Crouch arrives 0.15 s after the PSM change, inside the dodge", crouched - 30, 9);
        // the clean versions (P1): the edge is accepted at once and the Dodge that follows does not undo it
        const auto upClean = run(c, cat(cat(rep(3, 30), rep(1, 6)), cat(rep(7, 12), rep(1, 30))), 0.0f);
        bool standAfter = firstStance(upClean, 0, 30) == 30;
        for (size_t i = 30; i < upClean.size(); ++i)
            standAfter = standAfter && upClean[i].first == 0;
        Check(standAfter, "synthetic: Crouch -> Stand -> Dodge: the clean stand-up is accepted at once and the Dodge keeps it");
        const auto downClean = run(c, cat(cat(rep(1, 30), rep(3, 6)), cat(rep(7, 12), rep(3, 30))), 0.0f);
        bool crouchAfter = firstStance(downClean, 1, 30) == 30;
        for (size_t i = 30; i < downClean.size(); ++i)
            crouchAfter = crouchAfter && downClean[i].first == 1;
        Check(crouchAfter, "synthetic: Stand -> Crouch -> Dodge: the clean crouch is accepted at once and the Dodge keeps it");
    }
    // --- review: one old-value feed per 8 does not block a real change (periodic CrouchSprint with single Crouch feeds)
    {
        std::vector<std::pair<int, bool>> seq = rep(3, 30);
        for (int i = 0; i < 6; ++i)
            seq = cat(cat(seq, rep(30, 7)), rep(3, 1));
        const auto o = run(c, seq);
        int stood = -1;
        for (size_t i = 30; i < o.size() && stood < 0; ++i)
            if (o[i].first == 0)
                stood = int(i);
        Check(stood >= 30 && stood <= 30 + 20, "synthetic: CrouchSprint with a single Crouch feed every 8th still becomes Stand (feeds)", stood - 30, 20);
        // ... and the mirrored spikes (1 x CrouchSprint per 8 Crouch feeds) stay filtered
        std::vector<std::pair<int, bool>> spikes = rep(3, 30);
        for (int i = 0; i < 6; ++i)
            spikes = cat(cat(spikes, rep(30, 1)), rep(3, 7));
        bool crouch = true;
        for (const auto& p : run(c, spikes))
            crouch = crouch && p.first == 1;
        Check(crouch, "synthetic: single CrouchSprint feeds every 8th stay filtered");
    }
    // --- review: the sprint gait of CrouchSprint follows the filtered stance (a spike inside the crouch is neither Stand nor Sprint)
    {
        adapter::MapState m;
        adapter::MapContext cx;
        int sprintOnSpike = 0, sprintAfter = 0;
        for (int i = 0; i < 40; ++i)
        {
            const bool spike = i == 20;
            const motion::ChooserState cs = adapter::MapToChooser(SynthFeed(dt, spike ? 30 : 3, true), c, m, cx);
            sprintOnSpike += spike && cs.gait == 2;
        }
        for (int i = 0; i < 20; ++i)
            sprintAfter += adapter::MapToChooser(SynthFeed(dt, 30, true), c, m, cx).gait == 2;
        Check(sprintOnSpike == 0, "synthetic: one CrouchSprint feed inside the crouch does not set the Sprint gait", sprintOnSpike);
        Check(sprintAfter >= 8, "synthetic: a sustained CrouchSprint becomes Stand + Sprint after the filter time", sprintAfter);
    }
    // --- review 4: ChooserState::stanceRaw is the undebounced PSM stance while stance is held by the filter
    {
        adapter::MapState m;
        adapter::MapContext cx;
        int bad = 0, held = 0;
        for (int i = 0; i < 40; ++i)
        {
            const bool spike = i == 20 || i == 30; // single CrouchSprint 30 feeds inside the crouch
            const motion::ChooserState cs = adapter::MapToChooser(SynthFeed(dt, spike ? 30 : 3, true), c, m, cx);
            bad += cs.stanceRaw != (spike ? 0 : 1);
            held += spike && cs.stance == 1;
        }
        Check(bad == 0 && held == 2, "synthetic: stanceRaw follows the PSM on a CrouchSprint spike while stance is held at Crouch", bad, held);
    }
    // --- review 3: a one-feed Crouch 3 on the first feed after the slide is not held by the filter, Sprint 4 follows at once
    {
        const auto o = run(c, cat(cat(rep(5, 20), rep(3, 1)), rep(4, 30)));
        Check(o[20].first == 1 && o[21].first == 0 && o[22].first == 0, "synthetic: slide -> one Crouch feed -> Sprint: the Stand stance arrives on the Sprint feed",
              o[21].first, o[22].first);
        // ... but a Crouch exit stays a crouch, and a confirmed Crouch still debounces a later one-feed Sprint
        const auto o2 = run(c, cat(cat(rep(5, 20), rep(3, 30)), cat(rep(4, 1), rep(3, 30))));
        bool crouch = true;
        for (size_t i = 20; i < o2.size(); ++i)
            crouch = crouch && o2[i].first == 1;
        Check(crouch, "synthetic: a Crouch exit stays Crouch; a one-feed Sprint inside the confirmed crouch is filtered");
        adapter::MapState m;
        adapter::MapContext cx;
        int sprint = 0;
        for (int i = 0; i < 30; ++i)
            adapter::MapToChooser(SynthFeed(dt, 3, true), c, m, cx);
        sprint += adapter::MapToChooser(SynthFeed(dt, 4, true), c, m, cx).gait == 2;
        Check(sprint == 0, "synthetic: one Sprint 4 feed inside the filtered crouch does not set the Sprint gait", sprint);
    }
    // --- slide frames and the first frame after the slide are not filtered (the exit database is chosen by that stance)
    {
        const auto on = run(c, cat(cat(rep(4, 20), rep(5, 20)), rep(1, 6)));
        Check(on[40].first == 0 && on[41].first == 0, "synthetic: first frame after a slide into Stand has the Stand stance", on[40].first);
        const auto on2 = run(c, cat(cat(rep(4, 20), rep(5, 20)), rep(3, 6)));
        Check(on2[40].first == 1, "synthetic: first frame after a slide into the crouch has the Crouch stance", on2[40].first);
    }
    // --- slide air grace
    {
        // PSM keeps Slide 5 for 6 feeds (0.1 s) with the capsule off the ground: still sliding, no InAir
        const auto g = run(c, cat(cat(rep(5, 20), rep(5, 6, false)), rep(5, 20)));
        bool noAir = true;
        for (const auto& p : g)
            noAir = noAir && p.second != 1;
        Check(noAir, "synthetic: a 0.1 s !onGround drop at PSM Slide 5 stays ground");
        adapter::MapConfig off = c;
        off.slideAirGraceS = 0.0f;
        const auto o = run(off, cat(cat(rep(5, 20), rep(5, 6, false)), rep(5, 20)));
        Check(o[22].second == 1, "synthetic (mutation): without the grace the drop is InAir", o[22].second);
        // a drop that outlasts the grace is a real fall
        const auto longDrop = run(c, cat(cat(rep(5, 20), rep(5, 30, false)), rep(5, 5)));
        Check(longDrop[20 + 29].second == 1, "synthetic: a !onGround drop longer than the grace becomes InAir", longDrop[49].second);
        // the host mirrors the grace to its v2 consumers (head lead / lean / orient warp read f.air): airGraceActive per feed
        {
            adapter::MapState m;
            int graceFeeds = 0, graceAfter = 0;
            for (int i = 0; i < 20; ++i)
                adapter::MapToChooser(SynthFeed(dt, 5, true), c, m, ctx);
            for (int i = 0; i < 40; ++i)
            {
                adapter::MapToChooser(SynthFeed(dt, 5, false, -1.5f), c, m, ctx);
                if (i < 6)
                    graceFeeds += m.airGraceActive;
                else if (i >= 20)
                    graceAfter += m.airGraceActive;
            }
            Check(graceFeeds == 6 && graceAfter == 0, "synthetic: airGraceActive on the first feeds of a drop only, off after the grace", graceFeeds, graceAfter);
        }
        // the PSM saying Jump 18 is not delayed
        const auto jump = run(c, cat(rep(5, 20), rep(18, 3, false)));
        Check(jump[20].second == 1, "synthetic: PSM Jump 18 out of a slide is InAir at once", jump[20].second);
        // grace right after the slide (PSM already Crouch 3) too, then not for an ordinary crouch drop later
        const auto after = run(c, cat(cat(rep(5, 20), rep(3, 3)), rep(3, 6, false)));
        bool noAirAfter = true;
        for (const auto& p : after)
            noAirAfter = noAirAfter && p.second != 1;
        Check(noAirAfter, "synthetic: a short drop right after the slide (PSM Crouch 3) stays ground");
        const auto late = run(c, cat(cat(rep(5, 20), rep(3, 60)), rep(3, 6, false)));
        Check(late[20 + 60].second == 1, "synthetic: the same drop 1 s after the slide is InAir (no grace outside the slide context)",
              late[80].second);
    }
    // --- vehicle exit air grace (2026-10-24): a standing exit is not a jump-out
    {
        adapter::MapConfig e = c;
        e.exitAirGraceS = adapter::kExitAirGraceSeconds;
        adapter::MapContext ex;
        ex.vehicleExit = true;
        auto exitRun = [&](const adapter::MapConfig& aCfg, adapter::MapContext& aCtx, int aDetail, bool aGround, float aVz, int aN) {
            adapter::MapState m;
            int air = 0;
            for (int i = 0; i < aN; ++i)
                air += adapter::MapToChooser(SynthFeed(dt, aDetail, aGround, aVz, 0.0f), aCfg, m, aCtx).movementMode == 1;
            return air;
        };
        // 0.1 s !onGround / PSM Fall right after the unmount, not rising: stays ground
        Check(exitRun(e, ex, 3, false, -1.0f, 6) == 0, "exit grace: a short non-rising !onGround feed after the exit stays ground");
        Check(exitRun(e, ex, 14, true, -1.0f, 6) == 0, "exit grace: a short PSM Fall after the exit stays ground");
        // a real jump-out (PSM Jump 18, or rising fast) is not delayed
        Check(exitRun(e, ex, 18, false, 3.0f, 3) >= 1, "exit grace: PSM Jump 18 inside the exit window is InAir at once");
        Check(exitRun(e, ex, 3, false, 3.0f, 3) >= 1, "exit grace: a fast rise inside the exit window is InAir at once");
        // a fall that outlasts the grace is real
        Check(exitRun(e, ex, 3, false, -1.0f, 40) > 0, "exit grace: a !onGround feed longer than the grace becomes InAir");
        // outside the window, or with the grace off, nothing changes
        adapter::MapContext none;
        Check(exitRun(e, none, 3, false, -1.0f, 6) > 0, "exit grace: no vehicle exit window -> InAir as before");
        Check(exitRun(c, ex, 3, false, -1.0f, 6) > 0, "exit grace (mutation): grace off -> InAir as before");
        // 2026-10-28 bike jump-out (recording 01-22-53, feed rows without a LocomotionDetailed field, loco 0, no probe hit): the driver
        // now opens the exit window at the launch (MMDriver JumpOutLatch), so v2 sees the context from vz 9.7 on - the whole ballistic
        // flight (16 m/s^2, 1.33 s to the touchdown) is InAir. Contrast: the window opening at the apex (pre-fix: the vehicle bit fell
        // at vz -2.48) graces the first falling feeds as ground (recorded MOTION air=mode0 Stand_Idle_Loop in mid-air).
        auto flight = [&](float aVz0, int aN, int& aFirstGround) {
            adapter::MapState m;
            aFirstGround = -1;
            int air = 0;
            for (int i = 0; i < aN; ++i)
            {
                const float vel[3] = {0.0f, 0.0f, aVz0 - 16.0f * dt * float(i)};
                const float mv[2] = {0.0f, 0.0f};
                adapter::AirFeed af;
                af.valid = true;
                af.onGround = false;
                af.distanceToGround = -1.0f;
                const adapter::MotionFeed f = adapter::MakeMotionFeed(vel, mv, dt, 0u, 0, af, -8.4f, -8.4f);
                if (adapter::MapToChooser(f, e, m, ex).movementMode == 1)
                    ++air;
                else if (aFirstGround < 0)
                    aFirstGround = i;
            }
            return air;
        };
        int launchGround = 0, apexGround = 0;
        const int launchAir = flight(9.70f, 80, launchGround);
        const int apexAir = flight(-2.48f, 30, apexGround);
        Check(launchAir == 80 && launchGround < 0, "exit grace: bike jump-out with the window from the launch - InAir on every flight feed", launchAir,
              launchGround);
        Check(apexAir < 30 && apexGround == 0, "exit grace: the window opening at the apex (pre-fix) graces the first falling feeds as ground",
              apexAir, apexGround);
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_slide_exit_test <slide_exit_feeds.txt> <bundle path without extension> [--report] [--csv out.csv] "
                    "[--set <knob> <value>]...\n");
        return 2;
    }
    std::string csvPath;
    Variant prod;
    for (int i = 3; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--report"))
            g_report = true;
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc)
            csvPath = argv[++i];
        else if (!std::strcmp(argv[i], "--set") && i + 2 < argc)
        {
            if (!SetKnob(prod, argv[i + 1], std::strtof(argv[i + 2], nullptr)))
            {
                std::printf("FAIL: unknown knob %s\n", argv[i + 1]);
                return 2;
            }
            i += 2;
        }
    }
    SyntheticChecks();
    std::vector<Section> secs;
    if (!LoadFixture(argv[1], secs))
    {
        std::printf("FAIL: fixture %s\n", argv[1]);
        return 1;
    }
    auto model = std::make_unique<Model>();
    std::string err;
    if (!model->Load(argv[2], err))
    {
        std::printf("FAIL: bundle %s: %s\n", argv[2], err.c_str());
        return 1;
    }
    std::printf("bundle %s: %u databases, %u clips; %zu sections\n", argv[2], model->db.NumDatabases(), model->db.NumClips(), secs.size());
    FILE* csv = nullptr;
#ifdef _MSC_VER
    if (!csvPath.empty() && fopen_s(&csv, csvPath.c_str(), "w") != 0)
        csv = nullptr;
#else
    csv = csvPath.empty() ? nullptr : std::fopen(csvPath.c_str(), "w");
#endif
    if (csv)
        std::fprintf(csv, "section,t,detail,stanceRaw,stance,mode,modeLast,db,clip,jumped,blend,cost,contCost,clipT,rotHips,rotSpine3,rate,gait,moveState\n");

    // production configuration
    const Summary base = RunAll(*model, secs, prod, csv, true);
    CheckVerdict("prod", Judge(base, prod), base);
    if (csv)
        std::fclose(csv);

    // slides cut 0.05 s after a mid-slide re-jump of the Into clip (a long slide re-jumped to Into at 0.93 / 1.03 s every ~1 s; the real
    // feeds hold no exit at such a moment, the replay makes one: the PSM leaves the slide 0.05 s after the re-jump, the capsule does not
    // care). The exit must still be a settled one: the fixes act, the same bounds hold. RC3 (MatcherConfig::slideIntoHandover) replaced the
    // re-jump by the handover to the Loop clip of the same family: the cut follows that pick the same way.
    {
        std::vector<Section> cuts;
        for (const Section& sec : secs)
        {
            const std::vector<Frame> fr = Replay(*model, sec, prod);
            double slideT0 = 0;
            for (const double offset : {0.0167, 0.05})
            {
            int n = 0;
            for (size_t i = 1; i < fr.size() && n < 2; ++i)
            {
                if (fr[i].mode != motion::kMovementSliding)
                    continue;
                if (fr[i - 1].mode != motion::kMovementSliding)
                    slideT0 = fr[i].t;
                const bool sameFamily = (fr[i].clip.find("FootOut") != std::string::npos) == (fr[i - 1].clip.find("FootOut") != std::string::npos);
                if (!fr[i].jumped || fr[i - 1].mode != motion::kMovementSliding ||
                    (fr[i].clip.find("_Into_") == std::string::npos && fr[i].clip.find("_Loop") == std::string::npos) ||
                    fr[i].t - slideT0 < 0.5 || !sameFamily) // a FootOut <-> KneesOut switch is a slide to slide blend, not an exit
                    continue;
                Section c = sec;
                c.name = sec.name + "-cut" + std::to_string(offset > 0.03 ? 1 : 0) + std::to_string(n++);
                const double cutT = fr[i].t + offset;
                bool slideRows = true;
                for (Row& r : c.rows)
                    if (r.t >= cutT && slideRows && adapter::DetailFromFeed(r.psm, r.loco) == adapter::kPsmSlide)
                        r.psm = (r.psm & ~integ::kPsmDetailFieldMask) | ((uint32_t(adapter::kPsmCrouch) + 1u) << integ::kPsmDetailShift);
                    else if (r.t >= cutT)
                        slideRows = false;
                cuts.push_back(std::move(c));
            }
            }
        }
        const Summary sc = RunAll(*model, cuts, prod, nullptr, g_report);
        const Verdict jc = Judge(sc, prod);
        Check(jc.rotationExits >= 6, "re-jump cut: >= 6 settled ground exits 1 and 3 feeds after an Into re-jump / Loop handover", jc.rotationExits);
        Check(jc.rotation, "re-jump cut: Hips / Spine3 rotation per 0.1 s below the bound (worst deg)", jc.worstRot, kMaxRotDeg);
        Check(jc.entry, "re-jump cut: exit clip entered at its start (worst clip time s)", jc.worstEntry, kMaxEntryClipTimeS);
    }

    // fidelity gaps of the feed dump: native capsule stats and the hip-fire strafe. The bounds must hold for them too.
    {
        Variant brake = prod;
        brake.name = "native-braking";
        SetKnob(brake, "braking", 24.0f);
        const Summary s = RunAll(*model, secs, brake, nullptr, g_report);
        CheckVerdict("native braking 24 m/s^2", Judge(s, brake), s);
        Variant accel = prod;
        accel.name = "native-accel";
        SetKnob(accel, "accel", 3.91f);
        SetKnob(accel, "braking", 24.0f);
        const Summary sa = RunAll(*model, secs, accel, nullptr, g_report);
        CheckVerdict("native braking + acceleration", Judge(sa, accel), sa);
        Variant strafe = prod;
        strafe.name = "visual-strafe";
        SetKnob(strafe, "visualStrafe", 1.0f);
        const Summary ss = RunAll(*model, secs, strafe, nullptr, g_report);
        CheckVerdict("visualStrafe", Judge(ss, strafe), ss);
        // KneesOut slides everywhere: the body turns 60 deg off the capsule velocity during every slide (Direction Other)
        Variant knees = prod;
        knees.name = "knees-out";
        knees.yawShiftDeg = 60.0f;
        const Summary sk = RunAll(*model, secs, knees, nullptr, g_report);
        const Verdict jk = Judge(sk, knees);
        CheckVerdict("KneesOut slides (yaw +60 during the slide)", jk, sk);
        Check(jk.kneesExits >= 15, "KneesOut variant: the slides are KneesOut slides (exits with a KneesOut_Out clip, >= 15)", jk.kneesExits);
        // RMB held from the slide end: the stopped exits (Idle) have stationaryAimTurn on every feed, which is no START of an aim turn
        Variant aim = prod;
        aim.name = "aim-held";
        aim.aimMode = 1;
        const Summary sa1 = RunAll(*model, secs, aim, nullptr, g_report);
        const Verdict ja = Judge(sa1, aim);
        Check(ja.hold && ja.yields && ja.rotationExits >= 20, "RMB held after the slide: the exit hold is not released by the aim level (stopped exits included)",
              ja.shortestHold, ja.rotationExits);
        // a shot turn starting 0.3 s after the slide end releases the stopped exits at once
        Variant shot = prod;
        shot.name = "shot-turn";
        shot.aimMode = 2;
        const Summary ss2 = RunAll(*model, secs, shot, nullptr, g_report);
        const Verdict jsh = Judge(ss2, shot);
        Check(jsh.hold && jsh.yields && jsh.flagReleases >= 2, "a shot turn start releases the exit hold at once (exits released by it, >= 2)", jsh.flagReleases);
        // reversal of the stick and the capsule 0.1 s after the slide: pivot / spin / reface starts release the hold
        Variant rev = prod;
        rev.name = "reverse";
        rev.reverseAfterS = 0.1f;
        const Summary sr = RunAll(*model, secs, rev, nullptr, g_report);
        const Verdict jr = Judge(sr, rev);
        Check(jr.hold && jr.yields && jr.flagReleases >= 3, "a reversal after the slide releases the exit hold at once (exits released by it, >= 3)", jr.flagReleases);
        // review 4: a flag set when the clip is selected (pivot on the first exit feeds), dropped after 0.1 s and set again at 0.3 s
        // releases the hold on the second start
        Variant repiv = prod;
        repiv.name = "pivot-again";
        repiv.reverseFirstS = 0.1f;
        repiv.reverseAfterS = 0.3f;
        const Summary sp = RunAll(*model, secs, repiv, nullptr, g_report);
        const Verdict jp = Judge(sp, repiv);
        Check(jp.hold && jp.yields && jp.flagsAtSelect >= 3 && jp.flagReleases >= 3,
              "a flag set at the selection, dropped and set again releases the exit hold (exits with a flag at the selection >= 3, released >= 3)",
              jp.flagsAtSelect, jp.flagReleases);
        // slideExitYieldS = 0: a gait / movementState DIFFERENCE releases at once, no difference never does
        Variant y0 = prod;
        y0.name = "yield-0";
        SetKnob(y0, "slideExitYieldS", 0.0f);
        const Summary sy = RunAll(*model, secs, y0, nullptr, g_report);
        const Verdict jy = Judge(sy, y0);
        Check(jy.hold && jy.yields && jy.rotationExits >= 20, "slideExitYieldS 0: the exit hold stays while gait / movementState do not differ", jy.shortestHold,
              jy.rotationExits);
    }

    // mutations: the same replay with one fix off must violate that fix's bound (the test is sensitive to the fix)
    struct Mut
    {
        const char* knob;
        float off;
        const char* what;
        bool Verdict::*bound;
    };
    const Mut muts[] = {
        {"slideExitFamily", 0.0f, "slideExitFamily off -> the other slide family's exit clip pops", &Verdict::rotation},
        {"slideExitMinHoldS", 0.0f, "slideExitMinHoldS off -> exit clip dropped early", &Verdict::hold},
        {"slideExitBiasS", 0.0f, "slideExitBiasS off -> moving exit clip held to its end", &Verdict::holdMax},
        {"slideExitMaxEntryS", 0.0f, "slideExitMaxEntryS off -> exit clip entered mid-clip", &Verdict::entry},
        {"slideExitMinPlayRate", 0.0f, "slideExitMinPlayRate off -> the rise runs at the 0.85 floor, Spine3 above the bound", &Verdict::rotation},
        {"blendSlideExit", -1.0f, "blendSlideExit off -> the 0.2 s cross-fade adds rate, Spine3 above the bound", &Verdict::rotation},
        {"slideAirGraceS", 0.0f, "slideAirGraceS off -> false InAir / Jump clip after a slide", &Verdict::noFalseAir},
        {"stanceFilterS", 0.0f, "stanceFilterS off -> stance flicker reaches the chooser", &Verdict::stance},
        {"stanceHoldInDodge", 0.0f, "stanceHoldInDodge off -> Dodge stands the body up", &Verdict::dodge},
    };
    for (const Mut& mu : muts)
    {
        Variant v = prod;
        v.name = mu.knob;
        SetKnob(v, mu.knob, mu.off);
        // RC3 (slideIntoHandover) lowers the exits of the long slides (Loop pose instead of a re-jumped Into): without blendSlideExit the
        // worst exit is 24.1 deg (x1015-0009-230; 26.9 with the re-jumps), so that mutation is judged on the re-jumping slide it was set on
        if (!std::strcmp(mu.knob, "blendSlideExit"))
            SetKnob(v, "slideIntoHandover", 0.0f);
        const Summary s = RunAll(*model, secs, v, nullptr, g_report);
        const Verdict j = Judge(s, v);
        Check(!(j.*mu.bound), mu.what, j.worstRot, j.worstEntry);
    }
    {
        Variant v = prod; // the pre-fix replay: 60-90 deg per 0.1 s, exit clip entered mid-clip, dropped after 0.1 s or held 3.5 s
        v.name = "all slide-exit fixes off";
        SetKnob(v, "slideExitFamily", 0.0f);
        SetKnob(v, "slideExitMinHoldS", 0.0f);
        SetKnob(v, "slideExitMaxEntryS", 0.0f);
        SetKnob(v, "slideExitBiasS", 0.0f);
        SetKnob(v, "slideExitMinPlayRate", 0.0f);
        SetKnob(v, "blendSlideExit", -1.0f);
        SetKnob(v, "slideAirGraceS", 0.0f);
        SetKnob(v, "stanceFilterS", 0.0f);
        SetKnob(v, "stanceHoldInDodge", 0.0f);
        const Summary s = RunAll(*model, secs, v, nullptr, g_report);
        const Verdict j = Judge(s, v);
        Check(!j.rotation, "all fixes off -> exit rotation bound violated (pre-fix replay: 60-90 deg per 0.1 s)", j.worstRot, kMaxRotDeg);
        Check(!j.holdMax, "all fixes off -> an exit clip is held far beyond its rise (pre-fix replay 3.47 s)", j.longestHold, kMaxExitHoldS);
        Check(!j.noFalseJump || !j.noFalseAir, "all fixes off -> false air / Jump clip after a slide", s.falseAirFrames, s.falseJumpFrames);
    }
    std::printf("slide exit: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

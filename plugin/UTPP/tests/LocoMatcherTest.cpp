// Locomotion matcher fixes (branch loco-matcher, recording 2026-10-31 22:55, the recorded log). The real game feed of the whole
// diag_b scenario (tests/data/loco_feeds_20261031_2255.txt) goes through the production stack minus the game: adapter::MapToChooser ->
// BuildQueryTrajectory -> ChooserMatcher::Update -> motionstack::BlendStack (the live outgoing-clip crossfade PoseEvaluator uses for the base pose) on
// the installed slide bundle (utpp_motion_woman_knee_twistfix_slide = the installed woman v2 database).
//
// Usage: utpp_mm_loco_matcher_test <loco_feeds.txt> <bundle path without extension> [--report] [--dump t0 t1] [--set <knob> <value>]...
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

#include "../src/MotionDriver.hpp" // AirFloorStep (the production floor distance of the air feed: probe hits dead-reckoned, take-off plane seed)
#include "../src/mm/MotionBlendStack.hpp"
#include "../src/mm/MotionAdapter.hpp"

namespace adapter = utpp::mm::adapter;
namespace motion = utpp::mm::motion;
namespace motionstack = utpp::mm::motionstack;
namespace math = utpp::mm::math;
namespace chooser = utpp::mm::chooser;
namespace steer = utpp::mm::steer;
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

float Num(const std::string& aTok, float aDef = 0.0f) { return aTok == "-" ? aDef : std::strtof(aTok.c_str(), nullptr); }

bool LoadFixture(const char* aPath, std::vector<Row>& aOut)
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
        if (line.empty() || line[0] == '#' || line[0] == '@')
            continue;
        if (cols.empty())
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
        aOut.push_back(r);
    }
    return !aOut.empty();
}

struct Variant
{
    adapter::MapConfig mc;
    motion::MatcherConfig cfg;
    Variant()
    {
        mc.slide = true;
        mc.slideDeceleration = adapter::kSlideDecelerationMs2; // MotionDriver.cpp StepLocked
        mc.slideAirGraceS = adapter::kSlideAirGraceSeconds;
        mc.exitAirGraceS = adapter::kExitAirGraceSeconds;
        mc.stanceFilterS = adapter::kStanceFilterSeconds;
        mc.stanceHoldInDodge = true;
        mc.stanceCleanEdge = true;
        mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
        mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    }
};

struct Frame
{
    double t = 0;
    int detail = 0, stance = 0, mode = 0, gait = 0, moveState = 0, onGround = -1;
    float speedCmS = 0, capsuleMs = 0;
    std::string clip, db;
    int clipIndex = -1;
    bool jumped = false;
    float blend = 0, cost = 0, clipTime = 0, rate = 1;
    float hipsZ = 0, headZ = 0, leanDeg = 0, kneeL = 0, kneeR = 0; // blended base pose, model space (m, deg)
    float armL = 0, armR = 0; // upper arm (Arm -> ForeArm) angle from straight down, model space (deg)
};

struct Model
{
    motion::MotionDatabase db;
    chooser::Table table;
    adapter::SlideSpec slide;
    adapter::QueryTimes qtimes;
    std::vector<float> factors;
    int boneArmL = -1, boneArmR = -1, boneForeArmL = -1, boneForeArmR = -1;
    int boneHips = -1, boneHead = -1, boneUpLegL = -1, boneUpLegR = -1, boneLegL = -1, boneLegR = -1, boneFootL = -1, boneFootR = -1;
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
            auto set = [&](const char* n, int& o) { if (name == n) o = int(b); };
            set("Hips", boneHips);
            set("LeftArm", boneArmL);
            set("RightArm", boneArmR);
            set("LeftForeArm", boneForeArmL);
            set("RightForeArm", boneForeArmR);
            set("Head", boneHead);
            set("LeftUpLeg", boneUpLegL);
            set("RightUpLeg", boneUpLegR);
            set("LeftLeg", boneLegL);
            set("RightLeg", boneLegR);
            set("LeftFoot", boneFootL);
            set("RightFoot", boneFootR);
        }
        return boneArmL >= 0 && boneArmR >= 0 && boneForeArmL >= 0 && boneForeArmR >= 0 && boneHips >= 0 && boneHead >= 0 && boneUpLegL >= 0 && boneUpLegR >= 0 && boneLegL >= 0 && boneLegR >= 0 && boneFootL >= 0 &&
               boneFootR >= 0;
    }
    int ClipIndex(const char* aName) const
    {
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (db.ClipName(c) == aName)
                return int(c);
        return -1;
    }
};

math::Vec3 ModelPos(const motion::MotionDatabase& aDb, const BoneTransform* aPose, int aBone)
{
    std::vector<int> chain;
    for (int b = aBone; b >= 0; b = aDb.BoneParent(uint32_t(b)))
        chain.push_back(b);
    math::Quat q{0, 0, 0, 1};
    math::Vec3 p{0, 0, 0};
    for (size_t i = chain.size(); i-- > 0;)
    {
        const BoneTransform& t = aPose[chain[i]];
        p = p + math::QuatRotate(q, t.p);
        q = math::QuatNormalize(math::QuatMul(q, t.q));
    }
    return p;
}

float KneeFlexDeg(const math::Vec3& aUp, const math::Vec3& aKnee, const math::Vec3& aFoot)
{
    const math::Vec3 a = aUp - aKnee, b = aFoot - aKnee;
    const float la = std::sqrt(math::Dot(a, a)), lb = std::sqrt(math::Dot(b, b));
    if (la < 1e-6f || lb < 1e-6f)
        return 0.0f;
    const float c = std::fmax(-1.0f, std::fmin(1.0f, math::Dot(a, b) / (la * lb)));
    return 180.0f - std::acos(c) * 57.29577951f;
}

std::vector<Frame> Replay(Model& m, const std::vector<Row>& aRows, const Variant& aVar)
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
    utpp::motiondriver::AirFloorState floor;
    for (const Row& r : aRows)
    {
        const float dt = std::fmin(r.dt > 0 ? r.dt : 0.0f, 0.25f);
        adapter::AirFeed air;
        air.valid = r.onGround >= 0;
        air.onGround = r.onGround != 0;
        air.distanceToGround = r.dist;
        air.psmFall = r.fall;
        if (air.valid)
            air.distanceToGround = utpp::motiondriver::AirFloorStep(floor, air.onGround, r.dist, adapter::DetailFromFeed(r.psm, r.loco),
                                                              std::isfinite(r.vel[2]) ? r.vel[2] : 0.0f, dt).distance;
        adapter::MotionFeed f = adapter::MakeMotionFeed(r.vel, r.mv, dt, r.psm, r.loco, air, r.cam, r.yaw);
        f.wantsToWalk = r.walk != 0;
        f.haveTarget = r.haveTarget && std::isfinite(r.target);
        f.targetYawDeg = f.haveTarget ? r.target : 0.0f;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = r.yaw;
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
        fr.onGround = r.onGround;
        fr.stance = cs.stance;
        fr.mode = cs.movementMode;
        fr.gait = cs.gait;
        fr.moveState = cs.movementState;
        fr.speedCmS = cs.speed2dCmS;
        fr.capsuleMs = std::hypot(r.vel[0], r.vel[1]);
        fr.clip = std::string(m.db.ClipName(uint32_t(res.clip)));
        fr.clipIndex = res.clip;
        fr.db = std::string(m.db.DbName(uint32_t(res.database)));
        fr.jumped = res.jumped;
        fr.blend = res.jumped ? res.blend.blendTime : 0.0f;
        fr.cost = res.cost;
        fr.clipTime = res.clipTime;
        fr.rate = res.playRate;
        const BoneTransform* p = stack.Output();
        const math::Vec3 hips = ModelPos(m.db, p, m.boneHips), head = ModelPos(m.db, p, m.boneHead);
        fr.hipsZ = hips.z;
        fr.headZ = head.z;
        const math::Vec3 up = head - hips;
        const float lup = std::sqrt(math::Dot(up, up));
        fr.leanDeg = lup > 1e-6f ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, up.z / lup))) * 57.29577951f : 0.0f;
        fr.kneeL = KneeFlexDeg(ModelPos(m.db, p, m.boneUpLegL), ModelPos(m.db, p, m.boneLegL), ModelPos(m.db, p, m.boneFootL));
        fr.kneeR = KneeFlexDeg(ModelPos(m.db, p, m.boneUpLegR), ModelPos(m.db, p, m.boneLegR), ModelPos(m.db, p, m.boneFootR));
        auto down = [&](int aArm, int aFore) {
            const math::Vec3 a = ModelPos(m.db, p, aFore) - ModelPos(m.db, p, aArm);
            const float l = std::sqrt(math::Dot(a, a));
            return l > 1e-6f ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, -a.z / l))) * 57.29577951f : 0.0f;
        };
        fr.armL = down(m.boneArmL, m.boneForeArmL);
        fr.armR = down(m.boneArmR, m.boneForeArmR);
        out.push_back(std::move(fr));
    }
    return out;
}

void Dump(const std::vector<Frame>& aFr, double t0, double t1)
{
    std::printf("t        det st md gt ms  spd   cap  %-44s ct     rate  j blend  cost    hipsZ headZ lean  kneeL kneeR\n", "clip");
    for (const Frame& f : aFr)
        if (f.t >= t0 && f.t <= t1)
            std::printf("%.3f %3d %2d %2d %2d %2d %5.0f %5.2f %-44s %6.3f %5.3f %d %5.2f %7.3f %5.3f %5.3f %5.1f %5.1f %5.1f %5.1f %5.1f\n", f.t, f.detail, f.stance,
                        f.mode, f.gait, f.moveState, double(f.speedCmS), double(f.capsuleMs), f.clip.c_str(), double(f.clipTime), double(f.rate),
                        f.jumped ? 1 : 0, double(f.blend), double(f.cost), double(f.hipsZ), double(f.headZ), double(f.leanDeg), double(f.kneeL),
                        double(f.kneeR), double(f.armL), double(f.armR));
}

// the stored pose of a database frame
std::vector<BoneTransform> FramePose(const Model& m, uint32_t aGlobalFrame)
{
    const motion::PoseRecord* pr = m.db.Pose(aGlobalFrame);
    std::vector<BoneTransform> p(m.db.NumBones());
    for (uint32_t b = 0; b < m.db.NumBones(); ++b)
    {
        p[b].p = math::Vec3{pr[b].px, pr[b].py, pr[b].pz};
        p[b].q = math::Quat{pr[b].qx, pr[b].qy, pr[b].qz, pr[b].qw};
    }
    return p;
}
// Hips / Head height (m) of a database frame (model space, FK of the stored poses)
float FrameZ(const Model& m, uint32_t aGlobalFrame, int aBone)
{
    const auto p = FramePose(m, aGlobalFrame);
    return ModelPos(m.db, p.data(), aBone).z;
}
float FrameKnee(const Model& m, uint32_t aGlobalFrame, bool aRight)
{
    const auto p = FramePose(m, aGlobalFrame);
    return aRight ? KneeFlexDeg(ModelPos(m.db, p.data(), m.boneUpLegR), ModelPos(m.db, p.data(), m.boneLegR), ModelPos(m.db, p.data(), m.boneFootR))
                  : KneeFlexDeg(ModelPos(m.db, p.data(), m.boneUpLegL), ModelPos(m.db, p.data(), m.boneLegL), ModelPos(m.db, p.data(), m.boneFootL));
}

void ClipInfo(const Model& m, const char* aName)
{
    if (!std::strncmp(aName, "PSD_", 4)) // a database: its clips
    {
        for (uint32_t d = 0; d < m.db.NumDatabases(); ++d)
            if (m.db.DbName(d) == aName)
                for (uint32_t e = m.db.Db(d).firstEntry; e < m.db.Db(d).firstEntry + m.db.Db(d).numEntries; ++e)
                    std::printf("db %u entry %u clip %u %s rows %u flags 0x%x\n", d, e, m.db.Entry(e).clip,
                                std::string(m.db.ClipName(m.db.Entry(e).clip)).c_str(), m.db.Entry(e).numRows, m.db.Entry(e).flags);
        return;
    }
    const int c = m.ClipIndex(aName);
    if (c < 0)
    {
        std::printf("clip %s not found\n", aName);
        return;
    }
    const motion::MotionClipRecord& cl = m.db.Clip(uint32_t(c));
    const float fps = m.db.Header().fps;
    std::printf("clip %d %s: %u frames (%.3f s) flags 0x%x\n", c, aName, cl.numFrames, double(float(cl.numFrames - 1) / fps), cl.flags);
    for (uint32_t d = 0; d < m.db.NumDatabases(); ++d)
    {
        const auto& D = m.db.Db(d);
        for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
        {
            const auto& en = m.db.Entry(e);
            if (en.clip != uint32_t(c) || en.numRows == 0)
                continue;
            const auto& r0 = m.db.Row(en.firstRow);
            const auto& r1 = m.db.Row(en.firstRow + en.numRows - 1);
            std::printf("  db %u %s entry %u flags 0x%x rows %u: ct %.3f .. %.3f addend %.3f cont %.3f\n", d, std::string(m.db.DbName(d)).c_str(), e,
                        en.flags, en.numRows, double(float(r0.frame - cl.firstFrame) / fps), double(float(r1.frame - cl.firstFrame) / fps),
                        double(r0.costAddend), double(r0.continuingAddend));
        }
    }
    for (uint32_t f = 0; f < cl.numFrames; ++f)
        std::printf("  ct %.3f hips %.3f head %.3f kneeL %.1f kneeR %.1f\n", double(float(f) / fps), double(FrameZ(m, cl.firstFrame + f, m.boneHips)),
                    double(FrameZ(m, cl.firstFrame + f, m.boneHead)), double(FrameKnee(m, cl.firstFrame + f, false)),
                    double(FrameKnee(m, cl.firstFrame + f, true)));
}

bool SetKnob(Variant& v, const std::string& aName, float aValue)
{
    if (aName == "blendGroundToGround")
        v.cfg.blendGroundToGround = aValue;
    else if (aName == "blendLanding")
        v.cfg.blendLanding = aValue;
    else if (aName == "blendSlide")
        v.cfg.blendSlide = aValue;
    else if (aName == "slideEntryMaxS")
        v.cfg.slideEntryMaxS = aValue;
    else if (aName == "slideIntoHandover")
        v.cfg.slideIntoHandover = aValue != 0.0f;
    else if (aName == "slowCrouchLoops")
        v.cfg.slowCrouchLoops = aValue != 0.0f;
    else if (aName == "slowWalkStartTail")
        v.cfg.slowWalkStartTail = aValue != 0.0f;
    else
        return false;
    return true;
}

bool Has(const std::string& aClip, const char* aPart) { return aClip.find(aPart) != std::string::npos; }
bool IsSlideInto(const std::string& c) { return Has(c, "Slide_FootOut_Into_") || Has(c, "Slide_KneesOut_Into_"); }
bool IsSlideLoop(const std::string& c) { return Has(c, "Slide_FootOut_Loop") || Has(c, "Slide_KneesOut_Loop"); }
int SlideFamily(const std::string& c) { return Has(c, "Slide_FootOut_") ? 1 : (Has(c, "Slide_KneesOut_") ? 2 : 0); }

// index of the frame at or after aT (aFr.size() if none)
size_t At(const std::vector<Frame>& aFr, double aT)
{
    size_t i = 0;
    while (i < aFr.size() && aFr[i].t < aT)
        ++i;
    return i;
}
// largest change of aGet per 50 ms (frame i -> first frame j with t_j >= t_i + 0.05, scaled by 0.05 / (t_j - t_i): the feed rows come at
// 30-60 Hz) for frames i in [i0, i1)
template <class F> float MaxStep50(const std::vector<Frame>& aFr, size_t i0, size_t i1, F aGet)
{
    float m = 0.0f;
    for (size_t i = i0; i < i1 && i < aFr.size(); ++i)
    {
        size_t j = i;
        while (j < aFr.size() && aFr[j].t < aFr[i].t + 0.05 - 1e-6)
            ++j;
        if (j < aFr.size())
            m = std::fmax(m, float(std::fabs(aGet(aFr[j]) - aGet(aFr[i])) * 0.05 / std::fmax(0.05, aFr[j].t - aFr[i].t)));
    }
    return m;
}
// times of the frames where two replays differ (clip, clip time, rate, new selection)
std::vector<double> Diff(const std::vector<Frame>& a, const std::vector<Frame>& b)
{
    std::vector<double> out;
    if (a.size() != b.size())
    {
        out.push_back(-1.0);
        return out;
    }
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i].clipIndex != b[i].clipIndex || a[i].clipTime != b[i].clipTime || a[i].rate != b[i].rate || a[i].jumped != b[i].jumped)
            out.push_back(a[i].t);
    return out;
}
bool AllWithin(const std::vector<double>& aT, std::initializer_list<std::pair<double, double>> aWin)
{
    for (const double t : aT)
    {
        bool in = false;
        for (const auto& w : aWin)
            in = in || (t >= w.first && t <= w.second);
        if (!in)
            return false;
    }
    return true;
}
void PrintDiff(const char* aName, const std::vector<double>& aT)
{
    std::printf("%s: %zu differing frames", aName, aT.size());
    double s = -1.0, p = -1.0;
    for (const double t : aT)
    {
        if (s < 0.0)
            s = t;
        else if (t - p > 0.1)
        {
            std::printf(" [%.3f..%.3f]", s, p);
            s = t;
        }
        p = t;
    }
    if (s >= 0.0)
        std::printf(" [%.3f..%.3f]", s, p);
    std::printf("\n");
}

// ---- feed windows (feed t; wall clock = t + 22:55:27.96, plan_20261031-225624.json)
constexpr double kCwalkSlow0 = 61.5, kCwalkSlow1 = 63.8;   // cwalk_slow, steady Crouch_Loop_F at 0.58 m/s
constexpr double kCwalkFull0 = 64.6, kCwalkFull1 = 66.8;   // cwalk_full, 2.7 m/s (gait Run)
constexpr double kSlide0 = 83.0, kSlide1 = 85.5;           // sprint_crouch: the slide (det 5 from 83.147, exit at 85.39)

// CR-3: the slow crouch walk plays the crouch loop at the crouch walk rate floor; the full crouch walk keeps the ordinary bound
void CrouchRateChecks(const std::vector<Frame>& aFr, const std::vector<Frame>& aOff)
{
    float lo = 9.f, hi = 0.f;
    bool loops = true;
    for (size_t i = At(aFr, kCwalkSlow0); i < At(aFr, kCwalkSlow1); ++i)
    {
        loops = loops && aFr[i].db == "PSD_Dense_Crouch_Walk_Loops" && Has(aFr[i].clip, "Crouch_Loop_");
        lo = std::fmin(lo, aFr[i].rate);
        hi = std::fmax(hi, aFr[i].rate);
    }
    Check(loops, "CR-3 slow crouch walk 61.5-63.8 s plays a Crouch_Loop_* of PSD_Dense_Crouch_Walk_Loops (min rate, max rate)", lo, hi);
    Check(hi <= 0.5f + 1e-3f && lo >= 0.5f - 1e-3f, "CR-3 slow crouch walk: crouch loop rate at the 0.5 floor, not 0.85 (min, max)", lo, hi);
    float flo = 9.f;
    for (size_t i = At(aFr, kCwalkFull0); i < At(aFr, kCwalkFull1); ++i)
        if (aFr[i].gait == 1 && Has(aFr[i].clip, "Crouch_Loop_"))
            flo = std::fmin(flo, aFr[i].rate);
    Check(flo >= 0.85f - 1e-4f && flo < 9.f, "CR-3 full crouch walk (gait Run) keeps the ordinary 0.85 bound (min rate)", flo);
    float olo = 9.f;
    for (size_t i = At(aOff, kCwalkSlow0); i < At(aOff, kCwalkSlow1); ++i)
        olo = std::fmin(olo, aOff[i].rate);
    Check(olo >= 0.85f - 1e-4f, "CR-3 negative control (slowCrouchLoops off): the slow crouch walk is clamped at 0.85 (min rate)", olo);
    // the change is confined to the crouch segments: crouch_on .. crouch_off (59-69.3), run_crouch .. crouch_off2 (73.7-78.8), the slide + crouch
    // stop .. crouch_off3 (83-87.8); the stand run / sprint / jumps / walk before and after are bit-identical
    const auto d = Diff(aFr, aOff);
    PrintDiff("CR-3 slowCrouchLoops on vs off", d);
    Check(!d.empty() && AllWithin(d, {{59.0, 69.6}, {73.7, 79.1}, {83.0, 88.1}}), "CR-3 identity: on / off differ only in the crouch segments (differing frames)",
          double(d.size()));
}

// ST-1 (MatcherConfig::slowWalkStartTail): the two slow starts from rest of the feed (cwalk_slow 60.99, crouch stick 49 cm/s; walk 98.49, stand
// stick 104 cm/s) pick a Start clip on their IsStarting update and play its acceleration (0.2 / 0.3 s clip time, ~0.25 / 0.35 s at the clamped
// 0.85), then hand over to the loop database the chooser returns, where the CR-3 / G1 floor 0.5 applies; without the rule the Start tail is held
// for its whole indexed range at 0.85 (crouch 3.2 s: the full cwalk_slow; walk 2.9 s: up to the stop at 101.46). Run / sprint starts (70.49,
// 79.99, 91.58) are untouched.
constexpr double kCwalkStart = 60.99, kWalkStart = 98.49;
constexpr double kWalkSlow0 = 99.0, kWalkSlow1 = 101.3; // walk, steady at 0.82 m/s up to Walk_Stop at 101.46
// seconds from the Start pick at aT0 (a new selection of a clip containing aStart) to the first frame playing a clip containing aLoop (99 = none)
double Handover(const std::vector<Frame>& aFr, double aT0, const char* aStart, const char* aLoop)
{
    size_t i = At(aFr, aT0 - 0.05);
    while (i < aFr.size() && aFr[i].t < aT0 + 0.05 && !(aFr[i].jumped && Has(aFr[i].clip, aStart)))
        ++i;
    if (i >= aFr.size() || aFr[i].t >= aT0 + 0.05)
        return 99.0;
    for (size_t j = i; j < aFr.size() && aFr[j].t < aT0 + 3.0; ++j)
        if (Has(aFr[j].clip, aLoop))
            return aFr[j].t - aFr[i].t;
    return 99.0;
}
void StartTailChecks(const std::vector<Frame>& aFr, const std::vector<Frame>& aOff)
{
    const double hc = Handover(aFr, kCwalkStart, "Crouch_Start_", "Crouch_Loop_");
    Check(hc <= 0.6, "ST-1 crouch start 60.99: a Crouch_Start_* plays its acceleration, a Crouch_Loop_* takes over within 0.6 s (s)", hc);
    const double hw = Handover(aFr, kWalkStart, "Walk_Start_", "Walk_Loop_");
    Check(hw <= 0.6, "ST-1 walk start 98.49: a Walk_Start_* plays its acceleration, a Walk_Loop_* takes over within 0.6 s (s)", hw);
    float lo = 9.f, hi = 0.f;
    bool loops = true;
    for (size_t i = At(aFr, kWalkSlow0); i < At(aFr, kWalkSlow1); ++i)
    {
        loops = loops && aFr[i].db == "PSD_Dense_Stand_Walk_Loops" && Has(aFr[i].clip, "Walk_Loop_");
        lo = std::fmin(lo, aFr[i].rate);
        hi = std::fmax(hi, aFr[i].rate);
    }
    Check(loops && hi <= 0.5f + 1e-3f && lo >= 0.5f - 1e-3f,
          "ST-1 slow stand walk 99.0-101.3 s plays a Walk_Loop_* of PSD_Dense_Stand_Walk_Loops at the G1 0.5 floor (min rate, max rate)", lo, hi);
    // negative control: the motion DoNotInterrupt behaviour holds the Start tail at the clamped 0.85 (the problem the rule removes)
    bool tail = true;
    for (size_t i = At(aOff, kWalkSlow0); i < At(aOff, kWalkSlow1); ++i)
        tail = tail && Has(aOff[i].clip, "Walk_Start_") && aOff[i].rate >= 0.85f - 1e-4f;
    Check(tail, "ST-1 negative control (slowWalkStartTail off): the slow stand walk holds the Walk_Start_* tail at 0.85");
    const auto d = Diff(aFr, aOff);
    PrintDiff("ST-1 slowWalkStartTail on vs off", d);
    Check(!d.empty() && AllWithin(d, {{60.9, 69.6}, {98.4, 101.6}}), "ST-1 identity: on / off differ only after the two slow walk starts (differing frames)",
          double(d.size()));
}

// CR-4: the stationary Crouch -> Stand edges of the feed (crouch_off 69.2, crouch_off2 78.7, crouch_off3 87.7). Where the plain-cost competition
// selects M_Neutral_Transition_Crouch_to_Stand (crouch_off3) it is kept through its rise; the other edges (Stand_Idle_Loop wins) are unchanged.
struct RiseStats
{
    int edges = 0, picked = 0, held = 0;
    float head = 0.f; // largest head height change per 50 ms in the first 0.8 s of a picked edge (m)
};
RiseStats Rises(const std::vector<Frame>& aFr, const char* aTag)
{
    RiseStats st;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (!(aFr[i - 1].stance == 1 && aFr[i].stance == 0 && aFr[i].mode == 0 && aFr[i].moveState == 0))
            continue;
        ++st.edges;
        const Frame& f = aFr[i];
        const bool pick = f.jumped && Has(f.clip, "Transition_Crouch_to_Stand");
        const size_t k = At(aFr, f.t + 0.75);
        bool hold = pick && k < aFr.size() && aFr[k].clipIndex == f.clipIndex;
        for (size_t j = i + 1; j <= k && j < aFr.size(); ++j)
            hold = hold && !aFr[j].jumped; // no new selection in between
        const size_t r1 = At(aFr, f.t + 0.8);
        const float kn = std::fmax(MaxStep50(aFr, i - 1, r1, [](const Frame& x) { return x.kneeL; }), MaxStep50(aFr, i - 1, r1, [](const Frame& x) { return x.kneeR; }));
        const float hp = MaxStep50(aFr, i - 1, r1, [](const Frame& x) { return x.hipsZ; });
        const float hd = MaxStep50(aFr, i - 1, r1, [](const Frame& x) { return x.headZ; });
        st.picked += pick;
        st.held += hold;
        if (pick)
            st.head = std::fmax(st.head, hd);
        std::printf("%s crouch -> stand edge t %.3f: %s ct %.3f cost %.3f; at +0.75 s %s; per 50 ms knee %.1f deg hips %.3f m head %.3f m\n", aTag, f.t,
                    f.clip.c_str(), double(f.clipTime), double(f.cost), k < aFr.size() ? aFr[k].clip.c_str() : "-", double(kn), double(hp), double(hd));
    }
    return st;
}
void RiseChecks(const std::vector<Frame>& aFr, const std::vector<Frame>& aOff)
{
    const RiseStats on = Rises(aFr, "prod"), off = Rises(aOff, "hold-off");
    Check(on.edges == 3 && on.picked >= 1, "CR-4 the feed has 3 stationary crouch -> stand edges, Crouch_to_Stand selected on at least one (edges, picks)", on.edges, on.picked);
    Check(on.picked >= 1 && on.held == on.picked, "CR-4 a selected Crouch_to_Stand is still playing 0.75 s later, no reselection (picks, held)", on.picked, on.held);
    Check(on.picked >= 1 && on.head <= 0.09f, "CR-4 held rise: head at most 9 cm per 50 ms (m, hold off m)", on.head, off.head);
    Check(off.picked >= 1 && off.held == 0 && off.head > on.head, "CR-4 negative control (hold off): reselected half way up, faster head (held, m)", off.held, off.head);
    const auto d = Diff(aFr, aOff);
    PrintDiff("CR-4 crouchToStandHoldS on vs off", d);
    Check(!d.empty() && AllWithin(d, {{87.7, 89.5}}), "CR-4 identity: on / off differ only after the crouch_off3 rise (differing frames)", double(d.size()));
}

// STOP COMMIT (MatcherConfig::stopCommitTailS, dll-shoulder-brake 2026-11-09): a Stop clip selected with the stick released on the ground keeps
// playing (no new selection) while the stick stays released and the stance / mode do not change, up to its tail; the first frame with stick
// input selects exactly as without the commit (same frame, same clip: no added stick latency); on / off differ only inside the stop episodes.
struct StopStats
{
    int picks = 0, held = 0, repicks = 0, restarts = 0, restartSame = 0;
    std::vector<std::pair<double, double>> windows; // [selection, end of the stick-released episode]
};
StopStats Stops(const std::vector<Frame>& aFr, const std::vector<Frame>& aOff, const Model& m, const char* aTag)
{
    StopStats st;
    const float fps = m.db.Header().fps;
    for (size_t i = 0; i < aFr.size(); ++i)
    {
        const Frame& f = aFr[i];
        if (!(f.jumped && f.mode == 0 && f.moveState == 0 && f.clipIndex >= 0 &&
              steer::ClassifyClip(std::string_view(f.clip)) == steer::kClassStop))
            continue;
        ++st.picks;
        const float len = float(m.db.Clip(uint32_t(f.clipIndex)).numFrames - 1) / fps;
        size_t j = i + 1;
        const float keepTo = std::fmin(len - 0.25f, f.clipTime + 1.0f) - 0.05f; // stopCommitTailS / stopCommitMaxS, minus one feed row
        while (j < aFr.size() && aFr[j].mode == 0 && aFr[j].moveState == 0 && aFr[j].stance == f.stance && aFr[j].clipIndex == f.clipIndex &&
               !aFr[j].jumped)
            ++j;
        // the clip plays until the stick input / stance / mode change or a new selection; a new selection with the stick still released within
        // the commit (before the clip tail and at most 1 s of clip time after the selection) is a re-pick
        const int repick = j < aFr.size() && aFr[j].mode == 0 && aFr[j].moveState == 0 && aFr[j].stance == f.stance &&
                           aFr[j - 1].clipTime < keepTo ? 1 : 0;
        st.repicks += repick;
        st.held += !repick;
        size_t e = j;
        while (e < aFr.size() && aFr[e].moveState == 0 && aFr[e].mode == 0)
            ++e;
        st.windows.emplace_back(f.t - 1e-6, (e < aFr.size() ? aFr[e].t : aFr.back().t) + 1e-6);
        if (e < aFr.size() && e < aOff.size())
        {
            ++st.restarts;
            st.restartSame += aFr[e].clipIndex == aOff[e].clipIndex && aFr[e].jumped == aOff[e].jumped;
        }
        std::printf("%s stop pick t %.3f %s ct %.3f (len %.2f s): kept to t %.3f (%s), stick-released episode ends t %.3f%s\n", aTag, f.t, f.clip.c_str(),
                    double(f.clipTime), double(len), j < aFr.size() ? aFr[j].t : -1.0, repick ? "RE-PICK" : "held", e < aFr.size() ? aFr[e].t : -1.0,
                    e < aFr.size() && e < aOff.size() ? (aFr[e].clipIndex == aOff[e].clipIndex ? ", restart pick = off" : ", restart pick != off") : "");
    }
    return st;
}
void StopChecks(const std::vector<Frame>& aFr, const std::vector<Frame>& aOff, const Model& m)
{
    const StopStats on = Stops(aFr, aOff, m, "prod"), off = Stops(aOff, aOff, m, "commit-off");
    Check(on.picks >= 2 && on.held == on.picks, "STOP a selected Stop clip plays on while the stick stays released, no re-pick (picks, held)", on.picks, on.held);
    Check(off.repicks >= 1, "STOP negative control (commit off): a selected Stop clip is re-picked by another while the stick is released (re-picks)",
          off.repicks);
    Check(on.restarts >= 1 && on.restartSame == on.restarts, "STOP the first frame with stick input picks as without the commit (restarts, same)",
          on.restarts, on.restartSame);
    const auto d = Diff(aFr, aOff);
    PrintDiff("STOP stopCommitTailS on vs off", d);
    bool inside = true;
    for (const double t : d)
    {
        bool in = false;
        for (const auto& w : on.windows)
            in = in || (t >= w.first && t <= w.second + 0.5); // + the idle that follows a held stop (its own selection time differs)
        inside = inside && in;
    }
    Check(!d.empty() && inside, "STOP identity: on / off differ only inside the stop episodes (differing frames)", double(d.size()));
}

// RC2 data guard: every slide Into clip is still upright (hips >= 0.8 m) on every row the entry cap admits, so the cap never starts a slide lying
void IntoClipGuard(const Model& m, float aCap)
{
    const float fps = m.db.Header().fps;
    int clips = 0;
    float lowest = 9.f;
    for (uint32_t c = 0; c < m.db.NumClips(); ++c)
    {
        if (!IsSlideInto(std::string(m.db.ClipName(c))))
            continue;
        ++clips;
        const auto& cl = m.db.Clip(c);
        for (uint32_t f = 0; f < cl.numFrames && float(f) / fps <= aCap + 1e-4f; ++f)
            lowest = std::fmin(lowest, FrameZ(m, cl.firstFrame + f, m.boneHips));
    }
    Check(clips == 4 && lowest >= 0.8f, "RC2 data: the 4 slide Into clips are upright up to the entry cap (clips, lowest hips m)", clips, lowest);
}

// RC2 (aEntryCap; MatcherConfig::slideEntryMaxS, off in production, see there): the sprint -> slide entry starts the Into clip before its
// authored drop; RC3: an exhausted Into clip hands over to its family's Loop clip
void SlideChecks(const std::vector<Frame>& aFr, const char* aTag, bool aEntryCap)
{
    const size_t s0 = At(aFr, kSlide0);
    size_t e = s0;
    while (e < aFr.size() && aFr[e].mode != 2)
        ++e;
    size_t x = e;
    while (x < aFr.size() && aFr[x].mode == 2)
        ++x;
    if (e >= aFr.size() || x - e < 30)
    {
        Check(false, "slide window: no slide in the replay (entry index, length)", double(e), double(x - e));
        return;
    }
    char buf[256];
    const Frame& en = aFr[e];
    std::printf("%s slide entry t %.3f %s ct %.3f cost %.3f blend %.2f; slide frames %zu\n", aTag, en.t, en.clip.c_str(), double(en.clipTime), double(en.cost),
                double(en.blend), x - e);
    const size_t e1 = At(aFr, en.t + 0.5);
    const float drop = MaxStep50(aFr, e, e1, [](const Frame& f) { return f.hipsZ; });
    std::printf("%s slide entry: hips change per 50 ms in the first 0.5 s %.3f m\n", aTag, double(drop));
    if (aEntryCap)
    {
        std::snprintf(buf, sizeof buf, "RC2 %s slide entry plays an Into clip at most 0.6 s in, before its drop (ct, cost)", aTag);
        Check(en.jumped && IsSlideInto(en.clip) && en.clipTime <= 0.6f + 1e-3f, buf, en.clipTime, en.cost);
        std::snprintf(buf, sizeof buf, "RC2 %s slide entry: hips change per 50 ms in the first 0.5 s at most 0.25 m (m)", aTag);
        Check(drop <= 0.25f, buf, drop);
    }
    int intoPicks = 0, loopPicks = 0, familySwitch = 0;
    float handKnee = 0.f, handHips = 0.f;
    for (size_t i = e + 1; i < x; ++i)
    {
        if (!aFr[i].jumped)
            continue;
        intoPicks += IsSlideInto(aFr[i].clip);
        familySwitch += SlideFamily(aFr[i].clip) != SlideFamily(en.clip);
        if (IsSlideLoop(aFr[i].clip))
        {
            ++loopPicks;
            const size_t h1 = std::min(At(aFr, aFr[i].t + 0.3), x); // the slide exit is not the handover
            handKnee = std::fmax(handKnee, std::fmax(MaxStep50(aFr, i - 3, h1, [](const Frame& f) { return f.kneeL; }),
                                                     MaxStep50(aFr, i - 3, h1, [](const Frame& f) { return f.kneeR; })));
            handHips = std::fmax(handHips, MaxStep50(aFr, i - 3, h1, [](const Frame& f) { return f.hipsZ; }));
        }
    }
    std::printf("%s slide: %d Into re-picks, %d Loop picks, %d family switches; handover knee %.1f deg / hips %.3f m per 50 ms\n", aTag, intoPicks, loopPicks,
                familySwitch, double(handKnee), double(handHips));
    std::snprintf(buf, sizeof buf, "RC3 %s mid-slide: no re-pick of an Into clip after the entry (re-picks, Loop picks)", aTag);
    Check(intoPicks == 0, buf, intoPicks, loopPicks);
    std::snprintf(buf, sizeof buf, "RC3 %s mid-slide: the exhausted Into clip hands over to the Loop of its family (Loop picks, family switches)", aTag);
    Check(loopPicks >= 1 && familySwitch == 0, buf, loopPicks, familySwitch);
    std::snprintf(buf, sizeof buf, "RC3 %s handover: knees at most 5 deg and hips at most 1 cm per 50 ms (deg, m)", aTag);
    Check(loopPicks >= 1 && handKnee <= 5.0f && handHips <= 0.01f, buf, handKnee, handHips);
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_loco_matcher_test <loco_feeds.txt> <bundle path without extension> [--report] [--dump t0 t1] [--set <knob> <value>]...\n");
        return 2;
    }
    Variant prod;
    std::vector<std::pair<double, double>> dumps;
    std::vector<std::string> clipInfos;
    for (int i = 3; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--report"))
            g_report = true;
        else if (!std::strcmp(argv[i], "--dump") && i + 2 < argc)
        {
            dumps.emplace_back(std::strtod(argv[i + 1], nullptr), std::strtod(argv[i + 2], nullptr));
            i += 2;
        }
        else if (!std::strcmp(argv[i], "--clip") && i + 1 < argc)
            clipInfos.push_back(argv[++i]);
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
    std::vector<Row> rows;
    if (!LoadFixture(argv[1], rows))
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
    std::printf("bundle %s: %u databases, %u clips; %zu rows\n", argv[2], model->db.NumDatabases(), model->db.NumClips(), rows.size());
    for (const auto& c : clipInfos)
        ClipInfo(*model, c.c_str());
    if (!clipInfos.empty())
        return 0;
    const std::vector<Frame> fr = Replay(*model, rows, prod);
    for (const auto& d : dumps)
        Dump(fr, d.first, d.second);
    if (dumps.empty())
    {
        Variant off = prod;
        off.cfg.slowCrouchLoops = false;
        CrouchRateChecks(fr, Replay(*model, rows, off));
        Variant noTail = prod;
        noTail.cfg.slowWalkStartTail = false;
        StartTailChecks(fr, Replay(*model, rows, noTail));
        Variant noHold = prod;
        noHold.cfg.crouchToStandHoldS = 0.0f;
        RiseChecks(fr, Replay(*model, rows, noHold));
        Variant noStop = prod;
        noStop.cfg.stopCommitTailS = 0.0f;
        const std::vector<Frame> frNoStop = Replay(*model, rows, noStop);
        // production holds only the standing stops (stopCommitCrouch off): the recorded feed's stops that a re-pick crossed are crouched,
        // so production equals commit-off here; the mechanism is verified with the crouched stops held too
        Check(Diff(fr, frNoStop).empty(), "STOP production (standing stops only): identical to commit-off on this feed, whose re-picked stop is crouched (differing frames)",
              double(Diff(fr, frNoStop).size()));
        Variant stopCrouch = prod;
        stopCrouch.cfg.stopCommitCrouch = true;
        StopChecks(Replay(*model, rows, stopCrouch), frNoStop, *model);
        SlideChecks(fr, "prod", prod.cfg.slideEntryMaxS > 0.0f);
        // RC2 is off in production (MatcherConfig::slideEntryMaxS): its mechanism stays verified with the measured cap
        Variant cap = prod;
        cap.cfg.slideEntryMaxS = 0.6f;
        IntoClipGuard(*model, cap.cfg.slideEntryMaxS);
        SlideChecks(Replay(*model, rows, cap), "cap", true);
        // the slide fixes alone: the slide-free parts of the feed are bit-identical, the slide differs
        Variant noSlide = prod;
        noSlide.cfg.slideEntryMaxS = 0.0f;
        noSlide.cfg.slideIntoHandover = false;
        const auto ns = Replay(*model, rows, noSlide);
        const auto ds = Diff(fr, ns);
        PrintDiff("RC2+RC3 on vs off", ds);
        Check(!ds.empty() && AllWithin(ds, {{83.1, 87.8}}), "RC2+RC3 identity: on / off differ only from the slide entry to crouch_off3 (differing frames)",
              double(ds.size()));
    }
    std::printf("loco matcher: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

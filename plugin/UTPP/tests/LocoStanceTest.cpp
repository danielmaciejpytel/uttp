// Locomotion stance fixes (branch loco/stance, recording 2026-10-31 22:55, the recorded log). The real game feed of the diag_b
// scenario (tests/data/loco_feeds_20261031_2255.txt) goes through the production stack minus the game: adapter::MapToChooser -> BuildQueryTrajectory
// -> ChooserMatcher::Update -> motionpose::PoseEvaluator (LiveCrossfade base, as MotionDriver.cpp) with the weapon overlay of the session (Pistol2H,
// Relaxed, Feminine, woman). The composed pose (overlay layering over the MM base pose) is measured across the stance edges.
//
// Usage: utpp_mm_loco_stance_test <loco_feeds.txt> <bundle path without extension> <utpp_overlays_woman.utppov> <style weights .txt>
//        [--report] [--dump t0 t1]
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

#include "../src/MotionDriver.hpp" // AirFloorStep (the production floor distance of the air feed)
#include "../src/mm/MotionBlendStack.hpp"
#include "../src/mm/MotionAdapter.hpp"
#include "../src/mm/MotionPose.hpp"
#include "TestFailFast.hpp"

namespace adapter = utpp::mm::adapter;
namespace motion = utpp::mm::motion;
namespace motionstack = utpp::mm::motionstack;
namespace math = utpp::mm::math;
namespace chooser = utpp::mm::chooser;
namespace motionpose = utpp::mm::motionpose;
namespace ov = utpp::mm::ov;
using utpp::mm::BoneTransform;

namespace
{
int g_fail = 0, g_checks = 0;
bool g_report = false;
bool g_hash = false; // --hash: per-frame FNV-1a of the composed pose bytes (identity of two builds outside the stance fades)
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
        ++g_fail;
    if (!aOk || g_report)
        std::printf("%s %s (%.4f, %.4f)\n", aOk ? "PASS" : "FAIL", aWhat, aA, aB);
    if (!aOk)
    {
        char b[384];
        std::snprintf(b, sizeof b, " (%.4f, %.4f)", aA, aB);
        UtppFailFastNote(std::string("FAIL ") + aWhat + b);
    }
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

struct Model
{
    motion::MotionDatabase db;
    chooser::Table table;
    adapter::SlideSpec slide;
    adapter::QueryTimes qtimes;
    std::vector<float> factors;
    ov::OverlayFile ovf;
    ov::StyleFamilyData style;
    int boneHips = -1, boneHead = -1, boneSpine3 = -1, boneNeck = -1, boneHandR = -1;
    int boneUpLeg[2] = {-1, -1}, boneLeg[2] = {-1, -1}, boneFoot[2] = {-1, -1};
    bool Load(const std::string& aStem, const char* aOv, const char* aStyle, std::string& aErr)
    {
        if (!db.LoadFromFile((aStem + ".utppmm").c_str(), motion::kRigIdentityWoman, aErr))
            return false;
        if (!table.LoadFile((aStem + ".chooser.json").c_str(), aErr))
            return false;
        {
            std::ifstream f(aStem + ".chooser.json", std::ios::binary);
            std::stringstream ss;
            ss << f.rdbuf();
            if (!adapter::ParseSlideSpec(ss.str(), slide))
            {
                aErr = "no slide section";
                return false;
            }
        }
        if (!ovf.LoadFromFile(aOv, ov::kRigIdentityWomanOv, aErr))
            return false;
        {
            std::ifstream sf(aStyle, std::ios::binary);
            std::stringstream ss;
            ss << sf.rdbuf();
            if (!sf.good() || !style.Parse(ss.str(), &aErr))
            {
                aErr = std::string("style weights ") + aStyle + " " + aErr;
                return false;
            }
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
            set("Head", boneHead);
            set("Spine3", boneSpine3);
            set("Neck", boneNeck);
            set("RightHand", boneHandR);
            set("LeftUpLeg", boneUpLeg[0]);
            set("RightUpLeg", boneUpLeg[1]);
            set("LeftLeg", boneLeg[0]);
            set("RightLeg", boneLeg[1]);
            set("LeftFoot", boneFoot[0]);
            set("RightFoot", boneFoot[1]);
        }
        if (boneHips < 0 || boneHead < 0 || boneSpine3 < 0 || boneNeck < 0 || boneHandR < 0 || boneUpLeg[0] < 0 || boneUpLeg[1] < 0 ||
            boneLeg[0] < 0 || boneLeg[1] < 0 || boneFoot[0] < 0 || boneFoot[1] < 0)
        {
            aErr = "bones";
            return false;
        }
        return true;
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

// measured pose: heights (m, model space), pelvis -> head distance (m), torso pitch (Hips -> Neck from vertical, deg)
struct PoseM
{
    float hipsZ = 0, headZ = 0, reach = 0, torsoDeg = 0, handZ = 0;
    float knee[2] = {0, 0}; // knee flexion (deg), left / right
};
PoseM Measure(const Model& m, const BoneTransform* p)
{
    PoseM r;
    const math::Vec3 hips = ModelPos(m.db, p, m.boneHips), head = ModelPos(m.db, p, m.boneHead), neck = ModelPos(m.db, p, m.boneNeck);
    r.hipsZ = hips.z;
    r.headZ = head.z;
    r.handZ = ModelPos(m.db, p, m.boneHandR).z;
    const math::Vec3 d = head - hips;
    r.reach = std::sqrt(math::Dot(d, d));
    const math::Vec3 u = neck - hips;
    const float lu = std::sqrt(math::Dot(u, u));
    r.torsoDeg = lu > 1e-6f ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, u.z / lu))) * 57.29577951f : 0.0f;
    for (int s = 0; s < 2; ++s)
    {
        const math::Vec3 k = ModelPos(m.db, p, m.boneLeg[s]);
        const math::Vec3 a = ModelPos(m.db, p, m.boneUpLeg[s]) - k, b = ModelPos(m.db, p, m.boneFoot[s]) - k;
        const float la = std::sqrt(math::Dot(a, a)), lb = std::sqrt(math::Dot(b, b));
        r.knee[s] = la > 1e-6f && lb > 1e-6f ? 180.0f - std::acos(std::fmax(-1.0f, std::fmin(1.0f, math::Dot(a, b) / (la * lb)))) * 57.29577951f : 0.0f;
    }
    return r;
}

// knee offset (m, model +Y = facing) from the hip -> ankle line
float KneeFwd(const Model& m, const BoneTransform* aPose, int aSide)
{
    const math::Vec3 hip = ModelPos(m.db, aPose, m.boneUpLeg[aSide]), knee = ModelPos(m.db, aPose, m.boneLeg[aSide]),
                     ank = ModelPos(m.db, aPose, m.boneFoot[aSide]);
    const math::Vec3 ax = ank - hip;
    const float l2 = math::Dot(ax, ax);
    const math::Vec3 off = knee - hip - ax * (l2 > 1e-8f ? math::Dot(knee - hip, ax) / l2 : 0.0f);
    return off.y;
}

struct Frame
{
    double t = 0;
    int stance = 0, moveState = 0, gait = 0;
    float capsuleMs = 0;
    std::string clip;
    float clipTime = 0;
    bool jumped = false;
    PoseM base, comp;
    float cSpine = 0, cHead = 0, cPelvis = 0, cArmR = 0;
    uint64_t hash = 0;
    // model-space ankle height (m) per side of the base stack output, and the blend of the layers' own FK ankle heights with the same per-bone
    // (foot) weights as the pose mix; layers = live + stored layers of the stack
    // ankleRaw / ankleComp: the same stack with the height preservation off, and the composed pose (weapon overlay layering included).
    float ankleOut[2] = {0, 0}, ankleBlend[2] = {0, 0}, ankleRaw[2] = {0, 0}, ankleComp[2] = {0, 0};
    float kneeFwd[2] = {0, 0}, kneeFwdRaw[2] = {0, 0}; // knee offset (m, model +Y = facing) from the hip -> ankle line of the base stack output
    float kneeFwdComp[2] = {0, 0}, lowerLift[2] = {0, 0}; // OVLEGS: the same of the composed pose; the foot lift of KeepLowerFootHeight (m)
    float dz[2] = {0, 0};      // the vertical ankle correction the stack asked for
    bool clamped = false, identical = true; // IK reach limit hit; output bytes equal to the uncorrected stack's
    uint32_t layers = 0;
    // diagnostics of the foot height checks (FootDump): model-space Hips height of the output, the fold of the layers' own Hips heights with the
    // Hips weights and with the foot weights, the weight of the head layer on the foot, the clips of the (up to three) newest layers
    float hipsOut = 0, hipsFoldHips = 0, hipsFoldFoot = 0, wFoot = 0;
    int layerClip[6] = {-1, -1, -1, -1, -1, -1}; // clips of the layers, newest first (-1: the stored tail / unused)
    // FS-23 (ankle floor, BlendStack::KeepAnklesAboveLayers): the production stack (foot height keeping + floor) vs `stack` (keeping alone)
    bool identicalProd = true;
    float ankleProd[2] = {0, 0}, lowestLayer[2] = {0, 0}, floorGate = 0;
    bool replaced = false;  // the pick replaced a head that was less than 30 ms old (BlendStack::Step replace path)
    bool permitted = false; // the stack's foot height keeping was allowed to act on this frame (gate open on a side)
    float contact[2] = {0, 0};                   // the stack's output foot contact signals (left / right)
    float pelvisDrop = 0, headOut = 0, headRaw = 0; // the Hips drop of the keeping (m), the model-space head height of the output / of the uncorrected stack
};

// aForceStanceAt: the pick at that feed time (a gait pick) is flagged as a stance change (gate continuity probe)
// aUnflagGroundFrom: the picks from that feed time on are not flagged as ground locomotion fades (what a slide / air / traversal entry is)
// aHolster0 < aHolster1: the weapon is put away (overlay Default, no overlay pose) for feed t in [aHolster0, aHolster1)
// aReset: the matcher is reset (what the traversal hand-back does, MotionDriver.cpp travHandback) before the first update at or after aReset->at; the
// flags of that first pick are recorded
struct ResetProbe
{
    double at = -1.0;
    bool done = false, jumped = false, groundFade = false, stanceChange = false, valid = false;
};
std::vector<Frame> Replay(Model& m, const std::vector<Row>& aRows, double aHolster0 = -1.0, double aHolster1 = -1.0, double aForceStanceAt = -1.0, double aUnflagGroundFrom = 1e9,
                          ResetProbe* aReset = nullptr)
{
    std::vector<Frame> out;
    motion::ChooserMatcher matcher;
    std::string err;
    motion::MatcherConfig cfg;
    if (!matcher.Init(&m.db, &m.table, cfg, err))
    {
        std::printf("FAIL: matcher init %s\n", err.c_str());
        UtppFailFastNote(std::string("FAIL: matcher init ") + err);
        ++g_fail;
        return out;
    }
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    if (!ev->Init(&m.db, &m.ovf, &m.style, 2, err, motionpose::BaseBlendPolicy::LiveCrossfade)) // MotionDriver.cpp
    {
        std::printf("FAIL: PoseEvaluator init %s\n", err.c_str());
        UtppFailFastNote(std::string("FAIL: PoseEvaluator init ") + err);
        ++g_fail;
        return out;
    }
    // FS-1..FS-22 check the foot height keeping (KeepFootHeight) against the plain crossfade: both stacks without the ankle floor
    // (KeepAnklesAboveLayers, loco-fix 2026-11-09; it runs after the keeping and has its own FS-23 checks on `stackProd`)
    motionstack::BlendStack stack; // the base pose alone (the evaluator's own stack is internal)
    stack.Init(m.db, m.factors);
    stack.SetFootFloor(false);
    motionstack::BlendStack stackRaw; // the same input without the foot height preservation (SetFootHeightKeep(false)): the pre-fix crossfade
    stackRaw.Init(m.db, m.factors);
    stackRaw.SetFootHeightKeep(false);
    stackRaw.SetFootFloor(false);
    motionstack::BlendStack stackProd; // the production stack (MotionDriver / PoseEvaluator defaults: keeping + ankle floor)
    stackProd.Init(m.db, m.factors);
    adapter::MapConfig mcBase;
    mcBase.slide = true;
    mcBase.slideDeceleration = adapter::kSlideDecelerationMs2; // MotionDriver.cpp StepLocked
    mcBase.slideAirGraceS = adapter::kSlideAirGraceSeconds;
    mcBase.exitAirGraceS = adapter::kExitAirGraceSeconds;
    mcBase.stanceFilterS = adapter::kStanceFilterSeconds;
    mcBase.stanceHoldInDodge = true;
    mcBase.stanceCleanEdge = true;
    mcBase.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
    mcBase.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    adapter::MapState map;
    std::vector<motion::TrajSample> traj(adapter::kQueryTrajMax);
    const uint32_t n = m.db.NumBones();
    std::vector<BoneTransform> pose(n);
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
        adapter::MapConfig mc = mcBase;
        mc.slideSpec = m.slide;
        mc.stanceSpeedBlendTime = matcher.BlendTimeFor(motion::ChooserState{}, motion::ChooserState{});
        mc.gravity = 16.0f;
        const motion::ChooserState cs = adapter::MapToChooser(f, mc, map, ctx);
        const uint32_t nt = adapter::BuildQueryTrajectory(map, m.qtimes, traj.data(), uint32_t(traj.size()));
        if (nt == 0)
            continue;
        const bool resetNow = aReset && !aReset->done && aReset->at >= 0.0 && r.t >= aReset->at;
        if (resetNow)
            matcher.Reset(true);
        motion::MatchResult res = matcher.Update(dt, traj.data(), nt, cs, nullptr);
        if (resetNow)
        {
            aReset->done = true;
            aReset->jumped = res.jumped;
            aReset->valid = res.blend.valid;
            aReset->groundFade = res.blend.groundFade;
            aReset->stanceChange = res.blend.stanceChange;
        }
        if (res.clip < 0)
            continue;
        if (r.t >= aUnflagGroundFrom)
            res.blend.groundFade = false;
        if (res.jumped && res.blend.valid && std::fabs(r.t - aForceStanceAt) < 0.02)
            res.blend.stanceChange = true; // probe: a stance pick landing inside a running (gait) fade
        const bool replaced = res.jumped && stack.Count() > 0 && stack.At(0).clock < stack.At(0).duration && stack.At(0).clock < 0.03f;
        if (!havePrev || (res.jumped && !res.blend.valid))
            stack.Reset(), stackRaw.Reset(), stackProd.Reset();
        if (!stack.Step(res, dt) || !stackRaw.Step(res, dt) || !stackProd.Step(res, dt))
            continue;
        prevDb = res.database;
        // MotionDriver.cpp (motionpose::OverlayFrameInputs in): the session carried a drawn Pistol2H, no aim (ov=3/0/xx in the log)
        motionpose::OverlayFrameInputs in;
        in.machine.overlay = r.t >= aHolster0 && r.t < aHolster1 ? ov::PoseOverlay::Default : ov::PoseOverlay::Pistol2H;
        in.machine.style = ov::Style::Feminine;
        in.machine.crouch = cs.stance == 1;
        in.machine.moving = cs.movementState == 1;
        in.machine.hasVelocity = map.st.hasVelocity;
        in.machine.wantsToSprint = adapter::PsmSprint(adapter::DetailFromFeed(r.psm, r.loco));
        in.machine.sprinting = cs.gait == 2;
        in.machine.airW = 0.0f;
        in.bodyWoman = true;
        motionpose::FrameInfo info;
        if (!ev->Evaluate(dt, res, in, pose.data(), n, &info))
        {
            std::printf("FAIL: Evaluate at t %.3f\n", r.t);
            {
                char b[96];
                std::snprintf(b, sizeof b, "FAIL: Evaluate at t %.3f", r.t);
                UtppFailFastNote(b);
            }
            ++g_fail;
            return out;
        }
        havePrev = true;
        Frame fr;
        fr.t = r.t;
        fr.stance = cs.stance;
        fr.moveState = cs.movementState;
        fr.gait = cs.gait;
        fr.capsuleMs = std::hypot(r.vel[0], r.vel[1]);
        fr.clip = std::string(m.db.ClipName(uint32_t(res.clip)));
        fr.clipTime = res.clipTime;
        fr.jumped = res.jumped;
        fr.replaced = replaced && res.blend.valid;
        fr.base =Measure(m, stack.Output());
        fr.comp = Measure(m, pose.data());
        fr.layers = stack.Count();
        fr.clamped = stack.FootKeep().clamped[0] || stack.FootKeep().clamped[1];
        fr.identical = std::memcmp(stack.Output(), stackRaw.Output(), sizeof(BoneTransform) * n) == 0;
        fr.identicalProd = std::memcmp(stack.Output(), stackProd.Output(), sizeof(BoneTransform) * n) == 0;
        fr.floorGate = stackProd.FootFloor().gate;
        for (int s = 0; s < 2; ++s)
        {
            fr.ankleProd[s] = ModelPos(m.db, stackProd.Output(), m.boneFoot[s]).z;
            fr.lowestLayer[s] = ModelPos(m.db, stack.LayerPose(0), m.boneFoot[s]).z;
            for (uint32_t i = 1; i < stack.Count(); ++i)
                fr.lowestLayer[s] = std::fmin(fr.lowestLayer[s], ModelPos(m.db, stack.LayerPose(i), m.boneFoot[s]).z);
        }
        for (int s = 0; s < 2; ++s)
        {
            fr.ankleOut[s] = ModelPos(m.db, stack.Output(), m.boneFoot[s]).z;
            fr.ankleRaw[s] = ModelPos(m.db, stackRaw.Output(), m.boneFoot[s]).z;
            fr.ankleComp[s] = ModelPos(m.db, pose.data(), m.boneFoot[s]).z;
            fr.dz[s] = stack.FootKeep().dz[s];
            fr.kneeFwd[s] = KneeFwd(m, stack.Output(), s);
            fr.kneeFwdRaw[s] = KneeFwd(m, stackRaw.Output(), s);
            fr.kneeFwdComp[s] = KneeFwd(m, pose.data(), s);
            fr.lowerLift[s] = info.lowerLift[s];
            fr.ankleBlend[s] = fr.ankleOut[s];
            if (fr.layers > 1)
            {
                float z = 0;
                for (uint32_t i = 0; i < fr.layers; ++i)
                    z += stack.Weight(i, m.boneFoot[s]) * ModelPos(m.db, stack.LayerPose(i), m.boneFoot[s]).z;
                fr.ankleBlend[s] = z;
            }
        }
        fr.hipsOut = ModelPos(m.db, stack.Output(), m.boneHips).z;
        fr.wFoot = stack.Weight(0, m.boneFoot[0]);
        fr.contact[0] = stack.OutputSignals().left;
        fr.contact[1] = stack.OutputSignals().right;
        fr.pelvisDrop = stack.FootKeep().pelvis;
        fr.headOut = ModelPos(m.db, stack.Output(), m.boneHead).z;
        fr.headRaw = ModelPos(m.db, stackRaw.Output(), m.boneHead).z;
        for (uint32_t i = 0; i < fr.layers; ++i)
        {
            fr.hipsFoldHips += stack.Weight(i, m.boneHips) * ModelPos(m.db, stack.LayerPose(i), m.boneHips).z;
            fr.hipsFoldFoot += stack.Weight(i, m.boneFoot[0]) * ModelPos(m.db, stack.LayerPose(i), m.boneHips).z;
            if (i < 6)
                fr.layerClip[i] = stack.At(i).clip;
        }
        fr.permitted = stack.FootKeep().gate[0] > 0.0f || stack.FootKeep().gate[1] > 0.0f;
        {
            uint64_t h = 1469598103934665603ull;
            const auto* by = reinterpret_cast<const unsigned char*>(pose.data());
            for (size_t k = 0; k < sizeof(BoneTransform) * n; ++k)
                h = (h ^ by[k]) * 1099511628211ull;
            fr.hash = h;
        }
        fr.cSpine = info.curves[ov::kSpine];
        fr.cHead = info.curves[ov::kHead];
        fr.cPelvis = info.curves[ov::kPelvis];
        fr.cArmR = info.curves[ov::kArmR];
        out.push_back(std::move(fr));
    }
    return out;
}

void Dump(const std::vector<Frame>& aFr, double t0, double t1)
{
    std::printf("t       st ms gt  cap  %-40s ct    | base hips head reach torso kneeL kneeR | comp hips head reach torso handZ kneeL kneeR | spine head pelvis\n", "clip");
    for (const Frame& f : aFr)
        if (f.t >= t0 && f.t <= t1)
            std::printf("%.3f %2d %2d %2d %5.2f %-40s %5.3f%s| %5.3f %5.3f %5.3f %5.1f %5.1f %5.1f | %5.3f %5.3f %5.3f %5.1f %5.3f %5.1f %5.1f | %4.2f %4.2f %4.2f\n", f.t, f.stance,
                        f.moveState, f.gait, double(f.capsuleMs), f.clip.c_str(), double(f.clipTime), f.jumped ? "*" : " ", double(f.base.hipsZ),
                        double(f.base.headZ), double(f.base.reach), double(f.base.torsoDeg), double(f.base.knee[0]), double(f.base.knee[1]),
                        double(f.comp.hipsZ), double(f.comp.headZ), double(f.comp.reach), double(f.comp.torsoDeg), double(f.comp.handZ),
                        double(f.comp.knee[0]), double(f.comp.knee[1]), double(f.cSpine), double(f.cHead), double(f.cPelvis));
}
// Overlay excursion across a stance edge. d = composed - base (torso pitch deg, head height m): the share of the pose the overlay layering
// adds. Settled corridor = the range of d over [t0 - 0.4, t0) and [t0 + 1.3, t0 + 1.7) (B fade 1.0 s + the 0.5 s base blend done); the
// excursion is how far d leaves that corridor in [t0, t0 + 1.3).
struct Excursion
{
    float torso = 0, head = 0;      // deg, m outside the corridor
    double torsoT = 0, headT = 0;   // where
};
Excursion OverlayExcursion(const std::vector<Frame>& aFr, double t0)
{
    float tLo = 1e9f, tHi = -1e9f, hLo = 1e9f, hHi = -1e9f;
    for (const Frame& f : aFr)
        if ((f.t >= t0 - 0.4 && f.t < t0) || (f.t >= t0 + 1.3 && f.t < t0 + 1.7))
        {
            const float dt = f.comp.torsoDeg - f.base.torsoDeg, dh = f.comp.headZ - f.base.headZ;
            tLo = std::fmin(tLo, dt), tHi = std::fmax(tHi, dt), hLo = std::fmin(hLo, dh), hHi = std::fmax(hHi, dh);
        }
    Excursion e;
    for (const Frame& f : aFr)
        if (f.t >= t0 && f.t < t0 + 1.3)
        {
            const float dt = f.comp.torsoDeg - f.base.torsoDeg, dh = f.comp.headZ - f.base.headZ;
            const float et = std::fmax(tLo - dt, dt - tHi), eh = std::fmax(hLo - dh, dh - hHi);
            if (et > e.torso)
                e.torso = et, e.torsoT = f.t;
            if (eh > e.head)
                e.head = eh, e.headT = f.t;
        }
    return e;
}

void StanceChecks(const std::vector<Frame>& aFr)
{
    int up = 0, down = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (aFr[i].stance == aFr[i - 1].stance)
            continue;
        const double t0 = aFr[i].t;
        const Excursion e = OverlayExcursion(aFr, t0);
        const bool toStand = aFr[i].stance == 0;
        const bool still = aFr[i].capsuleMs < 0.3f;
        char what[256];
        std::snprintf(what, sizeof(what), "edge %.3f %s %s: overlay excursion torso %.1f deg at %.3f, head %.3f m at %.3f", t0,
                      toStand ? "crouch -> stand" : "stand -> crouch", still ? "still" : "moving", double(e.torso), e.torsoT,
                      double(e.head), e.headT);
        if (g_report)
            std::printf("  %s\n", what);
        // every edge, both directions: the overlay layering stays inside its settled corridor (torso 3 deg, head 2 cm)
        const char* tag = toStand ? "CR-4 crouch -> stand" : "RC1 stand -> crouch";
        std::snprintf(what, sizeof(what), "%s at %.3f (%s): the overlay keeps the torso pitch of the base pose (excursion deg, at)", tag, t0,
                      still ? "still" : "moving");
        Check(e.torso <= 3.0f, what, e.torso, e.torsoT);
        std::snprintf(what, sizeof(what), "%s at %.3f (%s): the overlay keeps the head height of the base pose (excursion m, at)", tag, t0,
                      still ? "still" : "moving");
        Check(e.head <= 0.02f, what, e.head, e.headT);
        if (toStand && still)
            ++up;
        if (!toStand && !still)
        {
            ++down;
            // RC1 guard (run -> crouch, R3 at full stick): the composed head height and pelvis -> head distance do not rise above the range
            // of the second before the edge in the second after it
            float headRun = 0, reachRun = 0, head = 0, reach = 0;
            double headT = 0, reachT = 0;
            for (const Frame& f : aFr)
            {
                if (f.t >= t0 - 1.0 && f.t < t0)
                    headRun = std::fmax(headRun, f.comp.headZ), reachRun = std::fmax(reachRun, f.comp.reach);
                if (f.t >= t0 && f.t < t0 + 1.0)
                {
                    if (f.comp.headZ > head)
                        head = f.comp.headZ, headT = f.t;
                    if (f.comp.reach > reach)
                        reach = f.comp.reach, reachT = f.t;
                }
            }
            std::snprintf(what, sizeof(what), "RC1 stand -> crouch at %.3f (moving): composed head height not above the second before (rise m, at)", t0);
            Check(head <= headRun + 0.005f, what, head - headRun, headT);
            std::snprintf(what, sizeof(what), "RC1 stand -> crouch at %.3f (moving): pelvis -> head distance not above the second before (rise m, at)",
                          t0);
            Check(reach <= reachRun + 0.005f, what, reach - reachRun, reachT);
        }
    }
    Check(up == 3, "CR-4 the three still crouch -> stand edges of diag_b (crouch_off 1..3) are in the replay (edges, -)", up, 0);
    Check(down == 2, "RC1 the two moving stand -> crouch edges of diag_b (run_crouch, sprint_crouch) are in the replay (edges, -)", down, 0);
}
// largest single-frame step of the overlay share (composed - base) of the torso pitch (deg) and head height (m) in [t0, t1)
void MaxStep(const std::vector<Frame>& aFr, double t0, double t1, float& aTorso, float& aHead, double& aTorsoT, double& aHeadT)
{
    aTorso = aHead = 0.0f;
    aTorsoT = aHeadT = 0.0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (aFr[i].t < t0 || aFr[i].t >= t1)
            continue;
        const Frame &a = aFr[i - 1], &b = aFr[i];
        const float st = std::fabs((b.comp.torsoDeg - b.base.torsoDeg) - (a.comp.torsoDeg - a.base.torsoDeg));
        const float sh = std::fabs((b.comp.headZ - b.base.headZ) - (a.comp.headZ - a.base.headZ));
        if (st > aTorso)
            aTorso = st, aTorsoT = b.t;
        if (sh > aHead)
            aHead = sh, aHeadT = b.t;
    }
}

// CR-4 x holster: the weapon is put away 0.3 s into the crouch -> stand fade of B and drawn again 0.3 s later. The Default keys have no overlay
// pose; the spine / head must stay on the per-stance-side form through the whole fade (L stands in for the keyless group), not drop to the
// crossfade path for a frame. Measured on this edge (largest step of the overlay share per 30 Hz frame): fix 0.23 deg torso, the early
// return of mutation 3 2.23 deg at the holster frame, the pre-fix crossfade (mutation 1) 1.64 deg from the stance fade itself. The head
// step (~1 cm at the draw) is the same in all of them: the 0.2 s fade of the head layer weight of the weapon pose, not the stance form.
void HolsterChecks(const std::vector<Frame>& aFr, double t0)
{
    float st = 0, sh = 0;
    double stT = 0, shT = 0;
    MaxStep(aFr, t0, t0 + 1.3, st, sh, stT, shT);
    char what[256];
    std::snprintf(what, sizeof(what), "CR-4 holster / draw inside the crouch -> stand fade at %.3f: overlay torso step per frame (deg, at)", t0);
    Check(st <= 1.0f, what, st, stT);
    std::snprintf(what, sizeof(what), "CR-4 holster / draw inside the crouch -> stand fade at %.3f: overlay head step per frame (m, at)", t0);
    Check(sh <= 0.015f, what, sh, shT);
}

// --ankles t0 t1: per frame the ankle heights (model space, m) of both sides - output / blend of the layers / uncorrected stack / composed - and
// the Hips heights (output, fold with the Hips weights, fold with the foot weights), the head layer's foot weight and the layers' clips
void FootDump(const Model& m, const std::vector<Frame>& aFr, double t0, double t1)
{
    std::printf("t       cap  ly wF   | L out   blend  raw    comp   dz     | R out   blend  raw    comp   dz     | hips out  foldH  foldF | gate perm | contact L R | clips (newest first)\n");
    for (const Frame& f : aFr)
    {
        if (f.t < t0 || f.t > t1)
            continue;
        std::printf("%.3f %5.2f %d %4.2f | %6.3f %6.3f %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f | %d    ", f.t, double(f.capsuleMs), int(f.layers), double(f.wFoot),
                    double(f.ankleOut[0]), double(f.ankleBlend[0]), double(f.ankleRaw[0]), double(f.ankleComp[0]), double(f.dz[0]), double(f.ankleOut[1]),
                    double(f.ankleBlend[1]), double(f.ankleRaw[1]), double(f.ankleComp[1]), double(f.dz[1]), double(f.hipsOut), double(f.hipsFoldHips),
                    double(f.hipsFoldFoot), f.permitted ? 1 : 0);
        std::printf("      %4.2f %4.2f ", double(f.contact[0]), double(f.contact[1]));
        for (int k = 0; k < 6; ++k)
            if (f.layerClip[k] >= 0)
                std::printf(" %s", std::string(m.db.ClipName(uint32_t(f.layerClip[k]))).c_str());
        std::printf("\n");
    }
}

// Foot height through the transition edges (BlendStack::KeepFootHeight). sink = blend of the layers' own FK ankle heights (the pose mix's
// per-bone foot weights) - output ankle height: positive = the foot is below where the layers put it. Raw = the same stack without the height
// preservation (the pre-fix crossfade). Window [t0 - 0.05, t0 + 1.0).
struct EdgeFoot
{
    float sinkFix = 0, sinkRaw = 0, riseFix = 0, riseRaw = 0, compSink = 0; // m; compSink: composed ankles below the blend, beyond the settled overlay offset
    float stepErr = 0, stepErrRaw = 0;                                    // max per-frame |d(output ankle z) - d(target ankle z)|; the uncorrected stack's
    float kneeMin = 1e9f, kneeMinRaw = 1e9f;                              // min knee forward offset (m)
    double sinkT = 0;
    int clampedFrames = 0, jumpFrames = 0;
    float floatContact = 0;  // max (output - blend of the layers) of a foot the output holds in ground contact (>= 0.5): a floating planted foot
    float drop = 0, headLow = 0, hipsStep = 0; // max pelvis drop (m); max (uncorrected head height - output head height); max per-frame change of the drop
    float hipsOver = 0, hipsBelow = 0;         // max per-frame Hips step beyond the layers' own folds (m); max (fold of the layers' Hips with the foot weights - output Hips)
    double floatT = 0;
};
EdgeFoot MeasureEdge(const std::vector<Frame>& aFr, double t0)
{
    EdgeFoot e;
    // the weapon overlay legitimately offsets the composed ankles from the base (the armed legs differ): settled corridor of that offset
    float settled = 0;
    for (const Frame& f : aFr)
        if ((f.t >= t0 - 0.4 && f.t < t0) || (f.t >= t0 + 1.3 && f.t < t0 + 1.7))
            for (int s = 0; s < 2; ++s)
                settled = std::fmax(settled, f.ankleBlend[s] - f.ankleComp[s]);
    for (size_t k = 1; k < aFr.size(); ++k)
    {
        const Frame &a = aFr[k - 1], &f = aFr[k];
        if (f.t < t0 - 0.05 || f.t >= t0 + 1.0)
            continue;
        if (f.clamped)
            ++e.clampedFrames;
        e.drop = std::fmax(e.drop, f.pelvisDrop);
        e.headLow = std::fmax(e.headLow, f.headRaw - f.headOut);
        e.hipsStep = std::fmax(e.hipsStep, std::fabs(f.pelvisDrop - a.pelvisDrop));
        e.hipsOver = std::fmax(e.hipsOver, std::fabs(f.hipsOut - a.hipsOut) - std::fmax(std::fabs(f.hipsFoldHips - a.hipsFoldHips), std::fabs(f.hipsFoldFoot - a.hipsFoldFoot)));
        if (f.pelvisDrop > 0.0f) // a Hips lagging below the legs' fold by itself (a rise out of a slide) is not the drop's doing
            e.hipsBelow = std::fmax(e.hipsBelow, f.hipsFoldFoot - f.hipsOut);
        for (int s = 0; s < 2; ++s)
        {
            const float d = f.ankleBlend[s] - f.ankleOut[s], dr = f.ankleBlend[s] - f.ankleRaw[s];
            if (d > e.sinkFix)
                e.sinkFix = d, e.sinkT = f.t;
            e.sinkRaw = std::fmax(e.sinkRaw, dr);
            e.riseFix = std::fmax(e.riseFix, -d);
            if (f.contact[s] >= 0.5f && -d > e.floatContact)
                e.floatContact = -d, e.floatT = f.t;
            e.riseRaw = std::fmax(e.riseRaw, -dr);
            e.compSink = std::fmax(e.compSink, f.ankleBlend[s] - f.ankleComp[s] - settled);
            e.stepErr = std::fmax(e.stepErr, std::fabs((f.ankleOut[s] - a.ankleOut[s]) - (f.ankleBlend[s] - a.ankleBlend[s])));
            e.stepErrRaw = std::fmax(e.stepErrRaw, std::fabs((f.ankleRaw[s] - a.ankleRaw[s]) - (f.ankleBlend[s] - a.ankleBlend[s])));
            e.kneeMin = std::fmin(e.kneeMin, f.kneeFwd[s]);
            e.kneeMinRaw = std::fmin(e.kneeMinRaw, f.kneeFwdRaw[s]);
        }
    }
    return e;
}

void SinkReport(const std::vector<Frame>& aFr, const char* aTag)
{
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        const bool stanceEdge = aFr[i].stance != aFr[i - 1].stance;
        const bool gaitEdge = !stanceEdge && aFr[i].gait != aFr[i - 1].gait;
        if (!stanceEdge && !gaitEdge)
            continue;
        const EdgeFoot e = MeasureEdge(aFr, aFr[i].t);
        std::printf("  foot edge (%s) %.3f %s gait %d->%d stance %d->%d cap %.2f: sink fix %.4f (raw %.4f) at %.3f, rise fix %.4f (raw %.4f), composed sink %.4f, "
                    "step err %.4f, knee fwd min %.4f (raw %.4f), IK clamped frames %d, contact float %.4f, pelvis drop %.4f (step %.4f), head lower than raw %.4f\n",
                    aTag, aFr[i].t, stanceEdge ? "STANCE" : "gait", aFr[i - 1].gait, aFr[i].gait, aFr[i - 1].stance, aFr[i].stance, double(aFr[i].capsuleMs),
                    double(e.sinkFix), double(e.sinkRaw), e.sinkT, double(e.riseFix), double(e.riseRaw), double(e.compSink), double(e.stepErr),
                    double(e.kneeMin), double(e.kneeMinRaw), e.clampedFrames, double(e.floatContact), double(e.drop), double(e.hipsStep), double(e.headLow));
    }
}

// Crouch -> stand without moving (the three crouch_off edges of diag_b): the feet stay on the layers' height through the stand-up. `aTag`
// names the replay (armed Pistol2H / unarmed Default); the composed pose is the one the game draws.
// aCompLimit: the composed-pose allowance (m). Unarmed the composed ankles are the base's; armed the Pistol2H lower-body layering added its own
// ~3 cm dip during the stance fade (the unarmed replay, whose composed pose is the base, has none of it): removed by the OVLEGS fix (per-side
// legs / pelvis + KeepLowerFootHeight, OvLegsChecks), so the armed allowance is the unarmed one.
void FootChecks(const std::vector<Frame>& aFr, const char* aTag, float aCompLimit)
{
    int edges = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (!(aFr[i - 1].stance == 1 && aFr[i].stance == 0 && aFr[i].capsuleMs < 0.3f))
            continue;
        ++edges;
        const double t0 = aFr[i].t;
        const EdgeFoot e = MeasureEdge(aFr, t0);
        char what[256];
        std::snprintf(what, sizeof(what), "FS-1 crouch -> stand at %.3f (%s): the base ankles stay on the blend of the layers (max sink m, at)", t0, aTag);
        Check(e.sinkFix <= 0.015f, what, e.sinkFix, e.sinkT);
        std::snprintf(what, sizeof(what), "FS-2 crouch -> stand at %.3f (%s): the composed ankles stay on the blend of the layers, beyond the settled overlay offset (max sink m, raw base)", t0, aTag);
        Check(e.compSink <= aCompLimit, what, e.compSink, e.sinkRaw);
        std::snprintf(what, sizeof(what), "FS-3 crouch -> stand at %.3f (%s): no ankle height jump beyond the layers' own (max step error m, -)", t0, aTag);
        Check(e.stepErr <= 0.012f, what, e.stepErr, 0);
        std::snprintf(what, sizeof(what), "FS-4 crouch -> stand at %.3f (%s): the knees keep bending forward (min knee forward offset m, raw)", t0, aTag);
        Check(e.kneeMin >= -0.01f, what, e.kneeMin, e.kneeMinRaw);
    }
    Check(edges == 3, "FS-0 the three still crouch -> stand edges are in the replay (edges, -)", edges, 0);
}

// FS-12..FS-16: the ground locomotion fades (BlendRequest::groundFade, lift) and the stand -> crouch floats (pelvis drop), measured on the
// edges of the recorded feed. aBase: base stack output of the replay, the composed pose of the unarmed replay is the base.
//   A  run -> stop (95.951, a landing recovery in front of it)       B  crouched gait / slide exit -> crouch idle (63.949 66.949 76.462 85.463)
//   C  stand -> crouch (59.749 still, 73.779 moving)                  floats on gait edges count where the output holds the foot in ground contact
// The bound 1.5 cm is the one of FS-1 (sink of every foot, float of the stance edges). A gait edge lowers only through the pelvis, so the planted foot that floats
// more than its partner keeps the difference: 2.5 cm there (measured 2.0 cm at 76.695); a swinging foot's float is not counted.
void Edge(const std::vector<Frame>& aFr, double aT, float aSink, float aFloat, const char* aTag, const char* aKind, bool aStance)
{
    const EdgeFoot e = MeasureEdge(aFr, aT);
    char what[256];
    std::snprintf(what, sizeof(what), "FS-12 %s edge %.3f (%s): the base ankles stay on the blend of the layers (max sink m, uncorrected stack)", aKind, aT, aTag);
    Check(e.sinkFix <= aSink && e.sinkRaw >= 0.0f, what, e.sinkFix, e.sinkRaw);
    std::snprintf(what, sizeof(what), "FS-13 %s edge %.3f (%s): no floating foot (max float m of a planted foot%s, at)", aKind, aT, aTag,
                  aStance ? ", any foot" : "");
    Check((aStance ? e.riseFix : e.floatContact) <= aFloat, what, aStance ? e.riseFix : e.floatContact, aStance ? 0.0 : e.floatT);
    std::snprintf(what, sizeof(what), "FS-14 %s edge %.3f (%s): the correction adds no ankle step beyond the uncorrected stack's (max step error m, raw)", aKind, aT, aTag);
    Check(e.stepErr <= e.stepErrRaw + 0.005f, what, e.stepErr, e.stepErrRaw);
    std::snprintf(what, sizeof(what), "FS-15 %s edge %.3f (%s): the knees keep bending forward (min knee forward offset m, raw)", aKind, aT, aTag);
    Check(e.kneeMin >= std::fmin(-0.01f, e.kneeMinRaw), what, e.kneeMin, e.kneeMinRaw);
    // the pelvis: never below the fold of the layers' Hips with the foot weights, no per-frame step beyond what the layers' own Hips paths do, the head
    // not further down than the drop
    std::snprintf(what, sizeof(what), "FS-16 %s edge %.3f (%s): the pelvis drop is bounded (<= 12 cm), stays above the legs' own Hips fold and adds no Hips step (max drop m, max step beyond the layers' own m)", aKind, aT, aTag);
    Check(e.drop <= 0.12f && e.hipsBelow <= 0.002f && e.hipsOver <= 0.01f && e.headLow <= e.drop + 0.005f, what, e.drop, e.hipsOver);
}
bool NearEdge(const std::vector<Frame>& aFr, double aT, double aTol = 0.02)
{
    for (const Frame& f : aFr)
        if (std::fabs(f.t - aT) < aTol)
            return true;
    return false;
}
void GroundChecks(const Model& m, const std::vector<Frame>& aFr, const char* aTag)
{
    struct Spec
    {
        double t;
        const char* kind;
        bool stance;
    };
    // the first pick of each edge sits at t0 (feed time of the replay's edge)
    const Spec edges[] = {{95.951, "A run -> stop", false},      {63.949, "B crouch start", false}, {66.949, "B crouch stop", false},
                          {76.462, "B crouch stop", false},      {85.463, "B slide exit -> crouch idle", false},
                          {59.749, "C stand -> crouch still", true}, {73.779, "C run -> crouch moving", true}};
    int found = 0;
    for (const Spec& sp : edges)
    {
        if (!NearEdge(aFr, sp.t))
            continue;
        ++found;
        Edge(aFr, sp.t, 0.015f, sp.stance ? 0.015f : 0.025f, aTag, sp.kind, sp.stance);
    }
    Check(found == 7, "FS-12 the seven A / B / C edges are in the replay (edges, -)", found, 0);
    // every other ground edge without a slide in its window: the same sink bound
    int others = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (aFr[i].stance == aFr[i - 1].stance && aFr[i].gait == aFr[i - 1].gait)
            continue;
        bool listed = false;
        for (const Spec& sp : edges)
            listed = listed || std::fabs(aFr[i].t - sp.t) < 0.02;
        bool slide = false;
        for (const Frame& f : aFr)
            if (f.t >= aFr[i].t - 0.05 && f.t < aFr[i].t + 1.0)
                for (int k = 0; k < 6; ++k)
                    slide = slide || (f.layerClip[k] >= 0 && std::string(m.db.ClipName(uint32_t(f.layerClip[k]))).find("Slide") != std::string::npos);
        if (listed || slide)
            continue;
        ++others;
        const EdgeFoot e = MeasureEdge(aFr, aFr[i].t);
        char what[256];
        std::snprintf(what, sizeof(what), "FS-12 other ground edge %.3f (%s): the base ankles stay on the blend of the layers (max sink m, at)", aFr[i].t, aTag);
        Check(e.sinkFix <= 0.015f, what, e.sinkFix, e.sinkT);
    }
    Check(others >= 6, "FS-12 the other ground edges are in the replay (edges, -)", others, 0);
}

// The gate adds no pop: the requested ankle change (m, signed) changes per frame by no more than the uncorrected stack's own owed change (target
// - raw output) changes, plus 1 cm (the ramp-in at a stance edge itself adds ~6 mm: owed lift 2 - 4 cm per frame there). `aMinFlips`: the probe must really switch the correction on / off (not vacuous).
void GateChecks(const std::vector<Frame>& aFr, const char* aTag, int aMinFlips, float aLimit = 0.01f)
{
    int flips = 0;
    float worst = 0;
    double worstT = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        flips += aFr[i].identical != aFr[i - 1].identical ? 1 : 0;
        for (int s = 0; s < 2; ++s)
        {
            const float own = std::fabs((aFr[i].ankleBlend[s] - aFr[i].ankleRaw[s]) - (aFr[i - 1].ankleBlend[s] - aFr[i - 1].ankleRaw[s]));
            const float step = std::fabs(aFr[i].dz[s] - aFr[i - 1].dz[s]);
            if (step - own > worst)
                worst = step - own, worstT = aFr[i].t;
        }
    }
    char what[256];
    std::snprintf(what, sizeof(what), "FS-8 %s: the lift switches on / off without a pop (largest step beyond the owed lift's own m, at)", aTag);
    Check(flips >= aMinFlips && worst <= aLimit, what, worst, worstT);
    if (g_report)
        std::printf("  gate %s: %d switches, largest lift step beyond the owed lift's own %.4f m at %.3f\n", aTag, flips, double(worst), worstT);
}

// FS-10: a forced collapse while an old, completed stance layer is the base of the stack. Crouch idle / stand idle picks every frame (a gait-class
// fade each, none flagged) on top of one flagged first pick fill the stack to its capacity: the tail collapse folds the oldest layers, among
// them the flagged base (its own fade-in long complete). The fades above it are not stance fades: the output stays the uncorrected stack's bit
// for bit through the whole sequence, the collapse included.
void CollapseProbe(const Model& m)
{
    int crouch = -1, stand = -1;
    for (uint32_t c = 0; c < m.db.NumClips(); ++c)
    {
        const std::string name(m.db.ClipName(c));
        if (crouch < 0 && name.find("Crouch_Idle") != std::string::npos)
            crouch = int(c);
        if (stand < 0 && name == "M_Neutral_Stand_Idle_Loop")
            stand = int(c);
    }
    Check(crouch >= 0 && stand >= 0, "FS-10 the crouch / stand idle clips exist (clips, -)", crouch, stand);
    if (crouch < 0 || stand < 0)
        return;
    motionstack::BlendStack fixStack, plain;
    fixStack.Init(m.db, m.factors);
    plain.Init(m.db, m.factors);
    plain.SetFootHeightKeep(false);
    int differing = 0, collapsed = 0, frames = 0;
    float lift = 0;
    for (int k = 0; k < 14; ++k)
    {
        motion::MatchResult res;
        res.clip = k % 2 ? stand : crouch;
        res.clipTime = 0.4f + 0.03f * float(k);
        res.playRate = 1.0f;
        res.jumped = true;
        res.blend.valid = k > 0;
        res.blend.blendTime = 0.5f;
        res.blend.stanceChange = k == 0; // the base: a stance fade of the past
        if (!fixStack.Step(res, 1.0f / 30.0f) || !plain.Step(res, 1.0f / 30.0f))
            break;
        ++frames;
        if (fixStack.Collapses() > 0)
            ++collapsed;
        if (std::memcmp(fixStack.Output(), plain.Output(), sizeof(BoneTransform) * m.db.NumBones()) != 0)
            ++differing;
        lift = std::fmax(lift, std::fmax(fixStack.FootKeep().dz[0], fixStack.FootKeep().dz[1]));
    }
    Check(frames == 14 && collapsed >= 2, "FS-10 the probe really collapses the stack (frames, frames after the first collapse)", frames, collapsed);
    Check(differing == 0, "FS-10 an old stance layer collapsed into the tail leaves the gait fades above it uncorrected (differing frames, largest lift m)", differing, lift);
}

// Independent classifier of the fades the foot height keeping may act on, by clip NAME (the matcher flags them by database): a fade into a
// ground locomotion clip out of a ground locomotion, landing-recovery or slide exit clip. Slide / jump / air / traversal clips on the incoming
// side, and slide loops / take-offs / traversals on the outgoing side, are never part of one.
bool NameHas(const std::string& aName, const char* aPart) { return aName.find(aPart) != std::string::npos; }
bool GroundTo(const std::string& aName) { return !NameHas(aName, "Jump") && !NameHas(aName, "Slide") && !NameHas(aName, "Traversal"); }
bool GroundFrom(const std::string& aName)
{
    return GroundTo(aName) || (NameHas(aName, "Jump") && NameHas(aName, "_Land_")) || (NameHas(aName, "Slide") && NameHas(aName, "_Out_"));
}
// some adjacent pair of layers (newer, older) is a ground fade; a stored tail (no clip) cannot be classified and counts as one
bool GroundFadeInside(const Model& m, const Frame& f)
{
    for (uint32_t i = 0; i + 1 < f.layers && i + 1 < 6; ++i)
    {
        if (f.layerClip[i] < 0 || f.layerClip[i + 1] < 0)
            return true;
        if (GroundTo(std::string(m.db.ClipName(uint32_t(f.layerClip[i])))) && GroundFrom(std::string(m.db.ClipName(uint32_t(f.layerClip[i + 1])))))
            return true;
    }
    return false;
}

// Single-layer frames (steady state, no blend running): the output is bit for bit the uncorrected stack's. Multi-layer frames differ somewhere
// (the check is not vacuous).
void IdentityChecks(const Model& m, const std::vector<Frame>& aFr)
{
    int single = 0, multi = 0, differ = 0, singleDiffer = 0;
    for (const Frame& f : aFr)
    {
        if (f.layers == 1)
            ++single, singleDiffer += f.identical ? 0 : 1;
        else
            ++multi, differ += f.identical ? 0 : 1;
    }
    // Fades that are neither stance fades nor ground locomotion fades (slide, air, traversal) stay the plain crossfade: outside the 1 s after a
    // stance change and the 0.2 s after a classified ground fade (the gate falls over 0.12 s) the output is the uncorrected stack's bit for bit.
    // The classification is the test's own (clip names), not the stack's gate.
    int outside = 0, outsideDiffer = 0, slideWindow = 0, slideDiffer = 0;
    double lastGround = -1e9;
    for (size_t i = 0; i < aFr.size(); ++i)
    {
        if (GroundFadeInside(m, aFr[i]))
            lastGround = aFr[i].t;
        bool near = aFr[i].t - lastGround < 0.2;
        for (size_t k = 1; k < aFr.size() && !near; ++k)
            near = aFr[k].stance != aFr[k - 1].stance && aFr[i].t >= aFr[k].t - 0.05 && aFr[i].t < aFr[k].t + 1.0;
        if (near)
            continue;
        ++outside;
        outsideDiffer += aFr[i].identical ? 0 : 1;
    }
    // the slide entry is a stance change too (sprint -> crouch slide): its blend frames are held to the same identity regardless
    for (const Frame& f : aFr)
        if (f.t >= 83.10 && f.t < 84.4 && f.layers > 1 && !GroundFadeInside(m, f))
            ++slideWindow, slideDiffer += f.identical ? 0 : 1;
    Check(outside > 1000 && outsideDiffer == 0, "FS-7 slide / air / traversal fades (outside the stance and ground locomotion fades): the output is bit-identical to the uncorrected stack (frames differing, frames)", outsideDiffer, outside);
    Check(slideWindow >= 6 && slideDiffer == 0, "FS-7 the sprint -> slide entry of 83.147 (Slide_Into over the sprint loop): bit-identical to the uncorrected stack (frames differing, blend frames)", slideDiffer, slideWindow);
    Check(single > 500 && singleDiffer == 0, "FS-5 single-layer frames: the output is bit-identical to the uncorrected stack (frames differing, frames)", singleDiffer, single);
    Check(multi > 100 && differ > 20, "FS-6 blend frames exist and the correction acts on them (frames corrected, blend frames)", differ, multi);
    // FS-23 (ankle floor, BlendStack::KeepAnklesAboveLayers, loco-fix 2026-11-09): the production stack (keeping + floor) vs the keeping alone
    // (`stack`, the floor's input). A: in the sprint -> slide entry of 83.147 the keeping alone takes an ankle >= 5 cm under the lowest layer
    // of the blend (the film feed: 21 cm), the production stack keeps both ankles within the floor tolerance (1 cm) + 2 mm of it. B: frames
    // whose keeping-only ankles stay above the lowest layer - 9 mm (one layer included): bit-identical, the floor acts nowhere else. C: the
    // floor adds no ankle step: per frame and side, |step of the production ankle| <= |step of the keeping-only ankle| + 1 cm.
    float entrySink = 0, entryProd = 0, pop = 0;
    int entryFrames = 0, calm = 0, calmDiffer = 0, acting = 0;
    double popT = 0;
    for (size_t i = 0; i < aFr.size(); ++i)
    {
        const Frame& f = aFr[i];
        const bool entry = f.t >= 83.10 && f.t < 83.6 && f.layers > 1;
        entryFrames += entry ? 1 : 0;
        bool below = false;
        for (int s = 0; s < 2; ++s)
        {
            below = below || (f.layers > 1 && f.lowestLayer[s] - f.ankleOut[s] > 0.009f);
            if (entry)
            {
                entrySink = std::fmax(entrySink, f.lowestLayer[s] - f.ankleOut[s]);
                entryProd = std::fmax(entryProd, f.lowestLayer[s] - f.ankleProd[s]);
            }
            if (i > 0)
            {
                const float extra = std::fabs(f.ankleProd[s] - aFr[i - 1].ankleProd[s]) - std::fabs(f.ankleOut[s] - aFr[i - 1].ankleOut[s]);
                if (extra > pop)
                    pop = extra, popT = f.t;
            }
        }
        if (below)
            acting += f.identicalProd ? 0 : 1;
        else
            ++calm, calmDiffer += f.identicalProd ? 0 : 1;
    }
    Check(entryFrames >= 6 && entrySink >= 0.05f && entryProd <= 0.012f,
          "FS-23 A the sprint -> slide entry of 83.147: the keeping alone sinks an ankle under the lowest layer, the ankle floor keeps it within 1 cm (sink m without, with)",
          entrySink, entryProd);
    Check(calm > 1000 && calmDiffer == 0, "FS-23 B frames above the floor (keeping-only ankles >= lowest layer - 9 mm): the production stack is bit-identical (frames differing, frames)",
          calmDiffer, calm);
    Check(pop <= 0.01f, "FS-23 C the ankle floor adds no ankle step beyond the keeping-only one + 1 cm (largest extra step m, at)", pop, popT);
    if (g_report)
        std::printf("  FS-23 floor: entry %d frames, sink without %.4f with %.4f m; %d calm frames (%d differ), %d frames acted on; extra step %.4f m at %.3f\n",
                    entryFrames, double(entrySink), double(entryProd), calm, calmDiffer, acting, double(pop), popT);
}

// OVLEGS probe (--ovlegs t0 t1): per-frame ankle / hips heights (cm): blend of the layers, base stack output, composed pose.
void OvLegsDump(const std::vector<Frame>& aFr, double t0, double t1)
{
    std::printf("t      st | hipsB hipsC | blendL blendR | baseL baseR | compL compR | comp-blend L R\n");
    for (const Frame& f : aFr)
        if (f.t >= t0 && f.t <= t1)
            std::printf("%.3f %d | %6.2f %6.2f | %6.2f %6.2f | %6.2f %6.2f | %6.2f %6.2f | %6.2f %6.2f\n", f.t, f.stance, double(f.base.hipsZ) * 100.0,
                        double(f.comp.hipsZ) * 100.0, double(f.ankleBlend[0]) * 100.0, double(f.ankleBlend[1]) * 100.0, double(f.ankleOut[0]) * 100.0,
                        double(f.ankleOut[1]) * 100.0, double(f.ankleComp[0]) * 100.0, double(f.ankleComp[1]) * 100.0,
                        double(f.ankleComp[0] - f.ankleBlend[0]) * 100.0, double(f.ankleComp[1] - f.ankleBlend[1]) * 100.0);
}

// OVLEGS (loco/ovlegs): the armed (Pistol2H) lower-body layering stays on the ground plane the base pose keeps through the stance fades.
// Pre-fix the composed left ankle sank 3.0 cm (right 1.0 cm) below the layers' blend on the crouch -> stand edge 69.236, and 2.6 / 2.6 / 2.7 /
// 0.6 / 4.4 / 0.5 cm below the base pose on the edges 69.236 / 78.739 / 87.746 (crouch -> stand), 73.779 (moving), 59.749 (still stand ->
// crouch) and 83.147 (slide) for the 1 s of the stance fade of B, the weapon's lower-body layering (OP + D(B -> L) at a stance alpha that L
// has not reached) being the only difference to the unarmed replay. Sink = base ankle - composed ankle (m, model
// space; positive = the composed foot is below the base's) beyond the settled offset of the armed legs (the corridor of the same difference
// just before the edge and in [t0 + 1.0, t0 + 1.25) after the base fade, clipped at the next edge); a float (composed above the base) is not
// counted. Step = the largest per-frame change of the composed ankle height beyond the base's own.
struct OvEdge
{
    double t0 = 0;
    float sink = 0, step = 0, kneeMin = 1e9f;
    double sinkT = 0, stepT = 0;
    float floatUp = 0; // largest composed - base excursion above the settled offset (reported, not bounded)
    float liftMax = 0;
};
OvEdge MeasureOvLegs(const std::vector<Frame>& aFr, size_t aEdge)
{
    // the base pose of the composed one is the evaluator's production stack (keeping + ankle floor): ankleProd, not the keeping-only ankleOut
    OvEdge e;
    e.t0 = aFr[aEdge].t;
    double tEnd = e.t0 + 1.25;
    for (size_t i = aEdge + 1; i < aFr.size(); ++i)
        if (aFr[i].stance != aFr[i - 1].stance || aFr[i].gait != aFr[i - 1].gait)
        {
            tEnd = std::fmin(tEnd, aFr[i].t);
            break;
        }
    float settledLo[2] = {0, 0}, settledHi[2] = {0, 0}; // range of (base - composed) in the corridor, per side
    for (const Frame& f : aFr)
        if ((f.t >= e.t0 - 0.4 && f.t < e.t0) || (f.t >= e.t0 + 1.0 && f.t < tEnd))
            for (int s = 0; s < 2; ++s)
            {
                const float d = f.ankleProd[s] - f.ankleComp[s];
                settledLo[s] = std::fmin(settledLo[s], d);
                settledHi[s] = std::fmax(settledHi[s], d);
            }
    for (size_t k = aEdge; k < aFr.size(); ++k)
    {
        const Frame &a = aFr[k - 1], &f = aFr[k];
        if (f.t < e.t0 - 0.05 || f.t >= e.t0 + 1.0)
            continue;
        for (int s = 0; s < 2; ++s)
        {
            const float d = f.ankleProd[s] - f.ankleComp[s];
            if (d - settledHi[s] > e.sink)
                e.sink = d - settledHi[s], e.sinkT = f.t;
            e.floatUp = std::fmax(e.floatUp, settledLo[s] - d);
            const float st = std::fabs((f.ankleComp[s] - a.ankleComp[s]) - (f.ankleProd[s] - a.ankleProd[s]));
            if (st > e.step)
                e.step = st, e.stepT = f.t;
            e.kneeMin = std::fmin(e.kneeMin, f.kneeFwdComp[s]);
            e.liftMax = std::fmax(e.liftMax, f.lowerLift[s]);
        }
    }
    return e;
}

// aLimit: sink bound (m); aTag names the replay. Every stance edge of the replay is reported; the bounds apply to aEdges (edge times).
void OvLegsChecks(const std::vector<Frame>& aFr, const char* aTag, const std::vector<double>& aEdges, float aLimit)
{
    int found = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        if (aFr[i].stance == aFr[i - 1].stance)
            continue;
        const OvEdge e = MeasureOvLegs(aFr, i);
        if (g_report)
            std::printf("  ovlegs %s edge %.3f %s: composed sink %.4f at %.3f, float %.4f, ankle step beyond the base's %.4f at %.3f, knee fwd min %.4f, lift max %.4f\n",
                        aTag, e.t0, aFr[i].stance == 0 ? "crouch -> stand" : "stand -> crouch", double(e.sink), e.sinkT, double(e.floatUp), double(e.step),
                        e.stepT, double(e.kneeMin), double(e.liftMax));
        bool listed = false;
        for (double t : aEdges)
            listed = listed || std::fabs(t - e.t0) < 0.02;
        if (!listed)
            continue;
        ++found;
        char what[256];
        std::snprintf(what, sizeof(what), "OVLEGS-1 %s edge %.3f: the composed ankles stay within %.1f cm of the base pose's height (max sink m, at)", aTag,
                      e.t0, double(aLimit) * 100.0);
        Check(e.sink <= aLimit, what, e.sink, e.sinkT);
        std::snprintf(what, sizeof(what), "OVLEGS-2 %s edge %.3f: no ankle height step beyond the base pose's own (max per-frame step m, at)", aTag, e.t0);
        Check(e.step <= 0.012f, what, e.step, e.stepT);
        std::snprintf(what, sizeof(what), "OVLEGS-3 %s edge %.3f: the knees of the composed legs keep bending forward (min knee forward offset m, -)", aTag, e.t0);
        Check(e.kneeMin >= -0.01f, what, e.kneeMin, 0);
    }
    char what[256];
    std::snprintf(what, sizeof(what), "OVLEGS-0 %s: the %d listed stance edges are in the replay (edges, listed)", aTag, int(aEdges.size()));
    Check(found == int(aEdges.size()), what, found, double(aEdges.size()));
}

// OVLEGS-4: outside the stance fades (no side set: 1 s after a stance edge) the foot lift is off and the composed pose is the pre-fix one.
// The hash itself is compared between builds (--hash); here the lift request must be exactly 0 on every such frame, and on the fade frames
// it must act (not vacuous).
void OvLegsIdentity(const std::vector<Frame>& aFr)
{
    int outside = 0, outsideLift = 0, fade = 0, fadeLift = 0;
    for (size_t i = 0; i < aFr.size(); ++i)
    {
        bool near = false;
        for (size_t k = 1; k < aFr.size() && !near; ++k)
            near = aFr[k].stance != aFr[k - 1].stance && aFr[i].t >= aFr[k].t - 0.05 && aFr[i].t < aFr[k].t + 1.05;
        const bool lift = aFr[i].lowerLift[0] > 0.0f || aFr[i].lowerLift[1] > 0.0f;
        if (near)
            ++fade, fadeLift += lift ? 1 : 0;
        else
            ++outside, outsideLift += lift ? 1 : 0;
    }
    Check(outside > 1500 && outsideLift == 0, "OVLEGS-4 outside the stance fades the foot lift is off (frames lifted, frames)", outsideLift, outside);
    Check(fade > 100 && fadeLift > 20, "OVLEGS-5 the foot lift acts inside the stance fades (frames lifted, fade frames)", fadeLift, fade);
}
// ---- footsink3 probes (review findings M1, M2, L1): synthetic pick sequences on real clips through the production stack (dt 1/60 or 1/30), against
// the same input without the foot height keeping. A fade the matcher does not flag (jump / slide / air entry) must stay the plain crossfade, bit for
// bit, once the gate has had its fall time (kFootGateFall 0.12 s) - whatever flagged layer still lies under it.
struct SynthPick
{
    const char* clip;
    float time;
    float blend;
    bool stance = false, ground = false, toGround = false; // BlendRequest::stanceChange / groundFade / toGround as the matcher would set them
};
int ClipByName(const Model& m, const char* aName)
{
    for (uint32_t c = 0; c < m.db.NumClips(); ++c)
        if (m.db.ClipName(c) == aName)
            return int(c);
    return -1;
}
struct Synth
{
    const Model& m;
    motionstack::BlendStack fix, plain;
    int clip = -1;
    float time = 0.0f;
    bool ok = true;
    struct Out
    {
        bool identical = true;
        float gate = 0.0f, dz = 0.0f, dzMin = 0.0f, pelvis = 0.0f, dAnkle = 0.0f, dHips = 0.0f; // dzMin: the most negative (lowering) ankle correction
        uint32_t layers = 0, collapses = 0;
    };
    explicit Synth(const Model& aM) : m(aM)
    {
        fix.Init(m.db, m.factors);
        plain.Init(m.db, m.factors);
        plain.SetFootHeightKeep(false);
    }
    // aPick: a new pick (jumped) starting that clip at that time; nullptr: the clip goes on
    Out Frame(const SynthPick* aPick, float aDt)
    {
        motion::MatchResult res;
        if (aPick)
        {
            clip = ClipByName(m, aPick->clip);
            time = aPick->time;
            res.jumped = true;
            res.blend.valid = fix.Count() > 0;
            res.blend.blendTime = aPick->blend;
            res.blend.stanceChange = aPick->stance;
            res.blend.groundFade = aPick->ground;
            res.blend.toGround = aPick->toGround;
        }
        else
            time += aDt;
        res.clip = clip;
        res.clipTime = time;
        res.playRate = 1.0f;
        Out o;
        if (clip < 0 || !fix.Step(res, aDt) || !plain.Step(res, aDt))
        {
            ok = false;
            return o;
        }
        const uint32_t n = m.db.NumBones();
        o.identical = std::memcmp(fix.Output(), plain.Output(), sizeof(BoneTransform) * n) == 0;
        o.gate = std::fmax(fix.FootKeep().gate[0], fix.FootKeep().gate[1]);
        o.dz = std::fmax(std::fabs(fix.FootKeep().dz[0]), std::fabs(fix.FootKeep().dz[1]));
        o.dzMin = std::fmin(fix.FootKeep().dz[0], fix.FootKeep().dz[1]);
        o.pelvis = fix.FootKeep().pelvis;
        for (int s = 0; s < 2; ++s)
            o.dAnkle = std::fmax(o.dAnkle, std::fabs(ModelPos(m.db, fix.Output(), m.boneFoot[s]).z - ModelPos(m.db, plain.Output(), m.boneFoot[s]).z));
        o.dHips = std::fabs(ModelPos(m.db, fix.Output(), m.boneHips).z - ModelPos(m.db, plain.Output(), m.boneHips).z);
        o.layers = fix.Count();
        o.collapses = fix.Collapses();
        return o;
    }
};
// What a window of frames shows: frames that differ from the plain crossfade, the largest ankle / Hips deviation and the largest gate
struct Window
{
    int frames = 0, differing = 0;
    float ankle = 0.0f, hips = 0.0f, gate = 0.0f;
    void Add(const Synth::Out& o)
    {
        ++frames;
        differing += o.identical ? 0 : 1;
        ankle = std::fmax(ankle, o.dAnkle);
        hips = std::fmax(hips, o.dHips);
        gate = std::fmax(gate, o.gate);
    }
};
void WindowCheck(const char* aWhat, const Window& w, int aMinFrames)
{
    char what[320];
    std::snprintf(what, sizeof(what), "%s (frames differing from the plain crossfade, largest ankle deviation m)", aWhat);
    Check(w.frames >= aMinFrames && w.differing == 0, what, w.differing, w.ankle);
    if (g_report)
        std::printf("  %s: %d / %d frames differ, ankle %.4f m, hips %.4f m, gate max %.3f\n", aWhat, w.differing, w.frames, double(w.ankle), double(w.hips), double(w.gate));
}

// FS-17 (M1): a flagged ground fade is running (Idle -> Run), a take-off pick (not flagged) lands inside it. The lift the flagged fade owes must
// not go on under the take-off blend: from the gate's fall time on the output is the plain crossfade.
void UnflaggedOverFlagged(const Model& m)
{
    const float dt = 1.0f / 60.0f;
    Synth s(m);
    const SynthPick idle{"M_Neutral_Stand_Idle_Loop", 0.3f, 0.0f}, run{"M_Neutral_Run_Loop_F", 0.2f, 0.4f, false, true, true},
        jump{"M_Neutral_Jump_F_Start_Run_Lfoot", 0.0f, 0.4f};
    s.Frame(&idle, dt);
    for (int i = 0; i < 10; ++i)
        s.Frame(nullptr, dt);
    s.Frame(&run, dt);
    Window flagged, early, late;
    for (int i = 0; i < 6; ++i)
        flagged.Add(s.Frame(nullptr, dt));
    float cut = 0.0f;
    for (int i = 0; i < 30; ++i)
    {
        const Synth::Out o = s.Frame(i == 0 ? &jump : nullptr, dt);
        const float since = float(i + 1) * dt;
        if (since >= 0.15f && o.layers > 1)
            late.Add(o);
        else if (o.layers > 1)
            early.Add(o);
        cut = std::fmax(cut, o.dz);
    }
    Check(s.ok, "FS-17 the synthetic sequence steps (ok, -)", s.ok ? 1 : 0, 0);
    Check(flagged.differing >= 4 && flagged.ankle > 0.002f, "FS-17 the flagged Idle -> Run fade is corrected before the take-off pick (frames corrected, largest ankle lift m)", flagged.differing, flagged.ankle);
    WindowCheck("FS-17 take-off pick (unflagged) inside a flagged ground fade: plain crossfade from 0.15 s on", late, 6);
    if (g_report)
        std::printf("  FS-17 first 0.15 s of the take-off: %d / %d frames differ, ankle %.4f m, largest dz %.4f\n", early.differing, early.frames, double(early.ankle), double(cut));
}

// FS-18 (M1, collapse): the same with the stack at its capacity. A flagged fade folded into the stored tail with unflagged picks piling up above it:
// the tail is a flagged stored layer, the gate must not be held by it.
void CollapsedFlaggedTail(const Model& m)
{
    const float dt = 1.0f / 30.0f;
    Synth s(m);
    const SynthPick idle{"M_Neutral_Stand_Idle_Loop", 0.3f, 0.0f}, run{"M_Neutral_Run_Loop_F", 0.2f, 0.5f, false, true, true},
        crouch{"M_Neutral_Crouch_Idle_Loop", 0.4f, 0.5f}, stand{"M_Neutral_Stand_Idle_Loop", 0.6f, 0.5f};
    s.Frame(&idle, dt);
    for (int i = 0; i < 6; ++i)
        s.Frame(nullptr, dt);
    s.Frame(&run, dt);
    Window flagged, tail;
    uint32_t collapsesAtEnd = 0;
    for (int k = 0; k < 14; ++k)
    {
        const Synth::Out o = s.Frame(k % 2 ? &stand : &crouch, dt); // one unflagged pick per frame: the stack fills up and collapses
        collapsesAtEnd = o.collapses;
        if (k >= 6 && o.layers > 1)
            tail.Add(o); // from 0.2 s of unflagged picks on
        else
            flagged.Add(o);
    }
    Check(s.ok && collapsesAtEnd >= 1, "FS-18 the probe collapses the stack with the flagged fade in the tail (ok, collapses)", s.ok ? 1 : 0, collapsesAtEnd);
    WindowCheck("FS-18 unflagged picks over a collapsed flagged tail: plain crossfade from 0.2 s on", tail, 6);
}

// FS-10 (second half): the same collapse with GROUND flagged gait picks (groundFade + toGround) over the old, completed stance base (a stand idle,
// picks alternating crouch / stand idle every frame with 0.8 s fades: a stand -> crouch float of 1.3 cm shows up under a stance gate). The ground
// fades above it are corrected on their own (the lift, never a lowering through the legs - that belongs to the stance fades); the collapsed tail,
// whose group base is the old stance layer, is not a stance fade, so the stance gate never opens: no frame lowers an ankle (dz >= 0).
void CollapseProbeGround(const Model& m)
{
    const float dt = 1.0f / 30.0f;
    Synth s(m);
    uint32_t collapses = 0;
    float lowest = 0.0f;
    for (int k = 0; k < 40; ++k)
    {
        const SynthPick pick{k % 2 ? "M_Neutral_Crouch_Idle_Loop" : "M_Neutral_Stand_Idle_Loop", 0.4f + 0.03f * float(k), 0.8f, k == 0, k > 0, k > 0};
        const Synth::Out o = s.Frame(&pick, dt);
        collapses = o.collapses;
        lowest = std::fmin(lowest, o.dzMin);
    }
    Check(s.ok && collapses >= 2, "FS-10 the ground-fade probe really collapses the stack (ok, collapses)", s.ok ? 1 : 0, collapses);
    Check(lowest >= -1.0e-6f, "FS-10 an old stance layer collapsed into the tail with ground fades above it opens no stance gate: no lowering through the legs (most negative dz m, -)", lowest, 0);
}

// FS-19 (M2, replace): [Fall, Land (not flagged)] - the run pick of the next frame replaces the 17 ms old Land head and sits over the Fall layer
// (the matcher judged it against Land: ground -> ground, flagged). A fade over an air layer is not a ground fade. aFrames: frames between the Land
// pick and the run pick (1 at 60 Hz: the replace; more: the run pick lands over a Land layer that has risen a little).
Window RunOverLanding(const Model& m, float aDt, int aFrames)
{
    Synth s(m);
    const SynthPick fall{"M_Neutral_Jump_Loop_Fall", 0.2f, 0.0f}, land{"M_Neutral_Jump_F_Land_Run_Light_Lfoot", 0.0f, 0.2f},
        run{"M_Neutral_Run_Loop_F", 0.2f, 0.3f, false, true, true};
    s.Frame(&fall, aDt);
    for (int i = 0; i < 12; ++i)
        s.Frame(nullptr, aDt);
    s.Frame(&land, aDt);
    for (int i = 1; i < aFrames; ++i)
        s.Frame(nullptr, aDt);
    Window after;
    after.Add(s.Frame(&run, aDt));
    for (int i = 0; i < 24; ++i)
    {
        const Synth::Out o = s.Frame(nullptr, aDt);
        if (o.layers > 1)
            after.Add(o);
    }
    if (!s.ok)
        after.frames = 0;
    return after;
}
void ReplacedHeadOverAir(const Model& m)
{
    WindowCheck("FS-19 run pick replacing a landing head over the air layer (60 Hz, next frame): plain crossfade", RunOverLanding(m, 1.0f / 60.0f, 1), 10);
    // Report only: a run pick that lands over a landing layer that is NOT replaced (30 Hz: 33 ms > 30 ms; 60 Hz: 2+ frames later) sits over a
    // layer that rises within ~0.1 s (feet factor 1 / 1.5) to the ground pose, so the lift it owes grows with that layer - the ground fade out of a
    // landing the flag exists for - and is not held to the plain crossfade.
    if (g_report)
    {
        struct Case { float dt; int frames; };
        for (const Case c : {Case{1.0f / 30.0f, 1}, Case{1.0f / 60.0f, 2}, Case{1.0f / 60.0f, 3}, Case{1.0f / 60.0f, 6}})
        {
            const Window w = RunOverLanding(m, c.dt, c.frames);
            std::printf("  FS-19 (report) run pick %d frame(s) after the landing pick at %.0f Hz: %d / %d frames differ, ankle %.4f m\n", c.frames, double(1.0f / c.dt), w.differing, w.frames,
                        double(w.ankle));
        }
    }
}

// FS-20 (L1): a stance pick (crouch idle) replaced one frame later by an air pick: the air pick inherited the stance flag (stanceFade) without the
// toGround condition of the ground flag, so the lowering / pelvis drop ran under a jump blend.
void ReplacedStanceByAir(const Model& m)
{
    const float dt = 1.0f / 60.0f;
    Synth s(m);
    const SynthPick idle{"M_Neutral_Stand_Idle_Loop", 0.3f, 0.0f}, crouch{"M_Neutral_Crouch_Idle_Loop", 0.4f, 0.5f, true, true, true},
        fall{"M_Neutral_Jump_Loop_Fall", 0.2f, 0.5f};
    s.Frame(&idle, dt);
    for (int i = 0; i < 10; ++i)
        s.Frame(nullptr, dt);
    s.Frame(&crouch, dt);
    Window after;
    const Synth::Out rep = s.Frame(&fall, dt); // the replace
    int i = 0;
    for (Synth::Out o = rep; i < 30; o = s.Frame(nullptr, dt), ++i)
        if (o.layers > 1 && float(i) * dt >= 0.12f)
            after.Add(o);
    Check(s.ok, "FS-20 the synthetic sequence steps (ok, -)", s.ok ? 1 : 0, 0);
    WindowCheck("FS-20 air pick replacing a stance pick: plain crossfade from the gate's fall time on", after, 8);
}

// FS-21 (M2, seed): after the matcher was reset (the traversal hand-back, MotionDriver.cpp travHandback; a yield re-entry resets it as well and nothing
// steps the matcher during the external base) its first pick carries no ground / stance flag: m_cur / m_haveLastState are invalid.
void HandbackProbe(Model& m, const std::vector<Row>& aRows)
{
    ResetProbe rp;
    rp.at = 95.0;
    Replay(m, aRows, -1.0, -1.0, -1.0, 1e9, &rp);
    Check(rp.done && rp.jumped, "FS-21 the first pick after a matcher reset is a new timeline (done, jumped)", rp.done ? 1 : 0, rp.jumped ? 1 : 0);
    Check(!rp.groundFade && !rp.stanceChange, "FS-21 the first pick after a matcher reset (the traversal hand-back seed) is flagged neither as ground nor as stance fade (groundFade, stanceChange)", rp.groundFade ? 1 : 0,
          rp.stanceChange ? 1 : 0);
}

// FS-22 (M1, recorded feed): the blind spot of the identity checks - a head pick that is a slide / jump / traversal clip (never flagged) lies over a
// flagged ground fade. From the gate's fall time after the pick on the output is the uncorrected stack's.
void RecordedUnflaggedHead(const Model& m, const std::vector<Frame>& aFr, const char* aTag)
{
    int frames = 0, differing = 0, replaced = 0, replacedDiffer = 0;
    double lastPick = -1e9, worstT = 0;
    for (const Frame& f : aFr)
    {
        if (f.jumped)
            lastPick = f.t;
        if (f.replaced)
            ++replaced, replacedDiffer += f.identical ? 0 : 1;
        if (f.layers < 2 || f.layerClip[0] < 0 || GroundTo(std::string(m.db.ClipName(uint32_t(f.layerClip[0])))) || f.t - lastPick < 0.15)
            continue;
        ++frames;
        if (!f.identical)
            ++differing, worstT = f.t;
    }
    char what[256];
    std::snprintf(what, sizeof(what), "FS-22 recorded feed (%s): slide / jump / traversal head over other layers, 0.15 s after its pick: the output is the uncorrected stack's (frames differing, frames)", aTag);
    Check(frames >= 5 && differing == 0, what, differing, frames); // the feed has few such frames (slide entries); the synthetic FS-17 carries the weight
    if (g_report)
        std::printf("  FS-22 %s: %d / %d frames differ (last at %.3f); %d replaced heads, %d of them corrected\n", aTag, differing, frames, worstT, replaced, replacedDiffer);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("usage: utpp_mm_loco_stance_test <loco_feeds.txt> <bundle stem> <overlays_woman.utppov> <style weights> [--report] [--dump t0 t1]\n");
        return 2;
    }
    std::vector<std::pair<double, double>> dumps, ankles, ovl;
    for (int i = 5; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--report"))
            g_report = true;
        else if (!std::strcmp(argv[i], "--hash"))
            g_hash = true;
        else if (!std::strcmp(argv[i], "--ankles") && i + 2 < argc)
        {
            ankles.emplace_back(std::strtod(argv[i + 1], nullptr), std::strtod(argv[i + 2], nullptr));
            i += 2;
        }
        else if (!std::strcmp(argv[i], "--ovlegs") && i + 2 < argc)
        {
            ovl.emplace_back(std::strtod(argv[i + 1], nullptr), std::strtod(argv[i + 2], nullptr));
            i += 2;
        }
        else if (!std::strcmp(argv[i], "--dump") && i + 2 < argc)
        {
            dumps.emplace_back(std::strtod(argv[i + 1], nullptr), std::strtod(argv[i + 2], nullptr));
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
    if (!model->Load(argv[2], argv[3], argv[4], err))
    {
        std::printf("FAIL: load: %s\n", err.c_str());
        return 1;
    }
    const std::vector<Frame> fr = Replay(*model, rows);
    for (const auto& d : dumps)
        Dump(fr, d.first, d.second);
    for (const auto& d : ankles)
        FootDump(*model, fr, d.first, d.second);
    for (const auto& d : ovl)
        OvLegsDump(fr, d.first, d.second);
    if (g_hash)
        for (const Frame& f : fr)
            std::printf("hash %.3f %d %016llx\n", f.t, f.stance, static_cast<unsigned long long>(f.hash));
    if (dumps.empty() && ankles.empty() && ovl.empty())
    {
        StanceChecks(fr);
        double t0 = -1.0; // the first still crouch -> stand edge (crouch_off, 69.236)
        for (size_t i = 1; i < fr.size() && t0 < 0.0; ++i)
            if (fr[i - 1].stance == 1 && fr[i].stance == 0 && fr[i].capsuleMs < 0.3f)
                t0 = fr[i].t;
        const std::vector<Frame> frH = Replay(*model, rows, t0 + 0.3, t0 + 0.6);
        if (g_report)
        {
            float st = 0, sh = 0;
            double stT = 0, shT = 0;
            MaxStep(fr, t0, t0 + 1.3, st, sh, stT, shT);
            std::printf("  edge %.3f armed: overlay step per frame torso %.2f deg at %.3f, head %.4f m at %.3f\n", t0, double(st), stT, double(sh), shT);
            MaxStep(frH, t0, t0 + 1.3, st, sh, stT, shT);
            std::printf("  edge %.3f holster: overlay step per frame torso %.2f deg at %.3f, head %.4f m at %.3f\n", t0, double(st), stT, double(sh), shT);
        }
        HolsterChecks(frH, t0);
        OvLegsChecks(frH, "holster", {t0}, 0.015f); // OVLEGS: the weapon put away / drawn again inside the fade (keyless groups in the sides)
        FootChecks(fr, "armed", 0.015f);
        // every stance edge of the armed replay: the three still crouch -> stand edges, the still stand -> crouch edge 59.749, the moving ones
        // 73.779 (run_crouch) and 83.147 (sprint_crouch, slide)
        OvLegsChecks(fr, "armed", {59.749, 69.236, 73.779, 78.739, 83.147, 87.746}, 0.015f);
        OvLegsIdentity(fr);
        IdentityChecks(*model, fr);
        const std::vector<Frame> frU = Replay(*model, rows, -1e9, 1e9); // weapon put away: the Default overlay (the recorded unarmed case)
        FootChecks(frU, "unarmed", 0.015f);
        GroundChecks(*model, fr, "armed");
        GroundChecks(*model, frU, "unarmed");
        GateChecks(fr, "recorded feed", 6);
        // the stance gate also lowers: a swinging foot that floats 5 cm when the forced stance pick lands comes down over the alpha ramp (2 cm / frame)
        GateChecks(Replay(*model, rows, -1.0, -1.0, 80.172), "stance pick inside the 79.989 gait fade", 6, 0.02f);
        // FS-9: the stance fade ends under a newer fade that is not flagged (the 79.989 pick flagged as a stance change, the 80.172 pick, as an
        // unflagged slide / air entry would be, lands inside it and outlives it)
        GateChecks(Replay(*model, rows, -1.0, -1.0, 79.989, 80.1), "stance fade ending under the newer 80.172 gait fade", 6);
        CollapseProbe(*model);
        CollapseProbeGround(*model);
        UnflaggedOverFlagged(*model);
        CollapsedFlaggedTail(*model);
        ReplacedHeadOverAir(*model);
        ReplacedStanceByAir(*model);
        HandbackProbe(*model, rows);
        RecordedUnflaggedHead(*model, fr, "armed");
    }
    if (g_report)
    {
        SinkReport(fr, "armed");
        SinkReport(Replay(*model, rows, -1e9, 1e9), "unarmed");
    }
    std::printf("%s: %d checks, %d failed\n", g_fail ? "FAIL" : "PASS", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

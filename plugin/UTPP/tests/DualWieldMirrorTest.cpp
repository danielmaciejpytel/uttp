// Offline test of src/mm/DualWieldMirror.hpp (dual wield: the left arm mirrors the right arm across the chest mid plane) on the real
// rigs: rest pose and bone order from the motion database, weapon poses from the overlay file, both bodies.
// Usage: utpp_dualwield_mirror_test <woman.utppmm> <woman.utppov> <man.utppmm> <man.utppov>
// Per rig:
//  1) the mirror builds on the rest pose (mirror-symmetric rig: |N - round(N)| <= 0.01), lateral axis and residue printed;
//  2) every one-handed weapon pose, also with the chest turned (spine yaw 35 deg, pitch 12 deg): after Apply(1) an independent forward
//     kinematics of the whole pose puts the left hand, forearm, upper arm, clavicle and WeaponLeft at the reflection of the right ones
//     in the Spine3 frame within 1 cm, and the left hand / weapon rotation within 0.5 deg of S * Rr * N;
//  3) every bone outside the five left bones is bit-identical after Apply;
//  4) weight 0 (and a ramp that never turned on, and a single weapon: DualWieldMirrorWanted false) leaves the pose bit-identical;
//  5) the blend (also a gap in the stepping: kept while wanted, 0 when not): 60 Hz ramp on, hold, off: the left hand moves continuously (largest step <= 3 x the mean step, and <= 2.5 cm),
//     reaches the full mirror at the end of the rise and the input pose (bit-identical) at the end of the fall;
//  6) fail closed: a rest pose with a turned left upper arm, or a missing bone, gives no mirror and Apply does nothing;
//  7) the twin stance (aim / hip fire): over the one-handed aim poses, the aim offset sweep and a turned chest the right barrel
//     lies along the chest mid plane with its elevation kept, the left barrel is parallel, hands 20..50 cm and weapons >= 15 cm
//     apart, only RightArm changes on the right side; the transitions (hip-fire raise, aim raise, taps, a reload envelope) move
//     the left hand <= 2.5 cm per frame at 60 Hz. With a dump file (5th argument) the constructed poses are written for the
//     anatomy gate of the tools.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/DualWieldMirror.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/Overlay.hpp"
#include "../src/mm/OverlayFile.hpp"

using namespace utpp::mm;
namespace dm = utpp::mm::dualmirror;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}

// Independent forward kinematics (float, root to leaf) -> model transforms.
struct Model
{
    std::vector<math::Quat> r;
    std::vector<math::Vec3> p;
};
static Model Fk(const std::vector<int32_t>& aPar, const std::vector<BoneTransform>& aL)
{
    Model m;
    m.r.resize(aL.size());
    m.p.resize(aL.size());
    for (size_t i = 0; i < aL.size(); ++i)
    {
        const int32_t pa = aPar[i];
        if (pa < 0)
        {
            m.r[i] = aL[i].q;
            m.p[i] = aL[i].p;
        }
        else
        {
            m.r[i] = math::QuatNormalize(math::QuatMul(m.r[pa], aL[i].q));
            m.p[i] = m.p[pa] + math::QuatRotate(m.r[pa], aL[i].p);
        }
    }
    return m;
}
static math::Vec3 ToChest(const Model& aM, int aChest, math::Vec3 aP)
{
    return math::QuatRotate(math::QuatInv(aM.r[aChest]), aP - aM.p[aChest]);
}
static float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
static math::Quat AxisAngle(math::Vec3 aAxis, float aDeg)
{
    const float h = 0.5f * aDeg * math::kPi / 180.0f;
    const float n = Len(aAxis);
    return math::Quat{aAxis.x / n * std::sin(h), aAxis.y / n * std::sin(h), aAxis.z / n * std::sin(h), std::cos(h)};
}
static double MatAngleDeg(const dm::M3& a, const dm::M3& b)
{
    const dm::M3 d = dm::Mul(dm::Transpose(a), b);
    const double c = std::fmax(-1.0, std::fmin(1.0, 0.5 * (d.m[0][0] + d.m[1][1] + d.m[2][2] - 1.0)));
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

struct RigData
{
    std::vector<std::string> names;
    std::vector<int32_t> par;
    std::vector<BoneTransform> rest;
    int Find(const std::string& n) const
    {
        for (size_t i = 0; i < names.size(); ++i)
            if (names[i] == n)
                return static_cast<int>(i);
        return -1;
    }
};

static dm::Mirror BuildOf(const RigData& aR, const std::vector<BoneTransform>& aRest)
{
    return dm::Build(static_cast<uint32_t>(aR.names.size()), [&](uint32_t i) { return std::string_view(aR.names[i]); },
                     [&](uint32_t i) { return aR.par[i]; }, aRest.data());
}

static void CheckMirrored(const char* aTag, const std::string& aPose, const RigData& aR, const dm::Mirror& aM,
                          const std::vector<BoneTransform>& aIn, double& aWorstCm, double& aWorstDeg)
{
    std::vector<BoneTransform> out = aIn;
    Check(dm::Apply(aM, out.data(), 1.0f), std::string(aTag) + " " + aPose + ": Apply edits the pose");
    // 3) bones outside the left set bit-identical
    bool untouched = true;
    for (size_t i = 0; i < out.size(); ++i)
    {
        bool isLeft = false;
        for (int k = 0; k < dm::kPairs; ++k)
            isLeft |= aM.left[k] == static_cast<int32_t>(i);
        if (!isLeft && std::memcmp(&out[i], &aIn[i], sizeof(BoneTransform)) != 0)
            untouched = false;
    }
    Check(untouched, std::string(aTag) + " " + aPose + ": bones outside the left arm chain unchanged");
    // 2) independent FK: reflection in the chest frame
    const Model m = Fk(aR.par, out);
    double worstCm = 0.0, worstDeg = 0.0;
    const dm::M3 chestR = dm::FromQuat(m.r[aM.chest]);
    for (int k = 0; k < dm::kPairs; ++k)
    {
        const math::Vec3 pr = ToChest(m, aM.chest, m.p[aM.right[k]]);
        const math::Vec3 pl = ToChest(m, aM.chest, m.p[aM.left[k]]);
        const dm::V3d want = dm::Add(aM.mid, dm::Apply3(aM.s, dm::Sub(dm::ToD(pr), aM.mid)));
        const dm::V3d d = dm::Sub(want, dm::ToD(pl));
        const double cm = 100.0 * std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        worstCm = std::fmax(worstCm, cm);
        const dm::M3 rr = dm::Mul(dm::Transpose(chestR), dm::FromQuat(m.r[aM.right[k]]));
        const dm::M3 rl = dm::Mul(dm::Transpose(chestR), dm::FromQuat(m.r[aM.left[k]]));
        const double deg = MatAngleDeg(dm::Mul(dm::Mul(aM.s, rr), aM.n[k]), rl);
        worstDeg = std::fmax(worstDeg, deg);
        Check(cm <= 1.0, std::string(aTag) + " " + aPose + ": " + std::string(dm::kLeftNames[k]) + " at the reflection of " +
                             std::string(dm::kRightNames[k]) + " within 1 cm (" + std::to_string(cm) + " cm)");
        Check(deg <= 0.5, std::string(aTag) + " " + aPose + ": " + std::string(dm::kLeftNames[k]) + " rotation within 0.5 deg (" +
                              std::to_string(deg) + ")");
    }
    // the hands are apart (shoulder width scale), not on top of each other
    const float hands = Len(m.p[aM.left[3]] - m.p[aM.right[3]]);
    std::printf("[%s] %-32s hands %.1f cm apart, worst mirror error %.4f cm / %.4f deg\n", aTag, aPose.c_str(), 100.0 * hands, worstCm,
                worstDeg);
    aWorstCm = std::fmax(aWorstCm, worstCm);
    aWorstDeg = std::fmax(aWorstDeg, worstDeg);
}

static void TestBlend(const char* aTag, const RigData& aR, const dm::Mirror& aM, const std::vector<BoneTransform>& aIn)
{
    const float dt = 1.0f / 60.0f;
    dm::Ramp ramp;
    std::vector<float> path;
    std::vector<BoneTransform> full = aIn;
    dm::Apply(aM, full.data(), 1.0f);
    const math::Vec3 fullHand = Fk(aR.par, full).p[aM.left[3]];
    math::Vec3 prev = Fk(aR.par, aIn).p[aM.left[3]];
    double sum = 0.0, worst = 0.0;
    int steps = 0;
    bool reachedFull = false, backToInput = false, finite = true;
    for (int f = 0; f < 120; ++f)
    {
        const bool on = f < 60; // 1 s on (rise dm::kBlendSeconds, hold), then off
        const float w = dm::ShownWeight(ramp.Step(on, dt), 1.0f, 0.0f);
        std::vector<BoneTransform> pose = aIn;
        dm::Apply(aM, pose.data(), w);
        const math::Vec3 hand = Fk(aR.par, pose).p[aM.left[3]];
        finite &= math::IsFinite(hand);
        const double step = Len(hand - prev);
        sum += step;
        worst = std::fmax(worst, step);
        ++steps;
        prev = hand;
        if (on && w >= 1.0f)
            reachedFull |= Len(hand - fullHand) <= 1e-4f;
        if (!on && w <= 0.0f)
            backToInput |= std::memcmp(pose.data(), aIn.data(), sizeof(BoneTransform) * aIn.size()) == 0;
    }
    const double mean = sum / steps;
    // the moving frames only: the rise and the fall frames carry the whole path twice
    const double meanMoving = sum / (2.0 * std::ceil(dm::kBlendSeconds * 60.0 - 1e-3));
    std::printf("[%s] blend: path %.1f cm, largest step %.2f cm, mean moving step %.2f cm\n", aTag, 100.0 * sum / 2.0, 100.0 * worst,
                100.0 * meanMoving);
    Check(finite, std::string(aTag) + ": blend finite");
    Check(worst <= 3.0 * meanMoving + 1e-6, std::string(aTag) + ": blend continuous (largest step <= 3 x mean moving step)");
    Check(worst <= 0.025, std::string(aTag) + ": blend step <= 2.5 cm at 60 Hz");
    Check(reachedFull, std::string(aTag) + ": blend reaches the full mirror");
    Check(backToInput, std::string(aTag) + ": blend returns to the input pose bit for bit");
    (void)mean;
}

// Twin stance (aim / hip fire with two weapons): the right upper arm is turned about the shoulder until the right barrel lies in
// the chest mid plane direction, the left arm mirrors it. Over the one-handed aim poses (standing and crouched), the aim offset sweep
// frames (pitch from up to down) and a turned chest:
//  - with a barrel heading forward (forward component >= dm::kTwinHeadingFade) the right barrel has no sideways component in the
//    chest frame (<= 0.5 deg out of the mid plane direction) and the barrels are parallel (<= 1 deg); steeply up (heading faded)
//    the barrels stay within 3 deg of each other; the barrel elevation is kept (barrel . up unchanged within 1e-4);
//  - the hands are 25..40 cm apart and the weapons at least 15 cm apart (they do not touch);
//  - only RightArm and RightHand change on the right side (clavicle position and rotation unchanged: the shoulders are not narrowed);
//  - weight 0 leaves the pose bit-identical.
// The constructed poses are written to the dump file (database bone order, game-local) for the anatomy gate of the tools.
static double Deg(double aRad) { return aRad * 180.0 / 3.14159265358979323846; }
static math::Vec3 BarrelOf(const Model& aM, int aBone)
{
    return math::QuatRotate(aM.r[aBone], math::Vec3{0.0f, 1.0f, 0.0f});
}
static double AngleDeg(math::Vec3 a, math::Vec3 b)
{
    const double c = math::Dot(a, b) / (static_cast<double>(Len(a)) * Len(b));
    return Deg(std::acos(std::fmax(-1.0, std::fmin(1.0, c))));
}
// The twin's left arm: the mirror targets with the pose's own left clavicle (weight aOwn), written to the pose.
static bool ApplyTwinMirror(const dm::Mirror& aM, std::vector<BoneTransform>& aP, float aOwn)
{
    BoneTransform t[dm::kPairs];
    if (!dm::MirroredLeftLocals(aM, aP.data(), t, aOwn))
        return false;
    for (int k = 0; k < dm::kPairs; ++k)
        aP[aM.left[k]] = t[k];
    return true;
}
static void DumpPose(std::FILE* aF, const char* aTag, const std::string& aName, const std::vector<BoneTransform>& aP)
{
    if (aF == nullptr)
        return;
    std::fprintf(aF, "pose %s %s\n", aTag, aName.c_str());
    for (const auto& b : aP)
        std::fprintf(aF, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f\n", b.p.x, b.p.y, b.p.z, b.q.x, b.q.y, b.q.z, b.q.w);
}

struct TwinStats
{
    double worstPlaneDeg = 0.0, worstParallelDeg = 0.0, worstElev = 0.0, minHands = 1e9, maxHands = 0.0, minGuns = 1e9, maxTurnDeg = 0.0;
    int poses = 0;
};

static void CheckTwin(const char* aTag, const std::string& aName, const RigData& aR, const dm::Mirror& aM,
                      const std::vector<BoneTransform>& aIn, TwinStats& aS, std::FILE* aDump)
{
    const size_t bytes = sizeof(BoneTransform) * aIn.size();
    {
        std::vector<BoneTransform> z = aIn;
        Check(dm::ApplyTwinRight(aM, z.data(), 0.0f) == 0.0 && std::memcmp(z.data(), aIn.data(), bytes) == 0,
              std::string(aTag) + " " + aName + ": twin weight 0: pose bit-identical");
    }
    const Model m0 = Fk(aR.par, aIn);
    std::vector<BoneTransform> out = aIn;
    double spread = 0.0;
    const double turn = dm::ApplyTwinRight(aM, out.data(), 1.0f, &spread);
    bool onlyArm = true;
    for (size_t i = 0; i < out.size(); ++i)
        if (static_cast<int32_t>(i) != aM.right[1] && static_cast<int32_t>(i) != aM.right[3] &&
            std::memcmp(&out[i], &aIn[i], sizeof(BoneTransform)) != 0)
            onlyArm = false;
    Check(onlyArm, std::string(aTag) + " " + aName + ": twin turn edits only RightArm / RightHand (clavicles unchanged)");
    const double clavBefore = 100.0 * Len(Fk(aR.par, out).p[aM.left[0]] - Fk(aR.par, out).p[aM.right[0]]);
    Check(ApplyTwinMirror(aM, out, 1.0f), std::string(aTag) + " " + aName + ": mirror applied on the twin");
    const Model m = Fk(aR.par, out);
    const math::Quat chestInv = math::QuatInv(m.r[aM.chest]);
    const math::Vec3 br = math::QuatRotate(chestInv, BarrelOf(m, aM.right[4]));
    const math::Vec3 bl = math::QuatRotate(chestInv, BarrelOf(m, aM.left[4]));
    const math::Vec3 br0 = math::QuatRotate(math::QuatInv(m0.r[aM.chest]), BarrelOf(m0, aM.right[4]));
    const math::Vec3 up = dm::ToF(aM.up), lat = dm::ToF(aM.lateral);
    const double plane = Deg(std::asin(std::fmin(1.0, std::fabs(static_cast<double>(math::Dot(br, lat))) / Len(br))));
    const double parallel = AngleDeg(br, bl);
    const double elev = std::fabs(static_cast<double>(math::Dot(br, up) - math::Dot(br0, up)));
    const double hands = 100.0 * Len(m.p[aM.left[3]] - m.p[aM.right[3]]);
    const double shoulders = 100.0 * Len(m.p[aM.left[1]] - m.p[aM.right[1]]);
    const double shouldersIn = 100.0 * Len(m0.p[aM.left[1]] - m0.p[aM.right[1]]);
    Check(shoulders >= shouldersIn - 0.05, std::string(aTag) + " " + aName + ": shoulder joints not narrowed (" + std::to_string(shoulders) +
                                               " cm, pose " + std::to_string(shouldersIn) + " cm)");
    (void)clavBefore;
    const double guns = 100.0 * Len(m.p[aM.left[4]] - m.p[aM.right[4]]);
    const double elevDeg = Deg(std::asin(std::fmax(-1.0, std::fmin(1.0, static_cast<double>(math::Dot(br, up))))));
    const bool heading = math::Dot(br0, dm::ToF(aM.fwd)) >= dm::kTwinHeadingFade;
    std::printf("[%s] twin %-44s turn %6.1f deg, spread %5.1f deg, barrel out of plane %.3f deg, barrels %.3f deg apart, elevation %5.1f deg, "
                "hands %.1f cm, guns %.1f cm%s\n", aTag, aName.c_str(), turn, spread, plane, parallel, elevDeg, hands, guns,
                heading ? "" : " (steep: heading faded)");
    Check(elev <= 1e-4, std::string(aTag) + " " + aName + ": barrel elevation kept");
    if (heading)
    {
        Check(plane <= 0.5, std::string(aTag) + " " + aName + ": right barrel along the mid plane direction (" + std::to_string(plane) + " deg)");
        Check(parallel <= 1.0, std::string(aTag) + " " + aName + ": barrels parallel (" + std::to_string(parallel) + " deg)");
        Check(hands >= 25.0 && hands <= 40.0, std::string(aTag) + " " + aName + ": hands 25..40 cm apart (" + std::to_string(hands) + ")");
        Check(guns >= 15.0, std::string(aTag) + " " + aName + ": weapons at least 15 cm apart (" + std::to_string(guns) + ")");
    }
    else
    {
        // steeply up (heading faded): the twin is never worse than the plain mirror of the same pose
        std::vector<BoneTransform> plainPose = aIn;
        dm::Apply(aM, plainPose.data(), 1.0f);
        const Model mp = Fk(aR.par, plainPose);
        const math::Quat ci = math::QuatInv(mp.r[aM.chest]);
        const double plainPar = AngleDeg(math::QuatRotate(ci, BarrelOf(mp, aM.right[4])), math::QuatRotate(ci, BarrelOf(mp, aM.left[4])));
        const double plainGuns = 100.0 * Len(mp.p[aM.left[4]] - mp.p[aM.right[4]]);
        std::printf("[%s]      steep: plain mirror barrels %.2f deg apart, weapons %.1f cm\n", aTag, plainPar, plainGuns);
        Check(parallel <= plainPar + 0.01, std::string(aTag) + " " + aName + ": steep: barrels no further apart than the plain mirror");
    }
    if (heading)
        aS.worstPlaneDeg = std::fmax(aS.worstPlaneDeg, plane);
    aS.worstParallelDeg = std::fmax(aS.worstParallelDeg, parallel);
    aS.worstElev = std::fmax(aS.worstElev, elev);
    aS.minHands = std::fmin(aS.minHands, hands);
    aS.maxHands = std::fmax(aS.maxHands, hands);
    aS.minGuns = std::fmin(aS.minGuns, guns);
    aS.maxTurnDeg = std::fmax(aS.maxTurnDeg, std::fabs(turn));
    ++aS.poses;
    DumpPose(aDump, aTag, aName, out);
    if (aDump != nullptr)
    {
        // references for the gate: the input pose and its plain mirror (no twin turn)
        DumpPose(aDump, aTag, aName + " [input]", aIn);
        std::vector<BoneTransform> plain = aIn;
        dm::Apply(aM, plain.data(), 1.0f);
        DumpPose(aDump, aTag, aName + " [mirror]", plain);
    }
}

// Transitions at 60 Hz: the machine's aiming weight a drives the twin turn of the right arm; the left arm follows its mirror target
// (dm::ApplyFollow). Ready <-> aim with the hip-fire raise (0.1 s) and the aim raise (0.2 s), hip-fire taps, a reload envelope in
// the stance. The left hand moves at most 2.5 cm per frame, and ends on the mirror of the twin stance / of the ready pose.
static std::vector<BoneTransform> Lerped(const std::vector<BoneTransform>& aA, const std::vector<BoneTransform>& aB, float aU)
{
    std::vector<BoneTransform> p = aA;
    for (size_t i = 0; i < p.size(); ++i)
    {
        p[i].q = math::QuatNlerpShortest(aA[i].q, aB[i].q, aU);
        p[i].p = math::Lerp(aA[i].p, aB[i].p, aU);
    }
    return p;
}
static void TestTwinTransitions(const char* aTag, const RigData& aR, const dm::Mirror& aM, const std::vector<BoneTransform>& aReady,
                                const std::vector<BoneTransform>& aAim, std::FILE* aDump)
{
    const float dt = 1.0f / 60.0f;
    // chest frame: the ready and the aim pose also differ in the spine, which moves both hands alike (the pose blend of the frame)
    auto leftHand = [&](const std::vector<BoneTransform>& aP) {
        const Model m = Fk(aR.par, aP);
        return ToChest(m, aM.chest, m.p[aM.left[3]]);
    };
    auto rightHand = [&](const std::vector<BoneTransform>& aP) {
        const Model m = Fk(aR.par, aP);
        return ToChest(m, aM.chest, m.p[aM.right[3]]);
    };
    auto frame = [&](dm::Follow& aF, float aAimW, float aShown, std::vector<BoneTransform>& aOut) {
        aOut = Lerped(aReady, aAim, aAimW);
        dm::ApplyTwinRight(aM, aOut.data(), aShown * aAimW);
        dm::ApplyFollow(aM, aF, aOut.data(), aShown, dt, aShown * aAimW);
    };
    auto final = [&](float aAimW) {
        std::vector<BoneTransform> p = Lerped(aReady, aAim, aAimW);
        dm::ApplyTwinRight(aM, p.data(), aAimW);
        ApplyTwinMirror(aM, p, aAimW);
        return p;
    };
    struct Case
    {
        const char* name;
        std::vector<float> aim; // aiming weight per frame
        std::vector<float> shown;
        float endAim;
    };
    std::vector<Case> cases;
    auto ramp = [](std::vector<float>& v, float from, float to, float seconds) {
        const int n = static_cast<int>(std::ceil(seconds * 60.0f));
        for (int i = 1; i <= n; ++i)
            v.push_back(from + (to - from) * static_cast<float>(i) / static_cast<float>(n));
    };
    auto hold = [](std::vector<float>& v, float value, float seconds) {
        for (int i = 0; i < static_cast<int>(std::ceil(seconds * 60.0f)); ++i)
            v.push_back(value);
    };
    {
        Case c{"hip fire raise 0.1 s, hold, lower 0.1 s", {}, {}, 0.0f};
        hold(c.aim, 0.0f, 0.2f), ramp(c.aim, 0.0f, 1.0f, 0.1f), hold(c.aim, 1.0f, 1.0f), ramp(c.aim, 1.0f, 0.0f, 0.1f), hold(c.aim, 0.0f, 1.0f);
        cases.push_back(c);
    }
    {
        Case c{"aim raise 0.2 s, hold", {}, {}, 1.0f};
        hold(c.aim, 0.0f, 0.2f), ramp(c.aim, 0.0f, 1.0f, 0.2f), hold(c.aim, 1.0f, 1.0f);
        cases.push_back(c);
    }
    {
        Case c{"five hip-fire taps (0.1 s up, 0.05 s, 0.1 s down, 0.1 s)", {}, {}, 0.0f};
        hold(c.aim, 0.0f, 0.1f);
        for (int t = 0; t < 5; ++t)
            ramp(c.aim, 0.0f, 1.0f, 0.1f), hold(c.aim, 1.0f, 0.05f), ramp(c.aim, 1.0f, 0.0f, 0.1f), hold(c.aim, 0.0f, 0.1f);
        hold(c.aim, 0.0f, 1.0f);
        cases.push_back(c);
    }
    {
        Case c{"reload begun in the stance (envelope 0.3 s in, 0.5 s, 0.3 s out)", {}, {}, 1.0f};
        hold(c.aim, 1.0f, 2.6f);
        hold(c.shown, 1.0f, 0.8f), ramp(c.shown, 1.0f, 0.0f, 0.3f), hold(c.shown, 0.0f, 0.5f), ramp(c.shown, 0.0f, 1.0f, 0.3f);
        cases.push_back(c);
    }
    for (const Case& c : cases)
    {
        dm::Follow f;
        std::vector<BoneTransform> p;
        frame(f, c.aim[0], c.shown.empty() ? 1.0f : c.shown[0], p);
        // the follower starts on the mirror with its weight risen (dual wield on before the case)
        for (int i = 0; i < 120; ++i)
            frame(f, c.aim[0], c.shown.empty() ? 1.0f : c.shown[0], p);
        math::Vec3 prev = leftHand(p), prevR = rightHand(p);
        double worst = 0.0, worstR = 0.0;
        bool finite = true;
        for (size_t i = 1; i < c.aim.size(); ++i)
        {
            const float shown = c.shown.empty() ? 1.0f : (i < c.shown.size() ? c.shown[i] : c.shown.back());
            frame(f, c.aim[i], shown, p);
            const math::Vec3 h = leftHand(p);
            finite &= math::IsFinite(h);
            worst = std::fmax(worst, static_cast<double>(Len(h - prev)));
            prev = h;
            const math::Vec3 hr = rightHand(p);
            worstR = std::fmax(worstR, static_cast<double>(Len(hr - prevR)));
            prevR = hr;
        }
        const double endErr = 100.0 * Len(prev - leftHand(final(c.endAim)));
        std::printf("[%s] transition %-62s largest step (chest frame) left hand %.2f cm, right hand %.2f cm, end %.4f cm from the mirror\n",
                    aTag, c.name, 100.0 * worst, 100.0 * worstR, endErr);
        Check(finite, std::string(aTag) + " " + c.name + ": finite");
        Check(worst <= 0.025, std::string(aTag) + " " + c.name + ": left hand <= 2.5 cm per frame at 60 Hz");
        Check(endErr <= 0.01, std::string(aTag) + " " + c.name + ": ends on the mirror of the target stance");
    }
    // blend frames for the anatomy gate: the aiming weight in steps of 0.25 (twin turn x mirror)
    for (int k = 0; k <= 4; ++k)
    {
        DumpPose(aDump, aTag, "blend_aim_" + std::to_string(k * 25), final(0.25f * static_cast<float>(k)));
        DumpPose(aDump, aTag, "blend_aim_" + std::to_string(k * 25) + " [input]", Lerped(aReady, aAim, 0.25f * static_cast<float>(k)));
    }
    // weight 0 on a fresh follower (a single weapon) leaves the pose bit-identical; a risen follower given weight 0 lowers its
    // weight within the hand limit and then leaves the pose bit-identical and resets
    {
        const size_t bytes = sizeof(BoneTransform) * aAim.size();
        dm::Follow f;
        std::vector<BoneTransform> p = aAim;
        const std::vector<BoneTransform> p0 = p;
        Check(!dm::ApplyFollow(aM, f, p.data(), 0.0f, dt) && !f.valid && std::memcmp(p.data(), p0.data(), bytes) == 0,
              std::string(aTag) + ": follower: weight 0, nothing shown: pose bit-identical");
        for (int i = 0; i < 120; ++i)
        {
            p = p0;
            dm::ApplyFollow(aM, f, p.data(), 1.0f, dt);
        }
        Check(f.w >= 1.0f, std::string(aTag) + ": follower: weight reaches 1");
        int frames = 0;
        math::Vec3 prev = leftHand(p);
        double worst = 0.0;
        bool edited = true;
        while (edited && frames < 600)
        {
            p = p0;
            edited = dm::ApplyFollow(aM, f, p.data(), 0.0f, dt);
            worst = std::fmax(worst, static_cast<double>(Len(leftHand(p) - prev)));
            prev = leftHand(p);
            ++frames;
        }
        std::printf("[%s] follower release: %d frames, largest left hand step %.2f cm\n", aTag, frames, 100.0 * worst);
        Check(!edited && !f.valid && std::memcmp(p.data(), p0.data(), bytes) == 0 && worst <= 0.025 && frames <= 60,
              std::string(aTag) + ": follower: weight 0 releases within 1 s at <= 2.5 cm per frame, then the pose is bit-identical");
    }
}

static void TestRig(const char* aDb, const char* aOv, uint64_t aHash, const char* aTag, std::FILE* aDump)
{
    auto db = std::make_unique<motion::MotionDatabase>();
    std::string err;
    const bool dbOk = db->LoadFromFile(aDb, aHash, err);
    Check(dbOk, std::string(aTag) + ": load " + aDb + (dbOk ? "" : " (" + err + ")"));
    auto ovf = std::make_unique<ov::OverlayFile>();
    const bool ovOk = ovf->LoadFromFile(aOv, aHash, err);
    Check(ovOk, std::string(aTag) + ": load " + aOv + (ovOk ? "" : " (" + err + ")"));
    if (!dbOk || !ovOk)
        return;
    RigData r;
    const uint32_t n = db->NumBones();
    for (uint32_t i = 0; i < n; ++i)
    {
        r.names.emplace_back(db->BoneName(i));
        r.par.push_back(db->BoneParent(i));
        const auto& rr = db->RigRest(i);
        r.rest.push_back(BoneTransform{math::Vec3{rr.px, rr.py, rr.pz}, math::QuatNormalize(math::Quat{rr.qx, rr.qy, rr.qz, rr.qw})});
    }
    // 1) build
    const dm::Mirror mir = BuildOf(r, r.rest);
    std::printf("[%s] bones %u, mirror %s, lateral (%.5f %.5f %.5f), mid (%.4f %.4f %.4f), rest residue %.5f\n", aTag, n,
                mir.ok ? "on" : "off", mir.lateral.x, mir.lateral.y, mir.lateral.z, mir.mid.x, mir.mid.y, mir.mid.z, mir.restAsymmetry);
    Check(mir.Valid(), std::string(aTag) + ": mirror built on the rest pose");
    if (!mir.Valid())
        return;
    for (int k = 0; k < dm::kPairs; ++k)
    {
        std::printf("[%s] N %-12s", aTag, std::string(dm::kLeftNames[k]).c_str());
        for (int i = 0; i < 3; ++i)
            std::printf(" [% .3f % .3f % .3f]", mir.n[k].m[i][0], mir.n[k].m[i][1], mir.n[k].m[i][2]);
        std::printf("\n");
    }
    // overlay bone order -> database order by name
    std::vector<int> ovOf(n, -1);
    for (int i = 0; i < ovf->NumBones(); ++i)
    {
        const int d = r.Find(std::string(ovf->BoneName(i)));
        if (d >= 0)
            ovOf[d] = i;
    }
    for (int k = 0; k < dm::kPairs; ++k)
        Check(ovOf[mir.left[k]] >= 0 && ovOf[mir.right[k]] >= 0, std::string(aTag) + ": overlay has the arm bones");
    const int spine = r.Find("Spine"), spine1 = r.Find("Spine1");
    Check(spine >= 0 && spine1 >= 0, std::string(aTag) + ": spine bones");
    double worstCm = 0.0, worstDeg = 0.0;
    int poses = 0;
    std::vector<BoneTransform> blendPose, stancePose;
    for (int pi = 0; pi < ovf->NumPoses(); ++pi)
    {
        const std::string name(ovf->PoseName(pi));
        if (name.rfind("Pose_Pistol1H", 0) != 0)
            continue;
        auto p = std::make_unique<ov::Pose>();
        ovf->FramePose(ovf->PoseAt(pi).firstFrame, *p);
        ov::Curves c;
        ovf->FrameCurves(pi, ovf->PoseAt(pi).firstFrame, c);
        std::printf("[%s] %-32s Layering_Arm_R %.2f (present %d)\n", aTag, name.c_str(), c[ov::kArmR], (c.present >> ov::kArmR) & 1u);
        std::vector<BoneTransform> in = r.rest;
        for (uint32_t i = 0; i < n; ++i)
            if (ovOf[i] >= 0)
                in[i] = BoneTransform{p->b[ovOf[i]].t, p->b[ovOf[i]].r};
        // 4) weight 0 / never-on ramp / single weapon: bit-identical
        {
            std::vector<BoneTransform> z = in;
            Check(!dm::Apply(mir, z.data(), 0.0f), std::string(aTag) + " " + name + ": weight 0 does not edit");
            dm::Ramp ramp;
            float w = 0.0f;
            for (int f = 0; f < 30; ++f)
                w = dm::ShownWeight(ramp.Step(ov::DualWieldMirrorWanted(ov::PoseOverlay::Pistol1H, false), 1.0f / 60.0f), 1.0f, 0.0f);
            dm::Apply(mir, z.data(), w);
            Check(w == 0.0f && std::memcmp(z.data(), in.data(), sizeof(BoneTransform) * n) == 0,
                  std::string(aTag) + " " + name + ": single weapon: pose bit-identical");
        }
        CheckMirrored(aTag, name, r, mir, in, worstCm, worstDeg);
        // chest turned: the mirror is in the chest frame, independent of the spine above the hips
        std::vector<BoneTransform> turned = in;
        turned[spine].q = math::QuatNormalize(math::QuatMul(turned[spine].q, AxisAngle({0.0f, 0.0f, 1.0f}, 35.0f)));
        turned[spine1].q = math::QuatNormalize(math::QuatMul(turned[spine1].q, AxisAngle({1.0f, 0.0f, 0.0f}, 12.0f)));
        CheckMirrored(aTag, name + " (chest turned)", r, mir, turned, worstCm, worstDeg);
        if (blendPose.empty() && name.find("Ready") != std::string::npos)
            blendPose = in;
        if (stancePose.empty() && name.find("Stand_Aim") != std::string::npos)
            stancePose = in;
        ++poses;
    }
    Check(poses >= 4, std::string(aTag) + ": one-handed weapon poses found (" + std::to_string(poses) + ")");
    // 7) twin stance over the aim poses x the aim offset sweep (mesh-space additive, as the aim offset applies it) x a turned chest
    {
        TwinStats ts;
        for (const char* aimName : {"Pose_Pistol1H_Stand_Aim_Idle", "Pose_Pistol1H_Crouch_Aim"})
        {
            const int pi = ovf->FindPose(aimName);
            Check(pi >= 0, std::string(aTag) + ": pose " + aimName);
            if (pi < 0)
                continue;
            const bool crouch = std::string(aimName).find("Crouch") != std::string::npos;
            const int sweep = ovf->FindDelta(crouch ? "AO_Pistol1H_Crouch_Sweep" : "AO_Pistol1H_Stand_Sweep");
            Check(sweep >= 0, std::string(aTag) + ": aim offset sweep of " + aimName);
            const int frames = sweep >= 0 ? static_cast<int>(ovf->DeltaAt(sweep).numFrames) : 1;
            for (int k = 0; k <= 8; ++k)
            {
                auto p = std::make_unique<ov::Pose>();
                ovf->FramePose(ovf->PoseAt(pi).firstFrame, *p);
                const int fr = sweep >= 0 ? (k * (frames - 1)) / 8 : 0;
                if (sweep >= 0)
                {
                    auto add = std::make_unique<ov::Additive>();
                    ovf->DeltaFrame(sweep, fr, true, *add);
                    ov::ApplyAdditive(ovf->Skel(), *p, *add, 1.0f);
                }
                std::vector<BoneTransform> in = r.rest;
                for (uint32_t i = 0; i < n; ++i)
                    if (ovOf[i] >= 0)
                        in[i] = BoneTransform{p->b[ovOf[i]].t, p->b[ovOf[i]].r};
                const std::string tag = std::string(aimName) + " sweep " + std::to_string(fr) + "/" + std::to_string(frames - 1);
                CheckTwin(aTag, tag, r, mir, in, ts, aDump);
                if (k % 4 == 0)
                    for (float yaw : {-35.0f, 35.0f})
                    {
                        std::vector<BoneTransform> turned = in;
                        turned[spine].q = math::QuatNormalize(math::QuatMul(turned[spine].q, AxisAngle({0.0f, 0.0f, 1.0f}, yaw)));
                        CheckTwin(aTag, tag + " chest " + std::to_string(static_cast<int>(yaw)), r, mir, turned, ts, aDump);
                    }
            }
        }
        std::printf("[%s] twin: %d poses, turn up to %.1f deg, out of plane <= %.4f deg, barrels <= %.4f deg apart, elevation change <= %.2e, "
                    "hands %.1f..%.1f cm, weapons >= %.1f cm\n", aTag, ts.poses, ts.maxTurnDeg, ts.worstPlaneDeg, ts.worstParallelDeg,
                    ts.worstElev, ts.minHands, ts.maxHands, ts.minGuns);
        Check(ts.poses >= 20, std::string(aTag) + ": twin stance poses measured");
    }
    std::printf("[%s] %d poses: worst mirror error %.5f cm, %.5f deg\n", aTag, poses, worstCm, worstDeg);
    // 4b) the gate never asks for the mirror with a single weapon, whatever the overlay
    for (int o = 0; o < static_cast<int>(ov::PoseOverlay::Count); ++o)
        Check(!ov::DualWieldMirrorWanted(ov::DualWieldOverlay(static_cast<ov::PoseOverlay>(o), false), false),
              std::string(aTag) + ": no mirror without dual wield, overlay " + std::to_string(o));
    Check(ov::DualWieldMirrorWanted(ov::DualWieldOverlay(ov::PoseOverlay::Pistol2H, true), true) &&
              ov::DualWieldMirrorWanted(ov::DualWieldOverlay(ov::PoseOverlay::Pistol1H, true), true) &&
              !ov::DualWieldMirrorWanted(ov::DualWieldOverlay(ov::PoseOverlay::Default, true), true),
          std::string(aTag) + ": dual wield gate (pistol overlays on, unarmed off)");
    // 5) blend
    if (!blendPose.empty())
        TestBlend(aTag, r, mir, blendPose);
    // 7b) twin stance transitions
    Check(!blendPose.empty() && !stancePose.empty(), std::string(aTag) + ": ready and standing aim poses found for the transitions");
    if (!blendPose.empty() && !stancePose.empty())
        TestTwinTransitions(aTag, r, mir, blendPose, stancePose, aDump);
    // gap in the stepping: still wanted -> no drop (no pop); no longer wanted -> exactly 0, pose bit-identical
    {
        dm::Ramp g;
        for (int f = 0; f < 60; ++f)
            g.Step(true, 1.0f / 60.0f);
        const float kept = g.Step(true, 1.0f / 60.0f, true);
        dm::Ramp h = g;
        const float gone = h.Step(false, 1.0f / 60.0f, true);
        std::vector<BoneTransform> z = blendPose.empty() ? r.rest : blendPose;
        const std::vector<BoneTransform> z0 = z;
        dm::Apply(mir, z.data(), dm::ShownWeight(gone, 1.0f, 0.0f));
        Check(kept == 1.0f, std::string(aTag) + ": gap while wanted keeps the mirror (" + std::to_string(kept) + ")");
        Check(gone == 0.0f && std::memcmp(z.data(), z0.data(), sizeof(BoneTransform) * n) == 0,
              std::string(aTag) + ": gap after dual wield ended: weight 0, pose bit-identical");
    }
    Check(dm::ShownWeight(1.0f, 1.0f, 1.0f) == 0.0f && dm::ShownWeight(1.0f, 0.0f, 0.0f) == 0.0f && dm::ShownWeight(1.0f, 1.0f, 0.0f) == 1.0f,
          std::string(aTag) + ": shown weight: reload envelope and right arm layer cut the mirror");
    // 6) fail closed
    {
        std::vector<BoneTransform> bad = r.rest;
        const int la = mir.left[1];
        bad[la].q = math::QuatNormalize(math::QuatMul(bad[la].q, AxisAngle({0.3f, 1.0f, 0.2f}, 10.0f)));
        const dm::Mirror m2 = BuildOf(r, bad);
        Check(!m2.Valid(), std::string(aTag) + ": asymmetric rest pose -> no mirror (residue " + std::to_string(m2.restAsymmetry) + ")");
        std::vector<BoneTransform> z = blendPose.empty() ? r.rest : blendPose;
        const std::vector<BoneTransform> z0 = z;
        Check(!dm::Apply(m2, z.data(), 1.0f) && std::memcmp(z.data(), z0.data(), sizeof(BoneTransform) * n) == 0,
              std::string(aTag) + ": no mirror -> Apply leaves the pose");
        RigData miss = r;
        miss.names[mir.left[4]] = "WeaponLeftMissing";
        Check(!BuildOf(miss, r.rest).Valid(), std::string(aTag) + ": missing WeaponLeft -> no mirror");
    }
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("usage: utpp_dualwield_mirror_test <woman.utppmm> <woman.utppov> <man.utppmm> <man.utppov> [pose dump file]\n");
        return 2;
    }
    std::FILE* dump = nullptr;
    if (argc >= 6 && fopen_s(&dump, argv[5], "w") != 0)
        dump = nullptr;
    TestRig(argv[1], argv[2], motion::kRigIdentityWoman, "woman", dump);
    TestRig(argv[3], argv[4], motion::kRigIdentityMan, "man", dump);
    if (dump != nullptr)
        std::fclose(dump);
    std::printf("dualwield mirror test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

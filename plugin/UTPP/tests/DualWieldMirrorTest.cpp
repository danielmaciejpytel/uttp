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
//  7) the stance hold: the right arm goes into the aim stance and back: the left hand stays on the mirrored ready pose (<= 1 mm) and
//     returns without a step above 2.5 cm per frame; weight 0 clears the hold; a stance with nothing held leaves the pose alone and
//     the mirror fades in afterwards.
#include <cmath>
#include <cstdio>
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

// Stance hold: the right arm goes from the ready pose into the aim stance and back; the left arm keeps the mirrored ready pose.
static void TestHold(const char* aTag, const RigData& aR, const dm::Mirror& aM, const std::vector<BoneTransform>& aReady,
                     const std::vector<BoneTransform>& aStance)
{
    const float dt = 1.0f / 60.0f;
    const size_t bytes = sizeof(BoneTransform) * aReady.size();
    auto rightAt = [&](float aU) {
        std::vector<BoneTransform> p = aReady;
        for (int k = 0; k < dm::kPairs; ++k)
        {
            const int i = aM.right[k];
            p[i].q = math::QuatNlerpShortest(aReady[i].q, aStance[i].q, aU);
            p[i].p = math::Lerp(aReady[i].p, aStance[i].p, aU);
        }
        return p;
    };
    auto leftHand = [&](const std::vector<BoneTransform>& aP) { return Fk(aR.par, aP).p[aM.left[3]]; };
    dm::Held held;
    {
        std::vector<BoneTransform> a = aReady, b = aReady;
        dm::Apply(aM, a.data(), 1.0f);
        dm::ApplyOrHold(aM, held, b.data(), 1.0f, false, dt);
        Check(std::memcmp(a.data(), b.data(), bytes) == 0 && held.valid, std::string(aTag) + ": hold: outside the stance it is the plain mirror");
    }
    std::vector<BoneTransform> ready = aReady;
    dm::Apply(aM, ready.data(), 1.0f);
    const math::Vec3 readyHand = leftHand(ready);
    std::vector<BoneTransform> plain = rightAt(1.0f);
    dm::Apply(aM, plain.data(), 1.0f);
    const double plainAway = Len(leftHand(plain) - readyHand);
    double farthest = 0.0, worstStep = 0.0;
    math::Vec3 prev = readyHand;
    bool finite = true;
    for (int f = 0; f < 60; ++f) // into the stance over 0.25 s, then held
    {
        std::vector<BoneTransform> p = rightAt(std::fmin(1.0f, static_cast<float>(f + 1) * dt / 0.25f));
        dm::ApplyOrHold(aM, held, p.data(), 1.0f, true, dt);
        const math::Vec3 h = leftHand(p);
        finite &= math::IsFinite(h);
        farthest = std::fmax(farthest, static_cast<double>(Len(h - readyHand)));
        worstStep = std::fmax(worstStep, static_cast<double>(Len(h - prev)));
        prev = h;
    }
    double backStep = 0.0;
    math::Vec3 last = prev;
    for (int f = 0; f < 60; ++f) // out of the stance: the right arm returns over 0.25 s, the hold lets go over dm::kBlendSeconds
    {
        std::vector<BoneTransform> p = rightAt(std::fmax(0.0f, 1.0f - static_cast<float>(f + 1) * dt / 0.25f));
        dm::ApplyOrHold(aM, held, p.data(), 1.0f, false, dt);
        last = leftHand(p);
        finite &= math::IsFinite(last);
        backStep = std::fmax(backStep, static_cast<double>(Len(last - prev)));
        prev = last;
    }
    std::printf("[%s] hold: plain mirror of the stance is %.1f cm away; held arm at most %.2f cm away, largest step in %.2f cm, out %.2f cm\n",
                aTag, 100.0 * plainAway, 100.0 * farthest, 100.0 * worstStep, 100.0 * backStep);
    Check(finite, std::string(aTag) + ": hold finite");
    Check(plainAway > 0.20, std::string(aTag) + ": hold: the plain mirror of the aim stance moves the left hand far away (the case)");
    Check(farthest <= 0.001, std::string(aTag) + ": hold: in the stance the left hand stays on the mirrored ready pose (<= 1 mm)");
    Check(backStep <= 0.025, std::string(aTag) + ": hold: leaving the stance moves the left hand <= 2.5 cm per frame at 60 Hz");
    Check(Len(last - readyHand) <= 1e-4f && !(held.u > 0.0f), std::string(aTag) + ": hold: after the stance the live mirror is back");
    {
        std::vector<BoneTransform> z = rightAt(1.0f);
        const std::vector<BoneTransform> z0 = z;
        Check(!dm::ApplyOrHold(aM, held, z.data(), 0.0f, true, dt) && std::memcmp(z.data(), z0.data(), bytes) == 0 && !held.valid,
              std::string(aTag) + ": hold: weight 0 leaves the pose and clears the hold");
        dm::Held fresh;
        Check(!dm::ApplyOrHold(aM, fresh, z.data(), 1.0f, true, dt) && std::memcmp(z.data(), z0.data(), bytes) == 0,
              std::string(aTag) + ": hold: two weapons drawn inside the stance: nothing held, pose left as it is");
    }
    {
        // Nothing held in the stance (a reload that ended in the stance): the ramp restarts from 0 and the mirror fades in once
        // the stance is over.
        dm::Ramp ramp;
        dm::Held none;
        for (int f = 0; f < 60; ++f)
            ramp.Step(true, dt); // the mirror was fully on before
        std::vector<BoneTransform> p = rightAt(1.0f);
        const std::vector<BoneTransform> p0 = p;
        const float r = dm::StepRamp(ramp, none, true, true, dt);
        const bool touched = dm::ApplyOrHold(aM, none, p.data(), dm::ShownWeight(r, 1.0f, 0.0f), true, dt);
        Check(r == 0.0f && !(ramp.u > 0.0f) && !touched && std::memcmp(p.data(), p0.data(), bytes) == 0,
              std::string(aTag) + ": hold: nothing held in the stance: the ramp restarts from 0, pose left as it is");
        math::Vec3 prevHand = leftHand(aReady);
        double fadeStep = 0.0;
        bool full = false;
        for (int f = 0; f < 60; ++f) // the stance is over, the right arm is in the ready pose again
        {
            std::vector<BoneTransform> q = aReady;
            const float w = dm::ShownWeight(dm::StepRamp(ramp, none, true, false, dt), 1.0f, 0.0f);
            dm::ApplyOrHold(aM, none, q.data(), w, false, dt);
            const math::Vec3 h = leftHand(q);
            fadeStep = std::fmax(fadeStep, static_cast<double>(Len(h - prevHand)));
            prevHand = h;
            full |= w >= 1.0f && Len(h - readyHand) <= 1e-4f;
        }
        std::printf("[%s] hold: fade in after a stance with nothing held: largest step %.2f cm\n", aTag, 100.0 * fadeStep);
        Check(fadeStep <= 0.025 && full,
              std::string(aTag) + ": hold: after such a stance the mirror fades in (<= 2.5 cm per frame) up to the full mirror");
    }
}

static void TestRig(const char* aDb, const char* aOv, uint64_t aHash, const char* aTag)
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
    // 5b) stance hold
    Check(!blendPose.empty() && !stancePose.empty(), std::string(aTag) + ": ready and standing aim poses found for the hold test");
    if (!blendPose.empty() && !stancePose.empty())
        TestHold(aTag, r, mir, blendPose, stancePose);
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
        std::printf("usage: utpp_dualwield_mirror_test <woman.utppmm> <woman.utppov> <man.utppmm> <man.utppov>\n");
        return 2;
    }
    TestRig(argv[1], argv[2], motion::kRigIdentityWoman, "woman");
    TestRig(argv[3], argv[4], motion::kRigIdentityMan, "man");
    std::printf("dualwield mirror test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

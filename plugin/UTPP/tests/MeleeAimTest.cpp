// Offline tests of the melee aim tilt core (src/mm/MeleeAim.hpp), not shipped.
// Usage: utpp_melee_aim_test   Exit code 0 = all checks passed.
// Rig: the real player rig 0..28 (names + parents from the player skeleton listing, as in
// tests/AimOffsetTest.cpp) with plausible bone offsets and non-trivial local rotations. Covers: rig resolution (fail closed),
// target angles (level = 0, clamps up / down / yaw, facing fade, too near / non-finite), the tilt on the records (level aim =
// bitwise identity, distribution 25/30/30/15 %, composition = Q at the neck, 85 % below Spine3, joint positions and bone lengths
// kept, untouched bones outside the chain, per-joint cap, fail closed), the smoothing (time constant, cap, invalid -> 0) and the
// weight (melee fade x game share of the upper body). Round 3: the target sources (melee target, crosshair hit within reach,
// camera ray with the neutral framing pitch removed).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/MeleeAim.hpp"

using namespace utpp::mm;
using namespace utpp::mm::meleeaim;
using math::Quat;
using math::Vec3;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c, d);
    return buf;
}

static constexpr float kRad2Deg = 57.2957795f;

// ------------------------------------------------------------------------------------------------ the player rig 0..28

static const char* kNames[] = {"Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1",
                               "LeftLeg", "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder",
                               "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head",
                               "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight"};
static const int16_t kParents[] = {-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24};
static constexpr uint32_t kN = 29;
enum : uint32_t
{
    kHips = 2, kSpine1 = 7, kSpine2 = 10, kSpine3 = 13, kNeck = 16, kNeck1 = 19, kHead = 22, kRightArm = 18, kRightHand = 24,
    kWeaponRight = 28, kLeftFoot = 11
};

struct Pose
{
    std::vector<uint8_t> bytes;
};

static void SetLocal(Pose& aPose, uint32_t aIndex, Vec3 aP, Quat aQ)
{
    float* r = reinterpret_cast<float*>(aPose.bytes.data() + size_t(aIndex) * integ::kGameBoneStride);
    r[0] = aP.x, r[1] = aP.y, r[2] = aP.z, r[3] = 0.0f;
    r[4] = aQ.x, r[5] = aQ.y, r[6] = aQ.z, r[7] = aQ.w;
    r[8] = r[9] = r[10] = r[11] = 1.0f;
}

static Quat Axis(Vec3 aAxis, float aDeg) { return aim::AxisAngle(aAxis, aDeg / kRad2Deg); }
static const Vec3 kX{1, 0, 0}, kY{0, 1, 0}, kZ{0, 0, 1};

// A standing pose: Z up, forward +Y; a slight forward lean in the spine, the arms raised in a guard, a twist in Spine3.
static Pose MakePose()
{
    Pose p;
    p.bytes.assign(size_t(kN) * integ::kGameBoneStride, 0);
    const Quat id{};
    const Vec3 off[kN] = {
        {0, 0, 0},        {0, 0, 0},       {0, 0, 1.0f},     {0, 0, 0},         {0, 0, 0.10f},    {-0.10f, 0, -0.05f},
        {0.10f, 0, -0.05f}, {0, 0, 0.12f}, {0, 0, -0.45f},   {0, 0, -0.45f},    {0, 0, 0.12f},    {0, 0, -0.42f},
        {0, 0, -0.42f},   {0, 0, 0.12f},   {-0.04f, 0, 0.16f}, {0.04f, 0, 0.16f}, {0, 0.01f, 0.18f}, {-0.13f, 0, 0},
        {0.13f, 0, 0},    {0, 0, 0.05f},   {-0.28f, 0, 0},   {0.28f, 0, 0},     {0, 0, 0.08f},    {-0.25f, 0, 0},
        {0.25f, 0, 0},    {-0.08f, 0, 0},  {-0.05f, 0.03f, 0}, {0.08f, 0, 0},   {0.05f, 0.03f, 0}};
    for (uint32_t i = 0; i < kN; ++i)
        SetLocal(p, i, off[i], id);
    SetLocal(p, kSpine1, off[kSpine1], Axis(kX, -6.0f));       // forward lean
    SetLocal(p, kSpine2, off[kSpine2], Axis(kX, -4.0f));
    SetLocal(p, kSpine3, off[kSpine3], math::QuatNormalize(math::QuatMul(Axis(kZ, 8.0f), Axis(kX, -3.0f)))); // guard twist
    SetLocal(p, kRightArm, off[kRightArm], Axis(kY, -60.0f));
    SetLocal(p, 17, off[17], Axis(kY, 60.0f));
    SetLocal(p, 21, off[21], Axis(kZ, 100.0f));
    SetLocal(p, 20, off[20], Axis(kZ, -100.0f));
    SetLocal(p, kHead, off[kHead], Axis(kX, 5.0f));
    return p;
}

static Rig RealRig()
{
    std::vector<uint64_t> h;
    for (const char* n : kNames)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> par(kParents, kParents + kN);
    return ResolveRig(h, par);
}

static void Model(const Pose& aPose, aim::Xf* aOut) { aim::ComputeModel(aPose.bytes.data(), kN, kParents, kN, aOut); }

static float QuatAngleDeg(const Quat& a, const Quat& b)
{
    const float d = std::fabs(math::QuatDot(math::QuatNormalize(a), math::QuatNormalize(b)));
    return 2.0f * std::acos(d > 1.0f ? 1.0f : d) * kRad2Deg;
}

static Quat LocalRot(const Pose& aPose, uint32_t aIndex) { return aim::ReadLocal(aPose.bytes.data(), aIndex).q; }

// ------------------------------------------------------------------------------------------------ tests

static void TestRig()
{
    const Rig r = RealRig();
    Expect("ResolveRig: player rig -> Spine1 7, Spine2 10, Spine3 13, Neck 16, fk 17",
           r.ok == 1 && r.joint[0] == 7 && r.joint[1] == 10 && r.joint[2] == 13 && r.joint[3] == 16 && r.fkCount == 17 && r.Chest() == 13);
    std::vector<uint64_t> h;
    for (const char* n : kNames)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> par(kParents, kParents + kN);
    std::vector<uint64_t> h2 = h;
    h2[kNeck] = integ::CNameHash("NeckX");
    Expect("ResolveRig: missing Neck -> not ok", ResolveRig(h2, par).ok == 0);
    std::vector<int16_t> p2 = par;
    p2[kSpine2] = 4; // Spine2 under Spine, not Spine1
    Expect("ResolveRig: Spine2 not a child of Spine1 -> not ok", ResolveRig(h, p2).ok == 0);
    std::vector<int16_t> p3 = par;
    p3[kNeck] = 10; // Neck under Spine2 (not below Spine3)
    Expect("ResolveRig: Neck not below Spine3 -> not ok", ResolveRig(h, p3).ok == 0);
    std::vector<int16_t> p4 = par;
    p4[5] = 9; // parent after child
    Expect("ResolveRig: non-topological parents -> not ok", ResolveRig(h, p4).ok == 0);
    std::vector<int16_t> p5(kParents, kParents + kN - 1);
    Expect("ResolveRig: names / parents size mismatch -> not ok", ResolveRig(h, p5).ok == 0);
    Expect("ResolveRig: empty -> not ok", ResolveRig({}, {}).ok == 0);
}

static Params On() { return Params{kDefaultMaxUpDeg, kDefaultMaxDownDeg, kDefaultSmoothHz}; }

static Vec3 Dir(float aYawDeg, float aPitchDeg, float aDist)
{
    const float y = aYawDeg / kRad2Deg, p = aPitchDeg / kRad2Deg;
    return Vec3{-std::sin(y) * std::cos(p) * aDist, std::cos(y) * std::cos(p) * aDist, std::sin(p) * aDist};
}

static void TestTarget()
{
    const Vec3 chest{0.02f, 0.05f, 1.46f};
    const Params pr = On();
    Target t = TargetAngles(chest, chest + Dir(0, 0, 10), pr);
    Expect("target: level, straight ahead -> pitch 0, yaw 0, valid, no flags", t.valid && t.pitch == 0.0f && std::fabs(t.yaw) < 1e-6f && t.flags == 0,
           Fmt("p %g y %g flags %g", t.pitch * kRad2Deg, t.yaw * kRad2Deg, t.flags));
    t = TargetAngles(chest, chest + Dir(0, 12, 8), pr);
    Expect("target: 12 deg up -> pitch 12, no clamp", t.valid && std::fabs(t.pitch * kRad2Deg - 12.0f) < 1e-3f && t.flags == 0);
    t = TargetAngles(chest, chest + Dir(0, 40, 8), pr);
    Expect("target: 40 deg up -> clamped to maxUp 20, kFlagClampUp",
           std::fabs(t.pitch * kRad2Deg - 20.0f) < 1e-3f && (t.flags & kFlagClampUp) && std::fabs(t.rawPitch * kRad2Deg - 40.0f) < 1e-3f);
    t = TargetAngles(chest, chest + Dir(0, -50, 3), pr);
    Expect("target: 50 deg down -> clamped to maxDown 35, kFlagClampDown", std::fabs(t.pitch * kRad2Deg + 35.0f) < 1e-3f && (t.flags & kFlagClampDown));
    t = TargetAngles(chest, chest + Dir(-30, -20, 3), pr);
    Expect("target: 30 deg right, 20 down -> yaw -25 (clamped), pitch -20",
           std::fabs(t.yaw * kRad2Deg + 25.0f) < 1e-3f && std::fabs(t.pitch * kRad2Deg + 20.0f) < 1e-3f && (t.flags & kFlagClampYaw) &&
               !(t.flags & kFlagFacing));
    t = TargetAngles(chest, chest + Dir(10, 0, 5), pr);
    Expect("target: 10 deg left -> yaw +10 (left positive)", std::fabs(t.yaw * kRad2Deg - 10.0f) < 1e-3f);
    t = TargetAngles(chest, chest + Dir(112.5f, 15, 5), pr);
    Expect("target: aim 112.5 deg to the side -> facing fade 0.5 (pitch 7.5, yaw 12.5), kFlagFacing",
           std::fabs(t.pitch * kRad2Deg - 7.5f) < 1e-2f && std::fabs(t.yaw * kRad2Deg - 12.5f) < 1e-2f && (t.flags & kFlagFacing),
           Fmt("p %g y %g", t.pitch * kRad2Deg, t.yaw * kRad2Deg));
    t = TargetAngles(chest, chest + Dir(150, -30, 5), pr);
    Expect("target: aim behind V (150 deg) -> no tilt (0 / 0), still valid", t.valid && t.pitch == 0.0f && t.yaw == 0.0f && (t.flags & kFlagFacing));
    t = TargetAngles(chest, chest + Vec3{0, 0.3f, 0}, pr);
    Expect("target: aim point 0.3 m from the chest -> invalid, kFlagNear", !t.valid && (t.flags & kFlagNear) && t.pitch == 0.0f);
    t = TargetAngles(chest, Vec3{NAN, 1, 1}, pr);
    Expect("target: non-finite aim point -> invalid", !t.valid && (t.flags & kFlagNear));
    t = TargetAngles(chest, chest + Vec3{0, 0, -5}, pr);
    Expect("target: straight down -> pitch -35 (clamped), yaw 0 (no heading)", std::fabs(t.pitch * kRad2Deg + 35.0f) < 1e-3f && t.yaw == 0.0f);
    Expect("FacingWeight: 0 / 90 / 112.5 / 135 / 180 deg -> 1 / 1 / 0.5 / 0 / 0",
           FacingWeight(0) == 1.0f && FacingWeight(90 / kRad2Deg) > 0.9999f && std::fabs(FacingWeight(112.5f / kRad2Deg) - 0.5f) < 1e-4f &&
               FacingWeight(135.01f / kRad2Deg) == 0.0f && FacingWeight(-3.1f) == 0.0f);
}

static void TestTilt()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    aim::Xf m0[kN], m1[kN];
    Model(base, m0);

    // level aim: bitwise identity
    Pose p = base;
    bool capped = true;
    const bool okLevel = ApplyTilt(p.bytes.data(), kN, rig, 0.0f, 0.0f, &capped);
    Expect("tilt: level aim (0 / 0) -> records bitwise unchanged", okLevel && !capped && p.bytes == base.bytes);

    // 20 deg up
    const float pitch = 20.0f / kRad2Deg;
    p = base;
    const bool ok = ApplyTilt(p.bytes.data(), kN, rig, pitch, 0.0f, &capped);
    Model(p, m1);
    const Quat q = TiltRotation(pitch, 0.0f);
    const float shareDeg[4] = {
        QuatAngleDeg(LocalRot(p, kSpine1), LocalRot(base, kSpine1)), QuatAngleDeg(LocalRot(p, kSpine2), LocalRot(base, kSpine2)),
        QuatAngleDeg(LocalRot(p, kSpine3), LocalRot(base, kSpine3)), QuatAngleDeg(LocalRot(p, kNeck), LocalRot(base, kNeck))};
    Expect("tilt: 20 deg up -> local turns 5 / 6 / 6 / 3 deg (25 / 30 / 30 / 15 %)",
           ok && !capped && std::fabs(shareDeg[0] - 5.0f) < 0.02f && std::fabs(shareDeg[1] - 6.0f) < 0.02f && std::fabs(shareDeg[2] - 6.0f) < 0.02f &&
               std::fabs(shareDeg[3] - 3.0f) < 0.02f,
           Fmt("%.3f %.3f %.3f %.3f", shareDeg[0], shareDeg[1], shareDeg[2], shareDeg[3]));
    const Quat neckWant = math::QuatMul(q, m0[kNeck].q);
    const Quat headWant = math::QuatMul(q, m0[kHead].q);
    Expect("tilt: composition -> Neck and Head model rotation = Q * source (Head keeps its own local motion)",
           QuatAngleDeg(m1[kNeck].q, neckWant) < 0.02f && QuatAngleDeg(m1[kHead].q, headWant) < 0.02f &&
               std::memcmp(p.bytes.data() + size_t(kHead) * integ::kGameBoneStride, base.bytes.data() + size_t(kHead) * integ::kGameBoneStride,
                           integ::kGameBoneStride) == 0,
           Fmt("neck %.4f head %.4f", QuatAngleDeg(m1[kNeck].q, neckWant), QuatAngleDeg(m1[kHead].q, headWant)));
    const Quat q85 = TiltRotation(0.85f * pitch, 0.0f);
    Expect("tilt: arms / hands / WeaponRight under Spine3 follow 85 % of Q",
           QuatAngleDeg(m1[kRightHand].q, math::QuatMul(q85, m0[kRightHand].q)) < 0.02f &&
               QuatAngleDeg(m1[kWeaponRight].q, math::QuatMul(q85, m0[kWeaponRight].q)) < 0.02f);
    // the direction: a forward vector of the head is raised
    const Vec3 fwd0 = math::QuatRotate(m0[kHead].q, kY), fwd1 = math::QuatRotate(m1[kHead].q, kY);
    Expect("tilt: positive pitch raises the forward axis of the head (about the model right axis +X)",
           fwd1.z > fwd0.z + 0.2f && std::fabs(fwd1.x - fwd0.x) < 0.05f, Fmt("z %.3f -> %.3f", fwd0.z, fwd1.z));
    // joint positions / bone lengths / bones outside the chain
    bool lengths = true;
    for (uint32_t i = 1; i < kN; ++i)
    {
        const int32_t par = kParents[i];
        if (par < 0)
            continue;
        const float l0 = aim::Length(m0[i].p - m0[par].p), l1 = aim::Length(m1[i].p - m1[par].p);
        lengths = lengths && std::fabs(l0 - l1) < 1e-5f;
    }
    Expect("tilt: every bone length kept (FK)", lengths);
    Expect("tilt: Spine1 joint does not move (rotation about its own joint)", aim::Length(m1[kSpine1].p - m0[kSpine1].p) < 1e-6f);
    bool outside = true;
    static const uint32_t kOutside[] = {0u, 1u, kHips, 3u, 4u, 5u, 6u, 8u, 9u, kLeftFoot, 12u};
    for (uint32_t i : kOutside)
        outside = outside && std::memcmp(p.bytes.data() + size_t(i) * integ::kGameBoneStride, base.bytes.data() + size_t(i) * integ::kGameBoneStride,
                                         integ::kGameBoneStride) == 0;
    Expect("tilt: Root, Trajectory, Hips, Spine and the legs bitwise unchanged", outside);
    bool posKept = true;
    for (uint32_t i = 0; i < kN; ++i)
    {
        const float* a = reinterpret_cast<const float*>(p.bytes.data() + size_t(i) * integ::kGameBoneStride);
        const float* b = reinterpret_cast<const float*>(base.bytes.data() + size_t(i) * integ::kGameBoneStride);
        posKept = posKept && std::memcmp(a, b, 4 * sizeof(float)) == 0 && std::memcmp(a + 8, b + 8, 4 * sizeof(float)) == 0;
    }
    Expect("tilt: only local rotations written (positions, scales bitwise kept)", posKept);

    // yaw only and pitch + yaw
    p = base;
    ApplyTilt(p.bytes.data(), kN, rig, 0.0f, 25.0f / kRad2Deg);
    Model(p, m1);
    Expect("tilt: yaw 25 deg -> Neck model = Rz(25) * source",
           QuatAngleDeg(m1[kNeck].q, math::QuatMul(Axis(kZ, 25.0f), m0[kNeck].q)) < 0.02f);
    p = base;
    const float pp = -30.0f / kRad2Deg, yy = -20.0f / kRad2Deg;
    ApplyTilt(p.bytes.data(), kN, rig, pp, yy, &capped);
    Model(p, m1);
    Expect("tilt: pitch -30 + yaw -20 -> Neck model = Q * source, no joint cap",
           !capped && QuatAngleDeg(m1[kNeck].q, math::QuatMul(TiltRotation(pp, yy), m0[kNeck].q)) < 0.02f);

    // per-joint cap (direct call beyond the clamps: 60 deg -> Spine2 / Spine3 0.3 x 60 = 18 > 15)
    p = base;
    ApplyTilt(p.bytes.data(), kN, rig, 60.0f / kRad2Deg, 0.0f, &capped);
    const float s2 = QuatAngleDeg(LocalRot(p, kSpine2), LocalRot(base, kSpine2)), s1 = QuatAngleDeg(LocalRot(p, kSpine1), LocalRot(base, kSpine1));
    Expect("tilt: per-joint cap 15 deg (Spine2 at 15, Spine1 at 15, capped flag)", capped && std::fabs(s2 - 15.0f) < 0.02f && std::fabs(s1 - 15.0f) < 0.02f,
           Fmt("%.3f %.3f", s1, s2));

    // fail closed
    Rig bad = rig;
    bad.ok = 0;
    p = base;
    Expect("tilt: rig not ok -> false, records unchanged", !ApplyTilt(p.bytes.data(), kN, bad, pitch, 0.0f) && p.bytes == base.bytes);
    Expect("tilt: count below fkCount -> false", !ApplyTilt(p.bytes.data(), 10, rig, pitch, 0.0f) && p.bytes == base.bytes);
    Expect("tilt: NaN pitch -> false", !ApplyTilt(p.bytes.data(), kN, rig, NAN, 0.0f) && p.bytes == base.bytes);
    Expect("tilt: null records -> false", !ApplyTilt(nullptr, kN, rig, pitch, 0.0f));
    float* rec = reinterpret_cast<float*>(p.bytes.data() + size_t(kSpine2) * integ::kGameBoneStride);
    rec[4] = NAN;
    const Pose broken = p;
    Expect("tilt: non-finite source record -> false, records unchanged", !ApplyTilt(p.bytes.data(), kN, rig, pitch, 0.0f) &&
                                                                           std::memcmp(p.bytes.data(), broken.bytes.data(), broken.bytes.size()) == 0);
}

static void TestSmoothing()
{
    const float dt = 1.0f / 60.0f, hz = kDefaultSmoothHz;
    const float target = 10.0f / kRad2Deg;
    Controller c;
    c.Step(target, 0.0f, true, dt, hz);
    const float one = c.Pitch() * kRad2Deg, expectOne = 10.0f * (1.0f - std::exp(-hz * dt));
    Expect("smoothing: one 60 Hz step at 10 Hz -> 10 x (1 - e^-1/6)", std::fabs(one - expectOne) < 1e-3f, Fmt("%.4f vs %.4f", one, expectOne));
    for (int i = 1; i < 6; ++i)
        c.Step(target, 0.0f, true, dt, hz);
    Expect("smoothing: 0.1 s -> 63 % (time constant 1 / smoothHz)", std::fabs(c.Pitch() * kRad2Deg - 6.321f) < 0.01f, Fmt("%.4f", c.Pitch() * kRad2Deg));
    for (int i = 0; i < 60; ++i)
        c.Step(target, 0.0f, true, dt, hz);
    Expect("smoothing: converges (1.1 s -> within 0.001 deg)", std::fabs(c.Pitch() - target) * kRad2Deg < 1e-3f);
    const float before = c.Pitch();
    c.Step(target * 3.0f, 0.0f, true, NAN, hz);
    c.Step(target * 3.0f, 0.0f, true, -1.0f, hz);
    Expect("smoothing: dt NaN / negative -> holds", c.Pitch() == before);
    Controller cap;
    cap.Step(target, 0.0f, true, 1.0f, hz);
    Expect("smoothing: dt 1 s capped at kMaxDt 0.1 s (63 %, no jump)", std::fabs(cap.Pitch() * kRad2Deg - 6.321f) < 0.01f);
    Controller inv = c;
    for (int i = 0; i < 6; ++i)
        inv.Step(target, target, false, dt, hz);
    Expect("smoothing: invalid target fades toward 0 (no snap): 0.1 s -> 37 % left",
           std::fabs(inv.Pitch() / target - 0.3679f) < 0.01f && inv.Yaw() == 0.0f, Fmt("%.4f", inv.Pitch() / target));
    Controller nanc = c;
    nanc.Step(NAN, NAN, true, dt, hz);
    Expect("smoothing: NaN angles -> target 0, state stays finite", std::isfinite(nanc.Pitch()) && nanc.Pitch() < c.Pitch());
    Controller snap;
    snap.Step(target, -target, true, dt, 0.0f);
    Expect("smoothing: smoothHz 0 -> no smoothing (snaps)", snap.Pitch() == target && snap.Yaw() == -target);
    snap.Reset();
    Expect("smoothing: Reset -> 0 / 0", snap.Pitch() == 0.0f && snap.Yaw() == 0.0f);
}

static void TestWeightAndParams()
{
    Expect("weight: melee fade 0 -> 0", TiltWeight(0.0f, 1.0f, 0.0f) == 0.0f);
    Expect("weight: melee 1, MM weight 1, upper body on the game (upperScale 0) -> 1", TiltWeight(1.0f, 1.0f, 0.0f) == 1.0f);
    Expect("weight: upper body on MM (upperScale 1, weight 1) -> 0", TiltWeight(1.0f, 1.0f, 1.0f) == 0.0f);
    Expect("weight: melee 0.5 mid-fade -> 0.5", std::fabs(TiltWeight(0.5f, 1.0f, 0.0f) - 0.5f) < 1e-6f);
    Expect("weight: NaN melee weight -> 0, > 1 clamps", TiltWeight(NAN, 1.0f, 0.0f) == 0.0f && TiltWeight(3.0f, 1.0f, 0.0f) == 1.0f);
    const Params def{};
    Expect("params: DLL default off (0 / 0)", !Enabled(def) && def.maxUpDeg == 0.0f && def.maxDownDeg == 0.0f);
    Expect("params: setting values 20 / 35 / 10 valid and on", ValidParams(20, 35, 10) && Enabled(On()));
    Expect("params: out of range rejected (up 50, down 70, hz 200, negative, NaN)",
           !ValidParams(50, 35, 10) && !ValidParams(20, 70, 10) && !ValidParams(20, 35, 200) && !ValidParams(-1, 35, 10) &&
               !ValidParams(NAN, 35, 10) && !ValidParams(20, 35, INFINITY));
    Expect("params: 0 / 0 valid (= off)", ValidParams(0, 0, 10) && !Enabled(Params{0, 0, 10}));
    // an up-only setting still tilts down by 0 (clamped), i.e. a one-sided limit works
    Target t = TargetAngles(Vec3{0, 0, 1.4f}, Vec3{0, 3, 0}, Params{20, 0, 10});
    Expect("params: maxDown 0 -> a downward aim gives pitch 0 (clamped)", t.valid && t.pitch == 0.0f && (t.flags & kFlagClampDown));
}

// ------------------------------------------------------------------------------------------------ target sources (round 3)

// Review finding 7 (2026-11-09): the tilt aims at what the player targets - the game's melee target (feet vs feet), else the
// crosshair hit within melee reach, else the camera ray with the neutral framing pitch removed. Limits unchanged.
static void TestTargetSources()
{
    Params p;
    p.maxUpDeg = kDefaultMaxUpDeg;
    p.maxDownDeg = kDefaultMaxDownDeg;
    const Vec3 chest{0.0f, 0.05f, 1.30f}; // Spine3 of a standing V (model space: feet at z 0, forward +Y)
    auto deg = [](float r) { return r * kRad2Deg; };
    // camera ray: the pilot rest pitch -15 deg (60 m point) -> no tilt; level -> no tilt; up -> up; further down -> the excess
    auto camAt = [&](float pitchDeg, float yawDeg) {
        const float pr = pitchDeg / kRad2Deg, yr = yawDeg / kRad2Deg;
        return chest + Vec3{-std::sin(yr) * std::cos(pr), std::cos(yr) * std::cos(pr), std::sin(pr)} * 60.0f;
    };
    struct CamCase
    {
        float pitch, want;
    };
    const CamCase cams[] = {{-15.0f, 0.0f}, {-14.0f, 0.0f}, {0.0f, 0.0f}, {-7.0f, 0.0f}, {10.0f, 10.0f}, {-40.0f, -25.0f},
                            {-15.5f, -0.5f}, {0.5f, 0.5f}, {30.0f, 20.0f}, {-60.0f, -35.0f}};
    for (const CamCase& c : cams)
    {
        AimInput ai;
        ai.camValid = true;
        ai.camPoint = camAt(c.pitch, 0.0f);
        TargetSource src = kTgtNone;
        const Target t = SelectTarget(chest, ai, p, &src);
        Expect(Fmt("sources: camera ray pitch %+.1f deg -> tilt %+.1f deg (dead band -15..0, clamps +20 / -35), src=cam", c.pitch, c.want).c_str(),
               src == kTgtCam && t.valid && std::fabs(deg(t.pitch) - c.want) < 0.02f && std::fabs(deg(t.rawPitch) - c.pitch) < 0.02f,
               Fmt("pitch %.3f raw %.3f", deg(t.pitch), deg(t.rawPitch)));
    }
    Expect("sources: CameraPitchTilt continuous at both band edges, NaN -> 0",
           std::fabs(CameraPitchTilt(-15.001f / kRad2Deg)) < 2e-5f && std::fabs(CameraPitchTilt(-14.999f / kRad2Deg)) < 1e-7f &&
               std::fabs(CameraPitchTilt(0.001f / kRad2Deg) - 0.001f / kRad2Deg) < 1e-7f && CameraPitchTilt(-0.001f / kRad2Deg) == 0.0f &&
               CameraPitchTilt(NAN) == 0.0f);
    {
        // the pilot rest yaw (-11 deg, idle orbit): the camera branch removes the pitch framing and (2026-11-12, fists review C5)
        // the +-kCamYawDeadDeg yaw dead zone: -11 -> -6 (was -11 before the dead zone)
        AimInput ai;
        ai.camValid = true;
        ai.camPoint = camAt(-15.0f, -11.0f);
        TargetSource src = kTgtNone;
        const Target t = SelectTarget(chest, ai, p, &src);
        Expect("sources: pilot rest (pitch -15, yaw -11) -> pitch 0, yaw -6 (src=cam, 5 deg yaw dead zone)",
               src == kTgtCam && std::fabs(deg(t.pitch)) < 0.02f && std::fabs(deg(t.yaw) + 6.0f) < 0.05f && std::fabs(deg(t.rawYaw) + 11.0f) < 0.05f,
               Fmt("%.3f / %.3f", deg(t.pitch), deg(t.yaw)));
    }
    {
        // lock: an enemy 1.5 m in front on level ground -> no tilt (feet vs feet; the chest-to-feet angle would be about -40)
        AimInput ai;
        ai.lockValid = true;
        ai.lockFeet = Vec3{0.0f, 1.55f, 0.0f};
        ai.camValid = true;
        ai.camPoint = camAt(-15.0f, -11.0f);
        TargetSource src = kTgtNone;
        Target t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock, enemy 1.5 m ahead on level ground -> pitch 0, yaw 0 (src=lock, preferred over the camera)",
               src == kTgtLock && t.valid && t.pitch == 0.0f && std::fabs(deg(t.yaw)) < 1e-3f, Fmt("%.3f / %.3f", deg(t.pitch), deg(t.yaw)));
        ai.lockFeet = Vec3{0.0f, 2.05f, 0.5f}; // 2 m ahead, standing 0.5 m higher (stairs)
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock, enemy 2 m ahead 0.5 m higher -> pitch up atan(0.5 / 2) = 14.0 deg",
               src == kTgtLock && std::fabs(deg(t.pitch) - 14.04f) < 0.05f, Fmt("%.3f", deg(t.pitch)));
        ai.lockFeet = Vec3{0.0f, 1.55f, -1.0f}; // 1 m lower: atan(-1 / 1.5) = -33.7
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock, enemy 1.5 m ahead 1 m lower -> pitch -33.7 deg (inside -35)",
               src == kTgtLock && std::fabs(deg(t.pitch) + 33.69f) < 0.05f, Fmt("%.3f", deg(t.pitch)));
        ai.lockFeet = Vec3{-1.5f, 0.05f + 1.5f, 0.0f}; // 45 deg to the left
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock, enemy 45 deg left -> yaw clamped +25 (kMaxYawDeg)",
               src == kTgtLock && std::fabs(deg(t.yaw) - 25.0f) < 0.02f && (t.flags & kFlagClampYaw), Fmt("%.3f", deg(t.yaw)));
        ai.lockFeet = Vec3{0.0f, 0.05f - 3.0f, 0.0f}; // behind V
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock behind V -> facing fade, no tilt", src == kTgtLock && t.pitch == 0.0f && t.yaw == 0.0f && (t.flags & kFlagFacing));
        ai.lockFeet = Vec3{0.0f, 0.05f + 6.0f, 0.0f}; // farther than kMaxLockDist
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock farther than 5 m -> falls through to the camera (src=cam)", src == kTgtCam && std::fabs(deg(t.pitch)) < 0.02f);
        ai.lockFeet = Vec3{0.0f, 0.05f + 0.3f, 0.0f}; // nearer than kMinTargetDist horizontally
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock nearer than 0.5 m -> falls through to the camera", src == kTgtCam);
        ai.lockFeet = Vec3{NAN, 1.0f, 0.0f};
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock NaN -> falls through to the camera", src == kTgtCam && t.valid);
        ai.camValid = false;
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: lock NaN and no camera -> invalid (no target, fades to level)",
               src == kTgtNone && !t.valid && t.pitch == 0.0f && (t.flags & kFlagNoTarget));
    }
    {
        // hit: the crosshair point (utpp_camera aim point feed) within melee reach = direct angles
        AimInput ai;
        ai.camValid = true;
        ai.camPoint = Vec3{0.0f, 0.05f + 1.8f, 1.30f}; // an enemy's chest 1.8 m ahead at chest height
        TargetSource src = kTgtNone;
        Target t = SelectTarget(chest, ai, p, &src);
        Expect("sources: crosshair hit on an enemy chest 1.8 m ahead -> pitch 0 (src=hit)", src == kTgtHit && std::fabs(deg(t.pitch)) < 1e-3f);
        ai.camPoint = Vec3{0.0f, 0.05f + 1.2f, 0.0f}; // looking down at the floor 1.2 m ahead
        t = SelectTarget(chest, ai, p, &src);
        // 2026-11-12 (fists review C4): was the -35 deg stop; the floor in front of V no longer bends the torso (limit -10)
        Expect("sources: crosshair hit on the floor 1.2 m ahead (2.0 m from the chest) -> pitch limited to -10 (src=hit, floor flag)",
               src == kTgtHit && std::fabs(deg(t.pitch) + kHitMaxDownDeg) < 0.02f && (t.flags & kFlagHitFloor) && deg(t.rawPitch) < -30.0f,
               Fmt("%.3f raw %.3f", deg(t.pitch), deg(t.rawPitch)));
        // chest-height / above hits are unchanged, and a mild downward hit (-8) keeps its value
        ai.camPoint = Vec3{0.0f, 0.05f + 2.0f, 1.30f - 2.0f * std::tan(8.0f / kRad2Deg)};
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: crosshair hit 8 deg below the chest -> -8 (under the limit, no floor flag)",
               src == kTgtHit && std::fabs(deg(t.pitch) + 8.0f) < 0.02f && !(t.flags & kFlagHitFloor), Fmt("%.3f", deg(t.pitch)));
        ai.camPoint = Vec3{0.0f, 0.05f + 2.0f, 1.30f + 0.5f};
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: crosshair hit above the chest (enemy head) -> up, not limited by the floor rule (src=hit)",
               src == kTgtHit && std::fabs(deg(t.pitch) - std::atan2(0.5f, 2.0f) * kRad2Deg) < 0.02f, Fmt("%.3f", deg(t.pitch)));
        // the lock source keeps its full range: a target 1 m lower 1.5 m ahead stays at -33.7 while the floor hit is limited
        {
            AimInput lk;
            lk.lockValid = true;
            lk.lockFeet = Vec3{0.0f, 1.55f, -1.0f};
            lk.camValid = true;
            lk.camPoint = Vec3{0.0f, 0.05f + 1.2f, 0.0f}; // the floor hit would be limited to -10
            TargetSource s2 = kTgtNone;
            const Target tl = SelectTarget(chest, lk, p, &s2);
            Expect("sources: lock keeps the full range (-33.7) even with a floor crosshair hit (limit applies to hit only)",
                   s2 == kTgtLock && std::fabs(deg(tl.pitch) + 33.69f) < 0.05f && !(tl.flags & kFlagHitFloor), Fmt("%.3f", deg(tl.pitch)));
        }
        // the default framing floor hit: camera 1.7 m high, 2 m behind the chest, pitched -15 -> ground ~6.3 m ahead of the camera,
        // ~4.3 m ahead of the chest (> kMaxHitDist): camera branch with the dead band, not the -17 deg lean
        const Vec3 cam{0.0f, 0.05f - 2.0f, 1.70f};
        const float run = 1.70f / std::tan(15.0f / kRad2Deg);
        ai.camPoint = Vec3{0.0f, cam.y + run, 0.0f};
        t = SelectTarget(chest, ai, p, &src);
        const float dist = aim::Length(ai.camPoint - chest);
        // seen from the chest (below the camera) the floor point is a little steeper than the camera pitch (-16.7 vs -15):
        // the dead band leaves under 2 deg instead of the -16.7 deg lean of a direct hit
        Expect("sources: default-framing floor hit (~4.5 m from the chest) -> camera branch (src=cam), dead band -> under 2 deg",
               src == kTgtCam && dist > kMaxHitDist && std::fabs(deg(t.pitch)) < 2.0f && deg(t.rawPitch) < -15.0f,
               Fmt("dist %.2f raw %.2f pitch %.3f", dist, deg(t.rawPitch), deg(t.pitch)));
        ai.camPoint = chest + Vec3{0.0f, 0.2f, 0.0f}; // too near
        t = SelectTarget(chest, ai, p, &src);
        Expect("sources: camera point nearer than 0.5 m -> invalid (kFlagNear | kFlagNoTarget)",
               src == kTgtNone && !t.valid && (t.flags & kFlagNear) && (t.flags & kFlagNoTarget));
    }
    Expect("sources: constants (neutral -15, lock 5 m, hit 3 m, lock age 0.3 s)",
           kNeutralPitchDeg == -15.0f && kMaxLockDist == 5.0f && kMaxHitDist == 3.0f && kMaxLockAge == 0.3f);
}

// 2026-11-12 (fists review C4 / C5): hit <-> cam hysteresis around 3 m, the camera-branch yaw dead zone.
static void TestHysteresisAndDeadZone()
{
    Params p;
    p.maxUpDeg = kDefaultMaxUpDeg;
    p.maxDownDeg = kDefaultMaxDownDeg;
    const Vec3 chest{0.0f, 0.05f, 1.30f};
    auto deg = [](float r) { return r * kRad2Deg; };
    // a camera point straight ahead of the chest at horizontal distance d, at chest height (hit pitch 0)
    auto pointAt = [&](float d) { return Vec3{0.0f, chest.y + d, chest.z}; };
    auto pick = [&](float d, bool& latch, Target* aOut = nullptr) {
        AimInput ai;
        ai.camValid = true;
        ai.camPoint = pointAt(d);
        TargetSource s = kTgtNone;
        const Target t = SelectTarget(chest, ai, p, &s, &latch);
        if (aOut)
            *aOut = t;
        return s;
    };
    {
        bool latch = false;
        int toggles = 0;
        TargetSource last = pick(2.9f, latch);
        Expect("hysteresis: 2.9 m from a cold start -> cam (entering hit needs < 2.6 m)", last == kTgtCam && !latch);
        // oscillation 2.9 <-> 3.1 m around the old 3 m switch: no toggle at all (stays cam, never entered hit)
        for (int i = 0; i < 40; ++i)
        {
            const TargetSource s = pick((i & 1) ? 3.1f : 2.9f, latch);
            toggles += s != last;
            last = s;
        }
        Expect("hysteresis: 2.9 <-> 3.1 m oscillation from cam -> no source change (no chatter at the old 3 m switch)", toggles == 0 && last == kTgtCam);
        // sweep in 2 -> 4 -> 2 m: hit below 2.6, leaves above 3.4, comes back below 2.6; exactly one toggle each way
        latch = false;
        toggles = 0;
        last = pick(2.0f, latch);
        const bool startsHit = last == kTgtHit;
        float leaveAt = -1.0f, enterAt = -1.0f;
        for (float d = 2.0f; d <= 4.0f; d += 0.01f)
        {
            const TargetSource s = pick(d, latch);
            if (s != last)
            {
                ++toggles;
                if (leaveAt < 0.0f)
                    leaveAt = d;
            }
            last = s;
        }
        for (float d = 4.0f; d >= 2.0f; d -= 0.01f)
        {
            const TargetSource s = pick(d, latch);
            if (s != last)
            {
                ++toggles;
                if (enterAt < 0.0f)
                    enterAt = d;
            }
            last = s;
        }
        Expect("hysteresis: sweep 2 -> 4 -> 2 m toggles exactly twice (leaves hit at ~3.4, re-enters at ~2.6)",
               startsHit && toggles == 2 && std::fabs(leaveAt - 3.4f) < 0.03f && std::fabs(enterAt - 2.6f) < 0.03f,
               Fmt("start %.0f toggles %.0f leave %.2f enter %.2f", startsHit ? 1 : 0, toggles, leaveAt, enterAt));
        // inside the band the previous source wins in both directions
        latch = false;
        pick(2.0f, latch);
        Expect("hysteresis: hit latched at 2.0 m stays hit at 3.0 m and 3.39 m", pick(3.0f, latch) == kTgtHit && pick(3.39f, latch) == kTgtHit && latch);
        Expect("hysteresis: ... and leaves above 3.4 m (src=cam, latch cleared)", pick(3.41f, latch) == kTgtCam && !latch);
        Expect("hysteresis: cam at 3.0 m stays cam (enter needs < 2.6 m)", pick(3.0f, latch) == kTgtCam && pick(2.61f, latch) == kTgtCam && pick(2.59f, latch) == kTgtHit);
        // another source clears the latch (lock wins, then the camera at 3.0 m is cam again, not a stale hit)
        {
            AimInput ai;
            ai.lockValid = true;
            ai.lockFeet = Vec3{0.0f, 1.55f, 0.0f};
            ai.camValid = true;
            ai.camPoint = pointAt(2.0f);
            TargetSource s = kTgtNone;
            bool l2 = true;
            SelectTarget(chest, ai, p, &s, &l2);
            Expect("hysteresis: lock wins -> latch cleared", s == kTgtLock && !l2);
        }
        // no latch (nullptr) = the single 3 m threshold of before
        AimInput ai;
        ai.camValid = true;
        ai.camPoint = pointAt(2.95f);
        TargetSource s = kTgtNone;
        SelectTarget(chest, ai, p, &s);
        const bool below = s == kTgtHit;
        ai.camPoint = pointAt(3.05f);
        SelectTarget(chest, ai, p, &s);
        Expect("hysteresis: nullptr latch = single threshold kMaxHitDist (2.95 hit, 3.05 cam)", below && s == kTgtCam);
        // the floor hit stays limited while latched and after the switch the camera branch has its own (dead-band) pitch
        latch = false;
        Target t;
        ai.camPoint = Vec3{0.0f, chest.y + 1.2f, 0.0f};
        SelectTarget(chest, ai, p, &s, &latch);
        SelectTarget(chest, ai, p, &s, &latch);
        t = SelectTarget(chest, ai, p, &s, &latch);
        Expect("hysteresis: floor 1.2 m ahead, latched hit -> pitch stays at -10 (no -35)", s == kTgtHit && std::fabs(deg(t.pitch) + 10.0f) < 0.02f);
    }
    {
        // yaw dead zone of the camera branch: 0 inside +-5, continuous at the edge, shrinkage outside, sign kept, raw yaw kept
        Expect("deadzone: 0 inside +-5 deg, continuous at the edge",
               CameraYawDeadZone(0.0f) == 0.0f && CameraYawDeadZone(4.99f / kRad2Deg) == 0.0f && CameraYawDeadZone(-4.99f / kRad2Deg) == 0.0f &&
                   std::fabs(CameraYawDeadZone(5.01f / kRad2Deg)) < 0.011f / kRad2Deg + 1e-7f &&
                   std::fabs(CameraYawDeadZone(-5.01f / kRad2Deg)) < 0.011f / kRad2Deg + 1e-7f && CameraYawDeadZone(NAN) == 0.0f);
        bool mono = true, odd = true;
        float prev = CameraYawDeadZone(-40.0f / kRad2Deg);
        for (float y = -40.0f; y <= 40.0f; y += 0.05f)
        {
            const float v = CameraYawDeadZone(y / kRad2Deg);
            mono = mono && v >= prev - 1e-7f && std::fabs(v - prev) < 0.06f / kRad2Deg;
            odd = odd && std::fabs(v + CameraYawDeadZone(-y / kRad2Deg)) < 1e-6f;
            prev = v;
        }
        Expect("deadzone: monotonic, no step above the scan step, odd (left / right symmetric)", mono && odd);
        auto camAt = [&](float pitchDeg, float yawDeg) {
            const float pr = pitchDeg / kRad2Deg, yr = yawDeg / kRad2Deg;
            return chest + Vec3{-std::sin(yr) * std::cos(pr), std::cos(yr) * std::cos(pr), std::sin(pr)} * 60.0f;
        };
        auto yawOf = [&](float yawDeg, TargetSource* s) {
            AimInput ai;
            ai.camValid = true;
            ai.camPoint = camAt(-15.0f, yawDeg);
            return SelectTarget(chest, ai, p, s);
        };
        TargetSource s = kTgtNone;
        Target t = yawOf(-6.0f, &s);
        Expect("deadzone: the pilot's constant -6 deg twist at rest -> ~ -1 deg (src=cam)", s == kTgtCam && std::fabs(deg(t.yaw) + 1.0f) < 0.05f, Fmt("%.3f", deg(t.yaw)));
        t = yawOf(4.0f, &s);
        Expect("deadzone: facing forward (+4 deg) -> yaw exactly 0", s == kTgtCam && t.yaw == 0.0f);
        t = yawOf(20.0f, &s);
        Expect("deadzone: 20 deg left -> 15 deg (shrinkage by 5)", s == kTgtCam && std::fabs(deg(t.yaw) - 15.0f) < 0.05f, Fmt("%.3f", deg(t.yaw)));
        t = yawOf(40.0f, &s);
        Expect("deadzone: 40 deg left -> clamp +25 after the dead zone (35 -> 25), facing uses the raw yaw",
               s == kTgtCam && std::fabs(deg(t.yaw) - 25.0f) < 0.02f && (t.flags & kFlagClampYaw) && std::fabs(deg(t.rawYaw) - 40.0f) < 0.05f, Fmt("%.3f", deg(t.yaw)));
        // lock yaw is NOT shrunk (a real enemy keeps the full direction)
        AimInput ai;
        ai.lockValid = true;
        ai.lockFeet = Vec3{-0.3f, chest.y + 3.0f, 0.0f}; // ~5.7 deg left
        t = SelectTarget(chest, ai, p, &s);
        Expect("deadzone: lock source yaw is untouched (5.7 deg stays 5.7)", s == kTgtLock && std::fabs(deg(t.yaw) - 5.71f) < 0.05f, Fmt("%.3f", deg(t.yaw)));
    }
    Expect("hysteresis: constants (enter 2.6, leave 3.4 around kMaxHitDist, floor limit 10, dead zone 5)",
           kHitEnterDist == 2.6f && kHitLeaveDist == 3.4f && kHitEnterDist < kMaxHitDist && kMaxHitDist < kHitLeaveDist && kHitMaxDownDeg == 10.0f &&
               kCamYawDeadDeg == 5.0f);
}

int main()
{
    TestRig();
    TestTarget();
    TestTilt();
    TestSmoothing();
    TestWeightAndParams();
    TestTargetSources();
    TestHysteresisAndDeadZone();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

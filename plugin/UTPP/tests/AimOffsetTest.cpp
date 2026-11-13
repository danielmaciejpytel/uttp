// Offline tests of the 0.6.3 aim offset core (src/mm/AimOffset.hpp), not shipped.
// Usage: utpp_aimoffset_test   Exit code 0 = all checks passed.
// Synthetic rigs: a 3-bone chain (FK known answer) and a 9-bone torso / arms / weapon rig with non-trivial rotations
// (two-handed grip); the real player rig 0..28 (names + parents from the player skeleton listing)
// for the bone resolution.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/AimOffset.hpp"

using namespace utpp::mm;
using namespace utpp::mm::aim;
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

// ------------------------------------------------------------------------------------------------ synthetic pose buffers

struct Pose
{
    std::vector<uint8_t> bytes;
    uint32_t count = 0;
};

static Pose MakePose(uint32_t aCount)
{
    Pose p;
    p.count = aCount;
    p.bytes.assign(size_t(aCount) * integ::kGameBoneStride, 0);
    for (uint32_t i = 0; i < aCount; ++i)
    {
        float* r = reinterpret_cast<float*>(p.bytes.data() + size_t(i) * integ::kGameBoneStride);
        r[7] = 1.0f;                       // identity rotation
        r[8] = r[9] = r[10] = r[11] = 1.0f; // scale
    }
    return p;
}

static void SetLocal(Pose& aPose, uint32_t aIndex, const Xf& aLocal)
{
    float* r = reinterpret_cast<float*>(aPose.bytes.data() + size_t(aIndex) * integ::kGameBoneStride);
    r[0] = aLocal.p.x;
    r[1] = aLocal.p.y;
    r[2] = aLocal.p.z;
    r[3] = 0.0f;
    r[4] = aLocal.q.x;
    r[5] = aLocal.q.y;
    r[6] = aLocal.q.z;
    r[7] = aLocal.q.w;
}

// Writes local records that produce the given model transforms (parents first).
static void SetFromModel(Pose& aPose, const int16_t* aParents, const Xf* aModel, uint32_t aN)
{
    for (uint32_t i = 0; i < aN; ++i)
    {
        const int32_t par = aParents[i];
        if (par < 0)
        {
            SetLocal(aPose, i, aModel[i]);
            continue;
        }
        const Quat inv = math::QuatInv(aModel[par].q);
        SetLocal(aPose, i, Xf{math::QuatRotate(inv, aModel[i].p - aModel[par].p), math::QuatNormalize(math::QuatMul(inv, aModel[i].q))});
    }
}

static Vec3 Z() { return Vec3{0.0f, 0.0f, 1.0f}; }
static Vec3 X() { return Vec3{1.0f, 0.0f, 0.0f}; }
static Vec3 Y() { return Vec3{0.0f, 1.0f, 0.0f}; }
static float Dist(Vec3 a, Vec3 b) { return Length(a - b); }
static float QuatAngleDeg(const Quat& a, const Quat& b)
{
    const float d = std::fabs(math::QuatDot(math::QuatNormalize(a), math::QuatNormalize(b)));
    return 2.0f * std::acos(d > 1.0f ? 1.0f : d) * kRad2Deg;
}

// ------------------------------------------------------------------------------------------------ the 9-bone test rig

enum : uint32_t
{
    kRoot = 0,
    kSpine3 = 1,
    kRArm = 2,
    kRFore = 3,
    kRHand = 4,
    kWeapon = 5,
    kLArm = 6,
    kLFore = 7,
    kLHand = 8,
    kTestBones = 9
};
static const char* kTestNames[kTestBones] = {"Root", "Spine3", "RightArm", "RightForeArm", "RightHand", "WeaponRight",
                                             "LeftArm", "LeftForeArm", "LeftHand"};
static const int16_t kTestParents[kTestBones] = {-1, 0, 1, 2, 3, 4, 1, 6, 7};

static Rig TestRig()
{
    std::vector<uint64_t> h;
    for (const char* n : kTestNames)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> p(kTestParents, kTestParents + kTestBones);
    return ResolveRig(h, p);
}

// Aiming pose: chest yawed 10 deg, weapon about forward (+Y = barrel), left hand 0.15 m ahead of the weapon bone.
static Xf g_srcModel[kTestBones];
static Pose SourcePose(bool aTwoHanded = true)
{
    Xf* m = g_srcModel;
    m[kRoot] = Xf{Vec3{0.0f, 0.0f, 0.0f}, AxisAngle(Z(), 5.0f * kDegF)};
    m[kSpine3] = Xf{Vec3{0.0f, -0.05f, 1.35f}, AxisAngle(Z(), 10.0f * kDegF)};
    m[kRArm] = Xf{Vec3{0.18f, -0.02f, 1.45f}, math::QuatMul(AxisAngle(Z(), 70.0f * kDegF), AxisAngle(X(), -20.0f * kDegF))};
    m[kRFore] = Xf{Vec3{0.22f, 0.18f, 1.25f}, math::QuatMul(AxisAngle(Z(), 30.0f * kDegF), AxisAngle(Y(), 15.0f * kDegF))};
    m[kRHand] = Xf{Vec3{0.12f, 0.42f, 1.33f}, AxisAngle(Y(), -25.0f * kDegF)};
    m[kWeapon] = Xf{Vec3{0.11f, 0.45f, 1.37f}, math::QuatMul(AxisAngle(Z(), 4.0f * kDegF), AxisAngle(X(), -3.0f * kDegF))};
    m[kLArm] = Xf{Vec3{-0.18f, -0.02f, 1.45f}, AxisAngle(Z(), -50.0f * kDegF)};
    m[kLFore] = Xf{Vec3{-0.12f, 0.22f, 1.22f}, AxisAngle(X(), 20.0f * kDegF)};
    m[kLHand] = aTwoHanded ? Xf{Vec3{0.08f, 0.60f, 1.33f}, AxisAngle(Y(), 60.0f * kDegF)}
                           : Xf{Vec3{-0.25f, 0.05f, 0.95f}, AxisAngle(X(), 80.0f * kDegF)};
    Pose p = MakePose(kTestBones + 3); // a few extra bones after the FK range (must stay untouched)
    SetFromModel(p, kTestParents, m, kTestBones);
    return p;
}

// Target point in model space at yaw / pitch (deg) relative to the source barrel, 60 m from the source muzzle.
static Vec3 TargetAt(float aYawDeg, float aPitchDeg, const Params& aP)
{
    const Vec3 fwd = math::QuatRotate(g_srcModel[kWeapon].q, AxisVector(aP.barrelAxis));
    const Vec3 muzzle = g_srcModel[kWeapon].p + fwd * aP.muzzleOffset;
    Quat q;
    YawPitchRotation(fwd, aYawDeg * kDegF, aPitchDeg * kDegF, q);
    return muzzle + math::QuatRotate(q, fwd) * 60.0f;
}

static bool Model(const Pose& aPose, Xf* aOut)
{
    return ComputeModel(aPose.bytes.data(), aPose.count, kTestParents, kTestBones, aOut);
}

// ------------------------------------------------------------------------------------------------ tests

static void TestFk()
{
    // 3-bone chain, known answer: root Rz(90) at the origin, child (1,0,0), grandchild (1,0,0) local
    const int16_t par[3] = {-1, 0, 1};
    Pose p = MakePose(3);
    SetLocal(p, 0, Xf{Vec3{}, AxisAngle(Z(), 90.0f * kDegF)});
    SetLocal(p, 1, Xf{Vec3{1.0f, 0.0f, 0.0f}, Quat{}});
    SetLocal(p, 2, Xf{Vec3{1.0f, 0.0f, 0.0f}, AxisAngle(X(), 30.0f * kDegF)});
    Xf m[3];
    const bool ok = ComputeModel(p.bytes.data(), 3, par, 3, m);
    Expect("FK 3-bone chain: child at (0,1,0), grandchild at (0,2,0)",
           ok && Dist(m[1].p, Vec3{0.0f, 1.0f, 0.0f}) < 1e-5f && Dist(m[2].p, Vec3{0.0f, 2.0f, 0.0f}) < 1e-5f,
           Fmt("child (%.4f,%.4f) grandchild y=%.4f", m[1].p.x, m[1].p.y, m[2].p.y));
    Expect("FK 3-bone chain: rotations compose (grandchild = Rz90 * Rx30)",
           ok && QuatAngleDeg(m[2].q, math::QuatMul(AxisAngle(Z(), 90.0f * kDegF), AxisAngle(X(), 30.0f * kDegF))) < 1e-3f);
    // round trip of the 9-bone rig
    Pose src = SourcePose();
    Xf fm[kTestBones];
    bool rt = Model(src, fm);
    for (uint32_t i = 0; rt && i < kTestBones; ++i)
        rt = Dist(fm[i].p, g_srcModel[i].p) < 1e-5f && QuatAngleDeg(fm[i].q, g_srcModel[i].q) < 1e-2f;
    Expect("FK round trip of the 9-bone rig (model -> local records -> FK)", rt);
    // fail closed
    const int16_t bad[3] = {-1, 2, 1};
    Expect("FK rejects non-topological parents", !ComputeModel(p.bytes.data(), 3, bad, 3, m));
    Expect("FK rejects n > count", !ComputeModel(p.bytes.data(), 2, par, 3, m));
    Pose nanp = MakePose(3);
    reinterpret_cast<float*>(nanp.bytes.data() + 48)[1] = std::nanf("");
    Expect("FK rejects a non-finite record", !ComputeModel(nanp.bytes.data(), 3, par, 3, m));
    Pose zq = MakePose(3);
    reinterpret_cast<float*>(zq.bytes.data() + 48)[7] = 0.0f;
    Expect("FK rejects a zero quaternion", !ComputeModel(zq.bytes.data(), 3, par, 3, m));
}

static void TestMath()
{
    // world -> model: entity at (10, 5, 0) yawed +90 deg (its forward +Y points to world -X)
    const Vec3 m = WorldToModel(Vec3{10.0f, 5.0f, 0.0f}, AxisAngle(Z(), 90.0f * kDegF), Vec3{9.0f, 5.0f, 2.0f});
    Expect("WorldToModel: world point 1 m along the entity forward -> model (0, 1, 2)", Dist(m, Vec3{0.0f, 1.0f, 2.0f}) < 1e-5f,
           Fmt("(%.4f, %.4f, %.4f)", m.x, m.y, m.z));
    // yaw / pitch rotation is exact
    float worst = 0.0f;
    const float yaws[] = {-170.0f, -35.0f, 0.0f, 20.0f, 120.0f};
    const float pitches[] = {-40.0f, 0.0f, 15.0f, 50.0f};
    for (const float y0 : yaws)
        for (const float p0 : pitches)
            for (const float dy : {-30.0f, 7.0f, 25.0f})
                for (const float dp : {-25.0f, 3.0f, 20.0f})
                {
                    if (std::fabs(p0 + dp) > 80.0f)
                        continue;
                    const Vec3 f{-std::sin(y0 * kDegF) * std::cos(p0 * kDegF), std::cos(y0 * kDegF) * std::cos(p0 * kDegF),
                                 std::sin(p0 * kDegF)};
                    Quat q;
                    if (!YawPitchRotation(f, dy * kDegF, dp * kDegF, q))
                    {
                        worst = 999.0f;
                        continue;
                    }
                    float ey, ep;
                    YawPitchDelta(f, math::QuatRotate(q, f), ey, ep);
                    worst = std::fmax(worst, std::fmax(std::fabs(ey * kRad2Deg - dy), std::fabs(ep * kRad2Deg - dp)));
                }
    Expect("YawPitchRotation changes yaw / pitch exactly (< 0.01 deg, 5x4x3x3 cases)", worst < 0.01f, Fmt("worst %.5f deg", worst));
    Quat q;
    Expect("YawPitchRotation fails closed for a vertical forward", !YawPitchRotation(Vec3{0.0f, 0.0f, 1.0f}, 0.1f, 0.1f, q));
    Expect("Barrel axes: +Y default, -Z", Dist(AxisVector(kDefaultBarrelAxis), Y()) < 1e-6f && Dist(AxisVector(-3), Vec3{0, 0, -1}) < 1e-6f &&
                                            !ValidAxis(0) && !ValidAxis(4) && ValidAxis(-1));
    Expect("Params validation (0..60 deg, 0..120 1/s, NaN rejected)",
           ValidParams(30, 25, 12) && ValidParams(0, 0, 0) && !ValidParams(61, 25, 12) && !ValidParams(30, -1, 12) &&
               !ValidParams(30, 25, 121) && !ValidParams(std::nanf(""), 25, 12));
    Expect("GamePoseShare = 1 - w * upperScale", std::fabs(GamePoseShare(1.0f, 0.0f) - 1.0f) < 1e-6f &&
                                                     std::fabs(GamePoseShare(1.0f, 0.25f) - 0.75f) < 1e-6f &&
                                                     std::fabs(GamePoseShare(0.5f, 0.5f) - 0.75f) < 1e-6f &&
                                                     GamePoseShare(1.0f, 1.0f) == 0.0f && GamePoseShare(std::nanf(""), 0.0f) == 1.0f);
}

static void TestRigResolve()
{
    const Rig r = TestRig();
    Expect("ResolveRig: synthetic rig resolved", r.ok == 1 && r.rightArm == 2 && r.rightHand == 4 && r.weapon == 5 && r.leftArm == 6 &&
                                                     r.leftForeArm == 7 && r.leftHand == 8 && r.fkCount == 9);
    // real player rig 0..28 (skeleton-player_woman_skeleton.txt)
    static const char* names[] = {"Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1",
                                  "LeftLeg", "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder",
                                  "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head",
                                  "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight"};
    static const int16_t parents[] = {-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24};
    std::vector<uint64_t> h;
    for (const char* n : names)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> p(parents, parents + 29);
    const Rig real = ResolveRig(h, p);
    Expect("ResolveRig: player rig -> RightArm 18, RightHand 24, WeaponRight 28, LeftArm 17, LeftForeArm 20, LeftHand 23, fk 29",
           real.ok == 1 && real.rightArm == 18 && real.rightHand == 24 && real.weapon == 28 && real.leftArm == 17 &&
               real.leftForeArm == 20 && real.leftHand == 23 && real.fkCount == 29);
    // fail closed
    std::vector<uint64_t> h2 = h;
    h2[28] = integ::CNameHash("WeaponRightX");
    Expect("ResolveRig: missing WeaponRight -> not ok", ResolveRig(h2, p).ok == 0);
    std::vector<int16_t> p2 = p;
    p2[28] = 23; // weapon under the left hand
    Expect("ResolveRig: WeaponRight not under RightHand -> not ok", ResolveRig(h, p2).ok == 0);
    std::vector<int16_t> p3 = p;
    p3[20] = 21; // left forearm under the right forearm
    Expect("ResolveRig: broken left chain -> not ok", ResolveRig(h, p3).ok == 0);
    std::vector<int16_t> p4 = p;
    p4[5] = 9; // parent after child
    Expect("ResolveRig: non-topological parents -> not ok", ResolveRig(h, p4).ok == 0);
    std::vector<uint64_t> big(100, 0);
    std::vector<int16_t> bigp(100, 0);
    bigp[0] = -1;
    for (int i = 0; i < 7; ++i)
        big[size_t(90 + i)] = integ::CNameHash(kTestNames[2 + i]);
    Expect("ResolveRig: bone index >= 80 -> not ok", ResolveRig(big, bigp).ok == 0);
    Expect("ResolveRig: size mismatch -> not ok", ResolveRig(h, std::vector<int16_t>(p.begin(), p.end() - 1)).ok == 0);
}

static void TestSolveWithinLimits()
{
    const Rig rig = TestRig();
    Params prm;
    prm.smoothHz = 0.0f; // snap
    Pose pose = SourcePose();
    const std::vector<uint8_t> before = pose.bytes;
    const Vec3 target = TargetAt(12.0f, -8.0f, prm);
    Controller c;
    Result r;
    const bool ok = c.Apply(pose.bytes.data(), pose.count, rig, prm, target, 1.0f, 1.0f / 60.0f, r);
    Expect("solve 12/-8 deg: applied", ok);
    Expect("solve: source error measured (~14.4 deg, yaw 12 / pitch -8)",
           std::fabs(r.errYaw * kRad2Deg - 12.0f) < 0.05f && std::fabs(r.errPitch * kRad2Deg + 8.0f) < 0.05f,
           Fmt("err %.3f / %.3f, before %.3f deg", r.errYaw * kRad2Deg, r.errPitch * kRad2Deg, r.errBefore * kRad2Deg));
    Expect("solve: barrel on target after the correction (< 0.05 deg)", r.errAfter * kRad2Deg < 0.05f,
           Fmt("after %.4f deg, residual %.4f / %.4f deg", r.errAfter * kRad2Deg, r.resYaw * kRad2Deg, r.resPitch * kRad2Deg));
    Expect("solve: residual at the wrist is small (< 0.5 deg)",
           std::fabs(r.resYaw) * kRad2Deg < 0.5f && std::fabs(r.resPitch) * kRad2Deg < 0.5f);
    Xf m[kTestBones];
    Model(pose, m);
    Expect("solve: shoulder joint does not move, chest / root records bitwise unchanged",
           Dist(m[kRArm].p, g_srcModel[kRArm].p) < 1e-5f &&
               std::memcmp(pose.bytes.data(), before.data(), size_t(kRArm) * integ::kGameBoneStride) == 0);
    Expect("solve: bones outside the FK range untouched",
           std::memcmp(pose.bytes.data() + size_t(kTestBones) * 48, before.data() + size_t(kTestBones) * 48, 3 * 48) == 0);
    Expect("solve: local positions never written (bone lengths kept)", [&] {
        for (uint32_t i = 0; i < kTestBones; ++i)
            if (std::memcmp(pose.bytes.data() + size_t(i) * 48, before.data() + size_t(i) * 48, 16) != 0)
                return false;
        return true;
    }());
    // two-handed grip kept: left hand transform relative to the weapon unchanged
    const Quat inv0 = math::QuatInv(g_srcModel[kWeapon].q);
    const Vec3 rel0 = math::QuatRotate(inv0, g_srcModel[kLHand].p - g_srcModel[kWeapon].p);
    const Vec3 rel1 = math::QuatRotate(math::QuatInv(m[kWeapon].q), m[kLHand].p - m[kWeapon].p);
    const Quat relq0 = math::QuatMul(inv0, g_srcModel[kLHand].q);
    const Quat relq1 = math::QuatMul(math::QuatInv(m[kWeapon].q), m[kLHand].q);
    Expect("solve: two-handed grip detected and IK solved", r.twoHanded && r.leftIkOk);
    Expect("solve: left hand stays on the weapon (< 1 mm, < 0.2 deg)", Dist(rel0, rel1) < 1e-3f && QuatAngleDeg(relq0, relq1) < 0.2f,
           Fmt("pos %.5f m rot %.4f deg lhErr %.5f", Dist(rel0, rel1), QuatAngleDeg(relq0, relq1), r.leftHandErr));
    Expect("solve: left shoulder joint fixed, forearm / upper arm lengths kept",
           Dist(m[kLArm].p, g_srcModel[kLArm].p) < 1e-5f &&
               std::fabs(Dist(m[kLFore].p, m[kLArm].p) - Dist(g_srcModel[kLFore].p, g_srcModel[kLArm].p)) < 1e-4f &&
               std::fabs(Dist(m[kLHand].p, m[kLFore].p) - Dist(g_srcModel[kLHand].p, g_srcModel[kLFore].p)) < 1e-4f);
    // other barrel axis: the same solver aims local -Z of a rotated weapon
    Params pz = prm;
    pz.barrelAxis = -3;
    Pose motionpose = SourcePose();
    Controller c2;
    Result r2;
    const bool ok2 = c2.Apply(motionpose.bytes.data(), motionpose.count, rig, pz, TargetAt(-6.0f, 4.0f, pz), 1.0f, 0.016f, r2);
    Expect("solve with barrel axis -Z on target (< 0.05 deg)", ok2 && r2.errAfter * kRad2Deg < 0.05f,
           Fmt("before %.3f after %.4f deg", r2.errBefore * kRad2Deg, r2.errAfter * kRad2Deg));
}

static void TestLimits()
{
    const Rig rig = TestRig();
    Params prm;
    prm.smoothHz = 0.0f;
    Pose pose = SourcePose();
    Controller c;
    Result r;
    const bool ok = c.Apply(pose.bytes.data(), pose.count, rig, prm, TargetAt(60.0f, -45.0f, prm), 1.0f, 0.016f, r);
    Expect("limits: shoulder yaw clamped to 30, pitch to -25", ok && std::fabs(r.cmdYaw * kRad2Deg - 30.0f) < 1e-3f &&
                                                                 std::fabs(r.cmdPitch * kRad2Deg + 25.0f) < 1e-3f,
           Fmt("cmd %.3f / %.3f", r.cmdYaw * kRad2Deg, r.cmdPitch * kRad2Deg));
    Expect("limits: wrist residual within 5 deg", std::fabs(r.resYaw * kRad2Deg) <= 5.0001f && std::fabs(r.resPitch * kRad2Deg) <= 5.0001f,
           Fmt("res %.3f / %.3f", r.resYaw * kRad2Deg, r.resPitch * kRad2Deg));
    float y, p;
    Xf m[kTestBones];
    Model(pose, m);
    YawPitchDelta(math::QuatRotate(g_srcModel[kWeapon].q, Y()), math::QuatRotate(m[kWeapon].q, Y()), y, p);
    Expect("limits: total barrel turn <= 30 + 5 yaw / 25 + 5 pitch", std::fabs(y * kRad2Deg) <= 35.01f && std::fabs(p * kRad2Deg) <= 30.01f,
           Fmt("turned %.3f / %.3f deg, error left %.2f deg", y * kRad2Deg, p * kRad2Deg, r.errAfter * kRad2Deg));
    Params tight = prm;
    tight.maxYawDeg = 10.0f;
    tight.maxPitchDeg = 5.0f;
    Pose motionpose = SourcePose();
    Controller c2;
    Result r2;
    c2.Apply(motionpose.bytes.data(), motionpose.count, rig, tight, TargetAt(-20.0f, 12.0f, tight), 1.0f, 0.016f, r2);
    Expect("limits: custom 10 / 5 respected (negative yaw)", std::fabs(r2.cmdYaw * kRad2Deg + 10.0f) < 1e-3f &&
                                                             std::fabs(r2.cmdPitch * kRad2Deg - 5.0f) < 1e-3f);
}

// Runs aFrames frames at aFps from the same source pose (the game graph re-evaluates the pose every frame).
static std::vector<Result> RunFrames(float aFps, int aFrames, float aSmoothHz, Vec3 aTarget, float aWeight = 1.0f)
{
    const Rig rig = TestRig();
    Params prm;
    prm.smoothHz = aSmoothHz;
    Controller c;
    std::vector<Result> out;
    for (int i = 0; i < aFrames; ++i)
    {
        Pose pose = SourcePose();
        Result r;
        c.Apply(pose.bytes.data(), pose.count, rig, prm, aTarget, aWeight, 1.0f / aFps, r);
        out.push_back(r);
    }
    return out;
}

static void TestSmoothing()
{
    Params prm;
    const Vec3 target = TargetAt(15.0f, 10.0f, prm);
    const std::vector<Result> rs = RunFrames(60.0f, 60, 12.0f, target);
    bool mono = true;
    float worstRise = 0.0f;
    for (size_t i = 1; i < rs.size(); ++i)
    {
        worstRise = std::fmax(worstRise, rs[i].errAfter - rs[i - 1].errAfter);
        mono = mono && rs[i].errAfter <= rs[i - 1].errAfter + 1e-6f;
    }
    Expect("smoothing 12/s @60 fps: first frame partial (18 % of the error)",
           rs[0].errAfter > 0.7f * rs[0].errBefore && rs[0].errAfter < 0.9f * rs[0].errBefore,
           Fmt("before %.3f after %.3f deg", rs[0].errBefore * kRad2Deg, rs[0].errAfter * kRad2Deg));
    Expect("smoothing: error decreases monotonically (no overshoot)", mono, Fmt("worst rise %.2e deg", worstRise * kRad2Deg));
    Expect("smoothing: converged after 0.5 s (< 0.1 deg)", rs[29].errAfter * kRad2Deg < 0.1f && rs[59].errAfter * kRad2Deg < 0.01f,
           Fmt("t=0.5 s %.4f deg, t=1 s %.5f deg", rs[29].errAfter * kRad2Deg, rs[59].errAfter * kRad2Deg));
    // frame-rate independence: command at t = 0.25 s for 30 / 60 / 144 fps
    const float c30 = RunFrames(30.0f, 8, 12.0f, target).back().cmdYaw;  // 8 / 30 = 0.267 s
    const float c60 = RunFrames(60.0f, 16, 12.0f, target).back().cmdYaw; // 0.267 s
    const float c144 = RunFrames(144.0f, 38, 12.0f, target).back().cmdYaw; // 0.264 s
    Expect("smoothing: frame-rate independent (30 / 60 / 144 fps at ~0.265 s within 1.5 %)",
           std::fabs(c30 - c60) < 0.015f * std::fabs(c60) && std::fabs(c144 - c60) < 0.015f * std::fabs(c60),
           Fmt("cmdYaw %.4f / %.4f / %.4f deg", c30 * kRad2Deg, c60 * kRad2Deg, c144 * kRad2Deg));
    // dt <= 0 holds the value; a huge dt is clamped to 0.1 s
    const Rig rig = TestRig();
    Params p0;
    Controller c;
    Result r;
    Pose a = SourcePose();
    c.Apply(a.bytes.data(), a.count, rig, p0, target, 1.0f, 0.0f, r);
    Expect("smoothing: dt 0 on the first frame applies nothing", std::fabs(r.cmdYaw) < 1e-7f && std::fabs(r.errAfter - r.errBefore) < 1e-4f);
    Pose b = SourcePose();
    c.Apply(b.bytes.data(), b.count, rig, p0, target, 1.0f, 10.0f, r);
    Expect("smoothing: dt clamped to 0.1 s (70 % step)", std::fabs(r.cmdYaw / (15.0f * kDegF) - (1.0f - std::exp(-1.2f))) < 0.01f,
           Fmt("ratio %.4f", r.cmdYaw / (15.0f * kDegF)));
    c.Reset();
    Expect("smoothing: Reset clears the state", !c.Has() && c.SmoothedYaw() == 0.0f && c.SmoothedPitch() == 0.0f);
    // weight scales the applied delta (mask fading in)
    const std::vector<Result> half = RunFrames(60.0f, 120, 12.0f, target, 0.5f);
    Expect("weight 0.5: half of the shoulder delta", std::fabs(half.back().cmdYaw - 0.5f * 15.0f * kDegF) < 1e-3f);
    const std::vector<Result> zero = RunFrames(60.0f, 10, 12.0f, target, 0.0f);
    Expect("weight 0: no correction", std::fabs(zero.back().cmdYaw) < 1e-7f && std::fabs(zero.back().errAfter - zero.back().errBefore) < 1e-4f);
}

static void TestNoOpAndFailClosed()
{
    Params prm;
    const Vec3 target = TargetAt(10.0f, 5.0f, prm);
    Controller c;
    Result r;
    // rig not ok
    Pose pose = SourcePose();
    const std::vector<uint8_t> before = pose.bytes;
    Rig bad = TestRig();
    bad.ok = 0;
    Expect("fail closed: rig not ok -> false, records bitwise unchanged",
           !c.Apply(pose.bytes.data(), pose.count, bad, prm, target, 1.0f, 0.016f, r) && pose.bytes == before);
    Rig big = TestRig();
    Expect("fail closed: FK range beyond the bone count -> unchanged",
           !c.Apply(pose.bytes.data(), kTestBones - 1, big, prm, target, 1.0f, 0.016f, r) && pose.bytes == before);
    Expect("fail closed: non-finite target -> unchanged",
           !c.Apply(pose.bytes.data(), pose.count, big, prm, Vec3{std::nanf(""), 0, 0}, 1.0f, 0.016f, r) && pose.bytes == before);
    Params badAxis = prm;
    badAxis.barrelAxis = 0;
    Expect("fail closed: invalid barrel axis -> unchanged",
           !c.Apply(pose.bytes.data(), pose.count, big, badAxis, target, 1.0f, 0.016f, r) && pose.bytes == before);
    Pose nanPose = SourcePose();
    reinterpret_cast<float*>(nanPose.bytes.data() + size_t(kRFore) * 48)[5] = std::nanf("");
    const std::vector<uint8_t> nanBefore = nanPose.bytes;
    Expect("fail closed: non-finite bone record -> unchanged",
           !c.Apply(nanPose.bytes.data(), nanPose.count, big, prm, target, 1.0f, 0.016f, r) &&
               std::memcmp(nanPose.bytes.data(), nanBefore.data(), nanBefore.size()) == 0);
    Expect("fail closed: a failure resets the smoothing state", !c.Has());
    // one-handed: the left arm is not touched
    Pose one = SourcePose(false);
    const std::vector<uint8_t> oneBefore = one.bytes;
    Params snap = prm;
    snap.smoothHz = 0.0f;
    Controller c1;
    const bool ok = c1.Apply(one.bytes.data(), one.count, TestRig(), snap, target, 1.0f, 0.016f, r);
    Expect("one-handed: corrected, left arm records bitwise unchanged",
           ok && !r.twoHanded && r.errAfter * kRad2Deg < 0.05f &&
               std::memcmp(one.bytes.data() + size_t(kLArm) * 48, oneBefore.data() + size_t(kLArm) * 48, 3 * 48) == 0);
}

static void TestGuards()
{
    GuardInput g;
    g.targetEnabled = true;
    g.targetValid = true;
    g.targetAge = 0.016f;
    g.feedAge = 0.016f;
    g.tpp = true;
    g.psm = uint32_t(integ::kPsmUpperBodyAim + 1) << integ::kPsmUpperShift;
    g.maskStates = 7;
    g.sinceMelee = -1.0f;
    g.upperScale = 0.0f;
    g.mmWeight = 1.0f;
    g.rigOk = true;
    Expect("guard: all conditions met -> 0", ComputeGuard(g) == 0, Fmt("0x%X", ComputeGuard(g)));
    auto with = [&](auto aMod) {
        GuardInput x = g;
        aMod(x);
        return ComputeGuard(x);
    };
    Expect("guard: target disabled", with([](GuardInput& x) { x.targetEnabled = false; }) == kGuardNoTarget);
    Expect("guard: target invalid", with([](GuardInput& x) { x.targetValid = false; }) == kGuardTargetInvalid);
    Expect("guard: target older than 0.25 s", with([](GuardInput& x) { x.targetAge = 0.26f; }) == kGuardTargetStale);
    Expect("guard: target 0.24 s old is fine", with([](GuardInput& x) { x.targetAge = 0.24f; }) == 0);
    Expect("guard: feed stale", with([](GuardInput& x) { x.feedAge = 0.5f; }) == kGuardFeedStale);
    Expect("guard: TPP off", with([](GuardInput& x) { x.tpp = false; }) == kGuardTppOff);
    Expect("guard: UpperBody not Aim (5 or not reported)",
           with([](GuardInput& x) { x.psm = uint32_t(5 + 1) << integ::kPsmUpperShift; }) == kGuardNotAim &&
               with([](GuardInput& x) { x.psm = 0; }) == kGuardNotAim);
    Expect("guard: mask state bit 2 off", with([](GuardInput& x) { x.maskStates = 5; }) == kGuardMaskStateOff);
    Expect("guard: melee bits now", (with([](GuardInput& x) { x.psm |= integ::kPsmMeleePose; }) & kGuardMelee) != 0);
    Expect("guard: melee 0.2 s ago blocks, 0.4 s ago does not",
           with([](GuardInput& x) { x.sinceMelee = 0.2f; }) == kGuardMelee && with([](GuardInput& x) { x.sinceMelee = 0.4f; }) == 0);
    Expect("guard: vehicle", (with([](GuardInput& x) { x.psm |= integ::kPsmVehicle; }) & kGuardVehicle) != 0);
    Expect("guard: other PSM block (scene)", with([](GuardInput& x) { x.psm |= integ::kPsmScene; }) == kGuardPsmBlock);
    Expect("guard: AimWalk detailed bit alone is fine", with([](GuardInput& x) {
               x.psm |= integ::kPsmDetailed | (uint32_t(integ::kPsmDetailAimWalk + 1) << integ::kPsmDetailShift);
           }) == 0);
    Expect("guard: mask inactive (upperScale 1) / MM weight 0",
           with([](GuardInput& x) { x.upperScale = 1.0f; }) == kGuardMaskInactive &&
               with([](GuardInput& x) { x.mmWeight = 0.0f; }) == kGuardMaskInactive &&
               with([](GuardInput& x) { x.upperScale = std::nanf(""); }) == kGuardMaskInactive);
    Expect("guard: rig not resolved", with([](GuardInput& x) { x.rigOk = false; }) == kGuardRig);
    Expect("guard: defaults (nothing set) block", ComputeGuard(GuardInput{}) != 0);
}

int main()
{
    TestFk();
    TestMath();
    TestRigResolve();
    TestSolveWithinLimits();
    TestLimits();
    TestSmoothing();
    TestNoOpAndFailClosed();
    TestGuards();
    std::printf("utpp_aimoffset_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

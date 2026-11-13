// Offline tests of the head look layer (src/mm/HeadLook.hpp), not shipped.
// Usage: utpp_headlook_test   Exit code 0 = all checks passed.
// Rig: the real player rig 0..32 (names + parents of the player skeleton as in tests/MMDatabaseSelfTest.cpp: Spine3 13, Neck 16,
// Neck1 19, Head 22, LeftEye 31, RightEye 32) with plausible bone offsets and a clip-like standing pose (Head local -Y = model +Y).
// Covers: rig resolution (fail closed, eyes optional), the gaze (left / right / up / down, clamps, the fade behind V, continuity at
// 180 deg, too near / non-finite), the spring (exact, frame-rate independent, critically damped), the layer on the records (turn
// reaches the gaze, distribution over Neck / Neck1 / Head 25 / 25 / 50 %, chest share and the weapon rule, limits, eyes, bone lengths,
// untouched bones), weight 0 / gate 0 / idle = bit-identical pose, ease in / out continuity at 30 / 60 / 144 fps, no pop when the target
// switches between camera and face, composition with a head that the head lead already turned (no double counting), the concurrent
// re-apply, fail closed.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/HeadLook.hpp"
#include "../src/mm/HeadLookRuntime.hpp"

extern "C" __declspec(dllimport) int __stdcall QueryPerformanceCounter(long long* aCount);
extern "C" __declspec(dllimport) void __stdcall Sleep(unsigned long aMs);

using namespace utpp::mm;
using namespace utpp::mm::headlook;
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

static constexpr float kR2D = 57.2957795f;

// ------------------------------------------------------------------------------------------------ the player rig 0..32

static const char* kNames[] = {"Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1",
                               "LeftLeg", "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder",
                               "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head", "LeftHand",
                               "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight", "LeftHeel", "RightHeel",
                               "LeftEye", "RightEye"};
static const int16_t kParents[] = {-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                                   23, 23, 24, 24, 11, 12, 22, 22};
static constexpr uint32_t kN = 33;
enum : uint32_t
{
    kHips = 2, kSpine3 = 13, kNeck = 16, kNeck1 = 19, kHead = 22, kLeftHand = 23, kRightHand = 24, kWeaponRight = 28, kLeftEye = 31,
    kRightEye = 32, kLeftShoulder = 14
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

static Quat Axis(Vec3 aAxis, float aDeg) { return aim::AxisAngle(aAxis, aDeg / kR2D); }
static const Vec3 kX{1, 0, 0}, kY{0, 1, 0}, kZ{0, 0, 1};

// A calm standing pose, Z up, forward +Y. The Hips carry a half turn about Z so that the bone-local -Y of the chain is the model +Y
// (as in the real clips); a slight lean in Spine1 / Spine2; the head tilted 4 deg. Eyes: identity local rotation (as written by the mod).
static Pose MakePose(float aHeadYawDeg = 0.0f)
{
    Pose p;
    p.bytes.assign(size_t(kN) * integ::kGameBoneStride, 0);
    const Quat id{};
    const Vec3 off[kN] = {
        {0, 0, 0},         {0, 0, 0},        {0, 0, 1.0f},       {0, 0, 0},         {0, 0, 0.10f},      {-0.10f, 0, -0.05f},
        {0.10f, 0, -0.05f}, {0, 0, 0.12f},   {0, 0, -0.45f},     {0, 0, -0.45f},    {0, 0, 0.12f},      {0, 0, -0.42f},
        {0, 0, -0.42f},    {0, 0, 0.12f},    {-0.04f, 0, 0.16f}, {0.04f, 0, 0.16f}, {0, 0.01f, 0.18f}, {-0.13f, 0, 0},
        {0.13f, 0, 0},     {0, 0, 0.05f},    {-0.28f, 0, 0},     {0.28f, 0, 0},     {0, 0, 0.08f},      {-0.25f, 0, 0},
        {0.25f, 0, 0},     {-0.08f, 0, 0},   {-0.05f, 0.03f, 0}, {0.08f, 0, 0},     {0.05f, 0.03f, 0},  {0, 0, -0.02f},
        {0, 0, -0.02f},    {-0.03f, -0.07f, 0.03f}, {0.03f, -0.07f, 0.03f}};
    for (uint32_t i = 0; i < kN; ++i)
        SetLocal(p, i, off[i], id);
    SetLocal(p, kHips, off[kHips], Axis(kZ, 180.0f));
    SetLocal(p, 7, off[7], Axis(kX, -4.0f));
    SetLocal(p, 10, off[10], Axis(kX, -3.0f));
    SetLocal(p, kSpine3, off[kSpine3], Axis(kZ, 3.0f));
    SetLocal(p, 17, off[17], Axis(kY, 60.0f));
    SetLocal(p, 18, off[18], Axis(kY, -60.0f));
    // head: tilt about the local X and an optional yaw about the model vertical (what a head lead would do)
    const Quat tilt = Axis(kX, 4.0f);
    SetLocal(p, kHead, off[kHead], tilt);
    if (aHeadYawDeg != 0.0f)
    {
        aim::Xf model[kN];
        aim::ComputeModel(p.bytes.data(), kN, kParents, kN, model);
        aim::RotateAboutJoint(p.bytes.data(), model, kParents, kHead, Axis(kZ, aHeadYawDeg));
    }
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

static bool Same(const Pose& a, const Pose& b) { return a.bytes == b.bytes; }

static float HeadYawDeg(const Pose& aPose)
{
    aim::Xf m[kN];
    Model(aPose, m);
    float y = 0, p = 0;
    aim::YawPitchOf(math::QuatRotate(m[kHead].q, kLocalForward), y, p);
    return y * kR2D;
}
static float HeadPitchDeg(const Pose& aPose)
{
    aim::Xf m[kN];
    Model(aPose, m);
    float y = 0, p = 0;
    aim::YawPitchOf(math::QuatRotate(m[kHead].q, kLocalForward), y, p);
    return p * kR2D;
}
// yaw of a bone's model-space local -Y axis (deg)
static float BoneYawDeg(const Pose& aPose, uint32_t aBone)
{
    aim::Xf m[kN];
    Model(aPose, m);
    float y = 0, p = 0;
    aim::YawPitchOf(math::QuatRotate(m[aBone].q, kLocalForward), y, p);
    return y * kR2D;
}

static Vec3 Dir(float aYawDeg, float aPitchDeg, float aDist)
{
    const float y = aYawDeg / kR2D, p = aPitchDeg / kR2D;
    return Vec3{-std::sin(y) * std::cos(p) * aDist, std::cos(y) * std::cos(p) * aDist, std::sin(p) * aDist};
}

static Vec3 HeadPos(const Pose& aPose)
{
    aim::Xf m[kN];
    Model(aPose, m);
    return m[kHead].p;
}

static Input MakeInput(const Pose& aBase, float aYawDeg, float aPitchDeg, float aDt)
{
    Input in;
    in.targetValid = true;
    in.target = HeadPos(aBase) + Dir(aYawDeg, aPitchDeg, 5.0f);
    in.weight = 1.0f;
    in.chest = 0.5f;
    in.gate = 1.0f;
    in.dt = aDt;
    return in;
}

// Runs the layer for aSeconds at aFps on copies of aBase (open loop: every call starts from the unmodified pose, like the FK detour).
// aOut = the pose after the last call.
static Output Run(Layer& aLayer, const Rig& aRig, const Pose& aBase, Input aIn, float aSeconds, float aFps, Pose& aOutPose)
{
    Output out;
    const int n = static_cast<int>(aSeconds * aFps + 0.5f);
    aIn.dt = 1.0f / aFps;
    for (int i = 0; i < n; ++i)
    {
        aOutPose = aBase;
        aLayer.Run(aOutPose.bytes.data(), kN, aRig, aIn, out);
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ tests

static void TestRig()
{
    const Rig r = RealRig();
    Expect("ResolveRig: player rig -> Spine3 13, Neck 16, Neck1 19, Head 22, eyes 31 / 32, fk 33",
           r.ok == 1 && r.spine3 == 13 && r.neck == 16 && r.neck1 == 19 && r.head == 22 && r.eye[0] == 31 && r.eye[1] == 32 &&
               r.fkCount == 33);
    std::vector<uint64_t> h;
    for (const char* n : kNames)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> par(kParents, kParents + kN);
    std::vector<uint64_t> h2 = h;
    h2[kHead] = integ::CNameHash("HeadX");
    Expect("ResolveRig: missing Head -> not ok", ResolveRig(h2, par).ok == 0);
    std::vector<uint64_t> h3 = h;
    h3[kLeftEye] = integ::CNameHash("LeftEyeX");
    h3[kRightEye] = integ::CNameHash("RightEyeX");
    const Rig noEyes = ResolveRig(h3, par);
    Expect("ResolveRig: no eye bones -> still ok, eyes -1", noEyes.ok == 1 && noEyes.eye[0] == -1 && noEyes.eye[1] == -1 && noEyes.fkCount == 23);
    std::vector<int16_t> p2 = par;
    p2[kNeck1] = 13; // Neck1 beside Neck, not below it
    Expect("ResolveRig: Neck1 not below Neck -> not ok", ResolveRig(h, p2).ok == 0);
    std::vector<int16_t> p3 = par;
    p3[kLeftEye] = 7; // eye under Spine1: left out, the rest stays
    const Rig oneEye = ResolveRig(h, p3);
    Expect("ResolveRig: an eye not below Head is left out", oneEye.ok == 1 && oneEye.eye[0] == -1 && oneEye.eye[1] == 32);
    std::vector<int16_t> p4 = par;
    p4[5] = 9; // parent after child
    Expect("ResolveRig: non-topological parents -> not ok", ResolveRig(h, p4).ok == 0);
    std::vector<int16_t> p5(kParents, kParents + kN - 1);
    Expect("ResolveRig: names / parents size mismatch -> not ok", ResolveRig(h, p5).ok == 0);
    Expect("ResolveRig: empty -> not ok", ResolveRig({}, {}).ok == 0);
}

static void TestGaze()
{
    const Vec3 head{0.0f, 0.05f, 1.62f};
    Gaze g = ComputeGaze(head, head + Dir(0, 0, 5), 100.0f);
    Expect("gaze: straight ahead -> yaw 0, pitch 0, no flags", g.valid && std::fabs(g.yaw) < 1e-6f && std::fabs(g.pitch) < 1e-6f && g.flags == 0);
    g = ComputeGaze(head, head + Dir(35, 0, 5), 100.0f);
    Expect("gaze: 35 deg to the left -> yaw +35 (left positive)", g.valid && std::fabs(g.yaw * kR2D - 35.0f) < 1e-3f);
    g = ComputeGaze(head, head + Dir(-50, 0, 5), 100.0f);
    Expect("gaze: 50 deg to the right -> yaw -50", g.valid && std::fabs(g.yaw * kR2D + 50.0f) < 1e-3f);
    g = ComputeGaze(head, head + Dir(0, 20, 5), 100.0f);
    Expect("gaze: 20 deg up -> pitch +20", g.valid && std::fabs(g.pitch * kR2D - 20.0f) < 1e-3f);
    g = ComputeGaze(head, head + Dir(0, -25, 5), 100.0f);
    Expect("gaze: 25 deg down -> pitch -25", g.valid && std::fabs(g.pitch * kR2D + 25.0f) < 1e-3f);
    g = ComputeGaze(head, head + Dir(0, 60, 5), 100.0f);
    Expect("gaze: 60 deg up -> clamped to +35, kFlagClampUp", std::fabs(g.pitch * kR2D - 35.0f) < 1e-3f && (g.flags & kFlagClampUp));
    g = ComputeGaze(head, head + Dir(0, -70, 5), 100.0f);
    Expect("gaze: 70 deg down -> clamped to -40, kFlagClampDown", std::fabs(g.pitch * kR2D + 40.0f) < 1e-3f && (g.flags & kFlagClampDown));
    g = ComputeGaze(head, head + Dir(110, 0, 5), 100.0f);
    Expect("gaze: 110 deg left -> clamped to maxYaw 100, kFlagClampYaw", std::fabs(g.yaw * kR2D - 100.0f) < 1e-3f && (g.flags & kFlagClampYaw) &&
           !(g.flags & kFlagBehind));
    g = ComputeGaze(head, head + Dir(-110, 0, 5), 80.0f);
    Expect("gaze: maxYaw 80 from the setting -> -80", std::fabs(g.yaw * kR2D + 80.0f) < 1e-3f);
    // behind: the fade runs from 150 to 180 deg (maxYaw 100 + 15 < 150)
    g = ComputeGaze(head, head + Dir(140, 0, 5), 100.0f);
    Expect("gaze: 140 deg (before the fade) -> clamped to 100, no fade", std::fabs(g.yaw * kR2D - 100.0f) < 1e-3f && !(g.flags & kFlagBehind));
    g = ComputeGaze(head, head + Dir(165.0f, 10, 5), 100.0f);
    Expect("gaze: 165 deg (halfway into the fade) -> yaw 50, pitch 5, kFlagBehind", std::fabs(g.yaw * kR2D - 50.0f) < 1e-2f &&
           std::fabs(g.pitch * kR2D - 5.0f) < 1e-2f && (g.flags & kFlagBehind) && std::fabs(g.fade - 0.5f) < 1e-4f,
           Fmt("y %g p %g fade %g", g.yaw * kR2D, g.pitch * kR2D, g.fade));
    g = ComputeGaze(head, head + Dir(179.9f, 0, 5), 100.0f);
    Expect("gaze: straight behind -> about 0", std::fabs(g.yaw * kR2D) < 0.5f);
    // continuity of the gaze angles over a full sweep of the target yaw through 180 deg
    float worst = 0.0f, prev = ComputeGaze(head, head + Dir(0, 0, 5), 100.0f).yaw;
    for (int a = 1; a <= 360; ++a)
    {
        const float y = ComputeGaze(head, head + Dir(static_cast<float>(a), 0, 5), 100.0f).yaw;
        worst = std::fmax(worst, std::fabs(y - prev) * kR2D);
        prev = y;
    }
    Expect("gaze: sweep 0..360 deg in 1 deg steps has no jump (largest step <= 3.5 deg = the fade slope, also across 180)", worst <= 3.5f, Fmt("worst %g", worst));
    g = ComputeGaze(head, Vec3{head.x + 0.1f, head.y, 3.0f}, 100.0f);
    Expect("gaze: target above the head, closer than 0.3 m horizontally -> invalid (kFlagNear)", !g.valid && (g.flags & kFlagNear));
    g = ComputeGaze(head, Vec3{std::nanf(""), 1, 1}, 100.0f);
    Expect("gaze: non-finite target -> invalid", !g.valid && (g.flags & kFlagNear));
    Expect("params: validation", ValidParams(100, 0.5f) && ValidParams(0, 0) && ValidParams(160, 1) && !ValidParams(-1, 0.5f) &&
           !ValidParams(100, 1.1f) && !ValidParams(161, 0.5f) && !ValidParams(std::nanf(""), 0.5f) && ValidMode(0) && ValidMode(2) &&
           !ValidMode(3) && !ValidMode(-1));
}

static void TestSpring()
{
    Spring s;
    const float target = 1.0f;
    float peak = 0.0f;
    for (int i = 0; i < 300; ++i)
    {
        s.Step(target, kHeadOmega, 1.0f / 100.0f);
        peak = std::fmax(peak, s.x);
    }
    Expect("spring: critically damped step response has no overshoot and settles", peak <= 1.0f + 1e-5f && std::fabs(s.x - 1.0f) < 1e-3f,
           Fmt("peak %g x %g", peak, s.x));
    // frame-rate independence: the same time, the same constant target, different partitions
    const float fps[] = {30.0f, 60.0f, 144.0f, 240.0f};
    float at[4];
    for (int k = 0; k < 4; ++k)
    {
        Spring q;
        const int n = static_cast<int>(0.5f * fps[k] + 0.5f);
        for (int i = 0; i < n; ++i)
            q.Step(target, kHeadOmega, 1.0f / fps[k]);
        at[k] = q.x;
    }
    float spread = 0.0f;
    for (int k = 1; k < 4; ++k)
        spread = std::fmax(spread, std::fabs(at[k] - at[0]));
    Expect("spring: x(0.5 s) is the same at 30 / 60 / 144 / 240 fps (exact solution)", spread < 1e-4f, Fmt("spread %g", spread));
    Spring r;
    r.x = 0.5f;
    r.v = 0.2f;
    for (int i = 0; i < 400; ++i)
        r.Step(0.0f, kHeadOmega, 1.0f / 60.0f);
    Expect("spring: released to target 0 ends at exactly 0 (rest)", r.x == 0.0f && r.v == 0.0f && r.Rest());
    Spring z;
    z.Step(1.0f, kHeadOmega, 0.0f);
    Expect("spring: dt 0 changes nothing", z.x == 0.0f && z.v == 0.0f);
}

static void TestTurn()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    const float y0 = HeadYawDeg(base);
    Expect("pose: the calm head looks forward (local -Y -> model +Y, within the 4 deg head tilt)", std::fabs(y0) < 5.0f && std::fabs(HeadPitchDeg(base)) < 8.0f,
           Fmt("yaw %g pitch %g", y0, HeadPitchDeg(base)));
    struct Case { const char* name; float yaw, pitch; };
    const Case cases[] = {{"left 40", 40, 0}, {"right 40", -40, 0}, {"left 60 up 20", 60, 20}, {"right 55 down 25", -55, -25}, {"up 30", 0, 30}};
    for (const Case& c : cases)
    {
        Layer layer;
        Pose out;
        const Input in = MakeInput(base, c.yaw, c.pitch, 1.0f / 60.0f);
        const Output o = Run(layer, rig, base, in, 2.0f, 60.0f, out);
        // head position moves a little with the turn, the target is far (5 m): allow 1.5 deg
        const float hy = HeadYawDeg(out), hp = HeadPitchDeg(out);
        const Gaze g = ComputeGaze(HeadPos(base), in.target, 100.0f);
        Expect((std::string("turn: ") + c.name + " -> head reaches the gaze").c_str(),
               std::fabs(hy - g.yaw * kR2D) < 1.5f && std::fabs(hp - g.pitch * kR2D) < 1.5f && o.wrote,
               Fmt("head %.2f/%.2f gaze %.2f/%.2f", hy, hp, g.yaw * kR2D, g.pitch * kR2D));
    }
    // sign: a target on the left turns the head to the left (positive yaw), on the right to the right
    {
        Layer l1, l2;
        Pose o1, o2;
        Run(l1, rig, base, MakeInput(base, 30, 0, 1.0f / 60.0f), 1.5f, 60.0f, o1);
        Run(l2, rig, base, MakeInput(base, -30, 0, 1.0f / 60.0f), 1.5f, 60.0f, o2);
        Expect("turn: left target -> positive head yaw, right target -> negative", HeadYawDeg(o1) > 20.0f && HeadYawDeg(o2) < -20.0f);
    }
}

static void TestDistribution()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Pose out;
    Input in = MakeInput(base, 60, 0, 1.0f / 60.0f);
    in.chestAllowed = false;
    const Output o = Run(layer, rig, base, in, 2.0f, 60.0f, out);
    const float chain = BoneYawDeg(out, kHead) - BoneYawDeg(base, kHead);
    const float dNeck = BoneYawDeg(out, kNeck) - BoneYawDeg(base, kNeck);
    const float dNeck1 = BoneYawDeg(out, kNeck1) - BoneYawDeg(base, kNeck1);
    // Neck turns by 25 % of the rotation, Neck1 by 25 % more (cumulative 50 %), Head by 50 % more (cumulative 100 %)
    Expect("distribution: Neck 25 %, Neck1 cumulative 50 %, Head cumulative 100 % of the chain turn",
           chain > 40.0f && std::fabs(dNeck / chain - 0.25f) < 0.03f && std::fabs(dNeck1 / chain - 0.50f) < 0.03f,
           Fmt("chain %g neck %g neck1 %g", chain, dNeck / chain, dNeck1 / chain));
    // the bones that must not move: everything but Neck, Neck1, Head (no weapon: Spine3 stays, and the eyes follow the head's frame)
    bool untouched = true;
    for (uint32_t b = 0; b < kN; ++b)
    {
        if (b == kNeck || b == kNeck1 || b == kHead || b == kLeftEye || b == kRightEye)
            continue;
        if (std::memcmp(out.bytes.data() + size_t(b) * integ::kGameBoneStride, base.bytes.data() + size_t(b) * integ::kGameBoneStride,
                        integ::kGameBoneStride) != 0)
            untouched = false;
    }
    Expect("distribution: no weapon rule -> only Neck / Neck1 / Head (and the eyes) change, every other record is bit-identical", untouched);
    // joint positions: local positions are never written
    bool posKept = true;
    for (uint32_t b = 0; b < kN; ++b)
        for (int k = 0; k < 3; ++k)
            if (std::memcmp(out.bytes.data() + size_t(b) * integ::kGameBoneStride + k * 4, base.bytes.data() + size_t(b) * integ::kGameBoneStride + k * 4, 4))
                posKept = false;
    Expect("distribution: local positions (bone lengths) untouched", posKept);
    aim::Xf mb[kN], mo[kN];
    Model(base, mb);
    Model(out, mo);
    Expect("distribution: bone lengths kept (Spine3 -> Neck, Neck -> Neck1, Neck1 -> Head), Neck joint does not move",
           std::fabs(aim::Length(mb[kNeck1].p - mb[kNeck].p) - aim::Length(mo[kNeck1].p - mo[kNeck].p)) < 1e-5f &&
               std::fabs(aim::Length(mb[kHead].p - mb[kNeck1].p) - aim::Length(mo[kHead].p - mo[kNeck1].p)) < 1e-5f &&
               aim::Length(mb[kNeck].p - mo[kNeck].p) < 1e-6f);
    Expect("distribution: the output telemetry matches (chest 0 with a weapon, chain yaw = the turn)", o.chestYaw == 0.0f &&
           std::fabs(o.yaw * kR2D - chain) < 0.5f, Fmt("yaw %g chain %g", o.yaw * kR2D, chain));
}

static void TestChestRule()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer a, b;
    Pose outFree, outWeapon;
    Input in = MakeInput(base, 60, 0, 1.0f / 60.0f);
    in.chestAllowed = true;
    const Output oa = Run(a, rig, base, in, 2.0f, 60.0f, outFree);
    in.chestAllowed = false;
    Run(b, rig, base, in, 2.0f, 60.0f, outWeapon);
    const float chestTurn = BoneYawDeg(outFree, kSpine3) - BoneYawDeg(base, kSpine3);
    Expect("chest: no weapon, chest 0.5 -> Spine3 takes about 15 % of the yaw (chest * 0.30)",
           chestTurn > 5.0f && chestTurn < 12.0f && std::fabs(oa.chestYaw * kR2D - chestTurn) < 0.5f, Fmt("chestTurn %g", chestTurn));
    Expect("chest: weapon in the hands -> Spine3 record bit-identical (arms, WeaponRight, helpers stay)",
           std::memcmp(outWeapon.bytes.data() + size_t(kSpine3) * integ::kGameBoneStride, base.bytes.data() + size_t(kSpine3) * integ::kGameBoneStride,
                       integ::kGameBoneStride) == 0 &&
               std::memcmp(outWeapon.bytes.data() + size_t(kWeaponRight) * integ::kGameBoneStride,
                           base.bytes.data() + size_t(kWeaponRight) * integ::kGameBoneStride, integ::kGameBoneStride) == 0);
    // chest slider 0 / 1
    Layer c0, c1;
    Pose o0, o1;
    in.chestAllowed = true;
    in.chest = 0.0f;
    Run(c0, rig, base, in, 2.0f, 60.0f, o0);
    in.chest = 1.0f;
    Run(c1, rig, base, in, 2.0f, 60.0f, o1);
    const float t0 = BoneYawDeg(o0, kSpine3) - BoneYawDeg(base, kSpine3), t1 = BoneYawDeg(o1, kSpine3) - BoneYawDeg(base, kSpine3);
    Expect("chest: slider 0 -> Spine3 untouched, slider 1 -> about 30 % (twice the default)", std::fabs(t0) < 1e-3f && t1 > 12.0f && t1 < 20.0f &&
           std::fabs(HeadYawDeg(o0) - HeadYawDeg(o1)) < 1.5f, Fmt("t0 %g t1 %g", t0, t1));
    // the chest share eases when a weapon is drawn mid-look: no jump
    Layer sw;
    Pose p;
    in.chest = 0.5f;
    in.chestAllowed = true;
    Run(sw, rig, base, in, 1.5f, 60.0f, p);
    float prev = BoneYawDeg(p, kSpine3), worst = 0.0f;
    in.chestAllowed = false;
    in.dt = 1.0f / 60.0f;
    Output o;
    for (int i = 0; i < 120; ++i)
    {
        p = base;
        sw.Run(p.bytes.data(), kN, rig, in, o);
        const float y = BoneYawDeg(p, kSpine3);
        worst = std::fmax(worst, std::fabs(y - prev));
        prev = y;
    }
    Expect("chest: drawing a weapon mid-look eases the chest out (largest frame step <= 1.5 deg = 90 deg/s at 60 fps), ends at the clip pose",
           worst <= 1.5f && std::fabs(prev - BoneYawDeg(base, kSpine3)) < 0.05f, Fmt("worst %g end %g", worst, prev - BoneYawDeg(base, kSpine3)));
}

static void TestLimits()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Pose out;
    Input in = MakeInput(base, 100, 0, 1.0f / 60.0f);
    in.chestAllowed = false;
    const Output o = Run(layer, rig, base, in, 3.0f, 60.0f, out);
    const float chain = HeadYawDeg(out) - HeadYawDeg(base);
    Expect("limits: gaze 100 deg left without a chest share -> the neck chain stops at 75 deg, kFlagChain, the eyes take the rest (about 22 deg: the clip head starts 3 deg to the left)",
           chain > 74.0f && chain < 76.0f && (o.flags & kFlagChain) && std::fabs(o.eyeYaw * kR2D - 22.0f) < 2.0f,
           Fmt("chain %g eyes %g flags %g", chain, o.eyeYaw * kR2D, o.flags));
    Layer l2;
    Pose out2;
    Input in2 = MakeInput(base, 0, 40, 1.0f / 60.0f);
    in2.chestAllowed = false;
    const Output o2 = Run(l2, rig, base, in2, 3.0f, 60.0f, out2);
    const float dp = HeadPitchDeg(out2) - HeadPitchDeg(base);
    Expect("limits: gaze 40 deg up (clamped to 35) -> chain pitch 30, eyes the rest (about 2 deg: the clip head looks 3 deg up)", dp > 29.0f && dp < 31.0f &&
           std::fabs(o2.eyePitch * kR2D - 2.0f) < 1.0f, Fmt("dp %g eyes %g", dp, o2.eyePitch * kR2D));
    // eye limits: a gaze far beyond what the chain and the eyes can do ends at the eye limit
    Layer l3;
    Pose out3;
    Input in3 = MakeInput(base, 100, 0, 1.0f / 60.0f);
    in3.chestAllowed = false;
    in3.maxYawDeg = 160.0f;
    in3.target = HeadPos(base) + Dir(150, 0, 5.0f);
    const Output o3 = Run(l3, rig, base, in3, 3.0f, 60.0f, out3);
    Expect("limits: eyes never exceed +-30 deg yaw", std::fabs(o3.eyeYaw * kR2D) <= 30.01f && (o3.flags & kFlagEyes), Fmt("eyes %g", o3.eyeYaw * kR2D));
    // maxYaw from the setting
    Layer l4;
    Pose out4;
    Input in4 = MakeInput(base, 90, 0, 1.0f / 60.0f);
    in4.maxYawDeg = 60.0f;
    Run(l4, rig, base, in4, 3.0f, 60.0f, out4);
    Expect("limits: maxYaw 60 -> the head ends at 60 deg", std::fabs(HeadYawDeg(out4) - 60.0f) < 1.5f, Fmt("yaw %g", HeadYawDeg(out4)));
}

static void TestEyes()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Pose out;
    Input in = MakeInput(base, 100, 0, 1.0f / 60.0f);
    in.chestAllowed = false;
    Run(layer, rig, base, in, 3.0f, 60.0f, out);
    const float headBefore = BoneYawDeg(out, kHead);
    // the eyes turn further than the head by the residual: model yaw of the eye = head yaw + eyeYaw
    aim::Xf m[kN];
    Model(out, m);
    float ey = 0, ep = 0;
    aim::YawPitchOf(math::QuatRotate(m[kLeftEye].q, kLocalForward), ey, ep);
    Expect("eyes: left and right turn by the same rotation, beyond the head by the residual",
           ey * kR2D - headBefore > 20.0f && std::fabs(BoneYawDeg(out, kLeftEye) - BoneYawDeg(out, kRightEye)) < 1e-3f,
           Fmt("eye %g head %g", ey * kR2D, headBefore));
    // the eyes alone (a gaze inside the chain's range) stay with the head: no residual
    Layer l2;
    Pose o2;
    Input in2 = MakeInput(base, 30, 5, 1.0f / 60.0f);
    Run(l2, rig, base, in2, 3.0f, 60.0f, o2);
    Expect("eyes: a gaze the head reaches leaves no residual (eyes still in the head frame)",
           std::fabs(BoneYawDeg(o2, kLeftEye) - BoneYawDeg(o2, kHead)) < 0.5f);
    // no eye bones in the rig: the head still works, nothing is written to eye records
    std::vector<uint64_t> h;
    for (const char* n : kNames)
        h.push_back(integ::CNameHash(n));
    h[kLeftEye] = integ::CNameHash("x1");
    h[kRightEye] = integ::CNameHash("x2");
    std::vector<int16_t> par(kParents, kParents + kN);
    const Rig noEyes = ResolveRig(h, par);
    Layer l3;
    Pose o3;
    Input in3 = MakeInput(base, 100, 0, 1.0f / 60.0f);
    Run(l3, noEyes, base, in3, 3.0f, 60.0f, o3);
    Expect("eyes: rig without eye bones -> head turns, eye records bit-identical",
           HeadYawDeg(o3) > 60.0f &&
               std::memcmp(o3.bytes.data() + size_t(kLeftEye) * integ::kGameBoneStride, base.bytes.data() + size_t(kLeftEye) * integ::kGameBoneStride,
                           integ::kGameBoneStride) == 0);
}

static void TestIdentity()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Output o;
    Pose p = base;
    Input in = MakeInput(base, 40, 0, 1.0f / 60.0f);
    in.weight = 0.0f;
    layer.Run(p.bytes.data(), kN, rig, in, o);
    Expect("identity: weight 0 from the start -> bit-identical pose, wrote = false", Same(p, base) && !o.wrote && layer.Idle());
    in.weight = 1.0f;
    in.targetValid = false;
    p = base;
    layer.Run(p.bytes.data(), kN, rig, in, o);
    Expect("identity: no valid target -> bit-identical pose", Same(p, base) && !o.wrote);
    in.targetValid = true;
    in.gate = 0.0f;
    p = base;
    layer.Run(p.bytes.data(), kN, rig, in, o);
    Expect("identity: gate 0 (the game owns the head) -> bit-identical pose", Same(p, base) && !o.wrote);
    // ease in, then release (weight 0): fades to exactly the clip pose
    in.gate = 1.0f;
    Pose mid;
    Run(layer, rig, base, in, 1.0f, 60.0f, mid);
    Expect("identity: the look is on after 1 s (the pose differs)", !Same(mid, base) && !layer.Idle());
    in.weight = 0.0f;
    Pose after;
    Run(layer, rig, base, in, 2.5f, 60.0f, after);
    Expect("identity: weight 0 for 2.5 s -> the layer is at rest and the pose is bit-identical again", Same(after, base) && layer.Idle());
    // a rig that is not ok writes nothing and fails only when there is something to do
    Layer l2;
    Rig bad;
    p = base;
    Expect("identity: an idle layer with a rig that is not ok succeeds without touching the pose", l2.Run(p.bytes.data(), kN, bad, in, o) && Same(p, base));
}

static void TestEasing()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    float endYaw[3] = {};
    float worstStep[3] = {};
    float worstAccel[3] = {};
    for (int k = 0; k < 3; ++k)
    {
        Layer layer;
        Input in = MakeInput(base, 70, 0, 1.0f / fpsList[k]);
        in.chestAllowed = false;
        Output o;
        float prev = HeadYawDeg(base), prevStep = 0.0f;
        const int n = static_cast<int>(1.0f * fpsList[k]);
        Pose p;
        for (int i = 0; i < n; ++i)
        {
            p = base;
            layer.Run(p.bytes.data(), kN, rig, in, o);
            const float y = HeadYawDeg(p);
            const float step = (y - prev) * fpsList[k]; // deg / s
            worstStep[k] = std::fmax(worstStep[k], std::fabs(step));
            if (i > 0)
                worstAccel[k] = std::fmax(worstAccel[k], std::fabs(step - prevStep) * fpsList[k]);
            prevStep = step;
            prev = y;
        }
        endYaw[k] = HeadYawDeg(p);
    }
    Expect("ease-in: after 1 s the head yaw is the same at 30 / 60 / 144 fps (within 1 deg)",
           std::fabs(endYaw[0] - endYaw[1]) < 1.0f && std::fabs(endYaw[1] - endYaw[2]) < 1.0f,
           Fmt("%g %g %g", endYaw[0], endYaw[1], endYaw[2]));
    Expect("ease-in: the speed is bounded (<= 420 deg/s at every frame rate) and starts without a jump (acceleration <= omega^2 * 70 deg ~ 4500 deg/s^2)",
           worstStep[0] < 420.0f && worstStep[1] < 420.0f && worstStep[2] < 420.0f && worstAccel[0] < 5500.0f && worstAccel[1] < 5500.0f &&
               worstAccel[2] < 5500.0f, Fmt("speed %g %g %g", worstStep[0], worstStep[1], worstStep[2]));
    // release: monotone return, no overshoot past the clip pose, same duration at every frame rate
    float released[3] = {};
    bool monotone = true;
    for (int k = 0; k < 3; ++k)
    {
        Layer layer;
        Pose p;
        Input in = MakeInput(base, 70, 0, 1.0f / fpsList[k]);
        Run(layer, rig, base, in, 1.5f, fpsList[k], p);
        in.weight = 0.0f;
        Output o;
        float prev = HeadYawDeg(p);
        const int n = static_cast<int>(0.5f * fpsList[k]);
        for (int i = 0; i < n; ++i)
        {
            p = base;
            layer.Run(p.bytes.data(), kN, rig, in, o);
            const float y = HeadYawDeg(p);
            if (y > prev + 1e-3f || y < HeadYawDeg(base) - 1e-3f)
                monotone = false;
            prev = y;
        }
        released[k] = prev;
    }
    Expect("ease-out: release is monotone (no overshoot) and equal after 0.5 s at 30 / 60 / 144 fps", monotone &&
           std::fabs(released[0] - released[1]) < 1.0f && std::fabs(released[1] - released[2]) < 1.0f,
           Fmt("%g %g %g", released[0], released[1], released[2]));
}

static void TestModeSwitch()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Pose p;
    Input in = MakeInput(base, 50, 0, 1.0f / 60.0f); // camera mode
    in.mode = kModeCamera;
    Run(layer, rig, base, in, 1.5f, 60.0f, p);
    // face mode: the target jumps to the other side
    in.mode = kModeFace;
    in.target = HeadPos(base) + Dir(-30, 5, 3.0f);
    Output o;
    float prev = HeadYawDeg(p), worst = 0.0f;
    for (int i = 0; i < 90; ++i)
    {
        p = base;
        layer.Run(p.bytes.data(), kN, rig, in, o);
        const float y = HeadYawDeg(p);
        worst = std::fmax(worst, std::fabs(y - prev));
        prev = y;
    }
    Expect("switch: camera -> face target 80 deg away: no pop (largest frame step <= 4 deg at 60 fps), arrives", worst <= 4.0f &&
           std::fabs(prev + 30.0f) < 2.0f && o.mode == kModeFace, Fmt("worst %g end %g", worst, prev));
    // gate closes mid-look (aiming / melee takes over): the output scales down with the gate, no step
    Layer g;
    Run(g, rig, base, MakeInput(base, 50, 0, 1.0f / 60.0f), 1.5f, 60.0f, p);
    Input gi = MakeInput(base, 50, 0, 1.0f / 60.0f);
    prev = HeadYawDeg(p);
    worst = 0.0f;
    for (int i = 0; i < 12; ++i) // the mask fades in 0.2 s
    {
        gi.gate = 1.0f - float(i + 1) / 12.0f;
        p = base;
        g.Run(p.bytes.data(), kN, rig, gi, o);
        const float y = HeadYawDeg(p);
        worst = std::fmax(worst, std::fabs(y - prev));
        prev = y;
    }
    Expect("switch: the gate closing over 0.2 s fades the turn out smoothly (largest frame step <= 6 deg) and ends at the clip pose",
           worst <= 6.0f && std::fabs(prev - HeadYawDeg(base)) < 0.2f, Fmt("worst %g end %g", worst, prev));
}

static void TestHeadLeadComposition()
{
    // The head lead of the motion driver has already turned the head by +20 deg in the published pose; the layer measures the head
    // the pose really has, so the result is the same direction as for the clip pose (no double counting).
    const Rig rig = RealRig();
    const Pose clip = MakePose(0.0f);
    const Pose led = MakePose(20.0f);
    Layer a, b;
    Pose oa, ob;
    Input ia = MakeInput(clip, 45, 8, 1.0f / 60.0f), ib = ia;
    ia.chestAllowed = false;
    ib.chestAllowed = false;
    Run(a, rig, clip, ia, 2.0f, 60.0f, oa);
    Run(b, rig, led, ib, 2.0f, 60.0f, ob);
    Expect("composition: a head already turned by the head lead (+20 deg) ends at the same direction as the clip head (45 deg)",
           std::fabs(HeadYawDeg(oa) - HeadYawDeg(ob)) < 1.0f && std::fabs(HeadYawDeg(ob) - 45.0f) < 1.5f &&
               std::fabs(HeadPitchDeg(oa) - HeadPitchDeg(ob)) < 1.0f, Fmt("clip %g led %g", HeadYawDeg(oa), HeadYawDeg(ob)));
    // and while the gaze is straight ahead the layer turns the lead back out (the look wins), also not a pop
    Layer c;
    Pose oc;
    Input ic = MakeInput(led, 0, 0, 1.0f / 60.0f);
    ic.chestAllowed = false;
    Run(c, rig, led, ic, 2.0f, 60.0f, oc);
    Expect("composition: gaze straight ahead on a led head (+20 deg) -> head ends forward", std::fabs(HeadYawDeg(oc)) < 1.5f, Fmt("yaw %g", HeadYawDeg(oc)));
}

static void TestGate()
{
    Expect("gate: full MM ownership, no melee -> 1", std::fabs(LayerGate(1.0f, 1.0f, 0.0f) - 1.0f) < 1e-6f);
    Expect("gate: MM weight 0.5 (hand back) -> 0.5", std::fabs(LayerGate(0.5f, 1.0f, 0.0f) - 0.5f) < 1e-6f);
    Expect("gate: upper body from the game (aim / melee mask, upperScale 0) -> 0", LayerGate(1.0f, 0.0f, 0.0f) == 0.0f);
    Expect("gate: melee fade 0.25 -> 0.75", std::fabs(LayerGate(1.0f, 1.0f, 0.25f) - 0.75f) < 1e-6f);
    Expect("gate: melee fully on -> 0; MM weight 0 -> 0", LayerGate(1.0f, 1.0f, 1.0f) == 0.0f && LayerGate(0.0f, 1.0f, 0.0f) == 0.0f);
    Expect("gate: non-finite inputs -> within [0,1]", LayerGate(std::nanf(""), 1.0f, 0.0f) == 0.0f &&
           LayerGate(1.0f, std::nanf(""), std::nanf("")) >= 0.0f && LayerGate(1.0f, std::nanf(""), std::nanf("")) <= 1.0f);
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer l, half;
    Pose o1, o2;
    Input in = MakeInput(base, 60, 0, 1.0f / 60.0f);
    in.chestAllowed = false;
    Run(l, rig, base, in, 2.0f, 60.0f, o1);
    in.gate = 0.5f;
    Run(half, rig, base, in, 2.0f, 60.0f, o2);
    Expect("gate: 0.5 -> half of the turn", std::fabs((HeadYawDeg(o2) - HeadYawDeg(base)) - 0.5f * (HeadYawDeg(o1) - HeadYawDeg(base))) < 1.0f,
           Fmt("full %g half %g", HeadYawDeg(o1), HeadYawDeg(o2)));
}

static void TestReapply()
{
    // the concurrent animation job re-applies the stored output on its own fresh pose: same result as the owner's call
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Pose owner;
    Input in = MakeInput(base, 60, 10, 1.0f / 60.0f);
    in.chestAllowed = true;
    const Output o = Run(layer, rig, base, in, 1.5f, 60.0f, owner);
    Pose other = base;
    const bool ok = Layer::WriteOutput(other.bytes.data(), kN, rig, o);
    Expect("reapply: WriteOutput on a fresh pose equals the owner's pose bit for bit", ok && Same(owner, other));
    Pose none = base;
    Output zero;
    Expect("reapply: a zero output writes nothing", Layer::WriteOutput(none.bytes.data(), kN, rig, zero) && Same(none, base));
}

static void TestFailClosed()
{
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Output o;
    Input in = MakeInput(base, 40, 0, 1.0f / 60.0f);
    Pose bad = base;
    float* q = reinterpret_cast<float*>(bad.bytes.data() + size_t(kNeck) * integ::kGameBoneStride) + 4;
    q[0] = std::nanf("");
    const Pose before = bad;
    const bool ok = layer.Run(bad.bytes.data(), kN, rig, in, o);
    // NaN != NaN: compare the bytes
    Expect("fail closed: a non-finite record -> false, nothing written", !ok && !o.wrote && std::memcmp(bad.bytes.data(), before.bytes.data(), bad.bytes.size()) == 0);
    Pose p = base;
    Expect("fail closed: bone count below the rig's FK range -> false", !layer.Run(p.bytes.data(), 20, rig, in, o) && Same(p, base));
    Expect("fail closed: no pose pointer -> false", !layer.Run(nullptr, kN, rig, in, o));
    // a NaN target never reaches the springs
    Input nan = in;
    nan.target = Vec3{std::nanf(""), 0, 0};
    Layer l2;
    p = base;
    l2.Run(p.bytes.data(), kN, rig, nan, o);
    Expect("fail closed: a non-finite target -> no gaze, pose bit-identical", Same(p, base) && l2.Idle());
}

static void TestWalkingSway()
{
    // the spring smooths the DELTA, so a clip sway above the spring bandwidth stays in the pose: a head that sways +-3 deg at 2 Hz
    // still sways (about the same amplitude) while the low part follows the target
    const Rig rig = RealRig();
    const Pose base = MakePose();
    Layer layer;
    Input in = MakeInput(base, 30, 0, 1.0f / 60.0f);
    in.chestAllowed = false;
    Output o;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 240; ++i)
    {
        Pose p = MakePose(3.0f * std::sin(6.2831853f * 2.0f * float(i) / 60.0f));
        in.target = HeadPos(p) + Dir(30, 0, 5.0f);
        layer.Run(p.bytes.data(), kN, rig, in, o);
        if (i >= 120)
        {
            const float y = HeadYawDeg(p);
            lo = std::fmin(lo, y);
            hi = std::fmax(hi, y);
        }
    }
    Expect("sway: a +-3 deg clip sway at 2 Hz is kept in the pose while it looks 30 deg left (amplitude >= 1 deg, mean near 30)",
           (hi - lo) * 0.5f > 1.0f && (hi + lo) * 0.5f > 25.0f && (hi + lo) * 0.5f < 35.0f, Fmt("lo %g hi %g", lo, hi));
}

// The PRODUCTION glue (src/mm/HeadLook.cpp: OnArmed, SetTarget, Apply as FkHook.cpp calls them) on real time. The first in-game run
// (2026-11-13) had the gaze at -97 deg and nothing applied: the glue restarted its step clock on every call while the springs were at
// rest, so every step had dt = 0. The layer tests above pass dt themselves and could not see it.
static void TestGlue()
{
    std::vector<uint64_t> h;
    for (const char* n : kNames)
        h.push_back(integ::CNameHash(n));
    std::vector<int16_t> par(kParents, kParents + kN);
    int rigTag = 0;
    utpp::headlook::OnArmed(&rigTag, h, par);
    Expect("glue: rig resolved at arm time", utpp::headlook::RigOk());
    const float target[3] = {-4.0f, 4.0f, 1.6f}; // 45 deg to one side, eye height, 5.7 m away
    Pose p = MakePose();
    const float rest = HeadYawDeg(p);
    for (int i = 0; i < 60; ++i)
    {
        const bool accepted = utpp::headlook::SetTarget(true, true, target, 1.0f, 0, 0.5f, 100.0f);
        if (i == 0)
            Expect("glue: target accepted", accepted);
        long long now = 0;
        QueryPerformanceCounter(&now);
        p = MakePose();
        utpp::headlook::Apply(p.bytes.data(), kN, &rigTag, 1.0f, 1.0f, 0.0f, 0, now);
        Sleep(8);
    }
    const float turned = HeadYawDeg(p) - rest;
    Expect("glue: after 0.5 s of calls with a target 45 deg aside the head has turned at least 25 deg", std::fabs(turned) > 25.0f,
           Fmt("turned %g deg", turned));
    for (int i = 0; i < 150; ++i)
    {
        utpp::headlook::SetTarget(false, false, target, 0.0f, 0, 0.5f, 100.0f);
        long long now = 0;
        QueryPerformanceCounter(&now);
        p = MakePose();
        utpp::headlook::Apply(p.bytes.data(), kN, &rigTag, 1.0f, 1.0f, 0.0f, 0, now);
        Sleep(8);
    }
    Expect("glue: target released: the pose is bit-identical again after 1.2 s", Same(p, MakePose()),
           Fmt("head yaw %g deg", HeadYawDeg(p) - rest));
    utpp::headlook::OnDisarmed();
    Expect("glue: disarmed -> rig not ok", !utpp::headlook::RigOk());
}

int main()
{
    TestRig();
    TestGaze();
    TestSpring();
    TestTurn();
    TestDistribution();
    TestChestRule();
    TestLimits();
    TestEyes();
    TestIdentity();
    TestEasing();
    TestModeSwitch();
    TestHeadLeadComposition();
    TestGate();
    TestReapply();
    TestFailClosed();
    TestWalkingSway();
    TestGlue();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

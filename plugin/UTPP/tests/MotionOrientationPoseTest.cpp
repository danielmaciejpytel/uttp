#include "../src/mm/MotionOrientationPose.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string_view>

using namespace utpp::mm;
using namespace utpp::mm::motionPose;

static int pass = 0, fail = 0;
static void C(const char *name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 2.0e-4f)
{
    return std::isfinite(a) && std::fabs(a - b) <= e;
}
static bool SameVec(math::Vec3 a, math::Vec3 b)
{
    return Near(a.x, b.x, 1.0e-7f) && Near(a.y, b.y, 1.0e-7f) && Near(a.z, b.z, 1.0e-7f);
}
static float Yaw(const math::Quat &q)
{
    const auto v = math::QuatRotate(q, math::Vec3{0.0f, 1.0f, 0.0f});
    return std::atan2(-v.x, v.y); // CP +Y forward.
}
static bool Unit(const math::Quat &q)
{
    return Near(math::QuatDot(q, q), 1.0f, 1.0e-5f);
}

// Bone-name indices mirror player_woman_skeleton: Root=0, Hips=2, Spine=4,
// Spine1=7, Spine2=10, Spine3=13. This is a test database adapter for
// Warp::Init.
struct RigStub
{
    static constexpr std::array<std::string_view, 14> names{
        "Root",   "Trajectory", "Hips",     "reference_joint", "Spine",    "LeftUpLeg", "RightUpLeg",
        "Spine1", "LeftLeg",    "RightLeg", "Spine2",          "LeftFoot", "RightFoot", "Spine3"};
    uint32_t NumBones() const
    {
        return static_cast<uint32_t>(names.size());
    }
    std::string_view BoneName(uint32_t i) const
    {
        return names[i];
    }
};
static constexpr std::array<uint32_t, 5> ix{2, 4, 7, 10, 13};

static math::Quat ComponentSpine3(const std::array<BoneTransform, 24> &pose)
{
    math::Quat q = pose[ix[0]].q;
    for (std::size_t i = 1; i < ix.size(); ++i)
        q = math::QuatNormalize(math::QuatMul(q, pose[ix[i]].q));
    return q;
}
static void SetIdentityPose(std::array<BoneTransform, 24> &pose)
{
    pose.fill(BoneTransform{});
}

int main()
{
    RigStub db;
    Warp warp;
    InitError initError{};
    C("Init resolves CP chain by names", warp.Init(db, &initError) && initError == InitError::None && warp.Ready());

    std::array<BoneTransform, 24> pose{};
    SetIdentityPose(pose);
    for (std::size_t i = 0; i < ix.size(); ++i)
        pose[ix[i]].p = {0.01f * float(i + 1), -0.02f * float(i), 0.03f * float(i)};
    const auto zeroBefore = pose;
    C("zero angle preserves the fresh pose",
      warp.Apply(pose, 0.0f) && pose[2].q.w == 1.0f && SameVec(pose[13].p, zeroBefore[13].p));

    pose = zeroBefore;
    constexpr float theta = math::kPi * 0.5f;
    C("90-degree post-alpha half-angle applies", warp.Apply(pose, theta));
    C("root half-angle is rebased into direct-child Hips", Near(Yaw(pose[2].q), theta));
    C("mapped UE counter weights sum to one",
      Near(kMappedCounterWeights[0] + kMappedCounterWeights[1] + kMappedCounterWeights[2] + kMappedCounterWeights[3],
           1.0f));
    C("identity hierarchy oracle cancels total upper heading", Near(Yaw(ComponentSpine3(pose)), 0.0f));
    C("lazy component-space child evaluation plus ResetRoot yields mapped local "
      "counter angles",
      Near(Yaw(pose[4].q), -theta * 0.2f) && Near(Yaw(pose[7].q), -theta * 0.2f) &&
          Near(Yaw(pose[10].q), -theta * 0.2f) && Near(Yaw(pose[13].q), -theta * 0.4f));
    const auto expectedHipsPoint =
        math::QuatRotate(math::Quat{0.0f, 0.0f, std::sin(theta * 0.5f), std::cos(theta * 0.5f)}, zeroBefore[2].p);
    C("Hips root-child point rotates around CP Root origin", SameVec(pose[2].p, expectedHipsPoint));
    C("Hips root-child point preserves its radius",
      Near(math::Dot(pose[2].p, pose[2].p), math::Dot(zeroBefore[2].p, zeroBefore[2].p)));
    C("upper segment local translations remain exact",
      SameVec(pose[4].p, zeroBefore[4].p) && SameVec(pose[7].p, zeroBefore[7].p) &&
          SameVec(pose[10].p, zeroBefore[10].p) && SameVec(pose[13].p, zeroBefore[13].p));
    C("unrelated pose transforms remain exact",
      pose[20].q.w == zeroBefore[20].q.w && SameVec(pose[20].p, zeroBefore[20].p));
    C("output chain rotations remain unit", [&] {
        for (uint32_t i : ix)
            if (!Unit(pose[i].q))
                return false;
        return true;
    }());
    C("foot contact remains explicitly deferred", warp.FootContactDeferred());
    VirtualFootTargetInput targetIn{{1.0f, 0.0f, 0.0f}, math::Quat{}, {0.0f, 1.0f, 0.0f}, math::Quat{}, 0.75f};
    VirtualFootTargetOutput targetOut{};
    C("virtual ankle target composes root-half pivot and full-angle local offset",
      warp.MapVirtualFootTarget(targetIn, theta, targetOut) &&
          SameVec(targetOut.ankleRootPosition, {0.0f, 0.0f, 0.0f}));
    C("virtual foot orientation applies final inverse-half counter-rotation",
      Near(Yaw(targetOut.ankleRootRotation), theta));
    C("virtual contact weight is preserved for downstream knee-plane IK", Near(targetOut.contactWeight, 0.75f));

    // Actual 392C DB clip 449, local frame 54 / global frame 46345. Values are
    // the baked CP local translations and xyzw rotations read from its POSE
    // section (full provenance in JSON).
    SetIdentityPose(pose);
    pose[2] = {{0.06756660f, -0.27635229f, 0.89980841f}, {0.73938495f, 0.12794140f, 0.66081721f, 0.01617140f}};
    pose[4] = {{0.08601080f, 0.0f, 0.0f}, {-0.00214040f, 0.04128800f, -0.15667920f, 0.98678386f}};
    pose[7] = {{0.05951680f, 0.0f, 0.0f}, {-0.00527270f, 0.01429160f, -0.01806930f, 0.99972069f}};
    pose[10] = {{0.07805660f, 0.0f, 0.0f}, {-0.00480470f, 0.01533320f, -0.02134710f, 0.99964297f}};
    pose[13] = {{0.10290760f, 0.0f, 0.0f}, {-0.03409170f, 0.03777270f, -0.07767830f, 0.99567920f}};
    const auto dbPose = pose;
    const auto beforeUpper = ComponentSpine3(dbPose);
    C("actual DB clip 449 frame 54 static pose is accepted", warp.Apply(pose, 0.31f));
    C("actual DB upper component orientation survives full counter distribution",
      math::QuatAngleBetween(beforeUpper, ComponentSpine3(pose)) < 4.0e-4f);
    const auto rootHalf449 = math::Quat{0.0f, 0.0f, std::sin(0.155f), std::cos(0.155f)};
    C("actual DB Hips position matches the root-half point oracle",
      SameVec(pose[2].p, math::QuatRotate(rootHalf449, dbPose[2].p)));
    C("actual DB Hips radial length is preserved",
      Near(math::Dot(pose[2].p, pose[2].p), math::Dot(dbPose[2].p, dbPose[2].p)));
    C("actual DB upper local segment translations remain unchanged",
      SameVec(pose[4].p, dbPose[4].p) && SameVec(pose[7].p, dbPose[7].p) && SameVec(pose[10].p, dbPose[10].p) &&
          SameVec(pose[13].p, dbPose[13].p));
    C("actual DB output chain rotations remain finite and unit", [&] {
        for (uint32_t i : ix)
            if (!math::IsFinite(pose[i].q) || !Unit(pose[i].q))
                return false;
        return true;
    }());

    // Actual 392C DB clip 113 (M_Neutral_Crouch_Loop_RR), local frame 59 / global
    // frame 12197.
    SetIdentityPose(pose);
    pose[2] = {{0.08035710f, -0.29173139f, 0.73158592f}, {-0.68097407f, -0.23832248f, -0.69175726f, 0.03080000f}};
    pose[4] = {{0.08601080f, 0.0f, 0.0f}, {0.00812030f, -0.07697169f, -0.21094589f, 0.97442865f}};
    pose[7] = {{0.05951680f, 0.0f, 0.0f}, {-0.02722850f, -0.02089180f, -0.05172350f, 0.99807155f}};
    pose[10] = {{0.07805660f, 0.0f, 0.0f}, {-0.01996080f, -0.01432880f, -0.05417080f, 0.99822932f}};
    pose[13] = {{0.10290760f, 0.0f, 0.0f}, {-0.10108079f, -0.01832520f, -0.10977028f, 0.98863411f}};
    const auto crouchPose = pose;
    const auto beforeCrouchUpper = ComponentSpine3(crouchPose);
    C("actual DB clip 113 crouch pose is accepted", warp.Apply(pose, -0.24f));
    C("actual DB crouch upper component orientation survives full counter "
      "distribution",
      math::QuatAngleBetween(beforeCrouchUpper, ComponentSpine3(pose)) < 4.0e-4f);
    const auto rootHalf113 = math::Quat{0.0f, 0.0f, std::sin(-0.12f), std::cos(-0.12f)};
    C("actual DB crouch Hips position matches the root-half point oracle",
      SameVec(pose[2].p, math::QuatRotate(rootHalf113, crouchPose[2].p)));
    C("actual DB crouch Hips radial length is preserved",
      Near(math::Dot(pose[2].p, pose[2].p), math::Dot(crouchPose[2].p, crouchPose[2].p)));
    C("actual DB crouch upper local segment translations remain unchanged",
      SameVec(pose[4].p, crouchPose[4].p) && SameVec(pose[7].p, crouchPose[7].p) &&
          SameVec(pose[10].p, crouchPose[10].p) && SameVec(pose[13].p, crouchPose[13].p));
    C("actual DB crouch output chain rotations remain finite and unit", [&] {
        for (uint32_t i : ix)
            if (!math::IsFinite(pose[i].q) || !Unit(pose[i].q))
                return false;
        return true;
    }());

    const auto beforeReject = pose;
    C("non-finite angle rejected transactionally",
      !warp.Apply(pose, std::numeric_limits<float>::infinity()) && pose[2].q.w == beforeReject[2].q.w);
    pose[7].q = {0.0f, 0.0f, 0.0f, 0.0f};
    const auto invalidBefore = pose;
    C("non-unit quaternion rejected transactionally", !warp.Apply(pose, 0.2f) && pose[7].q.w == invalidBefore[7].q.w);
    const std::array<std::string_view, 2> incomplete{"Root", "Hips"};
    struct IncompleteDb
    {
        const std::array<std::string_view, 2> &n;
        uint32_t NumBones() const
        {
            return 2;
        }
        std::string_view BoneName(uint32_t i) const
        {
            return n[i];
        }
    } incompleteDb{incomplete};
    Warp missing;
    C("missing CP chain bone prevents initialization",
      !missing.Init(incompleteDb, &initError) && initError == InitError::MissingBone);

    std::printf("%d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}

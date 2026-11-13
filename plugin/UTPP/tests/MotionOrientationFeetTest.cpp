#include "../src/mm/MotionOrientationFeet.hpp"
#include "../src/mm/MotionDatabase.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

using namespace utpp::mm;
using namespace utpp::mm::motionFeet;

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
static bool SameVec(math::Vec3 a, math::Vec3 b, float e = 1.0e-7f)
{
    return Near(a.x, b.x, e) && Near(a.y, b.y, e) && Near(a.z, b.z, e);
}
static float Len(math::Vec3 v)
{
    return std::sqrt(math::Dot(v, v));
}
static bool Unit(math::Quat q)
{
    return Near(math::QuatDot(q, q), 1.0f, 1.0e-5f);
}
static float QDistance(math::Quat a, math::Quat b)
{
    if (math::QuatDot(a, b) < 0)
        b = math::QuatNeg(b);
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z) + (a.w - b.w) * (a.w - b.w));
}

struct TestRig
{
    static constexpr std::array<std::string_view, 26> names{
        "Root",        "Trajectory",  "Hips",         "reference_joint", "Spine",    "LeftUpLeg",
        "RightUpLeg",  "Spine1",      "LeftLeg",      "RightLeg",        "Spine2",   "LeftFoot",
        "RightFoot",   "Spine3",      "LeftShoulder", "RightShoulder",   "Neck",     "Head",
        "LeftArm",     "RightArm",    "LeftForeArm",  "RightForeArm",    "LeftHand", "RightHand",
        "LeftToeBase", "RightToeBase"};
    static constexpr std::array<int32_t, 26> parents{-1, -1, 0,  2,  2,  2,  2,  4,  5,  6,  7,  8,  9,
                                                     10, 13, 13, 13, 16, 14, 15, 18, 19, 20, 21, 11, 12};
    uint32_t NumBones() const
    {
        return static_cast<uint32_t>(names.size());
    }
    std::string_view BoneName(uint32_t i) const
    {
        return names[i];
    }
    int32_t BoneParent(uint32_t i) const
    {
        return parents[i];
    }
};

template <class RigLike>
static void BuildModel(const RigLike &rig, const std::vector<BoneTransform> &pose, std::vector<BoneTransform> &model)
{
    for (size_t i = 0; i < pose.size(); ++i)
    {
        const auto p = rig.BoneParent(static_cast<uint32_t>(i));
        model[i] = p < 0 ? pose[i]
                         : BoneTransform{model[size_t(p)].p + math::QuatRotate(model[size_t(p)].q, pose[i].p),
                                         math::QuatNormalize(math::QuatMul(model[size_t(p)].q, pose[i].q))};
    }
}

static std::vector<BoneTransform> SyntheticPose()
{
    std::vector<BoneTransform> p(TestRig::names.size());
    p[2].p = {0.0f, 0.0f, 0.90f};
    p[4].p = {0.0f, 0.0f, 0.13f};
    p[7].p = {0.0f, 0.0f, 0.12f};
    p[10].p = {0.0f, 0.0f, 0.12f};
    p[13].p = {0.0f, 0.0f, 0.12f};
    p[5].p = {0.10f, 0.0f, -0.05f};
    p[6].p = {-0.10f, 0.0f, -0.05f};
    p[8].p = {0.12f, 0.02f, -0.30f};
    p[9].p = {-0.12f, 0.02f, -0.30f};
    p[11].p = {-0.12f, 0.01f, -0.30f};
    p[12].p = {0.12f, 0.01f, -0.30f};
    p[24].p = {0.0f, 0.12f, -0.01f};
    p[25].p = {0.0f, 0.12f, -0.01f};
    return p;
}

static bool FindFrame(const motion::MotionDatabase &db, std::string_view name, uint32_t local, uint32_t &frame)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name && local < db.Clip(i).numFrames)
        {
            frame = db.Clip(i).firstFrame + local;
            return true;
        }
    return false;
}

static std::vector<BoneTransform> DbPose(const motion::MotionDatabase &db, uint32_t frame)
{
    const auto src = db.Pose(frame);
    std::vector<BoneTransform> pose(db.NumBones());
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        pose[b] = {{src[b].px, src[b].py, src[b].pz}, {src[b].qx, src[b].qy, src[b].qz, src[b].qw}};
    }
    return pose;
}

template <class RigLike>
static bool RunCase(Warp &feet, motionPose::Warp &poseWarp, const RigLike &rig, std::vector<BoneTransform> raw,
                    float theta, const char *label)
{
    Targets targets{};
    if (!feet.Sample(raw, theta, targets) || !targets.valid || targets.warpWeight != 1.0f)
        return false;
    if (!poseWarp.Apply(raw, theta))
        return false;
    const auto before = raw;
    std::vector<BoneTransform> beforeModel(raw.size()), afterModel(raw.size());
    BuildModel(rig, before, beforeModel);
    const auto upperBefore = beforeModel[13].q;
    const float leftUpperLength = Len(beforeModel[5].p - beforeModel[2].p);
    const float leftLowerLength = Len(beforeModel[8].p - beforeModel[5].p);
    const float rightUpperLength = Len(beforeModel[6].p - beforeModel[2].p);
    const float rightLowerLength = Len(beforeModel[9].p - beforeModel[6].p);
    const math::Vec3 kneePlaneBefore =
        math::Cross(beforeModel[5].p - beforeModel[2].p, beforeModel[8].p - beforeModel[5].p);
    const math::Vec3 kneePlaneRightBefore =
        math::Cross(beforeModel[6].p - beforeModel[2].p, beforeModel[9].p - beforeModel[6].p);
    const ApplyResult out = feet.Apply(raw, targets);
    if (!out.applied || out.feet[0].status != legik::TwoBoneStatus::kOk ||
        out.feet[1].status != legik::TwoBoneStatus::kOk)
        return false;
    BuildModel(rig, raw, afterModel);
    const bool lengths = Near(Len(afterModel[8].p - afterModel[5].p), leftLowerLength, 1.0e-4f) &&
                         Near(Len(afterModel[5].p - afterModel[2].p), leftUpperLength, 1.0e-4f) &&
                         Near(Len(afterModel[9].p - afterModel[6].p), rightLowerLength, 1.0e-4f) &&
                         Near(Len(afterModel[6].p - afterModel[2].p), rightUpperLength, 1.0e-4f);
    const bool untouched = raw[4].q.x == before[4].q.x && raw[4].q.y == before[4].q.y && raw[4].q.z == before[4].q.z &&
                           raw[4].q.w == before[4].q.w && SameVec(raw[5].p, before[5].p) &&
                           SameVec(raw[8].p, before[8].p) && SameVec(raw[11].p, before[11].p);
    const bool targetsReached = out.feet[0].targetError < 0.04f && out.feet[1].targetError < 0.04f;
    const math::Vec3 kneePlaneAfter = math::Cross(afterModel[5].p - afterModel[2].p, afterModel[8].p - afterModel[5].p);
    const math::Vec3 kneePlaneRightAfter =
        math::Cross(afterModel[6].p - afterModel[2].p, afterModel[9].p - afterModel[6].p);
    const float planeDot =
        math::Dot(kneePlaneBefore, kneePlaneAfter) / std::fmax(1.0e-8f, Len(kneePlaneBefore) * Len(kneePlaneAfter));
    const float rightPlaneDot = math::Dot(kneePlaneRightBefore, kneePlaneRightAfter) /
                                std::fmax(1.0e-8f, Len(kneePlaneRightBefore) * Len(kneePlaneRightAfter));
    const bool kneeContinuity = planeDot > 0.0f && rightPlaneDot > 0.0f;
    const bool upperUntouched = math::QuatAngleBetween(upperBefore, afterModel[13].q) < 2.0e-4f;
    const bool orientations = math::QuatAngleBetween(targets.feet[0].q, afterModel[11].q) < 2.0e-4f &&
                              math::QuatAngleBetween(targets.feet[1].q, afterModel[12].q) < 2.0e-4f;
    std::printf("INFO %s upperAngleDelta=%g kneePlaneDots=%g,%g "
                "targetErrors=%g,%g clamped=%d,%d\n",
                label, math::QuatAngleBetween(upperBefore, afterModel[13].q), planeDot, rightPlaneDot,
                out.feet[0].targetError, out.feet[1].targetError, out.feet[0].clamped, out.feet[1].clamped);
    return lengths && untouched && targetsReached && kneeContinuity && upperUntouched && orientations &&
           !out.feet[0].clamped && !out.feet[1].clamped && Unit(afterModel[11].q) && Unit(afterModel[12].q);
}

int main(int argc, char **argv)
{
    TestRig rig;
    Warp feet;
    InitError err{};
    C("Init caches both named CP leg chains and parent map",
      feet.Init(rig, &err) && feet.Ready() && err == InitError::None);
    motion::MotionDatabase db;
    if (argc > 1)
    {
        std::string error;
        const bool loaded = db.LoadFromFile(argv[1], 0xF62810A88A8BFDA9ull, error);
        C("actual 392C DB loads for frame-level cases", loaded);
        if (loaded)
        {
            Warp actualFeet;
            motionPose::Warp actualPoseWarp;
            C("actual DB initializes CP feet and pose mappers", actualFeet.Init(db) && actualPoseWarp.Init(db));
            uint32_t f449 = 0, f113 = 0;
            const bool haveFrames = FindFrame(db, "M_Neutral_Run_Turn_L_180_Rfoot", 54, f449) &&
                                    FindFrame(db, "M_Neutral_Crouch_Loop_RR", 59, f113);
            C("actual DB clip 449 and 113 midpoint frames are located", haveFrames);
            if (haveFrames)
            {
                C("actual DB clip 449 full pose passes source-target and CP IK oracle",
                  RunCase(actualFeet, actualPoseWarp, db, DbPose(db, f449), 0.31f, "449"));
                C("actual DB clip 113 full pose passes source-target and CP IK oracle",
                  RunCase(actualFeet, actualPoseWarp, db, DbPose(db, f113), -0.24f, "113"));
            }
        }
    }
    else
        C("392C DB path required for actual frame cases", false);

    auto pose = SyntheticPose();
    Targets targets{};
    C("raw synthetic sample builds both root-local virtual ankle targets",
      feet.Sample(pose, 0.2f, targets) && targets.valid && targets.warpWeight == 1.0f);
    Targets identity{};
    std::vector<BoneTransform> rawModel(pose.size());
    BuildModel(rig, pose, rawModel);
    C("zero angle sample preserves raw ankle targets with exact no-warp weight",
      feet.Sample(pose, 0.0f, identity) && identity.valid && identity.warpWeight == 0.0f &&
          SameVec(identity.feet[0].p, rawModel[11].p) && SameVec(identity.feet[1].p, rawModel[12].p) &&
          math::QuatAngleBetween(identity.feet[0].q, rawModel[11].q) < 1.0e-5f &&
          math::QuatAngleBetween(identity.feet[1].q, rawModel[12].q) < 1.0e-5f);
    const auto exactBefore = pose;
    auto noop = feet.Apply(pose, identity);
    C("zero-weight application is exact no-op without solving",
      noop.applied && std::memcmp(pose.data(), exactBefore.data(), pose.size() * sizeof(BoneTransform)) == 0);
    motionPose::Warp bodyWarp;
    C("synthetic pose mapper initializes", bodyWarp.Init(rig));
    C("synthetic reachable target keeps knee bend plane and limb lengths",
      RunCase(feet, bodyWarp, rig, SyntheticPose(), 0.2f, "synthetic"));

    // Relative correction, not blended absolute animated positions, is the
    // neutral element. Opposite quaternion signs must represent that same zero.
    auto opposite = SyntheticPose();
    Targets oppositeTargets{};
    feet.Sample(opposite, .2f, oppositeTargets);
    bodyWarp.Apply(opposite, .2f);
    const auto expectedAbsolute = oppositeTargets;
    for (auto &bone : opposite)
        bone.q = math::QuatNeg(bone.q);
    C("relative conversion accepts equivalent negative quaternion signs",
      feet.MakeRelative(opposite, oppositeTargets) && oppositeTargets.relative);
    const auto relativeResult = feet.Apply(opposite, oppositeTargets);
    std::vector<BoneTransform> oppositeModel(opposite.size());
    BuildModel(rig, opposite, oppositeModel);
    C("single-player relative correction reaches original mapped targets",
      relativeResult.applied && Len(oppositeModel[11].p - expectedAbsolute.feet[0].p) < .0001f &&
          Len(oppositeModel[12].p - expectedAbsolute.feet[1].p) < .0001f &&
          QDistance(oppositeModel[11].q, expectedAbsolute.feet[0].q) < .0001f);
    auto almostStraight = SyntheticPose();
    almostStraight[8].p = {.004f, 0, -.4f};
    almostStraight[9].p = {-.004f, 0, -.4f};
    almostStraight[11].p = {-.004f, 0, -.4f};
    almostStraight[12].p = {.004f, 0, -.4f};
    for (float angle : {0.f, 1e-6f, 1e-5f, 1e-4f})
    {
        auto tinyPose = almostStraight;
        Targets tiny{};
        const bool sampled = feet.Sample(tinyPose, angle, tiny) && bodyWarp.Apply(tinyPose, angle) &&
                             feet.MakeRelative(tinyPose, tiny);
        const auto beforeTiny = tinyPose;
        const auto applied = feet.Apply(tinyPose, tiny);
        float qError = 0;
        for (int bone : {5, 6, 8, 9, 11, 12})
            qError = std::fmax(qError, QDistance(tinyPose[size_t(bone)].q, beforeTiny[size_t(bone)].q));
        std::printf("INFO almost-straight angle=%g max qdistance=%g\n", angle, qError);
        C("near-zero relative correction preserves nearly straight knee continuously",
          sampled && applied.applied && qError < 20 * angle + 1e-6f);
    }

    std::vector<BoneTransform> model(pose.size());
    BuildModel(rig, pose, model);
    targets.feet[0].p = model[5].p;
    targets.valid = true;
    targets.warpWeight = 1.0f;
    const auto rejectBefore = pose;
    const ApplyResult reject = feet.Apply(pose, targets);
    C("bad ankle target fails closed without partial pose mutation",
      !reject.applied && std::memcmp(pose.data(), rejectBefore.data(), pose.size() * sizeof(BoneTransform)) == 0);
    std::printf("%d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}

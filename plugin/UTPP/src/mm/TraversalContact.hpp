#pragma once
// Obstacle contact of a traversal montage: the supporting hand rests on the obstacle top while the clip has it planted, and the
// pelvis, torso and legs stay outside the obstacle volume. Header-only, no game types; unit-tested offline (MotionTraversalTest).
//
// Obstacle model (from the game's climb / vault geometry): a solid box in front of V. Its top surface is at the game's top point,
// it starts at the front edge through that point (normal = facing), reaches `depth` along the facing, ends downwards at `bottomZ`
// and is unbounded sideways (the game reports no width; the part V crosses is the part under V).
//
// Poses are local records with the database bone order; bones whose parent index is below `firstBone` are root children and hold
// the character-frame (model) transform. World = origin + RotZ(model, yaw), the same relation the driver uses for a montage frame
// (origin = visual root, yaw = plan root yaw).
//
// Body clearance: planned once per traversal from the warped plan (Build). Per clip frame the torso points give a root
// displacement (up over the top or back from the front face, whichever is shorter) and each leg gives an ankle displacement; both
// curves are widened and averaged over +/- kSmoothFrames frames, so the correction starts before the contact and never steps.
// A held hand comes first: the root displacement is reduced where it would take the hand's point out of the arm's reach.
// Hands: the planted span of each hand is read from the clip itself (palm slow and at ledge height in clip space); the span of
// the front-ledge window's own hand always includes that window's end. The held point is read from the warped plan's palm at
// that end, moved onto the top surface at the front edge. Where the game reports hand points it places the hands on itself
// (GameObstacle::hasHands) and the point lies on this edge, the wrist is held on the game's point instead (the point the plan's
// hand window already aims at). During the span the hand starts from where the plan has it (the root displacement does not carry
// it along) and moves onto the held point by a weight that eases in and out and is reduced when the point is out of reach, far
// from the clip's hand or passed by the body. Two-bone IK with the clip's own elbow plane; upper arm swing and wrist bend are
// limited against the clip.
#include "LegIk.hpp"
#include "MotionTraversal.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace utpp::mm::traversal::contact
{
inline constexpr int kMaxChain = 24;
inline constexpr int kMaxBones = 96;
inline constexpr int kLegPasses = 2;               // leg passes: the need, then what the first solve left
inline constexpr int kSmoothFrames = 4;            // clip frames each side (widen, then average)
inline constexpr double kRootLiftMaxM = 0.10;
inline constexpr double kRootBackMaxM = 0.08;
inline constexpr double kFootAlongMaxM = 0.15;     // horizontal move of a foot against the clip, root and ankle together
inline constexpr double kLegLiftMaxM = 0.35;
inline constexpr double kPalmHeightM = 0.025;      // palm joint line above the top surface (half the hand's thickness)
inline constexpr double kPalmMinPastEdgeM = 0.04;  // palm centre at least this far past the front edge
inline constexpr double kPalmMaxPastEdgeM = 0.08;  // and not further than this (nor than depth - kPalmBackMarginM): on the edge
inline constexpr double kPalmBackMarginM = 0.03;
// a game hand point is used when it lies this close to the top height and to the front edge, and this close to the planned wrist
inline constexpr double kGrabHeightM = 0.12, kGrabAlongM = 0.20, kGrabFromPlanM = 0.35;
inline constexpr double kPlantSpeedMs = 3.0;       // clip-space palm speed below which a hand counts as resting
inline constexpr double kPlantBelowM = 0.20, kPlantAboveM = 0.12; // palm height around the clip's ledge contact
inline constexpr double kPlantBackM = 0.25, kPlantAheadM = 0.35;  // palm along the facing around the clip's ledge contact
inline constexpr int kPlantMinFrames = 3;
inline constexpr int kPlantAfterFrontFrames = 14;
inline constexpr int kPlantJoinFrames = 4;         // the ledge window's end this close to the contact hand's span joins it
inline constexpr double kPlantMaxMoveM = 0.50;     // a planned palm further than this from the surface point is not supporting
inline constexpr double kHeldReach = 0.985;         // arm lengths: the body clearance never takes a held point farther than this
inline constexpr double kHandRampInFrames = 4.0, kHandRampOutFrames = 5.0;
inline constexpr double kReachFadeFrom = 1.00, kReachFadeTo = 1.15; // shoulder -> wrist target, in arm lengths
inline constexpr double kMoveFadeFromM = 0.35, kMoveFadeToM = 0.55; // wrist correction length
inline constexpr double kPastFadeFromM = 0.30, kPastFadeToM = 0.55; // shoulder past the held point along the facing: let go
inline constexpr double kArmSwingMaxDeg = 50.0;    // upper arm and forearm direction change against the clip pose
inline constexpr double kLegSwingMaxDeg = 38.0;
inline constexpr double kKneeMinInnerDeg = 30.0;   // the leg is never folded tighter than this (or than the clip already is)
inline constexpr double kElbowMinInnerDeg = 30.0;
inline constexpr double kWristMaxDeg = 25.0;      // the wrist turns at most this far from the clip's wrist; past it the hand follows the forearm
inline constexpr double kHandWeightRatePerS = 8.0;  // the applied hand weight changes at most this fast (full range in 125 ms)

enum Bone : int
{
    kHips, kSpine1, kSpine3, kHead,
    kLUpLeg, kLLeg, kLFoot, kLToe, kLHeel,
    kRUpLeg, kRLeg, kRFoot, kRToe, kRHeel,
    kLArm, kLForeArm, kLHand, kLKnuckle,
    kRArm, kRForeArm, kRHand, kRKnuckle,
    kBoneCount
};
inline constexpr const char* kBoneNames[kBoneCount] = {
    "Hips", "Spine1", "Spine3", "Head",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "LeftHeel",
    "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase", "RightHeel",
    "LeftArm", "LeftForeArm", "LeftHand", "LeftHandMiddle1",
    "RightArm", "RightForeArm", "RightHand", "RightHandMiddle1"};

struct Rig
{
    int n = 0, firstBone = 2;
    const int32_t* parents = nullptr; // owned by the caller (database parent table)
    int id[kBoneCount] = {};
    // the body-point bones and all their ancestors from firstBone, ascending (parents first)
    int need[kMaxBones] = {};
    int needCount = 0;
    bool ok = false;
};

// aName(i) -> std::string_view of bone i.
template <class NameFn>
inline Rig FindRig(int aCount, const int32_t* aParents, int aFirstBone, NameFn aName)
{
    Rig r;
    r.n = aCount;
    r.parents = aParents;
    r.firstBone = aFirstBone;
    r.ok = aCount > 0 && aParents != nullptr;
    for (int b = 0; b < kBoneCount && r.ok; ++b)
    {
        r.id[b] = -1;
        for (int i = aFirstBone; i < aCount; ++i)
            if (aName(i) == kBoneNames[b])
            {
                r.id[b] = i;
                break;
            }
        r.ok = r.id[b] >= 0 && aParents[r.id[b]] < r.id[b];
    }
    // the two-bone chains must be direct parent chains
    const int chains[4][3] = {{kLUpLeg, kLLeg, kLFoot}, {kRUpLeg, kRLeg, kRFoot}, {kLArm, kLForeArm, kLHand}, {kRArm, kRForeArm, kRHand}};
    for (const auto& c : chains)
        r.ok = r.ok && aParents[r.id[c[1]]] == r.id[c[0]] && aParents[r.id[c[2]]] == r.id[c[1]] && aParents[r.id[c[0]]] >= aFirstBone;
    r.ok = r.ok && aCount <= kMaxBones;
    if (r.ok)
    {
        bool used[kMaxBones] = {};
        for (int b = 0; b < kBoneCount; ++b)
            for (int i = r.id[b]; i >= aFirstBone && !used[i]; i = aParents[i])
                used[i] = true;
        for (int i = aFirstBone; i < aCount && r.ok; ++i)
            if (used[i])
            {
                r.ok = aParents[i] < i;
                r.need[r.needCount++] = i;
            }
    }
    return r;
}

// Model transform of bone aBone from the local records.
inline BoneTransform ModelOf(const Rig& aRig, const BoneTransform* aPose, int aBone)
{
    int chain[kMaxChain];
    int n = 0;
    for (int i = aBone; i >= aRig.firstBone && n < kMaxChain; i = aRig.parents[i])
        chain[n++] = i;
    BoneTransform t{math::Vec3{}, math::Quat{0, 0, 0, 1}};
    for (int k = n - 1; k >= 0; --k)
    {
        const BoneTransform& l = aPose[chain[k]];
        t.p = t.p + math::QuatRotate(t.q, l.p);
        t.q = math::QuatNormalize(math::QuatMul(t.q, l.q));
    }
    return t;
}

// Model transforms of the body-point bones of one pose, in one pass.
struct Models
{
    BoneTransform t[kMaxBones];
};
inline void ComputeModels(const Rig& aRig, const BoneTransform* aPose, Models& aOut)
{
    for (int k = 0; k < aRig.needCount; ++k)
    {
        const int i = aRig.need[k], parent = aRig.parents[i];
        if (parent < aRig.firstBone)
        {
            aOut.t[i] = aPose[i];
            continue;
        }
        const BoneTransform& g = aOut.t[parent];
        aOut.t[i].p = g.p + math::QuatRotate(g.q, aPose[i].p);
        aOut.t[i].q = math::QuatNormalize(math::QuatMul(g.q, aPose[i].q));
    }
}

struct Space
{
    V3 origin;      // world position of the model origin
    double yaw = 0; // world yaw of the model frame
};
inline V3 ToWorld(const Space& aS, math::Vec3 aModel) { return aS.origin + RotZ(V3{aModel.x, aModel.y, aModel.z}, aS.yaw); }
inline math::Vec3 ToModelDir(const Space& aS, const V3& aWorld)
{
    const V3 d = RotZ(aWorld, -aS.yaw);
    return {static_cast<float>(d.x), static_cast<float>(d.y), static_cast<float>(d.z)};
}
inline math::Vec3 ToModel(const Space& aS, const V3& aWorld) { return ToModelDir(aS, aWorld - aS.origin); }

struct Box
{
    V3 top;      // a point of the front top edge
    V3 facing;   // horizontal unit, into the obstacle
    double depth = 0.2;
    double bottomZ = 0;
    bool valid = false;
};
inline Box MakeBox(const v2::GameObstacle& aO, double aFeetZ)
{
    Box b;
    const V3 f{-aO.collisionNormal.x, -aO.collisionNormal.y, 0};
    const double l = std::hypot(f.x, f.y);
    if (!(l > 0.5) || !std::isfinite(aO.topPoint.x + aO.topPoint.y + aO.topPoint.z + aFeetZ))
        return b;
    b.facing = f * (1.0 / l);
    b.top = aO.topPoint;
    const V3 span = aO.obstacleEnd - aO.topPoint;
    double d = std::max(aO.topExtent, span.x * b.facing.x + span.y * b.facing.y);
    if (aO.action == v2::GameAction::Climb)
        d = std::max(aO.topExtent, 0.20); // the top V climbs onto (the climb info carries no reliable back edge)
    b.depth = std::isfinite(d) ? std::clamp(d, 0.05, 4.0) : 0.2;
    b.bottomZ = std::min(aFeetZ, aO.topPoint.z - 0.3);
    b.valid = true;
    return b;
}
inline double Along(const Box& aB, const V3& aP) { return (aP.x - aB.top.x) * aB.facing.x + (aP.y - aB.top.y) * aB.facing.y; }

// How deep a sphere (centre aP, radius aR) sits in the box (0 = outside).
inline double Penetration(const Box& aB, const V3& aP, double aR)
{
    const double s = Along(aB, aP), z = aP.z - aB.top.z, zb = aB.bottomZ - aB.top.z;
    const double ds = s < 0 ? -s : (s > aB.depth ? s - aB.depth : 0.0);
    const double dz = z > 0 ? z : (z < zb ? zb - z : 0.0);
    if (ds == 0.0 && dz == 0.0)
        return aR + std::min({s, aB.depth - s, -z, z - zb});
    return std::max(0.0, aR - std::hypot(ds, dz));
}

// The two ways out of the box for a sphere: up over the top (rounded edges) and back out of the front face (< 0 = not offered).
struct WayOut
{
    double up = 0, back = -1;
};
inline WayOut Clearance(const Box& aB, const V3& aP, double aR)
{
    WayOut w;
    const double s = Along(aB, aP), z = aP.z - aB.top.z;
    if (Penetration(aB, aP, aR) <= 0.0)
        return w;
    const double edge = s < 0 ? -s : (s > aB.depth ? s - aB.depth : 0.0);
    const double surface = edge >= aR ? 0.0 : std::sqrt(aR * aR - edge * edge);
    w.up = std::max(0.0, surface - z);
    if (s < 0.5 * aB.depth)
        w.back = std::max(0.0, s + aR);
    return w;
}

// Body points tested against the box: the position is a + (b - a) * u; lever = share of an ankle displacement that moves it.
struct BodyPoint
{
    const char* name;
    Bone a, b;
    float u, radius, lever;
    int group; // 0 torso (root displacement), 1 left leg, 2 right leg
};
inline constexpr BodyPoint kBodyPoints[] = {
    {"pelvis", kHips, kHips, 0, 0.11f, 0, 0},
    {"spine1", kSpine1, kSpine1, 0, 0.10f, 0, 0},
    {"chest", kSpine3, kSpine3, 0, 0.11f, 0, 0},
    {"head", kHead, kHead, 0, 0.09f, 0, 0},
    {"thighTopL", kLUpLeg, kLLeg, 0.2f, 0.075f, 0, 0},
    {"thighTopR", kRUpLeg, kRLeg, 0.2f, 0.075f, 0, 0},
    {"thighL", kLUpLeg, kLLeg, 0.6f, 0.060f, 0.45f, 1},
    {"kneeL", kLLeg, kLLeg, 0, 0.055f, 0.55f, 1},
    {"shinL", kLLeg, kLFoot, 0.5f, 0.045f, 0.75f, 1},
    {"ankleL", kLFoot, kLFoot, 0, 0.040f, 1.0f, 1},
    {"toeL", kLToe, kLToe, 0, 0.025f, 1.0f, 1},
    {"heelL", kLHeel, kLHeel, 0, 0.025f, 1.0f, 1},
    {"thighR", kRUpLeg, kRLeg, 0.6f, 0.060f, 0.45f, 2},
    {"kneeR", kRLeg, kRLeg, 0, 0.055f, 0.55f, 2},
    {"shinR", kRLeg, kRFoot, 0.5f, 0.045f, 0.75f, 2},
    {"ankleR", kRFoot, kRFoot, 0, 0.040f, 1.0f, 2},
    {"toeR", kRToe, kRToe, 0, 0.025f, 1.0f, 2},
    {"heelR", kRHeel, kRHeel, 0, 0.025f, 1.0f, 2},
};
inline constexpr int kBodyPointCount = static_cast<int>(sizeof kBodyPoints / sizeof kBodyPoints[0]);

inline V3 PointWorld(const Rig& aRig, const BoneTransform* aPose, const Space& aS, const BodyPoint& aP)
{
    const math::Vec3 a = ModelOf(aRig, aPose, aRig.id[aP.a]).p;
    const math::Vec3 b = aP.a == aP.b ? a : ModelOf(aRig, aPose, aRig.id[aP.b]).p;
    return ToWorld(aS, a + (b - a) * aP.u);
}

inline V3 PointWorld(const Rig& aRig, const Models& aModels, const Space& aS, const BodyPoint& aP)
{
    const math::Vec3 a = aModels.t[aRig.id[aP.a]].p, b = aModels.t[aRig.id[aP.b]].p;
    return ToWorld(aS, a + (b - a) * aP.u);
}

inline double Ease(double aX)
{
    const double x = std::clamp(aX, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}
inline double AngleDeg(math::Vec3 aA, math::Vec3 aB)
{
    const math::Vec3 a = legik::Norm(aA), b = legik::Norm(aB);
    return std::acos(std::clamp(static_cast<double>(math::Dot(a, b)), -1.0, 1.0)) * 180.0 / 3.14159265358979;
}

struct LimbResult
{
    bool applied = false;
    double weight = 0;     // share of the asked move that was used
    double swingDeg = 0;   // largest segment direction change against the input pose
    double errorM = 0;     // end joint to the asked point after the solve
};

// Two-bone IK of the chain upper -> lower -> end toward aTargetModel (end joint), weight aWeight, with the input pose's bend plane
// and the end joint's model rotation kept. Reach, inner angle and segment swing are limited; only the three local rotations change.
inline LimbResult SolveLimb(const Rig& aRig, BoneTransform* aPose, int aUpper, int aLower, int aEnd, math::Vec3 aTargetModel,
                            double aWeight, double aSwingMaxDeg, double aMinInnerDeg, float aMaxExtension, float aSoftExtension)
{
    LimbResult out;
    if (!(aWeight > 1e-3))
        return out;
    const int parent = aRig.parents[aUpper];
    const BoneTransform gp = ModelOf(aRig, aPose, parent);
    const BoneTransform ga = ModelOf(aRig, aPose, aUpper);
    const BoneTransform gb = ModelOf(aRig, aPose, aLower);
    const BoneTransform gc = ModelOf(aRig, aPose, aEnd);
    const double lab = legik::Len(gb.p - ga.p), lcb = legik::Len(gc.p - gb.p), lac = legik::Len(gc.p - ga.p);
    if (!(lab > 1e-3 && lcb > 1e-3))
        return out;
    const double inner = aMinInnerDeg * 3.14159265358979 / 180.0;
    const double minDist = std::min(lac, std::sqrt(lab * lab + lcb * lcb - 2.0 * lab * lcb * std::cos(inner)));
    double w = std::min(aWeight, 1.0);
    for (int pass = 0; pass < 3; ++pass)
    {
        math::Vec3 t = gc.p + (aTargetModel - gc.p) * static_cast<float>(w);
        const double d = legik::Len(t - ga.p);
        if (d < minDist && d > 1e-4)
            t = ga.p + (t - ga.p) * static_cast<float>(minDist / d);
        // the forward hint only matters for a straight limb: bend toward the input middle joint, else down
        const legik::TwoBoneSolution s = legik::SolveTwoBone(gp, ga, gb, gc, t, gc.p + math::Vec3{0, 0, -1}, aMaxExtension,
                                                             aSoftExtension, true, true);
        if (s.status != legik::TwoBoneStatus::kOk)
            return out;
        const math::Quat qa = math::QuatNormalize(math::QuatMul(gp.q, s.local[0]));
        const math::Quat qb = math::QuatNormalize(math::QuatMul(qa, s.local[1]));
        const math::Vec3 nb = ga.p + math::QuatRotate(qa, aPose[aLower].p);
        const math::Vec3 nc = nb + math::QuatRotate(qb, aPose[aEnd].p);
        const double swing = std::max(AngleDeg(gb.p - ga.p, nb - ga.p), AngleDeg(gc.p - gb.p, nc - nb));
        if (swing > aSwingMaxDeg + 0.5 && pass < 2)
        {
            w *= aSwingMaxDeg / swing;
            continue;
        }
        if (swing > aSwingMaxDeg + 8.0)
            return out;
        aPose[aUpper].q = s.local[0];
        aPose[aLower].q = s.local[1];
        aPose[aEnd].q = s.local[2];
        out.applied = true;
        out.weight = w;
        out.swingDeg = swing;
        out.errorM = legik::Len(nc - aTargetModel);
        return out;
    }
    return out;
}

// Arm solve that puts the palm on a point. aTargetPalm is in model space. The hand first keeps the clip's model orientation; its
// wrist rotation is then limited to kWristMaxDeg from the clip's own, and the arm is solved again for the palm offset that gives.
inline LimbResult SolveHand(const Rig& aRig, BoneTransform* aPose, int aSide, math::Vec3 aTarget, double aWeight, bool aAtWrist = false)
{
    const int up = aRig.id[aSide ? kRArm : kLArm], low = aRig.id[aSide ? kRForeArm : kLForeArm], hand = aRig.id[aSide ? kRHand : kLHand];
    const int knuckle = aRig.id[aSide ? kRKnuckle : kLKnuckle];
    const math::Quat clip[3] = {aPose[up].q, aPose[low].q, aPose[hand].q};
    auto restore = [&] {
        aPose[up].q = clip[0];
        aPose[low].q = clip[1];
        aPose[hand].q = clip[2];
    };
    auto wristOf = [&] { return ModelOf(aRig, aPose, hand).p; };
    auto palmOf = [&] { return (wristOf() + ModelOf(aRig, aPose, knuckle).p) * 0.5f; };
    math::Vec3 offset = aAtWrist ? math::Vec3{} : wristOf() - palmOf(); // wrist from the held point
    LimbResult r;
    for (int pass = 0; pass < 3; ++pass)
    {
        restore();
        r = SolveLimb(aRig, aPose, up, low, hand, aTarget + offset, aWeight, kArmSwingMaxDeg, kElbowMinInnerDeg, 0.995f, 0.96f);
        if (!r.applied)
            return r;
        math::Quat d = math::QuatNormalize(math::QuatMul(math::QuatInv(clip[2]), aPose[hand].q));
        if (d.w < 0)
            d = math::QuatNeg(d);
        const double angle = 2.0 * std::acos(std::min(1.0, static_cast<double>(d.w))) * 180.0 / 3.14159265358979;
        if (angle > kWristMaxDeg)
        {
            const math::Vec3 axis = legik::Norm(math::Vec3{d.x, d.y, d.z});
            const math::Quat limited = legik::AxisAngle(axis, static_cast<float>(kWristMaxDeg * 3.14159265358979 / 180.0));
            aPose[hand].q = math::QuatNormalize(math::QuatMul(clip[2], limited));
        }
        if (!aAtWrist)
            offset = wristOf() - palmOf();
    }
    r.errorM = legik::Len((aAtWrist ? wristOf() : palmOf()) - aTarget);
    return r;
}

struct HandPlant
{
    bool valid = false;
    int first = 0, last = 0, key = 0; // clip frames
    V3 target;                        // world point held during the span: the palm, or the wrist (atWrist)
    bool atWrist = false;             // the target is the game's hand point
    double moved = 0;                 // distance of the planned palm at the key frame from the target
};

struct Track
{
    bool valid = false;
    Box box;
    double fps = 30;
    int first = 0, last = 0;         // clip frames covered
    std::vector<V3> root;            // world root displacement per frame
    std::vector<V3> leg[2];          // world ankle displacement per frame (after the root displacement)
    HandPlant hand[2];               // 0 left, 1 right
};

namespace detail
{
// Widen (maximum over +/- k) then average over +/- k: continuous, starts early, and never below the input.
inline void Smooth(std::vector<double>& aV, int aK)
{
    const int n = static_cast<int>(aV.size());
    std::vector<double> wide(aV.size(), 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = std::max(0, i - aK); j <= std::min(n - 1, i + aK); ++j)
            wide[i] = std::max(wide[i], aV[j]);
    for (int i = 0; i < n; ++i)
    {
        double sum = 0;
        for (int j = i - aK; j <= i + aK; ++j)
            sum += wide[std::clamp(j, 0, n - 1)];
        aV[i] = sum / (2 * aK + 1);
    }
}
// Zero on the first frame (the blend into the montage); EndTaper is also zero on the last one (an ankle move has no fade
// after the hand-back; the root displacement has: the hand-back carry).
inline double StartTaper(int aIndex) { return Ease(aIndex / 3.0); }
inline double EndTaper(int aIndex, int aCount) { return StartTaper(aIndex) * Ease((aCount - 1 - aIndex) / 4.0); }
inline V3 Sample(const std::vector<V3>& aV, double aIndex)
{
    if (aV.empty())
        return {};
    const double x = std::clamp(aIndex, 0.0, static_cast<double>(aV.size() - 1));
    const size_t i = static_cast<size_t>(x);
    const size_t j = std::min(i + 1, aV.size() - 1);
    return aV[i] + (aV[j] - aV[i]) * (x - static_cast<double>(i));
}
inline math::Vec3 Palm(const Rig& aRig, const BoneTransform* aPose, int aSide)
{
    const math::Vec3 w = ModelOf(aRig, aPose, aRig.id[aSide ? kRHand : kLHand]).p;
    const math::Vec3 k = ModelOf(aRig, aPose, aRig.id[aSide ? kRKnuckle : kLKnuckle]).p;
    return (w + k) * 0.5f;
}
} // namespace detail

// Planted span of each hand, from the clip alone (clip space = the clip's own root track). aSample(frame, pose) fills the local
// records of a clip frame. Only the spans are set (no world target).
template <class SampleFn>
inline void FindPlants(const TravMontage& aM, const Rig& aRig, BoneTransform* aPose, SampleFn aSample, HandPlant (&aOut)[2])
{
    aOut[0] = aOut[1] = HandPlant{};
    const TravWindow* front = nullptr;
    for (const TravWindow& w : aM.windows)
        if (!front && w.target == "FrontLedge")
            front = &w;
    if (!front || !aRig.ok || aM.frames < 3 || static_cast<int>(aM.root.size()) < aM.frames)
        return;
    const RootKey& re = aM.root[static_cast<size_t>(front->endFrame)];
    const V3 contactPoint = re.pos + RotZ(front->offset, re.yaw);
    const V3 fwd{-std::sin(re.yaw), std::cos(re.yaw), 0};
    const int from = std::max(1, front->startFrame);
    const int to = std::min(aM.frames - 2, front->endFrame + kPlantAfterFrontFrames);
    if (to - from + 1 < kPlantMinFrames)
        return;
    std::vector<V3> palm[2];
    for (int f = from - 1; f <= to + 1; ++f)
    {
        aSample(f, aPose);
        const RootKey& r = aM.root[static_cast<size_t>(f)];
        for (int h = 0; h < 2; ++h)
            palm[h].push_back(ToWorld(Space{r.pos, r.yaw}, detail::Palm(aRig, aPose, h)));
    }
    for (int h = 0; h < 2; ++h)
    {
        int bestFirst = 0, bestLast = -1, bestScore = 0, runFirst = -1;
        for (int f = from; f <= to + 1; ++f)
        {
            bool rest = false;
            if (f <= to)
            {
                const size_t i = static_cast<size_t>(f - from + 1);
                const double speed = Len(palm[h][i + 1] - palm[h][i - 1]) * aM.fps * 0.5;
                const V3 d = palm[h][i] - contactPoint;
                const double s = d.x * fwd.x + d.y * fwd.y;
                rest = speed < kPlantSpeedMs && d.z > -kPlantBelowM && d.z < kPlantAboveM && s > -kPlantBackM && s < kPlantAheadM;
            }
            if (rest && runFirst < 0)
                runFirst = f;
            if (!rest && runFirst >= 0)
            {
                // frames of the run inside the front-ledge window or after it; the longest such run wins
                const int len = f - runFirst;
                if (len >= kPlantMinFrames && runFirst <= front->endFrame + 4 && len > bestScore)
                {
                    bestScore = len;
                    bestFirst = runFirst;
                    bestLast = f - 1;
                }
                runFirst = -1;
            }
        }
        if (bestScore == 0)
            continue;
        HandPlant& p = aOut[h];
        p.valid = true;
        p.first = bestFirst;
        p.last = bestLast;
        double slow = 1e9;
        for (int f = bestFirst; f <= bestLast; ++f)
        {
            const size_t i = static_cast<size_t>(f - from + 1);
            const double speed = Len(palm[h][i + 1] - palm[h][i - 1]);
            if (speed < slow)
            {
                slow = speed;
                p.key = f;
            }
        }
    }
    // the front-ledge window's own hand: its span always holds the window's end (the frame the plan lines that hand up with
    // the ledge), so the body's clearance never carries that hand off its point
    {
        HandPlant& p = aOut[front->contactBone == "RightHand" ? 1 : 0];
        const int at = std::clamp(front->endFrame, 1, aM.frames - 2);
        if (!p.valid || at < p.first - kPlantJoinFrames || at > p.last + kPlantJoinFrames)
        {
            p.valid = true;
            p.first = std::max(1, at - kPlantJoinFrames / 2);
            p.last = std::min(aM.frames - 2, at + kPlantJoinFrames / 2);
        }
        p.first = std::min(p.first, at);
        p.last = std::max(p.last, at);
        p.key = at;
    }
}

// Root and leg displacement of one posed frame. Applied to the pose: the root move on the root children, then each ankle.
inline void MoveRoot(const Rig& aRig, BoneTransform* aPose, math::Vec3 aModel)
{
    for (int i = aRig.firstBone; i < aRig.n; ++i)
        if (aRig.parents[i] >= 0 && aRig.parents[i] < aRig.firstBone)
            aPose[i].p = aPose[i].p + aModel;
}
inline LimbResult MoveAnkle(const Rig& aRig, BoneTransform* aPose, int aSide, math::Vec3 aModel)
{
    if (legik::Len(aModel) < 2e-3f)
        return {};
    const int up = aRig.id[aSide ? kRUpLeg : kLUpLeg], low = aRig.id[aSide ? kRLeg : kLLeg], foot = aRig.id[aSide ? kRFoot : kLFoot];
    return SolveLimb(aRig, aPose, up, low, foot, ModelOf(aRig, aPose, foot).p + aModel, 1.0, kLegSwingMaxDeg, kKneeMinInnerDeg,
                     0.999f, 0.985f);
}

// Builds the track of an accepted plan. aPose = scratch of aRig.n records; aSample(frame, pose) fills a clip frame.
template <class SampleFn>
inline Track Build(const v2::Plan& aPlan, const v2::GameObstacle& aO, double aFeetZ, const Rig& aRig, BoneTransform* aPose,
                   SampleFn aSample)
{
    Track t;
    t.box = MakeBox(aO, aFeetZ);
    if (aPlan.result != v2::StartResult::Ok || !aPlan.montage || !t.box.valid || !aRig.ok)
        return t;
    const TravMontage& m = *aPlan.montage;
    const int lastPlan = static_cast<int>(aPlan.warped.size()) - 1;
    t.fps = m.fps;
    t.first = std::clamp(aPlan.startFrame, 0, std::max(lastPlan, 0));
    t.last = std::clamp(aPlan.endFrame, t.first, std::max(lastPlan, 0));
    const int count = t.last - t.first + 1;
    if (count < 5 || m.frames <= t.last)
        return t;
    auto space = [&](int f) {
        const RootKey& r = aPlan.warped[static_cast<size_t>(f)];
        return Space{r.pos, r.yaw};
    };
    const V3 back = t.box.facing * -1.0;
    // every covered clip frame is sampled once; the passes below work on copies
    const size_t stride = static_cast<size_t>(aRig.n);
    std::vector<BoneTransform> frames(static_cast<size_t>(count) * stride);
    for (int i = 0; i < count; ++i)
        aSample(t.first + i, &frames[static_cast<size_t>(i) * stride]);
    auto load = [&](int i) { std::copy_n(&frames[static_cast<size_t>(i) * stride], stride, aPose); };
    Models models;
    // 1: torso -> root displacement
    std::vector<double> up(static_cast<size_t>(count), 0.0), out(static_cast<size_t>(count), 0.0);
    for (int i = 0; i < count; ++i)
    {
        ComputeModels(aRig, &frames[static_cast<size_t>(i) * stride], models);
        const Space s = space(t.first + i);
        for (const BodyPoint& bp : kBodyPoints)
        {
            if (bp.group != 0)
                continue;
            const V3 w = PointWorld(aRig, models, s, bp);
            const WayOut c = Clearance(t.box, w, bp.radius);
            if (c.back >= 0 && c.back < c.up)
                out[i] = std::max(out[i], std::min(c.back, kRootBackMaxM));
            else
                up[i] = std::max(up[i], std::min(c.up, kRootLiftMaxM));
        }
    }
    detail::Smooth(up, kSmoothFrames);
    detail::Smooth(out, kSmoothFrames);
    t.root.resize(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i)
        t.root[i] = (V3{0, 0, up[i]} + back * out[i]) * detail::StartTaper(i); // what is left at the end goes out with the hand-back
    // 2: hands -> the span and the point each hand is held on (from the plan alone, before the root displacement)
    FindPlants(m, aRig, aPose,
               [&](int aFrame, BoneTransform* aOut) {
                   if (aFrame >= t.first && aFrame <= t.last)
                       std::copy_n(&frames[static_cast<size_t>(aFrame - t.first) * stride], stride, aOut);
                   else
                       aSample(aFrame, aOut);
               },
               t.hand);
    for (int side = 0; side < 2; ++side)
    {
        HandPlant& p = t.hand[side];
        if (!p.valid)
            continue;
        if (p.last < t.first + 1 || p.first > t.last - 2)
        {
            p.valid = false;
            continue;
        }
        p.first = std::max(p.first, t.first);
        p.last = std::min(p.last, t.last);
        // the held point is read where the plan's warp lines the hand up with the edge: the front-ledge window's end, inside the span
        for (const TravWindow& w : m.windows)
            if (w.target == "FrontLedge")
            {
                p.key = w.endFrame;
                break;
            }
        p.key = std::clamp(p.key, p.first, p.last);
        aSample(p.key, aPose);
        const Space s = space(p.key);
        const V3 palm = ToWorld(s, detail::Palm(aRig, aPose, side));
        const double along = Along(t.box, palm);
        const double maxAlong = std::max(kPalmMinPastEdgeM, std::min(kPalmMaxPastEdgeM, t.box.depth - kPalmBackMarginM));
        const double minAlong = std::min(kPalmMinPastEdgeM, 0.5 * t.box.depth);
        const double want = std::clamp(along, minAlong, std::max(minAlong, maxAlong));
        p.target = palm + t.box.facing * (want - along);
        p.target.z = t.box.top.z + kPalmHeightM;
        p.moved = Len(p.target - palm);
        if (aO.hasHands)
        {
            const V3 grab = side ? aO.rightHand : aO.leftHand;
            const V3 wrist = ToWorld(s, ModelOf(aRig, aPose, aRig.id[side ? kRHand : kLHand]).p);
            if (std::isfinite(grab.x + grab.y + grab.z) && std::fabs(grab.z - t.box.top.z) <= kGrabHeightM &&
                std::fabs(Along(t.box, grab)) <= kGrabAlongM && Len(grab - wrist) <= kGrabFromPlanM)
            {
                p.target = grab;
                p.atWrist = true;
                p.moved = Len(grab - wrist);
            }
        }
        if (p.moved > kPlantMaxMoveM)
            p.valid = false;
    }
    // 3: a held hand comes first: where the root displacement would take its point out of the arm's reach, it is reduced
    // (lowest share over +/- kSmoothFrames frames, then averaged: never above the frame's own limit, no steps)
    {
        std::vector<double> share(static_cast<size_t>(count), 1.0);
        for (int side = 0; side < 2; ++side)
        {
            const HandPlant& p = t.hand[side];
            if (!p.valid)
                continue;
            const int upper = aRig.id[side ? kRArm : kLArm], low = aRig.id[side ? kRForeArm : kLForeArm],
                      hand = aRig.id[side ? kRHand : kLHand];
            for (int f = p.first; f <= p.last; ++f)
            {
                const int i = f - t.first;
                load(i);
                const Space s = space(f);
                const V3 shoulder = ToWorld(s, ModelOf(aRig, aPose, upper).p);
                const V3 wrist = ToWorld(s, ModelOf(aRig, aPose, hand).p);
                const V3 want = p.atWrist ? p.target : p.target + (wrist - ToWorld(s, detail::Palm(aRig, aPose, side)));
                const double arm = legik::Len(aPose[low].p) + legik::Len(aPose[hand].p);
                const double allowed = std::max(kHeldReach * arm, Len(shoulder - want));
                if (Len(shoulder + t.root[i] - want) <= allowed)
                    continue;
                double lo = 0, hi = 1;
                for (int k = 0; k < 12; ++k)
                {
                    const double mid = 0.5 * (lo + hi);
                    (Len(shoulder + t.root[i] * mid - want) <= allowed ? lo : hi) = mid;
                }
                share[i] = std::min(share[i], lo);
            }
        }
        std::vector<double> loss(share.size());
        for (size_t i = 0; i < share.size(); ++i)
            loss[i] = 1.0 - share[i];
        detail::Smooth(loss, kSmoothFrames);
        for (int i = 0; i < count; ++i)
            t.root[i] = t.root[i] * std::clamp(1.0 - loss[i], 0.0, 1.0);
    }
    // 4: legs -> ankle displacement; a second pass adds what the first solve left
    std::vector<double> legUp[2], legOut[2];
    for (int side = 0; side < 2; ++side)
    {
        legUp[side].assign(static_cast<size_t>(count), 0.0);
        legOut[side].assign(static_cast<size_t>(count), 0.0);
        t.leg[side].assign(static_cast<size_t>(count), V3{});
    }
    for (int pass = 0; pass < kLegPasses; ++pass)
    {
        std::vector<double> addUp[2], addOut[2];
        for (int side = 0; side < 2; ++side)
        {
            addUp[side].assign(static_cast<size_t>(count), 0.0);
            addOut[side].assign(static_cast<size_t>(count), 0.0);
        }
        for (int i = 0; i < count; ++i)
        {
            load(i);
            const Space s = space(t.first + i);
            MoveRoot(aRig, aPose, ToModelDir(s, t.root[i]));
            for (int side = 0; side < 2; ++side)
            {
                if (pass > 0)
                    MoveAnkle(aRig, aPose, side, ToModelDir(s, t.leg[side][i]));
                double needUp = 0, needOut = 0;
                bool outWorks = true;
                ComputeModels(aRig, aPose, models);
                for (const BodyPoint& bp : kBodyPoints)
                {
                    if (bp.group != side + 1)
                        continue;
                    const WayOut c = Clearance(t.box, PointWorld(aRig, models, s, bp), bp.radius);
                    if (c.up <= 0)
                        continue;
                    needUp = std::max(needUp, c.up / bp.lever);
                    if (c.back < 0)
                        outWorks = false;
                    else
                        needOut = std::max(needOut, c.back / bp.lever);
                }
                if (needUp <= 0)
                    continue;
                if (outWorks && needOut < needUp)
                    addOut[side][i] = needOut;
                else
                    addUp[side][i] = needUp;
            }
        }
        for (int side = 0; side < 2; ++side)
        {
            std::vector<double> u = legUp[side], o = legOut[side];
            for (int i = 0; i < count; ++i)
            {
                u[i] = std::min(u[i] + addUp[side][i], kLegLiftMaxM);
                o[i] = std::min(o[i] + addOut[side][i], kFootAlongMaxM);
            }
            legUp[side] = u;
            legOut[side] = o;
            detail::Smooth(u, kSmoothFrames);
            detail::Smooth(o, kSmoothFrames);
            for (int i = 0; i < count; ++i)
                t.leg[side][i] = (V3{0, 0, std::min(u[i], kLegLiftMaxM)} + back * std::min(o[i], std::max(0.0, kFootAlongMaxM - out[i]))) *
                                 detail::EndTaper(i, count);
        }
    }
    t.valid = true;
    return t;
}

// Hand weight at a clip time (frames): eased in before the planted span and out after it, cut at the end of the track.
inline double HandWeight(const Track& aT, int aSide, double aFrame)
{
    const HandPlant& p = aT.hand[aSide];
    if (!p.valid)
        return 0;
    const double in = Ease((aFrame - (p.first - kHandRampInFrames)) / kHandRampInFrames);
    const double outW = Ease(((p.last + kHandRampOutFrames) - aFrame) / kHandRampOutFrames);
    return std::min(in, outW) * Ease((aFrame - aT.first) / 3.0);
}

// Per traversal: the hand weights applied on the last frame (rate limit: a hand that goes out of reach fades, never drops).
struct HandState
{
    double w[2] = {0, 0};
    double poseS = -1; // clip time of that frame: a montage played faster than real time moves its weights as fast
};

struct Applied
{
    V3 root;                // world root displacement used this frame
    LimbResult leg[2], arm[2];
};

// One published montage frame. aSpace = the frame the pose is shown in (origin = visual root of the frame, yaw = its yaw).
// aHandAllowed[side] = false leaves that arm alone (a weapon is held). aState / aDt: the hand weights of this traversal.
inline Applied Apply(const Track& aT, const Rig& aRig, BoneTransform* aPose, const Space& aSpace, double aPoseTimeS,
                     const bool (&aHandAllowed)[2], HandState& aState, double aDt)
{
    Applied a;
    if (!aT.valid || !aRig.ok || !std::isfinite(aPoseTimeS))
        return a;
    const double frame = aPoseTimeS * aT.fps;
    const double index = frame - aT.first;
    a.root = detail::Sample(aT.root, index);
    if (Len(a.root) > 1e-5)
        MoveRoot(aRig, aPose, ToModelDir(aSpace, a.root));
    for (int side = 0; side < 2; ++side)
        a.leg[side] = MoveAnkle(aRig, aPose, side, ToModelDir(aSpace, detail::Sample(aT.leg[side], index)));
    const double played = aState.poseS >= 0 ? std::max(0.0, aPoseTimeS - aState.poseS) : 0.0;
    aState.poseS = aPoseTimeS;
    const double step = kHandWeightRatePerS * std::clamp(std::max(std::isfinite(aDt) ? aDt : 0.0, played), 0.0, 0.1);
    const math::Vec3 rootModel = ToModelDir(aSpace, a.root);
    for (int side = 0; side < 2; ++side)
    {
        const double prev = aState.w[side];
        aState.w[side] = 0;
        if (!aHandAllowed[side] || !aT.hand[side].valid)
            continue;
        const double span = HandWeight(aT, side, frame);
        if (!(span > 1e-3) && !(prev > 1e-3))
            continue;
        const int up = aRig.id[side ? kRArm : kLArm], low = aRig.id[side ? kRForeArm : kLForeArm], hand = aRig.id[side ? kRHand : kLHand];
        const double arm = legik::Len(aPose[low].p) + legik::Len(aPose[hand].p);
        if (!(arm > 0.1))
            continue;
        const double ending = Ease((aT.last - frame) / 3.0); // nothing left on the track's last frame (the hand-back)
        const math::Vec3 shoulder = ModelOf(aRig, aPose, up).p;
        const math::Vec3 wrist = ModelOf(aRig, aPose, hand).p;
        const math::Vec3 palm = detail::Palm(aRig, aPose, side);
        const bool atWrist = aT.hand[side].atWrist;
        const math::Vec3 held = atWrist ? wrist : palm; // the joint that is placed
        const math::Vec3 target = ToModel(aSpace, aT.hand[side].target);
        const math::Vec3 want = target + (wrist - held); // the wrist, with the clip's hand orientation
        double w = span;
        w *= 1.0 - Ease((legik::Len(want - shoulder) / arm - kReachFadeFrom) / (kReachFadeTo - kReachFadeFrom));
        w *= 1.0 - Ease((legik::Len(want - wrist) - kMoveFadeFromM) / (kMoveFadeToM - kMoveFadeFromM));
        const double past = Along(aT.box, ToWorld(aSpace, shoulder)) - Along(aT.box, aT.hand[side].target);
        w *= 1.0 - Ease((past - kPastFadeFromM) / (kPastFadeToM - kPastFadeFromM)); // the body has gone by: the hand leaves the edge
        w = std::clamp(w, prev - step, prev + step);
        aState.w[side] = w;
        // during the span the hand starts from where the plan has it (the body's root displacement does not carry it along),
        // and from there moves onto the held point by the applied weight
        const math::Vec3 planned = held - rootModel * static_cast<float>(span * ending);
        const math::Vec3 goal = planned + (target - planned) * static_cast<float>(w * ending);
        if (legik::Len(goal - held) < 2e-3f)
            continue;
        a.arm[side] = SolveHand(aRig, aPose, side, goal, 1.0, atWrist);
        a.arm[side].weight = a.arm[side].applied ? w * ending : 0.0;
    }
    return a;
}
} // namespace utpp::mm::traversal::contact

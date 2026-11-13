#pragma once
// UTPP motion matching runtime - forearm roll convention of the CP rig (recording 2026-10-25, video 01:32).
// The game keeps every forearm a pure hinge: in the 2.5 game pose (stage s0) the LeftForeArm / RightForeArm local roll
// about the forearm -> hand axis is 0.0 deg in every overlay / action / aim state, and the roll sits in the hand. The
// deformation rig builds the forearm twist helpers from the hand (l_forearm_twist_start_GRP is parented to LeftHand), so
// roll stored in the forearm bone is not spread along the forearm: the skin turns at the elbow instead (Pistol2H relaxed
// idle, key 36: -125.4 deg in LeftForeArm -> pinched elbow, stretched-looking arm).
// MoveForearmTwistToHand: swing-twist decomposition of the forearm local rotation about the forearm -> hand axis
// (normalized local translation of the hand), q = swing * twist; forearm' = swing, every child c of the forearm gets
// c.r' = twist * c.r and c.t' = twist(c.t). Model transforms of the hand and of everything below it are unchanged (the
// hand translation lies on the twist axis), the forearm position is unchanged, and no other bone is written (clavicles
// and upper arms keep their locals bit for bit). Always applied in full: the game has no residual forearm roll, and a
// threshold would make the roll pop when crossed. Idempotent (a second pass finds no twist).
// Header-only, allocation-free, no game types. Quaternions x, y, z, w, Hamilton product (MMMath.hpp).
// Mutations (UTPP_FOREARMTWIST_MUTATION, tests/ForearmTwistTest.cpp): 1 = pass not called by PoseEvaluator (MotionPose.hpp),
// 2 = forearm children not compensated (hand model rotation changes).

#include <cmath>
#include <initializer_list>

#include "Additive.hpp"

namespace utpp::mm::ov
{
// Signed twist angle (deg) of local rotation aQ about the unit axis aAxis (swing-twist, twist part taken with w >= 0).
inline float TwistAngleDeg(const math::Quat& aQ, math::Vec3 aAxis)
{
    const float d = aQ.x * aAxis.x + aQ.y * aAxis.y + aQ.z * aAxis.z;
    float w = aQ.w, p = d;
    if (w < 0.0f)
    {
        w = -w;
        p = -p;
    }
    return 2.0f * std::atan2(p, w) * (180.0f / math::kPi);
}

struct ForearmTwistResult
{
    bool applied = false;
    float movedDeg = 0.0f; // twist removed from the forearm (signed, about the normalized hand translation)
};

// aForearm: forearm bone; aChildren / aChildCount: every direct child of the forearm in aSk (the hand among them);
// aHand: the hand (its local translation defines the axis). Skips (pose untouched) on a degenerate axis or twist.
inline ForearmTwistResult MoveForearmTwistToHand(Pose& aPose, int aForearm, int aHand, const int* aChildren, int aChildCount)
{
    ForearmTwistResult r;
    if (aForearm < 0 || aHand < 0 || aForearm >= aPose.n || aHand >= aPose.n)
        return r;
    const math::Vec3 ht = aPose.b[aHand].t;
    const float len = std::sqrt(math::Dot(ht, ht));
    if (!(len > 1e-5f) || !std::isfinite(len))
        return r;
    const math::Vec3 a = ht * (1.0f / len);
    const math::Quat q = math::QuatNormalize(aPose.b[aForearm].r);
    const float d = q.x * a.x + q.y * a.y + q.z * a.z;
    math::Quat twist{a.x * d, a.y * d, a.z * d, q.w};
    const float tn = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z + twist.w * twist.w);
    if (!(tn > 1e-6f) || !std::isfinite(tn)) // ~180 deg swing: twist undefined (not reachable by an elbow)
        return r;
    twist = math::Quat{twist.x / tn, twist.y / tn, twist.z / tn, twist.w / tn};
    if (twist.w < 0.0f)
        twist = math::QuatNeg(twist);
    r.movedDeg = TwistAngleDeg(q, a);
#if defined(UTPP_FOREARMTWIST_MUTATION) && UTPP_FOREARMTWIST_MUTATION == 2
    aPose.b[aForearm].r = math::QuatNormalize(math::QuatMul(q, math::QuatInv(twist))); // mutation 2: children not compensated
    (void)aChildren;
    (void)aChildCount;
#else
    aPose.b[aForearm].r = math::QuatNormalize(math::QuatMul(q, math::QuatInv(twist)));
    for (int i = 0; i < aChildCount; ++i)
    {
        const int c = aChildren[i];
        if (c < 0 || c >= aPose.n)
            continue;
        Xf& x = aPose.b[c];
        x.r = math::QuatNormalize(math::QuatMul(twist, x.r));
        if (c != aHand) // the hand translation is on the axis: unchanged by construction, keep it bit for bit
            x.t = math::QuatRotate(twist, x.t);
    }
#endif
    r.applied = true;
    return r;
}

// Both arms of a skeleton: forearm / hand indices and the forearm children, resolved once (Init time).
struct ForearmTwistRig
{
    static constexpr int kMaxChildren = 8;
    int forearm[2] = {-1, -1}, hand[2] = {-1, -1};
    int children[2][kMaxChildren]{};
    int childCount[2] = {0, 0};
    bool Valid(int aSide) const { return forearm[aSide] >= 0 && hand[aSide] >= 0 && childCount[aSide] > 0; }
    // aForearmL/R, aHandL/R: bone indices (-1 = side disabled). The hand must be a direct child of the forearm.
    void Build(const Skeleton& aSk, int aForearmL, int aHandL, int aForearmR, int aHandR)
    {
        const int fa[2] = {aForearmL, aForearmR}, hd[2] = {aHandL, aHandR};
        for (int s = 0; s < 2; ++s)
        {
            forearm[s] = hand[s] = -1;
            childCount[s] = 0;
            if (fa[s] < 0 || hd[s] < 0 || fa[s] >= aSk.n || hd[s] >= aSk.n || aSk.parent[hd[s]] != fa[s])
                continue;
            bool overflow = false;
            for (int i = 0; i < aSk.n; ++i)
                if (aSk.parent[i] == fa[s])
                {
                    if (childCount[s] >= kMaxChildren)
                        overflow = true;
                    else
                        children[s][childCount[s]++] = i;
                }
            if (overflow)
            {
                childCount[s] = 0;
                continue;
            }
            forearm[s] = fa[s];
            hand[s] = hd[s];
        }
    }
};

// Applies MoveForearmTwistToHand to both arms; aMovedDeg[side] receives the removed twist (0 when skipped).
inline void MoveForearmTwistToHands(const ForearmTwistRig& aRig, Pose& aPose, float (&aMovedDeg)[2])
{
    for (int s = 0; s < 2; ++s)
    {
        aMovedDeg[s] = 0.0f;
        if (!aRig.Valid(s))
            continue;
        aMovedDeg[s] = MoveForearmTwistToHand(aPose, aRig.forearm[s], aRig.hand[s], aRig.children[s], aRig.childCount[s]).movedDeg;
    }
}

// ---- support-hand twist limit (rifle-hand-fix 2026-11-09) ----------------------------------------------------------------
// The authored motion Pistol2H Relaxed / Ready poses leave up to ~130 deg of twist about the forearm -> hand axis in the support
// hand once the roll sits in the hand (above). The game's own Female V handgun support hands stay within 85 deg in 24 of 30 player
// poses (Nue 41-44, Ajax 73-85; swing-twist, the measure of the deformer graph TwistConstraint that drives the forearm twist helpers).
// LimitHandTwistBySwivel turns the upper arm about the shoulder -> wrist axis (elbow swivel) so that the hand twist drops to aLimitDeg:
//   upper' = R(aP, phi) * upper   (aP = shoulder -> wrist direction in the clavicle frame),
//   hand'  = R(aF, -phi) * hand   (aF = the same direction in the forearm frame),
// the forearm local rotation is unchanged (it stays a pure hinge) and the MODEL transform of the hand - so the weapon contact, the
// fingers, WeaponLeft and every bone below the hand - is unchanged exactly (the wrist lies on the swivel axis). phi = the smallest
// swivel (|phi| <= aMaxSwivelDeg) that reaches the limit; out of reach the one that lowers |twist| most, never one that raises it.
// aWeight scales phi (continuous; 0 = pose bit for bit).
// |twist| <= aLimitDeg: pose untouched bit for bit. Skips when the forearm has another child besides the hand (none in the CP
// 73-bone rig) or on degenerate input. Only the locals of aUpper and aHand are written.
struct HandTwistLimitResult
{
    bool applied = false;
    float twistBeforeDeg = 0.0f; // signed hand twist about the forearm -> hand axis (swing-twist, deg)
    float twistAfterDeg = 0.0f;
    float swivelDeg = 0.0f;      // applied swivel about the shoulder -> wrist axis (signed, deg)
};
inline constexpr float kSupportHandTwistLimitDeg = 85.0f;
inline constexpr float kSupportHandMaxSwivelDeg = 60.0f;

inline math::Quat AxisAngleQuat(math::Vec3 aUnit, float aDeg)
{
    const float h = 0.5f * aDeg * (math::kPi / 180.0f);
    const float s = std::sin(h);
    return math::Quat{aUnit.x * s, aUnit.y * s, aUnit.z * s, std::cos(h)};
}

inline HandTwistLimitResult LimitHandTwistBySwivel(Pose& aPose, int aUpper, int aForearm, int aHand, int aForearmChildCount,
                                                   float aLimitDeg, float aMaxSwivelDeg, float aWeight)
{
    HandTwistLimitResult r;
    if (aUpper < 0 || aForearm < 0 || aHand < 0 || aUpper >= aPose.n || aForearm >= aPose.n || aHand >= aPose.n || aForearmChildCount != 1)
        return r;
    const float w = Clamp01(aWeight);
    const math::Vec3 ht = aPose.b[aHand].t;
    const float hl = std::sqrt(math::Dot(ht, ht));
    if (!(hl > 1e-5f) || !std::isfinite(hl))
        return r;
    const math::Vec3 x = ht * (1.0f / hl);
    const math::Quat qUp = math::QuatNormalize(aPose.b[aUpper].r);
    const math::Quat qFo = math::QuatNormalize(aPose.b[aForearm].r);
    const math::Quat qHand = math::QuatNormalize(aPose.b[aHand].r);
    const math::Vec3 sw = aPose.b[aForearm].t + math::QuatRotate(qFo, ht); // shoulder -> wrist, upper-arm frame
    const float sl = std::sqrt(math::Dot(sw, sw));
    if (!(sl > 1e-5f) || !std::isfinite(sl))
        return r;
    const math::Vec3 aF = math::QuatRotate(math::QuatInv(qFo), sw) * (1.0f / sl); // forearm frame
    const math::Vec3 aP = math::QuatRotate(qUp, sw) * (1.0f / sl);                // clavicle (parent of the upper arm) frame
    const auto twistAt = [&](float aPhi) {
        return TwistAngleDeg(math::QuatNormalize(math::QuatMul(AxisAngleQuat(aF, -aPhi), qHand)), x);
    };
    r.twistBeforeDeg = r.twistAfterDeg = TwistAngleDeg(qHand, x);
    const float before = std::fabs(r.twistBeforeDeg);
    if (!std::isfinite(before) || !(before > aLimitDeg) || !(w > kZeroWeight) || !(aMaxSwivelDeg > 0.0f))
        return r;
    // Smallest swivel that reaches the limit: 12 steps outwards on both sides (the side that lowers |twist| first), then bisection
    // inside the step that crosses the limit. Out of reach (strongly bent elbow: the swivel axis leaves the forearm axis): the grid
    // swivel with the lowest |twist|, only if it lowers it; otherwise the pose stays untouched.
    const float dir = std::fabs(twistAt(1.0f)) <= std::fabs(twistAt(-1.0f)) ? 1.0f : -1.0f;
    constexpr int kSteps = 12;
    const float step = aMaxSwivelDeg / static_cast<float>(kSteps);
    float best = 0.0f, bestAbs = before;
    bool reached = false;
    for (int k = 1; k <= kSteps && !reached; ++k)
        for (const float side : {dir, -dir})
        {
            const float a = std::fabs(twistAt(side * step * static_cast<float>(k)));
            if (a <= aLimitDeg)
            {
                float lo = step * static_cast<float>(k - 1), hi = step * static_cast<float>(k); // |twist(lo)| > limit >= |twist(hi)|
                for (int i = 0; i < 16; ++i)
                {
                    const float mid = 0.5f * (lo + hi);
                    if (std::fabs(twistAt(side * mid)) > aLimitDeg)
                        lo = mid;
                    else
                        hi = mid;
                }
                best = side * hi;
                reached = true;
                break;
            }
            if (a < bestAbs)
            {
                bestAbs = a;
                best = side * step * static_cast<float>(k);
            }
        }
    if (best == 0.0f)
        return r;
    const float phi = best * w;
    const math::Quat up = math::QuatNormalize(math::QuatMul(AxisAngleQuat(aP, phi), qUp));
    const math::Quat hand = math::QuatNormalize(math::QuatMul(AxisAngleQuat(aF, -phi), qHand));
    if (!math::IsFinite(up) || !math::IsFinite(hand))
        return r;
    aPose.b[aUpper].r = up;
    aPose.b[aHand].r = hand;
    r.applied = true;
    r.swivelDeg = phi;
    r.twistAfterDeg = TwistAngleDeg(hand, x);
    return r;
}
} // namespace utpp::mm::ov

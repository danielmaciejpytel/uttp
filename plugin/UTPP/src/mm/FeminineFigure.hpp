#pragma once
// TPP posture only (female V): sloping, set-back shoulders, arms closer to the body while they hang free, a lower head.
// Works on the composed pose like the shoulder width pass (FeminineShoulders.hpp): the clavicles turn down and back, the arm
// sockets move a little further inward, the neck root moves down along its own offset. Never edits rig assets or scales.
//   * A free arm (no weapon pose composed) keeps its direction and moves with its socket, then turns inward by armsInDeg.
//   * An arm of a weapon pose keeps its hand exactly where it was (position and rotation; two-bone solve). Its share of the
//     shoulder change fades out as the arm gets close to full extension, so the solve never has to clamp (continuous in the pose).
// The caller supplies a scratch pose; any failure leaves the pose untouched.
#include "GripIK.hpp"

namespace utpp::mm::femfigure
{
struct Amounts
{
    float downDeg = 12.0f;   // clavicle: socket down
    float backDeg = 8.0f;    // clavicle: socket back
    float narrowM = -0.010f; // socket shift per side toward the spine (negative = outward: takes back what the clavicle turn adds inward)
    float armsInDeg = 7.5f;  // free arm toward the body
    float headDownM = 0.02f; // neck root down along its offset from the chest
};
inline constexpr float kReachRampM = 0.02f; // the weapon-arm share fades over this much reach headroom

struct Result
{
    bool applied = false;
    float weight = 0.0f;         // requested weight
    float sideWeight[2] = {};    // shoulder weight actually used per side (left, right)
    float handShift[2] = {};     // how far each hand moved (m): 0 for a weapon arm, the socket move + the inward turn for a free arm
    float handError[2] = {};     // |hand - target| after the solve (m)
    gik::Result arm[2];
};

// weight: 0..1 (the feminine style weight of the frame); freeShare: 0 = weapon pose (hands stay), 1 = free arms.
inline bool Apply(const ov::Skeleton& sk, ov::Pose& pose, ov::Pose& scratch, const gik::Rig& rig, int spine, int shoulderL,
                  int shoulderR, int neck, float weight, float freeShare, const gik::Params& params, Result& result,
                  const Amounts& amounts = Amounts{})
{
    result = Result{};
    const float w = ov::Clamp01(weight), fs = ov::Clamp01(freeShare);
    result.weight = w;
    if (!std::isfinite(weight) || !std::isfinite(freeShare) || &pose == &scratch) return false;
    if (w <= ov::kZeroWeight) return true;
    if (!sk.Valid() || pose.n != sk.n || !rig.Valid(sk) || spine < 0 || spine >= sk.n || shoulderL < 0 || shoulderR < 0 ||
        shoulderL >= sk.n || shoulderR >= sk.n || shoulderL == shoulderR || neck < 0 || neck >= sk.n ||
        sk.parent[shoulderL] != spine || sk.parent[shoulderR] != spine || sk.parent[rig.left.upper] != shoulderL ||
        sk.parent[rig.right.upper] != shoulderR) return false;
    for (int i = 0; i < sk.n; ++i)
        if (!math::IsFinite(pose.b[i].r) || !math::IsFinite(pose.b[i].t)) return false;

    ov::Quat spineRot, clavRot[2], q;
    ov::Vec3 spinePos, clavPos[2], neckPos;
    const int clav[2] = {shoulderL, shoulderR};
    const gik::ArmChain* chain[2] = {&rig.left, &rig.right};
    gik::ModelOf(sk, pose, spine, spineRot, spinePos);
    gik::ModelOf(sk, pose, shoulderL, clavRot[0], clavPos[0]);
    gik::ModelOf(sk, pose, shoulderR, clavRot[1], clavPos[1]);
    gik::ModelOf(sk, pose, neck, q, neckPos);
    ov::Vec3 lateral, up, forward;
    if (!gik::Norm(clavPos[1] - clavPos[0], lateral)) return false;
    ov::Vec3 upRaw = neckPos - spinePos;
    upRaw = upRaw - lateral * math::Dot(upRaw, lateral);
    if (!gik::Norm(upRaw, up)) return false;
    if (!gik::Norm(math::Cross(up, lateral), forward)) return false;

    scratch = pose;
    const float kDeg = 0.017453292519943295f;
    for (int side = 0; side < 2; ++side)
    {
        const float s = side == 0 ? -1.0f : 1.0f;
        const gik::ArmChain& c = *chain[side];
        ov::Quat upperRot, handRot;
        ov::Vec3 socket, handPos;
        gik::ModelOf(sk, pose, c.upper, upperRot, socket);
        gik::ModelOf(sk, pose, c.hand, handRot, handPos);
        // full arm length from the pose (rigid chain)
        ov::Quat lowerRot; ov::Vec3 elbow;
        gik::ModelOf(sk, pose, c.lower, lowerRot, elbow);
        const float full = gik::Len(elbow - socket) + gik::Len(handPos - elbow);
        if (!(full > 1e-3f)) return false;

        // socket move at full weight (clavicle turn about its root + inward shift)
        const auto socketAt = [&](float sw, ov::Quat& clavNew) {
            const ov::Quat d = math::QuatNormalize(math::QuatMul(gik::AxisAngle(up, -s * amounts.backDeg * kDeg * sw),
                                                                gik::AxisAngle(forward, s * amounts.downDeg * kDeg * sw)));
            clavNew = math::QuatNormalize(math::QuatMul(d, clavRot[side]));
            return clavPos[side] + math::QuatRotate(d, socket - clavPos[side]) - lateral * (s * amounts.narrowM * sw);
        };
        // weapon-arm share: fades as the fixed hand would need more reach than the solver gives without clamping
        ov::Quat clavFull;
        const ov::Vec3 socketFull = socketAt(w, clavFull);
        const float need = std::fmax(gik::Len(handPos - socketFull), gik::Len(handPos - socket));
        const float room = params.softStart * full - need;
        const float armed = ov::Clamp01(room / kReachRampM);
        const float sw = w * (fs + (1.0f - fs) * armed);
        result.sideWeight[side] = sw;
        if (sw <= ov::kZeroWeight) continue;

        ov::Quat clavNew;
        const ov::Vec3 socketNew = socketAt(sw, clavNew);
        // clavicle local rotation; the upper arm keeps its model rotation and takes the new socket
        const ov::Quat parentRot = spineRot;
        scratch.b[clav[side]].r = math::QuatNormalize(math::QuatMul(math::QuatInv(parentRot), clavNew));
        scratch.b[c.upper].r = math::QuatNormalize(math::QuatMul(math::QuatInv(clavNew), upperRot));
        scratch.b[c.upper].t = math::QuatRotate(math::QuatInv(clavNew), socketNew - clavPos[side]);
        // hand target: a weapon arm keeps the hand; a free arm moves with the socket and turns inward
        const ov::Vec3 moved = handPos + (socketNew - socket);
        const ov::Vec3 arm = moved - socketNew;
        const ov::Vec3 freeTarget = socketNew + math::QuatRotate(gik::AxisAngle(forward, s * amounts.armsInDeg * kDeg * sw), arm);
        const ov::Vec3 target = handPos + (freeTarget - handPos) * fs;
        gik::Params p = params;
        if (fs >= 1.0f) { p.softStart = 0.9995f; p.maxReach = 0.99995f; } // a free arm keeps its own extension
        if (!gik::SolveArm(sk, scratch, c, target, &handRot, 1.0f, p, result.arm[side]) || !result.arm[side].ok) return false;
        ov::Quat r2; ov::Vec3 p2;
        gik::ModelOf(sk, scratch, c.hand, r2, p2);
        result.handError[side] = gik::Len(p2 - target);
        result.handShift[side] = gik::Len(p2 - handPos);
        if (!std::isfinite(result.handError[side])) return false;
    }
    // head: the neck root moves down along its own offset from its parent
    ov::Vec3 dir;
    if (gik::Norm(scratch.b[neck].t, dir))
        scratch.b[neck].t = scratch.b[neck].t - dir * (amounts.headDownM * w);
    for (int i = 0; i < sk.n; ++i)
        if (!math::IsFinite(scratch.b[i].r) || !math::IsFinite(scratch.b[i].t)) return false;
    pose = scratch;
    result.applied = true;
    return true;
}
} // namespace utpp::mm::femfigure

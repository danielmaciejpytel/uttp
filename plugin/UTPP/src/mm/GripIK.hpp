// UTPP motion matching runtime - weapon grip hand IK after ALS overlay layering (port stage 10.6,
// "Hand IK", weapons, PoseHook order
// "BlendIntoRecords -> grip IK -> aimofs::Apply").
// Header-only, allocation-free, no game types. Works on the CP rig in the ov:: pose types (Additive.hpp): local pose,
// bones in topological order, model space = composition of local transforms from the root.
//
// motion reference: after FootPlacement / LegIK, FABRIK hand_r -> weapon_l * HandIK_Right (alpha
// Enable_HandIK_R), then hand_l -> weapon_r * HandIK_Left (alpha Enable_HandIK_L); skipped while the DefaultSlot
// montage is active or the previous frame was in air. UTPP port on the CP rig:
//   - the weapon is attached to WeaponRight (child of RightHand), so the right hand is on the weapon by construction;
//     the optional right-hand IK pulls RightHand to a caller target (e.g. the game pose hand) with Enable_HandIK_R;
//   - left hand target = grip point in WeaponRight space: (a) cached from the GAME pose before the overlay blend
//     (CacheGripFromGamePose: LeftHand relative to WeaponRight, the game's own left-hand weapon IK has placed it), or
//     (b) a per-weapon-class offset table (data, ParseGripTable); then IK with Enable_HandIK_L after the layering;
//   - order right, then left (the left target follows the weapon moved by the right arm).
// Solver: analytic two-bone IK per arm (LeftArm -> LeftForeArm -> LeftHand), elbow plane preserved from the input
// pose (pole = elbow offset from the shoulder-hand line of the INPUT pose; fallback pole when the input arm is
// straight), soft reach clamp (no over-extension, never fully straight -> no elbow flips), minimum reach clamp,
// result blended with the input LOCAL rotations by the curve weight (weight 0 = input pose bit for bit, continuous in
// the weight). Only local rotations of upper / lower / hand change; bone lengths and translations are untouched.
// Quaternions x, y, z, w, Hamilton product (MMMath.hpp).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string_view>

#include "Additive.hpp"
#include "Overlay.hpp"

namespace utpp::mm::gik
{
using math::Quat;
using math::Vec3;
using ov::Pose;
using ov::Skeleton;
using ov::Xf;

struct ArmChain
{
    int upper = -1, lower = -1, hand = -1; // CP: LeftArm 17, LeftForeArm 20, LeftHand 23 / RightArm 18, 21, 24
    bool Valid(const Skeleton& aSk) const
    {
        return upper >= 0 && lower > upper && hand > lower && hand < aSk.n && aSk.parent[lower] == upper &&
               aSk.parent[hand] == lower;
    }
};

struct Rig
{
    ArmChain left, right;
    int weaponLeft = -1, weaponRight = -1; // CP: WeaponLeft 26 (child of LeftHand), WeaponRight 28 (child of RightHand)
    bool Valid(const Skeleton& aSk) const
    {
        return left.Valid(aSk) && right.Valid(aSk) && weaponRight > 0 && weaponRight < aSk.n &&
               aSk.parent[weaponRight] == right.hand && (weaponLeft < 0 || aSk.parent[weaponLeft] == left.hand);
    }
};

// aName(i) -> std::string_view bone name (e.g. OverlayFile::BoneName). CP names (player_woman_skeleton.rig).
template <class NameFn>
inline Rig FindRig(int aCount, NameFn aName)
{
    auto find = [&](std::string_view n) {
        for (int i = 0; i < aCount; ++i)
            if (aName(i) == n)
                return i;
        return -1;
    };
    Rig r;
    r.left = {find("LeftArm"), find("LeftForeArm"), find("LeftHand")};
    r.right = {find("RightArm"), find("RightForeArm"), find("RightHand")};
    r.weaponLeft = find("WeaponLeft");
    r.weaponRight = find("WeaponRight");
    return r;
}

struct Params
{
    float softStart = 0.95f; // fraction of the full arm length where the soft reach clamp starts
    float maxReach = 0.995f; // asymptote of the soft clamp (fraction of the full length): never fully straight
    float minReach = 0.20f;  // fraction of the full length (and >= |upper - lower| + eps): no folding through
    Vec3 fallbackPole{0.0f, 0.0f, -1.0f}; // model-space elbow direction when the input arm is straight (CP: Z up)
};

struct Result
{
    bool ok = false;      // false = chain invalid / degenerate / non-finite: pose left as it was
    bool clamped = false; // target outside [minReach, softStart] reach band
    float reqDist = 0.0f; // |target - shoulder| requested (m)
    float useDist = 0.0f; // after the clamp (m)
    float handErr = 0.0f; // |hand - target| after the solve at full weight (m)
};

// ---- small math -----------------------------------------------------------------------------------------------
inline float Len(Vec3 v) { return std::sqrt(math::Dot(v, v)); }
inline bool Norm(Vec3 v, Vec3& aOut, float aEps = 1e-7f)
{
    const float l = Len(v);
    if (!(l > aEps) || !std::isfinite(l))
        return false;
    aOut = v * (1.0f / l);
    return true;
}
inline Quat AxisAngle(Vec3 aAxisUnit, float aAngle)
{
    const float h = 0.5f * aAngle, s = std::sin(h);
    return Quat{aAxisUnit.x * s, aAxisUnit.y * s, aAxisUnit.z * s, std::cos(h)};
}
// Rotation matrix columns (orthonormal frame) -> quaternion.
inline Quat QuatFromFrame(Vec3 x, Vec3 y, Vec3 z)
{
    const float m00 = x.x, m10 = x.y, m20 = x.z, m01 = y.x, m11 = y.y, m21 = y.z, m02 = z.x, m12 = z.y, m22 = z.z;
    const float tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0.0f)
    {
        const float s = std::sqrt(tr + 1.0f) * 2.0f;
        q = Quat{(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    }
    else if (m00 > m11 && m00 > m22)
    {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = Quat{0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    }
    else if (m11 > m22)
    {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = Quat{(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    }
    else
    {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = Quat{(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    return math::QuatNormalize(q);
}

// Model transform of bone aBone (rotation, position) without computing the whole pose.
inline void ModelOf(const Skeleton& aSk, const Pose& aPose, int aBone, Quat& aRot, Vec3& aPos)
{
    int chain[ov::kMaxBones];
    int n = 0;
    for (int b = aBone; b >= 0 && n < ov::kMaxBones; b = aSk.parent[b])
        chain[n++] = b;
    aRot = Quat{};
    aPos = Vec3{};
    for (int k = n - 1; k >= 0; --k)
    {
        const Xf& x = aPose.b[chain[k]];
        aPos = aPos + math::QuatRotate(aRot, x.t);
        aRot = math::QuatNormalize(math::QuatMul(aRot, x.r));
    }
}

// Soft reach clamp: identity below softStart * L, exponential approach to maxReach * L above it (C1 continuous).
inline float SoftReach(float d, float aFull, const Params& aP, float aMinAbs)
{
    const float ds = aP.softStart * aFull, dmax = aP.maxReach * aFull;
    float out = d;
    if (d > ds && dmax > ds)
    {
        const float span = dmax - ds;
        out = ds + span * (1.0f - std::exp(-(d - ds) / span));
    }
    const float dmin = std::fmax(aP.minReach * aFull, aMinAbs);
    return out < dmin ? dmin : out;
}

// Two-bone IK of one arm in model space. aTargetPos: model-space hand position; aTargetRot: model-space hand rotation
// (nullptr = keep the input model rotation of the hand). aWeight in [0, 1]: blend of the solved local rotations with
// the input ones (weight 0 returns without touching the pose). The pose is modified only when ok.
inline bool SolveArm(const Skeleton& aSk, Pose& aIo, const ArmChain& aC, Vec3 aTargetPos, const Quat* aTargetRot,
                     float aWeight, const Params& aP, Result& aOut)
{
    aOut = Result{};
    const float w = ov::Clamp01(aWeight);
    if (!aC.Valid(aSk) || !math::IsFinite(aTargetPos) || (aTargetRot && !math::IsFinite(*aTargetRot)))
        return false;
    Quat rUp, rLo, rHand, rPar;
    Vec3 a, b, c, pPar;
    ModelOf(aSk, aIo, aC.upper, rUp, a);
    const int par = aSk.parent[aC.upper];
    if (par >= 0)
        ModelOf(aSk, aIo, par, rPar, pPar);
    else
        rPar = Quat{};
    rLo = math::QuatNormalize(math::QuatMul(rUp, aIo.b[aC.lower].r));
    b = a + math::QuatRotate(rUp, aIo.b[aC.lower].t);
    rHand = math::QuatNormalize(math::QuatMul(rLo, aIo.b[aC.hand].r));
    c = b + math::QuatRotate(rLo, aIo.b[aC.hand].t);
    const float lab = Len(b - a), lbc = Len(c - b), full = lab + lbc;
    if (!(lab > 1e-4f) || !(lbc > 1e-4f))
        return false;

    // pole: elbow offset from the shoulder-hand line of the input pose
    Vec3 ac;
    Vec3 pole;
    bool straight = !Norm(c - a, ac);
    if (!straight)
    {
        const Vec3 ab = b - a;
        straight = !Norm(ab - ac * math::Dot(ab, ac), pole, 1e-4f * full);
    }
    if (straight)
    {
        if (!Norm(aP.fallbackPole, pole))
            return false;
        if (Norm(c - a, ac))
            if (!Norm(pole - ac * math::Dot(pole, ac), pole))
                return false;
    }

    const Vec3 toT = aTargetPos - a;
    aOut.reqDist = Len(toT);
    const float minAbs = std::fabs(lab - lbc) + 1e-4f * full;
    const float d = SoftReach(aOut.reqDist, full, aP, minAbs);
    aOut.useDist = d;
    aOut.clamped = std::fabs(d - aOut.reqDist) > 1e-6f;
    Vec3 dir;
    if (!Norm(toT, dir))
        dir = straight || !Norm(c - a, dir) ? pole : dir; // target on the shoulder: keep the input direction
    Vec3 polePerp;
    if (!Norm(pole - dir * math::Dot(pole, dir), polePerp, 1e-4f))
    {
        // target along the pole: use the input bend-plane side (perpendicular to dir, closest to the input elbow)
        const Vec3 n0 = math::Cross(ac, pole);
        if (!Norm(math::Cross(n0, dir), polePerp, 1e-6f))
        {
            const Vec3 any = std::fabs(dir.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            if (!Norm(any - dir * math::Dot(any, dir), polePerp))
                return false;
        }
    }
    // new elbow: law of cosines
    const float x = (lab * lab - lbc * lbc + d * d) / (2.0f * d);
    const float h = std::sqrt(std::fmax(lab * lab - x * x, 0.0f));
    const Vec3 e = a + dir * x + polePerp * h;
    const Vec3 hNew = a + dir * d;

    // upper: frame (bone dir, bend-plane perp) of the input -> new
    auto frameRot = [](Vec3 aFrom0, Vec3 aFromSide, Vec3 aTo0, Vec3 aToSide, Quat& aQ) {
        Vec3 x0, y0, x1, y1;
        if (!Norm(aFrom0, x0) || !Norm(aTo0, x1))
            return false;
        if (!Norm(aFromSide - x0 * math::Dot(aFromSide, x0), y0, 1e-6f) ||
            !Norm(aToSide - x1 * math::Dot(aToSide, x1), y1, 1e-6f))
            return false;
        const Vec3 z0 = math::Cross(x0, y0), z1 = math::Cross(x1, y1);
        const Quat f0 = QuatFromFrame(x0, y0, z0), f1 = QuatFromFrame(x1, y1, z1);
        aQ = math::QuatNormalize(math::QuatMul(f1, math::QuatInv(f0)));
        return true;
    };
    // side vector of the input upper bone: toward the input hand (bend plane), or the pole when straight
    const Vec3 sideIn = straight ? pole : (c - a);
    const Vec3 sideOut = straight ? polePerp : (hNew - a);
    Quat qUp, qLo;
    if (!frameRot(b - a, sideIn, e - a, sideOut, qUp))
        return false;
    // lower: frame (forearm dir, toward the shoulder) input -> new
    const Vec3 sideLoIn = straight ? pole : (a - b);
    const Vec3 sideLoOut = straight ? polePerp : (a - e);
    if (!frameRot(c - b, sideLoIn, hNew - e, sideLoOut, qLo))
        return false;
    const Quat rUpN = math::QuatNormalize(math::QuatMul(qUp, rUp));
    const Quat rLoN = math::QuatNormalize(math::QuatMul(qLo, rLo));
    const Quat rHandN = aTargetRot ? math::QuatNormalize(*aTargetRot) : rHand;

    Xf solved[3] = {aIo.b[aC.upper], aIo.b[aC.lower], aIo.b[aC.hand]};
    solved[0].r = math::QuatNormalize(math::QuatMul(math::QuatInv(rPar), rUpN));
    solved[1].r = math::QuatNormalize(math::QuatMul(math::QuatInv(rUpN), rLoN));
    solved[2].r = math::QuatNormalize(math::QuatMul(math::QuatInv(rLoN), rHandN));
    for (const Xf& s : solved)
        if (!math::IsFinite(s.r))
            return false;
    // full-weight hand error (diagnostic)
    {
        const Vec3 bN = a + math::QuatRotate(rUpN, aIo.b[aC.lower].t);
        const Vec3 cN = bN + math::QuatRotate(rLoN, aIo.b[aC.hand].t);
        aOut.handErr = Len(cN - aTargetPos);
    }
    aOut.ok = true;
    if (w <= ov::kZeroWeight)
        return true; // weight 0: input pose untouched
    const int idx[3] = {aC.upper, aC.lower, aC.hand};
    for (int k = 0; k < 3; ++k)
        aIo.b[idx[k]].r = w >= 1.0f ? solved[k].r : ov::Nlerp(aIo.b[idx[k]].r, solved[k].r, w);
    return true;
}

// ---- grip sources -----------------------------------------------------------------------------------------------
// Left-hand grip in WeaponRight space (position + rotation of LeftHand relative to WeaponRight).
struct Grip
{
    Quat r{};
    Vec3 t{};
    bool valid = false;
};

// (a) Cache from the game pose BEFORE the overlay blend: the game's left-hand weapon IK has put LeftHand on the grip.
inline Grip CacheGripFromGamePose(const Skeleton& aSk, const Pose& aGamePose, const Rig& aRig)
{
    Grip g;
    if (!aRig.Valid(aSk))
        return g;
    Quat rw, rl;
    Vec3 pw, pl;
    ModelOf(aSk, aGamePose, aRig.weaponRight, rw, pw);
    ModelOf(aSk, aGamePose, aRig.left.hand, rl, pl);
    const Quat inv = math::QuatInv(rw);
    g.r = math::QuatNormalize(math::QuatMul(inv, rl));
    g.t = math::QuatRotate(inv, pl - pw);
    g.valid = math::IsFinite(g.r) && math::IsFinite(g.t);
    return g;
}

// (b) Per-weapon-class table (data). Text lines: "<class> tx ty tz qx qy qz qw" (WeaponRight space, metres), '#'
// comments. ParseGripTable fills up to aCap entries, returns the count or -1 on a malformed line (fail-closed).
struct GripEntry
{
    char cls[32]{};
    Grip grip;
};
inline int ParseGripTable(std::string_view aText, GripEntry* aOut, int aCap)
{
    int count = 0;
    size_t pos = 0;
    while (pos < aText.size())
    {
        size_t end = aText.find('\n', pos);
        if (end == std::string_view::npos)
            end = aText.size();
        std::string_view line = aText.substr(pos, end - pos);
        pos = end + 1;
        if (const size_t h = line.find('#'); h != std::string_view::npos)
            line = line.substr(0, h);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.remove_suffix(1);
        while (!line.empty() && (line.front() == ' ' || line.front() == '\t'))
            line.remove_prefix(1);
        if (line.empty())
            continue;
        if (count >= aCap)
            return -1;
        char buf[256];
        if (line.size() >= sizeof(buf))
            return -1;
        line.copy(buf, line.size());
        buf[line.size()] = 0;
        char* p = buf;
        while (*p && *p != ' ' && *p != '\t')
            ++p;
        const size_t nameLen = static_cast<size_t>(p - buf);
        if (nameLen == 0 || nameLen >= sizeof(GripEntry::cls))
            return -1;
        GripEntry& e = aOut[count];
        e = GripEntry{};
        for (size_t i = 0; i < nameLen; ++i)
            e.cls[i] = buf[i];
        float v[7];
        for (float& f : v)
        {
            char* q = nullptr;
            f = std::strtof(p, &q);
            if (q == p || !std::isfinite(f))
                return -1;
            p = q;
        }
        while (*p == ' ' || *p == '\t')
            ++p;
        if (*p != 0)
            return -1;
        const Quat r{v[3], v[4], v[5], v[6]};
        const float qn = std::sqrt(math::QuatDot(r, r));
        if (!(std::fabs(qn - 1.0f) < 1e-3f))
            return -1;
        e.grip = Grip{math::QuatNormalize(r), Vec3{v[0], v[1], v[2]}, true};
        ++count;
    }
    return count;
}
inline const Grip* FindGrip(const GripEntry* aTable, int aCount, std::string_view aClass)
{
    for (int i = 0; i < aCount; ++i)
        if (std::string_view(aTable[i].cls) == aClass)
            return &aTable[i].grip;
    return nullptr;
}

// ---- full grip pass (after the overlay layering, before aimofs::Apply) -------------------------------------------
struct GripInputs
{
    const Grip* leftGrip = nullptr;      // WeaponRight-space grip (cache or table); nullptr = no left IK
    const Xf* rightTarget = nullptr;     // model-space RightHand target (r, t); nullptr = no right IK
    float handIkL = 0.0f, handIkR = 0.0f; // Enable_HandIK_L / R (ov::kHandIKL / kHandIKR of the layered curves)
    bool skip = false;                   // DefaultSlot montage active or previous frame in air
    bool coupleWeaponHands = false;      // authored two-hand contact: preserve weapon rotation, fit within both arm reaches
};
struct GripResult
{
    Result left, right;
    float weaponShift = 0.0f;
};
inline GripInputs InputsFromCurves(const ov::Curves& aC, const Grip* aLeft, const Xf* aRight)
{
    GripInputs in;
    in.leftGrip = aLeft;
    in.rightTarget = aRight;
    in.handIkL = (aC.present >> ov::kHandIKL) & 1u ? aC[ov::kHandIKL] : 0.0f;
    in.handIkR = (aC.present >> ov::kHandIKR) & 1u ? aC[ov::kHandIKR] : 0.0f;
    return in;
}
inline bool ApplyGripIK(const Skeleton& aSk, Pose& aIo, const Rig& aRig, const GripInputs& aIn, const Params& aP,
                        GripResult& aOut)
{
    aOut = GripResult{};
    if (aIn.skip || !aRig.Valid(aSk))
        return false;
    if (aIn.rightTarget && ov::Clamp01(aIn.handIkR) > ov::kZeroWeight)
        SolveArm(aSk, aIo, aRig.right, aIn.rightTarget->t, &aIn.rightTarget->r, aIn.handIkR, aP, aOut.right);
    if (aIn.leftGrip && aIn.leftGrip->valid && ov::Clamp01(aIn.handIkL) > ov::kZeroWeight)
    {
        if (aIn.coupleWeaponHands && !aIn.rightTarget)
        {
            Quat weaponRot, shoulderRot, handRot;
            Vec3 weaponStart, shoulder, handPos;
            ModelOf(aSk, aIo, aRig.weaponRight, weaponRot, weaponStart);
            ModelOf(aSk, aIo, aRig.left.upper, shoulderRot, shoulder);
            ModelOf(aSk, aIo, aRig.right.hand, handRot, handPos);
            const float full = Len(aIo.b[aRig.left.lower].t) + Len(aIo.b[aRig.left.hand].t);
            const Vec3 target = weaponStart + math::QuatRotate(weaponRot, aIn.leftGrip->t);
            const Vec3 delta = target - shoulder;
            const float distance = Len(delta), safe = full * aP.softStart;
            if (std::isfinite(distance) && distance > safe && safe > 0.0f)
            {
                const Vec3 shift = delta * ((safe - distance) / distance);
                // Refuse large corrections from an unrelated target. No limb stretching or unbounded iteration.
                if (Len(shift) <= full * 0.10f)
                {
                    SolveArm(aSk, aIo, aRig.right, handPos + shift, &handRot,
                             ov::Clamp01(aIn.handIkL), aP, aOut.right);
                    Vec3 moved; Quat rotated;
                    ModelOf(aSk, aIo, aRig.weaponRight, rotated, moved);
                    aOut.weaponShift = Len(moved - weaponStart);
                }
            }
        }
        Quat rw;
        Vec3 pw;
        ModelOf(aSk, aIo, aRig.weaponRight, rw, pw);
        const Vec3 tp = pw + math::QuatRotate(rw, aIn.leftGrip->t);
        const Quat tr = math::QuatNormalize(math::QuatMul(rw, aIn.leftGrip->r));
        SolveArm(aSk, aIo, aRig.left, tp, &tr, aIn.handIkL, aP, aOut.left);
    }
    return true;
}
} // namespace utpp::mm::gik

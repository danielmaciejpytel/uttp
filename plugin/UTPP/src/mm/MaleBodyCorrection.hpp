#pragma once
// UTPP motion matching runtime - male V upper-body deformation correction. Header-only, allocation-free, no game types.
//
// Why: the offline skin measurement (Source/tools/analysis/male_body_correction_eval.py,
// same game deformer graph + linear-blend skin for both bodies, poses of the female and male retargets are the same data) shows that
// the male body mesh is stressed far more than the female one by the SAME arm pose: armpit / lateral chest edges collapse in the weapon
// aim poses and in run / walk (P2H stand aim: severity 11.1 male vs 4.4 female). The gap lives in the male mesh / deformer graph (the female
// V additionally gets the feminine shoulder pass); a pose-space correction can only offset a part of it.
//
// What: WORLD-PRESERVING bone rotations. For a bone b and a rotation D (expressed in the local frame of b)
//     b.r' = b.r * D,   for every child c of b:   c.r' = D^-1 * c.r,   c.t' = D^-1 * c.t
// leaves the model transform (position AND rotation) of every bone except b itself unchanged: hands, weapons, fingers, head, feet and the
// shoulder sockets (the width of the shoulders) keep their place bit-for-bit-ish (float rounding), only the skin of the corrected bone rotates
// relative to its neighbours. The correction rotates Spine3 (upper chest), both clavicles and rolls both upper arms about their length axis (+X);
// its rotation vectors (degrees, axis * angle in the bone-local frame) come from a per-family table fitted offline against the measured edge
// stretch. A family is the weapon class x stance of the pose keys
// of the overlay machine (key = overlay * 12 + aim * 4 + crouch * 2 + moving, Overlay.hpp), plus the unarmed locomotion family; the family
// weights are the pose-key weights, so a pose blend blends the correction (continuous, no pop). Strength scales every rotation (0 = identity,
// bit-identical pose; the config default is chosen by the caller). Female V: never called (caller gates on !bodyWoman), the female path is unchanged.
//
// Call after the whole layering (compose, AO, grip IK, forearm twist) on the final local pose: ApplyMaleBodyCorrection(...).
// Mutations (UTPP_MALECORR_MUTATION, tests/MaleBodyCorrectionTest.cpp): 1 = children not compensated, 2 = key->family map ignores the crouch bit,
// 3 = weights not clamped (the sum of the family weights may exceed 1).

#include <algorithm>
#include <cmath>

#include "Overlay.hpp"

namespace utpp::mm::ov
{
enum class MaleCorrFamily : int
{
    StandRifle = 0,
    StandP1H,
    StandP2H,
    CrouchRifle,
    CrouchP1H,
    CrouchP2H,
    Loco, // unarmed / default overlay (walk, run, sprint, crouch idle and move)
    kCount,
    None = kCount // overlays that were not measured (Injured, Game, ...) and invalid keys: no correction
};
inline constexpr int kMaleCorrFamilyCount = static_cast<int>(MaleCorrFamily::kCount);

// One family: rotation vectors in degrees (axis * angle, bone-local frame, post-multiplied) and the upper-arm roll about +X.
// Config default (key mm.maleBodyCorrection, 0..1): full table strength. No in-game evidence yet - 0 switches the pass off, bit-identical to before.
inline constexpr float kMaleBodyCorrectionDefaultStrength = 0.0f;  // 2026-11-06: off - renders show shrugged shoulders / twisted torso (metric improved, look worse)

struct MaleCorrectionSet
{
    float spine3[3];
    float clav[2][3]; // [0] LeftShoulder, [1] RightShoulder
    float roll[2];    // [0] LeftArm, [1] RightArm
};

// Fitted offline (male_body_correction_eval.py, family fits on the measured clips; see the JSON named above). Order = MaleCorrFamily.
inline constexpr MaleCorrectionSet kMaleCorrectionTable[kMaleCorrFamilyCount] = {
    /*StandRifle */ {{10.0f, -10.0f, -10.0f}, {{-10.0f, 10.0f, 10.0f}, {4.0f, -10.0f, 0.0f}}, {-10.0f, 10.0f}},
    /*StandP1H   */ {{0.0f, 10.0f, -10.0f}, {{-9.0f, -1.0f, 10.0f}, {1.0f, -4.0f, 0.0f}}, {0.0f, 10.0f}},
    /*StandP2H   */ {{10.0f, -10.0f, -10.0f}, {{-10.0f, 6.0f, 10.0f}, {8.0f, -10.0f, 2.0f}}, {-10.0f, 10.0f}},
    /*CrouchRifle*/ {{10.0f, -10.0f, -3.0f}, {{-10.0f, 10.0f, 7.0f}, {1.0f, -10.0f, 0.0f}}, {-10.0f, 10.0f}},
    /*CrouchP1H  */ {{10.0f, -10.0f, 0.0f}, {{-10.0f, 8.0f, 10.0f}, {0.0f, -1.0f, 2.0f}}, {-10.0f, 4.0f}},
    /*CrouchP2H  */ {{8.0f, -6.0f, -10.0f}, {{8.0f, -8.0f, 0.0f}, {8.0f, -10.0f, 0.0f}}, {2.0f, 10.0f}},
    /*Loco       */ {{4.0f, 10.0f, -10.0f}, {{0.0f, -2.0f, 10.0f}, {-10.0f, 0.0f, 10.0f}}, {10.0f, -10.0f}},
};

// Per family and aim state of the pose key (0 relaxed, 1 ready, 2 aiming): share of the family correction that applies. The family fits improve
// almost every pose; where the offline measurement shows a regression of a whole state (Pistol2H crouch ready) the share is 0; locomotion is halved.
inline constexpr float kMaleCorrAimGate[kMaleCorrFamilyCount][3] = {
    /*StandRifle  */ {1.0f, 1.0f, 1.0f},
    /*StandP1H    */ {1.0f, 1.0f, 1.0f},
    /*StandP2H    */ {1.0f, 1.0f, 1.0f},
    /*CrouchRifle */ {1.0f, 1.0f, 1.0f},
    /*CrouchP1H   */ {1.0f, 1.0f, 1.0f},
    /*CrouchP2H   */ {1.0f, 0.0f, 1.0f},
    /*Loco        */ {0.5f, 0.5f, 0.5f}, // measured weakest (arm swing is not constant): half strength, held-out run frames neutral instead of worse
};

struct MaleCorrFamilyWeights
{
    float w[kMaleCorrFamilyCount] = {};
    float Sum() const
    {
        float s = 0.0f;
        for (float v : w)
            s += v;
        return s;
    }
};

// Pose key -> family (key = overlay * 12 + aim * 4 + crouch * 2 + moving; overlay: 0 Default -> Loco, 1 Rifle, 2 Pistol1H, 3 Pistol2H; others / invalid -> None).
inline MaleCorrFamily MaleCorrFamilyOfKey(int aKey)
{
    if (aKey < 0)
        return MaleCorrFamily::None;
    const int overlay = aKey / 12;
#if defined(UTPP_MALECORR_MUTATION) && UTPP_MALECORR_MUTATION == 2
    const bool crouch = false; // mutation 2: crouch bit ignored
#else
    const bool crouch = ((aKey % 12) / 2) % 2 == 1;
#endif
    switch (overlay)
    {
    case 1: return crouch ? MaleCorrFamily::CrouchRifle : MaleCorrFamily::StandRifle;
    case 2: return crouch ? MaleCorrFamily::CrouchP1H : MaleCorrFamily::StandP1H;
    case 3: return crouch ? MaleCorrFamily::CrouchP2H : MaleCorrFamily::StandP2H;
    case 0: return MaleCorrFamily::Loco;
    default: return MaleCorrFamily::None;
    }
}

// Adds the weight of one pose key to its family, times the aim gate of the key (negative / non-finite weights are ignored).
inline void AddPoseKeyWeight(MaleCorrFamilyWeights& aW, int aKey, float aWeight)
{
    if (!std::isfinite(aWeight) || !(aWeight > 0.0f))
        return;
    const int family = static_cast<int>(MaleCorrFamilyOfKey(aKey));
    if (family >= kMaleCorrFamilyCount)
        return;
    const int aim = aKey >= 0 ? std::min(2, (aKey % 12) / 4) : 2;
    aW.w[family] += aWeight * kMaleCorrAimGate[family][aim];
}

// Bones of the correction (resolved once at Init time) and the children of every corrected bone.
struct MaleBodyCorrectionRig
{
    static constexpr int kMaxChildren = 12;
    int spine3 = -1, clav[2] = {-1, -1}, arm[2] = {-1, -1};
    int childCount[5] = {};         // 0 Spine3, 1 LeftShoulder, 2 RightShoulder, 3 LeftArm, 4 RightArm
    int children[5][kMaxChildren] = {};
    bool valid = false;

    // Bone indices (-1 = missing). Valid only when all five bones exist and no bone has more than kMaxChildren children.
    void Build(const Skeleton& aSk, int aSpine3, int aClavL, int aArmL, int aClavR, int aArmR)
    {
        valid = false;
        spine3 = aSpine3;
        clav[0] = aClavL;
        clav[1] = aClavR;
        arm[0] = aArmL;
        arm[1] = aArmR;
        const int b[5] = {spine3, clav[0], clav[1], arm[0], arm[1]};
        for (int k = 0; k < 5; ++k)
        {
            childCount[k] = 0;
            if (b[k] < 0 || b[k] >= aSk.n)
                return;
            for (int i = 0; i < aSk.n; ++i)
                if (aSk.parent[i] == b[k])
                {
                    if (childCount[k] >= kMaxChildren)
                        return;
                    children[k][childCount[k]++] = i;
                }
        }
        valid = true;
    }
};

// Local rotation D (rotation vector, degrees) -> quaternion; identity for non-finite or ~zero input.
inline math::Quat MaleCorrRotationFromDeg(float aX, float aY, float aZ)
{
    const float a = std::sqrt(aX * aX + aY * aY + aZ * aZ);
    if (!std::isfinite(a) || !(a > 1e-6f))
        return math::Quat{};
    const float half = 0.5f * a * (math::kPi / 180.0f);
    const float k = std::sin(half) / a;
    return math::Quat{aX * k, aY * k, aZ * k, std::cos(half)};
}

// bone.r' = bone.r * D; children c.r' = D^-1 c.r, c.t' = D^-1 c.t (model transforms of everything except the bone itself unchanged).
inline void MaleCorrRotateKeepChildren(Pose& aPose, int aBone, const int* aChildren, int aChildCount, const math::Quat& aD)
{
    aPose.b[aBone].r = math::QuatNormalize(math::QuatMul(aPose.b[aBone].r, aD));
#if defined(UTPP_MALECORR_MUTATION) && UTPP_MALECORR_MUTATION == 1
    (void)aChildren;
    (void)aChildCount; // mutation 1: children not compensated
#else
    const math::Quat inv = math::QuatInv(aD);
    for (int i = 0; i < aChildCount; ++i)
    {
        Xf& c = aPose.b[aChildren[i]];
        c.r = math::QuatNormalize(math::QuatMul(inv, c.r));
        c.t = math::QuatRotate(inv, c.t);
    }
#endif
}

struct MaleCorrectionResult
{
    bool applied = false;
    float spine3Deg = 0.0f;   // magnitude of the applied Spine3 rotation
    float clavDeg[2] = {};    // magnitude of the applied clavicle rotation
    float rollDeg[2] = {};    // signed applied upper-arm roll
};

// aStrength: global scale (config; 0 = off). The family weights are clamped to a total of 1 (a total below 1 scales the correction down:
// the missing share is "no correction"). The pose is untouched (bitwise) when the correction is off, the rig invalid or every rotation ~0.
inline MaleCorrectionResult ApplyMaleBodyCorrection(const Skeleton& aSk, const MaleBodyCorrectionRig& aRig, const MaleCorrFamilyWeights& aWeights,
                                                    float aStrength, Pose& aPose)
{
    MaleCorrectionResult r;
    if (!aRig.valid || !std::isfinite(aStrength) || !(aStrength > 0.0f) || aPose.n < aSk.n)
        return r;
    const float total = aWeights.Sum();
    if (!std::isfinite(total) || !(total > 1e-6f))
        return r;
#if defined(UTPP_MALECORR_MUTATION) && UTPP_MALECORR_MUTATION == 3
    const float norm = 1.0f; // mutation 3: weights not clamped
#else
    const float norm = total > 1.0f ? 1.0f / total : 1.0f;
#endif
    float sp[3] = {}, cl[2][3] = {}, roll[2] = {};
    for (int f = 0; f < kMaleCorrFamilyCount; ++f)
    {
        const float w = aWeights.w[f] * norm * aStrength;
        if (!(w > 0.0f))
            continue;
        const MaleCorrectionSet& s = kMaleCorrectionTable[f];
        for (int i = 0; i < 3; ++i)
        {
            sp[i] += w * s.spine3[i];
            cl[0][i] += w * s.clav[0][i];
            cl[1][i] += w * s.clav[1][i];
        }
        roll[0] += w * s.roll[0];
        roll[1] += w * s.roll[1];
    }
    const float spMag = std::sqrt(sp[0] * sp[0] + sp[1] * sp[1] + sp[2] * sp[2]);
    if (spMag > 1e-6f)
    {
        MaleCorrRotateKeepChildren(aPose, aRig.spine3, aRig.children[0], aRig.childCount[0], MaleCorrRotationFromDeg(sp[0], sp[1], sp[2]));
        r.spine3Deg = spMag;
        r.applied = true;
    }
    for (int side = 0; side < 2; ++side)
    {
        const float m = std::sqrt(cl[side][0] * cl[side][0] + cl[side][1] * cl[side][1] + cl[side][2] * cl[side][2]);
        if (m > 1e-6f)
        {
            MaleCorrRotateKeepChildren(aPose, aRig.clav[side], aRig.children[1 + side], aRig.childCount[1 + side],
                                       MaleCorrRotationFromDeg(cl[side][0], cl[side][1], cl[side][2]));
            r.clavDeg[side] = m;
            r.applied = true;
        }
        if (std::fabs(roll[side]) > 1e-6f)
        {
            MaleCorrRotateKeepChildren(aPose, aRig.arm[side], aRig.children[3 + side], aRig.childCount[3 + side],
                                       MaleCorrRotationFromDeg(roll[side], 0.0f, 0.0f));
            r.rollDeg[side] = roll[side];
            r.applied = true;
        }
    }
    return r;
}
} // namespace utpp::mm::ov

// UTPP motion matching runtime - bound on the clavicle rotation that the mesh-space arm layer produces. Header-only,
// allocation-free, no game types.
//
// Why: ComposeLayers (Overlay.hpp, ALS ABP_LayerBlending) blends the arm regions with LBB_MS when Layering_Arm_X_LS < 1
// (every Pistol1H / Pistol2H / Rifle Aim pose: Arm_X 1, Arm_X_Add 0, Arm_X_LS 0). The arm branch starts at the clavicle
// (LeftShoulder / RightShoulder), so the clavicle keeps the overlay MESH rotation while its parent Spine3 follows the
// locomotion spine (Spine_Add 1). Every deviation of the locomotion Spine3 from the base pose B in mesh space ends up in
// ONE joint: the clavicle local rotation. Recording 2026-10-25 01:24 (pose trace utpp-pose-4750377550594.csv, bone 14):
// Turn_180_R aim-align (hips 80-116 deg off the root) -> LeftShoulder 59 deg from the overlay clavicle at aimW 1, heavy
// landing Jump_B_Land_Stand_Heavy -> 84 deg; calm stand aim 6 deg, run aim 10-20 deg. The verified video frames show the
// shoulder balloon exactly while the clavicle is far off (skin deformation of the shoulder rig).
//
// What: after ComposeLayers, the clavicle local rotation may deviate from the arm-region clavicle (the local-space result:
// overlay clavicle relative to Spine3) by at most kMeshArmClavicleLimitDeg. The excess is removed from the clavicle and
// handed to its children (LeftArm ...): their MESH rotations stay exactly as composed (arm direction, gun aim), only the
// shoulder joint position follows the torso again. Below the limit the pose is untouched (bitwise identical), so calm aim
// poses and the overlay data (female overlay E7E5) keep their current look. Above the limit the angle is not hard-clamped but eased in
// (C1 tanh knee: identity up to kMeshArmClavicleLimitDeg, saturating at limit + kMeshArmClavicleSoftDeg), so engaging / releasing
// the bound never kinks or pops the shoulder from one frame to the next. aSoftDeg = 0 gives the hard clamp to the limit.
// STATUS (2026-10-25: runtime limit now 45 deg, see kMeshArmClavicleRuntimeLimitDeg; earlier: MEASUREMENT ONLY in the runtime (kMeshArmClavicleRuntimeLimitDeg = 180: never engages, the pose is untouched; the deviation is still
// reported in FrameInfo::clavicleLimit and the 'clavLimit=' trace). Reason: the offline survey on the real data (ClavicleLimitTest built with /DUTPP_CLAVLIMIT_SURVEY)
// shows that an absolute bound against the OVERLAY clavicle also hits calm, steady poses (Pistol1H run 50-56 deg, crouch 47-61 deg, Rifle /
// Pistol2H crouch 42-49 deg: the authored stance offset of the spine), which overlaps the balloon frames (59-84 deg). A usable limit needs a
// reference that excludes the steady state (final pose vs typical pose, or a slow baseline) - not decided yet.
// Mutations (UTPP_CLAVLIMIT_MUTATION, tests/ClavicleLimitTest.cpp): 1 = no limit, 2 = children not compensated,
// 3 = no shortest-arc (hemisphere) fix.
#pragma once

#include <cmath>

#include "Overlay.hpp"

namespace utpp::mm::ov
{
// Reference values of the unit tests (NOT the runtime value, see STATUS). Identity up to this deviation: stand idle aim measures 5-6 deg, but run / crouch aim reach 40-60 deg (see STATUS).
inline constexpr float kMeshArmClavicleLimitDeg = 30.0f;
// Width of the soft knee: the limited angle approaches (but never reaches) limit + soft = 45 deg; balloon frames are 59-84 deg.
inline constexpr float kMeshArmClavicleSoftDeg = 15.0f;
// Value the runtime passes (2026-10-25, recording 3: > 40 deg only in slides and heavy landings, clips 456/457/217, max 82 deg; stand/run/crouch aim 0-10 deg,
// ov=0 max 6.7): 45 deg with the 15 deg knee keeps every steady pose untouched and caps the balloon frames at 45 + 15 = 60 deg.
inline constexpr float kMeshArmClavicleRuntimeLimitDeg = 45.0f;

// Output angle for a given deviation: d for d <= L, L + R * tanh((d - L) / R) above (C1 at L, < L + R), hard clamp for R <= 0.
inline float SoftLimitAngleDeg(float aDeg, float aLimitDeg, float aSoftDeg)
{
    if (!(aDeg > aLimitDeg))
        return aDeg;
    if (!(aSoftDeg > 0.0f))
        return aLimitDeg;
    return aLimitDeg + aSoftDeg * std::tanh((aDeg - aLimitDeg) / aSoftDeg);
}

// Limits the local rotation of bone aClav in aPose to aLimitDeg (+ soft knee aSoftDeg) around aReference. Children of aClav keep
// their mesh rotation. Returns the deviation before the limit (deg, 0 when the bone index or any input is invalid); aPose is
// unchanged (bitwise) when the deviation is not above the limit or not finite.
inline float LimitClavicleDeviation(const Skeleton& aSk, const Quat& aReference, int aClav, float aLimitDeg, float aSoftDeg,
                                    Pose& aPose)
{
    if (aClav < 0 || aClav >= aSk.n || !(aLimitDeg >= 0.0f) || !std::isfinite(aSoftDeg) ||
        !math::IsFinite(aReference) || !math::IsFinite(aPose.b[aClav].r))
        return 0.0f;
    const Quat q = math::QuatNormalize(aPose.b[aClav].r);
    const Quat ref = math::QuatNormalize(aReference);
    const Quat relRaw = math::QuatNormalize(math::QuatMul(math::QuatInv(ref), q)); // q = ref * rel
#if defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 3
    const Quat rel = relRaw; // mutation 3: long arc when q and ref sit in opposite hemispheres
#else
    const Quat rel = math::QuatAbs(relRaw);
#endif
    const float s = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
    const float angleDeg = 2.0f * std::atan2(s, rel.w) * (180.0f / math::kPi);
    if (!std::isfinite(angleDeg) || !(angleDeg > aLimitDeg) || !(s > 1e-8f))
        return std::isfinite(angleDeg) ? angleDeg : 0.0f;
#if defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 1
    return angleDeg; // mutation 1: no limit
#else
    const float outDeg = SoftLimitAngleDeg(angleDeg, aLimitDeg, aSoftDeg);
    const float half = 0.5f * outDeg * (math::kPi / 180.0f);
    const float k = std::sin(half) / s;
    const Quat limited{rel.x * k, rel.y * k, rel.z * k, std::cos(half)};
    const Quat qNew = math::QuatNormalize(math::QuatMul(ref, limited));
#if !(defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 2)
    // mesh(child) = mesh(parent of clav) * q * child = mesh(parent of clav) * qNew * child'  ->  child' = qNew^-1 * q * child
    const Quat carry = math::QuatNormalize(math::QuatMul(math::QuatInv(qNew), q));
    for (int i = aClav + 1; i < aSk.n; ++i)
        if (aSk.parent[i] == aClav)
            aPose.b[i].r = math::QuatNormalize(math::QuatMul(carry, aPose.b[i].r));
#endif
    aPose.b[aClav].r = qNew;
    return angleDeg;
#endif
}

struct ClavicleLimitResult
{
    float leftDeg = 0.0f, rightDeg = 0.0f; // deviations before the limit
    bool applied = false;                  // at least one side was limited
};

// Call right after ComposeLayers(aSk, ..., aScratch, aPose) with the same scratch: aScratch.armL / armR hold the arm-region
// poses of that composition (their clavicle local rotation is the local-space reference). Only meaningful when the
// composition had an overlay pose and a base pose (ComposeLayers returns L untouched otherwise and the scratch is stale).
inline ClavicleLimitResult LimitMeshArmClavicles(const Skeleton& aSk, const LayerScratch& aScratch, int aClavL, int aClavR,
                                                 float aLimitDeg, float aSoftDeg, Pose& aPose)
{
    ClavicleLimitResult r;
    if (aClavL >= 0 && aClavL < aSk.n)
    {
        r.leftDeg = LimitClavicleDeviation(aSk, aScratch.armL.b[aClavL].r, aClavL, aLimitDeg, aSoftDeg, aPose);
        r.applied = r.applied || r.leftDeg > aLimitDeg;
    }
    if (aClavR >= 0 && aClavR < aSk.n)
    {
        r.rightDeg = LimitClavicleDeviation(aSk, aScratch.armR.b[aClavR].r, aClavR, aLimitDeg, aSoftDeg, aPose);
        r.applied = r.applied || r.rightDeg > aLimitDeg;
    }
    return r;
}
} // namespace utpp::mm::ov
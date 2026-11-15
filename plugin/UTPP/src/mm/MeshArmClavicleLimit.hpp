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

#include <algorithm>
#include <cmath>

#include "GripIK.hpp"
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

// ---- transient bound: the clavicle held with the chest through a weapon pose crossfade ----------------------------------------------
// Why: at a run / sprint stop with a two-handed weapon the weapon pose crossfades from the moving pose (Rifle Relaxed: arms in local
// space, Layering_Arm_X_LS 1, the shoulders follow the chest) to the standing pose (arms in mesh space). While the stop clip still leans
// and twists the chest, the crossfade puts the chest's deviation from the base pose into the clavicle local rotation (see above), and
// the shoulder joints swing forward / down and together for about 0.2 - 0.5 s until the chest has settled. The absolute limit above
// cannot catch it: the excursion stays below it, and a lower absolute bound would also move the steady authored offsets (40 - 60 deg).
// A free-running slow baseline cannot tell the gait oscillation of a mesh-space run pose from the stop excursion either (it changed
// settled run loops). The bound therefore acts only around the composition's own transition, like the swing gate of the legs layer:
//   - engage: on the first frame of a weapon pose crossfade (the caller's aFading; the runtime passes stop crossfades only: moving ->
//     standing pose of the same weapon pose set) the deviation rel = ref^-1 * q of the frame before (ref = the local-space arm
//     clavicle, q = the composed clavicle) becomes the baseline, and the gate g drops to 0 (a crossfade that starts while the gate is
//     still below 1 keeps baseline and gate); a zero time step changes no state;
//   - bound: while engaged, rel may deviate from the baseline by limitDeg with the C1 tanh knee of SoftLimitAngleDeg (softDeg); the
//     excess is handed to the children of the clavicle, whose mesh rotations stay exactly as composed (arm direction, weapon aim);
//   - release: once the crossfade has ended, g rises by dt / releaseSeconds per frame while rel is back within limitDeg of the
//     baseline (the excursion is over), or after maxHoldSeconds while rel moves slower than settledDegPerSec (the chest has settled
//     on another offset), or after 2 x maxHoldSeconds in any case; the output is the bounded clavicle blended to the composed one by g
//     (children compensated the same way), so the release never pops and does not depend on the frame rate beyond the rate itself;
//   - off: with g = 1 (free) the pose is untouched, bit for bit: steady poses, loops and every frame outside a crossfade and its
//     release are the composition as before. Engaging needs no step either: at engage rel is within limitDeg of the baseline.
// Mutation UTPP_CLAVLIMIT_MUTATION 4 = never engages, 5 = released in one frame when the crossfade ends.
inline constexpr float kClavicleTransientLimitDeg = 4.0f;        // excursion against the baseline passed untouched
inline constexpr float kClavicleTransientSoftDeg = 4.0f;         // knee width: the excursion stays below limit + soft
inline constexpr float kClavicleTransientReleaseSeconds = 0.3f;  // gate rise time once released
inline constexpr float kClavicleTransientSettledDegPerSec = 60.0f; // rel slower than this = the chest has settled
inline constexpr float kClavicleTransientMaxHoldSeconds = 1.5f;  // after the crossfade: release once settled after this, always after 2x

struct ClavicleTransientResult
{
    float leftDeg = 0.0f, rightDeg = 0.0f; // angle of rel against the baseline (0 while free)
    float gate = 1.0f;                     // smaller of both sides: 1 = free (pose untouched), 0 = fully bounded
    bool applied = false;                  // the pose was changed this frame
};

// Sets bone aClav of aPose to aNew and keeps the mesh rotation its direct children had under aOld (the clavicle's rotation before).
inline void SetClavicleKeepChildren(const Skeleton& aSk, int aClav, const Quat& aOld, const Quat& aNew, Pose& aPose)
{
    const Quat carry = math::QuatNormalize(math::QuatMul(math::QuatInv(aNew), aOld));
    for (int i = aClav + 1; i < aSk.n; ++i)
        if (aSk.parent[i] == aClav)
            aPose.b[i].r = math::QuatNormalize(math::QuatMul(carry, aPose.b[i].r));
    aPose.b[aClav].r = aNew;
}

struct ClavicleTransientBound
{
    float limitDeg = kClavicleTransientLimitDeg;
    float softDeg = kClavicleTransientSoftDeg;
    float releaseSeconds = kClavicleTransientReleaseSeconds;
    float settledDegPerSec = kClavicleTransientSettledDegPerSec;
    float maxHoldSeconds = kClavicleTransientMaxHoldSeconds;
    Quat base[2]{}, prevRel[2]{};
    float g[2] = {1.0f, 1.0f};
    float held = 0.0f; // seconds since the crossfade ended while engaged
    bool have[2] = {false, false};
    bool prevFading = false;

    void Reset()
    {
        have[0] = have[1] = false;
        g[0] = g[1] = 1.0f;
        held = 0.0f;
        prevFading = false;
    }

    // Call after LimitMeshArmClavicles with the same scratch (same preconditions). aFading = a weapon pose crossfade is in progress this
    // frame; aDt = frame time in seconds.
    ClavicleTransientResult Apply(const Skeleton& aSk, const LayerScratch& aScratch, int aClavL, int aClavR, bool aFading, float aDt,
                                  Pose& aPose)
    {
        ClavicleTransientResult r;
        const int clav[2] = {aClavL, aClavR};
        const Pose* arm[2] = {&aScratch.armL, &aScratch.armR};
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f;
        const bool step = dt > 0.0f; // a zero time step re-evaluates the frame: the state stays as it is
        const bool engage = step && aFading && !prevFading;
        if (step && !aFading && (g[0] < 1.0f || g[1] < 1.0f))
            held += dt;
        if (engage)
            held = 0.0f;
        for (int s = 0; s < 2; ++s)
        {
            const int c = clav[s];
            if (c < 0 || c >= aSk.n || !math::IsFinite(aPose.b[c].r) || !math::IsFinite(arm[s]->b[c].r))
            {
                have[s] = false;
                g[s] = 1.0f;
                continue;
            }
            const Quat ref = math::QuatNormalize(arm[s]->b[c].r);
            const Quat q = math::QuatNormalize(aPose.b[c].r);
            const Quat rel = math::QuatAbs(math::QuatNormalize(math::QuatMul(math::QuatInv(ref), q)));
            const float speed = have[s] && dt > 0.0f ? AngleDeg(prevRel[s], rel) / dt : 0.0f;
            (void)speed;
            (void)engage;
#if !(defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 4)
            if (engage && !(g[s] < 1.0f)) // a crossfade that starts while still held keeps its baseline and gate (no step)
            {
                base[s] = have[s] ? prevRel[s] : rel; // the last frame before the crossfade (this frame when there is none)
                g[s] = 0.0f;
            }
#endif
            if (step && !aFading && g[s] < 1.0f)
            {
#if defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 5
                g[s] = 1.0f; // mutation 5: released in one frame
#else
                const bool back = !(AngleDeg(base[s], rel) > limitDeg); // the excursion is over: the bound no longer acts
                if (back || (held >= maxHoldSeconds && speed < settledDegPerSec) || held >= 2.0f * maxHoldSeconds)
                    g[s] = releaseSeconds > 0.0f ? std::min(1.0f, g[s] + dt / releaseSeconds) : 1.0f;
#endif
            }
            if (step || !have[s])
            {
                prevRel[s] = rel;
                have[s] = true;
            }
            r.gate = std::min(r.gate, g[s]);
            if (!(g[s] < 1.0f))
                continue; // free: untouched
            // bounded clavicle: rel limited against the baseline
            const Quat d = math::QuatAbs(math::QuatNormalize(math::QuatMul(math::QuatInv(base[s]), rel)));
            const float sn = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            const float angleDeg = 2.0f * std::atan2(sn, d.w) * (180.0f / math::kPi);
            (s ? r.rightDeg : r.leftDeg) = angleDeg;
            if (!std::isfinite(angleDeg) || !(angleDeg > limitDeg) || !(sn > 1e-8f))
                continue; // within the limit: untouched
            const float outDeg = SoftLimitAngleDeg(angleDeg, limitDeg, softDeg);
            const float half = 0.5f * outDeg * (math::kPi / 180.0f);
            const float k = std::sin(half) / sn;
            const Quat bounded = math::QuatNormalize(math::QuatMul(ref, math::QuatMul(base[s], Quat{d.x * k, d.y * k, d.z * k, std::cos(half)})));
            const Quat out = g[s] > 0.0f ? Nlerp(bounded, q, g[s]) : bounded;
            SetClavicleKeepChildren(aSk, c, q, out, aPose);
            r.applied = true;
        }
        if (step)
            prevFading = aFading;
        return r;
    }
};

// ---- girdle width bound: the shoulders of a mesh-space arm pose kept near the body's own width -------------------------------------
// Why: under a chest that differs from the weapon pose's base pose (crouch: the motion clip's chest is about 23 deg off the crouch base
// pose) the mesh-space arm branch turns the clavicles with that difference (see the top of this file). With the two-handed pistol aim
// in the crouch it swings the right shoulder joint forward and in (Spine3 frame (16.6, -19.6, 15.3) instead of about (17.3, -13.9, 18.3)
// in the pose's own geometry): the shoulders are 3.5 cm narrower than the body's own girdle, steadily, on both bodies. Moving the
// clavicles back to their local-space reference does not work: the left joint would move in, and the right one 5.6 cm away from the
// hand, past the reach of the arm.
// What: own width = lateral distance of the two upper arm joints with each clavicle at its rotation in aOwn (the locomotion pose: the
// body's own girdle) under the composed chest; deficit = own width - composed width. While the deficit is not above limitM the pose is
// untouched, bit for bit. Above it, the missing width (deficit - limitM) is shared by the sides in proportion to their own lateral
// deficits; each side's clavicle turns its upper arm joint outward (shortest arc of the clavicle vector toward the lateral axis, never
// past the side's own lateral distance); a share one side cannot take goes to the other side. The two-handed aim arm is already near
// its full reach, so an outward joint would leave it: both hands then shift together by the same vector (the weapon hand first, then
// the other hand), just enough to keep each within the soft reach of its arm (aParams.softStart x arm length). The weapon follows the
// right hand, so it translates by that shift and keeps its rotation, and the left hand keeps its place on it (aTwoHanded; otherwise
// each hand shifts only as far as its own arm needs). The shift is bounded by
// aMaxShiftM: where the full share needs more, the largest share that fits is taken (continuous in the pose). The arms are solved
// (gik::SolveArm) to the shifted hand positions with the composed hand rotations; only the clavicles, upper arms and forearms turn.
// The joints only move outward: the width never falls. aShare (0..1) scales the missing width: the runtime passes the share of the
// standing (not moving) weapon poses, since a moving aim pose keeps its arms at the overlay's mesh rotations; 0 = untouched.
// Mutation UTPP_CLAVLIMIT_MUTATION 6 = never acts, 7 = the arm is not solved back (the hand moves with the shoulder joint).
inline constexpr float kGirdleDeficitLimitM = 0.015f;  // largest deficit against the own width left untouched / kept
inline constexpr float kGirdleMaxHandShiftM = 0.020f;  // largest shared shift of the hands and the weapon

struct GirdleResult
{
    float ownWidth = 0.0f, widthBefore = 0.0f, widthAfter = 0.0f; // m, lateral in the Spine3 frame
    float handErr[2] = {0.0f, 0.0f};                              // |hand - shifted hand target| after the solve (m)
    float handShift = 0.0f;                                       // shared shift of both hands and the weapon (m)
    bool capped[2] = {false, false};                              // the shift bound left part of the missing width
    bool applied = false;
};

namespace girdle_detail
{
struct Side
{
    Vec3 origin, vec;      // clavicle joint and clavicle -> upper arm joint, Spine3 frame (m)
    Vec3 hand;             // composed hand position, Spine3 frame
    float reach = 0.0f;    // softStart x arm length
    float own = 0.0f;      // own lateral distance |z|
    float sign = 1.0f;     // +1 right (positive lateral), -1 left
    Vec3 axis;             // rotation axis of the outward turn (unit), zero when no turn is possible
    float maxAngle = 0.0f; // angle of vec to the lateral direction
};
inline Vec3 Rot(Vec3 v, Vec3 a, float ang) // Rodrigues, a unit and perpendicular to nothing in particular
{
    const float c = std::cos(ang), s = std::sin(ang);
    return v * c + math::Cross(a, v) * s + a * (math::Dot(a, v) * (1.0f - c));
}
inline Vec3 JointAt(const Side& aS, float aAng) { return aS.origin + Rot(aS.vec, aS.axis, aAng); }
// largest angle in [0, aMaxAng] with lateral gain <= aGain and the joint within reach (lateral and reach are monotone along the arc
// toward the lateral axis for these poses; bisection keeps the result on the safe side of both)
inline float AngleFor(const Side& aS, float aGain, float& aGot, bool& aCapped)
{
    const float z0 = aS.sign * aS.vec.z;
    auto ok = [&](float a) {
        const Vec3 j = JointAt(aS, a);
        const float zg = aS.sign * (j.z - aS.origin.z) - z0;
        const Vec3 d = aS.hand - j;
        return zg <= aGain && std::sqrt(math::Dot(d, d)) <= aS.reach;
    };
    float lo = 0.0f, hi = aS.maxAngle;
    if (ok(hi))
        lo = hi;
    else
        for (int i = 0; i < 24; ++i)
        {
            const float m = 0.5f * (lo + hi);
            (ok(m) ? lo : hi) = m;
        }
    const Vec3 j = JointAt(aS, lo);
    aGot = aS.sign * (j.z - aS.origin.z) - z0;
    const Vec3 d = aS.hand - JointAt(aS, std::min(aS.maxAngle, lo + 1e-3f));
    aCapped = aGot < aGain - 1e-4f && std::sqrt(math::Dot(d, d)) > aS.reach;
    return lo;
}
} // namespace girdle_detail

// Call after the clavicle limits above, before any pass that reads the hands (feminine shoulders, grip, aim offset). aOwn = the
// locomotion pose of the composition (its clavicle local rotations are the body's own girdle); aClavL / aClavR = LeftShoulder /
// RightShoulder, children of aSpine3, with the upper arms of aRig as their children.
inline GirdleResult BoundGirdleWidth(const Skeleton& aSk, const Pose& aOwn, const gik::Rig& aRig, int aSpine3, int aClavL, int aClavR,
                                     float aLimitM, float aMaxShiftM, bool aTwoHanded, float aShare, const gik::Params& aParams,
                                     Pose& aPose)
{
    GirdleResult r;
    const int clav[2] = {aClavL, aClavR};
    const gik::ArmChain* chain[2] = {&aRig.left, &aRig.right};
    if (aSpine3 < 0 || aSpine3 >= aSk.n || !(aLimitM >= 0.0f) || !aRig.Valid(aSk))
        return r;
    for (int s = 0; s < 2; ++s)
        if (clav[s] < 0 || clav[s] >= aSk.n || aSk.parent[clav[s]] != aSpine3 || aSk.parent[chain[s]->upper] != clav[s] ||
            !math::IsFinite(aOwn.b[clav[s]].r) || !math::IsFinite(aPose.b[clav[s]].r))
            return r;
    Quat sRot, hRot[2];
    Vec3 sPos, hPos[2];
    gik::ModelOf(aSk, aPose, aSpine3, sRot, sPos);
    const Quat sInv = math::QuatInv(sRot);
    girdle_detail::Side side[2];
    gik::Params solve[2] = {aParams, aParams}; // per side: an arm the composition already stretched past softStart keeps its reach
    float zNow[2], zOwn[2];
    for (int s = 0; s < 2; ++s)
    {
        girdle_detail::Side& d = side[s];
        const Xf& c = aPose.b[clav[s]];
        const Vec3 armT = aPose.b[chain[s]->upper].t;
        d.origin = c.t;
        d.vec = math::QuatRotate(math::QuatNormalize(c.r), armT);
        const Vec3 ownVec = math::QuatRotate(math::QuatNormalize(aOwn.b[clav[s]].r), armT);
        d.sign = s ? 1.0f : -1.0f;
        zNow[s] = d.sign * (d.origin.z + d.vec.z);
        zOwn[s] = d.sign * (d.origin.z + ownVec.z);
        gik::ModelOf(aSk, aPose, chain[s]->hand, hRot[s], hPos[s]);
        d.hand = math::QuatRotate(sInv, hPos[s] - sPos);
        const float full = gik::Len(aPose.b[chain[s]->lower].t) + gik::Len(aPose.b[chain[s]->hand].t);
        const Vec3 toHand = d.hand - (d.origin + d.vec);
        const float now = full > 1e-6f ? std::sqrt(math::Dot(toHand, toHand)) / full : 0.0f;
        if (now > aParams.softStart)
        {
            solve[s].softStart = std::min(now, 0.9999f);
            solve[s].maxReach = std::max(aParams.maxReach, std::min(1.0f, solve[s].softStart + 1e-4f));
        }
        d.reach = solve[s].softStart * full;
        d.own = zOwn[s];
    }
    r.widthBefore = zNow[0] + zNow[1];
    r.ownWidth = zOwn[0] + zOwn[1];
    r.widthAfter = r.widthBefore;
    const float deficit = r.ownWidth - r.widthBefore;
    const float share = std::isfinite(aShare) ? std::clamp(aShare, 0.0f, 1.0f) : 0.0f;
    if (!std::isfinite(deficit) || !(deficit > aLimitM) || !(share > 0.0f))
        return r;
#if defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 6
    (void)aMaxShiftM, (void)aTwoHanded;
    return r; // mutation 6: never acts
#else
    const Vec3 lat{0.0f, 0.0f, 1.0f};
    float need[2], gain[2] = {0.0f, 0.0f}, ang[2] = {0.0f, 0.0f};
    for (int s = 0; s < 2; ++s)
    {
        girdle_detail::Side& d = side[s];
        need[s] = std::max(0.0f, zOwn[s] - zNow[s]);
        Vec3 u;
        const Vec3 to = lat * d.sign;
        if (gik::Norm(d.vec, u) && gik::Norm(math::Cross(d.vec, to), d.axis, 1e-6f))
            d.maxAngle = std::acos(std::clamp(math::Dot(u, to), -1.0f, 1.0f));
        else
        {
            d.axis = Vec3{};
            d.maxAngle = 0.0f;
        }
    }
    const float needSum = need[0] + need[1];
    if (!(needSum > 1e-6f))
        return r;
    girdle_detail::Side free[2] = {side[0], side[1]};
    free[0].reach = free[1].reach = 1e9f; // the turn itself ignores the reach: the hands shift instead (below)
    // plan for the share aFrac of the missing width: turn angles, lateral gains and the shared hand shift (Spine3 frame) that keeps
    // both hands within the soft reach of their arms
    auto plan = [&](float aFrac, float* aAng, float* aGain, Vec3* aShift) {
        const float missing = (deficit - aLimitM) * share * aFrac;
        aGain[0] = aGain[1] = aAng[0] = aAng[1] = 0.0f;
        // first pass: shares in proportion to the side deficits (never past the own lateral distance); second: the rest to the other side
        for (int pass = 0; pass < 2; ++pass)
        {
            if (!(missing - aGain[0] - aGain[1] > 1e-5f))
                break;
            for (int s = 0; s < 2; ++s)
            {
                if (free[s].maxAngle <= 0.0f)
                    continue;
                const float part = pass == 0 ? missing * need[s] / needSum : missing - aGain[0] - aGain[1];
                const float want = std::min(need[s], aGain[s] + part);
                if (!(want > aGain[s]))
                    continue;
                bool capped = false;
                float got = 0.0f;
                aAng[s] = girdle_detail::AngleFor(free[s], want, got, capped);
                aGain[s] = got;
            }
        }
        // two-handed: one shift for both hands, the weapon hand first, then the other hand with the shift so far; one-handed: each
        // hand its own
        Vec3 shared{};
        for (int s : {1, 0})
        {
            const Vec3 j = girdle_detail::JointAt(side[s], aAng[s]);
            const Vec3 h = side[s].hand + (aTwoHanded ? shared : Vec3{});
            const Vec3 d = h - j;
            const float len = std::sqrt(math::Dot(d, d));
            const Vec3 own = len > side[s].reach && len > 1e-6f ? d * -(1.0f - side[s].reach / len) : Vec3{};
            shared = shared + own;
            aShift[s] = own;
        }
        if (aTwoHanded)
            aShift[0] = aShift[1] = shared;
        return std::sqrt(std::max(math::Dot(aShift[0], aShift[0]), math::Dot(aShift[1], aShift[1])));
    };
    // the largest share whose hand shift stays within aMaxShiftM (the full share when it fits)
    Vec3 shift[2];
    float frac = 1.0f;
    if (plan(1.0f, ang, gain, shift) > aMaxShiftM)
    {
        float lo = 0.0f, hi = 1.0f;
        for (int i = 0; i < 24; ++i)
        {
            const float m = 0.5f * (lo + hi);
            (plan(m, ang, gain, shift) <= aMaxShiftM ? lo : hi) = m;
        }
        frac = lo;
        plan(frac, ang, gain, shift);
        r.capped[0] = r.capped[1] = true;
    }
    if (!(ang[0] > 0.0f) && !(ang[1] > 0.0f))
        return r;
    for (int s = 0; s < 2; ++s)
    {
        const Vec3 shiftMesh = math::QuatRotate(sRot, shift[s]);
        const float shiftLen = std::sqrt(math::Dot(shift[s], shift[s]));
        r.handShift = std::max(r.handShift, shiftLen);
        if (!(ang[s] > 0.0f) && !(shiftLen > 0.0f))
            continue;
        const int c = clav[s];
        if (ang[s] > 0.0f)
        {
            const Quat turn = math::QuatNormalize(gik::AxisAngle(side[s].axis, ang[s])); // Spine3 frame = parent frame of the clavicle
            const Quat q = math::QuatNormalize(aPose.b[c].r);
            SetClavicleKeepChildren(aSk, c, q, math::QuatNormalize(math::QuatMul(turn, q)), aPose);
        }
#if !(defined(UTPP_CLAVLIMIT_MUTATION) && UTPP_CLAVLIMIT_MUTATION == 7)
        gik::Result ik;
        gik::SolveArm(aSk, aPose, *chain[s], hPos[s] + shiftMesh, &hRot[s], 1.0f, solve[s], ik);
        r.handErr[s] = ik.handErr;
#endif
        r.applied = true;
    }
    r.widthAfter = r.widthBefore + gain[0] + gain[1];
    return r;
#endif
}
} // namespace utpp::mm::ov

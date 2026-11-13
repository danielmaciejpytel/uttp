// UTPP 0.6.3 - third-person aim offset core (design 29.3 item 4 / 29.6 step 8). Header-only, no RED4ext / Windows
// dependency: everything here is unit-tested offline (tests/AimOffsetTest.cpp). Runtime glue: mm/AimOffset.cpp.
//
// Problem: while aiming in TPP the upper body (Spine1 and above, arms, hands, WeaponLeft / WeaponRight) comes from the game's
// FPP graph through the 0.6.2 upper-body mask. That pose is authored for the FPP camera, so the gun does not point at the
// screen centre of the TPP camera (shots and the crosshair already use the active camera - confirmed in game).
//
// Post-process on the final local pose (after BlendIntoRecords, before the game's FK), all in MODEL space (FK root = the
// player entity; the target point is converted world -> model on the game thread):
//   1. FK of bones [0, fkCount) from the 48-byte local records (pos @0, quat xyzw @0x10; scale ignored = assumed 1);
//   2. weapon = WeaponRight model transform; barrel = weapon rotation * barrel axis (default +Y, see kDefaultBarrelAxis);
//      muzzle = weapon position + barrel * muzzleOffset; error = yaw / pitch from the barrel to (target - muzzle);
//   3. level (a): the clamped (maxYaw / maxPitch) and smoothed (exponential, rate smoothHz) yaw / pitch delta is applied as a
//      model-space rotation about RightArm for one-handed poses, or RightHand for two-handed poses (preserves left-hand reach);
//      Q = Rz(dYaw) * R(right(barrel), dPitch);
//   4. level (b): residual = error left after (a) minus the error intentionally left (unclamped part / smoothing lag), clamped
//      to maxResidual, applied about the right wrist (RightHand; WeaponRight is its child, the gun stays in the hand);
//   5. left hand: if it is within twoHandMaxDist of the weapon bone in the source pose (two-handed grip) it is moved rigidly
//      with the weapon (same weapon-relative transform) by a two-bone IK on LeftArm / LeftForeArm (elbow bend + swing) and
//      the hand rotation is set; one-handed poses leave the left arm untouched.
// The correction is open loop per frame: every FK call starts from the fresh game pose, so the output never feeds back.
//
// Yaw / pitch convention (GameIntegration.hpp): Z up, forward +Y; yaw = atan2(-x, y) (CCW about +Z), pitch = asin(z) (up > 0).
// Barrel axis: rig bind pose (player_woman_skeleton.rig.json, FK of boneTransforms): WeaponRight local
// +Y lies along the arm / index finger (model +X in the T-pose), +Z = model +Y (top of a palm-down grip), +X = model +Z.
// REDengine items use +Y forward / +Z up, attached to the slot bone, so the barrel is assumed to be WeaponRight local +Y.
// UNVERIFIED in engine (axis exposed: UTPPSetAimOffsetAxis).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

#include "GameIntegration.hpp"
#include "MMMath.hpp"

namespace utpp::mm::aim
{
using math::Quat;
using math::Vec3;

inline constexpr uint32_t kMaxFkBones = 80;
inline constexpr float kDegF = 0.0174532925199432958f;
inline constexpr int32_t kDefaultBarrelAxis = 2; // +Y (1 = X, 2 = Y, 3 = Z, negative = opposite direction)

// ------------------------------------------------------------------------------------------------ vector helpers

inline float Length(Vec3 v) { return std::sqrt(math::Dot(v, v)); }

inline bool NormalizeTo(Vec3 v, Vec3& aOut)
{
    const float n = Length(v);
    if (!std::isfinite(n) || !(n > 1e-6f))
        return false;
    aOut = v * (1.0f / n);
    return true;
}

inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline Quat AxisAngle(Vec3 aUnitAxis, float aAngle)
{
    const float s = std::sin(0.5f * aAngle);
    return Quat{aUnitAxis.x * s, aUnitAxis.y * s, aUnitAxis.z * s, std::cos(0.5f * aAngle)};
}

inline bool ValidAxis(int32_t aAxis) { return aAxis != 0 && aAxis >= -3 && aAxis <= 3; }

inline Vec3 AxisVector(int32_t aAxis)
{
    const float s = aAxis < 0 ? -1.0f : 1.0f;
    switch (aAxis < 0 ? -aAxis : aAxis)
    {
    case 1: return Vec3{s, 0.0f, 0.0f};
    case 3: return Vec3{0.0f, 0.0f, s};
    default: return Vec3{0.0f, s, 0.0f};
    }
}

// Angle (radians) between two directions; pi for degenerate input.
inline float AngleBetween(Vec3 a, Vec3 b)
{
    Vec3 na, nb;
    if (!NormalizeTo(a, na) || !NormalizeTo(b, nb))
        return math::kPi;
    return std::atan2(Length(math::Cross(na, nb)), math::Dot(na, nb)); // accurate near 0 (acos is not)
}

// ------------------------------------------------------------------------------------------------ transforms

struct Xf
{
    Vec3 p;
    Quat q;
};

inline Xf Mul(const Xf& aParent, const Xf& aLocal)
{
    return Xf{aParent.p + math::QuatRotate(aParent.q, aLocal.p), math::QuatNormalize(math::QuatMul(aParent.q, aLocal.q))};
}

// World point -> model space of an entity at aPos with rotation aRot (unit quaternion xyzw).
inline Vec3 WorldToModel(Vec3 aPos, const Quat& aRot, Vec3 aWorld)
{
    return math::QuatRotate(math::QuatInv(math::QuatNormalize(aRot)), aWorld - aPos);
}

// ------------------------------------------------------------------------------------------------ pose records

inline Xf ReadLocal(const uint8_t* aBones, uint32_t aIndex)
{
    const float* r = reinterpret_cast<const float*>(aBones + size_t(aIndex) * integ::kGameBoneStride);
    return Xf{Vec3{r[0], r[1], r[2]}, Quat{r[4], r[5], r[6], r[7]}};
}

// Writes a local rotation, sign-aligned with the previous record (no 4D flip for blends downstream).
inline void WriteLocalRot(uint8_t* aBones, uint32_t aIndex, Quat aQ)
{
    float* r = reinterpret_cast<float*>(aBones + size_t(aIndex) * integ::kGameBoneStride) + integ::kGameRotOffsetFloats;
    if (r[0] * aQ.x + r[1] * aQ.y + r[2] * aQ.z + r[3] * aQ.w < 0.0f)
        aQ = math::QuatNeg(aQ);
    r[0] = aQ.x;
    r[1] = aQ.y;
    r[2] = aQ.z;
    r[3] = aQ.w;
}

// Model transforms of bones [0, aN). Requires aN <= aCount, aN <= kMaxFkBones, parents[i] < i (topological order, as in the
// player rig) and finite records with a non-degenerate rotation. Returns false (nothing usable) otherwise.
inline bool ComputeModel(const uint8_t* aBones, uint32_t aCount, const int16_t* aParents, uint32_t aN, Xf* aOut)
{
    if (!aBones || !aParents || !aOut || aN == 0 || aN > aCount || aN > kMaxFkBones)
        return false;
    for (uint32_t i = 0; i < aN; ++i)
    {
        Xf l = ReadLocal(aBones, i);
        if (!math::IsFinite(l.p) || !math::IsFinite(l.q))
            return false;
        const float n2 = math::QuatDot(l.q, l.q);
        if (!(n2 > 0.25f) || !(n2 < 4.0f))
            return false;
        l.q = math::QuatNormalize(l.q);
        const int32_t par = aParents[i];
        if (par < 0)
            aOut[i] = l;
        else if (static_cast<uint32_t>(par) >= i)
            return false;
        else
            aOut[i] = Mul(aOut[par], l);
    }
    return true;
}

// Applies the model-space rotation aQ about the joint of aBone (its position does not move, its subtree follows):
// L' = inv(M_parent.q) * aQ * M_parent.q * L. aModel must be current for the parent.
inline void RotateAboutJoint(uint8_t* aBones, const Xf* aModel, const int16_t* aParents, uint32_t aBone, const Quat& aQ)
{
    const int32_t par = aParents[aBone];
    const Quat pq = par >= 0 ? aModel[par].q : Quat{};
    const Quat l = math::QuatNormalize(ReadLocal(aBones, aBone).q);
    const Quat nl = math::QuatNormalize(math::QuatMul(math::QuatMul(math::QuatInv(pq), math::QuatMul(aQ, pq)), l));
    WriteLocalRot(aBones, aBone, nl);
}

// Sets the MODEL rotation of aBone (parent model current).
inline void SetModelRot(uint8_t* aBones, const Xf* aModel, const int16_t* aParents, uint32_t aBone, const Quat& aModelRot)
{
    const int32_t par = aParents[aBone];
    const Quat pq = par >= 0 ? aModel[par].q : Quat{};
    WriteLocalRot(aBones, aBone, math::QuatNormalize(math::QuatMul(math::QuatInv(pq), aModelRot)));
}

// ------------------------------------------------------------------------------------------------ yaw / pitch

inline bool YawPitchOf(Vec3 aDir, float& aYaw, float& aPitch)
{
    Vec3 d;
    if (!NormalizeTo(aDir, d) || (d.x * d.x + d.y * d.y) < 1e-8f)
        return false;
    aYaw = std::atan2(-d.x, d.y);
    aPitch = std::asin(Clampf(d.z, -1.0f, 1.0f));
    return true;
}

// Yaw / pitch (radians) that turn aFrom into aTo (yaw wrapped to (-pi, pi]).
inline bool YawPitchDelta(Vec3 aFrom, Vec3 aTo, float& aDYaw, float& aDPitch)
{
    float y0, p0, y1, p1;
    if (!YawPitchOf(aFrom, y0, p0) || !YawPitchOf(aTo, y1, p1))
        return false;
    aDYaw = integ::WrapPi(y1 - y0);
    aDPitch = p1 - p0;
    return true;
}

// Rotation changing the yaw of aFwd by aDYaw and its pitch by aDPitch: Rz(aDYaw) * R(right, aDPitch), right = fwd x Z.
// (Exact while |pitch + aDPitch| < 90 deg.) False for a vertical aFwd.
inline bool YawPitchRotation(Vec3 aFwd, float aDYaw, float aDPitch, Quat& aOut)
{
    Vec3 f, r;
    if (!NormalizeTo(aFwd, f) || !NormalizeTo(math::Cross(f, Vec3{0.0f, 0.0f, 1.0f}), r))
        return false;
    aOut = math::QuatNormalize(math::QuatMul(AxisAngle(Vec3{0.0f, 0.0f, 1.0f}, aDYaw), AxisAngle(r, aDPitch)));
    return true;
}

// ------------------------------------------------------------------------------------------------ rig

struct Rig
{
    int32_t rightArm = -1;     // level (a) pivot (right upper arm, "RightArm")
    int32_t rightHand = -1;    // level (b) pivot ("RightHand")
    int32_t weapon = -1;       // "WeaponRight"
    int32_t leftArm = -1;      // left two-bone IK root ("LeftArm")
    int32_t leftForeArm = -1;  // left elbow ("LeftForeArm")
    int32_t leftHand = -1;     // left end effector ("LeftHand")
    uint32_t fkCount = 0;      // bones [0, fkCount) are FK'd (max index used + 1)
    uint32_t ok = 0;           // 1 = every check passed (fail closed: 0 -> never applied)
    int16_t parents[kMaxFkBones] = {};
};
static_assert(sizeof(Rig) % 4 == 0);

inline bool IsAncestor(const int16_t* aParents, uint32_t aN, int32_t aAncestor, int32_t aBone)
{
    for (int32_t b = aBone, guard = 0; b >= 0 && static_cast<uint32_t>(b) < aN && guard < int32_t(kMaxFkBones); ++guard)
    {
        b = aParents[b];
        if (b == aAncestor)
            return true;
    }
    return false;
}

// Resolves the bones by name (game CName hash) against the rig name / parent tables. Fail closed: ok = 0 when a name is
// missing, an index is >= kMaxFkBones, the parents of [0, fkCount) are not topological, or the chains are not
// RightArm -> ... -> RightHand -> ... -> WeaponRight and LeftArm -> LeftForeArm -> LeftHand (left chain not under RightArm).
inline Rig ResolveRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    Rig r;
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return r;
    auto find = [&](std::string_view aName) -> int32_t {
        const uint64_t h = integ::CNameHash(aName);
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == h)
                return static_cast<int32_t>(i);
        return -1;
    };
    r.rightArm = find("RightArm");
    r.rightHand = find("RightHand");
    r.weapon = find("WeaponRight");
    r.leftArm = find("LeftArm");
    r.leftForeArm = find("LeftForeArm");
    r.leftHand = find("LeftHand");
    const int32_t idx[] = {r.rightArm, r.rightHand, r.weapon, r.leftArm, r.leftForeArm, r.leftHand};
    int32_t maxIdx = -1;
    for (const int32_t i : idx)
    {
        if (i < 0 || i >= int32_t(kMaxFkBones))
            return Rig{};
        maxIdx = i > maxIdx ? i : maxIdx;
    }
    r.fkCount = static_cast<uint32_t>(maxIdx + 1);
    for (uint32_t i = 0; i < r.fkCount; ++i)
    {
        r.parents[i] = aParents[i];
        if (aParents[i] >= int32_t(i))
            return Rig{};
    }
    const bool chains = IsAncestor(r.parents, r.fkCount, r.rightHand, r.weapon) &&
                        IsAncestor(r.parents, r.fkCount, r.rightArm, r.rightHand) &&
                        r.parents[r.leftHand] == r.leftForeArm && r.parents[r.leftForeArm] == r.leftArm &&
                        !IsAncestor(r.parents, r.fkCount, r.rightArm, r.leftArm) &&
                        !IsAncestor(r.parents, r.fkCount, r.leftArm, r.rightArm);
    if (!chains)
        return Rig{};
    r.ok = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ guards

enum Guard : uint32_t
{
    kGuardNoTarget = 1u << 0,       // UTPPSetAimTarget(..., false) or never called
    kGuardTargetStale = 1u << 1,    // last target older than kMaxTargetAge
    kGuardTargetInvalid = 1u << 2,  // player transform unavailable / non-finite target
    kGuardFeedStale = 1u << 3,      // no locomotion feed for kMaxTargetAge (PSM state unknown)
    kGuardTppOff = 1u << 4,         // feed: TPP not active
    kGuardNotAim = 1u << 5,         // feed: PSM.UpperBody != Aim (6)
    kGuardMaskStateOff = 1u << 6,   // UTPPSetMMUpperMaskStates without bit 2 (aim)
    kGuardMelee = 1u << 7,          // feed: melee bits now or within kMeleeHold
    kGuardVehicle = 1u << 8,        // feed: PSM vehicle bit
    kGuardPsmBlock = 1u << 9,       // feed: any other PSM block reason (scene, swim, workspot, takedown...)
    kGuardMaskInactive = 1u << 10,  // published snapshot: upper-body mask not active (upperScale >= 1) or MM weight 0
    kGuardRig = 1u << 11,           // rig not resolved / FK rig differs / bone count too small
    kGuardBusy = 1u << 12,          // another thread is inside the aim offset (skipped, no reset)
    kGuardSolve = 1u << 13,         // FK / solve failure (non-finite record, vertical barrel...)
    kGuardDisabled = 1u << 14,      // kill switch (UTPPSetAimOffsetParams with maxYaw = maxPitch = 0)
};
inline constexpr float kMaxTargetAge = 0.25f; // s
inline constexpr float kMeleeHold = 0.3f;     // s after the melee bits clear (= integ::kMeleeReleaseHoldShortSeconds; deliberately NOT the 0.55 s
                                              // melee weapon hold: a gun aimed right after a fight gets its aim offset back at once)

struct GuardInput
{
    bool targetEnabled = false;
    bool targetValid = false;
    float targetAge = 1e9f;   // s since the last UTPPSetAimTarget
    float feedAge = 1e9f;     // s since the last UTPPFeedLocomotion
    bool tpp = false;         // last feed
    uint32_t psm = 0;         // last feed raw UTPPPsmBlockMask (diagnostic fields included)
    int32_t maskStates = int32_t(integ::kMaskStatesAll);
    float sinceMelee = -1.0f; // s since melee bits were last seen (< 0 = never)
    float upperScale = 1.0f;  // published snapshot (1 = no mask)
    float mmWeight = 0.0f;    // published snapshot weight
    bool rigOk = false;
};

inline uint32_t ComputeGuard(const GuardInput& g)
{
    uint32_t r = 0;
    if (!g.targetEnabled)
        r |= kGuardNoTarget;
    else if (!g.targetValid)
        r |= kGuardTargetInvalid;
    if (!(g.targetAge >= 0.0f && g.targetAge <= kMaxTargetAge))
        r |= kGuardTargetStale;
    if (!(g.feedAge >= 0.0f && g.feedAge <= kMaxTargetAge))
        r |= kGuardFeedStale;
    if (!g.tpp)
        r |= kGuardTppOff;
    if (integ::PsmUpperBodyOf(g.psm) != integ::kPsmUpperBodyAim)
        r |= kGuardNotAim;
    if ((g.maskStates & int32_t(integ::kMaskStateAim)) == 0)
        r |= kGuardMaskStateOff;
    if (integ::PsmMeleeBits(g.psm) != 0 || (g.sinceMelee >= 0.0f && g.sinceMelee < kMeleeHold))
        r |= kGuardMelee;
    const uint32_t block = integ::PsmBlockBits(g.psm);
    if (block & integ::kPsmVehicle)
        r |= kGuardVehicle;
    if (block & ~uint32_t(integ::kPsmVehicle | integ::kPsmDetailed))
        r |= kGuardPsmBlock;
    if (!(g.mmWeight > 0.0f) || !std::isfinite(g.upperScale) || !(g.upperScale < 0.999f))
        r |= kGuardMaskInactive;
    if (!g.rigOk)
        r |= kGuardRig;
    return r;
}

// Share of the upper body that is on the game pose (the pose the offset corrects): 1 - w * upperScale, in [0, 1].
inline float GamePoseShare(float aMMWeight, float aUpperScale)
{
    const float w = std::isfinite(aMMWeight) ? Clampf(aMMWeight, 0.0f, 1.0f) : 0.0f;
    const float us = std::isfinite(aUpperScale) ? Clampf(aUpperScale, 0.0f, 1.0f) : 1.0f;
    return Clampf(1.0f - w * us, 0.0f, 1.0f);
}

// ------------------------------------------------------------------------------------------------ solver

struct Params
{
    float maxYawDeg = 30.0f;      // level (a) clamp
    float maxPitchDeg = 25.0f;
    float smoothHz = 12.0f;       // exponential rate (1/s): alpha = 1 - exp(-smoothHz * dt), time constant 1/12 = 0.083 s; 0 = no smoothing
    float maxResidualDeg = 5.0f;  // level (b) clamp (wrist)
    float muzzleOffset = 0.25f;   // m along the barrel from the weapon bone
    float targetDistance = 60.0f; // m along the camera forward (applied by the caller that builds the target point)
    float twoHandMaxDist = 0.5f;  // m: left hand closer than this to the weapon bone in the source pose = two-handed grip
    int32_t barrelAxis = kDefaultBarrelAxis;
};

inline bool ValidParams(float aMaxYawDeg, float aMaxPitchDeg, float aSmoothHz)
{
    return std::isfinite(aMaxYawDeg) && std::isfinite(aMaxPitchDeg) && std::isfinite(aSmoothHz) && aMaxYawDeg >= 0.0f &&
           aMaxYawDeg <= 60.0f && aMaxPitchDeg >= 0.0f && aMaxPitchDeg <= 60.0f && aSmoothHz >= 0.0f && aSmoothHz <= 120.0f;
}

struct Result
{
    float errYaw = 0.0f, errPitch = 0.0f;   // source pose error (rad)
    float cmdYaw = 0.0f, cmdPitch = 0.0f;   // applied at the shoulder (rad, after smoothing and weight)
    float resYaw = 0.0f, resPitch = 0.0f;   // applied at the wrist (rad)
    float errBefore = 0.0f;                 // angle barrel vs muzzle -> target, source pose (rad)
    float errAfter = 0.0f;                  // same after the correction (rad)
    bool twoHanded = false;
    bool leftIkOk = false;
    float leftHandErr = 0.0f;               // m, left hand vs its weapon-relative target after the IK
};

// Two-bone IK in model space: moves bone aC (child of aB, child of aA) to aTarget by bending aB (keeps the bend plane) and
// swinging aA, then sets aC's model rotation to aCRot. aModel is recomputed (bones [0, aN)). False = degenerate (straight
// arm, zero lengths): the records are left as they were on entry for the IK bones when it fails before writing.
inline bool TwoBoneIk(uint8_t* aBones, uint32_t aCount, const int16_t* aParents, uint32_t aN, Xf* aModel, uint32_t aA,
                      uint32_t aB, uint32_t aC, Vec3 aTarget, const Quat& aCRot)
{
    const Vec3 a = aModel[aA].p, b = aModel[aB].p, c = aModel[aC].p;
    const float lab = Length(b - a), lcb = Length(c - b);
    if (!(lab > 1e-4f) || !(lcb > 1e-4f))
        return false;
    const float eps = 1e-4f;
    const float lat = Clampf(Length(aTarget - a), std::fabs(lab - lcb) + eps, lab + lcb - eps);
    Vec3 n;
    if (!NormalizeTo(math::Cross(a - b, c - b), n))
        return false; // straight arm: bend plane undefined
    const float cur = AngleBetween(a - b, c - b);
    const float want = std::acos(Clampf((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));
    RotateAboutJoint(aBones, aModel, aParents, aB, AxisAngle(n, want - cur));
    if (!ComputeModel(aBones, aCount, aParents, aN, aModel))
        return false;
    // swing: shortest arc (c - a) -> (target - a) about the joint of aA
    Vec3 u, v;
    if (NormalizeTo(aModel[aC].p - a, u) && NormalizeTo(aTarget - a, v))
    {
        const Vec3 ax = math::Cross(u, v);
        const float sinA = Length(ax), cosA = math::Dot(u, v);
        Vec3 axn;
        if (sinA > 1e-7f && NormalizeTo(ax, axn))
        {
            RotateAboutJoint(aBones, aModel, aParents, aA, AxisAngle(axn, std::atan2(sinA, cosA)));
            if (!ComputeModel(aBones, aCount, aParents, aN, aModel))
                return false;
        }
    }
    SetModelRot(aBones, aModel, aParents, aC, aCRot);
    return ComputeModel(aBones, aCount, aParents, aN, aModel);
}

class Controller
{
public:
    void Reset()
    {
        m_has = false;
        m_yaw = m_pitch = 0.0f;
    }
    bool Has() const { return m_has; }
    float SmoothedYaw() const { return m_yaw; }
    float SmoothedPitch() const { return m_pitch; }

    // aTargetModel: aim point in model space. aWeight: GamePoseShare (scales the applied delta). aDt: s since the previous
    // call (<= 0 holds the smoothed value). Returns false (records untouched, state reset) when the pose / rig cannot be
    // solved. No allocation; ~4 FK passes over rig.fkCount bones.
    bool Apply(uint8_t* aBones, uint32_t aCount, const Rig& aRig, const Params& aP, Vec3 aTargetModel, float aWeight, float aDt,
               Result& aOut)
    {
        aOut = Result{};
        if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount || !math::IsFinite(aTargetModel) ||
            !ValidAxis(aP.barrelAxis))
        {
            Reset();
            return false;
        }
        Xf model[kMaxFkBones];
        const uint32_t n = aRig.fkCount;
        const int16_t* par = aRig.parents;
        const uint32_t ra = uint32_t(aRig.rightArm), rh = uint32_t(aRig.rightHand), wp = uint32_t(aRig.weapon);
        const uint32_t la = uint32_t(aRig.leftArm), lf = uint32_t(aRig.leftForeArm), lh = uint32_t(aRig.leftHand);
        if (!ComputeModel(aBones, aCount, par, n, model))
        {
            Reset();
            return false;
        }
        const Vec3 axis = AxisVector(aP.barrelAxis);
        const Xf w0 = model[wp];
        const Xf lh0 = model[lh];
        const bool twoHanded = Length(lh0.p - w0.p) < aP.twoHandMaxDist;
        Vec3 fwd = math::QuatRotate(w0.q, axis);
        Vec3 muzzle = w0.p + fwd * aP.muzzleOffset;
        float eYaw, ePitch;
        if (!YawPitchDelta(fwd, aTargetModel - muzzle, eYaw, ePitch))
        {
            Reset();
            return false;
        }
        aOut.errYaw = eYaw;
        aOut.errPitch = ePitch;
        aOut.errBefore = AngleBetween(fwd, aTargetModel - muzzle);
        // clamp + smooth (level a)
        const float tYaw = Clampf(eYaw, -aP.maxYawDeg * kDegF, aP.maxYawDeg * kDegF);
        const float tPitch = Clampf(ePitch, -aP.maxPitchDeg * kDegF, aP.maxPitchDeg * kDegF);
        if (!m_has)
        {
            m_yaw = m_pitch = 0.0f;
            m_has = true;
        }
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > 0.1f ? 0.1f : aDt) : 0.0f;
        const float alpha = !(aP.smoothHz > 0.0f) ? 1.0f : 1.0f - std::exp(-aP.smoothHz * dt);
        m_yaw += (tYaw - m_yaw) * alpha;
        m_pitch += (tPitch - m_pitch) * alpha;
        const float wgt = std::isfinite(aWeight) ? Clampf(aWeight, 0.0f, 1.0f) : 0.0f;
        aOut.cmdYaw = m_yaw * wgt;
        aOut.cmdPitch = m_pitch * wgt;
        Quat q;
        if (!YawPitchRotation(fwd, aOut.cmdYaw, aOut.cmdPitch, q))
        {
            Reset();
            return false;
        }
        // A shoulder pivot translates the weapon around RightArm. In a two-handed pose that can move the authored
        // weapon-relative left-hand target outside the left arm's reach; rotate at the wrist instead, then re-solve contact.
        RotateAboutJoint(aBones, model, par, twoHanded ? rh : ra, q);
        if (!ComputeModel(aBones, aCount, par, n, model))
        {
            Reset();
            return false;
        }
        // level (b): only the error the shoulder was meant to remove but did not (muzzle translation), clamped
        fwd = math::QuatRotate(model[wp].q, axis);
        muzzle = model[wp].p + fwd * aP.muzzleOffset;
        float e2Yaw, e2Pitch;
        if (YawPitchDelta(fwd, aTargetModel - muzzle, e2Yaw, e2Pitch))
        {
            const float lim = aP.maxResidualDeg * kDegF;
            aOut.resYaw = Clampf(e2Yaw - (eYaw - aOut.cmdYaw), -lim, lim); // already scaled: cmd carries the weight
            aOut.resPitch = Clampf(e2Pitch - (ePitch - aOut.cmdPitch), -lim, lim);
            Quat qr;
            if ((aOut.resYaw != 0.0f || aOut.resPitch != 0.0f) && YawPitchRotation(fwd, aOut.resYaw, aOut.resPitch, qr))
            {
                RotateAboutJoint(aBones, model, par, rh, qr);
                if (!ComputeModel(aBones, aCount, par, n, model))
                {
                    Reset();
                    return false;
                }
            }
        }
        // left hand follows the weapon rigidly when it holds it
        aOut.twoHanded = twoHanded;
        if (aOut.twoHanded)
        {
            const Quat inv0 = math::QuatInv(w0.q);
            const Xf& w1 = model[wp];
            const Vec3 rel = math::QuatRotate(inv0, lh0.p - w0.p);
            const Vec3 tgt = w1.p + math::QuatRotate(w1.q, rel);
            const Quat rot = math::QuatNormalize(math::QuatMul(w1.q, math::QuatMul(inv0, lh0.q)));
            aOut.leftIkOk = TwoBoneIk(aBones, aCount, par, n, model, la, lf, lh, tgt, rot);
            if (!aOut.leftIkOk && !ComputeModel(aBones, aCount, par, n, model))
            {
                Reset();
                return false;
            }
            aOut.leftHandErr = Length(model[lh].p - tgt);
        }
        fwd = math::QuatRotate(model[wp].q, axis);
        aOut.errAfter = AngleBetween(fwd, aTargetModel - (model[wp].p + fwd * aP.muzzleOffset));
        return true;
    }

private:
    bool m_has = false;
    float m_yaw = 0.0f, m_pitch = 0.0f;
};

} // namespace utpp::mm::aim

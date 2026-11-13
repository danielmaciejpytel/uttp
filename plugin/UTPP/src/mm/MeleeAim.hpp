#pragma once
// Melee aim tilt (2026-11-09, user request: the strike is directed toward the aim - tilted like the body and rifle while aiming a
// gun - within natural human limits). Header-only, no RED4ext / Windows types; unit-tested offline (tests/MeleeAimTest.cpp).
// Runtime glue: mm/MeleeAim.cpp (FK detour, after the MM blend and the gun aim offset), mm/MeleeAimRuntime.hpp.
//
// While the v1 melee mask reason is active (attack / block / targeting + release hold; PoseSnapshot::meleeAimWeight fades with
// it) - in both mask modes, upper-body mask and melee full body - the spine chain is rotated in MODEL space (FK root = the player
// entity; Z up, forward +Y, right +X) by Q = Rz(yaw) * R(+X, pitch), distributed over Spine1 / Spine2 / Spine3 / Neck
// (kShare 0.25 / 0.30 / 0.30 / 0.15, each joint capped at kMaxJointDeg). Children follow: shoulders, arms, hands, WeaponLeft /
// WeaponRight under Spine3 get 85 % of the rotation, Neck1 / Head the whole of it; the head keeps the game's own local motion.
//   pitch = elevation of (aim point - Spine3) RELATIVE TO LEVEL: a level aim leaves the clip unchanged;
//   yaw   = heading of (aim point - Spine3) relative to the model forward (+Y): an aim straight ahead leaves the clip unchanged.
//           Needed because V does not always face the aim in melee: standing still, the camera's idle orbit (camera.idleOrbit,
//           default on) turns only the camera, and Locomotion / the body-turn executor are blocked by the melee attack bits;
//           only melee block / targeting (Locomotion reason 4096) run the aim align onto the camera.
// Aim point (round 3, SelectTarget): the game's melee target (feet vs feet: level ground = no pitch), else the crosshair hit
// within melee reach, else the camera ray with the neutral framing pitch removed (CameraPitchTilt); the tilt may also run
// through the melee guard (melee.guardAimTilt, MMDriver: PoseSnapshot::meleeAimWeight).
// Clamps: pitch up <= maxUpDeg (the melee clips already raise the fists toward eye level), down <= maxDownDeg, yaw <= kMaxYawDeg.
// An aim behind V (|yaw| beyond kFacingFullDeg .. kFacingZeroDeg) fades the whole tilt out: there is nothing to strike at.
// Exponential smoothing (smoothHz); an invalid / stale / too-near aim point targets 0 (the tilt fades out, no reset, no pop).
// The correction is open loop per call: every FK call starts from the fresh blended pose, the output never feeds back.

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>

#include "AimOffset.hpp"
#include "GameIntegration.hpp"
#include "MMMath.hpp"

namespace utpp::mm::meleeaim
{
using aim::Xf;
using math::Quat;
using math::Vec3;

inline constexpr float kDegF = aim::kDegF;
inline constexpr float kDefaultMaxUpDeg = 20.0f;   // the setting's value (DLL default: off)
inline constexpr float kDefaultMaxDownDeg = 35.0f;
inline constexpr float kDefaultSmoothHz = 10.0f;
inline constexpr float kMaxYawDeg = 25.0f;         // yaw part clamp (both sides)
inline constexpr float kMaxJointDeg = 15.0f;       // per-joint cap of the distributed rotation (defaults never reach it: 0.30 x 43)
inline constexpr float kFacingFullDeg = 90.0f;     // |aim yaw| up to this: full tilt
inline constexpr float kFacingZeroDeg = 135.0f;    // |aim yaw| from this on: no tilt (aim behind V)
inline constexpr float kMinTargetDist = 0.5f;      // m from Spine3: a closer aim point has no usable direction
inline constexpr float kMaxTargetAge = 0.25f;      // s (UTPPSetCameraTarget heartbeat: every camera frame)
inline constexpr float kMaxDt = 0.1f;              // s: the smoothing step is capped (pause, loading)
// Target sources (round 3, 2026-11-09 review finding 7: a point 60 m along the camera ray leaned every standing strike ~15 deg
// down at the default framing - pilot melee-lines.txt MELEEAIM tgtPitch -14.0 .. -15.3 at rest - and an enemy 1-3 m away read
// another ~6-8 deg low from the camera height / shoulder offset). In this order:
//   lock = the game's melee target (script: DefaultTransition.GetTargetObject, hostile / neutral active NPC within 5 m), its FEET
//          against V's feet (model z 0): level ground = no pitch whatever the two bodies' heights; yaw toward it;
//   hit  = the camera target point within kMaxHitDist of the chest (melee reach): only the crosshair hit point of utpp_camera's
//          aim point feed can be that close (CM.AIM_POINT, on with "Weapon aims at crosshair point"; off = always 60 m away);
//   cam  = the camera ray (60 m point) with the neutral framing pitch removed: CameraPitchTilt.
inline constexpr float kNeutralPitchDeg = -15.0f;  // camera framing pitch at rest (pilot 2026-11-09: tgtPitch -14.0 .. -15.3)
inline constexpr float kMaxLockDist = 5.0f;        // m horizontal chest -> target feet (the script queries 5 m as well)
inline constexpr float kMaxHitDist = 3.0f;         // m chest -> camera target point: nearer = the crosshair hit (melee reach)
inline constexpr float kMaxLockAge = 0.3f;         // s (UTPPSetMeleeAimTarget heartbeat: <= 20 Hz while a melee weapon is drawn)
// 2026-11-12 (fists review C4 / C5): the crosshair "hit" often is the ground or a bush in front of V, not an enemy.
//   - hit pitch is limited downward to kHitMaxDownDeg (an enemy's chest / head is level or above; the floor is not a target);
//     lock (a real enemy the game tracks) keeps the full range;
//   - hit <-> cam switches with hysteresis (enter hit below kHitEnterDist, leave above kHitLeaveDist; the latch is the caller's);
//   - the camera branch has a yaw dead zone (kCamYawDeadDeg) so facing forward does not hold the idle orbit's constant ~6 deg twist.
inline constexpr float kHitEnterDist = 2.6f;       // m: a camera point nearer than this becomes "hit"
inline constexpr float kHitLeaveDist = 3.4f;       // m: a latched "hit" ends beyond this
inline constexpr float kHitMaxDownDeg = 10.0f;     // deg: lowest pitch of the "hit" source
inline constexpr float kCamYawDeadDeg = 5.0f;      // deg: dead zone (both sides) of the camera branch yaw
inline constexpr int kChain = 4;
inline constexpr float kShare[kChain] = {0.25f, 0.30f, 0.30f, 0.15f}; // Spine1, Spine2, Spine3, Neck (sum 1)

// ------------------------------------------------------------------------------------------------ params

struct Params
{
    float maxUpDeg = 0.0f;   // 0 / 0 = off (DLL default; the setting "Melee aim tilt" sends 20 / 35 / 10)
    float maxDownDeg = 0.0f;
    float smoothHz = kDefaultSmoothHz; // exponential rate (1/s): alpha = 1 - exp(-smoothHz * dt); 0 = no smoothing
};

inline bool ValidParams(float aMaxUpDeg, float aMaxDownDeg, float aSmoothHz)
{
    return std::isfinite(aMaxUpDeg) && std::isfinite(aMaxDownDeg) && std::isfinite(aSmoothHz) && aMaxUpDeg >= 0.0f &&
           aMaxUpDeg <= 45.0f && aMaxDownDeg >= 0.0f && aMaxDownDeg <= 60.0f && aSmoothHz >= 0.0f && aSmoothHz <= 120.0f;
}

inline bool Enabled(const Params& aP) { return aP.maxUpDeg > 0.0f || aP.maxDownDeg > 0.0f; }

// ------------------------------------------------------------------------------------------------ rig

struct Rig
{
    int32_t joint[kChain] = {-1, -1, -1, -1}; // Spine1, Spine2, Spine3, Neck
    uint32_t fkCount = 0;                      // bones [0, fkCount) are FK'd (max index used + 1)
    uint32_t ok = 0;                           // 1 = every check passed (fail closed: 0 -> never applied)
    int16_t parents[aim::kMaxFkBones] = {};
    int32_t Chest() const { return joint[2]; }
};
static_assert(sizeof(Rig) % 4 == 0);

// Resolves Spine1 / Spine2 / Spine3 / Neck by name (game CName hash). Fail closed: ok = 0 when a name is missing, an index is
// >= aim::kMaxFkBones, the parents of [0, fkCount) are not topological, or the chain is not Spine1 -> Spine2 -> Spine3 (direct
// children) -> ... -> Neck.
inline Rig ResolveRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    Rig r;
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return r;
    static constexpr std::string_view kNames[kChain] = {"Spine1", "Spine2", "Spine3", "Neck"};
    int32_t maxIdx = -1;
    for (int k = 0; k < kChain; ++k)
    {
        const uint64_t h = integ::CNameHash(kNames[k]);
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == h)
            {
                r.joint[k] = static_cast<int32_t>(i);
                break;
            }
        if (r.joint[k] < 0 || r.joint[k] >= int32_t(aim::kMaxFkBones))
            return Rig{};
        maxIdx = r.joint[k] > maxIdx ? r.joint[k] : maxIdx;
    }
    r.fkCount = static_cast<uint32_t>(maxIdx + 1);
    for (uint32_t i = 0; i < r.fkCount; ++i)
    {
        r.parents[i] = aParents[i];
        if (aParents[i] >= int32_t(i))
            return Rig{};
    }
    const bool chain = r.parents[r.joint[1]] == r.joint[0] && r.parents[r.joint[2]] == r.joint[1] &&
                       aim::IsAncestor(r.parents, r.fkCount, r.joint[2], r.joint[3]);
    if (!chain)
        return Rig{};
    r.ok = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ target

enum Flag : uint32_t
{
    kFlagClampUp = 1u << 0,   // pitch limited by maxUpDeg
    kFlagClampDown = 1u << 1, // pitch limited by maxDownDeg
    kFlagClampYaw = 1u << 2,  // yaw limited by kMaxYawDeg
    kFlagFacing = 1u << 3,    // aim beside / behind V: tilt scaled down (facing fade < 1)
    kFlagJoint = 1u << 4,     // a joint hit kMaxJointDeg
    kFlagNear = 1u << 5,      // aim point closer than kMinTargetDist / non-finite (target 0)
    kFlagNoTarget = 1u << 6,  // no camera target, disabled or stale (target 0)
    kFlagHitFloor = 1u << 7,  // "hit" pitch limited to -kHitMaxDownDeg (the crosshair hit is the floor / low scenery)
};

struct Target
{
    float pitch = 0.0f; // rad, up > 0, clamped and facing-faded
    float yaw = 0.0f;   // rad, left > 0 (CCW about +Z), clamped and facing-faded
    float rawPitch = 0.0f, rawYaw = 0.0f; // rad before clamp / fade (diagnostics)
    uint32_t flags = 0;
    bool valid = false;
};

using aim::Clampf; // same body as aim::Clampf (AimOffset.hpp)

// Facing fade: 1 within kFacingFullDeg of the model forward, 0 from kFacingZeroDeg on, linear between.
inline float FacingWeight(float aYawRad)
{
    const float a = std::fabs(aYawRad) / kDegF;
    return Clampf((kFacingZeroDeg - a) / (kFacingZeroDeg - kFacingFullDeg), 0.0f, 1.0f);
}

// Clamps (maxUp / maxDown / kMaxYawDeg) and the facing fade of the wanted angles aPitch / aYaw; t.rawYaw decides the facing.
inline Target ClampAndFade(Target t, float aPitch, float aYaw, const Params& aP)
{
    const float up = aP.maxUpDeg * kDegF, down = aP.maxDownDeg * kDegF, yawLim = kMaxYawDeg * kDegF;
    float p = aPitch, y = aYaw;
    if (p > up)
    {
        p = up;
        t.flags |= kFlagClampUp;
    }
    else if (p < -down)
    {
        p = -down;
        t.flags |= kFlagClampDown;
    }
    if (std::fabs(y) > yawLim)
    {
        y = y > 0.0f ? yawLim : -yawLim;
        t.flags |= kFlagClampYaw;
    }
    const float f = FacingWeight(t.rawYaw);
    if (f < 1.0f)
        t.flags |= kFlagFacing;
    t.pitch = p * f;
    t.yaw = y * f;
    t.valid = true;
    return t;
}

// Angles of the aim point aTarget (model space) seen from the chest aChest. Invalid (valid = false, angles 0) for a non-finite
// input or an aim point closer than kMinTargetDist.
inline Target TargetAngles(Vec3 aChest, Vec3 aTarget, const Params& aP)
{
    Target t;
    const Vec3 d = aTarget - aChest;
    const float len = aim::Length(d);
    if (!math::IsFinite(aChest) || !math::IsFinite(aTarget) || !std::isfinite(len) || len < kMinTargetDist)
    {
        t.flags |= kFlagNear;
        return t;
    }
    const float horiz = std::sqrt(d.x * d.x + d.y * d.y);
    t.rawPitch = std::atan2(d.z, horiz);
    t.rawYaw = horiz > 1e-4f ? std::atan2(-d.x, d.y) : 0.0f;
    return ClampAndFade(t, t.rawPitch, t.rawYaw, aP);
}

// Camera ray pitch with the neutral framing pitch removed (dead band): from kNeutralPitchDeg (the camera looking down at V at
// rest) up to level there is no tilt; looking up tilts up by the camera pitch, looking further down tilts down by the excess.
// Plain subtraction (pitch - kNeutralPitchDeg) was rejected: the on-foot camera has no fixed rest pitch (it follows the player's
// look), so a level camera would lean every strike 15 deg UP.
inline float CameraPitchTilt(float aPitchRad)
{
    if (!std::isfinite(aPitchRad))
        return 0.0f;
    return aPitchRad - Clampf(aPitchRad, kNeutralPitchDeg * kDegF, 0.0f);
}

// Yaw of the camera branch with a dead zone of +-kCamYawDeadDeg (shrinkage: continuous at the edge, 0 inside). The facing fade
// still reads the raw yaw. NaN -> 0.
inline float CameraYawDeadZone(float aYawRad)
{
    if (!std::isfinite(aYawRad))
        return 0.0f;
    const float dead = kCamYawDeadDeg * kDegF;
    return aYawRad - Clampf(aYawRad, -dead, dead);
}

// Lock target: the game's melee target, its feet aTargetFeet in model space (V's feet = model z 0). Pitch = elevation of the
// target's ground above V's ground over the horizontal chest -> target distance (level ground = 0, whatever the heights of the
// two bodies); yaw = heading of the target from the chest. Invalid (kFlagNear) when non-finite, closer than kMinTargetDist or
// farther than kMaxLockDist horizontally.
inline Target LockAngles(Vec3 aChest, Vec3 aTargetFeet, const Params& aP)
{
    Target t;
    const float dx = aTargetFeet.x - aChest.x, dy = aTargetFeet.y - aChest.y;
    const float horiz = std::sqrt(dx * dx + dy * dy);
    if (!math::IsFinite(aChest) || !math::IsFinite(aTargetFeet) || !std::isfinite(horiz) || horiz < kMinTargetDist ||
        horiz > kMaxLockDist)
    {
        t.flags |= kFlagNear;
        return t;
    }
    t.rawPitch = std::atan2(aTargetFeet.z, horiz);
    t.rawYaw = std::atan2(-dx, dy);
    return ClampAndFade(t, t.rawPitch, t.rawYaw, aP);
}

enum TargetSource : int32_t
{
    kTgtNone = 0, // no usable target (target 0: the tilt fades to level)
    kTgtLock = 1, // the game's melee target
    kTgtHit = 2,  // camera target point within kMaxHitDist (crosshair hit)
    kTgtCam = 3,  // camera ray, neutral framing pitch removed
};

struct AimInput
{
    bool lockValid = false; // fresh melee target (UTPPSetMeleeAimTarget, <= kMaxLockAge)
    Vec3 lockFeet{};        // its feet, model space
    bool camValid = false;  // fresh camera target point (UTPPSetCameraTarget, <= kMaxTargetAge)
    Vec3 camPoint{};        // model space
};

// Picks the source in the order lock -> hit -> cam (see kNeutralPitchDeg). An unusable lock (near / far / NaN) falls through to
// the camera; no usable source -> invalid target (kFlagNoTarget, plus kFlagNear for a too-near / non-finite camera point).
// aHitLatch (optional, owned by the caller, one per smoothing state): hysteresis of the hit <-> cam switch (enter hit below
// kHitEnterDist, stay until beyond kHitLeaveDist; true = the last call picked "hit"; any other source clears it). nullptr = one
// threshold kMaxHitDist (no state).
inline Target SelectTarget(Vec3 aChest, const AimInput& aIn, const Params& aP, TargetSource* aSrc = nullptr, bool* aHitLatch = nullptr)
{
    TargetSource src = kTgtNone;
    Target t;
    if (aIn.lockValid)
    {
        t = LockAngles(aChest, aIn.lockFeet, aP);
        if (t.valid)
            src = kTgtLock;
    }
    if (src == kTgtNone && aIn.camValid)
    {
        const float len = aim::Length(aIn.camPoint - aChest);
        bool hit = std::isfinite(len) && len <= kMaxHitDist;
        if (aHitLatch)
            hit = std::isfinite(len) && (*aHitLatch ? len <= kHitLeaveDist : len < kHitEnterDist);
        t = TargetAngles(aChest, aIn.camPoint, aP); // invalid (kFlagNear) below kMinTargetDist
        if (t.valid && hit)
        {
            // the floor / low scenery in front of V is no target: limit the downward pitch (lock keeps the full range)
            Target h;
            h.rawPitch = t.rawPitch;
            h.rawYaw = t.rawYaw;
            const float floorPitch = -kHitMaxDownDeg * kDegF;
            const bool floorHit = t.rawPitch < floorPitch;
            t = ClampAndFade(h, floorHit ? floorPitch : t.rawPitch, t.rawYaw, aP);
            if (floorHit)
                t.flags |= kFlagHitFloor;
            src = kTgtHit;
        }
        else if (t.valid)
        {
            t = ClampAndFade(t, CameraPitchTilt(t.rawPitch), CameraYawDeadZone(t.rawYaw), aP);
            src = kTgtCam;
        }
    }
    if (aHitLatch)
        *aHitLatch = (src == kTgtHit);
    if (src == kTgtNone)
    {
        const uint32_t nearFlag = t.flags & kFlagNear; // ("near" is a Windows macro)
        t = Target{};
        t.flags = nearFlag | kFlagNoTarget;
    }
    if (aSrc)
        *aSrc = src;
    return t;
}

// ------------------------------------------------------------------------------------------------ rotation

// Q = Rz(aYaw) * R(+X, aPitch): pitch about the character's right axis (model +X), then the heading turn about +Z.
inline Quat TiltRotation(float aPitch, float aYaw)
{
    return math::QuatNormalize(
        math::QuatMul(aim::AxisAngle(Vec3{0.0f, 0.0f, 1.0f}, aYaw), aim::AxisAngle(Vec3{1.0f, 0.0f, 0.0f}, aPitch)));
}

// Distributes the tilt over the chain on the local records [0, rig.fkCount): joint k turns about its own joint by
// min(kShare[k] * angle(Q), kMaxJointDeg) about the axis of Q (one axis: the parts compose to Q below the last joint).
// Returns false (records untouched) when the rig / pose cannot be solved; aJointCapped = a joint hit kMaxJointDeg. No
// allocation; kChain FK passes over rig.fkCount bones.
inline bool ApplyTilt(uint8_t* aBones, uint32_t aCount, const Rig& aRig, float aPitch, float aYaw, bool* aJointCapped = nullptr)
{
    if (aJointCapped)
        *aJointCapped = false;
    if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount || aRig.fkCount > aim::kMaxFkBones ||
        !std::isfinite(aPitch) || !std::isfinite(aYaw))
        return false;
    const Quat q = TiltRotation(aPitch, aYaw);
    // axis-angle of q (w >= 0: the short way)
    const float s = q.w < 0.0f ? -1.0f : 1.0f;
    const Vec3 v{q.x * s, q.y * s, q.z * s};
    const float vn = aim::Length(v);
    const float angle = 2.0f * std::atan2(vn, q.w * s);
    if (!(vn > 1e-7f) || !(angle > 1e-6f))
        return true; // nothing to rotate
    const Vec3 axis = v * (1.0f / vn);
    Xf model[aim::kMaxFkBones];
    if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
        return false;
    const float cap = kMaxJointDeg * kDegF;
    for (int k = 0; k < kChain; ++k)
    {
        float a = kShare[k] * angle;
        if (a > cap)
        {
            a = cap;
            if (aJointCapped)
                *aJointCapped = true;
        }
        if (k > 0 && !aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
            return false; // the parent of this joint moved with the previous one (records already written: rare, fault only)
        aim::RotateAboutJoint(aBones, model, aRig.parents, uint32_t(aRig.joint[k]), aim::AxisAngle(axis, a));
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ smoothing

class Controller
{
public:
    void Reset() { m_pitch = m_yaw = 0.0f; }
    float Pitch() const { return m_pitch; }
    float Yaw() const { return m_yaw; }
    // One call per applied FK call. aValid false or non-finite angles: target 0 (fades out). aDt <= 0 / NaN holds; capped at
    // kMaxDt. aSmoothHz <= 0: no smoothing.
    void Step(float aPitch, float aYaw, bool aValid, float aDt, float aSmoothHz)
    {
        const float tp = (aValid && std::isfinite(aPitch)) ? aPitch : 0.0f;
        const float ty = (aValid && std::isfinite(aYaw)) ? aYaw : 0.0f;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > kMaxDt ? kMaxDt : aDt) : 0.0f;
        const float alpha = !(aSmoothHz > 0.0f) ? 1.0f : 1.0f - std::exp(-aSmoothHz * dt);
        m_pitch += (tp - m_pitch) * alpha;
        m_yaw += (ty - m_yaw) * alpha;
        if (!std::isfinite(m_pitch) || !std::isfinite(m_yaw))
            Reset();
    }

private:
    float m_pitch = 0.0f, m_yaw = 0.0f;
};

// Weight of the tilt for one FK call: the melee fade of the published snapshot times the share of the upper body that is on the
// game pose (1 - mmWeight * upperScale, aim::GamePoseShare): a frame whose upper body MM / v2 owns (upperScale 1) gets no tilt.
inline float TiltWeight(float aMeleeWeight, float aMMWeight, float aUpperScale)
{
    const float m = std::isfinite(aMeleeWeight) ? Clampf(aMeleeWeight, 0.0f, 1.0f) : 0.0f;
    return m * aim::GamePoseShare(aMMWeight, aUpperScale);
}

} // namespace utpp::mm::meleeaim

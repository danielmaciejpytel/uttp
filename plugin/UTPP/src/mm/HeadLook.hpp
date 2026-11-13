#pragma once
// Head look layer (CET module utpp_core decides when and where to look; this layer turns the body). Header-only, no RED4ext /
// Windows types; unit-tested offline (tests/HeadLookTest.cpp), replayed on recorded poses (tests/HeadLookReplay.cpp).
// Runtime glue: mm/HeadLook.cpp (FK detour, after the MM blend, the gun aim offset and the melee tilt), mm/HeadLookRuntime.hpp.
//
// Why a layer of its own: in third person the game does not turn Neck / Neck1 / Head of the player for a look-at request (recorded
// poses: <= 0.008 deg over 50 s), it only turns the eye bones, and the motion pose written by the mod puts the eyes back to the clip's
// value. So the mod turns the head itself, and the eyes, on the final local pose.
//
// The layer MEASURES and CORRECTS (it never adds a fixed angle on top of an unknown pose):
//   gaze   = direction head joint -> target in the model space of the player entity (Z up, forward +Y, left = positive yaw), clamped
//            to maxYaw (default 100 deg) from the body forward and to +35 / -40 deg pitch, faded out when the target is behind V (from 150 deg, so that the clamped side cannot flip at 180);
//   head   = the model-space direction of the Head's local -Y axis of the pose it receives (this already contains every earlier
//            layer, e.g. the head lead of the motion driver: no double counting by construction);
//   delta  = (gaze - head) * weight, smoothed by a critically damped spring (frame-rate independent, no pop when the target
//            switches between camera and face mode, when the gate closes, when the weight goes to 0). The spring smooths the
//            DELTA, so the natural sway of the clip above the spring bandwidth stays in the pose (the low part follows the target).
//   distribution of the delta: the chest (Spine3) takes chest * kChestShareMax of the yaw (not at all while a weapon is held: the
//            arms, WeaponRight and the hand helpers sit under Spine3), Neck / Neck1 / Head take the rest 0.25 / 0.25 / 0.5 about ONE
//            axis (they compose to the demanded rotation), limited to kChainMaxYawDeg / +30 / -35 deg;
//   eyes   = what is left between the head after the turn and the gaze, within +-30 deg yaw / +-20 deg pitch, from a faster spring.
//            Eye bones (LeftEye / RightEye, children of Head, local rotation identity in the written pose) ARE part of the snapshot the
//            mod writes (database bones 31 / 32), so the engine's eye turn is overwritten and the eyes are driven here.
// Weight 0 and idle springs: nothing is read, nothing is written (the pose is bit-identical).

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>

#include "AimOffset.hpp"
#include "GameIntegration.hpp"
#include "MMMath.hpp"

namespace utpp::mm::headlook
{
using aim::Xf;
using math::Quat;
using math::Vec3;

inline constexpr float kDegF = aim::kDegF;
inline constexpr float kPiF = math::kPi;

// Local forward axis of Head (the eyes sit in the Head frame): bone -Y maps to model +Y in the written pose (checked on a calm frame
// of the in-game pose recordings, tools/analysis/head_look_replay.py).
inline constexpr Vec3 kLocalForward{0.0f, -1.0f, 0.0f};

// Gaze limits (the CET decision uses the same): maxYaw from the body forward (the setting headLook.maxYaw), pitch up / down.
inline constexpr float kDefaultMaxYawDeg = 100.0f;
inline constexpr float kMaxYawSettingDeg = 160.0f;
inline constexpr float kGazeUpDeg = 35.0f;
inline constexpr float kGazeDownDeg = 40.0f;
inline constexpr float kBehindFadeLeadDeg = 15.0f; // the gaze fades out from max(maxYaw + this, kBehindFadeMinStartDeg) to straight behind (180)
inline constexpr float kBehindFadeMinStartDeg = 150.0f;
inline constexpr float kMinTargetDist = 0.3f;      // m horizontal head -> target: closer has no usable heading
// What the body can do (applied delta relative to the pose the layer receives).
inline constexpr float kChainMaxYawDeg = 75.0f;    // Neck + Neck1 + Head together
inline constexpr float kChainUpDeg = 30.0f;
inline constexpr float kChainDownDeg = 35.0f;
inline constexpr float kChestShareMax = 0.30f;     // chest = 1 -> Spine3 takes 30 % of the yaw ...
inline constexpr float kChestMaxDeg = 25.0f;       // ... but never more than this
inline constexpr float kEyeYawDeg = 30.0f;
inline constexpr float kEyeUpDeg = 20.0f;
inline constexpr float kEyeDownDeg = 20.0f;
inline constexpr float kShare[3] = {0.25f, 0.25f, 0.5f}; // Neck, Neck1, Head (same split as the head lead)
// Spring rates (1/s, critically damped: settles in about 4 / omega).
inline constexpr float kHeadOmega = 8.0f;
inline constexpr float kEyeOmega = 22.0f;
inline constexpr float kChestOmega = 8.0f;
inline constexpr float kMaxDt = 0.1f;              // s: one step is capped (pause, loading)
inline constexpr float kMaxTargetAge = 0.25f;      // s: the CET module sends the target every tick (heartbeat)

enum Mode : int32_t
{
    kModeCamera = 0, // what the camera looks at
    kModeFace = 1,   // into the camera (camera in front of V)
    kModeLoot = 2,   // at the selected item
};

inline bool ValidMode(int32_t aMode) { return aMode >= kModeCamera && aMode <= kModeLoot; }

// ------------------------------------------------------------------------------------------------ params

struct Params
{
    float maxYawDeg = kDefaultMaxYawDeg;
    float chest = 0.5f; // 0..1 (the setting headLook.chest)
};

inline bool ValidParams(float aMaxYawDeg, float aChest)
{
    return std::isfinite(aMaxYawDeg) && std::isfinite(aChest) && aMaxYawDeg >= 0.0f && aMaxYawDeg <= kMaxYawSettingDeg &&
           aChest >= 0.0f && aChest <= 1.0f;
}

// ------------------------------------------------------------------------------------------------ rig

inline constexpr int kJointCount = 4; // Spine3, Neck, Neck1, Head

struct Rig
{
    int32_t spine3 = -1, neck = -1, neck1 = -1, head = -1;
    int32_t eye[2] = {-1, -1}; // LeftEye, RightEye (optional: -1 = no eye drive)
    uint32_t fkCount = 0;      // bones [0, fkCount) are FK'd (max index used + 1)
    uint32_t ok = 0;           // 1 = every check passed (fail closed: 0 -> never applied)
    int16_t parents[aim::kMaxFkBones] = {};
};
static_assert(sizeof(Rig) % 4 == 0);

// Resolves the bones by name (game CName hash). Fail closed: ok = 0 when Spine3 / Neck / Neck1 / Head is missing, an index is >=
// aim::kMaxFkBones, the parents of [0, fkCount) are not topological or the chain is not Spine3 -> Neck -> Neck1 -> Head (each an
// ancestor of the next). An eye that is missing or not under Head is left out (eye = -1), the rest still works.
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
    r.spine3 = find("Spine3");
    r.neck = find("Neck");
    r.neck1 = find("Neck1");
    r.head = find("Head");
    r.eye[0] = find("LeftEye");
    r.eye[1] = find("RightEye");
    const int32_t must[] = {r.spine3, r.neck, r.neck1, r.head};
    int32_t maxIdx = -1;
    for (const int32_t i : must)
    {
        if (i < 0 || i >= int32_t(aim::kMaxFkBones))
            return Rig{};
        maxIdx = i > maxIdx ? i : maxIdx;
    }
    for (int32_t& e : r.eye)
    {
        if (e >= int32_t(aim::kMaxFkBones))
            e = -1;
        if (e >= 0)
            maxIdx = e > maxIdx ? e : maxIdx;
    }
    r.fkCount = static_cast<uint32_t>(maxIdx + 1);
    for (uint32_t i = 0; i < r.fkCount; ++i)
    {
        r.parents[i] = aParents[i];
        if (aParents[i] >= int32_t(i))
            return Rig{};
    }
    const bool chain = aim::IsAncestor(r.parents, r.fkCount, r.spine3, r.neck) &&
                       aim::IsAncestor(r.parents, r.fkCount, r.neck, r.neck1) &&
                       aim::IsAncestor(r.parents, r.fkCount, r.neck1, r.head);
    if (!chain)
        return Rig{};
    for (int32_t& e : r.eye)
        if (e >= 0 && !aim::IsAncestor(r.parents, r.fkCount, r.head, e))
            e = -1;
    r.ok = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ gaze

enum Flag : uint32_t
{
    kFlagClampYaw = 1u << 0,   // yaw limited by maxYaw
    kFlagClampUp = 1u << 1,    // pitch limited by kGazeUpDeg
    kFlagClampDown = 1u << 2,  // pitch limited by kGazeDownDeg
    kFlagBehind = 1u << 3,     // target behind V: the gaze fades to straight ahead
    kFlagNear = 1u << 4,       // target closer than kMinTargetDist / non-finite
    kFlagChain = 1u << 5,      // neck chain limit reached (the eyes take the rest)
    kFlagEyes = 1u << 6,       // eye limit reached
    kFlagNoTarget = 1u << 7,   // no fresh target / weight 0
};

struct Gaze
{
    float yaw = 0.0f;   // rad, left > 0, clamped and faded
    float pitch = 0.0f; // rad, up > 0, clamped and faded
    float rawYaw = 0.0f, rawPitch = 0.0f;
    float fade = 1.0f;
    uint32_t flags = 0;
    bool valid = false;
};

inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Gaze angles of aTarget (model space) seen from aHead (model space). Invalid (angles 0) for non-finite input or a target closer
// than kMinTargetDist horizontally.
inline Gaze ComputeGaze(Vec3 aHead, Vec3 aTarget, float aMaxYawDeg)
{
    Gaze g;
    const Vec3 d = aTarget - aHead;
    const float horiz = std::sqrt(d.x * d.x + d.y * d.y);
    if (!math::IsFinite(aHead) || !math::IsFinite(aTarget) || !std::isfinite(horiz) || !std::isfinite(d.z) || horiz < kMinTargetDist)
    {
        g.flags |= kFlagNear;
        return g;
    }
    g.rawYaw = std::atan2(-d.x, d.y);
    g.rawPitch = std::atan2(d.z, horiz);
    const float maxYaw = Clampf(aMaxYawDeg, 0.0f, kMaxYawSettingDeg) * kDegF;
    float y = g.rawYaw, p = g.rawPitch;
    if (std::fabs(y) > maxYaw)
    {
        y = y > 0.0f ? maxYaw : -maxYaw;
        g.flags |= kFlagClampYaw;
    }
    if (p > kGazeUpDeg * kDegF)
    {
        p = kGazeUpDeg * kDegF;
        g.flags |= kFlagClampUp;
    }
    else if (p < -kGazeDownDeg * kDegF)
    {
        p = -kGazeDownDeg * kDegF;
        g.flags |= kFlagClampDown;
    }
    // a target behind V (|yaw| beyond maxYaw + lead): the clamped side would flip at 180 deg, so the gaze fades to straight ahead
    const float startDeg = Clampf(std::fmax(aMaxYawDeg + kBehindFadeLeadDeg, kBehindFadeMinStartDeg), 0.0f, 175.0f);
    const float a = std::fabs(g.rawYaw) / kDegF;
    if (a > startDeg)
    {
        g.fade = Clampf((180.0f - a) / (180.0f - startDeg), 0.0f, 1.0f);
        g.flags |= kFlagBehind;
    }
    g.yaw = y * g.fade;
    g.pitch = p * g.fade;
    g.valid = true;
    return g;
}

inline float WrapPi(float a) { return integ::WrapPi(a); }

// Model-space gaze direction (yaw, pitch) of the Head of the pose (local -Y).
inline bool HeadDirection(const Xf* aModel, const Rig& aRig, float& aYaw, float& aPitch, Vec3* aFwd = nullptr)
{
    const Vec3 f = math::QuatRotate(aModel[aRig.head].q, kLocalForward);
    if (aFwd)
        *aFwd = f;
    return aim::YawPitchOf(f, aYaw, aPitch);
}

// ------------------------------------------------------------------------------------------------ smoothing

// Critically damped spring, exact solution for a constant target (frame-rate independent: the same state at the same time for any
// partition of the time into steps). x = position, v = velocity.
struct Spring
{
    float x = 0.0f, v = 0.0f;
    void Reset() { x = v = 0.0f; }
    void Step(float aTarget, float aOmega, float aDt)
    {
        if (!(aDt > 0.0f) || !(aOmega > 0.0f))
            return;
        const float e = std::exp(-aOmega * aDt);
        const float d = x - aTarget;
        const float tmp = (v + aOmega * d) * aDt;
        v = (v - aOmega * tmp) * e;
        x = aTarget + (d + tmp) * e;
        if (!std::isfinite(x) || !std::isfinite(v))
            Reset();
        else if (aTarget == 0.0f && std::fabs(x) < 1e-4f && std::fabs(v) < 1e-3f)
            Reset(); // settled at rest: exactly 0 (the layer goes idle, the pose is untouched again)
    }
    bool Rest() const { return x == 0.0f && v == 0.0f; }
};

// ------------------------------------------------------------------------------------------------ writing the pose

// Rotates the chest (Spine3) about the model vertical by aChestYaw, then Neck / Neck1 / Head by kShare of ONE rotation (yaw about
// the vertical, pitch about the head's right axis, as aim::YawPitchRotation; they compose to the demanded rotation). The chain part
// is limited to kChainMaxYawDeg / kChainUpDeg / kChainDownDeg. aChainYaw / aChainPitch in radians (up > 0, left > 0). On success
// the model transforms of the result are in aModel (the head direction can be read) and aCapped tells whether a limit was hit.
// False (records untouched or partly written: fault only) when the pose cannot be solved.
inline bool ApplyHead(uint8_t* aBones, uint32_t aCount, const Rig& aRig, float aChestYaw, float aChainYaw, float aChainPitch, Xf* aModel,
                      bool* aCapped = nullptr)
{
    if (aCapped)
        *aCapped = false;
    if (!aRig.ok || !aBones || !aModel || aRig.fkCount == 0 || aRig.fkCount > aCount || aRig.fkCount > aim::kMaxFkBones ||
        !std::isfinite(aChestYaw) || !std::isfinite(aChainYaw) || !std::isfinite(aChainPitch))
        return false;
    if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, aModel))
        return false;
    const float chestCap = kChestMaxDeg * kDegF;
    const float chestYaw = Clampf(aChestYaw, -chestCap, chestCap);
    if (chestYaw != aChestYaw && aCapped)
        *aCapped = true;
    if (chestYaw != 0.0f)
    {
        aim::RotateAboutJoint(aBones, aModel, aRig.parents, uint32_t(aRig.spine3), aim::AxisAngle(Vec3{0.0f, 0.0f, 1.0f}, chestYaw));
        if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, aModel))
            return false;
    }
    const float yawCap = kChainMaxYawDeg * kDegF;
    const float cy = Clampf(aChainYaw, -yawCap, yawCap);
    const float cp = Clampf(aChainPitch, -kChainDownDeg * kDegF, kChainUpDeg * kDegF);
    if ((cy != aChainYaw || cp != aChainPitch) && aCapped)
        *aCapped = true;
    if (cy == 0.0f && cp == 0.0f)
        return true;
    Vec3 fwd;
    Quat q;
    if (!aim::NormalizeTo(math::QuatRotate(aModel[aRig.head].q, kLocalForward), fwd) || !aim::YawPitchRotation(fwd, cy, cp, q))
        return true; // vertical head direction: nothing sensible to turn (cannot happen with the +-35 / 40 deg limits)
    const float s = q.w < 0.0f ? -1.0f : 1.0f;
    const Vec3 v{q.x * s, q.y * s, q.z * s};
    const float vn = aim::Length(v);
    const float angle = 2.0f * std::atan2(vn, q.w * s);
    if (!(vn > 1e-7f) || !(angle > 1e-6f))
        return true;
    const Vec3 axis = v * (1.0f / vn);
    const int32_t joint[3] = {aRig.neck, aRig.neck1, aRig.head};
    for (int k = 0; k < 3; ++k)
    {
        if (k > 0 && !aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, aModel))
            return false;
        aim::RotateAboutJoint(aBones, aModel, aRig.parents, uint32_t(joint[k]), aim::AxisAngle(axis, kShare[k] * angle));
    }
    return aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, aModel);
}

// Turns both eyes about their joints by the model-space rotation that changes the yaw of the head direction by aYaw and its pitch by
// aPitch (radians). aModel must be current (ApplyHead's result or a fresh ComputeModel). The eyes are siblings: one pass.
inline bool ApplyEyes(uint8_t* aBones, uint32_t aCount, const Rig& aRig, float aYaw, float aPitch, const Xf* aModel)
{
    if (!aRig.ok || !aBones || !aModel || !std::isfinite(aYaw) || !std::isfinite(aPitch))
        return false;
    if (aYaw == 0.0f && aPitch == 0.0f)
        return true;
    Vec3 fwd;
    Quat q;
    if (!aim::NormalizeTo(math::QuatRotate(aModel[aRig.head].q, kLocalForward), fwd) || !aim::YawPitchRotation(fwd, aYaw, aPitch, q))
        return true;
    for (const int32_t e : aRig.eye)
        if (e >= 0 && uint32_t(e) < aRig.fkCount && uint32_t(e) < aCount)
            aim::RotateAboutJoint(aBones, aModel, aRig.parents, uint32_t(e), q);
    return true;
}

// ------------------------------------------------------------------------------------------------ the layer

struct Input
{
    bool targetValid = false; // a fresh target was received (age <= kMaxTargetAge) and the player transform was readable
    Vec3 target{};            // model space
    float weight = 0.0f;      // 0..1 from the CET decision (0 = release)
    int32_t mode = kModeCamera;
    float chest = 0.5f;       // 0..1
    float maxYawDeg = kDefaultMaxYawDeg;
    float gate = 0.0f;        // 0..1 MM ownership of the pose x (1 - melee fade); 0 = the layer is not allowed to touch the pose
    bool chestAllowed = false;// no weapon in the hands (Spine3 carries the arms, the weapon and the hand helpers)
    float dt = 0.0f;          // s since the previous step (0 = first step after a reset)
};

struct Output
{
    bool wrote = false;           // any record changed this call
    float chestYaw = 0.0f;        // applied (after the gate), rad
    float yaw = 0.0f, pitch = 0.0f; // neck chain, rad
    float eyeYaw = 0.0f, eyePitch = 0.0f;
    float gazeYaw = 0.0f, gazePitch = 0.0f; // clamped gaze (rad), 0 without a target
    float headYaw = 0.0f, headPitch = 0.0f; // head direction of the input pose (rad)
    uint32_t flags = 0;
    int32_t mode = kModeCamera;
};

class Layer
{
public:
    void Reset()
    {
        m_yaw.Reset();
        m_pitch.Reset();
        m_eyeYaw.Reset();
        m_eyePitch.Reset();
        m_chest = 0.0f;
    }
    bool Idle() const { return m_yaw.Rest() && m_pitch.Rest() && m_eyeYaw.Rest() && m_eyePitch.Rest() && m_chest == 0.0f; }
    float Yaw() const { return m_yaw.x; }
    float Pitch() const { return m_pitch.x; }
    float EyeYaw() const { return m_eyeYaw.x; }
    float EyePitch() const { return m_eyePitch.x; }

    // One call per applied FK call. Returns false only when the pose could not be solved (records untouched or partly written).
    // An idle layer with nothing to do touches nothing and reads nothing (wrote = false).
    bool Run(uint8_t* aBones, uint32_t aCount, const Rig& aRig, const Input& aIn, Output& aOut)
    {
        aOut = Output{};
        aOut.mode = aIn.mode;
        const float gate = std::isfinite(aIn.gate) ? Clampf(aIn.gate, 0.0f, 1.0f) : 0.0f;
        const float weight = std::isfinite(aIn.weight) ? Clampf(aIn.weight, 0.0f, 1.0f) : 0.0f;
        const bool want = aIn.targetValid && weight > 0.0f && gate > 0.0f && aRig.ok;
        if (!want && Idle())
        {
            aOut.flags |= kFlagNoTarget;
            return true;
        }
        if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount)
            return false;
        const float dt = (std::isfinite(aIn.dt) && aIn.dt > 0.0f) ? (aIn.dt > kMaxDt ? kMaxDt : aIn.dt) : 0.0f;
        Xf model[aim::kMaxFkBones];
        if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
            return false;
        float hy = 0.0f, hp = 0.0f;
        const bool haveHead = HeadDirection(model, aRig, hy, hp);
        aOut.headYaw = hy;
        aOut.headPitch = hp;
        Gaze gaze;
        if (want && haveHead)
            gaze = ComputeGaze(model[aRig.head].p, aIn.target, aIn.maxYawDeg);
        if (!gaze.valid)
            aOut.flags |= kFlagNoTarget | (gaze.flags & kFlagNear);
        else
        {
            aOut.gazeYaw = gaze.yaw;
            aOut.gazePitch = gaze.pitch;
            aOut.flags |= gaze.flags;
        }
        // head + chest: the delta between the gaze and the head the pose already has (0 = release)
        const float dY = gaze.valid ? WrapPi(gaze.yaw - hy) * weight : 0.0f;
        const float dP = gaze.valid ? (gaze.pitch - hp) * weight : 0.0f;
        m_yaw.Step(dY, kHeadOmega, dt);
        m_pitch.Step(dP, kHeadOmega, dt);
        const float chestTarget = (gaze.valid && aIn.chestAllowed) ? Clampf(aIn.chest, 0.0f, 1.0f) * kChestShareMax : 0.0f;
        if (dt > 0.0f)
            m_chest += (chestTarget - m_chest) * (1.0f - std::exp(-kChestOmega * dt));
        if (m_chest < 1e-5f && chestTarget == 0.0f)
            m_chest = 0.0f;
        const float yaw = m_yaw.x * gate, pitch = m_pitch.x * gate;
        const float chestYaw = m_yaw.x * m_chest * gate;
        const float chainYaw = yaw - chestYaw;
        bool capped = false;
        const bool headWrite = chestYaw != 0.0f || chainYaw != 0.0f || pitch != 0.0f;
        bool ok = true;
        if (headWrite)
        {
            ok = ApplyHead(aBones, aCount, aRig, chestYaw, chainYaw, pitch, model, &capped);
            if (ok)
            {
                aOut.wrote = true;
                aOut.chestYaw = chestYaw;
                aOut.yaw = chainYaw;
                aOut.pitch = pitch;
            }
            if (capped)
                aOut.flags |= kFlagChain;
            if (!ok)
                return false;
            float ay = 0.0f, ap = 0.0f;
            if (HeadDirection(model, aRig, ay, ap))
            {
                hy = ay;
                hp = ap;
            }
        }
        // eyes: what is left between the (turned) head and the gaze
        float eY = 0.0f, eP = 0.0f;
        if (gaze.valid && (aRig.eye[0] >= 0 || aRig.eye[1] >= 0))
        {
            const float ry = WrapPi(gaze.yaw - hy), rp = gaze.pitch - hp;
            eY = Clampf(ry, -kEyeYawDeg * kDegF, kEyeYawDeg * kDegF) * weight;
            eP = Clampf(rp, -kEyeDownDeg * kDegF, kEyeUpDeg * kDegF) * weight;
            if (eY != ry * weight || eP != rp * weight)
                aOut.flags |= kFlagEyes;
        }
        m_eyeYaw.Step(eY, kEyeOmega, dt);
        m_eyePitch.Step(eP, kEyeOmega, dt);
        const float ey = m_eyeYaw.x * gate, ep = m_eyePitch.x * gate;
        if (ey != 0.0f || ep != 0.0f)
        {
            if (ApplyEyes(aBones, aCount, aRig, ey, ep, model))
            {
                aOut.wrote = true;
                aOut.eyeYaw = ey;
                aOut.eyePitch = ep;
            }
        }
        return true;
    }

    // Re-applies a stored output to a fresh pose (a concurrent animation job that does not own the state): no step, no measurement.
    static bool WriteOutput(uint8_t* aBones, uint32_t aCount, const Rig& aRig, const Output& aOut)
    {
        if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount)
            return false;
        Xf model[aim::kMaxFkBones];
        if (aOut.chestYaw != 0.0f || aOut.yaw != 0.0f || aOut.pitch != 0.0f)
        {
            if (!ApplyHead(aBones, aCount, aRig, aOut.chestYaw, aOut.yaw, aOut.pitch, model))
                return false;
        }
        else if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
            return false;
        return ApplyEyes(aBones, aCount, aRig, aOut.eyeYaw, aOut.eyePitch, model);
    }

private:
    Spring m_yaw, m_pitch, m_eyeYaw, m_eyePitch;
    float m_chest = 0.0f;
};

// Weight of the layer for one FK call: the share of the upper body the motion pose owns (snapshot weight x upper scale; during an
// aim / melee mask the game owns the head) times the melee fade: 1 = free, 0 = no write.
inline float LayerGate(float aMMWeight, float aUpperScale, float aMeleeWeight)
{
    const float own = 1.0f - aim::GamePoseShare(aMMWeight, aUpperScale);
    const float melee = std::isfinite(aMeleeWeight) ? Clampf(aMeleeWeight, 0.0f, 1.0f) : 0.0f;
    return Clampf(own * (1.0f - melee), 0.0f, 1.0f);
}

} // namespace utpp::mm::headlook

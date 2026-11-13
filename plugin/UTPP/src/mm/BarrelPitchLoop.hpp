#pragma once
// UTPP motion matching runtime - closed loop on the weapon barrel pitch of the v2 aim offset (aim pitch plan 2026-10-30,
// section 2.B). Header-only, no game types, no pose data. Used by motionpose::PoseEvaluator (MotionPose.hpp) between ApplyAimOffset and
// the grip IK; offline test: MotionPoseTest 10.
//
// Why: the v2 aim offset maps the target pitch to the authored motion sweep without feedback. The barrel (WeaponRight
// barrel axis, muzzle = weapon bone + axis * muzzle length) ends up 5-15 deg above the hit point P of the camera ray at
// level aim (MotionPoseTest 10, open loop: the base Aim pose elevation and a sweep gain below 1 per overlay / style / stance), and
// further off at steep pitch and short range. The bullet is a hitscan from the camera, so the visible weapon has to point at P.
//
// Two levels, both on the barrel measured after the aim offset (B1):
//   (B2) bias integrator on the sweep input: pitchCmd = target pitch + bias. One bias per overlay x stance x style slot; the
//        command of a frame is the bias of the slots blended by the overlay machine weights (no pop at a crossfade) and each
//        slot learns its weight share of the error. Rate kBarrelBiasHz, clamp +-kBarrelBiasClampDeg, the applied value moves at
//        most kBarrelStepDeg per frame. Keeps the authored motion anatomy (no new bone rotation).
//   (B3) bounded residual for what the sweep cannot reach (steep pitch saturation) and the integrator's transient: an
//        integrator on the error left AFTER the residual (rate kBarrelResidualRate, aim::Params::smoothHz convention),
//        clamped to +-kBarrelResidualMaxDeg; it takes effect on the next frame within the step the bias left of kBarrelStepDeg
//        (bias + residual move the barrel by at most kBarrelStepDeg per frame). Measured after the residual because the
//        muzzle moves with the arm: for a hit point ~1 m away the pre-residual error over- or under-states the rotation needed.
//        Rotation about the lateral axis of the barrel heading at RightArm; above kBarrelSpineStartDeg kBarrelSpineShare of the
//        excess goes to Spine2 / Spine3. Never the wrist.
// Weight (B4): learning only at a full aim offset (alpha >= kBarrelLearnMinAlpha) with a live target, outside a shot (+
// kBarrelShotHoldSeconds), a reload hand-over (aoScale < 1) and a handback (aoSuppress). Excluded frames keep the learned bias
// applied (the aim offset alpha scales it). The residual is FROZEN through a shot (+ hold) and a latch (kBarrelExFreezeMask: the
// recoil / shoot transfer runs after the loop, a fade there pumped the barrel by up to the residual with every semi-auto shot)
// and fades out for the other exclusions (kBarrelExFadeMask); without an aim point (or with the loop switched off) the
// applied bias and residual fade to 0 at kBarrelStepDeg per frame, after which the evaluator bypasses the loop bit for bit.
// A hit point within the gun's own length of the muzzle (a wall right in front, the floor at the feet: the camera sits 1.1 m
// behind V, so a 2 m camera ray ends ~0.5 m from the muzzle) has no aim direction to match: below kBarrelMinTargetDist the
// target weight fades the applied bias and the residual toward the open loop (0 at kBarrelFadeTargetDist) and nothing is learned.
// A frame with dt <= 0 (probe frames of the offline tests) moves no state.
//
// Mutation (UTPP_POSEV2_AO_MUTATION, shared with MotionPose.hpp): 18 = the loop never applies a correction (open loop).
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "AimOffset.hpp"
#include "MMMath.hpp"

namespace utpp::mm::motionpose
{
inline constexpr float kBarrelBiasClampDeg = 15.0f;   // plan B2: bias clamp
inline constexpr float kBarrelBiasHz = 7.0f;          // plan B2: 6-8 Hz, alpha = 1 - exp(-2 pi f dt) (0.52 at 60 fps)
inline constexpr float kBarrelStepDeg = 1.0f;         // plan B4 / section 5: largest change of a correction in one frame
inline constexpr float kBarrelResidualMaxDeg = 6.0f;  // plan B3: residual clamp (protected shoulder: small)
inline constexpr float kBarrelResidualRate = 12.0f;   // plan B3: v1 aim::Params::smoothHz, alpha = 1 - exp(-12 dt)
inline constexpr float kBarrelSpineStartDeg = 3.0f;   // plan B3: residual above this ...
inline constexpr float kBarrelSpineShare = 0.3f;      // ... puts this share of the excess on Spine2 / Spine3
inline constexpr float kBarrelShotHoldSeconds = 0.2f; // plan B4: no learning / residual 0.2 s after a shot (recoil, H8)
inline constexpr float kBarrelLearnMinAlpha = 0.99f;  // a fading aim offset measures a mixed pose: learn only at full AO
// A hit point closer to the muzzle than kBarrelMinTargetDist, or an error this large, is not an aim error the loop can fix
// (P beside / behind the gun at a wall, floor point under the feet): no learning, the target weight (smoothstep over
// [kBarrelFadeTargetDist, kBarrelMinTargetDist]) scales the applied bias and the residual target.
inline constexpr float kBarrelMinTargetDist = 1.0f;   // m, muzzle -> P: full loop from here
inline constexpr float kBarrelFadeTargetDist = 0.6f;  // m, muzzle -> P: open loop below
inline constexpr float kBarrelMaxErrDeg = 35.0f;
inline constexpr int kBarrelSlots = 64; // overlay (ov::PoseOverlay::Count 6) x stance 2 x style 3 = 36

// FrameInfo::barrelExcluded bits
enum BarrelExclusion : uint32_t
{
    kBarrelExShot = 1u << 0,     // shoot transfer active or within kBarrelShotHoldSeconds after it
    kBarrelExReload = 1u << 1,   // reload hand-over (aoScale < 1)
    kBarrelExLatch = 1u << 2,    // latched angles of a lost target
    kBarrelExSuppress = 1u << 3, // handback (aoSuppress)
    kBarrelExNoPoint = 1u << 4,  // no valid aim point / loop switched off: fading out
    kBarrelExFade = 1u << 5,     // aim offset alpha below kBarrelLearnMinAlpha
    kBarrelExTarget = 1u << 6,   // degenerate target (kBarrelMinTargetDist / kBarrelMaxErrDeg) or no measurement
};
// Residual handling of an excluded frame (review 2026-10-30): any fade bit -> target 0 (open loop); else any freeze bit -> the
// residual holds its value (no learning); no bit -> it integrates the error.
inline constexpr uint32_t kBarrelExFadeMask = kBarrelExNoPoint | kBarrelExFade | kBarrelExSuppress | kBarrelExReload | kBarrelExTarget;
inline constexpr uint32_t kBarrelExFreezeMask = kBarrelExShot | kBarrelExLatch;

struct BarrelMeasure
{
    float barrelPitchDeg = 0.0f; // elevation of the barrel axis (+ = up)
    float wantPitchDeg = 0.0f;   // elevation of muzzle -> P
    float errDeg = 0.0f;         // barrel - want (+ = the barrel points above P)
    float dist = 0.0f;           // |P - muzzle| (m)
    math::Vec3 barrel{};         // unit barrel direction (model / pose frame)
};

// Barrel and muzzle -> P of a WeaponRight model transform. aAxis: aim::AxisVector convention (WeaponRight local axis).
// False for a non-finite input, a degenerate direction or a vertical one (no heading).
inline bool MeasureBarrel(const math::Quat& aWeaponRot, const math::Vec3& aWeaponPos, int32_t aAxis, float aMuzzle,
                          const math::Vec3& aPoint, BarrelMeasure& aOut)
{
    if (!aim::ValidAxis(aAxis) || !std::isfinite(aMuzzle) || !math::IsFinite(aPoint) || !math::IsFinite(aWeaponPos))
        return false;
    math::Vec3 b;
    if (!aim::NormalizeTo(math::QuatRotate(aWeaponRot, aim::AxisVector(aAxis)), b))
        return false;
    const math::Vec3 want = aPoint - (aWeaponPos + b * aMuzzle);
    float y0, p0, y1, p1;
    if (!aim::YawPitchOf(b, y0, p0) || !aim::YawPitchOf(want, y1, p1))
        return false;
    aOut.barrel = b;
    aOut.barrelPitchDeg = p0 / aim::kDegF;
    aOut.wantPitchDeg = p1 / aim::kDegF;
    aOut.errDeg = aOut.barrelPitchDeg - aOut.wantPitchDeg;
    aOut.dist = aim::Length(want);
    return std::isfinite(aOut.errDeg) && std::isfinite(aOut.dist);
}

// 1 at muzzle -> P >= kBarrelMinTargetDist, 0 at <= kBarrelFadeTargetDist, smoothstep between.
inline float BarrelTargetWeight(float aDist)
{
    if (!std::isfinite(aDist))
        return 0.0f;
    const float t = aim::Clampf((aDist - kBarrelFadeTargetDist) / (kBarrelMinTargetDist - kBarrelFadeTargetDist), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

inline float BarrelStepToward(float aFrom, float aTo, float aMaxStep)
{
    const float d = aTo - aFrom;
    return std::fabs(d) <= aMaxStep ? aTo : aFrom + std::copysign(aMaxStep, d);
}

class BarrelPitchLoop
{
public:
    void Reset() { *this = BarrelPitchLoop{}; }
    // No layering / a montage without a target (review 2026-10-30): the applied correction and the residual end with the aim
    // offset, the learned slot biases stay (the next aim entry does not start from the open-loop error again).
    void ResetApplied()
    {
        m_applied = m_residual = m_residualTarget = m_lastStep = m_shotHold = 0.0f;
        m_targetW = 1.0f;
    }
    // Nothing is applied: the evaluator may bypass the loop (bit for bit the open-loop aim offset).
    bool Idle() const { return m_applied == 0.0f && m_residual == 0.0f; }
    float Residual() const { return m_residual; }
    float Bias(int aSlot) const { return aSlot >= 0 && aSlot < kBarrelSlots ? m_bias[aSlot] : 0.0f; }
    // Target weight (BarrelTargetWeight) of the last measurement; the next Command scales the bias with it.
    void SetTargetWeight(float aW) { m_targetW = aim::Clampf(aW, 0.0f, 1.0f); }

    // Start of a frame: the bias added to the sweep pitch, then the residual of this frame. aRun false (no aim point, loop off)
    // fades the applied bias to 0. The applied bias moves at most kBarrelStepDeg per frame toward the blended slot bias x
    // target weight, the residual toward the target the last frame left (SetResidualTarget) within the rest of that step.
    // dt <= 0 keeps both.
    float Command(const int* aSlots, const float* aW, int aCount, bool aRun, float aDt)
    {
        float raw = 0.0f;
        if (aRun)
            for (int k = 0; k < aCount; ++k)
                if (aSlots[k] >= 0 && aSlots[k] < kBarrelSlots)
                    raw += aW[k] * m_bias[aSlots[k]];
        raw *= m_targetW;
        m_lastStep = 0.0f;
        if (aDt > 0.0f && std::isfinite(aDt) && std::isfinite(raw))
        {
            const float before = m_applied;
            m_applied = BarrelStepToward(m_applied, raw, kBarrelStepDeg);
            m_lastStep = std::fabs(m_applied - before);
        }
        StepResidual(aRun ? m_residualTarget : 0.0f, aDt);
        return m_applied;
    }
    // End of a frame: the residual target of the next frame (0 = fade out).
    void SetResidualTarget(float aDeg) { m_residualTarget = std::isfinite(aDeg) ? aDeg : 0.0f; }

    // Integrator: each slot moves by its weight share of alpha * (want - barrel), at most kBarrelStepDeg per frame, clamped.
    void Learn(const int* aSlots, const float* aW, int aCount, float aErrDeg, float aDt)
    {
        if (!(aDt > 0.0f) || !std::isfinite(aDt) || !std::isfinite(aErrDeg))
            return;
        const float alpha = 1.0f - std::exp(-2.0f * math::kPi * kBarrelBiasHz * std::min(aDt, 0.1f));
        const float step = aim::Clampf(-aErrDeg * alpha, -kBarrelStepDeg, kBarrelStepDeg);
        for (int k = 0; k < aCount; ++k)
            if (aSlots[k] >= 0 && aSlots[k] < kBarrelSlots)
                m_bias[aSlots[k]] = aim::Clampf(m_bias[aSlots[k]] + step * aim::Clampf(aW[k], 0.0f, 1.0f),
                                                -kBarrelBiasClampDeg, kBarrelBiasClampDeg);
    }

    // Residual toward aTargetDeg (0 = fade out): smoothing, clamp, and a step within what the bias step of this frame left of
    // kBarrelStepDeg (the barrel correction as a whole moves at most kBarrelStepDeg per frame). dt <= 0 keeps it.
    float StepResidual(float aTargetDeg, float aDt)
    {
        if (aDt > 0.0f && std::isfinite(aDt) && std::isfinite(aTargetDeg))
        {
            const float target = aim::Clampf(aTargetDeg, -kBarrelResidualMaxDeg, kBarrelResidualMaxDeg);
            const float a = 1.0f - std::exp(-kBarrelResidualRate * std::min(aDt, 0.1f));
            const float budget = std::max(0.0f, kBarrelStepDeg - m_lastStep);
            m_residual = aim::Clampf(BarrelStepToward(m_residual, m_residual + (target - m_residual) * a, budget),
                                     -kBarrelResidualMaxDeg, kBarrelResidualMaxDeg);
            // a residual that has faded below float noise ends exactly (the evaluator bypasses an idle loop)
            if (aTargetDeg == 0.0f && std::fabs(m_residual) < 1e-4f)
                m_residual = 0.0f;
        }
        return m_residual;
    }
    void ClearResidual() { m_residual = m_residualTarget = 0.0f; }

    // Shot exclusion with the hold after it. dt <= 0 keeps the hold.
    bool ShotExcluded(bool aShot, float aDt)
    {
        if (aShot)
            m_shotHold = kBarrelShotHoldSeconds;
        else if (aDt > 0.0f && std::isfinite(aDt))
            m_shotHold = std::max(0.0f, m_shotHold - aDt);
        return aShot || m_shotHold > 0.0f;
    }

private:
    float m_bias[kBarrelSlots] = {};
    float m_applied = 0.0f;
    float m_targetW = 1.0f;  // BarrelTargetWeight of the last measurement
    float m_lastStep = 0.0f; // |change of the applied bias| in the last Command
    float m_residual = 0.0f;
    float m_residualTarget = 0.0f; // residual of the next frame (set at the end of a frame)
    float m_shotHold = 0.0f;
};

// Residual split (plan B3): the arm takes the residual up to kBarrelSpineStartDeg, above it kBarrelSpineShare of the excess
// goes to the spine (half Spine2, half Spine3). aArm + aSpine = aResidual.
inline void SplitBarrelResidual(float aResidualDeg, float& aArmDeg, float& aSpineDeg)
{
    const float excess = std::max(0.0f, std::fabs(aResidualDeg) - kBarrelSpineStartDeg);
    aSpineDeg = std::copysign(kBarrelSpineShare * excess, aResidualDeg);
    aArmDeg = aResidualDeg - aSpineDeg;
}
} // namespace utpp::mm::motionpose

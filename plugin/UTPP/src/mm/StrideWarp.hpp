// Shrink-only stride warping of the leg targets. Header-only, allocation-free, deterministic.
//
// The play rate speeds a clip up to the capsule; what the rate cannot absorb (a clip whose root, at its rate, still moves faster than the
// capsule) shows as planted feet sliding backwards. Stride warping shortens the step instead: every ankle target is pulled towards the
// thigh along the root-motion direction by the stride scale
//   scale = capsule speed / (clip root speed x play rate x clip factor), clamped to [kScaleFloor, 1]
// so a planted foot moves backwards in the character frame at capsule speed. The clip factor is the clip's own planted-foot speed over its
// root speed (ClipFactor, measured once per clip from its contact frames): a clip whose planted feet travel slower than its root needs less
// shortening than the root speed alone asks for. The scale never exceeds 1: a clip slower than the capsule
// keeps its strides (the rate handles that side), the step is never lengthened.
//
// Reference behaviour (the stride warping node of the locomotion graph, graph mode):
//   - scale from the locomotion speed over the root-motion speed, clamped to at most 1, interpolated at 10 / s both ways;
//   - off (scale 1) when the root-motion speed is below 10 cm/s;
//   - the node alpha is the clip's warping curve (the same curve that gates orientation warping);
//   - the feet are scaled along the stride direction about the thigh projected on the floor plane; the pelvis is only pulled down when
//     the warped legs would over-extend. Shortening moves the feet towards the thighs, the hip-to-ankle distance only shrinks, so the
//     pull-down never engages on this side and is not modelled here.
// Differences: the floor is 0.75 instead of 0 (a stride shorter than three quarters bends the knees beyond what the shin and skin
// helpers carry well), and the interpolation is the frame-rate independent exponential form of the same 10 / s rate.
#pragma once

#include "MMMath.hpp"

#include <algorithm>
#include <cmath>

#ifndef UTPP_STRIDE_SCALE_FLOOR
// Shortest stride as a fraction of the clip's own (kScaleFloor); a build may override it for comparison.
#define UTPP_STRIDE_SCALE_FLOOR 0.75f
#endif

namespace utpp::mm::stridewarp
{
inline constexpr float kScaleFloor = UTPP_STRIDE_SCALE_FLOOR; // shortest stride, fraction of the clip's own
inline constexpr float kInterpSpeed = 10.0f;  // 1/s, scale interpolation towards the target
inline constexpr float kMinRootSpeed = 0.10f; // m/s, root-motion speed below which the clip is not warped
inline constexpr float kIdentityEps = 1.0e-6f; // |1 - scale| at or below this is treated as no warp
inline constexpr float kClipFactorMin = 0.85f;  // clip factor range: planted-foot speed over root speed of a clip
inline constexpr float kClipFactorMax = 1.15f;
inline constexpr int kClipFactorMinSamples = 6; // planted samples a clip needs for a measured factor (else 1)
// Yaw fade: a visual root that keeps swinging (a crouched gait sways it in step with the stance foot) moves the planted foot in the world
// by the rotation alone, which no stride scale absorbs; the warp then adds slide. The smoothed |yaw rate| fades the warp out between
// these two rates.
inline constexpr float kYawFadeStart = 0.5236f; // rad/s (30 deg/s), full warp at or below
inline constexpr float kYawFadeEnd = 1.5708f;   // rad/s (90 deg/s), no warp at or above
inline constexpr float kYawSmoothing = 4.0f;    // 1/s, smoothing of |yaw rate|

struct Inputs
{
    float dt = 0;           // s
    float capsuleSpeed = 0; // m/s, horizontal capsule speed
    float rootSpeed = 0;    // m/s, horizontal clip root speed at the clip's play rate
    float clipFactor = 1;   // planted-foot speed over root speed of the clip (ClipFactor)
    float alpha = 1;        // warping curve of the clip at its current time, [0, 1]
    bool allowed = false;   // ground locomotion pick (not air, slide, traversal or vehicle)
    float yawFade = 1;      // YawFade of the smoothed visual yaw rate, [0, 1]
};

struct State
{
    float scale = 1;        // interpolated stride scale before alpha
    bool initialized = false;
};

// Stride scale the clip asks for this frame (before interpolation and alpha); 1 = no warp.
inline float TargetScale(const Inputs &in) noexcept
{
    if (!in.allowed || !std::isfinite(in.capsuleSpeed) || !std::isfinite(in.rootSpeed) || in.rootSpeed < kMinRootSpeed)
        return 1.0f;
    const float capsule = std::max(0.0f, in.capsuleSpeed);
    const float factor = std::isfinite(in.clipFactor) ? std::clamp(in.clipFactor, kClipFactorMin, kClipFactorMax) : 1.0f;
    return std::clamp(capsule / (in.rootSpeed * factor), kScaleFloor, 1.0f);
}

// Clip factor from summed planted samples of a clip: backward speed of the planted toes along the root direction in the character frame
// (plantedBack) over the root speed of the same samples (root), clamped to [kClipFactorMin, kClipFactorMax]; 1 with too few samples.
inline float ClipFactor(double plantedBack, double root, int samples) noexcept
{
    if (samples < kClipFactorMinSamples || !std::isfinite(plantedBack) || !std::isfinite(root) || root <= 1.0e-6)
        return 1.0f;
    return std::clamp(float(plantedBack / root), kClipFactorMin, kClipFactorMax);
}

// Advances the layer's stride state and returns the effective scale (alpha applied). The first frame of a layer takes the target
// directly (a fresh pick fades in through the blend); dt <= 0 holds the state.
inline float Step(State &state, const Inputs &in) noexcept
{
    const float target = TargetScale(in);
    if (!state.initialized)
    {
        state.scale = target;
        state.initialized = true;
    }
    else if (std::isfinite(in.dt) && in.dt > 0)
        state.scale += (target - state.scale) * (1.0f - std::exp(-kInterpSpeed * in.dt));
    state.scale = std::clamp(state.scale, kScaleFloor, 1.0f);
    const float alpha = std::isfinite(in.alpha) ? std::clamp(in.alpha, 0.0f, 1.0f) : 0.0f;
    const float fade = std::isfinite(in.yawFade) ? std::clamp(in.yawFade, 0.0f, 1.0f) : 1.0f;
    return 1.0f - alpha * fade * (1.0f - state.scale);
}

// Warp weight for a smoothed |visual yaw rate| (rad/s): 1 up to start (kYawFadeStart), 0 from end (kYawFadeEnd), linear between.
inline float YawFade(float smoothedYawRate, float start = kYawFadeStart, float end = kYawFadeEnd) noexcept
{
    if (!std::isfinite(smoothedYawRate) || !std::isfinite(start) || !std::isfinite(end) || !(end > start))
        return 1.0f;
    return std::clamp((end - std::fabs(smoothedYawRate)) / (end - start), 0.0f, 1.0f);
}

// Advances the smoothed |yaw rate| (rad/s) from the visual yaw change over dt (radians, any winding); dt <= 0 holds it.
inline float SmoothYawRate(float smoothed, float yawDelta, float dt) noexcept
{
    if (!(dt > 0) || !std::isfinite(dt) || !std::isfinite(yawDelta))
        return smoothed;
    const float wrapped = std::remainder(yawDelta, 6.28318531f);
    return smoothed + (std::fabs(wrapped) / dt - smoothed) * (1.0f - std::exp(-kYawSmoothing * dt));
}

// Knee limit: shortening a step moves the ankle towards the thigh, the hip-to-ankle distance shrinks and the two-bone leg folds further.
// A leg folded deep (the trailing leg of a run, heel up behind the hips) folds by up to 30 degrees more under a 0.9 scale, past what
// the knee skin helpers carry. Per leg, the scale is raised so the knee flexion (0 = straight) after the warp stays at or below
// max(flexion before + margin, free flexion): a leg that stays below the free flexion warps fully, a deeper one gains at most the margin.
// On recorded sessions a free flexion of 60 degrees kept most of the planted-foot gain with knee skinning close to the unwarped pose;
// 70 degrees and more left the loop knees visibly more folded.
inline constexpr float kKneeMargin = 0.0052360f;    // rad (0.3 deg), largest added flexion above the free flexion
inline constexpr float kKneeFreeFlexion = 1.0472f;  // rad (60 deg), flexion up to which the warp is not limited

// Knee flexion (rad, 0 = straight) of a two-bone leg with thigh / shin lengths whose hip-to-ankle distance is d.
inline float KneeFlexion(float d, float thigh, float shin) noexcept
{
    const float c = (d * d - thigh * thigh - shin * shin) / (2.0f * thigh * shin);
    return std::acos(std::clamp(c, -1.0f, 1.0f));
}

// Smallest scale of one leg that keeps its knee flexion within the limit (see kKneeMargin); returns max(scale, that). hipAnkle = distance
// from the thigh to the ankle target before the warp, along = the ankle's offset from the thigh along the stride direction (the warp
// changes the squared distance by along^2 (scale^2 - 1)). Degenerate inputs return the scale unchanged.
inline float KneeLimitedScale(float scale, float along, float hipAnkle, float thigh, float shin, float margin = kKneeMargin,
                              float freeFlexion = kKneeFreeFlexion) noexcept
{
    if (!(scale < 1.0f) || !std::isfinite(along) || !std::isfinite(hipAnkle) || !(thigh > 1.0e-4f) || !(shin > 1.0e-4f) ||
        !std::isfinite(thigh) || !std::isfinite(shin) || !(along * along > 1.0e-8f))
        return scale;
    const float before = KneeFlexion(hipAnkle, thigh, shin);
    const float limit = std::fmax(before + std::fmax(margin, 0.0f), freeFlexion);
    if (!(limit < 3.14159265f))
        return scale;
    const float minSq = thigh * thigh + shin * shin + 2.0f * thigh * shin * std::cos(limit);
    const float room = hipAnkle * hipAnkle - minSq; // squared distance the warp may take
    if (!(room > 0.0f))
        return 1.0f;
    const float sq = 1.0f - room / (along * along);
    return sq > 0.0f ? std::fmax(scale, std::sqrt(sq)) : scale;
}

inline bool IsIdentity(float scale) noexcept
{
    return !(std::fabs(1.0f - scale) > kIdentityEps);
}

// Horizontal unit stride direction from the root velocity in the character frame (x right, y forward); false when it has no length.
inline bool Direction(float rootX, float rootY, math::Vec3 &out) noexcept
{
    const float len = std::hypot(rootX, rootY);
    if (!std::isfinite(len) || len <= 1.0e-6f)
        return false;
    out = math::Vec3{rootX / len, rootY / len, 0.0f};
    return true;
}

// Offset that scales the ankle target along the horizontal unit direction about the origin (the thigh); zero across the direction.
inline math::Vec3 WarpOffset(math::Vec3 ankle, math::Vec3 origin, math::Vec3 direction, float scale) noexcept
{
    const float along = math::Dot(ankle - origin, direction);
    return direction * ((scale - 1.0f) * along);
}

// Ankle target scaled along the horizontal unit direction about the origin (the thigh); the other components are kept.
inline math::Vec3 WarpPoint(math::Vec3 ankle, math::Vec3 origin, math::Vec3 direction, float scale) noexcept
{
    return ankle + WarpOffset(ankle, origin, direction, scale);
}
} // namespace utpp::mm::stridewarp

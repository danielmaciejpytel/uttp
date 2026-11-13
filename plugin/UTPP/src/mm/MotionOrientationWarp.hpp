// Build-only planar prototype of orientation warping (Graph mode).
// This is intentionally not wired into MMDriver/BlendStack and does not edit CP bone transforms.
#pragma once

#include <algorithm>
#include <cmath>

namespace utpp::mm::motionow
{
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kMinRootMotionSpeed = 0.10f; // UE source pin: 10 cm/s
inline constexpr float kAngleThreshold = 135.0f * kPi / 180.0f;
inline constexpr float kRotationInterpSpeed = 8.0f;
inline constexpr float kCounterInterpSpeed = 45.0f;
inline constexpr float kMaxCounterDelta = 45.0f * kPi / 180.0f;
inline constexpr float kMaxCorrection = kPi;

struct Inputs
{
    float dt = 0.0f;
    float rootdeltaX = 0.0f; // planar animation root-motion delta, metres, animation/root frame
    float rootdeltaY = 0.0f;
    float rootdeltaYaw = 0.0f; // radians; preserved by this node
    float previousVisualWorldYaw = 0.0f; // maps world locomotion vector into the active warp-space planar frame
    float worldlocomotionX = 0.0f; // m/s, world frame
    float worldlocomotionY = 0.0f;
    float fallbackLocalLocomotionX = 0.0f; // CP warp-frame fallback; caller supplies projected mesh/component forward
    float fallbackLocalLocomotionY = 1.0f; // CP convention: forward is +Y
    float curvealpha = 0.0f; // Enable_OrientationWarping curve sampled at current animation time
};

struct Result
{
    float warpedRootdeltaX = 0.0f;
    float warpedRootdeltaY = 0.0f;
    float rootdeltaYaw = 0.0f;
    float actualPoseAngle = 0.0f; // radians; includes curve alpha and UE distribution alpha (0.5)
    float targetAngle = 0.0f; // radians before interpolation/curve weighting
    bool valid = false;
    bool active = false;
};

struct State
{
    bool firstUpdate = true;
    float prevRootDirectionX = 0.0f;
    float prevRootDirectionY = 0.0f;
    float prevRootRotationYaw = 0.0f;
    float counterAngle = 0.0f;
    float actualAngle = 0.0f;
};

inline float WrapRadians(float a)
{
    return std::remainder(a, 2.0f * kPi);
}

inline float SignedAngle(float fromX, float fromY, float toX, float toY)
{
    return std::atan2(fromX * toY - fromY * toX, fromX * toX + fromY * toY);
}

inline float ScalarInterpTo(float current, float target, float dt, float speed)
{
    const float delta = target - current;
    if (delta * delta < 1.0e-8f)
        return target;
    return current + delta * std::clamp(dt * speed, 0.0f, 1.0f);
}

inline Result Step(State& state, const Inputs& in)
{
    Result out{in.rootdeltaX, in.rootdeltaY, in.rootdeltaYaw};
    const bool finite = std::isfinite(in.dt) && std::isfinite(in.rootdeltaX) && std::isfinite(in.rootdeltaY) &&
        std::isfinite(in.rootdeltaYaw) && std::isfinite(in.previousVisualWorldYaw) &&
        std::isfinite(in.worldlocomotionX) && std::isfinite(in.worldlocomotionY) &&
        std::isfinite(in.fallbackLocalLocomotionX) && std::isfinite(in.fallbackLocalLocomotionY) &&
        std::isfinite(in.curvealpha);
    if (!finite || in.dt < 0.0f)
        return out;

    out.valid = true;
    const float alpha = std::clamp(in.curvealpha, 0.0f, 1.0f);
    if (alpha <= 1.0e-6f || in.dt <= 0.0f)
        return out;

    const float c = std::cos(in.previousVisualWorldYaw);
    const float s = std::sin(in.previousVisualWorldYaw);
    // Inverse planar yaw: world velocity -> selected visual/root warp frame.
    float locomotionX = c * in.worldlocomotionX + s * in.worldlocomotionY;
    float locomotionY = -s * in.worldlocomotionX + c * in.worldlocomotionY;
    float locomotionLen = std::hypot(locomotionX, locomotionY);
    if (locomotionLen > 1.0e-8f)
    {
        locomotionX /= locomotionLen;
        locomotionY /= locomotionLen;
    }
    else
    {
        // A zero world vector uses a caller-projected component/mesh forward in the CP warp frame.
        // An all-zero fallback remains zero and therefore retains targetAngle=0 rather than guessing an axis.
        locomotionX = in.fallbackLocalLocomotionX;
        locomotionY = in.fallbackLocalLocomotionY;
        locomotionLen = std::hypot(locomotionX, locomotionY);
        if (locomotionLen > 1.0e-8f)
        {
            locomotionX /= locomotionLen;
            locomotionY /= locomotionLen;
        }
    }

    const float rootLen = std::hypot(in.rootdeltaX, in.rootdeltaY);
    const float speed = rootLen / in.dt;
    const float previousRootRotationYaw = state.prevRootRotationYaw;
    float target = 0.0f;
    float rootDirX = state.prevRootDirectionX;
    float rootDirY = state.prevRootDirectionY;
    bool hasRootDirection = rootLen > 1.0e-8f;
    if (hasRootDirection)
    {
        rootDirX = in.rootdeltaX / rootLen;
        rootDirY = in.rootdeltaY / rootLen;
    }

    if (speed >= kMinRootMotionSpeed && locomotionLen > 1.0e-8f && hasRootDirection)
    {
        target = SignedAngle(rootDirX, rootDirY, locomotionX, locomotionY);
        if (std::fabs(target) > kAngleThreshold)
        {
            target = WrapRadians(target + kPi);
            rootDirX = -rootDirX;
            rootDirY = -rootDirY;
        }
    }
    out.targetAngle = target;

    // UE counter-compensates interpolation for smooth root-motion heading changes, excluding pivots.
    if (speed >= kMinRootMotionSpeed && hasRootDirection && !state.firstUpdate &&
        std::hypot(state.prevRootDirectionX, state.prevRootDirectionY) > 1.0e-8f)
    {
        const float pc = std::cos(previousRootRotationYaw);
        const float ps = std::sin(previousRootRotationYaw);
        const float previousUnrotatedX = pc * state.prevRootDirectionX + ps * state.prevRootDirectionY;
        const float previousUnrotatedY = -ps * state.prevRootDirectionX + pc * state.prevRootDirectionY;
        const float rootMotionAngle = SignedAngle(rootDirX, rootDirY, previousUnrotatedX, previousUnrotatedY);
        if (std::fabs(rootMotionAngle) < kMaxCounterDelta)
        {
            state.counterAngle += rootMotionAngle;
            const float counterStep = std::clamp(in.dt * kCounterInterpSpeed, 0.0f, 1.0f) * state.counterAngle;
            state.actualAngle = WrapRadians(state.actualAngle + counterStep);
            state.counterAngle -= counterStep;
        }
    }

    if (state.firstUpdate)
    {
        state.actualAngle = target;
        state.firstUpdate = false;
    }
    else
    {
        // UE uses scalar FInterpTo here: it does not take the shortest wrapped arc across +/-180 degrees.
        const float smooth = ScalarInterpTo(state.actualAngle, target, in.dt, kRotationInterpSpeed);
        const float maxStep = kMaxCorrection;
        state.actualAngle += std::clamp(smooth - state.actualAngle, -maxStep, maxStep);
    }
    state.actualAngle = std::clamp(state.actualAngle, -kMaxCorrection, kMaxCorrection);
    // ActualAlpha is applied to the state itself in UE (not only to this frame's bone output).
    state.actualAngle *= alpha;
    if (speed >= kMinRootMotionSpeed && hasRootDirection)
    {
        state.prevRootDirectionX = rootDirX;
        state.prevRootDirectionY = rootDirY;
    }
    // UE stores root-motion rotation each evaluated frame, including below-threshold frames.
    state.prevRootRotationYaw = in.rootdeltaYaw;

    // UE's graph side effect rotates planar root translation by target angle; it leaves root yaw intact.
    // Skeletal-control alpha weights the emitted pose angle; at alpha==0 the node is inactive and returns above.
    const float warpedX = std::cos(target) * in.rootdeltaX - std::sin(target) * in.rootdeltaY;
    const float warpedY = std::sin(target) * in.rootdeltaX + std::cos(target) * in.rootdeltaY;
    out.warpedRootdeltaX = warpedX;
    out.warpedRootdeltaY = warpedY;
    out.actualPoseAngle = state.actualAngle * 0.5f;
    out.active = true;
    return out;
}
} // namespace utpp::mm::motionow

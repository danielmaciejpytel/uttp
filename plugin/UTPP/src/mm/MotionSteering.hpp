// Offline planar per-player steering. Follows the behaviour of UE5.8.3
// AnimNode_Steering and Core SpringMath, restricted to yaw in radians/metres.
#pragma once
#include "MMMath.hpp"
#include <algorithm>
namespace utpp::mm::motionSteer
{
struct Inputs
{
    float dt = 0, animationYawDelta = 0, rootSpeedMs = 0;
    float previousVisualWorldYaw = 0, targetWorldYaw = 0, predictedAuthoredYaw = .0f;
    float alpha = 1;
    bool enabled = true, havePrediction = true, turnInPlace = false;
};
struct Result
{
    float yawDelta = 0, authoredRatio = 1, proceduralYaw = 0, residualYaw = 0;
    bool active = false, valid = true;
};
class Steering
{
  public:
    void Reset()
    {
        m_velocity = 0;
        m_alpha = m_alphaFrom = m_alphaElapsed = 0;
        m_alphaInitialized = false;
        m_alphaTarget = false;
    }
    float AngularVelocity() const
    {
        return m_velocity;
    }
    float Alpha() const
    {
        return m_alpha;
    }
    // FInputAlphaBoolBlend contract: first evaluated target snaps; later bool
    // changes rebase from the current value over the configured .1s linear fade.
    // Invalid dt (negative/nonfinite) leaves all alpha state unchanged. Valid dt0
    // may initialize/rebase the target, but does not advance an existing fade.
    float UpdateEnable(bool target, float dt)
    {
        if (!std::isfinite(dt) || dt < 0)
            return m_alpha;
        if (!m_alphaInitialized)
        {
            m_alphaInitialized = true;
            m_alphaTarget = target;
            m_alpha = m_alphaFrom = target ? 1.f : 0.f;
            m_alphaElapsed = 0;
            return m_alpha;
        }
        if (target != m_alphaTarget)
        {
            m_alphaTarget = target;
            m_alphaFrom = m_alpha;
            m_alphaElapsed = 0;
        }
        m_alphaElapsed = std::fmin(.1f, m_alphaElapsed + dt);
        const float end = target ? 1.f : 0.f;
        m_alpha = m_alphaElapsed >= .1f ? end : m_alphaFrom + (end - m_alphaFrom) * (m_alphaElapsed / .1f);
        return m_alpha;
    }
    Result Update(const Inputs &in)
    {
        Result out;
        out.yawDelta = in.animationYawDelta;
        if (!std::isfinite(in.dt) || in.dt < 0 || !std::isfinite(in.animationYawDelta) ||
            !std::isfinite(in.rootSpeedMs) || !std::isfinite(in.alpha) || !std::isfinite(in.previousVisualWorldYaw) ||
            !std::isfinite(in.targetWorldYaw) || (in.havePrediction && !std::isfinite(in.predictedAuthoredYaw)))
        {
            out.valid = false;
            return out;
        }
        const float alpha = std::clamp(in.alpha, 0.f, 1.f);
        // Native guards leave retained angular velocity untouched while inactive.
        const float minimumSpeed = in.turnInPlace ? -.01f : .10f;
        if (in.dt == 0 || !in.enabled || alpha <= 0 || !(in.rootSpeedMs > minimumSpeed))
            return out;
        out.active = true;
        float residual = math::WrapAngle(in.targetWorldYaw - in.previousVisualWorldYaw);
        const float prediction = math::WrapAngle(in.predictedAuthoredYaw);
        if (in.havePrediction && std::fabs(prediction) > math::kPi / 180.f)
        {
            float nearest = residual;
            if (prediction - nearest > math::kPi)
                nearest += 2 * math::kPi;
            else if (nearest - prediction > math::kPi)
                nearest -= 2 * math::kPi;
            const float ratio = std::clamp(nearest / prediction, .5f, 1.5f);
            out.authoredRatio = 1 + (ratio - 1) * alpha;
            out.yawDelta *= out.authoredRatio;
            residual = math::WrapAngle(residual - prediction * out.authoredRatio);
        }
        out.residualYaw = residual;
        // CriticalSpringDamperQuat starts the correction at identity EACH frame;
        // only angular velocity persists. HalfLifeToSmoothingTime=h/ln2;
        // Y=(4/smoothingTime)/2=2*ln2/h. All angles/velocity radians, rad/s.
        if (in.rootSpeedMs > -.01f)
        {
            const float halfLife = in.turnInPlace ? 1e6f : .2f;
            const float y = 2 * math::kLn2 / halfLife;
            const float j0 = -residual, j1 = m_velocity + j0 * y, x = y * in.dt;
            const float e = 1 / (1 + 1.00746054f * x + .45053901f * x * x + .25724632f * x * x * x);
            const float correction = math::WrapAngle(e * (j0 + j1 * in.dt) + residual);
            m_velocity = e * (m_velocity - j1 * y * in.dt);
            out.proceduralYaw = correction * alpha;
            out.yawDelta = math::WrapAngle(out.yawDelta + out.proceduralYaw);
        }
        if (!std::isfinite(out.yawDelta) || !std::isfinite(m_velocity))
        {
            out.valid = false;
            out.yawDelta = in.animationYawDelta;
            m_velocity = 0;
        }
        return out;
    }

  private:
    float m_velocity = 0, m_alpha = 0, m_alphaFrom = 0, m_alphaElapsed = 0;
    bool m_alphaInitialized = false, m_alphaTarget = false;
};
} // namespace utpp::mm::motionSteer

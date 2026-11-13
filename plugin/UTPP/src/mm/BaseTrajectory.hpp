// UTPP motion matching runtime - planar trajectory simulation and prediction. Header-only, allocation-free.
//
// The simulated object is the character frame (= the Trajectory bone of the database, Z up, forward +Y, yaw CCW about
// +Z). Its state lives in a re-based world frame: after every Update() the current position is moved back to the
// origin and the yaw is wrapped to (-pi, pi], so precision does not degrade over long sessions. Only relative
// quantities leave this class.
//
// Inputs are expressed in the CURRENT character frame (x right, y forward) at the time of the Update() call:
//  - desired velocity (m/s). FacingMode::Direction: converted once to world and treated as world-fixed for the whole
//    prediction horizon (camera-relative stick semantics, like orangeduck's non-strafe controller).
//    FacingMode::YawRate: the desired velocity turns together with the desired facing (curved path).
//  - FacingMode::Direction: desired facing unit xy (length < 1e-3 = keep the previous world goal).
//    FacingMode::YawRate: desired yaw rate (rad/s, CCW positive) integrated into a world goal yaw.
//
// Simulation (orangeduck controller.cpp simulation_positions_update / simulation_rotations_update): critically damped
// velocity spring with acceleration state per planar axis (half-life velocityHalflife) and critically damped yaw
// spring (half-life rotationHalflife). Prediction re-runs the same springs from the current state in fixed steps of
// 1/fps up to the largest horizon.
//
// Features (bit-for-bit the definition in Source/tools/mmbake/mmbake_base.py process_clip, FORMAT.txt):
//   trajectory_position[k]  = R(-yaw_now) * (p(t + h_k) - p_now)          (x, y), k over horizons
//   trajectory_direction[k] = (-sin d, cos d), d = yaw(t + h_k) - yaw_now  (+Y forward rotated by d)
#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "MMMath.hpp"

namespace utpp::mm
{
enum class FacingMode : int32_t
{
    Direction = 0,
    YawRate = 1,
};

struct TrajectoryInput
{
    float desiredVelocity[2] = {0.0f, 0.0f}; // m/s, current character frame
    FacingMode facingMode = FacingMode::Direction;
    float desiredFacing[2] = {0.0f, 1.0f}; // unit xy, current character frame (Direction mode)
    float yawRate = 0.0f;                  // rad/s CCW (YawRate mode)
    // Phase 5.1 external facing: the character yaw is driven from outside (UTPP body-yaw command -> game entity).
    // When set, the simulated yaw is NOT sprung towards the goal; it is advanced by the measured step and its angular
    // velocity is set to the measured rate, so the prediction starts from the real body state. The step is applied
    // BEFORE the goals are converted, i.e. desiredVelocity / desiredFacing are in the character frame after the step.
    bool externalYaw = false;
    float externalYawStep = 0.0f; // rad, measured body yaw change since the previous update (wrapped to [-pi, pi])
    float externalYawRate = 0.0f; // rad/s (in the time base of aDt)
    // Phase 5.3c (0.3.5) curvature (Direction mode only): the goal facing and the goal velocity direction keep turning at
    // this rate (rad/s CCW, time base of aDt) over the prediction horizon, so a steady arc / circle predicts a curved path
    // instead of "turn to the goal, then straight". The yaw / velocity springs lag a ramp goal by halflife / ln 2; the
    // goals are led by that lag so the predicted yaw turns at exactly turnRate in steady state. 0 = 0.3.4 bit for bit.
    float turnRate = 0.0f;
    // 0.3.8: zero the simulated angular velocity before this update (YawRate mode after the body-turn path ended, e.g. the
    // stick passing through the centre during a flick: the externally seeded body turn rate (up to ~5 rad/s matcher time)
    // was carried into the stop prediction -> Walk_Stop at cost 422 in game). false = 0.3.7.
    bool clearAngVel = false;
};

// Raw trajectory features from future samples (character frame of "now"). Shared by the runtime and the tests.
inline void TrajectoryFeaturesFromSamples(float aPx, float aPy, float aYaw, const float* aFx, const float* aFy, const float* aFyaw,
                                          uint32_t aNum, float* aOutPos, float* aOutDir)
{
    for (uint32_t k = 0; k < aNum; ++k)
    {
        float rx = 0.0f, ry = 0.0f;
        math::Rot2(-aYaw, aFx[k] - aPx, aFy[k] - aPy, rx, ry);
        const float d = aFyaw[k] - aYaw;
        aOutPos[2 * k] = rx;
        aOutPos[2 * k + 1] = ry;
        aOutDir[2 * k] = -std::sin(d);
        aOutDir[2 * k + 1] = std::cos(d);
    }
}

class Trajectory
{
public:
    static constexpr uint32_t kMaxHorizons = 4;
    static constexpr uint32_t kHistorySize = 32; // samples at 1/fps (~1 s at 30 fps)
    static constexpr float kMaxTurnRate = 6.0f;  // rad/s, clamp of TrajectoryInput::turnRate

    struct HistorySample
    {
        float x = 0.0f; // character frame of "now"
        float y = 0.0f;
        float yaw = 0.0f; // relative to "now"
    };

    // Phase 5.3b (0.3.4): velocity half-life used while the goal velocity is (near) zero, i.e. a stop. The game capsule
    // brakes at ~23 m/s^2 (0.17 s from run speed), far faster than the 0.27 s spring predicts, so the predicted stop
    // point was ~2 m ahead and the stop clips were reached late. 0 = off (default, identical to 0.3.3).
    bool SetStopHalflife(float aHalflife)
    {
        if (!std::isfinite(aHalflife) || aHalflife < 0.0f || aHalflife > 5.0f)
            return false;
        m_stopHalflife = aHalflife;
        return true;
    }
    // 0.5.9 (UTPPSetMMTrajCal): velocity half-life without re-init (calibrated per locomotion state from the capsule profile)
    bool SetVelocityHalflife(float aHalflife)
    {
        if (!std::isfinite(aHalflife) || aHalflife < 0.02f || aHalflife > 5.0f)
            return false;
        m_velHalflife = aHalflife;
        return true;
    }
    float VelocityHalflife() const { return m_velHalflife; }

    bool Init(const int32_t* aHorizonFrames, uint32_t aNumHorizons, float aFps, float aVelocityHalflife, float aRotationHalflife)
    {
        m_ok = false;
        if (!aHorizonFrames || aNumHorizons == 0 || aNumHorizons > kMaxHorizons || !(aFps > 0.0f) || !std::isfinite(aFps) ||
            !(aVelocityHalflife > 0.0f) || !(aRotationHalflife > 0.0f))
            return false;
        int32_t prev = 0;
        for (uint32_t k = 0; k < aNumHorizons; ++k)
        {
            if (aHorizonFrames[k] <= prev || aHorizonFrames[k] > 600)
                return false;
            prev = aHorizonFrames[k];
            m_horizons[k] = aHorizonFrames[k];
        }
        m_numHorizons = aNumHorizons;
        m_fps = aFps;
        m_velHalflife = aVelocityHalflife;
        m_rotHalflife = aRotationHalflife;
        m_ok = true;
        Reset();
        return true;
    }

    // At rest, facing +Y. Clears history.
    void Reset()
    {
        m_px = m_py = m_yaw = 0.0f;
        m_vx = m_vy = m_ax = m_ay = 0.0f;
        m_angVel = 0.0f;
        m_desiredYaw = 0.0f;
        m_goalVx = m_goalVy = 0.0f;
        m_stepX = m_stepY = m_stepYaw = 0.0f;
        m_prevYaw = m_prevYawBeforeStep = 0.0f;
        m_lastInput = TrajectoryInput{};
        m_history = {};
        m_historyCount = 0;
        m_historyHead = 0;
        m_historyAcc = 0.0f;
        Predict();
    }

    // Overrides the simulated motion state (e.g. sync with the measured capsule velocity). Velocity / acceleration in
    // the current character frame; the facing goal is reset to the current facing.
    void SetState(float aVelX, float aVelY, float aAccX, float aAccY, float aYawRate)
    {
        auto fin = [](float v) { return std::isfinite(v) ? v : 0.0f; };
        math::Rot2(m_yaw, fin(aVelX), fin(aVelY), m_vx, m_vy);
        math::Rot2(m_yaw, fin(aAccX), fin(aAccY), m_ax, m_ay);
        m_angVel = fin(aYawRate);
        m_desiredYaw = m_yaw;
        Predict();
    }

    void Update(float aDt, const TrajectoryInput& aIn)
    {
        if (!m_ok)
            return;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        TrajectoryInput in = Sanitize(aIn);
        m_lastInput = in;
        if (in.externalYaw)
        {
            m_yaw += in.externalYawStep;
            m_angVel = in.externalYawRate;
        }
        else if (in.clearAngVel)
            m_angVel = 0.0f;

        // goals in the (re-based) world frame
        math::Rot2(m_yaw, in.desiredVelocity[0], in.desiredVelocity[1], m_goalVx, m_goalVy);
        if (in.facingMode == FacingMode::Direction && in.turnRate != 0.0f)
        {
            // 5.3c: lead the velocity goal by the spring lag so the simulated velocity stays aligned with the turning body
            const float gx = m_goalVx, gy = m_goalVy;
            math::Rot2(in.turnRate * VelHalflife(gx, gy) / 0.69314718f, gx, gy, m_goalVx, m_goalVy);
        }
        if (in.facingMode == FacingMode::Direction)
        {
            const float fl = std::sqrt(in.desiredFacing[0] * in.desiredFacing[0] + in.desiredFacing[1] * in.desiredFacing[1]);
            if (fl > 1e-3f)
                m_desiredYaw = m_yaw + std::atan2(-in.desiredFacing[0], in.desiredFacing[1]);
        }
        else
        {
            m_desiredYaw += in.yawRate * dt;
        }

        if (dt > 0.0f)
        {
            const float vh = VelHalflife(m_goalVx, m_goalVy);
            math::SimulationAxisUpdate(m_px, m_vx, m_ax, m_goalVx, vh, dt);
            math::SimulationAxisUpdate(m_py, m_vy, m_ay, m_goalVy, vh, dt);
            if (!in.externalYaw)
                math::SimpleSpringDamperExact(m_yaw, m_angVel, m_desiredYaw, m_rotHalflife, dt);
            m_stepX = m_px;
            m_stepY = m_py;
        }
        else
        {
            m_stepX = m_stepY = 0.0f;
        }
        m_stepYaw = m_yaw - m_prevYaw;

        // history at fixed 1/fps samples (stores the state at the end of the update that crossed a sample boundary)
        m_historyAcc += dt;
        const float sampleDt = 1.0f / m_fps;
        if (m_historyAcc >= sampleDt || m_historyCount == 0)
        {
            m_historyAcc = std::fmod(m_historyAcc, sampleDt);
            m_history[m_historyHead] = {m_px, m_py, m_yaw};
            m_historyHead = (m_historyHead + 1) % kHistorySize;
            if (m_historyCount < kHistorySize)
                ++m_historyCount;
        }
        Rebase();
        Predict();
    }

    // ---- results (character frame of "now")
    uint32_t NumHorizons() const { return m_numHorizons; }
    const float* PositionFeatures() const { return m_featPos.data(); } // 2 * NumHorizons()
    const float* DirectionFeatures() const { return m_featDir.data(); }
    // Current simulated velocity / yaw rate in the character frame.
    void LocalVelocity(float& aX, float& aY) const { math::Rot2(-m_yaw, m_vx, m_vy, aX, aY); }
    float YawRate() const { return m_angVel; }
    // World-space (re-based) desired goals, used for the forced-search test.
    float GoalVelocityX() const { return m_goalVx; }
    float GoalVelocityY() const { return m_goalVy; }
    float DesiredYawRelative() const { return m_desiredYaw - m_yaw; }
    float WorldYaw() const { return m_yaw; } // (-pi, pi]
    // Planar step of the last Update(), expressed in the character frame at the START of that update.
    void LastStep(float& aDx, float& aDy, float& aDyaw) const
    {
        math::Rot2(-m_prevYawBeforeStep, m_stepX, m_stepY, aDx, aDy);
        aDyaw = m_stepYaw;
    }

    // aAge 0 = newest sample. Returned in the current character frame.
    HistorySample History(uint32_t aAge) const
    {
        HistorySample s{};
        if (aAge >= m_historyCount)
            return s;
        const HistorySample& h = m_history[(m_historyHead + kHistorySize - 1 - aAge) % kHistorySize];
        math::Rot2(-m_yaw, h.x - m_px, h.y - m_py, s.x, s.y);
        s.yaw = h.yaw - m_yaw;
        return s;
    }

private:
    float VelHalflife(float aGoalVx, float aGoalVy) const
    {
        return (m_stopHalflife > 0.0f && aGoalVx * aGoalVx + aGoalVy * aGoalVy < 1e-6f) ? m_stopHalflife : m_velHalflife;
    }

    static TrajectoryInput Sanitize(const TrajectoryInput& a)
    {
        TrajectoryInput in = a;
        auto fin = [](float& v) {
            if (!std::isfinite(v))
                v = 0.0f;
        };
        fin(in.desiredVelocity[0]);
        fin(in.desiredVelocity[1]);
        fin(in.desiredFacing[0]);
        fin(in.desiredFacing[1]);
        fin(in.yawRate);
        fin(in.externalYawStep);
        fin(in.externalYawRate);
        fin(in.turnRate);
        in.turnRate = std::fmax(-kMaxTurnRate, std::fmin(kMaxTurnRate, in.turnRate));
        in.externalYawStep = std::fmax(-math::kPi, std::fmin(math::kPi, in.externalYawStep));
        in.externalYawRate = std::fmax(-30.0f, std::fmin(30.0f, in.externalYawRate));
        if (in.facingMode != FacingMode::Direction && in.facingMode != FacingMode::YawRate)
            in.facingMode = FacingMode::Direction;
        return in;
    }

    // Moves the current position to the origin and wraps yaw (all stored world quantities shift together).
    void Rebase()
    {
        const float ox = m_px, oy = m_py;
        float wrap = 0.0f;
        if (m_yaw > math::kPi || m_yaw <= -math::kPi)
            wrap = m_yaw - math::WrapAngle(m_yaw);
        for (HistorySample& h : m_history)
        {
            h.x -= ox;
            h.y -= oy;
            h.yaw -= wrap;
        }
        m_px = 0.0f;
        m_py = 0.0f;
        m_yaw -= wrap;
        m_desiredYaw -= wrap;
        m_prevYawBeforeStep = m_prevYaw;
        m_prevYaw = m_yaw;
    }

    void Predict()
    {
        float px = m_px, py = m_py, vx = m_vx, vy = m_vy, ax = m_ax, ay = m_ay, yaw = m_yaw, w = m_angVel;
        const float step = 1.0f / m_fps;
        std::array<float, kMaxHorizons> fx{}, fy{}, fyaw{};
        uint32_t k = 0;
        const int32_t last = m_numHorizons ? m_horizons[m_numHorizons - 1] : 0;
        const bool yawRate = m_lastInput.facingMode == FacingMode::YawRate;
        const float curve = yawRate ? 0.0f : m_lastInput.turnRate; // 5.3c curvature (Direction mode)
        constexpr float kLn2 = 0.69314718f;
        for (int32_t i = 1; i <= last && k < m_numHorizons; ++i)
        {
            float gvx = m_goalVx, gvy = m_goalVy, gyaw = m_desiredYaw;
            if (yawRate)
            {
                const float turn = m_lastInput.yawRate * step * static_cast<float>(i);
                gyaw += turn;
                math::Rot2(turn, m_goalVx, m_goalVy, gvx, gvy);
            }
            else if (curve != 0.0f)
            {
                const float ti = step * static_cast<float>(i);
                gyaw += curve * (ti + m_rotHalflife / kLn2);
                math::Rot2(curve * ti, m_goalVx, m_goalVy, gvx, gvy); // m_goalV already led by the spring lag (Update)
            }
            const float pvh = VelHalflife(gvx, gvy);
            math::SimulationAxisUpdate(px, vx, ax, gvx, pvh, step);
            math::SimulationAxisUpdate(py, vy, ay, gvy, pvh, step);
            math::SimpleSpringDamperExact(yaw, w, gyaw, m_rotHalflife, step);
            if (i == m_horizons[k])
            {
                fx[k] = px;
                fy[k] = py;
                fyaw[k] = yaw;
                ++k;
            }
        }
        TrajectoryFeaturesFromSamples(m_px, m_py, m_yaw, fx.data(), fy.data(), fyaw.data(), m_numHorizons, m_featPos.data(),
                                      m_featDir.data());
    }

    bool m_ok = false;
    std::array<int32_t, kMaxHorizons> m_horizons{};
    uint32_t m_numHorizons = 0;
    float m_fps = 30.0f;
    float m_velHalflife = 0.27f;
    float m_stopHalflife = 0.0f; // Phase 5.3b: velocity half-life while the goal velocity is zero (0 = m_velHalflife)
    float m_rotHalflife = 0.27f;

    float m_px = 0.0f, m_py = 0.0f, m_yaw = 0.0f;
    float m_vx = 0.0f, m_vy = 0.0f, m_ax = 0.0f, m_ay = 0.0f;
    float m_angVel = 0.0f;
    float m_desiredYaw = 0.0f;
    float m_goalVx = 0.0f, m_goalVy = 0.0f;
    float m_stepX = 0.0f, m_stepY = 0.0f, m_stepYaw = 0.0f;
    float m_prevYaw = 0.0f, m_prevYawBeforeStep = 0.0f;
    TrajectoryInput m_lastInput{};

    std::array<HistorySample, kHistorySize> m_history{};
    uint32_t m_historyCount = 0;
    uint32_t m_historyHead = 0;
    float m_historyAcc = 0.0f;

    std::array<float, 2 * kMaxHorizons> m_featPos{};
    std::array<float, 2 * kMaxHorizons> m_featDir{};
};

} // namespace utpp::mm

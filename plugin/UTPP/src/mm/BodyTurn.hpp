// UTPP Phase 5.1 - body yaw follows the motion matching animation. Header-only, allocation-free, deterministic.
//
// 1. DesiredMotion: builds the matcher's desired velocity / facing from one locomotion feed. The legacy path
//    (BodyTurn off) is the Phase 4 code moved here unchanged (input-driven, body- or camera-relative stick, facing from
//    the measured yaw rate); the body-turn path uses the world target direction (camera yaw + Locomotion raw stick
//    angle) as both the desired facing (FacingMode::Direction) and the desired velocity direction, so the matcher sees
//    the whole remaining turn and selects pivot / turn clips.
// 2. BodyYawController: per call, next body yaw = current + max(animation root yaw step towards the target, bounded
//    fallback step), never past the target and never faster than maxRate. The animation drives the turn whenever it
//    turns the right way at least as fast as the fallback; the fallback (exponential with rate floor and cap) guarantees
//    convergence if the playing clip does not turn (enough). Rotation of the animation away from the target is ignored.
//
// Yaw convention as GameIntegration.hpp: radians, CCW about +Z from world +Y, forward = (-sin, cos).
#pragma once

#include <cmath>
#include <cstdint>

#include "BaseTrajectory.hpp"

namespace utpp::mm::bt
{
inline constexpr float kPi = 3.14159265358979323846f;

inline float Wrap(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::remainder(a, 2.0f * kPi);
    return a <= -kPi ? a + 2.0f * kPi : a;
}

// ------------------------------------------------------------------------------------------------ desired motion

struct MotionParams
{
    bool inputDriven = true;
    bool inputCameraRelative = false;
    float stateTargetSpeed = 0.0f; // game m/s of the current locomotion state (run 3.85, sprint 8.2, crouch 2.7)
    // Phase 5.3: floor of the desired speed while the stick is held, as a fraction of the state target. 0.5 = 0.2.x-0.3.2
    // behaviour; the driver uses 1.0 (pure input intent, motion-like: the trajectory spring shapes the ramp and start
    // clips are selected) while the capsule acceleration is matched to the clips (slower capsule ramp).
    float intentFraction = 0.5f;
    // Phase 5.3b (0.3.4): with the stick released the desired velocity is zero (stop intent, motion-like) instead of the
    // decelerating capsule velocity, so the trajectory predicts the stop at once and stop clips are reached in time.
    bool stopIntent = false;
    // Phase 5.3b (0.3.4): start reface. Starting (capsule < refaceCapsuleSpeed) towards a target far behind the body, the
    // desired speed is scaled by 0.25 + 0.75 * max(0, cos(angle)), so the query asks for a turn-on-the-spot / reface start
    // instead of full speed backwards (which selected running 180 turns / pivots at start in game).
    bool startReface = false;
    float refaceCapsuleSpeed = 1.0f;
};

struct MotionFrame
{
    float charVel[2] = {0.0f, 0.0f}; // measured capsule velocity, character frame (m/s)
    float move[2] = {0.0f, 0.0f};    // MoveX/MoveY (finite)
    float cameraYaw = 0.0f;          // rad; non-finite -> body yaw
    float bodyYaw = 0.0f;            // rad
    bool bodyTurn = false;           // Phase 5.1 path
    float targetYaw = 0.0f;          // rad, world, valid when bodyTurn
    // Phase 5.3c (0.3.5) steering follow mode (bodyTurn only): the body follows the target with a rate-limited spring, the
    // matcher's facing / velocity direction is the predicted body yaw (facingYaw, world) and the path curves at turnRate
    // (rad/s real time). false = 0.3.4 (facing = target).
    bool follow = false;
    float facingYaw = 0.0f;
    float turnRate = 0.0f;
};

struct DesiredMotion
{
    float vel[2] = {0.0f, 0.0f};    // character frame, game m/s (before speed mapping)
    float facing[2] = {0.0f, 1.0f}; // character frame unit vector
    bool direction = false;         // true: FacingMode::Direction with facing; false: YawRate (measured)
    float turnRate = 0.0f;          // 5.3c trajectory curvature (rad/s real time), Direction mode only
};

// Legacy (Phase 4, 0.2.6-0.2.8) desired motion - kept bit-for-bit (tests compare against a copy of the old code).
inline DesiredMotion LegacyDesiredMotion(const MotionFrame& f, const MotionParams& p)
{
    DesiredMotion d;
    const float cx = f.charVel[0], cy = f.charVel[1];
    float dx = cx, dy = cy, fx = 0.0f, fy = 1.0f;
    const float mx = f.move[0];
    const float my = f.move[1];
    const float mag = std::sqrt(mx * mx + my * my);
    const float capsule = std::sqrt(cx * cx + cy * cy);
    if (p.inputDriven && mag > 0.15f)
    {
        const float cam = std::isfinite(f.cameraYaw) ? f.cameraYaw : f.bodyYaw;
        float wx = -std::sin(cam) * my + std::cos(cam) * mx;
        float wy = std::cos(cam) * my + std::sin(cam) * mx;
        const float wl = std::sqrt(wx * wx + wy * wy);
        if (wl > 1e-4f)
        {
            wx /= wl;
            wy /= wl;
            float ix = 0.0f, iy = 1.0f;
            if (p.inputCameraRelative)
            {
                const float c = std::cos(f.bodyYaw), s = std::sin(f.bodyYaw);
                ix = c * wx + s * wy;
                iy = -s * wx + c * wy;
            }
            else
            {
                ix = mx / mag;
                iy = my / mag;
            }
            const float target = p.stateTargetSpeed * std::fmin(mag, 1.0f);
            const float sp = std::fmax(capsule, p.intentFraction * target);
            dx = ix * sp;
            dy = iy * sp;
            fx = ix;
            fy = iy;
        }
    }
    d.vel[0] = dx;
    d.vel[1] = dy;
    d.facing[0] = fx;
    d.facing[1] = fy;
    d.direction = p.inputDriven && p.inputCameraRelative;
    return d;
}

// Body-turn desired motion: facing and velocity direction = world target in the current character frame.
inline DesiredMotion BodyTurnDesiredMotion(const MotionFrame& f, const MotionParams& p)
{
    DesiredMotion d;
    const float goal = f.follow ? f.facingYaw : f.targetYaw;
    const float rel = Wrap((std::isfinite(goal) ? goal : f.bodyYaw) - f.bodyYaw);
    if (f.follow && std::isfinite(f.turnRate))
        d.turnRate = f.turnRate;
    const float ix = -std::sin(rel), iy = std::cos(rel);
    const float cx = f.charVel[0], cy = f.charVel[1];
    const float capsule = std::sqrt(cx * cx + cy * cy);
    const float mag = std::sqrt(f.move[0] * f.move[0] + f.move[1] * f.move[1]);
    if (mag > 0.15f)
    {
        float want = p.intentFraction * p.stateTargetSpeed * std::fmin(mag, 1.0f);
        if (p.startReface && capsule < p.refaceCapsuleSpeed)
            want *= 0.25f + 0.75f * std::fmax(0.0f, iy); // iy = cos(target - body)
        const float sp = std::fmax(capsule, want);
        d.vel[0] = ix * sp;
        d.vel[1] = iy * sp;
    }
    else
    {
        d.vel[0] = cx;
        d.vel[1] = cy;
    }
    d.facing[0] = ix;
    d.facing[1] = iy;
    d.direction = true;
    return d;
}

inline DesiredMotion ComputeDesiredMotion(const MotionFrame& f, const MotionParams& p)
{
    DesiredMotion d = f.bodyTurn ? BodyTurnDesiredMotion(f, p) : LegacyDesiredMotion(f, p);
    if (p.stopIntent && std::sqrt(f.move[0] * f.move[0] + f.move[1] * f.move[1]) <= 0.15f)
    {
        d.vel[0] = 0.0f;
        d.vel[1] = 0.0f;
    }
    return d;
}

// ------------------------------------------------------------------------------------------------ target latch

// World target held during a turn (0.3.1). While |held target - body| > turnAngle and the stick angle relative to the
// camera (target - camera) changed by less than stickAngle, the held target is returned, so camera motion during the
// turn cannot drag the target (feedback body -> camera -> target). Otherwise the input target is taken and held.
struct TargetLatch
{
    float turnAngle = 15.0f * kPi / 180.0f;
    float stickAngle = 10.0f * kPi / 180.0f;
    bool valid = false;
    bool latched = false;
    float target = 0.0f;
    float raw = 0.0f;
    uint64_t holds = 0;

    void Reset()
    {
        valid = latched = false;
        target = raw = 0.0f;
    }

    float Update(bool aHave, float aTarget, float aCamera, float aBody)
    {
        if (!aHave || !std::isfinite(aTarget) || !std::isfinite(aCamera) || !std::isfinite(aBody))
        {
            valid = latched = false;
            return aTarget;
        }
        const float r = Wrap(aTarget - aCamera);
        latched = valid && std::fabs(Wrap(target - aBody)) > turnAngle && std::fabs(Wrap(r - raw)) < stickAngle;
        if (latched)
        {
            ++holds;
            return target;
        }
        target = Wrap(aTarget);
        raw = r;
        valid = true;
        return target;
    }
};

// ------------------------------------------------------------------------------------------------ yaw controller

struct ControllerConfig
{
    float correctionTime = 0.4f;     // s, fallback exponential time constant (rate = |err| / time)
    float minCorrectionRate = 0.35f; // rad/s (20 deg/s), fallback floor so the tail finishes
    float maxCorrectionRate = 3.14f; // rad/s (180 deg/s), fallback cap (the animation may be faster)
    float maxRate = 15.7f;           // rad/s (900 deg/s), cap of the total commanded rate
    float animRateHalflife = 0.03f;  // s, smoothing of the animation root yaw rate across clip transitions
    float snapAngle = 0.0017f;       // rad (~0.1 deg), below this the target is returned
    float maxDt = 0.1f;              // s
    // Phase 5.3c (0.3.5) follow mode (continuous steering): rate = |err| / followTime, capped at maxFollowRate, rate
    // change limited to maxFollowAccel; never past the target.
    float followTime = 0.15f;        // s
    float maxFollowRate = 6.2832f;   // rad/s (360 deg/s)
    float maxFollowAccel = 25.0f;    // rad/s^2 (~1430 deg/s^2)
};

inline bool ValidConfig(const ControllerConfig& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    return pos(c.correctionTime) && std::isfinite(c.minCorrectionRate) && c.minCorrectionRate >= 0.0f &&
           pos(c.maxCorrectionRate) && c.maxCorrectionRate >= c.minCorrectionRate && pos(c.maxRate) &&
           std::isfinite(c.animRateHalflife) && c.animRateHalflife >= 0.0f && std::isfinite(c.snapAngle) &&
           c.snapAngle >= 0.0f && pos(c.maxDt) && pos(c.followTime) && pos(c.maxFollowRate) && pos(c.maxFollowAccel);
}

struct ControllerDiagnostics
{
    float error = 0.0f;     // rad, target - current at the last command
    float animStep = 0.0f;  // rad, animation contribution (towards the target, >= 0)
    float fallbackStep = 0.0f;
    float step = 0.0f;      // rad, signed applied step
    float rate = 0.0f;      // rad/s, signed applied rate
    bool animLed = false;   // animation step >= fallback step
    bool follow = false;    // 5.3c: last command used the follow law
    uint64_t followCommands = 0;
    uint64_t commands = 0;
    uint64_t animLedCommands = 0;
    uint64_t nonFinite = 0;
};

class BodyYawController
{
public:
    bool SetConfig(const ControllerConfig& c)
    {
        if (!ValidConfig(c))
            return false;
        m_cfg = c;
        return true;
    }
    const ControllerConfig& Config() const { return m_cfg; }

    void Reset()
    {
        m_animRate = 0.0f;
        m_rate = 0.0f;
        m_diag = ControllerDiagnostics{};
    }

    // Raw root yaw rate of the playing animation in REAL time (rad/s), once per locomotion feed.
    void FeedAnimRate(float aRaw, float aDt)
    {
        const float raw = std::isfinite(aRaw) ? std::fmax(-30.0f, std::fmin(30.0f, aRaw)) : 0.0f;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, m_cfg.maxDt) : 0.0f;
        if (m_cfg.animRateHalflife <= 0.0f)
        {
            m_animRate = raw;
            return;
        }
        const float k = 1.0f - std::exp(-0.69314718f * dt / m_cfg.animRateHalflife);
        m_animRate += (raw - m_animRate) * k;
        if (!std::isfinite(m_animRate))
            m_animRate = 0.0f;
    }
    float AnimRate() const { return m_animRate; }

    // Next body yaw (rad, wrapped) from the current yaw towards aTarget over aDt. aFollow (5.3c): continuous-steering law
    // (spring + rate / acceleration limits) instead of the animation-led law; aFollow false = 0.3.4 bit for bit.
    // aRateCap (AIR STEER stage, rad/s, follow law only): > 0 also caps the applied follow rate (hard, after the acceleration
    // limit); 0 (default) = the configured maxFollowRate only, bit for bit as before.
    float Command(float aCurrent, float aTarget, float aDt, bool aFollow = false, float aRateCap = 0.0f)
    {
        if (aFollow && std::isfinite(aCurrent) && std::isfinite(aTarget))
            return CommandFollow(aCurrent, aTarget, aDt, aRateCap);
        const float out = CommandAnim(aCurrent, aTarget, aDt);
        m_rate = std::isfinite(m_diag.rate) ? m_diag.rate : 0.0f;
        return out;
    }

    const ControllerDiagnostics& Diagnostics() const { return m_diag; }

private:
    float CommandFollow(float aCurrent, float aTarget, float aDt, float aRateCap)
    {
        ++m_diag.commands;
        ++m_diag.followCommands;
        m_diag.follow = true;
        m_diag.animLed = false;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, m_cfg.maxDt) : 0.0f;
        const float err = Wrap(aTarget - aCurrent);
        m_diag.error = err;
        m_diag.animStep = m_diag.fallbackStep = 0.0f;
        if (std::fabs(err) <= m_cfg.snapAngle)
        {
            m_diag.step = err;
            m_diag.rate = dt > 0.0f ? err / dt : 0.0f;
            m_rate = 0.0f;
            return Wrap(aTarget);
        }
        if (dt <= 0.0f)
        {
            m_diag.step = m_diag.rate = 0.0f;
            return Wrap(aCurrent);
        }
        const float want = std::fmax(-m_cfg.maxFollowRate, std::fmin(m_cfg.maxFollowRate, err / m_cfg.followTime));
        const float dv = m_cfg.maxFollowAccel * dt;
        float rate = std::fmax(m_rate - dv, std::fmin(m_rate + dv, want));
        if (rate * err < 0.0f)
            rate = 0.0f; // never turn away from the target
        if (aRateCap > 0.0f && std::isfinite(aRateCap))
            rate = std::fmax(-aRateCap, std::fmin(aRateCap, rate));
        float step = rate * dt;
        if (std::fabs(step) > std::fabs(err))
            step = err; // never past the target
        m_rate = step / dt;
        m_diag.step = step;
        m_diag.rate = m_rate;
        return Wrap(aCurrent + step);
    }

    float CommandAnim(float aCurrent, float aTarget, float aDt)
    {
        ++m_diag.commands;
        m_diag.follow = false;
        if (!std::isfinite(aCurrent) || !std::isfinite(aTarget))
        {
            ++m_diag.nonFinite;
            m_diag.step = m_diag.rate = 0.0f;
            return std::isfinite(aCurrent) ? Wrap(aCurrent) : (std::isfinite(aTarget) ? Wrap(aTarget) : 0.0f);
        }
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, m_cfg.maxDt) : 0.0f;
        const float err = Wrap(aTarget - aCurrent);
        const float a = std::fabs(err);
        m_diag.error = err;
        if (a <= m_cfg.snapAngle)
        {
            m_diag.animStep = m_diag.fallbackStep = 0.0f;
            m_diag.step = err;
            m_diag.rate = dt > 0.0f ? err / dt : 0.0f;
            return Wrap(aTarget);
        }
        if (dt <= 0.0f)
        {
            m_diag.animStep = m_diag.fallbackStep = m_diag.step = m_diag.rate = 0.0f;
            return Wrap(aCurrent);
        }
        const float s = err >= 0.0f ? 1.0f : -1.0f;
        const float anim = std::fmax(0.0f, s * m_animRate * dt);
        const float fbRate = std::fmin(std::fmax(a / m_cfg.correctionTime, m_cfg.minCorrectionRate), m_cfg.maxCorrectionRate);
        const float fb = fbRate * dt;
        float step = std::fmax(anim, fb);
        step = std::fmin(step, m_cfg.maxRate * dt);
        step = std::fmin(step, a);
        m_diag.animStep = anim;
        m_diag.fallbackStep = fb;
        m_diag.animLed = anim >= fb;
        if (m_diag.animLed)
            ++m_diag.animLedCommands;
        m_diag.step = s * step;
        m_diag.rate = s * step / dt;
        return Wrap(aCurrent + s * step);
    }

    ControllerConfig m_cfg{};
    float m_animRate = 0.0f;
    float m_rate = 0.0f; // 5.3c: last applied rate (follow-law continuity)
    ControllerDiagnostics m_diag{};
};

} // namespace utpp::mm::bt

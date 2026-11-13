// UTPP motion matching runtime - playback rate from the playing clip's root speed (Phase 5.3b, UTPP 0.3.4).
// Header-only, deterministic, allocation-free.
//
// 0.2.2-0.3.3 scaled matcher time by rate = |v_capsule| / MapSpeed(|v_capsule|), i.e. against the per-state LOOP speed
// (run 3.85 -> 5.46). Clips whose root is slower or faster than the loop (starts, stops, transitions, pivots) then moved
// the body at a different speed than the capsule, and at |v| < 0.05 the rate fell back to 1 (stop clips braked at full
// database speed after the capsule had stopped; offset root clamped at 25 cm in game).
// Root rate: target = (|v_capsule| + eps) / (|v_root| + eps), |v_root| = root speed of the segment being played (database
// m/s per matcher second), clamped to [rateMin, rateMax] and smoothed exponentially (halflife). eps keeps idle / near
// stopped poses at ~1 and limits the gain at low speeds. Steady run: 3.85 vs Run_Loop 5.46 -> 0.71 (same as the loop map).
// Active only while aActive (driver: stick released = stop phase; offline the start got worse with root rate on the full
// database, and starts were confirmed good in game with the loop rate). While inactive the controller follows the
// fallback (loop) rate, so switching is continuous (smoothed from the current rate).
#pragma once

#include <cmath>

namespace utpp::mm
{
// ---- speed map (game m/s -> database m/s) and loop rate (0.2.2-0.3.3 driver logic, shared with the offline tests)
// Per locomotion state up to 3 points through (0,0), piecewise linear, extrapolated with the last point's ratio.
struct SpeedPoints
{
    int count;
    float game[3];
    float db[3];
};

inline float MapSpeedPoints(const SpeedPoints& m, float aGame)
{
    if (aGame <= 0.0f)
        return 0.0f;
    if (m.count <= 0)
        return aGame;
    float pg = 0.0f, pd = 0.0f;
    for (int i = 0; i < m.count; ++i)
    {
        if (aGame <= m.game[i])
            return pd + (aGame - pg) * (m.db[i] - pd) / (m.game[i] - pg);
        pg = m.game[i];
        pd = m.db[i];
    }
    return aGame * (pd / pg);
}

struct LoopRateParams
{
    float rateMin = 0.6f;
    float rateMax = 1.4f;
    // Phase 5.3b (0.3.4) slow walk: below the first map point (stand 1.6 -> walk loop 2.19, crouch 2.7 -> 2.46) the query
    // asks for that loop's database speed (the database has nothing slower than Walk_Loop, so lower query speeds selected
    // turn / transition clips) and the rate = speed / loop speed with the lower floor walkRateMin.
    bool walkFloor = false;
    float walkRateMin = 0.25f;
};

// aSpeed = desired game speed (m/s). Outputs the query scale (database / game) and the matcher time rate.
// 0.5.3 aFactor: playback-rate factor of the playing clip (SpeedCalibration ComputeFamilyRates; 1 = 0.5.2), applied
// before the clamp. aFloorDb: database speed the slow-walk floor asks for (0 = m.db[0]).
inline void LoopRate(const SpeedPoints& m, float aSpeed, const LoopRateParams& p, float& aScale, float& aRate,
                     bool* aFloored = nullptr, float aFactor = 1.0f, float aFloorDb = 0.0f)
{
    aScale = 1.0f;
    aRate = 1.0f;
    if (aFloored)
        *aFloored = false;
    if (!(aSpeed > 0.05f))
        return;
    float db = MapSpeedPoints(m, aSpeed);
    float rateDb = db;
    float rmin = p.rateMin;
    if (p.walkFloor && m.count > 0 && aSpeed < m.game[0] && m.db[0] > 0.0f)
    {
        db = aFloorDb > 0.0f ? aFloorDb : m.db[0];
        rateDb = m.db[0];
        rmin = p.walkRateMin;
        if (aFloored)
            *aFloored = true;
    }
    aScale = db / aSpeed;
    aRate = std::fmax(rmin, std::fmin(p.rateMax, aSpeed / rateDb * (std::isfinite(aFactor) ? aFactor : 1.0f)));
}

// Phase 5.3c (0.3.5) slow walk from the real capsule speed. In game (pad half stick, 2026-10-06 22:08:38-47) the query
// speed = max(capsule, intent 3.85 * stick ~ 2.1 m/s) stayed above the first map point (1.6), so the 0.3.4 floor never
// triggered (rate pinned 0.72, Walk_Start / Pivot / Prism chosen) while the capsule walked 0.7-1.2 m/s. The gate is active
// while the stick is held, the capsule is below the first map point (x exitMargin once active) and either the intent is
// below it too or the capsule speed is steady (filtered |dv/dt| < steadyAccel for steadyTime above minSpeed) - so run
// starts (accelerating capsule) keep the 0.3.4 path. While active: query = loop speed of the first point, rate =
// capsule / loop speed in [walkRateMin, rateMax].
struct SlowWalkConfig
{
    float steadyAccel = 0.75f;   // m/s^2
    float steadyTime = 0.15f;    // s
    float accelHalflife = 0.1f;  // s
    float minSpeed = 0.2f;       // m/s
    float exitMargin = 1.1f;
};

class SlowWalkGate
{
public:
    void Reset()
    {
        m_init = m_active = false;
        m_prev = m_accel = m_steady = 0.0f;
    }
    bool Update(float aCapsule, bool aStick, float aIntent, float aThreshold, float aDt, const SlowWalkConfig& c = {})
    {
        const float v = std::isfinite(aCapsule) ? std::fabs(aCapsule) : 0.0f;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
        if (m_init && dt > 0.0f)
        {
            const float raw = (v - m_prev) / dt;
            const float k = c.accelHalflife > 0.0f ? 1.0f - std::exp(-0.69314718f * dt / c.accelHalflife) : 1.0f;
            m_accel += (raw - m_accel) * k;
            if (!std::isfinite(m_accel))
                m_accel = 0.0f;
        }
        m_prev = v;
        m_init = true;
        if (aStick && v > c.minSpeed && std::fabs(m_accel) < c.steadyAccel)
            m_steady += dt;
        else
            m_steady = 0.0f;
        const float lim = aThreshold * (m_active ? c.exitMargin : 1.0f);
        const bool slowIntent = std::isfinite(aIntent) && aIntent < aThreshold;
        m_active = aStick && aThreshold > 0.0f && v < lim && (slowIntent || m_steady >= c.steadyTime || (m_active && std::fabs(m_accel) < 2.0f * c.steadyAccel));
        return m_active;
    }
    bool Active() const { return m_active; }
    float Accel() const { return m_accel; }

private:
    bool m_init = false, m_active = false;
    float m_prev = 0.0f, m_accel = 0.0f, m_steady = 0.0f;
};

// Gate threshold: the first map point, but 80 % of it when it is the only point (crouch 2.7 = full crouch speed, a steady
// full-speed crouch must not be gated).
inline float SlowWalkThreshold(const SpeedPoints& m)
{
    if (m.count <= 0)
        return 0.0f;
    return m.count == 1 ? 0.8f * m.game[0] : m.game[0];
}

// Rate / scale for an active SlowWalkGate: aIntent = desired (query) game speed, aCapsule = real capsule speed.
// 0.5.3: aFactor / aQueryDb as in LoopRate (rate relative to the class point m.db[0] times the clip factor; the query asks
// for aQueryDb, e.g. the slowest walk loop of the database).
inline void SlowWalkRate(const SpeedPoints& m, float aIntent, float aCapsule, const LoopRateParams& p, float& aScale, float& aRate,
                         float aFactor = 1.0f, float aQueryDb = 0.0f)
{
    aScale = 1.0f;
    aRate = 1.0f;
    if (m.count <= 0 || !(m.db[0] > 0.0f) || !(aIntent > 0.05f))
        return;
    aScale = (aQueryDb > 0.0f ? aQueryDb : m.db[0]) / aIntent;
    const float cap = std::isfinite(aCapsule) ? std::fabs(aCapsule) : 0.0f;
    aRate = std::fmax(p.walkRateMin, std::fmin(p.rateMax, cap / m.db[0] * (std::isfinite(aFactor) ? aFactor : 1.0f)));
}

struct RootRateConfig
{
    float rateMin = 0.5f;
    float rateMax = 1.5f;
    float epsilon = 0.5f;   // m/s
    float halflife = 0.08f; // s, real time
    // s after activation (stick release) during which the root rate applies; afterwards the fallback (loop) rate. A stop
    // clip still in its full-speed lead-in would otherwise play in slow motion (rate at rateMin) for a long time while the
    // capsule stands, and the offset root would stay at its clamp (offline stop test).
    float activeWindow = 0.6f;
};

class RootRate
{
public:
    static bool Valid(const RootRateConfig& c)
    {
        return std::isfinite(c.rateMin) && std::isfinite(c.rateMax) && std::isfinite(c.epsilon) && std::isfinite(c.halflife) &&
               c.rateMin >= 0.1f && c.rateMax <= 4.0f && c.rateMin <= c.rateMax && c.epsilon >= 0.05f && c.epsilon <= 5.0f &&
               c.halflife >= 0.0f && c.halflife <= 2.0f && std::isfinite(c.activeWindow) && c.activeWindow >= 0.0f;
    }
    bool SetConfig(const RootRateConfig& c)
    {
        if (!Valid(c))
            return false;
        m_cfg = c;
        return true;
    }
    const RootRateConfig& Config() const { return m_cfg; }
    void Reset()
    {
        m_rate = 1.0f;
        m_init = false;
    }

    // aCapsuleSpeed game m/s; aRootSpeed database m/s (matcher time); aDt real s; aFallback = rate used while inactive.
    // Returns the rate to use.
    float Update(float aCapsuleSpeed, float aRootSpeed, float aDt, bool aActive = true, float aFallback = 1.0f)
    {
        if (aActive)
            m_activeTime = m_active ? m_activeTime + (std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f) : 0.0f;
        if (!aActive || m_activeTime > m_cfg.activeWindow)
        {
            const bool wasActive = aActive; // window expired: stay "active" for timing, use the fallback
            m_active = wasActive;
            if (wasActive)
            {
                // continuous hand-back to the fallback rate
                const float fb = std::isfinite(aFallback) ? aFallback : 1.0f;
                if (m_cfg.halflife > 0.0f && std::isfinite(aDt) && aDt > 0.0f)
                    m_rate += (fb - m_rate) * (1.0f - std::exp(-0.69314718f * aDt / m_cfg.halflife));
                else
                    m_rate = fb;
                m_target = fb;
                m_windowExpired = true;
                return m_rate;
            }
            m_windowExpired = false;
            m_rate = std::isfinite(aFallback) ? aFallback : 1.0f;
            m_target = m_rate;
            m_init = true;
            return m_rate;
        }
        m_active = true;
        m_windowExpired = false;
        const float cap = std::isfinite(aCapsuleSpeed) ? std::fabs(aCapsuleSpeed) : 0.0f;
        const float root = std::isfinite(aRootSpeed) ? std::fabs(aRootSpeed) : 0.0f;
        float target = (cap + m_cfg.epsilon) / (root + m_cfg.epsilon);
        target = std::fmax(m_cfg.rateMin, std::fmin(m_cfg.rateMax, target));
        m_target = target;
        if (!m_init || m_cfg.halflife <= 0.0f)
        {
            m_rate = target;
            m_init = true;
        }
        else if (std::isfinite(aDt) && aDt > 0.0f)
        {
            const float k = 1.0f - std::exp(-0.69314718f * aDt / m_cfg.halflife);
            m_rate += (target - m_rate) * k;
        }
        if (!std::isfinite(m_rate))
            Reset();
        return m_rate;
    }
    float Rate() const { return m_rate; }
    float Target() const { return m_target; }
    bool Active() const { return m_active && !m_windowExpired; }

private:
    RootRateConfig m_cfg{};
    float m_rate = 1.0f;
    float m_target = 1.0f;
    bool m_init = false;
    bool m_active = false;
    bool m_windowExpired = false;
    float m_activeTime = 0.0f;
};
} // namespace utpp::mm

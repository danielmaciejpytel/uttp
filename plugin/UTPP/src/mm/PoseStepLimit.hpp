// UTPP motion matching runtime - per-bone rotation step limiter for a published pose snapshot. Header-only, no allocation.
//
// Bike jump-out (recording 2026-10-28 05-03-09, jump 2, first 0.3 s of the flight): the MM start clip of the jump (M_Neutral_Jump_F_Start_
// Stand_Rfoot, 30 fps) is played from ct 0.067 although V is already airborne at 9.7 m/s, so the ground push-off of the clip (ct 0.17-0.23 s:
// right leg 30 / 35 deg and right foot 43 deg per SOURCE frame, 15 / 21 / 31 deg per 60 Hz frame in the output) is shown in mid-air as a
// leg kick. The pose is not wrong per frame (no clip switch, no missing blend: one clip, continuous ct) - it is too fast for a take-off that
// already happened. PoseStepLimiter caps the rotation step of every bone per published frame for a short window; the bone lags the clip for a
// few frames and catches up where the clip slows down (a rate-limited follow, the angle error is always the remaining distance).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace utpp::mm
{
inline constexpr float kJumpOutSlewSeconds = 0.4f;   // window after the jump-out launch
inline constexpr float kJumpOutSlewTailSeconds = 0.3f; // after it the cap stays while a bone still lags (the lag is never released in one frame)
inline constexpr float kJumpOutSlewDegPerS = 840.0f; // 14 deg per 60 Hz frame (the visible bar is 15 deg)
class PoseStepLimiter
{
public:
    static constexpr uint32_t kMaxBones = 256;
    // aMaxDegPerS: angular speed cap; the per-frame cap is clamped to [kMinStepDeg, kMaxStepDeg] so that a long frame cannot exceed the
    // visible 15 deg bar and a short one still moves.
    static constexpr float kMinStepDeg = 4.0f, kMaxStepDeg = 14.5f;
    void Reset() { m_valid = false; m_limited = 0; m_lagRad = 0.0f; }
    // History dropped (the next Apply starts from its own input): the pose the player sees changed by something other than the clip - the
    // exit hand-in started (game pose source), so the cap must follow it, not pull it back towards the older MM pose.
    void Restart() { m_valid = false; m_lagRad = 0.0f; }
    // Largest remaining distance (rad) between the capped output and the uncapped input of the last Apply (0 = caught up): the window is
    // not closed while a bone still lags, so the end of the window can never release a lag in one frame.
    float LagRad() const { return m_lagRad; }
    // aPose: bones with 7 floats {px,py,pz,qx,qy,qz,qw}; bones [aFirst, aN) are limited (rotation only). aActive false: the state is dropped
    // (the next active call starts from its own input, never from stale history). Returns the number of bones limited on this call.
    uint32_t Apply(float* aPose, uint32_t aFirst, uint32_t aN, float aDt, float aMaxDegPerS, bool aActive)
    {
        if (!aActive || !aPose || aN > kMaxBones || aFirst >= aN)
        {
            m_valid = false;
            m_lagRad = 0.0f;
            return 0;
        }
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 1.0f / 60.0f;
        const float stepDeg = std::min(kMaxStepDeg, std::max(kMinStepDeg, aMaxDegPerS * dt));
        const float stepRad = stepDeg * 3.14159265358979f / 180.0f;
        uint32_t limited = 0;
        float lag = 0.0f;
        for (uint32_t b = aFirst; b < aN; ++b)
        {
            float* q = aPose + size_t(b) * 7 + 3;
            const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (!std::isfinite(n2) || n2 < 1e-12f)
            {
                m_valid = false;
                return limited;
            }
            const float inv = 1.0f / std::sqrt(n2);
            float c[4] = {q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
            float* p = m_prev + size_t(b) * 4;
            if (m_valid)
            {
                float d = p[0] * c[0] + p[1] * c[1] + p[2] * c[2] + p[3] * c[3];
                if (d < 0.0f)
                {
                    for (int k = 0; k < 4; ++k)
                        c[k] = -c[k];
                    d = -d;
                }
                const float ang = 2.0f * std::acos(std::min(1.0f, d));
#if defined(UTPP_STEPLIMIT_MUTATION) && UTPP_STEPLIMIT_MUTATION == 1 // mutation 1: no cap
                if (false)
#else
                if (ang > stepRad)
#endif
                {
                    // slerp prev -> cur by stepRad / ang
                    const float t = stepRad / ang;
                    const float s = std::sqrt(std::max(0.0f, 1.0f - d * d));
                    float o[4];
                    if (s < 1e-6f)
                    {
                        for (int k = 0; k < 4; ++k)
                            o[k] = p[k] + (c[k] - p[k]) * t;
                    }
                    else
                    {
                        const float half = std::acos(std::min(1.0f, d));
                        const float w0 = std::sin((1.0f - t) * half) / s, w1 = std::sin(t * half) / s;
                        for (int k = 0; k < 4; ++k)
                            o[k] = p[k] * w0 + c[k] * w1;
                    }
                    const float on = 1.0f / std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2] + o[3] * o[3]);
                    for (int k = 0; k < 4; ++k)
                        c[k] = o[k] * on;
                    lag = std::max(lag, ang - stepRad);
                    ++limited;
                }
            }
            for (int k = 0; k < 4; ++k)
            {
                q[k] = c[k];
                p[k] = c[k];
            }
        }
        m_valid = true;
        m_limited += limited;
        m_lagRad = lag;
        return limited;
    }
    uint64_t LimitedBones() const { return m_limited; }

private:
    float m_prev[kMaxBones * 4] = {};
    bool m_valid = false;
    uint64_t m_limited = 0;
    float m_lagRad = 0.0f;
};
} // namespace utpp::mm

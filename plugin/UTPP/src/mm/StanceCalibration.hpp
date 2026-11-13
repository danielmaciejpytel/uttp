#pragma once
#include <algorithm>
#include <cmath>

namespace utpp::mm::stancecal
{
// Weight of the crouch calibration in a stand <-> crouch transition.
// Evaluate both maps at the current capsule speed; never filter physical velocity.
// Duration comes from the existing ground-to-ground pose transition policy.
class Blend
{
public:
    void Reset() noexcept { m_ready = m_active = false; m_weight = m_from = m_elapsed = 0.0f; }
    float Weight() const noexcept { return m_weight; }
    float Update(int stance, bool grounded, float duration, float dt) noexcept
    {
        const float target = stance == 1 ? 1.0f : 0.0f;
        if (!grounded || !std::isfinite(duration) || !(duration > 0.0f) || !std::isfinite(dt) || dt < 0.0f)
        {
            Reset(); m_weight = target; return m_weight;
        }
        if (!m_ready)
        {
            m_ready = true; m_stance = stance; m_weight = target;
            return m_weight;
        }
        if (stance != m_stance)
        {
            m_stance = stance; m_from = m_weight; m_elapsed = 0.0f;
            m_duration = duration; m_active = true;
            return m_weight; // C0 on the stance edge, including a repeated transition.
        }
        if (m_active)
        {
            m_elapsed = std::min(m_duration, m_elapsed + dt);
            if (m_elapsed >= m_duration)
                m_active = false;
            else
            {
                const float u = m_elapsed / m_duration;
                const float w = u * u * (3.0f - 2.0f * u);
                m_weight = m_from + (target - m_from) * w;
                return m_weight;
            }
        }
        m_weight = target;
        return m_weight;
    }
private:
    bool m_ready = false, m_active = false;
    int m_stance = 0;
    float m_weight = 0.0f, m_from = 0.0f, m_elapsed = 0.0f, m_duration = 0.0f;
};
}

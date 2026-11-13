// Timeline handover for locomotion signals, separate from bone inertialization.
// Ordinary playback passes through unchanged; storage is fixed and allocation-free.
#pragma once
#include <algorithm>
#include <cmath>

namespace utpp::mm::locosignal
{
struct Sample
{
    float rootVel[2]{};
    float rootYawRate = 0.0f;
    float contact[2]{};
};

class Blend
{
public:
    void Reset() noexcept
    {
        m_out = {}; m_from = {};
        m_ready = m_active = false;
        m_elapsed = m_duration = 0.0f;
    }
    const Sample& Output() const noexcept { return m_out; }
    static bool Valid(const Sample& s) noexcept
    {
        return std::isfinite(s.rootVel[0]) && std::isfinite(s.rootVel[1]) &&
               std::isfinite(s.rootYawRate) && std::isfinite(s.contact[0]) && std::isfinite(s.contact[1]);
    }
    const Sample& Update(const Sample& target, bool jumped, float duration, float dt) noexcept
    {
        if (!Valid(target) || !std::isfinite(dt) || dt < 0.0f || !std::isfinite(duration) || duration < 0.0f)
        {
            Reset(); return m_out;
        }
        if (!m_ready)
        {
            m_out = target; m_ready = true; return m_out;
        }
        if (jumped)
        {
            m_from = m_out;
            m_elapsed = 0.0f;
            m_duration = duration;
            m_active = duration > 0.0f;
            // Deliberately retain the last published signal on the jump feed.
            // Unlike the pose's dt-advanced polynomial, a contact must not step
            // across FootLocker's hysteresis threshold on the handover itself.
            if (m_active) return m_out;
        }
        if (!m_active) { m_out = target; return m_out; }
        m_elapsed = std::min(m_duration, m_elapsed + dt);
        if (m_elapsed >= m_duration)
        {
            m_active = false; m_out = target; return m_out;
        }
        const float u = m_elapsed / m_duration;
        const float w = u * u * (3.0f - 2.0f * u);
        for (int i = 0; i < 2; ++i)
        {
            m_out.rootVel[i] = m_from.rootVel[i] + (target.rootVel[i] - m_from.rootVel[i]) * w;
            m_out.contact[i] = m_from.contact[i] + (target.contact[i] - m_from.contact[i]) * w;
        }
        m_out.rootYawRate = m_from.rootYawRate + (target.rootYawRate - m_from.rootYawRate) * w;
        return m_out;
    }
    // Air retains root continuity but cannot carry a ground contact into flight.
    // The caller also supplies zero target contacts throughout the air episode.
    void ClearContacts() noexcept
    {
        m_out.contact[0] = m_out.contact[1] = m_from.contact[0] = m_from.contact[1] = 0.0f;
    }
private:
    Sample m_out{}, m_from{};
    bool m_ready = false, m_active = false;
    float m_elapsed = 0.0f, m_duration = 0.0f;
};
}

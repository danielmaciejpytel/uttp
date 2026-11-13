#pragma once
#include <atomic>
#include <cstdint>

namespace utpp::mm
{
// Camera-owned heartbeat. A single atomic publishes permission and its age
// together: zero denies, a positive QPC timestamp allows only while fresh.
// False is published BEFORE entering FPP; true only after entering full TPP.
// The locomotion fade and delayed feed must never reopen this gate.
class PerspectiveWriteGate
{
public:
    void Publish(bool tpp, int64_t ticks)
    {
        m_stamp.store(tpp && ticks > 0 ? ticks : 0, std::memory_order_release);
    }
    void Reset() { m_stamp.store(0, std::memory_order_release); }
    bool Allows(int64_t now, int64_t maxAge) const
    {
        const int64_t stamp = m_stamp.load(std::memory_order_acquire);
        return stamp > 0 && now >= stamp && maxAge > 0 && now - stamp <= maxAge;
    }
private:
    std::atomic<int64_t> m_stamp{0};
};
} // namespace utpp::mm

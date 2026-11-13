#pragma once
// The helper-bone capture rings (g_capture / g_postHelperCapture) have exactly one consumer at a time. The text capture
// (UTPPBodyPoseTraceContinuous, BodyTraceRuntime.cpp) and the compact capture (UTPPPoseCaptureStart, PoseCaptureRuntime.cpp) each take
// ownership before they start their writer thread and give it back when they stop; the second one is refused.
#include <atomic>

namespace utpp::mm::bodytrace
{
enum class RingOwner : int
{
    None = 0,
    TextCapture = 1,
    CompactCapture = 2,
};
inline std::atomic<int> g_ringOwner{0};
// true when aWho owns the rings afterwards (taking them again is idempotent).
inline bool AcquireRings(RingOwner aWho)
{
    int expected = static_cast<int>(RingOwner::None);
    return g_ringOwner.compare_exchange_strong(expected, static_cast<int>(aWho)) || expected == static_cast<int>(aWho);
}
inline void ReleaseRings(RingOwner aWho)
{
    int expected = static_cast<int>(aWho);
    g_ringOwner.compare_exchange_strong(expected, static_cast<int>(RingOwner::None));
}
inline RingOwner CurrentRingOwner() { return static_cast<RingOwner>(g_ringOwner.load()); }
} // namespace utpp::mm::bodytrace

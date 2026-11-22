// UTPP motion matching runtime - body height smoothing (mm/BodyHeightFilter.hpp), shared state between the game thread and the
// animation threads. Header-only, lock-free, allocation-free.
//
// The drawn capsule height is known only where the pose is drawn: the feed reads the capsule before the game moves it for the frame,
// so a filter on the feed height cannot see the ground snap of the same frame. The stage therefore runs where the published pose is
// written into the game pose (FK or mapping job of the player), with the player's world height read there; the feed only publishes
// whether the stage may act (gate) and a reset serial.
//   - game thread (feed / setting): SetEnabled, PublishGate, PublishRig (database load), SetTransformSource (player transform);
//   - animation thread: Step (one tracker update per frame; concurrent and repeated calls of the same frame reuse the stored offset),
//     ApplyToSnapshot (pelvis offset + leg solve on the local copy of the published pose).
// On by default (mm.bodyHeightSmooth). Off: no state is stepped and the snapshot is not touched.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "BodyHeightFilter.hpp"
#include "GameIntegration.hpp"

namespace utpp::mm::bodyheight::rt
{

inline std::atomic<bool> g_on{true}; // on by default (the menu default, motionhost g_bodyHeightOn)
// bit 0: the stage may act (feed gate); bit 1: a moving platform carries V (no smoothing, Filter::Step aPlatform);
// bits 8..31: reset serial (teleport, hand-back, toggle)
inline std::atomic<uint32_t> g_gate{0};
inline integ::SeqlockBuffer<LegRig> g_rig;
// player transform component (game pointer, read on the animation thread through an SEH guard by the DLL); 0 = none
inline std::atomic<uintptr_t> g_transformSource{0};

struct Stepper
{
    Filter filter;
    int64_t lastTicks = 0;
    uint32_t serial = 0;
};
inline Stepper g_stepper;               // owned by the thread that holds g_stepLock
inline std::atomic_flag g_stepLock = ATOMIC_FLAG_INIT;
inline std::atomic<float> g_offset{0.0f};

// diagnostics and the timing probe (drawn height at the last step and the feed height next to it)
inline std::atomic<float> g_drawZ{0.0f};
inline std::atomic<int64_t> g_drawTicks{0};
inline std::atomic<uint32_t> g_stepsSinceFeed{0};
inline std::atomic<uint64_t> g_steps{0}, g_noSource{0}, g_applied{0}, g_legSolves{0}, g_legClamps{0}, g_serialResets{0};
inline std::atomic<float> g_maxAnkleError{0.0f};

inline bool Enabled() { return g_on.load(std::memory_order_relaxed); }

inline void BumpSerial()
{
    g_gate.fetch_add(1u << 8, std::memory_order_acq_rel);
}

inline void SetEnabled(bool aOn)
{
    if (g_on.exchange(aOn, std::memory_order_acq_rel) != aOn)
    {
        BumpSerial();
        g_offset.store(0.0f, std::memory_order_relaxed);
    }
}

// Game thread, every publish while the stage is on.
inline void PublishGate(bool aAllowed, bool aReset, bool aPlatform = false)
{
    const uint32_t cur = g_gate.load(std::memory_order_relaxed);
    const uint32_t serial = (cur >> 8) + (aReset ? 1u : 0u);
    g_gate.store((serial << 8) | (aAllowed ? 1u : 0u) | (aPlatform ? 2u : 0u), std::memory_order_release);
}

inline void PublishRig(const LegRig& aRig) { g_rig.Publish(aRig); }

inline void SetTransformSource(uintptr_t aComponent) { g_transformSource.store(aComponent, std::memory_order_release); }
inline uintptr_t TransformSource() { return g_transformSource.load(std::memory_order_acquire); }

// Animation thread. aNow / aTicksPerSecond: the caller's clock; aHaveZ / aZ: the drawn world height of the player read now.
// Returns the pelvis offset (m). A second call within kSameFrameSeconds, or a call while another thread steps, returns the stored offset.
inline constexpr double kSameFrameSeconds = 0.0005;
inline float Step(int64_t aNow, int64_t aTicksPerSecond, bool aHaveZ, float aZ)
{
    if (g_stepLock.test_and_set(std::memory_order_acquire))
        return g_offset.load(std::memory_order_relaxed);
    Stepper& s = g_stepper;
    const uint32_t gate = g_gate.load(std::memory_order_acquire);
    if ((gate >> 8) != s.serial)
    {
        s.serial = gate >> 8;
        s.filter.Reset();
        s.lastTicks = 0;
        g_serialResets.fetch_add(1, std::memory_order_relaxed);
    }
    float off = g_offset.load(std::memory_order_relaxed);
    const double dt = s.lastTicks != 0 && aTicksPerSecond > 0 ? double(aNow - s.lastTicks) / double(aTicksPerSecond) : 0.0;
    if (!aHaveZ || !std::isfinite(aZ))
    {
        g_noSource.fetch_add(1, std::memory_order_relaxed);
        s.filter.Reset();
        s.lastTicks = 0;
        off = 0.0f;
    }
    else if (s.lastTicks == 0 || dt >= kSameFrameSeconds)
    {
        off = s.filter.Step(static_cast<float>(dt), aZ, (gate & 1u) != 0, (gate & 2u) != 0);
        s.lastTicks = aNow;
        g_steps.fetch_add(1, std::memory_order_relaxed);
        g_stepsSinceFeed.fetch_add(1, std::memory_order_relaxed);
        g_drawZ.store(aZ, std::memory_order_relaxed);
        g_drawTicks.store(aNow, std::memory_order_relaxed);
    }
    g_offset.store(off, std::memory_order_relaxed);
    g_stepLock.clear(std::memory_order_release);
    return off;
}

// Animation thread: applies aOffset to the local copy of the published pose. Not on a melee frame that hands the lower body to the
// game pose (lowerScale < 1), nor on a pose of another topology. True when the pelvis moved.
inline bool ApplyToSnapshot(integ::PoseSnapshot& aSnap, float aOffset)
{
    if (aOffset == 0.0f || !std::isfinite(aOffset) || !(aSnap.weight > 0.0f) || integ::SnapshotLowerScale(aSnap) < 1.0f)
        return false;
    LegRig rig;
    if (!g_rig.Read(rig) || !rig.Valid())
        return false;
    const ApplyResult r = ApplyOffset(rig, aSnap.pose, aSnap.numBones, aOffset);
    if (!r.applied)
        return false;
    g_applied.fetch_add(1, std::memory_order_relaxed);
    g_legSolves.fetch_add(uint64_t(r.solved), std::memory_order_relaxed);
    g_legClamps.fetch_add(uint64_t(r.clamped), std::memory_order_relaxed);
    float prev = g_maxAnkleError.load(std::memory_order_relaxed);
    while (r.ankleError > prev && !g_maxAnkleError.compare_exchange_weak(prev, r.ankleError, std::memory_order_relaxed))
    {
    }
    return true;
}

} // namespace utpp::mm::bodyheight::rt

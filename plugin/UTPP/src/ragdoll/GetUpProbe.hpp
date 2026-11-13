#pragma once
// UTPP ragdoll R0 probe: publish frame 0 of a motion get-up clip with the root children (Hips) moved far from the entity origin.
// Header only, no game / RED4ext / data dependency (the driver glue is in MotionDriver.cpp "ragdoll R0 probe").
//
// Question it answers: when a no-physics hybrid ragdoll publishes a
// pose whose hips are 1.2 / 3 m away from the entity root, do the body and its shadow stay visible (not culled)?
//
// Pose composition (Compose): all bones from firstBone are copied from the clip frame; the "root children" (bones whose parent is below
// firstBone, i.e. Hips) get the offset added to their local translation along the fixed CHARACTER-FRAME forward axis (+Y of the
// pose frame, the same frame in which the traversal warp moves the root children: loc = RotZ(world - capsule, -entityYaw)). With a
// Root bone aligned to the entity this is the horizontal facing direction. Every other bone is copied bit-identically, so the pose
// is the clip's own relative pose, only displaced as a whole.
//
// Lifetime (GetUpProbe): Idle -> Hold (durationS of feed time, clamped 0.5..5 s) -> HandBack (kHandBackS, the driver blends the
// published pose from the held probe pose to the normal MM pose) -> Idle. Abort() clears at once. A wall-clock watchdog (kWatchdogS)
// ends a run even if no feed arrives (pause, stalled republish). All time inputs are injected, so the unit test is deterministic.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef UTPP_RAGDOLL_PROBE_MUTATION
#define UTPP_RAGDOLL_PROBE_MUTATION 0
#endif

namespace utpp::ragdoll
{
inline constexpr float kProbeMinDurationS = 0.5f;
inline constexpr float kProbeMaxDurationS = 5.0f;
inline constexpr float kProbeMaxOffsetM = 10.0f;
inline constexpr double kProbeWatchdogS = 5.25;  // wall clock since Start; Hold is at most 5 s + the 0.12 s hand-back
inline constexpr float kProbeLogPeriodS = 0.5f;
inline constexpr float kProbeHandBackS = 0.12f;  // = motionhost::kDefaultHardOutTime (BlendFromHeld's span)
inline constexpr float kProbeMaxStepS = 0.25f;   // one feed counts for at most this much feed time (the driver clamps the same way)

// Fixed character-frame direction of the displacement (see the file comment): +Y.
#if UTPP_RAGDOLL_PROBE_MUTATION == 1
inline constexpr float kProbeAxis[3] = {1.0f, 0.0f, 0.0f}; // mutation 1: wrong axis (sideways) MUST fail the axis checks
#else
inline constexpr float kProbeAxis[3] = {0.0f, 1.0f, 0.0f};
#endif

struct ProbeArgs
{
    bool ok = false;
    float offsetM = 0.0f;
    float durationS = 0.0f;
};

// Non-finite input is refused; the offset is clamped to 0..kProbeMaxOffsetM, the duration to 0.5..5 s.
inline ProbeArgs SanitizeProbeArgs(float aOffsetM, float aDurationS)
{
    ProbeArgs a;
    if (!std::isfinite(aOffsetM) || !std::isfinite(aDurationS))
        return a;
    a.ok = true;
    a.offsetM = std::clamp(aOffsetM, 0.0f, kProbeMaxOffsetM);
    a.durationS = std::clamp(aDurationS, kProbeMinDurationS, kProbeMaxDurationS);
    return a;
}

// aSrc / aDst: aNumBones * 7 floats (local p.xyz, q.xyzw per bone = PoseSnapshot::pose = MotionDatabase PoseRecord layout).
// aParents: parent index per bone (-1 = none). Bones below aFirstBone are not touched in aDst.
// Returns the number of root children that were displaced.
inline uint32_t ComposeProbePose(const float* aSrc, const int32_t* aParents, uint32_t aNumBones, uint32_t aFirstBone, float aOffsetM,
                                 float* aDst)
{
    uint32_t moved = 0;
    for (uint32_t i = aFirstBone; i < aNumBones; ++i)
    {
        const float* s = aSrc + size_t(i) * 7;
        float* d = aDst + size_t(i) * 7;
        std::memcpy(d, s, 7 * sizeof(float));
        if (aParents[i] < static_cast<int32_t>(aFirstBone)) // root child (parent -1 included)
        {
            for (int k = 0; k < 3; ++k)
                d[k] += kProbeAxis[k] * aOffsetM;
            ++moved;
        }
    }
    return moved;
}

enum class ProbeEnd : uint8_t { None, Expired, Aborted, Watchdog };
enum class ProbePhase : uint8_t { Idle, Hold, HandBack };

class GetUpProbe
{
public:
    struct TickResult
    {
        bool logDue = false;         // a 0.5 s status line is due (Hold only)
        bool handBackStarted = false; // Hold just expired
        bool ended = false;          // back to Idle in this tick
        ProbeEnd reason = ProbeEnd::None;
    };

    // The cheap per-frame check of the driver.
    bool Active() const noexcept { return m_phase != ProbePhase::Idle; }
    ProbePhase Phase() const noexcept { return m_phase; }
    bool Holding() const noexcept { return m_phase == ProbePhase::Hold; }
    bool HandingBack() const noexcept { return m_phase == ProbePhase::HandBack; }
    float OffsetM() const noexcept { return m_offset; }
    float DurationS() const noexcept { return m_duration; }
    float ElapsedS() const noexcept { return m_elapsed; }
    float HandBackLeftS() const noexcept { return m_handLeft; }
    int32_t Clip() const noexcept { return m_clip; }
    uint32_t Runs() const noexcept { return m_runs; }

    // Offset the composed pose uses right now: only while holding.
    float ComposeOffsetM() const noexcept { return m_phase == ProbePhase::Hold ? m_offset : 0.0f; }

    bool Start(float aOffsetM, float aDurationS, double aNowS, int32_t aClip)
    {
        const ProbeArgs a = SanitizeProbeArgs(aOffsetM, aDurationS);
        if (!a.ok || aClip < 0 || !std::isfinite(aNowS))
            return false;
        m_phase = ProbePhase::Hold;
        m_offset = a.offsetM;
        m_duration = a.durationS;
        m_elapsed = 0.0f;
        m_nextLog = kProbeLogPeriodS;
        m_handLeft = 0.0f;
        m_startWall = aNowS;
        m_clip = aClip;
        ++m_runs;
        return true;
    }

    // Feed-time tick (game thread, driver lock held). aNowS: monotonic wall seconds (watchdog).
    TickResult Advance(float aDtS, double aNowS)
    {
        TickResult r;
        if (m_phase == ProbePhase::Idle)
            return r;
        if (WatchdogHit(aNowS))
        {
            Clear();
            r.ended = true;
            r.reason = ProbeEnd::Watchdog;
            return r;
        }
        const float dt = (std::isfinite(aDtS) && aDtS > 0.0f) ? std::fmin(aDtS, kProbeMaxStepS) : 0.0f;
        if (m_phase == ProbePhase::Hold)
        {
            m_elapsed += dt;
#if UTPP_RAGDOLL_PROBE_MUTATION == 2
            // mutation 2: the hold never expires (only the watchdog ends it) MUST fail the expiry / hand-back checks
#else
            if (m_elapsed >= m_duration)
            {
                m_phase = ProbePhase::HandBack;
                m_handLeft = kProbeHandBackS;
                r.handBackStarted = true;
                return r;
            }
#endif
            if (m_elapsed >= m_nextLog)
            {
                m_nextLog += kProbeLogPeriodS;
                r.logDue = true;
            }
            return r;
        }
        m_handLeft -= dt; // HandBack
        if (m_handLeft <= 0.0f)
        {
            Clear();
            r.ended = true;
            r.reason = ProbeEnd::Expired;
        }
        return r;
    }

    // Wall-clock watchdog only (publish path that may run without any feed). True = it ended the run.
    bool Watchdog(double aNowS)
    {
        if (m_phase == ProbePhase::Idle || !WatchdogHit(aNowS))
            return false;
        Clear();
        return true;
    }

    // Abort: all run state cleared at once (no hand-back: the reason is a state in which the pose must not be published).
    void Abort() noexcept { Clear(); }

private:
    bool WatchdogHit(double aNowS) const noexcept { return !(aNowS - m_startWall <= kProbeWatchdogS); } // NaN also trips it
    void Clear() noexcept
    {
        m_phase = ProbePhase::Idle;
        m_offset = m_duration = m_elapsed = m_nextLog = m_handLeft = 0.0f;
        m_clip = -1;
        m_startWall = 0.0;
    }

    ProbePhase m_phase = ProbePhase::Idle;
    float m_offset = 0.0f, m_duration = 0.0f, m_elapsed = 0.0f, m_nextLog = 0.0f, m_handLeft = 0.0f;
    double m_startWall = 0.0;
    int32_t m_clip = -1;
    uint32_t m_runs = 0;
};
} // namespace utpp::ragdoll

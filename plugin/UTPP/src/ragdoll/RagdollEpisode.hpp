// UTPP ragdoll production core: episode ownership, pelvis telemetry and the MM hand-over (plain C++, no Windows / RED4ext,
// unit-tested in tests/RagdollEpisodeTest.cpp).
//
// ===================================================================================================================
// NATIVES FOR SCRIPTS (registered in ragdoll/RagdollSlotProbe.cpp RegisterNatives; global functions, redscript form):
//
//   public native func UTPPRagdollSetPlayer(entityIdHash: Uint64) -> Void
//       Optional. The DLL reads the player's EntityID itself (entity+0x48, once per second while armed); a script that
//       knows it may publish it as well (Cast<Uint64>(player.GetEntityID())). A change drops the slot cache.
//   public native func UTPPRagdollBegin() -> Bool
//       Arms the episode (Idle -> Active, unconfirmed). Call it right before the activation request is sent. The overlay
//       gate (pose slot +0x29) then confirms it (that is when UTPP stops writing V's pose); when the gate does not rise
//       within 1.0 s the episode is abandoned. true = armed (also when already Active), false = refused (no hook / no player slot).
//   public native func UTPPRagdollRelease() -> Void
//       The script has sent the disable event. Only records the intent: the BlendOut timer starts at the real 1 -> 0 edge
//       of the gate. Before the gate ever rose it cancels the episode.
//   public native func UTPPRagdollAbort() -> Void
//       Ends the episode at once (owner flag cleared, keep store invalidated) and stays idle while the gate is still up (the
//       disable event not yet taken); a new activation needs the gate to fall first or a new Begin. Use on every abort path.
//   public native func UTPPRagdollState() -> Int32
//       0 idle, 1 active, 2 blend-out. While 1 or 2 UTPP does not write V's pose and MM is released to the game.
//   public native func UTPPRagdollPelvis() -> Vector4
//       V's pelvis (Hips) position in model space (entity frame) after FK for the full player descriptor; w = 1 when the
//       sample is fresh (< 0.5 s old), w = 0 (x = y = z = 0) otherwise. Sampled every frame during an episode, 4 Hz idle.
//   public native func UTPPRagdollPelvisSpeed() -> Float
//       Pelvis speed in m/s, exponentially smoothed with a 0.1 s time constant; 0 when there is no fresh sample.
//   public native func UTPPRagdollWorldSpeed(entityPos: Vector4, entityRot: Quaternion) -> Float
//       Smoothed speed (m/s) of the WORLD pelvis = entityPos + entityRot * Hips model position (the entity root follows the body
//       horizontally). Call it each script tick during an episode with player.GetWorldPosition() / GetWorldOrientation(); 0 when
//       idle or without a fresh pelvis sample.
//   public native func UTPPRagdollSettled(entityPos: Vector4, entityRot: Quaternion) -> Bool
//       Same input, also steps the settle detector: true when the body has been below 0.15 m/s for 0.5 s, not before 1.0 s after the
//       episode start, forced at 5 s; any later impact (speed back above the threshold) re-arms the hold. false while idle.
//   public native func UTPPRagdollFaceUp() -> Int32
//       1 face up, 0 face down, -1 unknown (on the side, no fresh sample, no standing reference yet). Computed from the
//       belly normal cross(Spine - Hips, RightUpLeg - LeftUpLeg) after FK, signed against the standing pose.
//   Probe natives UTPPRagdollSlotDump / UTPPRagdollEntityHash stay (optional, research only).
// ===================================================================================================================
//
// Episode state machine (injected time, seconds):
//   Idle -> Active    : the overlay gate is seen, or Begin() (State() = 1 at once, owner flag only once the gate is seen).
//   Active -> BlendOut: the gate falls after it was seen (or Release() was sent and the gate stays up for releaseStuckS: a forced
//                       blend-out; the still-raised gate is then ignored until it is seen low, and the episode ends in Idle with
//                       the lockout set so the stuck gate does not restart it). The engine's RagdollControl blends out over 0.5 s, so the keep
//                       store keeps restoring for blendOut + margin (0.65 s) counted from that real edge, not from Release().
//   BlendOut -> Idle  : the timer ran out.  BlendOut -> Active: the gate rises again (a new activation), except in a forced
//                       blend-out until the gate has been seen low.
//   any -> Idle       : Abort(), the hard maximum, an unconfirmed Begin() that the gate never confirmed.
// The owner flag (Owns()) is true in confirmed Active (the gate was seen) and in BlendOut; an unconfirmed Active (Begin() sent,
// gate not up yet) does not own the body, so nothing is withheld before the ragdoll is visible. It is the ONE atomic that UTPP's pose writers check: MM early
// mapping, the FK-side writers, body helpers, AO / hand IK / foot lock inside the MM pose blend, the seated neck cut and the
// MM driver feed (GameOwnedFeed). Idle = false = behaviour identical to a build without this code.
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

#include "../mm/GameIntegration.hpp"

#ifndef UTPP_RAGDOLL_EPISODE_MUTATION
#define UTPP_RAGDOLL_EPISODE_MUTATION 0
#endif

namespace utpp::ragdoll
{
enum class EpState : int32_t { Idle = 0, Active = 1, BlendOut = 2 };

struct EpisodeConfig
{
    double blendOutS = 0.5;       // animAnimNode_RagdollControl default blendOutDuration (Update 0x140bf26bc)
    double marginS = 0.15;
    double confirmTimeoutS = 1.0; // Begin() without a gate rise
    double releaseStuckS = 1.5;   // Release() but the gate never falls: blend out anyway
    double maxEpisodeS = 30.0;    // hard maximum from the start of the episode
};

struct EpisodeCounters
{
    uint64_t episodes = 0, confirmed = 0, confirmTimeouts = 0, aborts = 0, maxTimeouts = 0, releaseStuck = 0, reactivations = 0;
    uint64_t getUpHandOffs = 0;
};

class Episode
{
public:
    explicit Episode(EpisodeConfig aCfg = {}) : m_cfg(aCfg) {}

    // Script: arm the episode. Returns false only when refused (aAllowed false: no hook / no slot).
    bool Begin(double aNow, bool aAllowed = true)
    {
        if (!aAllowed)
            return false;
        std::lock_guard lock(m_mutex);
        m_lockout.store(false, std::memory_order_release);
        if (m_state == EpState::Active)
            return true;
        if (m_state == EpState::BlendOut) // a new request while the old one still blends out: keep ownership until the gate rises
        {
            ++m_counters.reactivations;
            m_forcedBlend = false; // a new request: the raised gate may reactivate the episode
            m_blendEnd = std::max(m_blendEnd, aNow + m_cfg.confirmTimeoutS);
            return true;
        }
        EnterActive(aNow, false, true);
        return true;
    }

    // Script: the disable event was sent. Intent only; the BlendOut timer starts at the gate edge.
    void Release(double aNow)
    {
        std::lock_guard lock(m_mutex);
        if (m_state != EpState::Active)
            return;
        if (!m_confirmed) // the request never activated: a disable before activation is a no-op in the engine
        {
            GoIdle();
            return;
        }
        m_released = true;
        m_releaseAt = aNow;
#if UTPP_RAGDOLL_EPISODE_MUTATION == 1
        EnterBlendOut(aNow); // mutation 1: the timer starts at Release instead of at the gate edge
#endif
    }

    // Script: the native get-up takes the body over (UTPPRagdollGetUpBegin). Active or BlendOut -> Idle with the stuck-gate lockout
    // armed: the still-raised gate is ignored until it was seen low (same rule as the forced blend-out), so the episode does not
    // restart when the gate needs a frame or two to fall. Counted as an abort-free end of the episode.
    void HandToGetUp(double aNow)
    {
        (void)aNow;
        std::lock_guard lock(m_mutex);
        if (m_state != EpState::Idle)
            ++m_counters.getUpHandOffs;
        GoIdle();
#if UTPP_RAGDOLL_EPISODE_MUTATION != 8
        m_lockout.store(true, std::memory_order_release); // mutation 8: the still-raised gate restarts the episode
#endif
    }

    void Abort(double aNow)
    {
        (void)aNow;
        std::lock_guard lock(m_mutex);
        if (m_state == EpState::Idle)
            return;
        ++m_counters.aborts;
        GoIdle();
        m_lockout.store(true, std::memory_order_release); // a gate that is still up (the disable event not taken yet) must not restart it
    }

    // Hot / per-frame: aGate = the overlay gate of V's slot (+0x29 != 0). Any thread.
    // aGateKnown = false: the slot could not be read (cache dropped, no player id, fault); aGate must then be false. The time-driven
    // transitions run as with a low gate, but an unknown reading is never taken as "the gate fell": it neither clears the lockout
    // of an Abort() nor ends a forced blend-out's gate-ignore.
    void Tick(bool aGate, double aNow, bool aGateKnown = true)
    {
        if (!aGate && m_pubState.load(std::memory_order_acquire) == static_cast<int32_t>(EpState::Idle) &&
            !m_lockout.load(std::memory_order_acquire))
            return; // idle fast path: no lock
        std::lock_guard lock(m_mutex);
        if (m_state != EpState::Idle && aNow - m_startedAt > m_cfg.maxEpisodeS)
        {
            ++m_counters.maxTimeouts;
            GoIdle();
            m_lockout.store(true, std::memory_order_release); // a gate that is still up does not start a new episode
        }
        switch (m_state)
        {
        case EpState::Idle:
            if (!aGate)
            {
#if UTPP_RAGDOLL_EPISODE_MUTATION != 5
                if (aGateKnown) // mutation 5: a missing slot reading clears the lockout
#endif
                    m_lockout.store(false, std::memory_order_release);
            }
            else if (!m_lockout.load(std::memory_order_acquire))
                EnterActive(aNow, true, true);
            break;
        case EpState::Active:
            if (aGate)
            {
                if (!m_confirmed)
                {
                    m_confirmed = true;
                    ++m_counters.confirmed;
                    Publish();
                }
            }
            else if (m_confirmed)
                EnterBlendOut(aNow); // the real 1 -> 0 edge
            else if (aNow - m_beginAt > m_cfg.confirmTimeoutS)
            {
                ++m_counters.confirmTimeouts;
                GoIdle();
            }
            if (m_state == EpState::Active && m_released && aGate && aNow - m_releaseAt > m_cfg.releaseStuckS)
            {
                ++m_counters.releaseStuck;
                EnterBlendOut(aNow);
#if UTPP_RAGDOLL_EPISODE_MUTATION != 4
                m_forcedBlend = true; // mutation 4: the forced blend-out bounces back to Active while the gate stays up
#endif
            }
            break;
        case EpState::BlendOut:
            if (!aGate && aGateKnown)
                m_forcedBlend = false; // the gate was really seen low: a later rise is a new activation
            if (aGate && !m_forcedBlend)
            {
                ++m_counters.reactivations;
                EnterActive(aNow, true, false);
            }
            else if (aNow >= m_blendEnd)
            {
                const bool stuck = m_forcedBlend; // the disable event was never taken: the raised gate must not restart the episode
                GoIdle();
                if (stuck)
                    m_lockout.store(true, std::memory_order_release);
            }
            break;
        }
    }

    EpState State() const { return static_cast<EpState>(m_pubState.load(std::memory_order_acquire)); }
    bool Owns() const noexcept { return m_owner.load(std::memory_order_acquire); }
    EpisodeCounters Counters() const
    {
        std::lock_guard lock(m_mutex);
        return m_counters;
    }
    double BlendTotalS() const { return m_cfg.blendOutS + m_cfg.marginS; }
    double StartedAt() const
    {
        std::lock_guard lock(m_mutex);
        return m_startedAt;
    }

private:
    // Caller holds the mutex.
    void Publish()
    {
        m_pubState.store(static_cast<int32_t>(m_state), std::memory_order_release);
        // An unconfirmed Active (Begin() sent, gate not up yet) does not own the body: until the engine's overlay shows the
        // ragdoll nothing may be withheld from the visible pose.
        m_owner.store(m_state == EpState::BlendOut || (m_state == EpState::Active && m_confirmed), std::memory_order_release);
    }
    void EnterActive(double aNow, bool aConfirmed, bool aNewEpisode)
    {
        m_state = EpState::Active;
        m_confirmed = aConfirmed;
        m_released = false;
        m_forcedBlend = false;
        m_beginAt = aNow;
        if (aNewEpisode)
        {
            m_startedAt = aNow;
            ++m_counters.episodes;
        }
        if (aConfirmed)
            ++m_counters.confirmed;
        Publish();
    }
    void EnterBlendOut(double aNow)
    {
        m_state = EpState::BlendOut;
        m_blendEnd = aNow + m_cfg.blendOutS + m_cfg.marginS;
        m_released = false;
        Publish();
    }
    void GoIdle()
    {
        m_state = EpState::Idle;
        m_confirmed = false;
        m_released = false;
        m_forcedBlend = false;
#if UTPP_RAGDOLL_EPISODE_MUTATION == 2
        m_pubState.store(static_cast<int32_t>(m_state), std::memory_order_release); // mutation 2: the owner flag is never cleared
#else
        Publish();
#endif
    }

    EpisodeConfig m_cfg;
    mutable std::mutex m_mutex;
    EpState m_state = EpState::Idle;
    bool m_confirmed = false, m_released = false;
    bool m_forcedBlend = false; // BlendOut entered by the stuck-release fallback: aGate is ignored until seen low
    double m_beginAt = 0.0, m_startedAt = 0.0, m_releaseAt = 0.0, m_blendEnd = 0.0;
    EpisodeCounters m_counters;
    std::atomic<int32_t> m_pubState{0};
    std::atomic<bool> m_owner{false};
    std::atomic<bool> m_lockout{false};
};

// The one process-wide episode and its owner flag (a single atomic load at every writer call site).
inline Episode g_episode;

// Native get-up (GetUpPlayer.hpp, glue in MotionDriver.cpp): true in the phases Blend, Play and HandOff. Written under the v2 driver
// lock only; read lock-free (MotionHost.cpp Feed, the pose writers through GameOwned()). While it is set UTPP owns V's pose again:
// the writers resume, the RdKeep restore stops and the feed no longer carries the game-owned knockdown detail.
inline std::atomic<bool> g_getUpOwns{false};
inline bool GetUpOwns() noexcept { return g_getUpOwns.load(std::memory_order_acquire); }
inline bool GameOwned() noexcept { return g_episode.Owns() && !GetUpOwns(); }

// v1 (MotionHost.cpp, which may be linked without the v2 driver) -> v2: a request to end the get-up, consumed by the next v2 Step.
inline constexpr int32_t kGetUpAbortBaseGate = 1;
inline std::atomic<int32_t> g_getUpAbortRequest{0};
inline void RequestGetUpAbort(int32_t aCode) noexcept { g_getUpAbortRequest.store(aCode, std::memory_order_release); }
inline int32_t TakeGetUpAbortRequest() noexcept { return g_getUpAbortRequest.exchange(0, std::memory_order_acq_rel); }

// ------------------------------------------------------------------------------------------------ MM hand-over
// While the episode owns the body the MM driver is fed the existing "game owned" state of knockdown (LocomotionDetailed 29,
// mm::integ::PsmDetailReleases): the gate closes, HoldBlocker reports release:detail and the normal gate fader fades MM out and
// (after the episode, through the reopen hold) back in, instead of snapping. No new MM behaviour.
inline constexpr int kGameOwnedDetail = 29; // adapter::kPsmKnockdown
inline uint32_t GameOwnedPsmBlock(uint32_t aPsmBlock)
{
    using namespace utpp::mm::integ;
    return (aPsmBlock & ~kPsmDetailFieldMask) | kPsmDetailed |
           (static_cast<uint32_t>(kGameOwnedDetail + 1) << kPsmDetailShift);
}
// The inverse for the native get-up: the knockdown detail the script injects is removed (detail treated as Stand), every other bit
// (vehicle, death, scene, ...) stays, so those still close the gate and end the get-up.
inline uint32_t GetUpPsmBlock(uint32_t aPsmBlock)
{
    using namespace utpp::mm::integ;
    return aPsmBlock & ~(kPsmDetailFieldMask | uint32_t(kPsmDetailed));
}

// Section 5 (false jump / landing after a ragdoll): the window in which a small drop to the floor is no fall. It opens when the
// raw knockdown detail (29) of the feed ends (the script stops injecting it, also on the vanilla recovery path) and runs for
// kRagdollRecoverWindowS of feed time. Stepped by MotionHost.cpp Feed (game thread, driver lock); idle = false = nothing changes.
class RecoverWindow
{
public:
    static constexpr float kWindowS = 1.5f; // = kRagdollRecoverWindowS
    void Reset() { m_prevDetail = -1; m_left = 0.0f; }
    void Step(int aDetail, float aDt)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
        if (m_prevDetail == kGameOwnedDetail && aDetail != kGameOwnedDetail)
            m_left = kWindowS;
        else if (aDetail == kGameOwnedDetail)
            m_left = 0.0f;
        else
            m_left = std::fmax(0.0f, m_left - dt);
        m_prevDetail = aDetail;
    }
    bool Active() const { return m_left > 0.0f; }

private:
    int m_prevDetail = -1;
    float m_left = 0.0f;
};
inline RecoverWindow g_recoverWindow;
// Episode running or the post-knockdown window open (the v2 grace of a running get-up is added by the driver).
inline bool RagdollRecoverActive() { return g_episode.State() != EpState::Idle || g_recoverWindow.Active(); }

// ------------------------------------------------------------------------------------------------ settle detection
// In-game fact (2026-11-03 17:44): the entity root follows the ragdoll body horizontally, so settle speed is measured on the WORLD
// pelvis (entity position + Hips model position) and a late impact (an NPC hitting V at t = 2.8 s) must re-arm the timer.
// Exponentially smoothed speed from successive world positions (injected time, seconds).
class SmoothSpeed
{
public:
    static constexpr double kTauS = 0.1, kRestartGapS = 0.5;
    void Reset() { m_have = false; m_speed = 0.0f; }
    float Step(const float aPos[3], double aNow)
    {
        if (m_have && aNow > m_t)
        {
            const double dt = aNow - m_t;
            if (dt > kRestartGapS)
                m_speed = 0.0f;
            else
            {
                const double dx = aPos[0] - m_p[0], dy = aPos[1] - m_p[1], dz = aPos[2] - m_p[2];
                const double raw = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
                m_speed = static_cast<float>(m_speed + (raw - m_speed) * (1.0 - std::exp(-dt / kTauS)));
            }
        }
        for (int i = 0; i < 3; ++i)
            m_p[i] = aPos[i];
        m_t = aNow;
        m_have = true;
        return m_speed;
    }
    float Speed() const { return m_speed; }

private:
    float m_p[3] = {0, 0, 0};
    double m_t = 0.0;
    float m_speed = 0.0f;
    bool m_have = false;
};

struct SettleConfig
{
    double minTimeS = 1.0;     // never settled before this long after the first sample
    float speedThreshold = 0.15f; // m/s
    double holdS = 0.5;        // the speed must stay below the threshold this long
    double maxTimeS = 5.0;     // forced settle (a body that never calms down)
};

// Settled = min time passed AND speed below the threshold continuously for holdS; any sample at or above the threshold re-arms the
// hold timer (a late impact). The maximum forces it. The script sends the disable event when this turns true.
class SettleDetector
{
public:
    explicit SettleDetector(SettleConfig aCfg = {}) : m_cfg(aCfg) {}
    void Reset() { m_started = false; m_belowSince = -1.0; }
    // Starts the clock at aStart (the episode start); without it the first Update starts it.
    void Begin(double aStart)
    {
        m_started = true;
        m_startAt = aStart;
        m_belowSince = -1.0;
    }
    bool Update(double aNow, float aSpeed)
    {
        if (!m_started)
        {
            m_started = true;
            m_startAt = aNow;
            m_belowSince = -1.0;
        }
        const double age = aNow - m_startAt;
        if (age >= m_cfg.maxTimeS)
            return true;
#if UTPP_RAGDOLL_EPISODE_MUTATION == 3
        if (aSpeed < m_cfg.speedThreshold && m_belowSince < 0.0) // mutation 3: a late impact does not re-arm the hold timer
            m_belowSince = aNow;
#else
        if (aSpeed >= m_cfg.speedThreshold)
            m_belowSince = -1.0;
        else if (m_belowSince < 0.0)
            m_belowSince = aNow;
#endif
        return age >= m_cfg.minTimeS && m_belowSince >= 0.0 && aNow - m_belowSince >= m_cfg.holdS;
    }

private:
    SettleConfig m_cfg;
    bool m_started = false;
    double m_startAt = 0.0, m_belowSince = -1.0;
};

// ------------------------------------------------------------------------------------------------ pelvis telemetry
struct PelvisSnapshot
{
    float pos[3] = {0, 0, 0};     // Hips, model space after FK
    float normal[3] = {0, 0, 0};  // belly normal, signed against the standing reference
    float speed = 0.0f;           // m/s, smoothed
    int64_t ticks = 0;
    bool valid = false;
};

inline bool BellyNormal(const float aHips[3], const float aSpine[3], const float aLUp[3], const float aRUp[3], float (&aOut)[3])
{
    const float u[3] = {aSpine[0] - aHips[0], aSpine[1] - aHips[1], aSpine[2] - aHips[2]};
    const float r[3] = {aRUp[0] - aLUp[0], aRUp[1] - aLUp[1], aRUp[2] - aLUp[2]};
    float n[3] = {u[1] * r[2] - u[2] * r[1], u[2] * r[0] - u[0] * r[2], u[0] * r[1] - u[1] * r[0]};
    const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (!(len > 1e-6f) || !std::isfinite(len))
        return false;
    for (int i = 0; i < 3; ++i)
        aOut[i] = n[i] / len;
    return true;
}

// 1 face up (belly normal up), 0 face down, -1 on the side / unknown. aNormalZ already signed against the standing pose.
inline int ClassifyFaceUp(float aNormalZ, float aDeadZone = 0.5f)
{
    if (!std::isfinite(aNormalZ))
        return -1;
    if (aNormalZ >= aDeadZone)
        return 1;
    if (aNormalZ <= -aDeadZone)
        return 0;
    return -1;
}

// One writer at a time (try-lock, a contended sample is dropped), any number of readers (seqlock). The writer smooths the speed
// from the samples of the full player descriptor only, so two tracks never interleave.
class PelvisTracker
{
public:
    static constexpr double kTauS = 0.1;       // speed smoothing time constant
    static constexpr double kRestartGapS = 0.3; // a longer gap restarts the filter
    static constexpr double kFreshS = 0.5;      // age limit of a readable sample

    // A standing-pose sample: fixes the sign of the belly normal (model forward = +Y) and marks the reference known.
    void SetReference(const float aHips[3], const float aSpine[3], const float aLUp[3], const float aRUp[3])
    {
        float n[3];
        if (!BellyNormal(aHips, aSpine, aLUp, aRUp, n))
            return;
        m_sign.store(n[1] >= 0.0f ? 1.0f : -1.0f, std::memory_order_relaxed);
        m_signSet.store(true, std::memory_order_relaxed);
    }
    bool HasReference() const { return m_signSet.load(std::memory_order_relaxed); }
    float ReferenceSign() const { return m_sign.load(std::memory_order_relaxed); } // +1 / -1; valid when HasReference()

    void Feed(const float aHips[3], const float aSpine[3], const float aLUp[3], const float aRUp[3], int64_t aTicks, int64_t aHz)
    {
        if (m_writer.test_and_set(std::memory_order_acquire))
            return;
        float n[3] = {0, 0, 0};
        const bool nOk = BellyNormal(aHips, aSpine, aLUp, aRUp, n);
        const float sign = m_sign.load(std::memory_order_relaxed);
        float speed = m_speed;
        if (m_havePrev && aHz > 0 && aTicks > m_prevTicks)
        {
            const double dt = static_cast<double>(aTicks - m_prevTicks) / static_cast<double>(aHz);
            if (dt > kRestartGapS)
                speed = 0.0f;
            else
            {
                const double dx = aHips[0] - m_prev[0], dy = aHips[1] - m_prev[1], dz = aHips[2] - m_prev[2];
                const double raw = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
                const double a = 1.0 - std::exp(-dt / kTauS);
                speed = static_cast<float>(m_speed + (raw - m_speed) * a);
            }
        }
        m_speed = speed;
        for (int i = 0; i < 3; ++i)
            m_prev[i] = aHips[i];
        m_prevTicks = aTicks;
        m_havePrev = true;
        m_seq.fetch_add(1, std::memory_order_acq_rel); // odd: writing
        for (int i = 0; i < 3; ++i)
        {
            m_pub.pos[i] = aHips[i];
            m_pub.normal[i] = nOk ? n[i] * sign : 0.0f;
        }
        m_pub.speed = speed;
        m_pub.ticks = aTicks;
        m_pub.valid = true;
        m_nValid = nOk;
        m_seq.fetch_add(1, std::memory_order_release); // even: done
        m_writer.clear(std::memory_order_release);
    }

    // Latest sample when it is fresh (age < kFreshS at aNowTicks). The belly normal is zero when it could not be built.
    bool Read(PelvisSnapshot& aOut, int64_t aNowTicks, int64_t aHz) const
    {
        for (int attempt = 0; attempt < 4; ++attempt)
        {
            const uint32_t s0 = m_seq.load(std::memory_order_acquire);
            if (s0 & 1u)
                continue;
            PelvisSnapshot c = m_pub;
            const bool nOk = m_nValid;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (m_seq.load(std::memory_order_relaxed) != s0)
                continue;
            if (!c.valid || aHz <= 0 || aNowTicks < c.ticks ||
                static_cast<double>(aNowTicks - c.ticks) / static_cast<double>(aHz) > kFreshS)
                return false;
            if (!nOk)
            {
                c.normal[0] = c.normal[1] = c.normal[2] = 0.0f;
            }
            aOut = c;
            return true;
        }
        return false;
    }

    // 1 / 0 / -1 for a snapshot read above (-1 when the normal is missing or there is no standing reference).
    int FaceUp(const PelvisSnapshot& aSnap) const
    {
        if (!m_signSet.load(std::memory_order_relaxed) || (aSnap.normal[0] == 0.0f && aSnap.normal[1] == 0.0f && aSnap.normal[2] == 0.0f))
            return -1;
        return ClassifyFaceUp(aSnap.normal[2]);
    }

private:
    std::atomic_flag m_writer = {};
    std::atomic<uint32_t> m_seq{0};
    std::atomic<float> m_sign{1.0f};
    std::atomic<bool> m_signSet{false};
    PelvisSnapshot m_pub;
    bool m_nValid = false;
    // writer-only state
    float m_prev[3] = {0, 0, 0};
    int64_t m_prevTicks = 0;
    float m_speed = 0.0f;
    bool m_havePrev = false;
};
} // namespace utpp::ragdoll

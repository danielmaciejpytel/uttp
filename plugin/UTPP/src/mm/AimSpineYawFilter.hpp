#pragma once
// UTPP motion matching runtime - continuity of the aim-offset spine yaw / sweep (fix batch 2, group AO).
// Header-only, no game types, no pose data. Used by motionpose::PoseEvaluator::ApplyAimOffset (MotionPose.hpp).
//
// Why (rec 2026-10-21 12:15): the AO spine yaw was clamp(atan2 yaw, +-60) with a hard gate on `haveAim`.
//   (A) the target yaw wraps at +-180 (camera behind the visual root): the clamped value jumps +60 -> -60 in one frame,
//       i.e. 120 deg of spine twist (30 deg on each of Spine..Spine3), 11 events in the recording;
//   (B) `haveAim` drops in one frame (end of the hipfire hold, AIM off, reload start / end of the aim hold) while the
//       overlay Aiming weight is still ~1: the whole AO (<= 60 deg of spine twist and the pitch sweep) vanishes in one
//       frame, 17 events + two head pops.
// The filter turns both into continuous signals:
//   (A) the side of the clamp is kept while the target is behind the back (hysteresis, |yaw| > kAoBackHysteresisDeg), and a
//       JUMP of the target (more than kAoJumpDeg in one frame: the side change, a root discontinuity) or the return of the
//       target after the latch is caught up at kAoYawRateDegPerSec; ordinary tracking is exact (clamp(raw) bit for bit);
//   (B) the last angles are latched when the target is lost; the caller keeps applying them with the overlay's own
//       fade-out weight (OverlayMachine::AimingWeight, 0.5 s Aiming -> Ready) times `release`, which goes to zero before
//       kAoLatchMaxAgeSeconds, so even an Aiming weight that stays at 1 cannot end the AO with a step.
// A frame with dt <= 0 (the offline tests probe the same frame repeatedly) is a pass-through of the raw angles without
// touching the state, exactly the former behaviour; once the filter has state it holds its last output instead.
//
// Mutations (UTPP_POSEV2_AO_MUTATION, shared with MotionPose.hpp): 5 = no continuity (raw clamp, no hysteresis / rate limit);
// 6 = no latch (the angles end with the target, the former hard gate); 7 = a handback fades like a lost target (0.5 s); 9 = a target that returns after the latch is followed
// at once; 10 = every live frame is rate limited (the first version: fast tracking lags behind the crosshair); 11 = the
// pitch of a returning target is followed at once (sweep jump); 12 = a restart (weapon class, long frame) resets the filter;
// 15 = the jump size is a constant 45 deg per frame (low frame rates limit the camera again); 14 = aoScale only hides the
// latched AO, it does not use it up (a cancelled reload brings it back); 19 = angles from the world aim target still run the
// root-relative jump band (a recentred root leaves the spine behind the still crosshair).
#include <algorithm>
#include <cmath>

namespace utpp::mm::motionpose
{
// Fastest change of the applied spine yaw while it CATCHES UP with a jumped target (deg/s): 8 deg/frame at 60 fps, the full
// clamp swing of 120 deg takes 0.25 s. Ordinary tracking is never limited: while the target moves like a camera (steps
// below kAoJumpDeg per frame, the recording has up to 21.6 deg = 1300 deg/s) the output is clamp(raw) bit for bit, as it was
// before the filter; a limited fast tracking would leave the weapon up to 64 deg behind the crosshair (109.41-109.52 s).
inline constexpr float kAoYawRateDegPerSec = 480.0f;
// A target step larger than the jump size in one frame is not camera motion but a jump (the side of the clamp changed behind
// the back: 120 deg; a root / pose discontinuity; a near-vertical target): the output catches up at kAoYawRateDegPerSec.
// The jump size follows the frame time, because the camera moves the same deg/s at any frame rate: kAoJumpRateDegPerSec * dt
// (the recording has camera flicks of up to 1300 deg/s = 21.7 deg at 60 fps, 43 at 30 fps, 65 at 20 fps or in a single 50 ms
// frame), between kAoJumpDeg (60 fps and above) and kAoJumpMaxDeg (below the 120 deg swing of the side change at any frame rate).
inline constexpr float kAoJumpDeg = 45.0f;
inline constexpr float kAoJumpRateDegPerSec = 1500.0f;
inline constexpr float kAoJumpMaxDeg = 90.0f;
inline float AoJumpDeg(float aDt)
{
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 15
    (void)aDt; // mutation 15: the first version, a constant 45 deg per frame and no explicit side change
    return kAoJumpDeg;
#else
    return std::max(kAoJumpDeg, std::min(kAoJumpMaxDeg, kAoJumpRateDegPerSec * aDt));
#endif
}
// The pitch of the sweep catches up like the yaw (a jump of more than kAoJumpDeg, or the target returning after the latch),
// but slower: 1 deg of pitch moves WeaponRight by ~1.1 cm (AO share 94 cm over 65 deg, MotionPoseTest 7g), 4 deg/frame = 4.4 cm.
inline constexpr float kAoPitchSlewDegPerSec = 240.0f;
// |target yaw| beyond this = behind the back: the side of the clamp (+60 / -60) chosen before is kept. 150 deg leaves a
// 30 deg dead sector around 180 deg where the side cannot flip.
inline constexpr float kAoBackHysteresisDeg = 150.0f;
// Latched angles survive a lost target for at most this long (longer than OverlayTimes::aimingToReady 0.5 s) ...
inline constexpr float kAoLatchMaxAgeSeconds = 0.6f;
// ... and `release` falls linearly to 0 over the last part of it (a no-op while the machine fades the weight in 0.5 s).
inline constexpr float kAoReleaseTailSeconds = 0.25f;
// While the game owns the upper body (handback: `aSuppress`) the AO is not applied any more, but it is not cut either: a cut
// would pop the MM upper body in the frames in which the published pose still carries its share (the envelope of the handback),
// the 60 deg jump of the former hard gate. The latched angles fade out in this time (the existing 0.15 s mask fade).
inline constexpr float kAoHandbackFadeSeconds = 0.15f;
// `release` climbs back at this rate when a target returns while the latch still fades (no snap back to full AO).
inline constexpr float kAoReacquireSeconds = 0.15f;
// A frame this long (pause, hitch, level load; the driver caps dt at 0.25 s) and a change of the weapon class do NOT restart the
// filter: the side, the angles and a running latch are kept (a restart picks the side of the raw yaw and sets the angles at once,
// the 120 deg jump the filter exists to remove, and a restart inside a fading latch ends the AO in one frame). They only start
// a catch-up of the yaw and the pitch from the kept angles.
inline constexpr float kAoGapSeconds = 0.25f;
// Longest time step of a catch-up: a long frame cannot turn the catch-up into a one-frame swing (16 deg of yaw, 8 of pitch).
inline constexpr float kAoMaxCatchUpSeconds = 1.0f / 30.0f;
inline constexpr float kAoFilterZeroWeight = 1e-5f; // = ov::kZeroWeight (UE ZERO_ANIMWEIGHT_THRESH)

class AimSpineYawFilter
{
public:
    struct Output
    {
        float yawDeg = 0.0f;   // clamped spine yaw, before the weight (+ = to the left, the ApplySpineYaw sense)
        float pitchDeg = 0.0f; // sweep pitch (the live pitch while a target drives it, the latched one after)
        float release = 1.0f;  // 1 while a target drives the angles; < 1 while only the latched angles fade out
        bool live = false;     // a target drove this frame
    };

    void Reset() { *this = AimSpineYawFilter{}; }
    // The part of a latched AO that the reload hand-over (aoScale) has hidden is USED UP: the release of the latch never goes
    // above the smallest scale seen, so a reload that ends or is cancelled before the latch is gone cannot bring the hidden
    // part back in one frame. A used-up latch (cap 0) is dropped. No effect while a target drives the angles.
    void CapRelease(float aCap)
    {
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 14
        (void)aCap; // mutation 14: aoScale only hides the latch, it does not use it up
#else
        if (!m_valid || !m_latched)
            return;
        m_release = std::min(m_release, aCap);
        if (!(m_release > 0.0f))
            Reset();
#endif
    }
    bool Valid() const { return m_valid; }
    float Yaw() const { return m_yaw; }

    // aHaveAim: a finite target exists this frame. aWeight: OverlayMachine::AimingWeight (0..1). aContext: anything that
    // identifies the weapon class (a change starts a catch-up, the filter is kept). aSuppress: the game owns the upper body, the
    // target is ignored and the latched angles fade out in kAoHandbackFadeSeconds. Returns true when the angles of aOut are to
    // be applied (with the weight min(aWeight, aOut.release)); false = no AO this frame.
    // aWorldTracked: the angles come from the world aim target (WorldAimTargetFilter, which already catches up jumps of the target in
    // world space) and were recomputed for this frame's root. A step of them is then a turn of the root, which the angles compensate in
    // the same frame: no jump band on them (the side change of the clamp, a re-acquire after the latch, a weapon class change and a
    // gap still catch up).
    bool Step(bool aHaveAim, float aWeight, float aRawYawDeg, float aRawPitchDeg, float aDt, float aClampDeg,
              int aContext, bool aSuppress, Output& aOut, bool aWorldTracked = false)
    {
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 19
        aWorldTracked = false; // mutation 19: world-tracked angles still run the root-relative jump band
#endif
        const bool rootJumpBand = !aWorldTracked;
        (void)rootJumpBand; // unused in the mutation builds that return early
        const bool live = aHaveAim && !aSuppress && std::isfinite(aRawYawDeg) && std::isfinite(aRawPitchDeg);
        const float clampedRaw = std::max(-aClampDeg, std::min(aClampDeg, aRawYawDeg));
        if (!(aDt > 0.0f) || !std::isfinite(aDt))
        {
            // repeated / probe frame: no time passes, the state does not move
            if (m_valid)
            {
                aOut = Output{m_yaw, m_pitch, m_release, live};
                return true;
            }
            if (!live)
                return false;
            aOut = Output{clampedRaw, aRawPitchDeg, 1.0f, true};
            return true;
        }
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 12
        if (m_valid && (aContext != m_context || aDt >= kAoGapSeconds))
            Reset(); // mutation 12: the first version, a restart picks the side of the raw yaw and sets the angles at once
        const float catchUpDt = aDt;
        (void)catchUpDt;
#else
        if (m_valid && (aContext != m_context || aDt >= kAoGapSeconds))
        {
            m_context = aContext;
            m_yawSlew = m_pitchSlew = true;
        }
        const float catchUpDt = std::min(aDt, kAoMaxCatchUpSeconds);
        (void)catchUpDt; // unused in the mutation builds that return early
#endif
        if (live)
        {
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 5
            // mutation 5: the former behaviour, the raw clamp every frame
            m_valid = true;
            m_context = aContext;
            m_yaw = clampedRaw;
            m_side = aRawYawDeg < 0.0f ? -1.0f : 1.0f;
            m_pitch = aRawPitchDeg;
            m_release = 1.0f;
            m_age = 0.0f;
            aOut = Output{m_yaw, m_pitch, 1.0f, true};
            return true;
#else
            if (!m_valid)
            {
                m_valid = true;
                m_context = aContext;
                m_side = aRawYawDeg < 0.0f ? -1.0f : 1.0f;
                m_yaw = clampedRaw;
                m_pitch = aRawPitchDeg;
                m_release = 1.0f;
            }
            else
            {
                const float prevSide = m_side;
                (void)prevSide; // unused in the mutation 15 build
                float effective = aRawYawDeg;
                if (std::fabs(aRawYawDeg) > kAoBackHysteresisDeg)
                    effective = m_side * std::fabs(aRawYawDeg); // behind the back: keep the side
                else if (aRawYawDeg > 0.0f)
                    m_side = 1.0f;
                else if (aRawYawDeg < 0.0f)
                    m_side = -1.0f;
                const float target = std::max(-aClampDeg, std::min(aClampDeg, effective));
                const float d = target - m_yaw;
                const float jumpDeg = AoJumpDeg(aDt);
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 15
                const bool sideChange = false;
#else
                // the side of the clamp changed after the target left the back sector: always a jump, at any frame time
                const bool sideChange = m_side != prevSide && std::fabs(aRawYawDeg) > 90.0f;
#endif
                (void)sideChange; // unused in the mutation 10 build
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 10
                m_yawSlew = true; // mutation 10: the first version, every live frame is rate limited
#elif defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 9
                if (sideChange || std::fabs(d) > jumpDeg) // mutation 9: a returning target is followed at once
                    m_yawSlew = true;
#else
                if (m_latched || sideChange || (rootJumpBand && std::fabs(d) > jumpDeg)) // a jump, or the target returns after the latch
                    m_yawSlew = true;
#endif
                if (m_yawSlew)
                {
                    const float maxStep = kAoYawRateDegPerSec * catchUpDt;
                    if (std::fabs(d) <= maxStep)
                    {
                        m_yaw = target;
                        m_yawSlew = false;
                    }
                    else
                        m_yaw += std::copysign(maxStep, d);
                }
                else
                    m_yaw = target;
                const float dp = aRawPitchDeg - m_pitch;
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 9
                if (std::fabs(dp) > jumpDeg)
                    m_pitchSlew = true;
#elif defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 11
                m_pitchSlew = false; // mutation 11: the pitch follows the target at once, also after the latch (sweep jump)
#else
                if (m_latched || (rootJumpBand && std::fabs(dp) > jumpDeg))
                    m_pitchSlew = true;
#endif
                if (m_pitchSlew)
                {
                    const float maxPitchStep = kAoPitchSlewDegPerSec * catchUpDt;
                    if (std::fabs(dp) <= maxPitchStep)
                    {
                        m_pitch = aRawPitchDeg;
                        m_pitchSlew = false;
                    }
                    else
                        m_pitch += std::copysign(maxPitchStep, dp);
                }
                else
                    m_pitch = aRawPitchDeg;
                m_release = std::min(1.0f, m_release + aDt / kAoReacquireSeconds);
            }
            m_latched = false;
            m_age = 0.0f;
            m_suppressAge = 0.0f;
            aOut = Output{m_yaw, m_pitch, m_release, true};
            return true;
#endif
        }
        if (!m_valid)
            return false;
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 6
        (void)aWeight;
        Reset(); // mutation 6: the angles end with the target (the former hard gate)
        return false;
#else
        if (!(aWeight > kAoFilterZeroWeight))
        {
            Reset();
            return false;
        }
        m_latched = true;
        m_age += aDt;
        float cap = std::max(0.0f, std::min(1.0f, (kAoLatchMaxAgeSeconds - m_age) / kAoReleaseTailSeconds));
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 7
        (void)aSuppress; // mutation 7: a handback fades like any lost target (0.5 s), not in kAoHandbackFadeSeconds
#else
        if (aSuppress)
        {
            m_suppressAge += aDt;
            cap = std::min(cap, std::max(0.0f, 1.0f - m_suppressAge / kAoHandbackFadeSeconds));
        }
        else
            m_suppressAge = 0.0f;
#endif
        m_release = std::min(m_release, cap);
        if (!(m_release > 0.0f))
        {
            Reset();
            return false;
        }
        aOut = Output{m_yaw, m_pitch, m_release, false};
        return true;
#endif
    }

private:
    bool m_valid = false;
    int m_context = 0;
    float m_side = 1.0f;
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_release = 1.0f;
    float m_age = 0.0f;
    bool m_latched = false; // the last frame had no target: the next live frame is a re-acquire
    bool m_yawSlew = false; // the yaw is catching up with a jumped target
    bool m_pitchSlew = false; // the sweep pitch is catching up
    float m_suppressAge = 0.0f; // time spent under a handback
};

// ------------------------------------------------------------------------------------------------ world aim target smoothing
// Optional (flag WorldAimTargetParams::enabled, default off: the caller does not use it and the AO path is unchanged).
// Smoothing the root-relative aim angles makes the spine lag whenever the ROOT turns (a turn in place, a root snap), although the
// crosshair did not move. This filter smooths the aim direction in WORLD space instead (exponential, smoothHz; 0 = exact) and the
// caller recomputes the root-relative angles from the smoothed world direction every frame (RootRelativeYawDeg), so a root turn is
// compensated in the same frame and only camera motion is smoothed. Jumps of the world target (a step above AoJumpDeg(dt) in one
// frame) are caught up at kAoYawRateDegPerSec / kAoPitchSlewDegPerSec, the band rule of AimSpineYawFilter applied to the world
// target. A lost target holds the last world direction (the latch of AimSpineYawFilter fades it); the next live frame re-acquires
// with a catch-up. dt <= 0 is a pass-through that keeps the state. The root-relative angles taken from it reach AimSpineYawFilter
// marked world tracked: a recentred root (a reseed after a teleport) moves them at once, as the crosshair did not move.
struct WorldAimTargetParams
{
    bool enabled = false;
    float smoothHz = 0.0f; // exponential rate of the world aim direction (1/s); 0 = no smoothing (exact tracking)
};

inline float WrapDeg180(float aDeg)
{
    if (!std::isfinite(aDeg))
        return 0.0f;
    float r = std::remainder(aDeg, 360.0f);
    return r <= -180.0f ? r + 360.0f : r;
}

// Root-relative yaw (deg, + = to the left) of a world yaw for a root of world yaw aRootYawDeg.
inline float RootRelativeYawDeg(float aWorldYawDeg, float aRootYawDeg) { return WrapDeg180(aWorldYawDeg - aRootYawDeg); }

class WorldAimTargetFilter
{
public:
    void Reset() { *this = WorldAimTargetFilter{}; }
    bool Valid() const { return m_valid; }
    float YawDeg() const { return m_yaw; }
    float PitchDeg() const { return m_pitch; }

    // aLive: a finite world target this frame. Returns true when aYawDeg / aPitchDeg hold a world direction to use.
    bool Step(bool aLive, float aWorldYawDeg, float aWorldPitchDeg, float aDt, const WorldAimTargetParams& aP, float& aYawDeg,
              float& aPitchDeg)
    {
        const bool live = aLive && std::isfinite(aWorldYawDeg) && std::isfinite(aWorldPitchDeg);
        if (!live)
        {
            if (m_valid)
                m_lost = true;
            aYawDeg = m_yaw;
            aPitchDeg = m_pitch;
            return m_valid;
        }
        if (!m_valid || !(aDt > 0.0f) || !std::isfinite(aDt))
        {
            if (!m_valid)
            {
                m_valid = true;
                m_yaw = WrapDeg180(aWorldYawDeg);
                m_pitch = aWorldPitchDeg;
            }
            aYawDeg = m_valid && !(aDt > 0.0f) ? m_yaw : WrapDeg180(aWorldYawDeg);
            aPitchDeg = m_valid && !(aDt > 0.0f) ? m_pitch : aWorldPitchDeg;
            return true;
        }
        const float dy = WrapDeg180(aWorldYawDeg - m_yaw);
        const float dp = aWorldPitchDeg - m_pitch;
        const float jump = AoJumpDeg(aDt);
        if (m_lost || std::fabs(dy) > jump)
            m_yawSlew = true;
        if (m_lost || std::fabs(dp) > jump)
            m_pitchSlew = true;
        m_lost = false;
        const float catchUpDt = std::min(aDt, kAoMaxCatchUpSeconds);
        const float k = aP.smoothHz > 0.0f ? 1.0f - std::exp(-aP.smoothHz * aDt) : 1.0f;
        if (m_yawSlew)
        {
            const float maxStep = kAoYawRateDegPerSec * catchUpDt;
            if (std::fabs(dy) <= maxStep)
            {
                m_yaw = WrapDeg180(aWorldYawDeg);
                m_yawSlew = false;
            }
            else
                m_yaw = WrapDeg180(m_yaw + std::copysign(maxStep, dy));
        }
        else
            m_yaw = WrapDeg180(m_yaw + dy * k);
        if (m_pitchSlew)
        {
            const float maxStep = kAoPitchSlewDegPerSec * catchUpDt;
            if (std::fabs(dp) <= maxStep)
            {
                m_pitch = aWorldPitchDeg;
                m_pitchSlew = false;
            }
            else
                m_pitch += std::copysign(maxStep, dp);
        }
        else
            m_pitch += dp * k;
        aYawDeg = m_yaw;
        aPitchDeg = m_pitch;
        return true;
    }

private:
    bool m_valid = false;
    bool m_lost = false;
    bool m_yawSlew = false;
    bool m_pitchSlew = false;
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
};
} // namespace utpp::mm::motionpose

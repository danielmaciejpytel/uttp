// Offline planar adaptation of the verified motion OffsetRoot contract.
// Primary: UE5.8.3 AnimNode_OffsetRootBone.cpp (mode selection/damping),
// UnrealMathUtility.h InvExpApprox; bindings documented in sourcepack-offsetroot-contract.
// Not integrated. No 3D root attributes, collision, graph consumption or teleport policy.
//
// UTPP policy on top of the contract (deliberate deviations, all tested in tests/MotionOffsetRootTest.cpp and
// tests/RestOffsetCollapseTest.cpp; Step fields / the macros below switch each of them off):
//  1. The montage rotation Release (half-life 0.1 s) is bounded by kReleaseMaxRate. motion has no bound; with a large hidden yaw
//     offset it turned the visible body ~800 deg/s without footwork (recording 2026-10-21 12:15). Offsets below ~43 deg release
//     exactly like motion.
//  2. Rest recentering (Step::restRecenter, opt-in) - the FALLBACK of the rest turn-in-place (MotionAdapter restTurn: the matcher turns a
//     hidden offset with authored footwork, the root absorbs the clip's yaw). While V stands, grounded and nothing owns the facing
//     (no explicit aim, no montage), an offset beyond the knee that no authored turn has taken for kRestFallbackDelay (0.5 s) is
//     folded back at <= kRestRate with a soft knee. It turns the root without footwork (the FootLocker lets the feet slide, about
//     r * angle), so it only runs when the matcher did not pick a turn. Losing the permission (aim, stick, montage) slews the speed
//     down over kRestExit instead of stopping in one frame.
// (RootMontage.hpp: a reload that began on the spot is not montage once V moves.)
#pragma once
#include "MMMath.hpp"
#include <algorithm>
#include <limits>

#ifndef UTPP_ROOT_RELEASE_MUTATION
#define UTPP_ROOT_RELEASE_MUTATION 0 // 1 = unbounded montage rotation release (motion): tests must FAIL
#endif
#ifndef UTPP_ROOT_REST_MUTATION
#define UTPP_ROOT_REST_MUTATION 0 // 1 = rest recentering never runs: tests must FAIL
#endif

namespace utpp::mm::motionRoot
{
inline constexpr float kDegToRad = math::kPi / 180.f;
// Authored standing turn-in-place footwork in the recording (visible yaw rate, capsule static, reload frames excluded):
// p50 45, p75 120, p90 210, p95 301, p99 357 deg/s.
inline constexpr float kReleaseMaxRate = 300.f * kDegToRad;  // p95: faster than any authored turn is a spin
inline constexpr float kRestRate = 120.f * kDegToRad;        // p75: the pace of a Turn_135 (mean 101-118 deg/s)
inline constexpr float kRestKneeLow = 40.f * kDegToRad;      // below ShouldTurnInPlace's 50 deg: AO / TIP own it, nothing moves
inline constexpr float kRestKneeHigh = 90.f * kDegToRad;     // full rate from here
inline constexpr float kRestRamp = 0.25f;                    // s: speed ramp-up, no jerk at the start
inline constexpr float kRestExit = 0.12f;                    // s: speed ramp-down when the permission is lost
inline constexpr float kRestFallbackDelay = 0.5f;            // s: the matcher's turn-in-place gets this long to take the offset
inline constexpr float kRestTurnGrace = 1.6f;                // s: a turn-in-place clip may idle this long (Turn_180 ~1.7 s, turn in the first 0.5 s)
inline constexpr float kRestAuthoredRate = 10.f * kDegToRad; // idle loops drift <= 3 deg/s, TIP clips are far above
inline constexpr float kRestAuthoredHold = 0.25f;            // s after the last authored turn: the clip owns the yaw
struct Step
{
    float dt = 0;
    math::Vec3 capsuleDeltaCurrent{}; // actual displacement in CURRENT capsule axes, metres
    float capsuleYawDelta = 0;
    math::Vec3 animationDeltaVisual{}; // extracted delta in PREVIOUS visual-root axes
    float animationYawDelta = 0;
    bool movingGround = false, montage = false;
    // Explicit gameplay aim owns facing, independently of montage translation.
    bool aimFacing = false;
    // UTPP policy (see the header comment). restRecenter is the caller's "grounded, standing or crouching, no traversal" permission.
    bool restRecenter = false;
    // The matcher plays from a turn-in-place database (also in the still phases of a clip: standing clips put the whole root turn into
    // the first 0.5 s and idle on): it owns the yaw for kRestTurnGrace without authored progress, the fallback stays out of its way.
    // Longer than that the "turn" is only the database (e.g. aim state with a stale camera: no turn is asked) and the fallback runs.
    bool turnPlaying = false;
    float releaseMaxRate = kReleaseMaxRate; // rad/s; <= 0 or non-finite = unbounded (the motion contract)
    // Rate bound of the moving-aim lock (rad/s). 0 / non-finite = the original 540 deg/s (ADS while moving). Shooter mode / H2 safeguard
    // pass their own, lower bound (MotionDriver kShooterModeLockRate) so a root left far from the capsule is not thrown onto it in one step.
    float lockMaxRate = 0.f;
    // Shooter mode in the air (no ground contact, no slide / traversal): the visual root follows the actor with the same bounded-follow law at
    // lockMaxRate (motion: rotation in the air 200 deg/s), so the offset at touchdown stays small. Needs lockMaxRate > 0.
    bool airFollow = false;
};
struct State
{
    float x = 0, y = 0, yaw = 0; // visual minus capsule, CURRENT capsule axes
    float correction = 0, maxCorrection = 0;
    bool translationRelease = true, rotationRelease = false;
    // rest recentering: speed (rad/s, towards zero), seconds the offset waited for a turn, authored-yaw hold (s), yaw moved this step (rad)
    float restSpeed = 0, restIdle = 0, restHold = 0, restStep = 0, turnAge = 0;
};
class OffsetRoot
{
  public:
    void Reset()
    {
        m = {};
    }
    bool Seed(float x, float y, float yaw)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
            return false;
        m = {};
        m.x = x;
        m.y = y;
        m.yaw = math::WrapAngle(yaw);
        return true;
    }
    float X() const
    {
        return m.x;
    }
    float Y() const
    {
        return m.y;
    }
    float Yaw() const
    {
        return m.yaw;
    }
    const State &Output() const
    {
        return m;
    }
    bool Update(float dt, float capCurrentX, float capCurrentY, float capYaw, float animPreviousVisualX,
                float animPreviousVisualY, float animYaw, bool movingGround, bool montage, bool aimFacing = false)
    {
        return Update(Step{dt,
                           {capCurrentX, capCurrentY, 0},
                           capYaw,
                           {animPreviousVisualX, animPreviousVisualY, 0},
                           animYaw,
                           movingGround,
                           montage,
                           aimFacing});
    }
    // Exposed for deterministic replay, not an engine installation/tuning interface.
    static float DampingAlpha(float halfLife, float dt)
    {
        if (!(std::isfinite(dt) && dt > 0))
            return 0;
        const float x = math::kLn2 * dt / (std::fmax(halfLife, dt) + 1e-8f);
        const float denominator = 1 + 1.00746054f * x + .45053901f * x * x + .25724632f * x * x * x;
        return std::clamp(1 - 1 / denominator, 0.f, 1.f);
    }
    // Bounded follow of the actor: the visual root turns with the capsule (zero lag while the capsule turns slower than aRate) and pulls the
    // remaining offset to zero (half-life 0.1 s), but its WORLD turn per step never exceeds aRate * dt. A capsule that jumps (a mouse flick, a
    // one-feed 180) leaves the excess in the offset (visual minus capsule), which the pull then resolves at the bounded rate. Returns the new offset.
    static float BoundedFollow(float aOffset, float aCapsuleDelta, float aDt, float aRate)
    {
        const float lim = aRate * aDt;
        const float step = std::clamp(aCapsuleDelta - aOffset * DampingAlpha(.1f, aDt), -lim, lim);
        return math::WrapAngle(aOffset + step - aCapsuleDelta);
    }
    bool Update(const Step &s)
    {
        if (!std::isfinite(s.dt) || s.dt < 0)
            return false;
        if (s.dt == 0)
            return true;
        if (!math::IsFinite(s.capsuleDeltaCurrent) || !math::IsFinite(s.animationDeltaVisual) ||
            !std::isfinite(s.capsuleYawDelta) || !std::isfinite(s.animationYawDelta))
            return false;
        State next = m;
        next.restStep = 0;
        next.restSpeed = 0; // kept only by the accumulate branch below (aim-moving / release have their own damped rotation)
        next.restIdle = 0;
        // An authored turn (TIP, idle break, start clip) in progress, or one that ended <= 0.25 s ago, owns the yaw.
        const bool authoredYaw = std::fabs(s.animationYawDelta) > kRestAuthoredRate * s.dt;
        next.turnAge = s.turnPlaying && !authoredYaw ? m.turnAge + s.dt : 0.f; // seconds a turn database has played without turning
        next.restHold = authoredYaw || (s.turnPlaying && next.turnAge <= kRestTurnGrace) ? kRestAuthoredHold
                                                                                         : std::fmax(0.f, m.restHold - s.dt);
        next.translationRelease = !s.movingGround || s.montage;
        const bool aimMoving = s.aimFacing && s.movingGround;
        next.rotationRelease = aimMoving || (s.montage && !s.aimFacing);
        float x = m.x, y = m.y;
        if (!next.translationRelease)
        {
            float ax = 0, ay = 0;
            math::Rot2(m.yaw, s.animationDeltaVisual.x, s.animationDeltaVisual.y, ax, ay);
            x += ax;
            y += ay;
        }
        float nx = 0, ny = 0;
        math::Rot2(-s.capsuleYawDelta, x, y, nx, ny);
        // Release follows component displacement in world space; no subtraction of
        // capsule delta remains in the relative offset. Interpolate consumes animation.
        if (!next.translationRelease)
        {
            nx -= s.capsuleDeltaCurrent.x;
            ny -= s.capsuleDeltaCurrent.y;
        }
        const float alpha = DampingAlpha(next.translationRelease ? .1f : .3f, s.dt);
        float dx = -nx * alpha, dy = -ny * alpha;
        next.maxCorrection = std::numeric_limits<float>::infinity();
        if (s.movingGround)
        {
            next.maxCorrection = .5f * std::sqrt(s.animationDeltaVisual.x * s.animationDeltaVisual.x +
                                                 s.animationDeltaVisual.y * s.animationDeltaVisual.y);
            const float distance = std::sqrt(dx * dx + dy * dy);
            if (distance > next.maxCorrection && distance > 0)
            {
                const float k = next.maxCorrection / distance;
                dx *= k;
                dy *= k;
            }
        }
        next.correction = std::sqrt(dx * dx + dy * dy);
        nx += dx;
        ny += dy;
        const float distance = std::sqrt(nx * nx + ny * ny);
        if (distance > .30f)
        {
            const float k = .30f / distance;
            nx *= k;
            ny *= k;
        }
        next.x = nx;
        next.y = ny;
        if (aimMoving)
        {
            // Moving aim follows the actor. Stationary aim instead consumes
            // authored turn-in-place yaw and cancels capsule rotation below:
            // releasing it here would spin planted feet and erase the heading
            // error that requests the turn animation.
            const bool bounded = std::isfinite(s.lockMaxRate) && s.lockMaxRate > 0;
            const float maxRate = bounded ? s.lockMaxRate : 540.f * math::kPi / 180.f;
            if (bounded)
                next.yaw = BoundedFollow(m.yaw, s.capsuleYawDelta, s.dt, maxRate); // Shooter / H2: the whole world turn of the root is bounded
            else
            {
                const float correction = std::clamp(-m.yaw * DampingAlpha(.1f, s.dt),
                                                     -maxRate * s.dt, maxRate * s.dt);
                next.yaw = m.yaw + correction; // ADS while moving: the root follows the actor at once (the capsule turn is not subtracted)
            }
        }
        else if (s.airFollow && std::isfinite(s.lockMaxRate) && s.lockMaxRate > 0)
        {
            next.yaw = BoundedFollow(m.yaw, s.capsuleYawDelta, s.dt, s.lockMaxRate);
        }
        else if (next.rotationRelease)
        {
            const float released = m.yaw * (1 - DampingAlpha(.1f, s.dt));
#if UTPP_ROOT_RELEASE_MUTATION == 1
            next.yaw = released;
#else
            const float bound = std::isfinite(s.releaseMaxRate) && s.releaseMaxRate > 0 ? s.releaseMaxRate * s.dt
                                                                                        : std::numeric_limits<float>::infinity();
            next.yaw = std::fabs(m.yaw - released) > bound ? m.yaw - std::copysign(bound, m.yaw) : released;
#endif
        }
        else
        {
            next.yaw = math::WrapAngle(m.yaw + s.animationYawDelta - s.capsuleYawDelta);
            // Rest recentering (restHold above: the authored yaw owns the turn while it lasts).
            const bool rest = s.restRecenter && !s.movingGround && !s.aimFacing && !s.montage;
            const float restGate = UTPP_ROOT_REST_MUTATION == 1 ? -1.f : 0.f; // mutation 1: restHold is never < 0, nothing moves
            const float magnitude = std::fabs(next.yaw);
            const bool waiting = rest && next.restHold <= restGate && magnitude > kRestKneeLow;
            next.restIdle = waiting ? m.restIdle + s.dt : 0.f;
            float target = 0;
            if (waiting && next.restIdle >= kRestFallbackDelay)
            {
                const float u = std::clamp((magnitude - kRestKneeLow) / (kRestKneeHigh - kRestKneeLow), 0.f, 1.f);
                target = kRestRate * u * u * (3.f - 2.f * u); // smoothstep: the pull fades out towards the knee
            }
            // speed slew: ramp up over kRestRamp, ramp down over kRestExit (also when the permission is lost: no stop in one frame)
            next.restSpeed = target > m.restSpeed ? std::fmin(target, m.restSpeed + kRestRate / kRestRamp * s.dt)
                                                  : std::fmax(target, m.restSpeed - kRestRate / kRestExit * s.dt);
            const float step = std::fmin(next.restSpeed * s.dt, std::fmax(magnitude - kRestKneeLow, 0.f));
            if (step > 0)
            {
                next.restStep = std::copysign(step, next.yaw);
                next.yaw -= next.restStep;
            }
        }
        if (!std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.yaw))
            return false;
        m = next;
        return true;
    }

  private:
    State m{};
};
} // namespace utpp::mm::motionRoot

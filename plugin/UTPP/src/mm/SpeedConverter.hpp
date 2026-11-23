#pragma once
// Speed converter: turns the game capsule velocity (already on the mapped query scale, cm/s) into the velocity the reference character
// movement would have while chasing it, one feed at a time. The game turns the capsule velocity to the input within a feed and brakes
// a run in ~0.18 s; the reference character turns it along the ground-friction arc (friction 5 per second on the off-direction part),
// accelerates at most at its maximum acceleration and brakes at 500 (input held) / 2000 (no input) cm/s^2. The converted velocity
// feeds the query trajectory (history and prediction start) and the movement predicates, so a game turn looks like the curved turn
// the database clips were recorded with. It converges to the game velocity whenever that velocity holds, so steady loops are unchanged.
#include <algorithm>
#include <cmath>
#include "MotionTrajectory.hpp"

#ifndef UTPP_SPEED_CONVERTER_MUTATION
#define UTPP_SPEED_CONVERTER_MUTATION 0
#endif

namespace utpp::mm::speedconv
{
// MapConfig::speedConverter bits
inline constexpr int kQuery = 1;       // converted velocity for the query trajectory and the predicates
inline constexpr int kStartsStops = 2; // also from rest and after the input is released (else those feeds use the game velocity)
inline constexpr int kLoopRate = 4;    // play rate of loop clips from the capsule speed (capsuleSpeedRate Both, scope loops only)
inline constexpr int kDirectionOnly = 8;  // only the heading is converted, the speed is the game speed (without kStartsStops)
inline constexpr int kSteadyLoopRate = 16; // kLoopRate only while the capsule speed holds (not on starts, stops, speed changes, landings)
// kSteadyLoopRate: the capsule speed counts as steady while its filtered rate of change stays below this (cm/s^2 on the query scale)
inline constexpr float kSteadyAccelCmS2 = 150.0f;
inline constexpr float kSteadyFilterS = 0.1f; // time constant of that filter (s)
// kDirectionOnly: a turn starts when the heading error to the game velocity exceeds kTurnStartDeg (a sharp input change; a smooth curve
// stays below it and passes the game heading unchanged) and ends below kTurnEndDeg
inline constexpr float kTurnStartDeg = 30.0f;
inline constexpr float kTurnEndDeg = 5.0f;

struct Limits
{
    float friction = 5.0f;          // ground friction while the input is held (per second)
    float acceleration = 800.0f;    // cm/s^2, maximum acceleration
    float brakingInput = 500.0f;    // cm/s^2 above the wanted speed while the input is held
    float brakingNoInput = 2000.0f; // cm/s^2 without input
    float maxDivergence = 1200.0f;  // cm/s: farther from the game velocity = reset to it (teleport, collision stop)
    // cos of the largest angle between the converted and the game velocity that is converted; a larger one (a reversal, which the
    // reference passes through a near stop and the pivot picks already cover) resets to the game velocity. -1 = every angle.
    float maxTurnCos = -1.0f;
    // the game speed falling below this share of the previous feed's game speed with the input held is a stall (a wall): reset
    float stallRatio = 0.5f;
};

struct State
{
    bool valid = false;
    motiontraj::V3 vel{};
    float gamePrevSpeed = 0.0f; // the game speed of the previous feed (stall detection)
    float stallSpeed = 0.0f;    // > 0: stalled (the game speed before the stall); the game velocity passes until the speed recovers
    bool turning = false;       // kDirectionOnly: a turn is being converted
    // kSteadyLoopRate: the previous speed (< 0 = none) and the filtered rate of change of the speed
    float steadyPrevSpeed = -1.0f;
    float steadyAccel = 0.0f;
};

// kSteadyLoopRate: one feed of the steady-speed filter. aEligible: on the ground, input held, outside the landing window. Returns true
// while the speed holds (the filtered |d speed / dt| below kSteadyAccelCmS2); any ineligible feed restarts the filter.
inline bool UpdateSteady(State& s, float aSpeedCmS, float aDt, bool aEligible)
{
    if (!aEligible || !(aDt > 0.0f) || !std::isfinite(aSpeedCmS) || s.steadyPrevSpeed < 0.0f)
    {
        s.steadyPrevSpeed = aEligible && std::isfinite(aSpeedCmS) ? aSpeedCmS : -1.0f;
        s.steadyAccel = 1.0e9f; // not steady until the filter has seen the speed hold
        return false;
    }
    // signed rate of change, filtered (the feed-to-feed jitter of a held speed averages out)
    const float a = (aSpeedCmS - s.steadyPrevSpeed) / aDt;
    const float k = std::min(aDt / kSteadyFilterS, 1.0f);
    s.steadyAccel = s.steadyAccel >= 1.0e8f ? a : s.steadyAccel + (a - s.steadyAccel) * k;
    s.steadyPrevSpeed = aSpeedCmS;
    return std::fabs(s.steadyAccel) < kSteadyAccelCmS2;
}

inline void Reset(State& s, motiontraj::V3 aGame)
{
    s.valid = true;
    s.vel = aGame;
    s.gamePrevSpeed = motiontraj::SizeXY(aGame);
    s.stallSpeed = 0.0f;
    s.turning = false;
}

// Below this angle between the converted and the game velocity the two count as aligned: without aShapeSpeed the converted speed is
// then the game speed (a straight start, speed-up or slow-down is not reshaped; only a turn is).
inline constexpr float kAlignedCos = 0.9659258f; // 15 deg

// One feed. aGame: the game velocity on the query scale (cm/s, z kept as is); aInput: the move input is held; aActive: false = the
// converted velocity is the game velocity (air, slide, a scope that excludes this feed); aShapeSpeed: the speed also follows the
// acceleration / braking limits on a straight line (else only through a turn); aDirectionOnly (without aShapeSpeed): only the heading
// is converted, the speed is always the game speed. Returns the converted velocity.
inline motiontraj::V3 Step(State& s, motiontraj::V3 aGame, bool aInput, bool aActive, float aDt, const Limits& l, bool aShapeSpeed = true,
                           bool aDirectionOnly = false)
{
    using motiontraj::V3;
#if UTPP_SPEED_CONVERTER_MUTATION == 1
    aActive = false; // mutation 1: no conversion (the game velocity passes through)
#endif
    if (!aActive || !s.valid || !(aDt > 0.0f) || !std::isfinite(aDt))
    {
        const float keepStall = aInput ? s.stallSpeed : 0.0f; // a stall outlasts the slow feeds that switch the conversion off
        Reset(s, aGame);
        s.stallSpeed = keepStall;
        return aGame;
    }
    const V3 game{aGame.x, aGame.y, 0.0f};
    V3 v{s.vel.x, s.vel.y, 0.0f};
    const float target = motiontraj::SizeXY(game);
    const float prev = motiontraj::SizeXY(v);
    // a stall (the capsule lost most of its speed within a feed with the input held: a wall) keeps the game velocity; it holds until the input is released or the game speed is back above stallRatio of the speed before it (a capsule pressed
    // against a wall jitters in direction from feed to feed)
    if (aInput && target < l.stallRatio * s.gamePrevSpeed && s.stallSpeed <= 0.0f)
        s.stallSpeed = s.gamePrevSpeed;
    const float stallSpeed = s.stallSpeed;
    const bool stall = stallSpeed > 0.0f && aInput && target < l.stallRatio * stallSpeed;
    if (stall)
    {
        Reset(s, aGame);
        s.stallSpeed = stallSpeed;
        return aGame;
    }
    s.stallSpeed = 0.0f;
    s.gamePrevSpeed = target;
    // a reversal beyond maxTurnCos keeps the game velocity
    if (l.maxTurnCos > -1.0f && target > motiontraj::kHasVelocity && prev > motiontraj::kHasVelocity &&
        motiontraj::Dot(v, game) < l.maxTurnCos * prev * target)
    {
        Reset(s, aGame);
        return aGame;
    }
    if (aDirectionOnly && !aShapeSpeed && target > motiontraj::kHasVelocity && aInput && prev > motiontraj::kHasVelocity)
    {
        // direction only: the heading error to the game velocity decays at the friction rate (the small-angle behaviour of the CMC
        // friction term), the speed is the game speed; a reversal turns along an arc to the side of the error instead of stalling
        const float err = motiontraj::NormalizeAxis(motiontraj::YawOf(game) - motiontraj::YawOf(v));
        if (std::fabs(err) > kTurnStartDeg)
            s.turning = true;
        if (!s.turning || std::fabs(err) < kTurnEndDeg)
        {
            s.turning = false;
            s.vel = aGame;
            return aGame;
        }
        const float yaw = motiontraj::YawOf(v) + err * std::min(aDt * l.friction, 1.0f);
        const float r = yaw / motiontraj::kRad2Deg;
        v = V3{std::cos(r) * target, std::sin(r) * target, 0.0f};
    }
    else if (target > motiontraj::kHasVelocity && aInput)
    {
        // CMC CalcVelocity with input: friction pulls the velocity towards the wanted direction, the acceleration adds along it
        const V3 dir = motiontraj::SafeNormal(game);
        v = v - (v - dir * prev) * std::min(aDt * l.friction, 1.0f);
        v = v + dir * (l.acceleration * aDt);
        // above the wanted speed: braking (not below the wanted speed)
        const float cap = std::max(target, prev - l.brakingInput * aDt);
        const float n = motiontraj::SizeXY(v);
        if (n > cap && n > 0.0f)
            v = v * (cap / n);
    }
    else
    {
        // no input (or the capsule stopped): braking along the current velocity, never below the game speed
        if (prev > 0.0f)
        {
            const float braking = aInput ? l.brakingInput : l.brakingNoInput;
            float n = std::max(prev - braking * aDt, target);
            if (n <= motiontraj::kBrakeToStop && target <= motiontraj::kBrakeToStop)
                n = target;
            // the direction follows the game velocity once it moves (a stop keeps its last heading)
            const V3 dir = target > motiontraj::kHasVelocity ? motiontraj::SafeNormal(game) : motiontraj::SafeNormal(v);
            v = dir * n;
        }
    }
    if (!aShapeSpeed && target > motiontraj::kHasVelocity)
    {
        // aligned with the game velocity: the game speed (the turn shaping above is kept until the directions agree)
        const float n = motiontraj::SizeXY(v);
        if (n > 0.0f && motiontraj::Dot(v, game) >= kAlignedCos * n * target)
            v = v * (target / n);
    }
    const V3 diff = v - game;
    if (motiontraj::SizeXY(diff) > l.maxDivergence)
        v = game;
    s.vel = V3{v.x, v.y, aGame.z};
    return s.vel;
}
} // namespace utpp::mm::speedconv

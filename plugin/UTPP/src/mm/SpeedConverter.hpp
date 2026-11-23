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
};

struct State
{
    bool valid = false;
    motiontraj::V3 vel{};
};

inline void Reset(State& s, motiontraj::V3 aGame)
{
    s.valid = true;
    s.vel = aGame;
}

// Below this angle between the converted and the game velocity the two count as aligned: without aShapeSpeed the converted speed is
// then the game speed (a straight start, speed-up or slow-down is not reshaped; only a turn is).
inline constexpr float kAlignedCos = 0.9659258f; // 15 deg

// One feed. aGame: the game velocity on the query scale (cm/s, z kept as is); aInput: the move input is held; aActive: false = the
// converted velocity is the game velocity (air, slide, a scope that excludes this feed); aShapeSpeed: the speed also follows the
// acceleration / braking limits on a straight line (else only through a turn). Returns the converted velocity.
inline motiontraj::V3 Step(State& s, motiontraj::V3 aGame, bool aInput, bool aActive, float aDt, const Limits& l, bool aShapeSpeed = true)
{
    using motiontraj::V3;
#if UTPP_SPEED_CONVERTER_MUTATION == 1
    aActive = false; // mutation 1: no conversion (the game velocity passes through)
#endif
    if (!aActive || !s.valid || !(aDt > 0.0f) || !std::isfinite(aDt))
    {
        Reset(s, aGame);
        return aGame;
    }
    const V3 game{aGame.x, aGame.y, 0.0f};
    V3 v{s.vel.x, s.vel.y, 0.0f};
    const float target = motiontraj::SizeXY(game);
    const float prev = motiontraj::SizeXY(v);
    if (l.maxTurnCos > -1.0f && target > motiontraj::kHasVelocity && prev > motiontraj::kHasVelocity &&
        motiontraj::Dot(v, game) < l.maxTurnCos * prev * target)
    {
        Reset(s, aGame);
        return aGame;
    }
    if (target > motiontraj::kHasVelocity && aInput)
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

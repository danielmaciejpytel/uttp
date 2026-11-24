#pragma once
// Speed converter, starts and stops (MapConfig::speedConverter bit kStartStopWindow). The game capsule reaches its speed within
// ~0.18 s of a start and brakes a run within ~0.18 s of a release; the reference character movement accelerates at its maximum
// acceleration (800 cm/s^2) and brakes at its braking deceleration (2000 cm/s^2 without input, braking friction 0), and its start and
// stop clips were recorded with that speed curve. This stage converts only the speed of the first / last fraction of a second of a
// start (from rest, input held; StartStopLimits::starts, off by default) and a stop (input released): the converted speed follows the
// reference curve until the window ends or the converted distance falls behind / runs ahead of the capsule distance by gapCapCm,
// then hands back to the speed of the turn converter (speedconv::Step) over handoffS. Only the query velocity changes; the capsule position stays the game's. Inside the
// window the capsule start / stop forecast of the prediction is dropped (the prediction continues the converted speed with the
// reference movement model, as the reference does) and the clip plays at its own rate (no game speed rate).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "MotionTrajectory.hpp"

#ifndef UTPP_SPEED_CONVERTER_MUTATION
#define UTPP_SPEED_CONVERTER_MUTATION 0
#endif

namespace utpp::mm::speedconv
{
// MapConfig::speedConverter bit (with kQuery): convert the speed of starts and stops inside a short window
inline constexpr int kStartStopWindow = 64;

struct StartStopLimits
{
    // Which windows open. Starts are off: a converted start speed made the start picks slide more (start world slide p95 +9.6 % on
    // the recorded sessions, with or without the game speed rate, the capsule forecast or a larger gap cap).
    bool starts = false;
    bool stops = true;
    float startWindowS = 0.5f;  // longest converted start (s)
    float stopWindowS = 0.4f;   // longest converted stop (s)
    float gapCapCm = 15.0f;     // largest distance between the converted and the capsule path along the move (cm)
    float handoffS = 0.1f;      // time constant of the hand-off back to the turn converter's speed (s)
    float handoffDoneCmS = 2.0f; // the hand-off ends within this speed of its target (cm/s)
};

enum class StartStopPhase : uint8_t
{
    None,
    Start,
    Stop,
    Handoff,
};

struct StartStopState
{
    bool valid = false;
    StartStopPhase phase = StartStopPhase::None;
    float t = 0.0f;       // time in the current start / stop window (s)
    float gapCm = 0.0f;   // capsule distance - converted distance in the window (cm)
    float speed = 0.0f;   // converted speed of the last feed (cm/s, horizontal)
    motiontraj::V3 dir{}; // heading of the last moving feed (unit, horizontal)
    bool input = false;   // the input was held on the last feed
    bool converting = false; // the last feed's speed differs from the turn converter's (inside a window or its hand-off)
};

// The last feed was converted (the hook drops the capsule start / stop forecast and the game speed rate on such a feed)
inline bool StartStopConverting(const StartStopState& s)
{
    return s.valid && s.converting;
}

inline void StartStopReset(StartStopState& s, motiontraj::V3 aVel, bool aInput)
{
    s.valid = true;
    s.phase = StartStopPhase::None;
    s.t = 0.0f;
    s.gapCm = 0.0f;
    s.speed = motiontraj::SizeXY(aVel);
    if (s.speed > motiontraj::kHasVelocity)
        s.dir = motiontraj::SafeNormal(motiontraj::V3{aVel.x, aVel.y, 0.0f});
    s.input = aInput;
    s.converting = false;
}

// One feed. aGame: the game velocity on the query scale (cm/s); aConverted: this feed's result of the turn converter (speedconv::Step,
// the game velocity when that does not convert); aInput: the move input is held; aGround: the converter's ground condition (on the
// ground, outside the landing window, no slide or vehicle exit); aAcceleration / aBraking: the reference maximum acceleration and the
// braking deceleration without input (cm/s^2). Returns the query velocity (z of aConverted kept).
inline motiontraj::V3 StartStopStep(StartStopState& s, motiontraj::V3 aGame, motiontraj::V3 aConverted, bool aInput, bool aGround,
                                    float aDt, float aAcceleration, float aBraking, const StartStopLimits& l = {})
{
    using motiontraj::V3;
    if (!aGround || !s.valid || !(aDt > 0.0f) || !std::isfinite(aDt))
    {
        StartStopReset(s, aConverted, aInput);
        return aConverted;
    }
    const float game = motiontraj::SizeXY(aGame);
    const float conv = motiontraj::SizeXY(aConverted);
    const bool wasInput = s.input;
    s.input = aInput;
    // window starts: from rest with the input held, or a release while moving
    if (l.starts && aInput && s.speed <= motiontraj::kHasVelocity && game > motiontraj::kHasVelocity && s.phase != StartStopPhase::Start)
    {
        s.phase = StartStopPhase::Start;
        s.t = 0.0f;
        s.gapCm = 0.0f;
    }
    else if (l.stops && !aInput && wasInput && s.speed > motiontraj::kHasVelocity && s.phase != StartStopPhase::Stop)
    {
        s.phase = StartStopPhase::Stop;
        s.t = 0.0f;
        s.gapCm = 0.0f;
    }
    else if ((s.phase == StartStopPhase::Start && !aInput) || (s.phase == StartStopPhase::Stop && aInput))
        s.phase = StartStopPhase::Handoff; // input changed inside the window: back to the turn converter
#if UTPP_SPEED_CONVERTER_MUTATION == 10
    if (s.phase == StartStopPhase::Start || s.phase == StartStopPhase::Stop)
        s.phase = StartStopPhase::None; // mutation 10: no window (the game speed passes)
#endif
    float speed = conv;
    s.converting = false;
    if (s.phase == StartStopPhase::Start)
    {
        // reference start: at most the maximum acceleration, never above the turn converter's speed. The window stays open for its
        // whole time (the game speed can still rise after the ramp caught up with it); it converts only while the ramp is below.
        speed = std::min(conv, s.speed + aAcceleration * aDt);
        s.t += aDt;
        s.gapCm += (game - speed) * aDt;
        s.converting = speed < conv - l.handoffDoneCmS;
        if (s.t >= l.startWindowS || std::fabs(s.gapCm) >= l.gapCapCm)
            s.phase = StartStopPhase::Handoff;
    }
    else if (s.phase == StartStopPhase::Stop)
    {
        // reference stop without input: braking deceleration, never below the game speed
        speed = std::max(conv, s.speed - aBraking * aDt);
        if (speed <= motiontraj::kBrakeToStop && conv <= motiontraj::kBrakeToStop)
        {
            speed = conv;
            s.phase = StartStopPhase::None; // both stopped
        }
        else
        {
            s.t += aDt;
            s.gapCm += (game - speed) * aDt;
            s.converting = speed > conv + l.handoffDoneCmS;
            if (s.t >= l.stopWindowS || std::fabs(s.gapCm) >= l.gapCapCm)
                s.phase = StartStopPhase::Handoff;
        }
    }
    else if (s.phase == StartStopPhase::Handoff)
    {
#if UTPP_SPEED_CONVERTER_MUTATION == 11
        speed = conv; // mutation 11: the hand-off jumps to the turn converter's speed
#else
        speed = s.speed + (conv - s.speed) * std::min(aDt / std::max(l.handoffS, 1.0e-3f), 1.0f);
#endif
        if (std::fabs(speed - conv) <= l.handoffDoneCmS)
        {
            speed = conv;
            s.phase = StartStopPhase::None;
        }
        s.converting = speed != conv;
    }
    // heading: the turn converter's, else the last moving heading (a stop keeps it), else the game's
    V3 dir = conv > motiontraj::kHasVelocity ? motiontraj::SafeNormal(V3{aConverted.x, aConverted.y, 0.0f})
             : game > motiontraj::kHasVelocity ? motiontraj::SafeNormal(V3{aGame.x, aGame.y, 0.0f})
                                               : s.dir;
    V3 out = speed == conv ? aConverted : V3{dir.x * speed, dir.y * speed, aConverted.z};
    s.speed = std::max(speed, 0.0f);
    if (s.speed > motiontraj::kHasVelocity && motiontraj::SizeXY(dir) > 0.5f)
        s.dir = dir;
    return out;
}
} // namespace utpp::mm::speedconv

#pragma once
// Speed converter, gait changes (MapConfig::speedConverter bit kGaitRamp): the reference character movement changes its maximum speed
// with the gait (walk / run / sprint) and reaches the new speed along its acceleration (speed-up, the sprint acceleration falls with the
// speed) or its braking with the input held (slow-down), so a gait change is a speed ramp. The game capsule changes its speed in a few
// feeds. On a gait change from a held speed the converted query speed follows that ramp from the speed before the change towards the
// game speed, along the converted heading. The ramp is used only once the input has stayed held for kGaitConfirmS after the change (a
// stick released to stop passes the walk gait for a few feeds; that stop is left to the stop conversion); until then it runs in the
// background from the change. It hands off to the game speed as soon as it reaches it, after kGaitRampMaxS, or when the conversion
// stops (air, no input, a stop, a stall reset), so held speeds and every other feed are unchanged.
#include <algorithm>
#include <cmath>
#include "MotionTrajectory.hpp"

#ifndef UTPP_SPEED_CONVERTER_MUTATION
#define UTPP_SPEED_CONVERTER_MUTATION 0
#endif

namespace utpp::mm::speedconv
{
// MapConfig::speedConverter bit: ramp the converted speed across a gait change (needs kQuery)
inline constexpr int kGaitRamp = 128;
// longest ramp (s): a ramp still away from the game speed after this hands off (a game speed that keeps moving away)
inline constexpr float kGaitRampMaxS = 1.0f;
// the ramp counts as reached within this distance of the game speed (cm/s on the query scale)
inline constexpr float kGaitRampDoneCmS = 2.0f;
// a ramp starts only from a held speed: the game speed stayed within kGaitHeldShare of itself for kGaitHeldS before the gait change
// (a gait change inside a start, a stop or a speed change is left to those conversions)
inline constexpr float kGaitHeldS = 0.2f;
inline constexpr float kGaitHeldShare = 0.08f;
// the ramp is used once the input has stayed held this long after the gait change (s)
#if UTPP_SPEED_CONVERTER_MUTATION == 24
inline constexpr float kGaitConfirmS = 0.0f; // mutation 24: the ramp is used at once (also on the walk gait of a stop)
#else
inline constexpr float kGaitConfirmS = 0.1f;
#endif

struct GaitState
{
    int prevGait = -1;     // gait of the previous feed (-1 = none)
    bool ramping = false;  // a gait ramp is running
    bool confirmed = false; // the ramp is used (the input stayed held kGaitConfirmS after the change)
    float speed = 0.0f;    // the ramp speed of the previous feed (cm/s)
    float ageS = 0.0f;     // time since the ramp started (s)
    float prevOut = 0.0f;  // the converted speed of the previous feed (the start of a ramp)
    float heldRef = 0.0f;  // the game speed the hold is measured against
    float heldS = 0.0f;    // time the game speed has stayed within kGaitHeldShare of heldRef (s)
};

// Rate of the ramp towards aTarget from aSpeed: the gait's maximum acceleration upwards, the braking with the input held downwards
inline float GaitRampRate(float aSpeed, float aTarget, float aAccel, float aBraking)
{
#if UTPP_SPEED_CONVERTER_MUTATION == 21
    (void)aBraking;
    return aAccel; // mutation 21: a slow-down ramps at the acceleration
#else
    return aTarget >= aSpeed ? aAccel : aBraking;
#endif
}

// One feed. aConv: the converted velocity of this feed (speedconv::Step); aGame: the game velocity (both on the query scale, cm/s);
// aGait: gait of this feed; aActive: the conversion is active this feed with the input held (else the ramp stops); aAccel: the maximum
// acceleration of the gait at the current ramp speed (motiontraj::MotionMaxAcceleration); aBraking: braking with the input held.
// Returns aConv with its planar speed replaced by the ramp speed while a ramp runs.
inline motiontraj::V3 GaitStep(GaitState& s, motiontraj::V3 aConv, motiontraj::V3 aGame, int aGait, bool aActive, float aDt, float aAccel,
                               float aBraking)
{
    const float conv = motiontraj::SizeXY(aConv);
    const float game = motiontraj::SizeXY(aGame);
    const int prevGait = s.prevGait;
    s.prevGait = aGait;
    const float heldBefore = s.heldS; // the hold up to the previous feed
    if (aDt > 0.0f && std::isfinite(game) && s.heldRef > motiontraj::kHasVelocity && std::fabs(game - s.heldRef) <= kGaitHeldShare * s.heldRef)
        s.heldS += aDt;
    else
    {
        s.heldRef = game;
        s.heldS = 0.0f;
    }
#if UTPP_SPEED_CONVERTER_MUTATION == 20
    aActive = false; // mutation 20: no gait ramp
#endif
    if (!aActive || !(aDt > 0.0f) || !std::isfinite(aDt) || !std::isfinite(conv) || conv <= motiontraj::kHasVelocity ||
        game <= motiontraj::kHasVelocity)
    {
        s.ramping = false;
        s.confirmed = false;
        s.prevOut = aActive ? conv : 0.0f;
        return aConv;
    }
#if UTPP_SPEED_CONVERTER_MUTATION == 23
    const bool fromHeld = true; // mutation 23: a ramp also starts inside a speed change (a start)
#else
    const bool fromHeld = heldBefore >= kGaitHeldS;
#endif
    if (prevGait >= 0 && aGait != prevGait && s.prevOut > motiontraj::kHasVelocity && (s.ramping || fromHeld))
    {
        // a new gait: start at the speed of the previous feed (a running ramp continues from its speed and keeps its confirmation)
        if (!s.ramping)
        {
            s.speed = s.prevOut;
            s.confirmed = false;
            s.ageS = 0.0f;
        }
        s.ramping = true;
    }
    if (!s.ramping)
    {
        s.prevOut = conv;
        return aConv;
    }
    s.ageS += aDt;
    const float rate = GaitRampRate(s.speed, conv, aAccel, aBraking);
    const float step = rate * aDt;
    s.speed = conv > s.speed ? std::min(s.speed + step, conv) : std::max(s.speed - step, conv);
#if UTPP_SPEED_CONVERTER_MUTATION == 22
    const bool done = std::fabs(s.speed - conv) <= kGaitRampDoneCmS; // mutation 22: no time hand-off
#else
    const bool done = std::fabs(s.speed - conv) <= kGaitRampDoneCmS || s.ageS >= kGaitRampMaxS;
#endif
    if (done)
    {
        s.ramping = false;
        s.confirmed = false;
        s.prevOut = conv;
        return aConv;
    }
    s.confirmed = s.confirmed || s.ageS >= kGaitConfirmS;
    if (!s.confirmed)
        return aConv; // the ramp runs in the background until the change is confirmed
    s.prevOut = s.speed;
    const float k = s.speed / conv;
    return motiontraj::V3{aConv.x * k, aConv.y * k, aConv.z};
}
} // namespace utpp::mm::speedconv

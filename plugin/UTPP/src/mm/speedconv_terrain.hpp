#pragma once
// Speed converter, terrain part (MapConfig::speedConverter bit kStepHold): the reference character movement keeps its horizontal
// velocity on slopes and steps (walking keeps the horizontal ground velocity, a step up to the maximum step height is climbed within
// the move). On slopes the game capsule does the same (horizontal speed within about 1 % of the flat speed), but on a stair step the
// game capsule often loses most of its horizontal speed for a few feeds and then accelerates again. Without this part the query sees
// that dip as a stop or a wall (speedconv::Limits::stallRatio) and the matcher changes the clip in the middle of a stair flight.
// kStepHold: a horizontal speed dip that coincides with a step of the capsule height (within kStepWindowS on either side) keeps the
// speed before the dip for at most kHoldS, then follows the game speed down at kReleaseDecelCmS2. Only the query velocity (the input
// of speedconv::Step) changes; the capsule, the play rate base and the feet keep the game velocity, so kGameSpeedRate slows the clip
// down with the capsule. Off (bit clear) = the game velocity passes unchanged, bit for bit.
#include <algorithm>
#include <cmath>
#include "MotionTrajectory.hpp"

#ifndef UTPP_SPEED_CONVERTER_MUTATION
#define UTPP_SPEED_CONVERTER_MUTATION 0
#endif

namespace utpp::mm::speedconv
{
inline constexpr int kStepHold = 1024; // MapConfig::speedConverter bit: hold the query speed through a stair step dip

inline constexpr float kStepMinM = 0.06f;            // capsule height change within one feed that counts as a step (m)
inline constexpr float kStepWindowS = 0.2f;          // a dip within this time of a step (either order) is a step dip
inline constexpr float kDipRatio = 0.8f;             // below this share of the speed before it the horizontal speed is a dip
inline constexpr float kHoldS = 0.3f;                // longest hold of the speed before the dip
inline constexpr float kReleaseDecelCmS2 = 500.0f;   // after the hold: the held speed follows the game speed down at this rate
inline constexpr float kSpeedFilterS = 0.1f;         // time constant of the speed before a dip (cm/s filter of the undipped speed)
inline constexpr float kMinHoldSpeedCmS = 100.0f;    // only a dip from at least this speed is held (a start from rest is no dip)
// only a straight climb is held: the game direction stays within 20 deg of the direction before the dip (a pivot, a reversal or a
// turn whose speed dip meets a step keeps the game velocity for the rest of that dip)
inline constexpr float kStraightCos = 0.9396926f;

struct StepHoldState
{
    bool havePrevZ = false;
    float prevZ = 0.0f;
    float sinceStep = 1.0e9f; // s since the last step of the capsule height
    float refSpeed = 0.0f;    // filtered horizontal game speed outside dips (cm/s)
    bool dipping = false;     // the game speed is below kDipRatio * refSpeed
    float sinceDip = 0.0f;    // s since the dip started
    bool stepDip = false;     // the current dip is a step dip (held)
    bool turned = false;      // the current dip left the direction before it (not held)
    motiontraj::V3 dipDir{};  // the direction before the dip
    float heldSpeed = 0.0f;   // the held query speed (cm/s); 0 = not holding
    motiontraj::V3 heldDir{}; // its direction (the game direction while it moves)
};

inline void ResetStepHold(StepHoldState& s)
{
    const bool haveZ = s.havePrevZ;
    const float z = s.prevZ;
    s = StepHoldState{};
    s.havePrevZ = haveZ;
    s.prevZ = z;
}

// One feed. aGame: the game velocity on the query scale (cm/s); aCapsuleZ: the capsule height (m), aHaveZ false = unknown (no hold);
// aEligible: on the ground with the move input held and the converter active. Returns the velocity for speedconv::Step (aGame when
// nothing is held).
inline motiontraj::V3 StepHoldVelocity(StepHoldState& s, motiontraj::V3 aGame, float aCapsuleZ, bool aHaveZ, bool aEligible, float aDt)
{
    using motiontraj::V3;
    const bool zOk = aHaveZ && std::isfinite(aCapsuleZ);
    const bool step = zOk && s.havePrevZ && std::fabs(aCapsuleZ - s.prevZ) >= kStepMinM;
    s.havePrevZ = zOk;
    s.prevZ = zOk ? aCapsuleZ : 0.0f;
    const float game = motiontraj::SizeXY(aGame);
    if (!aEligible || !(aDt > 0.0f) || !std::isfinite(aDt) || !std::isfinite(game))
    {
        ResetStepHold(s);
        return aGame;
    }
    s.sinceStep = step ? 0.0f : s.sinceStep + aDt;
    const bool dip = s.refSpeed > motiontraj::kHasVelocity && game < kDipRatio * s.refSpeed;
    if (dip && !s.dipping)
    {
        s.dipping = true;
        s.sinceDip = 0.0f;
        s.stepDip = false;
        s.turned = s.refSpeed < kMinHoldSpeedCmS;
        s.heldSpeed = s.refSpeed;
        s.heldDir = s.heldDir.x == 0.0f && s.heldDir.y == 0.0f ? motiontraj::SafeNormal(V3{aGame.x, aGame.y, 0.0f}) : s.heldDir;
        s.dipDir = s.heldDir;
    }
    else if (dip)
        s.sinceDip += aDt;
    if (!dip)
    {
        // outside a dip: the speed before the next dip follows the game speed; a hold still running decays to it
        s.dipping = false;
        s.stepDip = false;
        s.turned = false;
        const float k = std::min(aDt / kSpeedFilterS, 1.0f);
        s.refSpeed = s.refSpeed > motiontraj::kHasVelocity ? s.refSpeed + (game - s.refSpeed) * k : game;
        s.heldSpeed = 0.0f;
        if (game > motiontraj::kHasVelocity)
            s.heldDir = motiontraj::SafeNormal(V3{aGame.x, aGame.y, 0.0f});
        return aGame;
    }
    // a step dip: the step came at most kStepWindowS before the dip or within kStepWindowS after its start
#if UTPP_SPEED_CONVERTER_MUTATION == 50
    s.stepDip = true; // mutation 50: every dip is held (a stop or a wall too)
#else
    s.stepDip = s.stepDip || (s.sinceStep <= kStepWindowS + s.sinceDip && s.sinceDip <= kStepWindowS);
#endif
    if (game > motiontraj::kHasVelocity && motiontraj::Dot(motiontraj::SafeNormal(V3{aGame.x, aGame.y, 0.0f}), s.dipDir) < kStraightCos)
        s.turned = true;
    if (!s.stepDip || s.turned)
        return aGame; // a dip without a step (a stop, a wall) or with a turn: the game velocity
    if (game > motiontraj::kHasVelocity)
        s.heldDir = motiontraj::SafeNormal(V3{aGame.x, aGame.y, 0.0f});
#if UTPP_SPEED_CONVERTER_MUTATION == 51
    const bool hold = true; // mutation 51: the hold never ends
#else
    const bool hold = s.sinceDip <= kHoldS;
#endif
    if (!hold)
        s.heldSpeed = std::max(game, s.heldSpeed - kReleaseDecelCmS2 * aDt);
    if (s.heldSpeed <= game)
    {
        s.heldSpeed = 0.0f;
        return aGame;
    }
    return V3{s.heldDir.x * s.heldSpeed, s.heldDir.y * s.heldSpeed, aGame.z};
}
} // namespace utpp::mm::speedconv

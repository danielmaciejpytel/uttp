#pragma once
// Speed converter, jump and landing stage (MapConfig::speedConverter bit kLanding). Without it the converter is off in the air and
// in the landing window, so it restarts from the game velocity once the window ends. With it the converter keeps running through
// the landing:
// - on touch-down it is seeded with the game ground speed (the capsule snaps its speed on touch-down; the play rate and the feet
//   follow the capsule, so the landing clip is picked at the speed the capsule really moves) and with the heading of the flight when
//   the game heading differs (an air-control turn becomes the curved turn of the reference movement instead of a heading snap);
// - the first feeds after the touch-down often report a velocity pointing far off the flight while the input still points along it
//   (a one- or two-feed spike of the ground snap, up to a reversal). Such feeds are converted from the flight heading at the game
//   speed, so the landing pick sees no pivot or start that the player never made;
// - it stays on until the regular ground scope takes over, so the run after the landing resumes without a restart.
// A ground contact after a very short air phase (the contact flag flickering on a slope or a step) is no landing.
#include <cmath>
#include "SpeedConverter.hpp"

#ifndef UTPP_SPEED_CONVERTER_MUTATION
#define UTPP_SPEED_CONVERTER_MUTATION 0
#endif

namespace utpp::mm::speedconv
{
// MapConfig::speedConverter bit: the converter also runs through the landing (see above)
inline constexpr int kLanding = 2048;
// shorter air phases are contact flicker, not a landing (one 60 Hz feed in the air is flicker; a hop off a step or the game's
// short air phase of a running jump lasts two feeds or more)
inline constexpr float kLandingMinAirS = 0.025f;
inline constexpr float kLandingMaxWindowS = 1.0f; // the landing scope ends after this at the latest (s since touch-down)
// touch-down spike: the input within this angle of the flight heading and the game ground velocity beyond it, within kLandingSpikeS
// of the touch-down
inline constexpr float kLandingSpikeDeg = 45.0f;
inline constexpr float kLandingSpikeS = 0.1f;

struct LandingState
{
    float airS = 0.0f;       // time in the air of the current flight (s)
    motiontraj::V3 airVel{}; // game velocity of the last air feed
    bool window = false;     // the landing scope is open (touch-down until the regular ground scope takes over)
    float sinceLandS = 0.0f; // s since the touch-down of the open window
    int spikes = 0;          // touch-down spikes replaced (diagnostics)
};

// True when the touch-down velocity aGame is a spike: the flight and the input agree within kLandingSpikeDeg, the game ground
// velocity points beyond it (both moving, the input held).
inline bool LandingSpike(motiontraj::V3 aAirVel, motiontraj::V3 aGame, motiontraj::V3 aInput)
{
#if UTPP_SPEED_CONVERTER_MUTATION == 62
    return false; // mutation 62: the touch-down velocity is always trusted
#else
    if (motiontraj::SizeXY(aAirVel) <= motiontraj::kHasVelocity || motiontraj::SizeXY(aGame) <= motiontraj::kHasVelocity ||
        motiontraj::IsZero(aInput))
        return false;
    const float air = motiontraj::YawOf(aAirVel);
    return std::fabs(motiontraj::NormalizeAxis(motiontraj::YawOf(aInput) - air)) <= kLandingSpikeDeg &&
           std::fabs(motiontraj::NormalizeAxis(motiontraj::YawOf(aGame) - air)) > kLandingSpikeDeg;
#endif
}

// The game velocity along the flight heading at the game ground speed (vertical kept).
inline motiontraj::V3 AlongFlight(motiontraj::V3 aAirVel, motiontraj::V3 aGame)
{
    const motiontraj::V3 dir = motiontraj::SafeNormal(motiontraj::V3{aAirVel.x, aAirVel.y, 0.0f});
    const float ground = motiontraj::SizeXY(aGame);
    return motiontraj::V3{dir.x * ground, dir.y * ground, aGame.z};
}

// Seeds the converter on a touch-down: the game ground speed, the flight heading when both move and the turn between them stays
// within aMaxTurnCos (a larger one keeps the game velocity, as Step does), else the game velocity.
inline void SeedLanding(State& s, motiontraj::V3 aAirVel, motiontraj::V3 aGame, float aMaxTurnCos)
{
    Reset(s, aGame); // also: the previous game speed is the ground speed, so the touch-down snap is no stall
#if UTPP_SPEED_CONVERTER_MUTATION == 61
    return; // mutation 61: the flight heading is not kept (the game heading on touch-down)
#else
    const float ground = motiontraj::SizeXY(aGame);
    const float air = motiontraj::SizeXY(aAirVel);
    if (ground <= motiontraj::kHasVelocity || air <= motiontraj::kHasVelocity)
        return;
    const motiontraj::V3 airXY{aAirVel.x, aAirVel.y, 0.0f};
    const motiontraj::V3 gameXY{aGame.x, aGame.y, 0.0f};
    if (motiontraj::Dot(airXY, gameXY) < aMaxTurnCos * air * ground)
        return;
    s.vel = AlongFlight(aAirVel, aGame);
#endif
}

// One feed of the landing stage, before speedconv::Step. aGame: the game velocity, replaced by AlongFlight on a touch-down spike
// (the converter's input only); aInput: the input acceleration; aAir: the character is in the air; aGround: on the ground and
// eligible for the converter (no slide, no vehicle exit, no jump-out flight); aRegular: the regular ground scope of the converter
// covers this feed. Returns true while the landing scope converts this feed (the caller then runs Step as on the ground).
inline bool LandingFeed(LandingState& l, State& s, motiontraj::V3& aGame, motiontraj::V3 aInput, bool aAir, bool aGround, bool aRegular,
                        float aDt, float aMaxTurnCos)
{
    const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f;
    if (aAir)
    {
        l.airS += dt;
        l.airVel = aGame;
        l.window = false;
        return false;
    }
    if (!aGround)
    {
        const int spikes = l.spikes;
        l = LandingState{};
        l.spikes = spikes;
        return false;
    }
    const bool touchDown = l.airS >= kLandingMinAirS;
    if (!touchDown && l.window)
        l.sinceLandS += dt;
    if ((touchDown || (l.window && l.sinceLandS <= kLandingSpikeS)) && LandingSpike(l.airVel, aGame, aInput))
    {
        aGame = AlongFlight(l.airVel, aGame);
        ++l.spikes;
    }
    if (touchDown)
    {
        SeedLanding(s, l.airVel, aGame, aMaxTurnCos);
        l.window = true;
        l.sinceLandS = 0.0f;
    }
    l.airS = 0.0f;
#if UTPP_SPEED_CONVERTER_MUTATION == 60
    l.window = false; // mutation 60: no landing scope (the converter waits for the regular ground scope)
#endif
    if (l.window && (aRegular || l.sinceLandS > kLandingMaxWindowS))
        l.window = false;
    return l.window;
}
} // namespace utpp::mm::speedconv

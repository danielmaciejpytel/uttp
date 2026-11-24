#pragma once
// Speed converter, strafe travel (MapConfig::speedConverter bit kStanceTravel): the converter limits of one feed while the character
// keeps its facing and the velocity turns relative to it (aiming, shooter mode, aim jog; standing or crouched).
//
// The converter defaults were chosen on orient-to-movement turns: the body turns with the velocity, a reversal beyond the max turn
// angle is left to the game velocity (the pivot clips cover it) and the turn friction is below the ground friction (the heading reaches
// the input gradually while the body visibly turns). With a held facing none of that applies: a left / right or forward / back flip is a
// velocity reversal without a body turn, which the reference character movement passes through a slow-down along the old direction
// (ground friction on the off-direction part, then the acceleration along the new one), and a sideways turn of the velocity shows as a
// sideways slide of the feet for as long as the converted heading lags. With the bit set and the rotation mode strafing:
//  - reversals are converted too (no max turn reset), so the query trajectory shows the reference slow-down and the directional
//    pivot / stop picks see it;
//  - the turn friction is at least the ground friction of the reference movement for the gait (crouch and stand alike).
// The orient-to-movement feeds keep the limits unchanged, so the bit is inert outside strafing.
#include <algorithm>
#include <cmath>
#include "SpeedConverter.hpp"

namespace utpp::mm::speedconv
{
inline constexpr int kStanceTravel = 512; // MapConfig::speedConverter bit: strafe travel limits (StanceLimits)

// The limits of one converted feed. aBits = MapConfig::speedConverter, aStrafe = the rotation mode strafes (facing held),
// aGroundFriction = the reference ground friction of the feed's gait (per second). Unchanged without the bit or outside strafing.
inline Limits StanceLimits(Limits aLimits, int aBits, bool aStrafe, float aGroundFriction)
{
#if UTPP_SPEED_CONVERTER_MUTATION == 40
    return aLimits; // mutation 40: the strafe limits are never applied
#endif
    if (!(aBits & kStanceTravel) || !aStrafe)
        return aLimits;
#if UTPP_SPEED_CONVERTER_MUTATION != 41
    aLimits.maxTurnCos = -1.0f; // every reversal is converted (mutation 41: reversals keep the game velocity)
#endif
#if UTPP_SPEED_CONVERTER_MUTATION != 42
    if (std::isfinite(aGroundFriction) && aGroundFriction > aLimits.friction)
        aLimits.friction = aGroundFriction; // mutation 42: the turn friction stays the converter's
#endif
    return aLimits;
}
} // namespace utpp::mm::speedconv

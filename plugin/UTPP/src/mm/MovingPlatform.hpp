// Moving platform (elevator) feed conditioning. Pure logic, header-only.
//
// While a moving platform carries V (an elevator cabin), the game reports the platform's own motion in the capsule velocity and its
// ground contact (IsOnGround) can drop although V stands on the cabin floor. The game's own state machine ignores that contact loss
// there (no fall while IsOnMovingPlatform) and blocks jumps inside a moving elevator, so the platform motion is never a jump or a
// fall. The locomotion feed is therefore read in the platform frame: in contact with the floor, no vertical velocity, no floor probe.
// Planar velocity, the capsule position and every other field stay as sent.
//
// Applied only when the script reports the platform (FeedInput::onMovingPlatform) and the PSM detail is not an air detail (14..22:
// fall, jumps, other air): a real jump or fall keeps the raw fields. With the flag off nothing changes (bit-identical feed).
// Both feed entries apply it (motiondriver::FeedAirFromInput and motionhost::Feed); it is idempotent.
#pragma once

#include "../MotionHost.hpp"
#include "GameIntegration.hpp"

#ifndef UTPP_PLATFORM_MUTATION
#define UTPP_PLATFORM_MUTATION 0 // 1: no conditioning (the feed as the game sends it; tests/MotionDriverAirTest.cpp section L must fail)
#endif

namespace utpp::mm::platform
{
// True when aIn is conditioned on this feed (flag on, no air detail).
inline bool Applies(const motionhost::FeedInput& aIn)
{
#if UTPP_PLATFORM_MUTATION == 1
    (void)aIn;
    return false;
#else
    return aIn.onMovingPlatform && !integ::PsmIsAirDetail(integ::PsmDetailOf(aIn.psmBlock));
#endif
}

// Conditions aIn in place (see the header). Returns Applies(aIn).
inline bool Condition(motionhost::FeedInput& aIn)
{
    if (!Applies(aIn))
        return false;
    aIn.velWorld[2] = 0.0f;
    if (aIn.haveAir)
    {
        aIn.onGround = true;
        aIn.distToGround = -2.0f;
    }
    return true;
}
} // namespace utpp::mm::platform

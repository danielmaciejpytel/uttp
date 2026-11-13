// UTPP 0.6.5 game look write (input/GameLook.cpp): the game's FPP look (aim animation, shots, crosshair, Locomotion heading,
// AimOffset reference) is driven to a given world yaw / pitch. See GameLook.cpp for the 2.31 evidence.
#pragma once

#include <string>

#include <RED4ext/RED4ext.hpp>

namespace utpp::gamelook
{
// Load time, only on a validated 2.31 game (code bytes checked again here; fail closed: no hook, natives return false).
void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
void Shutdown();

// Game thread (script native). Wanted world yaw (deg, CCW positive: atan2(-x, y)) and pitch (deg, up positive) of the game's
// look for the coming ticks (expires after 0.25 s without a new call). Validates the hook, the RTTI layout of
// gameFPPCameraComponent / animAnimFeature_FPPCamera, the player's FPP camera component and the entity transform read.
// Returns true when the write is live (the hook then replaces the game's look delta while the heading is locked).
// 0.6.5b aEntityYawAheadDeg: V's yaw turn requested for the coming tick (utpp_bodyturn adjustTransform; the order against
// the camera update is measured); used once by the next write (|x| > 180 / NaN ignored). 0 = nothing to add: round 2 never
// clears a pending turn with 0, non-zero turns add up until a write consumes them (stale after 0.25 s). After a divergence
// trip the call fails for 3 s, then re-arms the write (at most 3 trips per game session).
bool SetTarget(float aWorldYawDeg, float aPitchDeg, float aEntityYawAheadDeg = 0.0f);
// Game thread. Ends the write at once (the game's own look input applies again). Returns whether it was active.
bool Stop();

// Game thread, every locomotion feed (2026-10-28): PSM vehicle bit and the TPP signal of the feed. The first on-foot feed
// after a vehicle feed in third person starts the vehicle edge zeroing (GameLookCore.hpp EdgeZero: for 0.3 s the hook
// drives the game's YawInput to 0, so the on-foot controller does not turn V from the exit yaw onto the held look), with or
// without the heading lock and UTPPSetGameLook. One log line at the edge and one summary at +0.4 s (entity yaw drift).
void OnFeed(bool aVehicleBit, bool aTpp);
// Game thread (script native UTPPGameLookVehicleExit, scripts/utpp_gamelook_exit.reds: PSM exit state enter). Round 2: the
// zeroing starts at the exit start and runs through the exit until the edge window ends (max 5 s), so the game's
// RotateEntityToCamera at the unmount (~0.13 s before the vehicle bit clears) finds YawInput 0. aUtppTpp: UTPP third
// person; aRideTpp: the ride's vehicle camera 1 TPP / 0 FPP / -1 unknown (zeroing only for true / 1). True = armed.
bool VehicleExitStart(bool aUtppTpp, int aRideTpp);

std::string Status();
} // namespace utpp::gamelook

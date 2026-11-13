// UTPP Locomotion (Phase 5N step 4a): native 360-degree movement input for the player state machine.
// Hook: atomic swap of the member-native handler table slots (table = address-library entry 0x5A7D28A9 itself) of
//   gamestateMachineGameScriptInterface.GetActionValue(CName) -> Float   (MoveX / MoveY only)
//   gamestateMachineGameScriptInterface.GetInputHeading() -> Float
// resolved from RTTI (function -> register index -> CBaseFunction handler table) with fail-closed proof on the first
// Running frame (after the script runtime has bound the natives' signatures).
// Logic: input/LocomotionCore.hpp. Natives: UTPPLocomotion* (scripts/utpp_locomotion.reds).

#pragma once

#include <RED4ext/RED4ext.hpp>

namespace utpp::locomotion
{
// Main.cpp OnLoad (first thing): stores the SDK, checks the kill file.
void Init(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
// Main.cpp PostRegisterTypes (end): registers the UTPPLocomotion* natives, runs the cheap checks (kill file,
// validation) and logs a pre-script RTTI snapshot. aGameValidated = UTPP's own 2.31 validation passed (!g_inert).
void OnPostRegister(bool aGameValidated);
// Main.cpp Running state (enter + update; idempotent, runs once): RTTI signature/handler proof and attach. Deferred
// because native class functions get their return/param types only when the script runtime loads.
void OnRunning();
// Main.cpp OnUnload: detaches the hook.
void Shutdown();
// Read only (no behaviour): the LocomotionCore::Reasons bits (freemove::Reason) of this moment. The v2 driver uses them to tell why the native
// body turn is off (H2 safeguard, MotionDriver SetLocomotionReasonsProbe). Thread-safe (atomics in the core).
uint32_t CurrentReasons();
} // namespace utpp::locomotion

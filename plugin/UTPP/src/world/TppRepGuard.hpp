// UTPP vehicle exit guard of the player's TPP representation. See TppRepGuard.cpp for the evidence and the hook.
#pragma once

#include <string>

#include <RED4ext/RED4ext.hpp>

namespace utpp::tpprepguard
{
// Load time, only on a validated 2.50 game (code sites checked here again, fail-closed: no hook, guard inert).
void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
void Shutdown();
// PostRegisterTypes: UTPPSetVehicleExitRepGuard(on: Bool) -> Bool (SetWindow) and UTPPVehicleExitRepGuardStatus() -> String.
void RegisterNatives();

// Exit window: on = open (if not open; at most kMaxWindowMs from the opening, a repeat does not extend it), off = close.
// The caller opens it only while UTPP wants the TPP body during a vehicle exit. Returns whether the hook is live.
bool SetWindow(bool aOn);
bool Installed();

std::string Status();
} // namespace utpp::tpprepguard

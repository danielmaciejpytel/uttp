// UTPP Phase 5N step 5: LootRange (loot prompt from the third-person view) + HeadGuard (near-camera head threshold).
// Entry points for Main.cpp. Implementation: world/LootHead.cpp (natives, TPP gate, tick), world/LootRange.cpp,
// render/HeadGuard.cpp; pure logic in world/LootRangeCore.hpp and render/HeadGuardCore.hpp.
// Script side: scripts/utpp_loothead.reds (natives, heartbeat). CET: cet/utpp_loothead/init.lua (settings, kill).

#pragma once

#include <RED4ext/RED4ext.hpp>

namespace utpp::loothead
{
// Main.cpp OnLoad: stores the SDK, checks the kill files.
void Init(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
// Main.cpp PostRegisterTypes (end): registers the natives and resolves RTTI. aGameValidated = UTPP's 2.31 validation.
void OnPostRegister(bool aGameValidated);
// Main.cpp OnRunningUpdate (every frame, game thread): drives both features (also reverts when the heartbeat stops).
void Tick();
// Main.cpp OnUnload: restores everything that was changed.
void Shutdown();
} // namespace utpp::loothead

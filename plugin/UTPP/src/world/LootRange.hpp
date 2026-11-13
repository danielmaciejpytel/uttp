// UTPP LootRange (Phase 5N step 5a), game side. Internal to the loothead module (see world/LootHead.hpp).
#pragma once

#include <cstdint>
#include <string>

#include "LootRangeCore.hpp"

namespace utpp::lootrange
{
void Init(bool aKillFile);
void Resolve(bool aGameValidated); // PostRegister: RTTI proof (classes, sizes, property offsets, vtables)
void Tick(double aNow, bool aTpp); // game thread
void Shutdown();                   // immediate revert of every slot

void SetEnabled(bool aOn);
void Kill();
bool SetWindow(const loot::WindowParams& aW);
bool AddDescriptor(const std::string& aPath);
bool Active();
uint32_t Reasons(double aNow, bool aTpp);
std::string Status(double aNow, bool aTpp);
} // namespace utpp::lootrange

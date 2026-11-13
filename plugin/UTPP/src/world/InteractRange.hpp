// UTPP InteractRange, game side. Internal to the loothead module (see world/LootHead.hpp, world/InteractRangeCore.hpp).
#pragma once

#include <cstdint>
#include <string>

#include "InteractRangeCore.hpp"

namespace RED4ext
{
struct IScriptable;
}

namespace utpp::interactrange
{
void Init(bool aKillFile);
void Resolve(bool aGameValidated); // PostRegister: RTTI proof (classes, sizes, property offsets, vtables, evaluate code)
void Tick(double aNow, bool aTpp); // game thread
void Shutdown();                   // immediate revert of every slot

void SetEnabled(bool aOn);
void Kill();
bool SetScale(float aScale);
bool SetVehicleMode(int32_t aMode);
bool SetVehiclePriority(float aPriority);
bool Active();
uint32_t Reasons(bool aTpp);
std::string Status(double aNow, bool aTpp);
// Console probe: names the interaction descriptor of an InteractionComponent and whether UTPP manages it.
std::string DescribeComponent(RED4ext::IScriptable* aComponent);
} // namespace utpp::interactrange

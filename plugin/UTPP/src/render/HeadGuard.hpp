// UTPP HeadGuard (Phase 5N step 5b), game side. Internal to the loothead module (see world/LootHead.hpp).
#pragma once

#include <cstdint>
#include <string>

#include "HeadGuardCore.hpp"

namespace utpp::headguard
{
void Init(bool aKillFile);
void Resolve(bool aGameValidated); // PostRegister: built-in near-camera site (signature + validation, fail-closed)
void Tick(bool aTpp);              // game thread
void Shutdown();                   // restores the original instruction

void SetEnabled(bool aOn);
void Kill();
bool SetSite(const hg::Site& aSite, std::string& aWhy); // runtime candidate (only while not patched)
bool SetValue(float aValue);
bool Active();
uint32_t Reasons(bool aTpp);
std::string Status(bool aTpp);
} // namespace utpp::headguard

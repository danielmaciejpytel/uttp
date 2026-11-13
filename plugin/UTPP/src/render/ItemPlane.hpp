// UTPP item render plane (0.6.4). See ItemPlane.cpp for the 2.31 mechanism (evidence) and the fix.
#pragma once

#include <cstdint>
#include <string>

#include <RED4ext/RED4ext.hpp>

namespace utpp::itemplane
{
// Load time, only on a validated 2.50 game (prologue bytes checked here again, fail-closed: no hook, override inert).
void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
void Shutdown();

// TPP override: while on, an RPl_Weapon (2) that the animation-parameter broadcast ("renderPlane" bindings of the
// player's AttachmentSlots) writes to a visual component is replaced by RPl_Scene (0). Returns whether the hook is live.
// 2026-11-01: on a change it also re-applies the new policy to every registered component (Reapply). Game thread only
// (script native): it calls the plane setter.
bool SetOverride(bool aOn);
bool OverrideOn();
bool Installed();

// Game thread only (script natives). Re-applies the current policy to every live component the broadcast has written
// (force: requested RPl_Weapon -> RPl_Scene; vanilla: exactly the requested plane; entries last written outside a broadcast
// are left alone). Returns the number of components whose plane was written (0 when the hook is not installed).
int32_t Reapply();
// Game thread only. A header line with the counters, then one line per registered live component (up to aMax): class,
// name (IComponent +0x40), requested plane, last write (broadcast / outside), current plane (+0x138), ownerChain (round 2:
// player | item:<slot>:<tdbid> | child:<slot>/<slot>:<tdbid> | other; aPlayer is the fallback when the item walk fails).
// Every line is also logged (LogInfo).
std::string Dump(int32_t aMax, const void* aPlayer);
// Round 2 (read-only, game thread only): walks the player entity, the items in its attachment slots and their child items.
// Header line, then per entity "PE owner=<chain> class= app= components=" and, for every entIVisualComponent /
// entEffectSpawnerComponent / entParticlesComponent (up to aMax rows), "PV owner=<chain> class=<rtti> name=<cname>
// param=<renderingPlaneAnimationParam: Skinned +0x238 / MorphTarget +0x2E8, else -> cur=<+0x138> reg=<requested/last|none|noref>".
// Writes nothing to the game; every line is also logged.
std::string DumpPlayer(int32_t aMax);

// Constructs gameItemEventsPropagateRenderingPlane through RTTI, writes the plane (int32 at +0x40) and queues it on the
// item (QueueEvent) and its child items / parts (QueueEventToChildItems). Game thread (script native) only.
bool QueuePlaneEvent(RED4ext::IScriptable* aItem, int32_t aPlane, std::string& aWhy);
void CountEvent(int32_t aPlane, bool aOk);

std::string Status();
} // namespace utpp::itemplane

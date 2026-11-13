// UTPP ragdoll: pose slot lookup, episode glue and script natives. See RagdollSlotProbe.cpp; the pure parts are
// RdKeep.hpp (keep store), RagdollEpisode.hpp (episode state machine, pelvis telemetry, native signatures).
#pragma once

#include <cstdint>
#include <string>

#include <RED4ext/RED4ext.hpp>

#include "RagdollEpisode.hpp"
#include "RdKeep.hpp"

namespace utpp::ragdollslot
{
// Load time, only on a validated 2.50 game (prologue checked again here, fail-closed: no hook, natives report "not installed").
void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle);
void Shutdown();
// PostRegisterTypes: the production natives (RagdollEpisode.hpp header comment) and the optional probes
// UTPPRagdollSlotDump(comp: ref<IComponent>) -> String and UTPPRagdollEntityHash(comp: ref<IComponent>, other: Int32) -> Uint64.
void RegisterNatives();
// True while V's pose slot has its overlay gate (slot+0x29) set. V's slot is found natively (owner EntityID == the player's id, see
// DetourUpdate) and re-validated every job tick; only a component owned by the known player can be bound (also by the research probe
// UTPPRagdollSlotDump). false also when there is no reading at all. Any thread; SEH-guarded.
bool PlayerOverlayActive();

// The keep store (RdKeep.hpp) and the pelvis telemetry; both are process-wide singletons.
ragdoll::RdKeep& Keep();
ragdoll::PelvisTracker& Pelvis();

// Game thread, once per second while armed: the player entity pointer (entity+0x48 = EntityID hash). Idempotent; a change drops
// the slot cache and aborts a running episode.
void PublishPlayerEntity(const void* aEntity);
// Game thread, every frame: ticks the episode with the live gate, edge actions, the 1 Hz status line during an episode.
void OnFrame();
// Disarm / player gone: aborts the episode, invalidates the keep store, drops the slot cache.
void OnDisarm();
// Arm time: merged-rig indices of Hips, Spine, LeftUpLeg, RightUpLeg (-1 = absent -> no pelvis telemetry).
void SetPelvisBones(int32_t aHips, int32_t aSpine, int32_t aLeftUpLeg, int32_t aRightUpLeg);
// FK detour, after the original FK: samples the pelvis from the player's full descriptor (cheap reject for every other call).
void OnPlayerFk(const uint8_t* aDesc, void* aRig, int64_t aTicks);
} // namespace utpp::ragdollslot

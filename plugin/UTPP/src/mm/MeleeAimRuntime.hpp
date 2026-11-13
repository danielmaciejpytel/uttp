#pragma once
// Melee aim tilt runtime (mm/MeleeAim.cpp): lock-free state between the game thread (natives, arming, camera target) and the FK
// detour (animation threads). Core math: mm/MeleeAim.hpp.
//
// Data flow:
//   utpp_camera (CET, every third-person camera frame) -> UTPPSetCameraTarget(org, fwd, enabled) -> Main.cpp: aim point
//   P = org + norm(fwd) * 60 m (the crosshair point when utpp_camera knows it) in the player's model space -> SetTarget();
//   MMDriver PublishLocked -> PoseSnapshot::meleeAimWeight (fade of the v1 melee mask reason);
//   FK detour -> ApplyPoseSnapshot -> BlendIntoRecords -> aimofs::Apply -> Apply() here on the local records.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace utpp::meleeaim
{
// Game thread, arm time: resolves Spine1 / Spine2 / Spine3 / Neck by name (fail closed).
void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents);
void OnDisarmed();
// Game thread: aim point in model space (aEnabled false = no target; aValid false = the player transform was unavailable).
void SetTarget(bool aEnabled, bool aValid, const float aTargetModel[3]);
// Game thread (UTPPSetMeleeAimTarget, script <= 20 Hz while a melee weapon is drawn): the game's melee target, its feet in model
// space (aValid false = no target). Preferred over the camera point while fresh (core::kMaxLockAge).
void SetLockTarget(bool aValid, const float aFeetModel[3]);
// maxUp 0..45, maxDown 0..60 deg, smoothHz 0..120 (1/s). maxUp = maxDown = 0 = off (the DLL default). False = invalid, kept.
bool SetParams(float aMaxUpDeg, float aMaxDownDeg, float aSmoothHz);
std::string Status();
// Game thread (MMDriver Feed, trace on): one 'UTPP MELEEAIM ...' line into aBuf; false = nothing new to report (no applied
// call since the last line and the tilt is idle). Throttling is the caller's.
bool FormatLogLine(char* aBuf, size_t aLen);
// True while the last FK call applied a non-zero tilt (diagnostics / log edges).
bool Active();

// FK detour (animation thread), after the MM blend and the gun aim offset. aMeleeWeight = PoseSnapshot::meleeAimWeight,
// aMMWeight / aUpperScale from the same snapshot. No allocation, no locks (a try-flag owns the smoothing state; a concurrent call
// re-applies the last angles), no logging.
void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, int64_t aNowTicks);
} // namespace utpp::meleeaim

#pragma once
// Head look runtime (mm/HeadLook.cpp): lock-free state between the game thread (native, arming) and the FK detour (animation
// threads). Core math and the layer: mm/HeadLook.hpp.
//
// Data flow:
//   utpp_core (CET, every tick while its head look is on) -> Game.UTPPSetHeadLook(target, weight, mode, chest, maxYaw) -> Main.cpp:
//   the world point in the player's model space -> SetTarget() (seqlock; the tick is the heartbeat: older than 0.25 s = no target);
//   FK detour -> ApplyPoseSnapshot -> BlendIntoRecords -> aimofs::Apply -> meleeaim::Apply -> Apply() here on the local records.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace utpp::headlook
{
// Game thread, arm time: resolves Spine3 / Neck / Neck1 / Head / LeftEye / RightEye by name (fail closed).
void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents);
void OnDisarmed();
// Game thread: target in model space. aOn false (or weight <= 0) = release. aValid false = the player transform was unavailable.
// aMode 0 camera / 1 face / 2 loot, aChest 0..1, aMaxYawDeg 0..160. Returns false (and keeps the last state) for out-of-range values.
bool SetTarget(bool aOn, bool aValid, const float aTargetModel[3], float aWeight, int32_t aMode, float aChest, float aMaxYawDeg);
// True when the rig resolved at arm time.
bool RigOk();
// True when the layer can turn the head now: the rig resolved and the gate (motion pose owns the upper body, no melee fade) has not been
// closed for a target within the last 0.5 s. False while the game owns the head (reload / shot / aim mask ...): the caller keeps the game's
// own look-at request meanwhile.
bool CanAct();
std::string Status();

// FK detour (animation thread), after the MM blend, the gun aim offset and the melee tilt. aMMWeight / aUpperScale / aMeleeWeight /
// aOverlay (PoseOverlay as int) from the same snapshot. No allocation, no locks (a try-flag owns the layer state; a concurrent call
// re-applies the last output), no logging. Idle (no target, springs at rest): two relaxed loads, no write.
void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, int32_t aOverlay,
           int64_t aNowTicks);
} // namespace utpp::headlook

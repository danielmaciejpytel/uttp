#pragma once
// Melee chest base correction runtime (mm/MeleeBase.cpp): lock-free state for the FK detour. Core math and the clip table: mm/MeleeBase.hpp,
// mm/MeleeBaseData.hpp. The body (clip table) comes from mm/MeleeBodyId.hpp.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "GameIntegration.hpp"
#include "MMMath.hpp"

namespace utpp::meleebase
{
// Game thread, arm time: resolves Hips / Spine..Spine3 and the key bones by name (fail closed) and hard-resets the layer's state
// (hints, gate, enable ease, lower body guard latch).
void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents);
void OnDisarmed();
std::string Status();
// ' base=...' diagnostics appended to the 'UTPP MELEEAIM' line (aBuf already holds that line). Returns the characters added.
size_t AppendLog(char* aBuf, size_t aLen, size_t aUsed);
int64_t TicksPerSecond(); // the unit of the aNowTicks arguments below (QueryPerformanceCounter)

// FK detour, BEFORE the MM blend (same thread, same call as the Apply below): remembers the key bones' rotations of the game pose. The
// match against the clip table is then made on the pose the game plays and not on the records the MM blend has mixed (during the 0.15 s
// upper-body mask blend-in they are 0..100 % MM pose and match no clip row, so the correction used to start late). aMeleeWeight =
// PoseSnapshot::meleeAimWeight (0 = melee not active: nothing is read). Cheap: a seqlock read and 13 quaternions.
void CaptureEngineKeys(const uint8_t* aBones, uint32_t aCount, void* aRig, float aMeleeWeight);

// FK detour, same call, between CaptureEngineKeys and Apply: copies the captured key bones of the game pose (kKeyBones quaternions,
// kKeyNames order) without consuming them. False when none were captured for this call / rig.
bool PeekEngineKeys(void* aRig, mm::math::Quat* aOut);
// FK detour, same call, before Apply: the share (0..1) with which the melee full pose (mm/MeleePoseRuntime.hpp) wrote the clip's lower
// body. Apply scales the chest correction by (1 - share), so the two never stack; reset by Apply.
void SetFullPoseShare(float aShare);

// FK detour (animation thread), after the MM blend and the gun aim offset, BEFORE the aim tilt. aMeleeWeight = PoseSnapshot::meleeAimWeight,
// aMMWeight / aUpperScale from the same snapshot, aEnabled = PoseSnapshot::meleeBase (fists drawn, standing on the ground): a TARGET weight
// eased over 0.2 s both ways, not a switch. No allocation, no locks, no logging.
void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, uint32_t aEnabled,
           int64_t aNowTicks);

// FK detour, before the MM blend, only while the melee full body hands the lower body to the game pose (snapshot lowerScale < 1): the weight
// (0..1) with which Hips / Spine / legs stay on MM instead (1 = the game pose of Hips is not where MM's Hips are: the kneeling engine base of
// Male V). Hysteresis on the decision and an ease of 0.2 s on the weight: no snap between engine and MM legs mid-episode.
float LowerBodyKeepWeight(const uint8_t* aBones, uint32_t aCount, void* aRig, const mm::integ::PoseSnapshot& aSnap, int64_t aNowTicks);
// FK detour, every other call (the melee full body is not handing the lower body to the game): clears the latch, returns 0.
float LowerBodyReset();

struct Counters
{
    uint64_t calls = 0, matched = 0, matchedFemale = 0, matchedMale = 0, noBody = 0, applied = 0;
};
Counters GetCounters(); // tests / diagnostics
struct LastStep // the last Apply that ran the layer (tests / the offline replay)
{
    int32_t body = -1, clip = -1, frame = -1, neighbour = -1;
    float t = 0, rmsDeg = 0, gate = 0, weight = 0, applied[3] = {0, 0, 0};
    float raw[3] = {0, 0, 0}, lim[3] = {0, 0, 0}; // yaw / pitch / roll, degrees
    uint32_t flags = 0;
    bool matched = false;
};
LastStep GetLastStep();
} // namespace utpp::meleebase

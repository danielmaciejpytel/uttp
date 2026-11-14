#pragma once
// Melee full pose runtime (mm/MeleePose.cpp): the clip's lower body while standing melee strikes run. Core: mm/MeleePose.hpp.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace utpp::meleepose
{
// Plugin init: loads <aDataDir>utpp_melee_pose_female.bin / _male.bin (aDataDir ends with a separator; the plugin passes
// <dll dir>\data\). Fail closed per body: a missing or invalid file leaves that body on the chest correction alone. Game thread,
// before any FK call.
void Load(const std::wstring& aDataDir);
// Game thread, arm time: resolves the key and lower-body bones by name (fail closed) and resets the layer.
void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents);
void OnDisarmed();
// Script: the anim variable (CName hash) of the drawn melee weapon; 0 = none / unknown (the fists section is used).
void SetWeapon(uint64_t aVariableHash);
// Feed: the PSM melee weapon state (-1 = not reported) and the planar capsule speed (m/s). Held states (hold, charged hold, block,
// targeting, deflect) stand the clip clock while the upper body is still; a moving capsule holds no foot.
void SetMeleeState(int32_t aState, float aPlanarSpeed);
// Game thread, about once per second while diagnostic logs are on: 'UTPP MELEEPOSE key=value ...' with the cumulative counters of the
// layer, its borders and the reference pose guard. False = nothing ran since the last line (no line).
bool FormatLogLine(char* aBuf, size_t aLen);
// User setting melee.fullPose (default on): off = the chest correction alone, as before.
void SetEnabled(bool aOn);
// Developer setting melee.switchBlend (default off): at a clip switch the arms and the spine above the layer blend from the pose of
// the previous call into the new clip over 0.1 s instead of the game's hard cut. Off: the upper body is left as the game plays it.
void SetSwitchBlend(bool aOn);
std::string Status();

// FK detour, right after the MM blend (before the gun aim offset and the chest correction). aLowerScale = PoseSnapshot::lowerScale
// (1 - the melee full-body fade: the standing gate and its fades). Writes Hips / Spine / legs of the matched clip; tells the chest
// correction of the same call its share (MeleeBaseRuntime.hpp SetFullPoseShare). No allocation, no locks, no logging.
void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aLowerScale, int64_t aNowTicks);

// FK detour, FIRST melee step of the call (before the lower-body keep, the key capture and the MM blend read the engine pose): the
// reference pose guard (mm/MeleePoseGuard.hpp). aMeleeWeight = PoseSnapshot::meleeAimWeight (0 = melee mask reason off: nothing is
// stored or written). A frame on the skeleton's reference pose gets the last good engine pose of the Spine subtree back (at most two
// calls). No allocation, no locks, no logging.
void GuardEngine(uint8_t* aBones, uint32_t aCount, void* aRig, float aMeleeWeight, int64_t aNowTicks);

struct Counters
{
    uint64_t calls = 0, written = 0, held = 0, relocks = 0, noTable = 0, noWeapon = 0, busy = 0;
    uint64_t guardCalls = 0, guardHeld = 0, guardPassed = 0, guardBusy = 0;
    uint64_t frozen = 0, toBase = 0, borderCalls = 0, tailCalls = 0, planted = 0, steps = 0, hipsLimited = 0;
    double meanMicros = 0.0, maxMicros = 0.0;
};
Counters GetCounters();
struct LastStep // the last Apply that ran the layer (tests / the offline replay)
{
    int32_t body = -1, clip = -1; // clip: global clip index of the body's table
    float clockRow = 0.0f, weight = 0.0f, rmsDeg = 0.0f;
    float hipsOffset = 0.0f; // borders: Hips height shown minus the layer's written height (m)
    float clipShare = 1.0f;  // share of the locked clip in the written pose (below 1 during a relock cross-fade)
    bool matched = false, held = false;
};
LastStep GetLastStep();
} // namespace utpp::meleepose

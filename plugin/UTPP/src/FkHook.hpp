#pragma once
#include <cstdint>
#include <span>
#include "mm/GameIntegration.hpp"

namespace utpp
{
// Detour for FK (RVA 0x582B3C). Signature as observed: FK(PoseDesc* D, Rig* R). The return value is
// passed through as void* in case the original leaves something meaningful in RAX.
using FK_t = void* (*)(void* aDesc, void* aRig);

void* FkDetour(void* aDesc, void* aRig);
FK_t* FkOriginalSlot();
// Ragdoll R1o probe (ragdoll/RdPoseTable.hpp): merged-rig Hips / Head / LeftFoot indices, set at arm time (-1 = off).
void RdFkTraceSetBones(int32_t aHips, int32_t aHead, int32_t aFoot);
// Shared capture/blend/AO pipeline. The caller supplies one already-read
// snapshot and wraps live memory accesses in SEH. No snapshot reread occurs.
uint32_t ApplyPoseSnapshot(void* aUser, uint8_t* aDesc, uint8_t* aBones, uint32_t aCount,
                           uint64_t aMergedRig, const mm::integ::PoseSnapshot& aSnapshot, int64_t aNow);

// Shadow skeleton of the player pose: 26 shadow_* bones in their own hierarchy, filled by the game from the main bones in a merge
// pass that runs before the FK owner's write. After the FK owner's write the shadow bones are set again from the main bones, so the
// shadow follows the written pose: shadow_X = X for 24 bones with the same parent chain, and shadow_ToeBase = Heel * ToeBase (the
// shadow toe hangs under the shadow foot, the main toe under the heel). Not used under the Mapping owner (the game's pass runs after
// that write).
namespace shadowcopy
{
enum class ArmStatus : uint32_t { Off, NoShadowBones, Ready, MissingBone, ShapeMismatch };
// Arm time, before armed is published. Returns the number of pairs (0 = off for this rig).
uint32_t Arm(uint64_t aRig, std::span<const uint64_t> aNames, std::span<const int16_t> aParents);
void Disarm();
ArmStatus Status();
const char* StatusName();
// Sets the shadow bones of a pose of the armed rig from its main bones; returns the bones written (0 = rig or count differ).
uint32_t Apply(uint8_t* aBones, uint32_t aCount, uint64_t aRig);
} // namespace shadowcopy
} // namespace utpp

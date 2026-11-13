#pragma once
#include <cstdint>
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
} // namespace utpp

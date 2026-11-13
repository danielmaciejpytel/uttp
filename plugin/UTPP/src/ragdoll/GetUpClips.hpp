#pragma once
// Frame-0 poses of the motion get-up clips used by the R0 probe (data: GetUpFrame0.hpp, generated). The installed v2 databases do
// not contain these clips, so the probe publishes this table. Clip 0 = M_Neutral_GetUp_Back (on the back), 1 = M_Neutral_GetUp_Front.
#include <cstdint>
#include <cstring>

#include "GetUpFrame0.hpp"

namespace utpp::ragdoll
{
inline constexpr int32_t kGetUpClipCount = 2;

inline const char* GetUpClipName(int32_t aClip) { return aClip == 0 ? "M_Neutral_GetUp_Back" : (aClip == 1 ? "M_Neutral_GetUp_Front" : "?"); }

// kGetUpBones * 7 floats (local p.xyz, q.xyzw per v2 database bone), nullptr for an unknown clip.
inline const float* GetUpFrame0(bool aWoman, int32_t aClip)
{
    if (aClip == 0)
        return aWoman ? kGetUpWomanBackFrame0 : kGetUpManBackFrame0;
    if (aClip == 1)
        return aWoman ? kGetUpWomanFrontFrame0 : kGetUpManFrontFrame0;
    return nullptr;
}

// True when the first kGetUpBones database bone names equal the table's order (guards the table against a re-baked bone order).
template <class NameFn>
inline bool GetUpBoneOrderMatches(uint32_t aNumBones, NameFn&& aName)
{
    if (aNumBones != kGetUpBones)
        return false;
    for (uint32_t i = 0; i < kGetUpBones; ++i)
        if (aName(i) != kGetUpBoneNames[i])
            return false;
    return true;
}
} // namespace utpp::ragdoll

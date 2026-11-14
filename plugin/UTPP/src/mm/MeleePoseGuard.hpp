#pragma once
// Melee reference pose guard. Header-only, no RED4ext / Windows types; unit-tested offline (tests/MeleePoseTest.cpp, synthetic rigs and
// recorded engine poses). Runtime glue: mm/MeleePose.cpp (GuardEngine, FK detour, before anything reads or blends the engine pose).
//
// When a melee strike starts (and when an item is drawn) the game's graph hands the upper-body slot over without data for one frame:
// Spine and everything below it (spine, neck, head, both arms, fingers and their helper joints) sit on the skeleton's reference pose for
// that frame, then the next clip's first frame follows. The legs and Hips keep the base pose. The reference pose stores the same local
// rotation on both upper arms (mirrored joint axes), which no animated frame does. While the melee mask reason is on, such a frame is
// replaced by the last good engine pose of the same bones, for at most kMaxHoldCalls consecutive calls and only while that pose is fresh
// (kMaxHoldSeconds). A frame that shows the signature but barely moves against the last good pose is left alone (a pose that really
// rests there), and a signature that lasts longer than the cap passes through unchanged.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>

#include "GameIntegration.hpp"
#include "MMMath.hpp"

namespace utpp::mm::meleepose
{
inline constexpr uint32_t kGuardMaxBones = 1024;     // bones of the held subtree (fail closed above)
inline constexpr float kGuardSignatureDeg = 0.5f;    // LeftArm local vs RightArm local: the reference pose stores the same rotation
inline constexpr float kGuardMinStepDeg = 20.0f;     // ... and an upper arm turned at least this much against the last good pose
inline constexpr int kGuardMaxHoldCalls = 2;         // at most this many consecutive calls are replaced
inline constexpr float kGuardMaxHoldSeconds = 0.1f;  // the last good pose older than this is not used (a frame hitch of 74 ms was recorded)

struct GuardRig
{
    uint32_t ok = 0;
    int32_t leftArm = -1, rightArm = -1;
    uint32_t leftSlot = 0, rightSlot = 0; // position of the upper arms in bones[] (= in the stored pose)
    uint32_t count = 0;                   // bones of the Spine subtree (Spine included)
    uint16_t bones[kGuardMaxBones] = {};
};

// Spine and every descendant (by the rig's parent indices; a parent may come after its child), plus both upper arms. Fail closed: a missing
// name, a broken parent table or a subtree larger than kGuardMaxBones -> ok = 0.
inline GuardRig ResolveGuardRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    GuardRig r;
    const size_t n = aHashes.size();
    if (n == 0 || aParents.size() != n || n > 65535)
        return GuardRig{};
    int32_t spine = -1;
    const uint64_t hs = integ::CNameHash("Spine"), hl = integ::CNameHash("LeftArm"), hr = integ::CNameHash("RightArm");
    for (size_t i = 0; i < n; ++i)
    {
        if (aHashes[i] == hs)
            spine = int32_t(i);
        else if (aHashes[i] == hl)
            r.leftArm = int32_t(i);
        else if (aHashes[i] == hr)
            r.rightArm = int32_t(i);
    }
    if (spine < 0 || r.leftArm < 0 || r.rightArm < 0)
        return GuardRig{};
    for (size_t i = 0; i < n; ++i)
    {
        // walk to the root; a chain longer than the rig is a cycle
        int32_t k = int32_t(i);
        size_t steps = 0;
        bool inside = false;
        while (k >= 0 && steps <= n)
        {
            if (k == spine)
            {
                inside = true;
                break;
            }
            const int16_t p = aParents[size_t(k)];
            if (p >= int32_t(n))
                return GuardRig{};
            k = p;
            ++steps;
        }
        if (steps > n)
            return GuardRig{};
        if (inside)
        {
            if (r.count >= kGuardMaxBones)
                return GuardRig{};
            r.bones[r.count++] = uint16_t(i);
        }
    }
    bool hasL = false, hasR = false;
    for (uint32_t k = 0; k < r.count; ++k)
    {
        if (r.bones[k] == uint16_t(r.leftArm))
        {
            hasL = true;
            r.leftSlot = k;
        }
        if (r.bones[k] == uint16_t(r.rightArm))
        {
            hasR = true;
            r.rightSlot = k;
        }
    }
    if (!hasL || !hasR)
        return GuardRig{};
    r.ok = 1;
    return r;
}

struct GuardState
{
    bool have = false;   // rec holds the last good engine pose of the subtree
    int held = 0;        // consecutive replaced calls
    int64_t ticks = 0;   // time stamp of rec
    uint8_t rec[size_t(kGuardMaxBones) * integ::kGameBoneStride] = {};
};

enum class GuardResult : uint8_t
{
    Off,    // inactive / bad input: nothing stored, nothing written, state cleared
    Stored, // a normal frame: remembered as the last good pose
    Held,   // a reference-pose frame: replaced by the last good pose
    Passed  // a reference-pose frame that could not be replaced (no fresh pose / cap reached / no step): left as it is
};

inline math::Quat GuardRot(const uint8_t* aRec, uint32_t aBone)
{
    const float* r = reinterpret_cast<const float*>(aRec + size_t(aBone) * integ::kGameBoneStride) + integ::kGameRotOffsetFloats;
    return math::Quat{r[0], r[1], r[2], r[3]};
}

inline float GuardAngleDeg(const math::Quat& aA, const math::Quat& aB)
{
    const float na = math::QuatDot(aA, aA), nb = math::QuatDot(aB, aB);
    if (!(na > 1e-12f) || !(nb > 1e-12f) || !std::isfinite(na) || !std::isfinite(nb))
        return 180.0f;
    float d = std::fabs(math::QuatDot(aA, aB)) / std::sqrt(na * nb);
    d = d > 1.0f ? 1.0f : d;
    return 2.0f * std::acos(d) * 57.2957795f;
}

// True when the records show the reference-pose signature: both upper arms carry the same local rotation.
inline bool GuardSignature(const uint8_t* aBones, const GuardRig& aRig)
{
    return GuardAngleDeg(GuardRot(aBones, uint32_t(aRig.leftArm)), GuardRot(aBones, uint32_t(aRig.rightArm))) < kGuardSignatureDeg;
}

// One FK call. aActive = the melee mask reason is on (PoseSnapshot::meleeAimWeight > 0); aAgeSeconds = time since aS.ticks (ignored
// without a stored pose). aBones = the engine's local records (aCount bones) before any mod layer reads or writes them.
inline GuardResult GuardStep(GuardState& aS, const GuardRig& aRig, uint8_t* aBones, uint32_t aCount, bool aActive, int64_t aNowTicks,
                             float aAgeSeconds)
{
    if (!aActive || !aRig.ok || !aBones || uint32_t(aRig.leftArm) >= aCount || uint32_t(aRig.rightArm) >= aCount)
    {
        aS.have = false;
        aS.held = 0;
        return GuardResult::Off;
    }
    for (uint32_t k = 0; k < aRig.count; ++k)
        if (aRig.bones[k] >= aCount)
        {
            aS.have = false;
            aS.held = 0;
            return GuardResult::Off;
        }
    if (GuardSignature(aBones, aRig))
    {
        const bool fresh = aS.have && std::isfinite(aAgeSeconds) && aAgeSeconds >= 0.0f && aAgeSeconds <= kGuardMaxHoldSeconds;
        const bool stepped = aS.have && (GuardAngleDeg(GuardRot(aBones, uint32_t(aRig.leftArm)), GuardRot(aS.rec, aRig.leftSlot)) >=
                                             kGuardMinStepDeg ||
                                         GuardAngleDeg(GuardRot(aBones, uint32_t(aRig.rightArm)), GuardRot(aS.rec, aRig.rightSlot)) >=
                                             kGuardMinStepDeg);
        if (fresh && stepped && aS.held < kGuardMaxHoldCalls)
        {
            for (uint32_t k = 0; k < aRig.count; ++k)
                std::memcpy(aBones + size_t(aRig.bones[k]) * integ::kGameBoneStride, aS.rec + size_t(k) * integ::kGameBoneStride,
                            integ::kGameBoneStride);
            ++aS.held;
            return GuardResult::Held;
        }
        // not replaceable: the stored pose stays (it is not refreshed with a reference frame), the cap keeps counting
        if (aS.held > 0)
            ++aS.held;
        return GuardResult::Passed;
    }
    for (uint32_t k = 0; k < aRig.count; ++k)
        std::memcpy(aS.rec + size_t(k) * integ::kGameBoneStride, aBones + size_t(aRig.bones[k]) * integ::kGameBoneStride,
                    integ::kGameBoneStride);
    aS.have = true;
    aS.held = 0;
    aS.ticks = aNowTicks;
    return GuardResult::Stored;
}
} // namespace utpp::mm::meleepose

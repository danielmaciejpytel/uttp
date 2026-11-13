#pragma once
// Complete preflight and narrowly bounded commit for an already-built helper
// patch. Caller owns freshness/pose authorization and supplies a final check.
#include "BodyHelperPatch.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>

namespace utpp::mm::bodyhelpercommit
{
namespace h = helperpatch;
namespace d = deformoffline;
// Legacy count of the first bundle schema (33 owned + 2 compensated children). The exact count of any plan is its outputCount.
inline constexpr uint32_t kRequiredOutputs = 35;

enum class Error : uint32_t
{
    None, Bounds, Plan, PatchCount, ApprovedIndex, DuplicateIndex,
    ProtectedCore, BeforeChanged, InvalidAfter, WChanged, Aliasing,
    PermissionDenied, TranslationJump
};
struct Result
{
    Error error = Error::None;
    uint32_t index = 0;
    explicit operator bool() const { return error == Error::None; }
};

using PermissionCallback = bool (*)(void*) noexcept;

// Safety net on the whole patch: if any helper's LOCAL translation would move further than this (metres, parent space) from the
// engine's own value, nothing is written (Error::TranslationJump). Measured on the recorded man frames (tests/BodyHelperBundleTest.cpp,
// stand / walk / aim, armed and unarmed): 0.0000 m on the raw engine pose, at most 0.277 m between the engine's stale helpers and the
// corrected ones. The cap is 3.6x that maximum, still far below the size of a body part, so it only catches a broken evaluation.
inline constexpr float kMaxTranslationDelta = 1.0f;

inline bool ValidAfter(const std::array<float,h::kRecordFloats>& r)
{
    // The two W/sentinel lanes are opaque bytes. Validate only the TRS lanes.
    for (uint32_t i : {0u,1u,2u,4u,5u,6u,7u,8u,9u,10u})
        if (!std::isfinite(r[i])) return false;
    d::Transform t;
    t.t={r[0],r[1],r[2]}; t.q={r[4],r[5],r[6],r[7]}; t.s={r[8],r[9],r[10]};
    return d::detail::Valid(t) && t.s.x>1e-6f && t.s.y>1e-6f && t.s.z>1e-6f;
}

// No allocation, I/O or fallback. Preflight failure makes no writes. The callback is invoked
// exactly once after every byte/index/value check and immediately before the
// first record copy. It must recheck the caller's fresh gate/generation.
inline Result Commit(const h::Plan& plan, std::span<float> live,
                     const h::Patch& patch, PermissionCallback permission,
                     void* user) noexcept
{
    if (!plan.valid || !plan.count || plan.count>h::kMaxRecords ||
        !plan.outputCount || plan.outputCount>h::kMaxOutputs ||
        live.size()<size_t(plan.count)*h::kRecordFloats)
        return {Error::Bounds};
    if (patch.count!=plan.outputCount) return {Error::PatchCount};

    std::array<uint8_t,h::kMaxRecords> approved{};
    std::array<uint8_t,h::kMaxRecords> covered{};
    for (uint32_t k=0;k<plan.outputCount;++k)
    {
        const uint32_t i=plan.outputs[k];
        if (i>=plan.count) return {Error::ApprovedIndex,k};
        if (approved[i]) return {Error::DuplicateIndex,k};
        if (plan.core[i]) return {Error::ProtectedCore,k};
        approved[i]=1;
    }
    for (uint32_t k=0;k<patch.count;++k)
    {
        const uint32_t i=patch.indices[k];
        if (i>=plan.count || !approved[i]) return {Error::ApprovedIndex,k};
        if (covered[i]) return {Error::DuplicateIndex,k};
        covered[i]=1;
        const float* current=live.data()+size_t(i)*h::kRecordFloats;
        if (std::memcmp(patch.before[k].data(),current,sizeof(float)*h::kRecordFloats)!=0)
            return {Error::BeforeChanged,k};
        if (std::memcmp(&patch.before[k][3],&patch.after[k][3],sizeof(float))!=0 ||
            std::memcmp(&patch.before[k][11],&patch.after[k][11],sizeof(float))!=0)
            return {Error::WChanged,k};
        if (!ValidAfter(patch.after[k])) return {Error::InvalidAfter,k};
        const float dx=patch.after[k][0]-patch.before[k][0], dy=patch.after[k][1]-patch.before[k][1], dz=patch.after[k][2]-patch.before[k][2];
        if (!(dx*dx+dy*dy+dz*dz <= kMaxTranslationDelta*kMaxTranslationDelta)) return {Error::TranslationJump,k};
    }
    for (uint32_t k=0;k<plan.outputCount;++k)
        if (!covered[plan.outputs[k]]) return {Error::ApprovedIndex,k};

    // Reject accidental overlap: the sources must remain immutable while the
    // destinations are copied. Address arithmetic is bounded by above limits.
    const uintptr_t liveBegin=reinterpret_cast<uintptr_t>(live.data());
    const uintptr_t liveBytes=size_t(plan.count)*h::kRecordFloats*sizeof(float);
    const uintptr_t patchBegin=reinterpret_cast<uintptr_t>(&patch);
    const uintptr_t patchEnd=patchBegin+sizeof(patch);
    if (liveBegin>UINTPTR_MAX-liveBytes || patchBegin>UINTPTR_MAX-sizeof(patch) ||
        (liveBegin<patchEnd && patchBegin<liveBegin+liveBytes))
        return {Error::Aliasing};

    if (!permission || !permission(user)) return {Error::PermissionDenied};
    for (uint32_t k=0;k<patch.count;++k)
        std::memcpy(live.data()+size_t(patch.indices[k])*h::kRecordFloats,
                    patch.after[k].data(),sizeof(float)*h::kRecordFloats);
    return {};
}
} // namespace utpp::mm::bodyhelpercommit

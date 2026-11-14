#pragma once
// Pure offset arithmetic of the helper recompute on the frames of the weight-0 core writers (vehicle transition, vehicle seat guard
// and pop filter, seated neck cut, vault torso). No hooks, no I/O, no allocation.
//
// The engine's own helper values are not always equal to the deformer graph: on some frames (vehicle exits of the woman body) they
// differ by up to a few centimetres for seconds, with no mod write at all. A plain recompute on the frames a weight-0 writer changes
// would switch between the graph (written frames) and the engine (the frames around them) at every border of a written run. Instead the
// recompute keeps the engine's deviation:
//   offset  = inverse(graph LOCAL) * engine LOCAL, per helper record, measured on a frame no layer wrote (identity after a frame whose
//             helpers were committed from the graph);
//   written = graph LOCAL of the written pose * offset carried from the frame before the run;
//   after the run the next frames move the offset from the carried one to the engine's live one over kFadeSeconds (linear in time),
//   so the end of a run is not a step either.
#include "BodyDeformerOffline.hpp"
#include "BodyHelperPatch.hpp"

#include <array>
#include <cstdint>

namespace utpp::mm::helpercarry
{
namespace d = deformoffline;
namespace h = helperpatch;

inline constexpr float kFadeSeconds = 0.4f;
using Offsets = std::array<d::Transform, h::kMaxOutputs>;

// Fade weight toward the engine's live offset after the last written frame: 0 at the written frame, 1 from kFadeSeconds on. Non-positive
// or non-finite input (no written frame, clock error) gives 1 (no fade).
inline float FadeAlpha(int64_t aNow, int64_t aLastWritten, int64_t aTicksPerSecond)
{
    if (aLastWritten <= 0 || aTicksPerSecond <= 0 || aNow < aLastWritten)
        return 1.0f;
    const double s = double(aNow - aLastWritten) / double(aTicksPerSecond);
    const double a = s / double(kFadeSeconds);
    return a >= 1.0 ? 1.0f : float(a);
}

inline bool DecodeRecord(const std::array<float, h::kRecordFloats>& aRecord, d::Transform& aOut)
{
    return h::Decode(aRecord.data(), aOut);
}

inline void EncodeRecord(const d::Transform& aIn, std::array<float, h::kRecordFloats>& aRecord)
{
    aRecord[0] = aIn.t.x; aRecord[1] = aIn.t.y; aRecord[2] = aIn.t.z;
    aRecord[4] = aIn.q.x; aRecord[5] = aIn.q.y; aRecord[6] = aIn.q.z; aRecord[7] = aIn.q.w;
    aRecord[8] = aIn.s.x; aRecord[9] = aIn.s.y; aRecord[10] = aIn.s.z;
    // [3] and [11] keep their original bytes (as in helperpatch::Emit)
}

// Offsets of the engine records (patch.before) against the graph records (patch.after) of one built patch.
inline bool Measure(const h::Patch& aPatch, Offsets& aOut)
{
    if (aPatch.count > h::kMaxOutputs)
        return false;
    for (uint32_t k = 0; k < aPatch.count; ++k)
    {
        d::Transform engine, graph;
        if (!DecodeRecord(aPatch.before[k], engine) || !DecodeRecord(aPatch.after[k], graph) ||
            !d::detail::Relative(graph, engine, aOut[k]))
            return false;
    }
    return true;
}

// patch.after[k] = graph * offset[k] for every record; an identity offset leaves the graph records as they are. All-or-nothing: on a
// failure the patch is left unchanged.
inline bool Apply(h::Patch& aPatch, const Offsets& aOffsets, bool aIdentity)
{
    if (aPatch.count > h::kMaxOutputs)
        return false;
    if (aIdentity)
        return true;
    std::array<d::Transform, h::kMaxOutputs> out{};
    for (uint32_t k = 0; k < aPatch.count; ++k)
    {
        d::Transform graph;
        if (!DecodeRecord(aPatch.after[k], graph) || !d::detail::Compose(graph, aOffsets[k], out[k]) ||
            !(out[k].s.x > 1e-6f && out[k].s.y > 1e-6f && out[k].s.z > 1e-6f))
            return false;
    }
    for (uint32_t k = 0; k < aPatch.count; ++k)
        EncodeRecord(out[k], aPatch.after[k]);
    return true;
}

// out = carried (alpha 0) .. live (alpha 1): translation and scale linear, rotation slerp. aCarriedIdentity: the carried offset is the
// identity (the frame before the run showed the graph).
inline bool Blend(const Offsets& aCarried, bool aCarriedIdentity, const Offsets& aLive, uint32_t aCount, float aAlpha, Offsets& aOut)
{
    if (aCount > h::kMaxOutputs || !(aAlpha >= 0.0f && aAlpha <= 1.0f))
        return false;
    for (uint32_t k = 0; k < aCount; ++k)
    {
        const d::Transform a = aCarriedIdentity ? d::Transform{} : aCarried[k];
        const d::Transform& b = aLive[k];
        d::Transform o;
        o.t = d::detail::Add(d::detail::Mul(a.t, 1.0f - aAlpha), d::detail::Mul(b.t, aAlpha));
        o.s = d::detail::Add(d::detail::Mul(a.s, 1.0f - aAlpha), d::detail::Mul(b.s, aAlpha));
        if (!d::detail::Slerp(a.q, b.q, aAlpha, o.q) || !d::detail::Valid(o))
            return false;
        aOut[k] = o;
    }
    return true;
}
} // namespace utpp::mm::helpercarry

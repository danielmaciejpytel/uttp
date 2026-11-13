// UTPP motion matching runtime - additive pose primitives for the ALS layering port (port stage 10.6).
// Header-only, allocation-free (fixed-capacity poses), no game types.
//
// Semantics follow the UE animation runtime (primitives, UE AnimNode_MakeDynamicAdditive.cpp:43-64,
// AnimationRuntime.cpp ConvertTransformToAdditive / BlendFromIdentityAndAccumulate / BlendPosesPerBoneFilter,
// AnimNode_TwoWayBlend.cpp:52-57) and the lean / aim layers (lean BS1D, aim offset grid, aim sweep, spine rotation, hand fix):
//   D_MS(A->X): dq = Q^X * (Q^A)^-1 (mesh-space rotations), dt = t^X - t^A, ds = s^X / s^A - 1
//   D_LS(A->X): the same on local rotations
//   A_MS(Y, d, a): Q <- nlerp(1, dq, a) * Q in mesh space, t += a*dt, s *= (1 + a*ds)   (translations stay local)
//   A_LS: the same on local rotations
//   Blend(A, B, a): TwoWayBlend, a = clamp(a, 0, 1), shortest-path nlerp / lerp per bone (local)
//   LBB_MS: LayeredBoneBlend, rotations blended in mesh space (weight 0 keeps the base MESH rotation),
//           translation / scale blended locally; LBB_LS: everything local.
//   branch weight = clamp(inc * (depth + 1), 0, 1) with inc = 1 / BlendDepth (1 when BlendDepth <= 0), max over
//   branches, times the layer weight.
// Quaternions x, y, z, w, Hamilton product (MMMath.hpp). Mesh rotation of bone i = Q(parent) * q(i).
// Bones must be in topological order (parent index < bone index), like every CP rig and .utppmm.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "MMMath.hpp"

namespace utpp::mm::ov
{
using math::Quat;
using math::Vec3;

inline constexpr int kMaxBones = 128;
inline constexpr float kZeroWeight = 1e-5f; // UE ZERO_ANIMWEIGHT_THRESH

struct Xf
{
    Quat r{};
    Vec3 t{};
    Vec3 s{1.0f, 1.0f, 1.0f}; // pose: scale; additive: scale delta (s^X / s^A - 1)
};

struct Skeleton
{
    int n = 0;
    int16_t parent[kMaxBones]{}; // -1 = root

    bool Valid() const
    {
        if (n <= 0 || n > kMaxBones)
            return false;
        for (int i = 0; i < n; ++i)
            if (parent[i] >= i || parent[i] < -1)
                return false;
        return true;
    }
    bool IsAncestorOrSelf(int aAncestor, int aBone) const
    {
        for (int b = aBone; b >= 0; b = parent[b])
            if (b == aAncestor)
                return true;
        return false;
    }
    int DepthBelow(int aAncestor, int aBone) const // -1 when aBone is not in the subtree of aAncestor
    {
        int d = 0;
        for (int b = aBone; b >= 0; b = parent[b], ++d)
            if (b == aAncestor)
                return d;
        return -1;
    }
};

struct Pose // local space
{
    int n = 0;
    Xf b[kMaxBones];
};

struct Additive // per-bone deltas (r = dq, t = dt, s = ds); meshSpace selects A_MS / A_LS application
{
    int n = 0;
    bool meshSpace = true;
    Xf b[kMaxBones];

    void SetIdentity(int aCount, bool aMeshSpace)
    {
        n = aCount;
        meshSpace = aMeshSpace;
        for (int i = 0; i < n; ++i)
            b[i] = Xf{Quat{}, Vec3{}, Vec3{}};
    }
};

struct BoneWeights
{
    int n = 0;
    float w[kMaxBones]{};
    void Clear(int aCount)
    {
        n = aCount;
        for (int i = 0; i < kMaxBones; ++i)
            w[i] = 0.0f;
    }
};

// ---- small helpers --------------------------------------------------------------------------------------------
inline float Clamp01(float a) { return a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : (std::isfinite(a) ? a : 0.0f)); }
inline Vec3 Mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline float SafeRecip(float a) { return std::fabs(a) > 1e-8f ? 1.0f / a : 0.0f; } // UE GetSafeScaleReciprocal

// UE FQuat::FastLerp(A, B, a) followed by normalization (shortest path, bias by sign of the dot product).
inline Quat Nlerp(const Quat& a, const Quat& b, float t)
{
    const float bias = math::QuatDot(a, b) >= 0.0f ? 1.0f : -1.0f;
    const float u = 1.0f - t;
    return math::QuatNormalize(Quat{a.x * u + b.x * t * bias, a.y * u + b.y * t * bias, a.z * u + b.z * t * bias,
                                    a.w * u + b.w * t * bias});
}
inline Quat ScaleFromIdentity(const Quat& dq, float a) { return Nlerp(Quat{}, dq, a); } // nlerp(1, dq, a)

inline float AngleDeg(const Quat& a, const Quat& b)
{
    // atan2 form: no float acos floor near 0 (acos(1 - eps) ~ 0.04 deg).
    const Quat r = math::QuatMul(math::QuatInv(math::QuatNormalize(a)), math::QuatNormalize(b));
    const float v = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    return 2.0f * std::atan2(v, std::fabs(r.w)) * (180.0f / math::kPi);
}

// ---- space conversion -----------------------------------------------------------------------------------------
inline void MeshRotations(const Skeleton& aSk, const Pose& aPose, Quat* aOut)
{
    for (int i = 0; i < aSk.n; ++i)
    {
        const int p = aSk.parent[i];
        aOut[i] = p < 0 ? aPose.b[i].r : math::QuatNormalize(math::QuatMul(aOut[p], aPose.b[i].r));
    }
}
// Writes local rotations of aPose from mesh rotations (UE ConvertMeshRotationPoseToLocalSpace).
inline void LocalFromMeshRotations(const Skeleton& aSk, const Quat* aMesh, Pose& aPose)
{
    for (int i = 0; i < aSk.n; ++i)
    {
        const int p = aSk.parent[i];
        aPose.b[i].r = p < 0 ? aMesh[i] : math::QuatNormalize(math::QuatMul(math::QuatInv(aMesh[p]), aMesh[i]));
    }
}
// Full mesh-space positions (for tests / IK; the layering itself never needs them).
inline void MeshPositions(const Skeleton& aSk, const Pose& aPose, Vec3* aPos, Quat* aRot)
{
    for (int i = 0; i < aSk.n; ++i)
    {
        const int p = aSk.parent[i];
        if (p < 0)
        {
            aRot[i] = aPose.b[i].r;
            aPos[i] = aPose.b[i].t;
        }
        else
        {
            aRot[i] = math::QuatNormalize(math::QuatMul(aRot[p], aPose.b[i].r));
            aPos[i] = aPos[p] + math::QuatRotate(aRot[p], aPose.b[i].t);
        }
    }
}

// ---- D_MS / D_LS ----------------------------------------------------------------------------------------------
inline void MakeAdditive(const Skeleton& aSk, const Pose& aBase, const Pose& aTarget, bool aMeshSpace, Additive& aOut)
{
    aOut.n = aSk.n;
    aOut.meshSpace = aMeshSpace;
    Quat qa[kMaxBones], qx[kMaxBones];
    if (aMeshSpace)
    {
        MeshRotations(aSk, aBase, qa);
        MeshRotations(aSk, aTarget, qx);
    }
    else
        for (int i = 0; i < aSk.n; ++i)
        {
            qa[i] = aBase.b[i].r;
            qx[i] = aTarget.b[i].r;
        }
    for (int i = 0; i < aSk.n; ++i)
    {
        Xf& d = aOut.b[i];
        d.r = math::QuatNormalize(math::QuatMul(qx[i], math::QuatInv(qa[i])));
        d.t = aTarget.b[i].t - aBase.b[i].t;
        const Vec3 sa = aBase.b[i].s, sx = aTarget.b[i].s;
        d.s = Vec3{sx.x * SafeRecip(sa.x) - 1.0f, sx.y * SafeRecip(sa.y) - 1.0f, sx.z * SafeRecip(sa.z) - 1.0f};
    }
}

// ---- A_MS / A_LS (optional per-bone mask: null = all bones) ----------------------------------------------------
inline void ApplyAdditive(const Skeleton& aSk, Pose& aPose, const Additive& aAdd, float aAlpha,
                          const BoneWeights* aMask = nullptr)
{
    const float a = Clamp01(aAlpha);
    if (a <= kZeroWeight || aAdd.n != aSk.n)
        return;
    Quat q[kMaxBones];
    if (aAdd.meshSpace)
        MeshRotations(aSk, aPose, q);
    else
        for (int i = 0; i < aSk.n; ++i)
            q[i] = aPose.b[i].r;
    for (int i = 0; i < aSk.n; ++i)
    {
        const float w = aMask ? a * Clamp01(aMask->w[i]) : a;
        if (w <= kZeroWeight)
            continue;
        const Xf& d = aAdd.b[i];
        const Quat dq = w >= 1.0f - kZeroWeight ? d.r : ScaleFromIdentity(d.r, w);
        q[i] = math::QuatNormalize(math::QuatMul(dq, q[i]));
        Xf& o = aPose.b[i];
        o.t = o.t + d.t * w;
        o.s = Mul(o.s, Vec3{1.0f + w * d.s.x, 1.0f + w * d.s.y, 1.0f + w * d.s.z});
    }
    if (aAdd.meshSpace)
        LocalFromMeshRotations(aSk, q, aPose);
    else
        for (int i = 0; i < aSk.n; ++i)
            aPose.b[i].r = q[i];
}

// ---- TwoWayBlend ----------------------------------------------------------------------------------------------
inline Xf BlendXf(const Xf& a, const Xf& b, float w)
{
    return Xf{Nlerp(a.r, b.r, w), math::Lerp(a.t, b.t, w), math::Lerp(a.s, b.s, w)};
}
inline void Blend(const Pose& aA, const Pose& aB, float aAlpha, Pose& aOut)
{
    const float a = Clamp01(aAlpha);
    aOut.n = aA.n;
    if (a <= kZeroWeight)
    {
        if (&aOut != &aA)
            std::memcpy(aOut.b, aA.b, sizeof(Xf) * aA.n);
        return;
    }
    if (a >= 1.0f - kZeroWeight)
    {
        if (&aOut != &aB)
            std::memcpy(aOut.b, aB.b, sizeof(Xf) * aA.n);
        return;
    }
    for (int i = 0; i < aA.n; ++i)
        aOut.b[i] = BlendXf(aA.b[i], aB.b[i], a);
}

// ---- LayeredBoneBlend ------------------------------------------------------------------------------------------
// Branch filter weights (UE FBranchFilter, BlendDepth): max over the branches added to aW.
inline void AddBranch(const Skeleton& aSk, int aRoot, int aBlendDepth, BoneWeights& aW)
{
    if (aRoot < 0 || aRoot >= aSk.n)
        return;
    const float inc = aBlendDepth > 0 ? 1.0f / static_cast<float>(aBlendDepth) : 1.0f;
    for (int i = aRoot; i < aSk.n; ++i)
    {
        const int d = aSk.DepthBelow(aRoot, i);
        if (d < 0)
            continue;
        aW.w[i] = std::max(aW.w[i], Clamp01(inc * static_cast<float>(d + 1)));
    }
}
// Excludes a subtree (weight 0), used for "thigh subtrees with weight 0" in the Pelvis layer.
inline void ExcludeBranch(const Skeleton& aSk, int aRoot, BoneWeights& aW)
{
    if (aRoot < 0 || aRoot >= aSk.n)
        return;
    for (int i = aRoot; i < aSk.n; ++i)
        if (aSk.IsAncestorOrSelf(aRoot, i))
            aW.w[i] = 0.0f;
}

// aOut may alias aBase. aLayerWeight multiplies every bone weight.
inline void LayeredBlend(const Skeleton& aSk, const Pose& aBase, const Pose& aLayer, const BoneWeights& aW,
                         float aLayerWeight, bool aMeshSpaceRotation, Pose& aOut)
{
    const float lw = Clamp01(aLayerWeight);
    Quat qb[kMaxBones], ql[kMaxBones];
    if (aMeshSpaceRotation)
    {
        MeshRotations(aSk, aBase, qb);
        MeshRotations(aSk, aLayer, ql);
    }
    aOut.n = aSk.n;
    for (int i = 0; i < aSk.n; ++i)
    {
        const float w = Clamp01(aW.w[i]) * lw;
        const Xf base = aBase.b[i];
        const Xf& lay = aLayer.b[i];
        if (aMeshSpaceRotation)
        {
            qb[i] = w <= kZeroWeight ? qb[i] : (w >= 1.0f - kZeroWeight ? ql[i] : Nlerp(qb[i], ql[i], w));
            aOut.b[i].t = math::Lerp(base.t, lay.t, w);
            aOut.b[i].s = math::Lerp(base.s, lay.s, w);
        }
        else
            aOut.b[i] = w <= kZeroWeight ? base : (w >= 1.0f - kZeroWeight ? lay : BlendXf(base, lay, w));
    }
    if (aMeshSpaceRotation)
        LocalFromMeshRotations(aSk, qb, aOut);
}

// ---- blend of additive samples (blend space of additive poses: weighted shortest-path sum, normalized) ---------
inline void BlendAdditives(const Additive* const* aSamples, const float* aWeights, int aCount, Additive& aOut)
{
    if (aCount <= 0 || !aSamples[0])
        return;
    const int n = aSamples[0]->n;
    aOut.n = n;
    aOut.meshSpace = aSamples[0]->meshSpace;
    for (int i = 0; i < n; ++i)
    {
        Quat acc{0.0f, 0.0f, 0.0f, 0.0f};
        Vec3 t{}, s{};
        bool first = true;
        Quat ref{};
        for (int k = 0; k < aCount; ++k)
        {
            const float w = aWeights[k];
            if (!(w > kZeroWeight) || !aSamples[k])
                continue;
            const Xf& d = aSamples[k]->b[i];
            if (first)
            {
                ref = d.r;
                first = false;
            }
            const float sg = math::QuatDot(ref, d.r) >= 0.0f ? w : -w;
            acc = Quat{acc.x + d.r.x * sg, acc.y + d.r.y * sg, acc.z + d.r.z * sg, acc.w + d.r.w * sg};
            t = t + d.t * w;
            s = s + d.s * w;
        }
        aOut.b[i] = Xf{first ? Quat{} : math::QuatNormalize(acc), t, s};
    }
}

// Weighted blend of full local poses (crossfades of overlay / base poses): shortest-path sum, normalized.
inline void BlendPosesWeighted(const Pose* const* aPoses, const float* aWeights, int aCount, Pose& aOut)
{
    if (aCount <= 0 || !aPoses[0])
        return;
    const int n = aPoses[0]->n;
    aOut.n = n;
    for (int i = 0; i < n; ++i)
    {
        Quat acc{0.0f, 0.0f, 0.0f, 0.0f}, ref{};
        Vec3 t{}, s{};
        float wsum = 0.0f;
        bool first = true;
        for (int k = 0; k < aCount; ++k)
        {
            const float w = aWeights[k];
            if (!(w > kZeroWeight) || !aPoses[k])
                continue;
            const Xf& x = aPoses[k]->b[i];
            if (first)
            {
                ref = x.r;
                first = false;
            }
            const float sg = math::QuatDot(ref, x.r) >= 0.0f ? w : -w;
            acc = Quat{acc.x + x.r.x * sg, acc.y + x.r.y * sg, acc.z + x.r.z * sg, acc.w + x.r.w * sg};
            t = t + x.t * w;
            s = s + x.s * w;
            wsum += w;
        }
        const float inv = wsum > 0.0f ? 1.0f / wsum : 0.0f;
        aOut.b[i] = first ? aPoses[0]->b[i] : Xf{math::QuatNormalize(acc), t * inv, s * inv};
    }
}

// ---- blend-space sample weight smoothing (TargetWeightInterpolationSpeedPerSec) -------------------------------
// Constant-rate move of every sample weight towards its target (UE FInterpConstantTo path), then normalization.
// [W] UE's default bTargetWeightInterpolationEaseInOut variant is not reproduced; speed <= 0 = no smoothing.
template <int N>
struct WeightSmoother
{
    float w[N]{};
    bool init = false;

    void Update(const float* aTarget, float aSpeed, float aDt)
    {
        if (!init || aSpeed <= 0.0f || !(aDt > 0.0f))
        {
            for (int k = 0; k < N; ++k)
                w[k] = aTarget[k];
            init = true;
            return;
        }
        const float step = aSpeed * aDt;
        float sum = 0.0f;
        for (int k = 0; k < N; ++k)
        {
            const float d = aTarget[k] - w[k];
            w[k] += std::fabs(d) <= step ? d : (d > 0.0f ? step : -step);
            sum += w[k];
        }
        if (sum > 1e-6f)
            for (int k = 0; k < N; ++k)
                w[k] /= sum;
    }
};

// ---- lean (BS1D_Additive_Lean_Run: samples -1 Left, 0 Base (identity), +1 Right) -------------------------------
struct LeanParams
{
    float speedLo = 2.0f;     // m/s (UE 200 cm/s)
    float speedHi = 5.0f;     // m/s (UE 500 cm/s)
    float scaleLo = 0.5f;     // MapRange(Speed2D 200..500 -> 0.5..1), clamped
    float scaleHi = 1.0f;
    float weightSpeed = 2.0f; // sample weight interpolation per second
    float alphaBlendTime = 0.2f; // IsMoving() alpha blend in / out
};

inline float MapRangeClamped(float v, float a0, float a1, float b0, float b1)
{
    if (a1 == a0)
        return b0;
    const float u = Clamp01((v - a0) / (a1 - a0));
    return b0 + (b1 - b0) * u;
}

inline float LeanAmount(float aRelAccelLateral, float aSpeed2D, const LeanParams& aP = {})
{
    const float x = aRelAccelLateral * MapRangeClamped(aSpeed2D, aP.speedLo, aP.speedHi, aP.scaleLo, aP.scaleHi);
    return std::max(-1.0f, std::min(1.0f, x));
}

struct LeanState
{
    WeightSmoother<3> weights; // [Left, Base, Right]
    float alpha = 0.0f;

    // aLeft / aRight: mesh-space additives (samples relative to the Base frame). Applies to aPose.
    void Apply(const Skeleton& aSk, Pose& aPose, const Additive& aLeft, const Additive& aRight, float aLeanX,
               bool aIsMoving, float aDt, const LeanParams& aP = {})
    {
        const float x = std::max(-1.0f, std::min(1.0f, aLeanX));
        const float target[3] = {x < 0.0f ? -x : 0.0f, 1.0f - std::fabs(x), x > 0.0f ? x : 0.0f};
        weights.Update(target, aP.weightSpeed, aDt);
        const float goal = aIsMoving ? 1.0f : 0.0f;
        if (aP.alphaBlendTime > 0.0f && aDt > 0.0f)
        {
            const float step = aDt / aP.alphaBlendTime;
            alpha += std::max(-step, std::min(step, goal - alpha));
        }
        else
            alpha = goal;
        Additive base;
        base.SetIdentity(aSk.n, true);
        const Additive* s[3] = {&aLeft, &base, &aRight};
        Additive d;
        BlendAdditives(s, weights.w, 3, d);
        d.meshSpace = true;
        ApplyAdditive(aSk, aPose, d, alpha);
    }
};

// ---- aim sweep (30-frame AAT_RotationOffsetMeshSpace, reference frame in the middle) ---------------------------
// Normalized time u = MapRange(pitch -90..90 -> 1..0); frame position u * (count - 1), linear between frames.
inline float SweepTimeFromPitch(float aPitchDeg) { return MapRangeClamped(aPitchDeg, -90.0f, 90.0f, 1.0f, 0.0f); }

// ---- aim offset grid (BS_Neutral_AO_Stand: yaw x pitch grid of mesh-space additives) ---------------------------
inline constexpr int kAOMaxSamples = 32;

struct AimOffsetGrid
{
    int nYaw = 5, nPitch = 3;
    float yawMin = -90.0f, yawMax = 90.0f, pitchMin = -90.0f, pitchMax = 90.0f;
    const Additive* samples[kAOMaxSamples]{}; // index = pitchIdx * nYaw + yawIdx

    void TargetWeights(float aYawDeg, float aPitchDeg, float* aW) const
    {
        const int n = nYaw * nPitch;
        for (int k = 0; k < n; ++k)
            aW[k] = 0.0f;
        const float fy = Clamp01((aYawDeg - yawMin) / (yawMax - yawMin)) * static_cast<float>(nYaw - 1);
        const float fp = Clamp01((aPitchDeg - pitchMin) / (pitchMax - pitchMin)) * static_cast<float>(nPitch - 1);
        const int y0 = std::min(nYaw - 1, static_cast<int>(fy)), p0 = std::min(nPitch - 1, static_cast<int>(fp));
        const int y1 = std::min(nYaw - 1, y0 + 1), p1 = std::min(nPitch - 1, p0 + 1);
        const float ty = fy - static_cast<float>(y0), tp = fp - static_cast<float>(p0);
        aW[p0 * nYaw + y0] += (1.0f - ty) * (1.0f - tp);
        aW[p0 * nYaw + y1] += ty * (1.0f - tp);
        aW[p1 * nYaw + y0] += (1.0f - ty) * tp;
        aW[p1 * nYaw + y1] += ty * tp;
    }
};

struct AimOffsetState
{
    WeightSmoother<kAOMaxSamples> weights;      // all bones (1.5 /s in motion)
    WeightSmoother<kAOMaxSamples> fastWeights;  // per-bone override bones (neck 4 /s)
    BoneWeights fastMask;                       // 1 for override bones

    void Apply(const Skeleton& aSk, Pose& aPose, const AimOffsetGrid& aGrid, float aYawDeg, float aPitchDeg,
               float aAlpha, float aDt, float aSpeed = 1.5f, float aFastSpeed = 4.0f)
    {
        float tw[kAOMaxSamples]{};
        aGrid.TargetWeights(aYawDeg, aPitchDeg, tw);
        weights.Update(tw, aSpeed, aDt);
        fastWeights.Update(tw, aFastSpeed, aDt);
        const int n = aGrid.nYaw * aGrid.nPitch;
        Additive slow, fast;
        BlendAdditives(aGrid.samples, weights.w, n, slow);
        BlendAdditives(aGrid.samples, fastWeights.w, n, fast);
        if (fastMask.n == aSk.n)
            for (int i = 0; i < aSk.n; ++i)
                if (fastMask.w[i] > 0.5f)
                    slow.b[i] = fast.b[i];
        slow.meshSpace = true;
        ApplyAdditive(aSk, aPose, slow, aAlpha);
    }
};

// ---- spine rotation (branch B, Enable_SpineRotation): yaw / count on each listed bone, mesh space about +Z -----
inline void ApplySpineYaw(const Skeleton& aSk, Pose& aPose, const int* aBones, int aCount, float aYawRad,
                          float aClampRad = 0.5f * math::kPi)
{
    if (aCount <= 0)
        return;
    const float yaw = std::max(-aClampRad, std::min(aClampRad, aYawRad)) / static_cast<float>(aCount);
    const Quat rz{0.0f, 0.0f, std::sin(0.5f * yaw), std::cos(0.5f * yaw)};
    Quat ms[kMaxBones];
    for (int k = 0; k < aCount; ++k)
    {
        const int b = aBones[k];
        if (b < 0 || b >= aSk.n)
            continue;
        MeshRotations(aSk, aPose, ms);
        const int p = aSk.parent[b];
        const Quat newMs = math::QuatMul(rz, ms[b]);
        aPose.b[b].r = p < 0 ? math::QuatNormalize(newMs) : math::QuatNormalize(math::QuatMul(math::QuatInv(ms[p]), newMs));
    }
}

} // namespace utpp::mm::ov

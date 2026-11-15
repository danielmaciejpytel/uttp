// UTPP motion - dual wield left arm: the left arm takes the mirror image of the right arm across the body's mid plane.
// Header-only, no game types.
//
// With two weapons drawn the one-handed weapon pose holds the right weapon; the left arm of that pose is a free arm. This module
// replaces the left clavicle, upper arm, forearm, hand and the left weapon bone (WeaponLeft) with the reflection of their right
// counterparts in the chest frame (Spine3). The fingers and the other children of the left hand keep their own locals.
//
// Reflection: S = I - 2 n n^T with n the lateral axis of the chest, the plane passes through the midpoint of the two clavicle heads;
// both are measured once on the rest pose. A reflected frame S * R is not a rotation, so each bone pair carries the constant
// N = Rr_rest^T * S * Rl_rest (chest frame): the left target is S * Rr * N, the position mid + S * (pr - mid). For a rig whose rest
// pose is mirror-symmetric N is a signed permutation with determinant -1; anything else (no such bone, other parents, an asymmetric
// rest pose) leaves the mirror off (Valid() false, Apply does nothing).
// Apply works on game-local transforms in database bone order and edits only the left bones; weight 0 returns at once (bit-identical).
#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

#include "Inertializer.hpp"
#include "MMMath.hpp"

namespace utpp::mm::dualmirror
{
struct M3
{
    double m[3][3]{};
};
inline M3 Identity3()
{
    M3 r;
    r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
    return r;
}
inline M3 Mul(const M3& a, const M3& b)
{
    M3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}
inline M3 Transpose(const M3& a)
{
    M3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[j][i];
    return r;
}
struct V3d
{
    double x = 0.0, y = 0.0, z = 0.0;
};
inline V3d Apply3(const M3& a, V3d v)
{
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}
inline V3d Sub(V3d a, V3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3d Add(V3d a, V3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3d ToD(math::Vec3 v) { return {v.x, v.y, v.z}; }
inline math::Vec3 ToF(V3d v) { return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)}; }

inline M3 FromQuat(const math::Quat& aQ)
{
    const double x = aQ.x, y = aQ.y, z = aQ.z, w = aQ.w;
    const double n = x * x + y * y + z * z + w * w;
    const double s = n > 0.0 ? 2.0 / n : 0.0;
    M3 r;
    r.m[0][0] = 1.0 - s * (y * y + z * z);
    r.m[0][1] = s * (x * y - z * w);
    r.m[0][2] = s * (x * z + y * w);
    r.m[1][0] = s * (x * y + z * w);
    r.m[1][1] = 1.0 - s * (x * x + z * z);
    r.m[1][2] = s * (y * z - x * w);
    r.m[2][0] = s * (x * z - y * w);
    r.m[2][1] = s * (y * z + x * w);
    r.m[2][2] = 1.0 - s * (x * x + y * y);
    return r;
}
inline math::Quat ToQuat(const M3& a)
{
    const auto& m = a.m;
    const double tr = m[0][0] + m[1][1] + m[2][2];
    double x, y, z, w;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s, x = (m[2][1] - m[1][2]) / s, y = (m[0][2] - m[2][0]) / s, z = (m[1][0] - m[0][1]) / s;
    }
    else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        w = (m[2][1] - m[1][2]) / s, x = 0.25 * s, y = (m[0][1] + m[1][0]) / s, z = (m[0][2] + m[2][0]) / s;
    }
    else if (m[1][1] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        w = (m[0][2] - m[2][0]) / s, x = (m[0][1] + m[1][0]) / s, y = 0.25 * s, z = (m[1][2] + m[2][1]) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        w = (m[1][0] - m[0][1]) / s, x = (m[0][2] + m[2][0]) / s, y = (m[1][2] + m[2][1]) / s, z = 0.25 * s;
    }
    return math::QuatNormalize(math::Quat{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)});
}

// Rigid transform in double precision (chest frame).
struct Xd
{
    M3 r = Identity3();
    V3d t;
};
inline Xd Compose(const Xd& aParent, const Xd& aLocal) { return {Mul(aParent.r, aLocal.r), Add(Apply3(aParent.r, aLocal.t), aParent.t)}; }
inline Xd Inverse(const Xd& a)
{
    const M3 rt = Transpose(a.r);
    const V3d t = Apply3(rt, a.t);
    return {rt, {-t.x, -t.y, -t.z}};
}
inline Xd FromBone(const BoneTransform& b) { return {FromQuat(b.q), ToD(b.p)}; }

// Mirrored pairs, parent first. Index 0 is the clavicle (child of the chest); WeaponLeft / WeaponRight hang on the hands.
inline constexpr int kPairs = 5;
inline constexpr std::string_view kLeftNames[kPairs] = {"LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "WeaponLeft"};
inline constexpr std::string_view kRightNames[kPairs] = {"RightShoulder", "RightArm", "RightForeArm", "RightHand", "WeaponRight"};
inline constexpr std::string_view kChestName = "Spine3";
// Largest |N - round(N)| accepted as a mirror-symmetric rest pose (both shipped rigs: 6e-4).
inline constexpr double kMaxRestAsymmetry = 0.01;

struct Mirror
{
    bool ok = false;
    int32_t chest = -1;
    int32_t left[kPairs] = {-1, -1, -1, -1, -1};
    int32_t right[kPairs] = {-1, -1, -1, -1, -1};
    M3 s = Identity3();   // reflection across the mid plane (chest frame)
    V3d mid;              // a point of the mid plane (chest frame)
    V3d lateral;          // plane normal (chest frame), for diagnostics
    M3 n[kPairs];         // per pair: Rr_rest^T S Rl_rest
    double restAsymmetry = 0.0; // largest |N - round(N)| over the pairs (diagnostics, tests)
    bool Valid() const { return ok; }
};

// aRest: game-local rest transforms (database RIGR) in bone order; aParent(i) -> parent index (-1 root).
template<class NameFn, class ParentFn>
inline Mirror Build(uint32_t aCount, NameFn aName, ParentFn aParent, const BoneTransform* aRest)
{
    Mirror m;
    if (aRest == nullptr)
        return m;
    auto find = [&](std::string_view s) -> int32_t {
        for (uint32_t i = 0; i < aCount; ++i)
            if (std::string_view(aName(i)) == s)
                return static_cast<int32_t>(i);
        return -1;
    };
    m.chest = find(kChestName);
    if (m.chest < 0)
        return m;
    for (int k = 0; k < kPairs; ++k)
    {
        m.left[k] = find(kLeftNames[k]);
        m.right[k] = find(kRightNames[k]);
        if (m.left[k] < 0 || m.right[k] < 0)
            return m;
        const int32_t wantL = k == 0 ? m.chest : m.left[k - 1];
        const int32_t wantR = k == 0 ? m.chest : m.right[k - 1];
        if (static_cast<int32_t>(aParent(static_cast<uint32_t>(m.left[k]))) != wantL ||
            static_cast<int32_t>(aParent(static_cast<uint32_t>(m.right[k]))) != wantR)
            return m;
    }
    // rest chains in the chest frame
    Xd restL[kPairs], restR[kPairs];
    for (int k = 0; k < kPairs; ++k)
    {
        const Xd pl = k == 0 ? Xd{} : restL[k - 1];
        const Xd pr = k == 0 ? Xd{} : restR[k - 1];
        restL[k] = Compose(pl, FromBone(aRest[m.left[k]]));
        restR[k] = Compose(pr, FromBone(aRest[m.right[k]]));
    }
    const V3d d = Sub(restL[0].t, restR[0].t);
    const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (!(len > 1e-4))
        return m;
    m.lateral = {d.x / len, d.y / len, d.z / len};
    m.mid = {0.5 * (restL[0].t.x + restR[0].t.x), 0.5 * (restL[0].t.y + restR[0].t.y), 0.5 * (restL[0].t.z + restR[0].t.z)};
    const double nv[3] = {m.lateral.x, m.lateral.y, m.lateral.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m.s.m[i][j] = (i == j ? 1.0 : 0.0) - 2.0 * nv[i] * nv[j];
    double worst = 0.0;
    for (int k = 0; k < kPairs; ++k)
    {
        m.n[k] = Mul(Mul(Transpose(restR[k].r), m.s), restL[k].r);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                worst = std::fmax(worst, std::fabs(m.n[k].m[i][j] - std::round(m.n[k].m[i][j])));
    }
    m.restAsymmetry = worst;
    m.ok = worst <= kMaxRestAsymmetry;
    return m;
}

// Mirrored left locals (targets) of the pose, without blending. aOut[k] = local transform for m.left[k].
inline bool MirroredLeftLocals(const Mirror& aM, const BoneTransform* aPose, BoneTransform (&aOut)[kPairs])
{
    if (!aM.ok || aPose == nullptr)
        return false;
    Xd right = {}, leftParent = {};
    for (int k = 0; k < kPairs; ++k)
    {
        right = Compose(right, FromBone(aPose[aM.right[k]])); // chest frame (k = 0: parent = chest)
        Xd target;
        target.r = Mul(Mul(aM.s, right.r), aM.n[k]);
        target.t = Add(aM.mid, Apply3(aM.s, Sub(right.t, aM.mid)));
        const Xd local = Compose(Inverse(leftParent), target);
        aOut[k].q = ToQuat(local.r);
        aOut[k].p = ToF(local.t);
        if (!math::IsFinite(aOut[k].q) || !math::IsFinite(aOut[k].p))
            return false;
        leftParent = target;
    }
    return true;
}

// Blend the left chain towards the mirror of the right chain with weight aW (0..1). aW <= 0: the pose is not touched.
// Returns true when the pose was edited.
inline bool Apply(const Mirror& aM, BoneTransform* aPose, float aW)
{
    if (!(aW > 0.0f) || !aM.ok || aPose == nullptr)
        return false;
    BoneTransform target[kPairs];
    if (!MirroredLeftLocals(aM, aPose, target))
        return false;
    const float w = aW < 1.0f ? aW : 1.0f;
    for (int k = 0; k < kPairs; ++k)
    {
        BoneTransform& b = aPose[aM.left[k]];
        if (w >= 1.0f)
            b = target[k];
        else
        {
            b.q = math::QuatNlerpShortest(b.q, target[k].q, w);
            b.p = math::Lerp(b.p, target[k].p, w);
        }
    }
    return true;
}

// Blend time of the mirror weight (see Ramp) and of the return from the stance hold. 0.65 s: the left hand travels up to ~0.6 m
// between the free arm of the weapon pose and the mirror (ready pose), so the smoothstep peak stays near 1.5 m/s (<= 2.5 cm per
// frame at 60 Hz).
inline constexpr float kBlendSeconds = 0.65f;

// Stance hold. The aim / fire pose of the one-handed weapon is a side stance: its mirror image points the left weapon far away
// from the target. While the right arm is in that stance the left chain therefore keeps the mirrored locals of the last frame
// outside it (the ready pose) instead of the mirror of the current right arm. Entering the stance is continuous (the held locals
// are the previous frame's target, so the switch is immediate); leaving it returns to the live mirror over kBlendSeconds, while
// the right arm blends back to the ready pose; a stance that comes back during that return takes the hold again over
// kHoldInSeconds.
inline constexpr float kHoldInSeconds = 0.1f;
struct Held
{
    bool valid = false;
    float u = 0.0f; // 0 = the live mirror, 1 = the held locals
    BoneTransform local[kPairs];
    void Reset() { valid = false; u = 0.0f; }
};

// Like Apply, with the stance hold (aStance: the right arm is in the aim / fire stance this frame). aW <= 0: the pose is not
// touched and the hold is cleared. In the stance with nothing held yet (two weapons drawn while aiming) the pose is left as it is.
inline bool ApplyOrHold(const Mirror& aM, Held& aHeld, BoneTransform* aPose, float aW, bool aStance, float aDt)
{
    if (!(aW > 0.0f) || !aM.ok || aPose == nullptr)
    {
        aHeld.Reset();
        return false;
    }
    const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f;
    if (aStance && aHeld.valid)
        aHeld.u = aHeld.u > 0.0f ? std::fmin(1.0f, aHeld.u + dt / kHoldInSeconds) : 1.0f;
    else
        aHeld.u = std::fmax(0.0f, aHeld.u - dt / kBlendSeconds);
    BoneTransform target[kPairs];
    if (!MirroredLeftLocals(aM, aPose, target))
        return false;
    if (!aStance && !(aHeld.u > 0.0f))
    {
        for (int k = 0; k < kPairs; ++k)
            aHeld.local[k] = target[k];
        aHeld.valid = true;
    }
    else if (aHeld.valid)
    {
        const float s = aHeld.u * aHeld.u * (3.0f - 2.0f * aHeld.u);
        for (int k = 0; k < kPairs; ++k)
        {
            target[k].q = math::QuatNlerpShortest(target[k].q, aHeld.local[k].q, s);
            target[k].p = math::Lerp(target[k].p, aHeld.local[k].p, s);
        }
    }
    else
        return false;
    const float w = aW < 1.0f ? aW : 1.0f;
    for (int k = 0; k < kPairs; ++k)
    {
        BoneTransform& b = aPose[aM.left[k]];
        if (w >= 1.0f)
            b = target[k];
        else
        {
            b.q = math::QuatNlerpShortest(b.q, target[k].q, w);
            b.p = math::Lerp(b.p, target[k].p, w);
        }
    }
    return true;
}

// Weight of the mirror, stepped once per frame: rises / falls over kBlendSeconds (smoothstep shaped) while the condition holds /
// is gone. The shown weight multiplies the shaped ramp by the caller's other factors.
// aGap: the ramp was not stepped for a while (skipped frames, game pause). Still wanted: it keeps its value (no pop of the arm when
// the frames resume); no longer wanted: it is 0 at once (two weapons were put away during the gap, nothing to fade from).
struct Ramp
{
    float u = 0.0f;
    float Step(bool aOn, float aDt, bool aGap = false)
    {
        if (aGap && !aOn)
            u = 0.0f;
        const float step = (std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f) / kBlendSeconds;
        u = aOn ? std::fmin(1.0f, u + step) : std::fmax(0.0f, u - step);
        return Shaped();
    }
    float Shaped() const { return u * u * (3.0f - 2.0f * u); }
    void Reset() { u = 0.0f; }
};

// Ramp step with the stance rule: in the stance with nothing held (two weapons drawn while aiming, a reload that ended in the
// stance) the left arm is left alone and the ramp restarts from 0, so the mirror fades in after the stance instead of appearing
// at full weight.
inline float StepRamp(Ramp& aRamp, const Held& aHeld, bool aWanted, bool aStance, float aDt, bool aGap = false)
{
    if (aStance && !aHeld.valid)
    {
        aRamp.Reset();
        return 0.0f;
    }
    return aRamp.Step(aWanted, aDt, aGap);
}

// Shown weight: ramp x the right arm layer weight of the weapon pose (a right arm that follows locomotion is not copied) x the
// reload envelope complement (the reload keeps its own behaviour).
inline float ShownWeight(float aRamp, float aRightArmLayer, float aReloadUpper)
{
    auto c01 = [](float v) { return std::isfinite(v) ? (v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v)) : 0.0f; };
    return c01(aRamp) * c01(aRightArmLayer) * (1.0f - c01(aReloadUpper));
}
} // namespace utpp::mm::dualmirror

#pragma once
// Pistol / rifle reload on a slide frame: the MM torso is set upright (geometry only; the weight is SlideDeRoll in ReloadContactFade.hpp,
// the gate SlideTorsoReload / RifleSlideExitLatch there, the call site MotionDriver SlideReloadTorsoStepLocked).
// The rolled phase of a slide (Slide_FootOut_Into_*) rolls the MM shoulder line by 34-39 deg for the whole reload and leans the
// torso back; the native / authored reload arm locals are authored for an upright torso and threw the weapon above the head on the
// slide torso (recorded 21.10 03:37 k10/k11). Instead of keeping the MM arms (SlideRollHold) the torso is uprighted in two steps,
// both split evenly over Spine1, Spine2 and Spine3 (each bone gains a third of the model-space rotation, so the Spine3 frame - and
// with it both clavicle roots, the neck and the head, which are its children - turns by the full angle):
//   1. de-roll: the minimal model-space rotation that levels the shoulder line (axis = horizontal, perpendicular to the line,
//      angle = weight * its elevation): the roll left is (1 - weight) * elevation;
//   2. de-pitch: a rotation about the shoulder line itself (so the roll is unchanged) by weight * (lean - kDePitchTargetDeg), lean =
//      forward lean of Spine3 -> Neck against the vertical (+ = towards the facing up x line). Offline replay of the 2026-10-25
//      Nue reloads (tools/analysis/replay_slide_deroll.cpp): the levelled slide torso leans -2 deg (matrix #4) / +0.3 deg (rec4
//      #27), a standing reload +9.8 deg; roll alone left the authored weapon 10.6 cm ABOVE the head, roll + pitch 5.5 cm below.
// Hips, Spine, the legs and every bone outside Spine1..Spine3 keep their local transforms.
// Header-only, allocation-free, no game types. Unit test: tests/ReloadSlideRollHoldTest.cpp.
#include <algorithm>
#include <cmath>

#include "Additive.hpp"

namespace utpp::mm::slide_deroll
{
using math::Quat;
using math::Vec3;

// Spine3 -> Neck forward lean of the standing Nue reload (matrix recording 2026-10-25 14:47, episode #0 stand, mean over the
// reload; replay_slide_deroll.cpp "levelled torso lean").
inline constexpr float kDePitchTargetDeg = 9.8f;
inline constexpr float kDePitchMaxDeg = 30.0f; // bound of the pitch correction (the slide range is -11..+10 deg)
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION == 5
inline constexpr bool kDePitchEnabled = false; // mutation 5: roll only, no de-pitch
#else
inline constexpr bool kDePitchEnabled = true;
#endif

struct Bones
{
    int spine[3] = {-1, -1, -1}; // Spine1, Spine2, Spine3 (ov indices)
    int lsh = -1, rsh = -1;      // LeftShoulder, RightShoulder (parents of LeftArm / RightArm)
    int neck = -1;               // Neck (lean reference; optional: without it there is no de-pitch)
    // Chain Spine1 -> Spine2 -> Spine3, both clavicle roots children of Spine3.
    bool Valid(const ov::Skeleton& aSk) const
    {
        auto in = [&](int b) { return b >= 0 && b < aSk.n; };
        return aSk.Valid() && in(spine[0]) && in(spine[1]) && in(spine[2]) && in(lsh) && in(rsh) && lsh != rsh &&
               aSk.parent[spine[1]] == spine[0] && aSk.parent[spine[2]] == spine[1] && aSk.parent[lsh] == spine[2] &&
               aSk.parent[rsh] == spine[2];
    }
    bool NeckValid(const ov::Skeleton& aSk) const { return neck >= 0 && neck < aSk.n && aSk.parent[neck] == spine[2]; }
};

// Model transform of aBone (rotation, position) by the local chain from the root (scale ignored, as gik::ModelOf).
inline void ModelOf(const ov::Skeleton& aSk, const ov::Pose& aPose, int aBone, Quat& aRot, Vec3& aPos)
{
    int chain[ov::kMaxBones];
    int n = 0;
    for (int b = aBone; b >= 0 && n < ov::kMaxBones; b = aSk.parent[b])
        chain[n++] = b;
    aRot = Quat{};
    aPos = Vec3{};
    for (int k = n - 1; k >= 0; --k)
    {
        const ov::Xf& x = aPose.b[chain[k]];
        aPos = aPos + math::QuatRotate(aRot, x.t);
        aRot = math::QuatNormalize(math::QuatMul(aRot, x.r));
    }
}

// Left -> right clavicle-root line in model space.
inline bool ShoulderLine(const ov::Skeleton& aSk, const ov::Pose& aPose, const Bones& aB, Vec3& aL, Vec3& aR)
{
    if (!aB.Valid(aSk) || aPose.n != aSk.n)
        return false;
    Quat q;
    ModelOf(aSk, aPose, aB.lsh, q, aL);
    ModelOf(aSk, aPose, aB.rsh, q, aR);
    return math::IsFinite(aL) && math::IsFinite(aR);
}

// Elevation of the shoulder line, 0..90 deg (the same proxy as reload_contacts_runtime::ShoulderLineRollDeg; 0 when invalid).
inline float RollDeg(const ov::Skeleton& aSk, const ov::Pose& aPose, const Bones& aB)
{
    Vec3 l, r;
    if (!ShoulderLine(aSk, aPose, aB, l, r))
        return 0.0f;
    const Vec3 d = r - l;
    const float len = std::sqrt(math::Dot(d, d));
    if (!std::isfinite(len) || len < 1.0e-4f)
        return 0.0f;
    return std::asin(std::clamp(std::fabs(d.z) / len, 0.0f, 1.0f)) * 57.29578f;
}

// Forward lean of Spine3 -> Neck against the vertical, deg (+ = towards the facing, facing = up x shoulder line); false when the
// neck is missing or the geometry is degenerate.
inline bool LeanDeg(const ov::Skeleton& aSk, const ov::Pose& aPose, const Bones& aB, float& aOut)
{
    Vec3 l, r;
    if (!aB.NeckValid(aSk) || !ShoulderLine(aSk, aPose, aB, l, r))
        return false;
    const Vec3 fw = math::Cross(Vec3{0.0f, 0.0f, 1.0f}, r - l);
    const float fl = std::sqrt(math::Dot(fw, fw));
    Quat q;
    Vec3 s3, nk;
    ModelOf(aSk, aPose, aB.spine[2], q, s3);
    ModelOf(aSk, aPose, aB.neck, q, nk);
    const Vec3 d = nk - s3;
    if (!(fl > 1.0e-6f) || !math::IsFinite(d) || !(math::Dot(d, d) > 1.0e-10f))
        return false;
    aOut = std::atan2(math::Dot(d, fw * (1.0f / fl)), d.z) * 57.29578f;
    return std::isfinite(aOut);
}

// Adds the model-space rotation q a third at a time to Spine1, Spine2, Spine3 (local' = P^-1 * q * P * local with the updated
// parent); restores the three locals and returns false on a non-finite result.
inline bool RotateChain(const ov::Skeleton& aSk, ov::Pose& aPose, const Bones& aB, Vec3 aAxisUnit, float aAngleRad)
{
    const float h = 0.5f * aAngleRad / 3.0f, s = std::sin(h);
    const Quat q{aAxisUnit.x * s, aAxisUnit.y * s, aAxisUnit.z * s, std::cos(h)};
    ov::Xf saved[3];
    for (int k = 0; k < 3; ++k)
        saved[k] = aPose.b[aB.spine[k]];
    for (int k = 0; k < 3; ++k)
    {
        const int b = aB.spine[k];
        Quat parent;
        Vec3 unused;
        ModelOf(aSk, aPose, aSk.parent[b], parent, unused);
        const Quat n = math::QuatNormalize(
            math::QuatMul(math::QuatMul(math::QuatMul(math::QuatInv(parent), q), parent), aPose.b[b].r));
        if (!math::IsFinite(n))
        {
            for (int j = 0; j < 3; ++j)
                aPose.b[aB.spine[j]] = saved[j];
            return false;
        }
        aPose.b[b].r = n;
    }
    return true;
}

struct Result
{
    float rollDeg = 0.0f;  // roll removed
    float pitchDeg = 0.0f; // lean removed (+ = the torso was turned backwards)
    bool Changed() const { return rollDeg != 0.0f || pitchDeg != 0.0f; }
};

// Uprights Spine1..Spine3 by aWeight (0..1), in place: de-roll, then de-pitch about the (partly) levelled shoulder line.
// Weight 0, an invalid chain or a degenerate / vertical line: no edit (Result all 0, pose bit-identical).
inline Result Apply(const ov::Skeleton& aSk, ov::Pose& aPose, const Bones& aB, float aWeight)
{
    Result res;
    if (!std::isfinite(aWeight) || !(aWeight > 0.0f))
        return res;
    const float w = std::min(aWeight, 1.0f);
    Vec3 l, r;
    if (!ShoulderLine(aSk, aPose, aB, l, r))
        return res;
    const Vec3 d = r - l;
    const Vec3 dh{d.x, d.y, 0.0f};
    const float len = std::sqrt(math::Dot(d, d)), lenH = std::sqrt(math::Dot(dh, dh));
    if (!std::isfinite(len) || len < 1.0e-4f || !(lenH > 1.0e-4f * len))
        return res;
    // 1. de-roll: rotating d about d x dh by +elev takes it onto its horizontal projection
    const float elev = std::asin(std::clamp(std::fabs(d.z) / len, 0.0f, 1.0f));
    Vec3 axis = math::Cross(d, dh);
    const float axisLen = std::sqrt(math::Dot(axis, axis));
    if (elev > 1.0e-6f && axisLen > 1.0e-9f && std::isfinite(axisLen))
    {
        if (!RotateChain(aSk, aPose, aB, axis * (1.0f / axisLen), w * elev))
            return res;
        res.rollDeg = w * elev * 57.29578f;
    }
    // 2. de-pitch about the shoulder line (its direction is the axis: the roll left does not change)
    if constexpr (kDePitchEnabled)
    {
        float lean = 0.0f;
        if (ShoulderLine(aSk, aPose, aB, l, r) && LeanDeg(aSk, aPose, aB, lean))
        {
            const Vec3 a = r - l;
            const float al = std::sqrt(math::Dot(a, a));
            const float corr = w * std::clamp(lean - kDePitchTargetDeg, -kDePitchMaxDeg, kDePitchMaxDeg);
            if (al > 1.0e-6f && corr != 0.0f && RotateChain(aSk, aPose, aB, a * (1.0f / al), corr * 0.01745329f))
                res.pitchDeg = corr;
        }
    }
    return res;
}
} // namespace utpp::mm::slide_deroll

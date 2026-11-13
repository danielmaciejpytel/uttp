// UTPP motion matching runtime - analytic two-bone leg IK (loco batch 1, 2026-10-13). Header-only, C++20, deterministic,
// allocation-free. Extracted bit-exactly from FootLocker::Solve (Phase 4.5) so the foot locker and the later ground foot placement
// (crouch C2, motion stage 10.5) share one solver; the arithmetic and its order are unchanged (utpp_mm_footlock_test output
// byte-identical before / after the extraction).
//
// Model space (character frame: x right, y forward, z up, metres). Inputs: model-space transforms of the UpLeg's parent (gp),
// UpLeg (ga), Leg (gb) and Foot (gc), the ankle target t and the animated lock point (toe) aToe (only its xy offset from the ankle
// is used: foot-forward pole bias). The knee is placed by the law of cosines on the pole side of the hip-target line, thigh and shin
// are rotated by minimal swings; the foot keeps its ANIMATED model-space rotation (heel / toe roll of the clip preserved). Reach is
// limited softly from softExtension to maxExtension of the leg length, never below the animated hip-ankle distance. Pole = the
// animated knee offset from the hip-target line, with a foot-forward bias (kPoleBias) that fades in only for an almost straight leg
// (offset < kPoleBlend), so the bend plane stays continuous and the knee never flips backwards.
#pragma once

#include <cmath>
#include <cstdint>

#include "Inertializer.hpp"
#include "MMMath.hpp"

namespace utpp::mm::legik
{
inline constexpr float kPoleBias = 0.05f;  // m, foot-forward knee bias for an almost straight leg
inline constexpr float kPoleBlend = 0.05f; // m, animated knee offset above which the bias is off

enum class TwoBoneStatus : uint8_t
{
    kBadLegLength = 0, // thigh or shin shorter than 0.1 mm: nothing evaluated (clamped not set)
    kBadTarget = 1,    // target on the hip (|t - hip| <= 0.1 mm): reach evaluated (clamped valid), no result
    kNonFinite = 2,    // non-finite result: reach evaluated, no result (the caller resets its state)
    kOk = 3,           // local[] / ankle valid
};

struct TwoBoneSolution
{
    TwoBoneStatus status = TwoBoneStatus::kBadLegLength;
    bool clamped = false;       // |t - hip| beyond the reach (status != kBadLegLength)
    math::Quat local[3];        // new LOCAL rotations of UpLeg, Leg, Foot (status kOk)
    math::Vec3 ankle{};         // resulting model-space ankle (status kOk)
};

inline float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
inline math::Vec3 Norm(math::Vec3 v)
{
    const float l = Len(v);
    return l > 1e-12f ? v * (1.0f / l) : math::Vec3{};
}
inline math::Quat AxisAngle(math::Vec3 aUnitAxis, float aAngle)
{
    const float h = 0.5f * aAngle;
    const float sh = std::sin(h);
    return math::Quat{aUnitAxis.x * sh, aUnitAxis.y * sh, aUnitAxis.z * sh, std::cos(h)};
}
// Minimal rotation taking direction u onto direction v.
inline math::Quat FromTo(math::Vec3 u, math::Vec3 v)
{
    const math::Vec3 un = Norm(u), vn = Norm(v);
    const float dot = math::Dot(un, vn);
    if (dot < -0.9999f)
    {
        math::Vec3 ax = math::Cross(un, math::Vec3{1.0f, 0.0f, 0.0f});
        if (Len(ax) < 1e-3f)
            ax = math::Cross(un, math::Vec3{0.0f, 1.0f, 0.0f});
        return AxisAngle(Norm(ax), math::kPi);
    }
    const math::Vec3 cr = math::Cross(un, vn);
    return math::QuatNormalize(math::Quat{cr.x, cr.y, cr.z, 1.0f + dot});
}

// Two-bone IK in model space (see the header). aMaxExtension / aSoftExtension: fractions of (thigh + shin).
// aPoleCarry (batch 2 knee fix, UTPPSetMMKneeFix bit 2; false = the loco batch 1 solver bit for bit): the pole is the animated knee
// offset from the ANIMATED hip-ankle axis, carried onto the new hip-target axis by the minimal rotation between the two axes,
// instead of the animated knee projected onto the new line. The projection is ill-conditioned for an almost straight leg whose lock
// target is a few cm sideways of the animated ankle (the projected offset is mostly the lateral lock offset itself: the knee swung
// sideways frame to frame on standing turn starts); the carried offset keeps the animated bend plane. Foot-forward bias unchanged.
inline TwoBoneSolution SolveTwoBone(const BoneTransform& gp, const BoneTransform& ga, const BoneTransform& gb, const BoneTransform& gc,
                                    math::Vec3 t, math::Vec3 aToe, float aMaxExtension, float aSoftExtension, bool aPoleCarry = false,
                                    bool aPreserveAnimatedPole = false)
{
    TwoBoneSolution r;
    const math::Vec3 a = ga.p, b = gb.p, c = gc.p;
    const float lab = Len(b - a), lcb = Len(c - b);
    if (!(lab > 1e-4f) || !(lcb > 1e-4f))
        return r; // kBadLegLength
    // Soft reach limit: identity below softStart, then exponential approach to maxExtension (the knee angle is very
    // sensitive to the length near full extension; a hard clamp makes the leg snap straight).
    // Never shorten the animated leg: limits start at the animated hip-ankle distance at the earliest.
    const float lac = Len(c - a);
    const float reach = std::fmax((lab + lcb) * aMaxExtension, lac);
    const float softStart = std::fmin(std::fmax((lab + lcb) * aSoftExtension, lac), reach);
    const float lt = Len(t - a);
    r.clamped = lt > reach;
    float lat = std::fmax(lt, 1e-4f);
    if (lat > softStart && reach > softStart)
        lat = softStart + (reach - softStart) * (1.0f - std::exp(-(lat - softStart) / (reach - softStart)));
    lat = std::fmin(lat, reach);
    // Analytic placement: knee on the pole side of the hip-target line (law of cosines), then minimal swings of
    // thigh and shin onto the new segments. Pole = animated knee offset from the line; when the leg is almost
    // straight (offset < kPoleBlend) a foot-forward bias (toe - ankle) takes over, so the bend plane stays
    // continuous and the knee never flips backwards.
    const math::Vec3 atv = t - a;
    if (!(Len(atv) > 1e-4f))
    {
        r.status = TwoBoneStatus::kBadTarget;
        return r;
    }
    const math::Vec3 atn = Norm(atv);
    auto perp = [&](math::Vec3 v) { return v - atn * math::Dot(v, atn); };
    math::Vec3 kneeOff = perp(b - a);
    if (aPoleCarry)
    {
        const math::Vec3 acv = c - a;
        if (Len(acv) > 1e-4f)
        {
            const math::Vec3 acn = Norm(acv);
            const math::Vec3 off0 = (b - a) - acn * math::Dot(b - a, acn); // animated knee offset from the animated axis
            kneeOff = perp(math::QuatRotate(FromTo(acn, atn), off0));
        }
    }
    const float kneeLen = Len(kneeOff);
    math::Vec3 fwd = aToe - c;
    fwd.z = 0.0f;
    if (!(Len(fwd) > 1e-4f))
        fwd = math::Vec3{0.0f, 1.0f, 0.0f};
    // A relative orientation-warp correction must approach the animated pose
    // continuously as its target delta tends to zero. The usual near-straight
    // foot-lock bias changes a defined bend plane even at an identity target.
    // Opt-in callers preserve that carried plane; truly straight/degenerate
    // knees still use the deterministic foot-forward fallback. Other callers
    // retain the original bias unchanged.
    const float bias = aPreserveAnimatedPole && kneeLen > 1e-6f
                           ? 0.0f
                           : kPoleBias * std::fmax(0.0f, 1.0f - kneeLen / kPoleBlend);
    math::Vec3 pole = kneeOff + perp(Norm(fwd)) * bias;
    if (Len(pole) < 1e-6f)
        pole = perp(math::Vec3{0.0f, 1.0f, 0.0f});
    if (Len(pole) < 1e-6f)
        pole = perp(math::Vec3{1.0f, 0.0f, 0.0f});
    const math::Vec3 pn = Norm(pole);
    const float cosA = std::fmin(1.0f, std::fmax(-1.0f, (lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat)));
    const float sinA = std::sqrt(std::fmax(0.0f, 1.0f - cosA * cosA));
    const math::Vec3 nb = a + atn * (cosA * lab) + pn * (sinA * lab);
    const math::Vec3 ncT = a + atn * lat;
    const math::Quat ra = FromTo(b - a, nb - a);
    const math::Quat rb = FromTo(math::QuatRotate(ra, c - b), ncT - nb);
    const math::Quat qa = math::QuatNormalize(math::QuatMul(ra, ga.q));
    const math::Quat qb = math::QuatNormalize(math::QuatMul(rb, math::QuatMul(ra, gb.q)));
    const math::Vec3 nc = nb + math::QuatRotate(math::QuatMul(rb, ra), c - b);
    const math::Quat la = math::QuatNormalize(math::QuatMul(math::QuatInv(gp.q), qa));
    const math::Quat lb = math::QuatNormalize(math::QuatMul(math::QuatInv(qa), qb));
    const math::Quat lc = math::QuatNormalize(math::QuatMul(math::QuatInv(qb), gc.q));
    if (!math::IsFinite(la) || !math::IsFinite(lb) || !math::IsFinite(lc) || !math::IsFinite(nc))
    {
        r.status = TwoBoneStatus::kNonFinite;
        return r;
    }
    r.status = TwoBoneStatus::kOk;
    r.local[0] = la;
    r.local[1] = lb;
    r.local[2] = lc;
    r.ankle = nc;
    return r;
}
} // namespace utpp::mm::legik

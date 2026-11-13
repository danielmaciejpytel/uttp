// UTPP strafe S2 - chest align (b1 strafe design, approach point 8, with the critique MUST changes). Header-only, C++20,
// deterministic, no allocations in Apply() after Init().
//
// Problem: the motion facing-fixed side / back loops twist the chest about the vertical axis (v6: mean -36..+23 deg per
// loop, swing +-10..21 deg relative to the forward walk). While aiming, the game owns Spine1+ (upper-body mask), so that
// twist rotates the whole aimed upper body and saturates the AimOffset 30 deg shoulder clamp.
//
// Reference (per stance, MUST change: stand AND crouch): sign-aligned quaternion mean of the MODEL rotation of Spine
// (= Hips_model * Spine_local, Spine is a child of Hips) over the stance's forward walk loop, built at init
// (BuildReference). dev = twist about +Z of (Spine_model * inv(ref[stance])), i.e. the yaw of the chest relative to its
// forward-walk orientation, measured on the pose AFTER OffsetRoot / StanceGuard (pipeline order OffsetRoot ->
// StanceGuard -> ChestAlign -> FootLocker -> publish).
//
// Per update:  devLow  = low-pass(dev, devLowHalflife)
//              target  = -clamp(devLow + (1 - swayKeep) * (dev - devLow), +-chestMaxDeg)
//              delta   = low-pass(target, smoothHalflife)
//              Spine_local' = (Hips_model^-1 * Rz(delta * w) * Hips_model) * Spine_local
// so Spine_model' = Rz(delta * w) * Spine_model (rotation about the Spine joint, world vertical axis); the twist about Z
// is exactly additive under that pre-multiplication, so swayKeep 0 cancels the twist, 1 keeps the sway and removes only
// the slow mean. Only the Spine local rotation is written: Hips, legs and every other bone stay bit-identical; weight 0
// (or a non-finite input / missing reference) leaves the pose bit-identical. Filters keep running at weight 0 so a fade-in
// starts from the settled correction.
//
// Driver weight w = strafe/aim blend x g_maskBlend.Weight(). NOTE (MUST change): g_maskBlend is advanced in the publish
// step after the pose pipeline, so the weight seen here is the PREVIOUS frame's mask blend (one-frame lag). Harmless for
// the smooth mask fades; it means the first frame of a mask fade-in applies 0 and the first frame of a fade-out still
// applies the previous weight to a pose whose Spine1+ is about to come back from MM.
//
// Stance switch (stand <-> crouch; chest_repair 2026-10-13): the driver's stance flips on the INPUT frame, but the matcher
// pose only crosses from the stand to the crouch loops over several frames (v6: the two references differ by -13.5 deg of
// twist). Measuring against the new reference at once yawed the chest by that offset ahead of the pose and then drifted
// back as devLow followed (a ~13 deg transient swing on every crouch toggle while aiming). Now the reference is blended:
// dev = devStand + c * wrap(devCrouch - devStand), c = crouch fraction of the reference, moved monotonically towards the
// requested stance: c follows the pose (Hips model height between the two references' mean Hips heights) and is never
// slower than a linear ramp over stanceBlendTime (so a pose that never crouches still completes the switch). devLow is
// NOT re-initialised (dev is continuous). Without a switch c equals the stance (0 / 1) and dev is the single-reference
// Deviation() exactly. stanceBlendTime 0 (or the other stance without a reference) = the old switch: c snaps and devLow
// re-initialises to the new stance's dev.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "Inertializer.hpp"
#include "MMMath.hpp"

namespace utpp::mm
{
struct ChestAlignConfig
{
    float swayKeep = 0.5f;        // fraction of the twist around its low-passed mean that is kept (0 = cancel all)
    float chestMaxDeg = 60.0f;    // deg, largest correction
    float devLowHalflife = 0.3f;  // s, low-pass of dev (mean twist of the loop)
    float smoothHalflife = 0.05f; // s, smoothing of the applied correction
    float stanceBlendTime = 0.8f; // s, longest stand <-> crouch reference blend (pose-driven, this ramp is the floor); 0 = snap
};

struct ChestAlignDiagnostics
{
    float dev = 0.0f;   // deg, twist of the chest vs the stance reference (input pose)
    float low = 0.0f;   // deg, low-passed dev
    float delta = 0.0f; // deg, applied correction (already multiplied by the weight)
    float weight = 0.0f;
    float blend = 0.0f; // crouch fraction of the reference (0 stand, 1 crouch; between = stance switch in progress)
    int stance = 0;
    bool valid = false; // dev / low are from this update (false: no reference / non-finite pose, dev = low = 0)
    uint64_t updates = 0, activeUpdates = 0, nonFinite = 0, noReference = 0;
    uint64_t blendUpdates = 0; // updates with a stance-switch reference blend in progress (0 < c < 1 after the update or before)
};

class ChestAlign
{
public:
    static constexpr int kStand = 0;
    static constexpr int kCrouch = 1;
    static constexpr int kNumStances = 2;

    // Resolves Hips / Spine by name (CP rig). Spine must be a direct child of Hips.
    bool Init(const Database& aDb, const ChestAlignConfig& aCfg = {})
    {
        int32_t hips = -1, spine = -1;
        for (uint32_t b = 0; b < aDb.NumBones(); ++b)
        {
            if (aDb.BoneName(b) == "Hips")
                hips = static_cast<int32_t>(b);
            else if (aDb.BoneName(b) == "Spine")
                spine = static_cast<int32_t>(b);
        }
        return Init(aDb, hips, spine, aCfg);
    }
    bool Init(const Database& aDb, int32_t aHips, int32_t aSpine, const ChestAlignConfig& aCfg = {})
    {
        m_ok = false;
        m_refOk[0] = m_refOk[1] = false;
        const uint32_t nb = aDb.NumBones();
        if (!aDb.Loaded() || !Valid(aCfg) || nb == 0 || aHips < 0 || aSpine < 0 || uint32_t(aHips) >= nb || uint32_t(aSpine) >= nb)
            return false;
        m_parents.assign(nb, -1);
        for (uint32_t b = 0; b < nb; ++b)
        {
            const int32_t p = aDb.BoneParent(b);
            if (p >= static_cast<int32_t>(b))
                return false;
            m_parents[b] = p;
        }
        if (m_parents[size_t(aSpine)] != aHips)
            return false;
        m_hips = aHips;
        m_spine = aSpine;
        m_cfg = aCfg;
        m_scratch.assign(nb, BoneTransform{});
        m_ok = true;
        Reset();
        return true;
    }
    bool Ready() const { return m_ok; }
    int32_t HipsBone() const { return m_hips; }
    int32_t SpineBone() const { return m_spine; }

    static bool Valid(const ChestAlignConfig& c)
    {
        return std::isfinite(c.swayKeep) && c.swayKeep >= 0.0f && c.swayKeep <= 1.0f && std::isfinite(c.chestMaxDeg) &&
               c.chestMaxDeg >= 0.0f && c.chestMaxDeg <= 90.0f && std::isfinite(c.devLowHalflife) && c.devLowHalflife > 0.0f &&
               c.devLowHalflife <= 10.0f && std::isfinite(c.smoothHalflife) && c.smoothHalflife >= 0.0f && c.smoothHalflife <= 2.0f &&
               std::isfinite(c.stanceBlendTime) && c.stanceBlendTime >= 0.0f && c.stanceBlendTime <= 5.0f;
    }
    bool SetConfig(const ChestAlignConfig& c)
    {
        if (!Valid(c))
            return false;
        m_cfg = c;
        return true;
    }
    const ChestAlignConfig& Config() const { return m_cfg; }
    const ChestAlignDiagnostics& Diagnostics() const { return m_diag; }

    void Reset()
    {
        m_init = false;
        m_low = m_delta = 0.0f;
        m_c = 0.0f;
        m_lastStance = -1;
    }

    // Finds a clip by exact name, -1 if absent.
    static int32_t FindClip(const Database& aDb, std::string_view aName)
    {
        for (uint32_t c = 0; c < aDb.NumClips(); ++c)
            if (aDb.ClipName(c) == aName)
                return static_cast<int32_t>(c);
        return -1;
    }

    // Reference of a stance from the frames [aFirst, aFirst + aCount) (database pose = local transforms). Init time only
    // (uses the scratch pose, no allocation). Returns false (reference left invalid) for an empty / non-finite range.
    bool BuildReference(const Database& aDb, int aStance, uint32_t aFirst, uint32_t aCount)
    {
        if (!m_ok || aStance < 0 || aStance >= kNumStances || aDb.NumBones() != m_scratch.size() || aCount == 0 ||
            uint64_t(aFirst) + aCount > aDb.NumFrames())
            return false;
        m_refOk[aStance] = false;
        double sx = 0.0, sy = 0.0, sz = 0.0, sw = 0.0, hz = 0.0;
        math::Quat first{};
        for (uint32_t f = aFirst; f < aFirst + aCount; ++f)
        {
            const std::span<const float> p = aDb.Pose(f);
            if (p.size() != m_scratch.size() * 7)
                return false;
            for (size_t b = 0; b < m_scratch.size(); ++b)
            {
                m_scratch[b].p = {p[b * 7 + 0], p[b * 7 + 1], p[b * 7 + 2]};
                m_scratch[b].q = {p[b * 7 + 3], p[b * 7 + 4], p[b * 7 + 5], p[b * 7 + 6]};
            }
            math::Quat q = math::QuatNormalize(Model(m_scratch, m_spine).q);
            if (f == aFirst)
                first = q;
            else if (math::QuatDot(q, first) < 0.0f)
                q = math::QuatNeg(q);
            sx += q.x;
            sy += q.y;
            sz += q.z;
            sw += q.w;
            hz += Model(m_scratch, m_hips).p.z;
        }
        const math::Quat mean = math::QuatNormalize(math::Quat{float(sx), float(sy), float(sz), float(sw)});
        const float hipsZ = float(hz / double(aCount));
        if (!math::IsFinite(mean) || !(sx * sx + sy * sy + sz * sz + sw * sw > 1e-12) || !std::isfinite(hipsZ))
            return false;
        m_ref[aStance] = mean;
        m_hipsZ[aStance] = hipsZ;
        m_refOk[aStance] = true;
        return true;
    }
    bool HasReference(int aStance) const { return aStance >= 0 && aStance < kNumStances && m_refOk[aStance]; }
    math::Quat Reference(int aStance) const { return HasReference(aStance) ? m_ref[aStance] : math::Quat{}; }
    // mean Hips model height (m) of the stance's reference loop (stance-switch blend), 0 without a reference
    float ReferenceHipsZ(int aStance) const { return HasReference(aStance) ? m_hipsZ[aStance] : 0.0f; }

    // Crouch fraction of the pose from its Hips model height between the two references' mean heights, clamped to [0,1];
    // NaN when unavailable (a reference missing, heights closer than 5 cm, non-finite pose). Pure.
    float PoseCrouchFraction(std::span<const BoneTransform> aPose) const
    {
        if (!m_ok || !HasReference(kStand) || !HasReference(kCrouch) || aPose.size() != m_parents.size())
            return std::numeric_limits<float>::quiet_NaN();
        const float span = m_hipsZ[kStand] - m_hipsZ[kCrouch];
        const float z = Model(aPose, m_hips).p.z;
        if (!(std::fabs(span) >= 0.05f) || !std::isfinite(z))
            return std::numeric_limits<float>::quiet_NaN();
        return std::fmin(std::fmax((m_hipsZ[kStand] - z) / span, 0.0f), 1.0f);
    }

    // Twist (rad, (-pi, pi]) about +Z of the Spine model rotation of aPose relative to the stance reference; NaN when
    // not available. Pure, no state change.
    float Deviation(std::span<const BoneTransform> aPose, int aStance) const
    {
        if (!m_ok || !HasReference(aStance) || aPose.size() != m_parents.size())
            return std::numeric_limits<float>::quiet_NaN();
        const math::Quat q = Model(aPose, m_spine).q;
        if (!math::IsFinite(q))
            return std::numeric_limits<float>::quiet_NaN();
        return TwistZ(math::QuatMul(math::QuatNormalize(q), math::QuatInv(m_ref[aStance])));
    }

    // In place on the local pose (NumBones transforms, Root/Trajectory as published: model space == character frame).
    // aWeight in [0,1] (clamped; non-finite = 0), aDt real seconds (non-finite / <= 0 = filters hold).
    void Apply(std::span<BoneTransform> aPose, int aStance, float aWeight, float aDt)
    {
        if (!m_ok || aPose.size() != m_parents.size())
            return;
        ++m_diag.updates;
        m_diag.weight = 0.0f;
        m_diag.delta = 0.0f;
        m_diag.dev = 0.0f;
        m_diag.low = 0.0f;
        m_diag.blend = 0.0f;
        m_diag.valid = false;
        m_diag.stance = aStance;
        if (!HasReference(aStance))
        {
            ++m_diag.noReference;
            return;
        }
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 1.0f) : 0.0f;
        // stance-switch reference blend (see the header): c moves monotonically towards the requested stance, following the
        // pose's Hips height, never slower than the stanceBlendTime ramp; snap without the other reference / with time 0
        const float cTarget = aStance == kCrouch ? 1.0f : 0.0f;
        const bool canBlend = HasReference(1 - aStance) && m_cfg.stanceBlendTime > 0.0f;
        float c = m_c;
        if (!m_init || !canBlend)
            c = cTarget;
        else if (c != cTarget)
        {
            const float ramp = dt / m_cfg.stanceBlendTime;
            const float cp = PoseCrouchFraction(aPose); // NaN = ramp only
            if (cTarget > c)
                c = std::fmin(1.0f, std::isfinite(cp) ? std::fmax(c + ramp, cp) : c + ramp);
            else
                c = std::fmax(0.0f, std::isfinite(cp) ? std::fmin(c - ramp, cp) : c - ramp);
        }
        float dev;
        if (c == cTarget)
            dev = Deviation(aPose, aStance); // no switch in progress: the single-reference path, exactly
        else
        {
            const float dS = Deviation(aPose, kStand), dC = Deviation(aPose, kCrouch);
            dev = math::WrapAngle(dS + c * math::WrapAngle(dC - dS));
        }
        if (!std::isfinite(dev))
        {
            ++m_diag.nonFinite;
            return;
        }
        if (m_init && canBlend && (c != cTarget || m_c != c))
            ++m_diag.blendUpdates;
        m_c = c;
        m_diag.blend = c;
        const float maxRad = m_cfg.chestMaxDeg * (math::kPi / 180.0f);
        if (!m_init)
        {
            m_low = dev;
            m_delta = -Clamp(dev, maxRad);
            m_init = true;
        }
        else
        {
            if (aStance != m_lastStance && !canBlend)
                m_low = dev; // snap switch (no blend): devLow restarts at the new stance's dev
            m_low += math::WrapAngle(dev - m_low) * Alpha(dt, m_cfg.devLowHalflife);
            m_low = math::WrapAngle(m_low);
        }
        m_lastStance = aStance;
        const float target = -Clamp(m_low + (1.0f - m_cfg.swayKeep) * math::WrapAngle(dev - m_low), maxRad);
        m_delta += (target - m_delta) * Alpha(dt, m_cfg.smoothHalflife);
        m_delta = Clamp(m_delta, maxRad);
        const float w = std::isfinite(aWeight) ? std::fmin(std::fmax(aWeight, 0.0f), 1.0f) : 0.0f;
        m_diag.dev = dev * (180.0f / math::kPi);
        m_diag.low = m_low * (180.0f / math::kPi);
        m_diag.valid = true;
        if (!(w > 0.0f))
            return;
        const float d = m_delta * w;
        const BoneTransform hips = Model(aPose, m_hips);
        const math::Quat hq = math::QuatNormalize(hips.q);
        const math::Quat rz{0.0f, 0.0f, std::sin(0.5f * d), std::cos(0.5f * d)};
        const math::Quat pre = math::QuatMul(math::QuatMul(math::QuatInv(hq), rz), hq);
        const math::Quat nq = math::QuatNormalize(math::QuatMul(pre, aPose[size_t(m_spine)].q));
        if (!math::IsFinite(nq))
        {
            ++m_diag.nonFinite;
            return;
        }
        aPose[size_t(m_spine)].q = nq;
        m_diag.weight = w;
        m_diag.delta = d * (180.0f / math::kPi);
        ++m_diag.activeUpdates;
    }

    // Twist angle (rad) about +Z of a unit rotation (swing-twist projection).
    static float TwistZ(const math::Quat& q)
    {
        if (!(std::fabs(q.z) + std::fabs(q.w) > 1e-6f))
            return 0.0f;
        return math::WrapAngle(2.0f * std::atan2(q.z, q.w));
    }

    BoneTransform Model(std::span<const BoneTransform> aPose, int32_t aBone) const
    {
        int32_t path[64];
        int32_t n = 0;
        for (int32_t b = aBone; b >= 0 && n < 64; b = m_parents[size_t(b)])
            path[n++] = b;
        BoneTransform g{};
        for (int32_t i = n - 1; i >= 0; --i)
            g = BoneTransform{g.p + math::QuatRotate(g.q, aPose[size_t(path[i])].p), math::QuatMul(g.q, aPose[size_t(path[i])].q)};
        return g;
    }

private:
    static float Clamp(float a, float m) { return std::fmin(std::fmax(a, -m), m); }
    static float Alpha(float aDt, float aHalflife)
    {
        if (!(aDt > 0.0f))
            return 0.0f;
        if (!(aHalflife > 0.0f))
            return 1.0f;
        return 1.0f - std::exp2(-aDt / aHalflife);
    }

    ChestAlignConfig m_cfg{};
    ChestAlignDiagnostics m_diag{};
    std::vector<int32_t> m_parents;
    std::vector<BoneTransform> m_scratch; // BuildReference only
    math::Quat m_ref[kNumStances]{};
    float m_hipsZ[kNumStances] = {0.0f, 0.0f}; // m, mean Hips model height of each reference loop
    bool m_refOk[kNumStances] = {false, false};
    int32_t m_hips = -1, m_spine = -1;
    bool m_ok = false, m_init = false;
    int m_lastStance = -1;
    float m_low = 0.0f, m_delta = 0.0f; // rad
    float m_c = 0.0f;                   // crouch fraction of the reference (stance-switch blend)
};
} // namespace utpp::mm

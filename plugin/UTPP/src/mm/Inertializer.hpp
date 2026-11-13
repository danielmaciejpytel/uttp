// UTPP motion matching runtime - offset-based pose inertialization. Header-only; allocates only in Init().
//
// Re-implementation of orangeduck's inertialize_transition / inertialize_update (Motion-Matching
// spring.h + controller.cpp inertialize_pose_transition / inertialize_pose_update; MIT License, Copyright (c) 2021
// Daniel Holden), applied to LOCAL bone transforms of bones [firstBone, numBones) (Hips position included). The
// root-space adjustment orangeduck does for bone 0 is not needed: the game moves the capsule (root motion) itself and
// our bones 0/1 stay identity.
//
//   transition: off_p = (src_p + off_p) - dst_p          off_v = (src_v + off_v) - dst_v
//               off_q = abs(off_q * src_q * inv(dst_q))   off_w = (src_w + off_w) - dst_w
//   update:     decay_spring_damper_exact(off, half-life, dt); out_p = in_p + off_p; out_q = normalize(off_q * in_q)
// Output quaternions are kept sign-continuous with the previous output. Any non-finite result resets the offsets and
// outputs the input pose (Apply() returns false).
// UTPP-MOTION SEAM (port addition; header comment lines 14-16 and the block between SEAM begin / SEAM end)
// UTPP-MOTION SEAM QuinticInertializer (below): blend-time based per-bone inertialization with UE AnimNode_Inertialization semantics
// UTPP-MOTION SEAM (port; not used by BaseMatcher yet).
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "MMMath.hpp"

namespace utpp::mm
{
struct BoneTransform
{
    math::Vec3 p;
    math::Quat q;
};
static_assert(sizeof(BoneTransform) == 28, "BoneTransform must match the .utppmm POSE record (7 floats)");

class PoseInertializer
{
public:
    bool Init(uint32_t aNumBones, uint32_t aFirstBone)
    {
        if (aNumBones == 0 || aFirstBone >= aNumBones)
            return false;
        m_numBones = aNumBones;
        m_firstBone = aFirstBone;
        m_offP.assign(aNumBones, math::Vec3{});
        m_offV.assign(aNumBones, math::Vec3{});
        m_offQ.assign(aNumBones, math::Quat{});
        m_offW.assign(aNumBones, math::Vec3{});
        m_prevOutQ.assign(aNumBones, math::Quat{});
        Reset();
        return true;
    }

    void Reset()
    {
        for (uint32_t b = 0; b < m_numBones; ++b)
        {
            m_offP[b] = {};
            m_offV[b] = {};
            m_offQ[b] = {};
            m_offW[b] = {};
        }
    }

    // Forget the previous output (sign continuity restarts).
    void ResetHistory() { m_havePrev = false; }

    void Transition(const BoneTransform* aSrc, const math::Vec3* aSrcLv, const math::Vec3* aSrcAv, const BoneTransform* aDst,
                    const math::Vec3* aDstLv, const math::Vec3* aDstAv)
    {
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            m_offP[b] = (aSrc[b].p + m_offP[b]) - aDst[b].p;
            m_offV[b] = (aSrcLv[b] + m_offV[b]) - aDstLv[b];
            m_offQ[b] = math::QuatAbs(math::QuatNormalize(math::QuatMul(math::QuatMul(m_offQ[b], aSrc[b].q), math::QuatInv(aDst[b].q))));
            m_offW[b] = (aSrcAv[b] + m_offW[b]) - aDstAv[b];
        }
    }

    void Decay(float aHalflife, float aDt)
    {
        if (!(aDt > 0.0f))
            return;
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            math::DecaySpringDamperExact(m_offP[b], m_offV[b], aHalflife, aDt);
            math::DecaySpringDamperExact(m_offQ[b], m_offW[b], aHalflife, aDt);
        }
    }

    bool Apply(const BoneTransform* aIn, BoneTransform* aOut)
    {
        bool finite = true;
        for (uint32_t b = 0; b < m_numBones; ++b)
        {
            if (b < m_firstBone)
            {
                aOut[b] = aIn[b];
                continue;
            }
            aOut[b].p = aIn[b].p + m_offP[b];
            aOut[b].q = math::QuatNormalize(math::QuatMul(m_offQ[b], aIn[b].q));
            finite = finite && math::IsFinite(aOut[b].p) && math::IsFinite(aOut[b].q) && math::IsFinite(m_offV[b]) &&
                     math::IsFinite(m_offW[b]);
        }
        if (!finite)
        {
            Reset();
            for (uint32_t b = 0; b < m_numBones; ++b)
                aOut[b] = aIn[b];
        }
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            if (m_havePrev && math::QuatDot(aOut[b].q, m_prevOutQ[b]) < 0.0f)
                aOut[b].q = math::QuatNeg(aOut[b].q);
            m_prevOutQ[b] = aOut[b].q;
        }
        m_havePrev = true;
        return finite;
    }

    // Largest current rotation offset (radians) and position offset (metres) over the inertialized bones.
    void MaxOffsets(float& aAngle, float& aDist) const
    {
        aAngle = 0.0f;
        aDist = 0.0f;
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            const float a = math::QuatAngleBetween(m_offQ[b], math::Quat{});
            const float d = std::sqrt(math::Dot(m_offP[b], m_offP[b]));
            aAngle = a > aAngle ? a : aAngle;
            aDist = d > aDist ? d : aDist;
        }
    }

private:
    uint32_t m_numBones = 0;
    uint32_t m_firstBone = 0;
    bool m_havePrev = false;
    std::vector<math::Vec3> m_offP, m_offV, m_offW;
    std::vector<math::Quat> m_offQ, m_prevOutQ;
};

// UTPP-MOTION SEAM begin: QuinticCurve / QuinticInertializer (port 8597dc4; used by the v2 path only)
// =====================================================================================================================
// Blend-time inertialization (port, stage 10.1): per-bone quintic offsets with UE AnimNode_Inertialization
// semantics (UE 5.8 Engine\Source\Runtime\Engine\Private\Animation\AnimNode_Inertialization.cpp):
//   * CalcInertialFloat (95-224): quintic x(t) with x(0) = x0, x'(0) = v0, x(t1) = x'(t1) = x''(t1) = 0. A v0 pointing
//     away from zero is clamped to 0 and t1 is shortened to -5 x0 / v0 for fast approaches, so the curve never
//     overshoots (x stays in [0, x0]); a0 is the zero-jerk-at-t1 value clamped to >= 0. Converges exactly at t1.
//   * InitFrom (761-923): offsets from the last two OUTPUT poses (snapshots) to the new target pose; translation as
//     direction + magnitude, rotation as axis + angle (left offset q = off * target); speed = rate of the magnitude /
//     angle along the direction / axis (the perpendicular part is dropped, as in UE).
//   * Requests (455-600): elapsed restarts at 0 and advances by the frame delta before the first evaluation (the first
//     output is the curve at t = dt); several requests in one frame keep the minimum duration; an interruption of a
//     running blend records the deficit (duration - elapsed) and a FURTHER interruption shortens the new duration by
//     it (repeated switches cannot keep the pose away from the target); per-bone durations = duration * factor
//     (blend profile: TimeFactor scale, WeightFactor 1 / scale - BlendProfile.cpp FillSkeletonBoneDurationsArray).
//   * The blend is inactive once elapsed >= the longest bone duration; the output is then the target pose exactly.
// Extension: Update(aDst, aDstPrev, ...) with the target pose one frame earlier measures the offset and its velocity
// relative to the target's own motion, so the output velocity stays continuous at the switch also when the target
// moves (UE ignores the target velocity; aDstPrev == nullptr reproduces UE).
// Blend durations are inputs (data-driven rules: Chooser.hpp GetBlendTime). Allocates only in Init().

inline constexpr float kInertialTimeEpsilon = 1.0e-7f; // INERTIALIZATION_TIME_EPSILON
inline constexpr float kInertialSmallSpeed = 1.0e-4f;  // UE_KINDA_SMALL_NUMBER (per second)
inline constexpr float kInertialSmallDist = 1.0e-6f;   // metres / radians: below = no offset direction

// Quintic inertialization curve (UE CalcInertialFloat), parameters precomputed once per transition.
struct QuinticCurve
{
    float x0 = 0.0f; // magnitude after the sign flip (>= 0)
    float v0 = 0.0f; // clamped initial speed (<= 0)
    float t1 = 0.0f; // effective end time (possibly shortened)
    float a = 0.0f, b = 0.0f, c = 0.0f, d = 0.0f;
    float sign = 1.0f;
    bool zero = true;

    void Init(float aX0, float aV0, float aT1)
    {
        *this = QuinticCurve{};
        float x = aX0, v = aV0, te = aT1;
        if (x < 0.0f)
        {
            x = -x;
            v = -v;
            sign = -1.0f;
        }
        if (v > 0.0f)
            v = 0.0f; // moving away from zero would overshoot
        if (!(x >= 0.0f && v <= 0.0f && te >= 0.0f) || !std::isfinite(x) || !std::isfinite(v) || !std::isfinite(te))
            return; // invalid input (UE ensure path): zero curve
        if (v < -kInertialSmallSpeed)
            te = std::fmin(te, -5.0f * x / v); // quadruple velocity root at t1: no overshoot below zero
        if (te <= kInertialTimeEpsilon)
            return;
        const float t12 = te * te, t13 = t12 * te, t14 = t13 * te, t15 = t14 * te;
        const float a0 = std::fmax(0.0f, (-8.0f * te * v - 20.0f * x) / t12);
        a = -0.5f * (a0 * t12 + 6.0f * te * v + 12.0f * x) / t15;
        b = 0.5f * (3.0f * a0 * t12 + 16.0f * te * v + 30.0f * x) / t14;
        c = -0.5f * (3.0f * a0 * t12 + 12.0f * te * v + 20.0f * x) / t13;
        d = 0.5f * a0;
        x0 = x;
        v0 = v;
        t1 = te;
        zero = false;
    }

    float Eval(float t) const
    {
        if (zero || t >= t1 - kInertialTimeEpsilon)
            return 0.0f;
        if (t < 0.0f)
            t = 0.0f;
        return sign * (((((a * t + b) * t + c) * t + d) * t + v0) * t + x0);
    }

    float Velocity(float t) const
    {
        if (zero || t >= t1 - kInertialTimeEpsilon)
            return 0.0f;
        if (t < 0.0f)
            t = 0.0f;
        return sign * ((((5.0f * a * t + 4.0f * b) * t + 3.0f * c) * t + 2.0f * d) * t + v0);
    }
};

// UE CalcInertialFloat(x0, v0, t, t1).
inline float InertialCurve(float aX0, float aV0, float aT, float aT1)
{
    QuinticCurve c;
    c.Init(aX0, aV0, aT1);
    return c.Eval(aT);
}

// Per-bone duration factor of a blend profile entry (UE BlendProfile.cpp FillSkeletonBoneDurationsArray).
inline float BlendProfileDurationFactor(bool aWeightFactor, float aScale)
{
    if (aWeightFactor)
        return aScale > 1.0e-8f ? 1.0f / aScale : 1.0f;
    return aScale > 0.0f ? aScale : 0.0f;
}

class QuinticInertializer
{
public:
    bool Init(uint32_t aNumBones, uint32_t aFirstBone)
    {
        if (aNumBones == 0 || aFirstBone >= aNumBones)
            return false;
        m_numBones = aNumBones;
        m_firstBone = aFirstBone;
        m_factor.assign(aNumBones, 1.0f);
        m_boneDuration.assign(aNumBones, 0.0f);
        m_tDir.assign(aNumBones, math::Vec3{});
        m_rAxis.assign(aNumBones, math::Vec3{});
        m_tCurve.assign(aNumBones, QuinticCurve{});
        m_rCurve.assign(aNumBones, QuinticCurve{});
        m_prev1.assign(aNumBones, BoneTransform{});
        m_prev2.assign(aNumBones, BoneTransform{});
        m_prevOutQ.assign(aNumBones, math::Quat{});
        Reset();
        return true;
    }

    // motionfix item 7 (pose snaps): floor of the EFFECTIVE duration of a transition. UE's deficit rule (Request / Update)
    // shortens a blend requested while another one just started by the remaining deficit, so back-to-back pose jumps (a
    // sprint stop whose game velocity collapses within two feeds, a re-takeoff 0.3 s after a landing, in-game 2026-10-13) end
    // with ~1-frame blends = pops. With aSeconds > 0 a transition lasts at least min(aSeconds, requested duration); the deficit
    // is still recorded and paid down to the floor, and a request that ignores the deficit (landing bypass) is unchanged
    // (its full duration >= the floor). 0 (default) = UE semantics; the v2 pipeline sets adapter::kBlendFloorSeconds.
    void SetMinDuration(float aSeconds) { m_minDuration = (std::isfinite(aSeconds) && aSeconds > 0.0f) ? aSeconds : 0.0f; }
    float MinDuration() const { return m_minDuration; }

    // Per-bone duration factors (size = numBones); nullptr = 1 for every bone.
    void SetDurationFactors(const float* aFactors)
    {
        for (uint32_t i = 0; i < m_numBones; ++i)
            m_factor[i] = (aFactors && std::isfinite(aFactors[i]) && aFactors[i] >= 0.0f) ? aFactors[i] : 1.0f;
    }

    // Forgets the pose history and stops any blend (teleport, database / rig switch).
    void Reset()
    {
        Deactivate();
        m_numSnapshots = 0;
        m_prebaseDt = 0.0f;
        m_requested = -1.0f;
        m_requestIgnoresDeficit = false;
        m_havePrevOut = false;
    }

    // Requests a transition to whatever target the next Update() receives. Several requests before one Update():
    // the shortest duration wins. Ignored until one output pose has been recorded (UE: no snapshot, no request).
    // aIgnoreDeficit (air): this transition does not pay the interruption deficit (UE CVar
    // a.AnimNode.Inertialization.IgnoreDeficit, here per request; the deficit is still recorded). false = UE default.
    void Request(float aDuration, bool aIgnoreDeficit = false)
    {
        if (!std::isfinite(aDuration))
            return;
        aDuration = std::fmax(aDuration, 0.0f);
        m_requested = m_requested < 0.0f ? aDuration : std::fmin(m_requested, aDuration);
        m_requestIgnoresDeficit = m_requestIgnoresDeficit || aIgnoreDeficit;
    }

    // aDst: target pose of this frame (local transforms, numBones). aDstPrev: the same target one frame earlier (same
    // animation timeline) or nullptr (UE: target velocity ignored). aDt: frame delta (s). Writes aOut (numBones).
    // Bones below firstBone are copied. Returns false when a non-finite value forced a reset (aOut = aDst).
    bool Update(const BoneTransform* aDst, const BoneTransform* aDstPrev, float aDt, BoneTransform* aOut)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        if (m_requested >= 0.0f && m_numSnapshots > 0)
        {
            float applied = 0.0f;
            if (m_state == State::Active)
            {
                const bool apply = m_deficit > 0.0f && !m_requestIgnoresDeficit;
                m_deficit = m_duration - m_elapsed;
                applied = apply ? m_deficit : 0.0f;
            }
            m_state = State::Pending;
            m_elapsed = 0.0f;
            m_duration = std::fmax(m_requested - applied, 0.0f);
#if !(defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 1) // mutation 1: no blend floor (UE deficit cut)
            if (m_minDuration > 0.0f && m_requested > 0.0f) // motionfix item 7: SetMinDuration
                m_duration = std::fmax(m_duration, std::fmin(m_minDuration, m_requested));
#endif
            m_maxDuration = m_duration;
            for (uint32_t b = 0; b < m_numBones; ++b)
            {
                m_boneDuration[b] = m_duration * m_factor[b];
                m_maxDuration = std::fmax(m_maxDuration, m_boneDuration[b]);
            }
            ++m_transitions;
        }
        m_requested = -1.0f;
        m_requestIgnoresDeficit = false;

        if (m_state != State::Inactive)
        {
            m_elapsed += dt;
            if (m_elapsed >= m_duration)
                m_deficit = 0.0f;
            else
                m_deficit -= std::fmin(m_deficit, dt);
            if (m_elapsed >= m_maxDuration)
                Deactivate();
        }
        if (m_state == State::Pending)
        {
            InitFrom(aDst, aDstPrev, dt);
            m_state = State::Active;
        }

        bool finite = true;
        for (uint32_t b = 0; b < m_numBones; ++b)
        {
            aOut[b] = aDst[b];
            if (b < m_firstBone || m_state != State::Active)
                continue;
            const float x = m_tCurve[b].Eval(m_elapsed);
            aOut[b].p = aDst[b].p + m_tDir[b] * x;
            const float ang = m_rCurve[b].Eval(m_elapsed);
            if (ang != 0.0f)
                aOut[b].q = math::QuatNormalize(math::QuatMul(AxisAngle(m_rAxis[b], ang), aDst[b].q));
            finite = finite && math::IsFinite(aOut[b].p) && math::IsFinite(aOut[b].q);
        }
        if (!finite)
        {
            Reset();
            for (uint32_t b = 0; b < m_numBones; ++b)
                aOut[b] = aDst[b];
        }
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            if (m_havePrevOut && math::QuatDot(aOut[b].q, m_prevOutQ[b]) < 0.0f)
                aOut[b].q = math::QuatNeg(aOut[b].q);
            m_prevOutQ[b] = aOut[b].q;
        }
        m_havePrevOut = true;

        // Pose snapshots: prev2 <- prev1 <- this output (with this frame's delta).
        if (m_numSnapshots > 0)
            m_prev2.swap(m_prev1);
        for (uint32_t b = 0; b < m_numBones; ++b)
            m_prev1[b] = aOut[b];
        m_prebaseDt = dt;
        m_numSnapshots = m_numSnapshots < 2 ? m_numSnapshots + 1 : 2;
        return finite;
    }

    bool Active() const { return m_state == State::Active; }
    float Elapsed() const { return m_elapsed; }
    float Duration() const { return m_duration; }
    float Deficit() const { return m_deficit; }

    // Offset of one bone at elapsed time aT of the running blend (zero when inactive): translation added to the target
    // and rotation multiplied on the left, plus their time derivatives (m/s, rad/s along the stored axis).
    void OffsetAt(uint32_t aBone, float aT, math::Vec3& aPos, math::Quat& aRot, math::Vec3* aLinVel = nullptr,
                  math::Vec3* aAngVel = nullptr) const
    {
        aPos = {};
        aRot = {};
        if (aLinVel)
            *aLinVel = {};
        if (aAngVel)
            *aAngVel = {};
        if (m_state != State::Active || aBone < m_firstBone || aBone >= m_numBones)
            return;
        aPos = m_tDir[aBone] * m_tCurve[aBone].Eval(aT);
        aRot = AxisAngle(m_rAxis[aBone], m_rCurve[aBone].Eval(aT));
        if (aLinVel)
            *aLinVel = m_tDir[aBone] * m_tCurve[aBone].Velocity(aT);
        if (aAngVel)
            *aAngVel = m_rAxis[aBone] * m_rCurve[aBone].Velocity(aT);
    }

    const QuinticCurve& TranslationCurve(uint32_t aBone) const { return m_tCurve[aBone]; }
    const QuinticCurve& RotationCurve(uint32_t aBone) const { return m_rCurve[aBone]; }

private:
    enum class State : uint8_t
    {
        Inactive,
        Pending,
        Active
    };

    static math::Quat AxisAngle(math::Vec3 aAxis, float aAngle)
    {
        const float s = std::sin(0.5f * aAngle);
        return math::Quat{aAxis.x * s, aAxis.y * s, aAxis.z * s, std::cos(0.5f * aAngle)};
    }

    // Angular velocity taking aCurr to aNext (parent space, rad/s); atan2 form, accurate for small frame rotations
    // (the acos form loses ~1e-5 rad per frame in float, i.e. ~3e-3 rad/s at 144 fps).
    static math::Vec3 AngularVelocity(const math::Quat& aNext, const math::Quat& aCurr, float aInvDt)
    {
        const math::Quat r = math::QuatAbs(math::QuatNormalize(math::QuatMul(aNext, math::QuatInv(aCurr))));
        const float s = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
        if (s < 1.0e-12f)
            return math::Vec3{r.x, r.y, r.z} * (2.0f * aInvDt);
        return math::Vec3{r.x, r.y, r.z} * (2.0f * std::atan2(s, r.w) / s * aInvDt);
    }

    void Deactivate()
    {
        m_state = State::Inactive;
        m_elapsed = 0.0f;
        m_duration = 0.0f;
        m_maxDuration = 0.0f;
        m_deficit = 0.0f;
    }

    void InitFrom(const BoneTransform* aDst, const BoneTransform* aDstPrev, float aDt)
    {
        const std::vector<BoneTransform>& p1 = m_prev1;
        const std::vector<BoneTransform>& p2 = m_numSnapshots >= 2 ? m_prev2 : m_prev1; // one snapshot: zero velocity
        const bool srcVel = m_numSnapshots >= 2 && m_prebaseDt > 0.0f;
        const float invSrcDt = srcVel ? 1.0f / m_prebaseDt : 0.0f;
        const bool dstVel = aDstPrev != nullptr && aDt > 0.0f;
        const float invDstDt = dstVel ? 1.0f / aDt : 0.0f;
        for (uint32_t b = m_firstBone; b < m_numBones; ++b)
        {
            const BoneTransform& ref = aDstPrev ? aDstPrev[b] : aDst[b];
            // translation
            const math::Vec3 t = p1[b].p - ref.p;
            const float mag = std::sqrt(math::Dot(t, t));
            math::Vec3 dir{};
            float speed = 0.0f;
            if (mag > kInertialSmallDist && std::isfinite(mag))
            {
                dir = t * (1.0f / mag);
                math::Vec3 v = (p1[b].p - p2[b].p) * invSrcDt;
                if (dstVel)
                    v = v - (aDst[b].p - aDstPrev[b].p) * invDstDt;
                speed = math::Dot(v, dir);
            }
            m_tDir[b] = dir;
            m_tCurve[b].Init(dir.x == 0.0f && dir.y == 0.0f && dir.z == 0.0f ? 0.0f : mag, speed, m_boneDuration[b]);

            // rotation: q0 = prev1 * inv(ref), angle in [0, pi]
            const math::Quat q0 = math::QuatAbs(math::QuatNormalize(math::QuatMul(p1[b].q, math::QuatInv(ref.q))));
            const float s = std::sqrt(q0.x * q0.x + q0.y * q0.y + q0.z * q0.z);
            const float angle = 2.0f * std::atan2(s, q0.w);
            math::Vec3 axis{};
            float rspeed = 0.0f;
            if (angle > kInertialSmallDist && s > 0.0f)
            {
                axis = math::Vec3{q0.x / s, q0.y / s, q0.z / s};
                math::Vec3 w = srcVel ? AngularVelocity(p1[b].q, p2[b].q, invSrcDt) : math::Vec3{};
                if (dstVel)
                    w = w - math::QuatRotate(q0, AngularVelocity(aDst[b].q, aDstPrev[b].q, invDstDt));
                rspeed = math::Dot(w, axis);
            }
            m_rAxis[b] = axis;
            m_rCurve[b].Init(s > 0.0f && angle > kInertialSmallDist ? angle : 0.0f, rspeed, m_boneDuration[b]);
        }
    }

    uint32_t m_numBones = 0;
    uint32_t m_firstBone = 0;
    State m_state = State::Inactive;
    float m_requested = -1.0f;
    bool m_requestIgnoresDeficit = false;
    float m_elapsed = 0.0f;
    float m_duration = 0.0f;
    float m_maxDuration = 0.0f;
    float m_deficit = 0.0f;
    float m_minDuration = 0.0f; // motionfix item 7: SetMinDuration (0 = UE semantics)
    float m_prebaseDt = 0.0f;
    uint32_t m_numSnapshots = 0;
    uint32_t m_transitions = 0;
    bool m_havePrevOut = false;
    std::vector<float> m_factor, m_boneDuration;
    std::vector<math::Vec3> m_tDir, m_rAxis;
    std::vector<QuinticCurve> m_tCurve, m_rCurve;
    std::vector<BoneTransform> m_prev1, m_prev2;
    std::vector<math::Quat> m_prevOutQ;
};
// UTPP-MOTION SEAM end: QuinticCurve / QuinticInertializer (additive; v1 BaseMatcher uses only Inertializer above)

} // namespace utpp::mm

// UTPP Phase 5.3c-2 (0.3.6) - stance-width guard. Header-only, deterministic, no allocations after Init().
//
// Measured (utpp_mm_steering_test, UTPP_TEST_STANCE=1, full_posture database): in the retargeted motion walk the ankles
// pass each other on the body midline - Walk_Loop_F passing clearance mean -0.7 cm, feet on the swapped side in 67 % of
// the passing frames, shins (knee-ankle segments) closer than 9 cm in 7.8 % of all frames (min 0.3 cm); Walk_Turn similar,
// run / sprint / crouch much less. The CP hip-joint width is ~14.5 cm, so legs pass through each other on the CP mesh.
// The crossing is in the database (retarget maps the narrow source ankle paths onto the wider CP pelvis); this guard is
// the runtime fix until a re-bake with a stance-width correction exists.
//
// Per leg, on the pose before the foot locker (character frame, x right): lateral ankle position relative to the
// midpoint of the two hip joints must be <= -minHalfWidth (left) / >= +minHalfWidth (right). The violation is passed
// through a C1 soft ramp (width softWidth) and capped at maxPush; the leg is rotated rigidly about the hip joint
// (UpLeg model rotation pre-multiplied by the minimal rotation taking hip->ankle onto hip->target), the foot keeps its
// model-space orientation. Knee follows the thigh, so the shins separate as well. Continuous in the input pose.
#pragma once

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "MMMath.hpp"

namespace utpp::mm
{
struct StanceGuardConfig
{
    // 0.3.8 defaults from the CP female walk reference (tpp_fem_v_locomotion fpp_walk_slow_0 / fast_0: ankles pass
    // 5.3-5.9 cm apart, knees 6.7-7.9 cm, hip joints 15.3 cm; Source/tools/recon/stance_reference.py). 0.3.6/0.3.7: 5 / 10 cm.
    float minHalfWidth = 0.03f; // m, ankle distance from the pelvis midline
    float softWidth = 0.02f;    // m, smooth ramp half-width
    float maxPush = 0.12f;      // m, largest lateral correction per ankle
    // 0.3.7 swing mode (Apply with contacts): push changes only on the swinging foot, held while planted
    bool swingOnly = true;
    float minSeparation = 0.06f; // m, swing ankle vs the other ankle (lateral); 0.3.7: 0.10
    bool kneeIk = true;          // 0.3.8: two-bone IK keeping the knee (false = 0.3.7 rigid leg rotation, knees bow out)
    bool kneeSwivel = true;      // 0.3.8: rotate the leg about the hip-ankle axis towards a forward knee (false = off)
    float swivelWeight = 1.0f;   // fraction of the knee-to-forward-pole angle removed
    float maxSwivel = 0.9f;      // rad (~52 deg) per leg
    // 0.3.9 anti-jitter (0 = 0.3.8 path): first-order filter time of the knee swivel (s), straight-leg fade, continuous
    // knee floor; see Apply. Requires Apply(..., aDt).
    float smoothTime = 0.08f;
    float kneeMinHalf = 0.045f;  // m, knee distance from the midline kept by the swivel (reference knees ~9.4 cm apart)
    float contactOn = 0.5f;      // contact above = planted
    float pushRate = 0.8f;       // m/s, rate of change of the push during the swing
};

struct StanceGuardDiagnostics
{
    float push[2] = {0.0f, 0.0f}; // m, applied lateral correction this update (left, right)
    float swivel[2] = {0.0f, 0.0f}; // rad, 0.3.8 knee swivel this update
    float maxPush = 0.0f;
    uint64_t updates = 0, activeUpdates = 0, nonFinite = 0;
};

class StanceGuard
{
public:
    bool Init(const Database& aDb, const StanceGuardConfig& aCfg = {})
    {
        m_ok = false;
        if (!aDb.Loaded() || !Valid(aCfg))
            return false;
        static constexpr std::string_view kNames[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"},
                                                          {"RightUpLeg", "RightLeg", "RightFoot"}};
        const uint32_t nb = aDb.NumBones();
        m_parents.assign(nb, -1);
        for (uint32_t b = 0; b < nb; ++b)
        {
            const int32_t p = aDb.BoneParent(b);
            if (p >= static_cast<int32_t>(b))
                return false;
            m_parents[b] = p;
        }
        for (int s = 0; s < 2; ++s)
            for (int k = 0; k < 3; ++k)
            {
                m_chain[s][k] = -1;
                for (uint32_t b = 0; b < nb; ++b)
                    if (aDb.BoneName(b) == kNames[s][k])
                        m_chain[s][k] = static_cast<int32_t>(b);
                if (m_chain[s][k] < 0)
                    return false;
            }
        for (int s = 0; s < 2; ++s)
            if (m_parents[size_t(m_chain[s][1])] != m_chain[s][0] || m_parents[size_t(m_chain[s][2])] != m_chain[s][1] ||
                m_parents[size_t(m_chain[s][0])] < 0)
                return false;
        m_cfg = aCfg;
        m_out.assign(nb, BoneTransform{});
        m_ok = true;
        return true;
    }
    bool Ready() const { return m_ok; }
    // 0.3.9: capsule speed (m/s) of this update, used to fade the knee swivel above walking speed (smooth path only).
    void SetSpeed(float aSpeed) { m_speed = std::isfinite(aSpeed) ? std::fabs(aSpeed) : 0.0f; }
    // batch 2 knee fix (UTPPSetMMKneeFix bit 1, the driver sets it per feed from the stance: crouch): knee swivel target 0 for
    // the next updates - on the smooth path the applied swivel fades out through its filter (no step), on the 0.3.8 path the
    // swivel is skipped. The bow-legged gait the swivel corrects is a standing walk problem; in crouch the forward pole (character
    // +Y, no stance gate) pushed the knee 15-35 cm inward. False (default) = the 0.3.9 path bit for bit.
    void SetSwivelSuppressed(bool aOn) { m_swivelOff = aOn; }
    bool SwivelSuppressed() const { return m_swivelOff; }
    // batch 5 knee fix (UTPPSetMMKneeFix bit 4; smooth path with contacts only, not while suppressed): the swivel filter gain is
    // scaled by g + (1 - g) * max(0, 1 - contact / contactOn), g = kPlantedSwivelGain - while the foot is planted the applied swivel
    // follows its target at 0.3 of the normal rate (no knee swing over a planted foot), the full rate returns as the contact falls
    // at lift-off. On standing turn starts (Reface / Start clips, utpp_base) the target swept by up to ~40 deg while the turning
    // foot was planted and the filtered swivel added 3-4 deg/frame to the raw knee swing (15 added break frames; all from the stance
    // guard: foot lock off gave the same count, stance guard off none). Measured and left out: a full hold (g 0: the held swivel
    // caught up after lift-off, standing restarts on v6 5 -> 11 break frames), a swivel rate limit 180 / 300 deg/s (no gain), a
    // pelvis-forward pole instead of +Y (turn starts 25 -> 22), a contact-independent 0.3 gain (turn starts about as good, but the
    // utpp_base steady gait got 3 added break frames - the contact gate keeps the full rate in the loops; review 2026-10-13).
    // False (default) = the 0.3.9 / batch 2 path bit for bit.
    static constexpr float kPlantedSwivelGain = 0.3f; // offline sweep 0 / 0.15 / 0.3 / 0.5 (v6, utpp_base): 0.3 best on both
    void SetPlantedHold(bool aOn) { m_plantedHold = aOn; }
    void Reset() { m_held[0] = m_held[1] = m_swivel[0] = m_swivel[1] = m_heldVel[0] = m_heldVel[1] = 0.0f; }
    void ResetDiagnostics() { m_diag = StanceGuardDiagnostics{}; } // b5 testorder (driver session reset)
    // b5 testorder repair: the filtered knee swivel / held push is non-zero (state Reset() clears)
    bool FilterActive() const
    {
        return m_held[0] != 0.0f || m_held[1] != 0.0f || m_swivel[0] != 0.0f || m_swivel[1] != 0.0f || m_heldVel[0] != 0.0f ||
               m_heldVel[1] != 0.0f;
    }
    static bool Valid(const StanceGuardConfig& c)
    {
        return std::isfinite(c.minHalfWidth) && c.minHalfWidth >= 0.0f && c.minHalfWidth <= 0.3f && std::isfinite(c.softWidth) &&
               c.softWidth > 0.0f && c.softWidth <= 0.1f && std::isfinite(c.maxPush) && c.maxPush >= 0.0f && c.maxPush <= 0.3f &&
               std::isfinite(c.minSeparation) && c.minSeparation >= 0.0f && c.minSeparation <= 0.4f && std::isfinite(c.contactOn) &&
               c.contactOn > 0.0f && c.contactOn < 1.0f && std::isfinite(c.pushRate) && c.pushRate > 0.0f && c.pushRate <= 10.0f &&
               std::isfinite(c.swivelWeight) && c.swivelWeight >= 0.0f && c.swivelWeight <= 1.0f && std::isfinite(c.maxSwivel) &&
               c.maxSwivel >= 0.0f && c.maxSwivel <= 1.6f && std::isfinite(c.kneeMinHalf) && c.kneeMinHalf >= 0.0f &&
               c.kneeMinHalf <= 0.3f && std::isfinite(c.smoothTime) && c.smoothTime >= 0.0f && c.smoothTime <= 1.0f;
    }
    bool SetConfig(const StanceGuardConfig& c)
    {
        if (!Valid(c))
            return false;
        m_cfg = c;
        return true;
    }
    const StanceGuardConfig& Config() const { return m_cfg; }
    const StanceGuardDiagnostics& Diagnostics() const { return m_diag; }
    std::span<const BoneTransform> Output() const { return m_out; }

    // aIn: NumBones local transforms (Root/Trajectory identity: model space == character frame). Result in Output().
    // 0.3.7 swing mode: aContactL/R in [0,1] (database contact of the played frame) and aDt (s) given and
    // Config().swingOnly: the push of a foot only changes while that foot swings (contact < contactOn), at most pushRate
    // m/s, and is HELD unchanged while it is planted (no lateral slide of a planted foot -> no extra foot-lock correction).
    // The swing foot also keeps minSeparation from the other ankle (crossing resolved by the swinging leg). Without
    // contacts (or swingOnly off) the 0.3.6 instantaneous push.
    void Apply(std::span<const BoneTransform> aIn, float aContactL = -1.0f, float aContactR = -1.0f, float aDt = 0.0f)
    {
        if (!m_ok || aIn.size() != m_out.size())
            return;
        for (size_t b = 0; b < m_out.size(); ++b)
            m_out[b] = aIn[b];
        ++m_diag.updates;
        m_diag.push[0] = m_diag.push[1] = 0.0f;
        m_diag.swivel[0] = m_diag.swivel[1] = 0.0f;
        const BoneTransform hipL = Model(m_chain[0][0]), hipR = Model(m_chain[1][0]);
        const float mid = 0.5f * (hipL.p.x + hipR.p.x);
        if (!std::isfinite(mid))
        {
            ++m_diag.nonFinite;
            return;
        }
        const bool swing = m_cfg.swingOnly && aContactL >= 0.0f && aContactR >= 0.0f && std::isfinite(aContactL) &&
                           std::isfinite(aContactR);
        float outward0[2] = {0.0f, 0.0f};
        BoneTransform gcs[2];
        for (int s = 0; s < 2; ++s)
        {
            const BoneTransform gp = Model(m_parents[size_t(m_chain[s][0])]);
            gcs[s] = Mul(Mul(Mul(gp, m_out[size_t(m_chain[s][0])]), m_out[size_t(m_chain[s][1])]), m_out[size_t(m_chain[s][2])]);
            outward0[s] = (s == 0 ? -1.0f : 1.0f) * (gcs[s].p.x - mid);
        }
        if (!swing)
            m_held[0] = m_held[1] = 0.0f;
        const float contacts[2] = {aContactL, aContactR};
        bool any = false;
        for (int s = 0; s < 2; ++s)
        {
            const float sign = s == 0 ? -1.0f : 1.0f; // outward direction (x) of this leg
            const BoneTransform gp = Model(m_parents[size_t(m_chain[s][0])]);
            const BoneTransform ga = Mul(gp, m_out[size_t(m_chain[s][0])]);
            const BoneTransform gb = Mul(ga, m_out[size_t(m_chain[s][1])]);
            const BoneTransform gc = Mul(gb, m_out[size_t(m_chain[s][2])]);
            const float outward = sign * (gc.p.x - mid);     // >= minHalfWidth wanted
            float need = m_cfg.minHalfWidth;
            if (swing) // the other ankle (with its held push) must stay minSeparation away
                need = std::fmax(need, m_cfg.minSeparation - (outward0[1 - s] + m_held[1 - s]));
            const float e = need - outward;                  // violation (> 0)
            const float k = m_cfg.softWidth;
            float push = e <= -k ? 0.0f : (e >= k ? e : (e + k) * (e + k) / (4.0f * k));
            push = std::fmin(push, m_cfg.maxPush);
            if (swing)
            {
                const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.1f) : 0.0f;
                if (m_cfg.smoothTime > 0.0f && dt > 0.0f)
                {
                    // 0.3.9: critically damped spring instead of the rate limit (the 0.3.8 rate limit switched the push
                    // velocity on / off at 0.8 m/s = a 13 mm/update kink at every swing start / end -> leg jitter). The
                    // goal is the wanted push while swinging and the current value while planted, so a planted foot only
                    // settles its residual velocity (no lateral slide target).
                    const float goal = contacts[s] < m_cfg.contactOn ? push : m_held[s];
                    math::SimpleSpringDamperExact(m_held[s], m_heldVel[s], goal, m_cfg.smoothTime, dt);
                    m_held[s] = std::fmax(0.0f, std::fmin(m_cfg.maxPush, m_held[s]));
                    if (!std::isfinite(m_held[s]) || !std::isfinite(m_heldVel[s]))
                        m_held[s] = m_heldVel[s] = 0.0f;
                }
                else if (contacts[s] < m_cfg.contactOn) // swinging: move towards the wanted push, rate limited
                {
                    const float step = m_cfg.pushRate * dt;
                    m_held[s] += std::fmax(-step, std::fmin(step, push - m_held[s]));
                }
                push = m_held[s]; // planted: held
            }
            if (!(push > 1e-6f))
                continue;
            math::Quat qa{}, qb{};
            const math::Vec3 t = gc.p + math::Vec3{sign * push, 0.0f, 0.0f};
            bool solved = false;
            // 0.3.9: with smoothing on the push uses the rigid hip rotation and the (filtered) knee swivel brings the knee
            // forward; the knee-keeping IK was the jitter source (offline jerk with only the IK off 0.79 vs 0.71 mm/update^2
            // guard off, with the IK 0.97-1.08; its pole flips when the knee crosses the hip-ankle line mid-stance).
            if (m_cfg.kneeIk && !(m_cfg.smoothTime > 0.0f))
                solved = SolveKeepKnee(ga, gb, gc, t, qa, qb); // 0.3.8: ankle moves, knee stays where it was (as far as possible)
            if (!solved)
            {
                const math::Quat r = FromTo(gc.p - ga.p, t - ga.p); // 0.3.6/0.3.7: rigid leg rotation about the hip
                qa = math::QuatNormalize(math::QuatMul(r, ga.q));
                qb = math::QuatNormalize(math::QuatMul(r, gb.q));
            }
            const math::Quat la = math::QuatNormalize(math::QuatMul(math::QuatInv(gp.q), qa));
            const math::Quat lb = math::QuatNormalize(math::QuatMul(math::QuatInv(qa), qb));
            const math::Quat lc = math::QuatNormalize(math::QuatMul(math::QuatInv(qb), gc.q)); // foot keeps model rotation
            if (!math::IsFinite(la) || !math::IsFinite(lb) || !math::IsFinite(lc))
            {
                ++m_diag.nonFinite;
                continue;
            }
            m_out[size_t(m_chain[s][0])].q = la;
            if (solved)
                m_out[size_t(m_chain[s][1])].q = lb;
            m_out[size_t(m_chain[s][2])].q = lc;
            m_diag.push[s] = push;
            m_diag.maxPush = std::fmax(m_diag.maxPush, push);
            any = true;
        }
        // 0.3.8 knee swivel: the retargeted motion walk is bow-legged - knees ~26 cm apart (character x) with the hip joints
        // 15 cm and the ankles on the midline, while the CP female walk (fpp_walk_slow_0) has knees ~9 cm apart. The
        // leg (thigh + shin) is rotated about the hip-ankle axis so the knee moves towards the forward pole: ankle and foot
        // stay exactly where they are (foot locks and ground contact unaffected).
        if (m_cfg.kneeSwivel)
            for (int s = 0; s < 2; ++s)
            {
                const BoneTransform gp = Model(m_parents[size_t(m_chain[s][0])]);
                const BoneTransform ga = Mul(gp, m_out[size_t(m_chain[s][0])]);
                const BoneTransform gb = Mul(ga, m_out[size_t(m_chain[s][1])]);
                const BoneTransform gc = Mul(gb, m_out[size_t(m_chain[s][2])]);
                math::Vec3 axis = gc.p - ga.p;
                const float al = std::sqrt(math::Dot(axis, axis));
                if (!(al > 1e-3f))
                    continue;
                axis = axis * (1.0f / al);
                math::Vec3 off = (gb.p - ga.p) - axis * math::Dot(gb.p - ga.p, axis);
                math::Vec3 pole = math::Vec3{0.0f, 1.0f, 0.0f} - axis * axis.y;
                const float ol = std::sqrt(math::Dot(off, off)), pl = std::sqrt(math::Dot(pole, pole));
                if (m_cfg.smoothTime > 0.0f)
                {
                    // 0.3.9 anti-jitter path (C1 in time): (a) the swivel target fades out for an almost straight leg
                    // (the knee offset direction is ill-conditioned when the knee crosses the hip-ankle line - the 0.3.8
                    // angle flipped there, mid-stance); (b) the knee floor is met by a continuous bisection instead of the
                    // 0.3.8 quarter steps; (c) the applied angle follows the target with a first-order filter (smoothTime);
                    // (d) no per-frame on/off: the filtered angle is applied every frame.
                    float tgt = 0.0f;
                    const float sgn = s == 0 ? -1.0f : 1.0f;
                    auto rotS = [&](float aAng) {
                        const float hh = 0.5f * aAng;
                        return math::Quat{axis.x * std::sin(hh), axis.y * std::sin(hh), axis.z * std::sin(hh), std::cos(hh)};
                    };
                    if (ol > 1e-4f && pl > 1e-2f && !m_swivelOff)
                    {
                        const math::Vec3 offn = off * (1.0f / ol), polen = pole * (1.0f / pl);
                        float a0 = std::atan2(math::Dot(axis, math::Cross(offn, polen)), math::Dot(offn, polen)) * m_cfg.swivelWeight;
                        a0 = std::fmax(-m_cfg.maxSwivel, std::fmin(m_cfg.maxSwivel, a0));
                        const float u = std::fmin(1.0f, std::fmax(0.0f, (ol - 0.01f) / 0.04f));
                        a0 *= u * u * (3.0f - 2.0f * u); // fade for an almost straight leg (ol < 5 cm)
                        // fade with speed (2 -> 4 m/s): the bow-legged gait is a walk problem; at run / sprint the swivel
                        // pulled the shins together (DB Sprint_Loop_F shins < 6 cm 0 -> 1.1 %)
                        const float vs = std::fmin(1.0f, std::fmax(0.0f, (m_speed - 2.0f) / 2.0f));
                        a0 *= 1.0f - vs * vs * (3.0f - 2.0f * vs);
                        const float kneeFloor = std::fmax(m_cfg.kneeMinHalf, sgn * (gc.p.x - mid));
                        auto meets = [&](float aScale) {
                            const math::Vec3 kb = ga.p + math::QuatRotate(rotS(a0 * aScale), gb.p - ga.p);
                            return sgn * (kb.x - mid) >= kneeFloor;
                        };
                        float sc = 1.0f;
                        if (!meets(1.0f))
                        {
                            float lo = 0.0f, hi = 1.0f;
                            for (int it = 0; it < 8; ++it)
                            {
                                const float mid2 = 0.5f * (lo + hi);
                                (meets(mid2) ? lo : hi) = mid2;
                            }
                            sc = meets(0.0f) ? lo : 0.0f;
                        }
                        tgt = a0 * sc;
                    }
                    const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.1f) : 0.0f;
                    float kf = dt > 0.0f ? 1.0f - std::exp(-dt / m_cfg.smoothTime) : 1.0f;
                    // batch 5 knee fix bit 4 (see SetPlantedHold): slowed while planted, full gain again with the lift-off
                    if (m_plantedHold && swing && !m_swivelOff)
                        kf *= kPlantedSwivelGain + (1.0f - kPlantedSwivelGain) * std::fmax(0.0f, 1.0f - contacts[s] / m_cfg.contactOn);
                    m_swivel[s] += (tgt - m_swivel[s]) * kf;
                    if (!std::isfinite(m_swivel[s]))
                        m_swivel[s] = 0.0f;
                    if (!(std::fabs(m_swivel[s]) > 1e-6f))
                        continue;
                    const math::Quat r = rotS(m_swivel[s]);
                    const math::Quat qa = math::QuatNormalize(math::QuatMul(r, ga.q));
                    const math::Quat qb = math::QuatNormalize(math::QuatMul(r, gb.q));
                    const math::Quat la = math::QuatNormalize(math::QuatMul(math::QuatInv(gp.q), qa));
                    const math::Quat lc = math::QuatNormalize(math::QuatMul(math::QuatInv(qb), gc.q));
                    if (!math::IsFinite(la) || !math::IsFinite(lc))
                    {
                        ++m_diag.nonFinite;
                        m_swivel[s] = 0.0f;
                        continue;
                    }
                    m_out[size_t(m_chain[s][0])].q = la;
                    m_out[size_t(m_chain[s][2])].q = lc;
                    m_diag.swivel[s] = m_swivel[s];
                    any = true;
                    continue;
                }
                if (!(ol > 2e-3f) || !(pl > 1e-2f) || m_swivelOff)
                    continue; // straight leg or leg along +y: no defined swivel
                off = off * (1.0f / ol);
                pole = pole * (1.0f / pl);
                float ang = std::atan2(math::Dot(axis, math::Cross(off, pole)), math::Dot(off, pole)) * m_cfg.swivelWeight;
                ang = std::fmax(-m_cfg.maxSwivel, std::fmin(m_cfg.maxSwivel, ang));
                // never knock-kneed: the knee stays at least kneeMinHalf from the midline and not inside its own ankle
                const float sgn = s == 0 ? -1.0f : 1.0f;
                const float kneeFloor = std::fmax(m_cfg.kneeMinHalf, sgn * (gc.p.x - mid));
                auto rot = [&](float aAng) {
                    const float hh = 0.5f * aAng;
                    return math::Quat{axis.x * std::sin(hh), axis.y * std::sin(hh), axis.z * std::sin(hh), std::cos(hh)};
                };
                {
                    float scale = 1.0f;
                    for (int it = 0; it < 4; ++it, scale -= 0.25f)
                    {
                        const math::Vec3 kb = ga.p + math::QuatRotate(rot(ang * scale), gb.p - ga.p);
                        if (sgn * (kb.x - mid) >= kneeFloor || sgn * (gb.p.x - mid) < kneeFloor)
                            break; // ok (or the animated knee is already inside the floor: only reduce further)
                    }
                    ang *= std::fmax(scale, 0.0f);
                }
                if (!(std::fabs(ang) > 1e-4f))
                    continue;
                const math::Quat r = rot(ang);
                const math::Quat qa = math::QuatNormalize(math::QuatMul(r, ga.q));
                const math::Quat qb = math::QuatNormalize(math::QuatMul(r, gb.q));
                const math::Quat la = math::QuatNormalize(math::QuatMul(math::QuatInv(gp.q), qa));
                const math::Quat lc = math::QuatNormalize(math::QuatMul(math::QuatInv(qb), gc.q));
                if (!math::IsFinite(la) || !math::IsFinite(lc))
                {
                    ++m_diag.nonFinite;
                    continue;
                }
                m_out[size_t(m_chain[s][0])].q = la; // shin local unchanged (rotated together with the thigh)
                m_out[size_t(m_chain[s][2])].q = lc;
                m_diag.swivel[s] = ang;
                any = true;
            }
        if (any)
            ++m_diag.activeUpdates;
    }

private:
    // Two-bone IK: hip a fixed, ankle onto t, knee on the side of the ORIGINAL knee relative to the new hip-target line
    // (pole = original knee), so a lateral ankle push bends the leg instead of swinging the knee outward (0.3.7 rigid
    // rotation made the knees bow outward in the pad slow walk). Model-space rotations of thigh / shin out.
    static bool SolveKeepKnee(const BoneTransform& ga, const BoneTransform& gb, const BoneTransform& gc, math::Vec3 t,
                              math::Quat& aQa, math::Quat& aQb)
    {
        const math::Vec3 a = ga.p, b = gb.p, c = gc.p;
        const math::Vec3 ab = b - a, bc = c - b;
        const float lab = std::sqrt(math::Dot(ab, ab)), lbc = std::sqrt(math::Dot(bc, bc));
        const math::Vec3 at = t - a;
        float lt = std::sqrt(math::Dot(at, at));
        if (!(lab > 1e-4f) || !(lbc > 1e-4f) || !(lt > 1e-4f))
            return false;
        const float reach = 0.999f * (lab + lbc);
        const float lac = std::sqrt(math::Dot(c - a, c - a));
        lt = std::fmin(lt, std::fmax(reach, lac));
        const math::Vec3 atn = at * (1.0f / std::sqrt(math::Dot(at, at)));
        math::Vec3 pole = ab - atn * math::Dot(ab, atn);
        const float pl = std::sqrt(math::Dot(pole, pole));
        if (!(pl > 1e-5f))
            return false; // straight leg: keep the rigid rotation
        pole = pole * (1.0f / pl);
        const float cosA = std::fmin(1.0f, std::fmax(-1.0f, (lab * lab + lt * lt - lbc * lbc) / (2.0f * lab * lt)));
        const float sinA = std::sqrt(std::fmax(0.0f, 1.0f - cosA * cosA));
        const math::Vec3 nb = a + atn * (cosA * lab) + pole * (sinA * lab);
        const math::Vec3 nc = a + atn * lt;
        const math::Quat ra = FromTo(ab, nb - a);
        const math::Quat rb = FromTo(math::QuatRotate(ra, bc), nc - nb);
        aQa = math::QuatNormalize(math::QuatMul(ra, ga.q));
        aQb = math::QuatNormalize(math::QuatMul(rb, math::QuatMul(ra, gb.q)));
        return math::IsFinite(aQa) && math::IsFinite(aQb);
    }

    static BoneTransform Mul(const BoneTransform& aParent, const BoneTransform& aLocal)
    {
        return BoneTransform{aParent.p + math::QuatRotate(aParent.q, aLocal.p), math::QuatMul(aParent.q, aLocal.q)};
    }
    static math::Quat FromTo(math::Vec3 u, math::Vec3 v)
    {
        const float lu = std::sqrt(math::Dot(u, u)), lv = std::sqrt(math::Dot(v, v));
        if (!(lu > 1e-6f) || !(lv > 1e-6f))
            return math::Quat{};
        const math::Vec3 un = u * (1.0f / lu), vn = v * (1.0f / lv);
        const float dot = math::Dot(un, vn);
        if (dot < -0.9999f)
            return math::Quat{};
        const math::Vec3 cr = math::Cross(un, vn);
        return math::QuatNormalize(math::Quat{cr.x, cr.y, cr.z, 1.0f + dot});
    }
    BoneTransform Model(int32_t aBone) const
    {
        int32_t path[64];
        int32_t n = 0;
        for (int32_t b = aBone; b >= 0 && n < 64; b = m_parents[size_t(b)])
            path[n++] = b;
        BoneTransform g{};
        for (int32_t i = n - 1; i >= 0; --i)
            g = Mul(g, m_out[size_t(path[i])]);
        return g;
    }

    bool m_ok = false;
    StanceGuardConfig m_cfg{};
    std::vector<int32_t> m_parents;
    std::vector<BoneTransform> m_out;
    int32_t m_chain[2][3] = {{-1, -1, -1}, {-1, -1, -1}};
    StanceGuardDiagnostics m_diag{};
    float m_held[2] = {0.0f, 0.0f};
    float m_swivel[2] = {0.0f, 0.0f};
    float m_heldVel[2] = {0.0f, 0.0f}; // 0.3.9 push spring velocity
    bool m_swivelOff = false;          // batch 2 knee fix (SetSwivelSuppressed)
    bool m_plantedHold = false;        // batch 5 knee fix bit 4 (SetPlantedHold)
    float m_speed = 0.0f;              // 0.3.9 capsule speed (SetSpeed) // 0.3.9 filtered knee swivel per leg (rad) // 0.3.7 swing mode: current push per foot (m)
};
} // namespace utpp::mm

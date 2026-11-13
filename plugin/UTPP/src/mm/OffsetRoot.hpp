// UTPP motion matching runtime - offset root (Phase 5.3, motion "Offset Root Bone" style). Header-only, C++20,
// deterministic, no allocations in Update()/Apply() after Init().
//
// Problem: the game capsule (entity transform) accelerates/turns on its own (vanilla movement), while the MM pose is
// published in the character frame, i.e. glued to the capsule. When the capsule runs ahead of the clip's root motion
// (starts: capsule ~0.1 s to run speed, motion Run_Start ~0.5-0.7 s) the body slides with the capsule and the foot lock
// has to correct 20-25 cm.
//
// Offset root: the VISUAL root moves with the animation root motion and is pulled back to the capsule by a critically
// damped spring. State, expressed in the CURRENT capsule character frame (x right, y forward, metres; yaw CCW rad):
//   offset o = visual root - capsule, yaw offset psi = visual yaw - capsule yaw.
// Per update (real dt):
//   anim step a  = animation root velocity (real time, visual frame) * dt, rotated by psi into the capsule frame;
//   capsule step c = capsule world velocity * dt in the CURRENT capsule frame (the caller's WorldToCharacter);
//   o' = Rot2(-dyawCapsule) * (o + a) - c  (world-fixed visual root while the capsule turns),
//   psi' = psi + animYawStep - dyawCapsule;
//   then both decay towards 0 with a critically damped spring (translation / rotation halflife), and are clamped to
//   maxDistance / maxYaw (outward spring velocity removed at the clamp).
// Steady state with a constant speed mismatch u: |o| ~= u * halflife / ln2 (bounded by maxDistance).
// Resets (offset 0): Reset() (gate weight 0, teleport, stale feed, non-finite), capsule step > teleportDistance.
// While the MM gate is closed (fading out) the halflives are replaced by gateHalflife, so the body returns to the
// capsule before the game pose takes over.
//
// Apply(): bones >= firstBodyBone whose parent is below firstBodyBone (Hips: child of Root) are transformed by
// T(o) * Rz(psi) in the character frame. The foot locker runs on the offset pose, so the locked toes stay planted in
// the world under the VISUAL body (its character->world mapping is the capsule; the pose carries the offset).
#pragma once

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "../BaseDatabase.hpp"
#include "Inertializer.hpp"
#include "MMMath.hpp"

namespace utpp::mm
{
struct OffsetRootConfig
{
    float translationHalflife = 0.25f; // s, spring towards the capsule
    float rotationHalflife = 0.20f;    // s
    float maxDistance = 0.25f;         // m, |offset| clamp
    float maxYaw = 0.26179939f;        // rad (15 deg), |psi| clamp
    float gateHalflife = 0.05f;        // s, while the MM gate is closed
    float teleportDistance = 0.5f;     // m, capsule step per update above this -> reset
    float maxDt = 0.25f;               // s, larger steps -> reset (feed gap)
    // 0.3.4: an offset held at the clamp means a persistent speed mismatch (rate / speed map), not a transient. After
    // saturatedTime at >= 98 % of maxDistance the translation halflife drops to saturatedHalflife until |offset| < 50 %.
    // saturatedTime <= 0 disables the guard (0.3.3 behaviour).
    float saturatedTime = 0.0f;        // s (driver default 0.3)
    float saturatedHalflife = 0.10f;   // s
    // 0.5.9 catch-up (UTPPSetMMOffsetCatchup; false = 0.5.8): motion-like convergence instead of sitting at the clamp. Above
    // softDistance the excess decays with catchupHalflife (the hard clamp maxDistance stays as a safety net); while the
    // offset stays above restDistance the translation half-life shrinks linearly from translationHalflife to
    // catchupHalflife over catchupTime (the visual body converges to the capsule within ~catchupTime + 3 catchupHalflife).
    bool catchup = false;
    float softDistance = 0.15f;    // m
    float restDistance = 0.05f;    // m
    float catchupTime = 0.6f;      // s
    float catchupHalflife = 0.06f; // s
};

struct OffsetRootDiagnostics
{
    float x = 0.0f, y = 0.0f, yaw = 0.0f; // current offset (character frame, m / rad)
    float maxDistance = 0.0f;             // max |offset| since the last ResetDiagnostics
    float maxYaw = 0.0f;
    uint64_t updates = 0;
    uint64_t clamps = 0;
    uint64_t yawClamps = 0;
    uint64_t resets = 0;
    uint64_t teleports = 0;
    uint64_t nonFinite = 0;
    float saturatedSeconds = 0.0f;  // total time at >= 98 % of maxDistance
    uint64_t fastDecays = 0;        // saturation guard activations
    // 0.5.9 catch-up
    float softSeconds = 0.0f;       // total time above softDistance (catch-up on)
    float farTime = 0.0f;           // current time above restDistance
    float farTimeMax = 0.0f;        // max farTime since the last ResetDiagnostics
    uint64_t softDecays = 0;        // updates with the soft-limit decay applied
};

class OffsetRoot
{
public:
    bool Init(const Database& aDb, uint32_t aFirstBodyBone, const OffsetRootConfig& aCfg = {})
    {
        m_ok = false;
        if (!aDb.Loaded() || !Valid(aCfg) || aDb.NumBones() <= aFirstBodyBone)
            return false;
        m_cfg = aCfg;
        m_nb = aDb.NumBones();
        m_out.assign(m_nb, BoneTransform{});
        m_rootChild.assign(m_nb, 0);
        uint32_t children = 0;
        for (uint32_t b = aFirstBodyBone; b < m_nb; ++b)
        {
            const int32_t p = aDb.BoneParent(b);
            if (p < static_cast<int32_t>(aFirstBodyBone))
            {
                m_rootChild[b] = 1;
                ++children;
            }
        }
        if (children == 0)
            return false;
        m_first = aFirstBodyBone;
        Reset();
        m_diag = OffsetRootDiagnostics{};
        m_ok = true;
        return true;
    }

    bool Ready() const { return m_ok; }
    bool SetConfig(const OffsetRootConfig& aCfg)
    {
        if (!Valid(aCfg))
            return false;
        m_cfg = aCfg;
        return true;
    }
    const OffsetRootConfig& Config() const { return m_cfg; }

    void Reset()
    {
        m_provided = false;
        m_x = m_y = m_vx = m_vy = m_yaw = m_vyaw = 0.0f;
        m_satTimer = 0.0f;
        m_fast = false;
        m_diag.farTime = 0.0f;
        ++m_diag.resets;
        Publish();
    }
    void ResetDiagnostics()
    {
        m_diag.maxDistance = m_diag.maxYaw = 0.0f;
        m_diag.saturatedSeconds = 0.0f;
        m_diag.softSeconds = 0.0f;
        m_diag.farTimeMax = 0.0f;
    }

    // aDt real s; aCapVel capsule velocity in the capsule character frame (m/s); aCapYawStep capsule yaw change over
    // this update (rad); aAnimVel animation root velocity in REAL time in the visual (animation) frame (m/s);
    // aAnimYawRate animation root yaw rate (rad/s real time); aGateOpen false -> fast return (gateHalflife).
    void Update(float aDt, const float aCapVel[2], float aCapYawStep, const float aAnimVel[2], float aAnimYawRate,
                bool aGateOpen)
    {
        if (!m_ok)
            return;
        // A provided MOTION frame has its own owner. Legacy resumes from zero;
        // the existing final-pose handover performs the visible continuity blend.
        if (m_provided)
            Reset();
        ++m_diag.updates;
        if (!(std::isfinite(aDt) && aDt > 0.0f))
            return;
        if (aDt > m_cfg.maxDt)
        {
            Reset();
            return;
        }
        const float dt = aDt;
        const float cvx = Fin(aCapVel[0]), cvy = Fin(aCapVel[1]);
        const float cx = cvx * dt, cy = cvy * dt;
        if (std::sqrt(cx * cx + cy * cy) > m_cfg.teleportDistance)
        {
            ++m_diag.teleports;
            Reset();
            return;
        }
        const float dyc = math::WrapAngle(Fin(aCapYawStep));
        float ax = 0.0f, ay = 0.0f;
        math::Rot2(m_yaw, Fin(aAnimVel[0]) * dt, Fin(aAnimVel[1]) * dt, ax, ay);
        float nx = 0.0f, ny = 0.0f;
        math::Rot2(-dyc, m_x + ax, m_y + ay, nx, ny);
        // spring velocity is expressed in the same (rotating) frame
        float nvx = 0.0f, nvy = 0.0f;
        math::Rot2(-dyc, m_vx, m_vy, nvx, nvy);
        // Existing offset/animation step belong to the previous capsule frame.
        // Capsule velocity was already rotated into the current frame by the
        // caller: subtract it after transport, without a second yaw rotation.
        m_x = nx - cx;
        m_y = ny - cy;
        m_vx = nvx;
        m_vy = nvy;
        m_yaw = math::WrapAngle(m_yaw + Fin(aAnimYawRate) * dt - dyc);

        float ht = aGateOpen ? m_cfg.translationHalflife : m_cfg.gateHalflife;
        if (m_fast)
            ht = std::fmin(ht, m_cfg.saturatedHalflife);
        if (m_cfg.catchup && aGateOpen)
        {
            // 0.5.9: the longer the body is away from the capsule, the faster it converges (<= catchupTime)
            const float d0 = std::sqrt(m_x * m_x + m_y * m_y);
            m_diag.farTime = d0 > m_cfg.restDistance ? m_diag.farTime + dt : 0.0f;
            m_diag.farTimeMax = std::fmax(m_diag.farTimeMax, m_diag.farTime);
            const float u = std::fmin(1.0f, m_diag.farTime / m_cfg.catchupTime);
            ht = std::fmin(ht, ht + (m_cfg.catchupHalflife - ht) * u);
        }
        const float hr = aGateOpen ? m_cfg.rotationHalflife : m_cfg.gateHalflife;
        math::Vec3 p{m_x, m_y, 0.0f}, v{m_vx, m_vy, 0.0f};
        math::DecaySpringDamperExact(p, v, ht, dt);
        m_x = p.x;
        m_y = p.y;
        m_vx = v.x;
        m_vy = v.y;
        math::SimpleSpringDamperExact(m_yaw, m_vyaw, m_yawGoal, hr, dt); // goal 0 except 0.3.7 heading lead

        if (m_cfg.catchup)
        {
            // 0.5.9 soft limit: the excess over softDistance decays with catchupHalflife (no pinning at the clamp)
            const float ds = std::sqrt(m_x * m_x + m_y * m_y);
            if (ds > m_cfg.softDistance && ds > 0.0f)
            {
                const float ex = (ds - m_cfg.softDistance) * std::exp2(-dt / m_cfg.catchupHalflife);
                const float k = (m_cfg.softDistance + ex) / ds;
                m_x *= k;
                m_y *= k;
                const float ux = m_x / (ds * k), uy = m_y / (ds * k);
                const float radial = m_vx * ux + m_vy * uy;
                if (radial > 0.0f)
                {
                    m_vx -= radial * ux;
                    m_vy -= radial * uy;
                }
                m_diag.softSeconds += dt;
                ++m_diag.softDecays;
            }
        }
        const float d = std::sqrt(m_x * m_x + m_y * m_y);
        if (d > m_cfg.maxDistance && d > 0.0f)
        {
            const float k = m_cfg.maxDistance / d;
            m_x *= k;
            m_y *= k;
            const float ux = m_x / m_cfg.maxDistance, uy = m_y / m_cfg.maxDistance;
            const float radial = m_vx * ux + m_vy * uy;
            if (radial > 0.0f)
            {
                m_vx -= radial * ux;
                m_vy -= radial * uy;
            }
            ++m_diag.clamps;
        }
        {
            const float dd = std::sqrt(m_x * m_x + m_y * m_y);
            if (m_cfg.maxDistance > 0.0f && dd >= 0.98f * m_cfg.maxDistance)
            {
                m_diag.saturatedSeconds += dt;
                m_satTimer += dt;
                if (!m_fast && m_cfg.saturatedTime > 0.0f && m_satTimer > m_cfg.saturatedTime)
                {
                    m_fast = true;
                    ++m_diag.fastDecays;
                }
            }
            else
            {
                m_satTimer = 0.0f;
                if (dd < 0.5f * m_cfg.maxDistance)
                    m_fast = false;
            }
        }
        if (std::fabs(m_yaw) > m_cfg.maxYaw)
        {
            m_yaw = std::copysign(m_cfg.maxYaw, m_yaw);
            if (m_vyaw * m_yaw > 0.0f)
                m_vyaw = 0.0f;
            ++m_diag.yawClamps;
        }
        if (!std::isfinite(m_x) || !std::isfinite(m_y) || !std::isfinite(m_vx) || !std::isfinite(m_vy) ||
            !std::isfinite(m_yaw) || !std::isfinite(m_vyaw))
        {
            ++m_diag.nonFinite;
            Reset();
            return;
        }
        Publish();
    }

    // Mirror a precomputed same-feed transform for Apply/diagnostics only. No
    // integration.
    bool AdoptFrame(float x, float y, float yaw)
    {
        if (!m_ok || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || std::hypot(x, y) > 1.f)
            return false;
        m_x = x;
        m_y = y;
        m_yaw = math::WrapAngle(yaw);
        m_vx = m_vy = m_vyaw = 0;
        m_provided = true;
        ++m_diag.updates;
        if (std::hypot(x, y) >= .299999f)
            ++m_diag.clamps;
        Publish();
        return true;
    }
    // aIn = matcher output (NumBones entries); result in Output().
    void Apply(std::span<const BoneTransform> aIn)
    {
        if (!m_ok || aIn.size() != m_nb)
            return;
        const math::Quat qz{0.0f, 0.0f, std::sin(0.5f * m_yaw), std::cos(0.5f * m_yaw)};
        const bool identity = m_x == 0.0f && m_y == 0.0f && m_yaw == 0.0f;
        for (uint32_t b = 0; b < m_nb; ++b)
        {
            if (identity || !m_rootChild[b])
            {
                m_out[b] = aIn[b];
                continue;
            }
            const math::Vec3 r = math::QuatRotate(qz, aIn[b].p);
            m_out[b].p = {r.x + m_x, r.y + m_y, r.z};
            m_out[b].q = math::QuatMul(qz, aIn[b].q);
        }
    }

    std::span<const BoneTransform> Output() const { return m_out; }
    const OffsetRootDiagnostics& Diagnostics() const { return m_diag; }
    float X() const { return m_x; }
    float Y() const { return m_y; }
    float Yaw() const { return m_yaw; }
    // 0.3.7: yaw the visual body springs to (rad, clamped to +-maxYaw; default 0 = 0.3.6). Used to lean the visual body
    // towards the capsule velocity heading while the entity is still turning (feet in contact move along the travel).
    void SetYawGoal(float aGoal)
    {
        const float g = std::isfinite(aGoal) ? aGoal : 0.0f;
        m_yawGoal = std::fmax(-m_cfg.maxYaw, std::fmin(m_cfg.maxYaw, g));
    }
    float YawGoal() const { return m_yawGoal; }

private:
    static float Fin(float v) { return std::isfinite(v) ? v : 0.0f; }
    static bool Valid(const OffsetRootConfig& c)
    {
        auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
        return pos(c.translationHalflife) && pos(c.rotationHalflife) && std::isfinite(c.maxDistance) &&
               c.maxDistance >= 0.0f && c.maxDistance <= 1.0f && std::isfinite(c.maxYaw) && c.maxYaw >= 0.0f &&
               c.maxYaw <= 1.6f && pos(c.gateHalflife) && pos(c.teleportDistance) && pos(c.maxDt) &&
               std::isfinite(c.saturatedTime) && pos(c.saturatedHalflife) && std::isfinite(c.softDistance) &&
               c.softDistance >= 0.0f && std::isfinite(c.restDistance) && c.restDistance >= 0.0f && pos(c.catchupTime) &&
               pos(c.catchupHalflife);
    }
    void Publish()
    {
        m_diag.x = m_x;
        m_diag.y = m_y;
        m_diag.yaw = m_yaw;
        const float d = std::sqrt(m_x * m_x + m_y * m_y);
        if (d > m_diag.maxDistance)
            m_diag.maxDistance = d;
        if (std::fabs(m_yaw) > m_diag.maxYaw)
            m_diag.maxYaw = std::fabs(m_yaw);
    }

    bool m_ok = false;
    bool m_provided = false;
    OffsetRootConfig m_cfg{};
    uint32_t m_nb = 0, m_first = 2;
    std::vector<BoneTransform> m_out;
    std::vector<uint8_t> m_rootChild;
    float m_x = 0.0f, m_y = 0.0f, m_vx = 0.0f, m_vy = 0.0f, m_yaw = 0.0f, m_vyaw = 0.0f;
    float m_yawGoal = 0.0f; // 0.3.7
    float m_satTimer = 0.0f;
    bool m_fast = false;
    OffsetRootDiagnostics m_diag{};
};
} // namespace utpp::mm

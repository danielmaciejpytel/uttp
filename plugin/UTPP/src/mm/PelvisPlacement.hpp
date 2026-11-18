// UTPP motion matching runtime - pelvis placement stage of the leg tail. Header-only, C++20, deterministic, allocation-free.
//
// Model space (character frame: x right, y forward, z up, metres). Given, per leg, the animated hip and ankle and the ankle target
// the foot lock wants (the animated ankle for a free leg), it returns a pelvis translation offset that lets every leg reach its
// target:
//   1. horizontal rebalancing: rebalance x the mean planar (target - animated ankle) of both legs, capped at maxHorizontal;
//   2. per leg, from the rebalanced hip:
//      - max offset: the highest pelvis height at which the leg reaches its target at reach x leg length. The target used for this
//        bound is moved towards the animated ankle by heelLift x min(distance, foot length) (planar), so a small part of a stretch
//        is left to the heel instead of the pelvis (heelLift 0 = the exact reach bound);
//      - desired offset: the pelvis height at which the hip-target distance equals the animated hip-ankle distance;
//   3. all legs: the offset lies between the lowest desired offset and the average one, weighted towards the reach bound, and never
//      above the lowest max offset (every leg reachable); then clamped to [-maxDrop, maxRaise].
// The offset follows its target through a critically damped spring (stiffness k: angular frequency sqrt(k)), integrated in closed
// form, so the response does not depend on the update rate. While placement is not allowed (air, vehicle, scripted scenes,
// traversal) the target is zero and the spring brings the pelvis back (fade); Reset() drops it at once.
#pragma once

#include <cmath>

#include "MMMath.hpp"

namespace utpp::mm
{

struct PelvisPlacementConfig
{
    float stiffness = 100.0f;    // spring stiffness (critically damped)
    float rebalance = 0.3f;      // horizontal rebalancing weight
    float maxHorizontal = 0.10f; // m, horizontal offset bound
    float heelLift = 0.5f;       // part of min(foot length, target - animated ankle) left to the heel in the reach bound
    float reach = 0.995f;        // fraction of the leg length a leg may reach at the pelvis offset
    float maxDrop = 0.25f;       // m, lowest pelvis offset (safety bound)
    float maxRaise = 0.0f;       // m, highest pelvis offset (0 = the pelvis only drops)
};

struct PelvisLeg
{
    math::Vec3 hip{};      // animated hip (UpLeg) position
    math::Vec3 ankle{};    // animated ankle position
    math::Vec3 target{};   // ankle target of this update (animated ankle for a free leg)
    float length = 0.0f;   // thigh + shin
    float footLength = 0.0f; // ankle - toe distance
};

class PelvisPlacement
{
public:
    static bool ValidConfig(const PelvisPlacementConfig& c)
    {
        auto nonneg = [](float v) { return std::isfinite(v) && v >= 0.0f; };
        return nonneg(c.stiffness) && c.stiffness > 0.0f && nonneg(c.rebalance) && c.rebalance <= 1.0f && nonneg(c.maxHorizontal) &&
               nonneg(c.heelLift) && c.heelLift <= 1.0f && std::isfinite(c.reach) && c.reach > 0.5f && c.reach <= 1.0f &&
               nonneg(c.maxDrop) && nonneg(c.maxRaise);
    }

    // Instantaneous target offset for two legs (no filtering). Non-finite input or a degenerate leg gives zero.
    static math::Vec3 TargetOffset(const PelvisPlacementConfig& c, const PelvisLeg (&aLegs)[2])
    {
        for (const PelvisLeg& l : aLegs)
            if (!math::IsFinite(l.hip) || !math::IsFinite(l.ankle) || !math::IsFinite(l.target) || !(l.length > 1e-3f) ||
                !std::isfinite(l.footLength))
                return math::Vec3{};
        // 1. horizontal rebalancing
        float hx = 0.0f, hy = 0.0f;
        for (const PelvisLeg& l : aLegs)
        {
            hx += 0.5f * (l.target.x - l.ankle.x);
            hy += 0.5f * (l.target.y - l.ankle.y);
        }
        hx *= c.rebalance;
        hy *= c.rebalance;
        const float hl = std::sqrt(hx * hx + hy * hy);
        if (hl > c.maxHorizontal && hl > 0.0f)
        {
            hx *= c.maxHorizontal / hl;
            hy *= c.maxHorizontal / hl;
        }
        // 2. per-leg range
        float desiredMin = 1e9f, desiredAvg = 0.0f, maxMin = 1e9f;
        for (const PelvisLeg& l : aLegs)
        {
            const float hipX = l.hip.x + hx, hipY = l.hip.y + hy;
            // reach bound, target moved towards the animated ankle by the heel share
            float tx = l.target.x, ty = l.target.y;
            {
                const float dx = l.ankle.x - tx, dy = l.ankle.y - ty;
                const float d = std::sqrt(dx * dx + dy * dy);
                if (d > 1e-6f)
                {
                    const float s = std::fmin(d, std::fmax(0.0f, l.footLength)) * c.heelLift / d;
                    tx += dx * s;
                    ty += dy * s;
                }
            }
            maxMin = std::fmin(maxMin, HeightOnSphere(hipX - tx, hipY - ty, l.target.z, c.reach * l.length) - l.hip.z);
            const math::Vec3 ext = l.ankle - l.hip;
            const float desiredLen = std::sqrt(math::Dot(ext, ext));
            const float desired = HeightOnSphere(hipX - l.target.x, hipY - l.target.y, l.target.z, desiredLen) - l.hip.z;
            desiredMin = std::fmin(desiredMin, desired);
            desiredAvg += 0.5f * desired;
        }
        // 3. combine (between the lowest and the average desired offset, pulled towards the reach bound), never above the reach bound
        const float minToAvg = desiredAvg - desiredMin;
        const float minToMax = maxMin - desiredMin;
        const float div = minToAvg + minToMax;
        float z = std::fabs(div) > 1e-6f ? desiredMin + (minToAvg * minToMax) / div : desiredMin;
        z = std::fmin(z, maxMin);
        z = std::fmin(std::fmax(z, -c.maxDrop), c.maxRaise);
        const math::Vec3 r{hx, hy, z};
        return math::IsFinite(r) ? r : math::Vec3{};
    }

    void SetConfig(const PelvisPlacementConfig& aCfg)
    {
        if (ValidConfig(aCfg))
            m_cfg = aCfg;
    }
    const PelvisPlacementConfig& Config() const { return m_cfg; }

    // One update: aAllowed false -> target zero (fade back). Returns the filtered offset.
    math::Vec3 Update(float aDt, const PelvisLeg (&aLegs)[2], bool aAllowed)
    {
        m_target = aAllowed ? TargetOffset(m_cfg, aLegs) : math::Vec3{};
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        const float w = std::sqrt(m_cfg.stiffness);
        const float e = std::exp(-w * dt);
        // closed-form critically damped spring towards the (constant over the step) target
        const math::Vec3 y = m_offset - m_target;
        const math::Vec3 j = m_vel + y * w;
        m_offset = m_target + (y + j * dt) * e;
        m_vel = (m_vel - j * (w * dt)) * e;
        if (!math::IsFinite(m_offset) || !math::IsFinite(m_vel))
            Reset();
        // the spring may overshoot the bounds only within the bounds of the target range
        m_offset.z = std::fmin(std::fmax(m_offset.z, -m_cfg.maxDrop), m_cfg.maxRaise);
        return m_offset;
    }

    void Reset()
    {
        m_offset = m_vel = m_target = math::Vec3{};
    }
    math::Vec3 Offset() const { return m_offset; }
    math::Vec3 Target() const { return m_target; }

private:
    // Height (z) of the point above the target on a sphere of radius aR around the target, at planar distance (aDx, aDy) from it.
    // Beyond the radius: the target height (the hip cannot reach; the drop is bounded by maxDrop).
    static float HeightOnSphere(float aDx, float aDy, float aTargetZ, float aR)
    {
        const float d2 = aDx * aDx + aDy * aDy;
        const float r2 = aR * aR;
        return d2 < r2 ? aTargetZ + std::sqrt(r2 - d2) : aTargetZ;
    }

    PelvisPlacementConfig m_cfg{};
    math::Vec3 m_offset{}, m_vel{}, m_target{};
};

} // namespace utpp::mm

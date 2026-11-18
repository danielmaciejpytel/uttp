// Live pose history for the matcher (pure logic, header-only).
//
// WHY. Without a caller history the pose channels of a search come from the continuing pose (or the one-sample last-pose fallback,
// ChooserMatcher BuildQuery aAllowLastPose); after the leg tail moved the feet, that is not the pose the player sees, so a stop or a
// start could pick the clip whose planted foot is the other one. This ring keeps the pose that was really published (after the
// foot locker), so the query reads the shown feet and their velocities.
//
// CONVENTIONS (same as ExitPoseHistory.hpp): the pushed pose is the database local pose (database bone order) as published; the ring
// stores the Root-space transforms (forward kinematics from aFirstBone, Root / Trajectory identity) of the tracked bones
// (TrackedHistoryBones). The published pose carries the visual root offset on the root's children (OffsetRoot::Apply: rotate by the
// offset yaw about +Z, then translate by x / y); Push takes that offset and removes it, so the history is in the animation's Root space
// like the database features. Root samples are NOT stored: Build takes them from the current
// query trajectory at the sample times (its past part is the capsule history the matcher already uses), so pose and root share one
// time base. Sample times: the newest push is the pose of the previous feed, t = -aDtNow; older ones add the feed dt of every later
// push. Only samples inside kLiveHistoryWindow are kept (at most kHistMaxSamples); Build needs at least two (a velocity needs two).
// Reset on every discontinuity the caller knows (re-entry, teleport, database / body change); Push resets itself when the database
// or its bone count changes.
#pragma once

#include <cmath>
#include <cstdint>

#include "ExitPoseHistory.hpp"

namespace utpp::mm::motion
{
inline constexpr float kLiveHistoryWindow = 0.4f; // s: the pose channels sample up to 0.3 s back, plus one velocity step

class LivePoseHistory
{
public:
    void Reset()
    {
        m_n = 0;
        m_nb = 0;
        m_db = nullptr;
        m_dbBones = 0;
    }

    uint32_t NumSamples() const { return m_n; }

    // aPose: aNumBones transforms with members p (Vec3) and q (Quat), database local space. aDt: feed time since the previous push.
    // aOffX / aOffY (m) / aOffYaw (rad): the visual root offset applied to the pose (0 = none). Returns false (nothing stored) when the
    // pose does not belong to aDb or a tracked chain has a non-finite bone.
    template <class Transform>
    bool Push(const MotionDatabase& aDb, uint32_t aFirstBone, const Transform* aPose, uint32_t aNumBones, float aDt, float aOffX = 0.0f,
              float aOffY = 0.0f, float aOffYaw = 0.0f)
    {
        if (!std::isfinite(aOffX) || !std::isfinite(aOffY) || !std::isfinite(aOffYaw))
            aOffX = aOffY = aOffYaw = 0.0f;
        const uint32_t nb = aDb.NumBones();
        if (!aPose || nb == 0 || nb > kExitSeedMaxDbBones || aNumBones != nb || !(aDt >= 0.0f) || !std::isfinite(aDt))
        {
            Reset();
            return false;
        }
        if (m_db != &aDb || m_dbBones != nb)
        {
            Reset();
            m_db = &aDb;
            m_dbBones = nb;
            m_nb = TrackedHistoryBones(aDb, m_bone);
        }
        if (m_nb == 0)
            return false;
        math::Vec3 gp[kExitSeedMaxDbBones];
        math::Quat gq[kExitSeedMaxDbBones];
        bool ok[kExitSeedMaxDbBones];
        RootSpaceFk(
            aDb, aFirstBone,
            [&](uint32_t i, math::Vec3& p, math::Quat& q) {
                p = aPose[i].p;
                q = aPose[i].q;
                return true;
            },
            gp, gq, ok);
        for (uint32_t k = 0; k < m_nb; ++k)
            if (!ok[m_bone[k]])
            {
                Reset();
                return false;
            }
        // age the stored samples, drop the ones outside the window (oldest first) and make room for the new one
        uint32_t drop = 0;
        for (uint32_t i = 0; i < m_n; ++i)
        {
            m_age[i] += aDt;
            if (m_age[i] > kLiveHistoryWindow)
                drop = i + 1;
        }
        if (m_n - drop >= kHistMaxSamples)
            drop = m_n - kHistMaxSamples + 1;
        if (drop > 0)
        {
            for (uint32_t i = drop; i < m_n; ++i)
            {
                m_age[i - drop] = m_age[i];
                for (uint32_t k = 0; k < m_nb; ++k)
                {
                    m_gp[i - drop][k] = m_gp[i][k];
                    m_gq[i - drop][k] = m_gq[i][k];
                }
            }
            m_n -= drop;
        }
        m_age[m_n] = 0.0f;
        const math::Quat inv{0.0f, 0.0f, -std::sin(0.5f * aOffYaw), std::cos(0.5f * aOffYaw)};
        for (uint32_t k = 0; k < m_nb; ++k)
        {
            const math::Vec3 p = gp[m_bone[k]];
            m_gp[m_n][k] = math::QuatRotate(inv, math::Vec3{p.x - aOffX, p.y - aOffY, p.z});
            m_gq[m_n][k] = math::QuatNormalize(math::QuatMul(inv, gq[m_bone[k]]));
        }
        ++m_n;
        return true;
    }

    // aDtNow: feed time since the newest push (the dt of the search). aTraj / aNumTraj: the query trajectory (sorted by t, caller
    // space) the search uses. Returns false (aOut.numSamples 0) with fewer than two samples.
    bool Build(float aDtNow, const TrajSample* aTraj, uint32_t aNumTraj, PoseHistory& aOut) const
    {
        aOut.numSamples = 0;
        aOut.numBones = 0;
        if (m_n < 2 || !aTraj || aNumTraj == 0 || !(aDtNow >= 0.0f) || !std::isfinite(aDtNow))
            return false;
        aOut.numBones = m_nb;
        for (uint32_t k = 0; k < m_nb; ++k)
            aOut.bone[k] = m_bone[k];
        for (uint32_t i = 0; i < m_n; ++i)
        {
            const float t = -(m_age[i] + aDtNow);
            aOut.root[i] = TrajAt(aTraj, aNumTraj, t);
            for (uint32_t k = 0; k < m_nb; ++k)
            {
                aOut.gp[i][k] = m_gp[i][k];
                aOut.gq[i][k] = m_gq[i][k];
            }
        }
        aOut.numSamples = m_n;
        return true;
    }

    // Linear interpolation of the trajectory at time t (clamped at both ends; yaw along the shorter arc).
    static TrajSample TrajAt(const TrajSample* aTraj, uint32_t aNum, float t)
    {
        TrajSample r = aTraj[0];
        if (t <= aTraj[0].t || aNum == 1)
            r = aTraj[0];
        else if (t >= aTraj[aNum - 1].t)
            r = aTraj[aNum - 1];
        else
        {
            uint32_t i = 1;
            while (i < aNum && aTraj[i].t < t)
                ++i;
            const TrajSample& a = aTraj[i - 1];
            const TrajSample& b = aTraj[i];
            const float span = b.t - a.t;
            const float u = span > 0.0f ? (t - a.t) / span : 0.0f;
            r.x = a.x + (b.x - a.x) * u;
            r.y = a.y + (b.y - a.y) * u;
            r.z = a.z + (b.z - a.z) * u;
            r.yaw = a.yaw + std::remainder(b.yaw - a.yaw, 6.28318530717958647692f) * u;
        }
        r.t = t;
        return r;
    }

private:
    const MotionDatabase* m_db = nullptr;
    uint32_t m_dbBones = 0;
    uint32_t m_n = 0;  // samples, oldest first
    uint32_t m_nb = 0; // tracked bones
    int32_t m_bone[kHistMaxBones] = {};
    float m_age[kHistMaxSamples] = {};
    math::Vec3 m_gp[kHistMaxSamples][kHistMaxBones];
    math::Quat m_gq[kHistMaxSamples][kHistMaxBones];
};
} // namespace utpp::mm::motion

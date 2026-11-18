// UTPP vehicle-exit pose seed for the first v2 search (pure logic, header-only, unit-tested offline: tests/MMVehExitSeedTest.cpp).
//
// WHY. After a vehicle exit MotionDriver re-enters (b.matcher.Reset) and the first search used no PoseHistory, so every pose channel of
// the query was the database MEAN (ChooserMatcher.hpp header: "Initial / invalidated queries still use the mean") and the pose V really
// has at the end of the game's exit animation was ignored: the first pick was always the same idle frame whatever the exit pose was, and
// an exit-to-idle clip of the database could never win. The vehicle transition (VehicleTransition.hpp BeginExit) already holds that
// pose (the capture of the last ride / latched exit frame); this header turns it into the PoseHistory the matcher reads.
//
// CONVENTIONS (what ChooserMatcher::BuildQuery / HistoryFeature read, FORMAT.txt v2 "Feature values"):
//   * PoseHistory.bone[] holds DATABASE bone indices (SCHC bone / origin bone are compared with it); gp / gq are the Root-space
//     (model space without root motion) transforms of those bones = forward kinematics of the database local pose over the database
//     parents. Root (bone 0) and Trajectory (bone 1) are identity (the baker keeps the planar root motion in Trajectory, a sibling of
//     Hips, so it is in no chain).
//   * The tracked bones are every non-trajectory channel bone / origin bone of every schema of the database (at most
//     kHistMaxBones; Hips, LeftFoot, RightFoot and the RightHand of the slide-exit schema in the shipped bakes). A bone whose chain
//     has no pose is left out: its channels then keep the database mean (HistoryFeature returns false) exactly as without a history.
//   * ONE sample at t = 0 (root = the caller's trajectory origin, identity in the driver). Every pose channel samples at ts = to = 0
//     and the history clamps outside its samples, so the single sample is what BuildQuery sees for every time; Position / Heading
//     channels get the real pose, Velocity channels get 0 (the pose is held still: the end of an exit clip). After normalization that
//     is close to the database mean for the idle databases (feet at rest) and a true standstill query for the others.
#pragma once

#include <cmath>
#include <cstdint>

#include "MotionDatabase.hpp"
#include "MMMath.hpp"
#include "ChooserMatcher.hpp"

namespace utpp::mm::motion
{
inline constexpr uint32_t kExitSeedMaxDbBones = 128;

// The bones a pose history tracks: every non-trajectory channel bone / origin bone of every schema of the database (database bone
// indices, at most kHistMaxBones, first come first kept). Returns the count.
inline uint32_t TrackedHistoryBones(const MotionDatabase& aDb, int32_t (&aWant)[kHistMaxBones])
{
    const uint32_t nb = aDb.NumBones();
    uint32_t nWant = 0;
    const auto add = [&](int32_t b) {
        if (b < 0 || b >= static_cast<int32_t>(nb))
            return;
        for (uint32_t i = 0; i < nWant; ++i)
            if (aWant[i] == b)
                return;
        if (nWant < kHistMaxBones)
            aWant[nWant++] = b;
    };
    for (uint32_t s = 0; s < aDb.Header().numSchemas; ++s)
    {
        const SchemaRecord& S = aDb.Schema(s);
        for (uint32_t c = S.firstChannel; c < S.firstChannel + S.numChannels; ++c)
        {
            const ChannelRecord& ch = aDb.Channel(c);
            if (ch.flags & kChanTrajectory)
                continue;
            add(ch.bone);
            add(ch.originBone);
        }
    }
    return nWant;
}

// Root-space forward kinematics over the database parents (parent index < child index). aLocalAt as in BuildExitPoseHistory; bones
// below aFirstBone are identity. aGp / aGq / aOk hold aDb.NumBones() entries (<= kExitSeedMaxDbBones); aOk[i] = false where the chain
// of bone i has a bone without a pose.
template <class LocalAt>
inline void RootSpaceFk(const MotionDatabase& aDb, uint32_t aFirstBone, LocalAt&& aLocalAt, math::Vec3* aGp, math::Quat* aGq, bool* aOk)
{
    const uint32_t nb = aDb.NumBones();
    for (uint32_t i = 0; i < nb; ++i)
    {
        aOk[i] = false;
        const int32_t par = aDb.BoneParent(i);
        math::Vec3 lp{};
        math::Quat lq{};
        if (i >= aFirstBone)
        {
            if (!aLocalAt(i, lp, lq) || !math::IsFinite(lp) || !math::IsFinite(lq))
                continue;
            lq = math::QuatNormalize(lq);
        }
        if (par < 0)
        {
            aGp[i] = lp;
            aGq[i] = lq;
            aOk[i] = true;
        }
        else if (par < static_cast<int32_t>(i) && aOk[par])
        {
            aGq[i] = math::QuatNormalize(math::QuatMul(aGq[par], lq));
            aGp[i] = aGp[par] + math::QuatRotate(aGq[par], lp);
            aOk[i] = true;
        }
    }
}

// aLocalAt(uint32_t dbBone, math::Vec3& aP, math::Quat& aQ) -> bool: local (parent space) pose of database bone aBone, false = unknown.
// Called only for bones >= aFirstBone (Root / Trajectory are identity). Returns false (aOut.numSamples 0) when no tracked bone resolved.
template <class LocalAt>
inline bool BuildExitPoseHistory(const MotionDatabase& aDb, uint32_t aFirstBone, LocalAt&& aLocalAt, const TrajSample& aRoot,
                                 PoseHistory& aOut)
{
    aOut.numSamples = 0;
    aOut.numBones = 0;
    const uint32_t nb = aDb.NumBones();
    if (nb == 0 || nb > kExitSeedMaxDbBones)
        return false;
    int32_t want[kHistMaxBones];
    const uint32_t nWant = TrackedHistoryBones(aDb, want);
    if (nWant == 0)
        return false;
    math::Vec3 gp[kExitSeedMaxDbBones];
    math::Quat gq[kExitSeedMaxDbBones];
    bool ok[kExitSeedMaxDbBones];
    RootSpaceFk(aDb, aFirstBone, aLocalAt, gp, gq, ok);

    uint32_t k = 0;
    for (uint32_t i = 0; i < nWant; ++i)
    {
        const int32_t b = want[i];
        if (!ok[b])
            continue;
        aOut.bone[k] = b;
        aOut.gp[0][k] = gp[b];
        aOut.gq[0][k] = gq[b];
        ++k;
    }
    if (k == 0)
        return false;
    aOut.numBones = k;
    aOut.numSamples = 1;
    aOut.root[0] = aRoot;
    aOut.root[0].t = 0.0f;
    return true;
}
} // namespace utpp::mm::motion

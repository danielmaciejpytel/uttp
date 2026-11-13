// CP projection of motion OrientationWarping foot markers into the existing leg IK solver.
// Sampling uses the raw pose before Hips/root-half warp; Apply consumes blended
// pose afterward, matching the intended per-player sample-then-blend flow.
#pragma once

#include "MotionOrientationPose.hpp"
#include "LegIk.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace utpp::mm::motionFeet
{
using math::Quat;
using math::Vec3;

struct FootTarget
{
    Vec3 p{}; // CP Root-local/model-space ankle target, metres.
    Quat q{}; // CP Root-local/model-space ankle orientation.
};
struct Targets
{
    std::array<FootTarget, 2> feet{}; // left, right
    float warpWeight = 0.0f;          // 0 for identity; 1 for a nonzero sampled OW angle.
    bool valid = false;
    // Relative targets are corrections from the player's post-OW FK ankle,
    // expressed in the same Root/model frame as the pose blend. They must not
    // contain the player's absolute animated foot trajectory.
    bool relative = false;
};

struct FootResult
{
    legik::TwoBoneStatus status = legik::TwoBoneStatus::kBadLegLength;
    bool clamped = false;
    float targetError = 0.0f;
};
struct ApplyResult
{
    bool applied = false;
    std::array<FootResult, 2> feet{};
};

enum class InitError : uint8_t
{
    None,
    MissingBone,
    DuplicateBone,
    InvalidParent,
    InvalidChain
};

class Warp
{
  public:
    template <class Database> bool Init(const Database &db, InitError *error = nullptr)
    {
        m_ready = false;
        m_parents.clear();
        m_model.clear();
        m_work.clear();
        if (error)
            *error = InitError::None;
        if (db.NumBones() == 0 || db.NumBones() > 1024)
            return Fail(error, InitError::InvalidParent);

        m_parents.resize(db.NumBones(), -1);
        m_model.resize(db.NumBones());
        m_work.resize(db.NumBones());
        for (uint32_t i = 0; i < db.NumBones(); ++i)
        {
            const int32_t p = db.BoneParent(i);
            if (p >= static_cast<int32_t>(i) || p < -1)
                return Fail(error, InitError::InvalidParent);
            m_parents[i] = p;
        }

        static constexpr std::array<std::array<std::string_view, 3>, 2> names{
            {{{"LeftUpLeg", "LeftLeg", "LeftFoot"}}, {{"RightUpLeg", "RightLeg", "RightFoot"}}}};
        static constexpr std::array<std::string_view, 2> toes{"LeftToeBase", "RightToeBase"};
        for (int side = 0; side < 2; ++side)
        {
            for (int k = 0; k < 3; ++k)
                if (!FindUnique(db, names[side][k], m_chain[side][k], error))
                    return false;
            const int32_t upper = m_chain[side][0], lower = m_chain[side][1], foot = m_chain[side][2];
            if (m_parents[lower] != upper || m_parents[foot] != lower || m_parents[upper] < 0)
                return Fail(error, InitError::InvalidChain);
            m_parent[side] = m_parents[upper];
            m_toe[side] = -1;
            for (uint32_t i = 0; i < db.NumBones(); ++i)
                if (db.BoneName(i) == toes[side])
                {
                    if (m_toe[side] >= 0)
                        return Fail(error, InitError::DuplicateBone);
                    m_toe[side] = static_cast<int32_t>(i);
                }
            if (m_toe[side] >= 0 && !IsDescendant(m_toe[side], foot))
                return Fail(error, InitError::InvalidChain);
        }
        if (!m_poseMapper.Init(db))
            return Fail(error, InitError::InvalidChain);
        m_ready = true;
        return true;
    }

    bool MakeRelative(std::span<const BoneTransform> warpedPose, Targets &targets) noexcept
    {
        if (!m_ready || warpedPose.size() != m_parents.size() || !targets.valid || targets.relative)
            return false;
        for (const auto &t : warpedPose)
            if (!Valid(t))
                return false;
        BuildModel(warpedPose, m_model);
        for (int side = 0; side < 2; ++side)
        {
            auto &f = targets.feet[size_t(side)];
            if (!math::IsFinite(f.p) || !Unit(f.q))
                return false;
            const auto &base = m_model[size_t(m_chain[side][2])];
            if (targets.warpWeight == 0)
                f = {}; // Exact neutral, independent of FK roundoff/quat sign.
            else
            {
                f.p = f.p - base.p;
                f.q = math::QuatNormalize(math::QuatMul(f.q, math::QuatInv(base.q)));
            }
        }
        targets.relative = true;
        return true;
    }

    bool Ready() const noexcept
    {
        return m_ready;
    }

    // CP projection: rig_inspect shows motion ik_foot_root at origin with identity
    // basis. CP has no virtual IK bones, so project its marker from the raw
    // animated CP ankle transform in Root-local/model space.
    bool Sample(std::span<const BoneTransform> rawPose, float rootHalfAngle, Targets &out) noexcept
    {
        if (!m_ready || rawPose.size() != m_parents.size() || !std::isfinite(rootHalfAngle))
            return false;
        for (const auto &t : rawPose)
            if (!Valid(t))
                return false;
        BuildModel(rawPose, m_model);

        Targets candidate{};
        candidate.warpWeight = std::fabs(rootHalfAngle) <= 1.0e-7f ? 0.0f : 1.0f;
        for (int side = 0; side < 2; ++side)
        {
            const BoneTransform &ankle = m_model[size_t(m_chain[side][2])];
            const motionPose::VirtualFootTargetInput input{Vec3{}, Quat{}, ankle.p, ankle.q, 1.0f};
            motionPose::VirtualFootTargetOutput mapped{};
            if (!m_poseMapper.MapVirtualFootTarget(input, rootHalfAngle, mapped))
                return false;
            candidate.feet[size_t(side)] = {mapped.ankleRootPosition, mapped.ankleRootRotation};
        }
        candidate.valid = true;
        out = candidate;
        return true;
    }

    // Apply after the pose blend. Uses the existing carried animated knee pole
    // solver and changes rotations only. No foot-contact state is stored here;
    // the existing FootLocker remains the later contact stage.
    ApplyResult Apply(std::span<BoneTransform> blendedPose, const Targets &targets, float maxExtension = 0.995f,
                      float softExtension = 0.97f) noexcept
    {
        ApplyResult result{};
        if (!m_ready || blendedPose.size() != m_parents.size() || !targets.valid ||
            !std::isfinite(targets.warpWeight) || targets.warpWeight < 0.0f || targets.warpWeight > 1.0f ||
            !std::isfinite(maxExtension) || !std::isfinite(softExtension) || maxExtension <= 0.0f ||
            softExtension <= 0.0f || softExtension > maxExtension)
            return result;
        if (targets.warpWeight <= 0.0f)
        {
            result.applied = true; // Exact no-op: no FK, validation, or IK solve.
            return result;
        }
        for (const auto &t : blendedPose)
            if (!Valid(t))
                return result;
        for (const auto &f : targets.feet)
            if (!math::IsFinite(f.p) || !Unit(f.q))
                return result;

        std::copy(blendedPose.begin(), blendedPose.end(), m_work.begin());
        BuildModel(blendedPose, m_model);
        for (int side = 0; side < 2; ++side)
        {
            const int32_t ia = m_chain[side][0], ib = m_chain[side][1], ic = m_chain[side][2];
            const BoneTransform gp = m_model[size_t(m_parent[side])];
            const BoneTransform ga = m_model[size_t(ia)];
            const BoneTransform gb = m_model[size_t(ib)];
            const BoneTransform gc = m_model[size_t(ic)];
            const auto &correction = targets.feet[size_t(side)];
            const FootTarget target = targets.relative
                                          ? FootTarget{gc.p + correction.p,
                                                       math::QuatNormalize(math::QuatMul(correction.q, gc.q))}
                                          : correction;
            // An exactly neutral correction must not wake IK's knee-pole bias.
            if (targets.relative && math::Dot(correction.p, correction.p) <= 1.0e-14f &&
                correction.q.x * correction.q.x + correction.q.y * correction.q.y +
                        correction.q.z * correction.q.z <= 1.0e-14f)
            {
                result.feet[size_t(side)].status = legik::TwoBoneStatus::kOk;
                continue;
            }
            const Vec3 toe =
                m_toe[side] >= 0 ? m_model[size_t(m_toe[side])].p : gc.p + math::QuatRotate(gc.q, Vec3{0, 0.1f, 0});
            const auto solved = legik::SolveTwoBone(gp, ga, gb, gc, target.p, toe, maxExtension,
                                                    softExtension, true, targets.relative);
            result.feet[size_t(side)].status = solved.status;
            result.feet[size_t(side)].clamped = solved.clamped;
            if (solved.status != legik::TwoBoneStatus::kOk)
                return result; // fail closed for both legs; do not commit a
                               // half-updated pose, retain cause

            auto mixLocal = [&](int32_t bone, const Quat &desired) {
                const Quat delta =
                    math::QuatNormalize(math::QuatMul(desired, math::QuatInv(blendedPose[size_t(bone)].q)));
                const Quat weighted = math::QuatNormalize(math::QuatMul(
                    math::QuatNlerpShortest(Quat{}, delta, targets.warpWeight), blendedPose[size_t(bone)].q));
                m_work[size_t(bone)].q = weighted;
            };
            mixLocal(ia, solved.local[0]);
            mixLocal(ib, solved.local[1]);

            // Rebuild the leg parent orientation after weighted thigh/shin rotations,
            // then express the mapped Root-local ankle target orientation as the
            // foot's local rotation.
            BuildModel(m_work, m_model);
            const Quat parentQ = m_model[size_t(ib)].q;
            const Quat footLocalTarget =
                math::QuatNormalize(math::QuatMul(math::QuatInv(parentQ), target.q));
            mixLocal(ic, footLocalTarget);

            BuildModel(m_work, m_model);
            result.feet[size_t(side)].targetError = Len(m_model[size_t(ic)].p - target.p);
            if (!std::isfinite(result.feet[size_t(side)].targetError))
                return result;
        }
        std::copy(m_work.begin(), m_work.end(), blendedPose.begin());
        result.applied = true;
        return result;
    }

  private:
    static float Len(Vec3 v) noexcept
    {
        return std::sqrt(math::Dot(v, v));
    }
    static bool Unit(const Quat &q) noexcept
    {
        const float n2 = math::QuatDot(q, q);
        return math::IsFinite(q) && std::isfinite(n2) && std::fabs(n2 - 1.0f) <= 1.0e-3f;
    }
    static bool Valid(const BoneTransform &t) noexcept
    {
        return math::IsFinite(t.p) && Unit(t.q);
    }
    static bool Fail(InitError *out, InitError e)
    {
        if (out)
            *out = e;
        return false;
    }
    template <class Database> bool FindUnique(const Database &db, std::string_view name, int32_t &out, InitError *error)
    {
        out = -1;
        for (uint32_t i = 0; i < db.NumBones(); ++i)
            if (db.BoneName(i) == name)
            {
                if (out >= 0)
                    return Fail(error, InitError::DuplicateBone);
                out = static_cast<int32_t>(i);
            }
        return out >= 0 ? true : Fail(error, InitError::MissingBone);
    }
    bool IsDescendant(int32_t bone, int32_t ancestor) const
    {
        for (int32_t p = bone; p >= 0; p = m_parents[size_t(p)])
            if (p == ancestor)
                return true;
        return false;
    }
    static BoneTransform Mul(const BoneTransform &a, const BoneTransform &b)
    {
        return {a.p + math::QuatRotate(a.q, b.p), math::QuatNormalize(math::QuatMul(a.q, b.q))};
    }
    void BuildModel(std::span<const BoneTransform> local, std::vector<BoneTransform> &model) const
    {
        for (size_t i = 0; i < local.size(); ++i)
            model[i] = m_parents[i] < 0 ? local[i] : Mul(model[size_t(m_parents[i])], local[i]);
    }

    bool m_ready = false;
    std::vector<int32_t> m_parents;
    int32_t m_chain[2][3]{{-1, -1, -1}, {-1, -1, -1}};
    int32_t m_parent[2]{-1, -1};
    int32_t m_toe[2]{-1, -1};
    std::vector<BoneTransform> m_model;
    std::vector<BoneTransform> m_work;
    motionPose::Warp m_poseMapper;
};
} // namespace utpp::mm::motionFeet

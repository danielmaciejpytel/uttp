// CP pose-space equivalent of motion OrientationWarping + ResetRoot.
// Stateless and allocation-free in Apply; evaluated for each live player.
#pragma once

#include "Inertializer.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace utpp::mm::motionPose
{
using math::Quat;
using math::Vec3;
inline constexpr std::array<std::string_view, 5> kRequiredBones{"Hips", "Spine", "Spine1", "Spine2", "Spine3"};
// UE assigns 1/5 to each of spine_01..spine_05 when every configured bone is
// present and unweighted. CP maps spine_01..04 to four spine bones; spine_05's
// remaining 1/5 is carried by Spine3 per bonemap.
inline constexpr std::array<float, 4> kMappedCounterWeights{0.2f, 0.2f, 0.2f, 0.4f};

enum class InitError : uint8_t
{
    None,
    MissingBone,
    DuplicateBone
};

// Optional virtual motion foot marker input, expressed before OW in CP
// Root-local/model frame. Keeping the IK-root pivot and ankle-local offset
// separate preserves the two-stage rotation.
struct VirtualFootTargetInput
{
    Vec3 ikRootPosition{};
    Quat ikRootRotation{};
    Vec3 ankleLocalPosition{};
    Quat ankleLocalRotation{};
    float contactWeight = 0.0f;
};
struct VirtualFootTargetOutput
{
    Vec3 ankleRootPosition{};
    Quat ankleRootRotation{};
    float contactWeight = 0.0f;
};

class Warp
{
  public:
    template <class Database> bool Init(const Database &db, InitError *error = nullptr) noexcept
    {
        m_ready = false;
        m_indices.fill(std::numeric_limits<uint32_t>::max());
        if (error)
            *error = InitError::None;
        for (uint32_t i = 0; i < kRequiredBones.size(); ++i)
        {
            uint32_t found = std::numeric_limits<uint32_t>::max();
            for (uint32_t bone = 0; bone < db.NumBones(); ++bone)
            {
                if (db.BoneName(bone) != kRequiredBones[i])
                    continue;
                if (found != std::numeric_limits<uint32_t>::max())
                {
                    if (error)
                        *error = InitError::DuplicateBone;
                    return false;
                }
                found = bone;
            }
            if (found == std::numeric_limits<uint32_t>::max())
            {
                if (error)
                    *error = InitError::MissingBone;
                return false;
            }
            m_indices[i] = found;
        }
        m_ready = true;
        return true;
    }

    bool Ready() const noexcept
    {
        return m_ready;
    }

    // rootHalfAngle is the post-curve-alpha, post-0.5-distribution angle from
    // MotionOrientationWarp::Result. The caller supplies a freshly sampled local CP
    // pose per player before blending. Root is CP identity. No heap allocation or
    // persistent pose history. Hips is a direct root child, so its root-relative
    // position rotates with the rebased root; all other local translations and
    // legs remain untouched.
    bool Apply(std::span<BoneTransform> pose, float rootHalfAngle) const noexcept
    {
        if (!m_ready || !std::isfinite(rootHalfAngle))
            return false;
        for (uint32_t i : m_indices)
            if (i >= pose.size())
                return false;
        for (uint32_t i : m_indices)
            if (!IsValid(pose[i]))
                return false;
        if (std::fabs(rootHalfAngle) <= 1.0e-7f)
            return true;

        // Work in fixed stack storage first so invalid input cannot partially
        // mutate the pose.
        std::array<BoneTransform, 5> warped{};
        const Quat rootHalf = AxisAngleZ(rootHalfAngle);
        Quat parentComponent = math::QuatNormalize(math::QuatMul(rootHalf, pose[m_indices[0]].q));
        warped[0] = pose[m_indices[0]];
        warped[0].p = math::QuatRotate(rootHalf, pose[m_indices[0]].p);
        warped[0].q = parentComponent; // ResetRoot preserves the computed
                                       // direct-child Hips CS under ref root.

        for (std::size_t i = 1; i < warped.size(); ++i)
        {
            const Quat unwarpedComponent = math::QuatNormalize(math::QuatMul(parentComponent, pose[m_indices[i]].q));
            const Quat counter = AxisAngleZ(-rootHalfAngle * kMappedCounterWeights[i - 1]);
            const Quat warpedComponent = math::QuatNormalize(math::QuatMul(counter, unwarpedComponent));
            warped[i] = pose[m_indices[i]];
            warped[i].q = math::QuatNormalize(math::QuatMul(math::QuatInv(parentComponent), warpedComponent));
            parentComponent = warpedComponent;
        }

        for (std::size_t i = 0; i < warped.size(); ++i)
        {
            pose[m_indices[i]].p = warped[i].p;
            pose[m_indices[i]].q = warped[i].q; // preserve all other local translations and non-chain
                                                // bones exactly
        }
        return true;
    }

    bool FootContactDeferred() const noexcept
    {
        return true;
    }

    // Maps a virtual IK ankle marker for a downstream CP two-bone solver.
    // Root-half rotates the IK-root direct-child position; OW adds another half
    // to its CS orientation, so the ankle-local offset sees the full angle. OW
    // then counter-rotates the foot marker orientation by one half. Coordinates
    // are Root-local/model frame; caller converts world/Trajectory frames and
    // solves legs.
    bool MapVirtualFootTarget(const VirtualFootTargetInput &in, float rootHalfAngle,
                              VirtualFootTargetOutput &out) const noexcept
    {
        if (!std::isfinite(rootHalfAngle) || !math::IsFinite(in.ikRootPosition) ||
            !math::IsFinite(in.ankleLocalPosition) || !math::IsFinite(in.ikRootRotation) ||
            !math::IsFinite(in.ankleLocalRotation) || !std::isfinite(in.contactWeight) || in.contactWeight < 0.0f ||
            in.contactWeight > 1.0f)
            return false;
        const float rootN2 = math::QuatDot(in.ikRootRotation, in.ikRootRotation);
        const float ankleN2 = math::QuatDot(in.ankleLocalRotation, in.ankleLocalRotation);
        if (std::fabs(rootN2 - 1.0f) > 1.0e-3f || std::fabs(ankleN2 - 1.0f) > 1.0e-3f)
            return false;

        const Quat firstHalf = AxisAngleZ(rootHalfAngle);
        const Quat fullIkRootRotation =
            math::QuatNormalize(math::QuatMul(firstHalf, math::QuatMul(firstHalf, in.ikRootRotation)));
        const Vec3 warpedIkRootPosition = math::QuatRotate(firstHalf, in.ikRootPosition);
        VirtualFootTargetOutput candidate;
        candidate.ankleRootPosition =
            warpedIkRootPosition + math::QuatRotate(fullIkRootRotation, in.ankleLocalPosition);
        candidate.ankleRootRotation = math::QuatNormalize(
            math::QuatMul(AxisAngleZ(-rootHalfAngle), math::QuatMul(fullIkRootRotation, in.ankleLocalRotation)));
        candidate.contactWeight = in.contactWeight;
        if (!math::IsFinite(candidate.ankleRootPosition) || !math::IsFinite(candidate.ankleRootRotation))
            return false;
        out = candidate;
        return true;
    }

  private:
    static Quat AxisAngleZ(float angle) noexcept
    {
        const float half = 0.5f * angle;
        return Quat{0.0f, 0.0f, std::sin(half), std::cos(half)};
    }
    static bool IsValid(const BoneTransform &x) noexcept
    {
        if (!math::IsFinite(x.p) || !math::IsFinite(x.q))
            return false;
        const float n2 = math::QuatDot(x.q, x.q);
        return std::isfinite(n2) && std::fabs(n2 - 1.0f) <= 1.0e-3f;
    }

    std::array<uint32_t, 5> m_indices{};
    bool m_ready = false;
};
} // namespace utpp::mm::motionPose

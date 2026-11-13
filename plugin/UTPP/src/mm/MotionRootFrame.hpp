// Transactional same-feed native planar root kernel. No allocation or game
// state.
#pragma once
#include "MotionOffsetRoot.hpp"
#include "VisualRootFrame.hpp"
#include <cstdint>
namespace utpp::mm::rootframe
{
struct Inputs
{
    float dt = 0,
          actorYaw = 0; // ORIGINAL real seconds (before caller clamp), radians
    math::Vec3 capsuleDeltaCurrent{}, animationDeltaPreviousVisual{};
    float capsuleYawDelta = 0, animationYawDelta = 0;
    bool movingGround = false, montage = false, teleport = false, rootReliable = true;
    float staticTailWeight = 0;
    bool aimFacing = false;
    bool restRecenter = false; // UTPP rest recentering permission (MotionOffsetRoot.hpp): grounded, standing / crouching
    bool restTurnPlaying = false; // the matcher plays a turn-in-place clip: the fallback stays out of its way
    float lockMaxRate = 0;        // rad/s bound of the moving-aim lock (0 = 540 deg/s, MotionOffsetRoot.hpp Step::lockMaxRate)
    bool airFollow = false;       // Shooter mode in the air: bounded follow of the actor at lockMaxRate (MotionOffsetRoot.hpp Step::airFollow)
    bool holdOnGap = false;       // Shooter mode, standing: a frame gap (dt > 0.25 s) keeps the held visual heading instead of reseeding onto the actor
};
enum class Reseed
{
    None,
    First,
    Gap,
    Teleport
};
class Processor;
class Proposal
{
  public:
    const visualroot::Frame &Frame() const
    {
        return m_frame;
    }
    float VisualWorldYaw() const
    {
        return m_visual;
    }
    bool RootReliable() const
    {
        return m_reliable;
    }
    float StaticTailWeight() const
    {
        return m_tail;
    }
    uint64_t Epoch() const
    {
        return m_nextEpoch;
    }
    uint64_t Sequence() const
    {
        return m_baseSequence + 1;
    }
    Reseed Reseeded() const
    {
        return m_reseed;
    }

  private:
    friend class Processor;
    const Processor *m_owner = nullptr;
    uint64_t m_baseEpoch = 0, m_baseSequence = 0, m_nextEpoch = 0;
    motionRoot::OffsetRoot m_native;
    visualroot::Frame m_frame{};
    float m_visual = 0, m_actor = 0, m_tail = 0;
    bool m_reliable = false;
    Reseed m_reseed = Reseed::None;
};
class Processor
{
  public:
    bool Reset(float actorYaw)
    {
        if (!std::isfinite(actorYaw))
            return false;
        m_native.Reset();
        m_actor = Wrap(actorYaw);
        m_visual = m_actor;
        m_initialized = true;
        m_first = true;
        ++m_epoch;
        m_sequence = 0;
        return true;
    }
    // Reset that keeps a held heading (Shooter mode: V stands, the camera has orbited her; a reseed must not turn her to face the camera).
    // The visual world yaw is aVisualYaw, the native offset is the difference to the actor.
    bool ResetHeld(float actorYaw, float aVisualYaw)
    {
        if (!std::isfinite(actorYaw) || !std::isfinite(aVisualYaw))
            return false;
        m_native.Reset();
        m_actor = Wrap(actorYaw);
        m_visual = Wrap(aVisualYaw);
        m_native.Seed(0, 0, Wrap(double(m_visual) - double(m_actor)));
        m_initialized = true;
        m_first = false;
        ++m_epoch;
        m_sequence = 0;
        return true;
    }
    float PreviousVisualWorldYaw() const
    {
        return m_visual;
    }
    visualroot::Frame Frame() const
    {
        return {m_native.X(), m_native.Y(), m_native.Yaw()};
    }
    uint64_t Epoch() const
    {
        return m_epoch;
    }
    uint64_t Sequence() const
    {
        return m_sequence;
    }
    bool Prepare(const Inputs &in, Proposal &output) const
    {
        if (!std::isfinite(in.dt) || in.dt < 0 || !std::isfinite(in.actorYaw) ||
            !math::IsFinite(in.capsuleDeltaCurrent) || !math::IsFinite(in.animationDeltaPreviousVisual) ||
            !std::isfinite(in.capsuleYawDelta) || !std::isfinite(in.animationYawDelta) ||
            !std::isfinite(in.staticTailWeight) || in.staticTailWeight < 0 || in.staticTailWeight > 1)
            return false;
        Proposal p;
        p.m_owner = this;
        p.m_baseEpoch = m_epoch;
        p.m_baseSequence = m_sequence;
        p.m_nextEpoch = m_epoch;
        p.m_native = m_native;
        p.m_actor = Wrap(in.actorYaw);
        p.m_tail = in.staticTailWeight;
        p.m_reliable = in.rootReliable && in.staticTailWeight == 0;
        const double displacement = std::hypot(double(in.capsuleDeltaCurrent.x), double(in.capsuleDeltaCurrent.y));
        if (!m_initialized || m_first)
            p.m_reseed = Reseed::First;
        else if (in.dt > .25f)
            p.m_reseed = Reseed::Gap;
        else if (in.teleport || displacement > .5)
            p.m_reseed = Reseed::Teleport;
        if (p.m_reseed != Reseed::None)
        {
            p.m_native.Reset();
            p.m_visual = p.m_actor;
            if (in.holdOnGap && p.m_reseed == Reseed::Gap && m_initialized)
            {
                p.m_native.Seed(0, 0, Wrap(double(m_visual) - double(p.m_actor)));
                p.m_visual = m_visual;
            }
            if (p.m_reseed != Reseed::First)
                ++p.m_nextEpoch;
        }
        else if (in.dt == 0)
        {
            // No physical advancement: retain the last published frame and heading
            // even if an inconsistent zero-duration feed carries a different actor
            // transform.
            p.m_actor = m_actor;
            p.m_visual = m_visual;
        }
        else
        {
            motionRoot::Step step;
            step.dt = in.dt;
            step.capsuleDeltaCurrent = {in.capsuleDeltaCurrent.x, in.capsuleDeltaCurrent.y, 0};
            step.capsuleYawDelta = in.capsuleYawDelta;
            step.animationDeltaVisual = {in.animationDeltaPreviousVisual.x, in.animationDeltaPreviousVisual.y, 0};
            step.animationYawDelta = in.animationYawDelta;
            step.movingGround = in.movingGround;
            step.montage = in.montage;
            step.aimFacing = in.aimFacing;
            step.restRecenter = in.restRecenter;
            step.turnPlaying = in.restTurnPlaying;
            step.lockMaxRate = in.lockMaxRate;
            step.airFollow = in.airFollow;
            if (!p.m_native.Update(step))
                return false;
            p.m_visual = Wrap(double(p.m_actor) + p.m_native.Yaw());
        }
        p.m_frame = {p.m_native.X(), p.m_native.Y(), p.m_native.Yaw()};
        if (!visualroot::Valid(p.m_frame) || !std::isfinite(p.m_visual))
            return false;
        output = p;
        return true;
    }
    bool Commit(const Proposal &p)
    {
        if (p.m_owner != this || p.m_baseEpoch != m_epoch || p.m_baseSequence != m_sequence)
            return false;
        m_native = p.m_native;
        m_actor = p.m_actor;
        m_visual = p.m_visual;
        m_epoch = p.m_nextEpoch;
        ++m_sequence;
        m_initialized = true;
        m_first = false;
        return true;
    }

  private:
    static float Wrap(double yaw)
    {
        return float(std::remainder(yaw, 2 * 3.14159265358979323846));
    }
    motionRoot::OffsetRoot m_native;
    float m_actor = 0, m_visual = 0;
    uint64_t m_epoch = 1, m_sequence = 0;
    bool m_initialized = false, m_first = true;
};
} // namespace utpp::mm::rootframe

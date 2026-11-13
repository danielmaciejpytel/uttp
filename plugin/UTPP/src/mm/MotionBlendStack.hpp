// Live outgoing-clip crossfade. Implements the documented blend contract.
#pragma once
#include "MotionAdapter.hpp"
#include "MotionOrientationFeet.hpp"
#include "MotionOrientationPose.hpp"
#include "MotionOrientationWarp.hpp"
#include "MotionSteering.hpp"
#include "MotionRootTrack.hpp"
#include <array>
#include <span>
#ifndef UTPP_FOOTSINK_MUTATION
#define UTPP_FOOTSINK_MUTATION 0
#endif
namespace utpp::mm::motionstack
{
struct Signals
{
    float rootX = 0, rootY = 0, rootYaw = 0, left = 0, right = 0;
};
struct Player
{
    int32_t clip = -1;
    float time = 0, rate = 1, clock = 0, duration = 0;
    float rootRate = 1; // actual elapsed nonloop animation interval / real dt
    bool stored = false;
    bool rootApproximate = false; // new post-search selection has no preceding interval
    motionSteer::Steering generalSteering, tipSteering;
    motionow::State orientationState;
    float lastSteeredYawRate = 0, lastOrientationHalfAngle = 0;
    bool haveSteered = false;
    int cancelStartSupport = -1;
    bool stanceFade = false; // this layer's fade-in is a ground stance change (crouch <-> stand), see KeepFootHeight
    bool groundFade = false; // this layer's fade-in is a ground locomotion -> ground locomotion fade (BlendRequest::groundFade), see KeepFootHeight
    bool airFade = false;    // this layer was picked in the air movement mode (BlendRequest::toAir), see KeepAnklesAboveLayers
};
struct GraphInputs
{
    bool enabled = false, generalSteering = false, tipSteering = false;
    float previousVisualWorldYaw = 0, targetWorldYaw = 0, actorWorldYaw = 0;
    float velocityWorldX = 0, velocityWorldY = 0;
};
class BlendStack
{
  public:
    static constexpr uint32_t kLayers = 4, kCapacity = 6;
    static constexpr float kFootGateFall = 0.12f; // s, full-scale fall time of the foot height gate
    // durations multiply base blend duration, already resolved to CP bones by caller.
    // No implicit UE skeleton mapping. Empty => uniform. Allocations occur only here.
    bool Init(const motion::MotionDatabase &db, std::span<const float> durationFactors = {})
    {
        if (!db.Loaded() || (!durationFactors.empty() && durationFactors.size() != db.NumBones()))
            return false;
        for (float x : durationFactors)
            if (!std::isfinite(x) || x < 0 || x > 1)
                return false;
        m_db = &db;
        m_n = db.NumBones();
        m_factors.assign(m_n, 1);
        if (!durationFactors.empty())
            std::copy(durationFactors.begin(), durationFactors.end(), m_factors.begin());
        for (auto &p : m_pose)
            p.resize(m_n);
        m_out.resize(m_n);
        m_count = 0;
        m_collapses = 0;
        m_footGate[0] = m_footGate[1] = m_stanceGate[0] = m_stanceGate[1] = 0.0f;
        m_orientationCurve = -1;
        m_poseWarp.Init(db);
        m_feetReady = m_feetWarp.Init(db);
        for (uint32_t k = 0; k < db.NumBones(); ++k)
        {
            if (db.BoneName(k) == "LeftFoot")
                m_footBones[0] = int32_t(k);
            if (db.BoneName(k) == "RightFoot")
                m_footBones[1] = int32_t(k);
            if (db.BoneName(k) == "Hips")
                m_hipsBone = int32_t(k);
        }
        m_hipsChain.clear();
        for (int32_t b = m_hipsBone; b >= 0; b = db.BoneParent(uint32_t(b)))
            m_hipsChain.insert(m_hipsChain.begin(), b);
        for (uint32_t k = 0; k < db.NumCurves(); ++k)
            if (db.CurveName(k) == "Enable_OrientationWarping")
                m_orientationCurve = int32_t(k);
        for (uint32_t side = 0; side < 2; ++side)
        {
            m_ankleChain[side].clear();
            for (int32_t b = m_footBones[side]; b >= 0; b = db.BoneParent(uint32_t(b)))
                m_ankleChain[side].insert(m_ankleChain[side].begin(), b);
        }
        return true;
    }
    void SetGraphInputs(const GraphInputs &input)
    {
        m_graph = input;
    }
    // Foot height keeping (see KeepFootHeight). Default on; off reproduces the plain crossfade bit for bit.
    void SetFootHeightKeep(bool enabled)
    {
        m_footHeightKeep = enabled;
    }
    // Ankle floor of the blend (see KeepAnklesAboveLayers). Default on; off (with SetFootHeightKeep(false)) reproduces the plain crossfade bit for bit.
    void SetFootFloor(bool enabled)
    {
        m_footFloor = enabled;
    }
    // Last KeepAnklesAboveLayers outcome: dz = the vertical ankle lift (m) per side, applied = the leg solve ran.
    struct FootFloorInfo
    {
        float dz[2]{};
        float gate = 0.0f; // applied floor gate (0 under an air pick, see KeepAnklesAboveLayers)
        bool applied = false;
    };
    const FootFloorInfo &FootFloor() const
    {
        return m_floorInfo;
    }
    // Last KeepFootHeight outcome: dz = the vertical ankle lift asked for (m), clamped = the IK reach limit cut it.
    struct FootKeepInfo
    {
        float dz[2]{};   // the signed vertical ankle change asked for (m) before the pelvis took its share: lift of both kinds of fade, lowering of stance fades
        float gate[2]{}; // the applied permission gate (0 = the plain crossfade, 1 = fully corrected) per side
        float pelvis = 0.0f; // the Hips drop (m) that took part of dz (both feet floating)
        float floating = 0.0f, room = 0.0f; // the smaller float of the two feet (m) and the room the Hips fold of the legs' own timing leaves for the drop
        bool clamped[2]{};
        bool applied = false;
    };
    const FootKeepInfo &FootKeep() const
    {
        return m_footInfo;
    }
    void Reset()
    {
        m_footGate[0] = m_footGate[1] = m_stanceGate[0] = m_stanceGate[1] = 0.0f;
        m_floorGate = 1.0f;
        m_count = 0;
        m_collapses = 0;
    }
    // Bridge only: seed the last actually emitted base pose after raw/traversal.
    // This static snapshot has no animation clock and approximate ROOT metadata.
    bool SeedPose(std::span<const BoneTransform> pose, Signals signals = {})
    {
        if (!m_db || pose.size() != m_n || !std::isfinite(signals.rootX) || !std::isfinite(signals.rootY) ||
            !std::isfinite(signals.rootYaw) || !std::isfinite(signals.left) || !std::isfinite(signals.right))
            return false;
        for (const auto &p : pose)
            if (!std::isfinite(p.p.x) || !std::isfinite(p.p.y) || !std::isfinite(p.p.z) || !math::IsFinite(p.q) ||
                std::fabs(math::QuatDot(p.q, p.q) - 1.f) > 1e-3f)
                return false;
        Reset();
        m_count = 1;
        m_players[0] = Player{};
        m_players[0].stored = true;
        std::copy(pose.begin(), pose.end(), m_pose[0].begin());
        std::copy(pose.begin(), pose.end(), m_out.begin());
        m_samples[0] = signals;
        CacheAnkleZ(0);
        m_targets[0] = {};
        if (m_feetReady && (!m_feetWarp.Sample(pose, 0, m_targets[0]) ||
                            !m_feetWarp.MakeRelative(pose, m_targets[0])))
            return false;
        m_signals = signals;
        return true;
    }
    uint32_t Count() const
    {
        return m_count;
    }
    uint32_t Collapses() const
    {
        return m_collapses;
    }
    const Player &At(uint32_t i) const
    {
        return m_players[i];
    }
    const Signals &OutputSignals() const
    {
        return m_signals;
    }
    int CancelledStartSupport() const
    {
        if (!m_count || m_players[0].cancelStartSupport < 0 || m_players[0].clock >= m_players[0].duration)
            return -1;
        const int side = m_players[0].cancelStartSupport;
        return (side ? m_signals.right : m_signals.left) >= .7f ? side : -1;
    }
    float StaticTailWeight() const
    {
        float weight = 0;
        for (uint32_t i = 0; i < m_count; ++i)
            if (m_players[i].stored)
                weight += Weight(i);
        return weight;
    }
    // Reliability of this rate approximation, not equivalence to UE graph attributes.
    bool RootReliable() const
    {
        if (!m_count || StaticTailWeight() > 0)
            return false;
        for (uint32_t i = 0; i < m_count; ++i)
            if (m_players[i].rootApproximate && Weight(i) > 0)
                return false;
        return true;
    }
    const BoneTransform *Output() const
    {
        return m_out.data();
    }
    // Local pose of layer i as it enters the mix (live: sampled and warped; stored: the collapsed tail); null past Count().
    const BoneTransform *LayerPose(uint32_t i) const
    {
        return i < m_count ? m_pose[i].data() : nullptr;
    }
    float Weight(uint32_t i, int32_t bone = -1) const
    {
        if (i >= m_count || bone >= int32_t(m_n) || bone < -1)
            return 0;
        float remaining = 1;
        for (uint32_t j = 0; j < i; ++j)
            remaining *= 1 - Alpha(m_players[j], bone);
        return i + 1 == m_count ? remaining : remaining * Alpha(m_players[i], bone);
    }
    // POST-SEARCH-SAMPLE phase: selected.time is authoritative and already advanced by
    // current UTPP matcher. Never add dt to head again. Outgoing time advances by its
    // stored rate. A native UE PRE-TICK adapter must be separate, not silently mixed.
    bool Step(const motion::MatchResult &selected, float dt)
    {
        if (!m_db || !std::isfinite(dt) || dt < 0 || selected.clip < 0 || uint32_t(selected.clip) >= m_db->NumClips() ||
            !std::isfinite(selected.clipTime) || selected.clipTime < 0 || !std::isfinite(selected.playRate) ||
            selected.playRate <= 0 || !std::isfinite(selected.blend.blendTime) || selected.blend.blendTime < 0)
            return false;
        const bool jump = !m_count || selected.jumped;
        if (!jump && m_players[0].clip != selected.clip)
            return false;
        const float previousHeadTime = m_count ? m_players[0].time : 0;
        cancelstart::Plan cancellation{-1, selected.blend.blendTime};
        if (jump && m_count && selected.blend.cancelledGroundStart && selected.cancelledStartSupportAllowed &&
            !m_players[0].stored && m_players[0].clip == selected.blend.fromClip)
            cancellation = cancelstart::Resolve(*m_db, m_players[0].clip, previousHeadTime, m_players[0].rate,
                selected.clip, selected.clipTime, selected.playRate, m_signals.left, m_signals.right,
                selected.blend.blendTime);
        // Advance existing live players before insertion: these represent the preceding
        // interval. The authoritative head is overwritten below, avoiding double time.
        for (uint32_t i = 0; i < m_count; ++i)
            if (!m_players[i].stored)
            {
                auto &p = m_players[i];
                const float previous = p.time;
                p.time = Advance(p.clip, p.time + dt * p.rate);
                if (dt > 0)
                {
                    p.rootRate = IsLoop(p.clip) ? p.rate : std::fmax(0.f, p.time - previous) / dt;
                    p.rootApproximate = false;
                }
            }
        if (jump)
        {
            bool replace = m_count && m_players[0].clock < m_players[0].duration && m_players[0].clock < .03f;
            if (!replace && m_count)
            {
                if (m_count == kCapacity)
                    Collapse();
                for (uint32_t i = m_count; i > 0; --i)
                {
                    m_players[i] = m_players[i - 1];
                    m_pose[i].swap(m_pose[i - 1]);
                    m_samples[i] = m_samples[i - 1];
                    m_targets[i] = m_targets[i - 1];
                    m_ankleZ[i] = m_ankleZ[i - 1];
                }
                ++m_count;
            }
            else if (!m_count)
                m_count = 1;
            Player p;
            p.clip = selected.clip;
            p.time = selected.clipTime;
            p.rate = selected.playRate;
            p.rootRate = selected.playRate;
            p.rootApproximate = true;
            p.duration = m_count == 1 ? 0 : cancellation.duration;
            p.cancelStartSupport = cancellation.support;
            // The matcher judges the fade against the clip it played before (its m_cur); a replaced head (< 30 ms old) is not what lies under the new
            // pick: the layer under the head is. A replaced head that was no ground / stance fade itself (a landing over the air layer, an air or
            // slide pick over a ground layer) sat over something that is not a ground fade's outgoing side, so the new pick's flags are not taken
            // from the matcher then; a flagged replaced head passes its flags on to a ground pick only (the inherited stance flag too).
            const bool oldStance = replace && m_players[0].stanceFade, oldGround = replace && m_players[0].groundFade;
#if UTPP_FOOTSINK_MUTATION == 9 // mutation 9: the flags of a replacing pick come from the matcher alone (judged against the replaced head)
            const bool underGround = true;
#else
            const bool underGround = !replace || oldStance || oldGround;
#endif
#if UTPP_FOOTSINK_MUTATION == 10 // mutation 10: the stance flag is inherited by any replacing pick (air / slide included)
            p.stanceFade = (selected.blend.stanceChange && underGround) || oldStance;
#else
            p.stanceFade = (selected.blend.stanceChange && underGround) || (oldStance && selected.blend.toGround);
#endif
            p.groundFade = (selected.blend.groundFade && underGround) || (oldGround && selected.blend.toGround);
            p.airFade = selected.blend.toAir;
            m_players[0] = p;
        }
        if (!selected.cancelledStartSupportAllowed) m_players[0].cancelStartSupport = -1;
        m_players[0].time = selected.clipTime;
        m_players[0].rate = selected.playRate;
        if (!jump && dt > 0)
        {
            auto &head = m_players[0];
            head.rootRate =
                IsLoop(head.clip) ? head.rate : std::fmax(0.f, Advance(head.clip, head.time) - previousHeadTime) / dt;
            head.rootApproximate = false;
        }
        for (uint32_t i = 0; i < m_count; ++i)
        {
            auto &p = m_players[i];
            p.clock += dt * (i == 0 ? 1 : std::pow(1.1f, float(i + 1)));
            if (!p.stored)
            {
                if (!adapter::SamplePose(*m_db, p.clip, p.time, m_pose[i].data(), m_n))
                    return false;
                m_samples[i] = Sample(p);
                if (m_graph.enabled)
                {
                    // Virtual foot markers are sampled before the Hips/spine warp, then
                    // blended with the same leg profile and solved once after pose
                    // blending.
                    if (!m_feetReady || !ProcessGraph(p, m_samples[i], dt) ||
                        !m_feetWarp.Sample(m_pose[i], p.lastOrientationHalfAngle, m_targets[i]) ||
                        !m_poseWarp.Apply(m_pose[i], p.lastOrientationHalfAngle) ||
                        !m_feetWarp.MakeRelative(m_pose[i], m_targets[i]))
                        return false;
                }
                CacheAnkleZ(i);
            }
        }
        // Consolidate the least-significant live tail at its current sampled time.
        if (m_count > kLayers + 1)
            Collapse();
        Evaluate();
        if (m_graph.enabled && !m_feetWarp.Apply(m_out, m_outputTargets).applied)
            return false;
        KeepFootHeight(dt);
        KeepAnklesAboveLayers(dt);
        // Match scalar UE relevance pruning; per-bone durations must not exceed base
        // duration for this pruning to be faithful (enforced by caller convention).
        for (uint32_t i = 0; i + 1 < m_count; ++i)
            if (Alpha(m_players[i], -1) >= 1)
            {
                m_count = i + 1;
                break;
            }
        return true;
    }

  private:
    // Model-space ankle height of one leg of a local pose: FK along the chain from the skeleton root down to the foot.
    float AnkleZ(const BoneTransform *pose, uint32_t side) const
    {
        math::Quat q{0, 0, 0, 1};
        math::Vec3 p{0, 0, 0};
        for (const int32_t b : m_ankleChain[side])
        {
            p = p + math::QuatRotate(q, pose[b].p);
            q = math::QuatNormalize(math::QuatMul(q, pose[b].q));
        }
        return p.z;
    }
    // The ankle height of a layer's own pose is cached per layer (sampled / warped pose, or the fold of the collapsed tail), so that the tail
    // collapse and the dt0 insertion leave the foot target exactly where it was.
    void CacheAnkleZ(uint32_t i)
    {
        for (uint32_t side = 0; side < 2; ++side)
            m_ankleZ[i][side] = m_ankleChain[side].empty() ? 0.0f : AnkleZ(m_pose[i].data(), side);
    }
    static float MixZ(float a, float b, float w)
    {
        return w <= 0 ? a : w >= 1 ? b : a + (b - a) * w; // as Mix(): the same fold order and weights
    }
    // Model-space Hips height of a local pose (FK along the chain from the skeleton root down to Hips).
    float HipsZ(const BoneTransform *pose) const
    {
        math::Quat q{0, 0, 0, 1};
        math::Vec3 p{0, 0, 0};
        for (const int32_t b : m_hipsChain)
        {
            p = p + math::QuatRotate(q, pose[b].p);
            q = math::QuatNormalize(math::QuatMul(q, pose[b].q));
        }
        return p.z;
    }
    // Lowers the whole body by `drop` metres in model space: the Hips translation moves by the drop in the frame of its parent.
    void DropPelvis(float drop)
    {
        math::Quat q{0, 0, 0, 1};
        for (size_t k = 0; k + 1 < m_hipsChain.size(); ++k)
            q = math::QuatNormalize(math::QuatMul(q, m_out[uint32_t(m_hipsChain[k])].q));
        auto &hips = m_out[uint32_t(m_hipsBone)];
        hips.p = hips.p + math::QuatRotate(math::QuatInv(q), math::Vec3{0.0f, 0.0f, -drop});
    }
    // Height preservation of the crossfade. Evaluate() mixes LOCAL transforms: the Hips translation lerps while the thigh / shin rotations
    // nlerp, and the leg bones run a shorter blend (WeightFactor 1/1.5) than the Hips (1), so mid-blend the legs are already at the
    // incoming pose while the pelvis is still on its way. FK of that mix is not the mix of the layers' FK: on crouch -> stand an almost
    // straight leg hangs from a pelvis that is still low and the ankle sinks far below the floor of both layers (measured up to 25 cm,
    // of which ~9 cm is the concavity of the nlerp alone), the feet standing "to the ankles" in the ground for ~0.5 s. The mirror case,
    // stand -> crouch, floats the feet up (up to 11 cm). Nothing downstream restores the height: the foot lock is planar and the motion feet
    // targets are relative orientation-warp corrections that carry no height.
    // The target is the ankle height of the layers' own FK folded with exactly Evaluate()'s order and the foot bone's alphas (the
    // per-bone weights the pose mix uses for the foot), the output's is read after the feet warp; dz = target - output is applied as a
    // vertical-only relative correction through the two-bone solver (rotations only, the carried animated knee pole keeps the bend plane,
    // the foot keeps its model-space orientation). dz tends to 0 continuously with the blend (all alphas -> 1: output == the layer 0 pose;
    // alphas -> 0: the older layer alone), a single layer is returned before any math (steady state stays bit-identical), and a failing
    // solve leaves the pose untouched (never fails the frame).
    // SCOPE (BlendRequest, both set by the matcher; every other fade stays the plain crossfade bit for bit):
    //  - stanceFade: a ground stance change (crouch <-> stand). Both directions: a sunk foot is lifted, a floating one lowered through the
    //    leg (the stand -> crouch float, up to 11 cm), and when BOTH feet float the pelvis drops first (DropPelvis: the legs already hold
    //    the crouched shape on their faster blend while the pelvis is still high, so extending them runs into the reach limit). The drop
    //    is the smaller of the two floats and at most the Hips height minus the fold of the layers' Hips heights WITH THE FOOT WEIGHTS
    //    (the pelvis never goes lower than where the legs' own timing puts it); the legs settle both feet on their targets afterwards.
    //  - groundFade: a ground locomotion -> ground locomotion fade (start / loop / stop / pivot / turn / idle of either stance, from a
    //    landing or a slide exit clip as well). A sunk foot is lifted (the visible defect: walk -> stop up to 13 cm, the crouched gait
    //    fades 6 cm). Floats are left to the pelvis alone: when both feet float it drops by the smaller float, within the same bound
    //    (a crouch stop: the legs are in the stop pose while the Hips still travel, 6 cm on both feet); a foot that floats more than
    //    its partner keeps the difference, a swinging foot is never pulled down through its leg (it would be snapped onto the blend as
    //    its contact signal rises). The short slide / air / traversal fades are not flagged: lifting there changed the thigh swing of
    //    fades whose feet are not planted (jump_pick fixture: slide -> jump 51.8 -> 65.5 deg per 50 ms, slide entry pop 124 -> 245).
    //    That holds when such a fade lands on top of a flagged one too: an unflagged layer with an older layer under it vetoes the permission of
    //    every flagged layer below it (the gate then falls over kFootGateFall like any lost permission). A pick that replaces a head younger
    //    than 30 ms does not take the matcher's flags (judged against the replaced head) unless the replaced head itself was flagged.
    // CONTINUITY: dz is 0 where a fade starts (the output is the older pose) and where it ends (the output is the new pose); a pick that
    // lands inside another fade ramps in over its first ~50 ms, and the gate falls over 0.12 s when its permission is lost (the layer
    // completing under a newer, unflagged fade), so neither edge steps dz (see the gates in the body). The pelvis drop follows the gate and
    // the continuous floats of the feet, so it is continuous as well.
    void KeepFootHeight(float dt)
    {
        m_footInfo = {};
        if (!m_footHeightKeep || m_count < 2 || !m_feetReady || m_ankleChain[0].empty() || m_ankleChain[1].empty())
        {
            m_footGate[0] = m_footGate[1] = m_stanceGate[0] = m_stanceGate[1] = 0.0f; // a single layer owes nothing: the output is its pose
            return;
        }
        // PERMISSION of a fade still running: its layer has an older one under it (or is the stored tail of one). It follows the foot alpha
        // of that layer (full once that alpha reaches 1/16, ~50 ms into the fade), so a pick landing inside another fade does not pop in
        // the lift that fade already owes. A stored tail never grants more than the gate that was running (hold).
        // perm[0]: any flagged fade (the lift, the pelvis drop); perm[1]: stance fades alone (the lowering through the legs).
        // The APPLIED gate is continuous in time: it follows a rising permission at once (the alpha ramp is its own continuity) and
        // falls at most 1 / kFootGateFall per second. The permission is lost in one frame when the flagged layer becomes the last one
        // (the layers above it are a fade that still owes its own sink: up to 13 cm), so a dropped permission must not step dz.
        (void)dt; // unused by the mutation 2 build
        // VETO: a layer that is flagged neither as stance nor as ground fade (a slide / jump / air / traversal entry) and has an older layer under
        // it takes the permission of every flagged layer below it: the feet of that fade are leaving the ground, the lift / lowering the layers
        // under it still owe would run through a blend whose feet are not planted (take-off, slide entry). The gate falls over kFootGateFall from
        // wherever it was (below), so the veto steps nothing. Layers above the unflagged one keep their own permission.
        float perm[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        float *const gates[2] = {m_footGate, m_stanceGate};
        bool vetoed = false;
        for (uint32_t i = 0; i < m_count; ++i)
        {
            const bool flagged = m_players[i].stanceFade || m_players[i].groundFade;
#if UTPP_FOOTSINK_MUTATION != 8 // mutation 8: nothing vetoes a flagged layer
            if (!flagged && !m_players[i].stored && i + 1 < m_count)
                vetoed = true;
#endif
            if (flagged && !vetoed && (i + 1 < m_count || m_players[i].stored))
                for (uint32_t side = 0; side < 2; ++side)
                    for (uint32_t k = 0; k < 2; ++k)
                    {
                        if (k == 1 && !m_players[i].stanceFade)
                            continue;
#if UTPP_FOOTSINK_MUTATION == 3 // mutation 3: a stored tail grants the full gate
                        perm[k][side] = std::fmax(perm[k][side], m_players[i].stored ? 1.0f : std::fmin(1.0f, 16.0f * Alpha(m_players[i], m_footBones[side])));
#else
                        perm[k][side] = std::fmax(perm[k][side], m_players[i].stored
                                                                     ? gates[k][side]
                                                                     : std::fmin(1.0f, 16.0f * Alpha(m_players[i], m_footBones[side])));
#endif
                    }
        }
        float gate[2][2];
        for (uint32_t k = 0; k < 2; ++k)
            for (uint32_t side = 0; side < 2; ++side)
            {
#if UTPP_FOOTSINK_MUTATION == 2 // mutation 2: the gate follows the permission at once (drops 1 -> 0 in a frame)
                gate[k][side] = perm[k][side];
#else
                gate[k][side] = std::fmax(perm[k][side], gates[k][side] - dt / kFootGateFall);
#endif
                gates[k][side] = gate[k][side] = std::clamp(gate[k][side], 0.0f, 1.0f);
            }
        m_footInfo.gate[0] = gate[0][0];
        m_footInfo.gate[1] = gate[0][1];
        if (!(gate[0][0] > 0.0f) && !(gate[0][1] > 0.0f))
            return;
        float want[2], floatAmt[2]; // floatAmt: the signed height difference target - output (negative = the foot floats)
        for (uint32_t side = 0; side < 2; ++side)
        {
            float z = m_ankleZ[m_count - 1][side];
            for (int i = int(m_count) - 2; i >= 0; --i)
                z = MixZ(z, m_ankleZ[uint32_t(i)][side], Alpha(m_players[uint32_t(i)], m_footBones[side]));
            const float d = z - AnkleZ(m_out.data(), side);
            // the lowering through the leg: stance fades only (a ground fade lowers through the pelvis alone: a swinging foot that lands
            // would be snapped to the blend within a frame or two as its contact signal rises)
            want[side] = gate[0][side] * std::fmax(0.0f, d)
#if UTPP_FOOTSINK_MUTATION != 7 // mutation 7: no lowering through the legs (a floating foot stays where the plain crossfade left it)
                         + gate[1][side] * std::fmin(0.0f, d)
#endif
                ;
            floatAmt[side] = std::fmin(0.0f, d);
            if (!std::isfinite(want[side]))
                return;
        }
        // Both feet float: the pelvis takes the smaller float, bounded by the Hips fold of the legs' own timing.
        float drop = 0.0f;
#if UTPP_FOOTSINK_MUTATION != 6 // mutation 6: no pelvis drop (the legs alone lower the floating feet)
        if (m_hipsBone >= 0)
        {
            const float floating = std::fmin(gate[0][0] * -floatAmt[0], gate[0][1] * -floatAmt[1]);
            if (floating > 0.0f)
            {
                float fold = HipsZ(m_pose[m_count - 1].data());
                for (int i = int(m_count) - 2; i >= 0; --i)
                    fold = MixZ(fold, HipsZ(m_pose[uint32_t(i)].data()), Alpha(m_players[uint32_t(i)], m_footBones[0]));
                m_footInfo.floating = floating;
                m_footInfo.room = std::fmax(0.0f, HipsZ(m_out.data()) - fold);
                drop = std::fmin(floating, m_footInfo.room);
            }
        }
#endif
        if (drop > 0.0f)
            DropPelvis(drop);
        m_footInfo.pelvis = drop;
        motionFeet::Targets targets{};
        targets.valid = true;
        targets.relative = true;
        targets.warpWeight = 1.0f;
        bool any = false;
        for (uint32_t side = 0; side < 2; ++side)
        {
            m_footInfo.dz[side] = want[side];
            // the pelvis already lowered both feet rigidly by `drop` (both floated at least that much: none sinks below its target); a lift
            // is asked for as it is (the drop is 0 then), a foot that is only partly lowered stays on the pelvis' height
            const float dz = want[side] > 0.0f ? want[side] : std::fmin(0.0f, want[side] + drop);
            targets.feet[side].p = {0.0f, 0.0f, dz};
            any = any || std::fabs(dz) > 1.0e-6f;
        }
        if (!any)
            return;
        const auto result = m_feetWarp.Apply(m_out, targets);
        m_footInfo.applied = result.applied;
        m_footInfo.clamped[0] = result.feet[0].clamped;
        m_footInfo.clamped[1] = result.feet[1].clamped;
    }
    // ANKLE FLOOR (loco-fix 2026-11-09). The crossfade of local transforms moves the Hips height
    // linearly while a leg's vertical reach changes with the cosine of its blended angles, so a fade between a standing leg and a folded one
    // (sprint -> slide entry: Hips 88 -> 17 cm in 0.2 s, the support leg still long) pushes that foot through the floor: the replayed base put
    // the left ankle 21 cm under the lowest of its layers (sole -21 cm, -25 cm after the FootLocker), a layer of the blend never had it
    // there. Rule: the blend may not take an ankle more than kAnkleFloorTolerance below the LOWEST ankle of its live layers (their own
    // sampled / warped poses, m_ankleZ); the part below is lifted through the leg (the two-bone, rotation-only lift of KeepFootHeight, knee
    // plane kept). It never lowers anything, needs no floor height (the layers are the reference: a clip that has its foot below the floor
    // keeps it there) and is 0 on single-layer frames and wherever the blend stays above that bound, so those frames are bit for bit the
    // previous output. It acts on every fade, also the unflagged slide / air / traversal fades that KeepFootHeight leaves alone (its lift to
    // the MIXED height of the layers swung the thighs of non-planted feet, jump_pick slide entry pop 124 -> 245); here only the sink below all
    // layers is removed. Off with SetFootFloor(false) and in the "no height preservation" mutation 1.
    // GATE: off while the head layer is an air pick (take-off / fall, BlendRequest::toAir): the feet leave the ground and a lift through the
    // legs only swings the thighs faster (jump_pick slide -> jump: worst thigh rotation 52 -> 62 deg per 50 ms with the floor, bound 55). The
    // gate moves at most 1 / kAnkleFloorGateTime per second both ways (the lift is gate x dz), so neither an air pick nor the landing after it
    // steps the output.
    static constexpr float kAnkleFloorTolerance = 0.01f; // m: dips up to this below the lowest layer stay (blend noise, not a sink)
    static constexpr float kAnkleFloorGateTime = kFootGateFall; // s, full-scale rise / fall of the floor gate
    void KeepAnklesAboveLayers(float dt)
    {
        m_floorInfo = {};
#if UTPP_FOOTSINK_MUTATION == 11 // mutation 11: no ankle floor (the slide entry sink stays)
        (void)dt;
        return;
#else
        const float rate = dt / kAnkleFloorGateTime;
        const float perm = m_count > 0 && m_players[0].airFade ? 0.0f : 1.0f;
        m_floorGate = std::clamp(std::clamp(perm, m_floorGate - rate, m_floorGate + rate), 0.0f, 1.0f);
        m_floorInfo.gate = m_floorGate;
        if (!m_footFloor || m_count < 2 || !m_feetReady || m_ankleChain[0].empty() || m_ankleChain[1].empty() || !(m_floorGate > 0.0f))
            return;
        motionFeet::Targets targets{};
        targets.valid = true;
        targets.relative = true;
        targets.warpWeight = 1.0f;
        bool any = false;
        for (uint32_t side = 0; side < 2; ++side)
        {
            float lowest = m_ankleZ[0][side];
            for (uint32_t i = 1; i < m_count; ++i)
                lowest = std::fmin(lowest, m_ankleZ[i][side]);
            const float dz = lowest - kAnkleFloorTolerance - AnkleZ(m_out.data(), side);
            if (!std::isfinite(dz))
                return;
            if (dz > 0.0f)
            {
                m_floorInfo.dz[side] = m_floorGate * dz;
                targets.feet[side].p = {0.0f, 0.0f, m_floorGate * dz};
                any = true;
            }
        }
        if (!any)
            return;
        m_floorInfo.applied = m_feetWarp.Apply(m_out, targets).applied;
#endif
    }
    motionFeet::Targets MixTargets(const motionFeet::Targets &a, const motionFeet::Targets &b, const Player &player) const
    {
        motionFeet::Targets out{};
        out.valid = a.valid && b.valid && a.relative && b.relative;
        out.relative = true;
        for (uint32_t side = 0; side < 2; ++side)
        {
            const float weight = Alpha(player, m_footBones[side]);
            out.feet[side].p = math::Lerp(a.feet[side].p, b.feet[side].p, weight);
            out.feet[side].q = math::QuatNlerpShortest(a.feet[side].q, b.feet[side].q, weight);
            // These are relative corrections, not absolute ankle trajectories:
            // FK(nlerp(local poses)) is not lerp(FK(each pose)). The latter used
            // to pull the whole leg towards an unrelated trajectory as soon as
            // even a tiny outgoing warp activated IK. Correction magnitude now
            // tends continuously to zero with angle and player relevance.
            if ((weight < 1 && a.warpWeight > 0) || (weight > 0 && b.warpWeight > 0))
                out.warpWeight = 1;
        }
        return out;
    }
    bool ProcessGraph(Player &p, Signals &signal, float dt)
    {
        if (dt <= 0)
        {
            if (p.haveSteered)
                signal.rootYaw = p.lastSteeredYawRate;
            return true;
        }
        motionow::Inputs warp;
        warp.dt = dt;
        warp.rootdeltaX = signal.rootX * dt;
        warp.rootdeltaY = signal.rootY * dt;
        warp.rootdeltaYaw = signal.rootYaw * dt;
        warp.previousVisualWorldYaw = m_graph.previousVisualWorldYaw;
        warp.worldlocomotionX = m_graph.velocityWorldX;
        warp.worldlocomotionY = m_graph.velocityWorldY;
        // Actor +Y, projected into the previous visual-root frame (the graph's
        // fallback).
        const float fallbackYaw = m_graph.actorWorldYaw - m_graph.previousVisualWorldYaw;
        warp.fallbackLocalLocomotionX = -std::sin(fallbackYaw);
        warp.fallbackLocalLocomotionY = std::cos(fallbackYaw);
        warp.curvealpha = Curve(p, m_orientationCurve);
        const auto warped = motionow::Step(p.orientationState, warp);
        if (!warped.valid)
            return false;
        signal.rootX = warped.warpedRootdeltaX / dt;
        signal.rootY = warped.warpedRootdeltaY / dt;
        p.lastOrientationHalfAngle = warped.actualPoseAngle;
        root2::Delta prediction;
        motionSteer::Inputs steering;
        steering.dt = dt;
        steering.animationYawDelta = signal.rootYaw * dt;
        steering.rootSpeedMs = std::hypot(signal.rootX, signal.rootY);
        steering.previousVisualWorldYaw = m_graph.previousVisualWorldYaw;
        steering.targetWorldYaw = m_graph.targetWorldYaw;
        steering.havePrediction = root2::Extract(*m_db, p.clip, p.time, .5, prediction);
        steering.predictedAuthoredYaw = float(prediction.yaw);
        steering.alpha = p.generalSteering.UpdateEnable(m_graph.generalSteering, dt);
        const auto general = p.generalSteering.Update(steering);
        if (!general.valid)
            return false;
        steering.animationYawDelta = general.yawDelta;
        steering.turnInPlace = true;
        steering.alpha = p.tipSteering.UpdateEnable(m_graph.tipSteering, dt);
        // Each native node re-extracts the same RAW future asset, not the first
        // node's scaled prediction.
        const auto tip = p.tipSteering.Update(steering);
        if (!tip.valid)
            return false;
        signal.rootYaw = p.lastSteeredYawRate = tip.yawDelta / dt;
        p.haveSteered = true;
        return true;
    }
    float Curve(const Player &p, int32_t curve) const
    {
        if (curve < 0)
            return 0;
        const auto &cl = m_db->Clip(uint32_t(p.clip));
        const float ft = std::clamp(p.time * m_db->Header().fps, 0.f, float(cl.numFrames - 1));
        const uint32_t i = cl.firstFrame + uint32_t(ft), j = std::min(i + 1, cl.firstFrame + cl.numFrames - 1);
        const uint32_t bit = 1u << uint32_t(curve);
        const float a = (m_db->CurvePresence(i) & bit) ? m_db->CurveValue(i, uint32_t(curve)) : 0;
        const float z = (m_db->CurvePresence(j) & bit) ? m_db->CurveValue(j, uint32_t(curve)) : 0;
        return a + (z - a) * (ft - std::floor(ft));
    }
    float Alpha(const Player &p, int32_t bone) const
    {
        const float duration = p.duration * (bone < 0 ? 1 : m_factors[uint32_t(bone)]);
        if (duration <= 1e-8f)
            return 1;
        float t = std::clamp(p.clock / duration, 0.f, 1.f);
        return t * t * (3 - 2 * t);
    }
    float Advance(int32_t clip, float time) const
    {
        const auto &c = m_db->Clip(uint32_t(clip));
        const float fps = m_db->Header().fps;
        const float len = float(c.numFrames - 1) / fps;
        if ((c.flags & motion::kClipLoop) && c.loopPeriod)
        {
            const float period = float(c.loopPeriod) / fps;
            return time >= 0 ? std::fmod(time, period) : 0;
        }
        return std::clamp(time, 0.f, len);
    }
    bool IsLoop(int32_t clip) const
    {
        const auto &c = m_db->Clip(uint32_t(clip));
        return (c.flags & motion::kClipLoop) && c.loopPeriod;
    }
    static BoneTransform Mix(const BoneTransform &a, const BoneTransform &b, float w)
    {
        if (w <= 0)
            return a;
        if (w >= 1)
            return b;
        BoneTransform r;
        r.p = math::Lerp(a.p, b.p, w);
        auto q = b.q;
        if (math::QuatDot(a.q, q) < 0)
            q = math::QuatNeg(q);
        r.q = math::QuatNormalize({a.q.x + (q.x - a.q.x) * w, a.q.y + (q.y - a.q.y) * w, a.q.z + (q.z - a.q.z) * w,
                                   a.q.w + (q.w - a.q.w) * w});
        return r;
    }
    static Signals MixSignals(Signals a, Signals b, float w)
    {
        return {a.rootX + (b.rootX - a.rootX) * w, a.rootY + (b.rootY - a.rootY) * w,
                a.rootYaw + (b.rootYaw - a.rootYaw) * w, a.left + (b.left - a.left) * w,
                a.right + (b.right - a.right) * w};
    }
    Signals Sample(const Player &p) const
    {
        const auto &c = m_db->Clip(uint32_t(p.clip));
        float ft = std::clamp(p.time * m_db->Header().fps, 0.f, float(c.numFrames - 1));
        uint32_t i = uint32_t(ft), j = std::min(i + 1, c.numFrames - 1);
        float u = ft - i;
        const auto &a = m_db->Root(c.firstFrame + i);
        const auto &b = m_db->Root(c.firstFrame + j);
        const auto &l = m_db->Contact(c.firstFrame + i);
        const auto &r = m_db->Contact(c.firstFrame + j);
        const float scale = m_db->Header().fps * p.rootRate;
        return {(a.dx + (b.dx - a.dx) * u) * scale, (a.dy + (b.dy - a.dy) * u) * scale,
                (a.dyaw + (b.dyaw - a.dyaw) * u) * scale, l.l + (r.l - l.l) * u, l.r + (r.r - l.r) * u};
    }
    void Collapse()
    {
        // Collapse all indices4..end into index4, preserving their normalized tail.
        const uint32_t last = m_count - 1;
        for (uint32_t b = 0; b < m_n; ++b)
        {
            auto pose = m_pose[last][b];
            for (int i = int(last) - 1; i >= int(kLayers); --i)
                pose = Mix(pose, m_pose[uint32_t(i)][b], Alpha(m_players[uint32_t(i)], int32_t(b)));
            m_pose[kLayers][b] = pose;
        }
        for (uint32_t side = 0; side < 2; ++side)
        {
            float z = m_ankleZ[last][side];
            for (int i = int(last) - 1; i >= int(kLayers); --i)
                z = MixZ(z, m_ankleZ[uint32_t(i)][side], Alpha(m_players[uint32_t(i)], m_footBones[side]));
            m_ankleZ[kLayers][side] = z;
        }
        auto sig = m_samples[last];
        for (int i = int(last) - 1; i >= int(kLayers); --i)
            sig = MixSignals(sig, m_samples[uint32_t(i)], Alpha(m_players[uint32_t(i)], -1));
        m_samples[kLayers] = sig;
        if (m_graph.enabled)
        {
            auto targets = m_targets[last];
            for (int i = int(last) - 1; i >= int(kLayers); --i)
                targets = MixTargets(targets, m_targets[uint32_t(i)], m_players[uint32_t(i)]);
            m_targets[kLayers] = targets;
        }
        // Static tail signals are snapshots, NOT a faithful native graph root attribute.
        // Consumers must not integrate these as an accepted production root delta.
        bool stanceFade = false, groundFade = false;
        // Only a layer that still has one under it can be mid-fade: the last one is the base of the group, its own fade-in is long complete
        // (ORing it in made the stored tail a permanent stance fade: the lift then ran on every gait fade above it).
        for (uint32_t i = kLayers; i < last; ++i)
        {
            stanceFade = stanceFade || m_players[i].stanceFade;
            groundFade = groundFade || m_players[i].groundFade;
        }
#if UTPP_FOOTSINK_MUTATION == 3 // mutation 3: the base of the collapsed group counts as a stance fade
        stanceFade = stanceFade || m_players[last].stanceFade;
        groundFade = groundFade || m_players[last].groundFade;
#endif
        m_players[kLayers] = Player{};
        m_players[kLayers].stored = true;
        m_players[kLayers].stanceFade = stanceFade;
        m_players[kLayers].groundFade = groundFade;
        m_count = kLayers + 1;
        ++m_collapses;
    }
    void Evaluate()
    {
        for (uint32_t b = 0; b < m_n; ++b)
        {
            auto p = m_pose[m_count - 1][b];
            for (int i = int(m_count) - 2; i >= 0; --i)
                p = Mix(p, m_pose[uint32_t(i)][b], Alpha(m_players[uint32_t(i)], int32_t(b)));
            m_out[b] = p;
        }
        m_signals = m_samples[m_count - 1];
        for (int i = int(m_count) - 2; i >= 0; --i)
            m_signals = MixSignals(m_signals, m_samples[uint32_t(i)], Alpha(m_players[uint32_t(i)], -1));
        // A short cancelled start can still contain an older double-support idle
        // layer. Its scalar contact must not carry the non-support foot's old
        // lock through the actual start player's swing. During this admitted
        // fade, new support needs contact in both live source and destination.
        // The already selected supporting foot keeps its ordinary blended signal.
        // Use the sampled live layers (including their real rates/phases), never
        // reconstruct source time from the incoming clip or a nominal duration.
        if (m_count > 1 && m_players[0].cancelStartSupport >= 0 &&
            m_players[0].clock < m_players[0].duration && !m_players[1].stored)
        {
            if (m_players[0].cancelStartSupport == 1)
                m_signals.left = std::min(m_samples[0].left, m_samples[1].left);
            else
                m_signals.right = std::min(m_samples[0].right, m_samples[1].right);
        }
        if (m_graph.enabled)
        {
            m_outputTargets = m_targets[m_count - 1];
            for (int i = int(m_count) - 2; i >= 0; --i)
                m_outputTargets = MixTargets(m_outputTargets, m_targets[uint32_t(i)], m_players[uint32_t(i)]);
        }
    }
    const motion::MotionDatabase *m_db = nullptr;
    uint32_t m_n = 0, m_count = 0, m_collapses = 0;
    std::array<Player, kCapacity> m_players{};
    std::array<std::vector<BoneTransform>, kCapacity> m_pose;
    std::array<Signals, kCapacity> m_samples{};
    std::vector<float> m_factors;
    std::vector<BoneTransform> m_out;
    Signals m_signals{};
    GraphInputs m_graph{};
    int32_t m_orientationCurve = -1;
    motionPose::Warp m_poseWarp;
    motionFeet::Warp m_feetWarp;
    bool m_feetReady = false;
    int32_t m_footBones[2]{-1, -1};
    std::vector<int32_t> m_ankleChain[2]; // root -> foot bone indices per side
#if UTPP_FOOTSINK_MUTATION == 1
    bool m_footHeightKeep = false; // test mutation: no height preservation (the pre-fix crossfade)
    bool m_footFloor = false;
#else
    bool m_footHeightKeep = true;
    bool m_footFloor = true; // KeepAnklesAboveLayers
#endif
    FootFloorInfo m_floorInfo{};
    float m_floorGate = 1.0f; // applied KeepAnklesAboveLayers gate (both sides; 0 under an air pick)
    FootKeepInfo m_footInfo{};
    float m_footGate[2]{0.0f, 0.0f};   // applied KeepFootHeight gate per side (stance and ground fades: the lift)
    float m_stanceGate[2]{0.0f, 0.0f}; // applied KeepFootHeight gate per side of the stance fades alone (the lowering through the legs)
    int32_t m_hipsBone = -1;
    std::vector<int32_t> m_hipsChain; // root -> Hips bone indices
    std::array<motionFeet::Targets, kCapacity> m_targets{};
    std::array<std::array<float, 2>, kCapacity> m_ankleZ{}; // per layer: model-space ankle height of its own pose, left / right
    motionFeet::Targets m_outputTargets{};
};
} // namespace utpp::mm::motionstack

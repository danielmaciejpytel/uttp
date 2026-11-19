// Live outgoing-clip crossfade. Implements the documented blend contract.
#pragma once
#include "MotionAdapter.hpp"
#include "MotionOrientationFeet.hpp"
#include "MotionOrientationPose.hpp"
#include "MotionOrientationWarp.hpp"
#include "MotionSteering.hpp"
#include "MotionRootTrack.hpp"
#include "StrideWarp.hpp"
#include <array>
#include <span>
#ifndef UTPP_BLEND_SIGNAL_MODE
// Default of BlendStack::SetPerBoneSignals: bit 0 = root delta / yaw on the Root bone's weight, bit 1 = contacts on the leg weights,
// 0 = every signal on the whole-layer weight (default until the per-bone blend factors are settled; a build may override it).
#define UTPP_BLEND_SIGNAL_MODE 0
#endif
#ifndef UTPP_STRIDE_LAYER_FADE
// 1 = the stride warping part of a layer's foot correction fades on the layer's whole-layer weight (MixTargets); 0 = on the leg weight
// like the rest of the correction. A build may override it.
#define UTPP_STRIDE_LAYER_FADE 1
#endif
#ifndef UTPP_STRIDE_YAW_FADE
// 1 = stride warping fades out while the visual root keeps swinging (stridewarp::YawFade); 0 = no yaw fade. A build may override it.
#define UTPP_STRIDE_YAW_FADE 0
#endif
#ifndef UTPP_STRIDE_CROUCH
// Stride warping of crouched loops (loop clips named "_Crouch_"): 0 = like every other loop, 1 = not warped, 2 = warped with the yaw fade
// (the crouched gait sways the visual root in step with the stance foot; the sway alone moves a planted foot, which the warp then pushes).
// On recorded sessions 2 was the only mode with less crouched loop slide than no warping on both bodies; the yaw fade on every loop also
// fades the walk / run gait sway of the male V and loses its loop gain. A build may override it.
#define UTPP_STRIDE_CROUCH 2
#endif
#ifndef UTPP_STRIDE_KNEE_LIMIT
// 1 = stride warping raises the scale per leg so the knee folds at most a little further than the clip's own pose
// (stridewarp::KneeLimitedScale); 0 = both legs take the layer's scale. A build may override it.
#define UTPP_STRIDE_KNEE_LIMIT 1
#endif
#ifndef UTPP_STRIDE_LEG_RELEASE
// 1 = a leg's stride scale falls (towards a shorter stride) at most stridewarp::kLegReleaseRate per second, so the knee limit lets go
// of its shortening over a few updates instead of one (stridewarp::LimitLegRelease); 0 = the knee-limited scale at once. A build may
// override it.
#define UTPP_STRIDE_LEG_RELEASE 1
#endif
#ifndef UTPP_STRIDE_WARP
// Default of BlendStack::SetStrideWarp: 1 = shrink-only stride warping of the leg targets (StrideWarp.hpp) on ground locomotion picks,
// 0 = off (the pose is bit for bit the one without it). A build may override it.
#define UTPP_STRIDE_WARP 0
#endif
#ifndef UTPP_STEER_MODE
// Default steering mode of the blend stack (BlendStack::SetSteerMode, switched at run time by the motion driver): 0 = general (moving)
// steering on every live layer, 1 = on the newest layer only, 2 = on the newest layer only with its correction carried through the
// layer mix (kSteerCarried). A build may override it.
#define UTPP_STEER_MODE 1
#endif
#ifndef UTPP_STEER_CARRY_MUTATION
// Test mutation: 1 = kSteerCarried carries nothing (as kSteerNewest), 2 = it also carries a short-way residual beyond kCarryResidual.
#define UTPP_STEER_CARRY_MUTATION 0
#endif
#ifndef UTPP_FOOTSINK_MUTATION
#define UTPP_FOOTSINK_MUTATION 0
#endif
// ---- hand-back blend (dead-blended tail) ----
#include "BlendProfile.hpp"
#include "DeadBlending.hpp"
#include <cstdlib>
#ifndef UTPP_HANDBACK_DEAD_BLEND
// Default of BlendStack::SetHandBackBlend: 1 = a pose handed back from an external source (SeedPose with its previous pose) is
// extrapolated with its decaying velocity while the new clip fades in over the hand-back profile and time; 0 = a frozen tail with the
// locomotion profile and the matcher's blend time (the pose is bit for bit the one without it). A build may override it.
#define UTPP_HANDBACK_DEAD_BLEND 0
#endif
// ---- end hand-back blend ----
namespace utpp::mm::motionstack
{
#if defined(UTPP_MOTION_DRIVER_TEST_TRAV)
// Test targets only (the replay stage tool): settings every BlendStack takes at Init in place of its defaults, so a replay can select
// the steering values and the per-bone signal mode without a rebuild (set before the first Step; nothing changes while they stay unset),
// and a probe of the yaw the steering nodes apply to the head layer: per call |steered yaw - authored yaw| (rad), summed.
struct TestStackOverrides
{
    bool steering = false;
    float halfLife = 0, targetTime = 0;
    int32_t signalMode = -1; // -1 = the build default, else bit 0 root, bit 1 contacts (SetPerBoneSignals)
    int32_t handBack = -1;   // -1 = the build default or the UTPP_TEST_HANDBACK_BLEND environment value (0 / 1), else SetHandBackBlend
};
inline TestStackOverrides g_testStackOverrides{};
struct TestSteeringProbe
{
    uint64_t calls = 0;          // ProcessGraph calls of the head layer with dt > 0
    double appliedAbsRad = 0;    // summed |steered - authored| yaw of those calls
    double lastAppliedRad = 0;   // signed steered - authored yaw of the last call
    uint64_t lastCall = 0;       // calls at the last update (a reader sees whether this step steered)
};
inline TestSteeringProbe g_testSteeringProbe{};
#endif
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
    float generalSteerRate = 0; // yaw rate the general steering adds to this layer's own authored yaw (rad/s), see kSteerCarried
    bool generalCarry = false;  // that steering may be carried through the layer mix (kSteerCarried)
    bool haveSteered = false;
    int cancelStartSupport = -1;
    bool stanceFade = false; // this layer's fade-in is a ground stance change (crouch <-> stand), see KeepFootHeight
    bool groundFade = false; // this layer's fade-in is a ground locomotion -> ground locomotion fade (BlendRequest::groundFade), see KeepFootHeight
    bool airFade = false;    // this layer was picked in the air movement mode (BlendRequest::toAir), see KeepAnklesAboveLayers
    bool fullLegFade = false; // the leg bones of this layer fade over the whole blend time (BlendRequest::fullLegFade), see Alpha
    bool strideGround = false; // this layer was picked as ground locomotion in a ground movement mode (BlendRequest::toGround), see WarpStride
    stridewarp::State stride;  // stride scale of this layer, see WarpStride
    bool handBackFade = false; // this layer fades in over a hand-back tail with the hand-back profile, see Alpha
    bool deadTail = false;     // this stored layer is the extrapolated hand-back pose, see SeedPose
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
    using StrideOffset = std::array<math::Vec3, 2>; // stride warping offsets of the left / right ankle target, model space
    static constexpr float kFootGateFall = 0.12f; // s, full-scale fall time of the foot height gate
    // Steering of the root yaw (ProcessGraph): spring half-life of the general node and the animation lookahead both nodes extract the
    // authored turn over. The shipped default keeps the previous values (0.2 s / 0.5 s): the reference values were worse on pivots and crouch
    // loops in replay; SetSteering(kSteeringHalfLife, kSteeringTargetTime) selects them for comparison.
    static constexpr float kSteeringHalfLife = 0.4f, kSteeringTargetTime = 2.0f;
    static constexpr float kSteeringHalfLifePrevious = 0.2f, kSteeringTargetTimePrevious = 0.5f;
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
        m_rootBone = -1;
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
            if (db.BoneName(k) == "Root")
                m_rootBone = int32_t(k);
        }
        m_hipsChain.clear();
        for (int32_t b = m_hipsBone; b >= 0; b = db.BoneParent(uint32_t(b)))
            m_hipsChain.insert(m_hipsChain.begin(), b);
        m_warpCurve = -1;
        for (uint32_t k = 0; k < db.NumCurves(); ++k)
        {
            if (db.CurveName(k) == "Enable_OrientationWarping")
                m_orientationCurve = int32_t(k);
            if (db.CurveName(k) == "Enable_Warping")
                m_warpCurve = int32_t(k);
        }
        // Stride warping reads the clip's warping curve; data that names it after orientation warping carries the same curve.
        if (m_warpCurve < 0)
            m_warpCurve = m_orientationCurve;
        for (uint32_t side = 0; side < 2; ++side)
        {
            m_ankleChain[side].clear();
            for (int32_t b = m_footBones[side]; b >= 0; b = db.BoneParent(uint32_t(b)))
                m_ankleChain[side].insert(m_ankleChain[side].begin(), b);
        }
        // Leg bones = the bone below the Hips on each ankle chain (the thigh) and everything under it. A layer with fullLegFade fades them over
        // the whole blend time (factor 1); every other bone keeps its factor.
        m_fullLegFactors = m_factors;
        int32_t thigh[2]{-1, -1};
        for (uint32_t side = 0; side < 2; ++side)
            for (size_t k = 0; k + 1 < m_ankleChain[side].size(); ++k)
                if (m_ankleChain[side][k] == m_hipsBone)
                    thigh[side] = m_ankleChain[side][k + 1];
        m_thighBones[0] = thigh[0];
        m_thighBones[1] = thigh[1];
        InitStrideClipFactors();
        for (uint32_t k = 0; k < m_n; ++k)
            for (int32_t b = int32_t(k); b >= 0; b = db.BoneParent(uint32_t(b)))
                if (b == thigh[0] || b == thigh[1])
                {
                    m_fullLegFactors[k] = 1.0f;
                    break;
                }
#if defined(UTPP_MOTION_DRIVER_TEST_TRAV)
        if (g_testStackOverrides.steering)
            SetSteering(g_testStackOverrides.halfLife, g_testStackOverrides.targetTime);
        if (g_testStackOverrides.signalMode >= 0)
            SetPerBoneSignals((g_testStackOverrides.signalMode & 1) != 0, (g_testStackOverrides.signalMode & 2) != 0);
        if (g_testStackOverrides.handBack >= 0)
            SetHandBackBlend(g_testStackOverrides.handBack != 0);
        else if (const char *e = std::getenv("UTPP_TEST_HANDBACK_BLEND"); e && (e[0] == '0' || e[0] == '1'))
            SetHandBackBlend(e[0] == '1');
        // stride warping: UTPP_TEST_STRIDE_WARP (0 / 1), UTPP_TEST_STRIDE_OPTIONS = three digits: yaw fade (0 / 1), crouched loop mode
        // (0 / 1 / 2), whole-layer fade (0 / 1)
        if (const char *e = std::getenv("UTPP_TEST_STRIDE_WARP"); e && (e[0] == '0' || e[0] == '1'))
            SetStrideWarp(e[0] == '1');
        if (const char *e = std::getenv("UTPP_TEST_STRIDE_OPTIONS");
            e && (e[0] == '0' || e[0] == '1') && e[1] >= '0' && e[1] <= '2' && (e[2] == '0' || e[2] == '1'))
            SetStrideOptions(e[0] == '1', e[1] - '0', e[2] == '1');
        // UTPP_TEST_STRIDE_CROUCH_YAW = "start end" in deg/s: yaw fade rates of crouched loops (mode 2)
        if (const char *e = std::getenv("UTPP_TEST_STRIDE_CROUCH_YAW"))
        {
            char *rest = nullptr;
            const float a = std::strtof(e, &rest), b = std::strtof(rest, nullptr);
            if (std::isfinite(a) && std::isfinite(b) && b > a && a >= 0)
            {
                m_strideCrouchYawStart = a * 0.017453293f;
                m_strideCrouchYawEnd = b * 0.017453293f;
            }
        }
        // UTPP_TEST_STRIDE_KNEE = "margin free" in degrees (see stridewarp::KneeLimitedScale); a negative margin turns the knee limit off
        if (const char *e = std::getenv("UTPP_TEST_STRIDE_KNEE"))
        {
            char *rest = nullptr;
            const float a = std::strtof(e, &rest), b = std::strtof(rest, nullptr);
            if (std::isfinite(a) && std::isfinite(b) && b >= 0)
            {
                m_strideKneeLimit = a >= 0;
                m_strideKneeMargin = std::fmax(a, 0.0f) * 0.017453293f;
                m_strideKneeFree = b * 0.017453293f;
            }
        }
        // UTPP_TEST_STRIDE_LEG_RELEASE = 0 / 1: the rate-limited release of the knee limit off / on (see stridewarp::LimitLegRelease)
        if (const char *e = std::getenv("UTPP_TEST_STRIDE_LEG_RELEASE"); e && (e[0] == '0' || e[0] == '1'))
            m_strideLegRelease = e[0] == '1';
#endif
        m_handBackFactors = blendprofile::Resolve(blendprofile::kHandBack, m_n, [&](uint32_t b) { return db.BoneName(b); });
        m_deadPrev.resize(m_n);
        m_deadCurr.resize(m_n);
        m_deadDst.resize(m_n);
        return true;
    }
    // Hand-back from an external pose (see UTPP_HANDBACK_DEAD_BLEND).
    void SetHandBackBlend(bool on)
    {
        m_handBack = on;
    }
    bool HandBackBlend() const
    {
        return m_handBack;
    }
    // Bridge with velocity: the last emitted external pose, the one emitted dt before it. With the hand-back blend off this is exactly
    // SeedPose(pose, signals); on, the stored layer is extrapolated each Step (dead blending) and the next pick fades in over the
    // hand-back profile and time.
    bool SeedPose(std::span<const BoneTransform> pose, std::span<const BoneTransform> previous, float dt, Signals signals = {})
    {
        if (!SeedPose(pose, signals))
            return false;
        if (!m_handBack || previous.size() != m_n || !std::isfinite(dt) || dt <= 0)
            return true;
        for (uint32_t b = 0; b < m_n; ++b)
        {
            m_deadCurr[b] = ToDead(pose[b]);
            m_deadPrev[b] = ToDead(previous[b]);
        }
        deadblend::Settings s;
        s.maxTranslationVelocity = 5.0f; // m/s (the node's 500 cm/s)
        m_dead.Start(m_deadCurr, m_deadPrev, dt, s);
        m_deadTime = 0;
        m_deadHalfLivesPending = true;
        m_handBackPending = true;
        m_players[0].deadTail = true;
        return true;
    }
    void SetGraphInputs(const GraphInputs &input)
    {
        m_graph = input;
    }
    // Steering parameters (see kSteeringHalfLife). Non-finite or non-positive values are rejected and leave the current ones.
    bool SetSteering(float halfLife, float targetTime)
    {
        if (!std::isfinite(halfLife) || halfLife <= 0 || !std::isfinite(targetTime) || targetTime <= 0 || targetTime > 10)
            return false;
        m_steeringHalfLife = halfLife;
        m_steeringTargetTime = targetTime;
        return true;
    }
    // Root-motion signals of the mix (OutputSignals). root: the root delta and yaw fade with the Root bone's weight; contacts: each foot's
    // contact fades with its leg's weight, so they reach the new clip together with the bones that carry them. Off: the signal fades with
    // the whole-layer weight (the previous mix).
    void SetPerBoneSignals(bool root, bool contacts)
    {
        m_perBoneRoot = root;
        m_perBoneContacts = contacts;
    }
    bool PerBoneRoot() const
    {
        return m_perBoneRoot;
    }
    bool PerBoneContacts() const
    {
        return m_perBoneContacts;
    }
    // Steering modes of the general (moving) steering (default UTPP_STEER_MODE). kSteerAll: every live layer steers. kSteerNewest:
    // only the newest layer's general steering is enabled, as in the reference graph; its correction reaches the output at the layer's
    // fade weight, so the body lags the capsule through a long fade. kSteerCarried (project policy, not the reference behaviour): as kSteerNewest, and the yaw the newest layer's
    // general steering adds to its authored yaw (scaled authored turn and spring correction) reaches the output at full weight, while
    // the older layers' authored yaw still fades out under it. Not carried while the target is more than kCarryResidual from the
    // visual root and the newest clip has no authored turn ahead to take the residual side from (see ProcessGraph).
    static constexpr int32_t kSteerAll = 0, kSteerNewest = 1, kSteerCarried = 2;
    static constexpr float kCarryResidual = .75f * math::kPi;
    // In kSteerNewest / kSteerCarried an older layer's general steering fades out over the enable blend once a newer pick covers it; the
    // turn-in-place node is unchanged in every mode. Safe to switch at any time (each layer's steering weight fades over the enable
    // blend). Invalid modes are rejected and leave the current one.
    bool SetSteerMode(int32_t mode)
    {
        if (mode < kSteerAll || mode > kSteerCarried)
            return false;
        m_steerMode = mode;
        return true;
    }
    int32_t SteerMode() const
    {
        return m_steerMode;
    }
    float SteeringHalfLife() const
    {
        return m_steeringHalfLife;
    }
    float SteeringTargetTime() const
    {
        return m_steeringTargetTime;
    }
    // Shrink-only stride warping of the leg targets (see WarpStride). Default UTPP_STRIDE_WARP (off); off reproduces the pose bit for bit.
    // Safe to switch at any time: switched on under live layers, each layer's stride scale starts at 1 and shortens at the interpolation
    // rate (a fresh pick takes its target at once and fades in with its layer, as always); switched off, the shortening the output
    // still carries is released at the interpolation rate (LimitStrideRelease) and the pose is the unwarped one once it reaches 0.
    void SetStrideWarp(bool enabled)
    {
        if (enabled && !m_strideWarp)
        {
            for (auto &p : m_players)
            {
                p.stride.scale = 1.0f;
                p.stride.initialized = true;
            }
            m_strideYawRate = 0.0f;
            m_strideYawValid = false;
        }
        m_strideWarp = enabled;
    }
    bool StrideWarp() const
    {
        return m_strideWarp;
    }
    // Knee limit of stride warping in force (see SetStrideKneeLimit).
    bool StrideKneeLimit() const
    {
        return m_strideKneeLimit;
    }
    // Stride warping details (defaults UTPP_STRIDE_YAW_FADE, UTPP_STRIDE_CROUCH, UTPP_STRIDE_LAYER_FADE): yaw fade on every loop,
    // crouched loop mode (0 like the others, 1 not warped, 2 yaw fade), stride offsets fading on the whole-layer weight.
    void SetStrideOptions(bool yawFade, int32_t crouchMode, bool layerFade)
    {
        m_strideYawFade = yawFade;
        m_strideCrouchMode = std::clamp(crouchMode, 0, 2);
        m_strideLayerFade = layerFade;
    }
    // Knee limit of stride warping (default UTPP_STRIDE_KNEE_LIMIT, see LegStrideScale): on / off, margin and free flexion in radians.
    void SetStrideKneeLimit(bool enabled, float margin = stridewarp::kKneeMargin, float freeFlexion = stridewarp::kKneeFreeFlexion)
    {
        m_strideKneeLimit = enabled;
        m_strideKneeMargin = std::isfinite(margin) ? std::fmax(margin, 0.0f) : stridewarp::kKneeMargin;
        m_strideKneeFree = std::isfinite(freeFlexion) ? std::fmax(freeFlexion, 0.0f) : stridewarp::kKneeFreeFlexion;
    }
    // Rate-limited release of the knee limit (default UTPP_STRIDE_LEG_RELEASE, see stridewarp::LimitLegRelease).
    void SetStrideLegRelease(bool enabled)
    {
        m_strideLegRelease = enabled;
    }
    // A loop clip of the crouched gait (named "_Crouch_"), see UTPP_STRIDE_CROUCH.
    bool StrideCrouchClip(int32_t clip) const
    {
        return clip >= 0 && size_t(clip) < m_strideCrouchClip.size() && m_strideCrouchClip[size_t(clip)] != 0;
    }
    // Planted-foot speed over root speed of a clip (stridewarp::ClipFactor), measured at Init; 1 for a clip that is not a loop.
    float StrideClipFactor(int32_t clip) const
    {
        return clip >= 0 && size_t(clip) < m_strideClipFactor.size() ? m_strideClipFactor[size_t(clip)] : 1.0f;
    }
    // Effective stride scale of the head layer in the last Step (1 = no warp).
    float StrideScale() const
    {
        return m_strideScale;
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
        m_strideHeld = 0.0f;
        m_strideYawValid = false;
        m_count = 0;
        m_collapses = 0;
        m_handBackPending = m_deadHalfLivesPending = false;
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
        m_strideOffset[0] = {};
        m_strideAmount[0] = 0.0f;
        m_strideHeld = 0.0f;
        m_strideYawValid = false;
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
                    m_strideOffset[i] = m_strideOffset[i - 1];
                    m_strideAmount[i] = m_strideAmount[i - 1];
                    m_strideDir[i] = m_strideDir[i - 1];
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
#if UTPP_FOOTSINK_MUTATION != 12 // mutation 12: the leg bones keep their shorter fade on a flagged pick
            p.fullLegFade = selected.blend.fullLegFade;
#endif
            p.strideGround = selected.blend.toGround;
            if (m_handBackPending && m_count > 1)
            {
                // the first pick over a dead-blended hand-back tail: hand-back profile and time
                p.handBackFade = true;
                p.fullLegFade = false;
                p.duration = blendprofile::kBlendHandBack;
                m_handBackPending = false;
            }
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
        m_strideScale = 1.0f;
        if (m_strideWarp && (m_strideYawFade || m_strideCrouchMode == 2) && m_graph.enabled)
        {
            if (m_strideYawValid)
                m_strideYawRate = stridewarp::SmoothYawRate(m_strideYawRate, m_graph.previousVisualWorldYaw - m_strideLastYaw, dt);
            m_strideLastYaw = m_graph.previousVisualWorldYaw;
            m_strideYawValid = true;
        }
        for (uint32_t i = 0; i < m_count; ++i)
        {
            auto &p = m_players[i];
            p.clock += dt * (i == 0 ? 1 : std::pow(1.1f, float(i + 1)));
            if (!p.stored || p.deadTail)
            {
                m_strideOffset[i] = {}; // a sampled layer warps afresh below; the dead-blended tail is never warped
                m_strideAmount[i] = 0.0f;
            }
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
                    const float rootSpeed = std::hypot(m_samples[i].rootX, m_samples[i].rootY); // clip root at its rate
                    const float rawRootX = m_samples[i].rootX, rawRootY = m_samples[i].rootY;   // before orientation warping
                    if (!m_feetReady || !ProcessGraph(p, m_samples[i], dt) ||
                        !m_feetWarp.Sample(m_pose[i], p.lastOrientationHalfAngle, m_targets[i]) ||
                        !m_poseWarp.Apply(m_pose[i], p.lastOrientationHalfAngle))
                        return false;
                    if (m_strideWarp)
                        WarpStride(p, i, rootSpeed, rawRootX, rawRootY, dt);
                    if (!m_feetWarp.MakeRelative(m_pose[i], m_targets[i]))
                        return false;
                }
                CacheAnkleZ(i);
            }
            else if (p.deadTail && !StepDeadTail(i, dt))
                return false;
        }
        // Consolidate the least-significant live tail at its current sampled time.
        if (m_count > kLayers + 1)
            Collapse();
        Evaluate();
        if (m_graph.enabled)
            LimitStrideRelease(dt);
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
    // Mixes the relative corrections of the layers below (a, with their mixed stride offsets sa) with layer b (stride offsets sb) and
    // writes the mixed stride offsets to outStride (may alias sa or sb). The stride offset part of a correction fades on the layer's
    // whole-layer weight, the rest on its leg weight: the leg bones fade faster than the body, and a warped outgoing layer's shortening
    // cut on the leg weight dropped by up to 11 cm in one update, which snapped the knee. Without stride offsets on either side the
    // mix is the plain leg-weight mix, bit for bit.
    motionFeet::Targets MixTargets(const motionFeet::Targets &a, const StrideOffset &sa, const motionFeet::Targets &b,
                                   const StrideOffset &sb, const Player &player, StrideOffset &outStride) const
    {
        motionFeet::Targets out{};
        out.valid = a.valid && b.valid && a.relative && b.relative;
        out.relative = true;
        StrideOffset stride{};
        bool strideLeft = false;
        for (uint32_t side = 0; side < 2; ++side)
        {
            const float weight = Alpha(player, m_footBones[side]);
            if (!m_strideLayerFade || (IsZero(sa[side]) && IsZero(sb[side])))
                out.feet[side].p = math::Lerp(a.feet[side].p, b.feet[side].p, weight);
            else
            {
                stride[side] = math::Lerp(sa[side], sb[side], Alpha(player, -1));
                out.feet[side].p = math::Lerp(a.feet[side].p - sa[side], b.feet[side].p - sb[side], weight) + stride[side];
                strideLeft = strideLeft || !IsZero(stride[side]);
            }
            out.feet[side].q = math::QuatNlerpShortest(a.feet[side].q, b.feet[side].q, weight);
            // These are relative corrections, not absolute ankle trajectories:
            // FK(nlerp(local poses)) is not lerp(FK(each pose)). The latter used
            // to pull the whole leg towards an unrelated trajectory as soon as
            // even a tiny outgoing warp activated IK. Correction magnitude now
            // tends continuously to zero with angle and player relevance.
            if ((weight < 1 && a.warpWeight > 0) || (weight > 0 && b.warpWeight > 0))
                out.warpWeight = 1;
        }
        if (strideLeft)
            out.warpWeight = 1; // a stride offset still fading out keeps the correction active
        outStride = stride;
        return out;
    }
    static bool IsZero(const math::Vec3 &v)
    {
        return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
    }
    // Shrink-only stride warping (StrideWarp.hpp) of layer i: its absolute ankle targets (after orientation warping, before they become
    // corrections) are pulled towards the thighs of its warped pose along its stride direction, by capsule speed / clip root speed,
    // gated by the clip's warping curve and allowed on ground locomotion loops only (an air or slide pick, a traversal montage and a
    // disabled graph never warp). Starts, stops, pivots and transitions are left alone: on recorded sessions warping them made start
    // and pivot slide worse (their warping curve does not mark the parts where the capsule lags the clip). An identity scale leaves the
    // targets untouched.
    void WarpStride(Player &p, uint32_t i, float rootSpeed, float rawRootX, float rawRootY, float dt)
    {
        stridewarp::Inputs in;
        in.dt = dt;
        in.capsuleSpeed = std::hypot(m_graph.velocityWorldX, m_graph.velocityWorldY);
        in.rootSpeed = rootSpeed;
        in.clipFactor = StrideClipFactor(p.clip);
        in.alpha = m_warpCurve >= 0 ? Curve(p, m_warpCurve) : 1.0f;
        const bool crouched = StrideCrouchClip(p.clip);
        in.allowed = p.strideGround && !p.airFade && IsLoop(p.clip) && !(crouched && m_strideCrouchMode == 1);
        if (crouched && m_strideCrouchMode == 2)
            in.yawFade = stridewarp::YawFade(m_strideYawRate, m_strideCrouchYawStart, m_strideCrouchYawEnd);
        else if (m_strideYawFade)
            in.yawFade = stridewarp::YawFade(m_strideYawRate);
        const float scale = stridewarp::Step(p.stride, in);
        // Stride direction: the clip root turned by the angle orientation warping turned this layer's legs and feet (twice the pose half
        // angle), not the warped root signal: that one turns by the full target angle at once and swung the shortening sideways by up to
        // 90 degrees in one update when a turn pick changed the visual root under a warped loop.
        const float poseAngle = 2.0f * p.lastOrientationHalfAngle;
        if (i == 0)
            m_strideScale = scale;
        math::Vec3 direction;
        if (stridewarp::IsIdentity(scale) || !m_targets[i].valid || m_thighBones[0] < 0 || m_thighBones[1] < 0 ||
            !stridewarp::Direction(std::cos(poseAngle) * rawRootX - std::sin(poseAngle) * rawRootY,
                                   std::sin(poseAngle) * rawRootX + std::cos(poseAngle) * rawRootY, direction))
        {
            p.stride.leg[0] = p.stride.leg[1] = 1.0f; // not warped in this update: a later warp falls from 1 (LimitLegRelease)
            p.stride.legValid = true;
            return;
        }
        for (uint32_t side = 0; side < 2; ++side)
        {
            // Model-space thigh of the warped pose: FK along the ankle chain down to the thigh.
            math::Quat q{0, 0, 0, 1};
            math::Vec3 origin{0, 0, 0};
            const auto &chain = m_ankleChain[side];
            size_t k = 0;
            for (; k < chain.size(); ++k)
            {
                const int32_t b = chain[k];
                origin = origin + math::QuatRotate(q, m_pose[i][b].p);
                if (b == m_thighBones[side])
                    break;
                q = math::QuatNormalize(math::QuatMul(q, m_pose[i][b].q));
            }
            auto &foot = m_targets[i].feet[side].p;
            float legScale = LegStrideScale(m_pose[i], chain, k, foot, origin, direction, scale);
            if (m_strideLegRelease && p.stride.legValid)
                legScale = stridewarp::LimitLegRelease(p.stride.leg[side], legScale, dt);
            p.stride.leg[side] = legScale;
            const math::Vec3 offset = stridewarp::WarpOffset(foot, origin, direction, legScale);
            foot = foot + offset;
            m_strideOffset[i][side] = offset;
        }
        p.stride.legValid = true;
        m_strideAmount[i] = 1.0f - scale;
        m_strideDir[i] = direction;
        m_targets[i].warpWeight = 1.0f;
    }
    // Stride scale of one leg (see stridewarp::KneeLimitedScale): the thigh is chain[k], the knee and the ankle the next two chain bones of
    // the pose; without them, or with the knee limit off, the layer's scale.
    float LegStrideScale(const std::vector<BoneTransform> &pose, const std::vector<int32_t> &chain, size_t k, math::Vec3 ankle,
                         math::Vec3 thigh, math::Vec3 direction, float scale) const
    {
        if (!m_strideKneeLimit || k + 2 >= chain.size())
            return scale;
        const math::Vec3 shinOffset = pose[size_t(chain[k + 1])].p, ankleOffset = pose[size_t(chain[k + 2])].p;
        const math::Vec3 r = ankle - thigh;
        return stridewarp::KneeLimitedScale(scale, math::Dot(r, direction), std::sqrt(math::Dot(r, r)),
                                            std::sqrt(math::Dot(shinOffset, shinOffset)), std::sqrt(math::Dot(ankleOffset, ankleOffset)),
                                            m_strideKneeMargin, m_strideKneeFree);
    }
    // Clip factor of every loop clip (the clips stride warping applies to): over the frame pairs where a foot's contact is set at both ends,
    // the backward motion of its toe (the foot when the rig has no toe) along the clip's root direction in the character frame, summed
    // over the root motion of the same pairs. The rest of the clips keep 1.
    void InitStrideClipFactors()
    {
        const auto &db = *m_db;
        m_strideClipFactor.assign(db.NumClips(), 1.0f);
        m_strideCrouchClip.assign(db.NumClips(), 0);
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (IsLoop(int32_t(c)) && db.ClipName(c).find("_Crouch_") != std::string_view::npos)
                m_strideCrouchClip[c] = 1;
        int32_t toe[2]{m_footBones[0], m_footBones[1]};
        for (uint32_t k = 0; k < db.NumBones(); ++k)
        {
            if (db.BoneName(k) == "LeftToeBase")
                toe[0] = int32_t(k);
            if (db.BoneName(k) == "RightToeBase")
                toe[1] = int32_t(k);
        }
        if (toe[0] < 0 || toe[1] < 0)
            return;
        std::vector<int32_t> chain[2];
        for (uint32_t side = 0; side < 2; ++side)
            for (int32_t b = toe[side]; b >= 0; b = db.BoneParent(uint32_t(b)))
                chain[side].insert(chain[side].begin(), b);
        const auto toePosition = [&](uint32_t frame, uint32_t side) {
            const motion::PoseRecord *pose = db.Pose(frame);
            math::Quat q{0, 0, 0, 1};
            math::Vec3 o{0, 0, 0};
            for (const int32_t b : chain[side])
            {
                const auto &r = pose[b];
                o = o + math::QuatRotate(q, math::Vec3{r.px, r.py, r.pz});
                q = math::QuatNormalize(math::QuatMul(q, math::Quat{r.qx, r.qy, r.qz, r.qw}));
            }
            return o;
        };
        for (uint32_t c = 0; c < db.NumClips(); ++c)
        {
            if (!IsLoop(int32_t(c)))
                continue;
            const auto &clip = db.Clip(c);
            double back = 0, root = 0;
            int samples = 0;
            for (uint32_t f = 0; f + 1 < clip.numFrames; ++f)
            {
                const uint32_t a = clip.firstFrame + f, b = a + 1;
                const float dx = 0.5f * (db.Root(a).dx + db.Root(b).dx), dy = 0.5f * (db.Root(a).dy + db.Root(b).dy);
                const float len = std::hypot(dx, dy);
                if (!std::isfinite(len) || len <= 1.0e-5f)
                    continue;
                for (uint32_t side = 0; side < 2; ++side)
                {
                    const float ca = side ? db.Contact(a).r : db.Contact(a).l, cb = side ? db.Contact(b).r : db.Contact(b).l;
                    if (ca < 0.5f || cb < 0.5f)
                        continue;
                    const math::Vec3 d = toePosition(b, side) - toePosition(a, side);
                    back += -(d.x * dx + d.y * dy) / len;
                    root += len;
                    ++samples;
                }
            }
            m_strideClipFactor[c] = stridewarp::ClipFactor(back, root, samples);
        }
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
        steering.halfLife = m_steeringHalfLife;
        steering.havePrediction = root2::Extract(*m_db, p.clip, p.time, m_steeringTargetTime, prediction);
        steering.predictedAuthoredYaw = float(prediction.yaw);
        steering.alpha = p.generalSteering.UpdateEnable(m_graph.generalSteering && (m_steerMode == kSteerAll || &p == &m_players[0]), dt);
        const auto general = p.generalSteering.Update(steering);
        if (!general.valid)
            return false;
        p.generalSteerRate = math::WrapAngle(general.yawDelta - steering.animationYawDelta) / dt;
        // Not carried with the target more than kCarryResidual away unless the residual side came from the clip's authored turn ahead
        // (the steering node takes it from there when the turn is larger than 1 deg; without it an about 180 deg residual is wrapped
        // the short way, which may be against the turn the clip is about to play).
        const bool predicted =
            steering.havePrediction && std::fabs(math::WrapAngle(steering.predictedAuthoredYaw)) > math::kPi / 180.f;
#if UTPP_STEER_CARRY_MUTATION == 2
        (void)predicted;
        p.generalCarry = true;
#else
        p.generalCarry =
            predicted || std::fabs(math::WrapAngle(steering.targetWorldYaw - steering.previousVisualWorldYaw)) <= kCarryResidual;
#endif
        steering.animationYawDelta = general.yawDelta;
        steering.turnInPlace = true;
        steering.alpha = p.tipSteering.UpdateEnable(m_graph.tipSteering, dt);
        // Each native node re-extracts the same RAW future asset, not the first
        // node's scaled prediction.
        const auto tip = p.tipSteering.Update(steering);
        if (!tip.valid)
            return false;
#if defined(UTPP_MOTION_DRIVER_TEST_TRAV)
        if (&p == &m_players[0])
        {
            const double applied = double(tip.yawDelta) - double(signal.rootYaw) * double(dt);
            ++g_testSteeringProbe.calls;
            g_testSteeringProbe.appliedAbsRad += std::fabs(applied);
            g_testSteeringProbe.lastAppliedRad = applied;
            g_testSteeringProbe.lastCall = g_testSteeringProbe.calls;
        }
#endif
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
    // Fade weight of a layer for one bone (-1 = the layer as a whole). The per-bone factors shorten the fade of some bones (the legs run
    // faster than the Hips); a fullLegFade layer runs its leg bones over the whole blend time instead: two leg poses far apart (the two
    // slide families) otherwise swing the thigh and shin through their difference in a third less time than the rest of the body.
    float Alpha(const Player &p, int32_t bone) const
    {
        const float factor = bone < 0             ? 1.0f
                             : p.handBackFade     ? m_handBackFactors[uint32_t(bone)]
                             : p.fullLegFade      ? m_fullLegFactors[uint32_t(bone)]
                                                  : m_factors[uint32_t(bone)];
        const float duration = p.duration * factor;
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
        return MixSignals(a, b, w, w, w);
    }
    // root = weight of the root delta / yaw, left / right = weight of each foot's contact.
    static Signals MixSignals(Signals a, Signals b, float root, float left, float right)
    {
        return {a.rootX + (b.rootX - a.rootX) * root, a.rootY + (b.rootY - a.rootY) * root,
                a.rootYaw + (b.rootYaw - a.rootYaw) * root, a.left + (b.left - a.left) * left,
                a.right + (b.right - a.right) * right};
    }
    // Signal mix of one layer over the layers under it (see SetPerBoneSignals).
    Signals MixLayerSignals(const Signals &under, const Signals &layer, const Player &p) const
    {
        const float whole = Alpha(p, -1);
        return MixSignals(under, layer, m_perBoneRoot ? Alpha(p, m_rootBone) : whole,
                          m_perBoneContacts ? Alpha(p, m_footBones[0]) : whole, m_perBoneContacts ? Alpha(p, m_footBones[1]) : whole);
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
    // ---- hand-back blend (dead-blended tail) ----
    static deadblend::BoneTR ToDead(const BoneTransform &b)
    {
        return {{b.p.x, b.p.y, b.p.z}, {b.q.x, b.q.y, b.q.z, b.q.w}};
    }
    // Stored layer i is the hand-back pose: advance its extrapolation clock and resample it (the head, layer 0, is already sampled this
    // Step and is the destination the half-lives are taken from on the first call).
    bool StepDeadTail(uint32_t i, float dt)
    {
        if (m_dead.Size() != m_n)
            return false;
        if (m_deadHalfLivesPending && i > 0)
        {
            for (uint32_t b = 0; b < m_n; ++b)
                m_deadDst[b] = ToDead(m_pose[0][b]);
            m_dead.SetDestination(m_deadDst);
            m_deadHalfLivesPending = false;
        }
        m_deadTime += dt;
        for (uint32_t b = 0; b < m_n; ++b)
        {
            const auto e = m_dead.Sample(b, m_deadTime);
            const math::Quat q = math::QuatNormalize(math::Quat{e.r.x, e.r.y, e.r.z, e.r.w});
            if (!std::isfinite(e.t.x) || !std::isfinite(e.t.y) || !std::isfinite(e.t.z) || !math::IsFinite(q))
                return false;
            m_pose[i][b] = BoneTransform{math::Vec3{e.t.x, e.t.y, e.t.z}, q};
        }
        CacheAnkleZ(i);
        if (m_feetReady &&
            (!m_feetWarp.Sample(m_pose[i], 0, m_targets[i]) || !m_feetWarp.MakeRelative(m_pose[i], m_targets[i])))
            return false;
        return true;
    }
    // ---- end hand-back blend ----
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
            sig = MixLayerSignals(sig, m_samples[uint32_t(i)], m_players[uint32_t(i)]);
        m_samples[kLayers] = sig;
        if (m_graph.enabled)
        {
            auto targets = m_targets[last];
            auto stride = m_strideOffset[last];
            for (int i = int(last) - 1; i >= int(kLayers); --i)
                targets = MixTargets(targets, stride, m_targets[uint32_t(i)], m_strideOffset[uint32_t(i)], m_players[uint32_t(i)], stride);
            m_targets[kLayers] = targets;
            m_strideOffset[kLayers] = stride;
            float amount = m_strideAmount[last];
            math::Vec3 along = m_strideDir[last] * amount;
            for (int i = int(last) - 1; i >= int(kLayers); --i)
            {
                const float w = Alpha(m_players[uint32_t(i)], -1);
                amount = amount + (m_strideAmount[uint32_t(i)] - amount) * w;
                along = math::Lerp(along, m_strideDir[uint32_t(i)] * m_strideAmount[uint32_t(i)], w);
            }
            m_strideAmount[kLayers] = amount;
            stridewarp::Direction(along.x, along.y, m_strideDir[kLayers]);
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
            m_signals = MixLayerSignals(m_signals, m_samples[uint32_t(i)], m_players[uint32_t(i)]);
#if UTPP_STEER_CARRY_MUTATION != 1
        if (m_steerMode == kSteerCarried && m_graph.enabled && m_count > 1 && !m_players[0].stored && m_players[0].generalCarry)
        {
            // kSteerCarried: the newest layer's general steering reaches the output at full weight; the older layers' authored yaw
            // still fades out under it.
            const auto &head = m_players[0];
            const float weight = m_perBoneRoot ? Alpha(head, m_rootBone) : Alpha(head, -1);
            m_signals.rootYaw += (1.0f - weight) * head.generalSteerRate;
        }
#endif
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
            auto stride = m_strideOffset[m_count - 1];
            for (int i = int(m_count) - 2; i >= 0; --i)
                m_outputTargets = MixTargets(m_outputTargets, stride, m_targets[uint32_t(i)], m_strideOffset[uint32_t(i)],
                                             m_players[uint32_t(i)], stride);
            float amount = m_strideAmount[m_count - 1];
            math::Vec3 along = m_strideDir[m_count - 1] * amount;
            for (int i = int(m_count) - 2; i >= 0; --i)
            {
                const float w = Alpha(m_players[uint32_t(i)], -1);
                amount = amount + (m_strideAmount[uint32_t(i)] - amount) * w;
                along = math::Lerp(along, m_strideDir[uint32_t(i)] * m_strideAmount[uint32_t(i)], w);
            }
            m_strideMixed = amount;
            if (amount > 0.0f)
                stridewarp::Direction(along.x, along.y, m_strideHeldDir); // the held part keeps the last mixed direction
        }
    }
    // The mixed shortening of the output (1 - stride scale, mixed on the whole-layer weights) may grow at once (each layer's scale already
    // moves at the interpolation rate) but is released no faster than that rate: a short blend to an unwarped pick (a turn out of a warped
    // loop) or the pruning of the warped layer would otherwise drop it by up to 11 cm of ankle offset in one update and snap the knee. The
    // held part shortens the output's own stride: its ankle targets are pulled towards its thighs along the last mixed stride direction.
    // Without a held part nothing changes. With stride warping switched off the held part keeps being released at the same rate (no
    // layer adds to it any more), so switching it off never drops the shortening in one update.
    void LimitStrideRelease(float dt)
    {
        if ((!m_strideWarp && !(m_strideHeld > 0.0f)) || !m_outputTargets.valid || m_thighBones[0] < 0 || m_thighBones[1] < 0)
        {
            m_strideHeld = 0.0f;
            return;
        }
        const float keep = std::isfinite(dt) && dt > 0 ? std::exp(-stridewarp::kInterpSpeed * dt) : 1.0f;
        const float held = std::fmax(m_strideMixed, m_strideHeld * keep);
        m_strideHeld = held > 1.0e-4f ? held : 0.0f;
        const float extra = m_strideHeld - m_strideMixed;
        const math::Vec3 direction = m_strideHeldDir;
        if (!(extra > 0.0f) || !(math::Dot(direction, direction) > 0.5f))
            return;
        for (uint32_t side = 0; side < 2; ++side)
        {
            // Model-space thigh and ankle of the mixed pose: FK along the ankle chain.
            math::Quat q{0, 0, 0, 1};
            math::Vec3 pos{0, 0, 0}, thigh{0, 0, 0};
            const auto &chain = m_ankleChain[side];
            size_t thighAt = chain.size();
            for (size_t k = 0; k < chain.size(); ++k)
            {
                const int32_t b = chain[k];
                pos = pos + math::QuatRotate(q, m_out[uint32_t(b)].p);
                q = math::QuatNormalize(math::QuatMul(q, m_out[uint32_t(b)].q));
                if (b == m_thighBones[side])
                {
                    thigh = pos;
                    thighAt = k;
                }
            }
            auto &corr = m_outputTargets.feet[side].p;
            const float legScale = LegStrideScale(m_out, chain, thighAt, pos + corr, thigh, direction, 1.0f - extra);
            corr = corr + stridewarp::WarpOffset(pos + corr, thigh, direction, legScale);
        }
        m_outputTargets.warpWeight = 1.0f;
    }
    const motion::MotionDatabase *m_db = nullptr;
    uint32_t m_n = 0, m_count = 0, m_collapses = 0;
    std::array<Player, kCapacity> m_players{};
    std::array<std::vector<BoneTransform>, kCapacity> m_pose;
    std::array<Signals, kCapacity> m_samples{};
    std::vector<float> m_factors;
    std::vector<float> m_fullLegFactors; // m_factors with the leg bones at 1 (fullLegFade layers)
    std::vector<BoneTransform> m_out;
    Signals m_signals{};
    GraphInputs m_graph{};
    float m_steeringHalfLife = kSteeringHalfLifePrevious, m_steeringTargetTime = kSteeringTargetTimePrevious;
    int32_t m_steerMode = UTPP_STEER_MODE == kSteerCarried ? kSteerCarried : UTPP_STEER_MODE != 0 ? kSteerNewest : kSteerAll;
    int32_t m_orientationCurve = -1;
    int32_t m_warpCurve = -1; // clip warping curve that gates stride warping (see Init)
    int32_t m_thighBones[2]{-1, -1};
    bool m_strideWarp = UTPP_STRIDE_WARP != 0; // SetStrideWarp
    float m_strideScale = 1.0f;                // StrideScale
    std::vector<float> m_strideClipFactor;     // StrideClipFactor, per clip
    std::vector<uint8_t> m_strideCrouchClip;   // StrideCrouchClip, per clip
    bool m_strideYawFade = UTPP_STRIDE_YAW_FADE != 0;  // SetStrideOptions
    int32_t m_strideCrouchMode = UTPP_STRIDE_CROUCH;  // SetStrideOptions
    float m_strideCrouchYawStart = stridewarp::kYawFadeStart, m_strideCrouchYawEnd = stridewarp::kYawFadeEnd; // yaw fade of crouched loops
    bool m_strideLayerFade = UTPP_STRIDE_LAYER_FADE != 0; // SetStrideOptions
    bool m_strideKneeLimit = UTPP_STRIDE_KNEE_LIMIT != 0; // LegStrideScale
    bool m_strideLegRelease = UTPP_STRIDE_LEG_RELEASE != 0; // WarpStride, stridewarp::LimitLegRelease
    float m_strideKneeMargin = stridewarp::kKneeMargin, m_strideKneeFree = stridewarp::kKneeFreeFlexion; // LegStrideScale
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
    int32_t m_rootBone = -1; // "Root" (-1 = none: the root signals keep the whole-layer weight)
    bool m_perBoneRoot = (UTPP_BLEND_SIGNAL_MODE & 1) != 0, m_perBoneContacts = (UTPP_BLEND_SIGNAL_MODE & 2) != 0; // SetPerBoneSignals
    std::vector<int32_t> m_hipsChain; // root -> Hips bone indices
    std::array<motionFeet::Targets, kCapacity> m_targets{};
    std::array<StrideOffset, kCapacity> m_strideOffset{}; // per layer: the stride warping part of its relative correction, see MixTargets
    std::array<float, kCapacity> m_strideAmount{}; // per layer: its shortening, 1 - effective stride scale (0 = not warped)
    float m_strideMixed = 0.0f; // shortening of the output mixed on the whole-layer weights (Evaluate)
    float m_strideHeld = 0.0f;  // shortening of the output after LimitStrideRelease
    float m_strideYawRate = 0.0f, m_strideLastYaw = 0.0f; // smoothed |visual yaw rate| (rad/s) and the last visual yaw, yaw fade
    bool m_strideYawValid = false;
    std::array<math::Vec3, kCapacity> m_strideDir{}; // per layer: its stride direction (WarpStride)
    math::Vec3 m_strideHeldDir{};                    // last mixed stride direction of the output, for the held shortening
    std::array<std::array<float, 2>, kCapacity> m_ankleZ{}; // per layer: model-space ankle height of its own pose, left / right
    motionFeet::Targets m_outputTargets{};
    // ---- hand-back blend (dead-blended tail) ----
    bool m_handBack = UTPP_HANDBACK_DEAD_BLEND != 0; // SetHandBackBlend
    bool m_handBackPending = false;                  // the next pick fades in with the hand-back profile
    bool m_deadHalfLivesPending = false;             // the extrapolation half-lives wait for the first destination pose
    float m_deadTime = 0;                            // s since the hand-back pose
    std::vector<float> m_handBackFactors;            // blendprofile::kHandBack resolved to the rig
    deadblend::Extrapolator m_dead;
    std::vector<deadblend::BoneTR> m_deadPrev, m_deadCurr, m_deadDst;
    // ---- end hand-back blend ----
};
} // namespace utpp::mm::motionstack

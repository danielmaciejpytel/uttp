// UTPP ragdoll get-up: pure logic of the native get-up owner (plain C++, no Windows / RED4ext / game data; unit-tested in
// tests/RagdollGetUpTest.cpp). The glue (state under the driver lock, snapshot conversion, natives, logging) is in MotionDriver.cpp.
//
// Contents
//   - constants of the get-up (feed-time constants are tuning parameters, offline-tested)
//   - quaternion / pose helpers (local pose arrays are n * 7 floats: p.xyz, q.xyzw per database bone = PoseRecord layout)
//   - ClassifyLie  : which of the four stand clips (B, F, L, R) matches the way V lies
//   - AlignYawXY   : weighted 2D Procrustes (yaw + translation) of the clip's frame 0 onto the ragdoll joints
//   - BuildGetUpPlan: snapshot (local pose per database bone) -> clip index + world anchor + anchor yaw
//   - ComposeBlend / FloorDeficit / StepLift: the per-frame pose (world anchored, crossfaded from the snapshot, floor safety net)
//   - GetUpPlayer  : phase machine Idle -> Blend -> Play -> HandOff -> Idle on injected feed time
//   - BuildGetUpClips: validation + precompute of a parsed utpp_getup_<body>.utpptrav (the traversal grammar, version 2)
//
// Conventions: model frame = entity frame, +Z up, +Y forward. Yaw is the angle of a CCW rotation about +Z that takes +Y to the
// forward direction (forward(psi) = (-sin psi, cos psi)); world = entityPos + RotZ(entityYaw) * model. Lateral axis of the lie
// classifier = LeftUpLeg - RightUpLeg (the belly normal uses RightUpLeg - LeftUpLeg; the two are never mixed).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "../mm/LegIk.hpp"
#include "../mm/TraversalFile.hpp"
#include "RagdollEpisode.hpp"
#include "RdLastPose.hpp"

namespace utpp::ragdoll
{
// ------------------------------------------------------------------------------------------------ constants
inline constexpr float kGuBlendInS = 0.30f;           // shortest blend from the ragdoll pose (a snapshot close to the clip's frame 0)
inline constexpr float kGuBlendMaxS = 0.60f;          // longest one
inline constexpr float kGuBlendRmsK = 1.0f;           // blend length = kGuBlendInS + kGuBlendRmsK * (snapshot -> clip rms in m), clamped
inline constexpr float kGuBlendClipRate = 0.5f;       // the clip clock runs at this rate while the blend runs (the clip must not run away from it)
inline constexpr float kGuBlendOutS = 0.30f;
inline constexpr float kGuMaxStepS = 0.25f;           // one feed counts for at most this much feed time (= kProbeMaxStepS)
inline constexpr float kGuWatchdogFeedPadS = 1.0f;    // feed watchdog = clip length + this
inline constexpr double kGuWallWatchdogS = 20.0;      // wall clock since Start (generous: time dilation / slow motion stretch it), paused in menu / photo; the feed-time watchdog is the real limit
inline constexpr double kGuSnapMaxAgeS = 0.15;
inline constexpr float kGuMaxRmsM = 0.60f;
inline constexpr float kGuAnchorMaxOffM = 0.60f;
inline constexpr float kGuGroundMaxM = 1.0f;
inline constexpr float kGuLiftMaxM = 0.10f;
inline constexpr float kGuLiftMaxBlendM = 0.25f;       // the lift cap while the blend from the ragdoll pose runs (seq 9: a 0.21 m residual of the
                                                       // crossfade); the cap of kGuLiftMaxM caps only the TARGET after the blend, the lift falls to it
                                                       // at kGuLiftFallMs (no drop at the Blend -> Play switch)
inline constexpr float kGuHandRateRadS = 6.0f;         // hand step: the applied wrist correction changes at most 0.10 rad per 60 fps frame
                                                       // (<= 1.8 cm at the farthest finger joint; r13d's 0.50 rad in 5 frames)
inline constexpr float kGuLiftRateMs = 3.0f;           // whole-body residual lift, rise (the limbs are corrected first, see FloorCorrect): fast, the floor is never entered
inline constexpr float kGuLiftFallMs = 0.25f;          // the same lift, release: slow (<= 0.5 cm per frame at 60 fps, 2.5 cm in the longest 0.1 s publish gap), so a residual deficit that vanishes does not drop the body
inline constexpr float kGuLiftFallHighMs = 0.75f;      // the part of the lift above kGuLiftMaxM (only reached in the blend) falls faster: a 0.21 m blend
                                                       // residual that clears does not leave the feet hovering for ~0.8 s (1.25 cm per 60 fps frame)
inline constexpr float kGuInputStick = 0.30f;          // stick magnitude that counts as movement input
inline constexpr float kGuOffAnchorM = 0.15f;          // capsule XY distance from the anchor that ends the get-up before Interrupt
inline constexpr float kGuArmWithinM = 0.05f;          // the off-anchor check is armed once the capsule was seen this close
inline constexpr float kGuGroundGraceS = 0.5f;         // MapContext::ragdollRecover tail after the hand-off entry
inline constexpr float kRagdollRecoverMaxDropM = 0.5f; // a drop up to this far above the floor is no fall
inline constexpr float kRagdollRecoverWindowS = 1.5f;  // feed time after the ragdoll knockdown detail ends
inline constexpr float kGuInterruptHipsM = 0.80f;      // informational (baked into the file's Interrupt marker)
inline constexpr uint32_t kGuMaxBones = 256;
inline constexpr int kGuClipCount = 4;
inline const char* const kGuClipNames[kGuClipCount] = {"GetUp_B", "GetUp_F", "GetUp_L", "GetUp_R"};

enum GuPlanCode : int32_t
{
    kGuPlanUnavailable = -1,
    kGuPlanNoSnapshot = -2,
    kGuPlanBonesMissing = -3,
    kGuPlanPoseMismatch = -4,
    kGuPlanAnchorFar = -5,
    kGuPlanBusy = -6,
};
inline const char* GuPlanCodeName(int32_t aCode)
{
    switch (aCode)
    {
    case kGuPlanUnavailable: return "unavailable";
    case kGuPlanNoSnapshot: return "no-snapshot";
    case kGuPlanBonesMissing: return "bones-missing";
    case kGuPlanPoseMismatch: return "pose-mismatch";
    case kGuPlanAnchorFar: return "anchor-far";
    case kGuPlanBusy: return "busy";
    default: return aCode >= 0 ? "ok" : "?";
    }
}

enum class GuPhase : int32_t { Idle = 0, Blend = 1, Play = 2, HandOff = 3 };

// ------------------------------------------------------------------------------------------------ small math
namespace gu
{
struct Q { float x, y, z, w; };
inline Q Mul(const Q& a, const Q& b)
{
    return Q{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Q Normalize(const Q& q)
{
    const float l2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!(l2 > 1e-12f) || !std::isfinite(l2))
        return Q{0, 0, 0, 1};
    const float inv = 1.0f / std::sqrt(l2);
    return Q{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}
inline void Rotate(const Q& q, const float v[3], float o[3])
{
    const float cx = q.y * v[2] - q.z * v[1], cy = q.z * v[0] - q.x * v[2], cz = q.x * v[1] - q.y * v[0];
    const float dx = q.y * cz - q.z * cy, dy = q.z * cx - q.x * cz, dz = q.x * cy - q.y * cx;
    o[0] = v[0] + 2.0f * (q.w * cx + dx);
    o[1] = v[1] + 2.0f * (q.w * cy + dy);
    o[2] = v[2] + 2.0f * (q.w * cz + dz);
}
inline Q YawQ(float aRad) { return Q{0.0f, 0.0f, std::sin(0.5f * aRad), std::cos(0.5f * aRad)}; }
inline void RotZ(float aRad, const float v[3], float o[3])
{
    const float c = std::cos(aRad), s = std::sin(aRad);
    const float x = c * v[0] - s * v[1], y = s * v[0] + c * v[1], z = v[2];
    o[0] = x;
    o[1] = y;
    o[2] = z;
}
inline float WrapPi(float a)
{
    constexpr float kPi = 3.14159265358979f;
    a = std::fmod(a + kPi, 2.0f * kPi);
    if (a < 0.0f)
        a += 2.0f * kPi;
    return a - kPi;
}
inline float Smooth(float u)
{
    u = std::clamp(u, 0.0f, 1.0f);
    return u * u * (3.0f - 2.0f * u);
}
inline Q Get(const float* p) { return Q{p[0], p[1], p[2], p[3]}; }
inline void Put(float* p, const Q& q)
{
    p[0] = q.x;
    p[1] = q.y;
    p[2] = q.z;
    p[3] = q.w;
}
// Shortest-arc normalized lerp of unit quaternions.
inline Q Nlerp(const Q& a, Q b, float t)
{
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f)
        b = Q{-b.x, -b.y, -b.z, -b.w};
    return Normalize(Q{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
}
inline bool RootChild(const int32_t* aParents, uint32_t aBone, uint32_t aFirstBone)
{
    return aBone >= aFirstBone && aParents[aBone] < static_cast<int32_t>(aFirstBone);
}

// Model-space FK of a local pose. Bones below aFirstBone and root children (parent below aFirstBone) are placed directly (the
// published pose never writes Root / Trajectory). Parents must precede their children (BuildGetUpClips checks it).
inline void FkModel(const int32_t* aParents, uint32_t aN, uint32_t aFirstBone, const float* aLocal, float* aP, float* aQ)
{
    for (uint32_t i = 0; i < aN; ++i)
    {
        const float* l = aLocal + size_t(i) * 7;
        if (i < aFirstBone)
        {
            aP[i * 3] = aP[i * 3 + 1] = aP[i * 3 + 2] = 0.0f;
            aQ[i * 4] = aQ[i * 4 + 1] = aQ[i * 4 + 2] = 0.0f;
            aQ[i * 4 + 3] = 1.0f;
        }
        else if (aParents[i] < static_cast<int32_t>(aFirstBone))
        {
            std::memcpy(aP + size_t(i) * 3, l, 3 * sizeof(float));
            std::memcpy(aQ + size_t(i) * 4, l + 3, 4 * sizeof(float));
        }
        else
        {
            const uint32_t pi = static_cast<uint32_t>(aParents[i]);
            const Q pq = Get(aQ + size_t(pi) * 4);
            float r[3];
            Rotate(pq, l, r);
            for (int k = 0; k < 3; ++k)
                aP[i * 3 + k] = aP[pi * 3 + k] + r[k];
            Put(aQ + size_t(i) * 4, Normalize(Mul(pq, Get(l + 3))));
        }
    }
}
} // namespace gu

// ------------------------------------------------------------------------------------------------ named bones
enum GuJoint : int { kJHips, kJSpine, kJSpine3, kJHead, kJLUp, kJRUp, kJLLeg, kJRLeg, kJLFoot, kJRFoot, kJLHand, kJRHand, kJCount };
inline constexpr const char* kGuJointNames[kJCount] = {"Hips",     "Spine",     "Spine3",   "Head",     "LeftUpLeg", "RightUpLeg",
                                                       "LeftLeg",  "RightLeg",  "LeftFoot", "RightFoot", "LeftHand",  "RightHand"};
// Alignment weights (0 = not used) and surface radii (m) of the named joints (the lowest-point fallback of the ground height).
inline constexpr float kGuAlignWeight[kJCount] = {3.0f, 0.0f, 2.0f, 2.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.5f, 0.5f};
inline constexpr float kGuJointRadius[kJCount] = {0.10f, 0.08f, 0.10f, 0.09f, 0.08f, 0.08f, 0.05f, 0.05f, 0.05f, 0.05f, 0.03f, 0.03f};

// Floor safety net: the clamp set and radii (m). Fingers are not in the clamp set (they never drive the whole-body lift); the finger joints
// ((Left|Right)Hand(Thumb|Index|Middle|Ring|Pinky)[123]) are kept above the floor by the hand step of FloorCorrect with kGuFingerRadius.
inline constexpr float kGuFingerRadius = 0.01f;
inline constexpr int kGuFingerMax = 15; // per hand: 5 fingers x 3 joints
inline constexpr int kGuClampCount = 15;
inline constexpr const char* kGuClampNames[kGuClampCount] = {"Hips",     "Spine3",   "Head",          "LeftArm",       "RightArm",
                                                             "LeftForeArm", "RightForeArm", "LeftHand", "RightHand", "LeftLeg",
                                                             "RightLeg", "LeftFoot", "RightFoot", "LeftToeBase", "RightToeBase"};
inline constexpr float kGuClampRadius[kGuClampCount] = {0.10f, 0.10f, 0.09f, 0.05f, 0.05f, 0.04f, 0.04f, 0.03f,
                                                        0.03f, 0.05f, 0.05f, 0.05f, 0.05f, 0.02f, 0.02f};

struct GuBones
{
    int32_t joint[kJCount];
    int32_t clamp[kGuClampCount]; // database index, -1 = absent
    int32_t finger[2][kGuFingerMax]; // [0] left, [1] right hand: database indices of the finger joints present
    int fingerN[2] = {0, 0};
    bool jointsOk = false;
    GuBones()
    {
        for (auto& j : joint)
            j = -1;
        for (auto& c : clamp)
            c = -1;
        for (auto& h : finger)
            for (auto& f : h)
                f = -1;
    }
};

// 0 = left, 1 = right hand finger joint ((Left|Right)Hand(Thumb|Index|Middle|Ring|Pinky)[123]), -1 = other bone
inline int GuFingerSide(std::string_view aName)
{
    int side;
    if (aName.substr(0, 8) == "LeftHand")
        side = 0, aName.remove_prefix(8);
    else if (aName.substr(0, 9) == "RightHand")
        side = 1, aName.remove_prefix(9);
    else
        return -1;
    if (aName.size() < 2 || aName.back() < '1' || aName.back() > '3')
        return -1;
    aName.remove_suffix(1);
    return (aName == "Thumb" || aName == "Index" || aName == "Middle" || aName == "Ring" || aName == "Pinky") ? side : -1;
}

template <class NameFn>
inline GuBones ResolveGuBones(uint32_t aN, NameFn&& aName)
{
    GuBones b;
    for (uint32_t i = 0; i < aN; ++i)
    {
        const std::string_view nm = aName(i);
        for (int j = 0; j < kJCount; ++j)
            if (nm == kGuJointNames[j])
                b.joint[j] = static_cast<int32_t>(i);
        for (int j = 0; j < kGuClampCount; ++j)
            if (nm == kGuClampNames[j])
                b.clamp[j] = static_cast<int32_t>(i);
        const int side = GuFingerSide(nm);
        if (side >= 0 && b.fingerN[side] < kGuFingerMax)
            b.finger[side][b.fingerN[side]++] = static_cast<int32_t>(i);
    }
    b.jointsOk = true;
    for (int j = 0; j < kJCount; ++j)
        b.jointsOk = b.jointsOk && b.joint[j] >= 0;
    return b;
}

// ------------------------------------------------------------------------------------------------ lie classification
// Clip frame-0 features: (frontZ, latZ) in the order of kGuClipNames. The values the stand clips have when baked in this rig:
// B (on the back) front +1, F (on the front) front -1, L lateral -1 (left hip lower), R lateral +1.
inline constexpr float kGuCanonicalFeatures[kGuClipCount][2] = {{1.0f, 0.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}, {0.0f, 1.0f}};

// frontZ / latZ of a pose from the model positions of Hips, Spine, LeftUpLeg, RightUpLeg. aRefSign: sign of the belly normal's
// forward component in the standing pose (PelvisTracker reference). false = degenerate.
inline bool LieAxes(const float aHips[3], const float aSpine[3], const float aLUp[3], const float aRUp[3], float aRefSign, float& aFrontZ,
                    float& aLatZ)
{
    float n[3];
    if (!BellyNormal(aHips, aSpine, aLUp, aRUp, n))
        return false;
#if UTPP_GETUP_MUTATION == 1
    const float lat[3] = {aRUp[0] - aLUp[0], aRUp[1] - aLUp[1], aRUp[2] - aLUp[2]}; // mutation 1: lateral sign swapped
#else
    const float lat[3] = {aLUp[0] - aRUp[0], aLUp[1] - aRUp[1], aLUp[2] - aRUp[2]};
#endif
    const float len = std::sqrt(lat[0] * lat[0] + lat[1] * lat[1] + lat[2] * lat[2]);
    if (!(len > 1e-6f) || !std::isfinite(len))
        return false;
    aFrontZ = n[2] * aRefSign;
    aLatZ = lat[2] / len;
    return std::isfinite(aFrontZ) && std::isfinite(aLatZ);
}

// Clip index 0..3 (B, F, L, R) of the clip whose frame-0 features are closest in direction to the lie; -1 degenerate.
inline int ClassifyLie(const float aHips[3], const float aSpine[3], const float aLUp[3], const float aRUp[3], float aRefSign,
                       const float (*aFeat)[2], float& aFrontZ, float& aLatZ)
{
    if (!LieAxes(aHips, aSpine, aLUp, aRUp, aRefSign, aFrontZ, aLatZ))
        return -1;
    int best = 0;
    float bestDot = -std::numeric_limits<float>::infinity();
    for (int c = 0; c < kGuClipCount; ++c)
    {
        const float d = aFrontZ * aFeat[c][0] + aLatZ * aFeat[c][1];
        if (d > bestDot)
        {
            bestDot = d;
            best = c;
        }
    }
    return best;
}

// ------------------------------------------------------------------------------------------------ yaw alignment
struct GuAlign
{
    bool ok = false;
    float yaw = 0.0f;       // rad
    float anchor[2] = {0, 0};
    float rms = 0.0f;       // weighted XY residual (m)
    float mismatch = 0.0f;  // blend-length measure (m): max(3D weighted rms, half the worst joint) after the fit; heights compared with the mean offset removed
};

// Weighted 2D Procrustes (yaw only + translation): finds yaw and anchorXY such that r_k ~ anchor + RotZ(yaw) * c_k. aC: clip
// frame-0 joints in the clip root frame, aR: ragdoll joints in the world. Entries with weight <= 0 are ignored.
inline GuAlign AlignYawXY(const float (*aC)[3], const float (*aR)[3], const float* aW, uint32_t aK)
{
    GuAlign o;
    double sw = 0, cx = 0, cy = 0, rx = 0, ry = 0;
    for (uint32_t k = 0; k < aK; ++k)
    {
#if UTPP_GETUP_MUTATION == 2
        const double w = aW[k] > 0.0f ? 1.0 : 0.0; // mutation 2: the weights are ignored
#else
        const double w = aW[k] > 0.0f ? aW[k] : 0.0;
#endif
        sw += w;
        cx += w * aC[k][0];
        cy += w * aC[k][1];
        rx += w * aR[k][0];
        ry += w * aR[k][1];
    }
    if (!(sw > 0.0))
        return o;
    cx /= sw;
    cy /= sw;
    rx /= sw;
    ry /= sw;
    double num = 0, den = 0;
    for (uint32_t k = 0; k < aK; ++k)
    {
#if UTPP_GETUP_MUTATION == 2
        const double w = aW[k] > 0.0f ? 1.0 : 0.0;
#else
        const double w = aW[k] > 0.0f ? aW[k] : 0.0;
#endif
        const double ax = aC[k][0] - cx, ay = aC[k][1] - cy, bx = aR[k][0] - rx, by = aR[k][1] - ry;
        num += w * (ax * by - ay * bx);
        den += w * (ax * bx + ay * by);
    }
    if (!(std::fabs(num) + std::fabs(den) > 1e-9))
        return o;
    const double yaw = std::atan2(num, den);
    const double s = std::sin(yaw), c = std::cos(yaw);
    const double axy = rx - (c * cx - s * cy), ayy = ry - (s * cx + c * cy);
    double res = 0, res3 = 0, worst3 = 0, dzMean = 0;
    for (uint32_t k = 0; k < aK; ++k)
    {
        const double w = aW[k] > 0.0f ? aW[k] : 0.0;
        dzMean += w * (aR[k][2] - aC[k][2]);
    }
    dzMean /= sw;
    for (uint32_t k = 0; k < aK; ++k)
    {
#if UTPP_GETUP_MUTATION == 2
        const double w = aW[k] > 0.0f ? 1.0 : 0.0;
#else
        const double w = aW[k] > 0.0f ? aW[k] : 0.0;
#endif
        const double px = axy + c * aC[k][0] - s * aC[k][1], py = ayy + s * aC[k][0] + c * aC[k][1];
        const double d2 = (aR[k][0] - px) * (aR[k][0] - px) + (aR[k][1] - py) * (aR[k][1] - py);
        res += w * d2;
        const double dz = (aR[k][2] - aC[k][2]) - dzMean;
        res3 += w * (d2 + dz * dz);
        if (w > 0.0)
            worst3 = std::fmax(worst3, std::sqrt(d2 + dz * dz));
    }
    o.ok = std::isfinite(yaw) && std::isfinite(axy) && std::isfinite(ayy);
    o.yaw = static_cast<float>(yaw);
    o.anchor[0] = static_cast<float>(axy);
    o.anchor[1] = static_cast<float>(ayy);
    o.rms = static_cast<float>(std::sqrt(res / sw));
    o.mismatch = static_cast<float>(std::fmax(std::sqrt(res3 / sw), 0.5 * worst3));
    return o;
}

// ------------------------------------------------------------------------------------------------ clips and plan
struct GuClipInfo
{
    bool ok = false;
    uint32_t frames = 0;
    float fps = 30.0f;
    float lengthS = 0.0f, handOffS = 0.0f, interruptS = 0.0f;
    float feat[2] = {0, 0};   // frame-0 (frontZ, latZ)
    float standSign = 1.0f;   // belly-normal forward sign of the standing end pose
    float hipsLastZ = 0.0f;   // Hips height at the hand-off frame
    std::vector<float> frame0Model; // n * 3
    std::vector<float> frame0Local; // n * 7 (database bone order)
};

struct GuPlan
{
    int clip = -1;
    float anchor[3] = {0, 0, 0}; // world: x, y, ground z
    float yaw = 0.0f;            // rad, anchor yaw (the entity yaw the hand-off needs)
    float fwd[2] = {0, 1};       // world forward of the anchor
    float rms = 0.0f;
    bool ok = false;
    const char* why = "";
    float handOffS = 0.0f, interruptS = 0.0f, lengthS = 0.0f;
    float blendInS = 0.0f; // adaptive blend length (0 = kGuBlendInS)
};

struct GuPlanIn
{
    uint32_t n = 0, firstBone = 2;
    const int32_t* parents = nullptr;
    const GuBones* bones = nullptr;
    const GuClipInfo* clips = nullptr; // kGuClipCount
    const float* snapLocal = nullptr;  // n * 7
    const uint8_t* have = nullptr;     // n: 1 = the bone has a ragdoll record
    float entityPos[3] = {0, 0, 0};
    float entityYaw = 0.0f; // rad
    bool haveRefSign = false;
    float refSign = 1.0f;
    float airGroundZ = std::numeric_limits<float>::quiet_NaN(); // capsule z - distance to ground (NaN = none)
};

struct GuPlanOut
{
    int32_t code = kGuPlanUnavailable;
    GuPlan plan;
    std::vector<float> snapLocal;     // n * 7, completed
    std::vector<float> snapRootWorld; // n * 7, world transform of the root children (others unused)
    uint32_t missing = 0;
    float frontZ = 0.0f, latZ = 0.0f;
    char groundSrc = '-'; // 'a' air feed, 'j' lowest joint
};

inline void GuFillMissing(const GuPlanIn& aIn, const GuClipInfo& aClip, GuPlanOut& aOut)
{
    for (uint32_t i = 0; i < aIn.n; ++i)
        if (!aIn.have[i])
            std::memcpy(&aOut.snapLocal[size_t(i) * 7], &aClip.frame0Local[size_t(i) * 7], 7 * sizeof(float));
}

inline GuPlanOut BuildGetUpPlan(const GuPlanIn& aIn)
{
    GuPlanOut o;
    const uint32_t n = aIn.n;
    if (n == 0 || n > kGuMaxBones || !aIn.parents || !aIn.bones || !aIn.bones->jointsOk || !aIn.clips || !aIn.snapLocal || !aIn.have)
    {
        o.plan.why = "unavailable";
        return o;
    }
    for (int c = 0; c < kGuClipCount; ++c)
        if (!aIn.clips[c].ok || aIn.clips[c].frame0Local.size() != size_t(n) * 7 || aIn.clips[c].frame0Model.size() != size_t(n) * 3)
        {
            o.plan.why = "unavailable";
            return o;
        }
    o.snapLocal.assign(aIn.snapLocal, aIn.snapLocal + size_t(n) * 7);
    o.missing = 0;
    for (uint32_t i = aIn.firstBone; i < n; ++i)
        o.missing += aIn.have[i] ? 0u : 1u;
    // bones of the classification set must have a record
    const int need[] = {kJHips, kJSpine, kJSpine3, kJHead, kJLUp, kJRUp};
    for (int j : need)
        if (!aIn.have[aIn.bones->joint[j]])
        {
            o.code = kGuPlanBonesMissing;
            o.plan.why = "bones-missing";
            return o;
        }
    float feat[kGuClipCount][2];
    for (int c = 0; c < kGuClipCount; ++c)
    {
        feat[c][0] = aIn.clips[c].feat[0];
        feat[c][1] = aIn.clips[c].feat[1];
    }
    float P[kGuMaxBones * 3], Q[kGuMaxBones * 4];
    GuFillMissing(aIn, aIn.clips[0], o); // provisional filler for the FK chain; replaced below by the chosen clip's frame 0
    gu::FkModel(aIn.parents, n, aIn.firstBone, o.snapLocal.data(), P, Q);
    const auto jp = [&](int j) { return P + size_t(aIn.bones->joint[j]) * 3; };
    const float sign = aIn.haveRefSign ? aIn.refSign : aIn.clips[0].standSign;
    const int clip = ClassifyLie(jp(kJHips), jp(kJSpine), jp(kJLUp), jp(kJRUp), sign, feat, o.frontZ, o.latZ);
    if (clip < 0)
    {
        o.code = kGuPlanBonesMissing;
        o.plan.why = "degenerate";
        return o;
    }
    if (clip != 0 && o.missing > 0)
    {
        GuFillMissing(aIn, aIn.clips[clip], o);
        gu::FkModel(aIn.parents, n, aIn.firstBone, o.snapLocal.data(), P, Q);
    }
    const GuClipInfo& ci = aIn.clips[clip];
    // alignment joints
    float c[kJCount][3], r[kJCount][3], w[kJCount];
    float minZ = std::numeric_limits<float>::infinity();
    for (int j = 0; j < kJCount; ++j)
    {
        const size_t bi = static_cast<size_t>(aIn.bones->joint[j]);
        float world[3];
        gu::RotZ(aIn.entityYaw, P + bi * 3, world);
        for (int k = 0; k < 3; ++k)
        {
            r[j][k] = aIn.entityPos[k] + world[k];
            c[j][k] = ci.frame0Model[bi * 3 + k];
        }
        w[j] = kGuAlignWeight[j];
        minZ = std::fmin(minZ, r[j][2] - kGuJointRadius[j]);
    }
    const GuAlign al = AlignYawXY(c, r, w, kJCount);
    if (!al.ok || !(al.rms <= kGuMaxRmsM))
    {
        o.code = kGuPlanPoseMismatch;
        o.plan.why = "pose-mismatch";
        o.plan.rms = al.rms;
        return o;
    }
    // The clip's Root is not where the body is: frame-0 Hips sits ~0.4 m from it (the baked Trajectory is folded into Hips). The distance
    // the teleport has to cover is that of the PREDICTED frame-0 Hips (anchor + RotZ(yaw) * hips0 XY) from the entity, which follows
    // the ragdoll body; comparing the raw anchor would refuse a normal Back / Front get-up.
    const size_t hb = static_cast<size_t>(aIn.bones->joint[kJHips]);
    const float h0[3] = {ci.frame0Model[hb * 3], ci.frame0Model[hb * 3 + 1], 0.0f};
    float h0w[3];
    gu::RotZ(al.yaw, h0, h0w);
    const float offX = al.anchor[0] + h0w[0] - aIn.entityPos[0], offY = al.anchor[1] + h0w[1] - aIn.entityPos[1];
    if (!(std::sqrt(offX * offX + offY * offY) <= kGuAnchorMaxOffM))
    {
        o.code = kGuPlanAnchorFar;
        o.plan.why = "anchor-far";
        o.plan.rms = al.rms;
        return o;
    }
    GuPlan& p = o.plan;
    p.clip = clip;
    p.anchor[0] = al.anchor[0];
    p.anchor[1] = al.anchor[1];
    if (std::isfinite(aIn.airGroundZ))
    {
        p.anchor[2] = aIn.airGroundZ;
        o.groundSrc = 'a';
    }
    else
    {
        p.anchor[2] = minZ;
        o.groundSrc = 'j';
    }
    p.yaw = al.yaw;
    p.fwd[0] = -std::sin(al.yaw);
    p.fwd[1] = std::cos(al.yaw);
    p.rms = al.rms;
    p.handOffS = ci.handOffS;
    p.interruptS = ci.interruptS;
    p.lengthS = ci.lengthS;
    // a snapshot far from the clip's frame 0 (large rms: the body lies differently) needs a longer blend, else the limbs whip across
    // (rms and the worst joint: a hand or the head can be far off while the rms stays moderate)
    p.blendInS = std::clamp(kGuBlendInS + kGuBlendRmsK * al.mismatch, kGuBlendInS, kGuBlendMaxS);
    p.ok = true;
    p.why = "ok";
    // root children of the snapshot in world space (re-expressed in the live entity frame every frame)
    o.snapRootWorld.assign(size_t(n) * 7, 0.0f);
    const gu::Q qy = gu::YawQ(aIn.entityYaw);
    for (uint32_t i = aIn.firstBone; i < n; ++i)
        if (gu::RootChild(aIn.parents, i, aIn.firstBone))
        {
            float wp[3];
            gu::RotZ(aIn.entityYaw, &o.snapLocal[size_t(i) * 7], wp);
            float* d = &o.snapRootWorld[size_t(i) * 7];
            for (int k = 0; k < 3; ++k)
                d[k] = aIn.entityPos[k] + wp[k];
            gu::Put(d + 3, gu::Normalize(gu::Mul(qy, gu::Get(&o.snapLocal[size_t(i) * 7 + 3]))));
        }
    o.code = clip;
    return o;
}

// ------------------------------------------------------------------------------------------------ per-frame pose
struct GuShape
{
    uint32_t n = 0, firstBone = 2;
    const int32_t* parents = nullptr;
};
struct GuFrame // the live entity transform (capsule position, yaw rad)
{
    float cap[3] = {0, 0, 0};
    float yaw = 0.0f;
};
struct GuAnchor
{
    float pos[3] = {0, 0, 0};
    float yaw = 0.0f;
};

// The entity transform the pose is anchored to: every frame the live one (a late or failed teleport, or the drop of the entity at
// the end, never moves the body); the plan-time one only when no live sample exists.
inline GuFrame SelectFrame(const GuFrame& aPlan, const GuFrame& aLive, bool aHaveLive)
{
#if UTPP_GETUP_MUTATION == 4
    (void)aLive;
    (void)aHaveLive;
    return aPlan; // mutation 4: the plan-time capsule
#else
    return aHaveLive ? aLive : aPlan;
#endif
}

// Entity frame of the release. The script teleports the entity to (anchor xy, ground z, plan yaw) in the same call that begins the
// run, so from the first frame after it the engine shows the entity in that TARGET frame while the feed samples can still be the
// ones of the PRE frame (the teleport reaches the next feed one frame later). A pose composed against such a stale sample is
// re-expressed wrongly for that frame. Resolve() returns the target frame until a feed sample matches it; a sample that is still
// the pre frame (stale) is not trusted; a sample somewhere else (teleport failed or went elsewhere, a push) or kGuTargetMaxFeeds
// feeds without a match hand the anchoring back to the live sample (the body then stays in the world as before).
inline constexpr float kGuTargetPosTolM = 0.02f;
inline constexpr float kGuTargetZTolM = 0.05f; // the capsule rests a little differently from the ray hit
inline constexpr float kGuTargetYawTolRad = 0.0174533f;
inline constexpr int kGuTargetMaxFeeds = 3;
struct GuTarget
{
    GuFrame target, pre;
    bool pending = false;
    int feeds = 0;
    uint32_t targetFrames = 0; // frames composed against the target (diagnostics)
    static bool Near(const GuFrame& a, const GuFrame& b)
    {
        return std::hypot(a.cap[0] - b.cap[0], a.cap[1] - b.cap[1]) <= kGuTargetPosTolM && std::fabs(a.cap[2] - b.cap[2]) <= kGuTargetZTolM &&
               std::fabs(gu::WrapPi(a.yaw - b.yaw)) <= kGuTargetYawTolRad;
    }
    void Arm(const GuFrame& aTarget, const GuFrame& aPre)
    {
        target = aTarget;
        pre = aPre;
        pending = true;
        feeds = 0;
        targetFrames = 0;
    }
    void Disarm() { pending = false; }
    void OnFeed()
    {
        if (pending)
            ++feeds;
    }
    GuFrame Resolve(const GuFrame& aPlan, const GuFrame& aLive, bool aHaveLive)
    {
#if UTPP_GETUP_MUTATION == 9
        pending = false; // mutation 9: the live (possibly stale) sample is always trusted
#endif
        if (!pending)
            return SelectFrame(aPlan, aLive, aHaveLive);
        if (aHaveLive)
        {
            if (Near(aLive, target))
            {
                pending = false; // the feed has caught up with the teleport
                return aLive;
            }
            if (!Near(aLive, pre) || feeds >= kGuTargetMaxFeeds)
            {
                pending = false; // the entity went elsewhere / never arrived: world anchoring through the live sample
                return aLive;
            }
        }
        ++targetFrames;
        return target;
    }
};

// Crossfade from the snapshot (weight 0) to the clip (weight 1), both anchored to the world. aClipLocal: the clip sampled at the
// clip time (n * 7 local); aSnapLocal: the snapshot (n * 7 local); aSnapRootWorld: world transforms of the snapshot's root children
// (BuildGetUpPlan). Root children are warped into the live entity frame (the traversal warp of MotionDriver.cpp); every other bone
// takes its local values. aW is the already-smoothed blend weight.
inline void ComposeBlend(const GuShape& aS, const float* aClipLocal, const float* aSnapLocal, const float* aSnapRootWorld,
                         const GuAnchor& aAnchor, const GuFrame& aLive, float aW, float* aOut)
{
    const float dYaw = gu::WrapPi(aAnchor.yaw - aLive.yaw);
    const gu::Q qd = gu::YawQ(dYaw), qLive = gu::YawQ(-aLive.yaw);
    const float off[3] = {aAnchor.pos[0] - aLive.cap[0], aAnchor.pos[1] - aLive.cap[1], aAnchor.pos[2] - aLive.cap[2]};
    float anchorLoc[3];
    gu::RotZ(-aLive.yaw, off, anchorLoc);
    const float w = std::clamp(aW, 0.0f, 1.0f);
    for (uint32_t i = 0; i < aS.n; ++i)
    {
        float cp[3], sp[3];
        gu::Q cq, sq;
        const float* c = aClipLocal + size_t(i) * 7;
        const float* s = aSnapLocal + size_t(i) * 7;
        if (gu::RootChild(aS.parents, i, aS.firstBone))
        {
            float r[3];
            gu::Rotate(qd, c, r);
            for (int k = 0; k < 3; ++k)
                cp[k] = r[k] + anchorLoc[k];
            cq = gu::Normalize(gu::Mul(qd, gu::Get(c + 3)));
            const float* sw = aSnapRootWorld + size_t(i) * 7;
            const float d[3] = {sw[0] - aLive.cap[0], sw[1] - aLive.cap[1], sw[2] - aLive.cap[2]};
            gu::RotZ(-aLive.yaw, d, sp);
            sq = gu::Normalize(gu::Mul(qLive, gu::Get(sw + 3)));
        }
        else
        {
            std::memcpy(cp, c, sizeof cp);
            std::memcpy(sp, s, sizeof sp);
            cq = gu::Get(c + 3);
            sq = gu::Get(s + 3);
        }
        float* o = aOut + size_t(i) * 7;
        for (int k = 0; k < 3; ++k)
            o[k] = sp[k] + (cp[k] - sp[k]) * w;
        gu::Put(o + 3, gu::Nlerp(sq, cq, w));
    }
}

// Deepest penetration (m, >= 0) of the clamp set below the ground plane zGround (model space: anchor z - capsule z). aWorst (optional):
// the clamp index (kGuClampNames) of the deepest joint, -1 when nothing is below.
inline float FloorDeficit(const GuShape& aS, const float* aPose, const GuBones& aBones, float aZGround, int* aWorst = nullptr)
{
    if (aWorst != nullptr)
        *aWorst = -1;
#if UTPP_GETUP_MUTATION == 3
    (void)aS;
    (void)aPose;
    (void)aBones;
    (void)aZGround;
    return 0.0f; // mutation 3: no floor safety net
#else
    if (aS.n == 0 || aS.n > kGuMaxBones)
        return 0.0f;
    float P[kGuMaxBones * 3], Q[kGuMaxBones * 4];
    gu::FkModel(aS.parents, aS.n, aS.firstBone, aPose, P, Q);
    float deficit = 0.0f;
    for (int j = 0; j < kGuClampCount; ++j)
    {
        const int32_t b = aBones.clamp[j];
        if (b < 0 || static_cast<uint32_t>(b) >= aS.n)
            continue;
        const float d = aZGround + kGuClampRadius[j] - P[size_t(b) * 3 + 2];
        if (d > deficit)
        {
            deficit = d;
            if (aWorst != nullptr)
                *aWorst = j;
        }
    }
    return std::isfinite(deficit) ? deficit : 0.0f;
#endif
}

// Deepest penetration (m, >= 0) of the finger joints (radius kGuFingerRadius) below the ground plane. Only gates FloorCorrect: a finger
// deficit never feeds the whole-body lift (the hand step of FloorCorrect owns it).
inline float FingerDeficit(const GuShape& aS, const float* aPose, const GuBones& aBones, float aZGround)
{
#if UTPP_GETUP_MUTATION == 3 || UTPP_GETUP_MUTATION == 12
    (void)aS;
    (void)aPose;
    (void)aBones;
    (void)aZGround;
    return 0.0f; // mutation 3: no floor safety net; mutation 12: fingers are not checked
#else
    if (aS.n == 0 || aS.n > kGuMaxBones || aBones.fingerN[0] + aBones.fingerN[1] == 0)
        return 0.0f;
    float P[kGuMaxBones * 3], Q[kGuMaxBones * 4];
    gu::FkModel(aS.parents, aS.n, aS.firstBone, aPose, P, Q);
    float deficit = 0.0f;
    for (int h = 0; h < 2; ++h)
        for (int k = 0; k < aBones.fingerN[h]; ++k)
        {
            const int32_t b = aBones.finger[h][k];
            if (b >= 0 && static_cast<uint32_t>(b) < aS.n)
                deficit = std::fmax(deficit, aZGround + kGuFingerRadius - P[size_t(b) * 3 + 2]);
        }
    return std::isfinite(deficit) ? deficit : 0.0f;
#endif
}

// Whole-body lift state: follows min(deficit, cap) (kGuLiftMaxM; kGuLiftMaxBlendM while the blend runs), rising at kGuLiftRateMs and
// falling at kGuLiftFallMs (asymmetric: the rise keeps
// the floor, a slow release hides the end of the deficit; the lift is only ever above the deficit while it falls, so the floor stays kept).
struct GuLift
{
    float lift = 0.0f;
    uint32_t frames = 0;    // frames whose RESIDUAL deficit (after the limb correction) is over 2 cm
    float maxDeficit = 0.0f; // largest residual deficit
    uint32_t limbFrames = 0; // frames in which the limb correction acted
    float maxRaw = 0.0f;     // largest deficit before the correction
    int maxJoint = -1, maxRawJoint = -1; // clamp index (kGuClampNames) of maxDeficit / maxRaw (diagnostics, set by the caller)
    float maxJointW = 0.0f, maxRawW = 0.0f; // blend weight at which they occurred
    void Reset() { *this = GuLift{}; }
    // aCap: the target cap (kGuLiftMaxM, kGuLiftMaxBlendM while the blend runs); the lift itself never exceeds kGuLiftMaxBlendM
    float Step(float aDeficit, float aDt, float aCap = kGuLiftMaxM)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, kGuMaxStepS) : 0.0f;
#if UTPP_GETUP_MUTATION == 13
        aCap = kGuLiftMaxM; // mutation 13: the blend has the plain 0.10 m cap
#endif
        const float cap = std::clamp(std::isfinite(aCap) ? aCap : kGuLiftMaxM, 0.0f, kGuLiftMaxBlendM);
        const float target = std::clamp(std::isfinite(aDeficit) ? aDeficit : 0.0f, 0.0f, cap);
#if UTPP_GETUP_MUTATION == 10
        lift += std::clamp(target - lift, -kGuLiftRateMs * dt, kGuLiftRateMs * dt); // mutation 10: the release is as fast as the rise
#else
        // above kGuLiftMaxM the lift falls at kGuLiftFallHighMs down to kGuLiftMaxM (not below it in that step), under it at kGuLiftFallMs
        const float lo = lift > kGuLiftMaxM ? std::fmax(kGuLiftMaxM, lift - kGuLiftFallHighMs * dt) : lift - kGuLiftFallMs * dt;
        lift = std::clamp(target, std::fmin(lo, lift), lift + kGuLiftRateMs * dt);
#endif
        lift = std::clamp(lift, 0.0f, kGuLiftMaxBlendM);
        if (aDeficit > 0.02f)
            ++frames;
        maxDeficit = std::fmax(maxDeficit, aDeficit);
        return lift;
    }
};

// Rate limit of the hand step (FloorCorrect): the applied wrist correction of each hand, a rotation in the hand's LOCAL frame (local = A * base
// local; independent of the entity frame, so a teleport / a frame switch does not swing it). Each frame it turns toward the exact correction by at
// most kGuHandRateRadS * dt, toward identity when no finger is low. Reset at Begin.
struct GuHandRate
{
    gu::Q applied[2] = {gu::Q{0, 0, 0, 1}, gu::Q{0, 0, 0, 1}};
    void Reset() { *this = GuHandRate{}; }
    bool Active() const noexcept { return std::fabs(applied[0].w) < 0.99999f || std::fabs(applied[1].w) < 0.99999f; }
};

// a rotated toward b by at most aMaxAng (rad), shortest arc
inline gu::Q GuRotateToward(const gu::Q& a, const gu::Q& b, float aMaxAng)
{
    gu::Q d = gu::Mul(b, gu::Q{-a.x, -a.y, -a.z, a.w});
    if (d.w < 0.0f)
        d = gu::Q{-d.x, -d.y, -d.z, -d.w};
    const float ang = 2.0f * std::acos(std::fmin(1.0f, d.w));
    if (!(ang > aMaxAng) || !(aMaxAng >= 0.0f))
        return gu::Normalize(ang > aMaxAng ? a : b);
    const float sn = std::sqrt(std::fmax(0.0f, d.x * d.x + d.y * d.y + d.z * d.z));
    if (!(sn > 1e-7f))
        return gu::Normalize(b);
    const float sh = std::sin(0.5f * aMaxAng) / sn;
    return gu::Normalize(gu::Mul(gu::Q{d.x * sh, d.y * sh, d.z * sh, std::cos(0.5f * aMaxAng)}, a));
}

// Per-limb floor correction (runs before the whole-body lift): arms (Arm -> ForeArm -> Hand) and legs (UpLeg -> Leg -> Foot -> ToeBase). A clamp-set
// joint of the chain below ground + radius is raised by rotating the parent bone about its own position (a closed form: the joint swings on a circle
// around its parent, z(theta) = |r| sin(theta + phi)); elbow / knee first, then hand / foot, then the toes. The angle is capped (the rest stays
// as a residual for the whole-body lift) and everything is a continuous function of the pose: no pop when a limb starts to dip. Local rotations of
// the rotated bones change, children keep their local transforms, so world anchoring is untouched. Returns the number of corrections made.
// Hand step (r13d M1): the ragdoll's hand collider can rest on the floor with the fingers bent through it (the wrist 3 cm up, a pinky joint 5.7 cm
// under the floor). The lowest finger joint below ground + kGuFingerRadius is raised by rotating the Hand about the wrist (the palm flattens on
// the floor); exact (no ease-in, the deficit can be there from the first frame of the release), up to three passes for the next lowest finger,
// capped at kFingerMaxAng / kFingerSwingM. Only the hand and its children move; the elbow, the wrist and the rest of the body do not.
// aRate (optional, the live path): the hand step is applied through GuHandRate (at most kGuHandRateRadS * aDt per frame, no snap at the
// release); nullptr = the exact correction at once (pre-check, offline probes).
inline int FloorCorrect(const GuShape& aS, float* aPose, const GuBones& aBones, float aZGround, GuHandRate* aRate = nullptr, float aDt = 0.0f)
{
#if UTPP_GETUP_MUTATION == 3
    (void)aS; (void)aPose; (void)aBones; (void)aZGround; (void)aRate; (void)aDt;
    return 0; // mutation 3: no floor safety net
#else
    if (aS.n == 0 || aS.n > kGuMaxBones)
        return 0;
    // chain table by clamp index: arms 3 5 7, legs UpLeg(joint) 9 11 13
    const int32_t chains[4][4] = {{aBones.clamp[3], aBones.clamp[5], aBones.clamp[7], -1},
                                  {aBones.clamp[4], aBones.clamp[6], aBones.clamp[8], -1},
                                  {aBones.joint[kJLUp], aBones.clamp[9], aBones.clamp[11], aBones.clamp[13]},
                                  {aBones.joint[kJRUp], aBones.clamp[10], aBones.clamp[12], aBones.clamp[14]}};
    const int radiusIdx[4][4] = {{-1, 5, 7, -1}, {-1, 6, 8, -1}, {-1, 9, 11, 13}, {-1, 10, 12, 14}};
    constexpr float kEps = 0.002f, kPi2 = 1.57079632679f, kMaxSwingM = 0.04f;
    const float maxAng[3] = {1.2f, 1.4f, 0.7f}; // knee / elbow swing, hand / foot swing, toes
    float P[kGuMaxBones * 3], Q[kGuMaxBones * 4];
    int acted = 0;
    const auto fk = [&]() { gu::FkModel(aS.parents, aS.n, aS.firstBone, aPose, P, Q); };
    // raise joint aTip by aNeed by rotating bone aRot about its own position
    const auto raise = [&](int32_t aRot, int32_t aTip, float aNeed, float aMax, bool aEase = true, float aSwingM = 0.0f) { // aSwingM 0 = kMaxSwingM
        const float* pr = P + size_t(aRot) * 3;
        const float* pt = P + size_t(aTip) * 3;
        const float r[3] = {pt[0] - pr[0], pt[1] - pr[1], pt[2] - pr[2]};
        const float rl = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        if (!(rl > 1e-4f))
            return false;
        const float rh = std::sqrt(r[0] * r[0] + r[1] * r[1]);
        const float phi = std::atan2(r[2], rh);
        const float sv = (r[2] + aNeed) / rl;
        float ang = sv >= 1.0f ? kPi2 - phi : std::asin(std::fmax(sv, -1.0f)) - phi;
        if (!(ang > 1e-5f))
            return false;
        ang = std::fmin(ang, aMax) * (aEase ? gu::Smooth(aNeed / 0.03f) : 1.0f);
        // a limb that hangs straight down needs a big swing for a few millimetres (sqrt behaviour): cap the horizontal travel of the tip, the rest
        // of the deficit stays for the whole-body lift
        const float hd = std::fabs(rh * (std::cos(ang) - 1.0f) - r[2] * std::sin(ang));
        const float swing = aSwingM > 0.0f ? aSwingM : kMaxSwingM;
        if (hd > swing)
            ang *= swing / hd;
        // (the angle above also eases in over the first 3 cm of deficit: no sudden swing when a limb starts to dip)
        const float axis[3] = {rh > 1e-4f ? r[1] / rh : 1.0f, rh > 1e-4f ? -r[0] / rh : 0.0f, 0.0f};
        const float sh = std::sin(0.5f * ang);
        const gu::Q dq{axis[0] * sh, axis[1] * sh, axis[2] * sh, std::cos(0.5f * ang)};
        const gu::Q nm = gu::Normalize(gu::Mul(dq, gu::Get(Q + size_t(aRot) * 4)));
        const int32_t par = aS.parents[aRot];
        const gu::Q qp = par >= static_cast<int32_t>(aS.firstBone) ? gu::Get(Q + size_t(par) * 4) : gu::Q{0, 0, 0, 1};
        gu::Put(aPose + size_t(aRot) * 7 + 3, gu::Normalize(gu::Mul(gu::Q{-qp.x, -qp.y, -qp.z, qp.w}, nm)));
        return true;
    };
    for (int c = 0; c < 4; ++c)
    {
        const int32_t a = chains[c][0], b = chains[c][1], e = chains[c][2], t = chains[c][3];
        if (a < 0 || b < 0 || e < 0 || static_cast<uint32_t>(a) >= aS.n || static_cast<uint32_t>(b) >= aS.n || static_cast<uint32_t>(e) >= aS.n)
            continue;
        fk();
        float d = aZGround + kGuClampRadius[radiusIdx[c][1]] - P[size_t(b) * 3 + 2];
        if (d > kEps && raise(a, b, d, maxAng[0]))
        {
            ++acted;
            fk();
        }
        d = aZGround + kGuClampRadius[radiusIdx[c][2]] - P[size_t(e) * 3 + 2];
        if (d > kEps && raise(b, e, d, maxAng[1]))
        {
            ++acted;
            fk();
        }
        if (t >= 0 && static_cast<uint32_t>(t) < aS.n)
        {
            d = aZGround + kGuClampRadius[radiusIdx[c][3]] - P[size_t(t) * 3 + 2];
            if (d > kEps && raise(e, t, d, maxAng[2]))
                ++acted;
        }
#if UTPP_GETUP_MUTATION != 12 // mutation 12: no hand step (fingers stay under the floor)
        if (c < 2 && aBones.fingerN[c] > 0)
        {
            // the hand step: lowest finger joint of this hand, rotate the Hand about the wrist; the cap is the total over the passes
            constexpr float kFingerMaxAng = 1.2f, kFingerSwingM = 0.06f; // wrist flex (rad), horizontal travel of the finger joint (m)
            const gu::Q baseLocal = gu::Get(aPose + size_t(e) * 7 + 3);
            bool turned = false;
            float used = 0.0f;
            for (int pass = 0; pass < 3 && used < kFingerMaxAng - 1e-4f; ++pass)
            {
                int32_t low = -1;
                float need = kEps;
                for (int k = 0; k < aBones.fingerN[c]; ++k)
                {
                    const int32_t f = aBones.finger[c][k];
                    if (f < 0 || static_cast<uint32_t>(f) >= aS.n)
                        continue;
                    const float df = aZGround + kGuFingerRadius - P[size_t(f) * 3 + 2];
                    if (df > need)
                        need = df, low = f;
                }
                if (low < 0)
                    break;
                const gu::Q before = gu::Get(Q + size_t(e) * 4);
                if (!raise(e, low, need, kFingerMaxAng - used, false, kFingerSwingM))
                    break;
                ++acted;
                turned = true;
                fk();
                const gu::Q after = gu::Get(Q + size_t(e) * 4);
                const float dot = std::fabs(before.x * after.x + before.y * after.y + before.z * after.z + before.w * after.w);
                used += 2.0f * std::acos(std::fmin(1.0f, dot));
            }
            if (aRate != nullptr && (turned || aRate->Active()))
            {
                // exact correction in the hand's local frame -> rate-limited applied correction (also back toward identity)
                const gu::Q exact = gu::Get(aPose + size_t(e) * 7 + 3);
                const gu::Q target = gu::Normalize(gu::Mul(exact, gu::Q{-baseLocal.x, -baseLocal.y, -baseLocal.z, baseLocal.w}));
                const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, kGuMaxStepS) : 0.0f;
                aRate->applied[c] = GuRotateToward(aRate->applied[c], target, kGuHandRateRadS * dt);
                gu::Put(aPose + size_t(e) * 7 + 3, gu::Normalize(gu::Mul(aRate->applied[c], baseLocal)));
                fk();
            }
        }
#endif
    }
    return acted;
#endif
}

inline void AddLift(const GuShape& aS, float aLift, float* aPose)
{
    for (uint32_t i = aS.firstBone; i < aS.n; ++i)
        if (gu::RootChild(aS.parents, i, aS.firstBone))
            aPose[size_t(i) * 7 + 2] += aLift;
}

// Hand-off foot step (r13c M3). The hand-off blends the held get-up pose into the motion-matching pose; the two stances differ (r13c: toes
// 24 cm apart at the clip end, 43 cm in the idle), so a plain blend slides both planted feet ~9 cm over 0.3 s. A planted foot whose stance
// differs by more than 2 cm instead makes one short lifted step: the blend carries the toe horizontally as before, the ankle is raised by
// min(kGuStepLiftM, kGuStepFrac * d) * sin(pi * s) (d = held toe -> MM toe distance, s = the blend weight) with the shared two-bone leg IK
// (mm/LegIk.hpp: the foot keeps its blended model rotation, the knee its bend plane). Only raises, so the floor is never entered; the
// hips are not touched. aHeld / aMm: the held and the MM local pose (n * 7), aBlended: the blended pose (in / out: thigh, shin and foot
// rotations of a stepping leg). Model space = entity frame (the capsule stands still at the hand-off). Returns the number of legs stepped.
inline constexpr float kGuStepLiftM = 0.04f;     // peak ankle lift of the step
inline constexpr float kGuStepFrac = 0.45f;      // ... at most this fraction of the stance mismatch
inline constexpr float kGuStepMinM = 0.02f;      // no step below this mismatch (the plain blend moves a toe less than that), full from kGuStepFullM
inline constexpr float kGuStepFullM = 0.05f;
inline constexpr float kGuStepPlantedM = 0.06f;  // held toe height above the floor up to which the foot counts as planted (fades out over the next 6 cm)

inline int HandOffFootStep(const GuShape& aS, const GuBones& aBones, const float* aHeld, const float* aMm, float aBlendS, float aZGround,
                           float* aBlended)
{
#if UTPP_GETUP_MUTATION == 11
    (void)aS; (void)aBones; (void)aHeld; (void)aMm; (void)aBlendS; (void)aZGround; (void)aBlended;
    return 0; // mutation 11: a plain blend, the planted feet slide
#else
    if (aS.n == 0 || aS.n > kGuMaxBones || !std::isfinite(aBlendS) || !std::isfinite(aZGround))
        return 0;
    constexpr float kPi = 3.14159265358979f;
    const float prog = std::sin(kPi * std::clamp(aBlendS, 0.0f, 1.0f));
    if (!(prog > 1e-3f))
        return 0;
    float Ph[kGuMaxBones * 3], Qh[kGuMaxBones * 4], Pm[kGuMaxBones * 3], Qm[kGuMaxBones * 4], Pb[kGuMaxBones * 3], Qb[kGuMaxBones * 4];
    gu::FkModel(aS.parents, aS.n, aS.firstBone, aHeld, Ph, Qh);
    gu::FkModel(aS.parents, aS.n, aS.firstBone, aMm, Pm, Qm);
    gu::FkModel(aS.parents, aS.n, aS.firstBone, aBlended, Pb, Qb);
    const int32_t chains[2][4] = {{aBones.joint[kJLUp], aBones.clamp[9], aBones.clamp[11], aBones.clamp[13]},
                                  {aBones.joint[kJRUp], aBones.clamp[10], aBones.clamp[12], aBones.clamp[14]}};
    const auto bt = [&](const float* aP, const float* aQ, int32_t i) {
        return utpp::mm::BoneTransform{utpp::mm::math::Vec3{aP[size_t(i) * 3], aP[size_t(i) * 3 + 1], aP[size_t(i) * 3 + 2]},
                                       utpp::mm::math::Quat{aQ[size_t(i) * 4], aQ[size_t(i) * 4 + 1], aQ[size_t(i) * 4 + 2], aQ[size_t(i) * 4 + 3]}};
    };
    int stepped = 0;
    for (int c = 0; c < 2; ++c)
    {
        const int32_t a = chains[c][0], b = chains[c][1], e = chains[c][2], t = chains[c][3];
        if (a < 0 || b < 0 || e < 0 || t < 0 || static_cast<uint32_t>(a) >= aS.n || static_cast<uint32_t>(b) >= aS.n ||
            static_cast<uint32_t>(e) >= aS.n || static_cast<uint32_t>(t) >= aS.n)
            continue;
        const int32_t p = aS.parents[a];
        if (p < 0 || static_cast<uint32_t>(p) >= aS.n)
            continue;
        const float* th = Ph + size_t(t) * 3;
        const float* tm = Pm + size_t(t) * 3;
        const float d = std::hypot(th[0] - tm[0], th[1] - tm[1]);
        const float planted = 1.0f - gu::Smooth((th[2] - aZGround - kGuStepPlantedM) / 0.06f);
        const float k = gu::Smooth((d - kGuStepMinM) / (kGuStepFullM - kGuStepMinM));
        const float h = std::fmin(kGuStepLiftM, kGuStepFrac * d) * k * planted * prog;
        if (!(h > 1e-4f))
            continue;
        const utpp::mm::BoneTransform gp = bt(Pb, Qb, p), ga = bt(Pb, Qb, a), gb = bt(Pb, Qb, b), gc = bt(Pb, Qb, e);
        const utpp::mm::math::Vec3 target{gc.p.x, gc.p.y, gc.p.z + h};
        const utpp::mm::legik::TwoBoneSolution s =
            utpp::mm::legik::SolveTwoBone(gp, ga, gb, gc, target, bt(Pb, Qb, t).p, 0.995f, 0.97f, true, true);
        if (s.status != utpp::mm::legik::TwoBoneStatus::kOk)
            continue;
        const int32_t bones[3] = {a, b, e};
        for (int j = 0; j < 3; ++j)
            gu::Put(aBlended + size_t(bones[j]) * 7 + 3, gu::Q{s.local[j].x, s.local[j].y, s.local[j].z, s.local[j].w});
        ++stepped;
    }
    return stepped;
#endif
}

// ------------------------------------------------------------------------------------------------ phase machine
class GetUpPlayer
{
public:
    struct TickResult
    {
        bool blendDone = false;      // Blend -> Play
        bool handOffStarted = false; // -> HandOff in this tick
        bool ended = false;          // back to Idle in this tick (reason set)
        const char* reason = nullptr;
    };

    bool Active() const noexcept { return m_phase != GuPhase::Idle; }
    GuPhase Phase() const noexcept { return m_phase; }
    const GuPlan& Plan() const noexcept { return m_plan; }
    uint32_t Runs() const noexcept { return m_runs; }
    const char* EndReason() const noexcept { return m_endReason; }

    bool Start(const GuPlan& aPlan, double aWallNow)
    {
        if (m_phase != GuPhase::Idle || !aPlan.ok || aPlan.clip < 0 || aPlan.clip >= kGuClipCount || !(aPlan.handOffS > 0.0f) ||
            !std::isfinite(aWallNow))
            return false;
        m_plan = aPlan;
        m_clock = m_clip = m_feed = m_handLeft = 0.0f;
        m_phase = GuPhase::Blend;
        m_startWall = m_lastWall = aWallNow;
        m_pausedWall = 0.0;
        m_anchorSeen = false;
        m_endReason = "none";
        ++m_runs;
        return true;
    }

    // Feed-time tick. aCapsuleOffM: XY distance of the live capsule from the anchor (< 0 = unknown). aPaused: menu / photo (the
    // clock and the wall watchdog stop).
    TickResult Advance(float aDt, [[maybe_unused]] bool aInputMove, float aCapsuleOffM, double aWallNow, bool aPaused = false)
    {
        TickResult r;
        if (m_phase == GuPhase::Idle)
            return r;
        if (WallHit(aWallNow, aPaused))
        {
            Finish("watchdog-wall", r);
            return r;
        }
        if (aPaused)
            return r;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, kGuMaxStepS) : 0.0f;
        if (m_phase == GuPhase::HandOff)
        {
            m_handLeft -= dt;
            if (m_handLeft <= 0.0f)
                Finish("done", r);
            return r;
        }
        {
            // the clip clock runs at kGuBlendClipRate while the blend runs, at 1 after it
            const float inBlend = std::fmax(0.0f, std::fmin(dt, BlendInS() - m_clock));
            m_clip += inBlend * kGuBlendClipRate + (dt - inBlend);
        }
        m_clock += dt;
        m_feed += dt;
        if (m_feed > m_plan.lengthS + kGuWatchdogFeedPadS)
        {
            Finish("watchdog-feed", r);
            return r;
        }
        if (aCapsuleOffM >= 0.0f && aCapsuleOffM <= kGuArmWithinM)
            m_anchorSeen = true;
        if (m_phase == GuPhase::Blend && m_clock >= BlendInS())
        {
            m_phase = GuPhase::Play;
            r.blendDone = true;
        }
        const char* why = nullptr;
        if (m_clip >= m_plan.handOffS)
            why = "clip-end";
#if UTPP_GETUP_MUTATION == 7
        // mutation 7: neither the early input hand-off nor the off-anchor hand-off exists
#else
        else if (m_clip >= m_plan.interruptS && aInputMove)
            why = "input";
        else if (m_clip < m_plan.interruptS && m_anchorSeen && aCapsuleOffM > kGuOffAnchorM)
            why = "input-interrupt";
#endif
        if (why != nullptr)
        {
            m_phase = GuPhase::HandOff;
            m_handLeft = kGuBlendOutS;
            m_handReason = why;
            r.handOffStarted = true;
            r.reason = why;
        }
        return r;
    }

    // Wall-clock watchdog only (a publish path that may run without any feed). true = it ended the run.
    bool Watchdog(double aWallNow, bool aPaused = false)
    {
        if (m_phase == GuPhase::Idle || !WallHit(aWallNow, aPaused))
            return false;
        TickResult r;
        Finish("watchdog-wall", r);
        return true;
    }

    // Any phase -> Idle at once (the glue hands back with the 0.12 s tail).
    void Abort(const char* aWhy)
    {
        if (m_phase == GuPhase::Idle)
            return;
        TickResult r;
        Finish(aWhy ? aWhy : "abort", r);
    }

    float ClipTime() const noexcept { return std::fmin(m_clip, m_plan.handOffS); }
    float BlendInS() const noexcept
    {
        return m_plan.blendInS > 0.0f ? std::clamp(m_plan.blendInS, kGuBlendInS, kGuBlendMaxS) : kGuBlendInS;
    }
    float BlendW() const noexcept
    {
        if (m_phase == GuPhase::Idle)
            return 0.0f;
        return gu::Smooth(m_clock / BlendInS());
    }
    float HandOffLeft() const noexcept { return m_phase == GuPhase::HandOff ? std::fmax(m_handLeft, 0.0f) : 0.0f; }
    const char* HandOffReason() const noexcept { return m_handReason; }
    float FeedTime() const noexcept { return m_feed; }

private:
    bool WallHit(double aWallNow, bool aPaused)
    {
        if (!std::isfinite(aWallNow))
            return true;
        if (aPaused)
            m_pausedWall += std::fmax(0.0, aWallNow - m_lastWall);
        m_lastWall = aWallNow;
        return aWallNow - m_startWall - m_pausedWall > kGuWallWatchdogS;
    }
    void Finish(const char* aWhy, TickResult& aR)
    {
        m_phase = GuPhase::Idle;
        m_endReason = aWhy;
        aR.ended = true;
        aR.reason = aWhy;
    }

    GuPhase m_phase = GuPhase::Idle;
    GuPlan m_plan;
    float m_clock = 0.0f, m_clip = 0.0f, m_feed = 0.0f, m_handLeft = 0.0f;
    double m_startWall = 0.0, m_lastWall = 0.0, m_pausedWall = 0.0;
    bool m_anchorSeen = false;
    const char* m_endReason = "none";
    const char* m_handReason = "";
    uint32_t m_runs = 0;
};

// ------------------------------------------------------------------------------------------------ loader
struct GuClips
{
    bool ok = false;
    std::string err;
    std::vector<int32_t> poseMap; // database bone -> pose bone of the file
    GuClipInfo clip[kGuClipCount];
    GuBones bones;
};

// Samples a montage's pose at aTime into aOut (n * 7, database bone order): frame lerp + shortest-arc nlerp, clamped to the frames.
inline void SampleGetUpPose(const mm::traversal::TravMontage& aM, size_t aPoseBones, const std::vector<int32_t>& aMap, double aTime, float* aOut)
{
    double ft = (std::isfinite(aTime) ? aTime : 0.0) * aM.fps;
    ft = std::clamp(ft, 0.0, static_cast<double>(aM.frames - 1));
    const int f0 = static_cast<int>(ft);
    const int f1 = f0 + 1 < aM.frames ? f0 + 1 : f0;
    const float u = static_cast<float>(ft - f0);
    const float* a = &aM.pose[static_cast<size_t>(f0) * aPoseBones * 7];
    const float* c = &aM.pose[static_cast<size_t>(f1) * aPoseBones * 7];
    for (size_t i = 0; i < aMap.size(); ++i)
    {
        const size_t k = static_cast<size_t>(aMap[i]) * 7;
        float* o = aOut + i * 7;
        for (int j = 0; j < 3; ++j)
            o[j] = a[k + j] + (c[k + j] - a[k + j]) * u;
        gu::Put(o + 3, gu::Nlerp(gu::Get(a + k + 3), gu::Get(c + k + 3), u));
    }
}

// Validates a parsed utpp_getup_<body>.utpptrav against the database (names, parents) and precomputes the per-clip data.
// aRigName: "player_woman_skeleton" / "player_man_skeleton". Fail closed: ok = false and err set.
template <class NameFn>
inline bool BuildGetUpClips(const mm::traversal::TraversalFile& aFile, const std::string& aRigName, uint32_t aN, uint32_t aFirstBone,
                            const int32_t* aParents, NameFn&& aName, GuClips& aOut)
{
    aOut = GuClips{};
    const auto fail = [&](const std::string& aWhy) {
        aOut.ok = false;
        aOut.err = aWhy;
        return false;
    };
    if (aN == 0 || aN > kGuMaxBones || aN <= aFirstBone)
        return fail("bone count");
    if (aFile.version != 2 || aFile.poseBones.empty())
        return fail("no poses (version 2 expected)");
    if (aFile.rigName != aRigName)
        return fail("rig " + aFile.rigName + " != " + aRigName);
    for (uint32_t i = aFirstBone; i < aN; ++i)
        if (aParents[i] >= static_cast<int32_t>(i))
            return fail("bone order (parent after child)");
    const size_t nb = aFile.poseBones.size();
    aOut.poseMap.assign(aN, -1);
    for (uint32_t i = 0; i < aN; ++i)
    {
        const std::string_view nm = aName(i);
        for (size_t k = 0; k < nb; ++k)
            if (aFile.poseBones[k] == nm)
                aOut.poseMap[i] = static_cast<int32_t>(k);
        if (aOut.poseMap[i] < 0)
            return fail("pose bone missing: " + std::string(nm));
    }
    aOut.bones = ResolveGuBones(aN, aName);
    if (!aOut.bones.jointsOk)
        return fail("named joints missing");
    std::vector<float> loc(size_t(aN) * 7), P(size_t(aN) * 3), Q(size_t(aN) * 4);
    for (int c = 0; c < kGuClipCount; ++c)
    {
        const mm::traversal::TravMontage* m = aFile.Find(kGuClipNames[c]);
        if (!m)
            return fail(std::string("montage missing: ") + kGuClipNames[c]);
        if (!m->HasPose() || m->frames < 2 || m->pose.size() != static_cast<size_t>(m->frames) * nb * 7 || !(m->fps > 0.0))
            return fail(std::string("poses: ") + kGuClipNames[c]);
        double handOff = -1.0, interrupt = -1.0;
        for (const auto& s : m->sections)
        {
            if (s.name == "HandOff")
                handOff = s.startS;
            else if (s.name == "Interrupt")
                interrupt = s.startS;
        }
        const double lastT = (m->frames - 1) / m->fps;
        if (!(handOff > 0.0) || handOff > lastT + 1e-6)
            return fail(std::string("SEC HandOff: ") + kGuClipNames[c]);
        if (!(interrupt >= 0.0) || interrupt > handOff)
            return fail(std::string("SEC Interrupt: ") + kGuClipNames[c]);
        GuClipInfo& ci = aOut.clip[c];
        ci.frames = static_cast<uint32_t>(m->frames);
        ci.fps = static_cast<float>(m->fps);
        ci.lengthS = static_cast<float>(m->lengthS);
        ci.handOffS = static_cast<float>(handOff);
        ci.interruptS = static_cast<float>(interrupt);
        SampleGetUpPose(*m, nb, aOut.poseMap, 0.0, loc.data());
        ci.frame0Local = loc;
        gu::FkModel(aParents, aN, aFirstBone, loc.data(), P.data(), Q.data());
        ci.frame0Model = P;
        const auto at = [&](int j) { return P.data() + size_t(aOut.bones.joint[j]) * 3; };
        // standing sign: the belly normal of the pose at the hand-off frame points forward (+Y) in the clip root frame
        SampleGetUpPose(*m, nb, aOut.poseMap, handOff, loc.data());
        std::vector<float> P1(size_t(aN) * 3), Q1(size_t(aN) * 4);
        gu::FkModel(aParents, aN, aFirstBone, loc.data(), P1.data(), Q1.data());
        const auto at1 = [&](int j) { return P1.data() + size_t(aOut.bones.joint[j]) * 3; };
        float n1[3];
        if (!BellyNormal(at1(kJHips), at1(kJSpine), at1(kJLUp), at1(kJRUp), n1))
            return fail(std::string("standing pose degenerate: ") + kGuClipNames[c]);
        ci.standSign = n1[1] >= 0.0f ? 1.0f : -1.0f;
        ci.hipsLastZ = at1(kJHips)[2];
        if (!LieAxes(at(kJHips), at(kJSpine), at(kJLUp), at(kJRUp), ci.standSign, ci.feat[0], ci.feat[1]))
            return fail(std::string("frame 0 degenerate: ") + kGuClipNames[c]);
        ci.ok = true;
    }
    aOut.ok = true;
    aOut.err.clear();
    return true;
}
} // namespace utpp::ragdoll

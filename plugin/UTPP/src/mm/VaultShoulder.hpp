#pragma once
// Vault shoulder limiter (2026-11-15, car vault released to the game animation, every weapon state). Header-only, no RED4ext / Windows
// types; unit-tested offline (tests/VaultShoulderTest.cpp, real captured poses of the vault torso fixture).
//
// PROBLEM: a vault over a vehicle is not played by v2 (MotionTraversal.hpp Config::vehicleVaultToGame, Unsupported(vehicle)); the v1 gate
// releases the body to the game's own vault animation (release reason travRelease, MM weight 0). In that clip the clavicles swing 56 - 88 deg
// and their joints move toward the spine: the distance of the shoulder joints (LeftArm -> RightArm along the lateral axis of the Spine3 frame)
// falls from about 33 - 38 cm to 19 - 22 cm for 20 - 54 frames, unarmed or armed. The width measure of the body allows 32.0 - 40.3 cm.
//
// FIX: while the gate is on (a released vault 9 over a vehicle v2 rejected, any weapon state) the FK detour brings the clavicles back toward
// V's own pose so that the shoulder width stays above a soft floor:
//   - the width only depends on the local records of LeftShoulder / RightShoulder (children of Spine3) and the translations of LeftArm /
//     RightArm: no FK pass, the whole solve is in the Spine3 frame;
//   - reference = V's own clavicle records, latched on the first FK call of each episode (the pose shown when the release starts: the motion
//     pose with the body's own shoulder setting, or the game's locomotion pose);
//   - soft floor: identity at and above kFloorM, below it the deficit d is compressed to knee * tanh(d / knee) (asymptote kFloorM - kKneeM =
//     32.5 cm, above the lower band edge of both bodies: Female 32.00, Male 32.02 cm; base width 38.45 cm on both);
//   - the needed widening is split over the two sides by how far each side is narrower than its reference, so a side that keeps its own
//     width stays as the game posed it; per side one parameter s in [0, 1] moves the clavicle joint toward the reference translation and swings
//     the clavicle toward the lateral axis (about the axis normal to both) until, at s = 1, its angle from that axis is the reference's; solved
//     by bisection. s = 1 is the most the limiter ever does: no shoulder joint ends farther from the mid plane than in V's own pose;
//   - the arms keep their own rotations: the upper-arm local rotation is rewritten so that its rotation in the Spine3 frame stays the game's
//     (q_arm' = q_clav'^-1 * q_clav * q_arm). Forearms, hands and weapons follow rigidly: the hands translate by exactly the displacement of
//     the shoulder joint (no swing of the whole arm);
//   - written records: LeftShoulder / RightShoulder (translation and rotation), LeftArm / RightArm (rotation only); nothing else.
//   - fade 0.06 s in (the correction is near zero when the release starts) and 0.15 s out; exit hold while the measured width is still below
//     kHoldBelowM (the clip is still recovering when the PSM leaves Vault 9), at most kMaxHoldSeconds.
//
// Runtime glue at the end (namespace rt): game thread Feed (MotionHost, next to the vault torso feed), FK detour Apply (FkHook.cpp
// ApplyMotionMatching, right after the vault torso pass, before the owner check: it runs under every pose owner).

#include <atomic>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "AimOffset.hpp"
#include "GameIntegration.hpp"
#include "MMMath.hpp"
#include "VaultTorso.hpp"

#ifndef UTPP_VAULTSHOULDER_MUTATION
#define UTPP_VAULTSHOULDER_MUTATION 0 // test builds: 1 = the pass is compiled out, 2 = the gate needs a ranged weapon (the vault torso gate),
                                      // 3 = no exit hold, 4 = no upper-arm compensation, 5 = the FK pass needs MM active:
                                      // utpp_vehexit_vaultshoulder_mut1..5 MUST FAIL
#endif

namespace utpp::mm::vaultshoulder
{
using aim::Xf;
using math::Quat;
using math::Vec3;

inline constexpr float kFloorM = 0.340f;         // shoulder width (m) at and above which nothing is changed
inline constexpr float kKneeM = 0.015f;          // the deficit below the floor saturates at this (asymptote 0.325 m)
inline constexpr float kBandMinM = 0.3202f;      // lower edge of the width band, the larger of both bodies (tests)
inline constexpr float kHoldBelowM = 0.325f;     // exit hold while the measured width (before the limiter) is below this
inline constexpr float kFadeInSeconds = 0.06f;
inline constexpr float kFadeOutSeconds = 0.15f;
inline constexpr float kMaxHoldSeconds = 0.60f;
inline constexpr float kStaleSeconds = 0.25f;      // a published weight older than this is not applied (the feed stopped)
inline constexpr float kWidthStaleSeconds = 0.10f; // the measured width older than this does not hold the exit fade
inline constexpr int kDetailVault = 9;             // gamePSMDetailedLocomotionStates.Vault

// ------------------------------------------------------------------------------------------------ gate

// Where the limiter runs: a vault (PSM detail 9) released to the game animation (travRelease) over a vehicle v2 rejected, on foot. Any weapon
// state: unarmed, melee, ranged. aVehicleVault = rt::g_vehicleVault (set by the motion driver with the vault torso event, no overlay test).
inline bool Wanted(const vaulttorso::GateInput& aIn, bool aVehicleVault)
{
#if UTPP_VAULTSHOULDER_MUTATION == 2
    return vaulttorso::Wanted(aIn) && aVehicleVault; // mutation 2: only with a ranged weapon drawn (the vault torso gate)
#else
    return vaulttorso::FootOnly(aIn) && aIn.travRelease && aIn.psmDetail == kDetailVault && aVehicleVault;
#endif
}
// The vault is over but the clip is still recovering: the weight may be held in the same episode while the PSM is back on plain locomotion
// (none / Stand / AimWalk / Sprint). A crouch, slide, mantle, ladder, knockdown, a mount or scene bit ends the hold.
inline bool HoldAllowed(const vaulttorso::GateInput& aIn, bool aEpisode)
{
    const int d = aIn.psmDetail;
    const bool plain = d == -1 || d == 1 || d == 2 || d == 4;
    return vaulttorso::FootOnly(aIn) && aEpisode && plain;
}

// Soft floor of a width (m): identity at and above the floor, below it the deficit d is compressed to knee * tanh(d / knee).
inline float Limit(float aWidth)
{
    if (!(aWidth < kFloorM))
        return aWidth;
    return kFloorM - kKneeM * std::tanh((kFloorM - aWidth) / kKneeM);
}

// ------------------------------------------------------------------------------------------------ rig

struct Rig
{
    int32_t spine3 = -1;
    int32_t clav[2] = {-1, -1}; // LeftShoulder, RightShoulder
    int32_t arm[2] = {-1, -1};  // LeftArm, RightArm
    uint32_t count = 0;          // max index used + 1
    uint32_t ok = 0;             // 1 = every check passed (fail closed: 0 -> never applied)
};
static_assert(sizeof(Rig) % 4 == 0);

// Resolves Spine3, LeftShoulder, RightShoulder, LeftArm, RightArm by name. Fail closed (ok = 0): a name is missing, an index >= aim::kMaxFkBones,
// a shoulder is not a direct child of Spine3 or an upper arm not a direct child of its shoulder.
inline Rig ResolveRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return Rig{};
    static constexpr std::string_view kNames[5] = {"Spine3", "LeftShoulder", "RightShoulder", "LeftArm", "RightArm"};
    int32_t idx[5] = {-1, -1, -1, -1, -1};
    int32_t maxIdx = -1;
    for (int k = 0; k < 5; ++k)
    {
        const uint64_t h = integ::CNameHash(kNames[k]);
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == h)
            {
                idx[k] = static_cast<int32_t>(i);
                break;
            }
        if (idx[k] < 0 || idx[k] >= int32_t(aim::kMaxFkBones))
            return Rig{};
        maxIdx = idx[k] > maxIdx ? idx[k] : maxIdx;
    }
    if (aParents[idx[1]] != idx[0] || aParents[idx[2]] != idx[0] || aParents[idx[3]] != idx[1] || aParents[idx[4]] != idx[2])
        return Rig{};
    Rig r;
    r.spine3 = idx[0];
    r.clav[0] = idx[1];
    r.clav[1] = idx[2];
    r.arm[0] = idx[3];
    r.arm[1] = idx[4];
    r.count = static_cast<uint32_t>(maxIdx + 1);
    r.ok = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ limiter

// V's own clavicle records of the episode (rt latches them on the first FK call).
struct Ref
{
    Vec3 t[2] = {};
    Quat q[2] = {Quat{0, 0, 0, 1}, Quat{0, 0, 0, 1}};
    float half[2] = {0.0f, 0.0f}; // m, distance of the shoulder joint from the Spine3 mid plane
    bool ok = false;
};

struct Result
{
    bool applied = false;        // records were written
    bool saturated = false;      // the floor was out of reach (both sides at the reference)
    float widthBefore = 0.0f;    // m
    float widthAfter = 0.0f;     // m
    float s[2] = {0.0f, 0.0f};   // per side share of the way back to the reference
    float shiftM[2] = {0.0f, 0.0f}; // displacement of the shoulder joint (= of the hand: the arm keeps its Spine3-frame rotation), m
};

inline float SideSign(int aSide) { return aSide == 0 ? -1.0f : 1.0f; } // left = -Z of the Spine3 frame, right = +Z

// Shoulder joint (upper-arm joint) in the Spine3 frame from the clavicle record and the upper-arm translation.
inline Vec3 JointOf(Vec3 aClavT, const Quat& aClavQ, Vec3 aArmT)
{
    return aClavT + math::QuatRotate(aClavQ, aArmT);
}

// Distance of each shoulder joint from the Spine3 mid plane and their sum (the width) on the local records; false = rig / records unusable.
inline bool MeasureWidth(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, float aHalf[2], float& aWidth)
{
    if (!aRig.ok || !aBones || aRig.count == 0 || aRig.count > aCount)
        return false;
    for (int k = 0; k < 2; ++k)
    {
        const Xf c = aim::ReadLocal(aBones, uint32_t(aRig.clav[k]));
        const Xf a = aim::ReadLocal(aBones, uint32_t(aRig.arm[k]));
        const Vec3 p = JointOf(c.p, math::QuatNormalize(c.q), a.p);
        if (!math::IsFinite(p) || !math::IsFinite(c.q))
            return false;
        aHalf[k] = SideSign(k) * p.z;
    }
    aWidth = aHalf[0] + aHalf[1];
    return std::isfinite(aWidth);
}

inline bool Latch(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, Ref& aRef)
{
    aRef = Ref{};
    float w = 0.0f;
    if (!MeasureWidth(aBones, aCount, aRig, aRef.half, w))
        return false;
    for (int k = 0; k < 2; ++k)
    {
        const Xf c = aim::ReadLocal(aBones, uint32_t(aRig.clav[k]));
        aRef.t[k] = c.p;
        aRef.q[k] = math::QuatNormalize(c.q);
    }
    aRef.ok = true;
    return true;
}

// Limits the shoulder width on the local records of one FK call; aWeight 0..1 scales the correction (the faded share). False = nothing
// written (bad rig / records / reference / weight 0). No allocation.
inline bool Apply(uint8_t* aBones, uint32_t aCount, const Rig& aRig, const Ref& aRef, float aWeight, Result* aOut = nullptr)
{
    Result res;
    if (aOut)
        *aOut = res;
#if UTPP_VAULTSHOULDER_MUTATION == 1
    (void)aBones; (void)aCount; (void)aRig; (void)aRef; (void)aWeight;
    return false; // mutation 1: the pass is compiled out (the build before)
#else
    if (!aRig.ok || !aRef.ok || !aBones || aRig.count == 0 || aRig.count > aCount || !std::isfinite(aWeight) || !(aWeight > 0.0f))
        return false;
    const float w = aim::Clampf(aWeight, 0.0f, 1.0f);
    float half[2] = {0.0f, 0.0f}, width = 0.0f;
    if (!MeasureWidth(aBones, aCount, aRig, half, width))
        return false;
    res.widthBefore = res.widthAfter = width;
    const float want = (Limit(width) - width) * w;
    if (!(want > 1e-6f))
    {
        if (aOut)
            *aOut = res;
        return true; // at or above the floor: identity
    }
    Xf clav[2], arm[2];
    float deficit[2], sum = 0.0f;
    for (int k = 0; k < 2; ++k)
    {
        clav[k] = aim::ReadLocal(aBones, uint32_t(aRig.clav[k]));
        clav[k].q = math::QuatNormalize(clav[k].q);
        arm[k] = aim::ReadLocal(aBones, uint32_t(aRig.arm[k]));
        deficit[k] = aRef.half[k] - half[k] > 0.0f ? aRef.half[k] - half[k] : 0.0f;
        sum += deficit[k];
    }
    if (!(sum > 1e-6f))
    {
        res.saturated = true; // both sides already at least as wide as V's own pose: nothing to bring back
        if (aOut)
            *aOut = res;
        return true;
    }
    // per side: the clavicle swings about the axis normal to its own direction and the side's lateral axis (the shortest way out), by
    // s * (its angle from the lateral axis - the reference's), while its joint moves s of the way back to the reference translation. The
    // swing changes the protraction / elevation only as much as the lateral gain needs (a slerp toward the reference would also undo the
    // clip's elevation for a 1 cm gain), and the half width grows monotonically with s.
    Vec3 axis[2];
    float swing[2] = {0.0f, 0.0f};
    for (int k = 0; k < 2; ++k)
    {
        const Vec3 lat{0.0f, 0.0f, SideSign(k)};
        Vec3 d{}, dr{};
        axis[k] = Vec3{0.0f, 0.0f, 0.0f};
        if (!aim::NormalizeTo(math::QuatRotate(clav[k].q, arm[k].p), d) || !aim::NormalizeTo(math::QuatRotate(aRef.q[k], arm[k].p), dr))
            continue;
        const float phi = aim::AngleBetween(d, lat), phiRef = aim::AngleBetween(dr, lat);
        if (phi > phiRef && aim::NormalizeTo(math::Cross(d, lat), axis[k]))
            swing[k] = phi - phiRef;
    }
    auto poseAt = [&](int k, float s, Vec3& aT, Quat& aQ) {
        aT = math::Lerp(clav[k].p, aRef.t[k], s);
        aQ = swing[k] > 0.0f ? math::QuatNormalize(math::QuatMul(aim::AxisAngle(axis[k], s * swing[k]), clav[k].q)) : clav[k].q;
    };
    auto halfAt = [&](int k, float s) {
        Vec3 t;
        Quat q;
        poseAt(k, s, t, q);
        return SideSign(k) * JointOf(t, q, arm[k].p).z;
    };
    // returns what this side could not give: never beyond its own reference half width (a side whose clavicle is already nearer the lateral
    // axis than the reference's would otherwise overshoot it through the translation alone)
    auto solve = [&](int k, float target, float& s) {
        float extra = 0.0f;
        if (target > aRef.half[k])
        {
            extra = target - aRef.half[k];
            target = aRef.half[k];
        }
        const float h1 = halfAt(k, 1.0f);
        if (h1 <= target)
        {
            s = 1.0f;
            return extra + (target - h1);
        }
        float lo = 0.0f, hi = 1.0f;
        for (int it = 0; it < 30; ++it)
        {
            const float mid = 0.5f * (lo + hi);
            (halfAt(k, mid) < target ? lo : hi) = mid;
        }
        s = hi;
        return extra;
    };
    float s[2] = {0.0f, 0.0f}, left[2] = {0.0f, 0.0f};
    for (int k = 0; k < 2; ++k)
        left[k] = deficit[k] > 0.0f ? solve(k, half[k] + want * deficit[k] / sum, s[k]) : 0.0f;
    // what one side could not give goes to the other (once)
    for (int k = 0; k < 2; ++k)
        if (left[1 - k] > 0.0f && s[k] < 1.0f && deficit[k] > 0.0f)
            left[k] = solve(k, halfAt(k, s[k]) + left[1 - k], s[k]);
    res.saturated = s[0] >= 1.0f && s[1] >= 1.0f;
    for (int k = 0; k < 2; ++k)
    {
        res.s[k] = s[k];
        if (!(s[k] > 0.0f))
            continue;
        Vec3 t;
        Quat q;
        poseAt(k, s[k], t, q);
        float* r =reinterpret_cast<float*>(aBones + size_t(aRig.clav[k]) * integ::kGameBoneStride);
        r[0] = t.x;
        r[1] = t.y;
        r[2] = t.z;
        aim::WriteLocalRot(aBones, uint32_t(aRig.clav[k]), q);
#if UTPP_VAULTSHOULDER_MUTATION != 4
        // the upper arm keeps its rotation in the Spine3 frame (mutation 4: it turns with the clavicle)
        const Quat qa = math::QuatNormalize(math::QuatMul(math::QuatInv(q), math::QuatMul(clav[k].q, math::QuatNormalize(arm[k].q))));
        aim::WriteLocalRot(aBones, uint32_t(aRig.arm[k]), qa);
#endif
        res.shiftM[k] = aim::Length(JointOf(t, q, arm[k].p) - JointOf(clav[k].p, clav[k].q, arm[k].p));
    }
    res.applied = s[0] > 0.0f || s[1] > 0.0f;
    float h2[2];
    if (res.applied)
        MeasureWidth(aBones, aCount, aRig, h2, res.widthAfter);
    if (aOut)
        *aOut = res;
    return true;
#endif
}

// ------------------------------------------------------------------------------------------------ fade

// Weight: 0 -> 1 in kFadeInSeconds while the gate is on, 1 -> 0 in kFadeOutSeconds after it (smoothstep of a linear ramp). aHold (the gate
// ended but the shown width is still below kHoldBelowM): the weight stays, at most kMaxHoldSeconds.
class Fade
{
public:
    void Reset() { m_lin = 0.0f; m_hold = 0.0f; }
    float Weight() const { return m_lin * m_lin * (3.0f - 2.0f * m_lin); }
    float Step(float aDt, bool aWanted, bool aHold)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > 0.1f ? 0.1f : aDt) : 0.0f;
        if (aWanted)
        {
            m_hold = 0.0f;
            m_lin = aim::Clampf(m_lin + dt / kFadeInSeconds, 0.0f, 1.0f);
        }
        else if (aHold && m_lin > 0.0f && m_hold < kMaxHoldSeconds)
            m_hold += dt;
        else
            m_lin = aim::Clampf(m_lin - dt / kFadeOutSeconds, 0.0f, 1.0f);
        return Weight();
    }

private:
    float m_lin = 0.0f, m_hold = 0.0f;
};

// ------------------------------------------------------------------------------------------------ runtime glue

namespace rt
{
struct RigMsg
{
    Rig rig;
    uint64_t rigPtr = 0;
};
static_assert(sizeof(RigMsg) % 4 == 0);

inline integ::SeqlockBuffer<RigMsg> g_rig;
// set by the motion driver together with its vault torso event: the last planned traversal event is a vault over a vehicle that v2 did not
// accept (any overlay); cleared with the episode
inline std::atomic<bool> g_vehicleVault{false};
// published by the game thread (rt::Feed), read by the FK detour
inline std::atomic<float> g_weight{0.0f};
inline std::atomic<int64_t> g_pubTicks{0};
inline std::atomic<bool> g_foot{false};
inline std::atomic<uint32_t> g_episodeId{0}; // +1 on every episode start (the FK side latches the reference once per id)
// measured by the FK detour (width before the limiter), read by the game thread for the exit hold
inline std::atomic<float> g_widthM{1.0f};
inline std::atomic<int64_t> g_widthTicks{0};
// diagnostics
inline std::atomic<uint64_t> g_calls{0}, g_applied{0}, g_saturated{0};
inline std::atomic<float> g_minWidthM{1.0f}, g_maxShiftM{0.0f};
#if UTPP_VAULTSHOULDER_MUTATION == 5
inline std::atomic<bool> g_mmActiveMirror{true}; // mutation 5 / tests: the "MM is active" flag (false at MM weight 0)
#endif
// game thread only (MMDriver Feed and OnDisarmed, both under the v1 mutex)
inline Fade g_fade;
inline bool g_on = false;
inline bool g_episode = false;
// FK detour only (one player FK call at a time): the reference of the episode g_refId
inline Ref g_ref;
inline uint32_t g_refId = 0;

inline double Seconds(int64_t aNew, int64_t aOld, int64_t aTps)
{
    return aTps > 0 ? static_cast<double>(aNew - aOld) / static_cast<double>(aTps) : 1e9;
}

// Game thread, arm time (resolved by name, fail closed).
inline void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RigMsg m;
    m.rig = ResolveRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    m.rigPtr = reinterpret_cast<uint64_t>(aRig);
    g_rig.Publish(m);
}
// Game thread, from motionhost::OnDisarmed (under the v1 mutex, like Feed): forgets the rig and the episode.
inline void OnDisarmed()
{
    g_rig.Publish(RigMsg{});
    g_fade.Reset();
    g_on = false;
    g_episode = false;
    g_weight.store(0.0f, std::memory_order_relaxed);
    g_foot.store(false, std::memory_order_relaxed);
    g_widthM.store(1.0f, std::memory_order_relaxed);
    g_widthTicks.store(0, std::memory_order_relaxed);
    g_episodeId.fetch_add(1, std::memory_order_relaxed); // a stale reference is never reused
}

// Game thread, once per feed: steps the fade and publishes the weight. +1 on the feed that starts an episode, -1 on the feed that ends it.
// Not foot-only: the weight drops to 0 at once. Wanted: rises in 0.06 s. Not wanted: falls in 0.15 s, unless HoldAllowed and the FK side
// measured the width below kHoldBelowM within the last 0.1 s: then held, at most 0.6 s.
inline int Feed(const vaulttorso::GateInput& aGate, bool aVehicleVault, float aDt, int64_t aNowTicks, int64_t aTps)
{
    const bool foot = vaulttorso::FootOnly(aGate);
    float w = 0.0f;
    if (!foot)
    {
        g_fade.Reset();
        g_episode = false;
    }
    else
    {
        const bool wanted = vaultshoulder::Wanted(aGate, aVehicleVault);
        if (wanted)
            g_episode = true;
        const bool measured = Seconds(aNowTicks, g_widthTicks.load(std::memory_order_relaxed), aTps) <= kWidthStaleSeconds;
#if UTPP_VAULTSHOULDER_MUTATION == 3
        const bool hold = false; // mutation 3: no exit hold
        (void)measured;
#else
        const bool hold = !wanted && vaultshoulder::HoldAllowed(aGate, g_episode) && measured && g_widthM.load(std::memory_order_relaxed) < kHoldBelowM;
#endif
        w = g_fade.Step(aDt, wanted, hold);
        if (!(w > 0.0f))
            g_episode = false;
    }
    int edge = 0;
    if (!g_on && w > 0.0f)
    {
        g_on = true;
        edge = 1;
        g_episodeId.fetch_add(1, std::memory_order_relaxed);
        g_minWidthM.store(1.0f, std::memory_order_relaxed);
        g_maxShiftM.store(0.0f, std::memory_order_relaxed);
    }
    else if (g_on && !(w > 0.0f))
    {
        g_on = false;
        edge = -1;
    }
    g_foot.store(foot, std::memory_order_relaxed);
    g_weight.store(w, std::memory_order_relaxed);
    g_pubTicks.store(aNowTicks, std::memory_order_relaxed);
    return edge;
}

// FK detour, cheap gate: one relaxed load while idle.
inline bool Active(int64_t aNowTicks, int64_t aTps)
{
    return g_weight.load(std::memory_order_relaxed) > 0.0f && g_foot.load(std::memory_order_relaxed) &&
           Seconds(aNowTicks, g_pubTicks.load(std::memory_order_relaxed), aTps) <= kStaleSeconds;
}

// FK detour (animation thread): the limiter on the local records of the player rig. No allocation, no locks, no logging.
inline bool Apply(uint8_t* aBones, uint32_t aCount, void* aRig, int64_t aNowTicks, int64_t aTps)
{
    if (!Active(aNowTicks, aTps))
        return false;
    RigMsg m;
    if (!aBones || !g_rig.Read(m) || !m.rig.ok || m.rigPtr != reinterpret_cast<uint64_t>(aRig))
        return false;
    g_calls.fetch_add(1, std::memory_order_relaxed);
    const uint32_t id = g_episodeId.load(std::memory_order_relaxed);
    if (g_refId != id || !g_ref.ok)
    {
        if (!Latch(aBones, aCount, m.rig, g_ref))
            return false;
        g_refId = id;
    }
    float half[2], width = 0.0f;
    if (!MeasureWidth(aBones, aCount, m.rig, half, width))
        return false;
    g_widthM.store(width, std::memory_order_relaxed);
    g_widthTicks.store(aNowTicks, std::memory_order_relaxed);
    Result r;
    if (!vaultshoulder::Apply(aBones, aCount, m.rig, g_ref, g_weight.load(std::memory_order_relaxed), &r) || !r.applied)
        return false;
    g_applied.fetch_add(1, std::memory_order_relaxed);
    if (r.saturated)
        g_saturated.fetch_add(1, std::memory_order_relaxed);
    float cur = g_minWidthM.load(std::memory_order_relaxed);
    while (width < cur && !g_minWidthM.compare_exchange_weak(cur, width, std::memory_order_relaxed))
    {
    }
    const float shift = r.shiftM[0] > r.shiftM[1] ? r.shiftM[0] : r.shiftM[1];
    float mx = g_maxShiftM.load(std::memory_order_relaxed);
    while (shift > mx && !g_maxShiftM.compare_exchange_weak(mx, shift, std::memory_order_relaxed))
    {
    }
    return true;
}

// The FK-stage pass of FkHook.cpp (ApplyMotionMatching, right after the vault torso pass, before the owner check). Like the vault torso pass it
// does NOT ask whether MM is active: the vault is the game's pose at MM weight 0.
inline bool FkPass(uint8_t* aBones, uint32_t aCount, void* aRig, int64_t aNowTicks, int64_t aTps, bool aPerspectiveAllows)
{
#if UTPP_VAULTSHOULDER_MUTATION == 5
    if (!g_mmActiveMirror.load(std::memory_order_relaxed))
        return false; // mutation 5: the pass needs MM to be active
#endif
    return aPerspectiveAllows && Apply(aBones, aCount, aRig, aNowTicks, aTps);
}
} // namespace rt
} // namespace utpp::mm::vaultshoulder

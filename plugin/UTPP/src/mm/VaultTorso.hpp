#pragma once
// Vault torso limiter (2026-11-10, hood slide with a weapon in hand). Header-only, no RED4ext / Windows types; unit-tested offline
// (tests/VaultTorsoTest.cpp, real captured poses).
//
// PROBLEM (recording 2026-11-10 13:00): V slides over a car hood (PSM Vault 9). The vault over a vehicle is not
// played by v2 (MotionTraversal.hpp Config::vehicleVaultToGame, Unsupported(vehicle)); the v1 gate releases the body to the game's own vault
// animation (release reason travRelease), MM weight 0. With a ranged weapon in hand the game's upper-body weapon layer overwrites the
// spine compensation of that clip: Spine .. Spine3 follow the pelvis rigidly, and on the walking-speed vault the chest hangs 30 deg
// BELOW the horizontal (Spine1 -> Spine3 elevation -29 .. -37 deg, Hips -> Spine3 -28 .. -33 deg). Without a weapon the same clip keeps
// the chest at +50 deg (Hips -> Spine3 +38 deg). The pose is the game's: no MM / v2 layer is involved, so nothing in the overlay,
// the aim offset or the grip passes can be switched off. The only place that sees this pose is the FK detour.
//
// FIX: while the gate is on (a released vault over a vehicle with a ranged weapon in hand) the FK detour limits the elevation of the CHEST
// vector Spine1 -> Spine3 from below (soft floor kFloorDeg = +15 deg, knee kKneeDeg, asymptote +5 deg: a chest that already stays above the
// floor - the run and standing rifle vaults of the recording - is not touched) by rotating Spine, Spine1, Spine2, Spine3 about their own
// joints:
//   - the controlled quantity is the ELEVATION of the vector above the horizontal plane (not a pitch in the model's Y-Z plane): the
//     recorded rifle vault also rolls the chest sideways, where a plane pitch is misleading;
//   - the rotation axis is horizontal and perpendicular to the horizontal heading of that vector (the character's lateral axis of the
//     lean); a vector that is nearly vertical takes the heading of Hips -> Spine1, else model -Y;
//   - Hips, legs, root are never written; only the local rotations of the four spine bones are. Children (shoulders, arms, hands,
//     WeaponLeft / WeaponRight) follow the chest rigidly, so the weapon stays in the hands (no hand IK, no new memory-write path: the
//     records are written through aim::WriteLocalRot like the aim offset and the melee aim tilt do);
//   - slope <= 1 everywhere (the limiter never makes a per-frame step larger than the clip's own), shares kShare over the four joints,
//     total rotation <= kMaxTotalDeg (no spine joint turns by more than 0.30 * kMaxTotalDeg = 21 deg), weight faded 0.15 s in and out (Fade).
// Only the controlled elevation is solved (bisection on the total angle, analytic 3D rotation of the joint positions), then the same
// rotations are written to the records.
//
// WHY NOT THE NO-WEAPON ENVELOPE (+50 deg chest): the same clip without a weapon is a different pose overall - the rifle vault has the
// pelvis rolled 40 deg further (Hips -> Spine1 elevation -27 deg vs +12 deg), and the hips must stay untouched. Lifting the chest to +50 deg
// folds it 75-110 deg onto the pelvis: on skin the arms and the rifle go into the belly and the legs (tried at floors +46 / +35 deg, total
// 180 / 100 deg). The floor above removes the recorded fault (chest hanging below the horizontal)
// with at most 21 deg per joint.
//
// Runtime glue is at the end of this file (namespace rt: inline state shared by the game thread feed and the FK detour).
//   game thread: MMDriver Feed (every feed) -> Wanted(gate inputs) -> rt::Feed -> published weight;
//   FK detour (FkHook.cpp ApplyMotionMatching, after NeckGuard, before the owner check): rt::Apply on the final local records.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "AimOffset.hpp"
#include "GameIntegration.hpp"
#include "MMMath.hpp"

#ifndef UTPP_VAULTTORSO_MUTATION
#define UTPP_VAULTTORSO_MUTATION 0 // test builds: 1 = the pass is compiled out (the build before the fix), 2 = the FK gate needs MM active (first version),
                                   // 3 = no exit hold, 4 = the foot-only condition ignored: utpp_vehexit_vaulttorso_mut1..4 MUST FAIL
#endif

namespace utpp::mm::vaulttorso
{
using aim::Xf;
using math::Quat;
using math::Vec3;

inline constexpr float kDegF = aim::kDegF;
inline constexpr float kFloorDeg = 15.0f;        // elevation of the chest (Spine1 -> Spine3) above which nothing is changed
inline constexpr float kKneeDeg = 10.0f;         // the deficit below the floor saturates at this (tanh knee): asymptote floor - knee
inline constexpr float kFadeSeconds = 0.15f;     // weight fade in / out
inline constexpr float kMaxHoldSeconds = 0.60f;  // the exit hold (below) never lasts longer than this
inline constexpr float kMaxTotalDeg = 70.0f;     // solver range of the total rotation (elevation is monotone in it)
inline constexpr float kMinHorizFrac = 0.15f;    // horizontal part of the vector below this fraction: heading from Hips -> Spine1
inline constexpr float kMinLength = 0.05f;       // m, Spine1 -> Spine3 shorter than this is not a spine
inline constexpr float kStaleSeconds = 0.25f;    // a published weight older than this is not applied (the feed stopped)
inline constexpr float kElevStaleSeconds = 0.10f; // the measured elevation older than this does not hold the exit fade
inline constexpr int kChain = 4;
// Spine, Spine1, Spine2, Spine3 (sum 1). Spine3 takes no share: it does not move the controlled chest vector (Spine1 -> Spine3), it would only
// pitch the shoulders, head, arms and the weapon further than the correction asked for (review round 3). What sits above Spine2 is turned by
// the sum of the shares of Spine, Spine1, Spine2 = the whole T, the chest vector by about 0.87 T: head / arms / weapon follow the chest +-15 %.
inline constexpr float kShare[kChain] = {0.30f, 0.40f, 0.30f, 0.0f};
inline constexpr int kDetailVault = 9;           // gamePSMDetailedLocomotionStates.Vault

// ------------------------------------------------------------------------------------------------ gate

// Where the limiter runs: a vault (PSM detail 9) that the v1 gate released to the game animation (travRelease), over a vehicle that v2
// rejected (Unsupported(vehicle), MotionTraversal), with a ranged weapon in hand (weapon drawn bit of the feed AND a ranged overlay: rifle /
// pistols). Everything else (a mantle / climb 8, a vault v2 plays, no weapon, melee / fists, no release) leaves the pose alone.
struct GateInput
{
    bool travRelease = false;       // v1 release reason kReleaseTrav on this feed
    bool otherRelease = false;      // any other v1 release reason (there is none today; a future one must not be rotated under)
    bool psmReleaseBit = false;     // PSM vehicle / scene / workspot / takedown / carry / vitals / swimming (kPsmReleaseMask) on this feed
    int psmDetail = -1;             // PSM LocomotionDetailed of the feed
    bool weaponDrawn = false;       // PsmWeaponDrawn of the feed
    bool vehicleVaultRanged = false; // motiondriver::VehicleVaultWeapon: last traversal event = vault over a vehicle v2 rejected, overlay is ranged
};
// The body is on foot and animated by the game: no release reason but travRelease, no vehicle / scene / workspot / takedown / carry /
// vitals / swimming bit. Anything else (a mount, a scene, death, a takedown ...) owns the pose and the limiter drops AT ONCE.
inline bool FootOnly(const GateInput& aIn)
{
#if UTPP_VAULTTORSO_MUTATION == 4
    (void)aIn;
    return true; // mutation 4: the foot-only condition is ignored
#else
    return !aIn.otherRelease && !aIn.psmReleaseBit;
#endif
}
inline bool Wanted(const GateInput& aIn)
{
    return FootOnly(aIn) && aIn.travRelease && aIn.psmDetail == kDetailVault && aIn.weaponDrawn && aIn.vehicleVaultRanged;
}
// The vault is over (the PSM left Vault 9) but the game's clip is still recovering: the weight may be held while the same episode goes on
// (aEpisode: rt::Feed latched it while Wanted; the v2 event flag is already gone by then), the weapon is still drawn and the PSM is back on
// plain locomotion (none / Stand / AimWalk / Sprint). A crouch, slide, climb / mantle, ladder, hover jump, knockdown ... ends the hold: the
// limiter never rotates what follows.
inline bool HoldAllowed(const GateInput& aIn, bool aEpisode)
{
    const int d = aIn.psmDetail;
    const bool plain = d == -1 || d == 1 || d == 2 || d == 4;
    return FootOnly(aIn) && aIn.weaponDrawn && aEpisode && plain;
}

// ------------------------------------------------------------------------------------------------ limiter

using aim::Clampf; // same body as aim::Clampf (AimOffset.hpp)

// Soft floor of an elevation (radians): identity at and above the floor, below it the deficit d is compressed to knee * tanh(d / knee).
inline float Limit(float aElev)
{
    const float floorR = kFloorDeg * kDegF, kneeR = kKneeDeg * kDegF;
    if (!(aElev < floorR))
        return aElev;
    const float d = floorR - aElev;
    return floorR - kneeR * std::tanh(d / kneeR);
}

struct Rig
{
    int32_t hips = -1;
    int32_t joint[kChain] = {-1, -1, -1, -1}; // Spine, Spine1, Spine2, Spine3
    uint32_t fkCount = 0;                      // bones [0, fkCount) are FK'd (max index used + 1)
    uint32_t ok = 0;                           // 1 = every check passed (fail closed: 0 -> never applied)
    int16_t parents[aim::kMaxFkBones] = {};
};
static_assert(sizeof(Rig) % 4 == 0);

// Resolves Hips / Spine / Spine1 / Spine2 / Spine3 by name (game CName hash). Fail closed (ok = 0): a name is missing, an index >=
// aim::kMaxFkBones, the parents of [0, fkCount) are not topological, or the chain is not Hips -> ... -> Spine -> Spine1 -> Spine2 ->
// Spine3 (direct children).
inline Rig ResolveRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    Rig r;
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return r;
    static constexpr std::string_view kNames[kChain + 1] = {"Hips", "Spine", "Spine1", "Spine2", "Spine3"};
    int32_t idx[kChain + 1] = {-1, -1, -1, -1, -1};
    int32_t maxIdx = -1;
    for (int k = 0; k <= kChain; ++k)
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
    r.fkCount = static_cast<uint32_t>(maxIdx + 1);
    for (uint32_t i = 0; i < r.fkCount; ++i)
    {
        r.parents[i] = aParents[i];
        if (aParents[i] >= int32_t(i))
            return Rig{};
    }
    r.hips = idx[0];
    for (int k = 0; k < kChain; ++k)
        r.joint[k] = idx[k + 1];
    const bool chain = aim::IsAncestor(r.parents, r.fkCount, r.hips, r.joint[0]) && r.parents[r.joint[1]] == r.joint[0] &&
                       r.parents[r.joint[2]] == r.joint[1] && r.parents[r.joint[3]] == r.joint[2];
    if (!chain)
        return Rig{};
    r.ok = 1;
    return r;
}

struct Result
{
    bool applied = false;     // records were written
    bool saturated = false;   // the target elevation was out of reach inside kMaxTotalDeg
    float elevBefore = 0.0f;  // deg, chest (Spine1 -> Spine3) above the horizontal, before the limiter
    float elevAfter = 0.0f;   // deg, after
    float totalDeg = 0.0f;    // total rotation about the lateral axis (sum over the joints of share * T is T)
    float axis[3] = {0, 0, 0};
};

inline float ElevationOf(Vec3 aD)
{
    return std::atan2(aD.z, std::sqrt(aD.x * aD.x + aD.y * aD.y));
}

// Measures the elevation (deg) of the chest (Spine1 -> Spine3) on the local records; false when the records / rig cannot be solved.
inline bool MeasureElevation(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, float& aElevDeg)
{
    if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount || aRig.fkCount > aim::kMaxFkBones)
        return false;
    Xf model[aim::kMaxFkBones];
    if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
        return false;
    const Vec3 d = model[aRig.joint[3]].p - model[aRig.joint[1]].p;
    if (!math::IsFinite(d))
        return false;
    aElevDeg = ElevationOf(d) / kDegF;
    return true;
}

// Limits the elevation of the chest (Spine1 -> Spine3) on the local records [0, rig.fkCount) of one FK call; aWeight 0..1 scales the correction
// (the faded share of the limiter, so a fade never jumps). False = nothing written (bad rig / records / weight 0). No allocation;
// at most kChain + 1 FK passes over rig.fkCount bones.
inline bool Apply(uint8_t* aBones, uint32_t aCount, const Rig& aRig, float aWeight, Result* aOut = nullptr)
{
    Result res;
    if (aOut)
        *aOut = res;
#if UTPP_VAULTTORSO_MUTATION == 1
    (void)aBones; (void)aCount; (void)aRig; (void)aWeight;
    return false; // mutation 1: the pass is compiled out (the build before)
#else
    if (!aRig.ok || !aBones || aRig.fkCount == 0 || aRig.fkCount > aCount || aRig.fkCount > aim::kMaxFkBones || !std::isfinite(aWeight) ||
        !(aWeight > 0.0f))
        return false;
    const float w = Clampf(aWeight, 0.0f, 1.0f);
    Xf model[aim::kMaxFkBones];
    if (!aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
        return false;
    const Vec3 hips = model[aRig.hips].p;
    const Vec3 d = model[aRig.joint[3]].p - model[aRig.joint[1]].p; // chest: Spine1 -> Spine3
    const float len = aim::Length(d);
    if (!math::IsFinite(d) || !std::isfinite(len) || len < kMinLength)
        return false;
    const float e = ElevationOf(d);
    res.elevBefore = e / kDegF;
    res.elevAfter = res.elevBefore;
    const float want = (Limit(e) - e) * w;
    if (!(want > 1e-6f))
    {
        if (aOut)
            *aOut = res;
        return true; // at or above the floor: the limiter is the identity, nothing to write
    }
    // lateral axis of the lean: horizontal, perpendicular to the horizontal heading of the vector (positive angle raises the elevation)
    const float hl = std::sqrt(d.x * d.x + d.y * d.y);
    Vec3 h{0.0f, -1.0f, 0.0f};
    if (hl >= kMinHorizFrac * len)
        h = Vec3{d.x / hl, d.y / hl, 0.0f};
    else
    {
        const Vec3 lo = model[aRig.joint[1]].p - hips;
        const float ll = std::sqrt(lo.x * lo.x + lo.y * lo.y);
        if (ll > 1e-4f)
            h = Vec3{lo.x / ll, lo.y / ll, 0.0f};
    }
    const Vec3 axis{h.y, -h.x, 0.0f};
    Vec3 p0[kChain];
    for (int k = 0; k < kChain; ++k)
        p0[k] = model[aRig.joint[k]].p;
    auto elevAt = [&](float aTotal) {
        Vec3 p[kChain] = {p0[0], p0[1], p0[2], p0[3]};
        for (int k = 0; k < kChain; ++k)
        {
            const Quat q = aim::AxisAngle(axis, kShare[k] * aTotal);
            for (int m = k + 1; m < kChain; ++m)
                p[m] = p[k] + math::QuatRotate(q, p[m] - p[k]);
        }
        return ElevationOf(p[kChain - 1] - p[1]);
    };
    const float target = e + want;
    float lo = 0.0f, hi = kMaxTotalDeg * kDegF, total = hi;
    if (elevAt(hi) < target)
        res.saturated = true;
    else
    {
        for (int it = 0; it < 40; ++it)
        {
            const float mid = 0.5f * (lo + hi);
            (elevAt(mid) < target ? lo : hi) = mid;
        }
        total = 0.5f * (lo + hi);
    }
    for (int k = 0; k < kChain; ++k)
    {
        if (!(kShare[k] > 0.0f))
            continue; // Spine3: no share, its record is not touched
        if (k > 0 && !aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
            return false; // the parent of this joint moved with the previous one (records already written: fault only)
        aim::RotateAboutJoint(aBones, model, aRig.parents, uint32_t(aRig.joint[k]), aim::AxisAngle(axis, kShare[k] * total));
    }
    res.applied = true;
    res.totalDeg = total / kDegF;
    res.axis[0] = axis.x;
    res.axis[1] = axis.y;
    res.axis[2] = axis.z;
    if (aim::ComputeModel(aBones, aCount, aRig.parents, aRig.fkCount, model))
        res.elevAfter = ElevationOf(model[aRig.joint[3]].p - model[aRig.joint[1]].p) / kDegF;
    if (aOut)
        *aOut = res;
    return true;
#endif
}

// ------------------------------------------------------------------------------------------------ fade

// Weight of the limiter: 0 -> 1 in kFadeSeconds while the gate is on, 1 -> 0 in kFadeSeconds after it ends (smoothstep of a linear ramp).
// aHold (the gate ended but the pose the game still shows is below the floor): the weight stays, at most kMaxHoldSeconds. The game's vault
// clip is still recovering when the PSM leaves Vault 9 (13:03:32: release off at +0.73 s, the chest still below the horizontal at +0.72 s),
// so fading while the correction is needed would let the chest dip; above the floor the limiter is the identity, so holding costs nothing.
class Fade
{
public:
    void Reset() { m_lin = 0.0f; m_hold = 0.0f; }
    float Linear() const { return m_lin; }
    float Weight() const { return m_lin * m_lin * (3.0f - 2.0f * m_lin); }
    float Step(float aDt, bool aWanted, bool aHold)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > 0.1f ? 0.1f : aDt) : 0.0f;
        const float step = dt / kFadeSeconds;
        if (aWanted)
        {
            m_hold = 0.0f;
            m_lin = Clampf(m_lin + step, 0.0f, 1.0f);
        }
        else if (aHold && m_lin > 0.0f && m_hold < kMaxHoldSeconds)
            m_hold += dt;
        else
            m_lin = Clampf(m_lin - step, 0.0f, 1.0f);
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
inline std::atomic<bool> g_rigOk{false};
// published by the game thread (rt::Feed), read by the FK detour
inline std::atomic<float> g_weight{0.0f};
inline std::atomic<int64_t> g_pubTicks{0};
inline std::atomic<bool> g_foot{false}; // the foot-only condition of the last feed
// measured by the FK detour (chest elevation before the limiter), read by the game thread for the exit hold
inline std::atomic<float> g_elevDeg{90.0f};
inline std::atomic<int64_t> g_elevTicks{0};
// diagnostics
inline std::atomic<uint64_t> g_calls{0}, g_applied{0}, g_saturated{0}, g_episodes{0};
inline std::atomic<float> g_maxCorrDeg{0.0f}, g_minElevDeg{90.0f};
#if UTPP_VAULTTORSO_MUTATION == 2
inline std::atomic<bool> g_mmActiveMirror{true}; // mutation 2 / tests: the "MM is active" flag the FK gate used to require (false at MM weight 0)
#endif
// game thread only (MMDriver Feed and OnDisarmed, both under the v1 mutex)
inline Fade g_fade;
inline bool g_on = false; // an episode (weight > 0) is running
inline bool g_episode = false; // latched while Wanted, cleared when the weight is 0 or the foot-only condition breaks (the exit hold needs it)

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
    g_rigOk.store(m.rig.ok != 0, std::memory_order_release);
}
// Game thread, from motionhost::OnDisarmed (under the v1 mutex, like Feed): forgets the rig and every state of the episode.
inline void OnDisarmed()
{
    RigMsg m;
    g_rig.Publish(m);
    g_rigOk.store(false, std::memory_order_release);
    g_fade.Reset();
    g_on = false;
    g_episode = false;
    g_weight.store(0.0f, std::memory_order_relaxed);
    g_foot.store(false, std::memory_order_relaxed);
    g_elevDeg.store(90.0f, std::memory_order_relaxed);
    g_elevTicks.store(0, std::memory_order_relaxed);
}

// Game thread, once per feed: steps the fade and publishes the weight. Returns +1 on the first feed of an episode (weight rises from 0),
// -1 on the feed that ends it (weight back to 0), else 0 (the caller logs the edges).
// - Not foot-only (a mount, a scene, death ...): the weight drops to 0 AT ONCE (no fade, nothing is rotated under another owner).
// - Wanted: the weight rises in 0.15 s. Not wanted: it falls in 0.15 s, unless HoldAllowed and the FK side measured the chest below the
//   floor within the last 0.1 s (the clip is still recovering): then it is held, at most 0.6 s.
inline int Feed(const GateInput& aGate, float aDt, int64_t aNowTicks, int64_t aTps)
{
    const bool foot = FootOnly(aGate);
    float w = 0.0f;
    if (!foot)
    {
        g_fade.Reset();
        g_episode = false;
    }
    else
    {
        const bool wanted = Wanted(aGate);
        if (wanted)
            g_episode = true;
        const bool measured = Seconds(aNowTicks, g_elevTicks.load(std::memory_order_relaxed), aTps) <= kElevStaleSeconds;
#if UTPP_VAULTTORSO_MUTATION == 3
        const bool hold = false; // mutation 3: no exit hold (the weight fades while the clip is still below the floor)
#else
        const bool hold = !wanted && HoldAllowed(aGate, g_episode) && measured && g_elevDeg.load(std::memory_order_relaxed) < kFloorDeg;
#endif
        w = g_fade.Step(aDt, wanted, hold);
        if (!(w > 0.0f))
            g_episode = false;
    }
    g_foot.store(foot, std::memory_order_relaxed);
    g_weight.store(w, std::memory_order_relaxed);
    g_pubTicks.store(aNowTicks, std::memory_order_relaxed);
    int edge = 0;
    if (!g_on && w > 0.0f)
    {
        g_on = true;
        edge = 1;
        g_episodes.fetch_add(1, std::memory_order_relaxed);
        g_maxCorrDeg.store(0.0f, std::memory_order_relaxed);
        g_minElevDeg.store(90.0f, std::memory_order_relaxed);
    }
    else if (g_on && !(w > 0.0f))
    {
        g_on = false;
        edge = -1;
    }
    return edge;
}

// FK detour, cheap gate: one relaxed load while idle. The published weight must be fresh and the foot-only condition must hold.
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
    Result r;
    float elev = 0.0f;
    if (!MeasureElevation(aBones, aCount, m.rig, elev))
        return false;
    g_elevDeg.store(elev, std::memory_order_relaxed);
    g_elevTicks.store(aNowTicks, std::memory_order_relaxed);
    float cur = g_minElevDeg.load(std::memory_order_relaxed);
    while (elev < cur && !g_minElevDeg.compare_exchange_weak(cur, elev, std::memory_order_relaxed))
    {
    }
    if (!vaulttorso::Apply(aBones, aCount, m.rig, g_weight.load(std::memory_order_relaxed), &r) || !r.applied)
        return false;
    g_applied.fetch_add(1, std::memory_order_relaxed);
    if (r.saturated)
        g_saturated.fetch_add(1, std::memory_order_relaxed);
    const float corr = r.elevAfter - r.elevBefore;
    float mx = g_maxCorrDeg.load(std::memory_order_relaxed);
    while (corr > mx && !g_maxCorrDeg.compare_exchange_weak(mx, corr, std::memory_order_relaxed))
    {
    }
    return true;
}

// The whole FK-stage pass of FkHook.cpp (ApplyMotionMatching, after NeckGuard, before the owner check): the published weight, the perspective
// permission (TPP only) and the armed rig. It does NOT ask whether MM is active - the vault is the game's own pose and MM weight is 0 there
// (the gate hard-outs it within ~0.12 s of the release), like the seated neck cut.
inline bool FkPass(uint8_t* aBones, uint32_t aCount, void* aRig, int64_t aNowTicks, int64_t aTps, bool aPerspectiveAllows)
{
#if UTPP_VAULTTORSO_MUTATION == 2
    if (!g_mmActiveMirror.load(std::memory_order_relaxed))
        return false; // mutation 2: the pass needs MM to be active (the gate of the first version)
#endif
    return aPerspectiveAllows && Apply(aBones, aCount, aRig, aNowTicks, aTps);
}
} // namespace rt
} // namespace utpp::mm::vaulttorso

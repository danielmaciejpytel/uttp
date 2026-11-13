// UTPP Phase 4b - game integration helpers for motion matching. Header-only, no RED4ext dependency, so every piece is
// unit-tested offline (tests/MMIntegrationTest.cpp).
//
// Contents:
//   - yaw convention + world -> character velocity conversion;
//   - gating (reason bitmask) and the smoothstep weight follower;
//   - bone map: database bone -> game metaRig index, by NAME hash, with parent consistency check (fail closed);
//   - PoseSnapshot + SeqlockBuffer: single-writer (game thread) / multi-reader (animation threads) lock-free publish;
//   - BlendIntoRecords: blend of the published MM local pose into the game's 48-byte bone records (FK input);
//   - 0.6.2 upper-body bone mask (kMaskUpperFromGame): per-bone weight, mask states (melee / aim / vehicle exit window),
//     mask decision and the 0.15 s mask fade.
//
// YAW CONVENTION (game: Z up, forward +Y, right +X):
//   yaw psi (degrees at the native boundary, radians inside) is the counter-clockwise angle about +Z from world +Y to
//   the character forward, i.e. forward = (-sin psi, cos psi), right = (cos psi, sin psi), psi = atan2(-fwd.x, fwd.y).
//   This is the same convention as the database Trajectory frame (BaseTrajectory.hpp: direction (-sin d, cos d)).
//   Character-frame velocity: x = dot(v, right) = vx cos psi + vy sin psi, y = dot(v, fwd) = -vx sin psi + vy cos psi.
#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string_view>
#include <type_traits>

#include "../MutationSelect.hpp"

namespace utpp::mm::integ
{
inline constexpr float kPiF = 3.14159265358979323846f;
inline constexpr float kDegToRad = kPiF / 180.0f;
inline constexpr uint32_t kMaxMMBones = 80; // database bones carried in a snapshot (current db: 73)
inline constexpr uint32_t kGameBoneStride = 48;
inline constexpr uint32_t kGameRotOffsetFloats = 4; // quaternion xyzw at +0x10

// ------------------------------------------------------------------------------------------------ yaw / frames

// Yaw (radians, convention above) of a planar forward vector. Returns 0 for a degenerate vector.
inline float YawFromForward(float aFx, float aFy)
{
    if (!std::isfinite(aFx) || !std::isfinite(aFy) || (aFx * aFx + aFy * aFy) < 1e-12f)
        return 0.0f;
    return std::atan2(-aFx, aFy);
}

inline void ForwardFromYaw(float aYaw, float& aFx, float& aFy)
{
    aFx = -std::sin(aYaw);
    aFy = std::cos(aYaw);
}

// World planar velocity -> character frame (x right, y forward) for a character yaw in radians.
inline void WorldToCharacter(float aVx, float aVy, float aYaw, float& aOutX, float& aOutY)
{
    const float c = std::cos(aYaw);
    const float s = std::sin(aYaw);
    aOutX = c * aVx + s * aVy;
    aOutY = -s * aVx + c * aVy;
}

inline float WrapPi(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::remainder(a, 2.0f * kPiF);
    return a <= -kPiF ? a + 2.0f * kPiF : a;
}

// ------------------------------------------------------------------------------------------------ gating

enum GateReason : uint32_t
{
    kGateMMDisabled = 1u << 0,   // UTPPSetMMEnabled(false) (default)
    kGateDbNotLoaded = 1u << 1,  // database missing / invalid
    kGateNotMapped = 1u << 2,    // not armed or the bone map failed
    kGateTppInactive = 1u << 3,  // TPP not active
    kGateLocoState = 1u << 4,    // locomotion state not in the allowed mask
    kGateMatcherNotReady = 1u << 5,
    kGateFault = 1u << 6,        // FK detour fault latched
    kGateStaleFeed = 1u << 7,    // no UTPPFeedLocomotion for > stale time (evaluated by the per-frame tick)
    kGatePsmBlock = 1u << 8,     // 0.3.2: PSM blackboard says the game owns the body (vehicle, scene, workspot, swim...)
    // 0.5.8 (driver bit, UTPPSetMMSpeedCap): capsule speed above the supported range (> factor x game sprint speed: dash /
    // dodge bursts of 10-17.6 m/s in game 0.5.7). Not soft (no debounce), normal fade, reopen hold after it clears.
    kGateOverSpeed = 1u << 9,
    // 0.6.0 (driver bit, UTPPSetMMMeleeGate): a whole-body melee action (PsmBlock kPsmMeleeAttack / kPsmMeleePose from
    // PSM.MeleeWeapon / PSM.RangedWeapon QuickMelee), held for MeleeGateConfig::releaseHold after the state ends (MeleeGate).
    // Debounced inside MeleeGate (like a crouch blip: the bits must persist GateDebounceConfig::holdSeconds - a debounce after
    // the release hold would let a single-sample state close the gate for the whole hold), then immediate; normal fade
    // (blendTime), no extra reopen hold (the release hold is the hysteresis).
    kGateMelee = 1u << 10,
};

// 0.3.2: PSM block reasons computed in scripts/utpp.reds (UTPPPsmBlockMask) and passed with every feed. Any non-zero
// value closes the gate (kGatePsmBlock); the bits are only kept for the status/trace.
enum PsmBlock : uint32_t
{
    kPsmVehicle = 1u << 0,       // PSM.Vehicle != Default or MountedToVehicle
    kPsmScene = 1u << 1,         // PSM.HighLevel SceneTier3..5 or an unknown HighLevel value
    kPsmSwimming = 1u << 2,      // PSM.HighLevel Swimming or PSM.Swimming != Default
    kPsmWorkspot = 1u << 3,      // PSM.IsInWorkspot != 0
    kPsmTakedown = 1u << 4,      // PSM.Takedown != Default (grapple, leap, takedown)
    kPsmCarry = 1u << 5,         // PSM.BodyCarrying != Default or PSM.Carrying
    kPsmDetailed = 1u << 6,      // PSM.LocomotionDetailed not Stand/Crouch/Sprint (ladder, jump, fall, land, dodge...)
    kPsmVitals = 1u << 7,        // PSM.Vitals != Alive (dead / resurrecting)
    kPsmPhotoOrMenu = 1u << 8,   // photo mode active or UI_System.IsInMenu
    kPsmNoBlackboard = 1u << 9,  // PSM blackboard unavailable (fail closed)
    kPsmReaction = 1u << 10,     // PSM.Reaction != Default (stagger)
    // 0.6.0 melee bits (scripts/utpp.reds 0.6.0). Not PSM gate reasons: the driver strips them (PsmBlockBits) and turns them
    // into kGateMelee through MeleeGate (UTPPSetMMMeleeGate; off = ignored = 0.5.9).
    kPsmMeleeAttack = 1u << 11,  // PSM.MeleeWeapon ComboAttack..EquipAttack (11..21) or PSM.RangedWeapon QuickMelee (3)
    kPsmMeleePose = 1u << 12,    // PSM.MeleeWeapon Parried, Hold, ChargedHold, Block, Targeting, Deflect (5..10)
};
inline constexpr uint32_t kPsmMeleeMask = kPsmMeleeAttack | kPsmMeleePose;

// Closures that hand the body to a game animation which must be visible at once: faster fade-out.
inline constexpr uint32_t kHardGateMask = kGatePsmBlock | kGateTppInactive | kGateStaleFeed | kGateFault | kGateNotMapped;
// Closures driven by the game state (PSM block, unsupported locomotion state): after they clear, the gate must stay
// open for reopenHold before MM fades back in (vanilla exit/landing/dash recovery poses finish first).
inline constexpr uint32_t kReopenHoldMask = kGatePsmBlock | kGateLocoState | kGateStaleFeed | kGateOverSpeed;

// Locomotion state ids (mapping defined in scripts/utpp.reds UTPPLocoStateId).
enum LocoState : int32_t
{
    kLocoOther = 0,
    kLocoStand = 1,
    kLocoCrouch = 2,
    kLocoSprint = 3,
};
inline constexpr uint32_t kDefaultAllowedLocoMask = (1u << kLocoStand) | (1u << kLocoCrouch) | (1u << kLocoSprint);

struct GateInput
{
    bool mmEnabled = false;
    bool dbLoaded = false;
    bool mapped = false;
    bool tppActive = false;
    int32_t locoState = 0;
    uint32_t allowedLocoMask = kDefaultAllowedLocoMask;
    bool matcherReady = false;
    bool fault = false;
    bool staleFeed = false;
    uint32_t psmBlock = 0; // 0.3.2: PsmBlock bits, non-zero closes the gate
};

inline uint32_t ComputeGate(const GateInput& g)
{
    uint32_t r = 0;
    if (!g.mmEnabled)
        r |= kGateMMDisabled;
    if (!g.dbLoaded)
        r |= kGateDbNotLoaded;
    if (!g.mapped)
        r |= kGateNotMapped;
    if (!g.tppActive)
        r |= kGateTppInactive;
    if (g.locoState < 0 || g.locoState > 31 || ((g.allowedLocoMask >> g.locoState) & 1u) == 0)
        r |= kGateLocoState;
    if (!g.matcherReady)
        r |= kGateMatcherNotReady;
    if (g.fault)
        r |= kGateFault;
    if (g.staleFeed)
        r |= kGateStaleFeed;
    if (g.psmBlock != 0)
        r |= kGatePsmBlock;
    return r;
}

// Linear progress towards the target over blendTime, output weight = smoothstep(progress).
class WeightFollower
{
public:
    void Reset() { m_progress = 0.0f; }
    void Set(float aProgress) { m_progress = aProgress >= 1.0f ? 1.0f : (aProgress > 0.0f ? aProgress : 0.0f); } // 0.6.2
    // Returns the new weight in [0, 1]. aBlendTime <= 0 snaps.
    float Update(float aTarget, float aDt, float aBlendTime) { return Update(aTarget, aDt, aBlendTime, aBlendTime); }
    // 0.3.2: separate fade-in / fade-out times (<= 0 or non-finite snaps in that direction).
    float Update(float aTarget, float aDt, float aInTime, float aOutTime)
    {
        const float target = aTarget >= 0.5f ? 1.0f : 0.0f;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        const float t = (m_progress < target) ? aInTime : aOutTime;
        if (!(t > 0.0f) || !std::isfinite(t))
            m_progress = target;
        else if (m_progress < target)
            m_progress = std::fmin(target, m_progress + dt / t);
        else if (m_progress > target)
            m_progress = std::fmax(target, m_progress - dt / t);
        return Weight();
    }
    float Progress() const { return m_progress; }
    float Weight() const { return m_progress * m_progress * (3.0f - 2.0f * m_progress); }

private:
    float m_progress = 0.0f;
};

// 0.3.2 gate -> weight: smoothstep crossfade between the MM pose and the live game pose (the vanilla graph keeps running
// underneath, so fading out hands over to whatever the game animates at that moment).
//   - fade-in: blendTime; fade-out: blendTime, or hardOut when the gate contains a kHardGateMask reason;
//   - reopen hold: after a closure containing a kReopenHoldMask reason the gate must stay open for reopenHold seconds
//     before the target goes back to 1 (an open gate during the hold keeps fading out / stays at 0).
struct FadeTimes
{
    float blendTime = 0.25f;
    float hardOut = 0.12f;
    float reopenHold = 0.10f;
};

class GateFader
{
public:
    void Reset()
    {
        m_weight.Reset();
        m_openFor = 0.0f;
        m_holdPending = false;
        m_hard = false;
    }
    // 0.6.0 (UTPPSetMMMountCut): weight to 0 at once (no fade-out), closed with the reopen hold pending - the next Update with
    // a closed gate keeps 0, the reopen follows the normal hold + fade-in.
    void Cut()
    {
        m_weight.Reset();
        m_openFor = 0.0f;
        m_target = 0.0f;
        m_holdPending = true;
        m_hard = true;
    }
    // 0.6.2 (vehicle exit window with the upper-body mask): drop a pending reopen hold - the lower body may fade in at once.
    void ClearHold()
    {
        m_holdPending = false;
        m_hard = false;
    }
    // 0.6.2 (hold-last-pose database switch): full weight at once, open (the new database continues the held body).
    void Fill()
    {
        m_weight.Set(1.0f);
        m_target = 1.0f;
        m_openFor = 0.0f;
        m_holdPending = false;
        m_hard = false;
    }
    // Returns the new weight; target() is 1 only when the gate is open and any pending hold has elapsed.
    float Update(uint32_t aGate, float aDt, const FadeTimes& aT)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        if (aGate != 0)
        {
            m_openFor = 0.0f;
            if (aGate & kReopenHoldMask)
                m_holdPending = true;
            m_target = 0.0f;
            // a hard closure keeps the fast fade-out until MM is allowed back (reason may turn soft mid-fade)
            m_hard = m_hard || (aGate & kHardGateMask) != 0;
        }
        else
        {
            m_openFor += dt;
            if (m_holdPending && m_openFor >= aT.reopenHold)
                m_holdPending = false;
            m_target = m_holdPending ? 0.0f : 1.0f;
            if (m_target > 0.0f)
                m_hard = false;
        }
        const float out = m_hard ? aT.hardOut : aT.blendTime;
        return m_weight.Update(m_target, dt, aT.blendTime, out);
    }
    float Weight() const { return m_weight.Weight(); }
    float Progress() const { return m_weight.Progress(); }
    float Target() const { return m_target; }
    bool HoldPending() const { return m_holdPending; }
    bool HardOut() const { return m_hard; }

private:
    WeightFollower m_weight;
    float m_openFor = 0.0f;
    float m_target = 0.0f;
    bool m_holdPending = false;
    bool m_hard = false;
};

// 0.5.6: PsmBlock diagnostic detail. scripts/utpp.reds 0.5.6 puts PSM.LocomotionDetailed + 1 into bits 16..23 when
// kPsmDetailed is set (only then, so an older DLL still sees a non-zero mask exactly when it did before). The native
// side strips these bits before the gate (PsmBlockBits) and keeps them for the trace (PsmDetailOf: -1 = not reported).
inline constexpr uint32_t kPsmDetailShift = 16;
inline constexpr uint32_t kPsmDetailFieldMask = 0xFFu << kPsmDetailShift;
// 0.6.0 diagnostic fields (always reported by reds 0.6.0, never gate reasons): PSM.UpperBody + 1 in bits 13..15,
// PSM.MeleeWeapon + 1 in bits 24..28 (0 = not reported).
inline constexpr uint32_t kPsmUpperShift = 13;
inline constexpr uint32_t kPsmUpperFieldMask = 0x7u << kPsmUpperShift;
inline constexpr uint32_t kPsmMeleeStateShift = 24;
inline constexpr uint32_t kPsmMeleeStateFieldMask = 0x1Fu << kPsmMeleeStateShift;
// WeaponRight read result: known bit 29, drawn bit 30; never a gate reason. Missing known = older/unavailable feed.
inline constexpr uint32_t kPsmWeaponKnown = 1u << 29;
inline constexpr uint32_t kPsmWeaponDrawn = 1u << 30;
// Dual wield flag (dual-wield D0, diagnostic only, never a gate reason): bit 31 (the Int32 sign bit on the reds side) is set by
// scripts/utpp.reds UTPPPsmBlockMask exactly when its local `dualWield` is true (PSM.UpperBody DualWield 7 or PSM.IsDualWielding).
// The 3-bit UpperBody field is NOT widened: UpperBody 7 does not fit it (7 + 1 = 8) and is still left unreported there (field 0),
// so PsmUpperBodyOf keeps returning -1 for it and every existing consumer behaves exactly as before. Field 0 together with the
// flag means UpperBody 7 (inferred: no other UpperBody value is left unreported); PsmUpperBodyRawOf reports that for diagnostics.
inline constexpr uint32_t kPsmDualFlag = 1u << 31;
inline constexpr int kPsmUpperBodyDualWield = 7; // gamePSMUpperBodyStates.DualWield (game 2.5)
#ifdef UTPP_DUALWIELD_MUTATION
// Mutation check only (tests/DualWieldPsmTest.cpp): 1 = the flag is no longer stripped from the gate bits.
inline constexpr uint32_t kPsmDualStripped = (UTPP_DUALWIELD_MUTATION == 1) ? 0u : kPsmDualFlag;
#else
inline constexpr uint32_t kPsmDualStripped = kPsmDualFlag;
#endif
inline constexpr uint32_t kPsmDiagFieldMask = kPsmDetailFieldMask | kPsmUpperFieldMask | kPsmMeleeStateFieldMask |
                                           kPsmWeaponKnown | kPsmWeaponDrawn | kPsmDualStripped;
inline bool PsmWeaponKnown(uint32_t aMask) { return (aMask & kPsmWeaponKnown) != 0; }
inline bool PsmWeaponDrawn(uint32_t aMask) { return PsmWeaponKnown(aMask) && (aMask & kPsmWeaponDrawn) != 0; }
// PSM gate reasons only (diagnostic fields and the 0.6.0 melee bits stripped)
inline uint32_t PsmBlockBits(uint32_t aMask) { return aMask & ~(kPsmDiagFieldMask | kPsmMeleeMask); }
inline int PsmDetailOf(uint32_t aMask) { return int((aMask & kPsmDetailFieldMask) >> kPsmDetailShift) - 1; }
inline uint32_t PsmMeleeBits(uint32_t aMask) { return aMask & kPsmMeleeMask; }
inline int PsmMeleeStateOf(uint32_t aMask) { return int((aMask & kPsmMeleeStateFieldMask) >> kPsmMeleeStateShift) - 1; }
// Legacy 3-bit view, used by every gameplay consumer (aim, strafe, aim offset, driver trace). It deliberately ignores
// kPsmDualFlag: UpperBody 7 (DualWield) stays -1 here, as it was before the flag existed (dual-wield D0 behaviour identity).
#ifdef UTPP_DUALWIELD_MUTATION
inline int PsmUpperBodyOf(uint32_t aMask)
{
    if constexpr (UTPP_DUALWIELD_MUTATION == 2)
    {
        if ((aMask & kPsmDualFlag) != 0 && (aMask & kPsmUpperFieldMask) == 0)
            return kPsmUpperBodyDualWield; // mutation 2: consumers would see the new value 7
    }
    return int((aMask & kPsmUpperFieldMask) >> kPsmUpperShift) - 1;
}
#else
inline int PsmUpperBodyOf(uint32_t aMask) { return int((aMask & kPsmUpperFieldMask) >> kPsmUpperShift) - 1; }
#endif
inline bool PsmDualOf(uint32_t aMask) { return (aMask & kPsmDualFlag) != 0; }
// Diagnostics only (status string, logs): UpperBody including DualWield 7. -1 = not reported.
inline int PsmUpperBodyRawOf(uint32_t aMask)
{
    const int field = int((aMask & kPsmUpperFieldMask) >> kPsmUpperShift) - 1;
    if (field >= 0)
        return field;
    return PsmDualOf(aMask) ? kPsmUpperBodyDualWield : -1;
}
// " dualWield=<0|1> upperBodyRaw=<-1..7>" appended to Game.UTPPGetMotionStatus(). Keys are letters only (tools/gametest/gt_log.py
// FIELD_RE) and unique in the status line. dualWield = PsmDualOf (UpperBody 7 || PSM.IsDualWielding as read by UTPPPsmBlockMask).
inline int PsmDualStatusSuffix(char* aBuf, size_t aSize, uint32_t aMask)
{
    return std::snprintf(aBuf, aSize, " dualWield=%d upperBodyRaw=%d", PsmDualOf(aMask) ? 1 : 0, PsmUpperBodyRawOf(aMask));
}

// 0.6.0 melee gate. In game (0.5.9, play session 2026-10-07 11:50:10-11:51:30, fists equipped, attacking) the gate never
// closed: UTPPPsmBlockMask had no melee input and LocomotionDetailed stayed Stand, so MM (w = 1.00, locomotion clips)
// replaced the attack animation. Melee bits from the feed for the debounce time (first sample counts 0) -> active; after they clear, active for releaseHold more seconds
// (attack chains / recovery poses must not flash MM back in between hits).
// Release of the melee mask (2026-11-12, fists review C1 / C2: the hands dropped from the guard in 3-4 frames ~0.3 s after the last
// strike). After a state of a real melee WEAPON (PSM.MeleeWeapon >= Idle 2: fists, knife, katana, blunt...) the mask is held 0.55 s
// and then eased out over 0.35 s (smoothstep); the fade IN stays kMaskFadeSeconds. A gun bash (PSM.RangedWeapon QuickMelee: the
// MeleeWeapon state is NotReady 0 / not reported) keeps the previous 0.3 s hold and the plain fade, the gun upper body must come back.
inline constexpr int kPsmMeleeWeaponIdle = 2;                 // gamePSMMeleeWeapon Idle: states >= this belong to a melee weapon
inline constexpr float kMeleeReleaseHoldSeconds = 0.55f;      // s the melee reason is held after the melee bits clear (melee weapon)
inline constexpr float kMeleeReleaseHoldShortSeconds = 0.3f;  // s for a gun bash / state not reported (previous behaviour)
inline constexpr float kMeleeReleaseFadeOutSeconds = 0.35f;   // s of the eased upper-body fade out after a melee weapon release

struct MeleeGateConfig
{
    bool enabled = true;
    float releaseHold = kMeleeReleaseHoldSeconds;           // melee weapon states
    float releaseHoldShort = kMeleeReleaseHoldShortSeconds; // gun bash (aWeaponMelee false)
};

class MeleeGate
{
public:
    void Reset()
    {
        m_active = false;
        m_since = 0.0f;
        m_pending = -1.0f;
        m_longRelease = false;
    }
    // aMeleeBits = PsmMeleeBits of the feed. Returns true while the melee action closes the gate. aWeaponMelee = the feed's
    // PSM.MeleeWeapon state belongs to a melee weapon (>= kPsmMeleeWeaponIdle); the value of the last feed with melee bits picks
    // the release hold (releaseHold, else releaseHoldShort: gun bash / state not reported).
    bool Update(uint32_t aMeleeBits, float aDt, const MeleeGateConfig& aCfg, float aDebounce, bool aWeaponMelee = false)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        if (!aCfg.enabled)
        {
            Reset();
            return false;
        }
        if (aMeleeBits != 0)
        {
            ++m_feeds;
            m_longRelease = aWeaponMelee;
            if (!m_active)
            {
                m_pending = m_pending < 0.0f ? 0.0f : m_pending + dt;
                if (aDebounce > 0.0f && m_pending < aDebounce)
                    return false;
                m_pending = -1.0f;
                ++m_episodes;
            }
            m_active = true;
            m_since = 0.0f;
            return true;
        }
        if (m_pending >= 0.0f)
            ++m_suppressed;
        m_pending = -1.0f;
        if (m_active)
        {
            m_since += dt;
            if (m_since >= (m_longRelease ? aCfg.releaseHold : aCfg.releaseHoldShort))
                m_active = false;
        }
        return m_active;
    }
    bool LongRelease() const { return m_longRelease; } // the last melee bits came with a melee weapon state (long hold + eased fade)
    bool Active() const { return m_active; }
    float Since() const { return m_since; }       // s since the melee bits cleared (while active)
    uint64_t Episodes() const { return m_episodes; }
    uint64_t Feeds() const { return m_feeds; }
    uint64_t Suppressed() const { return m_suppressed; } // melee states shorter than the debounce

private:
    bool m_active = false;
    bool m_longRelease = false;
    float m_since = 0.0f;
    float m_pending = -1.0f;
    uint64_t m_suppressed = 0;
    uint64_t m_episodes = 0;
    uint64_t m_feeds = 0;
};

// 0.5.6 gate debounce. In game (0.5.5, a former test database, 2026-10-07 02:52:17-02:52:19.7 and 02:54:15.1-02:54:17.2,
// both right after a crouch slide) the feed reported for ONE sample every ~0.107 s: locoState 0 (state name not stand /
// crouch / sprint) + PsmBlock kPsmDetailed, the next sample (6 ms later) crouch again. Every blip closed the gate and
// armed the 0.10 s reopen hold, so MM never faded back in (w = 0.00 for 2 s of crouch walking). A soft closure - only
// kGateLocoState and/or kGatePsmBlock from kPsmDetailed alone, no vertical motion - must now persist for holdSeconds
// before it closes an open gate. Every other reason (vehicle, scene, swim, workspot, takedown, carry, vitals, menu,
// reaction, no blackboard, TPP off, stale feed, fault, map, database) and any soft reason with |vz| >= maxVertical
// (jump take-off, fall) close at once, as before. Once closed, a soft gate stays closed until the raw gate opens; the
// reopen (fade-in after the reopen hold) is unchanged. holdSeconds <= 0 = off (0.5.5).
struct GateDebounceConfig
{
    float holdSeconds = 0.06f; // > one frame at 30 fps (0.033 s); a real slide / dodge lasts >= 0.3 s
    float maxVertical = 1.0f;  // m/s |world vz|: at or above, a soft closure is immediate (jump, fall)
};

class GateDebounce
{
public:
    void Reset()
    {
        m_pending = -1.0f;
        m_out = 0;
    }
    // aRaw = ComputeGate(...) (+ driver bits); aPsm = PsmBlockBits of the feed; aVz = world vertical velocity (m/s).
    // Returns the gate to use (0 while a soft closure is being debounced).
    uint32_t Update(uint32_t aRaw, uint32_t aPsm, float aVz, float aDt, const GateDebounceConfig& aCfg)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        if (aRaw == 0 || !(aCfg.holdSeconds > 0.0f))
        {
            m_pending = -1.0f;
            m_out = aRaw;
            return m_out;
        }
        const bool soft = (aRaw & ~(kGateLocoState | kGatePsmBlock)) == 0 && (aPsm & ~uint32_t(kPsmDetailed)) == 0 &&
                          std::isfinite(aVz) && std::fabs(aVz) < aCfg.maxVertical;
        if (!soft || m_out != 0)
        {
            m_pending = -1.0f;
            m_out = aRaw;
            return m_out;
        }
        // open gate, soft closure: time since its first sample (the first one counts 0)
        if (m_pending < 0.0f)
        {
            m_pending = 0.0f;
            ++m_suppressed;
        }
        else
            m_pending += dt;
        if (m_pending >= aCfg.holdSeconds)
        {
            m_pending = -1.0f;
            m_out = aRaw;
            ++m_confirmed;
            return m_out;
        }
        return 0u;
    }
    bool Pending() const { return m_pending >= 0.0f; }
    uint64_t Suppressed() const { return m_suppressed; } // soft closures started (debounced episodes)
    uint64_t Confirmed() const { return m_confirmed; }   // soft closures that lasted holdSeconds and closed the gate

private:
    float m_pending = -1.0f;
    uint32_t m_out = 0;
    uint64_t m_suppressed = 0;
    uint64_t m_confirmed = 0;
};

// ------------------------------------------------------------------------------------------------ bone map

// Game CName hash (RED4ext CName(const char*)): FNV-1a 64 over the bytes; "" and "None" map to 0. ASCII names only
// (the SDK hashes signed chars; identical for bytes < 0x80).
inline uint64_t CNameHash(std::string_view aName)
{
    uint64_t h = 0xCBF29CE484222325ull;
    for (const char c : aName)
    {
        h ^= static_cast<uint64_t>(static_cast<int64_t>(c)); // sign-extend like the SDK's `hash ^= *aText`
        h *= 0x100000001B3ull;
    }
    return (aName.empty() || aName == "None") ? 0 : h;
}

struct BoneMapResult
{
    int32_t gameIndex[kMaxMMBones]; // -1 = unmapped (never written)
    uint32_t numDbBones = 0;
    uint32_t mapped = 0;           // db bones in [firstBone, numDbBones) with a game index
    uint32_t missing = 0;          // names not found in the game rig (any db bone)
    uint32_t parentMismatch = 0;   // mapped bones whose game parent differs from the mapped db parent
    int32_t firstMissing = -1;     // db index of the first missing name
    int32_t firstParentMismatch = -1;
    bool ok = false;               // fail closed: true only if missing == 0 && parentMismatch == 0 && mapped > 0
};

// aDbHashes / aDbParents: database bones (name hash computed with the game's CName hash, parent index or -1).
// aGameHashes / aGameParents: game metaRig boneNames / parentIndeces (parents may be empty = no parent check).
// Bones below aFirstBone (Root, Trajectory) are resolved for the parent check only and are never mapped for writing.
inline BoneMapResult BuildBoneMap(std::span<const uint64_t> aDbHashes, std::span<const int32_t> aDbParents,
                                  std::span<const uint64_t> aGameHashes, std::span<const int16_t> aGameParents,
                                  uint32_t aFirstBone)
{
    BoneMapResult r{};
    for (int32_t& g : r.gameIndex)
        g = -1;
    const size_t n = aDbHashes.size();
    if (n == 0 || n > kMaxMMBones || aDbParents.size() != n || aGameHashes.empty() || aFirstBone >= n)
        return r;
    r.numDbBones = static_cast<uint32_t>(n);
    int32_t resolved[kMaxMMBones];
    for (size_t i = 0; i < n; ++i)
    {
        resolved[i] = -1;
        if (aDbHashes[i] != 0)
            for (size_t j = 0; j < aGameHashes.size(); ++j)
                if (aGameHashes[j] == aDbHashes[i])
                {
                    resolved[i] = static_cast<int32_t>(j);
                    break;
                }
        if (resolved[i] < 0)
        {
            ++r.missing;
            if (r.firstMissing < 0)
                r.firstMissing = static_cast<int32_t>(i);
        }
    }
    const bool checkParents = aGameParents.size() == aGameHashes.size();
    for (size_t i = aFirstBone; i < n; ++i)
    {
        if (resolved[i] < 0)
            continue;
        if (checkParents)
        {
            const int32_t dp = aDbParents[i];
            const int32_t expected = (dp >= 0 && static_cast<size_t>(dp) < n) ? resolved[dp] : -1;
            const int32_t actual = aGameParents[static_cast<size_t>(resolved[i])];
            if (dp >= 0 && actual != expected)
            {
                ++r.parentMismatch;
                if (r.firstParentMismatch < 0)
                    r.firstParentMismatch = static_cast<int32_t>(i);
                continue;
            }
        }
        r.gameIndex[i] = resolved[i];
        ++r.mapped;
    }
    r.ok = r.missing == 0 && r.parentMismatch == 0 && r.mapped > 0;
    if (!r.ok)
    {
        for (int32_t& g : r.gameIndex)
            g = -1;
        r.mapped = 0;
    }
    return r;
}

// ------------------------------------------------------------------------------------------------ snapshot / seqlock

struct PoseSnapshot
{
    float weight = 0.0f;
    uint32_t numBones = 0;       // database bones carried
    uint32_t firstBone = 2;      // first bone ever written
    uint32_t pad = 0;
    uint64_t rig = 0;            // game metaRig the map belongs to (FK R must equal it)
    int64_t publishTicks = 0;    // QueryPerformanceCounter at publish (0 offline)
    uint64_t sequence = 0;       // publish counter (diagnostics)
    int32_t gameIndex[kMaxMMBones];
    float pose[kMaxMMBones * 7]; // local p.xyz, q.xyzw per database bone
    // 0.6.2 upper-body mask: the bones flagged in upperBits (database index) are written with weight * upperScale; the others
    // with weight. upperScale 1 (default) = full body (0.6.1); 0 = those bones keep the game pose.
    float upperScale = 1.0f;
    uint32_t upperBits[(kMaxMMBones + 31) / 32] = {};
    int32_t traceAction = 0, traceOverlay = 0, traceAiming = 0, traceClip = -1;
    bool reloadContactsEligible = false; // v2 stationary pistol / rifle on the ground; same-cycle correction at early mapping only
    bool reloadAuthoredClip = false; // verified local reload episode; never inferred from PSM alone
    float reloadContactsWeight = 0.0f;
    int32_t bodyType = -1; // 0 woman / 1 man only on an actively supplied v2 pose; -1 unknown
    // UTPP-MOTION SEAM motionfix item 6 (agreed with the runtime): 1 = the published v2 pose has BOTH hands on the weapon
    // (Pistol2H / Rifle in Ready / Aiming, steady: no overlay or aim crossfade, no v1 <-> v2 blend, upper body not handed
    // back to the game, no reload/weapon-action handback; Pistol1H only if its left hand is within kOverlayArmsMaxHandDist of
    // WeaponRight - none of the measured poses is). 0 on every v1 frame, blend and release. Written only by motiondriver ApplyLocked.
    uint32_t overlayArms = 0;
    // Melee full body (2026-11-09, mm/MeleeFullBody.hpp): the bones NOT flagged in upperBits are written with weight * lowerScale.
    // 1 (default) = every pre-melee-full-body path bit for bit; 0 = Hips, Spine and the legs keep the game pose as well.
    float lowerScale = 1.0f;
    // Melee aim tilt (mm/MeleeAim.hpp): 0..1 fade of the v1 melee mask reason; 0 (default) = the tilt layer does nothing.
    float meleeAimWeight = 0.0f;
    // Melee chest base correction (mm/MeleeBase.hpp): 1 = fists drawn (script flag) and V stands on the ground (not crouch / sprint /
    // slide / air): the fists clip table may correct the chest. 0 (default) = the layer does nothing.
    uint32_t meleeBase = 0;
};
static_assert(std::is_trivially_copyable_v<PoseSnapshot>);
static_assert(sizeof(PoseSnapshot) % 4 == 0);

// Two slots, each protected by a sequence counter; the payload is stored as relaxed atomic 32-bit words so there is
// no data race in the C++ sense. Single writer. Readers never block and never allocate; a torn read returns false.
template<typename T>
class SeqlockBuffer
{
    static_assert(std::is_trivially_copyable_v<T> && sizeof(T) % 4 == 0);
    static constexpr size_t kWords = sizeof(T) / 4;

public:
    void Publish(const T& aValue)
    {
        const uint32_t slot = m_latest.load(std::memory_order_relaxed) ^ 1u;
        Slot& s = m_slots[slot];
        const uint32_t seq = s.seq.load(std::memory_order_relaxed);
        s.seq.store(seq + 1, std::memory_order_relaxed); // odd = writing
        std::atomic_thread_fence(std::memory_order_release);
        uint32_t w[kWords];
        std::memcpy(w, &aValue, sizeof(T));
        for (size_t i = 0; i < kWords; ++i)
            s.words[i].store(w[i], std::memory_order_relaxed);
        s.seq.store(seq + 2, std::memory_order_release);
        m_latest.store(slot, std::memory_order_release);
        m_published.store(true, std::memory_order_release);
    }

    // Copies the latest complete value into aOut. Returns false if nothing was published or both attempts were torn.
    bool Read(T& aOut) const
    {
        if (!m_published.load(std::memory_order_acquire))
            return false;
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            const Slot& s = m_slots[m_latest.load(std::memory_order_acquire)];
            const uint32_t s1 = s.seq.load(std::memory_order_acquire);
            if (s1 & 1u)
                continue;
            uint32_t w[kWords];
            for (size_t i = 0; i < kWords; ++i)
                w[i] = s.words[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (s.seq.load(std::memory_order_relaxed) != s1)
                continue;
            std::memcpy(&aOut, w, sizeof(T));
            return true;
        }
        return false;
    }

    // Test hook: begin/abandon a write on the slot readers will look at next (simulates a writer preempted mid-write).
    void TestBeginWriteOnLatest()
    {
        Slot& s = m_slots[m_latest.load(std::memory_order_relaxed)];
        s.seq.fetch_add(1, std::memory_order_relaxed);
    }
    void TestEndWriteOnLatest()
    {
        Slot& s = m_slots[m_latest.load(std::memory_order_relaxed)];
        s.seq.fetch_add(1, std::memory_order_release);
    }

private:
    struct Slot
    {
        std::atomic<uint32_t> seq{0};
        std::atomic<uint32_t> words[kWords]{};
    };
    Slot m_slots[2];
    std::atomic<uint32_t> m_latest{0};
    std::atomic<bool> m_published{false};
};

// ------------------------------------------------------------------------------------------------ blend

// Blends the snapshot pose into the game's local bone records (48 B: pos float4 @0, quat xyzw @0x10, scale @0x20).
// Only bones [firstBone, numBones) with a valid game index < aCount are touched; scale and pos.w are kept.
// Per bone weight (0.6.2): aWeight, times aSnap.upperScale for the bones in aSnap.upperBits (upper-body mask; a non-finite
// scale counts as 1 = full body, the scale is clamped to [0, 1]); the other bones times aSnap.lowerScale (melee full body,
// same clamp; 1 = the 0.6.2 formula bit for bit).
// Bone weight <= 0: nothing is written for it (bitwise identical). >= 1: the MM transform is written (quaternion sign
// aligned to the game's). Otherwise position lerp + shortest-path normalized quaternion lerp.
// Returns the number of bones written. No allocation, no locks.
inline bool SnapshotUpperBone(const PoseSnapshot& aSnap, uint32_t aBone)
{
    return aBone < kMaxMMBones && ((aSnap.upperBits[aBone >> 5] >> (aBone & 31u)) & 1u) != 0;
}
// lowerScale clamped to [0, 1]; non-finite = 1 (no full body).
inline float SnapshotLowerScale(const PoseSnapshot& aSnap)
{
    const float ls = aSnap.lowerScale;
    return std::isfinite(ls) ? (ls < 0.0f ? 0.0f : (ls > 1.0f ? 1.0f : ls)) : 1.0f;
}

inline uint32_t BlendIntoRecords(uint8_t* aBones, uint32_t aCount, const PoseSnapshot& aSnap, float aWeight, float aKeepLowerOnMM = 0.0f)
{
    if (!aBones || !(aWeight > 0.0f))
        return 0;
    const float wFull = aWeight >= 1.0f ? 1.0f : aWeight;
    const float us = std::isfinite(aSnap.upperScale) ? (aSnap.upperScale < 0.0f ? 0.0f : (aSnap.upperScale > 1.0f ? 1.0f : aSnap.upperScale))
                                                     : 1.0f;
    const float wUpper = wFull * us;
    // aKeepLowerOnMM (0..1, default 0 = the formula above bit for bit): the melee full body guard (mm/MeleeBase.hpp) keeps Hips / Spine /
    // legs on MM with this weight instead of handing them to the game pose (eased by the caller, no snap)
    const float lowerScale = SnapshotLowerScale(aSnap);
    const float lowerEff = !(aKeepLowerOnMM > 0.0f) ? lowerScale : (aKeepLowerOnMM >= 1.0f ? 1.0f : lowerScale + (1.0f - lowerScale) * aKeepLowerOnMM);
    const float wLower = wFull * lowerEff;
    const uint32_t nb = aSnap.numBones <= kMaxMMBones ? aSnap.numBones : kMaxMMBones;
    uint32_t written = 0;
    for (uint32_t b = aSnap.firstBone; b < nb; ++b)
    {
        const int32_t g = aSnap.gameIndex[b];
        if (g < 0 || static_cast<uint32_t>(g) >= aCount)
            continue;
        const float w = SnapshotUpperBone(aSnap, b) ? wUpper : wLower;
        if (!(w > 0.0f))
            continue; // masked bone fully on the game pose (bitwise untouched)
        const float* m = &aSnap.pose[size_t(b) * 7];
        bool finite = true;
        for (int k = 0; k < 7; ++k)
            finite = finite && std::isfinite(m[k]);
        if (!finite)
            continue;
        float* rec = reinterpret_cast<float*>(aBones + size_t(g) * kGameBoneStride);
        float* q = rec + kGameRotOffsetFloats;
        float mq[4] = {m[3], m[4], m[5], m[6]};
        if (q[0] * mq[0] + q[1] * mq[1] + q[2] * mq[2] + q[3] * mq[3] < 0.0f)
            for (float& v : mq)
                v = -v;
        if (w >= 1.0f)
        {
            rec[0] = m[0];
            rec[1] = m[1];
            rec[2] = m[2];
            q[0] = mq[0];
            q[1] = mq[1];
            q[2] = mq[2];
            q[3] = mq[3];
        }
        else
        {
            rec[0] += (m[0] - rec[0]) * w;
            rec[1] += (m[1] - rec[1]) * w;
            rec[2] += (m[2] - rec[2]) * w;
            float r[4];
            for (int k = 0; k < 4; ++k)
                r[k] = q[k] + (mq[k] - q[k]) * w;
            const float n2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3];
            if (!(n2 > 1e-20f) || !std::isfinite(n2))
                continue; // keep the game rotation (position already blended)
            const float inv = 1.0f / std::sqrt(n2);
            for (int k = 0; k < 4; ++k)
                q[k] = r[k] * inv;
        }
        ++written;
    }
    return written;
}

// ------------------------------------------------------------------------------------------------ 0.6.2 upper-body mask
// Every MM gate closure handed the WHOLE body to the game's vanilla FPP sets (earlier test,
// 2026-10-07 14:07-14:22: "fists attacks and block = V crouches", "after leaving the car / bike V crouches"). Design 29
// item 4: a bone mask - pelvis, legs and Spine from MM, Spine1 and everything above it (neck, head, clavicles, arms, hands,
// fingers, WeaponLeft / WeaponRight, eyes) from the game graph. For the masked states the gate does not close: MM keeps the
// lower body (normal stand / walk / run query) and the upper-body bones fade to the game pose (UpperMaskBlend, 0.15 s).
enum MaskProfile : int32_t
{
    kMaskFullBody = 0,      // every mapped bone written by MM (0.6.1)
    kMaskUpperFromGame = 1, // lower body (Hips, Spine, legs, feet) by MM, the rest by the game
};
inline constexpr uint32_t kMaskWords = (kMaxMMBones + 31) / 32;

// kMaskUpperFromGame: bones MM keeps while masked (rig player_woman_skeleton 0..72):
// Hips 2, Spine 4, Left/RightUpLeg 5/6, Left/RightLeg 8/9, Left/RightFoot 11/12,
// Left/RightHeel 29/30, Left/RightToeBase 43/44. Exact names for Hips / Spine (Spine1..3 are upper body), prefixes for the
// leg chains. Every other name (incl. unknown ones) belongs to the game while masked (= the 0.6.1 closed-gate behaviour).
inline bool IsMaskLowerBone(std::string_view aName)
{
    if (aName == "Hips" || aName == "Spine")
        return true;
    static constexpr std::string_view kPrefixes[] = {"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftHeel", "LeftToe",
                                                     "RightUpLeg", "RightLeg", "RightFoot", "RightHeel", "RightToe"};
    for (const std::string_view p : kPrefixes)
        if (aName.substr(0, p.size()) == p)
            return true;
    return false;
}

struct UpperMask
{
    uint32_t bits[kMaskWords] = {}; // database bones owned by the game while masked
    uint32_t lower = 0;             // bones [firstBone, numBones) kept by MM
    uint32_t upper = 0;             // bones [firstBone, numBones) given to the game
    bool ok = false;                // fail closed: Hips and Spine1 found, both sets non-empty (else no mask = full close, 0.6.1)
    bool Upper(uint32_t aBone) const { return aBone < kMaxMMBones && ((bits[aBone >> 5] >> (aBone & 31u)) & 1u) != 0; }
};

// aNameAt(uint32_t bone) -> std::string_view. Bones below aFirstBone (Root, Trajectory) are never written: not flagged.
template <typename NameAt>
inline UpperMask BuildUpperMask(uint32_t aNumBones, uint32_t aFirstBone, NameAt&& aNameAt)
{
    UpperMask m;
    if (aNumBones == 0 || aNumBones > kMaxMMBones || aFirstBone >= aNumBones)
        return m;
    bool hips = false, spine1 = false;
    for (uint32_t b = aFirstBone; b < aNumBones; ++b)
    {
        const std::string_view n = aNameAt(b);
        hips = hips || n == "Hips";
        spine1 = spine1 || n == "Spine1";
        if (IsMaskLowerBone(n))
            ++m.lower;
        else
        {
            m.bits[b >> 5] |= 1u << (b & 31u);
            ++m.upper;
        }
    }
    m.ok = hips && spine1 && m.lower > 0 && m.upper > 0;
    if (!m.ok)
        for (uint32_t& w : m.bits)
            w = 0;
    return m;
}

// TPP weapon-action handback: retain the game's timed arm/hand/weapon motion (notably reload), while
// keeping the torso, neck, head, and lower body on the authored MM/motion pose. FPP action clips often
// contort the torso for a first-person camera; passing the spine back with the arms carries that bend
// into the visible TPP body. Both arm chains and both weapon attachment bones are selected together so
// two-handed contact and the weapon's right-hand attachment remain in the same source pose.
inline bool IsWeaponActionBone(std::string_view aName)
{
    static constexpr std::string_view kPrefixes[] = {"LeftShoulder", "RightShoulder", "LeftArm", "RightArm",
                                                     "LeftForeArm", "RightForeArm", "LeftHand", "RightHand",
                                                     "LeftInHand", "RightInHand"};
    for (const std::string_view p : kPrefixes)
        if (aName.substr(0, p.size()) == p)
            return true;
    return aName == "WeaponLeft" || aName == "WeaponRight";
}

template <typename NameAt>
inline UpperMask BuildWeaponActionMask(uint32_t aNumBones, uint32_t aFirstBone, NameAt&& aNameAt)
{
    UpperMask m;
    if (aNumBones == 0 || aNumBones > kMaxMMBones || aFirstBone >= aNumBones)
        return m;
    bool hips = false, spine1 = false, leftShoulder = false, rightShoulder = false;
    bool leftHand = false, rightHand = false, weaponLeft = false, weaponRight = false;
    for (uint32_t b = aFirstBone; b < aNumBones; ++b)
    {
        const std::string_view n = aNameAt(b);
        hips = hips || n == "Hips";
        spine1 = spine1 || n == "Spine1";
        leftShoulder = leftShoulder || n == "LeftShoulder";
        rightShoulder = rightShoulder || n == "RightShoulder";
        leftHand = leftHand || n == "LeftHand";
        rightHand = rightHand || n == "RightHand";
        weaponLeft = weaponLeft || n == "WeaponLeft";
        weaponRight = weaponRight || n == "WeaponRight";
        if (IsWeaponActionBone(n))
        {
            m.bits[b >> 5] |= 1u << (b & 31u);
            ++m.upper;
        }
        else
            ++m.lower;
    }
    m.ok = hips && spine1 && leftShoulder && rightShoulder && leftHand && rightHand && weaponLeft && weaponRight &&
           m.lower > 0 && m.upper > 0;
    if (!m.ok)
        for (uint32_t& w : m.bits)
            w = 0;
    return m;
}

// Mask states (UTPPSetMMUpperMaskStates bits).
enum UpperMaskState : uint32_t
{
    kMaskStateMelee = 1u << 0, // melee gate active (kGateMelee: attack / block / targeting states + release hold)
    kMaskStateAim = 1u << 1,   // PSM.UpperBody == Aim (6)
    kMaskStateExit = 1u << 2,  // vehicle exit window: from the end of the PSM vehicle state until MM is fully back (max kExitWindowSeconds)
};
inline constexpr uint32_t kMaskStatesAll = kMaskStateMelee | kMaskStateAim | kMaskStateExit;
inline constexpr int kPsmUpperBodyAim = 6;       // gamePSMUpperBodyStates.Aim
inline constexpr int kPsmDetailNotInBase = 0;    // gamePSMDetailedLocomotionStates.NotInBaseLocomotion
inline constexpr int kPsmDetailAimWalk = 2;      // gamePSMDetailedLocomotionStates.AimWalk
inline constexpr float kMaskFadeSeconds = 0.15f; // upper-body bones fade game <-> MM
inline constexpr float kExitWindowSeconds = 2.0f;

struct UpperMaskInput
{
    uint32_t gate = 0;       // gate after the debounce, driver bits included
    uint32_t psm = 0;        // PsmBlockBits of the feed
    int psmDetail = -1;      // PsmDetailOf (-1 = not reported)
    float vz = 0.0f;         // world vertical velocity (m/s)
    bool meleeActive = false;
    int upperBody = -1;      // PsmUpperBodyOf (-1 = not reported)
    bool exitWindow = false;
    uint32_t states = kMaskStatesAll;
    float maxVertical = 1.0f; // m/s: at or above, the soft locomotion reasons are not covered (jump, fall)
    // CAR-1 / MOTO-6 (vehicle batch): third-person feed and a weapon in hand (draw / holster / held). Defaults = the 0.6.2
    // behaviour (upper body on the game pose for every mask reason).
    bool tpp = false;
    bool weaponAction = false;
};

struct UpperMaskDecision
{
    uint32_t reasons = 0; // kMaskState* bits holding the mask (0 = no mask: the gate acts on the whole body)
    uint32_t covered = 0; // gate bits replaced by the mask (removed from the gate the weight follows)
    // CAR-1 / MOTO-6: the upper-body bones take the game pose. False only for an exit-window-only mask in TPP without a weapon
    // action: the game pose after an exit is the vanilla first-person set (deep squat, hand at the head), so the mask keeps
    // covering the gate bits (legs on MM) but the upper body stays on MM too. Melee / aim, FPP and weapon actions: true.
    bool upperFromGame = false;
};

// A masked state covers kGateMelee (melee) and the soft locomotion reasons that come with it: kGateLocoState and kPsmDetailed
// with AimWalk (aim; fists block / targeting raise UpperBody Aim + AimWalk, game 14:18:40.741 gate=0x510 detail=2) or, in
// the exit window, NotInBaseLocomotion / locoState 0 (bike exit, game 14:14:59.756 gate=0x10 loco=0 for 0.6 s) - only with
// |vz| < maxVertical. Any other reason (vehicle, scene, workspot, swim, ladder, ragdoll / takedown, fall, dash over-speed,
// TPP off, stale feed, fault...) keeps the full close: reasons = 0.
inline UpperMaskDecision DecideUpperMask(const UpperMaskInput& aIn)
{
    UpperMaskDecision d;
    uint32_t r = 0;
    if ((aIn.states & kMaskStateMelee) && aIn.meleeActive)
        r |= kMaskStateMelee;
    if ((aIn.states & kMaskStateAim) && aIn.upperBody == kPsmUpperBodyAim)
        r |= kMaskStateAim;
    if ((aIn.states & kMaskStateExit) && aIn.exitWindow)
        r |= kMaskStateExit;
    if (r == 0)
        return d;
    uint32_t covered = (r & kMaskStateMelee) ? uint32_t(kGateMelee) : 0u;
    const bool still = std::isfinite(aIn.vz) && std::fabs(aIn.vz) < aIn.maxVertical;
    bool detailOk = aIn.psm == 0;
    if (aIn.psm == kPsmDetailed)
        detailOk = aIn.psmDetail == kPsmDetailAimWalk || ((r & kMaskStateExit) != 0 && aIn.psmDetail == kPsmDetailNotInBase);
    if (still && detailOk)
        covered |= kGateLocoState | (aIn.psm != 0 ? uint32_t(kGatePsmBlock) : 0u);
    if ((aIn.gate & ~covered) != 0)
        return d;
    d.reasons = r;
    d.covered = covered & aIn.gate;
    d.upperFromGame = (r & ~uint32_t(kMaskStateExit)) != 0 || !aIn.tpp || aIn.weaponAction;
    UTPP_IF_MUT(VEHEXIT, 3)
        d.upperFromGame = true; // mutation 3: the TPP exit window keeps the game upper body (0.6.2)
    return d;
}

// ------------------------------------------------------------------------------------------------ 0.6.2 hold last pose
// A/B (MM disabled in TPP): with the MM weight at 0 the game's full body is a
// literal T-pose in idle - the vanilla player graph has no third-person animation outside a few states. So in TPP MM must not
// hand the body back to the game except where the game really animates a third-person body. Release (no hold): PSM vehicle,
// scene, swim, workspot, takedown, body carry, vitals (death), ladder (LocomotionDetailed Ladder..LadderJump 10..13), knockdown /
// felled (29 / 31), TPP off, no usable MM pose (database / map / matcher / fault). Every other
// closure is held: MM disabled, database switch, gate debounce, unsupported locomotion state (jump, fall, slide, dodge, climb,
// AimWalk...), melee, aim, exit window, over-speed, stale feed, menu / photo mode, stagger, missing blackboard.
inline constexpr uint32_t kPsmReleaseMask = kPsmVehicle | kPsmScene | kPsmSwimming | kPsmWorkspot | kPsmTakedown | kPsmCarry | kPsmVitals;
inline constexpr uint32_t kHoldGateMask = kGateMMDisabled | kGateNotMapped | kGateLocoState | kGateStaleFeed | kGatePsmBlock |
                                          kGateOverSpeed | kGateMelee;
// AIR HOLD stage (2026-10-13): ONE list of the releasing LocomotionDetailed ids (ladder 10..13, knockdown 29, felled 31), shared by
// HoldEligible and the air area (AirState Reset); agreed with the motion session. Landings 23..25 stay hold-eligible.
inline constexpr int kPsmReleaseDetails[] = {10, 11, 12, 13, 29, 31};
inline constexpr bool PsmDetailReleases(int aDetail)
{
    for (int r : kPsmReleaseDetails)
        if (r == aDetail)
            return true;
    return false;
}

// AIR HOLD stage: air / land / no-trigger LocomotionDetailed helpers (single source of truth; mm/AirState.hpp uses these).
inline constexpr int kPsmAirDetailFirst = 14;  // 14..22 airborne (14 fall, 18..21 jump variants, 22 other air)
inline constexpr int kPsmAirDetailLast = 22;
inline constexpr int kPsmJumpDetailFirst = 18; // 18..21 jump starts (take-off trigger)
inline constexpr int kPsmJumpDetailLast = 21;
inline constexpr int kPsmLandDetailFirst = 23; // 23..28 landing states (touchdown)
inline constexpr int kPsmLandDetailLast = 28;
inline constexpr int kPsmNoTriggerDetails[] = {5, 7, 8, 9}; // slide, dodge, climb, vault: never start an air episode
inline constexpr bool PsmIsAirDetail(int d) { return d >= kPsmAirDetailFirst && d <= kPsmAirDetailLast; }
inline constexpr bool PsmIsJumpDetail(int d) { return d >= kPsmJumpDetailFirst && d <= kPsmJumpDetailLast; }
inline constexpr bool PsmIsLandDetail(int d) { return d >= kPsmLandDetailFirst && d <= kPsmLandDetailLast; }
inline constexpr bool PsmIsClimbDetail(int d) { return d == 8 || d == 9; } // entering these mid-episode -> air Reset
inline constexpr bool PsmIsNoTriggerDetail(int d)
{
    for (int r : kPsmNoTriggerDetails)
        if (r == d)
            return true;
    return PsmDetailReleases(d);
}

// CLIMB RELEASE stage (2026-10-13; motion + game animations only): PSM
// LocomotionDetailed Climb 8 / Vault 9 hand the body to the GAME climb / vault animation unless v2 plays a traversal
// (motiondriver::OwnsTraversal). Driver release reasons are not gate bits: the gate is already closed there (kGatePsmBlock from
// kPsmDetailed); a release reason only makes a closure that would be held (HoldEligible) release, so the normal gate fader fades
// MM out and the reopen hold / fade-in bring it back after the episode.
// TRAV-1 (vehicle batch): "motion first, the game animation where motion has none". With v2 traversal active
// (aMotionActive = MotionHooks::travActive: mm.motionDriver on and the bundle has traversal data) every Climb / Vault that v2 does not play is
// released whatever mm.climbRelease says (a held ground pose walks through the obstacle, e.g. a car hood). The setting
// (UTPPSetMMClimbRelease) keeps its meaning for v1 only (mm.motionDriver off): false = 8 / 9 hold-eligible as before.
enum ReleaseReason : uint32_t
{
    kReleaseTrav = 1u << 0, // 'travRelease': detail 8 / 9, (mm.climbRelease on or v2 traversal active), v2 does not own it
};
inline UTPP_MUT_CONSTEXPR_VEHEXIT bool TravReleases(bool aOn, bool aMotionActive, uint32_t aPsm, int aDetail, bool aMotionOwnsTraversal)
{
    UTPP_IF_MUT(VEHEXIT, 4)
        aMotionActive = false; // mutation 4: v2 traversal activity ignored (pre-TRAV-1 semantics)
    return (aOn || aMotionActive) && (aPsm & kPsmDetailed) != 0 && PsmIsClimbDetail(aDetail) && !aMotionOwnsTraversal;
}

struct HoldInput
{
    bool on = true;         // UTPPSetMMHoldPose
    bool tpp = false;       // third-person active (last feed)
    bool poseOk = false;    // database + matcher + bone map + armed rig, no fault, a pose computed since the last matcher init
    uint32_t gate = 0;      // gate (driver bits included)
    uint32_t psm = 0;       // PsmBlockBits
    int psmDetail = -1;     // PsmDetailOf
    uint32_t release = 0;   // CLIMB RELEASE stage: driver release reasons (kRelease*); any -> not eligible
    // CAR-1 (vehicle batch): vehicle exit window (from the vehicle-bit falling edge until MM is fully back, max
    // kExitWindowSeconds) and the PSM bits latched at that edge (only kPsmWorkspot is used: the vanilla exit workspot, set during
    // the ride and still set on the edge feed; the driver clears it from the latch on the first feed without it).
    bool exitWindow = false;
    uint32_t exitLatchedPsm = 0;
};

inline constexpr int kPsmDetailVeryHardLand = 25; // gamePSMDetailedLocomotionStates.VeryHardLand

// CAR-1: PSM release bits that apply to this feed. Inside the exit window a workspot latched at the vehicle-bit falling edge is
// the exit animation's own workspot (the game pose there is the first-person set): it is held, not released. A NEW workspot
// (rising edge inside the window, or any workspot on foot) still releases. Outside the window: kPsmReleaseMask unchanged.
inline UTPP_MUT_CONSTEXPR_VEHEXIT uint32_t ExitReleaseMask(bool aExitWindow, uint32_t aExitLatchedPsm)
{
    UTPP_IF_MUT(VEHEXIT, 1)
        return kPsmReleaseMask; // mutation 1: no workspot latch exemption
    return (aExitWindow && (aExitLatchedPsm & kPsmWorkspot) != 0) ? (kPsmReleaseMask & ~uint32_t(kPsmWorkspot)) : kPsmReleaseMask;
}

// Bike jump-out (2026-10-28, recording 01-22-53; 34 jump-outs in 13 feed logs): at the unmount the game
// clears PSM.IsInWorkspot but keeps PSM.Vehicle != Default through the whole flight (0.77-0.78 s; launch at 9.65-10.4 m/s, vz >= 3 m/s
// 0.03-0.055 s after the workspot drop; 3.2 m rise). The vehicle bit is a release bit, so the body stayed on the game's rigid flight
// pose (straight hanging legs) until the apex, where the bit fell and MM took over in mid-air. Normal unmounts (18 in the same logs)
// show the vehicle-only state for 2 feeds at vz 0; the one crash ejection (knockdown 29) carries the LocomotionDetailed bit. JumpOutLatch: after a ride feed
// (vehicle + workspot), a vehicle-only feed (no other block bit) within kJumpOutArmSeconds of the workspot drop rising at
// >= kJumpOutRiseMs in TPP releases the vehicle bit for the REST of that PSM vehicle episode (the driver strips it from the
// conditioned PSM, so that feed is the unmount edge: exit window, exit hand-in, MM air). Only the trigger is time-limited; the latch
// ends when the raw vehicle bit falls, the workspot comes back (a new ride / seat switch) or on Reset - never mid-flight (a latch
// expiring while the raw bit is still set would re-raise the vehicle bit and cut the weight to 0 in the air).
inline constexpr float kJumpOutRiseMs = 3.0f;     // m/s
inline constexpr float kJumpOutArmSeconds = 0.25f; // s after the workspot drop (observed launch 0.03-0.055 s)
class JumpOutLatch
{
public:
    void Reset()
    {
        m_ride = m_active = false;
        m_sinceDrop = 0.0f;
    }
    // aPsm: PsmBlockBits of this feed (gate bits only). True: this feed's vehicle bit is no ride any more (strip kPsmVehicle).
    bool Update(uint32_t aPsm, float aVz, bool aTpp, float aDt)
    {
        const bool veh = (aPsm & kPsmVehicle) != 0;
        if (!veh)
        {
            Reset();
            return false;
        }
        if ((aPsm & kPsmWorkspot) != 0)
        {
            m_ride = true; // mounted (the ride is vehicle + workspot); also a seat switch / re-mount after a jump-out
            m_active = false;
            m_sinceDrop = 0.0f;
            return false;
        }
        if (m_active)
            return true;
        m_sinceDrop += (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        UTPP_IF_MUT(VEHEXIT, 10)
        {
            // mutation 10: no jump-out release (pre-fix: the game flight pose until the vehicle bit falls)
        }
        else UTPP_IF_MUT(VEHEXIT, 11)
            // mutation 11: no ride / arm window guard (any vehicle-only rising feed releases)
            m_active = aTpp && (aPsm & ~uint32_t(kPsmVehicle)) == 0 && std::isfinite(aVz) && aVz >= kJumpOutRiseMs;
        else
            m_active = m_ride && m_sinceDrop <= kJumpOutArmSeconds && aTpp && (aPsm & ~uint32_t(kPsmVehicle)) == 0 && std::isfinite(aVz) &&
                       aVz >= kJumpOutRiseMs;
        if (m_active)
            ++m_fired;
        return m_active;
    }
    bool Active() const { return m_active; }
    float SinceDrop() const { return m_sinceDrop; }
    uint64_t Fired() const { return m_fired; }
    void ClearStats() { m_fired = 0; }

private:
    bool m_ride = false;      // a ride feed (vehicle + workspot) since the vehicle bit rose
    bool m_active = false;    // the vehicle bit of this episode is released
    float m_sinceDrop = 0.0f; // s of vehicle-only feeds since the workspot drop
    uint64_t m_fired = 0;
};

// The first clause that makes HoldEligible false (nullptr = eligible). Diagnostic names for the EXITDIAG line.
inline const char* HoldBlocker(const HoldInput& aIn)
{
    if (!aIn.on)
        return "off";
    if (!aIn.tpp)
        return "tpp";
    if (!aIn.poseOk)
        return "pose";
    if ((aIn.gate & ~kHoldGateMask) != 0)
        return "gate";
    const uint32_t rel = aIn.psm & ExitReleaseMask(aIn.exitWindow, aIn.exitLatchedPsm);
    if (rel != 0)
    {
        if (rel & kPsmVehicle)
            return "release:vehicle";
        if (rel & kPsmScene)
            return "release:scene";
        if (rel & kPsmSwimming)
            return "release:swim";
        if (rel & kPsmWorkspot)
            return "release:workspot";
        if (rel & kPsmTakedown)
            return "release:takedown";
        if (rel & kPsmCarry)
            return "release:carry";
        return "release:vitals";
    }
    if ((aIn.psm & kPsmDetailed) != 0 && PsmDetailReleases(aIn.psmDetail))
        return "release:detail";
    // CAR-2: after a vehicle exit a VeryHardLand (impact beyond the motion land set) goes to the game's land animation
    if (aIn.exitWindow && (aIn.psm & kPsmDetailed) != 0 && aIn.psmDetail == kPsmDetailVeryHardLand)
        return "release:veryHardLand";
    if (aIn.release != 0)
        return "release:trav";
    return nullptr;
}

// True when the body must stay on the MM pose (the gate the weight follows is treated as open, no reopen hold).
// Outside the exit window (exitWindow false) this is the pre-batch rule bit for bit. Inside it the stale feed, loco state,
// over-speed and the non-releasing details (NotInBase 0, SlideFall 6, Fall 14, Jump 18, lands 23 / 24) are held as everywhere;
// vehicle again, scene, swim, takedown, carry, vitals, ladder 10-13, VeryHardLand 25, knockdown 29, felled 31 release.
inline bool HoldEligible(const HoldInput& aIn)
{
    return HoldBlocker(aIn) == nullptr;
}

// Mask fade (linear progress, smoothstep weight): 1 = upper-body bones on the game pose. Wanted -> 1 over aTime; not wanted
// and the gate open -> 0 over aTime; not wanted with the gate closed (full close fading the whole body out) -> held, so the
// upper body never flashes back to MM during a fade-out. AIR ownership can explicitly release this hold, because the
// effective MM gate stays open while the raw PSM gate is closed. Nothing visible (MM weight 0 before the update) -> snaps.
class UpperMaskBlend
{
public:
    void Reset() { m_progress = 0.0f; }
    float Update(bool aWanted, bool aGateOpen, bool aVisible, float aDt, float aTime)
    {
        return Update(aWanted, aGateOpen, aVisible, aDt, aTime, false);
    }
    // aReleaseWhileHeld: explicitly validated AIR-owned MM frame, with an effective open gate despite a closed raw gate.
    // Keep false for every legacy caller, releases, and frames owned by v2 or the game.
    float Update(bool aWanted, bool aGateOpen, bool aVisible, float aDt, float aTime, bool aReleaseWhileHeld)
    {
        return UpdateInOut(aWanted, aGateOpen, aVisible, aDt, aTime, aTime, aReleaseWhileHeld);
    }
    // Same with its own duration per direction: aTimeIn while wanted (progress rises), aTimeOut while not wanted (falls). With
    // aTimeIn == aTimeOut it is Update bit for bit. Melee release: the eased slow fade out (kMeleeReleaseFadeOutSeconds).
    float UpdateInOut(bool aWanted, bool aGateOpen, bool aVisible, float aDt, float aTimeIn, float aTimeOut, bool aReleaseWhileHeld = false)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        const float aTime = aWanted ? aTimeIn : aTimeOut;
        if (!aVisible)
            m_progress = aWanted ? 1.0f : 0.0f;
        else if (aWanted || aGateOpen || (aReleaseWhileHeld && !aWanted))
        {
            const float target = aWanted ? 1.0f : 0.0f;
            if (!(aTime > 0.0f) || !std::isfinite(aTime))
                m_progress = target;
            else if (m_progress < target)
                m_progress = std::fmin(target, m_progress + dt / aTime);
            else
                m_progress = std::fmax(target, m_progress - dt / aTime);
        }
        return Weight();
    }
    float Progress() const { return m_progress; }
    float Weight() const { return m_progress * m_progress * (3.0f - 2.0f * m_progress); }

private:
    float m_progress = 0.0f;
};

} // namespace utpp::mm::integ

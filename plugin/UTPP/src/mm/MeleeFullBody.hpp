#pragma once
// Melee full body (2026-11-09: melee goes full body; masks while moving maybe later) and the melee guard
// (2026-11-09 round 3, option a'). Header-only, no RED4ext types; unit-tested
// offline (tests/MeleeFullBodyTest.cpp). Runtime: MotionHost.cpp Feed (decisions + fades) and PublishLocked (PoseSnapshot::
// lowerScale = 1 - full-body fade weight; the guard folds into the existing upper-mask / full-body / tilt fades, no new field).
//
// The 0.6.2 upper-body mask keeps Hips, Spine and the legs (integ::IsMaskLowerBone) on MM during melee because the vanilla
// first-person melee clips have a static, crouched lower body. With the UTPP melee clips (standing lower body, planted feet,
// registered on the player root and gated by the anim wrapper variable utpp_foot_tpp) the lower body may come from the game
// too. Full body = those lower-body bones ALSO fade to the game pose, only while every condition below holds:
//   1. the script flag (UTPPSetMeleeFullBody: UTPP third person on foot and the equipped weapon's anim variable is covered by
//      the UTPP clips, arm cyberware excluded) and the user setting (UTPPSetMeleeFullBodySetting) are both on;
//   2. the v1 melee mask reason is active with the upper body on the game pose (attack / block / targeting + release hold),
//      or the guard holds the upper body with its own full-body setting on (stage 2);
//   3. the PSM.MeleeWeapon state of the feed is one whose third-person clips exist (allowed-state mask, UTPPSetMeleeFullBody
//      States; default kFullBodyDefaultStates = the attack states without ThrowAttack). Block, targeting, hold, parried,
//      deflect, the 0.3 s release hold (Idle), equipping and throws play clips that were never rebuilt (vanilla first-person
//      legs; final-mb7 coverage.txt "FOR THE DLL") -> legs stay on MM. Stage 2 of the guard adds Idle 2 only;
//   4. v1 owns the pose of the feed, third person, no release reason;
//   5. standing: stance Stand (the conditioned stance: block / targeting AimWalk = locoState 0 maps to the last stance),
//      LocomotionDetailed not reported / NotInBase / Stand / AimWalk (jump, fall, land, dodge, slide... end it at once);
//   6. on the ground: with the air feed of the frame, ungrounded for kAirSeconds ends it (debounced); without an air feed,
//      |vz| >= maxVertical ends it;
//   7. not moving: the stick is not held (move intent ends it on the same feed - block while walking keeps the legs on MM),
//      and the capsule has not been faster than kOffSpeed for kSustainSeconds of fast time (pushed / carried; dips shorter
//      than kBridgeSeconds are bridged: the pilot capsule speed alternated ~0.2 / ~0.97 m/s); after such a stretch it comes
//      back once below kOnSpeed for kBridgeSeconds. The attacks themselves move the capsule briefly (in-game pilot
//      2026-11-09: 33-66 ms at ~0.97 m/s per punch, ~0.18 s at ~3.9 m/s in combo finishers): no flicker of the legs to MM.
// The fade (integ::UpperMaskBlend, kFadeSeconds) runs both ways; with the effective gate closed (the whole body fading out) it
// holds, so the legs never flash back to MM during a full close. Root / Trajectory are never written (firstBone) as before.
// Flag off (default at DLL load): Update never returns true, the fade stays 0, lowerScale stays 1 = the previous output bit for bit.
//
// Guard: while a guard-capable melee weapon is drawn (script flag UTPPSetMeleeGuard: fists only for now, never Gorilla Arms)
// and the user setting melee.guard is on, the upper body stays on the game pose (the graph's Ready / Idle pose = the guard
// once the third-person melee_idlepose is installed) between strikes too, so the mask does not drop and rise on every strike.
// Conditions (GuardGate): TPP, v1 owns the pose, the effective gate open, PSM.MeleeWeapon Idle 2 or Parried..EquipAttack 5..21
// (attack states keep it: no gap at the strike start, MeleeGate debounce), PSM.UpperBody not reported / Default / Aim
// (SwitchItems / holster end it), standing or walking / jogging (stance Stand, StandingDetail), on the ground. Fades 0.15 s
// (the upper mask's own fade). Stage 2 (melee.guardFullBody): the legs on the game pose while standing in the guard. The
// melee aim tilt may keep working through the guard (melee.guardAimTilt) so it does not fade in and out at every strike.
// All guard settings default off: then GuardGate never reports on and the driver terms are the previous ones bit for bit.

#include <cmath>
#include <cstdint>

namespace utpp::mm::melee
{
inline constexpr float kOnSpeed = 0.35f;        // m/s planar capsule speed: a sustained-speed stop ends below this ...
inline constexpr float kOffSpeed = 0.60f;       // m/s: above this ...
inline constexpr float kSustainSeconds = 0.35f; // ... for this much fast time (stick or not) -> off until below kOnSpeed
inline constexpr float kBridgeSeconds = 0.15f;  // s: a dip shorter than this neither resets the fast time nor ends the stop
inline constexpr float kAirSeconds = 0.10f;     // ungrounded (air feed) this long -> off
inline constexpr float kMaxStepDt = 0.10f;      // s: one feed advances the timers by at most this (a hitch alone trips nothing)
inline constexpr float kFadeSeconds = 0.2f;     // lower-body fade game <-> MM (smoothstep through integ::UpperMaskBlend)
inline constexpr float kReleaseFadeSeconds = 0.35f; // lower-body fade OUT after a melee weapon state ended (Why state / melee), see LegFadeOutSeconds
inline constexpr int32_t kStanceStand = 1;      // integ::kLocoStand

// ------------------------------------------------------------------------------------------------ PSM.MeleeWeapon states

// gamePSMMeleeWeapon (game 2.31 orphans.script:6830-6854; scripts/utpp.reds reports it in the PSM block, -1 = not reported)
enum PsmMeleeWeapon : int32_t
{
    kMwNotReady = 0,
    kMwEquipping = 1,
    kMwIdle = 2,
    kMwSafe = 3,
    kMwPublicSafe = 4,
    kMwParried = 5,
    kMwHold = 6,
    kMwChargedHold = 7,
    kMwBlock = 8,
    kMwTargeting = 9,
    kMwDeflect = 10,
    kMwComboAttack = 11,
    kMwFinalAttack = 12,
    kMwStrongAttack = 13,
    kMwSafeAttack = 14,
    kMwBlockAttack = 15,
    kMwSprintAttack = 16,
    kMwCrouchAttack = 17,
    kMwJumpAttack = 18,
    kMwThrowAttack = 19,
    kMwDeflectAttack = 20,
    kMwEquipAttack = 21,
    kMwLastState = 21, // Default 22 is no state of its own
};

inline constexpr uint32_t StateBit(int32_t aState) { return (aState >= 0 && aState <= kMwLastState) ? (1u << aState) : 0u; }
inline constexpr uint32_t kStatesValid = (1u << (kMwLastState + 1)) - 1u; // bits 0..21
// Attack states 11..21 without ThrowAttack 19 (the throw clips were rejected by the export gate and stay vanilla, final-mb7
// coverage.txt). Block 8 / Hold 6 / ... are added by the script (UTPPSetMeleeFullBodyStates) once their clips are installed.
inline constexpr uint32_t kFullBodyDefaultStates =
    StateBit(kMwComboAttack) | StateBit(kMwFinalAttack) | StateBit(kMwStrongAttack) | StateBit(kMwSafeAttack) |
    StateBit(kMwBlockAttack) | StateBit(kMwSprintAttack) | StateBit(kMwCrouchAttack) | StateBit(kMwJumpAttack) |
    StateBit(kMwDeflectAttack) | StateBit(kMwEquipAttack);
static_assert(kFullBodyDefaultStates == 0x37F800u);
inline constexpr bool StateAllowed(uint32_t aMask, int32_t aState) { return (StateBit(aState) & aMask) != 0; }
// UTPPSetMeleeFullBodyStates: bits 0..21 only (Int32 from the script; negative / higher bits rejected)
inline constexpr bool ValidStates(int64_t aMask) { return aMask >= 0 && (static_cast<uint64_t>(aMask) & ~uint64_t(kStatesValid)) == 0; }

// ------------------------------------------------------------------------------------------------ full body

// First condition that keeps the full body off (kWhyOn = every condition holds). Names: WhyName.
enum Why : uint8_t
{
    kWhyOn = 0,
    kWhyFlag,    // script flag or user setting off
    kWhyMelee,   // no melee mask reason / upper body not on the game pose (and no stage-2 guard)
    kWhyOwner,   // v1 does not own the pose, not third person, or a release reason
    kWhyStance,  // crouch / sprint / unknown stance
    kWhyDetail,  // LocomotionDetailed: jump, fall, land, slide, dodge, climb, ladder, crouch, sprint...
    kWhyAir,     // ungrounded >= kAirSeconds (air feed) or, without an air feed, |vz| >= maxVertical
    kWhySpeed,   // stick held (same feed) or sustained capsule speed
    kWhyState,   // PSM.MeleeWeapon state without third-person clips (not in the allowed-state mask)
};

inline const char* WhyName(Why aWhy)
{
    switch (aWhy)
    {
    case kWhyOn: return "on";
    case kWhyFlag: return "flag";
    case kWhyMelee: return "melee";
    case kWhyOwner: return "owner";
    case kWhyStance: return "stance";
    case kWhyDetail: return "detail";
    case kWhyAir: return "air";
    case kWhySpeed: return "speed";
    case kWhyState: return "state";
    }
    return "?";
}

struct Input
{
    bool flag = false;        // script flag AND user setting
    bool meleeMask = false;   // (mask reasons & kMaskStateMelee) && upper body from the game
    bool guard = false;       // stage 2: the guard holds the upper body AND melee.guardFullBody is on (adds Idle 2 only)
    int32_t meleeState = -1;  // PSM.MeleeWeapon of the feed (-1 = not reported -> not allowed)
    uint32_t allowedStates = kFullBodyDefaultStates; // UTPPSetMeleeFullBodyStates
    bool owner = false;       // v1 owns the feed pose && TPP && no release reason
    int32_t stance = 0;       // conditioned Locomotion state (1 stand, 2 crouch, 3 sprint)
    int32_t psmDetail = -1;   // PSM LocomotionDetailed (-1 = not reported)
    bool stickHeld = false;   // move intent (steer::StickFilter held, or |move| > its press threshold with the filter off)
    bool haveAir = false;     // the air feed of this frame arrived
    bool onGround = true;
    float vz = 0.0f;          // m/s world vertical velocity (judged only without an air feed)
    float maxVertical = 1.0f; // m/s (GateDebounceConfig::maxVertical, the mask's own limit)
    float planarSpeed = 0.0f; // m/s capsule
    float dt = 0.0f;          // s since the previous feed (timers; capped at kMaxStepDt)
};

// LocomotionDetailed values that still count as standing: not reported (-1), NotInBaseLocomotion 0, Stand 1 (also walking /
// jogging), AimWalk 2 (fists block / targeting raise AimWalk). Everything else (Crouch 3, Sprint 4, Slide 5, Dodge 7, Climb 8,
// Vault 9, ladder 10..13, air 14..22, lands 23..28, Knockdown 29, CrouchSprint 30, Felled 31) keeps the legs on MM. Which melee
// states may take the full body is decided by the allowed-state mask (block / targeting AimWalk only once their clips exist).
inline constexpr bool StandingDetail(int32_t aDetail) { return aDetail >= -1 && aDetail <= 2; }

// Air rule shared by the full body and the guard: with the air feed of the frame ungrounded for kAirSeconds (debounced, the
// timer is the caller's), without it |vz| >= maxVertical (or a non-finite vz).
inline bool AirStep(bool aHaveAir, bool aOnGround, float aVz, float aMaxVertical, float aDt, float& aAirT)
{
    if (aHaveAir)
    {
        aAirT = aOnGround ? 0.0f : aAirT + aDt;
        return aAirT >= kAirSeconds;
    }
    return !std::isfinite(aVz) || !(std::fabs(aVz) < aMaxVertical);
}

inline float StepDt(float aDt) { return (std::isfinite(aDt) && aDt > 0.0f) ? (aDt < kMaxStepDt ? aDt : kMaxStepDt) : 0.0f; }

class Gate
{
public:
    void Reset()
    {
        m_fastT = 0.0f;
        m_dipT = 0.0f;
        m_slowT = 0.0f;
        m_fastOff = false;
        m_airT = 0.0f;
        m_why = kWhyFlag;
        m_standing = false;
    }
    // Once per feed. The speed and air timers are tracked on every call, whatever the other conditions say.
    bool Update(const Input& aIn)
    {
        const float dt = StepDt(aIn.dt);
        const float s = aIn.planarSpeed;
        const bool fast = !std::isfinite(s) || s > kOffSpeed; // a non-finite speed counts as fast (sustained rule only)
        const bool slow = std::isfinite(s) && s < kOnSpeed;
        // fast time with a short peak hold: a dip below kOffSpeed shorter than kBridgeSeconds keeps the accumulated time (the
        // dip itself does not count); a longer dip clears it
        if (fast)
        {
            m_fastT += dt;
            m_dipT = 0.0f;
        }
        else
        {
            m_dipT += dt;
            if (m_dipT >= kBridgeSeconds)
                m_fastT = 0.0f;
        }
        m_slowT = slow ? m_slowT + dt : 0.0f;
        if (m_fastOff && m_slowT >= kBridgeSeconds)
            m_fastOff = false; // below kOnSpeed for the bridge time (one slow feed of an alternation does not end the stop)
        else if (m_fastT >= kSustainSeconds)
            m_fastOff = true;
        const bool air = AirStep(aIn.haveAir, aIn.onGround, aIn.vz, aIn.maxVertical, dt, m_airT);
        const bool stateOk = StateAllowed(aIn.allowedStates, aIn.meleeState) || (aIn.guard && aIn.meleeState == kMwIdle);
        // standing and on the ground, evaluated on its own (m_why only keeps the FIRST failing reason, so Stance / Detail / Air are not
        // visible there when the flag, the melee mask, the state or the owner fails first): the chest base correction reads this
        m_standing = aIn.stance == kStanceStand && StandingDetail(aIn.psmDetail) && !air;
        m_why = !aIn.flag                        ? kWhyFlag
                : !(aIn.meleeMask || aIn.guard)  ? kWhyMelee
                : !stateOk                       ? kWhyState
                : !aIn.owner                     ? kWhyOwner
                : aIn.stance != kStanceStand     ? kWhyStance
                : !StandingDetail(aIn.psmDetail) ? kWhyDetail
                : air                            ? kWhyAir
                : aIn.stickHeld || m_fastOff     ? kWhySpeed
                                                 : kWhyOn;
        return m_why == kWhyOn;
    }
    Why LastWhy() const { return m_why; }
    bool StandingGrounded() const { return m_standing; } // stance Stand + standing LocomotionDetailed + grounded, whatever else holds
    float FastSeconds() const { return m_fastT; } // s of fast time (dips < kBridgeSeconds bridged)
    bool FastOff() const { return m_fastOff; }    // sustained-speed stop active (until below kOnSpeed for kBridgeSeconds)
    float AirSeconds() const { return m_airT; }   // s ungrounded (air feed)

private:
    float m_fastT = 0.0f;
    float m_dipT = 0.0f;  // s continuously at or below kOffSpeed
    float m_slowT = 0.0f; // s continuously below kOnSpeed
    bool m_fastOff = false;
    float m_airT = 0.0f;
    Why m_why = kWhyFlag;
    bool m_standing = false;
};

// Fade-out time of the lower body (2026-11-12): the legs leave the game pose slowly (eased kReleaseFadeSeconds) only when the
// melee state itself ended (Why state / melee) after a melee WEAPON (aLongRelease = MeleeGate::LongRelease); every other reason
// (stick, speed, air, detail, stance, owner, flag) keeps the quick kFadeSeconds - walking off must bring the legs back fast. With
// the 0.55 s mask hold the legs (0.35 s) are back on MM before the arms start to fade out: no leg / arm pop in the same moment.
inline float LegFadeOutSeconds(Why aWhy, bool aLongRelease)
{
    return (aLongRelease && (aWhy == kWhyState || aWhy == kWhyMelee)) ? kReleaseFadeSeconds : kFadeSeconds;
}

// ------------------------------------------------------------------------------------------------ guard

// PSM.MeleeWeapon states that keep the guard: Idle 2 and Parried..EquipAttack 5..21 (attacks keep it, so the MeleeGate
// debounce at the strike start leaves no gap). Not: NotReady 0, Equipping 1 (melee_first_equip stays vanilla first person),
// Safe 3 / PublicSafe 4 (melee_safepose stays vanilla), not reported -1.
inline constexpr bool GuardMeleeState(int32_t aState) { return aState == kMwIdle || (aState >= kMwParried && aState <= kMwEquipAttack); }
// PSM.UpperBody (orphans.script:6663-6671): not reported -1, Default 0, Aim 6 keep it; SwitchItems 1 (draw / holster / swap),
// SwitchCyberware 2, Reload 3, TemporaryUnequip 4, ForceEmptyHands 5 end it.
inline constexpr bool GuardUpperBody(int32_t aUpper) { return aUpper == -1 || aUpper == 0 || aUpper == 6; }

enum GuardWhy : uint8_t
{
    kGuardOn = 0,
    kGuardFlag,   // script flag (guard-capable weapon drawn) or the user setting melee.guard off
    kGuardMask,   // upper-body mask off / not resolved
    kGuardOwner,  // v1 does not own the pose, not third person, or a release reason
    kGuardGate,   // effective gate closed (no AIR hold) or an exit-release PSM bit
    kGuardState,  // PSM.MeleeWeapon outside GuardMeleeState
    kGuardUpper,  // PSM.UpperBody outside GuardUpperBody
    kGuardStance, // conditioned stance not Stand (crouch, sprint, unknown)
    kGuardDetail, // LocomotionDetailed not standing (sprint, crouch, slide, dodge, jump, fall, land...)
    kGuardAir,    // ungrounded (same rule as the full body)
};

inline const char* GuardWhyName(GuardWhy aWhy)
{
    switch (aWhy)
    {
    case kGuardOn: return "on";
    case kGuardFlag: return "flag";
    case kGuardMask: return "mask";
    case kGuardOwner: return "owner";
    case kGuardGate: return "gate";
    case kGuardState: return "state";
    case kGuardUpper: return "upper";
    case kGuardStance: return "stance";
    case kGuardDetail: return "detail";
    case kGuardAir: return "air";
    }
    return "?";
}

struct GuardInput
{
    bool flag = false;        // UTPPSetMeleeGuard (script: guard-capable weapon drawn, UTPP third person on foot)
    bool setting = false;     // UTPPSetMeleeGuardSetting (melee.guard, default off)
    bool maskOk = false;      // upper-body mask on and resolved
    bool owner = false;       // v1 owns the feed pose && TPP && no release reason
    bool gateOpen = false;    // (effective gate 0 || AIR hold) && no exit-release PSM bit
    int32_t meleeState = -1;  // PSM.MeleeWeapon
    int32_t upperBody = -1;   // PSM.UpperBody
    int32_t stance = 0;       // conditioned Locomotion state
    int32_t psmDetail = -1;   // PSM LocomotionDetailed
    bool haveAir = false;
    bool onGround = true;
    float vz = 0.0f;
    float maxVertical = 1.0f;
    float dt = 0.0f;
};

class GuardGate
{
public:
    void Reset()
    {
        m_airT = 0.0f;
        m_why = kGuardFlag;
    }
    // Once per feed (the air timer is tracked on every call).
    GuardWhy Update(const GuardInput& aIn)
    {
        const bool air = AirStep(aIn.haveAir, aIn.onGround, aIn.vz, aIn.maxVertical, StepDt(aIn.dt), m_airT);
        m_why = !(aIn.flag && aIn.setting)          ? kGuardFlag
                : !aIn.maskOk                       ? kGuardMask
                : !aIn.owner                        ? kGuardOwner
                : !aIn.gateOpen                     ? kGuardGate
                : !GuardMeleeState(aIn.meleeState)  ? kGuardState
                : !GuardUpperBody(aIn.upperBody)    ? kGuardUpper
                : aIn.stance != kStanceStand        ? kGuardStance
                : !StandingDetail(aIn.psmDetail)    ? kGuardDetail
                : air                               ? kGuardAir
                                                    : kGuardOn;
        return m_why;
    }
    GuardWhy LastWhy() const { return m_why; }
    float AirSeconds() const { return m_airT; }

private:
    float m_airT = 0.0f;
    GuardWhy m_why = kGuardFlag;
};

// ------------------------------------------------------------------------------------------------ driver terms

// The terms the MMDriver Feed feeds into its fades. The guard folds into the existing ones (upper mask, full body, tilt); with
// guardWanted false every term equals the pre-guard expression (tests/MeleeFullBodyTest.cpp checks all input combinations).
struct DriverIn
{
    bool maskReasonsUpper = false; // g_maskReasons != 0 && g_maskUpperFromGame
    bool weaponMaskWanted = false; // mm.weaponUpper path
    bool aimFadeEligible = false;  // g_aimUpperFadeOn && g_maskFadeFromAim && g_maskReasons == 0
    bool meleeMask = false;        // melee mask reason with the upper body from the game, mask on and resolved
    bool guardWanted = false;      // GuardGate on
    bool guardFullSetting = false; // melee.guardFullBody
    bool guardTiltSetting = false; // melee.guardAimTilt
};

struct DriverTerms
{
    bool maskWanted = false;  // g_maskBlend wanted
    bool aimFadeLong = false; // mask fade 0.25 s (aim fade) instead of kMaskFadeSeconds
    bool fullMelee = false;   // Input::meleeMask of the full-body gate
    bool fullGuard = false;   // Input::guard of the full-body gate (stage 2)
    bool tiltWanted = false;  // g_meleeAimBlend wanted
};

inline DriverTerms CombineDriverTerms(const DriverIn& aIn)
{
    DriverTerms t;
    t.maskWanted = aIn.maskReasonsUpper || aIn.weaponMaskWanted || aIn.guardWanted;
    t.aimFadeLong = aIn.aimFadeEligible && !aIn.weaponMaskWanted && !aIn.guardWanted;
    t.fullMelee = aIn.meleeMask;
    t.fullGuard = aIn.guardWanted && aIn.guardFullSetting;
    t.tiltWanted = aIn.meleeMask || (aIn.guardWanted && aIn.guardTiltSetting);
    return t;
}

} // namespace utpp::mm::melee

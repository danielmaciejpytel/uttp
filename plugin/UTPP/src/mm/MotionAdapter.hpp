#pragma once
// Adapter between the existing v1 feed (MMDriver::Feed, utpp.reds
// LocomotionEventsTransition.OnUpdate) and the v2 modules (ChooserMatcher + chooser, QuinticInertializer, overlay layers).
// NOT included by the DLL; compile-checked only by tests/MotionAdapterCompileTest.cpp. Every threshold here is a placeholder [P]
// until the merge plan (6.10.8) confirms it in game.
//
// Frame (game thread, inside MMDriver::Feed when the effective version is 2):
//   MotionFeed (v1 FeedInput + PSM extras) -> MapToChooser -> ChooserMatcher::Update (chooser + search + blend request)
//   -> SamplePose(clip, time) -> QuinticInertializer (Request on blend) -> [overlay ComposeLayers + Additive, 10.6]
//   -> existing OffsetRoot / StanceGuard / FootLocker (v1 modules, pose span) -> PublishLocked (same PoseSnapshot)
// FK detour (animation thread): unchanged BlendIntoRecords + upper-body mask + aimofs::Apply.
// RTZ (root vertical of the jump/land/traversal clips) is a FEATURE only: the capsule (game physics, runtime AirState)
// owns the entity height; the visual root vertical relative to the capsule stays 0 (report 6.6.4).
//
// Correction 2026-10-13 (verifier + lead / runtime session agreement, report 6.10):
// - v2 does NOT open the hard gate and has NO release set of its own. It runs in the existing 0.6.2 hold path:
//   integ::HoldEligible -> g_effGate = 0, weight 1, matcher kept running (v1: locoState = g_lastGoodLoco). "Hold-eligible"
//   counts as open for v2. The release details are the runtime constant (today integ::PsmDetailReleases {10-13, 29, 31},
//   to be moved into constexpr kPsmReleaseDetails in GameIntegration.hpp by the runtime session); 23-25 must stay open.
// - No UTPPFeedLocomotionExt: LocomotionDetailed / UpperBody come from the existing psmBlock packing (bits 16..23 / 13..15),
//   air inputs from the runtime session's UTPPFeedAir(onGround, distanceToGround or -1, psmFall).

#include <cmath>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "../MutationSelect.hpp"
#include "GameIntegration.hpp"
#include "Inertializer.hpp"
#include "MiniJson.hpp"
#include "ChooserMatcher.hpp"
#include "PlaybackRate.hpp"
#include "StanceCalibration.hpp"
#include "StationaryShotTurn.hpp"
#include "MotionTrajectory.hpp"

namespace utpp::mm::adapter
{
// ---------------------------------------------------------------- version switch (CET mm.version, UTPPSetMMVersion)
enum class Version : uint8_t
{
    V1 = 1,
    V2 = 2,
};

inline bool ParseVersion(std::string_view s, Version& v)
{
    if (s == "1" || s == "base")
        v = Version::V1;
    else if (s == "2" || s == "motion")
        v = Version::V2;
    else
        return false;
    return true;
}

// ---------------------------------------------------------------- body / data selection
enum class Body : uint8_t
{
    Unknown = 0,
    Woman = 1, // player_woman_skeleton, woman_base
    Man = 2,   // player_man_skeleton, man_base
};

inline uint64_t ExpectedRigIdentity(Body b)
{
    return b == Body::Woman ? motion::kRigIdentityWoman : (b == Body::Man ? motion::kRigIdentityMan : 0ull);
}

struct DataPaths
{
    std::string db;      // <dir>\data\utpp_motion_<body>.utppmm
    std::string chooser; // <dir>\data\utpp_motion_<body>.chooser.json (local data)
    std::string overlay; // <dir>\data\utpp_overlays_<body>.utppov
    std::string style;   // <dir>\data\utpp_style_weights_<body>.txt
};

inline DataPaths PathsFor(Body b, std::string_view aDir)
{
    DataPaths p;
    if (b == Body::Unknown)
        return p;
    const std::string body = b == Body::Woman ? "woman" : "man";
    std::string dir(aDir);
    if (!dir.empty() && dir.back() != '\\' && dir.back() != '/')
        dir += '\\';
    dir += "data\\";
    p.db = dir + "utpp_motion_" + body + ".utppmm";
    p.chooser = dir + "utpp_motion_" + body + ".chooser.json";
    p.overlay = dir + "utpp_overlays_" + body + ".utppov";
    p.style = dir + "utpp_style_weights_" + body + ".txt";
    return p;
}

// FNV-1a 64 of the bone name = RED4ext CName hash. Lets the runtime compare the live rig (name hashes + parents read at arm,
// ReadRigNamesAndParents) with the database bones without strings on the game side.
inline uint64_t CNameHash(std::string_view s)
{
    uint64_t h = 0xCBF29CE484222325ull;
    for (const char c : s)
    {
        h ^= static_cast<uint8_t>(c);
        h *= 0x100000001B3ull;
    }
    return h;
}

// Topology of database bones 0..n-1 found in the live rig (same name hash; parent name hash equal). 0 = all match.
inline uint32_t TopologyMismatches(const motion::MotionDatabase& aDb, std::span<const uint64_t> aLiveHashes,
                                   std::span<const int16_t> aLiveParents)
{
    uint32_t bad = 0;
    for (uint32_t i = 0; i < aDb.NumBones(); ++i)
    {
        const uint64_t h = CNameHash(aDb.BoneName(i));
        int32_t live = -1;
        for (size_t k = 0; k < aLiveHashes.size(); ++k)
            if (aLiveHashes[k] == h)
            {
                live = static_cast<int32_t>(k);
                break;
            }
        if (live < 0)
        {
            ++bad;
            continue;
        }
        const int32_t dp = aDb.BoneParent(i);
        const int32_t lp = static_cast<size_t>(live) < aLiveParents.size() ? aLiveParents[static_cast<size_t>(live)] : -1;
        const uint64_t dph = dp >= 0 ? CNameHash(aDb.BoneName(static_cast<uint32_t>(dp))) : 0ull;
        const uint64_t lph = (lp >= 0 && static_cast<size_t>(lp) < aLiveHashes.size()) ? aLiveHashes[static_cast<size_t>(lp)] : 0ull;
        if (dph != lph)
            ++bad;
    }
    return bad;
}

// ---------------------------------------------------------------- fallbacks (fail closed to v1)
enum class Fallback : uint8_t
{
    None = 0,       // v2 active
    NotRequested,   // mm.version = 1
    BodyUnknown,    // no body reported yet (script / CET) -> v1 until known
    DbMissing,      // file not found
    DbInvalid,      // CRC / format / version
    RigMismatch,    // database rig identity != expected for the body, or live topology mismatch
    ChooserMissing, // chooser table file not found
    ChooserInvalid, // chooser parse / database names not in the file
    MatcherInit,    // ChooserMatcher::Init failed
    BoneMap,        // bone map against the armed rig failed
    Budget,         // cost budget tripped (see Budget)
    Fault,          // non-finite pose / exception in the v2 path
};

inline const char* FallbackName(Fallback f)
{
    switch (f)
    {
    case Fallback::None: return "none";
    case Fallback::NotRequested: return "not-requested";
    case Fallback::BodyUnknown: return "body-unknown";
    case Fallback::DbMissing: return "db-missing";
    case Fallback::DbInvalid: return "db-invalid";
    case Fallback::RigMismatch: return "rig-mismatch";
    case Fallback::ChooserMissing: return "chooser-missing";
    case Fallback::ChooserInvalid: return "chooser-invalid";
    case Fallback::MatcherInit: return "matcher-init";
    case Fallback::BoneMap: return "bone-map";
    case Fallback::Budget: return "budget";
    case Fallback::Fault: return "fault";
    }
    return "?";
}

struct ResolveInputs
{
    Version requested = Version::V1;
    Body body = Body::Unknown;
    bool dbFile = false;
    bool dbParsed = false;
    uint64_t dbRigIdentity = 0;
    uint32_t topologyMismatches = 0; // TopologyMismatches against the armed rig (0 when not armed yet is NOT ok: pass ~0u)
    bool chooserFile = false;
    bool chooserParsed = false;
    bool matcherInit = false;
    bool boneMapOk = false;
    bool budgetTripped = false;
    bool faulted = false;
};

// First failing condition in load order; v1 keeps running for every value except None.
inline Fallback Resolve(const ResolveInputs& r)
{
    if (r.requested != Version::V2)
        return Fallback::NotRequested;
    if (r.body == Body::Unknown)
        return Fallback::BodyUnknown;
    if (!r.dbFile)
        return Fallback::DbMissing;
    if (!r.dbParsed)
        return Fallback::DbInvalid;
    if (r.dbRigIdentity != ExpectedRigIdentity(r.body) || r.topologyMismatches != 0)
        return Fallback::RigMismatch;
    if (!r.chooserFile)
        return Fallback::ChooserMissing;
    if (!r.chooserParsed)
        return Fallback::ChooserInvalid;
    if (!r.matcherInit)
        return Fallback::MatcherInit;
    if (!r.boneMapOk)
        return Fallback::BoneMap;
    if (r.faulted)
        return Fallback::Fault;
    if (r.budgetTripped)
        return Fallback::Budget;
    return Fallback::None;
}

inline Version Effective(Fallback f) { return f == Fallback::None ? Version::V2 : Version::V1; }

// ---------------------------------------------------------------- PSM (gamePSMDetailedLocomotionStates, report 5.3)
enum PsmDetailed : int32_t
{
    kPsmStand = 1, kPsmAimWalk = 2, kPsmCrouch = 3, kPsmSprint = 4,
    kPsmSlide = 5, kPsmSlideFall = 6, kPsmDodge = 7, kPsmClimb = 8, kPsmVault = 9,
    kPsmLadderFirst = 10, kPsmLadderLast = 13,
    kPsmFall = 14, kPsmAirThrusters = 15, kPsmAirHover = 16, kPsmSuperheroFall = 17,
    kPsmJump = 18, kPsmDoubleJump = 19, kPsmChargeJump = 20, kPsmHoverJump = 21, kPsmDodgeAir = 22,
    kPsmRegularLand = 23, kPsmHardLand = 24, kPsmVeryHardLand = 25, kPsmDeathLand = 26, kPsmSuperheroLand = 27,
    kPsmRecovery = 28, kPsmKnockdown = 29, kPsmCrouchSprint = 30, kPsmFelled = 31,
};

inline bool PsmGround(int32_t s) { return s == kPsmStand || s == kPsmAimWalk || s == kPsmCrouch || s == kPsmSprint || s == kPsmCrouchSprint; }
// motionfix item 7: CrouchSprint 30 (gamePSMDetailedLocomotionStates, the sprint started from the crouch: the game stands V
// up and runs her at the sprint speed, feed dump 2026-10-13 loco 3 / detail 30) maps to Stand + Sprint, not to the crouch
// stance (motion has no crouch sprint: Crouch x Moving selects the crouch walk databases, 225 cm/s cap).
#if defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 3 // mutation 3: CrouchSprint = crouch stance, no sprint
inline bool PsmSprint(int32_t s) { return s == kPsmSprint; }
inline bool PsmCrouchStance(int32_t s) { return s == kPsmCrouch || s == kPsmCrouchSprint; }
#else
inline bool PsmSprint(int32_t s) { return s == kPsmSprint || s == kPsmCrouchSprint; }
inline bool PsmCrouchStance(int32_t s) { return s == kPsmCrouch; }
#endif
inline bool PsmAir(int32_t s) { return s == kPsmFall || s == kPsmJump || s == kPsmSlideFall || (s >= kPsmDoubleJump && s <= kPsmDodgeAir) || (s >= kPsmAirThrusters && s <= kPsmSuperheroFall); }
inline bool PsmSlide(int32_t s) { return s == kPsmSlide; }
inline bool PsmLand(int32_t s) { return s == kPsmRegularLand || s == kPsmHardLand || s == kPsmVeryHardLand; }
inline bool PsmTraversal(int32_t s) { return s == kPsmClimb || s == kPsmVault; }
// Correction 2026-10-13: no v2 release set. The body goes back to the game only where the v1 hold path releases it: the
// same runtime constant (integ::PsmDetailReleases today, kPsmReleaseDetails after the runtime session's move) plus the
// kPsmReleaseMask bits. Slide 5, Dodge 7, DeathLand 26, SuperheroLand 27, Recovery 28 stay held (MM) until in-game evidence
// adds them to that constant; AimWalk 2, SlideFall 6, Climb 8, Vault 9, air 14-22, lands 23-25 and CrouchSprint 30 stay held.
inline bool PsmReleasedToGame(int32_t s) { return integ::PsmDetailReleases(s); }

// v1 hold path, read by the v2 frame: the gate the weight follows (g_effGate) is 0 when the gate is open OR the closure is
// held (integ::HoldEligible). v2 runs exactly when v1 would run its matcher; it never changes the gate.
inline bool MotionRuns(uint32_t aEffGate) { return aEffGate == 0; }

// Existing psmBlock packing (scripts/utpp.reds UTPPPsmBlockMask): LocomotionDetailed + 1 in bits 16..23 ONLY together with
// kPsmDetailed (bit 6, i.e. detail not Stand 1 / Crouch 3 / Sprint 4), UpperBody + 1 in bits 13..15. Field 0 -> the detail is
// one of 1 / 3 / 4: taken from the v1 locoState (UTPPLocoStateId: 1 stand, 2 crouch, 3 sprint; other -> Stand).
inline int32_t DetailFromFeed(uint32_t aPsmBlockRaw, int32_t aLocoState)
{
    const int d = integ::PsmDetailOf(aPsmBlockRaw);
    if (d >= 0)
        return d;
    return aLocoState == integ::kLocoCrouch ? kPsmCrouch : (aLocoState == integ::kLocoSprint ? kPsmSprint : kPsmStand);
}
inline constexpr int kUpperBodyAim = 6; // gamePSMUpperBodyStates.Aim
inline bool AimingFromFeed(uint32_t aPsmBlockRaw) { return integ::PsmUpperBodyOf(aPsmBlockRaw) == kUpperBodyAim; }

// Runtime session's UTPPFeedAir(onGround, distanceToGround or -1, psmFall), sent in UTPPFeedFromLocomotion right before
// UTPPFeedLocomotion (same frame). valid = false until the native exists / was called this frame (v2 then uses the PSM
// detail only and ttlNoHit).
struct AirFeed
{
    bool valid = false;
    bool onGround = true;
    float distanceToGround = -1.0f; // m; -1 = not queried / no hit
    int32_t psmFall = 0;            // PlayerStateMachine.Fall (gamePSMFallStates 1 Regular .. 5 Death; 0 none)
};

// v1 FeedInput fields used by v2 (velWorld, move, dt, cameraYawDeg, playerYawDeg, psmBlock raw -> detail / aiming,
// locoState) plus AirFeed. No new locomotion native (correction 2026-10-13: UTPPFeedLocomotionExt dropped).
struct MotionFeed
{
    float velWorld[3] = {0, 0, 0}; // m/s, z up (velWorld[2] = vertical velocity, already in the v1 feed)
    float move[2] = {0, 0};        // MoveX (right) / MoveY (forward), camera space
    float dt = 0.0f;
    float cameraYawDeg = 0.0f;     // v1 FeedInput.cameraYawDeg (control rotation yaw; integ yaw convention, degrees)
    float playerYawDeg = 0.0f;     // v1 FeedInput.playerYawDeg (actor yaw = motion CharacterTransform rotation)
    int32_t psmDetailed = 0;      // DetailFromFeed(psmBlock raw, locoState)
    bool aiming = false;          // AimingFromFeed(psmBlock raw)
    bool haveWeapon = false, weaponDrawn = false; // live WeaponRight diagnostic; absent = legacy feed
    // motion CharacterInputState.WantsToWalk (CBP IA_Walk toggle l. 865-866) = the game's walk toggle: MotionDriver Step sets it
    // from FeedInput.walkToggled (PSM WalkToggled, batch 1b; false without FeedInput.haveExtra). Not set by MakeMotionFeed.
    // WantsToSprint = PSM Sprint detail (GaitFromInput; sprint wins over walk).
    bool wantsToWalk = false;
    // GameplayRestriction.Tier2LocomotionSlow is active (PSM HighLevel SceneTier2; scripts/utpp.reds UTPPFeedMotionTier2Slow, MotionDriver Step).
    // The game adds -1.3 m/s to MaxSpeed (kTier2SlowDeltaMs), so the stand run's full-stick speed is 3.85 - 1.3 = 2.55 m/s: MapToChooser lowers the
    // run cap / analog modifier accordingly (EffectiveMapConfig). false = nothing changes, bit for bit. Not set by MakeMotionFeed.
    bool tier2Slow = false;
    // G9: the aim jog gives no target (UTPPMMAimJogTarget answered 0: CET bodyturn.aimJog off, strafe off / not Active), so the AimWalk detail keeps the vanilla aim walk
    // speed (player_locomotion_data_AimWalk, measured capsule 1.63-1.67 m/s ~ the 1.6 m/s walk point): MapToChooser then asks for the Walk
    // gait in AimWalk (2) like the walk toggle, instead of Run strafe loops played at the rate floor. false = nothing changes, bit for bit.
    // Set by MotionDriver Step, not by MakeMotionFeed.
    bool aimWalkVanilla = false;
    AirFeed air;                  // UTPPFeedAir (runtime session)
    // UTPP-MOTION SEAM motionfix (item 1): v1 FeedInput haveTarget / targetYawDeg = world run direction (camera yaw + Locomotion raw
    // stick angle, or player yaw + stick angle without Locomotion: scripts/utpp.reds UTPPOwnMoveTarget).
    // Under Locomotion the game rewrites the stick to (0, |s|) (utpp.reds l. 228), so move[] alone is NOT a camera-space
    // direction there (in-game v2 test 2026-10-13, feed t 1458: mv (0, 1), cam -82.2, target 97.8 = velocity yaw 97.9).
    // MotionDriver Step sets them from FeedInput; not set by MakeMotionFeed (false: camera-relative move[] as before).
    bool haveTarget = false;
    float targetYawDeg = 0.0f;    // integ yaw convention, degrees (forward(psi) = (-sin psi, cos psi))
};

inline MotionFeed MakeMotionFeed(const float aVelWorld[3], const float aMove[2], float aDt, uint32_t aPsmBlockRaw, int32_t aLocoState,
                         const AirFeed& aAir, float aCameraYawDeg = 0.0f, float aPlayerYawDeg = 0.0f)
{
    MotionFeed f;
    for (int i = 0; i < 3; ++i)
        f.velWorld[i] = aVelWorld[i];
    f.move[0] = aMove[0];
    f.move[1] = aMove[1];
    f.dt = aDt;
    f.cameraYawDeg = aCameraYawDeg;
    f.playerYawDeg = aPlayerYawDeg;
    f.psmDetailed = DetailFromFeed(aPsmBlockRaw, aLocoState);
    f.aiming = AimingFromFeed(aPsmBlockRaw);
    f.haveWeapon = integ::PsmWeaponKnown(aPsmBlockRaw);
    f.weaponDrawn = integ::PsmWeaponDrawn(aPsmBlockRaw);
    f.air = aAir;
    return f;
}

// Stick (MoveX right, MoveY forward, camera space) -> world planar vector, NOT normalised (keeps the analog magnitude).
// Exactly the v1 convention of BodyTurn.hpp LegacyDesiredMotion (l. 90-92: wx = -sin(cam) my + cos(cam) mx,
// wy = cos(cam) my + sin(cam) mx; cam in radians, integ yaw convention: forward(psi) = (-sin psi, cos psi), right(psi) =
// (cos psi, sin psi), GameIntegration.hpp header). Stick forward with camera yaw 90 -> world (-1, 0).
inline void StickToWorld(float aMx, float aMy, float aCameraYawDeg, float& aWx, float& aWy)
{
    const float cam = (std::isfinite(aCameraYawDeg) ? aCameraYawDeg : 0.0f) * integ::kDegToRad;
    const float mx = std::isfinite(aMx) ? aMx : 0.0f, my = std::isfinite(aMy) ? aMy : 0.0f;
    aWx = -std::sin(cam) * my + std::cos(cam) * mx;
    aWy = std::cos(cam) * my + std::sin(cam) * mx;
}

// UTPP-MOTION SEAM motionfix (item 1): world planar input vector of the feed, NOT normalised. haveTarget (finite targetYawDeg):
// direction = targetYawDeg, magnitude = |move| (the analog stick size; Locomotion sends (0, |s|)); otherwise (strafe / aim, no
// target: move[] carries the real camera-space direction) StickToWorld(move, cameraYawDeg).
// UTPP_ADAPTER_FIX_MUTATION=1 (mutation builds only, undefined / 0 = production): the target is ignored (pre-fix input).
#ifndef UTPP_ADAPTER_FIX_MUTATION
#define UTPP_ADAPTER_FIX_MUTATION 0
#endif
inline void InputWorld(const MotionFeed& f, float& aWx, float& aWy)
{
    if (UTPP_ADAPTER_FIX_MUTATION != 1 && f.haveTarget && std::isfinite(f.targetYawDeg))
    {
        const float mx = std::isfinite(f.move[0]) ? f.move[0] : 0.0f, my = std::isfinite(f.move[1]) ? f.move[1] : 0.0f;
        const float m = std::sqrt(mx * mx + my * my);
        const float t = f.targetYawDeg * integ::kDegToRad;
        aWx = -std::sin(t) * m;
        aWy = std::cos(t) * m;
        return;
    }
    StickToWorld(f.move[0], f.move[1], f.cameraYawDeg, aWx, aWy);
}

// [P] stick magnitude at or below which the input counts as released (Acceleration == 0). This only swallows residual noise: the
// game moves the capsule below the old 0.1 (feed 2026-10-31 17:18: |mv| 0.091 -> 0.23-0.25 m/s, while v2 chose Idles for ~3 s because
// the input counted as released) and reports exactly 0 at rest, so the stop detection (IsZero(acc)) is unchanged. 0.02 = the
// removal threshold of the minimum analog walk multiplier (utpp.reds UTPPUpdateAnalogFloor, |mv|^2 <= 0.0004).
#ifndef UTPP_STICK_RELEASED_MUTATION
#define UTPP_STICK_RELEASED_MUTATION 0
#endif
inline constexpr float kStickReleased = UTPP_STICK_RELEASED_MUTATION == 1 ? 0.1f : 0.02f; // mutation 1: the pre-fix 0.1

// motion E_AnalogStickBehavior (CBP MovementStickMode, default FixedSpeed-SingleGait: CBP pseudo l. 5).
enum class StickMode : int
{
    FixedSpeedSingleGait = 0,
    FixedSpeedWalkRun = 1,
    VariableSpeedSingleGait = 2,
    VariableSpeedWalkRun = 3,
};
inline bool StickModeFixedSpeed(StickMode s) { return s == StickMode::FixedSpeedSingleGait || s == StickMode::FixedSpeedWalkRun; }
inline bool StickModeSingleGait(StickMode s) { return s == StickMode::FixedSpeedSingleGait || s == StickMode::VariableSpeedSingleGait; }

// motion Acceleration = CharacterMovement GetCurrentAcceleration = ScaleInputAcceleration(input) = GetMaxAcceleration() *
// input.GetClampedToMaxSize(1) (UE CharacterMovementComponent), input = AddMovementInput(control-yaw right, X) +
// (control-yaw forward, Y) with ScaleValue = GetMovementInputScaleValue (CBP IA_Move l. 840-841): Normal2D(input) in the
// FixedSpeed modes, raw input in the VariableSpeed modes (CBP l. 1118-1119). World cm/s^2 in the game world frame (z = 0).
inline motiontraj::V3 InputAcceleration(const MotionFeed& f, float aMaxAcceleration, StickMode aMode = StickMode::FixedSpeedSingleGait)
{
    float wx = 0.0f, wy = 0.0f;
    InputWorld(f, wx, wy); // UTPP-MOTION SEAM motionfix (item 1): Locomotion target direction when present
    const float m = std::sqrt(wx * wx + wy * wy);
    if (!(m > kStickReleased))
        return {};
    const float k = ((StickModeFixedSpeed(aMode) || m > 1.0f) ? 1.0f / m : 1.0f) * aMaxAcceleration;
    return {wx * k, wy * k, 0.0f};
}

// CBP GetDesiredGait (pseudo l. 1171-1187), 0 Walk / 1 Run / 2 Sprint. FullMovementInput: SingleGait modes true, Walk/Run
// modes |input| >= AnalogWalk/RunThreshold (l. 1174). CanSprint (l. 1377-1378): WantsToSprint && (bOrientRotationToMovement ||
// |NormalizedDelta(actor yaw, input yaw)| < 50); WantsToSprint = game PSM Sprint detail [P]. aInputYawDeg: yaw of the pending
// input vector in the integ convention (0 for no input, like Conv_VectorToRotator of a zero vector).
// G3 (MapConfig::gaitHysteresis, default OFF = motion, a single threshold): aPrevGait >= 0 and aHystUp > 0 switch the Walk/Run/Run split
// of the Walk/Run stick modes to a hysteresis around aWalkRunThreshold - from Walk to Run at stick >= aHystUp, from Run (or Sprint) back
// to Walk below aHystDown - so a stick resting on the 0.7 threshold does not flip the gait (and the database) every frame. The
// default arguments (-1, 0, 0) are the plain threshold, bit for bit. SingleGait modes are always "full": no hysteresis.
inline int GaitFromInput(bool aWantsToSprint, bool aWantsToWalk, float aStick, StickMode aMode, float aWalkRunThreshold,
                         bool aOrientToMovement, float aActorYawDeg, float aInputYawDeg, int aPrevGait = -1,
                         float aHystUp = 0.0f, float aHystDown = 0.0f)
{
    bool full = StickModeSingleGait(aMode) ? true : aStick >= aWalkRunThreshold;
    if (!StickModeSingleGait(aMode) && aPrevGait >= 0 && aHystUp > 0.0f)
        full = aPrevGait >= 1 ? aStick >= aHystDown : aStick >= aHystUp;
    float dy = std::fmod(aActorYawDeg - aInputYawDeg, 360.0f);
    if (dy > 180.0f)
        dy -= 360.0f;
    else if (dy < -180.0f)
        dy += 360.0f;
    const bool canSprint = aWantsToSprint && (aOrientToMovement ? true : std::fabs(dy) < 50.0f);
    if (canSprint)
        return full ? 2 : 1;
    if (aWantsToWalk)
        return 0;
    return full ? 1 : 0;
}

// Game 2.31 measured speeds (m/s): the same values as SpeedCalibration.hpp kGameStandPoints / kGameCrouchPoint (that header
// is not included here: it pulls the v1 database).
inline constexpr float kGameStandPointsMotion[3] = {1.6f, 3.85f, 8.2f};
inline constexpr float kGameCrouchPointMotion = 2.7f;

// Game steady capsule speed vs stick |MoveX, MoveY| (stand, m/s). The 0.18 / 0.27 / 0.73 / 0.82 / 1.00 points are game 2.31
// medians of the steady MMTRACE samples of the 37 night sessions (the gait speed analysis script): |mv| 0.18 -> 0.30 (n 159), 0.27 -> 0.41 (n 25), 0.73 -> 1.94 (n 59), 0.82 -> 2.48 (n 170),
// 1.00 -> 3.84 (n 1785). Game 2.5 (feeds 30-31.10): the full stick 3.84 m/s is CONFIRMED, and the gap between 0.27 and 0.73 has a
// measured point, |mv| 0.455 -> 0.82 m/s (median, n 1650); it lies below the straight 0.27 -> 0.73 interpolation (1.03 m/s), so the
// ramp is convex there, not a step. One curve for stand and crouch: crouch |mv| 0.82 -> 1.76 m/s = 0.65 x 2.7, stand 0.82 -> 2.48 =
// 0.65 x 3.84. The speed varies with the stick inside each measured cluster. The 2.31 data had no steady samples between |mv| ~0.35
// and 0.73 (coarse 0.05 buckets: 0.35 n 7 median 0.52, 0.40 n 0, 0.60 n 0, 0.65 n 1 at 1.47, 0.80 n 1).
// [P] linear between the points and from (0, 0) (|mv| 0.16 -> 0.27 m/s measured, n 10, = linear); the 0.27 -> 0.455 and
// 0.455 -> 0.73 segments are still interpolations (they set AnalogInputModifier and the prediction cap for that stick range).
inline constexpr int kGameStickPointsMotion = 6;
inline constexpr float kGameStickMotion[kGameStickPointsMotion] = {0.18f, 0.27f, 0.455f, 0.73f, 0.82f, 1.0f};
inline constexpr float kGameStickSpeedMotion[kGameStickPointsMotion] = {0.30f, 0.41f, 0.82f, 1.94f, 2.48f, 3.84f};

// Fraction of the full-stick game speed at stick aStick (0..1) from the measured curve above.
inline float GameStickSpeedFraction(float aStick)
{
    if (!(aStick > 0.0f))
        return 0.0f;
    const float full = kGameStickSpeedMotion[kGameStickPointsMotion - 1];
    if (aStick >= kGameStickMotion[kGameStickPointsMotion - 1])
        return 1.0f;
    float s0 = 0.0f, v0 = 0.0f;
    for (int i = 0; i < kGameStickPointsMotion; ++i)
    {
        if (aStick <= kGameStickMotion[i])
            return (v0 + (kGameStickSpeedMotion[i] - v0) * (aStick - s0) / (kGameStickMotion[i] - s0)) / full;
        s0 = kGameStickMotion[i];
        v0 = kGameStickSpeedMotion[i];
    }
    return 1.0f;
}

// ---- Minimum analog walk speed (CET mm.minWalkSpeed; the game-side counterpart of motion's CMC MinAnalogWalkSpeed) ----
// A small stick tilt moves the capsule slower than any clip of the database (|mv| 0.09 -> 0.24 m/s, 0.27 -> 0.42 m/s; the slowest walk
// loop at the 0.5 rate floor is ~0.87 game m/s), so V glides. The script (utpp.reds UTPPUpdateAnalogFloor) raises the capsule with one
// MaxSpeed multiplier = AnalogWalkFloorMul: the fraction of the full-stick speed at the stick is lifted to aFracMin (= setting /
// full-stick stand run 3.84, 0.82 m/s -> 0.2135 = the |mv| 0.455 point). A fraction, not m/s: the same multiplier serves stand, crouch
// and the walk toggle, and the Shooter mode / walk toggle multipliers stay as they are (they multiply with it).
// The curve is kGameStickMotion extended with the measured point |mv| 0.09 -> 0.24 m/s (feed 2026-10-31 17-18-29, 0.23-0.25 m/s; the
// linear 0 -> 0.18 segment gives 0.15 there, x5.5 instead of x3.4). Only this function uses the extended curve: GameSteadySpeed /
// GameAnalogModifier keep kGameStickMotion (no change with the setting at 0).
inline constexpr int kAnalogFloorPoints = kGameStickPointsMotion + 1;
inline constexpr float kAnalogFloorStick[kAnalogFloorPoints] = {0.09f, 0.18f, 0.27f, 0.455f, 0.73f, 0.82f, 1.0f};
inline constexpr float kAnalogFloorSpeed[kAnalogFloorPoints] = {0.24f, 0.30f, 0.41f, 0.82f, 1.94f, 2.48f, 3.84f};
inline constexpr float kAnalogFloorMulMax = 4.0f;  // the |mv| < 0.09 range is not measured (linear from 0 there)
inline constexpr float kAnalogFloorMulMin = 1.03f; // below: 1 (the measured |mv| 0.45 cluster, just under 0.455, would keep ~1.01)
inline constexpr float kMinAnalogWalkMaxMs = 1.6f; // upper bound of the setting (CET mm.minWalkSpeed, SetMinAnalogWalk); 0 = off
#ifndef UTPP_ANALOGFLOOR_MUTATION
#define UTPP_ANALOGFLOOR_MUTATION 0
#endif

// Fraction of the full-stick game speed at stick aStick on the extended curve (linear from (0, 0) to the 0.09 point).
inline float AnalogFloorStickFraction(float aStick)
{
    if (!(aStick > 0.0f))
        return 0.0f;
    const float full = kAnalogFloorSpeed[kAnalogFloorPoints - 1];
    float s0 = 0.0f, v0 = 0.0f;
    for (int i = 0; i < kAnalogFloorPoints; ++i)
    {
        if (aStick <= kAnalogFloorStick[i])
            return (v0 + (kAnalogFloorSpeed[i] - v0) * (aStick - s0) / (kAnalogFloorStick[i] - s0)) / full;
        s0 = kAnalogFloorStick[i];
        v0 = kAnalogFloorSpeed[i];
    }
    return 1.0f;
}

// Fraction of the full-stick speed of a floor setting (game m/s, stand run): 0 for 0 / NaN / out of range (= off).
inline float AnalogFloorFraction(float aMinMs)
{
    if (!std::isfinite(aMinMs) || !(aMinMs > 0.0f) || aMinMs > kMinAnalogWalkMaxMs)
        return 0.0f;
    return aMinMs / kAnalogFloorSpeed[kAnalogFloorPoints - 1];
}

// MaxSpeed multiplier of the floor: clamp(aFracMin / frac(|stick|), 1, 4); exactly 1 for a released stick (<= kStickReleased), at and
// above the floor's stick, for a multiplier below kAnalogFloorMulMin, for aFracMin 0 (off) and for NaN input.
inline float AnalogWalkFloorMul(float aStick, float aFracMin, float aMulMax = kAnalogFloorMulMax)
{
    if (!std::isfinite(aStick) || !std::isfinite(aFracMin) || !(aFracMin > 0.0f) || !(aStick > kStickReleased))
        return 1.0f;
    const float frac = AnalogFloorStickFraction(std::fmin(aStick, 1.0f));
    if (!(frac > 0.0f))
        return 1.0f;
    const float mul = std::fmin(aFracMin / frac, aMulMax);
#if UTPP_ANALOGFLOOR_MUTATION == 1
    return mul > 1.0f ? mul : 1.0f; // mutation 1: no 1.03 threshold
#else
    return mul < kAnalogFloorMulMin ? 1.0f : mul;
#endif
}

// Inverse of AnalogFloorStickFraction (the same extended curve, both directions): the stick whose fraction of the full-stick speed is
// aFrac; 0 for aFrac <= 0 / NaN, 1 at and above 1.
inline float AnalogFloorStickForFraction(float aFrac)
{
    if (!std::isfinite(aFrac) || !(aFrac > 0.0f))
        return 0.0f;
    const float full = kAnalogFloorSpeed[kAnalogFloorPoints - 1];
    float s0 = 0.0f, v0 = 0.0f;
    for (int i = 0; i < kAnalogFloorPoints; ++i)
    {
        const float v = kAnalogFloorSpeed[i] / full;
        if (aFrac == v)
            return kAnalogFloorStick[i]; // exact point (0.82 -> 0.455): no rounding of the interpolation above it
        if (aFrac < v)
            return s0 + (kAnalogFloorStick[i] - s0) * (aFrac - v0) / (v - v0);
        s0 = kAnalogFloorStick[i];
        v0 = v;
    }
    return 1.0f;
}

// MapConfig::analogFloorStick of a floor setting: the stick whose fraction on the extended curve is the floor's (0.455 at 0.82 m/s);
// 0 when off.
inline float AnalogFloorStickFor(float aMinMs)
{
    return AnalogFloorStickForFraction(AnalogFloorFraction(aMinMs));
}

// Walk toggle: the floor is the same m/s, but the full-stick speed is the walk toggle speed (MapConfig::gameWalkTogglePoint, CET
// mm.walkSpeed), so its fraction is setting / walk speed (0.82 / 1.6 = 0.5125) instead of setting / 3.84 (0.2135 x 1.6 = only 0.34 m/s:
// V glided in Idle under G with a small tilt, 2026-10-31 22:57). Capped at 1 (a floor at or above the walk speed lifts every tilt to the
// full walk speed). 0 for a floor that is off (AnalogFloorFraction 0) or a walk speed <= 0 / NaN.
inline float AnalogFloorFractionWalk(float aMinMs, float aWalkFullMs)
{
    if (!(AnalogFloorFraction(aMinMs) > 0.0f) || !std::isfinite(aWalkFullMs) || !(aWalkFullMs > 0.0f))
        return 0.0f;
    return std::fmin(aMinMs / aWalkFullMs, 1.0f);
}
// The multiplier cap under the walk toggle: the same reachable m/s as without it (kAnalogFloorMulMax x the stick's own stand-run speed),
// so kAnalogFloorMulMax x 3.84 / walk speed (9.6 at 1.6: |mv| 0.15 needs ~7 to reach 0.82 m/s at the walk speed), at least
// kAnalogFloorMulMax, at most kAnalogFloorMulMaxWalk.
inline constexpr float kAnalogFloorMulMaxWalk = 10.0f;
inline float AnalogFloorMulMaxWalk(float aWalkFullMs)
{
    if (!std::isfinite(aWalkFullMs) || !(aWalkFullMs > 0.0f))
        return kAnalogFloorMulMax;
    return std::fmin(std::fmax(kAnalogFloorMulMax * kAnalogFloorSpeed[kAnalogFloorPoints - 1] / aWalkFullMs, kAnalogFloorMulMax),
                     kAnalogFloorMulMaxWalk);
}

// [P] motion CMC values for the game capsule: CBP CMC CDO (MotionTrajectory MotionCmc) except MinAnalogWalkSpeed (CDO 150 cm/s,
// UE PoseSearchTrajectoryLibrary.cpp l. 73 MaxSpeed = max(MaxSpeed x AnalogInputModifier, MinAnalogSpeed)). The game capsule
// moves steadily at 0.27-0.41 m/s (mapped 34-51 cm/s) at |mv| 0.16-0.27 (curve above), so a 150 cm/s floor would predict an
// acceleration the game never performs (IsStarting on every steady slow-walk frame); the floor is 0 and the cap comes from
// the calibrated AnalogInputModifier (GameAnalogModifier).
inline motiontraj::MotionCmc MotionGameCmc()
{
    motiontraj::MotionCmc c;
    c.minAnalogSpeed = 0.0f;
    return c;
}

// Shooter mode speed ratio by direction (motion ratios, sideways 70 %, backward 60 % of the forward run speed;
// walk / crouch by the motion StrafeSpeedMap, like the aim jog). aMoveX / aMoveY = body-relative stick (MoveX right, MoveY forward),
// aDetailed = PSM LocomotionDetailed. Exactly 1.0f for anything but Stand (1) / Crouch (3), a non-finite input and a released stick
// (|move| < 0.15); otherwise MotionMaxSpeed(strafe, direction) / MotionMaxSpeed(orient) of the gait the stick selects: stand run 1 / 0.70 /
// 0.60, walk 1 / 0.90 / 0.75, crouch 1 / 0.889 / 0.80 (linear in StrafeSpeedMap: 45 deg 0, 80-100 deg 1, 135-180 deg 2).
// aGait: the gait the chooser plays (0 walk, 1 run; the walk toggle makes it walk at full stick); -1 = from the stick size (>= 0.7 runs).
inline float ShooterModeSpeedRatio(float aMoveX, float aMoveY, int32_t aDetailed, int aGait = -1)
{
    if (!std::isfinite(aMoveX) || !std::isfinite(aMoveY) || (aDetailed != kPsmStand && aDetailed != kPsmCrouch))
        return 1.0f;
    const float mag = std::sqrt(aMoveX * aMoveX + aMoveY * aMoveY);
    if (!(mag >= 0.15f))
        return 1.0f;
    const float dir = std::fabs(std::atan2(-aMoveX, aMoveY)) * (180.0f / 3.14159265358979323846f);
    const bool crouch = aDetailed == kPsmCrouch;
    const int gait = crouch ? 1 : (aGait >= 0 ? (aGait == 0 ? 0 : 1) : (mag >= 0.7f ? 1 : 0));
    const motiontraj::MotionCmc cmc = MotionGameCmc();
    const float fwd = motiontraj::MotionMaxSpeed(cmc, gait, crouch ? 1 : 0, false, 0.0f);
    const float strafe = motiontraj::MotionMaxSpeed(cmc, gait, crouch ? 1 : 0, true, dir);
    return fwd > 0.0f && std::isfinite(strafe) ? strafe / fwd : 1.0f;
}

// ---------------------------------------------------------------- slide feed (sprint -> crouch; CET mm.slide)
// Game: PSM LocomotionDetailed 5 (kPsmSlide), started from a sprint: the capsule boosts +2.85 m/s within 0.1 s (8.25 -> ~11 m/s),
// then brakes linearly (median -2.75 m/s^2) down to ~2.73 m/s, where the PSM goes to Crouch 3 (full slide 3.13 s / ~20 m; the
// player input ends it earlier). Measured over 18 recorded slides,
// replay fixture tests\data\slide_feeds.txt. The feed's locoState is 0 and the stick stays forward during the slide, so the
// PSM detail is the only signal; the camera pivot already treats Slide as crouch (cet\utpp_camera).
// motion chooser (bundle chooser.json "slide" section, ParseSlideSpec): root row MovementMode == Sliding -> nested slide
// table: PSD_Slide_FeetOut (MovementDirection Forward) / PSD_Slide_KneesOut (Other); exit rows (MovementMode_LastFrame == Sliding
// on the first frame after the slide, every other column any) -> PSD_Slide_ExitTo{StandIdle, CrouchIdle, CrouchWalk, Walk, Run,
// Sprint}. Everything below is OPTIONAL: MapConfig::slide false (the default, also a bundle without the section) leaves
// MapToChooser bit-identical to the pre-slide code; the only visible difference is the added ChooserState fields (movementModeLast
// = movementMode of the previous frame, movementDirection 0), which no slide-less chooser reads.
inline constexpr float kSlideDirectionEnterDeg = 45.0f; // capsule velocity > this far off the body facing -> Direction Other
inline constexpr float kSlideDirectionExitDeg = 40.0f;  // back to Forward below this (5 deg hysteresis: no DB flicker at the edge) [P]
inline constexpr float kSlideDirectionMinSpeed = 0.5f;  // m/s: below, the velocity heading is noise and the direction holds [P]
inline constexpr float kSlideDecelerationMs2 = 2.75f;   // measured median game braking of the slide (m/s^2)
inline constexpr float kExitAirGraceSeconds = 0.35f;  // MapConfig::exitAirGraceS the runtime uses
inline constexpr float kContactHoldSeconds = 0.05f; // MapConfig::contactHoldS the runtime uses
inline constexpr float kJumpOutLandConfirmS = 0.10f; // review r2 finding 16: ground contact this long after the jump-out flight = its touchdown (the flag ends there)
inline constexpr float kContactFallMs = 4.0f;          // m/s downward: a contact feed falling faster than this is no landing yet
inline constexpr float kRagdollRecoverMaxDropM = 0.5f; // MapContext::ragdollRecover: a drop up to this far above the floor is no fall
#ifndef UTPP_RAGDOLL_RECOVER_MUTATION
#define UTPP_RAGDOLL_RECOVER_MUTATION 0
#endif
inline constexpr float kExitAirRiseMs = 1.5f;         // m/s: a feed rising at least this fast inside the exit window is a real jump-out
// MapConfig::jumpTakeoffHoldS: longest grounded PSM Jump 18 run kept OnGround before the launch (s). The recorded take-offs launch on the
// next feed (16-17 ms at 60 Hz); 0.1 s covers a 30 Hz frame pair with margin without holding a capsule that stays on the ground.
inline constexpr float kJumpTakeoffHoldSeconds = 0.1f;
inline constexpr float kSlideAirGraceSeconds = 0.2f;   // MapConfig::slideAirGraceS the runtime uses [P] (drops seen: 0.08-0.10 s)
// MapConfig::groundDropGraceS the runtime uses: longer than the recorded step-downs (<= 6 feeds, 0.10 s, 10 cm) and short enough that a
// fall without the PSM Fall detail still turns InAir after ~4 cm more of free fall. Two feeds at least, so one long frame cannot pass it.
inline constexpr float kGroundDropGraceSeconds = 0.12f;
inline constexpr int kGroundDropMinFeeds = 2;
inline constexpr float kGroundDropFallM = 0.2f; // m of capsule drop that is a fall inside the grace (fast or long frames)
inline constexpr float kGroundDropProbeM = 0.5f; // m: a floor probe farther below than this is a fall at once (step-downs probed <= 0.24 m)
inline constexpr float kStanceFilterSeconds = 0.15f;    // MapConfig::stanceFilterS the runtime uses (task bound: changes < ~0.15 s filtered)
// P1 fix batch 2 (2026-10-21 12:15 recording, tests/P1StanceLatencyTest.cpp): the clean Stand <-> Crouch edge bypasses the filter
// (MapConfig::stanceCleanEdge). The previous raw value must have held kStanceCleanHoldSeconds (= the filter time), a crouch ENTRY
// needs the capsule slower than kStanceCleanMaxSpeedMs: just above the game's crouch walk speed (kGameCrouchPointMotion 2.7 m/s) and well
// below its run speed (3.85 m/s), where Stand -> Crouch 3 for 0.10-0.12 s -> CrouchSprint 30 happens (2 of 2 such blips, feed t
// 278.786 / 280.435, entered at 3.84-3.85 m/s; the 5 held crouches entered at that speed keep the pre-fix filter delay).
inline constexpr float kStanceCleanHoldSeconds = kStanceFilterSeconds;
inline constexpr float kStanceCleanMaxSpeedMs = 3.0f;
// Offline mutation switch of the P1 stance changes (tests build mutated copies with -DUTPP_P1_MUTATION=k; 0 / undefined = production):
// 1 no clean-edge bypass (the pre-fix 0.15 s on every edge), 2 the overlay pose crossfade keeps poseSwitch on a stance change (Overlay.hpp),
// 3 no speed limit on the crouch entry, 4 no min-hold of the previous raw value (a return inside stanceFilterS of a bypassed edge is
// bypassed too). Shared with Overlay.hpp.
#ifndef UTPP_P1_MUTATION
#define UTPP_P1_MUTATION 0
#endif
// Offline mutation switch of Shooter mode / the H2 safeguard (tests build mutated copies with -DUTPP_SHOOTER_MODE_MUTATION=k; 0 / undefined =
// production): 1 the shooterMode setting never strafes, 2 spin transitions are not suppressed, 3 the standing V does not hold the visual root
// while the camera orbits (MotionDriver: the rest turn-in-place turns her behind the camera), 4 no root lock while moving, 5 the H2 gate
// ignores bodyTurnSwitchedOff, 6 the H2 clear hold is never reset, 7 CanSprint is bypassed (the sprint cone does not apply), 8 the raw
// hip-fire state is not an H2 exclusion, 9 spin suppression follows the windowed shooterModeActive instead of the setting (slide window),
// 10 the Shooter root lock has no rate bound.
#ifndef UTPP_SHOOTER_MODE_MUTATION
#define UTPP_SHOOTER_MODE_MUTATION 0
#endif

// The chooser-side values the adapter writes (parsed from chooser.json, defaults = format version 1).
struct SlideSpec
{
    bool declared = false; // chooser.json carries a complete, version 1 "slide" section
    int slidingValue = 2;  // E_MovementMode Sliding (OnGround 0, InAir 1)
    int forwardValue = 0;  // E_MovementDirection Forward
    int otherValue = 1;    // E_MovementDirection Other (B / LR / LL / RL / RR collapsed)
};

// Reads the "slide" section of a chooser.json text (load time, worker thread). declared only when the section is version 1, has
// integer sliding_value / direction_values.Forward / direction_values.Other and lists both root databases (PSD_Slide_FeetOut /
// PSD_Slide_KneesOut). Any other layout -> declared false (the bundle then behaves as before). aOut is reset first.
inline bool ParseSlideSpec(std::string_view aChooserJson, SlideSpec& aOut)
{
    aOut = SlideSpec{};
    json::Value root;
    std::string err;
    if (!json::Parse(aChooserJson, root, err))
        return false;
    const json::Value* sl = root.Find("slide");
    if (!sl || !sl->IsObject())
        return false;
    auto integer = [](const json::Value* v, int& out) {
        if (!v || !v->IsNumber() || !v->isInteger || !(v->num >= 0.0) || !(v->num <= 64.0))
            return false;
        out = static_cast<int>(v->num);
        return true;
    };
    const json::Value* ver = sl->Find("version");
    const json::Value* dv = sl->Find("direction_values");
    const json::Value* dbs = sl->Find("databases");
    SlideSpec s;
    if (!ver || !ver->IsNumber() || ver->num != 1.0 || !dv || !dv->IsObject() || !dbs || !dbs->IsObject() ||
        !integer(sl->Find("sliding_value"), s.slidingValue) || !integer(dv->Find("Forward"), s.forwardValue) ||
        !integer(dv->Find("Other"), s.otherValue) || s.slidingValue < 2 || s.forwardValue == s.otherValue ||
        !dbs->Find("PSD_Slide_FeetOut") || !dbs->Find("PSD_Slide_KneesOut"))
        return false;
    s.declared = true;
    aOut = s;
    return true;
}

// Capsule velocity heading vs the body facing (both in the integ yaw convention, degrees) -> MovementDirection Other. Hysteresis
// kSlideDirectionEnterDeg / kSlideDirectionExitDeg around the previous answer.
inline bool SlideDirectionOther(float aVelocityYawDeg, float aActorYawDeg, bool aPreviousOther)
{
    const float d = std::fabs(motiontraj::NormalizeAxis(aVelocityYawDeg - aActorYawDeg));
    return aPreviousOther ? d > kSlideDirectionExitDeg : d > kSlideDirectionEnterDeg;
}

// Actor (capsule) yaw source for the query facing / gait / direction / root delta (facing experiment, replay --facing).
// Game: the feed playerYawDeg (current, see the [P] note in MapToChooser). MotionActor [P] (experiment, not the in-game
// owner: actor rotation in game belongs to the runtime bodyturn module): the yaw motion's CMC would give the actor -
// CBP UpdateRotation_PreCMC (GASPALS__Blueprints__CBP_SandboxCharacter.txt l. 1144-1156): Strafe / Aim ->
// bUseControllerDesiredRotation (DesiredRotation = controller desired rotation = control yaw, UE 5.8
// CharacterMovementComponent.cpp l. 6653-6656), else bOrientRotationToMovement (ComputeOrientToMovementRotation l. 6606-
// 6621: Acceleration direction, unchanged at zero acceleration); RotationRate yaw -1 on the ground (CDO l. 766-770,
// PreCMC l. 1156) = instant (GetAxisDeltaRotation l. 6598: rate < 0 -> 360 deg), 200 deg/s while falling (l. 1154).
// Limit: this is motion with WantsToStrafe toggled OFF, not motion's default mode. The CBP CDO sets
// CharacterInputState.WantsToStrafe = True (CBP l. 18; toggled by input at l. 887) and l. 1146 tests
// WantsToStrafe || WantsToAim, so default motion strafes (actor yaw = control yaw) all the time; here Strafe is entered
// only through MapConfig::strafeWhileAiming && aiming (the game's own rule), everything else is OrientToMovement.
enum class FacingSource : uint8_t
{
    Game,
    MotionActor
};

struct MapConfig
{
    // Horizontal game m/s -> motion cm/s. speedScale > 0: uniform (x 100 x speedScale). speedScale 0 (default): piecewise map
    // like v1 SpeedCalibration (PlaybackRate.hpp MapSpeedPoints) from the game 2.31 points (SpeedCalibration.hpp
    // kGameStandPoints 1.6 / 3.85 / 8.2, kGameCrouchPoint 2.7) onto the motion CMC gait caps of cmc (Walk 200 / Run 500 /
    // Sprint 700, Crouch 225). [P] The prediction clamps to those caps (UE StepCharacterMovementGroundPrediction
    // ClampMaxSize), so a steady game speed must land on the cap: uniform x 1 put the steady 3.8 m/s run at 380 cm/s, the
    // prediction accelerated to 500 and IsStarting (+100) stayed true on every steady-run frame.
    float speedScale = 0.0f;
    float gameStandPoints[3] = {kGameStandPointsMotion[0], kGameStandPointsMotion[1], kGameStandPointsMotion[2]};
    float gameCrouchPoint = kGameCrouchPointMotion;
    // Gait from input (CBP GetDesiredGait), not from speed. [P] stick mode VariableSpeed-Walk/Run (CBP E_AnalogStickBehavior;
    // motion default FixedSpeed-SingleGait, CBP l. 5): the game speed is analog in the stick (kGameStickSpeedMotion), FixedSpeed
    // would put every partial-stick game walk on the Run cap 500 (IsStarting on 60 % of slow_walk_half_stick frames) and
    // SingleGait gives Run at half stick. Walk/Run split at the motion AnalogWalk/RunThreshold 0.7 (CBP l. 7) on the game
    // |MoveX, MoveY|: half stick (night logs 0.16-0.27) -> Walk, full stick (0.73-1.0) -> Run. WantsToWalk: the game walk
    // toggle (ToggleWalk_Button IK_G, keyboard only, game r6\config\inputUserMappings.xml) = MotionFeed::wantsToWalk (PSM WalkToggled,
    // MotionDriver Step) or the vanilla-speed AimWalk (MotionFeed::aimWalkVanilla, G9).
    StickMode stickMode = StickMode::VariableSpeedWalkRun;
    float analogWalkRunThreshold = 0.7f;
    // G3 (default OFF = motion, one threshold): Walk -> Run at stick >= gaitHysteresisUp, Run -> Walk below gaitHysteresisDown, from the
    // previous frame's gait (MapState::st.gait). Only the Walk/Run stick modes (GaitFromInput).
    bool gaitHysteresis = false;
    float gaitHysteresisUp = 0.75f, gaitHysteresisDown = 0.65f;
    // Gait return hold (0 = off, the pre-guard behaviour): a RISING stick keeps the Run gait for this long after the last Run frame while the capsule is still
    // running (the stick that passed the centre when the direction reversed); see the comment at the use in MapToChooser.
    float gaitReturnHoldS = 0.3f;
    // Dodge gait hold (0 = off): the game's ground Dodge (PSM detail 7) replaces the Sprint detail for ~0.2 s and its impulse decays
    // over a few feeds after it (one Stand feed between them in the recorded sprint dashes). The Sprint input of the feed before the
    // dodge is kept through the detail and, after it, while the capsule speed keeps falling (at most this long), and the chooser's
    // dodge flag (MatcherConfig::dodgeHold) is kept over the same decay: the search no longer drops Sprint -> Run under the impulse and
    // back (Sprint_Loop -> Transition_Sprint_to_Run -> Run_Loop -> Transition_Run_to_Sprint in 0.2 s).
    float dodgeGaitHoldS = 0.4f;
    // Gait return hold: the stick counts as rising when it grows faster than this (stick magnitude per second; 0.3 = 0.005 per feed at 60 feeds per second)
    float gaitReturnRiseRate = 0.3f;
    // Digital heading step (ChooserState::headingStepAgeS, MatcherConfig::headingCatchUpS): the wanted run direction turns by at least this many
    // degrees from one feed to the next (a direction key; the camera turns 3-5 deg per feed under a fast mouse flick). 0 = never.
    // A start from rest counts as a step too when the first wanted direction is at least this far from the facing.
    float headingStepDeg = 35.0f;
    // Swept heading step: the wanted direction counts as a step also when it turns by headingStepDeg within this many seconds over several
    // feeds (a thumbstick swung to a new direction; the camera under a fast mouse flick turns 3-5 deg per feed, under 35 deg in 0.1 s).
    // 0 = only the change between two feeds.
    float headingSweepS = 0.1f;
    // G2: capsule speed (game m/s) at the FULL stick while the game walk toggle is on (CET mm.walkSpeed -> UTPPSetMotionWalkSpeed; the
    // script holds the capsule there with one MaxSpeed multiplier, utpp.reds UTPPUpdateWalkSpeed). 0 = off: nothing below changes, bit
    // for bit. > 0 (and < gameStandPoints[1]) and MotionFeed::wantsToWalk on the stand ground (EffectiveMapConfig): the first stand speed
    // point becomes this speed -> motion Walk cap (cmc.walkSpeed), so this speed is a steady walk (gameCap = point, analog modifier 1 at the
    // full stick) instead of the 2.72 m/s the toggle gives unmodified, which the 1.6 -> 200 / 3.85 -> 500 map put at ~350 cm/s over the
    // Walk cap 200 (measured game 2.5: G + full W = 2.72 m/s).
    float gameWalkTogglePoint = 0.0f;
    // Minimum analog walk speed (CET mm.minWalkSpeed -> UTPPSetMotionMinAnalogWalk; the script lifts the capsule with one MaxSpeed
    // multiplier, utpp.reds UTPPUpdateAnalogFloor / AnalogWalkFloorMul). Set both with SetAnalogFloor: analogFloorFrac = the floor's
    // fraction of the full-stick speed (AnalogFloorFraction), analogFloorStick = the stick of that fraction (AnalogFloorStickFor, 0.455 at
    // 0.82 m/s). With input below analogFloorStick on the ground in Stand / Crouch without sprint (the script's gate) MapToChooser queries
    // the speed the lifted capsule really reaches (AnalogFloorLiftFraction: the floor, or 4 x the speed at a very small tilt). 0 = off
    // (default): nothing changes, bit for bit.
    float analogFloorStick = 0.0f;
    float analogFloorFrac = 0.0f;
    // the floor setting itself (m/s, 0 = off): EffectiveMapConfig recomputes the fraction / stick against the walk toggle speed
    float analogFloorMs = 0.0f;
    // the multiplier cap the script applies (kAnalogFloorMulMax; under the walk toggle AnalogFloorMulMaxWalk, set by EffectiveMapConfig)
    float analogFloorMulMax = kAnalogFloorMulMax;
    // Internal, set by EffectiveMapConfig only (never by the host): walkMapActive = gameStandPoints[0] is the walk toggle point and the
    // full-stick speed of the stand is that point; gameRunReductionMs = Tier2 Slow, m/s taken off the full-stick stand run speed.
    bool walkMapActive = false;
    float gameRunReductionMs = 0.0f;
    // [P] true: AnalogInputModifier = mapped steady game speed at this stick / motion gait cap (GameAnalogModifier) instead of
    // the CMC |Acceleration| / MaxAcceleration (= |stick| in the VariableSpeed modes), so the prediction cap equals the mapped
    // steady game speed at every measured stick. false: CMC value (motion).
    bool gameAnalogModifier = true;
    float gravity = 9.81f;          // Standalone fallback; runtime passes this feed's learned AirState gravity (m/s^2).
    float capsuleBrakingDeceleration = 0.0f; // fresh native base stat, physical m/s^2; zero keeps motion fallback
    float capsuleAcceleration = 0.0f; // fresh observed base * applied multiplier, physical m/s^2
    float stanceSpeedBlendTime = 0.0f; // Runtime supplies its existing ground-to-ground transition duration.
    bool visualStrafe = false; // Explicit grounded firing-pose movement; never changes native ADS/PSM.
    bool strafeWhileAiming = true;  // [P] motion RotationMode = Strafe when !bOrientRotationToMovement; the game strafes while aiming
    // Shooter mode (CET bodyturn.shooterMode -> UTPPSetMotionShooterMode): motion's default Strafe rotation mode (camera-locked actor) on the ground and in the
    // air, outside the slide itself and traversal; no spin transitions. false = the pre-Shooter MapToChooser, bit for bit.
    bool shooterMode = false;
    // H2 safeguard enable (MotionDriver sets it from g_h2GuardSetting). It only acts while MapContext::bodyTurnSwitchedOff is true.
    bool h2Strafe = false;
    motiontraj::MotionCmc cmc = MotionGameCmc(); // CBP CMC values driving the prediction (MotionTrajectory.hpp), MinAnalogSpeed 0 [P]
    FacingSource facingSource = FacingSource::Game;
    // Slide feed (see "slide feed" above). slide false = the pre-slide MapToChooser, bit for bit. The driver sets it only when the
    // CET setting mm.slide is on AND the active bundle declares the slide section (SlideSpec::declared).
    bool slide = false;
    SlideSpec slideSpec;
    // physical game m/s^2 of the linear braking the query trajectory follows while sliding (input acceleration off, no Run-cap
    // acceleration towards the held stick); 0 keeps the standard input-driven prediction [P]
    float slideDeceleration = 0.0f;
    // Slide speed map (game m/s -> motion cm/s of the query trajectory), design choice 2026-10-19 (replay of 13 recorded slides):
    // the game slide runs 11 -> 2.7 m/s, the Neutral slide clips 5.0 (Into) -> 4.5 m/s (Loop constant), the exit clips start at
    // 5-7 m/s. While sliding the query speed is slideLoopCmS at slideLowMs and below, rising linearly to slideEntryCmS at
    // slideHighMs and above (so Loop matches during the braking, Into at the entry); the first slideExitMapS after the slide
    // the speed decays linearly from slideExitStartCmS to the normal map (the exit clip decelerates the same way). [P] values.
    float slideLoopCmS = 450.0f, slideEntryCmS = 450.0f, slideExitStartCmS = 550.0f;
    float slideLowMs = 2.0f, slideHighMs = 8.25f, slideExitMapS = 0.6f;
    // fix/slide-exit (2026-10-21, replay tests/SlideExitTest.cpp). The defaults below (0 / false) are the pre-fix behaviour, bit for
    // bit; MotionDriver Step sets them (slideAirGraceS only with mc.slide, like slideDeceleration; the stance filter always, the
    // flicker comes from the game PSM whatever the slide setting is):
    // slideAirGraceS: while the PSM still reports the slide (5) or within slideExitMapS after it, a !onGround air feed that the PSM
    // does not confirm (no PsmAir detail: Fall / Jump / SlideFall) counts as ground for this long. 2026-10-21 03:37: a 0.08-0.10 s
    // drop (slope / step, vz -1.2 .. -2.8 m/s, ground 0.10-0.15 m below) with the PSM at Slide 5 / Crouch 3 made the chooser pick
    // Jump_F_Off_* (take-off clip, then a landing) after a slide that never left the ground; real take-offs from a slide arrive as PSM
    // Jump 18 and are not delayed. Only with MapConfig::slide and a slide bundle.
    float slideAirGraceS = 0.0f;
    // exitAirGraceS (2026-10-24): inside the vehicle exit window a !onGround feed / PSM Fall that is not rising (the unmount puts V on
    // the ground from the exit workspot: Jump_F_Start then Land_Stand_Light squat on a standing exit, night run 20261024-145425) counts
    // as ground for this long. A real jump-out (PSM Jump or vz >= kExitAirRiseMs) is not delayed.
    float exitAirGraceS = 0.0f;
    // groundDropGraceS: everywhere else a !onGround feed out of a ground frame that the PSM does not confirm (no air detail) and that
    // does not rise counts as ground for this long (and at least kGroundDropMinFeeds feeds), unless the capsule dropped kGroundDropFallM or
    // the floor probe reports more than kGroundDropProbeM below.
    // A one-feed contact loss while V runs against a ledge edge (capsule sinks 2-3 cm, then depenetrates up) or a step-down off a curb
    // (0.05-0.10 s, 3-10 cm) then keeps the ground clips; a real fall gets the PSM Fall detail within ~0.05 s or outlasts the grace,
    // a real jump rises (vz >= kExitAirRiseMs) or carries the PSM Jump detail and is never delayed. 0 = off (the previous mapping).
    float groundDropGraceS = 0.0f;
    // contactHoldS (fix/jumpout-cam, log 2026-11-03 17:34:36 jump-out B): inside the vehicle exit window / jump-out flight a ground-contact feed (onGround, no air
    // PSM detail) while the capsule still falls faster than kContactFallMs is no landing (it grazed the exiting car at 0.23 m, vz -11 m/s, for 2 feeds): the
    // chooser stays InAir for at most this many seconds of such feeds, so the heavy land database is not picked 0.1 s early. The real landing zeroes the vertical
    // speed on the contact feed or the next one. 0 = off (the previous mapping).
    float contactHoldS = 0.0f;
    // jumpTakeoffHoldS (JS1, 2026-10-31 loco diagnostic, tests/LocoStartTest.cpp): the PSM reports Jump 18 one feed BEFORE the capsule
    // launches (onGround 1, vz 0 standing / -0.39 running, the 5.66 m/s impulse arrives on the next feed). InAir on that feed searched
    // the jump databases on a query with no vertical velocity and no floor (TimeToLand = the no-hit 1.52 s: Jump_F_Start_Walk on a
    // standing jump, Jump_F_Off_Run at 0.6 s on a running one) and the impulse feed re-picked 15-33 ms later (two pose targets, the
    // visible snap). A grounded Jump 18 feed out of a ground frame, with the capsule not rising (vz < kExitAirRiseMs), stays OnGround
    // for at most this long, so the take-off search runs once, on the impulse feed (rising arc, the AirFloorStep take-off plane seed:
    // TimeToLand ~0.7 s). The default is the production value (the host builds MapConfig{}); 0 = the pre-fix mode, bit for bit.
    float jumpTakeoffHoldS = kJumpTakeoffHoldSeconds;
    // stanceFilterS: the Stand / Crouch stance seen by the chooser changes only after the new value held this long (single-feed PSM
    // flicker Crouch 3 <-> CrouchSprint 30 while crouch + sprint are held: 24 runs < 0.15 s, 0.017 s spikes every 0.13 s on
    // 2026-10-21 03:37, each one a stance interrupt + 0.5 s blend). Feeds of the old value drain the pending time instead of
    // restarting it, so a periodic single old-value feed (7 x Crouch, 1 x CrouchSprint) delays a real change but cannot block it. The
    // slide frames and the first frame after a slide pass unfiltered (the exit database is chosen by the stance of that frame).
    // CrouchSprint counts as sprint only once the filtered stance is Stand (a spike inside the crouch is neither Stand nor Sprint).
    // stanceHoldInDodge: the Dodge 7 PSM detail (0.2 s from the crouch on 03:37 v61.7) reads as the last RAW stance before it, so a
    // dodge neither stands the body up nor erases a change that is already pending (the filter timer keeps running through it).
    float stanceFilterS = 0.0f;
    bool stanceHoldInDodge = false;
    // P1 (fix batch 2): stanceCleanEdge = a CLEAN edge Stand 1 / AimWalk 2 <-> Crouch 3 reaches the chooser on the feed it appears on
    // instead of stanceFilterS later (measured: every in-place crouch / stand-up picked its clip 0.13-0.15 s after the PSM change, 8-9
    // feeds, the 0.15 s of the filter). Clean = this and the previous feed both read Stand / AimWalk / Crouch, the raw value before the
    // edge held >= stanceCleanHoldS (no flicker train; also the min-hold of a bypassed edge: a return inside stanceCleanHoldS of it goes
    // through the filter), no slide / provisional value / slide-exit window, and a crouch ENTRY slower than stanceCleanMaxSpeedMs.
    // CrouchSprint 30, Dodge 7, Sprint 4, air, landing and slide frames never bypass; they keep the filter above. false = the pre-P1
    // behaviour bit for bit.
    bool stanceCleanEdge = false;
    float stanceCleanHoldS = 0.15f;
    float stanceCleanMaxSpeedMs = 3.0f;
};

// Factor for motiontraj::GameVelocityToCm (horizontal cm/s = m/s x 100 x factor) at the game planar speed aGameSpeed (m/s).
inline float SpeedScaleFor(const MapConfig& c, int aStance, float aGameSpeed)
{
    if (c.speedScale > 0.0f)
        return c.speedScale;
    if (!(aGameSpeed > 1.0e-4f) || !std::isfinite(aGameSpeed))
    {
        // below / at 0: slope of the first segment (MapSpeedPoints is linear from 0 to the first point)
        aGameSpeed = aStance == 1 ? c.gameCrouchPoint : c.gameStandPoints[0];
    }
    const SpeedPoints pts = aStance == 1
                                ? SpeedPoints{1, {c.gameCrouchPoint, 0.0f, 0.0f}, {c.cmc.crouchSpeed, 0.0f, 0.0f}}
                                : SpeedPoints{3, {c.gameStandPoints[0], c.gameStandPoints[1], c.gameStandPoints[2]},
                                              {c.cmc.walkSpeed, c.cmc.runSpeed, c.cmc.sprintSpeed}};
    return MapSpeedPoints(pts, aGameSpeed) / (aGameSpeed * 100.0f);
}

inline float BlendedSpeedScaleFor(const MapConfig& c, float aCrouchWeight, float aGameSpeed)
{
    if (aCrouchWeight <= 0.0f) return SpeedScaleFor(c, 0, aGameSpeed);
    if (aCrouchWeight >= 1.0f) return SpeedScaleFor(c, 1, aGameSpeed);
    const float stand = SpeedScaleFor(c, 0, aGameSpeed), crouch = SpeedScaleFor(c, 1, aGameSpeed);
    return stand + (crouch - stand) * aCrouchWeight;
}

inline brake::Prediction CapsuleStopPrediction(const MapConfig& c, float crouchWeight, float gameSpeed)
{
    brake::Prediction p;
    p.deceleration = c.capsuleBrakingDeceleration;
    p.initialSpeed = gameSpeed;
    p.crouchWeight = std::clamp(crouchWeight, 0.0f, 1.0f);
    p.stand = {3, {c.gameStandPoints[0], c.gameStandPoints[1], c.gameStandPoints[2]},
                   {c.cmc.walkSpeed, c.cmc.runSpeed, c.cmc.sprintSpeed}};
    p.crouch = {1, {c.gameCrouchPoint, 0, 0}, {c.cmc.crouchSpeed, 0, 0}};
    if (c.speedScale > 0.0f)
        p.stand = p.crouch = SpeedPoints{1, {1, 0, 0}, {100.0f * c.speedScale, 0, 0}};
    return p;
}

// Slide braking prediction: the capsule brakes linearly at slideDeceleration whatever the stick does; the query speed follows
// the slide speed map (0 -> 0, slideLowMs -> slideLoopCmS, slideHighMs -> slideEntryCmS).
inline brake::Prediction SlideBrakePrediction(const MapConfig& c, float gameSpeed)
{
    brake::Prediction p;
    p.deceleration = c.slideDeceleration;
    p.initialSpeed = gameSpeed;
    p.crouchWeight = 0.0f;
    p.stand = p.crouch = SpeedPoints{3, {0.0f, c.slideLowMs, c.slideHighMs}, {0.0f, c.slideLoopCmS, c.slideEntryCmS}};
    return p;
}

// Steady game speed (m/s) the stick asks for: full-stick game speed of the stance / PSM (stand run 3.85, sprint 8.2, crouch
// 2.7 = gameStandPoints / gameCrouchPoint) x GameStickSpeedFraction(|stick|).
// G2 / G5: with the walk toggle map (walkMapActive) the full-stick stand speed is the walk toggle point; with Tier2 Slow the stand run
// loses gameRunReductionMs (gameStandPoints[1] 3.85 - 1.3 = 2.55 m/s; 2.54 from the 3.84 measurement, a 0.01 m/s difference). Both are 0 / false in the unmodified config: the plain stand run speed.
inline float GameFullStickSpeed(const MapConfig& c, int aStance, bool aSprint)
{
    return aStance == 1 ? c.gameCrouchPoint
                        : (aSprint ? c.gameStandPoints[2]
                                   : (c.walkMapActive ? c.gameStandPoints[0] : c.gameStandPoints[1] - c.gameRunReductionMs));
}
inline float GameSteadySpeed(const MapConfig& c, int aStance, bool aSprint, float aStick)
{
    return GameFullStickSpeed(c, aStance, aSprint) * GameStickSpeedFraction(aStick);
}

inline accel::Prediction CapsuleStartPrediction(const MapConfig& c, float crouchWeight,
                                                float gameSpeed, float gameCap)
{
    const auto map = CapsuleStopPrediction(c, crouchWeight, gameSpeed);
    accel::Prediction p;
    p.acceleration = c.capsuleAcceleration;
    p.initialSpeed = gameSpeed;
    p.maxSpeed = gameCap;
    p.crouchWeight = map.crouchWeight;
    p.stand = map.stand;
    p.crouch = map.crouch;
    return p;
}

// [P] AnalogInputModifier for the prediction cap (UE l. 73 MaxSpeed = GaitMaxSpeed x modifier): mapped steady game speed
// (cm/s, SpeedScaleFor) / motion gait cap (MakeDerived: crouch 225, Sprint 700, Run 500, Walk 200), clamped to [0, 1].
// The same from the fraction of the full-stick speed instead of the stick (GameAnalogModifier = GameAnalogModifierFrac of
// GameStickSpeedFraction(stick), bit for bit); the minimum analog walk speed passes the lifted fraction of the extended curve.
inline float GameAnalogModifierFrac(const MapConfig& c, int aGait, int aStance, bool aSprint, float aFrac)
{
    const float v = GameFullStickSpeed(c, aStance, aSprint) * aFrac;
    if (!(v > 0.0f))
        return 0.0f;
    const float target = v * 100.0f * SpeedScaleFor(c, aStance, v);
    const float cap = aStance == 1 ? c.cmc.crouchSpeed
                                   : (aGait == 2 ? c.cmc.sprintSpeed : (aGait == 1 ? c.cmc.runSpeed : c.cmc.walkSpeed));
    return cap > 0.0f ? std::fmin(std::fmax(target / cap, 0.0f), 1.0f) : 0.0f;
}

// Minimum analog walk speed: MapConfig fields of a floor setting (game m/s; 0 / NaN / out of range = off).
inline void SetAnalogFloor(MapConfig& c, float aMinMs)
{
    c.analogFloorFrac = AnalogFloorFraction(aMinMs);
    c.analogFloorStick = AnalogFloorStickFor(aMinMs);
    c.analogFloorMs = c.analogFloorFrac > 0.0f ? aMinMs : 0.0f;
}

// Fraction of the full-stick speed the capsule reaches under the script's floor multiplier at stick aStick (both on the extended curve):
// the floor aFracMin, or kAnalogFloorMulMax x the stick's own fraction where the multiplier is capped (|mv| < ~0.077 at 0.82 m/s), so the
// query never runs ahead of the capsule. Mutation 3 (UTPP_ANALOGFLOOR_MUTATION=3): the cap is ignored (always the floor).
inline float AnalogFloorLiftFraction(float aStick, float aFracMin, float aMulMax = kAnalogFloorMulMax)
{
#if UTPP_ANALOGFLOOR_MUTATION == 3
    (void)aStick;
    return aFracMin;
#else
    return std::fmin(aFracMin, aMulMax * AnalogFloorStickFraction(aStick));
#endif
}

inline float GameAnalogModifier(const MapConfig& c, int aGait, int aStance, bool aSprint, float aStick)
{
    const float v = GameSteadySpeed(c, aStance, aSprint, aStick);
    if (!(v > 0.0f))
        return 0.0f;
    const float target = v * 100.0f * SpeedScaleFor(c, aStance, v);
    const float cap = aStance == 1 ? c.cmc.crouchSpeed
                                   : (aGait == 2 ? c.cmc.sprintSpeed : (aGait == 1 ? c.cmc.runSpeed : c.cmc.walkSpeed));
    return cap > 0.0f ? std::fmin(std::fmax(target / cap, 0.0f), 1.0f) : 0.0f;
}

// Per-frame context MapToChooser cannot derive from the feed (explicit inputs, set by the caller every frame).
struct MapContext
{
    // motion CurrentDatabaseTags contains "Pivots": tags of the database of the PREVIOUS search result
    // (ABP l. 1901 sets them after the selection; Update_States of the next frame reads them, l. 1785 / 1875).
    bool currentDbPivots = false;
    // motion RootTransform yaw (ABP Update_EssentialValues l. 1760: OffsetRoot node transform, RotationMode Accumulate l. 1230 /
    // Get_OffsetRootRotationMode l. 1944-1950) in the integ yaw convention, degrees. rootYawValid false = motion else branch
    // (l. 1762 RootTransform = CharacterTransform): root yaw = actor yaw, no turn in place / spin.
    bool rootYawValid = false;
    float rootYawDeg = 0.0f;
    bool aimFacing = false; // runtime-verified aim intent and fresh camera target
    bool shotTurn = false;
    float shotTargetYawDeg = 0;
    int shotTurnSide = 0; // shotturn::Hold::side: +1 left, -1 right, 0 none (keeps the clip direction stable near 180 deg)
    // Rest turn-in-place: the caller's permission - ranged weapon in WeaponRight, TPP, no aim intent, nothing owning the facing
    // (aimFacing / shot goal) and no ADS montage. A hidden yaw offset between the capsule and the visible root of at least
    // kTurnInPlaceDeg is then turned by the authored standing / crouching TIP footwork, goal = the capsule (the actor stands on
    // the game's yaw); the root absorbs the clip's yaw. Without it the offset stays (or MotionOffsetRoot's slow fallback folds it).
    bool restTurn = false;
    // H2 safeguard inputs (MotionDriver Step). bodyTurnSwitchedOff: the native body turn is off for a reason other than aim / hip fire (the
    // host computes it from g_bodyTurnOn and the Locomotion state; the default false applies to hand-built contexts and every offline replay).
    // hipFireAny: the RAW hip-fire state (weapon action 8 or the DLL pose hold), independent of the pose gate (visualStrafe): the CET hip-fire
    // lease switches the native off for any weapon, also those whose resolved aim pose is Default.
    bool bodyTurnSwitchedOff = false;
    bool hipFireAny = false;
    // hipLease: the CET hip-fire lease (UTPPSetMotionHipLease) is running. It switches the native body turn off for ANY weapon that reports Shoot,
    // also those whose action never reaches the DLL (resolved aim pose Default: grenade / projectile launcher, unlisted types), so hipFireAny
    // cannot see it. An H2 exclusion with the same clear hold.
    bool hipLease = false;
    bool vehicleExit = false; // inside the vehicle exit window (MapConfig::exitAirGraceS)
    // After a ragdoll (the knockdown detail just ended, an episode or the native get-up running): a !onGround / PSM-air feed whose
    // floor is at most kRagdollRecoverMaxDropM below the capsule and that is not rising is the entity settling on the floor, not a
    // fall (it would pick Jump_*_Start and Lands_Light for a 0.17 m drop). Default false = bit-identical to before.
    bool ragdollRecover = false;
    // fix/jumpout-cam: inside the exit window of a vehicle JUMP-OUT (MMDriver g_jumpOutExit: the launch of the cool exit, double F while driving). The
    // chooser state then carries ChooserState::jumpOutFlight, which ChooserMatcher reads in the air only. A hand-built context leaves it false.
    bool jumpOutFlight = false;
};

// [P] database tag "Pivots" from the motion database name (PSD_*_Pivots); the tag arrays are not exported to the chooser data.
inline bool DatabaseHasPivotsTag(std::string_view aDbName) { return aDbName.find("Pivots") != std::string_view::npos; }

// motionfix item 7 (pose snaps): v2 blend floor and the stop on a game-side speed collapse.
// Blend floor [P] 0.1 s (task bound 0.08-0.10 s): QuinticInertializer::SetMinDuration on every v2 inertializer (PoseEvaluator =
// game path incl. MM jumps, landing and traversal blends; MotionAdapter adapter = offline / replay). The v2 <-> v1 handover /
// hand-in blends (MotionDriver BlendFromHeld) run over the fixed v1 hard fade-out 0.12 s without a deficit: already above it.
inline constexpr float kBlendFloorSeconds = 0.1f;
// Stop on a speed collapse: the game stops the capsule within 1-2 feeds (car / wall / its own stop: 8.2 -> 5.2 -> 1.2 m/s at
// t 1503.53 of the 2026-10-13 dump) with no deceleration for the chooser to see, so Speed2D falls below the Stops rows
// ([20 / 100 / 550, 999] of CHT Stand Idles) before a stop clip can be chosen; when the stick-release feed is missed too, the
// first Idle feed already sits at ~1 m/s. motion's CMC brakes at CalculateBrakingDeceleration 2000 cm/s^2 without input
// (MotionTrajectory.hpp Derived), so the chooser speed is held at the pre-collapse speed braking at that rate (Sprint_Stops
// first, then Run / Walk_Stops, then Idles like a motion friction stop) while the state stays Idle on the ground.
inline constexpr float kStopCollapseDeltaMs = 4.0f;      // game m/s within the window = a collapse (task bound)
inline constexpr float kStopCollapseWindowS = 0.15f;     // [P] s the pre-collapse peak is remembered (a few missed feeds)
inline constexpr float kStopCollapseBrakeCmS2 = 2000.0f; // motion CalculateBrakingDeceleration without input
// [P] a feed within 5 % of the remembered peak counts as still at the peak (the 2026-10-13 sprint plateau 8.26 m/s ends with
// 7.99 on the stick-release feed): the braking of the hold starts at the last such feed, not at the first plateau feed.
inline constexpr float kStopCollapsePlateau = 0.95f;

// H2 safeguard (native body turn off for another reason + running backward): only the BACKWARD sector latches, the
// input angle against the actor enters at >= 105 deg (B / BL / BR sector) and leaves below 65 deg. kH2MinStick = reds dead zone
// (|move|^2 > 0.0225). Holds: the switch must have been off kH2OffHoldS and no exclusion (aim, hip fire, slide, traversal, AimWalk) for
// kH2ClearHoldS (toggle-lag protection, 9-12 feeds @60 Hz); kH2ReleaseS = no input this long ends the latch (the stop stays directional).
inline constexpr float kH2EnterDeg = 105.0f;
inline constexpr float kH2ExitDeg = 65.0f;
inline constexpr float kH2MinStick = 0.15f;
inline constexpr float kH2MinSpeedMs = 1.5f; // entry only (a spin needs >= 150 cm/s): a standing start is not a reversal
inline constexpr float kH2OffHoldS = 0.2f;
inline constexpr float kH2ClearHoldS = 0.15f;
inline constexpr float kH2ReleaseS = 0.25f;

// gait return hold: the capsule counts as running above this speed (m/s)
inline constexpr float kGaitHoldRunSpeed = 1.0f;
struct MapState
{
    int32_t prevPsm = 0;
    stancecal::Blend stanceCalibration;
    // motionfix item 7: speed collapse detection (game m/s) and the held chooser speed (motion cm/s)
    float collapsePeakSp = 0.0f;
    float collapsePeakAge = 0.0f;
    float prevStick = 0.0f;  // gait return hold: stick magnitude of the previous feed
    float sinceRunS = 1.0e9f; // gait return hold: seconds since the gait was Run (the stick's own gait, not the forced one)
    bool gaitHeld = false;    // gait return hold: the previous feed's gait was forced to Run by the hold
    // dodge gait hold (MapConfig::dodgeGaitHoldS): the previous feed's sprint input / dodge detail / game speed (m/s), the sprint input
    // kept from the feed before the dodge, the decay after the detail (on, seconds)
    bool dodgePrevSprint = false, dodgePrevDetail = false, dodgeSprint = false, dodgeDecay = false;
    float dodgePrevSpeed = 0.0f, dodgeDecayS = 0.0f;
    bool headingValid = false;      // heading step: the previous feed had a wanted direction (OrientToMovement on the ground)
    float headingYawDeg = 0.0f;     // heading step: that direction (game yaw convention)
    float headingStepAgeS = 1.0e9f; // heading step: seconds since the last step (ChooserState::headingStepAgeS)
    bool headingStepFromRest = false; // heading step: the last step was a start from rest (ChooserState::headingStepFromRest)
    // swept heading step (MapConfig::headingSweepS): the wanted directions of the recent feeds and their ages (seconds), oldest first
    static constexpr int kHeadingSweepFeeds = 16;
    float sweepYawDeg[kHeadingSweepFeeds] = {};
    float sweepAgeS[kHeadingSweepFeeds] = {};
    int sweepCount = 0;
    float stopHoldCmS = 0.0f;
    float capsuleBrakeModel = 0.0f; // active grounded no-input CP deceleration; trace only
    float capsuleAccelModel = 0.0f; // active grounded full-input applied acceleration; trace only
    motiontraj::Trajectory traj; // Update_Trajectory (history 30 + prediction 15 x 0.1 s)
    motiontraj::States st;       // Update_States values with the _LastFrame copies + predicates of this frame
    float lastAirVzCmS = 0.0f;
    float landHeavyCmS = motiontraj::kHeavyLand; // heavy-landing threshold latched at the last touchdown (HeavyLandThresholdCmS)
    float sinceLand = 1.0e9f; // s since the last CBP OnLanded
    bool jumpOutHeavyLand = false; // r3 landing: the last touchdown was the heavy one of a jump-out flight (cleared in the air)
    bool motionYawValid = false;  // FacingSource::MotionActor state seeded from the first feed playerYaw
    float motionYawDeg = 0.0f;    // motion CMC actor yaw (game yaw convention, degrees)
    float actorYawDeg = 0.0f;   // actor yaw used this frame (game convention): feed playerYaw or motionYawDeg
    // slide feed: movementMode reported to the chooser LAST frame (-> ChooserState::movementModeLast; Sliding on the first frame
    // after a slide, so the exit rows fire exactly once), this frame's slide flag and direction (hysteresis state)
    int chooserMode = 0;
    bool slideActive = false;
    bool slideDirectionOther = false;
    float slideExitAge = 1e9f; // s since the slide ended (post-slide speed decay)
    // fix/slide-exit: seconds of an unconfirmed !onGround feed (MapConfig::slideAirGraceS); stance filter output / pending age
    float slideAirAge = 0.0f;
    bool exitGraceActive = false; // last feed's air state was kept as ground by exitAirGraceS
    float exitAirAge = 0.0f; // seconds of a non-rising air feed inside the vehicle exit window (MapConfig::exitAirGraceS)
    float contactHoldAge = 0.0f; // seconds of ground-contact feeds kept InAir in a fast fall (MapConfig::contactHoldS)
    // review r2 finding 16: MapContext::jumpOutFlight stays true until the exit window of the host ends (up to 2 s), also after the jump-out touchdown. The
    // chooser state and the contact hold use it only for the flight itself: jumpOutSeenAir (an InAir feed under the flag), jumpOutGroundAge (seconds of
    // ground feeds since), jumpOutLanded (the touchdown, kJumpOutLandConfirmS of ground contact: a later jump / walk-off in the same window is an ordinary
    // flight). All three reset on a feed without the flag (the next jump-out starts clean).
    bool jumpOutSeenAir = false;
    float jumpOutGroundAge = 0.0f;
    bool jumpOutLanded = false;
    float takeoffHoldAge = 0.0f; // seconds of grounded PSM Jump 18 feeds kept OnGround before the launch (MapConfig::jumpTakeoffHoldS)
    bool takeoffHoldActive = false; // this feed is a held take-off feed (TakeoffHoldFeed): mapped with the previous feed's PSM detail
    bool airGraceActive = false; // this feed's !onGround was kept as ground by MapConfig::slideAirGraceS / groundDropGraceS (the host mirrors it to its v2 consumers)
    float groundDropAge = 0.0f; // seconds / feeds / metres of the current unconfirmed contact loss (MapConfig::groundDropGraceS)
    int groundDropFeeds = 0;
    float groundDropM = 0.0f;
    int stanceOut = -1;
    int stanceRawLast = -1; // last raw stance of a feed that was not a Dodge
    bool stanceProvisional = false; // stanceOut came from a pass-through feed (slide / first feed after it), not yet held for the filter time
    float stanceConfirmAge = 0.0f;
    float stanceCandAge = 0.0f;
    // P1 (MapConfig::stanceCleanEdge): detail of the previous feed, the raw stance value of the running run and its age
    int32_t stanceDetailLast = -1;
    int stanceRunValue = -1;
    float stanceRunAge = 0.0f;
    float slideAge = 0.0f;  // seconds of the running slide (ChooserState::slideLastDurationS on the first exit frame)
    // Shooter mode / H2 safeguard (this feed's result; the flags are read by MotionDriver for the root lock and the status)
    bool shooterModeActive = false, h2Active = false, h2Latched = false;
    float h2OffAge = 0.0f, h2ClearAge = 1.0e9f, h2NoInputAge = 0.0f;
};

// P1: details whose Stand <-> Crouch edge is "clean" (MapConfig::stanceCleanEdge): Stand 1, AimWalk 2, Crouch 3.
inline bool CleanStanceDetail(int32_t aDetail) { return aDetail == kPsmStand || aDetail == kPsmAimWalk || aDetail == kPsmCrouch; }

// fix/slide-exit: Stand (0) / Crouch (1) stance as the chooser sees it (MapConfig::stanceFilterS / stanceHoldInDodge). aRaw = the
// PSM stance of this feed. Only two values, so the pending candidate is always the other one: it is accepted once it held for
// stanceFilterS without interruption (a spike shorter than that never reaches the chooser); the previous state is held meanwhile.
// P1 (MapConfig::stanceCleanEdge): a clean edge skips that wait. aSpeedMs = planar capsule speed of the feed (m/s).
inline int FilteredStance(const MapConfig& c, MapState& m, int32_t aPsmDetail, int aRaw, bool aPassThrough, float aDt, float aSpeedMs)
{
    int raw = aRaw;
#if UTPP_SLIDEEXIT_MUTATION == 1
    if (c.stanceHoldInDodge && aPsmDetail == kPsmDodge && m.stanceOut >= 0)
        raw = m.stanceOut; // mutation 1: the Dodge keeps the filtered stance and drops the pending change
#else
    if (c.stanceHoldInDodge && aPsmDetail == kPsmDodge && m.stanceOut >= 0)
        raw = m.stanceRawLast >= 0 ? m.stanceRawLast : m.stanceOut;
    else
        m.stanceRawLast = aRaw;
#endif
    // P1 bookkeeping, every feed (also the pass-through ones): the run of the raw value (the age of the run that ended at an edge feed is
    // what the edge reads), the previous feed detail
    const bool rawEdge = raw != m.stanceRunValue;
    const float prevRunAge = rawEdge ? m.stanceRunAge : 0.0f;
    const int32_t detailLast = m.stanceDetailLast;
    m.stanceRunAge = rawEdge ? aDt : m.stanceRunAge + aDt;
    m.stanceRunValue = raw;
    m.stanceDetailLast = aPsmDetail;
    if (m.stanceOut < 0 || aPassThrough || !(c.stanceFilterS > 0.0f))
    {
        m.stanceOut = raw;
        m.stanceCandAge = 0.0f;
        m.stanceProvisional = aPassThrough && c.stanceFilterS > 0.0f; // the pass-through value is not confirmed by the filter yet
        m.stanceConfirmAge = 0.0f;
        return raw;
    }
    if (raw == m.stanceOut)
    {
#if UTPP_SLIDEEXIT_MUTATION == 2
        m.stanceCandAge = 0.0f; // mutation 2: every old-value feed restarts the timer
#else
        m.stanceCandAge = std::fmax(0.0f, m.stanceCandAge - aDt);
#endif
        if (m.stanceProvisional && (m.stanceConfirmAge += aDt) >= c.stanceFilterS)
            m.stanceProvisional = false;
        return raw;
    }
#if UTPP_SLIDEEXIT_MUTATION != 9
    // A one-feed value right after a slide (PSM Crouch 3 for one feed, then Sprint 4) is not held by the filter: the opposite value of
    // any state but the known CrouchSprint / Dodge flicker replaces it at once (nothing to debounce: it never lasted the filter time).
    if (m.stanceProvisional && aPsmDetail != kPsmCrouchSprint && aPsmDetail != kPsmDodge)
    {
        m.stanceOut = raw;
        m.stanceCandAge = 0.0f;
        m.stanceProvisional = false;
        return raw;
    }
#endif
    // P1: a clean edge (Stand / AimWalk <-> Crouch, nothing but those details on this and the previous feed) is the player deciding, not
    // a flicker: the 12:15 recording has no 1-feed Stand <-> Crouch spike in 75 such edges, and the filter delayed each of them by 8-9 feeds.
    // The raw value before it must have held stanceCleanHoldS, which is also the min-hold after the previous bypassed edge.
    constexpr bool kBypass = UTPP_P1_MUTATION != 1, kNeedHold = UTPP_P1_MUTATION != 4, kSpeedLimit = UTPP_P1_MUTATION != 3; // offline mutations
    if (c.stanceCleanEdge && kBypass && rawEdge && !m.stanceProvisional && !m.slideActive && m.slideExitAge >= c.slideExitMapS &&
        CleanStanceDetail(aPsmDetail) && CleanStanceDetail(detailLast) && (prevRunAge >= c.stanceCleanHoldS || !kNeedHold) &&
        (raw == 0 || aSpeedMs < c.stanceCleanMaxSpeedMs || !kSpeedLimit)) // a crouch ENTRY above the crouch walk speed stays filtered
    {
        m.stanceOut = raw;
        m.stanceCandAge = 0.0f;
        return raw;
    }
    m.stanceProvisional = false;
    m.stanceCandAge += aDt;
    if (m.stanceCandAge >= c.stanceFilterS)
    {
        m.stanceOut = raw;
        m.stanceCandAge = 0.0f;
    }
    return m.stanceOut;
}

// Ballistic time to land: h + vz t - g t^2 / 2 = 0 (h >= 0 metres above ground, vz up). No hit -> ttlNoHit.
// Cross-check helper only (tests); MapToChooser uses the trajectory floor prediction like motion.
inline float TimeToLand(float aHeight, float aVz, float aGravity, float aNoHit)
{
    if (!(aHeight >= 0.0f) || !(aGravity > 0.0f) || !std::isfinite(aVz))
        return aNoHit;
    const float disc = aVz * aVz + 2.0f * aGravity * aHeight;
    const float t = (aVz + std::sqrt(disc)) / aGravity;
    return std::isfinite(t) ? (t < aNoHit ? t : aNoHit) : aNoHit;
}

// One frame of ABP Update_Logic (pseudo l. 2264-2266): Update_Trajectory -> Update_EssentialValues -> Update_States, then
// the chooser inputs. motion units inside (cm, cm/s, deg); the game feed is converted with GameVelocityToCm (horizontal x
// SpeedScaleFor, vertical x 100) so database speeds and motion thresholds compare on the same scale.
inline motion::ChooserState MapToChooserCore(const MotionFeed& f, const MapConfig& c, MapState& m, const MapContext& x)
{
    motiontraj::States& st = m.st;
    const float dt = std::isfinite(f.dt) && f.dt > 0.0f ? f.dt : 0.0f;
    float velIn[3];
    for (int i = 0; i < 3; ++i)
        velIn[i] = std::isfinite(f.velWorld[i]) ? f.velWorld[i] : 0.0f;
    const bool slideDeclared = c.slide && c.slideSpec.declared;
    // motionfix item 7: CrouchSprint = Stand + Sprint. fix/slide-exit: single-feed flicker / Dodge filter (FilteredStance); slide
    // frames and the first frame after a slide (m.slideActive still holds the previous frame here) are not filtered.
    const float gameSp = std::sqrt(velIn[0] * velIn[0] + velIn[1] * velIn[1]);
    const int stance = FilteredStance(c, m, f.psmDetailed, PsmCrouchStance(f.psmDetailed) ? 1 : 0,
                                      slideDeclared && (PsmSlide(f.psmDetailed) || m.slideActive), dt, gameSp);

    // CMC MovementMode: Falling = InAir; Climb / Vault count as ground (motion: MOVE_Flying -> OnGround, Update_States l. 1804-
    // 1813); UTPPFeedAir !onGround also counts (runtime AirState leads the PSM by a frame).
    bool airFeed = f.air.valid && !f.air.onGround && !PsmTraversal(f.psmDetailed);
    // fix/slide-exit (MapConfig::slideAirGraceS): in the slide context a !onGround feed the PSM does not confirm is a ground drop
    m.airGraceActive = false;
    if (slideDeclared && c.slideAirGraceS > 0.0f)
    {
        if (airFeed && !PsmAir(f.psmDetailed) && (PsmSlide(f.psmDetailed) || m.slideActive || m.slideExitAge < c.slideExitMapS))
        {
            m.slideAirAge += dt;
            if (m.slideAirAge < c.slideAirGraceS)
            {
                airFeed = false;
                m.airGraceActive = true;
            }
        }
        else
            m.slideAirAge = 0.0f;
    }
    // MapConfig::groundDropGraceS: a contact loss out of a ground frame that the PSM does not confirm (no air detail) and that does not
    // rise stays ground until it lasted groundDropGraceS over at least kGroundDropMinFeeds feeds, or the capsule fell kGroundDropFallM.
    // The vehicle exit / jump-out flight keep their own rules below.
    {
        const bool dropCandidate = c.groundDropGraceS > 0.0f && f.air.valid && !f.air.onGround && !PsmTraversal(f.psmDetailed) &&
                                   !PsmAir(f.psmDetailed) && velIn[2] < kExitAirRiseMs && !(f.air.distanceToGround > kGroundDropProbeM) &&
                                   st.movementMode == 0 && !x.vehicleExit && !x.jumpOutFlight;
        if (dropCandidate)
        {
            m.groundDropAge += dt;
            ++m.groundDropFeeds;
            if (velIn[2] < 0.0f)
                m.groundDropM -= velIn[2] * dt;
            const bool fall = m.groundDropFeeds >= kGroundDropMinFeeds &&
                              (m.groundDropAge >= c.groundDropGraceS || m.groundDropM >= kGroundDropFallM);
            if (!fall && airFeed)
            {
                airFeed = false;
                m.airGraceActive = true;
            }
        }
        else
        {
            m.groundDropAge = m.groundDropM = 0.0f;
            m.groundDropFeeds = 0;
        }
    }
    bool psmAir = PsmAir(f.psmDetailed);
    // groundDropGraceS: the PSM Fall detail on a feed whose fresh contact still reports the ground, out of a ground frame, is no fall yet (the
    // PSM flickers it for one feed at a ledge edge); a real fall loses the contact on the same or the next feed and is InAir then.
    if (c.groundDropGraceS > 0.0f && psmAir && f.psmDetailed == kPsmFall && f.air.valid && f.air.onGround && st.movementMode == 0 && !x.vehicleExit &&
        !x.jumpOutFlight)
        psmAir = false;
    // only an air episode that BEGAN inside the window without a rise is graced (st.movementMode is still last frame's here): the
    // descending half of a real jump-out never goes back to ground
    const bool exitGraceActiveLast = m.exitGraceActive;
    m.exitGraceActive = false;
    if (c.exitAirGraceS > 0.0f && x.vehicleExit && (airFeed || psmAir) && f.psmDetailed != kPsmJump && velIn[2] < kExitAirRiseMs &&
        (st.movementMode == 0 || exitGraceActiveLast))
    {
        m.exitAirAge += dt;
        if (m.exitAirAge < c.exitAirGraceS)
        {
            airFeed = psmAir = false;
            m.exitGraceActive = true;
        }
    }
    else
        m.exitAirAge = 0.0f;
    // after a ragdoll: a small drop to the floor stays ground (no Jumps pick, no JustLanded 1 -> 0 edge). The distance guard keeps a
    // real fall off a ledge in the air; a real jump (PSM Jump 18, rising) is never touched.
#if UTPP_RAGDOLL_RECOVER_MUTATION == 5
    const bool recoverDropOk = true; // mutation 5: no distance guard (a ledge fall stays on the ground)
#else
    const bool recoverDropOk = f.air.valid && f.air.distanceToGround >= 0.0f && f.air.distanceToGround <= kRagdollRecoverMaxDropM;
#endif
#if UTPP_RAGDOLL_RECOVER_MUTATION == 8
    const bool recoverSlow = true; // mutation 8: no downward-speed condition (the last 0.5 m of a real fall is swallowed)
#else
    const bool recoverSlow = velIn[2] > -kContactFallMs; // only a small settle drop: by the last 0.5 m a real fall is faster than this
#endif
    if (x.ragdollRecover && (airFeed || psmAir) && f.psmDetailed != kPsmJump && velIn[2] < kExitAirRiseMs && recoverDropOk && recoverSlow)
        airFeed = psmAir = false;
    // review r2 finding 16: the jump-out flag counts for the flight only (until its touchdown)
    if (!x.jumpOutFlight)
    {
        m.jumpOutSeenAir = false;
        m.jumpOutGroundAge = 0.0f;
        m.jumpOutLanded = false;
    }
    // JumpPickTest mutation 20: the flag stays on after the jump-out touchdown for the rest of the exit window (pre-fix)
    const bool jumpOutFlight = x.jumpOutFlight && (UTPP_OR(JUMPPICK, 20) !m.jumpOutLanded);
    // fix/jumpout-cam (MapConfig::contactHoldS): contact feeds in the fast fall of a vehicle jump-out stay InAir for a few feeds
    // JumpPickTest mutation 18: no contact hold (the one-feed contact reaches the chooser as a landing)
    UTPP_IF_MUT(JUMPPICK, 18)
    {
        // no contact hold
    }
    else if (c.contactHoldS > 0.0f && (x.vehicleExit || jumpOutFlight) && !airFeed && !psmAir && st.movementMode == 1 && velIn[2] < -kContactFallMs &&
             m.contactHoldAge < c.contactHoldS)
    {
        m.contactHoldAge += dt;
        airFeed = true;
    }
    else if (airFeed || psmAir || velIn[2] >= -kContactFallMs || st.movementMode != 1)
        m.contactHoldAge = 0.0f;
    // JS1 take-off hold (MapConfig::jumpTakeoffHoldS): decided in MapToChooser (TakeoffHoldFeed), which hands this function the held feed
    // with the previous feed's PSM detail, so it maps as that ground frame (mode 0) and nothing below sees the Jump 18.
#if UTPP_LOCOSTART_MUTATION == 4
    if (m.takeoffHoldActive)
        psmAir = false; // mutation 4: the first JS1 hold (OnGround, but the detail stays 18: Run gait, no visualStrafe / analog floor)
#endif
    const int mode = (psmAir || airFeed) ? 1 : 0;
    if (x.jumpOutFlight)
    {
        if (mode == 1)
        {
            m.jumpOutSeenAir = true;
            m.jumpOutGroundAge = 0.0f;
        }
        else if (m.jumpOutSeenAir)
        {
            m.jumpOutGroundAge += dt;
            if (m.jumpOutGroundAge >= kJumpOutLandConfirmS)
                m.jumpOutLanded = true;
        }
    }
    // Slide feed: PSM detail 5 on the ground (the chooser-side MovementMode / Stance / Direction are applied at the end, behind
    // MapConfig::slide; everything else below keeps seeing the PSM as before)
    const bool slideActive = c.slide && c.slideSpec.declared && PsmSlide(f.psmDetailed) && mode == 0;
    const float calibrationWeight = m.stanceCalibration.Update(stance,
        mode == 0 && PsmGround(f.psmDetailed), c.stanceSpeedBlendTime, dt);
    float scale = BlendedSpeedScaleFor(c, calibrationWeight, gameSp);
    if (c.slide && c.slideSpec.declared)
    {
        // m.slideActive still holds the previous frame here (written at the end of this function)
        m.slideExitAge = (m.slideActive && !slideActive) ? 0.0f : m.slideExitAge + dt;
        if (gameSp > 0.01f)
        {
            if (slideActive)
            {
                const float u = std::fmin(1.0f, std::fmax(0.0f, (gameSp - c.slideLowMs) / (c.slideHighMs - c.slideLowMs)));
                const float target = gameSp < c.slideLowMs ? c.slideLoopCmS * gameSp / c.slideLowMs :
                                                              c.slideLoopCmS + (c.slideEntryCmS - c.slideLoopCmS) * u;
                scale = target / (gameSp * 100.0f);
            }
            else if (mode == 0 && m.slideExitAge < c.slideExitMapS)
            {
                const float k = m.slideExitAge / c.slideExitMapS;
                const float normal = gameSp * 100.0f * scale;
                scale = (c.slideExitStartCmS + (normal - c.slideExitStartCmS) * k) / (gameSp * 100.0f);
            }
        }
    }
    const motiontraj::V3 vel = motiontraj::GameVelocityToCm(velIn, scale);
    const float sp = motiontraj::SizeXY(vel);
    const bool visualStrafe = c.visualStrafe && mode == 0 && PsmGround(f.psmDetailed);
    // fix/slide-exit: CrouchSprint is sprint only once the (filtered) stance is Stand; with the filter off stance == Stand there
    bool sprintInput = PsmSprint(f.psmDetailed) && !visualStrafe // No strafe Sprint assets.
#if UTPP_SLIDEEXIT_MUTATION != 5
        && !(stance == 1) // a Sprint feed (4 or CrouchSprint 30) inside the filtered crouch is no sprint yet
#endif
        ;
    // Dodge gait hold (MapConfig::dodgeGaitHoldS): the sprint input of the feed before a ground dodge is kept through the dodge detail and
    // its decay (ground feeds while the speed keeps falling, at most dodgeGaitHoldS after the detail); the chooser keeps its dodge flag
    // over the decay. Mutation UTPP_DODGE_GAIT_MUTATION 1: no hold (the detail drops the sprint input as before).
    bool dodgeHeld = false;
    {
        const bool detail = f.psmDetailed == kPsmDodge;
#if UTPP_DODGE_GAIT_MUTATION != 1
        const bool on = c.dodgeGaitHoldS > 0.0f;
#else
        const bool on = false;
#endif
        if (on && detail)
        {
            if (!m.dodgePrevDetail)
                m.dodgeSprint = m.dodgePrevSprint;
            m.dodgeDecay = true;
            m.dodgeDecayS = 0.0f;
        }
        else if (on && m.dodgeDecay && mode == 0 && PsmGround(f.psmDetailed) && gameSp < m.dodgePrevSpeed && m.dodgeDecayS + dt <= c.dodgeGaitHoldS)
        {
            m.dodgeDecayS += dt;
            dodgeHeld = true;
        }
        else
            m.dodgeDecay = m.dodgeSprint = false;
        // the sprint input of the feed before the dodge (kept also through an air feed inside the detail), on ground feeds only
        if ((dodgeHeld || (on && detail)) && m.dodgeSprint && mode == 0 && !visualStrafe && stance != 1)
            sprintInput = true;
        m.dodgePrevDetail = detail;
        m.dodgePrevSpeed = gameSp;
        m.dodgePrevSprint = sprintInput;
    }

    // CBP UpdateMovement_PreCMC (l. 1130-1140): Gait = GetDesiredGait() from the input, then MaxAcceleration of that gait.
    // [P] same-frame inputs: motion sets MaxAcceleration / GroundFriction / MaxWalkSpeed (CalculateMaxSpeed) in PreCMC, before
    // the CMC tick, so they read LAST frame's CharacterMovement.Velocity and K2_GetActorRotation (CBP l. 1190-1192, 1202-1209); here
    // they use this frame's mapped velocity (sp, vel) and feed playerYaw. Reason: the feed is sampled after the game's
    // movement, one 60 Hz frame of lag is not modelled; it only matters while speed or direction changes across a curve key.
    float stickRaw = std::sqrt(f.move[0] * f.move[0] + f.move[1] * f.move[1]);
    stickRaw = std::isfinite(stickRaw) ? stickRaw : 0.0f;
    const motiontraj::V3 dir = InputAcceleration(f, 1.0f, c.stickMode);
    const float inputYaw = motiontraj::IsZero(dir) ? 0.0f : std::atan2(-dir.x, dir.y) / integ::kDegToRad; // forward(psi) = (-sin, cos)
    // Actor yaw (MapConfig::FacingSource). yawPre = K2_GetActorRotation as the PreCMC functions read it (gait, CalculateMaxSpeed
    // direction: before the CMC tick), yawNow = after CMC PhysicsRotation (ABP Update_Trajectory facing, root delta). Game: both
    // = feed playerYaw (unchanged behaviour).
    float yawPre = f.playerYawDeg, yawNow = f.playerYawDeg;
    // RotationMode. Moved here from before the input block (nothing between read it): the H2 safeguard needs the input yaw and the actor yaw.
    // Shooter mode: Strafe (motion's default) except in the slide itself and in traversal. Orient only while the PSM reports the slide (or the
    // previous feed was one): after it the exit window already strafes; the lock of the root onto the actor is rate bounded, so the
    // slide-heading root is not thrown onto the camera (MotionDriver: rootLockRate).
    const bool baseStrafe = (c.strafeWhileAiming && f.aiming) || visualStrafe;
    const bool slideNow = PsmSlide(f.psmDetailed) || m.slideActive;
#if UTPP_SHOOTER_MODE_MUTATION == 1
    const bool shooterModeActive = false;
#else
    const bool shooterModeActive = c.shooterMode && !slideNow && !PsmTraversal(f.psmDetailed);
#endif
    bool h2Active = false;
    if (c.h2Strafe && !c.shooterMode)
    {
        const bool slideWindow = slideNow || (slideDeclared && m.slideExitAge < c.slideExitMapS);
#if UTPP_SHOOTER_MODE_MUTATION == 5
        m.h2OffAge = m.h2OffAge + dt; // mutation 5: the gate is always open (the body-turn switch is ignored)
#else
        m.h2OffAge = x.bodyTurnSwitchedOff ? m.h2OffAge + dt : 0.0f;
#endif
#if UTPP_SHOOTER_MODE_MUTATION == 8
        const bool rawHip = false;
#else
#if UTPP_SHOOTER_MODE_MUTATION == 12
        const bool rawHip = x.hipFireAny; // mutation 12: the CET lease is no exclusion
#else
        const bool rawHip = x.hipFireAny || x.hipLease;
#endif
#endif
        const bool excluded = f.aiming || visualStrafe || rawHip || slideWindow || PsmTraversal(f.psmDetailed) || f.psmDetailed == kPsmAimWalk;
#if UTPP_SHOOTER_MODE_MUTATION == 6
        m.h2ClearAge = m.h2ClearAge + dt;
        (void)excluded;
#else
        m.h2ClearAge = excluded ? 0.0f : m.h2ClearAge + dt;
#endif
        const bool input = !motiontraj::IsZero(dir) && stickRaw >= kH2MinStick;
        const float off = input && std::isfinite(yawPre) ? std::fabs(motiontraj::NormalizeAxis(inputYaw - yawPre)) : 0.0f;
#if UTPP_SHOOTER_MODE_MUTATION == 5
        const bool switched = true;
#else
        const bool switched = x.bodyTurnSwitchedOff;
#endif
        const bool gate = switched && m.h2OffAge >= kH2OffHoldS && m.h2ClearAge >= kH2ClearHoldS;
        if (!gate)
            m.h2Latched = false;
        else if (!m.h2Latched)
        {
            m.h2Latched = mode == 0 && PsmGround(f.psmDetailed) && (stance == 0 || stance == 1) && input && off >= kH2EnterDeg &&
                          gameSp >= kH2MinSpeedMs;
            m.h2NoInputAge = 0.0f;
        }
        else if (input)
        {
            m.h2NoInputAge = 0.0f;
            if (off < kH2ExitDeg)
                m.h2Latched = false;
        }
        else if ((m.h2NoInputAge += dt) > kH2ReleaseS)
            m.h2Latched = false;
        h2Active = m.h2Latched;
    }
    else
    {
        m.h2Latched = false;
        m.h2OffAge = 0.0f;
        m.h2ClearAge = 1.0e9f;
        m.h2NoInputAge = 0.0f;
    }
    m.shooterModeActive = shooterModeActive;
    m.h2Active = h2Active;
    const motiontraj::RotationMode rot =
        (baseStrafe || shooterModeActive || h2Active) ? motiontraj::RotationMode::Strafe : motiontraj::RotationMode::OrientToMovement;
    if (c.facingSource == FacingSource::MotionActor)
    {
        if (!m.motionYawValid)
        {
            m.motionYawDeg = std::isfinite(f.playerYawDeg) ? f.playerYawDeg : 0.0f;
            m.motionYawValid = true;
        }
        yawPre = m.motionYawDeg;
        bool haveTarget = false;
        float target = m.motionYawDeg;
        if (rot == motiontraj::RotationMode::Strafe)
        {
            haveTarget = std::isfinite(f.cameraYawDeg); // controller desired rotation
            target = haveTarget ? f.cameraYawDeg : target;
        }
        else if (!motiontraj::IsZero(dir))
        {
            haveTarget = true; // ComputeOrientToMovementRotation: acceleration direction
            target = inputYaw;
        }
        if (haveTarget)
        {
            const float delta = motiontraj::NormalizeAxis(target - m.motionYawDeg);
            const float maxStep = mode == 1 ? 200.0f * dt : 360.0f; // RotationRate yaw 200 falling / -1 (instant) on ground
            m.motionYawDeg = motiontraj::NormalizeAxis(m.motionYawDeg + std::fmax(-maxStep, std::fmin(maxStep, delta)));
        }
        yawNow = m.motionYawDeg;
    }
    m.actorYawDeg = yawNow;
#if UTPP_SHOOTER_MODE_MUTATION == 7
    const bool sprintCone = rot == motiontraj::RotationMode::OrientToMovement || shooterModeActive; // mutation 7: no CanSprint cone in Shooter
#else
    const bool sprintCone = rot == motiontraj::RotationMode::OrientToMovement;
#endif
    // G3: st.gait is still the PREVIOUS frame's gait here (st.gaitLast = st.gait runs below, in Update_States)
    const bool gaitHyst = c.gaitHysteresis;
    const bool wantsToWalk = f.wantsToWalk || (f.aimWalkVanilla && f.psmDetailed == kPsmAimWalk); // G9
    int gait = GaitFromInput(sprintInput, wantsToWalk, motiontraj::IsZero(dir) ? 0.0f : stickRaw, c.stickMode,
                             c.analogWalkRunThreshold, sprintCone, yawPre, inputYaw, gaitHyst ? st.gait : -1,
                             gaitHyst ? c.gaitHysteresisUp : 0.0f, gaitHyst ? c.gaitHysteresisDown : 0.0f);
    // Gait return hold (MapConfig::gaitReturnHoldS): a stick that returns after a short dip below the Walk / Run threshold - it passes the
    // centre when the direction reverses (1 -> 0.78 -> 0 -> 0.37 -> 1 within 0.1 s) - keeps the Run gait on its way back up while the capsule is
    // still running, so the Walk pivot of the dip is not picked a few frames before the Run pivot (Walk_Turn_L_180 -> Run_Turn_R_180 17 ms apart).
    // Only a RISING stick is held: a stick that falls (run -> walk, release) changes the gait at once as before, and a rising stick the capsule
    // has not been running on (a start, a walk) is untouched.
    // The hold clock runs from the last REAL Run frame (gaitRaw, before the forced value): a forced frame must not reset it, or a slowly rising stick would
    // hold the gait for ever; and the frame after a forced one (st.gait == 1 because of the hold, m.gaitHeld) continues the hold instead of falling back to Walk.
    const float stickNow = motiontraj::IsZero(dir) ? 0.0f : stickRaw;
    const int gaitRaw = gait;
    // "rising" is a rate (stick per second, MapConfig::gaitReturnRiseRate), so the hold does not depend on the frame time
    // (mutation 31: the former fixed rise of 0.005 per feed, 0.3 per second only at 60 feeds per second)
    const float riseMin = UTPP_TN_JUMPPICK(31, 0.005f, c.gaitReturnRiseRate * std::fmax(dt, 0.0f));
    const bool held = UTPP_AND_NOT(JUMPPICK, 27) c.gaitReturnHoldS > 0.0f && gait == 0 && (st.gait == 0 || m.gaitHeld) && stickNow > m.prevStick + riseMin &&
                      m.sinceRunS < c.gaitReturnHoldS && gameSp > kGaitHoldRunSpeed && !wantsToWalk && mode == 0 && stance == 0 && st.stance == 0 &&
                      !m.takeoffHoldActive;
    if (held)
        gait = 1;
    m.gaitHeld = held;
    m.prevStick = stickNow;
    m.sinceRunS = gaitRaw >= 1 ? 0.0f : m.sinceRunS + dt; // Sprint counts as running: a sprint flick passes the centre too (Male 17:19:16 flick, Walk_Turn_L_180 before Run_Turn_R_180)

    // Acceleration (input acceleration, CMC GetCurrentAcceleration) with the motion max acceleration of the gait (CBP CMC);
    // AnalogInputModifier = |Acceleration| / MaxAcceleration (CMC ComputeAnalogInputModifier; UE PoseSearchTrajectoryLibrary.cpp
    // l. 73 MaxSpeed = max(GetMaxSpeed() x AnalogInputModifier, MinAnalogSpeed)) -> 1 with input in the FixedSpeed modes.
    const float maxAccel = motiontraj::MotionMaxAcceleration(c.cmc, gait, sp);
    motiontraj::V3 acc = InputAcceleration(f, maxAccel, c.stickMode);
    // minimum analog walk speed: below analogFloorStick the script lifts the capsule (same gate: ground, Stand / Crouch, no sprint, no
    // AimWalk) to the fraction AnalogFloorLiftFraction (the floor; 4 x the stick's own speed where its multiplier is capped), so the query
    // behaves as at the stick of that fraction (liftStick, the same extended curve both ways): the cap of that speed (GameAnalogModifierFrac
    // below) and, in the VariableSpeed modes, the input acceleration of that stick (|acc| = maxAccel x |stick|: at |mv| 0.09 the raw
    // acceleration let the prediction rise to only ~28 cm/s, IsStarting stayed false and the first pick was a Walk_Pivots row instead of
    // the Walk_Starts of a half-stick start). Not capped (|mv| 0.09 at 0.82 m/s): liftStick = analogFloorStick and the cap of that stick, the
    // first version bit for bit. The direction and IsZero(acc) are unchanged; at and above analogFloorStick nothing changes (identical).
    // Mutation 2 (UTPP_ANALOGFLOOR_MUTATION=2): no driver lift at all (acc and cap of the raw stick).
#if UTPP_ANALOGFLOOR_MUTATION == 2
    const bool analogFloor = c.analogFloorStick < 0.0f; // mutation 2: never (a runtime value, no constant condition)
#else
    const bool analogFloor = c.analogFloorStick > 0.0f && c.analogFloorFrac > 0.0f && mode == 0 &&
                             !sprintInput && (f.psmDetailed == kPsmStand || f.psmDetailed == kPsmCrouch) && !motiontraj::IsZero(acc) &&
                             stickRaw > 0.0f && stickRaw < c.analogFloorStick;
#endif
    float liftFrac = 0.0f, liftStick = 0.0f;
    if (analogFloor)
    {
        liftFrac = AnalogFloorLiftFraction(stickRaw, c.analogFloorFrac, c.analogFloorMulMax);
        liftStick = liftFrac == c.analogFloorFrac ? c.analogFloorStick : AnalogFloorStickForFraction(liftFrac);
        if (!StickModeFixedSpeed(c.stickMode) && liftStick > stickRaw)
        {
            const float lift = std::fmin(liftStick, 1.0f) / stickRaw;
            acc = {acc.x * lift, acc.y * lift, acc.z * lift};
        }
    }
    float stick = (motiontraj::IsZero(acc) || !(maxAccel > 0.0f)) ? 0.0f : std::fmin(motiontraj::Size(acc) / maxAccel, 1.0f);
    if (c.gameAnalogModifier && !motiontraj::IsZero(acc))
    {
        // the floor's own stick on the measured part of kGameStickMotion (>= its first point, where both curves agree): the stick path of the
        // first version; a capped lift or a floor below that point: the fraction of the extended curve (kGameStickMotion is linear from 0
        // there and would predict below the real capsule speed)
        if (analogFloor && !(liftStick == c.analogFloorStick && liftStick >= kGameStickMotion[0]))
            stick = GameAnalogModifierFrac(c, gait, stance, sprintInput, std::fmax(liftFrac, AnalogFloorStickFraction(stickRaw)));
        else
            stick = GameAnalogModifier(c, gait, stance, sprintInput, std::fmin(analogFloor ? liftStick : stickRaw, 1.0f));
    }
    // Slide: the held stick does not drive the slide (the capsule brakes by itself), so the QUERY TRAJECTORY gets no input
    // acceleration and brakes linearly at the measured slide rate (SlideStopPrediction below) instead of accelerating towards the
    // Run cap. The states (IsMoving needs the input acceleration, st.acceleration) keep the real input.
    const bool slideBrake = slideActive && c.slideDeceleration > 0.0f;
    const motiontraj::V3 trajAcc = slideBrake ? motiontraj::V3{} : acc;
    if (slideBrake)
        stick = 0.0f;

    // Update_Trajectory (l. 1740-1747): history from the capsule, CMC ground prediction; in air the gravity drop with the
    // floor at -distanceToGround (UTPPFeedAir; no hit -> TimeToLand = last sample time).
    // Facing (query trajectory facing, finding 8): actor (mesh) yaw = feed playerYawDeg, control yaw = feed cameraYawDeg
    // (GetViewRotation, PSTL l. 95), both converted with GameYawToTrajYaw; generation data from Speed2D of the PREVIOUS frame
    // (ABP l. 1741: Update_Trajectory runs before Update_EssentialValues, Update_Logic l. 2264-2266). CalculateMaxSpeed
    // direction = CalculateDirection(Velocity, ActorRotation) (CBP l. 1205-1209).
    // [P] actor yaw = the game's playerYaw, not motion's: in OrientToMovement on the ground motion turns the actor to the
    // acceleration instantly (CBP UpdateRotation_PreCMC l. 1156: RotationRate (0, -1, 0)), so its t = 0 / history facing is already
    // near the predicted facings, which snap to the acceleration yaw (RotateTowardsMovementSpeed 0, ABP l. 31-32; PSTL
    // l. 463-469 QInterpConstantTo speed 0 = target). The game's playerYaw lags during turns, so heading features at
    // 0 / 0.35 / 0.70 can show a turn motion would not. Reason: the pose is rendered on the game mesh at playerYaw, so the
    // query history / current facing must be the game's; untested beyond the steady and strafe query checks.
    const float actorYaw = motiontraj::GameYawToTrajYaw(std::isfinite(yawNow) ? yawNow : 0.0f);
    const float actorYawPre = motiontraj::GameYawToTrajYaw(std::isfinite(yawPre) ? yawPre : 0.0f);
    const float controlYaw = motiontraj::GameYawToTrajYaw(std::isfinite(f.cameraYawDeg) ? f.cameraYawDeg : 0.0f);
    motiontraj::FacingInput facing;
    facing.valid = true;
    facing.actorYawDeg = actorYaw;
    facing.controlYawDeg = controlYaw;
    facing.gen = motiontraj::SelectGenerationData(st.speed2D);
    // A stationary rendered body may lag the gameplay capsule while its feet
    // turn. Query that visible heading, then predict toward the held view even
    // after the mouse stops; capsule heading alone makes every future flat.
    facing.aimTurnInPlace = x.aimFacing && x.rootYawValid && mode == 0 &&
        PsmGround(f.psmDetailed) && motiontraj::IsZero(acc) && sp <= motiontraj::kHasVelocity;
    if (facing.aimTurnInPlace)
        facing.actorYawDeg = motiontraj::GameYawToTrajYaw(x.rootYawDeg);
    facing.shotTurn = facing.aimTurnInPlace && x.shotTurn;
    if (facing.shotTurn)
        facing.controlYawDeg = motiontraj::GameYawToTrajYaw(x.shotTargetYawDeg);
    // Rest TIP (review of 16d03c3: the slow recentering slid the feet 0.3-0.4 m for 157 deg): the same query as the aim turn, with
    // the visible root as the actor and the capsule as the goal, admitted by ShouldTurnInPlace below. Standing / crouching only.
#if defined(UTPP_REST_TURN_MUTATION) && UTPP_REST_TURN_MUTATION == 1
    const bool restTurn = false; // mutation 1: the rest TIP is never admitted
#else
    const bool restTurn = x.restTurn && !facing.aimTurnInPlace && x.rootYawValid && std::isfinite(yawNow) && mode == 0 &&
        PsmGround(f.psmDetailed) && (stance == 0 || stance == 1) && motiontraj::IsZero(acc) && sp <= motiontraj::kHasVelocity &&
        motiontraj::RootDeltaYaw(yawNow, x.rootYawDeg) >= motiontraj::kTurnInPlaceDeg;
#endif
    if (restTurn)
    {
        facing.aimTurnInPlace = true;
        facing.actorYawDeg = motiontraj::GameYawToTrajYaw(x.rootYawDeg);
        facing.controlYawDeg = actorYaw; // the capsule
    }
    motiontraj::Derived d = motiontraj::MakeDerived(c.cmc, gait, stance, stick, sp, trajAcc, rot == motiontraj::RotationMode::OrientToMovement,
                                                mode == 1, motiontraj::CalculateDirection(vel, actorYawPre));
    if (mode == 0 && PsmGround(f.psmDetailed) && motiontraj::IsZero(acc))
        d.capsuleStop = CapsuleStopPrediction(c, calibrationWeight, gameSp);
    else if (slideBrake)
        d.capsuleStop = SlideBrakePrediction(c, gameSp);
    // Native Acceleration is a scalar stat, not a measurement of turning
    // friction, analog response, or stance blending. Admit only the straight
    // full-stick domain whose physical ramp is represented by that sample.
    constexpr float alignedCos = .9961947f; // five degrees, excludes redirection / pivots
    const motiontraj::V3 planarVelocity{vel.x, vel.y, 0.f};
    const bool alignedVelocity = gameSp <= .01f ||
        motiontraj::Dot(motiontraj::SafeNormal(planarVelocity), motiontraj::SafeNormal(dir)) >= alignedCos;
    const bool alignedActor = std::isfinite(yawNow) &&
        std::fabs(motiontraj::NormalizeAxis(inputYaw - yawNow)) <= 5.f;
    if (dt > 0.f && mode == 0 && PsmGround(f.psmDetailed) && stickRaw >= .99f &&
        rot == motiontraj::RotationMode::OrientToMovement && alignedActor && alignedVelocity &&
        std::fabs(calibrationWeight - float(stance)) <= .0001f)
    {
        float gameCap = stance == 1 ? c.gameCrouchPoint : c.gameStandPoints[gait];
        if (stance != 1 && gait == 1)
            gameCap -= c.gameRunReductionMs; // G5 Tier2 Slow: the run cap is 3.85 - 1.3 (0 = unchanged)
        if (c.speedScale > 0.f) gameCap = d.maxSpeed / (100.f * c.speedScale);
        d.capsuleStart = CapsuleStartPrediction(c, calibrationWeight, gameSp, gameCap);
        // Keep custom/analog cap policies authoritative if their query cap does
        // not equal the calibrated native gait cap.
        if (std::fabs(d.capsuleStart.QuerySpeed(gameCap) - d.maxSpeed) > .1f)
            d.capsuleStart.acceleration = 0.f;
    }
    m.capsuleBrakeModel = d.capsuleStop.Active() ? d.capsuleStop.deceleration : 0.0f;
    const bool hasFloor = f.air.valid && f.air.distanceToGround >= 0.0f;
    m.traj.Update(dt, vel, trajAcc, d, c.gravity * 100.0f, hasFloor, hasFloor ? -f.air.distanceToGround * 100.0f : 0.0f, facing);
    m.capsuleAccelModel = m.traj.CapsuleAcceleration();
    st.trjPast = m.traj.PastVelocity();
    st.trjCurrent = m.traj.CurrentVelocity();
    st.trjFuture = m.traj.FutureVelocity();

    // Update_EssentialValues (l. 1755-1766): Acceleration / Velocity with _LastFrame copies, Speed2D, HasVelocity
    st.accelerationLast = st.acceleration;
    st.acceleration = acc;
    st.velocityLast = st.velocity;
    st.velocity = vel;
    st.speed2D = sp;
    st.hasVelocity = sp > motiontraj::kHasVelocity;

    // Update_States (l. 1801-1840): MovementMode, RotationMode, MovementState (IsMoving), Gait, Stance with _LastFrame copies
    st.movementModeLast = st.movementMode;
    st.movementMode = mode;
    st.rotationModeLast = st.rotationMode;
    st.rotationMode = rot;
    st.isMoving = motiontraj::IsMoving(st.trjFuture, acc);
    st.movementStateLast = st.movementState;
    st.movementState = st.isMoving ? 1 : 0;
    st.gaitLast = st.gait;
    st.gait = gait;
    st.stanceLast = st.stance;
    st.stance = stance;

    // CBP OnLanded (l. 889-890, 918-923): LandVelocity = CMC Velocity at Falling -> Walking; JustLanded = true for 0.3 s
    // (RetriggerableDelay). The CMC calls Landed before SetPostLandedPhysics zeroes vz; the game feed may already report the
    // ground velocity on the landing frame, so the larger downward vz of {this frame, last air frame} is used.
    if (mode == 1)
        m.lastAirVzCmS = vel.z;
    if (st.movementMode == 0 && st.movementModeLast == 1)
    {
        m.sinceLand = 0.0f;
        st.landVz = std::fmin(vel.z, m.lastAirVzCmS);
        // the motion 700 cm/s threshold rescaled to this feed's learned gravity (same 2.5 m drop); latched here so the up / down
        // gravity of the feed cannot move it inside the 0.3 s JustLanded window
        m.landHeavyCmS = motiontraj::HeavyLandThresholdCmS(c.gravity);
        // r3 landing: a jump-out flight lands at 10-12 m/s, a heavy land; its recovery is capped (st / ChooserState::jumpOutLandOver below)
        m.jumpOutHeavyLand = jumpOutFlight && std::fabs(st.landVz) >= m.landHeavyCmS;
    }
    else
        m.sinceLand += dt;
    if (st.movementMode == 1)
        m.jumpOutHeavyLand = false;
    st.justLanded = m.sinceLand < motiontraj::kJustLandedSeconds;
    st.justLandedTimer = st.justLanded ? motiontraj::kJustLandedSeconds - m.sinceLand : 0.0f;

    // predicates (ABP functions l. 1777-1937)
    st.turnAngle = motiontraj::TurnAngle(st.trjFuture, vel);
    st.isStarting = motiontraj::IsStarting(st.isMoving, st.trjFuture, vel, x.currentDbPivots);
    st.isPivoting = motiontraj::IsPivoting(st.isMoving, st.trjFuture, vel, rot);
    const bool stationaryAimTurn = facing.aimTurnInPlace && f.aiming;
    const bool stationaryRestTurn = restTurn && !stationaryAimTurn;
    const float rootTargetYaw = facing.shotTurn ? x.shotTargetYawDeg : stationaryAimTurn ? f.cameraYawDeg : yawNow;
    const float rootDelta = x.rootYawValid ? motiontraj::RootDeltaYaw(rootTargetYaw, x.rootYawDeg) : 0.0f;
    st.shouldTurnInPlace = motiontraj::ShouldTurnInPlace(rootDelta, f.aiming || visualStrafe || stationaryRestTurn, st.movementState,
                                                    st.movementStateLast, false);
    if (facing.shotTurn)
        st.shouldTurnInPlace = rootDelta > shotturn::kFinishDegrees;
    // Shooter mode / H2 safeguard and the spin transition (the 180 deg Reface clips of PSD_*_SpinTransition).
    // H2: never (the root hung 180 deg off the camera-locked capsule and the spin was picked again and again). Shooter: the visual root is
    // locked onto the actor while V moves on the ground (MotionDriver: strafeLock, rate bound), so the offset that triggers the spin (>= 130 deg
    // at >= 1.5 m/s) is resolved within a few feeds and the spin can be picked once - a start-with-turn out of a held heading or a turn
    // around while running - never repeatedly. Where the lock does not run the root can stay far from the capsule, so the spin is suppressed:
    // in the air, in the slide itself, in its exit window (the stick is pulled back there with the camera anywhere), in traversal. The
    // suppression follows the SETTING, not the windowed shooterModeActive: the windows must not be spin windows.
    const bool slideExitWindow = slideDeclared && m.slideExitAge < c.slideExitMapS;
    [[maybe_unused]] const bool fnSpinOff = c.shooterMode && !(shooterModeActive && st.isMoving && mode == 0 && !slideExitWindow);
#if UTPP_SHOOTER_MODE_MUTATION == 2
    st.shouldSpin = motiontraj::ShouldSpinTransition(rootDelta, sp, x.currentDbPivots);
#elif UTPP_SHOOTER_MODE_MUTATION == 9
    // mutation 9: the suppression follows the windowed shooterModeActive (slide, exit window and traversal are spin windows)
    st.shouldSpin = motiontraj::ShouldSpinTransition(rootDelta, sp, x.currentDbPivots) && !(shooterModeActive && !(st.isMoving && mode == 0)) && !h2Active;
#else
    st.shouldSpin = motiontraj::ShouldSpinTransition(rootDelta, sp, x.currentDbPivots) && !fnSpinOff && !h2Active;
#endif
    st.justLandedLight = motiontraj::JustLandedLight(st.justLanded, st.landVz, m.landHeavyCmS);
    st.justLandedHeavy = motiontraj::JustLandedHeavy(st.justLanded, st.landVz, m.landHeavyCmS);
    st.playLand = motiontraj::PlayLand(st.movementMode, st.movementModeLast);
    st.playMovingLand = motiontraj::PlayMovingLand(st.movementMode, st.movementModeLast, st.turnAngle);
    st.timeToLand = m.traj.TimeToLand();

    // motionfix item 7: stop on a speed collapse (kStopCollapse*): the chooser speed is held at the pre-collapse speed braking
    // at the motion rate while the state stays Idle on the ground; any input / air / new motion drops the hold.
    if (gameSp >= m.collapsePeakSp || m.collapsePeakAge > kStopCollapseWindowS)
    {
        m.collapsePeakSp = gameSp;
        m.collapsePeakAge = 0.0f;
    }
    else if (gameSp >= kStopCollapsePlateau * m.collapsePeakSp)
        m.collapsePeakAge = 0.0f; // still on the plateau
    else
        m.collapsePeakAge += dt;
    float chooserSpeed = sp;
#if !(defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 2) // mutation 2: no stop hold on a speed collapse
    if (mode == 0 && !st.isMoving)
    {
        if (m.collapsePeakSp - gameSp > kStopCollapseDeltaMs)
        {
            const float peakCmS = m.collapsePeakSp * 100.0f * BlendedSpeedScaleFor(c, calibrationWeight, m.collapsePeakSp);
            m.stopHoldCmS = std::fmax(m.stopHoldCmS, peakCmS - kStopCollapseBrakeCmS2 * m.collapsePeakAge);
            m.collapsePeakSp = gameSp; // one hold per collapse
            m.collapsePeakAge = 0.0f;
        }
        chooserSpeed = std::fmax(sp, m.stopHoldCmS);
        m.stopHoldCmS = std::fmax(0.0f, m.stopHoldCmS - kStopCollapseBrakeCmS2 * dt);
    }
    else
        m.stopHoldCmS = 0.0f;
#endif
    st.stopHoldCmS = chooserSpeed > sp ? chooserSpeed : 0.0f;

    motion::ChooserState s;
    s.stationaryShotTurn = facing.shotTurn;
#if defined(UTPP_REST_TURN_MUTATION) && UTPP_REST_TURN_MUTATION == 2
    s.stationaryAimTurn = stationaryAimTurn || stationaryRestTurn; // mutation 2: the rest TIP counts as an aim start (HoldFlags bit 0)
#else
    s.stationaryAimTurn = stationaryAimTurn;
    s.restTurn = stationaryRestTurn; // finishes an admitted same-side clip like the aim turn, but is no aim start
#endif
    s.explicitWalk = f.wantsToWalk;
    if (facing.shotTurn)
    {
        shotturn::Hold side; // R3 review L1: the error is read on the admitted side near 180 deg (same rule as the hold)
        side.side = x.shotTurnSide;
        s.shotTurnYawDeg = side.Unwrapped(std::remainder(x.shotTargetYawDeg - x.rootYawDeg, 360.f));
    }
    else
        s.shotTurnYawDeg = 0;
    const float aimGoalYaw = stationaryRestTurn ? yawNow : f.cameraYawDeg; // rest TIP: the capsule, ADS: the camera
    s.stationaryAimYawDeg = (s.stationaryAimTurn || s.restTurn) ? std::remainder(aimGoalYaw - x.rootYawDeg, 360.f) : 0;
    s.stationaryAimTargetYawDeg = (s.stationaryAimTurn || s.restTurn) ? aimGoalYaw : 0;
    s.speed2dCmS = chooserSpeed;
    s.verticalVelocityCmS = vel.z;
    s.movementState = st.movementState;
    s.gait = st.gait;
    s.stance = st.stance;
#if UTPP_SLIDEEXIT_MUTATION == 13
    s.stanceRaw = st.stance; // mutation 13: the filtered stance under the raw name
#else
    s.stanceRaw = PsmCrouchStance(f.psmDetailed) ? 1 : 0; // fix/slide-exit: the PSM stance without the debounce (ChooserState::stanceRaw)
#endif
    s.dodge = f.psmDetailed == kPsmDodge || dodgeHeld; // dodgeHeld: the decay after it (MapConfig::dodgeGaitHoldS)
    s.psmJump = integ::PsmIsJumpDetail(f.psmDetailed); // fix5/jump: game jump vs walk-off (MatcherConfig::jumpRiseNoOff)
    s.jumpOutFlight = jumpOutFlight; // fix/jumpout-cam (MatcherConfig::jumpOutNoBack / jumpOutFallHandover)
    // Chooser MovementMode / MovementMode_LastFrame / MovementDirection. Slide off: movementMode = st.movementMode (0 / 1) as
    // before and movementModeLast its previous-frame value. Slide on: MovementMode Sliding and Stance Crouch for the whole PSM
    // detail 5 (the game puts V into the crouch capsule; the stance stays Stand for the speed calibration above, which keeps the
    // sprint speed map until the PSM reports Crouch), Direction Other when the capsule velocity heads > 45 deg off the body.
    // The first frame after the slide carries movementModeLast = Sliding (previous chooserMode), once.
    const int chooserMode = slideActive ? c.slideSpec.slidingValue : st.movementMode;
    s.movementModeLast = m.chooserMode;
    m.chooserMode = chooserMode;
    s.movementMode = chooserMode;
    s.slideLastDurationS = m.slideActive && !slideActive ? m.slideAge : 0.0f; // m.slideActive: the previous frame
    m.slideAge = slideActive ? m.slideAge + dt : 0.0f;
    m.slideActive = slideActive;
    if (slideActive)
    {
        s.stance = 1;
        if (gameSp > kSlideDirectionMinSpeed)
            m.slideDirectionOther = SlideDirectionOther(std::atan2(-velIn[0], velIn[1]) / integ::kDegToRad,
                                                        std::isfinite(yawNow) ? yawNow : 0.0f, m.slideDirectionOther);
        s.movementDirection = m.slideDirectionOther ? c.slideSpec.otherValue : c.slideSpec.forwardValue;
    }
    else
        m.slideDirectionOther = false;
    s.isStarting = st.isStarting;
    s.isPivoting = st.isPivoting;
    // digital heading step (MapConfig::headingStepDeg): the wanted direction jumped from one feed to the next while it stayed valid
    {
        const bool headingNow = rot == motiontraj::RotationMode::OrientToMovement && mode == 0 && !slideActive && !motiontraj::IsZero(dir);
        const float lagNow = headingNow && std::isfinite(f.playerYawDeg) ? std::fabs(motiontraj::NormalizeAxis(inputYaw - f.playerYawDeg)) : 0.0f;
        // swept step (MapConfig::headingSweepS): the directions of the feeds of the last headingSweepS seconds (the newest kept feeds only)
        bool swept = false;
        if (UTPP_AND_NOT(JUMPPICK, 38) headingNow && m.headingValid && c.headingSweepS > 0.0f && c.headingStepDeg > 0.0f)
        {
            int keep = 0;
            for (int k = 0; k < m.sweepCount; ++k)
            {
                const float age = m.sweepAgeS[k] + dt;
                if (age > c.headingSweepS)
                    continue;
                swept = swept || std::fabs(motiontraj::NormalizeAxis(inputYaw - m.sweepYawDeg[k])) >= c.headingStepDeg;
                m.sweepYawDeg[keep] = m.sweepYawDeg[k];
                m.sweepAgeS[keep++] = age;
            }
            m.sweepCount = keep;
        }
        else
            m.sweepCount = 0;
        if (!headingNow)
            m.headingStepAgeS = 1.0e9f;
        else if (m.headingValid && c.headingStepDeg > 0.0f &&
                 (swept || std::fabs(motiontraj::NormalizeAxis(inputYaw - m.headingYawDeg)) >= c.headingStepDeg))
        {
            m.headingStepAgeS = 0.0f;
            m.headingStepFromRest = false;
            m.sweepCount = 0; // one sweep is one step
        }
        // a start from rest toward a direction away from the facing: the body catches up from the start clip as after a key step
        else if (!m.headingValid && c.headingStepDeg > 0.0f && lagNow >= c.headingStepDeg)
        {
            m.headingStepAgeS = 0.0f;
            m.headingStepFromRest = true;
        }
        else if (m.headingStepAgeS < 1.0e8f)
            m.headingStepAgeS += dt;
        if (headingNow && c.headingSweepS > 0.0f)
        {
            if (m.sweepCount == MapState::kHeadingSweepFeeds) // full: the oldest goes
            {
                for (int k = 1; k < m.sweepCount; ++k)
                    m.sweepYawDeg[k - 1] = m.sweepYawDeg[k], m.sweepAgeS[k - 1] = m.sweepAgeS[k];
                --m.sweepCount;
            }
            m.sweepYawDeg[m.sweepCount] = inputYaw;
            m.sweepAgeS[m.sweepCount++] = 0.0f;
        }
        m.headingValid = headingNow;
        m.headingYawDeg = inputYaw;
        s.headingStepAgeS = m.headingStepAgeS;
        s.headingLagDeg = lagNow;
        s.headingStepFromRest = m.headingStepFromRest;
    }
    s.shouldTurnInPlace = st.shouldTurnInPlace;
    s.shouldSpinTransition = st.shouldSpin;
    s.justLandedLight = st.justLandedLight;
    s.justLandedHeavy = st.justLandedHeavy;
    // JumpPickTest mutation 21: the heavy land of a jump-out keeps playing (no recovery cap)
    s.jumpOutLandOver = UTPP_AND_NOT(JUMPPICK, 21) m.jumpOutHeavyLand && st.movementMode == 0 && m.sinceLand >= motiontraj::kJumpOutLandRecoverS;
    s.timeToLand = st.timeToLand;
    // Climb / Vault exit derived from the detail transition (no script flag; 10.8)
    s.justTraversed = PsmTraversal(m.prevPsm) && !PsmTraversal(f.psmDetailed);
    m.prevPsm = f.psmDetailed;
    return s;
}

// G2 / G5: the per-frame game-speed calibration of this feed. Tier2 Slow (MotionFeed::tier2Slow, PSM Stand 1 only): the game adds -1.3 m/s to
// MaxSpeed (kTier2SlowDeltaMs), so the stand run's full-stick speed is gameStandPoints[1] - 1.3 (gameCap and analog modifier follow,
// MapConfig::gameRunReductionMs). Walk toggle (MotionFeed::wantsToWalk, MapConfig::gameWalkTogglePoint > 0): on the stand ground without
// aim, air, sprint or Tier2 Slow the first stand speed point becomes the toggle point, so every user of the table - SpeedScaleFor,
// BlendedSpeedScaleFor, CapsuleStopPrediction / CapsuleStartPrediction, GameSteadySpeed / GameAnalogModifier, the gameCap - sees the
// same walk calibration (point -> cmc.walkSpeed). The result is a copy of c with only those fields changed; with neither condition the
// caller keeps c itself (EffectiveMapConfigNeeded false), so the defaults stay bit-identical.
inline constexpr float kTier2SlowDeltaMs = 1.3f;
inline constexpr float kWalkSpeedMaxMs = 3.5f; // upper bound of the walk speed setting (CET mm.walkSpeed, SetWalkSpeed); 0 = off
inline bool WalkMapApplies(const MapConfig& c, const MotionFeed& f)
{
    const float p = c.gameWalkTogglePoint;
    return std::isfinite(p) && p > 0.0f && p < c.gameStandPoints[1] && !(c.speedScale > 0.0f) && f.wantsToWalk && !f.tier2Slow &&
           f.psmDetailed == kPsmStand && !f.aiming && !(f.air.valid && !f.air.onGround);
}
inline bool EffectiveMapConfigNeeded(const MapConfig& c, const MotionFeed& f)
{
    return WalkMapApplies(c, f) || (f.tier2Slow && f.psmDetailed == kPsmStand);
}
inline MapConfig EffectiveMapConfig(const MapConfig& c, const MotionFeed& f)
{
    MapConfig e = c;
    if (WalkMapApplies(c, f))
    {
        e.gameStandPoints[0] = c.gameWalkTogglePoint;
        e.walkMapActive = true;
        // minimum analog walk speed under the walk toggle: the same m/s against the walk speed (AnalogFloorFractionWalk; the script's
        // multiplier, MotionDriver.cpp AnalogFloorMul with walk, uses the same fraction)
#if UTPP_ANALOGFLOOR_MUTATION != 4 // mutation 4: the stand-run fraction and cap under the walk toggle
        if (c.analogFloorFrac > 0.0f)
        {
            e.analogFloorFrac = AnalogFloorFractionWalk(c.analogFloorMs, c.gameWalkTogglePoint);
            e.analogFloorStick = AnalogFloorStickForFraction(e.analogFloorFrac);
#if UTPP_ANALOGFLOOR_MUTATION != 5
            e.analogFloorMulMax = AnalogFloorMulMaxWalk(c.gameWalkTogglePoint);
#endif
        }
#endif
    }
    if (f.tier2Slow && f.psmDetailed == kPsmStand)
        e.gameRunReductionMs = kTier2SlowDeltaMs;
    return e;
}

// JS1 take-off hold (MapConfig::jumpTakeoffHoldS): a PSM Jump 18 feed whose capsule still touches the ground (the runtime AirState
// onGround, so no air feed) and does not rise, out of a ground frame (m.st.movementMode is still last frame's here, so a touchdown that
// arrives on 18 is not affected), stays OnGround until the launch feed, at most jumpTakeoffHoldS. Only 18: the double / charge / hover
// jumps (19-21) are unchanged. No air feed (air.valid false) = unchanged. Not out of a slide or its exit window (m.slideActive /
// m.slideExitAge are still last frame's here): a held feed there would end the slide on the ground for one feed (a Slide_ExitTo* pick
// right before the jump pick, tests/JumpPickTest.cpp slide -> jump); the slide -> jump path keeps its own rules (fix3/jump), unchanged.
// The held feed is mapped with the PREVIOUS feed's PSM detail (m.prevPsm, the detail MapToChooserCore saw last frame), so every ground
// input of the chooser stays the previous frame's: the sprint gait (PsmSprint), visualStrafe (PsmGround), the analog floor lift (Stand /
// Crouch), the stance filter and calibration, the walk-toggle map (EffectiveMapConfig) and ChooserState::stanceRaw / psmJump (false; the
// matcher reads psmJump only in a flight, which starts on the launch feed with its own 18). The first hold forced only the movement mode
// and kept 18 for the rest: a sprint (4) -> 18 take-off mapped to the Run gait, the matcher interrupted on the gait change
// (InterruptOnDatabaseChange) and picked a ground clip (Sprint_to_Run) one feed before the jump pick (review 2026-11-01, replay of
// tests/data/jump_feeds_20261025_1341.txt t 58.854 / 60.038 / 61.171; tests/LocoStartTest.cpp). Consecutive held feeds keep the detail
// (m.prevPsm stays the substituted one). Without a ground detail to keep (an air detail kept as ground by exitAirGraceS) there is no
// hold (the feed goes to the air as before JS1). The same values as MapToChooserCore read at its entry (last frame's state).
inline bool TakeoffHoldFeed(const MotionFeed& f, const MapConfig& c, MapState& m)
{
    m.takeoffHoldActive = false;
#if UTPP_LOCOSTART_MUTATION == 3
    m.takeoffHoldAge = 0.0f; // mutation 3: no take-off hold
    (void)f;
    (void)c;
    return false;
#else
    const float dt = std::isfinite(f.dt) && f.dt > 0.0f ? f.dt : 0.0f;
    const float vz = std::isfinite(f.velWorld[2]) ? f.velWorld[2] : 0.0f;
    const bool slideContext = c.slide && c.slideSpec.declared && (m.slideActive || m.slideExitAge < c.slideExitMapS);
#if UTPP_LOCOSTART_MUTATION == 4
    const bool groundDetail = true; // mutation 4: the first JS1 hold (no detail kept, MapToChooserCore forces only the mode)
#else
    const bool groundDetail = !PsmAir(m.prevPsm);
#endif
    if (c.jumpTakeoffHoldS > 0.0f && f.psmDetailed == kPsmJump && f.air.valid && f.air.onGround && vz < kExitAirRiseMs &&
        m.st.movementMode == 0 && !slideContext && groundDetail && m.takeoffHoldAge + dt <= c.jumpTakeoffHoldS)
    {
        m.takeoffHoldAge += dt;
        m.takeoffHoldActive = true;
        return true;
    }
    m.takeoffHoldAge = 0.0f;
    return false;
#endif
}

inline motion::ChooserState MapToChooser(const MotionFeed& aFeed, const MapConfig& c, MapState& m, const MapContext& x)
{
    MotionFeed held;
#if UTPP_LOCOSTART_MUTATION == 4
    const bool hold = false; // mutation 4: the detail stays 18 (MapToChooserCore reads m.takeoffHoldActive)
    (void)TakeoffHoldFeed(aFeed, c, m);
#else
    const bool hold = TakeoffHoldFeed(aFeed, c, m);
#endif
    if (hold)
    {
        held = aFeed;
        held.psmDetailed = m.prevPsm;
    }
    const MotionFeed& f = hold ? held : aFeed;
    if (!EffectiveMapConfigNeeded(c, f))
        return MapToChooserCore(f, c, m, x);
    const MapConfig e = EffectiveMapConfig(c, f);
    return MapToChooserCore(f, e, m, x);
}

// ---------------------------------------------------------------- MM query trajectory (finding 8)
// UE 5.8 builds every trajectory channel of the query from the FTransformTrajectory of the pose history:
// FArchivedPoseHistory::GetTransformAtTime (PoseSearchHistory.cpp l. 594-605) = Trajectory.GetSampleAtTime(Time, bExtrapolate)
// .GetTransform() (no extrapolation, TrajectoryTypes.cpp l. 40-64), velocities by the finite difference of PoseSearchContext.cpp
// l. 1013-1016 (FiniteDelta 1/60). ChooserMatcher samples the caller trajectory linearly (SampleTraj) and takes the same
// differences (TrajectoryFeature), so the query trajectory holds motiontraj::Trajectory::SampleAt at EXACTLY every time a
// trajectory channel of any schema reads (t_sample, t_origin, t_sample - 1/60, t_origin - 1/60 for character-space
// velocities): ChooserMatcher then never interpolates between them and every feature equals the UE value on the
// MotionTrajectory trajectory.
// Space (ChooserMatcher TrajSample / FORMAT.txt "Feature values"): metres, +Z up, yaw in radians with the forward +Y rotated
// by yaw = the game yaw psi. MotionTrajectory is in motion cm (horizontal x SpeedScaleFor = the database speed space, report 3.6;
// vertical x 100) with facing in the atan2 convention (GameYawToTrajYaw = psi + 90), so x / 100 and yaw = facing - 90.
// Z: the trajectory's own vertical = capsule history + gravity / floor pass (report 3.3 / 6.10.4; Z_FROM_RTZ channels
// compare it with the database RTZ). Positions are made relative to the current sample (t = DeltaTime, Prediction(0)) to
// keep float precision on long sessions; every feature is relative, so this does not change any value.
inline constexpr uint32_t kQueryTrajMax = 64;
struct QueryTimes
{
    float t[kQueryTrajMax] = {};
    uint32_t n = 0;
    bool overflow = false;
};

// Sorted unique trajectory read times of every schema in aDb (SCHC flag TRAJECTORY). Same float arithmetic as
// ChooserMatcher::TrajectoryFeature (ts - 1.0f / 60.0f) so the times match bit for bit.
inline QueryTimes BuildQueryTimes(const motion::MotionDatabase& aDb)
{
    QueryTimes q;
    auto add = [&q](float aT) {
        for (uint32_t i = 0; i < q.n; ++i)
            if (q.t[i] == aT)
                return;
        if (q.n >= kQueryTrajMax)
        {
            q.overflow = true;
            return;
        }
        uint32_t i = q.n++;
        while (i > 0 && q.t[i - 1] > aT)
        {
            q.t[i] = q.t[i - 1];
            --i;
        }
        q.t[i] = aT;
    };
    constexpr float h = 1.0f / 60.0f;
    for (uint32_t s = 0; s < aDb.Header().numSchemas; ++s)
    {
        const motion::SchemaRecord& sc = aDb.Schema(s);
        for (uint32_t c = sc.firstChannel; c < sc.firstChannel + sc.numChannels; ++c)
        {
            const motion::ChannelRecord& ch = aDb.Channel(c);
            if (!(ch.flags & motion::kChanTrajectory))
            {
                // pose channels read the trajectory root only at future times (ChooserMatcher::HistRoot, t > 0)
                for (const float t : {ch.tSample, ch.tOrigin, ch.tSample - h, ch.tOrigin - h})
                    if (t > 0.0f)
                        add(t);
                continue;
            }
            add(ch.tSample);
            add(ch.tOrigin);
            if (ch.kind == 2 || ch.kind == 3)
            {
                add(ch.tSample - h);
                add(ch.velocitySpace == 1 ? ch.tOrigin - h : ch.tOrigin);
            }
        }
    }
    return q;
}

// One query trajectory sample from the motion trajectory at time aT (see above).
inline motion::TrajSample QuerySample(const motiontraj::Trajectory& aTraj, float aT)
{
    const motiontraj::V3 o = aTraj.Prediction(0).pos;
    const motiontraj::Sample s = aTraj.SampleAt(aT);
    motion::TrajSample r;
    r.t = aT;
    r.x = (s.pos.x - o.x) * 0.01f;
    r.y = (s.pos.y - o.y) * 0.01f;
    r.z = (s.pos.z - o.z) * 0.01f;
    r.yaw = motiontraj::NormalizeAxis(s.facing - 90.0f) * integ::kDegToRad;
    return r;
}

// Fills aOut[0 .. aTimes.n) (aMax >= aTimes.n) with the query trajectory of MapState::traj after MapToChooser. Returns the
// sample count; 0 = not built (trajectory not initialised, overflow or aMax too small). Allocation-free.
inline uint32_t BuildQueryTrajectory(const MapState& m, const QueryTimes& aTimes, motion::TrajSample* aOut, uint32_t aMax)
{
    if (m.traj.NumHistory() == 0 || aTimes.overflow || aTimes.n == 0 || aTimes.n > aMax || !aOut)
        return 0;
    for (uint32_t i = 0; i < aTimes.n; ++i)
        aOut[i] = QuerySample(m.traj, aTimes.t[i]);
    return aTimes.n;
}

// ---------------------------------------------------------------- cost budget [P]
struct Budget
{
    static constexpr float kFeedP95Us = 250.0f;     // whole v2 Feed on the game thread (1.5 % of 16.6 ms)
    static constexpr float kMatcherP95Us = 150.0f;  // ChooserMatcher::Update incl. chooser
    static constexpr float kInertialP95Us = 15.0f;
    static constexpr float kLayersP95Us = 20.0f;    // overlay + additive (report 6.6 #9)
    static constexpr float kTripFactor = 2.0f;      // trip when p95 > factor x kFeedP95Us over kWindow feeds
    static constexpr uint32_t kWindow = 600;
    // A trip pauses the motion pose for a span of feed time, then the statistics restart empty and the pose comes back. The first
    // pause lasts kPauseFirstS; each trip after a resumed window doubles it up to kPauseMaxS; a full kWindow of feeds without a trip
    // sets it back to kPauseFirstS.
    static constexpr float kPauseFirstS = 5.0f;
    static constexpr float kPauseMaxS = 60.0f;
};

// ---------------------------------------------------------------- pose sampling (game-local, frame lerp + nlerp)
inline bool SamplePose(const motion::MotionDatabase& aDb, int32_t aClip, float aClipTime, BoneTransform* aOut, uint32_t aNumBones)
{
    if (aClip < 0 || static_cast<uint32_t>(aClip) >= aDb.NumClips() || aNumBones > aDb.NumBones() || !aOut)
        return false;
    const motion::MotionClipRecord& c = aDb.Clip(static_cast<uint32_t>(aClip));
    if (c.numFrames == 0)
        return false;
    const float fps = aDb.Header().fps > 0.0f ? aDb.Header().fps : 30.0f;
    float ft = (std::isfinite(aClipTime) ? aClipTime : 0.0f) * fps;
    if (ft < 0.0f)
        ft = 0.0f;
    const float last = static_cast<float>(c.numFrames - 1);
    if (ft > last)
        ft = last;
    const uint32_t f0 = static_cast<uint32_t>(ft);
    const uint32_t f1 = f0 + 1 < c.numFrames ? f0 + 1 : f0;
    const float u = ft - static_cast<float>(f0);
    const motion::PoseRecord* a = aDb.Pose(c.firstFrame + f0);
    const motion::PoseRecord* b = aDb.Pose(c.firstFrame + f1);
    for (uint32_t i = 0; i < aNumBones; ++i)
    {
        aOut[i].p = math::Lerp(math::Vec3{a[i].px, a[i].py, a[i].pz}, math::Vec3{b[i].px, b[i].py, b[i].pz}, u);
        math::Quat qa{a[i].qx, a[i].qy, a[i].qz, a[i].qw};
        math::Quat qb{b[i].qx, b[i].qy, b[i].qz, b[i].qw};
        if (math::QuatDot(qa, qb) < 0.0f)
            qb = math::QuatNeg(qb);
        aOut[i].q = math::QuatNormalize(math::Quat{qa.x + (qb.x - qa.x) * u, qa.y + (qb.y - qa.y) * u,
                                                   qa.z + (qb.z - qa.z) * u, qa.w + (qb.w - qa.w) * u});
    }
    return true;
}

// ---------------------------------------------------------------- adapter
// Owns the v2 matcher + inertializer for one database. The caller (MMDriver, game thread, under g_mutex) builds the
// trajectory samples from its existing Trajectory (input-driven past/future) and passes Pose() on to OffsetRoot /
// StanceGuard / FootLocker and PublishLocked exactly like the v1 matcher output.
class MotionAdapter
{
public:
    bool Init(const motion::MotionDatabase* aDb, const chooser::Table* aTable, const motion::MatcherConfig& aCfg, uint32_t aFirstBone,
              std::string& aError)
    {
        m_ready = false;
        if (!aDb || !aDb->Loaded() || !m_matcher.Init(aDb, aTable, aCfg, aError))
            return false;
        m_db = aDb;
        const uint32_t n = aDb->NumBones();
        m_raw.assign(n, BoneTransform{});
        m_prev.assign(n, BoneTransform{});
        m_out.assign(n, BoneTransform{});
        if (!m_inertial.Init(n, aFirstBone))
        {
            aError = "inertializer init";
            return false;
        }
        m_inertial.SetMinDuration(kBlendFloorSeconds); // motionfix item 7
        m_havePrev = false;
        m_ready = true;
        return true;
    }

    bool Ready() const { return m_ready; }
    const QuinticInertializer& Inertial() const { return m_inertial; } // motionfix item 7: tests (effective blend durations)

    // One feed. Returns false (caller falls back / keeps the last pose) when the matcher produced no pose.
    bool Update(float aDt, const motion::TrajSample* aTraj, uint32_t aNumTraj, const motion::ChooserState& aState,
                const motion::PoseHistory* aHist = nullptr)
    {
        if (!m_ready)
            return false;
        const motion::MatchResult& r = m_matcher.Update(aDt, aTraj, aNumTraj, aState, aHist);
        if (r.clip < 0 || !SamplePose(*m_db, r.clip, r.clipTime, m_raw.data(), static_cast<uint32_t>(m_raw.size())))
            return false;
        if (r.blend.valid && r.blend.blendTime > 0.0f)
            m_inertial.Request(r.blend.blendTime);
        // previous target only when it is on the same timeline (no pose jump this update)
        if (!m_inertial.Update(m_raw.data(), (m_havePrev && !r.jumped) ? m_prev.data() : nullptr, aDt, m_out.data()))
            m_out = m_raw;
        m_prev = m_raw;
        m_havePrev = true;
        const motion::RtzRecord& z = m_db->Rtz(r.frame);
        m_rootZ = z.rootZ; // feature / diagnostics only, never written to the visual pose (6.6.4)
        m_rootVz = z.vz;
        return true;
    }

    std::span<const BoneTransform> Pose() const { return m_out; }
    const motion::MatchResult& Last() const { return m_matcher.Last(); }
    float RootZ() const { return m_rootZ; }
    void Reset()
    {
        m_matcher.Reset();
        m_inertial.Reset();
        m_havePrev = false;
    }

private:
    const motion::MotionDatabase* m_db = nullptr;
    motion::ChooserMatcher m_matcher;
    QuinticInertializer m_inertial;
    std::vector<BoneTransform> m_raw, m_prev, m_out;
    bool m_havePrev = false;
    bool m_ready = false;
    float m_rootZ = 0.0f;
    float m_rootVz = 0.0f;
};
} // namespace utpp::mm::adapter

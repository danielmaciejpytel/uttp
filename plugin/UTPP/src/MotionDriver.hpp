#pragma once
// v2 integration, native side. Owns, per body (only the active
// body is loaded, lazily, on a worker thread; the old body is released on the worker): the v2 database + chooser table +
// overlays + style weights + traversal data (layout below), and the adapter state (adapter::MapState, motion::ChooserMatcher,
// motionpose::PoseEvaluator). Missing / invalid data or a rig / topology mismatch falls back to v1 with a reason
// (adapter::Resolve, adapter::FallbackName); never a crash.
//
// ---------------------------------------------------------------- FIXED NATIVE INTERFACE (registered in Main.cpp)
//   UTPPSetMotionEnabled(on: Bool)        mm.motionDriver switch, default OFF (off = v1 bit-identical: Step / ApplyToSnapshot never change
//                                       the v1 pose or snapshot once a running handover has finished; they still run: each
//                                       takes the v2 lock, and ApplyToSnapshot copies the published v1 snapshot (~2.6 KB)
//                                       per publish as the start pose of a later v1 -> v2 hand-in)
//   UTPPSetMMRigBody(body: Int32)       0 = woman / female V, 1 = man / male V (other values ignored, returns false)
//   UTPPSetMMOverlayInputs(pose: Int32, aim: Int32, release: Bool, injured: Bool, resolvedAimPose: Int32)
//       pose    = ov::PoseOverlay value of mm/Overlay.hpp (0 Default, 1 Rifle, 2 Pistol1H, 3 Pistol2H, Injured; the motion
//                 Enum_OverlayPose value 9 is accepted as Injured); invalid -> Default. PoseOverlay::Game = release.
//       aim     = ov::AimState value (Aiming -> OverlayInputs.wantsToAim, also set by the PSM UpperBody Aim of the feed).
//       release = the weapon class stays on game animations (melee, body carry): v2 yields, v1 runs unchanged (v1's own
//                 release / upper mask hands the body back to the game).
//       injured = overlay Injured when pose is Default (weapon overlays keep precedence) [P].
//   UTPPGetMotionStatus() -> String       one line: enabled, body, db loaded, fallback reason, last pick, feed p95 us.
//
// ---------------------------------------------------------------- data folder layout (next to the DLL, local only)
//   <dll dir>\data\utpp_motion_<body>.utppmm         database            (adapter::PathsFor)
//   <dll dir>\data\utpp_motion_<body>.chooser.json   chooser table       (adapter::PathsFor)
//   <dll dir>\data\utpp_overlays_<body>.utppov           overlays            (adapter::PathsFor)
//   <dll dir>\data\utpp_style_weights_<body>.txt         style weights       (adapter::PathsFor; missing = evaluator defaults)
//   <dll dir>\data\utpp_traversal_<body>.utpptrav        traversal montages  (Paths.traversal; UTPP-MOTION SEAM trav: played;
//                                                      version 2 = + poses, staged from Data\traversal\utpp_traversal_<body>_motion_knee.utpptrav)
//   <dll dir>\data\utpp_traversal_table.json            traversal table     (Paths.traversalTable = Data\traversal\traversal_table.json)
//   <body> = woman | man. motion-derived data. Narrow (ANSI) paths.
//
// ---------------------------------------------------------------- threading
// Step / ApplyToSnapshot / OnArmed / OnDisarmed run inside MMDriver on the game thread under the v1 g_mutex and take the v2
// mutex after it (lock order v1 g_mutex -> v2 mutex; natives and the worker take only the v2 mutex). The worker does all file
// I/O and parsing and installs a finished bundle under the v2 mutex. The v2 pose reaches the FK hook through the v1
// PublishLocked (same g_snap / g_mmPose seqlock, publishTicks / kDetourStaleSeconds staleness, Tick republish under hold):
// ApplyToSnapshot overwrites pose + bone map of g_snap right before the publish; weight and fade stay v1's.
//
// ---------------------------------------------------------------- seam (lines marked "UTPP-MOTION SEAM")
//   MotionHost.cpp Feed: Step(aRaw, g_effGate, dt) after the 0.6.2 hold block, before the weight update.
//     Commit A (int): MotionStepLocked right before g_matcher.Update; Step true + Frame true = v2 owns the feed: the v1
//     search is skipped (MotionKeepBaseTrajectoryLocked keeps the v1 trajectory current), the v2 pose (+ ROOT motion, CONT contacts,
//     *_Stops / *_Idles = replant Stops [P]) runs through OffsetRoot + ApplyLegPipelineLocked (v1 stage instances: shared
//     topology kExpectedRigHash, Frame checks the bone map per bone), ApplyToSnapshot then keeps the tail output. When the
//     matcher block does not run, Step is called after it as before. Handovers: v2 -> v1 (yield, toggle off, body switch) and
//     v1 -> v2 (entry) blend from the held snapshot over kDefaultHardOutTime (BlendFromHeld); a fault cuts to v1.
//   MotionHost.cpp PublishLocked: ApplyToSnapshot(g_snap) before g_mmPose.Publish (v1 pose kept when it returns false).
//     Batch 1: after the copy of PipelineOutputLocked() (the v1-only selector, unchanged) into g_snap; v2 overrides it there.
//   MotionHost.cpp OnArmed / OnDisarmed: rig hashes + parents (bone map, adapter::TopologyMismatches).
//   Main.cpp: SetDataDir(<dll dir>) at load, natives in RegisterTypes / PostRegisterTypes, Shutdown at unload.
// Frame (Step): yield unless enabled && Resolve == None && g_effGate == 0 (gate open or HoldEligible; v2 never opens the
// gate) && !PsmDetailReleases(raw psm detail) && !release. v2 OWNS AIR (motionfix item 5): while mm.motionDriver is on, v2 steps
// every air feed itself (Jumps / Lands databases, InAir from AirFeed / PSM); the runtime never claimed air frames (main 22d4062
// has no SetAirOwns call), so the old AirOwns yield (SetAirOwns / SetMotionOwnsAir, skip 'air-owns') was removed. Strafe inside
// MapToChooser. AirFeed.valid = false on a feed without UTPPFeedAir. air: SetAirFeed feeds AirFeed (valid = true for
// the next Step).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "MotionHost.hpp"
#include "MutationSelect.hpp"
#include "mm/MotionAdapter.hpp"
#include "mm/GameIntegration.hpp"

namespace utpp::motiondriver
{
namespace adapter = utpp::mm::adapter;
namespace integ = utpp::mm::integ;

inline constexpr int32_t kBodyWoman = 0;
inline constexpr int32_t kBodyMan = 1;
// Root and Trajectory are never written: same first bone as v1 (MatcherConfig::firstBodyBone 2, PoseSnapshot::firstBone 2)
// and as the offline v2 replay (FeedReplayMotion.cpp drv.Init(..., 2, ...)).
inline constexpr uint32_t kFirstBone = 2;

struct Paths
{
    adapter::DataPaths files;
    std::string traversal;
    std::string traversalTable; // UTPP-MOTION SEAM trav: CHT_TraversalAnims decision table (both bodies)
    std::string lean;           // optional motion lean pack utpp_lean_<body>.bin (mm.lean); missing / invalid = lean unavailable
    std::string getup;          // optional native get-up clips utpp_getup_<body>.utpptrav; missing / invalid = get-up unavailable (vanilla path)
};
Paths PathsForBody(adapter::Body aBody, const std::string& aDir);

// ---- natives (game thread)
void SetEnabled(bool aOn);
bool Enabled();
bool SetRigBody(int32_t aBody);                                             // false = invalid value (ignored)
bool SetOverlayInputs(int32_t aPose, int32_t aAim, bool aRelease, bool aInjured, int32_t aResolvedAimPose = -1); // false = pose / aim out of range
bool SetWeaponAction(int32_t aAction); // 0 none, 2 reload, 8 shoot (PSM.Weapon); independent of Ready/Aim
// Equipped-weapon token + authored recoil profile id (mm/RecoilProfiles.hpp; 0 = none, unknown ids = none).
void SetRecoilProfile(uint32_t aEpoch, int32_t aProfile);
void SetRecoilWeapon(uint32_t aEpoch, bool aSupportedNue); // = SetRecoilProfile(aEpoch, aSupportedNue ? Nue : none)
void SetAuthoredReload(bool aOn);
void FeedRecoilShot(uint32_t aEpoch, bool aAiming);
// Magazine top-up gesture (mm/MagTap.hpp; Main.cpp UTPPFeedMotionMagTap, scripts/utpp_motion.reds OnMagazineAmmo): the magazine of the
// equipped weapon was refilled without a reload state (game Shinobi slide / air dash bonus). Queues one short left-hand gesture to
// the magazine for the next v2 Step; that Step plays it only while v2 owns the arms in TPP (weapon overlay drawn, no reload /
// game-owned upper body / traversal / handover, gate open) and the left hand is within magtap::kMaxTravel of the magazine (not a
// one-handed hold), otherwise the request is dropped (never played later). false = refused
// at once (setting off - SetMagTapEnabled, the default -, v2 off, release, no weapon overlay, reload action). Status: magTapW=
// magTap=<fed>/<started>/<dropped>/<frames>
// magTapMoved= magTapMovedMax= magTapErr= magTapTravel=<hand -> magazine m at the last owned request> magTapUnowned=<dropped: not
// owned / stale> magTapOn=<setting> magTapOff=<refused: setting off>; MOTION trace (only while a gesture ran since the last line):
// magTap=<weight>/<peak>/<started>.
bool FeedMagTap();
// CET mm.shinobiBonusReload "Reload animation during Shinobi bonus" (native UTPPSetMotionMagTap), default OFF: the gesture above plays
// only while this is on. Process-lifetime flag (not reset by SetEnabled / Load / Shutdown). Off drops a pending request; a running
// gesture fades out over its remaining time.
void SetMagTapEnabled(bool aOn);
// UTPP-MOTION SEAM motionfix item 2: the aim target of UTPPSetAimTarget (model space of the player entity, the same point
// aimofs::SetTarget receives; Main.cpp Native_SetAimTarget). aEnabled false = aim ended; aValid false = transform unreadable.
// Gives the v2 aim offset its yaw (camera - entity) and pitch (AimAnglesFromModel).
void SetAimTarget(bool aEnabled, bool aValid, const float aTargetModel[3]);
// Active TPP camera ray for owned hipfire/ADS overlays; independent of the
// native FPP aim-mask target.
void SetCameraTarget(bool aEnabled, bool aValid, const float aTargetModel[3]);
// mm.aimGameUpper: while v2 owns the frame with a weapon overlay and the player aims, hand the upper body back to the game
// graph + aimofs (the snapshot keeps v1's upperScale / upperBits instead of upperScale 1; v2's own aim offset is skipped).
// Default kAimGameUpperDefault (chosen by the item 2 measurement, see MotionDriver.cpp).
void SetAimGameUpper(bool aOn);
void SetGripEnabled(bool aOn);
void SetShootTransferEnabled(bool aOn); // opt-in Pistol2H Shoot experiment; default off
// Opt-in head lead (mm/HeadLead.hpp; CET mm.headLead; default off): Neck / Neck1 / Head turn into the turn of the run, applied to
// the published v2 pose (v1-owned frames unchanged). SetHeadLeadParams(leadDegAtRef 0..45, maxLeadDeg 0..45): false on NaN / out
// of range (nothing changed). Status / MMTRACE: headLead= lead= corr=.
bool SetHeadLead(bool aOn);
bool SetHeadLeadParams(float aLeadDeg, float aMaxDeg);
// Slide feed (CET mm.slide, default OFF; mm/MotionAdapter.hpp "slide feed"): PSM LocomotionDetailed 5 (slide, sprint -> crouch) is
// fed to the chooser as MovementMode Sliding / Stance Crouch / MovementDirection / MovementMode_LastFrame and v2 owns the body for
// it (no hold-last-pose behaviour of the v1 gate: no foot contacts, no body-yaw command). Active ONLY when the setting is on AND
// the loaded bundle's chooser.json declares the "slide" section (databases PSD_Slide_FeetOut / PSD_Slide_KneesOut baked in the
// database file); otherwise nothing changes. SetSlide always returns true. Status: slide= slideBundle= slideFrames=; MMTRACE
// (only with the setting on and a slide bundle): slide=<active>/<direction>/<modeLast>.
bool SetSlide(bool aOn);
// Shooter mode (CET bodyturn.shooterMode, default OFF; mm/MotionAdapter.hpp MapConfig::shooterMode): motion's default Strafe rotation mode - the
// capsule follows the camera, the chooser queries camera-relative directional clips, no spin transitions, the visual root is locked to
// the actor while V moves, and the root HOLDS its heading while V stands without input (the camera may orbit her). SetShooterMode stores the
// setting under g_m and refreshes the live flag; it never resets a matcher, root, map or body-turn state (unlike UTPPSetMMBodyTurn).
// Always true, idempotent. Takes effect on the next v2 Step. Process-lifetime flag. Status: shooterMode= shooterModeLive= shooterFrames=.
bool SetShooterMode(bool aOn);
// Lock-free (reds calls it once per locomotion update): the speed ratio by direction (MotionAdapter.hpp ShooterModeSpeedRatio) while Shooter is
// LIVE (setting on, v2 enabled, no fallback, bundle loaded); exactly 1.0f otherwise. ShooterModeLive mirrors that flag.
float ShooterModeSpeedMul(float aMoveX, float aMoveY, int32_t aDetailed);
bool ShooterModeLive();
// Walk toggle speed (CET mm.walkSpeed, DLL default 0 = off; mm/MotionAdapter.hpp MapConfig::gameWalkTogglePoint): the capsule speed (game
// m/s, 0 .. kWalkSpeedMaxMs) the game walk toggle (G) should hold at the full stick. SetWalkSpeed stores it (false on NaN / out of range,
// nothing changed); the driver maps that speed onto the motion Walk cap while the toggle is on (stand ground, no aim / air / sprint), and
// the script (utpp.reds UTPPUpdateWalkSpeed) holds the capsule there with one MaxSpeed multiplier = WalkSpeedTarget() / 2.72 (the speed
// the toggle gives unmodified, measured game 2.5). WalkSpeedSetting = the stored value; WalkSpeedTarget is lock-free (reds calls it once
// per locomotion update) and answers the setting only while v2 really drives third-person locomotion (v2 live, fresh TPP feed - the same
// gate as ShooterModeSpeedMul), else exactly 0 (= no modifier). Status: walkSpeed=.
bool SetWalkSpeed(float aMs);
float WalkSpeedSetting();
float WalkSpeedTarget();
// Minimum analog walk speed (CET mm.minWalkSpeed, DLL default 0 = off; mm/MotionAdapter.hpp AnalogWalkFloorMul / MapConfig::analogFloorStick):
// the capsule speed (game m/s at the stand run curve, 0 .. kMinAnalogWalkMaxMs) a small stick tilt is lifted to. SetMinAnalogWalk stores it
// (false on NaN / out of range, nothing changed); the driver gives the prediction the cap of the lifted speed, and the script (utpp.reds
// UTPPUpdateAnalogFloor) applies one MaxSpeed multiplier = AnalogFloorMul. AnalogFloorMul is lock-free (reds calls it once per locomotion
// update) and answers AnalogWalkFloorMul only while v2 really drives third-person locomotion (the WalkSpeedTarget gate) for Stand (1) / Crouch
// (3), else exactly 1.0f. aWalk = the game walk toggle is on in the stand state (reds): with a walk speed set (SetWalkSpeed > 0) and no Tier2
// Slow the floor's fraction is taken against that walk speed (adapter::AnalogFloorFractionWalk, as the driver's EffectiveMapConfig does).
// Status: minAnalogWalk=.
bool SetMinAnalogWalk(float aMs);
float MinAnalogWalkSetting();
float AnalogFloorMul(float aMoveX, float aMoveY, int32_t aDetailed, bool aWalk);
// Selection churn guards as a group (CET mm.selectionGuards, developer option, DLL default ON): off = every clip selection guard of the
// matcher (ChooserMatcher::SetSelectionGuards) and the gait return hold of the speed map (MapConfig::gaitReturnHoldS 0) are off, for A/B tests.
// Read on every Step, for both bodies and every bundle loaded later. Always true. Status: selGuards=.
bool SetSelectionGuards(bool aOn);
bool SelectionGuards();
// Tier2 Slow flag (GameplayRestriction.Tier2LocomotionSlow; reds UTPPFeedMotionTier2Slow, sent right before every UTPPFeedLocomotion): the next
// v2 Steps lower the stand run cap by 1.3 m/s (MotionFeed::tier2Slow). Process-lifetime atomic, false at start. Status: tier2Slow=.
void FeedTier2Slow(bool aSlow);
bool Tier2SlowFed();
// G9: the aim jog gives no target (Main.cpp Native_MMAimJogTarget answered 0), so AimWalk keeps the vanilla aim walk speed
// (~1.65 m/s) and the v2 driver asks for the Walk gait there (MotionFeed::aimWalkVanilla).
void SetAimJogOff(bool aOff);
// H2 safeguard (default ON; console kill switch, CET never calls it): with the native body turn off for a reason other than aim / hip fire
// (weapon turn off with a weapon in hand, kill switch, guard pause) and V running backward, v2 uses the Strafe rotation mode so motion plays
// the backward loops instead of looping the 180 deg spin. Never acts with Shooter on or while the body turn drives.
bool SetH2Guard(bool aOn);
// CET hip-fire lease (UTPPSetMotionHipLease; CET sends it on every change of the lease state, Shooter or not): while it runs the native body turn
// is off for the lease, whatever the weapon reports to the DLL, so the H2 safeguard treats it as an exclusion (same clear hold as aim / hip fire).
void SetHipLease(bool aOn);
// Host hook (Main.cpp): the Locomotion blocker bits (src/input/LocomotionCore.hpp Reason) of this moment, read only. Used to tell WHY the native
// body turn is off (H2 safeguard). nullptr (offline tests) = every off state counts as armed.
void SetLocomotionReasonsProbe(uint32_t (*aProbe)());
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
// test targets only: the strafe state of the last v2 Step (Shooter / H2 acceptance tests).
struct StrafeProbe
{
    int32_t rotationMode = 0; // motiontraj::RotationMode of the last chooser mapping
    bool shooterModeActive = false, h2Active = false;
    int32_t gait = 0;
    float actorYawDeg = 0.0f, rootYawDeg = 0.0f; // feed player yaw / committed visual root yaw (world, deg) of that Step
    bool shouldSpin = false;                       // chooser input ShouldSpinTransition of that Step
};
StrafeProbe TestStrafeProbe();
// test targets only: the aim point / barrel loop seam of the last v2 evaluation (aim pitch plan 2026-10-30 seam C).
struct BarrelProbe
{
    bool evaluated = false;   // a v2 evaluation ran (MM pick or traversal montage frame)
    bool rootPath = false;    // P came from PrepareRootOverlay (InversePoint with the root frame), else the actor point fallback
    float frame[3] = {0, 0, 0};    // root frame of PrepareRootOverlay: x, y (m, actor space), yaw (rad)
    float actor[3] = {0, 0, 0};    // actor-space aim point given to the evaluator
    bool haveVisual = false;       // the evaluator saw an aim point (FrameInfo)
    float visual[3] = {0, 0, 0};   // the aim point the evaluator used (pose frame)
    float aimYawDeg = 0, aimPitchDeg = 0; // the sweep target angles the evaluator used
    bool loop = false, active = false, measured = false;
    float errBeforeDeg = 0, errAfterDeg = 0, biasDeg = 0, residualDeg = 0, targetDist = 0;
    uint32_t excluded = 0;
};
BarrelProbe TestBarrelProbe();
// test targets only: the root frame step of the last v2 Step (inputs of the native offset root and its offset before / after).
struct RootProbe
{
    bool prepared = false;
    float dt = 0, capsuleDelta[2] = {0, 0}, capsuleYawDelta = 0, animDelta[2] = {0, 0}, animYawDelta = 0;
    float yawAhead = 0; // the coming tick's capsule turn the frame is relative to (rad, Proposal::Ahead)
    bool movingGround = false, montage = false, aimFacing = false;
    float before[3] = {0, 0, 0}, after[3] = {0, 0, 0}; // x, y (m), yaw (rad): visual minus capsule, capsule axes
};
RootProbe TestRootProbe();
#endif
// Orientation warp (CET mm.orientWarp, default OFF; mm/OrientWarp.hpp): while V runs a circle the pelvis (extra visual root yaw on
// the provided root frame of the leg tail) and the torso (twist over Spine .. Spine3) are brought onto the path of motion instead of
// the outside of the turn; loop clips on the ground only, no aim / weapon / air / crouch / slide / traversal, weight 0 (bit-identical)
// for straight running, low speed and non-loop clips. SetOrientWarp always returns true. Status: orientWarp= owW= owRoot= owTorso=;
// MMTRACE (only while on or fading): ow=<on>/<weight>/<root deg>/<torso deg>.
bool SetOrientWarp(bool aOn);
// Run pelvis yaw trim (CET mm.pelvisYawTrim; mm/OrientWarp.hpp PelvisTrim): aDeg (+ = left, -10..10, default 0) turns the pelvis of
// the v2 pose while V walks / runs / sprints forward on the ground (chest, head and feet keep their direction; the motion forward
// loops carry the pelvis ~5 deg to the right, +5 squares it). Not while aiming / weapon drawn / air / crouch / slide / traversal;
// scaled down by the orientation warp weight. False on NaN / out of range (nothing changed). MMTRACE: pt=<deg>/<weight>/<reason> while
// set; reason = the first gate holding the weight down (aim, drawn, wpnact, air, crouch, trav, release, noroot, noskel) or the motion
// (idle, slow, dir, ok). Weight / reason of the last Step: PelvisYawTrimWeight / PelvisYawTrimReason (diagnostics, tests).
// Feminine shoulder narrowing (CET mm.shoulderWidth; mm/FeminineShoulders.hpp, MotionPose.hpp): aCm (0..5, default 2.5) = how far each
// upper-arm socket of a female body moves toward the spine at full weight; hands and weapon stay exactly where the animation put them.
// 0 = off. False on NaN / out of range (nothing changed). Applied to the installed bundle at once and to every bundle loaded later.
bool SetFeminineShoulderCm(float aCm);
bool SetPelvisYawTrim(float aDeg);
float PelvisYawTrim();
float PelvisYawTrimWeight();
std::string PelvisYawTrimReason();
// Body yaw offset (CET mm.bodyYawOffset; mm/OrientWarp.hpp BodyYawOffset): aDeg (+ = left, -20..20, default 0) turns the WHOLE v2
// character (pelvis, torso, head, legs and feet as one rigid pose) about the visual root: added to the visual root yaw of the leg-tail
// frame only; entity / capsule yaw, movement direction, matcher state and pose are untouched. On foot in every stance / gait, weapon
// holstered or drawn; not while aiming, during a weapon action (shot, reload, hip-fire pose, melee), traversal / release / no root
// frame (weight fades 0.25 s). False on NaN / out of range (nothing changed). MMTRACE: bo=<deg>/<weight>/<reason> while set; reason =
// the first gate holding the weight down (aim, melee, wpnact, trav, release, noroot) or ok. Weight / reason of the last Step:
// BodyYawOffsetWeight / BodyYawOffsetReason (diagnostics, tests).
bool SetBodyYawOffset(float aDeg);
float BodyYawOffset();
float BodyYawOffsetWeight();
std::string BodyYawOffsetReason();
// Barrel pitch loop of the v2 aim offset (aim pitch plan 2026-10-30; CET mm.aimAtCrosshair, DLL default OFF = the open-loop aim offset
// bit for bit; mm/BarrelPitchLoop.hpp): closes the loop on the barrel elevation of WeaponRight against the aim point (UTPPSetAimTarget /
// UTPPSetCameraTarget, P in the pose frame). Off: a running loop fades out (bias / residual at most 1 deg per frame). Always true.
// SetBarrelAxis: WeaponRight local barrel axis (aim::AxisVector: 1 X, 2 Y default, 3 Z, negative opposite; false = invalid, kept),
// fed by UTPPSetAimOffsetAxis together with aimofs. SetBarrelMuzzle: m from WeaponRight to the muzzle along the axis, 0.05 .. 1.0
// (default aim::Params::muzzleOffset 0.25; false = NaN / out of range, kept). MMTRACE: bp=... while the loop runs or is measured.
bool SetBarrelLoop(bool aOn);
bool SetBarrelAxis(int32_t aAxis);
bool SetBarrelMuzzle(float aMeters);
// motion lateral-acceleration lean (CET mm.lean, default OFF; mm/LeanRuntime.hpp on the pack v2\utpp_lean_<body>.bin, loaded
// optionally on the worker like the traversal file; a missing / invalid pack leaves the bundle fine and the lean unavailable): while V
// runs / walks on the ground the body leans into the turn (mesh space rotation offset additive on the v2 pose, after the pose evaluator
// and before the orientation warp and the leg tail). Same gates as the orientation warp (no aim / weapon / air / crouch / slide /
// traversal), weight 0 (bit-identical) otherwise. SetLeanHead (mm.leanHead, default OFF): the motion head layer replaces the head
// lead while on (the two are never summed). SetLeanParams(scale, headScale): amount multipliers 0 .. 2 (default 1 = motion strength);
// false = NaN / out of range (nothing changed). SetLean / SetLeanHead always return true. Status: lean=<on>/<bundle>/<alpha>/<lat>/<x>/
// <curve> (bundle: -1 no bundle, 0 pack unavailable, 1 ready) leanHead= leanScale=; MMTRACE (only while on or fading):
// lean=<on>/<alpha>/<lat>/<x>/<curve>.
bool SetLean(bool aOn);
bool SetLeanHead(bool aOn);
bool SetLeanParams(float aScale, float aHeadScale);
bool SlideSetting();
bool SlideBundleDeclared(); // the installed bundle declares the slide section (false while loading / no bundle)
// UTPP-MOTION SEAM slide: true while the slide feed is live (setting on, v2 able to run, bundle declares slide) and aPsmDetail is the
// slide detail 5. Lock-free (one atomic load): MMDriver Feed calls it under the v1 mutex before this feed's Step, so it lags the
// setting / bundle by at most one feed. MMDriver then issues no body-yaw command for that feed.
bool OwnsSlide(int32_t aPsmDetail) noexcept;
// UTPP-MOTION SEAM slide camera: the same lock-free flag without the detail test - true while the slide feed is live (mm.slide on,
// v2 able to run, the active bundle declares slide). Exposed to the scripts as UTPPSlideOwned (utpp_locomotion.reds GetCameraYaw wrap:
// the vanilla slide exit by camera yaw >= 95 deg is skipped for Locomotion sprints toward the camera). False when v2 is off, the
// setting is off, the bundle has no slide section, after OnDisarmed and after a bundle swap that dropped the section.
bool SlideHandlingActive() noexcept;
void SetAimUpperFade(bool aOn); // keep v1's mask during aim exit instead of one-frame upperScale=1
void SetWeaponUpper(bool aOn); // low-ready from the game, based on explicit live weapon feed
// Aim yaw / pitch (deg) of a model-space target seen from the chest origin (0, 0, kAimOriginZ): yaw = atan2(-x, y) (+ = left,
// CCW about +Z, GameIntegration / AimOffset convention), pitch = atan2(z - kAimOriginZ, |xy|). false = non-finite / zero.
bool AimAnglesFromModel(const float aTargetModel[3], float& aYawDeg, float& aPitchDeg);
std::string Status();

// ---- host
void SetDataDir(const std::string& aDir);           // layout root (the DLL folder)
void SetPathsOverride(int32_t aBody, const Paths& aPaths); // tests: explicit files per body
// ---- commit C prep (air): v2 consumer side of the runtime's UTPPFeedAir (report 3.15, 6.10.2, 6.10.8 commit C, 10.3)
// SetAirFeed: the air fields of the NEXT Step (one-shot, consumed by that Step; a Step without a SetAirFeed since the previous
// one runs with adapter::AirFeed.valid = false, as before this commit). Takes only the v2 mutex; no-op-safe (any thread, any
// time, also with mm.motionDriver off / no bundle; non-finite distance = no hit). Arguments = the runtime FeedInput air fields:
//   aOnGround         scriptInterface.IsOnGround()
//   aDistanceToGround downward raycast distance in m; -1 = no hit; -2 (any value < -1.5) = not probed this feed: v2 dead-reckons
//                     the last probe hit with the feed vz (dist += vz dt, >= 0) while airborne, else no floor. J2 (vehicle
//                     batch): a grounded not-probed feed on a ground / slide detail seeds the take-off plane (0); the seed is
//                     dead-reckoned the same way only for a flight started by a PSM jump detail (18-21, latched at take-off);
//                     a walk-off keeps no floor until the first probe hit, and any probe hit replaces the seed
//   aPsmFall          PSM.Fall (gamePSMFallStates; the previous frame's value, read by the feed wrap)
// WHERE THE RUNTIME CALLS IT (exact): Main.cpp Native_FeedLocomotion, inside the existing block
//   if (t_feedAir.valid) { in.haveAir = true; ...; t_feedAir.valid = false; }
// add, after that block and before motionhost::Feed(in) (UTPP-MOTION SEAM; v2 mutex only, the v1 g_mutex is not held there):
//   if (in.haveAir) motiondriver::SetAirFeed(in.onGround, in.distToGround, in.psmFall);
// (equivalently MotionHost.cpp Feed right before the MotionStepLocked seam from aRaw.haveAir / onGround / distToGround / psmFall).
void SetAirFeed(bool aOnGround, float aDistanceToGround, int32_t aPsmFall);
// The matcher configuration of the production bundle load (MotionDriver.cpp LoadBundle), header only so the offline replays (tests/JumpPickTest.cpp)
// run the rules the game runs. r5 air: the fall-loop hand-over of a vehicle jump-out flight is ON - with it off the ground jump-start clip
// (Jump_F_Start_Stand_*, authored for a ~0.6 s push-off) was held for the whole 1.2-1.5 s flight (V crouched mid-air) and a Start row could
// be picked again in the last feeds before the touchdown. JumpPickTest mutation 22 returns the pre-r5 configuration (hand-over off).
inline utpp::mm::motion::MatcherConfig ProductionMatcherConfig()
{
    utpp::mm::motion::MatcherConfig cfg;
    UTPP_IF_MUT(JUMPPICK, 22)
        cfg.jumpOutFallHandover = false; // mutation 22: the pre-r5 production configuration
    return cfg;
}
// The floor distance one v2 Step feeds to the trajectory (adapter::AirFeed::distanceToGround) from the SetAirFeed distance: header
// only and pure, so the offline replays (tests/JumpPickTest.cpp) run the exact production rule. aDetail = adapter::DetailFromFeed of
// the feed, aVz = feed capsule vz (m/s), aDt = Step dt. aMut: air test mutation builds only (6 no dead reckoning, 12 no take-off
// plane seed, 13 seed also for a walk-off); 0 = production.
struct AirFloorState
{
    float deadDist = -1.0f;  // last probe hit (or take-off plane seed) dead-reckoned with the feed vz (m), -1 = none
    bool seed = false;       // deadDist is the take-off plane seed (J2), not a probe hit
    bool seedJump = false;   // the flight was started by a PSM jump detail (18-21): the seed is dead-reckoned while airborne
};
struct AirFloorOut
{
    float distance = -1.0f; // >= 0 floor distance, -1 none
    bool seedUsed = false;  // airborne and the distance comes from the take-off plane seed
};
inline AirFloorOut AirFloorStep(AirFloorState& aS, bool aOnGround, float aDist, int32_t aDetail, float aVz, float aDt, int aMut = 0)
{
    // J2 (vehicle batch): the runtime probes only descending airborne feeds (MotionHost.cpp AirWantsProbe), so a jump had no floor
    // until its first descending probe (TimeToLand 1.5 s through the whole rise: Jumps_Far rows on short flights). A grounded
    // not-probed feed on a ground / slide detail seeds the take-off plane (0 m); a flight started by a PSM jump detail (the
    // take-off feed itself still touches the ground with detail 18) dead-reckons it like a probe hit. A walk-off (no jump detail
    // on the first airborne feed) keeps no floor until its first probe hit, as before; a probe hit always wins.
    AirFloorOut o;
    const bool jumpDetail = integ::PsmIsJumpDetail(aDetail);
    const bool notProbed = aDist < -1.5f;
    if (aOnGround)
    {
        if (!notProbed)
        {
            aS.deadDist = aDist >= 0.0f ? aDist : -1.0f;
            aS.seed = aS.seedJump = false;
        }
        else if ((adapter::PsmGround(aDetail) || adapter::PsmSlide(aDetail)) && aMut != 12)
        {
            aS.deadDist = 0.0f;
            aS.seed = true;
            aS.seedJump = false;
        }
        else if (jumpDetail && aS.seed)
        {
            aS.deadDist = 0.0f; // take-off feed: PSM Jump while the capsule still touches the ground
            aS.seedJump = true;
        }
        else
        {
            aS.deadDist = -1.0f;
            aS.seed = aS.seedJump = false;
        }
        o.distance = notProbed ? -1.0f : aDist; // grounded feeds: the probe as sent (no reckoning)
        return o;
    }
    if (aDist >= 0.0f)
    {
        aS.deadDist = aDist;
        aS.seed = aS.seedJump = false;
    }
    else if (notProbed && aS.deadDist >= 0.0f && aMut != 6)
    {
        if (aS.seed && (jumpDetail || aMut == 13))
            aS.seedJump = true; // latched at the take-off edge (the PSM falls 18 -> 14 at the apex)
        if (aS.seed && !aS.seedJump)
        {
            aS.deadDist = -1.0f; // walk-off drop: the take-off plane is not the floor
            aS.seed = false;
        }
        else
        {
            aS.deadDist = std::fmax(0.0f, aS.deadDist + aVz * aDt);
            o.seedUsed = aS.seed;
        }
    }
    else
    {
        aS.deadDist = -1.0f;
        aS.seed = aS.seedJump = false;
    }
    o.distance = notProbed ? aS.deadDist : aDist;
    return o;
}
// next: the runtime call site above as one function (Main.cpp Native_FeedLocomotion calls it right before
// motionhost::Feed(in); the air test calls the same function): in.haveAir (UTPPFeedAir this frame) -> SetAirFeed(in.onGround,
// in.distToGround, in.psmFall); no UTPPFeedAir -> nothing (the next Step runs with AirFeed.valid = false).
void FeedAirFromInput(const motionhost::FeedInput& aIn);
// Air diagnostics of the last Step that produced a v2 pose (tests / status "air:" segment).
struct AirDiag
{
    bool feedValid = false;     // an AirFeed was consumed by that Step
    int32_t movementMode = 0;   // chooser MovementMode (0 OnGround, 1 InAir, 2 Sliding with mm.slide)
    float timeToLand = 0.0f;    // chooser TimeToLand (s, trajectory floor)
    float distanceUsed = -1.0f; // floor distance fed to the trajectory (m; -1 = none), after not-probed dead reckoning
    bool justLandedLight = false, justLandedHeavy = false;
    float landVzCmS = 0.0f;     // MapState landing vz (cm/s)
    // root vertical (report 6.6.4: RTZ is a feature only, never applied to the pose): over the current air episode, the
    // integral of the playing frame's RTZ vz (animation root) and of the feed vz (capsule), and their difference (m)
    float rtzRootZ = 0.0f;      // RTZ root_z of the playing frame (m, absolute in its clip)
    float animDz = 0.0f, capsuleDz = 0.0f, residual = 0.0f, maxResidual = 0.0f;
    uint64_t airSteps = 0, episodes = 0, feeds = 0;
    // landing blend deficit bypass (v2 owns air): armed going into this Step's pose evaluation / consumed by its blend
    // request; landBypasses = consumed requests in total (at most one per landing)
    bool landBypassArmed = false, landBypassConsumed = false;
    uint64_t landBypasses = 0;
    uint64_t chooserDbMask = 0; // databases the chooser offered to that Step's search (MatchResult::dbMask, bit = database index)
    bool takeoffSeed = false;   // J2: that Step's floor came from the take-off plane seed of a PSM jump (no probe hit yet)
    int32_t stance = 0;         // chooser Stance of that Step (0 Stand, 1 Crouch; vehicle batch CAR-1 exit window check)
};
AirDiag AirDiagnostics();
// Selection diagnostics of the last Step that ran the chooser (tests / offline replay): the chooser inputs and the match result.
struct SelectionDiag
{
    int32_t gait = 0, movementState = 0, movementMode = 0, stance = 0;
    bool isStarting = false, isPivoting = false, shouldSpin = false;
    float speedCmS = 0.0f;
    int32_t interrupt = 0; // motion::Interrupt of the update
    bool searched = false, jumped = false, turnHold = false;
    float cost = 0.0f, continuingCost = 0.0f;
};
SelectionDiag SelectionDiagnostics();
void SetLogger(void (*aLog)(const char*));          // fallback reason lines / the first fault cause
// UTPP-MOTION SEAM motionfix (item 4, male V test): "UTPP mm.motionDriver fallback=<reason> [cause=..] body= bundle= rig= map= .." on every
// change of the Step's reason (also back to none = "v2 path"; was once per reason per session), at most 4 lines per 1 s of
// feed time [P] (excess counted: suppressed=); "UTPP mm.motionDriver load start #n body= why= from= db=<path>" when a load starts and
// "UTPP mm.motionDriver load done #n body= installed|dropped|fault:.. ms= dbBytes= bones= clips= dbs= frames= rigId= .. map= err=" on the
// next Step (game thread) after the worker finished. Status() appends bundle= rigId= rig= gameBones= map= loads= counters.
// UTPP-MOTION SEAM motionfix (item 4): "UTPP MOTION ..." trace line through the SetLogger logger on v2 frames (game thread,
// fixed buffer): db, clip, clip time, cost, blend time, rotation mode, IsStarting / IsPivoting, input yaw vs velocity yaw,
// Locomotion target flag, overlay pose key + crossfade weights, layering curves, AimingWeight, air (mode / ttl / land),
// traversal (active / montage / time), body / bundle (+ loading) / live rig / database rigId / bone map. On change (db, clip,
// rotation mode, IsStarting, IsPivoting, overlay pose key, air mode, body, rig,
// traversal) and every 0.5 s of feed time (2 Hz). Default off; Main.cpp UTPPSetMMTrace switches it with mm.trace.
void SetTrace(bool aOn);
void Shutdown();                                    // joins the worker, frees the bundle, disables
// Plugin only (MotionDriverNatives.cpp, RED4ext): natives of the FIXED NATIVE INTERFACE; Install = SetDataDir(<dll dir>,
// ANSI) + motionhost::SetMotionHooks; Uninstall = SetMotionHooks(nullptr) + Shutdown.
void RegisterNatives();
void Install(const std::wstring& aDllDir);
void Uninstall();

// ---- MMDriver seam (game thread, under the v1 g_mutex)
void OnArmed(uint64_t aRig, const std::vector<uint64_t>& aGameHashes, const std::vector<int16_t>& aGameParents);
void OnDisarmed();
// aRaw = Feed's unconditioned input (6.10: raw locoState / psmBlock). True = a v2 pose is ready for this feed.
bool Step(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt);
// commit A: the v2 frame of the last Step for the v1 leg tail (false = layout differs from v1 -> v2 yields this feed).
// aOut.buf / aOut.bufCap: caller-owned buffer (MMDriver Feed: a fixed array, no per-frame allocation). The v2 pose is copied
// into it under the v2 lock and aOut.pose = aOut.buf, so the caller may read it after the lock is released while the worker
// swaps / frees the bundle (body switch, reload). No buffer or bufCap < bones -> false (v2 yields this feed).
bool Frame(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut);
// Writes the last v2 pose (+ its bone map) into the v1 snapshot when v2 is running; false = snapshot untouched (v1).
bool ApplyToSnapshot(integ::PoseSnapshot& aSnap);

// ---- UTPP-MOTION SEAM trav: MotionTraversal (mm/MotionTraversal.hpp) at a game climb (PSM detail 8) / vault (9) start.
// UTPPFeedTraversal (scripts/utpp_motion.reds, ClimbEvents / VaultEvents OnEnter wraps) -> FeedTraversal: the plan is built once
// (traversal::v2::PlanTraversal on the body's traversal table + the playable .utpptrav montages: sequence = a clip of the v2
// database, or the montage carries version 2 poses), in FeedTraversal when the bundle is ready, else at the next enabled Step
// (OwnsTraversal); the plan waits (at most kTravEventMaxAge of feed time) for the first v2 Step with PSM detail == action.
// Plan Ok: while the detail stays 8 / 9 and v2 owns the feed, the montage pose (database clip, else the .utpptrav version 2
// poses, sampled at the Player's pose time through the evaluator, inertialized in) replaces the MM pick; the warped plan root minus the capsule is
// added to the root children (warp root). End of the montage (Player justTraversed) or detail leaving 8 / 9: MM resumes with
// JustTraversed and an inertialized fresh search. Plan not Ok (gate Rejected, unsupported, no montage, no traversal data):
// v2 yields (skip "trav-yield") for that traversal to the v1 path; with mm.climbRelease on (main 7c8bb58), or (vehicle batch,
// after the v1 gate takes MotionHooks::travActive) whenever TravActive(), the v1 gate releases Climb 8 / Vault 9 to the game's
// climb / vault animation ('travRelease') unless OwnsTraversal(); otherwise v1 holds its MM pose. Vehicle batch: a vault over a
// vehicle (TraversalEvent::obstacleKind 1) or deeper than 0.90 m (obstacleEnd past topPoint) is Unsupported (MotionTraversal.hpp);
// every plan writes one "UTPP trav:" info line (TravPlanInfoLocked).
// A pending event ages on every enabled Step (skipped ones too). No event: MM as before (v1 hold path unchanged).
struct TraversalEvent
{
    int32_t action = 0; // 8 Climb, 9 Vault (gamePSMDetailedLocomotionStates)
    float topPoint[3] = {0, 0, 0};
    float collisionNormal[3] = {0, 0, 0};
    float topExtent = 0.0f;
    float behindPoint[3] = {0, 0, 0};
    float obstacleEnd[3] = {0, 0, 0};
    float inputDirection[3] = {0, 0, 0};
    bool hasHands = false;
    float leftHand[3] = {0, 0, 0};
    float rightHand[3] = {0, 0, 0};
    float playerPos[3] = {0, 0, 0};
    float ownerForward[3] = {0, 1, 0};
    float speed = 0.0f;
    bool airborne = false;
    float climbSpeedModifier = 1.0f;
    // UTPP-MOTION SEAM next review minor 5: the live playerStateMachineLocomotion.climb / .vault static parameters, read
    // by the OnEnter wrap exactly as the game reads them (GetStaticFloatParameterDefault, LT:2727-2751 climb, LT:3063 vault).
    // haveTweaks false (older caller) or a non-finite / <= 0 value -> the 2.31 data (MotionTraversal.hpp GameTweaks defaults).
    bool haveTweaks = false;
    float highThreshold = 1.6f;
    float midThreshold = 1.2f;
    float verticalDuration[3] = {1.0f, 0.6f, 0.3f}; // High, Mid, Low
    float horizontalDuration[3] = {0.3f, 0.2f, 0.1f};
    float forwardStep = 0.0f; // climb.forwardStep (action 8) / vault.forwardStep (action 9)
    // Vehicle batch (TRAV-1 B): descResult.climbedEntity of the event, sent by UTPPFeedTraversalObstacleKind right before
    // UTPPFeedTraversal (MotionDriverNatives.cpp keeps it pending for exactly one event). kObstacle* values; other = static.
    int32_t obstacleKind = 0;
};
inline constexpr int32_t kObstacleStatic = 0;      // no climbed entity (world geometry)
inline constexpr int32_t kObstacleVehicle = 1;     // VehicleObject
inline constexpr int32_t kObstacleOtherEntity = 2; // any other entity
inline constexpr float kTravEventMaxAge = 0.5f; // [P] s of feed time an event waits for its PSM detail
bool FeedTraversal(const TraversalEvent& aEv); // false = invalid action / non-finite input (ignored)
// UTPP-MOTION SEAM trav (task A, agreed with the runtime session): true while v2 owns the current Climb 8 / Vault 9
// episode, i.e. it accepted a traversal plan for it. Lock-free (one std::atomic<bool> load, no v2 mutex): the v1 gate
// ('travRelease', runtime side) calls it inside MMDriver Feed under the v1 g_mutex, before v2's Step of that feed.
// Set: plan accepted (Ok). The plan is made in FeedTraversal when the body's bundle is ready (so the flag is already set for
// the next v1 Feed), else at the first enabled Step that reaches the traversal code (one feed later). Contract (main
// dc221fa): a false answer on an 8 / 9 feed releases v1 (gate closed -> v2 skips 'gate' and never plans), so a deferred
// event (not planned in FeedTraversal: bundle loading, v2 release / air ownership at the event, an episode running) is
// already owned while v2 can still plan it (v2 on, body known, no fault / budget trip, bundle loading or ready with
// traversal data and no fallback). Cleared: plan not Ok (rejected / unsupported / montage missing / no traversal data),
// any v2 yield of the episode (skip on a feed with the event's detail - also for a deferred event, which is then given
// up - or while the montage / episode runs), episode end (PSM detail leaves 8 / 9; the montage hand-back itself keeps it
// while the detail stays 8 / 9, MM then owns the feed), the event aging out before its detail arrived, fault, toggle
// off, body switch, bundle swap (a re-pended plan: owned while it can be planned again), re-arm / disarm, Shutdown
// (unload). Reached through MotionHooks::ownsTraversal (null hook = false); replaces main 7c8bb58's stub (same signature).
bool OwnsTraversal() noexcept;
// Vehicle batch (TRAV-1 A/B, MotionHooks::travActive): true while v2 can play motion traversals at all - mm.motionDriver enabled, no fault /
// budget trip, and the installed bundle carries traversal data (table + playable montages). The v1 gate uses it to hand a
// Climb 8 / Vault 9 that v2 does not own (OwnsTraversal false: rejected / unsupported, e.g. a vault over a vehicle) to the
// game's climb / vault animation instead of holding a ground MM pose. Lock-free (one std::atomic<bool> load), refreshed under
// the v2 mutex wherever the switch, the fault state or the bundle changes; false after Shutdown.
bool TravActive() noexcept;
// Vault torso limiter (mm/VaultTorso.hpp, MotionHooks::vehicleVaultWeapon): the last traversal event was a vault over a vehicle that v2 did not
// accept (Unsupported(vehicle), the game's vault animation plays) AND the overlay of the equipped weapon is a ranged pose (rifle / pistols).
// Lock-free (atomics only). The flag is set at the plan and cleared at the episode end / reset; the v1 feed additionally requires the
// travRelease reason, PSM Vault 9 and the weapon-drawn bit.
bool VehicleVaultWeapon() noexcept;
struct TraversalProbe // tests / status
{
    uint64_t events = 0, plans = 0, accepted = 0, yields = 0, frames = 0, handbacks = 0;
    int32_t lastResult = -1; // traversal::v2::StartResult of the last plan
    bool active = false, yielding = false;
    int32_t clip = -1;       // database clip played (-1: montage played from the .utpptrav version 2 poses)
    bool fromPoseFile = false; // last accepted plan samples the .utpptrav poses (montage sequence not a clip of the db)
    char montage[96] = {};   // name of the last accepted montage
    uint32_t playable = 0, playableFromPoses = 0; // montages of the installed bundle with a pose source (db clip / poses)
    float poseTime = 0.0f;   // montage time of the last frame
    float root[3] = {0, 0, 0}; // warped plan root of the last frame (world)
    float rootYaw = 0.0f;
    float offset[3] = {0, 0, 0}; // plan root - capsule (world) added to the root children this frame
    bool justTraversed = false;  // last Step handed back to MM with JustTraversed
    float handTarget[3] = {0, 0, 0}; // FrontLedge window target (world) of the plan
    float handContact[3] = {0, 0, 0}; // plan contact point (warped root + window offset) at the FrontLedge window end
    int32_t frontEndFrame = -1, startFrame = -1, endFrame = -1;
    float maxPlanStep = 0.0f, maxWarpAdded = 0.0f;
    // review minor 5: game values of the last plan (live tweaks of the event or the 2.31 defaults)
    int32_t climbClass = -1;        // CpClimbClass High 0 / Mid 1 / Low 2
    float gameDuration = -1.0f;     // climb: ClimbSpeedModifier * (vertical + horizontal duration); vault -1
    float destination[3] = {0, 0, 0}; // HorizontalDestination (climb) / Destination (vault)
    float handbackYaw = 0.0f;       // verifier round 2: handback turn of the last montage root yaw applied by the last MM Step (rad)
    // vehicle batch (TRAV-1 B/D): game-side rule of the last plan ("vehicle", "deep", "" none), its obstacle kind, and the
    // always-on "UTPP trav:" info lines written so far (at most one per traversal event)
    char reason[16] = {};
    int32_t kind = 0;
    uint64_t infoLines = 0;
};
TraversalProbe ProbeTraversal();

struct GripProbe
{
    bool supplied = false, applied = false, clamped = false;
    float error = 0.0f;
    float requestedReach = 0.0f, usedReach = 0.0f;
    float relativeTarget[3] = {};
    uint32_t source = 0; // 0 none, 1 game pose, 2 authored pistol overlay
    uint64_t authoredFrames = 0;
    float weaponShift = 0.0f;
    uint64_t accepted = 0, rejected = 0;
};
GripProbe ProbeGrip();

// ---- diagnostics / tests
adapter::Fallback Reason();
const char* LastSkipReason();         // why the last feed was handed to v1 / the game ("none" while v2 runs); static string
bool LoadedBody(int32_t& aBody);      // false = no bundle installed
uint64_t Loads();                     // bundles loaded so far (body switch reload check)
void WaitLoader();                    // blocks until the worker finished (tests, shutdown)
const integ::PoseSnapshot* LastMotionSnapshot(); // last snapshot written by ApplyToSnapshot (tests)
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
// Test-only, bounded rendezvous after WaitLoader moves its thread out of g_worker.t and releases g_m.
void TestWaitLoaderPauseAfterMove(bool aEnable);
bool TestWaitLoaderMoved();
void TestWaitLoaderResume();
void TestWorkerPause(bool aEnable);
bool TestWorkerBlocked();
uint32_t TestShutdownEntries();
void TestWorkerResume();
void TestThrowInNextStep(); // unit test target only: the next enabled Step throws inside the lock (fault path, 6.10.6)
void TestKeepRetiredPoses(bool aOn); // unit test target only: bundles replaced by the worker keep their pose buffer, NaN-filled
size_t TestRetiredPoses();             // pose buffers retired so far (TestKeepRetiredPoses)
// unit test target only: every following motion feed records aWallUs as its wall time and aThreadUs as the thread's own time
// for the feed budget (aWallUs < 0 = measured again; aThreadUs < 0 = no thread time, wall time alone).
void TestSetFeedTimesUs(float aWallUs, float aThreadUs);
// unit test target only: the magazine top-up gesture of the last main-path Step, with the published MM pose (database bone order,
// 7 floats per bone: p xyz, q xyzw) right before and right after the gesture edit of that same Step.
struct MagTapProbe
{
    bool stepped = false;   // the last Step reached the gesture pass (main path)
    bool owned = false;     // ... and MM owned the arms (admission state)
    bool edited = false;    // the pose was edited by the gesture
    float weight = 0.0f;
    int32_t kind = -1;      // magtap::Kind of the gesture
    float handMoved = 0.0f, handErr = 0.0f, targetDist = 0.0f;
    bool clamped = false;
    std::vector<float> before, after;
};
MagTapProbe TestMagTapProbe();
#endif
// ---- ragdoll R0 probe (debug, CET utpp_probe RagdollProbe; natives UTPPRagdollGetUpProbe / ...Stop / ...Status): publishes frame 0 of a
// motion get-up clip (set3 M_Neutral_GetUp_Back, from an embedded table: the v2 databases carry no get-up clip, stand_F/B/L/R neither) with the Hips offset
// aOffsetM along the character-frame +Y (forward of the entity when the Root bone is aligned with it), held aDurationS (0.5..5 s),
// then handed back to MM. Requires mm.motionDriver on and running (bundle ready, armed). Zero effect unless commanded. Logs "[utpp_ragdoll]
// getup-probe ..." lines through the SetLogger logger. Returns a one-line answer: "started: ..." or "refused:<why>".
std::string RagdollGetUpProbeStart(float aOffsetM, float aDurationS);
std::string RagdollGetUpProbeStop();   // aborts a run ("stopped" / "idle")
std::string RagdollGetUpProbeStatus(); // "idle ..." or "hold|handback ..."

// ---- ragdoll native get-up (ragdoll/GetUpPlayer.hpp; natives UTPPRagdollGetUp* in MotionDriverNatives.cpp, sections 3.6 of the get-up
// spec). Game thread (script VM), each takes the driver lock. Nothing happens unless the script calls Plan / Begin; with the setting
// off (the shipped CET default is ON; the script pushes the setting, the DLL starts with false until the first push) or the clip file
// missing every function answers "unavailable" and the vanilla recovery stays unchanged.
void RagdollSetGetUp(bool aOn);
// Setting on, v2 enabled and not faulted, TPP, bundle with valid get-up clips for the current body, a ragdoll pose store younger
// than 0.15 s (aNowTicks / aHz: QueryPerformanceCounter).
bool RagdollGetUpAvailable(int64_t aNowTicks, int64_t aHz);
// Freezes the pose store, converts the snapshot, classifies the lie, aligns. aEntityRot: i, j, k, r. aRefSign: PelvisTracker sign
// (aHaveRef false = take the clip's). Returns 0..3 (B, F, L, R) or GuPlanCode < 0; on < 0 the store is unfrozen.
int32_t RagdollGetUpPlan(const float aEntityPos[3], const float aEntityRot[4], int64_t aNowTicks, int64_t aHz, bool aHaveRef, float aRefSign);
void RagdollGetUpTarget(float aOut[4]);  // anchor x, y, z, rms (zeros without a plan)
void RagdollGetUpForward(float aOut[2]); // world forward of the anchor (x, y)
// aGroundZ: script ray at the anchor (NaN / < -1e5 = none). false = refused (no plan, plan stale, state changed).
bool RagdollGetUpBegin(float aGroundZ);
// Begin + the v1 publish that carries the get-up pose before the first get-up feed (motionhost::PublishGetUpGap, called after g_m is released).
// The script natives call this; RagdollGetUpBegin alone leaves the frames before the first feed to the game pose.
bool RagdollGetUpBeginPublish(float aGroundZ);
// The v1 publish of RagdollGetUpBeginPublish (motionhost::PublishGetUpGap), registered next to the v2 hooks (null = none, e.g. a build without the v1 host).
void SetGetUpGapPublisher(void (*aFn)());
void RagdollGetUpCancel();
int32_t RagdollGetUpState(); // 0 idle, 1 blend, 2 play, 3 handoff, -1 the last run aborted (cleared by the next Plan)
std::string RagdollGetUpStatus();
void GetUpAbort(const char* aWhy); // ends a running get-up (any thread that may take the driver lock), no-op when idle
bool RagdollRecoverActive();       // section 5: the post-ragdoll window (also used by tests)

#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
// test targets only (never the DLL): the next plans use motiontrav::Config::forceMontage = aMontage (empty = decision table) and
// Config::gate = aGate; plays one specific montage through the real path (trav: all 34 playable)
void TestForceTraversal(const std::string& aMontage, bool aGate);
// test targets only (never the DLL): the clock of the authored recoil (shot pulse window, clip time, admission ramp, camera target age)
// becomes a simulated one, aSeconds >= 0 (< 0 = the QPC wall clock again), advanced by TestAdvanceRecoilClock. Makes the recoil tests
// independent of the machine load.
void TestSetRecoilClock(double aSeconds);
void TestAdvanceRecoilClock(double aSeconds);
// test targets only (never the DLL): the steady clock of the driver (SteadyNs: native get-up wall watchdog, lift slew between publishes,
// yield timers, Shooter hold ages) becomes a simulated one, aNs >= 0 (< 0 = std::chrono::steady_clock again), advanced by
// TestAdvanceSteadyClockNs. Makes the get-up host test independent of the machine load.
void TestSetSteadyClockNs(int64_t aNs);
void TestAdvanceSteadyClockNs(int64_t aNs);
#endif
} // namespace utpp::motiondriver

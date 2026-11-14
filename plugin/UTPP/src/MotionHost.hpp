#pragma once
// Phase 4b motion matching driver (game thread). Owns the database, the BaseMatcher, gating, the weight follower and
// publishes PoseSnapshot into g_mmPose for the FK detour. All entry points lock an internal mutex (game thread only;
// never called from the FK detour).
//
// Data flow: redscript LocomotionEventsTransition.OnUpdate -> UTPPFeedLocomotion -> Feed() -> BaseMatcher::Update ->
// Publish(snapshot{weight, rig, bone map, pose}) -> FK detour (animation thread) -> BlendIntoRecords before FK.
// Tick() (RED4ext Running-state update, every frame) fades the weight to 0 when no feed arrived for kStaleFeedSeconds.

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace utpp::mm::integ { struct PoseSnapshot; } // namespace utpp::mm::integ
namespace utpp::mm { struct BoneTransform; } // namespace utpp::mm

namespace utpp::motionhost
{
inline constexpr uint64_t kExpectedRigHash = 0x72CB314EB01F562Dull; // player_woman_skeleton 0..72 (Phase 3 bake)
inline constexpr float kDefaultBlendTime = 0.25f;                   // s, smoothstep
inline constexpr float kDefaultHardOutTime = 0.12f;                 // 0.3.2: fade-out for hard closures (PSM block...)
inline constexpr float kDefaultReopenHold = 0.10f;                  // 0.3.2: open time required after a state closure
inline constexpr float kStaleFeedSeconds = 0.2f;                    // Tick(): no feed for this long -> target 0
inline constexpr float kDetourStaleSeconds = 0.5f;                  // FK detour ignores snapshots older than this

struct FeedInput
{
    float velWorld[3] = {0, 0, 0};
    float move[2] = {0, 0};
    float cameraYawDeg = 0.0f;
    float playerYawDeg = 0.0f;
    int32_t locoState = 0;
    bool tppActive = false;
    float dt = 0.0f;
    // Phase 5.1: world target yaw (deg) = camera yaw + Locomotion raw stick angle; valid only while Locomotion is
    // active and the stick is moved. Without it the body-turn path is inactive (legacy matcher input).
    bool haveTarget = false;
    float targetYawDeg = 0.0f;
    // 0.3.2: PsmBlock bits from scripts/utpp.reds UTPPPsmBlockMask (vehicle, scene, workspot, swim...); != 0 -> gate.
    uint32_t psmBlock = 0;
    // batch 1b (2026-10-13): extra fields sent by scripts/utpp.reds UTPPFeedLocomotionExtra right before the feed (false: an
    // older script did not send them). walkToggled = PSM permanent bool WalkToggled (vanilla walk toggle: LocomotionDetailed
    // Crouch while Locomotion stays stand; never a psmBlock bit), capsulePos = player entity world position. Diagnostics only
    // (feed dump columns walk, capX capY capZ); the matcher does not read them.
    // UTPP-MOTION SEAM mm.motionDriver: MotionDriver Step reads walkToggled (haveExtra) as motion WantsToWalk (MotionFeed.wantsToWalk).
    bool haveExtra = false;
    bool walkToggled = false;
    float capsulePos[3] = {0, 0, 0};
    // AIR HOLD stage (2026-10-13): fields of scripts/utpp.reds UTPPFeedAir, sent right before this feed (haveAir false: an older
    // script / no air feed this frame -> the air area infers onGround from the PSM detail). onGround = IsOnGround();
    // distToGround = downward raycast distance (m) while airborne and UTPPMMAirWantsProbe(), -1 = no hit, -2 = not probed;
    // psmFall = PSM.Fall read by the feed wrap (written by the game AFTER it: the previous frame's value).
    bool haveAir = false;
    bool onGround = true;
    float distToGround = -2.0f;
    int32_t psmFall = 0;
    // Internal v1 -> v2 handoff, filled after this feed's AirState update. Not script inputs.
    // Zero means unavailable (standalone v2 callers retain their MapConfig default).
    float predictedGravityUp = 0.0f;
    float predictedGravityDown = 0.0f;
    // Fresh native base Deceleration stat (game m/s^2), after removing our own
    // multiplier. Internal only; zero retains the standalone motion prediction.
    float capsuleBrakingDeceleration = 0.0f;
    // Fresh actually applied Acceleration stat, game m/s^2. Prediction only;
    // zero keeps the original trajectory model for untrusted/missing samples.
    float capsuleAcceleration = 0.0f;
    // Native host -> MOTION only. Never read from script or retained feed dumps.
    bool haveRootContext = false, nativeRootEnabled = false;
    bool nativeBodyTurnEnabled = false;
    // g_bodyTurnOn itself (UTPPSetMMBodyTurn state; nativeBodyTurnEnabled above also needs a body-turn feed). Native host -> MOTION only (H2 safeguard:
    // the body turn is switched off). The default true keeps hand-built inputs on the pre-safeguard path.
    bool nativeBodyTurnSwitch = true;
    // Native host -> MOTION only: the vehicle exit window is open (V just left a vehicle). The hip-fire pose is held through it.
    bool vehicleExitWindow = false;
    // Native host -> MOTION only: that exit window opened as a bike jump-out (MMDriver JumpOutLatch); the hold ends at the touchdown.
    bool vehicleJumpOut = false;
    // Native host -> MOTION only: the melee full-body layer owns the whole body on this feed (PSM.MeleeWeapon is a melee weapon
    // state and the full-body layer would be on if the base pose owned the body, or still has weight). A "Game" overlay (melee
    // weapon drawn) releases MOTION only while this is set.
    bool meleeOwnsBody = false;
    // Native host -> MOTION only: the game's melee clip (or the melee guard pose) owns the upper body only (a melee weapon state and
    // the melee gate active - melee bits or their release hold -, the guard on / would be on / fading out, or the upper mask still
    // fading; never together with meleeOwnsBody). MOTION keeps running and the lower body, applies the host's upper mask and fade
    // on its own bones, and keeps its head / orientation / lean layers quiet.
    bool meleeUpperMasked = false;
    // Native host -> MOTION only: the game pose captured at a normal (non jump-out) vehicle exit whose hand-in was accepted on THIS feed
    // (MMDriver MotionStepLocked, null on every other feed / path). MOTION turns it into the PoseHistory of the first search after the
    // re-entry (ExitPoseHistory.hpp) instead of the database mean. Layout of the vehicle capture: local pose per database bone of
    // the frozen snapshot with its game indices.
    const mm::integ::PoseSnapshot* vehicleExitPose = nullptr;
    // Native host only (Tick): a feed the driver made itself for a missing game update (r3b landing: the PSM landing state after a jump-out gives no
    // locomotion update). Never set by a script feed; the dump skips it and the jump-out hold counters do not advance on it.
    bool synthetic = false;
    uint64_t rootFeedSequence = 0;
    float capsuleDeltaCurrent[2] = {0, 0};
    float capsuleYawDelta = 0;
    // Native host -> MOTION only: the body turn requested for the coming tick (rad). The game applies it after this feed read the
    // yaw, in the same tick, so the pose of this feed is drawn with playerYaw + capsuleYawAhead (rootframe::Inputs::capsuleYawAhead).
    float capsuleYawAhead = 0;
    // Recorded replays: the coming tick's turn in degrees (the next feed's yaw minus this one). NaN = the host derives it from its last
    // body yaw command (BodyYawCommand issued since the previous feed for the yaw of this feed).
    float yawAheadDeg = std::numeric_limits<float>::quiet_NaN();
};

// Loads <aPath> (wide path). Fail closed: on error MM stays unavailable, reason kept for the status.
// aName = name reported by DatabaseName() ("default" = <dll dir>\utpp.utppmm).
bool LoadDatabase(const std::wstring& aPath, const std::string& aName = "default");
// b5 testorder: per-session runtime state (feed / input / gate / playback state, diagnostic counters) back to a freshly loaded
// DLL's values; settings, database, armed rig and feed dump kept. LoadDatabase applies it itself (start of the load); exposed
// for the offline suites (each loco section starts from the same state whatever ran before). A runtime database switch
// (UTPPLoadDatabase -> swap) resets only the playback state of the old database's clips, not the player's input / gate state.
void ResetSessionState();

// ---- 0.5.1 runtime database switching ----
// Database folder layout: <dir>\utpp.utppmm = "default", <dir>\db\<name>.utppmm = <name>.
void SetDatabaseDir(const std::wstring& aDir);
// "default;name1;name2" (default only if <dir>\utpp.utppmm exists; db\ names sorted, case-insensitive).
std::string ListDatabases();
// Valid names: 1..64 chars of [A-Za-z0-9_.-], no leading '.'.
bool IsValidDatabaseName(const std::string& aName);
// Starts loading <name> on a worker thread (read, SHA-256, full validation, same rig hash and bone set as the active
// database, matcher init probe). When ready, MM fades out (hard closure), the database / matcher / bone map are swapped
// on the game thread (Feed or Tick) with all runtime state reset, and MM fades back in. Any failure keeps the current
// database. False = invalid / missing name or a load already running; true = queued, or already the active database.
bool RequestDatabase(const std::string& aName);
std::string DatabaseName();          // active database name ("none" when nothing is loaded)
std::string DatabaseSha();           // hex SHA-256 of the active file ("" when unknown)
// "idle" | "loading <name>" | "pending <name>" (loaded, waiting for the fade-out) | "ok <name>" | "FAILED <name>: reason"
std::string DatabaseSwitchStatus();
// Blocks until no load is running (tests, shutdown).
void WaitDatabaseWorker();

// ---- 0.5.2 per-database speed calibration (mm/SpeedCalibration.hpp) ----
// On every database load / switch the database side of the speed map (and the slow-walk floor = walk point) is derived
// from the database's forward loop clips, optionally overridden by <database>.json next to the file. UTPPSetMMSpeedMap /
// UTPPSetMMSpeedPoint still override at runtime until the next load. Summary also in Status() (speedCal="...").
std::string SpeedCalibrationSummary();
// Stance-change transition clips (steer::kClassStanceTransition) allowed only this long after a stand <-> crouch change
// (s, default 0.6; < 0 = always allowed = 0.5.1). False = invalid (> 5 or NaN).
bool SetStanceTransitionWindow(float aSeconds);
// ---- 0.5.3 locomotion fixes from the gamepad tests (all default on; off = 0.5.2) ----
// Stick filter: |move| > 0.05 counts as held (lifted to 0.16), releases shorter than 0.12 s are bridged (180 deg flick).
void SetStickFilter(bool aOn);
// Per-clip playback-rate factor (source family / own forward-loop speed) + slow-walk query = slowest forward walk loop.
void SetFamilyRate(bool aOn);
// Start rules: no pivots / moving turns below 0.8 m/s, no stop clips while the stick is held, no sprint starts unless
// sprinting (steer::CurationConfig::startRules).
void SetStartRules(bool aOn);
// ---- 0.5.4 speed bands (default on; off = 0.5.3) ----
// Pivots / moving turns only of the speed class of the motion's band (walk / run / sprint; per-database edges from the
// speed calibration, hysteresis; next slower class only when the database has none for the band), no idle clips while
// the stick is held and the capsule moves (steer::CurationConfig::speedBands, mm/SpeedBands.hpp).
void SetSpeedBands(bool aOn);
// ---- 0.5.6 (default on; off = 0.5.5) ----
// Gate debounce: a soft closure (loco state not stand / crouch / sprint and/or PSM detailed locomotion only, |vz| < 1 m/s)
// must persist aSeconds before it closes an open gate (default 0.06; 0 = off = 0.5.5; false = invalid: < 0, > 0.5, NaN).
// Vehicle / scene / workspot / swim / takedown / carry / vitals / menu / stagger / TPP off / stale feed close at once.
bool SetGateDebounce(float aSeconds);
// Crouch rules (steer::CurationConfig::crouchRules): stance transitions only in the direction of the current stance and
// not above 6 m/s (slide entry); in crouch, lateral loops / moves (Left / Right / SideStep / Strafe) not for forward
// motion when the crouch set has a non-lateral forward loop.
void SetCrouchRules(bool aOn);
// ---- 0.5.7 (default on; off = 0.5.6) ----
// Crouch direction tag: with the crouch rules, crouch loops / moves whose direction tag is not forward (L / R / B / BL /
// BR / LL / LR / RL / RR after "_Loop_" / "_Move_") count as lateral (excluded for forward motion where a forward loop exists).
void SetCrouchDirTag(bool aOn);
// Stance transitions once per stance change and only from their first frames (steer::kStanceStartFrames). The first-frames
// window is a matcher frame range: it follows the switch at the next matcher init (UTPPSetMMEnabled(true), database load).
void SetStanceOnce(bool aOn);
// Turn commit: a playing moving turn / pivot / reface clip is not replaced by another turn-class clip until its root turn
// is done (same turn direction, stick held).
void SetTurnCommit(bool aOn);
// Standstill idle: no stick (or MM gated out) and capsule < 0.2 m/s -> idle candidates only (a playing stop clip ends).
void SetStandIdle(bool aOn);
// Stage standstill idle (default on; off = batch 1): while the standstill idle rule has held for 0.25 s and the stance's clip
// set has a looping idle, a playing idle or a clip with <= 12 frames left is followed by the idle family only - no re-entry
// into a stop clip's tail (the motion stops re-entered their own tail about once per second). Status: idleHold= (only while on).
void SetIdleHold(bool aOn);
bool IdleHold();
// Start turn: while an animation-led start / turn is wanted (20..160 deg), clips whose root turns against the body's
// (shortest) turn direction and slow turning starts (95 deg over ~1.3 s) are not candidates;
// near reversals (>= 160 deg) keep both directions and the body turns with the playing clip; a playing start / reface /
// turn-in-place segment turning on the spot (root < 0.5 m/s) runs at least at rate 1 below 1 m/s capsule.
void SetStartTurn(bool aOn);
// ---- 0.5.8 (default on; off = 0.5.7) ----
// Speed cap: a capsule faster than aFactor x the calibrated game sprint speed (default 1.25 x 8.2 = 10.25 m/s) is unsupported -
// gate bit kGateOverSpeed (no debounce, normal fade to 0, reopen hold after it clears); the matcher query velocity is clamped
// to the sprint speed. 0 = off; false = invalid (< 0, 0 < f < 1, > 4, NaN).
bool SetSpeedCap(float aFactor);
// Cost cap: a search whose best candidate costs more than aCost keeps the playing clip (<= 0.3 s in a row); without a
// continuing frame it searches the ordinary candidate set again (no turn commit / settle, no stance transitions). Default 100;
// 0 = off; false = invalid.
bool SetCostCap(float aCost);
// Stance cap: the turn commit applies only while the playing clip is still in the clip set (stance change), and a stance
// transition is never selected above the cost cap (the retry result is taken).
void SetStanceCap(bool aOn);
// Turn settle: after a committed turn clip reached 90 % of its root turn, no other turn-class clip for 0.35 s while the
// remaining error stays on the same side (no same-direction restart; a reversal is free).
void SetTurnSettle(bool aOn);
// Loop bands: stand / sprint loops / moves and speed transitions only of the speed band of the query speed (transitions by
// their source class; loops fall back to the next slower class when the database has none of the band's class).
void SetLoopBands(bool aOn);
// Start bind: a playing start / reface / turn-in-place clip whose root still turns binds the body turn to its root-yaw sign
// (root profile) until 90 % of its turn (the body turns only in the clip's direction, the long way if needed: animation step,
// fallback step in the same direction), and for the first 0.5 s only clips turning the same way are candidates, unless the
// stick moves > 90 deg.
void SetStartBind(bool aOn);
// ---- 0.5.9 (Phase 5.3; default on; off = 0.5.8) ----
// Capsule sync: per-database start / stop clip root-speed profiles (measured at load, mm/CapsuleSync.hpp) replace the fixed
// acceleration targets (stand 3.75 / crouch 12.2 / sprint 6.96) and give a stop deceleration target (DecelTarget).
void SetCapsuleSync(bool aOn);
// b5 testorder repair (default on; off = 0.7.0 swap): a runtime database swap (UTPPLoadDatabase) clears the playback state of the
// old database's clips (turn commit / settle / body follow / start rate / rate flags, chest align, StanceGuard filter, RootRate
// window, the matcher's cost-cap hold). Status swapReset= swapResets= swapCarry= swapPost= (bits: see PlaybackCarryLocked).
void SetSwapReset(bool aOn);
// Offset catch-up: offset root soft limit (above aSoftDistance the excess decays with aCatchupHalflife), translation
// half-life shrinking to aCatchupHalflife over aCatchupTime while the body is away from the capsule, hard clamp aMaxDistance
// (defaults 0.30 m / 0.6 s / 0.15 m / 0.06 s); the foot lock slides softly above the slip start (0.05 m, half-life 0.02 s).
void SetOffsetCatchup(bool aOn);
bool SetOffsetCatchupParams(float aMaxDistance, float aCatchupTime, float aSoftDistance, float aCatchupHalflife);
bool SetFootSlipParams(float aStart, float aHalflife);
// Start-class slip start (m): planted toe drift allowed in Start / StartSprint clips before the soft lock slides. Default 0.09; 0 or <= the base slip = the single slip.
bool SetFootStartSlip(float aStart);
float FootStartSlip();
// Trajectory calibration: matcher-time velocity half-life per locomotion state from the capsule profile (0.413 v / a * rate).
void SetTrajCal(bool aOn);
// Called by the script after the feed with the vanilla (unmodified) Deceleration stat and the multiplier currently applied by
// the script; returns the stop deceleration target (game m/s^2, 0 = remove the modifier). Non-zero only with capsule sync on,
// MM driving the body (gate open, fresh feed, stand / crouch / sprint) and the stick released. The script only lowers.
float DecelTarget(float aBaseDecel, float aAppliedMul);
// ---- 0.6.0 (default on; off = 0.5.9) ----
// Melee gate: PSM.MeleeWeapon attack (ComboAttack..EquipAttack, RangedWeapon QuickMelee) / pose (Parried, Hold, ChargedHold,
// Block, Targeting, Deflect) bits of the feed (scripts/utpp.reds) close the gate (kGateMelee: soft - debounced like a crouch
// blip, normal fade) and keep it closed 0.3 s after the state ends. Off = the bits are ignored (0.5.9).
void SetMeleeGate(bool aOn);
// Mount cut: the vehicle PSM bit (or MountingEvent through MountEvent) cuts the MM weight to 0 at once (no fade-out) and keeps
// it 0 while the vehicle state lasts; the reopen follows the normal hold + fade-in. Off = 0.12 s hard fade (0.5.9).
void SetMountCut(bool aOn);
// aIsCar: the mounted vehicle is a car (utpp.reds UTPPMMMountEventKind): the mount transition may blend a large root yaw
// (vehtrans YAW ALIGN). Default / UTPPMMMountEvent = not a car = the cumulative hold only.
void MountEvent(bool aIsCar = false);
// Per-mount hold cap of the standing-entry root offset (vehtrans HOLD CAP; utpp_vehicle_door.reds UTPPSetVehicleEntryHoldCap): seconds
// after the mount cut by which the visible root must be released onto the entity path; < 0 = none. May run just before or up to ~0.1 s
// after the mount cut.
void SetVehicleEntryHoldCap(float aSeconds);
// Stance clip minimum speed: a moving into-crouch transition (RunToCrouch / WalkToCrouch, steer::kClassStanceMoveToCrouch) is a
// candidate only when the capsule speed at the stand -> crouch change was >= aMps and the stick >= 0.7 forward (default 3.5;
// 0 = off = 0.5.9; false = invalid < 0 / > 12 / NaN).
bool SetStanceClipMinSpeed(float aMps);
// ---- 0.6.2 (default on; off = 0.6.1) ----
// Upper-body bone mask (design 29 item 4, profile kMaskUpperFromGame: Hips, Spine, legs and feet by MM; Spine1..3, neck, head,
// clavicles, arms, hands, fingers, WeaponLeft / WeaponRight by the game). For the masked states the gate does not fade the
// whole body out: MM keeps the lower body with the normal stand / walk query and the upper-body bones fade to the game pose
// over 0.15 s (and back). Off = every closure fades the whole body (0.6.1).
void SetUpperMask(bool aOn);
void SetAimUpperFade(bool aOn); // Batch6: aim exit fades over 0.25 s; off = legacy 0.15 s
void SetWeaponUpper(bool aOn); // known WeaponRight held: game upper body over MM legs, also in owned/held AIR
// Masked states (bits): 1 melee gate (attack / block / targeting + release hold), 2 PSM.UpperBody Aim, 4 vehicle exit window
// (from the end of the PSM vehicle state until MM is fully back, max 2 s; no reopen hold). Default 7; false = invalid (< 0, > 7).
bool SetUpperMaskStates(int32_t aBits);
// Melee full body (2026-11-09, mm/MeleeFullBody.hpp). Flag = script (UTPPSetMeleeFullBody: UTPP third person on foot and a
// covered melee weapon type; default off = the upper-body mask only, bit for bit); setting = user toggle "Melee full body"
// (UTPPSetMeleeFullBodySetting, default on). Both on + standing melee mask -> Hips, Spine and the legs fade to the game pose too.
void SetMeleeFullBody(bool aOn);
void SetMeleeFullBodySetting(bool aOn);
// Round 3 (2026-11-09 review, mm/MeleeFullBody.hpp). Allowed PSM.MeleeWeapon states for the full body (bit = state 0..21; default
// mm::melee::kFullBodyDefaultStates = the attacks without ThrowAttack; false = bits outside 0..21, kept). Guard: script flag
// (guard-capable melee weapon drawn) + user settings melee.guard / melee.guardFullBody (stage 2: legs) / melee.guardAimTilt,
// all default off (= the previous behaviour bit for bit).
bool SetMeleeFullBodyStates(int32_t aMask);
void SetMeleeGuard(bool aOn);
void SetMeleeGuardSetting(bool aOn);
void SetMeleeGuardFullSetting(bool aOn);
void SetMeleeGuardTiltSetting(bool aOn);
// Melee aim tilt log line source (Main.cpp: meleeaim::FormatLogLine; null = no 'UTPP MELEEAIM' lines). Trace only, 4 Hz.
void SetMeleeAimLogSource(bool (*aSource)(char*, size_t));
// Melee full pose log line source (Main.cpp: meleepose::FormatLogLine; null = no 'UTPP MELEEPOSE' lines). Trace only, about 1 Hz
// while a melee weapon is out.
void SetMeleePoseLogSource(bool (*aSource)(char*, size_t));
// Melee full pose: receives the PSM melee weapon state and the planar capsule speed (m/s) of every feed (Main.cpp:
// meleepose::SetMeleeState; null = none).
void SetMeleePoseStateSink(void (*aSink)(int32_t, float));
std::string MeleeFullBodyStatus();
// Hold last pose (default on; off = 0.6.1). In TPP the game's own full-body pose is a T-pose outside the states
// it animates, so a closure keeps the MM body (no fade-out, no reopen hold; without feeds Tick republishes the last pose):
// MM disabled, database switch (swapped at full weight), gate debounce, unsupported locomotion state, melee / aim / exit
// window without the mask, over-speed, stale feed (menu, pause), stagger. Released (normal fade): vehicle, scene, workspot,
// swim, ladder, takedown, body carry, death, knockdown, TPP off, no valid MM pose.
void SetHoldPose(bool aOn);
// CLIMB RELEASE stage (driver default on; the CET menu default is off and is pushed at load; off = hold-eligible 8 / 9 as
// before): PSM LocomotionDetailed Climb 8 / Vault 9 release the body to the game animation (release reason travRelease, normal
// gate fade) unless v2 owns the traversal (MotionHooks::ownsTraversal). TRAV-1: with v2 traversal active (MotionHooks::travActive) the
// release happens whatever this setting says; the setting only decides for v1 (mm.motionDriver off). Always true. Status: climbRelease= travRelease= releaseReasons= travReleaseFeeds= travReleaseEpisodes= travOwnedFeeds=.
bool SetClimbRelease(bool aOn);
// ---- 0.6.3 (default on; off = 0.6.2) ----
// Turn rate ("run turns in slow motion, the body slides"): with the stick held the playback rate is the
// loop rate (game speed / database loop speed x clip factor: motion run 3.85 / 5.46 = 0.70), which is
// right for loops but played the plant of a moving turn / pivot / reface (root 5.5 -> 0.6 -> 5.5 m/s) at 0.70: the turn took
// 1.43 x its authored time while the capsule kept 3.85 m/s (offset root at its clamp, feet dragged). While a turn-class clip
// (pivot / moving turn / reface) plays its plant (root below its own peak root speed) with the stick held and a recent capsule
// speed >= 1 m/s (decaying max, half-life 0.5 s), the rate is raised towards the clip's own timing:
// target = clamp((capsule + 0.5) / (root + 0.5), aMin, aMax), weight = (peak - root) / (0.35 peak) in [0, 1] (run-in / run-out
// frames at the peak keep the loop rate), smoothed 0.015 s up / 0.04 s down; the rate is only raised, never lowered.
// Defaults 0.9 / 1.4 (0.6.4; 0.6.3 shipped 1.1). 0.6.4: one info line per turn through the logger when the raise ends:
// "turnRate: clip=<name> avg=<rate> peak=<rate> dur=<s> capsule=<m/s> root=<m/s>" (means over the raised feeds).
void SetTurnRate(bool aOn);
bool SetTurnRateParams(float aMin, float aMax); // false = invalid (NaN, aMin < 0.5, aMax > 2, aMin > aMax)
// "turnRate on= min= max= lastTarget= active= w= feeds= turns= last=\"<last turn summary>\"" (0.6.4).
std::string TurnRateStatus();
// ---- B2 strafe S1 (2026-10-13; default on - CET bodyturn.aimStrafe; off = the 0.6.5 path bit for bit) ----
// Aim strafe (runtime motion RotationMode=Strafe): while V aims (PSM UpperBody 6) with the upper-body mask Aim state (weapon in hand;
// aUnarmed: also without), on the ground (|vz| < 1, no air / land detail) and the stance has facing-fixed loops in >= 6 of 8
// directions, the legs walk in the stick direction relative to the body: strafe class set (mm/Strafe.hpp: clips whose ROOT turns
// >= aRootTurnMaxDeg, turn / reface / turn-in-place / arc clips and off-gait patterns never chosen; facing-fixed patterns /
// pivots allowed), facing fixed with the trajectory yaw = the measured body step at rate 0, per-direction loop speed map + loop
// own-speed rate law, slow-walk gate bypassed, no turn-rate boost, no heading lean, root warp <= aRootWarpMaxDeg. Leaves after
// aExitHold s without aiming (capsule direction meanwhile). aJogF > 0 = S4 jog gait (run loops, forward game speed; the reds
// MaxSpeed multiplier: AimJogTarget). SetStrafeParams: false = invalid (NaN, rootTurnMaxDeg outside 1..90, rootWarpMaxDeg outside
// 0..90, exitHold outside 0..1, jogF not 0 / 3.3..6 = strafe::kJogMinF..kJogMaxF), previous kept. Status gets a strafe segment while on.
void SetStrafe(bool aOn);
bool SetStrafeParams(bool aUnarmed, float aRootTurnMaxDeg, float aRootWarpMaxDeg, float aExitHold, float aJogF);
// Internal (tests / tuning, no native): facing-fixed Box / Diamond / Hourglass / Prism patterns allowed while strafing.
void SetStrafePatterns(bool aOn);
// Tests: strafe mode Active / Exiting; strafe data digest (availability, sectors, speed table, class set 3 hash).
// B2 strafe S4 jog while aiming (native UTPPMMAimJogTarget; reds UTPPUpdateAimSpeed): target capsule speed (m/s) for the aim walk
// MaxSpeed multiplier = jogF x strafe::JogProfile(body-relative stick angle) (F : side : back = 1 : 0.70 : 0.60); 0 = vanilla aim
// speed (strafe off / not Active, walk gait (jogF 0, crouch, no run loops), MM not driving, stale feed). Status: aimJogT= while
// jogF > 0.
float AimJogTarget(float aMoveX, float aMoveY);
std::string StrafeDataDigest();
// ---- B2 strafe S2 chest align (2026-10-13; default on - CET bodyturn.aimChestAlign; off = the S1 pose path bit for bit) ----
// While the strafe mode is Active / Exiting, mm/ChestAlign.hpp rotates the Spine local rotation (only that bone) so the chest
// twist about the vertical axis returns towards the stance's forward-walk orientation (stand / crouch references from
// M_Neutral_Walk_Loop_F / M_Neutral_Crouch_Loop_F, built per database; a database without them: no-op, logged once).
// Leg pipeline StanceGuard -> ChestAlign -> FootLocker; weight = strafe (1) x the previous feed's upper-body mask blend.
// SetChestAlignParams: swayKeep 0..1 (default 0.5: half the sway around the low-passed mean kept), chestMaxDeg 0..90
// (default 60); false = invalid (NaN / out of range), previous kept. Status: chest* fields appended to the strafe segment;
// MMTRACE: " chestDev= chestCorr=" appended at the end only on feeds where it ran with valid diagnostics.
void SetChestAlign(bool aOn);
bool SetChestAlignParams(float aSwayKeep, float aChestMaxDeg);
// chest_repair: longest stand <-> crouch reference blend on a stance switch, 0..5 s (default 0.8; 0 = the old snap that
// yawed the chest by the reference offset ahead of the pose). C++ only (tests / A-B), no native; rejects NaN / out of range.
bool SetChestAlignStanceBlend(float aSeconds);
bool ChestAlignEnabled();
// ---- AIR HOLD stage (2026-10-13; default auto; off = the 1cf5307 path bit for bit) ----
// mm/AirState.hpp phase machine while V is airborne (jump detail 18..21 or vz >= 1.5 off the ground; ledge drop vz <= -1 after
// 0.10 s with TTL >= 0.35 s or a probe miss). Active only while the body is held / the gate is open, TPP on, a usable pose
// and no release reason (PSM release bits, ladder / knockdown details); a release, DB swap or mode change resets it. Hold mode
// (this stage; auto and clips fall back to it - no .airz table yet): while airborne no matcher search (no turn / pivot / reface
// picks), no body-turn input, foot-lock contacts 0, offset root released (no heading lean / root warp), stride rate x holdRate;
// at touchdown the search resumes with a dwell exemption (contacts ramp back over 0.08 s). Strafe reads the previous feed's
// air ownership (AirOwns > Strafe > ground). SetAir: 0 auto, 1 clips, 2 hold, 3 off (false = invalid). SetAirParams: false for
// NaN / out of range (takeoffLeadFrames 0..30, landLead 0..1 s, 0.1 <= rateMin <= rateMax <= 4, recover 0..2 s, holdRate 0..2,
// heavyVz 1..50 m/s), previous kept. SetAirSteer (default off): while the air area owns the body with a valid
// Locomotion target and planar capsule speed >= 0.5 m/s, BodyYawCommand follows the measured velocity heading plus at most
// 35 degrees of stick-requested lead (rate / acceleration limited, <= 200 deg/s); no turn intent or v1 yaw while v2 owns a feed.
// Only while the hold pose keeps the body in the air (effective gate 0: UTPPSetMMHoldPose on, hold eligible); without the hold the
// air area does not own the body and nothing is armed (airSteerFed 0); status airSteerBlocked counts armed feeds whose effective
// gate is still not 0 (expected 0). Status air= fields airSteerFed..airSteerBlocked are appended at the end of the segment.
// AirWantsProbe: the reds
// raycast is wanted (airborne per the last air feed, descending, probe on, every 2nd feed). Status: air= segment; trace: one
// 'UTPP MMAIR' line per phase change.
bool SetAir(int32_t aMode);
bool SetAirParams(int32_t aTakeoffLeadFrames, float aLandLeadSeconds, float aRateMin, float aRateMax, float aRecoverLightSeconds,
                  float aRecoverHeavySeconds, float aHoldRate, float aHeavyVz);
void SetAirProbe(bool aOn);
void SetAirSteer(bool aOn);
// BATCH 6 AIR stage (default on): physics fall episodes (no jump, any non-never-trigger PSM detail, also after a vehicle release)
// and the touchdown guard (no one-frame / rising touchdown, LandApproach abort, no re-take-off 0.15 s after a touchdown). Off = the
// previous AirState machine. Status: air= ... airRobust / airPhysFalls / airTouchRejects / airLandAborts / airRetakeBlocks.
void SetAirRobust(bool aOn);
bool AirWantsProbe();
#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
// Tests: the air area owns the body after the last feed.
bool AirOwnsForTest();
#endif
// Test hook: ChestAlign init + references of the loaded database (ready, Hips / Spine, stand / crouch quaternions).
std::string ChestAlignDigest();
// ---- loco P0 (2026-10-13; default on; off = 0.6.5b) ----
// Derived class data: every table computed from the per-state tag masks (0.5.8 loop bands and loop availability, the crouch
// lateral rule) is rebuilt right after the masks on a database load / switch and in SetExcludeLands, and after
// SetCrouchDirTag / SetStartTurn. 0.6.5b built the loop bands in the matcher init BEFORE the masks:
// empty after the first load, the previous database's tag bits after a switch (startup default -> v6: v6 walk loops never
// banded). On = the active database is recomputed at once; off = the 0.6.5b order from the next load / switch (A/B). The same
// hook will rebuild the strafe profiles, outside this switch. While on: one info line per load / switch ("UTPP MM class data
// [<db>]: recomputed ... loopAvail w/r/s ...") and Status ends with " derivedRecompute=1 derivedRecomputes= derivedStale="
// (stale = loop bands; a changed lateral rule only on the runtime paths). Off: no line, the 0.6.5b Status text.
void SetDerivedRecompute(bool aOn);
// Test hook: digest of the mask-derived and clip-class data (state tag masks, loop bands + availability, crouch lateral rule,
// clip classes, matcher class sets, turn sign / end frames). Independent of the database name: equal text = equal data.
std::string DerivedClassDataDigest();
// Test hook: the class buffer the matcher holds against the buffer the current tables give (ApplyClassSetLocked's build).
struct ClassSetProbe
{
    bool ready = false;            // matcher ready (hashes below valid)
    int set = 0;                   // active class set (0 base, 1 / 2 against a left / right turn)
    int band = 0;                  // active loop band (lo + 4 * hi; 0 = unbanded)
    uint64_t held = 0;             // hash of the matcher's class buffer
    uint64_t expectActive = 0;     // hash of the buffer the current tables give for the active set / band
    uint64_t expectAt = 0;         // ... for aSet / aBand (< 0 = the active one)
};
ClassSetProbe ProbeClassSet(int aSet = -1, int aBand = -1);
// ---- loco C0 (2026-10-13; default on; off = the loco P0 trace / status text) ----
// Feet diagnostics: while the trace is on, one "UTPP MMTRACE feet L=<P|R|F> cL= offL=<cm>cm corrL=<cm>cm wL= slipsL= driftL=
// stepsL= R=... locks= relocks= clamps= settles= events= fl= crouchExcl=" line after every MMTRACE TRANS / tick line (the main
// line is unchanged) and one "UTPP MMTRACE footstep side=L|R reason=drift|radius|clamp|tidy dist=<cm>cm clip= f= loco= |v|="
// line per foot event (drift unlock now; replant steps: loco C1b); Status segment feet="diag= crouchExcl= ... L=... R=...".
// Phase P planted (locked), R releasing (blending out), F free. Off: no feet / footstep lines, no feet segment unless the
// crouch idle exclusion is on.
void SetFeetDiag(bool aOn);
// ---- loco C1a (2026-10-13; default on; off = the loco P0 crouch clip set) ----
// Crouch clip-set exclusion: the crouch clip set leaves out the tags named EXACTLY in the exclusion list (default: empty,
// nothing excluded). Fallback: the full crouch set is kept (log "skipped") when no looping, searchable idle-class clip would
// remain. Applied on the next feed without a playback reset; the derived class data is rebuilt (loco P0 hook). One info line
// per load / switch while on and per setter call: "UTPP MM crouch idle exclusion [<db>] (<why>): ...".
void SetCrouchIdleExclude(bool aOn);
// Comma-separated exact tag names (letters, digits, '_'; spaces around a name ignored; at most 16 names / 512 characters);
// "" = the default list. False = invalid (previous list kept). Names missing in the active database are kept (another
// database may have them) and reported in the status (exclMissing=).
bool SetCrouchExcludeTags(const std::string& aList);
std::string CrouchExcludeTags();
// ---- loco C1b (2026-10-13; default crouch; 0 = the loco C1a build) ----
// Plant / replant in the foot locker (mm/FootLocker.hpp FootReplantConfig) per stance: bit 1 = stand / sprint, bit 2 = crouch
// (0 off, 2 crouch = default, 3 all). In replant mode a planted foot stays planted (no 0.5.9 soft slip, no settle); when its lock
// is more than the unplant radius (7 cm, idle / stop clips 10 cm) from the animated toe, the IK reach was clamped for 0.1 s, or
// (tidy) the capsule stands with a lasting offset > 4 cm, the foot makes one short lifted step (1.0 m/s, idle / stop 2.5 m/s, lift
// <= 3 cm) to the animated toe - one foot at a time. A stance switched off while a foot is planted hands it over smoothly (one
// handover step when its offset is beyond the slip start). Bit 1 (standing replant) is experimental, console only (not a CET
// choice): in run starts / sharp turns the waiting foot is stretched beyond the 15 cm foot-correction guard (offline 20-23 cm).
// False = invalid mask (< 0, > 3). Trace: footstep lines with
// reason=radius|clamp|tidy|handover, feet line phase S + " replant= active= liftL= liftR="; status feet segment replant= ... .
bool SetFootReplant(int32_t aMask);
int32_t FootReplantMask();
// Radii (m, 0.02..0.15: the 15 cm foot-correction guard), relock ratios (0..0.95), step speeds (m/s, 0.2..10), lift (m, 0..0.1);
// the other replant constants keep their defaults. False = invalid / NaN, previous kept. Applies from the next step.
bool SetFootReplantParams(float aRadius, float aStopsRadius, float aRatio, float aStopsRatio, float aStepSpeed, float aStopsStepSpeed,
                          float aLift);
// ---- batch 2 knee fix (2026-10-13; default 7 since batch 5, was 3; 0 = the batch 1b leg pipeline bit for bit) ----
// The right knee "broke" (swung sideways / inward frame to frame) on starts. Bit 1: the StanceGuard knee swivel (forward pole at
// character +Y) gets target 0 while crouching - the applied swivel fades out through its 80 ms filter; bit 2: the foot-lock
// two-bone IK carries the animated knee offset from the animated
// hip-ankle axis onto the new hip-target axis (instead of projecting the animated knee onto the new line: ill-conditioned for an
// almost straight leg with a lateral lock offset, standing turn starts). Crouch replant unchanged. Bit 4 (batch 5): the StanceGuard
// swivel follows its target at 0.3 of its rate while the foot is planted (standing turn starts). False = invalid mask (< 0, > 7).
bool SetKneeFix(int32_t aMask);
int32_t KneeFixMask();
#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
// Tests: matcher output pose of the last feed (7 floats per bone: p xyz, q xyzw local); false = no matcher.
bool MatcherPoseForTest(std::vector<float>& aOut);
#endif
// ---- batch 1b feed dump (2026-10-13; default off = no record, no thread, no file, the loco C1b status text) ----
// On: one line per Feed() call (mm/FeedDump.hpp: raw FeedInput, v1 choice, gates; header "# cols ...") into
// <dir>\utpp-feed-<yyyy-mm-dd-hh-mm-ss>.txt, written by a background thread about every second and on off / Shutdown; the game
// thread only copies a fixed-size record into a preallocated ring (full ring: the line is dropped and counted, one log line per
// session). Info lines on / off; Status gets a feedDump="..." segment once a dump was started. False = could not start (no
// directory set, allocation / thread failure); a file that cannot be created turns the dump off at the next feed (logged).
bool SetFeedDump(bool aOn);
// Folder of the dump files (Main.cpp: <game>\red4ext\logs\, next to the RED4ext UTPP logs). Takes effect at the next start.
void SetFeedDumpDir(const std::wstring& aDir);
// "on= lines= written= dropped= sessions= totalLines= totalDropped= ring= error= file=<path>"
std::string FeedDumpStatus();
#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
// Test hook: wake the dump writer and wait up to aTimeoutMs until every accepted line is written (true = all written).
bool FeedDumpFlushForTest(int aTimeoutMs);
#endif
// Test hook: speed calibration, family rates (incl. a hash of the per-clip factors), capsule profile, band data and speed maps
// of the active database (loco C1a calibration invariance: equal text = equal calibration).
std::string CalibrationDigest();
// Test hook (offline simulations faster than real time): BodyYawCommand integrates the feed dt instead of the real time. simulations faster than real time): BodyYawCommand integrates the feed dt instead of the real time.
void SetCommandDtFromFeed(bool aOn);
// Current speed map of a locomotion state (0..3); false for an invalid state.
bool GetSpeedMap(int32_t aLoco, float aGame[3], float aDb[3], int32_t& aCount);

// Bone map against the armed rig (game thread, from arming). aRig = metaRig pointer used as FK R.
void OnArmed(void* aRig, const std::vector<uint64_t>& aGameHashes, const std::vector<int16_t>& aGameParents);
void OnDisarmed();

void Feed(const FeedInput& aIn);
void Tick();
// Ragdoll native get-up: called by the script natives right after a successful Begin (never from inside it: lock order v1 g_mutex -> v2 mutex).
// One v1 publish at weight 1 so the v2 apply hook writes the get-up pose for the frame(s) before the first get-up feed (no game-pose frame).
// No-op unless the get-up owns the body, an MM pose is available and the last feed was TPP.
void PublishGetUpGap();

void SetEnabled(bool aEnabled);
bool SetTagFilter(const std::string& aSubstring); // "" = all clips; false if no tag matches (previous kept)
void SetBlendTime(float aSeconds);
void SetAllowedStates(uint32_t aMask);
// 0.3.2: fade-out time for hard closures (PSM block, TPP off, stale feed, fault) and the reopen hold after a state
// closure (PSM block / loco state / stale feed). Both 0..5 s; false = invalid, previous kept.
bool SetGateFade(float aHardOutSeconds, float aReopenHoldSeconds);
// Diagnostic trace (game thread): when on, every clip transition and a 1 Hz summary are passed to aLog.
void SetTrace(bool aOn);
// Phase 4.5 contact-based foot lock with two-bone leg IK (default on). Off: locked feet blend out.
void SetFootLock(bool aOn);
// Input-driven trajectory on/off (default on) and the continuing-clip bias (default 0.15, re-inits the matcher).
void SetInputDriven(bool aOn);
void SetTransitionCost(float aCost);
// Input frame: false (default) = body-relative MoveX/MoveY as produced by Locomotion, facing from the measured yaw
// rate; true = camera-relative stick with the input direction as desired facing.
void SetInputCameraRelative(bool aOn);
// Set/append one speed point (game m/s -> database m/s) for a locomotion state (1 stand, 2 crouch, 3 sprint).
void SetSpeedPoint(int32_t aLoco, int32_t aIndex, float aGame, float aDb);
// Speed mapping game -> database ("animation space"). The capsule speed |v| is mapped piecewise-linearly through
// (0,0), (gameWalk, dbWalk), (gameRun, dbRun) and extrapolated with the last slope; the whole matcher then runs in
// animation time scaled by rate = |v| / |v_db| (clamped), so the distance covered per real second equals the game's
// while the chosen clips match the database speeds (stride cadence follows the capsule).
void SetSpeedMap(float aGameWalk, float aGameRun, float aDbWalk, float aDbRun, float aRateMin, float aRateMax);
void SetLogger(void (*aLog)(const char*));
// Simulation clock for the jump-out landing continuation (Tick): monotonic game SIM seconds (they do not advance in pause / menu / photo mode), false = unavailable.
// Default (nullptr): the native ScriptGameInstance.GetSimTime in the plugin build, none in the standalone test builds (-> 0.25 s wall-clock fallback).
// The test hook injects a clock; it is called from Tick() outside the driver mutex.
using SimClockFn = bool (*)(double& aSeconds);
void SetSimClock(SimClockFn aClock);
#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
// Tests: every time the host reads (feed and publish stamps, Tick silences, command dt) becomes aTicks in QueryPerformanceCounter units
// (the real frequency); aTicks < 0 = the real counter again. The test sets it before each step, so results do not depend on the machine load.
void TestSetQpcTicks(int64_t aTicks);
#endif
bool WantsFeed();
// Number of vehicle jump-out launches seen so far (the edge that logs 'JUMPOUT vehicle bit released'). Monotonic, never reset: a caller
// that compares it with a stored value sees every launch, however short the flight state is.
int32_t JumpOutLaunchEpoch();
// Phase 5.1 body yaw from the MM animation. Kill switch (default on); off = Phase 4 behaviour (fixed-time body turn).
void SetBodyTurn(bool aOn);
// Fallback controller: time constant (s), floor / cap of the fallback rate and cap of the total rate (deg/s).
bool SetBodyTurnParams(float aCorrectionTime, float aMinRateDeg, float aMaxCorrectionRateDeg, float aMaxRateDeg);
bool BodyTurnActive();
// Next body yaw (deg) for the body-turn executor; returns aBaseYawDeg unchanged when not active.
float BodyYawCommand(float aCurrentYawDeg, float aBaseYawDeg); // enabled && db loaded (lets the script skip the per-frame work)

// ---- Phase 5.3 (0.3.3) ----
// Offset root (motion style): the visual body follows the animation root motion and springs back to the capsule
// (bounded). Kill switch default on; off = 0.3.2 pose path.
void SetOffsetRoot(bool aOn);
// Translation / rotation halflife (s, 0.02..5), max distance (m, 0..1), max yaw (deg, 0..90). False = invalid.
bool SetOffsetRootParams(float aTranslationHalflife, float aRotationHalflife, float aMaxDistance, float aMaxYawDeg);
// Capsule acceleration matching: kill switch (default on) and per-state target accelerations (m/s^2; 0 = no change,
// otherwise kMinAccelTarget..100). The script lowers the Acceleration stat only (multiplier <= 1).
inline constexpr float kMinAccelTarget = 1.0f;
void SetAccelMatch(bool aOn);
bool SetAccelParams(float aStand, float aCrouch, float aSprint);
// Called by the script after the feed with the vanilla (unmodified) Acceleration stat and the multiplier currently
// applied by UTPP; returns the target acceleration (m/s^2) or 0 = remove the UTPP modifier.
float AccelTarget(float aBaseAccel, float aAppliedMul);

// ---- Phase 5.3b (0.3.4) ----
// Playback rate from the played clip's root speed vs the capsule (default on); off = 0.3.3 loop-speed map rate.
void SetRootRate(bool aOn);
// rateMin/rateMax (0.1..4), epsilon (m/s, 0.05..5), smoothing halflife (s, 0..2). False = invalid, previous kept.
bool SetRootRateParams(float aRateMin, float aRateMax, float aEpsilon, float aHalflife);
// Stop sync (default on): zero desired velocity when the stick is released + trajectory stop half-life (s, 0.02..2).
bool SetStopSync(bool aOn, float aStopHalflife);
// Slow walk (default on): below the first speed-map point the query asks for the walk/crouch loop speed and the rate
// floor is aWalkRateMin (0.1..1, default 0.25). Off = 0.3.3 map and rate floor.
bool SetSlowWalk(bool aOn, float aWalkRateMin);
// Offset root saturation guard: after aSeconds at the clamp the translation halflife becomes aHalflife (0 s = off).
bool SetOffsetSaturation(float aSeconds, float aHalflife);
// Start reface (default on): starting towards a target behind the body asks for a turn / reface start (Locomotion path).
void SetStartReface(bool aOn);
// Exclude landing / from-traversal clip databases from the stand/crouch/sprint sets (default on).
void SetExcludeLands(bool aOn);

// ---- Phase 5.3c (0.3.5) steering stability (all default on; all off = 0.3.4) ----
// Body-turn hysteresis: animation-led turns only for sharp changes, otherwise follow spring + curved trajectory.
void SetSteerFollow(bool aOn);
// enter / exit angle (deg, exit < enter <= 180; defaults 100 / 30), follow time (s, 0.15), max follow rate (deg/s, 360).
bool SetSteerParams(float aEnterDeg, float aExitDeg, float aFollowTime, float aMaxFollowRateDeg);
// Minimum segment dwell (matcher s, 0..2, default 0.3) and relative cost threshold (0..1, default 0.5).
bool SetDwell(bool aOn, float aMinDwell, float aCostRatio);
// Per-search clip-class curation (patterns never, pivots / moving turns / reface / turn in place by situation).
void SetCuration(bool aOn);
// Slow-walk gate from the real capsule speed (requires SetSlowWalk on).
void SetSlowWalkCapsule(bool aOn);

// ---- Phase 5.3c-2 (0.3.6) ----
// Moving-state curation: no stop clips while the stick is held above 1 m/s, no start / speed-transition clips mid-run.
void SetCurationMoving(bool aOn);
// Follow-mode query facing gap (deg, 0..180, default 8; 0 = 0.3.5: the whole target gap in the query).
bool SetSteerGap(float aGapDeg);
// Stance-width guard (default on, 5 cm): each ankle at least aMinHalfWidthCm (0..30) from the pelvis midline.
bool SetStanceGuard(bool aOn, float aMinHalfWidthCm);

// ---- Phase 5.3c-3 (0.3.7), default on; off = 0.3.6 ----
// Animation root velocity turned towards the capsule velocity for the offset root (max angle deg, 0..180, default 60).
bool SetRootWarp(bool aOn, float aMaxDeg);
// Visual body leans (offset yaw, <= 15 deg) towards the capsule travel heading while the stick is held.
void SetHeadingLean(bool aOn);
// Stance guard acts on the swing foot only (push held while the foot is planted); off = 0.3.6 instantaneous push.
void SetStanceSwing(bool aOn);

// ---- Phase 5.3d (0.3.8) ----
// Large turns (default on; off = 0.3.7): speed-class pivots / turns, pivot capsule acceleration, stop prediction fixes.
void SetLargeTurn(bool aOn);
// Capsule acceleration during an animation-led large turn, m/s^2 for stand / crouch / sprint (0 = no limit; 8/8/12).
bool SetPivotAccel(float aStand, float aCrouch, float aSprint);
// Stance guard: ankle half width (cm), swing-ankle separation (cm), knee-keeping IK (0.3.8 defaults 3 / 6 / true;
// 0.3.7 = 5 / 10 / false).
bool SetStanceParams(float aMinHalfWidthCm, float aMinSeparationCm, bool aKneeIk);

// ---- Phase 5.3e (0.3.9), default on; off = 0.3.8 ----
// No start clips in a steady slow walk (slow-walk gate) after the start window.
void SetSlowWalkLoop(bool aOn);
// Stance guard anti-jitter filter time (s, 0..1; default 0.08; 0 = 0.3.8 path: knee IK, rate-limited push, raw swivel).
bool SetStanceSmooth(float aSmoothTime);

std::string Status();
int32_t LastPsmDetail(); // PSM LocomotionDetailed of the last feed (-1 = not reported); diagnostics
void Shutdown();

// UTPP-MOTION SEAM: optional motion v2 driver hooks (MotionDriver.hpp). Null (default) = v1 only, nothing below is called.
// UTPP-MOTION SEAM step: in Feed right before the v1 matcher update (raw input, final g_effGate, dt; once per feed: after the matcher
// UTPP-MOTION SEAM block when that block does not run); true + frame -> v1 search skipped, leg tail on the v2 pose; apply: in PublishLocked before
// UTPP-MOTION SEAM the publish (true = it replaced pose + bone map); armed / disarmed: rig hashes + parents. All under g_mutex.
// UTPP-MOTION SEAM v2 frame for the leg tail (MMDriver Feed): pose in the v1 bone layout, CONT contacts, root motion (real time).
struct MotionSetFrame // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM buf / bufCap: caller-owned pose buffer (fixed, no per-frame allocation); frame copies the v2 pose into it under
    // UTPP-MOTION SEAM the v2 lock and sets pose = buf, so a worker bundle swap / free after frame returned cannot dangle (MotionDriver.hpp Frame)
    mm::BoneTransform* buf = nullptr; // UTPP-MOTION SEAM
    uint32_t bufCap = 0; // UTPP-MOTION SEAM bones
    const mm::BoneTransform* pose = nullptr; // UTPP-MOTION SEAM = buf after a successful frame
    uint32_t numBones = 0; // UTPP-MOTION SEAM
    float contact[2] = {0.0f, 0.0f}; // UTPP-MOTION SEAM left / right, v2 database CONT of the playing frame
    float rootVel[2] = {0.0f, 0.0f}; // UTPP-MOTION SEAM m/s, character frame (ROOT dx / dy x fps)
    float rootYawRate = 0.0f; // UTPP-MOTION SEAM rad/s (ROOT dyaw x fps)
    int startTurnDirection = 0; // current MOTION ordinary reface start, never legacy v1 state
    int cancelStartSupport = -1; // short cancelled-start support: -1 none, 0 left, 1 right
    uint8_t clipClass = 0; // actual current clip class; idle databases can also contain turns
    bool stops = false; // UTPP-MOTION SEAM idle / stop database (replant Stops profile)
    bool travFrame = false;                  // UTPP-MOTION SEAM trav: traversal montage frame (root already
                                             // warped onto the capsule: offset root gets animVel = capVel)
    bool dodgeClip = false; // a ground dodge clip is shown (mm/Dodge.hpp): the body turn holds the capsule yaw (BodyYawCommand)
    // Prepared from the same live base as pose/AO. Host applies once, bypassing
    // legacy integration.
    bool providedRoot = false;
    uint64_t rootFeedSequence = 0;
    float rootOffsetX = 0, rootOffsetY = 0, rootOffsetYaw = 0;
}; // UTPP-MOTION SEAM
struct MotionHooks // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    bool (*step)(const FeedInput& aRaw, uint32_t aEffGate, float aDt) = nullptr; // UTPP-MOTION SEAM
    bool (*apply)(mm::integ::PoseSnapshot& aSnap) = nullptr; // UTPP-MOTION SEAM
    void (*armed)(uint64_t aRig, const std::vector<uint64_t>& aGameHashes, const std::vector<int16_t>& aGameParents) = nullptr; // UTPP-MOTION SEAM
    void (*disarmed)() = nullptr; // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM commit A (6.10.1 item 3): after step returned true, the v2 frame for the shared leg tail. aBaseGameIndex /
    // UTPP-MOTION SEAM aBaseBones = v1 bone map; false = layout differs from v1 (v2 yields this feed, reason logged once).
    bool (*frame)(const int32_t* aBaseGameIndex, uint32_t aBaseBones, MotionSetFrame& aOut) = nullptr; // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM climb release: motiondriver::OwnsTraversal (v2 plays the current Climb 8 / Vault 9 traversal); null = false
    bool (*ownsTraversal)() = nullptr; // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM slide feed: motiondriver::OwnsSlide(PSM LocomotionDetailed) - v2 owns the slide (mm.slide on + a slide bundle): the
    // v1 body-yaw command stays out of it. null / false = the pre-slide behaviour.
    bool (*ownsSlide)(int32_t aPsmDetail) = nullptr; // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM TRAV-1: motiondriver::TravActive (v2 enabled and the bundle has traversal data; lock-free). True: a Climb 8 /
    // Vault 9 that v2 does not own releases to the game animation even with mm.climbRelease off. null = false. Keep LAST.
    bool (*travActive)() = nullptr; // UTPP-MOTION SEAM
    // Vault torso limiter (mm/VaultTorso.hpp): motiondriver::VehicleVaultWeapon - the last traversal event is a vault over a vehicle that v2
    // rejected (Unsupported(vehicle)) and the overlay is a ranged pose. Lock-free (atomics only: v1 calls it under its own mutex). null = false.
    // Keep LAST.
    bool (*vehicleVaultWeapon)() = nullptr; // UTPP-MOTION SEAM
}; // UTPP-MOTION SEAM
void SetMotionHooks(const MotionHooks* aHooks); // UTPP-MOTION SEAM (null = uninstall)
// UTPP-MOTION SEAM test hook (motion_adapter integration test): v1 matcher / steering state around a v2-owned span.
struct MotionSeamProbe // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    bool ready = false; // UTPP-MOTION SEAM v1 matcher ready (fields below valid)
    int32_t clip = -1, frame = -1; // UTPP-MOTION SEAM v1 matcher playing clip / global frame
    uint64_t updates = 0, searches = 0; // UTPP-MOTION SEAM v1 matcher counters
    bool searched = false; // UTPP-MOTION SEAM the last v1 update searched
    float maxOffsetAngle = 0.0f, maxOffsetDist = 0.0f; // UTPP-MOTION SEAM v1 inertialization offset after the last update
    int bindSign = 0; // UTPP-MOTION SEAM 0.5.8 start bind sign (0 = released)
    int bodyFollowDir = 0; // UTPP-MOTION SEAM 0.5.7 long-way start turn direction (0 = none)
    bool motionOwned = false; // UTPP-MOTION SEAM v2 owned a feed since the last v1 matcher update
    uint64_t handbacks = 0; // UTPP-MOTION SEAM fresh v1 searches after a v2-owned span
    bool turnCommitted = false, settling = false; // UTPP-MOTION SEAM 0.5.7 turn commit / 0.5.8 settle window on the last feed
    bool stanceUsed = false; // UTPP-MOTION SEAM 0.5.7 a stance transition was selected since the last stance change
    // UTPP-MOTION SEAM batch 2 (1cf5307): leg context of the last feed and the strafe / chest align state around a v2 span
    bool legMotionSet = false; // UTPP-MOTION SEAM the last leg context came from a v2 frame
    int32_t legStance = 0; // UTPP-MOTION SEAM LegContext::stance of the last feed (1 stand, 2 crouch, 3 sprint)
    bool swivelSuppressed = false; // UTPP-MOTION SEAM StanceGuard crouch swivel gate (knee fix bit 1) after the last feed
    bool strafeOn = false; // UTPP-MOTION SEAM B2 strafe mode Active / Exiting after the last feed
    uint64_t strafeExits = 0; // UTPP-MOTION SEAM strafe mode exits (counter)
    bool chestOut = false; // UTPP-MOTION SEAM B2 ChestAlign ran in the last leg pipeline
    // UTPP-MOTION SEAM x AIR (merge b4): air fields of the last leg context, the air area's decision and phase on the last feed
    bool legAirborne = false; // UTPP-MOTION SEAM LegContext::airborne
    float legContactScale = 1.0f; // UTPP-MOTION SEAM LegContext::contactScale
    bool airOwns = false; // UTPP-MOTION SEAM AirDecision::airOwns
    int airPhase = 0; // UTPP-MOTION SEAM air::Phase (0 Ground)
    uint32_t airResets = 0; // UTPP-MOTION SEAM AirStats::resets
    uint64_t airPlays = 0; // UTPP-MOTION SEAM v1 air clips forced (ForcePlay) so far
    uint64_t airQueryOverrides = 0; // UTPP-MOTION SEAM x AIR (merge repair): query bone frame overrides latched so far
    int32_t airQueryFrame = -1; // UTPP-MOTION SEAM the last latched query bone frame (-1 none yet)
    bool frameRootZ = false; // UTPP-MOTION SEAM the v1 matcher's current frame carries root_z (dirty bone features)
    bool airTableValid = false; // UTPP-MOTION SEAM a valid .airz for the loaded v1 database (Clips mode possible)
}; // UTPP-MOTION SEAM
MotionSeamProbe ProbeMotionSeam(); // UTPP-MOTION SEAM
} // namespace utpp::motionhost

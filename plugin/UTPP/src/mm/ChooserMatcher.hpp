// UTPP motion matching runtime - motion-style pose selection on a .utppmm VERSION 2 database. Header-only, C++20.
//
// Model: UE 5.8 AnimNode_MotionMatching / UPoseSearchLibrary::UpdateMotionMatchingState as configured by motion:
//   * databases to search = chooser table (data file next to the database, <name>.chooser.json) evaluated in
//     AllResults mode every update (SetDatabasesToSearch(EvaluateChooser(...), Get_MMInterruptMode())).
//   * query per searched database built from the database's schema exactly as the baker defines the raw features
//     (Source/tools/mmbake/FORMAT.txt VERSION 2 "Feature values"), then normalized with that database's NRMV
//     (q = (raw - mean) * scale). Trajectory channels (SCHC flag TRAJECTORY) come from the caller's trajectory;
//     pose channels come from the continuing pose (input query 0, same schema) else from the caller's pose history,
//     else they use the last indexed pose when the current clip exits its search range without invalidation.
//     This one-sample fallback preserves phase when no caller history exists; it is not a full pose history.
//     Initial/invalidated queries still use the mean (neutral, 0 after normalization).
//   * continuing pose first: feature cost + cost addend + continuing addend (continuing-pose cost bias), then every
//     chooser database with FORMAT.txt v2 search (AABB, BLOCK rows never candidates), excluding rows selected or
//     played within PoseReselectHistory (0.3 s) and, in the continuing database, every row of a
//     DISABLE_RESELECTION entry. Lowest total cost wins; a non-continuing winner is a jump = blend request.
//   * interrupt mode (Get_MMInterruptMode): InterruptOnDatabaseChange when MovementMode changed, or on the ground
//     when MovementState, Gait or Stance changed; else DoNotInterrupt. Continuing pose exactly as UE 5.8
//     UPoseSearchLibrary::MotionMatch: looked up in the playing database and in every chooser database indexing the
//     same clip; DoNotInterrupt keeps evaluating it even when its database is no longer chosen;
//     InterruptOnDatabaseChange drops it from the search then but still builds the query from it; the
//     *AndInvalidateContinuingPose modes also drop it from the query; ForceInterrupt never evaluates it.
//   * blend time (Get_MMBlendTime): ground->ground 0.5 s, landing 0.2 s, air with Vz > 100 cm/s 0.15 s, else
//     0.5 s; chooser data-file mm_rules override these when present.
//   * PoseJumpThresholdTime (motion: 0 = off): a winner in the continuing clip within this time of the continuing
//     time is not a jump.
// Allocates only in Init(); Update() is allocation-free. Not used by the runtime yet (MMDriver / Main untouched).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../MutationSelect.hpp"
#include "Chooser.hpp"
#include "CancelledStart.hpp"
#include "MotionDatabase.hpp"
#include "MMMath.hpp"
#include "MotionPlayRate.hpp"
#include "Steering.hpp"

// Offline mutation switch of the fix/slide-exit changes (tests build a mutated copy of SlideExitTest with -DUTPP_SLIDEEXIT_MUTATION=k;
// 0 / undefined = production code): 1 the Dodge keeps the FILTERED stance (pending change dropped), 2 the stance filter restarts its
// timer on every old-value feed, 3 a slide counts as settled by the clip time of the playing clip, 4 the exit hold ignores stance /
// gait / movementState changes and aim / shot turns, 5 the sprint gait of CrouchSprint is not tied to the filtered stance, 6 the exit hold yields to a one-feed gait / movementState change, 7 the aim
// release is level based (stationaryAimTurn alone), 8 the hold ignores pivot / spin / reface starts, 9 the first feed after a slide
// fixes the stance for the filter time, 10 no long-slide rule: a slide is settled by a late clip phase alone (>= 1.2 s Into / Loop),
// 11 slideExitYieldS = 0 releases the hold on the first feed even without a gait / movementState difference, 12 a flag that dropped during
// the hold is not re-armed (the next start of it is masked), 13 ChooserState::stanceRaw carries the filtered stance.
#ifndef UTPP_SLIDEEXIT_MUTATION
#define UTPP_SLIDEEXIT_MUTATION 0
#endif

namespace utpp::mm::motion
{
// One trajectory sample (caller space: metres, +Z up, yaw in radians, character forward = +Y rotated by yaw
// counter-clockwise about +Z). t: seconds relative to now (past < 0, future > 0). Samples sorted by t.
struct TrajSample
{
    float t = 0.0f;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float yaw = 0.0f;
};

// Pose history (0.3 s in motion): per sample the root (x, y, z, yaw as TrajSample) and the Root-space (model space
// without root motion) position + rotation of the tracked game bones. Samples sorted by t (<= 0).
inline constexpr uint32_t kHistMaxSamples = 24;
inline constexpr uint32_t kHistMaxBones = 8;
struct PoseHistory
{
    uint32_t numSamples = 0;
    uint32_t numBones = 0;
    int32_t bone[kHistMaxBones] = {};
    TrajSample root[kHistMaxSamples];
    math::Vec3 gp[kHistMaxSamples][kHistMaxBones];
    math::Quat gq[kHistMaxSamples][kHistMaxBones];
};

// motion chooser inputs (report 3.4 / 5.3). Enum values are the E_* values of the chooser data file.
inline constexpr int kMovementSliding = 2; // E_MovementMode Sliding (motion NewEnumerator6; chooser slide section)
inline constexpr int kDirectionForward = 0, kDirectionOther = 1;

struct ChooserState
{
    int gait = 0;          // E_Gait 0 Walk, 1 Run, 2 Sprint
    int stance = 0;        // E_Stance 0 Stand, 1 Crouch
    int movementState = 0; // E_MovementState 0 Idle, 1 Moving
    int movementMode = 0;  // E_MovementMode 0 OnGround, 1 InAir, 2 Sliding (only with mm.slide and a slide bundle)
    float speed2dCmS = 0.0f;
    float timeToLand = 1.5f; // s (no landing predicted = last trajectory sample time)
    bool isStarting = false;
    bool isPivoting = false;
    bool justLandedLight = false;
    bool justLandedHeavy = false;
    bool justTraversed = false;
    bool shouldTurnInPlace = false;
    bool shouldSpinTransition = false;
    int databaseLod = 0; // 0 Dense, 1 Sparse
    float verticalVelocityCmS = 0.0f; // for the blend time (air rising)
    bool stationaryShotTurn = false; // adapter intent, not an authored chooser input
    bool stationaryAimTurn = false; // live, grounded stationary ADS/RMB intent; no shot lease
    // Rest turn-in-place of a hidden capsule-vs-root offset (MotionAdapter restTurn): finishes an admitted same-side clip like the aim turn
    // (it shares stationaryAimYawDeg / stationaryAimTargetYawDeg) but is NOT an aim start: HoldFlags ignores it, so it never releases
    // the hold of a slide-exit clip.
    bool restTurn = false;
    bool explicitWalk = false; // actual walk toggle, distinct from idle's inferred Walk gait
    float shotTurnYawDeg = 0;
    float stationaryAimYawDeg = 0, stationaryAimTargetYawDeg = 0;
    // Ordinary reface-start admission shared with the actor controller. The caller
    // owns input/goal lifetime; 0 disables the policy, +1 is CCW/left, -1 CW/right.
    int startTurnDirection = 0;
    int32_t committedStartTurnClip = -1;
    // Slide (motion, sprint -> crouch). FEED fields written by adapter::MapToChooser (the matcher / chooser side that reads them
    // is owned by the motion session; chooser.json "slide" section: MovementMode value sliding_value, inputs MovementDirection
    // and MovementMode_LastFrame). Defaults = no slide: a feed that never slides leaves movementModeLast = the previous frame's
    // movementMode (0 / 1) and movementDirection = 0, which no chooser without the slide section reads.
    int movementModeLast = 0;  // movementMode of the previous frame (chooser MovementMode_LastFrame); Sliding on the first exit frame
    int movementDirection = 0; // E_MovementDirection 0 Forward, 1 Other (B / LR / LL / RL / RR collapsed); only read by the Sliding rows
    // fix/slide-exit: seconds the slide that ended on THIS (first exit) frame lasted, 0 on every other frame (MapToChooser). A hand-built
    // state leaves it 0 = unknown: the slide exit fixes then stay off for it (MatcherConfig::slideExitSettledS).
    float slideLastDurationS = 0.0f;
    // fix/slide-exit: the stance of the game PSM of this feed, NOT debounced (0 Stand, 1 Crouch = adapter::PsmCrouchStance(LocomotionDetailed);
    // Slide 5 / Dodge 7 / CrouchSprint 30 / air read as Stand like in the PSM). `stance` is the debounced value (MapConfig::stanceFilterS,
    // up to 0.15 s late, Dodge kept, 1 on slide frames); latching consumers that classify the stance on an edge (reload CorrectionEpisode)
    // read this one. A hand-built state leaves it 0.
    int stanceRaw = 0;
    // fix5/jump: the game PSM of this feed is a jump detail 18-21 (integ::PsmIsJumpDetail: take-off, double jump, ...; a walk-off a ledge
    // has none). Written by adapter::MapToChooser; a hand-built state leaves it false (MatcherConfig::jumpRiseNoOff then never acts).
    bool psmJump = false;
    // fix/jumpout-cam: the flight of a vehicle jump-out (double F while driving: MMDriver exit window + JumpOutLatch / the exit edge). Written by
    // adapter::MapToChooser from MapContext::jumpOutFlight; a hand-built state leaves it false (MatcherConfig::jumpOutNoBack /
    // jumpOutFallHandover then never act). Every other flight is untouched.
    bool jumpOutFlight = false;
    // r3 landing: the heavy land that followed a vehicle jump-out flight has played motiontraj::kJumpOutLandRecoverS seconds (written by
    // adapter::MapToChooser; a hand-built state leaves it false = the land clip plays on as before). InterruptFor then lets the search leave
    // the land database.
    bool jumpOutLandOver = false;
    // The game's own Dodge (PSM detail 7, on the ground): an impulse of up to 11 m/s that decays within ~0.25 s and is no motion this database
    // represents. Written by adapter::MapToChooser; MatcherConfig::dodgeHold keeps the playing clip through it.
    bool dodge = false;
    // Digital heading step (MatcherConfig::headingCatchUpS): seconds since the wanted run direction jumped by at least
    // MapConfig::headingStepDeg within one feed (a direction key pressed or released while moving, OrientToMovement on the ground;
    // a large value = no step), and the absolute angle (deg) between that direction and the rendered actor facing. Written by
    // adapter::MapToChooser; a hand-built state leaves the defaults (the catch-up rule never acts).
    float headingStepAgeS = 1.0e9f;
    float headingLagDeg = 0.0f;
    // The step was a start from rest toward a direction away from the facing (the rule then acts only while a start clip plays).
    bool headingStepFromRest = false;
};

inline cancelstart::State CancellationState(const ChooserState& state)
{
    return {state.movementMode, state.movementState, state.stance, state.gait, state.speed2dCmS, state.isStarting,
        state.stationaryShotTurn || state.stationaryAimTurn || state.justLandedLight || state.justLandedHeavy || state.justTraversed ||
        state.shouldTurnInPlace || state.shouldSpinTransition || state.isPivoting || state.startTurnDirection != 0,
        state.explicitWalk};
}

enum class Interrupt : uint8_t
{
    DoNotInterrupt,
    InterruptOnDatabaseChange,
    InterruptOnDatabaseChangeAndInvalidateContinuingPose,
    ForceInterrupt,
    ForceInterruptAndInvalidateContinuingPose
};

// motionfix item 5 (standing jump). [P] 0.5 m/s = the standing bound of the task (in-game standing jumps: capsule Speed2D
// 0.00-0.04 m/s, moving jumps >= 2.6 m/s, feed dump 2026-10-13). Default bias 0 = off, MEASURED on the real standing jumps
// (MotionAdapterCompileTest 'motionfix air', the motionfix progress log item 5): without a bias every airborne post-apex
// update already plays the motion stand takeoff F_Start_Stand_L/Rfoot (motion has no in-place jump clip); M_Neutral_Jump_Loop_Fall
// is a terminal fall (root Vz -9.8 m/s constant): an addend of -0.02 .. -0.2 never selects it, -0.3 .. -1.0 only for
// the last 1-5 updates before touchdown, alternating with the stand clip on consecutive updates (pops).
inline constexpr float kAirFallMaxSpeedCmS = 50.0f;
inline constexpr float kAirFallBiasDefault = 0.0f;
// motion clip names (posesearch.json PSD_Dense_Jumps, 21 entries: 0-15 B / F (Stand, Walk, Run) / LL / RL
// Start and F Off clips, 16 M_Neutral_Jump_Loop_Fall (looping, the only neutral fall pose), 17-20 Land clips disabled).
inline constexpr const char* kAirFallClipNames[] = {"M_Neutral_Jump_Loop_Fall"};

// fix3/jump (P6 K2, recording 2026-10-21 12:15, feeds tests/data/jump_feeds_20261021_1213.txt). A STANDING jump (no stick input, capsule
// speed ~0: in-game 0.00-0.04 m/s) has no clip in the motion database either, and the row costs of PSD_Dense_Jumps then picked
//  - M_Neutral_Jump_B_Start_L/Rfoot (a jump BACKWARDS: running-like knee-to-hip pose, legs swapped in the descent, hips turned ~46 deg),
//    9 of 10 standing jumps of the recording, and
//  - the take-off crouch of M_Neutral_Jump_F_Start_Stand_Lfoot (clip time 0.10-0.13, hips low) 0.04-0.11 s before touchdown, on every one.
// kStandingJumpMaxSpeedCmS = 0.3 m/s of the task (chooser speed in motion cm/s, 0 = the rules are off).
inline constexpr float kStandingJumpMaxSpeedCmS = 30.0f;
inline constexpr float kStandingJumpLandBlockS = 0.15f;
// The first InAir update of a jump (PSM Jump while the capsule has not left the ground: vz 0, TimeToLand 1.52) picked the run-off clip
// M_Neutral_Jump_F_Off_Walk_Rfoot at 1.10 s (an airborne pose, 0.5 s fade) on every standing jump, one frame before the take-off row; the F Off
// clips are out of a standing jump's search for its first kStandingJumpOffBlockS seconds.
inline constexpr float kStandingJumpOffBlockS = 0.2f;
// A new take-off in the air (double jump PSM 19, hover 21, air dash 22, thrusters: the movement mode stays InAir) is an impulse of the vertical
// velocity (jump ~ +566 cm/s, gravity takes 27 cm/s per 60 Hz update) or of the horizontal speed within one update; the standing-jump decision
// of the flight is taken again there.
inline constexpr float kStandingJumpRejumpDvzCmS = 150.0f;
inline constexpr float kStandingJumpRejumpDSpeedCmS = 150.0f;
// Clip classes of PSD_Dense_Jumps by name: 1 = B / LL / RL Start (jumps to the back / side), 2 = F Start (take-off phase = rows below
// kJumpStartPhaseEndS, the unblocked rows of the Start clips end at 0.33-0.53 s), 3 = F Off (rows below kJumpOffPhaseEndS are the
// run-off entry row, the later rows are the airborne phase).
inline constexpr float kJumpStartPhaseEndS = 0.6f;
inline constexpr float kJumpOffPhaseEndS = 0.2f;
// fix3/jump N3: cross-fade times of the quick slide combinations (MatcherConfig::blendSlideAir / blendTipAfterSlide / blendSlideFamily). The
// value is where the peak pose change of the recorded transitions falls to the rate of the sprint legs themselves (~40 deg per 50 ms):
// slide -> jump 86 -> 44 deg (Hips 35 -> 17 cm) per 50 ms at 0.3 s, turn-in-place out of the rising exit clip 96 -> 32 at 0.3 s,
// FootOut <-> KneesOut at 0.25 s with the leg bones faded over the whole blend time (BlendRequest::fullLegFade) 45 (Female V) / 63 (Male V),
// 63 / 77 with the shorter leg fade (tests/JumpPickTest.cpp, replay of tests/data/jump_feeds_20261021_1213.txt). The family fade is
// capped by the slide EXIT (tests/SlideExitTest.cpp): the exit clip is matched against the family on display, so a longer mix of two
// families that is still running at the exit pops it (0.3 s: Hips / Spine3 31 deg per 0.1 s of the synthetic KneesOut slides, bound 25).
inline constexpr float kBlendSlideAirS = 0.3f;
inline constexpr float kSlideAirWindowS = 0.4f;
inline constexpr float kBlendTipAfterSlideS = 0.3f;
// ... and only while the exit clip really is still rising: the Hips of the turn-in-place row are more than kTipRiseGapM above the Hips of the
// exit clip on display (299.58: 19 cm lying -> 55 cm crouching = 35 cm). Review: the first gate, the age of the exit clip (hold + 0.2 s), also
// covered a rest turn-in-place after the hold, when the clip had stood up and the long fade only cut the authored yaw of the turn.
inline constexpr float kTipRiseGapM = 0.15f;
inline constexpr float kBlendSlideFamilyS = 0.25f;
#ifndef UTPP_JUMPPICK_MUTATION
#define UTPP_JUMPPICK_MUTATION 0
#endif

struct MatcherConfig
{
    // motion MotionMatching node AllowedPlayRate (ordinary, non-event searches).
    float minPlayRate = 0.85f;
    float maxPlayRate = 1.15f;
    float poseReselectHistory = 0.3f;    // s
    float poseJumpThresholdTime = 0.0f;  // s, 0 = off (motion)
    float blendGroundToGround = 0.5f;
    float blendLanding = 0.2f;
    float blendSlide = 0.2f; // motion: every slide state cross-fade is 0.2 s (ABP State Controller); entry and exit of Sliding
    // Slide exit (fix/slide-exit, 2026-10-21; replay tests/SlideExitTest.cpp, game feeds 2026-10-13 .. 10-21). Defaults = the fix, the
    // pre-fix behaviour is false / 0. Measured on the 10 ground exits of the 03:37 / 10:11 sessions: the exit clip (PSD_Slide_ExitTo*,
    // schema != the slide schema) was entered at clip time 0.3-0.53 s because its pose channels (Hips / foot) fell back to the
    // database mean when no pose history exists, so the 0.2 s crossfade from the lying slide pose to a half risen pose ran
    // 60-80 deg / 0.1 s on the hips (the adapter has no pose history, so the pose channels of an exit database, which has its own
    // schema, fell back to the database mean and the trajectory part alone chose the row); 5 of those 10 exits were dropped again after 0.07-0.10 s (stance 3 <-> CrouchSprint 30 flicker,
    // a closer Crouch_Stop row) and one was held 3.47 s.
    // slideExitMinHoldS: the exit clip, once selected, is kept for this long (seconds, from the selection) while the character
    // stays on the ground and nothing releases it: a stance / gait / movementState change that reached the chooser (the stance is
    // already debounced by MapConfig::stanceFilterS), the start of an aim / shot turn, a forced interrupt, a new slide, air. The
    // released update searches normally with the interrupt of that change (not swallowed). 0 = off.
    float slideExitMinHoldS = 0.6f;
    // slideExitYieldS: a gait / movementState change (not debounced anywhere: the PSM flickers Stand 1 / Crouch 3 -> Sprint 4 on the
    // first feeds after a slide, movementState follows the decaying speed) releases the hold only when it has lasted this long, or is
    // still there when the hold ends (the edge is then delivered at that update). Stance (debounced by MapConfig::stanceFilterS),
    // an aim / shot turn start, a pivot / spin / reface start, air and a forced interrupt release at once. 0 = at once.
    float slideExitYieldS = 0.25f;
    // slideExitFamily: an exit database holds a FootOut_Out_* and a KneesOut_Out_* clip per state; the slide pose of the other family
    // is 60 deg (Hips) / 45 deg (Spine3) away from the first frame (vs 5 / 1 deg for the own family), but the pose channels weigh only
    // ~0.1 against the trajectory part, so a KneesOut exit followed a FootOut slide. Only the clips of the slide's own family are
    // searched on the first frame after the slide.
    bool slideExitFamily = true;
    // slideExitMaxEntryS: on the first frame after a settled slide only the rows of an exit clip up to this clip time are searched.
    // Every exit clip starts lying (hips 17 cm) and has risen by ~0.3 s, so a later row cannot continue the displayed slide pose; the
    // trajectory features still preferred one (slide cancelled at 9 m/s: entered at 0.50 s, 81 deg per 0.1 s; KneesOut exits 0.27-0.37 s;
    // Idle exits 0.2-0.3 s). 0.07 s = the first three rows (30 fps); the pose channels cannot add to that. 0 = off.
    float slideExitMaxEntryS = 0.07f;
    // slideExitBiasS: the cost addends of the exit rows (costAddend -0.1 / -0.3, continuing bias -0.5) keep an exit clip alive; once the
    // rise is over they hold the crouch-walk tail of the clip against the ordinary locomotion databases (replayed game slides: moving
    // exit clips held 2.1-2.2 s, 3.47 s with the old mid-clip entry). After this many seconds from the selection the exit clip competes
    // on its feature cost alone. 0 = the addends stay for the whole clip.
    float slideExitBiasS = 1.0f;
    // slideExitMinPlayRate: lower play rate bound of an exit clip (0 = the ordinary minPlayRate 0.85). The query speed after the slide
    // (2.7 m/s) is half the root speed of the exit clips (5.3-6.1 m/s, wanted rate ~0.5), so the rate clamps at 0.85 and the rise of
    // the body (authored Spine3 up to 27 deg per 0.1 s for the Crouch clips, 36 for the KneesOut ones, 35 for Run) runs faster than the
    // ground; 0.55 keeps Hips / Spine3 of all Crouch, Idle and KneesOut exits of the replayed game slides below 25 deg per 0.1 s (the
    // Run exits keep 28-30 deg Spine3: their wanted rate is ~0.73, the peak is authored).
    float slideExitMinPlayRate = 0.55f;
    // blendSlideExit: cross-fade of the slide -> exit transition (< 0 = blendSlide). The matched exit clip starts within 5 deg of the
    // slide pose, so a long cross-fade only adds rate (the outgoing pose stays while the incoming rises): 0.2 s -> Spine3 30 deg per
    // 0.1 s, 0.12 s -> 23 deg. A slide that never settled (duration < slideExitSettledS) keeps blendSlide.
    float blendSlideExit = 0.12f;
    // slideExitSettledS / slideExitSettledPhaseS / slideExitSettledLongS: slideExitMaxEntryS / slideExitFamily / blendSlideExit act only after a SETTLED slide, i.e.
    // the displayed pose is the lying slide pose: the slide lasted slideExitSettledS (ChooserState::slideLastDurationS: the 0.2 s entry
    // cross-fade plus a margin; a 0.1 s tap still shows the fade from the sprint pose) AND the playing slide clip is past its lowering
    // (an Into clip at slideExitSettledPhaseS, hips on the ground at 1.0 s; the Loop clips at any time: they are not flagged as loops
    // and a long slide re-jumps to Into at 0.93 / 1.03 s every ~1 s, so the clip time alone is no measure of the slide). A slide
    // cancelled while it is still lowering (Into at 0.5 s) has no row of an exit clip that continues its pose: pre-fix behaviour.
    // A slide of slideExitSettledLongS has lowered whatever clip phase the last re-jump left (Into 0.93 / 1.03 s every ~1 s of a long slide).
    float slideExitSettledS = 0.25f;
    float slideExitSettledPhaseS = 0.95f;
    float slideExitSettledLongS = 1.0f;
    float blendAirRising = 0.15f;
    float blendAirOther = 0.5f;
    float airRisingThresholdCmS = 100.0f;
    bool useAabb = true;
    // motionfix item 5 (standing jump): after the apex (Vz <= 0) of an InAir update with Speed2D < airFallMaxSpeedCmS,
    // the rows of the motion fall clip (kAirFallClipNames, PSD_Dense_Jumps entry 16) get this cost addend (< 0 favours them;
    // 0 = off, the default). Same mechanism as the baked per-row cost addends (looping_cost_bias -0.005 of
    // M_Neutral_Jump_Loop_Fall in PSD_Dense_Jumps), applied at runtime because it depends on the feed.
    float airFallBias = kAirFallBiasDefault;
    float airFallMaxSpeedCmS = kAirFallMaxSpeedCmS;
    // fix3/jump (P6 K2): standing jump = InAir, no stick input (movementState Idle) and a chooser speed below standingJumpMaxSpeedCmS
    // on the FIRST InAir update of the flight (0 = the standing-jump rules are off); the decision is kept for the whole flight (steering in
    // the air used to switch the landing block off and the take-off row picked then played to the touchdown) and taken again at every new
    // take-off in the air (an impulse of vz or of the speed, kStandingJumpRejumpDvzCmS): a directional double jump / air dash after a
    // standing jump may use the B / LL / RL Start rows. Such an update never selects a B / LL / RL Start row (jumps backwards / sideways: the motion
    // stand takeoff F_Start_Stand_L/Rfoot is the only forward-neutral take-off left in the database), and standingJumpLandBlockS
    // seconds before touchdown (TimeToLand below it) it selects no take-off-phase row (Start rows below 0.6 s, F Off rows below 0.2 s):
    // the character does not crouch to jump again in the last 0.1 s of the descent. 0 = no landing block.
    float standingJumpMaxSpeedCmS = kStandingJumpMaxSpeedCmS;
    float standingJumpLandBlockS = kStandingJumpLandBlockS;
    bool standingJumpExcludeBack = true; // false = the B / LL / RL Start rows stay selectable in a standing jump (the land block still acts)
    float standingJumpOffBlockS = kStandingJumpOffBlockS; // seconds from the first InAir update without F Off rows (0 = off)
    // fix5/jump (recording 2026-10-25 13:41, tests/data/jump_feeds_20261025_1341.txt): a jump in motion, i.e. a flight whose updates carry a
    // PSM jump detail (ChooserState::psmJump) and that is no standing one, takes no F Off row (the run-off-a-ledge clips: arms hanging, no
    // push-off) and does not continue one from the first airborne update (the second InAir update) while it rises (vz > 0). The first InAir
    // update is the take-off feed (the capsule still on the ground, no floor yet: TimeToLand 1.52, Jumps_Far chooseable too); it keeps its
    // pick (F_Off_Run at 0.6 s, an airborne pose, for one update; without it the 1-update pick is Jumps_Far F_Start_Sprint at 0.0 s, the
    // crouch, and the slide -> jump pose pops). The next update has the J2 floor (TimeToLand ~0.7 s, PSD_Dense_Jumps), where the continuing
    // F_Off_Run competed with F_Start_Run: 8 of 20 running jumps of the recording played F_Off_Run the whole flight. A walk-off (no jump
    // detail) keeps the F Off rows. false = off.
    bool jumpRiseNoOff = true;
    // airKeepLeadFootS: in a flight without a PSM jump detail (a fall / walk-off), from its second update until this many seconds into it, the
    // run-off / take-off rows (F Off, B / LL / RL / F Start) of the other lead foot than the jump clip on display are out: the air entry keeps the
    // foot it started on (a running ledge fall flipped Off_Run_Lfoot -> Off_Run_Rfoot -> F_Start_Run_Lfoot within 33 ms). Later picks of the
    // flight and game jumps (their take-off / impulse picks are set by the rules above) are unchanged. 0 = off.
    float airKeepLeadFootS = 0.15f;
    // fix/jumpout-cam (game log 2026-11-03 16:28:47, car jump-outs JO1 / JO2): the flight of a vehicle jump-out (ChooserState::jumpOutFlight)
    //  - jumpOutNoBack: takes no B / LL / RL Start row. The jump-out launches along the exit; the stick held against the momentum (brake) pulled the
    //    capsule's horizontal speed through zero and back (JO1: 5 m/s -> 0.05 m/s at 1.0 s -> -1.4 m/s) and the query then asked for a jump BACKWARDS
    //    (F_Start_Sprint_Lfoot -> B_Start_Rfoot at clip time 0.87 with the 0.5 s air fade, 0.45 s before the touchdown).
    //  - jumpOutFallHandover: from the apex (vz <= 0) to the touchdown no Start / Off clip is played or continued, so the fall loop
    //    (M_Neutral_Jump_Loop_Fall) of the database takes over (JO2: the stand take-off F_Start_Stand_Rfoot, authored for a ~0.6 s push-off, was held
    //    through the whole 1.4 s flight: clip time 1.35 at the landing). Only in databases that hold a fall clip; the decision is latched for the flight.
    // false = the pre-fix selection, bit for bit.
    bool jumpOutNoBack = true;
    bool jumpOutFallHandover = true;

    // fix3/jump (N3, recording 2026-10-21 12:15): quick slide combinations popped the pose in 1-3 frames. Every knob is a cross-fade
    // time in seconds, 0 = the pre-fix time.
    // blendSlideAir: floor of the cross-fade into the AIR within slideAirWindowS seconds of the last slide update (the displayed pose is
    // then still the lying slide pose or the rising exit clip). The first InAir update used blendSlide (0.2 s), the take-off row replaced
    // that pick one frame later with the 0.15 s air-rising time, and the legs (x 1/1.5) crossed the sitting -> standing change in 0.1 s:
    // Hips 47 cm and the thigh 85 deg per 50 ms.
    float blendSlideAir = kBlendSlideAirS;
    float slideAirWindowS = kSlideAirWindowS;
    // blendSlideShortExit: cross-fade of the exit of a slide that never settled (MatcherConfig::slideExitSettledS; was blendSlide).
    float blendSlideShortExit = 0.0f; // 0 = blendSlide: no gain measured (a slide shorter than its own 0.2 s entry fade has no pose to continue)
    // blendTipAfterSlide: cap of the stationary aim / shot / rest turn-in-place fade (0.1 s) while the displayed pose is a slide exit clip that is
    // still rising: its Hips are more than tipRiseGapM below the Hips of the turn-in-place row (the lying -> crouching change then ran in 0.1 s).
    float blendTipAfterSlide = kBlendTipAfterSlideS;
    float tipRiseGapM = kTipRiseGapM;
    // blendSlideFamily: cross-fade of a FootOut <-> KneesOut switch inside a slide (the leg pose differs by ~90 deg).
    float blendSlideFamily = kBlendSlideFamilyS;

    // G1 (established slow walk; tests/SlowWalkLoopTest.cpp, installed bake utpp_motion_woman, 726 clips). false = the motion behaviour
    // bit for bit (both rules below off). MEASURED, not assumed: at the chooser speed of an analog half stick (0.82 m/s -> 102 cm/s) and of
    // the walk toggle at the walk point (1.6 m/s -> 200 cm/s) the forward rows of PSD_Dense_Stand_Walk_Loops nearest the query are NOT those of
    // M_Neutral_Walk_Loop_F (2.19 m/s) but the dip of the non-loop M_Neutral_Transition_Run_to_Walk_L/Rfoot clips (clip time 1.7-2.4 s, root
    // speed ~1.9-2.0 m/s, the slowest forward rows of the database): at 102 cm/s the best row costs 0.021-0.029 against 0.035-0.039 for the
    // loop, almost all of it the trajectory channels (position / velocity at +0.35 / +0.70 s: 0.002-0.004 against 0.009-0.010), the pose
    // channels play no part. The clips carry the notify bias PoseSearchOverrideContinuingPoseCostBias -0.1 (continuingAddend), so once picked
    // the clip is held to its end (4.2 s), and its end re-picks it (entry flag DISABLE_RESELECTION only guards the continuing candidate): the
    // game logs of 2026-10-30 / 31 never show a Walk_Loop_* in a steady slow walk. The play rate plays no part in the pick (it is computed
    // after the winner is chosen). Two rules:
    //  - walkTransitionWindowS: the Run_to_Walk transition clips are legal NEW candidates only within this many seconds of a fast update
    //    (gait Run / Sprint, or chooser speed above walkTransitionFastCmS: a real run -> walk deceleration, which is what they are for).
    //    An idle -> walk start or a steady walk never starts one. A transition that was admitted still plays on (continuing candidate).
    //    0 = the transition clips are never excluded.
    float walkTransitionWindowS = 1.0f;
    float walkTransitionFastCmS = 260.0f; // motion walk cap 200 cm/s + margin
    // walkMinPlayRate: lower play rate bound of a LOOP clip of PSD_*_Stand_Walk_Loops while the chooser gait is Walk (0 = the ordinary
    // minPlayRate 0.85). The wanted rate is query speed / row speed: 0.47-0.52 at 102 cm/s on the forward loops (row 2.19 m/s), 0.52 on
    // LL / RL (1.97), 0.62 on B (1.64); 0.5 keeps the feet within ~6% of the ground at the measured half stick, and a slow motion below it
    // would only trade the foot slide for a crawl (the database has no walk below 1.64 m/s). It also lowers the rate inside motion's own
    // 150-185 cm/s walk range (0.75-0.85 instead of the 0.85 clamp), which is the point (less foot slide).
    float walkMinPlayRate = 0.5f;
    bool slowWalkLoops = true;

    // G10 (steady Run gait between the walk cap and the run loops; tests/SteadyRunLoopTest.cpp, installed bake utpp_motion_woman, 726 clips).
    // false = the motion behaviour bit for bit (all rules below off). MEASURED on the game feed of 2026-10-31 15:44 (analog ramp, stand, no weapon;
    // replay through the installed bake): the game speed steps 1.50 / 1.95 / 2.50 / 3.85 m/s map to the chooser speed 187 / 247 / 320 / 500 cm/s,
    // and the Run gait starts at the stick 0.7 (analogWalkRunThreshold), i.e. at 247 cm/s, while the databases hold no steady motion between the
    // Walk_Loop_F (219 cm/s, rate 0.85-1.15 -> 186-252) and the Run_Loop_B / LL / RL (330-388 cm/s; F / FL / FR 554). In that band the cheapest rows of
    // PSD_*_Stand_Run_Loops are the ones of the non-loop M_Neutral_Transition_Walk_to_Run_L/Rfoot clips (they pass 2-4 m/s):
    //  - 247 cm/s: cost 0.027-0.074, against 0.25 for Run_Loop_F; the held clip drifts away from the steady query (its cost rises 0.029 -> 0.047
    //    in 0.15 s) until the isolated row at clip time 0 (the clip is indexed at frames 0 and 28.., nothing between) of the OTHER foot costs less;
    //    the next update has no continuing row (frame 1 is not indexed, cont = -1), the full search then re-picks the same clip at 0.93-0.97 s (the
    //    reselection history holds the row at 0): two cross-fades, a pose reset every ~0.17 s for as long as the stick rests (3 s in the feed);
    //  - 320 cm/s: the same clips with the other foot at 0.93-1.38 s, a pick every ~0.9 s; Run_Loop_F only at 500 cm/s.
    // The play rate plays no part in the pick (computed after the winner is chosen). Rules:
    //  - runTransitionWindowS: the Walk_to_Run transition clips are legal NEW candidates only within this many seconds of a slow update (chooser speed
    //    at most runTransitionSlowCmS, standing included: a real walk -> run acceleration, which is what they are for); 0 = never excluded. The
    //    admitted clip still plays on (continuing candidate).
    //  - runTransitionNoHeadRow: the head row (clip time 0) of those clips is no NEW candidate while the next row is not the next frame: it has no
    //    continuing row (the reset above); the clip is entered at 0.93 s. Only above runTransitionSlowCmS: a run start from rest (chooser speed 57-69 cm/s
    //    in the 2026-10-25..28 feeds, Walk_to_Run head -> Run_Start_F) keeps the motion pick.
    //  - runWalkLoopsMinCmS .. runWalkLoopsMaxCmS: in the Run gait on the stand ground, moving, not starting / pivoting, in this chooser speed range (a
    //    run-gait update at 62 cm/s took Walk_Loop_FR over Run_Diamond_B_FR in the 2026-10-30 feed: below the range nothing changes) the stand Walk_Loops database
    //    of the same LOD joins the searched set (measured: Walk_Loop_F at rate 1.12 matches 247 cm/s, cost -0.013 against 0.25 for Run_Loop_F at its rate
    //    floor), so the stick passing 0.7 does not change the playing clip either (the gait flip is an InterruptOnDatabaseChange). Above
    //    runWalkLoopsMaxCmS + runWalkLoopsHoldCmS a continuing Walk_Loops pose is not evaluated any more (it would be held by DoNotInterrupt at 1.15, 21% slide
    //    at 320 cm/s); 0 = off.
    //  - runMinPlayRate: lower play rate bound of a LOOP clip of PSD_*_Stand_Run_Loops while the chooser gait is Run (0 = the ordinary minPlayRate 0.85).
    //    Wanted rate = query speed / row speed: 0.58 at 320 cm/s on Run_Loop_F, 0.77 at 427, 0.90 at 500 (untouched); 0.5 keeps the feet within 15% of
    //    the ground (as walkMinPlayRate); the database has no steady run below 330 cm/s. Whether 0.5-0.6 reads as a slow-motion run is a visual question.
    bool steadyRunLoops = true;
    float runTransitionWindowS = 1.0f;
    float runTransitionSlowCmS = 210.0f; // motion walk cap 200 cm/s + margin
    bool runTransitionNoHeadRow = true;
    float runMinPlayRate = 0.5f;
    float runWalkLoopsMaxCmS = 260.0f;   // Walk_Loop_F 219 cm/s x maxPlayRate 1.15 = 252 + margin
    float runWalkLoopsMinCmS = 180.0f;   // Walk_Loop_F 219 cm/s x 0.85 = 186: below it the ordinary bound would slide the feet
    float runWalkLoopsHoldCmS = 20.0f;

    // CR-3 (slow crouch walk; recording 2026-10-31 22:55, replay tests/LocoMatcherTest.cpp). false = the motion
    // behaviour bit for bit. MEASURED: at the slow crouch stick (feed |mv| 0.18, capsule 0.58 m/s -> chooser speed 48 cm/s, gait Walk) the matcher
    // plays M_Neutral_Crouch_Loop_F of PSD_Dense_Crouch_Walk_Loops (root speed 2.46 m/s, the slowest crouch loop ~2 m/s: the database has no slow
    // crouch walk), wanted rate 0.2, clamped at minPlayRate 0.85: ~2.1 m/s of stride over 0.58 m/s of capsule (3.5x foot slide). The G1 / G10 rate
    // exemptions cover only the stand Walk / Run loop databases. crouchWalkMinPlayRate: lower play rate bound of a LOOP clip of PSD_*_Crouch_Walk_Loops
    // while the chooser stance is Crouch and the gait Walk (0 = the ordinary minPlayRate 0.85); the full crouch stick (gait Run, ~0.92) and every
    // non-loop clip keep the ordinary bound. 0.5 = walkMinPlayRate: it halves the slide at the slow stick (the rest needs a slower clip, not a rate).
    bool slowCrouchLoops = true;
    float crouchWalkMinPlayRate = 0.5f;

    // ST-1 (slow walk start tail; same recording, replay tests/LocoMatcherTest.cpp). false = the UE DoNotInterrupt behaviour bit for bit. A start
    // from rest is IsStarting on its first stick update only (the game capsule reaches its slow target within ~3 feeds), so the chooser returns the
    // *_Walk_Starts database once: the Start clip picked there then lives on as the continuing pose outside the chooser set, which DoNotInterrupt
    // keeps evaluating (UE PopulateContinuingPoseSearches, the comment block in Update). MEASURED: after the crouch start (cwalk_slow, chooser speed
    // 49 cm/s) M_Neutral_Crouch_Start_F_Rfoot costs 0.088-0.130 (continuing addend -0.010 included) against 0.136-0.196 for the best
    // Crouch_Loop_F row, after the stand walk start (104 cm/s) M_Neutral_Walk_Start_F_Rfoot 0.03-0.04 against 0.11-0.15 for Walk_Loop_F: the
    // Start clip is held for its whole indexed range (crouch 3.2 s, walk 2.9 s; motion indexes the whole clip, sampling range 0-0)
    // at the clamped minPlayRate 0.85 (wanted ~0.2 / ~0.47: Start tails run at the steady 2.46 / 2.19 m/s),
    // while the loops it would hand over to get the CR-3 / G1 0.5 floor. motion's own walk speeds (>= 150 cm/s) keep the tail within the rate range,
    // the game's slow sticks do not. slowWalkStartTail: in the Walk gait, moving on the ground and not starting, a continuing clip of a
    // *_Walk_Starts database that the chooser no longer returns is not evaluated (it still builds the query pose channels, as G10's dropped
    // Walk_Loops pose) once it is past its authored acceleration (startTailAccelFraction of the clip's steady root speed, per clip at Init:
    // crouch F 0.2 s, walk F 0.33 s; a reface start also past its authored root turn) AND its wanted play rate is below minPlayRate (the tail
    // can only play with foot slide). The search then hands over to the loop database the chooser returns. Run / Sprint starts, a tail that fits
    // the rate range, the acceleration phase and a reface turn keep the motion behaviour. 0 fraction = the rule is off.
    bool slowWalkStartTail = true;
    float startTailAccelFraction = 0.9f;

    // RC2 (sprint -> slide entry; same recording). Every slide Into clip (PSD_Slide_FeetOut / KneesOut, M_Neutral_Slide_*_Into_L/Rfoot, 6.07 s,
    // indexed 0-2.07 s) is an upright sprint up to ~0.7 s and lowers the hips 0.7 -> 1.0 s (FootOut_Into_Lfoot 0.85 m -> 0.16 m), then lies. The
    // first slide update has no pose of the slide schema to continue (the locomotion databases use another schema and the adapter has no pose
    // history), so its pose channels are the slide database's mean (mostly lying frames) and the trajectory alone chose Into_Lfoot at 0.933 s
    // (hips 0.25 m, cost 3.44): the 0.2 s cross-fade from the upright sprint lowered the hips 36 cm per 50 ms. slideEntryMaxS: on the first Sliding
    // update after a non-slide clip only the Into rows up to this clip time are searched and the slide Loop clips not at all (the trajectory
    // takes the latest row allowed), so the slide starts just before the authored drop: replayed peak hips change per 50 ms 0.36 m (off) ->
    // 0.28 (0.7 s) -> 0.22 (0.6 s) -> 0.19 (0.5 s; the authored drop alone ~0.16). 0.6 s: the entry fade overlaps only the first half of the
    // drop and the drop starts 0.1 s after the slide (not at 0: the capsule already slides at 11 m/s, a 0.7 s sprint lead would drop late).
    // Same mechanism as slideExitMaxEntryS on the exit. 0 = off.
    // OFF in production (trade-off, not solved): the capped Into is ~0.33 s behind the old entry for the whole slide. When the chooser turns
    // the slide table to KneesOut mid-slide (MovementDirection Forward -> Other while the capsule decelerates) the FootOut Into at ~1.2-1.3 s
    // (still sliding forward, its cost rising 7 -> 186 over 0.17 s) loses to KneesOut_Into at 1.17 s (cost 217) right before the exit, and that
    // exit turns Hips 52-59 deg per 0.1 s (replayed game slides x1015-1438-70 / x1019-1743-266, SlideExitTest bound 25; 14 / 24 deg off). A
    // larger cap does not converge: 0.7 / 0.75 -> 62 deg, 0.8 passes the rotation bound but not the exit -> locomotion handover (50.2 / 50 deg)
    // and keeps the entry drop at 0.29 m per 50 ms; taking the family
    // from an unrestricted search of the entry update changed nothing (the switch happens mid-slide, not at the entry).
    float slideEntryMaxS = 0.0f;
    // RC3 (mid-slide re-jump; same recording; the known behaviour of the slideExitSettled* comment above). An Into clip reaches the end of its
    // indexed range at 2.07 s while the slide goes on (the bake excludes 2.1-6.07 s): FindRow finds no continuing row, the DISABLE_RESELECTION
    // protection lapses and the full search re-jumped to the mirrored Into clip at 1.03 s, which replays the settling of the legs (knee L 68 -> 44
    // -> 71 deg over ~1 s) every ~1 s of a long slide. slideIntoHandover: that update searches only the slide Loop clip of the same family in
    // the playing database (FootOut_Loop / KneesOut_Loop: the same lying pose; the replayed handover row continues the Into end within 1 deg at the
    // knees), which then continues under its own DISABLE_RESELECTION. false = off.
    bool slideIntoHandover = true;

    // CR-4 / RC6 / HIP-6 (crouch -> stand at rest; same recording). The stance edge admits the stance-transition clips for 0.6 s
    // (UpdateStanceAdmission), where they compete on plain cost; M_Neutral_Transition_Crouch_to_Stand carries the baked cost addend +0.1. When it
    // won (crouch_off3: ct 0, cost 0.100) nothing kept it: 0.33 s later, half way up, M_Neutral_Stand_Idle_Loop at 4.77 s (0.073) beat its
    // continuing cost (0.090) and a second 0.5 s cross-fade finished the rise (replayed head 10.5 cm per 50 ms against 8.2 for the authored rise).
    // crouchToStandHoldS: a Crouch -> Stand transition selected at rest is kept this long (its authored rise: hips up by 0.5 s, the torso by
    // ~0.8 s) while the character stays at rest in Stand on the ground; stick input, a stance change, air, a landing / traversal, a forced
    // interrupt or an aim / shot / pivot start release it at once. 0 = off. MEASURED, not changed here: when Stand_Idle_Loop wins the edge
    // (crouch_off / crouch_off2) the 0.5 s cross-fade raises the hips as fast as the authored clip does (7.6 against 8.7 cm per 50 ms, head 8.4
    // against 8.1, knees 28 against 23 deg), so forcing the transition clip would not change the rise time (~0.45-0.5 s either way).
    float crouchToStandHoldS = 0.8f;

    // Vehicle-exit transition clips (name prefix "VX_", authored by the bundle: game exit end pose -> generated transition -> motion stand idle).
    // Recorded 2026-11-05 14:41: the seeded first search picked VX_Exit_Idle_N10_MB (ct 0.30, cost 0.179); 0.5 s later its continuing cost
    // (0.094) lost to M_Neutral_Stand_Idle_Loop (0.087) and a generic 0.5 s cross-fade replaced the remaining ~2 s of the authored transition.
    // exitTransitionTailS: such a clip selected at rest (movementState Idle, speed <= exitTransitionMaxSpeedCmS, on the ground) is kept
    // (the other databases are not searched, like the CR-4 rise hold) until its last exitTransitionTailS seconds, where the ordinary search
    // takes over (the clip already ends in the idle pose). Stick input / speed, a stance change, air, a landing / traversal, a forced
    // interrupt, an aim / shot / pivot / spin / reface start or a missing continuing row release it at once. Without VX_ clips nothing
    // changes. 0 = off.
    float exitTransitionTailS = 0.25f;
    const char* exitTransitionPrefix = "VX_"; // clip name prefix resolved once in Init (null / empty = no clip); a test may name a full clip name
    float exitTransitionMaxSpeedCmS = 30.0f; // = kStandingJumpMaxSpeedCmS (0.3 m/s): the chooser's "at rest"

    // STOP COMMIT (dll-shoulder-brake 2026-11-09). A stop from a sprint: the Sprint_Stop clip stops its root ~0.2 s before the capsule (the
    // game brakes at ~9.3 m/s2, the clip harder). The continuing cost then lost to a Walk_Stop row half way through its own stop (film 1
    // f1555: Sprint_Stop ct 0.90 -> Walk_Stop_F_Lfoot ct 1.17) and the 0.25 s cross-fade between two stops passed the swinging foot
    // through the planted one (shins 2.3 cm in the evaluator pose, 0.6 cm after the foot lock; the clips keep 15 cm). stopCommitTailS: a
    // Stop clip (steer::kClassStop) selected with the stick released on the ground (movementState Idle) is kept (the other databases are
    // not searched, like exitTransitionTailS) until its last stopCommitTailS seconds; the ordinary search then picks the idle. Stick input
    // (movementState Moving), a stance change, air, a landing / traversal, a forced interrupt, an aim / shot / pivot / spin / reface start
    // or a missing continuing row release it in the same update (no added stick latency). 0 = off. stopCommitMaxS: the commit ends at the
    // latest this much clip time after the selection (the stop itself: the authored root motion ends within ~0.5 s, the capsule within
    // ~0.65 s of a 6 m/s sprint at 9.3 m/s2; the long Stop clips then idle on for seconds, where the ordinary search must stay free).
    // stopCommitCrouch: the commit also holds the crouched Stop clips. Off: held through the recorded crouch stop at 76.462 (Crouch_Stop_F_Lfoot
    // instead of the re-pick Crouch_Stop_B_Rfoot) the planted foot floated 3.25 cm (LocoStanceTest FS-13, bound 2.5 cm); the crossing it fixes
    // is a standing walk / run / sprint stop. The mechanism stays verified with it on (LocoMatcherTest StopChecks).
    float stopCommitTailS = 0.25f;
    float stopCommitMaxS = 1.0f;
    bool stopCommitCrouch = false;

    // SELECTION CHURN GUARD (2026-11-11; game harness 10:29 Female / 12:37 Male: slalom 3.9, flick 4.0, sprint dash 4.0 clip changes per second
    // against the 3.0 bound). An offline replay of the 10:29 feed dump through the real driver reproduces the clip changes of every window
    // (tests/MMChurnTest.cpp). The changes came from a missing hysteresis in three places here (and the stick gait in MapConfig::
    // gaitDownshiftHoldS); none of them delays a stick edge: every gait / movement state change still re-searches on the same update as before.
    // Each guard is off at 0 / false (the pre-guard behaviour). They act on the STAND stance only (every window above is stand locomotion; the
    // crouch stop keeps its re-pick, see stopCommitCrouch) and never on a Stop clip (the stop commit owns those).
    //  1. loopDwellS / loopDwellOverride: a steady loop swapped with its sibling loop (Run_Loop_F <-> Run_Loop_F_L_20 / _F_R_20 in the slalom,
    //     a change every 0.2-0.3 s) because the continuing pose lost by a rounding error (median 0.007, p90 0.027 over the whole dump, cost
    //     scale 0.1-0.4). A LOOP clip younger than loopDwellS gives way to another loop clip only when that one is better by more than
    //     loopDwellOverride (a heading change that really needs the other loop). Loop -> event clip (start / stop / pivot / transition) and
    //     every event -> anything switch are untouched.
    //  2. freshEventS / freshEventBias: an EVENT clip (not a loop) younger than freshEventS keeps freshEventBias of cost off its continuing
    //     cost while no stick edge reached the chooser (DoNotInterrupt), so a 0.002 .. 0.05 cost race does not replace a pivot / start a few
    //     frames after it was picked (Run_Diamond -> Transition_Walk_to_Run -> Run_Start_F 66 / 33 ms apart in the 180 deg reversal). A stick
    //     edge (InterruptOnDatabaseChange) re-searches at once as before; air, landing, traversal, stance change, aim / shot / rest turns,
    //     a forced interrupt and a slide exit never reach the bias.
    //  3. dodgeHold: the game's own ground Dodge (PSM detail 7) is an impulse of up to 11 m/s that decays within ~0.25 s; the trajectory
    //     query of that impulse picked a new pivot clip every 17-50 ms (Run_Prism_FL_F -> Run_Prism_FR_F -> ..., 4 of the 8 changes of the
    //     sprint dash windows). The playing clip is kept through it (like the other hold rules); a mode change (the dodge's jump) and a
    //     forced interrupt still search.
    //     loopDwellOverride 0.2: while the camera turns under a held direction key (mouse look) the wanted heading sweeps through the loop
    //     directions; with 0.3 the old loop was kept through such a turn and the planted foot drifted further (replays of keyboard-and-mouse
    //     sessions). 0.15 lets the slalom window churn again.
    float loopDwellS = 0.5f;
    float loopDwellOverride = 0.2f;
    // freshEventBias 0.025: half of the first value keeps the same reduction of clip changes over replays of six recorded sessions while a
    // Walk start under an accelerating capsule is no longer kept against a better start (toe drift of the starts back to the unguarded level)
    float freshEventS = 0.2f;
    float freshEventBias = 0.025f;
    bool dodgeHold = true;
    //  4. startUpshiftHoldS: a fast-rising stick passes the gait thresholds within ~0.1-0.4 s (Walk -> Run -> Sprint), and every gait edge
    //     (InterruptOnDatabaseChange) dropped the playing start clip and picked a fresh one of the new gait's start database: Walk start ->
    //     Run start -> Sprint start in a fraction of a second (flick_180_sprint / accel 5 clip changes in 1.5 s, sprint_again 9; a Walk_Start ->
    //     Run_Start swap at the ramp of run_arc_left / accel). A WALK start clip (picked in the Walk gait) or a REFACE start clip (a 180 deg
    //     reversal start of any gait: it carries the heading change, a new pick per gait edge restarts the turn) that is younger than
    //     startUpshiftHoldS meets a gait UPSHIFT (same movementState, gait above the one the clip was picked in) as a non-interrupting edge
    //     (DoNotInterrupt: the playing start keeps competing, so the freshEvent bias applies and a clearly better start still wins). When the time
    //     is up the deferred edge is delivered once (InterruptOnDatabaseChange), as a gait edge would have been. NOT held: a Run / Sprint straight
    //     start meeting the next upshift (the recorded 2026-10-31 sprint start, LocoStanceTest FS-8/9, pins that edge at 0.18 s), presses from idle
    //     (the first start is picked on the first feed), releases / movementState changes, downshifts, air, stance changes, forced interrupts; a
    //     clip older than startUpshiftHoldS re-searches at the edge as before (a slow, genuine walk start that is passed by the gait later is not
    //     held). Cost: the swap to the faster gait's start is deferred by up to the hold time minus the age of the start; reface starts have their own,
    //     shorter hold times (below). startUpshiftHoldS 0 = guard off. 0.25 s: the same counts in the windows that needed the guard (0.20 s lets
    //     a stalled walk start churn again) with less toe drift after a release that follows a short start than 0.35 s.
    float startUpshiftHoldS = 0.25f;
    // REFACE starts (2026-11-11 game runs 15:43-16:25 with the 0.35 s hold on them): the hold deferred the swap of a Walk reface start to the other
    // turn side (Walk_Reface_Start_F_L_180 -> Run_Reface_Start_F_R_180) from 66 ms to 183 ms after the pick, where the body had already turned and the
    // foot lock had to correct 16.1 cm (Male camera_whip_180 / accel; 6.7 cm before the hold). A reface start is therefore held only very briefly:
    // startUpshiftRefaceWalkHoldS (picked in the Walk gait; the swap follows ~35-50 ms later than without the hold, ~100 ms after the pick; a
    // toe-slide proxy of the replay stays within +2.5 cm of no hold, against +7 cm with 0.35 s) and startUpshiftRefaceRunHoldS (picked in the Run /
    // Sprint gait: the same-side Run -> Sprint reface chain of sprint_again; 0.28 s measured, the swap is a same-side one). 0 = no hold for that case.
    float startUpshiftRefaceWalkHoldS = 0.085f;
    float startUpshiftRefaceRunHoldS = 0.28f;
    //  5. startRestartDwellS / startRestartOverride / startStalledSpeedCmS (Female 2026-11-11 17:37 weapon_aim / walk_armed, aim_walk and weapon_walk /
    //     walk_unarmed: 4.0-4.3 clip changes per second, a visible stutter): the stick is held (Run gait) while the capsule does not move (blocked), the
    //     state stays "starting", and the playing start clip (Run_Start_F_Lfoot, clip time 0.18 s) loses by 0.015-0.02 of cost to the frame 0 of the
    //     start of the other foot (Run_Start_F_Rfoot) - 0.2 s later the same again: a start clip restarted from its first frame five times a second
    //     (the same with every guard off: 15 changes in 3 s; the freshEvent bias ends at 0.2 s = the period). Without a stick edge (the continuing
    //     pose is still searched) an event clip younger than startRestartDwellS gives way
    //       - to another start clip (not reface) of the SAME speed class that starts from its first frame (< 0.1 s), or
    //       - while the capsule is stalled (speed <= startStalledSpeedCmS, the stick held: movementState Moving), to any other event clip (start,
    //         pivot, transition; a Stop clip is never held and never replaced here: the stop commit owns it)
    //     only if that one is better by more than startRestartOverride. A gait / movementState edge, a pivot while the capsule moves and every loop
    //     are untouched. startRestartDwellS 0 = off.
    float startRestartDwellS = 1.0f;
    float startRestartOverride = 0.3f;
    float startStalledSpeedCmS = 30.0f;

    // TURN COMMIT. On a reversal of the stick the capsule velocity flips within one or two frames; the pivot query then matches the
    // straight run of the new direction and the continuing cost of a just picked 180 deg turn lost to a loop half way through its turn
    // (the turn was dropped while the authored root still had to rotate, a loop / reface chain followed and the planted foot drifted).
    // turnCommit: a pivot or moving-turn clip (steer::IsPivot / IsMoveTurn) selected with isPivoting on the stand ground while moving is
    // kept (the other databases are not searched, gait / movement state edges do not interrupt it) until its authored root yaw is done
    // (less than 1 deg left). A new reversal (isPivoting rising again after it had dropped during the commit), a stick release (movement
    // state 0, so the Stop is picked on the release frame), a stance change, air, a landing / traversal, a forced interrupt, an aim / shot /
    // spin / reface start or a missing continuing row release it in the same update. false = off.
    bool turnCommit = true;
    // turnCommitTailS: after the authored root turn is done the turn clip (already running in the new direction) stays held this long
    // (seconds), under the same release conditions plus any new heading step since the turn was picked. Keyboard taps 0.35 s apart
    // otherwise handed the finished turn to a loop one to seven feeds before the next tap picked the next turn (two changes for one
    // tap). 0 = the commit ends at the turn end.
    float turnCommitTailS = 0.2f;
    // downshiftDeferS: for this long after any gait downshift while moving (seconds) the edge does not interrupt the playing clip (it is
    // delivered once afterwards) and a Run_to_Walk transition is not picked over the continuing clip. A stick falling through the Walk
    // band to a release showed the transition for one feed before the Stop; a release within this time picks its Stop directly. A
    // downshift that stays reaches its new clip up to this much later. 0 = off.
    float downshiftDeferS = 0.04f;
    // headingDipHoldS: for this long after a heading step (ChooserState::headingStepAgeS) a gait downshift while moving is searched in the
    // gait of the playing clip. A thumbstick swung to a new direction at a run passes inside its rim for one or two feeds (the stick
    // length dips below the Run threshold); a Walk pivot / turn picked there was replaced by a Run clip one feed later. 0 = off.
    float headingDipHoldS = 0.1f;

    // HEADING CATCH-UP. A direction key changes the wanted run direction in one feed: the capsule velocity follows within one or two
    // feeds while the actor facing turns at its own rate (about 0.3-0.5 s for 90 deg). In that time the velocity points sideways or
    // backwards relative to the lagging facing, and the search picked the clips made for a body that keeps its facing (strafe loops
    // FR / RL / RR_offset ..., box / hourglass / prism patterns, pivots without an authored root turn), each replaced 50-200 ms later as
    // the facing caught up (one 90 deg key change: box -> RL -> FR -> F_R_20 -> F; one 45 deg change: F -> FR -> F_R_20 -> F).
    // headingCatchUpS: for this long after a heading step (ChooserState::headingStepAgeS), while the facing is still more than
    // headingSettledDeg away from the wanted direction (ChooserState::headingLagDeg), moving on the ground (standing or crouched), those clips are not NEW
    // candidates: the pick is a turn or a forward / curved loop that turns with the body (a picked turn is then kept by turnCommit). The
    // continuing pose is not affected. 0 = off.
    float headingCatchUpS = 1.0f;
    float headingSettledDeg = 12.0f;
};
#ifndef UTPP_LOCOMATCHER_MUTATION
#define UTPP_LOCOMATCHER_MUTATION 0
#endif
#ifndef UTPP_FOOTSINK_MUTATION
#define UTPP_FOOTSINK_MUTATION 0
#endif

struct BlendRequest
{
    bool valid = false;
    float blendTime = 0.0f;
    int32_t fromClip = -1;
    float fromTime = 0.0f;
    int32_t toClip = -1;
    float toTime = 0.0f;
    bool cancelledGroundStart = false;
    bool stanceChange = false; // ground -> ground fade across a stance change (crouch <-> stand): the pelvis travels, the feet stay planted
    // ground locomotion -> ground locomotion fade (idle / start / loop / stop / pivot / turn / transition databases of either stance; the
    // outgoing clip may also be a landing or a slide exit clip), both ends in a ground movement mode: the feet are planted or swing near the
    // floor, so BlendStack::KeepFootHeight may hold their height on the blend of the layers (lift only). Never a slide, jump, air or traversal
    // clip on the incoming side.
    bool groundFade = false;
    bool toGround = false; // the incoming clip alone is a ground locomotion clip in a ground movement mode (a replaced head inherits the fade only then)
    bool toAir = false;    // the incoming pick is made in the air movement mode (take-off / fall): BlendStack::KeepAnklesAboveLayers stays off under it
    bool fullLegFade = false; // a FootOut <-> KneesOut switch inside a slide (blendSlideFamily): BlendStack fades the leg bones over the whole blend time
};

struct MatchResult
{
    int32_t database = -1; // DBAS index of the playing pose
    int32_t localRow = -1;
    uint32_t entry = 0;
    int32_t clip = -1;
    uint32_t frame = 0;    // global frame of the selected / continuing pose
    float clipTime = 0.0f; // s
    float playRate = 1.0f; // selected player's rate for the next continuing interval
    float playRateRatio = 1.0f; // diagnostic: unclamped query / pose speed ratio of the last rate calculation (rate2::Result::ratio)
    float cost = std::numeric_limits<float>::max();
    float continuingCost = std::numeric_limits<float>::max();
    bool jumped = false;
    bool searched = false;
    bool continuingOutsideSet = false; // continuing pose kept although its database is not a chooser database (UE DoNotInterrupt)
    bool turnHold = false;             // MatcherConfig::turnCommit kept the playing turn clip on this update
    Interrupt interrupt = Interrupt::DoNotInterrupt;
    BlendRequest blend;
    uint64_t dbMask = 0; // chooser databases (DBAS bits)
    uint32_t numDatabases = 0;
    uint32_t rowsEvaluated = 0;
    bool airFallBias = false; // motionfix item 5: the fall bias was applied on this update
    bool cancelledStartSupportAllowed = false;
};

class ChooserMatcher
{
public:
    // aDb must outlive the matcher. aTable: chooser table whose database results reference DBAS indices of aDb.
    bool Init(const MotionDatabase* aDb, const chooser::Table* aTable, const MatcherConfig& aCfg, std::string& aError)
    {
        *this = ChooserMatcher{};
        if (!aDb || !aDb->Loaded())
            return Fail(aError, "database not loaded");
        if (!aTable || !aTable->Loaded())
            return Fail(aError, "chooser table not loaded");
        m_db = aDb;
        m_table = aTable;
        m_cfg = aCfg;
        uint32_t maxRows = 0;
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            maxRows = std::max(maxRows, aDb->Db(d).numRows);
        m_excluded.assign(maxRows, 0);
        m_raw.assign(aDb->Header().maxDims + 1, 0.0f);
        m_query.assign(aDb->Header().maxDims + 1, 0.0f);
        m_history.assign(256, HistEntry{});
        m_startTurnDirection.assign(aDb->NumClips(), 0);
        m_stanceTransitionDirection.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const auto& clip = aDb->Clip(c);
            if (steer::ClassifyClip(aDb->ClipName(c)) == steer::kClassStanceTransition)
            {
                const int direction = steer::StanceDirOfName(aDb->ClipName(c));
                m_stanceTransitionDirection[c] = direction ? static_cast<int8_t>(direction) : 2;
            }
            if ((clip.flags & kClipLoop) || steer::ClassifyClip(aDb->ClipName(c)) != steer::kClassReface)
                continue;
            double yaw = 0;
            for (uint32_t f = 0; f < clip.numFrames; ++f)
                yaw += aDb->Root(clip.firstFrame + f).dyaw;
            if (std::isfinite(yaw) && std::fabs(yaw) > math::kPi / 180.f)
                m_startTurnDirection[c] = yaw > 0 ? 1 : -1;
        }
        m_clipExitTransition.assign(aDb->NumClips(), 0);
        if (m_cfg.exitTransitionPrefix && m_cfg.exitTransitionPrefix[0] && m_cfg.exitTransitionTailS > 0.0f)
            for (uint32_t c = 0; c < aDb->NumClips(); ++c)
                m_clipExitTransition[c] = aDb->ClipName(c).starts_with(std::string_view(m_cfg.exitTransitionPrefix)) &&
                                          !(aDb->Clip(c).flags & kClipLoop) ? 1 : 0;
        m_exitHoldClip = -1;
        m_stopHoldClip = -1;
        m_turnHoldClip = -1;
        m_headingRestArmed = false;
        m_dbSlideExit.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            m_dbSlideExit[d] = aDb->DbName(d).starts_with("PSD_Slide_ExitTo") ? 1 : 0;
        // ground locomotion databases for BlendRequest::groundFade, by name: the dense / sparse locomotion sets minus jumps, landings and
        // traversal hand-overs (to); the outgoing side also accepts the landing and the slide exit databases (their feet are planted)
        m_dbFadeTo.assign(aDb->NumDatabases(), 0);
        m_dbFadeFrom.assign(aDb->NumDatabases(), 0);
        m_dbTurnInPlace.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
        {
            const auto name = aDb->DbName(d);
            const bool locomotion = name.starts_with("PSD_Dense_") || name.starts_with("PSD_Sparse_");
            const bool jump = name.find("Jump") != std::string_view::npos, land = name.find("_Lands_") != std::string_view::npos,
                       traversal = name.find("FromTraversal") != std::string_view::npos;
            m_dbFadeTo[d] = locomotion && !jump && !land && !traversal ? 1 : 0;
            m_dbFadeFrom[d] = m_dbFadeTo[d] || (locomotion && land) || m_dbSlideExit[d] ? 1 : 0;
            m_dbTurnInPlace[d] = name.find("TurnInPlace") != std::string_view::npos ? 1 : 0;
        }
        m_dbSlide.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            m_dbSlide[d] = aDb->DbName(d).starts_with("PSD_Slide_") && !m_dbSlideExit[d] ? 1 : 0;
        m_clipSlideFamily.assign(aDb->NumClips(), 0);
        m_clipSlideLoop.assign(aDb->NumClips(), 0);
        m_clipSlideInto.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const auto n = aDb->ClipName(c);
            m_clipSlideLoop[c] = n.find("Slide_FootOut_Loop") != std::string_view::npos || n.find("Slide_KneesOut_Loop") != std::string_view::npos;
            m_clipSlideInto[c] = n.find("Slide_FootOut_Into_") != std::string_view::npos || n.find("Slide_KneesOut_Into_") != std::string_view::npos;
            if (n.find("Slide_FootOut_") != std::string_view::npos)
                m_clipSlideFamily[c] = 1;
            else if (n.find("Slide_KneesOut_") != std::string_view::npos)
                m_clipSlideFamily[c] = 2;
        }
        // G1 (MatcherConfig::slowWalkLoops): the stand walk-loop databases and the Run_to_Walk transition clips, by name
        m_dbStandWalkLoops.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            m_dbStandWalkLoops[d] = aDb->DbName(d).starts_with("PSD_Dense_Stand_Walk_Loops") ||
                                    aDb->DbName(d).starts_with("PSD_Sparse_Stand_Walk_Loops") ? 1 : 0;
        m_dbStandRunLoops.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            m_dbStandRunLoops[d] = aDb->DbName(d).starts_with("PSD_Dense_Stand_Run_Loops") ||
                                   aDb->DbName(d).starts_with("PSD_Sparse_Stand_Run_Loops") ? 1 : 0;
        m_dbCrouchWalkLoops.assign(aDb->NumDatabases(), 0); // CR-3 (MatcherConfig::slowCrouchLoops)
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            m_dbCrouchWalkLoops[d] = aDb->DbName(d).starts_with("PSD_Dense_Crouch_Walk_Loops") ||
                                     aDb->DbName(d).starts_with("PSD_Sparse_Crouch_Walk_Loops") ? 1 : 0;
        // ST-1 (MatcherConfig::slowWalkStartTail): the walk-start databases (stand and crouch) and the end of each of their clips' acceleration,
        // the first frame whose 0.1 s root speed reaches startTailAccelFraction of the clip's steady speed (mean root speed of its second half)
        m_dbWalkStarts.assign(aDb->NumDatabases(), 0);
        m_clipAccelEndS.assign(aDb->NumClips(), -1.0f);
        const uint32_t speedWin = std::max(1u, static_cast<uint32_t>(std::lround(aDb->Header().fps * 0.1f)));
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
        {
            const auto dn = aDb->DbName(d);
            m_dbWalkStarts[d] = (dn.starts_with("PSD_Dense_") || dn.starts_with("PSD_Sparse_")) && dn.ends_with("_Walk_Starts") ? 1 : 0;
            if (!m_dbWalkStarts[d])
                continue;
            const DatabaseRecord& D = aDb->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const uint32_t c = aDb->Entry(e).clip;
                const MotionClipRecord& cl = aDb->Clip(c);
                if (m_clipAccelEndS[c] >= 0.0f || cl.numFrames < 2 * speedWin + 2)
                    continue;
                double steady = 0.0;
                const uint32_t half = cl.numFrames / 2;
                for (uint32_t f = half; f < cl.numFrames; ++f)
                    steady += std::hypot(aDb->Root(cl.firstFrame + f).dx, aDb->Root(cl.firstFrame + f).dy);
                steady /= static_cast<double>(cl.numFrames - half);
                double win = 0.0;
                for (uint32_t f = 1; f < cl.numFrames; ++f)
                {
                    win += std::hypot(aDb->Root(cl.firstFrame + f).dx, aDb->Root(cl.firstFrame + f).dy);
                    if (f > speedWin)
                        win -= std::hypot(aDb->Root(cl.firstFrame + f - speedWin).dx, aDb->Root(cl.firstFrame + f - speedWin).dy);
                    if (f >= speedWin && win / speedWin >= m_cfg.startTailAccelFraction * steady)
                    {
                        m_clipAccelEndS[c] = static_cast<float>(f) / aDb->Header().fps;
                        break;
                    }
                }
                // a reface start (m_startTurnDirection) keeps its authored root turn: the 90 / 180 deg turns end after the acceleration (0.37-0.53 s
                // crouch, 0.4-0.7 s walk, against 0.27-0.4 s), so the tail starts where less than 1 deg of the turn is left
                if (m_clipAccelEndS[c] >= 0.0f && m_startTurnDirection[c])
                {
                    double total = 0.0, done = 0.0;
                    for (uint32_t f = 0; f < cl.numFrames; ++f)
                        total += std::fabs(aDb->Root(cl.firstFrame + f).dyaw);
                    for (uint32_t f = 0; f < cl.numFrames; ++f)
                    {
                        done += std::fabs(aDb->Root(cl.firstFrame + f).dyaw);
                        if (total - done <= math::kPi / 180.0)
                        {
                            m_clipAccelEndS[c] = std::fmax(m_clipAccelEndS[c], static_cast<float>(f) / aDb->Header().fps);
                            break;
                        }
                    }
                }
            }
        }
        m_dbWalkSibling.assign(aDb->NumDatabases(), -1);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
            if (m_dbStandRunLoops[d])
            {
                std::string n(aDb->DbName(d));
                n.replace(n.find("Run_Loops"), 9, "Walk_Loops");
                for (uint32_t w = 0; w < aDb->NumDatabases(); ++w)
                    if (aDb->DbName(w) == n)
                        m_dbWalkSibling[d] = static_cast<int32_t>(w);
            }
        m_clipWalkToRun.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
            m_clipWalkToRun[c] = aDb->ClipName(c).find("Transition_Walk_to_Run") != std::string_view::npos ? 1 : 0;
        m_clipRunToWalk.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
            m_clipRunToWalk[c] = aDb->ClipName(c).find("Transition_Run_to_Walk") != std::string_view::npos ? 1 : 0;
        m_clipStop.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
            m_clipStop[c] = steer::ClassifyClip(aDb->ClipName(c)) == steer::kClassStop ? 1 : 0;
        // turnCommit: clip time at which less than 1 deg of the authored root turn of a pivot / moving-turn clip is left (-1 = no such clip)
        m_clipTurnEndS.assign(aDb->NumClips(), -1.0f);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const uint8_t cls = steer::ClassifyClip(aDb->ClipName(c));
            if (!steer::IsPivot(cls) && !steer::IsMoveTurn(cls))
                continue;
            const MotionClipRecord& cl = aDb->Clip(c);
            double total = 0.0, done = 0.0;
            for (uint32_t f = 0; f < cl.numFrames; ++f)
                total += std::fabs(aDb->Root(cl.firstFrame + f).dyaw);
            for (uint32_t f = 0; f < cl.numFrames; ++f)
            {
                done += std::fabs(aDb->Root(cl.firstFrame + f).dyaw);
                if (total - done <= math::kPi / 180.0)
                {
                    m_clipTurnEndS[c] = static_cast<float>(f) / aDb->Header().fps;
                    break;
                }
            }
        }
        // headingCatchUpS: clips made for a body that keeps its facing - pattern and side-step shuffle clips, pivots / turns with less than 45 deg of authored
        // root yaw, and loops that run in any direction but forward (forward = Loop_F and the curved Loop_F_L_* / Loop_F_R_*)
        m_clipKeepsFacing.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const std::string_view name = aDb->ClipName(c);
            const uint8_t cls = steer::ClassifyClip(name);
            if (cls == steer::kClassPattern || name.find("_Shuffle_") != std::string_view::npos)
                m_clipKeepsFacing[c] = 1;
            else if (steer::IsPivot(cls) || steer::IsMoveTurn(cls))
            {
                const MotionClipRecord& cl = aDb->Clip(c);
                double yaw = 0.0;
                for (uint32_t f = 0; f < cl.numFrames; ++f)
                    yaw += aDb->Root(cl.firstFrame + f).dyaw;
                m_clipKeepsFacing[c] = std::fabs(yaw) < 45.0 * math::kPi / 180.0 ? 1 : 0;
            }
            else if (aDb->Clip(c).flags & kClipLoop)
            {
                const size_t p = name.find("_Loop_");
                if (p != std::string_view::npos)
                {
                    const std::string_view dir = name.substr(p + 6);
                    m_clipKeepsFacing[c] = (dir == "F" || dir.starts_with("F_L_") || dir.starts_with("F_R_")) ? 0 : 1;
                }
            }
        }
        m_clipStart.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const uint8_t cc = steer::ClassifyClip(aDb->ClipName(c));
            m_clipStart[c] = cc == steer::kClassReface ? 2 : ((cc == steer::kClassStart || cc == steer::kClassStartSprint) ? 1 : 0);
        }
        m_fallClip.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
            for (const char* n : kAirFallClipNames)
                if (aDb->ClipName(c) == n)
                {
                    m_fallClip[c] = 1;
                    m_anyFallClip = true;
                }
        m_hipsChain.clear();
        for (uint32_t b = 0; b < aDb->NumBones(); ++b)
            if (aDb->BoneName(b) == "Hips")
                for (int k = static_cast<int>(b); k >= 0; k = aDb->BoneParent(static_cast<uint32_t>(k)))
                    m_hipsChain.insert(m_hipsChain.begin(), k);
        m_clipJumpKind.assign(aDb->NumClips(), 0);
        m_clipJumpFoot.assign(aDb->NumClips(), 0);
        for (uint32_t c = 0; c < aDb->NumClips(); ++c)
        {
            const auto n = aDb->ClipName(c);
            if (n.find("Jump_B_Start_") != std::string_view::npos || n.find("Jump_LL_Start_") != std::string_view::npos ||
                n.find("Jump_RL_Start_") != std::string_view::npos)
                m_clipJumpKind[c] = 1;
            else if (n.find("Jump_F_Start_") != std::string_view::npos)
                m_clipJumpKind[c] = 2;
            else if (n.find("Jump_F_Off_") != std::string_view::npos)
                m_clipJumpKind[c] = 3;
            if (m_clipJumpKind[c] != 0)
                m_clipJumpFoot[c] = n.find("_Lfoot") != std::string_view::npos ? 1 : n.find("_Rfoot") != std::string_view::npos ? 2 : 0;
        }
        // fix/jumpout-cam: databases that index a fall clip (rows searchable): only there may the jump-out hand-over drop the Start / Off rows
        m_dbHasFall.assign(aDb->NumDatabases(), 0);
        for (uint32_t d = 0; d < aDb->NumDatabases(); ++d)
        {
            const DatabaseRecord& D = aDb->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
                if (m_fallClip[aDb->Entry(e).clip] && aDb->Entry(e).numRows)
                    m_dbHasFall[d] = 1;
        }
        auto in = [&](const char* k) { return aTable->FindInput(k); };
        m_in.gait = in("Gait");
        m_in.stance = in("Stance");
        m_in.movementState = in("MovementState");
        m_in.movementMode = in("MovementMode");
        m_in.speed = in("Speed2D");
        m_in.ttl = in("TrajectoryCollision.TimeToLand");
        m_in.starting = in("IsStarting");
        m_in.pivoting = in("IsPivoting");
        m_in.landLight = in("JustLanded_Light");
        m_in.landHeavy = in("JustLanded_Heavy");
        m_in.traversed = in("JustTraversed");
        m_in.tip = in("ShouldTurnInPlace");
        m_in.spin = in("ShouldSpinTransition");
        m_in.lod = in("MMDatabaseLOD");
        if (aTable->GetSlide().present)
        {
            m_in.movementDirection = aTable->GetSlide().directionInput;
            m_in.movementModeLast = aTable->GetSlide().lastModeInput;
        }
        if (m_in.gait < 0 || m_in.stance < 0 || m_in.movementState < 0 || m_in.movementMode < 0)
            return Fail(aError, "chooser table lacks Gait / Stance / MovementState / MovementMode inputs");
        m_ready = true;
        return true;
    }

    bool Ready() const { return m_ready; }
    // fix/slide-exit: the chooser flags that start a different motion than the exit clip plays; the exit hold is released when one is
    // set that was not set when the clip was selected (a start, not a level): bit 0 aim turn that needs footwork, 1 shot turn, 2 pivot,
    // 3 spin transition, 4 ordinary reface start.
    static int HoldFlags(const ChooserState& s)
    {
#if UTPP_SLIDEEXIT_MUTATION == 8
        return (s.stationaryAimTurn && s.shouldTurnInPlace ? 1 : 0) | (s.stationaryShotTurn ? 2 : 0);
#else
        return (s.stationaryAimTurn && s.shouldTurnInPlace ? 1 : 0) | (s.stationaryShotTurn ? 2 : 0) | (s.isPivoting ? 4 : 0) |
               (s.shouldSpinTransition ? 8 : 0) | (s.startTurnDirection != 0 ? 16 : 0);
#endif
    }
    const MatchResult& Last() const { return m_last; }
    // fix3/jump (test access): the standing-jump rules (MatcherConfig::standingJumpMaxSpeedCmS) are decided for the flight in progress
    bool StandingJumpRules() const { return m_cfg.standingJumpMaxSpeedCmS > 0.0f && m_flightActive && m_flightStanding; }
    // fix5/jump (test access): the last update kept the F Off rows out as a jump in motion (MatcherConfig::jumpRiseNoOff); FlightJump: the flight
    // in progress carried a PSM jump detail
    bool JumpRiseOffBlock() const { return m_jumpRiseOff; }
    bool FlightJump() const { return m_flightActive && m_flightJump; }
    // fix/jumpout-cam (test access): the last update was inside a vehicle jump-out flight (ChooserState::jumpOutFlight, latched for the flight); JumpOutNoBack: the
    // B / LL / RL Start rows were kept out; JumpOutFall: past the apex the Start / Off rows were dropped (the fall loop owns the descent)
    bool JumpOutNoBack() const { return m_jumpOutBack; }
    bool JumpOutFall() const { return m_jumpOutFall; }
    // Selection churn guards as a group (MatcherConfig "SELECTION CHURN GUARD": loop dwell, young event bias, dodge hold, start upshift hold,
    // start restart dwell). true (default) = each guard as configured; false = every one of them off, the same selections as with their
    // knobs at 0 / false. Takes effect on the next Update; no other state is touched.
    void SetSelectionGuards(bool aOn) { m_guardsOn = aOn; }
    bool SelectionGuards() const { return m_guardsOn; }
    // Authored ROOT, not a name suffix or the already steered blended root rate.
    // Zero means an invalid clip, a non-reface, or no meaningful authored turn.
    int StartTurnDirection(int32_t aClip) const
    {
        return aClip >= 0 && static_cast<size_t>(aClip) < m_startTurnDirection.size() ?
            m_startTurnDirection[static_cast<size_t>(aClip)] : 0;
    }
    // Reset the player/history after a runtime yield. Preserve a consumed stance
    // transition if requested, but close an unconsumed admission window: the
    // duration and stance edges during the yield are unknown. Re-entry is not an edge.
    void Reset(bool aPreserveCompletedNoReselect = false)
    {
        m_cur = {};
        m_slideExitHoldLeft = 0.0f;
        m_riseHoldLeft = 0.0f;
        m_exitHoldClip = -1;
        m_stopHoldClip = -1;
        m_turnHoldClip = -1;
        m_headingRestArmed = false;
        m_slideExitAge = 1.0e9f;
        m_sinceJumpS = 1.0e9f;
        m_sinceSlideS = 1.0e9f;
        m_sinceFastS = 1.0e9f;
        m_sinceSlowS = 1.0e9f;
        UTPP_IF_NOT_MUT(JUMPPICK, 11) // mutation 11: a flight resumed after a yield / traversal hand-back inherits the old standing-jump decision
        {
            m_flightActive = false;
            m_flightStanding = false;
            m_flightAge = 0.0f;
        }
        m_flightJump = false;
        m_flightJumpOut = false;
        m_jumpOutFallLatched = false;
        if (!aPreserveCompletedNoReselect)
        {
            m_completedNoReselectClip = -1;
            m_completedNoReselectStance = -1;
            m_stanceObserved = false;
            m_observedStance = -1;
        }
        m_stanceGround = false;
        m_stanceEntryRemaining = 0.f;
        m_stanceEntryAllowed = false;
        m_numHist = 0;
        m_numCand = 0;
        m_haveLastState = false;
        m_now = 0.0;
    }

    // Fills the chooser input vector from a ChooserState.
    void FillInputs(const ChooserState& s, chooser::Inputs& aIn) const
    {
        const chooser::Table& t = *m_table;
        t.ResetInputs(aIn);
        auto num = [&](int i, double v) { if (i >= 0) t.SetNumber(aIn, i, v); };
        auto bo = [&](int i, bool v) { if (i >= 0) t.SetBool(aIn, i, v); };
        auto en = [&](int i, int v) { if (i >= 0) t.SetEnum(aIn, i, v); };
        en(m_in.gait, s.gait);
        en(m_in.stance, s.stance);
        en(m_in.movementState, s.movementState);
        en(m_in.movementMode, s.movementMode);
        en(m_in.movementDirection, s.movementDirection);
        en(m_in.movementModeLast, s.movementModeLast);
        num(m_in.speed, s.speed2dCmS);
        num(m_in.ttl, s.timeToLand);
        bo(m_in.starting, s.isStarting);
        bo(m_in.pivoting, s.isPivoting);
        bo(m_in.landLight, s.justLandedLight);
        bo(m_in.landHeavy, s.justLandedHeavy);
        bo(m_in.traversed, s.justTraversed);
        bo(m_in.tip, s.shouldTurnInPlace);
        bo(m_in.spin, s.shouldSpinTransition);
        num(m_in.lod, s.databaseLod);
    }

    // Get_MMInterruptMode (report 3.8).
    Interrupt InterruptFor(const ChooserState& s, const ChooserState& aLast) const
    {
        if (s.jumpOutLandOver)
            return Interrupt::InterruptOnDatabaseChange; // r3 landing: the capped heavy recovery of a jump-out landing ends
        if (s.movementMode != aLast.movementMode)
            return Interrupt::InterruptOnDatabaseChange;
        if (s.movementMode == 0 &&
            (s.movementState != aLast.movementState || s.gait != aLast.gait || s.stance != aLast.stance))
            return Interrupt::InterruptOnDatabaseChange;
        return Interrupt::DoNotInterrupt;
    }

    // Get_MMBlendTime (report 3.8); chooser mm_rules override when present.
    float BlendTimeFor(const ChooserState& s, const ChooserState& aLast) const
    {
        if (m_table->GetSlide().present && (s.movementMode == kMovementSliding || aLast.movementMode == kMovementSliding))
            return m_cfg.blendSlideExit >= 0.0f && s.movementMode == 0 && aLast.movementMode == kMovementSliding ? m_cfg.blendSlideExit : m_cfg.blendSlide;
        const chooser::BlendTimeRules& r = m_table->GetBlendTimeRules();
        if (r.valid)
            return chooser::GetBlendTime(r, s.movementMode, aLast.movementMode, s.verticalVelocityCmS, m_cfg.blendGroundToGround);
        if (s.movementMode == 0)
            return aLast.movementMode == 1 ? m_cfg.blendLanding : m_cfg.blendGroundToGround;
        return s.verticalVelocityCmS > m_cfg.airRisingThresholdCmS ? m_cfg.blendAirRising : m_cfg.blendAirOther;
    }

    // One update: advances the playing pose by aDt, evaluates the chooser, searches, and selects.
    // aTraj: trajectory samples (past + future, must include every schema sample time within its range; values are
    // linearly interpolated, clamped at the ends). aHist: optional pose history. aForce: optional interrupt override.
    const MatchResult& Update(float aDt, const TrajSample* aTraj, uint32_t aNumTraj, const ChooserState& aState,
                              const PoseHistory* aHist = nullptr, const Interrupt* aForce = nullptr)
    {
        MatchResult res;
        if (!m_ready || !aTraj || aNumTraj == 0)
        {
            m_last = res;
            return m_last;
        }
        // MatcherConfig::headingDipHoldS: the stick dip of a swing to a new direction keeps the gait of the playing clip
        if (UTPP_AND_NOT(JUMPPICK, 39) m_guardsOn && m_cfg.headingDipHoldS > 0.0f && aState.headingStepAgeS < m_cfg.headingDipHoldS && m_cur.valid && !aForce &&
            aState.gait < m_holdGait && aState.movementState == 1 && m_holdMovementState == 1 && aState.movementMode == 0 && aState.stance == 0 &&
            m_holdStance == 0)
        {
            ChooserState held = aState;
            held.gait = m_holdGait;
            return Update(aDt, aTraj, aNumTraj, held, aHist, aForce);
        }
        m_now += aDt;
        const bool holdWasActive = m_slideExitHoldLeft > 0.0f;
        m_slideExitHoldLeft = std::fmax(0.0f, m_slideExitHoldLeft - aDt);
        const bool holdExpiredNow = holdWasActive && m_slideExitHoldLeft <= 0.0f;
        m_slideExitAge += aDt;
        m_sinceJumpS += aDt;
        m_riseHoldLeft = std::fmax(0.0f, m_riseHoldLeft - aDt);
        m_sinceSlideS = aState.movementMode == kMovementSliding ? 0.0f : m_sinceSlideS + aDt;
        m_sinceFastS = aState.gait >= 1 || aState.speed2dCmS > m_cfg.walkTransitionFastCmS ? 0.0f : m_sinceFastS + aDt;
        m_sinceSlowS = aState.speed2dCmS <= m_cfg.runTransitionSlowCmS ? 0.0f : m_sinceSlowS + aDt;
        m_speedCmS = aState.speed2dCmS;
        m_traj = aTraj;
        m_numTraj = aNumTraj;
        m_hist = aHist;
        const MotionDatabase& db = *m_db;
        const float fps = db.Header().fps;
        UpdateStanceAdmission(aDt, aState);

        // ---- advance the continuing pose
        bool contValid = false;
        if (m_cur.valid)
        {
            // This adapter samples before evaluation; UE syncs from its asset player's
            // accumulated time before search. Advance by the previously selected rate,
            // then choose the next rate below. History/blend clocks retain real aDt.
            m_cur.clipTime += aDt * m_cur.playRate;
            const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(m_cur.clip));
            float t = m_cur.clipTime;
            const float len = static_cast<float>(cl.numFrames - 1) / fps;
            if (cl.flags & kClipLoop)
            {
                if (len > 0.0f)
                    t = std::fmod(t, len);
                m_cur.clipTime = t;
                contValid = true;
            }
            else
                contValid = t <= len + 1e-4f;
            if (!contValid && (db.Entry(m_cur.entry).flags & kEntryDisableReselection) &&
                steer::ClassifyClip(db.ClipName(static_cast<uint32_t>(m_cur.clip))) == steer::kClassStanceTransition)
            {
                m_completedNoReselectClip = m_cur.clip;
                m_completedNoReselectStance = aState.stance;
            }
#if defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 4 // mutation 4: a non-looping clip end never hands over
            if (!contValid)
            {
                m_cur.clipTime = len;
                m_last.clipTime = len;
                return m_last; // frozen on the last frame, no search
            }
#endif
            if (contValid)
            {
                const uint32_t lf = std::min(static_cast<uint32_t>(std::lround(t * fps)), cl.numFrames - 1);
                m_cur.frame = cl.firstFrame + lf;
            }
        }

        // fix/slide-exit (MatcherConfig::slideExitFamily): on the first frame after a slide the exit search only takes the clips of the
        // slide's own family (the continuing pose is the slide clip, still m_cur here)
        const bool exitFrame = contValid && m_cur.valid && m_cur.db >= 0 && m_dbSlide[static_cast<uint32_t>(m_cur.db)] &&
            aState.movementMode == 0 && aState.movementModeLast == kMovementSliding;
#if UTPP_SLIDEEXIT_MUTATION == 3
        m_slideExitSettled = exitFrame && m_cur.clipTime >= 1.1f && !m_clipSlideLoop[static_cast<uint32_t>(m_cur.clip)]; // mutation 3: clip time only
#elif UTPP_SLIDEEXIT_MUTATION == 10
        m_slideExitSettled = exitFrame && aState.slideLastDurationS >= m_cfg.slideExitSettledS &&
            (m_clipSlideLoop[static_cast<uint32_t>(m_cur.clip)] || m_cur.clipTime >= 1.2f); // mutation 10: no long-slide rule, a late phase threshold
#else
        m_slideExitSettled = exitFrame && aState.slideLastDurationS >= m_cfg.slideExitSettledS &&
            (aState.slideLastDurationS >= m_cfg.slideExitSettledLongS || m_clipSlideLoop[static_cast<uint32_t>(m_cur.clip)] ||
             m_cur.clipTime >= m_cfg.slideExitSettledPhaseS);
#endif
        m_slideExitFamily = m_cfg.slideExitFamily && m_slideExitSettled ? m_clipSlideFamily[static_cast<uint32_t>(m_cur.clip)] : 0;

        // ---- chooser
        chooser::Inputs in;
        FillInputs(aState, in);
        chooser::Output out;
        m_table->Evaluate(in, chooser::Mode::AllResults, out);
        uint32_t dbList[chooser::kMaxResults];
        uint32_t numDb = 0;
        for (uint32_t i = 0; i < out.count; ++i)
        {
            const int id = out.db[i];
            if (id < 0 || id >= static_cast<int>(m_table->NumDatabases()))
                continue;
            const int d = m_table->GetDatabase(static_cast<uint32_t>(id)).dbIndex;
            if (d < 0 || d >= static_cast<int>(db.NumDatabases()) || !db.DatabaseUsable(static_cast<uint32_t>(d)))
                continue;
            if (std::find(dbList, dbList + numDb, static_cast<uint32_t>(d)) == dbList + numDb)
            {
                dbList[numDb++] = static_cast<uint32_t>(d);
                res.dbMask |= uint64_t(1) << d;
            }
        }
        // G10 (MatcherConfig::runWalkLoopsMaxCmS): the Walk_Loops database of the same LOD joins the Run gait's searched set at walking pace
        m_runGaitWalkLoops = m_cfg.steadyRunLoops && m_cfg.runWalkLoopsMaxCmS > 0.0f && aState.gait == 1 && aState.stance == 0 &&
                             aState.movementMode == 0 && aState.movementState == 1 && !aState.isStarting && !aState.isPivoting &&
                             aState.speed2dCmS >= m_cfg.runWalkLoopsMinCmS && aState.speed2dCmS <= m_cfg.runWalkLoopsMaxCmS;
        if (m_runGaitWalkLoops)
        {
            const uint32_t n0 = numDb;
            for (uint32_t i = 0; i < n0; ++i)
            {
                const int32_t w = m_dbWalkSibling[dbList[i]];
                if (w >= 0 && numDb < chooser::kMaxResults && db.DatabaseUsable(static_cast<uint32_t>(w)) &&
                    std::find(dbList, dbList + numDb, static_cast<uint32_t>(w)) == dbList + numDb)
                {
                    dbList[numDb++] = static_cast<uint32_t>(w);
                    res.dbMask |= uint64_t(1) << w;
                }
            }
        }
        // Slide exit (motion: rows with MovementMode_LastFrame == Sliding): the frame after the slide only the ExitTo* databases
        // are searched, so the exit clip is chosen against the slide pose instead of losing to the ordinary locomotion rows; the
        // following frames keep it through the continuing pose (DoNotInterrupt, strong continuing bias of the exit databases).
        if (m_table->GetSlide().present && aState.movementMode == 0 && aState.movementModeLast == kMovementSliding)
        {
            uint32_t k = 0;
            uint64_t mask = 0;
            for (uint32_t i = 0; i < numDb; ++i)
                if (db.DbName(dbList[i]).starts_with("PSD_Slide_ExitTo"))
                {
                    dbList[k++] = dbList[i];
                    mask |= uint64_t(1) << dbList[i];
                }
            if (k)
            {
                numDb = k;
                res.dbMask = mask;
            }
        }
        res.numDatabases = numDb;

        // Reset() intentionally forgets motion history across a yield. Keep a completed one-shot transition suppressed
        // until the caller actually requests another stance, even when no previous state is available.
        if (m_completedNoReselectClip >= 0 && aState.stance != m_completedNoReselectStance)
        {
            m_completedNoReselectClip = -1;
            m_completedNoReselectStance = -1;
        }
        const ChooserState& last = m_haveLastState ? m_lastState : aState;
        const auto cancellationCurrent = CancellationState(aState);
        res.cancelledStartSupportAllowed = cancelstart::SupportAllowed(cancellationCurrent);
        const bool stanceChanged = m_haveLastState && aState.stance != m_lastState.stance;
        if (stanceChanged)
        {
            m_completedNoReselectClip = -1; // a fresh stance transition may legitimately use the clip again
            m_completedNoReselectStance = -1;
        }
        res.interrupt = aForce ? *aForce : InterruptFor(aState, last);
        // fix/slide-exit (MatcherConfig::slideExitMinHoldS): the selected exit clip plays on for its hold time. A stance / gait /
        // state change (game PSM flicker Crouch 3 <-> CrouchSprint 30 right after a slide) would interrupt it ("InterruptOnDatabaseChange",
        // the exit database is no longer a chooser database) and a closer row of an ordinary database would replace it after 0.07-0.10 s.
        // Air, a new slide, a landing, a traversal or a forced interrupt release it at once.
        const bool gaitStateDiffers = aState.gait != m_holdGait || aState.movementState != m_holdMovementState;
        m_holdChangeAge = gaitStateDiffers ? m_holdChangeAge + aDt : 0.0f;
        bool holdReleasedNow = false;
        if (m_slideExitHoldLeft > 0.0f)
        {
#if UTPP_SLIDEEXIT_MUTATION == 4
            const bool release = aState.movementMode != 0;
#elif UTPP_SLIDEEXIT_MUTATION == 6
            const bool release = aState.movementMode != 0 || aState.stance != m_holdStance || gaitStateDiffers || (HoldFlags(aState) & ~m_holdFlags) != 0;
#elif UTPP_SLIDEEXIT_MUTATION == 7
            const bool release = aState.movementMode != 0 || aState.stance != m_holdStance ||
                                 (gaitStateDiffers && m_holdChangeAge >= m_cfg.slideExitYieldS) || aState.stationaryAimTurn || aState.stationaryShotTurn;
#elif UTPP_SLIDEEXIT_MUTATION == 11
            const bool release = aState.movementMode != 0 || aState.stance != m_holdStance || m_holdChangeAge >= m_cfg.slideExitYieldS ||
                                 (HoldFlags(aState) & ~m_holdFlags) != 0;
#else
            // slideExitYieldS 0 = at once on a difference; a flag set when the clip was selected is armed again once it has dropped
            const bool release = aState.movementMode != 0 || aState.stance != m_holdStance ||
                                 (gaitStateDiffers && m_holdChangeAge >= m_cfg.slideExitYieldS) || (HoldFlags(aState) & ~m_holdFlags) != 0;
#endif
            if (release)
            {
                m_slideExitHoldLeft = 0.0f;
                holdReleasedNow = aState.movementMode == 0;
            }
#if UTPP_SLIDEEXIT_MUTATION != 12
            else
                m_holdFlags &= HoldFlags(aState); // a flag that dropped during the hold re-arms: its next START releases
#endif
        }
        const bool slideExitHold = m_cfg.slideExitMinHoldS > 0.0f && m_slideExitHoldLeft > 0.0f && contValid && m_cur.valid &&
            m_cur.db >= 0 && m_dbSlideExit[static_cast<uint32_t>(m_cur.db)] && !aForce && !aState.justTraversed &&
            !aState.justLandedLight && !aState.justLandedHeavy;
        if (slideExitHold)
            res.interrupt = Interrupt::DoNotInterrupt;
        else if ((holdReleasedNow || (holdExpiredNow && gaitStateDiffers)) && !aForce && res.interrupt == Interrupt::DoNotInterrupt &&
                 m_cur.valid && m_cur.db >= 0 && m_dbSlideExit[static_cast<uint32_t>(m_cur.db)])
            res.interrupt = Interrupt::InterruptOnDatabaseChange; // the deferred / swallowed edge: the exit clip leaves the search now
        // MatcherConfig::turnCommit: the selected turn clip plays on through its authored root turn; gait / movement state edges do not interrupt it
        bool turnHold = false;
        if (m_turnHoldClip >= 0)
        {
            if (!aState.isPivoting)
                m_turnHoldPivotDropped = true;
            // turnCommitTailS: past the turn end the hold lasts a little longer unless a heading step came after the pick
            const bool turnDone = m_cur.valid && m_cur.clip == m_turnHoldClip && m_cur.clipTime >= m_clipTurnEndS[static_cast<uint32_t>(m_cur.clip)];
            m_turnTailS = turnDone ? m_turnTailS + aDt : 0.0f;
            const bool inTurn = !turnDone ||
                (UTPP_AND_NOT(JUMPPICK, 34) m_guardsOn && m_turnTailS <= m_cfg.turnCommitTailS && aState.headingStepAgeS + 0.05f >= m_sinceJumpS);
            const bool keep = m_cfg.turnCommit && contValid && m_cur.valid && m_cur.clip == m_turnHoldClip && !aForce &&
                              aState.movementMode == 0 && aState.stance == m_turnHoldStance && !aState.justLandedLight &&
                              !aState.justLandedHeavy && !aState.justTraversed && !(aState.isPivoting && m_turnHoldPivotDropped) &&
                              aState.movementState != 0 && (HoldFlags(aState) & ~4 & ~m_turnHoldFlags) == 0 && inTurn;
            if (keep)
            {
                turnHold = true;
                res.interrupt = Interrupt::DoNotInterrupt;
            }
            else
            {
                m_turnHoldClip = -1;
                // the gait / movement state edge swallowed during the commit is delivered once
                if (gaitStateDiffers && !aForce && res.interrupt == Interrupt::DoNotInterrupt && aState.movementMode == 0)
                    res.interrupt = Interrupt::InterruptOnDatabaseChange;
            }
        }
        // churn guard 4 (MatcherConfig::startUpshiftHoldS): a gait upshift during a young start is not an interrupting edge for a short time;
        // when the time is up the deferred edge re-searches (a held DoNotInterrupt would keep the outdated start competing for its whole length)
        if (UTPP_AND_NOT(JUMPPICK, 28) m_guardsOn && m_cfg.startUpshiftHoldS > 0.0f && contValid && m_cur.valid && m_clipStart[static_cast<uint32_t>(m_cur.clip)] != 0 && !aForce &&
            (m_clipStart[static_cast<uint32_t>(m_cur.clip)] == 2 || m_holdGait == 0) && aState.gait > m_holdGait && aState.movementState == m_holdMovementState && aState.movementMode == 0 && last.movementMode == 0 && aState.stance == 0 &&
            last.stance == 0 && !aState.jumpOutLandOver && !aState.justLandedLight && !aState.justLandedHeavy && !aState.justTraversed &&
            !aState.stationaryAimTurn && !aState.stationaryShotTurn && !aState.restTurn && !slideExitHold && m_slideExitHoldLeft <= 0.0f)
        {
            // a reface start is held much shorter than a plain start (startUpshiftRefaceWalkHoldS / startUpshiftRefaceRunHoldS)
            const float holdS = m_clipStart[static_cast<uint32_t>(m_cur.clip)] == 2 ?
                UTPP_TN_JUMPPICK(29, 0.35f, (m_holdGait >= 1 ? m_cfg.startUpshiftRefaceRunHoldS : m_cfg.startUpshiftRefaceWalkHoldS)) :
                m_cfg.startUpshiftHoldS;
            if (m_sinceJumpS < holdS)
            {
                if (res.interrupt == Interrupt::InterruptOnDatabaseChange)
                    res.interrupt = Interrupt::DoNotInterrupt;
            }
            else if (!m_upshiftEdgeDone && res.interrupt == Interrupt::DoNotInterrupt)
            {
                res.interrupt = Interrupt::InterruptOnDatabaseChange; // the deferred edge, once
                m_upshiftEdgeDone = true;
            }
        }
        // MatcherConfig::downshiftDeferS: a young gait downshift while moving keeps the continuing pose (the edge is delivered once afterwards)
        const bool downshiftYoung = UTPP_AND_NOT(JUMPPICK, 37) m_guardsOn && m_cfg.downshiftDeferS > 0.0f && contValid && m_cur.valid && !aForce &&
            aState.gait < m_holdGait && aState.movementState == 1 && aState.movementState == m_holdMovementState && aState.movementMode == 0 &&
            last.movementMode == 0 && aState.stance == 0 && last.stance == 0 && !slideExitHold && m_slideExitHoldLeft <= 0.0f;
        if (downshiftYoung && m_holdChangeAge < m_cfg.downshiftDeferS)
        {
            if (res.interrupt == Interrupt::InterruptOnDatabaseChange)
                res.interrupt = Interrupt::DoNotInterrupt;
            m_downshiftDeferred = true;
        }
        else if (downshiftYoung && m_downshiftDeferred && res.interrupt == Interrupt::DoNotInterrupt)
        {
            res.interrupt = Interrupt::InterruptOnDatabaseChange;
            m_downshiftDeferred = false;
        }
        else if (!downshiftYoung)
            m_downshiftDeferred = false;
        // churn guard 2 (MatcherConfig::freshEventS): the young event clip keeps competing with a cost advantage while no stick edge arrived
        const bool freshEvent = UTPP_AND_NOT(JUMPPICK, 25) m_guardsOn && m_cfg.freshEventS > 0.0f && m_cfg.freshEventBias > 0.0f && contValid && m_cur.valid &&
            m_sinceJumpS < m_cfg.freshEventS && !(db.Clip(static_cast<uint32_t>(m_cur.clip)).flags & kClipLoop) && !m_clipStop[static_cast<uint32_t>(m_cur.clip)] &&
            aState.stance == 0 && !aForce &&
            res.interrupt == Interrupt::DoNotInterrupt && aState.movementMode == 0 && last.movementMode == 0 && aState.stance == last.stance &&
            !aState.jumpOutLandOver && !aState.justLandedLight && !aState.justLandedHeavy && !aState.justTraversed && !aState.stationaryAimTurn &&
            !aState.stationaryShotTurn && !aState.restTurn && !slideExitHold && m_slideExitHoldLeft <= 0.0f;
        const int startTurnDirection = aState.movementMode == 0 && aState.movementState == 1 &&
            aState.isStarting && !aState.stationaryShotTurn &&
            (aState.startTurnDirection == 1 || aState.startTurnDirection == -1) ?
            aState.startTurnDirection : 0;
        float blendTime = BlendTimeFor(aState, last);
        if (aState.movementMode == 0 && last.movementMode == kMovementSliding && !m_slideExitSettled && m_cfg.blendSlideExit >= 0.0f)
            blendTime = m_cfg.blendSlideShortExit > 0.0f ? m_cfg.blendSlideShortExit : m_cfg.blendSlide; // an unsettled slide keeps the entry cross-fade time
        // fix3/jump (blendSlideAir): a take-off just after a slide (the lying / rising pose is still on display)
        // mutation 4: the take-off after a slide keeps the 0.2 / 0.15 s fades
        if (UTPP_AND_NOT(JUMPPICK, 4) aState.movementMode == 1 && m_cfg.blendSlideAir > 0.0f && m_sinceSlideS < m_cfg.slideAirWindowS)
            blendTime = std::fmax(blendTime, m_cfg.blendSlideAir);
        const bool allowLastPose = m_cur.valid &&
            !IsInvalidating(res.interrupt, (res.dbMask >> static_cast<uint32_t>(m_cur.db)) & 1u);

        // ---- continuing pose searches (UE 5.8 UPoseSearchLibrary::MotionMatch, PoseSearchLibrary.cpp 1340-1373;
        // PopulateContinuingPoseSearches 276-300; IsForceInterrupt 326-344; IsInvalidatingContinuingPose 346-365).
        // The playing clip is looked up in the playing database and in every chooser database that indexes the same
        // clip at the current frame. Per database c (in = c is a chooser database):
        //   invalidated (no continuing pose for c, query pose channels from pose history) when
        //     InterruptOnDatabaseChangeAndInvalidateContinuingPose && !in, or ForceInterruptAndInvalidateContinuingPose;
        //   else cached (query pose channels of every database with c's schema come from it; DISABLE_RESELECTION rows
        //     of the clip excluded in c), and its continuing cost is evaluated unless force-interrupted:
        //     InterruptOnDatabaseChange* && !in, or ForceInterrupt*.
        // So DoNotInterrupt (motion every frame without a state change) keeps competing with a continuing clip whose
        // database the chooser no longer returns; InterruptOnDatabaseChange (motion on a MovementMode change or a
        // ground MovementState / Gait / Stance change) drops it from the search but still uses it to build the query.
        m_numCand = 0;
        // G10 (MatcherConfig::runWalkLoopsHoldCmS): a Walk_Loops pose that DoNotInterrupt would keep in the Run gait is not evaluated above the cap
        const bool dropWalkCont = m_cfg.steadyRunLoops && m_cfg.runWalkLoopsMaxCmS > 0.0f && aState.gait == 1 && aState.stance == 0 &&
                                  aState.speed2dCmS > m_cfg.runWalkLoopsMaxCmS + m_cfg.runWalkLoopsHoldCmS;
        if (contValid)
        {
            const auto addCand = [&](uint32_t c) {
                for (uint32_t k = 0; k < m_numCand; ++k)
                    if (m_cand[k].db == static_cast<int32_t>(c))
                        return;
                const bool in = (res.dbMask >> c) & 1u;
                if (IsInvalidating(res.interrupt, in))
                    return;
                const DatabaseRecord& C = db.Db(c);
                for (uint32_t e = C.firstEntry; e < C.firstEntry + C.numEntries; ++e)
                {
                    if (db.Entry(e).clip != static_cast<uint32_t>(m_cur.clip))
                        continue;
                    const int32_t row = FindRow(c, e, m_cur.frame);
                    if (row < 0)
                        continue;
                    ContCand& k = m_cand[m_numCand++];
                    k.db = static_cast<int32_t>(c);
                    k.localRow = row;
                    k.entry = e;
                    k.searched = !IsForceInterrupt(res.interrupt, in) && !(dropWalkCont && m_dbStandWalkLoops[c]);
                    return;
                }
            };
            addCand(static_cast<uint32_t>(m_cur.db));
            for (uint32_t i = 0; i < numDb; ++i)
                addCand(dbList[i]);
        }
        m_candActive = true;

        // RC2 (MatcherConfig::slideEntryMaxS): the first Sliding update after a non-slide clip searches the early Into rows only (MarkExcluded)
        const bool curIsSlide = m_cur.valid && m_cur.db >= 0 && m_dbSlide[static_cast<uint32_t>(m_cur.db)];
#if UTPP_LOCOMATCHER_MUTATION == 2 // mutation 2: no slide entry cap
        m_slideEntryCap = false;
#else
        m_slideEntryCap = m_cfg.slideEntryMaxS > 0.0f && m_table->GetSlide().present && aState.movementMode == kMovementSliding &&
                          last.movementMode != kMovementSliding && !curIsSlide;
#endif
        // RC3 (MatcherConfig::slideIntoHandover): an Into clip past its indexed range (no continuing row) hands over to the Loop clip of its family,
        // only when its database is searched on this update (the Loop sits in the same database; nothing else is excluded otherwise)
        m_slideHandoverDb = -1;
#if UTPP_LOCOMATCHER_MUTATION != 4 // mutation 4: no Into -> Loop handover (the mirrored Into re-jump)
        if (m_cfg.slideIntoHandover && aState.movementMode == kMovementSliding && contValid && curIsSlide && m_numCand == 0 &&
            m_clipSlideInto[static_cast<uint32_t>(m_cur.clip)] && ((res.dbMask >> static_cast<uint32_t>(m_cur.db)) & 1u))
        {
            const DatabaseRecord& C = db.Db(static_cast<uint32_t>(m_cur.db));
            for (uint32_t e = C.firstEntry; e < C.firstEntry + C.numEntries; ++e)
                if (m_clipSlideLoop[db.Entry(e).clip] && db.Entry(e).numRows &&
                    m_clipSlideFamily[db.Entry(e).clip] == m_clipSlideFamily[static_cast<uint32_t>(m_cur.clip)])
                    m_slideHandoverDb = m_cur.db;
        }
#endif

        // motionfix item 5: standing-jump fall bias (MatcherConfig::airFallBias)
#if UTPP_MOTION_AIRFALL_MUTATION == 1
        const bool fallBias = false; // mutation 1: never applied
#elif UTPP_MOTION_AIRFALL_MUTATION == 2
        const bool fallBias = m_anyFallClip && m_cfg.airFallBias < 0.0f && aState.movementMode == 1 &&
                              aState.speed2dCmS < m_cfg.airFallMaxSpeedCmS; // mutation 2: also before the apex
#else
        const bool fallBias = m_anyFallClip && m_cfg.airFallBias < 0.0f && aState.movementMode == 1 &&
                              aState.verticalVelocityCmS <= 0.0f && aState.speed2dCmS < m_cfg.airFallMaxSpeedCmS;
#endif
        res.airFallBias = fallBias;
        // fix3/jump (MatcherConfig::standingJumpMaxSpeedCmS / standingJumpLandBlockS): the new candidates of a standing jump
        if (aState.movementMode != 1)
            m_flightActive = false;
        else
        {
            // mutation 10: the decision of the first InAir update holds for the whole flight, re-jumps included
            const bool rejump = UTPP_AND_NOT(JUMPPICK, 10) m_flightActive && m_haveLastState && last.movementMode == 1 &&
                                (aState.verticalVelocityCmS - last.verticalVelocityCmS > kStandingJumpRejumpDvzCmS ||
                                 aState.speed2dCmS - last.speed2dCmS > kStandingJumpRejumpDSpeedCmS);
            if (!m_flightActive || rejump)
            {
                const bool newFlight = !m_flightActive; // a re-jump inside a flight keeps the jump-out latch
                m_flightActive = true;
                m_flightAge = 0.0f;
                m_flightStanding = aState.movementState == 0 && aState.speed2dCmS < m_cfg.standingJumpMaxSpeedCmS;
                m_flightJump = aState.psmJump;
                m_flightFoot = 0; // taken from the clip this update picks (next update)
                // review r2 finding 15: the jump-out latch lives for the FLIGHT (reset when a new flight starts, not on a re-jump): the exit window can close
                // mid-flight (gate 0 and weight 1) and a bump of the capsule against the exiting car reads as a re-jump; a rebuilt-from-the-flag latch
                // would drop jumpOutNoBack (JO1 failure) for the rest of the flight. A real second jump in the air keeps the rules of the flight too
                // (a double jump out of a vehicle stays a jump-out flight until the touchdown).
                UTPP_IF_MUT(JUMPPICK, 19)
                {
                    m_flightJumpOut = false; // mutation 19: every re-jump rebuilds the latch from the window flag (pre-fix)
                    m_jumpOutFallLatched = false;
                }
                else if (newFlight)
                {
                    m_flightJumpOut = false; // taken from this update below (a vehicle jump-out launch is no re-jump of an earlier flight)
                    m_jumpOutFallLatched = false;
                }
            }
            else
            {
                m_flightAge += aDt;
                if (m_flightFoot == 0 && m_cur.clip >= 0)
                    m_flightFoot = m_clipJumpFoot[static_cast<uint32_t>(m_cur.clip)];
                UTPP_IF_NOT_MUT(JUMPPICK, 13)
                    m_flightJump = m_flightJump || aState.psmJump; // the PSM jump may arrive one feed after the first InAir update
            }
            UTPP_IF_MUT(JUMPPICK, 13)
                m_flightJump = true; // mutation 13: every flight counts as a game jump, walk-offs included
            UTPP_IF_MUT(JUMPPICK, 17)
                m_flightJumpOut = true; // mutation 17: every flight is a vehicle jump-out flight (the rules leak into jumps and walk-offs)
            else
                m_flightJumpOut = m_flightJumpOut || aState.jumpOutFlight; // the exit window flag may arrive one feed after the first InAir update
        }
        bool standing = m_cfg.standingJumpMaxSpeedCmS > 0.0f && aState.movementMode == 1 && m_flightStanding;
        UTPP_IF_MUT(JUMPPICK, 3)
            standing = m_cfg.standingJumpMaxSpeedCmS > 0.0f && aState.movementMode == 1; // mutation 3: every jump, moving ones too
        else UTPP_IF_MUT(JUMPPICK, 8)
            standing = m_cfg.standingJumpMaxSpeedCmS > 0.0f && aState.movementMode == 1 && aState.movementState == 0 &&
                       aState.speed2dCmS < m_cfg.standingJumpMaxSpeedCmS; // mutation 8: decided again on every update (stick touch in the air)
        m_standJumpBack = standing && m_cfg.standingJumpExcludeBack;
        m_airFootLock = aState.movementMode == 1 && m_flightActive && !m_flightJump && m_flightFoot != 0 && m_flightAge < m_cfg.airKeepLeadFootS;
        UTPP_IF_MUT(JUMPPICK, 1)
            m_standJumpBack = false; // mutation 1: B / LL / RL Start rows stay selectable
        m_standJumpTakeoff = standing && aState.timeToLand < m_cfg.standingJumpLandBlockS;
        UTPP_IF_MUT(JUMPPICK, 2)
            m_standJumpTakeoff = false; // mutation 2: take-off-phase rows stay selectable before touchdown
        m_standJumpOff = standing && m_flightAge < m_cfg.standingJumpOffBlockS;
        UTPP_IF_MUT(JUMPPICK, 7)
            m_standJumpOff = false; // mutation 7: the run-off clips stay selectable on the take-off update
        // fix/jumpout-cam (MatcherConfig::jumpOutNoBack / jumpOutFallHandover): the flight of a vehicle jump-out
        const bool jumpOutNow = aState.movementMode == 1 && m_flightActive && m_flightJumpOut;
        if (jumpOutNow && aState.verticalVelocityCmS <= 0.0f)
            m_jumpOutFallLatched = true;
        m_jumpOutBack = m_cfg.jumpOutNoBack && jumpOutNow;
        m_jumpOutFall = m_cfg.jumpOutFallHandover && m_anyFallClip && jumpOutNow && m_jumpOutFallLatched;
        // r5 air (game test r4-jump-cycle, jump_move): the near jump database (PSD_Dense_Jumps, the fall loop) is in every chooser result, and PSD_Dense_Jumps_Far
        // (no fall clip) joins it while the floor is unknown (TimeToLand >= 1.2 s: a jump-out over open ground). The rule below only dropped the Start / Off rows
        // of databases that index a fall clip themselves, so the sprint Start clip of the Far database played through the whole descent. With the fall loop in
        // the result, no database of the result keeps a Start / Off row.
        m_jumpOutFallMask = false;
        // mutation 23: only databases that index a fall clip themselves drop their Start / Off rows (pre-r5)
        if (UTPP_AND_NOT(JUMPPICK, 23) m_jumpOutFall)
            for (uint32_t i = 0; i < numDb; ++i)
                m_jumpOutFallMask = m_jumpOutFallMask || m_dbHasFall[dbList[i]] != 0;
        UTPP_IF_MUT(JUMPPICK, 15)
            m_jumpOutBack = false; // mutation 15: the B / LL / RL Start rows stay selectable in a jump-out flight
        UTPP_IF_MUT(JUMPPICK, 16)
            m_jumpOutFall = false; // mutation 16: no fall-loop hand-over (the Start clip is held through the descent)
        // fix5/jump (MatcherConfig::jumpRiseNoOff): a jump in motion neither takes nor continues an F Off row from its first airborne update to the apex
        // (not the take-off update itself: the previous update was InAir already; the impulse update counts as a re-jump, m_flightAge 0 again)
        m_jumpRiseOff = m_cfg.jumpRiseNoOff && aState.movementMode == 1 && m_flightActive && m_flightJump && !standing && m_haveLastState &&
                        last.movementMode == 1 && aState.verticalVelocityCmS > 0.0f;
        UTPP_IF_MUT(JUMPPICK, 14)
            m_jumpRiseOff = m_jumpRiseOff && m_flightAge == 0.0f; // mutation 14: only on the first airborne update (the impulse), not through the rise
        // mutation 12: the continuing F Off clip of the take-off update stays selectable in the rise
        const bool contOffOut = UTPP_AND_NOT(JUMPPICK, 12) m_jumpRiseOff && m_cur.clip >= 0 && m_clipJumpKind[static_cast<uint32_t>(m_cur.clip)] == 3;
        // fix/jumpout-cam: the Start / Off clip on display does not continue past the apex of a jump-out flight (database with a fall clip only)
        const bool contJumpOutOut = m_jumpOutFall && m_cur.clip >= 0 && m_cur.db >= 0 && m_clipJumpKind[static_cast<uint32_t>(m_cur.clip)] != 0 &&
                                    (m_dbHasFall[static_cast<uint32_t>(m_cur.db)] || m_jumpOutFallMask);
        float bestCost = std::numeric_limits<float>::max();
        int32_t bestDb = -1, bestRow = -1;
        uint32_t bestEntry = 0;
        bool bestIsCont = false;
        float contBestCost = std::numeric_limits<float>::max(); // the best continuing candidate (churn guard 1)
        int32_t contDb = -1, contRow = -1;
        uint32_t contEntry = 0;
        for (uint32_t k = 0; k < m_numCand; ++k)
        {
            const ContCand& cc = m_cand[k];
            if (!cc.searched || contOffOut || contJumpOutOut)
                continue;
            const int continuingDirection = StartTurnDirection(m_cur.clip);
            if (startTurnDirection && continuingDirection && continuingDirection != startTurnDirection)
                continue;
            const uint32_t d = static_cast<uint32_t>(cc.db);
            BuildQuery(d, true, allowLastPose);
            // ST-1 (MatcherConfig::slowWalkStartTail): a walk Start clip past its acceleration that the chooser dropped and that cannot match the
            // capsule speed within the play rate range is not evaluated (it stays the source of the query pose channels)
#if UTPP_LOCOMATCHER_MUTATION != 6 // mutation 6: the Start tail keeps competing (UE DoNotInterrupt)
            if (m_cfg.slowWalkStartTail && m_cfg.startTailAccelFraction > 0.0f && aState.movementMode == 0 && aState.movementState == 1 &&
                aState.gait == 0 && !aState.isStarting && m_dbWalkStarts[d] && !((res.dbMask >> d) & 1u) &&
                m_clipAccelEndS[static_cast<uint32_t>(m_cur.clip)] >= 0.0f && m_cur.clipTime >= m_clipAccelEndS[static_cast<uint32_t>(m_cur.clip)])
            {
                const auto tail = rate2::Calculate(db, d, static_cast<uint32_t>(cc.localRow), m_raw.data(), m_cfg.minPlayRate, m_cfg.maxPlayRate);
                if (tail.valid && tail.ratio < m_cfg.minPlayRate)
                    continue;
            }
#endif
            const RowRecord& rr = db.Row(db.Db(d).firstRow + static_cast<uint32_t>(cc.localRow));
            // fix/slide-exit (MatcherConfig::slideExitBiasS): an exit clip older than that competes without its addends
            const bool exitBiasOver = m_cfg.slideExitBiasS > 0.0f && m_dbSlideExit[d] && m_slideExitAge > m_cfg.slideExitBiasS;
            const float c = db.RowCost(d, static_cast<uint32_t>(cc.localRow), m_query.data()) +
                            (exitBiasOver ? 0.0f : rr.costAddend + rr.continuingAddend) +
                            (fallBias && m_fallClip[static_cast<uint32_t>(m_cur.clip)] ? m_cfg.airFallBias : 0.0f) -
                            (freshEvent ? m_cfg.freshEventBias : 0.0f);
            if (c < res.continuingCost)
                res.continuingCost = c;
            if (std::isfinite(c) && c < contBestCost)
            {
                contBestCost = c;
                contDb = cc.db;
                contRow = cc.localRow;
                contEntry = cc.entry;
            }
            if (std::isfinite(c) && c < bestCost)
            {
                bestCost = c;
                bestDb = cc.db;
                bestRow = cc.localRow;
                bestEntry = cc.entry;
                bestIsCont = true;
            }
        }
        const bool contEvaluated = bestIsCont;

        // A shot's full-facing query changes as its authored root turns. Searching
        // again halfway through a 180 can replace it with a 90/45 and discard the
        // remaining footwork. Finish this admitted clip while its side is still
        // requested; movement, ADS, stance/air changes and goal reversal release it.
        bool continueShotTurn = false;
        if (contEvaluated && aState.stationaryShotTurn && aState.shouldTurnInPlace &&
            aState.movementMode == 0 && aState.movementState == 0 &&
            ((res.dbMask >> static_cast<uint32_t>(bestDb)) & 1u))
        {
            const auto name = db.ClipName(static_cast<uint32_t>(m_cur.clip));
            continueShotTurn = steer::ClassifyClip(name) == steer::kClassTurnInPlace &&
                name.ends_with(aState.shotTurnYawDeg > 0 ? "_L" : "_R");
        }

        // Stationary ADS/RMB has the same authored footwork requirement as a
        // shot, but follows the live view rather than a held shot goal. Finish
        // an admitted same-side TIP only while that view and stance are stable.
        // A changed goal returns to normal search even if it stays on the same
        // side, so a large turn is not blindly retained for a smaller new aim.
        const bool stationaryAim = (aState.stationaryAimTurn || aState.restTurn) && aState.shouldTurnInPlace &&
            aState.movementMode == 0 && aState.movementState == 0 &&
            last.movementMode == 0 && aState.stance == last.stance &&
            !aState.justLandedLight && !aState.justLandedHeavy && !aState.justTraversed &&
            std::isfinite(aState.stationaryAimYawDeg) && std::isfinite(aState.stationaryAimTargetYawDeg);
        // Compare with the goal accepted by this player, not the preceding
        // frame: a slowly moving view must not bypass cancellation at high FPS.
        if (!stationaryAim || !(last.stationaryAimTurn || last.restTurn) || res.interrupt != Interrupt::DoNotInterrupt ||
            !std::isfinite(m_cur.stationaryAimGoal) ||
            std::fabs(std::remainder(aState.stationaryAimTargetYawDeg - m_cur.stationaryAimGoal, 360.f)) > .1f)
            m_cur.stationaryAimGoalValid = false;
        bool continueAimTurn = false;
        if (contEvaluated && stationaryAim && m_cur.stationaryAimGoalValid &&
            ((res.dbMask >> static_cast<uint32_t>(bestDb)) & 1u))
        {
            const auto name = db.ClipName(static_cast<uint32_t>(m_cur.clip));
            continueAimTurn = steer::ClassifyClip(name) == steer::kClassTurnInPlace &&
                name.ends_with(aState.stationaryAimYawDeg > 0 ? "_L" : "_R");
        }

        // Preserve the admitted reface's footwork while the caller still owns
        // this exact start and direction. A changed state/forced interrupt,
        // chooser removal, exhausted indexed range or released intent restores
        // ordinary search. This is not a hold on generic starts or run loops.
        const bool continueStartTurn = contEvaluated && startTurnDirection &&
            res.interrupt == Interrupt::DoNotInterrupt &&
            aState.committedStartTurnClip == m_cur.clip &&
            StartTurnDirection(m_cur.clip) == startTurnDirection &&
            ((res.dbMask >> static_cast<uint32_t>(bestDb)) & 1u);

        // fix/slide-exit: the held exit clip keeps playing, no other database is searched during its hold
        const bool continueSlideExit = slideExitHold && contEvaluated;
        // CR-4 (MatcherConfig::crouchToStandHoldS): the selected Crouch -> Stand clip keeps playing through its rise while the character stays at
        // rest in Stand on the ground; anything else releases the hold for good
        bool continueRise = false;
        if (m_riseHoldLeft > 0.0f)
        {
#if UTPP_LOCOMATCHER_MUTATION == 5 // mutation 5: no hold (the selected transition competes on cost from its next update on)
            const bool keep = false;
#else
            const bool keep = contValid && m_cur.valid && m_cur.clip == m_riseHoldClip && !aForce && aState.movementMode == 0 &&
                              aState.movementState == 0 && aState.stance == 0 && !aState.justLandedLight && !aState.justLandedHeavy &&
                              !aState.justTraversed && (HoldFlags(aState) & ~m_riseHoldFlags) == 0;
#endif
            if (keep)
                continueRise = contEvaluated;
            else
                m_riseHoldLeft = 0.0f;
        }

        // MatcherConfig::exitTransitionTailS: a selected VX_ exit transition plays on while the character stays at rest, up to its tail
        bool continueExitClip = false;
        if (m_exitHoldClip >= 0)
        {
            const float exitLen = m_cur.valid && m_cur.clip == m_exitHoldClip ?
                static_cast<float>(db.Clip(static_cast<uint32_t>(m_cur.clip)).numFrames - 1) / fps : 0.0f;
            const bool keep = contValid && m_cur.valid && m_cur.clip == m_exitHoldClip && !aForce && aState.movementMode == 0 &&
                              aState.movementState == 0 && aState.speed2dCmS <= m_cfg.exitTransitionMaxSpeedCmS &&
                              aState.stance == m_exitHoldStance && !aState.justLandedLight && !aState.justLandedHeavy &&
                              !aState.justTraversed && (HoldFlags(aState) & ~m_exitHoldFlags) == 0 &&
                              m_cur.clipTime < exitLen - m_cfg.exitTransitionTailS;
            if (keep)
                continueExitClip = contEvaluated;
            else
                m_exitHoldClip = -1;
        }

        // MatcherConfig::stopCommitTailS: a Stop clip selected with the stick released plays on while the stick stays released, up to its tail
        bool continueStopClip = false;
        if (m_stopHoldClip >= 0)
        {
#if UTPP_LOCOMATCHER_MUTATION == 7 // mutation 7: no stop commit (the selected stop competes on cost from its next update on)
            const bool keep = false;
#else
            const float stopLen = m_cur.valid && m_cur.clip == m_stopHoldClip ?
                static_cast<float>(db.Clip(static_cast<uint32_t>(m_cur.clip)).numFrames - 1) / fps : 0.0f;
            const bool keep = contValid && m_cur.valid && m_cur.clip == m_stopHoldClip && !aForce && aState.movementMode == 0 &&
                              aState.movementState == 0 && aState.stance == m_stopHoldStance && !aState.justLandedLight &&
                              !aState.justLandedHeavy && !aState.justTraversed && (HoldFlags(aState) & ~m_stopHoldFlags) == 0 &&
                              m_cur.clipTime < std::min(stopLen - m_cfg.stopCommitTailS, m_stopHoldStart + m_cfg.stopCommitMaxS);
#endif
            if (keep)
                continueStopClip = contEvaluated;
            else
                m_stopHoldClip = -1;
        }

        // churn guard 3 (MatcherConfig::dodgeHold): the playing clip is kept through the game's ground Dodge
        const bool continueDodge = UTPP_AND_NOT(JUMPPICK, 26) m_guardsOn && m_cfg.dodgeHold && contEvaluated && aState.dodge && aState.movementMode == 0 &&
            last.movementMode == 0 && aState.stance == 0 && !aForce;
        const bool continueTurn = turnHold && contEvaluated;
        if (turnHold && !contEvaluated)
            m_turnHoldClip = -1;
        res.turnHold = continueTurn;
        // MatcherConfig::headingCatchUpS: after a digital heading step, until the facing has caught up, no clip that keeps its facing is a new candidate
        // mutation 33: no heading catch-up rule
        m_headingCatchUp = UTPP_AND_NOT(JUMPPICK, 33) m_guardsOn && m_cfg.headingCatchUpS > 0.0f && aState.headingStepAgeS < m_cfg.headingCatchUpS &&
                           aState.headingLagDeg > m_cfg.headingSettledDeg && aState.movementMode == 0 && aState.movementState == 1 &&
                           (aState.stance == 0 || aState.stance == 1) && !aForce && !aState.justLandedLight && !aState.justLandedHeavy && !aState.justTraversed &&
                           !aState.stationaryAimTurn && !aState.stationaryShotTurn && !aState.restTurn &&
                           (!aState.headingStepFromRest || m_headingRestArmed || (m_cur.valid && m_clipStart[static_cast<uint32_t>(m_cur.clip)] != 0));
        // a start from rest arms the rule while its start clip plays; it then stays on until the facing has caught up or the window ends
        m_headingRestArmed = m_headingCatchUp && aState.headingStepFromRest;
        // ---- search every chooser database
        for (uint32_t i = 0; i < numDb && !continueShotTurn && !continueAimTurn && !continueStartTurn && !continueSlideExit && !continueRise &&
                                  !continueExitClip && !continueStopClip && !continueDodge && !continueTurn; ++i)
        {
            const uint32_t d = dbList[i];
            BuildQuery(d, true, allowLastPose);
            const uint32_t marked = MarkExcluded(d, startTurnDirection);
            MotionSearchResult r = m_cfg.useAabb ? db.SearchAabb(d, m_query.data(), m_excluded.data())
                                             : db.SearchLinear(d, m_query.data(), m_excluded.data());
            if (fallBias) // exact: the biased rows (bias < 0) are re-evaluated with their addend, the rest keeps the bound search
                FallBiasSearch(d, r);
            ClearExcluded(d, marked);
            res.rowsEvaluated += r.rowsEvaluated;
            if (r.localRow >= 0 && r.cost < bestCost)
            {
                bestCost = r.cost;
                bestDb = static_cast<int32_t>(d);
                bestRow = r.localRow;
                bestIsCont = false;
            }
        }
        res.searched = true;

        // ---- select
        if (bestDb >= 0)
        {
            const DatabaseRecord& D = db.Db(static_cast<uint32_t>(bestDb));
            const RowRecord& rr = db.Row(D.firstRow + static_cast<uint32_t>(bestRow));
            const EntryRecord& en = db.Entry(rr.entry);
            const MotionClipRecord& cl = db.Clip(en.clip);
            const float newTime = static_cast<float>(rr.frame - cl.firstFrame) / fps;
            bool jump = !bestIsCont;
            if (jump && contEvaluated && m_cfg.poseJumpThresholdTime > 0.0f && static_cast<int32_t>(en.clip) == m_cur.clip &&
                std::fabs(newTime - m_cur.clipTime) < m_cfg.poseJumpThresholdTime)
            {
                jump = false; // keep playing the continuing pose
                for (uint32_t k = 0; k < m_numCand; ++k)
                    if (m_cand[k].searched)
                    {
                        bestDb = m_cand[k].db;
                        bestRow = m_cand[k].localRow;
                        bestEntry = m_cand[k].entry;
                        break;
                    }
            }
            // churn guard 7 (MatcherConfig::startRestartDwellS): a young start clip is not restarted from its first frame by the start of the other foot.
            // Guards 7 and 1 act only without a stick edge (DoNotInterrupt): a gait / movement state edge whose continuing database is still searched
            // re-picks freely, as the other guards do
            if (UTPP_AND_NOT(JUMPPICK, 30) jump && m_guardsOn && res.interrupt == Interrupt::DoNotInterrupt && m_cfg.startRestartDwellS > 0.0f && contDb >= 0 && m_cur.valid && m_sinceJumpS < m_cfg.startRestartDwellS && !aForce &&
                aState.movementMode == 0 && last.movementMode == 0 && aState.stance == 0 && bestCost > contBestCost - m_cfg.startRestartOverride &&
                ((m_clipStart[static_cast<uint32_t>(m_cur.clip)] == 1 && m_clipStart[en.clip] == 1 && newTime < 0.1f &&
                  steer::SpeedClassOfName(db.ClipName(static_cast<uint32_t>(m_cur.clip))) == steer::SpeedClassOfName(db.ClipName(en.clip))) ||
                 (aState.movementState == 1 && aState.speed2dCmS <= m_cfg.startStalledSpeedCmS && !(cl.flags & kClipLoop) &&
                  !(db.Clip(static_cast<uint32_t>(m_cur.clip)).flags & kClipLoop) && !m_clipStop[static_cast<uint32_t>(m_cur.clip)] && !m_clipStop[en.clip])))
            {
                jump = false;
                bestDb = contDb;
                bestRow = contRow;
                bestEntry = contEntry;
            }
            // MatcherConfig::downshiftDeferS: a young gait downshift does not hand the continuing clip to a Run_to_Walk transition
            if (UTPP_AND_NOT(JUMPPICK, 37) jump && m_guardsOn && m_cfg.downshiftDeferS > 0.0f && contDb >= 0 && m_cur.valid && !aForce &&
                m_clipRunToWalk[en.clip] && aState.gait < m_holdGait && aState.movementState == 1 && m_holdChangeAge < m_cfg.downshiftDeferS &&
                aState.movementMode == 0 && last.movementMode == 0 && aState.stance == 0)
            {
                jump = false;
                bestDb = contDb;
                bestRow = contRow;
                bestEntry = contEntry;
            }
            // churn guard 1 (MatcherConfig::loopDwellS): a young loop gives way to another loop only by a clear margin
            if (UTPP_AND_NOT(JUMPPICK, 24) jump && m_guardsOn && res.interrupt == Interrupt::DoNotInterrupt && m_cfg.loopDwellS > 0.0f && contDb >= 0 && m_cur.valid && m_sinceJumpS < m_cfg.loopDwellS &&
                aState.movementMode == 0 && last.movementMode == 0 && aState.stance == 0 && !aForce && (cl.flags & kClipLoop) &&
                (db.Clip(static_cast<uint32_t>(m_cur.clip)).flags & kClipLoop) && bestCost > contBestCost - m_cfg.loopDwellOverride)
            {
                jump = false;
                bestDb = contDb;
                bestRow = contRow;
                bestEntry = contEntry;
            }
            // Queries in the search loop are scratch per database. Rebuild the winner
            // while continuing candidates still refer to the previous player; the last
            // searched schema need not be the selected schema.
            BuildQuery(static_cast<uint32_t>(bestDb), true, allowLastPose);
            float rateMin = m_cfg.slideExitMinPlayRate > 0.0f && m_dbSlideExit[static_cast<uint32_t>(bestDb)] ?
                std::min(m_cfg.minPlayRate, m_cfg.slideExitMinPlayRate) : m_cfg.minPlayRate;
            // G1 (MatcherConfig::walkMinPlayRate): a loop clip of the stand walk-loop databases in the Walk gait; the Run_to_Walk
            // transition clips (not loops), every other database and a fixed rate (minPlayRate == maxPlayRate) keep the ordinary bound
            if (m_cfg.slowWalkLoops && m_cfg.walkMinPlayRate > 0.0f && m_cfg.maxPlayRate - m_cfg.minPlayRate > 1e-4f && aState.gait == 0 &&
                m_dbStandWalkLoops[static_cast<uint32_t>(bestDb)] &&
                (cl.flags & kClipLoop))
                rateMin = std::min(rateMin, m_cfg.walkMinPlayRate);
            // G10: only at a steady jog pace above the Walk_Loops band (runWalkLoopsMaxCmS), so a Run_Loop kept through a deceleration / stop
            // below it keeps the ordinary 0.85 bound instead of a slow-motion run (review 2026-10-31)
            if (m_cfg.steadyRunLoops && m_cfg.runMinPlayRate > 0.0f && m_cfg.maxPlayRate - m_cfg.minPlayRate > 1e-4f && aState.gait == 1 &&
                aState.speed2dCmS > m_cfg.runWalkLoopsMaxCmS &&
                m_dbStandRunLoops[static_cast<uint32_t>(bestDb)] && (cl.flags & kClipLoop))
                rateMin = std::min(rateMin, m_cfg.runMinPlayRate);
            // CR-3 (MatcherConfig::crouchWalkMinPlayRate): a loop clip of the crouch walk-loop databases at the slow crouch stick (gait Walk)
#if UTPP_LOCOMATCHER_MUTATION != 1 // mutation 1: the crouch walk loops keep the ordinary 0.85 bound
            if (m_cfg.slowCrouchLoops && m_cfg.crouchWalkMinPlayRate > 0.0f && m_cfg.maxPlayRate - m_cfg.minPlayRate > 1e-4f && aState.stance == 1 &&
                aState.gait == 0 && m_dbCrouchWalkLoops[static_cast<uint32_t>(bestDb)] && (cl.flags & kClipLoop))
                rateMin = std::min(rateMin, m_cfg.crouchWalkMinPlayRate);
#endif
            const auto rate = rate2::Calculate(db, static_cast<uint32_t>(bestDb),
                static_cast<uint32_t>(bestRow), m_raw.data(), rateMin, m_cfg.maxPlayRate);
            if (!jump)
            {
                // continuing pose result (bIsContinuingPoseSearch): its database may differ from the previous one
                // when another chooser database indexes the same clip (PopulateContinuingPoseSearches).
                m_cur.db = bestDb;
                m_cur.localRow = bestRow;
                m_cur.entry = bestEntry;
                res.continuingOutsideSet = !((res.dbMask >> bestDb) & 1u);
            }
            if (jump)
            {
                if (m_stanceTransitionDirection[en.clip])
                {
                    // Consume at selection, not at physical clip completion:
                    // the indexed range may hand over before the physical end.
                    m_stanceEntryRemaining = 0.f;
                    m_stanceEntryAllowed = false;
                    m_completedNoReselectClip = static_cast<int32_t>(en.clip);
                    m_completedNoReselectStance = aState.stance;
                }
                // CR-4: a Crouch -> Stand transition selected at rest is held through its rise
                m_riseHoldLeft = m_cfg.crouchToStandHoldS > 0.0f && m_stanceTransitionDirection[en.clip] == -1 &&
                    aState.movementMode == 0 && aState.movementState == 0 && aState.stance == 0 ? m_cfg.crouchToStandHoldS : 0.0f;
                m_riseHoldClip = static_cast<int32_t>(en.clip);
                m_riseHoldFlags = HoldFlags(aState);
                // exitTransitionTailS: armed only by a VX_ clip selected at rest on the ground, every other selection disarms
                m_exitHoldClip = m_cfg.exitTransitionTailS > 0.0f && m_clipExitTransition[en.clip] && aState.movementMode == 0 &&
                                 aState.movementState == 0 && aState.speed2dCmS <= m_cfg.exitTransitionMaxSpeedCmS ?
                    static_cast<int32_t>(en.clip) : -1;
                m_exitHoldStance = aState.stance;
                // stopCommitTailS: armed only by a Stop clip selected with the stick released on the ground, every other selection disarms
                m_stopHoldClip = m_cfg.stopCommitTailS > 0.0f && aState.movementMode == 0 && aState.movementState == 0 &&
                                 (aState.stance == 0 || m_cfg.stopCommitCrouch) && steer::ClassifyClip(db.ClipName(en.clip)) == steer::kClassStop ? static_cast<int32_t>(en.clip) : -1;
                m_stopHoldStance = aState.stance;
                m_stopHoldStart = newTime;
                // turnCommit: armed only by a pivot / moving-turn clip selected on a reversal while moving on the stand ground
                // mutation 32: no turn commit (the selected turn competes on cost from its next update on)
                m_turnHoldClip = UTPP_AND_NOT(JUMPPICK, 32) m_cfg.turnCommit && aState.isPivoting && aState.movementMode == 0 && aState.movementState == 1 &&
                                 aState.stance == 0 && m_clipTurnEndS[en.clip] > newTime ? static_cast<int32_t>(en.clip) : -1;
                m_turnHoldStance = aState.stance;
                m_turnHoldFlags = HoldFlags(aState) & ~4;
                m_turnHoldPivotDropped = false;
                m_turnTailS = 0.0f;
                m_stopHoldFlags = HoldFlags(aState);
                m_exitHoldFlags = HoldFlags(aState);
                res.blend.valid = m_cur.valid;
                // Standing TIP assets put their entire root turn in the first
                // .5s. The generic .5s ground crossfade removes most of that yaw
                // against stationary idle. Admit stationary aim/shot footwork
                // with a short fade; other ground/air transitions stay unchanged.
                // fix3/jump (blendTipAfterSlide): not while a slide exit clip is on display and still rising
                float tipCap = .1f;
                // mutation 5: the turn-in-place out of the exit clip keeps the 0.1 s fade; mutation 9: gated by the age of the exit clip (hold + 0.2 s), not by its rise
                if (UTPP_AND_NOT(JUMPPICK, 5) m_cfg.blendTipAfterSlide > 0.0f && m_cur.valid && m_cur.db >= 0 && m_dbSlideExit[static_cast<uint32_t>(m_cur.db)] &&
                    UTPP_TERN(JUMPPICK, 9, m_slideExitAge < m_cfg.slideExitMinHoldS + 0.2f,
                              HipsHeightM(rr.frame) - HipsHeightM(m_cur.frame) > m_cfg.tipRiseGapM))
                    tipCap = m_cfg.blendTipAfterSlide;
                res.blend.blendTime = (aState.stationaryShotTurn || stationaryAim) &&
                    steer::ClassifyClip(db.ClipName(en.clip)) == steer::kClassTurnInPlace ?
                    std::min(blendTime, tipCap) : blendTime;
                // fix3/jump (blendSlideFamily): FootOut <-> KneesOut inside a slide
                // mutation 6: the family switch keeps the 0.2 s slide fade
                if (UTPP_AND_NOT(JUMPPICK, 6) m_cfg.blendSlideFamily > 0.0f && m_cur.valid && m_cur.db >= 0 && m_dbSlide[static_cast<uint32_t>(m_cur.db)] &&
                    m_dbSlide[static_cast<uint32_t>(bestDb)] && m_clipSlideFamily[static_cast<uint32_t>(m_cur.clip)] && m_clipSlideFamily[en.clip] &&
                    m_clipSlideFamily[static_cast<uint32_t>(m_cur.clip)] != m_clipSlideFamily[en.clip])
                {
                    res.blend.blendTime = std::fmax(res.blend.blendTime, m_cfg.blendSlideFamily);
                    res.blend.fullLegFade = true;
                }
                res.blend.stanceChange = m_cur.valid && m_haveLastState && aState.movementMode == 0 && last.movementMode == 0 &&
                    (aState.stance != last.stance || std::abs(m_stanceTransitionDirection[en.clip]) == 1);
#if UTPP_FOOTSINK_MUTATION == 4 // mutation 4: no ground fade is ever flagged (the KeepFootHeight permission stays the stance fades')
                res.blend.groundFade = false;
#elif UTPP_FOOTSINK_MUTATION == 5 // mutation 5: every fade is flagged, slide / jump / air / traversal clips included
                res.blend.groundFade = m_cur.valid;
#else
                res.blend.groundFade = m_cur.valid && m_cur.db >= 0 && m_haveLastState && aState.movementMode == 0 && last.movementMode == 0 &&
                    m_dbFadeFrom[static_cast<uint32_t>(m_cur.db)] && m_dbFadeTo[static_cast<uint32_t>(bestDb)] &&
                    // the turn in place out of a rising slide exit clip keeps its plain fade (fix3/jump blendTipAfterSlide measures its pop)
                    !(m_dbSlideExit[static_cast<uint32_t>(m_cur.db)] && m_dbTurnInPlace[static_cast<uint32_t>(bestDb)]);
#endif
                res.blend.toGround = aState.movementMode == 0 && m_dbFadeTo[static_cast<uint32_t>(bestDb)];
                res.blend.toAir = aState.movementMode == 1;
                res.blend.fromClip = m_cur.valid ? m_cur.clip : -1;
                res.blend.fromTime = m_cur.clipTime;
                res.blend.toClip = static_cast<int32_t>(en.clip);
                res.blend.toTime = newTime;
                if (m_cur.valid && m_haveLastState)
                {
                    const auto& outgoing = db.Clip(static_cast<uint32_t>(m_cur.clip));
                    res.blend.cancelledGroundStart = cancelstart::Candidate(CancellationState(last), cancellationCurrent,
                        !(outgoing.flags & kClipLoop), steer::ClassifyClip(db.ClipName(static_cast<uint32_t>(m_cur.clip))),
                        m_cur.clipTime, steer::ClassifyClip(db.ClipName(en.clip)),
                        res.interrupt == Interrupt::ForceInterrupt || res.interrupt == Interrupt::ForceInterruptAndInvalidateContinuingPose);
                }
                m_cur.valid = true;
                m_cur.db = bestDb;
                m_cur.localRow = bestRow;
                m_cur.entry = rr.entry;
                m_cur.clip = static_cast<int32_t>(en.clip);
                m_cur.clipTime = newTime;
                m_cur.frame = rr.frame;
                m_slideExitHoldLeft = m_dbSlideExit[static_cast<uint32_t>(bestDb)] ? m_cfg.slideExitMinHoldS : 0.0f;
                m_slideExitAge = 0.0f;
                m_sinceJumpS = 0.0f;
                m_upshiftEdgeDone = false;
                m_holdStance = aState.stance;
                m_holdGait = aState.gait;
                m_holdMovementState = aState.movementState;
                m_holdFlags = HoldFlags(aState);
                m_holdChangeAge = 0.0f;
                m_cur.stationaryAimGoalValid = stationaryAim &&
                    steer::ClassifyClip(db.ClipName(en.clip)) == steer::kClassTurnInPlace;
                m_cur.stationaryAimGoal = m_cur.stationaryAimGoalValid ? aState.stationaryAimTargetYawDeg : 0.f;
            }
            res.jumped = jump;
            res.cost = jump ? bestCost : res.continuingCost;
            m_cur.playRate = rate.wanted;
            m_lastRateRatio = rate.ratio;
        }
        else
            m_cur.valid = false; // invalid search result: UE resets FMotionMatchingState::SearchResult (no continuing pose)
        m_candActive = false;

        if (m_cur.valid)
        {
            res.database = m_cur.db;
            res.localRow = m_cur.localRow;
            res.entry = m_cur.entry;
            res.clip = m_cur.clip;
            res.frame = m_cur.frame;
            res.clipTime = m_cur.clipTime;
            res.playRate = m_cur.playRate;
            res.playRateRatio = m_lastRateRatio;
            PushHistory(static_cast<uint32_t>(m_cur.db), static_cast<uint32_t>(m_cur.localRow));
        }
        m_lastState = aState;
        m_haveLastState = true;
        m_last = res;
        return m_last;
    }

    // fix3/jump: model-space height of the Hips (metres) of a database frame, the FK of the game-local poses along the skeleton chain
    float HipsHeightM(uint32_t aGlobalFrame) const
    {
        const PoseRecord* p = m_db->Pose(aGlobalFrame);
        math::Vec3 pos{0, 0, 0};
        math::Quat q{0, 0, 0, 1};
        for (const int b : m_hipsChain)
        {
            const PoseRecord& r = p[b];
            pos = pos + math::QuatRotate(q, math::Vec3{r.px, r.py, r.pz});
            q = math::QuatNormalize(math::QuatMul(q, math::Quat{r.qx, r.qy, r.qz, r.qw}));
        }
        return pos.z;
    }

    // motionfix item 5: rows of the fall clips in database d (not BLOCK, not excluded) with the fall bias added; a lower
    // total replaces aR. Linear over those entries only (3 clips, <= ~250 rows).
    void FallBiasSearch(uint32_t d, MotionSearchResult& aR) const
    {
        const MotionDatabase& db = *m_db;
        const DatabaseRecord& D = db.Db(d);
        for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
        {
            const EntryRecord& en = db.Entry(e);
            if (!m_fallClip[en.clip] || en.numRows == 0)
                continue;
            const uint32_t r0 = en.firstRow - D.firstRow;
            for (uint32_t row = r0; row < r0 + en.numRows; ++row)
            {
                const RowRecord& rr = db.Row(D.firstRow + row);
                if ((rr.flags & kRowBlock) || m_excluded[row])
                    continue;
                ++aR.rowsEvaluated;
                const float c = db.RowCost(d, row, m_query.data()) + rr.costAddend + m_cfg.airFallBias;
                if (c < aR.cost)
                {
                    aR.cost = c;
                    aR.localRow = static_cast<int32_t>(row);
                }
            }
        }
    }

    // True when (database, local row) was selected / played within the reselection history window.
    bool InHistory(uint32_t d, uint32_t aRow) const
    {
        for (uint32_t i = 0; i < m_numHist; ++i)
            if (m_history[i].db == d && m_history[i].row == aRow && m_now - m_history[i].time <= m_cfg.poseReselectHistory)
                return true;
        return false;
    }

    // Raw (un-normalized) query of database d built by the last BuildQuery (test access).
    const float* RawQuery() const { return m_raw.data(); }
    const float* NormalizedQuery() const { return m_query.data(); }

    // Sets the trajectory / pose history read by BuildQuery outside Update (test access; pointers must stay valid).
    void SetQueryInputs(const TrajSample* aTraj, uint32_t aNumTraj, const PoseHistory* aHist)
    {
        m_traj = aTraj;
        m_numTraj = aNumTraj;
        m_hist = aHist;
    }

    // Builds the normalized query of database d (public for tests).
    void BuildQuery(uint32_t d, bool aUseContinuing, bool aAllowLastPose = false)
    {
        const MotionDatabase& db = *m_db;
        const DatabaseRecord& D = db.Db(d);
        const SchemaRecord& S = db.Schema(D.schema);
        const float* mean = db.NormMean(D.norm);
        const float* scale = db.NormScale(D.norm);
        // continuing pose raw features when its database uses the same schema
        const float* contF = nullptr;
        const float* contMean = nullptr;
        const float* contScale = nullptr;
        int32_t contDb = -1, contRow = -1;
        if (aUseContinuing && m_candActive)
        {
            // inside Update: first cached (non-invalidated) continuing pose with this schema (MotionMatch caches the
            // continuing pose per database and per schema, first instance wins)
            for (uint32_t k = 0; k < m_numCand && contDb < 0; ++k)
                if (db.Db(static_cast<uint32_t>(m_cand[k].db)).schema == D.schema)
                {
                    contDb = m_cand[k].db;
                    contRow = m_cand[k].localRow;
                }
        }
        else if (aUseContinuing && m_cur.valid && m_cur.localRow >= 0 && db.Db(static_cast<uint32_t>(m_cur.db)).schema == D.schema)
        {
            contDb = m_cur.db;
            contRow = m_cur.localRow;
        }
        // Excluded non-loop tails are not continuing candidates in UE either. The
        // game adapter currently has no PoseHistory, however: losing that final
        // indexed row used to replace every pose channel with the database mean,
        // repeatedly selecting the same earlier transition phase. Retain only
        // the last sampled feature row for this query; do not evaluate its old
        // continuing cost or extend DISABLE_RESELECTION beyond the source rule.
        // Caller history and explicit continuing-pose invalidation take priority.
        if (contDb < 0 && aUseContinuing && aAllowLastPose && !m_hist && m_cur.valid &&
            m_cur.localRow >= 0 && db.Db(static_cast<uint32_t>(m_cur.db)).schema == D.schema)
        {
            contDb = m_cur.db;
            contRow = m_cur.localRow;
        }
        if (contDb >= 0)
        {
            const DatabaseRecord& C = db.Db(static_cast<uint32_t>(contDb));
            contF = db.Features(static_cast<uint32_t>(contDb), static_cast<uint32_t>(contRow));
            contMean = db.NormMean(C.norm);
            contScale = db.NormScale(C.norm);
        }
        for (uint32_t k = 0; k < S.dims; ++k)
            m_raw[k] = mean[k];
        for (uint32_t c = S.firstChannel; c < S.firstChannel + S.numChannels; ++c)
        {
            const ChannelRecord& ch = db.Channel(c);
            float v[3] = {0.0f, 0.0f, 0.0f};
            bool have = false;
            if (ch.flags & kChanTrajectory)
                have = TrajectoryFeature(ch, v);
            else if (ch.inputQuery == 0 && contF)
            {
                for (uint32_t k = 0; k < ch.cardinality; ++k)
                {
                    const uint32_t o = ch.dataOffset + k;
                    m_raw[o] = contF[o] / contScale[o] + contMean[o];
                }
                continue;
            }
            else
                have = HistoryFeature(ch, v);
            if (!have)
                continue;
            float out[3];
            const uint32_t n = Strip(ch, v, out);
            for (uint32_t k = 0; k < n && k < ch.cardinality; ++k)
                m_raw[ch.dataOffset + k] = out[k];
        }
        for (uint32_t k = 0; k < S.dims; ++k)
            m_query[k] = (m_raw[k] - mean[k]) * scale[k];
    }

private:
    struct Cur
    {
        bool valid = false;
        int32_t db = -1;
        int32_t localRow = -1;
        uint32_t entry = 0;
        int32_t clip = -1;
        float clipTime = 0.0f;
        float playRate = 1.0f;
        uint32_t frame = 0;
        bool stationaryAimGoalValid = false;
        float stationaryAimGoal = 0;
    };
    struct HistEntry
    {
        uint32_t db = 0;
        uint32_t row = 0;
        double time = 0.0;
    };
    struct InputIdx
    {
        int gait = -1, stance = -1, movementState = -1, movementMode = -1, speed = -1, ttl = -1, starting = -1,
            pivoting = -1, landLight = -1, landHeavy = -1, traversed = -1, tip = -1, spin = -1, lod = -1,
            movementDirection = -1, movementModeLast = -1; // slide inputs: -1 in a bundle without slide data
    };

    static bool Fail(std::string& e, const char* m)
    {
        e = m;
        return false;
    }

    // Local row of global frame aFrame in entry aEntry of database d (-1 = not indexed, e.g. excluded range).
    int32_t FindRow(uint32_t d, uint32_t aEntry, uint32_t aFrame) const
    {
        const MotionDatabase& db = *m_db;
        const EntryRecord& en = db.Entry(aEntry);
        uint32_t lo = en.firstRow, hi = en.firstRow + en.numRows;
        while (lo < hi)
        {
            const uint32_t mid = (lo + hi) / 2;
            if (db.Row(mid).frame < aFrame)
                lo = mid + 1;
            else
                hi = mid;
        }
        if (lo < en.firstRow + en.numRows && db.Row(lo).frame == aFrame)
            return static_cast<int32_t>(lo - db.Db(d).firstRow);
        return -1;
    }

    void PushHistory(uint32_t d, uint32_t aRow)
    {
        // drop expired entries, then append (ring kept compact; bounded by window / update period)
        uint32_t w = 0;
        for (uint32_t i = 0; i < m_numHist; ++i)
            if (m_now - m_history[i].time <= m_cfg.poseReselectHistory)
                m_history[w++] = m_history[i];
        m_numHist = w;
        if (m_numHist == m_history.size())
        {
            std::memmove(m_history.data(), m_history.data() + 1, (m_history.size() - 1) * sizeof(HistEntry));
            --m_numHist;
        }
        m_history[m_numHist++] = HistEntry{d, aRow, m_now};
    }

    // UE 5.8 IsForceInterrupt / IsInvalidatingContinuingPose (PoseSearchLibrary.cpp 326-365); aIn = the database is
    // among the databases to search.
    static bool IsForceInterrupt(Interrupt m, bool aIn)
    {
        switch (m)
        {
        case Interrupt::DoNotInterrupt:
            return false;
        case Interrupt::InterruptOnDatabaseChange:
        case Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose:
            return !aIn;
        case Interrupt::ForceInterrupt:
        case Interrupt::ForceInterruptAndInvalidateContinuingPose:
            return true;
        }
        return false;
    }
    static bool IsInvalidating(Interrupt m, bool aIn)
    {
        switch (m)
        {
        case Interrupt::DoNotInterrupt:
        case Interrupt::InterruptOnDatabaseChange:
        case Interrupt::ForceInterrupt:
            return false;
        case Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose:
            return !aIn;
        case Interrupt::ForceInterruptAndInvalidateContinuingPose:
            return true;
        }
        return false;
    }

    // Pose-history reselection rows + (UE PoseSearchDatabase.cpp 1557-1580) when d holds a cached continuing pose of
    // a DISABLE_RESELECTION entry, every row of every entry of that clip in d.
    uint32_t MarkExcluded(uint32_t d, int aStartTurnDirection = 0)
    {
        uint32_t n = 0;
        // Chooser idle databases also contain stand<->crouch entry clips. They
        // are legal NEW candidates only near a real requested stance edge.
        // Cached continuing candidates are evaluated separately and retain
        // their full indexed playback after this admission window closes.
        const DatabaseRecord& domain = m_db->Db(d);
        for (uint32_t e = domain.firstEntry; e < domain.firstEntry + domain.numEntries; ++e)
        {
            const EntryRecord& en = m_db->Entry(e);
            const int direction = m_stanceTransitionDirection[en.clip];
            if (!direction || (m_stanceEntryAllowed &&
                direction == (m_observedStance == 1 ? 1 : -1)))
                continue;
#if defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 5 // mutation 5: stance entry clips always admitted (with the completed-clip exclusion below off too)
            continue;
#endif
            const uint32_t r0 = en.firstRow - domain.firstRow;
            if (en.numRows) std::memset(m_excluded.data() + r0, 1, en.numRows);
            n += en.numRows;
        }
        for (uint32_t i = 0; i < m_numHist; ++i)
            if (m_history[i].db == d && m_now - m_history[i].time <= m_cfg.poseReselectHistory)
            {
                m_excluded[m_history[i].row] = 1;
                ++n;
            }
        for (uint32_t k = 0; k < m_numCand; ++k)
        {
            if (m_cand[k].db != static_cast<int32_t>(d))
                continue;
            const DatabaseRecord& D = m_db->Db(d);
            const EntryRecord& cen = m_db->Entry(m_cand[k].entry);
            if (!(cen.flags & kEntryDisableReselection))
                break;
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (en.clip != cen.clip || en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
                std::memset(m_excluded.data() + r0, 1, en.numRows);
                n += en.numRows;
            }
            break;
        }
        // A completed DISABLE_RESELECTION one-shot is no longer a continuing candidate, so the per-entry
        // continuing-pose rule above cannot protect it at the end-of-clip search. Keep its rows out until
        // the requested stance changes.
#if !(defined(UTPP_MOTION_SNAP_MUTATION) && UTPP_MOTION_SNAP_MUTATION == 5) // mutation 5: the completed one-shot is selectable again (crouch idle re-entry)
        if (m_completedNoReselectClip >= 0)
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (static_cast<int32_t>(en.clip) != m_completedNoReselectClip || en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
                std::memset(m_excluded.data() + r0, 1, en.numRows);
                n += en.numRows;
            }
        }
#endif
        // G1 (MatcherConfig::walkTransitionWindowS): the Run_to_Walk transition clips are not NEW candidates unless the chooser speed was fast
        // (Run / Sprint gait, above walkTransitionFastCmS) within the window; their continuing pose is not affected (searched separately)
        if (m_cfg.slowWalkLoops && m_cfg.walkTransitionWindowS > 0.0f && m_sinceFastS > m_cfg.walkTransitionWindowS && m_dbStandWalkLoops[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (!m_clipRunToWalk[en.clip] || en.numRows == 0)
                    continue;
                std::memset(m_excluded.data() + (en.firstRow - D.firstRow), 1, en.numRows);
                n += en.numRows;
            }
        }
        // G10 (MatcherConfig::runTransitionWindowS): the Walk_to_Run transition clips are not NEW candidates unless the chooser speed was at most
        // runTransitionSlowCmS within the window; their continuing pose is not affected (searched separately)
        if (m_cfg.steadyRunLoops && m_cfg.runTransitionWindowS > 0.0f && m_sinceSlowS > m_cfg.runTransitionWindowS && m_dbStandRunLoops[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (!m_clipWalkToRun[en.clip] || en.numRows == 0)
                    continue;
                std::memset(m_excluded.data() + (en.firstRow - D.firstRow), 1, en.numRows);
                n += en.numRows;
            }
        }
        // MatcherConfig::headingCatchUpS: the clips for a body that keeps its facing are not NEW candidates while the facing catches up with a heading step
        if (m_headingCatchUp)
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (!m_clipKeepsFacing[en.clip] || en.numRows == 0)
                    continue;
                std::memset(m_excluded.data() + (en.firstRow - D.firstRow), 1, en.numRows);
                n += en.numRows;
            }
        }
        // G10 (MatcherConfig::runWalkLoopsMaxCmS): the Walk_Loops database joined the Run gait for its LOOPS only: its Run_to_Walk transitions
        // (motion: gait Walk) and Walk_Shuffle clips are not NEW candidates (measured: Run_to_Walk_Rfoot at 2.1 s won at cost -0.09 and played 4 s at 247 cm/s)
        if (m_runGaitWalkLoops && m_dbStandWalkLoops[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if ((m_db->Clip(en.clip).flags & kClipLoop) || en.numRows == 0)
                    continue;
                std::memset(m_excluded.data() + (en.firstRow - D.firstRow), 1, en.numRows);
                n += en.numRows;
            }
        }
        // G10 (MatcherConfig::runTransitionNoHeadRow): the isolated head row (frame 0, next row at frame 28) has no continuing row on the next update
        if (m_cfg.steadyRunLoops && m_cfg.runTransitionNoHeadRow && m_speedCmS > m_cfg.runTransitionSlowCmS && m_dbStandRunLoops[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (!m_clipWalkToRun[en.clip] || en.numRows < 2 || m_db->Row(en.firstRow + 1).frame == m_db->Row(en.firstRow).frame + 1)
                    continue;
                if (!m_excluded[en.firstRow - D.firstRow])
                {
                    m_excluded[en.firstRow - D.firstRow] = 1;
                    ++n;
                }
            }
        }
        if (m_slideExitSettled && m_dbSlideExit[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            const float fps = m_db->Header().fps;
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
                const int family = m_clipSlideFamily[en.clip];
                if (m_slideExitFamily && family && family != m_slideExitFamily)
                {
                    std::memset(m_excluded.data() + r0, 1, en.numRows);
                    n += en.numRows;
                    continue;
                }
                if (m_cfg.slideExitMaxEntryS > 0.0f)
                {
                    const uint32_t first = m_db->Clip(en.clip).firstFrame;
                    for (uint32_t r = 0; r < en.numRows; ++r)
                        if (static_cast<float>(m_db->Row(en.firstRow + r).frame - first) / fps > m_cfg.slideExitMaxEntryS)
                        {
                            m_excluded[r0 + r] = 1;
                            ++n;
                        }
                }
            }
        }
        // RC2 (MatcherConfig::slideEntryMaxS): the slide entry takes an Into row up to the cap, no Loop row (both lie already)
        if (m_slideEntryCap && m_dbSlide[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            const float fps = m_db->Header().fps;
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
#if UTPP_LOCOMATCHER_MUTATION != 3 // mutation 3: the entry cap leaves the Loop clips in
                if (m_clipSlideLoop[en.clip])
                {
                    std::memset(m_excluded.data() + r0, 1, en.numRows);
                    n += en.numRows;
                    continue;
                }
#endif
                if (!m_clipSlideInto[en.clip])
                    continue;
                const uint32_t first = m_db->Clip(en.clip).firstFrame;
                for (uint32_t r = 0; r < en.numRows; ++r)
                    if (static_cast<float>(m_db->Row(en.firstRow + r).frame - first) / fps > m_cfg.slideEntryMaxS && !m_excluded[r0 + r])
                    {
                        m_excluded[r0 + r] = 1;
                        ++n;
                    }
            }
        }
        // RC3 (MatcherConfig::slideIntoHandover): only the same-family Loop clip of the playing database is a candidate
        if (m_slideHandoverDb >= 0 && m_dbSlide[d])
        {
            const DatabaseRecord& D = m_db->Db(d);
            const int family = m_clipSlideFamily[static_cast<uint32_t>(m_cur.clip)];
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                if (en.numRows == 0 || (static_cast<int32_t>(d) == m_slideHandoverDb && m_clipSlideLoop[en.clip] && m_clipSlideFamily[en.clip] == family))
                    continue;
                std::memset(m_excluded.data() + (en.firstRow - D.firstRow), 1, en.numRows);
                n += en.numRows;
            }
        }
        if (m_standJumpBack || m_standJumpTakeoff || m_standJumpOff || m_jumpRiseOff || m_jumpOutBack || m_jumpOutFall || m_airFootLock)
        {
            // fix3/jump: a standing jump takes no B / LL / RL Start row, no F Off row on the take-off and, near touchdown, no take-off-phase row;
            // fix5/jump: a jump in motion no F Off row from its first airborne update to the apex
            const DatabaseRecord& D = m_db->Db(d);
            const float fps = m_db->Header().fps;
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                const int kind = m_clipJumpKind[en.clip];
                if (!kind || en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
                // airKeepLeadFootS: no take-off / run-off row of the other lead foot inside the flight
                if (m_airFootLock && m_clipJumpFoot[en.clip] != 0 && m_clipJumpFoot[en.clip] != m_flightFoot)
                {
                    std::memset(m_excluded.data() + r0, 1, en.numRows);
                    n += en.numRows;
                    continue;
                }
                // fix/jumpout-cam: a jump-out flight takes no B / LL / RL Start row; past its apex no Start / Off row at all (the fall loop of this
                // database is the only jump clip left)
                if ((kind == 1 && (m_standJumpBack || m_jumpOutBack)) || (m_jumpOutFall && (m_dbHasFall[d] || m_jumpOutFallMask)))
                {
                    std::memset(m_excluded.data() + r0, 1, en.numRows);
                    n += en.numRows;
                    continue;
                }
                if (kind == 3 && (m_standJumpOff || m_jumpRiseOff))
                {
                    std::memset(m_excluded.data() + r0, 1, en.numRows);
                    n += en.numRows;
                    continue;
                }
                if (!m_standJumpTakeoff || kind == 1)
                    continue;
                const float phaseEnd = kind == 2 ? kJumpStartPhaseEndS : kJumpOffPhaseEndS;
                const uint32_t first = m_db->Clip(en.clip).firstFrame;
                for (uint32_t r = 0; r < en.numRows; ++r)
                    if (static_cast<float>(m_db->Row(en.firstRow + r).frame - first) / fps < phaseEnd)
                    {
                        m_excluded[r0 + r] = 1;
                        ++n;
                    }
            }
        }
        if (aStartTurnDirection)
        {
            const DatabaseRecord& D = m_db->Db(d);
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = m_db->Entry(e);
                const int direction = StartTurnDirection(static_cast<int32_t>(en.clip));
                if (!direction || direction == aStartTurnDirection || en.numRows == 0)
                    continue;
                const uint32_t r0 = en.firstRow - D.firstRow;
                std::memset(m_excluded.data() + r0, 1, en.numRows);
                n += en.numRows;
            }
        }
        return n;
    }

    void ClearExcluded(uint32_t d, uint32_t aMarked)
    {
        if (aMarked)
            std::memset(m_excluded.data(), 0, m_db->Db(d).numRows);
    }

    void UpdateStanceAdmission(float dt, const ChooserState& state)
    {
        // Matches the existing stance-entry window, without applying legacy
        // curation to unrelated V2 states. Air, landing, traversal and a long
        // sampling gap invalidate the edge rather than inventing one on return.
        constexpr float window = .6f;
        const bool validDt = std::isfinite(dt) && dt >= 0.f && dt <= window;
        const bool grounded = validDt && state.movementMode == 0 &&
            (state.stance == 0 || state.stance == 1) &&
            !state.justTraversed && !state.justLandedLight && !state.justLandedHeavy;
        const bool edge = grounded && m_stanceGround && m_stanceObserved && state.stance != m_observedStance;
        if (!grounded)
            m_stanceEntryRemaining = 0.f;
        else if (edge)
            m_stanceEntryRemaining = window;
        else
            m_stanceEntryRemaining = std::max(0.f, m_stanceEntryRemaining - dt);
        // A zero-time observation may record an edge but cannot select/consume
        // a new player. The next real sample ages the same bounded window.
        m_stanceEntryAllowed = grounded && dt > 0.f && m_stanceEntryRemaining > 0.f;
        m_observedStance = state.stance;
        m_stanceObserved = true;
        m_stanceGround = grounded;
    }

    // ---- trajectory sampling (linear, clamped)
    TrajSample SampleTraj(float t) const
    {
        const TrajSample* s = m_traj;
        const uint32_t n = m_numTraj;
        if (t <= s[0].t)
            return s[0];
        if (t >= s[n - 1].t)
            return s[n - 1];
        uint32_t i = 1;
        while (i < n && s[i].t < t)
            ++i;
        const TrajSample& a = s[i - 1];
        const TrajSample& b = s[i];
        const float u = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0f;
        TrajSample r;
        r.t = t;
        r.x = a.x + (b.x - a.x) * u;
        r.y = a.y + (b.y - a.y) * u;
        r.z = a.z + (b.z - a.z) * u;
        r.yaw = a.yaw + math::WrapAngle(b.yaw - a.yaw) * u;
        return r;
    }

    static void Rot(float aYaw, float x, float y, float& ox, float& oy)
    {
        const float c = std::cos(aYaw), s = std::sin(aYaw);
        ox = c * x - s * y;
        oy = s * x + c * y;
    }

    // rel(root, root, ts, to) of FORMAT.txt v2.
    void RelRoot(float ts, float to, float out[3]) const
    {
        const TrajSample a = SampleTraj(ts);
        const TrajSample o = SampleTraj(to);
        Rot(-o.yaw, a.x - o.x, a.y - o.y, out[0], out[1]);
        out[2] = a.z - o.z;
    }

    bool TrajectoryFeature(const ChannelRecord& ch, float v[3]) const
    {
        constexpr float h = 1.0f / 60.0f;
        const float ts = ch.tSample, to = ch.tOrigin;
        switch (ch.kind)
        {
        case 1: // Position
            RelRoot(ts, to, v);
            return true;
        case 2: // Velocity
        case 3: // VelocityDirection
        {
            float a[3], b[3];
            RelRoot(ts, to, a);
            RelRoot(ts - h, ch.velocitySpace == 1 ? to - h : to, b);
            for (int k = 0; k < 3; ++k)
                v[k] = (a[k] - b[k]) * 60.0f;
            if (ch.kind == 3)
            {
                for (int k = 0; k < 3; ++k)
                    v[k] *= 100.0f;
                const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (l > 1.0f)
                    for (int k = 0; k < 3; ++k)
                        v[k] /= l;
            }
            return true;
        }
        case 4: // Heading
        {
            const float dy = SampleTraj(ts).yaw - SampleTraj(to).yaw;
            Rot(dy, ch.axis[0], ch.axis[1], v[0], v[1]);
            v[2] = ch.axis[2];
            return true;
        }
        default:
            return false;
        }
    }

    // Pose channels from the pose history (bone and origin in the history; root origin = -1).
    bool HistoryFeature(const ChannelRecord& ch, float v[3]) const
    {
        if (!m_hist || m_hist->numSamples == 0)
            return false;
        const int bi = HistBone(ch.bone);
        if (bi < 0 || (ch.originBone >= 0 && HistBone(ch.originBone) < 0))
            return false;
        constexpr float h = 1.0f / 60.0f;
        float a[3], b[3];
        switch (ch.kind)
        {
        case 1:
            return HistRel(ch.bone, ch.originBone, ch.tSample, ch.tOrigin, v);
        case 2:
        case 3:
            if (!HistRel(ch.bone, ch.originBone, ch.tSample, ch.tOrigin, a) ||
                !HistRel(ch.bone, ch.originBone, ch.tSample - h, ch.velocitySpace == 1 ? ch.tOrigin - h : ch.tOrigin, b))
                return false;
            for (int k = 0; k < 3; ++k)
                v[k] = (a[k] - b[k]) * 60.0f;
            if (ch.kind == 3)
            {
                for (int k = 0; k < 3; ++k)
                    v[k] *= 100.0f;
                const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (l > 1.0f)
                    for (int k = 0; k < 3; ++k)
                        v[k] /= l;
            }
            return true;
        case 4:
        {
            uint32_t i0, i1;
            float u;
            if (!HistAt(ch.tSample, i0, i1, u))
                return false;
            const math::Quat q = math::QuatNlerpShortest(m_hist->gq[i0][bi], m_hist->gq[i1][bi], u);
            const math::Vec3 ax = math::QuatRotate(q, math::Vec3{ch.axis[0], ch.axis[1], ch.axis[2]});
            const TrajSample rs = HistRoot(ch.tSample), ro = HistRoot(ch.tOrigin);
            Rot(rs.yaw - ro.yaw, ax.x, ax.y, v[0], v[1]);
            v[2] = ax.z;
            return true;
        }
        default:
            return false;
        }
    }

    int HistBone(int32_t aBone) const
    {
        for (uint32_t i = 0; i < m_hist->numBones; ++i)
            if (m_hist->bone[i] == aBone)
                return static_cast<int>(i);
        return -1;
    }

    bool HistAt(float t, uint32_t& i0, uint32_t& i1, float& u) const
    {
        const PoseHistory& H = *m_hist;
        const uint32_t n = H.numSamples;
        if (t <= H.root[0].t)
        {
            i0 = i1 = 0;
            u = 0.0f;
            return true;
        }
        if (t >= H.root[n - 1].t)
        {
            i0 = i1 = n - 1;
            u = 0.0f;
            return true;
        }
        uint32_t i = 1;
        while (i < n && H.root[i].t < t)
            ++i;
        i0 = i - 1;
        i1 = i;
        const float dt = H.root[i1].t - H.root[i0].t;
        u = dt > 0.0f ? (t - H.root[i0].t) / dt : 0.0f;
        return true;
    }

    TrajSample HistRoot(float t) const
    {
        // future pose times (t > 0) are not in the history: use the trajectory for the root
        if (t > 0.0f || !m_hist)
            return SampleTraj(t);
        uint32_t i0, i1;
        float u;
        HistAt(t, i0, i1, u);
        const TrajSample& a = m_hist->root[i0];
        const TrajSample& b = m_hist->root[i1];
        TrajSample r;
        r.t = t;
        r.x = a.x + (b.x - a.x) * u;
        r.y = a.y + (b.y - a.y) * u;
        r.z = a.z + (b.z - a.z) * u;
        r.yaw = a.yaw + math::WrapAngle(b.yaw - a.yaw) * u;
        return r;
    }

    bool HistPos(int32_t aBone, float t, float p[3]) const
    {
        const TrajSample r = HistRoot(t);
        if (aBone < 0)
        {
            p[0] = r.x;
            p[1] = r.y;
            p[2] = r.z;
            return true;
        }
        const int bi = HistBone(aBone);
        if (bi < 0)
            return false;
        uint32_t i0, i1;
        float u;
        HistAt(std::min(t, 0.0f), i0, i1, u);
        const math::Vec3 gp = math::Lerp(m_hist->gp[i0][bi], m_hist->gp[i1][bi], u);
        float x, y;
        Rot(r.yaw, gp.x, gp.y, x, y);
        p[0] = x + r.x;
        p[1] = y + r.y;
        p[2] = gp.z + r.z;
        return true;
    }

    bool HistRel(int32_t aBone, int32_t aOrigin, float ts, float to, float out[3]) const
    {
        float a[3], o[3];
        if (!HistPos(aBone, ts, a) || !HistPos(aOrigin, to, o))
            return false;
        const TrajSample ro = HistRoot(to);
        Rot(-ro.yaw, a[0] - o[0], a[1] - o[1], out[0], out[1]);
        out[2] = a[2] - o[2];
        return true;
    }

    // Component stripping + heading normalization (FORMAT.txt v2). Returns the number of values written.
    // Headings (NORMALIZE_OUTPUT) are normalized after stripping, like mmbake_motion.encode.
    static uint32_t Strip(const ChannelRecord& ch, float v[3], float out[3])
    {
        uint32_t n = 0;
        if (ch.components == 3)
        {
            out[0] = v[0];
            out[1] = v[1];
            n = 2;
        }
        else if (ch.components == 4)
        {
            out[0] = v[2];
            n = 1;
        }
        else
        {
            out[0] = v[0];
            out[1] = v[1];
            out[2] = v[2];
            n = 3;
        }
        if (ch.flags & kChanNormalizeOutput)
        {
            float l2 = 0.0f;
            for (uint32_t k = 0; k < n; ++k)
                l2 += out[k] * out[k];
            if (l2 > 1e-8f)
            {
                const float inv = 1.0f / std::sqrt(l2);
                for (uint32_t k = 0; k < n; ++k)
                    out[k] *= inv;
            }
        }
        return n;
    }

    const MotionDatabase* m_db = nullptr;
    const chooser::Table* m_table = nullptr;
    MatcherConfig m_cfg;
    InputIdx m_in;
    bool m_ready = false;
    std::vector<uint8_t> m_excluded;
    std::vector<int8_t> m_startTurnDirection;
    std::vector<int8_t> m_stanceTransitionDirection; // 0 none, +/-1 destination, 2 unknown (not admitted)
    std::vector<uint8_t> m_fallClip; // motionfix item 5: clip index -> 1 = kAirFallClipNames
    bool m_anyFallClip = false;
    std::vector<uint8_t> m_clipJumpKind; // fix3/jump: clip index -> 1 B / LL / RL Start, 2 F Start, 3 F Off, 0 other (by name)
    std::vector<uint8_t> m_clipJumpFoot; // clip index -> lead foot of a jump clip (m_clipJumpKind != 0): 1 Lfoot, 2 Rfoot, 0 none
    uint8_t m_flightFoot = 0;            // airKeepLeadFootS: lead foot of the jump clip this flight started on (0 = not yet known)
    bool m_airFootLock = false;          // airKeepLeadFootS: this update excludes the other lead foot's jump rows
    bool m_standJumpBack = false;        // fix3/jump: this update is a standing jump: no B / LL / RL Start row
    bool m_standJumpTakeoff = false;     // fix3/jump: ... and near touchdown: no take-off-phase row either
    bool m_standJumpOff = false;         // fix3/jump: ... and in the first standingJumpOffBlockS of the flight: no F Off row either
    bool m_flightActive = false;         // fix3/jump: the previous update was InAir
    bool m_flightStanding = false;       // fix3/jump: the first InAir update of this flight was a standing jump (kept to the touchdown)
    float m_flightAge = 0.0f;            // fix3/jump: seconds since the first InAir update of this flight
    bool m_flightJump = false;           // fix5/jump: an update of this flight carried a PSM jump detail (ChooserState::psmJump)
    bool m_jumpRiseOff = false;          // fix5/jump: this update is a jump in motion in its rise (after the take-off update): no F Off row
    bool m_flightJumpOut = false;        // fix/jumpout-cam: an update of this flight carried ChooserState::jumpOutFlight (latched to the touchdown)
    bool m_jumpOutFallLatched = false;   // fix/jumpout-cam: the jump-out flight passed its apex (vz <= 0): the fall loop owns the descent
    bool m_jumpOutBack = false;          // fix/jumpout-cam: this update is a jump-out flight: no B / LL / RL Start row
    bool m_jumpOutFall = false;          // fix/jumpout-cam: ... and past the apex: no Start / Off row, no continuing Start / Off clip (databases with a fall clip)
    bool m_jumpOutFallMask = false;      // r5 air: ... and a database of this update's chooser result indexes the fall clip: the rule holds for every database of the result
    std::vector<uint8_t> m_dbHasFall;    // fix/jumpout-cam: database index -> 1 = indexes a fall clip
    float m_sinceSlideS = 1.0e9f;        // fix3/jump: seconds since the last update with the chooser MovementMode Sliding
    float m_sinceFastS = 1.0e9f;         // G1: seconds since the last update with gait Run / Sprint or chooser speed > walkTransitionFastCmS
    std::vector<uint8_t> m_dbStandWalkLoops; // G1: database index -> 1 = PSD_Dense / Sparse_Stand_Walk_Loops
    bool m_guardsOn = true;                  // SetSelectionGuards: false = every selection churn guard off
    bool m_upshiftEdgeDone = false;          // churn guard 4:the deferred gait edge of the playing start clip has been delivered (reset on every selection)
    std::vector<uint8_t> m_clipStart;        // churn guard 4: clip index -> 1 start (Walk / Run / Sprint), 2 reface start (MatcherConfig::startUpshiftHoldS)
    std::vector<uint8_t> m_clipStop;         // churn guard 2: clip index -> 1 = a Stop clip (the stop commit owns those)
    std::vector<uint8_t> m_clipRunToWalk;    // G1: clip index -> 1 = M_Neutral_Transition_Run_to_Walk_L/Rfoot
    bool m_runGaitWalkLoops = false;         // G10: this update searches the Walk_Loops database in the Run gait (runWalkLoopsMaxCmS)
    std::vector<uint8_t> m_clipKeepsFacing;  // headingCatchUpS: clip index -> 1 a clip for a body that keeps its facing
    bool m_headingCatchUp = false;           // headingCatchUpS: this update excludes the m_clipKeepsFacing clips from the new candidates
    bool m_headingRestArmed = false;         // headingCatchUpS: the rule acts for a start from rest (armed while its start clip played)
    float m_speedCmS = 0.0f;                 // G10: chooser speed of this update
    float m_sinceSlowS = 1.0e9f;             // G10: seconds since the last update with chooser speed <= runTransitionSlowCmS
    std::vector<uint8_t> m_dbStandRunLoops;  // G10: database index -> 1 = PSD_Dense / Sparse_Stand_Run_Loops
    std::vector<uint8_t> m_dbCrouchWalkLoops; // CR-3: database index -> 1 = PSD_Dense / Sparse_Crouch_Walk_Loops
    std::vector<uint8_t> m_dbWalkStarts;      // ST-1: database index -> 1 = PSD_Dense / Sparse_*_Walk_Starts (stand and crouch)
    std::vector<float> m_clipAccelEndS;       // ST-1: clip index -> clip time where its acceleration ends (-1 = not a walk-start clip)
    std::vector<int32_t> m_dbWalkSibling;    // G10: Stand_Run_Loops database index -> the Stand_Walk_Loops database of the same LOD (-1 = none)
    std::vector<uint8_t> m_clipWalkToRun;    // G10: clip index -> 1 = M_Neutral_Transition_Walk_to_Run_L/Rfoot
    std::vector<uint8_t> m_dbSlideExit; // fix/slide-exit: database index -> 1 = PSD_Slide_ExitTo*
    std::vector<uint8_t> m_dbFadeTo;    // footsink2: database index -> 1 = ground locomotion (BlendRequest::groundFade, incoming side)
    std::vector<uint8_t> m_dbFadeFrom;  // footsink2: database index -> 1 = ground locomotion, landing or slide exit (outgoing side)
    std::vector<uint8_t> m_dbTurnInPlace; // footsink2: database index -> 1 = *TurnInPlace*
    float m_slideExitHoldLeft = 0.0f;   // fix/slide-exit: seconds the selected exit clip is still kept (MatcherConfig::slideExitMinHoldS)
    float m_sinceJumpS = 1.0e9f;        // seconds since the playing clip was selected (MatcherConfig::freshEventS, loopDwellS)
    float m_slideExitAge = 1.0e9f;      // fix/slide-exit: seconds since an exit clip was selected (MatcherConfig::slideExitBiasS)
    int m_holdStance = 0, m_holdGait = 0, m_holdMovementState = 0, m_holdFlags = 0; // fix/slide-exit: chooser state when the exit clip was selected
    float m_holdChangeAge = 0.0f;       // fix/slide-exit: seconds the gait / movementState has differed from the hold start
    std::vector<uint8_t> m_dbSlide;     // fix/slide-exit: database index -> 1 = PSD_Slide_FeetOut / KneesOut (not ExitTo)
    std::vector<uint8_t> m_clipSlideFamily; // fix/slide-exit: clip index -> 1 FootOut, 2 KneesOut, 0 other
    std::vector<int> m_hipsChain;           // fix3/jump: bones from the skeleton root to Hips (HipsHeightM)
    std::vector<uint8_t> m_clipSlideLoop;   // fix/slide-exit: clip index -> 1 = Slide_*_Loop (the slide loop, flagged as a loop or not)
    std::vector<uint8_t> m_clipSlideInto;   // RC2 / RC3: clip index -> 1 = Slide_FootOut_Into_* / Slide_KneesOut_Into_*
    bool m_slideEntryCap = false;           // RC2: this update is the first Sliding update after a non-slide clip (MatcherConfig::slideEntryMaxS)
    int32_t m_slideHandoverDb = -1;         // RC3: database whose same-family Loop clip takes over an exhausted Into clip (-1 = none)
    float m_riseHoldLeft = 0.0f;            // CR-4: seconds the selected Crouch -> Stand clip is still kept (MatcherConfig::crouchToStandHoldS)
    int32_t m_riseHoldClip = -1;            // CR-4: the held clip
    int m_riseHoldFlags = 0;                // CR-4: HoldFlags when it was selected
    std::vector<uint8_t> m_clipExitTransition; // exitTransitionTailS: clip index -> 1 = non-looping "VX_*" clip
    int32_t m_exitHoldClip = -1;            // exitTransitionTailS: the held VX_ clip (-1 = none)
    int32_t m_stopHoldClip = -1;            // stopCommitTailS: the held Stop clip (-1 = none)
    int m_stopHoldStance = 0, m_stopHoldFlags = 0; // stopCommitTailS: stance / HoldFlags when it was selected
    float m_stopHoldStart = 0.0f;           // stopCommitMaxS: clip time of the held Stop clip at its selection
    std::vector<float> m_clipTurnEndS;      // turnCommit: clip index -> clip time where the authored root turn is done (-1 = not a turn clip)
    int32_t m_turnHoldClip = -1;            // turnCommit: the held turn clip (-1 = none)
    float m_turnTailS = 0.0f;               // turnCommitTailS: seconds the held turn clip has played past its turn end
    bool m_downshiftDeferred = false;       // downshiftDeferS: a downshift edge was deferred and is still to be delivered
    int m_turnHoldStance = 0, m_turnHoldFlags = 0; // turnCommit: stance / HoldFlags (pivot bit cleared) when it was selected
    bool m_turnHoldPivotDropped = false;    // turnCommit: isPivoting dropped since the selection (a rise afterwards is a new reversal)
    int m_exitHoldStance = 0, m_exitHoldFlags = 0; // exitTransitionTailS: stance / HoldFlags when it was selected
    int m_slideExitFamily = 0;          // fix/slide-exit: family the exit search of this update is restricted to (0 = none)
    bool m_slideExitSettled = false;    // fix/slide-exit: this update is the first after a slide that had settled (slideExitSettledS)
    std::vector<float> m_raw;
    std::vector<float> m_query;
    std::vector<HistEntry> m_history;
    uint32_t m_numHist = 0;
    double m_now = 0.0;
    int32_t m_completedNoReselectClip = -1;
    int32_t m_completedNoReselectStance = -1;
    int m_observedStance = -1;
    float m_stanceEntryRemaining = 0.f;
    bool m_stanceObserved = false, m_stanceGround = false, m_stanceEntryAllowed = false;
    struct ContCand
    {
        int32_t db = -1;
        int32_t localRow = -1;
        uint32_t entry = 0;
        bool searched = false; // continuing cost evaluated (not force-interrupted)
    };
    Cur m_cur;
    float m_lastRateRatio = 1.0f; // diagnostic (MatchResult::playRateRatio)
    ContCand m_cand[chooser::kMaxResults + 1];
    uint32_t m_numCand = 0;
    bool m_candActive = false;
    ChooserState m_lastState;
    bool m_haveLastState = false;
    MatchResult m_last;
    const TrajSample* m_traj = nullptr;
    uint32_t m_numTraj = 0;
    const PoseHistory* m_hist = nullptr;
};

} // namespace utpp::mm::motion

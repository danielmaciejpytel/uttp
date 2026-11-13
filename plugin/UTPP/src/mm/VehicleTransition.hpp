// UTPP vehicle pose transitions (mount pop CAR-5 / MOTO-3, exit hand-in MOTO-6 / CAR-1 step 5). Header-only, no RED4ext
// dependency, unit-tested offline (tests/MMVehicleTransitionTest.cpp).
//
// MOUNT. The vehicle bit cuts the MM weight to 0 on one sample (MMDriver mount cut) and the game workspot moves the entity to
// the seat entry spot in the same or the next sample, so the body pops from the MM pose to the game entry pose with a root /
// yaw snap. BeginMount() starts a short write that runs at MM weight 0, only while the transition is active:
//   - hold: the frozen MM pose (the last published MM snapshot, blended onto the live game records with its own weight and
//     upper-body scale, exactly like BlendIntoRecords) for kMountHoldFrames distinct animation frames (capped at
//     kMountHoldMaxSeconds), which hides the 1-2 game frames before the vehicle animset override applies;
//   - then the live game pose plus per-bone local rotation offsets (target minus game, taken on the first post-hold frame):
//     q = pow(off, f) * qGame, f = 2^(-t / kMountHalfLife) * (1 - smoothstep(t / D)), D = rest of kMountSeconds, so the
//     offsets decay monotonically and are exactly 0 at kMountSeconds after BeginMount;
//   - Hips: the same decay on the hips translation offset, whose height (z) is always 0 (no 'standing over the bike');
//   - leg chains (LeftUpLeg..LeftToe, RightUpLeg..RightToe) get offsets only while their largest difference is below
//     kLegMaxOffsetDeg; a bent seated leg goes to the game pose at the end of the hold;
//   - root: a world jump of the entity (> kJumpDistance or > kJumpYawDeg between two UpdateWorld samples within
//     kWorldWatchSeconds of BeginMount) becomes a root offset on the Hips (translation x/y in the character frame + yaw about
//     +Z, OffsetRoot convention) equal to -(world jump), clamped to kRootMaxDistance / kRootMaxYawDeg, decaying by
//     smoothstep over kRootSeconds. The vertical part of a jump is never compensated (hips height offset 0).
//   After the last offset reaches 0 ApplyIfActive returns 0 and leaves the records untouched (the epoch expires).
//
// EXIT. While the vehicle bit is set (and TPP is on) SetVehicleCapture(true, rig) makes CaptureIfWanted copy the live game
// local pose of the mapped bones (no write). On the falling edge BeginExit() returns the capture if it is younger than
// kCaptureMaxAgeSeconds and arms HandInBlend: the caller publishes MM at weight 1 at once (GateFader::Fill) and passes every
// MM snapshot it publishes through HandInBlend, which adds the inertialized offset (captured minus MM on the first call,
// half-life kHandInHalfLife, exactly 0 at kHandInSeconds) so the live post-exit game pose is never shown.
//
// THREADS. Game-thread API (BeginMount, UpdateWorld, SetVehicleCapture, BeginExit, HandInBlend, Abort, SetBoneRoles): one
// thread at a time (the MMDriver feed, under its lock). Animation-thread API (ApplyIfActive, CaptureIfWanted, FkSide,
// FkSideMapping): any number of threads, lock-free: the game thread publishes a Control block through its own SeqlockBuffer;
// the per-epoch animation state (frame count, offsets) is published through a second SeqlockBuffer by the one thread that wins a
// try-lock (a losing thread computes the same values locally for its call and drops them); captures go through a
// GamePoseMailbox. Expiry, a rig change and a perspective denial end the epoch with one compare-exchange.
//
// SEAT POSE SWITCH (2026-10-28 recording 11:49:37, bike mount 11:55:08.141). The game's vehicle graph swaps the seated pose in one
// frame (Switch on TPPRepresentation.IsActive, blend 0) when the vehicle camera turns third person: the hunched end of the first-person
// enter clip -> 'drive_slow_bike' of the third-person set, Hips +9 cm, Spine 17 deg / Spine3 29 deg, head +17 cm, 1.74 s after the
// mount cut (after the mount transition above has ended). BeginMount arms a one-shot guard for kSeatWatchSeconds: once the mounted
// body was the UTPP third-person one (g_mountTpp latched into g_seatTpp) and while no mount transition runs, the animation thread compares the live game pose with
// the previous frame's; a torso step (Hips translation >= kSeatMinHipsStep AND a Spine-chain rotation >= kSeatMinTorsoDeg above
// what the previous compared step's rate gives over this frame gap, AND the Hips step >= kSeatStepRatio x that) starts an
// inertialized hand-in from the previous frame's pose (offsets previous minus new on every mapped bone, translations included,
// decaying to exactly 0 at kSeatSeconds). Once per mount; it runs through the vehicle hold's permission veto like the mount
// transition. The enter clip's own motion (Hips <= 5.4 cm / torso <= 2.7 deg per frame, ~3.7 cm per 1/60 s) and the one-frame neck
// spike of the seating frame (Hips 0.2 cm) never trigger it. Review 2026-10-28: steps are compared as rates (a frame hitch on the
// enter clip, 8.7120 -> 8.8112 in 99 ms = Hips 21 cm / torso 9.5 deg, is the clip's motion, not a cut); without a previous rate
// (first comparison, or the previous gap > kSeatMaxGapSeconds) the absolute thresholds are per 1/60 s. Plain per-1/60 s
// thresholds would miss the swap after a 50 ms hitch (3.0 cm). FPP stays vanilla: the latch covers a momentary drop of the
// mounted-TPP flag around the seating frame (kSeatFppGraceSeconds); the flag off for longer, or off again after it was on at /
// after the fire (the player switched to FPP during the hand-in), ends the guard at once (no write from that frame on).
//
// SEATED NECK CUT (2026-10-29 recording 01:28:54, bike ride 1, pop at 01:29:44.617; again 18:22:46, 18:23:39.613). On the first bike
// mount of a game session the game's vehicle graph pops the upper body for one frame on the seating edge (PSM Vehicle 4 -> 1, log
// 44.616): Neck 29.6 / Neck1 34.6 / Head 23.2 deg local, Spine1 / Spine3 11 deg, arms and hands 17-35 deg, Hips 1 cm; the game pose
// returns to the clip by itself in ~0.23 s. The pop is added after the early mapping job: the mounted guards there (seat guard, pop
// filter) ran on every ride frame of the 18:22:46 session and did not see it (no frozen frame, no extra pop filter fire), while the
// trace of the pose the FK pass receives (s0) shows it. NeckGuard therefore runs on that pose (FkHook, before the FK pass, every write owner).
// The frame before the edge has every rotation of the role bones equal to the previous one up to the sign and float noise (<= 2.4e-7
// per component: the graph's state switch; 44.601 / 59.090, 18:23:39.598 / 54.080); the guard takes the latest such frozen frame as the
// seating edge (native proxy: the PSM vehicle value is not fed to the native side). Within kSeatNeckWindowSeconds after it, a
// neck-chain step (largest Neck / Neck1 / Head step above the previous compared step's rate x this gap) >= kSeatNeckMinDeg AND >=
// kSeatStepRatio x that expectation, in TPP, fires once per mount: D_b = previous frame's rotation (-) live rotation on every upper-body
// bone (Spine1..Head, shoulders, arms, forearms, hands); each frame the shown rotation is D_b^(r g) * live_b with r = the remaining game
// deviation of the neck chain from the pre-pop pose along the pop (sum over Neck / Neck1 / Head of (live * pre^-1) . pop / sum of
// |pop|^2, rotation vectors, pop = live_fire * pre^-1; clamped 0..1: the correction follows the game's own return and never pushes past
// it; an unsigned angle ratio also counted the clip's own progress away from the pre-pop pose, r 0.34 at +0.17 s against 0.07 along the
// pop: 12.6 deg overshoot in the replay) and g = 1 - smootherstep(t / kSeatNeckSeconds) (hard end). Rotations only. The enter clip's
// own neck steps are <= 2.2 deg per frame. An FPP frame after the fire ends it. Mutation 19: no neck guard.
//
// EXIT LATCH (recording 2026-10-28 19-55-01, bike exits 17.268 / 32.254). The vehicle hold withdraws the pose-write permission for the
// whole ride and exit, so captures run on the denied frames too (after the mounted guards: the shown pose). Two animation frames before
// the vehicle-bit edge the game swaps the standing end of the exit clip (Hips 1.011 m) to its on-foot squat (Hips 0.337 m, then knees
// 125 deg); MM is at weight 0 there. A break of a standing ride capture (previous Hips >= kStandHipsZ, then a Hips drop > kLatchHipsDrop
// or a Hips step > kLatchHipsStep per 1/60 s) freezes the last capture and holds it on the records (translations included). BeginExit
// hands the held capture in and pins the hold; it ends on the first allowed frame after MMDriver published the hand-in snapshot (the
// first MM write), after kExitHoldMaxSeconds at most. An unpinned hold (no edge) ends when the capture is kCaptureMaxAgeSeconds old,
// through an inertialized release (kLatchReleaseSeconds). The seated ride (Hips ~0.1 m) never latches; rotation pops stay with the
// pop filter. ExitWantsFill: a third-person ride under the vehicle hold without a capture at the edge snaps MM in (GateFader::Fill),
// no fade through the game's post-exit pose.
//
// STANDING ENTRY (bike, recording 19-55-01 mount 07.853 / 22.338). The game Hips at the first post-hold frame >= kStandHipsZ (bike
// third-person entry frame 0: 1.004 m; the Archer car entry 0.973 m - car entries are standing too, only a seat pose is low)
// selects: leg chains keep their offsets up to kLegMaxOffsetStandingDeg
// (recorded knee difference 35-39 deg), the leg offsets decay over kLegDecaySeconds and finish within kLegFinishSeconds after the
// game legs start moving (the enter clip holds frame 0 during the workspot slide); the root follows the cumulative entity change
// (sub-threshold samples, recorded +12.9 / +3.8 deg): once it passes kJumpYawDeg / kJumpDistance within kRootTrackSeconds, the
// visual root stays where it was on the previous sample; afterwards the offset decays by time over kRootTailSeconds, the yaw part
// also with the entity turning back (the clip's root-yaw progress). A per-sample jump keeps the old jump path.
// YAW ALIGN (recording 2026-11-01 07-50-12, car ride 3, rootPeakYaw 63.4 deg; bike 07-47-01 ride 2, 88.4 deg). The vanilla workspot
// turns the entity ~75 deg onto the entry slot in ~0.28 s while V starts parallel to the vehicle. The cumulative hold kept V
// parallel for 0.45 s (the door opening) and released the yaw over the next 0.6 s while the clip turned back: ~12 deg pop, hold,
// +23 deg toward the door, -40 deg back - a spin about her axis. Once the held yaw |entity - anchor| passes kAlignYawDeg the yaw
// part is no longer held: it blends to the entity over kAlignSeconds (YawFactor, smootherstep, evaluated per animation frame from
// the latch; factor 1 at the latch = no step), so V turns monotonically toward the entry pose while the door opens and the tail has
// no yaw offset left. The planar part is unchanged. Entries that stay below kAlignYawDeg (ride 2 8.7 deg, the c9 bike entries
// 12.9 / 3.8 deg) keep the hold. The engage stays cumulative: the standing gate is only known at the first post-hold frame (+34 ms),
// so the first possible engage sample is the third one (+49 ms), where ride 3 / the bike already pass kJumpYawDeg (18.8 / 20.1 deg);
// a faster engage on the pivot rate (design option) would move no anchor there and would catch the 211-228 deg/s of the c9 bike's
// 1.9 deg steps. Cars only (BeginMount aCar, from the vehicle class at the MountingEvent): the bike entries passed the in-game
// checks (c9 / c10) with the hold and keep it bit-identically. Mutation 20: the old hold (no yaw align).
//
// HOLD CAP (recording 2026-11-02 02:01-02:50, car doors). The
// standing-entry hold keeps the RENDERED root at the pre-mount spot (cumulative path: kRootTrackSeconds + kRootTailSeconds; jump path:
// kRootSeconds from the jump sample) while the game opens the door; when that spot lies in the door's swept sector the door swings through
// the body (hella ride 4 hip, bratsk ride 30 head); the entity root itself is clear. The door script (utpp_vehicle_door.reds) knows the
// door arc and V's pre-mount position and calls SetHoldCap(seconds after BeginMount by which the visible root must be back on the
// entity path; < 0 = none). Only the PLANAR offset is capped (PlanarFactor); the yaw part, the yaw align, the leg offsets and the epoch
// lifetime keep their own timing. Cars only (BeginMount aCar); bikes and a mount without a cap are bit-identical to the code before.
//   * arrival: before BeginMount (pending slot, consumed by BeginMount when at most kHoldCapMaxAgeSeconds old) or up to that long
//     after it; applied on the NEXT UpdateWorld sample, never retroactively: the capped factor is min(natural, release curve) and the
//     release curve is 1 until its start, so a late cap cannot step the visible root. A second cap for a mount that already has an
//     applied cap is ignored (counted in g_capSecondIgnored).
//   * gate: it acts only if the hold carried the visible root >= kHoldCapGateDistM away from the entity within kHoldCapGateWithinSeconds
//     (a fast, large vanilla slide: rides 4 / 9 / 30; the small holds of rides 15 / 28 / 29 / 31 / 32 keep the natural release, and so
//     does everything else). The gate distance and window are fitted to the 30 recorded rides, not derived.
//   * cumulative path: the release (smoothstep) starts at max(sample time, min(kRootTrackSeconds, cap - kHoldCapTailSeconds)) and lasts
//     clamp(cap - start, kHoldCapMinReleaseSeconds, kRootTailSeconds), so it ends by the cap when that leaves the minimum release.
//     The planar offset is FROZEN in the world at the first sample at or after the release start (g.capFrozen, capFx / capFy; the same
//     "fixed in the world, decays by time" semantics as after the tracking window): from there the visible root is the entity plus the
//     frozen vector times the release curve. Tracking the anchor on (offset = anchor - entity, which keeps growing while the vanilla slide
//     runs) made the release collapse a 0.8 m offset in the 0.15 s minimum tail: the camera follows the ENTITY (utpp_camera pivot =
//     player:GetWorldPosition()), so on screen the body drifted off the framing while it was held and then jumped back in one video frame
//     (columbus 'door' ride 2026-11-02 11:16:39: 4.8 m/s). The frozen offset is the hold at the release start (>= kHoldCapGateDistM, about
//     0.25-0.3 m for a gated ride), so the shrink rate is at most 1.5 * offset / tail (about 2.5 m/s) and the visible root moves with the
//     entity from the release start on.
//   * jump path: the decay (natural kRootSeconds from the jump sample) is shortened to clamp(cap - sample time, min release, ..) only if
//     the offset is >= kHoldCapJumpGateDistM and that is at least kHoldCapJumpGainSeconds shorter than the natural decay (the recorded
//     jump-path rides lag 0.16-0.5 m: nothing to cap). These gates decide once per mount: once the cap is applied, later jump samples
//     (which rebase the offset onto the capped visible root) only restart the release from the sample with tail max(min release,
//     cap - sample time), so it still ends by max(cap, sample time + min release) and the cap is never dropped.
//
// GLIDE LIMIT (report 2026-11-03, car entries from the driver side: V slides 1-2 m toward the rear of the car and through its side). The
// vanilla workspot moves the entity to the clip start in a straight line with a sin ease over ~0.30 s whatever the distance (the clip start of a
// driver is 1.4-2.2 m behind the pre-mount spot: 7-11 m/s at the start of the ease). The hold above hid only part of it: the jump path clamped
// the held offset at kRootMaxDistance (1.2 m), re-based only inside kWorldWatchSeconds and released it over a fixed kRootSeconds, so the rest of
// the slide was shown at engine speed; the cumulative path clamped at 1.2 m too and released over a fixed kRootTailSeconds (0.5-1.5 m shrinking in
// 0.6 s = 1-4 m/s). For a car mount WITHOUT a door cap (no door arc to beat: parallel / behind / passenger approaches) the glide limit
//   * raises the planar clamp to kGlideMaxDistance;
//   * jump path: further samples (also the ones below kJumpDistance and the ones after the watch window) keep re-basing the PLANAR part onto the visible
//     root while the entity moves faster than kGlideFollowSpeedMps (the slide, not the clip's walk) up to kGlideFollowSeconds, so the body stays where
//     it was while the entity slides; the yaw part is not re-based there (Control::yawTicks: it keeps its own decay clock, the entity's pivot goes on
//     as before);
//   * releases the planar offset (jump decay / cumulative tail) over D = clamp(1.5 d / kGlideSpeedMps, kBase, kGlideEndSeconds - start) seconds,
//     d = the held planar offset: the smoothstep's peak closing speed is 1.5 d / D <= kGlideSpeedMps (a brisk walk, 1.4-2.0 m/s) wherever the
//     time budget allows. D is stored per mount in Control::planarS and used by the planar factor and the epoch lifetime only: the yaw part,
//     the yaw align and the leg offsets keep their own timing, so the epoch ends when the longest of them ends. The planar clamp stays at
//     kGlideMaxDistance for the rest of the mount once the limit was active (a cap arriving later must not step a 2 m hold down to 1.2 m): the cap's
//     min() releases it. Diagnostics: g_glidePlanarS (last planar release duration), g_glideFollows (slide-follow samples).
// A cap that arrives AFTER the glide started (up to kHoldCapMaxAgeSeconds after BeginMount; glide is off from the next sample, the clamp stays) is judged against
// the glided release: the jump gate (ApplyJumpCap) compares the cap with planarS minus the age of the last re-base instead of kRootSeconds, a later re-base keeps a
// release bounded by the cap (min(kGlideEndSeconds, cap) - sample time, never below kRootSeconds) instead of zeroing planarS, and the cumulative latch skips a cap only
// when it lies behind kRootTrackSeconds + planarS (its tail is clamped to planarS, not kRootTailSeconds). The glided cumulative tail (to kGlideEndSeconds) keeps being
// re-published from the world-fixed peak (trackOn window kRootTrackSeconds + max(kRootTailSeconds, planarS)). Mounts without a glided release (planarS 0) are unchanged.
// Mutations 27-30 -> (G).
// A mount WITH a door cap keeps the pre-glide behaviour (the door arrives on its own clock: the cap releases the root by then whatever the
// speed). Bikes (aCar false) never glide. Mutation 26: the old fixed hold.
//
// ON FOOT. Without a Begin* / SetVehicleCapture call Wanted() is false: FkSide / ApplyIfActive return 0 before touching any
// record, CaptureIfWanted copies nothing.
//
// STORAGE. The state lives in inline variables of this header (as mm/GamePoseCapture.hpp does), so every executable that
// compiles MotionHost.cpp links without stubs; FkHook.cpp only calls FkSide, PoseHook.cpp only FkSideMapping (main mapping job).
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>

#include "../MutationSelect.hpp" // tests only: a runtime-selected mutation (UTPP_RUNTIME_MUTATION_VEHTRANS); production keeps the constexpr constant
#include "GameIntegration.hpp"
#include "GamePoseCapture.hpp"

#ifndef UTPP_VEHTRANS_MUTATION
// tests only: 1 = mount blend off (old cut), 2 = root offset sign flipped, 3 = hips height kept, 4 = every mapping job writes
// (no main-job filter, the helper jobs of the same frame apply the transition again), 5 = no bone roles from the game rig (the
// per-bone fallback for the 25 deg leg rule), 6 = a perspective denial always ends the mount transition (the vehicle hold's veto
// of the CET pose-write permission kills it one frame after the cut), 7 = no seat pose switch guard (the game's one-frame swap
// to the third-person seated pose is shown as is), 8 = raw per-frame steps in the seat guard and the pop filter (no frame-gap
// rates; a pop may re-fire on every frame), 9 = the seat guard follows its TPP latch only (no current mounted-TPP check),
// 10 = no capture under a perspective denial, 11 = no exit latch (the squat swap before the edge is captured / shown), 12 = no
// Fill for a third-person exit without a capture, 13 = the standing leg cap without the standing gate (also seated entries),
// 14 = no standing entry path (legs 25 deg cap, common decay), 15 = no cumulative root tracking, 16 = no Hips translation term in
// the pop filter, 17 = the planar root offset of a standing entry decays with the root-yaw progress (old c9 rule), 18 = the leg
// offsets of a standing entry keep a residual (30 %) until the game legs move (never released while the enter clip holds frame 0),
// 19 = no seated neck cut (NeckGuard never armed: the game's one-frame upper-body pop on the seating edge is shown as is),
// 20 = the old cumulative yaw hold (no yaw align: the yaw offset is released by the tail only),
// 22 = the per-mount hold cap (SetHoldCap, HOLD CAP above) is ignored, 23 = the rendered Hips root offset ignores the cap (natural factor),
// 24 = the jump path asks its cap gates again at every jump sample, 25 = the capped cumulative release keeps tracking the anchor (the
// planar offset grows with the sliding entity during the release instead of being frozen at its release-start value),
// 26 = no car glide limit (GLIDE LIMIT: 1.2 m clamp, no slide follow, fixed 0.30 / 0.60 s releases)
#define UTPP_VEHTRANS_MUTATION 0
#endif

#if defined(_WIN32)
union _LARGE_INTEGER;
extern "C" __declspec(dllimport) int __stdcall QueryPerformanceFrequency(union _LARGE_INTEGER* lpFrequency);
#endif

namespace utpp::mm::vehtrans
{
#ifdef UTPP_RUNTIME_MUTATION_VEHTRANS // tests only (tests/MMVehicleTransitionTest.cpp runtime mutants): the number comes from UTPP_MUTATION_VEHTRANS
inline constexpr mutsel::RuntimeValue<2> kMutation{"UTPP_MUTATION_VEHTRANS"};
#else
inline constexpr int kMutation = UTPP_VEHTRANS_MUTATION;
#endif

inline constexpr uint32_t kMountHoldFrames = 2;          // distinct animation frames with the frozen MM pose
inline constexpr float kMountHoldMaxSeconds = 0.10f;     // hold cap (long frames / missed calls)
inline constexpr float kNewFrameGapSeconds = 0.003f;     // calls closer than this belong to the same animation frame
inline constexpr float kMountSeconds = 0.20f;            // rotation / hips offsets are 0 from here (since BeginMount)
inline constexpr float kMountHalfLife = 0.06f;
inline constexpr float kLegMaxOffsetDeg = 25.0f;
inline constexpr float kRootSeconds = 0.30f;             // root offset smoothstep decay (since the jump sample)
inline constexpr float kRootMaxDistance = 1.2f;          // m
inline constexpr float kRootMaxYawDeg = 120.0f;
inline constexpr float kWorldWatchSeconds = 0.15f;       // UpdateWorld jump detection window after BeginMount
inline constexpr float kJumpDistance = 0.15f;            // m, planar step between two samples
inline constexpr float kJumpYawDeg = 15.0f;
inline constexpr float kCaptureMaxAgeSeconds = 0.10f;
inline constexpr float kHandInHalfLife = 0.08f;
inline constexpr float kHandInSeconds = 0.25f;
// seat pose switch guard (see SEAT POSE SWITCH above)
inline constexpr float kSeatWatchSeconds = 4.0f;  // armed from BeginMount (recorded switch: 1.74 s)
inline constexpr float kSeatMinHipsStep = 0.04f;  // m above the motion (recorded: 9.1 cm; enter clip <= 5.4 cm per frame)
inline constexpr float kSeatMinTorsoDeg = 8.0f;   // largest Spine-chain step above the motion (recorded: 28.6 deg; clip <= 2.7)
inline constexpr float kSeatStepRatio = 3.0f;     // Hips step vs the previous step's rate x this gap (a cut, not a fast motion)
inline constexpr float kSeatMaxGapSeconds = 0.10f; // previous pose older than this is not compared (stalled frames)
// hand-in f = 1 - smootherstep(t / kSeatSeconds): zero speed at the fire frame and at the end, peak 1.875 / kSeatSeconds of the step
// (9.1 cm -> 38 cm/s, 0.63 cm per 1/60 s); 50 % at 0.5 kSeatSeconds. Was exp2(-t / 0.10) x (1 - smoothstep(t / 0.35)): 63 cm/s on the
// first frame, 90 % done at 0.20 s (seen as a quick straightening in the 2026-10-28 recording).
inline constexpr float kSeatSeconds = 0.45f;
// mounted-TPP flag off for longer than this ends the guard. With utpp_camera's vehicle hold (the TPP mount path) the TPP signal stays
// on through the vehicle FPP decision of the seating frame (log 11:55:09.593 'decision FPP (vehicle) ignored', MMTRACE tpp=1 at 09.396
// and 10.411); the latch covers a drop of the flag there (the decision lasted 0.33 s: 09.578 'FPP' .. 09.910 'TPP', swap at 0.30 s)
inline constexpr float kSeatFppGraceSeconds = 0.5f;
// seated neck cut (see SEATED NECK CUT): recorded pop Neck1 34.6 deg in one frame, enter clip <= 2.2 deg per frame
inline constexpr float kSeatNeckMinDeg = 15.0f;          // per 1/60 s without a previous rate, else above the rate x gap
inline constexpr float kSeatNeckWindowSeconds = 0.15f;   // fire only this long after the seating edge (frozen frame)
inline constexpr float kSeatNeckSeconds = 0.35f;         // hard end of the correction (g = 1 - smootherstep(t / this))
inline constexpr float kSeatEdgeFrozenEps = 1e-5f;      // largest quaternion component change of a frozen (state switch) frame
// standing game pose (Hips height above the root): bike third-person entry frame 0 1.004 m, exit clip end 1.011 m
inline constexpr float kStandHipsZ = 0.85f;
// exit latch (see EXIT LATCH)
inline constexpr float kLatchHipsDrop = 0.20f;      // m per 1/60 s (recorded swap: 67.4 cm in one frame)
inline constexpr float kLatchHipsStep = 0.15f;      // m per 1/60 s
inline constexpr float kExitHoldMaxSeconds = 0.25f; // pinned hold cap (recorded: edge 17.304, permission back 17.317)
inline constexpr float kLatchReleaseSeconds = 0.40f; // 30 cm -> <= 2.3 cm per 1/60 s (1 - smootherstep)
// standing entry (see STANDING ENTRY)
inline constexpr float kLegMaxOffsetStandingDeg = 60.0f;
// recorded: legs on enter frame 0 for 0.28-0.30 s after the cut (c8b). The c9 clips hold it to +0.33 / +0.35 s after BeginMount
// (2026-10-28 23:05 trace), so the decay ends first (legMoveAt=-1, 2-3 still frames before the clip); a longer decay would write
// past the 0.30 s mount window of the car path, so it stays
inline constexpr float kLegDecaySeconds = 0.28f;
inline constexpr float kLegFinishSeconds = 0.10f; // rest of the leg offsets once the game legs move
inline constexpr float kLegMoveDeg = 0.5f;        // largest leg step of a moving frame (frame 0 held: 0)
inline constexpr float kRootTrackSeconds = 0.45f; // cumulative entity change watched / followed from BeginMount
// yaw align (see YAW ALIGN): latch / blend of a large held yaw
inline constexpr float kAlignYawDeg = 30.0f;
inline constexpr float kAlignSeconds = 0.35f; // the door opens at +0.3-0.4 s, the clip's back-turn starts at ~+0.45 s
// then the offset decays by time - the epoch ends <= 1.05 s (before the ride; the seated ride never carries a root offset).
// Planar part: time only (peak speed 1.5 x distance / tail: 0.68 m -> 1.7 m/s, 2.8 cm per 1/60 s); recorded 2026-10-28 23:05
// ride 2 with the yaw-progress factor on top: both feet carried 4-7 cm per frame during the first step. Yaw part: also the
// clip's root-yaw progress.
inline constexpr float kRootTailSeconds = 0.60f;
// hold cap (see HOLD CAP): per-mount limit on the PLANAR part of the standing-entry root offset, set by the door script
inline constexpr float kHoldCapMaxAgeSeconds = 0.25f;   // a cap is accepted from this long before BeginMount until this long after it
inline constexpr float kHoldCapTailSeconds = 0.25f;     // preferred release length of a capped hold (it is moved earlier, never longer than the natural tail)
inline constexpr float kHoldCapMinReleaseSeconds = 0.15f; // shortest release: a faster glide to the entity would skate (ride 14: 8.9 m/s accepted)
inline constexpr float kHoldCapGateDistM = 0.25f;       // the cap acts only on a hold that carried the visible root this far ...
inline constexpr float kHoldCapGateWithinSeconds = 0.13f; // ... within this long after BeginMount (a fast, large vanilla slide)
inline constexpr float kHoldCapJumpGainSeconds = 0.10f; // jump path: the cap acts only if it shortens the natural release by at least this
inline constexpr float kHoldCapJumpGateDistM = 0.60f;   // jump path: and only for a lag this large (rides 23 / 26 lag 0.5 m over two jump samples, no hold to cap)
// car glide limit (see GLIDE LIMIT)
inline constexpr float kGlideSpeedMps = 2.0f;        // peak closing speed of the held planar offset (smoothstep peak 1.5 d / D): a brisk walk (1.4 m/s walk, 1.8-2.0 brisk); holds up to 0.8 m keep the old 0.6 s tail
inline constexpr float kGlideMaxDistance = 3.0f;     // m, planar clamp of a car hold without a door cap (kRootMaxDistance saturates at the 1.4-2.2 m driver slides)
inline constexpr float kGlideFollowSeconds = 0.35f;  // jump path of a car: later samples still re-base while the vanilla slide runs (it ends at T = 0.30 s) ...
inline constexpr float kGlideFollowSpeedMps = 2.0f;  // ... and the entity moves faster than this (the slide starts at 5-11 m/s; the clip root cannot move before T + 0.02 s)
inline constexpr float kGlideEndSeconds = 1.50f;     // the planar offset is 0 by then: the 2.0 s enter clip's root correction runs to ~1.9 s after the mount
// pop filter Hips translation term: fires when the Hips step exceeds its own motion over the gap by this much
inline constexpr float kPopHipsM = 0.03f;

// ------------------------------------------------------------------------------------------------ small math (xyzw)
namespace detail
{
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kDeg = kPi / 180.0f;

inline void QMul(const float a[4], const float b[4], float o[4])
{
    const float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    const float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    const float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    o[0] = x; o[1] = y; o[2] = z; o[3] = w;
}
inline bool QNormalize(float q[4])
{
    const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!(n2 > 1e-20f) || !std::isfinite(n2))
        return false;
    const float inv = 1.0f / std::sqrt(n2);
    for (int k = 0; k < 4; ++k)
        q[k] *= inv;
    return true;
}
inline bool Finite(const float* v, int n)
{
    for (int k = 0; k < n; ++k)
        if (!std::isfinite(v[k]))
            return false;
    return true;
}
// Rotation angle (rad) between two unit quaternions.
inline float QAngle(const float a[4], const float b[4])
{
    float d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    d = d > 1.0f ? 1.0f : d;
    return 2.0f * std::acos(d);
}
// q^f for a unit quaternion: same axis, angle scaled by f in [0, 1] (shortest arc).
inline void QPow(const float q[4], float f, float o[4])
{
    float x = q[0], y = q[1], z = q[2], w = q[3];
    if (w < 0.0f) { x = -x; y = -y; z = -z; w = -w; }
    const float s = std::sqrt(x * x + y * y + z * z);
    if (!(f > 0.0f) || s < 1e-9f)
    {
        o[0] = 0.0f; o[1] = 0.0f; o[2] = 0.0f; o[3] = 1.0f;
        return;
    }
    const float half = std::atan2(s, w) * f;
    const float k = std::sin(half) / s;
    o[0] = x * k; o[1] = y * k; o[2] = z * k; o[3] = std::cos(half);
}
// Rotation vector (axis x angle, rad, shortest arc) of a unit quaternion.
inline void QRotVec(const float q[4], float o[3])
{
    float x = q[0], y = q[1], z = q[2], w = q[3];
    if (w < 0.0f) { x = -x; y = -y; z = -z; w = -w; }
    const float s = std::sqrt(x * x + y * y + z * z);
    const float k = s < 1e-9f ? 2.0f : 2.0f * std::atan2(s, w) / s;
    o[0] = x * k; o[1] = y * k; o[2] = z * k;
}
inline float Smoothstep(float x)
{
    const float s = x <= 0.0f ? 0.0f : (x >= 1.0f ? 1.0f : x);
    return s * s * (3.0f - 2.0f * s);
}
// 6s^5 - 15s^4 + 10s^3: zero first and second derivative at both ends.
inline float Smootherstep(float x)
{
    const float s = x <= 0.0f ? 0.0f : (x >= 1.0f ? 1.0f : x);
    return s * s * s * (s * (s * 6.0f - 15.0f) + 10.0f);
}
inline float WrapPi(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::fmod(a + kPi, 2.0f * kPi);
    if (a < 0.0f)
        a += 2.0f * kPi;
    return a - kPi;
}
// Character frame (x right, y forward; GameIntegration yaw convention) -> world planar.
inline void CharacterToWorld(float aX, float aY, float aYaw, float& aOutX, float& aOutY)
{
    const float c = std::cos(aYaw), s = std::sin(aYaw);
    aOutX = c * aX - s * aY;
    aOutY = s * aX + c * aY;
}
inline float Seconds(int64_t aTicks, int64_t aTps)
{
    return aTps > 0 ? static_cast<float>(static_cast<double>(aTicks) / static_cast<double>(aTps)) : 0.0f;
}
} // namespace detail

// ------------------------------------------------------------------------------------------------ clock
// QPC frequency. Tests store their own tick rate first; 0 = query once.
inline std::atomic<int64_t> g_ticksPerSecond{0};
inline int64_t TicksPerSecond()
{
    int64_t f = g_ticksPerSecond.load(std::memory_order_relaxed);
    if (f > 0)
        return f;
#if defined(_WIN32)
    int64_t q = 0;
    QueryPerformanceFrequency(reinterpret_cast<union _LARGE_INTEGER*>(&q));
    f = q > 0 ? q : 10000000;
#else
    f = 1000000000;
#endif
    g_ticksPerSecond.store(f, std::memory_order_relaxed);
    return f;
}

// ------------------------------------------------------------------------------------------------ bone roles
// Database bone roles. Without SetBoneRoles the fallback is: Hips = snapshot.firstBone, every other bone kept by MM under the
// upper-body mask (not in upperBits) is its own 'leg chain' for the 25 deg rule.
enum : uint8_t { kChainNone = 0, kChainLeftLeg = 1, kChainRightLeg = 2, kChainSingle = 3 };
struct BoneRoles
{
    int32_t hips = -1;
    uint32_t valid = 0;
    uint8_t chain[integ::kMaxMMBones] = {};
    uint8_t torso[integ::kMaxMMBones] = {}; // 1 = Spine, Spine1, Spine2, Spine3 (seat pose switch guard only)
    uint8_t neck[integ::kMaxMMBones] = {};  // 1 = Neck, Neck1, Head (seated neck cut: measured)
    uint8_t upper[integ::kMaxMMBones] = {}; // 1 = Spine1..Head, shoulders, arms, forearms, hands (seated neck cut: written)
};
static_assert(std::is_trivially_copyable_v<BoneRoles> && sizeof(BoneRoles) % 4 == 0);

// bones the seated neck cut writes (SEATED NECK CUT)
inline constexpr std::string_view kUpperNames[] = {"Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head",
                                                   "LeftShoulder", "RightShoulder", "LeftArm", "RightArm",
                                                   "LeftForeArm", "RightForeArm", "LeftHand", "RightHand"};

// aNameAt(uint32_t bone) -> std::string_view (database bone names, e.g. g_db.BoneName).
template <typename NameAt>
inline BoneRoles BuildBoneRoles(uint32_t aNumBones, uint32_t aFirstBone, NameAt&& aNameAt)
{
    BoneRoles r;
    if (aNumBones == 0 || aNumBones > integ::kMaxMMBones || aFirstBone >= aNumBones)
        return r;
    static constexpr std::string_view kLeft[] = {"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftHeel", "LeftToe"};
    static constexpr std::string_view kRight[] = {"RightUpLeg", "RightLeg", "RightFoot", "RightHeel", "RightToe"};
    for (uint32_t b = aFirstBone; b < aNumBones; ++b)
    {
        const std::string_view n = aNameAt(b);
        if (n == "Hips")
            r.hips = static_cast<int32_t>(b);
        for (const std::string_view p : kLeft)
            if (n.substr(0, p.size()) == p)
                r.chain[b] = kChainLeftLeg;
        for (const std::string_view p : kRight)
            if (n.substr(0, p.size()) == p)
                r.chain[b] = kChainRightLeg;
        if (n == "Spine" || n == "Spine1" || n == "Spine2" || n == "Spine3")
            r.torso[b] = 1;
        for (const std::string_view u : kUpperNames)
            if (n == u)
                r.upper[b] = 1;
        if (n == "Neck" || n == "Neck1" || n == "Head")
            r.neck[b] = 1;
    }
    r.valid = r.hips >= 0 ? 1u : 0u;
    return r;
}

// Bone roles of one snapshot from the GAME bone names its own map points at (aHashAt(int32_t gameIndex) -> CName hash of that
// game bone, 0 = unknown). The bone map is by name, so a v1 database and a v2 bundle (different bone indices, the same game
// names) both get the roles of the snapshot that is actually frozen. Exact rig names (Hips, Left/Right UpLeg, Leg, Foot, Heel,
// ToeBase, Toe). Game thread (MMDriver VtBeginMountLocked, before BeginMount).
template <typename HashAt>
inline BoneRoles BuildBoneRolesFromGame(const integ::PoseSnapshot& aSnap, HashAt&& aHashAt)
{
    BoneRoles r;
    UTPP_MUT_IF (kMutation == 5)
        return r; // mutation 5: the old production path (no roles, per-bone fallback)
    const uint32_t nb = aSnap.numBones <= integ::kMaxMMBones ? aSnap.numBones : integ::kMaxMMBones;
    if (aSnap.firstBone >= nb)
        return r;
    static const uint64_t kHips = integ::CNameHash("Hips");
    static const std::array<uint64_t, 6> kLeft = {integ::CNameHash("LeftUpLeg"), integ::CNameHash("LeftLeg"),
                                                  integ::CNameHash("LeftFoot"), integ::CNameHash("LeftHeel"),
                                                  integ::CNameHash("LeftToeBase"), integ::CNameHash("LeftToe")};
    static const std::array<uint64_t, 6> kRight = {integ::CNameHash("RightUpLeg"), integ::CNameHash("RightLeg"),
                                                   integ::CNameHash("RightFoot"), integ::CNameHash("RightHeel"),
                                                   integ::CNameHash("RightToeBase"), integ::CNameHash("RightToe")};
    static const std::array<uint64_t, 4> kTorso = {integ::CNameHash("Spine"), integ::CNameHash("Spine1"),
                                                   integ::CNameHash("Spine2"), integ::CNameHash("Spine3")};
    static const std::array<uint64_t, 3> kNeck = {integ::CNameHash("Neck"), integ::CNameHash("Neck1"), integ::CNameHash("Head")};
    static const auto kUpper = [] {
        std::array<uint64_t, std::size(kUpperNames)> a{};
        for (size_t i = 0; i < a.size(); ++i)
            a[i] = integ::CNameHash(kUpperNames[i]);
        return a;
    }();
    for (uint32_t b = aSnap.firstBone; b < nb; ++b)
    {
        const int32_t gi = aSnap.gameIndex[b];
        if (gi < 0)
            continue;
        const uint64_t h = aHashAt(gi);
        if (!h)
            continue;
        if (h == kHips && r.hips < 0)
            r.hips = static_cast<int32_t>(b);
        for (const uint64_t n : kLeft)
            if (h == n)
                r.chain[b] = kChainLeftLeg;
        for (const uint64_t n : kRight)
            if (h == n)
                r.chain[b] = kChainRightLeg;
        for (const uint64_t n : kTorso)
            if (h == n)
                r.torso[b] = 1;
        for (const uint64_t n : kNeck)
            if (h == n)
                r.neck[b] = 1;
        for (const uint64_t n : kUpper)
            if (h == n)
                r.upper[b] = 1;
    }
    r.valid = r.hips >= 0 ? 1u : 0u;
    return r;
}

// Main mapping job of the Mapping-owner write path (PoseHook ApplyEarlyMapping), the identity earlyruntime::EvaluateMapping
// admits: the source rig carries every main anchor (absent from the helper / deformation sources) and the job's pairs cover the
// destination of every snapshot bone from firstBone. Only such a job starts from records the original mapping has just
// overwritten with fresh game data; a helper job of the same frame rewrites helper destinations only, so the core records still
// hold this frame's transition write and a second write would stack the offsets and the root offset.
inline bool MainMappingJob(std::span<const uint64_t> aSourceNames, std::span<const uint32_t> aPairs, uint32_t aOutputCount,
                           const integ::PoseSnapshot& aMap)
{
    static const std::array<uint64_t, 5> kAnchors = {integ::CNameHash("reference_joint"), integ::CNameHash("WeaponLeft"),
                                                     integ::CNameHash("WeaponRight"), integ::CNameHash("LeftEye"),
                                                     integ::CNameHash("RightEye")};
    constexpr uint32_t kMaxDestinations = 1024; // earlyruntime::kMaxRigBones (PoseHook rejects larger jobs before this call)
    if (aSourceNames.empty() || aPairs.empty() || (aPairs.size() & 1u) || !aOutputCount || aOutputCount > kMaxDestinations)
        return false;
    for (const uint64_t a : kAnchors)
    {
        bool found = false;
        for (const uint64_t n : aSourceNames)
            if (n == a)
            {
                found = true;
                break;
            }
        if (!found)
            return false;
    }
    const uint32_t nb = aMap.numBones <= integ::kMaxMMBones ? aMap.numBones : integ::kMaxMMBones;
    if (aMap.firstBone >= nb)
        return false;
    uint64_t seen[kMaxDestinations / 64] = {};
    for (size_t i = 0; i + 1 < aPairs.size(); i += 2)
        if (aPairs[i + 1] < aOutputCount)
            seen[aPairs[i + 1] >> 6u] |= uint64_t{1} << (aPairs[i + 1] & 63u);
    for (uint32_t b = aMap.firstBone; b < nb; ++b)
    {
        const int32_t dst = aMap.gameIndex[b];
        if (dst < 0 || static_cast<uint32_t>(dst) >= aOutputCount ||
            !(seen[static_cast<uint32_t>(dst) >> 6u] & (uint64_t{1} << (static_cast<uint32_t>(dst) & 63u))))
            return false;
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ shared state
struct Control
{
    uint64_t epoch = 0;
    uint64_t rig = 0;
    int64_t beginTicks = 0;
    int64_t rootTicks = 0;     // jump sample; the root offset decays from here
    int64_t tps = 0;
    float rootX = 0.0f, rootY = 0.0f, rootYaw = 0.0f; // character frame of the post-jump entity, metres / rad
    uint32_t hold = 0;         // 1 = the frozen MM pose is valid (weight > 0, mapped bones)
    uint32_t rootMode = 0;     // 0 = none, 1 = jump (decay from rootTicks), 2 = cumulative tracking (time factor from beginTicks)
    int64_t alignTicks = 0;    // yaw align latch time (YAW ALIGN); the yaw offset blends to 0 over kAlignSeconds from here
    uint32_t yawAlign = 0;
    uint32_t pad2 = 0;
    int64_t capTicks = 0;      // hold cap (HOLD CAP): the planar release curve is 1 until here, then 1 - smoothstep over capTailS
    float capTailS = 0.0f;
    uint32_t capOn = 0;
    float planarS = 0.0f;      // GLIDE LIMIT: planar natural release duration (jump decay / cumulative tail), 0 = kRootSeconds / kRootTailSeconds
    int64_t yawTicks = 0;      // GLIDE LIMIT: jump path yaw decay clock when a slide-follow sample re-based the planar part only (0 = rootTicks)
    BoneRoles roles;
    integ::PoseSnapshot frozen; // also the bone map of the captures
};
static_assert(std::is_trivially_copyable_v<Control> && sizeof(Control) % 4 == 0);

struct AnimState
{
    uint64_t epoch = 0;
    int64_t lastFrameTicks = 0;
    int64_t offsetTicks = 0;
    int64_t legPrevTicks = 0;  // standing entry: previous frame of the live game legs
    int64_t legMoveTicks = 0;  // first frame the game legs moved (0 = not yet)
    uint32_t frames = 0;
    uint32_t offsetsReady = 0;
    uint32_t standing = 0;     // the first post-hold game pose was standing (STANDING ENTRY)
    uint32_t legMoving = 0;
    float legMoveF = 0.0f;     // leg factor on the first moving frame
    float hipsP[3] = {};
    float offQ[integ::kMaxMMBones * 4] = {};
    float legPrevQ[integ::kMaxMMBones * 4] = {}; // live game rotations of the leg bones, previous frame
};
static_assert(std::is_trivially_copyable_v<AnimState> && sizeof(AnimState) % 4 == 0);

struct Capture
{
    int64_t ticks = 0;
    integ::PoseSnapshot snap; // map of the Control block, pose = captured live game local pose
};
static_assert(std::is_trivially_copyable_v<Capture> && sizeof(Capture) % 4 == 0);

struct Counters
{
    std::atomic<uint64_t> mounts{0}, holdWrites{0}, offsetWrites{0}, rootOnlyWrites{0}, expired{0};
    std::atomic<uint64_t> rigAborts{0}, perspectiveAborts{0}, captures{0}, exitsHanded{0}, exitsStale{0};
    std::atomic<uint64_t> pops{0}, popWrites{0}, rearmHoldFrames{0}; // mounted pop filter / re-arm hold (2026-10-28 11-49-37)
    std::atomic<uint64_t> seatFires{0}, seatWrites{0};
    // the seated neck cut (NeckGuard) counts apart from the seat pose switch guard (2026-11-01: 'fires' summed both and the
    // last fire's Hips / torso fields were overwritten by whichever guard fired last)
    std::atomic<uint64_t> neckFires{0}, neckWrites{0};
    // diagnostics 2026-10-28 19-55-01 (EXITDIAG end line): where the pop filter / captures stop, exit latch, mapping jobs
    std::atomic<uint64_t> popCalls{0}, popSkipRig{0}, popSkipEpoch{0}, popSkipBusy{0}, hipsPops{0};
    std::atomic<uint64_t> fkSideDenied{0}, mapCalls{0}, mapRejects{0};
    std::atomic<uint64_t> exitBreaks{0}, exitHoldFrames{0}, exitHoldPins{0}, exitHoldEnds{0}, exitHoldReleases{0};
};
// counter snapshot (MMDriver prints the deltas of one ride / exit)
struct CounterSnap
{
    uint64_t captures, exitsHanded, exitsStale, pops, hipsPops, popWrites, popCalls, popSkipRig, popSkipEpoch, popSkipBusy;
    uint64_t fkSideDenied, mapCalls, mapRejects, exitBreaks, exitHoldFrames, exitHoldPins, exitHoldEnds, exitHoldReleases;
};

// Mounted pop filter (recording 2026-10-28 11-49-37, bike exit): while V sits on the vehicle in TPP the game's own exit animation
// jumps in single frames (48.110 right leg 66 deg, 48.596 arms 86 / 76 deg and knees 110 -> 155 deg when the game drops its TPP
// representation, 48.707 arm 71 / Spine3 45 deg when it is activated again); MM is at weight 0 there, the body is the game's. A live
// game step above kPopDeg on any mapped bone (a pop: the exit animation moves <= ~10 deg per frame) starts an inertialization from
// the pose shown on the previous frame: offset = shown x game^-1, decaying with kPopHalfLife to exactly 0 at kPopSeconds, on top
// of the live game pose. Frames without a pop and without a running offset are not written (the game records stay untouched).
// Review 2026-10-28: the step is measured against the bone's own motion over the frame gap (its rate on the last frame without a
// fire), so ordinary motion through a frame hitch (3 frames x 10 deg = one 30-40 deg step) is no pop; without a rate (first frame
// of a history) the threshold is per 1/60 s. A fire on the frame right after a fire only re-anchors when the new offset stays
// <= kPopDeg on every bone (a one-frame spike and back); sustained fast motion (> kPopDeg per frame at a low frame rate) is not
// frozen at the previous output (at most one frozen frame, then the motion's rate is learnt).
inline constexpr float kPopDeg = 30.0f;
inline constexpr float kPopHalfLife = 0.08f;
inline constexpr float kPopSeconds = 0.25f;
inline constexpr float kPopGapSeconds = 0.25f; // longer without a frame (other object, pause): the history starts again
struct PopState
{
    uint64_t rig = 0;
    int64_t lastTicks = 0, offTicks = 0;
    uint32_t valid = 0, offOn = 0;
    uint32_t rateOk = 0, firedPrev = 0; // rate[] is from a frame without a fire / the last distinct frame fired
    float prevGame[integ::kMaxMMBones * 4] = {};
    float prevOut[integ::kMaxMMBones * 4] = {};
    float offQ[integ::kMaxMMBones * 4] = {};
    float rate[integ::kMaxMMBones] = {}; // rad/s: the game step of the last frame without a fire / its gap
    uint8_t ok[integ::kMaxMMBones] = {};
    // Hips translation term (own fire state: a translation fire never suppresses a rotation fire of the next frame or back).
    // Decay 1 - smootherstep(t / kPopSeconds): zero speed at the fire, peak 1.875 / kPopSeconds of the step (a 23.9 cm one-frame
    // step -> 3.0 cm per 1/60 s; the rotation decay's first frame would move 14.6 % = 3.5 cm).
    int64_t offPTicks = 0;
    uint32_t hipsOk = 0, offPOn = 0, rateOkP = 0, firedPrevP = 0;
    float rateP = 0.0f; // m/s
    float prevHipsGame[3] = {}, prevHipsOut[3] = {}, offP[3] = {};
};

// Seat pose switch guard, per armed mount (animation thread, published by the try-lock owner).
struct SeatState
{
    uint64_t epoch = 0;        // Control epoch the guard belongs to
    int64_t prevTicks = 0;
    int64_t fireTicks = 0;
    int64_t fppTicks = 0;      // first frame of the current run with the mounted-TPP flag off (0 = on)
    uint32_t prevValid = 0;
    uint32_t fired = 0;
    uint32_t rateOk = 0;       // hipsRate / torsoRate are from the previous compared step
    uint32_t tppAfterFire = 0; // the mounted-TPP flag was on at / after the fire
    float hipsRate = 0.0f;     // m/s, previous compared Hips step / its gap
    float torsoRate = 0.0f;    // rad/s, previous compared Spine-chain step / its gap
    float prev[integ::kMaxMMBones * 7] = {}; // previous frame's live game pose (p.xyz q.xyzw) of the mapped bones
    float offQ[integ::kMaxMMBones * 4] = {};
    float offP[integ::kMaxMMBones * 3] = {};
    uint8_t offOk[integ::kMaxMMBones] = {};
};
static_assert(std::is_trivially_copyable_v<SeatState> && sizeof(SeatState) % 4 == 0);

// Seated neck cut, per armed mount (animation threads, under g_neckBusy).
struct NeckState
{
    uint64_t epoch = 0;
    int64_t prevTicks = 0, edgeTicks = 0, fireTicks = 0; // edgeTicks: latest frozen (state switch) frame, 0 = none yet
    uint32_t prevValid = 0, fired = 0, done = 0, rateOk = 0, wroteFrame = 0;
    int32_t refBone = -1;            // first neck bone of the correction (repeated-call check)
    float rate = 0.0f;               // rad/s, previous compared neck-chain step / its gap
    float ref = 0.0f;                // sum of |pop|^2 over the neck chain on the fire frame (rad^2)
    float refOut[4] = {};            // refBone's rotation as written on the last written frame
    float prevHips[3] = {};
    float prevQ[integ::kMaxMMBones * 4] = {}; // previous frame's live rotations
    float preQ[integ::kMaxMMBones * 4] = {};  // rotations of the frame before the fire
    float offQ[integ::kMaxMMBones * 4] = {};  // D_b = pre (-) live on the fire frame
    uint8_t prevOk[integ::kMaxMMBones] = {};
    uint8_t ok[integ::kMaxMMBones] = {};
};

inline integ::SeqlockBuffer<Control> g_control;
inline integ::SeqlockBuffer<AnimState> g_animState;
inline std::atomic_flag g_animWriter = ATOMIC_FLAG_INIT;
inline integ::GamePoseMailbox<Capture> g_captures;
inline integ::SeqlockBuffer<BoneRoles> g_roles;
inline std::atomic<uint64_t> g_activeEpoch{0}; // 0 = no mount transition
inline std::atomic<bool> g_captureOn{false};
inline std::atomic<uint64_t> g_captureRig{0};
// Game-thread flag (MMDriver feed): the mounted body is the UTPP third-person body (vehicle bit and TPP representation on). While
// it is set, a perspective denial does not end a running mount transition: during a vehicle ride utpp_camera's vehicle hold
// withdraws the pose-write permission on purpose (the seated game pose is wanted), 5-10 ms after the mount cut, and the
// transition (hold + decaying offsets, 0.2 s) is the one write that must still run through it. Captures stay gated.
inline std::atomic<bool> g_mountTpp{false};
inline Counters g_counters;
// Game-thread flags read by the animation threads: the mounted pop filter is on (MMDriver: vehicle and TPP); the re-arm hold is set
// (BeginRearmCapture: a same-rig re-arm in the jump-out flight) - FkSide keeps the live game pose (no MM) on those frames until the
// hand-in from the new capture is published (EndRearmHold).
inline std::atomic<bool> g_popOn{false};
inline std::atomic<bool> g_rearmHold{false};
inline PopState g_pop;                         // animation threads, under g_popBusy
inline std::atomic_flag g_popBusy = ATOMIC_FLAG_INIT;
inline integ::SeqlockBuffer<SeatState> g_seatState;
inline std::atomic_flag g_seatWriter = ATOMIC_FLAG_INIT;
inline std::atomic<uint64_t> g_seatEpoch{0}; // armed seat pose switch guard (Control epoch), 0 = off
inline std::atomic<uint64_t> g_neckEpoch{0}; // armed seated neck cut (Control epoch), 0 = off
inline NeckState g_neck;                      // animation threads, under g_neckBusy
inline std::atomic_flag g_neckBusy = ATOMIC_FLAG_INIT;
inline std::atomic<uint64_t> g_neckCalls{0}, g_neckRigSkips{0}; // NeckGuard calls past its gates / rig mismatches (diagnostics)
// latched: the mounted body was the UTPP third-person one (SetMountTpp(true)) at some point since BeginMount. The guard and its
// Wanted() term need it, so a first-person mount stays exactly as before; the latch also covers the vehicle perspective
// decision of the seating frame (recording 11:55:09.578 'FPP <- vehicle', 09.910 'TPP <- vehicle'), whatever the TPP signal does there
// (a drop of the mounted-TPP flag up to kSeatFppGraceSeconds; longer, or after the fire was seen in TPP, SeatGuard ends: FPP vanilla).
inline std::atomic<bool> g_seatTpp{false};
inline std::atomic<int> g_seatLastMountTpp{-1}; // g_mountTpp on the fire frame (diagnostics; -1 = no fire yet)
// last fire (diagnostics, MMDriver ride line): Hips step cm, torso step deg, seconds after BeginMount
inline std::atomic<float> g_seatLastHipsCm{0.0f}, g_seatLastTorsoDeg{0.0f}, g_seatLastAt{0.0f};
// last fire of the seated neck cut (NeckGuard only): Hips step cm and torso step deg on the fire frame, seconds after BeginMount
inline std::atomic<float> g_neckLastHipsCm{0.0f}, g_neckLastTorsoDeg{0.0f}, g_neckLastAt{-1.0f};
// seated neck cut, per mount (diagnostics, MMDriver seat line): fired, largest neck-chain step above its rate inside the edge window
// (deg; the fire step when it fired), seating edge (latest frozen frame) seconds after BeginMount (-1 = none)
inline std::atomic<int> g_seatNeckFired{0};
inline std::atomic<float> g_seatNeckStepDeg{0.0f}, g_seatNeckEdgeAt{-1.0f};
// pop filter: largest game step seen since BeginMount (rotation deg / Hips cm; diagnostics)
inline std::atomic<float> g_popMaxStepDeg{0.0f}, g_popMaxHipsCm{0.0f};

// Exit latch (EXIT LATCH). g_rideCapture: SetVehicleCapture's ride capture (not the live exit capture). g_exitHold: the animation
// threads write g_exitHoldPose on the records; pinned by BeginExit (hand-in source), released on the first allowed frame after the
// hand-in snapshot was published (g_exitHoldPublished).
inline std::atomic<bool> g_rideCapture{false};
inline std::atomic<bool> g_exitHold{false}, g_exitHoldPinned{false}, g_exitHoldPublished{false};
inline std::atomic<int64_t> g_exitHoldPinTicks{0};
inline integ::SeqlockBuffer<Capture> g_exitHoldPose;
inline std::atomic<uint32_t> g_latchReset{0}; // game thread: drop the animation-side latch state (edge, abort, mount)
// exit latch -> pop filter: the held frames replace the records, the pop filter's history restarts after them (its own offset of the
// swap frame must not decay on top of the release blend)
inline std::atomic<bool> g_popRestart{false};
struct LatchState
{
    uint64_t rig = 0;
    int64_t prevTicks = 0, relTicks = 0;
    uint32_t prevValid = 0, relOn = 0, resetSeen = 0, pad = 0;
    float prevHips[3] = {};
    float relQ[integ::kMaxMMBones * 4] = {};
    float relP[integ::kMaxMMBones * 3] = {};
    uint8_t relOk[integ::kMaxMMBones] = {};
};
inline LatchState g_latch;                       // animation threads, under g_latchBusy
inline std::atomic_flag g_latchBusy = ATOMIC_FLAG_INIT;

// Standing entry diagnostics (MMDriver mount line) and the gate for UpdateWorld (epoch of a standing first post-hold frame).
inline std::atomic<uint64_t> g_mountStandingEpoch{0};
inline std::atomic<float> g_mountGateHipsZ{0.0f}, g_mountLegCapDeg{0.0f}, g_mountLegMoveAt{-1.0f};
inline std::atomic<uint32_t> g_mountLegOffsetBones{0}, g_mountLegsDropped{0};
inline std::atomic<float> g_rootEngageAt{-1.0f}, g_rootPeakYawDeg{0.0f}, g_rootPeakDistM{0.0f};
inline std::atomic<float> g_rootAlignAt{-1.0f}; // seconds since BeginMount of the yaw align latch (-1 = none)
// GLIDE LIMIT diagnostics (the 'vehtrans mount:' line can print them): the planar release duration last set (0 = the fixed 0.30 / 0.60 s) and the
// number of slide-follow samples (jump path re-bases after the first jump sample)
inline std::atomic<float> g_glidePlanarS{0.0f};
inline std::atomic<uint32_t> g_glideFollows{0};
inline std::atomic<bool> g_mountAlignCar{false}; // this mount may latch the yaw align (cars only, see YAW ALIGN)
// hold cap diagnostics (HOLD CAP; MMDriver mount line). g_capState: 0 no cap received, 1 received but not needed (gate / gain not met),
// 2 applied on the cumulative path, 3 applied on the jump path, 5 refused (bike), 6 dropped (older than kHoldCapMaxAgeSeconds), 7 ignored (mutation 22),
// 8 late (arrived more than kHoldCapMaxAgeSeconds into this mount: not applied, not kept for the next one)
inline std::atomic<int> g_capState{0};
inline std::atomic<float> g_capSeconds{-1.0f};   // the accepted cap (seconds after BeginMount; -1 = none)
inline std::atomic<float> g_capArriveAt{0.0f};   // seconds after BeginMount when it arrived (< 0: before BeginMount)
inline std::atomic<float> g_capStartAt{-1.0f}, g_capTailUsed{0.0f}; // release start (seconds after BeginMount) and length of the applied cap
inline std::atomic<int> g_capSecondIgnored{0}; // caps that arrived while a cap was already applied to this mount (ignored: a new curve would pop)
// jump path (rootMode 1) statistics: first jump sample (seconds after BeginMount; -1 = none), largest planar offset after the clamp,
// end of the last release (jump sample + decay length, capped or natural)
inline std::atomic<float> g_jumpEngageAt{-1.0f}, g_jumpPeakDistM{0.0f}, g_jumpReleaseEnd{-1.0f};
// a cap that arrived before BeginMount (game thread): consumed by the next BeginMount if fresh
inline bool g_capPending = false;
inline float g_capPendingSeconds = -1.0f;
inline int64_t g_capPendingTicks = 0;

// Game-thread private state (never read by animation threads).
struct GameSide
{
    Control c;
    uint64_t nextEpoch = 0;
    float lastPos[3] = {};
    float lastYaw = 0.0f; // rad
    int64_t lastTicks = 0; // time of lastPos (UpdateWorld / BeginMount)
    bool glideLatched = false; // the glide limit was active in this mount: its planar clamp stays (a cap arriving later must not step a 2 m hold down to 1.2 m)
    bool alignCar = false; // the vehicle is a car: the yaw align may latch (BeginMount)
    // hold cap (HOLD CAP)
    bool capHave = false, capGate = false, capLatched = false, capJumpDone = false;
    bool capFrozen = false;      // cumulative cap: the planar offset is frozen in the world from the release start (capFx / capFy)
    float capFx = 0.0f, capFy = 0.0f;
    bool capJumpApplied = false; // the jump path applied the cap once in this mount: its gates are not asked again (ApplyJumpCap)
    float capSec = -1.0f;
    float jumpMag = 0.0f; // planar offset (after the clamp) of the last jump sample
    // cumulative root tracking (STANDING ENTRY): entity before BeginMount, visual anchor, offsets at the end of the tracking window
    float beginPos[3] = {};
    float beginYaw = 0.0f;
    float visX = 0.0f, visY = 0.0f, visYaw = 0.0f;
    float peakX = 0.0f, peakY = 0.0f, peakYaw = 0.0f; // world metres / rad
    float trackG = 1.0f;                              // remaining fraction (monotonic)
    bool handIn = false, handInStarted = false;
    bool livePending = false; // jump-out exit without a fresh ride capture: the first live game pose after the edge becomes the hand-in source
    int64_t liveTicks = 0;
    int64_t handInTicks = 0;
    integ::PoseSnapshot captured;
    float offQ[integ::kMaxMMBones * 4] = {};
    float offP[integ::kMaxMMBones * 3] = {};
    uint8_t offOk[integ::kMaxMMBones] = {};
};
inline GameSide g_game;
// tests only: the glide limit (GLIDE LIMIT) is off while set (the hold cap groups replay the pre-glide hold they were written for)
inline std::atomic<bool> g_glideOff{false};

// Cheap gate for the animation threads: false = nothing to do (on foot).
inline bool Wanted()
{
    return g_activeEpoch.load(std::memory_order_relaxed) != 0 || g_captureOn.load(std::memory_order_relaxed) ||
           g_popOn.load(std::memory_order_relaxed) || g_rearmHold.load(std::memory_order_relaxed) ||
           g_exitHold.load(std::memory_order_relaxed) ||
           ((g_seatEpoch.load(std::memory_order_relaxed) != 0 || g_neckEpoch.load(std::memory_order_relaxed) != 0) &&
            g_seatTpp.load(std::memory_order_relaxed));
}
inline bool SeatGuardArmed() { return g_seatEpoch.load(std::memory_order_acquire) != 0; }
inline bool MountActive() { return g_activeEpoch.load(std::memory_order_acquire) != 0; }
inline bool HandInActive() { return g_game.handIn; } // game thread

namespace detail
{
inline void EndEpoch(uint64_t aEpoch)
{
    uint64_t e = aEpoch;
    g_activeEpoch.compare_exchange_strong(e, 0, std::memory_order_acq_rel);
}
inline float RootFactorTimed(const Control& aC, int64_t aNow, float aTailS, float aDecayS, bool aYaw = false)
{
    if (aC.rootTicks <= 0 || (aC.rootX == 0.0f && aC.rootY == 0.0f && aC.rootYaw == 0.0f))
        return 0.0f;
    // tracking republishes rootTicks every game feed; an animation-thread aNow read just before that publish must not
    // read as 'not engaged yet'
    if (aC.rootMode != 2 && aNow < aC.rootTicks)
    {
        // GLIDE LIMIT: the jump path of a glided car republishes at every slide-follow feed (1-2 m offsets): an animation thread that read its aNow just
        // before the publish must not read 'not engaged yet' (a one-frame snap of the whole hold) - it sees the new offset at its start
        if (aC.planarS <= 0.0f && aC.yawTicks <= 0)
            return 0.0f;
        aNow = aC.rootTicks;
    }
    if (aC.rootMode == 2)
    {
        // cumulative tracking: the published offset follows the entity; the time fallback runs from BeginMount
        const float t = Seconds(aNow - aC.beginTicks, aC.tps);
        return t <= kRootTrackSeconds ? 1.0f : 1.0f - Smoothstep((t - kRootTrackSeconds) / aTailS);
    }
    return 1.0f - Smoothstep(Seconds(aNow - ((aYaw && aC.yawTicks > 0) ? aC.yawTicks : aC.rootTicks), aC.tps) / aDecayS);
}
inline float RootFactor(const Control& aC, int64_t aNow) { return RootFactorTimed(aC, aNow, kRootTailSeconds, kRootSeconds, true); }
// Natural factor of the PLANAR part (GLIDE LIMIT): the same curve over the per-mount duration; exactly RootFactor without one.
inline float PlanarNatural(const Control& aC, int64_t aNow)
{
    return aC.planarS > 0.0f ? RootFactorTimed(aC, aNow, aC.planarS, aC.planarS) : RootFactor(aC, aNow);  // (the yaw clock only moves under the glide limit)
}
// Yaw part of the root offset after the yaw align latch (YAW ALIGN): 1 until the latch, then 1 - smootherstep over kAlignSeconds.
// Evaluated per animation frame; shown yaw = entity + rootYaw * RootFactor * YawFactor.
inline float YawFactor(const Control& aC, int64_t aNow)
{
    if (!aC.yawAlign)
        return 1.0f;
    return 1.0f - Smootherstep(Seconds(aNow - aC.alignTicks, aC.tps) / kAlignSeconds);
}
// Planar factor of the root offset (HOLD CAP): the natural factor, limited by the release curve of a cap (1 until capTicks, then
// 1 - smoothstep over capTailS). Without a cap exactly the natural factor (bit-identical).
inline float PlanarFactor(const Control& aC, int64_t aNow)
{
    const float f = PlanarNatural(aC, aNow);
    if (!aC.capOn || !(aC.capTailS > 0.0f))
        return f;
    const float t = Seconds(aNow - aC.capTicks, aC.tps);
    const float curve = t <= 0.0f ? 1.0f : 1.0f - Smoothstep(t / aC.capTailS);
    return curve < f ? curve : f;
}
// Leg factor of a standing entry (STANDING ENTRY): smoothstep over kLegDecaySeconds from the first post-hold frame; once the game
// legs move, the rest finishes within kLegFinishSeconds.
inline float LegFactor(const AnimState& aA, int64_t aTps, int64_t aNow)
{
    const float t1 = Seconds(aNow - aA.offsetTicks, aTps);
    if (!aA.legMoving)
    {
        const float f = 1.0f - Smoothstep((t1 > 0.0f ? t1 : 0.0f) / kLegDecaySeconds);
        return kMutation == 18 ? (f > 0.3f ? f : 0.3f) : f;
    }
    const float td = Seconds(aNow - aA.legMoveTicks, aTps);
    return aA.legMoveF * (1.0f - Smoothstep((td > 0.0f ? td : 0.0f) / kLegFinishSeconds));
}
inline bool IsLegChain(uint8_t aChain) { return aChain == kChainLeftLeg || aChain == kChainRightLeg || aChain == kChainSingle; }
inline uint8_t ChainOf(const Control& aC, uint32_t aBone, int32_t aHips)
{
    if (static_cast<int32_t>(aBone) == aHips)
        return kChainNone;
    if (aC.roles.valid)
        return aC.roles.chain[aBone];
    return integ::SnapshotUpperBone(aC.frozen, aBone) ? kChainNone : kChainSingle;
}
inline int32_t HipsOf(const Control& aC)
{
    return aC.roles.valid ? aC.roles.hips : static_cast<int32_t>(aC.frozen.firstBone);
}
// Hold target of one bone: the frozen MM transform blended onto the live record with the snapshot's own bone weight
// (BlendIntoRecords formula). Returns false for a bone the hold leaves untouched.
inline bool HoldTarget(const integ::PoseSnapshot& aS, uint32_t aBone, const float* aRec, float aP[3], float aQ[4])
{
    const float wFull = aS.weight >= 1.0f ? 1.0f : aS.weight;
    const float us = std::isfinite(aS.upperScale) ? (aS.upperScale < 0.0f ? 0.0f : (aS.upperScale > 1.0f ? 1.0f : aS.upperScale))
                                                  : 1.0f;
    const float w = integ::SnapshotUpperBone(aS, aBone) ? wFull * us : wFull * integ::SnapshotLowerScale(aS);
    if (!(w > 0.0f))
        return false;
    const float* m = &aS.pose[size_t(aBone) * 7];
    if (!Finite(m, 7) || !Finite(aRec, 3) || !Finite(aRec + integ::kGameRotOffsetFloats, 4))
        return false;
    const float* q = aRec + integ::kGameRotOffsetFloats;
    float mq[4] = {m[3], m[4], m[5], m[6]};
    if (q[0] * mq[0] + q[1] * mq[1] + q[2] * mq[2] + q[3] * mq[3] < 0.0f)
        for (float& v : mq)
            v = -v;
    if (w >= 1.0f)
    {
        aP[0] = m[0]; aP[1] = m[1]; aP[2] = m[2];
        for (int k = 0; k < 4; ++k)
            aQ[k] = mq[k];
        return QNormalize(aQ);
    }
    for (int k = 0; k < 3; ++k)
        aP[k] = aRec[k] + (m[k] - aRec[k]) * w;
    for (int k = 0; k < 4; ++k)
        aQ[k] = q[k] + (mq[k] - q[k]) * w;
    return QNormalize(aQ);
}
// Natural planar release duration of a held offset aD (m): the smoothstep's peak closing speed 1.5 aD / D stays at kGlideSpeedMps, never
// shorter than the old fixed aBase and never later than aBudget (the offset must be gone before the sit-down; the speed then exceeds the limit).
inline float GlideSeconds(float aD, float aBase, float aBudget)
{
    float d = 1.5f * aD / kGlideSpeedMps;
    const float cap = aBudget > aBase ? aBudget : aBase;
    d = d > cap ? cap : d;
    return d < aBase ? aBase : d;
}
// Root offset on the Hips record: p' = Rz(yaw) p + (x, y, 0), q' = qz q (OffsetRoot::Apply convention).
inline void ApplyRoot(float* aRec, float aX, float aY, float aYaw)
{
    const float c = std::cos(aYaw), s = std::sin(aYaw);
    const float px = aRec[0], py = aRec[1];
    aRec[0] = c * px - s * py + aX;
    aRec[1] = s * px + c * py + aY;
    const float qz[4] = {0.0f, 0.0f, std::sin(0.5f * aYaw), std::cos(0.5f * aYaw)};
    float* q = aRec + integ::kGameRotOffsetFloats;
    float o[4];
    QMul(qz, q, o);
    if (QNormalize(o))
        for (int k = 0; k < 4; ++k)
            q[k] = o[k];
}
// Hold cap on the jump path (HOLD CAP): called with the sample time aTs (seconds after BeginMount) at a jump sample (g.c.rootTicks =
// aNow) or at a later sample while the jump decay runs. Sets g.c.capOn / capTicks / capTailS (the caller publishes). Returns true when a
// cap was applied.
inline bool ApplyJumpCap(GameSide& g, float aTs, int64_t aNow)
{
    if (!g.capHave || g.capJumpDone || g.c.rootMode != 1)
        return false;
    g.capJumpDone = true;
    const float rem = g.capSec - aTs; // time left until the cap
    // The gates decide once per mount: a later jump sample rebases the offset onto the already capped visible root (smaller, and the
    // time to the cap is shorter), so asking again would drop a cap that is working and restart the natural decay.
    if (!g.capJumpApplied || kMutation == 24) // mutation 24: the gates are asked again at every jump sample (a working cap is dropped)
    {
        if (g.jumpMag < kHoldCapJumpGateDistM)
        {
            g_capState.store(1, std::memory_order_relaxed);
            return false;
        }
        // review r2 finding 17: a glided release (Control::planarS > 0: the glide limit was active in this mount) runs over planarS from the last re-base
        // sample, not over kRootSeconds - judged against 0.3 s, a late cap (it may arrive up to kHoldCapMaxAgeSeconds after BeginMount) was refused
        // while the glided hold (up to 3 m) kept the body back long after the door edge. Without a glided release the literal gate (bit-identical).
        float natural = kRootSeconds;
        if (g.c.planarS > 0.0f && kMutation != 27) // mutation 27: the glided hold is judged against kRootSeconds (pre-fix)
        {
            const float since = Seconds(aNow - g.c.rootTicks, g.c.tps); // 0 at a jump sample, the age of the re-base at a later sample
            natural = g.c.planarS - (since > 0.0f ? since : 0.0f);
        }
        if (rem > natural - kHoldCapJumpGainSeconds)
        {
            g_capState.store(1, std::memory_order_relaxed);
            return false; // the natural decay is not materially longer than the cap allows
        }
    }
    g.capJumpApplied = true;
    const float tail = rem < kHoldCapMinReleaseSeconds ? kHoldCapMinReleaseSeconds : rem;
    g.c.capOn = 1;
    g.c.capTicks = aNow;
    g.c.capTailS = tail;
    g_capState.store(3, std::memory_order_relaxed);
    g_capStartAt.store(aTs, std::memory_order_relaxed);
    g_capTailUsed.store(tail, std::memory_order_relaxed);
    g_jumpReleaseEnd.store(aTs + tail, std::memory_order_relaxed);
    return true;
}
} // namespace detail

// ------------------------------------------------------------------------------------------------ game-thread API
// Bone roles of the active database (call on database load; optional, see BoneRoles).
inline void SetBoneRoles(const BoneRoles& aRoles) { g_roles.Publish(aRoles); }

// Mount cut. aLastMm = the last published MM snapshot (pose + bone map), aPos / aYawDeg = the entity world transform BEFORE
// the workspot jump (normally the previous sample). A new mount ends a running hand-in. aCar: the vehicle is a car (YAW ALIGN may latch); false (bikes, unknown) = the
// cumulative hold only.
inline void BeginMount(const integ::PoseSnapshot& aLastMm, const float aPos[3], float aYawDeg, int64_t aNow, bool aCar = false)
{
    GameSide& g = g_game;
    g.handIn = false;
    g_activeEpoch.store(0, std::memory_order_release);
    g_seatEpoch.store(0, std::memory_order_release);
    g_neckEpoch.store(0, std::memory_order_release);
    g_seatTpp.store(false, std::memory_order_release);
    g_exitHold.store(false, std::memory_order_release);
    g_exitHoldPinned.store(false, std::memory_order_release);
    g_exitHoldPublished.store(false, std::memory_order_release);
    g_latchReset.fetch_add(1, std::memory_order_acq_rel);
    g_mountStandingEpoch.store(0, std::memory_order_release);
    g_popMaxStepDeg.store(0.0f, std::memory_order_relaxed);
    g_popMaxHipsCm.store(0.0f, std::memory_order_relaxed);
    g_seatNeckFired.store(0, std::memory_order_relaxed);
    g_seatNeckStepDeg.store(0.0f, std::memory_order_relaxed);
    g_neckLastHipsCm.store(0.0f, std::memory_order_relaxed);
    g_neckLastTorsoDeg.store(0.0f, std::memory_order_relaxed);
    g_neckLastAt.store(-1.0f, std::memory_order_relaxed);
    g_seatNeckEdgeAt.store(-1.0f, std::memory_order_relaxed);
    g_mountGateHipsZ.store(0.0f, std::memory_order_relaxed);
    g_mountLegCapDeg.store(0.0f, std::memory_order_relaxed);
    g_mountLegMoveAt.store(-1.0f, std::memory_order_relaxed);
    g_mountLegOffsetBones.store(0, std::memory_order_relaxed);
    g_mountLegsDropped.store(0, std::memory_order_relaxed);
    g_rootEngageAt.store(-1.0f, std::memory_order_relaxed);
    g_rootPeakYawDeg.store(0.0f, std::memory_order_relaxed);
    g_rootPeakDistM.store(0.0f, std::memory_order_relaxed);
    g_rootAlignAt.store(-1.0f, std::memory_order_relaxed);
    g_glidePlanarS.store(0.0f, std::memory_order_relaxed);
    g_glideFollows.store(0, std::memory_order_relaxed);
    g.glideLatched = false;
    g_mountAlignCar.store(aCar, std::memory_order_relaxed);
    g_game.alignCar = aCar;
    // hold cap: fresh state; a cap that arrived shortly before this BeginMount (the door script ran first) is consumed here
    g_capState.store(0, std::memory_order_relaxed);
    g_capSeconds.store(-1.0f, std::memory_order_relaxed);
    g_capArriveAt.store(0.0f, std::memory_order_relaxed);
    g_capStartAt.store(-1.0f, std::memory_order_relaxed);
    g_capTailUsed.store(0.0f, std::memory_order_relaxed);
    g_capSecondIgnored.store(0, std::memory_order_relaxed);
    g_jumpEngageAt.store(-1.0f, std::memory_order_relaxed);
    g_jumpPeakDistM.store(0.0f, std::memory_order_relaxed);
    g_jumpReleaseEnd.store(-1.0f, std::memory_order_relaxed);
    g.capHave = g.capGate = g.capLatched = g.capJumpDone = g.capJumpApplied = g.capFrozen = false;
    g.capSec = -1.0f;
    g.jumpMag = 0.0f;
    if (g_capPending)
    {
        const float age = detail::Seconds(aNow - g_capPendingTicks, TicksPerSecond());
        g_capPending = false;
        if (aNow >= g_capPendingTicks && age <= kHoldCapMaxAgeSeconds)
        {
            g_capSeconds.store(g_capPendingSeconds, std::memory_order_relaxed);
            g_capArriveAt.store(-age, std::memory_order_relaxed);
            if (!aCar)
                g_capState.store(5, std::memory_order_relaxed);
            else
            {
                g.capHave = true;
                g.capSec = g_capPendingSeconds;
                g_capState.store(1, std::memory_order_relaxed);
            }
        }
        else
            g_capState.store(6, std::memory_order_relaxed);
    }
    if (!aLastMm.rig || aNow <= 0)
        return;
    Control c;
    c.epoch = ++g.nextEpoch;
    c.rig = aLastMm.rig;
    c.beginTicks = aNow;
    c.tps = TicksPerSecond();
    c.frozen = aLastMm;
    const uint32_t nb = aLastMm.numBones <= integ::kMaxMMBones ? aLastMm.numBones : integ::kMaxMMBones;
    c.frozen.numBones = nb;
    c.hold = (kMutation != 1 && aLastMm.weight > 0.0f && aLastMm.firstBone < nb) ? 1u : 0u;
    BoneRoles roles;
    if (g_roles.Read(roles) && roles.valid && roles.hips >= 0 && static_cast<uint32_t>(roles.hips) < nb)
        c.roles = roles;
    const bool posOk = aPos && detail::Finite(aPos, 3) && std::isfinite(aYawDeg);
    for (int k = 0; k < 3; ++k)
        g.lastPos[k] = g.beginPos[k] = posOk ? aPos[k] : 0.0f;
    g.lastYaw = g.beginYaw = posOk ? aYawDeg * detail::kDeg : 0.0f;
    g.lastTicks = aNow;
    g.trackG = 1.0f;
    g.c = c;
    g_control.Publish(c);
    g_activeEpoch.store(c.epoch, std::memory_order_release);
    UTPP_MUT_IF (kMutation != 7)
        g_seatEpoch.store(c.epoch, std::memory_order_release); // seat pose switch guard (mutation 7: off)
    UTPP_MUT_IF (kMutation != 19)
        g_neckEpoch.store(c.epoch, std::memory_order_release); // seated neck cut (mutation 19: off)
    g_counters.mounts.fetch_add(1, std::memory_order_relaxed);
}

// Entity world transform per sample (call during the first kWorldWatchSeconds after BeginMount; later calls are ignored).
inline void UpdateWorld(const float aPos[3], float aYawDeg, int64_t aNow)
{
    GameSide& g = g_game;
    if (!aPos || !detail::Finite(aPos, 3) || !std::isfinite(aYawDeg) || !g.c.epoch ||
        g_activeEpoch.load(std::memory_order_acquire) != g.c.epoch || aNow < g.c.beginTicks)
        return;
    const float yaw = aYawDeg * detail::kDeg;
    const float ts = detail::Seconds(aNow - g.c.beginTicks, g.c.tps);
    // GLIDE LIMIT: a car mount without a door cap (see the header comment); a cap arriving later switches it off from the next sample on
    const bool glide = g.alignCar && !g.capHave && kMutation != 26 && !g_glideOff.load(std::memory_order_relaxed);
    if (glide)
        g.glideLatched = true;
    const float maxDist = (glide || g.glideLatched) ? kGlideMaxDistance : kRootMaxDistance;
    const float stepDt = detail::Seconds(aNow - g.lastTicks, g.c.tps);
    const float speedXY = std::hypot(aPos[0] - g.lastPos[0], aPos[1] - g.lastPos[1]) / (stepDt > 1e-3f ? stepDt : 1e-3f);
    // the vanilla slide goes on after the jump sample(s): a car keeps re-basing onto the visible root while the entity still outruns the clip's walk
    // (also the samples below kJumpDistance inside the watch window, which would otherwise carry the held body with the entity)
    const bool follow = glide && g.c.rootMode == 1 && !g.capJumpApplied && ts <= kGlideFollowSeconds && speedXY > kGlideFollowSpeedMps;
    if ((ts <= kWorldWatchSeconds && g.c.rootMode != 2) || follow)
    {
        const float dx = aPos[0] - g.lastPos[0], dy = aPos[1] - g.lastPos[1];
        const float dyaw = detail::WrapPi(yaw - g.lastYaw);
        const bool jumpSample = ts <= kWorldWatchSeconds && g.c.rootMode != 2 &&
                                (std::hypot(dx, dy) > kJumpDistance || std::fabs(dyaw) > kJumpYawDeg * detail::kDeg);
        const bool followOnly = follow && !jumpSample; // the planar part only: the yaw keeps its decay (the entity's pivot goes on)
        if (jumpSample || followOnly)
        {
            // keep the VISUAL root where it is: residual offset (old frame) -> world -> new frame, minus the jump
            const float f = detail::RootFactor(g.c, aNow);
            const float fp = detail::PlanarFactor(g.c, aNow); // the hold cap shortens the planar part only
            float wx = 0.0f, wy = 0.0f;
            detail::CharacterToWorld(g.c.rootX * fp, g.c.rootY * fp, g.lastYaw, wx, wy);
            const float visX = g.lastPos[0] + wx, visY = g.lastPos[1] + wy;
            const float visYaw = g.lastYaw + g.c.rootYaw * f;
            float ox = 0.0f, oy = 0.0f;
            integ::WorldToCharacter(visX - aPos[0], visY - aPos[1], yaw, ox, oy);
            float oyaw = detail::WrapPi(visYaw - yaw);
            UTPP_MUT_IF (kMutation == 2)
            {
                ox = -ox; oy = -oy; oyaw = -oyaw;
            }
            const float d = std::hypot(ox, oy);
            if (d > maxDist)
            {
                ox *= maxDist / d;
                oy *= maxDist / d;
            }
            const float maxYaw = kRootMaxYawDeg * detail::kDeg;
            oyaw = oyaw > maxYaw ? maxYaw : (oyaw < -maxYaw ? -maxYaw : oyaw);
            if (followOnly)
            {
                if (g.c.yawTicks <= 0)
                    g.c.yawTicks = g.c.rootTicks; // the yaw decay clock stays where the last full jump sample put it
            }
            else
            {
                g.c.rootYaw = oyaw;
                g.c.yawTicks = 0;
            }
            g.c.rootX = ox; g.c.rootY = oy;
            g.c.rootTicks = aNow;
            g.c.rootMode = 1;
            // planar release over the speed-limited duration (GLIDE LIMIT), from this (re-)base sample; the yaw part keeps kRootSeconds
            // review r2 finding 17: a re-base after the glide switched off (a cap arrived) in a mount that glided keeps a speed-limited release, bounded by
            // the cap: zeroing it released the 2-3 m hold of the still latched clamp over 0.30 s (10-15 m/s)
            if (glide)
                g.c.planarS = detail::GlideSeconds(std::hypot(ox, oy), kRootSeconds, kGlideEndSeconds - ts);
            else if (g.glideLatched && g.capHave && kMutation != 28) // mutation 28: the release drops back to kRootSeconds (pre-fix)
                g.c.planarS = detail::GlideSeconds(std::hypot(ox, oy), kRootSeconds, (g.capSec < kGlideEndSeconds ? g.capSec : kGlideEndSeconds) - ts);
            else
                g.c.planarS = 0.0f;
            g_glidePlanarS.store(g.c.planarS, std::memory_order_relaxed);
            if (followOnly)
                g_glideFollows.fetch_add(1, std::memory_order_relaxed);
            // hold cap: a new jump sample restarts the natural decay, so the cap is evaluated again for it
            g.c.capOn = 0;
            if (!followOnly)
                g.capJumpDone = false; // a follow sample (no cap in this mount) never asks the cap gates again
            g.jumpMag = std::hypot(ox, oy);
            if (g_jumpEngageAt.load(std::memory_order_relaxed) < 0.0f)
                g_jumpEngageAt.store(ts, std::memory_order_relaxed);
            g_jumpPeakDistM.store(std::fmax(g_jumpPeakDistM.load(std::memory_order_relaxed), g.jumpMag), std::memory_order_relaxed);
            g_jumpReleaseEnd.store(ts + (g.c.planarS > 0.0f ? g.c.planarS : kRootSeconds), std::memory_order_relaxed);
            if (!followOnly)
                detail::ApplyJumpCap(g, ts, aNow);
            g_control.Publish(g.c);
        }
        else if (g.c.rootMode == 1 && g.capHave && !g.capJumpDone && detail::ApplyJumpCap(g, ts, aNow))
            g_control.Publish(g.c); // a cap that arrived after the jump sample: applied on this next sample
    }
    else if (g.c.rootMode == 1 && g.capHave && !g.capJumpDone && detail::ApplyJumpCap(g, ts, aNow))
        g_control.Publish(g.c);
    // cumulative tracking of a standing entry (STANDING ENTRY); never after a per-sample jump (car path unchanged)
    // review r2 finding 20: a glided planar tail (planarS up to kGlideEndSeconds - kRootTrackSeconds) outlasts kRootTailSeconds: the world-fixed offset keeps being
    // re-published until the planar factor is 0 (the character-frame offset would otherwise turn with the entity's yaw for the rest of the release)
    const float trackTail = (g.c.planarS > kRootTailSeconds && kMutation != 30) ? g.c.planarS : kRootTailSeconds; // mutation 30: kRootTailSeconds (pre-fix)
    const bool trackOn = kMutation != 15 && g.c.rootMode != 1 && g_mountStandingEpoch.load(std::memory_order_acquire) == g.c.epoch &&
                         ts <= kRootTrackSeconds + trackTail;
    if (trackOn)
    {
        bool track = g.c.rootMode == 2;
        if (!track && ts <= kRootTrackSeconds)
        {
            const float cdx = aPos[0] - g.beginPos[0], cdy = aPos[1] - g.beginPos[1];
            const float cdyaw = detail::WrapPi(yaw - g.beginYaw);
            if (std::hypot(cdx, cdy) > kJumpDistance || std::fabs(cdyaw) > kJumpYawDeg * detail::kDeg)
            {
                // engage: the visual root stays where it was on the previous sample (no step on this one)
                g.visX = g.lastPos[0];
                g.visY = g.lastPos[1];
                g.visYaw = g.lastYaw;
                g.trackG = 1.0f;
                g.c.rootMode = 2;
                track = true;
                g_rootEngageAt.store(ts, std::memory_order_relaxed);
            }
        }
        if (track)
        {
            float wx = g.visX - aPos[0], wy = g.visY - aPos[1];
            float oyaw = detail::WrapPi(g.visYaw - yaw);
            // yaw align latch (YAW ALIGN): the held yaw is large - from here the yaw part blends to the entity (factor 1 at the latch)
            UTPP_MUT_IF (kMutation != 20) // mutation 20: the old hold
            {
                if (g.alignCar && !g.c.yawAlign && ts <= kRootTrackSeconds && std::fabs(oyaw) > kAlignYawDeg * detail::kDeg)
                {
                    g.c.yawAlign = 1;
                    g.c.alignTicks = aNow;
                    g_rootAlignAt.store(ts, std::memory_order_relaxed);
                }
            }
            if (ts <= kRootTrackSeconds)
            {
                const float liveDist = std::hypot(wx, wy); // the diagnostics and the gate read the live hold, not the frozen vector
                g_rootPeakYawDeg.store(std::fmax(g_rootPeakYawDeg.load(std::memory_order_relaxed), std::fabs(oyaw) / detail::kDeg),
                                       std::memory_order_relaxed);
                g_rootPeakDistM.store(std::fmax(g_rootPeakDistM.load(std::memory_order_relaxed), liveDist),
                                      std::memory_order_relaxed);
                // hold cap (HOLD CAP): the gate is tracked whether or not a cap exists; the cap latches on the first sample with
                // both, so a cap that arrives late acts from the next sample on (the release curve is 1 until its start)
                if (ts <= kHoldCapGateWithinSeconds && liveDist >= kHoldCapGateDistM)
                    g.capGate = true;
                if (g.capHave && g.capGate && !g.capLatched)
                {
                    g.capLatched = true;
                    // review r2 finding 17: the natural release of a glided hold runs over planarS (up to kGlideEndSeconds - kRootTrackSeconds), not kRootTailSeconds
                    const float effTail = (g.c.planarS > 0.0f && kMutation != 29) ? g.c.planarS : kRootTailSeconds; // mutation 29: kRootTailSeconds (pre-fix)
                    if (g.capSec >= kRootTrackSeconds + effTail)
                        g_capState.store(1, std::memory_order_relaxed); // the natural release ends before the cap: nothing to do
                    else
                    {
                        const float hold = g.capSec - kHoldCapTailSeconds;
                        const float start = std::fmax(ts, std::fmin(kRootTrackSeconds, hold));
                        float tail = g.capSec - start;
                        tail = tail < kHoldCapMinReleaseSeconds ? kHoldCapMinReleaseSeconds : (tail > effTail ? effTail : tail);
                        g.c.capOn = 1;
                        g.c.capTicks = start <= ts ? aNow : g.c.beginTicks + static_cast<int64_t>(static_cast<double>(start) * static_cast<double>(g.c.tps));
                        g.c.capTailS = tail;
                        g_capState.store(2, std::memory_order_relaxed);
                        g_capStartAt.store(start, std::memory_order_relaxed);
                        g_capTailUsed.store(tail, std::memory_order_relaxed);
                    }
                }
                // the release has started (this sample or an earlier one): from its first sample the planar offset is frozen in the world
                if (g.c.capOn && kMutation != 25)
                {
                    const float startTs = detail::Seconds(g.c.capTicks - g.c.beginTicks, g.c.tps);
                    if (!g.capFrozen && ts >= startTs - 1e-6f)
                    {
                        g.capFrozen = true;
                        g.capFx = wx;
                        g.capFy = wy;
                    }
                    if (g.capFrozen)
                    {
                        wx = g.capFx;
                        wy = g.capFy;
                    }
                }
                g.peakX = wx; g.peakY = wy; g.peakYaw = oyaw;
            }
            else
            {
                // after the window: the yaw offset decays with the clip's root-yaw progress (the entity turning back to the
                // anchor), monotonic; the planar offset stays fixed in the world and decays by time only (RootFactor). The yaw
                // progress of the c9 entry clips (10 deg in total) says nothing about the planar distance left: as a planar factor
                // it multiplied with the time decay and slid the body 0.5 m in 0.3 s under planted feet (recording 23:05:22.2-22.5)
                float gp = 1.0f;
                if (std::fabs(g.peakYaw) > 1.0f * detail::kDeg)
                {
                    gp = oyaw / g.peakYaw;
                    gp = gp < 0.0f ? 0.0f : (gp > 1.0f ? 1.0f : gp);
                }
                g.trackG = gp < g.trackG ? gp : g.trackG;
                const float planarG = kMutation == 17 ? g.trackG : 1.0f; // mutation 17: the planar offset follows the yaw progress
                wx = g.peakX * planarG;
                wy = g.peakY * planarG;
                oyaw = g.peakYaw * g.trackG;
            }
            float ox = 0.0f, oy = 0.0f;
            integ::WorldToCharacter(wx, wy, yaw, ox, oy);
            const float d = std::hypot(ox, oy);
            if (d > maxDist)
            {
                ox *= maxDist / d;
                oy *= maxDist / d;
            }
            // the planar tail of the natural release is fixed by the offset held at the end of the tracking window (GLIDE LIMIT)
            if (ts <= kRootTrackSeconds && glide)
            {
                g.c.planarS = detail::GlideSeconds(std::hypot(ox, oy), kRootTailSeconds, kGlideEndSeconds - kRootTrackSeconds);
                g_glidePlanarS.store(g.c.planarS, std::memory_order_relaxed);
            }
            const float maxYaw = kRootMaxYawDeg * detail::kDeg;
            oyaw = oyaw > maxYaw ? maxYaw : (oyaw < -maxYaw ? -maxYaw : oyaw);
            g.c.rootX = ox; g.c.rootY = oy; g.c.rootYaw = oyaw;
            g.c.rootTicks = aNow;
            g_control.Publish(g.c);
        }
    }
    for (int k = 0; k < 3; ++k)
        g.lastPos[k] = aPos[k];
    g.lastYaw = yaw;
    g.lastTicks = aNow;
}

// Per-mount hold cap (HOLD CAP): aSeconds = seconds after BeginMount by which the visible root must be back on the entity path (< 0 = no
// cap). Game thread. Before BeginMount (the door script ran first) it waits in a pending slot; after it, up to kHoldCapMaxAgeSeconds.
inline void SetHoldCap(float aSeconds, int64_t aNow)
{
    UTPP_MUT_IF (kMutation == 22)
    {
        (void)aSeconds;
        (void)aNow;
        g_capState.store(7, std::memory_order_relaxed); // mutation 22: the cap is ignored
        return;
    }
    GameSide& g = g_game;
    if (!std::isfinite(aSeconds) || aSeconds < 0.0f)
    {
        g_capPending = false;
        g.capHave = false;
        return;
    }
    const bool active = g.c.epoch && g_activeEpoch.load(std::memory_order_acquire) == g.c.epoch;
    const float age = active ? detail::Seconds(aNow - g.c.beginTicks, g.c.tps) : 0.0f;
    if (active && aNow >= g.c.beginTicks && age <= kHoldCapMaxAgeSeconds)
    {
        g_capSeconds.store(aSeconds, std::memory_order_relaxed);
        g_capArriveAt.store(age, std::memory_order_relaxed);
        if (!g.alignCar)
        {
            g_capState.store(5, std::memory_order_relaxed); // bike: the validated hold stays
            return;
        }
        if (g.c.capOn)
        {
            // a cap is already releasing this mount: a second curve (1 until its start) would take the planar factor back up
            g_capSecondIgnored.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        g.capHave = true;
        g.capSec = aSeconds;
        g.capLatched = false;
        g.capJumpDone = false;
        g_capState.store(1, std::memory_order_relaxed);
        return;
    }
    if (active && aNow >= g.c.beginTicks)
    {
        // later than kHoldCapMaxAgeSeconds into a running mount: recorded on this ride as late, never kept for the next mount
        if (g.c.capOn)
            g_capSecondIgnored.fetch_add(1, std::memory_order_relaxed);
        else
        {
            g_capSeconds.store(aSeconds, std::memory_order_relaxed);
            g_capArriveAt.store(age, std::memory_order_relaxed);
            g_capState.store(8, std::memory_order_relaxed);
        }
        return;
    }
    g_capPending = true;
    g_capPendingSeconds = aSeconds;
    g_capPendingTicks = aNow;
}

inline void SetMountTpp(bool aOn)
{
    g_mountTpp.store(aOn, std::memory_order_release);
    if (aOn && (g_seatEpoch.load(std::memory_order_acquire) != 0 || g_neckEpoch.load(std::memory_order_acquire) != 0))
        g_seatTpp.store(true, std::memory_order_release);
}
// While the vehicle bit is set and TPP is on: capture the live game pose each animation frame (no write).
inline void SetVehicleCapture(bool aOn, uint64_t aRig)
{
    g_captureRig.store(aRig, std::memory_order_relaxed);
    g_rideCapture.store(aOn && aRig != 0, std::memory_order_release);
    g_captureOn.store(aOn && aRig != 0, std::memory_order_release);
}
// Exit latch state (EXIT LATCH): everything off (edge without a held capture, abort, new mount).
inline void ClearExitHold()
{
    g_exitHold.store(false, std::memory_order_release);
    g_exitHoldPinned.store(false, std::memory_order_release);
    g_exitHoldPublished.store(false, std::memory_order_release);
    g_latchReset.fetch_add(1, std::memory_order_acq_rel);
}
inline bool ExitHoldActive() { return g_exitHold.load(std::memory_order_acquire); }
// MMDriver PublishLocked: the snapshot carrying the exit hand-in (weight > 0) was published - the pinned hold ends on the next
// allowed animation frame (the first MM write).
inline void MarkExitHandInPublished()
{
    if (g_exitHoldPinned.load(std::memory_order_acquire))
        g_exitHoldPublished.store(true, std::memory_order_release);
}
// Vehicle-bit edge without a hand-in (BeginExit false): a third-person ride (mounted-TPP on the last vehicle feed) under the vehicle
// hold (aVehicleHold: perspective-denied animation frames in this ride - the permission comes back only after the edge, so a fade
// would start through the game's post-exit pose) snaps MM in at weight 1 (GateFader::Fill). A ride with the permission allowed keeps
// the normal reopen; a jump-out has its live capture instead.
inline bool ExitWantsFill(bool aHanded, bool aTppRide, bool aJumpOutEdge, bool aVehicleHold)
{
    return kMutation != 12 && !aHanded && aTppRide && !aJumpOutEdge && aVehicleHold;
}

// Vehicle-bit falling edge. Ends the mount transition and the capture. true (and aOutCaptured = the capture, weight 1,
// publishTicks = capture ticks) if the last capture is younger than kCaptureMaxAgeSeconds; HandInBlend is then armed.
// aPinJumpOut (jump-out edge, r6 launch): a seated ride never arms the exit latch, but the jump-out edge has the same gap the latch closes for a
// standing one: the pop filter ends with the ride (g_popOn off) and the pose-write permission of the vehicle hold returns a frame after the edge
// (recorded 2026-11-04 06:07:27.190 edge, 27.201 permission back), so an animation frame in between shows the raw game flight pose - a one-frame
// step of up to the whole pop-filter offset - before MM continues from the capture. The edge capture (the pose the player saw: ride captures run
// after the mounted guards on denied frames) is then pinned as a hold exactly like a latched one; it ends on the first allowed frame after the
// hand-in snapshot was published. false (every normal exit): nothing changes.
inline bool BeginExit(int64_t aNow, integ::PoseSnapshot* aOutCaptured, bool aPinJumpOut = false)
{
    GameSide& g = g_game;
    g_activeEpoch.store(0, std::memory_order_release);
    g_captureOn.store(false, std::memory_order_release);
    g_rideCapture.store(false, std::memory_order_release);
    g_mountTpp.store(false, std::memory_order_release);
    g_popOn.store(false, std::memory_order_release);
    g_seatEpoch.store(0, std::memory_order_release);
    g_neckEpoch.store(0, std::memory_order_release);
    g_seatTpp.store(false, std::memory_order_release);
    g.handIn = false;
    bool held = g_exitHold.load(std::memory_order_acquire);
    g_latchReset.fetch_add(1, std::memory_order_acq_rel); // no release blend / latch history after the edge
    Capture cap;
    // exit latch: the held capture (the last one before the game's squat swap) is the hand-in source; the hold stays pinned
    const bool ok = held ? g_exitHoldPose.Read(cap) : g_captures.Read(cap);
    if (!ok || cap.ticks <= 0 || !cap.snap.rig)
    {
        ClearExitHold();
        return false;
    }
    const int64_t tps = TicksPerSecond();
    const float age = std::fabs(detail::Seconds(aNow - cap.ticks, tps));
    if (!(age < kCaptureMaxAgeSeconds))
    {
        ClearExitHold();
        g_counters.exitsStale.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (!held && aPinJumpOut)
    {
        g_exitHoldPose.Publish(cap); // the pose first, then the flags the animation threads read (ExitStep reads g_exitHoldPose under g_exitHold)
        held = true;
    }
    if (held)
    {
        g_exitHoldPinTicks.store(aNow, std::memory_order_relaxed);
        g_exitHoldPublished.store(false, std::memory_order_relaxed);
        g_exitHoldPinned.store(true, std::memory_order_release);
        g_exitHold.store(true, std::memory_order_release); // (an animation thread may have aged it out on this very frame)
        g_counters.exitHoldPins.fetch_add(1, std::memory_order_relaxed);
    }
    g.captured = cap.snap;
    g.handIn = true;
    g.handInStarted = false;
    g.handInTicks = 0;
    if (aOutCaptured)
        *aOutCaptured = cap.snap;
    g_counters.exitsHanded.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// Live exit (2026-10-28 bike jump-out): BeginExit found no fresh ride capture (recording 05-03-09: every exit ran without a hand-in).
// The animation thread captures the live game pose of its next frame(s) (CaptureIfWanted, same capture mailbox); PromoteLiveExit then
// arms the ordinary hand-in from it (the game pose of the launch frame is the inertialization source, MM at weight 1 from that feed on).
// The game pose stays visible (MM weight not yet raised) until then; kLiveExitWaitSeconds without a capture gives up (normal fade-in).
inline constexpr float kLiveExitWaitSeconds = 0.10f;
inline void BeginLiveExit(int64_t aNow)
{
    GameSide& g = g_game;
    g.handIn = false;
    g_seatEpoch.store(0, std::memory_order_release);
    g_neckEpoch.store(0, std::memory_order_release);
    g_seatTpp.store(false, std::memory_order_release);
    g.livePending = true;
    g.liveTicks = aNow;
    g_captureOn.store(true, std::memory_order_release);
}
inline bool LiveExitPending() { return g_game.livePending; }
// Same-rig re-arm in the jump-out flight (recording 2026-10-28 11-49-37: the game replaced the animated object; until the re-arm the
// visible body showed the game pose, not MM): a live capture of the new object becomes the hand-in source as at the launch, and the
// animation threads keep the game pose (no MM blend) until the hand-in snapshot is published - MM stays at its weight (no fade).
inline void BeginRearmCapture(int64_t aNow)
{
    BeginLiveExit(aNow);
    g_rearmHold.store(true, std::memory_order_release);
}
inline bool RearmHold() { return g_rearmHold.load(std::memory_order_acquire); }
inline void EndRearmHold() { g_rearmHold.store(false, std::memory_order_release); }
// MMDriver feed: the mounted pop filter (vehicle and TPP).
inline void SetPopFilter(bool aOn) { g_popOn.store(aOn, std::memory_order_release); }
// 1 = hand-in armed from a post-edge capture (caller: weight 1 now); 0 = still waiting; -1 = gave up / nothing pending.
inline int PromoteLiveExit(int64_t aNow)
{
    GameSide& g = g_game;
    if (!g.livePending)
        return -1;
    Capture cap;
    if (g_captures.Read(cap) && cap.ticks >= g.liveTicks && cap.snap.rig)
    {
        g.livePending = false;
        g_captureOn.store(false, std::memory_order_release);
        g.captured = cap.snap;
        g.handIn = true;
        g.handInStarted = false;
        g.handInTicks = 0;
        g_counters.exitsHanded.fetch_add(1, std::memory_order_relaxed);
        return 1;
    }
    if (!(detail::Seconds(aNow - g.liveTicks, TicksPerSecond()) < kLiveExitWaitSeconds))
    {
        g.livePending = false;
        g_captureOn.store(false, std::memory_order_release);
        g_counters.exitsStale.fetch_add(1, std::memory_order_relaxed);
        return -1;
    }
    return 0;
}

// Exit hand-in on the MM snapshot about to be published (in place). The first call takes the offsets (captured minus MM,
// matched by game bone index); the offsets decay (half-life kHandInHalfLife) to exactly 0 at kHandInSeconds. Returns true
// while the snapshot was modified; false (snapshot untouched) when no hand-in runs, it is done or the rig changed.
inline bool HandInBlend(integ::PoseSnapshot& ioSnap, int64_t aNow)
{
    GameSide& g = g_game;
    if (!g.handIn)
        return false;
    const uint32_t nb = ioSnap.numBones <= integ::kMaxMMBones ? ioSnap.numBones : integ::kMaxMMBones;
    if (ioSnap.rig != g.captured.rig || nb == 0)
    {
        g.handIn = false;
        return false;
    }
    const int64_t tps = TicksPerSecond();
    if (!g.handInStarted)
    {
        const uint32_t cnb = g.captured.numBones <= integ::kMaxMMBones ? g.captured.numBones : integ::kMaxMMBones;
        for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
            g.offOk[b] = 0;
        for (uint32_t b = ioSnap.firstBone; b < nb; ++b)
        {
            const int32_t gi = ioSnap.gameIndex[b];
            if (gi < 0)
                continue;
            for (uint32_t j = 0; j < cnb; ++j)
            {
                if (g.captured.gameIndex[j] != gi)
                    continue;
                const float* c = &g.captured.pose[size_t(j) * 7];
                const float* m = &ioSnap.pose[size_t(b) * 7];
                if (!detail::Finite(c, 7) || !detail::Finite(m, 7))
                    break;
                float mq[4] = {m[3], m[4], m[5], m[6]};
                float cq[4] = {c[3], c[4], c[5], c[6]};
                if (!detail::QNormalize(mq) || !detail::QNormalize(cq))
                    break;
                const float mInv[4] = {-mq[0], -mq[1], -mq[2], mq[3]};
                float off[4];
                detail::QMul(cq, mInv, off);
                if (!detail::QNormalize(off))
                    break;
                for (int k = 0; k < 4; ++k)
                    g.offQ[b * 4 + k] = off[k];
                for (int k = 0; k < 3; ++k)
                    g.offP[b * 3 + k] = c[k] - m[k];
                g.offOk[b] = 1;
                break;
            }
        }
        g.handInStarted = true;
        g.handInTicks = aNow;
    }
    const float t = detail::Seconds(aNow - g.handInTicks, tps);
    if (!(t < kHandInSeconds))
    {
        g.handIn = false;
        return false;
    }
    const float tt = t > 0.0f ? t : 0.0f;
    const float f = std::exp2(-tt / kHandInHalfLife) * (1.0f - detail::Smoothstep(tt / kHandInSeconds));
    for (uint32_t b = ioSnap.firstBone; b < nb; ++b)
    {
        if (!g.offOk[b])
            continue;
        float* m = &ioSnap.pose[size_t(b) * 7];
        float mq[4] = {m[3], m[4], m[5], m[6]};
        if (!detail::QNormalize(mq))
            continue;
        float d[4], o[4];
        detail::QPow(&g.offQ[b * 4], f, d);
        detail::QMul(d, mq, o);
        if (!detail::QNormalize(o))
            continue;
        for (int k = 0; k < 3; ++k)
            m[k] += g.offP[b * 3 + k] * f;
        for (int k = 0; k < 4; ++k)
            m[3 + k] = o[k];
    }
    return true;
}

// Death, scene, rig change, unmount during the blend: everything off at once.
inline void Abort()
{
    g_activeEpoch.store(0, std::memory_order_release);
    g_captureOn.store(false, std::memory_order_release);
    g_rideCapture.store(false, std::memory_order_release);
    g_mountTpp.store(false, std::memory_order_release);
    g_popOn.store(false, std::memory_order_release);
    g_rearmHold.store(false, std::memory_order_release);
    ClearExitHold();
    g_seatEpoch.store(0, std::memory_order_release);
    g_neckEpoch.store(0, std::memory_order_release);
    g_seatTpp.store(false, std::memory_order_release);
    g_game.handIn = false;
    g_game.livePending = false;
}

// Root offset of the running mount transition at aNow (x, y metres in the character frame, yaw degrees). false = none.
inline bool RootOffset(int64_t aNow, float aOut[3])
{
    const uint64_t epoch = g_activeEpoch.load(std::memory_order_acquire);
    Control c;
    if (!epoch || !g_control.Read(c) || c.epoch != epoch)
        return false;
    const float f = detail::RootFactor(c, aNow);
    const float fp = detail::PlanarFactor(c, aNow);
    aOut[0] = c.rootX * fp;
    aOut[1] = c.rootY * fp;
    aOut[2] = c.rootYaw * f * detail::YawFactor(c, aNow) / detail::kDeg;
    return f > 0.0f || detail::PlanarNatural(c, aNow) > 0.0f;
}

// ------------------------------------------------------------------------------------------------ animation-thread API
// Copies the live local pose of the mapped bones while the capture flag is set and the rig matches. Never writes records.
inline bool CaptureIfWanted(const uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow)
{
    if (!aBones || !aCount || !aRig || aNow <= 0 || !g_captureOn.load(std::memory_order_acquire) ||
        g_captureRig.load(std::memory_order_relaxed) != aRig || g_exitHold.load(std::memory_order_acquire))
        return false; // (exit latch: no capture while a capture is held)
    Control c;
    if (!g_control.Read(c) || c.frozen.rig != aRig || !c.frozen.numBones)
        return false;
    Capture cap;
    cap.ticks = aNow;
    cap.snap = c.frozen;
    cap.snap.weight = 1.0f;
    cap.snap.upperScale = 1.0f;
    cap.snap.lowerScale = 1.0f;   // melee full body: a capture is always written in full
    cap.snap.meleeAimWeight = 0.0f;
    cap.snap.publishTicks = aNow;
    if (!integ::CaptureMappedGamePose(aBones, aCount, c.frozen, cap.snap.pose, integ::kMaxMMBones, c.frozen.firstBone))
        return false;
    if (!g_captures.TryPublish(cap))
        return false;
    g_counters.captures.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// Mount transition write on the live records (FK input). Returns the number of bones written; 0 = records untouched.
inline uint32_t ApplyIfActive(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow)
{
    const uint64_t epoch = g_activeEpoch.load(std::memory_order_acquire);
    if (!epoch || !aBones || !aCount)
        return 0;
    Control c;
    if (!g_control.Read(c) || c.epoch != epoch)
        return 0;
    if (aRig != c.rig)
    {
        detail::EndEpoch(epoch);
        g_counters.rigAborts.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    if (aNow < c.beginTicks)
        return 0;
    const float t = detail::Seconds(aNow - c.beginTicks, c.tps);
    const float frYaw = detail::RootFactor(c, aNow);
    const float fpn = detail::PlanarNatural(c, aNow); // the planar natural factor outlives the yaw factor under the glide limit
    const float fr = frYaw > fpn ? frYaw : fpn;       // the epoch / write decisions follow the longer of the two
    const float fp = detail::PlanarFactor(c, aNow);   // hold cap: planar part only
    AnimState a;
    const bool have = g_animState.Read(a) && a.epoch == epoch;
    if (!have)
    {
        a = AnimState{};
        a.epoch = epoch;
    }
    // standing entry: the leg offsets may outlast kMountSeconds (LegFactor)
    const bool legsLeft = a.offsetsReady && a.standing && detail::LegFactor(a, c.tps, aNow) > 0.0f;
    const bool rotPhase = c.hold != 0 && (t < kMountSeconds || legsLeft);
    if (!rotPhase && !(fr > 0.0f))
    {
        // a standing entry keeps the epoch through the cumulative root tracking window (UpdateWorld may still engage)
        const float watch = (a.standing && kMutation != 15) ? kRootTrackSeconds : kWorldWatchSeconds;
        if (t > watch)
        {
            detail::EndEpoch(epoch);
            g_counters.expired.fetch_add(1, std::memory_order_relaxed);
        }
        return 0;
    }
    const integ::PoseSnapshot& s = c.frozen;
    const uint32_t nb = s.numBones <= integ::kMaxMMBones ? s.numBones : integ::kMaxMMBones;
    const int32_t hips = detail::HipsOf(c);
    const float fpr = kMutation == 23 ? fpn : fp; // mutation 23: the rendered root ignores the cap (RootOffset still reports it)
    const float rx = c.rootX * fpr, ry = c.rootY * fpr, ryaw = c.rootYaw * frYaw * detail::YawFactor(c, aNow);
    auto record = [&](uint32_t b) -> float* {
        const int32_t gi = s.gameIndex[b];
        return (gi < 0 || static_cast<uint32_t>(gi) >= aCount) ? nullptr
                                                               : reinterpret_cast<float*>(aBones + size_t(gi) * integ::kGameBoneStride);
    };

    uint32_t written = 0;
    if (rotPhase)
    {
        const bool owner = !g_animWriter.test_and_set(std::memory_order_acquire);
        bool dirty = !have;
        if (a.frames == 0 || detail::Seconds(aNow - a.lastFrameTicks, c.tps) >= kNewFrameGapSeconds)
        {
            ++a.frames;
            a.lastFrameTicks = aNow;
            dirty = true;
        }
        const bool holdFrame = !a.offsetsReady && a.frames <= kMountHoldFrames && t < kMountHoldMaxSeconds;
        if (holdFrame)
        {
            for (uint32_t b = s.firstBone; b < nb; ++b)
            {
                float* rec = record(b);
                float p[3], q[4];
                if (!rec || !detail::HoldTarget(s, b, rec, p, q))
                    continue;
                for (int k = 0; k < 3; ++k)
                    rec[k] = p[k];
                for (int k = 0; k < 4; ++k)
                    rec[integ::kGameRotOffsetFloats + k] = q[k];
                if (static_cast<int32_t>(b) == hips && fr > 0.0f)
                    detail::ApplyRoot(rec, rx, ry, ryaw);
                ++written;
            }
            if (written)
                g_counters.holdWrites.fetch_add(1, std::memory_order_relaxed);
        }
        else
        {
            if (!a.offsetsReady)
            {
                // offsets = hold target minus the live game pose of this (first post-hold) frame
                float angle[integ::kMaxMMBones] = {};
                float chainMax[4] = {};
                for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
                {
                    a.offQ[b * 4 + 0] = 0.0f; a.offQ[b * 4 + 1] = 0.0f; a.offQ[b * 4 + 2] = 0.0f; a.offQ[b * 4 + 3] = 1.0f;
                }
                a.hipsP[0] = a.hipsP[1] = a.hipsP[2] = 0.0f;
                for (uint32_t b = s.firstBone; b < nb; ++b)
                {
                    const float* rec = record(b);
                    float p[3], q[4];
                    if (!rec || !detail::HoldTarget(s, b, rec, p, q))
                        continue;
                    const float* rq = rec + integ::kGameRotOffsetFloats;
                    float gq[4] = {rq[0], rq[1], rq[2], rq[3]};
                    if (!detail::QNormalize(gq))
                        continue;
                    const float gInv[4] = {-gq[0], -gq[1], -gq[2], gq[3]};
                    float off[4];
                    detail::QMul(q, gInv, off);
                    if (!detail::QNormalize(off))
                        continue;
                    for (int k = 0; k < 4; ++k)
                        a.offQ[b * 4 + k] = off[k];
                    angle[b] = detail::QAngle(q, gq);
                    const uint8_t ch = detail::ChainOf(c, b, hips);
                    if (ch == kChainLeftLeg || ch == kChainRightLeg)
                        chainMax[ch] = angle[b] > chainMax[ch] ? angle[b] : chainMax[ch];
                    if (static_cast<int32_t>(b) == hips)
                    {
                        a.hipsP[0] = p[0] - rec[0];
                        a.hipsP[1] = p[1] - rec[1];
                        a.hipsP[2] = kMutation == 3 ? p[2] - rec[2] : 0.0f; // hips height offset always 0
                    }
                }
                // standing entry gate: the live game Hips of this first post-hold frame (bike third-person entry frame 0: 1.004 m, car
                // entry 0.973 m; a car SEAT pose is low) - larger leg cap, own leg decay, cumulative root tracking (UpdateWorld)
                const float* hrec = (hips >= 0 && static_cast<uint32_t>(hips) < nb) ? record(static_cast<uint32_t>(hips)) : nullptr;
                const float gateZ = (hrec && std::isfinite(hrec[2])) ? hrec[2] : 0.0f;
                const bool standing = kMutation != 14 && gateZ >= kStandHipsZ;
                a.standing = standing ? 1u : 0u;
                const float capDeg = (standing || kMutation == 13) ? kLegMaxOffsetStandingDeg : kLegMaxOffsetDeg;
                const float legMax = capDeg * detail::kDeg;
                uint32_t legKept = 0, legDropped = 0;
                for (uint32_t b = s.firstBone; b < nb; ++b)
                {
                    const uint8_t ch = detail::ChainOf(c, b, hips);
                    const bool drop = (ch == kChainSingle && !(angle[b] < legMax)) ||
                                      ((ch == kChainLeftLeg || ch == kChainRightLeg) && !(chainMax[ch] < legMax));
                    if (detail::IsLegChain(ch) && angle[b] > 1e-6f)
                        ++(drop ? legDropped : legKept);
                    if (drop)
                    {
                        a.offQ[b * 4 + 0] = 0.0f; a.offQ[b * 4 + 1] = 0.0f; a.offQ[b * 4 + 2] = 0.0f; a.offQ[b * 4 + 3] = 1.0f;
                    }
                    // the live game leg rotations of this frame: the first moving frame is detected against them
                    const float* rec = detail::IsLegChain(ch) ? record(b) : nullptr;
                    for (int k = 0; k < 4; ++k)
                        a.legPrevQ[b * 4 + k] = rec ? rec[integ::kGameRotOffsetFloats + k] : (k == 3 ? 1.0f : 0.0f);
                }
                a.legPrevTicks = aNow;
                a.legMoving = 0;
                a.offsetsReady = 1;
                a.offsetTicks = aNow;
                dirty = true;
                g_mountGateHipsZ.store(gateZ, std::memory_order_relaxed);
                g_mountLegCapDeg.store(capDeg, std::memory_order_relaxed);
                g_mountLegOffsetBones.store(legKept, std::memory_order_relaxed);
                g_mountLegsDropped.store(legDropped, std::memory_order_relaxed);
                if (standing)
                    g_mountStandingEpoch.store(epoch, std::memory_order_release);
            }
            else if (a.standing && !a.legMoving && detail::Seconds(aNow - a.legPrevTicks, c.tps) >= kNewFrameGapSeconds)
            {
                // standing entry: the first frame the live game legs move (the enter clip leaves its held frame 0)
                float mx = 0.0f;
                for (uint32_t b = s.firstBone; b < nb; ++b)
                {
                    const float* rec = detail::IsLegChain(detail::ChainOf(c, b, hips)) ? record(b) : nullptr;
                    if (!rec || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
                        continue;
                    float gq[4] = {rec[integ::kGameRotOffsetFloats], rec[integ::kGameRotOffsetFloats + 1],
                                   rec[integ::kGameRotOffsetFloats + 2], rec[integ::kGameRotOffsetFloats + 3]};
                    float pq[4] = {a.legPrevQ[b * 4], a.legPrevQ[b * 4 + 1], a.legPrevQ[b * 4 + 2], a.legPrevQ[b * 4 + 3]};
                    if (detail::QNormalize(gq) && detail::QNormalize(pq))
                        mx = std::fmax(mx, detail::QAngle(gq, pq));
                    for (int k = 0; k < 4; ++k)
                        a.legPrevQ[b * 4 + k] = gq[k];
                }
                if (mx > kLegMoveDeg * detail::kDeg)
                {
                    a.legMoveF = detail::LegFactor(a, c.tps, aNow);
                    a.legMoving = 1;
                    a.legMoveTicks = aNow;
                    g_mountLegMoveAt.store(t, std::memory_order_relaxed);
                }
                a.legPrevTicks = aNow;
                dirty = true;
            }
            const float t1 = detail::Seconds(aNow - a.offsetTicks, c.tps);
            const float span = kMountSeconds - detail::Seconds(a.offsetTicks - c.beginTicks, c.tps);
            const float tt = t1 > 0.0f ? t1 : 0.0f;
            const float f = span > 0.0f ? std::exp2(-tt / kMountHalfLife) * (1.0f - detail::Smoothstep(tt / span)) : 0.0f;
            const float fl = a.standing ? detail::LegFactor(a, c.tps, aNow) : 0.0f; // standing entry: leg chains
            for (uint32_t b = s.firstBone; b < nb; ++b)
            {
                float* rec = record(b);
                if (!rec)
                    continue;
                const bool isHips = static_cast<int32_t>(b) == hips;
                const float* off = &a.offQ[b * 4];
                const float fb = (a.standing && detail::IsLegChain(detail::ChainOf(c, b, hips))) ? fl : f;
                const bool needRot = fb > 0.0f && !(off[0] == 0.0f && off[1] == 0.0f && off[2] == 0.0f);
                const bool hipsOffset = a.hipsP[0] != 0.0f || a.hipsP[1] != 0.0f || a.hipsP[2] != 0.0f;
                const bool needHips = isHips && (fr > 0.0f || (f > 0.0f && hipsOffset));
                if (!needRot && !needHips)
                    continue;
                if (!detail::Finite(rec, 3) || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
                    continue;
                if (needRot)
                {
                    float d[4], o[4];
                    float* q = rec + integ::kGameRotOffsetFloats;
                    detail::QPow(off, fb, d);
                    detail::QMul(d, q, o);
                    if (detail::QNormalize(o))
                        for (int k = 0; k < 4; ++k)
                            q[k] = o[k];
                }
                if (isHips)
                {
                    for (int k = 0; k < 3; ++k)
                        rec[k] += a.hipsP[k] * f;
                    if (fr > 0.0f)
                        detail::ApplyRoot(rec, rx, ry, ryaw);
                }
                ++written;
            }
            if (written)
                g_counters.offsetWrites.fetch_add(1, std::memory_order_relaxed);
        }
        if (owner)
        {
            if (dirty)
                g_animState.Publish(a);
            g_animWriter.clear(std::memory_order_release);
        }
        return written;
    }
    // rotation phase over (or no frozen pose): only the root offset on the Hips
    if (hips >= 0 && static_cast<uint32_t>(hips) < nb)
        if (float* rec = record(static_cast<uint32_t>(hips)))
            if (detail::Finite(rec, 3) && detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
            {
                detail::ApplyRoot(rec, rx, ry, ryaw);
                ++written;
                g_counters.rootOnlyWrites.fetch_add(1, std::memory_order_relaxed);
            }
    return written;
}

// Mounted pop filter (see PopState): animation-thread write on the live records while g_popOn. Returns the bones written.
inline uint32_t PopFilter(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow)
{
    if (!g_popOn.load(std::memory_order_acquire) || !aBones || !aCount || !aRig || aNow <= 0)
        return 0;
    if (g_popBusy.test_and_set(std::memory_order_acquire))
    {
        g_counters.popSkipBusy.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    struct Release
    {
        ~Release() { g_popBusy.clear(std::memory_order_release); }
    } release;
    g_counters.popCalls.fetch_add(1, std::memory_order_relaxed);
    PopState& p = g_pop;
    if (g_popRestart.exchange(false, std::memory_order_acq_rel))
        p.valid = 0; // the exit latch held the previous frame
    Control c;
    if (g_activeEpoch.load(std::memory_order_acquire) != 0)
    {
        g_counters.popSkipEpoch.fetch_add(1, std::memory_order_relaxed);
        p.valid = 0; // the mount transition writes these frames: the history starts again after it
        return 0;
    }
    if (!g_control.Read(c) || c.frozen.rig != aRig || !c.frozen.numBones)
    {
        g_counters.popSkipRig.fetch_add(1, std::memory_order_relaxed);
        p.valid = 0;
        return 0;
    }
    const int64_t tps = TicksPerSecond();
    const uint32_t nb = c.frozen.numBones <= integ::kMaxMMBones ? c.frozen.numBones : integ::kMaxMMBones;
    const uint32_t fb = c.frozen.firstBone;
    const float gap = detail::Seconds(aNow - p.lastTicks, tps);
    const bool fresh = !p.valid || p.rig != aRig || aNow < p.lastTicks || gap > kPopGapSeconds;
    // the step against the bone's own motion over this gap (rate of the last frame without a fire); a repeated call within one
    // animation frame (gap < kNewFrameGapSeconds) and mutation 8 use the raw step; without a rate the threshold is per 1/60 s
    const bool sameFrame = gap < kNewFrameGapSeconds;
    const bool useRate = kMutation != 8 && p.rateOk && !sameFrame;
    const float scale = (useRate || sameFrame || kMutation == 8) ? 1.0f : (gap * 60.0f > 1.0f ? gap * 60.0f : 1.0f);
    float game[integ::kMaxMMBones * 4];
    float step[integ::kMaxMMBones] = {};
    uint8_t okNow[integ::kMaxMMBones] = {};
    float maxExcess = 0.0f, maxStep = 0.0f;
    // Hips translation term (live game Hips record)
    const int32_t hipsB = detail::HipsOf(c);
    const int32_t hipsGi = (hipsB >= static_cast<int32_t>(fb) && static_cast<uint32_t>(hipsB) < nb) ? c.frozen.gameIndex[hipsB] : -1;
    float* hipsRec = (hipsGi >= 0 && static_cast<uint32_t>(hipsGi) < aCount)
                         ? reinterpret_cast<float*>(aBones + size_t(hipsGi) * integ::kGameBoneStride)
                         : nullptr;
    float hg[3] = {0.0f, 0.0f, 0.0f};
    const bool hOk = hipsRec && detail::Finite(hipsRec, 3);
    if (hOk)
        for (int k = 0; k < 3; ++k)
            hg[k] = hipsRec[k];
    for (uint32_t b = fb; b < nb; ++b)
    {
        const int32_t gi = c.frozen.gameIndex[b];
        float* q = &game[b * 4];
        const float* r = (gi >= 0 && static_cast<uint32_t>(gi) < aCount)
                             ? reinterpret_cast<const float*>(aBones + size_t(gi) * integ::kGameBoneStride) + integ::kGameRotOffsetFloats
                             : nullptr;
        for (int k = 0; k < 4; ++k)
            q[k] = r ? r[k] : 0.0f;
        okNow[b] = (r && detail::Finite(q, 4) && detail::QNormalize(q)) ? 1 : 0;
        if (!fresh && okNow[b] && p.ok[b])
        {
            step[b] = detail::QAngle(&p.prevGame[b * 4], q);
            const float e = useRate ? step[b] - p.rate[b] * gap : step[b] / scale;
            maxExcess = e > maxExcess ? e : maxExcess;
            maxStep = step[b] > maxStep ? step[b] : maxStep;
        }
    }
    if (fresh)
    {
        for (uint32_t b = fb; b < nb; ++b)
        {
            p.ok[b] = okNow[b];
            p.rate[b] = 0.0f;
            for (int k = 0; k < 4; ++k)
                p.prevGame[b * 4 + k] = p.prevOut[b * 4 + k] = game[b * 4 + k];
        }
        p.valid = 1;
        p.offOn = 0;
        p.rateOk = 0;
        p.firedPrev = 0;
        p.hipsOk = hOk ? 1u : 0u;
        p.offPOn = 0;
        p.rateOkP = 0;
        p.firedPrevP = 0;
        p.rateP = 0.0f;
        for (int k = 0; k < 3; ++k)
            p.prevHipsGame[k] = p.prevHipsOut[k] = hg[k];
        p.rig = aRig;
        p.lastTicks = aNow;
        return 0;
    }
    // Hips translation: the step against the Hips' own motion over the gap (same rate rules as the rotations, own state)
    float stepP = 0.0f, excessP = 0.0f;
    if (hOk && p.hipsOk)
    {
        stepP = std::sqrt((hg[0] - p.prevHipsGame[0]) * (hg[0] - p.prevHipsGame[0]) + (hg[1] - p.prevHipsGame[1]) * (hg[1] - p.prevHipsGame[1]) +
                          (hg[2] - p.prevHipsGame[2]) * (hg[2] - p.prevHipsGame[2]));
        const bool useRateP = kMutation != 8 && p.rateOkP && !sameFrame;
        excessP = useRateP ? stepP - p.rateP * gap : stepP / scale;
    }
    if (!sameFrame)
    {
        g_popMaxStepDeg.store(std::fmax(g_popMaxStepDeg.load(std::memory_order_relaxed), maxStep / detail::kDeg), std::memory_order_relaxed);
        g_popMaxHipsCm.store(std::fmax(g_popMaxHipsCm.load(std::memory_order_relaxed), stepP * 100.0f), std::memory_order_relaxed);
    }
    UTPP_IF_MUT(VEHEXIT, 17)
    {
        maxExcess = 0.0f; // mutation 17: no pop detection (the game's one-frame jumps stay visible)
        excessP = 0.0f;
    }
    bool fireP = kMutation != 16 && std::isfinite(excessP) && excessP > kPopHipsM;
    if (fireP && p.firedPrevP && kMutation != 8)
    {
        // a translation fire right after one: only a re-anchor close to the shown Hips (a spike and back)
        const float d = std::sqrt((hg[0] - p.prevHipsOut[0]) * (hg[0] - p.prevHipsOut[0]) + (hg[1] - p.prevHipsOut[1]) * (hg[1] - p.prevHipsOut[1]) +
                                  (hg[2] - p.prevHipsOut[2]) * (hg[2] - p.prevHipsOut[2]));
        fireP = !(d > kPopHipsM);
    }
    if (!sameFrame)
    {
        p.firedPrevP = fireP ? 1u : 0u;
        if (!fireP)
        {
            p.rateP = (hOk && p.hipsOk && gap > 0.0f) ? stepP / gap : 0.0f;
            p.rateOkP = 1;
        }
    }
    else if (fireP)
        p.firedPrevP = 1;
    if (fireP)
    {
        for (int k = 0; k < 3; ++k)
            p.offP[k] = p.prevHipsOut[k] - hg[k];
        p.offPOn = 1;
        p.offPTicks = aNow;
        g_counters.pops.fetch_add(1, std::memory_order_relaxed);
        g_counters.hipsPops.fetch_add(1, std::memory_order_relaxed);
    }
    float fP = 0.0f;
    if (p.offPOn)
    {
        const float tP = detail::Seconds(aNow - p.offPTicks, tps);
        if (!(tP < kPopSeconds))
            p.offPOn = 0;
        else
            fP = 1.0f - detail::Smootherstep((tP > 0.0f ? tP : 0.0f) / kPopSeconds);
    }
    p.hipsOk = hOk ? 1u : 0u;
    bool hipsWritten = false;
    for (int k = 0; k < 3; ++k)
        p.prevHipsGame[k] = p.prevHipsOut[k] = hg[k];
    if (p.offPOn && hOk)
    {
        for (int k = 0; k < 3; ++k)
            hipsRec[k] = p.prevHipsOut[k] = hg[k] + p.offP[k] * fP;
        hipsWritten = true;
    }
    bool fire = maxExcess > kPopDeg * detail::kDeg;
    if (fire && p.firedPrev && kMutation != 8)
    {
        // a fire right after a fire: only a re-anchor close to the shown pose (a spike and back); otherwise the offset of the last
        // fire decays on its own (re-firing would hold the previous output: sustained fast motion would freeze)
        float maxOff = 0.0f;
        for (uint32_t b = fb; b < nb; ++b)
            if (okNow[b] && p.ok[b])
            {
                const float a = detail::QAngle(&p.prevOut[b * 4], &game[b * 4]);
                maxOff = a > maxOff ? a : maxOff;
            }
        fire = !(maxOff > kPopDeg * detail::kDeg);
    }
    if (!sameFrame)
    {
        p.firedPrev = fire ? 1u : 0u;
        if (!fire)
        {
            for (uint32_t b = fb; b < nb; ++b)
                p.rate[b] = (okNow[b] && p.ok[b]) ? step[b] / gap : 0.0f;
            p.rateOk = 1;
        }
    }
    else if (fire)
        p.firedPrev = 1;
    if (fire)
    {
        // a pop: continue from the pose shown on the previous frame (a running offset is part of it)
        for (uint32_t b = fb; b < nb; ++b)
        {
            float off[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            if (okNow[b] && p.ok[b])
            {
                const float* g = &game[b * 4];
                const float gInv[4] = {-g[0], -g[1], -g[2], g[3]};
                detail::QMul(&p.prevOut[b * 4], gInv, off);
                if (!detail::QNormalize(off))
                {
                    off[0] = off[1] = off[2] = 0.0f;
                    off[3] = 1.0f;
                }
            }
            for (int k = 0; k < 4; ++k)
                p.offQ[b * 4 + k] = off[k];
        }
        p.offOn = 1;
        p.offTicks = aNow;
        g_counters.pops.fetch_add(1, std::memory_order_relaxed);
    }
    float f = 0.0f;
    if (p.offOn)
    {
        const float t = detail::Seconds(aNow - p.offTicks, tps);
        if (!(t < kPopSeconds))
            p.offOn = 0;
        else
        {
            const float tt = t > 0.0f ? t : 0.0f;
            f = std::exp2(-tt / kPopHalfLife) * (1.0f - detail::Smoothstep(tt / kPopSeconds));
        }
    }
    uint32_t written = 0;
    for (uint32_t b = fb; b < nb; ++b)
    {
        float* out = &p.prevOut[b * 4];
        const float* g = &game[b * 4];
        p.ok[b] = okNow[b];
        for (int k = 0; k < 4; ++k)
        {
            p.prevGame[b * 4 + k] = g[k];
            out[k] = g[k];
        }
        if (!p.offOn || !okNow[b])
            continue;
        float d[4], o[4];
        detail::QPow(&p.offQ[b * 4], f, d);
        detail::QMul(d, g, o);
        if (!detail::QNormalize(o))
            continue;
        float* r = reinterpret_cast<float*>(aBones + size_t(c.frozen.gameIndex[b]) * integ::kGameBoneStride) + integ::kGameRotOffsetFloats;
        for (int k = 0; k < 4; ++k)
            r[k] = out[k] = o[k];
        ++written;
    }
    p.lastTicks = aNow;
    if (hipsWritten && !(p.offOn && hipsB >= 0 && okNow[hipsB]))
        ++written; // the Hips record (translation only on this frame)
    if (written)
        g_counters.popWrites.fetch_add(1, std::memory_order_relaxed);
    return written;
}

// Seat pose switch guard (SEAT POSE SWITCH above) on the live records of one animation frame. Runs only while armed (BeginMount,
// once per mount), the mounted body is the UTPP third-person one and no mount transition runs. Returns the bones written (0 =
// records untouched: observing, not fired, other rig, done).
inline uint32_t SeatGuard(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow)
{
    const uint64_t se = g_seatEpoch.load(std::memory_order_acquire);
    if (!se || !aBones || !aCount || !g_seatTpp.load(std::memory_order_acquire) ||
        g_activeEpoch.load(std::memory_order_acquire) != 0)
        return 0;
    Control c;
    if (!g_control.Read(c) || c.epoch != se || aRig != c.rig || aNow < c.beginTicks)
        return 0;
    const int32_t hips = detail::HipsOf(c);
    const uint32_t nb = c.frozen.numBones <= integ::kMaxMMBones ? c.frozen.numBones : integ::kMaxMMBones;
    if (!c.roles.valid || hips < 0 || static_cast<uint32_t>(hips) >= nb)
        return 0;
    SeatState s;
    if (!g_seatState.Read(s) || s.epoch != se)
    {
        s = SeatState{};
        s.epoch = se;
    }
    const float tMount = detail::Seconds(aNow - c.beginTicks, c.tps);
    const float tFire = s.fired ? detail::Seconds(aNow - s.fireTicks, c.tps) : 0.0f;
    // FPP stays vanilla: the mounted-TPP flag now (the latch only bridges a drop of up to kSeatFppGraceSeconds before the flag was
    // seen on at / after the fire); mutation 9: the latch alone
    const bool tppNow = kMutation == 9 || g_mountTpp.load(std::memory_order_acquire);
    bool stateDirty = false;
    if (tppNow)
    {
        if (s.fppTicks)
        {
            s.fppTicks = 0;
            stateDirty = true;
        }
        if (s.fired && !s.tppAfterFire)
        {
            s.tppAfterFire = 1;
            stateDirty = true;
        }
    }
    else if (!s.fppTicks)
    {
        s.fppTicks = aNow;
        stateDirty = true;
    }
    const bool fppEnd = !tppNow && (s.tppAfterFire || detail::Seconds(aNow - s.fppTicks, c.tps) > kSeatFppGraceSeconds);
    if (fppEnd || (s.fired && !(tFire < kSeatSeconds)) || (!s.fired && tMount > kSeatWatchSeconds))
    {
        uint64_t e = se;
        g_seatEpoch.compare_exchange_strong(e, 0, std::memory_order_acq_rel); // done: once per mount (or FPP)
        return 0;
    }
    auto record = [&](uint32_t b) -> float* {
        const int32_t gi = c.frozen.gameIndex[b];
        return (gi < 0 || static_cast<uint32_t>(gi) >= aCount) ? nullptr
                                                               : reinterpret_cast<float*>(aBones + size_t(gi) * integ::kGameBoneStride);
    };
    const float* hipsRec = record(static_cast<uint32_t>(hips));
    if (!hipsRec)
        return 0;
    const bool owner = !g_seatWriter.test_and_set(std::memory_order_acquire);
    bool dirty = stateDirty;
    if (owner)
    {
        const float gap = detail::Seconds(aNow - s.prevTicks, c.tps);
        const bool newFrame = !s.prevValid || gap >= kNewFrameGapSeconds;
        if (newFrame)
        {
            if (!(!s.fired && s.prevValid && gap <= kSeatMaxGapSeconds))
                s.rateOk = 0; // no comparison on this frame (first one, stalled gap): the next one has no previous rate
            else
            {
                const float* hp = &s.prev[size_t(hips) * 7];
                const float hipsStep = std::sqrt((hipsRec[0] - hp[0]) * (hipsRec[0] - hp[0]) + (hipsRec[1] - hp[1]) * (hipsRec[1] - hp[1]) +
                                                 (hipsRec[2] - hp[2]) * (hipsRec[2] - hp[2]));
                float torso = 0.0f;
                for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
                {
                    const float* rec = c.roles.torso[b] ? record(b) : nullptr;
                    if (!rec)
                        continue;
                    const float* rq = rec + integ::kGameRotOffsetFloats;
                    const float* pq = &s.prev[size_t(b) * 7 + 3];
                    if (detail::Finite(rq, 4) && detail::Finite(pq, 4))
                        torso = std::fmax(torso, detail::QAngle(rq, pq));
                }
                // a cut, not motion: the step above what the previous compared step's rate gives over this gap (a frame hitch on
                // the enter clip is its motion); without a rate (mutation 8: always) the absolute thresholds per 1/60 s (mutation 8:
                // per frame)
                const bool useRate = kMutation != 8 && s.rateOk;
                const float expH = useRate ? s.hipsRate * gap : 0.0f;
                const float expT = useRate ? s.torsoRate * gap : 0.0f;
                const float scale = (useRate || kMutation == 8) ? 1.0f : (gap * 60.0f > 1.0f ? gap * 60.0f : 1.0f);
                if (std::isfinite(hipsStep) && hipsStep - expH >= kSeatMinHipsStep * scale &&
                    torso - expT >= kSeatMinTorsoDeg * detail::kDeg * scale && (!useRate || hipsStep >= kSeatStepRatio * expH))
                {
                    // fire: offsets = previous frame's pose minus this one (the previous frame showed the game pose as is)
                    for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
                        s.offOk[b] = 0;
                    for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
                    {
                        const float* rec = record(b);
                        const float* pv = &s.prev[size_t(b) * 7];
                        if (!rec || !detail::Finite(rec, 3) || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4) || !detail::Finite(pv, 7))
                            continue;
                        float gq[4] = {rec[integ::kGameRotOffsetFloats], rec[integ::kGameRotOffsetFloats + 1],
                                       rec[integ::kGameRotOffsetFloats + 2], rec[integ::kGameRotOffsetFloats + 3]};
                        float pq[4] = {pv[3], pv[4], pv[5], pv[6]};
                        if (!detail::QNormalize(gq) || !detail::QNormalize(pq))
                            continue;
                        const float gInv[4] = {-gq[0], -gq[1], -gq[2], gq[3]};
                        float off[4];
                        detail::QMul(pq, gInv, off);
                        if (!detail::QNormalize(off))
                            continue;
                        for (int k = 0; k < 4; ++k)
                            s.offQ[b * 4 + k] = off[k];
                        for (int k = 0; k < 3; ++k)
                            s.offP[b * 3 + k] = pv[k] - rec[k];
                        s.offOk[b] = 1;
                    }
                    s.fired = 1;
                    s.fireTicks = aNow;
                    s.tppAfterFire = tppNow ? 1u : 0u;
                    g_counters.seatFires.fetch_add(1, std::memory_order_relaxed);
                    g_seatLastHipsCm.store(hipsStep * 100.0f, std::memory_order_relaxed);
                    g_seatLastTorsoDeg.store(torso / detail::kDeg, std::memory_order_relaxed);
                    g_seatLastAt.store(tMount, std::memory_order_relaxed);
                    g_seatLastMountTpp.store(g_mountTpp.load(std::memory_order_relaxed) ? 1 : 0, std::memory_order_relaxed);
                }
                s.rateOk = (std::isfinite(hipsStep) && gap > 0.0f) ? 1u : 0u;
                s.hipsRate = s.rateOk ? hipsStep / gap : 0.0f;
                s.torsoRate = s.rateOk ? torso / gap : 0.0f;
            }
            // remember this frame's live game pose (before any write of this call)
            for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
            {
                const float* rec = record(b);
                float* pv = &s.prev[size_t(b) * 7];
                if (!rec)
                {
                    for (int k = 0; k < 7; ++k)
                        pv[k] = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }
                for (int k = 0; k < 3; ++k)
                    pv[k] = rec[k];
                for (int k = 0; k < 4; ++k)
                    pv[3 + k] = rec[integ::kGameRotOffsetFloats + k];
            }
            s.prevTicks = aNow;
            s.prevValid = 1;
            dirty = true;
        }
    }
    uint32_t written = 0;
    if (s.fired)
    {
        const float t = detail::Seconds(aNow - s.fireTicks, c.tps);
        const float tt = t > 0.0f ? t : 0.0f;
        const float f = tt < kSeatSeconds ? 1.0f - detail::Smootherstep(tt / kSeatSeconds) : 0.0f;
        if (f > 0.0f)
        {
            for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
            {
                float* rec = s.offOk[b] ? record(b) : nullptr;
                if (!rec || !detail::Finite(rec, 3) || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
                    continue;
                float* q = rec + integ::kGameRotOffsetFloats;
                float d[4], o[4];
                detail::QPow(&s.offQ[b * 4], f, d);
                detail::QMul(d, q, o);
                if (!detail::QNormalize(o))
                    continue;
                for (int k = 0; k < 4; ++k)
                    q[k] = o[k];
                for (int k = 0; k < 3; ++k)
                    rec[k] += s.offP[b * 3 + k] * f;
                ++written;
            }
            if (written)
                g_counters.seatWrites.fetch_add(1, std::memory_order_relaxed);
        }
    }
    if (owner)
    {
        if (dirty)
            g_seatState.Publish(s);
        g_seatWriter.clear(std::memory_order_release);
    }
    return written;
}

// Seated neck cut (SEATED NECK CUT) on the pose the FK pass receives (FkHook ApplyMotionMatching, every write owner). The pop is not
// in the pose of the early mapping job (2026-10-29 18:22:46: the mounted guards ran on every ride frame there - pop filter calls 506,
// as many pop filter fires in ride 1 as in ride 2 without the pop, no frozen frame seen by the seat guard - while the FK-stage trace s0
// shows the frozen frame and the pop); the game adds it after the mapping. Runs while armed (BeginMount, once per mount) and no mount transition runs; fires and writes only with the
// mounted-TPP flag on. One owner per call (try-lock; a busy call skips). A second call on the same animation frame
// whose reference bone already holds this frame's output writes nothing (no double correction). Returns the bones written.
inline uint32_t NeckGuard(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow)
{
    const uint64_t ne = g_neckEpoch.load(std::memory_order_acquire);
    // no TPP latch here (the fire and every write need the mounted-TPP flag of their frame): the guard observes from the first frame
    // after the mount transition whatever the latch did
    if (!ne || !aBones || !aCount || g_activeEpoch.load(std::memory_order_acquire) != 0)
        return 0;
    Control c;
    if (!g_control.Read(c) || c.epoch != ne || aNow < c.beginTicks)
        return 0;
    if (aRig != c.rig)
    {
        g_neckRigSkips.fetch_add(1, std::memory_order_relaxed);
        return 0;
    }
    const int32_t hips = detail::HipsOf(c);
    const uint32_t nb = c.frozen.numBones <= integ::kMaxMMBones ? c.frozen.numBones : integ::kMaxMMBones;
    if (!c.roles.valid || hips < 0 || static_cast<uint32_t>(hips) >= nb)
        return 0;
    if (g_neckBusy.test_and_set(std::memory_order_acquire))
        return 0;
    struct Release
    {
        ~Release() { g_neckBusy.clear(std::memory_order_release); }
    } release;
    NeckState& n = g_neck;
    if (n.epoch != ne)
    {
        n = NeckState{};
        n.epoch = ne;
    }
    g_neckCalls.fetch_add(1, std::memory_order_relaxed);
    const float tMount = detail::Seconds(aNow - c.beginTicks, c.tps);
    const bool tppNow = g_mountTpp.load(std::memory_order_acquire);
    const float tFire = n.fired ? detail::Seconds(aNow - n.fireTicks, c.tps) : 0.0f;
    // FPP after a fire seen in TPP ends the correction at once; the watch ends without a fire; the correction ends at kSeatNeckSeconds
    if (n.fired && !tppNow)
        n.done = 1;
    if ((!n.fired && tMount > kSeatWatchSeconds) || (n.fired && (n.done || !(tFire < kSeatNeckSeconds))))
    {
        uint64_t e = ne;
        g_neckEpoch.compare_exchange_strong(e, 0, std::memory_order_acq_rel); // done: once per mount
        return 0;
    }
    auto record = [&](uint32_t b) -> float* {
        const int32_t gi = c.frozen.gameIndex[b];
        return (gi < 0 || static_cast<uint32_t>(gi) >= aCount) ? nullptr
                                                               : reinterpret_cast<float*>(aBones + size_t(gi) * integ::kGameBoneStride);
    };
    const float* hipsRec = record(static_cast<uint32_t>(hips));
    if (!hipsRec)
        return 0;
    const float gap = detail::Seconds(aNow - n.prevTicks, c.tps);
    const bool newFrame = !n.prevValid || gap >= kNewFrameGapSeconds;
    if (!newFrame)
    {
        // a repeated call on this animation frame: nothing to do when the records already carry this frame's output
        const float* rr = (n.refBone >= 0) ? record(static_cast<uint32_t>(n.refBone)) : nullptr;
        if (!n.wroteFrame || (rr && std::memcmp(rr + integ::kGameRotOffsetFloats, n.refOut, sizeof n.refOut) == 0))
            return 0;
    }
    else
    {
        n.wroteFrame = 0;
        if (n.prevValid && gap <= kSeatMaxGapSeconds && !n.fired)
        {
            const float* hp = &n.prevHips[0];
            const float hipsStep = std::sqrt((hipsRec[0] - hp[0]) * (hipsRec[0] - hp[0]) + (hipsRec[1] - hp[1]) * (hipsRec[1] - hp[1]) +
                                             (hipsRec[2] - hp[2]) * (hipsRec[2] - hp[2]));
            // the neck-chain step, the torso step (diagnostics) and the frozen-frame test (the seating edge proxy) over the role bones
            // (Hips, torso, neck, upper body, legs: the bones of the recorded evidence)
            float neck = 0.0f, torso = 0.0f, frozenD = 0.0f;
            uint32_t frozenN = 0;
            for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
            {
                const bool role = static_cast<int32_t>(b) == hips || c.roles.torso[b] || c.roles.neck[b] || c.roles.upper[b] ||
                                  c.roles.chain[b] != kChainNone;
                const float* rec = (role && n.prevOk[b]) ? record(b) : nullptr;
                if (!rec)
                    continue;
                const float* rq = rec + integ::kGameRotOffsetFloats;
                const float* pq = &n.prevQ[b * 4];
                if (!detail::Finite(rq, 4))
                    continue;
                // q and -q are the same rotation (the recorded frozen frames flip a hand's sign)
                float dSame = 0.0f, dFlip = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    dSame = std::fmax(dSame, std::fabs(rq[k] - pq[k]));
                    dFlip = std::fmax(dFlip, std::fabs(rq[k] + pq[k]));
                }
                frozenD = std::fmax(frozenD, std::fmin(dSame, dFlip));
                ++frozenN;
                if (c.roles.neck[b])
                    neck = std::fmax(neck, detail::QAngle(rq, pq));
                if (c.roles.torso[b])
                    torso = std::fmax(torso, detail::QAngle(rq, pq));
            }
            if (frozenN && frozenD <= kSeatEdgeFrozenEps)
            {
                n.edgeTicks = aNow;
                g_seatNeckEdgeAt.store(tMount, std::memory_order_relaxed);
            }
            // a cut, not motion: the step above what the previous compared step's rate gives over this gap; without a rate the
            // threshold per 1/60 s
            const float expN = n.rateOk ? n.rate * gap : 0.0f;
            const float excessN = neck - expN;
            const float scaleN = n.rateOk ? 1.0f : (gap * 60.0f > 1.0f ? gap * 60.0f : 1.0f);
            const bool inWindow = n.edgeTicks != 0 && aNow >= n.edgeTicks &&
                                  detail::Seconds(aNow - n.edgeTicks, c.tps) <= kSeatNeckWindowSeconds;
            if (inWindow && std::isfinite(excessN))
                g_seatNeckStepDeg.store(std::fmax(g_seatNeckStepDeg.load(std::memory_order_relaxed), excessN / detail::kDeg),
                                        std::memory_order_relaxed);
            const bool fire = tppNow && inWindow && std::isfinite(excessN) && excessN >= kSeatNeckMinDeg * detail::kDeg * scaleN &&
                              (!n.rateOk || neck >= kSeatStepRatio * expN);
            if (fire)
            {
                // offsets = previous frame's rotation minus this one on the upper-body bones; pop = live * pre^-1 (= D^-1) of the
                // neck chain is the reference of the remaining game deviation r
                float ref = 0.0f;
                for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
                    n.ok[b] = 0;
                for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
                {
                    const float* rec = (c.roles.upper[b] && n.prevOk[b]) ? record(b) : nullptr;
                    if (!rec || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
                        continue;
                    float gq[4] = {rec[integ::kGameRotOffsetFloats], rec[integ::kGameRotOffsetFloats + 1],
                                   rec[integ::kGameRotOffsetFloats + 2], rec[integ::kGameRotOffsetFloats + 3]};
                    float pq[4] = {n.prevQ[b * 4], n.prevQ[b * 4 + 1], n.prevQ[b * 4 + 2], n.prevQ[b * 4 + 3]};
                    if (!detail::QNormalize(gq) || !detail::QNormalize(pq))
                        continue;
                    const float gInv[4] = {-gq[0], -gq[1], -gq[2], gq[3]};
                    float off[4];
                    detail::QMul(pq, gInv, off);
                    if (!detail::QNormalize(off))
                        continue;
                    for (int k = 0; k < 4; ++k)
                    {
                        n.offQ[b * 4 + k] = off[k];
                        n.preQ[b * 4 + k] = pq[k];
                    }
                    n.ok[b] = 1;
                    if (c.roles.neck[b])
                    {
                        float v[3];
                        detail::QRotVec(off, v);
                        ref += v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                        if (n.refBone < 0)
                            n.refBone = static_cast<int32_t>(b);
                    }
                }
                if (ref > 1e-6f)
                {
                    n.fired = 1;
                    n.fireTicks = aNow;
                    n.ref = ref;
                    g_counters.neckFires.fetch_add(1, std::memory_order_relaxed);
                    UTPP_MUT_IF (kMutation == 21) // mutation 21: the old shared counters / fields (the neck fire counts as a seat guard fire)
                    {
                        g_counters.seatFires.fetch_add(1, std::memory_order_relaxed);
                        g_seatLastHipsCm.store(hipsStep * 100.0f, std::memory_order_relaxed);
                        g_seatLastAt.store(tMount, std::memory_order_relaxed);
                    }
                    g_neckLastHipsCm.store(hipsStep * 100.0f, std::memory_order_relaxed);
                    g_neckLastTorsoDeg.store(torso / detail::kDeg, std::memory_order_relaxed);
                    g_neckLastAt.store(tMount, std::memory_order_relaxed);
                    g_seatNeckFired.store(1, std::memory_order_relaxed);
                    g_seatNeckStepDeg.store(excessN / detail::kDeg, std::memory_order_relaxed);
                }
                else
                    n.refBone = -1;
            }
            n.rateOk = (std::isfinite(neck) && gap > 0.0f) ? 1u : 0u;
            n.rate = n.rateOk ? neck / gap : 0.0f;
        }
        else if (!n.fired)
            n.rateOk = 0; // no comparison on this frame (first one, stalled gap): the next one has no previous rate
        // remember this frame's live pose (before any write of this call)
        for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
        {
            const float* rec = record(b);
            n.prevOk[b] = (rec && detail::Finite(rec + integ::kGameRotOffsetFloats, 4)) ? 1 : 0;
            if (n.prevOk[b])
                std::memcpy(&n.prevQ[b * 4], rec + integ::kGameRotOffsetFloats, 4 * sizeof(float));
        }
        for (int k = 0; k < 3; ++k)
            n.prevHips[k] = hipsRec[k];
        n.prevTicks = aNow;
        n.prevValid = 1;
    }
    if (!n.fired || n.done)
        return 0;
    // D_b^(r g) * live_b: r = the live deviation from the pre-pop rotation (live * pre^-1) projected on the pop, summed over the neck
    // chain, relative to the pop itself (the clip's own progress across the pop direction does not count; a return past the pre-pop
    // rotation gives r <= 0); g = 1 - smootherstep(t / kSeatNeckSeconds)
    const float t = detail::Seconds(aNow - n.fireTicks, c.tps);
    const float tt = t > 0.0f ? t : 0.0f;
    const float g = tt < kSeatNeckSeconds ? 1.0f - detail::Smootherstep(tt / kSeatNeckSeconds) : 0.0f;
    float sum = 0.0f;
    for (uint32_t b = c.frozen.firstBone; b < nb && g > 0.0f; ++b)
    {
        const float* rec = (n.ok[b] && c.roles.neck[b]) ? record(b) : nullptr;
        if (!rec || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
            continue;
        const float* pq = &n.preQ[b * 4];
        const float preInv[4] = {-pq[0], -pq[1], -pq[2], pq[3]};
        float dev[4], dv[3], pop[3];
        detail::QMul(rec + integ::kGameRotOffsetFloats, preInv, dev);
        if (!detail::QNormalize(dev))
            continue;
        detail::QRotVec(dev, dv);
        detail::QRotVec(&n.offQ[b * 4], pop);
        sum -= dv[0] * pop[0] + dv[1] * pop[1] + dv[2] * pop[2];
    }
    float r = n.ref > 0.0f ? sum / n.ref : 0.0f;
    r = std::isfinite(r) ? (r < 0.0f ? 0.0f : (r > 1.0f ? 1.0f : r)) : 0.0f;
    const float w = r * g;
    if (!(w > 0.0f))
        return 0;
    uint32_t written = 0;
    for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
    {
        float* rec = n.ok[b] ? record(b) : nullptr;
        if (!rec || !detail::Finite(rec + integ::kGameRotOffsetFloats, 4))
            continue;
        float* q = rec + integ::kGameRotOffsetFloats;
        float d[4], o[4];
        detail::QPow(&n.offQ[b * 4], w, d);
        detail::QMul(d, q, o);
        if (!detail::QNormalize(o))
            continue;
        for (int k = 0; k < 4; ++k)
            q[k] = o[k];
        if (static_cast<int32_t>(b) == n.refBone)
            std::memcpy(n.refOut, q, sizeof n.refOut);
        ++written;
    }
    if (written)
    {
        n.wroteFrame = 1;
        g_counters.neckWrites.fetch_add(1, std::memory_order_relaxed);
    }
    return written;
}

// Both mounted guards on a frame without a mount transition write: the seat pose switch guard first (it smooths the seating switch
// itself, so its output no longer steps above kPopDeg), then the pop filter on the result. aDenied = perspective denial: the pop
// filter only under the mounted-TPP flag (the seated game pose is wanted there). Returns the seat guard's bones; the pop filter's
// writes do not make a transition frame.
inline uint32_t MountedGuards(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow, bool aDenied)
{
    const uint32_t s = SeatGuard(aBones, aCount, aRig, aNow);
    if (!aDenied || g_mountTpp.load(std::memory_order_acquire))
        PopFilter(aBones, aCount, aRig, aNow);
    return s;
}

namespace detail
{
// A captured pose (translation + rotation of every mapped bone from firstBone) onto the records. Returns the bones written.
inline uint32_t WriteCapturedPose(uint8_t* aBones, uint32_t aCount, const integ::PoseSnapshot& aS)
{
    const uint32_t nb = aS.numBones <= integ::kMaxMMBones ? aS.numBones : integ::kMaxMMBones;
    uint32_t n = 0;
    for (uint32_t b = aS.firstBone; b < nb; ++b)
    {
        const int32_t gi = aS.gameIndex[b];
        if (gi < 0 || static_cast<uint32_t>(gi) >= aCount)
            continue;
        const float* m = &aS.pose[size_t(b) * 7];
        float q[4] = {m[3], m[4], m[5], m[6]};
        if (!Finite(m, 7) || !QNormalize(q))
            continue;
        float* r = reinterpret_cast<float*>(aBones + size_t(gi) * integ::kGameBoneStride);
        for (int k = 0; k < 3; ++k)
            r[k] = m[k];
        for (int k = 0; k < 4; ++k)
            r[integ::kGameRotOffsetFloats + k] = q[k];
        ++n;
    }
    return n;
}
// The live game Hips translation of the ride's bone map (read before the mounted guards write).
inline bool RawHips(const uint8_t* aBones, uint32_t aCount, uint64_t aRig, float aOut[3])
{
    if (!aBones || !aCount || !(g_rideCapture.load(std::memory_order_acquire) || g_exitHold.load(std::memory_order_acquire)))
        return false;
    Control c;
    if (!g_control.Read(c) || c.frozen.rig != aRig)
        return false;
    const int32_t hips = HipsOf(c);
    const uint32_t nb = c.frozen.numBones <= integ::kMaxMMBones ? c.frozen.numBones : integ::kMaxMMBones;
    if (hips < 0 || static_cast<uint32_t>(hips) >= nb)
        return false;
    const int32_t gi = c.frozen.gameIndex[hips];
    if (gi < 0 || static_cast<uint32_t>(gi) >= aCount)
        return false;
    const float* r = reinterpret_cast<const float*>(aBones + size_t(gi) * integ::kGameBoneStride);
    if (!Finite(r, 3))
        return false;
    for (int k = 0; k < 3; ++k)
        aOut[k] = r[k];
    return true;
}
// Break of a standing ride capture (EXIT LATCH) of this frame against the latch history (caller holds g_latchBusy, history current).
inline bool LatchBreak(const LatchState& aL, const float aRawHips[3], bool aRawOk, int64_t aNow, int64_t aTps)
{
    if (!aRawOk || !aL.prevValid || aNow <= aL.prevTicks || !(aL.prevHips[2] >= kStandHipsZ))
        return false;
    const float gap = Seconds(aNow - aL.prevTicks, aTps);
    if (gap < kNewFrameGapSeconds || gap > kPopGapSeconds)
        return false;
    const float scale = gap * 60.0f > 1.0f ? gap * 60.0f : 1.0f;
    const float drop = aL.prevHips[2] - aRawHips[2];
    const float stepH = std::sqrt((aRawHips[0] - aL.prevHips[0]) * (aRawHips[0] - aL.prevHips[0]) +
                                  (aRawHips[1] - aL.prevHips[1]) * (aRawHips[1] - aL.prevHips[1]) + drop * drop);
    const bool brk = drop > kLatchHipsDrop * scale || stepH > kLatchHipsStep * scale;
    return kMutation != 11 && brk; // mutation 11: no exit latch
}
} // namespace detail

// Allowed frames keep the ride capture where it was (the live game pose, before the transition write and the guards); a frame that
// breaks (EXIT LATCH) is not captured: ExitStep holds the previous capture. Busy latch (another thread): no capture this frame.
inline bool ExitBreakNow(const float aRawHips[3], bool aRawOk, uint64_t aRig, int64_t aNow)
{
    if (g_latchBusy.test_and_set(std::memory_order_acquire))
        return true;
    const LatchState& L = g_latch;
    const bool current = L.resetSeen == g_latchReset.load(std::memory_order_acquire) && L.rig == aRig;
    const bool brk = current && detail::LatchBreak(L, aRawHips, aRawOk, aNow, TicksPerSecond());
    g_latchBusy.clear(std::memory_order_release);
    return brk;
}

// Exit latch on one animation frame after the mounted guards (EXIT LATCH): the held capture, the break detection on a standing ride
// capture, the release blend of an unpinned hold, then the ride capture of the shown pose. aRawHips = the live game Hips before the
// guards (aRawOk false: none). aAllows = the perspective permission of this frame. Returns the bones written by the hold or the
// release blend (non-zero = the caller keeps these records: no MM blend).
inline uint32_t ExitStep(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow, const float aRawHips[3], bool aRawOk,
                         bool aAllows, bool aCapture)
{
    const bool holdOn = g_exitHold.load(std::memory_order_acquire);
    const bool rideCap = g_rideCapture.load(std::memory_order_acquire);
    if ((!holdOn && !rideCap) || !aBones || !aCount || !aRig || aNow <= 0)
        return 0;
    const int64_t tps = TicksPerSecond();
    if (g_latchBusy.test_and_set(std::memory_order_acquire))
    {
        // another thread runs the latch on this frame: the hold only (no detection, no capture)
        Capture h;
        if (!holdOn || !g_exitHoldPose.Read(h) || h.snap.rig != aRig)
            return 0;
        return detail::WriteCapturedPose(aBones, aCount, h.snap);
    }
    struct Release
    {
        ~Release() { g_latchBusy.clear(std::memory_order_release); }
    } release;
    LatchState& L = g_latch;
    const uint32_t reset = g_latchReset.load(std::memory_order_acquire);
    if (L.resetSeen != reset || L.rig != aRig)
    {
        L.prevValid = 0;
        L.relOn = 0;
        L.resetSeen = reset;
        L.rig = aRig;
    }
    const float gap = L.prevValid ? detail::Seconds(aNow - L.prevTicks, tps) : 0.0f;
    const bool newFrame = !L.prevValid || aNow < L.prevTicks || gap >= kNewFrameGapSeconds;
    auto remember = [&]() {
        if (!aRawOk)
            L.prevValid = 0;
        else if (newFrame)
        {
            for (int k = 0; k < 3; ++k)
                L.prevHips[k] = aRawHips[k];
            L.prevTicks = aNow;
            L.prevValid = 1;
        }
    };

    // 1. a held capture
    if (holdOn)
    {
        const bool pinned = g_exitHoldPinned.load(std::memory_order_acquire);
        if (pinned && aAllows && g_exitHoldPublished.load(std::memory_order_acquire))
        {
            // the hand-in snapshot is published and MM writes on this frame: the hold ends (MM continues from the same capture)
            g_exitHold.store(false, std::memory_order_release);
            g_exitHoldPinned.store(false, std::memory_order_release);
            g_exitHoldPublished.store(false, std::memory_order_release);
            g_counters.exitHoldEnds.fetch_add(1, std::memory_order_relaxed);
            remember();
            return 0;
        }
        Capture h;
        const bool okH = g_exitHoldPose.Read(h) && h.snap.rig == aRig && h.ticks > 0;
        const int64_t from = pinned ? g_exitHoldPinTicks.load(std::memory_order_relaxed) : h.ticks;
        const float age = okH ? detail::Seconds(aNow - from, tps) : 1e9f;
        if (okH && age < (pinned ? kExitHoldMaxSeconds : kCaptureMaxAgeSeconds))
        {
            g_popRestart.store(true, std::memory_order_release);
            const uint32_t n = detail::WriteCapturedPose(aBones, aCount, h.snap);
            if (n)
                g_counters.exitHoldFrames.fetch_add(1, std::memory_order_relaxed);
            remember();
            return n;
        }
        // aged out. Without an edge (unpinned) the shown pose blends from the held capture to the live one (no step); a pinned hold
        // whose hand-in never reached an allowed frame just ends (cap)
        if (okH && !pinned)
        {
            const integ::PoseSnapshot& m = h.snap;
            const uint32_t nb = m.numBones <= integ::kMaxMMBones ? m.numBones : integ::kMaxMMBones;
            for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
                L.relOk[b] = 0;
            for (uint32_t b = m.firstBone; b < nb; ++b)
            {
                const int32_t gi = m.gameIndex[b];
                if (gi < 0 || static_cast<uint32_t>(gi) >= aCount)
                    continue;
                const float* r = reinterpret_cast<const float*>(aBones + size_t(gi) * integ::kGameBoneStride);
                const float* hp = &m.pose[size_t(b) * 7];
                float gq[4] = {r[integ::kGameRotOffsetFloats], r[integ::kGameRotOffsetFloats + 1], r[integ::kGameRotOffsetFloats + 2],
                               r[integ::kGameRotOffsetFloats + 3]};
                float hq[4] = {hp[3], hp[4], hp[5], hp[6]};
                if (!detail::Finite(r, 3) || !detail::Finite(hp, 7) || !detail::QNormalize(gq) || !detail::QNormalize(hq))
                    continue;
                const float gInv[4] = {-gq[0], -gq[1], -gq[2], gq[3]};
                float off[4];
                detail::QMul(hq, gInv, off);
                if (!detail::QNormalize(off))
                    continue;
                for (int k = 0; k < 4; ++k)
                    L.relQ[b * 4 + k] = off[k];
                for (int k = 0; k < 3; ++k)
                    L.relP[b * 3 + k] = hp[k] - r[k];
                L.relOk[b] = 1;
            }
            L.relOn = 1;
            L.relTicks = aNow;
            g_counters.exitHoldReleases.fetch_add(1, std::memory_order_relaxed);
        }
        g_exitHold.store(false, std::memory_order_release);
        g_exitHoldPinned.store(false, std::memory_order_release);
        g_exitHoldPublished.store(false, std::memory_order_release);
    }

    // 2. break of a standing ride capture: the game's squat swap before the vehicle-bit edge (Hips drop / step per 1/60 s)
    if (rideCap && newFrame && detail::LatchBreak(L, aRawHips, aRawOk, aNow, tps))
    {
        Capture cap;
        if (g_captures.Read(cap) && cap.snap.rig == aRig && cap.ticks > 0 && detail::Seconds(aNow - cap.ticks, tps) < kCaptureMaxAgeSeconds)
        {
            g_exitHoldPose.Publish(cap);
            g_exitHoldPinned.store(false, std::memory_order_release);
            g_exitHoldPublished.store(false, std::memory_order_release);
            g_exitHold.store(true, std::memory_order_release);
            L.relOn = 0;
            g_popRestart.store(true, std::memory_order_release);
            g_counters.exitBreaks.fetch_add(1, std::memory_order_relaxed);
            const uint32_t n = detail::WriteCapturedPose(aBones, aCount, cap.snap);
            if (n)
                g_counters.exitHoldFrames.fetch_add(1, std::memory_order_relaxed);
            remember();
            return n;
        }
    }
    remember();

    // 3. release blend of an unpinned hold (1 - smootherstep: zero speed where the hold ends and at the end)
    uint32_t written = 0;
    if (L.relOn)
    {
        const float tr = detail::Seconds(aNow - L.relTicks, tps);
        Control c;
        if (!(tr < kLatchReleaseSeconds) || !g_control.Read(c) || c.frozen.rig != aRig)
            L.relOn = 0;
        else
        {
            const float f = 1.0f - detail::Smootherstep((tr > 0.0f ? tr : 0.0f) / kLatchReleaseSeconds);
            const uint32_t nb = c.frozen.numBones <= integ::kMaxMMBones ? c.frozen.numBones : integ::kMaxMMBones;
            for (uint32_t b = c.frozen.firstBone; b < nb; ++b)
            {
                const int32_t gi = c.frozen.gameIndex[b];
                if (!L.relOk[b] || gi < 0 || static_cast<uint32_t>(gi) >= aCount)
                    continue;
                float* r = reinterpret_cast<float*>(aBones + size_t(gi) * integ::kGameBoneStride);
                float* q = r + integ::kGameRotOffsetFloats;
                if (!detail::Finite(r, 3) || !detail::Finite(q, 4))
                    continue;
                float d[4], o[4];
                detail::QPow(&L.relQ[b * 4], f, d);
                detail::QMul(d, q, o);
                if (!detail::QNormalize(o))
                    continue;
                for (int k = 0; k < 4; ++k)
                    q[k] = o[k];
                for (int k = 0; k < 3; ++k)
                    r[k] += L.relP[b * 3 + k] * f;
                ++written;
            }
        }
    }
    // 4. the ride capture of the shown pose (denied frames: after the guards / the release blend; allowed frames captured before)
    if (rideCap && aCapture)
        CaptureIfWanted(aBones, aCount, aRig, aNow);
    return written;
}

// What the FK / early-mapping write path calls (FkHook.cpp): nothing without a Begin* / SetVehicleCapture call. A perspective
// denial ends the mount transition (not under g_mountTpp, see above); on the other frames the transition write, else the mounted
// guards and the exit latch. Denied frames (the vehicle hold's veto covers the whole ride and exit) capture the shown pose after the
// guards; allowed frames keep the capture of the live game pose before the transition write (unchanged), except on a latch break.
// Non-zero = a transition frame: the caller skips the MM blend, reload contacts, aim offset and CaptureGamePoseSample.
inline uint32_t FkSide(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow, bool aPerspectiveAllows)
{
    if (!Wanted())
        return 0;
    float rawHips[3] = {0.0f, 0.0f, 0.0f};
    if (!aPerspectiveAllows)
    {
        g_counters.fkSideDenied.fetch_add(1, std::memory_order_relaxed);
        const uint64_t e = g_activeEpoch.load(std::memory_order_acquire);
        // vehicle hold veto (g_mountTpp): the running mount transition goes on; any other denial ends it
        const bool vehicleVeto = kMutation != 6 && e != 0 && g_mountTpp.load(std::memory_order_acquire);
        if (vehicleVeto)
        {
            const uint32_t n = ApplyIfActive(aBones, aCount, aRig, aNow);
            if (n)
                return n;
            // the transition expired on this frame: the guards below
        }
        else if (e)
        {
            detail::EndEpoch(e);
            g_counters.perspectiveAborts.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }
        // no mount transition: the seat pose switch guard runs through the denial (it checks its TPP latch itself)
        const bool rawOk = detail::RawHips(aBones, aCount, aRig, rawHips);
        const uint32_t s = MountedGuards(aBones, aCount, aRig, aNow, true);
        UTPP_MUT_IF (kMutation == 10)
            if (!g_exitHold.load(std::memory_order_acquire))
                return s; // mutation 10: no capture under the denial
        const uint32_t x = ExitStep(aBones, aCount, aRig, aNow, rawHips, rawOk, false, true);
        return x ? x : s;
    }
    const bool hold = g_rearmHold.load(std::memory_order_acquire);
    // allowed frames: the capture of the live game pose first, as before the exit latch (live exit / jump-out and ride capture); a
    // ride frame that breaks the latch is not captured (ExitStep holds the previous capture)
    const bool rawOk = detail::RawHips(aBones, aCount, aRig, rawHips);
    if (!g_rideCapture.load(std::memory_order_acquire) || !ExitBreakNow(rawHips, rawOk, aRig, aNow))
        CaptureIfWanted(aBones, aCount, aRig, aNow);
    const uint32_t n = ApplyIfActive(aBones, aCount, aRig, aNow);
    if (n)
        return n;
    const uint32_t s = MountedGuards(aBones, aCount, aRig, aNow, false);
    const uint32_t x = ExitStep(aBones, aCount, aRig, aNow, rawHips, rawOk, true, false);
    if (hold)
    {
        g_counters.rearmHoldFrames.fetch_add(1, std::memory_order_relaxed);
        return x ? x : (s ? s : 1); // re-arm hold: the live game pose stays on this frame (the caller skips the MM blend)
    }
    return x ? x : s;
}

// The Mapping-owner write path (PoseHook ApplyEarlyMapping, called once per mapping job): FkSide on the main mapping job only
// (MainMappingJob against the bone map of the running transition / capture). A helper job returns 0 before touching any record,
// capture or epoch (EvaluateMapping rejects it as well, so it writes nothing at all).
inline uint32_t FkSideMapping(uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aNow, bool aPerspectiveAllows,
                              std::span<const uint64_t> aSourceNames, std::span<const uint32_t> aPairs)
{
    if (!Wanted())
        return 0;
    g_counters.mapCalls.fetch_add(1, std::memory_order_relaxed);
    UTPP_MUT_IF (kMutation != 4)
    {
        Control c;
        if (!g_control.Read(c) || !MainMappingJob(aSourceNames, aPairs, aCount, c.frozen))
        {
            g_counters.mapRejects.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }
    }
    return FkSide(aBones, aCount, aRig, aNow, aPerspectiveAllows);
}

inline CounterSnap SnapCounters()
{
    const Counters& k = g_counters;
    CounterSnap s{};
    s.captures = k.captures.load(); s.exitsHanded = k.exitsHanded.load(); s.exitsStale = k.exitsStale.load();
    s.pops = k.pops.load(); s.hipsPops = k.hipsPops.load(); s.popWrites = k.popWrites.load(); s.popCalls = k.popCalls.load();
    s.popSkipRig = k.popSkipRig.load(); s.popSkipEpoch = k.popSkipEpoch.load(); s.popSkipBusy = k.popSkipBusy.load();
    s.fkSideDenied = k.fkSideDenied.load(); s.mapCalls = k.mapCalls.load(); s.mapRejects = k.mapRejects.load();
    s.exitBreaks = k.exitBreaks.load(); s.exitHoldFrames = k.exitHoldFrames.load(); s.exitHoldPins = k.exitHoldPins.load();
    s.exitHoldEnds = k.exitHoldEnds.load(); s.exitHoldReleases = k.exitHoldReleases.load();
    return s;
}
} // namespace utpp::mm::vehtrans

// port v2 integration, native side. See MotionDriver.hpp.

#include "MotionDriver.hpp"
#include "CommonUtil.hpp"
#include "mm/AuthoredRecoil.hpp"
#include "mm/MeleeBodyId.hpp"
#include "mm/MotionRootFrame.hpp"
#include "mm/HeadLead.hpp"
#include "mm/HipFirePose.hpp"
#include "mm/LeanRuntime.hpp"
#include "mm/LocomotionSignalBlend.hpp"
#include "mm/MagTap.hpp"
#include "mm/NativeReloadContactsRuntime.hpp"
#include "mm/OrientWarp.hpp"
#include "mm/RecoilProfiles.hpp"
#include "mm/ReloadContactFade.hpp"
#include "mm/RootMontage.hpp"
#include "mm/SlideDeRoll.hpp"
#include "mm/Steering.hpp"
#include "input/LocomotionCore.hpp" // Reason bits only (pure logic): H2 safeguard cause of a native body turn that is off
#include "ragdoll/GetUpProbe.hpp"   // ragdoll R0 probe (pure logic: clamp, phases, watchdog, pose composition)
#include "ragdoll/GetUpClips.hpp"   // frame 0 of the motion get-up clips (the v2 databases do not carry them)
#include "ragdoll/GetUpPlayer.hpp"  // native get-up (pure logic: classifier, alignment, blend, phases, loader checks)
#include "ragdoll/RdLastPose.hpp"   // last ragdoll pose of V (written by PoseHook on the animation worker)

#include <algorithm>
#include <array>
#include <atomic> // UTPP-MOTION SEAM trav: OwnsTraversal
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "mm/Chooser.hpp"
#include "mm/ExitPoseHistory.hpp"
#include "mm/GamePoseCapture.hpp"
#include "mm/MotionDatabase.hpp"
#include "mm/ChooserMatcher.hpp"
#include "mm/NativeShootTransfer.hpp"
#include "mm/Overlay.hpp"
#include "mm/OverlayFile.hpp"
#include "mm/MotionPose.hpp"
#include "mm/TraversalFile.hpp"
#include "mm/MotionTraversal.hpp" // UTPP-MOTION SEAM trav
#include "mm/RefaceStart.hpp"

#include <optional>
#include <unordered_map>

#include <windows.h> // after the mm headers (WIN32_LEAN_AND_MEAN / NOMINMAX from the build)
#include <bcrypt.h>
#pragma comment(lib, "bcrypt") // SHA-256 of the database on the worker

namespace hlead = utpp::mm::headlead;
namespace owarp = utpp::mm::orientwarp;
namespace lrt = utpp::mm::leanrt;

namespace utpp::motiondriver
{
namespace
{
using namespace utpp::mm;
using adapter::Body;
using adapter::Fallback;

// Everything one body needs; built and initialised on the worker, installed under g_m, destroyed on the worker.
struct Bundle
{
    Body body = Body::Unknown;
    // Authored recoil per profile id (mm/RecoilProfiles.hpp): [id][0] hip shot, [id][1] aimed shot. recoilOk[id]: both files loaded.
    std::array<std::array<authored_recoil::Clip, 2>, recoil_profile::kSlots> recoil;
    std::array<bool, recoil_profile::kSlots> recoilOk{};
    motion::MotionDatabase db;
    chooser::Table table;
    ov::OverlayFile ov;
    ov::StyleFamilyData style;
    traversal::TraversalFile trav;
    // UTPP-MOTION SEAM trav: decision table + the playable montages of trav (moved out of it): sequence = a clip of db
    // (travPlayClip = db clip), else the montage's .utpptrav version 2 poses (travPlayClip = -1; travPoseMap = pose bone of
    // every db bone, travRaw = per-frame scratch of the sampled pose)
    traversal::DecisionTable travTable;
    traversal::TraversalFile travPlay;
    std::vector<int32_t> travPlayClip;
    std::vector<int32_t> travPoseMap;
    std::vector<BoneTransform> travRaw;
    uint32_t travFromPoses = 0;
    bool travFileOk = false, travTableOk = false, travPosesOk = false;
    // native get-up (ragdoll/GetUpPlayer.hpp): utpp_getup_<body>.utpptrav (version 2 poses, 4 montages GetUp_B/F/L/R); getupOk false = missing or
    // invalid (getupErr "missing" / "bad:<why>"): the vanilla recovery stays. getup keeps the parsed file (pose arrays), guClips the validated data.
    traversal::TraversalFile getup;
    utpp::ragdoll::GuClips guClips;
    bool getupOk = false;
    std::string getupErr = "missing";
    motion::ChooserMatcher matcher;
    motionpose::PoseEvaluator eval;
    locosignal::Blend locomotion;
    rootframe::Processor root;
    rootframe::Proposal rootFrame;
    bool rootPolicyEnabled = false, rootFrameValid = false;
    uint64_t rootHostSequence = 0;
    float lastVelocityWorld[2] = {0, 0};
    adapter::QueryTimes qtimes;
    adapter::MapState map;
    refacestart::Episode startTurn;
    std::vector<uint64_t> dbHashes; // integ::CNameHash of the database bones (BuildBoneMap)
    std::vector<int32_t> dbParents;
    std::vector<BoneTransform> pose;
    adapter::ResolveInputs res;
    bool styleOk = false;
    bool travOk = false;
    bool ovOk = false;
    bool evalOk = false;
    adapter::SlideSpec slide; // chooser.json "slide" section (declared false = the bundle has no slide data: mm.slide does nothing)
    // optional motion lean pack (CET mm.lean): loaded on the worker like trav; leanOk false = lean unavailable for this bundle
    mm::lean::LeanPack leanPack;
    lrt::LeanRuntime lean; // runtime state follows the bundle (fresh on every install)
    std::vector<int32_t> leanClipMap; // database clip -> pack Disable_AdditiveLeans curve table (-1 = none, by name + frame count)
    bool leanOk = false;
    int leanMapped = 0, leanMismatch = 0;
    std::string leanErr; // "" = absent file / ok
    std::string err; // first load error (status)
    std::string sha; // hex SHA-256 of the database file, computed on the worker (status sha=; "" = not read / failed)
    uint64_t dbBytes = 0; // UTPP-MOTION SEAM motionfix (item 4): database file size (load done line)
};

// Foot contacts and root rates belong to the same timeline as the sampled pose.
// Sample them once per Step; FrameLocked must not bypass the timeline handover.
locosignal::Sample SampleLocomotion(const Bundle& b, const motion::MatchResult& r)
{
    locosignal::Sample out;
    if (r.clip < 0 || static_cast<uint32_t>(r.clip) >= b.db.Header().numClips)
        return out;
    const auto& cl = b.db.Clip(static_cast<uint32_t>(r.clip));
    if (cl.numFrames == 0) return out;
    const float fps = b.db.Header().fps > 0.0f ? b.db.Header().fps : 30.0f;
    const float local = std::isfinite(r.clipTime) ? std::fmax(0.0f, r.clipTime * fps) : 0.0f;
    const uint32_t f0 = cl.firstFrame + std::min(static_cast<uint32_t>(local), cl.numFrames - 1);
    const uint32_t f1 = f0 + 1 < cl.firstFrame + cl.numFrames ? f0 + 1 : f0;
    const float u = std::clamp(local - std::floor(local), 0.0f, 1.0f);
    const auto& c0 = b.db.Contact(f0);
    const auto& c1 = b.db.Contact(f1);
    out.contact[0] = c0.l + (c1.l - c0.l) * u;
    out.contact[1] = c0.r + (c1.r - c0.r) * u;
    const auto& a = b.db.Root(f0);
    const auto& z = b.db.Root(f1);
    // ROOT stores deltas per animation frame. Publish rates per real second;
    // contacts retain their sampled amplitude and consumers still integrate real dt.
    const float rate = std::isfinite(r.playRate) && r.playRate > 0.0f ? r.playRate : 1.0f;
    out.rootVel[0] = (a.dx + (z.dx - a.dx) * u) * fps * rate;
    out.rootVel[1] = (a.dy + (z.dy - a.dy) * u) * fps * rate;
    out.rootYawRate = (a.dyaw + (z.dyaw - a.dyaw) * u) * fps * rate;
    return locosignal::Blend::Valid(out) ? out : locosignal::Sample{};
}

std::mutex g_m;
// Only blocking lifecycle callers take this mutex, always before g_m.
// Step/WorkerMain never take it: joining must leave g_m available to the worker.
std::mutex g_joinM;
bool g_stopping = false; // protected by g_m; no new lazy load during Shutdown
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
std::mutex g_testWaitM;
std::condition_variable g_testWaitCv;
bool g_testWaitPause = false, g_testWaitMoved = false, g_testWaitResume = false;
bool g_testWorkerPause = false, g_testWorkerEntered = false, g_testWorkerResume = false;
bool g_testWorkerTimedOut = false;
uint32_t g_testShutdownEntries = 0;
#endif
bool g_enabled = false;
Body g_body = Body::Unknown;
std::string g_dataDir;
Paths g_override[2];
bool g_haveOverride[2] = {false, false};
int32_t g_pose = 0, g_aim = 0, g_resolvedAimPose = 0;
ov::HipFirePoseHold g_hipFirePoseHold;
ov::JumpOutExitHold g_jumpOutExitHold; // 2026-10-28: the jump-out exit window stops holding the hip-fire pose (r5 air: in the flight, not at the landing)
ov::AirLegsFade g_airLegsFade; // fix2/hip P6 K1: weight of the real jump / fall for the weapon pose's leg layer (ov::FadeLowerBodyInAir)
ov::JumpOutArmsFade g_jumpOutArmsFade; // r9: weight of a vehicle jump-out flight for the weapon pose's arm / hand layers (ov::FadeArmsInJumpOutFlight)
bool g_jumpOutWindowNow = false; // the vehicle jump-out exit window of the last v2 Step (r9 arms fade input)
mm::shotturn::Hold g_stationaryShotTurn;
// Who built the capsule-vs-visible-root offset: true while it was turned (or is held) by one of UTPP's own facing owners (hipfire lease /
// pose hold, aim align, shot turn). Only such an offset is turned back by the rest turn-in-place / its fallback; a foreign jump (the game
// turning V onto the look when a heading lock ends) or the follow-mode camera (the game turns V with the mouse) stays hidden, as on main.
bool g_restOffsetOwned = false;
float g_restOwnerTail = 0.0f; // seconds left of the owner grace (a facing owner just dropped)
// A capsule turn faster than this without a facing owner is foreign (UTPP's own lease turns at <= 1080 deg/s but only under an owner;
// look-around in follow mode and the unlock jump are not).
constexpr float kRestForeignTurnDegPerS = 90.0f;
// The lease's last capsule step can land in the feed in which the pose hold has just run out (recording 12:15, 92.152 s: 341 deg/s, the
// offset of the 94.985 s reload). An owner counts for this long after it dropped. Without it 20 of the 21 recorded offsets are owned.
constexpr float kRestOwnerGraceS = 0.25f;
bool g_hipFirePoseNow = false;
bool g_reloadContactsEligible = false;
bool g_authoredReload = false;
reload_contacts_runtime::CorrectionFade g_reloadContactFade;
reload_contacts_runtime::CorrectionEpisode g_reloadContactEpisode;
// UTPP-MOTION SEAM motionfix item 2: aim target (model space) + mm.aimGameUpper; g_aimingNow = wantsToAim of the last
// v2 frame (ApplyLocked: hand the upper body back while aiming when aimGameUpper is on)
bool g_aimTgtOn = false, g_aimTgtValid = false;
float g_aimTgt[3] = {0, 0, 0};
bool g_cameraTargetValid = false;
float g_cameraTarget[3] = {0, 0, 0};
double g_cameraTargetTime = 0;
// mm.aimGameUpper default: chosen by the item 2 measurement on the real 2026-10-13 dump (unit test 12c, progress file):
// the open-loop v2 AO leaves the WeaponRight barrel ~9 deg off the aim target (mean 9.3, pitch 0) against v1 game pose +
// aimofs errAfter median 1.9 deg in the same log -> the game upper body + aimofs while aiming is the default.
constexpr bool kAimGameUpperDefault = true;
bool g_aimGameUpper = kAimGameUpperDefault;
bool g_aimingNow = false;
// Opt-in head lead (CET mm.headLead -> SetHeadLead, default OFF): mm/HeadLead.hpp turns Neck / Neck1 / Head into the turn
// (motion BS_Relaxed_Lean_Head equivalent). Step per v2 feed (heading rate, gates), Apply on the published v2 pose
// (ApplyLocked) before the aim offset / finish stages. v1-owned frames are not covered. OFF: no code path touches the pose.
bool g_headLeadOn = false;
hlead::Params g_hlParams;
hlead::HeadLead g_hl;
hlead::Skeleton g_hlSk;
const void* g_hlSkBundle = nullptr;
std::chrono::steady_clock::time_point g_hlLastApply{};
bool g_hlHaveApply = false;
// Opt-in orientation warp (CET mm.orientWarp -> SetOrientWarp, default OFF): mm/OrientWarp.hpp aligns the pelvis (extra visual
// root yaw added to the provided root frame of the leg tail) and the torso (twist over Spine .. Spine3 of the tail's pose copy) with
// the path of motion while V runs a circle. Step + Measure per v2 feed (StepLocked), applied in FrameLocked. OFF and idle (weight 0,
// nothing applied): no code path touches the frame, the pose and the root yaw are bit-identical.
bool g_orientWarpOn = false;
owarp::Params g_owParams;
owarp::OrientWarp g_ow;
owarp::Skeleton g_owSk;
const void* g_owSkBundle = nullptr;
// Run pelvis yaw trim (CET mm.pelvisYawTrim -> SetPelvisYawTrim, deg, default 0 = nothing): mm/OrientWarp.hpp PelvisTrim. Step per
// v2 feed (gates, forward-locomotion weight, feed dt), applied in FrameLocked on the leg-tail copy together with the orientation warp
// (root +trim, torso -trim: only the pelvis turns). 0: no code path touches the frame, the pose and the root yaw are bit-identical.
float g_pelvisTrimDeg = 0.0f;
// Feminine shoulder narrowing in metres per side (CET mm.shoulderWidth -> cm per side -> SetFeminineShoulderCm); default = MotionPose.hpp constant.
float g_femShoulderM = motionpose::kFeminineShoulderAmount;
owarp::TrimParams g_ptParams;
owarp::PelvisTrim g_pt;
owarp::Skeleton g_ptSk;
const void* g_ptSkBundle = nullptr;
const char* g_ptWhy = "off"; // trace pt=<deg>/<weight>/<reason>: first gate that holds the weight down, else the motion reason
// Body yaw offset (CET mm.bodyYawOffset -> SetBodyYawOffset, deg, default 0 = nothing): mm/OrientWarp.hpp BodyYawOffset. Step per
// v2 feed (gates, feed dt), applied in FrameLocked as an addition to the visual root yaw of the leg-tail frame only (the whole pose
// turns as one rigid unit; no pose edit). 0: no code path touches the frame, the root yaw is bit-identical.
float g_bodyOffsetDeg = 0.0f;
owarp::BodyOffsetParams g_boParams;
owarp::BodyYawOffset g_bo;
const char* g_boWhy = "off"; // trace bo=<deg>/<weight>/<reason>: first gate that holds the weight down, else "ok"
// Aim pitch plan 2026-10-30 seam C: closed barrel pitch loop of the v2 aim offset (MotionPose OverlayFrameInputs::barrelLoop; CET
// mm.aimAtCrosshair -> SetBarrelLoop, DLL default OFF = the open-loop aim offset bit for bit). Barrel axis of WeaponRight (the same id
// as aimofs, UTPPSetAimOffsetAxis) and the muzzle length along it (UTPPSetMotionBarrelMuzzle). The aim point P reaches the evaluator in
// the pose (visual root) frame: FrameInputsLocked = the actor point (no root frame), PrepareRootOverlay = InversePoint of the same
// actor point with the frame ComputeAim uses for the aim angles (one point, one frame for the sweep angles and the loop).
bool g_barrelLoop = false;
int32_t g_barrelAxis = aim::kDefaultBarrelAxis;
float g_barrelMuzzle = aim::Params{}.muzzleOffset;
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
BarrelProbe g_barrelProbe; // the last v2 evaluation (TestBarrelProbe)
RootProbe g_rootProbe;     // the last root frame step (TestRootProbe)
#endif
// Opt-in motion lean (CET mm.lean -> SetLean, default OFF): mm/LeanRuntime.hpp leans the v2 pose into the turn from the lateral
// acceleration of the feed velocity (Step + Apply per v2 feed on b.pose in StepLocked, before the orientation warp and the leg tail).
// The state lives in the Bundle. OFF and idle (weight 0): no code path touches the pose, it is bit-identical.
bool g_leanSetting = false;
bool g_leanHeadSetting = false; // mm.leanHead: the motion head layer replaces the head lead while the lean is on
lrt::Params g_leanParams;
bool g_leanHeadOwns = false;    // last StepLocked: the head lead is blocked in favour of the lean head layer
// Slide feed (CET mm.slide -> SetSlide, default OFF; mm/MotionAdapter.hpp "slide feed"): PSM LocomotionDetailed 5 reaches the
// chooser as MovementMode Sliding (+ Stance Crouch, MovementDirection, MovementMode_LastFrame) and the driver lets v2 own the body
// instead of the hold-last-pose path: no foot contacts on slide frames, no body-yaw command (MMDriver Feed asks OwnsSlide). Only when
// the setting is on AND the active bundle declares the slide section; otherwise every code path is the pre-slide one.
bool g_slideSetting = false;
// Shooter mode (CET bodyturn.shooterMode -> SetShooterMode, default OFF) and the H2 safeguard (SetH2Guard, default ON; mm/MotionAdapter.hpp MapConfig::
// shooterMode / h2Strafe). g_shooter_modeLive = the setting AND v2 can run (enabled, no fallback, bundle): the lock-free copy ShooterModeSpeedMul reads.
bool g_shooter_modeSetting = false;
std::atomic<bool> g_shooter_modeLive{false};
bool g_h2GuardSetting = true;
uint64_t g_shooterFrames = 0, g_h2Frames = 0; // v2 Steps with MapState::shooterModeActive / h2Active
bool g_lastBtSwitch = true;              // FeedInput::nativeBodyTurnSwitch of the last Step
uint32_t (*g_locomotionProbe)() = nullptr;  // Main.cpp: Locomotion blocker bits (read only); nullptr offline
bool g_hipLease = false;                 // UTPPSetMotionHipLease: the CET hip-fire lease is running (an H2 exclusion), under g_m
// ShooterModeSpeedMul answers only while v2 really drives third-person locomotion: g_fnDrivingNs = steady-clock ns of the last Step that reached the
// chooser mapping in TPP (0 = none, cleared by every Skip and by a non-TPP Step); g_fnLastGait = the gait that Step's chooser played
// (0 walk / 1 run / 2 sprint; the walk toggle makes it walk), so the speed ratio matches the strafe clips that really play.
std::atomic<int64_t> g_fnDrivingNs{0};
std::atomic<int> g_fnLastGait{-1};
// Walk toggle speed (SetWalkSpeed, CET mm.walkSpeed; MotionAdapter.hpp MapConfig::gameWalkTogglePoint) and the Tier2 Slow feed flag
// (FeedTier2Slow). g_walkSpeedLive = target > 0 AND v2 can run (enabled, no fallback, bundle): the lock-free gate WalkSpeedTarget reads,
// refreshed with the Shooter live flag.
std::atomic<float> g_walkSpeed{0.0f};
std::atomic<bool> g_walkSpeedLive{false};
// Minimum analog walk speed (SetMinAnalogWalk, CET mm.minWalkSpeed; MotionAdapter.hpp AnalogWalkFloorMul): g_minAnalogWalkLive = setting > 0
// AND v2 can run, refreshed with the walk speed flag (AnalogFloorMul gate).
std::atomic<float> g_minAnalogWalk{0.0f};
std::atomic<bool> g_minAnalogWalkLive{false};
std::atomic<bool> g_tier2Slow{false};
// Selection churn guards as a group (SetSelectionGuards, CET mm.selectionGuards): read by every Step
std::atomic<bool> g_selectionGuards{true};
// Dual-wield D0: last raw psmBlock of UTPPFeedLocomotion (FeedAirFromInput); status string (dualWield= upperBodyRaw=) only.
std::atomic<uint32_t> g_feedPsmRaw{0};
std::string DualStatusSuffix()
{
    char buf[64];
    integ::PsmDualStatusSuffix(buf, sizeof buf, g_feedPsmRaw.load(std::memory_order_relaxed));
    return buf;
}
std::atomic<bool> g_aimJogOff{true}; // G9: the last UTPPMMAimJogTarget answer was 0 (vanilla aim walk speed; default before any query)
constexpr int64_t kShooterModeDrivingStaleNs = 250000000; // 0.25 s: the feed is fresh
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
std::atomic<int64_t> g_testSteadyNs{-1}; // test targets only (never the DLL): >= 0 replaces the steady clock below (simulated, load independent)
#endif
int64_t SteadyNs()
{
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    const int64_t simulated = g_testSteadyNs.load(std::memory_order_acquire);
    if (simulated >= 0)
        return simulated;
#endif
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// The idle heading hold across a root reseed: set by every Step in which V stands without input (Shooter): the committed visual world yaw.
bool g_fnHoldValid = false;
float g_fnHoldVisualDeg = 0.0f;
int64_t g_fnHoldNs = 0;
constexpr int64_t kShooterModeHoldMaxNs = 3000000000; // 3 s
// motion: rotation rate in the air 200 deg/s (CBP UpdateRotation_PreCMC): the Shooter root follows the actor in the air at most this fast.
constexpr float kShooterModeAirRate = 200.0f * 3.14159265358979323846f / 180.0f;
// A capsule turn above this at idle (no stick) in Shooter mode is the camera orbit (follow mode), never UTPP's own facing turn.
constexpr float kShooterModeOrbitDegPerS = 3.0f;
// Rate bound of the visual-root lock while V moves in Shooter mode / H2 (MotionOffsetRoot Step::lockMaxRate): 450 deg/s = 7.5 deg per
// 60 Hz feed, under the 8 deg per feed that reads as a snap; 540 (ADS while moving) is kept for the aim lock.
#if UTPP_SHOOTER_MODE_MUTATION == 10
constexpr float kShooterModeLockRate = 1.0e6f;
#else
constexpr float kShooterModeLockRate = 450.0f * 3.14159265358979323846f / 180.0f;
#endif
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
StrafeProbe g_strafeProbe;
#endif
std::atomic<bool> g_slideHandling{false}; // lock-free copy for OwnsSlide (read by MMDriver Feed under the v1 mutex)
bool g_slideFrame = false;                // the frame of the last Step is a slide frame (FrameLocked)
uint64_t g_slideFrames = 0;               // slide frames stepped (status)
bool g_aimUpperFade = true, g_aimUpperTail = false;
bool g_weaponUpperOn = true, g_weaponUpperFrame = false, g_weaponUpperTail = false;
int32_t g_weaponAction = 0;
bool g_shootTransferEnabled = false; // explicit opt-in; never changes the installed default
integ::GamePoseSample g_shootLastReady{}, g_shootBase{};
bool g_shootLastReadyValid = false, g_shootBaseValid = false;
int64_t g_shootEventTicks = 0;
native_shoot::DeltaHold g_shootHold;
native_shoot::EpisodeGate g_shootEpisode;
uint64_t g_shootAppliedEvaluations = 0;
bool g_shootApplied = false;
// P9: this frame's weapon has a recoil route (authored recoil domain or native FK transfer). Only then can the native first-burst
// fallback of the hipfire shot be replaced by a transfer; every other weapon keeps the authored Aim arms through the shot.
bool g_shootRouteNow = false;
// P9 review C: the route is decided once per shot. g_shootRouteLive is this frame's gate (authoredDomain || shootEligible; shootEligible
// follows movementState without hysteresis, one micro-step flickered it 1 -> 0 -> 1 inside a single act=8); g_shootRouteHeld is the value
// kept for the rest of the shot: set on the act=8 edge, afterwards it can only fall (a route that was lost stays lost until act=8 ends), so
// the arms never owe more than one extra FPP <-> authored switch per shot, and a route that was absent on the edge never appears mid-shot.
bool g_shootRouteLive = false, g_shootRouteHeld = false, g_shootRouteInShot = false;
authored_recoil::Player g_authoredRecoil;
uint32_t g_recoilWeaponEpoch = 0;
uint64_t g_recoilSerial = 0;
bool g_recoilSupported = false, g_recoilPending = false, g_recoilAim = false; // g_recoilSupported: the equipped weapon has a profile id
int32_t g_recoilProfile = recoil_profile::kNone; // profile id of the equipped weapon (mm/RecoilProfiles.hpp), kNone = no authored recoil
bool g_recoilOwns = false, g_recoilAdmitted = false;
int g_recoilClip = 0;
double g_recoilShotTime = 0, g_recoilAdmitTime = 0;
int g_recoilPendingSteps = 0; // evaluated Steps since the pending shot pulse (hitch tolerance, see FrameInputsLocked)
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
double g_testRecoilClock = -1.0; // test targets only (never the DLL): >= 0 replaces the QPC clock below (simulated, load independent)
#endif
double RecoilClock()
{
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    if (g_testRecoilClock >= 0.0) return g_testRecoilClock;
#endif
    LARGE_INTEGER now{}, hz{};
    QueryPerformanceCounter(&now); QueryPerformanceFrequency(&hz);
    return hz.QuadPart > 0 ? double(now.QuadPart) / double(hz.QuadPart) : 0;
}
// Leaving the authored recoil domain (reload start, stance / movement mode change, traversal) fades a running kick out over
// kRecoilExitSeconds instead of deleting it in one frame (a 13-25 cm / up to 37 degree jump with the larger profiles). The hard
// resets (weapon change, release, TPP off, invalid inputs, unready profile) stay immediate.
constexpr float kRecoilExitSeconds = 0.12f; // = the upper-body entry time of a reload (ReloadContactFade UpperEnvelope)
float g_recoilExitW = 1.0f;                // 1 = in the domain, falls to 0 while it is left
double g_recoilExitT = 0.0;                // clock of the last update of g_recoilExitW
float g_recoilEffT = 0.0f, g_recoilEffR = 0.0f; // translation (m) / rotation (rad) of the recoil the arms were given on the last frame
void ResetAuthoredRecoil()
{
    g_authoredRecoil.Reset(); g_recoilPending = g_recoilOwns = g_recoilAdmitted = false; g_recoilPendingSteps = 0;
    g_recoilExitW = 1.0f; g_recoilExitT = 0.0; g_recoilEffT = g_recoilEffR = 0.0f;
}
float g_shootReachableScale = 0.0f;
float g_exitAirTail = 0.0f; // s left of the post-exit window in which a non-rising air feed counts as ground (MotionAdapter exitAirGraceS)
float g_reloadUpperWeight = 0.0f;
rootframe::ReloadRootLatch g_reloadRootLatch; // whether the reload began in motion (self-resetting at weight 0), see RootMontage.hpp
reload_contacts_runtime::UpperEnvelope g_reloadUpperEnv; // weight mirrors g_reloadUpperWeight; latches whether the reload interrupted ADS
reload_contacts_runtime::SlideRollHold g_reloadRollHold; // MM arms keep ownership while the slide rolls the torso (see ReloadContactFade.hpp)
reload_contacts_runtime::SlideDeRoll g_slideDeRoll; // weight of the MM torso counter-roll of a pistol / rifle reload on a slide frame (SlideDeRoll.hpp)
reload_contacts_runtime::RifleSlideExitLatch g_rifleSlideLatch; // rifle de-roll kept through the still-rolled slide exit of the same reload / carry
reload_contacts_runtime::SlideCarryAim g_slideCarryAim; // (1 - AimingWeight) scale of the rifle carry de-roll (ReloadContactFade.hpp)
uint64_t g_slideCarryFrames = 0; // Steps whose torso uprighting gate was opened by the rifle carry path (status slideCarry=)
float g_slideCarryScale = 1.0f;  // aim scale of the last Step that read the de-roll (status)
slide_deroll::Bones g_derollBones; // Spine1..3 / clavicle roots (ov indices) of g_derollBundle
const void* g_derollBundle = nullptr;
float g_derollAppliedDeg = 0.0f; // roll removed from the published MM pose this Step (status)
float g_depitchAppliedDeg = 0.0f; // Spine3->Neck lean removed from it (status)
bool g_rollHoldStepped = false; // the roll hold was advanced by the main path of this Step
bool g_reloadNowStep = false; // `reload` of the last Step: PSM reload action with a weapon overlay, not released (pistols: envelope / roll hold)
bool g_slideTorsoReloadStep = false; // the reload of the last Step gets the slide torso uprighting (pistol or rifle, SlideTorsoReload)
bool g_reloadRifleStep = false;      // ... and it is a rifle reload (RifleSlideExitLatch)
// Magazine top-up gesture (mm/MagTap.hpp): a refill without a reload state reported by the glue (FeedMagTap) is a pending request
// for the next main-path Step (MagTapStepLocked: admitted only while MM owns the arms in TPP, else dropped); it never outlives
// kMagTapPendingSteps Steps / kMagTapPendingSeconds, so a refill seen while gated (FPP, vehicle, menu) never plays later.
constexpr int kMagTapPendingSteps = 2;
constexpr double kMagTapPendingSeconds = 0.25;
magtap::Envelope g_magTap;
magtap::Kind g_magTapKind = magtap::Kind::Pistol;
bool g_magTapPending = false;
int g_magTapPendingAge = 0;     // Steps since the request
double g_magTapPendingTime = 0; // RecoilClock() of the request
// Rifle request on a slide frame waiting for the carry de-roll (magtap::SlideDefer): no Step-count ageing while it waits (owned), the
// stale window of the request grows by magtap::kSlideDelaySeconds; an ownership loss during the wait ages / drops it as before.
bool g_magTapDeferred = false;
float g_magTapWaited = 0.0f;     // feed time the pending request has waited (magtap::kSlideDelaySeconds bound)
float g_magTapLastWait = 0.0f;   // feed time waited by the last started gesture (status magTapWait=)
uint64_t g_magTapDeferredReqs = 0; // requests that waited at least one Step
bool g_magTapStepped = false;   // the gesture was advanced by the main path of this Step
uint64_t g_magTapFed = 0, g_magTapStarted = 0, g_magTapDropped = 0, g_magTapFrames = 0; // requests / gestures / dropped / edited frames
float g_magTapPeak = 0.0f;      // highest weight since the last MOTION trace line (a 0.45 s gesture can fall between two lines)
float g_magTapMoved = 0.0f, g_magTapErr = 0.0f, g_magTapMovedMax = 0.0f; // last frame: hand displacement / full-weight target error (m)
float g_magTapTravel = -1.0f;   // hand -> magazine distance (m) at the last owned request (> kMaxTravel = dropped by distance)
uint64_t g_magTapUnowned = 0;   // requests dropped because MM did not own the arms (FPP, vehicle, ADS / low-ready hand-back, ...)
// CET mm.shinobiBonusReload (SetMagTapEnabled, native UTPPSetMotionMagTap), default OFF: while off every request is refused at once.
// Process lifetime: never reset by SetEnabled / Load / ResetMagTap / Shutdown (CET sends it once and caches what it sent).
bool g_magTapEnabled = false;
uint64_t g_magTapOff = 0;       // requests refused because the setting is off
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
MagTapProbe g_magTapProbe;      // unit test target only: the same-frame pre / post pose of the last main-path Step
#endif
void ResetMagTap()
{
    g_magTap.Reset();
    g_magTapPending = false;
    g_magTapPendingAge = 0;
    g_magTapDeferred = false;
    g_magTapWaited = 0.0f;
    g_magTapPeak = g_magTapMoved = g_magTapErr = 0.0f;
}
// Stale window of the pending request (RecoilClock seconds): a request deferred for the slide de-roll gets kSlideDelaySeconds more.
double MagTapPendingWindow()
{
    return g_magTapDeferred ? kMagTapPendingSeconds + static_cast<double>(magtap::kSlideDelaySeconds) : kMagTapPendingSeconds;
}
GripProbe g_gripProbe;
bool g_gripEnabled = true;
void RecordGripInfoLocked(const motionpose::FrameInfo& aInfo)
{
    g_shootApplied = aInfo.shootApplied;
    if (g_recoilOwns && aInfo.shootApplied && !g_recoilAdmitted)
    { g_recoilAdmitted = true; g_recoilAdmitTime = RecoilClock(); }
    if (g_hipFirePoseNow && !g_aimingNow)
        // ACTION-only safety fallback: a motion idle-base opt-out must not erase enabled native recoil.
        // Native FPP arm deformation may remain during this temporary fallback, as with Reload.
        // P9: only for a weapon that has a transfer route (g_shootRouteNow); a rifle, SMG, shotgun or non-Nue pistol out of the
        // native FK stance has nothing to wait for, and the FPP arms would pop in for the whole shot (upperScale 1 -> 0 -> 1).
        g_weaponUpperFrame = ov::HipFireNativeActionOwner(g_weaponUpperOn, g_shootTransferEnabled, g_shootRouteNow,
            true, g_weaponAction, aInfo.shootApplied, aInfo.aoWeight,
            g_shootEpisode.pending || g_shootEpisode.admitted || g_recoilOwns);
    g_shootReachableScale = aInfo.shootScale;
    if (aInfo.shootApplied) ++g_shootAppliedEvaluations;
    g_gripProbe.applied = aInfo.grip.left.ok;
    g_gripProbe.clamped = aInfo.grip.left.clamped;
    g_gripProbe.error = aInfo.grip.left.handErr;
    g_gripProbe.requestedReach = aInfo.grip.left.reqDist;
    g_gripProbe.usedReach = aInfo.grip.left.useDist;
    g_gripProbe.weaponShift = aInfo.grip.weaponShift;
    if (aInfo.gripFromOverlay && aInfo.gripTarget.valid)
    {
        g_gripProbe.supplied = true;
        g_gripProbe.source = 2;
        ++g_gripProbe.authoredFrames;
        g_gripProbe.relativeTarget[0] = aInfo.gripTarget.t.x;
        g_gripProbe.relativeTarget[1] = aInfo.gripTarget.t.y;
        g_gripProbe.relativeTarget[2] = aInfo.gripTarget.t.z;
    }
}
uint64_t g_aoFrames = 0, g_aimHandback = 0;
float g_aoYaw = 0.0f, g_aoU = 0.0f, g_aoW = 0.0f;
bool g_release = false, g_injured = false;
// commit C prep (air): SetAirFeed one-shot (consumed by the next Step), not-probed dead reckoning. motionfix item 5:
// v2 owns air while mm.motionDriver is on (the runtime never claimed air frames: SetAirOwns / SetMotionOwnsAir and the 'air-owns' skip removed).
adapter::AirFeed g_airIn;
bool g_airFresh = false;
// last probe hit dead-reckoned with the feed vz (-1 = none) + J2 take-off plane seed (MotionDriver.hpp AirFloorStep). Mutations (air
// test builds only, AirMut): 6 no dead reckoning, 12 no take-off plane seed (pre-J2), 13 the seed is also used for a walk-off.
AirFloorState g_airFloor;
bool g_airEpisode = false;   // the previous v2 Step was InAir (episode integrals running)
bool g_airPrevLanded = false; // the previous v2 Step had justLandedLight / Heavy (landing blend deficit bypass edge)
AirDiag g_airDiag;
// mutation switch for the air test's mutation builds only (UTPP_MOTION_DRIVER_AIR_MUTATION=k; 0 / undefined = production code)
#ifndef UTPP_MOTION_DRIVER_AIR_MUTATION
#define UTPP_MOTION_DRIVER_AIR_MUTATION 0
#endif
int AirMut() { return UTPP_MOTION_DRIVER_AIR_MUTATION; } // a call, not a constant expression (no C4127 in production)
// UTPP-MOTION SEAM next review fix: traversal test mutations, compiled only into the mutation builds
// (UTPP_MOTION_DRIVER_TRAV_MUTATION=k, the local mutation build script); undefined / 0 = production: every mutation site is an
// #if block, so the shipped DLL carries no mutation branch, flag or function. 1 TravFrame ignores the plan root yaw,
// 2 hand-less plans warp the hands (pre-fix), 3 PlanTraversal uses GameTweaks{} instead of the fed tweaks, 4 deferral not
// gated on release / air / loading (pre-fix), 5 the montage starts one detail feed late, 6 OwnsTraversal kept on the first
// feed after the episode, 7 no handback turn of the last montage root yaw (pre-fix snap), 8 a handback turn cut by a skip
// resumes on the re-entry (stale turn), 9 the handback turn also starts on a re-entry feed (the handback feed itself skipped)
#ifndef UTPP_MOTION_DRIVER_TRAV_MUTATION
#define UTPP_MOTION_DRIVER_TRAV_MUTATION 0
#endif
// UTPP-MOTION SEAM motionfix: mutation builds only (UTPP_MOTION_DRIVER_FIX_MUTATION=k); undefined / 0 = production.
// 1 the Locomotion world target is not forwarded to MotionFeed (pre-fix: mv rotated by the camera yaw), 2 the MOTION trace ignores
// db / clip changes (TraceLocked), 3 each fallback reason logged once per session (pre-fix LogReasonOnceLocked), 4 no
// bundle load lines (pre-fix), 5 the trace / status carry no body / rig / bone map fields (pre-fix), 6 no rate bound on the
// fallback reason lines, 21 (fix2/hip) the v2 slide (chooser MovementMode 2) is outside the authored weapon recoil domain
// (pre-fix: every slide hipfire shot fell back to the native action pose), 22 (fix2/hip) the leg layer of the weapon pose is
// also faded in the slide (chooser mode != 0), not only in the real air, 25 (fix2/hip) the authored recoil shot pulse expires on wall
// time alone (0.1 s), so a late first Step after the shot drops the recoil, 23 (fix2/hip) a skip (frames handed to the game / v1) does
// not reset the air weight, 24 (fix2/hip) the air weight is reset on the first traversal montage frame (pop of the legs)
#ifndef UTPP_MOTION_DRIVER_FIX_MUTATION
#define UTPP_MOTION_DRIVER_FIX_MUTATION 0
#endif

std::unique_ptr<Bundle> g_bundle; // installed bundle (game thread uses it under g_m)
// Loader thread. Static destruction (DLL unload / process exit without Shutdown) must not std::terminate on a joinable
// thread (a finished load stays joinable until the next load or WaitLoader joins it): a leftover worker is detached, as
// MotionHost.cpp SwitchState / mm/FeedDump.hpp. Shutdown joins it normally.
struct WorkerThread
{
    std::thread t;
    ~WorkerThread()
    {
        if (t.joinable())
            t.detach();
    }
};
WorkerThread g_worker;
bool g_loading = false;
bool g_reload = false; // data dir / path override changed, or re-enabled after a failed load: next Step loads again
Body g_loadingBody = Body::Unknown;
uint64_t g_loads = 0;
Fallback g_lastLoadFail = Fallback::None; // load result of a body whose bundle could not be built (no bundle installed)
Body g_failBody = Body::Unknown;
std::string g_failErr;

// rig (OnArmed)
uint64_t g_rig = 0;
std::vector<uint64_t> g_gameHashes;
std::vector<int16_t> g_gameParents;
integ::BoneMapResult g_map{};
uint64_t g_vehExitSeeds[2] = {}; // first search after a vehicle exit: [0] capture not usable (mean query), [1] seeded from the capture pose
uint32_t g_topology = ~0u;

// frame
Fallback g_reason = Fallback::NotRequested;
bool g_faulted = false, g_budgetTripped = false;
// Slide handling is live when the setting is on, v2 can run (enabled, no fallback) and the installed bundle declares the slide
// section. Refreshed wherever one of those changes (SetSlide, SetEnabled, every Step, FaultLocked); MMDriver reads it lock-free.
void RefreshSlideLocked()
{
    g_slideHandling.store(g_slideSetting && g_enabled && g_reason == Fallback::None && g_bundle && g_bundle->slide.declared,
                          std::memory_order_release);
}
void RefreshShooterModeLocked()
{
    g_shooter_modeLive.store(g_shooter_modeSetting && g_enabled && g_reason == Fallback::None && g_bundle != nullptr, std::memory_order_release);
    g_walkSpeedLive.store(g_walkSpeed.load(std::memory_order_relaxed) > 0.0f && g_enabled && g_reason == Fallback::None &&
                              g_bundle != nullptr, std::memory_order_release);
    g_minAnalogWalkLive.store(g_minAnalogWalk.load(std::memory_order_relaxed) > 0.0f && g_enabled && g_reason == Fallback::None &&
                                  g_bundle != nullptr, std::memory_order_release);
}
// Vehicle batch (TRAV-1, MotionDriver.hpp TravActive): lock-free copy for the v1 gate. Refreshed next to the slide / Shooter
// flags (SetEnabled, every Step, FaultLocked, bundle install, SetSlide) and cleared by Shutdown.
std::atomic<bool> g_travActive{false};
void RefreshTravActiveLocked()
{
    g_travActive.store(g_enabled && !g_faulted && !g_budgetTripped && g_bundle != nullptr && g_bundle->travOk,
                       std::memory_order_release);
}
const char* g_faultWhy = nullptr; // first fault cause (static string), status only
void (*g_log)(const char*) = nullptr; // host logger (Main.cpp MMTraceLog); one line per change
// UTPP-MOTION SEAM motionfix (item 4): MOTION trace state (TraceLocked, SetTrace)
inline constexpr float kTracePeriod = 0.5f;
bool g_trace = false;
float g_traceClock = 0.0f, g_traceSince = 1.0e9f;
uint64_t g_traceLines = 0;
char g_traceBuf[1280] = {}; // motionfix: was 768 (body / rig / map fields)
struct TraceKey
{
    int32_t db = -2, clip = -2, rot = -1, poseKey = -1, mode = -1, weaponAction = -1;
    bool starting = false, pivoting = false, trav = false;
    int32_t body = -1;  // body / rig in the line, a change writes one
    uint64_t rig = 0;
    bool operator==(const TraceKey&) const = default;
};
TraceKey g_traceKey;
uint32_t g_loggedReasons = 0;         // bit per Fallback value already logged this session (mutation 3 only)
// UTPP-MOTION SEAM motionfix (item 4, male V test 2026-10-13: the man bundle load left no line, LogReasonOnceLocked hid every
// later reason): a "UTPP mm.motionDriver fallback=" line on every change of the Step's reason (incl. back to none = v2 path), rate
// bounded [P] (kReasonLinesPerWindow per kReasonWindow s of feed time; the excess is counted, the reason still differing
// after the window is written then); bundle load lines: "load start" when the worker starts (game thread, MaybeLoadLocked),
// "load done" written by the next Step on the game thread from g_loadDone (filled by the worker under g_m, fixed buffers).
inline constexpr float kReasonWindow = 1.0f;
inline constexpr uint32_t kReasonLinesPerWindow = 4;
bool g_haveLoggedReason = false;
Fallback g_loggedReason = Fallback::None;
float g_reasonClock = 0.0f, g_reasonWindowStart = -1.0e9f;
uint32_t g_reasonInWindow = 0, g_reasonSuppressed = 0;
uint64_t g_reasonLines = 0, g_loadLines = 0;
struct LoadDone
{
    bool pending = false;
    Body body = Body::Unknown;
    const char* outcome = "installed"; // installed / dropped (body changed while loading) / fault
    uint64_t load = 0;                 // g_loads after this load
    float ms = 0.0f;
    uint64_t dbBytes = 0, rigId = 0;
    uint32_t bones = 0, clips = 0, dbs = 0, frames = 0, travMontages = 0, travFromPoses = 0;
    bool dbOk = false, chooserOk = false, matcherOk = false, ovOk = false, styleOk = false, travOk = false;
    char sha[17] = {};
    char err[192] = {};
    char lean[112] = {}; // lean pack: "none" / "ok:<mapped>/<mismatch>" / "fail:<why>"
};
LoadDone g_loadDone;
char g_loadBuf[768] = {};
char g_reasonBuf[512] = {};
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
bool g_testThrowNextStep = false; // unit test only (TestThrowInNextStep)
// unit test only (TestKeepRetiredPoses): the pose buffer of a bundle replaced by the worker is filled with NaN and kept alive
// (moved out of the bundle, same address) instead of freed, so a stale pointer into it reads defined, detectable memory.
bool g_testKeepRetired = false;
std::vector<std::vector<BoneTransform>> g_testRetired;
void RetireForTestLocked(Bundle* aOld)
{
    if (!g_testKeepRetired || aOld == nullptr)
        return;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (BoneTransform& t : aOld->pose)
        t = BoneTransform{{nan, nan, nan}, {nan, nan, nan, nan}};
    g_testRetired.push_back(std::move(aOld->pose));
}
#endif
bool g_pending = false, g_wasRunning = false;
const char* g_lastSkip = "none"; // static strings only: no per-frame allocation
uint64_t g_steps = 0, g_frames = 0, g_applied = 0, g_skipGate = 0, g_skipPsm = 0, g_skipRelease = 0,
         g_evalFail = 0;
motion::MatchResult g_last{};
SelectionDiag g_selDiag{}; // SelectionDiagnostics: the chooser inputs and the match result of the last Step that ran the chooser
// MapContext of the offline replay (FeedReplayMotion.cpp RunFeed, the reference adapter path): root yaw = motion OffsetRoot rotation in
// RotationMode Accumulate (follows the playing frame's ROOT dyaw only, seeded with the actor yaw on (re)entry) and the database of
// the previous valid pick (motion CurrentDatabaseTags "Pivots").
float g_rootYawDeg = 0.0f;
int32_t g_prevDb = -1;
std::array<float, adapter::Budget::kWindow> g_times{};     // per feed: the smaller of the wall time and the thread's own time (trip decision)
std::array<float, adapter::Budget::kWindow> g_wallTimes{}; // per feed: wall time (status / log only)
std::array<float, adapter::Budget::kWindow> g_sorted{};
uint64_t g_timeCount = 0;
// Budget pause (FinishFrameLocked trips it, BudgetPauseStepLocked ends it on feed time): g_budgetTripped is the pause flag.
#ifndef UTPP_MOTION_DRIVER_BUDGET_MUTATION
#define UTPP_MOTION_DRIVER_BUDGET_MUTATION 0 // test builds only: 1 = a pause never ends, 2 = wall time only
#endif
float g_budgetPauseLeft = 0.0f;                             // feed time left in the current pause
float g_budgetPauseS = 0.0f;                                // length of the current / last pause
float g_budgetNextPauseS = adapter::Budget::kPauseFirstS;   // length of the next pause
uint64_t g_budgetTrips = 0;
float g_budgetTripP95 = 0.0f, g_budgetTripWallP95 = 0.0f;   // statistics of the window that tripped last
ULONG64 g_feedCycles0 = 0;                                  // thread cycles at the start of this feed
bool g_feedCyclesOk = false;
char g_budgetBuf[320] = {};
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
float g_testWallUs = -1.0f, g_testThreadUs = -1.0f;         // TestSetFeedTimesUs (< 0 = measured)
#endif
// Processor cycles of a thread per microsecond (QueryThreadCycleTime against QueryPerformanceCounter), measured once on the loader
// thread: the largest ratio of three ~1 ms busy spans (preemption during a span only lowers it, towards wall time). -1 = not
// measured yet, 0 = unavailable or implausible: the trip decision then uses wall time alone.
std::atomic<float> g_cyclesPerUs{-1.0f};
void MeasureThreadCyclesOnce()
{
    if (g_cyclesPerUs.load(std::memory_order_acquire) >= 0.0f)
        return;
    LARGE_INTEGER hz{};
    double best = 0.0;
    if (QueryPerformanceFrequency(&hz) && hz.QuadPart > 0)
    {
        for (int k = 0; k < 3; ++k)
        {
            LARGE_INTEGER q0{}, q1{};
            ULONG64 c0 = 0, c1 = 0;
            QueryPerformanceCounter(&q0);
            if (!QueryThreadCycleTime(GetCurrentThread(), &c0))
            {
                best = 0.0;
                break;
            }
            do
                QueryPerformanceCounter(&q1);
            while (q1.QuadPart - q0.QuadPart < hz.QuadPart / 1000);
            if (!QueryThreadCycleTime(GetCurrentThread(), &c1))
            {
                best = 0.0;
                break;
            }
            QueryPerformanceCounter(&q1); // the cycle span lies inside the counter span: the ratio errs low
            const double us = static_cast<double>(q1.QuadPart - q0.QuadPart) * 1.0e6 / static_cast<double>(hz.QuadPart);
            if (us > 0.0 && c1 >= c0)
                best = std::max(best, static_cast<double>(c1 - c0) / us);
        }
    }
    g_cyclesPerUs.store(best >= 100.0 && best <= 20000.0 ? static_cast<float>(best) : 0.0f, std::memory_order_release);
}
integ::PoseSnapshot g_lastSnap{};
bool g_haveLastSnap = false;
// v2 -> v1 handover: after a yield the last published v2 pose stays published (held, as under g_holdSwap) and is then
// blended into the v1 pose over the v1 hard fade-out time (motionhost::kDefaultHardOutTime 0.12 s, MotionHost.hpp:20; the non-hold
// path fades over the same time). Weight / fade / upper mask stay v1's. Not used for toggle off, re-arm, disarm or
// a body switch (no handover there: the v1 path is bit-identical once v2 is off).
float g_hoLeft = 0.0f;
uint64_t g_handoverFrames = 0;
// commit A (item 3): the v2 pose of this feed went through the v1 leg tail (FrameLocked); PublishLocked then already
// carries the tail output (PipelineOutputLocked) and ApplyLocked keeps it instead of the raw evaluator pose.
bool g_tailed = false;
// UTPP-MOTION SEAM motionfix item 6: the last evaluated v2 frame has both hands on the weapon (PoseSnapshot::overlayArms source;
// ApplyLocked adds the blend / handback conditions)
bool g_ovArmsNow = false;
// commit A v1 -> v2 handover (mirror of g_hoLeft): the last snapshot ApplyLocked let through without a v2 pose (pure v1 or the
// v2 -> v1 handover blend, i.e. what was last published) stays the start pose; the v2 pose of the entry feed and the following
// ones blend from it over the same kDefaultHardOutTime (smoothstep, feed time).
integ::PoseSnapshot g_lastBaseSnap{};
bool g_haveLastBase = false;
float g_hiLeft = 0.0f;
uint64_t g_handinFrames = 0;
uint64_t g_tailFrames = 0, g_skipLayout = 0;
bool g_layoutLogged = false;
std::array<motion::TrajSample, adapter::kQueryTrajMax> g_traj{};

// UTPP-MOTION SEAM trav: one traversal at a time (MotionDriver.hpp TraversalEvent). plan.montage points into the installed
// bundle's travPlay: TravResetLocked runs before any bundle swap / body switch / toggle / re-arm.
namespace motiontrav = utpp::mm::traversal::v2;
inline constexpr float kTravBlendIn = 0.2f;  // [P] s, inertialized entry into the montage pose (motion montage blend-in 0.25)
inline constexpr float kTravBlendOut = 0.2f; // [P] s, inertialized handback to MM
// States of one event: havePending (not planned yet) -> planned (plan made, waits for the PSM detail == action; planOk) ->
// episode (detail reached with an Ok plan: active while the montage plays, then MM after the hand-back until the detail
// leaves 8 / 9) or yielding (plan not Ok, or any v2 skip during the episode: v1 path for the rest of the episode).
struct TravRun
{
    bool havePending = false;
    TraversalEvent ev;
    float age = 0.0f;
    bool planned = false, planOk = false, episode = false;
    bool active = false, yielding = false, first = false, handbackNext = false;
    int32_t action = 0, clip = -1;
    motiontrav::Plan plan;
    std::optional<motiontrav::Player> player;
    float cap[3] = {0, 0, 0}; // capsule integrated from velWorld when the feed carries no capsule position (haveExtra)
    bool frame = false;       // the pose of this Step is a traversal frame (FrameLocked root motion from the plan)
    float vel[2] = {0, 0};    // plan root velocity this frame, character frame (m/s)
    float noContactLeft = 0.0f; // s after a handback without foot contacts (no foot lock on the blend-out from the montage)
    // verifier round 2: root-children yaw (plan root yaw - entity yaw, review major 2) of the last traversal frame; it is applied
    // after Evaluate, so the MM inertializer never sees it: the handback keeps it and turns it back to 0 over kTravBlendOut
    float lastYaw = 0.0f, yawBack = 0.0f, yawBackT = -1.0f; // rad, rad, s since the handback (< 0 = none)
};
TravRun g_trav;
TraversalProbe g_travProbe;
uint64_t g_skipTrav = 0;
int g_stepDetail = -1; // PSM detail of the last enabled Step (Skip: a yield on the event's detail ends the ownership)
// task A: MotionDriver.hpp OwnsTraversal. Written under g_m only, read lock-free by the v1 gate.
std::atomic<bool> g_ownsTrav{false};
void TravOwnLocked(bool aOn) { g_ownsTrav.store(aOn, std::memory_order_release); }
// Vault torso limiter (mm/VaultTorso.hpp, MotionHooks::vehicleVaultWeapon). g_vehVaultEvt: the last planned traversal event is a vault over a
// vehicle (obstacle kind 1) that v2 did not accept (Unsupported(vehicle): the game's vault animation plays); set at the plan, cleared with
// the episode (TravResetLocked). g_poseMirror: lock-free copy of g_pose (the overlay the CET script asked for). Both written under g_m,
// read lock-free by the v1 feed (which holds its own mutex).
std::atomic<bool> g_vehVaultEvt{false};
std::atomic<int32_t> g_poseMirror{0};

// ---- ragdoll R0 probe (CET utpp_probe RagdollProbe -> UTPPRagdollGetUpProbe): publishes frame 0 of a motion get-up clip (embedded table, ragdoll/GetUpClips.hpp: the v2 databases carry no get-up clip)
// with the root children (Hips) moved offsetM along the character-frame +Y, for a few seconds, then hands back to MM.
// Ownership: MM (Step / matcher / evaluator / v1 tail) keeps running untouched underneath; only the PUBLISHED snapshot is overwritten
// at the end of ApplyLocked and blended back (BlendFromHeld) at the end. Everything below lives under g_m (the single lock of this
// file: Step, ApplyToSnapshot and the natives all take it), nothing is atomic. Not commanded = g_probe.Active() false = one flag test
// per Step / Apply, no other work. Abort list (ProbeStepLocked / ApplyLocked / lifecycle): v2 off, fault, disarm, shutdown, body or
// bundle change, not TPP, PSM vehicle / scene / swim / workspot / takedown / carry / dead / photo-or-menu / no blackboard, closed
// v2 gate, overlay release, a traversal (it owns the pose), v2 yielding (no pending pose), plus a wall-clock watchdog.
namespace rag = utpp::ragdoll;
rag::GetUpProbe g_probe;
const void* g_probeBundle = nullptr; // identity of the bundle the clip index belongs to (never dereferenced)
integ::PoseSnapshot g_probeHeld{};   // last published probe snapshot: start of the hand-back blend
bool g_probeHeldValid = false;
bool g_probeLogDue = false;          // set by the 0.5 s feed-time tick, consumed by the next published frame
uint32_t g_probeHips = 0;            // bone logged as "hips" (first root child named Hips, else the first root child)
float g_probeCap[3] = {0, 0, 0};     // entity position of the last feed (haveExtra), for the log only
float g_probeYawDeg = 0.0f;
bool g_probeHaveCap = false;
uint32_t g_probeFrames = 0;          // frames published with the probe pose in the current / last run
char g_probeClipName[64] = {};
char g_probeLastEnd[48] = "none";
char g_probeBuf[640];
static_assert(rag::kProbeHandBackS == motionhost::kDefaultHardOutTime, "the hand-back blend span is BlendFromHeld's");
double ProbeWallS() { return static_cast<double>(SteadyNs()) * 1e-9; }

void ProbeLogLocked(const char* aTag, const char* aDetail, const integ::PoseSnapshot* aSnap)
{
    if (g_log == nullptr)
        return;
    char hips[160] = "hipsLocal=n/a hipsWorld~=n/a";
    char ent[96] = "entity=n/a";
    if (g_probeHaveCap)
        std::snprintf(ent, sizeof ent, "entity=(%.3f,%.3f,%.3f)", static_cast<double>(g_probeCap[0]),
                      static_cast<double>(g_probeCap[1]), static_cast<double>(g_probeCap[2]));
    if (aSnap != nullptr && g_probeHips < integ::kMaxMMBones)
    {
        const float* p = &aSnap->pose[size_t(g_probeHips) * 7];
        if (g_probeHaveCap)
        {
            const double yaw = static_cast<double>(g_probeYawDeg) * static_cast<double>(integ::kDegToRad);
            const double c = std::cos(yaw), s = std::sin(yaw); // world = capsule + RotZ(local, +yaw) (inverse of the traversal's RotZ(-yaw))
            std::snprintf(hips, sizeof hips, "hipsLocal=(%.3f,%.3f,%.3f) hipsWorld~=(%.3f,%.3f,%.3f)", static_cast<double>(p[0]),
                          static_cast<double>(p[1]), static_cast<double>(p[2]), g_probeCap[0] + c * p[0] - s * p[1],
                          g_probeCap[1] + s * p[0] + c * p[1], static_cast<double>(g_probeCap[2] + p[2]));
        }
        else
            std::snprintf(hips, sizeof hips, "hipsLocal=(%.3f,%.3f,%.3f) hipsWorld~=n/a", static_cast<double>(p[0]),
                          static_cast<double>(p[1]), static_cast<double>(p[2]));
    }
    // the host logger truncates long rows: offset, entity and hips first, the rest (weight, phase, clip, detail) after
    std::snprintf(g_probeBuf, sizeof g_probeBuf,
                  "[utpp_ragdoll] getup-probe %s offset=%.2f %s %s t=%.2f/%.2f w=%.2f %s f=%u %s%s%s", aTag,
                  static_cast<double>(g_probe.OffsetM()), ent, hips, static_cast<double>(g_probe.ElapsedS()),
                  static_cast<double>(g_probe.DurationS()), aSnap != nullptr ? static_cast<double>(aSnap->weight) : -1.0,
                  g_probe.Holding() ? "hold" : (g_probe.HandingBack() ? "handback" : "idle"), g_probeFrames, g_probeClipName,
                  aDetail ? " " : "", aDetail ? aDetail : "");
    g_log(g_probeBuf);
}

// Ends the run at once (no hand-back): the reason is a state in which the pose must not be published (or a normal end after the blend).
void ProbeEndLocked(const char* aWhy)
{
    if (!g_probe.Active())
        return;
    std::snprintf(g_probeLastEnd, sizeof g_probeLastEnd, "%s", aWhy);
    ProbeLogLocked("END", aWhy, g_probeHeldValid ? &g_probeHeld : nullptr);
    g_probe.Abort();
    g_probeHeldValid = false;
    g_probeLogDue = false;
    g_probeBundle = nullptr;
}

// ---- ragdoll native get-up (ragdoll/GetUpPlayer.hpp; natives UTPPRagdollGetUp* in MotionDriverNatives.cpp). After a ragdoll the script calls
// Plan (freezes the last ragdoll pose, classifies how V lies, aligns the clip) and Begin: from then on this driver publishes the motion
// stand clip, world anchored (the live capsule moves the body in the entity frame, never the other way round), crossfaded from the
// ragdoll pose over kGuBlendInS, and hands back to MM with BlendFromHeldSpan. The ragdoll episode is handed over (Episode::HandToGetUp);
// ragdoll::GetUpOwns() makes the UTPP pose writers resume. MM (Step / matcher / evaluator / v1 tail) keeps running untouched underneath,
// like the R0 probe: only the PUBLISHED snapshot is overwritten at the end of ApplyLocked. Everything lives under g_m.
// Abort list (GetUpStepLocked / ApplyLocked / lifecycle): v2 off, fault, disarm, shutdown, body or bundle change, not TPP, PSM vehicle /
// death / scene / swim / workspot / takedown / carry / no blackboard, a gate closed for 3 feeds, a traversal, a new ragdoll episode,
// the v1 gate closed on the first feed, no capsule for 3 feeds, v2 yielding for 0.1 s, the watchdogs, UTPPRagdollAbort.
// Menu / photo pause the clock. No Plan / Begin = g_gu.Active() false = one flag test per Step / Apply.
rag::GetUpPlayer g_gu;
bool g_guSetting = false; // UTPPRagdollSetGetUp (CET ragdoll.getUp; the shipped default is ON, pushed by the script - false only until the first push)
rag::GuPlan g_guPlan;
bool g_guPlanValid = false;
uint64_t g_guPlanStep = 0;                      // g_steps when Plan ran: Begin refuses when a feed came in between
std::vector<float> g_guSnapLocal, g_guSnapRootWorld; // completed snapshot (database bone order), world transforms of its root children
rag::GuFrame g_guPlanFrame;
rag::GuTarget g_guTarget;                       // entity frame of the release (script teleport target) until a feed sample matches it
const void* g_guBundle = nullptr;               // identity of the bundle the plan belongs to (never dereferenced)
std::array<float, integ::kMaxMMBones * 7> g_guClip{}, g_guOut{};
rag::GuLift g_guLift;
rag::GuHandRate g_guHand; // rate limit of the hand step (fingers above the floor without a snap at the release)
std::array<float, integ::kMaxMMBones * 7> g_guMm{}; // HandOff: the MM pose before the blend (rag::HandOffFootStep: stance of the MM pose)
float g_guZGround = 0.0f;                        // model-space floor of the last composed get-up pose (the hand-off foot step needs it)
bool g_guZGroundValid = false;
integ::PoseSnapshot g_guHeld{};                 // last published get-up snapshot: start of the hand-off blend
integ::PoseSnapshot g_guShown{};                // HandOff: the pose actually published (the blend so far), start of an abort tail
bool g_guShownValid = false, g_guInHandOff = false;
bool g_guHeldValid = false, g_guLogDue = false, g_guPaused = false;
bool g_guGapHeld = false;                       // the held pose is the one of the first publish before any Step (the first-Step grace still applies)
float g_guCap[3] = {0, 0, 0};                   // entity position of the last feed (haveExtra)
float g_guYawDeg = 0.0f;
bool g_guHaveCap = false;
int g_guNoCapFeeds = 0, g_guGateClosed = 0;
bool g_guLastTpp = false;
float g_guAirDist = -1.0f;                      // distanceToGround of the last Step's air feed (-1 = none)
float g_guGraceLeft = 0.0f, g_guTailLeft = 0.0f, g_guLogClock = 0.0f;
bool g_guHandOffEntry = false;                  // MM restarts like after a traversal on the next Step that reaches it
int32_t g_guLastAborted = 0;
float g_guYawErrDeg = 0.0f, g_guFront = 0.0f, g_guLat = 0.0f;
char g_guGroundSrc = '-';
uint32_t g_guFootSteps = 0; // hand-off frames in which a leg stepped
uint32_t g_guFrames = 0, g_guMissing = 0, g_guSnapN = 0, g_guSnapDesc = 0, g_guGapFrames = 0;
uint64_t g_guRuns = 0, g_guAborts = 0, g_guPlans = 0, g_guRefusals = 0;
int32_t g_guLastPlanCode = 0;
char g_guLastEnd[48] = "none";
char g_guBuf[704];
int64_t g_guYieldSinceNs = 0, g_guLastApplyNs = 0;

const char* GetUpPhaseName()
{
    switch (g_gu.Phase())
    {
    case rag::GuPhase::Blend: return "blend";
    case rag::GuPhase::Play: return "play";
    case rag::GuPhase::HandOff: return "handoff";
    default: return "idle";
    }
}

void GetUpLogLocked(const char* aTag, const char* aDetail)
{
    if (g_log == nullptr)
        return;
    const rag::GuPlan& p = g_guPlan;
    std::snprintf(g_guBuf, sizeof g_guBuf,
                  "[utpp_ragdoll] getup %s clip=%s phase=%s t=%.2f/%.2f w=%.2f rms=%.3f anchor=(%.3f,%.3f,%.3f) yawErr=%.1fdeg lift=%.3f "
                  "liftFrames=%u maxDeficit=%.3f limbFrames=%u maxRaw=%.3f frames=%u missing=%u ground=%c front=%.2f lat=%.2f footSteps=%u deficitAt=%s@w%.2f%s%s",
                  aTag, p.clip >= 0 && p.clip < rag::kGuClipCount ? rag::kGuClipNames[p.clip] : "-", GetUpPhaseName(),
                  static_cast<double>(g_gu.ClipTime()), static_cast<double>(p.handOffS), static_cast<double>(g_gu.BlendW()),
                  static_cast<double>(p.rms), static_cast<double>(p.anchor[0]), static_cast<double>(p.anchor[1]),
                  static_cast<double>(p.anchor[2]), static_cast<double>(g_guYawErrDeg), static_cast<double>(g_guLift.lift), g_guLift.frames,
                  static_cast<double>(g_guLift.maxDeficit), g_guLift.limbFrames, static_cast<double>(g_guLift.maxRaw), g_guFrames, g_guMissing,
                  g_guGroundSrc, static_cast<double>(g_guFront), static_cast<double>(g_guLat), g_guFootSteps,
                  g_guLift.maxJoint >= 0 ? rag::kGuClampNames[g_guLift.maxJoint] : "-", static_cast<double>(g_guLift.maxJointW), aDetail ? " " : "",
                  aDetail ? aDetail : "");
    g_log(g_guBuf);
}

// Ends the run at once. aAborted: not a normal end (the 0.12 s tail then blends the held get-up pose into MM). Idempotent, cheap when idle.
void GetUpEndLocked(const char* aWhy, bool aAborted)
{
    if (!g_gu.Active() && !rag::GetUpOwns())
        return;
    const bool fromHandOff = g_guInHandOff || g_gu.Phase() == rag::GuPhase::HandOff; // MM was already restarted for this hand-off
    g_guInHandOff = false;
    if (g_gu.Active())
        g_gu.Abort(aWhy);
    std::snprintf(g_guLastEnd, sizeof g_guLastEnd, "%s", aWhy);
    GetUpLogLocked("End", aWhy);
    g_guLastAborted = aAborted ? -1 : 0;
    g_guAborts += aAborted ? 1u : 0u;
    rag::g_getUpOwns.store(false, std::memory_order_release); // the pose writers see the new owner state from here on
    rag::LastPose().Clear();                                   // lifts the freeze, the next episode starts empty
    g_guTarget.Disarm();
    g_guPlanValid = false;
    g_guLogDue = false;
    g_guYieldSinceNs = 0;
    g_guBundle = nullptr;
    if (aAborted && fromHandOff && g_guShownValid)
    {
        // aborted inside the hand-off blend: the tail starts from the pose that was SHOWN (partly MM), not from the last Play pose,
        // and MM is not restarted a second time (its hand-off restart already ran)
        g_guHeld = g_guShown;
        g_guHeldValid = true;
        g_guTailLeft = rag::kProbeHandBackS;
    }
    else if (aAborted && g_guHeldValid)
    {
        g_guTailLeft = rag::kProbeHandBackS; // the last get-up pose fades into MM instead of snapping
        g_guHandOffEntry = true;             // MM restarts fresh like after a traversal
    }
    else
    {
        g_guTailLeft = 0.0f;
        g_guHeldValid = false;
    }
    g_guShownValid = false;
}
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
std::string g_testForceMontage; // TestForceTraversal
bool g_testTravGate = true;
#endif

Fallback ResolveLocked();

// OwnsTraversal contract (main dc221fa, MotionDriver.hpp): the v1 gate reads OwnsTraversal BEFORE v2's Step on the same feed and a
// false answer on a Climb 8 / Vault 9 feed releases the body (v1 g_effGate closed -> v2 skips at 'gate' and never reaches its
// traversal code). So an event that is pending but not planned yet (deferred: bundle not ready, v2 release / air ownership at
// the event, an episode was running) keeps the traversal owned while v2 can still plan it: v2 on, body known, no fault /
// budget trip, and the body's bundle either loading or ready with traversal data and no fallback. Bounded: the event ages out
// after kTravEventMaxAge, a skipped Step on the event's detail gives it up (Skip), the plan replaces the answer.
// Review minor 4: the deferral is gated on the same conditions on which Step would skip the event's first detail feed
// before reaching the traversal code (fallback incl. Loading, overlay release = melee / fists / body carry / game-pose
// weapon): v2 could not plan it then, so the v1 gate must release at once (motionfix: v2 owns air, no air condition).
// SetOverlayInputs re-evaluates a deferred event's answer when its inputs change.
bool TravCanPlanLaterLocked()
{
    if (!g_enabled || g_body == Body::Unknown || g_faulted || g_budgetTripped)
        return false;
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 4
    return !(g_bundle && g_bundle->body == g_body) || (g_bundle->travOk && ResolveLocked() == Fallback::None); // pre-fix
#else
    if (g_release)
        return false;
    return g_bundle && g_bundle->body == g_body && g_bundle->travOk && ResolveLocked() == Fallback::None;
#endif
}

// A deferred event (pending, not planned, no episode): its OwnsTraversal answer follows the current inputs.
void TravRecheckDeferredLocked()
{
    if (g_trav.havePending && !g_trav.planned && !g_trav.active && !g_trav.episode && !g_trav.yielding)
        TravOwnLocked(TravCanPlanLaterLocked());
}

void TravResetLocked(bool aDropPending)
{
    // a plan made but not started points into the bundle being replaced: re-plan the same event later (bundle swap)
    if (!aDropPending && g_trav.planned)
        g_trav.havePending = true;
    g_trav.planned = g_trav.planOk = g_trav.episode = false;
    g_trav.active = g_trav.yielding = g_trav.first = g_trav.handbackNext = g_trav.frame = false;
    g_trav.noContactLeft = 0.0f;
    g_trav.player.reset();
    g_trav.plan = motiontrav::Plan{};
    g_trav.clip = -1;
    if (aDropPending)
    {
        g_trav.havePending = false;
        g_trav.yawBackT = -1.0f; // dropped (disable / body switch / bundle): no handback turn pending
    }
    g_travProbe.active = g_travProbe.yielding = false;
    g_vehVaultEvt.store(false, std::memory_order_release); // vault torso limiter: the episode is over (a re-planned event sets it again)
    // a pending event survives (bundle swap re-plan, or an event that arrived during the episode now ending): still owned while
    // v2 can plan it (contract above); otherwise not owned
    TravOwnLocked(g_trav.havePending && TravCanPlanLaterLocked());
}

const char* BodyName(Body b) { return b == Body::Woman ? "woman" : (b == Body::Man ? "man" : "unknown"); }
int BodyIndex(Body b) { return b == Body::Man ? 1 : 0; }

bool ReadText(const std::string& aPath, std::string& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::stringstream ss;
    ss << f.rdbuf();
    aOut = ss.str();
    return true;
}

using util::Sha256Hex; // hex SHA-256 (Windows CNG); "" on failure. Worker thread only.

bool ReadBytes(const std::string& aPath, std::vector<uint8_t>& aOut)
{
    std::ifstream f(aPath, std::ios::binary | std::ios::ate);
    if (!f)
        return false;
    const std::streamoff n = f.tellg();
    if (n < 0)
        return false;
    aOut.resize(static_cast<size_t>(n));
    f.seekg(0);
    return n == 0 || static_cast<bool>(f.read(reinterpret_cast<char*>(aOut.data()), n));
}

// Worker: all file I/O and every allocation-heavy init. Fails closed with the adapter::Fallback of the first failing step.
std::unique_ptr<Bundle> LoadBundle(Body aBody, const Paths& aPaths)
{
    auto b = std::make_unique<Bundle>();
    b->body = aBody;
    // Optional weapon-specific data. Missing curves preserve the previous path.
    // All I/O stays on this worker; installed clips are immutable thereafter.
    const auto slash = aPaths.files.db.find_last_of("\\/");
    if (slash != std::string::npos)
    {
        const std::string dir = aPaths.files.db.substr(0, slash + 1) + "recoil\\";
        std::string recoilError;
        for (int32_t id = 1; id < recoil_profile::kSlots; ++id)
        {
            if (!recoil_profile::Valid(id)) continue; // retired id (RecoilProfiles.hpp)
            // A profile with a missing or invalid file stays off (recoilOk false): that weapon keeps the plain authored aim pose.
            const recoil_profile::Entry& entry = recoil_profile::kTable[static_cast<size_t>(id)];
            b->recoilOk[static_cast<size_t>(id)] = b->recoil[static_cast<size_t>(id)][0].Load((dir + entry.shoulder).c_str(), recoilError) &&
                                                   b->recoil[static_cast<size_t>(id)][1].Load((dir + entry.ironsight).c_str(), recoilError);
        }
    }
    adapter::ResolveInputs& r = b->res;
    r.requested = adapter::Version::V2;
    r.body = aBody;
    std::vector<uint8_t> bytes;
    r.dbFile = !aPaths.files.db.empty() && ReadBytes(aPaths.files.db, bytes);
    if (!r.dbFile)
    {
        b->err = "db file " + aPaths.files.db;
        return b;
    }
    // rig identity from the header (offset 24, MotionDatabase MotionHeader::rigHash): a database of the other body parses with
    // its own identity so Resolve reports RigMismatch, not DbInvalid
    b->sha = Sha256Hex(bytes); // SHA on the worker thread, never on the game thread
    b->dbBytes = bytes.size(); // UTPP-MOTION SEAM motionfix (item 4)
    uint64_t hdrRig = 0;
    if (bytes.size() >= 32)
        std::memcpy(&hdrRig, bytes.data() + 24, sizeof(hdrRig));
    std::string err;
    r.dbParsed = b->db.LoadFromMemory(bytes, hdrRig, err);
    std::vector<uint8_t>().swap(bytes);
    if (!r.dbParsed)
    {
        b->err = "db: " + err;
        return b;
    }
    r.dbRigIdentity = b->db.RigIdentity();
    const uint32_t n = b->db.NumBones();
    b->dbHashes.resize(n);
    b->dbParents.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        b->dbHashes[i] = integ::CNameHash(b->db.BoneName(i));
        b->dbParents[i] = b->db.BoneParent(i);
    }
    if (r.dbRigIdentity != adapter::ExpectedRigIdentity(aBody))
    {
        b->err = "db rig identity of another body";
        return b;
    }
    r.chooserFile = !aPaths.files.chooser.empty() && std::ifstream(aPaths.files.chooser).good();
    if (!r.chooserFile)
    {
        b->err = "chooser file " + aPaths.files.chooser;
        return b;
    }
    r.chooserParsed = b->table.LoadFile(aPaths.files.chooser.c_str(), err);
    if (!r.chooserParsed)
    {
        b->err = "chooser: " + err;
        return b;
    }
    {
        // Optional slide section (motion slide bundle). Absent / incomplete / a root database missing from the baked file:
        // declared stays false and mm.slide does nothing for this bundle (the pre-slide behaviour).
        std::string chooserText;
        if (ReadText(aPaths.files.chooser, chooserText) && adapter::ParseSlideSpec(chooserText, b->slide))
        {
            bool feet = false, knees = false;
            for (uint32_t i = 0; i < b->db.NumDatabases(); ++i)
            {
                const std::string_view dbName = b->db.DbName(i);
                feet = feet || dbName == "PSD_Slide_FeetOut";
                knees = knees || dbName == "PSD_Slide_KneesOut";
            }
            if (!feet || !knees)
                b->slide = adapter::SlideSpec{};
        }
    }
    // r5 air: the fall-loop hand-over of a vehicle jump-out is on (ProductionMatcherConfig, MotionDriver.hpp). 2026-11-03 it was off because the offline
    // touchdown pose step grew; the r4 game test then showed the ground jump-start clip held for the whole flight (V crouched mid-air). COST (offline
    // JumpPickTest, worst bone, deg per 50 ms within 0.15 s of the touchdown): JO1 33.8 vs 34.6 and JO2 49.7 vs 35.0 for the same flight without the jump-out rules - the fall loop -> heavy
    // land step is larger than the held Start clip's (the test bounds it at +30 deg and at the flight's own worst step, 53.2 / 64.8)
    const motion::MatcherConfig cfg = ProductionMatcherConfig();
    r.matcherInit = b->matcher.Init(&b->db, &b->table, cfg, err);
    if (!r.matcherInit)
    {
        b->err = "matcher: " + err;
        return b;
    }
    b->qtimes = adapter::BuildQueryTimes(b->db);
    // overlay + style weights: the pose evaluator needs both (missing overlay -> MatcherInit fallback with the reason text;
    // Resolve has no overlay entry, so the evaluator init is folded into matcherInit)
    const uint64_t ovRig = aBody == Body::Man ? ov::kRigIdentityManOv : ov::kRigIdentityWomanOv;
    b->ovOk = b->ov.LoadFromFile(aPaths.files.overlay.c_str(), ovRig, err);
    if (!b->ovOk)
    {
        r.matcherInit = false;
        b->err = "overlay: " + err;
        return b;
    }
    std::string text;
    b->styleOk = ReadText(aPaths.files.style, text) && b->style.Parse(text, &err);
    b->evalOk = b->eval.Init(&b->db, &b->ov, b->styleOk ? &b->style : nullptr, kFirstBone, err,
                                motionpose::BaseBlendPolicy::LiveCrossfade);
    if (!b->evalOk)
    {
        r.matcherInit = false;
        b->err = "pose evaluator: " + err;
        return b;
    }
    if (n > integ::kMaxMMBones || b->qtimes.overflow)
    {
        r.matcherInit = false;
        b->err = n > integ::kMaxMMBones ? "more bones than a snapshot carries" : "query time overflow";
        return b;
    }
    b->pose.assign(n, BoneTransform{});
    if (!aPaths.lean.empty() && std::ifstream(aPaths.lean, std::ios::binary).good())
    {
        // optional lean pack: any failure leaves leanOk false (one line in the load-done line), never the bundle
        std::string lerr;
        const uint64_t leanRig = aBody == Body::Man ? mm::lean::kRigIdentityMan : mm::lean::kRigIdentityWoman;
        if (!b->leanPack.LoadFromFile(aPaths.lean.c_str(), leanRig, lerr))
            b->leanErr = "pack: " + lerr;
        else if (!lrt::PackMatchesOverlay(b->leanPack, b->ov, lerr))
            b->leanErr = "rig: " + lerr;
        else if (!b->lean.Bind(b->leanPack))
            b->leanErr = "layers";
        else
        {
            lrt::BuildClipMap(
                b->leanPack, b->db.NumClips(), [&](uint32_t c) { return b->db.ClipName(c); },
                [&](uint32_t c) { return b->db.Clip(c).numFrames; }, b->leanClipMap, b->leanMismatch);
            b->leanMapped = 0;
            for (const int32_t m : b->leanClipMap)
                b->leanMapped += m >= 0 ? 1 : 0;
            b->leanOk = true;
        }
    }
    b->travFileOk = !aPaths.traversal.empty() && traversal::LoadTraversalFile(aPaths.traversal, b->trav, err);
    // UTPP-MOTION SEAM trav: table + playable montages (pose = the database clip of the montage's sequence, else the
    // montage's version 2 poses when every db bone has a pose bone); any missing part leaves travOk false -> every
    // traversal of this body yields (game / v1 path), MM itself is unaffected
    std::string tj;
    b->travTableOk = !aPaths.traversalTable.empty() && ReadText(aPaths.traversalTable, tj) && b->travTable.Load(tj, err);
    if (b->travFileOk)
    {
        // poses only from the file of this body's rig (bone names are shared by both rigs)
        if (!b->trav.poseBones.empty() &&
            b->trav.rigName == (aBody == Body::Man ? "player_man_skeleton" : "player_woman_skeleton"))
        {
            b->travPoseMap.assign(n, -1);
            b->travPosesOk = true;
            for (uint32_t i = 0; i < n && b->travPosesOk; ++i)
            {
                for (size_t k = 0; k < b->trav.poseBones.size(); ++k)
                    if (b->trav.poseBones[k] == b->db.BoneName(i))
                        b->travPoseMap[i] = static_cast<int32_t>(k);
                b->travPosesOk = b->travPoseMap[i] >= 0;
            }
            if (b->travPosesOk)
                b->travRaw.assign(n, BoneTransform{});
        }
        std::unordered_map<std::string_view, int32_t> clips;
        for (uint32_t c = 0; c < b->db.Header().numClips; ++c)
            clips.emplace(b->db.ClipName(c), static_cast<int32_t>(c));
        const size_t poseFloats = b->trav.poseBones.size() * 7;
        for (traversal::TravMontage& m : b->trav.montages)
        {
            const auto it = clips.find(m.sequence);
            const bool inDb = it != clips.end() && b->db.Clip(static_cast<uint32_t>(it->second)).numFrames > 1;
            const bool poses = b->travPosesOk && m.HasPose() && m.pose.size() == static_cast<size_t>(m.frames) * poseFloats;
            if (!inDb && !poses)
                continue;
            b->travPlayClip.push_back(inDb ? it->second : -1);
            b->travFromPoses += inDb ? 0u : 1u;
            b->travPlay.montages.push_back(std::move(m)); // trav is not read after the load
        }
        b->travPlay.version = b->trav.version;
        b->travPlay.rigName = b->trav.rigName;
        b->travPlay.rigHash = b->trav.rigHash;
        b->travPlay.rigBones = b->trav.rigBones;
        b->travPlay.poseBones = b->trav.poseBones;
    }
    b->travOk = b->travFileOk && b->travTableOk && !b->travPlay.montages.empty();
    // native get-up clips: optional, own file (the traversal planner never sees these montages). Any problem leaves getupOk false and
    // the reason in getupErr; the vanilla recovery then stays exactly as it is.
    if (!aPaths.getup.empty() && std::ifstream(aPaths.getup, std::ios::binary).good())
    {
        std::string gerr;
        if (!traversal::LoadTraversalFile(aPaths.getup, b->getup, gerr))
            b->getupErr = "bad:" + gerr;
        else if (!utpp::ragdoll::BuildGetUpClips(
                     b->getup, aBody == Body::Man ? "player_man_skeleton" : "player_woman_skeleton", n, kFirstBone, b->dbParents.data(),
                     [&](uint32_t i) { return b->db.BoneName(i); }, b->guClips))
            b->getupErr = "bad:" + b->guClips.err;
        else
        {
            b->getupOk = true;
            b->getupErr.clear();
        }
    }
    return b;
}

Paths PathsLocked(Body aBody)
{
    const int i = BodyIndex(aBody);
    return g_haveOverride[i] ? g_override[i] : PathsForBody(aBody, g_dataDir);
}

void RemapLocked()
{
    g_authoredReload = false;
    ResetAuthoredRecoil();
    g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset();
    g_reloadContactsEligible = false;
    reload_contacts_runtime::g_config.Publish(reload_contacts_runtime::Prepared{});
    integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
    g_shootLastReadyValid = g_shootBaseValid = false;
    g_shootEventTicks = 0;
    g_shootHold.Reset();
    g_shootEpisode.Reset();
    static uint64_t captureGeneration = 0;
    integ::PoseSnapshot captureMap{};
    captureMap.sequence = ++captureGeneration;
    integ::g_gamePoseReadMap.Publish(captureMap); // invalidate even when this remap cannot resolve a rig
    g_map = integ::BoneMapResult{};
    g_topology = ~0u;
    if (!g_bundle || !g_bundle->db.Loaded() || g_rig == 0 || g_gameHashes.empty())
        return;
    g_topology = adapter::TopologyMismatches(g_bundle->db, g_gameHashes, g_gameParents);
    g_map = integ::BuildBoneMap(g_bundle->dbHashes, g_bundle->dbParents, g_gameHashes, g_gameParents, kFirstBone);
    const auto readMap = integ::BuildBoneMap(g_bundle->dbHashes, g_bundle->dbParents, g_gameHashes, g_gameParents, 0);
    if (g_map.ok && readMap.ok && g_topology == 0)
    {
        captureMap.rig = g_rig;
        captureMap.numBones = readMap.numDbBones;
        std::memcpy(captureMap.gameIndex, readMap.gameIndex, sizeof captureMap.gameIndex);
        integ::g_gamePoseReadMap.Publish(captureMap);
        reload_contacts_runtime::Prepared reload;
        // Same builder as the offline tests: the cold readMap maps Root and Trajectory too (firstBone 0), Hips/Head by name.
        if (reload_contacts_runtime::FillPrepared(
                reload, g_rig, readMap.numDbBones, readMap.gameIndex,
                [&](uint32_t i) { return g_bundle->db.BoneParent(i); }, // validated DB topology, <=128 bones
                [&](uint32_t i) { return g_bundle->db.BoneName(i); }))
            reload_contacts_runtime::g_config.Publish(reload);
    }
}

void JoinWorker()
{
    if (g_worker.t.joinable())
        g_worker.t.join();
}

// fault: non-finite pose or an exception in the v2 path -> v1 for the rest of the session (never cleared)
// (no per-frame logging) + UTPP-MOTION SEAM motionfix (item 4): a line on every change of the fallback reason of the
// Step (also back to none = v2 path), rate bounded [P] (see kReasonWindow). Was: each reason once per session (mutation 3).
bool g_haveSeenReason = false;
Fallback g_seenReason = Fallback::None;
void LogReasonLocked(Fallback aReason, const char* aDetail)
{
#if UTPP_MOTION_DRIVER_FIX_MUTATION == 3
    const uint32_t bit = 1u << (static_cast<uint32_t>(aReason) & 31u);
    if (aReason == Fallback::None || (g_loggedReasons & bit) != 0)
        return;
    g_loggedReasons |= bit;
#else
    const bool change = !g_haveSeenReason || aReason != g_seenReason;
    g_haveSeenReason = true;
    g_seenReason = aReason;
    if (g_haveLoggedReason && aReason == g_loggedReason)
        return; // unchanged since the last line (A -> B -> A inside a full window: back at the written reason)
    if (g_reasonClock - g_reasonWindowStart >= kReasonWindow)
    {
        g_reasonWindowStart = g_reasonClock;
        g_reasonInWindow = 0;
    }
#if UTPP_MOTION_DRIVER_FIX_MUTATION != 6 // mutation 6: no rate bound
    if (g_reasonInWindow >= kReasonLinesPerWindow)
    {
        g_reasonSuppressed += change ? 1u : 0u; // written once the window ends if the reason still differs
        return;
    }
#else
    (void)change;
#endif
    ++g_reasonInWindow;
    g_haveLoggedReason = true;
    g_loggedReason = aReason;
#endif
    if (g_log == nullptr)
        return;
    const bool own = g_bundle && g_bundle->body == g_body;
    const char* cause = aDetail;
    if (cause == nullptr && aReason != Fallback::None)
    {
        if (g_loading && !own)
            cause = "loading"; // the active body's bundle is not in yet (body switch / first load): transient db-missing
        else if (own && !g_bundle->err.empty())
            cause = g_bundle->err.c_str();
        else if (aReason == Fallback::RigMismatch && g_rig == 0)
            cause = "not armed";
        else if (aReason == Fallback::BoneMap || aReason == Fallback::RigMismatch)
            cause = "bone map / topology against the armed rig";
    }
    std::snprintf(g_reasonBuf, sizeof(g_reasonBuf),
                  "UTPP mm.motionDriver fallback=%s%s%.200s body=%s bundle=%s%s%s rig=0x%llx map=%d:%u/%u topo=%u suppressed=%u (%s path)",
                  adapter::FallbackName(aReason), cause ? " cause=" : "", cause ? cause : "", BodyName(g_body),
                  g_bundle ? BodyName(g_bundle->body) : "none", g_loading ? " loading:" : "",
                  g_loading ? BodyName(g_loadingBody) : "", static_cast<unsigned long long>(g_rig), g_map.ok ? 1 : 0,
                  g_map.mapped, g_map.numDbBones, g_topology, g_reasonSuppressed, aReason == Fallback::None ? "motion" : "base");
    g_reasonSuppressed = 0;
    ++g_reasonLines;
    g_log(g_reasonBuf);
}

void FaultLocked(const char* aWhy)
{
    g_faulted = true;
    if (g_faultWhy == nullptr)
        g_faultWhy = aWhy; // first cause only (static string)
    LogReasonLocked(Fallback::Fault, g_faultWhy);
    g_reason = Fallback::Fault;
    RefreshSlideLocked(); RefreshShooterModeLocked(); RefreshTravActiveLocked();
    g_lastSkip = aWhy;
    g_pending = false;
    g_wasRunning = false;
    g_hoLeft = 0.0f; // fault: the held pose may be the faulty one, v1 at once
    g_hiLeft = 0.0f;
    TravResetLocked(true); // UTPP-MOTION SEAM trav
    ProbeEndLocked("fault");
    GetUpEndLocked("fault", true);
}

// UTPP-MOTION SEAM motionfix (item 4): result of a finished load for the "load done" line (worker, under g_m; no I/O here)
void NoteLoadDoneLocked(Body aBody, const Bundle* b, const char* aOutcome, float aMs)
{
    LoadDone& d = g_loadDone;
    d = LoadDone{};
    d.pending = true;
    d.body = aBody;
    d.outcome = aOutcome;
    d.load = g_loads;
    d.ms = aMs;
    if (b == nullptr)
        return;
    d.dbBytes = b->dbBytes;
    d.rigId = b->res.dbRigIdentity;
    d.dbOk = b->res.dbParsed;
    d.chooserOk = b->res.chooserParsed;
    d.matcherOk = b->res.matcherInit;
    d.ovOk = b->ovOk;
    d.styleOk = b->styleOk;
    d.travOk = b->travOk;
    if (b->res.dbParsed)
    {
        d.bones = b->db.NumBones();
        d.clips = b->db.Header().numClips;
        d.dbs = b->db.NumDatabases();
        d.frames = b->db.NumFrames();
    }
    d.travMontages = static_cast<uint32_t>(b->travPlay.montages.size());
    d.travFromPoses = b->travFromPoses;
    std::snprintf(d.sha, sizeof(d.sha), "%.16s", b->sha.empty() ? "none" : b->sha.c_str());
    std::snprintf(d.err, sizeof(d.err), "%s", b->err.c_str());
    if (b->leanOk)
        std::snprintf(d.lean, sizeof(d.lean), "ok:%d/%d", b->leanMapped, b->leanMismatch);
    else
        std::snprintf(d.lean, sizeof(d.lean), "%s", b->leanErr.empty() ? "none" : ("fail:" + b->leanErr).c_str());
}

// Game thread (Step): the "load done" line of a finished load, once
void EmitLoadDoneLocked()
{
    if (!g_loadDone.pending)
        return;
    g_loadDone.pending = false;
#if UTPP_MOTION_DRIVER_FIX_MUTATION == 4
    return; // mutation 4: no load lines (pre-fix)
#else
    if (g_log == nullptr)
        return;
    const LoadDone& d = g_loadDone;
    const bool installed = std::strcmp(d.outcome, "installed") == 0;
    std::snprintf(g_loadBuf, sizeof(g_loadBuf),
                  "UTPP mm.motionDriver load done #%llu body=%s %s ms=%.0f dbBytes=%llu bones=%u clips=%u dbs=%u frames=%u rigId=0x%016llx(%s)"
                  " db=%d chooser=%d matcher=%d ov=%d style=%d trav=%d:%u/%u sha=%s map=%d:%u/%u lean=%s err='%s'",
                  static_cast<unsigned long long>(d.load), BodyName(d.body), d.outcome, static_cast<double>(d.ms),
                  static_cast<unsigned long long>(d.dbBytes), d.bones, d.clips, d.dbs, d.frames,
                  static_cast<unsigned long long>(d.rigId),
                  d.rigId == adapter::ExpectedRigIdentity(d.body) ? "body" : "other", d.dbOk ? 1 : 0, d.chooserOk ? 1 : 0,
                  d.matcherOk ? 1 : 0, d.ovOk ? 1 : 0, d.styleOk ? 1 : 0, d.travOk ? 1 : 0, d.travMontages, d.travFromPoses, d.sha,
                  installed && g_map.ok ? 1 : 0, installed ? g_map.mapped : 0u, installed ? g_map.numDbBones : 0u, d.lean, d.err);
    ++g_loadLines;
    g_log(g_loadBuf);
#endif
}

// Thread entry: an exception must never reach the thread boundary (std::terminate would end the game process).
void WorkerMain(Body aBody, Paths aPaths) noexcept
{
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    {
        std::unique_lock hold(g_testWaitM);
        if (g_testWorkerPause)
        {
            g_testWorkerEntered = true;
            g_testWaitCv.notify_all();
            if (!g_testWaitCv.wait_for(hold, std::chrono::seconds(10), [] { return g_testWorkerResume; }))
                g_testWorkerTimedOut = true;
        }
    }
#endif
    MeasureThreadCyclesOnce(); // feed budget: thread time factor, off the game thread, before the first feed of this bundle
    const auto t0 = std::chrono::steady_clock::now(); // UTPP-MOTION SEAM motionfix (item 4): load time (load done line)
    std::unique_ptr<Bundle> b;
    const char* fault = nullptr;
    try
    {
        b = LoadBundle(aBody, aPaths);
    }
    catch (const std::bad_alloc&)
    {
        fault = "fault:load-bad-alloc";
    }
    catch (...)
    {
        fault = "fault:load-exception";
    }
    std::unique_ptr<Bundle> old;
    const float ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    {
        std::lock_guard lock(g_m);
        ++g_loads;
        // UTPP-MOTION SEAM motionfix (item 4)
        NoteLoadDoneLocked(aBody, b.get(), fault != nullptr ? fault : (g_body == aBody && !g_stopping ? "installed" : "dropped"), ms);
        if (fault != nullptr)
            FaultLocked(fault);
        else if (g_body == aBody && !g_stopping)
        {
            old = std::move(g_bundle); // the other body (or an older load) is released outside the lock
            // (MotionDriver.hpp Frame: a MotionSetFrame pose is a caller-owned copy, never a pointer into this bundle)
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
            RetireForTestLocked(old.get());
#endif
            ProbeEndLocked("bundle-change"); // the probe's clip index belongs to the old bundle
            GetUpEndLocked("bundle-change", true);
            g_bundle = std::move(b);
            // a new evaluator starts at the constant: carry the setting over here, under g_m, so a change made while the bundle was loading is not lost
            if (g_bundle->evalOk)
                g_bundle->eval.SetFeminineShoulders(g_femShoulderM);
            RefreshSlideLocked(); RefreshShooterModeLocked(); RefreshTravActiveLocked(); // UTPP-MOTION SEAM slide camera: the new bundle may not declare the slide section
            g_budgetTripped = false;
            g_budgetPauseLeft = 0.0f;
            g_timeCount = 0;
            g_wasRunning = false; // commit A: a handover started by the body switch keeps running (held snapshot = game rig)
            g_pending = false;
            TravResetLocked(false); // UTPP-MOTION SEAM trav: the plan points into the replaced bundle
            RemapLocked();
        }
        else
            old = std::move(b); // body changed while loading: drop this load, the next Step starts the right one
    }
    old.reset(); // frees ~200 MB outside the lock; g_loading stays set until it is done, so MaybeLoadLocked never joins
                 // a worker that is still freeing (the game thread holds the v1 g_mutex there)
    std::lock_guard lock(g_m);
    g_loading = false;
}

// Game thread (Step) or settings: starts the lazy load of the active body. Never blocks on I/O.
void MaybeLoadLocked()
{
    if (!g_enabled || g_stopping || g_body == Body::Unknown || g_loading)
        return;
    if (g_bundle && g_bundle->body == g_body && !g_reload)
        return;
    if (g_worker.t.joinable())
        g_worker.t.join(); // g_loading false = the worker freed the old bundle and only returns: join is immediate
    // the worker needs g_m (held here) before it can clear g_loading, so the flags are set after a successful start;
    // a failed start (std::system_error) leaves them unchanged and reaches the Step guard (fault)
    Paths p = PathsLocked(g_body);
    // UTPP-MOTION SEAM motionfix (item 4): "load start" line (one per load, game thread; written after a successful start)
    const char* why = !g_bundle ? "first" : (g_bundle->body != g_body ? "body-switch" : "reload");
    std::snprintf(g_loadBuf, sizeof(g_loadBuf), "UTPP mm.motionDriver load start #%llu body=%s why=%s from=%s db=%.400s",
                  static_cast<unsigned long long>(g_loads + 1), BodyName(g_body), why, g_bundle ? BodyName(g_bundle->body) : "none",
                  p.files.db.c_str());
    g_worker.t = std::thread(WorkerMain, g_body, std::move(p));
    g_reload = false;
    g_loading = true;
    g_loadingBody = g_body;
#if UTPP_MOTION_DRIVER_FIX_MUTATION != 4
    if (g_log != nullptr)
    {
        ++g_loadLines;
        g_log(g_loadBuf);
    }
#endif
}

Fallback ResolveLocked()
{
    adapter::ResolveInputs r;
    if (g_bundle && g_bundle->body == g_body)
        r = g_bundle->res;
    r.requested = g_enabled ? adapter::Version::V2 : adapter::Version::V1;
    r.body = g_body;
    if (g_bundle && g_bundle->body == g_body && r.dbParsed)
        r.topologyMismatches = g_topology;
    r.boneMapOk = g_map.ok;
    r.faulted = g_faulted;
    r.budgetTripped = g_budgetTripped;
    return adapter::Resolve(r);
}

float P95Of(const std::array<float, adapter::Budget::kWindow>& aRing)
{
    const size_t n = std::min<size_t>(g_timeCount, aRing.size());
    if (n == 0)
        return 0.0f;
    std::copy(aRing.begin(), aRing.begin() + static_cast<std::ptrdiff_t>(n), g_sorted.begin());
    const size_t k = static_cast<size_t>(std::ceil(0.95 * static_cast<double>(n))) - 1;
    std::nth_element(g_sorted.begin(), g_sorted.begin() + static_cast<std::ptrdiff_t>(k),
                     g_sorted.begin() + static_cast<std::ptrdiff_t>(n));
    return g_sorted[k];
}
float P95Locked() { return P95Of(g_times); }         // the trip statistic (status feedP95us)
float WallP95Locked() { return P95Of(g_wallTimes); } // status / log only, never per feed

ov::PoseOverlay OverlayLocked()
{
    ov::PoseOverlay o = ov::PoseOverlay::Default;
    if (g_pose == 9 || g_pose == static_cast<int32_t>(ov::PoseOverlay::Injured))
        o = ov::PoseOverlay::Injured;
    else if (g_pose >= 0 && g_pose < static_cast<int32_t>(ov::PoseOverlay::Count))
        o = static_cast<ov::PoseOverlay>(g_pose);
    if (o == ov::PoseOverlay::Default && g_injured)
        o = ov::PoseOverlay::Injured;
    return o;
}

void Skip(uint64_t& aCounter, const char* aWhy)
{
#if UTPP_SHOOTER_MODE_MUTATION != 13
    g_fnDrivingNs.store(0, std::memory_order_release); // v2 does not drive the body this feed: no Shooter speed ratio
#endif
    ResetAuthoredRecoil();
    g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset();
    g_reloadContactsEligible = false;
    // the frames go to the game / v1: the air weight of the weapon pose's legs does not outlive them (the v2 re-entry may be on the ground)
#if !(defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 23) // mutation 23: the weight is frozen across a skip
    g_airLegsFade.Reset();
    g_jumpOutArmsFade.Reset();
    g_jumpOutWindowNow = false;
#endif
    ++aCounter;
    g_lastSkip = aWhy;
    // UTPP-MOTION SEAM trav: a montage / episode interrupted by any yield is not resumed (rest of it = yield), and an
    // accepted plan whose detail arrives on a skipped feed is given up: v2 no longer owns the traversal (OwnsTraversal)
    // A deferred (not yet planned) event whose detail arrives on a skipped feed is given up the same way (it was owned while
    // v2 could still plan it; v2 cannot start the montage from the episode's first feed now).
    const bool pendingOnDetail = g_trav.havePending && !g_trav.planned && !g_trav.active && !g_trav.episode &&
                                 !g_trav.yielding && g_stepDetail == g_trav.ev.action;
    if (pendingOnDetail)
    {
        g_trav.havePending = false;
        g_vehVaultEvt.store(false, std::memory_order_release); // vault torso limiter: the deferred event never planned, nothing to keep
    }
    if (g_trav.active || g_trav.episode || (g_trav.planned && g_trav.planOk && g_stepDetail == g_trav.ev.action) ||
        pendingOnDetail)
    {
        g_trav.active = g_trav.episode = g_trav.planned = g_trav.planOk = false;
        g_trav.player.reset();
        g_trav.yielding = true;
        g_travProbe.active = false;
        g_travProbe.yielding = true;
        TravOwnLocked(false);
    }
    if (g_wasRunning && g_haveLastSnap)
        g_hoLeft = motionhost::kDefaultHardOutTime; // handover starts: this publish still shows the held v2 pose
    g_pending = false;
    g_wasRunning = false;
}
} // namespace

Paths PathsForBody(Body aBody, const std::string& aDir)
{
    Paths p;
    p.files = adapter::PathsFor(aBody, aDir);
    if (aBody != Body::Unknown)
    {
        std::string dir(aDir);
        if (!dir.empty() && dir.back() != '\\' && dir.back() != '/')
            dir += '\\';
        p.traversal = dir + "data\\utpp_traversal_" + BodyName(aBody) + ".utpptrav";
        p.traversalTable = dir + "data\\utpp_traversal_table.json"; // UTPP-MOTION SEAM trav
        p.lean = dir + "data\\utpp_lean_" + BodyName(aBody) + ".bin";
        p.getup = dir + "data\\utpp_getup_" + BodyName(aBody) + ".utpptrav";
    }
    return p;
}

void SetEnabled(bool aOn)
{
    std::lock_guard lock(g_m);
    g_fnHoldValid = false; // a new v2 session never inherits a held heading
    if (!aOn) g_authoredReload = false;
    if (!aOn) integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
    if (!aOn) { g_airLegsFade.Reset(); g_jumpOutArmsFade.Reset(); g_jumpOutWindowNow = false; }
    if (!aOn) { ResetAuthoredRecoil(); g_weaponAction = 0; g_hipFirePoseHold.Reset(); g_jumpOutExitHold.Reset(); g_stationaryShotTurn.Reset(); g_reloadUpperWeight = 0.0f; g_reloadUpperEnv.Reset(); g_reloadRollHold.Reset(); g_slideDeRoll.Reset(); g_rifleSlideLatch.Reset(); g_slideCarryAim.Reset(); ResetMagTap(); g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset(); g_reloadContactsEligible = false; g_shootLastReadyValid = g_shootBaseValid = false; g_shootHold.Reset(); g_shootEpisode.Reset(); }
    if (g_enabled != aOn)
    {
        g_aimUpperTail = false;
        g_weaponUpperFrame = g_weaponUpperTail = false;
        g_shootRouteNow = false;
        g_shootRouteHeld = g_shootRouteInShot = g_shootRouteLive = false;
        // re-enable after a failed load (files copied meanwhile) loads again; a good bundle is kept
        if (aOn && g_bundle && !g_bundle->res.matcherInit)
            g_reload = true;
        // commit A: toggle off while v2 runs -> the same v2 -> v1 handover as a yield; toggle on -> v1 -> v2 hand-in
        g_hoLeft = (!aOn && g_wasRunning && g_haveLastSnap && !g_faulted) ? motionhost::kDefaultHardOutTime : 0.0f;
        g_hiLeft = 0.0f;
        g_pending = false;
        g_wasRunning = false;
        g_budgetTripped = false; // g_faulted stays: fault = v1 until the end of the session
        g_budgetPauseLeft = 0.0f; // the pause length / trip count stay (session values)
        g_timeCount = 0;
        TravResetLocked(true); // UTPP-MOTION SEAM trav
        ProbeEndLocked(aOn ? "enable-change" : "motion-off");
        GetUpEndLocked(aOn ? "enable-change" : "motion-off", true);
    }
    g_enabled = aOn;
    g_reason = ResolveLocked();
    RefreshSlideLocked(); RefreshShooterModeLocked(); RefreshTravActiveLocked();
}

bool Enabled()
{
    std::lock_guard lock(g_m);
    return g_enabled;
}

bool SetRigBody(int32_t aBody)
{
    if (aBody != kBodyWoman && aBody != kBodyMan)
    {
        integ::SetKnownBody(-1); // "unknown" (sent when the player detaches): host poses carry no body until the next report
        return false;
    }
    mm::meleebase::SetArmedBody(aBody == kBodyMan ? 1 : 0); // the melee chest base correction picks its clip table from this (0 Female, 1 Male)
    integ::SetKnownBody(aBody); // the body of every host publish too (mm/GameIntegration.hpp StampHostIdentity), so the skin helper recompute runs on v1 poses
    std::lock_guard lock(g_m);
    const Body b = aBody == kBodyMan ? Body::Man : Body::Woman;
    if (b != g_body)
    {
        g_authoredReload = false;
        ResetAuthoredRecoil();
        g_weaponAction = 0;
        g_hipFirePoseHold.Reset(); g_jumpOutExitHold.Reset(); g_stationaryShotTurn.Reset();
        g_reloadUpperWeight = 0.0f; g_reloadUpperEnv.Reset(); g_reloadRollHold.Reset(); g_slideDeRoll.Reset(); g_rifleSlideLatch.Reset(); g_slideCarryAim.Reset(); ResetMagTap(); g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset(); g_reloadContactsEligible = false;
        g_aimUpperTail = false;
        g_weaponUpperFrame = g_weaponUpperTail = false;
        g_shootRouteNow = false;
        g_shootRouteHeld = g_shootRouteInShot = g_shootRouteLive = false;
        g_airLegsFade.Reset();
        g_jumpOutArmsFade.Reset();
        g_jumpOutWindowNow = false;
        g_body = b;
        TravResetLocked(true); // UTPP-MOTION SEAM trav
        ProbeEndLocked("body-change");
        GetUpEndLocked("body-change", true);
        // commit A: body switch while v2 runs -> v2 -> v1 handover (dropped by HandoverLocked when the rig changes)
        g_hoLeft = (g_wasRunning && g_haveLastSnap && !g_faulted) ? motionhost::kDefaultHardOutTime : 0.0f;
        g_hiLeft = 0.0f;
        g_pending = false;
        g_wasRunning = false;
        if (g_bundle && g_bundle->body != b)
            g_map = integ::BoneMapResult{}; // the installed bundle is the other body: no pose until the new one is in
        else
            RemapLocked(); // A -> B -> A before B was installed: the bundle is this body again, map it against the armed rig
    }
    return true;
}

// motionfix item 2 [P]: chest height of the aim origin (m above the model origin). The target point is 60 m out
// (AimOffset targetDistance), so +-0.5 m here moves the pitch by < 0.5 deg.
constexpr float kAimOriginZ = 1.4f;

bool AimAnglesFromModel(const float aT[3], float& aYawDeg, float& aPitchDeg)
{
    const float x = aT[0], y = aT[1], z = aT[2] - kAimOriginZ;
    const float h = std::sqrt(x * x + y * y);
    if (!std::isfinite(x + y + z) || (h <= 1e-4f && std::fabs(z) <= 1e-4f))
        return false;
    aYawDeg = std::atan2(-x, y) * (180.0f / 3.14159265358979f);
    aPitchDeg = std::atan2(z, h) * (180.0f / 3.14159265358979f);
    return true;
}

void SetAimTarget(bool aEnabled, bool aValid, const float aTargetModel[3])
{
    std::lock_guard lock(g_m);
    g_aimTgtOn = aEnabled;
    g_aimTgtValid = aEnabled && aValid && aTargetModel != nullptr;
    if (g_aimTgtValid)
        std::memcpy(g_aimTgt, aTargetModel, sizeof g_aimTgt);
}

void SetCameraTarget(bool aEnabled, bool aValid, const float aTargetModel[3])
{
    std::lock_guard lk(g_m);
    g_cameraTargetValid = aEnabled && aValid && aTargetModel && std::isfinite(aTargetModel[0]) &&
                          std::isfinite(aTargetModel[1]) && std::isfinite(aTargetModel[2]);
    if (g_cameraTargetValid)
    {
        std::copy_n(aTargetModel, 3, g_cameraTarget);
        g_cameraTargetTime = RecoilClock();
    }
}

void SetAimGameUpper(bool aOn)
{
    std::lock_guard lock(g_m);
    g_aimGameUpper = aOn;
    if (!aOn)
        g_aimUpperTail = false;
}

void SetAimUpperFade(bool aOn)
{
    std::lock_guard lock(g_m);
    g_aimUpperFade = aOn;
    if (!aOn)
        g_aimUpperTail = false;
}

void SetGripEnabled(bool aOn)
{
    std::lock_guard lock(g_m);
    if (!aOn) ResetAuthoredRecoil();
    g_gripEnabled = aOn;
    if (!aOn) integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
}

bool SetHeadLead(bool aOn)
{
    std::lock_guard lock(g_m);
    if (aOn && !g_headLeadOn && g_hl.Weight() == 0.0f && g_hl.Corr() == 0.0f)
    {
        g_hl.Reset();
        g_hlHaveApply = false;
    }
    g_headLeadOn = aOn; // off: the running correction fades out (HeadLeadStepLocked blocks it) instead of popping
    return true;
}

bool SetHeadLeadParams(float aLeadDeg, float aMaxDeg)
{
    if (!hlead::ValidTuning(aLeadDeg, aMaxDeg))
        return false;
    std::lock_guard lock(g_m);
    g_hlParams.leadDegAtRef = aLeadDeg;
    g_hlParams.maxLeadDeg = aMaxDeg;
    return true;
}

bool SetOrientWarp(bool aOn)
{
    std::lock_guard lock(g_m);
    if (aOn && !g_orientWarpOn && g_ow.Idle())
        g_ow.Reset();
    g_orientWarpOn = aOn; // off: the running correction fades out (OrientWarpStepLocked blocks it) instead of popping
    return true;
}

bool SetFeminineShoulderCm(float aCm)
{
    if (!std::isfinite(aCm) || aCm < 0.0f || aCm > utpp::mm::femshoulder::kMaxAmountCm)
        return false;
    std::lock_guard lock(g_m);
    g_femShoulderM = utpp::mm::femshoulder::AmountFromCm(aCm); // exact: 2.5 cm == kFeminineShoulderAmount bit for bit
    if (g_bundle && g_bundle->evalOk)
        g_bundle->eval.SetFeminineShoulders(g_femShoulderM);
    return true;
}

bool SetPelvisYawTrim(float aDeg)
{
    if (!owarp::ValidPelvisTrim(aDeg))
        return false;
    std::lock_guard lock(g_m);
    g_pelvisTrimDeg = aDeg; // the CET slider moves in 0.5 deg steps; the weight fade covers on / off, a step is applied as set
    return true;
}

float PelvisYawTrim()
{
    std::lock_guard lock(g_m);
    return g_pelvisTrimDeg;
}

float PelvisYawTrimWeight()
{
    std::lock_guard lock(g_m);
    return g_pt.Weight();
}

std::string PelvisYawTrimReason()
{
    std::lock_guard lock(g_m);
    return g_ptWhy;
}

bool SetBodyYawOffset(float aDeg)
{
    if (!owarp::ValidBodyYawOffset(aDeg))
        return false;
    std::lock_guard lock(g_m);
    g_bodyOffsetDeg = aDeg; // the CET slider moves in 0.5 deg steps; the weight fade covers on / off, a step is applied as set
    return true;
}

float BodyYawOffset()
{
    std::lock_guard lock(g_m);
    return g_bodyOffsetDeg;
}

float BodyYawOffsetWeight()
{
    std::lock_guard lock(g_m);
    return g_bo.Weight();
}

std::string BodyYawOffsetReason()
{
    std::lock_guard lock(g_m);
    return g_boWhy;
}

bool SetBarrelLoop(bool aOn)
{
    std::lock_guard lock(g_m);
    g_barrelLoop = aOn; // off: MotionPose fades a running loop out (BarrelPitchLoop), then bypasses it
    return true;
}

bool SetBarrelAxis(int32_t aAxis)
{
    if (!aim::ValidAxis(aAxis))
        return false;
    std::lock_guard lock(g_m);
    g_barrelAxis = aAxis;
    return true;
}

bool SetBarrelMuzzle(float aMeters)
{
    if (!(aMeters >= 0.05f && aMeters <= 1.0f)) // NaN fails too
        return false;
    std::lock_guard lock(g_m);
    g_barrelMuzzle = aMeters;
    return true;
}

bool SetLean(bool aOn)
{
    std::lock_guard lock(g_m);
    g_leanSetting = aOn; // off: a running lean fades out (LeanStepLocked blocks it) instead of popping
    return true;
}

bool SetLeanHead(bool aOn)
{
    std::lock_guard lock(g_m);
    g_leanHeadSetting = aOn;
    return true;
}

bool SetLeanParams(float aScale, float aHeadScale)
{
    if (!lrt::ScaleValid(aScale) || !lrt::ScaleValid(aHeadScale))
        return false;
    std::lock_guard lock(g_m);
    g_leanParams.scale = aScale;
    g_leanParams.headScale = aHeadScale;
    return true;
}

bool SetSlide(bool aOn)
{
    std::lock_guard lock(g_m);
    g_slideSetting = aOn;
    RefreshSlideLocked(); RefreshShooterModeLocked(); RefreshTravActiveLocked();
    return true;
}

bool SlideSetting()
{
    std::lock_guard lock(g_m);
    return g_slideSetting;
}

bool SetShooterMode(bool aOn)
{
    std::lock_guard lock(g_m);
    g_shooter_modeSetting = aOn; // no matcher / root / map / body-turn state is touched: the next Step sees the flag
    RefreshShooterModeLocked();
    return true;
}

bool ShooterModeLive()
{
    return g_shooter_modeLive.load(std::memory_order_acquire);
}

float ShooterModeSpeedMul(float aMoveX, float aMoveY, int32_t aDetailed)
{
    if (!g_shooter_modeLive.load(std::memory_order_acquire))
        return 1.0f;
    // only while v2 drives third-person locomotion: not in first person, not while v2 skips (gate, psm release, overlay release, fallback), and
    // only with a fresh feed
    const int64_t driving = g_fnDrivingNs.load(std::memory_order_acquire);
#if UTPP_SHOOTER_MODE_MUTATION != 13
    if (driving == 0 || SteadyNs() - driving > kShooterModeDrivingStaleNs)
        return 1.0f;
#else
    (void)driving;
#endif
#if UTPP_SHOOTER_MODE_MUTATION == 17
    const int gait = -1; // mutation 17: the gait from the stick size, not the one the chooser plays
#else
    const int gait = g_fnLastGait.load(std::memory_order_acquire);
#endif
    return adapter::ShooterModeSpeedRatio(aMoveX, aMoveY, aDetailed, gait);
}

bool SetWalkSpeed(float aMs)
{
    if (!std::isfinite(aMs) || aMs < 0.0f || aMs > adapter::kWalkSpeedMaxMs)
        return false;
    std::lock_guard lock(g_m);
    g_walkSpeed.store(aMs, std::memory_order_release); // the next Step maps it (mc.gameWalkTogglePoint); 0 = off, bit for bit
    RefreshShooterModeLocked();
    return true;
}

float WalkSpeedSetting()
{
    return g_walkSpeed.load(std::memory_order_acquire);
}

float WalkSpeedTarget()
{
    if (!g_walkSpeedLive.load(std::memory_order_acquire))
        return 0.0f;
    // same gate as ShooterModeSpeedMul: only while v2 drives third-person locomotion with a fresh feed
    const int64_t driving = g_fnDrivingNs.load(std::memory_order_acquire);
    if (driving == 0 || SteadyNs() - driving > kShooterModeDrivingStaleNs)
        return 0.0f;
    return g_walkSpeed.load(std::memory_order_acquire);
}

bool SetMinAnalogWalk(float aMs)
{
    if (!std::isfinite(aMs) || aMs < 0.0f || aMs > adapter::kMinAnalogWalkMaxMs)
        return false;
    std::lock_guard lock(g_m);
    g_minAnalogWalk.store(aMs, std::memory_order_release); // the next Step maps it (mc.analogFloorStick); 0 = off, bit for bit
    RefreshShooterModeLocked();
    return true;
}

float MinAnalogWalkSetting()
{
    return g_minAnalogWalk.load(std::memory_order_acquire);
}

bool SetSelectionGuards(bool aOn)
{
    g_selectionGuards.store(aOn, std::memory_order_release); // the next Step applies it to the matcher and the speed map
    return true;
}

bool SelectionGuards()
{
    return g_selectionGuards.load(std::memory_order_acquire);
}

float AnalogFloorMul(float aMoveX, float aMoveY, int32_t aDetailed, bool aWalk)
{
    if (!g_minAnalogWalkLive.load(std::memory_order_acquire))
        return 1.0f;
    if (!std::isfinite(aMoveX) || !std::isfinite(aMoveY) || (aDetailed != adapter::kPsmStand && aDetailed != adapter::kPsmCrouch))
        return 1.0f;
    // same gate as WalkSpeedTarget: only while v2 drives third-person locomotion with a fresh feed
    const int64_t driving = g_fnDrivingNs.load(std::memory_order_acquire);
    if (driving == 0 || SteadyNs() - driving > kShooterModeDrivingStaleNs)
        return 1.0f;
    const float ms = g_minAnalogWalk.load(std::memory_order_acquire);
    const float walk = g_walkSpeed.load(std::memory_order_acquire);
    // walk toggle (stand, walk speed set, no Tier2 Slow - EffectiveMapConfig's walk map): the same m/s against the walk speed
    const bool walkMap = aWalk && walk > 0.0f && walk < adapter::kWalkSpeedMaxMs + 0.001f && !g_tier2Slow.load(std::memory_order_acquire);
    if (walkMap)
        return adapter::AnalogWalkFloorMul(std::hypot(aMoveX, aMoveY), adapter::AnalogFloorFractionWalk(ms, walk), adapter::AnalogFloorMulMaxWalk(walk));
    return adapter::AnalogWalkFloorMul(std::hypot(aMoveX, aMoveY), adapter::AnalogFloorFraction(ms));
}

void FeedTier2Slow(bool aSlow)
{
    g_tier2Slow.store(aSlow, std::memory_order_release);
}

bool Tier2SlowFed()
{
    return g_tier2Slow.load(std::memory_order_acquire);
}

void SetAimJogOff(bool aOff)
{
    g_aimJogOff.store(aOff, std::memory_order_release);
}

void SetHipLease(bool aOn)
{
    std::lock_guard lock(g_m);
    g_hipLease = aOn;
}

bool SetH2Guard(bool aOn)
{
    std::lock_guard lock(g_m);
    g_h2GuardSetting = aOn;
    return true;
}

void SetLocomotionReasonsProbe(uint32_t (*aProbe)())
{
    std::lock_guard lock(g_m);
    g_locomotionProbe = aProbe;
}

#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
StrafeProbe TestStrafeProbe()
{
    std::lock_guard lock(g_m);
    return g_strafeProbe;
}

BarrelProbe TestBarrelProbe()
{
    std::lock_guard lock(g_m);
    return g_barrelProbe;
}

RootProbe TestRootProbe()
{
    std::lock_guard lock(g_m);
    return g_rootProbe;
}
#endif

bool SlideBundleDeclared()
{
    std::lock_guard lock(g_m);
    return g_bundle && g_bundle->slide.declared;
}

bool OwnsSlide(int32_t aPsmDetail) noexcept
{
    return aPsmDetail == adapter::kPsmSlide && g_slideHandling.load(std::memory_order_acquire);
}

bool SlideHandlingActive() noexcept
{
    return g_slideHandling.load(std::memory_order_acquire);
}

void SetShootTransferEnabled(bool aOn)
{
    std::lock_guard lock(g_m);
    ResetAuthoredRecoil();
    g_shootTransferEnabled = aOn;
    g_shootLastReadyValid = g_shootBaseValid = false;
    g_shootEventTicks = 0;
    g_shootHold.Reset();
    g_shootEpisode.Reset();
    if (!aOn) integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
}

void SetWeaponUpper(bool aOn)
{
    std::lock_guard lock(g_m);
    g_weaponUpperOn = aOn;
    if (!aOn) g_weaponUpperFrame = g_weaponUpperTail = false;
}

bool SetOverlayInputs(int32_t aPose, int32_t aAim, bool aRelease, bool aInjured, int32_t aResolvedAimPose)
{
    std::lock_guard lock(g_m);
    const bool poseOk = aPose == 9 || (aPose >= 0 && aPose < static_cast<int32_t>(ov::PoseOverlay::Count));
    const bool aimOk = aAim >= 0 && aAim <= static_cast<int32_t>(ov::AimState::Aiming);
    const bool resolvedOk = aResolvedAimPose == -1 || aResolvedAimPose == 9 ||
        (aResolvedAimPose >= 0 && aResolvedAimPose < static_cast<int32_t>(ov::PoseOverlay::Count));
    g_resolvedAimPose = poseOk && resolvedOk ? (aResolvedAimPose < 0 ? aPose : aResolvedAimPose) : 0;
    if (aRelease || !poseOk || !resolvedOk) { g_hipFirePoseHold.Reset(); g_stationaryShotTurn.Reset(); }
    if (!poseOk || !aimOk || !resolvedOk || aRelease || g_pose != aPose)
    { ResetAuthoredRecoil(); g_reloadContactEpisode.Reset(); }
    if (!poseOk || !aimOk || !resolvedOk || aRelease) g_authoredReload = false;
    g_pose = poseOk ? aPose : 0;
    g_poseMirror.store(g_pose, std::memory_order_release); // vault torso limiter (VehicleVaultWeapon)
    g_aim = aimOk ? aAim : 0;
    // Game (5) = no overlay, game animations own the upper body: same as release
    g_release = aRelease || g_pose == static_cast<int32_t>(ov::PoseOverlay::Game);
    g_injured = aInjured;
    TravRecheckDeferredLocked(); // UTPP-MOTION SEAM next review minor 4
    return poseOk && aimOk && resolvedOk;
}

bool SetWeaponAction(int32_t aAction)
{
    if (aAction != 0 && aAction != 2 && aAction != 8)
        return false;
    std::lock_guard lock(g_m);
    if (aAction == 8 && g_weaponAction != 8 && g_shootTransferEnabled)
    {
        g_shootHold.Reset();
        LARGE_INTEGER now{};
        QueryPerformanceCounter(&now);
        g_shootEventTicks = now.QuadPart;
        g_shootEpisode.Begin(g_shootEventTicks);
        g_shootBase = g_shootLastReady;
        g_shootBaseValid = g_shootLastReadyValid;
    }
    else if (aAction != 8)
    {
        g_shootApplied = false;
        g_shootLastReadyValid = false;
        g_shootBaseValid = false;
        g_shootEventTicks = 0;
        g_shootHold.Reset();
        g_shootEpisode.Reset();
    }
    if (aAction == 2 && g_weaponAction != 2)
        g_reloadContactEpisode.Begin();
    g_weaponAction = aAction;
    if (aAction == 2)
    {
        g_hipFirePoseHold.Reset(); g_stationaryShotTurn.Reset();
#if defined(UTPP_MOTION_DRIVER_RECOIL_EXIT_MUTATION) && UTPP_MOTION_DRIVER_RECOIL_EXIT_MUTATION == 1
        ResetAuthoredRecoil(); // mutation 1: the kick is deleted in the frame the reload starts (pre-fade build)
#else
        g_recoilPending = false; // a shot pulse that did not arrive before the reload is stale; the running kick fades in FrameInputsLocked
#endif
    }
    return true;
}

void SetRecoilProfile(uint32_t aEpoch, int32_t aProfile)
{
    std::lock_guard lock(g_m);
    int32_t profile = aEpoch != 0 && recoil_profile::Valid(aProfile) ? aProfile : recoil_profile::kNone; // unknown ids fail closed
#if defined(UTPP_MOTION_DRIVER_RECOIL_PROFILE_MUTATION) && UTPP_MOTION_DRIVER_RECOIL_PROFILE_MUTATION == 1
    if (profile != recoil_profile::kNue) profile = recoil_profile::kNone; // mutation 1: the per-weapon profile is ignored (pre-profile build)
#endif
    const bool supported = profile != recoil_profile::kNone;
    if (aEpoch != g_recoilWeaponEpoch || profile != g_recoilProfile)
    { ResetAuthoredRecoil(); g_recoilSerial = 0; g_authoredReload = false; g_reloadContactEpisode.Reset(); }
    g_recoilWeaponEpoch = aEpoch; g_recoilSupported = supported; g_recoilProfile = profile;
}

void SetRecoilWeapon(uint32_t aEpoch, bool aSupportedNue)
{
    SetRecoilProfile(aEpoch, aSupportedNue ? recoil_profile::kNue : recoil_profile::kNone);
}

void SetAuthoredReload(bool aOn)
{
    std::lock_guard lock(g_m);
    // The authored reload clip exists for the Nue only; the other profiles are recoil kicks and leave the reload on the game path.
    g_authoredReload = aOn && g_enabled && g_body == Body::Woman && !g_release &&
        g_recoilProfile == recoil_profile::kNue && g_recoilWeaponEpoch != 0;
}

void FeedRecoilShot(uint32_t aEpoch, bool aAiming)
{
    std::lock_guard lock(g_m);
    if (!g_enabled || !g_shootTransferEnabled || !g_recoilSupported ||
        aEpoch == 0 || aEpoch != g_recoilWeaponEpoch || g_release) return;
    // The verified native shot can precede the glue's action8 publication.
    // A short pending pulse is consumed by the next eligible MAIN frame.
    ++g_recoilSerial; g_recoilShotTime = RecoilClock();
    g_recoilAim = aAiming; g_recoilPending = true; g_recoilPendingSteps = 0;
}

bool FeedMagTap()
{
    std::lock_guard lock(g_m);
    ++g_magTapFed;
#if !(defined(UTPP_MAGTAP_MUTATION) && UTPP_MAGTAP_MUTATION == 3) // mutation 3: the setting is ignored (MUST fail "I off")
    if (!g_magTapEnabled)
    {
        ++g_magTapDropped;
        ++g_magTapOff;
        return false;
    }
#endif
    const bool weaponPose = g_pose == static_cast<int32_t>(ov::PoseOverlay::Rifle) ||
        g_pose == static_cast<int32_t>(ov::PoseOverlay::Pistol1H) || g_pose == static_cast<int32_t>(ov::PoseOverlay::Pistol2H);
    if (!g_enabled || g_release || !weaponPose || g_weaponAction == 2)
    {
        ++g_magTapDropped;
        return false;
    }
    if (!g_magTapPending)
    {
        g_magTapDeferred = false; // a second refill while one waits for the slide de-roll keeps that wait (bounded from the first)
        g_magTapWaited = 0.0f;
    }
    g_magTapPending = true; // the next main-path Step decides (MagTapStepLocked)
    g_magTapPendingAge = 0;
    g_magTapPendingTime = RecoilClock();
    return true;
}

void SetMagTapEnabled(bool aOn)
{
    std::lock_guard lock(g_m);
    g_magTapEnabled = aOn;
    // off: an accepted request that has not started yet never plays; a running gesture fades out over its remaining time (no pose
    // jump while the menu option is switched)
    if (!aOn)
    {
        g_magTapPending = false;
        g_magTapPendingAge = 0;
        g_magTapDeferred = false;
        g_magTapWaited = 0.0f;
    }
}

void SetDataDir(const std::string& aDir)
{
    std::lock_guard lock(g_m);
    if (g_dataDir != aDir)
        g_reload = true;
    g_dataDir = aDir;
}

void SetPathsOverride(int32_t aBody, const Paths& aPaths)
{
    if (aBody != kBodyWoman && aBody != kBodyMan)
        return;
    std::lock_guard lock(g_m);
    g_override[aBody] = aPaths;
    g_haveOverride[aBody] = true;
    g_reload = true;
}

void SetLogger(void (*aLog)(const char*))
{
    std::lock_guard lock(g_m);
    g_log = aLog;
}

// UTPP-MOTION SEAM motionfix (item 4)
void SetTrace(bool aOn)
{
    std::lock_guard lock(g_m);
    g_trace = aOn;
    g_traceSince = 1.0e9f; // first traced Step writes a line
    g_traceKey = TraceKey{};
}

#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
void TestThrowInNextStep()
{
    std::lock_guard lock(g_m);
    g_testThrowNextStep = true;
}

void TestKeepRetiredPoses(bool aOn)
{
    std::lock_guard lock(g_m);
    g_testKeepRetired = aOn;
}

size_t TestRetiredPoses()
{
    std::lock_guard lock(g_m);
    return g_testRetired.size();
}

void TestSetFeedTimesUs(float aWallUs, float aThreadUs)
{
    std::lock_guard lock(g_m);
    g_testWallUs = aWallUs;
    g_testThreadUs = aThreadUs;
}

MagTapProbe TestMagTapProbe()
{
    std::lock_guard lock(g_m);
    return g_magTapProbe;
}
#endif

void SetAirFeed(bool aOnGround, float aDistanceToGround, int32_t aPsmFall)
{
    std::lock_guard lock(g_m);
    if (AirMut() == 1)
        return; // mutation 1: the air feed never reaches v2
    g_airIn.valid = true;
    g_airIn.onGround = aOnGround;
    g_airIn.distanceToGround = std::isfinite(aDistanceToGround) ? aDistanceToGround : -1.0f;
    g_airIn.psmFall = aPsmFall;
    g_airFresh = true;
    ++g_airDiag.feeds;
}

void FeedAirFromInput(const motionhost::FeedInput& aIn)
{
    // Dual-wield D0 diagnostics: the raw (unconditioned) psmBlock of every feed, for the status string only. Nothing reads it back.
    g_feedPsmRaw.store(aIn.psmBlock, std::memory_order_relaxed);
    if (!aIn.haveAir || AirMut() == 10)
        return; // mutation 10: the runtime call site forwards nothing
    // mutation 9: onGround not forwarded (always true)
    SetAirFeed(AirMut() == 9 ? true : aIn.onGround, aIn.distToGround, aIn.psmFall);
}

AirDiag AirDiagnostics()
{
    std::lock_guard lock(g_m);
    return g_airDiag;
}

SelectionDiag SelectionDiagnostics()
{
    std::lock_guard lock(g_m);
    return g_selDiag;
}

void OnArmed(uint64_t aRig, const std::vector<uint64_t>& aGameHashes, const std::vector<int16_t>& aGameParents)
{
    std::lock_guard lock(g_m);
    g_fnHoldValid = false;
    g_weaponAction = 0;
    g_hipFirePoseHold.Reset(); g_jumpOutExitHold.Reset(); g_stationaryShotTurn.Reset();
    g_reloadUpperWeight = 0.0f; g_reloadUpperEnv.Reset(); g_reloadRollHold.Reset(); g_slideDeRoll.Reset(); g_rifleSlideLatch.Reset(); g_slideCarryAim.Reset(); ResetMagTap(); g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset(); g_reloadContactsEligible = false;
    g_airLegsFade.Reset();
    g_jumpOutArmsFade.Reset();
    g_jumpOutWindowNow = false;
    g_pending = false;
    g_aimUpperTail = false;
    g_weaponUpperFrame = g_weaponUpperTail = false;
    g_shootRouteNow = false;
    g_shootRouteHeld = g_shootRouteInShot = g_shootRouteLive = false;
    g_wasRunning = false;
    g_hoLeft = 0.0f;
    g_hiLeft = 0.0f;
    g_haveLastBase = false;
    TravResetLocked(true); // UTPP-MOTION SEAM trav
    try
    {
        g_rig = aRig;
        g_gameHashes = aGameHashes;
        g_gameParents = aGameParents;
        RemapLocked();
    }
    catch (...) // hook inside the v1 arm path: no exception may leave it
    {
        g_rig = 0;
        g_map = integ::BoneMapResult{};
        FaultLocked("fault:armed-exception");
    }
}

void OnDisarmed()
{
    std::lock_guard lock(g_m);
    g_fnHoldValid = false;
    g_rig = 0;
    g_gameHashes.clear();
    g_gameParents.clear();
    g_pending = false;
    g_wasRunning = false;
    g_hoLeft = 0.0f;
    g_hiLeft = 0.0f;
    g_haveLastBase = false;
    ResetMagTap();
    TravResetLocked(true); // UTPP-MOTION SEAM trav
    ProbeEndLocked("disarmed");
    GetUpEndLocked("disarmed", true);
    RemapLocked();
    g_slideHandling.store(false, std::memory_order_release); // disarmed: no Step will refresh it, the game owns the slide again
}

namespace
{
// UTPP-MOTION SEAM trav ---------------------------------------------------------------------------------------------------
traversal::V3 TravV(const float* a) { return {a[0], a[1], a[2]}; }

// Overlay machine inputs of a v2 frame (MM pick and traversal frames alike).
struct RootEvaluation
{
    rootframe::Processor *processor = nullptr;
    rootframe::Inputs input;
    rootframe::Proposal proposal;
    bool prepared = false, haveActorAim = false;
    math::Vec3 actorAim{};
    bool aimFromRoot = false; // seam C: the barrel loop target came from the root frame (diagnostics)
};
bool PrepareRootOverlay(void *opaque, const motionpose::BaseFrameView &base, motionpose::OverlayFrameInputs &in)
{
    auto &ctx = *static_cast<RootEvaluation *>(opaque);
    ctx.input.animationDeltaPreviousVisual = {base.signals.rootX * base.dt, base.signals.rootY * base.dt, 0};
    ctx.input.animationYawDelta = base.signals.rootYaw * base.dt;
    ctx.input.rootReliable = base.rootReliable;
    ctx.input.staticTailWeight = base.staticTailWeight;
    if (!ctx.processor->Prepare(ctx.input, ctx.proposal))
        return false;
    if (in.haveAim && ctx.haveActorAim)
    {
        // the aim point is given in the fed actor's axes, the frame is relative to the drawn actor (turned by Ahead)
        math::Vec3 aimPoint = ctx.actorAim;
        if (ctx.proposal.Ahead() != 0.0f)
            math::Rot2(-ctx.proposal.Ahead(), ctx.actorAim.x, ctx.actorAim.y, aimPoint.x, aimPoint.y);
        visualroot::Aim aim;
        if (!visualroot::ComputeAim(ctx.proposal.Frame(), aimPoint, aim))
            return false;
        in.aimYawDeg = aim.yawDeg;
        in.aimPitchDeg = aim.pitchDeg;
        // seam C: the barrel loop target is the same actor point in the same frame as the aim angles above (pose frame)
        math::Vec3 visual;
        in.haveAimPointVisual = visualroot::InversePoint(ctx.proposal.Frame(), aimPoint, visual);
        if (in.haveAimPointVisual)
            in.aimPointVisual = visual;
        ctx.aimFromRoot = in.haveAimPointVisual;
    }
    ctx.prepared = true;
    return true;
}
// test targets: the aim point seam of this evaluation (aCtx = the root evaluation of the live path, nullptr on montage frames)
void RecordBarrelProbeLocked(const motionpose::OverlayFrameInputs& aIn, const motionpose::FrameInfo& aInfo, const RootEvaluation* aCtx)
{
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    BarrelProbe p;
    p.evaluated = true;
    p.rootPath = aCtx && aCtx->aimFromRoot;
    if (p.rootPath)
    {
        const visualroot::Frame fr = aCtx->proposal.Frame();
        p.frame[0] = fr.x;
        p.frame[1] = fr.y;
        p.frame[2] = fr.yaw;
    }
    p.actor[0] = aIn.actorAimPoint.x;
    p.actor[1] = aIn.actorAimPoint.y;
    p.actor[2] = aIn.actorAimPoint.z;
    p.haveVisual = aInfo.haveAimPointVisual;
    p.visual[0] = aInfo.aimPointVisual.x;
    p.visual[1] = aInfo.aimPointVisual.y;
    p.visual[2] = aInfo.aimPointVisual.z;
    p.aimYawDeg = aInfo.aimYawInDeg;
    p.aimPitchDeg = aInfo.aimPitchInDeg;
    p.loop = aIn.barrelLoop;
    p.active = aInfo.barrelActive;
    p.measured = aInfo.barrelMeasured;
    p.errBeforeDeg = aInfo.barrelErrBeforeDeg;
    p.errAfterDeg = aInfo.barrelErrAfterDeg;
    p.biasDeg = aInfo.barrelBiasDeg;
    p.residualDeg = aInfo.barrelResidualDeg;
    p.targetDist = aInfo.barrelTargetDist;
    p.excluded = aInfo.barrelExcluded;
    g_barrelProbe = p;
#else
    (void)aIn;
    (void)aInfo;
    (void)aCtx;
#endif
}
const slide_deroll::Bones& DeRollBonesLocked(const Bundle& aB);
motionpose::OverlayFrameInputs FrameInputsLocked(const Bundle& b, const motion::ChooserState& cs, const adapter::MotionFeed& f, float dt)
{
    const bool reloadDomain = (g_weaponAction == 2 || g_weaponAction == 0) && !g_release &&
        reload_contacts_runtime::ReloadContactsOverlay(g_pose) && f.haveWeapon && f.weaponDrawn && !g_trav.active; // pistols + rifle
    // The episode latches this on the reload edge, so it reads the PSM stance of the feed (ChooserState::stanceRaw), not the 0.15 s
    // debounced cs.stance: a stance change shorter than the filter before the edge would otherwise decide the whole reload.
#if defined(UTPP_RELOAD_EPISODE_STANCE_MUTATION) && UTPP_RELOAD_EPISODE_STANCE_MUTATION == 1
    const bool stationaryReload = cs.stance == 0 && cs.movementState == 0 && cs.movementMode == 0; // mutation 1: the filtered stance
#else
    const bool stationaryReload = cs.stanceRaw == 0 && cs.movementState == 0 && cs.movementMode == 0;
#endif
    // Chooser can select a stop while the capsule is still braking (recorded
    // reload entry: 1.91m/s with released input). Never acquire a different arm
    // solve halfway through that authored action when physical motion ends.
    const bool stationaryEpisode = g_reloadContactEpisode.Observe(g_weaponAction == 2,
        stationaryReload, std::hypot(f.velWorld[0], f.velWorld[1]), reloadDomain,
        g_reloadUpperWeight, g_authoredReload);
    // The CET authored flag can clear before the existing action-0 tail ends.
    // Do not fall back to legacy admission and reacquire the solve during that tail.
    const bool authoredEpisode = g_authoredReload || g_reloadContactEpisode.authoredSeen;
    // Rifle: any ground stance / gait and slide frames, never in the air (ReloadCorrectionPose); pistols: stationaryReload as before.
    // A map slide frame (the de-roll's slide frame) admits a rifle only with a valid de-roll chain: the native hands are never carried
    // in the chest frame of a slide torso that SlideReloadTorsoStepLocked cannot upright (static per bundle).
    const bool rifleOverlay = g_pose == static_cast<int32_t>(ov::PoseOverlay::Rifle);
    const bool correctionPose = reload_contacts_runtime::ReloadCorrectionPose(rifleOverlay,
        stationaryReload, cs.movementMode, b.map.slideActive || adapter::PsmSlide(f.psmDetailed)) &&
        !(rifleOverlay && b.map.slideActive && !DeRollBonesLocked(b).Valid(b.eval.Skel()));
    const bool allowStationaryCorrection = (reload_contacts_runtime::kReloadCorrectionMoving || correctionPose) && (!authoredEpisode || stationaryEpisode);
    g_reloadContactFade.Step(dt, g_reloadUpperWeight, allowStationaryCorrection, reloadDomain, true);
    // Perspective permission is independently checked before Capture and immediately before Finish.
    g_reloadContactsEligible = reloadDomain && g_reloadContactFade.Active();
    g_gripProbe.supplied = g_gripProbe.applied = g_gripProbe.clamped = false;
    g_gripProbe.error = 0.0f;
    g_gripProbe.source = 0; g_gripProbe.weaponShift = 0.0f;
    g_gripProbe.requestedReach = g_gripProbe.usedReach = 0.0f;
    std::fill(std::begin(g_gripProbe.relativeTarget), std::end(g_gripProbe.relativeTarget), 0.0f);
    motionpose::OverlayFrameInputs in;
    in.machine.overlay = OverlayLocked();
    in.machine.style = b.body == Body::Man ? ov::Style::Masculine : ov::Style::Feminine;
    in.machine.crouch = cs.stance == 1;
    in.machine.moving = cs.movementState == 1;
    in.machine.hasVelocity = b.map.st.hasVelocity;
    in.machine.wantsToAim = g_aim == static_cast<int32_t>(ov::AimState::Aiming) || f.aiming;
    in.machine.wantsReady = g_aim == static_cast<int32_t>(ov::AimState::Ready);
    in.machine.wantsToSprint = adapter::PsmSprint(f.psmDetailed); // motionfix item 7: CrouchSprint 30 = Stand + Sprint
    in.machine.sprinting = cs.gait == 2;
    // fix2/hip P6 K1: only the real air (chooser MovementMode InAir = 1) fades the leg layer of the weapon pose. The slide and its
    // air grace are mode 2 and keep the pose they have today. A traversal montage does NOT own the legs: the overlay (leg layer
    // included) still composes over it at overlayWeight 1 after the base inertializer, so a jump -> climb / vault hands the weight
    // over like a landing (hold, then fall) instead of resetting it, which would pop the legs from the clip to the weapon stance
    // in one frame.
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 22
    in.machine.airW = g_airLegsFade.Step(cs.movementMode != 0, dt); // mutation 22: the slide (mode 2) also fades the legs
#elif defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 24
    if (g_trav.active) // mutation 24: the weight is reset on the first montage frame (pop of the leg layer)
    {
        g_airLegsFade.Reset();
        in.machine.airW = 0.0f;
    }
    else
        in.machine.airW = g_airLegsFade.Step(cs.movementMode == 1, dt);
#else
    in.machine.airW = g_airLegsFade.Step(cs.movementMode == 1 && !g_trav.active, dt);
#endif
    in.capturePoseDiagnostics = g_trace; // selected bone deltas are emitted only by the existing low-rate trace
    // UTPP-MOTION SEAM motionfix item 2: aim offset inputs (target of UTPPSetAimTarget); skipped while the upper body is
    // handed back to the game (mm.aimGameUpper)
    // Native ADS remains the sole input for game-upper handback and camera-related state.
    g_aimingNow = in.machine.wantsToAim;
    // A reload that interrupts ADS: the game drops its aim flag at once, which would pop the MM arms while they still
    // carry ~full weight. Keep the overlay aim state (visual only: g_aimingNow above stays the real flag) until the arms
    // are fully native. Not held while the player asks for a sprint (the overlay machine would show an aimed pose in
    // a sprint); other native-upper owners (aimGameUpper, weaponUpper) read g_aimingNow, not this overlay request.
    bool reloadAimHeld = false;
    if ((g_reloadUpperEnv.AimHold() || (g_reloadUpperEnv.fromAim && g_reloadRollHold.hold > 0.0f)) &&
        g_weaponAction == 2 && !g_release && !in.machine.wantsToSprint && !in.machine.sprinting)
    {
        in.machine.wantsToAim = true;
        reloadAimHeld = true;
    }
    const auto resolvedAim = g_resolvedAimPose == 9 ? ov::PoseOverlay::Injured :
        static_cast<ov::PoseOverlay>(g_resolvedAimPose);
    const bool hipFirePose = ov::UseHipFirePose(g_aimingNow, g_weaponAction, g_hipFirePoseHold.Active(),
        g_release, f.haveWeapon, f.weaponDrawn, in.machine.overlay, resolvedAim);
    if (hipFirePose)
    {
        in.machine.overlay = resolvedAim;
        in.machine.wantsToAim = true;
        in.machine.wantsReady = true;
        in.machine.hipFireRaise = true; // P9 C: the shot (and its hold) raises the weapon from Ready in 0.1 s, real ADS keeps 0.2 s;
                                        // HIP-1: from Relaxed in one eased 0.25 s raise (OverlayTimes::hipFireRelaxedToAiming)
        // Overlay pose only: actual chooser gait/velocity and Locomotion inputs remain untouched.
        in.machine.wantsToSprint = false;
        in.machine.sprinting = false;
    }
    g_hipFirePoseNow = hipFirePose;
    // r9: a vehicle jump-out flight (window open, really airborne, no traversal montage) takes the arm / hand layers off the weapon pose.
    // 2026-11-04 r11: the weapon request is read here, after the reload aim hold and the hip-fire override above, from the real inputs (native ADS,
    // reload, shot, the ADS reload hold): a hip shot or a reload in the flight keeps the weapon arms (the weight falls in 0.15 s), plain flight fades them.
    // The synthetic exit-window hold of the hip-fire pose (released in the flight) is not a request.
    const bool jumpOutWeaponRequest = g_aimingNow || g_weaponAction == 2 || g_weaponAction == 8 || reloadAimHeld;
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 26
    in.machine.jumpOutArmW = g_jumpOutArmsFade.Step(false && g_jumpOutWindowNow && cs.movementMode == 1 && !g_trav.active, dt, jumpOutWeaponRequest); // mutation 26: the jump-out window never reaches the arm fade
#elif defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 27
    in.machine.jumpOutArmW = g_jumpOutArmsFade.Step(cs.movementMode == 1 && !g_trav.active, dt, jumpOutWeaponRequest); // mutation 27: every flight (a normal weapon jump too) fades the arms
#elif defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 28
    in.machine.jumpOutArmW = g_jumpOutArmsFade.Step(g_jumpOutWindowNow && cs.movementMode == 1 && !g_trav.active, dt); (void)jumpOutWeaponRequest; // mutation 28: aiming / a shot / a reload in the flight fade the arms too
#elif defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 29
    in.machine.jumpOutArmW = g_jumpOutArmsFade.Step(g_jumpOutWindowNow && !g_aimingNow && cs.movementMode == 1 && !g_trav.active, dt); (void)jumpOutWeaponRequest; // mutation 29: pre-r11, only native ADS keeps the arms (a hip shot / a reload fade them)
#else
    in.machine.jumpOutArmW = g_jumpOutArmsFade.Step(g_jumpOutWindowNow && cs.movementMode == 1 && !g_trav.active, dt, jumpOutWeaponRequest);
#endif
    const bool visualAiming = in.machine.wantsToAim;
    g_weaponUpperFrame = g_weaponUpperOn && f.haveWeapon && f.weaponDrawn && !g_aimingNow && !hipFirePose;
    const bool handback = (g_aimGameUpper && g_aimingNow) || g_weaponUpperFrame;
    // Keep the corrected authored idle base stable underneath native action/air ownership.
    // Reload/Shoot handback blends independently; toggling style with action or ground bits would snap.
    in.bodyWoman = b.body == Body::Woman;
    // Male V: same relaxed-1H idle stance policy as the female V (offline ablation 2026-11-06), without the feminine shoulder pass.
    in.maleIdlePistolStance = b.body == Body::Man;
    // Pistol1H / Relax / standing idle scope and movement/aim continuity use existing pose weights.
    const float *actorTarget =
        g_aimTgtOn && g_aimTgtValid
            ? g_aimTgt
            : (visualAiming && g_cameraTargetValid && RecoilClock() - g_cameraTargetTime <= .25 ? g_cameraTarget
                                                                                                : nullptr);
    if (actorTarget && !handback)
    {
        in.haveAim = AimAnglesFromModel(actorTarget, in.aimYawDeg, in.aimPitchDeg);
        in.haveActorAimPoint = in.haveAim;
        in.actorAimPoint = {actorTarget[0], actorTarget[1], actorTarget[2]};
    }
    // Fix batch 2 (AO): a lost target (haveAim false) no longer ends the aim offset in one frame, MotionPose fades the latched
    // angles out with the Aiming weight. The game-owned upper body (handback) fades the latch out in 0.15 s (not in one frame).
    in.aoSuppress = handback;
    // Fix batch 2 (AO) x reload contacts: while the game holds the arms (the reload envelope: the 0.12 / 0.30 s entry, the reload and
    // its 0.20 s exit tail, as long as ApplyLocked hands the upper body over: reloadHandback) the AO goes down with the MM share of
    // the upper body (the expression of reloadScale in ApplyLocked), so the contacts that put the hands on the native model positions
    // never meet a torso that the AO keeps tilting; the roll hold of a slide reload keeps the MM arms and with them the AO. A gate
    // on the reload action alone would let the AO come back in one frame while the arms are still returning over 0.2 s.
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 13
    if (g_weaponAction == 2 && !g_release && g_reloadUpperEnv.active) // mutation 13: the first gate, no exit tail
#else
    if (!g_release && g_reloadUpperWeight > 0.0f)
#endif
        in.aoScale = motionpose::AoReloadScale(g_authoredReload ? reload_contacts_runtime::AuthoredFadeWeight(g_reloadUpperWeight)
                                                           : g_reloadUpperWeight,
                                          g_reloadRollHold.hold);
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 8
    in.haveAim = false; // mutation 8: aim target not forwarded to the evaluator
#endif
    // Aim pitch plan 2026-10-30 seam C: the barrel loop target P in the pose frame. Without a root frame (Quintic base, montage frames,
    // no root evaluation) the pose frame is the actor frame, the same one AimAnglesFromModel used above; PrepareRootOverlay replaces it
    // with the root-frame point on the live path. Loop off: MotionPose bypasses the loop, these fields change nothing.
    in.barrelLoop = g_barrelLoop;
    in.barrelAxis = g_barrelAxis;
    in.barrelMuzzle = g_barrelMuzzle;
    in.haveAimPointVisual = in.haveAim && in.haveActorAimPoint;
    if (in.haveAimPointVisual)
        in.aimPointVisual = in.actorAimPoint;
    // Rifle and two-handed pistol overlays author their own left-hand contact. Their weapon-relative
    // grip must follow that pose; an FPP game-pose target can be far from the TPP overlay weapon.
    const bool gripWanted = g_gripEnabled && !handback && f.haveWeapon && f.weaponDrawn &&
        (in.machine.overlay == ov::PoseOverlay::Rifle || in.machine.overlay == ov::PoseOverlay::Pistol2H);
    // Pistol1H can still be fading out of Pistol2H. Let the evaluator inspect the composed pose
    // before releasing authored contact, instead of dropping it on the class switch.
    in.authoredTwoHandGrip = motionpose::MayUseAuthoredTwoHandGrip(
        g_gripEnabled, handback, f.haveWeapon, f.weaponDrawn, in.machine.overlay);
    const bool gameGripWanted = gripWanted && !in.authoredTwoHandGrip;
    // rifle-grip-stock 2026-11-10: the equipped long gun's game grip on the handguard + stock contact (mm/SupportGrip.hpp, weapon identity =
    // the recoil profile id of the equipped record). Rifle overlay only; MotionPose applies it on the authored two-hand route and fades it out.
    // (the "build before" mutation is UTPP_SUPPORTGRIP_MUTATION=1 of utpp_supportgrip_mut1: the whole pass compiled out of PoseEvaluator)
    in.supportGrip = g_recoilWeaponEpoch != 0 && in.machine.overlay == ov::PoseOverlay::Rifle ? sgrip::Of(g_recoilProfile) : nullptr;
    // The v2 slide feeds the chooser MovementMode Sliding (2, b.map.slideActive on that feed), which is neither 0 nor 1.
    // Leaving it out of the domain reset the authored recoil on every slide shot and handed hipfire to the native action
    // fallback (upperScale 0, FPP arms) for the whole shot.
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 21
    const bool slideRecoil = false; // mutation 21: the slide is outside the authored recoil domain (pre-fix)
#else
    const bool slideRecoil = b.map.slideActive;
#endif
    // The recoil profile of the equipped weapon (RecoilProfiles.hpp) decides the domain, not the overlay class: Rifle joins the
    // pistol overlays for every weapon whose profile files loaded. A weapon without a (valid) profile keeps the plain authored aim.
    const bool profileReady = recoil_profile::Valid(g_recoilProfile) && b.recoilOk[static_cast<size_t>(g_recoilProfile)];
    const bool authoredDomain = profileReady && g_recoilSupported && g_recoilWeaponEpoch &&
        g_shootTransferEnabled && g_gripEnabled && !handback && !g_release &&
        (in.machine.overlay == ov::PoseOverlay::Pistol1H || in.machine.overlay == ov::PoseOverlay::Pistol2H ||
         in.machine.overlay == ov::PoseOverlay::Rifle) &&
        // Weapon-local recoil composes over the current aiming pose in both stances, in air and in the v2 slide.
        // Dropping the route at takeoff would return hipfire arms to FPP while the torso stays TPP.
        // Keep traversal excluded and the legacy native FK capture below standing-only.
        f.haveWeapon && f.weaponDrawn && (cs.stance == 0 || cs.stance == 1) &&
        (cs.movementMode == 0 || cs.movementMode == 1 || slideRecoil) && !g_trav.active &&
        (g_weaponAction == 0 || g_weaponAction == 8);
    const double recoilNow = RecoilClock();
    // What can end a kick at once (it is not a movement of the player): unready or changed weapon, release, TPP off, hand-back to the game.
    const bool hardDomain = profileReady && g_recoilSupported && g_recoilWeaponEpoch && g_shootTransferEnabled && g_gripEnabled &&
        !handback && !g_release && f.haveWeapon && f.weaponDrawn &&
        (in.machine.overlay == ov::PoseOverlay::Pistol1H || in.machine.overlay == ov::PoseOverlay::Pistol2H ||
         in.machine.overlay == ov::PoseOverlay::Rifle);
    bool recoilExiting = false;
    if (!authoredDomain)
    {
#if defined(UTPP_MOTION_DRIVER_RECOIL_EXIT_MUTATION) && UTPP_MOTION_DRIVER_RECOIL_EXIT_MUTATION == 1
        ResetAuthoredRecoil(); // mutation 1: pre-fade build
#else
        if (!hardDomain || !g_authoredRecoil.Active()) ResetAuthoredRecoil();
        else
        {
            g_recoilPending = false;
            g_recoilExitW -= static_cast<float>(std::clamp(recoilNow - g_recoilExitT, 0.0, 0.1)) / kRecoilExitSeconds;
            g_recoilExitT = recoilNow;
            if (g_recoilExitW <= 0.0f) ResetAuthoredRecoil();
            else recoilExiting = true;
        }
#endif
    }
    else if (g_recoilExitW < 1.0f)
    {
        // back in the domain (a micro-step, a one-frame mode flicker): the weight recovers at the same rate instead of snapping
        g_recoilExitW = std::min(1.0f, g_recoilExitW + static_cast<float>(std::clamp(recoilNow - g_recoilExitT, 0.0, 0.1)) / kRecoilExitSeconds);
        g_recoilExitT = recoilNow;
    }
    else g_recoilExitT = recoilNow;
    // The shot pulse is valid for 0.1 s of wall time, but a frame hitch (or a loaded machine) can delay the first Step after the shot
    // past that. A pulse dropped unseen leaves the shot to the native action fallback (upperScale 0, FPP arms for the whole action),
    // so it survives its first two Steps whatever the wall time says (a stale pulse still dies after 0.5 s).
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 25
    if (g_recoilPending && recoilNow - g_recoilShotTime > .1) g_recoilPending = false; // mutation 25: wall time only (pre-fix)
#else
    if (g_recoilPending)
    {
        const double pendingAge = recoilNow - g_recoilShotTime;
        if (pendingAge > .5 || (pendingAge > .1 && g_recoilPendingSteps >= 2)) g_recoilPending = false;
        else ++g_recoilPendingSteps;
    }
#endif
    const bool authoredRoute = recoilExiting || (authoredDomain && (g_recoilPending || g_authoredRecoil.Active() ||
                                                                    (g_recoilAdmitted && g_weaponAction == 8)));
    g_recoilOwns = authoredRoute;
    const bool shootEligible = g_shootTransferEnabled && g_gripEnabled && !handback && !g_release &&
        in.machine.overlay == ov::PoseOverlay::Pistol2H && visualAiming && f.haveWeapon && f.weaponDrawn &&
        cs.stance == 0 && cs.movementState == 0 && cs.movementMode == 0 && !g_trav.active &&
        (g_weaponAction == 0 || g_weaponAction == 8);
    // The route exists only where an authored recoil episode (a loaded profile) can own the shot. The legacy native FK transfer
    // (shootEligible) is still attempted (shootPrepare / shootAction below) but is NOT a route: for a Pistol2H without a profile
    // (the retired Overture / Nova / Burya revolvers) it applies on a handful of frames at best, and a route that waits for it handed
    // the arms to the native shot (upperScale 0, hip-level support arm, 93-142 degrees in one frame) for the whole 0.3 s shot.
#if defined(UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION) && UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION == 3
    g_shootRouteLive = authoredDomain || shootEligible; // mutation 3: the legacy native FK stance is a route (the revolver hand-over)
#else
    g_shootRouteLive = authoredDomain;
#endif
    if (g_weaponAction == 8)
    {
        g_shootRouteHeld = g_shootRouteInShot ? (g_shootRouteHeld && g_shootRouteLive) : g_shootRouteLive;
        g_shootRouteInShot = true;
    }
    else
    {
        g_shootRouteInShot = false;
        g_shootRouteHeld = g_shootRouteLive;
    }
    g_shootRouteNow = g_shootRouteHeld;
#if defined(UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION) && UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION == 1
    g_shootRouteNow = true; // mutation 1: pre-P9, every weapon keeps the native first-burst fallback (FPP arms, upperScale 0)
#endif
    in.shootPrepare = shootEligible && (g_weaponAction == 0 || (hipFirePose && g_aoW < 1.0f - ov::kZeroWeight));
    in.shootAction = shootEligible && g_weaponAction == 8;
    // Native pre-shot reference can be captured in idle without forcing native ADS or an authored pose.
    const bool preShotCapture = g_shootTransferEnabled && g_gripEnabled && !g_release &&
        resolvedAim == ov::PoseOverlay::Pistol2H && f.haveWeapon && f.weaponDrawn &&
        cs.stance == 0 && cs.movementState == 0 && cs.movementMode == 0 && !g_trav.active && g_weaponAction == 0;
    integ::g_gamePoseCaptureWanted.store(gameGripWanted || shootEligible || preShotCapture, std::memory_order_relaxed);
    if ((shootEligible || preShotCapture) && !authoredRoute)
    {
        integ::GamePoseSample sample;
        integ::PoseSnapshot readMap;
        LARGE_INTEGER now{}, frequency{};
        QueryPerformanceCounter(&now);
        frequency.QuadPart = util::QpcFrequency();
        const bool mapReady = integ::g_gamePoseReadMap.Read(readMap);
        const bool sampleReady = integ::g_gamePoseSamples.Read(sample);
        const bool fresh = mapReady && sampleReady &&
            integ::FreshGamePoseSample(sample, g_rig, readMap.sequence, static_cast<uint32_t>(b.pose.size()),
                                      now.QuadPart, frequency.QuadPart / 10);
        const bool nativeReadyCapture = ov::CaptureNativeReadyReference(
            g_weaponAction, in.shootPrepare, preShotCapture);
        if (nativeReadyCapture)
        {
            g_shootHold.Reset();
            g_shootEpisode.Reset();
            g_shootLastReadyValid = fresh;
            if (fresh) g_shootLastReady = sample;
        }
        else
        {
            // Authored Aim preparation and native recoil admission are independent.
            // Keep the event's pre-shot sample while the authored arms blend into Aim.
            bool freshDelta = false;
            ov::Xf candidateDelta{};
            if (g_shootBaseValid && fresh &&
                 native_shoot::AdmitSamples(g_shootBase, sample, g_rig, readMap.sequence,
                                            static_cast<uint32_t>(b.pose.size()), g_shootEventTicks,
                                            now.QuadPart, frequency.QuadPart / 10))
            {
                static ov::Pose nativeBase, nativeNow;
                BoneTransform localBase[integ::kMaxMMBones], localNow[integ::kMaxMMBones];
                for (uint32_t i = 0; i < sample.count; ++i)
                {
                    const float* before = g_shootBase.pose + i * 7;
                    const float* after = sample.pose + i * 7;
                    localBase[i] = {{before[0], before[1], before[2]}, {before[3], before[4], before[5], before[6]}};
                    localNow[i] = {{after[0], after[1], after[2]}, {after[3], after[4], after[5], after[6]}};
                }
                b.eval.ToOv(localBase, nativeBase);
                b.eval.ToOv(localNow, nativeNow);
                freshDelta = native_shoot::WeaponDelta(b.eval.Skel(), nativeBase, nativeNow,
                                                       b.eval.GripRig(), candidateDelta);
            }
            const bool identityOk = mapReady && g_shootBaseValid && g_shootBase.rig == g_rig &&
                g_shootBase.generation == readMap.sequence &&
                g_shootBase.count == b.pose.size() && g_shootBase.ticks > 0 &&
                g_shootBase.ticks < g_shootEventTicks &&
                g_shootEventTicks - g_shootBase.ticks <= frequency.QuadPart / 10 &&
                (!sampleReady || (sample.rig == g_rig && sample.generation == readMap.sequence &&
                                  sample.count == b.pose.size()));
            const bool awaitingSample = identityOk && (!sampleReady || sample.ticks <= g_shootEventTicks);
            float admissionWeight = 0.0f;
            const bool episodeOk = g_shootEpisode.Accept(freshDelta && identityOk, awaitingSample,
                now.QuadPart, frequency.QuadPart / 10, frequency.QuadPart / 20, admissionWeight);
            in.shootDeltaValid = episodeOk && g_shootHold.Resolve(identityOk, freshDelta, candidateDelta,
                                                     g_rig, readMap.sequence, now.QuadPart,
                                                     frequency.QuadPart / 10, in.shootDelta,
                                                     &in.shootWeight);
            in.shootWeight *= admissionWeight;
            if (episodeOk && !in.shootDeltaValid) g_shootEpisode.Reset();
        }
    }
    else
    {
        g_shootHold.Reset();
        g_shootEpisode.Reset();
        if (g_weaponAction != 8) g_shootLastReadyValid = false;
    }
    g_recoilEffT = g_recoilEffR = 0.0f;
    if (authoredRoute)
    {
        integ::PoseSnapshot map;
        const bool mapped = integ::g_gamePoseReadMap.Read(map) && map.rig == g_rig && map.sequence != 0;
        if (mapped)
        {
            const authored_recoil::Binding binding{g_recoilWeaponEpoch, g_rig, map.sequence};
            if (g_recoilPending && (g_weaponAction == 8 || g_aimingNow || g_hipFirePoseHold.Active()))
            {
                const int selected = g_recoilAim ? 1 : 0;
                // a hip <-> ADS switch inside a burst keeps the running kick (Player::Trigger carries it), so it keeps its admission too
                if (!g_authoredRecoil.Active()) g_recoilAdmitted = false;
                g_recoilClip = selected;
                g_authoredRecoil.Trigger(b.recoil[static_cast<size_t>(g_recoilProfile)][g_recoilClip], binding, g_recoilSerial, recoilNow,
                                         recoil_profile::LimitsOf(g_recoilProfile));
                g_recoilPending = false;
            }
            in.shootDeltaValid = g_authoredRecoil.Sample(b.recoil[static_cast<size_t>(g_recoilProfile)][g_recoilClip], binding, recoilNow,
                                                         in.shootDelta);
            // The 1H evaluator can defer until the outgoing 2H pose has faded.
            // Start activation at first actual admission, never at an old shot's age.
            const float u = g_recoilAdmitted ? std::clamp(float((recoilNow - g_recoilAdmitTime) / .05), 0.0f, 1.0f) : 0.0f;
            const float admissionWeight = g_recoilAdmitted ? std::max(.001f, u * u * (3.0f - 2.0f * u)) : .001f;
            const float exitW = std::clamp(g_recoilExitW, 0.0f, 1.0f);
            in.shootWeight = admissionWeight * g_authoredRecoil.TailWeight(recoilNow) * (exitW * exitW * (3.0f - 2.0f * exitW));
            const ov::Xf effective = in.shootDeltaValid ? ov::BlendXf(ov::Xf{}, in.shootDelta, in.shootWeight) : ov::Xf{};
            g_recoilEffT = authored_recoil::Translation(effective); g_recoilEffR = authored_recoil::Rotation(effective);
        }
        in.shootPrepare = true; in.shootAction = true;
        in.shootOneHand = in.machine.overlay == ov::PoseOverlay::Pistol1H;
    }
    if (gameGripWanted)
    {
        static ov::Pose gamePose;
        static gik::Grip leftGrip;
        integ::GamePoseSample sample;
        integ::PoseSnapshot readMap;
        LARGE_INTEGER now{}, frequency{};
        QueryPerformanceCounter(&now);
        frequency.QuadPart = util::QpcFrequency();
        if (integ::g_gamePoseReadMap.Read(readMap) && integ::g_gamePoseSamples.Read(sample) &&
            integ::FreshGamePoseSample(sample, g_rig, readMap.sequence, static_cast<uint32_t>(b.pose.size()),
                                      now.QuadPart, frequency.QuadPart / 10))
        {
            BoneTransform local[integ::kMaxMMBones];
            for (uint32_t i = 0; i < sample.count; ++i)
            {
                const float* p = sample.pose + i * 7;
                local[i].p = {p[0], p[1], p[2]};
                local[i].q = {p[3], p[4], p[5], p[6]};
            }
            b.eval.ToOv(local, gamePose);
            leftGrip = gik::CacheGripFromGamePose(b.eval.Skel(), gamePose, b.eval.GripRig());
            if (leftGrip.valid)
            {
                in.leftGrip = &leftGrip;
                g_gripProbe.supplied = true;
                g_gripProbe.source = 1;
                g_gripProbe.relativeTarget[0] = leftGrip.t.x;
                g_gripProbe.relativeTarget[1] = leftGrip.t.y;
                g_gripProbe.relativeTarget[2] = leftGrip.t.z;
                ++g_gripProbe.accepted;
            }
        }
        if (!in.leftGrip) ++g_gripProbe.rejected;
    }
    return in;
}

// UTPP-MOTION SEAM motionfix item 6: both hands on the weapon in the pose just evaluated: a weapon overlay in Ready / Aiming, one
// pose key + one aim state + one base pose carrying the weight (no crossfade), LeftHand within kOverlayArmsMaxHandDist of WeaponRight.
bool OverlayArmsOf(const Bundle& b, const motionpose::FrameInfo& aInfo)
{
    const ov::OverlayMachine& m = b.eval.Machine();
    const bool weapon = m.overlay == ov::PoseOverlay::Rifle || m.overlay == ov::PoseOverlay::Pistol1H ||
                        m.overlay == ov::PoseOverlay::Pistol2H;
    const bool aimed = m.aim == ov::AimState::Ready || m.aim == ov::AimState::Aiming;
#if defined(UTPP_MOTION_DRIVER_OVARMS_MUTATION) && UTPP_MOTION_DRIVER_OVARMS_MUTATION == 1
    return weapon; // mutation 1: every weapon overlay frame claims both hands
#else
    return weapon && aimed && aInfo.layered && aInfo.overlaySteady && aInfo.leftHandToWeapon >= 0.0f &&
           aInfo.leftHandToWeapon <= motionpose::kOverlayArmsMaxHandDist;
#endif
}

// TRAV-1 D (vehicle batch): one always-on info line per traversal event (the plan is made once per event; a bundle swap that
// re-plans the same event writes nothing), so a release recording tells owned / released / held apart without mm.trace:
// "UTPP trav: action= h= ext= depth= back= v= kind= decision= result=<StartResult>(<reason>) montage=". depth = obstacleEnd -
// topPoint along the facing (the TRAV-1 C rule), back = topPoint.z - landing z. Fixed buffer, host logger (info level).
uint64_t g_travInfoEvent = 0; // g_travProbe.events of the last event that wrote its line
void TravPlanInfoLocked(const motiontrav::GameObstacle& aO, const motiontrav::CapsuleState& aC, const motiontrav::Plan& aP, bool aTravOk)
{
    std::snprintf(g_travProbe.reason, sizeof g_travProbe.reason, "%s", aP.reason ? aP.reason : "");
    g_travProbe.kind = aO.kind;
    if (g_log == nullptr || g_travInfoEvent == g_travProbe.events)
        return;
    g_travInfoEvent = g_travProbe.events;
    const double fx = -aO.collisionNormal.x, fy = -aO.collisionNormal.y, fl = std::hypot(fx, fy);
    const double depth = fl > 1e-9 ? ((aO.obstacleEnd.x - aO.topPoint.x) * fx + (aO.obstacleEnd.y - aO.topPoint.y) * fy) / fl : 0.0;
    const char* decision = !aP.decision.actionName.empty() ? aP.decision.actionName.c_str() : "-";
    char line[320];
    std::snprintf(line, sizeof line,
                  "UTPP trav: action=%d h=%.2f ext=%.2f depth=%.2f back=%.2f v=%.1f kind=%d decision=%s result=%s(%s) montage=%s%s",
                  aO.action == motiontrav::GameAction::Vault ? 9 : 8, aP.obstacleHeight, aO.topExtent, depth,
                  aO.topPoint.z - aP.landingZ, aC.speed, aO.kind, decision, motiontrav::ToString(aP.result),
                  aP.reason && aP.reason[0] ? aP.reason : "-", aP.montage ? aP.montage->name.c_str() : "-",
                  aTravOk ? "" : " (no traversal data)");
    ++g_travProbe.infoLines;
    g_log(line);
}

// The plan of the pending event (FeedTraversal when the bundle is ready, else the next Step that reaches the traversal code):
// Ok -> planned + planOk, v2 owns the traversal (OwnsTraversal) until the episode ends; anything but Ok -> planned, not Ok:
// the episode yields (TravStartLocked).
void TravPlanLocked(Bundle& b)
{
    g_trav.havePending = false;
    ++g_travProbe.plans;
    const TraversalEvent& e = g_trav.ev;
    motiontrav::GameObstacle o;
    o.action = e.action == 9 ? motiontrav::GameAction::Vault : motiontrav::GameAction::Climb;
    o.topPoint = TravV(e.topPoint);
    o.collisionNormal = TravV(e.collisionNormal);
    o.obstacleEnd = TravV(e.obstacleEnd);
    o.behindPoint = TravV(e.behindPoint);
    o.inputDirection = TravV(e.inputDirection);
    o.topExtent = e.topExtent;
    o.hasHands = e.hasHands;
    o.leftHand = TravV(e.leftHand);
    o.rightHand = TravV(e.rightHand);
    o.kind = e.obstacleKind; // TRAV-1 B (vehicle batch)
    motiontrav::CapsuleState c;
    c.pos = TravV(e.playerPos);
    c.yaw = std::atan2(-static_cast<double>(e.ownerForward[0]), static_cast<double>(e.ownerForward[1])); // forward = (-sin, cos)
    c.speed = e.speed;
    c.airborne = e.airborne;
    c.climbSpeedModifier = e.climbSpeedModifier;
    motiontrav::Config cfg;
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    cfg.forceMontage = g_testForceMontage;
    cfg.gate = g_testTravGate;
#endif
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 2
    cfg.centreRootWithoutHands = false; // mutation 2: pre-review hand-less targets (hands on topPoint)
#endif
    g_trav.plan = motiontrav::Plan{};
    // review minor 5: the live tweaks of the event (each value falls back to the 2.31 data when missing / non-finite / <= 0)
    motiontrav::GameTweaks tw;
#if UTPP_MOTION_DRIVER_TRAV_MUTATION != 3 // mutation 3: GameTweaks{} instead of the fed tweaks
    if (e.haveTweaks)
    {
        auto use = [](double& aDst, float aV) {
            if (std::isfinite(aV) && aV > 0.0f)
                aDst = aV;
        };
        use(tw.highThreshold, e.highThreshold);
        use(tw.midThreshold, e.midThreshold);
        for (int k = 0; k < 3; ++k)
        {
            use(tw.verticalDuration[k], e.verticalDuration[k]);
            use(tw.horizontalDuration[k], e.horizontalDuration[k]);
        }
        use(e.action == 9 ? tw.vaultForwardStep : tw.climbForwardStep, e.forwardStep);
    }
#endif
    if (b.travOk)
        g_trav.plan = motiontrav::PlanTraversal(b.travTable, b.travPlay, o, c, tw, cfg);
    else
        g_trav.plan.result = motiontrav::StartResult::MontageFailed; // no traversal data for this body
    const motiontrav::Plan& p = g_trav.plan;
    g_travProbe.lastResult = static_cast<int32_t>(p.result);
    g_vehVaultEvt.store(o.action == motiontrav::GameAction::Vault && o.kind == 1 && p.result != motiontrav::StartResult::Ok, std::memory_order_release); // vault torso limiter
    g_travProbe.startFrame = p.startFrame;
    g_travProbe.endFrame = p.endFrame;
    g_travProbe.maxPlanStep = static_cast<float>(p.gatePlanarStep);
    g_travProbe.maxWarpAdded = static_cast<float>(p.gateWarpAdded);
    g_travProbe.climbClass = static_cast<int32_t>(p.climbClass);
    g_travProbe.gameDuration = static_cast<float>(p.gameDuration);
    g_travProbe.destination[0] = static_cast<float>(p.gameDestination.x);
    g_travProbe.destination[1] = static_cast<float>(p.gameDestination.y);
    g_travProbe.destination[2] = static_cast<float>(p.gameDestination.z);
    const ptrdiff_t idx = p.montage ? p.montage - b.travPlay.montages.data() : -1;
    g_trav.planned = true;
    g_trav.planOk = p.result == motiontrav::StartResult::Ok && idx >= 0 && static_cast<size_t>(idx) < b.travPlayClip.size();
    TravPlanInfoLocked(o, c, p, b.travOk);
    if (!g_trav.planOk)
    {
        ++g_travProbe.yields;
        g_travProbe.yielding = true;
        TravOwnLocked(false);
        return;
    }
    g_trav.clip = b.travPlayClip[static_cast<size_t>(idx)];
    ++g_travProbe.accepted;
    g_travProbe.yielding = false;
    g_travProbe.clip = g_trav.clip;
    g_travProbe.fromPoseFile = g_trav.clip < 0;
    std::snprintf(g_travProbe.montage, sizeof g_travProbe.montage, "%s", p.montage->name.c_str());
    // FrontLedge window of the plan (windows 0..n-1 = the montage's baked windows in order, MakeWarpWindows)
    g_travProbe.frontEndFrame = -1;
    for (size_t k = 0; k < p.montage->windows.size() && k < p.windows.size(); ++k)
        if (p.montage->windows[k].target == "FrontLedge")
        {
            const traversal::WarpWindow& w = p.windows[k];
            const traversal::RootKey& r =
                p.warped[static_cast<size_t>(std::clamp(w.endFrame, 0, static_cast<int>(p.warped.size()) - 1))];
            const traversal::V3 hc = r.pos + traversal::RotZ(p.windowOffsets[k], r.yaw);
            g_travProbe.frontEndFrame = w.endFrame;
            g_travProbe.handTarget[0] = static_cast<float>(p.windowTargets[k].x);
            g_travProbe.handTarget[1] = static_cast<float>(p.windowTargets[k].y);
            g_travProbe.handTarget[2] = static_cast<float>(p.windowTargets[k].z);
            g_travProbe.handContact[0] = static_cast<float>(hc.x);
            g_travProbe.handContact[1] = static_cast<float>(hc.y);
            g_travProbe.handContact[2] = static_cast<float>(hc.z);
            break;
        }
    TravOwnLocked(true);
}

// First Step with the event's PSM detail: an Ok plan starts the montage (episode), any other plan yields the episode.
void TravStartLocked()
{
    g_trav.planned = false;
    if (!g_trav.planOk)
    {
        g_trav.yielding = true;
        g_travProbe.yielding = true;
        TravOwnLocked(false);
        return;
    }
    g_trav.planOk = false;
    g_trav.player.emplace(g_trav.plan);
    g_trav.active = g_trav.first = g_trav.episode = true;
    g_trav.action = g_trav.ev.action;
    std::memcpy(g_trav.cap, g_trav.ev.playerPos, sizeof g_trav.cap);
    g_travProbe.active = true;
    g_travProbe.yielding = false;
}

// Montage pose from the .utpptrav version 2 P lines (db bone order through travPoseMap), sampled like adapter::SamplePose:
// frame lerp + quaternion nlerp (shortest arc) at the montage pose time, clamped to the montage frames.
void SampleTravPose(const Bundle& b, const traversal::TravMontage& m, double aTime, BoneTransform* aOut)
{
    const size_t nb = b.travPlay.poseBones.size();
    double ft = (std::isfinite(aTime) ? aTime : 0.0) * m.fps;
    ft = std::clamp(ft, 0.0, static_cast<double>(m.frames - 1));
    const int f0 = static_cast<int>(ft);
    const int f1 = f0 + 1 < m.frames ? f0 + 1 : f0;
    const float u = static_cast<float>(ft - f0);
    const float* a = &m.pose[static_cast<size_t>(f0) * nb * 7];
    const float* c = &m.pose[static_cast<size_t>(f1) * nb * 7];
    for (size_t i = 0; i < b.travPoseMap.size(); ++i)
    {
        const size_t k = static_cast<size_t>(b.travPoseMap[i]) * 7;
        const float* x = a + k;
        const float* y = c + k;
        aOut[i].p = math::Lerp(math::Vec3{x[0], x[1], x[2]}, math::Vec3{y[0], y[1], y[2]}, u);
        math::Quat qa{x[3], x[4], x[5], x[6]};
        math::Quat qb{y[3], y[4], y[5], y[6]};
        if (math::QuatDot(qa, qb) < 0.0f)
            qb = math::QuatNeg(qb);
        aOut[i].q = math::QuatNormalize(math::Quat{qa.x + (qb.x - qa.x) * u, qa.y + (qb.y - qa.y) * u,
                                                   qa.z + (qb.z - qa.z) * u, qa.w + (qb.w - qa.w) * u});
    }
}

// Shared end of a v2-owned Step (MM pick or traversal frame).
// Verifier round 2 (handback after an approach off the wall normal): the traversal frames turn the root children by the plan
// root yaw - entity yaw after Evaluate (review major 2), so the MM handback blend starts from a pose without that turn and
// the published body snapped back by it in one feed (0.5 rad off the normal: ~29 deg). From the handback feed on the root
// children keep that yaw, scaled by 1 - smoothstep(t / kTravBlendOut) (zero turn rate at both ends), about the root.
void TravYawBackLocked(Bundle& b, float aDt)
{
    g_travProbe.handbackYaw = 0.0f;
    if (g_trav.yawBackT < 0.0f)
        return;
    const float u = std::fmin(1.0f, g_trav.yawBackT / kTravBlendOut);
    const double y = static_cast<double>(g_trav.yawBack) * (1.0 - static_cast<double>(u * u * (3.0f - 2.0f * u)));
    g_trav.yawBackT = u >= 1.0f ? -1.0f : g_trav.yawBackT + aDt;
    g_travProbe.handbackYaw = static_cast<float>(y);
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 7
    (void)b;
    (void)y;
    return; // mutation 7: no handback turn (pre-fix snap)
#else
    if (y == 0.0)
        return;
    const math::Quat q{0.0f, 0.0f, static_cast<float>(std::sin(0.5 * y)), static_cast<float>(std::cos(0.5 * y))};
    for (uint32_t i = kFirstBone; i < b.pose.size(); ++i)
        if (b.dbParents[i] >= 0 && b.dbParents[i] < static_cast<int32_t>(kFirstBone)) // root children
        {
            b.pose[i].p = math::QuatRotate(q, b.pose[i].p);
            b.pose[i].q = math::QuatNormalize(math::QuatMul(q, b.pose[i].q));
        }
#endif
}

// UTPP-MOTION SEAM motionfix (item 4): "UTPP MOTION" trace line through the host logger (g_log = Main.cpp MMTraceLog, the same
// synchronous LogInfo path as the v1 mm.trace MMTRACE lines). Switch: SetTrace (Main.cpp UTPPSetMMTrace drives mm.trace and
// this together, default off). Rate [P]: on change of db / clip / rotation mode / IsStarting / IsPivoting / overlay pose key /
// air mode / traversal active, else every kTracePeriod of feed time (2 Hz). Fixed buffers, no allocation per frame.

void TraceLocked(const Bundle& b, const motion::ChooserState& cs, const motion::MatchResult& r, const motionpose::FrameInfo& info,
                 const adapter::MotionFeed& f, float dt)
{
    if (!g_trace || g_log == nullptr)
        return;
    g_traceClock += dt;
    g_traceSince += dt;
    const ov::OverlayMachine& mach = b.eval.Machine();
    TraceKey k;
    k.db = r.database;
    k.clip = r.clip;
    k.rot = static_cast<int32_t>(b.map.st.rotationMode);
    k.poseKey = ov::PoseKey(mach.overlay, mach.aim, mach.crouch, mach.moving);
    k.weaponAction = g_weaponAction;
    k.mode = cs.movementMode;
    k.starting = cs.isStarting;
    k.pivoting = cs.isPivoting;
    k.trav = g_trav.active;
    k.body = static_cast<int32_t>(b.body);
    k.rig = g_rig;
#if UTPP_MOTION_DRIVER_FIX_MUTATION == 2
    k.db = k.clip = -1; // mutation 2: a clip / db change alone does not write a line
#endif
    const bool changed = !(k == g_traceKey);
    if (!changed && g_traceSince < kTracePeriod)
        return;
    g_traceKey = k;
    g_traceSince = 0.0f;
    const bool dbOk = r.database >= 0 && static_cast<uint32_t>(r.database) < b.db.NumDatabases();
    const bool clipOk = r.clip >= 0 && static_cast<uint32_t>(r.clip) < b.db.Header().numClips;
    const std::string_view dn = dbOk ? b.db.DbName(static_cast<uint32_t>(r.database)) : std::string_view("-");
    const std::string_view cn = clipOk ? b.db.ClipName(static_cast<uint32_t>(r.clip)) : std::string_view("-");
    float wx = 0.0f, wy = 0.0f;
    adapter::InputWorld(f, wx, wy);
    const bool haveInput = wx * wx + wy * wy > adapter::kStickReleased * adapter::kStickReleased;
    const float inYaw = haveInput ? std::atan2(-wx, wy) / integ::kDegToRad : 0.0f;
    const float vx = std::isfinite(f.velWorld[0]) ? f.velWorld[0] : 0.0f, vy = std::isfinite(f.velWorld[1]) ? f.velWorld[1] : 0.0f;
    const float velYaw = (vx * vx + vy * vy) > 1.0e-6f ? std::atan2(-vx, vy) / integ::kDegToRad : 0.0f;
    char ovs[160];
    int n = std::snprintf(ovs, sizeof(ovs), "%d/%d/%d%d", static_cast<int>(mach.overlay), static_cast<int>(mach.aim), mach.crouch ? 1 : 0,
                          mach.moving ? 1 : 0);
    for (int i = 0; i < mach.poses.count && n > 0 && n < static_cast<int>(sizeof(ovs)) - 16; ++i)
        n += std::snprintf(ovs + n, sizeof(ovs) - static_cast<size_t>(n), "%s%d:%.2f", i == 0 ? " w=" : ",", mach.poses.key[i],
                           static_cast<double>(mach.poses.w[i]));
    const ov::Curves& cu = info.curves;
    // body, bundle (installed body, loading), live rig, database rig identity, bone map
    char slideTxt[64] = ""; // only while the slide feed is armed (setting on + slide bundle): the OFF trace line is unchanged
    if (g_slideSetting && b.slide.declared)
        std::snprintf(slideTxt, sizeof(slideTxt), " slide=%d/%d/%d", b.map.slideActive ? 1 : 0, cs.movementDirection, cs.movementModeLast);
    char shooterTxt[24] = ""; // only with Shooter mode on or the H2 safeguard latched: the OFF trace line is unchanged
    if (g_shooter_modeSetting || b.map.h2Active)
        std::snprintf(shooterTxt, sizeof(shooterTxt), " shooter=%d/%d", b.map.shooterModeActive ? 1 : 0, b.map.h2Active ? 1 : 0);
    char owTxt[96] = ""; // only while the orientation warp is on or still fading: the OFF trace line is unchanged
    if (g_orientWarpOn || !g_ow.Idle())
        std::snprintf(owTxt, sizeof(owTxt), " ow=%d/%.2f/%.1f/%.1f", g_orientWarpOn ? 1 : 0, static_cast<double>(g_ow.Weight()),
                      static_cast<double>(g_ow.RootDeg()), static_cast<double>(g_ow.TorsoDeg()));
    char leanTxt[112] = ""; // only while the lean is on or still fading: the OFF trace line is unchanged
    if (b.leanOk && (g_leanSetting || !b.lean.Idle()))
        std::snprintf(leanTxt, sizeof(leanTxt), " lean=%d/%.2f/%.2f/%.2f/%.2f", g_leanSetting ? 1 : 0, static_cast<double>(b.lean.Weight()),
                      static_cast<double>(b.lean.Tel().lat), static_cast<double>(b.lean.Tel().x), static_cast<double>(b.lean.Tel().disable));
    char ptTxt[48] = ""; // only with a pelvis trim set or still fading: the default trace line is unchanged
    if (g_pelvisTrimDeg != 0.0f || g_pt.Weight() != 0.0f)
        std::snprintf(ptTxt, sizeof(ptTxt), " pt=%.1f/%.2f/%s", static_cast<double>(g_pelvisTrimDeg), static_cast<double>(g_pt.Weight()), g_ptWhy);
    char boTxt[48] = ""; // only with a body yaw offset set or still fading: the default trace line is unchanged
    if (g_bodyOffsetDeg != 0.0f || g_bo.Weight() != 0.0f)
        std::snprintf(boTxt, sizeof(boTxt), " bo=%.1f/%.2f/%s", static_cast<double>(g_bodyOffsetDeg), static_cast<double>(g_bo.Weight()), g_boWhy);
    char hipTxt[40] = ""; // P9: only on hipfire frames (the other trace lines are unchanged): the native first-burst fallback and its route
    if (g_hipFirePoseNow)
        std::snprintf(hipTxt, sizeof(hipTxt), " hipNative=%d route=%d", g_weaponUpperFrame ? 1 : 0, g_shootRouteNow ? 1 : 0);
    char magTxt[64] = ""; // only while a magazine top-up gesture ran since the last line: the other trace lines are unchanged
    if (g_magTapPeak > 0.0f || !g_magTap.Idle())
        std::snprintf(magTxt, sizeof(magTxt), " magTap=%.2f/%.2f/%llu", static_cast<double>(g_magTap.weight),
                      static_cast<double>(g_magTapPeak), static_cast<unsigned long long>(g_magTapStarted));
    g_magTapPeak = 0.0f;
    char bpTxt[112] = ""; // aim pitch plan 2026-10-30 section 4: only while the barrel loop runs / fades or measured (loop off: unchanged)
    if (info.barrelActive || info.barrelMeasured)
        std::snprintf(bpTxt, sizeof(bpTxt), " bp=%.1f/%.1f/%.2f/%.1f/%.1f/%.2f/%.2f/%.1f/%.2f/%u", static_cast<double>(info.aoPitchInDeg),
                      static_cast<double>(info.aoPitchCmdDeg), static_cast<double>(info.barrelBiasDeg), static_cast<double>(info.barrelPitchDeg),
                      static_cast<double>(info.barrelWantPitchDeg), static_cast<double>(info.barrelErrAfterDeg),
                      static_cast<double>(info.barrelResidualDeg), static_cast<double>(info.barrelTargetDist),
                      static_cast<double>(info.barrelLoopWeight), static_cast<unsigned>(info.barrelExcluded));
    char who[160] = "";
#if UTPP_MOTION_DRIVER_FIX_MUTATION != 5
    std::snprintf(who, sizeof(who), " body=%s bundle=%s%s%s rig=0x%llx rigId=0x%016llx map=%d:%u/%u", BodyName(g_body),
                  BodyName(b.body), g_loading ? " loading:" : "", g_loading ? BodyName(g_loadingBody) : "",
                  static_cast<unsigned long long>(g_rig), static_cast<unsigned long long>(b.res.dbRigIdentity), g_map.ok ? 1 : 0,
                  g_map.mapped, g_map.numDbBones);
#endif
    std::snprintf(g_traceBuf, sizeof(g_traceBuf),
                  "UTPP MOTION t=%.3f db=%d:%.*s clip=%d:%.*s ct=%.3f cost=%.3f blend=%.2f%s rot=%s start=%d pivot=%d inYaw=%.1f velYaw=%.1f"
                  " target=%d ov=%s lay=spine%.2f,armL%.2f,armR%.2f,legs%.2f,head%.2f,pelvis%.2f aimW=%.2f"
                  " air=mode%d,ttl%.2f,land%d%d trav=%d:%s@%.2f act=%d reloadUpper=%.2f shot=%d:%d/%d:%d:%.3f:%llu%s brake=%s/%.2f accel=%s/%.2f support=%d%s headLead=%d lead=%.2f corr=%.2f%s%s%s%s%s%s%s%s%s",
                  static_cast<double>(g_traceClock), r.database, static_cast<int>(dn.size()), dn.data(), r.clip,
                  static_cast<int>(cn.size()), cn.data(), static_cast<double>(r.clipTime),
                  static_cast<double>(r.cost < 1e30f ? r.cost : -1.0f),
                  static_cast<double>(info.liveCrossfade ? info.inertialDuration : (r.blend.valid ? r.blend.blendTime : 0.0f)), info.liveCrossfade ? "(live)" : (info.inertialActive ? "(inert)" : ""),
                  k.rot == 1 ? "strafe" : "orient", cs.isStarting ? 1 : 0, cs.isPivoting ? 1 : 0, static_cast<double>(inYaw),
                  static_cast<double>(velYaw), f.haveTarget ? 1 : 0, ovs, static_cast<double>(cu[ov::kSpine]),
                  static_cast<double>(cu[ov::kArmL]), static_cast<double>(cu[ov::kArmR]), static_cast<double>(cu[ov::kLegs]),
                  static_cast<double>(cu[ov::kHead]), static_cast<double>(cu[ov::kPelvis]),
                  static_cast<double>(mach.AimingWeight()), g_airDiag.movementMode, static_cast<double>(g_airDiag.timeToLand),
                  g_airDiag.justLandedLight ? 1 : 0, g_airDiag.justLandedHeavy ? 1 : 0, g_trav.active ? 1 : 0,
                  g_trav.active ? g_travProbe.montage : "-", static_cast<double>(g_travProbe.poseTime), g_weaponAction,
                  static_cast<double>(g_reloadUpperWeight), g_shootTransferEnabled ? 1 : 0,
                  g_shootEpisode.pending ? 1 : 0, g_shootEpisode.admitted ? 1 : 0, info.shootApplied ? 1 : 0,
                  static_cast<double>(info.shootScale),
                  static_cast<unsigned long long>(g_shootAppliedEvaluations), who,
                  b.map.capsuleBrakeModel > 0.0f ? "cp" : "motion", static_cast<double>(b.map.capsuleBrakeModel),
                  b.map.capsuleAccelModel > 0.0f ? "cp" : "motion", static_cast<double>(b.map.capsuleAccelModel),
                  b.eval.CancelledStartSupport(), changed ? " chg" : "", g_headLeadOn ? 1 : 0, static_cast<double>(g_hl.Lead()),
                  static_cast<double>(g_hl.Corr()), slideTxt, owTxt, leanTxt, ptTxt, boTxt, hipTxt, shooterTxt, magTxt, bpTxt);
    ++g_traceLines;
    g_log(g_traceBuf);
    if (info.poseDiagValid)
    {
        char poseTrace[2200];
        int used = std::snprintf(poseTrace, sizeof(poseTrace), "UTPP MOTIONPOSE t=%.3f", static_cast<double>(g_traceClock));
        const auto appendStage = [&](const char* aTag, const motionpose::FrameInfo::PoseStageDelta& aStage) {
            if (used <= 0 || used >= static_cast<int>(sizeof(poseTrace)))
                return;
            used += std::snprintf(poseTrace + used, sizeof(poseTrace) - static_cast<size_t>(used), " %s=", aTag);
            for (size_t i = 0; i < motionpose::kPoseDiagnosticBoneNames.size() && used > 0 && used < static_cast<int>(sizeof(poseTrace)); ++i)
                used += std::snprintf(poseTrace + used, sizeof(poseTrace) - static_cast<size_t>(used), "%s%s:%.1f/%.4f",
                                      i == 0 ? "" : ",", motionpose::kPoseDiagnosticBoneNames[i].data(),
                                      static_cast<double>(aStage.rotDeg[i]), static_cast<double>(aStage.posM[i]));
        };
        appendStage("compose", info.composeDelta);
        appendStage("feminine", info.feminineDelta);
        appendStage("overlay", info.overlayDelta);
        appendStage("aim", info.aimDelta);
        appendStage("grip", info.gripDelta);
        if (used > 0 && used < static_cast<int>(sizeof(poseTrace)))
            used += std::snprintf(poseTrace + used, sizeof(poseTrace) - static_cast<size_t>(used), " clavLimit=%.1f/%.1f%s",
                                  static_cast<double>(info.clavicleLimit.leftDeg),
                                  static_cast<double>(info.clavicleLimit.rightDeg), info.clavicleLimit.applied ? " lim" : "");
        g_log(poseTrace);
    }
}

// Feed budget. Each feed that produced a motion pose records its wall time and the smaller of the wall time and the thread's own
// processor time (time the game thread spends preempted by other programs is not a cost of the feed). When the p95 of the
// last kWindow decision times exceeds kTripFactor x kFeedP95Us the motion pose pauses (Resolve -> Budget, the held pose hands
// over to the base path) for g_budgetPauseS of feed time; then the statistics restart empty and the next Step re-enters with the
// hand-in from the published pose. One log line per trip and per resume.
void BudgetLogLocked(const char* aLine)
{
    if (g_log != nullptr)
        g_log(aLine);
}

void BudgetTripLocked(float aP95)
{
    g_budgetTripped = true; // takes effect from the next Step (Resolve -> Budget), this pose is still published
    ++g_budgetTrips;
    g_budgetPauseS = g_budgetNextPauseS;
    g_budgetPauseLeft = g_budgetPauseS;
    g_budgetNextPauseS = std::fmin(2.0f * g_budgetPauseS, adapter::Budget::kPauseMaxS);
    g_budgetTripP95 = aP95;
    g_budgetTripWallP95 = WallP95Locked();
    RefreshTravActiveLocked();
    TravRecheckDeferredLocked();
    std::snprintf(g_budgetBuf, sizeof(g_budgetBuf),
                  // no "feedP95us=" token here: log readers take the newest line carrying it as the status line
                  "UTPP mm.motionDriver budget trip #%llu p95us=%.1f wallP95us=%.1f limit=%.0f pause=%.1fs next=%.0fs threadTime=%d",
                  static_cast<unsigned long long>(g_budgetTrips), static_cast<double>(aP95), static_cast<double>(g_budgetTripWallP95),
                  static_cast<double>(adapter::Budget::kTripFactor * adapter::Budget::kFeedP95Us), static_cast<double>(g_budgetPauseS),
                  static_cast<double>(g_budgetNextPauseS), g_cyclesPerUs.load(std::memory_order_relaxed) > 0.0f ? 1 : 0);
    BudgetLogLocked(g_budgetBuf);
}

// Game thread, every enabled Step before the path is resolved: a pause counts down on feed time (dt clamped like the other feed
// clocks) and ends here, so this Step already runs the motion path again.
void BudgetPauseStepLocked(float aDt)
{
    if (!g_budgetTripped)
        return;
    if (std::isfinite(aDt) && aDt > 0.0f)
        g_budgetPauseLeft = std::fmax(0.0f, g_budgetPauseLeft - std::fmin(aDt, 0.25f));
#if UTPP_MOTION_DRIVER_BUDGET_MUTATION == 1
    return; // mutation 1: the pause never ends
#else
    if (g_budgetPauseLeft > 0.0f)
        return;
    g_budgetTripped = false;
    g_timeCount = 0; // the statistics window restarts empty
    RefreshTravActiveLocked();
    TravRecheckDeferredLocked();
    std::snprintf(g_budgetBuf, sizeof(g_budgetBuf),
                  "UTPP mm.motionDriver budget resume #%llu after=%.1fs tripP95us=%.1f tripWallP95us=%.1f next=%.0fs",
                  static_cast<unsigned long long>(g_budgetTrips), static_cast<double>(g_budgetPauseS),
                  static_cast<double>(g_budgetTripP95), static_cast<double>(g_budgetTripWallP95),
                  static_cast<double>(g_budgetNextPauseS));
    BudgetLogLocked(g_budgetBuf);
#endif
}

void BudgetRecordLocked(float aWallUs, float aUs)
{
    const size_t i = static_cast<size_t>(g_timeCount % g_times.size());
    g_times[i] = aUs;
    g_wallTimes[i] = aWallUs;
    ++g_timeCount;
    if (g_timeCount < adapter::Budget::kWindow)
        return;
    const float p95 = P95Locked();
    if (p95 > adapter::Budget::kTripFactor * adapter::Budget::kFeedP95Us)
        BudgetTripLocked(p95);
    else if (g_timeCount == adapter::Budget::kWindow)
        g_budgetNextPauseS = adapter::Budget::kPauseFirstS; // a full window without a trip
}

bool FinishFrameLocked(std::chrono::steady_clock::time_point t0)
{
    ++g_frames;
    g_pending = true;
    g_wasRunning = true;
    g_hoLeft = 0.0f;
    g_lastSkip = "none";
    float wallUs = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
    float us = wallUs;
    const float cyclesPerUs = g_cyclesPerUs.load(std::memory_order_relaxed);
    ULONG64 c1 = 0;
    if (g_feedCyclesOk && cyclesPerUs > 0.0f && QueryThreadCycleTime(GetCurrentThread(), &c1) && c1 >= g_feedCycles0)
        us = std::fmin(wallUs, static_cast<float>(static_cast<double>(c1 - g_feedCycles0) / static_cast<double>(cyclesPerUs)));
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    if (g_testWallUs >= 0.0f)
    {
        wallUs = g_testWallUs;
        us = g_testThreadUs >= 0.0f ? std::fmin(wallUs, g_testThreadUs) : wallUs;
    }
#endif
#if UTPP_MOTION_DRIVER_BUDGET_MUTATION == 2
    us = wallUs; // mutation 2: wall time only
#endif
    BudgetRecordLocked(wallUs, us);
    return true;
}

// One traversal frame: montage pose (database clip at the Player's pose time) + warp root (plan root - capsule on the root
// children). The Player hands back at its end frame: this frame still shows the montage, the next Step is MM (JustTraversed).
bool TravFrameLocked(Bundle& b, const motionhost::FeedInput& aRaw, const motion::ChooserState& cs, const adapter::MotionFeed& f, float dt,
                     std::chrono::steady_clock::time_point t0)
{
    const motiontrav::Frame fr = g_trav.player->Tick(dt);
    motion::MatchResult mr{};
    mr.clip = g_trav.clip;
    mr.cost = 0.0f;
    const bool fromPoses = g_trav.clip < 0; // montage not in the db: .utpptrav version 2 poses
    if (fromPoses)
    {
        const traversal::TravMontage& m = *g_trav.plan.montage;
        mr.clipTime = std::clamp(static_cast<float>(fr.poseTime), 0.0f, static_cast<float>((m.frames - 1) / m.fps));
        SampleTravPose(b, m, mr.clipTime, b.travRaw.data());
    }
    else
    {
        const auto& cl = b.db.Clip(static_cast<uint32_t>(g_trav.clip));
        const float fps = b.db.Header().fps > 0.0f ? b.db.Header().fps : 30.0f;
        const float tMax = static_cast<float>(cl.numFrames - 1) / fps;
        mr.clipTime = std::clamp(static_cast<float>(fr.poseTime), 0.0f, tMax);
        mr.frame = cl.firstFrame + std::min(static_cast<uint32_t>(mr.clipTime * fps), cl.numFrames - 1);
    }
    if (g_trav.first)
    {
        mr.blend.valid = true; // inertialized from the last v2 pose
        mr.blend.blendTime = kTravBlendIn;
        mr.blend.toClip = g_trav.clip;
        mr.blend.fromClip = g_last.clip;
        mr.jumped = true; // new timeline (PoseEvaluator: no target-prev from the MM clip)
    }
    g_trav.first = false;
    g_last = mr;
    motionpose::OverlayFrameInputs in = FrameInputsLocked(b, cs, f, dt);
    in.montageBase = true; // DB and raw traversal share the protected external bridge
    motionpose::FrameInfo info;
    g_ovArmsNow = false; // traversal montage frames never claim both hands on the weapon
    const bool ok = fromPoses ? b.eval.EvaluateWithPose(dt, mr, b.travRaw.data(), in, b.pose.data(),
                                                        static_cast<uint32_t>(b.pose.size()), &info)
                              : b.eval.Evaluate(dt, mr, in, b.pose.data(), static_cast<uint32_t>(b.pose.size()), &info);
    RecordGripInfoLocked(info);
    RecordBarrelProbeLocked(in, info, nullptr);
    if (!info.finite)
        FaultLocked("fault:non-finite-pose");
    if (!ok)
    {
        Skip(g_evalFail, info.finite ? "no-pose" : "fault");
        return false;
    }
    // capsule: the feed's entity position (batch 1b UTPPFeedLocomotionExtra), else integrated from the event position
    if (aRaw.haveExtra && std::isfinite(aRaw.capsulePos[0] + aRaw.capsulePos[1] + aRaw.capsulePos[2]))
        std::memcpy(g_trav.cap, aRaw.capsulePos, sizeof g_trav.cap);
    else
        for (int k = 0; k < 3; ++k)
            g_trav.cap[k] += (std::isfinite(aRaw.velWorld[k]) ? aRaw.velWorld[k] : 0.0f) * dt;
    const double yaw = (std::isfinite(aRaw.playerYawDeg) ? aRaw.playerYawDeg : 0.0f) * static_cast<double>(integ::kDegToRad);
    const traversal::V3 off = fr.root.pos - TravV(g_trav.cap);
    const traversal::V3 loc = traversal::RotZ(off, -yaw); // world -> character frame (entity yaw)
    // review major 2: the warped plan root also turns (owner yaw -> faceYaw = -collisionNormal over the warp windows, plus
    // the clip's own root turn); the montage pose is root-relative, so the root children rotate about the root by
    // (plan root yaw - entity yaw) before the translation: the published hand lands where the plan (probe) puts it
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 1
    const double dYaw = 0.0; // mutation 1: the plan root yaw is ignored
#else
    const double dYaw = traversal::WrapPi(fr.root.yaw - yaw);
#endif
    const float hs = static_cast<float>(std::sin(0.5 * dYaw)), hc = static_cast<float>(std::cos(0.5 * dYaw));
    const math::Quat qYaw{0.0f, 0.0f, hs, hc};
    g_trav.lastYaw = static_cast<float>(dYaw);
    g_trav.yawBackT = -1.0f; // a montage frame: no handback turn running
    for (uint32_t i = kFirstBone; i < b.pose.size(); ++i)
        if (b.dbParents[i] >= 0 && b.dbParents[i] < static_cast<int32_t>(kFirstBone)) // root children (warp root)
        {
            if (dYaw != 0.0)
            {
                b.pose[i].p = math::QuatRotate(qYaw, b.pose[i].p);
                b.pose[i].q = math::QuatNormalize(math::QuatMul(qYaw, b.pose[i].q));
            }
            b.pose[i].p.x += static_cast<float>(loc.x);
            b.pose[i].p.y += static_cast<float>(loc.y);
            b.pose[i].p.z += static_cast<float>(loc.z);
        }
    // root motion handed to the v1 tail = the capsule velocity (character frame): the warp above already places the root
    // relative to the capsule, so v1's offset root (animation root motion - capsule motion, sprung back) must not add its
    // own drift on top (montage delta here put the published hips / hand 0.2-0.3 m off the plan root: integ 8f / 8g)
    integ::WorldToCharacter(std::isfinite(aRaw.velWorld[0]) ? aRaw.velWorld[0] : 0.0f,
                            std::isfinite(aRaw.velWorld[1]) ? aRaw.velWorld[1] : 0.0f, static_cast<float>(yaw), g_trav.vel[0],
                            g_trav.vel[1]);
    g_trav.frame = true;
    ++g_travProbe.frames;
    g_travProbe.poseTime = mr.clipTime;
    g_travProbe.root[0] = static_cast<float>(fr.root.pos.x);
    g_travProbe.root[1] = static_cast<float>(fr.root.pos.y);
    g_travProbe.root[2] = static_cast<float>(fr.root.pos.z);
    g_travProbe.rootYaw = static_cast<float>(fr.root.yaw);
    g_travProbe.offset[0] = static_cast<float>(off.x);
    g_travProbe.offset[1] = static_cast<float>(off.y);
    g_travProbe.offset[2] = static_cast<float>(off.z);
    g_travProbe.justTraversed = false;
    TraceLocked(b, cs, mr, info, f, dt); // UTPP-MOTION SEAM motionfix (item 4): MOTION trace line (montage frame)
    if (fr.done)
    {
        g_trav.active = false;
        g_trav.player.reset();
        g_trav.handbackNext = true;
        g_travProbe.active = false;
    }
    return FinishFrameLocked(t0);
}
// ------------------------------------------------------------------------------------------------- UTPP-MOTION SEAM trav

// air diagnostics of one v2 Step (after the matcher update): chooser air inputs and the root vertical residual. RTZ is a
// feature only (report 6.6.4): the animation root vertical (integral of the playing frame's RTZ vz) is compared with the capsule
// (integral of the feed vz) over the air episode and reported, never applied to the pose.
void AirStepLocked(const Bundle& b, const motion::ChooserState& cs, const adapter::AirFeed& aAir, float aVz, float aDt)
{
    AirDiag& d = g_airDiag;
    d.feedValid = aAir.valid;
    d.movementMode = cs.movementMode;
    d.timeToLand = cs.timeToLand;
    d.distanceUsed = aAir.valid ? aAir.distanceToGround : -1.0f;
    d.justLandedLight = cs.justLandedLight;
    d.justLandedHeavy = cs.justLandedHeavy;
    d.stance = cs.stance;
    d.landVzCmS = b.map.st.landVz;
    const bool inAir = cs.movementMode == 1;
    float rz = 0.0f, rvz = 0.0f;
    if (g_last.clip >= 0 && g_last.frame < b.db.NumFrames())
    {
        const motion::RtzRecord& z = b.db.Rtz(g_last.frame);
        rz = std::isfinite(z.rootZ) ? z.rootZ : 0.0f;
        rvz = std::isfinite(z.vz) ? z.vz : 0.0f;
    }
    d.rtzRootZ = rz;
    if (inAir)
    {
        if (!g_airEpisode)
        {
            ++d.episodes;
            d.animDz = d.capsuleDz = d.residual = d.maxResidual = 0.0f;
        }
        ++d.airSteps;
        if (AirMut() != 7)
            d.animDz += rvz * aDt;
        d.capsuleDz += aVz * aDt;
        d.residual = d.animDz - d.capsuleDz;
        d.maxResidual = std::fmax(d.maxResidual, std::fabs(d.residual));
    }
    g_airEpisode = inAir;
}

// Spine1..Spine3, the clavicle roots (parents of the grip rig's upper arms) and Neck of this bundle, in ov indices.
const slide_deroll::Bones& DeRollBonesLocked(const Bundle& aB)
{
    if (g_derollBundle != &aB)
    {
        g_derollBones = slide_deroll::Bones{};
        const ov::Skeleton& sk = aB.eval.Skel();
        const gik::Rig& rig = aB.eval.GripRig();
        const uint32_t n = static_cast<uint32_t>(aB.pose.size());
        if (rig.Valid(sk))
        {
            g_derollBones.lsh = sk.parent[rig.left.upper];
            g_derollBones.rsh = sk.parent[rig.right.upper];
            static constexpr std::string_view kSpine[3] = {"Spine1", "Spine2", "Spine3"};
            for (uint32_t i = 0; i < n; ++i)
            {
                for (int k = 0; k < 3; ++k)
                    if (aB.db.BoneName(i) == kSpine[k])
                        g_derollBones.spine[k] = aB.eval.OvBone(i);
                if (aB.db.BoneName(i) == std::string_view("Neck"))
                    g_derollBones.neck = aB.eval.OvBone(i);
            }
        }
        g_derollBundle = &aB;
    }
    return g_derollBones;
}

// Pistol / rifle reload on a slide frame (end of StepLocked, b.pose is the final MM pose of the frame): the MM torso is set upright
// (de-roll + de-pitch, SlideDeRoll.hpp; weight from the roll BEFORE the edit), then the roll hold sees the roll that is LEFT (torso roll proxy =
// elevation of the MM shoulder line, see SlideRollHold) and stays as the safety (pistols only: g_reloadNowStep). A rifle reload keeps the
// de-roll through the still-rolled slide exit (RifleSlideExitLatch). A drawn rifle whose arms MM owns gets the same uprighting outside
// a reload (carry path, SlideCarryRifle: early ramp, weight x (1 - AimingWeight), the same exit latch). Without a slide reload / carry
// slide frame and with the de-roll at rest nothing reads or writes the pose and the hold decays as before.
void SlideReloadTorsoStepLocked(Bundle& aB, bool aMMOwnsArms, float aDt)
{
    g_derollAppliedDeg = g_depitchAppliedDeg = 0.0f;
    float roll = 0.0f;
    const bool slideReload = g_slideTorsoReloadStep && g_slideFrame;
    const bool rifleCarry =
        reload_contacts_runtime::SlideCarryRifle(OverlayLocked() == ov::PoseOverlay::Rifle, aMMOwnsArms, g_slideTorsoReloadStep);
    bool engaged = false;
    if (slideReload || (rifleCarry && g_slideFrame) || !g_slideDeRoll.Idle() || g_rifleSlideLatch.latched)
    {
        const ov::Skeleton& sk = aB.eval.Skel();
        const slide_deroll::Bones& bones = DeRollBonesLocked(aB);
        const bool valid = bones.Valid(sk) && aB.pose.size() <= static_cast<size_t>(ov::kMaxBones);
        ov::Pose p;
        float before = 0.0f;
        if (valid)
        {
            aB.eval.ToOv(aB.pose.data(), p);
            before = slide_deroll::RollDeg(sk, p, bones);
        }
        // reload and carry exclude each other (MMOwnsArmsLocked needs g_weaponAction != 2): without a carry Step the latch and the
        // reload gate see exactly the former inputs
        const bool exitLatch = g_rifleSlideLatch.Step((g_reloadRifleStep || rifleCarry) && valid, g_slideFrame, before);
        const bool gate = (slideReload || (exitLatch && g_reloadRifleStep)) && valid;
        const bool carryGate = rifleCarry && (g_slideFrame || exitLatch) && valid;
        engaged = reload_contacts_runtime::SlideDeRoll::Engaged(gate);
        const float w = g_slideDeRoll.Step(aDt, gate || carryGate, before, carryGate);
        g_slideCarryScale = g_slideCarryAim.Step(aDt, carryGate, gate, aB.eval.Machine().AimingWeight());
        const float wApplied = w * g_slideCarryScale; // scale exactly 1 without a carry frame in this episode
        g_slideCarryFrames += carryGate ? 1u : 0u;
        if (g_slideDeRoll.Idle())
            g_slideCarryAim.Reset();
        roll = before;
        if (valid && wApplied > 0.0f)
        {
            const slide_deroll::Result applied = slide_deroll::Apply(sk, p, bones, wApplied);
            g_derollAppliedDeg = applied.rollDeg;
            g_depitchAppliedDeg = applied.pitchDeg;
            if (applied.Changed())
            {
                aB.eval.FromOv(p, aB.pose.data());
                roll = slide_deroll::RollDeg(sk, p, bones);
            }
        }
        if (!valid && slideReload)
        {
            // no Spine1..3 chain: the former measurement on the grip rig (the hold alone owns the safety)
            const gik::Rig& rig = aB.eval.GripRig();
            if (rig.Valid(sk))
            {
                aB.eval.ToOv(aB.pose.data(), p);
                const auto l = native_reload::Model(sk, p, sk.parent[rig.left.upper]).t;
                const auto r = native_reload::Model(sk, p, sk.parent[rig.right.upper]).t;
                roll = reload_contacts_runtime::ShoulderLineRollDeg(l.x, l.y, l.z, r.x, r.y, r.z);
            }
        }
    }
    g_reloadRollHold.Step(aDt, g_reloadNowStep, g_slideFrame, slideReload ? roll : 0.0f, engaged);
    g_rollHoldStepped = true;
}

// A pending magazine top-up request ages by one Step; it is dropped after kMagTapPendingSteps Steps or its stale window
// (kMagTapPendingSeconds, + magtap::kSlideDelaySeconds once it waited for the slide de-roll: MagTapPendingWindow).
void MagTapAgePendingLocked()
{
    if (!g_magTapPending)
        return;
    ++g_magTapPendingAge;
    if (g_magTapPendingAge >= kMagTapPendingSteps || RecoilClock() - g_magTapPendingTime > MagTapPendingWindow())
    {
        g_magTapPending = false;
        g_magTapDeferred = false;
        g_magTapWaited = 0.0f;
        ++g_magTapDropped;
        ++g_magTapUnowned;
    }
}

// MM owns the arms in TPP: ApplyLocked publishes the MM upper body at upperScale 1 (weapon overlay drawn, no release, no reload action /
// envelope / contact tail, no game-owned upper body or its fade tail, no traversal, no v1 <-> v2 hand-in / handover, gate open).
// Admission of the magazine top-up gesture and of the rifle carry de-roll (SlideReloadTorsoStepLocked).
bool MMOwnsArmsLocked(const motionhost::FeedInput& aRaw, uint32_t aEffGate, const adapter::MotionFeed& aF)
{
    const ov::PoseOverlay o = OverlayLocked();
    const bool weaponOverlay = o == ov::PoseOverlay::Rifle || o == ov::PoseOverlay::Pistol1H || o == ov::PoseOverlay::Pistol2H;
    return aRaw.tppActive && aEffGate == 0 && weaponOverlay && aF.haveWeapon && aF.weaponDrawn && !g_release &&
        g_weaponAction != 2 && g_reloadUpperWeight <= 0.0f && !g_reloadContactFade.Active() && !(g_aimGameUpper && g_aimingNow) &&
        !g_weaponUpperFrame && !g_aimUpperTail && !g_weaponUpperTail && !g_trav.active && g_hiLeft <= 0.0f && g_hoLeft <= 0.0f;
}

// Every Step that does not reach MagTapStepLocked (skip / yield / fallback / traversal montage): the pose of such a frame is not
// edited, a running gesture fades out (ownership lost) instead of freezing and replaying later, a pending request ages.
void MagTapIdleStepLocked(float aDt)
{
    if (!g_magTap.Idle())
        (void)g_magTap.Step(aDt, false);
    g_magTapMoved = g_magTapErr = 0.0f;
    MagTapAgePendingLocked();
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    g_magTapProbe = MagTapProbe{};
#endif
}

// Magazine top-up gesture (mm/MagTap.hpp) on the final MM pose of a main-path Step (after the slide torso uprighting, so the hand
// meets the weapon where it is published). Admission = MM owns the arms in TPP: the state in which ApplyLocked publishes the MM upper
// body at upperScale 1 (weapon overlay drawn, no release, no reload action / envelope / contact tail, no game-owned upper body or its
// fade tail, no traversal, no v1 <-> v2 hand-in / handover, gate open). Weight 0: nothing reads or writes the pose (bit-identical).
void MagTapStepLocked(Bundle& aB, bool aOwned, float aDt)
{
    g_magTapStepped = true;
    g_magTapMoved = g_magTapErr = 0.0f;
    const ov::PoseOverlay o = OverlayLocked();
    const magtap::Kind kind = o == ov::PoseOverlay::Rifle ? magtap::Kind::Rifle : magtap::Kind::Pistol;
    const bool owned = aOwned;
    if (g_magTapPending && RecoilClock() - g_magTapPendingTime > MagTapPendingWindow())
    {
        g_magTapPending = false; // the refill is too old to show (a hitch / a long gap without Steps)
        g_magTapDeferred = false;
        g_magTapWaited = 0.0f;
        ++g_magTapDropped;
        ++g_magTapUnowned;
    }
    bool deferNow = false;
    if (g_magTapPending && owned && g_magTap.Idle())
    {
        // slide roll-in: a new rifle gesture waits for the carry de-roll (weight >= .9) or kSlideDelaySeconds of feed time
        const float d = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.25f) : 0.0f;
        deferNow = magtap::SlideDefer(kind, g_slideFrame, g_slideDeRoll.weight, g_magTapWaited + d);
        if (deferNow)
        {
            g_magTapDeferredReqs += g_magTapDeferred ? 0u : 1u;
            g_magTapDeferred = true;
            g_magTapWaited += d;
        }
    }
    if (g_magTapPending && !deferNow)
    {
        if (owned && (g_magTap.Idle() || kind == g_magTapKind))
        {
            // a new gesture needs the left hand near the magazine (a one-handed hold would be an arm swing): dropped at once
            float travel = g_magTap.Idle() ? -1.0f : 0.0f; // a running gesture keeps its target (restart, no new check)
            if (g_magTap.Idle() && aB.pose.size() <= static_cast<size_t>(ov::kMaxBones))
            {
                ov::Pose p;
                aB.eval.ToOv(aB.pose.data(), p);
                travel = magtap::TargetDistance(aB.eval.Skel(), p, aB.eval.GripRig(), kind);
                g_magTapTravel = travel;
            }
            if (travel >= 0.0f && travel <= magtap::kMaxTravel)
            {
                if (g_magTap.Idle())
                    g_magTapKind = kind;
                g_magTap.Trigger();
                ++g_magTapStarted;
                g_magTapLastWait = g_magTapWaited;
            }
            else
                ++g_magTapDropped;
            g_magTapPending = false;
            g_magTapDeferred = false;
            g_magTapWaited = 0.0f;
        }
        else
            MagTapAgePendingLocked();
    }
    const float w = g_magTap.Idle() ? 0.0f : g_magTap.Step(aDt, owned && kind == g_magTapKind);
    g_magTapPeak = std::fmax(g_magTapPeak, w);
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    g_magTapProbe = MagTapProbe{};
    g_magTapProbe.stepped = true;
    g_magTapProbe.owned = owned;
    g_magTapProbe.weight = w;
    g_magTapProbe.kind = static_cast<int32_t>(g_magTapKind);
    const auto copyPose = [&](std::vector<float>& aDst) {
        aDst.resize(aB.pose.size() * 7);
        for (size_t i = 0; i < aB.pose.size(); ++i)
        {
            const BoneTransform& x = aB.pose[i];
            const float v[7] = {x.p.x, x.p.y, x.p.z, x.q.x, x.q.y, x.q.z, x.q.w};
            std::memcpy(&aDst[i * 7], v, sizeof v);
        }
    };
    copyPose(g_magTapProbe.before);
#endif
    if (w > 0.0f && aB.pose.size() <= static_cast<size_t>(ov::kMaxBones))
    {
        ov::Pose p;
        aB.eval.ToOv(aB.pose.data(), p);
        magtap::ApplyResult res;
        if (magtap::Apply(aB.eval.Skel(), p, aB.eval.GripRig(), g_magTapKind, w, res))
        {
            aB.eval.FromOv(p, aB.pose.data());
            ++g_magTapFrames;
            g_magTapMoved = res.handMoved;
            g_magTapErr = res.handErr;
            g_magTapMovedMax = std::fmax(g_magTapMovedMax, res.handMoved);
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
            g_magTapProbe.edited = true;
            g_magTapProbe.handMoved = res.handMoved;
            g_magTapProbe.handErr = res.handErr;
            g_magTapProbe.targetDist = res.targetDist;
            g_magTapProbe.clamped = res.clamped;
#endif
        }
    }
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    copyPose(g_magTapProbe.after);
#endif
}

// Head lead, feed rate: the signals of this v2 feed (called at the end of StepLocked, after the weapon / aim state of the frame is known).
void HeadLeadStepLocked(const adapter::MotionFeed& aF, float aDt)
{
    if (!g_headLeadOn && g_hl.Weight() == 0.0f && g_hl.Corr() == 0.0f)
        return; // off and fully faded: nothing runs, the pose path is untouched
    hlead::Signals sg;
    sg.vx = aF.velWorld[0];
    sg.vy = aF.velWorld[1];
    sg.yawDeg = aF.playerYawDeg; // actor yaw: the lead is measured against the travel direction in the actor frame (HeadLead.hpp)
    sg.yawValid = std::isfinite(aF.playerYawDeg);
    sg.aiming = g_aimingNow || aF.aiming;
    sg.weaponBusy = g_weaponAction == 2 || g_weaponAction == 8 || g_hipFirePoseNow || g_reloadUpperWeight > 0.0f;
    sg.air = (aF.air.valid && !aF.air.onGround) || adapter::PsmAir(aF.psmDetailed);
    sg.crouch = adapter::PsmCrouchStance(aF.psmDetailed);
    sg.blocked = g_trav.active || g_release || !g_headLeadOn || g_leanHeadOwns; // switched off: the weight fades out first
    g_hl.Step(g_hlParams, sg, aDt);
}

// Head lead, publish rate: corrects Neck / Neck1 / Head of the v2 snapshot pose in place (database bone order = snapshot order).
void HeadLeadApplyLocked(const Bundle& aB, integ::PoseSnapshot& aSnap)
{
    if ((!g_headLeadOn && g_hl.Weight() == 0.0f && g_hl.Corr() == 0.0f) || !(aSnap.weight > 0.0f))
        return;
    const uint32_t n = static_cast<uint32_t>(aB.pose.size());
    if (g_hlSkBundle != &aB || (g_hlSk.ok && (g_hlSk.head >= static_cast<int32_t>(n) || aB.db.BoneName(static_cast<uint32_t>(g_hlSk.head)) != "Head")))
    {
        g_hlSk = hlead::ResolveDb(aB.db, n);
        g_hlSkBundle = &aB;
        g_hl.Reset();
        g_hlHaveApply = false;
    }
    const auto now = std::chrono::steady_clock::now();
    float dt = 1.0f / 60.0f;
    if (g_hlHaveApply)
        dt = std::clamp(std::chrono::duration<float>(now - g_hlLastApply).count(), 1.0f / 240.0f, 0.1f);
    g_hlLastApply = now;
    g_hlHaveApply = true;
    g_hl.Apply(g_hlParams, g_hlSk, aSnap.pose, std::min<uint32_t>(n, integ::kMaxMMBones), dt);
}

// Orientation warp, feed rate: gates, path / turn rate (Step) and the measurement of the v2 pose of this feed (Measure). The pose and
// the root frame are final here (Evaluate + Commit done); FrameLocked applies the result to the leg-tail copy of the same pose.
void OrientWarpStepLocked(Bundle& aB, const adapter::MotionFeed& aF, int32_t aClip, float aDt)
{
    if (!g_orientWarpOn && g_ow.Idle())
        return; // off and fully faded: nothing runs, the frame path is untouched
    const uint32_t n = static_cast<uint32_t>(aB.pose.size());
    if (g_owSkBundle != &aB || (g_owSk.ok && g_owSk.hips >= static_cast<int32_t>(n)))
    {
        g_owSk = owarp::ResolveDb(aB.db, n);
        g_owSkBundle = &aB;
        g_ow.Reset();
    }
    // loop clips only (F / FL / FR / F_L_20 ..): starts, stops, pivots, turns, reface, idles and transitions keep their own yaw
    const bool loopClip = aClip >= 0 && static_cast<uint32_t>(aClip) < aB.db.NumClips() &&
                          steer::ClassifyClip(aB.db.ClipName(static_cast<uint32_t>(aClip))) == steer::kClassNone;
    owarp::Signals sg;
    sg.vx = aF.velWorld[0];
    sg.vy = aF.velWorld[1];
    sg.yawDeg = aF.playerYawDeg;
    sg.yawValid = std::isfinite(aF.playerYawDeg);
    sg.aiming = g_aimingNow || aF.aiming;
    sg.weaponBusy = g_weaponAction == 2 || g_weaponAction == 8 || g_hipFirePoseNow || g_reloadUpperWeight > 0.0f || g_weaponUpperFrame ||
                    (aF.haveWeapon && aF.weaponDrawn);
    sg.air = (aF.air.valid && !aF.air.onGround) || adapter::PsmAir(aF.psmDetailed);
    sg.crouch = adapter::PsmCrouchStance(aF.psmDetailed) || g_slideFrame;
    sg.blocked = g_trav.active || g_release || !g_orientWarpOn || !aB.rootFrameValid || !loopClip || !g_owSk.ok || // off: fades out first
                 aB.map.shooterModeActive || aB.map.h2Active; // the Strafe modes hold the root on the actor: nothing to warp towards the path
    g_ow.Step(g_owParams, sg, aDt);
    const float rootYawDeg = aB.rootFrameValid ? aB.rootFrame.Frame().yaw / integ::kDegToRad : 0.0f;
    static_assert(sizeof(BoneTransform) == 7 * sizeof(float), "pose layout: p.xyz q.xyzw");
    g_ow.Measure(g_owParams, g_owSk, reinterpret_cast<const float*>(aB.pose.data()), n, rootYawDeg, aDt);
}

// Run pelvis yaw trim, feed rate: the forward-locomotion weight of this feed (same gates as the orientation warp, no turn-rate window,
// no loop-clip restriction; FrameLocked scales it by 1 - the warp weight). Trim 0 and faded: nothing runs.
void PelvisTrimStepLocked(const Bundle& aB, const adapter::MotionFeed& aF, float aDt)
{
    if (g_pelvisTrimDeg == 0.0f && g_pt.Weight() == 0.0f)
        return;
    const uint32_t n = static_cast<uint32_t>(aB.pose.size());
    if (g_ptSkBundle != &aB || (g_ptSk.ok && g_ptSk.hips >= static_cast<int32_t>(n)))
    {
        g_ptSk = owarp::ResolveDb(aB.db, n);
        g_ptSkBundle = &aB;
        g_pt.Reset();
    }
    owarp::Signals sg;
    sg.vx = aF.velWorld[0];
    sg.vy = aF.velWorld[1];
    sg.yawDeg = aF.playerYawDeg;
    sg.yawValid = std::isfinite(aF.playerYawDeg);
    sg.aiming = g_aimingNow || aF.aiming;
    sg.weaponBusy = g_weaponAction == 2 || g_weaponAction == 8 || g_hipFirePoseNow || g_reloadUpperWeight > 0.0f || g_weaponUpperFrame ||
                    (aF.haveWeapon && aF.weaponDrawn);
    sg.air = (aF.air.valid && !aF.air.onGround) || adapter::PsmAir(aF.psmDetailed);
    sg.crouch = adapter::PsmCrouchStance(aF.psmDetailed) || g_slideFrame;
    sg.blocked = g_trav.active || g_release || !aB.rootFrameValid || !g_ptSk.ok;
    g_pt.Step(g_ptParams, sg, aDt);
    const bool drawn = aF.haveWeapon && aF.weaponDrawn;
    g_ptWhy = sg.aiming ? "aim" : drawn ? "drawn" : sg.weaponBusy ? "wpnact" : sg.air ? "air" : sg.crouch ? "crouch" : g_trav.active ? "trav"
            : g_release ? "release" : !aB.rootFrameValid ? "noroot" : !g_ptSk.ok ? "noskel" : g_pt.MotionReason();
}

// Body yaw offset, feed rate: weight 1 on foot (every stance / gait, weapon holstered or drawn), 0 while aiming, during a weapon action
// (shot, reload, hip-fire pose or its hold, melee: aMelee = the melee definition of the H2 inputs) and blocked (traversal, release, no
// root frame). Offset 0 and faded: nothing runs.
void BodyOffsetStepLocked(const Bundle& aB, const adapter::MotionFeed& aF, bool aMelee, float aDt)
{
    if (g_bodyOffsetDeg == 0.0f && g_bo.Weight() == 0.0f)
    {
        g_boWhy = "off";
        return;
    }
    owarp::Signals sg;
    sg.aiming = g_aimingNow || aF.aiming;
    sg.weaponBusy = g_weaponAction == 2 || g_weaponAction == 8 || g_hipFirePoseNow || g_hipFirePoseHold.Active() ||
                    g_reloadUpperWeight > 0.0f || aMelee;
    // traversal: the montage branch resets the weight; after the handback it stays 0 while the handback blend runs (no foot contacts /
    // the montage root yaw turned back, kTravBlendOut), so the offset never snaps in on top of that blend
    const bool trav = g_trav.active || g_trav.noContactLeft > 0.0f || g_trav.yawBackT >= 0.0f;
    sg.blocked = trav || g_release || !aB.rootFrameValid;
    g_bo.Step(g_boParams, sg, aDt);
    g_boWhy = sg.aiming ? "aim" : aMelee ? "melee" : sg.weaponBusy ? "wpnact" : trav ? "trav" : g_release ? "release"
            : !aB.rootFrameValid ? "noroot" : "ok";
}

// motion lean, feed rate: estimator + gates + blend space weights (Step) and the additive on the v2 pose of this feed (Apply). Called
// right after the pose evaluator and the root commit, so the orientation warp measures the leaned pose and the leg tail (stance guard,
// foot locker) sees it. OFF and faded out: returns before touching anything (pose bit-identical).
void LeanStepLocked(Bundle& aB, const motion::ChooserState& aCs, const motion::MatchResult& aR, const adapter::MotionFeed& aF, float aDt)
{
    const bool headOwns = g_leanSetting && g_leanHeadSetting && aB.leanOk && aB.lean.HasHead();
    g_leanHeadOwns = headOwns; // HeadLeadStepLocked of this feed blocks the head lead in favour of the lean head layer
    if (!aB.leanOk || (!g_leanSetting && aB.lean.Idle()))
        return; // unavailable, or off and fully faded: nothing runs
    lrt::Signals sg;
    sg.vx = aF.velWorld[0];
    sg.vy = aF.velWorld[1];
    sg.velValid = std::isfinite(sg.vx) && std::isfinite(sg.vy);
    sg.moving = aCs.movementState == 1 && aCs.movementMode == 0; // Moving on the ground (motion: MovementState Moving)
    sg.aiming = g_aimingNow || aF.aiming;
    sg.weaponBusy = g_weaponAction == 2 || g_weaponAction == 8 || g_hipFirePoseNow || g_reloadUpperWeight > 0.0f || g_weaponUpperFrame ||
                    (aF.haveWeapon && aF.weaponDrawn);
    sg.air = (aF.air.valid && !aF.air.onGround) || adapter::PsmAir(aF.psmDetailed) || aCs.movementMode != 0;
    sg.crouch = adapter::PsmCrouchStance(aF.psmDetailed) || aCs.stance != 0;
    sg.slide = g_slideFrame;
    sg.blocked = g_trav.active || g_release || !g_leanSetting;
    sg.head = headOwns;
    if (aR.clip >= 0 && static_cast<size_t>(aR.clip) < aB.leanClipMap.size())
    {
        sg.curveClip = aB.leanClipMap[static_cast<size_t>(aR.clip)];
        sg.clipSeconds = aR.clipTime;
    }
    aB.lean.Step(aB.leanPack, g_leanParams, sg, aDt);
    if (aB.lean.Idle())
        return;
    ov::Pose p;
    aB.eval.ToOv(aB.pose.data(), p);
    if (aB.lean.Apply(aB.leanPack, g_leanParams, p))
        aB.eval.FromOv(p, aB.pose.data());
}

// ragdoll R0 probe, feed side (called only while g_probe.Active(), at the top of StepLocked, before every gate / skip return so a
// skipped Step still aborts): remembers the entity pose for the log, ends the run on an abort condition, else ticks its feed clock.
void ProbeStepLocked(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    if (aRaw.haveExtra && std::isfinite(aRaw.capsulePos[0] + aRaw.capsulePos[1] + aRaw.capsulePos[2]))
    {
        std::memcpy(g_probeCap, aRaw.capsulePos, sizeof g_probeCap);
        g_probeHaveCap = true;
    }
    if (std::isfinite(aRaw.playerYawDeg))
        g_probeYawDeg = aRaw.playerYawDeg;
    const uint32_t blk = aRaw.psmBlock;
    const char* why = nullptr;
    if (!g_enabled)
        why = "motion-off";
    else if (g_faulted)
        why = "fault";
    else if (!aRaw.tppActive)
        why = "not-tpp";
    else if ((blk & integ::kPsmVehicle) != 0)
        why = "vehicle";
    else if ((blk & integ::kPsmVitals) != 0)
        why = "death";
    else if ((blk & integ::kPsmPhotoOrMenu) != 0)
        why = "menu-photo";
    else if ((blk & (integ::kPsmScene | integ::kPsmSwimming | integ::kPsmWorkspot | integ::kPsmTakedown | integ::kPsmCarry |
                     integ::kPsmNoBlackboard)) != 0)
        why = "psm-block";
    else if (!adapter::MotionRuns(aEffGate))
        why = "gate-closed";
    else if (g_release)
        why = "overlay-release";
    else if (g_trav.havePending || g_trav.planned || g_trav.active || g_trav.episode || g_trav.yielding)
        why = "traversal";
    else if (!g_bundle || g_bundle.get() != g_probeBundle || g_bundle->body != g_body)
        why = "bundle-changed";
    if (why != nullptr)
    {
        ProbeEndLocked(why);
        return;
    }
    const rag::GetUpProbe::TickResult t = g_probe.Advance(aDt, ProbeWallS());
    if (t.ended)
    {
        ProbeEndLocked(t.reason == rag::ProbeEnd::Watchdog ? "watchdog" : "done");
        return;
    }
    if (t.logDue)
        g_probeLogDue = true;
    if (t.handBackStarted)
        ProbeLogLocked("hold-done", "handing back to MM", g_probeHeldValid ? &g_probeHeld : nullptr);
}

// native get-up, feed side (called only while g_gu.Active(), at the top of StepLocked next to the probe): remembers the live entity
// transform, ends the run on an abort condition, else ticks the clip clock on feed time (dt) and the early hand-off rules.
void GetUpStepLocked(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    if (aRaw.haveExtra && std::isfinite(aRaw.capsulePos[0] + aRaw.capsulePos[1] + aRaw.capsulePos[2]))
        g_guNoCapFeeds = 0;
    else
        ++g_guNoCapFeeds; // g_guCap / g_guHaveCap are refreshed for every Step at the top of StepLocked
    const uint32_t blk = aRaw.psmBlock;
    const char* why = nullptr;
    if (rag::TakeGetUpAbortRequest() == rag::kGetUpAbortBaseGate)
        why = "base-gate"; // the v1 gate was closed on the first get-up feed: MM does not own the body, the clip would not show
    else if (!g_enabled)
        why = "motion-off";
    else if (g_faulted)
        why = "fault";
    else if (!aRaw.tppActive)
        why = "not-tpp";
    else if ((blk & integ::kPsmVehicle) != 0)
        why = "vehicle";
    else if ((blk & integ::kPsmVitals) != 0)
        why = "death";
    else if ((blk & (integ::kPsmScene | integ::kPsmSwimming | integ::kPsmWorkspot | integ::kPsmTakedown | integ::kPsmCarry |
                     integ::kPsmNoBlackboard)) != 0)
        why = "psm-block";
    else if (rag::g_episode.Owns())
        why = "episode-restart"; // a new ragdoll took the body again: GameOwned() is false while the get-up flag is set
    else if (g_guNoCapFeeds >= 3)
        why = "no-capsule";
    else if (g_trav.havePending || g_trav.planned || g_trav.active || g_trav.episode || g_trav.yielding)
        why = "traversal";
    else if (!g_bundle || g_bundle.get() != g_guBundle || g_bundle->body != g_body)
        why = "bundle-changed";
    if (why == nullptr)
    {
        g_guGateClosed = adapter::MotionRuns(aEffGate) ? 0 : g_guGateClosed + 1; // a single closed feed is not an abort
        if (g_guGateClosed >= 3)
            why = "gate-closed";
    }
    if (why != nullptr)
    {
        GetUpEndLocked(why, true);
        return;
    }
    g_guPaused = (blk & integ::kPsmPhotoOrMenu) != 0; // menu / photo: the clock and the wall watchdog stop
    g_guTarget.OnFeed();                              // feeds since Begin (the release frame target is trusted for a bounded number of them)
    const rag::GuPlan& p = g_gu.Plan();
    float off = -1.0f;
    if (g_guHaveCap)
        off = std::hypot(g_guCap[0] - p.anchor[0], g_guCap[1] - p.anchor[1]);
    const bool input = std::hypot(std::isfinite(aRaw.move[0]) ? aRaw.move[0] : 0.0f, std::isfinite(aRaw.move[1]) ? aRaw.move[1] : 0.0f) >
                       rag::kGuInputStick;
    g_guYawErrDeg = rag::gu::WrapPi(g_guYawDeg * integ::kDegToRad - p.yaw) / integ::kDegToRad;
    const rag::GetUpPlayer::TickResult t = g_gu.Advance(aDt, input, off, ProbeWallS(), g_guPaused);
    if (t.ended)
    {
        GetUpEndLocked(t.reason ? t.reason : "ended", std::strcmp(t.reason ? t.reason : "", "done") != 0);
        return;
    }
    if (t.handOffStarted)
    {
        g_guHandOffEntry = true; // MM restarts fresh (like a traversal handback) while the held pose fades into it
        g_guInHandOff = true;
        g_guShownValid = false;
        g_guGraceLeft = rag::kGuGroundGraceS;
        GetUpLogLocked("HandOff", t.reason);
    }
    else if (t.blendDone)
        GetUpLogLocked("Play", nullptr);
    const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
    g_guLogClock += dt;
    if (g_guLogClock >= 0.5f)
    {
        g_guLogClock = 0.0f;
        g_guLogDue = true;
    }
}

// First search after a vehicle-exit re-entry: the PoseHistory of the game pose captured at the exit edge (see mm/ExitPoseHistory.hpp for
// the conventions). The capture is the frozen snapshot of the mount (its own bone layout), so every database bone is looked up through
// its game index (g_map: database bone -> game bone, the same map the live snapshot uses); Root / Trajectory are identity. null = the
// capture does not belong to the live rig or lacks a bone of a chain: the first search then keeps the database mean (previous behaviour).
// The history is read by the matcher during that one Update only (static storage, no allocation).
const motion::PoseHistory* VehicleExitHistoryLocked(const Bundle& b, const integ::PoseSnapshot& aCap)
{
    static motion::PoseHistory history;
    if (!g_map.ok || g_rig == 0 || aCap.rig != g_rig)
        return nullptr;
    const uint32_t nc = std::min<uint32_t>(aCap.numBones, integ::kMaxMMBones);
    const bool ok = motion::BuildExitPoseHistory(
        b.db, kFirstBone,
        [&](uint32_t i, math::Vec3& p, math::Quat& q) {
            if (i >= integ::kMaxMMBones || g_map.gameIndex[i] < 0)
                return false;
            for (uint32_t j = 0; j < nc; ++j)
            {
                if (aCap.gameIndex[j] != g_map.gameIndex[i])
                    continue;
                const float* c = aCap.pose + size_t(j) * 7;
                p = math::Vec3{c[0], c[1], c[2]};
                q = math::Quat{c[3], c[4], c[5], c[6]};
                return true;
            }
            return false;
        },
        motion::TrajSample{}, history);
    return ok ? &history : nullptr;
}

bool StepLocked(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    if (g_probe.Active())
        ProbeStepLocked(aRaw, aEffGate, aDt);
    // native get-up: the live entity transform of every feed (Plan reads it too), the run itself, the grace / tail clocks
    g_guLastTpp = aRaw.tppActive;
    if (aRaw.haveExtra && std::isfinite(aRaw.capsulePos[0] + aRaw.capsulePos[1] + aRaw.capsulePos[2]))
    {
        std::memcpy(g_guCap, aRaw.capsulePos, sizeof g_guCap);
        g_guHaveCap = true;
    }
    if (std::isfinite(aRaw.playerYawDeg))
        g_guYawDeg = aRaw.playerYawDeg;
    if (g_gu.Active())
    {
        GetUpStepLocked(aRaw, aEffGate, aDt);
        if (g_gu.Active())
            g_guGraceLeft = rag::kGuGroundGraceS; // a running get-up: no air / landing for MM (MapContext::ragdollRecover)
    }
    else if (g_guGraceLeft > 0.0f || g_guTailLeft > 0.0f)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
        g_guGraceLeft = std::fmax(0.0f, g_guGraceLeft - dt);
        g_guTailLeft = std::fmax(0.0f, g_guTailLeft - dt);
        if (g_guTailLeft <= 0.0f)
            g_guHeldValid = false;
    }
    if (!aRaw.tppActive)
        g_cameraTargetValid = false;
    if (!g_enabled || !g_shootTransferEnabled || !aRaw.tppActive || aEffGate != 0 || g_release)
        ResetAuthoredRecoil();
    integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
    g_gripProbe.supplied = g_gripProbe.applied = g_gripProbe.clamped = false;
    g_gripProbe.error = 0.0f;
    g_gripProbe.source = 0; g_gripProbe.weaponShift = 0.0f;
    g_gripProbe.requestedReach = g_gripProbe.usedReach = 0.0f;
    std::fill(std::begin(g_gripProbe.relativeTarget), std::end(g_gripProbe.relativeTarget), 0.0f);
    g_weaponUpperFrame = false; // every Step has its own admission; invalid/yielded frames cannot reuse the last one
    g_shootRouteNow = false;
    g_shootApplied = false;
    EmitLoadDoneLocked(); // UTPP-MOTION SEAM motionfix (item 4): only after a load (none while mm.motionDriver was never on)
    g_tailed = false;
    g_slideFrame = false;
    g_trav.frame = false; // UTPP-MOTION SEAM trav
    if (g_bundle)
        g_bundle->rootFrameValid = false;
    // air: the air feed belongs to this Step only (consumed even when v2
    // yields / is off)
    adapter::AirFeed air;
    if (g_airFresh)
        air = g_airIn;
    g_airFresh = false;
    g_airIn.valid = false;
    g_guAirDist = (air.valid && std::isfinite(air.distanceToGround)) ? air.distanceToGround : -1.0f; // native get-up: floor height under the capsule
    if (g_hoLeft > 0.0f && std::isfinite(aDt) && aDt > 0.0f)
        g_hoLeft = std::fmax(0.0f, g_hoLeft - std::fmin(aDt, 0.25f)); // handover clock: feed time, like the v1 fade
    if (g_hiLeft > 0.0f && std::isfinite(aDt) && aDt > 0.0f)
        g_hiLeft = std::fmax(0.0f, g_hiLeft - std::fmin(aDt, 0.25f)); // hand-in clock (commit A)
    if (!g_enabled)
    {
        g_pending = false;
        return false;
    }
    ++g_steps;
    // Only Reload gives the vanilla upper-body graph temporary ownership. Keep the lower-body MM motion and
    // return to the authored weapon pose after the state ends; Shoot needs a separate additive recoil path.
    if (std::isfinite(aDt) && aDt > 0.0f)
    {
        const bool reload = g_weaponAction == 2 && !g_release &&
            (g_pose == 1 || g_pose == 2 || g_pose == 3);
        // The aim-interrupt ramp / aim hold and the slide roll hold are measured on Nue pistols only (Pistol1H/2H);
        // a rifle reload keeps the former .12s ramp bit for bit.
        const bool pistol = g_pose == 2 || g_pose == 3;
        // g_aimingNow still holds the overlay aim state of the previous published frame (set in FrameInputsLocked).
        g_reloadUpperEnv.weight = g_reloadUpperWeight;
        g_reloadUpperWeight = g_reloadUpperEnv.Step(aDt, reload, g_aimingNow && pistol, pistol);
        g_reloadNowStep = reload && pistol;
        const bool rifle = g_pose == static_cast<int32_t>(ov::PoseOverlay::Rifle);
        g_slideTorsoReloadStep = reload_contacts_runtime::SlideTorsoReload(reload, pistol, rifle);
        g_reloadRifleStep = g_slideTorsoReloadStep && rifle;
        g_rollHoldStepped = false;
    }
    // Every return below (traversal montage, Skip, yield) still advances the roll hold, so it can neither freeze nor
    // leave a stale `active` that blocks the reset on the next reload edge. The normal path steps it with the real roll.
    struct RollHoldStepGuard
    {
        float dt;
        ~RollHoldStepGuard()
        {
            if (!g_rollHoldStepped && std::isfinite(dt) && dt > 0.0f)
            {
                g_reloadRollHold.Step(dt, g_reloadNowStep, false, 0.0f);
                g_slideDeRoll.Step(dt, false, 0.0f); // the torso counter-roll releases too (the pose of such a frame is not edited)
                g_rifleSlideLatch.Reset(); g_slideCarryAim.Reset();
                g_derollAppliedDeg = g_depitchAppliedDeg = 0.0f;
            }
        }
    } rollHoldGuard{aDt};
    // The same for the magazine top-up gesture: a Step that does not reach MagTapStepLocked still advances (fades) it.
    g_magTapStepped = false;
    struct MagTapStepGuard
    {
        float dt;
        ~MagTapStepGuard()
        {
            if (!g_magTapStepped)
                MagTapIdleStepLocked(dt);
        }
    } magTapGuard{aDt};
    if (std::isfinite(aDt) && aDt > 0.0f)
        g_reasonClock += std::fmin(aDt, 0.25f); // UTPP-MOTION SEAM motionfix (item 4): reason line rate window (feed time)
    // UTPP-MOTION SEAM trav: a pending traversal event ages on every enabled Step, also the ones skipped below (fallback /
    // gate / psm-release / air-owns / overlay-release), so it never outlives kTravEventMaxAge of real feed time
    // (an accepted plan still waiting for its detail ages the same way: stale -> v2 no longer owns the traversal)
    g_stepDetail = integ::PsmDetailOf(aRaw.psmBlock);
    if (g_trav.havePending || g_trav.planned)
    {
        if (std::isfinite(aDt) && aDt > 0.0f)
            g_trav.age += std::fmin(aDt, 0.25f);
        if (g_trav.age > kTravEventMaxAge)
        {
            g_trav.havePending = false; // the PSM never reached this action in time: stale
            if (!g_trav.planned && !g_trav.active && !g_trav.episode)
            {
                TravOwnLocked(false); // a deferred event aged out (contract: then v1 releases to the game animation)
                g_vehVaultEvt.store(false, std::memory_order_release); // vault torso limiter: never planned, nothing to keep
            }
            if (g_trav.planned)
            {
                g_trav.planned = g_trav.planOk = false;
                g_trav.plan = motiontrav::Plan{};
                g_trav.clip = -1;
                TravOwnLocked(false);
                // vault torso limiter: a vehicle vault is planned at the event and never started while the gate is closed, so it ages out
                // after 0.5 s of a ~0.7 s slide - the flag lives on until the PSM leaves the event's action (the episode is over)
                if (g_stepDetail != g_trav.ev.action)
                    g_vehVaultEvt.store(false, std::memory_order_release);
            }
        }
    }
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    if (g_testThrowNextStep) // unit test only: an exception inside the step (fault path)
    {
        g_testThrowNextStep = false;
        throw std::runtime_error("utpp mm.motionDriver test fault");
    }
#endif
    MaybeLoadLocked();
    BudgetPauseStepLocked(aDt); // feed budget: a pause ends on feed time, before this Step resolves its path
    const auto t0 = std::chrono::steady_clock::now(); // feed cost only (after a possible worker join / thread start)
    g_feedCyclesOk = QueryThreadCycleTime(GetCurrentThread(), &g_feedCycles0) != 0; // feed budget: the thread's own time
    g_reason = ResolveLocked();
    RefreshSlideLocked(); RefreshShooterModeLocked(); RefreshTravActiveLocked();
    LogReasonLocked(g_reason, nullptr); // UTPP-MOTION SEAM motionfix (item 4): every change (incl. -> none), was once per reason
    if (g_reason != Fallback::None)
    {
        Skip(g_skipGate, adapter::FallbackName(g_reason));
        return false;
    }
    // v2 runs exactly where v1 runs its matcher: gate open or hold-eligible (g_effGate 0); never opens the gate itself
    const int detail = integ::PsmDetailOf(aRaw.psmBlock);
    if (!adapter::MotionRuns(aEffGate))
    {
        Skip(g_skipGate, "gate");
        return false;
    }
    if (detail >= 0 && integ::PsmDetailReleases(detail))
    {
        Skip(g_skipPsm, "psm-release");
        return false;
    }
    // motionfix item 5: v2 owns air (no air-owns skip; the runtime never claimed air frames - main 22d4062 has no SetAirOwns
    // call - so the yield path was dead). Strafe precedence stays inside MapToChooser.
    if (AirMut() == 2 && air.valid && !air.onGround)
    {
        Skip(g_skipGate, "air-yield-mutation"); // mutation 2: v2 yields airborne feeds again (air test H must fail)
        return false;
    }
    if (g_release)
    {
        Skip(g_skipRelease, "overlay-release");
        return false;
    }
    Bundle& b = *g_bundle;
    const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
    const bool reentry = !g_wasRunning; // UTPP-MOTION SEAM trav: handback turn below starts only without a re-entry
    const bool rootWanted = aRaw.haveRootContext && aRaw.nativeRootEnabled;
    const bool rootDiscontinuity =
        rootWanted && (aRaw.dt > .25f || std::hypot(aRaw.capsuleDeltaCurrent[0], aRaw.capsuleDeltaCurrent[1]) > .5f);
    if (reentry || rootWanted != b.rootPolicyEnabled || rootDiscontinuity)
    {
        b.startTurn.Reset();
        // Shooter mode: V stands, the camera has orbited her and her visual root holds its heading (by design far from the capsule). A frame gap
        // or a re-entry after a skipped feed must not reseed that heading onto the actor (she would snap to face the camera); a real teleport
        // (displacement) and every other reseed keep the full reset.
        const bool teleported = rootWanted && std::hypot(aRaw.capsuleDeltaCurrent[0], aRaw.capsuleDeltaCurrent[1]) > .5f;
        (void)teleported;
#if UTPP_SHOOTER_MODE_MUTATION != 15
        if (g_shooter_modeSetting && rootWanted && g_fnHoldValid && !teleported && SteadyNs() - g_fnHoldNs <= kShooterModeHoldMaxNs)
            b.root.ResetHeld(aRaw.playerYawDeg * integ::kDegToRad, g_fnHoldVisualDeg * integ::kDegToRad);
        else
#endif
            b.root.Reset(aRaw.playerYawDeg * integ::kDegToRad);
        b.lastVelocityWorld[0] = b.lastVelocityWorld[1] = 0;
        if (!reentry)
        {
            b.matcher.Reset(true);
            b.eval.Reset();
            b.locomotion.Reset();
            b.map = adapter::MapState{}; g_stationaryShotTurn.Reset(); g_restOffsetOwned = false; g_restOwnerTail = 0.0f;
        }
    }
    b.rootPolicyEnabled = rootWanted;
    if (!g_wasRunning)
    {
        // re-entry after a skip / fallback: the matcher, inertializer and trajectory restart from this frame
        b.matcher.Reset(true); // preserve a completed one-shot transition across this gate/fallback re-entry
        b.eval.Reset();
        b.locomotion.Reset();
        // commit A v1 -> v2 hand-in from the last published non-v2 snapshot (a v2 -> v1 handover still in progress: its blend)
        g_hiLeft = g_haveLastBase ? motionhost::kDefaultHardOutTime : 0.0f;
        // commit A review: the start pose is the last PUBLISHED snapshot (ApplyLocked keeps it in g_lastBaseSnap on every
        // publish without a v2 pose: pure v1, or the v2 -> v1 handover blend still running), never the pure held v2 pose
        b.map = adapter::MapState{}; g_stationaryShotTurn.Reset();
        g_rootYawDeg = aRaw.playerYawDeg;
        g_hl.Reset(); // head lead: a re-entry starts without a carried correction
        g_hlHaveApply = false;
        g_pt.Reset(); // pelvis trim: the weight fades in again
        g_bo.Reset(); // body yaw offset: the yielded frames had none, the weight fades in again
        g_prevDb = -1;
        g_airEpisode = false; // air: a new air episode starts with the re-entry
        g_airPrevLanded = false;
#if UTPP_MOTION_DRIVER_TRAV_MUTATION != 8
        // UTPP-MOTION SEAM trav: a handback turn cut by a skip belongs to the MM blend the skip abandoned (inertializer reset
        // above, hand-in from the published pose): never resumed on re-entry. Mutation 8: kept (stale turn).
        g_trav.yawBackT = -1.0f;
#endif
    }
    // air: AirFeed from SetAirFeed (valid only on the Step it was sent for). Not probed (-2): the last hit, dead-reckoned
    // with the feed vz while airborne; a miss (-1) or landing clears it.
    const float vzFeed = std::isfinite(aRaw.velWorld[2]) ? aRaw.velWorld[2] : 0.0f;
    bool airSeedUsed = false;
    if (air.valid)
    {
        // J2 (vehicle batch): take-off plane seed for jumps, see MotionDriver.hpp AirFloorStep (shared with the offline replays)
        const AirFloorOut fl = AirFloorStep(g_airFloor, air.onGround, air.distanceToGround,
                                            adapter::DetailFromFeed(aRaw.psmBlock, aRaw.locoState), vzFeed, dt, AirMut());
        air.distanceToGround = fl.distance;
        airSeedUsed = fl.seedUsed;
    }
    // UTPP-MOTION SEAM trav: traversal state for this PSM detail (Climb 8 / Vault 9)
    const bool travDetail = detail == static_cast<int>(motiontrav::GameAction::Climb) || detail == static_cast<int>(motiontrav::GameAction::Vault);
    bool travHandback = false;
    if (!travDetail && (g_trav.active || g_trav.yielding || g_trav.handbackNext || g_trav.episode))
    {
        travHandback = g_trav.active || g_trav.handbackNext; // detail left 8 / 9 (montage cut short or just ended)
        TravResetLocked(false); // episode end: OwnsTraversal false
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 6
        TravOwnLocked(true); // mutation 6: ownership outlives the episode
#endif
    }
    else if (g_trav.handbackNext)
    {
        travHandback = true; // montage ended while the detail is still 8 / 9 (v2 MM keeps the episode: still owned)
        g_trav.handbackNext = false;
    }
    if (g_guHandOffEntry)
    {
        // native get-up hand-off (or an aborted get-up): MM restarts like after a traversal (fresh search, root / locomotion /
        // pelvis trim / body offset reset) while BlendFromHeldSpan fades the held get-up pose into it
        travHandback = true;
        g_guHandOffEntry = false;
    }
    // an event not planned in FeedTraversal (bundle not ready then, or an episode was running): plan it now, so the flag is
    // set for the next feed; it then waits for its detail (aged at the top of the Step)
    if (g_trav.havePending && !g_trav.planned && !g_trav.active && !g_trav.episode && !g_trav.yielding)
        TravPlanLocked(b);
    if (g_trav.planned && travDetail && g_trav.ev.action == detail)
    {
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 5
        static bool s_mut5Late = false;
        if (!s_mut5Late)
            s_mut5Late = true; // mutation 5: the montage starts one detail feed late
        else
        {
            s_mut5Late = false;
            TravStartLocked();
        }
#else
        TravStartLocked(); // Ok plan -> montage episode, else yielding
#endif
    }
    if (travDetail && g_trav.yielding)
    {
        // plan rejected / unsupported: v2 yields this traversal to the v1 path (OwnsTraversal false): with mm.climbRelease on,
        // or (vehicle batch, MotionHooks::travActive) whenever v2 traversal is active, the v1 gate releases Climb 8 / Vault 9 to the
        // game's climb / vault animation; otherwise v1 holds its own MM / hold pose
        Skip(g_skipTrav, "trav-yield");
        return false;
    }
    adapter::MotionFeed f = adapter::MakeMotionFeed(aRaw.velWorld, aRaw.move, dt, aRaw.psmBlock, aRaw.locoState, air,
                                      aRaw.cameraYawDeg, aRaw.playerYawDeg);
    // Quick fix (2026-10-23): through the vehicle exit window V holds the hip-fire pose (the same hold a shot starts), so the
    // jump-out (double F) shows it instead of the plain stance. A reload still clears it.
    // 2026-10-28 bike jump-out: the window pose is released in the flight (r5 air; before: at the landing) (HipFirePose.hpp JumpOutExitHold); a normal exit window is unchanged.
    g_jumpOutExitHold.Step(aRaw.vehicleExitWindow && aRaw.vehicleJumpOut,
        (f.air.valid && !f.air.onGround) || b.map.st.movementMode == 1, b.map.st.timeToLand,
        (f.air.valid && f.air.onGround) || adapter::PsmLand(f.psmDetailed) || b.map.st.justLanded);
    if (g_jumpOutExitHold.ReleaseEdge())
        g_hipFirePoseHold.Reset(); // the 0.75 s window hold must not outlive the release (a touchdown without a landing flag)
    const bool exitWindowPose = aRaw.vehicleExitWindow && !g_jumpOutExitHold.ReleasesExitHold();
    const int hipHoldAction = (exitWindowPose && g_weaponAction != 2) ? 8 : g_weaponAction;
    // R3 review: a sprint request ends a sustained (2 s) hold, the exit window is no shot of an engagement (HipFirePose.hpp).
    // Combat probe 2026-10-24 (shoot_jump): in the air, a traversal, on a ladder and in the landing window (JustLanded of the
    // previous feed, 0.3 s) no hold is kept, so the jump / landing clips own the arms.
    const int32_t holdPsm = f.psmDetailed;
    // The v2 slide is NOT excluded: hipfire between shots in a slide keeps the raised pose by design (motionfix 21, slide recoil test
    // "between-shots-in-slide"); the slide is a grounded mode for the hold.
    const bool holdGrounded = !adapter::PsmAir(holdPsm) && !adapter::PsmLand(holdPsm) &&
        !adapter::PsmTraversal(holdPsm) && !(holdPsm >= adapter::kPsmLadderFirst && holdPsm <= adapter::kPsmLadderLast) &&
        !(f.air.valid && !f.air.onGround) && !g_trav.active && b.map.st.movementMode != 1 &&
        !b.map.st.justLanded;
    g_hipFirePoseHold.Step(hipHoldAction, dt, !g_release && ov::RangedPose(OverlayLocked()) &&
        f.haveWeapon && f.weaponDrawn, adapter::PsmSprint(f.psmDetailed), hipHoldAction == 8 && g_weaponAction != 8,
        holdGrounded || exitWindowPose); // the exit window deliberately shows the pose through the jump-out (quick fix 2026-10-23)
    // motion WantsToWalk = the game's walk toggle (PSM permanent bool WalkToggled, batch 1b UTPPFeedLocomotionExtra; raw flag,
    // the game clears it on sprint / combat). An older script without the extra call: false (as before).
    f.wantsToWalk = aRaw.haveExtra && aRaw.walkToggled;
    // G5: Tier2 Slow (separate native, sent right before this feed) lowers the stand run cap; G2: the walk toggle speed (0 = off)
    f.tier2Slow = g_tier2Slow.load(std::memory_order_acquire);
    // G9: aim jog off -> AimWalk at the vanilla speed asks for the Walk gait (MotionAdapter MapToChooser)
    f.aimWalkVanilla = g_aimJogOff.load(std::memory_order_acquire);
    // UTPP-MOTION SEAM motionfix (item 1): Locomotion world target = the real input direction (mv is (0, |s|) under Locomotion)
#if UTPP_MOTION_DRIVER_FIX_MUTATION != 1
    f.haveTarget = aRaw.haveTarget && std::isfinite(aRaw.targetYawDeg);
    f.targetYawDeg = f.haveTarget ? aRaw.targetYawDeg : 0.0f;
#endif
    adapter::MapContext ctx;
    // H2 safeguard inputs. The native body turn is off (g_bodyTurnOn false, CET UTPPSetMMBodyTurn(false)) for several reasons: ADS / hip-fire
    // lease (excluded in the mapping), weapon turn off with a weapon in hand, kill switch, guard pause (those arm the safeguard) and every
    // other Locomotion blocker - free look / hold, stagger, traversal, vehicle, scene, FPP, melee poses - which do not. The Locomotion reason bits
    // tell them apart (read only); no probe (offline tests) counts every off state as armed. Melee (PSM melee bits / MeleeWeapon state)
    // never arms it.
    {
        bool armCause = true;
        if (g_locomotionProbe != nullptr)
        {
            constexpr uint32_t kArmReasons = freemove::kWeapon | freemove::kDisabled | freemove::kKilled | freemove::kNotInstalled;
            armCause = (g_locomotionProbe() & ~kArmReasons) == 0;
        }
        const bool melee = integ::PsmMeleeBits(aRaw.psmBlock) != 0 || integ::PsmMeleeStateOf(aRaw.psmBlock) >= 5;
        ctx.bodyTurnSwitchedOff = !aRaw.nativeBodyTurnSwitch && armCause && aRaw.tppActive && !melee;
        ctx.hipFireAny = g_weaponAction == 8 || g_hipFirePoseHold.Active();
        ctx.hipLease = g_hipLease;
    }
    // Chooser feedback and Steering share the last actually committed visual
    // root. Traversal handback reseeds BEFORE MapToChooser rather than after its
    // state predicates.
    if (rootWanted && travHandback)
        b.root.Reset(aRaw.playerYawDeg * integ::kDegToRad);
    if (rootWanted)
        g_rootYawDeg = b.root.PreviousVisualWorldYaw() / integ::kDegToRad;
    ctx.rootYawValid = std::isfinite(g_rootYawDeg);
    ctx.rootYawDeg = ctx.rootYawValid ? g_rootYawDeg : 0.0f;
    if (g_prevDb >= 0 && static_cast<uint32_t>(g_prevDb) < b.db.NumDatabases())
        ctx.currentDbPivots = adapter::DatabaseHasPivotsTag(b.db.DbName(static_cast<uint32_t>(g_prevDb)));
    adapter::MapConfig mc;
    mc.capsuleBrakingDeceleration = aRaw.capsuleBrakingDeceleration;
    mc.capsuleAcceleration = aRaw.capsuleAcceleration;
    // Slide feed: only with the setting on and a bundle that declares the slide section (else the pre-slide MapToChooser)
    mc.slide = g_slideSetting && b.slide.declared;
    mc.shooterMode = g_shooter_modeSetting;
    mc.h2Strafe = g_h2GuardSetting;
    mc.gameWalkTogglePoint = g_walkSpeed.load(std::memory_order_acquire);
    adapter::SetAnalogFloor(mc, g_minAnalogWalk.load(std::memory_order_acquire)); // analogFloorStick / analogFloorFrac; 0 = off
    // selection churn guards as a group (SetSelectionGuards): the gait return hold of the speed map and the matcher's guards
    const bool selectionGuards = g_selectionGuards.load(std::memory_order_acquire);
    if (!selectionGuards)
        mc.gaitReturnHoldS = 0.0f;
    b.matcher.SetSelectionGuards(selectionGuards);
    mc.slideSpec = b.slide;
    mc.slideDeceleration = mc.slide ? adapter::kSlideDecelerationMs2 : 0.0f;
    // fix/slide-exit: an unconfirmed 0.08-0.10 s !onGround drop at the slide stays ground; single-feed stance flicker (Crouch 3 <->
    // CrouchSprint 30) and the 0.2 s Dodge 7 from the crouch do not reach the chooser (tests/SlideExitTest.cpp)
    mc.slideAirGraceS = mc.slide ? adapter::kSlideAirGraceSeconds : 0.0f;
    mc.exitAirGraceS = adapter::kExitAirGraceSeconds;
    mc.groundDropGraceS = adapter::kGroundDropGraceSeconds; // a short unconfirmed contact loss (ledge edge, step-down) stays ground
    mc.contactHoldS = adapter::kContactHoldSeconds; // fix/jumpout-cam
    // the exit window of the gate can close on the first open feed; the grace keeps its own short tail
    g_exitAirTail = aRaw.vehicleExitWindow ? 0.6f : std::fmax(0.0f, g_exitAirTail - std::fmax(aRaw.dt, 0.0f));
    ctx.vehicleExit = g_exitAirTail > 0.0f;
    // after a ragdoll: a small drop to the floor is no fall (a running / just finished get-up, a running episode, the 1.5 s after the
    // knockdown detail ended - also on the vanilla recovery path). false everywhere else = unchanged.
    ctx.ragdollRecover = g_guGraceLeft > 0.0f || rag::RagdollRecoverActive();
    g_jumpOutWindowNow = aRaw.vehicleExitWindow && aRaw.vehicleJumpOut; // r9 arms fade input (FrameInputsLocked)
    ctx.jumpOutFlight = aRaw.vehicleExitWindow && aRaw.vehicleJumpOut; // fix/jumpout-cam: the cool-exit flight (B / LL / RL Start rows out, fall loop from the apex)
    mc.stanceFilterS = adapter::kStanceFilterSeconds;
    mc.stanceHoldInDodge = true;
    // P1 (fix batch 2): a clean Stand <-> Crouch edge reaches the chooser on its own feed, not 0.15 s later (tests/P1StanceLatencyTest.cpp)
    mc.stanceCleanEdge = true;
    mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
    mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    // Keep stance calibration changes continuous under the existing transition policy.
    // Physical capsule velocity remains current; only its game -> query scale changes.
    mc.stanceSpeedBlendTime = b.matcher.BlendTimeFor(motion::ChooserState{}, motion::ChooserState{});
    // AirState already learns gravity on the same feed, before calling v2. Reuse it instead
    // of predicting CP2077's ~16 m/s^2 flight with the adapter's standalone 9.81 default.
    const float gravity = aRaw.velWorld[2] > 0.0f ? aRaw.predictedGravityUp : aRaw.predictedGravityDown;
    if (std::isfinite(gravity) && gravity >= 8.0f && gravity <= 30.0f)
        mc.gravity = gravity;
    mc.visualStrafe = ov::UseHipFirePose(f.aiming, g_weaponAction, g_hipFirePoseHold.Active(),
        g_release, f.haveWeapon, f.weaponDrawn, OverlayLocked(),
        static_cast<ov::PoseOverlay>(g_resolvedAimPose));
    const auto facingOverlay = OverlayLocked();
    const bool facingWeapon = facingOverlay == ov::PoseOverlay::Rifle || facingOverlay == ov::PoseOverlay::Pistol1H ||
                              facingOverlay == ov::PoseOverlay::Pistol2H;
    const double facingCameraAge = RecoilClock() - g_cameraTargetTime;
    const bool facingEligible = rootWanted && aRaw.tppActive && !g_release && !g_trav.active &&
        f.haveWeapon && (!f.weaponDrawn || facingWeapon) &&
        g_cameraTargetValid && facingCameraAge >= 0 && facingCameraAge <= .25;
    const float shotSpeed = std::hypot(f.velWorld[0], f.velWorld[1]);
    const float shotMappedSpeed = shotSpeed * 100.f * adapter::SpeedScaleFor(mc,
        adapter::PsmCrouchStance(f.psmDetailed) ? 1 : 0, shotSpeed);
    const bool stationaryShotEligible = facingEligible && !f.aiming && ctx.rootYawValid && f.weaponDrawn && facingWeapon &&
        ov::RangedPose(static_cast<ov::PoseOverlay>(g_resolvedAimPose)) && adapter::PsmGround(f.psmDetailed) &&
        (!f.air.valid || f.air.onGround) && shotMappedSpeed <= motiontraj::kHasVelocity &&
        std::hypot(f.move[0], f.move[1]) <= adapter::kStickReleased;
    static_assert(mm::shotturn::kAdmitDegrees == motiontraj::kTurnInPlaceDeg, "shot turn admission = the ADS turn-in-place angle");
    g_stationaryShotTurn.Step(g_weaponAction, dt, stationaryShotEligible, f.cameraYawDeg, ctx.rootYawDeg,
                             mc.visualStrafe); // hipfire pose hold = the successor owner of the facing
    ctx.shotTurn = g_stationaryShotTurn.Active();
    ctx.shotTargetYawDeg = g_stationaryShotTurn.targetYawDeg;
    ctx.shotTurnSide = g_stationaryShotTurn.side;
    ctx.aimFacing = facingEligible && (f.aiming || mc.visualStrafe || ctx.shotTurn);
    // Who built the offset (g_restOffsetOwned): a facing owner of UTPP is active now, or the offset was built under one and no foreign
    // capsule turn came since. The offset is resolved (< the turn-in-place angle) -> not owned. A capsule turn faster than
    // kRestForeignTurnDegPerS without an owner is foreign: the game's RotateEntityToCamera at the end of a heading lock (consumable,
    // hover jump, vault, slide end) or the mouse in the follow-mode camera; it must not start a turn-in-place or the fallback
    // (V would turn her back to the camera with no input). UTPP_REST_OWNER_MUTATION=1: every offset counts as owned.
    const bool aimIntent = g_aim == static_cast<int32_t>(ov::AimState::Aiming) || f.aiming;
    {
        const bool facingOwner = ctx.aimFacing || aimIntent || g_hipFirePoseHold.Active() || g_stationaryShotTurn.Active();
#if defined(UTPP_REST_OWNER_MUTATION) && UTPP_REST_OWNER_MUTATION == 2
        g_restOwnerTail = 0.0f; // mutation 2: no owner grace
#else
        g_restOwnerTail = facingOwner ? kRestOwnerGraceS : std::fmax(0.0f, g_restOwnerTail - std::fmax(aRaw.dt, 0.0f));
#endif
        const float capsuleTurnDegPerS = aRaw.dt > 0.0f ? std::fabs(aRaw.capsuleYawDelta) / integ::kDegToRad / aRaw.dt : 0.0f;
        const float offsetDeg = motiontraj::RootDeltaYaw(aRaw.playerYawDeg, ctx.rootYawDeg);
#if defined(UTPP_REST_OWNER_MUTATION) && UTPP_REST_OWNER_MUTATION == 3
        // mutation 3 (the previous rule): resolved below the 50 deg admission angle even inside the grace, set by a live owner only
        if (!ctx.rootYawValid || offsetDeg < motiontraj::kTurnInPlaceDeg)
            g_restOffsetOwned = false;
        if (facingOwner)
            g_restOffsetOwned = true;
        else if (capsuleTurnDegPerS > kRestForeignTurnDegPerS && g_restOwnerTail <= 0.0f)
            g_restOffsetOwned = false;
#else
        // While an owner is active or has just dropped (grace) every capsule step is the owner's, also the one that carries the
        // offset over the 50 deg admission angle after the owner is gone (the lease's last step arrives a feed late): the offset is
        // owned. Afterwards it stays owned until it is folded below the fallback's knee (40 deg, hysteresis: the fallback has to be
        // allowed to finish), or a foreign turn comes.
        if (!ctx.rootYawValid)
            g_restOffsetOwned = false;
        else if (facingOwner || g_restOwnerTail > 0.0f)
            g_restOffsetOwned = true;
        else if (offsetDeg < motionRoot::kRestKneeLow / motionRoot::kDegToRad)
            g_restOffsetOwned = false;
        else if (capsuleTurnDegPerS > kRestForeignTurnDegPerS)
            g_restOffsetOwned = false;
#endif
        // Shooter mode: a standing V (no stick, nothing owning the facing) HOLDS her heading while the camera orbits her - the follow-mode
        // mouse turn is foreign at any speed, not only above kRestForeignTurnDegPerS, or a slow orbit after an aim episode would hand the
        // held offset to the rest turn-in-place / its fallback and turn her behind the camera.
#if UTPP_SHOOTER_MODE_MUTATION != 3 && UTPP_SHOOTER_MODE_MUTATION != 11
        if (g_shooter_modeSetting && !facingOwner && g_restOwnerTail <= 0.0f && std::hypot(f.move[0], f.move[1]) <= adapter::kStickReleased &&
            capsuleTurnDegPerS > kShooterModeOrbitDegPerS)
            g_restOffsetOwned = false;
#endif
    }
#if defined(UTPP_REST_OWNER_MUTATION) && UTPP_REST_OWNER_MUTATION == 1
    const bool restOwned = true;
#else
    const bool restOwned = g_restOffsetOwned;
#endif
#if defined(UTPP_REST_TURN_MUTATION) && UTPP_REST_TURN_MUTATION == 3
    const bool restAimBlock = false; // mutation 3: an aim intent does not exclude the rest TIP
#else
    const bool restAimBlock = aimIntent;
#endif
    // Rest turn-in-place permission (MotionAdapter.hpp restTurn): a ranged weapon in WeaponRight (the glue leaves resolvedAimPose at
    // Default for the grenade throw and the projectile launcher), no aim intent of any kind, nothing owning the facing, and an
    // offset that UTPP's own owners built.
    ctx.restTurn = rootWanted && aRaw.tppActive && !g_release && !g_trav.active && f.haveWeapon && f.weaponDrawn && facingWeapon &&
        ov::RangedPose(static_cast<ov::PoseOverlay>(g_resolvedAimPose)) && !restAimBlock && !ctx.aimFacing && restOwned;
#if UTPP_SHOOTER_MODE_MUTATION == 3
    // mutation 3 (the first design): the standing V turns in place behind the camera instead of holding her heading
    const bool shooterModeRest = g_shooter_modeSetting && rootWanted && aRaw.tppActive && !g_release && !g_trav.active && !restAimBlock && !ctx.aimFacing;
    ctx.restTurn = ctx.restTurn || shooterModeRest;
#endif
    motion::ChooserState cs = adapter::MapToChooser(f, mc, b.map, ctx); // UTPP-MOTION SEAM trav: not const (JustTraversed)
    g_shooterFrames += b.map.shooterModeActive ? 1u : 0u;
    g_h2Frames += b.map.h2Active ? 1u : 0u;
    g_lastBtSwitch = aRaw.nativeBodyTurnSwitch;
#if UTPP_SHOOTER_MODE_MUTATION == 13
    g_fnDrivingNs.store(SteadyNs(), std::memory_order_release); // mutation 13: FPP / skipped feeds still count as driving
#else
    g_fnDrivingNs.store(aRaw.tppActive ? SteadyNs() : 0, std::memory_order_release);
#endif
    g_fnLastGait.store(cs.gait, std::memory_order_release);
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    g_strafeProbe.rotationMode = static_cast<int32_t>(b.map.st.rotationMode);
    g_strafeProbe.shooterModeActive = b.map.shooterModeActive;
    g_strafeProbe.h2Active = b.map.h2Active;
    g_strafeProbe.gait = cs.gait;
    g_strafeProbe.actorYawDeg = aRaw.playerYawDeg;
    g_strafeProbe.shouldSpin = cs.shouldSpinTransition;
#endif
    if (b.map.airGraceActive)
        f.air.onGround = true; // fix/slide-exit: head lead / lean / orient warp below follow the ground the chooser kept (the v1 leg tail still sees the drop)
    g_slideFrame = b.map.slideActive; // false unless mc.slide: FrameLocked drops the foot contacts of a slide frame
    if (g_slideFrame)
        ++g_slideFrames;
    if (AirMut() == 5)
        cs.justLandedLight = cs.justLandedHeavy = false; // mutation 5: landing flags not forwarded
    if (travDetail && g_trav.active) // UTPP-MOTION SEAM trav: the montage owns this feed (MapToChooser still tracks PSM)
    {
        b.startTurn.Reset();
        b.root.Reset(aRaw.playerYawDeg * integ::kDegToRad);
        b.eval.SetGraphInputs({});
        b.locomotion.Reset();
        // pelvis trim / body yaw offset: a montage frame carries neither (FrameLocked's traversal path); weight 0 for the episode, the
        // body offset gate also holds it through the handback blend (reason trav), then both fade in again
        g_pt.Reset();
        g_bo.Reset();
        g_ptWhy = g_boWhy = "trav";
        return TravFrameLocked(b, aRaw, cs, f, dt, t0);
    }
    if (travHandback) // UTPP-MOTION SEAM trav: back to MM with JustTraversed, fresh search, inertialized
    {
        b.startTurn.Reset();
        cs.justTraversed = true;
        b.matcher.Reset(true); // a traversal handback must not replay a completed stance transition
        b.locomotion.Reset();
        g_rootYawDeg = aRaw.playerYawDeg; // root yaw re-seeded as on re-entry (not integrated over the montage)
        g_prevDb = -1;
        g_pt.Reset(); // pelvis trim / body yaw offset: no snap back in on the handback feed (montage cut short: no montage-branch reset)
        g_bo.Reset();
    }
    const uint32_t nt = adapter::BuildQueryTrajectory(b.map, b.qtimes, g_traj.data(), static_cast<uint32_t>(g_traj.size()));
    if (nt == 0)
    {
        Skip(g_evalFail, "no-trajectory");
        return false;
    }
    const bool startEligible = rootWanted && aRaw.nativeBodyTurnEnabled && aRaw.tppActive &&
        f.haveTarget && cs.movementMode == 0 && cs.stance == 0 && !f.aiming &&
        !mc.visualStrafe && !ctx.aimFacing && !travDetail && !travHandback &&
        b.map.st.rotationMode == motiontraj::RotationMode::OrientToMovement;
    b.startTurn.Step(aRaw.dt, startEligible, std::hypot(f.move[0], f.move[1]) > adapter::kStickReleased,
        cs.isStarting, shotSpeed, aRaw.playerYawDeg * integ::kDegToRad,
        f.targetYawDeg * integ::kDegToRad, cs.gait);
    cs.startTurnDirection = b.startTurn.direction;
    cs.committedStartTurnClip = b.startTurn.clip;
    // Re-entry after a normal vehicle exit: the first search gets the captured exit pose (the matcher was just reset, so its pose channels
    // would be the database mean). Only the re-entry feed itself, on the ground, without a traversal / get-up handback; the jump-out
    // launch never offers a pose (MMDriver MotionStepLocked). Everything else passes nullptr as before.
    const motion::PoseHistory* exitHistory = nullptr;
    if (reentry && !travHandback && !travDetail && cs.movementMode == 0 && !aRaw.vehicleJumpOut && aRaw.vehicleExitPose)
    {
        exitHistory = VehicleExitHistoryLocked(b, *aRaw.vehicleExitPose);
        ++g_vehExitSeeds[exitHistory ? 1 : 0];
        if (g_log)
        {
            char line[160];
            std::snprintf(line, sizeof line, "UTPP MOTION vehicle exit seed: %s bones=%u (seeded %llu, mean %llu)",
                          exitHistory ? "pose history" : "capture unusable, mean query", exitHistory ? exitHistory->numBones : 0u,
                          static_cast<unsigned long long>(g_vehExitSeeds[1]), static_cast<unsigned long long>(g_vehExitSeeds[0]));
            g_log(line);
        }
    }
    motion::MatchResult r = b.matcher.Update(dt, g_traj.data(), nt, cs, exitHistory); // UTPP-MOTION SEAM trav: copy (handback blend)
    b.startTurn.Selected(r.clip, b.matcher.StartTurnDirection(r.clip));
    g_travProbe.justTraversed = travHandback; // UTPP-MOTION SEAM trav
    if (travHandback) // UTPP-MOTION SEAM trav
    {
        r.jumped = true; // new timeline: the inertializer's target-prev must not be the montage's previous raw pose
        if (!r.blend.valid || !(r.blend.blendTime > 0.0f))
        {
            r.blend.valid = true;
            r.blend.blendTime = kTravBlendOut;
        }
        ++g_travProbe.handbacks;
        g_trav.noContactLeft = kTravBlendOut; // [P] the leg tail must not lock feet of a pose still blending out
        g_trav.yawBack = g_trav.lastYaw; // verifier round 2: the montage's last root-children yaw, turned back after Evaluate
#if UTPP_MOTION_DRIVER_TRAV_MUTATION == 9
        (void)reentry;
        g_trav.yawBackT = 0.0f; // mutation 9: the turn also starts on a re-entry feed
#else
        // the turn continues the last published montage frame. On a re-entry feed (the handback feed itself was skipped:
        // Skip gives the episode up but handbackNext survives) the published pose is the v1 / handover pose and the hand-in
        // blends from it, so turning by the last montage yaw would twist the body: no turn
        g_trav.yawBackT = reentry ? -1.0f : 0.0f;
#endif
    }
    else if (g_trav.noContactLeft > 0.0f)
        g_trav.noContactLeft = std::fmax(0.0f, g_trav.noContactLeft - dt);
    g_last = r;
    g_selDiag.gait = cs.gait;
    g_selDiag.movementState = cs.movementState;
    g_selDiag.movementMode = cs.movementMode;
    g_selDiag.stance = cs.stance;
    g_selDiag.isStarting = cs.isStarting;
    g_selDiag.isPivoting = cs.isPivoting;
    g_selDiag.shouldSpin = cs.shouldSpinTransition;
    g_selDiag.speedCmS = cs.speed2dCmS;
    g_selDiag.interrupt = static_cast<int32_t>(r.interrupt);
    g_selDiag.searched = r.searched;
    g_selDiag.jumped = r.jumped;
    g_selDiag.cost = r.cost < 1e30f ? r.cost : -1.0f;
    g_selDiag.continuingCost = r.continuingCost < 1e30f ? r.continuingCost : -1.0f;
    g_selDiag.turnHold = r.turnHold;
    AirStepLocked(b, cs, air, vzFeed, dt);
    g_airDiag.chooserDbMask = r.dbMask;
    g_airDiag.takeoffSeed = airSeedUsed;
    if (r.clip >= 0 && r.database >= 0)
    {
        g_prevDb = r.database;
    }
    auto in = FrameInputsLocked(b, cs, f, dt);
    if (std::hypot(aRaw.velWorld[0], aRaw.velWorld[1]) > 1e-6f)
    {
        b.lastVelocityWorld[0] = aRaw.velWorld[0];
        b.lastVelocityWorld[1] = aRaw.velWorld[1];
    }
    motionstack::GraphInputs graph;
    graph.enabled = rootWanted;
    graph.generalSteering = cs.movementState == 1 || cs.movementMode == 1;
    graph.tipSteering = r.database >= 0 && uint32_t(r.database) < b.db.NumDatabases() &&
                        b.db.DbName(uint32_t(r.database)).find("TurnInPlace") != std::string_view::npos;
    graph.previousVisualWorldYaw = b.root.PreviousVisualWorldYaw();
    graph.actorWorldYaw = aRaw.playerYawDeg * integ::kDegToRad;
    graph.targetWorldYaw = adapter::QuerySample(b.map.traj, .5f).yaw;
    graph.velocityWorldX = b.lastVelocityWorld[0];
    graph.velocityWorldY = b.lastVelocityWorld[1];
    b.eval.SetGraphInputs(graph);
    RootEvaluation rootEvaluation;
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    g_rootProbe = RootProbe{};
#endif
    if (rootWanted)
    {
        rootEvaluation.processor = &b.root;
        rootEvaluation.input.dt = aRaw.dt; // gap policy sees original, unclamped elapsed time
        rootEvaluation.input.actorYaw = graph.actorWorldYaw;
        rootEvaluation.input.capsuleDeltaCurrent = {aRaw.capsuleDeltaCurrent[0], aRaw.capsuleDeltaCurrent[1], 0};
        rootEvaluation.input.capsuleYawDelta = aRaw.capsuleYawDelta;
        // The pose is drawn with the capsule after this tick's requested turn: the frame stays relative to it (MotionRootFrame.hpp).
        rootEvaluation.input.capsuleYawAhead = std::isfinite(aRaw.capsuleYawAhead) ? std::clamp(aRaw.capsuleYawAhead, -1.5f, 1.5f) : 0.0f;
        // The offset seen by the pose frame changes its velocity at a bounded rate: a capsule that jumps in speed or direction carries the
        // body with it instead of moving it inside the frame (MotionOffsetRoot.hpp item 3).
        rootEvaluation.input.offsetAccel = motionRoot::kOffsetAccel;
        rootEvaluation.input.movingGround = b.map.st.isMoving && (cs.movementMode == 0 || b.map.slideActive);
        // A native idle upper mask does not own the MM legs or their root motion.
        // Only explicit native ADS/reload ownership uses the adapted Release policy.
        const auto overlay = OverlayLocked();
        const bool weaponOverlay = overlay == ov::PoseOverlay::Rifle || overlay == ov::PoseOverlay::Pistol1H ||
                                   overlay == ov::PoseOverlay::Pistol2H;
        // Match ApplyLocked's actual native-upper ownership. This montage policy
        // also affects translation; explicit gameplay facing is separate below.
        // A reload on the spot is not montage for the ROTATION policy (it would shed the hidden yaw offset in ~0.3 s), and
        // neither is one that began on the spot once V moves: mm/RootMontage.hpp, tests/ReloadRootHoldTest.cpp.
        const bool reloadBeganMoving = g_reloadRootLatch.Step(g_reloadUpperWeight, rootEvaluation.input.movingGround);
        rootEvaluation.input.montage = rootframe::RootMontage(weaponOverlay, g_aimGameUpper, g_aimingNow, g_reloadUpperWeight,
                                                              rootEvaluation.input.movingGround, reloadBeganMoving);
        // A live TPP aim request owns visible facing even without a weapon or
        // native upper-body handback. Require the camera heartbeat and current
        // PSM ADS / admitted hipfire intent, not merely a configured overlay.
        // Known empty hands may zoom; drawn non-ranged/unknown weapon states and
        // ordinary idle upper masks retain the locomotion root policy.
        // Shooter mode / H2 safeguard: while V moves on the ground the visual root is locked onto the actor (the moving-aim branch of
        // MotionOffsetRoot) at a bounded rate; standing still (no input) she keeps her heading wherever the camera takes the capsule.
#if UTPP_SHOOTER_MODE_MUTATION == 4
        const bool strafeLock = false;
#else
        const bool strafeLock = (b.map.shooterModeActive || b.map.h2Active) && rootEvaluation.input.movingGround && !b.map.slideActive;
#endif
        rootEvaluation.input.aimFacing = (ctx.aimFacing || strafeLock) && cs.movementMode == 0 && (cs.stance == 0 || cs.stance == 1);
        // in the air the root follows the actor at the motion air rotation rate (no ground lock there); the offset at touchdown stays small
#if UTPP_SHOOTER_MODE_MUTATION == 16
        const bool airLock = false; // mutation 16: the root keeps its heading in the air
#else
        const bool airLock = b.map.shooterModeActive && cs.movementMode == 1 && !b.map.slideActive && !ctx.aimFacing;
#endif
        rootEvaluation.input.airFollow = airLock;
        rootEvaluation.input.lockMaxRate = airLock ? kShooterModeAirRate : (strafeLock && !ctx.aimFacing ? kShooterModeLockRate : 0.0f);
#if UTPP_SHOOTER_MODE_MUTATION != 15
        rootEvaluation.input.holdOnGap = g_shooter_modeSetting && g_fnHoldValid && SteadyNs() - g_fnHoldNs <= kShooterModeHoldMaxNs;
#endif
        // A hidden yaw offset beyond the turn-in-place angle has no consumer while V stands and nothing owns the facing: the root folds
        // it back at a bounded pace (MotionOffsetRoot.hpp, tests/RestOffsetCollapseTest.cpp). Only with a RANGED WEAPON in WeaponRight
        // (the offsets come from the hipfire lease / reload / ADS flows; unarmed states keep the root as before; the glue maps a
        // grenade throw and the projectile launcher to Pistol1H too but leaves resolvedAimPose at Default), on the ground,
        // standing / crouching.
        rootEvaluation.input.restRecenter = weaponOverlay && ov::RangedPose(static_cast<ov::PoseOverlay>(g_resolvedAimPose)) &&
                                            restOwned && !restAimBlock && cs.movementMode == 0 && (cs.stance == 0 || cs.stance == 1) &&
                                            !b.map.slideActive; // an aim intent (the ADS turn owns it) excludes the fallback too
#if UTPP_SHOOTER_MODE_MUTATION == 3
        rootEvaluation.input.restRecenter = rootEvaluation.input.restRecenter ||
            (shooterModeRest && cs.movementMode == 0 && (cs.stance == 0 || cs.stance == 1) && !b.map.slideActive);
#endif
        rootEvaluation.input.restTurnPlaying = graph.tipSteering; // the current database is a TurnInPlace one: its clip owns the yaw
        rootEvaluation.haveActorAim = in.haveAim && in.haveActorAimPoint;
        rootEvaluation.actorAim = in.actorAimPoint;
        in.preOverlay = &PrepareRootOverlay;
        in.preOverlayContext = &rootEvaluation;
    }
    // air: v2 owns air (motionfix item 5: always while mm.motionDriver is on),
    // so the touchdown blend (first blend request of the justLanded window) skips
    // the inertializer interruption deficit: back-to-back air re-picks just
    // before touchdown would otherwise shorten the landing blend to 0 s (UE
    // deficit rule) and pop the pose (v1 leg context, integ 7g; air test J).
    // Scope (verifier minor 2): armed on the touchdown Step, it stays armed for
    // the rest of the 0.3 s window only while no blend request consumes it, so a
    // landing pick that comes a few feeds after touchdown (no transition on the
    // touchdown Step) also gets its full blend; at most one request per landing
    // skips the deficit, and skipping it only lengthens that one blend. AirDiag
    // landBypass* report which Step consumed it (air test F: the touchdown feed,
    // once per landing).
    const bool landed = cs.justLandedLight || cs.justLandedHeavy;
    if (landed && !g_airPrevLanded && AirMut() != 8)
        b.eval.SetIgnoreDeficitOnNextBlend(true);
    else if (!landed)
        b.eval.SetIgnoreDeficitOnNextBlend(false);
    g_airPrevLanded = landed;
    const bool bypassArmed = b.eval.IgnoreDeficitArmed();
    motionpose::FrameInfo info;
    const bool ok = b.eval.Evaluate(dt, r, in, b.pose.data(), static_cast<uint32_t>(b.pose.size()), &info);
    g_ovArmsNow = ok && OverlayArmsOf(b, info);
    RecordGripInfoLocked(info);
    RecordBarrelProbeLocked(in, info, rootWanted ? &rootEvaluation : nullptr);
    // UTPP-MOTION SEAM motionfix item 2: aim offset diagnostics (status ao=)
    g_aoFrames += info.aoApplied ? 1u : 0u;
    g_aoYaw = info.aoYawDeg;
    g_aoU = info.aoSweepU;
    g_aoW = info.aoWeight;
    TravYawBackLocked(b, dt); // UTPP-MOTION SEAM trav: verifier round 2, handback turn of the last montage root yaw
    g_airDiag.landBypassArmed = bypassArmed;
    g_airDiag.landBypassConsumed = bypassArmed && !b.eval.IgnoreDeficitArmed();
    g_airDiag.landBypasses += g_airDiag.landBypassConsumed ? 1u : 0u;
    if (!info.finite)
        FaultLocked("fault:non-finite-pose"); // session-sticky
    if (!ok)
    {
        Skip(g_evalFail, info.finite ? "no-pose" : "fault");
        return false;
    }
    if (rootEvaluation.prepared)
    {
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
        {
            const auto& ri = rootEvaluation.input;
            RootProbe p;
            p.prepared = true;
            p.dt = ri.dt;
            p.capsuleDelta[0] = ri.capsuleDeltaCurrent.x, p.capsuleDelta[1] = ri.capsuleDeltaCurrent.y;
            p.capsuleYawDelta = ri.capsuleYawDelta;
            p.animDelta[0] = ri.animationDeltaPreviousVisual.x, p.animDelta[1] = ri.animationDeltaPreviousVisual.y;
            p.animYawDelta = ri.animationYawDelta;
            p.yawAhead = rootEvaluation.proposal.Ahead();
            p.movingGround = ri.movingGround, p.montage = ri.montage, p.aimFacing = ri.aimFacing;
            const auto before = b.root.Frame(), after = rootEvaluation.proposal.Frame();
            p.before[0] = before.x, p.before[1] = before.y, p.before[2] = before.yaw;
            p.after[0] = after.x, p.after[1] = after.y, p.after[2] = after.yaw;
            g_rootProbe = p;
        }
#endif
        if (!b.root.Commit(rootEvaluation.proposal))
        {
            Skip(g_evalFail, "root-commit");
            return false;
        }
        b.rootFrame = rootEvaluation.proposal;
        b.rootHostSequence = aRaw.rootFeedSequence;
        b.rootFrameValid = true;
    }
    LeanStepLocked(b, cs, r, f, dt); // opt-in mm.lean: before the orientation warp measurement and the leg tail
    auto target = SampleLocomotion(b, r);
    if (b.eval.UsesLiveBase())
    {
        // Same live-player clocks and weights as the bones. ROOT is the DB
        // delta-to-rate approximation, including a stored velocity snapshot;
        // it is not the exact UE graph root-motion custom attribute.
        const auto& base = b.eval.BaseSignals();
        target.rootVel[0] = base.rootX; target.rootVel[1] = base.rootY;
        target.rootYawRate = base.rootYaw;
        target.contact[0] = base.left; target.contact[1] = base.right;
        b.locomotion.Reset(); // no second snapshot blend after the live stack
    }
    const bool noContact = (g_airDiag.movementMode == 1 && AirMut() != 4) || g_trav.noContactLeft > 0.0f;
    if (noContact)
    {
        b.locomotion.ClearContacts();
        target.contact[0] = target.contact[1] = 0.0f;
    }
    const auto& signals = b.locomotion.Update(target, r.jumped && r.blend.valid,
                                             info.inertialDuration, dt);
    g_rootYawDeg = b.rootFrameValid ? b.root.PreviousVisualWorldYaw() / integ::kDegToRad
                                    : motiontraj::NormalizeAxis(g_rootYawDeg + signals.rootYawRate * dt / integ::kDegToRad);
    g_fnHoldValid = g_shooter_modeSetting && b.rootFrameValid && cs.movementMode == 0 && cs.movementState == 0 && !g_trav.active &&
                    std::hypot(f.move[0], f.move[1]) <= adapter::kStickReleased;
    g_fnHoldVisualDeg = g_rootYawDeg;
    g_fnHoldNs = SteadyNs();
#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
    g_strafeProbe.rootYawDeg = g_rootYawDeg;
#endif
    HeadLeadStepLocked(f, dt);
    OrientWarpStepLocked(b, f, r.clip, dt);
    PelvisTrimStepLocked(b, f, dt);
    BodyOffsetStepLocked(b, f, integ::PsmMeleeBits(aRaw.psmBlock) != 0 || integ::PsmMeleeStateOf(aRaw.psmBlock) >= 5, dt);
    const bool mmOwnsArms = MMOwnsArmsLocked(aRaw, aEffGate, f);
    SlideReloadTorsoStepLocked(b, mmOwnsArms, dt);
    MagTapStepLocked(b, mmOwnsArms, dt); // last pose edit of the frame: FrameLocked / ApplyLocked publish b.pose as left here
    TraceLocked(b, cs, r, info, f, dt); // UTPP-MOTION SEAM motionfix (item 4): MOTION trace line (off unless SetTrace)
    return FinishFrameLocked(t0); // UTPP-MOTION SEAM trav: shared with traversal frames
}

// commit A: the v2 frame for the v1 leg tail. Bone i of the v2 pose must drive the same game bone as v1 bone i (shared topology,
// otherwise v2 yields this feed (logged once) so a v2 pose never reaches a stage built for another layout.
bool FrameLocked(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    if (!g_enabled || !g_pending || !g_bundle || !g_map.ok || aBaseGameIndex == nullptr)
        return false;
    Bundle& b = *g_bundle;
    const uint32_t n = static_cast<uint32_t>(b.pose.size());
    bool same = n == aBaseBones && n <= integ::kMaxMMBones;
    for (uint32_t i = 0; i < n && same; ++i)
        same = aBaseGameIndex[i] == g_map.gameIndex[i];
    if (!same)
    {
        if (!g_layoutLogged && g_log != nullptr)
            g_log("UTPP mm.motionDriver tail refused: motion bone layout differs from the base database (base path)");
        g_layoutLogged = true;
        Skip(g_skipLayout, "base-layout");
        return false;
    }
    if (aOut.buf == nullptr || n > aOut.bufCap)
        return false; // no caller buffer: never hand out a pointer into the bundle (a worker swap / free may follow)
    // copied under g_m: the pose stays valid after the lock is released, whatever the worker does with the bundle
    std::copy(b.pose.begin(), b.pose.end(), aOut.buf);
    aOut.pose = aOut.buf;
    aOut.numBones = n;
    aOut.clipClass = steer::kClassNone;
    if (g_trav.frame) // UTPP-MOTION SEAM trav: montage frame - root motion = warped plan root, no foot contacts / replant
    {
        aOut.rootVel[0] = g_trav.vel[0];
        aOut.rootVel[1] = g_trav.vel[1];
        aOut.travFrame = true; // MMDriver offset root: animVel = its (capped) capsule velocity
        g_tailed = true;
        ++g_tailFrames;
        return true;
    }
    const motion::MatchResult& r = g_last;
    if (b.rootFrameValid)
    {
        const auto &root = b.rootFrame.Frame();
        aOut.providedRoot = true;
        aOut.rootFeedSequence = b.rootHostSequence;
        aOut.rootOffsetX = root.x;
        aOut.rootOffsetY = root.y;
        aOut.rootOffsetYaw = root.yaw;
        // opt-in orientation warp (mm.orientWarp): pelvis = extra visual root yaw, torso = twist over Spine .. Spine3 of this copy.
        // Idle (OFF, weight 0, nothing applied) leaves both bit-identical.
        const bool owLive = !g_ow.Idle() && g_owSkBundle == &b;
        // run pelvis yaw trim (mm.pelvisYawTrim): extra root yaw +t with the torso twisted back by t (only the pelvis turns), scaled
        // down where the warp owns the pelvis (run circles). Trim 0 / weight 0: exactly 0, the warp path below is the previous one.
        const float trim = (g_ptSkBundle == &b && g_ptSk.ok) ? g_pt.Deg(g_pelvisTrimDeg) * (1.0f - (owLive ? g_ow.Weight() : 0.0f)) : 0.0f;
        if (trim != 0.0f)
        {
            const float dp = (owLive ? g_ow.RootDeg() : 0.0f) + trim, tau = (owLive ? g_ow.TorsoDeg() : 0.0f) - trim;
            aOut.rootOffsetYaw = root.yaw + dp * integ::kDegToRad;
            owarp::OrientWarp::ApplyAngles(g_owParams, g_ptSk, reinterpret_cast<float*>(aOut.buf), n, dp, tau);
        }
        else if (owLive)
        {
            if (g_ow.RootDeg() != 0.0f)
                aOut.rootOffsetYaw = root.yaw + g_ow.RootDeg() * integ::kDegToRad;
            if (g_ow.TorsoDeg() != 0.0f || g_ow.RootDeg() != 0.0f)
                g_ow.Apply(g_owParams, g_owSk, reinterpret_cast<float*>(aOut.buf), n); // spine twist + leg counter-rotation
        }
        // body yaw offset (mm.bodyYawOffset): the whole pose turns as one rigid unit about the visual root, added on top of the
        // warp / trim root yaw only (their leg hold / torso twist never see it; no pose edit). Offset 0 / weight 0: exactly 0, skipped.
        const float body = g_bo.Deg(g_bodyOffsetDeg);
        if (body != 0.0f)
            aOut.rootOffsetYaw += body * integ::kDegToRad;
    }
    const auto& signals = b.locomotion.Output();
    aOut.contact[0] = signals.contact[0];
    aOut.contact[1] = signals.contact[1];
    aOut.rootVel[0] = signals.rootVel[0];
    aOut.rootVel[1] = signals.rootVel[1];
    aOut.rootYawRate = signals.rootYawRate;
    aOut.startTurnDirection = b.startTurn.direction;
    aOut.cancelStartSupport = b.eval.CancelledStartSupport();
    if (g_trav.noContactLeft > 0.0f)
        aOut.contact[0] = aOut.contact[1] = 0.0f;
    // air (report 3.15: foot placement off in air): no foot contact while the chooser is InAir (the runtime AirState
    // also sets contactScale 0 there); the leg tail then never locks a foot mid-jump
    const bool airNoContact = g_airDiag.movementMode == 1 && AirMut() != 4;
    if (airNoContact)
        aOut.contact[0] = aOut.contact[1] = 0.0f;
    // Slide frames: the feet slide over the ground, so the leg tail must not plant them (FootLocker contacts / replant / support
    // hold off; the pose keeps the animated feet). g_slideFrame is false unless mm.slide is on with a slide bundle.
    if (g_slideFrame)
        aOut.contact[0] = aOut.contact[1] = 0.0f;
    if (airNoContact || g_slideFrame || g_trav.noContactLeft > 0.0f)
        aOut.cancelStartSupport = -1;
    else if (r.clip >= 0 && static_cast<uint32_t>(r.clip) < b.db.NumClips())
        aOut.clipClass = steer::ClassifyClip(b.db.ClipName(static_cast<uint32_t>(r.clip)));
    if (r.database >= 0 && static_cast<uint32_t>(r.database) < b.db.NumDatabases())
    {
        // [P] replant Stops profile on the motion idle / stop databases (v1: clip class idle / stop)
        const std::string_view dn = b.db.DbName(static_cast<uint32_t>(r.database));
        aOut.stops = !airNoContact && (dn.ends_with("_Stops") || dn.ends_with("_Idles"));
    }
    g_tailed = true;
    ++g_tailFrames;
    return true;
}

// handover (see g_hoLeft): the v1 snapshot (layout, weight, mask) is kept; each v1 bone that the held v2 pose also
// carries (same game bone) is blended from the held v2 pose to the v1 pose, smoothstep like the v1 fade. Returns false: no
// v2 pose is published, the snapshot only converges on v1.
// BlendFromHeld: the same blend from any held snapshot (v1 -> v2 hand-in uses the last v1 snapshot).
// BlendFromHeldSpan: the same over aSpan seconds (the native get-up hands off over kGuBlendOutS, an aborted one over 0.12 s);
// BlendFromHeld below keeps the v1 hard fade-out span.
bool BlendFromHeldSpan(integ::PoseSnapshot& aSnap, const integ::PoseSnapshot& aHeld, float aLeft, float aSpan)
{
    if (aLeft <= 0.0f || aSnap.rig == 0 || aSnap.rig != aHeld.rig)
        return false;
    const uint32_t n = std::min(aSnap.numBones, integ::kMaxMMBones);
    const uint32_t m = std::min(aHeld.numBones, integ::kMaxMMBones);
    const float a = 1.0f - std::fmin(1.0f, aLeft / aSpan); // 0 = held pose
    const float s = a * a * (3.0f - 2.0f * a);
    for (uint32_t i = aSnap.firstBone; i < n; ++i)
    {
        const int32_t gi = aSnap.gameIndex[i];
        if (gi < 0)
            continue;
        uint32_t j = aHeld.firstBone;
        while (j < m && aHeld.gameIndex[j] != gi)
            ++j;
        if (j >= m)
            continue; // bone not driven by v2: v1 as is
        float* d = &aSnap.pose[size_t(i) * 7];
        const float* h = &aHeld.pose[size_t(j) * 7];
        for (int k = 0; k < 3; ++k)
            d[k] = h[k] + (d[k] - h[k]) * s;
        const float dot = h[3] * d[3] + h[4] * d[4] + h[5] * d[5] + h[6] * d[6];
        const float sign = dot < 0.0f ? -1.0f : 1.0f; // shortest arc
        float q[4], len = 0.0f;
        for (int k = 0; k < 4; ++k)
        {
            q[k] = h[3 + k] + (sign * d[3 + k] - h[3 + k]) * s;
            len += q[k] * q[k];
        }
        if (!(len > 1e-12f) || !std::isfinite(len))
            continue; // degenerate: v1 as is
        const float inv = 1.0f / std::sqrt(len);
        for (int k = 0; k < 4; ++k)
            d[3 + k] = q[k] * inv;
    }
    return true;
}

bool BlendFromHeld(integ::PoseSnapshot& aSnap, const integ::PoseSnapshot& aHeld, float aLeft)
{
    return BlendFromHeldSpan(aSnap, aHeld, aLeft, motionhost::kDefaultHardOutTime);
}

bool HandoverLocked(integ::PoseSnapshot& aSnap)
{
    // commit A: also after a toggle off (g_enabled false); the clock runs in StepLocked
    if (g_hoLeft <= 0.0f || !g_haveLastSnap)
        return false;
    if (!BlendFromHeld(aSnap, g_lastSnap, g_hoLeft))
        return false;
    ++g_handoverFrames;
    return true;
}

// ragdoll R0 probe, publish side (called only while g_probe.Active(), at the end of the normal ApplyLocked path, after g_lastSnap
// was taken from the MM pose, so MM / handover state never sees the probe pose). Hold: bones from kFirstBone = clip frame 0 with the
// root children displaced; the v1/v2 masks are forced to "v2 owns the whole body" (upperScale 1, no overlay arms / reload
// contacts) so no game-pose bone, leg tail or weapon correction pulls part of the body back to the capsule. HandBack: normal MM
// pose blended from the held probe pose (BlendFromHeld, kDefaultHardOutTime).
void ProbeApplyLocked(const Bundle& b, integ::PoseSnapshot& aSnap)
{
    const uint32_t n = static_cast<uint32_t>(b.pose.size());
    if (&b != g_probeBundle || n > integ::kMaxMMBones || n != b.db.NumBones())
    {
        ProbeEndLocked("bundle-changed");
        return;
    }
    if (g_probe.HandingBack())
    {
        if (g_probeHeldValid)
            (void)BlendFromHeld(aSnap, g_probeHeld, g_probe.HandBackLeftS());
        return;
    }
    if (!g_probe.Holding())
        return;
    const float* src = rag::GetUpFrame0(b.body == Body::Woman, g_probe.Clip()); // validated against the bone order at Start
    if (src == nullptr || n != rag::kGetUpBones)
    {
        ProbeEndLocked("clip-invalid");
        return;
    }
    (void)rag::ComposeProbePose(src, b.dbParents.data(), n, kFirstBone, g_probe.ComposeOffsetM(), aSnap.pose);
    aSnap.upperScale = 1.0f;
    aSnap.lowerScale = 1.0f; // melee full body (v1 fade in the reused snapshot) never reaches the probe pose
    aSnap.overlayArms = 0;
    aSnap.reloadContactsEligible = false;
    aSnap.reloadAuthoredClip = false;
    aSnap.reloadContactsWeight = 0.0f;
    g_probeHeld = aSnap;
    g_probeHeldValid = true;
    ++g_probeFrames;
    if (g_probeLogDue)
    {
        g_probeLogDue = false;
        ProbeLogLocked("tick", nullptr, &aSnap);
    }
}

// native get-up, publish side (called only while g_gu.Active(), at the end of the normal ApplyLocked path, after g_lastSnap was taken from
// the MM pose, so MM / handover state never sees the get-up pose). Blend / Play: the clip sampled at the clip time, crossfaded from the
// ragdoll snapshot with smoothstep, both anchored to the world through the LIVE entity transform of the last feed; then the floor safety
// net (whole-body lift, rate limited). HandOff: the normal MM pose in aSnap, blended from the last get-up pose (BlendFromHeldSpan).
// The masks are forced to "v2 owns the whole body" (like the probe), so no game-pose bone, leg tail or weapon correction pulls part of
// the body back to the capsule.
void GetUpApplyLocked(const Bundle& b, integ::PoseSnapshot& aSnap)
{
    const uint32_t n = static_cast<uint32_t>(b.pose.size());
    if (&b != g_guBundle || n > integ::kMaxMMBones || n != b.db.NumBones() || g_guSnapLocal.size() != size_t(n) * 7 ||
        g_guSnapRootWorld.size() != size_t(n) * 7)
    {
        GetUpEndLocked("bundle-changed", true);
        return;
    }
    g_guYieldSinceNs = 0;
    if (g_gu.Phase() == rag::GuPhase::HandOff)
    {
        if (g_guHeldValid)
        {
            // r13c M3: the MM stance differs from the clip's end stance (toes 24 vs 43 cm apart): a planted foot of the blend makes one short
            // lifted step instead of sliding (rag::HandOffFootStep: raise only, hips untouched)
            const bool footStep = g_guZGroundValid && g_guHeld.rig != 0 && g_guHeld.rig == aSnap.rig;
            if (footStep)
                std::memcpy(g_guMm.data(), aSnap.pose, size_t(n) * 7 * sizeof(float));
            const float left = g_gu.HandOffLeft();
            if (BlendFromHeldSpan(aSnap, g_guHeld, left, rag::kGuBlendOutS) && footStep)
            {
                const float a = 1.0f - std::fmin(1.0f, left / rag::kGuBlendOutS), w = a * a * (3.0f - 2.0f * a); // the weight BlendFromHeldSpan used
                const rag::GuShape shape{n, kFirstBone, b.dbParents.data()};
                g_guFootSteps += static_cast<uint32_t>(
                    rag::HandOffFootStep(shape, b.guClips.bones, g_guHeld.pose, g_guMm.data(), w, g_guZGround, aSnap.pose));
            }
            g_guShown = aSnap; // what is shown now: an abort from here fades from it
            g_guShownValid = true;
        }
        return;
    }
    const traversal::TravMontage* m = b.getup.Find(rag::kGuClipNames[g_gu.Plan().clip]);
    if (m == nullptr)
    {
        GetUpEndLocked("clip-missing", true);
        return;
    }
    rag::SampleGetUpPose(*m, b.getup.poseBones.size(), b.guClips.poseMap, static_cast<double>(g_gu.ClipTime()), g_guClip.data());
    // the entity frame of this publish: the live one of the last feed (a late or failed teleport, or the drop of the entity at the end,
    // never moves the body); the plan-time one only without any feed sample
    rag::GuFrame live;
    std::memcpy(live.cap, g_guCap, sizeof live.cap);
    live.yaw = g_guYawDeg * integ::kDegToRad;
    // the release: the script teleports the entity to the target in the Begin call, the feed samples catch up one frame later; the pose is
    // composed against the target until a sample matches it (a stale pre-teleport sample must not re-express the pose)
    const bool targetBefore = g_guTarget.pending;
    const rag::GuFrame frame = g_guTarget.Resolve(g_guPlanFrame, live, g_guHaveCap);
    const bool onTarget = targetBefore && g_guTarget.pending; // composed against the target, not a live sample
    rag::GuAnchor anchor;
    std::memcpy(anchor.pos, g_gu.Plan().anchor, sizeof anchor.pos);
    anchor.yaw = g_gu.Plan().yaw;
    const rag::GuShape shape{n, kFirstBone, b.dbParents.data()};
    rag::ComposeBlend(shape, g_guClip.data(), g_guSnapLocal.data(), g_guSnapRootWorld.data(), anchor, frame, g_gu.BlendW(), g_guOut.data());
    // floor safety net: ground plane in model space = anchor z - capsule z (the fresh air-feed distance under the capsule when there is one)
    const float zGround = (!onTarget && g_guAirDist >= 0.0f && g_guAirDist <= rag::kGuGroundMaxM) ? -g_guAirDist : anchor.pos[2] - frame.cap[2];
    g_guZGround = zGround;
    g_guZGroundValid = std::isfinite(zGround);
    const int64_t nowNs = SteadyNs();
    const float dtApply = g_guLastApplyNs > 0 ? std::fmin(0.1f, static_cast<float>(nowNs - g_guLastApplyNs) * 1e-9f) : 0.0f;
    g_guLastApplyNs = nowNs;
    // 1. the limbs: a hand / foot / elbow / knee that the local-space blend swings below the floor is raised on its own chain (FloorCorrect);
    // 2. the rest (torso, head, shoulders, a limb out of reach) goes to the whole-body lift (3 m/s, at most 0.25 m in the blend, 0.10 m after it)
    // (fingers under the floor alone also run the limb pass: its hand step owns them, rate limited through g_guHand; they never feed the lift.
    // The first apply after Begin has dtApply 0: the hand step gets one nominal 60 fps frame there, the lift none)
    const float dtHand = dtApply > 0.0f ? dtApply : 1.0f / 60.0f;
    int rawJ = -1, resJ = -1;
    const float raw = rag::FloorDeficit(shape, g_guOut.data(), b.guClips.bones, zGround, &rawJ);
    float residual = raw;
    resJ = rawJ;
    if (raw > 0.002f || rag::FingerDeficit(shape, g_guOut.data(), b.guClips.bones, zGround) > 0.002f || g_guHand.Active())
    {
        if (rag::FloorCorrect(shape, g_guOut.data(), b.guClips.bones, zGround, &g_guHand, dtHand) > 0)
            ++g_guLift.limbFrames;
        residual = rag::FloorDeficit(shape, g_guOut.data(), b.guClips.bones, zGround, &resJ);
    }
    const float wNow = g_gu.BlendW();
    if (raw > g_guLift.maxRaw)
        g_guLift.maxRawJoint = rawJ, g_guLift.maxRawW = wNow;
    if (residual > g_guLift.maxDeficit)
        g_guLift.maxJoint = resJ, g_guLift.maxJointW = wNow;
    g_guLift.maxRaw = std::fmax(g_guLift.maxRaw, raw);
    const float lift = g_guLift.Step(residual, dtApply, g_gu.Phase() == rag::GuPhase::Blend ? rag::kGuLiftMaxBlendM : rag::kGuLiftMaxM);
    if (lift > 0.0f)
        rag::AddLift(shape, lift, g_guOut.data());
    std::memcpy(&aSnap.pose[size_t(kFirstBone) * 7], &g_guOut[size_t(kFirstBone) * 7], size_t(n - kFirstBone) * 7 * sizeof(float));
    aSnap.upperScale = 1.0f;
    aSnap.lowerScale = 1.0f; // melee full body (v1 fade in the reused snapshot) never reaches the get-up pose
    aSnap.overlayArms = 0;
    aSnap.reloadContactsEligible = false;
    aSnap.reloadAuthoredClip = false;
    aSnap.reloadContactsWeight = 0.0f;
    g_guHeld = aSnap;
    g_guHeldValid = true;
    ++g_guFrames;
    if (g_guLogDue)
    {
        g_guLogDue = false;
        GetUpLogLocked("tick", nullptr);
    }
}

// The first publish after Begin when no v2 Step has run yet (g_pending is still false from the ragdoll yield): the yield branch of
// ApplyLocked would show the game pose here. At clip time 0 / blend weight 0 the get-up pose equals the ragdoll snapshot in world space, so
// it is written at once: the v1 layout of aSnap is replaced by the v2 layout like the normal path does, then GetUpApplyLocked composes the
// pose. False (aSnap untouched) = not a get-up gap frame or the run could not publish.
bool GetUpGapPoseLocked(integ::PoseSnapshot& aSnap)
{
#if defined(UTPP_MOTION_DRIVER_GAP_MUTATION) && UTPP_MOTION_DRIVER_GAP_MUTATION == 1
    (void)aSnap; // mutation 1: the yield branch shows the game pose until the first Step
    return false;
#else
    if (!g_gu.Active() || g_guHeldValid || g_gu.Phase() == rag::GuPhase::HandOff || !g_enabled || !g_bundle || g_bundle.get() != g_guBundle ||
        !g_map.ok || g_rig == 0 || aSnap.rig != g_rig || g_bundle->body != g_body)
        return false;
    const Bundle& b = *g_bundle;
    const uint32_t n = static_cast<uint32_t>(b.pose.size());
    if (n > integ::kMaxMMBones || n != b.db.NumBones() || g_guSnapLocal.size() != size_t(n) * 7 || g_guSnapRootWorld.size() != size_t(n) * 7 ||
        b.getup.Find(rag::kGuClipNames[g_gu.Plan().clip]) == nullptr) // the checks of GetUpApplyLocked, before aSnap is touched
        return false;
    aSnap.bodyType = b.body == Body::Woman ? kBodyWoman : kBodyMan;
    aSnap.source = integ::kPoseSourceDriver;
    aSnap.numBones = n;
    aSnap.firstBone = kFirstBone;
    for (uint32_t i = 0; i < integ::kMaxMMBones; ++i)
        aSnap.gameIndex[i] = i < n ? g_map.gameIndex[i] : -1;
    for (uint32_t i = 0; i < n; ++i) // the bones below kFirstBone (reference joints) keep MM's values, the rest is overwritten by the get-up pose
    {
        float* d = &aSnap.pose[size_t(i) * 7];
        d[0] = b.pose[i].p.x;
        d[1] = b.pose[i].p.y;
        d[2] = b.pose[i].p.z;
        d[3] = b.pose[i].q.x;
        d[4] = b.pose[i].q.y;
        d[5] = b.pose[i].q.z;
        d[6] = b.pose[i].q.w;
    }
    GetUpApplyLocked(b, aSnap);
    ++g_guGapFrames;
    g_guGapHeld = g_gu.Active() && g_guHeldValid;
    return g_guGapHeld;
#endif
}

bool ApplyLocked(integ::PoseSnapshot& aSnap)
{
    if (g_probe.Active() && g_probe.Watchdog(ProbeWallS())) // wall clock: also when no feed arrives (Tick republish under hold)
        ProbeEndLocked("watchdog");
    if (g_gu.Active() && g_gu.Watchdog(ProbeWallS(), g_guPaused)) // the same for the get-up
        GetUpEndLocked("watchdog-wall", true);
    aSnap.overlayArms = 0; // item 6: set below only for a steady two-hand v2 pose; v1 frames, blends and releases stay 0
    // g_pending stays set until a Step yields: a Tick republish under hold (no feed) re-applies the last v2 pose, like v1
    if (!g_enabled || !g_pending)
    {
        ProbeEndLocked("motion-yield"); // no v2 pose is published this frame: the probe does not own the body
        (void)HandoverLocked(aSnap); // blends aSnap while the v2 -> v1 handover runs (the get-up hold / tail below is applied over it)
        if (GetUpGapPoseLocked(aSnap))
            return true; // Begin was called, no Step yet: the get-up pose from the first publish on, never the game pose
        if (g_gu.Active())
        {
            // the get-up holds its last pose for at most 0.1 s of v2 yielding (a single stale feed must not drop it), then ends
            // (0.3 s while no get-up pose was published yet: Begin was just called and the first Step is still due)
            const int64_t nowNs = SteadyNs();
            if (g_guYieldSinceNs == 0)
                g_guYieldSinceNs = nowNs;
#if defined(UTPP_MOTION_DRIVER_GAP_MUTATION) && UTPP_MOTION_DRIVER_GAP_MUTATION == 2
            const bool shortGrace = g_guHeldValid; // mutation 2: the gap publish ends the first-Step grace
#else
            const bool shortGrace = g_guHeldValid && !g_guGapHeld;
#endif
            if (nowNs - g_guYieldSinceNs > (shortGrace ? 100000000 : 300000000)) // before the first Step-driven pose: the first Step is still due
                GetUpEndLocked("motion-yield", true);
            else if (g_guInHandOff && g_guShownValid)
                (void)BlendFromHeldSpan(aSnap, g_guShown, rag::kGuBlendOutS, rag::kGuBlendOutS); // hold exactly what was shown
            else if (g_guHeldValid)
                (void)BlendFromHeldSpan(aSnap, g_guHeld, rag::kGuBlendOutS, rag::kGuBlendOutS); // left == span: exactly the held pose
        }
        else if (g_guTailLeft > 0.0f && g_guHeldValid) // an aborted get-up while v2 yields (the script may still inject the knockdown detail)
            (void)BlendFromHeldSpan(aSnap, g_guHeld, g_guTailLeft, rag::kProbeHandBackS);
        // commit A: start pose of a later v1 -> v2 hand-in = this published snapshot (pure v1, or the running v2 -> v1
        // handover blend: a re-entry mid-handover continues from what was shown, not from the held v2 pose)
        // review minor A: at weight 0 the game pose is shown and v1's pipeline pose is the one of the last v1 matcher update
        // (stale for the whole span when the gate is closed: vehicle, menu, ...), so storing it would hand in from the
        // pre-span pose again. Drop it instead: the next re-entry has no hand-in and the v1 weight fade-in (from the game
        // pose) is the transition, as for v1 alone; any v1 publish at weight > 0 before it stores a fresh start pose.
        if (aSnap.weight > 0.0f)
        {
            g_lastBaseSnap = aSnap;
            g_haveLastBase = true;
        }
        else
            g_haveLastBase = false;
        return false;
    }
    if (!g_bundle || !g_map.ok || g_rig == 0 || aSnap.rig != g_rig || g_bundle->body != g_body)
    {
        ProbeEndLocked("motion-yield");
        GetUpEndLocked("motion-yield", true);
        if (aSnap.weight > 0.0f) // published as v1 (no v2 pose this publish): also the start pose of a later hand-in
        {
            g_lastBaseSnap = aSnap;
            g_haveLastBase = true;
        }
        else
            g_haveLastBase = false; // review minor A: game pose shown, no valid start pose (see above)
        return false;
    }
    const Bundle& b = *g_bundle;
    const uint32_t n = static_cast<uint32_t>(b.pose.size());
    aSnap.bodyType = b.body == Body::Woman ? kBodyWoman : kBodyMan;
    aSnap.source = integ::kPoseSourceDriver;
    aSnap.traceAction = g_weaponAction;
    aSnap.reloadContactsEligible = g_reloadContactsEligible && (g_weaponAction == 2 || g_weaponAction == 0);
    aSnap.reloadAuthoredClip = aSnap.reloadContactsEligible && g_authoredReload;
    aSnap.reloadContactsWeight = aSnap.reloadContactsEligible ?
        (g_authoredReload ? reload_contacts_runtime::AuthoredFadeWeight(g_reloadContactFade.weight)
                          : g_reloadContactFade.weight) : 0.0f;
    aSnap.traceOverlay = static_cast<int32_t>(OverlayLocked());
    aSnap.traceAiming = g_aimingNow ? 1 : 0;
    aSnap.traceClip = g_last.clip;
    // the v1 upper-body mask bits are database bone indices: kept only when both databases map to the same game bones.
    // with a weapon overlay active the upper body comes from MM + overlay (upperScale 1); game actions (melee,
    // body carry, ...) arrive as release and never reach this point. upperBits are v1's (written only by
    // BuildUpperMaskLocked) and stay untouched: upperScale 1 already makes them inert, and v1 needs them after a yield.
    bool sameBones = aSnap.numBones == n;
    for (uint32_t i = 0; i < n && sameBones; ++i)
        sameBones = aSnap.gameIndex[i] == g_map.gameIndex[i];
    const ov::PoseOverlay o = OverlayLocked();
    const bool weaponOverlay =
        o == ov::PoseOverlay::Rifle || o == ov::PoseOverlay::Pistol1H || o == ov::PoseOverlay::Pistol2H;
    // UTPP-MOTION SEAM motionfix item 2: mm.aimGameUpper hands the upper body back to the game graph + aimofs while
    // aiming: v1's upperScale (its aim mask fade, PSM UpperBody Aim) stays; the upper bits are v1's when both databases map
    // to the same game bones, else the same rule (integ::BuildUpperMask: Spine1 and above, arms, head) on the v2 bones
    const bool handback = weaponOverlay && g_aimGameUpper && g_aimingNow;
#if defined(UTPP_MOTION_DRIVER_FIX_MUTATION) && UTPP_MOTION_DRIVER_FIX_MUTATION == 7
    const bool handbackUsed = false; // mutation 7: aimGameUpper ignored
#else
    const bool handbackUsed = handback;
#endif
    if (handbackUsed)
        g_aimUpperTail = true;
    if (g_weaponUpperFrame)
        g_weaponUpperTail = true;
    const bool fadingWeapon = !g_hipFirePoseNow && g_weaponUpperOn && g_weaponUpperTail && aSnap.upperScale < 1.0f;
    if (!g_weaponUpperFrame && !fadingWeapon)
        g_weaponUpperTail = false;
    const bool fadingAim = g_aimUpperFade && g_aimGameUpper && g_aimUpperTail && aSnap.upperScale < 1.0f;
    if (!handbackUsed && !fadingAim)
        g_aimUpperTail = false;
    const bool reloadHandback = weaponOverlay && g_reloadUpperWeight > 0.0f;
    const bool reloadCorrectionTail = aSnap.reloadContactsEligible && g_reloadContactFade.Active();
    if (handbackUsed || fadingAim || g_weaponUpperFrame || fadingWeapon || reloadHandback || reloadCorrectionTail)
    {
        if (handbackUsed)
            ++g_aimHandback;
        if (!sameBones)
        {
            const integ::UpperMask m =
                integ::BuildUpperMask(n, kFirstBone, [&](uint32_t i) { return b.db.BoneName(i); });
            std::memcpy(aSnap.upperBits, m.bits, sizeof aSnap.upperBits);
            if (!m.ok)
            {
                aSnap.upperScale = 1.0f; // no Hips / Spine1: fail closed to the full v2 body
                aSnap.lowerScale = 1.0f;
            }
        }
        // Reload keeps the game's arm/hand/weapon action, but leaves the FPP-contorted spine/head
        // on MM/motion for a more natural TPP silhouette. Fall back to the established full-upper
        // mask if a rig is missing required hand/weapon bones.
        if ((reloadHandback || reloadCorrectionTail) && !handbackUsed)
        {
            const integ::UpperMask action = integ::BuildWeaponActionMask(
                n, kFirstBone, [&](uint32_t i) { return b.db.BoneName(i); });
            if (action.ok)
                std::memcpy(aSnap.upperBits, action.bits, sizeof aSnap.upperBits);
        }
    }
    else if (!sameBones || weaponOverlay)
    {
        aSnap.upperScale = 1.0f;
        aSnap.lowerScale = 1.0f; // the full v2 body: v1's melee full-body fade must not thin v2's legs / hips
    }
    // A safety fallback must own actual action records even when the persistent weapon-upper
    // option is OFF (the incoming v1 snapshot then normally has upperScale=1).
    const bool hipFireNativeFallback = g_hipFirePoseNow && g_weaponAction == 8 && g_weaponUpperFrame;
    if (hipFireNativeFallback)
    {
        const integ::UpperMask action = integ::BuildWeaponActionMask(
            n, kFirstBone, [&](uint32_t i) { return b.db.BoneName(i); });
        if (action.ok)
        {
            std::memcpy(aSnap.upperBits, action.bits, sizeof aSnap.upperBits);
            aSnap.upperScale = 0.0f;
        }
        else
        {
            aSnap.upperScale = 1.0f; // missing complete action mask: never yield unrelated torso/legs
            aSnap.lowerScale = 1.0f;
        }
    }
    if (reloadHandback)
    {
        // Smooth BOTH authored influences; smoothing correction alone leaves
        // the native upper-mask envelope's nonzero endpoint velocity intact.
        // The slide roll hold keeps the MM overlay arms (it is 0 outside a slide reload, so max() is the old value).
        const float reloadScale = std::fmax(1.0f - (g_authoredReload ?
            reload_contacts_runtime::AuthoredFadeWeight(g_reloadUpperWeight) : g_reloadUpperWeight), g_reloadRollHold.hold);
        // The v1 snapshot also contains its ADS mask even when v2 aimGameUpper
        // is OFF. Reusing that mask during the reload tail can reverse the fade
        // toward FPP, then snap to full MM ownership when the tail expires.
        const bool otherNativeOwner = handbackUsed || fadingAim || g_weaponUpperFrame ||
                                      fadingWeapon || hipFireNativeFallback;
        aSnap.upperScale = otherNativeOwner ? std::fmin(aSnap.upperScale, reloadScale) : reloadScale;
    }
    // UTPP-MOTION SEAM motionfix item 6: overlayArms only for a steady two-hand pose that v2 owns completely (no hand-in /
    // handover blend, no release, no game-upper / weapon-action / reload handback, full upper body)
#if defined(UTPP_MOTION_DRIVER_OVARMS_MUTATION) && UTPP_MOTION_DRIVER_OVARMS_MUTATION == 2
    aSnap.overlayArms = g_ovArmsNow ? 1u : 0u; // mutation 2: set through blends and the game-upper handback
#else
    aSnap.overlayArms = (g_ovArmsNow && !handbackUsed && !fadingAim && !g_weaponUpperFrame && !fadingWeapon && !reloadHandback &&
                         !reloadCorrectionTail && !hipFireNativeFallback && aSnap.upperScale >= 1.0f && g_hiLeft <= 0.0f &&
                         g_hoLeft <= 0.0f && !g_release) ? 1u : 0u;
#endif
    aSnap.numBones = n;
    aSnap.firstBone = kFirstBone;
    for (uint32_t i = 0; i < integ::kMaxMMBones; ++i)
        aSnap.gameIndex[i] = i < n ? g_map.gameIndex[i] : -1;
    for (uint32_t i = 0; i < n && !g_tailed; ++i) // commit A: tailed -> the snapshot already holds the leg-tail output
    {
        float* d = &aSnap.pose[size_t(i) * 7];
        d[0] = b.pose[i].p.x;
        d[1] = b.pose[i].p.y;
        d[2] = b.pose[i].p.z;
        d[3] = b.pose[i].q.x;
        d[4] = b.pose[i].q.y;
        d[5] = b.pose[i].q.z;
        d[6] = b.pose[i].q.w;
    }
    if (g_hiLeft > 0.0f && g_haveLastBase && BlendFromHeld(aSnap, g_lastBaseSnap, g_hiLeft))
        ++g_handinFrames; // commit A v1 -> v2 hand-in
    HeadLeadApplyLocked(b, aSnap); // opt-in mm.headLead: before g_lastSnap, so a v2 -> v1 handover blends from what was shown
    ++g_applied;
    g_lastSnap = aSnap;
    g_haveLastSnap = true;
    if (g_probe.Active()) // ragdoll R0 probe: the only per-frame cost when not commanded is this flag test
        ProbeApplyLocked(b, aSnap);
    if (g_gu.Active()) // native get-up: the same flag test (a run that never starts costs nothing else)
    {
        g_guGapHeld = false; // a Step-driven pose from here on
        GetUpApplyLocked(b, aSnap);
    }
    else if (g_guTailLeft > 0.0f && g_guHeldValid) // an aborted get-up: its last pose fades into MM over 0.12 s
        (void)BlendFromHeldSpan(aSnap, g_guHeld, g_guTailLeft, rag::kProbeHandBackS);
    return true;
}
} // namespace

// Game thread hooks: no exception may leave them (the caller is MMDriver::Feed / PublishLocked inside the game's frame).
bool Step(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    try
    {
        std::lock_guard lock(g_m);
        return StepLocked(aRaw, aEffGate, aDt);
    }
    catch (const std::bad_alloc&)
    {
        std::lock_guard lock(g_m);
        FaultLocked("fault:step-bad-alloc");
    }
    catch (...)
    {
        std::lock_guard lock(g_m);
        FaultLocked("fault:step-exception");
    }
    return false;
}

bool Frame(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    try
    {
        std::lock_guard lock(g_m);
        return FrameLocked(aBaseGameIndex, aBaseBones, aOut);
    }
    catch (...)
    {
        std::lock_guard lock(g_m);
        FaultLocked("fault:frame-exception");
    }
    return false;
}

bool ApplyToSnapshot(integ::PoseSnapshot& aSnap)
{
    try
    {
        std::lock_guard lock(g_m);
        return ApplyLocked(aSnap);
    }
    catch (...)
    {
        std::lock_guard lock(g_m);
        FaultLocked("fault:apply-exception");
    }
    return false;
}

// UTPP-MOTION SEAM trav
bool FeedTraversal(const TraversalEvent& aEv)
{
    if (aEv.action != 8 && aEv.action != 9)
        return false;
    const float* vs[] = {aEv.topPoint, aEv.collisionNormal, aEv.behindPoint, aEv.obstacleEnd, aEv.inputDirection,
                         aEv.leftHand, aEv.rightHand, aEv.playerPos, aEv.ownerForward};
    for (const float* v : vs)
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
            return false;
    if (!std::isfinite(aEv.topExtent) || !std::isfinite(aEv.speed) || !std::isfinite(aEv.climbSpeedModifier))
        return false;
    try
    {
        std::lock_guard lock(g_m);
        if (!g_enabled)
            return false; // v2 off: no state (v1 bit-identical)
        const bool episodeRuns = g_trav.active || g_trav.episode || g_trav.yielding;
        if (!episodeRuns && g_trav.planned) // a newer event replaces a plan that has not started yet
        {
            g_trav.planned = g_trav.planOk = false;
            g_trav.plan = motiontrav::Plan{};
            g_trav.clip = -1;
            TravOwnLocked(false);
        }
        g_trav.ev = aEv;
        g_trav.havePending = true;
        g_trav.age = 0.0f;
        ++g_travProbe.events;
        // task A: plan now when the body's bundle is ready and v2 is not skipping for a global reason, so OwnsTraversal is
        // already set when the next v1 Feed runs its gate; otherwise the next Step that reaches the traversal code plans it
        if (!episodeRuns && g_bundle && g_bundle->body == g_body && !g_release &&
            ResolveLocked() == Fallback::None)
        {
            try
            {
                TravPlanLocked(*g_bundle);
            }
            catch (...) // allocation in PlanTraversal: drop this traversal (v1 path), never leave the native
            {
                TravResetLocked(true);
            }
        }
        // deferred (not planned here): owned while v2 can still plan it at the next Step that reaches the traversal code, so
        // the v1 gate of the next feed does not release the body first (OwnsTraversal contract, main dc221fa)
        if (!episodeRuns && g_trav.havePending && !g_trav.planned)
            TravOwnLocked(TravCanPlanLaterLocked());
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// ragdoll R0 probe natives' bodies (see MotionDriver.hpp). Game thread (script VM), g_m like every other entry point.
std::string RagdollGetUpProbeStart(float aOffsetM, float aDurationS)
{
    try
    {
        std::lock_guard lock(g_m);
        const rag::ProbeArgs a = rag::SanitizeProbeArgs(aOffsetM, aDurationS);
        if (!a.ok)
            return "refused:bad-args";
        if (!g_enabled)
            return "refused:motion-off";
        if (g_faulted)
            return "refused:fault";
        if (!g_bundle || g_bundle->body != g_body || !g_bundle->db.Loaded())
            return "refused:no-bundle";
        if (!g_map.ok || g_rig == 0)
            return "refused:not-armed";
        if (!g_pending || g_release)
            return "refused:motion-not-running"; // no v2 pose is being published (gate / release / yield): nothing to take over
        if (g_trav.havePending || g_trav.planned || g_trav.active || g_trav.episode || g_trav.yielding)
            return "refused:traversal";
        const Bundle& b = *g_bundle;
        const uint32_t n = static_cast<uint32_t>(b.pose.size());
        if (n > integ::kMaxMMBones || n != b.db.NumBones() || b.dbParents.size() != n)
            return "refused:pose-size";
        // clip: set3 M_Neutral_GetUp_Back (1.5 s), frame 0 from the embedded table: the v2 databases carry no get-up clip
        // (stand_F/B/L/R, 5 s, are not baked either) and an asset pipeline is out of scope for a probe
        const int32_t clip = 0;
        if (!rag::GetUpBoneOrderMatches(n, [&](uint32_t i) { return b.db.BoneName(i); }))
            return "refused:bone-layout"; // the bundle's bone order is not the table's (re-bake): refuse instead of publishing garbage
        uint32_t hips = 0;
        bool haveHips = false;
        for (uint32_t i = kFirstBone; i < n; ++i)
            if (b.dbParents[i] < static_cast<int32_t>(kFirstBone))
            {
                if (!haveHips || b.db.BoneName(i) == "Hips")
                    hips = i;
                haveHips = true;
                if (b.db.BoneName(i) == "Hips")
                    break;
            }
        if (!haveHips)
            return "refused:no-root-child";
        if (g_probe.Active())
            ProbeEndLocked("restart");
        std::snprintf(g_probeClipName, sizeof g_probeClipName, "%s", rag::GetUpClipName(clip));
        g_probeHips = hips;
        g_probeBundle = g_bundle.get();
        g_probeHeldValid = false;
        g_probeLogDue = false;
        g_probeFrames = 0;
        g_probeHaveCap = false; // refreshed by the next Step (haveExtra); a stale position is never logged
        if (!g_probe.Start(a.offsetM, a.durationS, ProbeWallS(), clip))
            return "refused:bad-args";
        char detail[200];
        std::snprintf(detail, sizeof detail, "frame0 hold=%.2fs axis=char+Y body=%s hipsBone=%s watchdog=%.2fs",
                      static_cast<double>(a.durationS), BodyName(g_body), std::string(b.db.BoneName(hips)).c_str(), rag::kProbeWatchdogS);
        ProbeLogLocked("START", detail, nullptr);
        if (g_log != nullptr)
            g_log("[utpp_ragdoll] getup-probe NOTE pose = embedded set3 GetUp table frame 0 (the motion databases and stand_F/B/L/R have no "
                  "get-up clip); axis +Y is entity forward if the Root bone is aligned; root-frame visual yaw is dropped while held");
        return std::string("started: clip=") + g_probeClipName + " offset=" + std::to_string(a.offsetM) +
               " hold=" + std::to_string(a.durationS);
    }
    catch (...)
    {
        return "refused:exception";
    }
}

std::string RagdollGetUpProbeStop()
{
    try
    {
        std::lock_guard lock(g_m);
        if (!g_probe.Active())
            return "idle";
        ProbeEndLocked("stopped");
        return "stopped";
    }
    catch (...)
    {
        return "error";
    }
}

std::string RagdollGetUpProbeStatus()
{
    try
    {
        std::lock_guard lock(g_m);
        char buf[256];
        std::snprintf(buf, sizeof buf, "%s clip=%s offset=%.2f t=%.2f/%.2f frames=%u runs=%u lastEnd=%s",
                      g_probe.Holding() ? "hold" : (g_probe.HandingBack() ? "handback" : "idle"), g_probeClipName,
                      static_cast<double>(g_probe.OffsetM()), static_cast<double>(g_probe.ElapsedS()),
                      static_cast<double>(g_probe.DurationS()), g_probeFrames, g_probe.Runs(), g_probeLastEnd);
        return buf;
    }
    catch (...)
    {
        return "error";
    }
}

// ---- ragdoll native get-up: natives' bodies (see MotionDriver.hpp). Game thread (script VM), g_m like every other entry point.
namespace
{
// The get-up can run: setting on, v2 enabled and healthy, TPP, a bundle of the current body with valid get-up clips, an armed bone map.
bool GuAvailableLocked()
{
    return g_guSetting && g_enabled && !g_faulted && g_guLastTpp && g_bundle && g_bundle->body == g_body && g_bundle->getupOk && g_map.ok &&
           g_rig != 0;
}
} // namespace

void RagdollSetGetUp(bool aOn)
{
    try
    {
        std::lock_guard lock(g_m);
        g_guSetting = aOn;
        if (!aOn)
        {
            GetUpEndLocked("setting-off", true);
            rag::LastPose().Freeze(false);
            g_guPlanValid = false;
        }
    }
    catch (...)
    {
    }
}

bool RagdollGetUpAvailable(int64_t aNowTicks, int64_t aHz)
{
    try
    {
        std::lock_guard lock(g_m);
        uint32_t n = 0;
        return GuAvailableLocked() && !g_gu.Active() && rag::LastPose().Peek(aNowTicks, aHz, rag::kGuSnapMaxAgeS, n);
    }
    catch (...)
    {
        return false;
    }
}

int32_t RagdollGetUpPlan(const float aEntityPos[3], const float aEntityRot[4], int64_t aNowTicks, int64_t aHz, bool aHaveRef, float aRefSign)
{
    try
    {
        std::lock_guard lock(g_m);
        g_guLastAborted = 0; // cleared by the next Plan
        ++g_guPlans;
        const auto refuse = [&](int32_t aCode, const char* aWhy) {
            rag::LastPose().Freeze(false);
            g_guPlanValid = false;
            ++g_guRefusals;
            g_guLastPlanCode = aCode;
            char d[160];
            std::snprintf(d, sizeof d, "refused code=%d (%s) rms=%.3f front=%.2f lat=%.2f missing=%u snapN=%u", aCode, aWhy,
                          static_cast<double>(g_guPlan.rms), static_cast<double>(g_guFront), static_cast<double>(g_guLat), g_guMissing, g_guSnapN);
            GetUpLogLocked("plan", d);
            return aCode;
        };
        g_guPlan = rag::GuPlan{};
        g_guFront = g_guLat = 0.0f;
        g_guMissing = g_guSnapN = g_guSnapDesc = 0;
        g_guGroundSrc = '-';
        if (!GuAvailableLocked() || aEntityPos == nullptr || aEntityRot == nullptr)
            return refuse(rag::kGuPlanUnavailable, "unavailable");
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(aEntityPos[i]))
                return refuse(rag::kGuPlanUnavailable, "entity");
        if (g_gu.Active())
            return refuse(rag::kGuPlanBusy, "busy");
        rag::LastPose().Freeze(true);
        static rag::RdLastPose::Snapshot s_snap; // 13 KB, only touched under g_m
        if (!rag::LastPose().Read(s_snap, aNowTicks, aHz, rag::kGuSnapMaxAgeS))
            return refuse(rag::kGuPlanNoSnapshot, "no fresh snapshot");
        g_guSnapN = s_snap.n;
        g_guSnapDesc = s_snap.descCount;
        const Bundle& b = *g_bundle;
        const uint32_t n = static_cast<uint32_t>(b.pose.size());
        if (n > rag::kGuMaxBones || n != b.db.NumBones() || b.dbParents.size() != n || !b.guClips.ok)
            return refuse(rag::kGuPlanUnavailable, "bundle");
        std::vector<float> local(size_t(n) * 7, 0.0f);
        std::vector<uint8_t> have(n, 0);
        for (uint32_t i = 0; i < n; ++i)
            local[size_t(i) * 7 + 6] = 1.0f;
        for (uint32_t i = kFirstBone; i < n; ++i)
        {
            const int32_t gi = g_map.gameIndex[i];
            if (gi < 0)
                continue;
            for (uint32_t k = 0; k < s_snap.n; ++k)
                if (s_snap.dst[k] == static_cast<uint32_t>(gi))
                {
                    const float* r = s_snap.rec[k];
                    const float ql = r[4] * r[4] + r[5] * r[5] + r[6] * r[6] + r[7] * r[7];
                    if (!std::isfinite(r[0] + r[1] + r[2] + ql) || !(ql > 0.5f && ql < 2.0f))
                        break; // not a pose record: treated as missing
                    float* d = &local[size_t(i) * 7];
                    d[0] = r[0];
                    d[1] = r[1];
                    d[2] = r[2];
                    const float inv = 1.0f / std::sqrt(ql);
                    for (int j = 0; j < 4; ++j)
                        d[3 + j] = r[4 + j] * inv;
                    have[i] = 1;
                    break;
                }
        }
        // entity yaw: the heading of +Y under the entity rotation (the same convention as UTPPYawDegFromForward)
        rag::gu::Q q{aEntityRot[0], aEntityRot[1], aEntityRot[2], aEntityRot[3]};
        if (q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 0.0f)
            q.w = 1.0f;
        q = rag::gu::Normalize(q);
        const float up[3] = {0.0f, 1.0f, 0.0f};
        float fw[3];
        rag::gu::Rotate(q, up, fw);
        rag::GuPlanIn in;
        in.n = n;
        in.firstBone = kFirstBone;
        in.parents = b.dbParents.data();
        in.bones = &b.guClips.bones;
        in.clips = b.guClips.clip;
        in.snapLocal = local.data();
        in.have = have.data();
        std::memcpy(in.entityPos, aEntityPos, sizeof in.entityPos);
        in.entityYaw = std::atan2(-fw[0], fw[1]);
        in.haveRefSign = aHaveRef;
        in.refSign = aRefSign;
        if (g_guHaveCap && g_guAirDist >= 0.0f && g_guAirDist <= rag::kGuGroundMaxM)
            in.airGroundZ = g_guCap[2] - g_guAirDist; // the floor under the capsule from the last air feed
        rag::GuPlanOut out = rag::BuildGetUpPlan(in);
        g_guFront = out.frontZ;
        g_guLat = out.latZ;
        g_guMissing = out.missing;
        g_guPlan = out.plan;
        if (out.code < 0)
            return refuse(out.code, out.plan.why);
        g_guSnapLocal = std::move(out.snapLocal);
        g_guSnapRootWorld = std::move(out.snapRootWorld);
        g_guGroundSrc = out.groundSrc;
        std::memcpy(g_guPlanFrame.cap, aEntityPos, sizeof g_guPlanFrame.cap);
        g_guPlanFrame.yaw = in.entityYaw;
        g_guPlanValid = true;
        g_guPlanStep = g_steps;
        g_guLastPlanCode = out.code;
        const double ageMs = aHz > 0 && aNowTicks > s_snap.ticks ? 1000.0 * static_cast<double>(aNowTicks - s_snap.ticks) / static_cast<double>(aHz) : 0.0;
        char d[200];
        std::snprintf(d, sizeof d, "planned ref=%s%.0f snapN=%u snapDesc=%u gameBones=%zu snapAge=%.0fms entityYaw=%.1fdeg",
                      aHaveRef ? "pelvis" : "clip", static_cast<double>(aHaveRef ? aRefSign : b.guClips.clip[0].standSign), g_guSnapN,
                      g_guSnapDesc, g_gameHashes.size(), ageMs, static_cast<double>(in.entityYaw / integ::kDegToRad));
        GetUpLogLocked("plan", d);
        return out.code;
    }
    catch (...)
    {
        rag::LastPose().Freeze(false);
        return rag::kGuPlanUnavailable;
    }
}

void RagdollGetUpTarget(float aOut[4])
{
    aOut[0] = aOut[1] = aOut[2] = aOut[3] = 0.0f;
    try
    {
        std::lock_guard lock(g_m);
        if (!g_guPlanValid && !g_gu.Active())
            return;
        aOut[0] = g_guPlan.anchor[0];
        aOut[1] = g_guPlan.anchor[1];
        aOut[2] = g_guPlan.anchor[2];
        aOut[3] = g_guPlan.rms;
    }
    catch (...)
    {
    }
}

void RagdollGetUpForward(float aOut[2])
{
    aOut[0] = 0.0f;
    aOut[1] = 1.0f;
    try
    {
        std::lock_guard lock(g_m);
        if (!g_guPlanValid && !g_gu.Active())
            return;
        aOut[0] = g_guPlan.fwd[0];
        aOut[1] = g_guPlan.fwd[1];
    }
    catch (...)
    {
    }
}

bool RagdollGetUpBegin(float aGroundZ)
{
    try
    {
        std::lock_guard lock(g_m);
        if (!g_guPlanValid || g_gu.Active() || g_steps != g_guPlanStep || !GuAvailableLocked() || !g_bundle)
            return false; // no plan, a feed came in since the plan (state changed): the script uses the vanilla recovery
        const Bundle& b = *g_bundle;
        const uint32_t n = static_cast<uint32_t>(b.pose.size());
        if (n > integ::kMaxMMBones || n != b.db.NumBones() || g_guSnapLocal.size() != size_t(n) * 7 || g_guSnapRootWorld.size() != size_t(n) * 7)
            return false;
        rag::GuPlan plan = g_guPlan;
        if (std::isfinite(aGroundZ) && aGroundZ > -1.0e5f)
        {
            plan.anchor[2] = aGroundZ; // the script's ray at the anchor beats the DLL's own estimate
            g_guGroundSrc = 'r';
        }
        if (!g_gu.Start(plan, ProbeWallS()))
            return false;
        g_guPlan = plan;
        g_guBundle = g_bundle.get();
        g_guLift.Reset();
        g_guHand.Reset();
        g_guZGroundValid = false;
        g_guFootSteps = 0;
        g_guHeldValid = false;
        g_guGapHeld = false;
        g_guShownValid = g_guInHandOff = false;
        g_guLogDue = false;
        g_guPaused = false;
        g_guFrames = g_guGapFrames = 0;
        {
            // the entity frame the script teleports to in this very call (feet = anchor xy + the ray / plan ground z, rot = plan yaw): the
            // pose is composed against it until a feed sample matches (see GuTarget); the pre frame is the last feed sample
            rag::GuFrame target;
            std::memcpy(target.cap, plan.anchor, sizeof target.cap);
            target.yaw = plan.yaw;
            rag::GuFrame pre = g_guPlanFrame; // Begin refuses when a feed came in since the plan; the feed samples are the same entity transform (position + yaw)
            if (g_guHaveCap)
            {
                std::memcpy(pre.cap, g_guCap, sizeof pre.cap);
                pre.yaw = g_guYawDeg * integ::kDegToRad;
            }
            g_guTarget.Arm(target, pre);
        }
        g_guNoCapFeeds = g_guGateClosed = 0;
        g_guYieldSinceNs = g_guLastApplyNs = 0;
        g_guLogClock = 0.0f;
        g_guTailLeft = 0.0f;
        g_guGraceLeft = rag::kGuGroundGraceS;
        g_guHandOffEntry = false;
        g_guYawErrDeg = rag::gu::WrapPi(g_guYawDeg * integ::kDegToRad - plan.yaw) / integ::kDegToRad;
        g_guLastAborted = 0;
        ++g_guRuns;
        (void)rag::TakeGetUpAbortRequest();       // an old v1 request belongs to an old run
        rag::g_episode.HandToGetUp(ProbeWallS()); // the episode ends; the still-raised gate must not restart it
        rag::g_getUpOwns.store(true, std::memory_order_release);
        // pre-check (diagnostics): the deficit the blend will produce, sampled at 9 steps from the snapshot to the clip at the blend end, before and
        // after the limb correction (what is left for the whole-body lift); the live correction itself runs every frame in GetUpApplyLocked
        char pred[176] = "";
        if (const traversal::TravMontage* m = b.getup.Find(rag::kGuClipNames[plan.clip]))
        {
            const rag::GuShape shape{n, kFirstBone, b.dbParents.data()};
            rag::GuAnchor anchor;
            std::memcpy(anchor.pos, plan.anchor, sizeof anchor.pos);
            anchor.yaw = plan.yaw;
            const float zG = anchor.pos[2] - g_guPlanFrame.cap[2];
            float maxRaw = 0.0f, maxRes = 0.0f, maxFinger = 0.0f, maxFingerRes = 0.0f, wRaw = 0.0f, wRes = 0.0f;
            int jRaw = -1, jRes = -1;
            const float blendS = g_gu.BlendInS();
            for (int k = 0; k <= 8; ++k)
            {
                const float t = blendS * static_cast<float>(k) / 8.0f; // time inside the blend; the clip runs at kGuBlendClipRate meanwhile
                rag::SampleGetUpPose(*m, b.getup.poseBones.size(), b.guClips.poseMap, static_cast<double>(t * rag::kGuBlendClipRate), g_guClip.data());
                rag::ComposeBlend(shape, g_guClip.data(), g_guSnapLocal.data(), g_guSnapRootWorld.data(), anchor, g_guPlanFrame,
                                  rag::gu::Smooth(t / blendS), g_guOut.data());
                const float wk = rag::gu::Smooth(t / blendS);
                int j = -1;
                const float raw = rag::FloorDeficit(shape, g_guOut.data(), b.guClips.bones, zG, &j);
                const float fingers = rag::FingerDeficit(shape, g_guOut.data(), b.guClips.bones, zG);
                if (raw > maxRaw)
                    maxRaw = raw, jRaw = j, wRaw = wk;
                maxFinger = std::fmax(maxFinger, fingers);
                if (raw > 0.002f || fingers > 0.002f)
                {
                    (void)rag::FloorCorrect(shape, g_guOut.data(), b.guClips.bones, zG);
                    const float res = rag::FloorDeficit(shape, g_guOut.data(), b.guClips.bones, zG, &j);
                    if (res > maxRes)
                        maxRes = res, jRes = j, wRes = wk;
                    maxFingerRes = std::fmax(maxFingerRes, rag::FingerDeficit(shape, g_guOut.data(), b.guClips.bones, zG));
                }
            }
            std::snprintf(pred, sizeof pred, "blend=%.2fs predicted deficit raw=%.3f afterLimbs=%.3f fingers=%.3f/%.3f rawAt=%s@w%.2f limbsAt=%s@w%.2f",
                          static_cast<double>(blendS), static_cast<double>(maxRaw), static_cast<double>(maxRes), static_cast<double>(maxFinger),
                          static_cast<double>(maxFingerRes), jRaw >= 0 ? rag::kGuClampNames[jRaw] : "-", static_cast<double>(wRaw),
                          jRes >= 0 ? rag::kGuClampNames[jRes] : "-", static_cast<double>(wRes));
        }
        GetUpLogLocked("Begin", pred);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

namespace
{
std::atomic<void (*)()> g_guGapPublisher{nullptr};
}

void SetGetUpGapPublisher(void (*aFn)())
{
    g_guGapPublisher.store(aFn, std::memory_order_release);
}

bool RagdollGetUpBeginPublish(float aGroundZ)
{
    const bool ok = RagdollGetUpBegin(aGroundZ); // g_m is released here: the v1 host takes g_mutex, then the v2 mutex (the apply hook)
    if (ok)
        if (void (*publish)() = g_guGapPublisher.load(std::memory_order_acquire))
            publish();
    return ok;
}

void RagdollGetUpCancel()
{
    try
    {
        std::lock_guard lock(g_m);
        if (g_gu.Active())
            return; // a run owns the store; it unfreezes at its end
        rag::LastPose().Freeze(false);
        g_guPlanValid = false;
    }
    catch (...)
    {
    }
}

int32_t RagdollGetUpState()
{
    try
    {
        std::lock_guard lock(g_m);
        if (g_gu.Active())
            return static_cast<int32_t>(g_gu.Phase());
        return g_guLastAborted != 0 ? -1 : 0;
    }
    catch (...)
    {
        return 0;
    }
}

std::string RagdollGetUpStatus()
{
    try
    {
        std::lock_guard lock(g_m);
        const char* avail = !g_bundle ? "no-bundle" : (g_bundle->getupOk ? "ok" : g_bundle->getupErr.c_str());
        const rag::GuPlan& p = g_guPlan;
        char buf[900];
        std::snprintf(buf, sizeof buf,
                      "getup setting=%d avail=%s phase=%s clip=%s t=%.2f/%.2f w=%.2f rms=%.3f yawErr=%.1fdeg lift=%.3f liftFrames=%u maxDeficit=%.3f limbFrames=%u maxRaw=%.3f "
                      "frames=%u missing=%u ground=%c front=%.2f lat=%.2f snapN=%u snapDesc=%u runs=%llu aborts=%llu plans=%llu refused=%llu "
                      "lastPlan=%s lastEnd=%s owns=%d grace=%.2f gap=%u tgt=%u deficitAt=%s@w%.2f rawAt=%s@w%.2f",
                      g_guSetting ? 1 : 0, avail, GetUpPhaseName(),
                      p.clip >= 0 && p.clip < rag::kGuClipCount ? rag::kGuClipNames[p.clip] : "-", static_cast<double>(g_gu.ClipTime()),
                      static_cast<double>(p.handOffS), static_cast<double>(g_gu.BlendW()), static_cast<double>(p.rms),
                      static_cast<double>(g_guYawErrDeg), static_cast<double>(g_guLift.lift), g_guLift.frames,
                      static_cast<double>(g_guLift.maxDeficit), g_guLift.limbFrames, static_cast<double>(g_guLift.maxRaw), g_guFrames, g_guMissing,
                      g_guGroundSrc, static_cast<double>(g_guFront),
                      static_cast<double>(g_guLat), g_guSnapN, g_guSnapDesc, static_cast<unsigned long long>(g_guRuns),
                      static_cast<unsigned long long>(g_guAborts), static_cast<unsigned long long>(g_guPlans),
                      static_cast<unsigned long long>(g_guRefusals), rag::GuPlanCodeName(g_guLastPlanCode), g_guLastEnd,
                      rag::GetUpOwns() ? 1 : 0, static_cast<double>(g_guGraceLeft), g_guGapFrames, g_guTarget.targetFrames,
                      g_guLift.maxJoint >= 0 ? rag::kGuClampNames[g_guLift.maxJoint] : "-", static_cast<double>(g_guLift.maxJointW),
                      g_guLift.maxRawJoint >= 0 ? rag::kGuClampNames[g_guLift.maxRawJoint] : "-", static_cast<double>(g_guLift.maxRawW));
        return buf;
    }
    catch (...)
    {
        return "error";
    }
}

void GetUpAbort(const char* aWhy)
{
    try
    {
        std::lock_guard lock(g_m);
        GetUpEndLocked(aWhy ? aWhy : "abort", true);
        if (!g_gu.Active())
        {
            rag::LastPose().Freeze(false);
            g_guPlanValid = false;
        }
    }
    catch (...)
    {
    }
}

bool RagdollRecoverActive()
{
    try
    {
        std::lock_guard lock(g_m);
        return g_guGraceLeft > 0.0f || rag::RagdollRecoverActive();
    }
    catch (...)
    {
        return false;
    }
}

GripProbe ProbeGrip()
{
    std::lock_guard lock(g_m);
    return g_gripProbe;
}

TraversalProbe ProbeTraversal()
{
    std::lock_guard lock(g_m);
    TraversalProbe p = g_travProbe;
    if (g_bundle && g_bundle->travOk)
    {
        p.playable = static_cast<uint32_t>(g_bundle->travPlay.montages.size());
        p.playableFromPoses = g_bundle->travFromPoses;
    }
    return p;
}

bool OwnsTraversal() noexcept
{
    return g_ownsTrav.load(std::memory_order_acquire);
}

bool TravActive() noexcept
{
    return g_travActive.load(std::memory_order_acquire);
}

bool VehicleVaultWeapon() noexcept
{
    return g_vehVaultEvt.load(std::memory_order_acquire) &&
           ov::RangedPose(static_cast<ov::PoseOverlay>(g_poseMirror.load(std::memory_order_acquire)));
}

#if defined(UTPP_MOTION_DRIVER_TEST_FAULTS) || defined(UTPP_MOTION_DRIVER_TEST_TRAV)
void TestForceTraversal(const std::string& aMontage, bool aGate)
{
    std::lock_guard lock(g_m);
    g_testForceMontage = aMontage;
    g_testTravGate = aGate;
}

void TestSetRecoilClock(double aSeconds)
{
    std::lock_guard lock(g_m);
    g_testRecoilClock = aSeconds;
}

void TestAdvanceRecoilClock(double aSeconds)
{
    std::lock_guard lock(g_m);
    if (g_testRecoilClock >= 0.0 && aSeconds > 0.0) g_testRecoilClock += aSeconds;
}

void TestSetSteadyClockNs(int64_t aNs)
{
    g_testSteadyNs.store(aNs, std::memory_order_release);
}

void TestAdvanceSteadyClockNs(int64_t aNs)
{
    if (aNs > 0 && g_testSteadyNs.load(std::memory_order_acquire) >= 0)
        g_testSteadyNs.fetch_add(aNs, std::memory_order_acq_rel);
}

#endif

Fallback Reason()
{
    std::lock_guard lock(g_m);
    return ResolveLocked();
}

const char* LastSkipReason()
{
    std::lock_guard lock(g_m);
    return g_lastSkip;
}

namespace
{
// P9: summed weight of the Aiming-state pose keys of the overlay machine (the hands / arms crossfade; AimingWeight is the AO alpha).
float AimPoseWeightLocked()
{
    float w = 0.0f;
    if (!g_bundle)
        return w;
    const ov::OverlayMachine& m = g_bundle->eval.Machine();
    for (int i = 0; i < m.poses.count; ++i)
    {
        ov::PoseOverlay ko;
        ov::AimState ka;
        bool kc, km;
        ov::DecodePoseKey(m.poses.key[i], ko, ka, kc, km);
        if (ka == ov::AimState::Aiming)
            w += m.poses.w[i];
    }
    return w;
}

// Feed budget fields at the end of the status line: wall-time p95 of the window, trips this session, feed time left in the pause,
// length of the current / next pause, thread time in use (1) or wall time alone (0).
std::string BudgetStatusSuffixLocked()
{
    char buf[200];
    std::snprintf(buf, sizeof buf, " wallP95us=%.1f budgetTrips=%llu budgetPauseS=%.2f budgetCoolS=%.0f threadTime=%d",
                  static_cast<double>(WallP95Locked()), static_cast<unsigned long long>(g_budgetTrips),
                  static_cast<double>(g_budgetTripped ? g_budgetPauseLeft : 0.0f),
                  static_cast<double>(g_budgetTripped ? g_budgetPauseS : g_budgetNextPauseS),
                  g_cyclesPerUs.load(std::memory_order_relaxed) > 0.0f ? 1 : 0);
    return buf;
}

std::string StatusLocked()
{
    std::lock_guard lock(g_m);
    const Fallback f = ResolveLocked();
    std::string why = adapter::FallbackName(f);
    if (f == Fallback::RigMismatch && g_rig == 0)
        why += "(not armed)";
    const char* loaded = g_bundle ? BodyName(g_bundle->body) : "none";
    const bool dbOk = g_bundle && g_bundle->body == g_body && g_bundle->res.dbParsed;
    char buf[1600]; // UTPP-MOTION SEAM trav + air: was 768 (trav= field, air: segment); motionfix: was 1280
    std::snprintf(buf, sizeof(buf),
                  " motion enabled=%d body=%s db=%s(%s)%s err='%s' reason=%s skip=%s pick=clip:%d@%.2f/db%d/cost%.3f rate=%.4f"
                  " frames=%llu applied=%llu steps=%llu feedP95us=%.1f budget=%.0f trav=%d overlay=%d/%d rel=%d fault=%s tail=%llu layoutSkips=%llu handin=%llu handover=%llu trav=%d/%llu/%llu/%llu/%llu sha=%s"
                  " air:owns=motion feed=%d mode=%d ttl=%.2f dist=%.2f land=%d%d rtzRes=%.3f/%.3f",
                  g_enabled ? 1 : 0, BodyName(g_body), dbOk ? "loaded" : "no", loaded, g_loading ? " loading" : "",
                  g_bundle ? g_bundle->err.c_str() : "", why.c_str(), g_lastSkip, g_last.clip,
                  static_cast<double>(g_last.clipTime), g_last.database,
                  static_cast<double>(g_last.cost < 1e30f ? g_last.cost : -1.0f), static_cast<double>(g_last.playRate),
                  static_cast<unsigned long long>(g_frames),
                  static_cast<unsigned long long>(g_applied), static_cast<unsigned long long>(g_steps),
                  static_cast<double>(P95Locked()), static_cast<double>(adapter::Budget::kFeedP95Us),
                  g_bundle && g_bundle->travOk ? 1 : 0, g_pose, g_aim, g_release ? 1 : 0,
                  g_faultWhy ? g_faultWhy : "none", static_cast<unsigned long long>(g_tailFrames),
                  static_cast<unsigned long long>(g_skipLayout), static_cast<unsigned long long>(g_handinFrames),
                  static_cast<unsigned long long>(g_handoverFrames),
                  g_travProbe.lastResult, static_cast<unsigned long long>(g_travProbe.accepted), // UTPP-MOTION SEAM trav
                  static_cast<unsigned long long>(g_travProbe.yields), static_cast<unsigned long long>(g_travProbe.frames),
                  static_cast<unsigned long long>(g_skipTrav),
                  g_bundle && !g_bundle->sha.empty() ? g_bundle->sha.c_str() : "none",
                  g_airDiag.feedValid ? 1 : 0, g_airDiag.movementMode, static_cast<double>(g_airDiag.timeToLand),
                  static_cast<double>(g_airDiag.distanceUsed), g_airDiag.justLandedLight ? 1 : 0, g_airDiag.justLandedHeavy ? 1 : 0,
                  static_cast<double>(g_airDiag.residual), static_cast<double>(g_airDiag.maxResidual));
#if UTPP_MOTION_DRIVER_FIX_MUTATION != 5
    // UTPP-MOTION SEAM motionfix (item 4): bundle state, rig identity, bone map, load / log counts
    const size_t used = std::strlen(buf);
    std::snprintf(buf + used, sizeof(buf) - used,
                  " bundle=%s/%s/%s rigId=0x%016llx(%s) rig=0x%llx gameBones=%zu map=%d:%u/%u/miss%u/par%u topo=%u"
                  " loads=%llu loadLines=%llu reasonLines=%llu",
                  g_bundle ? BodyName(g_bundle->body) : "none",
                  g_bundle ? (g_bundle->res.matcherInit ? "ready" : "failed") : "-",
                  g_loading ? BodyName(g_loadingBody) : "idle",
                  static_cast<unsigned long long>(g_bundle ? g_bundle->res.dbRigIdentity : 0ull),
                  g_bundle && g_bundle->res.dbRigIdentity == adapter::ExpectedRigIdentity(g_bundle->body) ? "body" : "other",
                  static_cast<unsigned long long>(g_rig), g_gameHashes.size(), g_map.ok ? 1 : 0, g_map.mapped, g_map.numDbBones,
                  g_map.missing, g_map.parentMismatch, g_topology, static_cast<unsigned long long>(g_loads),
                  static_cast<unsigned long long>(g_loadLines), static_cast<unsigned long long>(g_reasonLines));
#endif
    // UTPP-MOTION SEAM motionfix item 2: aim target, aim offset, upper-body handback
    const size_t used2 = std::strlen(buf);
    std::snprintf(buf + used2, sizeof(buf) - used2, " aim=tgt%d%d aiming=%d ao=%llu aoW=%.2f aoU=%.3f aoYaw=%.1f gameUpper=%d handback=%llu",
                  g_aimTgtOn ? 1 : 0, g_aimTgtValid ? 1 : 0, g_aimingNow ? 1 : 0, static_cast<unsigned long long>(g_aoFrames),
                  static_cast<double>(g_aoW), static_cast<double>(g_aoU), static_cast<double>(g_aoYaw), g_aimGameUpper ? 1 : 0,
                  static_cast<unsigned long long>(g_aimHandback));
    char grip[360];
    std::snprintf(grip, sizeof grip, " grip=%d supplied=%d applied=%d clamped=%d error=%.6f accepted=%llu rejected=%llu target=%.6f,%.6f,%.6f reach=%.6f/%.6f source=%u authored=%llu shift=%.6f",
                  g_gripEnabled ? 1 : 0, g_gripProbe.supplied ? 1 : 0, g_gripProbe.applied ? 1 : 0,
                  g_gripProbe.clamped ? 1 : 0, static_cast<double>(g_gripProbe.error),
                  static_cast<unsigned long long>(g_gripProbe.accepted), static_cast<unsigned long long>(g_gripProbe.rejected),
                  static_cast<double>(g_gripProbe.relativeTarget[0]), static_cast<double>(g_gripProbe.relativeTarget[1]),
                  static_cast<double>(g_gripProbe.relativeTarget[2]), static_cast<double>(g_gripProbe.requestedReach),
                  static_cast<double>(g_gripProbe.usedReach), g_gripProbe.source,
                  static_cast<unsigned long long>(g_gripProbe.authoredFrames), static_cast<double>(g_gripProbe.weaponShift));
    return std::string(buf) + grip + " action=" + std::to_string(g_weaponAction) +
           " capsuleAccel=" + std::to_string(g_bundle ? g_bundle->map.capsuleAccelModel : 0.f) +
           " cancelSupport=" + std::to_string(g_bundle ? g_bundle->eval.CancelledStartSupport() : -1) +
           " shotTurn=" + std::to_string(g_stationaryShotTurn.Active()) +
           " shotGoal=" + std::to_string(g_stationaryShotTurn.targetYawDeg) +
           " hipFirePose=" + std::to_string(g_hipFirePoseNow) +
           " resolvedAim=" + std::to_string(g_resolvedAimPose) +
           " reloadUpper=" + std::to_string(g_reloadUpperWeight) +
           " reloadAuthored=" + std::to_string(g_authoredReload) +
           " shootTransfer=" + std::to_string(g_shootTransferEnabled) +
           " shootPending=" + std::to_string(g_shootEpisode.pending) +
           " shootAdmitted=" + std::to_string(g_shootEpisode.admitted) +
           " shootApplied=" + std::to_string(g_shootApplied) +
           " shootRoute=" + std::to_string(g_shootRouteNow) +
           " shootRouteLive=" + std::to_string(g_shootRouteLive) +
           " aimPoseW=" + std::to_string(AimPoseWeightLocked()) +
           " recoilSupported=" + std::to_string(g_recoilSupported) +
           " recoilProfile=" + std::to_string(g_recoilProfile) +
           " recoilSerial=" + std::to_string(g_recoilSerial) + // shot pulses of this weapon: one per round when OnEnter runs per round
           " recoilEff=" + std::to_string(g_recoilEffT) + "/" + std::to_string(g_recoilEffR) + " recoilExitW=" + std::to_string(g_recoilExitW) +
           " recoilProfileReady=" + std::to_string(g_bundle && recoil_profile::Valid(g_recoilProfile) &&
                                                   g_bundle->recoilOk[static_cast<size_t>(g_recoilProfile)]) +
           " nueRecoilReady=" + std::to_string(g_bundle && g_bundle->recoilOk[static_cast<size_t>(recoil_profile::kNue)]) +
           " nueRecoilOwns=" + std::to_string(g_recoilOwns) +
           " nueRecoilActive=" + std::to_string(g_authoredRecoil.Active()) +
           " nueRecoilAdmitted=" + std::to_string(g_recoilAdmitted) +
           " nueRecoilPending=" + std::to_string(g_recoilPending) +
           " shootScale=" + std::to_string(g_shootReachableScale) +
           " shootEvaluations=" + std::to_string(g_shootAppliedEvaluations) +
           " airLegs=" + std::to_string(g_airLegsFade.w) +
           " joArms=" + std::to_string(g_jumpOutArmsFade.w) +
           " headLead=" + std::to_string(g_headLeadOn ? 1 : 0) +
           " lead=" + std::to_string(g_hl.Lead()) +
           " corr=" + std::to_string(g_hl.Corr()) +
           " hlW=" + std::to_string(g_hl.Weight()) +
           " slide=" + std::to_string(g_slideSetting ? 1 : 0) +
           " slideBundle=" + std::to_string(g_bundle && g_bundle->slide.declared ? 1 : 0) +
           " slideFrames=" + std::to_string(g_slideFrames) +
           " slideDeRollW=" + std::to_string(g_slideDeRoll.weight) +
           " slideRifleLatch=" + std::to_string(g_rifleSlideLatch.latched ? 1 : 0) +
           " slideCarry=" + std::to_string(g_slideCarryFrames) + " slideCarryScale=" + std::to_string(g_slideCarryScale) +
           " magTapW=" + std::to_string(g_magTap.weight) +
           " magTap=" + std::to_string(g_magTapFed) + "/" + std::to_string(g_magTapStarted) + "/" + std::to_string(g_magTapDropped) +
           "/" + std::to_string(g_magTapFrames) +
           " magTapMoved=" + std::to_string(g_magTapMoved) + " magTapMovedMax=" + std::to_string(g_magTapMovedMax) +
           " magTapErr=" + std::to_string(g_magTapErr) +
           " magTapTravel=" + std::to_string(g_magTapTravel) + " magTapUnowned=" + std::to_string(g_magTapUnowned) +
           " magTapOn=" + std::to_string(g_magTapEnabled ? 1 : 0) + " magTapOff=" + std::to_string(g_magTapOff) +
           " magTapDeferred=" + std::to_string(g_magTapDeferredReqs) + " magTapWait=" + std::to_string(g_magTapLastWait) +
           " slideDeRollDeg=" + std::to_string(g_derollAppliedDeg) +
           " slideDePitchDeg=" + std::to_string(g_depitchAppliedDeg) +
           " slideRollHold=" + std::to_string(g_reloadRollHold.hold) +
           " walkSpeed=" + std::to_string(g_walkSpeed.load(std::memory_order_acquire)) +
           " minAnalogWalk=" + std::to_string(g_minAnalogWalk.load(std::memory_order_acquire)) +
           " selGuards=" + std::to_string(g_selectionGuards.load(std::memory_order_acquire) ? 1 : 0) +
           " tier2Slow=" + std::to_string(g_tier2Slow.load(std::memory_order_acquire) ? 1 : 0) +
           " shooterMode=" + std::to_string(g_shooter_modeSetting ? 1 : 0) +
           " shooterModeLive=" + std::to_string(g_shooter_modeLive.load(std::memory_order_acquire) ? 1 : 0) +
           " shooterFrames=" + std::to_string(g_shooterFrames) +
           " h2Guard=" + std::to_string(g_h2GuardSetting ? 1 : 0) +
           " h2=" + std::to_string(g_bundle && g_bundle->map.h2Active ? 1 : 0) +
           " h2Frames=" + std::to_string(g_h2Frames) +
           " btSwitch=" + std::to_string(g_lastBtSwitch ? 1 : 0) +
           " hipLease=" + std::to_string(g_hipLease ? 1 : 0) +
           " orientWarp=" + std::to_string(g_orientWarpOn ? 1 : 0) +
           " owW=" + std::to_string(g_ow.Weight()) +
           " owRoot=" + std::to_string(g_ow.RootDeg()) +
           " owTorso=" + std::to_string(g_ow.TorsoDeg()) +
           " lean=" + std::to_string(g_leanSetting ? 1 : 0) + "/" + std::to_string(g_bundle ? (g_bundle->leanOk ? 1 : 0) : -1) + "/" +
           std::to_string(g_bundle && g_bundle->leanOk ? g_bundle->lean.Weight() * g_leanParams.scale : 0.0f) + "/" +
           std::to_string(g_bundle && g_bundle->leanOk ? g_bundle->lean.Tel().lat : 0.0f) + "/" +
           std::to_string(g_bundle && g_bundle->leanOk ? g_bundle->lean.Tel().x : 0.0f) + "/" +
           std::to_string(g_bundle && g_bundle->leanOk ? g_bundle->lean.Tel().disable : 0.0f) +
           " leanHead=" + std::to_string(g_leanHeadSetting ? 1 : 0) + " leanScale=" + std::to_string(g_leanParams.scale) +
           " leanClips=" + std::to_string(g_bundle ? g_bundle->leanMapped : 0) + "/" + std::to_string(g_bundle ? g_bundle->leanMismatch : 0) +
           DualStatusSuffix() + BudgetStatusSuffixLocked();
}
} // namespace

std::string Status()
{
    try
    {
        return StatusLocked();
    }
    catch (...)
    {
        return "motion status-error"; // short string (no heap); natives must not throw
    }
}

bool LoadedBody(int32_t& aBody)
{
    std::lock_guard lock(g_m);
    if (!g_bundle)
        return false;
    aBody = g_bundle->body == Body::Man ? kBodyMan : kBodyWoman;
    return true;
}

uint64_t Loads()
{
    std::lock_guard lock(g_m);
    return g_loads;
}

namespace
{
// Caller owns g_joinM for the entire move-and-join operation.
void WaitLoaderSerial()
{
    std::thread t;
    {
        std::lock_guard lock(g_m);
        t = std::move(g_worker.t);
    }
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    if (t.joinable())
    {
        std::unique_lock hold(g_testWaitM);
        if (g_testWaitPause && !g_testWaitMoved)
        {
            g_testWaitMoved = true;
            g_testWaitCv.notify_all();
            (void)g_testWaitCv.wait_for(hold, std::chrono::seconds(10), [] { return g_testWaitResume; });
        }
    }
#endif
    if (t.joinable())
        t.join();
}
} // namespace

void WaitLoader()
{
    std::lock_guard joinLock(g_joinM);
    WaitLoaderSerial();
}
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
void TestWorkerPause(bool aEnable)
{
    std::lock_guard hold(g_testWaitM);
    g_testWorkerPause = aEnable;
    g_testWorkerEntered = false;
    g_testWorkerResume = !aEnable;
    g_testWorkerTimedOut = false;
    g_testShutdownEntries = 0;
    g_testWaitCv.notify_all();
}
bool TestWorkerBlocked()
{
    std::lock_guard hold(g_testWaitM);
    return g_testWorkerEntered && !g_testWorkerResume && !g_testWorkerTimedOut;
}
uint32_t TestShutdownEntries()
{
    std::lock_guard hold(g_testWaitM);
    return g_testShutdownEntries;
}
void TestWorkerResume()
{
    std::lock_guard hold(g_testWaitM);
    g_testWorkerResume = true;
    g_testWaitCv.notify_all();
}
void TestWaitLoaderPauseAfterMove(bool aEnable)
{
    std::lock_guard hold(g_testWaitM);
    g_testWaitPause = aEnable;
    g_testWaitMoved = false;
    g_testWaitResume = !aEnable;
}
bool TestWaitLoaderMoved()
{
    std::lock_guard hold(g_testWaitM);
    return g_testWaitMoved;
}
void TestWaitLoaderResume()
{
    std::lock_guard hold(g_testWaitM);
    g_testWaitResume = true;
    g_testWaitCv.notify_all();
}
#endif

const integ::PoseSnapshot* LastMotionSnapshot()
{
    std::lock_guard lock(g_m);
    return g_haveLastSnap ? &g_lastSnap : nullptr;
}

void Shutdown()
{
    integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
#ifdef UTPP_MOTION_DRIVER_TEST_FAULTS
    {
        std::lock_guard hold(g_testWaitM);
        ++g_testShutdownEntries;
        g_testWaitCv.notify_all();
    }
#endif
    std::lock_guard joinLock(g_joinM);
    {
        std::lock_guard lock(g_m);
        ResetAuthoredRecoil();
        g_stopping = true;
        g_authoredReload = false;
        integ::g_gamePoseCaptureWanted.store(false, std::memory_order_relaxed);
        g_enabled = false;
        g_weaponAction = 0;
        g_hipFirePoseHold.Reset(); g_jumpOutExitHold.Reset(); g_stationaryShotTurn.Reset();
        g_reloadUpperWeight = 0.0f; g_reloadUpperEnv.Reset(); g_reloadRollHold.Reset(); g_slideDeRoll.Reset(); g_rifleSlideLatch.Reset(); g_slideCarryAim.Reset(); ResetMagTap(); g_reloadContactFade.Reset(); g_reloadContactEpisode.Reset(); g_reloadContactsEligible = false;
        g_aimUpperTail = false;
        g_weaponUpperFrame = g_weaponUpperTail = false;
        g_shootRouteNow = false;
        g_shootRouteHeld = g_shootRouteInShot = g_shootRouteLive = false;
    }
    WaitLoaderSerial();
    std::unique_ptr<Bundle> old;
    {
        std::lock_guard lock(g_m);
        g_enabled = false;
        g_pending = false;
        TravResetLocked(true); // UTPP-MOTION SEAM trav
        ProbeEndLocked("shutdown");
        GetUpEndLocked("shutdown", true);
        old = std::move(g_bundle);
        RefreshTravActiveLocked(); // disabled, no bundle: false
        g_map = integ::BoneMapResult{};
    }
    old.reset(); // keep admission closed while the retired bundle is destroyed outside g_m
    {
        std::lock_guard lock(g_m);
        g_enabled = false; // Shutdown wins over a concurrent enable during its drain
        g_stopping = false;
    }
}
} // namespace utpp::motiondriver

// Phase 4b motion matching driver (game thread). See MotionHost.hpp.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <bcrypt.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <atomic>
#if defined(RED4EXT_HEADER_ONLY) // plugin build only (target UTPP links the SDK): the native sim clock; the standalone test builds have none
#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/EngineTime.hpp>
#include <RED4ext/Scripting/Natives/ScriptGameInstance.hpp>
#endif

#include "CommonUtil.hpp"
#include "BaseDatabase.hpp"
#include "MotionHost.hpp"
#include "MutationSelect.hpp"
#include "State.hpp"
#include "mm/AirState.hpp" // AIR HOLD stage
#include "mm/AirTable.hpp" // AIR CLIPS stage
#include "mm/BodyTurn.hpp"
#include "mm/RefaceStart.hpp"
#include "mm/CapsuleSync.hpp"
#include "mm/CapsuleBraking.hpp"
#include "mm/CapsuleAcceleration.hpp"
#include "mm/ChestAlign.hpp"
#include "mm/FeedDump.hpp"
#include "mm/FootLocker.hpp"
#include "mm/GameIntegration.hpp"
#include "mm/MeleeFullBody.hpp" // melee full body (2026-11-09)
#include "mm/VaultTorso.hpp" // vault torso limiter (2026-11-10)
#include "mm/BaseMatcher.hpp"
#include "mm/OffsetRoot.hpp"
#include "mm/PlaybackRate.hpp"
#include "mm/PoseTrace.hpp"
#include "mm/SpeedBands.hpp"
#include "mm/SpeedCalibration.hpp"
#include "mm/StanceGuard.hpp"
#include "mm/PoseStepLimit.hpp"
#include "mm/Steering.hpp"
#include "mm/Strafe.hpp"
#include "ragdoll/RagdollEpisode.hpp" // ragdoll episode: game-owned feed
#include "mm/VehicleTransition.hpp" // vehicle batch W1 P2: mount pop / exit hand-in (CAR-5 / MOTO-3, MOTO-6 / CAR-1 step 5)

namespace utpp::motionhost
{
namespace
{
using namespace utpp::mm;
using namespace utpp::mm::integ;

std::mutex g_mutex;
Database g_db;
BaseMatcher g_matcher;
MatcherConfig g_cfg;
bool g_dbLoaded = false;
std::string g_dbReason = "not loaded";
std::string g_dbPath;
std::string g_tagFilter;
uint64_t g_tagMask = ~0ull;
// Per-locomotion-state tag sets (computed at database load; 0 = no clips for that state -> state gated out).
// Used when no manual filter is set (UTPPSetMMTagFilter("") ). Stand and sprint share the "Stand_" clips.
uint64_t g_stateTagMask[4] = {0, 0, 0, 0};
uint64_t g_appliedTagMask = ~0ull;

std::vector<uint64_t> g_dbHashes;
std::vector<int32_t> g_dbParents;

BoneMapResult g_map{};
uint64_t g_rig = 0;
uint32_t g_rigBones = 0;
std::string g_mapReason = "not armed";

bool g_enabled = false;
uint32_t g_allowedMask = kDefaultAllowedLocoMask;
GateFader g_weight;       // 0.3.2: asymmetric fade (hard fade-out, reopen hold) - see GameIntegration.hpp
FadeTimes g_fade{kDefaultBlendTime, kDefaultHardOutTime, kDefaultReopenHold};
uint32_t g_psmBlock = 0;  // last PsmBlock bits from the feed
uint64_t g_gateCloses = 0;
uint64_t g_psmBlockFeeds = 0;
uint32_t g_gatePrev = 0xFFFFFFFFu;
uint32_t g_gate = kGateMMDisabled | kGateDbNotLoaded | kGateNotMapped;
float g_target = 0.0f;

// feed diagnostics
uint64_t g_feedCalls = 0;
FeedInput g_lastFeed{};
float g_lastCharVel[2] = {0, 0};
float g_lastYawRate = 0.0f;
bool g_havePrevYaw = false;
float g_prevYaw = 0.0f;
int64_t g_lastFeedTicks = 0;

// diagnostic trace
bool g_trace = false;

// speed mapping (defaults: game run measured 3.85 m/s in 2.31; db = motion subset loops after retarget scaling)
// Per locomotion state: up to 3 (game m/s -> database m/s) points, piecewise linear through (0,0), extrapolated
// with the last segment's ratio (db/game of the last point). Defaults measured in 2.31 (TPP): run 3.85,
// sprint 8.2, crouch 2.7 m/s; database loops (motion after retarget): walk 2.19, run 5.46, sprint 7.65, crouch 2.46.
// (SpeedPoints / MapSpeedPoints / LoopRate: mm/PlaybackRate.hpp, shared with the offline tests)
SpeedPoints g_speedMap[4] = {
    {0, {0, 0, 0}, {0, 0, 0}},                 // other (gated out)
    {3, {1.6f, 3.85f, 8.2f}, {2.19f, 5.46f, 7.65f}}, // stand
    {1, {2.7f, 0, 0}, {2.46f, 0, 0}},          // crouch
    {3, {1.6f, 3.85f, 8.2f}, {2.19f, 5.46f, 7.65f}}, // sprint (same clip set as stand)
};
float g_rateMin = 0.6f, g_rateMax = 1.4f;

// Input-driven trajectory (motion-like): desired direction from MoveX/MoveY relative to the camera, so the matcher
// knows the intended direction before the capsule/the body turns. Target speeds per state (game m/s).
bool g_inputDriven = true;
// Locomotion rewrites MoveX/MoveY relative to the BODY (measured in game: after a 180 turn mv=(0,1) while moving
// away from the camera), so by default the input is read in the character frame; camera-relative is optional.
bool g_inputCameraRelative = false;
float g_stateTargetSpeed[4] = {0.0f, 3.85f, 2.7f, 8.2f}; // other, stand(run), crouch, sprint
float g_transitionCost = 0.15f;                          // bias towards continuing the playing clip
float g_lastRate = 1.0f;

// 0.3.4: ground-locomotion clip sets exclude landing / from-traversal databases (PSD_*_Lands_*, *_FromTraversal). MM is
// gated off during jumps and falls, so these clips were only wrong candidates (offline: Jump_B_Land_Stand_Heavy chosen
// as a slow walk). Kill switch UTPPSetMMExcludeLands (default on).
bool g_excludeLands = true;

// ---- loco C1a (2026-10-13; default on, off = the loco P0 build): crouch idle curation ----
// The crouch clip set leaves out the tags of an EXACT-name list (UTPPSetMMCrouchExcludeTags; the default list is empty: the
// package databases carry no tag that has to leave the crouch set). Exact names only: a substring "Crouch_Idle" would also take
// PSD_Dense_Crouch_Idles (motion crouch idle + Stand_to_Crouch). Fallback: the full crouch set is kept when no looping,
// searchable idle-class clip would remain in it. Idles have speed 0: the speed calibration, family
// rates and capsule profile do not change (asserted offline). Kill switch UTPPSetMMCrouchIdleExclude (off = the P0 masks).
constexpr const char* kCrouchExclDefault = "";
bool g_crouchExclOn = true;
std::string g_crouchExclTags = kCrouchExclDefault;
uint64_t g_crouchExclMask = 0;     // tag bits the last mask computation took out of the crouch set (0 = none)
int g_crouchExclClips = 0;         // clips of the crouch set taken out (in the skipped case: that would have gone)
int g_crouchExclFound = 0;         // names of the list found among the database's tags
int g_crouchExclMissing = 0;       // ... not found
int g_crouchIdleLoops = 0;         // looping, searchable idle-class clips in the crouch set in use
bool g_crouchExclSkipped = false;  // fallback: no crouch idle loop would remain, full crouch set kept

// OR of the masks of the tags whose name equals one of the comma-separated names of aList (case-sensitive, spaces around a
// name ignored). aFound / aMissing: names of the list found / not found among the database's tags.
uint64_t TagMaskExactLocked(std::string_view aList, int* aFound = nullptr, int* aMissing = nullptr)
{
    uint64_t m = 0;
    int found = 0, missing = 0;
    size_t p = 0;
    while (p <= aList.size())
    {
        size_t e = aList.find(',', p);
        if (e == std::string_view::npos)
            e = aList.size();
        std::string_view n = aList.substr(p, e - p);
        while (!n.empty() && (n.front() == ' ' || n.front() == '\t'))
            n.remove_prefix(1);
        while (!n.empty() && (n.back() == ' ' || n.back() == '\t'))
            n.remove_suffix(1);
        if (!n.empty())
        {
            bool hit = false;
            for (uint32_t t = 0; t < g_db.NumTags() && t < 64; ++t)
                if (g_db.TagName(t) == n)
                {
                    m |= 1ull << t;
                    hit = true;
                }
            ++(hit ? found : missing);
        }
        p = e + 1;
    }
    if (aFound)
        *aFound = found;
    if (aMissing)
        *aMissing = missing;
    return m;
}

// Looping, searchable idle-class clips (steer::ClassifyClip by name) allowed by aMask.
int CrouchIdleLoopsLocked(uint64_t aMask)
{
    int n = 0;
    const auto clips = g_db.Clips();
    for (size_t c = 0; c < clips.size(); ++c)
        if ((clips[c].tagMask & aMask) && (clips[c].flags & kClipLoop) && clips[c].numSearchable > 0 &&
            steer::ClassifyClip(g_db.ClipName(uint32_t(c))) == steer::kClassIdle)
            ++n;
    return n;
}

// Applies the exclusion to g_stateTagMask[kLocoCrouch] (called by ComputeStateTagMasksLocked; off = no change).
void ApplyCrouchExclusionLocked()
{
    g_crouchExclMask = 0;
    g_crouchExclClips = 0;
    g_crouchExclFound = g_crouchExclMissing = 0;
    g_crouchExclSkipped = false;
    g_crouchIdleLoops = 0;
    if (!g_crouchExclOn || !g_db.Loaded())
        return;
    const uint64_t full = g_stateTagMask[kLocoCrouch];
    const uint64_t ex = TagMaskExactLocked(g_crouchExclTags, &g_crouchExclFound, &g_crouchExclMissing) & full;
    const uint64_t cut = full & ~ex;
    const auto clips = g_db.Clips();
    for (size_t c = 0; c < clips.size(); ++c)
        if ((clips[c].tagMask & full) && !(clips[c].tagMask & cut))
            ++g_crouchExclClips;
    const int left = ex ? CrouchIdleLoopsLocked(cut) : 0;
    if (ex == 0 || left == 0)
    {
        g_crouchExclSkipped = ex != 0; // no listed tag in the crouch set: nothing to exclude (not a fallback)
        if (ex == 0)
            g_crouchExclClips = 0;
        g_crouchIdleLoops = CrouchIdleLoopsLocked(full);
        return;
    }
    g_stateTagMask[kLocoCrouch] = cut;
    g_crouchExclMask = ex;
    g_crouchIdleLoops = left;
}

void ComputeStateTagMasksLocked()
{
    uint64_t excl = 0;
    if (g_excludeLands)
        excl = BaseMatcher::TagMaskContaining(g_db, "_Lands_") | BaseMatcher::TagMaskContaining(g_db, "FromTraversal");
    g_stateTagMask[kLocoStand] = BaseMatcher::TagMaskContaining(g_db, "Stand_") & ~excl;
    g_stateTagMask[kLocoCrouch] = BaseMatcher::TagMaskContaining(g_db, "Crouch_") & ~excl;
    g_stateTagMask[kLocoSprint] = g_stateTagMask[kLocoStand];
    ApplyCrouchExclusionLocked(); // loco C1a (off: the masks above unchanged)
}
bool g_slowWalkOn = true;   // kill switch UTPPSetMMSlowWalk (0.3.4, default on); off = 0.3.3 map / rate floor
float g_walkRateMin = 0.25f;
bool g_lastFloored = false;
void (*g_log)(const char*) = nullptr;
uint64_t g_traceLastTransitions = 0;
int64_t g_traceLastSummary = 0;
double g_traceMaxSearchUs = 0.0; // max search time seen while tracing (diagnostic)
int64_t g_lastTickTicks = 0;
uint64_t g_publishes = 0;
uint64_t g_matcherUpdates = 0;
uint64_t g_staleFades = 0;
double g_lastUpdateUs = 0.0;

PoseSnapshot g_snap{}; // reused publish buffer (no allocation)

// Phase 4.5 foot lock (contact-based, two-bone leg IK) on the matcher output, driven by the real capsule motion.
FootLocker g_footLock;
bool g_footLockOn = true;
double g_lastFootLockUs = 0.0;
// loco C0 (2026-10-13; default on, off = the loco P0 trace / status text): feet diagnostics - one "UTPP MMTRACE feet" line after
// every MMTRACE TRANS / tick line, one "UTPP MMTRACE footstep" line per foot event (FootLockDiagnostics::lastEvent), Status
// segment feet="...". Kill switch UTPPSetMMFeetDiag.
bool g_feetDiagOn = true;
uint64_t g_feetEventSeen[2] = {0, 0}; // last foot event sequence handled per foot (also while the trace is off: no backlog)
uint64_t g_feetEvents = 0;            // foot events seen
// loco C1b (2026-10-13; default crouch, 0 = the loco C1a build): plant / replant in the foot locker (FootReplantConfig) instead of the
// 0.5.9 soft slip and the settle, per stance: bit 1 stand / sprint, bit 2 crouch (UTPPSetMMFootReplant, CET mm.footReplant). The
// Stops profile follows the playing clip class (idle / stop). Off for a stance = the 0.5.9 path bit for bit.
constexpr int32_t kReplantStand = 1, kReplantCrouch = 2;
int32_t g_footReplantMask = kReplantCrouch;
FootReplantConfig g_replantCfg{};
// batch 2 knee fix (2026-10-13; default 3, 0 = the batch 1b leg pipeline bit for bit): the right knee "broke" on starts. Bit 1:
// StanceGuard knee swivel target 0 in crouch (faded by its filter); bit 2: FootLocker two-bone pole carried from the animated
// hip-ankle axis (legik::SolveTwoBone aPoleCarry). UTPPSetMMKneeFix, CET mm.kneeFix. Crouch replant unchanged. A swivel rate
// limit (180 deg/s) was measured and left out: no change in crouch, standing turn starts 10 -> 14 break frames.
// batch 5 (2026-10-13) bit 4: StanceGuard knee swivel held while the foot is planted, released with the lift-off contact
// (StanceGuard::SetPlantedHold; standing turn starts on utpp_base: 25 -> 11 break frames, added 15 -> 1). Mask range 0..7,
// default 7 (v6 / v7b: standing restarts 5 -> 7, turn starts 10 -> 11 - the installed motion-only database decides).
constexpr int32_t kKneeFixCrouchSwivel = 1, kKneeFixPoleCarry = 2, kKneeFixPlantedHold = 4;
constexpr int32_t kKneeFixAll = kKneeFixCrouchSwivel | kKneeFixPoleCarry | kKneeFixPlantedHold;
int32_t g_kneeFixMask = kKneeFixAll; // batch 5: default 7 (was 3)

// Phase 5.1: body yaw follows the MM animation (replaces the fixed-time Locomotion body turn while active).
bool g_bodyTurnOn = true;        // kill switch UTPPSetMMBodyTurn (default on)
bt::BodyYawController g_bodyCtl; // command law + smoothed animation root yaw rate
bool g_haveTarget = false;       // last feed carried a valid world target (Locomotion target valid + stick moved)
float g_targetYaw = 0.0f;        // rad, world
bool g_bodyTurnFed = false;      // last feed used the body-turn matcher input
float g_rawAnimRate = 0.0f;      // rad/s real time, last feed
int64_t g_lastCmdTicks = 0;
float g_lastCmdDeg = 0.0f, g_lastCmdCurDeg = 0.0f, g_lastCmdBaseDeg = 0.0f, g_lastCmdDt = 0.0f;
uint64_t g_bodyCmds = 0, g_bodyPassThrough = 0;
bt::TargetLatch g_latch;         // 0.3.1: world target held during a turn (no camera feedback)

// Phase 5.3 (0.3.3): offset root (visual body lags the capsule, spring back) and capsule acceleration matching.
OffsetRoot g_offsetRoot;
bool g_offsetRootOn = true;      // kill switch UTPPSetMMOffsetRoot (default on); off = 0.3.2 pose path
OffsetRootConfig g_offsetCfg = []
{
    OffsetRootConfig c;
    c.saturatedTime = 0.3f; // 0.3.4 saturation guard (UTPPSetMMOffsetSaturation)
    return c;
}();
bool g_startRefaceOn = true; // 0.3.4 kill switch UTPPSetMMStartReface (default on)
bool g_accelOn = true;           // kill switch UTPPSetMMAccelMatch (default on); off = vanilla Acceleration stat
// Target capsule acceleration per locomotion state (game m/s^2), measured from the motion start clips (full Dense DB,
// Source/tools/mmbake/measure_root_speed.py, 10->90 % mean acceleration scaled by the speed map and the playback rate):
// Run_Start 3.75, Walk_Start 3.0, Sprint_Start 6.96, Crouch_Start 12.2. The script only ever LOWERS the stat (multiplier
// <= 1), so values above the vanilla acceleration are no-ops. 0 = no change for that state.
float g_accelTarget[4] = {0.0f, 3.75f, 12.2f, 6.96f};
float g_accelBase = 0.0f, g_accelMul = 1.0f, g_accelLastTarget = 0.0f;
float g_accelObserved = 0.0f;
int64_t g_accelSampleTicks = 0;
int32_t g_accelSampleLoco = 0, g_accelSampleDetail = -1;
uint64_t g_accelQueries = 0, g_accelActive = 0;

// Phase 5.3b (0.3.4): playback rate from the playing clip's root speed, stop intent + trajectory stop half-life.
bool g_rootRateOn = true;        // kill switch UTPPSetMMRootRate (default on); off = loop-speed map rate (0.3.3)
RootRate g_rootRate;
float g_loopRate = 1.0f;         // rate the 0.3.3 loop map would use (diagnostic)
float g_rootSpeedDb = 0.0f;      // root speed of the played segment (database m/s), last feed
bool g_stopSyncOn = true;        // kill switch UTPPSetMMStopSync (default on); off = 0.3.3 stop input
float g_stopHalflife = 0.05f;    // s (matcher time); chosen offline (stop test sweep)

// Phase 5.3c (0.3.5): steering stability (all default on; all off = 0.3.4). See mm/Steering.hpp.
bool g_steerOn = true;           // kill switch UTPPSetMMSteerFollow: animation-led turns only for sharp changes
steer::Steering g_steer;
bool g_followFed = false;        // last feed used the follow mode (body spring + curved trajectory)
bool g_dwellOn = true;           // kill switch UTPPSetMMDwell: minimum segment dwell + relative cost threshold
float g_minDwell = 0.3f, g_dwellRatio = 0.5f;
bool g_curationOn = true;        // kill switch UTPPSetMMCuration: per-search clip-class exclusion
bool g_curMovingOn = true;       // 0.3.6 kill switch UTPPSetMMCurationMoving (stop / start / transition rules)
StanceGuard g_stanceGuard;       // 0.3.6 stance-width guard (mm/StanceGuard.hpp)
StanceGuardConfig g_stanceCfg;
bool g_stanceOn = true;          // kill switch UTPPSetMMStanceGuard (default on); off = 0.3.5 pose path
// 0.3.7 (all default on; off = 0.3.6)
bool g_rootWarpOn = true;        // UTPPSetMMRootWarp: animation root velocity turned towards the capsule velocity
float g_rootWarpMax = 60.0f * 0.017453292f;
bool g_leanOn = true;            // UTPPSetMMHeadingLean: visual body leans towards the travel heading (offset yaw)
bool g_stanceSwingOn = true;     // UTPPSetMMStanceSwing: stance guard on the swing foot only
// 0.3.8 large turns (UTPPSetMMLargeTurn, default on; off = 0.3.7): speed-class pivots, pivot capsule acceleration,
// no body-turn angular velocity in the stop prediction, stop yaw-rate clamp, dwell exemption on recovery.
bool g_largeTurnOn = true;
float g_pivotAccel[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // m/s^2 per state during a large turn; default OFF (unverified)
float g_stopYawRateMax = 1.0f;                    // rad/s matcher input yaw rate while stopping
bool g_pivotAccelActive = false;
bool g_prevBtFed = false;
// 0.3.9 (default on; off = 0.3.8)
bool g_slowLoopOn = true;        // UTPPSetMMSlowWalkLoop: no start clips in the slow-walk gate after the start window
steer::CurationConfig g_curCfg = []
{
    steer::CurationConfig c;
    c.speedBands = true; // 0.5.4 (UTPPSetMMSpeedBands); the struct default (off) is the 0.5.3 rule set
    c.crouchRules = true; // 0.5.6 (UTPPSetMMCrouchRules); the struct default (off) is the 0.5.5 rule set
    return c;
}();
std::vector<uint8_t> g_clipClasses;
uint32_t g_lastExclude = 0;
bool g_slowCapOn = true;         // kill switch UTPPSetMMSlowWalkCapsule: slow-walk gate from the real capsule speed
SlowWalkGate g_slowGate;
bool g_slowGateActive = false;
bool g_prevStick = false;
float g_stickEdgeTime = 1e6f;    // s since the last stick press / release (dwell exemption window)
// 0.5.2: stance-change transition clips only right after a real stand <-> crouch change (steer::kClassStanceTransition)
int g_prevStance = 0;            // 0 other, 1 stand (stand / sprint), 2 crouch
float g_sinceStance = 1e6f;      // s since the last stand <-> crouch change
uint64_t g_stanceChanges = 0;
// 0.5.3 (all default on; off = 0.5.2): stick filter, per-source rate factors / slow-walk reference, start rules
steer::StickFilter g_stick;
bool g_stickFilterOn = true;
bool g_stickHeld = false;
bool g_familyRateOn = true;
bool g_cmdDtFromFeed = false;    // test hook (SetCommandDtFromFeed)
float g_rateFactor = 1.0f;       // smoothed factor of the playing clip
float g_rateFactorTarget = 1.0f;
float g_slowRef[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // per loco state: slowest forward loop (0 = map point)
// 0.5.4 speed bands (mm/SpeedBands.hpp; on/off = g_curCfg.speedBands, off = 0.5.3)
BandEdges g_bandEdges;
BandConfig g_bandCfg;
BandAvailability g_bandAvail;
SpeedBandTracker g_band;
int g_lastBand = 0;
// 0.5.6 (default on; off = 0.5.5): gate debounce (mm/GameIntegration.hpp GateDebounce) and crouch clip rules
// (steer::CurationConfig::crouchRules: stance-transition direction / speed, lateral loops for forward motion)
GateDebounce g_gateDebounce;
GateDebounceConfig g_gateDebounceCfg;
uint32_t g_rawGate = 0;           // gate before the debounce (status / trace)
int g_lastPsmDetail = -1;         // PSM.LocomotionDetailed of the last blocking feed (reds 0.5.6; -1 = not reported)
int g_lastSoftDetail = -1;        // ... of the last debounced (suppressed) soft closure
int32_t g_lastGoodLoco = 0;       // last stand / crouch / sprint state (used by the matcher logic while debouncing)
bool g_lateralOk[4] = {false, false, false, false}; // per loco state: a non-lateral forward loop / move exists
uint64_t g_lateralExcluded = 0;   // searches with kClassLateral excluded
// 0.5.7 (default on; off = 0.5.6), see MotionHost.hpp
bool g_crouchDirTagOn = true;     // UTPPSetMMCrouchDirTag: crouch loops / moves with a side / back direction tag = lateral
bool g_stanceOnceOn = true;       // UTPPSetMMStanceOnce: stance transitions once per change, first frames only
bool g_turnCommitOn = true;       // UTPPSetMMTurnCommit: no turn-class restart while a turn clip turns
bool g_standIdleOn = true;        // UTPPSetMMStandIdle: standstill without stick (or gated out) -> idle candidates only
bool g_stanceUsed = false;        // a stance transition was selected since the last stance change
uint64_t g_turnCommitHolds = 0, g_standIdleSearches = 0;
// stage standstill idle (UTPPSetMMIdleHold, default on; off = batch 1). At a standstill the motion stop clips re-entered their own
// tail (M_Neutral_Crouch_Stop_FR_Rtfoot f=153 every 0.96 s, v1 / v2 / v3 / v6 / default): the end-of-clip search excludes the
// playing frame, and the stop tail (standing still, same feet) is always cheaper than the idle loop's pose. While the standstill
// rule has held for kIdleHoldDelay s and the stance's clip set has an idle loop: once the playing clip is an idle, or a non-idle
// clip has <= kIdleHoldTailFrames left, stop clips are no longer candidates (the playing one continues) - the end-of-clip search
// goes to the idle family (inertialized as every transition); idle-family / idle-break choices are unchanged.
bool g_idleHoldOn = true;
float g_idleHoldStill = 0.0f;      // s the standstill rule has held (feed time)
uint64_t g_idleHoldSearches = 0;   // feeds with the stop exclusion applied
uint64_t g_idleHoldNoLoop = 0;     // feeds where it would apply but the clip set has no idle loop
constexpr float kIdleHoldDelay = 0.25f;
constexpr int32_t kIdleHoldTailFrames = 12; // 0.4 s at 30 fps
bool g_turnCommitted = false;     // last feed: turn commit active
std::vector<int32_t> g_turnEnd;   // per clip: local frame where the root turn is 90 % done (-1 = no turn clip)
std::vector<int8_t> g_turnSign;   // per clip: +1 CCW (left), -1 CW (right), 0 = no root turn >= 30 deg
bool g_startTurnOn = true;        // UTPPSetMMStartTurn: turn direction / slow turning starts / start rate
std::vector<uint8_t> g_matchClasses[4]; // matcher class sets: 0 base, 1 left turn wanted, 2 right turn wanted, 3 aim strafe (B2)
int g_activeClassSet = 0;
// ---- B2 strafe S1 (2026-10-13; UTPPSetMMStrafe, default on - CET bodyturn.aimStrafe; off = the 0.6.5 path bit for bit: every
// branch below is gated on the strafe mode, which never leaves Off while the switch is off). Aim strafe = the runtime motion
// RotationMode=Strafe while V aims with a weapon in hand (PSM UpperBody 6, upper-body mask Aim state): the legs walk in the stick
// direction relative to the body (facing fixed, trajectory yaw = the measured body step at rate 0), class set 3 (Strafe.hpp
// ComputeStrafeProfiles: ROOT-turning / turn-class / off-band pattern clips -> kClassStrafeTurn, facing-fixed patterns allowed),
// per-direction loop speed map + loop own-speed rate law, slow-walk gate bypassed while the mode is Active / Exiting (the
// strafe rate law replaces the slow-walk rate / map floor), no turn-rate boost, offset-root heading lean off and root warp
// <= rootWarpMaxDeg (30). Precedence for the later air area: strafe reads the previous-frame g_airOwnsPrev (stub: false).
bool g_strafeOn = true;                 // UTPPSetMMStrafe
strafe::StrafeConfig g_strafeCfg;        // UTPPSetMMStrafeParams (unarmed, rootTurnMaxDeg, rootWarpMaxDeg, exitHold)
float g_strafeJogF = 0.0f;               // UTPPSetMMStrafeParams jogF (S4: 0 = walk gait; > 0 = run gait, forward game speed)
constexpr float kStrafeAimWalk = 1.65f;  // m/s vanilla aim-walk capsule speed (measured 1.63-1.67): strafe intent / speed-map point
strafe::StrafeProfiles g_strafeProf;     // RecomputeDerivedClassDataLocked (after the state tag masks)
strafe::StrafeMode g_strafeMode;
strafe::StrafeRateSmoother g_strafeRate;
bool g_airOwnsPrev = false;              // previous feed: the air area owns the body (stub until the air stage)
bool g_strafeEdge = false;               // this feed: strafe entered / exited (dwell exemption)
int g_strafeStance = 0, g_strafeGait = 0;
// B2 strafe S4 jog (AimJogTarget, the reds MaxSpeed multiplier query): last answer (m/s, 0 = vanilla), queries, non-zero answers
float g_aimJogLastTarget = 0.0f;
uint64_t g_aimJogQueries = 0, g_aimJogOnAnswers = 0;
// UTPP-MOTION SEAM: this feed, strafe forced off by a v2 span (g_motionHeldMatcher) while its Wanted conditions hold: the aim jog speed
// UTPP-MOTION SEAM stays (AimJogTarget) - the jog is a capsule speed, not a strafe animation. Always false with v2 off.
bool g_aimJogMotionHeld = false; // UTPP-MOTION SEAM
float g_strafeDirDeg = 0.0f;             // last strafe desired-motion direction (deg, CCW from the body forward)
int g_strafeRateSrc = 0;                 // 0 off, 1 speed map, 2 loop own speed
uint64_t g_strafeSearches = 0;           // matcher searches while strafing
double g_strafeSUsSum = 0.0, g_strafeSUsMax = 0.0; // search time while strafing (us)
bool g_strafeLogged = false;             // availability summary logged for this database
bool g_strafePatterns = true;            // internal (SetStrafePatterns, tests): facing-fixed patterns allowed while strafing
// ---- B2 strafe S2 chest align (2026-10-13; UTPPSetMMChestAlign, default on - CET bodyturn.aimChestAlign; off, or the strafe
// mode Off = the S1 pose path bit for bit: ChestAlign runs only while the strafe mode is Active / Exiting). mm/ChestAlign.hpp:
// the motion facing-fixed side / back loops twist the chest about the vertical axis; while aiming the game owns Spine1+ (upper-body
// mask), so that twist turned the whole aimed upper body. ChestAlign rotates the Spine local rotation (Hips, legs and every
// other bone untouched) towards the stance's forward-walk chest orientation (stand / crouch references, built per database in
// RecomputeDerivedClassDataLocked). Leg pipeline order: StanceGuard -> ChestAlign -> FootLocker.
ChestAlign g_chestAlign;
ChestAlignConfig g_chestCfg;             // UTPPSetMMChestAlignParams (swayKeep 0.5, chestMaxDeg 60)
bool g_chestOn = true;                   // UTPPSetMMChestAlign
std::vector<BoneTransform> g_chestPose;  // ChestAlign output buffer (sized with the references, no per-frame allocation)
bool g_chestOut = false;                 // the last leg pipeline ran ChestAlign (g_chestPose = its output)
bool g_chestValid = false;               // this feed: ChestAlign diagnostics valid (dev / correction of this feed)
bool g_chestNoRefLogged = false;         // "no reference clips" logged for this database
uint64_t g_chestTeleports = 0;           // offset root teleports seen (a new one resets ChestAlign)
// ---- AIR HOLD stage (2026-10-13; default auto; off = the 1cf5307 path bit for bit) ----
// mm/AirState.hpp phase machine. UpdateAirLocked runs after the hold block of Feed (before the matcher block and before
// ComputeLegContextLocked); the strafe decision reads the previous feed's ownership (g_airOwnsPrev): AirOwns > Strafe > ground.
// This stage ships the data-free Hold mode only (auto / clips fall back to hold: no .airz table is loaded yet).
// AIR CLIPS stage: Heavy impact threshold in CP gravity. motion's 700 cm/s is a ~2.4 m drop under motion gravity (~10.4 m/s^2); the
// same height under the CP fall gravity (16 m/s^2) lands at ~8.7 m/s, so a 2 m drop stays Light and a 6 m drop is Heavy. Only the
// clips mode reads it (land weight / recover time); hold hands back at the touchdown.
air::AirState g_air{[] {
    air::AirConfig c;
    c.heavyVz = 8.7f;
    c.physFall = c.touchGuard = true; // BATCH 6 AIR stage: UTPPSetMMAirRobust default on (female v1 / male V air findings)
    return c;
}()};
int32_t g_airSetting = 0;               // UTPPSetMMAir: 0 auto, 1 clips, 2 hold, 3 off
bool g_airProbeOn = true;               // UTPPSetMMAirProbe (CET mm.airProbe)
bool g_airSteerOn = false;              // UTPPSetMMAirSteer (CET mm.airSteer, default off): in-air body steering
// AIR STEER: optional, default off. In air the capsule travel heading is authoritative. The desired stick heading may
// lead it by at most 35 degrees, so an abrupt camera/stick reversal cannot spin the airborne body ahead of its momentum.
// The normal follow controller limits angular velocity and acceleration. This affects body yaw only (no turn clips).
// AIR STEER repair: only effective while the hold pose keeps the body in the air (g_effGate 0: UTPPSetMMHoldPose on, hold eligible);
// without the hold the air area does not own the body and nothing is armed (airSteerFed 0); airSteerBlocked counts armed feeds
// whose effective gate is still not 0 (expected 0). Game movement remains authoritative for capsule velocity.
constexpr float kAirSteerMaxRate = 200.0f * 3.14159265f / 180.0f; // rad/s
constexpr float kAirSteerMaxLead = 35.0f * kDegToRad;
constexpr float kAirSteerMinSpeed = 0.5f; // m/s: velocity direction is not meaningful near rest
bool g_airSteerFed = false;             // this feed armed the in-air body steering
float g_airSteerVelocityYaw = 0.0f;     // current world-space capsule velocity heading (radians)
uint64_t g_airSteerFeeds = 0, g_airSteerCmds = 0;
// AIR STEER repair: armed feeds whose effective gate is still not 0 - BodyTurnActive stays false there, the setting has no effect
// (expected 0: the air area owns the body only under the hold pose; session counter, like the other air counters)
uint64_t g_airSteerBlocked = 0;
float g_airSteerMaxRateDeg = 0.0f;      // largest |applied rate| of an air steer command (deg/s)
air::AirDecision g_airDec;              // this feed's decision (defaults = no effect)
bool g_airOwns = false;                 // this feed: the air area owns the body
bool g_airLastAirborne = false;         // last feed: off the ground (air feed, else the PSM detail) - UTPPMMAirWantsProbe
float g_airLastVz = 0.0f;               // last feed: capsule vz
bool g_airLastRelease = false;          // last feed: an external release reason (no episode possible) - no probe
uint64_t g_airFeeds = 0, g_airFreshFeeds = 0, g_airProbes = 0, g_airProbeHits = 0, g_airProbeWants = 0;
uint64_t g_airOwnedFeeds = 0, g_airSuppressed = 0;
double g_airUs = 0.0, g_airUsMax = 0.0;
air::Phase g_airPrevPhase = air::Phase::Ground;
// ---- AIR CLIPS stage (2026-10-13): .airz root_z table + air clip playback (mm.air auto / clips with a valid table) ----
// The table is parsed in LoadDatabase / TrySwapLocked (bytes read by the switch worker: no file I/O on the game thread) before
// InitMatcherLocked, which applies it (BaseMatcher::SetRootZ / SetLoopOverride). Missing / invalid table = Hold (reason in
// the status). AirDecision play actions are resolved through AirTable::Pick (explicit fallback table) and ForcePlay'ed.
airtab::AirTable g_airTable;
bool g_airTableValid = false;
std::string g_airTableReason = "noTable";
std::array<uint32_t, 6> g_airLegChain{};
uint32_t g_airLegChainN = 0;
int32_t g_airPlayMeta = -1;       // AirTable meta index of the clip forced last (-1 none / handed back)
int32_t g_airPlayLocal = -1;      // its entry frame (clip-local)
int32_t g_airLastCleanFrame = -1; // last ground frame with root_z 0 (queryBoneFrame hand-back)
uint64_t g_airPlays = 0, g_airPlayFails = 0, g_airQueryOverrides = 0;
// AIR CLIPS touchdown: cap a predicted Land clip's progress on the first genuine Recover feed.
// A delayed physical touchdown must not play the crouched/deep post-contact part before the capsule touches the floor.
bool g_airLandWasApproach = false;
uint64_t g_airLandRealigns = 0;
// Keep the touchdown Land pose in the published lower body briefly after Recover hands control to ground MM.
// The ordinary matcher search remains immediate; only the legs/Hips get a bounded visual crossfade.
constexpr float kAirLandGroundBlendSeconds = 0.18f;
float g_airLandGroundBlendLeft = 0.0f;
bool g_airLandGroundBlendPending = false;
bool g_airLandGroundBlendOutputValid = false;
std::array<BoneTransform, integ::kMaxMMBones> g_airLandGroundPose{};
std::array<BoneTransform, integ::kMaxMMBones> g_airLandGroundOutput{};
uint32_t g_airLandGroundBones = 0;
uint64_t g_airLandGroundBlends = 0;
float g_airLandGroundElapsed = 0.0f;
float g_airLandGroundStartHipsZ = 0.0f;
float g_airLandGroundFirstHipsError = 0.0f;
bool g_airLandGroundStartPending = false;
// A physical Hold-mode touchdown can immediately return the ground matcher to a turn/shuffle clip
// while the capsule accelerates back to running speed. For that short transition, allow the
// existing one-foot-at-a-time replant algorithm to move a planted foot instead of stretching its
// old landing anchor against the new clip. Ordinary ground, AIR OFF and v2 ownership are unaffected.
constexpr float kAirHoldGroundReplantSeconds = 0.50f;
float g_airHoldGroundReplantLeft = 0.0f;
bool g_stanceAirBypassed = false; // current leg pipeline: StanceGuard did not run, output must not be selected
std::span<const BoneTransform> g_legPipelineOutput{}; // exact published stage source, valid through the current feed
bool g_motionHandbackFeed = false; // exact first v1 feed following v2 ownership, held until post-publish mask decision
// AIR CLIPS repair: the table is applied to the matcher (root_z subtraction + fall loop override) only while the setting can play
// clips (mm.air auto / clips); hold / off sample the database exactly as 1cf5307. Called after Init and by SetAir.
bool g_airTableApplied = false;
void ApplyAirTableLocked()
{
    const bool want = g_airTableValid && (g_airSetting == 0 || g_airSetting == 1) && g_matcher.Ready();
    if (want == g_airTableApplied)
        return;
    const int32_t loopClip = g_airTable.FallLoopClip();
    if (want)
    {
        g_matcher.SetRootZ(g_airTable.RootZ());
        if (loopClip >= 0 && g_airTable.LoopOk())
            g_matcher.SetLoopOverride(loopClip, true);
    }
    else if (g_airTableApplied && g_matcher.Ready())
    {
        g_matcher.SetRootZ({});
        if (loopClip >= 0)
            g_matcher.SetLoopOverride(loopClip, false);
    }
    g_airTableApplied = want && g_matcher.HasRootZ();
}
bool g_startRateActive = false;
int g_bodyFollowDir = 0;          // 0.5.7: +1 / -1 = the body turns the long way with the playing clip (|error| >= 160 deg)
uint64_t g_againstSearches = 0, g_startRateFeeds = 0;
// 0.6.3 turn rate (default on; off = 0.6.2), see MotionHost.hpp SetTurnRate
bool g_turnRateOn = true;
float g_turnRateMin = 0.9f, g_turnRateMax = 1.1f; // 0.6.4: shipped default stays 1.1 (foot-lock regression guard 15.5 cm); the CET setting bodyturn.turnRateMax (default 1.4) raises it in game via UTPPSetMMTurnRateParams - 20:24 test: run turns still too slow at 1.1
constexpr float kTurnRateMinCapsule = 1.0f; // m/s, recent capsule speed (decaying max): slower = 0.5.7 start rate / slow walk
constexpr float kTurnRateRecentHl = 0.5f;   // s, half-life of the recent capsule speed (a flick brakes through 0)
constexpr float kTurnRateBand = 0.35f;      // root below its clip peak by this fraction -> full weight
constexpr float kTurnRateEps = 0.5f;        // m/s, as RootRate
constexpr float kTurnRateRiseHl = 0.015f;   // s, weight smoothing up (a plant entered mid-way: ~1 frame)
constexpr float kTurnRateFallHl = 0.04f;    // s, weight smoothing down (back to the loop rate)
float g_turnRateW = 0.0f;                   // smoothed weight 0..1
float g_turnRateTarget = 1.0f;              // last target rate (clamped root match)
bool g_turnRateActive = false;              // last feed: the rate was raised
uint64_t g_turnRateFeeds = 0;
// 0.6.4 per-turn summary (one info line when g_turnRateActive goes true -> false)
struct TurnRateAcc
{
    int32_t clip = -1;
    uint32_t frames = 0;
    double rateSum = 0.0, capsuleSum = 0.0, rootSum = 0.0, dur = 0.0;
    float peak = 0.0f;
};
TurnRateAcc g_turnAcc;
std::string g_turnRateLastSummary = "none";
uint64_t g_turnRateTurns = 0;
float g_turnRecentSpeed = 0.0f;             // decaying max of the capsule speed (m/s)
std::vector<float> g_clipPeakRoot;          // per clip: peak root speed (database m/s), turn-class clips only (0 = other)
// 0.5.8 (default on; off = 0.5.7), see MotionHost.hpp
float g_speedCapFactor = 1.25f;   // UTPPSetMMSpeedCap: unsupported above factor x game sprint speed (0 = off)
bool g_overSpeed = false;         // last feed: capsule above the speed cap (gate bit kGateOverSpeed)
float g_rawSpeed = 0.0f;          // last feed: measured capsule speed (before the query clamp)
uint64_t g_overSpeedFeeds = 0, g_queryClamped = 0;
float g_costCap = 100.0f;         // UTPPSetMMCostCap: best-cost cap of a transition (0 = off)
float g_costCapHold = 0.3f;       // s matcher time the continuing frame may be kept in a row
uint64_t g_traceCapHolds = 0, g_traceCapRetries = 0, g_traceCapForced = 0; // trace: last logged cost-cap counters
bool g_stanceCapOn = true;        // UTPPSetMMStanceCap: commit only for an allowed playing clip, no stance transition above the cap
uint64_t g_commitNotAllowed = 0;  // turn commits skipped because the playing clip left the clip set (stance change)
bool g_turnSettleOn = true;       // UTPPSetMMTurnSettle: no same-direction turn restart right after a committed turn
int g_commitSign = 0;             // turn direction of the last committed clip
bool g_prevCommitted = false;
float g_settleLeft = 0.0f;        // s of the settle window left
int g_settleSign = 0;
bool g_settling = false;          // last feed: settle window excluded the turn classes
uint64_t g_settleHolds = 0;
bool g_loopBandsOn = true;        // UTPPSetMMLoopBands: loops / speed transitions of the query speed band only
SpeedBandTracker g_loopBandTracker, g_loopBandHiTracker;
int g_lastLoopBand = 0;           // band of the capsule speed (1 walk, 2 run, 3 sprint; 0 = not applied)
int g_lastLoopBandHi = 0;         // band of the stick intent (state speed x |stick|, >= query speed; >= g_lastLoopBand)
int g_activeLoopBand = 0;         // bands of the active matcher class set (lo + 4 * hi)
std::vector<int8_t> g_loopBandOf; // per clip: loop speed class / transition source class (0 = not banded)
int g_loopAvail[4] = {0, 0, 0, 0};// stand / sprint loops per speed class
std::vector<uint8_t> g_classScratch;
uint64_t g_loopBandSearches = 0;
bool g_startBindOn = true;        // UTPPSetMMStartBind: body turn bound to the playing start segment's root turn direction
// ---- 0.5.9 (Phase 5.3, default on; off = 0.5.8): capsule sync, offset catch-up, trajectory calibration ----
bool g_capSyncOn = true;          // UTPPSetMMCapsuleSync: per-database start/stop profiles -> capsule acceleration / stop deceleration
// b5 testorder repair: UTPPSetMMSwapReset (default on): a database swap clears the playback state of the old database's clips
// (ResetDbPlaybackStateLocked + the matcher's cost-cap hold); off = 0.7.0 swap (that state carries over). swapCarry / swapPost =
// PlaybackCarryLocked() bits just before the swap / after the swap's resets (diagnostic, status).
bool g_swapResetOn = true;
uint64_t g_swapResets = 0;
uint32_t g_swapCarry = 0;
uint32_t g_swapPost = 0;
bool g_catchupOn = true;          // UTPPSetMMOffsetCatchup: offset root soft limit + time-bounded convergence, foot soft lock
bool g_trajCalOn = true;          // UTPPSetMMTrajCal: trajectory velocity half-life from the capsule profile
CapsuleProfile g_capProfile;
float g_decelBase = 0.0f, g_decelMul = 1.0f, g_decelLastTarget = 0.0f;
int64_t g_decelSampleTicks = 0;
int32_t g_decelSampleLoco = 0, g_decelSampleDetail = -1;
uint64_t g_decelQueries = 0, g_decelActive = 0;
float g_catchMaxDistance = 0.30f, g_catchTime = 0.6f, g_catchSoft = 0.15f, g_catchHalflife = 0.06f;
float g_slipStart = 0.05f, g_slipHalflife = 0.02f; // loco sim sweep 0.08/0.04 -> 0.05/0.02 (Section059)
bool g_slipProfileOn = false;    // the Start-class slip profile ran on the last feed (ApplyLegPipelineLocked)
float g_slipStartStart = 0.09f; // Start-class slip start (FootSlipProfile start; 0 / <= g_slipStart = the single slip), motionhost::SetFootStartSlip (C++ API; a native + CET setting are not wired yet)
float g_trajHlApplied = 0.0f;
uint64_t g_trajCalFeeds = 0;
constexpr float kMinDecelTarget = 10.0f; // m/s^2
// 0.6.0 (default on; off = 0.5.9), see MotionHost.hpp
MeleeGate g_meleeGate;            // UTPPSetMMMeleeGate: melee bits of the feed -> kGateMelee (+ 0.3 s release hold)
MeleeGateConfig g_meleeCfg;
bool g_meleeActive = false;       // last feed: kGateMelee raised
uint32_t g_meleeBits = 0;         // last feed: PsmMeleeBits
int g_meleeState = -1;            // last feed: PSM.MeleeWeapon (-1 = not reported)
int g_upperBody = -1;             // last feed: PSM.UpperBody (-1 = not reported)
bool g_mountCutOn = true;         // UTPPSetMMMountCut: vehicle PSM bit / MountingEvent -> weight 0 at once
bool g_mountMuted = false;        // last feed: muted by the vehicle bit
uint64_t g_mountCuts = 0, g_mountEventCuts = 0;
float g_mountEventLeft = 0.0f;    // s the MountingEvent keeps MM muted until the PSM vehicle bit arrives (max kMountEventHold)
constexpr float kMountEventHold = 1.0f;
float g_stanceClipMin = 4.0f;     // UTPPSetMMStanceClipMinSpeed: moving into-crouch transitions only from >= this capsule speed (0 = off)
constexpr float kStanceClipStick = 0.7f; // ... and with the stick >= 0.7 forward
bool g_stanceClipOk = true;       // decided at the last stand -> crouch change
float g_stanceClipSpeed = 0.0f, g_stanceClipStickFwd = 0.0f; // capsule speed / forward stick at that change
uint64_t g_stanceClipSkips = 0, g_stanceClipAllows = 0, g_stanceClipExcl = 0;
// 0.6.2 upper-body mask (default on; off = 0.6.1), see MotionHost.hpp and GameIntegration.hpp (kMaskUpperFromGame)
bool g_upperMaskOn = true;              // UTPPSetMMUpperMask
uint32_t g_maskStates = kMaskStatesAll; // UTPPSetMMUpperMaskStates (melee | aim | vehicle exit window)
UpperMask g_upperMask;                  // from the database bone names (BuildUpperMaskLocked)
bool g_maskLogged = false;              // the mask is logged once per build
UpperMaskBlend g_maskBlend;             // 1 = upper-body bones on the game pose
mm::melee::MaskTimeTracker g_maskTimes; // mask fade times: the eased fade out after a melee weapon release / a soft guard end
bool g_aimUpperFadeOn = true, g_maskFadeFromAim = false;
bool g_weaponUpperOn = true, g_weaponKnown = false, g_weaponHeld = false, g_weaponUpperActive = false;
uint32_t g_maskReasons = 0;             // last feed: kMaskState* bits holding the mask (0 = none)
uint32_t g_maskCovered = 0;             // last feed: gate bits replaced by the mask
// melee full body (2026-11-09, mm/MeleeFullBody.hpp): the lower-body bones fade to the game pose as well while standing in melee
bool g_meleeFullFlag = false;           // UTPPSetMeleeFullBody (script: UTPP third person on foot + covered melee weapon type)
bool g_meleeFullSetting = true;         // UTPPSetMeleeFullBodySetting (user setting "Melee full body")
mm::melee::Gate g_meleeFullGate;        // decision + speed hysteresis
UpperMaskBlend g_meleeFullBlend;        // 1 = lower-body bones on the game pose (PoseSnapshot::lowerScale = 1 - weight)
UpperMaskBlend g_meleeAimBlend;         // melee aim tilt weight: fade of the melee mask reason (PoseSnapshot::meleeAimWeight)
bool g_meleeFullWant = false;           // last feed: full body wanted
bool g_meleeFullStick = false;          // last feed: move intent (stick held)
int32_t g_meleeFullGround = -1;         // last feed: 1 / 0 = air feed on / off the ground, -1 = no air feed
uint64_t g_meleeFullSwitches = 0;       // wanted off -> on
int64_t g_meleeAimLogTicks = 0;         // last 'UTPP MELEEAIM' line (trace, 4 Hz)
bool (*g_meleeAimLogSource)(char*, size_t) = nullptr; // Main.cpp: meleeaim::FormatLogLine (null in the offline builds)
// melee round 3 (2026-11-09 review, mm/MeleeFullBody.hpp): full-body allowed PSM.MeleeWeapon states + the melee guard
uint32_t g_meleeFullStates = mm::melee::kFullBodyDefaultStates; // UTPPSetMeleeFullBodyStates (a setting: kept by resets)
bool g_meleeGuardFlag = false;          // UTPPSetMeleeGuard (script: guard-capable melee weapon drawn, UTPP third person on foot)
bool g_meleeGuardSetting = false;       // UTPPSetMeleeGuardSetting (melee.guard, default off)
bool g_meleeGuardFullSetting = false;   // UTPPSetMeleeGuardFullSetting (melee.guardFullBody, default off: stage 2, legs)
bool g_meleeGuardTiltSetting = false;   // UTPPSetMeleeGuardAimTiltSetting (melee.guardAimTilt, default off)
mm::melee::GuardGate g_meleeGuardGate;  // decision + air debounce
bool g_meleeGuardWant = false;          // last feed: guard wanted
uint64_t g_meleeGuardSwitches = 0;      // guard off -> on
uint32_t g_effGate = kGateMMDisabled | kGateDbNotLoaded | kGateNotMapped; // gate the MM weight follows (g_gate minus g_maskCovered)
bool g_getUpBaseSeen = false; // ragdoll native get-up: the first get-up feed was handled (its gate check ran)
uint64_t g_maskSwitches = 0, g_maskFeeds = 0, g_exitWindows = 0;
const MotionHooks* g_motion = nullptr; // UTPP-MOTION SEAM (null = v1 only)
bool g_prevVeh = false;                 // last feed: PSM vehicle bit
JumpOutLatch g_jumpOut;                 // 2026-10-28: bike jump-out flight - the vehicle bit after the workspot drop is released
uint64_t g_jumpOutFeeds = 0;            // feeds with the vehicle bit released by g_jumpOut
int32_t g_jumpOutLaunchEpoch = 0;       // launches of a jump-out flight since the start (never reset; UTPPVehicleJumpOutEpoch)
bool g_jumpOutExit = false;               // 2026-10-28: this exit window opened as a bike jump-out (the v2 hip-fire hold ends at the touchdown)
// r3 landing: the exit window closes on the first feed with a zero gate, which a short first ground contact (the capsule grazing the
// car body in the fall) already is. g_jumpOutExit then stays up until the touchdown is confirmed, so the chooser keeps its jump-out
// flight (contact hold, no Start row, jump-out landing threshold); mirrors adapter::kJumpOutLandConfirmS / kContactFallMs.
constexpr float kJumpOutHoldConfirmS = 0.10f; // s of ground contact (not a fast fall) after the window closed = the touchdown
constexpr float kJumpOutHoldFallMs = 4.0f;    // m/s downward: a contact feed falling faster than this is no touchdown yet
constexpr float kJumpOutHoldMaxS = 0.5f;      // s after the window closed at most (a flight that never touches down must not keep the state)
constexpr int kJumpOutHoldFeeds = 2;          // consecutive contact feeds the touchdown needs besides the time (one feed is no touchdown)
constexpr float kJumpOutHoldRiseMs = 0.5f;    // m/s: without air info a contact feed rising faster than this is no touchdown (take-off / apex)
// r3b landing (game log 2026-11-04 02:06:36, jump-out with the vehicle at 8 m/s; feed dump t 296.6-297.7): the PSM landing state that follows a very fast fall of the
// jump-out gives no locomotion update at all - the last air feed is 0.01 s above the floor, the next feed comes 0.65 s later in Stand. No Feed() runs through the
// landing: the gate keeps its last (air) value, the MM clock stalls, MMAIR stays in LandApproach and the pose is held until the state ends. Tick() therefore feeds the
// touchdown itself (a synthetic ground feed built from the last real one) once the predicted touchdown of a jump-out flight is overdue by kLandDropoutS; the first
// real feed ends the continuation. Synthetic feeds do not move the jump-out hold counters (the real feeds confirm the touchdown) and do not touch the exit window
// (g_exitLeft stays: a real VeryHardLand feed after the silence still finds the window for release:veryHardLand).
// A pause / menu / photo mode stops the feed just like the silent landing state does and a wall clock cannot tell them apart, so the silence is measured in GAME SIM
// time (the engine's sim clock does not advance in pause / menu / photo mode): the silent landing state is a real gap of ~0.65 s that the continuation covers whole
// (kLandDropoutMaxS = 1.2 s of sim time, the episode cap), a pause adds no sim time -> no synthetic feed, the episode does not progress and continues on the resume.
// While the sim clock is unavailable (the call failed) the continuation falls back to a short wall-clock bound (kLandDropoutWallMaxS) instead.
constexpr float kLandDropoutS = 0.06f;    // s past the predicted touchdown (AirState time to land at the last feed) without a feed: Tick() feeds the landing
constexpr float kLandDropoutMaxS = 1.20f; // s of SIM time after the last real feed at most (the episode cap)
constexpr float kLandDropoutWallMaxS = 0.25f; // fallback without a sim clock: s of wall time after the last real feed at most (a longer gap may be a pause)
bool g_landDropout = false;               // Tick() is feeding the landing for the missing locomotion updates
int64_t g_lastRealFeedTicks = 0;          // QPC ticks of the last non-synthetic Feed()
// sim-time silence since the last REAL feed (Tick(); valid while g_simClockUsable)
double g_simPrev = 0.0;                   // the sim sample the last silence step was taken against
bool g_simPrevValid = false;              // g_simPrev is a continuous sample (the previous Tick sampled too)
float g_simSilence = 0.0f;                // sim s since the last real feed (accumulated from sim deltas, reset when a real feed ran)
int64_t g_simSeenFeedTicks = 0;           // g_lastRealFeedTicks the silence was last reset for
bool g_simClockUsable = false;            // every sample since the last real feed succeeded: the sim silence is the criterion (never mixed with wall time in one episode)
std::atomic<bool> g_simWatch{false};      // Tick() samples the sim clock (only while a jump-out landing could be pending: g_jumpOutExit / g_landDropout)
std::atomic<SimClockFn> g_simClockHook{nullptr}; // SetSimClock(): test hook, nullptr = the native clock of the plugin build
int64_t g_synthGuardTicks = 0;            // g_lastRealFeedTicks at the moment Tick() decided to feed the landing (valid while g_synthGuardOn)
bool g_synthGuardOn = false;              // Tick() released the lock for one synthetic Feed(): it is dropped when a real feed ran in the gap
float g_ttlAtFeed = -1.0f;                // AirState time to land (s) of the last real feed, < 0 unknown
uint64_t g_landDropoutEpisodes = 0, g_landDropoutFeeds = 0;
float g_jumpOutHoldT = 0.0f;              // s since the window closed with g_jumpOutExit still held
float g_jumpOutGroundT = 0.0f;            // s of confirmed-grade ground contact in that time
int g_jumpOutGroundN = 0;                 // consecutive contact feeds in that time
bool g_jumpOutLimHandIn = false;          // HandInActive() of the previous publish (the limiter history restarts when a hand-in starts)
mm::PoseStepLimiter g_jumpOutLimiter;       // 2026-10-28: rotation step cap of the published pose through the first 0.4 s of a jump-out flight
uint64_t g_jumpOutSlewFrames = 0;         // publishes inside that window
float g_jumpOutSlewLeft = 0.0f;         // s of that window left
int64_t g_jumpOutSlewTicks = 0;         // QPC of the previous limited publish
uint64_t g_jumpOutRearms = 0;           // 2026-10-28 11-49-37: same-rig re-arms in the jump-out flight handed in from a new live capture
bool g_jumpOutRearmPromoted = false;    // that capture became the hand-in source on this feed: the re-arm hold ends after the publish
uint64_t g_jumpOutFills = 0;            // jump-out exits handed in from a live capture after the edge (no ride capture)
float g_exitLeft = 0.0f;                // s of the vehicle exit window left
// CAR-1 (vehicle batch): exit workspot latch + EXITDIAG (always on, at most kExitDiagMaxLines per exit, one line per change)
uint32_t g_exitLatchedPsm = 0;          // PSM bits latched at the vehicle-bit falling edge (kPsmWorkspot until it first clears)
bool g_maskUpperFromGame = false;       // last feed: UpperMaskDecision::upperFromGame (false = TPP exit window, upper body on MM)
const char* g_holdWhy = nullptr;        // last feed: HoldBlocker (nullptr = hold eligible)
constexpr float kExitDiagSeconds = 3.0f;
constexpr uint32_t kExitDiagMaxLines = 40;
float g_exitDiagT = -1.0f;              // s since the falling edge (< 0 = no exit diagnosis running)
uint32_t g_exitDiagLines = 0;           // EXITDIAG lines of the current / last exit (status exitDiag=)
char g_exitDiagKey[200] = {};           // last logged state (the line without its time stamp)
// vehicle batch W1 P2: mm/VehicleTransition.hpp wiring (only with the mount cut on; off = the previous behaviour)
bool g_vtMounted = false;               // BeginMount called for the current vehicle state (cleared at the exit edge / abort)
// review fix (vehdll MMDriver:3307): the vehicle state of this ride is handled - set at BeginMount, cleared only at the exit edge,
// when a MountingEvent expires without a ride, on a session reset and when the mount cut is switched off. An abort (scene, death,
// rig change) leaves it set: the mount transition never starts again mid-ride (the vehicle's own motion is no workspot jump);
// the capture is re-enabled from the next unblocked feed, so the exit is still handed in.
bool g_vtRide = false;
uint64_t g_vtRearmsKept = 0;            // review fix (vehdll MMDriver:2612): same-rig re-arms in a ride / exit kept (no abort, no fade reset)
bool g_vtWorldOk = false;               // BeginMount got a valid entity position (UpdateWorld only then)
bool g_vtPrevPosOk = false;             // last feed: capsulePos / playerYawDeg valid (haveAir)
float g_vtPrevPos[3] = {0.0f, 0.0f, 0.0f};
float g_vtPrevYaw = 0.0f;               // deg
bool g_vtExitPending = false;           // this feed: BeginExit returned a fresh capture (Fill once the hold keeps MM)
// first v2 search after a normal exit: BeginExit's capture copy and the feed whose hand-in was accepted (0 = none). MotionStepLocked offers it to
// MotionDriver (FeedInput::vehicleExitPose) only on that very feed, so a skipped step can never leak it into a later re-entry.
PoseSnapshot g_vtExitSeedSnap{};
uint64_t g_vtExitSeedFeed = 0;
uint64_t g_vtMounts = 0, g_vtExitsHanded = 0, g_vtExitsNoCapture = 0, g_vtAborts = 0;
int64_t g_vtMountTicks = 0; // BeginMount ticks of this ride (one counter line per ride)
bool g_vtRideLogged = true;
bool g_vtSeatLogged = true; // seat pose switch guard line (once per ride, after its window)
uint64_t g_vtBase[12] = {}; // vehtrans counters at the mount cut: the ride lines print deltas
// 2026-10-28 19-55-01 (diag c9): the mounted-TPP flag of the last vehicle feed (the edge feed already has it off), the no-capture
// Fill of a third-person exit (vehtrans::ExitWantsFill), counter snapshot at the mount cut (EXITDIAG end line: this ride's deltas)
bool g_vtLastRideTpp = false;
bool g_vtExitFillPending = false;
uint64_t g_vtExitsFilledNoCapture = 0;
uint64_t g_vtNoCapBase = 0, g_vtFillBase = 0;
vehtrans::CounterSnap g_vtExitBase{};
bool g_vtMountCar = false;          // MountEvent(aIsCar) hint for the next VtBeginMountLocked
int64_t g_vtMountCarTicks = 0;
constexpr float kVtMountCarHintSeconds = 2.0f;
constexpr float kVtMountLogSeconds = 1.2f; // 'vehtrans mount' line: after the longest standing entry epoch (<= 1.05 s)
BoneMapResult g_readMap;                // map from bone 0 (Root / Trajectory too): capture map of the frozen mount snapshot
// 0.6.2 hold last pose (default on; off = 0.6.1): in TPP the body stays on the MM pose except in states the game animates
bool g_holdPoseOn = true;               // UTPPSetMMHoldPose
bool g_holdActive = false;              // last feed / tick: a closure is held (weight kept)
bool g_poseValid = false;               // a pose was computed since the last matcher init (safe to republish)
bool g_holdSwap = false;                // TrySwapLocked under hold: keep the last published (held) pose, no weight-0 publish
int g_lastDetail = -1;                  // last feed: PSM.LocomotionDetailed (diagnostic field, -1 = not reported)
uint64_t g_holdFrames = 0, g_holdEpisodes = 0, g_holdSwaps = 0, g_holdTicks = 0;
// CLIMB RELEASE stage: Climb 8 / Vault 9 -> game animation unless v2 owns the traversal (UTPPSetMMClimbRelease; driver default
// on, the CET menu default is off and is pushed at load). TRAV-1: with v2 traversal active the setting is not consulted.
bool g_climbReleaseOn = true;
uint32_t g_releaseReasons = 0;          // last feed: kRelease* (HoldInput::release; the no-feed tick reuses it)
uint32_t g_releaseTracePrev = 0;        // release reasons of the last MMGATE decision (trace only)
uint64_t g_travReleaseFeeds = 0, g_travReleaseEpisodes = 0, g_travOwnedFeeds = 0;
OffsetRootConfig EffectiveOffsetCfgLocked();
void ApplyCatchupLocked();

std::vector<int32_t> g_rotEnd;    // per clip: local frame where the root turn is 90 % done (every turning clip; -1 = none)
int g_bindSign = 0;               // +1 left / -1 right while a start segment is bound (0 = none)
int32_t g_bindClip = -1, g_bindBlockClip = -1;
float g_bindTarget = 0.0f;        // world target when the binding started (stick reversal test)
float g_bindSince = 1e6f;         // s since the binding started (same-direction commit window)
int g_stSign = 0;                 // last body command: 0 off (pass-through), 1 clip (bound), 2 fallback (shortest way / 0.5.7)
uint64_t g_bindFeeds = 0, g_bindCommitSearches = 0, g_bindReversals = 0, g_binds = 0;

void ResetOffsetLocked()
{
    if (g_offsetRoot.Ready() && (g_offsetRoot.X() != 0.0f || g_offsetRoot.Y() != 0.0f || g_offsetRoot.Yaw() != 0.0f))
        g_offsetRoot.Reset();
}

int64_t Qpc() { return util::QpcTicks(); }
double QpcFreq() { return static_cast<double>(util::QpcFrequency()); }

// Game SIM time in seconds (GameInstance.GetSimTime): does not advance in pause / menu / photo mode. In redscript GetSimTime is a static native of the
// ScriptGameInstance class (not a global "...;GameInstance" function), EngineTime is {uint64 ticks} in QueryPerformanceFrequency units. Called from Tick() (main
// thread, every frame) outside g_mutex and only while a jump-out landing could be pending. False = unavailable (no game instance / function not found / failed).
bool NativeSimSeconds(double& aSeconds)
{
#if defined(RED4EXT_HEADER_ONLY)
    static RED4ext::CBaseFunction* s_fn = nullptr; // cached once found (a failed lookup is retried on the next call)
    RED4ext::ScriptGameInstance gameInstance;
    if (!gameInstance.instance)
        return false;
    if (!s_fn)
    {
        auto* rtti = RED4ext::CRTTISystem::Get();
        auto* cls = rtti ? rtti->GetClass("ScriptGameInstance") : nullptr;
        s_fn = cls ? cls->GetFunction("GetSimTime") : nullptr; // CClass::GetFunction searches the static functions first
        if (!s_fn)
            return false;
    }
    RED4ext::EngineTime t{};
    RED4ext::StackArgs_t args;
    args.emplace_back(nullptr, &gameInstance);
    if (!RED4ext::ExecuteFunction(static_cast<void*>(nullptr), s_fn, &t, args))
        return false;
    aSeconds = static_cast<double>(t.ticks) / QpcFreq();
    return std::isfinite(aSeconds);
#else
    (void)aSeconds;
    return false;
#endif
}

bool ReadSimClock(double& aSeconds)
{
    const SimClockFn hook = g_simClockHook.load(std::memory_order_acquire);
    const bool ok = hook ? hook(aSeconds) : NativeSimSeconds(aSeconds);
    return ok && std::isfinite(aSeconds);
}

uint8_t ClipClassOfLocked(uint32_t aClip)
{
    const std::string_view n = g_db.ClipName(aClip);
    const uint8_t k = steer::DriverClipClass(n, steer::ClassifyClip(n), g_crouchDirTagOn); // 0.5.4 / 0.5.6 / 0.5.7
    // 0.6.0: moving into-crouch transitions (RunToCrouch / WalkToCrouch) - same stance rules, plus the speed / stick rule
    return (k == steer::kClassStanceToCrouch && steer::IsMovingToCrouchName(n)) ? uint8_t(steer::kClassStanceMoveToCrouch) : k;
}

// 0.5.8 loop speed bands: per clip loop speed class / transition source class, loops per class in the stand / sprint set.
// Reads g_stateTagMask: loco P0 runs it again in RecomputeDerivedClassDataLocked once the masks of the database are computed
// (the matcher init that also calls it runs before them on a database load / switch).
void ComputeLoopBandsLocked()
{
    const auto clips = g_db.Clips();
    g_loopBandOf.assign(clips.size(), 0);
    for (int b = 0; b < 4; ++b)
        g_loopAvail[b] = 0;
    const uint64_t standSet = g_stateTagMask[kLocoStand] | g_stateTagMask[kLocoSprint];
    for (size_t c = 0; c < clips.size() && c < g_clipClasses.size(); ++c)
    {
        if (!(clips[c].tagMask & standSet))
            continue;
        const int b = steer::LoopBandOfName(g_db.ClipName(uint32_t(c)), g_clipClasses[c]);
        g_loopBandOf[c] = int8_t(b);
        if (b > 0 && g_clipClasses[c] == steer::kClassNone)
            ++g_loopAvail[b];
    }
}

// 0.5.7: per-clip root turn of the turning clips (turn classes, turn in place, starts): direction (total root yaw >= 30 deg)
// and, for the turn classes, the end frame (90 % of the total root yaw; turn commit). With UTPPSetMMStartTurn, start clips
// with a slow root turn (>= 45 deg, 10 -> 90 % taking > 0.8 s) become kClassSlowTurnStart. Then the matcher class sets:
// 0 = base, 1 / 2 = base with the clips turning against a left / right turn as kClassTurnAgainst.
void ComputeTurnProfilesLocked()
{
    const auto clips = g_db.Clips();
    g_turnEnd.assign(clips.size(), -1);
    g_turnSign.assign(clips.size(), 0);
    g_clipPeakRoot.assign(clips.size(), 0.0f); // 0.6.3 turn rate: plant = root below its clip's peak
    {
        const float fpsP = g_db.Fps() > 0.0f ? g_db.Fps() : 30.0f;
        for (size_t c = 0; c < clips.size() && c < g_clipClasses.size(); ++c)
        {
            if (!steer::IsTurnClass(g_clipClasses[c]))
                continue;
            float pk = 0.0f;
            for (uint32_t f = 0; f < clips[c].numFrames; ++f)
            {
                const auto* rd = g_db.Root(clips[c].firstFrame + f);
                if (rd)
                    pk = std::fmax(pk, std::sqrt(rd->dx * rd->dx + rd->dy * rd->dy) * fpsP);
            }
            g_clipPeakRoot[c] = std::isfinite(pk) ? pk : 0.0f;
        }
    }
    g_rotEnd.assign(clips.size(), -1);
    const float fps = g_db.Fps() > 0.0f ? g_db.Fps() : 30.0f;
    for (size_t c = 0; c < clips.size() && c < g_clipClasses.size(); ++c)
    {
        const uint8_t k = g_clipClasses[c];
        const bool start = steer::IsStart(k);
        if (!steer::IsTurnClass(k) && k != steer::kClassTurnInPlace && !start)
            continue;
        float total = 0.0f;
        for (uint32_t f = 0; f < clips[c].numFrames; ++f)
            total += g_db.Root(clips[c].firstFrame + f)->dyaw;
        if (!(std::fabs(total) >= 30.0f * kDegToRad))
            continue;
        g_turnSign[c] = total > 0.0f ? 1 : -1;
        float acc = 0.0f;
        int32_t f10 = -1, f90 = -1;
        for (uint32_t f = 0; f < clips[c].numFrames; ++f)
        {
            acc += g_db.Root(clips[c].firstFrame + f)->dyaw;
            if (f10 < 0 && std::fabs(acc) >= 0.1f * std::fabs(total))
                f10 = int32_t(f);
            if (std::fabs(acc) >= 0.9f * std::fabs(total))
            {
                f90 = int32_t(f);
                break;
            }
        }
        if (steer::IsTurnClass(k))
            g_turnEnd[c] = f90;
        g_rotEnd[c] = f90; // 0.5.8 start bind (every turning clip)
        if (g_startTurnOn && start && std::fabs(total) >= 45.0f * kDegToRad && f10 >= 0 && f90 >= 0 && float(f90 - f10) / fps > 0.8f)
            g_clipClasses[c] = steer::kClassSlowTurnStart;
    }
    for (int s = 0; s < 3; ++s)
        g_matchClasses[s] = g_clipClasses;
    for (size_t c = 0; c < g_clipClasses.size(); ++c)
    {
        if (g_turnSign[c] < 0)
            g_matchClasses[1][c] = steer::kClassTurnAgainst; // turns right while a left turn is wanted
        else if (g_turnSign[c] > 0)
            g_matchClasses[2][c] = steer::kClassTurnAgainst;
    }
    g_activeClassSet = 0;
    ComputeLoopBandsLocked(); // 0.5.8 (loco P0: also RecomputeDerivedClassDataLocked after the state tag masks)
    g_activeLoopBand = 0;
    g_classScratch.reserve(clips.size());
}

// 0.5.8: loop / transition of speed class b allowed in the bands [aLo, aHi] (capsule band .. query band: while the capsule
// accelerates towards the intent both bands; loops: the next slower class when the database has no loops in the range -
// a former test database has no sprint loops; transitions: source class in the range).
bool LoopBandAllowedLocked(size_t aClip, int aLo, int aHi)
{
    const int b = g_loopBandOf[aClip];
    if (b == 0 || (b >= aLo && b <= aHi))
        return true;
    if (g_clipClasses[aClip] == steer::kClassTransition)
        return false;
    int avail = 0;
    for (int k = aLo; k <= aHi && k <= 3; ++k)
        avail += g_loopAvail[k];
    return b == aLo - 1 && aLo >= 2 && avail == 0;
}

// The class buffer of class set aSet with the loop bands [aLo, aHi] (aLo 0 = unbanded: g_matchClasses[aSet] itself; banded:
// built in aScratch). Shared by ApplyClassSetLocked and the loco P0 test hook ProbeClassSet.
const std::vector<uint8_t>& ClassSetBufferLocked(int aSet, int aLo, int aHi, std::vector<uint8_t>& aScratch)
{
    if (!(aLo > 0 && g_loopBandOf.size() == g_matchClasses[aSet].size()))
        return g_matchClasses[aSet];
    aScratch.assign(g_matchClasses[aSet].begin(), g_matchClasses[aSet].end());
    for (size_t c = 0; c < aScratch.size(); ++c)
        if ((aScratch[c] == steer::kClassNone || aScratch[c] == steer::kClassTransition || aScratch[c] == steer::kClassLateral) &&
            !LoopBandAllowedLocked(c, aLo, std::max(aLo, aHi)))
            aScratch[c] = steer::kClassOffBand;
    return aScratch;
}

// 0.5.7 class set (0 base, 1 / 2 against a left / right turn) + 0.5.8 loop band (kClassOffBand for loops / transitions out of
// the band). Rebuilt only when either changes; SetClipClasses copies into the matcher's buffer (same size: no allocation).
void ApplyClassSetLocked(int aSet, int aLo, int aHi)
{
    const int aBand = aLo > 0 ? aLo + 4 * std::max(aLo, aHi) : 0;
    if (aSet == g_activeClassSet && aBand == g_activeLoopBand)
        return;
    const std::vector<uint8_t>* src = &ClassSetBufferLocked(aSet, aLo, aHi, g_classScratch);
    if (g_matcher.SetClipClasses(*src))
    {
        g_activeClassSet = aSet;
        g_activeLoopBand = aBand;
    }
}

// 0.5.8 start bind: after the matcher update - a playing start segment (start / reface / turn in place) whose root still turns
// (before 90 % of its root yaw) binds the body-turn direction to its root-yaw sign (from the root profile, not the name).
void ReleaseStartBindLocked()
{
    g_bindSign = 0;
    g_bindClip = -1;
    g_bindSince = 1e6f;
}
void UpdateStartBindLocked(bool aStick, bool aBodyTurn, bool aAnimLed, float aDt)
{
    const MatcherDiagnostics& d = g_matcher.Diagnostics();
    if (d.clip != g_bindBlockClip)
        g_bindBlockClip = -1;
    if (!g_startBindOn || !aStick || !aBodyTurn || !aAnimLed || g_followFed || g_gate != 0)
    {
        ReleaseStartBindLocked();
        return;
    }
    const int32_t pc = d.clip;
    const bool seg = pc >= 0 && size_t(pc) < g_clipClasses.size() && size_t(pc) < g_rotEnd.size() &&
                     steer::IsStartSegmentClass(g_clipClasses[size_t(pc)]) && g_turnSign[size_t(pc)] != 0 &&
                     g_rotEnd[size_t(pc)] >= 0 && d.clipFrame < g_rotEnd[size_t(pc)] && pc != g_bindBlockClip;
    if (!seg)
    {
        ReleaseStartBindLocked();
        return;
    }
    // stick reversal (target moved > kStartReverseDeg since the binding started): free the body and the search; the playing
    // clip is not bound again
    if (g_bindSign != 0 && std::fabs(WrapPi(g_targetYaw - g_bindTarget)) > steer::kStartReverseDeg * kDegToRad)
    {
        ++g_bindReversals;
        g_bindBlockClip = pc;
        ReleaseStartBindLocked();
        return;
    }
    const int sign = g_turnSign[size_t(pc)];
    if (g_bindSign != sign)
    {
        g_bindSince = 0.0f;
        g_bindTarget = g_targetYaw;
        ++g_binds;
    }
    else
        g_bindSince = std::fmin(g_bindSince + aDt, 1e6f);
    g_bindSign = sign;
    g_bindClip = pc;
    ++g_bindFeeds;
}

// 0.5.7: allowed frame ranges = every frame except the tails of the stance-transition clips (after kStanceStartFrames).
std::vector<Range> StanceStartRangesLocked()
{
    std::vector<Range> out;
    if (!g_stanceOnceOn)
        return out;
    const auto clips = g_db.Clips();
    uint32_t cursor = 0;
    bool any = false;
    for (size_t c = 0; c < clips.size() && c < g_clipClasses.size(); ++c)
    {
        if (!steer::IsStanceClass(g_clipClasses[c]) || clips[c].numFrames <= uint32_t(steer::kStanceStartFrames) + 1)
            continue;
        const uint32_t holeStart = clips[c].firstFrame + uint32_t(steer::kStanceStartFrames) + 1;
        if (holeStart > cursor)
            out.push_back({cursor, holeStart});
        cursor = clips[c].firstFrame + clips[c].numFrames;
        any = true;
    }
    if (!any)
        return {};
    if (cursor < g_db.NumFrames())
        out.push_back({cursor, g_db.NumFrames()});
    return out;
}

// 0.5.6 / 0.5.7: lateral clips are excluded for forward motion only in crouch and only where a non-lateral forward crouch
// loop / move exists (a database whose only "_Crouch_Loop_F" is a sideways loop keeps lateral clips allowed).
// Stand / sprint: not applied - "strafe" loops measured 6-13 deg between root motion and hip facing (forward-diagonal
// runs), the SideSteps (70 deg) carry L / R in {mdir} and do not match forward motion.
void ComputeLateralOkLocked()
{
    for (int st = 0; st < 4; ++st)
        g_lateralOk[st] = false;
    const auto clips = g_db.Clips();
    for (size_t i = 0; i < clips.size() && i < g_clipClasses.size(); ++i)
    {
        if (g_clipClasses[i] != steer::kClassNone)
            continue;
        const std::string_view n = g_db.ClipName(uint32_t(i));
        if ((clips[i].tagMask & g_stateTagMask[kLocoCrouch]) && (IsClassLoopName(n, "Crouch", true)))
            g_lateralOk[kLocoCrouch] = true;
    }
}

// ---- loco P0 (2026-10-13; default on, off = the 0.6.5b order): data derived from the per-state tag masks ----
// 0.6.5b built the 0.5.8 loop bands only inside the matcher init, which a database load / switch runs BEFORE
// ComputeStateTagMasksLocked: after the first load the masks were still 0 (nothing banded until UTPPSetMMEnabled re-ran the
// init), after a switch they held the previous database's tag bits (startup default -> v6: 35 vs 47 tags, indices shifted,
// the v6 walk loops never banded). The offline loco suite did not see it: its setters re-run ComputeTurnProfilesLocked
// after every load.
bool g_derivedRecomputeOn = true;         // UTPPSetMMDerivedRecompute
uint64_t g_derivedRecomputes = 0;         // recomputes run
uint64_t g_derivedStale = 0;              // recomputes that found stale data (loop bands; lateral rule: runtime paths only)
int g_derivedPrevAvail[4] = {0, 0, 0, 0}; // loop availability before the last recompute (log)

// Rebuilds every table that reads g_stateTagMask: the 0.5.8 loop bands + loop availability and the crouch lateral rule.
// Called right after ComputeStateTagMasksLocked (database load / switch, SetExcludeLands) and after the setters that change
// the clip classes (SetCrouchDirTag, SetStartTurn). The call sites are not gated: the P0 part below runs only while
// UTPPSetMMDerivedRecompute is on (off = nothing here = the 0.6.5b order). Changed matcher class set tables put the matcher
// back on the base class set (ApplyClassSetLocked rebuilds the banded set at the next search, no playback reset), unchanged
// data touches nothing. Returns true when the data was stale. aLoadSwitch (database load / switch): g_lateralOk still holds
// the previous database's rule there (CalibrateLocked rebuilds it right after), so only the loop bands count as stale; on the
// runtime paths (SetExcludeLands, the class setters, switching the recompute on) a changed lateral rule counts as well.
// Hook (strafe stage): tables outside the P0 A/B (strafe profiles: per-stance availability / loop own speeds) are rebuilt
// here after the P0 block, regardless of g_derivedRecomputeOn, and a change in a matcher class set table (set 3) joins
// classSetsChanged so the reset below covers it as well.
// B2 strafe S1: strafe profiles (per clip max |cumulative ROOT yaw|, strafe class sets, per stance x gait x |theta| loop own
// speeds, 8-sector coverage / availability per stance) from the clip classes and the state tag masks -> class set 3. Runs in
// RecomputeDerivedClassDataLocked (after the masks) and in the matcher init (masks may still be the previous database's there;
// the recompute right after fixes it). One allocation per call, never per frame.
void ComputeStrafeProfilesLocked()
{
    if (!g_db.Loaded() || g_clipClasses.size() != g_db.NumClips())
    {
        g_strafeProf = strafe::StrafeProfiles{};
        g_matchClasses[3] = g_clipClasses;
        return;
    }
    strafe::ComputeStrafeProfiles(g_db, std::span<const uint8_t>(g_clipClasses), g_stateTagMask[kLocoStand], g_stateTagMask[kLocoCrouch],
                                  g_strafeCfg, g_strafeProf);
    const bool jog = g_strafeJogF > 0.0f && g_strafeProf.availGait[strafe::kStanceStand][strafe::kGaitRun];
    g_matchClasses[3] = g_strafeProf.classes[jog ? strafe::kGaitRun : strafe::kGaitWalk];
}

// B2 strafe S2: ChestAlign init (Hips / Spine) + the stand / crouch references (sign-aligned mean Spine model rotation over one
// loop period of the motion forward walk / crouch loop) of the loaded database. Called from RecomputeDerivedClassDataLocked on a
// database load / switch (and while not ready): the references never mix two databases. A missing clip leaves that stance
// without a reference (Apply is a no-op there, logged once by the leg pipeline). Init time only (one allocation per call).
void BuildChestReferencesLocked()
{
    g_chestNoRefLogged = false;
    g_chestOut = false;
    g_chestValid = false;
    if (!g_db.Loaded() || !g_chestAlign.Init(g_db, g_chestCfg))
        return;
    g_chestPose.assign(g_db.NumBones(), BoneTransform{});
    static constexpr const char* kRefClip[ChestAlign::kNumStances] = {"M_Neutral_Walk_Loop_F", "M_Neutral_Crouch_Loop_F"};
    for (int s = 0; s < ChestAlign::kNumStances; ++s)
    {
        const int32_t c = ChestAlign::FindClip(g_db, kRefClip[s]);
        if (c < 0)
            continue;
        const ClipRecord& cr = g_db.Clips()[size_t(c)];
        const uint32_t n = (cr.loopPeriod > 0 && cr.loopPeriod <= cr.numFrames) ? cr.loopPeriod : cr.numFrames;
        g_chestAlign.BuildReference(g_db, s, cr.firstFrame, n);
    }
}

void LogStrafeProfilesLocked(const char* aWhy)
{
    if (!g_log)
        return;
    const strafe::StrafeProfiles& p = g_strafeProf;
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "UTPP MM strafe profiles (%s): avail stand=%d crouch=%d (sectors walk %d/8 run %d/8 crouch %d/8), %u facing-fixed loops, "
                  "%u root turners, %u turn-class clips, patterns kept walk %u / run %u (off band %u / %u), speeds walk %.2f/%.2f/%.2f/%.2f/%.2f "
                  "run %.2f/%.2f/%.2f/%.2f/%.2f crouch %.2f/%.2f/%.2f/%.2f/%.2f%s",
                  aWhy, p.avail[0] ? 1 : 0, p.avail[1] ? 1 : 0, strafe::SectorCount(p.sectors[0][0]), strafe::SectorCount(p.sectors[0][1]),
                  strafe::SectorCount(p.sectors[1][0]), p.facingFixedLoops, p.rootTurners, p.turnClassClips, p.patternsKept[0],
                  p.patternsKept[1], p.patternsOffBand[0], p.patternsOffBand[1], static_cast<double>(p.table.speed[0][0][0]),
                  static_cast<double>(p.table.speed[0][0][1]), static_cast<double>(p.table.speed[0][0][2]),
                  static_cast<double>(p.table.speed[0][0][3]), static_cast<double>(p.table.speed[0][0][4]),
                  static_cast<double>(p.table.speed[0][1][0]), static_cast<double>(p.table.speed[0][1][1]),
                  static_cast<double>(p.table.speed[0][1][2]), static_cast<double>(p.table.speed[0][1][3]),
                  static_cast<double>(p.table.speed[0][1][4]), static_cast<double>(p.table.speed[1][0][0]),
                  static_cast<double>(p.table.speed[1][0][1]), static_cast<double>(p.table.speed[1][0][2]),
                  static_cast<double>(p.table.speed[1][0][3]), static_cast<double>(p.table.speed[1][0][4]),
                  (!p.avail[0] || !p.avail[1]) ? " - a stance without facing-fixed loops in >= 6 of 8 directions keeps the previous strafe path while aiming"
                                               : "");
    g_log(buf);
}

bool RecomputeDerivedClassDataLocked(bool aLoadSwitch)
{
    if (!g_db.Loaded() || g_clipClasses.size() != g_db.NumClips())
        return false;
    bool bands = false, stale = false;
    if (g_derivedRecomputeOn)
    {
        const std::vector<int8_t> prevBandOf = g_loopBandOf;
        bool prevLateral[4];
        for (int i = 0; i < 4; ++i)
        {
            g_derivedPrevAvail[i] = g_loopAvail[i];
            prevLateral[i] = g_lateralOk[i];
        }
        ComputeLoopBandsLocked();
        ComputeLateralOkLocked();
        ++g_derivedRecomputes;
        bands = prevBandOf != g_loopBandOf;
        bool lateral = false;
        for (int i = 0; i < 4; ++i)
        {
            bands = bands || g_derivedPrevAvail[i] != g_loopAvail[i];
            lateral = lateral || prevLateral[i] != g_lateralOk[i];
        }
        stale = bands || (lateral && !aLoadSwitch);
        if (stale)
            ++g_derivedStale;
    }
    // B2 strafe S1 (outside the P0 A/B): strafe profiles / class set 3; a changed set 3 resets the matcher class set only while
    // set 3 is the active one (strafe off: set 3 is never active - nothing here touches the matcher)
    bool strafeSetChanged = false;
    {
        const std::vector<uint8_t> prev3 = g_matchClasses[3];
        ComputeStrafeProfilesLocked();
        strafeSetChanged = prev3 != g_matchClasses[3];
        if (!g_strafeLogged || strafeSetChanged)
        {
            LogStrafeProfilesLocked(aLoadSwitch ? "database load" : "recompute");
            g_strafeLogged = true;
        }
    }
    // B2 strafe S2: chest references per database (pose data only; the masks / class setters do not change them)
    if (aLoadSwitch || !g_chestAlign.Ready())
        BuildChestReferencesLocked();
    const bool classSetsChanged = bands || (strafeSetChanged && g_activeClassSet == 3);
    if (classSetsChanged)
    {
        g_activeClassSet = 0;
        g_activeLoopBand = 0;
        if (g_matcher.Ready())
            g_matcher.SetClipClasses(g_matchClasses[0]);
    }
    return stale;
}

void LogDerivedLocked(const std::string& aDb, bool aChanged)
{
    if (!g_log)
        return;
    int banded = 0;
    for (const int8_t b : g_loopBandOf)
        banded += b > 0 ? 1 : 0;
    char buf[320];
    std::snprintf(buf, sizeof buf,
                  "UTPP MM class data [%s]: recomputed after the state tag masks: loopAvail w%d/r%d/s%d, %d banded clips, crouch "
                  "lateral rule %d (%s; before w%d/r%d/s%d)",
                  aDb.c_str(), g_loopAvail[1], g_loopAvail[2], g_loopAvail[3], banded, g_lateralOk[kLocoCrouch] ? 1 : 0,
                  aChanged ? "was stale" : "unchanged", g_derivedPrevAvail[1], g_derivedPrevAvail[2], g_derivedPrevAvail[3]);
    g_log(buf);
}

// loco C1a: one info line per load / switch while the exclusion is on, and per setter call
void LogCrouchExclLocked(const std::string& aDb, const char* aWhy)
{
    if (!g_log)
        return;
    char buf[1024];
    if (!g_crouchExclOn)
        std::snprintf(buf, sizeof buf, "UTPP MM crouch idle exclusion [%s] (%s): off - full crouch set, %d crouch idle loop(s)", aDb.c_str(),
                      aWhy, g_crouchIdleLoops > 0 ? g_crouchIdleLoops : CrouchIdleLoopsLocked(g_stateTagMask[kLocoCrouch]));
    else if (g_crouchExclSkipped)
        std::snprintf(buf, sizeof buf,
                      "UTPP MM crouch idle exclusion [%s] (%s): skipped - no looping crouch idle would remain without tags \"%s\" (%d "
                      "clip(s) would go); full crouch set kept, %d crouch idle loop(s)",
                      aDb.c_str(), aWhy, g_crouchExclTags.c_str(), g_crouchExclClips, g_crouchIdleLoops);
    else if (g_crouchExclMask == 0)
        std::snprintf(buf, sizeof buf,
                      "UTPP MM crouch idle exclusion [%s] (%s): no listed tag in the crouch set (tags \"%s\": %d found, %d missing) - "
                      "nothing excluded, %d crouch idle loop(s)",
                      aDb.c_str(), aWhy, g_crouchExclTags.c_str(), g_crouchExclFound, g_crouchExclMissing, g_crouchIdleLoops);
    else
        std::snprintf(buf, sizeof buf,
                      "UTPP MM crouch idle exclusion [%s] (%s): on - tags \"%s\" (%d found, %d missing), %d clip(s) out of the crouch "
                      "set, %d crouch idle loop(s) left",
                      aDb.c_str(), aWhy, g_crouchExclTags.c_str(), g_crouchExclFound, g_crouchExclMissing, g_crouchExclClips,
                      g_crouchIdleLoops);
    g_log(buf);
}

// loco C0: per-foot fragment of the "MMTRACE feet" line and the status segment:
// "L=<phase> cL=<contact in> offL=<lock offset>cm corrL=<correction>cm wL=<IK weight> slipsL= driftL= stepsL= R=..."
// phase: P planted (locked), S replant step this update (loco C1b, also its last update), R releasing (blending out), F free
void FeetFragmentLocked(char* aBuf, size_t aSize)
{
    const FootLockDiagnostics& fd = g_footLock.Diagnostics();
    const auto phase = [&](int s)
    { return (fd.foot[s].stepping || fd.foot[s].stepped) ? 'S' : (fd.foot[s].locked ? 'P' : (fd.foot[s].weight > 0.0f ? 'R' : 'F')); };
    std::snprintf(aBuf, aSize,
                  "L=%c cL=%.2f offL=%.1fcm corrL=%.1fcm wL=%.2f slipsL=%llu driftL=%llu stepsL=%llu R=%c cR=%.2f offR=%.1fcm "
                  "corrR=%.1fcm wR=%.2f slipsR=%llu driftR=%llu stepsR=%llu",
                  phase(0), static_cast<double>(fd.foot[0].contactIn), static_cast<double>(fd.foot[0].offset) * 100.0,
                  static_cast<double>(fd.foot[0].correction) * 100.0, static_cast<double>(fd.foot[0].weight),
                  static_cast<unsigned long long>(fd.footSlips[0]), static_cast<unsigned long long>(fd.footDriftUnlocks[0]),
                  static_cast<unsigned long long>(fd.footSteps[0]), phase(1), static_cast<double>(fd.foot[1].contactIn),
                  static_cast<double>(fd.foot[1].offset) * 100.0, static_cast<double>(fd.foot[1].correction) * 100.0,
                  static_cast<double>(fd.foot[1].weight), static_cast<unsigned long long>(fd.footSlips[1]),
                  static_cast<unsigned long long>(fd.footDriftUnlocks[1]), static_cast<unsigned long long>(fd.footSteps[1]));
}

// loco C0: "UTPP MMTRACE feet" (after every MMTRACE TRANS / tick line; separate line, the main format is unchanged)
void TraceFeetLocked()
{
    if (!g_log)
        return;
    const FootLockDiagnostics& fd = g_footLock.Diagnostics();
    char feet[512];
    FeetFragmentLocked(feet, sizeof feet);
    char line[768];
    const int n = std::snprintf(line, sizeof line,
                                "UTPP MMTRACE feet %s locks=%llu relocks=%llu clamps=%llu settles=%llu events=%llu fl=%d crouchExcl=%d", feet,
                                static_cast<unsigned long long>(fd.locks), static_cast<unsigned long long>(fd.relocks),
                                static_cast<unsigned long long>(fd.clamps), static_cast<unsigned long long>(fd.settles),
                                static_cast<unsigned long long>(g_feetEvents), g_footLock.Ready() && g_footLockOn ? 1 : 0,
                                g_crouchExclMask != 0 ? 1 : 0);
    // loco C1b: appended only while replant is on for some stance (mask 0 = the loco C1a line)
    if (g_footReplantMask != 0 && n > 0 && static_cast<size_t>(n) < sizeof line)
        std::snprintf(line + n, sizeof line - static_cast<size_t>(n), " replant=%d active=%d liftL=%.1fcm liftR=%.1fcm",
                      g_footReplantMask, g_footLock.ReplantOn() ? 1 : 0, static_cast<double>(fd.foot[0].lift) * 100.0,
                      static_cast<double>(fd.foot[1].lift) * 100.0);
    g_log(line);
}

// loco C0: new foot events -> one "UTPP MMTRACE footstep side=L|R reason=drift|radius|clamp|tidy dist=<cm>cm clip= f= loco= |v|="
// line each while aEmit (trace + diagnostics on); events are always marked as handled (no backlog when the trace is switched on).
void FeetEventsLocked(bool aEmit, int32_t aLoco)
{
    const FootLockDiagnostics& fd = g_footLock.Diagnostics();
    for (int s = 0; s < 2; ++s)
    {
        const FootLockEvent& e = fd.lastEvent[s];
        if (e.seq == 0 || e.seq <= g_feetEventSeen[s])
            continue;
        g_feetEventSeen[s] = e.seq;
        ++g_feetEvents;
        if (!aEmit || !g_log)
            continue;
        const MatcherDiagnostics* d = g_matcher.Ready() ? &g_matcher.Diagnostics() : nullptr;
        const std::string_view clip = (d && d->clip >= 0) ? g_db.ClipName(static_cast<uint32_t>(d->clip)) : std::string_view{};
        char line[400];
        std::snprintf(line, sizeof line, "UTPP MMTRACE footstep side=%c reason=%s dist=%.1fcm clip=%.*s f=%d loco=%d |v|=%.2f",
                      s == 0 ? 'L' : 'R', FootEventReasonName(e.reason), static_cast<double>(e.dist) * 100.0,
                      static_cast<int>(clip.size()), clip.data(), d ? d->clipFrame : -1, aLoco, static_cast<double>(g_rawSpeed));
        g_log(line);
    }
}

bool InitMatcherLocked()
{
    g_poseValid = false; // 0.6.2: foot lock / offset root outputs are reset - nothing to republish until the next matcher update
    g_air.RequestReset(); // AIR HOLD stage: database load / swap / matcher re-init ends an air episode (Reset on the next feed)
    g_cfg = MatcherConfig{};
    g_cfg.allowedTagMask = g_tagMask;
    g_cfg.transitionCost = g_transitionCost;
    g_cfg.stopHalflife = g_stopSyncOn ? g_stopHalflife : 0.0f;
    g_cfg.minDwell = g_dwellOn ? g_minDwell : 0.0f;
    g_cfg.dwellCostRatio = g_dwellRatio;
    g_rootRate.Reset();
    g_turnRateW = 0.0f; // 0.6.3
    g_turnRateActive = false;
    g_turnRecentSpeed = 0.0f;
    g_turnAcc = TurnRateAcc{}; // 0.6.4
    g_steer.Reset();
    g_slowGate.Reset();
    g_band.Reset(); // 0.5.4
    g_appliedTagMask = g_tagMask;
    g_stanceUsed = false; // 0.5.7
    g_cfg.costCap = g_costCap; // 0.5.8
    g_cfg.costCapHoldMax = g_costCapHold;
    g_loopBandTracker.Reset();
    g_loopBandHiTracker.Reset();
    g_lastLoopBand = g_lastLoopBandHi = 0;
    g_settleLeft = 0.0f;
    g_prevCommitted = false;
    g_idleHoldStill = 0.0f;
    ReleaseStartBindLocked();
    if (g_db.Loaded())
    {
        g_clipClasses.assign(g_db.NumClips(), 0);
        for (uint32_t c = 0; c < g_db.NumClips(); ++c)
            g_clipClasses[c] = ClipClassOfLocked(c);
        g_cfg.allowedFrameRanges = StanceStartRangesLocked(); // 0.5.7
        ComputeTurnProfilesLocked();                          // 0.5.7
        ComputeStrafeProfilesLocked();                        // B2 strafe (rebuilt after the state tag masks by the recompute)
    }
    g_strafeLogged = false;
    g_strafeMode.Reset();
    g_aimJogMotionHeld = false; // UTPP-MOTION SEAM
    g_strafeRate.Reset();
    const bool ok = g_db.Loaded() && g_matcher.Init(g_db, g_cfg);
    if (ok)
        g_matcher.SetClipClasses(g_matchClasses[0]);
    g_airTableApplied = false; // Init cleared the matcher's root_z / loop override
    if (ok)
        ApplyAirTableLocked(); // AIR CLIPS stage: root_z out of the Hips, Jump_Loop_Fall as a loop (clips-capable setting only)
    g_airPlayMeta = -1;
    g_airPlayLocal = -1;
    g_airLastCleanFrame = -1;
    g_airLandWasApproach = false;
    g_airLandGroundBlendLeft = 0.0f;
    g_airLandGroundBlendPending = false;
    g_airLandGroundBlendOutputValid = false;
    g_airLandGroundBones = 0;
    g_airLandGroundElapsed = 0.0f;
    g_airLandGroundStartPending = false;
    g_airLandGroundFirstHipsError = 0.0f;
    g_airHoldGroundReplantLeft = 0.0f;
    g_stanceAirBypassed = false;
    g_legPipelineOutput = {};
    // Foot lock is optional: a failed init (missing leg bones) leaves the matcher output unchanged.
    if (ok && g_footLock.Init(g_db))
    {
        g_footLock.SetEnabled(g_footLockOn);
        g_footLock.SetSlip(g_catchupOn ? g_slipStart : 0.0f, g_slipHalflife); // 0.5.9
        g_footLock.SetReplantConfig(g_replantCfg);                            // loco C1b (mode per feed: ApplyLegPipelineLocked)
    }
    if (ok)
        g_offsetRoot.Init(g_db, g_cfg.firstBodyBone, EffectiveOffsetCfgLocked()); // optional: failure = 0.3.2 pose path
    if (ok)
        g_stanceGuard.Init(g_db, g_stanceCfg); // optional (0.3.6): failure = 0.3.5 pose path
    return ok;
}

// 0.5.9: offset root config in use (catch-up adds the soft limit / convergence and the larger hard clamp)
OffsetRootConfig EffectiveOffsetCfgLocked()
{
    OffsetRootConfig c = g_offsetCfg;
    if (g_catchupOn)
    {
        c.catchup = true;
        c.maxDistance = std::fmax(c.maxDistance, g_catchMaxDistance);
        c.softDistance = std::fmin(g_catchSoft, c.maxDistance);
        c.catchupTime = g_catchTime;
        c.catchupHalflife = g_catchHalflife;
    }
    return c;
}

void ApplyCatchupLocked()
{
    if (g_offsetRoot.Ready())
        g_offsetRoot.SetConfig(EffectiveOffsetCfgLocked());
    if (g_footLock.Ready())
        g_footLock.SetSlip(g_catchupOn ? g_slipStart : 0.0f, g_slipHalflife);
}

// ---- loco leg pipeline (2026-10-13): the per-feed leg post-processing of the matcher pose in one place ----
// Order agreed for the loco batch (crouch / strafe / air designs): OffsetRoot (in Feed) -> StanceGuard -> [overlay: ALS Feminine,
// motion v2] -> [ChestAlign: strafe S2] -> [FootPlacement::ApplyPelvis: crouch C2] -> FootLocker (+ placement dz: crouch C2)
// -> publish. The published pose is chosen in ONE place (PipelineOutputLocked); a new stage adds its line there.
// LegContext is computed once per feed right AFTER g_matcher.Update (contacts and clip class of the frame just played; the gate /
// hold fields are final before the matcher block). The FootLocker is gated only by airborne / contactScale (air area), never by
// hold: the 0.6.2 hold states (unarmed aim walk, dodge, slide, menu / stale feed) keep their foot locks. placementAllowed is the
// separate gate of the ground foot placement (crouch C2).
struct LegContext
{
    bool airborne = false;           // air area (AirState decision of this feed, AIR HOLD stage): FootLocker contacts -> 0
    float contactScale = 1.0f;       // air area: landing ramp 0 -> 1 of the FootLocker contacts (1 with the air area off / grounded)
    bool placementAllowed = false;   // gate open, no hold, |vz| < the debounce's maxVertical, stand / crouch / sprint (crouch C2)
    int32_t stance = 0;              // conditioned locomotion state (1 stand, 2 crouch, 3 sprint; 0 other)
    float contact[2] = {0.0f, 0.0f}; // CONT of the played frame (FootLocker::SampleContact, left / right)
    int32_t clip = -1;               // playing clip after the matcher update (-1 none)
    uint8_t clipClass = 0;           // its driver class (g_clipClasses)
    bool stops = false;              // clip class idle / stop (replant Stops profile, loco C1b)
    float capsuleSpeed = 0.0f;       // m/s, planar capsule speed (after the 0.5.8 query clamp)
    float vz = 0.0f;                 // m/s, capsule vertical velocity
    bool v2 = false; // UTPP-MOTION SEAM the frame is v2-owned (MotionLegContextLocked); always false with v2 off
    int32_t cancelStartSupport = -1;
};
LegContext g_legCtx; // last feed (status / tests)

LegContext ComputeLegContextLocked(const FeedInput& aIn, float aCapsuleSpeed)
{
    LegContext c;
    const MatcherDiagnostics& md = g_matcher.Diagnostics();
    c.stance = aIn.locoState;
    c.contact[0] = FootLocker::SampleContact(g_db, md.frame, md.alpha, FootLocker::kLeft);
    c.contact[1] = FootLocker::SampleContact(g_db, md.frame, md.alpha, FootLocker::kRight);
    c.clip = md.clip;
    c.clipClass = (md.clip >= 0 && size_t(md.clip) < g_clipClasses.size()) ? g_clipClasses[size_t(md.clip)] : uint8_t(0);
    c.stops = c.clip >= 0 && (c.clipClass == steer::kClassIdle || c.clipClass == steer::kClassStop);
    c.capsuleSpeed = aCapsuleSpeed;
    c.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
    // A v2->v1 handback resets AirState on the next feed. That reset can transiently report Ground/contactScale=1
    // despite a fresh physics probe still saying airborne: never plant feet or run ground pelvis placement there.
    const bool physicsAir = g_airSetting != 3 && aIn.haveAir && !aIn.onGround;
    c.placementAllowed = !physicsAir && g_effGate == 0 && !g_holdActive && std::fabs(c.vz) < g_gateDebounceCfg.maxVertical &&
                         aIn.locoState >= kLocoStand && aIn.locoState <= kLocoSprint;
    // AIR HOLD stage: from the air decision of this feed (UpdateAirLocked ran before the matcher block): no plants in the air,
    // contacts ramp back after the touchdown. Air area off / grounded: false / 1 (the sampled contacts exactly).
    c.airborne = physicsAir || (g_airOwns && g_airDec.contactScale <= 0.0f);
    c.contactScale = physicsAir ? 0.0f : g_airDec.contactScale;
    return c;
}

// UTPP-MOTION SEAM commit A (6.10.1 item 3, 6.10.3 step 6): when v2 owns a feed its pose takes the matcher's place in the same tail
// UTPP-MOTION SEAM (OffsetRoot in Feed -> ApplyLegPipelineLocked -> PipelineOutputLocked). The v1 stage instances serve it: the v2
// UTPP-MOTION SEAM database has the v1 topology (rig hash kExpectedRigHash 0x72CB314EB01F562D, 73 bones; 6.10.5) and the frame hook
// UTPP-MOTION SEAM refuses any bone whose game bone differs from the v1 map. The v1 matcher search does not run; its trajectory is
// UTPP-MOTION SEAM kept current (MotionKeepBaseTrajectoryLocked) so the handback starts from the present motion.
MotionSetFrame g_motionFrame;        // UTPP-MOTION SEAM
std::array<BoneTransform, integ::kMaxMMBones> g_motionPoseBuf{}; // UTPP-MOTION SEAM caller-owned copy of the v2 pose (MotionSetFrame buf)
bool g_motionStepped = false; // UTPP-MOTION SEAM step already called this feed (inside the matcher block)
bool g_motionHeldMatcher = false; // UTPP-MOTION SEAM v2 owned a feed since the last v1 matcher update (MotionHandbackLocked)
int g_motionStartTurnDirection = 0;
uint64_t g_motionHandbacks = 0; // UTPP-MOTION SEAM fresh v1 searches after a v2-owned span (ProbeMotionSeam)
// UTPP-MOTION SEAM x AIR (merge repair): the AirState reset requested by MotionHandbackLocked is emitted on the next feed; its query bone
// UTPP-MOTION SEAM frame (AirState's last clean frame, taken from the frozen v1 frame before the span) is not latched there.
bool g_motionAirResetPending = false; // UTPP-MOTION SEAM
int32_t g_motionProbeQueryFrame = -1; // UTPP-MOTION SEAM last latched air query bone frame (ProbeMotionSeam only)
bool GroundBrakingDomain(const FeedInput& f)
{
    return f.tppActive && f.locoState >= kLocoStand && f.locoState <= kLocoSprint &&
        (!f.haveAir || f.onGround) && !PsmIsAirDetail(PsmDetailOf(f.psmBlock));
}
FeedInput WithAirGravityLocked(const FeedInput& aRaw)
{
    FeedInput f = aRaw;
    f.predictedGravityUp = g_air.GUp();
    f.predictedGravityDown = g_air.GDown();
    const float decelAge = g_decelSampleTicks ?
        static_cast<float>(static_cast<double>(Qpc() - g_decelSampleTicks) / QpcFreq()) : -1.0f;
    // FeedLocomotion precedes UpdateDecel in REDscript. A fresh sample from the
    // previous airborne/stance/detail domain must not drive the first landing
    // or stance-change prediction; wait one query for the new native stat.
    const bool sameDomain = GroundBrakingDomain(aRaw) && aRaw.locoState == g_decelSampleLoco &&
        PsmDetailOf(aRaw.psmBlock) == g_decelSampleDetail;
    f.capsuleBrakingDeceleration = sameDomain ?
        brake::FreshDeceleration(g_decelBase, decelAge, kStaleFeedSeconds) : 0.0f;
    const float accelAge = g_accelSampleTicks ?
        static_cast<float>(static_cast<double>(Qpc() - g_accelSampleTicks) / QpcFreq()) : -1.0f;
    const bool sameAccelDomain = GroundBrakingDomain(aRaw) && aRaw.locoState == g_accelSampleLoco &&
        PsmDetailOf(aRaw.psmBlock) == g_accelSampleDetail;
    f.capsuleAcceleration = accel::FreshAcceleration(g_accelObserved, accelAge, kStaleFeedSeconds, sameAccelDomain);
    return f;
}
const MotionSetFrame* MotionStepLocked(const FeedInput& aRaw, float aDt, float capX, float capY, float capYaw) // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    g_motionStepped = true; // UTPP-MOTION SEAM
    g_motionStartTurnDirection = 0;
    auto raw = WithAirGravityLocked(aRaw);
    raw.haveRootContext = true;
    raw.nativeRootEnabled = g_offsetRootOn && g_offsetRoot.Ready();
    raw.nativeBodyTurnEnabled = g_bodyTurnOn && g_bodyTurnFed;
    raw.nativeBodyTurnSwitch = g_bodyTurnOn;
    // mutation 23: v2 sees the closed exit window only (the jump-out state does not reach the chooser after a false contact)
    raw.vehicleExitWindow = g_exitLeft > 0.0f || (UTPP_AND_NOT(VEHEXIT, 23) g_jumpOutExit); // r3 landing: the jump-out state outlives the closed window until the touchdown
    raw.vehicleJumpOut = g_jumpOutExit;
    raw.vehicleExitPose = g_vtExitSeedFeed != 0 && g_vtExitSeedFeed == g_feedCalls ? &g_vtExitSeedSnap : nullptr;
    raw.rootFeedSequence = g_feedCalls;
    raw.capsuleDeltaCurrent[0] = capX * aDt;
    raw.capsuleDeltaCurrent[1] = capY * aDt;
    raw.capsuleYawDelta = capYaw;
    if (!g_motion || !g_motion->step || !g_motion->step(raw, g_effGate, aDt)) // UTPP-MOTION SEAM
        return nullptr; // UTPP-MOTION SEAM
    // UTPP-MOTION SEAM no leg stage ready -> PipelineOutputLocked would be the v1 matcher pose: v2 publishes its raw pose (apply)
    const bool stage = g_footLock.Ready() || (g_stanceOn && g_stanceGuard.Ready()) || (g_offsetRootOn && g_offsetRoot.Ready()); // UTPP-MOTION SEAM
    g_motionFrame = MotionSetFrame{}; // UTPP-MOTION SEAM
    g_motionFrame.buf = g_motionPoseBuf.data(); // UTPP-MOTION SEAM
    g_motionFrame.bufCap = static_cast<uint32_t>(g_motionPoseBuf.size()); // UTPP-MOTION SEAM
    if (!stage || !g_motion->frame || !g_map.ok || !g_motion->frame(g_map.gameIndex, g_db.NumBones(), g_motionFrame) || // UTPP-MOTION SEAM
        g_motionFrame.pose != g_motionPoseBuf.data() || g_motionFrame.numBones != g_db.NumBones()) // UTPP-MOTION SEAM pose must be our copy
        return nullptr; // UTPP-MOTION SEAM
    if (g_motionFrame.providedRoot && g_motionFrame.rootFeedSequence != g_feedCalls)
        return nullptr;
    g_motionStartTurnDirection = g_motionFrame.travFrame ? 0 : g_motionFrame.startTurnDirection;
    return &g_motionFrame; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM
void MotionKeepBaseTrajectoryLocked(float aDt, const MatcherInput& aIn) // UTPP-MOTION SEAM: BaseMatcher::Update step 1 only
{ // UTPP-MOTION SEAM
    const float dt = std::min((std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f, g_matcher.Config().maxDt); // UTPP-MOTION SEAM
    TrajectoryInput ti; // UTPP-MOTION SEAM
    ti.desiredVelocity[0] = aIn.desiredVelocity[0]; // UTPP-MOTION SEAM
    ti.desiredVelocity[1] = aIn.desiredVelocity[1]; // UTPP-MOTION SEAM
    ti.facingMode = aIn.facingMode; // UTPP-MOTION SEAM
    ti.desiredFacing[0] = aIn.desiredFacing[0]; // UTPP-MOTION SEAM
    ti.desiredFacing[1] = aIn.desiredFacing[1]; // UTPP-MOTION SEAM
    ti.yawRate = aIn.yawRate; // UTPP-MOTION SEAM
    ti.externalYaw = aIn.externalYaw; // UTPP-MOTION SEAM
    ti.externalYawStep = aIn.externalYawStep; // UTPP-MOTION SEAM
    ti.externalYawRate = aIn.externalYawRate; // UTPP-MOTION SEAM
    ti.turnRate = aIn.turnRate; // UTPP-MOTION SEAM
    ti.clearAngVel = aIn.clearAngVel; // UTPP-MOTION SEAM
    g_matcher.GetTrajectory().Update(dt, ti); // UTPP-MOTION SEAM
    g_motionHeldMatcher = true; // UTPP-MOTION SEAM the v1 search / playback / inertializer / dwell stand still from here
} // UTPP-MOTION SEAM
// UTPP-MOTION SEAM review major 1: first v1 matcher update after a v2-owned span (yield, toggle off, body switch, fault): the matcher
// UTPP-MOTION SEAM stood still there (clip, frame, search timer, inertialization offset, dwell), so it searches afresh for the present
// UTPP-MOTION SEAM trajectory (BaseMatcher::ForceFreshSearch: no current frame, no stale offset) before its first published frame.
void MotionHandbackLocked() // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    g_motionStartTurnDirection = 0;
    if (!g_motionHeldMatcher) // UTPP-MOTION SEAM
        return; // UTPP-MOTION SEAM
    g_motionHeldMatcher = false; // UTPP-MOTION SEAM
    g_airHoldGroundReplantLeft = 0.0f; // never carry a v1 landing-step policy across v2 ownership
    g_airLandGroundBlendLeft = 0.0f; // never blend a cached v1 Land pose across an independently owned v2 span
    g_airLandGroundBlendPending = false;
    g_airLandGroundBlendOutputValid = false;
    g_airLandGroundElapsed = 0.0f;
    g_airLandGroundStartPending = false;
    g_matcher.ForceFreshSearch(); // UTPP-MOTION SEAM
    g_air.RequestReset(); // UTPP-MOTION SEAM x AIR (merge): an air episode running over the v2 span ends (Action::Reset next feed)
    g_motionAirResetPending = true; // UTPP-MOTION SEAM x AIR (merge repair)
    ++g_motionHandbacks; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM
// UTPP-MOTION SEAM review major 2: the steering helpers read the v1 matcher diagnostics, frozen while v2 owns the feed: no start bind
// UTPP-MOTION SEAM and no long-way start turn there (BodyYawCommand takes the ordinary target; the v1 search they steer does not run).
void MotionReleaseSteeringLocked() // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    ReleaseStartBindLocked(); // UTPP-MOTION SEAM
    g_bodyFollowDir = 0; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM
// UTPP-MOTION SEAM merge rework (b4): the v2 leg context is the v1 one (ComputeLegContextLocked: conditioned stance, capsule speed,
// UTPP-MOTION SEAM vz, placement gate, air fields airborne / contactScale and any later field) with the frame-owned fields taken
// UTPP-MOTION SEAM from the v2 frame: contacts, stops, actual clip class, clip = -1 (v1 clip does not apply), v2 = true (ChestAlign
// UTPP-MOTION SEAM off on v2 frames). Air on v2-owned frames: AirState still runs on every feed (UpdateAirLocked, before the matcher
// UTPP-MOTION SEAM block), so a v2 frame in the air gets airborne / contactScale 0 and the landing ramp like a v1 frame; only the v1 air
// UTPP-MOTION SEAM clip playback (ForcePlay, search suppression, query bone override) is skipped on the frozen v1 matcher. On the v2
// UTPP-MOTION SEAM handback the AirState is reset (MotionHandbackLocked -> RequestReset; Action::Reset on the next feed).
LegContext MotionLegContextLocked(const MotionSetFrame& aF, const FeedInput& aIn, float aCapsuleSpeed) // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    LegContext c = ComputeLegContextLocked(aIn, aCapsuleSpeed); // UTPP-MOTION SEAM
    c.contact[0] = aF.contact[0]; // UTPP-MOTION SEAM
    c.contact[1] = aF.contact[1]; // UTPP-MOTION SEAM
    c.clip = -1; // UTPP-MOTION SEAM v1 clip / class do not apply
    c.clipClass = aF.clipClass; // actual v2 class, never the frozen v1 clip
    c.stops = aF.stops; // UTPP-MOTION SEAM
    c.v2 = true; // UTPP-MOTION SEAM
    c.cancelStartSupport = aF.cancelStartSupport;
    return c; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM

// The leg stages after the offset root (aPose = offset-root output, or the matcher output with the offset root off). The result
// is published through PipelineOutputLocked.
void ApplyLegPipelineLocked(std::span<const BoneTransform> aPose, const LegContext& aCtx, float aDt, const FeedInput& aIn, float aYaw)
{
    std::span<const BoneTransform> pose = aPose;
    g_legPipelineOutput = {};
    // StanceGuard modifies thigh/shin rotations even when both contacts are zero, including
    // its legacy no-contact path. AIR frames must retain the supplied animation legs.
    // Reset its temporal filters on the airborne edge, then resume from a clean state at touchdown.
    const bool stanceAirBypass = g_airSetting != 3 && aCtx.airborne;
    g_stanceAirBypassed = stanceAirBypass;
    if (stanceAirBypass && g_stanceGuard.FilterActive())
        g_stanceGuard.Reset();
    // 0.3.6 stance-width guard (retargeted motion walk ankles pass on the midline; legs interpenetrate on the CP mesh):
    // before the foot locker, so locks are placed under the corrected feet.
    if (g_stanceOn && g_stanceGuard.Ready() && !stanceAirBypass)
    {
        g_stanceGuard.SetSpeed(aCtx.capsuleSpeed); // 0.3.9 swivel speed fade
        // batch 2 knee fix bit 1: no knee swivel in crouch (off = the 0.3.9 path)
        g_stanceGuard.SetSwivelSuppressed((g_kneeFixMask & kKneeFixCrouchSwivel) != 0 && aCtx.stance == kLocoCrouch);
        g_stanceGuard.SetPlantedHold((g_kneeFixMask & kKneeFixPlantedHold) != 0); // batch 5 knee fix bit 4
        if (g_stanceSwingOn) // 0.3.7: swing foot only, push held while planted
            g_stanceGuard.Apply(pose, aCtx.airborne ? 0.0f : aCtx.contact[0] * aCtx.contactScale,
                                aCtx.airborne ? 0.0f : aCtx.contact[1] * aCtx.contactScale, aDt);
        else
            g_stanceGuard.Apply(pose);
        pose = g_stanceGuard.Output();
    }
    // [overlay insertion point (ALS Feminine overlay, motion v2): feet and IK below act on the overlaid pose]
    // B2 strafe S2 ChestAlign (after the overlay point, which also changes the spine): only while the strafe mode is Active /
    // Exiting and UTPPSetMMChestAlign is on; otherwise nothing here runs (the S1 pose path). Only the Spine local rotation changes.
    g_chestValid = false;
    g_chestOut = false;
    if (g_chestOn && g_chestAlign.Ready() && g_strafeMode.On() && pose.size() == g_chestPose.size() && // UTPP-MOTION SEAM (line wrapped)
        !aCtx.v2) // UTPP-MOTION SEAM no chest align on a v2-owned frame (strafe is not mapped to v2)
    {
        // teleport (offset root reset to the capsule): the low-passed chest twist belongs to the old pose stream
        const uint64_t tp = g_offsetRoot.Diagnostics().teleports;
        if (tp != g_chestTeleports)
        {
            g_chestTeleports = tp;
            g_chestAlign.Reset();
        }
        const int stance = g_strafeStance == strafe::kStanceCrouch ? ChestAlign::kCrouch : ChestAlign::kStand;
        // weight = strafe blend (1 while the mode is Active / Exiting) x g_maskBlend.Weight(). ONE-FRAME LAG: g_maskBlend is
        // advanced in the Feed tail after this pipeline, so this is the PREVIOUS feed's mask blend (the first frame of a mask
        // fade-in applies 0, the first frame of a fade-out still the previous weight); harmless for the smooth mask fades.
        const float w = g_maskBlend.Weight();
        std::copy(pose.begin(), pose.end(), g_chestPose.begin());
        g_chestAlign.Apply(std::span<BoneTransform>(g_chestPose), stance, w, aDt);
        pose = g_chestPose;
        g_chestOut = true;
        g_chestValid = g_chestAlign.Diagnostics().valid;
        if (!g_chestAlign.HasReference(stance) && !g_chestNoRefLogged && g_log)
        {
            g_chestNoRefLogged = true;
            char buf[256];
            std::snprintf(buf, sizeof buf,
                          "UTPP MM chest align: this database has no %s reference clip (%s) - chest align is a no-op there (logged once "
                          "per database; references stand=%d crouch=%d)",
                          stance == ChestAlign::kCrouch ? "crouch" : "stand",
                          stance == ChestAlign::kCrouch ? "M_Neutral_Crouch_Loop_F" : "M_Neutral_Walk_Loop_F",
                          g_chestAlign.HasReference(ChestAlign::kStand) ? 1 : 0, g_chestAlign.HasReference(ChestAlign::kCrouch) ? 1 : 0);
            g_log(buf);
        }
    }
    // [FootPlacement::ApplyPelvis (crouch C2)]
    // Foot lock in REAL time with the measured capsule motion (planted feet stay fixed on the ground the capsule actually moves
    // over, independent of the animation-time rate). Contacts x contactScale, 0 while airborne (air area); exactly the sampled
    // contacts while contactScale is 1.
    if (g_footLock.Ready())
    {
        // loco C1b: replant mode for the stance of this feed (mask bit 1 stand / sprint, bit 2 crouch), Stops profile on idle / stop.
        // The per-feed mode selection is shared with the offline replay (mm/FootLocker.hpp ApplyFootFeedPolicy, tests/FeedReplayMotion.cpp).
        const bool airHoldGroundReplant = g_airSetting != 3 && !aCtx.v2 && !aCtx.airborne && aCtx.contactScale > 0.0f &&
                                          g_airHoldGroundReplantLeft > 0.0f &&
                                          (aCtx.stance == kLocoStand || aCtx.stance == kLocoSprint);
        const float k = aCtx.airborne ? 0.0f : aCtx.contactScale;
        const float cl = k == 1.0f ? aCtx.contact[0] : aCtx.contact[0] * k;
        const float cr = k == 1.0f ? aCtx.contact[1] : aCtx.contact[1] * k;
        const int64_t t1 = Qpc();
        const bool noMove = std::isfinite(aIn.move[0]) && std::isfinite(aIn.move[1]) &&
            aIn.move[0] * aIn.move[0] + aIn.move[1] * aIn.move[1] <= 0.15f * 0.15f;
        FootFeedPolicy pol;
        pol.stance = aCtx.stance;
        pol.stops = aCtx.stops;
        pol.v2 = aCtx.v2;
        pol.airborne = aCtx.airborne;
        pol.contactScale = aCtx.contactScale;
        pol.replantMask = g_footReplantMask;
        pol.airHoldGroundReplant = airHoldGroundReplant;
        pol.poleCarry = (g_kneeFixMask & kKneeFixPoleCarry) != 0; // batch 2 knee fix bit 2
        pol.cancelStartSupport = aCtx.cancelStartSupport;
        // A database named Idles may also contain turn-in-place. Only actual
        // stop -> plain-idle classes may retain this support episode.
        pol.standingStopGate = g_effGate == 0 && !g_holdActive && aIn.tppActive && noMove;
        pol.classStop = aCtx.clipClass == steer::kClassStop;
        pol.classIdle = aCtx.clipClass == steer::kClassIdle;
        ApplyFootFeedPolicy(g_footLock, pol);
        // 2026-11-06 start slip: a start clip may let the planted toe drift further (g_slipStartStart) before the soft lock slides
        // (measured offline: planted toe speed of the Start classes -13..-31 % on both bodies, tests/FeedReplayMotion.cpp --footlock).
        // Off (0, or catch-up off / not above the base slip) = the single slip of SetSlip, bit for bit.
        // Only on ground locomotion (gate open, not airborne, stand / crouch / sprint; not the vz / hold part of placementAllowed, which may flicker): anywhere else (vehicle, montage, air, hold) the base slip
        // returns at once, so a foot released into a game pose (mount, exit) does not keep a longer lock offset.
        if (g_catchupOn && g_slipStart > 0.0f && g_slipStartStart > g_slipStart && g_effGate == 0 && !aCtx.airborne &&
            aCtx.stance >= kLocoStand && aCtx.stance <= kLocoSprint)
        {
            FootSlipProfile sp;
            sp.active = true;
            sp.base = g_slipStart;
            sp.start = g_slipStartStart;
            sp.turn = g_slipStart;
            sp.halflife = g_slipHalflife;
            ApplyFootSlipProfile(g_footLock, sp, aCtx.clipClass);
            g_slipProfileOn = true;
        }
        else if (g_slipProfileOn)
        {
            g_footLock.SetSlip(g_catchupOn ? g_slipStart : 0.0f, g_slipHalflife);
            g_slipProfileOn = false;
        }
        g_footLock.Update(aDt, std::isfinite(aIn.velWorld[0]) ? aIn.velWorld[0] : 0.0f,
                          std::isfinite(aIn.velWorld[1]) ? aIn.velWorld[1] : 0.0f, aYaw, cl, cr, pose);
        g_lastFootLockUs = static_cast<double>(Qpc() - t1) * 1.0e6 / QpcFreq();
        pose = g_footLock.Output();
    }
    g_legPipelineOutput = pose;
}

// Crossfade the pose before StanceGuard / FootLocker so both feet are solved against the final Hips pose.
// Hips and spine share the blend fraction to avoid a hard torso kink, while root translation and yaw remain
// authoritative from the new matcher. The source is refreshed from the actual published Recover pose every feed.
std::span<const BoneTransform> BlendAirLandToGroundLocked(std::span<const BoneTransform> aGround, float aDt)
{
    g_airLandGroundBlendOutputValid = false;
    if (g_airLandGroundBlendLeft <= 0.0f || g_airSetting == 3 ||
        aGround.size() != g_airLandGroundBones || g_airLandGroundBones > integ::kMaxMMBones)
        return aGround;

    const float t = std::clamp(g_airLandGroundElapsed / kAirLandGroundBlendSeconds, 0.0f, 1.0f);
    const float w = t * t * (3.0f - 2.0f * t);
    std::copy(aGround.begin(), aGround.end(), g_airLandGroundOutput.begin());
    for (uint32_t i = g_cfg.firstBodyBone; i < g_airLandGroundBones; ++i)
    {
        // Do not touch the trajectory/root bone or its yaw. All mapped body bones use one blend time,
        // preserving the chain through Hips -> Spine rather than bending only the hip and legs.
        const BoneTransform& source = g_airLandGroundPose[i];
        BoneTransform& dst = g_airLandGroundOutput[i];
        dst.p = source.p + (dst.p - source.p) * w;
        dst.q = math::QuatNlerpShortest(source.q, dst.q, w);
    }
    if (g_airLandGroundStartPending && g_cfg.firstBodyBone < g_airLandGroundBones)
    {
        g_airLandGroundFirstHipsError = std::fabs(g_airLandGroundOutput[g_cfg.firstBodyBone].p.z - g_airLandGroundStartHipsZ);
        g_airLandGroundStartPending = false;
    }
    g_airLandGroundBlendOutputValid = true;
    g_airLandGroundElapsed = std::min(kAirLandGroundBlendSeconds, g_airLandGroundElapsed + std::clamp(aDt, 0.0f, 0.05f));
    g_airLandGroundBlendLeft = std::max(0.0f, kAirLandGroundBlendSeconds - g_airLandGroundElapsed);
    return {g_airLandGroundOutput.data(), g_airLandGroundBones};
}

// The pose PublishLocked writes: the output of the last leg-pipeline stage that runs (the only selector of the published pose).
std::span<const BoneTransform> PipelineOutputLocked()
{
    if (!g_legPipelineOutput.empty())
        return g_legPipelineOutput;
    return g_footLock.Ready()                         ? g_footLock.Output()
           : g_chestOut                               ? std::span<const BoneTransform>(g_chestPose) // B2 strafe S2
           : (g_stanceOn && g_stanceGuard.Ready() && !g_stanceAirBypassed) ? g_stanceGuard.Output()
           : (g_offsetRootOn && g_offsetRoot.Ready()) ? g_offsetRoot.Output()
                                                      : std::span<const BoneTransform>(g_matcher.Output().bones);
}

void PublishLocked(float aWeight)
{
    // 0.6.2: right after a matcher init (enable, database switch) the foot lock / offset root outputs are reset (identity bones):
    // a visible publish waits for the next matcher update, the last published (held) snapshot stays in place meanwhile
    if (aWeight > 0.0f && !g_poseValid)
        return;
    g_snap.weight = aWeight;
    g_snap.numBones = g_dbLoaded ? g_db.NumBones() : 0;
    g_snap.firstBone = g_cfg.firstBodyBone;
    g_snap.rig = g_rig;
    g_snap.publishTicks = Qpc();
    g_snap.sequence = ++g_publishes;
    // Restore v1's canonical mask on every publish: v2 may replace the mask in this reused snapshot
    // (e.g. arms-only Reload). Its previous frame must not change later Aim or v2 -> v1 ownership.
    std::memcpy(g_snap.upperBits, g_upperMask.bits, sizeof g_snap.upperBits);
    // 0.6.2: upper-body bones at weight x (1 - mask)
    g_snap.upperScale = g_upperMask.ok ? 1.0f - g_maskBlend.Weight() : 1.0f;
    // melee full body: the other bones at weight x (1 - full body); melee aim tilt weight (mm/MeleeAim.hpp, FK detour)
    g_snap.lowerScale = g_upperMask.ok ? 1.0f - g_meleeFullBlend.Weight() : 1.0f;
    g_snap.meleeAimWeight = g_meleeAimBlend.Weight();
    // chest base correction: only for the fists flag (the script sets it for w_melee_fists without arm cyberware) and only standing / walking
    // on the ground (the table holds the standing guard family; sprint / crouch / slide / air clips are not in it)
    {
        g_snap.meleeBase = (g_meleeFullFlag && g_meleeFullGate.StandingGrounded()) ? 1u : 0u; // explicit predicate, whichever reason fails first
    }
    for (uint32_t b = 0; b < kMaxMMBones; ++b)
        g_snap.gameIndex[b] = g_map.ok ? g_map.gameIndex[b] : -1;
    if (g_matcher.Ready())
    {
        // Phase 5.3: the foot locker already ran on the offset-root pose (see Feed); loco leg pipeline: one selector.
        const std::span<const BoneTransform> bones = PipelineOutputLocked();
        const uint32_t n = static_cast<uint32_t>(bones.size()) < kMaxMMBones ? static_cast<uint32_t>(bones.size()) : kMaxMMBones;
        for (uint32_t b = 0; b < n; ++b)
        {
            float* d = &g_snap.pose[size_t(b) * 7];
            d[0] = bones[b].p.x;
            d[1] = bones[b].p.y;
            d[2] = bones[b].p.z;
            d[3] = bones[b].q.x;
            d[4] = bones[b].q.y;
            d[5] = bones[b].q.z;
            d[6] = bones[b].q.w;
        }
    }
    else
    {
        g_snap.weight = 0.0f;
    }
    g_snap.bodyType = -1; // reused snapshot must not retain a prior body/bundle identity
    g_snap.overlayArms = 0; // UTPP-MOTION SEAM motionfix item 6: v1 frames never claim both hands on the weapon
    if (g_motion && g_motion->apply) // UTPP-MOTION SEAM
        g_motion->apply(g_snap); // UTPP-MOTION SEAM: replaces pose + bone map only while v2 runs; weight / fade stay v1's
    // vehicle batch W1 P2 (MOTO-6 / CAR-1 step 5): exit hand-in from the captured game exit pose (no-op without BeginExit)
    const bool handInNow = vehtrans::HandInActive();
    const bool handInStart = handInNow && !g_jumpOutLimHandIn;
    g_jumpOutLimHandIn = handInNow;
    if (vehtrans::HandInActive())
        vehtrans::HandInBlend(g_snap, g_snap.publishTicks);
    UTPP_IF_NOT_MUT(VEHEXIT, 13) // mutation 13: no rotation step cap in the jump-out window
    {
        // bike jump-out, first 0.4 s: no bone turns faster than mm::kJumpOutSlewDegPerS (the MM start clip is played from its ground push-off while
        // V is already airborne: leg kicks of 15-31 deg per frame; PoseStepLimit.hpp). The cap follows the final published pose (MM + hand-in).
        // the window, then a tail while a bone still lags (no release of a lag in one frame); the hand-in start restarts the history: the
        // visible pose jumps to the game pose source there (live capture: the publishes before it carried the plain MM pose), the cap must
        // follow that pose and its inertialization, not pull it back towards the older MM pose
        const bool slew = (g_jumpOutSlewLeft > 0.0f || (g_jumpOutSlewLeft > -mm::kJumpOutSlewTailSeconds && g_jumpOutLimiter.LagRad() > 0.01f)) &&
                          g_weight.Weight() > 0.0f;
        if (UTPP_AND_NOT(VEHEXIT, 14) handInStart) // mutation 14: the hand-in start does not restart the limiter history
            g_jumpOutLimiter.Restart();
        const int64_t t = g_snap.publishTicks;
        const float dtS = g_jumpOutSlewTicks > 0 ? vehtrans::detail::Seconds(t - g_jumpOutSlewTicks, vehtrans::TicksPerSecond()) : 1.0f / 60.0f;
        g_jumpOutSlewTicks = slew ? t : 0;
        if (slew)
            ++g_jumpOutSlewFrames;
        g_jumpOutLimiter.Apply(g_snap.pose, g_snap.firstBone, g_snap.numBones, dtS, mm::kJumpOutSlewDegPerS, slew);
    }
    g_mmPose.Publish(g_snap);
    g_hot.mmActive.store(g_snap.weight > 0.0f, std::memory_order_release);
    if (handInNow && g_snap.weight > 0.0f)
        vehtrans::MarkExitHandInPublished(); // exit latch: the held capture gives way to MM on the next allowed animation frame
    if (g_jumpOutRearmPromoted && handInNow)
    {
        g_jumpOutRearmPromoted = false;
        vehtrans::EndRearmHold(); // the animation threads take MM again: this snapshot carries the hand-in from the captured game pose
    }
}

GateInput BaseGateLocked()
{
    GateInput g;
    g.mmEnabled = g_enabled;
    g.dbLoaded = g_dbLoaded;
    g.mapped = g_map.ok && g_rig != 0 && g_hot.armed.load(std::memory_order_acquire);
    g.allowedLocoMask = g_allowedMask;
    g.matcherReady = g_matcher.Ready();
    g.fault = g_hot.fault.load(std::memory_order_acquire);
    return g;
}

// 0.6.2: an MM pose can be shown (database, matcher, bone map, armed rig, no fault, a pose computed since the last init)
bool PoseAvailableLocked()
{
    return g_dbLoaded && g_matcher.Ready() && g_map.ok && g_rig != 0 && g_hot.armed.load(std::memory_order_acquire) &&
           !g_hot.fault.load(std::memory_order_acquire) && g_poseValid;
}

// Body-yaw command allowed: kill switch on, MM enabled + ready, gate open, weight > 0.5, the last feed used the body-turn
// input (valid Locomotion target) and is fresh.
bool BodyTurnActiveLocked(int64_t aNow)
{
    const float sinceFeed = g_lastFeedTicks ? static_cast<float>(static_cast<double>(aNow - g_lastFeedTicks) / QpcFreq()) : 1e9f;
    // AIR STEER stage: in the air the jump closes g_gate and the hold pose keeps the body (g_effGate 0) - the air steer term
    // takes the effective gate; off (g_airSteerFed never set) = the previous condition
    // v2 can own a frame while the v1 gate carries a covered upper-body reason. Its successful Step requires
    // g_effGate == 0; keep the body-yaw command available to the executor on those frames.
    // MotionReleaseSteeringLocked clears the v1 clip-derived turn bind before this command can run.
    const bool ground = g_bodyTurnFed && (g_motionHeldMatcher ? g_effGate == 0 : g_gate == 0);
    return g_bodyTurnOn && g_enabled && g_dbLoaded && g_matcher.Ready() &&
           (ground || (g_airSteerFed && g_effGate == 0)) &&
           g_weight.Weight() > 0.5f && g_haveTarget &&
           sinceFeed <= kStaleFeedSeconds;
}

// ---- 0.5.1 runtime database switching ----
// Lock order: g_mutex (game thread state) -> g_sw.m (worker hand-off). The worker never takes g_mutex.
std::wstring g_dbDir;               // <dll dir> with trailing separator
std::string g_dbName = "none";
std::string g_dbSha;
std::vector<uint64_t> g_gameHashes; // armed rig (kept for re-mapping after a database swap)
std::vector<int16_t> g_gameParents;
bool g_haveGameRig = false;
uint64_t g_dbSwitches = 0, g_dbSwitchFailures = 0;

struct SwitchState
{
    std::mutex m;
    std::thread worker;
    bool busy = false;
    std::unique_ptr<Database> ready; // validated candidate waiting for the fade-out
    std::string name, sha, status = "idle";
    bool haveSidecar = false; // 0.5.2: sidecar text read by the worker (no file I/O on the game thread)
    std::string sidecar;
    bool haveAirz = false; // AIR CLIPS stage: <db>.airz bytes read by the worker
    std::vector<uint8_t> airz;
    // Static destruction at DLL unload must not std::terminate on a joinable thread (Shutdown() joins it normally;
    // joining under the loader lock could deadlock, so a leftover worker is detached).
    ~SwitchState()
    {
        if (worker.joinable())
            worker.detach();
    }
};
SwitchState g_sw;

// ---- 0.5.2 per-database speed calibration (mm/SpeedCalibration.hpp) ----
SpeedCalibration g_cal;
FamilyRates g_famRates; // 0.5.3

// <db>.utppmm -> <db>.json
std::wstring SidecarPathFor(const std::wstring& aDbPath)
{
    const size_t dot = aDbPath.find_last_of(L'.');
    const size_t slash = aDbPath.find_last_of(L"\\/");
    const std::wstring stem = (dot != std::wstring::npos && (slash == std::wstring::npos || dot > slash)) ? aDbPath.substr(0, dot) : aDbPath;
    return stem + L".json";
}

// Sidecar text (small file); false when absent / unreadable / > 64 KiB.
bool ReadSidecar(const std::wstring& aDbPath, std::string& aText)
{
    aText.clear();
    FILE* f = nullptr;
    if (_wfopen_s(&f, SidecarPathFor(aDbPath).c_str(), L"rb") != 0 || !f)
        return false;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0 && aText.size() < (64u << 10))
        aText.append(buf, n);
    std::fclose(f);
    return aText.size() < (64u << 10);
}

// Derives the calibration of g_db (state tag masks computed) + optional sidecar and installs the speed maps.
void CalibrateLocked(bool aHaveSidecar, const std::string& aSidecar)
{
    g_cal = DeriveSpeedCalibration(g_db, g_stateTagMask[kLocoStand], g_stateTagMask[kLocoCrouch]);
    if (aHaveSidecar)
        ApplySpeedSidecar(g_cal, aSidecar);
    g_speedMap[kLocoStand] = g_cal.stand;
    g_speedMap[kLocoSprint] = g_cal.stand;
    g_speedMap[kLocoCrouch] = g_cal.crouch;
    // 0.5.3: per-clip rate factors (source families / own loop speed) and the slow-walk query speed
    g_famRates = ComputeFamilyRates(g_db, g_stateTagMask[kLocoStand], g_stateTagMask[kLocoCrouch], g_cal);
    g_slowRef[kLocoStand] = g_slowRef[kLocoSprint] = SlowestForwardLoop(g_cal, kSpeedWalk);
    g_slowRef[kLocoCrouch] = SlowestForwardLoop(g_cal, kSpeedCrouch);
    g_rateFactor = g_rateFactorTarget = 1.0f;
    // 0.5.4: speed-band edges (database m/s) and pivot / turn availability per speed class in the stand clip set
    g_bandEdges = ComputeBandEdges(g_cal);
    {
        std::vector<uint64_t> tags(g_db.NumClips(), 0);
        const auto clips = g_db.Clips();
        for (size_t i = 0; i < clips.size() && i < tags.size(); ++i)
            tags[i] = clips[i].tagMask;
        g_bandAvail = ComputeBandAvailability(g_clipClasses, tags, g_stateTagMask[kLocoStand]);
        ComputeLateralOkLocked(); // 0.5.6 / 0.5.7
    }
    g_band.Reset();
    g_capProfile = DeriveCapsuleProfile(g_db, g_stateTagMask[kLocoStand], g_stateTagMask[kLocoCrouch], g_cal); // 0.5.9
    if (g_log)
    {
        const std::string line = "UTPP MM speed calibration [" + g_dbName + "]: " + g_cal.Summary();
        g_log(line.c_str());
        const std::string cl = "UTPP MM capsule profile [" + g_dbName + "]: " + g_capProfile.Summary();
        g_log(cl.c_str());
        char buf[128];
        std::snprintf(buf, sizeof buf, "UTPP MM slow-walk query %.3f / crouch %.3f; source families: ",
                      double(g_slowRef[kLocoStand]), double(g_slowRef[kLocoCrouch]));
        const std::string fam = buf + g_famRates.Summary();
        g_log(fam.c_str());
        std::snprintf(buf, sizeof buf, "UTPP MM speed bands: edges walk/run %.2f run/sprint %.2f (database m/s); ",
                      double(g_bandEdges.walkRun), double(g_bandEdges.runSprint));
        const std::string bands = buf + g_bandAvail.Summary();
        g_log(bands.c_str());
        if (!g_bandEdges.sprintData) // 0.5.4: report only (data to be added); sprint plays run loops at the rate cap
        {
            const std::string warn = "UTPP MM WARNING [" + g_dbName +
                                     "]: no sprint loop clips - sprint uses run loops (speed fallback, playback rate capped: foot sliding "
                                     "expected); sprint pivots/turns fall back to run where missing";
            g_log(warn.c_str());
        }
    }
}

std::wstring Widen(const std::string& aUtf8)
{
    if (aUtf8.empty())
        return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, aUtf8.c_str(), -1, nullptr, 0);
    std::wstring w(static_cast<size_t>(n > 0 ? n : 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, aUtf8.c_str(), -1, w.data(), n);
    if (!w.empty() && w.back() == L'\0')
        w.pop_back();
    return w;
}

std::string Narrow(const std::wstring& aWide)
{
    char buf[1024] = {};
    WideCharToMultiByte(CP_UTF8, 0, aWide.c_str(), -1, buf, static_cast<int>(sizeof(buf) - 1), nullptr, nullptr);
    return buf;
}

bool ReadWholeFile(const std::wstring& aPath, std::vector<uint8_t>& aBytes, std::string& aErr)
{
    FILE* f = nullptr;
    if (_wfopen_s(&f, aPath.c_str(), L"rb") != 0 || !f)
    {
        aErr = "file not found / unreadable";
        return false;
    }
    bool ok = std::fseek(f, 0, SEEK_END) == 0;
    const long size = ok ? std::ftell(f) : -1;
    ok = ok && size > 0 && size < (512l << 20) && std::fseek(f, 0, SEEK_SET) == 0;
    if (ok)
    {
        aBytes.resize(static_cast<size_t>(size));
        ok = std::fread(aBytes.data(), 1, aBytes.size(), f) == aBytes.size();
    }
    std::fclose(f);
    if (!ok)
        aErr = "read failed / bad size";
    return ok;
}

using util::Sha256Hex; // hex SHA-256 (Windows CNG); "" on failure

// ---- AIR CLIPS stage: <db>.utppmm -> <db>.airz (motion-derived data) ----
std::wstring AirzPathFor(const std::wstring& aDbPath)
{
    const size_t dot = aDbPath.find_last_of(L'.');
    const size_t slash = aDbPath.find_last_of(L"\\/");
    const std::wstring stem = (dot != std::wstring::npos && (slash == std::wstring::npos || dot > slash)) ? aDbPath.substr(0, dot) : aDbPath;
    return stem + L".airz";
}

// Sidecar bytes; false when absent / unreadable / > 16 MiB.
bool ReadAirz(const std::wstring& aDbPath, std::vector<uint8_t>& aBytes)
{
    std::string err;
    aBytes.clear();
    if (!ReadWholeFile(AirzPathFor(aDbPath), aBytes, err) || aBytes.size() > (16u << 20))
    {
        aBytes.clear();
        return false;
    }
    return true;
}

// Parses the table for g_db (call before InitMatcherLocked). Status reason: table / noTable / badTable(<why>).
void ParseAirTableLocked(bool aHave, const std::vector<uint8_t>& aBytes, const std::string& aShaHex, const std::string& aName)
{
    g_airTable.Reset();
    g_airTableValid = false;
    g_airLegChainN = 0;
    if (!aHave)
    {
        g_airTableReason = "noTable";
        return;
    }
    uint8_t sha[32] = {};
    bool shaOk = aShaHex.size() == 64;
    for (size_t i = 0; shaOk && i < 32; ++i)
    {
        auto nib = [](char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1); };
        const int hi = nib(aShaHex[2 * i]), lo = nib(aShaHex[2 * i + 1]);
        shaOk = hi >= 0 && lo >= 0;
        sha[i] = static_cast<uint8_t>((hi << 4) | (lo & 15));
    }
    std::string err;
    if (!shaOk)
        err = "no db sha256";
    else if (!g_airTable.Parse(aBytes, g_db, std::span<const uint8_t>(sha, 32), err))
        err = err.empty() ? "parse failed" : err;
    else if (g_airTable.HipsBone() != static_cast<int32_t>(MatcherConfig{}.firstBodyBone))
        err = "Hips is not the first body bone";
    else if (g_airTable.Pick(airtab::Kind::Start, airtab::Dir::F, airtab::Band::Run, airtab::Weight::None, airtab::Foot::None).clip < 0 ||
             g_airTable.Pick(airtab::Kind::Land, airtab::Dir::F, airtab::Band::Run, airtab::Weight::Light, airtab::Foot::None).clip < 0 ||
             g_airTable.CountStatus(airtab::ClipStatus::Applied) == 0)
        err = "no usable start / land clips";
    if (!err.empty())
    {
        for (char& c : err)
            if (c == ' ')
                c = '-';
        g_airTableReason = "badTable(" + err + ")";
        g_airTable.Reset();
    }
    else
    {
        g_airTableValid = true;
        g_airTableReason = "table";
        g_airLegChainN = airtab::FindLegChain(g_db, g_airLegChain);
    }
    if (g_log)
    {
        char line[320];
        std::snprintf(line, sizeof line,
                      "UTPP MM air table [%s]: %s clips %u applied %u clean %u rejected %u metas %zu fallLoop %d loopOk %d (%.3f deg) legChain %u",
                      aName.c_str(), g_airTableReason.c_str(), g_airTable.NumAirzClips(), g_airTable.CountStatus(airtab::ClipStatus::Applied),
                      g_airTable.NumAirzClean(), g_airTable.NumAirzRejected(), g_airTable.Meta().size(), g_airTable.FallLoopClip(),
                      g_airTable.LoopOk() ? 1 : 0, static_cast<double>(g_airTable.LoopMaxDeg()), g_airLegChainN);
        g_log(line);
    }
}

std::wstring DbPathFor(const std::string& aName)
{
    if (aName == "default")
        return g_dbDir + L"data\\utpp_base.utppmm";
    return g_dbDir + L"data\\" + Widen(aName) + L".utppmm";
}

// Worker: everything heavy happens here (no g_mutex). Result handed over through g_sw.
void WorkerLoad(std::string aName, std::wstring aPath, std::vector<uint64_t> aExpectHashes)
{
    auto db = std::make_unique<Database>();
    std::string err, sha;
    bool ok = false;
    {
        std::vector<uint8_t> bytes;
        ok = ReadWholeFile(aPath, bytes, err);
        if (ok)
        {
            sha = Sha256Hex(bytes);
            ok = db->LoadFromMemory(bytes, kExpectedRigHash, err);
            if (!ok)
                err = "invalid database (rig hash / format): " + err;
        }
    } // file bytes freed before the matcher probe
    if (ok && (db->NumBones() > kMaxMMBones || db->NumBones() <= 2))
    {
        ok = false;
        err = "unsupported bone count";
    }
    if (ok && !aExpectHashes.empty())
    {
        bool same = db->NumBones() == aExpectHashes.size();
        for (uint32_t b = 0; same && b < db->NumBones(); ++b)
            same = CNameHash(db->BoneName(b)) == aExpectHashes[b];
        if (!same)
        {
            ok = false;
            err = "bone set differs from the active database";
        }
    }
    if (ok)
    {
        auto probe = std::make_unique<BaseMatcher>();
        if (!probe->Init(*db, MatcherConfig{}))
        {
            ok = false;
            err = "matcher init failed";
        }
    }
    std::vector<uint8_t> airz; // AIR CLIPS stage
    const bool haveAirz = ok && ReadAirz(aPath, airz);
    std::lock_guard lk(g_sw.m);
    g_sw.busy = false;
    if (ok)
    {
        g_sw.ready = std::move(db);
        g_sw.name = aName;
        g_sw.sha = sha;
        g_sw.haveSidecar = ReadSidecar(aPath, g_sw.sidecar);
        g_sw.haveAirz = haveAirz;
        g_sw.airz = std::move(airz);
        g_sw.status = "pending " + aName;
    }
    else
    {
        g_sw.ready.reset();
        g_sw.status = "FAILED " + aName + ": " + err;
        ++g_dbSwitchFailures;
    }
}

bool CandidateReadyLocked()
{
    std::lock_guard lk(g_sw.m);
    return g_sw.ready != nullptr;
}

// 0.6.2: upper-body mask (kMaskUpperFromGame) of the active database, by bone name; logged once by the next Feed.
void BuildUpperMaskLocked()
{
    g_upperMask = g_db.Loaded() ? BuildUpperMask(g_db.NumBones(), MatcherConfig{}.firstBodyBone, [](uint32_t b) { return g_db.BoneName(b); })
                                : UpperMask{};
    for (uint32_t i = 0; i < kMaskWords; ++i)
        g_snap.upperBits[i] = g_upperMask.bits[i];
    g_maskLogged = false;
}

void LogUpperMaskLocked()
{
    if (g_maskLogged || !g_log)
        return;
    g_maskLogged = true;
    std::string lo, up;
    for (uint32_t b = MatcherConfig{}.firstBodyBone; g_db.Loaded() && b < g_db.NumBones(); ++b)
    {
        std::string& s = g_upperMask.Upper(b) ? up : lo;
        s += s.empty() ? "" : ",";
        s += g_db.BoneName(b);
    }
    std::string line = "UTPP MM upper mask kMaskUpperFromGame ok=" + std::to_string(g_upperMask.ok ? 1 : 0) + " on=" +
                       std::to_string(g_upperMaskOn ? 1 : 0) + " states=0x" + std::to_string(g_maskStates) + " mmBones=" +
                       std::to_string(g_upperMask.lower) + " [" + lo + "] gameBones=" + std::to_string(g_upperMask.upper) + " [" + up + "]";
    g_log(line.c_str());
}

void RebuildDbBonesLocked()
{
    g_dbHashes.resize(g_db.NumBones());
    g_dbParents.resize(g_db.NumBones());
    for (uint32_t b = 0; b < g_db.NumBones(); ++b)
    {
        g_dbHashes[b] = CNameHash(g_db.BoneName(b));
        g_dbParents[b] = g_db.BoneParent(b);
    }
    BuildUpperMaskLocked();
}

// Tag filter of the active database (manual filter kept by substring when the new database has it, else cleared).
void ApplyTagFilterForDbLocked()
{
    g_tagMask = ~0ull;
    if (!g_tagFilter.empty())
    {
        const uint64_t mask = BaseMatcher::TagMaskContaining(g_db, g_tagFilter);
        if (mask != 0)
            g_tagMask = mask;
        else
            g_tagFilter.clear();
    }
    g_appliedTagMask = g_tagMask;
}

void ResetRuntimeLocked()
{
    g_accelSampleTicks = 0; // database/session replacement needs a fresh applied-stat observation
    g_bodyCtl.Reset();
    g_latch.Reset();
    g_footLock.Reset();
    ResetOffsetLocked();
    g_havePrevYaw = false;
    g_weight.Reset();
    g_maskBlend.Reset(); // 0.6.2
    g_maskTimes.Reset();
    g_maskFadeFromAim = false;
    g_weaponKnown = g_weaponHeld = g_weaponUpperActive = false;
    g_maskReasons = g_maskCovered = 0;
    g_exitLeft = 0.0f;
    g_exitLatchedPsm = 0; // CAR-1
    g_maskUpperFromGame = false;
    g_meleeFullGate.Reset(); // melee full body / aim tilt (flag and setting are settings: kept)
    g_meleeFullBlend.Reset();
    g_meleeAimBlend.Reset();
    g_meleeFullWant = false;
    g_meleeFullStick = false;
    g_meleeFullGround = -1;
    g_meleeGuardGate.Reset(); // melee guard (flag, settings and the full-body state mask are settings: kept)
    g_meleeGuardWant = false;
}

// b5 testorder: playback state that refers to the previous database's clips / pose stream (a playing clip's turn sign / commit,
// body-follow direction, start-rate / rate-floor flags, loop rate, the chest-twist low-pass). InitMatcherLocked re-inits the
// matcher (no playing clip) but left these from the old database. Called by TrySwapLocked (game database switch; the next feed
// recomputes each of them) and by ResetSessionStateLocked. The player's input / gate state (stick filter, stance history, gate
// debounce, melee / mount / vehicle-exit state) is deliberately NOT touched here: the player did not change with the database.
void ResetDbPlaybackStateLocked()
{
    g_motionStartTurnDirection = 0;
    g_turnCommitted = false;
    g_commitSign = 0;
    g_settleSign = 0;
    g_settling = false;
    g_bodyFollowDir = 0;
    g_startRateActive = false;
    g_lastExclude = 0;
    g_lastFloored = false;
    g_lastRate = 1.0f;
    g_loopRate = 1.0f;
    g_rootSpeedDb = 0.0f;
    g_rawAnimRate = 0.0f;
    g_turnRateTarget = 1.0f;
    g_strafeRateSrc = 0;
    g_chestAlign.Reset();
    g_chestOut = false;
    g_chestValid = false;
    g_stanceGuard.Reset(); // filtered knee swivel / held push of the old pose stream (StanceGuard::Init keeps them)
    {
        // RootRate::Reset (InitMatcherLocked) keeps the active-window timer / flags of the old clip: a fresh one, config kept
        const RootRateConfig rc = g_rootRate.Config();
        g_rootRate = RootRate{};
        g_rootRate.SetConfig(rc);
    }
}

// b5 testorder repair: which playback state of the current clips is non-default (status swapCarry / swapPost bits): 1 turn
// committed, 2 settling, 4 body-follow direction, 8 start rate, 16 RootRate active window, 32 StanceGuard filter, 64 chest align
// valid, 128 matcher cost-cap hold, 256 rate / loop rate != 1.
uint32_t PlaybackCarryLocked()
{
    uint32_t m = 0;
    m |= g_turnCommitted ? 1u : 0u;
    m |= g_settling ? 2u : 0u;
    m |= g_bodyFollowDir != 0 ? 4u : 0u;
    m |= g_startRateActive ? 8u : 0u;
    m |= g_rootRate.Active() ? 16u : 0u;
    m |= g_stanceGuard.FilterActive() ? 32u : 0u;
    m |= g_chestValid ? 64u : 0u;
    m |= g_matcher.CapHoldCarried() ? 128u : 0u;
    m |= (g_lastRate != 1.0f || g_loopRate != 1.0f) ? 256u : 0u;
    return m;
}

// b5 testorder: the whole per-session runtime state (feed / input / gate / playback state and the diagnostic counters) back to
// the values of a freshly loaded DLL; settings (Set* switches / parameters), the database, the armed rig, the feed dump, the
// last handled foot event sequence per foot (g_feetEventSeen: the FootLocker's sequence keeps counting, an old event must not be
// handled again; only the "events seen" diagnostic counter g_feetEvents is zeroed below) and the UTPP-V2 seam state are kept;
// the matcher's cost-cap hold is cleared (Init keeps it). Called by LoadDatabase: in game that is
// the one load at plugin start (state already fresh = no change); in the offline suites every section's LoadDatabase now starts
// from the same state whatever ran before (the loco suite's section results no longer depend on the section order).
void ResetSessionStateLocked()
{
    ResetRuntimeLocked(); // body controller, latch, foot lock, offset root, weight, mask blend, vehicle exit window
    ResetDbPlaybackStateLocked();
    g_matcher.ResetCapHold(); // b5 testorder repair: moved here from BaseMatcher::Init (other re-inits keep it as in 0.7.0)
    // gate / feed
    g_psmBlock = 0;
    g_gateCloses = g_psmBlockFeeds = 0;
    g_gatePrev = 0xFFFFFFFFu;
    g_gate = kGateMMDisabled | kGateDbNotLoaded | kGateNotMapped;
    g_effGate = kGateMMDisabled | kGateDbNotLoaded | kGateNotMapped;
    g_target = 0.0f;
    g_feedCalls = 0;
    g_lastFeed = FeedInput{};
    g_lastCharVel[0] = g_lastCharVel[1] = 0.0f;
    g_lastYawRate = 0.0f;
    g_prevYaw = 0.0f;
    g_lastFeedTicks = 0;
    g_lastTickTicks = 0;
    g_gateDebounce = GateDebounce{}; // with its episode counters (status debounced / debounceClosed)
    g_rawGate = 0;
    g_lastPsmDetail = g_lastSoftDetail = -1;
    g_lastGoodLoco = 0;
    g_lastDetail = -1;
    // trace / timing diagnostics
    g_traceLastTransitions = 0;
    g_traceLastSummary = 0;
    g_traceMaxSearchUs = 0.0;
    g_traceCapHolds = g_traceCapRetries = g_traceCapForced = 0;
    g_publishes = g_matcherUpdates = g_staleFades = 0;
    g_lastUpdateUs = g_lastFootLockUs = 0.0;
    g_feetEvents = 0;
    // body turn / steering input
    g_haveTarget = false;
    g_targetYaw = 0.0f;
    g_bodyTurnFed = false;
    g_followFed = false;
    g_lastCmdTicks = 0;
    g_lastCmdDeg = g_lastCmdCurDeg = g_lastCmdBaseDeg = g_lastCmdDt = 0.0f;
    g_bodyCmds = g_bodyPassThrough = 0;
    g_latch.holds = 0;
    {
        const steer::SteerConfig sc = g_steer.Config(); // fresh steering state incl. its last error and entry / exit counters
        g_steer = steer::Steering{};
        g_steer.SetConfig(sc);
    }
    g_stanceGuard.ResetDiagnostics();
    g_stSign = 0;
    g_pivotAccelActive = false;
    g_prevBtFed = false;
    g_accelLastTarget = g_decelLastTarget = 0.0f;
    g_accelBase = g_decelBase = 0.0f; // last AccelTarget / DecelTarget query inputs
    g_accelObserved = 0.0f;
    g_accelSampleTicks = 0;
    g_accelSampleLoco = 0;
    g_accelSampleDetail = -1;
    g_decelSampleTicks = 0;
    g_decelSampleLoco = 0;
    g_decelSampleDetail = -1;
    g_accelMul = g_decelMul = 1.0f;
    g_accelQueries = g_accelActive = g_decelQueries = g_decelActive = 0;
    g_trajHlApplied = 0.0f;
    g_trajCalFeeds = 0;
    // stick / stance history (0.5.x sinceStance carry-over)
    g_stick.Reset();
    g_stickHeld = false;
    g_prevStick = false;
    g_stickEdgeTime = 1e6f;
    g_slowGateActive = false;
    g_prevStance = 0;
    g_sinceStance = 1e6f;
    g_stanceChanges = 0;
    g_stanceClipOk = true;
    g_stanceClipSpeed = g_stanceClipStickFwd = 0.0f;
    g_stanceClipSkips = g_stanceClipAllows = g_stanceClipExcl = 0;
    g_lastBand = 0;
    // curation / turn counters
    g_lateralExcluded = 0;
    g_turnCommitHolds = g_standIdleSearches = 0;
    g_idleHoldSearches = g_idleHoldNoLoop = 0;
    g_againstSearches = g_startRateFeeds = 0;
    g_turnRateFeeds = 0;
    g_turnRateTurns = 0;
    g_turnRateLastSummary = "none";
    g_commitNotAllowed = 0;
    g_settleHolds = 0;
    g_loopBandSearches = 0;
    g_bindFeeds = g_bindCommitSearches = g_bindReversals = g_binds = 0;
    g_bindTarget = 0.0f;
    g_overSpeed = false;
    g_rawSpeed = 0.0f;
    g_overSpeedFeeds = g_queryClamped = 0;
    // strafe / chest
    g_airOwnsPrev = false;
    g_strafeEdge = false;
    g_strafeStance = g_strafeGait = 0;
    g_aimJogLastTarget = 0.0f;
    g_aimJogQueries = g_aimJogOnAnswers = 0;
    g_strafeDirDeg = 0.0f;
    g_strafeSearches = 0;
    g_strafeSUsSum = g_strafeSUsMax = 0.0;
    g_chestTeleports = 0;
    // air (a fresh AirState: phase, learned gravity and episode statistics; config kept, mode applied by the next feed)
    g_air = air::AirState(g_air.Config());
    g_airSteerFed = false;
    g_airSteerVelocityYaw = 0.0f;
    g_airSteerFeeds = g_airSteerCmds = g_airSteerBlocked = 0;
    g_airSteerMaxRateDeg = 0.0f;
    g_airDec = air::AirDecision{};
    g_airOwns = false;
    g_airLastAirborne = false;
    g_airLastVz = 0.0f;
    g_airLastRelease = false;
    g_airFeeds = g_airFreshFeeds = g_airProbes = g_airProbeHits = g_airProbeWants = 0;
    g_airOwnedFeeds = g_airSuppressed = 0;
    g_airUs = g_airUsMax = 0.0;
    g_airPrevPhase = air::Phase::Ground;
    g_airPlays = g_airPlayFails = g_airQueryOverrides = 0;
    g_airLandWasApproach = false;
    g_airLandRealigns = 0;
    g_airLandGroundBlendLeft = 0.0f;
    g_airLandGroundBlendPending = false;
    g_airLandGroundBlendOutputValid = false;
    g_airLandGroundBones = 0;
    g_airLandGroundBlends = 0;
    g_airLandGroundElapsed = 0.0f;
    g_airLandGroundStartPending = false;
    g_airLandGroundFirstHipsError = 0.0f;
    g_airHoldGroundReplantLeft = 0.0f;
    g_stanceAirBypassed = false;
    g_legPipelineOutput = {};
    // melee / mount / upper-body mask / hold / release
    g_meleeGate = MeleeGate{}; // with its counters
    g_meleeActive = false;
    g_meleeBits = 0;
    g_meleeState = g_upperBody = -1;
    g_mountMuted = false;
    g_mountCuts = g_mountEventCuts = 0;
    vehtrans::Abort(); // vehicle batch W1 P2
    g_vtMounted = g_vtExitPending = g_vtWorldOk = g_vtPrevPosOk = g_vtRide = false;
    g_vtMounts = g_vtExitsHanded = g_vtExitsNoCapture = g_vtAborts = g_vtRearmsKept = 0;
    g_vtLastRideTpp = g_vtExitFillPending = false;
    g_vtExitSeedFeed = 0;
    g_vtExitsFilledNoCapture = g_vtNoCapBase = g_vtFillBase = 0;
    g_vtExitBase = vehtrans::SnapCounters();
    g_mountEventLeft = 0.0f;
    g_maskSwitches = g_maskFeeds = g_exitWindows = 0;
    g_prevVeh = false;
    g_jumpOut.Reset();
    g_jumpOut.ClearStats();
    g_jumpOutFeeds = 0;
    g_jumpOutExit = false;
    g_jumpOutHoldT = g_jumpOutGroundT = 0.0f;
    g_jumpOutGroundN = 0;
    g_jumpOutFills = 0;
    g_jumpOutRearms = 0;
    g_landDropout = false;
    g_lastRealFeedTicks = 0;
    g_simPrev = 0.0;
    g_simPrevValid = false;
    g_simSilence = 0.0f;
    g_simSeenFeedTicks = 0;
    g_simClockUsable = false;
    g_simWatch.store(false, std::memory_order_relaxed);
    g_synthGuardOn = false;
    g_ttlAtFeed = -1.0f;
    g_landDropoutEpisodes = g_landDropoutFeeds = 0;
    g_jumpOutRearmPromoted = false;
    g_jumpOutLimiter.Reset();
    g_jumpOutSlewFrames = 0;
    g_jumpOutSlewLeft = 0.0f;
    g_jumpOutLimHandIn = false;
    g_holdWhy = nullptr; // CAR-1
    g_exitDiagT = -1.0f;
    g_exitDiagLines = 0;
    g_exitDiagKey[0] = '\0';
    g_holdActive = false;
    g_holdSwap = false;
    g_holdFrames = g_holdEpisodes = g_holdSwaps = g_holdTicks = 0;
    g_releaseReasons = g_releaseTracePrev = 0;
    g_travReleaseFeeds = g_travReleaseEpisodes = g_travOwnedFeeds = 0;
    // derived class data recompute counters (the load's own recompute counts from 0 again)
    g_derivedRecomputes = g_derivedStale = 0;
}

// Game thread, g_mutex held, MM weight 0: swaps the database in. Failure restores the previous one.
void TrySwapLocked()
{
    std::unique_ptr<Database> cand;
    std::string name, sha, sidecar;
    bool haveSidecar = false;
    bool haveAirz = false;     // AIR CLIPS stage
    std::vector<uint8_t> airz;
    {
        std::lock_guard lk(g_sw.m);
        if (!g_sw.ready)
            return;
        cand = std::move(g_sw.ready);
        name = g_sw.name;
        sha = g_sw.sha;
        haveSidecar = g_sw.haveSidecar;
        sidecar = std::move(g_sw.sidecar);
        haveAirz = g_sw.haveAirz; // AIR CLIPS stage
        airz = std::move(g_sw.airz);
        g_sw.airz.clear();
        g_sw.haveAirz = false;
    }
    const uint32_t carry = PlaybackCarryLocked(); // b5 testorder repair: playback state of the old database before the swap
    Database old = std::move(g_db);
    const bool oldLoaded = g_dbLoaded;
    const std::vector<uint64_t> oldHashes = g_dbHashes;
    const std::vector<int32_t> oldParents = g_dbParents;
    const std::string oldFilter = g_tagFilter;
    g_db = std::move(*cand);
    cand.reset();
    RebuildDbBonesLocked();
    ApplyTagFilterForDbLocked();
    airtab::AirTable oldAirTable = std::move(g_airTable); // AIR CLIPS stage: restored with the old database on failure
    const bool oldAirValid = g_airTableValid;
    const std::string oldAirReason = g_airTableReason;
    const std::array<uint32_t, 6> oldAirChain = g_airLegChain;
    const uint32_t oldAirChainN = g_airLegChainN;
    ParseAirTableLocked(haveAirz, airz, sha, name);
    airz.clear();
    bool ok = InitMatcherLocked();
    std::string err = ok ? "" : "matcher init failed";
    BoneMapResult map{};
    if (ok && g_haveGameRig)
    {
        map = BuildBoneMap(g_dbHashes, g_dbParents, g_gameHashes, g_gameParents, g_cfg.firstBodyBone);
        if (!map.ok)
        {
            ok = false;
            err = "bone map failed against the armed rig";
        }
    }
    if (!ok)
    {
        g_db = std::move(old);
        g_dbHashes = oldHashes;
        g_dbParents = oldParents;
        g_tagFilter = oldFilter;
        ApplyTagFilterForDbLocked();
        g_airTable = std::move(oldAirTable); // AIR CLIPS stage
        g_airTableValid = oldAirValid;
        g_airTableReason = oldAirReason;
        g_airLegChain = oldAirChain;
        g_airLegChainN = oldAirChainN;
        g_dbLoaded = oldLoaded && InitMatcherLocked();
        BuildUpperMaskLocked(); // 0.6.2: mask of the restored database
        ResetRuntimeLocked();
        if (!g_holdSwap)
            PublishLocked(0.0f); // 0.6.2: under hold the last (held) pose stays published until the next feed
        ++g_dbSwitchFailures;
        std::lock_guard lk(g_sw.m);
        g_sw.status = "FAILED " + name + ": " + err;
        return;
    }
    if (g_haveGameRig)
    {
        g_map = map;
        g_readMap = BuildBoneMap(g_dbHashes, g_dbParents, g_gameHashes, g_gameParents, 0); // vehicle batch W1 P2 capture map
        char buf[96];
        std::snprintf(buf, sizeof buf, "ok %u/%u (after switch)", g_map.mapped, g_db.NumBones() - g_cfg.firstBodyBone);
        g_mapReason = buf;
    }
    g_dbLoaded = true;
    ComputeStateTagMasksLocked();
    {
        // loco P0: InitMatcherLocked above built the loop bands from the previous database's masks
        const bool stale = RecomputeDerivedClassDataLocked(true);
        if (g_derivedRecomputeOn)
            LogDerivedLocked(name, stale);
    }
    if (g_crouchExclOn)
        LogCrouchExclLocked(name, "switch"); // loco C1a
    ResetRuntimeLocked();
    if (g_swapResetOn)
    {
        ResetDbPlaybackStateLocked(); // b5 testorder: no playback state of the old database's clips
        g_matcher.ResetCapHold();
        ++g_swapResets;
    }
    g_swapCarry = carry;
    g_swapPost = PlaybackCarryLocked();
    g_dbName = name;
    g_dbSha = sha;
    CalibrateLocked(haveSidecar, sidecar); // 0.5.2
    g_dbPath = util::RelativeToGame(DbPathFor(name)); // shown in status / log lines: relative to the game folder
    char buf[160];
    std::snprintf(buf, sizeof buf, "ok: %u frames, %u bones, %u clips, %u tags", g_db.NumFrames(), g_db.NumBones(),
                  g_db.NumClips(), g_db.NumTags());
    g_dbReason = buf;
    ++g_dbSwitches;
    if (!g_holdSwap)
        PublishLocked(0.0f); // 0.6.2: under hold the last (held) pose stays published until the next feed
    std::lock_guard lk(g_sw.m);
    g_sw.status = "ok " + name;
    // `old` (previous database memory) is freed here
}
} // namespace

// vehicle batch W1 P2 (defined before Feed)
void VtBeginMountLocked(const FeedInput& aIn, int64_t aNow);
void VtAbortLocked();

bool LoadDatabase(const std::wstring& aPath, const std::string& aName)
{
    std::lock_guard lock(g_mutex);
    g_dbLoaded = false;
    g_db.Reset();
    g_dbName = "none";
    g_dbSha.clear();
    g_dbPath = util::RelativeToGame(aPath);
    // b5 testorder: a load starts a fresh session (game: the one load at plugin start, state already fresh). The state tag masks
    // of the previous database are cleared as well: InitMatcherLocked below builds the loop bands before the new masks are
    // computed (loco P0 recompute), so the previous database's masks made the derived-data log depend on what was loaded before.
    ResetSessionStateLocked();
    for (uint64_t& m : g_stateTagMask)
        m = 0;

    std::vector<uint8_t> bytes;
    std::string err;
    if (!ReadWholeFile(aPath, bytes, err))
    {
        g_dbReason = err;
        return false;
    }
    const std::string sha = Sha256Hex(bytes);
    if (!g_db.LoadFromMemory(bytes, kExpectedRigHash, err))
    {
        g_dbReason = "invalid database: " + err;
        g_db.Reset();
        return false;
    }
    if (g_db.NumBones() > kMaxMMBones || g_db.NumBones() <= 2)
    {
        g_dbReason = "unsupported bone count";
        g_db.Reset();
        return false;
    }
    RebuildDbBonesLocked();
    ApplyTagFilterForDbLocked(); // b5 testorder: as TrySwapLocked (the old tag mask indexed the previous database's tags)
    {
        std::vector<uint8_t> airz; // AIR CLIPS stage: parsed before InitMatcherLocked (which applies it)
        const bool haveAirz = ReadAirz(aPath, airz);
        ParseAirTableLocked(haveAirz, airz, sha, aName);
    }
    if (!InitMatcherLocked())
    {
        g_dbReason = "matcher init failed";
        g_db.Reset();
        return false;
    }
    g_hot.mmStaleTicks.store(static_cast<int64_t>(kDetourStaleSeconds * QpcFreq()), std::memory_order_relaxed);
    g_dbLoaded = true;
    g_dbName = aName;
    g_dbSha = sha;
    ComputeStateTagMasksLocked();
    {
        // loco P0: InitMatcherLocked above built the loop bands before the masks (0 / previous database)
        const bool stale = RecomputeDerivedClassDataLocked(true);
        if (g_derivedRecomputeOn)
            LogDerivedLocked(aName, stale);
    }
    if (g_crouchExclOn)
        LogCrouchExclLocked(aName, "load"); // loco C1a
    {
        std::string sidecar;
        const bool haveSidecar = ReadSidecar(aPath, sidecar);
        CalibrateLocked(haveSidecar, sidecar); // 0.5.2
    }
    char buf[160];
    std::snprintf(buf, sizeof buf, "ok: %u frames, %u bones, %u clips, %u tags", g_db.NumFrames(), g_db.NumBones(),
                  g_db.NumClips(), g_db.NumTags());
    g_dbReason = buf;
    return true;
}

void ResetSessionState()
{
    std::lock_guard lock(g_mutex);
    ResetSessionStateLocked();
    PublishLocked(0.0f); // weight 0 (ResetRuntimeLocked reset the fader): nothing of the previous session stays published
}

void OnArmed(void* aRig, const std::vector<uint64_t>& aGameHashes, const std::vector<int16_t>& aGameParents)
{
    std::lock_guard lock(g_mutex);
    // review fix (vehdll MMDriver:2612, CAR-1 / MOTO-6): the game re-arms (animated object / tracks / desc changed) in the exit
    // state and 0.05-0.52 s after many unmounts with the SAME rig and bone names. Inside a ride, the exit window or a running
    // hand-in such a re-arm keeps the vehicle transition / capture / hand-in and the MM weight (the map below is rebuilt from the
    // same inputs, so it is identical); the weight reset would show the live post-exit squat for a frame and fade MM in again.
    // Any other re-arm (on foot, another rig or other names) resets as before.
    const bool vtSameRig = g_haveGameRig && g_dbLoaded && g_rig == reinterpret_cast<uint64_t>(aRig) && g_gameHashes == aGameHashes &&
                           g_gameParents == aGameParents;
    // mutation 8: every re-arm aborts the hand-in and resets the weight (pre-fix)
    const bool vtKeep = UTPP_AND_NOT(VEHEXIT, 8) vtSameRig && (g_vtRide || g_vtExitPending || vehtrans::HandInActive() || g_exitLeft > 0.0f);
    g_rig = reinterpret_cast<uint64_t>(aRig);
    g_rigBones = static_cast<uint32_t>(aGameHashes.size());
    g_gameHashes = aGameHashes; // 0.5.1: re-mapping after a database switch
    g_gameParents = aGameParents;
    g_haveGameRig = true;
    if (!g_dbLoaded)
    {
        g_map = BoneMapResult{};
        g_mapReason = "db not loaded";
    }
    else
    {
        g_map = BuildBoneMap(g_dbHashes, g_dbParents, aGameHashes, aGameParents, g_cfg.firstBodyBone);
        g_readMap = BuildBoneMap(g_dbHashes, g_dbParents, aGameHashes, aGameParents, 0); // vehicle batch W1 P2 capture map
        char buf[256];
        if (g_map.ok)
            std::snprintf(buf, sizeof buf, "ok %u/%u%s", g_map.mapped, g_db.NumBones() - g_cfg.firstBodyBone,
                          aGameParents.empty() ? " (no parent check)" : "");
        else
            std::snprintf(buf, sizeof buf, "FAILED missing=%u (first '%.*s') parentMismatch=%u (first '%.*s') rigBones=%u",
                          g_map.missing, g_map.firstMissing >= 0 ? int(g_db.BoneName(uint32_t(g_map.firstMissing)).size()) : 0,
                          g_map.firstMissing >= 0 ? g_db.BoneName(uint32_t(g_map.firstMissing)).data() : "",
                          g_map.parentMismatch,
                          g_map.firstParentMismatch >= 0 ? int(g_db.BoneName(uint32_t(g_map.firstParentMismatch)).size()) : 0,
                          g_map.firstParentMismatch >= 0 ? g_db.BoneName(uint32_t(g_map.firstParentMismatch)).data() : "",
                          g_rigBones);
        g_mapReason = buf;
    }
    if (g_motion && g_motion->armed) // UTPP-MOTION SEAM
        g_motion->armed(g_rig, aGameHashes, aGameParents); // UTPP-MOTION SEAM
    if (vtKeep)
    {
        ++g_vtRearmsKept;
        // mutation 18: a re-arm in the flight keeps MM at once (pre-fix)
        // 2026-10-28 11-49-37: a same-rig re-arm in the jump-out flight follows frames in which the game showed its own pose on a new
        // animated object (the hooks matched the old one until now); publishing MM at the kept weight would jump from that pose to MM
        // in one frame (108 deg). The new object's live pose is captured and handed in like the launch (the weight stays: no fade).
        if (UTPP_AND_NOT(VEHEXIT, 18) g_jumpOutExit && !g_vtRide && g_weight.Weight() > 0.0f)
        {
            vehtrans::BeginRearmCapture(Qpc());
            g_jumpOutRearmPromoted = false;
            ++g_jumpOutRearms;
        }
        if (g_log)
        {
            char line[160];
            std::snprintf(line, sizeof line, "UTPP vehtrans: same-rig re-arm kept (ride %d exitLeft %.2f handIn %d w %.2f), kept %llu",
                          g_vtRide ? 1 : 0, static_cast<double>(g_exitLeft), vehtrans::HandInActive() ? 1 : 0,
                          static_cast<double>(g_weight.Weight()), static_cast<unsigned long long>(g_vtRearmsKept));
            g_log(line);
        }
        PublishLocked(g_weight.Weight()); // the same rig and map: the published weight and the hand-in continue
        return;
    }
    VtAbortLocked(); // vehicle batch W1 P2: rig change
    // A new map must not be combined with a weight from the old rig: restart the fade-in.
    g_weight.Reset();
    PublishLocked(0.0f);
}

void OnDisarmed()
{
    std::lock_guard lock(g_mutex);
    g_map = BoneMapResult{};
    g_readMap = BoneMapResult{}; // vehicle batch W1 P2
    VtAbortLocked();
    g_rig = 0;
    g_rigBones = 0;
    g_gameHashes.clear();
    g_gameParents.clear();
    g_haveGameRig = false;
    g_mapReason = "not armed";
    if (g_motion && g_motion->disarmed) // UTPP-MOTION SEAM
        g_motion->disarmed(); // UTPP-MOTION SEAM
    g_weight.Reset();
    mm::vaulttorso::rt::OnDisarmed(); // vault torso limiter: forgets the rig and the episode (under g_mutex, like its Feed)
    PublishLocked(0.0f);
}

// ---- batch 1b feed dump (SetFeedDump, default off; mm/FeedDump.hpp) ----
namespace
{
feeddump::Dumper g_feedDump{feeddump::kDefaultRing};
bool g_feedDumpOn = false;          // records are built and pushed (game thread, under g_mutex)
std::wstring g_feedDumpDir;         // SetFeedDumpDir (with a trailing separator)
int64_t g_feedDumpT0 = 0;           // Qpc at the start of the dump (column t)
feeddump::Record g_dumpRec;         // record of the Feed call in progress
bool g_feedDumpFailLogged = false;  // writer failure of this session logged

// raw FeedInput (before the stick filter / debounce / hold conditioning)
void BeginFeedDumpLocked(const FeedInput& aRaw)
{
    feeddump::Record& r = g_dumpRec;
    r = feeddump::Record{};
    r.t = static_cast<double>(Qpc() - g_feedDumpT0) / QpcFreq();
    r.dt = aRaw.dt;
    for (int i = 0; i < 3; ++i)
        r.vel[i] = aRaw.velWorld[i];
    r.move[0] = aRaw.move[0];
    r.move[1] = aRaw.move[1];
    r.yawDeg = aRaw.playerYawDeg;
    r.camDeg = aRaw.cameraYawDeg;
    r.loco = aRaw.locoState;
    r.psm = aRaw.psmBlock;
    r.haveTarget = aRaw.haveTarget;
    r.targetYawDeg = aRaw.targetYawDeg;
    r.haveCapsule = aRaw.haveExtra;
    if (aRaw.haveExtra)
    {
        for (int i = 0; i < 3; ++i)
            r.capsule[i] = aRaw.capsulePos[i];
        r.walk = aRaw.walkToggled ? 1 : 0;
    }
    r.upperBody = PsmUpperBodyOf(aRaw.psmBlock);
    r.detailed = PsmDetailOf(aRaw.psmBlock);
    if (aRaw.haveAir) // AIR HOLD stage: UTPPFeedAir columns ('-' without an air feed this frame)
    {
        r.onGround = aRaw.onGround ? 1 : 0;
        r.distGround = aRaw.distToGround;
        r.psmFall = aRaw.psmFall;
    }
}

// v1 choice right after the matcher update of this feed (before a database swap can replace the matcher / names)
void CaptureFeedDumpBaseLocked()
{
    const MatcherDiagnostics& d = g_matcher.Diagnostics();
    feeddump::Record& r = g_dumpRec;
    r.v1 = true;
    r.trans = d.transitionedThisUpdate;
    r.baseFlags = d.clip >= 0 ? d.clipFrame : -1;
    feeddump::SetClip(r, d.clip >= 0 ? g_db.ClipName(static_cast<uint32_t>(d.clip)) : std::string_view{});
    r.baseSearched = d.searches > 0;
    r.baseCost = d.lastSearchCost;
    r.baseMicros = d.lastSearchMicros;
}

void EndFeedDumpLocked()
{
    g_dumpRec.effGate = g_effGate;
    g_dumpRec.gateRaw = g_rawGate;
    const feeddump::PushResult res = g_feedDump.Push(g_dumpRec);
    if (res == feeddump::PushResult::FirstDrop && g_log)
    {
        char line[200];
        std::snprintf(line, sizeof line, "UTPP MM feed dump: the writer is behind - lines dropped (ring %u lines), counted in the status (feedDump dropped=)",
                      g_feedDump.Capacity());
        g_log(line);
    }
    else if (res == feeddump::PushResult::Off && g_feedDump.Failed() && !g_feedDumpFailLogged)
    {
        g_feedDumpFailLogged = true;
        g_feedDumpOn = false;
        g_feedDump.Stop();
        if (g_log)
            g_log(("UTPP MM feed dump FAILED, dump off: " + g_feedDump.Status()).c_str());
    }
}

// One dump record per Feed call, pushed when Feed returns (every return path). Constructed after the driver lock, so it is
// destroyed before the lock is released. Dump off: nothing.
struct FeedDumpScope
{
    explicit FeedDumpScope(const FeedInput& aRaw) : m_on(g_feedDumpOn && !aRaw.synthetic)
    {
        if (m_on)
            BeginFeedDumpLocked(aRaw);
    }
    ~FeedDumpScope()
    {
        if (m_on)
            EndFeedDumpLocked();
    }
    const bool m_on;
    FeedDumpScope(const FeedDumpScope&) = delete;
    FeedDumpScope& operator=(const FeedDumpScope&) = delete;
};
} // namespace

// B2 strafe S1: the strafe decision of this feed (before the steering block). Returns true while the mode is Active / Exiting.
// Edges (entered / exited): steering, target latch, start bind, turn commit / settle and the rate smoother restart; on exit also
// the body-yaw controller (orient-to-movement restarts cleanly once Locomotion is back). Switch off: Wanted is false, the mode
// stays Off (or leaves within exitHold) and nothing else here runs.
bool UpdateStrafeLocked(const FeedInput& aIn, int aPsmDetail, float aDt)
{
    g_strafeEdge = false;
    g_aimJogMotionHeld = false; // UTPP-MOTION SEAM
    if (!g_strafeOn && !g_strafeMode.On())
        return false;
    strafe::StrafeInput si;
    si.strafeOn = g_strafeOn && g_dbLoaded && g_matcher.Ready();
    si.tpp = aIn.tppActive;
    si.upperMaskOn = g_upperMaskOn;
    si.upperMaskOk = g_upperMask.ok;
    si.upperBody = g_upperBody;
    si.maskStates = g_maskStates;
    si.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
    si.psmDetail = aPsmDetail;
    // AimWalk reports an unsupported locomotion state (0) = standing aim; crouch aim keeps locoState 2
    si.stance = aIn.locoState == kLocoCrouch ? strafe::kStanceCrouch : strafe::kStanceStand;
    const bool wanted = !g_airOwnsPrev && strafe::Wanted(si, g_strafeCfg, g_strafeProf.avail);
    // UTPP-MOTION SEAM batch 2: strafe is not mapped to v2 - while v2 holds the v1 matcher (from the feed after the first v2-owned
    // UTPP-MOTION SEAM one up to the handback feed) the mode leaves at once (Exited edge below: steering / chest resets, no exit hold)
    // UTPP-MOTION SEAM one-feed lag at both ends (ownership is decided later in Feed, MotionStepLocked / MotionHandbackLocked in the matcher
    // UTPP-MOTION SEAM block): entry - strafe may still be on for the first v2-owned feed (ChestAlign off there via LegContext::v2);
    // UTPP-MOTION SEAM exit - the handback feed (v1 owns the frame again) still sees g_motionHeldMatcher, so strafe stays off on it and
    // UTPP-MOTION SEAM re-enters through Wanted on the next feed (integ 7e handback phase)
    if (g_motionHeldMatcher) // UTPP-MOTION SEAM always false with v2 off
    { // UTPP-MOTION SEAM
        g_strafeMode.ForceOff(); // UTPP-MOTION SEAM
        g_aimJogMotionHeld = wanted; // UTPP-MOTION SEAM jog speed decoupled from the strafe animation state (AimJogTarget)
    } // UTPP-MOTION SEAM
    else // UTPP-MOTION SEAM
        g_strafeMode.Update(wanted, aDt, g_strafeCfg); // UTPP-MOTION SEAM (v1 line, now the else branch)
    g_strafeStance = si.stance;
    g_strafeGait = (si.stance == strafe::kStanceStand && g_strafeJogF > 0.0f && g_strafeProf.availGait[strafe::kStanceStand][strafe::kGaitRun])
                       ? strafe::kGaitRun
                       : strafe::kGaitWalk;
    if (g_strafeMode.Entered() || g_strafeMode.Exited())
    {
        g_strafeEdge = true;
        g_steer.Reset();
        g_latch.Reset();
        ReleaseStartBindLocked();
        g_turnCommitted = false;
        g_prevCommitted = false;
        g_settleLeft = 0.0f;
        g_strafeRate.Reset();
        g_chestAlign.Reset(); // B2 strafe S2: chest twist filters restart on strafe enter / exit
        if (g_strafeMode.Exited())
            g_bodyCtl.Reset();
        if (g_trace && g_log)
        {
            char line[200];
            std::snprintf(line, sizeof line, "UTPP MMSTRAFE %s stance=%d gait=%d upperBody=%d states=0x%X vz=%.2f detail=%d entries=%llu blips=%llu",
                          g_strafeMode.Entered() ? "on" : "off", g_strafeStance, g_strafeGait, g_upperBody, g_maskStates,
                          static_cast<double>(si.vz), aPsmDetail, static_cast<unsigned long long>(g_strafeMode.Entries()),
                          static_cast<unsigned long long>(g_strafeMode.Blips()));
            g_log(line);
        }
    }
    return g_strafeMode.On();
}

// AIR HOLD stage: the air decision of this feed (after the hold block). Release (Reset of a running episode, no new one while it
// lasts): TPP off, MM not driving / no usable pose, PSM release bits, the body neither held nor on an open gate; the release
// details (kPsmReleaseDetails) and climb / vault entries are handled inside AirState. A stale air feed (no UTPPFeedAir this
// frame): AirState infers onGround from the PSM detail.
// AIR CLIPS stage: AirInput feedback from the clip the air area forced last (still playing): TakeOff -> time to its apex, Descend
// -> time to the root_z zero crossing (Start) / clip end (Off), Recover -> the land clip's planar root speed (database time).
void AirClipFeedbackLocked(air::AirInput& aAi)
{
    if (!g_airTableValid || g_airPlayMeta < 0 || size_t(g_airPlayMeta) >= g_airTable.Meta().size() || !g_matcher.Ready())
        return;
    const airtab::ClipMeta& m = g_airTable.Meta()[size_t(g_airPlayMeta)];
    const MatcherDiagnostics& d = g_matcher.Diagnostics();
    if (d.clip != m.clip)
        return;
    const float fps = g_db.Fps() > 0.0f ? g_db.Fps() : 30.0f;
    const float local = static_cast<float>(d.clipFrame) + d.alpha;
    const air::Phase ph = g_air.CurrentPhase();
    if (ph == air::Phase::TakeOff && m.kind == airtab::Kind::Start && m.apex >= 0 && float(m.apex) > local)
        aAi.clipRemainSeconds = (float(m.apex) - local) / fps;
    else if (ph == air::Phase::Descend)
    {
        const int32_t target = m.kind == airtab::Kind::Start ? m.zeroCross : (m.kind == airtab::Kind::Off ? int32_t(m.numFrames) - 1 : -1);
        if (target >= 0 && float(target) > local)
            aAi.clipRemainSeconds = (float(target) - local) / fps;
    }
    else if (ph == air::Phase::Recover && m.kind == airtab::Kind::Land)
    {
        float vx = 0.0f, vy = 0.0f;
        g_matcher.RootVelocity(vx, vy);
        aAi.clipRootSpeed = std::sqrt(vx * vx + vy * vy);
    }
}

// AIR CLIPS stage: leg-pose cost of a candidate entry frame against the current matcher output (UpLeg / Leg / Foot both sides).
float AirLegCostLocked(uint32_t aGlobalFrame)
{
    const auto pose = g_db.Pose(aGlobalFrame);
    const Pose& out = g_matcher.Output();
    float c = 0.0f;
    for (uint32_t i = 0; i < g_airLegChainN; ++i)
    {
        const uint32_t b = g_airLegChain[i];
        const size_t o = size_t(b) * 7 + 3;
        if (o + 4 > pose.size() || b >= out.bones.size())
            continue;
        const math::Quat& q = out.bones[b].q;
        const float a[4] = {q.x, q.y, q.z, q.w};
        c += airtab::QuatAngle(a, pose.data() + o);
    }
    return c;
}

// AIR CLIPS stage: entry frame (clip-local) of a pick. Start / Land never enter at local frame 0 (the first segment of some motion
// land clips jumps 0.4 m in the Hips).
int32_t AirEntryLocal(const airtab::ClipMeta& aM, const air::PickRequest& aP)
{
    int32_t f = 0;
    switch (aP.entry)
    {
    case air::Entry::LiftoffMinus: f = std::max(aM.liftoff, 0) - aP.entryFrames; break;
    case air::Entry::DropFrame: f = aM.drop >= 0 ? aM.drop : std::max(aM.liftoff, 0); break;
    case air::Entry::LoopStart: f = 0; break;
    case air::Entry::TouchMinus: f = std::max(aM.touch, 0) - aP.entryFrames; break;
    default: f = 0; break;
    }
    const int32_t lo = (aM.kind == airtab::Kind::Start || aM.kind == airtab::Kind::Land) ? 1 : 0;
    return std::clamp(f, std::min(lo, int32_t(aM.numFrames) - 1), int32_t(aM.numFrames) - 1);
}

// AIR CLIPS stage: resolves this feed's AirDecision play action (kind / direction / band / weight, foot by leg-pose cost) and
// ForcePlays the entry frame with the decision's half-life. Called right before g_matcher.Update.
void ApplyAirPlayLocked()
{
    const air::Action a = g_airDec.action;
    if (!g_airTableValid || g_airDec.effectiveMode != air::Mode::Clips || !g_matcher.Ready() ||
        !(a == air::Action::PlayTakeOff || a == air::Action::PlayOff || a == air::Action::PlayFallLoop || a == air::Action::PlayLand))
        return;
    const air::PickRequest& p = g_airDec.pick;
    static constexpr airtab::Kind kKind[] = {airtab::Kind::None, airtab::Kind::Start, airtab::Kind::Off, airtab::Kind::FallLoop, airtab::Kind::Land};
    static constexpr airtab::Dir kDir[] = {airtab::Dir::F, airtab::Dir::B, airtab::Dir::L, airtab::Dir::R};
    static constexpr airtab::Band kBand[] = {airtab::Band::Stand, airtab::Band::Walk, airtab::Band::Run, airtab::Band::Sprint};
    const airtab::Kind kind = kKind[static_cast<int>(p.kind) <= 4 ? static_cast<int>(p.kind) : 0];
    const airtab::Dir dir = kDir[static_cast<int>(p.dir) & 3];
    const airtab::Band band = kBand[static_cast<int>(p.band) & 3];
    const airtab::Weight weight = p.weight == air::Weight::Heavy ? airtab::Weight::Heavy : airtab::Weight::Light;
    airtab::PickResult best{};
    int32_t bestLocal = -1;
    if (p.foot == air::Foot::Any)
    {
        float bestCost = 1e30f;
        for (const airtab::Foot ft : {airtab::Foot::L, airtab::Foot::R})
        {
            const airtab::PickResult r = g_airTable.Pick(kind, dir, band, weight, ft);
            if (r.meta < 0)
                continue;
            const airtab::ClipMeta& m = g_airTable.Meta()[size_t(r.meta)];
            const int32_t local = AirEntryLocal(m, p);
            const float c = (best.meta == r.meta) ? bestCost : AirLegCostLocked(m.firstFrame + uint32_t(local));
            if (best.meta < 0 || c < bestCost)
            {
                best = r;
                bestCost = c;
                bestLocal = local;
            }
        }
    }
    else
    {
        best = g_airTable.Pick(kind, dir, band, weight, p.foot == air::Foot::L ? airtab::Foot::L : airtab::Foot::R);
        if (best.meta >= 0)
            bestLocal = AirEntryLocal(g_airTable.Meta()[size_t(best.meta)], p);
    }
    if (best.meta < 0 || bestLocal < 0)
    {
        ++g_airPlayFails;
        return;
    }
    const airtab::ClipMeta& m = g_airTable.Meta()[size_t(best.meta)];
    const int32_t global = int32_t(m.firstFrame) + bestLocal;
    if (!g_matcher.ForcePlay(global, g_airDec.halflife))
    {
        ++g_airPlayFails;
        return;
    }
    ++g_airPlays;
    g_airPlayMeta = best.meta;
    g_airPlayLocal = bestLocal;
    if (g_trace && g_log)
    {
        static constexpr const char* kActions[] = {"none", "takeoff", "off", "fallLoop", "land", "handBack", "reset"};
        const std::string_view cn = g_db.ClipName(uint32_t(m.clip));
        char line[320];
        std::snprintf(line, sizeof line,
                      "UTPP MMAIRCLIP action=%s clip=%.*s entry=%d/%u frame=%d hl=%.2f relaxed=%u rate=%.2f ttl=%.2f touch=%d",
                      kActions[static_cast<int>(a)], static_cast<int>(cn.size()), cn.data(), bestLocal, m.numFrames, global,
                      static_cast<double>(g_airDec.halflife), best.relaxed, static_cast<double>(g_airDec.rate), static_cast<double>(g_airDec.ttl),
                      m.touch); // AIR CLIPS repair: touch= appended (clip-local touch frame, -1 none)
        g_log(line);
    }
}

// On a confirmed touchdown a predicted land clip may have run past its contact frame. Re-align only once,
// only on LandApproach -> Recover and only when the same Land clip is still playing. A new late PlayLand is
// handled by ApplyAirPlayLocked; an explicit HandBack / Reset must never be rewound.
void AlignAirLandTouchLocked()
{
    const bool touchdown = g_airLandWasApproach && g_airDec.phase == air::Phase::Recover;
    g_airLandWasApproach = g_airDec.phase == air::Phase::LandApproach;
    if (!touchdown || g_airDec.action != air::Action::None || g_airDec.effectiveMode != air::Mode::Clips ||
        !g_airTableValid || !g_matcher.Ready() || g_airPlayMeta < 0 || size_t(g_airPlayMeta) >= g_airTable.Meta().size())
        return;
    const airtab::ClipMeta& m = g_airTable.Meta()[size_t(g_airPlayMeta)];
    if (m.kind != airtab::Kind::Land || m.touch < 1 || uint32_t(m.touch) >= m.numFrames)
        return;
    const MatcherDiagnostics& d = g_matcher.Diagnostics();
    if (d.clip != m.clip || d.clipFrame <= m.touch + 2)
        return;
    const int32_t from = d.clipFrame;
    // A slower inertialized transition avoids a hard pose snap when the currently sampled Land pose is already deep.
    const float hl = std::max(0.15f, g_air.Config().landHalflife);
    if (g_matcher.ForcePlay(int32_t(m.firstFrame) + m.touch, hl))
    {
        ++g_airLandRealigns;
        if (g_trace && g_log)
        {
            char line[160];
            std::snprintf(line, sizeof line, "UTPP MMAIRALIGN landTouch clip=%d from=%d touch=%d hl=%.2f", m.clip, from, m.touch,
                          static_cast<double>(hl));
            g_log(line);
        }
    }
}

void UpdateAirLocked(const FeedInput& aIn, int aPsmDetail, float aDt, float aCx, float aCy, bool aStrafe)
{
    const int64_t t0 = Qpc();
    ++g_airFeeds;
    air::AirInput ai;
    ai.dt = aDt;
    ai.groundStale = !aIn.haveAir;
    ai.onGround = aIn.haveAir ? aIn.onGround : true;
    ai.distToGround = (aIn.haveAir && std::isfinite(aIn.distToGround)) ? aIn.distToGround : -2.0f;
    ai.psmFallPrev = aIn.haveAir ? aIn.psmFall : 0; // previous-frame PSM.Fall (see FeedInput)
    ai.psmDetail = aPsmDetail;
    ai.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
    ai.planarSpeed = std::isfinite(g_rawSpeed) ? g_rawSpeed : 0.0f;
    ai.velBodyX = aCx;
    ai.velBodyY = aCy;
    ai.strafeActive = aStrafe;
    {
        const float mx = std::isfinite(aIn.move[0]) ? aIn.move[0] : 0.0f, my = std::isfinite(aIn.move[1]) ? aIn.move[1] : 0.0f;
        ai.stickMagnitude = std::fmin(1.0f, std::sqrt(mx * mx + my * my));
    }
    const bool usable = g_enabled && g_dbLoaded && g_matcher.Ready() && PoseAvailableLocked();
    // CAR-1 / CAR-2: inside the exit window the latched exit workspot is no release (the post-exit fall belongs to MM air)
    ai.release = !aIn.tppActive || !usable || (aIn.psmBlock & ExitReleaseMask(g_exitLeft > 0.0f, g_exitLatchedPsm)) != 0 ||
                 !(g_holdActive || g_effGate == 0);
    ai.vehicleExit = g_exitLeft > 0.0f; // CAR-2: planar speed clamp after a jump-out (AirConfig::vehicleExitMaxSpeed)
    // after a ragdoll (section 5, same rule as the v2 MapContext::ragdollRecover): a small drop to the floor is no fall
    if (ragdoll::RagdollRecoverActive() && ai.psmDetail != 18 /* Jump */ && ai.vz < 1.5f && ai.vz > -4.0f /* kContactFallMs: a real fall is faster by the last 0.5 m */ && aIn.haveAir && std::isfinite(aIn.distToGround) &&
        aIn.distToGround >= 0.0f && aIn.distToGround <= 0.5f)
    {
        ai.onGround = true;
        ai.psmFallPrev = 0;
    }
    if (aIn.haveTarget && std::isfinite(aIn.targetYawDeg) && std::isfinite(aIn.playerYawDeg)) // AIR CLIPS stage: turn on landing
        ai.stickTurnDeg = std::remainder(aIn.targetYawDeg - aIn.playerYawDeg, 360.0f);
    ai.tableValid = g_airTableValid; // AIR CLIPS stage: a valid .airz for the loaded database (else AirState runs Hold)
    ai.currentFrame = g_matcher.Ready() ? g_matcher.Diagnostics().frame : -1;
    AirClipFeedbackLocked(ai); // AIR CLIPS stage: remaining rise / descend time and root speed of the forced clip
    g_air.SetMode(g_airSetting == 3 ? air::Mode::Off : (g_airSetting == 2 ? air::Mode::Hold : air::Mode::Clips)); // AIR CLIPS stage
    const uint64_t landsBefore = g_air.Stats().lands;
    g_airDec = g_air.Update(ai);
    const bool physicalHoldTouch = g_air.Stats().lands > landsBefore && g_airDec.effectiveMode == air::Mode::Hold &&
                                   g_airDec.action == air::Action::HandBack && aIn.haveAir && aIn.onGround && !ai.release;
    if (physicalHoldTouch)
        g_airHoldGroundReplantLeft = kAirHoldGroundReplantSeconds;
    else if (g_airSetting == 3 || ai.release || !aIn.onGround || g_airDec.action == air::Action::Reset)
        g_airHoldGroundReplantLeft = 0.0f;
    else
        g_airHoldGroundReplantLeft = std::max(0.0f, g_airHoldGroundReplantLeft - std::clamp(aDt, 0.0f, 0.05f));
    g_airOwns = g_airDec.airOwns;
    if (!aIn.synthetic)
        g_ttlAtFeed = g_airDec.ttlKnown ? g_airDec.ttl : -1.0f; // r3b landing: Tick() compares it with the silence after this feed
    g_airOwnsPrev = g_airOwns; // read by the NEXT feed's strafe decision
    if (g_airOwns)
        ++g_airOwnedFeeds;
    if (aIn.haveAir)
    {
        ++g_airFreshFeeds;
        if (std::isfinite(aIn.distToGround) && aIn.distToGround >= -1.5f)
        {
            ++g_airProbes;
            if (aIn.distToGround >= 0.0f)
                ++g_airProbeHits;
        }
    }
    g_airLastAirborne = aIn.haveAir ? !aIn.onGround : PsmIsAirDetail(aPsmDetail);
    g_airLastVz = ai.vz;
    g_airLastRelease = ai.release; // hold repair: the probe serves no purpose while the air area cannot own the body
    if (!g_airOwns && g_airDec.phase == air::Phase::Ground && ai.currentFrame >= 0 && std::fabs(g_matcher.RootZAt(ai.currentFrame)) < 0.005f)
        g_airLastCleanFrame = ai.currentFrame; // AIR CLIPS stage
    if (g_airDec.action == air::Action::HandBack || g_airDec.action == air::Action::Reset)
        g_airPlayMeta = -1;
    g_airUs = static_cast<double>(Qpc() - t0) * 1.0e6 / QpcFreq();
    g_airUsMax = std::fmax(g_airUsMax, g_airUs);
    if (g_airDec.phase != g_airPrevPhase || g_airDec.action == air::Action::Reset || g_airDec.action == air::Action::HandBack)
    {
        if (g_trace && g_log)
        {
            static constexpr const char* kActions[] = {"none", "takeoff", "off", "fallLoop", "land", "handBack", "reset"};
            const air::AirStats& st = g_air.Stats();
            char line[400];
            std::snprintf(line, sizeof line,
                          "UTPP MMAIR phase=%s was=%s action=%s mode=%s detail=%d onG=%d stale=%d vz=%.2f dist=%.2f fall=%d ttl=%.2f "
                          "known=%d own=%d contact=%.2f rate=%.2f ep=%u to=%u falls=%u lands=%u hb=%u resets=%u",
                          air::PhaseName(g_airDec.phase), air::PhaseName(g_airPrevPhase), kActions[static_cast<int>(g_airDec.action)],
                          g_airDec.effectiveMode == air::Mode::Off ? "off" : (g_airDec.effectiveMode == air::Mode::Hold ? "hold" : "clips"),
                          aPsmDetail, aIn.haveAir ? (aIn.onGround ? 1 : 0) : -1, aIn.haveAir ? 0 : 1, static_cast<double>(ai.vz),
                          static_cast<double>(ai.distToGround), ai.psmFallPrev, static_cast<double>(g_airDec.ttl), g_airDec.ttlKnown ? 1 : 0,
                          g_airOwns ? 1 : 0, static_cast<double>(g_airDec.contactScale), static_cast<double>(g_airDec.rate), st.episodes,
                          st.takeoffs, st.falls, st.lands, st.handbacks, st.resets);
            g_log(line);
        }
        g_airPrevPhase = g_airDec.phase;
    }
}

void SetMotionHooks(const MotionHooks* aHooks) // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    std::lock_guard lock(g_mutex); // UTPP-MOTION SEAM
    g_motion = aHooks; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM
MotionSeamProbe ProbeMotionSeam() // UTPP-MOTION SEAM
{ // UTPP-MOTION SEAM
    std::lock_guard lock(g_mutex); // UTPP-MOTION SEAM
    MotionSeamProbe p; // UTPP-MOTION SEAM
    p.ready = g_matcher.Ready(); // UTPP-MOTION SEAM
    if (p.ready) // UTPP-MOTION SEAM
    { // UTPP-MOTION SEAM
        const MatcherDiagnostics& d = g_matcher.Diagnostics(); // UTPP-MOTION SEAM
        p.clip = d.clip; // UTPP-MOTION SEAM
        p.frame = d.frame; // UTPP-MOTION SEAM
        p.updates = d.updates; // UTPP-MOTION SEAM
        p.searches = d.searches; // UTPP-MOTION SEAM
        p.searched = d.searchedThisUpdate; // UTPP-MOTION SEAM
        p.maxOffsetAngle = d.maxOffsetAngle; // UTPP-MOTION SEAM
        p.maxOffsetDist = d.maxOffsetDist; // UTPP-MOTION SEAM
    } // UTPP-MOTION SEAM
    p.bindSign = g_bindSign; // UTPP-MOTION SEAM
    p.bodyFollowDir = g_bodyFollowDir; // UTPP-MOTION SEAM
    p.motionOwned = g_motionHeldMatcher; // UTPP-MOTION SEAM
    p.handbacks = g_motionHandbacks; // UTPP-MOTION SEAM
    p.turnCommitted = g_turnCommitted; // UTPP-MOTION SEAM
    p.settling = g_settling; // UTPP-MOTION SEAM
    p.stanceUsed = g_stanceUsed; // UTPP-MOTION SEAM
    p.legMotionSet = g_legCtx.v2; // UTPP-MOTION SEAM
    p.legStance = g_legCtx.stance; // UTPP-MOTION SEAM
    p.swivelSuppressed = g_stanceGuard.SwivelSuppressed(); // UTPP-MOTION SEAM
    p.strafeOn = g_strafeMode.On(); // UTPP-MOTION SEAM
    p.strafeExits = g_strafeMode.Exits(); // UTPP-MOTION SEAM
    p.chestOut = g_chestOut; // UTPP-MOTION SEAM
    p.legAirborne = g_legCtx.airborne; // UTPP-MOTION SEAM x AIR (merge b4)
    p.legContactScale = g_legCtx.contactScale; // UTPP-MOTION SEAM
    p.airOwns = g_airOwns; // UTPP-MOTION SEAM
    p.airPhase = static_cast<int>(g_air.CurrentPhase()); // UTPP-MOTION SEAM
    p.airResets = g_air.Stats().resets; // UTPP-MOTION SEAM
    p.airPlays = g_airPlays; // UTPP-MOTION SEAM
    p.airQueryOverrides = g_airQueryOverrides; // UTPP-MOTION SEAM x AIR (merge repair)
    p.airQueryFrame = g_motionProbeQueryFrame; // UTPP-MOTION SEAM
    p.frameRootZ = p.ready && p.frame >= 0 && g_matcher.HasRootZ() && std::fabs(g_matcher.RootZAt(p.frame)) >= 0.005f; // UTPP-MOTION SEAM
    p.airTableValid = g_airTableValid; // UTPP-MOTION SEAM
    return p; // UTPP-MOTION SEAM
} // UTPP-MOTION SEAM

// 2026-10-28 mount start diagnostic (log only, one line per ride): the vehtrans root compensation never fired on bike mounts
// (rootOnlyWrites=0 in 5/5 rides) while the entity moved about 1 m / 75 deg onto the seat slot. The line shows the 'before'
// pose BeginMount took (g_vtPrevPos of the previous feed), the feed at the begin (MountEvent: g_lastFeed; feed path: the
// begin feed) and the next 3 feeds with the planar distance / dz / yaw from 'before' - whether the snap is already inside
// 'before' (H2) or arrives in later samples (per-sample threshold vs cumulative). No behaviour change.
struct VtMountDiag
{
    uint64_t ride = 0;   // g_vtMounts the line belongs to (0 = none yet)
    int n = -1;          // samples after the begin (-1 = idle, 3 = logged)
    bool fromEvent = false;
    bool beforeOk = false;
    float before[3] = {0.0f, 0.0f, 0.0f};
    float beforeYaw = 0.0f;
    int64_t at = 0;      // BeginMount ticks
    int len = 0;
    char buf[512] = {};
};
static VtMountDiag g_vtMountDiag;
static bool g_vtMountDiagEvent = false; // set by MountEvent right before its BeginMount
static bool g_vtMountDiagEvOk = false;
static float g_vtMountDiagEvPos[3] = {0.0f, 0.0f, 0.0f};
static float g_vtMountDiagEvYaw = 0.0f;

static bool VtDiagPosOk(const FeedInput& aIn)
{
    return aIn.haveAir && std::isfinite(aIn.capsulePos[0]) && std::isfinite(aIn.capsulePos[1]) && std::isfinite(aIn.capsulePos[2]) &&
           std::isfinite(aIn.playerYawDeg);
}

// MountEvent call site: the feed at the begin is g_lastFeed
static void VtMountDiagEvent(const FeedInput& aIn)
{
    g_vtMountDiagEvent = true;
    g_vtMountDiagEvOk = VtDiagPosOk(aIn);
    for (int k = 0; k < 3; ++k)
        g_vtMountDiagEvPos[k] = aIn.capsulePos[k];
    g_vtMountDiagEvYaw = aIn.playerYawDeg;
}

// snprintf result of an append at d.buf + d.len (room VtMountDiagRoom()): the length stays inside the buffer
static size_t VtMountDiagRoom() { return sizeof g_vtMountDiag.buf - static_cast<size_t>(g_vtMountDiag.len); }
static void VtMountDiagAdvance(int aWritten)
{
    if (aWritten > 0)
        g_vtMountDiag.len = std::min(g_vtMountDiag.len + aWritten, static_cast<int>(sizeof g_vtMountDiag.buf) - 1);
}

// 'before' pose capture site, called BEFORE g_vtPrevPos is overwritten with this feed (it still holds BeginMount's 'before')
static void VtMountDiagFeed(const FeedInput& aIn, bool aPosOk, bool aVeh, int64_t aNow)
{
    auto& d = g_vtMountDiag;
    const auto yawDelta = [](float aA, float aB) {
        float y = std::fmod(aA - aB, 360.0f);
        if (y > 180.0f)
            y -= 360.0f;
        else if (y < -180.0f)
            y += 360.0f;
        return y;
    };
    if (g_vtMounts != 0 && g_vtMounts != d.ride)
    {
        // a BeginMount ran since the last feed: in this feed (feed path) or at a MountEvent before it
        d.ride = g_vtMounts;
        d.n = 0;
        d.at = g_vtMountTicks;
        d.fromEvent = g_vtMountDiagEvent;
        g_vtMountDiagEvent = false;
        d.beforeOk = g_vtPrevPosOk;
        for (int k = 0; k < 3; ++k)
            d.before[k] = g_vtPrevPos[k];
        d.beforeYaw = g_vtPrevYaw;
        const bool curOk = d.fromEvent ? g_vtMountDiagEvOk : aPosOk;
        const float* cur = d.fromEvent ? g_vtMountDiagEvPos : aIn.capsulePos;
        const float curYaw = d.fromEvent ? g_vtMountDiagEvYaw : aIn.playerYawDeg;
        d.len = 0;
        VtMountDiagAdvance(std::snprintf(
            d.buf, VtMountDiagRoom(),
            "UTPP vehtrans mountdiag: ride=%llu src=%s before=(%.2f,%.2f,%.2f) yaw=%.1f ok=%d cur=(%.2f,%.2f,%.2f) yaw=%.1f ok=%d",
            static_cast<unsigned long long>(d.ride), d.fromEvent ? "event" : "feed", static_cast<double>(d.before[0]),
            static_cast<double>(d.before[1]), static_cast<double>(d.before[2]), static_cast<double>(d.beforeYaw), d.beforeOk ? 1 : 0,
            static_cast<double>(cur[0]), static_cast<double>(cur[1]), static_cast<double>(cur[2]), static_cast<double>(curYaw),
            curOk ? 1 : 0));
        if (!d.fromEvent)
            return; // this feed is the begin feed ('cur'); the samples are the next feeds
    }
    if (d.n < 0 || d.n >= 3)
        return;
    const float ms = vehtrans::detail::Seconds(aNow - d.at, vehtrans::TicksPerSecond()) * 1000.0f;
    ++d.n;
    if (aPosOk)
    {
        const float dx = aIn.capsulePos[0] - d.before[0], dy = aIn.capsulePos[1] - d.before[1];
        VtMountDiagAdvance(std::snprintf(d.buf + d.len, VtMountDiagRoom(), " s%d=+%.0fms veh=%d d=%.2fm dz=%+.2f dyaw=%+.1f", d.n,
                                         static_cast<double>(ms), aVeh ? 1 : 0, static_cast<double>(std::sqrt(dx * dx + dy * dy)),
                                         static_cast<double>(aIn.capsulePos[2] - d.before[2]),
                                         static_cast<double>(yawDelta(aIn.playerYawDeg, d.beforeYaw))));
    }
    else
        VtMountDiagAdvance(std::snprintf(d.buf + d.len, VtMountDiagRoom(), " s%d=+%.0fms veh=%d pos=-", d.n, static_cast<double>(ms),
                                         aVeh ? 1 : 0));
    if ((d.n >= 3 || ms > 1000.0f) && g_log)
    {
        g_log(d.buf);
        d.n = 3;
    }
}

// vehicle batch W1 P2 (CAR-5 / MOTO-3): mount cut -> vehtrans::BeginMount with the last published snapshot (still the last
// VISIBLE MM frame: called before the cut's PublishLocked(0)) and the entity transform of the previous feed (before the
// workspot jump). CaptureMappedGamePose needs every snapshot bone mapped; the bones below firstBone (Root / Trajectory, never
// written by the transition) take their game index from the bone-0 read map.
void VtBeginMountLocked(const FeedInput& aIn, int64_t aNow)
{
    UTPP_IF_MUT(VEHEXIT, 6)
        return; // mutation 6: no vehicle transition wiring (the old mount cut / exit fade-in)
    static PoseSnapshot frozen; // game thread, g_mutex held (large: not on the stack)
    frozen = g_snap;
    for (uint32_t b = 0; b < frozen.firstBone && b < kMaxMMBones; ++b)
        if (frozen.gameIndex[b] < 0 && g_readMap.ok)
            frozen.gameIndex[b] = g_readMap.gameIndex[b];
    const bool curOk = aIn.haveAir && std::isfinite(aIn.capsulePos[0]) && std::isfinite(aIn.capsulePos[1]) &&
                       std::isfinite(aIn.capsulePos[2]) && std::isfinite(aIn.playerYawDeg);
    const float* pos = g_vtPrevPosOk ? g_vtPrevPos : (curOk ? aIn.capsulePos : nullptr);
    const float yaw = g_vtPrevPosOk ? g_vtPrevYaw : (curOk ? aIn.playerYawDeg : 0.0f);
    // mutation 9: no roles (per-bone 25 deg fallback, pre-fix)
    // review fix (vehdll VehicleTransition:366): leg-chain / Hips roles of THIS snapshot from the game bone names its own map
    // points at (v1 database or v2 bundle alike: the bone indices differ, the game names do not)
    UTPP_IF_NOT_MUT(VEHEXIT, 9)
        vehtrans::SetBoneRoles(vehtrans::BuildBoneRolesFromGame(frozen, [](int32_t aGi) -> uint64_t {
            return aGi >= 0 && static_cast<size_t>(aGi) < g_gameHashes.size() ? g_gameHashes[static_cast<size_t>(aGi)] : 0ull;
        }));
    // vehicle class of the MountingEvent (MountEvent): only a fresh hint counts, it is used once
    const bool car = g_vtMountCar && vehtrans::detail::Seconds(aNow - g_vtMountCarTicks, vehtrans::TicksPerSecond()) < kVtMountCarHintSeconds;
    g_vtMountCar = false;
    vehtrans::BeginMount(frozen, pos, yaw, aNow, car);
    g_vtWorldOk = pos != nullptr;
    g_vtMounted = true;
    g_vtRide = true;
    ++g_vtMounts;
    g_vtMountTicks = aNow;
    g_vtRideLogged = false;
    g_vtSeatLogged = false;
    {
        const auto& vc = vehtrans::g_counters;
        g_vtBase[0] = vc.holdWrites.load(); g_vtBase[1] = vc.offsetWrites.load(); g_vtBase[2] = vc.rootOnlyWrites.load();
        g_vtBase[3] = vc.perspectiveAborts.load(); g_vtBase[4] = vc.rigAborts.load(); g_vtBase[5] = vc.expired.load();
        g_vtBase[6] = vc.seatFires.load(); g_vtBase[7] = vc.seatWrites.load();
        g_vtBase[8] = vehtrans::g_neckCalls.load(); g_vtBase[9] = vehtrans::g_neckRigSkips.load();
        g_vtBase[10] = vc.neckFires.load(); g_vtBase[11] = vc.neckWrites.load();
    }
    g_vtExitBase = vehtrans::SnapCounters();
    g_vtNoCapBase = g_vtExitsNoCapture;
    g_vtFillBase = g_vtExitsFilledNoCapture;
    g_vtLastRideTpp = false;
    // mutation 15: the flag only from the vehicle bit (pre-fix)
    // MountEvent (the MountingEvent) runs before the vehicle bit: the mounted-TPP flag has to be set at the cut already, a CET
    // denial can arrive before the first vehicle feed
    UTPP_IF_NOT_MUT(VEHEXIT, 15)
        vehtrans::SetMountTpp(aIn.tppActive);
}

// Death, scene, rig change, mount cut switched off: the transition / capture / hand-in end at once.
void VtAbortLocked()
{
    if (g_vtMounted || g_vtExitPending || vehtrans::HandInActive() || vehtrans::Wanted())
        ++g_vtAborts;
    vehtrans::Abort();
    g_vtMounted = g_vtExitPending = false;
}

// CAR-1 EXITDIAG (vehicle batch; always on, not per frame): from the vehicle-bit falling edge for kExitDiagSeconds or until MM
// is fully back on the open gate, one line per change of (gate, effective gate, PSM bits, detail, loco state, TPP, hold
// decision with its first failing clause, v1 / v2 owner, weight bucket {0, <1, 1}, mask reasons, upper-body source), at most
// kExitDiagMaxLines per exit. Lets the next recording tell which reason keeps the body on the game pose after an exit.
void ExitDiagLocked(const FeedInput& aIn, int32_t aRawLoco, int aPsmDetail, float aDt, float aW)
{
    if (g_exitDiagT < 0.0f)
        return;
    const float t = g_exitDiagT;
    g_exitDiagT += (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
    const bool done = (aW >= 1.0f && g_gate == 0) || t >= kExitDiagSeconds;
    const char* v2 = g_legCtx.v2 ? "run" : ((g_motion && g_motion->step) ? "yield" : "off");
    const bool upperGame = g_maskBlend.Weight() > 0.0f;
    char key[200];
    std::snprintf(key, sizeof key, "gate=0x%X eff=0x%X psm=0x%X det=%d loco=%d tpp=%d hold=%d(%s) motion=%s w=%s mask=0x%X upper=%s latch=0x%X",
                  g_gate, g_effGate, aIn.psmBlock, aPsmDetail, aRawLoco, aIn.tppActive ? 1 : 0, g_holdActive ? 1 : 0,
                  g_holdWhy ? g_holdWhy : "ok", v2, aW <= 0.0f ? "0" : (aW < 1.0f ? "<1" : "1"), g_maskReasons, upperGame ? "game" : "mm",
                  g_exitLatchedPsm);
    if (g_log && g_exitDiagLines < kExitDiagMaxLines && (done || std::strcmp(key, g_exitDiagKey) != 0))
    {
        char line[300];
        std::snprintf(line, sizeof line, "UTPP EXITDIAG +%.2fs %s exitLeft=%.2f%s", static_cast<double>(t), key, static_cast<double>(g_exitLeft),
                      done ? " end" : "");
        if (done)
        {
            // 2026-10-28 19-55-01 (diag c9): this ride's vehtrans counters (since the mount cut) - captures / hand-in, exit latch,
            // pop filter reach (calls / skips / largest steps), mapping jobs, denied FkSide calls
            const vehtrans::CounterSnap c = vehtrans::SnapCounters();
            const vehtrans::CounterSnap& b = g_vtExitBase;
            auto d = [](uint64_t aNow, uint64_t aBase) { return static_cast<unsigned long long>(aNow - aBase); };
            char full[900];
            std::snprintf(full, sizeof full,
                          "%s | vt captures=%llu exitsHanded=%llu exitsStale=%llu noCapture=%llu fillNoCapture=%llu breaks=%llu holdFrames=%llu "
                          "holdPins=%llu holdEnds=%llu holdReleases=%llu pops=%llu hipsPops=%llu popWrites=%llu popCalls=%llu popSkipRig=%llu "
                          "popSkipEpoch=%llu popSkipBusy=%llu popMaxStep=%.1fdeg popMaxHips=%.1fcm mapCalls=%llu mapRejects=%llu fkSideDenied=%llu",
                          line, d(c.captures, b.captures), d(c.exitsHanded, b.exitsHanded), d(c.exitsStale, b.exitsStale),
                          d(g_vtExitsNoCapture, g_vtNoCapBase), d(g_vtExitsFilledNoCapture, g_vtFillBase), d(c.exitBreaks, b.exitBreaks),
                          d(c.exitHoldFrames, b.exitHoldFrames), d(c.exitHoldPins, b.exitHoldPins), d(c.exitHoldEnds, b.exitHoldEnds),
                          d(c.exitHoldReleases, b.exitHoldReleases), d(c.pops, b.pops), d(c.hipsPops, b.hipsPops), d(c.popWrites, b.popWrites),
                          d(c.popCalls, b.popCalls), d(c.popSkipRig, b.popSkipRig), d(c.popSkipEpoch, b.popSkipEpoch),
                          d(c.popSkipBusy, b.popSkipBusy), static_cast<double>(vehtrans::g_popMaxStepDeg.load()),
                          static_cast<double>(vehtrans::g_popMaxHipsCm.load()), d(c.mapCalls, b.mapCalls), d(c.mapRejects, b.mapRejects),
                          d(c.fkSideDenied, b.fkSideDenied));
            g_log(full);
        }
        else
            g_log(line);
        ++g_exitDiagLines;
    }
    std::snprintf(g_exitDiagKey, sizeof g_exitDiagKey, "%s", key);
    if (done)
        g_exitDiagT = -1.0f;
}

void Feed(const FeedInput& aRawIn)
{
    // Ragdoll episode (ragdoll/RagdollEpisode.hpp): while the engine owns V's body the feed carries the existing game-owned state of
    // knockdown (LocomotionDetailed 29, integ::PsmDetailReleases). The gate closes, HoldBlocker reports release:detail and the normal
    // gate fader fades MM out and, after the episode, back in through the reopen hold; v2 yields on the same detail (psm-release).
    // Idle: the flag is false and aRaw is aRawIn itself, nothing changes.
    std::optional<FeedInput> ragdollFeed;
    if (ragdoll::GameOwned())
    {
        ragdollFeed = aRawIn;
        ragdollFeed->psmBlock = ragdoll::GameOwnedPsmBlock(aRawIn.psmBlock);
    }
    else if (ragdoll::GetUpOwns())
    {
        // native get-up (ragdoll/GetUpPlayer.hpp): the knockdown detail the script injects is removed again, so the gate can open and MM
        // (v1 weight filled below) owns V while the clip is published. Vehicle / death / scene bits stay and still close the gate.
        ragdollFeed = aRawIn;
        ragdollFeed->psmBlock = ragdoll::GetUpPsmBlock(aRawIn.psmBlock);
    }
    const FeedInput& aRaw = ragdollFeed ? *ragdollFeed : aRawIn;
    std::lock_guard lock(g_mutex);
    if (aRawIn.synthetic && g_synthGuardOn)
    {
        g_synthGuardOn = false;
        if (g_lastRealFeedTicks != g_synthGuardTicks)
            return; // a real locomotion update arrived between Tick()'s decision and this lock: the stale synthetic sample must not follow it
    }
    const FeedDumpScope dumpScope(aRaw); // batch 1b
    ++g_feedCalls;
    if (!aRawIn.synthetic)
        ragdoll::g_recoverWindow.Step(PsmDetailOf(aRawIn.psmBlock), aRawIn.dt); // section 5: opens when the knockdown detail ends
    g_motionHandbackFeed = false;
    g_chestValid = false; // B2 strafe S2: set by the leg pipeline of this feed
    g_lastFeed = aRaw;
    // 0.5.3 stick filter: half-stick magnitudes count as held, short releases (180 deg flick) are bridged
    FeedInput conditioned = aRaw;
    if (g_stickFilterOn)
        g_stickHeld = g_stick.Update(conditioned.move[0], conditioned.move[1], conditioned.haveTarget, conditioned.targetYawDeg,
                                     aRaw.dt);
    else
        g_stick.Reset();
    // 0.5.6: reds 0.5.6 reports PSM.LocomotionDetailed in bits 16..23 of the block mask (diagnostic only)
    const int psmDetail = PsmDetailOf(conditioned.psmBlock);
    // 0.6.0: melee bits (kGateMelee through g_meleeGate) and the UpperBody / MeleeWeapon diagnostic fields
    g_meleeBits = PsmMeleeBits(conditioned.psmBlock);
    g_meleeState = PsmMeleeStateOf(conditioned.psmBlock);
    g_upperBody = PsmUpperBodyOf(conditioned.psmBlock);
    const bool prevWeaponKnown = g_weaponKnown, prevWeaponHeld = g_weaponHeld;
    g_weaponKnown = PsmWeaponKnown(conditioned.psmBlock);
    g_weaponHeld = PsmWeaponDrawn(conditioned.psmBlock);
    conditioned.psmBlock = PsmBlockBits(conditioned.psmBlock);
    bool jumpOutEdge = false; // this feed is the jump-out launch (the exit edge)
    // 2026-10-28 bike jump-out (GameIntegration.hpp JumpOutLatch): the PSM vehicle bit that outlives the unmount through the flight
    // is no ride any more - stripped once here, so the mount cut, capture, exit edge, hold, air release and upper mask all agree
    {
        const bool wasJumpOut = g_jumpOut.Active();
        const float vzIn = std::isfinite(conditioned.velWorld[2]) ? conditioned.velWorld[2] : 0.0f;
        if (g_jumpOut.Update(conditioned.psmBlock, vzIn, conditioned.tppActive, aRaw.dt))
        {
            conditioned.psmBlock &= ~uint32_t(kPsmVehicle);
            ++g_jumpOutFeeds;
            jumpOutEdge = !wasJumpOut;
            if (jumpOutEdge)
                ++g_jumpOutLaunchEpoch;
            if (!wasJumpOut && g_log) // always on (info), once per jump-out
            {
                char line[200];
                std::snprintf(line, sizeof line,
                              "UTPP JUMPOUT vehicle bit released (flight after the unmount): vz=%.2f %.3f s after the workspot drop, onGround=%d -> exit edge now",
                              static_cast<double>(vzIn), static_cast<double>(g_jumpOut.SinceDrop()),
                              conditioned.haveAir ? (conditioned.onGround ? 1 : 0) : -1);
                g_log(line);
            }
        }
    }
    const FeedInput& aIn = conditioned;
    const int64_t now = Qpc();
    g_lastFeedTicks = now;
    if (!aRaw.synthetic)
    {
        g_lastRealFeedTicks = now;
        g_landDropout = false; // the game feeds again
    }
    else
        ++g_landDropoutFeeds;

    float dt = (std::isfinite(aIn.dt) && aIn.dt > 0.0f) ? aIn.dt : 0.0f;
    dt = std::fmin(dt, 0.25f);

    const float yaw = std::isfinite(aIn.playerYawDeg) ? aIn.playerYawDeg * kDegToRad : 0.0f;
    float cx = 0.0f, cy = 0.0f;
    WorldToCharacter(std::isfinite(aIn.velWorld[0]) ? aIn.velWorld[0] : 0.0f,
                     std::isfinite(aIn.velWorld[1]) ? aIn.velWorld[1] : 0.0f, yaw, cx, cy);
    g_lastCharVel[0] = cx;
    g_lastCharVel[1] = cy;
    // 0.5.8 speed cap: a capsule faster than factor x the game sprint speed (dash / dodge bursts 10-17.6 m/s in game 0.5.7) is
    // unsupported (gate bit kGateOverSpeed: normal fade to 0, reopen hold); the query velocity never exceeds the sprint speed.
    g_rawSpeed = std::sqrt(cx * cx + cy * cy);
    g_overSpeed = false;
    if (g_speedCapFactor > 0.0f)
    {
        const SpeedPoints& sm = g_speedMap[kLocoStand];
        const float sprint = (sm.count >= 3 && sm.game[2] > 0.0f) ? sm.game[2] : g_stateTargetSpeed[kLocoSprint];
        g_overSpeed = g_rawSpeed > g_speedCapFactor * sprint;
        if (g_overSpeed)
            ++g_overSpeedFeeds;
        if (g_rawSpeed > sprint)
        {
            const float k = sprint / g_rawSpeed;
            cx *= k;
            cy *= k;
            ++g_queryClamped;
        }
    }
    float yawRate = 0.0f;
    const float yawStep = g_havePrevYaw ? WrapPi(yaw - g_prevYaw) : 0.0f;
    if (g_havePrevYaw && dt > 0.0f)
        yawRate = WrapPi(yaw - g_prevYaw) / dt;
    g_haveTarget = aIn.haveTarget && std::isfinite(aIn.targetYawDeg);
    g_targetYaw = g_haveTarget ? WrapPi(aIn.targetYawDeg * kDegToRad) : 0.0f;
    g_bodyTurnFed = false;
    g_airSteerFed = false; // AIR STEER stage (set at the mf.bodyTurn line)
    const float capsuleSpeed = std::sqrt(cx * cx + cy * cy);
    // Phase 5.3c: steering mode from the RAW target (animation-led only for sharp changes). In follow mode the latch is
    // bypassed (continuous steering incl. camera steering; the camera does not follow the body since 0.3.1 booking).
    // B2 strafe S1: aim strafe owns the legs (no steering / follow / latch; the reds fallback target is fed but ignored)
    const bool strafeS = UpdateStrafeLocked(aIn, psmDetail, dt);
    bool animLed = true; // 0.3.4: every body turn animation-led
    if (g_steerOn && g_bodyTurnOn && !strafeS)
        animLed = g_steer.Update(g_haveTarget, g_targetYaw, yaw, capsuleSpeed, dt);
    else
        g_steer.Reset();
    if (strafeS)
        animLed = false;
    g_followFed = g_steerOn && g_bodyTurnOn && g_haveTarget && !animLed && !strafeS;
    if (g_followFed || strafeS)
        g_latch.Reset();
    // Target latch (0.3.1): while a turn is in progress (|latched target - body| > kLatchTurnDeg) and the stick angle
    // relative to the camera is unchanged (< kLatchStickDeg), keep the latched world target, so camera motion during the
    // turn cannot drag the target (0.3.0 feedback: body turn -> camera turn -> target turn). Otherwise follow the input.
    if (!g_followFed && !strafeS)
        g_targetYaw = g_latch.Update(g_haveTarget, g_targetYaw,
                                     std::isfinite(aIn.cameraYawDeg) ? aIn.cameraYawDeg * kDegToRad : yaw, yaw);
    yawRate = std::fmax(-12.0f, std::fmin(12.0f, yawRate));
    g_prevYaw = yaw;
    g_havePrevYaw = true;
    g_lastYawRate = yawRate;

    // Stance-dependent clip set (only when no manual filter is set): switch without resetting playback.
    int32_t locoForGate = aIn.locoState;
    if (g_tagFilter.empty() && g_dbLoaded && g_matcher.Ready())
    {
        const uint64_t want = (aIn.locoState > 0 && aIn.locoState < 4) ? g_stateTagMask[aIn.locoState] : 0ull;
        if (want == 0)
            locoForGate = kLocoOther; // no clips for this state -> fade out to the game pose
        else if (want != g_appliedTagMask && g_matcher.SetAllowedTagMask(want))
            g_appliedTagMask = want;
    }

    GateInput gi = BaseGateLocked();
    gi.tppActive = aIn.tppActive;
    gi.locoState = locoForGate;
    gi.psmBlock = aIn.psmBlock;
    g_psmBlock = aIn.psmBlock;
    if (aIn.psmBlock != 0)
    {
        ++g_psmBlockFeeds;
        g_lastPsmDetail = psmDetail;
    }
    g_rawGate = ComputeGate(gi);
    if (g_overSpeed)
        g_rawGate |= kGateOverSpeed; // 0.5.8: not a soft reason -> no debounce
    if (CandidateReadyLocked())
        g_rawGate |= kGateNotMapped; // 0.5.1: database switch pending -> hard fade-out, swap at weight 0
    // 0.6.0 melee gate: whole-body melee action (debounced like a crouch blip, + release hold) -> driver bit, normal fade
    g_meleeActive = g_meleeGate.Update(g_meleeBits, dt, g_meleeCfg, g_gateDebounceCfg.holdSeconds, g_meleeState >= kPsmMeleeWeaponIdle);
    if (g_meleeActive)
        g_rawGate |= kGateMelee;
    // 0.6.0 mount cut: the vehicle bit hands the body to the mount animation on this very sample (no 0.12 s fade-out with the
    // standing MM pose over the bike - game 0.5.9, 11:57:04.896: w 1.00 -> 0.93 -> 0.86 after the vehicle bit); kept muted until
    // the vehicle state ends, then the normal reopen hold + fade-in.
    {
        const bool vehBit = g_mountCutOn && (aIn.psmBlock & kPsmVehicle) != 0;
        // MountingEvent latch: muted until the vehicle bit is fed (then the bit rules) or kMountEventHold has passed
        g_mountEventLeft = (vehBit || !g_mountCutOn) ? 0.0f : std::fmax(0.0f, g_mountEventLeft - dt);
        const bool veh = vehBit || g_mountEventLeft > 0.0f;
        if (veh)
        {
            if (!g_mountMuted && g_weight.Weight() > 0.0f)
                ++g_mountCuts;
            // mutation 7: no ride latch - any abort restarts the mount transition mid-ride (pre-fix)
            if (UTPP_TERN(VEHEXIT, 7, !g_vtMounted, !g_vtRide)) // review fix (vehdll MMDriver:3307): once per ride, never again after an abort
                VtBeginMountLocked(aIn, now); // vehicle batch W1 P2: before the cut publishes weight 0
            g_weight.Cut();
        }
        g_mountMuted = veh;
        // vehicle batch W1 P2: entity jump watch (first kWorldWatchSeconds), live game pose capture while mounted in TPP; death /
        // scene during the ride or the hand-in end the transition; a MountingEvent that never got the vehicle bit ends it too
        const bool posOk = aIn.haveAir && std::isfinite(aIn.capsulePos[0]) && std::isfinite(aIn.capsulePos[1]) &&
                           std::isfinite(aIn.capsulePos[2]) && std::isfinite(aIn.playerYawDeg);
        const bool vtBlocked = (aIn.psmBlock & (kPsmScene | kPsmVitals)) != 0;
        if (vtBlocked && (g_vtMounted || vehtrans::HandInActive() || (g_vtRide && vehtrans::Wanted())))
            VtAbortLocked();
        else if (!vtBlocked && g_vtRide)
        {
            if (g_vtMounted && veh && posOk && g_vtWorldOk)
                vehtrans::UpdateWorld(aIn.capsulePos, aIn.playerYawDeg, now);
            vehtrans::SetVehicleCapture(vehBit && aIn.tppActive, g_rig); // also re-enabled after an abort earlier in the ride
            // the UTPP third-person body is mounted: the vehicle hold's pose-write veto (CET, 5-10 ms after the cut) must not end the
            // mount transition (hold + decaying offsets) - recording 2026-10-28 05:05:24: hold frame, then the raw game pose
            vehtrans::SetMountTpp(UTPP_TERN(VEHEXIT, 15, vehBit, veh) && aIn.tppActive);
            vehtrans::SetPopFilter(veh && aIn.tppActive); // 2026-10-28 11-49-37: one-frame jumps of the game's seated / exit pose
            if (veh)
                g_vtLastRideTpp = aIn.tppActive; // the edge feed has the flags off already (vehtrans::ExitWantsFill)
            // one line per ride, kVtMountLogSeconds after the mount cut: did the mount transition run (hold / offset writes) or was it
            // ended; the standing entry gate, the leg rule and the cumulative root tracking (2026-10-28 19-55-01)
            if (!g_vtRideLogged && g_log && vehtrans::detail::Seconds(now - g_vtMountTicks, vehtrans::TicksPerSecond()) >= kVtMountLogSeconds)
            {
                g_vtRideLogged = true;
                const auto& vc = vehtrans::g_counters;
                char line[1024];
                const float alignAt = vehtrans::g_rootAlignAt.load();
                const int capState = vehtrans::g_capState.load();
                std::snprintf(line, sizeof line,
                              "UTPP vehtrans mount: ride=%llu holdWrites=%llu offsetWrites=%llu rootOnlyWrites=%llu perspectiveAborts=%llu "
                              "rigAborts=%llu expired=%llu (this ride) mountTpp=%d active=%d gateHipsZ=%.3f standing=%d legCap=%.0f "
                              "legOffsetBones=%u legsDroppedByCap=%u legMoveAt=%.3f rootEngageAt=%.3f rootPeakYaw=%.1f rootPeakDist=%.2f yawAlign=%d alignAt=%.3f car=%d "
                              "holdCap=%.3f capState=%d(%s) capArriveAt=%.3f capStartAt=%.3f capTail=%.3f capExtraIgnored=%d jumpEngageAt=%.3f jumpPeakDist=%.2f jumpReleaseEnd=%.3f",
                              static_cast<unsigned long long>(g_vtMounts), static_cast<unsigned long long>(vc.holdWrites.load() - g_vtBase[0]),
                              static_cast<unsigned long long>(vc.offsetWrites.load() - g_vtBase[1]),
                              static_cast<unsigned long long>(vc.rootOnlyWrites.load() - g_vtBase[2]),
                              static_cast<unsigned long long>(vc.perspectiveAborts.load() - g_vtBase[3]),
                              static_cast<unsigned long long>(vc.rigAborts.load() - g_vtBase[4]),
                              static_cast<unsigned long long>(vc.expired.load() - g_vtBase[5]), vehtrans::g_mountTpp.load() ? 1 : 0,
                              vehtrans::MountActive() ? 1 : 0, static_cast<double>(vehtrans::g_mountGateHipsZ.load()),
                              vehtrans::g_mountStandingEpoch.load() != 0 ? 1 : 0, static_cast<double>(vehtrans::g_mountLegCapDeg.load()),
                              vehtrans::g_mountLegOffsetBones.load(), vehtrans::g_mountLegsDropped.load(),
                              static_cast<double>(vehtrans::g_mountLegMoveAt.load()), static_cast<double>(vehtrans::g_rootEngageAt.load()),
                              static_cast<double>(vehtrans::g_rootPeakYawDeg.load()), static_cast<double>(vehtrans::g_rootPeakDistM.load()),
                              alignAt >= 0.0f ? 1 : 0, static_cast<double>(alignAt),
                              vehtrans::g_mountAlignCar.load() ? 1 : 0, static_cast<double>(vehtrans::g_capSeconds.load()), capState,
                              capState == 0 ? "none" : capState == 1 ? "received, not needed" : capState == 2 ? "applied cumulative" :
                              capState == 3 ? "applied jump" : capState == 5 ? "refused bike" : capState == 6 ? "dropped old" : capState == 8 ? "late" : "ignored",
                              static_cast<double>(vehtrans::g_capArriveAt.load()), static_cast<double>(vehtrans::g_capStartAt.load()),
                              static_cast<double>(vehtrans::g_capTailUsed.load()), vehtrans::g_capSecondIgnored.load(), static_cast<double>(vehtrans::g_jumpEngageAt.load()),
                              static_cast<double>(vehtrans::g_jumpPeakDistM.load()), static_cast<double>(vehtrans::g_jumpReleaseEnd.load()));
                g_log(line);
            }
            // seat pose switch guard (VehicleTransition SEAT POSE SWITCH): one line per ride once its window is over
            if (!g_vtSeatLogged && g_log &&
                vehtrans::detail::Seconds(now - g_vtMountTicks, vehtrans::TicksPerSecond()) >= vehtrans::kSeatWatchSeconds + 0.5f)
            {
                g_vtSeatLogged = true;
                const auto& vc = vehtrans::g_counters;
                const unsigned long long fires = vc.seatFires.load() - g_vtBase[6];
                // neckFire / neckStep / neckEdge / neckCalls / neckRigSkips: the seated neck cut (VehicleTransition SEATED NECK CUT,
                // FK-stage NeckGuard) - fired, the largest neck-chain step above its rate inside the edge window (the fire step when it
                // fired), the seating edge (frozen frame, seconds after the mount cut, -1 = none seen), guard calls past its gates and
                // rig mismatches this ride. fires / writes / hipsStep / torsoStep / at / mountTppAtFire are the seat pose switch guard
                // alone; the neck cut has its own neckFires / neckWrites / neckHipsStep / neckTorsoStep / neckAt (appended at the end:
                // a parser of the older keys reads them unchanged, only a neck fire no longer shows up as fires=1 with the neck's Hips step)
                const unsigned long long neckFires = vc.neckFires.load() - g_vtBase[10];
                char line[512];
                std::snprintf(line, sizeof line,
                              "UTPP vehtrans seat: ride=%llu fires=%llu writes=%llu hipsStep=%.1fcm torsoStep=%.1fdeg at=%.2fs (after the "
                              "mount cut) mountTppAtFire=%d mountTpp=%d armed=%d neckFire=%d neckStep=%.1fdeg neckEdge=%.2fs neckCalls=%llu "
                              "neckRigSkips=%llu neckFires=%llu neckWrites=%llu neckHipsStep=%.1fcm neckTorsoStep=%.1fdeg neckAt=%.2fs",
                              static_cast<unsigned long long>(g_vtMounts), fires,
                              static_cast<unsigned long long>(vc.seatWrites.load() - g_vtBase[7]),
                              fires ? static_cast<double>(vehtrans::g_seatLastHipsCm.load()) : 0.0,
                              fires ? static_cast<double>(vehtrans::g_seatLastTorsoDeg.load()) : 0.0,
                              fires ? static_cast<double>(vehtrans::g_seatLastAt.load()) : 0.0,
                              fires ? vehtrans::g_seatLastMountTpp.load() : -1, vehtrans::g_mountTpp.load() ? 1 : 0,
                              vehtrans::SeatGuardArmed() ? 1 : 0, vehtrans::g_seatNeckFired.load(),
                              static_cast<double>(vehtrans::g_seatNeckStepDeg.load()),
                              static_cast<double>(vehtrans::g_seatNeckEdgeAt.load()),
                              static_cast<unsigned long long>(vehtrans::g_neckCalls.load() - g_vtBase[8]),
                              static_cast<unsigned long long>(vehtrans::g_neckRigSkips.load() - g_vtBase[9]), neckFires,
                              static_cast<unsigned long long>(vc.neckWrites.load() - g_vtBase[11]),
                              neckFires ? static_cast<double>(vehtrans::g_neckLastHipsCm.load()) : 0.0,
                              neckFires ? static_cast<double>(vehtrans::g_neckLastTorsoDeg.load()) : 0.0,
                              neckFires ? static_cast<double>(vehtrans::g_neckLastAt.load()) : -1.0);
                g_log(line);
            }
            if (!veh && (aIn.psmBlock & kPsmVehicle) == 0 && !g_prevVeh)
            {
                VtAbortLocked(); // MountingEvent expired without a ride
                g_vtRide = false;
            }
        }
        VtMountDiagFeed(aIn, posOk, veh, now); // 2026-10-28 mount start diagnostic (log only; g_vtPrevPos = BeginMount's 'before')
        g_vtPrevPosOk = posOk;
        if (posOk)
        {
            for (int k = 0; k < 3; ++k)
                g_vtPrevPos[k] = aIn.capsulePos[k];
            g_vtPrevYaw = aIn.playerYawDeg;
        }
    }
    // 0.5.6: a single-sample soft closure (loco state / detailed locomotion, no vertical motion) does not close the gate
    const bool wasPending = g_gateDebounce.Pending();
    g_gate = g_gateDebounce.Update(g_rawGate, aIn.psmBlock, std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f, dt,
                                   g_gateDebounceCfg);
    const bool debouncing = g_gateDebounce.Pending();
    if (debouncing && !wasPending)
        g_lastSoftDetail = psmDetail;
    if (debouncing && (aIn.locoState < kLocoStand || aIn.locoState > kLocoSprint) && g_lastGoodLoco != 0)
        conditioned.locoState = g_lastGoodLoco; // the matcher logic keeps the stance of the debounced blip
    if (g_gate == 0 && aIn.locoState >= kLocoStand && aIn.locoState <= kLocoSprint)
        g_lastGoodLoco = aIn.locoState;
    // 0.6.2 upper-body mask: for the masked states (melee gate, UpperBody Aim, vehicle exit window) the gate does not close the
    // whole body - MM keeps the lower body (normal stand / walk query) and the upper-body bones fade to the game pose. Every
    // other closure (vehicle ride, scene, workspot, swim, ladder, ragdoll, dash over-speed, fall...) still fades the whole body.
    LogUpperMaskLocked();
    {
        const bool vehNow = (aIn.psmBlock & kPsmVehicle) != 0;
        if (vehNow)
        {
            g_exitLeft = 0.0f;
            g_exitLatchedPsm = 0;
        }
        else if (g_prevVeh)
        {
            g_exitLeft = kExitWindowSeconds;
            ++g_exitWindows;
            // CAR-1: latch the exit workspot set on this falling-edge feed (the vanilla exit workspot); it stays latched while set
            g_exitLatchedPsm = aIn.psmBlock & kPsmWorkspot;
            // CAR-1 EXITDIAG: one diagnosis per exit, from this falling edge
            g_exitDiagT = 0.0f;
            g_exitDiagLines = 0;
            g_exitDiagKey[0] = '\0';
            // vehicle batch W1 P2 (MOTO-6 / CAR-1 step 5): a fresh capture of the game's exit pose -> MM at full weight at once
            // (after the hold decision below), handed in from the captured pose by HandInBlend in PublishLocked
            if (g_vtRide) // review fix: also after an abort earlier in the ride (the capture ran again from the next feed)
            {
                g_vtMounted = g_vtRide = false;
                // mutation 30: no pinned hold of the edge capture at the jump-out launch
                // r6 launch: a jump-out edge pins the shown pose until the hand-in snapshot is published (a normal exit passes false: unchanged)
                g_vtExitPending = vehtrans::BeginExit(now, &g_vtExitSeedSnap, UTPP_AND_NOT(VEHEXIT, 30) jumpOutEdge);
                if (!g_vtExitPending)
                    ++g_vtExitsNoCapture;
                // 2026-10-28 19-55-01: a third-person ride under the vehicle hold (denied animation frames this ride) without a capture
                // snaps MM in below (no 0.25 s fade through the game's post-exit squat); a jump-out has the live exit capture instead
                g_vtExitFillPending = vehtrans::ExitWantsFill(g_vtExitPending, g_vtLastRideTpp, jumpOutEdge,
                                                              vehtrans::g_counters.fkSideDenied.load() > g_vtExitBase.fkSideDenied);
                g_vtLastRideTpp = false;
            }
            // mutation 12: no live exit capture at the jump-out launch
            // 2026-10-28 bike jump-out: no fresh ride capture reached the edge (recording 05-03-09: handIn 0 on every exit) - the game pose
            // of the next animation frame becomes the hand-in source (PromoteLiveExit below), so there is neither a fade-in through the
            // game flight pose nor a snap to MM at weight 1
            if (UTPP_AND_NOT(VEHEXIT, 12) jumpOutEdge && !g_vtExitPending)
                vehtrans::BeginLiveExit(now);
            // the game pose after an exit is the vanilla FPP set (crouch-like): no reopen hold, the lower body fades in at once
            if (g_upperMaskOn && g_upperMask.ok && (g_maskStates & kMaskStateExit))
                g_weight.ClearHold();
        }
        else
        {
            // mutation 29: synthetic landing feeds run the exit window down / close it
            if (UTPP_OR(VEHEXIT, 29) !aIn.synthetic) // the synthetic landing feed leaves the window as it is: the next real feed (VeryHardLand) still finds it open
                g_exitLeft = std::fmax(0.0f, g_exitLeft - dt);
            if ((aIn.psmBlock & kPsmWorkspot) == 0)
                g_exitLatchedPsm &= ~uint32_t(kPsmWorkspot); // the latched exit workspot ended: a later one is a NEW workspot
            // MM fully back on the open gate: the window ends, the upper body fades back to MM. The RAW gate (not the effective
            // gate after the hold) - a held exit workspot keeps the window (and its latch) until it ends or the window times out.
            const uint32_t endGate = UTPP_TERN(VEHEXIT, 2, g_effGate, g_gate); // mutation 2: window ends on the effective (held) gate
            if (endGate == 0 && g_weight.Progress() >= 1.0f && (UTPP_OR(VEHEXIT, 29) !aIn.synthetic))
                g_exitLeft = 0.0f;
        }
        if (g_exitLeft <= 0.0f)
            g_exitLatchedPsm = 0;
        g_jumpOutSlewLeft = jumpOutEdge ? mm::kJumpOutSlewSeconds : std::fmax(-mm::kJumpOutSlewTailSeconds, g_jumpOutSlewLeft - dt);
        if (jumpOutEdge)
        {
            g_jumpOutExit = true; // the exit window of this feed opened at the launch
            g_jumpOutHoldT = g_jumpOutGroundT = 0.0f;
            g_jumpOutGroundN = 0;
        }
        else if (vehNow)
            g_jumpOutExit = false;
        else if (g_exitLeft > 0.0f)
        {
            g_jumpOutHoldT = g_jumpOutGroundT = 0.0f;
            g_jumpOutGroundN = 0;
        }
        else if (g_jumpOutExit && !aIn.synthetic)
        {
            // the window is closed but the jump-out is not over: until ground contact has lasted kJumpOutHoldConfirmS (and the capsule is
            // no longer in a fast fall) the state stays - only this flag (and with it the v2 exit window / jump-out flight); g_exitLeft,
            // the upper mask and the planar speed clamp see the closed window as before
            const float vzIn = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
            // without air info (haveAir false) the PSM detail stands in for the ground state, so a rising or apex feed can pass as contact: it needs vz <= +kJumpOutHoldRiseMs
            const bool contact = (!aIn.haveAir || aIn.onGround) && vzIn > -kJumpOutHoldFallMs && (aIn.haveAir || vzIn <= kJumpOutHoldRiseMs);
            g_jumpOutHoldT += dt;
            g_jumpOutGroundT = contact ? g_jumpOutGroundT + dt : 0.0f;
            g_jumpOutGroundN = contact ? g_jumpOutGroundN + 1 : 0;
            // mutation 22: the closed window ends the jump-out state at once (pre-r3 landing)
            if (UTPP_OR(VEHEXIT, 22)
                (g_jumpOutGroundT >= kJumpOutHoldConfirmS && g_jumpOutGroundN >= kJumpOutHoldFeeds) || g_jumpOutHoldT >= kJumpOutHoldMaxS)
                g_jumpOutExit = false;
        }
        g_prevVeh = vehNow;
        UpperMaskInput mi;
        mi.gate = g_gate;
        mi.psm = aIn.psmBlock;
        mi.psmDetail = psmDetail;
        mi.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
        mi.meleeActive = g_meleeActive;
        mi.upperBody = g_upperBody;
        mi.exitWindow = g_exitLeft > 0.0f;
        mi.states = g_maskStates;
        mi.maxVertical = g_gateDebounceCfg.maxVertical;
        mi.tpp = aIn.tppActive; // CAR-1 / MOTO-6: the TPP exit window keeps the upper body on MM unless a weapon is in hand
        mi.weaponAction = g_weaponUpperOn && g_weaponKnown && g_weaponHeld;
        const UpperMaskDecision md = (g_upperMaskOn && g_upperMask.ok) ? DecideUpperMask(mi) : UpperMaskDecision{};
        g_maskUpperFromGame = md.upperFromGame;
        if (md.reasons != 0 && g_maskReasons == 0)
            ++g_maskSwitches;
        if (md.reasons != 0)
            ++g_maskFeeds;
        if (g_trace && g_log && md.reasons != g_maskReasons)
        {
            // melee full body fields (2026-11-09, appended): state of the previous feed (the decision runs after the matcher)
            char line[256];
            std::snprintf(line, sizeof(line), "UTPP MMMASK %s reasons=0x%X (was 0x%X) gate=0x%X covered=0x%X loco=%d upperBody=%d melee=%d exit=%.2f w=%.2f mask=%.2f full=%.2f fullWant=%d fullWhy=%s",
                          md.reasons != 0 ? "on" : "off", md.reasons, g_maskReasons, g_gate, md.covered, aIn.locoState, g_upperBody,
                          g_meleeActive ? 1 : 0, static_cast<double>(g_exitLeft), static_cast<double>(g_weight.Weight()),
                          static_cast<double>(g_maskBlend.Weight()), static_cast<double>(g_meleeFullBlend.Weight()),
                          g_meleeFullWant ? 1 : 0, mm::melee::WhyName(g_meleeFullGate.LastWhy()));
            g_log(line);
        }
        g_maskReasons = md.reasons;
        if (md.reasons != 0)
            g_maskFadeFromAim = (md.reasons & kMaskStateAim) != 0;
        g_maskCovered = md.covered;
        g_effGate = md.reasons != 0 ? (g_gate & ~md.covered) : g_gate;
        // the legs stand / walk: an unsupported locomotion state (AimWalk, exit) queries the last stance (stand by default)
        if (md.reasons != 0 && (aIn.locoState < kLocoStand || aIn.locoState > kLocoSprint))
            conditioned.locoState = g_lastGoodLoco != 0 ? g_lastGoodLoco : kLocoStand;
    }
    // 0.6.2 hold last pose: in TPP a closure outside the states the game animates keeps the MM body (the game pose
    // would be a T-pose); no reopen hold either. Release states (vehicle, scene, workspot, swim, ladder, takedown, death...) and
    // TPP off still fade the whole body out.
    {
        g_lastDetail = psmDetail;
        HoldInput hi;
        hi.on = g_holdPoseOn;
        hi.tpp = aIn.tppActive;
        hi.poseOk = PoseAvailableLocked();
        hi.gate = g_gate;
        hi.psm = aIn.psmBlock;
        hi.psmDetail = psmDetail;
        // CLIMB RELEASE stage: Climb 8 / Vault 9 release to the game animation unless v2 plays the traversal. TRAV-1: v2 traversal
        // active (MotionHooks::travActive) releases whatever mm.climbRelease says; v1 only (no hook / false) with the setting off: 0
        {
            const bool climb = (aIn.psmBlock & kPsmDetailed) != 0 && PsmIsClimbDetail(psmDetail);
            const bool motionActive = climb && g_motion && g_motion->travActive && g_motion->travActive(); // UTPP-MOTION SEAM
            const bool motionOwns = climb && (g_climbReleaseOn || motionActive) && g_motion && g_motion->ownsTraversal && g_motion->ownsTraversal(); // UTPP-MOTION SEAM
            const uint32_t rel = TravReleases(g_climbReleaseOn, motionActive, aIn.psmBlock, psmDetail, motionOwns) ? uint32_t(kReleaseTrav) : 0u;
            if (motionOwns)
                ++g_travOwnedFeeds;
            if (rel != 0)
                ++g_travReleaseFeeds;
            if ((rel & kReleaseTrav) != (g_releaseReasons & kReleaseTrav))
            {
                if (rel & kReleaseTrav)
                {
                    ++g_travReleaseEpisodes;
                    if (g_log) // TRAV-1 D: always on (info), once per release episode edge
                    {
                        char line[96];
                        std::snprintf(line, sizeof(line), "UTPP trav: released to game animation (detail=%d)", psmDetail);
                        g_log(line);
                    }
                }
                if (g_trace && g_log)
                {
                    char line[160];
                    std::snprintf(line, sizeof(line), "UTPP MMTRAV travRelease %s detail=%d gate=0x%X motionOwns=%d w=%.2f", (rel & kReleaseTrav) ? "on" : "off",
                                  psmDetail, g_gate, motionOwns ? 1 : 0, static_cast<double>(g_weight.Weight()));
                    g_log(line);
                }
            }
            g_releaseReasons = rel;
            hi.release = rel;
            // Vault torso limiter (mm/VaultTorso.hpp): every feed steps the fade and publishes the weight for the FK detour. Gate = a vault
            // released to the game animation, over a vehicle v2 rejected, with a ranged weapon in hand. Any other feed: wanted false.
            {
                mm::vaulttorso::GateInput gi;
                gi.travRelease = (rel & kReleaseTrav) != 0;
                gi.otherRelease = (rel & ~uint32_t(kReleaseTrav)) != 0;
                gi.psmReleaseBit = (aIn.psmBlock & kPsmReleaseMask) != 0; // vehicle / scene / workspot / takedown / carry / vitals / swimming
                gi.psmDetail = psmDetail;
                gi.weaponDrawn = g_weaponHeld;
                gi.vehicleVaultRanged = g_motion && g_motion->vehicleVaultWeapon && g_motion->vehicleVaultWeapon(); // also the hold after the release ends
                static const int64_t vaultTps = static_cast<int64_t>(QpcFreq());
                const int edge = mm::vaulttorso::rt::Feed(gi, aRaw.dt, Qpc(), vaultTps);
                if (edge != 0 && g_log)
                {
                    char line[160];
                    if (edge > 0)
                        std::snprintf(line, sizeof(line), "UTPP trav: vault torso limiter on (released vault over a vehicle, weapon in hand, floor %.0f deg)",
                                      static_cast<double>(mm::vaulttorso::kFloorDeg));
                    else
                        std::snprintf(line, sizeof(line), "UTPP trav: vault torso limiter off (applied=%llu saturated=%llu minElev=%.1f maxCorr=%.1f deg)",
                                      static_cast<unsigned long long>(mm::vaulttorso::rt::g_applied.load()),
                                      static_cast<unsigned long long>(mm::vaulttorso::rt::g_saturated.load()),
                                      static_cast<double>(mm::vaulttorso::rt::g_minElevDeg.load()),
                                      static_cast<double>(mm::vaulttorso::rt::g_maxCorrDeg.load()));
                    g_log(line);
                }
            }
        }
        hi.exitWindow = g_exitLeft > 0.0f; // CAR-1: the latched exit workspot is held inside the window
        hi.exitLatchedPsm = g_exitLatchedPsm;
        g_holdWhy = HoldBlocker(hi);
        const bool eligible = g_holdWhy == nullptr; // == HoldEligible(hi)
        if (g_feedDumpOn)
            g_dumpRec.holdEligible = eligible; // batch 1b
        if (eligible)
            g_weight.ClearHold();
        const bool hold = eligible && g_effGate != 0;
        if (hold && !g_holdActive)
        {
            ++g_holdEpisodes;
            if (g_trace && g_log)
            {
                char line[160];
                std::snprintf(line, sizeof(line), "UTPP MMHOLD on gate=0x%X effGate=0x%X psm=0x%X detail=%d loco=%d w=%.2f", g_gate, g_effGate,
                              aIn.psmBlock, psmDetail, aIn.locoState, static_cast<double>(g_weight.Weight()));
                g_log(line);
            }
        }
        g_holdActive = hold;
        if (hold)
        {
            ++g_holdFrames;
            g_effGate = 0;
            if (aIn.locoState < kLocoStand || aIn.locoState > kLocoSprint)
                conditioned.locoState = g_lastGoodLoco != 0 ? g_lastGoodLoco : kLocoStand;
        }
    }
    // ragdoll native get-up: MM owns V at full weight from the first get-up feed on (the clip is published over MM's pose, no fade-in
    // through the engine's ragdoll blend-out). The gate must be open on that first feed; otherwise v2 would not publish the clip.
    if (ragdoll::GetUpOwns())
    {
        if (g_effGate == 0 && aIn.tppActive)
            g_weight.Fill();
        else if (!g_getUpBaseSeen)
            ragdoll::RequestGetUpAbort(ragdoll::kGetUpAbortBaseGate);
        g_getUpBaseSeen = true;
    }
    else
        g_getUpBaseSeen = false;
    // vehicle batch W1 P2: exit hand-in only where MM keeps the body (open / held gate, TPP): weight 1 on this very feed (no
    // fade-in through the live post-exit game squat); otherwise the capture is dropped and the normal fade applies
    if (g_vtExitPending)
    {
        g_vtExitPending = false;
        g_vtExitFillPending = false;
        if (g_effGate == 0 && aIn.tppActive)
        {
            g_weight.Fill();
            ++g_vtExitsHanded;
            // first v2 search seeds its pose channels from this capture (normal exits only: the jump-out launch keeps the mean query)
            if (!jumpOutEdge)
                g_vtExitSeedFeed = g_feedCalls;
        }
        else
            VtAbortLocked();
    }
    else if (g_vtExitFillPending)
    {
        g_vtExitFillPending = false;
        if (g_effGate == 0 && aIn.tppActive)
        {
            g_weight.Fill();
            ++g_vtExitsFilledNoCapture;
        }
    }
    else if (vehtrans::LiveExitPending())
    {
        // the live exit capture (BeginLiveExit): MM stays on the weight ramp (game pose visible) until the animation thread delivered the
        // game pose of a post-edge frame; then MM at weight 1 + the inertialized offset game pose -> MM (the ordinary hand-in)
        const int live = vehtrans::PromoteLiveExit(now);
        if (live < 0)
            vehtrans::EndRearmHold(); // no capture within kLiveExitWaitSeconds: MM again (no-op without a re-arm hold)
        else if (live > 0 && vehtrans::RearmHold())
            g_jumpOutRearmPromoted = true;
        if (live > 0)
        {
            if (g_effGate == 0 && aIn.tppActive)
            {
                g_weight.Fill();
                ++g_vtExitsHanded;
                ++g_jumpOutFills;
            }
            else
                VtAbortLocked();
        }
    }
    UpdateAirLocked(aIn, psmDetail, dt, cx, cy, strafeS); // AIR HOLD stage (H1): after the hold block, before the matcher block
    // B2 strafe: AimWalk (unsupported locomotion state 0) is a standing aim - the legs query the stand clip set, not the last
    // stance (after a crouch the 0.6.2 mask path kept the crouch set while standing; crouch aim keeps locoState 2)
    if (strafeS && (aRaw.locoState < kLocoStand || aRaw.locoState > kLocoSprint))
    {
        conditioned.locoState = g_strafeStance == strafe::kStanceCrouch ? kLocoCrouch : kLocoStand;
        if (g_tagFilter.empty() && g_dbLoaded && g_matcher.Ready())
        {
            const uint64_t want = g_stateTagMask[conditioned.locoState];
            if (want != 0 && want != g_appliedTagMask && g_matcher.SetAllowedTagMask(want))
                g_appliedTagMask = want;
        }
    }
    if (g_gate != 0 && g_gatePrev == 0)
        ++g_gateCloses;
    // CLIMB RELEASE stage: a release reason change also writes the line (jump -> climb keeps the gate bits); none while off
    if (g_trace && g_log && (g_gate != g_gatePrev || g_releaseReasons != g_releaseTracePrev))
    {
        char line[400];
        std::snprintf(line, sizeof(line),
                      "UTPP MMGATE gate=0x%X (was 0x%X) psm=0x%X detail=%d loco=%d tpp=%d w=%.2f debounced=%llu melee=0x%X meleeState=%d "
                      "meleeHold=%d upperBody=%d mountCut=%d mask=0x%X hold=%d release=0x%X%s",
                      g_gate, g_gatePrev, aIn.psmBlock, psmDetail, aIn.locoState, aIn.tppActive ? 1 : 0,
                      static_cast<double>(g_weight.Weight()), static_cast<unsigned long long>(g_gateDebounce.Suppressed()), g_meleeBits,
                      g_meleeState, (g_meleeActive && g_meleeBits == 0) ? 1 : 0, g_upperBody, g_mountMuted ? 1 : 0, g_maskReasons,
                      g_holdActive ? 1 : 0, g_releaseReasons, (g_releaseReasons & kReleaseTrav) ? " travRelease" : "");
        g_log(line);
    }
    g_gatePrev = g_gate;
    g_releaseTracePrev = g_releaseReasons;

    if (g_enabled && g_dbLoaded && g_matcher.Ready())
    {
        // Desired velocity = measured capsule velocity (character frame); facing = character forward turning at the
        // measured player yaw rate (YawRate mode).
        // Speed mapping into animation space (see SetSpeedMap): velocity scaled to database speeds, matcher time
        // scaled by rate so distance per real second is unchanged; yaw rate is per animation second as well.
        // Desired motion (character frame). Input-driven: direction from the stick relative to the camera; speed is
        // the capsule speed, but at least half the state's target speed so starts and pivots are anticipated. The
        // desired facing is the input direction (the body turns towards movement). Without input: capsule
        // velocity (deceleration) and the current forward.
        // Phase 5.1: with a valid world target (Locomotion) and the kill switch on, the desired facing / velocity
        // direction is the target and the trajectory yaw follows the measured body (driven by UTPPMMBodyYawCommand).
        bt::MotionFrame mf;
        mf.charVel[0] = cx;
        mf.charVel[1] = cy;
        mf.move[0] = std::isfinite(aIn.move[0]) ? aIn.move[0] : 0.0f;
        mf.move[1] = std::isfinite(aIn.move[1]) ? aIn.move[1] : 0.0f;
        mf.cameraYaw = std::isfinite(aIn.cameraYawDeg) ? aIn.cameraYawDeg * kDegToRad : yaw;
        mf.bodyYaw = yaw;
        mf.bodyTurn = g_bodyTurnOn && g_haveTarget && !strafeS; // B2 strafe: the CET aim align keeps sole ownership of V's yaw
        // UTPP-MOTION SEAM slide feed: a v2-owned slide (mm.slide + slide bundle, PSM detail 5) keeps the facing the game gave it; the
        // body-yaw command would turn the body towards the stick target against the sliding capsule (BodyYawCommand: pass-through)
        if (mf.bodyTurn && g_motion && g_motion->ownsSlide && g_motion->ownsSlide(psmDetail))
            mf.bodyTurn = false;
        if (g_airOwns)
        {
            // AIR STEER stage: the body-yaw command steers in the air (mf.bodyTurn stays false: no turn intent / clips)
            g_airSteerFed = g_airSteerOn && mf.bodyTurn && !g_motionHeldMatcher && g_rawSpeed >= kAirSteerMinSpeed &&
                            std::isfinite(aIn.velWorld[0]) && std::isfinite(aIn.velWorld[1]) &&
                            g_effGate == 0;
            if (g_airSteerFed)
            {
                g_airSteerVelocityYaw = WrapPi(std::atan2(-aIn.velWorld[0], aIn.velWorld[1]));
                ++g_airSteerFeeds;
            }
            mf.bodyTurn = false; // AIR HOLD stage (H2): no turn intent in the air
        }
        mf.targetYaw = g_targetYaw;
        mf.follow = g_followFed;
        mf.facingYaw = g_followFed ? g_steer.FacingYaw(g_targetYaw, yaw) : g_targetYaw;
        mf.turnRate = g_followFed ? g_steer.TargetRate() : 0.0f;
        bt::MotionParams mp;
        mp.inputDriven = g_inputDriven;
        mp.inputCameraRelative = g_inputCameraRelative;
        mp.stateTargetSpeed = g_stateTargetSpeed[(aIn.locoState >= 0 && aIn.locoState < 4) ? aIn.locoState : 0];
        // Phase 5.3: full input intent while the script lowers the capsule acceleration (last AccelTarget query).
        mp.intentFraction = (g_accelOn && g_accelLastTarget > 0.0f && g_accelLastTarget < g_accelBase) ? 1.0f : 0.5f;
        mp.stopIntent = g_stopSyncOn && g_inputDriven;
        mp.startReface = g_startRefaceOn;
        // B2 strafe S1: stick direction in the body frame (capsule direction while Exiting), speed max(capsule, aim intent x
        // |stick|), facing the body, Direction mode at turn rate 0
        bt::MotionParams mpS = mp;
        if (strafeS)
        {
            mpS.intentFraction = 1.0f;
            // S4 jog: intent = the forward jog speed F in every direction (like the flat aim walk intent). The capsule target is
            // F x JogProfile (reds multiplier; MapSpeed point below). A per-direction intent F x JogProfile was not better
            // (review jog_repair, 8 directions 60 Hz: agreement 3.0 97.7 vs 99.4 %, 3.85 98.0 vs 98.8 %, 3.3 98.3 vs 99.1 %;
            // steady foot-lock 3.85 7.1 vs 8.3 cm, 3.3 8.5 vs 8.3 cm; 8 s cruise loops 413 vs 472 of 472 at 3.0).
            mpS.stateTargetSpeed = g_strafeGait == strafe::kGaitRun ? g_strafeJogF : kStrafeAimWalk;
        }
        const bt::DesiredMotion dm = strafeS ? strafe::StrafeDesiredMotion(mf, mpS, g_strafeMode.State()) : bt::ComputeDesiredMotion(mf, mp);
        g_bodyTurnFed = mf.bodyTurn;
        const bool releasedBt = g_prevBtFed && !mf.bodyTurn; // 0.3.8: body-turn path ended this feed
        g_prevBtFed = mf.bodyTurn;
        const float dx = dm.vel[0], dy = dm.vel[1];
        const float speed = std::sqrt(dx * dx + dy * dy);
        float scale = 1.0f, rate = 1.0f;
        LoopRateParams lp;
        lp.rateMin = g_rateMin;
        lp.rateMax = g_rateMax;
        lp.walkFloor = g_slowWalkOn;
        lp.walkRateMin = g_walkRateMin;
        const SpeedPoints& smap = g_speedMap[(aIn.locoState >= 0 && aIn.locoState < 4) ? aIn.locoState : 1];
        const bool stick = std::sqrt(mf.move[0] * mf.move[0] + mf.move[1] * mf.move[1]) > 0.15f;
        const bool wasSlow = g_slowGateActive;
        g_slowGateActive = g_slowCapOn && g_slowWalkOn && g_slowGate.Update(capsuleSpeed, stick, speed, SlowWalkThreshold(smap), dt);
        if (strafeS) // B2 strafe: slow-walk gate bypassed while Active / Exiting (strafe::StrafeBypassesSlowWalkGate)
        {
            g_slowGateActive = false;
            g_slowGate.Reset();
        }
        // 0.5.3: playback-rate factor of the playing clip (source family / own loop speed), smoothed (0.1 s halflife)
        {
            const int32_t playing = g_matcher.Ready() ? g_matcher.Diagnostics().clip : -1;
            g_rateFactorTarget = (g_familyRateOn && playing >= 0 && size_t(playing) < g_famRates.clipFactor.size())
                                     ? g_famRates.clipFactor[size_t(playing)]
                                     : 1.0f;
            const float k = dt > 0.0f ? 1.0f - std::exp(-0.69314718f * dt / 0.1f) : 1.0f;
            g_rateFactor += (g_rateFactorTarget - g_rateFactor) * k;
        }
        const int locoIdx = (aIn.locoState >= 0 && aIn.locoState < 4) ? aIn.locoState : 1;
        const float slowQuery = g_familyRateOn ? g_slowRef[locoIdx] : 0.0f;
        if (!(g_slowCapOn && g_slowWalkOn))
            g_slowGate.Reset();
        g_strafeRateSrc = 0;
        if (strafeS)
        {
            // B2 strafe S1: query speed = the per-direction loop speed map (game aim point -> DB loop speed S(theta)); rate =
            // capsule / own speed of the playing facing-fixed loop (stick held), else game speed / mapped DB speed; clamped to
            // [rateMin, rateMax] and smoothed (0.1 s half-life). No slow-walk floor, no family factor.
            const float th = speed > 0.05f ? std::atan2(-dx, dy) : 0.0f;
            g_strafeDirDeg = th / kDegToRad;
            // S4 jog: game point of direction theta = F x JogProfile(theta) -> the run loop S(theta) at full stick
            const float point = g_strafeGait == strafe::kGaitRun ? g_strafeJogF * strafe::JogProfile(th) : kStrafeAimWalk;
            const float db = g_strafeProf.table.MapSpeed(g_strafeStance, g_strafeGait, th, speed, point);
            float target = 1.0f;
            if (speed > 0.05f && db > 0.05f)
            {
                scale = db / speed;
                target = std::fmax(lp.rateMin, std::fmin(lp.rateMax, speed / db));
                g_strafeRateSrc = 1;
            }
            const int32_t playing = g_matcher.Diagnostics().clip;
            const float own = (playing >= 0 && size_t(playing) < g_strafeProf.loopOwnSpeed.size()) ? g_strafeProf.loopOwnSpeed[size_t(playing)] : 0.0f;
            const float ro = strafe::LoopOwnSpeedRate(capsuleSpeed, own, lp.rateMin, lp.rateMax);
            if (stick && ro > 0.0f)
            {
                target = ro;
                g_strafeRateSrc = 2;
            }
            rate = g_strafeRate.Update(target, dt);
            g_lastFloored = false;
        }
        else if (g_slowGateActive)
        {
            SlowWalkRate(smap, speed, capsuleSpeed, lp, scale, rate, g_rateFactor, slowQuery); // 5.3c / 0.5.3
            g_lastFloored = true;
        }
        else
            LoopRate(smap, speed, lp, scale, rate, &g_lastFloored, g_rateFactor, slowQuery);
        g_loopRate = rate;
        {
            float rvx = 0.0f, rvy = 0.0f;
            g_matcher.RootVelocity(rvx, rvy);
            g_rootSpeedDb = std::sqrt(rvx * rvx + rvy * rvy);
        }
        if (g_rootRateOn) // stop phase only (stick released): capsule vs played root; otherwise the loop rate
            rate = g_rootRate.Update(std::sqrt(cx * cx + cy * cy), g_rootSpeedDb, dt,
                                     std::sqrt(mf.move[0] * mf.move[0] + mf.move[1] * mf.move[1]) <= 0.15f, rate);
        // 0.6.3 turn rate: the plant of a moving turn / pivot / reface plays near its own timing instead of the loop rate
        // (0.70 / 0.80 at a run: 180 deg turn 1.43 x slower than authored while the capsule kept its speed - test
        // 2026-10-12, night traces Run_Turn_R_180_Rfoot rate=0.70). Weight from the clip's root below its own peak (the
        // plant), so the run-in / run-out frames (root at the loop speed) keep the loop rate (no cadence jump, no overshoot).
        // Gate: stick held and a recent capsule speed >= 1 m/s (decaying max: a flick brakes the capsule through 0 during
        // the plant; a slow walk keeps its slow-walk rate).
        const bool turnRateWasActive = g_turnRateActive; // 0.6.4 summary
        g_turnRateActive = false;
        {
            const float kr = dt > 0.0f ? std::exp2(-dt / kTurnRateRecentHl) : 1.0f;
            g_turnRecentSpeed = std::fmax(std::isfinite(capsuleSpeed) ? capsuleSpeed : 0.0f, g_turnRecentSpeed * kr);
            if (!std::isfinite(g_turnRecentSpeed))
                g_turnRecentSpeed = 0.0f;
            float wTarget = 0.0f;
            const int32_t pc = g_matcher.Diagnostics().clip;
            const uint8_t k = (pc >= 0 && size_t(pc) < g_clipClasses.size()) ? g_clipClasses[size_t(pc)] : steer::kClassNone;
            const float peak = (pc >= 0 && size_t(pc) < g_clipPeakRoot.size()) ? g_clipPeakRoot[size_t(pc)] : 0.0f;
            if (g_turnRateOn && !strafeS && stick && g_turnRecentSpeed >= kTurnRateMinCapsule && steer::IsTurnClass(k) && peak > 1.0f && rate > 0.0f)
                wTarget = std::fmax(0.0f, std::fmin(1.0f, (peak - g_rootSpeedDb) / (kTurnRateBand * peak)));
            const float hlw = wTarget > g_turnRateW ? kTurnRateRiseHl : kTurnRateFallHl;
            const float kw = dt > 0.0f ? 1.0f - std::exp(-0.69314718f * dt / hlw) : 1.0f;
            g_turnRateW += (wTarget - g_turnRateW) * kw; // both ways: the rate returns smoothly to the loop rate
            if (!g_turnRateOn || !stick || !(g_turnRateW > 1e-3f) || !std::isfinite(g_turnRateW))
                g_turnRateW = 0.0f;
            g_turnRateTarget = std::fmax(g_turnRateMin, std::fmin(g_turnRateMax, (capsuleSpeed + kTurnRateEps) / (g_rootSpeedDb + kTurnRateEps)));
            const float raised = rate + (g_turnRateTarget - rate) * g_turnRateW;
            if (g_turnRateW > 0.0f && raised > rate)
            {
                rate = raised;
                g_turnRateActive = true;
                ++g_turnRateFeeds;
            }
            // 0.6.4: accumulate while raised; one summary line per turn when it ends (info level, not trace).
            if (g_turnRateActive)
            {
                if (g_turnAcc.frames == 0)
                    g_turnAcc.clip = pc;
                ++g_turnAcc.frames;
                g_turnAcc.rateSum += rate;
                g_turnAcc.peak = std::fmax(g_turnAcc.peak, rate);
                g_turnAcc.dur += dt > 0.0f ? dt : 0.0f;
                g_turnAcc.capsuleSum += std::isfinite(capsuleSpeed) ? capsuleSpeed : 0.0f;
                g_turnAcc.rootSum += g_rootSpeedDb;
            }
            else if (turnRateWasActive && g_turnAcc.frames > 0)
            {
                const double n = static_cast<double>(g_turnAcc.frames);
                const std::string_view cn =
                    g_turnAcc.clip >= 0 ? g_db.ClipName(static_cast<uint32_t>(g_turnAcc.clip)) : std::string_view{};
                char line[384];
                std::snprintf(line, sizeof line, "turnRate: clip=%.*s avg=%.2f peak=%.2f dur=%.2f capsule=%.2f root=%.2f",
                              int(cn.size()), cn.data() ? cn.data() : "", g_turnAcc.rateSum / n,
                              static_cast<double>(g_turnAcc.peak), g_turnAcc.dur, g_turnAcc.capsuleSum / n, g_turnAcc.rootSum / n);
                g_turnRateLastSummary = line;
                ++g_turnRateTurns;
                if (g_trace && g_log) // the summary stays in the status either way; the log line only while tracing
                    g_log(line);
                g_turnAcc = TurnRateAcc{};
            }
        }
        // 0.5.7 start rate: an animation-led start / turn from (near) standstill plays at least at its own speed. The capsule
        // speed rate (slow-walk floor 0.25, game 0.40-0.81) stretched the turning wind-up (game 03:30:13.86-03:30:14.7,
        // f 11 -> 22 in ~0.8 s; offline the 180 deg turn in place ran at rate 0.25 and could not lead the body).
        g_startRateActive = false;
        // Only while the played segment turns on the spot (root speed < 0.5 m/s database): a translating start at rate 1
        // with the capsule still at 0 only pushes the offset root to its clamp.
        if (g_startTurnOn && stick && mf.bodyTurn && animLed && capsuleSpeed < 1.0f && rate < 1.0f && g_rootSpeedDb < 0.5f)
        {
            const int32_t pc = g_matcher.Diagnostics().clip;
            const uint8_t k = (pc >= 0 && size_t(pc) < g_clipClasses.size()) ? g_clipClasses[size_t(pc)] : steer::kClassNone;
            if (steer::IsStart(k) || k == steer::kClassTurnInPlace || k == steer::kClassReface ||
                ((steer::kRefaceVariantMask >> k) & 1u) != 0)
            {
                rate = 1.0f;
                g_startRateActive = true;
                ++g_startRateFeeds;
            }
        }
        const float rateNoAir = rate; // UTPP-MOTION SEAM x AIR (merge repair): v2-owned / handback feeds step v1 without the air warp
        if (g_airOwns && g_airDec.rateIsMultiplier && g_airDec.rate != 1.0f)
            rate *= g_airDec.rate; // AIR HOLD stage: stride rate x holdRate while airborne (hold mode)
        if (g_airOwns && !g_airDec.rateIsMultiplier && g_airDec.effectiveMode == air::Mode::Clips)
            rate = g_airDec.rate; // AIR CLIPS stage: rise / descend warp of the forced air clip, recover rate of the land clip
        g_lastRate = rate;
        MatcherInput in;
        in.desiredVelocity[0] = dx * scale;
        in.desiredVelocity[1] = dy * scale;
        if (dm.direction)
        {
            in.facingMode = FacingMode::Direction;
            in.desiredFacing[0] = dm.facing[0];
            in.desiredFacing[1] = dm.facing[1];
        }
        else
        {
            in.facingMode = FacingMode::YawRate;
            in.yawRate = yawRate / rate;
        }
        if (dm.direction && dm.turnRate != 0.0f)
            in.turnRate = steer::MatcherCurve(dm.turnRate, rate, g_steer.Config()); // 5.3c curvature, matcher time (capped)
        // 5.3c dwell exemption: stick press / release, entering an animation-led turn, slow-walk gate change.
        // (no dwell for 0.5 s after a stick press / release: starts and stops stay as responsive as 0.3.4)
        g_stickEdgeTime = stick != g_prevStick ? 0.0f : std::fmin(g_stickEdgeTime + dt, 1e6f);
        {
            const int stance = aIn.locoState == kLocoCrouch ? 2 : ((aIn.locoState == kLocoStand || aIn.locoState == kLocoSprint) ? 1 : 0);
            if (stance != 0 && g_prevStance != 0 && stance != g_prevStance)
            {
                g_sinceStance = 0.0f;
                ++g_stanceChanges;
                g_stanceUsed = false; // 0.5.7: one stance transition per stance change
                // 0.6.0: a moving into-crouch transition (RunToCrouch / WalkToCrouch) only from a fast, forward-held run;
                // otherwise the change goes straight into the crouch loop / start at the matched phase (inertialized)
                if (stance == 2)
                {
                    const float mag = std::fmin(1.0f, std::sqrt(mf.move[0] * mf.move[0] + mf.move[1] * mf.move[1]));
                    g_stanceClipSpeed = g_rawSpeed;
                    g_stanceClipStickFwd = (stick && speed > 1e-3f) ? mag * std::fmax(0.0f, dy / speed) : 0.0f;
                    g_stanceClipOk = !(g_stanceClipMin > 0.0f) ||
                                     (g_rawSpeed >= g_stanceClipMin && g_stanceClipStickFwd >= kStanceClipStick);
                    if (g_stanceClipOk)
                        ++g_stanceClipAllows;
                    else
                        ++g_stanceClipSkips;
                }
                else
                    g_stanceClipOk = true;
            }
            else
                g_sinceStance = std::fmin(g_sinceStance + dt, 1e6f);
            if (stance != 0)
                g_prevStance = stance;
        }
        in.dwellExempt = g_stickEdgeTime < 0.5f || g_steer.Entered() || wasSlow != g_slowGateActive ||
                         (g_largeTurnOn && g_steer.Exited()) || // 0.3.8: quick recovery after a large turn
                         g_strafeEdge;                          // B2 strafe: mode entered / exited
        if (g_airDec.dwellExempt)
            in.dwellExempt = true; // AIR HOLD stage: hand-back at the touchdown (the first search may switch at once)
        if (g_largeTurnOn)
        {
            // 0.3.8: the body-turn angular velocity must not leak into the stop prediction (Walk_Stop at cost 422 in game)
            in.clearAngVel = releasedBt;
            if (!stick && !mf.bodyTurn && in.facingMode == FacingMode::YawRate)
                in.yawRate = std::fmax(-g_stopYawRateMax, std::fmin(g_stopYawRateMax, in.yawRate));
        }
        g_prevStick = stick;
        // 0.5.4 speed band of the motion (stand / sprint clip set only; crouch pivots / turns are not speed-classed)
        if (g_curCfg.speedBands && (aIn.locoState == kLocoStand || aIn.locoState == kLocoSprint))
            g_lastBand = g_band.Update(MapSpeedPoints(g_speedMap[kLocoStand], std::fmax(speed, capsuleSpeed)), stick,
                                       animLed && mf.bodyTurn, dt, g_bandEdges, g_bandCfg);
        else
        {
            g_band.Reset();
            g_lastBand = 0;
        }
        // 0.5.8 loop bands: from the band of the capsule speed (the speed the loop has to match) up to the band of the stick
        // intent (state speed x |stick|, at least the query speed): at a steady speed one band, while the capsule accelerates
        // towards the intent every band in between. Offline: the capsule band alone (or with the 0.5.3 half-speed query floor)
        // excluded the run moves in the flick re-acceleration at 1.2-1.9 m/s and a walk side step was taken at cost 65 (offset
        // saturated 0.52 s); the query band alone put a sprint start at 2 m/s into the run band (walk loops out). Stand /
        // sprint with the stick held; hysteresis / down hold as the 0.5.4 bands.
        if (g_loopBandsOn && strafeS && stick && (aIn.locoState == kLocoStand || aIn.locoState == kLocoSprint))
        {
            // B2 strafe: loop band = the strafe gait (walk; run with the S4 jog)
            g_loopBandTracker.Reset();
            g_loopBandHiTracker.Reset();
            g_lastLoopBand = g_lastLoopBandHi = g_strafeGait == strafe::kGaitRun ? 2 : 1;
        }
        else if (g_loopBandsOn && stick && (aIn.locoState == kLocoStand || aIn.locoState == kLocoSprint))
        {
            g_lastLoopBand =
                g_loopBandTracker.Update(MapSpeedPoints(g_speedMap[kLocoStand], capsuleSpeed), true, false, dt, g_bandEdges, g_bandCfg);
            const float intent = mp.stateTargetSpeed * std::fmin(std::sqrt(mf.move[0] * mf.move[0] + mf.move[1] * mf.move[1]), 1.0f);
            g_lastLoopBandHi =
                std::max(g_lastLoopBand, g_loopBandHiTracker.Update(MapSpeedPoints(g_speedMap[kLocoStand], std::fmax(speed, intent)), true,
                                                                    false, dt, g_bandEdges, g_bandCfg));
        }
        else
        {
            g_loopBandTracker.Reset();
            g_loopBandHiTracker.Reset();
            g_lastLoopBand = g_lastLoopBandHi = 0;
        }
        if (g_curationOn)
        {
            steer::CurationInput ci;
            ci.input = stick;
            ci.haveTarget = mf.bodyTurn;
            ci.animLed = animLed;
            ci.absErr = std::fabs(WrapPi(g_targetYaw - yaw));
            ci.capsule = capsuleSpeed;
            ci.moving = g_curMovingOn; // 0.3.6 moving-state rules (no stop / start / transition clips mid-run)
            ci.sinceInput = g_stickEdgeTime;
            ci.intent = speed;
            ci.speedClass = g_largeTurnOn ? steer::SpeedClassOfSpeed(std::fmax(speed, capsuleSpeed)) : 0; // 0.3.8
            ci.slowWalk = g_slowLoopOn && g_slowGateActive; // 0.3.9: no Walk_Start kept in a steady slow walk
            ci.sinceStance = g_sinceStance;                  // 0.5.2: stance-change transitions only after a change
            ci.turnRate = g_steer.TargetRate();              // 0.5.3: arc clips only while the target turns
            ci.speedBand = g_lastBand;                       // 0.5.4: pivots / turns of the band's class only
            ci.pivotFallback = g_bandAvail.PivotFallback(g_lastBand);
            ci.turnFallback = g_bandAvail.TurnFallback(g_lastBand);
            // 0.5.6 crouch rules: current stance, forward desired motion (character frame, +y forward, +-45 deg)
            ci.stance = aIn.locoState == kLocoCrouch ? 2 : ((aIn.locoState == kLocoStand || aIn.locoState == kLocoSprint) ? 1 : 0);
            ci.forward = stick && speed > 0.3f && dy > 0.0f && std::fabs(dx) <= dy; // |atan2(dx, dy)| <= 45 deg
            ci.lateralOk = g_lateralOk[(aIn.locoState >= 0 && aIn.locoState < 4) ? aIn.locoState : 0];
            ci.strafe = strafeS; // B2 strafe: patterns allowed, StrafeTurn out, no haveTarget pivot / turn rules
            in.excludeClasses = steer::WithVariants(steer::ExcludedClasses(ci, g_curCfg)); // 0.5.4: + reface variants
            in.excludeCurrentClasses = steer::WithVariants(steer::ExcludedCurrentClasses(ci, g_curCfg));
            if (strafeS && !g_strafePatterns) // B2 strafe internal switch: Box / Diamond / Hourglass / Prism patterns out
            {
                in.excludeClasses |= steer::Bit(steer::kClassPattern);
                in.excludeCurrentClasses |= steer::Bit(steer::kClassPattern);
            }
            // 0.5.7 start turn: slow turning starts follow the start rules; while an animation-led turn is wanted, clips turning
            // against it and slow turning starts are out (also as the playing clip)
            if (in.excludeClasses & steer::Bit(steer::kClassStart))
                in.excludeClasses |= steer::Bit(steer::kClassSlowTurnStart);
            if (in.excludeCurrentClasses & steer::Bit(steer::kClassStart))
                in.excludeCurrentClasses |= steer::Bit(steer::kClassSlowTurnStart);
            // Near reversals (>= 160 deg) are ambiguous: both directions stay candidates and the body turns with the playing clip
            // instead (BodyYawCommand), so the animation still leads (offline fresh 180 deg start: the left turn in place matched
            // at cost 6.9, the right one at 17.2).
            int wantSet = 0;
            g_bodyFollowDir = 0;
            if (g_startTurnOn && stick && mf.bodyTurn && animLed)
            {
                const float e = WrapPi(g_targetYaw - yaw);
                if (std::fabs(e) >= 20.0f * kDegToRad && std::fabs(e) < 160.0f * kDegToRad)
                    wantSet = e > 0.0f ? 1 : 2;
                else if (std::fabs(e) >= 160.0f * kDegToRad)
                {
                    const int32_t pc = g_matcher.Diagnostics().clip;
                    // UTPP-MOTION SEAM v1 matcher held by a v2 span (also the handback feed, searched afresh below): pc is stale
                    if (pc >= 0 && size_t(pc) < g_turnSign.size() && !g_motionHeldMatcher) // UTPP-MOTION SEAM
                        g_bodyFollowDir = g_turnSign[size_t(pc)];
                }
            }
            // 0.5.8 start bind: for the first kStartCommitSeconds of a bound start segment only clips turning its way are
            // candidates (the clip direction cannot flip unless the stick reverses - UpdateStartBindLocked)
            if (g_startBindOn && g_bindSign != 0 && g_bindSince < steer::kStartCommitSeconds && stick && mf.bodyTurn)
            {
                wantSet = g_bindSign > 0 ? 1 : 2;
                ++g_bindCommitSearches;
            }
            if (strafeS)
                wantSet = 3; // B2 strafe class set (the start turn / bind sets need the body-turn path, off while strafing)
            ApplyClassSetLocked(wantSet, g_lastLoopBand, g_lastLoopBandHi);
            if (g_lastLoopBand > 0)
            {
                in.excludeClasses |= steer::Bit(steer::kClassOffBand);
                ++g_loopBandSearches;
            }
            if (wantSet == 1 || wantSet == 2)
            {
                const uint32_t m = steer::Bit(steer::kClassTurnAgainst) | steer::Bit(steer::kClassSlowTurnStart);
                in.excludeClasses |= m;
                in.excludeCurrentClasses |= m;
                ++g_againstSearches;
            }
            // 0.5.7: one stance transition per stance change (the playing one continues)
            if (g_stanceOnceOn && g_stanceUsed)
                in.excludeClasses |= steer::kStanceAllMask;
            // 0.6.0: moving into-crouch transition not allowed at this stance change (speed / stick rule)
            if (!g_stanceClipOk && g_stanceClipMin > 0.0f)
            {
                in.excludeClasses |= steer::Bit(steer::kClassStanceMoveToCrouch);
                ++g_stanceClipExcl;
            }
            // 0.5.7 turn commit: while a turn-class clip is still turning towards the target (its root turn not done, same
            // direction, animation-led with the stick held) no other turn-class clip / frame is a candidate; the playing frame
            // continues unless a loop is clearly better (dwell rule). Not for turn-in-place clips: offline the commit then left
            // only bad alternatives once the capsule moved (cost 345 in the fresh 180 deg start).
            g_turnCommitted = false;
            const uint32_t preCommit = in.excludeClasses;
            if (g_turnCommitOn && stick && mf.bodyTurn && animLed && !g_motionHeldMatcher) // UTPP-MOTION SEAM no commit from a stale clip
            {
                const MatcherDiagnostics& cd = g_matcher.Diagnostics();
                if (cd.clip >= 0 && size_t(cd.clip) < g_turnEnd.size() && g_turnEnd[size_t(cd.clip)] >= 0 &&
                    cd.clipFrame < g_turnEnd[size_t(cd.clip)] && WrapPi(g_targetYaw - yaw) * float(g_turnSign[size_t(cd.clip)]) > 0.0f)
                {
                    // 0.5.8 stance cap: the playing clip must still be in the clip set - after a stand -> crouch change the
                    // standing reface (not continuable) excluded every crouch turn / reface and left WalkToCrouch at cost 224
                    // (game 0.5.7, 05:09:55.300)
                    if (g_stanceCapOn && !g_matcher.ClipAllowed(cd.clip))
                        ++g_commitNotAllowed;
                    else
                    {
                        in.excludeClasses |= steer::kTurnClassMask;
                        g_turnCommitted = true;
                        ++g_turnCommitHolds;
                        g_commitSign = g_turnSign[size_t(cd.clip)];
                    }
                }
            }
            // 0.5.8 turn settle: a committed turn clip reached 90 % of its root turn with error left on the same side -> no
            // other turn-class clip for kTurnSettleSeconds (the body-turn fallback finishes the rest; an opposite error ends it)
            g_settling = false;
            if (g_turnSettleOn && g_turnCommitOn && !g_motionHeldMatcher) // UTPP-MOTION SEAM held: no settle window from a stale commit
            {
                if (g_prevCommitted && !g_turnCommitted && g_commitSign != 0)
                {
                    g_settleLeft = steer::kTurnSettleSeconds;
                    g_settleSign = g_commitSign;
                }
                if (g_settleLeft > 0.0f && (!stick || !mf.bodyTurn || !animLed || WrapPi(g_targetYaw - yaw) * float(g_settleSign) <= 0.0f))
                    g_settleLeft = 0.0f;
                if (g_settleLeft > 0.0f && !g_turnCommitted)
                {
                    in.excludeClasses |= steer::kTurnClassMask;
                    g_settling = true;
                    ++g_settleHolds;
                }
                g_settleLeft = std::fmax(0.0f, g_settleLeft - dt);
            }
            else
                g_settleLeft = 0.0f;
            g_prevCommitted = g_turnCommitted;
            const uint32_t commitBits = (in.excludeClasses & ~preCommit) & steer::kTurnClassMask;
            // 0.5.7 standstill idle: no stick (or MM gated out) and the capsule stands -> idle candidates only, a playing stop
            // clip ends (in game 0.5.6 start / move clips were chosen at |v| 0 while the gate was closed)
            if (g_standIdleOn && (!stick || g_effGate != 0) && capsuleSpeed < steer::kStandIdleSpeed)
            {
                in.excludeClasses |= steer::kStandIdleExclude;
                in.excludeCurrentClasses |= steer::kStandIdleExcludeCurrent;
                ++g_standIdleSearches;
                // stage standstill idle: no stop (tail) re-entry once the standstill held and the playing clip is an idle or
                // ends (see g_idleHoldOn)
                g_idleHoldStill = std::fmin(g_idleHoldStill + dt, 1e6f);
                if (g_idleHoldOn && g_idleHoldStill >= kIdleHoldDelay)
                {
                    const MatcherDiagnostics& hd = g_matcher.Diagnostics();
                    bool tail = false;
                    if (hd.clip >= 0 && size_t(hd.clip) < g_clipClasses.size())
                    {
                        const ClipRecord& hc = g_db.Clips()[size_t(hd.clip)];
                        tail = g_clipClasses[size_t(hd.clip)] == steer::kClassIdle ||
                               (hc.flags & kClipLoop) != 0 || int32_t(hc.numFrames) - 1 - hd.clipFrame <= kIdleHoldTailFrames;
                    }
                    if (tail)
                    {
                        bool idleLoop = false;
                        for (size_t c = 0; c < g_clipClasses.size() && !idleLoop; ++c)
                            idleLoop = g_clipClasses[c] == steer::kClassIdle && (g_db.Clips()[c].flags & kClipLoop) != 0 &&
                                       g_db.Clips()[c].numSearchable > 0 && g_matcher.ClipAllowed(int32_t(c));
                        if (idleLoop)
                        {
                            in.excludeClasses |= steer::Bit(steer::kClassStop);
                            ++g_idleHoldSearches;
                        }
                        else
                            ++g_idleHoldNoLoop;
                    }
                }
            }
            else
                g_idleHoldStill = 0.0f;
            // 0.5.8 cost cap retry set: the ordinary candidates (without the turn commit / settle) and, with the stance cap,
            // never a stance transition above the cap
            in.capRetry = g_costCap > 0.0f;
            in.capRetryExclude = (in.excludeClasses & ~commitBits) | (g_stanceCapOn ? steer::kStanceAllMask : 0u);
            in.capNeverClasses = g_stanceCapOn ? steer::kStanceAllMask : 0u;
            if (strafeS)
                in.capNeverClasses |= steer::Bit(steer::kClassStrafeTurn); // B2 strafe: the cost-cap retry never takes StrafeTurn
        }
        g_lastExclude = in.excludeClasses;
        if (in.excludeClasses & steer::Bit(steer::kClassLateral))
            ++g_lateralExcluded;
        if (mf.bodyTurn)
        {
            in.externalYaw = true;
            in.externalYawStep = yawStep;
            in.externalYawRate = g_followFed ? steer::MatcherCurve(yawRate, rate, g_steer.Config()) : yawRate / rate;
        }
        else if (strafeS)
        {
            // B2 strafe (motion Moving MaxControllerYawRate 0): the trajectory yaw takes the measured body step (aim-align snaps),
            // no extrapolated yaw rate, no turn query
            in.externalYaw = true;
            in.externalYawStep = yawStep;
            in.externalYawRate = 0.0f;
        }
        in.gait = aIn.locoState;
        {
            // 0.5.9 trajectory spring calibration (matcher-time velocity half-life per locomotion state)
            const int ls = (aIn.locoState >= 0 && aIn.locoState < 4) ? aIn.locoState : 0;
            const bool cal = g_trajCalOn && g_capProfile.trajHalflife[ls] > 0.0f;
            // within +-30 % of the configured half-life (the matcher features were tuned with it; larger changes added transitions)
            const float hl = cal ? std::clamp(g_capProfile.trajHalflife[ls], 0.7f * g_cfg.velocityHalflife, 1.3f * g_cfg.velocityHalflife)
                                 : g_cfg.velocityHalflife;
            if (hl != g_matcher.GetTrajectory().VelocityHalflife())
                g_matcher.GetTrajectory().SetVelocityHalflife(hl);
            g_trajHlApplied = hl;
            if (cal)
                ++g_trajCalFeeds;
        }
        // Physical displacement uses uncapped velocity; cx/cy below are query
        // calibration values.
        const MotionSetFrame* motionf = MotionStepLocked(aRaw, dt, g_lastCharVel[0], g_lastCharVel[1], yawStep);
        const bool motionHandback = !motionf && g_motionHeldMatcher; // UTPP-MOTION SEAM x AIR: first v1 update after a v2-owned span
        if (motionf || motionHandback)
            g_airHoldGroundReplantLeft = 0.0f;
        g_motionHandbackFeed = motionHandback;
        if (!motionf && !motionHandback)
            AlignAirLandTouchLocked(); // v1 only: reconcile genuine touchdown before advancing this feed's clip
        else
            g_airLandWasApproach = false; // v2 ownership / first handback invalidates a pending v1 Land correction
        if (motionf || motionHandback) // v2 and its first handback feed must not accept a v1 air-yaw command
            g_airSteerFed = false;
        if (!motionf) // UTPP-MOTION SEAM
            MotionHandbackLocked(); // UTPP-MOTION SEAM (also requests the AirState reset)
        if (motionf || motionHandback) // UTPP-MOTION SEAM x AIR (merge): the frozen v1 matcher plays no air clip (AirState still decides the leg context)
        { // UTPP-MOTION SEAM
            if (motionf) // UTPP-MOTION SEAM x AIR (merge repair): a reset emitted on a v2-owned feed has nothing to latch
                g_motionAirResetPending = false; // UTPP-MOTION SEAM
            g_airPlayMeta = -1; // UTPP-MOTION SEAM no clip feedback from the frozen v1 matcher
            // UTPP-MOTION SEAM handback: the fresh search runs (no suppression, no ForcePlay from the episode being reset); a playing
            // UTPP-MOTION SEAM frame that still carries root_z takes its bone features from the last clean ground frame (as AIR CLIPS)
            if (motionHandback && g_airLastCleanFrame >= 0 && g_matcher.HasRootZ() && // UTPP-MOTION SEAM
                std::fabs(g_matcher.RootZAt(g_matcher.Diagnostics().frame)) >= 0.005f) // UTPP-MOTION SEAM
            { // UTPP-MOTION SEAM
                in.queryBoneFrame = g_airLastCleanFrame; // UTPP-MOTION SEAM
                g_motionProbeQueryFrame = in.queryBoneFrame; // UTPP-MOTION SEAM
                ++g_airQueryOverrides; // UTPP-MOTION SEAM
            } // UTPP-MOTION SEAM
        } // UTPP-MOTION SEAM
        else // UTPP-MOTION SEAM
        {
        const bool motionReset = g_motionAirResetPending && g_airDec.action == air::Action::Reset; // UTPP-MOTION SEAM x AIR (merge repair)
        g_motionAirResetPending = false; // UTPP-MOTION SEAM
        in.suppressSearch = g_airDec.suppressSearch; // AIR HOLD stage (H3): no search while the air area owns the body
        if (in.suppressSearch)
            ++g_airSuppressed;
        ApplyAirPlayLocked(); // AIR CLIPS stage: ForcePlay the picked air clip (inertialized, decision half-life)
        if (g_airDec.action == air::Action::Reset || g_airDec.action == air::Action::HandBack)
        {
            // AIR CLIPS stage: the first search after the air area lets go takes its bone features from a clean ground frame when
            // the playing frame still carries root_z (dirty features); Reset carries AirState's own last clean frame.
            // UTPP-MOTION SEAM x AIR (merge repair): not AirState's frame on the reset after a v2 handback (pre-span frame; the
            // UTPP-MOTION SEAM handback feed already searched afresh); the root_z check below still applies
            int32_t q = motionReset ? -1 : g_airDec.queryBoneFrame;
            if (q < 0 && g_matcher.HasRootZ() && std::fabs(g_matcher.RootZAt(g_matcher.Diagnostics().frame)) >= 0.005f)
                q = g_airLastCleanFrame;
            if (q >= 0)
            {
                in.queryBoneFrame = q;
                in.dwellExempt = true;
                g_motionProbeQueryFrame = q; // UTPP-MOTION SEAM (probe only)
                ++g_airQueryOverrides;
            }
        }
        } // UTPP-MOTION SEAM x AIR (merge)
        const int64_t t0 = Qpc();
        if (motionf) // UTPP-MOTION SEAM
            MotionKeepBaseTrajectoryLocked(dt * rateNoAir, in); // UTPP-MOTION SEAM x AIR (merge repair): no v1 air clip warp on v2 feeds
        else // UTPP-MOTION SEAM
        g_matcher.Update(dt * (motionHandback ? rateNoAir : rate), in); // UTPP-MOTION SEAM x AIR: handback = fresh search, not the air clip
        if (!motionf) // UTPP-MOTION SEAM: lastUpdateUs / updates = v1 matcher updates only (v2 cost: mm.motionDriver status feedP95us)
        { // UTPP-MOTION SEAM
        g_lastUpdateUs = static_cast<double>(Qpc() - t0) * 1.0e6 / QpcFreq();
        ++g_matcherUpdates;
        if (strafeS && g_matcher.Diagnostics().searchedThisUpdate) // B2 strafe: search time with the patterns in the set
        {
            ++g_strafeSearches;
            const double us = g_matcher.Diagnostics().lastSearchMicros;
            g_strafeSUsSum += us;
            g_strafeSUsMax = std::fmax(g_strafeSUsMax, us);
        }
        if (g_feedDumpOn)
            CaptureFeedDumpBaseLocked(); // batch 1b
        } // UTPP-MOTION SEAM
        if (!motionf) // UTPP-MOTION SEAM: no v1 update this feed (frozen diagnostics)
        {
            const MatcherDiagnostics& td = g_matcher.Diagnostics(); // 0.5.7: a stance transition was selected
            if (td.transitionedThisUpdate && td.clip >= 0 && size_t(td.clip) < g_clipClasses.size() &&
                steer::IsStanceClass(g_clipClasses[size_t(td.clip)]))
                g_stanceUsed = true;
        }
        if (motionf) // UTPP-MOTION SEAM
            MotionReleaseSteeringLocked(); // UTPP-MOTION SEAM
        else // UTPP-MOTION SEAM
        UpdateStartBindLocked(stick, mf.bodyTurn, animLed, dt); // 0.5.8
        g_rawAnimRate = motionf ? motionf->rootYawRate : g_matcher.RootYawRate() * rate; // animation time -> real time; UTPP-MOTION SEAM
        g_bodyCtl.FeedAnimRate(g_rawAnimRate, dt);
        // Phase 5.3 offset root: visual root follows the animation root motion, spring back to the capsule. Runs
        // before the foot locker so the locks are placed under the visual body.
        std::span<const BoneTransform> pose(g_matcher.Output().bones);
        if (motionf) // UTPP-MOTION SEAM
            pose = std::span<const BoneTransform>(motionf->pose, motionf->numBones); // UTPP-MOTION SEAM
        if (g_offsetRootOn && g_offsetRoot.Ready())
        {
            float avx = 0.0f, avy = 0.0f;
            g_matcher.RootVelocity(avx, avy);
            float animVel[2] = {avx * rate, avy * rate}; // matcher time -> real time
            if (motionf) // UTPP-MOTION SEAM: v2 root motion (real time)
                animVel[0] = motionf->rootVel[0], animVel[1] = motionf->rootVel[1]; // UTPP-MOTION SEAM
            const float capVel[2] = {cx, cy};
            if (motionf && motionf->travFrame) // UTPP-MOTION SEAM trav: the montage root is warped onto the capsule by v2; the offset
                animVel[0] = cx, animVel[1] = cy; // UTPP-MOTION SEAM root must add no drift (cx / cy are sprint-capped: raw velocity drifted 0.3 m)
            // 0.3.7: in game the capsule velocity turns at once to the stick direction while the body turns with a
            // 0.2 s smoothing (velChar 28 deg off the body in the 0.3.6 keyboard slalom). Heading lean: the visual body
            // springs towards the travel heading (offset yaw, <= maxYaw 15 deg); root warp: the animation root velocity is
            // turned towards the capsule velocity (visual frame, <= 60 deg), so the offset root keeps only the speed
            // mismatch (it swung +-25 cm laterally every 0.9 s before).
            if (!motionf && g_leanOn && stick && capsuleSpeed > 0.5f && !strafeS && !g_airOwns) // B2 strafe: no heading lean (the body faces the aim); AIR: none in the air
                g_offsetRoot.SetYawGoal(std::atan2(-cx, cy));
            else
                g_offsetRoot.SetYawGoal(0.0f);
            if (!motionf && g_rootWarpOn && stick && !g_airOwns) // AIR HOLD stage (H4): no root warp in the air
            {
                float vx = 0.0f, vy = 0.0f;
                math::Rot2(-g_offsetRoot.Yaw(), cx, cy, vx, vy);
                // B2 strafe: root warp limited to rootWarpMaxDeg (30) between the loop directions
                const float warpMax = strafeS ? std::fmin(g_rootWarpMax, g_strafeCfg.rootWarpMaxDeg * kDegToRad) : g_rootWarpMax;
                steer::WarpToCapsule(animVel[0], animVel[1], vx, vy, warpMax);
            }
            // 0.3.6: in follow mode (UTPPSetMMSteerGap > 0) the body turn comes from the follow law, not the animation:
            // the visual body turns with the capsule (offset yaw lag made the offset root swing +-24 cm laterally in the
            // 0.3.5 keyboard slalom).
            const float offYawRate = (g_followFed && g_steer.Config().maxFacingGap > 0.0f && dt > 0.0f) ? yawStep / dt : g_rawAnimRate;
            // 0.6.2: lower body shown while masked; AIR HOLD stage (H4): released in the air (fast return to the capsule)
            if (motionf && motionf->providedRoot)
                (void)g_offsetRoot.AdoptFrame(motionf->rootOffsetX, motionf->rootOffsetY, motionf->rootOffsetYaw);
            else
                g_offsetRoot.Update(dt, capVel, yawStep, animVel, offYawRate, g_effGate == 0 && !g_airDec.releaseOffsetRoot);
            g_offsetRoot.Apply(pose);
            pose = g_offsetRoot.Output();
        }
        // loco leg pipeline: StanceGuard -> ... -> FootLocker (ApplyLegPipelineLocked), context from the frame just played
        g_legCtx = motionf ? MotionLegContextLocked(*motionf, aIn, capsuleSpeed) // UTPP-MOTION SEAM (built from ComputeLegContextLocked)
                       : ComputeLegContextLocked(aIn, capsuleSpeed);
        // HandBack is a search decision, not a new animation pose. Blend from the last frame that was
        // actually published in Recover, and run StanceGuard / FootLocker on the blended pose afterwards.
        const bool recoverHandback = !motionf && !motionHandback && g_airSetting != 3 &&
                                     g_airDec.effectiveMode == air::Mode::Clips &&
                                     g_airDec.action == air::Action::HandBack &&
                                     g_airLandGroundBlendPending && g_airLandGroundBones == pose.size() &&
                                     g_airLandGroundBlendLeft <= 0.0f;
        if (recoverHandback)
        {
            g_airLandGroundBlendLeft = kAirLandGroundBlendSeconds;
            g_airLandGroundElapsed = 0.0f;
            g_airLandGroundStartPending = true;
            g_airLandGroundStartHipsZ = g_airLandGroundPose[g_cfg.firstBodyBone].p.z;
            ++g_airLandGroundBlends;
        }
        if (motionf || motionHandback || g_airSetting == 3)
        {
            g_airLandGroundBlendLeft = 0.0f;
            g_airLandGroundBlendPending = false;
            g_airLandGroundBlendOutputValid = false;
        }
        else if (g_airLandGroundBlendLeft > 0.0f)
            pose = BlendAirLandToGroundLocked(pose, dt);
        ApplyLegPipelineLocked(pose, g_legCtx, dt, aIn, yaw);
        if (!motionf && !motionHandback && g_airSetting != 3 && g_airDec.phase == air::Phase::Recover &&
            g_airDec.effectiveMode == air::Mode::Clips && !g_airLandGroundBlendOutputValid)
        {
            // Snapshot every Recover feed, including the last one before HandBack, from the final
            // published pipeline pose. It includes ground-safe FootLocker output, never a stale first frame.
            const auto land = PipelineOutputLocked();
            if (land.size() <= integ::kMaxMMBones && g_cfg.firstBodyBone < land.size())
            {
                std::copy(land.begin(), land.end(), g_airLandGroundPose.begin());
                g_airLandGroundBones = static_cast<uint32_t>(land.size());
                g_airLandGroundBlendPending = true;
            }
        }
        if (g_airDec.phase != air::Phase::Recover && g_airLandGroundBlendLeft <= 0.0f)
            g_airLandGroundBlendPending = false;
        g_poseValid = true; // 0.6.2: the pipeline ran - the published pose may be held / republished
    }
    if (!g_motionStepped && g_motion && g_motion->step) // matcher block did not run this feed
    {
        // The held/open publication path must use the SAME host context and leg
        // tail. A bare step would prepare visual-local AO but publish an
        // untransformed root.
        const MotionSetFrame *frame = MotionStepLocked(aRaw, dt, g_lastCharVel[0], g_lastCharVel[1], yawStep);
        if (frame)
        {
            g_motionHeldMatcher = true;
            g_airHoldGroundReplantLeft = 0;
            g_motionAirResetPending = false;
            g_rawAnimRate = frame->rootYawRate;
            g_bodyCtl.FeedAnimRate(g_rawAnimRate, dt);
            std::span<const BoneTransform> pose(frame->pose, frame->numBones);
            if (g_offsetRootOn && g_offsetRoot.Ready())
            {
                if (frame->providedRoot)
                    (void)g_offsetRoot.AdoptFrame(frame->rootOffsetX, frame->rootOffsetY, frame->rootOffsetYaw);
                else
                {
                    const float cap[2] = {cx, cy};
                    const float anim[2] = {frame->travFrame ? cx : frame->rootVel[0],
                                           frame->travFrame ? cy : frame->rootVel[1]};
                    g_offsetRoot.Update(dt, cap, yawStep, anim, frame->rootYawRate, g_effGate == 0);
                }
                g_offsetRoot.Apply(pose);
                pose = g_offsetRoot.Output();
            }
            g_legCtx = MotionLegContextLocked(*frame, aIn, capsuleSpeed);
            ApplyLegPipelineLocked(pose, g_legCtx, dt, aIn, yaw);
            g_poseValid = true;
        }
    }
    // Preserve the actual feed's ownership until after the mask update;
    // pending/reset flags may have cleared already.
    const bool baseMaskOwner = !g_legCtx.v2 && !g_motionHeldMatcher && !g_motionAirResetPending && !g_motionHandbackFeed &&
                             g_poseValid && g_airDec.action != air::Action::Reset;
    g_motionStepped = false; // UTPP-MOTION SEAM
    const bool shown = g_weight.Weight() > 0.0f;
    const float w = g_weight.Update(g_effGate, dt, g_fade); // 0.6.2: masked reasons keep the lower body
    // AIR hold may leave the raw jump gate closed while the effective gate is open. Once AIR owns
    // the frame, release a stale previous upper mask so the land/ground legs are not partly hidden.
    const bool airMaskRelease = g_airSetting != 3 && g_airOwns && g_holdActive && g_effGate == 0 &&
                                baseMaskOwner &&
                                g_airDec.action != air::Action::HandBack && g_airDec.action != air::Action::Reset &&
                                g_releaseReasons == 0 && g_maskReasons == 0;
    const bool weaponMaskWanted = g_upperMaskOn && g_upperMask.ok && g_weaponUpperOn && g_weaponKnown && g_weaponHeld &&
                                  aIn.tppActive && g_poseValid && (g_effGate == 0 || g_holdActive) &&
                                  (aIn.psmBlock & ExitReleaseMask(g_exitLeft > 0.0f, g_exitLatchedPsm)) == 0 && g_releaseReasons == 0;
    if (g_trace && g_log && (weaponMaskWanted != g_weaponUpperActive || prevWeaponKnown != g_weaponKnown || prevWeaponHeld != g_weaponHeld))
    {
        char line[160];
        std::snprintf(line, sizeof line, "UTPP MMWEAPON on=%d known=%d held=%d active=%d air=%d motion=%d gate=0x%X",
                      g_weaponUpperOn ? 1 : 0, g_weaponKnown ? 1 : 0, g_weaponHeld ? 1 : 0, weaponMaskWanted ? 1 : 0,
                      g_legCtx.airborne ? 1 : 0, g_legCtx.v2 ? 1 : 0, g_gate);
        g_log(line);
    }
    g_weaponUpperActive = weaponMaskWanted;
    // melee guard (round 3, mm/MeleeFullBody.hpp GuardGate): while a guard-capable melee weapon is drawn the upper body stays on
    // the game pose between strikes too. It folds into the upper mask, the full body (stage 2) and the tilt below; with every
    // guard setting off (default) guardWanted is false and the terms are the previous expressions bit for bit
    // (tests/MeleeFullBodyTest.cpp TestDriverTerms).
    mm::melee::GuardInput guardIn;
    guardIn.flag = g_meleeGuardFlag;
    guardIn.setting = g_meleeGuardSetting;
    guardIn.maskOk = g_upperMaskOn && g_upperMask.ok;
    guardIn.owner = baseMaskOwner && aIn.tppActive && g_releaseReasons == 0;
    guardIn.gateOpen = (g_effGate == 0 || g_holdActive) && (aIn.psmBlock & ExitReleaseMask(g_exitLeft > 0.0f, g_exitLatchedPsm)) == 0;
    guardIn.meleeState = g_meleeState;
    guardIn.upperBody = g_upperBody;
    guardIn.stance = conditioned.locoState;
    guardIn.psmDetail = psmDetail;
    guardIn.haveAir = aIn.haveAir;
    guardIn.onGround = aIn.onGround;
    guardIn.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
    guardIn.maxVertical = g_gateDebounceCfg.maxVertical;
    guardIn.dt = dt;
    const bool guardWanted = g_meleeGuardGate.Update(guardIn) == mm::melee::kGuardOn;
    mm::melee::DriverIn di;
    di.maskReasonsUpper = g_maskReasons != 0 && g_maskUpperFromGame;
    di.weaponMaskWanted = weaponMaskWanted;
    di.aimFadeEligible = g_aimUpperFadeOn && g_maskFadeFromAim && g_maskReasons == 0;
    di.meleeMask = (g_maskReasons & kMaskStateMelee) != 0 && g_maskUpperFromGame && g_upperMaskOn && g_upperMask.ok;
    di.guardWanted = guardWanted;
    di.guardFullSetting = g_meleeGuardFullSetting;
    di.guardTiltSetting = g_meleeGuardTiltSetting;
    const mm::melee::DriverTerms terms = mm::melee::CombineDriverTerms(di);
    // Fade times of the mask (mm/MeleeFullBody.hpp MaskTimeTracker). Melee release (2026-11-12): while the melee reason (melee
    // weapon, no weapon / guard / vehicle-exit reason) holds the mask the fade out is armed for the eased
    // kMeleeReleaseFadeOutSeconds; any other holder disarms it. Melee guard (2026-11-13): the guard ending for a soft reason
    // (state left the guard states, crouch / sprint) eases out the same way, a hard end (closed gate, UpperBody swap, air, slide,
    // dodge ...) keeps the quick fade; a guard-only raise takes kGuardFadeInSeconds. Fade IN after a strike stays kMaskFadeSeconds.
    mm::melee::MaskTimeIn mti;
    mti.meleeReasonOnly = (g_maskReasons & kMaskStateMelee) != 0 && (g_maskReasons & ~(kMaskStateMelee | kMaskStateAim)) == 0;
    mti.weaponMaskWanted = weaponMaskWanted;
    mti.guardWanted = guardWanted;
    mti.guardSoft = g_meleeGuardGate.LastSoft();
    mti.longRelease = g_meleeGate.LongRelease();
    mti.maskProgress = g_maskBlend.Progress();
    const mm::melee::MaskTimes maskTimes = g_maskTimes.Update(terms, mti, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds);
    const float maskOutTime = maskTimes.out;
    // CAR-1 / MOTO-6: an exit-window-only mask in TPP covers the gate (legs on MM) but leaves the upper body on MM as well
    g_maskBlend.UpdateInOut(terms.maskWanted, g_gate == 0, shown, dt, maskTimes.in, maskOutTime, airMaskRelease);
    if (guardWanted && !g_meleeGuardWant)
        ++g_meleeGuardSwitches;
    if (g_trace && g_log && guardWanted != g_meleeGuardWant)
    {
        char line[320];
        std::snprintf(line, sizeof line, "UTPP MMGUARD %s why=%s state=%d upper=%d loco=%d rawLoco=%d detail=%d airT=%.2f mask=%.2f full=%.2f tilt=%.2f reasons=0x%X gate=0x%X effGate=0x%X hold=%d fullSet=%d tiltSet=%d soft=%d",
                      guardWanted ? "on" : "off", mm::melee::GuardWhyName(g_meleeGuardGate.LastWhy()), g_meleeState, g_upperBody,
                      conditioned.locoState, aIn.locoState, psmDetail, static_cast<double>(g_meleeGuardGate.AirSeconds()),
                      static_cast<double>(g_maskBlend.Weight()), static_cast<double>(g_meleeFullBlend.Weight()),
                      static_cast<double>(g_meleeAimBlend.Weight()), g_maskReasons, g_gate, g_effGate, g_holdActive ? 1 : 0,
                      g_meleeGuardFullSetting ? 1 : 0, g_meleeGuardTiltSetting ? 1 : 0, g_meleeGuardGate.LastSoft() ? 1 : 0);
        g_log(line);
    }
    g_meleeGuardWant = guardWanted;
    if (g_maskBlend.Progress() <= 0.0f)
        g_maskFadeFromAim = false;
    // melee full body (2026-11-09, mm/MeleeFullBody.hpp): standing melee with a covered weapon -> the lower-body bones fade to
    // the game pose too. Gate open = the effective gate: with the whole body fading out the fade holds (no leg flash to MM).
    // Melee aim tilt weight (mm/MeleeAim.hpp, applied by the FK detour): fade of the melee mask reason, both mask modes.
    {
        mm::melee::Input fi;
        fi.flag = g_meleeFullFlag && g_meleeFullSetting;
        fi.meleeMask = terms.fullMelee;        // the melee mask reason with the upper body from the game (unchanged term)
        fi.guard = terms.fullGuard;            // stage 2 of the guard (melee.guardFullBody): adds Idle 2 only
        fi.meleeState = g_meleeState;          // review finding 1: only states whose third-person clips exist ...
        fi.allowedStates = g_meleeFullStates;  // ... (default: the attacks without ThrowAttack)
        fi.owner = baseMaskOwner && aIn.tppActive && g_releaseReasons == 0;
        // the stance the legs query: fists block / targeting report AimWalk as locoState 0 (logged 'MMMASK on reasons=0x3 ...
        // loco=0'); the mask path above maps it to the last stance (stand by default), crouch aim keeps 2
        fi.stance = conditioned.locoState;
        fi.psmDetail = psmDetail;
        // move intent: the stick filter's held state (half stick counts, short releases bridged); with the filter off the raw
        // stick against the same press threshold. Moving = stick, not the capsule: the attacks move the capsule briefly.
        {
            const float mx = std::isfinite(aRaw.move[0]) ? aRaw.move[0] : 0.0f, my = std::isfinite(aRaw.move[1]) ? aRaw.move[1] : 0.0f;
            const float press = steer::StickFilterConfig{}.press;
            fi.stickHeld = g_stickFilterOn ? g_stickHeld : (mx * mx + my * my > press * press);
        }
        fi.haveAir = aIn.haveAir;
        fi.onGround = aIn.onGround;
        fi.vz = std::isfinite(aIn.velWorld[2]) ? aIn.velWorld[2] : 0.0f;
        fi.maxVertical = g_gateDebounceCfg.maxVertical;
        fi.planarSpeed = g_rawSpeed;
        fi.dt = dt;
        g_meleeFullStick = fi.stickHeld;
        g_meleeFullGround = fi.haveAir ? (fi.onGround ? 1 : 0) : -1;
        const bool fullWant = g_meleeFullGate.Update(fi);
        if (fullWant && !g_meleeFullWant)
            ++g_meleeFullSwitches;
        if (g_trace && g_log && fullWant != g_meleeFullWant)
        {
            char line[320];
            std::snprintf(line, sizeof line, "UTPP MMMELEEFULL %s why=%s full=%.2f speed=%.2f loco=%d rawLoco=%d detail=%d vz=%.2f reasons=0x%X gate=0x%X stick=%d ground=%d fastT=%.2f fastOff=%d airT=%.2f state=%d states=0x%X guard=%d",
                          fullWant ? "on" : "off", mm::melee::WhyName(g_meleeFullGate.LastWhy()),
                          static_cast<double>(g_meleeFullBlend.Weight()), static_cast<double>(g_rawSpeed), fi.stance, aIn.locoState,
                          psmDetail, static_cast<double>(fi.vz), g_maskReasons, g_gate, fi.stickHeld ? 1 : 0, g_meleeFullGround,
                          static_cast<double>(g_meleeFullGate.FastSeconds()), g_meleeFullGate.FastOff() ? 1 : 0,
                          static_cast<double>(g_meleeFullGate.AirSeconds()), g_meleeState, g_meleeFullStates, guardWanted ? 1 : 0);
            g_log(line);
        }
        g_meleeFullWant = fullWant;
        // legs: quick fade out except when the melee weapon state itself ended (then eased, back on MM before the arms fade)
        g_meleeFullBlend.UpdateInOut(fullWant, g_effGate == 0, shown, dt, mm::melee::kFadeSeconds,
                                     mm::melee::LegFadeOutSeconds(g_meleeFullGate.LastWhy(), g_meleeGate.LongRelease()));
        // tilt: follows the mask (the same eased fade out, else 25 deg of tilt would unwind in 0.15 s under slow arms)
        g_meleeAimBlend.UpdateInOut(terms.tiltWanted, true, shown, dt, kMaskFadeSeconds, maskTimes.tiltOut); // melee reason (+ guard with guardAimTilt)
        if (g_trace && g_log && g_meleeAimLogSource)
        {
            const int64_t nowTicks = Qpc();
            if (nowTicks - g_meleeAimLogTicks > static_cast<int64_t>(QpcFreq() * 0.25))
            {
                char line[704]; // the MELEEAIM line + the chest base correction state (mm/MeleeBase.cpp AppendLog)
                if (g_meleeAimLogSource(line, sizeof line))
                {
                    g_meleeAimLogTicks = nowTicks;
                    g_log(line);
                }
            }
        }
    }
    ExitDiagLocked(aIn, aRaw.locoState, psmDetail, dt, w);
    g_target = g_weight.Target();
    if (w <= 0.0f)
        ResetOffsetLocked(); // body not shown: next fade-in starts at the capsule
    PublishLocked(w);
    if (w <= 0.0f && CandidateReadyLocked())
    {
        TrySwapLocked(); // 0.5.1: the new database fades in through the normal gate from the next feed
        return;
    }
    if (g_holdActive && CandidateReadyLocked())
    {
        // 0.6.2: database switch under hold - swap at full weight (the bone set is validated identical); the held pose stays
        // published until the next feed runs the new matcher, then the body continues on the new database at weight 1
        g_holdSwap = true;
        TrySwapLocked();
        g_holdSwap = false;
        g_weight.Fill();
        ++g_holdSwaps;
        return;
    }

    if (g_trace && g_log && g_matcher.Ready())
    {
        const MatcherDiagnostics& d = g_matcher.Diagnostics();
        if (d.searchedThisUpdate && d.lastSearchMicros > g_traceMaxSearchUs)
            g_traceMaxSearchUs = d.lastSearchMicros;
        const bool transitioned = d.transitions != g_traceLastTransitions;
        // 0.5.8 cost cap events
        if (d.costCapHolds != g_traceCapHolds || d.costCapRetries != g_traceCapRetries || d.costCapForced != g_traceCapForced)
        {
            const char* kind = d.costCapHolds != g_traceCapHolds ? "hold" : (d.costCapForced != g_traceCapForced ? "forced" : "retry");
            const int32_t bc = d.costCapLastIndex >= 0 ? g_db.ClipOfFrame(static_cast<uint32_t>(d.costCapLastIndex)) : -1;
            const std::string_view bn = bc >= 0 ? g_db.ClipName(static_cast<uint32_t>(bc)) : std::string_view{};
            const std::string_view pn = d.clip >= 0 ? g_db.ClipName(static_cast<uint32_t>(d.clip)) : std::string_view{};
            char cl[400];
            std::snprintf(cl, sizeof cl, "UTPP MMTRACE costcap %s clip=%.*s f=%d best=%.*s bestF=%d bestCost=%.1f cap=%.1f |v|=%.2f loco=%d w=%.2f",
                          kind, static_cast<int>(pn.size()), pn.data(), d.clipFrame, static_cast<int>(bn.size()), bn.data(),
                          bc >= 0 ? d.costCapLastIndex - static_cast<int32_t>(g_db.Clips()[size_t(bc)].firstFrame) : -1,
                          static_cast<double>(d.costCapLastCost), static_cast<double>(g_costCap), static_cast<double>(g_rawSpeed),
                          aIn.locoState, static_cast<double>(w));
            g_log(cl);
            g_traceCapHolds = d.costCapHolds;
            g_traceCapRetries = d.costCapRetries;
            g_traceCapForced = d.costCapForced;
        }
        const bool summary = (now - g_traceLastSummary) > static_cast<int64_t>(QpcFreq());
        if (transitioned || summary)
        {
            const std::string_view clip = d.clip >= 0 ? g_db.ClipName(static_cast<uint32_t>(d.clip)) : std::string_view{};
            const FootLockDiagnostics& fd = g_footLock.Diagnostics();
            // B2 strafe S2: " chestDev= chestCorr=" (deg) at the END, only on feeds where ChestAlign ran with valid diagnostics
            char chestTail[64] = "";
            if (g_chestValid)
                std::snprintf(chestTail, sizeof chestTail, " chestDev=%.1f chestCorr=%.1f",
                              static_cast<double>(g_chestAlign.Diagnostics().dev), static_cast<double>(g_chestAlign.Diagnostics().delta));
            char line[1400];
            std::snprintf(line, sizeof(line),
                          "UTPP MMTRACE %s clip=%.*s f=%d cost=%.3f velChar=(%.2f,%.2f) |v|=%.2f rate=%.2f yawRate=%.2f sUs=%.0f maxSUs=%.0f w=%.2f gate=0x%X loco=%d tpp=%d searches=%llu trans=%llu mv=(%.2f,%.2f) cam=%.0f pl=%.0f fl=%d L=%d/%.2f/%.1fcm R=%d/%.2f/%.1fcm flUs=%.1f bt=%d lat=%d tgt=%.0f err=%.1f animYR=%.1f cmdYR=%.1f btCmds=%llu off=(%.1f,%.1f)cm offYaw=%.1f accel=%.2f/%.2fx%.2f root=%.2f loopRate=%.2f fol=%d tRate=%.0f excl=0x%X slow=%d dwH=%llu band=%d loopBand=%d-%d stSign=%s decel=%.2f/%.2fx%.2f far=%.2f trajHl=%.3f mask=%.2f/0x%X hold=%d turnRate=%.2f/%.2f strafe=%d dir=%.0f gait=%d rateSrc=%s%s",
                          transitioned ? "TRANS" : "tick", static_cast<int>(clip.size()), clip.data(), d.clipFrame,
                          d.lastSearchCost, cx, cy, g_rawSpeed, g_lastRate, yawRate, d.lastSearchMicros, g_traceMaxSearchUs, w, g_gate, aIn.locoState,
                          aIn.tppActive ? 1 : 0, static_cast<unsigned long long>(d.searches),
                          static_cast<unsigned long long>(d.transitions), aIn.move[0], aIn.move[1], aIn.cameraYawDeg, aIn.playerYawDeg,
                          g_footLock.Ready() && g_footLockOn ? 1 : 0, fd.foot[0].locked ? 1 : 0, fd.foot[0].weight,
                          fd.foot[0].correction * 100.0f, fd.foot[1].locked ? 1 : 0, fd.foot[1].weight,
                          fd.foot[1].correction * 100.0f, g_lastFootLockUs, g_bodyTurnFed ? 1 : 0, g_latch.latched ? 1 : 0,
                          static_cast<double>(g_targetYaw / kDegToRad),
                          static_cast<double>(WrapPi(g_targetYaw - yaw) / kDegToRad),
                          static_cast<double>(g_bodyCtl.AnimRate() / kDegToRad),
                          static_cast<double>(g_bodyCtl.Diagnostics().rate / kDegToRad),
                          static_cast<unsigned long long>(g_bodyCmds), static_cast<double>(g_offsetRoot.X() * 100.0f),
                          static_cast<double>(g_offsetRoot.Y() * 100.0f),
                          static_cast<double>(g_offsetRoot.Yaw() / kDegToRad), static_cast<double>(g_accelLastTarget),
                          static_cast<double>(g_accelBase), static_cast<double>(g_accelMul),
                          static_cast<double>(g_rootSpeedDb), static_cast<double>(g_loopRate), g_followFed ? 1 : 0,
                          static_cast<double>(g_steer.TargetRate() / kDegToRad), g_lastExclude, g_slowGateActive ? 1 : 0,
                          static_cast<unsigned long long>(d.dwellHolds), g_lastBand, g_lastLoopBand, g_lastLoopBandHi,
                          g_stSign == 1 ? "clip" : (g_stSign == 2 ? "fallback" : "off"), static_cast<double>(g_decelLastTarget),
                          static_cast<double>(g_decelBase), static_cast<double>(g_decelMul),
                          static_cast<double>(g_offsetRoot.Diagnostics().farTime), static_cast<double>(g_trajHlApplied),
                          static_cast<double>(g_maskBlend.Weight()), g_maskReasons, g_holdActive ? 1 : 0,
                          static_cast<double>(g_turnRateActive ? g_turnRateW : 0.0f), static_cast<double>(g_turnRateTarget), // 0.6.3
                          g_strafeMode.On() ? 1 : 0, static_cast<double>(g_strafeMode.On() ? g_strafeDirDeg : 0.0f), g_strafeGait,
                          g_strafeRateSrc == 2 ? "own" : (g_strafeRateSrc == 1 ? "map" : "off"), // B2 strafe (appended)
                          chestTail); // B2 strafe S2 (appended, only while valid)
            g_log(line);
            if (g_feetDiagOn)
                TraceFeetLocked(); // loco C0: separate line, the format above is unchanged
            g_traceLastTransitions = d.transitions;
            if (summary)
            {
                g_traceLastSummary = now;
                g_traceMaxSearchUs = 0.0; // per-second maximum
            }
        }
    }
    FeetEventsLocked(g_trace && g_feetDiagOn && g_matcher.Ready(), aIn.locoState); // loco C0
}

void SetSpeedMap(float aGameWalk, float aGameRun, float aDbWalk, float aDbRun, float aRateMin, float aRateMax)
{
    std::lock_guard lock(g_mutex);
    const bool ok = std::isfinite(aGameWalk) && std::isfinite(aGameRun) && std::isfinite(aDbWalk) && std::isfinite(aDbRun) &&
                    std::isfinite(aRateMin) && std::isfinite(aRateMax) && aGameWalk > 0.1f && aGameRun > aGameWalk + 0.1f &&
                    aDbWalk > 0.1f && aDbRun > aDbWalk + 0.1f && aRateMin > 0.1f && aRateMax >= aRateMin && aRateMax <= 4.0f;
    if (!ok)
        return;
    // Stand/sprint: walk and run points from the arguments, keep the sprint point.
    for (int st : {1, 3})
    {
        g_speedMap[st].game[0] = aGameWalk;
        g_speedMap[st].db[0] = aDbWalk;
        g_speedMap[st].game[1] = aGameRun;
        g_speedMap[st].db[1] = aDbRun;
    }
    g_rateMin = aRateMin;
    g_rateMax = aRateMax;
}

void SetSpeedPoint(int32_t aLoco, int32_t aIndex, float aGame, float aDb)
{
    std::lock_guard lock(g_mutex);
    if (aLoco < 1 || aLoco > 3 || aIndex < 0 || aIndex > 2 || !std::isfinite(aGame) || !std::isfinite(aDb) || aGame <= 0.1f ||
        aDb <= 0.1f)
        return;
    SpeedPoints& m = g_speedMap[aLoco];
    if (aIndex > m.count)
        return; // points must be added in order
    m.game[aIndex] = aGame;
    m.db[aIndex] = aDb;
    if (aIndex == m.count)
        ++m.count;
}

void SetInputDriven(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_inputDriven = aOn;
}

void SetInputCameraRelative(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_inputCameraRelative = aOn;
}

void SetTransitionCost(float aCost)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aCost) || aCost < 0.0f || aCost > 10.0f)
        return;
    g_transitionCost = aCost;
    if (g_dbLoaded)
        InitMatcherLocked();
}

void SetFootLock(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_footLockOn = aOn;
    g_footLock.SetEnabled(aOn); // off: locked feet blend out (blendOutTime), no snap
}

void SetOffsetRoot(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_offsetRootOn = aOn;
    if (g_offsetRoot.Ready())
        g_offsetRoot.Reset(); // off: pose exactly as 0.3.2 (an active offset snaps back, at most maxDistance)
}

bool SetOffsetRootParams(float aTranslationHalflife, float aRotationHalflife, float aMaxDistance, float aMaxYawDeg)
{
    if (!(std::isfinite(aMaxYawDeg) && aMaxYawDeg >= 0.0f && aMaxYawDeg <= 90.0f) ||
        !(std::isfinite(aTranslationHalflife) && aTranslationHalflife >= 0.02f && aTranslationHalflife <= 5.0f) ||
        !(std::isfinite(aRotationHalflife) && aRotationHalflife >= 0.02f && aRotationHalflife <= 5.0f))
        return false;
    std::lock_guard lock(g_mutex);
    OffsetRootConfig c = g_offsetCfg;
    c.translationHalflife = aTranslationHalflife;
    c.rotationHalflife = aRotationHalflife;
    c.maxDistance = aMaxDistance;
    c.maxYaw = aMaxYawDeg * kDegToRad;
    if (g_offsetRoot.Ready() ? !g_offsetRoot.SetConfig(c) : !(std::isfinite(aMaxDistance) && aMaxDistance >= 0.0f && aMaxDistance <= 1.0f))
        return false;
    g_offsetCfg = c;
    ApplyCatchupLocked(); // 0.5.9
    return true;
}

void SetRootRate(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_rootRateOn = aOn;
    g_rootRate.Reset();
}

bool SetRootRateParams(float aRateMin, float aRateMax, float aEpsilon, float aHalflife)
{
    std::lock_guard lock(g_mutex);
    RootRateConfig c;
    c.rateMin = aRateMin;
    c.rateMax = aRateMax;
    c.epsilon = aEpsilon;
    c.halflife = aHalflife;
    return g_rootRate.SetConfig(c);
}

bool SetSlowWalk(bool aOn, float aWalkRateMin)
{
    if (!std::isfinite(aWalkRateMin) || aWalkRateMin < 0.1f || aWalkRateMin > 1.0f)
        return false;
    std::lock_guard lock(g_mutex);
    g_slowWalkOn = aOn;
    g_walkRateMin = aWalkRateMin;
    return true;
}

bool SetOffsetSaturation(float aSeconds, float aHalflife)
{
    std::lock_guard lock(g_mutex);
    OffsetRootConfig c = g_offsetCfg;
    c.saturatedTime = aSeconds;
    c.saturatedHalflife = aHalflife;
    if (!std::isfinite(aSeconds) || aSeconds < 0.0f || aSeconds > 10.0f || !std::isfinite(aHalflife) || aHalflife < 0.02f ||
        aHalflife > 5.0f || (g_offsetRoot.Ready() && !g_offsetRoot.SetConfig(c)))
        return false;
    g_offsetCfg = c;
    ApplyCatchupLocked(); // 0.5.9
    return true;
}

void SetExcludeLands(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_excludeLands = aOn;
    if (g_dbLoaded)
    {
        ComputeStateTagMasksLocked(); // applied on the next feed (SetAllowedTagMask, no playback reset)
        RecomputeDerivedClassDataLocked(false); // loco P0 (runs only while UTPPSetMMDerivedRecompute is on)
    }
}

// ---- loco P0 ----
void SetDerivedRecompute(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_derivedRecomputeOn = aOn;
    if (aOn && g_dbLoaded && g_matcher.Ready()) // on: the active database's data at once; off: 0.6.5b order from the next load
        LogDerivedLocked(g_dbName, RecomputeDerivedClassDataLocked(false));
}

namespace
{
unsigned long long Fnv1a64(const void* aData, size_t aBytes)
{
    uint64_t h = 1469598103934665603ull; // FNV-1a 64
    const auto* p = static_cast<const uint8_t*>(aData);
    for (size_t i = 0; i < aBytes; ++i)
    {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return static_cast<unsigned long long>(h);
}
} // namespace

ClassSetProbe ProbeClassSet(int aSet, int aBand)
{
    std::lock_guard lock(g_mutex);
    ClassSetProbe r;
    r.set = g_activeClassSet;
    r.band = g_activeLoopBand;
    if (!g_matcher.Ready())
        return r;
    r.ready = true;
    const std::span<const uint8_t> held = g_matcher.ClipClasses();
    r.held = Fnv1a64(held.data(), held.size());
    std::vector<uint8_t> scratch;
    // band = lo + 4 * hi (ApplyClassSetLocked; lo <= hi <= 3)
    const auto expect = [&](int aS, int aB)
    {
        if (aS < 0 || aS >= 4)
            return 0ull;
        const std::vector<uint8_t>& v = ClassSetBufferLocked(aS, aB % 4, aB / 4, scratch);
        return Fnv1a64(v.data(), v.size());
    };
    r.expectActive = expect(g_activeClassSet, g_activeLoopBand);
    r.expectAt = expect(aSet < 0 ? g_activeClassSet : aSet, aBand < 0 ? g_activeLoopBand : aBand);
    return r;
}

std::string DerivedClassDataDigest()
{
    std::lock_guard lock(g_mutex);
    const auto hash = [](const void* aData, size_t aBytes) { return Fnv1a64(aData, aBytes); };
    int banded = 0;
    for (const int8_t b : g_loopBandOf)
        banded += b > 0 ? 1 : 0;
    char buf[640];
    std::snprintf(buf, sizeof buf,
                  "clips=%u masks=%llx/%llx/%llx loopAvail=%d/%d/%d/%d banded=%d bandOf=%016llx lateral=%d%d%d%d classes=%016llx "
                  "match=%016llx/%016llx/%016llx turnSign=%016llx turnEnd=%016llx rotEnd=%016llx",
                  g_db.Loaded() ? g_db.NumClips() : 0u, static_cast<unsigned long long>(g_stateTagMask[kLocoStand]),
                  static_cast<unsigned long long>(g_stateTagMask[kLocoCrouch]),
                  static_cast<unsigned long long>(g_stateTagMask[kLocoSprint]), g_loopAvail[0], g_loopAvail[1], g_loopAvail[2],
                  g_loopAvail[3], banded, hash(g_loopBandOf.data(), g_loopBandOf.size()), g_lateralOk[0] ? 1 : 0,
                  g_lateralOk[1] ? 1 : 0, g_lateralOk[2] ? 1 : 0, g_lateralOk[3] ? 1 : 0,
                  hash(g_clipClasses.data(), g_clipClasses.size()), hash(g_matchClasses[0].data(), g_matchClasses[0].size()),
                  hash(g_matchClasses[1].data(), g_matchClasses[1].size()), hash(g_matchClasses[2].data(), g_matchClasses[2].size()),
                  hash(g_turnSign.data(), g_turnSign.size()), hash(g_turnEnd.data(), g_turnEnd.size() * sizeof(int32_t)),
                  hash(g_rotEnd.data(), g_rotEnd.size() * sizeof(int32_t)));
    return buf;
}

// ---- loco C0 / C1a ----
void SetFeetDiag(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_feetDiagOn = aOn;
}

// ---- loco C1b ----
bool SetFootReplant(int32_t aMask)
{
    if (aMask < 0 || aMask > (kReplantStand | kReplantCrouch))
        return false;
    std::lock_guard lock(g_mutex);
    g_footReplantMask = aMask; // applied from the next feed (a planted foot of a stance switched off is handed over smoothly)
    return true;
}

int32_t FootReplantMask()
{
    std::lock_guard lock(g_mutex);
    return g_footReplantMask;
}

// ---- batch 2 knee fix ----
bool SetKneeFix(int32_t aMask)
{
    if (aMask < 0 || aMask > kKneeFixAll)
        return false;
    std::lock_guard lock(g_mutex);
    g_kneeFixMask = aMask; // applied from the next feed (the filtered swivel fades / follows, no reset)
    return true;
}

int32_t KneeFixMask()
{
    std::lock_guard lock(g_mutex);
    return g_kneeFixMask;
}

#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
bool MatcherPoseForTest(std::vector<float>& aOut)
{
    std::lock_guard lock(g_mutex);
    if (!g_matcher.Ready())
        return false;
    const auto& b = g_matcher.Output().bones;
    aOut.resize(b.size() * 7);
    for (size_t i = 0; i < b.size(); ++i)
    {
        float* d = &aOut[i * 7];
        d[0] = b[i].p.x;
        d[1] = b[i].p.y;
        d[2] = b[i].p.z;
        d[3] = b[i].q.x;
        d[4] = b[i].q.y;
        d[5] = b[i].q.z;
        d[6] = b[i].q.w;
    }
    return true;
}
#endif

bool SetFootReplantParams(float aRadius, float aStopsRadius, float aRatio, float aStopsRatio, float aStepSpeed, float aStopsStepSpeed,
                          float aLift)
{
    std::lock_guard lock(g_mutex);
    FootReplantConfig c = g_replantCfg;
    c.radius = aRadius;
    c.radiusStops = aStopsRadius;
    c.ratio = aRatio;
    c.ratioStops = aStopsRatio;
    c.stepSpeed = aStepSpeed;
    c.stepSpeedStops = aStopsStepSpeed;
    c.lift = aLift;
    if (!FootReplantConfig::Valid(c))
        return false;
    g_replantCfg = c;
    if (g_footLock.Ready())
        g_footLock.SetReplantConfig(c);
    return true;
}

void SetCrouchIdleExclude(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_crouchExclOn = aOn;
    if (g_dbLoaded)
    {
        ComputeStateTagMasksLocked();           // applied on the next feed (SetAllowedTagMask, no playback reset)
        RecomputeDerivedClassDataLocked(false); // loco P0 hook (crouch lateral rule; strafe profiles later)
        LogCrouchExclLocked(g_dbName, "setting");
    }
}

bool SetCrouchExcludeTags(const std::string& aList)
{
    if (aList.size() > 512)
        return false;
    std::string norm;
    int names = 0;
    size_t p = 0;
    while (p <= aList.size())
    {
        size_t e = aList.find(',', p);
        if (e == std::string::npos)
            e = aList.size();
        size_t a = p, b = e;
        while (a < b && (aList[a] == ' ' || aList[a] == '\t'))
            ++a;
        while (b > a && (aList[b - 1] == ' ' || aList[b - 1] == '\t'))
            --b;
        if (b > a)
        {
            for (size_t i = a; i < b; ++i)
            {
                const unsigned char ch = static_cast<unsigned char>(aList[i]);
                if (!(std::isalnum(ch) || ch == '_'))
                    return false; // exact tag names: letters, digits, '_'
            }
            if (!norm.empty())
                norm += ',';
            norm.append(aList, a, b - a);
            ++names;
        }
        p = e + 1;
    }
    if (names > 16)
        return false;
    std::lock_guard lock(g_mutex);
    g_crouchExclTags = names == 0 ? std::string(kCrouchExclDefault) : norm; // "" = the default list
    if (g_dbLoaded)
    {
        ComputeStateTagMasksLocked();
        RecomputeDerivedClassDataLocked(false);
        LogCrouchExclLocked(g_dbName, "tags");
    }
    return true;
}

std::string CrouchExcludeTags()
{
    std::lock_guard lock(g_mutex);
    return g_crouchExclTags;
}

std::string CalibrationDigest()
{
    std::lock_guard lock(g_mutex);
    std::string out = "cal=\"" + g_cal.Summary() + "\" fam=\"" + g_famRates.Summary() + "\" cap=\"" + g_capProfile.Summary() +
                      "\" bandAvail=\"" + g_bandAvail.Summary() + "\"";
    char buf[256];
    std::snprintf(buf, sizeof buf, " famFactors=%016llx slowRef=%.6f/%.6f/%.6f bandEdges=%.6f/%.6f sprintData=%d",
                  Fnv1a64(g_famRates.clipFactor.data(), g_famRates.clipFactor.size() * sizeof(float)),
                  static_cast<double>(g_slowRef[kLocoStand]), static_cast<double>(g_slowRef[kLocoCrouch]),
                  static_cast<double>(g_slowRef[kLocoSprint]), static_cast<double>(g_bandEdges.walkRun),
                  static_cast<double>(g_bandEdges.runSprint), g_bandEdges.sprintData ? 1 : 0);
    out += buf;
    for (int st = 1; st < 4; ++st)
    {
        const SpeedPoints& m = g_speedMap[st];
        std::snprintf(buf, sizeof buf, " map%d=%d:%.6f>%.6f,%.6f>%.6f,%.6f>%.6f", st, m.count, static_cast<double>(m.game[0]),
                      static_cast<double>(m.db[0]), static_cast<double>(m.game[1]), static_cast<double>(m.db[1]),
                      static_cast<double>(m.game[2]), static_cast<double>(m.db[2]));
        out += buf;
    }
    return out;
}

void SetSteerFollow(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_steerOn = aOn;
    g_steer.Reset();
    g_latch.Reset();
}

bool SetSteerParams(float aEnterDeg, float aExitDeg, float aFollowTime, float aMaxFollowRateDeg)
{
    std::lock_guard lock(g_mutex);
    steer::SteerConfig sc = g_steer.Config();
    sc.enterAngle = aEnterDeg * kDegToRad;
    sc.exitAngle = aExitDeg * kDegToRad;
    sc.followTime = aFollowTime;
    bt::ControllerConfig cc = g_bodyCtl.Config();
    cc.followTime = aFollowTime;
    cc.maxFollowRate = aMaxFollowRateDeg * kDegToRad;
    if (!steer::ValidSteer(sc) || !bt::ValidConfig(cc) || aEnterDeg > 180.0f)
        return false;
    g_steer.SetConfig(sc);
    g_bodyCtl.SetConfig(cc);
    return true;
}

bool SetDwell(bool aOn, float aMinDwell, float aCostRatio)
{
    if (!std::isfinite(aMinDwell) || aMinDwell <= 0.0f || aMinDwell > 2.0f || !std::isfinite(aCostRatio) || aCostRatio <= 0.0f ||
        aCostRatio > 1.0f)
        return false;
    std::lock_guard lock(g_mutex);
    g_dwellOn = aOn;
    g_minDwell = aMinDwell;
    g_dwellRatio = aCostRatio;
    g_cfg.minDwell = aOn ? aMinDwell : 0.0f;
    g_cfg.dwellCostRatio = aCostRatio;
    if (g_matcher.Ready())
        g_matcher.SetDwell(g_cfg.minDwell, aCostRatio); // no re-init (playback continues)
    return true;
}

void SetCuration(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_curationOn = aOn;
}

void SetCurationMoving(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_curMovingOn = aOn;
}

void SetLargeTurn(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_largeTurnOn = aOn;
}

bool SetPivotAccel(float aStand, float aCrouch, float aSprint)
{
    for (float v : {aStand, aCrouch, aSprint})
        if (!std::isfinite(v) || v < 0.0f || v > 100.0f)
            return false;
    std::lock_guard lock(g_mutex);
    g_pivotAccel[kLocoStand] = aStand;
    g_pivotAccel[kLocoCrouch] = aCrouch;
    g_pivotAccel[kLocoSprint] = aSprint;
    return true;
}

bool SetStanceParams(float aMinHalfWidthCm, float aMinSeparationCm, bool aKneeIk)
{
    StanceGuardConfig c = g_stanceCfg;
    c.minHalfWidth = aMinHalfWidthCm * 0.01f;
    c.minSeparation = aMinSeparationCm * 0.01f;
    c.kneeIk = aKneeIk;
    c.kneeSwivel = aKneeIk; // knee-keeping IK and knee swivel together (0.3.8)
    if (!std::isfinite(aMinHalfWidthCm) || !std::isfinite(aMinSeparationCm) || !StanceGuard::Valid(c))
        return false;
    std::lock_guard lock(g_mutex);
    g_stanceCfg = c;
    if (g_stanceGuard.Ready())
        g_stanceGuard.SetConfig(c);
    return true;
}

void SetSlowWalkLoop(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_slowLoopOn = aOn;
}

bool SetStanceSmooth(float aSmoothTime)
{
    StanceGuardConfig c = g_stanceCfg;
    c.smoothTime = aSmoothTime;
    if (!std::isfinite(aSmoothTime) || !StanceGuard::Valid(c))
        return false;
    std::lock_guard lock(g_mutex);
    g_stanceCfg = c;
    if (g_stanceGuard.Ready())
    {
        g_stanceGuard.SetConfig(c);
        g_stanceGuard.Reset();
    }
    return true;
}

bool SetRootWarp(bool aOn, float aMaxDeg)
{
    if (!std::isfinite(aMaxDeg) || aMaxDeg <= 0.0f || aMaxDeg > 180.0f)
        return false;
    std::lock_guard lock(g_mutex);
    g_rootWarpOn = aOn;
    g_rootWarpMax = aMaxDeg * kDegToRad;
    return true;
}

void SetHeadingLean(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_leanOn = aOn;
}

void SetStanceSwing(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_stanceSwingOn = aOn;
    g_stanceGuard.Reset();
}

bool SetStanceGuard(bool aOn, float aMinHalfWidthCm)
{
    StanceGuardConfig c = g_stanceCfg;
    c.minHalfWidth = aMinHalfWidthCm * 0.01f;
    if (!std::isfinite(aMinHalfWidthCm) || !StanceGuard::Valid(c))
        return false;
    std::lock_guard lock(g_mutex);
    g_stanceOn = aOn;
    g_stanceCfg = c;
    if (g_stanceGuard.Ready())
        g_stanceGuard.SetConfig(c);
    return true;
}

bool SetSteerGap(float aGapDeg)
{
    if (!std::isfinite(aGapDeg) || aGapDeg < 0.0f || aGapDeg > 180.0f)
        return false;
    std::lock_guard lock(g_mutex);
    steer::SteerConfig sc = g_steer.Config();
    sc.maxFacingGap = aGapDeg * kDegToRad;
    return g_steer.SetConfig(sc);
}

void SetSlowWalkCapsule(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_slowCapOn = aOn;
    g_slowGate.Reset();
    g_slowGateActive = false;
}

void SetStartReface(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_startRefaceOn = aOn;
}

bool SetStopSync(bool aOn, float aStopHalflife)
{
    if (!std::isfinite(aStopHalflife) || aStopHalflife < 0.02f || aStopHalflife > 2.0f)
        return false;
    std::lock_guard lock(g_mutex);
    g_stopSyncOn = aOn;
    g_stopHalflife = aStopHalflife;
    g_cfg.stopHalflife = aOn ? aStopHalflife : 0.0f;
    if (g_matcher.Ready())
        g_matcher.GetTrajectory().SetStopHalflife(g_cfg.stopHalflife); // no re-init (playback continues)
    return true;
}

void SetAccelMatch(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_accelOn = aOn;
}

bool SetAccelParams(float aStand, float aCrouch, float aSprint)
{
    for (float v : {aStand, aCrouch, aSprint})
        if (!std::isfinite(v) || (v != 0.0f && (v < kMinAccelTarget || v > 100.0f)))
            return false;
    std::lock_guard lock(g_mutex);
    g_accelTarget[kLocoStand] = aStand;
    g_accelTarget[kLocoCrouch] = aCrouch;
    g_accelTarget[kLocoSprint] = aSprint;
    return true;
}

float AccelTarget(float aBaseAccel, float aAppliedMul)
{
    std::lock_guard lock(g_mutex);
    ++g_accelQueries;
    g_accelBase = std::isfinite(aBaseAccel) ? aBaseAccel : 0.0f;
    g_accelMul = std::isfinite(aAppliedMul) ? aAppliedMul : 1.0f;
    const int64_t now = Qpc();
    const float sinceFeed = g_lastFeedTicks ? static_cast<float>(static_cast<double>(now - g_lastFeedTicks) / QpcFreq()) : 1e9f;
    const int32_t loco = g_lastFeed.locoState;
    float t = 0.0f;
    // Only while MM drives the body: enabled, matcher ready, gate open (TPP, allowed state, no PSM block), fresh feed.
    if (g_accelOn && g_enabled && g_dbLoaded && g_matcher.Ready() && g_gate == 0 && sinceFeed <= kStaleFeedSeconds &&
        loco > 0 && loco < 4)
        t = (g_capSyncOn && g_accelTarget[loco] != 0.0f && g_capProfile.accel[loco] > 0.0f) ? g_capProfile.accel[loco] // 0.5.9
                                                                                              : g_accelTarget[loco];
    // 0.3.8: during an animation-led large turn (|target - body| > 90 deg, stick held) the capsule acceleration is lowered
    // to the pivot clips' redirect rate, so the capsule does not slide sideways / backwards ahead of the turning body.
    g_pivotAccelActive = false;
    if (g_largeTurnOn && g_accelOn && g_enabled && g_matcher.Ready() && g_gate == 0 && sinceFeed <= kStaleFeedSeconds &&
        loco > 0 && loco < 4 && g_haveTarget && g_steer.AnimLed() && g_steer.AbsError() > 1.5708f && g_pivotAccel[loco] > 0.0f)
    {
        t = t > 0.0f ? std::fmin(t, g_pivotAccel[loco]) : g_pivotAccel[loco];
        g_pivotAccelActive = true;
    }
    g_accelLastTarget = t;
    g_accelObserved = accel::AppliedAcceleration(aBaseAccel, aAppliedMul, t);
    g_accelSampleTicks = GroundBrakingDomain(g_lastFeed) && sinceFeed <= kStaleFeedSeconds &&
        g_accelObserved > 0.f ? now : 0;
    g_accelSampleLoco = g_lastFeed.locoState;
    g_accelSampleDetail = PsmDetailOf(g_lastFeed.psmBlock);
    if (t > 0.0f && t < g_accelBase)
        ++g_accelActive;
    return t;
}

void SetBodyTurn(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_bodyTurnOn = aOn;
    g_bodyCtl.Reset();
    g_latch.Reset();
}

bool SetBodyTurnParams(float aCorrectionTime, float aMinRateDeg, float aMaxCorrectionRateDeg, float aMaxRateDeg)
{
    std::lock_guard lock(g_mutex);
    bt::ControllerConfig c = g_bodyCtl.Config();
    c.correctionTime = aCorrectionTime;
    c.minCorrectionRate = aMinRateDeg * kDegToRad;
    c.maxCorrectionRate = aMaxCorrectionRateDeg * kDegToRad;
    c.maxRate = aMaxRateDeg * kDegToRad;
    return g_bodyCtl.SetConfig(c);
}

bool BodyTurnActive()
{
    std::lock_guard lock(g_mutex);
    return BodyTurnActiveLocked(Qpc());
}

float BodyYawCommand(float aCurrentYawDeg, float aBaseYawDeg)
{
    std::lock_guard lock(g_mutex);
    const int64_t now = Qpc();
    if (!BodyTurnActiveLocked(now) || !std::isfinite(aCurrentYawDeg))
    {
        ++g_bodyPassThrough;
        g_stSign = 0;
        return aBaseYawDeg;
    }
    // dt between commands (called once per frame while the body is turning); after a pause use the last feed dt.
    float dt = g_lastCmdTicks ? static_cast<float>(static_cast<double>(now - g_lastCmdTicks) / QpcFreq()) : 1e9f;
    if (g_cmdDtFromFeed) // 0.5.3 offline tests (faster than real time): command dt = feed dt
        dt = 1e9f;
    if (!(dt > 0.0f) || dt > 0.1f)
        dt = (std::isfinite(g_lastFeed.dt) && g_lastFeed.dt > 0.0f) ? std::fmin(g_lastFeed.dt, 0.05f) : (1.0f / 60.0f);
    g_lastCmdTicks = now;
    float target = g_targetYaw;
    g_stSign = 2;
    if (g_airSteerFed && !g_bodyTurnFed)
    {
        // Velocity-relative air yaw: the stick can request only a bounded lead around actual capsule momentum.
        // A controller rate cap alone cannot prevent a 180-degree mid-air spin when the velocity has barely turned.
        target = WrapPi(g_airSteerVelocityYaw + std::clamp(WrapPi(target - g_airSteerVelocityYaw),
                                                          -kAirSteerMaxLead, kAirSteerMaxLead));
        const float cmdA = g_bodyCtl.Command(aCurrentYawDeg * kDegToRad, target, dt, true, kAirSteerMaxRate);
        ++g_bodyCmds;
        ++g_airSteerCmds;
        g_airSteerMaxRateDeg = std::fmax(g_airSteerMaxRateDeg, std::fabs(g_bodyCtl.Diagnostics().rate) / kDegToRad);
        g_lastCmdCurDeg = aCurrentYawDeg;
        g_lastCmdBaseDeg = aBaseYawDeg;
        g_lastCmdDt = dt;
        g_lastCmdDeg = cmdA / kDegToRad;
        return g_lastCmdDeg;
    }
    if (g_motionHeldMatcher && g_motionStartTurnDirection != 0 && !g_followFed)
    {
        target = refacestart::ControllerTarget(aCurrentYawDeg * kDegToRad, target, g_motionStartTurnDirection);
    }
    else if (g_startBindOn && g_bindSign != 0 && !g_followFed)
    {
        // 0.5.8 start bind: the body turns in the bound start segment's root direction (the long way if needed) until the clip's
        // root turn is 90 % done; the animation step leads, the bounded fallback step only adds speed in the same direction (it
        // can no longer take the shortest way against the clip). Offline (start180, motion_plus v1/v2/v3): animation-only
        // binding gave |err| 61 deg 0.5 s after a 180 deg press (0.5.7: 49), the bound fallback 45 deg; no flip either way.
        const float cur = aCurrentYawDeg * kDegToRad;
        float eDir = WrapPi(g_targetYaw - cur) * float(g_bindSign);
        if (eDir < 0.0f)
            eDir += 2.0f * 3.14159265f;
        target = WrapPi(cur + float(g_bindSign) * std::fmin(eDir, 179.0f * kDegToRad));
        g_stSign = 1;
    }
    else if (g_startTurnOn && g_bodyFollowDir != 0 && !g_followFed)
    {
        // 0.5.7: near-reversal turn led by a clip turning the long way - aim 179 deg ahead in the clip's direction until the
        // remaining error is on that side (then the shortest way is the clip's way)
        const float cur = aCurrentYawDeg * kDegToRad;
        const float e = WrapPi(g_targetYaw - cur);
        if (e * float(g_bodyFollowDir) < 0.0f && std::fabs(e) >= 150.0f * kDegToRad)
            target = WrapPi(cur + float(g_bodyFollowDir) * 179.0f * kDegToRad);
    }
    const float cmd = g_bodyCtl.Command(aCurrentYawDeg * kDegToRad, target, dt, g_followFed); // 5.3c follow law
    ++g_bodyCmds;
    g_lastCmdCurDeg = aCurrentYawDeg;
    g_lastCmdBaseDeg = aBaseYawDeg;
    g_lastCmdDt = dt;
    g_lastCmdDeg = cmd / kDegToRad;
    return g_lastCmdDeg;
}

void SetTrace(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_trace = aOn;
}

void SetLogger(void (*aLog)(const char*))
{
    std::lock_guard lock(g_mutex);
    g_log = aLog;
}

void SetSimClock(SimClockFn aClock)
{
    g_simClockHook.store(aClock, std::memory_order_release);
}

void PublishGetUpGap()
{
    std::lock_guard lock(g_mutex);
    if (!ragdoll::GetUpOwns() || !g_lastFeed.tppActive || !PoseAvailableLocked())
        return;
#if !(defined(UTPP_MOTION_DRIVER_GAP_MUTATION) && UTPP_MOTION_DRIVER_GAP_MUTATION == 3) // mutation 3: no v1 publish after Begin (the game pose shows until the first feed)
    g_weight.Fill();
    PublishLocked(g_weight.Weight());
#endif
}

void Tick()
{
    // r4 landing: the sim clock is sampled before the driver lock (a script call must not run under g_mutex), only while a jump-out landing could be pending
    double simNow = 0.0;
    bool simSampled = false, simFailed = false;
    if (g_simWatch.load(std::memory_order_relaxed))
    {
        simSampled = ReadSimClock(simNow);
        simFailed = !simSampled;
    }
    std::unique_lock lock(g_mutex, std::try_to_lock);
    if (!lock.owns_lock())
        return; // the sample is dropped: the next Tick measures against the last sample that was used, so no sim time is lost
    const int64_t now = Qpc();
    const float dt = g_lastTickTicks ? static_cast<float>(static_cast<double>(now - g_lastTickTicks) / QpcFreq()) : 0.0f;
    g_lastTickTicks = now;
    // sim-time silence since the last REAL feed: accumulated from the sim deltas (0 in pause / menu / photo mode), reset by every real feed. A delta is never
    // larger than the wall time of the same interval (a wrong tick scale cannot run it fast). One failed sample ends the sim criterion until the next real feed
    // (the continuation then uses the wall-clock fallback; the two clocks are never mixed inside one episode).
    float simDt = 0.0f;
    g_simWatch.store(g_jumpOutExit || g_landDropout, std::memory_order_relaxed);
    if (g_lastRealFeedTicks != g_simSeenFeedTicks)
    {
        g_simSeenFeedTicks = g_lastRealFeedTicks;
        g_simSilence = 0.0f;
        g_simClockUsable = true;
    }
    if (simSampled)
    {
        if (g_simPrevValid)
            simDt = static_cast<float>(std::fmin(std::fmax(simNow - g_simPrev, 0.0), static_cast<double>(dt)));
        g_simPrev = simNow;
        g_simPrevValid = true;
    }
    else
        g_simPrevValid = false;
    if (simFailed)
        g_simClockUsable = false;
    if (g_simClockUsable && g_simSilence < 1e6f)
        g_simSilence += simDt;
    const float sinceFeed = g_lastFeedTicks ? static_cast<float>(static_cast<double>(now - g_lastFeedTicks) / QpcFreq()) : 1e9f;
    const bool stale = sinceFeed > kStaleFeedSeconds;
    const bool off = !g_enabled || g_hot.fault.load(std::memory_order_acquire) || !g_hot.armed.load(std::memory_order_acquire);
    // r3b landing (see kLandDropoutS): the descent of a jump-out flight reached its predicted touchdown and the landing state gives no locomotion update -> feed the
    // touchdown from here, so the gate, the MM clock and the landing clip run from the contact instead of from the end of the game's landing state
    // mutation 25: Tick never feeds the landing (pre-r3b: gate and clock stall)
    if (UTPP_AND_NOT(VEHEXIT, 25) !off && g_dbLoaded && g_lastFeed.tppActive && g_lastRealFeedTicks != 0)
    {
        const float sinceReal = static_cast<float>(static_cast<double>(now - g_lastRealFeedTicks) / QpcFreq());
        // the silence criterion: sim seconds since the last real feed (cap kLandDropoutMaxS, step = the sim delta), without a sim clock the wall time (short cap)
        float silence = g_simClockUsable ? g_simSilence : sinceReal;
        float cap = g_simClockUsable ? kLandDropoutMaxS : kLandDropoutWallMaxS;
        float stepDt = g_simClockUsable ? simDt : dt;
        UTPP_IF_MUT(VEHEXIT, 27) // mutation 27: the wall clock decides (a pause invents a second of landing, the sim clock is ignored)
        {
            silence = sinceReal;
            cap = kLandDropoutMaxS;
            stepDt = dt;
        }
        const bool overdue = g_jumpOutExit && g_airDec.phase == air::Phase::LandApproach && g_airDec.ttlKnown && g_ttlAtFeed >= 0.0f &&
                             g_lastFeed.haveAir && !g_lastFeed.onGround && std::isfinite(g_lastFeed.velWorld[2]) &&
                             g_lastFeed.velWorld[2] < -kJumpOutHoldFallMs && silence >= g_ttlAtFeed + kLandDropoutS;
        if ((g_landDropout || overdue) && silence <= cap && stepDt > 0.0f) // stepDt 0: paused (the sim clock stands still) - nothing to feed, the episode does not progress
        {
            if (!g_landDropout)
            {
                ++g_landDropoutEpisodes;
                if (g_log)
                {
                    char line[260];
                    std::snprintf(line, sizeof line,
                                  "UTPP LANDFEED no locomotion update %.3f s after the last feed (%s clock: sim %.3f s, wall %.3f s; predicted touchdown in %.3f s, vz=%.2f): Tick feeds the touchdown",
                                  static_cast<double>(silence), g_simClockUsable ? "sim" : "wall", static_cast<double>(g_simSilence), static_cast<double>(sinceReal),
                                  static_cast<double>(g_ttlAtFeed), static_cast<double>(g_lastFeed.velWorld[2]));
                    g_log(line);
                }
            }
            g_landDropout = true;
            FeedInput f = g_lastFeed;
            f.synthetic = true;
            f.dt = std::fmin(stepDt, 0.05f);
            f.haveAir = true;
            f.onGround = true;
            f.distToGround = -2.0f;
            f.psmFall = 0;
            f.locoState = kLocoStand; // the state that follows the game's landing state (feed dump 297.669: Stand, zero velocity)
            // the first feed after the silence reports no detailed state (0x61002000): the Fall detail 14 (0x610F2040) of the last air rows would keep the gate at 0x110 and v2 in the air
            // mutation 26: the synthetic landing keeps the Fall detail of the last air feed
            UTPP_IF_NOT_MUT(VEHEXIT, 26)
                f.psmBlock &= ~(kPsmDetailFieldMask | uint32_t(kPsmDetailed));
            f.velWorld[0] = f.velWorld[1] = f.velWorld[2] = 0.0f;
            g_synthGuardTicks = g_lastRealFeedTicks; // Feed() drops this synthetic feed when a real one ran between the unlock and its own lock
            g_synthGuardOn = true;
            lock.unlock();
            Feed(f);
            return;
        }
        if (silence > cap)
            g_landDropout = false;
    }
    if (g_weight.Weight() <= 0.0f && CandidateReadyLocked())
        TrySwapLocked(); // 0.5.1: switch while MM is idle / disabled / not fed
    if (!stale && !off)
        return; // Feed() drives the weight
    // 0.6.2 hold last pose: no feed (MM disabled, locomotion PSM not ticking: menu, pause, load) in TPP outside the release states
    // -> keep the body on the last MM pose (republished every tick so the FK detour never sees a stale snapshot)
    {
        HoldInput hi;
        hi.on = g_holdPoseOn;
        hi.tpp = g_lastFeed.tppActive;
        hi.poseOk = PoseAvailableLocked();
        hi.gate = g_gate | (stale ? uint32_t(kGateStaleFeed) : 0u) | (!g_enabled ? uint32_t(kGateMMDisabled) : 0u);
        hi.psm = g_psmBlock;
        hi.psmDetail = g_lastDetail;
        hi.release = g_releaseReasons; // CLIMB RELEASE stage: the last feed's release reasons
        // CAR-1: a stale feed inside the exit window keeps the latched exit workspot held; the window keeps running without feeds
        if (stale && g_exitLeft > 0.0f)
        {
            g_exitLeft = std::fmax(0.0f, g_exitLeft - std::fmin(dt, 0.25f));
            if (g_exitLeft <= 0.0f)
            {
                g_exitLatchedPsm = 0;
                // g_jumpOutExit is NOT cleared here: a feed dropout around the touchdown (the move case holds the pose for 0.6-0.8 s while the vanilla landing state
                // does not feed) must keep the jump-out state; the Feed side ends it (0.10 s of contact, or the cap after the window closed)
                UTPP_IF_MUT(VEHEXIT, 24) // mutation 24: the stale path clears the jump-out state with the closed window
                    g_jumpOutExit = false;
            }
        }
        hi.exitWindow = g_exitLeft > 0.0f;
        hi.exitLatchedPsm = g_exitLatchedPsm;
        if (HoldEligible(hi) && g_weight.Weight() > 0.0f)
        {
            if (!g_holdActive)
                ++g_holdEpisodes;
            g_holdActive = true;
            ++g_holdTicks;
            g_weight.ClearHold();
            PublishLocked(g_weight.Update(0u, std::fmin(dt, 0.25f), g_fade));
            g_target = g_weight.Target();
            return;
        }
    }
    g_holdActive = false;
    if (g_weight.Progress() <= 0.0f)
    {
        // 0.3.2: a stale feed (locomotion PSM not ticking: vehicle, scene...) still arms the reopen hold, so MM does not
        // fade in on the very first feed after it resumes (dt 0: weight unchanged).
        if (stale)
            g_weight.Update(kGateStaleFeed, 0.0f, g_fade);
        if (g_hot.mmActive.load(std::memory_order_relaxed))
            PublishLocked(0.0f);
        return;
    }
    if (stale)
    {
        g_gate |= kGateStaleFeed;
        ++g_staleFades;
        g_havePrevYaw = false;
        g_footLock.Reset(); // integrated capsule position is no longer valid
        ResetOffsetLocked();
        g_chestAlign.Reset(); // B2 strafe S2: the pose stream restarts after a stale feed
    }
    if (!g_enabled)
        g_gate |= kGateMMDisabled;
    g_gatePrev = g_gate;
    g_effGate = g_gate; // 0.6.2: stale / disabled = full close (the mask fade holds while the whole body fades out)
    g_maskReasons = g_maskCovered = 0;
    PublishLocked(g_weight.Update(g_gate != 0 ? g_gate : kGateStaleFeed, std::fmin(dt, 0.25f), g_fade));
    g_target = g_weight.Target();
}

void SetEnabled(bool aEnabled)
{
    std::lock_guard lock(g_mutex);
    if (g_enabled != aEnabled) g_accelSampleTicks = 0;
    g_enabled = aEnabled;
    if (aEnabled && g_dbLoaded)
        InitMatcherLocked(); // fresh state (idle start)
}

bool SetTagFilter(const std::string& aSubstring)
{
    std::lock_guard lock(g_mutex);
    if (!g_dbLoaded)
    {
        g_tagFilter = aSubstring;
        return false;
    }
    const uint64_t mask = aSubstring.empty() ? ~0ull : BaseMatcher::TagMaskContaining(g_db, aSubstring);
    if (mask == 0)
        return false;
    const uint64_t prev = g_tagMask;
    g_tagMask = mask;
    if (!InitMatcherLocked())
    {
        g_tagMask = prev;
        InitMatcherLocked();
        return false;
    }
    g_tagFilter = aSubstring;
    g_appliedTagMask = g_tagMask;
    return true;
}

void SetBlendTime(float aSeconds)
{
    std::lock_guard lock(g_mutex);
    if (std::isfinite(aSeconds) && aSeconds >= 0.0f && aSeconds <= 5.0f)
        g_fade.blendTime = aSeconds;
}

bool SetGateFade(float aHardOutSeconds, float aReopenHoldSeconds)
{
    if (!std::isfinite(aHardOutSeconds) || !std::isfinite(aReopenHoldSeconds) || aHardOutSeconds < 0.0f ||
        aHardOutSeconds > 5.0f || aReopenHoldSeconds < 0.0f || aReopenHoldSeconds > 5.0f)
        return false;
    std::lock_guard lock(g_mutex);
    g_fade.hardOut = aHardOutSeconds;
    g_fade.reopenHold = aReopenHoldSeconds;
    return true;
}

void SetAllowedStates(uint32_t aMask)
{
    std::lock_guard lock(g_mutex);
    g_allowedMask = aMask;
}

bool WantsFeed()
{
    std::lock_guard lock(g_mutex);
    // 0.6.2: while MM is disabled but the body is held (hold last pose), the feed keeps reporting the PSM state so a release
    // state (vehicle, scene, ...) or TPP off still hands the body back to the game; MM itself stays off (no matcher update)
    return g_dbLoaded && (g_enabled || (g_holdPoseOn && g_holdActive));
}

int32_t JumpOutLaunchEpoch()
{
    std::lock_guard lock(g_mutex);
    return g_jumpOutLaunchEpoch;
}

std::string Status()
{
    std::lock_guard lock(g_mutex);
    const MatcherDiagnostics* d = g_matcher.Ready() ? &g_matcher.Diagnostics() : nullptr;
    const int32_t clip = d ? d->clip : -1;
    const std::string_view clipName = (d && clip >= 0) ? g_db.ClipName(uint32_t(clip)) : std::string_view{};
    const FootLockDiagnostics& fl = g_footLock.Diagnostics();
    char buf[5120];
    std::snprintf(
        buf, sizeof buf,
        " | MM enabled=%d db=%d dbReason=\"%s\" dbPath=\"%s\" map=\"%s\" mappedBones=%u rig=0x%llX tagFilter=\"%s\" "
        "tagMask=0x%llX allowedStates=0x%X blendTime=%.3f weight=%.3f target=%.0f gate=0x%X clip=%d(%.*s) frame=%d "
        "clipFrame=%d searches=%llu transitions=%llu lastSearchUs=%.1f lastUpdateUs=%.1f updates=%llu feedCalls=%llu "
        "lastDt=%.4f velWorld=(%.2f,%.2f,%.2f) velChar=(%.2f,%.2f) move=(%.2f,%.2f) playerYaw=%.1f camYaw=%.1f "
        "yawRate=%.2f loco=%d tpp=%d publishes=%llu staleFades=%llu mmApplied=%llu mmBones=%llu torn=%llu stale=%llu "
        "rigMismatch=%llu footLock=%d flReady=%d lockL=%d wL=%.2f corrL=%.1fcm lockR=%d wR=%.2f corrR=%.1fcm "
        "flMaxCorr=%.1fcm flLocks=%llu flRelocks=%llu flDriftUnlocks=%llu flClamps=%llu flNonFinite=%llu flUs=%.1f "
        "bodyTurn=%d btActive=%d btFed=%d haveTarget=%d targetYaw=%.1f animYawRate=%.1f rawAnimYawRate=%.1f btCmds=%llu "
        "btPass=%llu btAnimLed=%llu btNonFinite=%llu lastCmd=%.1f lastCur=%.1f lastBase=%.1f lastErr=%.1f lastCmdRate=%.1f "
        "lastCmdDt=%.4f latched=%d latchHolds=%llu psmBlock=0x%X hardOut=%.3f reopenHold=%.3f holdPending=%d "
        "fadeHard=%d gateCloses=%llu psmBlockFeeds=%llu offsetRoot=%d offReady=%d offX=%.1fcm offY=%.1fcm offYaw=%.1f "
        "offMax=%.1fcm offMaxYaw=%.1f offClamps=%llu offYawClamps=%llu offResets=%llu offTeleports=%llu offNonFinite=%llu "
        "offHalflife=%.3f/%.3f offLimit=%.3f/%.1f accelMatch=%d accelTarget=%.2f accelBase=%.2f accelMul=%.3f "
        "accelTargets=%.2f/%.2f/%.2f accelQueries=%llu accelActive=%llu rootRate=%d rate=%.3f rateTarget=%.3f "
        "rootRateActive=%d loopRate=%.3f rootSpeedDb=%.2f rootRateCfg=%.2f/%.2f/%.2f/%.3f stopSync=%d stopHalflife=%.3f slowWalk=%d walkRateMin=%.2f floored=%d "
        "offSatSeconds=%.2f offFastDecays=%llu offSatCfg=%.2f/%.3f startReface=%d excludeLands=%d standMask=0x%llX "
        "steerFollow=%d followFed=%d animLed=%d steerErr=%.1f targetRate=%.1f steerEntries=%llu steerExits=%llu btFollowCmds=%llu "
        "steerCfg=%.0f/%.0f/%.3f/%.0f dwell=%d minDwell=%.2f dwellRatio=%.2f dwellHolds=%llu dwellExempts=%llu "
        "sinceTrans=%.2f curation=%d exclude=0x%X filteredSearches=%llu slowCapsule=%d slowGate=%d slowAccel=%.2f curationMoving=%d steerGap=%.1f "
        "stanceGuard=%d stanceReady=%d stanceHalfWidth=%.1fcm stancePushL=%.1fcm stancePushR=%.1fcm stanceMaxPush=%.1fcm stanceActive=%llu "
        "rootWarp=%d rootWarpMax=%.0f headingLean=%d offYawGoal=%.1f stanceSwing=%d largeTurn=%d pivotAccel=%.1f/%.1f/%.1f "
        "pivotAccelActive=%d stanceSep=%.1fcm kneeIk=%d slowWalkLoop=%d stanceSmooth=%.3f swivelL=%.1f swivelR=%.1f",
        g_enabled ? 1 : 0, g_dbLoaded ? 1 : 0, g_dbReason.c_str(), g_dbPath.c_str(), g_mapReason.c_str(), g_map.mapped,
        static_cast<unsigned long long>(g_rig), g_tagFilter.c_str(), static_cast<unsigned long long>(g_tagMask), g_allowedMask,
        static_cast<double>(g_fade.blendTime), static_cast<double>(g_weight.Weight()), static_cast<double>(g_target), g_gate,
        clip, int(clipName.size()), clipName.data() ? clipName.data() : "", d ? d->frame : -1, d ? d->clipFrame : -1,
        static_cast<unsigned long long>(d ? d->searches : 0), static_cast<unsigned long long>(d ? d->transitions : 0),
        d ? d->lastSearchMicros : 0.0, g_lastUpdateUs, static_cast<unsigned long long>(g_matcherUpdates),
        static_cast<unsigned long long>(g_feedCalls), static_cast<double>(g_lastFeed.dt),
        static_cast<double>(g_lastFeed.velWorld[0]), static_cast<double>(g_lastFeed.velWorld[1]),
        static_cast<double>(g_lastFeed.velWorld[2]), static_cast<double>(g_lastCharVel[0]),
        static_cast<double>(g_lastCharVel[1]), static_cast<double>(g_lastFeed.move[0]),
        static_cast<double>(g_lastFeed.move[1]), static_cast<double>(g_lastFeed.playerYawDeg),
        static_cast<double>(g_lastFeed.cameraYawDeg), static_cast<double>(g_lastYawRate), g_lastFeed.locoState,
        g_lastFeed.tppActive ? 1 : 0, static_cast<unsigned long long>(g_publishes),
        static_cast<unsigned long long>(g_staleFades),
        static_cast<unsigned long long>(g_hot.mmAppliedCalls.value.load()),
        static_cast<unsigned long long>(g_hot.mmBonesWritten.value.load()),
        static_cast<unsigned long long>(g_hot.mmTornReads.value.load()),
        static_cast<unsigned long long>(g_hot.mmStaleSkips.value.load()),
        static_cast<unsigned long long>(g_hot.mmRigMismatch.value.load()), g_footLockOn ? 1 : 0, g_footLock.Ready() ? 1 : 0,
        fl.foot[0].locked ? 1 : 0, static_cast<double>(fl.foot[0].weight), static_cast<double>(fl.foot[0].correction) * 100.0,
        fl.foot[1].locked ? 1 : 0, static_cast<double>(fl.foot[1].weight), static_cast<double>(fl.foot[1].correction) * 100.0,
        static_cast<double>(fl.maxCorrection) * 100.0, static_cast<unsigned long long>(fl.locks),
        static_cast<unsigned long long>(fl.relocks), static_cast<unsigned long long>(fl.driftUnlocks),
        static_cast<unsigned long long>(fl.clamps), static_cast<unsigned long long>(fl.nonFiniteResets), g_lastFootLockUs,
        g_bodyTurnOn ? 1 : 0, BodyTurnActiveLocked(Qpc()) ? 1 : 0, g_bodyTurnFed ? 1 : 0, g_haveTarget ? 1 : 0,
        static_cast<double>(g_targetYaw / kDegToRad), static_cast<double>(g_bodyCtl.AnimRate() / kDegToRad),
        static_cast<double>(g_rawAnimRate / kDegToRad), static_cast<unsigned long long>(g_bodyCmds),
        static_cast<unsigned long long>(g_bodyPassThrough),
        static_cast<unsigned long long>(g_bodyCtl.Diagnostics().animLedCommands),
        static_cast<unsigned long long>(g_bodyCtl.Diagnostics().nonFinite), static_cast<double>(g_lastCmdDeg),
        static_cast<double>(g_lastCmdCurDeg), static_cast<double>(g_lastCmdBaseDeg),
        static_cast<double>(g_bodyCtl.Diagnostics().error / kDegToRad),
        static_cast<double>(g_bodyCtl.Diagnostics().rate / kDegToRad), static_cast<double>(g_lastCmdDt),
        g_latch.latched ? 1 : 0, static_cast<unsigned long long>(g_latch.holds), g_psmBlock,
        static_cast<double>(g_fade.hardOut), static_cast<double>(g_fade.reopenHold), g_weight.HoldPending() ? 1 : 0,
        g_weight.HardOut() ? 1 : 0, static_cast<unsigned long long>(g_gateCloses),
        static_cast<unsigned long long>(g_psmBlockFeeds), g_offsetRootOn ? 1 : 0, g_offsetRoot.Ready() ? 1 : 0,
        static_cast<double>(g_offsetRoot.X() * 100.0f), static_cast<double>(g_offsetRoot.Y() * 100.0f),
        static_cast<double>(g_offsetRoot.Yaw() / kDegToRad),
        static_cast<double>(g_offsetRoot.Diagnostics().maxDistance * 100.0f),
        static_cast<double>(g_offsetRoot.Diagnostics().maxYaw / kDegToRad),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().clamps),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().yawClamps),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().resets),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().teleports),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().nonFinite),
        static_cast<double>(g_offsetCfg.translationHalflife), static_cast<double>(g_offsetCfg.rotationHalflife),
        static_cast<double>(g_offsetCfg.maxDistance), static_cast<double>(g_offsetCfg.maxYaw / kDegToRad),
        g_accelOn ? 1 : 0, static_cast<double>(g_accelLastTarget), static_cast<double>(g_accelBase),
        static_cast<double>(g_accelMul), static_cast<double>(g_accelTarget[1]), static_cast<double>(g_accelTarget[2]),
        static_cast<double>(g_accelTarget[3]), static_cast<unsigned long long>(g_accelQueries),
        static_cast<unsigned long long>(g_accelActive), g_rootRateOn ? 1 : 0, static_cast<double>(g_lastRate),
        static_cast<double>(g_rootRate.Target()), g_rootRate.Active() ? 1 : 0, static_cast<double>(g_loopRate), static_cast<double>(g_rootSpeedDb),
        static_cast<double>(g_rootRate.Config().rateMin), static_cast<double>(g_rootRate.Config().rateMax),
        static_cast<double>(g_rootRate.Config().epsilon), static_cast<double>(g_rootRate.Config().halflife),
        g_stopSyncOn ? 1 : 0, static_cast<double>(g_stopSyncOn ? g_stopHalflife : 0.0f), g_slowWalkOn ? 1 : 0,
        static_cast<double>(g_walkRateMin), g_lastFloored ? 1 : 0,
        static_cast<double>(g_offsetRoot.Diagnostics().saturatedSeconds),
        static_cast<unsigned long long>(g_offsetRoot.Diagnostics().fastDecays),
        static_cast<double>(g_offsetCfg.saturatedTime), static_cast<double>(g_offsetCfg.saturatedHalflife),
        g_startRefaceOn ? 1 : 0, g_excludeLands ? 1 : 0, static_cast<unsigned long long>(g_stateTagMask[kLocoStand]),
        g_steerOn ? 1 : 0, g_followFed ? 1 : 0, g_steer.AnimLed() ? 1 : 0, static_cast<double>(g_steer.AbsError() / kDegToRad),
        static_cast<double>(g_steer.TargetRate() / kDegToRad), static_cast<unsigned long long>(g_steer.Entries()),
        static_cast<unsigned long long>(g_steer.Exits()), static_cast<unsigned long long>(g_bodyCtl.Diagnostics().followCommands),
        static_cast<double>(g_steer.Config().enterAngle / kDegToRad), static_cast<double>(g_steer.Config().exitAngle / kDegToRad),
        static_cast<double>(g_bodyCtl.Config().followTime), static_cast<double>(g_bodyCtl.Config().maxFollowRate / kDegToRad),
        g_dwellOn ? 1 : 0, static_cast<double>(g_minDwell), static_cast<double>(g_dwellRatio),
        static_cast<unsigned long long>(d ? d->dwellHolds : 0), static_cast<unsigned long long>(d ? d->dwellExempts : 0),
        static_cast<double>(d ? d->sinceTransition : 0.0f), g_curationOn ? 1 : 0, g_lastExclude,
        static_cast<unsigned long long>(d ? d->filteredSearches : 0), g_slowCapOn ? 1 : 0, g_slowGateActive ? 1 : 0,
        static_cast<double>(g_slowGate.Accel()), g_curMovingOn ? 1 : 0,
        static_cast<double>(g_steer.Config().maxFacingGap / kDegToRad), g_stanceOn ? 1 : 0, g_stanceGuard.Ready() ? 1 : 0,
        static_cast<double>(g_stanceCfg.minHalfWidth * 100.0f), static_cast<double>(g_stanceGuard.Diagnostics().push[0] * 100.0f),
        static_cast<double>(g_stanceGuard.Diagnostics().push[1] * 100.0f),
        static_cast<double>(g_stanceGuard.Diagnostics().maxPush * 100.0f),
        static_cast<unsigned long long>(g_stanceGuard.Diagnostics().activeUpdates), g_rootWarpOn ? 1 : 0,
        static_cast<double>(g_rootWarpMax / kDegToRad), g_leanOn ? 1 : 0, static_cast<double>(g_offsetRoot.YawGoal() / kDegToRad),
        g_stanceSwingOn ? 1 : 0, g_largeTurnOn ? 1 : 0, static_cast<double>(g_pivotAccel[1]), static_cast<double>(g_pivotAccel[2]),
        static_cast<double>(g_pivotAccel[3]), g_pivotAccelActive ? 1 : 0, static_cast<double>(g_stanceCfg.minSeparation * 100.0f),
        g_stanceCfg.kneeIk ? 1 : 0, g_slowLoopOn ? 1 : 0, static_cast<double>(g_stanceCfg.smoothTime),
        static_cast<double>(g_stanceGuard.Diagnostics().swivel[0] / kDegToRad),
        static_cast<double>(g_stanceGuard.Diagnostics().swivel[1] / kDegToRad));
    std::string out = buf;
    out += " dbName=" + g_dbName + " dbSha=" + (g_dbSha.size() >= 12 ? g_dbSha.substr(0, 12) : g_dbSha);
    {
        std::lock_guard lk(g_sw.m);
        out += " dbSwitch=\"" + g_sw.status + "\"";
    }
    out += " dbSwitches=" + std::to_string(g_dbSwitches) + " dbSwitchFailures=" + std::to_string(g_dbSwitchFailures);
    out += " speedCal=\"" + g_cal.Summary() + "\"";
    char st[512];
    std::snprintf(st, sizeof st, " stanceWindow=%.2f sinceStance=%.2f stanceChanges=%llu stickFilter=%d stickHeld=%d stickBridged=%llu "
                  "stickLifted=%llu familyRate=%d rateFactor=%.3f slowQuery=%.2f/%.2f startRules=%d families=%d",
                  static_cast<double>(g_curCfg.stanceWindow), static_cast<double>(std::fmin(g_sinceStance, 999.0f)),
                  static_cast<unsigned long long>(g_stanceChanges), g_stickFilterOn ? 1 : 0, g_stickHeld ? 1 : 0,
                  static_cast<unsigned long long>(g_stick.Bridged()), static_cast<unsigned long long>(g_stick.Lifted()),
                  g_familyRateOn ? 1 : 0, static_cast<double>(g_rateFactor), static_cast<double>(g_slowRef[kLocoStand]),
                  static_cast<double>(g_slowRef[kLocoCrouch]), g_curCfg.startRules ? 1 : 0, g_famRates.families);
    out += st;
    std::snprintf(st, sizeof st, " speedBands=%d band=%d bandEdges=%.2f/%.2f bandChanges=%llu bandData=\"%s\" sprintData=%d idleSpeed=%.2f",
                  g_curCfg.speedBands ? 1 : 0, g_lastBand, static_cast<double>(g_bandEdges.walkRun),
                  static_cast<double>(g_bandEdges.runSprint), static_cast<unsigned long long>(g_band.Changes()),
                  g_bandAvail.Summary().c_str(), g_bandEdges.sprintData ? 1 : 0, static_cast<double>(g_curCfg.idleSpeed));
    out += st;
    std::snprintf(st, sizeof st, " gateDebounce=%.3f rawGate=0x%X debounced=%llu debounceClosed=%llu psmDetail=%d softDetail=%d "
                  "crouchRules=%d lateralOk=%d/%d lateralExcl=%llu",
                  static_cast<double>(g_gateDebounceCfg.holdSeconds), g_rawGate,
                  static_cast<unsigned long long>(g_gateDebounce.Suppressed()),
                  static_cast<unsigned long long>(g_gateDebounce.Confirmed()), g_lastPsmDetail, g_lastSoftDetail,
                  g_curCfg.crouchRules ? 1 : 0, g_lateralOk[kLocoStand] ? 1 : 0, g_lateralOk[kLocoCrouch] ? 1 : 0,
                  static_cast<unsigned long long>(g_lateralExcluded));
    out += st;
    std::snprintf(st, sizeof st, " crouchDirTag=%d stanceOnce=%d stanceUsed=%d turnCommit=%d committed=%d commitHolds=%llu standIdle=%d "
                  "standIdleSearches=%llu startTurn=%d classSet=%d againstFeeds=%llu startRate=%d startRateFeeds=%llu",
                  g_crouchDirTagOn ? 1 : 0, g_stanceOnceOn ? 1 : 0, g_stanceUsed ? 1 : 0, g_turnCommitOn ? 1 : 0, g_turnCommitted ? 1 : 0,
                  static_cast<unsigned long long>(g_turnCommitHolds), g_standIdleOn ? 1 : 0,
                  static_cast<unsigned long long>(g_standIdleSearches), g_startTurnOn ? 1 : 0, g_activeClassSet,
                  static_cast<unsigned long long>(g_againstSearches), g_startRateActive ? 1 : 0,
                  static_cast<unsigned long long>(g_startRateFeeds));
    out += st;
    std::snprintf(st, sizeof st, " turnRateOn=%d turnRateActive=%d turnRateW=%.3f turnRateTarget=%.3f turnRateCfg=%.2f/%.2f turnRateFeeds=%llu",
                  g_turnRateOn ? 1 : 0, g_turnRateActive ? 1 : 0, static_cast<double>(g_turnRateW), static_cast<double>(g_turnRateTarget),
                  static_cast<double>(g_turnRateMin), static_cast<double>(g_turnRateMax), static_cast<unsigned long long>(g_turnRateFeeds)); // 0.6.3
    out += st;
    {
        const MatcherDiagnostics& cd = g_matcher.Diagnostics();
        const SpeedPoints& sm = g_speedMap[kLocoStand];
        const float sprint = (sm.count >= 3 && sm.game[2] > 0.0f) ? sm.game[2] : g_stateTargetSpeed[kLocoSprint];
        std::snprintf(st, sizeof st, " speedCap=%.2f speedCapMs=%.2f overSpeed=%d overSpeedFeeds=%llu queryClamped=%llu costCap=%.1f "
                      "costCapHold=%.2f costCapHits=%llu costCapHolds=%llu costCapRetries=%llu costCapForced=%llu costCapNever=%llu "
                      "stanceCap=%d commitNotAllowed=%llu turnSettle=%d settling=%d settleHolds=%llu",
                      static_cast<double>(g_speedCapFactor), static_cast<double>(g_speedCapFactor * sprint), g_overSpeed ? 1 : 0,
                      static_cast<unsigned long long>(g_overSpeedFeeds), static_cast<unsigned long long>(g_queryClamped),
                      static_cast<double>(g_costCap), static_cast<double>(g_costCapHold),
                      static_cast<unsigned long long>(cd.costCapHits), static_cast<unsigned long long>(cd.costCapHolds),
                      static_cast<unsigned long long>(cd.costCapRetries), static_cast<unsigned long long>(cd.costCapForced),
                      static_cast<unsigned long long>(cd.costCapNever), g_stanceCapOn ? 1 : 0,
                      static_cast<unsigned long long>(g_commitNotAllowed), g_turnSettleOn ? 1 : 0, g_settling ? 1 : 0,
                      static_cast<unsigned long long>(g_settleHolds));
        out += st;
        std::snprintf(st, sizeof st, " loopBands=%d loopBand=%d-%d loopAvail=w%d/r%d/s%d loopBandSearches=%llu startBind=%d bindSign=%d "
                      "bindFeeds=%llu binds=%llu bindCommitSearches=%llu bindReversals=%llu stSign=%s",
                      g_loopBandsOn ? 1 : 0, g_lastLoopBand, g_lastLoopBandHi, g_loopAvail[1], g_loopAvail[2], g_loopAvail[3],
                      static_cast<unsigned long long>(g_loopBandSearches), g_startBindOn ? 1 : 0, g_bindSign,
                      static_cast<unsigned long long>(g_bindFeeds), static_cast<unsigned long long>(g_binds),
                      static_cast<unsigned long long>(g_bindCommitSearches), static_cast<unsigned long long>(g_bindReversals),
                      g_stSign == 1 ? "clip" : (g_stSign == 2 ? "fallback" : "off"));
        out += st;
        // 0.5.9
        const OffsetRootDiagnostics& od = g_offsetRoot.Diagnostics();
        char s9[1024];
        std::snprintf(s9, sizeof s9, " capsuleSync=%d decelTarget=%.2f decelBase=%.2f decelMul=%.3f decelQueries=%llu decelActive=%llu "
                      "capAccel=%.2f/%.2f/%.2f capDecel=%.2f/%.2f/%.2f offCatchup=%d catchCfg=%.2f/%.2f/%.2f/%.3f offSoftS=%.2f offSatS=%.2f "
                      "offFar=%.2f offFarMax=%.2f offSoftDecays=%llu footSlip=%.3f/%.3f footSlips=%llu trajCal=%d trajHl=%.3f trajCalFeeds=%llu",
                      g_capSyncOn ? 1 : 0, static_cast<double>(g_decelLastTarget), static_cast<double>(g_decelBase),
                      static_cast<double>(g_decelMul), static_cast<unsigned long long>(g_decelQueries),
                      static_cast<unsigned long long>(g_decelActive), static_cast<double>(g_capProfile.accel[1]),
                      static_cast<double>(g_capProfile.accel[2]), static_cast<double>(g_capProfile.accel[3]),
                      static_cast<double>(g_capProfile.decel[1]), static_cast<double>(g_capProfile.decel[2]),
                      static_cast<double>(g_capProfile.decel[3]), g_catchupOn ? 1 : 0, static_cast<double>(g_catchMaxDistance),
                      static_cast<double>(g_catchTime), static_cast<double>(g_catchSoft), static_cast<double>(g_catchHalflife),
                      static_cast<double>(od.softSeconds), static_cast<double>(od.saturatedSeconds), static_cast<double>(od.farTime),
                      static_cast<double>(od.farTimeMax), static_cast<unsigned long long>(od.softDecays),
                      static_cast<double>(g_catchupOn ? g_slipStart : 0.0f), static_cast<double>(g_slipHalflife),
                      static_cast<unsigned long long>(g_footLock.Diagnostics().slips), g_trajCalOn ? 1 : 0,
                      static_cast<double>(g_trajHlApplied), static_cast<unsigned long long>(g_trajCalFeeds));
        out += s9;
        // 0.6.0
        char s6[640];
        std::snprintf(s6, sizeof s6, " meleeGate=%d meleeBlocks=%llu meleeState=%d meleeActive=%d meleeBits=0x%X meleeFeeds=%llu meleeBlips=%llu meleeRelease=%.2f "
                      "upperBody=%d mountCut=%d mountCuts=%llu mountEventCuts=%llu mountMuted=%d stanceClipMin=%.2f stanceClipSkips=%llu "
                      "stanceClipAllows=%llu stanceClipOk=%d stanceClipSpeed=%.2f stanceClipStick=%.2f stanceClipExcl=%llu",
                      g_meleeCfg.enabled ? 1 : 0, static_cast<unsigned long long>(g_meleeGate.Episodes()), g_meleeState,
                      g_meleeActive ? 1 : 0, g_meleeBits, static_cast<unsigned long long>(g_meleeGate.Feeds()),
                      static_cast<unsigned long long>(g_meleeGate.Suppressed()), static_cast<double>(g_meleeCfg.releaseHold), g_upperBody, g_mountCutOn ? 1 : 0,
                      static_cast<unsigned long long>(g_mountCuts), static_cast<unsigned long long>(g_mountEventCuts), g_mountMuted ? 1 : 0,
                      static_cast<double>(g_stanceClipMin), static_cast<unsigned long long>(g_stanceClipSkips),
                      static_cast<unsigned long long>(g_stanceClipAllows), g_stanceClipOk ? 1 : 0, static_cast<double>(g_stanceClipSpeed),
                      static_cast<double>(g_stanceClipStickFwd), static_cast<unsigned long long>(g_stanceClipExcl));
        out += s6;
        // 0.6.2
        char s62[640];
        std::snprintf(s62, sizeof s62, " upperMask=%d maskStates=0x%X maskOk=%d maskBones=%u maskLower=%u maskActive=0x%X maskCovered=0x%X maskBlend=%.2f "
                      "upperWeight=%.3f effGate=0x%X maskSwitches=%llu maskFeeds=%llu exitWindow=%.2f exitWindows=%llu exitDiag=%u "
                      "holdPose=%d holdPoseActive=%d holdPoseFrames=%llu holdPoseTicks=%llu holdEpisodes=%llu holdSwaps=%llu poseValid=%d",
                      g_upperMaskOn ? 1 : 0, g_maskStates, g_upperMask.ok ? 1 : 0, g_upperMask.upper, g_upperMask.lower, g_maskReasons,
                      g_maskCovered, static_cast<double>(g_maskBlend.Weight()),
                      static_cast<double>(g_weight.Weight() * (g_upperMask.ok ? 1.0f - g_maskBlend.Weight() : 1.0f)), g_effGate,
                      static_cast<unsigned long long>(g_maskSwitches), static_cast<unsigned long long>(g_maskFeeds),
                      static_cast<double>(g_exitLeft), static_cast<unsigned long long>(g_exitWindows), g_exitDiagLines, g_holdPoseOn ? 1 : 0,
                      g_holdActive ? 1 : 0, static_cast<unsigned long long>(g_holdFrames), static_cast<unsigned long long>(g_holdTicks),
                      static_cast<unsigned long long>(g_holdEpisodes), static_cast<unsigned long long>(g_holdSwaps),
                      g_poseValid ? 1 : 0);
        out += s62;
        char sJump[420];
        std::snprintf(sJump, sizeof sJump, " jumpOuts=%llu jumpOutFeeds=%llu jumpOutActive=%d jumpOutFills=%llu jumpOutExit=%d jumpOutSlewFrames=%llu jumpOutLimited=%llu"
                      " jumpOutRearms=%llu vehPops=%llu vehPopWrites=%llu rearmHoldFrames=%llu landDropouts=%llu landDropoutFeeds=%llu",
                      static_cast<unsigned long long>(g_jumpOut.Fired()), static_cast<unsigned long long>(g_jumpOutFeeds),
                      g_jumpOut.Active() ? 1 : 0, static_cast<unsigned long long>(g_jumpOutFills), g_jumpOutExit ? 1 : 0, static_cast<unsigned long long>(g_jumpOutSlewFrames),
                      static_cast<unsigned long long>(g_jumpOutLimiter.LimitedBones()), static_cast<unsigned long long>(g_jumpOutRearms),
                      static_cast<unsigned long long>(vehtrans::g_counters.pops.load()),
                      static_cast<unsigned long long>(vehtrans::g_counters.popWrites.load()),
                      static_cast<unsigned long long>(vehtrans::g_counters.rearmHoldFrames.load()),
                      static_cast<unsigned long long>(g_landDropoutEpisodes), static_cast<unsigned long long>(g_landDropoutFeeds));
        out += sJump;
        char sWeapon[120];
        std::snprintf(sWeapon, sizeof sWeapon, " weaponUpper=%d weaponKnown=%d weaponHeld=%d weaponUpperActive=%d",
                      g_weaponUpperOn ? 1 : 0, g_weaponKnown ? 1 : 0, g_weaponHeld ? 1 : 0, g_weaponUpperActive ? 1 : 0);
        out += sWeapon;
        // CLIMB RELEASE stage
        char sCr[220];
        std::snprintf(sCr, sizeof sCr, " climbRelease=%d travRelease=%d releaseReasons=0x%X travReleaseFeeds=%llu travReleaseEpisodes=%llu travOwnedFeeds=%llu",
                      g_climbReleaseOn ? 1 : 0, (g_releaseReasons & kReleaseTrav) ? 1 : 0, g_releaseReasons,
                      static_cast<unsigned long long>(g_travReleaseFeeds), static_cast<unsigned long long>(g_travReleaseEpisodes),
                      static_cast<unsigned long long>(g_travOwnedFeeds));
        out += sCr;
        // vehicle batch W1 P2: mount transitions begun / exits handed in from a capture / exits without a fresh capture / aborts
        // (review fix: same-rig re-arms kept in a ride / exit, vehTransKept=)
        char sVt[160];
        std::snprintf(sVt, sizeof sVt, " vehTrans=%llu/%llu/%llu/%llu vehTransActive=%d vehTransKept=%llu",
                      static_cast<unsigned long long>(g_vtMounts), static_cast<unsigned long long>(g_vtExitsHanded),
                      static_cast<unsigned long long>(g_vtExitsNoCapture), static_cast<unsigned long long>(g_vtAborts),
                      vehtrans::Wanted() || vehtrans::HandInActive() ? 1 : 0, static_cast<unsigned long long>(g_vtRearmsKept));
        out += sVt;
        out += " capProfile=\"" + g_capProfile.Summary() + "\"";
        // loco P0: only while UTPPSetMMDerivedRecompute is on (off = the 0.6.5b status text)
        if (g_derivedRecomputeOn)
        {
            char sp0[160];
            std::snprintf(sp0, sizeof sp0, " derivedRecompute=1 derivedRecomputes=%llu derivedStale=%llu",
                          static_cast<unsigned long long>(g_derivedRecomputes), static_cast<unsigned long long>(g_derivedStale));
            out += sp0;
        }
        // loco C0 / C1a: only while UTPPSetMMFeetDiag or UTPPSetMMCrouchIdleExclude is on (both off = the loco P0 status text);
        // loco C1b: also while replant is on for some stance, its fields only then
        if (g_feetDiagOn || g_crouchExclOn || g_footReplantMask != 0)
        {
            char sc[320];
            std::snprintf(sc, sizeof sc,
                          " feet=\"diag=%d crouchExcl=%d exclActive=%d exclMask=0x%llX exclClips=%d exclFound=%d exclMissing=%d "
                          "exclSkipped=%d crouchIdleLoops=%d footEvents=%llu",
                          g_feetDiagOn ? 1 : 0, g_crouchExclOn ? 1 : 0, g_crouchExclMask != 0 ? 1 : 0,
                          static_cast<unsigned long long>(g_crouchExclMask), g_crouchExclClips, g_crouchExclFound,
                          g_crouchExclMissing, g_crouchExclSkipped ? 1 : 0, g_crouchIdleLoops,
                          static_cast<unsigned long long>(g_feetEvents));
            out += sc;
            out += " exclTags=" + g_crouchExclTags;
            if (g_footReplantMask != 0)
            {
                const FootLockDiagnostics& fd = g_footLock.Diagnostics();
                const FootReplantConfig& rc = g_replantCfg;
                char sr[480];
                std::snprintf(sr, sizeof sr,
                              " replant=%d replantActive=%d replantCfg=%.3f/%.3f/%.2f/%.2f/%.2f/%.2f/%.3f steps=%llu stepsRadius=%llu "
                              "stepsClamp=%llu stepsTidy=%llu stepsHandover=%llu handovers=%llu stepUpdates=%llu simSteps=%llu maxLift=%.1fcm",
                              g_footReplantMask, g_footLock.ReplantOn() ? 1 : 0, static_cast<double>(rc.radius),
                              static_cast<double>(rc.radiusStops), static_cast<double>(rc.ratio), static_cast<double>(rc.ratioStops),
                              static_cast<double>(rc.stepSpeed), static_cast<double>(rc.stepSpeedStops), static_cast<double>(rc.lift),
                              static_cast<unsigned long long>(fd.footSteps[0] + fd.footSteps[1]),
                              static_cast<unsigned long long>(fd.stepReasons[kFootEventRadius]),
                              static_cast<unsigned long long>(fd.stepReasons[kFootEventClamp]),
                              static_cast<unsigned long long>(fd.stepReasons[kFootEventTidy]),
                              static_cast<unsigned long long>(fd.stepReasons[kFootEventHandover]),
                              static_cast<unsigned long long>(fd.handovers), static_cast<unsigned long long>(fd.stepUpdates),
                              static_cast<unsigned long long>(fd.simultaneousSteps), static_cast<double>(fd.maxLift) * 100.0);
                out += sr;
            }
            if (g_feetDiagOn)
            {
                char feet[512];
                FeetFragmentLocked(feet, sizeof feet);
                out += " ";
                out += feet;
            }
            out += "\"";
        }
    }
    // batch 1b: only once a feed dump was started (never started = the loco C1b status text)
    if (g_feedDump.Sessions() > 0)
        out += " feedDump=\"" + g_feedDump.Status() + "\"";
    // stage standstill idle: only while UTPPSetMMIdleHold is on (off = the batch 1b status text)
    if (g_idleHoldOn)
    {
        char si[160];
        std::snprintf(si, sizeof si, " idleHold=1 idleHoldStill=%.2f idleHoldSearches=%llu idleHoldNoLoop=%llu",
                      static_cast<double>(std::fmin(g_idleHoldStill, 99.0f)), static_cast<unsigned long long>(g_idleHoldSearches),
                      static_cast<unsigned long long>(g_idleHoldNoLoop));
        out += si;
    }
    // batch 2 knee fix: only while UTPPSetMMKneeFix is not 0 (0 = the batch 1b status text)
    if (g_kneeFixMask != 0)
        out += " kneeFix=" + std::to_string(g_kneeFixMask);
    // B2 strafe S1: only while UTPPSetMMStrafe is on or the mode still runs (off = the knee-fix status text)
    if (g_strafeOn || g_strafeMode.On())
    {
        const strafe::StrafeProfiles& p = g_strafeProf;
        char ss[512];
        std::snprintf(ss, sizeof ss,
                      " strafe=%d strafeOn=%d strafeState=%d strafeEp=%llu strafeExits=%llu strafeBlips=%llu strafeFrames=%llu "
                      "strafeAvail=%d/%d strafeSectors=%d/%d/%d strafeGait=%d strafeDir=%.0f strafeRateSrc=%d strafeSearches=%llu "
                      "strafeSUs=%.0f/%.0f strafeCfg=%d/%.1f/%.1f/%.2f/%.2f strafeSet=%u/%u/%u",
                      g_strafeMode.On() ? 1 : 0, g_strafeOn ? 1 : 0, static_cast<int>(g_strafeMode.State()),
                      static_cast<unsigned long long>(g_strafeMode.Entries()), static_cast<unsigned long long>(g_strafeMode.Exits()),
                      static_cast<unsigned long long>(g_strafeMode.Blips()), static_cast<unsigned long long>(g_strafeMode.Frames()),
                      p.avail[0] ? 1 : 0, p.avail[1] ? 1 : 0, strafe::SectorCount(p.sectors[0][0]), strafe::SectorCount(p.sectors[0][1]),
                      strafe::SectorCount(p.sectors[1][0]), g_strafeGait, static_cast<double>(g_strafeDirDeg), g_strafeRateSrc,
                      static_cast<unsigned long long>(g_strafeSearches),
                      g_strafeSearches > 0 ? g_strafeSUsSum / static_cast<double>(g_strafeSearches) : 0.0, g_strafeSUsMax,
                      g_strafeCfg.unarmed ? 1 : 0, static_cast<double>(g_strafeCfg.rootTurnMaxDeg),
                      static_cast<double>(g_strafeCfg.rootWarpMaxDeg), static_cast<double>(g_strafeCfg.exitHold),
                      static_cast<double>(g_strafeJogF), p.facingFixedLoops, p.patternsKept[0], p.rootTurners);
        out += ss;
        // B2 strafe S2 chest align (appended to the strafe segment): chestValid=0 -> dev / low / corr are 0 (no run this feed)
        const ChestAlignDiagnostics& cd = g_chestAlign.Diagnostics();
        char cs[448];
        // chestBlend / chestBlends / chestStanceBlend (chest_repair): crouch fraction of the reference this feed (stance-switch
        // blend), feeds with a blend in progress, the configured longest blend (s)
        std::snprintf(cs, sizeof cs,
                      " chestAlign=%d chestReady=%d chestRef=%d/%d chestValid=%d chestStance=%d chestDev=%.2f chestLow=%.2f chestCorr=%.2f "
                      "chestW=%.3f chestUpd=%llu chestAct=%llu chestNoRef=%llu chestNonFinite=%llu chestCfg=%.2f/%.1f chestBlend=%.3f "
                      "chestBlends=%llu chestStanceBlend=%.2f",
                      g_chestOn ? 1 : 0, g_chestAlign.Ready() ? 1 : 0, g_chestAlign.HasReference(ChestAlign::kStand) ? 1 : 0,
                      g_chestAlign.HasReference(ChestAlign::kCrouch) ? 1 : 0, g_chestValid ? 1 : 0, cd.stance,
                      static_cast<double>(g_chestValid ? cd.dev : 0.0f), static_cast<double>(g_chestValid ? cd.low : 0.0f),
                      static_cast<double>(g_chestValid ? cd.delta : 0.0f), static_cast<double>(g_chestValid ? cd.weight : 0.0f),
                      static_cast<unsigned long long>(cd.updates), static_cast<unsigned long long>(cd.activeUpdates),
                      static_cast<unsigned long long>(cd.noReference), static_cast<unsigned long long>(cd.nonFinite),
                      static_cast<double>(g_chestCfg.swayKeep), static_cast<double>(g_chestCfg.chestMaxDeg),
                      static_cast<double>(g_chestValid ? cd.blend : 0.0f), static_cast<unsigned long long>(cd.blendUpdates),
                      static_cast<double>(g_chestCfg.stanceBlendTime));
        out += cs;
    }
    // AIR HOLD stage: air segment (appended; only while UTPPSetMMAir is not off). air=<effective mode> airReason=<why> airPhase=
    // episodes / take-offs / falls / lands / hand-backs / resets, last TTL error (s, + = late), gUp / gDown (m/s^2), probes
    // (accepted / rejected by AirState; raycasts fed / hits), feeds owned, searches suppressed, airUs last / max (us).
    if (g_airSetting != 3)
    {
        const air::AirStats& ast = g_air.Stats();
        static constexpr const char* kSet[] = {"auto", "clips", "hold", "off"};
        char as[768];
        std::snprintf(as, sizeof as,
                      " air=%s airReason=%s:%s airPhase=%s airOwns=%d airEpisodes=%u airTakeoffs=%u airFalls=%u airLands=%u "
                      "airHandbacks=%u airResets=%u airTtlErr=%.3f airG=%.1f/%.1f airProbes=%u/%u airRay=%llu/%llu airProbeOn=%d "
                      "airSteer=%d airFeeds=%llu/%llu airOwned=%llu airSuppressed=%llu airUs=%.1f/%.1f airSteerFed=%d airSteerFeeds=%llu "
                      "airSteerCmds=%llu airSteerMaxRate=%.1f airSteerBlocked=%llu",
                      (g_airSetting == 2 || !g_airTableValid) ? "hold" : "clips", kSet[g_airSetting >= 0 && g_airSetting < 3 ? g_airSetting : 0],
                      g_airTableReason.c_str(), air::PhaseName(g_air.CurrentPhase()),
                      g_airOwns ? 1 : 0, ast.episodes, ast.takeoffs, ast.falls, ast.lands, ast.handbacks, ast.resets,
                      static_cast<double>(ast.lastTtlErrValid ? ast.lastTtlErr : 0.0f), static_cast<double>(g_air.GUp()),
                      static_cast<double>(g_air.GDown()), ast.probesAccepted, ast.probesRejected, static_cast<unsigned long long>(g_airProbes),
                      static_cast<unsigned long long>(g_airProbeHits), g_airProbeOn ? 1 : 0, g_airSteerOn ? 1 : 0,
                      static_cast<unsigned long long>(g_airFreshFeeds), static_cast<unsigned long long>(g_airFeeds),
                      static_cast<unsigned long long>(g_airOwnedFeeds), static_cast<unsigned long long>(g_airSuppressed), g_airUs, g_airUsMax,
                      g_airSteerFed ? 1 : 0, static_cast<unsigned long long>(g_airSteerFeeds),
                      static_cast<unsigned long long>(g_airSteerCmds), static_cast<double>(g_airSteerMaxRateDeg),
                      static_cast<unsigned long long>(g_airSteerBlocked));
        out += as;
        { // BATCH 6 AIR stage (appended): physics fall / touchdown guard
            char ar[160];
            std::snprintf(ar, sizeof ar, " airRobust=%d airPhysFalls=%u airTouchRejects=%u airLandAborts=%u airRetakeBlocks=%u",
                          g_air.Config().physFall ? 1 : 0, ast.physFalls, ast.touchRejects, ast.landAborts, ast.retakeBlocks);
            out += ar;
        }
        if (g_airTableValid) // AIR CLIPS stage (appended): applied / metas, loop override, plays / failures, query overrides, last clip
        {
            const char* lastClip = "-";
            std::string lc;
            if (g_airPlayMeta >= 0 && size_t(g_airPlayMeta) < g_airTable.Meta().size())
            {
                lc = std::string(g_db.ClipName(uint32_t(g_airTable.Meta()[size_t(g_airPlayMeta)].clip)));
                lastClip = lc.c_str();
            }
            char cs[256];
            std::snprintf(cs, sizeof cs, " airTable=%u/%zu airLoop=%d airPlays=%llu/%llu airQuery=%llu airClip=%s airLandAlign=%llu airLandBlend=%llu/%.3f airBlendHipsErr=%.4f",
                          g_airTable.CountStatus(airtab::ClipStatus::Applied), g_airTable.Meta().size(), g_airTable.LoopOk() ? 1 : 0,
                          static_cast<unsigned long long>(g_airPlays), static_cast<unsigned long long>(g_airPlayFails),
                          static_cast<unsigned long long>(g_airQueryOverrides), lastClip, static_cast<unsigned long long>(g_airLandRealigns),
                          static_cast<unsigned long long>(g_airLandGroundBlends), static_cast<double>(g_airLandGroundBlendLeft),
                          static_cast<double>(g_airLandGroundFirstHipsError));
            out += cs;
            // AIR CLIPS repair (appended): table applied to the matcher (root_z on), fall loop override active in the matcher
            const int32_t lc2 = g_airTable.FallLoopClip();
            std::snprintf(cs, sizeof cs, " airRootZ=%d airLoopOv=%d", g_matcher.HasRootZ() ? 1 : 0,
                          (lc2 >= 0 && size_t(lc2) < g_db.Clips().size() && g_matcher.Ready() && g_matcher.IsLoopClip(lc2) &&
                           (g_db.Clips()[size_t(lc2)].flags & kClipLoop) == 0)
                              ? 1
                              : 0);
            out += cs;
        }
    }
    // B2 strafe S4 jog: only while the jog gait is configured (jogF > 0; 0 = the S2 status text). aimJogT = last UTPPMMAimJogTarget
    // answer (m/s, 0 = vanilla aim speed), queries / non-zero answers.
    if (g_strafeJogF > 0.0f)
    {
        char js[128];
        std::snprintf(js, sizeof js, " aimJogT=%.2f aimJogQ=%llu aimJogOn=%llu", static_cast<double>(g_aimJogLastTarget),
                      static_cast<unsigned long long>(g_aimJogQueries), static_cast<unsigned long long>(g_aimJogOnAnswers));
        out += js;
    }
    {
        // b5 testorder repair (appended): database swap playback reset switch, swaps that reset, PlaybackCarryLocked bits of the
        // last swap before / after it
        char sr[96];
        std::snprintf(sr, sizeof sr, " swapReset=%d swapResets=%llu swapCarry=0x%X swapPost=0x%X", g_swapResetOn ? 1 : 0,
                      static_cast<unsigned long long>(g_swapResets), g_swapCarry, g_swapPost);
        out += sr;
    }
    return out;
}

// ---- 0.5.1 runtime database switching (public) ----
void SetDatabaseDir(const std::wstring& aDir)
{
    std::lock_guard lock(g_mutex);
    g_dbDir = aDir;
    if (!g_dbDir.empty() && g_dbDir.back() != L'\\' && g_dbDir.back() != L'/')
        g_dbDir += L'\\';
}

bool IsValidDatabaseName(const std::string& aName)
{
    if (aName.empty() || aName.size() > 64 || aName[0] == '.')
        return false;
    for (const char c : aName)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.'))
            return false;
    return true;
}

std::string ListDatabases()
{
    std::wstring dir;
    {
        std::lock_guard lock(g_mutex);
        dir = g_dbDir;
    }
    std::string out;
    if (dir.empty())
        return out;
    if (GetFileAttributesW((dir + L"data\\utpp_base.utppmm").c_str()) != INVALID_FILE_ATTRIBUTES)
        out = "default";
    std::vector<std::string> names;
    WIN32_FIND_DATAW fd{};
    HANDLE h = FindFirstFileW((dir + L"data\\*.utppmm").c_str(), &fd);
    if (h != INVALID_HANDLE_VALUE)
    {
        do
        {
            if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
                continue;
            std::string n = Narrow(fd.cFileName);
            if (n.size() > 7 && _stricmp(n.c_str() + n.size() - 7, ".utppmm") == 0)
                n.resize(n.size() - 7);
            // utpp_motion_<body> are the per-body motion data (chooser driven), not base databases: never offered as a choice
            if (IsValidDatabaseName(n) && n != "default" && n != "utpp_base" && n.rfind("utpp_motion_", 0) != 0)
                names.push_back(n);
        } while (FindNextFileW(h, &fd));
        FindClose(h);
    }
    std::sort(names.begin(), names.end(), [](const std::string& a, const std::string& b) { return _stricmp(a.c_str(), b.c_str()) < 0; });
    for (const std::string& n : names)
        out += (out.empty() ? "" : ";") + n;
    return out;
}

bool RequestDatabase(const std::string& aName)
{
    if (!IsValidDatabaseName(aName))
        return false;
    std::wstring path;
    std::vector<uint64_t> expect;
    {
        std::lock_guard lock(g_mutex);
        if (g_dbDir.empty())
            return false;
        path = DbPathFor(aName);
        if (g_dbLoaded && aName == g_dbName && !CandidateReadyLocked())
        {
            std::lock_guard lk(g_sw.m);
            if (!g_sw.busy)
            {
                g_sw.status = "ok " + aName;
                return true;
            }
        }
        if (g_dbLoaded)
            expect = g_dbHashes;
    }
    if (GetFileAttributesW(path.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        std::lock_guard lk(g_sw.m);
        g_sw.status = "FAILED " + aName + ": file not found";
        ++g_dbSwitchFailures;
        return false;
    }
    std::lock_guard lk(g_sw.m);
    if (g_sw.busy)
        return false;
    if (g_sw.worker.joinable())
        g_sw.worker.join(); // previous worker has finished (busy == false)
    g_sw.ready.reset();     // a newer request replaces a pending candidate
    g_sw.busy = true;
    g_sw.status = "loading " + aName;
    g_sw.worker = std::thread(WorkerLoad, aName, path, std::move(expect));
    return true;
}

std::string DatabaseName()
{
    std::lock_guard lock(g_mutex);
    return g_dbName;
}

std::string DatabaseSha()
{
    std::lock_guard lock(g_mutex);
    return g_dbSha;
}

void SetStickFilter(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_stickFilterOn = aOn;
    g_stick.Reset();
}

void SetCommandDtFromFeed(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_cmdDtFromFeed = aOn;
}

void SetFamilyRate(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_familyRateOn = aOn;
}

void SetStartRules(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_curCfg.startRules = aOn;
}

void SetSpeedBands(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_curCfg.speedBands = aOn;
    g_band.Reset();
    g_lastBand = 0;
}

// ---- 0.5.6 kill switches (default on; off = 0.5.5)
bool SetGateDebounce(float aSeconds)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aSeconds) || aSeconds < 0.0f || aSeconds > 0.5f)
        return false;
    g_gateDebounceCfg.holdSeconds = aSeconds;
    g_gateDebounce.Reset();
    return true;
}

void SetCrouchRules(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_curCfg.crouchRules = aOn;
}

// ---- 0.5.7 kill switches (default on; off = 0.5.6)
void SetCrouchDirTag(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_crouchDirTagOn = aOn;
    if (g_dbLoaded && g_matcher.Ready())
    {
        for (uint32_t c = 0; c < g_db.NumClips() && c < g_clipClasses.size(); ++c)
            g_clipClasses[c] = ClipClassOfLocked(c);
        ComputeTurnProfilesLocked();
        g_matcher.SetClipClasses(g_matchClasses[0]); // no playback reset
        ComputeLateralOkLocked();
        RecomputeDerivedClassDataLocked(false); // loco P0 (loop bands / lateral rule just rebuilt: no change; strafe profiles hook)
    }
}

void SetStanceOnce(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_stanceOnceOn = aOn;
    g_stanceUsed = false;
    // The first-frames window of the stance transitions is a matcher frame range: it follows this switch at the next matcher
    // init (UTPPSetMMEnabled(true), database load / switch); no re-init here (playback continues).
}

void SetTurnCommit(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_turnCommitOn = aOn;
}

void SetStandIdle(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_standIdleOn = aOn;
}

void SetIdleHold(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_idleHoldOn = aOn;
}

bool IdleHold()
{
    std::lock_guard lock(g_mutex);
    return g_idleHoldOn;
}

// ---- 0.6.3
void SetTurnRate(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_turnRateOn = aOn;
    g_turnRateW = 0.0f;
    g_turnRateActive = false;
    g_turnRecentSpeed = 0.0f;
    g_turnAcc = TurnRateAcc{}; // 0.6.4
}

bool SetTurnRateParams(float aMin, float aMax)
{
    if (!std::isfinite(aMin) || !std::isfinite(aMax) || aMin < 0.5f || aMax > 2.0f || aMin > aMax)
        return false;
    std::lock_guard lock(g_mutex);
    g_turnRateMin = aMin;
    g_turnRateMax = aMax;
    return true;
}

// ---- B2 strafe S1
void SetStrafe(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_strafeOn = aOn; // off: the running mode leaves through Exiting at the next feeds (edge resets), then the 0.6.5 path
}

void SetChestAlign(bool aOn)
{
    std::lock_guard lock(g_mutex);
    if (aOn != g_chestOn)
        g_chestAlign.Reset(); // both ways: a later run starts from the current pose
    g_chestOn = aOn;
}

bool SetChestAlignParams(float aSwayKeep, float aChestMaxDeg)
{
    if (!std::isfinite(aSwayKeep) || !std::isfinite(aChestMaxDeg) || aSwayKeep < 0.0f || aSwayKeep > 1.0f || aChestMaxDeg < 0.0f ||
        aChestMaxDeg > 90.0f)
        return false;
    std::lock_guard lock(g_mutex);
    ChestAlignConfig c = g_chestCfg;
    c.swayKeep = aSwayKeep;
    c.chestMaxDeg = aChestMaxDeg;
    if (!ChestAlign::Valid(c))
        return false;
    g_chestCfg = c;
    if (g_chestAlign.Ready())
        g_chestAlign.SetConfig(c); // filters kept: the new target is reached through the correction smoothing
    return true;
}

bool SetChestAlignStanceBlend(float aSeconds)
{
    if (!std::isfinite(aSeconds) || aSeconds < 0.0f || aSeconds > 5.0f)
        return false;
    std::lock_guard lock(g_mutex);
    ChestAlignConfig c = g_chestCfg;
    c.stanceBlendTime = aSeconds;
    if (!ChestAlign::Valid(c))
        return false;
    g_chestCfg = c;
    if (g_chestAlign.Ready())
        g_chestAlign.SetConfig(c);
    return true;
}

bool ChestAlignEnabled()
{
    std::lock_guard lock(g_mutex);
    return g_chestOn;
}

// ---- AIR HOLD stage (2026-10-13) ----
bool SetAir(int32_t aMode)
{
    if (aMode < 0 || aMode > 3)
        return false;
    std::lock_guard lock(g_mutex);
    if (aMode != g_airSetting)
    {
        g_air.RequestReset(); // a running episode ends (Reset on the next feed)
        g_airHoldGroundReplantLeft = 0.0f;
    }
    g_airSetting = aMode;
    ApplyAirTableLocked(); // AIR CLIPS repair: the table applies to the matcher only under auto / clips
    return true;
}

bool SetAirParams(int32_t aTakeoffLeadFrames, float aLandLeadSeconds, float aRateMin, float aRateMax, float aRecoverLightSeconds,
                  float aRecoverHeavySeconds, float aHoldRate, float aHeavyVz)
{
    const auto fin = [](float v) { return std::isfinite(v); };
    if (!fin(aLandLeadSeconds) || !fin(aRateMin) || !fin(aRateMax) || !fin(aRecoverLightSeconds) || !fin(aRecoverHeavySeconds) ||
        !fin(aHoldRate) || !fin(aHeavyVz))
        return false;
    if (aTakeoffLeadFrames < 0 || aTakeoffLeadFrames > 30 || aLandLeadSeconds < 0.0f || aLandLeadSeconds > 1.0f || aRateMin < 0.1f ||
        aRateMax < aRateMin || aRateMax > 4.0f || aRecoverLightSeconds < 0.0f || aRecoverLightSeconds > 2.0f ||
        aRecoverHeavySeconds < 0.0f || aRecoverHeavySeconds > 2.0f || aHoldRate < 0.0f || aHoldRate > 2.0f || aHeavyVz < 1.0f ||
        aHeavyVz > 50.0f)
        return false;
    std::lock_guard lock(g_mutex);
    air::AirConfig c = g_air.Config();
    c.takeoffLeadFrames = aTakeoffLeadFrames;
    c.landLead = aLandLeadSeconds;
    c.rateMin = aRateMin;
    c.rateMax = aRateMax;
    c.recoverLight = aRecoverLightSeconds;
    c.recoverHeavy = aRecoverHeavySeconds;
    c.holdRate = aHoldRate;
    c.heavyVz = aHeavyVz;
    g_air.SetConfig(c);
    return true;
}

void SetAirProbe(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_airProbeOn = aOn;
}

void SetAirRobust(bool aOn)
{
    std::lock_guard lock(g_mutex);
    air::AirConfig c = g_air.Config();
    c.physFall = c.touchGuard = aOn; // BATCH 6 AIR stage: read at the next feed
    g_air.SetConfig(c);
}

void SetAirSteer(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_airSteerOn = aOn; // AIR STEER stage: read at the next feed (g_airSteerFed)
    if (!aOn)
        g_airSteerFed = false;
}

bool AirWantsProbe()
{
    std::lock_guard lock(g_mutex);
    if (!g_airProbeOn || g_airSetting == 3 || !g_airLastAirborne || g_airLastVz > 0.0f || g_airLastRelease)
        return false;
    if ((g_feedCalls & 1u) != 0)
        return false; // every 2nd feed
    ++g_airProbeWants;
    return true;
}

#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
bool AirOwnsForTest()
{
    std::lock_guard lock(g_mutex);
    return g_airOwns;
}
#endif

std::string ChestAlignDigest()
{
    std::lock_guard lock(g_mutex);
    char buf[320];
    const math::Quat s = g_chestAlign.Reference(ChestAlign::kStand), c = g_chestAlign.Reference(ChestAlign::kCrouch);
    std::snprintf(buf, sizeof buf, "ready=%d hips=%d spine=%d ref=%d/%d stand=(%.6f,%.6f,%.6f,%.6f) crouch=(%.6f,%.6f,%.6f,%.6f) buf=%zu",
                  g_chestAlign.Ready() ? 1 : 0, g_chestAlign.HipsBone(), g_chestAlign.SpineBone(),
                  g_chestAlign.HasReference(ChestAlign::kStand) ? 1 : 0, g_chestAlign.HasReference(ChestAlign::kCrouch) ? 1 : 0,
                  static_cast<double>(s.x), static_cast<double>(s.y), static_cast<double>(s.z), static_cast<double>(s.w),
                  static_cast<double>(c.x), static_cast<double>(c.y), static_cast<double>(c.z), static_cast<double>(c.w), g_chestPose.size());
    return buf;
}

bool SetStrafeParams(bool aUnarmed, float aRootTurnMaxDeg, float aRootWarpMaxDeg, float aExitHold, float aJogF)
{
    if (!std::isfinite(aRootTurnMaxDeg) || !std::isfinite(aRootWarpMaxDeg) || !std::isfinite(aExitHold) || !std::isfinite(aJogF) ||
        aRootTurnMaxDeg < 1.0f || aRootTurnMaxDeg > 90.0f || aRootWarpMaxDeg < 0.0f || aRootWarpMaxDeg > 90.0f || aExitHold < 0.0f ||
        aExitHold > 1.0f || !strafe::JogSpeedValid(aJogF))
        return false;
    std::lock_guard lock(g_mutex);
    const bool reprofile = aRootTurnMaxDeg != g_strafeCfg.rootTurnMaxDeg || (aJogF > 0.0f) != (g_strafeJogF > 0.0f);
    g_strafeCfg.unarmed = aUnarmed;
    g_strafeCfg.rootTurnMaxDeg = aRootTurnMaxDeg;
    g_strafeCfg.rootWarpMaxDeg = aRootWarpMaxDeg;
    g_strafeCfg.exitHold = aExitHold;
    g_strafeJogF = aJogF;
    if (reprofile && g_dbLoaded && g_matcher.Ready())
        RecomputeDerivedClassDataLocked(false); // strafe class set 3 rebuilt (active set 3 -> matcher back on the base set)
    return true;
}

void SetStrafePatterns(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_strafePatterns = aOn;
}

float AimJogTarget(float aMoveX, float aMoveY)
{
    std::lock_guard lock(g_mutex);
    ++g_aimJogQueries;
    const int64_t now = Qpc();
    const float sinceFeed = g_lastFeedTicks ? static_cast<float>(static_cast<double>(now - g_lastFeedTicks) / QpcFreq()) : 1e9f;
    float t = 0.0f;
    // Only while the strafe mode is Active (aiming, weapon in hand, stand) in the jog gait and MM drives the body (fresh feed).
    if (g_strafeOn && g_strafeJogF > 0.0f && g_enabled && g_dbLoaded && g_matcher.Ready() && sinceFeed <= kStaleFeedSeconds &&
        (g_strafeMode.State() == strafe::StrafeState::Active || g_aimJogMotionHeld) && // UTPP-MOTION SEAM (v1: Active only)
        g_strafeGait == strafe::kGaitRun)
        t = strafe::JogTargetSpeed(g_strafeJogF, aMoveX, aMoveY);
    g_aimJogLastTarget = t;
    if (t > 0.0f)
        ++g_aimJogOnAnswers;
    return t;
}

std::string StrafeDataDigest()
{
    std::lock_guard lock(g_mutex);
    const strafe::StrafeProfiles& p = g_strafeProf;
    std::string out;
    char buf[200];
    std::snprintf(buf, sizeof buf, "avail=%d/%d gait=%d%d/%d%d sectors=%02X/%02X/%02X/%02X loops=%u turners=%u kept=%u/%u off=%u/%u set3=%016llx speeds=",
                  p.avail[0] ? 1 : 0, p.avail[1] ? 1 : 0, p.availGait[0][0] ? 1 : 0, p.availGait[0][1] ? 1 : 0, p.availGait[1][0] ? 1 : 0,
                  p.availGait[1][1] ? 1 : 0, p.sectors[0][0], p.sectors[0][1], p.sectors[1][0], p.sectors[1][1], p.facingFixedLoops,
                  p.rootTurners, p.patternsKept[0], p.patternsKept[1], p.patternsOffBand[0], p.patternsOffBand[1],
                  Fnv1a64(g_matchClasses[3].data(), g_matchClasses[3].size()));
    out = buf;
    for (int s = 0; s < 2; ++s)
        for (int g = 0; g < 2; ++g)
            for (int b = 0; b < strafe::kNumBins; ++b)
            {
                std::snprintf(buf, sizeof buf, "%.3f%s", static_cast<double>(p.table.speed[s][g][b]), (s == 1 && g == 1 && b == 4) ? "" : ",");
                out += buf;
            }
    return out;
}

// ---- 0.6.4
std::string TurnRateStatus()
{
    std::lock_guard lock(g_mutex);
    char st[640];
    std::snprintf(st, sizeof st,
                  "turnRate on=%d min=%.2f max=%.2f lastTarget=%.3f active=%d w=%.3f feeds=%llu turns=%llu last=\"%s\"",
                  g_turnRateOn ? 1 : 0, static_cast<double>(g_turnRateMin), static_cast<double>(g_turnRateMax),
                  static_cast<double>(g_turnRateTarget), g_turnRateActive ? 1 : 0, static_cast<double>(g_turnRateW),
                  static_cast<unsigned long long>(g_turnRateFeeds), static_cast<unsigned long long>(g_turnRateTurns),
                  g_turnRateLastSummary.c_str());
    return st;
}

void SetStartTurn(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_startTurnOn = aOn;
    if (g_dbLoaded && g_matcher.Ready())
    {
        for (uint32_t c = 0; c < g_db.NumClips() && c < g_clipClasses.size(); ++c)
            g_clipClasses[c] = ClipClassOfLocked(c);
        ComputeTurnProfilesLocked(); // slow turning starts reclassified (on) or restored (off)
        g_matcher.SetClipClasses(g_matchClasses[0]);
        RecomputeDerivedClassDataLocked(false); // loco P0 (loop bands just rebuilt; lateral rule / strafe profiles hook)
    }
}

// ---- 0.5.8 kill switches (default on; off = 0.5.7)
bool SetSpeedCap(float aFactor)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aFactor) || aFactor < 0.0f || (aFactor > 0.0f && aFactor < 1.0f) || aFactor > 4.0f)
        return false;
    g_speedCapFactor = aFactor;
    g_overSpeed = false;
    return true;
}

bool SetCostCap(float aCost)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aCost) || aCost < 0.0f || aCost > 100000.0f)
        return false;
    g_costCap = aCost;
    if (g_matcher.Ready())
        g_matcher.SetCostCap(g_costCap, g_costCapHold);
    return true;
}

void SetStanceCap(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_stanceCapOn = aOn;
}

void SetTurnSettle(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_turnSettleOn = aOn;
    g_settleLeft = 0.0f;
}

void SetLoopBands(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_loopBandsOn = aOn;
    g_loopBandTracker.Reset();
    g_loopBandHiTracker.Reset();
    g_lastLoopBand = g_lastLoopBandHi = 0;
}

void SetStartBind(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_startBindOn = aOn;
    ReleaseStartBindLocked();
}

// ---- 0.5.9 ----
void SetCapsuleSync(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_capSyncOn = aOn;
}

// b5 testorder repair
void SetSwapReset(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_swapResetOn = aOn;
}

void SetOffsetCatchup(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_catchupOn = aOn;
    ApplyCatchupLocked();
}

bool SetOffsetCatchupParams(float aMaxDistance, float aCatchupTime, float aSoftDistance, float aCatchupHalflife)
{
    if (!(std::isfinite(aMaxDistance) && aMaxDistance >= 0.05f && aMaxDistance <= 1.0f) ||
        !(std::isfinite(aCatchupTime) && aCatchupTime >= 0.05f && aCatchupTime <= 5.0f) ||
        !(std::isfinite(aSoftDistance) && aSoftDistance >= 0.0f && aSoftDistance <= aMaxDistance) ||
        !(std::isfinite(aCatchupHalflife) && aCatchupHalflife >= 0.01f && aCatchupHalflife <= 2.0f))
        return false;
    std::lock_guard lock(g_mutex);
    g_catchMaxDistance = aMaxDistance;
    g_catchTime = aCatchupTime;
    g_catchSoft = aSoftDistance;
    g_catchHalflife = aCatchupHalflife;
    ApplyCatchupLocked();
    return true;
}

bool SetFootSlipParams(float aStart, float aHalflife)
{
    if (!(std::isfinite(aStart) && aStart >= 0.0f && aStart <= 0.25f) || !(std::isfinite(aHalflife) && aHalflife >= 0.01f && aHalflife <= 1.0f))
        return false;
    std::lock_guard lock(g_mutex);
    g_slipStart = aStart;
    g_slipHalflife = aHalflife;
    ApplyCatchupLocked();
    return true;
}

bool SetFootStartSlip(float aStart)
{
    if (!(std::isfinite(aStart) && aStart >= 0.0f && aStart <= 0.25f))
        return false;
    std::lock_guard lock(g_mutex);
    g_slipStartStart = aStart;
    ApplyCatchupLocked(); // re-seats the base slip (a running start ramp ends)
    return true;
}

float FootStartSlip()
{
    std::lock_guard lock(g_mutex);
    return g_slipStartStart;
}

void SetTrajCal(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_trajCalOn = aOn;
}

float DecelTarget(float aBaseDecel, float aAppliedMul)
{
    std::lock_guard lock(g_mutex);
    ++g_decelQueries;
    g_decelBase = std::isfinite(aBaseDecel) ? aBaseDecel : 0.0f;
    g_decelMul = std::isfinite(aAppliedMul) ? aAppliedMul : 1.0f;
    const int64_t now = Qpc();
    g_decelSampleTicks = GroundBrakingDomain(g_lastFeed) && std::isfinite(aBaseDecel) && aBaseDecel > 0.0f &&
        std::isfinite(aAppliedMul) && aAppliedMul > 0.0f ? now : 0;
    g_decelSampleLoco = g_lastFeed.locoState;
    g_decelSampleDetail = PsmDetailOf(g_lastFeed.psmBlock);
    const float sinceFeed = g_lastFeedTicks ? static_cast<float>(static_cast<double>(now - g_lastFeedTicks) / QpcFreq()) : 1e9f;
    const int32_t loco = g_lastFeed.locoState;
    const float mx = std::isfinite(g_lastFeed.move[0]) ? g_lastFeed.move[0] : 0.0f;
    const float my = std::isfinite(g_lastFeed.move[1]) ? g_lastFeed.move[1] : 0.0f;
    const bool released = mx * mx + my * my <= 0.15f * 0.15f;
    float t = 0.0f;
    // Only the stop (stick released) while MM drives the body; turns / reversals with the stick held keep the vanilla value.
    // The legacy profile was measured for the v1 playback clock. V2 predicts in
    // real seconds and now consumes the native base stat; returning zero removes
    // our old slowdown instead of imposing an unrelated v1 stop on the capsule.
    if (!g_motionHeldMatcher && g_capSyncOn && g_enabled && g_dbLoaded && g_matcher.Ready() && g_gate == 0 && sinceFeed <= kStaleFeedSeconds && loco > 0 &&
        loco < 4 && released)
    {
        // a stop from sprint speed (the state is often already stand) uses the sprint stop profile
        const int cls = (loco != kLocoCrouch && g_rawSpeed > 6.0f && g_capProfile.decel[kLocoSprint] > 0.0f) ? kLocoSprint : loco;
        t = g_capProfile.decel[cls];
        if (t > 0.0f)
            t = std::fmax(t, kMinDecelTarget); // gameplay floor: a run stops within ~0.75 m
    }
    g_decelLastTarget = t;
    if (t > 0.0f && t < g_decelBase)
        ++g_decelActive;
    return t;
}

// ---- 0.6.0 (default on; off = 0.5.9)
void SetMeleeGate(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeCfg.enabled = aOn;
    g_meleeGate.Reset();
    g_meleeActive = false;
}

void SetMountCut(bool aOn)
{
    std::lock_guard lock(g_mutex);
    const bool was = g_mountCutOn;
    g_mountCutOn = aOn;
    g_mountMuted = false;
    g_mountEventLeft = 0.0f;
    // review fix (vehdll MMDriver:3307): a settings re-push with the same value keeps the running transition / capture / ride
    // latch (it restarted the mount transition mid-ride). Switched off: everything ends (no exit hand-in, as before). Switched on
    // inside a vehicle: the ride counts as handled (no mount transition mid-ride; the capture runs, the exit is handed in).
    UTPP_IF_MUT(VEHEXIT, 7)
        VtAbortLocked(); // mutation 7: pre-fix (every push aborts, the next vehicle feed begins again)
    else if (!aOn || !was)
    {
        VtAbortLocked(); // vehicle batch W1 P2
        g_vtRide = aOn && g_prevVeh;
    }
}

void MountEvent(bool aIsCar)
{
    std::lock_guard lock(g_mutex);
    g_vtMountCar = aIsCar; // read by the VtBeginMountLocked of this event (or of the vehicle bit feed right after it)
    g_vtMountCarTicks = Qpc();
    if (!g_mountCutOn)
        return;
    if (g_weight.Weight() > 0.0f)
        ++g_mountEventCuts;
    if (!g_vtRide)
    {
        VtMountDiagEvent(g_lastFeed); // 2026-10-28 mount start diagnostic (log only): this begin came from the MountingEvent
        VtBeginMountLocked(g_lastFeed, Qpc()); // vehicle batch W1 P2: before the PublishLocked(0) below (last visible frame)
    }
    g_weight.Cut();
    g_mountEventLeft = kMountEventHold;
    PublishLocked(0.0f); // the FK hook blends the game pose from now on (before the next locomotion feed)
}

void SetVehicleEntryHoldCap(float aSeconds)
{
    std::lock_guard lock(g_mutex);
    vehtrans::SetHoldCap(aSeconds, Qpc()); // game thread; waits for the next mount cut when none runs yet
}

// ---- 0.6.2
void SetHoldPose(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_holdPoseOn = aOn; // off: the next feed / tick fades a closure out as in 0.6.1
}

// CLIMB RELEASE stage
bool SetClimbRelease(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_climbReleaseOn = aOn;
    // off: the next feed / tick holds 8 / 9 again (as before this stage) - v1 only; TRAV-1: v2 traversal active keeps releasing
    if (!aOn && !(g_motion && g_motion->travActive && g_motion->travActive()))
        g_releaseReasons = 0;
    return true;
}

void SetUpperMask(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_upperMaskOn = aOn; // off: the next feed closes the whole body for the masked states (the mask fade holds, no pop)
    g_maskLogged = false;
}

bool SetUpperMaskStates(int32_t aBits)
{
    std::lock_guard lock(g_mutex);
    if (aBits < 0 || static_cast<uint32_t>(aBits) > kMaskStatesAll)
        return false;
    g_maskStates = static_cast<uint32_t>(aBits);
    return true;
}

void SetMeleeFullBody(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeFullFlag = aOn; // off: the next feed fades the legs back to MM (no pop)
}

void SetMeleeFullBodySetting(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeFullSetting = aOn;
}

bool SetMeleeFullBodyStates(int32_t aMask)
{
    std::lock_guard lock(g_mutex);
    if (!mm::melee::ValidStates(aMask))
        return false;
    g_meleeFullStates = static_cast<uint32_t>(aMask);
    return true;
}

void SetMeleeGuard(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeGuardFlag = aOn; // off: the next feed fades the upper mask back (0.15 s)
}

void SetMeleeGuardSetting(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeGuardSetting = aOn;
}

void SetMeleeGuardFullSetting(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeGuardFullSetting = aOn;
}

void SetMeleeGuardTiltSetting(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_meleeGuardTiltSetting = aOn;
}

void SetMeleeAimLogSource(bool (*aSource)(char*, size_t))
{
    std::lock_guard lock(g_mutex);
    g_meleeAimLogSource = aSource;
}

std::string MeleeFullBodyStatus()
{
    std::lock_guard lock(g_mutex);
    char buf[640];
    std::snprintf(buf, sizeof buf, "meleefull flag=%d setting=%d want=%d why=%s stick=%d ground=%d fastT=%.2f fastOff=%d airT=%.2f full=%.2f progress=%.2f aimW=%.2f switches=%llu states=0x%X mwState=%d guardFlag=%d guardSet=%d guardFull=%d guardTilt=%d guardWant=%d guardWhy=%s guardAirT=%.2f mask=%.2f guardSwitches=%llu",
                  g_meleeFullFlag ? 1 : 0, g_meleeFullSetting ? 1 : 0, g_meleeFullWant ? 1 : 0,
                  mm::melee::WhyName(g_meleeFullGate.LastWhy()), g_meleeFullStick ? 1 : 0, g_meleeFullGround,
                  static_cast<double>(g_meleeFullGate.FastSeconds()), g_meleeFullGate.FastOff() ? 1 : 0,
                  static_cast<double>(g_meleeFullGate.AirSeconds()),
                  static_cast<double>(g_meleeFullBlend.Weight()), static_cast<double>(g_meleeFullBlend.Progress()),
                  static_cast<double>(g_meleeAimBlend.Weight()), static_cast<unsigned long long>(g_meleeFullSwitches),
                  g_meleeFullStates, g_meleeState, g_meleeGuardFlag ? 1 : 0, g_meleeGuardSetting ? 1 : 0,
                  g_meleeGuardFullSetting ? 1 : 0, g_meleeGuardTiltSetting ? 1 : 0, g_meleeGuardWant ? 1 : 0,
                  mm::melee::GuardWhyName(g_meleeGuardGate.LastWhy()), static_cast<double>(g_meleeGuardGate.AirSeconds()),
                  static_cast<double>(g_maskBlend.Weight()), static_cast<unsigned long long>(g_meleeGuardSwitches));
    return buf;
}

void SetAimUpperFade(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_aimUpperFadeOn = aOn;
}

void SetWeaponUpper(bool aOn)
{
    std::lock_guard lock(g_mutex);
    g_weaponUpperOn = aOn;
}

bool SetStanceClipMinSpeed(float aMps)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aMps) || aMps < 0.0f || aMps > 12.0f)
        return false;
    g_stanceClipMin = aMps;
    g_stanceClipOk = true;
    return true;
}

bool SetStanceTransitionWindow(float aSeconds)
{
    std::lock_guard lock(g_mutex);
    if (!std::isfinite(aSeconds) || aSeconds > 5.0f)
        return false;
    g_curCfg.stanceWindow = aSeconds < 0.0f ? -1.0f : aSeconds;
    return true;
}

std::string SpeedCalibrationSummary()
{
    std::lock_guard lock(g_mutex);
    return g_cal.Summary();
}

bool GetSpeedMap(int32_t aLoco, float aGame[3], float aDb[3], int32_t& aCount)
{
    std::lock_guard lock(g_mutex);
    if (aLoco < 0 || aLoco > 3)
        return false;
    const SpeedPoints& m = g_speedMap[aLoco];
    aCount = m.count;
    for (int i = 0; i < 3; ++i)
    {
        aGame[i] = m.game[i];
        aDb[i] = m.db[i];
    }
    return true;
}

std::string DatabaseSwitchStatus()
{
    std::lock_guard lk(g_sw.m);
    return g_sw.status;
}

void WaitDatabaseWorker()
{
    std::thread t;
    {
        std::lock_guard lk(g_sw.m);
        t = std::move(g_sw.worker);
    }
    if (t.joinable())
        t.join();
}

// ---- batch 1b feed dump (public) ----
bool SetFeedDump(bool aOn)
{
    std::lock_guard lock(g_mutex);
    if (!aOn)
    {
        mm::posetrace::g_dump.Stop();
        if (g_feedDumpOn)
        {
            g_feedDump.Stop(); // the writer writes the rest, closes the file and exits
            if (g_log)
            {
                char line[160];
                std::snprintf(line, sizeof line, "UTPP MM feed dump off: lines=%llu dropped=%llu file=",
                              static_cast<unsigned long long>(g_feedDump.Pushed()), static_cast<unsigned long long>(g_feedDump.Dropped()));
                g_log((line + util::RelativeToGame(g_feedDump.Path())).c_str());
            }
        }
        g_feedDumpOn = false;
        return true;
    }
    if (g_feedDumpOn && g_feedDump.Accepting())
        return true;
    g_feedDump.Stop(); // a session whose writer failed: Start joins it
    const bool ok = g_feedDump.Start(g_feedDumpDir);
    std::error_code ec;
    // The text pose CSV (utpp-pose-<time>.csv, 100-190 MB/min) is NOT part of the feed dump or of the player's diagnostic logs (they use
    // the compact pose recording). Only the project's own tools get it: the marker file <feed dump folder>\pose-text-csv.txt must exist
    // when SetFeedDump(true) is called (the harness / probe scripts create it next to the logs).
    if (ok && std::filesystem::exists(std::filesystem::path(g_feedDumpDir) / L"pose-text-csv.txt", ec))
    {
        if (mm::posetrace::g_dump.Start(g_feedDumpDir))
        {
            if (g_log)
                g_log(("UTPP pose text CSV on (marker pose-text-csv.txt): " + util::RelativeToGame(mm::posetrace::g_dump.Path().wstring())).c_str());
        }
        else if (g_log)
            g_log("UTPP pose trace FAILED to start");
    }
    g_feedDumpOn = ok;
    g_feedDumpFailLogged = false;
    g_feedDumpT0 = Qpc();
    if (g_log)
    {
        if (ok)
            g_log(("UTPP MM feed dump on: " + util::RelativeToGame(g_feedDump.Path())).c_str());
        else
            g_log(("UTPP MM feed dump FAILED to start (folder \"" + util::RelativeToGame(g_feedDumpDir) + "\")").c_str());
    }
    return ok;
}

void SetFeedDumpDir(const std::wstring& aDir)
{
    std::lock_guard lock(g_mutex);
    g_feedDumpDir = aDir;
    if (!g_feedDumpDir.empty() && g_feedDumpDir.back() != L'\\' && g_feedDumpDir.back() != L'/')
        g_feedDumpDir += L'\\';
}

std::string FeedDumpStatus()
{
    std::lock_guard lock(g_mutex);
    return g_feedDump.Status() + " " + mm::posetrace::g_dump.Status();
}

#ifndef UTPP_PLUGIN_DLL // test hook: compiled out of UTPP.dll (CMakeLists.txt)
bool FeedDumpFlushForTest(int aTimeoutMs)
{
    for (int waited = 0;; waited += 2)
    {
        if (g_feedDump.Written() >= g_feedDump.Pushed())
            return true;
        if (waited >= aTimeoutMs)
            return false;
        g_feedDump.TestWake();
        Sleep(2);
    }
}
#endif

void Shutdown()
{
    WaitDatabaseWorker();
    std::lock_guard lock(g_mutex);
    if (g_feedDumpOn && g_log)
    {
        char line[160];
        std::snprintf(line, sizeof line, "UTPP MM feed dump off (shutdown): lines=%llu dropped=%llu file=",
                      static_cast<unsigned long long>(g_feedDump.Pushed()), static_cast<unsigned long long>(g_feedDump.Dropped()));
        g_log((line + util::RelativeToGame(g_feedDump.Path())).c_str());
    }
    g_feedDump.Shutdown(); // batch 1b: stop + join (the writer writes the rest and closes the file)
    mm::posetrace::g_dump.Stop();
    g_feedDumpOn = false;
    g_enabled = false;
    g_accelSampleTicks = 0;
    g_weight.Reset();
    g_hot.mmActive.store(false, std::memory_order_release);
}
} // namespace utpp::motionhost

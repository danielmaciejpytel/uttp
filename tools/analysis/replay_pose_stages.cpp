// Replays a recorded game feed dump through the real motion driver and leg tail and writes the local pose of every database bone at two
// stages per feed: the driver output handed to the leg tail ("driver": sampled clips, blends, root frame not yet applied) and the published
// snapshot ("final": root offset applied, stance guard, foot locker, head lead, hand-in). No game or input access.
//
//   utpp_replay_pose_stages <data dir> <base database> <feed dump> <out prefix> [--from T] [--to T] [--settings-log LOG] [--ahead recorded]
//                           [--body man]
//
// <out prefix>.txt  header: bone names and parents, the record layout, the clip names seen
// <out prefix>.bin  per feed one record of float32: kMetaFloats meta values (see kMeta below), then driver pose, then final pose
//                   (numBones x [px py pz qx qy qz qw] each, database local space)
// --from / --to limit the written records to feed times in [T0, T1]; every feed is still stepped.
// --settings-log applies the setting calls of a game log of the session (the ones this tool knows, in their order) before the first feed.
// --ahead recorded passes the recorded turn of the coming tick (next feed's yaw minus this one) as the drawn capsule's turn.
// --body man replays with the male motion database and rig (default: woman); the clip dump (--clips) is always the woman database.
// --history 1 turns the live pose history on (mm/LivePoseHistory.hpp; default off as in the game): the published pose after the leg
// tail is fed back to the next search through the same host hook the game uses.
// --turn-foot-phase 0|1 switches the turn foot-phase re-rank (MatcherConfig::turnFootPhase; default off as in the game); a TURNFOOT line
// reports the reversal searches that received the shown pose.
// Off-by-default options of the runtime path (each absent = the build default, bit for bit):
//   --steering h,t      blend stack steering half-life / animation lookahead (s), e.g. 0.4,2.0
//   --signals N         per-bone root-motion signals: bit 0 root on the Root weight, bit 1 contacts on the leg weights
//   --pelvis 1          pelvis stage of the foot locker
//   --capsule-query 1   chooser query trajectory in capsule units
//   --velocity-predicates 1  moving / pivot predicates from the current velocity and the input acceleration
//   --tip-deg D         turn-in-place angle of the chooser (deg)
//   --pick-dump 1       one PICKDUMP line per stop / start pick (inspection)
//   --capsule-stats 1   after every feed, report the capsule Acceleration / Deceleration stats and keep the script's stat multipliers
//                       as the game does after each locomotion feed (AccelTarget / DecelTarget, see CapsuleStatReplay below); the
//                       stats are the per-state values of the recorded sessions (default off: no stat reported, motion fallback)
//   --air-feed 1        hand each feed's air fields (onGround, distToGround, psmFall of the dump) to the driver right before the feed,
//                       as the game's feed entry does (motiondriver::FeedAirFromInput); default off: the driver never receives an air feed
//   --qpc-feed 1        every clock the host reads during a feed (feed and publish stamps, capsule stat ages, command dt) is the recorded
//                       feed time instead of the wall clock (motionhost::TestSetQpcTicks); default off: wall clock
//   --neutral-aim 1     neutral aim offset on (motiondriver::SetNeutralAimOffset, the mm.neutralAim setting), set after --settings-log
//   --aim-world-smooth H  world aim target on, smoothed at H Hz (motiondriver::SetAimWorldSmoothing, mm.aimWorldSmooth; the script sends
//                       0 = no smoothing), after --settings-log
//   --shooter-mode 1    shooter facing on after --settings-log (exposure probe: the recorded sessions ran with it off, and the neutral
//                       aim offset only acts in a strafe / shooter facing)
//                       Any of these three adds one AIMOPT line: feeds with a neutral aim offset weight / a world aim smoothing delta.
//   --aim-camera P      feed the camera and aim targets the script sends (the dumps do not record them): every feed a level point 60 m
//                       along the recorded camera yaw, the aim target while the PSM upper body is Aim; overlay inputs pose P with a
//                       drawn weapon (0 Default, 1 Rifle, 2 Pistol1H, 3 Pistol2H; the weapon is not recorded), Aiming / Ready / Relaxed
//                       from the PSM; mm.aimGameUpper false (the script default). No camera pitch in the dumps: level only. One AIMCAM line.
//   --aim-camera-alt Q  with --aim-camera P: while the weapon is drawn the overlay pose alternates P / Q every 90 drawn feeds (feed
//                       count over the whole run: P for drawn feeds 0..89, Q for 90..179, ...), so the replay contains weapon class changes
//                       The settings log calls MMNeutralAimOffset and MMAimWorldSmoothing are applied as well (later sessions).
//   --stride-warp 0|1   stride warping of the blend stack off / on after --settings-log (motiondriver::SetStrideWarp, the mm.strideWarp
//                       setting; the settings log call MMStrideWarp of later sessions is applied as well); not passed: the log / build default
//   --overlay-inertial-switch 0|1  inertialized overlay switch off / on after --settings-log (motiondriver::SetOverlayInertialSwitch,
//                       the mm.overlayInertialSwitch setting; the settings log call MMOverlayInertialSwitch is applied as well); it acts on
//                       overlay changes only, so a replay without --aim-camera (overlay always Default) is unchanged by it
//   --step-down-reach 0|1 short step-downs kept on the ground off / on (the mm.stepDownReach setting: chooser mapping and air
//                       state, motiondriver::SetStepDownReach + motionhost::SetAirStepDownReach)
//   --swivel-pole 0|1|2 knee swivel pole of the stance guard: capsule forward / pelvis forward (build default) / foot forward
//                       (motionhost::SetStancePoleMode)
//   --stick-speed 0|1   stick speed model of the driver analog / fixed after --settings-log (motiondriver::SetStickSpeedMode, the
//                       mm.stickSpeed setting; the settings log call MotionStickSpeedMode of later sessions is applied as well)
//   --firm-feet 0|1     firm feet in turns and stops off / on after --settings-log (motionhost::SetFirmFeet, the mm.firmFeet setting;
//                       the settings log call MMFirmFeet is applied as well); adds one FIRMFEET line (latches, held updates, of them
//                       lowered by a leg limit, hand-backs)
//   --firm-limit L      hold limit (m), --firm-handback V hand-back speed (m/s), --firm-knee D knee fold margin (deg),
//                       --firm-release V release speed (m/s) and --firm-latch-speed V latch capsule speed (m/s) of the firm feet
//                       (motionhost::SetFirmFeetParams; not passed = build default)
//                       With --settings-log, the driver first gets the values a game session held before its first setting call in
//                       builds whose defaults were off (lean, head lean, slide, orientation warp, stride warping off, analog stick),
//                       then the logged calls: an older log without one of these calls replays as recorded with any build default.
//                       The same baseline holds the earlier DLL values of the settings whose calls the game log does not carry in a
//                       form this parser applies (aim game upper body on, world aim smoothing off, weapon upper body on, Pistol2H
//                       shot transfer off, magazine top-up off, air steering off, melee guard off, inertialized overlay switch off):
//                       with or without a log entry the
//                       replay keeps the state of the earlier replays for them, whatever the build default.
//                       The settings log calls MMSteerMode / MMSteerActiveOnly of test builds are ignored (turn steering has no
//                       option: the newest layer steered, carried through the layer mix).
//   --body-height-smooth 0|1  body height smoothing on uneven ground (motionhost::SetBodyHeightSmooth, the mm.bodyHeightSmooth setting;
//                       the settings log call MMBodyHeightSmooth is applied as well). The game runs the stage where the pose is drawn,
//                       with the drawn world height; here it runs after each feed on the published pose, with the drawn height of
//                       --draw-z next (default: the next feed's capsule z, the capsule after the tick's movement) or same (this feed's
//                       capsule z) and the gate the host published. One BODYHEIGHT line. Off / not passed: output unchanged.
//   --turn-rate 0|1     turn clip rate of the legacy matcher off / on after --settings-log (motionhost::SetTurnRate, the bodyturn.turnRate
//                       setting); not passed: the build default (on)
//   --turn-rate-max M   turn clip rate cap after --settings-log (motionhost::SetTurnRateParams(0.9, M), bodyturn.turnRateMax; the
//                       settings log call MMTurnRateParams is applied as well); either option adds one TURNRATE line (the host
//                       turn rate status: raised feeds, turns, cap)
//   --aim-game-upper 0|1  mm.aimGameUpper off / on (motiondriver::SetAimGameUpper), set after --settings-log and after the
//                       --aim-camera script default (false); not passed: unchanged. On, the published pose keeps the motion upper body
//                       without the aim offset (the game's upper body is not recorded): barrel measurements of it are not the shown pose.
//   --capsule-forecast 0|1  capsule forecast for turning and partial-stick starts (motiondriver::SetCapsuleForecast, the
//                       mm.capsuleForecast setting; the settings log call MMCapsuleForecast is applied as well). Off / not passed: unchanged.
//   --forecast-dump 1   also write <out prefix>.forecast.txt: per feed the feed time, the query trajectory's current sample position and
//                       its predicted displacement 0.35 / 0.7 s ahead (adapter cm) and the applied capsule acceleration of the forecast
//                       (inspection only; the other outputs are unchanged)
//   --crouch-stop LIST  crouch stop options of the chooser switched on (motion::ApplyCrouchStopOptions names, comma separated; set as
//                       UTPP_TEST_CROUCH_STOP for the matcher's Init). Not passed: the build configuration.
//   --test-env N=V      set one UTPP_TEST_* switch of the matcher (e.g. UTPP_TEST_GUARD_OFF=crouchrate,stopgate) for this process only;
//                       repeatable. Lets one side of a before / after comparison differ from the other.
//   --lean-pack 0|1     1: the driver loads the motion lean pack <data>\utpp_lean_<body>.bin for both bodies, as the game does from the
//                       mod data folder, so the lean body / head layers run and the head lead gives way to the lean head layer when the
//                       settings ask for both (mm.lean + mm.leanHead, the menu defaults). Not passed / 0: no lean pack (earlier replays:
//                       lean unavailable, the head lead runs instead). Either value adds one LEANPACK line (requested, loaded).
//   --lean-scale B,H    body / head lean amounts after --settings-log (motiondriver::SetLeanParams, the mm.leanScale / mm.leanHeadScale
//                       settings); not passed: the log / build default
//   --lean-coverage C   lean coverage 0 standard / 1 wide after --settings-log (motiondriver::SetLeanCoverage, mm.leanCoverage)
//   --force-drawn 1     every feed reports a drawn weapon (PSM weapon known + drawn bits; probe of weapon-in-hand locomotion on
//                       sessions recorded without one; only the motion side sees the flag, the game's weapon layer is not replayed)
//   --aim-jog-query 1   after every feed ask the host for the aim jog target with the feed's stick and hand the answer to the driver, as
//                       the locomotion script does right after each feed (utpp.reds UTPPUpdateAimSpeed: UTPPMMAimJogTarget, which also
//                       sets motiondriver::SetAimJogOff); without it the driver keeps the vanilla aim walk mapping while aiming. Adds
//                       one AIMJOG line (queries, feeds with a jog target). Off / not passed: unchanged.
//   --motion-log 1      also write <out prefix>.motion.log: the driver's MOTION trace lines (the game log format, 2 Hz plus every pick
//                       change), each stamped with the feed time as a wall time of 2000-01-01 (00:00:00 = feed time 0); the world
//                       slide tools rebuild the contact track from it exactly as they do from a game log (no effect on the pose)
// Recommended gate settings (both sides the same): --capsule-stats 1 --air-feed 1 --motion-log 1 (see tools/replay/gate_eval.py);
// --qpc-feed 1 leaves the outputs of the recorded sessions unchanged and is a guard only (it covers the host clocks; driver-side
// clocks such as the recoil clock still read the real counter). --air-feed 1 adds one AIRFEED line to the summary.
// <feed dump> = --clips [name part] writes the stored frames of the woman database clips whose name contains the part (all without it)
// instead (same layout, see the clip mode below).
// <out prefix> = - writes no files (summary only).
//
// Offline measurement split (one command: tools/replay/compare_runs.py runs both tools over the same game dumps):
//   - utpp_motion_replay = the matcher / evaluator / foot locker lab (fast, deterministic, the slide tables FLTAB / FLROB / FLPEL);
//   - this tool = the real runtime path (MotionDriver with the blend stack and root frame, MotionHost leg tail with the foot locker).
// Both read the per-frame game dumps directly. The runtime path is not ported into utpp_motion_replay: it needs the whole driver and
// host (this tool links them through the integration test harness), and the lab tables must stay comparable with earlier runs.
//
// Every run ends with one line
//   RUNTIME run=<name> secs clipChangesPerS ikCorrMaxCm ikCorrP95Cm turns procYawPerTurnDeg procYawShare feedUsP50 feedUsP95
// clipChangesPerS: changes of the driver pick clip (Status pick=clip:) per second of feed time.
// ikCorr*: foot locker correction of either foot (host Status corrL= / corrR=, cm), max and p95 over the frames with a correction.
// Procedural yaw of a step: the visual root turn not carried by the sampled animation, capsule yaw delta + (offset yaw after - offset yaw
// before) - animation root yaw delta (root frame probe, deg). A turn: a run of consecutive feeds whose recorded body yaw changes faster
// than 30 deg/s, at least 0.1 s long and 20 deg in total. procYawPerTurnDeg = mean over the turns of the summed |procedural yaw| inside
// the turn; procYawShare = summed |procedural yaw| / summed |capsule yaw| over all steps. feedUs: wall time of MotionHost Feed (this
// machine, single run; a cost comparison needs repeats). historySearches: searches that read the live pose history (--history 1).
// A second line PICKS covers the picks into stop / start clips: the first-frame foot offset (the picked clip frame's LeftFoot /
// RightFoot against the shown feet of the previous feed, Root space, horizontal, max of both feet; wrong foot = above 10 cm) and the
// planted toe drift in the world during the first 0.3 s after the pick (toes of the published pose put into the world with the next
// feed's capsule and yaw, max distance from where each foot planted while the tail contact >= 0.5). PlantedN / PlantedKept /
// PlantedOffP50Cm: the same offset for the foot(s) planted in the shown pose (tail contact >= 0.5 on the previous feed) only, and how
// many picks keep that foot within 10 cm.
#include <chrono>
#include <filesystem>
#define main FullIntegrationSuiteNotRun
#include "../../plugin/UTPP/tests/MotionDriverIntegrationTest.cpp"
#undef main
#include "../../plugin/UTPP/src/mm/ExitPoseHistory.hpp"
#include "../../plugin/UTPP/src/mm/MotionBlendStack.hpp"
#include "../../plugin/UTPP/src/mm/BodyHeightRuntime.hpp"

namespace utpp::motiondriver
{
void TestSetMapOverrides(int32_t aCapsuleSpeedQuery, int32_t aVelocityPredicates, float aTurnInPlaceDeg);
void TestSetMapStepDownReach(int32_t aOn);
void SetStepDownReach(bool aOn);
void TestAimOptionFeeds(uint64_t& aNeutralAo, uint64_t& aAimSmooth);
bool SetNeutralAimOffset(bool aOn);
bool SetAimWorldSmoothing(bool aOn, float aHz);
bool SetLeanCoverage(int32_t aCoverage);
float TestAoGridShare();
void TestAoDiag(float aOut[15]);
}
namespace utpp::motionhost
{
void TestSetPelvisPlacement(bool aOn);
void SetBodyHeightSmooth(bool aOn);
void SetAirStepDownReach(bool aOn);
}

namespace
{
constexpr const char* kMeta[] = {"t", "dt", "owned", "published", "tailed", "clip", "clipTime", "db", "contactL", "contactR",
                                 "rootVelX", "rootVelY", "rootYawRate", "providedRoot", "rootOffX", "rootOffY", "rootOffYaw",
                                 "offX_cm", "offY_cm", "offYaw_deg", "offClamps", "offResets", "offTeleports",
                                 "lockL", "wL", "corrL_cm", "lockR", "wR", "corrR_cm", "playRate", "capX", "capY", "yawDeg",
                                 "mvx", "mvy", "loco", "rpPrepared", "rpCapDX", "rpCapDY", "rpCapDYaw", "rpAnimDX", "rpAnimDY",
                                 "rpAnimDYaw", "rpMovingGround", "rpMontage", "rpBeforeX", "rpBeforeY", "rpBeforeYaw", "rpAfterX", "rpAfterY",
                                 "rpAfterYaw", "rpYawAhead", "aoGrid", "aoW", "aoGridShare", "aoYaw", "aoGridYaw", "aoPitch",
                                 "aoSpineYaw", "aoHcIn", "aoHcGrid", "aoHcOut", "nYaw", "nPitch", "nW", "aimW", "haveAim", "aoSrcYaw"};
constexpr size_t kMetaFloats = sizeof(kMeta) / sizeof(kMeta[0]);

motionhost::MotionSetFrame g_tail;
std::vector<float> g_driverPose;
bool g_tailValid = false;

// Lean pack state in the driver status (" lean=<setting>/<pack loaded>/..."): 1 / 0, -1 = no bundle installed.
int LeanPackLoaded(const std::string& aStatus)
{
    const size_t p = aStatus.find(" lean=");
    if (p == std::string::npos)
        return -1;
    const size_t s = aStatus.find('/', p);
    return s == std::string::npos ? -1 : std::atoi(aStatus.c_str() + s + 1);
}

bool CaptureTail(const int32_t* aMap, uint32_t aN, motionhost::MotionSetFrame& aFrame)
{
    const bool ok = motiondriver::Frame(aMap, aN, aFrame);
    if (ok)
    {
        g_tailValid = true;
        g_tail = aFrame;
        g_driverPose.assign(size_t(aFrame.numBones) * 7, 0.0f);
        for (uint32_t k = 0; k < aFrame.numBones; ++k)
        {
            const auto& v = aFrame.pose[k];
            float* p = &g_driverPose[size_t(k) * 7];
            p[0] = v.p.x, p[1] = v.p.y, p[2] = v.p.z, p[3] = v.q.x, p[4] = v.q.y, p[5] = v.q.z, p[6] = v.q.w;
        }
    }
    return ok;
}

// 0 = stop clip, 1 = start clip, -1 = other (clip name convention of the databases)
int StopStartClass(const std::string& aName)
{
    const auto has = [&](const char* k) { return aName.find(k) != std::string::npos; };
    if (has("Transition_") || has("Pivot") || has("Reface"))
        return -1;
    if (has("_Stop") || has("Stop_"))
        return 0;
    if (has("_Start") || has("Start_"))
        return 1;
    return -1;
}

// Positions of two bones by forward kinematics from aFirstBone (0 = with the root offset, 2 = Root space); false if a bone is missing.
template <class LocalAt>
bool FeetAt(const mm::motion::MotionDatabase& aDb, const int32_t (&aBones)[2], uint32_t aFirstBone, LocalAt&& aLocalAt, math::Vec3 (&aOut)[2])
{
    if (aBones[0] < 0 || aBones[1] < 0 || aDb.NumBones() > mm::motion::kExitSeedMaxDbBones)
        return false;
    math::Vec3 gp[mm::motion::kExitSeedMaxDbBones];
    math::Quat gq[mm::motion::kExitSeedMaxDbBones];
    bool ok[mm::motion::kExitSeedMaxDbBones];
    mm::motion::RootSpaceFk(aDb, aFirstBone, aLocalAt, gp, gq, ok);
    for (int s = 0; s < 2; ++s)
    {
        if (!ok[aBones[s]])
            return false;
        aOut[s] = gp[aBones[s]];
    }
    return true;
}

float Num(const std::string& aStatus, const char* aKey)
{
    const std::string v = StatusField(aStatus, aKey);
    return v.empty() ? std::nanf("") : std::strtof(v.c_str(), nullptr);
}

// The setting calls of a game log ("UTPPSet<Name>(<args>) -> 1"), applied in their order through the C++ API; unknown names are counted.
void ApplyLoggedSettings(const std::string& aLog, int& aApplied, int& aUnknown)
{
    std::ifstream in(aLog);
    std::string line;
    while (std::getline(in, line))
    {
        const size_t at = line.find("UTPPSet");
        const size_t open = at == std::string::npos ? at : line.find('(', at);
        const size_t close = open == std::string::npos ? open : line.find(')', open);
        if (close == std::string::npos || line.find("-> 1", close) == std::string::npos)
            continue;
        const std::string name = line.substr(at + 7, open - at - 7);
        std::vector<float> a;
        std::vector<std::string> toks;
        std::istringstream args(line.substr(open + 1, close - open - 1));
        std::string tok;
        while (std::getline(args, tok, ','))
        {
            a.push_back(std::strtof(tok.c_str(), nullptr));
            toks.push_back(tok);
        }
        const auto arg = [&](size_t i) { return i < a.size() ? a[i] : 0.0f; };
        // a Bool argument as the script logs it ("true" / "false") or as a number
        const auto flag = [&](size_t i) { return i < toks.size() && (toks[i].find("true") != std::string::npos || a[i] != 0.0f); };
        bool known = true;
        if (name == "MotionOrientWarp") motiondriver::SetOrientWarp(arg(0) != 0);
        else if (name == "MotionPelvisYawTrim") motiondriver::SetPelvisYawTrim(arg(0));
        else if (name == "MotionBodyYawOffset") motiondriver::SetBodyYawOffset(arg(0));
        else if (name == "MotionBodyYawOffsets") motiondriver::SetBodyYawOffsets(arg(0), arg(1));
        else if (name == "MMStrideWarp") motiondriver::SetStrideWarp(flag(0));
        else if (name == "MMOverlayInertialSwitch") motiondriver::SetOverlayInertialSwitch(flag(0));
        else if (name == "MMSteerMode" || name == "MMSteerActiveOnly") {} // turn steering options of test builds: no option now
        else if (name == "MotionLean") motiondriver::SetLean(arg(0) != 0);
        else if (name == "MotionLeanHead") motiondriver::SetLeanHead(arg(0) != 0);
        else if (name == "MotionLeanParams") motiondriver::SetLeanParams(arg(0), arg(1));
        else if (name == "MMHeadLead") motiondriver::SetHeadLead(arg(0) != 0);
        else if (name == "MotionWalkSpeed") motiondriver::SetWalkSpeed(arg(0));
        else if (name == "MotionMinAnalogWalk") motiondriver::SetMinAnalogWalk(arg(0));
        else if (name == "MotionSlide") motiondriver::SetSlide(arg(0) != 0);
        else if (name == "MotionStickSpeedMode") motiondriver::SetStickSpeedMode(static_cast<int>(arg(0)));
        else if (name == "MotionBarrelLoop") motiondriver::SetBarrelLoop(arg(0) != 0);
        else if (name == "MotionShooterMode") motiondriver::SetShooterMode(arg(0) != 0);
        else if (name == "MMTurnRateParams") motionhost::SetTurnRateParams(arg(0), arg(1));
        else if (name == "MMStrafeParams") motionhost::SetStrafeParams(arg(0) != 0, arg(1), arg(2), arg(3), arg(4));
        else if (name == "MMAir") motionhost::SetAir(static_cast<int32_t>(arg(0)));
        else if (name == "MMFootReplant") motionhost::SetFootReplant(static_cast<int32_t>(arg(0)));
        else if (name == "MMKneeFix") motionhost::SetKneeFix(static_cast<int32_t>(arg(0)));
        else if (name == "MMClimbRelease") motionhost::SetClimbRelease(arg(0) != 0);
        else if (name == "MMFirmFeet") motionhost::SetFirmFeet(flag(0));
        else if (name == "MMNeutralAimOffset") motiondriver::SetNeutralAimOffset(flag(0));
        else if (name == "MMAimWorldSmoothing") motiondriver::SetAimWorldSmoothing(flag(0), arg(1));
        else if (name == "MMBodyHeightSmooth") motionhost::SetBodyHeightSmooth(flag(0));
        else if (name == "MMCapsuleForecast") motiondriver::SetCapsuleForecast(flag(0));
        else known = false;
        (known ? aApplied : aUnknown) += 1;
    }
}
// The driver state of a recorded session before its first logged setting call: the settings whose game logs only carry a call when
// the value differed from the earlier DLL defaults (off / analog). Applied before the log so the replay does not depend on the build's
// own defaults.
void ApplyLogBaseline()
{
    motiondriver::SetLean(false);
    motiondriver::SetLeanHead(false);
    motiondriver::SetSlide(false);
    motiondriver::SetOrientWarp(false);
    motiondriver::SetStrideWarp(false);
    motiondriver::SetStickSpeedMode(0);
    // earlier DLL values of settings the parser does not read from a game log (Bool lines without "-> 1", or no log line at all)
    motiondriver::SetAimGameUpper(true);
    motiondriver::SetAimWorldSmoothing(false, 0.0f);
    motiondriver::SetWeaponUpper(true);
    motionhost::SetWeaponUpper(true);
    motiondriver::SetShootTransferEnabled(false);
    motiondriver::SetMagTapEnabled(false);
    motionhost::SetAirSteer(false);
    motionhost::SetMeleeGuardSetting(false);
    motionhost::SetFirmFeet(false);
    motiondriver::SetOverlayInertialSwitch(false);
}
// --motion-log: driver trace lines -> file, stamped with the feed time of the feed being stepped
FILE* g_motionLog = nullptr;
double g_motionLogT = 0.0;
void MotionLogLine(const char* aLine)
{
    if (!g_motionLog || !aLine)
        return;
    const double t = g_motionLogT < 0.0 ? 0.0 : g_motionLogT;
    const long long whole = static_cast<long long>(t);
    const long long micros = static_cast<long long>((t - double(whole)) * 1e6 + 0.5);
    std::fprintf(g_motionLog, "[2000-01-01 %02lld:%02lld:%02lld.%06lld] %s\n", (whole / 3600) % 24, (whole / 60) % 60, whole % 60,
                 micros > 999999 ? 999999 : micros, aLine);
}
} // namespace

// The locomotion script queries the capsule stats right after every feed (UTPPUpdateAccel, then UTPPUpdateDecel): it reports the
// stat without its own multiplier and the multiplier it applies, then keeps, replaces or removes that multiplier from the returned
// target. The stats come from the session's log when it has the stat samples (CAPSTAT lines, logged while the feed dump is on: the
// latest sample before the next feed row); a log without them falls back to the values that were constant per locomotion state in the
// recorded sessions (trace lines: Acceleration 24 in every state, Deceleration 24 while moving and 48 standing still).
struct CapsuleStatReplay
{
    float accelBase = 24.0f, decelMoving = 24.0f, decelIdle = 48.0f;
    float accelMul = 1.0f, decelMul = 1.0f;
    bool accelMod = false, decelMod = false;
    uint64_t accelQueries = 0, decelQueries = 0, accelKept = 0, decelKept = 0, logged = 0, segments = 0;
    struct Sample
    {
        double t;
        float base;
    };
    std::vector<Sample> samples[2]; // accel, decel; sorted by dump time
    size_t next[2] = {0, 0};
    float current[2] = {-1.0f, -1.0f};

    void ReadLog(const std::string& aPath)
    {
        // every feed dump restarts its time: a sample earlier than the one before it starts the samples of a new dump; only the
        // last dump's samples are kept (the replayed feed file is the newest dump of the session)
        std::ifstream in(aPath);
        std::string line;
        double last = -1.0;
        while (std::getline(in, line))
        {
            const size_t at = line.find("UTPP CAPSTAT t=");
            if (at == std::string::npos)
                continue;
            double t = 0;
            char stat[16] = {};
            float base = 0, mul = 0;
            if (std::sscanf(line.c_str() + at, "UTPP CAPSTAT t=%lf stat=%15s base=%f mul=%f", &t, stat, &base, &mul) != 4)
                continue;
            const int k = std::strcmp(stat, "accel") == 0 ? 0 : (std::strcmp(stat, "decel") == 0 ? 1 : -1);
            if (k < 0)
                continue;
            if (segments == 0 || t < last)
            {
                ++segments;
                samples[0].clear();
                samples[1].clear();
                logged = 0;
            }
            last = t;
            samples[k].push_back({t, base});
            ++logged;
        }
        for (auto& v : samples)
            std::stable_sort(v.begin(), v.end(), [](const Sample& a, const Sample& b) { return a.t < b.t; });
    }
    // logged stat k for the query after a feed whose next row starts at aNextT; aFallback without samples
    float Base(int k, double aNextT, float aFallback)
    {
        if (samples[k].empty())
            return aFallback;
        while (next[k] < samples[k].size() && samples[k][next[k]].t < aNextT)
            current[k] = samples[k][next[k]++].base;
        return current[k] >= 0.0f ? current[k] : aFallback;
    }

    static void Keep(float aTarget, float aBase, float aMin, float& aMul, bool& aMod, uint64_t& aKept)
    {
        const float applied = aMod && aMul > 0.0001f ? aMul : 1.0f;
        float want = 1.0f;
        if (aTarget > 0.0f && aBase > aTarget)
            want = std::clamp(aTarget / aBase, aMin, 1.0f);
        if (want >= 0.999f)
        {
            aMod = false;
            aMul = 1.0f;
            return;
        }
        if (aMod && std::fabs(want - applied) < 0.02f)
            return;
        aMod = true;
        aMul = want;
        ++aKept;
    }
    void AfterFeed(int32_t aLoco, double aNextT)
    {
        const float accelApplied = accelMod && accelMul > 0.0001f ? accelMul : 1.0f;
        const float aBase = Base(0, aNextT, accelBase);
        Keep(motionhost::AccelTarget(aBase, accelApplied), aBase, 0.05f, accelMul, accelMod, accelKept);
        ++accelQueries;
        const float decelBase = Base(1, aNextT, aLoco >= 1 && aLoco <= 3 ? decelMoving : decelIdle);
        const float decelApplied = decelMod && decelMul > 0.0001f ? decelMul : 1.0f;
        Keep(motionhost::DecelTarget(decelBase, decelApplied), decelBase, 0.1f, decelMul, decelMod, decelKept);
        ++decelQueries;
    }
};

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::fprintf(stderr, "usage: utpp_replay_pose_stages <data dir> <base database> <feed dump> <out prefix> [--from T] [--to T]\n");
        return 2;
    }
    const std::string data = argv[1], feedPath = argv[3], out = argv[4];
    float from = -1e30f, to = 1e30f;
    std::string settingsLog;
    bool aheadRecorded = false, man = false, liveHistory = false, pelvis = false, pickDump = false;
    int turnFootPhase = -1; // -1 = build default
    int32_t capsuleQuery = -1, velocityPredicates = -1;
    bool capsuleStats = false, airFeed = false, qpcFeed = false, motionLog = false;
    int32_t neutralAim = -1, shooterMode = -1, strideWarp = -1, stickSpeed = -1; // -1 = not passed (the settings log / build default)
    int32_t firmFeet = -1;                     // -1 = not passed
    uint64_t firmTotal[4] = {}, firmLast[4] = {}; // --firm-feet: FIRMFEET counters (summed over the foot locker resets)
    // < 0 = not passed (build defaults)
    float firmLimit = -1.0f, firmHandBack = -1.0f, firmKnee = -1.0f, firmRelease = -1.0f, firmLatch = -1.0f;
    int32_t turnRate = -1;                     // -1 = not passed
    float turnRateMax = -1.0f;                 // < 0 = not passed
    int32_t swivelPole = -1;                   // -1 = not passed
    int32_t stepDownReach = -1;                // -1 = not passed
    int32_t overlayInertialSwitch = -1;                                          // -1 = not passed
    float aimWorldSmoothHz = -1.0f;            // < 0 = not passed
    int32_t aimCameraPose = -1;                // --aim-camera: overlay pose of a drawn weapon, < 0 = not passed (no aim target fed)
    int32_t bodyHeight = -1;                   // --body-height-smooth: < 0 = not passed (the settings log / build default)
    bool drawZNext = true;                     // --draw-z next (default) | same
    int32_t aimCameraAlt = -1;                 // --aim-camera-alt: second overlay pose of a drawn weapon, < 0 = not passed
    uint64_t aimCameraDrawnFeeds = 0;
    int32_t aimGameUpper = -1;                 // --aim-game-upper: < 0 = not passed
    int32_t capsuleForecast = -1;              // --capsule-forecast: < 0 = not passed (the settings log / build default)
    bool forecastDump = false;                 // --forecast-dump 1
    int32_t leanPack = -1;                     // --lean-pack: < 0 = not passed (no lean pack)
    float leanScaleBody = -1.0f, leanScaleHead = -1.0f; // --lean-scale B,H: < 0 = not passed
    int32_t leanCoverage = -1;                 // --lean-coverage: < 0 = not passed
    bool forceDrawn = false;                   // --force-drawn 1: every feed reports a drawn weapon
    int leanLoaded = -1;                       // with --lean-pack: lean pack state of the bundles seen (LEANPACK line)
    bool aimJogQuery = false;                  // --aim-jog-query 1
    uint64_t aimJogQueries = 0, aimJogOnFeeds = 0;
    uint64_t aimCameraFeeds = 0, aimCameraAimFeeds = 0;
    uint64_t airFeeds = 0, airFeedRows = 0; // --air-feed: feeds handed to the driver / of them with air fields in the dump
    float tipDeg = std::numeric_limits<float>::quiet_NaN();
    auto& stackOverrides = mm::motionstack::g_testStackOverrides;
    for (int i = 5; i + 1 < argc; i += 2)
    {
        if (std::strcmp(argv[i], "--from") == 0)
            from = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--to") == 0)
            to = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--settings-log") == 0)
            settingsLog = argv[i + 1];
        else if (std::strcmp(argv[i], "--ahead") == 0)
            aheadRecorded = std::strcmp(argv[i + 1], "recorded") == 0;
        else if (std::strcmp(argv[i], "--body") == 0)
            man = std::strcmp(argv[i + 1], "man") == 0;
        else if (std::strcmp(argv[i], "--history") == 0)
            liveHistory = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--turn-foot-phase") == 0)
            turnFootPhase = std::atoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--steering") == 0)
        {
            stackOverrides.steering = std::sscanf(argv[i + 1], "%f,%f", &stackOverrides.halfLife, &stackOverrides.targetTime) == 2;
            if (!stackOverrides.steering)
                return 2;
        }
        else if (std::strcmp(argv[i], "--signals") == 0)
            stackOverrides.signalMode = std::atoi(argv[i + 1]) & 3;
        else if (std::strcmp(argv[i], "--pelvis") == 0)
            pelvis = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--capsule-query") == 0)
            capsuleQuery = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--velocity-predicates") == 0)
            velocityPredicates = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--tip-deg") == 0)
            tipDeg = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--pick-dump") == 0)
            pickDump = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--capsule-stats") == 0)
            capsuleStats = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--air-feed") == 0)
            airFeed = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--qpc-feed") == 0)
            qpcFeed = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--motion-log") == 0)
            motionLog = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--neutral-aim") == 0)
            neutralAim = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--aim-world-smooth") == 0)
            aimWorldSmoothHz = std::max(0.0f, std::stof(argv[i + 1]));
        else if (std::strcmp(argv[i], "--shooter-mode") == 0)
            shooterMode = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--stride-warp") == 0)
            strideWarp = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--overlay-inertial-switch") == 0)
            overlayInertialSwitch = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--stick-speed") == 0)
            stickSpeed = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--firm-feet") == 0)
            firmFeet = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--firm-limit") == 0)
            firmLimit = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--firm-handback") == 0)
            firmHandBack = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--firm-knee") == 0)
            firmKnee = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--firm-release") == 0)
            firmRelease = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--firm-latch-speed") == 0)
            firmLatch = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--turn-rate") == 0)
            turnRate = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--turn-rate-max") == 0)
            turnRateMax = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--aim-camera") == 0)
            aimCameraPose = std::atoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--body-height-smooth") == 0)
            bodyHeight = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--draw-z") == 0)
            drawZNext = std::strcmp(argv[i + 1], "same") != 0;
        else if (std::strcmp(argv[i], "--aim-camera-alt") == 0)
            aimCameraAlt = std::atoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--aim-game-upper") == 0)
            aimGameUpper = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--aim-jog-query") == 0)
            aimJogQuery = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--capsule-forecast") == 0)
            capsuleForecast = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--forecast-dump") == 0)
            forecastDump = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--step-down-reach") == 0)
            stepDownReach = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--swivel-pole") == 0)
            swivelPole = std::atoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--lean-pack") == 0)
            leanPack = std::atoi(argv[i + 1]) != 0 ? 1 : 0;
        else if (std::strcmp(argv[i], "--lean-scale") == 0)
        {
            if (std::sscanf(argv[i + 1], "%f,%f", &leanScaleBody, &leanScaleHead) != 2)
            {
                std::fprintf(stderr, "--lean-scale %s rejected (B,H)\n", argv[i + 1]);
                return 2;
            }
        }
        else if (std::strcmp(argv[i], "--lean-coverage") == 0)
            leanCoverage = std::atoi(argv[i + 1]);
        else if (std::strcmp(argv[i], "--force-drawn") == 0)
            forceDrawn = std::atoi(argv[i + 1]) != 0;
        else if (std::strcmp(argv[i], "--crouch-stop") == 0)
            _putenv_s("UTPP_TEST_CROUCH_STOP", argv[i + 1]);
        else if (std::strcmp(argv[i], "--test-env") == 0)
        {
            // NAME=VALUE of one UTPP_TEST_* switch of the matcher, set for this process only (one side of a before / after comparison)
            const std::string kv = argv[i + 1];
            const size_t eq = kv.find('=');
            if (eq == std::string::npos || kv.rfind("UTPP_TEST_", 0) != 0)
            {
                std::fprintf(stderr, "--test-env needs UTPP_TEST_<name>=<value>, got %s\n", kv.c_str());
                return 2;
            }
            _putenv_s(kv.substr(0, eq).c_str(), kv.substr(eq + 1).c_str());
        }
    }
    const bool aimOptions = neutralAim >= 0 || aimWorldSmoothHz >= 0.0f || shooterMode >= 0;
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    const auto womanPaths = DataPaths(data, "woman");
    if (!RigFromDb(DataPaths(data, man ? "man" : "woman").files.db, rig, &db))
        return 3;
    if (feedPath == "--clips")
    {
        // clip mode: <out prefix>.bin = the stored frames of every clip (one record per frame, same layout; both pose blocks = the frame,
        // meta: t = frame time in the clip, clip = clip index, dt = 1 / fps), for the content check of a recorded step
        const uint32_t nbc = db->NumBones();
        const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
        std::ofstream cbin(out + ".bin", std::ios::binary);
        std::vector<float> rec(kMetaFloats + size_t(nbc) * 14, 0.0f);
        size_t frames = 0;
        const std::string filter = argc > 5 ? argv[5] : "";
        for (uint32_t c = 0; c < db->NumClips(); ++c)
        {
            if (!filter.empty() && db->ClipName(c).find(filter) == std::string_view::npos)
                continue;
            const auto& clip = db->Clip(c);
            for (uint32_t k = 0; k < clip.numFrames; ++k)
            {
                std::fill(rec.begin(), rec.end(), 0.0f);
                rec[0] = float(k) / fps, rec[1] = 1.0f / fps, rec[5] = float(c);
                const auto* pose = db->Pose(clip.firstFrame + k);
                for (uint32_t b = 0; b < nbc; ++b)
                {
                    const float v[7] = {pose[b].px, pose[b].py, pose[b].pz, pose[b].qx, pose[b].qy, pose[b].qz, pose[b].qw};
                    std::copy(v, v + 7, rec.begin() + kMetaFloats + size_t(b) * 7);
                    std::copy(v, v + 7, rec.begin() + kMetaFloats + size_t(nbc + b) * 7);
                }
                cbin.write(reinterpret_cast<const char*>(rec.data()), std::streamsize(rec.size() * sizeof(float)));
                ++frames;
            }
        }
        std::ofstream ctxt(out + ".txt");
        ctxt << "UTPP_POSE_STAGES 1\nbones " << nbc << "\nmeta " << kMetaFloats;
        for (const char* m : kMeta)
            ctxt << ' ' << m;
        ctxt << "\nrecords " << frames << "\nfps " << fps << "\n";
        for (uint32_t b = 0; b < nbc; ++b)
            ctxt << "bone " << b << ' ' << db->BoneName(b) << ' ' << db->BoneParent(b) << '\n';
        for (uint32_t c = 0; c < db->NumClips(); ++c)
            ctxt << "clip " << c << ' ' << db->ClipName(c) << '\n';
        std::printf("clips=%u frames=%zu fps=%g\n", db->NumClips(), frames, double(fps));
        return 0;
    }
    CapsuleStatReplay capsuleStatReplay;
    std::vector<motionhost::FeedInput> inputs;
    std::vector<Frame> rows;
    {
        std::ifstream fin(feedPath);
        std::string line;
        while (std::getline(fin, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream stream(line);
            std::vector<std::string> v;
            std::string token;
            while (stream >> token)
                v.push_back(token);
            if (v.size() < 17)
                return 6;
            Frame fr;
            fr.t = std::stof(v[0]);
            fr.dt = std::stof(v[1]);
            fr.vx = std::stof(v[2]);
            fr.vy = std::stof(v[3]);
            fr.vz = std::stof(v[4]);
            fr.mvx = std::stof(v[5]);
            fr.mvy = std::stof(v[6]);
            fr.yawDeg = std::stof(v[7]);
            fr.camDeg = std::stof(v[8]);
            fr.loco = std::stoi(v[9]);
            fr.psm = int(std::stoul(v[10]));
            auto in = ToInput(fr);
            if (v.size() >= 31)
            {
                in.haveTarget = std::stoi(v[17]) != 0;
                in.targetYawDeg = std::stof(v[18]);
                in.haveExtra = true;
                for (int j = 0; j < 3; ++j)
                    in.capsulePos[j] = std::stof(v[19 + j]);
                fr.cap[0] = in.capsulePos[0];
                fr.cap[1] = in.capsulePos[1];
                in.haveAir = true;
                in.onGround = std::stoi(v[27]) != 0;
                in.distToGround = std::stof(v[28]);
                in.psmFall = std::stoi(v[29]);
                in.walkToggled = std::stoi(v[30]) != 0;
            }
            if (forceDrawn)
                in.psmBlock |= integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn;
            rows.push_back(fr);
            inputs.push_back(in);
        }
    }
    if (inputs.empty())
        return 6;
    // --ahead recorded: the body turn of the coming tick is the next feed's yaw minus this one (what the game applied after the feed);
    // without it the host derives it from its own body yaw command, which an open-loop replay never issues (0).
    if (aheadRecorded)
        for (size_t i = 0; i + 1 < inputs.size(); ++i)
            inputs[i].yawAheadDeg = std::remainder(rows[i + 1].yawDeg - rows[i].yawDeg, 360.0f);
    motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &CaptureTail,
                                  &motiondriver::OwnsTraversal};
    hooks.published = &motiondriver::RecordPublishedPose;
    g_baseDb = Wide(argv[2]);
    {
        auto woman = womanPaths, manPaths = DataPaths(data, "man");
        if (leanPack > 0)
        {
            woman.lean = data + "\\utpp_lean_woman.bin";
            manPaths.lean = data + "\\utpp_lean_man.bin";
        }
        motiondriver::SetPathsOverride(0, woman);
        motiondriver::SetPathsOverride(1, manPaths);
    }
    motiondriver::SetRigBody(man ? 1 : 0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motionhost::SetMotionHooks(&hooks);
    if (!ResetBase(rig))
        return 4;
    int applied = 0, unknown = 0;
    if (!settingsLog.empty())
    {
        ApplyLogBaseline();
        ApplyLoggedSettings(settingsLog, applied, unknown);
    }
    if (capsuleStats && !settingsLog.empty())
        capsuleStatReplay.ReadLog(settingsLog);
    if (neutralAim >= 0)
        motiondriver::SetNeutralAimOffset(neutralAim != 0);
    if (aimWorldSmoothHz >= 0.0f)
        motiondriver::SetAimWorldSmoothing(true, aimWorldSmoothHz); // 0 Hz = on without smoothing, as the script sends it
    if (shooterMode >= 0)
        motiondriver::SetShooterMode(shooterMode != 0);
    if (strideWarp >= 0)
        motiondriver::SetStrideWarp(strideWarp != 0);
    if (overlayInertialSwitch >= 0)
        motiondriver::SetOverlayInertialSwitch(overlayInertialSwitch != 0);
    if (stickSpeed >= 0)
        motiondriver::SetStickSpeedMode(stickSpeed);
    if (firmFeet >= 0)
        motionhost::SetFirmFeet(firmFeet != 0);
    if (firmLimit >= 0.0f || firmHandBack >= 0.0f || firmKnee >= 0.0f || firmRelease >= 0.0f || firmLatch >= 0.0f)
    {
        // a value not passed keeps the build default (negative)
        if (!motionhost::SetFirmFeetParams(firmLimit, firmHandBack, firmKnee, firmRelease, firmLatch))
            return 2;
    }
    if (turnRate >= 0)
        motionhost::SetTurnRate(turnRate != 0);
    if (turnRateMax >= 0.0f && !motionhost::SetTurnRateParams(0.9f, turnRateMax))
    {
        std::fprintf(stderr, "--turn-rate-max %g rejected\n", static_cast<double>(turnRateMax));
        return 2;
    }
    int32_t lastAimCameraPose = -1, lastAimCameraState = -1;
    if (aimCameraPose >= 0)
        motiondriver::SetAimGameUpper(false); // the script setting mm.aimGameUpper (default false, the DLL default too)
    if (aimGameUpper >= 0)
        motiondriver::SetAimGameUpper(aimGameUpper != 0);
    if (swivelPole >= 0 && !motionhost::SetStancePoleMode(swivelPole))
        return 2;
    if (leanScaleBody >= 0.0f && !motiondriver::SetLeanParams(leanScaleBody, leanScaleHead))
    {
        std::fprintf(stderr, "--lean-scale rejected by the driver\n");
        return 2;
    }
    if (leanCoverage >= 0 && !motiondriver::SetLeanCoverage(leanCoverage))
    {
        std::fprintf(stderr, "--lean-coverage %d rejected by the driver\n", leanCoverage);
        return 2;
    }
    motiondriver::SetEnabled(true);
    motiondriver::SetLivePoseHistory(liveHistory);
    if (turnFootPhase >= 0)
        motiondriver::SetTurnFootPhase(turnFootPhase != 0);
    motiondriver::TestSetMapOverrides(capsuleQuery, velocityPredicates, tipDeg);
    if (stepDownReach >= 0)
    {
        // the mm.stepDownReach setting: chooser mapping and runtime air state together, as the native switches them
        motiondriver::SetStepDownReach(stepDownReach != 0);
        motiondriver::TestSetMapStepDownReach(stepDownReach);
        motionhost::SetAirStepDownReach(stepDownReach != 0);
    }
    if (capsuleForecast >= 0)
        motiondriver::SetCapsuleForecast(capsuleForecast != 0);
    FILE* forecastOut = nullptr;
    if (forecastDump && out != "-")
    {
        forecastOut = std::fopen((out + ".forecast.txt").c_str(), "w");
        if (!forecastOut)
            return 7;
        std::fprintf(forecastOut, "# t valid posX posY posZ d35X d35Y d35Z d70X d70Y d70Z accel\n");
    }
    motionhost::TestSetPelvisPlacement(pelvis);
    if (bodyHeight >= 0)
        motionhost::SetBodyHeightSmooth(bodyHeight != 0);
    // body height stage (the game's animation-thread step, here once per feed on the published pose): tracker + counters
    mm::bodyheight::Filter bodyHeightFilter;
    uint32_t bodyHeightSerial = ~0u;
    uint64_t bodyHeightRows = 0, bodyHeightApplied = 0, bodyHeightAllowed = 0;
    double bodyHeightAbsMax = 0.0;
    std::vector<double> bodyHeightAbs;
    // --qpc-feed: feed time t -> QueryPerformanceCounter ticks (offset by 1 s so the first stamp is not 0, which the host reads as unset)
    LARGE_INTEGER qpcHz{};
    QueryPerformanceFrequency(&qpcHz);
    const auto feedTicks = [&](size_t aRow) {
        return int64_t((double(rows[aRow].t) - double(rows.front().t) + 1.0) * double(qpcHz.QuadPart));
    };
    if (qpcFeed)
        motionhost::TestSetQpcTicks(feedTicks(0));
    if (motionLog && out != "-")
    {
        g_motionLog = std::fopen((out + ".motion.log").c_str(), "w");
        if (!g_motionLog)
            return 7;
        g_motionLogT = rows.front().t;
        motiondriver::SetLogger(&MotionLogLine);
        motiondriver::SetTrace(true);
    }
    motiondriver::Step(inputs.front(), 1, inputs.front().dt);
    motiondriver::WaitLoader();
    const uint32_t nb = db->NumBones();
    const bool files = out != "-";
    std::ofstream bin;
    if (files)
        bin.open(out + ".bin", std::ios::binary);
    std::vector<float> rec(kMetaFloats + size_t(nb) * 14);
    // summary accumulators (see the header)
    std::vector<double> feedUs, corrCm;
    double ikCorrMax = 0, procSum = 0, capSum = 0, turnProc = 0, turnYaw = 0, turnTime = 0, procTurnsSum = 0;
    int lastPickClip = -2;
    uint32_t clipChanges = 0, turns = 0;
    feedUs.reserve(inputs.size());
    // stop / start picks: first-frame foot offset (cm, Root space, the picked clip frame vs the shown feet of the previous feed) and the
    // planted toe drift in the world during the first 0.3 s after the pick (cm, max over both feet while the tail contact >= 0.5)
    const auto boneIndex = [&](const char* aName) {
        for (uint32_t b = 0; b < nb; ++b)
            if (db->BoneName(b) == aName)
                return int32_t(b);
        return int32_t(-1);
    };
    const int32_t footBones[2] = {boneIndex("LeftFoot"), boneIndex("RightFoot")};
    const int32_t toeBones[2] = {boneIndex("LeftToeBase"), boneIndex("RightToeBase")};
    std::vector<double> pickOffsets[2], drifts[2], plantedOffsets[2], plantedOffsetsDrv[2], plantedOffsetsHip[2];
    math::Vec3 driverFeet[2], driverHips;
    const int32_t hipsBone = boneIndex("Hips");
    bool haveDriverFeet = false;
    // steering probe: yaw the steering nodes applied to the head layer per feed (deg) against the procedural yaw readout (deg)
    const auto& steerProbe = mm::motionstack::g_testSteeringProbe;
    double steerPrevAbs = 0, steerSum = 0, procSumAll = 0, sxy = 0, sxx = 0, syy = 0, sx = 0, sy = 0, steerTurnSum = 0, steerTurn = 0, lastSteerDeg = 0;
    uint64_t steerFeeds = 0;
    float prevContact[2] = {0.0f, 0.0f};
    math::Vec3 shownFeet[2];
    bool haveShownFeet = false, driftAnchor[2] = {false, false};
    double driftLeft = 0, driftMax = 0, driftAt[2][2] = {};
    int driftCls = 0;
    auto closeTurn = [&] {
        if (turnTime >= 0.1 && turnYaw >= 20.0)
        {
            ++turns;
            procTurnsSum += turnProc;
        }
        turnProc = turnYaw = turnTime = 0;
    };
    std::map<int, std::string> clipsSeen;
    size_t written = 0, missing = 0;
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        g_log = {};
        g_tailValid = false;
        if (qpcFeed)
            motionhost::TestSetQpcTicks(feedTicks(i));
        if (airFeed)
        {
            motiondriver::FeedAirFromInput(inputs[i]);
            ++airFeeds;
            airFeedRows += inputs[i].haveAir ? 1 : 0;
        }
        if (aimCameraPose >= 0)
        {
            // the camera / aim target the script feeds every frame: a level point 60 m along the recorded camera yaw (actor axes)
            const bool drawn = (inputs[i].psmBlock & integ::kPsmWeaponDrawn) != 0;
            const bool aim = integ::PsmUpperBodyOf(inputs[i].psmBlock) == integ::kPsmUpperBodyAim;
            const bool alt = drawn && aimCameraAlt >= 0 && (aimCameraDrawnFeeds / 90) % 2 == 1;
            aimCameraDrawnFeeds += drawn ? 1 : 0;
            const int32_t pose = drawn ? (alt ? aimCameraAlt : aimCameraPose) : 0, state = aim ? 2 : (drawn ? 1 : 0);
            if (pose != lastAimCameraPose || state != lastAimCameraState)
            {
                motiondriver::SetOverlayInputs(pose, state, false, false);
                lastAimCameraPose = pose;
                lastAimCameraState = state;
            }
            const float rel = std::remainder(inputs[i].cameraYawDeg - inputs[i].playerYawDeg, 360.0f) * 0.017453292519943f;
            const float point[3] = {-std::sin(rel) * 60.0f, std::cos(rel) * 60.0f, 1.4f};
            const bool finite = std::isfinite(point[0]) && std::isfinite(point[1]);
            motiondriver::SetCameraTarget(true, finite, point);
            motiondriver::SetAimTarget(aim, aim && finite, point);
            ++aimCameraFeeds;
            aimCameraAimFeeds += aim ? 1 : 0;
        }
        g_motionLogT = rows[i].t;
        const auto feed0 = std::chrono::steady_clock::now();
        motionhost::Feed(inputs[i]);
        feedUs.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - feed0).count());
        if (capsuleStats)
            capsuleStatReplay.AfterFeed(rows[i].loco, i + 1 < rows.size() ? double(rows[i + 1].t) : 1e30);
        if (aimJogQuery)
        {
            // the script's query right after the feed: the next feed's mapping sees the answer (Main.cpp Native_MMAimJogTarget)
            const float jog = motionhost::AimJogTarget(rows[i].mvx, rows[i].mvy);
            motiondriver::SetAimJogOff(!(jog > 0.0f));
            ++aimJogQueries;
            aimJogOnFeeds += jog > 0.0f ? 1 : 0;
        }
        const bool published = ReadPublished();
        if (mm::bodyheight::rt::Enabled())
        {
            const uint32_t gate = mm::bodyheight::rt::g_gate.load(std::memory_order_acquire);
            if ((gate >> 8) != bodyHeightSerial)
            {
                bodyHeightSerial = gate >> 8;
                bodyHeightFilter.Reset();
            }
            const size_t zi = drawZNext ? std::min(i + 1, inputs.size() - 1) : i;
            const bool haveZ = inputs[zi].haveExtra && std::isfinite(inputs[zi].capsulePos[2]);
            float off = 0.0f;
            if (haveZ)
                off = bodyHeightFilter.Step(bodyHeightRows ? rows[i].dt : 0.0f, inputs[zi].capsulePos[2], (gate & 1u) != 0,
                                              (gate & 2u) != 0);
            else
                bodyHeightFilter.Reset();
            ++bodyHeightRows;
            bodyHeightAllowed += (gate & 1u) != 0 ? 1 : 0;
            if (published && mm::bodyheight::rt::ApplyToSnapshot(g_read, off))
            {
                ++bodyHeightApplied;
                bodyHeightAbs.push_back(std::fabs(double(off)) * 100.0);
                bodyHeightAbsMax = std::max(bodyHeightAbsMax, std::fabs(double(off)) * 100.0);
            }
        }
        if (forecastOut)
        {
            const auto fp = motiondriver::TestForecastProbe();
            std::fprintf(forecastOut, "%.6f %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", double(rows[i].t), fp.valid ? 1 : 0,
                         double(fp.pos[0]), double(fp.pos[1]), double(fp.pos[2]), double(fp.d35[0]), double(fp.d35[1]), double(fp.d35[2]),
                         double(fp.d70[0]), double(fp.d70[1]), double(fp.d70[2]), double(fp.accel));
        }
        if (rows[i].t < from || rows[i].t > to)
            continue;
        {
            constexpr double kRadToDeg = 57.29577951308232;
            const std::string sst = motiondriver::Status(), hst = motionhost::Status();
            const std::string pk = StatusField(sst, "pick=clip:");
            const int pc = pk.empty() ? -1 : std::atoi(pk.c_str());
            if (pc >= 0 && lastPickClip >= 0 && pc != lastPickClip)
            {
                ++clipChanges;
                // a pick into a stop / start clip: first-frame foot offset against the shown feet of the previous feed
                const std::string cname = uint32_t(pc) < db->NumClips() ? std::string(db->ClipName(uint32_t(pc))) : std::string();
                const int cls = StopStartClass(cname);
                if (cls >= 0)
                {
                    if (driftLeft > 0.0) // a new stop / start pick closes the running window
                        drifts[driftCls].push_back(driftMax);
                    const auto at = pk.find('@');
                    const float ct = at == std::string::npos ? 0.f : float(std::atof(pk.c_str() + at + 1));
                    const auto& clip = db->Clip(uint32_t(pc));
                    const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
                    const uint32_t k = std::min<uint32_t>(clip.numFrames ? clip.numFrames - 1 : 0, uint32_t(std::max(0.0f, ct * fps + 0.5f)));
                    const auto* cpose = db->Pose(clip.firstFrame + k);
                    math::Vec3 clipFeet[2];
                    const bool okClip = FeetAt(*db, footBones, 2,
                                               [&](uint32_t b, math::Vec3& p, math::Quat& q) {
                                                   p = math::Vec3{cpose[b].px, cpose[b].py, cpose[b].pz};
                                                   q = math::Quat{cpose[b].qx, cpose[b].qy, cpose[b].qz, cpose[b].qw};
                                                   return true;
                                               },
                                               clipFeet);
                    if (okClip && haveShownFeet)
                    {
                        double off = 0;
                        for (int s = 0; s < 2; ++s)
                            off = std::max(off, 100.0 * std::hypot(double(clipFeet[s].x - shownFeet[s].x), double(clipFeet[s].y - shownFeet[s].y)));
                        pickOffsets[cls].push_back(off);
                        // the foot planted in the shown pose (tail contact of the previous feed): does the new clip keep it?
                        double planted = -1.0;
                        for (int s = 0; s < 2; ++s)
                            if (prevContact[s] >= 0.5f)
                                planted = std::max(planted, 100.0 * std::hypot(double(clipFeet[s].x - shownFeet[s].x), double(clipFeet[s].y - shownFeet[s].y)));
                        if (planted >= 0.0)
                            plantedOffsets[cls].push_back(planted);
                        // the same against the driver pose of the previous feed (no foot locker correction in it)
                        double plantedDrv = -1.0, offDrv[2] = {-1.0, -1.0};
                        if (haveDriverFeet)
                            for (int s = 0; s < 2; ++s)
                            {
                                offDrv[s] = 100.0 * std::hypot(double(clipFeet[s].x - driverFeet[s].x), double(clipFeet[s].y - driverFeet[s].y));
                                if (prevContact[s] >= 0.5f)
                                    plantedDrv = std::max(plantedDrv, offDrv[s]);
                            }
                        if (plantedDrv >= 0.0)
                            plantedOffsetsDrv[cls].push_back(plantedDrv);
                        // the same relative to the Hips (horizontal): the runtime pose carries the visual root offset (the body up to the
                        // offset radius behind its Root), the stored clip frame does not; Hips-relative feet compare the two poses alone
                        double plantedHip = -1.0;
                        if (haveDriverFeet && hipsBone >= 0 && db->BoneParent(uint32_t(hipsBone)) < 2)
                        {
                            const float hx = cpose[hipsBone].px, hy = cpose[hipsBone].py;
                            for (int s = 0; s < 2; ++s)
                                if (prevContact[s] >= 0.5f)
                                    plantedHip = std::max(plantedHip, 100.0 * std::hypot(double((clipFeet[s].x - hx) - (driverFeet[s].x - driverHips.x)),
                                                                                        double((clipFeet[s].y - hy) - (driverFeet[s].y - driverHips.y))));
                        }
                        if (plantedHip >= 0.0)
                            plantedOffsetsHip[cls].push_back(plantedHip);
                        if (pickDump)
                            std::printf("PICKDUMP t=%.3f cls=%s clip=%s ct=%.3f contactL=%.2f contactR=%.2f shownOffL=%.2f shownOffR=%.2f "
                                        "drvOffL=%.2f drvOffR=%.2f clipL=%.3f,%.3f clipR=%.3f,%.3f shownL=%.3f,%.3f shownR=%.3f,%.3f "
                                        "drvL=%.3f,%.3f drvR=%.3f,%.3f\n",
                                        double(rows[i].t), cls ? "start" : "stop", cname.c_str(), double(ct), double(prevContact[0]),
                                        double(prevContact[1]),
                                        100.0 * std::hypot(double(clipFeet[0].x - shownFeet[0].x), double(clipFeet[0].y - shownFeet[0].y)),
                                        100.0 * std::hypot(double(clipFeet[1].x - shownFeet[1].x), double(clipFeet[1].y - shownFeet[1].y)),
                                        offDrv[0], offDrv[1], double(clipFeet[0].x), double(clipFeet[0].y), double(clipFeet[1].x),
                                        double(clipFeet[1].y), double(shownFeet[0].x), double(shownFeet[0].y), double(shownFeet[1].x),
                                        double(shownFeet[1].y), double(driverFeet[0].x), double(driverFeet[0].y), double(driverFeet[1].x),
                                        double(driverFeet[1].y));
                    }
                    driftCls = cls;
                    driftLeft = 0.3;
                    driftMax = 0;
                    driftAnchor[0] = driftAnchor[1] = false;
                }
            }
            if (pc >= 0)
                lastPickClip = pc;
            haveDriverFeet = g_tailValid && g_driverPose.size() == size_t(nb) * 7 &&
                             FeetAt(*db, footBones, 2, [&](uint32_t b, math::Vec3& p, math::Quat& q) {
                                 const float* v = g_driverPose.data() + size_t(b) * 7;
                                 p = math::Vec3{v[0], v[1], v[2]};
                                 q = math::Quat{v[3], v[4], v[5], v[6]};
                                 return true;
                             }, driverFeet);
            if (haveDriverFeet && hipsBone >= 0)
                driverHips = math::Vec3{g_driverPose[size_t(hipsBone) * 7], g_driverPose[size_t(hipsBone) * 7 + 1], g_driverPose[size_t(hipsBone) * 7 + 2]};
            prevContact[0] = g_tailValid ? g_tail.contact[0] : 0.0f;
            prevContact[1] = g_tailValid ? g_tail.contact[1] : 0.0f;
            // shown feet of this feed (Root space, for the next pick) and the planted drift window after a stop / start pick
            haveShownFeet = published && g_read.numBones == nb &&
                            FeetAt(*db, footBones, 2, [&](uint32_t b, math::Vec3& p, math::Quat& q) {
                                const float* v = g_read.pose + size_t(b) * 7;
                                p = math::Vec3{v[0], v[1], v[2]};
                                q = math::Quat{v[3], v[4], v[5], v[6]};
                                return true;
                            }, shownFeet);
            if (haveShownFeet) // the published pose carries the visual root offset on the root's children: back to the animation Root space
            {
                const float ox = Num(hst, "offX=") / 100.0f, oy = Num(hst, "offY=") / 100.0f;
                const float oyaw = Num(hst, "offYaw=") * 3.14159265f / 180.0f;
                if (std::isfinite(ox) && std::isfinite(oy) && std::isfinite(oyaw))
                    for (math::Vec3& f : shownFeet)
                    {
                        const float x = f.x - ox, y = f.y - oy, c = std::cos(-oyaw), s = std::sin(-oyaw);
                        f = math::Vec3{c * x - s * y, s * x + c * y, f.z};
                    }
            }
            if (driftLeft > 0.0)
            {
                math::Vec3 toes[2];
                const bool okToes = published && g_read.numBones == nb && g_tailValid &&
                                    FeetAt(*db, toeBones, 0, [&](uint32_t b, math::Vec3& p, math::Quat& q) {
                                        const float* v = g_read.pose + size_t(b) * 7;
                                        p = math::Vec3{v[0], v[1], v[2]};
                                        q = math::Quat{v[3], v[4], v[5], v[6]};
                                        return true;
                                    }, toes);
                // world: the drawn capsule of the next feed (the frame the pose is shown in)
                const Frame& nx = rows[std::min(i + 1, rows.size() - 1)];
                const double yw = double(nx.yawDeg) * 3.14159265358979 / 180.0;
                for (int s = 0; s < 2 && okToes; ++s)
                {
                    const double wx = std::cos(yw) * toes[s].x - std::sin(yw) * toes[s].y + nx.cap[0];
                    const double wy = std::sin(yw) * toes[s].x + std::cos(yw) * toes[s].y + nx.cap[1];
                    if (g_tail.contact[s] < 0.5f)
                    {
                        driftAnchor[s] = false;
                        continue;
                    }
                    if (!driftAnchor[s])
                    {
                        driftAnchor[s] = true;
                        driftAt[s][0] = wx, driftAt[s][1] = wy;
                    }
                    driftMax = std::max(driftMax, 100.0 * std::hypot(wx - driftAt[s][0], wy - driftAt[s][1]));
                }
                driftLeft -= std::max(1e-4, double(rows[i].dt));
                if (driftLeft <= 0.0)
                    drifts[driftCls].push_back(driftMax);
            }
            for (const char* key : {"corrL=", "corrR="})
            {
                const float c = Num(hst, key);
                if (std::isfinite(c) && c > 0.0f)
                {
                    corrCm.push_back(c);
                    ikCorrMax = std::max(ikCorrMax, double(c));
                }
            }
            const motiondriver::RootProbe pr = motiondriver::TestRootProbe();
            double proc = 0;
            if (pr.prepared)
            {
                proc = std::remainder(double(pr.capsuleYawDelta) + double(pr.after[2]) - double(pr.before[2]) - double(pr.animYawDelta),
                                      2.0 * 3.14159265358979) * kRadToDeg;
                procSum += std::fabs(proc);
                capSum += std::fabs(double(pr.capsuleYawDelta)) * kRadToDeg;
            }
            {
                const double steerDeg = (steerProbe.appliedAbsRad - steerPrevAbs) * kRadToDeg;
                steerPrevAbs = steerProbe.appliedAbsRad;
                if (pr.prepared)
                {
                    const double y = std::fabs(proc);
                    ++steerFeeds;
                    steerSum += steerDeg, procSumAll += y;
                    sx += steerDeg, sy += y, sxx += steerDeg * steerDeg, syy += y * y, sxy += steerDeg * y;
                }
                lastSteerDeg = steerDeg;
            }
            const double yawStep = i > 0 ? std::fabs(std::remainder(double(rows[i].yawDeg) - double(rows[i - 1].yawDeg), 360.0)) : 0.0;
            const double fdt = std::max(1e-4, double(rows[i].dt));
            if (yawStep / fdt > 30.0)
            {
                turnTime += fdt;
                turnYaw += yawStep;
                turnProc += std::fabs(proc);
                steerTurn += lastSteerDeg;
            }
            else
            {
                if (turnTime >= 0.1 && turnYaw >= 20.0)
                    steerTurnSum += steerTurn;
                steerTurn = 0;
                closeTurn();
            }
        }
        if (!files)
        {
            ++written;
            continue;
        }
        if (!published)
            ++missing;
        const std::string st = motiondriver::Status();
        if (leanPack >= 0)
            leanLoaded = std::max(leanLoaded, LeanPackLoaded(st));
        const std::string pick = StatusField(st, "pick=clip:");
        const auto at = pick.find('@'), slash = pick.find("/db");
        const int clip = pick.empty() ? -1 : std::atoi(pick.c_str());
        const float ct = at == std::string::npos ? 0.f : float(std::atof(pick.substr(at + 1, slash - at - 1).c_str()));
        const int dbi = slash == std::string::npos ? -1 : std::atoi(pick.c_str() + slash + 3);
        if (clip >= 0 && uint32_t(clip) < db->NumClips())
            clipsSeen.emplace(clip, std::string(db->ClipName(uint32_t(clip))));
        const std::string hs = motionhost::Status();
        const motiondriver::RootProbe rp = motiondriver::TestRootProbe();
        if (firmFeet >= 0)
        {
            uint64_t now[4] = {};
            motionhost::FirmFeetStats(&now[0], &now[1], &now[2], &now[3]);
            for (int k = 0; k < 4; ++k)
            {
                firmTotal[k] += now[k] >= firmLast[k] ? now[k] - firmLast[k] : now[k];
                firmLast[k] = now[k];
            }
        }
        float aod[15];
        motiondriver::TestAoDiag(aod);
        const float meta[kMetaFloats] = {rows[i].t, rows[i].dt, g_log.stepResult ? 1.f : 0.f, published ? 1.f : 0.f, g_tailValid ? 1.f : 0.f,
                                         float(clip), ct, float(dbi), g_tail.contact[0], g_tail.contact[1], g_tail.rootVel[0],
                                         g_tail.rootVel[1], g_tail.rootYawRate, g_tail.providedRoot ? 1.f : 0.f, g_tail.rootOffsetX,
                                         g_tail.rootOffsetY, g_tail.rootOffsetYaw, Num(hs, "offX="), Num(hs, "offY="), Num(hs, "offYaw="),
                                         Num(hs, "offClamps="), Num(hs, "offResets="), Num(hs, "offTeleports="), Num(hs, "lockL="),
                                         Num(hs, "wL="), Num(hs, "corrL="), Num(hs, "lockR="), Num(hs, "wR="), Num(hs, "corrR="),
                                         Num(st, "rate="), rows[i].cap[0], rows[i].cap[1], rows[i].yawDeg, rows[i].mvx, rows[i].mvy,
                                         float(rows[i].loco), rp.prepared ? 1.f : 0.f, rp.capsuleDelta[0], rp.capsuleDelta[1],
                                         rp.capsuleYawDelta, rp.animDelta[0], rp.animDelta[1], rp.animYawDelta, rp.movingGround ? 1.f : 0.f,
                                         rp.montage ? 1.f : 0.f, rp.before[0], rp.before[1], rp.before[2], rp.after[0], rp.after[1], rp.after[2],
                                         rp.yawAhead, motiondriver::TestAoGridShare(), aod[0], aod[1], aod[2], aod[3], aod[4], aod[5],
                                         aod[6], aod[7], aod[8], aod[9], aod[10], aod[11], aod[12], aod[13], aod[14]};
        std::copy(meta, meta + kMetaFloats, rec.begin());
        std::fill(rec.begin() + kMetaFloats, rec.end(), std::nanf(""));
        if (g_tailValid && g_driverPose.size() == size_t(nb) * 7)
            std::copy(g_driverPose.begin(), g_driverPose.end(), rec.begin() + kMetaFloats);
        if (published && g_read.numBones == nb)
            std::copy(g_read.pose, g_read.pose + size_t(nb) * 7, rec.begin() + kMetaFloats + size_t(nb) * 7);
        bin.write(reinterpret_cast<const char*>(rec.data()), std::streamsize(rec.size() * sizeof(float)));
        ++written;
    }
    if (turnTime >= 0.1 && turnYaw >= 20.0)
        steerTurnSum += steerTurn;
    closeTurn();
    if (driftLeft > 0.0)
        drifts[driftCls].push_back(driftMax);
    const uint64_t liveHistorySearches = motiondriver::TestLiveHistorySearches();
    if (bodyHeightRows)
    {
        std::sort(bodyHeightAbs.begin(), bodyHeightAbs.end());
        const double p99 = bodyHeightAbs.empty() ? 0.0 : bodyHeightAbs[std::min(bodyHeightAbs.size() - 1, size_t(0.99 * double(bodyHeightAbs.size())))];
        const auto& fd = bodyHeightFilter.Diagnostics();
        std::printf("BODYHEIGHT draw=%s rows=%llu allowed=%llu applied=%llu offAbsP99Cm=%.2f offAbsMaxCm=%.2f resets=%llu hitches=%llu "
                    "clamps=%llu legSolves=%llu legClamps=%llu ankleErrMaxMm=%.2f\n",
                    drawZNext ? "next" : "same", static_cast<unsigned long long>(bodyHeightRows),
                    static_cast<unsigned long long>(bodyHeightAllowed), static_cast<unsigned long long>(bodyHeightApplied), p99,
                    bodyHeightAbsMax, static_cast<unsigned long long>(fd.resets), static_cast<unsigned long long>(fd.hitches),
                    static_cast<unsigned long long>(fd.clamps), static_cast<unsigned long long>(mm::bodyheight::rt::g_legSolves.load()),
                    static_cast<unsigned long long>(mm::bodyheight::rt::g_legClamps.load()),
                    double(mm::bodyheight::rt::g_maxAnkleError.load()) * 1000.0);
    }
    if (forecastOut)
        std::fclose(forecastOut);
    motionhost::SetMotionHooks(nullptr);
    motionhost::TestSetQpcTicks(-1);
    if (g_motionLog)
    {
        motiondriver::SetTrace(false);
        motiondriver::SetLogger(nullptr);
        std::fclose(g_motionLog);
        g_motionLog = nullptr;
    }
    motiondriver::Shutdown();
    motionhost::Shutdown();
    {
        auto pct = [](std::vector<double> v, double q) {
            if (v.empty())
                return 0.0;
            std::sort(v.begin(), v.end());
            return v[std::min(v.size() - 1, size_t(q * double(v.size())))];
        };
        double secs = 0;
        for (const Frame& r : rows)
            if (r.t >= from && r.t <= to)
                secs += r.dt;
        std::string name = std::filesystem::path(feedPath).filename().string();
        const std::filesystem::path logs = std::filesystem::path(feedPath).parent_path();
        if (logs.filename() == "logs" && !logs.parent_path().filename().empty())
            name = logs.parent_path().filename().string();
        std::printf("RUNTIME run=%s secs=%.1f clipChangesPerS=%.3f ikCorrMaxCm=%.2f ikCorrP95Cm=%.2f turns=%u procYawPerTurnDeg=%.2f "
                    "procYawShare=%.4f feedUsP50=%.1f feedUsP95=%.1f historySearches=%llu\n",
                    name.c_str(), secs, clipChanges / std::max(1e-6, secs), ikCorrMax, pct(corrCm, 0.95), turns,
                    turns ? procTurnsSum / turns : 0.0, capSum > 0 ? procSum / capSum : 0.0, pct(feedUs, 0.5), pct(feedUs, 0.95),
                    static_cast<unsigned long long>(liveHistorySearches));
        std::printf("TURNFOOT run=%s searches=%llu\n", name.c_str(), static_cast<unsigned long long>(motiondriver::TestTurnFootPhaseSearches()));
        if (capsuleStats)
            std::printf("CAPSTAT run=%s loggedSamples=%llu logSegments=%llu accelQueries=%llu accelModifierChanges=%llu decelQueries=%llu decelModifierChanges=%llu\n",
                        name.c_str(), static_cast<unsigned long long>(capsuleStatReplay.logged),
                        static_cast<unsigned long long>(capsuleStatReplay.segments),
                        static_cast<unsigned long long>(capsuleStatReplay.accelQueries),
                        static_cast<unsigned long long>(capsuleStatReplay.accelKept),
                        static_cast<unsigned long long>(capsuleStatReplay.decelQueries),
                        static_cast<unsigned long long>(capsuleStatReplay.decelKept));
        if (aimCameraPose >= 0)
            std::printf("AIMCAM run=%s pose=%d feeds=%llu aimFeeds=%llu\n", name.c_str(), aimCameraPose,
                        static_cast<unsigned long long>(aimCameraFeeds), static_cast<unsigned long long>(aimCameraAimFeeds));
        if (aimGameUpper >= 0)
            std::printf("AIMGAMEUPPER run=%s on=%d handback=%s\n", name.c_str(), aimGameUpper,
                        StatusField(motiondriver::Status(), " handback=").c_str());
        if (aimOptions)
        {
            uint64_t neutralFeeds = 0, smoothFeeds = 0;
            motiondriver::TestAimOptionFeeds(neutralFeeds, smoothFeeds);
            std::printf("AIMOPT run=%s neutralAim=%d aimWorldSmoothHz=%.2f shooterMode=%d neutralAoFeeds=%llu aimSmoothFeeds=%llu\n", name.c_str(),
                        neutralAim, aimWorldSmoothHz, shooterMode, static_cast<unsigned long long>(neutralFeeds),
                        static_cast<unsigned long long>(smoothFeeds));
        }
        if (firmFeet >= 0)
            std::printf("FIRMFEET run=%s on=%d latches=%llu heldUpdates=%llu legBoundUpdates=%llu handBacks=%llu\n", name.c_str(), firmFeet,
                        static_cast<unsigned long long>(firmTotal[0]), static_cast<unsigned long long>(firmTotal[1]),
                        static_cast<unsigned long long>(firmTotal[2]), static_cast<unsigned long long>(firmTotal[3]));
        if (turnRate >= 0 || turnRateMax >= 0.0f) // turn clip rate state of the legacy matcher (raised feeds, turns, cap)
            std::printf("TURNRATE run=%s %s\n", name.c_str(), motionhost::TurnRateStatus().c_str());
        if (leanPack >= 0) // loaded: the pack state of the installed bundle (1 / 0), -1 = no bundle seen
            std::printf("LEANPACK run=%s requested=%d loaded=%d\n", name.c_str(), leanPack, leanLoaded);
        if (aimJogQuery)
            std::printf("AIMJOG run=%s queries=%llu jogFeeds=%llu\n", name.c_str(), static_cast<unsigned long long>(aimJogQueries),
                        static_cast<unsigned long long>(aimJogOnFeeds));
        if (airFeed) // only with the option: the scripts refuse a tool that was asked for it and printed no AIRFEED line
            std::printf("AIRFEED run=%s forwarded=%llu withAirFields=%llu\n", name.c_str(), static_cast<unsigned long long>(airFeeds),
                        static_cast<unsigned long long>(airFeedRows));
        // PICKS line: per class (stop, start) picks, wrong-foot picks (first-frame foot offset > 10 cm), offset p50 / p95 (cm), planted
        // drift in the first 0.3 s mean / p95 (cm)
        const auto mean = [](const std::vector<double>& v) {
            double a = 0;
            for (double x : v)
                a += x;
            return v.empty() ? 0.0 : a / double(v.size());
        };
        std::printf("PICKS run=%s", name.c_str());
        const char* cn[2] = {"stop", "start"};
        for (int c = 0; c < 2; ++c)
        {
            size_t wrong = 0;
            for (double o : pickOffsets[c])
                wrong += o > 10.0;
            size_t keep = 0, keepDrv = 0, keepHip = 0;
            for (double o : plantedOffsetsHip[c])
                keepHip += o <= 10.0;
            for (double o : plantedOffsets[c])
                keep += o <= 10.0;
            for (double o : plantedOffsetsDrv[c])
                keepDrv += o <= 10.0;
            std::printf(" %sPicks=%zu %sWrongFoot=%zu %sOffP50Cm=%.2f %sOffP95Cm=%.2f %sDriftN=%zu %sDriftMeanCm=%.2f %sDriftP95Cm=%.2f"
                        " %sPlantedN=%zu %sPlantedKept=%zu %sPlantedOffP50Cm=%.2f",
                        cn[c], pickOffsets[c].size(), cn[c], wrong, cn[c], pct(pickOffsets[c], 0.5), cn[c], pct(pickOffsets[c], 0.95), cn[c],
                        drifts[c].size(), cn[c], mean(drifts[c]), cn[c], pct(drifts[c], 0.95), cn[c], plantedOffsets[c].size(), cn[c], keep,
                        cn[c], pct(plantedOffsets[c], 0.5));
            std::printf(" %sPlantedDrvN=%zu %sPlantedDrvKept=%zu %sPlantedDrvOffP50Cm=%.2f", cn[c], plantedOffsetsDrv[c].size(), cn[c],
                        keepDrv, cn[c], pct(plantedOffsetsDrv[c], 0.5));
            std::printf(" %sPlantedHipN=%zu %sPlantedHipKept=%zu %sPlantedHipOffP50Cm=%.2f", cn[c], plantedOffsetsHip[c].size(), cn[c],
                        keepHip, cn[c], pct(plantedOffsetsHip[c], 0.5));
        }
        std::printf("\n");
        // STEER line: the yaw the steering nodes applied to the head layer (blend stack probe) against the procedural yaw readout, over
        // the feeds with a prepared root frame: sums (deg), per-turn sum of the probe and the Pearson correlation per feed
        const double n = double(steerFeeds);
        const double den = std::sqrt(std::max(0.0, n * sxx - sx * sx) * std::max(0.0, n * syy - sy * sy));
        std::printf("STEER run=%s feeds=%llu calls=%llu steerAbsDeg=%.1f procAbsDeg=%.1f ratio=%.3f corr=%.3f steerPerTurnDeg=%.2f\n",
                    name.c_str(), static_cast<unsigned long long>(steerFeeds), static_cast<unsigned long long>(steerProbe.calls), steerSum,
                    procSumAll, procSumAll > 0 ? steerSum / procSumAll : 0.0, den > 0 ? (n * sxy - sx * sy) / den : 0.0,
                    turns ? steerTurnSum / turns : 0.0);
    }
    if (!files)
    {
        std::printf("feeds=%zu written=%zu unpublished=%zu settingsApplied=%d settingsUnknown=%d\n", inputs.size(), written, missing,
                    applied, unknown);
        return 0;
    }
    std::ofstream txt(out + ".txt");
    txt << "UTPP_POSE_STAGES 1\nbones " << nb << "\nmeta " << kMetaFloats;
    for (const char* m : kMeta)
        txt << ' ' << m;
    txt << "\nrecords " << written << "\n";
    for (uint32_t b = 0; b < nb; ++b)
        txt << "bone " << b << ' ' << db->BoneName(b) << ' ' << db->BoneParent(b) << '\n';
    for (const auto& [id, name] : clipsSeen)
        txt << "clip " << id << ' ' << name << '\n';
    std::printf("feeds=%zu written=%zu unpublished=%zu settingsApplied=%d settingsUnknown=%d\n", inputs.size(), written, missing, applied,
                unknown);
    return 0;
}

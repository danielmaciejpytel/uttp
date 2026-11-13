// UTPP 0.5.3 offline regression test: the automated gamepad scenarios of 2026-10-07 (session
// 20261007-013459, UTPP 0.5.2, a former test database) replayed through the real MMDriver with a stick-driven game model.
// Usage: utpp_mm_loco_test <database.utppmm> [extra.utppmm ...] (first normally utpp_base.utppmm)
// 0.5.4: a play recording (the recorded log, 01:44:24-01:46:44) - crouch movement with
// idle clips, walk pivots / turns at a run, a database without sprint clips - replayed on the first and every extra
// database in three modes: 0.5.2 (all fixes off), 0.5.3 (speed bands off) and 0.5.4.
//
// 0.6.0 (Section060): a play recording - fists attacks overwritten by MM
// (11:50:10-11:51:30), RunToCrouch on every run -> crouch (11:44:34-11:46:16) - and a second play log (standing
// pose over the bike for 0.12 s after the vehicle bit, 11:57:04.896); melee gate / stance clip speed / mount cut on vs off.
// 0.6.2 (Section062): test 2 (14:07-14:22: "fists attacks and block =
// V crouches", "after leaving the car / bike V crouches") - upper-body bone mask: fists attack mid-run, fists block while
// idle (UpperBody Aim + AimWalk), gun aim walk, car / bike exit window; negative controls (mask off = 0.6.1, scene / dash /
// vehicle ride / fall keep the full close, state bits).
// 0.5.8: a play recording (a former test database) - dash bursts at a sprint,
// stance double flip, same-direction crouch turns, loops out of their speed band, 180 deg starts after a stop - on every
// database in modes 0.5.7 (0.5.8 switches off) and 0.5.8 (Section058).
// Game model (from the traces): Locomotion reports |move| = (|stick| - 0.3) / 0.7 straight ahead of the body (mv=(0,m)),
// the world target = camera yaw + stick angle; capsule speed from |move| (0.18 -> 0.30, 0.36 -> 0.59, 0.82 -> 2.5,
// 1.0 -> 3.85 m/s; measured) with an 8 m/s^2 vector acceleration limit; the base turn moves the body towards the target at 540 deg/s
// and UTPP's body-yaw command replaces it while active (UTPPMMBodyYawCommand). 60 Hz.
// Each scenario runs with the 0.5.3 fixes and with all of them off (0.5.2) - the 0.5.2 run must show the in-game
// problem (negative control), the 0.5.3 run must meet the criteria. Arcs (passing in game) must not get worse.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "TestFailFast.hpp" // mutation early exit: UtppFailFastNote(<FAIL line>) after every printed FAIL line
#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/State.hpp"
#include "../src/mm/Steering.hpp"
#include "../src/mm/Strafe.hpp"
#include "../src/mm/ChestAlign.hpp" // B2 strafe S2: test-side chest twist measurement
#include "../src/mm/SpeedBands.hpp"
#include "../src/mm/SpeedCalibration.hpp"
#include "../src/mm/FootLocker.hpp" // loco C1b: FootReplantConfig defaults
#include "../src/mm/VehicleTransition.hpp" // vehicle batch W1 P2: simulated animation-thread side (FkSide)

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

namespace
{
using namespace utpp::motionhost;
constexpr float kDegT = 0.017453292f;
int g_pass = 0, g_fail = 0;
void Check(const std::string& aName, bool aOk, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName.c_str(), aDetail.empty() ? "" : " -- ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote("[FAIL] " + aName + (aDetail.empty() ? "" : " -- ") + aDetail);
}

float Field(const std::string& s, const char* key)
{
    const std::string k = std::string(" ") + key + "=";
    const size_t p = s.find(k);
    return p == std::string::npos ? 0.0f : float(std::atof(s.c_str() + p + k.size()));
}
std::string ClipOf(const std::string& s)
{
    const size_t p = s.find(" clip=");
    if (p == std::string::npos)
        return {};
    const size_t a = s.find('(', p), b = s.find(')', a);
    return (a == std::string::npos || b == std::string::npos) ? std::string() : s.substr(a + 1, b - a - 1);
}
uint32_t HexField(const std::string& s, const char* key)
{
    const std::string k = std::string(" ") + key + "=0x";
    const size_t p = s.find(k);
    return p == std::string::npos ? 0u : uint32_t(std::strtoul(s.c_str() + p + k.size(), nullptr, 16));
}
float WrapDeg(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

// trace cost of the last transition (MMTRACE TRANS lines through the driver logger)
std::vector<std::pair<std::string, float>> g_trans;
std::vector<int> g_transFrame;   // 0.5.7: clip frame of each TRANS line
std::vector<float> g_transTime;  // 0.5.7: simulation clock of each TRANS line
float g_clock = 0.0f;            // 0.5.7: simulation clock (s), advanced by every RunScenario feed
std::vector<float> g_transW, g_transV, g_transMv; // 0.5.8: weight, capsule |v| and stick of each TRANS line
std::vector<std::string> g_infoLines; // 0.5.4: calibration / band / warning lines
// loco C0 / C1a: feet / footstep trace lines, MMTRACE TRANS / tick lines (count), crouch idle exclusion info lines
std::vector<std::string> g_feetLines, g_footstepLines, g_exclLines;
int g_mainTraceLines = 0;
void TraceLog(const char* aLine)
{
    if (std::strstr(aLine, "UTPP MMTRACE feet "))
        g_feetLines.push_back(aLine);
    else if (std::strstr(aLine, "UTPP MMTRACE footstep "))
        g_footstepLines.push_back(aLine);
    else if (std::strstr(aLine, "UTPP MMTRACE TRANS ") || std::strstr(aLine, "UTPP MMTRACE tick "))
        ++g_mainTraceLines;
    if (std::strstr(aLine, "crouch idle exclusion"))
        g_exclLines.push_back(aLine);
    if (std::strstr(aLine, "speed bands") || std::strstr(aLine, "WARNING") || std::strstr(aLine, "speed calibration"))
        g_infoLines.push_back(aLine);
    if (std::strstr(aLine, "MMTRACE TRANS"))
    {
        const char* fr = std::strstr(aLine, " f=");
        g_transFrame.push_back(fr ? std::atoi(fr + 3) : -1);
        g_transTime.push_back(g_clock);
        {
            const char* w = std::strstr(aLine, " w=");
            const char* v = std::strstr(aLine, " |v|=");
            const char* mv = std::strstr(aLine, " mv=(");
            const char* mvy = mv ? std::strchr(mv + 5, ',') : nullptr;
            g_transW.push_back(w ? float(std::atof(w + 3)) : 0.0f);
            g_transV.push_back(v ? float(std::atof(v + 5)) : 0.0f);
            g_transMv.push_back(mvy ? float(std::atof(mvy + 1)) : 0.0f);
        }
        const char* c = std::strstr(aLine, " clip=");
        const char* k = std::strstr(aLine, " cost=");
        std::string clip;
        if (c)
        {
            c += 6;
            const char* e = std::strchr(c, ' ');
            clip.assign(c, e ? size_t(e - c) : std::strlen(c));
        }
        g_trans.push_back({clip, k ? float(std::atof(k + 6)) : 0.0f});
    }
}

std::string ClassName(const std::string& clip)
{
    using namespace utpp::mm::steer;
    if (clip.find("Idle") != std::string::npos)
        return "Idle";
    const uint8_t c = ClassifyClip(clip);
    if (IsPivot(c))
        return "Pivot";
    if (IsMoveTurn(c))
        return "Turn";
    if (IsStart(c))
        return "Start";
    switch (c)
    {
    case kClassStop: return "Stop";
    case kClassReface: return "Reface";
    case kClassTransition: return "Transition";
    case kClassStanceTransition: return "Stance";
    case kClassTurnInPlace: return "TurnInPlace";
    case kClassArc: return "Arc";
    default: break;
    }
    return clip.find("_Loop") != std::string::npos ? "Loop" : "Other";
}

// 0.5.8: turn direction of a clip by its name (+1 left: "_L_", -1 right: "_R_"; 0 none)
int NameDir(const std::string& clip)
{
    const bool l = clip.find("_L_") != std::string::npos, r = clip.find("_R_") != std::string::npos;
    return l == r ? 0 : (l ? 1 : -1);
}

struct Step
{
    std::string name;
    float seconds;
    float stick;            // |stick| (0..1) at the end of the step
    float stickFrom = -1.0f;// ramp from this magnitude (< 0: constant)
    float wobble = 0.0f;    // +- magnitude, 0.5 Hz (half-stick hand)
    float angleDeg = 0.0f;  // jump of the stick angle relative to the camera at the step start (deg, CCW; flick = 180)
    float angleRate = 0.0f; // deg/s (arcs)
    int zeroFrames = 0;     // frames with the stick at 0 at the step start (flick through the centre)
    int loco = 1;
    // 0.5.4: game-like capsule in large turns (play trace 0.5.2: |v| 3.05-3.86 m/s while turning 120-175 deg at a
    // run) - the velocity direction turns towards the target at kRotateVelDegPerS keeping the speed, instead of braking
    // the cross component through zero (0.5.3 model)
    bool rotateVel = false;
    // 0.5.6 (play 0.5.5, a former test database): PSM block bits fed every frame, single-sample blips (loco 0 +
    // kPsmDetailed with a detail value) every blinkEvery-th frame, world vz, capsule speed set at once (slide), and
    // whether the gate is expected to stay open (weight metrics)
    uint32_t psm = 0;
    int blinkEvery = 0;
    float vz = 0.0f;
    float speed = -1.0f;
    bool expectOpen = false;
    // 0.5.7: capsule held at 0 for the first speedDelay s of the step (game: |v| 0 for ~0.3 s after a side press at 03:30:13.86)
    float speedDelay = 0.0f;
    // 0.6.0: blink with this PSM mask instead of the 0.5.6 detailed-locomotion blip (locoState unchanged)
    uint32_t blinkPsm = 0;
    // loco C1a: deg/s the body turns in place while the stick is released (camera-led turn; 0 = none)
    float bodyRate = 0.0f;
};
constexpr float kRotateVelDegPerS = 360.0f;
// 0.6.3: plant segment [first, last] local frame of each turn-class clip (Section063; empty = metric off)
std::map<std::string, std::pair<int, int>> g_turnSeg;
std::set<std::string> g_turnSegNames; // turn-class clips of the database under test (run-in check)
bool g_game059 = false; // 0.5.9: game-like capsule acceleration / deceleration from the driver targets (Section059)
void (*g_feedProbe)(const FeedInput&, const std::string&) = nullptr; // loco C1b: called after every RunScenario step feed

struct Metrics
{
    float seconds = 0, costMax = 0, corrMax = 0, offSat = 0, offMax = 0, rateMin = 9, rateMax = 0, speedMean = 0;
    int transitions = 0;
    int famChanges = 0;    // transitions between source families (clip name prefix; motion only on the package database)
    float rateLoMoving = 9; // lowest playback rate while the capsule moves >= 0.5 m/s
    int pivotTurnSlow = 0; // pivot / moving-turn clips selected below 0.8 m/s capsule
    // 0.5.4 (play recording)
    int idleMoving = 0;       // idle clips selected while the stick is held and the capsule is faster than 0.5 m/s
    float idleMovingTime = 0; // s an idle clip plays while the stick is held and the capsule is faster than 0.5 m/s
    int tipMoving = 0;        // turn-in-place clips selected while held and capsule > 0.5 m/s
    int walkAtRun = 0;        // walk-class pivots / turns / refaces selected while the intent or the capsule is a run (>= 3 m/s)
    int runAtWalk = 0;        // run / sprint-class pivots / turns / refaces selected in a walk (intent and capsule <= 2.2, capsule >= 0.8)
    int sprintAtRun = 0;      // sprint-class pivots / turns / refaces selected at a run (capsule >= 3, intent and capsule < 5)
    int walkTurns = 0;        // walk-class pivots / turns selected
    int walkAllowedLed = 0;   // frames: animation-led, stick held, walk pivots / turns NOT excluded (status exclude mask)
    int walkLedFrames = 0;    // frames: animation-led with the stick held
    int bandMax = 0;          // highest speed band reported while moving (status band=)
    float errSum = 0, errMax = 0;
    int errN = 0;
    float offLast = 0;     // mean offset root over the last second of the step (steady state)
    // 0.5.6
    float wLowTime = 0;       // s with MM weight < 0.5 in an expectOpen step
    float wMinLast = 9;       // lowest weight over the last 1.5 s of the step
    float closeLatency = -1;  // s from the step start to the first closed gate (status gate != 0); -1 = never
    float zeroLatency = -1;   // s from the step start to weight 0; -1 = never
    int wrongStance = 0;      // stance transitions selected against the current stance (CrouchTo* crouched, *ToCrouch standing)
    int rightStance = 0;      // stance transitions selected in the current stance's direction
    int stanceFast = 0;       // stance transitions selected above 6 m/s capsule
    int lateralFwd = 0;       // lateral (Left / Right / SideStep / Strafe) loops selected while moving forward
    float lateralTime = 0;    // s a lateral loop plays while moving forward with weight > 0.5
    // 0.5.7
    float sideLoopTime = 0;   // s a crouch loop / move with a side / back direction tag or lateral name plays while crouch-walking forward
    int stancePicks = 0;      // selections (TRANS) of stance-transition clips in the step
    int stanceLate = 0;       // ... entered after frame kStanceStartFrames
    int turnRestarts = 0;     // turn-class selections <= 0.4 s after the previous turn-class selection
    int idleStill = 0;        // non-idle / non-stop selections at standstill (|v| < 0.2) without stick or with MM gated out
    float errAt05 = -1;       // |target - body| 0.5 s into the step (deg)
    std::string transList;    // 0.5.7 diagnostics: TRANS sequence of the step
    // 0.5.8
    int capOver = 0;          // selections (TRANS) above the cost cap (100) with weight > 0.5
    float visCostMax = 0;     // highest selection cost with weight > 0.5
    float overSpeedVis = 0;   // s with the simulated capsule above 10.25 m/s and weight > 0.5
    int stanceOver = 0;       // stance-transition selections above the cost cap (any weight)
    float stanceCostMax = 0;  // highest stance-transition selection cost
    int sameDirRestarts = 0;  // turn-class selections <= 0.6 s after a turn-class selection turning the same way (name L / R)
    int firstTurnDir = 0;     // name direction (+1 L / -1 R) of the first turn-class selection of the step
    float firstTurnAt = -1;   // s from the step start
    int offBand = 0;          // loop / speed-transition selections (stick, weight > 0.5) whose speed class is off the capsule's band
    float errAt10 = -1;       // |target - body| 1.0 s into the step (deg)
    int dirFlips = 0;         // start-segment selections in the first 1 s turning against the previous start-segment selection
    int bodyRev = 0;          // body turn direction changes in the first 1 s while |error| > 30 deg
    float offSat24 = 0;       // 0.5.9: s with |offset| >= 24 cm (harness SATURATION_CM)
    // 0.6.0
    float openLatency = -1;   // s from the step start to the first open gate (status gate == 0); -1 = never
    float wUpLatency = -1;    // s from the step start to the first frame with the weight rising above 0.01 after being <= 0.01
    float wMin = 9, wEnd = 0; // lowest / last weight of the step
    int moveToCrouch = 0;     // selections of moving into-crouch transitions (RunToCrouch / WalkToCrouch)
    std::string firstPick;    // first TRANS of the step (clip f=frame c=cost)
    // 0.6.2 (status upperWeight= maskBlend= maskActive=, snapshot g_mmPose)
    float upperMin = 9, upperEnd = 0, maskMax = 0; // upper-body bone weight (weight x (1 - mask)), highest mask blend
    float maskOnLatency = -1;     // s from the step start to mask blend >= 0.99
    float upperZeroLatency = -1;  // s from the step start to upper weight <= 0.01
    float upperBackLatency = -1;  // s from the step start to upper weight >= 0.99 after it was <= 0.01 in this step
    int maskFrames = 0;           // frames with maskActive != 0
    int snapMasked = 0;           // published snapshots with upperScale <= 0 (upper body fully on the game pose)
    int snapUpperLeak = 0;        // ... of them writing any upper-body bone
    int snapLowerMissing = 0;     // ... of them writing no lower-body bone
    std::string endClip;          // clip playing at the last frame of the step
    // 0.6.3 turn rate: frames of a turn-class clip inside its plant segment (g_turnSeg: root below 65 % of the clip's peak root
    // speed), stick held, capsule >= 1 m/s
    int turnSegFrames = 0, turnSegLow = 0; // ... and of them at a playback rate < 0.9
    float turnSegRateMin = 9, turnSegRateSum = 0, turnSegCorrMax = 0, turnSegOffMax = 0;
    float TurnSegRateMean() const { return turnSegFrames > 0 ? turnSegRateSum / float(turnSegFrames) : 0.0f; }
    std::map<std::string, int> classes;
    std::vector<std::string> clips;
    std::string ClassesString() const
    {
        std::string s;
        for (const auto& kv : classes)
            s += kv.first + ":" + std::to_string(kv.second) + " ";
        return s;
    }
    std::string Line() const
    {
        char buf[256];
        std::snprintf(buf, sizeof buf, "trans/s %.2f cost max %.2f footlock max %.1f cm offset sat %.2f s (max %.1f cm) rate %.2f..%.2f |v| %.2f | %s",
                      transitions / std::max(seconds, 1e-3f), costMax, corrMax, offSat, offMax, rateMin, rateMax, speedMean,
                      ClassesString().c_str());
        return buf;
    }
};

struct Sim
{
    float camYaw = -82.0f, bodyYaw = 0.0f, vx = 0.0f, vy = 0.0f;
    float t = 0.0f;
};

float SpeedOfMove(float m)
{
    static const float mv[] = {0.0f, 0.15f, 0.18f, 0.36f, 0.82f, 1.0f};
    static const float sp[] = {0.0f, 0.25f, 0.30f, 0.59f, 2.50f, 3.85f};
    if (m <= 0.0f)
        return 0.0f;
    for (int i = 1; i < 6; ++i)
        if (m <= mv[i])
            return sp[i - 1] + (m - mv[i - 1]) * (sp[i] - sp[i - 1]) / (mv[i] - mv[i - 1]);
    return sp[5];
}

// 0.6.2: the published snapshot applied to synthetic bone records (sentinel pose): with upperScale <= 0 no upper-body bone may
// change and at least one lower-body bone must be written.
void SnapshotCheck(Metrics& m)
{
    using namespace utpp::mm::integ;
    static PoseSnapshot snap{};
    if (!utpp::g_mmPose.Read(snap) || !(snap.weight > 0.0f) || !(snap.upperScale <= 0.0f))
        return;
    static float rec[kMaxMMBones * 12];
    for (uint32_t i = 0; i < kMaxMMBones; ++i)
    {
        float* r = &rec[size_t(i) * 12];
        const float init[12] = {9.0f, 9.0f, 9.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        std::memcpy(r, init, sizeof init);
    }
    BlendIntoRecords(reinterpret_cast<uint8_t*>(rec), kMaxMMBones, snap, snap.weight);
    bool upperTouched = false;
    int lowerWritten = 0;
    for (uint32_t b = snap.firstBone; b < snap.numBones && b < kMaxMMBones; ++b)
    {
        const int32_t g = snap.gameIndex[b];
        if (g < 0 || uint32_t(g) >= kMaxMMBones)
            continue;
        const float* r = &rec[size_t(g) * 12];
        const bool changed = r[0] != 9.0f || r[1] != 9.0f || r[2] != 9.0f || r[4] != 0.0f || r[5] != 0.0f || r[6] != 0.0f || r[7] != 1.0f;
        if (SnapshotUpperBone(snap, b))
            upperTouched = upperTouched || changed;
        else if (changed)
            ++lowerWritten;
    }
    ++m.snapMasked;
    if (upperTouched)
        ++m.snapUpperLeak;
    if (lowerWritten == 0)
        ++m.snapLowerMissing;
}

std::map<std::string, Metrics> RunScenario(const std::vector<Step>& aSteps, float aBodyYaw, bool aFresh = false)
{
    std::map<std::string, Metrics> out;
    Sim s;
    s.bodyYaw = aBodyYaw;
    const float dt = 1.0f / 60.0f;
    // 2026-10-07 (clip curation): aFresh = fresh matcher state (idle start) before the scenario. The 180 deg start
    // (turnstart) used to begin from the end state of the previous scenario (run flick), so a change of the
    // family-rate factors there flipped its motion-only path between two branches (offset saturation 0.30 / 0.62 s);
    // from a fresh state old and new factors give the same path (0.30 s, cost 13.61) - settle sweep in
    // a settle sweep log. Only turnstart uses it (a start from standstill by definition);
    // the other scenarios keep their sequence (their thresholds were set on it).
    if (aFresh)
        SetEnabled(true);
    // settle: idle
    FeedInput idle;
    idle.tppActive = true;
    idle.locoState = 1;
    idle.dt = dt;
    idle.cameraYawDeg = s.camYaw;
    idle.playerYawDeg = s.bodyYaw;
    for (int i = 0; i < 90; ++i)
        Feed(idle);
    float angle = 0.0f;
    int bodyDir = 0;       // 0.5.8 body turn direction / sustained angle (across steps; reset when the stick is released)
    float bodyAcc = 0.0f;
    int lastSegDir = 0;    // 0.5.8 last start-segment selection direction / clock (across steps)
    float lastSegClock = -1e6f;
    for (const Step& st : aSteps)
    {
        if (st.stick <= 0.0f)
        {
            bodyDir = 0;
            bodyAcc = 0.0f;
            lastSegDir = 0;
        }
        Metrics& m = out[st.name];
        angle += st.angleDeg;
        const int frames = int(st.seconds / dt + 0.5f);
        double speedSum = 0;
        std::string prevClip = ClipOf(Status());
        const size_t transBefore = g_trans.size();
        const float stepClock = g_clock; // 0.5.8
        float prevBody = s.bodyYaw;
        for (int i = 0; i < frames; ++i)
        {
            s.t += dt;
            const float u = frames > 1 ? float(i) / float(frames - 1) : 1.0f;
            float stick = st.stickFrom >= 0.0f ? st.stickFrom + (st.stick - st.stickFrom) * u : st.stick;
            stick += st.wobble * std::sin(6.2831853f * 0.5f * s.t);
            if (i < st.zeroFrames)
                stick = 0.0f;
            angle += st.angleRate * dt;
            const float mvMag = std::clamp((stick - 0.3f) / 0.7f, 0.0f, 1.0f);
            const bool moving = mvMag > 0.0f;
            const float target = WrapDeg(s.camYaw + angle);
            // capsule
            const float want = SpeedOfMove(mvMag) * (st.loco == 2 ? 0.7f : (st.loco == 3 ? 8.25f / 3.85f : 1.0f));
            const bool snap = st.speed >= 0.0f || float(i) * dt < st.speedDelay; // 0.5.6 slide; 0.5.7 delayed capsule start
            const float snapSpeed = st.speed >= 0.0f ? st.speed : 0.0f;
            // game-like capsule (traces: speeding up ~3 m/s^2, braking / turning ~11 m/s^2): the velocity component across
            // the target direction and any excess along it brake at 11 m/s^2, the component along it grows at 3.2 m/s^2
            float accUp = 3.2f, brake = 11.0f;
            if (g_game059) // 0.5.9 game model: vanilla 24 / 20 m/s^2 lowered by the driver targets of the previous feed
            {
                const float at = AccelTarget(24.0f, 1.0f);
                const float dtg = DecelTarget(20.0f, 1.0f);
                accUp = at > 0.0f ? std::fmin(at, 24.0f) : 24.0f;
                brake = dtg > 0.0f ? std::fmin(dtg, 20.0f) : 20.0f;
            }
            if (snap)
            {
                s.vx = -std::sin(target * kDegT) * snapSpeed;
                s.vy = std::cos(target * kDegT) * snapSpeed;
            }
            else if (st.rotateVel)
            {
                float sp = std::sqrt(s.vx * s.vx + s.vy * s.vy);
                float head = sp > 1e-4f ? std::atan2(-s.vx, s.vy) / kDegT : target;
                head += std::clamp(WrapDeg(target - head), -kRotateVelDegPerS * dt, kRotateVelDegPerS * dt);
                sp = sp > want ? std::fmax(want, sp - brake * dt) : std::fmin(want, sp + accUp * dt);
                s.vx = -std::sin(head * kDegT) * sp;
                s.vy = std::cos(head * kDegT) * sp;
            }
            else
            {
                const float dx = -std::sin(target * kDegT), dy = std::cos(target * kDegT);
                float along = s.vx * dx + s.vy * dy;
                float px = s.vx - along * dx, py = s.vy - along * dy;
                const float pl = std::sqrt(px * px + py * py);
                const float pk = pl > 1e-6f ? std::fmax(0.0f, pl - brake * dt) / pl : 0.0f;
                px *= pk;
                py *= pk;
                if (along < 0.0f || along > want)
                    along = along < 0.0f ? std::fmin(0.0f, along + brake * dt) : std::fmax(want, along - brake * dt);
                else
                    along = std::fmin(want, along + accUp * dt);
                s.vx = px + along * dx;
                s.vy = py + along * dy;
            }
            // base body turn towards the movement target, replaced by UTPP's command while active
            float baseYaw = s.bodyYaw;
            if (moving)
            {
                const float e = WrapDeg(target - s.bodyYaw);
                baseYaw = s.bodyYaw + std::clamp(e, -540.0f * dt, 540.0f * dt);
            }
            else if (st.bodyRate != 0.0f) // loco C1a: turn in place with the stick released
                baseYaw = s.bodyYaw + st.bodyRate * dt;
            s.bodyYaw = WrapDeg(BodyYawCommand(s.bodyYaw, baseYaw));
            FeedInput in;
            in.velWorld[0] = s.vx;
            in.velWorld[1] = s.vy;
            in.move[1] = mvMag;
            in.cameraYawDeg = s.camYaw;
            in.playerYawDeg = s.bodyYaw;
            in.locoState = st.loco;
            in.tppActive = true;
            in.dt = dt;
            in.haveTarget = moving;
            in.targetYawDeg = target;
            in.velWorld[2] = st.vz;
            in.psmBlock = st.psm;
            if (st.blinkEvery > 0 && i % st.blinkEvery == st.blinkEvery - 1 && st.blinkPsm != 0)
                in.psmBlock = st.blinkPsm; // 0.6.0: single-sample PSM mask (e.g. a melee state blip)
            else if (st.blinkEvery > 0 && i % st.blinkEvery == st.blinkEvery - 1)
            {
                in.locoState = 0;                        // state name not stand / crouch / sprint for one sample
                in.psmBlock = 0x40u | ((5u + 1u) << 16); // kPsmDetailed + reds 0.5.6 detail field (5 = hypothetical)
            }
            g_clock += dt;
            Feed(in);
            const std::string status = Status();
            if (g_feedProbe) // loco C1b: per-feed measurements of a section (null elsewhere)
                g_feedProbe(in, status);
            const std::string clip = ClipOf(status);
            if (clip != prevClip && !clip.empty())
            {
                ++m.transitions;
                if (!prevClip.empty() && utpp::mm::ClipFamily(prevClip) != utpp::mm::ClipFamily(clip))
                    ++m.famChanges;
                const std::string cn = ClassName(clip);
                m.classes[cn]++;
                const float capNow = std::sqrt(s.vx * s.vx + s.vy * s.vy);
                if ((cn == "Pivot" || cn == "Turn") && capNow < 0.8f)
                    ++m.pivotTurnSlow;
                if (cn == "Idle" && moving && capNow > 0.5f)
                    ++m.idleMoving;
                if (cn == "TurnInPlace" && moving && capNow > 0.5f)
                    ++m.tipMoving;
                if ((Field(status, "stickHeld") < 0.5f || HexField(status, "gate") != 0u) && capNow < 0.2f && cn != "Idle" &&
                    cn != "Stop") // 0.5.7 (driver stick state: a release is bridged for 0.12 s)
                    ++m.idleStill;
                if (cn == "Pivot" || cn == "Turn" || (cn == "Reface" && capNow >= 1.0f)) // reface: moving only (starts: any class)
                {
                    const int sc = utpp::mm::steer::SpeedClassOfName(clip);
                    if (sc == 1 && cn != "Reface")
                        ++m.walkTurns;
                    if (sc == 1 && moving && std::fmax(want, capNow) >= 3.0f)
                        ++m.walkAtRun;
                    if (sc >= 2 && moving && want <= 2.2f && capNow <= 2.2f && capNow >= 0.8f)
                        ++m.runAtWalk;
                    if (sc == 3 && moving && capNow >= 3.0f && want < 5.0f && capNow < 5.0f)
                        ++m.sprintAtRun;
                }
                if (cn == "Stance") // 0.5.6
                {
                    const int dir = utpp::mm::steer::StanceDirOfName(clip);
                    const int stance = st.loco == 2 ? 2 : ((st.loco == 1 || st.loco == 3) ? 1 : 0);
                    if ((dir > 0 && stance == 1) || (dir < 0 && stance == 2))
                        ++m.wrongStance;
                    else if (dir != 0 && stance != 0)
                        ++m.rightStance;
                    if (capNow > 6.0f)
                        ++m.stanceFast;
                }
                if (utpp::mm::steer::DriverClipClass(clip, utpp::mm::steer::ClassifyClip(clip)) == utpp::mm::steer::kClassLateral &&
                    moving && std::fabs(WrapDeg(target - s.bodyYaw)) < 30.0f)
                    ++m.lateralFwd;
                if (std::find(m.clips.begin(), m.clips.end(), clip) == m.clips.end())
                    m.clips.push_back(clip);
                {
                    // 0.5.8: loop / speed transition off the capsule's speed band - the database's band edges (status bandEdges=,
                    // database m/s) with the capsule mapped through the stand speed map; +-10 % around an edge ignored (hysteresis):
                    // two bands off at any time, one band off at a steady speed (|capsule - wanted| < 0.3 m/s)
                    using namespace utpp::mm::steer;
                    const int lb = LoopBandOfName(clip, ClassifyClip(clip));
                    const float wNow = Field(status, "weight");
                    const size_t be = status.find(" bandEdges=");
                    float eWR = 0.0f, eRS = 0.0f;
                    if (be != std::string::npos)
                    {
                        eWR = float(std::atof(status.c_str() + be + 11));
                        const size_t sl = status.find('/', be);
                        eRS = sl != std::string::npos ? float(std::atof(status.c_str() + sl + 1)) : 0.0f;
                    }
                    float gm[3] = {}, dbm[3] = {};
                    int32_t gc = 0;
                    GetSpeedMap(1, gm, dbm, gc);
                    utpp::mm::SpeedPoints sp{gc, {gm[0], gm[1], gm[2]}, {dbm[0], dbm[1], dbm[2]}};
                    const float vdb = utpp::mm::MapSpeedPoints(sp, capNow);
                    const bool nearEdge = std::fabs(vdb - eWR) < 0.1f * eWR || std::fabs(vdb - eRS) < 0.1f * eRS;
                    if (lb > 0 && moving && wNow > 0.5f && eWR > 0.0f && !nearEdge)
                    {
                        const int vb = vdb < eWR ? 1 : (vdb < eRS ? 2 : 3);
                        int avail[4] = {1, 1, 1, 1}; // status loopAvail=w/r/s: a band without loops falls back to the slower class
                        const size_t la = status.find(" loopAvail=w");
                        if (la != std::string::npos)
                        {
                            const char* q = status.c_str() + la + 12;
                            avail[1] = std::atoi(q);
                            const char* qr = std::strstr(q, "/r");
                            const char* qs = std::strstr(q, "/s");
                            avail[2] = qr ? std::atoi(qr + 2) : 1;
                            avail[3] = qs ? std::atoi(qs + 2) : 1;
                        }
                        const bool fallback = ClassifyClip(clip) != kClassTransition && lb == vb - 1 && avail[vb] == 0;
                        if (!fallback && (std::abs(lb - vb) >= 2 || (lb != vb && std::fabs(capNow - want) < 0.3f)))
                            ++m.offBand;
                    }
                }
            }
            {
                // 0.5.6 gate / weight metrics
                const float w = Field(status, "weight");
                const float tNow = float(i) * dt;
                if (st.expectOpen && w < 0.5f)
                    m.wLowTime += dt;
                if (i >= frames - int(1.5f / dt))
                    m.wMinLast = std::min(m.wMinLast, w);
                if (m.closeLatency < 0.0f && HexField(status, "gate") != 0u)
                    m.closeLatency = tNow;
                if (m.zeroLatency < 0.0f && w <= 0.0f)
                    m.zeroLatency = tNow;
                // 0.6.0
                if (m.openLatency < 0.0f && HexField(status, "gate") == 0u)
                    m.openLatency = tNow;
                if (m.wUpLatency < 0.0f && w > 0.01f && m.wMin <= 0.01f)
                    m.wUpLatency = tNow;
                m.wMin = std::min(m.wMin, w);
                m.wEnd = w;
                // 0.6.2
                {
                    const float uw = Field(status, "upperWeight");
                    const float mb = Field(status, "maskBlend");
                    m.upperMin = std::min(m.upperMin, uw);
                    m.upperEnd = uw;
                    m.maskMax = std::max(m.maskMax, mb);
                    if (m.maskOnLatency < 0.0f && mb >= 0.99f)
                        m.maskOnLatency = tNow;
                    if (m.upperZeroLatency < 0.0f && uw <= 0.01f)
                        m.upperZeroLatency = tNow;
                    if (m.upperBackLatency < 0.0f && m.upperMin <= 0.01f && uw >= 0.99f)
                        m.upperBackLatency = tNow;
                    if (HexField(status, "maskActive") != 0u)
                        ++m.maskFrames;
                    SnapshotCheck(m);
                    m.endClip = clip;
                }
                if (moving && w > 0.5f && std::fabs(WrapDeg(target - s.bodyYaw)) < 30.0f &&
                    utpp::mm::steer::DriverClipClass(clip, utpp::mm::steer::ClassifyClip(clip)) == utpp::mm::steer::kClassLateral)
                    m.lateralTime += dt;
                // 0.5.7
                if (moving && w > 0.5f && st.loco == 2 && std::fabs(WrapDeg(target - s.bodyYaw)) < 30.0f &&
                    utpp::mm::steer::DriverClipClass(clip, utpp::mm::steer::ClassifyClip(clip), true) == utpp::mm::steer::kClassLateral)
                    m.sideLoopTime += dt;
                if (i == int(0.5f / dt + 0.5f))
                    m.errAt05 = std::fabs(WrapDeg(target - s.bodyYaw));
                // 0.5.8
                if (i == int(1.0f / dt + 0.5f))
                    m.errAt10 = std::fabs(WrapDeg(target - s.bodyYaw));
                if (std::sqrt(s.vx * s.vx + s.vy * s.vy) > 10.25f && w > 0.5f)
                    m.overSpeedVis += dt;
                {
                    const float d = WrapDeg(s.bodyYaw - prevBody);
                    const int sg = d > 0.3f ? 1 : (d < -0.3f ? -1 : 0);
                    if (sg != 0 && std::fabs(WrapDeg(target - s.bodyYaw)) > 30.0f)
                    {
                        if (bodyDir != 0 && sg != bodyDir)
                        {
                            if (bodyAcc >= 15.0f)
                                ++m.bodyRev; // a sustained turn (>= 15 deg) reversed
                            bodyAcc = 0.0f;
                        }
                        bodyDir = sg;
                        bodyAcc += std::fabs(d);
                    }
                    prevBody = s.bodyYaw;
                }
            }
            {
                // optional frame dump of one step: set UTPP_LOCO_DEBUG=<step name>
                static char dbg[64] = {};
                static bool dbgRead = false;
                if (!dbgRead)
                {
                    dbgRead = true;
                    GetEnvironmentVariableA("UTPP_LOCO_DEBUG", dbg, sizeof dbg);
                }
                if (dbg[0] && st.name == dbg && (i % 6 == 0 || clip != prevClip))
                    std::printf("  dbg t=%.2f tgt=%.1f body=%.1f |v|=%.2f mv=%.2f clip=%s f=%.0f btActive=%.0f followFed=%.0f animLed=%.0f "
                                "haveTarget=%.0f w=%.2f rate=%.2f corr=%.1f/%.1f cost=%.2f excl=0x%X off=%.1f/%.1f set=%.0f\n",
                                s.t, target, s.bodyYaw, std::sqrt(s.vx * s.vx + s.vy * s.vy), mvMag, clip.c_str(), Field(status, "clipFrame"),
                                Field(status, "btActive"), Field(status, "followFed"), Field(status, "animLed"), Field(status, "haveTarget"),
                                Field(status, "weight"), Field(status, "rate"), Field(status, "corrL"), Field(status, "corrR"),
                                g_trans.empty() ? 0.0 : double(g_trans.back().second), HexField(status, "exclude"), Field(status, "offX"),
                                Field(status, "offY"), Field(status, "classSet"));
            }
            prevClip = clip;
            {
                const float capNow = std::sqrt(s.vx * s.vx + s.vy * s.vy);
                if (moving && capNow > 0.5f && ClassName(clip) == "Idle")
                    m.idleMovingTime += dt;
                if (moving && Field(status, "animLed") > 0.5f)
                {
                    using namespace utpp::mm::steer;
                    ++m.walkLedFrames;
                    if ((HexField(status, "exclude") & (Bit(kClassPivotWalk) | Bit(kClassTurnWalk))) == 0)
                        ++m.walkAllowedLed;
                }
                if (moving)
                {
                    m.bandMax = std::max(m.bandMax, int(Field(status, "band") + 0.5f));
                    const float er = std::fabs(WrapDeg(target - s.bodyYaw));
                    m.errSum += er;
                    m.errMax = std::max(m.errMax, er);
                    ++m.errN;
                }
            }
            m.corrMax = std::max({m.corrMax, Field(status, "corrL"), Field(status, "corrR")});
            const float ox = Field(status, "offX"), oy = Field(status, "offY");
            const float off = std::sqrt(ox * ox + oy * oy);
            if (!g_turnSeg.empty() && moving && std::sqrt(s.vx * s.vx + s.vy * s.vy) >= 1.0f) // 0.6.3
            {
                const auto it = g_turnSeg.find(clip);
                const int cf = int(Field(status, "clipFrame"));
                if (it != g_turnSeg.end() && cf >= it->second.first && cf <= it->second.second)
                {
                    const float r = Field(status, "rate");
                    ++m.turnSegFrames;
                    if (r < 0.9f)
                        ++m.turnSegLow;
                    m.turnSegRateMin = std::min(m.turnSegRateMin, r);
                    m.turnSegRateSum += r;
                    m.turnSegCorrMax = std::max({m.turnSegCorrMax, Field(status, "corrL"), Field(status, "corrR")});
                    m.turnSegOffMax = std::max(m.turnSegOffMax, off);
                }
            }
            m.offMax = std::max(m.offMax, off);
            if (off >= 24.9f)
                m.offSat += dt;
            if (off >= 24.0f)
                m.offSat24 += dt;
            if (i >= frames - 60)
                m.offLast += off / 60.0f;
            const float rate = Field(status, "rate");
            if (std::sqrt(s.vx * s.vx + s.vy * s.vy) >= 0.5f)
                m.rateLoMoving = std::min(m.rateLoMoving, rate);
            m.rateMin = std::min(m.rateMin, rate);
            m.rateMax = std::max(m.rateMax, rate);
            speedSum += std::sqrt(s.vx * s.vx + s.vy * s.vy);
        }
        for (size_t k = transBefore; k < g_trans.size(); ++k)
        {
            m.costMax = std::max(m.costMax, g_trans[k].second);
            // 0.5.7: stance picks / late entries, turn-class restarts (TRANS lines incl. jumps inside the same clip)
            using namespace utpp::mm::steer;
            const std::string& cl = g_trans[k].first;
            const uint8_t cc = DriverClipClass(cl, ClassifyClip(cl));
            if (IsStanceClass(cc))
            {
                ++m.stancePicks;
                if (k < g_transFrame.size() && g_transFrame[k] > kStanceStartFrames)
                    ++m.stanceLate;
            }
            if (IsTurnClass(cc) && k > 0 && k < g_transTime.size())
            {
                const std::string& pc = g_trans[k - 1].first;
                if (IsTurnClass(DriverClipClass(pc, ClassifyClip(pc))) && g_transTime[k] - g_transTime[k - 1] <= 0.4f)
                    ++m.turnRestarts;
            }
            // 0.5.8
            {
                const float wk = k < g_transW.size() ? g_transW[k] : 0.0f;
                const float vk = k < g_transV.size() ? g_transV[k] : 0.0f;
                const float ck = g_trans[k].second;
                const float tk = k < g_transTime.size() ? g_transTime[k] - stepClock : 0.0f;
                if (wk > 0.5f)
                {
                    m.visCostMax = std::max(m.visCostMax, ck);
                    if (ck > 100.0f)
                        ++m.capOver;
                }
                if (IsStanceClass(cc))
                {
                    m.stanceCostMax = std::max(m.stanceCostMax, ck);
                    if (ck > 100.0f)
                        ++m.stanceOver;
                }
                const int nd = NameDir(cl);
                if (IsTurnClass(cc) && nd != 0)
                {
                    if (m.firstTurnDir == 0)
                    {
                        m.firstTurnDir = nd;
                        m.firstTurnAt = tk;
                    }
                    if (k > 0 && k < g_transTime.size())
                    {
                        const std::string& pc = g_trans[k - 1].first;
                        if (IsTurnClass(DriverClipClass(pc, ClassifyClip(pc))) && NameDir(pc) == nd && g_transTime[k] - g_transTime[k - 1] <= 0.6f)
                            ++m.sameDirRestarts;
                    }
                }
                if (IsStartSegmentClass(cc) && nd != 0 && k < g_transTime.size())
                {
                    if (lastSegDir != 0 && nd != lastSegDir && g_transTime[k] - lastSegClock <= 1.0f)
                        ++m.dirFlips;
                    lastSegDir = nd;
                    lastSegClock = g_transTime[k];
                }
                (void)vk;
            }
            if (utpp::mm::steer::IsMovingToCrouchName(cl)) // 0.6.0
                ++m.moveToCrouch;
            if (m.firstPick.empty())
            {
                char fb[160];
                std::snprintf(fb, sizeof fb, "%s f=%d c=%.1f", cl.c_str(), k < g_transFrame.size() ? g_transFrame[k] : -1, double(g_trans[k].second));
                m.firstPick = fb;
            }
            if (m.transList.size() < 900)
            {
                char tb[160];
                std::snprintf(tb, sizeof tb, "%s f=%d c=%.1f; ", cl.c_str(), k < g_transFrame.size() ? g_transFrame[k] : -1, double(g_trans[k].second));
                m.transList += tb;
            }
        }
        m.seconds = st.seconds;
        m.speedMean = float(speedSum / std::max(frames, 1));
    }
    return out;
}

// 0.5.7 switches; diagnostic UTPP_LOCO_057=<bits> limits "on" to a subset (1 crouch dir tag, 2 stance once, 4 turn commit,
// 8 standstill idle, 16 start turn; default 31)
void Set057(bool aOn)
{
    char env[8] = {};
    const int bits = GetEnvironmentVariableA("UTPP_LOCO_057", env, sizeof env) > 0 ? std::atoi(env) : 31;
    SetCrouchDirTag(aOn && (bits & 1));
    SetStanceOnce(aOn && (bits & 2));
    SetTurnCommit(aOn && (bits & 4));
    SetStandIdle(aOn && (bits & 8));
    SetStartTurn(aOn && (bits & 16));
}

// 0.5.8 switches; diagnostic UTPP_LOCO_058=<bits> limits "on" to a subset (1 speed cap, 2 cost cap, 4 stance cap, 8 turn settle,
// 16 loop bands, 32 start bind; default 63)
void Set058(bool aOn)
{
    char env[8] = {};
    const int bits = GetEnvironmentVariableA("UTPP_LOCO_058", env, sizeof env) > 0 ? std::atoi(env) : 63;
    SetSpeedCap(aOn && (bits & 1) ? 1.25f : 0.0f);
    SetCostCap(aOn && (bits & 2) ? 100.0f : 0.0f);
    SetStanceCap(aOn && (bits & 4));
    SetTurnSettle(aOn && (bits & 8));
    SetLoopBands(aOn && (bits & 16));
    SetStartBind(aOn && (bits & 32));
}

// 0.6.0 switches (melee gate, mount cut, stance clip minimum speed 4.0 m/s); off = 0.5.9
void Set060(bool aOn)
{
    SetMeleeGate(aOn);
    SetMountCut(aOn);
    SetStanceClipMinSpeed(aOn ? 4.0f : 0.0f);
}

// 0.6.3 diagnostic tuning: UTPP_LOCO_TURNRATE_PARAMS=<min>,<max> (shipped default 0.9,1.1; the CET setting raises max to 1.4 in game)
void ApplyTurnRateEnv()
{
    char e[64] = {};
    float a = 0.0f, b = 0.0f;
    if (GetEnvironmentVariableA("UTPP_LOCO_TURNRATE_PARAMS", e, sizeof e) > 0 && sscanf_s(e, "%f,%f", &a, &b) == 2 && !SetTurnRateParams(a, b))
        std::printf("[INFO] 0.6.3 turn rate params %s rejected\n", e);
}

void Configure(bool aFixes)
{
    SetStickFilter(aFixes);
    SetFamilyRate(aFixes);
    SetStartRules(aFixes);
    SetSpeedBands(aFixes); // 0.5.4: the 0.5.3 suite runs with every fix (= current state) against 0.5.2
}

// 0.5.4 modes: 0 = 0.5.2 (all off), 1 = 0.5.3 (= current with the speed bands off), 2 = 0.5.4 (= current: every fix on).
// 0.5.6: the 0.5.6 switches follow "every other fix" (off in mode 0, on in modes 1 and 2), so mode 1 vs 2 still isolates
// the speed bands; the chained per-database scenarios start from the end state of the crouch scenario, which the crouch
// rules change. Diagnostic: UTPP_LOCO_056=0 keeps the 0.5.6 switches off in modes 1 and 2 (= the 0.5.5 suite).
void ConfigureMode(int aMode)
{
    Configure(aMode > 0);
    SetSpeedBands(aMode >= 2);
    char env[8] = {};
    const bool off056 = GetEnvironmentVariableA("UTPP_LOCO_056", env, sizeof env) > 0 && env[0] == '0';
    SetCrouchRules(aMode >= 1 && !off056);
    SetGateDebounce(aMode >= 1 && !off056 ? 0.06f : 0.0f);
    Set057(aMode >= 1); // 0.5.7: like the 0.5.6 switches (off in the 0.5.2 mode)
    Set058(aMode >= 1); // 0.5.8: likewise
    // 0.5.9: the older sections keep their 0.5.8 numbers (the 0.5.9 switches are compared in Section059)
    SetCapsuleSync(false);
    SetOffsetCatchup(false);
    SetTrajCal(false);
    Set060(false); // 0.6.0: likewise (compared in Section060)
    SetUpperMask(false); // 0.6.2: likewise (compared in Section062)
    SetUpperMaskStates(7);
    SetHoldPose(false);
    // 0.6.3: likewise (compared in Section063); diagnostic UTPP_LOCO_TURNRATE_ALL=1 keeps it on in modes 1 and 2
    char e63[4] = {};
    const bool tr63 = GetEnvironmentVariableA("UTPP_LOCO_TURNRATE_ALL", e63, sizeof e63) > 0 && e63[0] == '1';
    SetTurnRate(tr63 && aMode >= 1);
    ApplyTurnRateEnv();
}
// 0.5.6: current state with the 0.5.6 switches off = 0.5.5
void Configure055(bool a056)
{
    ConfigureMode(2);
    SetCrouchRules(a056);
    SetGateDebounce(a056 ? 0.06f : 0.0f);
}
const char* kModeName[3] = {"0.5.2", "0.5.3", "0.5.4"};

Metrics Total(const std::map<std::string, Metrics>& aSteps)
{
    Metrics t;
    t.rateMin = 9;
    for (const auto& kv : aSteps)
    {
        const Metrics& m = kv.second;
        t.seconds += m.seconds;
        t.costMax = std::max(t.costMax, m.costMax);
        t.corrMax = std::max(t.corrMax, m.corrMax);
        t.offSat += m.offSat;
        t.offMax = std::max(t.offMax, m.offMax);
        t.rateMin = std::min(t.rateMin, m.rateMin);
        t.rateMax = std::max(t.rateMax, m.rateMax);
        t.transitions += m.transitions;
        t.pivotTurnSlow += m.pivotTurnSlow;
        t.idleMoving += m.idleMoving;
        t.idleMovingTime += m.idleMovingTime;
        t.tipMoving += m.tipMoving;
        t.walkAtRun += m.walkAtRun;
        t.runAtWalk += m.runAtWalk;
        t.sprintAtRun += m.sprintAtRun;
        t.walkTurns += m.walkTurns;
        t.walkAllowedLed += m.walkAllowedLed;
        t.walkLedFrames += m.walkLedFrames;
        t.bandMax = std::max(t.bandMax, m.bandMax);
        t.errSum += m.errSum;
        t.errN += m.errN;
        t.errMax = std::max(t.errMax, m.errMax);
        t.wLowTime += m.wLowTime;
        t.wrongStance += m.wrongStance;
        t.rightStance += m.rightStance;
        t.stanceFast += m.stanceFast;
        t.lateralFwd += m.lateralFwd;
        t.lateralTime += m.lateralTime;
        t.sideLoopTime += m.sideLoopTime; // 0.5.7
        t.stancePicks += m.stancePicks;
        t.stanceLate += m.stanceLate;
        t.turnRestarts += m.turnRestarts;
        t.idleStill += m.idleStill;
        t.capOver += m.capOver; // 0.5.8
        t.visCostMax = std::max(t.visCostMax, m.visCostMax);
        t.overSpeedVis += m.overSpeedVis;
        t.stanceOver += m.stanceOver;
        t.stanceCostMax = std::max(t.stanceCostMax, m.stanceCostMax);
        t.sameDirRestarts += m.sameDirRestarts;
        t.offBand += m.offBand;
        t.dirFlips += m.dirFlips;
        t.bodyRev += m.bodyRev;
        t.speedMean += m.speedMean * m.seconds;
        for (const auto& c : m.classes)
            t.classes[c.first] += c.second;
    }
    if (t.seconds > 0)
        t.speedMean /= t.seconds;
    return t;
}

std::string Line056(const Metrics& m)
{
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "wLow %.2f s wMinLast %.2f close %.3f s zero %.3f s wrongStance %d rightStance %d stanceFast %d lateralFwd %d (%.2f s) | %s",
                  m.wLowTime, m.wMinLast, m.closeLatency, m.zeroLatency, m.wrongStance, m.rightStance, m.stanceFast, m.lateralFwd,
                  m.lateralTime, m.Line().c_str());
    return buf;
}

std::string Line057(const Metrics& m)
{
    char buf[640];
    std::snprintf(buf, sizeof buf,
                  "sideLoop %.2f s stancePicks %d late %d turnRestarts %d idleStill %d err@0.5 %.1f costMax %.1f offSat %.2f trans %d | %s",
                  m.sideLoopTime, m.stancePicks, m.stanceLate, m.turnRestarts, m.idleStill, m.errAt05, m.costMax, m.offSat, m.transitions,
                  m.ClassesString().c_str());
    return buf;
}

bool HasClipContaining(const char* aPath, const char* aText)
{
    utpp::mm::Database d;
    std::string e;
    if (!d.LoadFromFile(aPath, kExpectedRigHash, e))
        return false;
    for (uint32_t c = 0; c < d.NumClips(); ++c)
        if (d.ClipName(c).find(aText) != std::string_view::npos)
            return true;
    return false;
}

// b5 testorder: every Section* starts with BeginSection (marker line "==== SECTION <name>" for the order comparison:
// a section run alone through UTPP_LOCO_ONLY* must print the same [INFO] / [PASS] lines as inside the full suite)
void BeginSection(const char* aName); // defined before main

// Parallel ctest shards (utpp_mm_loco_test_shard* in CMakeLists.txt): "--units=a,b,c" (anywhere after the executable name) runs only
// the named units of the full suite, in the suite's own order; no option = every unit (the single-process full run). Units: main
// (the inline 0.5.3 / 0.5.4 / 0.5.5 / 0.5.6 scenarios of main()), 057 058 059 060 062 063 c0c1a c1b idle knee strafe air airclips
// airsteer vehexit. Every Section* starts from BeginSection (same base settings, ResetSessionState, test-side state cleared; the
// testorder check compares the settings with the first section) and every measured run starts with LoadDatabase (a fresh session),
// so a unit alone prints the same PASS / FAIL / INFO lines as in the full run (wall-time numbers aside; checked by diffing the
// lines of the shards against the single-process run). Exceptions: "airclips" reads the toe bone indices that SectionAir sets
// (always together with "air"), and SectionAir keeps FeedCalls() parity for the later sections.
// "knee" = the whole knee section; "knee.<cell>" = one cell of its (sub-suite x family group) grid, see SectionKnee.
// UTPP_LOCO_TIMING=1 prints the wall time of every unit that ran ([TIME] lines, for balancing the shards).
std::set<std::string> g_units;
bool g_unitFilter = false;
bool UnitOn(const char* aName)
{
    if (!g_unitFilter || g_units.count(aName) > 0)
        return true;
    const std::string pfx = std::string(aName) + ".";
    for (const std::string& u : g_units)
        if (u.compare(0, pfx.size(), pfx) == 0)
            return true;
    return false;
}
bool SubUnitOn(const char* aUnit, const char* aCell)
{
    return !g_unitFilter || g_units.count(aUnit) > 0 || g_units.count(std::string(aUnit) + "." + aCell) > 0;
}
bool TimingOn()
{
    char et[4] = {};
    return GetEnvironmentVariableA("UTPP_LOCO_TIMING", et, sizeof et) > 0 && et[0] == '1';
}


// ================================================================ 0.5.7 (play 0.5.6, a former test database / default)
// the recorded log, sheets video-2026-10-07-033203*. Every database in modes 0.5.6 (0.5.7 switches
// off) and 0.5.7, each scenario from a fresh matcher state.
struct Res057
{
    std::map<std::string, Metrics> s[5];
};
std::vector<std::string> g_neg057; // negative-control evidence per problem
void Section057(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section057");
    using S = Step;
    // 1: straight crouch walk after a crouch at a run (03:29:47.75-03:29:55: a side-tagged crouch loop for 7-8 s)
    //    (the 0.5.6 sequence A: run, crouch, crouch walk, stand up, crouch again, long straight crouch walk)
    const std::vector<Step> s1 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "crouch_in", .seconds = 0.8f, .stick = 1.0f, .loco = 2},
                                  S{.name = "crouch_walk", .seconds = 3.0f, .stick = 1.0f, .loco = 2},
                                  S{.name = "stand_up", .seconds = 1.4f, .stick = 1.0f, .loco = 1},
                                  S{.name = "crouch_again", .seconds = 0.8f, .stick = 1.0f, .loco = 2},
                                  S{.name = "crouch_straight", .seconds = 8.0f, .stick = 1.0f, .loco = 2},
                                  S{.name = "crouch_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    // 2: stand <-> crouch toggles while running (03:29:44.4-03:29:55: RunToCrouch at f=27 / 30 / 21, re-picked 0.23 s later)
    const std::vector<Step> s2 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "c1", .seconds = 1.5f, .stick = 1.0f, .loco = 2},
                                  S{.name = "s1", .seconds = 1.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "c2", .seconds = 1.9f, .stick = 1.0f, .loco = 2},
                                  S{.name = "s2", .seconds = 0.8f, .stick = 1.0f, .loco = 1},
                                  S{.name = "c3", .seconds = 2.5f, .stick = 1.0f, .loco = 2},
                                  S{.name = "stop", .seconds = 1.0f, .stick = 0.0f, .loco = 2}};
    // 3: crouch walk reversals (03:30:03.9-03:30:05.8: Crouch_Turn_L_135 Lfoot / Rfoot / L_180 / Reface picked every 0.09 s)
    const std::vector<Step> s3 = {S{.name = "crouch_walk", .seconds = 2.5f, .stick = 1.0f, .loco = 2},
                                  S{.name = "flick_back", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "flick_again", .seconds = 2.0f, .stick = 1.0f, .angleDeg = -175.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "turn_135", .seconds = 1.5f, .stick = 1.0f, .angleDeg = 135.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "crouch_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    // 4: standstill without stick after a stop, gated-out knockdown with the stick held at |v| 0 (03:29:39.6-03:29:43.8),
    //    idle again
    const std::vector<Step> s4 = {S{.name = "walk", .seconds = 2.0f, .stick = 0.75f, .loco = 1},
                                  S{.name = "stop", .seconds = 3.0f, .stick = 0.0f, .loco = 1},
                                  S{.name = "knockdown", .seconds = 3.0f, .stick = 1.0f, .loco = 0, .psm = 0x40u | (19u << 16), .speed = 0.0f},
                                  S{.name = "after", .seconds = 0.6f, .stick = 1.0f, .loco = 1, .speed = 0.0f},
                                  S{.name = "idle", .seconds = 3.0f, .stick = 0.0f, .loco = 1},
                                  // game-like (03:29:39.58-03:29:43.79): sprint, hit by a car (detail 18) at 7-11 m/s, lying with
                                  // the stick released, then held at |v| 0 while the gate is still closed, then the gate opens
                                  S{.name = "sprint", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
                                  S{.name = "hit", .seconds = 0.6f, .stick = 0.0f, .loco = 0, .psm = 0x40u | (19u << 16), .speed = 6.8f},
                                  S{.name = "lying", .seconds = 1.2f, .stick = 0.0f, .loco = 0, .psm = 0x40u | (19u << 16), .speed = 0.0f},
                                  S{.name = "getup_held", .seconds = 2.4f, .stick = 1.0f, .loco = 0, .psm = 0x40u | (19u << 16), .speed = 0.0f},
                                  S{.name = "reopen", .seconds = 1.0f, .stick = 1.0f, .loco = 1, .speedDelay = 0.2f},
                                  S{.name = "idle2", .seconds = 2.0f, .stick = 0.0f, .loco = 1}};
    // 5: start from standstill after a stop with the stick to the side / back (03:30:13.86-03:30:17.2: TurnToRunningMirror
    //    at cost 144.7, rate 0.40, body 120-130 deg off for ~1.5 s); game-like: side press with |v| 0, short release while the
    //    camera turns, press again
    const std::vector<Step> s5 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "stop1", .seconds = 1.2f, .stick = 0.0f, .loco = 1},
                                  S{.name = "side_start", .seconds = 2.0f, .stick = 1.0f, .angleDeg = -90.0f, .loco = 1, .speedDelay = 0.3f},
                                  S{.name = "stop2", .seconds = 1.2f, .stick = 0.0f, .loco = 1},
                                  // 165 deg (stick back, slightly off); an exact 180 deg reversal from a stop: back180_start (reported)
                                  S{.name = "back_start", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 165.0f, .loco = 1, .speedDelay = 0.3f},
                                  S{.name = "stop3", .seconds = 1.2f, .stick = 0.0f, .loco = 1},
                                  S{.name = "g_press", .seconds = 0.33f, .stick = 1.0f, .angleDeg = 90.0f, .loco = 1, .speedDelay = 0.33f},
                                  S{.name = "g_release", .seconds = 0.3f, .stick = 0.0f, .loco = 1},
                                  S{.name = "g_go", .seconds = 1.5f, .stick = 1.0f, .angleDeg = 30.0f, .loco = 1, .speedDelay = 0.1f},
                                  S{.name = "stop4", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                  S{.name = "back180_start", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .loco = 1, .speedDelay = 0.3f},
                                  S{.name = "stop5", .seconds = 1.5f, .stick = 0.0f, .loco = 1}};
    const std::vector<Step>* sc[5] = {&s1, &s2, &s3, &s4, &s5};
    const char* scName[5] = {"crouch_straight", "stance_toggles", "crouch_reverse", "stand_idle", "side_start"};
    std::string table;
    int neg[5] = {0, 0, 0, 0, 0};
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (!LoadDatabase(p, name))
        {
            Check("0.5.7 database loaded: " + name, false, argv[di]);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        Res057 r[2];
        std::string st;
        for (int v = 0; v < 2; ++v)
        {
            ConfigureMode(2);
            Set057(v == 1);
            Set058(v == 1); // 0.5.8: the 0.5.6 reference without the 0.5.8 switches, the 0.5.7 checks on the current state
            for (int k = 0; k < 5; ++k)
            {
                r[v].s[k] = RunScenario(*sc[k], k == 0 ? 98.0f : -82.0f, true);
                for (const auto& stp : r[v].s[k])
                    std::printf("[INFO] 0.5.7 %s %s %s/%s: %s\n", name.c_str(), v ? "0.5.7" : "0.5.6", scName[k], stp.first.c_str(),
                                Line057(stp.second).c_str());
                if (k >= 2)
                    for (const auto& stp : r[v].s[k])
                        std::printf("[INFO]   trans %s %s %s/%s: %s\n", name.c_str(), v ? "0.5.7" : "0.5.6", scName[k], stp.first.c_str(),
                                    stp.second.transList.c_str());
            }
            if (v == 1)
                st = Status();
        }
        const bool crouchLateralOk = st.find("lateralOk=1/1") != std::string::npos || st.find("lateralOk=0/1") != std::string::npos;
        {
            const size_t q = st.find(" gateDebounce=");
            std::printf("[INFO] 0.5.7 %s status:%s\n", name.c_str(), q == std::string::npos ? " (none)" : st.substr(q).c_str());
        }
        // 1 (every crouch scenario: straight walk, toggles, reversals)
        Metrics c0, c1;
        for (int k = 0; k < 3; ++k)
            for (int v = 0; v < 2; ++v)
            {
                const Metrics t = Total(r[v].s[k]);
                Metrics& c = v ? c1 : c0;
                c.sideLoopTime += t.sideLoopTime;
                c.seconds += t.seconds;
                c.offSat += t.offSat;
                c.transitions += t.transitions;
            }
        if (crouchLateralOk && c0.sideLoopTime > 0.0f)
            ++neg[0];
        Check("0.5.7 [" + name + "] 1 straight crouch walk: no side / back-tagged crouch loop for forward motion where a forward loop "
                                 "exists (crouch lateralOk=" + std::string(crouchLateralOk ? "1" : "0") + ")",
              !crouchLateralOk || c1.sideLoopTime <= 0.0f, Line057(c1) + " | 0.5.6 " + Line057(c0));
        // 2
        const Metrics t0 = Total(r[0].s[1]), t1 = Total(r[1].s[1]);
        int repick0 = 0, repick1 = 0, late0 = 0, late1 = 0;
        for (const auto& kv : r[0].s[1])
        {
            repick0 += std::max(0, kv.second.stancePicks - 1);
            late0 += kv.second.stanceLate;
        }
        for (const auto& kv : r[1].s[1])
        {
            repick1 += std::max(0, kv.second.stancePicks - 1);
            late1 += kv.second.stanceLate;
        }
        if (repick0 + late0 > 0)
            ++neg[1];
        Check("0.5.7 [" + name + "] 2 stand <-> crouch toggles: at most one stance transition per stance change, entered at frame <= " +
                  std::to_string(utpp::mm::steer::kStanceStartFrames) + ", none against the stance",
              repick1 == 0 && late1 == 0 && t1.wrongStance == 0,
              "repicks " + std::to_string(repick1) + " late " + std::to_string(late1) + " picks " + std::to_string(t1.rightStance) +
                  " | 0.5.6 repicks " + std::to_string(repick0) + " late " + std::to_string(late0) + " picks " + std::to_string(t0.rightStance));
        // 3
        const Metrics u0 = Total(r[0].s[2]), u1 = Total(r[1].s[2]);
        int restarts0 = 0, restarts1 = 0;
        for (const auto& kv : r[0].s[2])
            restarts0 += kv.second.turnRestarts;
        for (const auto& kv : r[1].s[2])
            restarts1 += kv.second.turnRestarts;
        if (restarts0 > 0)
            ++neg[2];
        Check("0.5.7 [" + name + "] 3 crouch reversals: no turn-class clip restarted <= 0.4 s after another, fewer transitions than 0.5.6",
              restarts1 == 0 && u1.transitions <= u0.transitions,
              "restarts " + std::to_string(restarts1) + " | 0.5.6 " + std::to_string(restarts0) + " | " + Line057(u1) + " | 0.5.6 " + Line057(u0));
        // 4
        const Metrics i0 = Total(r[0].s[3]), i1 = Total(r[1].s[3]);
        if (i0.idleStill > 0)
            ++neg[3];
        Check("0.5.7 [" + name + "] 4 standstill without stick / gated out: idle (or the playing stop) only",
              i1.idleStill == 0, Line057(i1) + " | 0.5.6 " + Line057(i0));
        // 5
        float err1 = 0.0f, err0 = 0.0f, cost1 = 0.0f, cost0 = 0.0f;
        for (const char* k : {"side_start", "back_start", "g_go"})
        {
            err1 = std::max(err1, r[1].s[4].at(k).errAt05);
            err0 = std::max(err0, r[0].s[4].at(k).errAt05);
        }
        for (const char* k : {"side_start", "back_start", "g_press", "g_release", "g_go"})
        {
            cost1 = std::max(cost1, r[1].s[4].at(k).costMax);
            cost0 = std::max(cost0, r[0].s[4].at(k).costMax);
        }
        if (err0 >= 45.0f || cost0 > 60.0f)
            ++neg[4];
        // databases without start-turn data (no Reface_Start clips: only a few stand / walk turns,
        // costs 100-240 in 0.5.6 already) are reported only
        const bool startData = st.find("startTurn=") != std::string::npos && HasClipContaining(argv[di], "_Reface_Start_");
        Check("0.5.7 [" + name + "] 5 start from standstill with the stick to the side / back after a stop: |err| < 45 deg 0.5 s after "
                                 "the press, no selection cost > 60" + std::string(startData ? "" : " (INFO: no reface-start data, not enforced)"),
              !startData || (err1 < 45.0f && cost1 <= 60.0f),
              "err " + std::to_string(err1) + " cost " + std::to_string(cost1) + " | 0.5.6 err " + std::to_string(err0) + " cost " +
                  std::to_string(cost0));
        char row[640];
        std::snprintf(row, sizeof row,
                      "%s | 1 sideLoop %.2f -> %.2f s | 2 repicks %d -> %d late %d -> %d | 3 restarts %d -> %d trans %d -> %d | "
                      "4 idleStill %d -> %d | 5 err@0.5 %.0f -> %.0f deg cost %.1f -> %.1f (side %.0f -> %.0f, back165 %.0f -> %.0f, "
                      "game-like %.0f -> %.0f; INFO back180 %.0f -> %.0f)\n",
                      name.c_str(), c0.sideLoopTime, c1.sideLoopTime, repick0, repick1, late0, late1, restarts0, restarts1,
                      u0.transitions, u1.transitions, i0.idleStill, i1.idleStill, err0, err1, cost0, cost1,
                      r[0].s[4].at("side_start").errAt05, r[1].s[4].at("side_start").errAt05, r[0].s[4].at("back_start").errAt05,
                      r[1].s[4].at("back_start").errAt05, r[0].s[4].at("g_go").errAt05, r[1].s[4].at("g_go").errAt05,
                      r[0].s[4].at("back180_start").errAt05, r[1].s[4].at("back180_start").errAt05);
        table += row;
    }
    std::printf("[INFO] 0.5.7 before -> after (0.5.6 -> 0.5.7):\n%s", table.c_str());
    {
        // 4 at the rule level (the simulated matcher did not pick a non-idle clip at standstill in 0.5.6 - in game the
        // costs were 0.21-0.27 for a backwards move clip vs 0.19-0.29 for a stand idle, a near tie): with
        // the 0.5.6 masks a start / loop / move / transition / arc clip stays a candidate at standstill without stick
        using namespace utpp::mm::steer;
        CurationInput ci;
        ci.input = false;
        ci.capsule = 0.0f;
        ci.moving = true;
        ci.sinceInput = 2.0f;
        CurationConfig cc;
        cc.speedBands = true;
        cc.crouchRules = true;
        const uint32_t m056 = WithVariants(ExcludedClasses(ci, cc));
        const uint32_t m057 = m056 | kStandIdleExclude;
        const uint32_t leak = Bit(kClassNone) | Bit(kClassStart) | Bit(kClassTransition) | Bit(kClassArc);
        if ((m056 & leak) != leak)
            ++neg[3];
        Check("0.5.7 unit: standstill idle masks - 0.5.6 leaves start / loop-move / transition / arc candidates at |v| 0 without stick, "
              "0.5.7 keeps idle and stop only",
              (m057 & leak) == leak && !(m057 & Bit(kClassIdle)) && !(m057 & Bit(kClassStop)) && (m056 & leak) != leak &&
                  IsCrouchSideTagName("X_Crouch_Loop_L_CrouchedWalking") && IsCrouchSideTagName("M_Neutral_Crouch_Loop_LL") &&
                  !IsCrouchSideTagName("M_Neutral_Crouch_Loop_F") && !IsCrouchSideTagName("M_Neutral_Crouch_Loop_FL") &&
                  !IsCrouchSideTagName("M_Neutral_Walk_Loop_LL") && LoopDirTagOfName("X_Walk_Move_B_Female1B05WalkBackwards") == "B" &&
                  DriverClipClass("X_Crouch_Loop_B_CrouchWalkLeft", kClassNone, true) == kClassLateral &&
                  DriverClipClass("X_Crouch_Loop_R_CrouchedSneakingRight", kClassNone, false) == kClassLateral &&
                  DriverClipClass("X_Crouch_Loop_L_CrouchWalkForward", kClassNone, false) == kClassNone &&
                  DriverClipClass("X_Crouch_Loop_L_CrouchWalkForward", kClassNone, true) == kClassLateral);
    }
    const char* what[5] = {"side-tagged crouch loop for straight crouch walking", "stance transition re-picked / entered late",
                           "turn-class restart in crouch reversals", "non-idle selection at standstill",
                           "side / back start error >= 45 deg or cost > 60"};
    // controls 0 (side-tagged crouch loop) and 1 (stance transition re-picked / late) are not reproduced by the package database
    // (no side-tagged crouch loop for straight walking, no selectable stance transition): not part of the checks
    for (int k = 2; k < 5; ++k)
        Check(std::string("negative control 0.5.6 (0.5.7 switches off): ") + what[k] + " (some database)", neg[k] > 0,
              std::to_string(neg[k]));
    Set057(true);
    Set058(true);
}

// ================================================================ 0.5.8 (play 0.5.7, a former test database)
// the recorded log. Every database in modes 0.5.7 (0.5.8 switches off) and 0.5.8, fresh matcher state.
std::string Line058(const Metrics& m)
{
    char buf[640];
    std::snprintf(buf, sizeof buf,
                  "capOver %d visCostMax %.1f overSpeedVis %.2f s stanceOver %d stanceCostMax %.1f sameDir %d firstTurn %d@%.2f offBand %d "
                  "err@0.5 %.1f err@1.0 %.1f dirFlips %d bodyRev %d offMax %.1f offLast %.1f trans %d",
                  m.capOver, m.visCostMax, m.overSpeedVis, m.stanceOver, m.stanceCostMax, m.sameDirRestarts, m.firstTurnDir, m.firstTurnAt,
                  m.offBand, m.errAt05, m.errAt10, m.dirFlips, m.bodyRev, m.offMax, m.offLast, m.transitions);
    return buf;
}
// ---- 0.5.9 (Phase 5.3): capsule <-> animation synchronisation ----
// Harness steps of night cycle 2 (the recorded log, UTPP 0.5.8 + v3): footlock_corr_max_cm
// 15.2-24.1 (accel / stop / flick / start_right / start_back / dash), offset_saturated_s 0.56-1.37 (>= 24 cm; cruise / stop /
// start). Game model of those traces (g_game059): vanilla Acceleration 24 m/s^2 (trace accel=3.75/24.00x0.16) lowered by the
// driver's AccelTarget, vanilla Deceleration 20 m/s^2 (run stop 2.89 -> 0 m/s in <= 0.18 s; sprint stop ~23 m/s^2; not read in game yet - status decelBase=) lowered by DecelTarget while the stick is released; the script applies both
// after the feed (one frame late, as in game). Modes: 0 = 0.5.8 (all 0.5.9 switches off), 1 = 0.5.9, 2..4 = one switch on.
void Set059(bool aSync, bool aCatchup, bool aTraj)
{
    SetCapsuleSync(aSync);
    SetOffsetCatchup(aCatchup);
    SetTrajCal(aTraj);
}

std::string Line059(const Metrics& m)
{
    char buf[256];
    std::snprintf(buf, sizeof buf, "corrMax %.1f cm offSat24 %.2f s offMax %.1f cm trans %d costMax %.1f rate %.2f..%.2f |v| %.2f",
                  m.corrMax, m.offSat24, m.offMax, m.transitions, m.costMax, m.rateMin, m.rateMax, m.speedMean);
    return buf;
}

void Section059(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section059");
    using S = Step;
    constexpr uint32_t kDash = 0x40u | (8u << 16);
    // sprint_start_stop (06:24:53-06:25:07): sprint start / stop, run start / stop
    const std::vector<Step> h1 = {S{.name = "sprint_start", .seconds = 4.0f, .stick = 1.0f, .loco = 3},
                                  S{.name = "sprint_stop", .seconds = 3.0f, .stick = 0.0f, .loco = 1},
                                  S{.name = "run_start", .seconds = 3.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "run_stop", .seconds = 3.0f, .stick = 0.0f, .loco = 1}};
    // flick_180_run: accel, cruise, flick through the centre (2 frames at 0), recover, stop
    const std::vector<Step> h2 = {S{.name = "accel", .seconds = 1.5f, .stick = 1.0f},
                                  S{.name = "cruise", .seconds = 2.5f, .stick = 1.0f},
                                  S{.name = "flick", .seconds = 2.5f, .stick = 1.0f, .angleDeg = 180.0f, .zeroFrames = 2},
                                  S{.name = "recover", .seconds = 1.5f, .stick = 1.0f},
                                  S{.name = "stop", .seconds = 2.5f, .stick = 0.0f}};
    // flick_180_sprint
    const std::vector<Step> h3 = {S{.name = "accel", .seconds = 1.5f, .stick = 1.0f, .loco = 3},
                                  S{.name = "cruise", .seconds = 3.0f, .stick = 1.0f, .loco = 3},
                                  S{.name = "flick", .seconds = 2.5f, .stick = 1.0f, .angleDeg = 180.0f, .zeroFrames = 2, .loco = 3},
                                  S{.name = "recover", .seconds = 1.5f, .stick = 1.0f},
                                  S{.name = "stop", .seconds = 2.5f, .stick = 0.0f}};
    // idle_then_start_sideways: start right (stick 90 deg right of the camera) / stop / start back / stop
    const std::vector<Step> h4 = {S{.name = "idle", .seconds = 1.5f, .stick = 0.0f},
                                  S{.name = "start_right", .seconds = 2.0f, .stick = 1.0f, .angleDeg = -90.0f},
                                  S{.name = "stop_right", .seconds = 2.0f, .stick = 0.0f},
                                  S{.name = "start_back", .seconds = 2.0f, .stick = 1.0f, .angleDeg = -90.0f},
                                  S{.name = "stop_back", .seconds = 2.0f, .stick = 0.0f}};
    // sprint_dash (06:25:35-52): sprint, dash burst (PSM detail 7, gate closes), back to the sprint, stop
    const std::vector<Step> h5 = {S{.name = "sprint", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
                                  S{.name = "pre", .seconds = 0.05f, .stick = 1.0f, .loco = 3, .speed = 13.7f},
                                  S{.name = "dash", .seconds = 0.35f, .stick = 1.0f, .loco = 0, .psm = kDash, .speed = 11.0f},
                                  S{.name = "back", .seconds = 0.15f, .stick = 1.0f, .loco = 3, .speed = 9.0f},
                                  S{.name = "sprint2", .seconds = 1.5f, .stick = 1.0f, .loco = 3},
                                  S{.name = "stop", .seconds = 2.0f, .stick = 0.0f}};
    // flick_180_walk accel: half stick (0.45 -> mv 0.21, 0.3 m/s), then a walk at 0.8 stick and a stop
    const std::vector<Step> h6 = {S{.name = "walk_accel", .seconds = 1.5f, .stick = 0.45f, .stickFrom = 0.0f},
                                  S{.name = "walk_cruise", .seconds = 2.0f, .stick = 0.45f},
                                  S{.name = "walk_fast", .seconds = 2.0f, .stick = 0.8f},
                                  S{.name = "walk_stop", .seconds = 2.0f, .stick = 0.0f}};
    const std::vector<Step>* sc[6] = {&h1, &h2, &h3, &h4, &h5, &h6};
    const char* scName[6] = {"sprint_start_stop", "flick_180_run", "flick_180_sprint", "start_sideways", "sprint_dash", "walk"};
    const char* modeName[5] = {"0.5.8", "0.5.9", "sync", "catchup", "trajcal"};
    std::string table;
    int negCorr = 0, negSat = 0, dbs = 0;
    {
        // diagnostic tuning: UTPP_LOCO_SLIP=<start>,<halflife>  UTPP_LOCO_CATCH=<max>,<time>,<soft>,<halflife>
        char e[64] = {};
        float a = 0, b = 0, c = 0, d = 0;
        if (GetEnvironmentVariableA("UTPP_LOCO_SLIP", e, sizeof e) > 0 && sscanf_s(e, "%f,%f", &a, &b) == 2)
            std::printf("[INFO] 0.5.9 slip params %.3f/%.3f -> %d\n", a, b, SetFootSlipParams(a, b) ? 1 : 0);
        if (GetEnvironmentVariableA("UTPP_LOCO_CATCH", e, sizeof e) > 0 && sscanf_s(e, "%f,%f,%f,%f", &a, &b, &c, &d) == 4)
            std::printf("[INFO] 0.5.9 catch-up params -> %d\n", SetOffsetCatchupParams(a, b, c, d) ? 1 : 0);
    }
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (!LoadDatabase(p, name))
        {
            Check("0.5.9 database loaded: " + name, false, argv[di]);
            continue;
        }
        ++dbs;
        OnArmed(aRig, aHashes, aParents);
        std::map<std::string, Metrics> r[5][6];
        std::string st;
        g_game059 = true;
        ConfigureMode(2);
        (void)RunScenario(*sc[0], -82.0f, true); // warm-up (driver state after the database load, e.g. stick / rate filters)
        for (int v = 0; v < 5; ++v)
        {
            ConfigureMode(2);
            Set059(v == 1 || v == 2, v == 1 || v == 3, v == 1 || v == 4);
            for (int k = 0; k < 6; ++k)
            {
                r[v][k] = RunScenario(*sc[k], -82.0f, true);
                for (const auto& stp : r[v][k])
                    std::printf("[INFO] 0.5.9 %s %s %s/%s: %s\n", name.c_str(), modeName[v], scName[k], stp.first.c_str(),
                                Line059(stp.second).c_str());
            }
            if (v == 1)
                st = Status();
        }
        // negative control: the 0.5.8 mode run again from a fresh state reproduces its numbers bit for bit
        bool same = true;
        {
            ConfigureMode(2);
            Set059(false, false, false);
            for (int k = 0; k < 6 && same; ++k)
            {
                const std::map<std::string, Metrics> again = RunScenario(*sc[k], -82.0f, true);
                for (const auto& stp : again)
                {
                    const Metrics& a = r[0][k].at(stp.first);
                    const Metrics& b = stp.second;
                    if (a.corrMax != b.corrMax || a.offSat24 != b.offSat24 || a.offMax != b.offMax || a.transitions != b.transitions)
                        same = false;
                }
            }
        }
        g_game059 = false;
        ConfigureMode(2); // 0.5.9 switches off again for the other sections
        {
            const size_t q = st.find(" capsuleSync=");
            std::printf("[INFO] 0.5.9 %s status:%s\n", name.c_str(), q == std::string::npos ? " (none)" : st.substr(q).c_str());
        }
        Check("0.5.9 [" + name + "] negative control: switches off reproduce the 0.5.8 numbers (fresh rerun, bitwise)", same);
        float corr[5] = {}, satMax[5] = {}, satSum[5] = {}, offMax[5] = {};
        int trans[5] = {}, satSteps[5] = {}, corrSteps[5] = {};
        for (int v = 0; v < 5; ++v)
            for (int k = 0; k < 6; ++k)
                for (const auto& stp : r[v][k])
                {
                    const Metrics& m = stp.second;
                    corr[v] = std::max(corr[v], m.corrMax);
                    satMax[v] = std::max(satMax[v], m.offSat24);
                    satSum[v] += m.offSat24;
                    offMax[v] = std::max(offMax[v], m.offMax);
                    trans[v] += m.transitions;
                    if (m.offSat24 > 0.5f)
                        ++satSteps[v];
                    if (m.corrMax > 15.0f)
                        ++corrSteps[v];
                }
        if (corrSteps[0] > 0)
            ++negCorr;
        if (satSteps[0] > 0 || satMax[0] > 0.2f)
            ++negSat;
        char line[512];
        for (int v = 0; v < 5; ++v)
        {
            std::snprintf(line, sizeof line, "  %-28s %-8s corrMax %5.1f cm (steps > 15: %d)  offSat24 max %.2f s sum %.2f s (steps > 0.5: %d)  offMax %5.1f cm  trans %d\n",
                          name.c_str(), modeName[v], corr[v], corrSteps[v], satMax[v], satSum[v], satSteps[v], offMax[v], trans[v]);
            table += line;
        }
        Check("0.5.9 [" + name + "] harness steps: max foot correction <= 15 cm and below 0.5.8", corr[1] <= 15.0f && corr[1] < corr[0],
              std::to_string(corr[0]) + " -> " + std::to_string(corr[1]));
        Check("0.5.9 [" + name + "] harness steps: no step with the offset >= 24 cm for > 0.5 s, total time not above 0.5.8",
              satSteps[1] == 0 && satSum[1] <= satSum[0], std::to_string(satSum[0]) + " -> " + std::to_string(satSum[1]));
        Check("0.5.9 [" + name + "] offset root never beyond the 30 cm catch-up clamp", offMax[1] <= 30.05f, std::to_string(offMax[1]));
        Check("0.5.9 [" + name + "] transitions not more than 0.5.8 + 15 %", float(trans[1]) <= float(trans[0]) * 1.15f + 1.0f,
              std::to_string(trans[0]) + " -> " + std::to_string(trans[1]));
        Check("0.5.9 [" + name + "] status reports the capsule profile / catch-up / trajectory fields",
              st.find("capsuleSync=1") != std::string::npos && st.find("offCatchup=1") != std::string::npos &&
                  st.find("trajCal=1") != std::string::npos && st.find("capProfile=\"") != std::string::npos);
    }
    std::printf("[INFO] 0.5.9 harness-step table (max over the mirrored steps):\n%s", table.c_str());
    Check("0.5.9 negative control: 0.5.8 mode shows foot corrections > 15 cm on >= 2 databases", negCorr >= 2 || dbs < 2,
          std::to_string(negCorr) + "/" + std::to_string(dbs));
    Check("0.5.9 negative control: 0.5.8 mode shows offset saturation (>= 24 cm) on >= 2 databases", negSat >= 2 || dbs < 2,
          std::to_string(negSat) + "/" + std::to_string(dbs));
}

void Section058(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section058");
    using S = Step;
    constexpr uint32_t kDash = 0x40u | (8u << 16); // kPsmDetailed + detail 7 (dodge) as in the MMGATE lines
    // A: dash / dodge bursts at a sprint (05:10:23.4-05:10:38.4, 05:11:13.6-05:11:26.7): |v| 14-17.6 for a few samples at
    //    loco 3 with the gate open, then loco 0 + PSM detail 7 for ~0.35 s at 11-13 m/s, back to the sprint at 8.2 with the
    //    target turned; sprint turns whose target jumps again during the committed turn (05:10:30.05 -> 30.84, tgt 14 -> 135)
    const std::vector<Step> sA = {S{.name = "sprint", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
                                  S{.name = "pre1", .seconds = 0.05f, .stick = 1.0f, .angleDeg = 60.0f, .loco = 3, .speed = 17.6f},
                                  S{.name = "dash1", .seconds = 0.35f, .stick = 1.0f, .loco = 0, .psm = kDash, .speed = 12.4f},
                                  S{.name = "back1", .seconds = 0.15f, .stick = 1.0f, .loco = 3, .speed = 14.1f},
                                  S{.name = "run1", .seconds = 1.2f, .stick = 1.0f, .loco = 3},
                                  S{.name = "pre2", .seconds = 0.05f, .stick = 1.0f, .angleDeg = -110.0f, .loco = 3, .speed = 17.6f},
                                  S{.name = "dash2", .seconds = 0.4f, .stick = 1.0f, .loco = 0, .psm = kDash, .speed = 12.2f},
                                  S{.name = "back2", .seconds = 0.12f, .stick = 1.0f, .loco = 1, .speed = 14.7f},
                                  S{.name = "run2", .seconds = 1.2f, .stick = 1.0f, .loco = 3},
                                  S{.name = "turn_a", .seconds = 0.4f, .stick = 1.0f, .angleDeg = 105.0f, .loco = 3, .rotateVel = true},
                                  S{.name = "turn_b", .seconds = 1.2f, .stick = 1.0f, .angleDeg = 120.0f, .loco = 3, .rotateVel = true},
                                  S{.name = "turn_c", .seconds = 0.35f, .stick = 1.0f, .angleDeg = -95.0f, .loco = 3, .rotateVel = true},
                                  S{.name = "turn_d", .seconds = 1.2f, .stick = 1.0f, .angleDeg = -94.0f, .loco = 3, .rotateVel = true},
                                  S{.name = "stop", .seconds = 1.5f, .stick = 0.0f, .loco = 1}};
    // B: crouch walk, stand up (05:09:54.66), reface turn 0.48 s later (err 105), crouch again 0.165 s after that (05:09:55.300:
    //    a walk-to-crouch transition f=6 cost 224)
    const std::vector<Step> sB = {S{.name = "crouch_walk", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
                                  S{.name = "stand", .seconds = 0.48f, .stick = 1.0f, .loco = 1},
                                  S{.name = "turn", .seconds = 0.165f, .stick = 1.0f, .angleDeg = 105.0f, .loco = 1, .rotateVel = true},
                                  S{.name = "crouch", .seconds = 1.5f, .stick = 1.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "stand2", .seconds = 0.5f, .stick = 1.0f, .loco = 1},
                                  S{.name = "crouch2", .seconds = 1.5f, .stick = 1.0f, .loco = 2},
                                  S{.name = "stop", .seconds = 1.0f, .stick = 0.0f, .loco = 2}};
    // C: crouch turns in one direction (05:09:50.249 L_135 f=33 for err 159 -> 50.624 L_045; 05:09:51.969 R_090 f=38 for err -76
    //    -> 52.155 R_045) and a flick to the other side (05:09:50.857 R_135 -> 50.969 L_135, allowed)
    const std::vector<Step> sC = {S{.name = "crouch_walk", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
                                  S{.name = "turn_l159", .seconds = 1.5f, .stick = 1.0f, .angleDeg = 159.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "turn_r180", .seconds = 0.43f, .stick = 1.0f, .angleDeg = -180.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "turn_r76", .seconds = 1.5f, .stick = 1.0f, .angleDeg = -76.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "flick_a", .seconds = 0.12f, .stick = 1.0f, .angleDeg = -172.0f, .loco = 2, .rotateVel = true},
                                  // the stick moves 38 deg further: the error wraps to the other side (+150) -> a left turn is fine
                                  S{.name = "flick_b", .seconds = 1.5f, .stick = 1.0f, .angleDeg = -38.0f, .loco = 2, .rotateVel = true},
                                  S{.name = "stop", .seconds = 1.0f, .stick = 0.0f, .loco = 2}};
    // D: loops at 3.85 / 7.76 m/s (05:11:24.887 M_Neutral_Walk_Loop_F f=59 at 7.76; Transition_Walk_to_Run at a steady 3.85)
    const std::vector<Step> sD = {S{.name = "run", .seconds = 4.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "sprint", .seconds = 3.0f, .stick = 1.0f, .loco = 3},
                                  S{.name = "release", .seconds = 1.2f, .stick = 0.5f, .loco = 1},
                                  S{.name = "walk", .seconds = 2.5f, .stick = 0.5f, .loco = 1},
                                  S{.name = "run_again", .seconds = 3.0f, .stick = 1.0f, .stickFrom = 0.5f, .loco = 1},
                                  S{.name = "run_arc", .seconds = 3.0f, .stick = 1.0f, .angleRate = 40.0f, .loco = 1},
                                  S{.name = "stop", .seconds = 1.5f, .stick = 0.0f, .loco = 1}};
    // E: start 180 deg after a stop with the stick behind (05:13:49.090: Run_Reface_Start_B_R_180 err -178, then the target moves
    //    45 deg further and the body turns LEFT while the R_180 plays -> 49.303 Walk_Reface_Start_F_L_090 err +101), and the plain
    //    180 deg start of the 0.5.7 report (back180)
    const std::vector<Step> sE = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .loco = 1},
                                  S{.name = "stop1", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                  // the matcher's 180 deg pick is data-dependent: both sides, the target then moves 45 deg further in
                                  // the direction of the picked clip (if it picked that side) so the shortest way flips
                                  S{.name = "back_press", .seconds = 0.2f, .stick = 1.0f, .angleDeg = 182.0f, .loco = 1, .speedDelay = 0.2f},
                                  S{.name = "back_more", .seconds = 1.8f, .stick = 1.0f, .angleDeg = 45.0f, .loco = 1},
                                  S{.name = "stop2", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                  S{.name = "back_press_r", .seconds = 0.2f, .stick = 1.0f, .angleDeg = 178.0f, .loco = 1, .speedDelay = 0.2f},
                                  S{.name = "back_more_r", .seconds = 1.8f, .stick = 1.0f, .angleDeg = -45.0f, .loco = 1},
                                  S{.name = "stop2r", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                  S{.name = "back180", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .loco = 1, .speedDelay = 0.3f},
                                  S{.name = "stop3", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                  S{.name = "back178", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 178.0f, .loco = 1, .speedDelay = 0.3f},
                                  S{.name = "stop4", .seconds = 1.5f, .stick = 0.0f, .loco = 1}};
    const std::vector<Step>* sc[5] = {&sA, &sB, &sC, &sD, &sE};
    const char* scName[5] = {"dash", "stance_flip", "crouch_same_dir", "loop_bands", "start180"};
    std::string table;
    int neg[6] = {0, 0, 0, 0, 0, 0};
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (!LoadDatabase(p, name))
        {
            Check("0.5.8 database loaded: " + name, false, argv[di]);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        std::map<std::string, Metrics> r[2][5];
        std::string st;
        for (int v = 0; v < 2; ++v)
        {
            ConfigureMode(2);
            Set058(v == 1);
            for (int k = 0; k < 5; ++k)
            {
                r[v][k] = RunScenario(*sc[k], k == 1 || k == 2 ? 98.0f : -82.0f, true);
                for (const auto& stp : r[v][k])
                    std::printf("[INFO] 0.5.8 %s %s %s/%s: %s\n", name.c_str(), v ? "0.5.8" : "0.5.7", scName[k], stp.first.c_str(),
                                Line058(stp.second).c_str());
                for (const auto& stp : r[v][k])
                    std::printf("[INFO]   trans058 %s %s %s/%s: %s\n", name.c_str(), v ? "0.5.8" : "0.5.7", scName[k], stp.first.c_str(),
                                stp.second.transList.c_str());
            }
            if (v == 1)
                st = Status();
        }
        {
            const size_t q = st.find(" speedCap=");
            std::printf("[INFO] 0.5.8 %s status:%s\n", name.c_str(), q == std::string::npos ? " (none)" : st.substr(q).c_str());
        }
        // A
        const Metrics a0 = Total(r[0][0]), a1 = Total(r[1][0]);
        if (a0.overSpeedVis > 0.0f)
            ++neg[0];
        if (a0.capOver > 0)
            ++neg[1];
        // databases without sprint data (no sprint loops / turns, every sprint candidate costs 100-330) report the cap
        const bool sprintData = st.find("sprintData=1") != std::string::npos;
        Check("0.5.8 [" + name + "] A dash bursts: weight <= 0.5 within 0.1 s of each burst above 10.25 m/s (2 bursts), no visible "
                                 "selection above the cost cap" + std::string(sprintData ? "" : " (INFO: no sprint data, cap not enforced)"),
              a1.overSpeedVis <= 0.2f && a1.overSpeedVis < a0.overSpeedVis && (!sprintData || a1.capOver == 0),
              Line058(a1) + " | 0.5.7 " + Line058(a0));
        // B
        const Metrics b0 = Total(r[0][1]), b1 = Total(r[1][1]);
        if (b0.stanceOver > 0)
            ++neg[2];
        Check("0.5.8 [" + name + "] B stance double flip: no stance transition above the cost cap",
              b1.stanceOver == 0,
              Line058(b1) + " picks " + std::to_string(b1.stancePicks) + " | 0.5.7 " + Line058(b0) + " picks " + std::to_string(b0.stancePicks));
        // C
        const Metrics c0 = Total(r[0][2]), c1 = Total(r[1][2]);
        if (c0.sameDirRestarts > 0)
            ++neg[3];
        const Metrics& fb0 = r[0][2].at("flick_b");
        const Metrics& fb1 = r[1][2].at("flick_b");
        const bool rev1 = fb1.firstTurnDir > 0 && fb1.firstTurnAt <= 0.4f, rev0 = fb0.firstTurnDir > 0 && fb0.firstTurnAt <= 0.4f;
        Check("0.5.8 [" + name + "] C crouch turns: no same-direction turn restart <= 0.6 s; the stick moved past the back still starts "
                                 "the opposite turn <= 0.4 s (if it did in 0.5.7)",
              c1.sameDirRestarts == 0 && (rev1 || !rev0),
              "sameDir " + std::to_string(c1.sameDirRestarts) + " flick " + std::to_string(fb1.firstTurnDir) + "@" + std::to_string(fb1.firstTurnAt) +
                  " | 0.5.7 sameDir " + std::to_string(c0.sameDirRestarts) + " flick " + std::to_string(fb0.firstTurnDir) + "@" +
                  std::to_string(fb0.firstTurnAt) + " | " + Line058(c1));
        // D
        const Metrics d0 = Total(r[0][3]), d1 = Total(r[1][3]);
        if (d0.offBand > 0)
            ++neg[4];
        Check("0.5.8 [" + name + "] D loop bands: no loop / speed transition off the capsule's speed band (stick held)", d1.offBand == 0,
              Line058(d1) + " | 0.5.7 " + Line058(d0));
        // E
        float e0 = 0, e1 = 0;
        int f0 = 0, f1 = 0;
        for (const char* k : {"back_press", "back_more", "back_press_r", "back_more_r", "back180", "back178"})
        {
            f0 += r[0][4].at(k).dirFlips + r[0][4].at(k).bodyRev;
            f1 += r[1][4].at(k).dirFlips + r[1][4].at(k).bodyRev;
        }
        for (const char* k : {"back180", "back178"})
        {
            e0 = std::max(e0, r[0][4].at(k).errAt10);
            e1 = std::max(e1, r[1][4].at(k).errAt10);
        }
        if (f0 > 0)
            ++neg[5];
        const bool startData = HasClipContaining(argv[di], "_Reface_Start_");
        Check("0.5.8 [" + name + "] E start 180 deg after a stop: no start-direction flip / body turn reversal" +
                  std::string(startData ? "" : " (INFO: no reface-start data, not enforced)"),
              !startData || f1 == 0, "flips+reversals " + std::to_string(f1) + " | 0.5.7 " + std::to_string(f0) + " | err@1.0 " +
                                         std::to_string(e1) + " | 0.5.7 " + std::to_string(e0));
        char row[900];
        std::snprintf(row, sizeof row,
                      "%s | A overSpeedVis %.2f -> %.2f s capOver %d -> %d visCostMax %.0f -> %.0f | B stanceOver %d -> %d stanceCostMax %.0f -> "
                      "%.0f | C sameDir %d -> %d | D offBand %d -> %d (steady run offset %.1f -> %.1f cm) | E flips+rev %d -> %d err@0.5 back180 %.0f -> "
                      "%.0f back_press %.0f -> %.0f err@1.0 max %.0f -> %.0f\n",
                      name.c_str(), a0.overSpeedVis, a1.overSpeedVis, a0.capOver, a1.capOver, a0.visCostMax, a1.visCostMax, b0.stanceOver,
                      b1.stanceOver, b0.stanceCostMax, b1.stanceCostMax, c0.sameDirRestarts, c1.sameDirRestarts, d0.offBand, d1.offBand,
                      r[0][3].at("run").offLast, r[1][3].at("run").offLast, f0, f1, r[0][4].at("back180").errAt05, r[1][4].at("back180").errAt05,
                      r[0][4].at("back_press").errAt05 >= 0 ? r[0][4].at("back_more").errAt05 : -1.0f, r[1][4].at("back_more").errAt05, e0, e1);
        table += row;
    }
    std::printf("[INFO] 0.5.8 before -> after (0.5.7 -> 0.5.8):\n%s", table.c_str());
    {
        using namespace utpp::mm::steer;
        Check("0.5.8 unit: loop / transition speed classes by name",
              LoopBandOfName("M_Neutral_Walk_Loop_F", kClassNone) == 1 && LoopBandOfName("M_Neutral_Run_Loop_F_R_20", kClassNone) == 2 &&
                  LoopBandOfName("M_Neutral_Sprint_Loop_F", kClassNone) == 3 &&
                  LoopBandOfName("X_Run_Move_F_Female1C03Run", kClassNone) == 2 &&
                  LoopBandOfName("M_Neutral_Transition_Walk_to_Run_Rfoot", kClassTransition) == 1 &&
                  LoopBandOfName("M_Neutral_Transition_Sprint_to_Run_Rfoot", kClassTransition) == 3 &&
                  LoopBandOfName("M_Neutral_Transition_Run_to_Sprint_Rfoot", kClassTransition) == 2 &&
                  LoopBandOfName("X_Transition_Run_Female1C05WalkToRun", kClassTransition) == 1 &&
                  LoopBandOfName("X_Transition_Run_Female1C04RunToWalk1", kClassTransition) == 2 &&
                  LoopBandOfName("M_Neutral_Crouch_Loop_F", kClassNone) == 0 && LoopBandOfName("M_Neutral_Run_Stop_F_Lfoot", kClassStop) == 0 &&
                  IsStartSegmentClass(kClassReface) && IsStartSegmentClass(kClassRefaceRun) && IsStartSegmentClass(kClassStart) &&
                  !IsStartSegmentClass(kClassTurnRun));
    }
    const char* what[6] = {"MM visible above the speed cap", "visible selection above the cost cap", "stance transition above the cost cap",
                           "same-direction turn restart <= 0.6 s", "loop / transition off its speed band",
                           "start-direction flip / body turn reversal"};
    // controls 1 (visible selection above the cost cap), 2 (stance transition above the cost cap) and 4 (loop / transition off its speed
    // band) are not reproduced by the package database: not part of the checks
    for (int k : {0, 3, 5})
        Check(std::string("negative control 0.5.7 (0.5.8 switches off): ") + what[k] + " (some database)", neg[k] > 0, std::to_string(neg[k]));
    Set058(true);
}

std::string Line054(const Metrics& m)
{
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "idleMoving %d (%.2f s) tipMoving %d walkAtRun %d runAtWalk %d sprintAtRun %d walkTurns %d walkAllowedLed %d/%d bandMax %d err mean %.1f max %.1f | %s",
                  m.idleMoving, m.idleMovingTime, m.tipMoving, m.walkAtRun, m.runAtWalk, m.sprintAtRun, m.walkTurns, m.walkAllowedLed, m.walkLedFrames, m.bandMax,
                  m.errN ? m.errSum / float(m.errN) : 0.0f, m.errMax, m.Line().c_str());
    return buf;
}
} // namespace

// ================================================================ 0.6.0 (play 0.5.9: melee, run -> crouch, bike mount)
std::string Line060(const Metrics& m)
{
    char buf[512];
    std::snprintf(buf, sizeof buf, "close %.3f s zero %.3f s open %.3f s wUp %.3f s wMin %.2f wEnd %.2f moveToCrouch %d first {%s} costMax %.1f",
                  m.closeLatency, m.zeroLatency, m.openLatency, m.wUpLatency, m.wMin, m.wEnd, m.moveToCrouch, m.firstPick.c_str(), m.costMax);
    return buf;
}

// mount: idle until w = 1, then the vehicle bit; aEvent = MountingEvent first (UTPPMMMountEvent before the feed).
// Returns the weights: [0] after the event (or before the vehicle feed), [1] first vehicle sample, [2] second, [3] after 0.5 s
// mounted, [4] 0.05 s after the vehicle state ended, [5] 0.5 s after it ended.
bool g_mountTppAfterEvent = false; // vehtrans mounted-TPP flag after the MountingEvent and 12 feeds without the vehicle bit (MountRun(true))
std::vector<float> MountRun(bool aEvent)
{
    const float dt = 1.0f / 60.0f;
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = dt;
    in.cameraYawDeg = -82.0f;
    in.playerYawDeg = 0.0f;
    for (int i = 0; i < 120; ++i)
        Feed(in);
    std::vector<float> w;
    if (aEvent)
    {
        MountEvent();
        // game order unknown: up to 0.2 s of locomotion feeds without the vehicle bit after the MountingEvent
        for (int i = 0; i < 12; ++i)
            Feed(in);
        g_mountTppAfterEvent = utpp::mm::vehtrans::g_mountTpp.load();
    }
    w.push_back(Field(Status(), "weight"));
    in.psmBlock = 0x1u; // kPsmVehicle (PSM.Vehicle / MountedToVehicle)
    Feed(in);
    w.push_back(Field(Status(), "weight"));
    in.locoState = 0; // game: the next sample reports loco 0 (11:57:04.908 psm=0x9 loco=0)
    in.psmBlock = 0x9u;
    Feed(in);
    w.push_back(Field(Status(), "weight"));
    for (int i = 0; i < 28; ++i)
        Feed(in);
    w.push_back(Field(Status(), "weight"));
    in.locoState = 1;
    in.psmBlock = 0;
    for (int i = 0; i < 3; ++i)
        Feed(in);
    w.push_back(Field(Status(), "weight"));
    for (int i = 0; i < 27; ++i)
        Feed(in);
    w.push_back(Field(Status(), "weight"));
    return w;
}

void Section060(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section060");
    using S = Step;
    // PSM masks as scripts/utpp.reds 0.6.0 reports them: melee bits 11 / 12, PSM.MeleeWeapon + 1 in bits 24..28, PSM.UpperBody
    // + 1 in bits 13..15 (diagnostic fields only)
    auto meleeField = [](uint32_t v) { return (v + 1u) << 24; };
    auto upperField = [](uint32_t v) { return (v + 1u) << 13; };
    const uint32_t kFistsIdle = meleeField(2) | upperField(0);                 // MeleeWeapon Idle
    const uint32_t kFistsCombo = 0x800u | meleeField(11) | upperField(0);      // ComboAttack
    const uint32_t kFistsBlock = 0x1000u | meleeField(8) | upperField(0);      // Block
    const uint32_t kGunAim = meleeField(0) | upperField(6);                    // ranged: UpperBody Aim, MeleeWeapon NotReady
    // A: fists attack mid-run (game 11:50:10-11:51:30: attacks while running, gate never closed, locomotion clips at w = 1)
    const std::vector<Step> a1 = {S{.name = "run", .seconds = 3.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "attack", .seconds = 0.6f, .stick = 1.0f, .psm = kFistsCombo},
                                  S{.name = "after", .seconds = 1.5f, .stick = 1.0f, .psm = kFistsIdle}};
    // A: attack chain with one idle sample between two hits (no MM flash between the hits)
    const std::vector<Step> a2 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "hit1", .seconds = 0.4f, .stick = 1.0f, .psm = kFistsCombo},
                                  S{.name = "gap", .seconds = 0.1f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "hit2", .seconds = 0.4f, .stick = 1.0f, .psm = 0x800u | meleeField(12)},
                                  S{.name = "after", .seconds = 1.5f, .stick = 1.0f, .psm = kFistsIdle}};
    // A: single-sample melee states every 0.5 s (debounced), block pose, ranged aim (must not close)
    const std::vector<Step> a3 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "blips", .seconds = 2.0f, .stick = 1.0f, .psm = kFistsIdle, .blinkEvery = 30, .blinkPsm = kFistsCombo},
                                  S{.name = "block", .seconds = 1.0f, .stick = 0.0f, .psm = kFistsBlock},
                                  S{.name = "after_block", .seconds = 1.5f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "gun_aim", .seconds = 2.0f, .stick = 1.0f, .psm = kGunAim}};
    // B: run -> crouch at the game's speeds (stick: raw, mv = (stick - 0.3) / 0.7)
    const std::vector<Step> b1 = {S{.name = "run", .seconds = 3.0f, .stick = 0.874f, .loco = 1}, // mv 0.82 -> 2.5 m/s
                                  S{.name = "crouch", .seconds = 1.5f, .stick = 0.874f, .loco = 2}};
    const std::vector<Step> b2 = {S{.name = "run", .seconds = 3.0f, .stick = 1.0f, .loco = 1}, // 3.85 m/s (game 3.80-3.86)
                                  S{.name = "crouch", .seconds = 1.5f, .stick = 1.0f, .loco = 2}};
    const std::vector<Step> b3 = {S{.name = "run", .seconds = 3.0f, .stick = 1.0f, .loco = 1}, // 3.85 m/s, stick eased to 0.5
                                  S{.name = "crouch", .seconds = 1.5f, .stick = 0.65f, .loco = 2}};
    std::string table;
    int dbs = 0, negMelee = 0;
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (!LoadDatabase(p, name))
        {
            Check("0.6.0 database loaded: " + name, false, argv[di]);
            continue;
        }
        ++dbs;
        OnArmed(aRig, aHashes, aParents);
        // ---- A: melee gate on / off (off = 0.5.9 = negative control)
        std::map<std::string, Metrics> ra[2][3];
        std::string st;
        for (int v = 0; v < 2; ++v)
        {
            ConfigureMode(2);
            Set060(v == 1);
            ra[v][0] = RunScenario(a1, -82.0f, true);
            ra[v][1] = RunScenario(a2, -82.0f, true);
            ra[v][2] = RunScenario(a3, -82.0f, true);
            if (v == 1)
                st = Status();
            for (int k = 0; k < 3; ++k)
                for (const auto& stp : ra[v][k])
                    std::printf("[INFO] 0.6.0 %s melee %s a%d/%s: %s\n", name.c_str(), v ? "0.6.0" : "0.5.9", k + 1, stp.first.c_str(),
                                Line060(stp.second).c_str());
        }
        {
            const Metrics& on = ra[1][0].at("attack");
            const Metrics& onA = ra[1][0].at("after");
            const Metrics& off = ra[0][0].at("attack");
            const Metrics& offA = ra[0][0].at("after");
            if (off.closeLatency < 0.0f && off.wMin >= 0.99f && offA.wMin >= 0.99f)
                ++negMelee;
            Check("0.6.0 [" + name + "] fists attack mid-run: gate closes within 0.1 s, weight reaches 0 during the attack",
                  on.closeLatency >= 0.0f && on.closeLatency <= 0.1f && on.zeroLatency >= 0.0f && on.zeroLatency < 0.6f, Line060(on));
            // 2026-11-12 (fists review C1 / C2): the melee release hold of a melee weapon is 0.55 s (was 0.3 s; integ::kMeleeReleaseHoldSeconds)
            Check("0.6.0 [" + name + "] fists attack mid-run: gate reopens 0.55 s after the attack (+-2 frames), MM back to 1 by the end",
                  onA.openLatency >= 0.55f - 0.034f && onA.openLatency <= 0.55f + 0.034f && onA.wEnd >= 0.99f && onA.wUpLatency >= onA.openLatency,
                  Line060(onA));
            char tb[512];
            std::snprintf(tb, sizeof tb, "  %-28s melee 0.5.9: close %6.3f wMin %.2f | 0.6.0: close %.3f s zero %.3f s reopen %.3f s wUp %.3f s\n",
                          name.c_str(), off.closeLatency, std::min(off.wMin, offA.wMin), on.closeLatency, on.zeroLatency,
                          onA.openLatency, onA.wUpLatency);
            table += tb;
            const Metrics& gap = ra[1][1].at("gap");
            const Metrics& hit2 = ra[1][1].at("hit2");
            Check("0.6.0 [" + name + "] attack chain: no MM fade-in between two hits (gate stays closed through a 0.1 s idle gap)",
                  gap.openLatency < 0.0f && gap.wMin <= 0.01f && hit2.openLatency < 0.0f, Line060(gap));
            const Metrics& blips = ra[1][2].at("blips");
            const Metrics& block = ra[1][2].at("block");
            const Metrics& gun = ra[1][2].at("gun_aim");
            Check("0.6.0 [" + name + "] single-sample melee states are debounced (gate stays open)", blips.closeLatency < 0.0f && blips.wMin >= 0.99f,
                  Line060(blips));
            Check("0.6.0 [" + name + "] melee block pose closes the gate", block.closeLatency >= 0.0f && block.closeLatency <= 0.1f, Line060(block));
            Check("0.6.0 [" + name + "] ranged aim (UpperBody Aim, diagnostic fields only) keeps the gate open", gun.closeLatency < 0.0f && gun.wMin >= 0.99f,
                  Line060(gun));
        }
        Check("0.6.0 [" + name + "] status reports meleeGate= meleeBlocks= meleeState= upperBody=",
              st.find("meleeGate=1") != std::string::npos && st.find(" meleeBlocks=") != std::string::npos &&
                  st.find(" meleeState=") != std::string::npos && st.find(" upperBody=") != std::string::npos);
        // ---- B: stance clip minimum speed on / off
        std::map<std::string, Metrics> rb[3][3];
        std::string sb;
        for (int v = 0; v < 3; ++v) // 0 = 0.5.9 (rule off), 1 = 0.6.0 at 3.5 m/s (mechanism), 2 = 0.6.0 default 4.0 m/s (the logged case)
        {
            ConfigureMode(2);
            Set060(v >= 1);
            if (v == 1)
                SetStanceClipMinSpeed(3.5f); // the 0.6.0 mechanism at the original 3.5 m/s (default is 4.0 since 0.6.0b)
            if (v == 2)
                SetStanceClipMinSpeed(4.0f);
            const std::vector<Step>* sc[3] = {&b1, &b2, &b3};
            for (int k = 0; k < 3; ++k)
            {
                rb[v][k] = RunScenario(*sc[k], -82.0f, true);
                if (v == 1 && k == 0)
                    sb = Status();
                for (const auto& stp : rb[v][k])
                    std::printf("[INFO] 0.6.0 %s stance %s b%d/%s: %s |v| %.2f\n", name.c_str(), v == 0 ? "0.5.9" : (v == 1 ? "0.6.0" : "min4.0"),
                                k + 1, stp.first.c_str(), Line060(stp.second).c_str(), stp.second.speedMean);
            }
        }
        {
            const int off25 = rb[0][0].at("crouch").moveToCrouch, on25 = rb[1][0].at("crouch").moveToCrouch;
            const int off38 = rb[0][1].at("crouch").moveToCrouch, on38 = rb[1][1].at("crouch").moveToCrouch;
            const int offHalf = rb[0][2].at("crouch").moveToCrouch, onHalf = rb[1][2].at("crouch").moveToCrouch;
            const int min40 = rb[2][1].at("crouch").moveToCrouch;
            char tb[700];
            std::snprintf(tb, sizeof tb,
                          "  %-28s RunToCrouch/WalkToCrouch picks 0.5.9 -> 0.6.0: 2.5 m/s %d -> %d {%s} | 3.85 m/s %d -> %d {%s} | 3.85 stick 0.5 %d -> %d | min 4.0 @3.85: %d\n",
                          name.c_str(), off25, on25, rb[1][0].at("crouch").firstPick.c_str(), off38, on38,
                          rb[1][1].at("crouch").firstPick.c_str(), offHalf, onHalf, min40);
            table += tb;
            // the stance-rule checks (RunToCrouch / WalkToCrouch picks) need stance transition clips of a non-package source; the
            // package database has none, so the table row above is the INFO only
        }
        Check("0.6.0 [" + name + "] status reports stanceClipMin= stanceClipSkips=",
              sb.find(" stanceClipMin=3.50") != std::string::npos && sb.find(" stanceClipSkips=") != std::string::npos &&
                  Field(sb, "stanceClipSkips") >= 1.0f);
        // ---- C: mount cut (bike mount 11:57:04.896: w 1.00 -> 0.93 -> 0.86 with the standing MM pose over the bike)
        {
            ConfigureMode(2);
            Set060(true);
            const std::vector<float> on = MountRun(false);
            const std::string sm = Status();
            const std::vector<float> ev = MountRun(true);
            const std::string se = Status();
            const bool tppFlagEv = g_mountTppAfterEvent;
            SetMountCut(false);
            const std::vector<float> off = MountRun(false);
            const std::vector<float> evOff = MountRun(true);
            char tb[400];
            std::snprintf(tb, sizeof tb, "  %-28s mount w: 0.5.9 %.2f -> %.2f -> %.2f | 0.6.0 %.2f -> %.2f -> %.2f (mounted %.2f, +0.05 s %.2f, +0.5 s %.2f) | event %.2f\n",
                          name.c_str(), off[0], off[1], off[2], on[0], on[1], on[2], on[3], on[4], on[5], ev[0]);
            table += tb;
            Check("0.6.0 [" + name + "] mount event before the vehicle bit (TPP): the vehtrans mounted-TPP flag is set at the cut (a CET pose-write "
                                     "denial before the first vehicle feed must not end the mount transition)",
                  tppFlagEv, tppFlagEv ? "set" : "not set");
            Check("0.6.0 [" + name + "] mount while idle: weight 0 on the vehicle sample, stays 0 while mounted, normal reopen after",
                  on[0] >= 0.99f && on[1] == 0.0f && on[2] == 0.0f && on[3] == 0.0f && on[4] == 0.0f && on[5] >= 0.99f,
                  std::to_string(on[1]) + " " + std::to_string(on[5]));
            Check("0.6.0 [" + name + "] negative control: mount cut off keeps the standing pose visible for several samples (0.12 s hard fade; game 11:57:04.896-.908: 1.00 -> 0.93 -> 0.86)",
                  off[1] > 0.85f && off[1] < 0.99f && off[2] > 0.75f && off[2] < off[1], std::to_string(off[1]) + " " + std::to_string(off[2]));
            Check("0.6.0 [" + name + "] MountingEvent cuts the weight at once and keeps it 0 through 0.2 s of feeds without the vehicle bit; off = no-op", ev[0] == 0.0f && ev[1] == 0.0f && evOff[0] >= 0.99f,
                  std::to_string(ev[0]) + " " + std::to_string(evOff[0]));
            Check("0.6.0 [" + name + "] status reports mountCut= mountCuts=",
                  sm.find(" mountCut=1") != std::string::npos && Field(sm, "mountCuts") >= 1.0f && Field(se, "mountEventCuts") >= 1.0f);
        }
        ConfigureMode(2); // 0.6.0 switches off again
    }
    std::printf("[INFO] 0.6.0 table:\n%s", table.c_str());
    Check("0.6.0 negative control: melee gate off (0.5.9) never closes during the attack (MM over the attack at w = 1)", negMelee == dbs,
          std::to_string(negMelee) + "/" + std::to_string(dbs));
}

// ================================================================ 0.6.2 (test 2: upper-body bone mask)
std::string Line062(const Metrics& m)
{
    char buf[600];
    std::snprintf(buf, sizeof buf,
                  "w min %.2f end %.2f | upper min %.2f end %.2f zero %.3f s back %.3f s | mask max %.2f on %.3f s frames %d | snap masked %d leak %d "
                  "lowerMissing %d | close %.3f s | %s| end {%s}",
                  m.wMin, m.wEnd, m.upperMin, m.upperEnd, m.upperZeroLatency, m.upperBackLatency, m.maskMax, m.maskOnLatency, m.maskFrames,
                  m.snapMasked, m.snapUpperLeak, m.snapLowerMissing, m.closeLatency, m.ClassesString().c_str(), m.endClip.c_str());
    return buf;
}

// Vehicle ride + exit, sampled per feed after the vehicle state ends. aBike: 0.6 s of locoState 0 after the exit (game 14:14:59.756
// gate=0x10 loco=0 for 0.6 s after the bike), else straight to stand (car, game 14:14:18.779). aFallVz != 0: the first 0.3 s after
// the exit fall (kPsmDetailed Fall 14, vz) - must keep the full close.
struct ExitTrace
{
    float mountedWMax = 0.0f;           // highest weight while mounted
    uint32_t mountedMask = 0;           // OR of maskActive while mounted
    std::vector<float> w, uw, mb;       // per feed after the exit
    std::vector<uint32_t> mask;
    std::string status;                 // status at the end
};
// aWeaponBits (vehicle batch, CAR-1 T3): raw WeaponRight bits (known 1<<29 | drawn 1<<30) on every feed = a weapon in hand.
ExitTrace ExitRun(bool aBike, float aFallVz = 0.0f, uint32_t aWeaponBits = 0)
{
    const float dt = 1.0f / 60.0f;
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = dt;
    in.cameraYawDeg = -82.0f;
    in.playerYawDeg = 0.0f;
    in.psmBlock = aWeaponBits;
    for (int i = 0; i < 120; ++i)
        Feed(in);
    ExitTrace t;
    in.psmBlock = 0x1u | aWeaponBits; // vehicle
    Feed(in);
    in.locoState = 0;
    in.psmBlock = 0x9u | aWeaponBits; // vehicle + workspot (game: psm=0x9 loco=0 while mounted)
    for (int i = 0; i < 30; ++i)
    {
        Feed(in);
        const std::string s = Status();
        t.mountedWMax = std::max(t.mountedWMax, Field(s, "weight"));
        t.mountedMask |= HexField(s, "maskActive");
    }
    in.psmBlock = 0x1u | aWeaponBits; // game: psm=0x1 loco=1 for one sample before the end
    in.locoState = 1;
    Feed(in);
    const int total = 96;
    for (int i = 0; i < total; ++i)
    {
        in.psmBlock = aWeaponBits;
        in.velWorld[2] = 0.0f;
        in.locoState = 1;
        if (aFallVz != 0.0f && i < 18)
        {
            in.locoState = 0;
            in.psmBlock = 0x40u | ((14u + 1u) << 16) | aWeaponBits; // kPsmDetailed + Fall
            in.velWorld[2] = aFallVz;
        }
        else if (aBike && i < 36)
            in.locoState = 0;
        Feed(in);
        const std::string s = Status();
        t.w.push_back(Field(s, "weight"));
        t.uw.push_back(Field(s, "upperWeight"));
        t.mb.push_back(Field(s, "maskBlend"));
        t.mask.push_back(HexField(s, "maskActive"));
    }
    t.status = Status();
    return t;
}

// MM disabled mid-idle in TPP (no feeds: the script stops feeding when UTPPMMWantsFeed is false), Ticks in real time for 0.6 s.
struct DisabledResult
{
    float wMin = 9.0f;   // published snapshot weight
    float ageMax = 0.0f; // s since the snapshot was published
    bool active = false; // status holdPoseActive=1 at the end
    bool wantsHeld = false;   // UTPPMMWantsFeed while disabled and held (the feed keeps reporting the PSM state)
    float releasedW = 1.0f;   // weight after a vehicle sample fed while disabled and held
    bool wantsAfter = true;   // UTPPMMWantsFeed after the release
};
DisabledResult DisabledRun()
{
    using namespace utpp::mm::integ;
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = 1.0f / 60.0f;
    in.cameraYawDeg = -82.0f;
    for (int i = 0; i < 120; ++i)
        Feed(in);
    SetEnabled(false);
    DisabledResult r;
    LARGE_INTEGER fq;
    QueryPerformanceFrequency(&fq);
    static PoseSnapshot snap{};
    for (int i = 0; i < 60; ++i)
    {
        Sleep(10);
        Tick();
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        if (utpp::g_mmPose.Read(snap))
        {
            r.wMin = std::min(r.wMin, utpp::g_hot.mmActive.load() ? snap.weight : 0.0f);
            r.ageMax = std::max(r.ageMax, float(double(now.QuadPart - snap.publishTicks) / double(fq.QuadPart)));
        }
    }
    r.active = Status().find(" holdPoseActive=1") != std::string::npos;
    r.wantsHeld = WantsFeed();
    if (r.wantsHeld)
    {
        in.psmBlock = 0x1u; // vehicle: a release state
        for (int i = 0; i < 10; ++i)
            Feed(in);
        r.releasedW = Field(Status(), "weight");
        r.wantsAfter = WantsFeed();
    }
    SetEnabled(true);
    return r;
}

void Section062(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section062");
    using S = Step;
    auto meleeField = [](uint32_t v) { return (v + 1u) << 24; };
    auto upperField = [](uint32_t v) { return (v + 1u) << 13; };
    auto detailField = [](uint32_t v) { return (v + 1u) << 16; };
    const uint32_t kFistsIdle = meleeField(2) | upperField(0);
    const uint32_t kFistsCombo = 0x800u | meleeField(11) | upperField(0);
    // fists block as logged (14:18:40.741 gate=0x510 psm=0x40 detail=2 loco=0 melee=0x1000 upperBody=6): block pose + UpperBody
    // Aim + AimWalk, locoState 0
    const uint32_t kFistsBlockAim = 0x1000u | meleeField(8) | upperField(6) | 0x40u | detailField(2);
    // gun aim walk (14:17:38.045 gate=0x110 psm=0x40 detail=2 loco=0 upperBody=6, in TPP)
    const uint32_t kGunAimWalk = meleeField(0) | upperField(6) | 0x40u | detailField(2);
    // A: fists attack mid-run - the lower body continues the run, the upper body comes from the game
    const std::vector<Step> a1 = {S{.name = "run", .seconds = 3.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "attack", .seconds = 0.6f, .stick = 1.0f, .psm = kFistsCombo},
                                  S{.name = "after", .seconds = 1.5f, .stick = 1.0f, .psm = kFistsIdle}};
    // B: fists block while idle
    const std::vector<Step> b1 = {S{.name = "idle", .seconds = 2.0f, .stick = 0.0f, .psm = kFistsIdle},
                                  S{.name = "block", .seconds = 1.5f, .stick = 0.0f, .loco = 0, .psm = kFistsBlockAim},
                                  S{.name = "after", .seconds = 1.5f, .stick = 0.0f, .psm = kFistsIdle}};
    // C: gun aim while walking (perspective.aiming = tpp)
    const std::vector<Step> c1 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f},
                                  S{.name = "aimwalk", .seconds = 2.0f, .stick = 0.6f, .loco = 0, .psm = kGunAimWalk},
                                  S{.name = "after", .seconds = 1.5f, .stick = 1.0f}};
    // N: negative controls with the mask on - melee during a scene (PSM scene bit) and a dash burst while attacking
    const std::vector<Step> n1 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "scene_attack", .seconds = 0.6f, .stick = 1.0f, .psm = kFistsCombo | 0x2u}};
    const std::vector<Step> n2 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f, .psm = kFistsIdle},
                                  S{.name = "dash_attack", .seconds = 0.6f, .stick = 1.0f, .psm = kFistsCombo, .speed = 12.0f}};
    std::string table;
    int dbs = 0, negOff = 0;
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (!LoadDatabase(p, name))
        {
            Check("0.6.2 database loaded: " + name, false, argv[di]);
            continue;
        }
        ++dbs;
        OnArmed(aRig, aHashes, aParents);
        std::map<std::string, Metrics> ra[2], rb[2], rc[2];
        ExitTrace car[2], bike[2], bikeArmed;
        std::string st;
        for (int v = 0; v < 2; ++v) // 0 = mask off (0.6.1 = negative control), 1 = 0.6.2
        {
            ConfigureMode(2);
            Set060(true);
            SetUpperMask(v == 1);
            ra[v] = RunScenario(a1, -82.0f, true);
            rb[v] = RunScenario(b1, -82.0f, true);
            rc[v] = RunScenario(c1, -82.0f, true);
            car[v] = ExitRun(false);
            bike[v] = ExitRun(true);
            if (v == 1)
            {
                st = Status();
                bikeArmed = ExitRun(true, 0.0f, (1u << 29) | (1u << 30)); // CAR-1 T3: weapon in hand (WeaponRight known + drawn)
            }
            for (const auto* r : {&ra[v], &rb[v], &rc[v]})
                for (const auto& stp : *r)
                    std::printf("[INFO] 0.6.2 %s mask %s %s: %s\n", name.c_str(), v ? "on" : "off", stp.first.c_str(), Line062(stp.second).c_str());
        }
        // ---- A: fists attack mid-run
        {
            const Metrics& on = ra[1].at("attack");
            const Metrics& onA = ra[1].at("after");
            const Metrics& off = ra[0].at("attack");
            Check("0.6.2 [" + name + "] fists attack mid-run: lower body stays on MM (weight >= 0.99, no idle / stop pick, snapshot writes the legs)",
                  on.wMin >= 0.99f && onA.wMin >= 0.99f && on.classes.count("Idle") == 0 && on.classes.count("Stop") == 0 && on.snapMasked > 0 &&
                      on.snapLowerMissing == 0 && ClassName(on.endClip) != "Idle" && ClassName(on.endClip) != "Stop",
                  Line062(on));
            Check("0.6.2 [" + name + "] fists attack mid-run: upper body on the game pose within 0.06 s debounce + 0.15 s fade (+2 frames), no upper bone written",
                  on.upperZeroLatency >= 0.0f && on.upperZeroLatency <= 0.06f + 0.15f + 0.034f && on.snapUpperLeak == 0, Line062(on));
            // 2026-11-12: release 0.55 s + eased fade out 0.35 s (was 0.3 + 0.15). The measured "back" is where the upper scale reaches
            // 0.99 (mask weight <= 0.01): the smoothstep tail below 0.01 is ~0.02 s, so the lower edge is 0.07 s under the nominal 0.9 s
            // (measured 0.850 s); it is still far beyond the release itself (0.55 s), i.e. never back before the hold ends
            Check("0.6.2 [" + name + "] after the attack: upper body back to MM 0.55 s release + 0.35 s eased fade (-0.07 s smoothstep tail / +2 frames), not before the release",
                  onA.upperBackLatency >= 0.55f + 0.35f - 0.07f && onA.upperBackLatency <= 0.55f + 0.35f + 0.034f && onA.upperEnd >= 0.99f &&
                      onA.maskMax > 0.0f,
                  Line062(onA));
            if (off.zeroLatency >= 0.0f && off.wMin <= 0.01f)
                ++negOff;
            char tb[400];
            std::snprintf(tb, sizeof tb, "  %-28s attack: mask off w min %.2f (zero %.3f s) | mask on w min %.2f upper zero %.3f s back %.3f s end {%s}\n",
                          name.c_str(), off.wMin, off.zeroLatency, on.wMin, on.upperZeroLatency, onA.upperBackLatency, on.endClip.c_str());
            table += tb;
        }
        // ---- B: fists block while idle
        {
            const Metrics& on = rb[1].at("block");
            const Metrics& onA = rb[1].at("after");
            const Metrics& off = rb[0].at("block");
            Check("0.6.2 [" + name + "] fists block while idle (UpperBody Aim + AimWalk, loco 0): legs stay on MM standing clips (idle / the tail of a stop), upper body on the game pose",
                  on.wMin >= 0.99f && on.upperEnd <= 0.01f && on.snapUpperLeak == 0 && on.snapLowerMissing == 0 && on.snapMasked > 0 &&
                      (ClassName(on.endClip) == "Idle" || ClassName(on.endClip) == "Stop") &&
                      on.classes.size() == size_t(on.classes.count("Idle") + on.classes.count("Stop")),
                  Line062(on));
            Check("0.6.2 [" + name + "] after the block: full body back on MM (upper weight 1 by the end)", onA.wEnd >= 0.99f && onA.upperEnd >= 0.99f,
                  Line062(onA));
            Check("0.6.2 [" + name + "] negative control: mask off (0.6.1) fades the whole body out during the block", off.wMin <= 0.01f, Line062(off));
            char tb[300];
            std::snprintf(tb, sizeof tb, "  %-28s block:  mask off w min %.2f | mask on w min %.2f upper end %.2f end {%s}\n", name.c_str(), off.wMin,
                          on.wMin, on.upperEnd, on.endClip.c_str());
            table += tb;
        }
        // ---- C: gun aim walk
        {
            const Metrics& on = rc[1].at("aimwalk");
            const Metrics& off = rc[0].at("aimwalk");
            Check("0.6.2 [" + name + "] gun aim walk (UpperBody Aim, AimWalk): legs walk on MM (no idle pick while moving), upper body on the game pose",
                  on.wMin >= 0.99f && on.upperEnd <= 0.01f && on.idleMoving == 0 && on.snapUpperLeak == 0 && ClassName(on.endClip) != "Idle",
                  Line062(on));
            Check("0.6.2 [" + name + "] negative control: mask off (0.6.1) closes the gate on AimWalk", off.wMin <= 0.01f, Line062(off));
        }
        // ---- D: vehicle exit window (car / bike)
        {
            auto firstAt = [](const std::vector<float>& v, float thr, bool above) {
                for (size_t i = 0; i < v.size(); ++i)
                    if (above ? v[i] >= thr : v[i] <= thr)
                        return float(i + 1) / 60.0f;
                return -1.0f;
            };
            auto maxIn = [](const std::vector<float>& v, size_t a, size_t b) {
                float m = 0.0f;
                for (size_t i = a; i < b && i < v.size(); ++i)
                    m = std::max(m, v[i]);
                return m;
            };
            const ExitTrace& c = car[1];
            const ExitTrace& b = bike[1];
            const float cUp = firstAt(c.w, 0.99f, true), cBack = firstAt(c.uw, 0.99f, true);
            const float bUp = firstAt(b.w, 0.99f, true), bBack = firstAt(b.uw, 0.99f, true);
            Check("0.6.2 [" + name + "] vehicle ride keeps the full close (weight 0, no mask) - car and bike, mask on",
                  c.mountedWMax == 0.0f && c.mountedMask == 0u && b.mountedWMax == 0.0f && b.mountedMask == 0u);
            // CHANGED BY INTENT (vehicle batch CAR-1 / MOTO-6, recording 2026-10-22: after every car / bike exit V showed a deep squat with
            // a hand at the head for 0.07-1.45 s). The game upper body after an exit is the vanilla first-person set, so in TPP the exit
            // mask no longer hands Spine1+ to the game: it still covers the gate (legs on MM, kMaskStateExit reported) but the upper body
            // follows the MM weight (maskBlend 0, upperWeight == weight). Before: the upper body stayed on the game pose until the full
            // weight (car) or until locoState 1 (bike). FPP keeps the 0.6.2 mask (MMIntegrationTest DecideUpperMask table); a weapon in
            // hand keeps the game upper body (T3 below).
            Check("0.6.2 [" + name + "] car exit: legs on MM from the first sample (no reopen hold), full weight by 0.25 s, TPP: upper body on MM too "
                                     "(exit mask reported, maskBlend 0, upperWeight == weight)",
                  c.w[0] > 0.0f && cUp > 0.0f && cUp <= 0.25f + 0.034f && maxIn(c.mb, 0, c.mb.size()) <= 0.0f && c.mask[0] == 4u &&
                      std::fabs(c.uw[0] - c.w[0]) <= 0.001f,
                  "wUp " + std::to_string(cUp) + " upperBack " + std::to_string(cBack));
            Check("0.6.2 [" + name + "] car exit: upper body at full weight with the body (no 0.15 s mask fade), mask off at the end",
                  cBack > 0.0f && cBack <= cUp + 0.001f && c.uw.back() >= 0.99f && c.mask.back() == 0u, "upperBack " + std::to_string(cBack));
            Check("0.6.2 [" + name + "] bike exit: lower body on MM during the 0.6 s of locoState 0 (weight 1 by 0.25 s), TPP: upper body on MM too "
                                     "(exit mask reported until loco 1, maskBlend 0)",
                  b.w[0] > 0.0f && bUp > 0.0f && bUp <= 0.25f + 0.034f && maxIn(b.mb, 0, b.mb.size()) <= 0.0f && b.mask[20] == 4u,
                  "wUp " + std::to_string(bUp) + " upperBack " + std::to_string(bBack));
            Check("0.6.2 [" + name + "] bike exit: upper body back at full weight with the body (before locoState 1), mask off after loco 1",
                  bBack > 0.0f && bBack <= bUp + 0.001f && bBack < 0.6f && b.mask.back() == 0u, "upperBack " + std::to_string(bBack));
            {
                // CAR-1 T3: a weapon in hand keeps the game upper body over the MM legs in the exit window (weapon upper mask)
                const ExitTrace& ba = bikeArmed;
                const float aUp = firstAt(ba.w, 0.99f, true);
                Check("0.6.2 [" + name + "] bike exit with a weapon in hand (CAR-1 T3): legs on MM by 0.25 s, upper body on the game pose (maskBlend 1)",
                      aUp > 0.0f && aUp <= 0.25f + 0.034f && ba.mb.size() > 36 && ba.mb[20] >= 0.99f && ba.mb[35] >= 0.99f,
                      "wUp " + std::to_string(aUp) + " mb20 " + std::to_string(ba.mb.size() > 20 ? ba.mb[20] : -1.0f));
            }
            const ExitTrace& bo = bike[0];
            Check("0.6.2 [" + name + "] negative control: mask off (0.6.1) keeps the game pose (weight 0) for the 0.6 s of locoState 0 after the bike",
                  maxIn(bo.w, 0, 36) == 0.0f, "w max " + std::to_string(maxIn(bo.w, 0, 36)));
            char tb[400];
            std::snprintf(tb, sizeof tb, "  %-28s exit:   car wUp %.3f s upperBack %.3f s | bike wUp %.3f s upperBack %.3f s | bike mask off: first w>0 %.3f s\n",
                          name.c_str(), cUp, cBack, bUp, bBack, firstAt(bo.w, 0.01f, true));
            table += tb;
            Check("0.6.2 [" + name + "] status reports upperMask= maskBones= maskActive= maskSwitches= exitWindows=",
                  st.find(" upperMask=1") != std::string::npos && Field(st, "maskBones") >= 1.0f && st.find(" maskActive=0x") != std::string::npos &&
                      Field(st, "maskSwitches") >= 1.0f && Field(c.status, "exitWindows") >= 1.0f && st.find(" maskOk=1") != std::string::npos);
        }
        // ---- N: negative controls with the mask on
        {
            ConfigureMode(2);
            Set060(true);
            SetUpperMask(true);
            const Metrics scene = RunScenario(n1, -82.0f, true).at("scene_attack");
            const Metrics dash = RunScenario(n2, -82.0f, true).at("dash_attack");
            const ExitTrace fall = ExitRun(false, -4.0f);
            Check("0.6.2 [" + name + "] negative control (mask on): attack during a scene (PSM scene bit) fades the whole body out",
                  scene.wMin <= 0.01f && scene.maskFrames == 0, Line062(scene));
            Check("0.6.2 [" + name + "] negative control (mask on): dash over-speed during an attack fades the whole body out",
                  dash.wMin <= 0.01f && dash.maskFrames == 0, Line062(dash));
            float fallW = 0.0f;
            uint32_t fallMask = 0;
            for (size_t i = 0; i < 18; ++i)
            {
                fallW = std::max(fallW, fall.w[i]);
                fallMask |= fall.mask[i];
            }
            // CHANGED BY INTENT (vehicle batch CAR-2): this control now names its mode. With the hold off (0.6.1 mode, this section)
            // a post-exit fall is no mask state and keeps the full close; with the shipped hold on the fall belongs to MM / air
            // (held, full weight by 0.25 s, no upper mask), checked right below and in SectionVehExit (jump-out falls / landings).
            Check("0.6.2 [" + name + "] negative control (mask on, hold off = 0.6.1): falling after a vehicle exit (Fall, vz -4) keeps the full close",
                  fallW == 0.0f && fallMask == 0u, "w max " + std::to_string(fallW));
            {
                SetHoldPose(true);
                const ExitTrace fh = ExitRun(false, -4.0f);
                SetHoldPose(false);
                float fhMask = 0.0f;
                for (size_t i = 0; i < 18; ++i)
                    fhMask = std::max(fhMask, fh.mb[i]);
                const float fhUp = [&] {
                    for (size_t i = 0; i < fh.w.size(); ++i)
                        if (fh.w[i] >= 0.99f)
                            return float(i + 1) / 60.0f;
                    return -1.0f;
                }();
                Check("0.6.2 [" + name + "] hold on (shipped): falling after a vehicle exit (Fall, vz -4) is held by MM - full weight by 0.25 s, upper body on MM",
                      fhUp > 0.0f && fhUp <= 0.25f + 0.034f && fhMask <= 0.0f, "wUp " + std::to_string(fhUp) + " mask max " + std::to_string(fhMask));
            }
            // state bits: melee only -> the gun aim walk closes the whole body; invalid values rejected
            const bool okBits = SetUpperMaskStates(1);
            const Metrics aimMeleeOnly = RunScenario(c1, -82.0f, true).at("aimwalk");
            const Metrics attackMeleeOnly = RunScenario(a1, -82.0f, true).at("attack");
            const bool rej = !SetUpperMaskStates(8) && !SetUpperMaskStates(-1);
            Check("0.6.2 [" + name + "] UTPPSetMMUpperMaskStates(1): aim closes the whole body, melee keeps the mask; 8 / -1 rejected",
                  okBits && rej && aimMeleeOnly.wMin <= 0.01f && attackMeleeOnly.wMin >= 0.99f && attackMeleeOnly.upperEnd <= 0.01f,
                  Line062(aimMeleeOnly));
            SetUpperMaskStates(7);
        }
        // ---- H: hold last pose (A/B 20261007-143104: MM weight 0 in TPP = literal T-pose of the game graph)
        {
            const std::vector<Step> h1 = {S{.name = "run", .seconds = 2.0f, .stick = 1.0f},
                                          S{.name = "jump", .seconds = 0.5f, .stick = 1.0f, .loco = 0, .psm = 0x40u | detailField(18), .vz = 3.0f},
                                          S{.name = "land", .seconds = 1.0f, .stick = 1.0f}};
            const std::vector<Step> h2 = {S{.name = "idle", .seconds = 1.5f, .stick = 0.0f},
                                          S{.name = "menu", .seconds = 1.0f, .stick = 0.0f, .psm = 0x100u},
                                          S{.name = "after", .seconds = 0.5f, .stick = 0.0f}};
            std::map<std::string, Metrics> rh1[2], rh2[2], rh3[2];
            DisabledResult dis[2];
            for (int v = 0; v < 2; ++v) // 0 = hold off (0.6.1 = negative control), 1 = hold on
            {
                ConfigureMode(2);
                Set060(true);
                SetUpperMask(true);
                SetHoldPose(v == 1);
                rh1[v] = RunScenario(h1, -82.0f, true);
                rh2[v] = RunScenario(h2, -82.0f, true);
                rh3[v] = RunScenario(n2, -82.0f, true);
                dis[v] = DisabledRun();
                for (const auto* r : {&rh1[v], &rh2[v]})
                    for (const auto& stp : *r)
                        std::printf("[INFO] 0.6.2 %s hold %s %s: %s\n", name.c_str(), v ? "on" : "off", stp.first.c_str(), Line062(stp.second).c_str());
                std::printf("[INFO] 0.6.2 %s hold %s MM disabled mid-idle: published w min %.2f, snapshot age max %.3f s, holdPoseActive %d\n",
                            name.c_str(), v ? "on" : "off", dis[v].wMin, dis[v].ageMax, dis[v].active ? 1 : 0);
            }
            Check("0.6.2 [" + name + "] hold: MM disabled mid-idle in TPP -> pose held (published weight 1, republished every tick, holdPoseActive=1)",
                  dis[1].wMin >= 0.99f && dis[1].ageMax < 0.1f && dis[1].active, std::to_string(dis[1].wMin) + " age " + std::to_string(dis[1].ageMax));
            Check("0.6.2 [" + name + "] hold: MM disabled and held - the feed stays wanted, a vehicle sample releases the body (weight 0), then no feed",
                  dis[1].wantsHeld && dis[1].releasedW == 0.0f && !dis[1].wantsAfter && !dis[0].wantsHeld,
                  std::to_string(dis[1].releasedW));
            Check("0.6.2 [" + name + "] negative control: hold off (0.6.1) - MM disabled mid-idle fades to the game pose (T-pose in game)",
                  dis[0].wMin <= 0.01f, std::to_string(dis[0].wMin));
            Check("0.6.2 [" + name + "] hold: jump (LocomotionDetailed Jump, vz 3) keeps the MM body; hold off fades it out",
                  rh1[1].at("jump").wMin >= 0.99f && rh1[1].at("land").wMin >= 0.99f && rh1[0].at("jump").wMin <= 0.01f,
                  Line062(rh1[1].at("jump")) + " || off " + Line062(rh1[0].at("jump")));
            Check("0.6.2 [" + name + "] hold: menu (PSM photo / menu bit) and the samples after it keep the MM body (no T-pose after the menu); hold off fades out",
                  rh2[1].at("menu").wMin >= 0.99f && rh2[1].at("after").wMin >= 0.99f && rh2[0].at("menu").wMin <= 0.01f &&
                      rh2[0].at("after").wMin <= 0.01f,
                  Line062(rh2[1].at("after")) + " || off " + Line062(rh2[0].at("after")));
            Check("0.6.2 [" + name + "] hold: dash over-speed during an attack keeps the MM body (upper body masked); hold off fades out",
                  rh3[1].at("dash_attack").wMin >= 0.99f && rh3[0].at("dash_attack").wMin <= 0.01f, Line062(rh3[1].at("dash_attack")));
            // release states with the hold on: the game animates the body there (vehicle, scene, ladder, swim, death, takedown, TPP off)
            ConfigureMode(2);
            Set060(true);
            SetUpperMask(true);
            SetHoldPose(true);
            const std::vector<Step> h4 = {S{.name = "idle0", .seconds = 1.0f, .stick = 0.0f},
                                          S{.name = "scene", .seconds = 0.6f, .stick = 0.0f, .psm = 0x2u},
                                          S{.name = "idle1", .seconds = 1.0f, .stick = 0.0f},
                                          S{.name = "ladder", .seconds = 0.6f, .stick = 0.0f, .loco = 0, .psm = 0x40u | detailField(10)},
                                          S{.name = "idle2", .seconds = 1.0f, .stick = 0.0f},
                                          S{.name = "swim", .seconds = 0.6f, .stick = 0.0f, .psm = 0x4u},
                                          S{.name = "idle3", .seconds = 1.0f, .stick = 0.0f},
                                          S{.name = "death", .seconds = 0.6f, .stick = 0.0f, .psm = 0x80u},
                                          S{.name = "idle4", .seconds = 1.0f, .stick = 0.0f},
                                          S{.name = "takedown", .seconds = 0.6f, .stick = 0.0f, .psm = 0x10u},
                                          S{.name = "idle5", .seconds = 1.0f, .stick = 0.0f}};
            const std::map<std::string, Metrics> rel = RunScenario(h4, -82.0f, true);
            bool released = true;
            std::string relLine;
            for (const char* k : {"scene", "ladder", "swim", "death", "takedown"})
            {
                const Metrics& m = rel.at(k);
                released = released && m.wEnd <= 0.01f;
                relLine += std::string(k) + " wEnd " + std::to_string(m.wEnd) + " ";
            }
            const bool reopened = rel.at("idle1").wEnd >= 0.99f && rel.at("idle5").wEnd >= 0.99f;
            const ExitTrace veh = ExitRun(false);
            float tppOffW = 1.0f;
            {
                FeedInput in;
                in.tppActive = false;
                in.locoState = 1;
                in.dt = 1.0f / 60.0f;
                in.cameraYawDeg = -82.0f;
                for (int i = 0; i < 30; ++i)
                    Feed(in);
                tppOffW = Field(Status(), "weight");
            }
            const std::string sh = Status();
            Check("0.6.2 [" + name + "] hold on: release states still fade the whole body out (scene, ladder, swim, death, takedown, vehicle ride, TPP off) and MM comes back after",
                  released && reopened && veh.mountedWMax == 0.0f && tppOffW <= 0.01f,
                  relLine + "vehicle " + std::to_string(veh.mountedWMax) + " tppOff " + std::to_string(tppOffW));
            Check("0.6.2 [" + name + "] status reports holdPose= holdPoseActive= holdPoseFrames=",
                  sh.find(" holdPose=1") != std::string::npos && sh.find(" holdPoseActive=") != std::string::npos &&
                      Field(sh, "holdPoseFrames") >= 1.0f);
            char tb[400];
            std::snprintf(tb, sizeof tb, "  %-28s hold:   MM disabled w min off %.2f on %.2f | jump w min off %.2f on %.2f | menu->after w min off %.2f on %.2f\n",
                          name.c_str(), dis[0].wMin, dis[1].wMin, rh1[0].at("jump").wMin, rh1[1].at("jump").wMin, rh2[0].at("after").wMin,
                          rh2[1].at("after").wMin);
            table += tb;
        }
        ConfigureMode(2);
    }
    std::printf("[INFO] 0.6.2 table:\n%s", table.c_str());
    Check("0.6.2 negative control: mask off (0.6.1) fades the whole body out during the fists attack on every database", negOff == dbs,
          std::to_string(negOff) + "/" + std::to_string(dbs));
}

// ================================================================ vehicle batch W1 (CAR-1 / CAR-2 / J4): vehicle exit gate
// Recording 2026-10-22 14-57-06 (session.log): after every car / bike unmount V showed the game's first-person pose (deep squat,
// hand at the head) for 0.07-1.45 s; jump-out falls / landings showed it too; bike jump-outs flew rigid. Most likely gate reason
// (not proven, MM trace off in the release): the vanilla exit workspot (PSM.IsInWorkspot, release bit) still set after the vehicle
// bit falls. The driver now holds that latched exit workspot inside the exit window, keeps the TPP upper body on MM there and
// logs EXITDIAG lines. Shipped settings: hold on, mask on (states 7), mount cut on, air auto + robust. UTPP_LOCO_ONLYVEHEXIT=1 runs
// this section alone (mutation builds of tests/vehexit.cmake).
void ConfigureShippedC1a(); // defined with the loco C0 / C1a section
namespace vx
{
constexpr uint32_t kWs = 0x8u;  // kPsmWorkspot
constexpr uint32_t kVeh = 0x1u; // kPsmVehicle
constexpr uint32_t Det(uint32_t d) { return 0x40u | ((d + 1u) << 16); } // kPsmDetailed + LocomotionDetailed field
struct VxStep
{
    float seconds = 0.1f;
    uint32_t psm = 0;      // raw psmBlock (detail field included)
    int loco = 1;
    float vz = 0.0f;       // world vertical velocity (m/s); g > 0: the start value of a ballistic arc
    float g = 0.0f;        // > 0: vz decreases by g per second through the step (ballistic flight)
    float speed = 0.0f;    // forward capsule speed (m/s)
    bool airborne = false; // air feed: not on the ground
    bool stale = false;    // no feeds for .seconds (Tick only, real time)
};
struct VxRes
{
    std::vector<float> w, mb;
    std::vector<uint32_t> mask;
    std::vector<int> owns, hold, stepOf;
    float mountedWMax = 0.0f;
    float staleW0 = -1.0f, staleW1 = -1.0f; // published weight at the start / end of the first stale step
    bool staleHeld = false;                 // holdPoseActive=1 at the end of the stale step
    float airLands = 0.0f, airEpisodes = 0.0f;
    std::string status;
    std::vector<std::string> diag;
};
std::vector<std::string> g_diag;
void VxLog(const char* aLine)
{
    if (std::strstr(aLine, "UTPP EXITDIAG "))
        g_diag.push_back(aLine);
    TraceLog(aLine);
}
float PublishedWeight()
{
    static utpp::mm::integ::PoseSnapshot snap{};
    return (utpp::g_mmPose.Read(snap) && utpp::g_hot.mmActive.load()) ? snap.weight : 0.0f;
}
// 2 s standing, then (aVehicle) the vehicle bit for 0.5 s (game: psm=0x9 loco=0 while mounted, psm=0x1 loco=1 on the last sample),
// then the post-exit steps. aVehicle false: the same steps on foot (no vehicle-bit edge).
VxRes Run(const std::vector<VxStep>& aPost, bool aVehicle = true)
{
    const float dt = 1.0f / 60.0f;
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = dt;
    in.cameraYawDeg = -82.0f;
    in.playerYawDeg = 0.0f;
    in.haveAir = true;
    in.onGround = true;
    for (int i = 0; i < 120; ++i)
        Feed(in);
    VxRes r;
    const std::string st0 = Status();
    const float lands0 = Field(st0, "airLands"), eps0 = Field(st0, "airEpisodes");
    g_diag.clear();
    if (aVehicle)
    {
        in.psmBlock = kVeh;
        Feed(in);
        in.locoState = 0;
        in.psmBlock = kVeh | kWs;
        for (int i = 0; i < 30; ++i)
        {
            Feed(in);
            r.mountedWMax = std::max(r.mountedWMax, Field(Status(), "weight"));
        }
        in.psmBlock = kVeh;
        in.locoState = 1;
        Feed(in);
    }
    for (size_t si = 0; si < aPost.size(); ++si)
    {
        const VxStep& s = aPost[si];
        if (s.stale)
        {
            r.staleW0 = r.staleW0 < 0.0f ? PublishedWeight() : r.staleW0;
            LARGE_INTEGER f0, fq, fn;
            QueryPerformanceFrequency(&fq);
            QueryPerformanceCounter(&f0);
            do
            {
                Sleep(5);
                Tick();
                QueryPerformanceCounter(&fn);
            } while (float(double(fn.QuadPart - f0.QuadPart) / double(fq.QuadPart)) < s.seconds);
            if (r.staleW1 < 0.0f)
            {
                r.staleW1 = PublishedWeight();
                r.staleHeld = Status().find(" holdPoseActive=1") != std::string::npos;
            }
            continue;
        }
        const int n = std::max(1, int(std::lround(s.seconds / dt)));
        for (int i = 0; i < n; ++i)
        {
            in.psmBlock = s.psm;
            in.locoState = s.loco;
            in.velWorld[2] = s.g > 0.0f ? s.vz - s.g * float(i) * dt : s.vz;
            in.velWorld[1] = s.speed;
            in.move[1] = s.speed > 0.0f ? 1.0f : 0.0f;
            in.onGround = !s.airborne;
            in.distToGround = s.airborne ? -1.0f : -2.0f;
            Feed(in);
            const std::string st = Status();
            r.w.push_back(Field(st, "weight"));
            r.mb.push_back(Field(st, "maskBlend"));
            r.mask.push_back(HexField(st, "maskActive"));
            r.owns.push_back(int(Field(st, "airOwns")));
            r.hold.push_back(st.find(" holdPoseActive=1") != std::string::npos ? 1 : 0);
            r.stepOf.push_back(int(si));
        }
    }
    r.status = Status();
    r.airLands = Field(r.status, "airLands") - lands0;
    r.airEpisodes = Field(r.status, "airEpisodes") - eps0;
    r.diag = g_diag;
    return r;
}
// first feed (s from the first post-exit feed) with w >= aThr, -1 = never
float FirstAt(const std::vector<float>& v, float aThr)
{
    for (size_t i = 0; i < v.size(); ++i)
        if (v[i] >= aThr)
            return float(i + 1) / 60.0f;
    return -1.0f;
}
// after the weight first reached 0.99: lowest weight until the end of step aLastStep (inclusive)
float MinAfterUp(const VxRes& r, int aLastStep)
{
    float m = 9.0f;
    bool up = false;
    for (size_t i = 0; i < r.w.size(); ++i)
    {
        if (r.stepOf[i] > aLastStep)
            break;
        up = up || r.w[i] >= 0.99f;
        if (up)
            m = std::min(m, r.w[i]);
    }
    return up ? m : -1.0f;
}
float MaxOf(const std::vector<float>& v)
{
    float m = 0.0f;
    for (float x : v)
        m = std::max(m, x);
    return m;
}
float WAtStepEnd(const VxRes& r, int aStep)
{
    float w = -1.0f;
    for (size_t i = 0; i < r.w.size(); ++i)
        if (r.stepOf[i] == aStep)
            w = r.w[i];
    return w;
}
// ---- 2026-10-28 bike jump-out: the recorded feed rows (the recorded log
// t 80.157-82.143, raw psmBlock with the weapon / UpperBody / MeleeWeapon fields): ride 0x9, the workspot drop at 80.310 (vz 0),
// the launch at 80.346 (vz 9.70), the vehicle bit until 81.090 (0.78 s), Fall 2 from 81.323, touchdown at 81.674 (vz -11.0).
struct JoRow
{
    float dt, vx, vy, vz, mvx, mvy, yaw, cam;
    int loco;
    uint32_t psm;
    int onGround;
    float dist;
    int psmFall, haveTarget;
    float targetYaw;
};
const JoRow kJoRows[] = {
    {0.0171f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.157
    {0.0169f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.174
    {0.0183f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.193
    {0.0147f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.207
    {0.0166f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.224
    {0.0165f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.240
    {0.0169f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.257
    {0.0164f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.273
    {0.0169f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002009u, 0, -2.000f, 0, 0, 0.00f}, // t 80.290
    {0.0194f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.310
    {0.0190f, 0.0000f, 0.0000f, 0.0000f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.330
    {0.0156f, 0.0000f, 0.0000f, 9.6956f, 0.000f, 0.000f, -8.35f, -8.35f, 0, 0x21002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.346
    {0.0145f, 0.0000f, 0.0000f, 9.4459f, 0.000f, 0.000f, -8.36f, -8.35f, 0, 0x21002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.359
    {0.0144f, 0.0000f, 0.0000f, 9.2136f, 0.000f, 0.000f, -8.36f, -8.35f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.374
    {0.0168f, 0.0000f, 0.0000f, 8.9830f, 0.000f, 0.000f, -8.36f, -8.29f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.391
    {0.0164f, 0.0000f, 0.0000f, 8.7135f, 0.000f, 0.000f, -8.37f, -8.27f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.407
    {0.0166f, 0.0000f, 0.0000f, 8.4504f, 0.000f, 0.000f, -8.37f, -8.26f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.424
    {0.0171f, 0.0000f, 0.0000f, 8.1854f, 0.000f, 0.000f, -8.38f, -8.28f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.441
    {0.0181f, 0.0000f, 0.0000f, 7.9117f, 0.000f, 0.000f, -8.38f, -8.30f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.459
    {0.0151f, 0.0000f, 0.0000f, 7.6217f, 0.000f, 0.000f, -8.39f, -8.34f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.474
    {0.0172f, 0.0000f, 0.0000f, 7.3793f, 0.000f, 0.000f, -8.39f, -8.39f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.491
    {0.0160f, 0.0000f, 0.0000f, 7.1044f, 0.000f, 0.000f, -8.40f, -8.44f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.507
    {0.0167f, 0.0000f, 0.0000f, 6.8482f, 0.000f, 0.000f, -8.40f, -8.49f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.524
    {0.0169f, 0.0000f, 0.0000f, 6.5818f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.541
    {0.0163f, 0.0000f, 0.0000f, 6.3108f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.557
    {0.0181f, 0.0000f, 0.0000f, 6.0497f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.575
    {0.0151f, 0.0000f, 0.0000f, 5.7599f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.590
    {0.0172f, 0.0000f, 0.0000f, 5.5187f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.607
    {0.0162f, 0.0000f, 0.0000f, 5.2443f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.623
    {0.0167f, 0.0000f, 0.0000f, 4.9855f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.640
    {0.0168f, 0.0000f, 0.0000f, 4.7188f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.657
    {0.0165f, 0.0000f, 0.0000f, 4.4496f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.673
    {0.0184f, 0.0000f, 0.0000f, 4.1852f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.692
    {0.0153f, 0.0000f, 0.0000f, 3.8908f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.707
    {0.0164f, 0.0000f, 0.0000f, 3.6466f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.723
    {0.0167f, 0.0000f, 0.0000f, 3.3848f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.740
    {0.0169f, 0.0000f, 0.0000f, 3.1169f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.757
    {0.0166f, 0.0000f, 0.0000f, 2.8471f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.774
    {0.0163f, 0.0000f, 0.0000f, 2.5811f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.790
    {0.0185f, 0.0000f, 0.0000f, 2.3199f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.809
    {0.0154f, 0.0000f, 0.0000f, 2.0246f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.824
    {0.0162f, 0.0000f, 0.0000f, 1.7776f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.840
    {0.0170f, 0.0000f, 0.0000f, 1.5185f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.857
    {0.0163f, 0.0000f, 0.0000f, 1.2457f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.873
    {0.0165f, 0.0000f, 0.0000f, 0.9848f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.890
    {0.0168f, 0.0000f, 0.0000f, 0.7202f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.907
    {0.0177f, 0.0000f, 0.0000f, 0.4508f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.924
    {0.0156f, 0.0000f, 0.0000f, 0.1677f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.940
    {0.0168f, 0.0000f, 0.0000f, -0.0825f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.957
    {0.0166f, 0.0000f, 0.0000f, -0.3507f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.973
    {0.0165f, 0.0000f, 0.0000f, -0.6159f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 80.990
    {0.0169f, 0.0000f, 0.0000f, -0.8803f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.007
    {0.0164f, 0.0000f, 0.0000f, -1.1505f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.023
    {0.0183f, 0.0000f, 0.0000f, -1.4134f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.042
    {0.0151f, 0.0000f, 0.0000f, -1.7060f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.057
    {0.0167f, 0.0000f, 0.0000f, -1.9480f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.073
    {0.0166f, 0.0000f, 0.0000f, -2.2151f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002001u, 0, -2.000f, 0, 0, 0.00f}, // t 81.090
    {0.0169f, 0.0000f, 0.0000f, -2.4801f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.107
    {0.0166f, 0.0000f, 0.0000f, -2.7499f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.772f, 0, 0, 0.00f}, // t 81.123
    {0.0166f, 0.0000f, 0.0000f, -3.0157f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.140
    {0.0182f, 0.0000f, 0.0000f, -3.2816f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.665f, 0, 0, 0.00f}, // t 81.158
    {0.0152f, 0.0000f, 0.0000f, -3.5730f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.173
    {0.0165f, 0.0000f, 0.0000f, -3.8163f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.540f, 0, 0, 0.00f}, // t 81.190
    {0.0168f, 0.0000f, 0.0000f, -4.0801f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.207
    {0.0167f, 0.0000f, 0.0000f, -4.3481f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.401f, 0, 0, 0.00f}, // t 81.223
    {0.0166f, 0.0000f, 0.0000f, -4.6150f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.240
    {0.0185f, 0.0000f, 0.0000f, -4.8805f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.247f, 0, 0, 0.00f}, // t 81.259
    {0.0149f, 0.0000f, 0.0000f, -5.1771f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.273
    {0.0166f, 0.0000f, 0.0000f, -5.4149f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 3.077f, 0, 0, 0.00f}, // t 81.290
    {0.0167f, 0.0000f, 0.0000f, -5.6803f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 0, 0, 0.00f}, // t 81.307
    {0.0167f, 0.0000f, 0.0000f, -5.9475f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 2.889f, 2, 0, 0.00f}, // t 81.323
    {0.0166f, 0.0000f, 0.0000f, -6.2143f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.340
    {0.0169f, 0.0000f, 0.0000f, -6.4803f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 2.684f, 2, 0, 0.00f}, // t 81.357
    {0.0183f, 0.0000f, 0.0000f, -6.7506f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.375
    {0.0149f, 0.0000f, 0.0000f, -7.0429f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 2.447f, 2, 0, 0.00f}, // t 81.390
    {0.0167f, 0.0000f, 0.0000f, -7.2807f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.407
    {0.0169f, 0.0000f, 0.0000f, -7.5487f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 2.217f, 2, 0, 0.00f}, // t 81.424
    {0.0165f, 0.0000f, 0.0000f, -7.8197f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.440
    {0.0167f, 0.0000f, 0.0000f, -8.0834f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 1.954f, 2, 0, 0.00f}, // t 81.457
    {0.0166f, 0.0000f, 0.0000f, -8.3505f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.473
    {0.0181f, 0.0000f, 0.0000f, -8.6153f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 1.674f, 2, 0, 0.00f}, // t 81.492
    {0.0154f, 0.0000f, 0.0000f, -8.9051f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.507
    {0.0166f, 0.0000f, 0.0000f, -9.1508f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 1.374f, 2, 0, 0.00f}, // t 81.523
    {0.0167f, 0.0000f, 0.0000f, -9.4156f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.540
    {0.0168f, 0.0000f, 0.0000f, -9.6830f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 1.057f, 2, 0, 0.00f}, // t 81.557
    {0.0165f, 0.0000f, 0.0000f, -9.9525f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.573
    {0.0181f, 0.0000f, 0.0000f, -10.2162f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 0.721f, 2, 0, 0.00f}, // t 81.592
    {0.0153f, 0.0000f, 0.0000f, -10.5055f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.607
    {0.0166f, 0.0000f, 0.0000f, -10.7497f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, 0.367f, 2, 0, 0.00f}, // t 81.623
    {0.0167f, 0.0000f, 0.0000f, -11.0146f, 0.000f, 0.000f, -8.40f, -8.55f, 0, 0x61002000u, 0, -2.000f, 2, 0, 0.00f}, // t 81.640
    {0.0167f, 0.1239f, 0.1057f, -0.3041f, 0.000f, 0.000f, -8.40f, -8.55f, 1, 0x61002000u, 1, -2.000f, 0, 0, 0.00f}, // t 81.674
    {0.0184f, 0.0000f, 0.0018f, 0.0000f, 0.000f, 0.000f, -8.40f, -8.55f, 1, 0x61002000u, 1, -2.000f, 0, 0, 0.00f}, // t 81.692
};
// r3 landing: a v2 step hook that only records the exit-window / jump-out flags the host hands to the v2 step for each recorded row (it returns false: the v1
// matcher keeps the pose). The V2 side of the seam MotionHost.cpp MotionStepLocked: vehicleExitWindow = window open || jump-out state held.
struct MotionSeenFlags
{
    int window = -1, jumpOut = -1;
};
std::map<int, MotionSeenFlags> g_motionSeen;
int g_motionRow = -1;
bool CaptureMotionStep(const FeedInput& aRaw, uint32_t, float)
{
    g_motionSeen[g_motionRow] = {aRaw.vehicleExitWindow ? 1 : 0, aRaw.vehicleJumpOut ? 1 : 0};
    return false;
}
const MotionHooks kCaptureMotionHooks{&CaptureMotionStep};
// r3b landing: counts the v2 steps (one per Feed that reaches the v2 seam) and the ones that carried the exit window + jump-out flag
int g_landMotionCalls = 0, g_landMotionFlagged = 0;
bool CountMotionStep(const FeedInput& aRaw, uint32_t, float)
{
    ++g_landMotionCalls;
    if (aRaw.vehicleExitWindow && aRaw.vehicleJumpOut)
        ++g_landMotionFlagged;
    return false;
}
const MotionHooks kCountMotionHooks{&CountMotionStep};
// Sleeps aSeconds of wall time calling Tick() every 5 ms (the engine's per-frame tick while the script feed is silent).
void TickFor(float aSeconds)
{
    LARGE_INTEGER f0, fq, fn;
    QueryPerformanceFrequency(&fq);
    QueryPerformanceCounter(&f0);
    do
    {
        Sleep(5);
        Tick();
        QueryPerformanceCounter(&fn);
    } while (float(double(fn.QuadPart - f0.QuadPart) / double(fq.QuadPart)) < aSeconds);
}
// r4 landing: the injected game sim clock (MMDriver SetSimClock). Follow = sim time runs with the wall time, Frozen = pause / menu (sim time stands still while the wall
// time runs), Unavailable = the clock call fails (Tick falls back to the short wall-clock bound).
enum class SimMode
{
    Follow,
    Frozen,
    Unavailable
};
SimMode g_simMode = SimMode::Follow;
double g_simPausedTotal = 0.0, g_simFrozenAt = 0.0;
double WallSeconds()
{
    LARGE_INTEGER fq, fn;
    QueryPerformanceFrequency(&fq);
    QueryPerformanceCounter(&fn);
    return double(fn.QuadPart) / double(fq.QuadPart);
}
bool TestSimClock(double& aSeconds)
{
    if (g_simMode == SimMode::Unavailable)
        return false;
    aSeconds = g_simMode == SimMode::Frozen ? g_simFrozenAt : WallSeconds() - g_simPausedTotal;
    return true;
}
void SimSetMode(SimMode aMode)
{
    if (g_simMode == SimMode::Frozen && aMode != SimMode::Frozen)
        g_simPausedTotal = WallSeconds() - g_simFrozenAt; // the paused wall time never entered the sim time
    else if (g_simMode != SimMode::Frozen && aMode == SimMode::Frozen)
        g_simFrozenAt = WallSeconds() - g_simPausedTotal;
    g_simMode = aMode;
}
struct SimClockScope // SectionVehExit: the injected sim clock follows the wall time unless a check changes the mode
{
    SimClockScope()
    {
        g_simMode = SimMode::Follow;
        g_simPausedTotal = g_simFrozenAt = 0.0;
        SetSimClock(&TestSimClock);
    }
    ~SimClockScope() { SetSimClock(nullptr); }
};
struct JumpOutRes
{
    int launch = -1, land = -1, diagStarts = 0;
    float rideWMax = 0.0f, tUp = -1.0f, wMinAfterUp = -1.0f, mbMax = 0.0f, exitWindows = 0.0f, jumpOuts = 0.0f, flightCuts = 0.0f;
    int epochDelta = 0;       // UTPPVehicleJumpOutEpoch (JumpOutLaunchEpoch) advance over the run: one per launch
    float slewFrames = -1.0f; // status jumpOutSlewFrames at the end of the run (publishes inside the 0.4 s step-cap window)
    float exitFlagFlight = -1.0f, exitFlagBefore = -1.0f, exitFlagEnd = -1.0f; // status jumpOutExit (the v2 hip-fire release flag): over the flight / before the launch / after the window
    bool ownedFlight = false, activeAtEnd = true;
    std::vector<float> flagRow, windowRow; // status jumpOutExit / exitWindow after every fed row (recorded rows, then the trailing ground rows)
    std::string Line() const
    {
        char b[340];
        std::snprintf(b, sizeof b, "launch row %d land row %d ride w max %.2f w>=0.99 %.3f s after the launch, w min after %.2f, air owns the flight %d, "
                      "maskBlend max %.2f, exit windows %.0f, EXITDIAG starts %d, jumpOuts %.0f, mount cuts after the ride %.0f, active at end %d",
                      launch, land, rideWMax, tUp, wMinAfterUp, ownedFlight ? 1 : 0, mbMax, exitWindows, diagStarts, jumpOuts, flightCuts,
                      activeAtEnd ? 1 : 0);
        return b;
    }
};
// 2 s standing, mount (vehicle, then vehicle + workspot, as Run), the recorded rows, then 0.5 s standing (the last row repeated).
// mm.weaponUpper as shipped and as in the recording (CET default false, log 'MMWEAPON on=0'): the drawn weapon in the rows does
// not move the upper body to the game; restored to the driver default afterwards for the later sections.
// aHook(i) runs before recorded row i (the game re-arms with the same rig around unmounts: review fix vehdll MMDriver:2612).
// aMod(i, in) edits the feed of row i after the recorded values were applied (r3 landing: a one-feed false ground contact in the flight).
JumpOutRes RunRecordedJumpOut(const std::function<void(int)>& aHook = {}, const std::function<void(int, FeedInput&)>& aMod = {})
{
    JumpOutRes r;
    SetWeaponUpper(false);
    struct Restore
    {
        ~Restore() { SetWeaponUpper(true); }
    } restore;
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = 1.0f / 60.0f;
    in.cameraYawDeg = -8.35f;
    in.playerYawDeg = -8.35f;
    in.haveAir = true;
    in.onGround = true;
    for (int i = 0; i < 120; ++i)
        Feed(in);
    const float exit0 = Field(Status(), "exitWindows"), jump0 = Field(Status(), "jumpOuts");
    const int epoch0 = JumpOutLaunchEpoch();
    float cuts0 = 0.0f;
    g_diag.clear();
    in.psmBlock = kVeh;
    Feed(in);
    in.locoState = 0;
    in.psmBlock = kVeh | kWs;
    for (int i = 0; i < 30; ++i)
    {
        Feed(in);
        r.rideWMax = std::max(r.rideWMax, Field(Status(), "weight"));
    }
    cuts0 = Field(Status(), "mountCuts");
    const int n = int(std::size(kJoRows));
    std::vector<float> w, mb, t;
    std::vector<int> owns;
    float clock = 0.0f;
    for (int i = 0; i < n + 30; ++i)
    {
        const JoRow& row = kJoRows[std::min(i, n - 1)];
        in.dt = row.dt;
        in.velWorld[0] = i < n ? row.vx : 0.0f;
        in.velWorld[1] = i < n ? row.vy : 0.0f;
        in.velWorld[2] = i < n ? row.vz : 0.0f;
        in.move[0] = row.mvx;
        in.move[1] = row.mvy;
        in.playerYawDeg = row.yaw;
        in.cameraYawDeg = row.cam;
        in.locoState = row.loco;
        in.psmBlock = row.psm;
        in.onGround = row.onGround != 0;
        in.distToGround = row.dist;
        in.psmFall = row.psmFall;
        in.haveTarget = row.haveTarget != 0;
        in.targetYawDeg = row.targetYaw;
        if (aHook && i < n)
            aHook(i);
        if (aMod)
            aMod(i, in);
        Feed(in);
        clock += row.dt;
        const std::string st = Status();
        r.flagRow.push_back(Field(st, "jumpOutExit"));
        r.windowRow.push_back(Field(st, "exitWindow"));
        if ((row.psm & kWs) != 0)
            r.rideWMax = std::max(r.rideWMax, Field(st, "weight"));
        if (r.launch < 0 && i < n && row.vz >= 3.0f)
            r.launch = i;
        if (r.launch >= 0 && r.land < 0 && i < n && row.onGround != 0)
            r.land = i;
        if (r.launch < 0 && i < n && row.vz < 3.0f)
            r.exitFlagBefore = std::max(r.exitFlagBefore, Field(st, "jumpOutExit"));
        if (r.launch >= 0 && i == r.launch + 20)
            r.exitFlagFlight = Field(st, "jumpOutExit");
        w.push_back(Field(st, "weight"));
        mb.push_back(Field(st, "maskBlend"));
        owns.push_back(int(Field(st, "airOwns")));
        t.push_back(clock);
    }
    if (r.launch >= 0)
    {
        for (size_t i = size_t(r.launch); i < w.size(); ++i)
        {
            if (r.tUp < 0.0f && w[i] >= 0.99f)
            {
                r.tUp = t[i] - t[size_t(r.launch)] + kJoRows[r.launch].dt;
                r.wMinAfterUp = w[i];
            }
            if (r.tUp >= 0.0f)
                r.wMinAfterUp = std::min(r.wMinAfterUp, w[i]);
            r.mbMax = std::max(r.mbMax, mb[i]);
        }
        r.ownedFlight = r.land > r.launch + 1;
        for (int i = r.launch + 1; i < r.land && i < int(owns.size()); ++i)
            r.ownedFlight = r.ownedFlight && owns[size_t(i)] == 1;
    }
    const std::string st = Status();
    r.exitWindows = Field(st, "exitWindows") - exit0;
    r.exitFlagEnd = Field(st, "jumpOutExit");
    r.slewFrames = Field(st, "jumpOutSlewFrames");
    r.jumpOuts = Field(st, "jumpOuts") - jump0;
    r.epochDelta = JumpOutLaunchEpoch() - epoch0;
    r.flightCuts = Field(st, "mountCuts") - cuts0;
    r.activeAtEnd = st.find(" jumpOutActive=1") != std::string::npos;
    for (const std::string& d : g_diag)
        r.diagStarts += d.find("UTPP EXITDIAG +0.00s ") != std::string::npos ? 1 : 0;
    return r;
}
// ---- P2 helpers: simulated animation thread (game local-pose records, kGameBoneStride, FkSide) at real 60 Hz
namespace vti = utpp::mm::integ;
namespace vtr = utpp::mm::vehtrans;
int64_t QpcNow()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}
float QpcSeconds(int64_t aTicks)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return float(double(aTicks) / double(f.QuadPart));
}
void SpinUntil(int64_t aTicks)
{
    while (QpcNow() < aTicks)
        ;
}
int64_t FrameTicks()
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return f.QuadPart / 60;
}
struct Recs
{
    std::vector<uint8_t> buf;
    explicit Recs(uint32_t n) : buf(size_t(n) * vti::kGameBoneStride, 0) {}
    float* At(uint32_t g) { return reinterpret_cast<float*>(buf.data() + size_t(g) * vti::kGameBoneStride); }
    uint32_t Count() const { return uint32_t(buf.size() / vti::kGameBoneStride); }
};
// game records from a snapshot pose (db bone b -> game index), every other record identity; aRotDeg(b) rotates bone b about X,
// aDz(b) moves it along Z (local)
template <typename RotDeg, typename Dz>
void FillRecs(Recs& r, const vti::PoseSnapshot& s, RotDeg&& aRotDeg, Dz&& aDz)
{
    for (uint32_t g = 0; g < r.Count(); ++g)
    {
        float* x = r.At(g);
        x[0] = x[1] = x[2] = 0.0f;
        x[vti::kGameRotOffsetFloats + 0] = x[vti::kGameRotOffsetFloats + 1] = x[vti::kGameRotOffsetFloats + 2] = 0.0f;
        x[vti::kGameRotOffsetFloats + 3] = 1.0f;
        x[8] = x[9] = x[10] = 1.0f;
    }
    for (uint32_t b = 0; b < s.numBones && b < vti::kMaxMMBones; ++b)
    {
        const int32_t g = s.gameIndex[b];
        if (g < 0 || uint32_t(g) >= r.Count())
            continue;
        const float* m = &s.pose[size_t(b) * 7];
        float* x = r.At(uint32_t(g));
        x[0] = m[0];
        x[1] = m[1];
        x[2] = m[2] + aDz(b);
        const float h = 0.5f * aRotDeg(b) * kDegT;
        const float qa[4] = {std::sin(h), 0.0f, 0.0f, std::cos(h)};
        float o[4];
        vtr::detail::QMul(qa, m + 3, o);
        vtr::detail::QNormalize(o);
        for (int k = 0; k < 4; ++k)
            x[vti::kGameRotOffsetFloats + k] = o[k];
    }
}
float RecAngleDeg(const float* aA, const float* aB)
{
    return vtr::detail::QAngle(aA + vti::kGameRotOffsetFloats, aB + vti::kGameRotOffsetFloats) / kDegT;
}
FeedInput MountFeed()
{
    FeedInput in;
    in.tppActive = true;
    in.locoState = 1;
    in.dt = 1.0f / 60.0f;
    in.cameraYawDeg = -82.0f;
    in.playerYawDeg = 0.0f;
    in.haveAir = true;
    in.onGround = true;
    return in;
}
bool ReadSnap(vti::PoseSnapshot& aOut)
{
    return utpp::g_mmPose.Read(aOut);
}
struct MountRes
{
    bool ok = false;
    float diffDeg = 0.0f, cutJumpDeg = -1.0f, maxJumpDeg = 0.0f, maxDt = 0.0f;
    int writtenFrames = 0, writtenLate = 0, lateFrames = 0;
    float rootDist = -1.0f, rootYaw = 0.0f;
    bool rootLate = false;
    std::string Line() const
    {
        char b[260];
        std::snprintf(b, sizeof b, "ok %d diff %.1f deg cut jump %.1f max jump %.1f deg written %d (late %d) root %.2f m %.1f deg late %d maxDt %.3f s "
                      "skipped slow frames %d",
                      ok ? 1 : 0, diffDeg, cutJumpDeg, maxJumpDeg, writtenFrames, writtenLate, rootDist, rootYaw, rootLate ? 1 : 0, maxDt, lateFrames);
        return b;
    }
};
// Stand 2 s (MM visible), mount: the game entry pose = MM + 50 deg on the upper body / 10 deg on the legs; the entity jumps 0.8 m
// and 30 deg on the 3rd vehicle sample (workspot alignment). Visible pose per frame = the records after FkSide (the published MM
// weight is 0 from the cut).
MountRes RunMount(uint64_t aRig, uint32_t aCount)
{
    MountRes r;
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones)
        return r;
    Recs game(aCount), prev(aCount);
    const auto upperDeg = [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? 50.0f : 10.0f; };
    FillRecs(prev, a, [](uint32_t) { return 0.0f; }, [](uint32_t) { return 0.0f; }); // visible before the cut = MM
    {
        Recs b0(aCount);
        FillRecs(b0, a, upperDeg, [](uint32_t) { return 0.0f; });
        for (uint32_t b = a.firstBone; b < a.numBones; ++b)
            if (a.gameIndex[b] >= 0 && vti::SnapshotUpperBone(a, b))
                r.diffDeg = std::max(r.diffDeg, RecAngleDeg(b0.At(uint32_t(a.gameIndex[b])), prev.At(uint32_t(a.gameIndex[b]))));
    }
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow();
    int64_t cut = 0, last = 0;
    for (int i = 0; i < 40; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        if (i >= 2)
        {
            in.capsulePos[0] = 0.8f;
            in.playerYawDeg = 30.0f;
        }
        Feed(in);
        const int64_t now = QpcNow();
        if (i == 0)
            cut = now;
        const float t = QpcSeconds(now - cut);
        float ro[3] = {};
        if (i == 2 && vtr::RootOffset(now, ro))
        {
            r.rootDist = std::hypot(ro[0], ro[1]);
            r.rootYaw = ro[2];
        }
        if (t > 0.32f && vtr::RootOffset(now, ro))
            r.rootLate = true;
        FillRecs(game, a, upperDeg, [](uint32_t) { return 0.0f; });
        const uint32_t n = vtr::FkSide(game.buf.data(), aCount, aRig, now, true);
        r.writtenFrames += n > 0 ? 1 : 0;
        r.writtenLate += (n > 0 && t > 0.36f) ? 1 : 0; // the root offset decays over 0.30 s from the jump sample (+2 frames)
        const float dtF = last ? QpcSeconds(now - last) : 0.0f;
        last = now;
        r.maxDt = std::max(r.maxDt, dtF);
        // the Hips record carries the root offset (the entity yaw jump compensated): compare it in the world (entity yaw x local)
        {
            float* h = game.At(uint32_t(a.gameIndex[a.firstBone]));
            const float hy = 0.5f * in.playerYawDeg * kDegT;
            const float qz[4] = {0.0f, 0.0f, std::sin(hy), std::cos(hy)};
            float o[4];
            vtr::detail::QMul(qz, h + vti::kGameRotOffsetFloats, o);
            for (int k = 0; k < 4; ++k)
                h[vti::kGameRotOffsetFloats + k] = o[k];
        }
        float jump = 0.0f;
        for (uint32_t b = a.firstBone; b < a.numBones; ++b)
            if (a.gameIndex[b] >= 0)
                jump = std::max(jump, RecAngleDeg(game.At(uint32_t(a.gameIndex[b])), prev.At(uint32_t(a.gameIndex[b]))));
        if (i == 0)
            r.cutJumpDeg = jump;
        if (dtF <= 0.025f)
            r.maxJumpDeg = std::max(r.maxJumpDeg, jump);
        else
            ++r.lateFrames; // a descheduled test thread: the per-frame step is not a 60 Hz step
        prev.buf = game.buf;
    }
    r.ok = true;
    return r;
}
struct ExitHandRes
{
    bool ok = false;
    float wEdge = -1.0f, minDistCm = 1e9f, handed = 0.0f, wMin = 2.0f, kept = 0.0f;
    float jumpOutFlag = -1.0f; // status jumpOutExit after the exit window opened (a normal unmount: 0)
    float slewFrames = -1.0f;  // status jumpOutSlewFrames (a normal unmount: 0)
    int nearSquat = 0, frames = 0;
    std::string Line() const
    {
        char b[260];
        std::snprintf(b, sizeof b, "ok %d w edge %.2f frames %d near squat %d min distance to the squat pelvis %.1f cm handed %.0f w min %.2f kept %.0f",
                      ok ? 1 : 0, wEdge, frames, nearSquat, minDistCm, handed, wMin, kept);
        return b;
    }
};
// Mount, ride 0.6 s with the game pose = the standing exit pose S (= MM), unmount: the live game pose becomes the squat Q (pelvis
// 0.40 m lower), the exit workspot is latched (CAR-1 T1). Visible pelvis height = Q + (published - Q) x weight (lower body).
// aHook(phase, i) runs before the feed (phase 0 = ride feed i of 36, phase 1 = exit feed i of 30): review fix re-arm cases.
ExitHandRes RunExitHandIn(uint64_t aRig, uint32_t aCount, const std::function<void(int, int)>& aHook = {})
{
    ExitHandRes r;
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a, s;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones || a.gameIndex[a.firstBone] < 0)
        return r;
    const uint32_t hips = a.firstBone;
    const float squatZ = a.pose[size_t(hips) * 7 + 2] - 0.40f;
    Recs game(aCount);
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow();
    for (int i = 0; i < 36; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        if (i == 35)
        {
            in.psmBlock = kVeh; // game: psm=0x1 loco=1 for one sample before the end
            in.locoState = 1;
        }
        if (aHook)
            aHook(0, i);
        Feed(in);
        FillRecs(game, a, [](uint32_t) { return 0.0f; }, [](uint32_t) { return 0.0f; }); // standing exit pose S (= MM)
        vtr::FkSide(game.buf.data(), aCount, aRig, QpcNow(), true);
    }
    int64_t edge = 0;
    for (int i = 0; i < 30; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = kWs | Det(0);
        in.locoState = 0;
        if (aHook)
            aHook(1, i);
        // the visible frame right after a re-arm (OnArmed publishes before the next feed): the published weight at that moment
        if (aHook && i > 0 && ReadSnap(s))
            r.wMin = std::min(r.wMin, utpp::g_hot.mmActive.load() ? s.weight : 0.0f);
        Feed(in);
        const int64_t now = QpcNow();
        if (i == 0)
            edge = now;
        if (!ReadSnap(s))
            continue;
        const float w = utpp::g_hot.mmActive.load() ? s.weight : 0.0f;
        if (i == 0)
            r.wEdge = w;
        if (QpcSeconds(now - edge) <= 0.30f)
            r.wMin = std::min(r.wMin, w);
        FillRecs(game, a, [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? 60.0f : 30.0f; },
                 [&](uint32_t b) { return b == hips ? -0.40f : 0.0f; }); // squat Q
        const uint32_t n = vtr::FkSide(game.buf.data(), aCount, aRig, now, true);
        float visZ = game.At(uint32_t(a.gameIndex[hips]))[2];
        if (n == 0)
            visZ = visZ + (s.pose[size_t(hips) * 7 + 2] - visZ) * std::min(1.0f, w);
        if (QpcSeconds(now - edge) <= 0.30f)
        {
            ++r.frames;
            const float d = std::fabs(visZ - squatZ);
            r.minDistCm = std::min(r.minDistCm, d * 100.0f);
            r.nearSquat += d < 0.10f ? 1 : 0;
        }
    }
    {
        const std::string st = Status();
        const size_t p = st.find(" vehTrans=");
        if (p != std::string::npos)
        {
            const size_t sl = st.find('/', p);
            r.handed = sl == std::string::npos ? 0.0f : float(std::atof(st.c_str() + sl + 1));
        }
        r.kept = Field(st, "vehTransKept");
        r.jumpOutFlag = Field(st, "jumpOutExit");
        r.slewFrames = Field(st, "jumpOutSlewFrames");
    }
    r.ok = true;
    return r;
}
// vehTrans=mounts/handed/noCapture/aborts of the status line
std::array<float, 4> VtCounts()
{
    std::array<float, 4> c = {-1.0f, -1.0f, -1.0f, -1.0f};
    const std::string st = Status();
    const size_t p = st.find(" vehTrans=");
    if (p != std::string::npos)
    {
        const char* s = st.c_str() + p + std::strlen(" vehTrans=");
        for (int k = 0; k < 4; ++k)
        {
            char* e = nullptr;
            c[size_t(k)] = float(std::strtoull(s, &e, 10));
            if (e == s || (k < 3 && *e != '/'))
                return {-1.0f, -1.0f, -1.0f, -1.0f};
            s = e + 1;
        }
    }
    return c;
}
// 2026-10-28 bike jump-out + exit hand-in: the recorded rows (kJoRows: ride, workspot drop, launch 2 rows later) with the simulated
// animation thread (FkSide every frame at 60 Hz). The game pose while riding is the standing exit pose S (= MM); from the workspot
// drop the live game pose is the flight pose Q (upper body 60 deg / legs 30 deg off, pelvis 0.40 m lower).
struct JumpOutHandRes
{
    bool ok = false;
    float wEdge = -1.0f, wMin = 2.0f, handed = -1.0f, noCapture = -1.0f, launchWorstDeg = 0.0f;
    float maxVisStepCm = 0.0f, tFull = -1.0f; // visible pelvis height: largest per-frame step from the edge; s from the edge to MM weight >= 0.99
    float maxVisRotDeg = 0.0f, handInDevDeg = 0.0f; // visible rotation: largest per-frame step from the edge / distance to the game pose at the hand-in start
    int launchRow = -1, frames = 0;
    // 2026-10-28 11-49-37 re-arm in the flight (aStallFrom >= 0): the swap itself (MM -> the game pose of the new object, not ours), the
    // largest visible step from the re-arm frame on, the published weight from the re-arm on, re-arm hold frames, jump-out re-arms
    float swapJumpDeg = 0.0f, rearmMaxStepDeg = 0.0f, wMinAfterRearm = 2.0f, holdFrames = 0.0f, rearms = -1.0f;
    std::string RearmLine() const
    {
        char b[220];
        std::snprintf(b, sizeof b, "swap jump %.1f deg (game, not ours), visible step max from the re-arm %.4f deg, w min from the re-arm %.2f, "
                      "re-arm hold frames %.0f, jumpOutRearms %.0f", swapJumpDeg, rearmMaxStepDeg, wMinAfterRearm, holdFrames, rearms);
        return b;
    }
    std::string Line() const
    {
        char b[420];
        std::snprintf(b, sizeof b, "ok %d launch row %d w on the edge feed %.2f w min within 0.3 s %.2f frames %d vehTrans handed %.0f no capture %.0f max visible pelvis step %.1f cm, w >= 0.99 after %.3f s, visible rotation step max %.1f deg, hand-in start vs the game pose %.1f deg",
                      ok ? 1 : 0, launchRow, wEdge, wMin, frames, handed, noCapture, maxVisStepCm, tFull, maxVisRotDeg, handInDevDeg);
        return b;
    }
};
// aStallFrom >= 0 (frames after the launch frame): from there the animation thread does not reach the visible object for aStallLen
// frames (the game replaced it: the player sees the live game pose), then aRearm() runs before the feed (the same-rig re-arm).
JumpOutHandRes RunJumpOutHandIn(uint64_t aRig, uint32_t aCount, bool aRideCapture = true, int aStallFrom = -1, int aStallLen = 0,
                                const std::function<void()>& aRearm = {})
{
    JumpOutHandRes r;
    const uint64_t holdBase = vtr::g_counters.rearmHoldFrames.load();
    int sinceLaunch = -1;
    SetWeaponUpper(false);
    struct Restore
    {
        ~Restore() { SetWeaponUpper(true); }
    } restore;
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a, s;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones || a.gameIndex[a.firstBone] < 0)
        return r;
    const uint32_t hips = a.firstBone;
    // with ride captures the ride is seated: the bike seat Hips 0.08 m (below vehtrans::kStandHipsZ, the exit latch never arms; the
    // seated twin in MMVehicleTransitionTest), the flight pose Q 0.40 m below the seat. Without ride captures nothing reaches the
    // latch before the launch: S stays the MM pose, Q 0.40 m below it (the live capture's 40 cm hand-in offset).
    const float seatDz = aRideCapture ? 0.08f - a.pose[size_t(hips) * 7 + 2] : 0.0f;
    Recs game(aCount);
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow();
    for (int i = 0; i < 36; ++i) // ride: the game pose = the ride pose S (the MM pose, the Hips at the seat height with ride captures)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        Feed(in);
        FillRecs(game, a, [](uint32_t) { return 0.0f; }, [&](uint32_t b) { return b == hips ? seatDz : 0.0f; });
        if (aRideCapture) // false: the animation thread delivered no ride capture (the exit finds none)
            vtr::FkSide(game.buf.data(), aCount, aRig, QpcNow(), true);
    }
    const int n = int(std::size(kJoRows));
    float prevVis = 1e9f;
    std::vector<std::array<float, 4>> visQ(vti::kMaxMMBones);
    bool visHave = false;
    int64_t edge = 0;
    bool launched = false;
    for (int i = 9; i < n && r.frames < 40; ++i) // from the workspot drop
    {
        const JoRow& row = kJoRows[i];
        SpinUntil(next);
        next += frame;
        const int pf = launched ? ++sinceLaunch : -1; // frames after the launch frame
        const bool stallRun = aStallFrom >= 0;
        const bool stalled = stallRun && pf >= aStallFrom && pf < aStallFrom + aStallLen;
        const bool afterRearm = stallRun && pf >= aStallFrom + aStallLen;
        if (stallRun && aRearm && pf == aStallFrom + aStallLen)
            aRearm();
        in.dt = 1.0f / 60.0f;
        in.velWorld[2] = row.vz;
        in.locoState = row.loco;
        in.psmBlock = row.psm;
        in.onGround = row.onGround != 0;
        in.distToGround = row.dist;
        in.psmFall = row.psmFall;
        Feed(in);
        const int64_t now = QpcNow();
        const bool fired = Field(Status(), "jumpOuts") >= 1.0f;
        if (!launched && fired)
        {
            launched = true;
            sinceLaunch = 0;
            edge = now;
            r.launchRow = i;
            if (ReadSnap(s))
                r.wEdge = utpp::g_hot.mmActive.load() ? s.weight : 0.0f;
        }
        if (launched && ReadSnap(s))
        {
            ++r.frames;
            if (QpcSeconds(now - edge) <= 0.30f)
                r.wMin = std::min(r.wMin, utpp::g_hot.mmActive.load() ? s.weight : 0.0f);
        }
        // the live game pose: S while the workspot runs, the flight pose Q from the drop
        if ((row.psm & kWs) != 0)
            FillRecs(game, a, [](uint32_t) { return 0.0f; }, [&](uint32_t b) { return b == hips ? seatDz : 0.0f; });
        else
            FillRecs(game, a, [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? 60.0f : 30.0f; },
                     [&](uint32_t b) { return b == hips ? seatDz - 0.40f : 0.0f; });
        // re-arm run: the animation frame runs before the visible pose is taken (its return value decides whether MM is blended:
        // non-zero = the re-arm hold keeps the game pose); a stalled frame never reaches the visible object (the game pose is shown)
        bool showGame = false;
        if (stallRun && launched)
        {
            if (stalled)
                showGame = true;
            else
                showGame = vtr::FkSide(game.buf.data(), aCount, aRig, now, true) != 0;
        }
        if (launched && afterRearm && ReadSnap(s))
            r.wMinAfterRearm = std::min(r.wMinAfterRearm, utpp::g_hot.mmActive.load() ? s.weight : 0.0f);
        if (launched && ReadSnap(s)) // visible pelvis height = the live game record + (published MM + hand-in offset - game record) x weight
        {
            const float w = showGame ? 0.0f : (utpp::g_hot.mmActive.load() ? s.weight : 0.0f);
            const float q = game.At(uint32_t(a.gameIndex[hips]))[2];
            const float vis = q + (s.pose[size_t(hips) * 7 + 2] - q) * w;
            if (prevVis < 1e8f)
                r.maxVisStepCm = std::max(r.maxVisStepCm, std::fabs(vis - prevVis) * 100.0f);
            prevVis = vis;
            // visible ROTATION as the player sees it (BlendIntoRecords: the record quaternion blended to the published snapshot quaternion by the
            // weight), per bone: largest step per frame from the edge, and at the first frame with MM at full weight (the hand-in start) the
            // distance to the live game record (the hand-in must continue the game pose the player saw, not return to the MM pose)
            for (uint32_t b = a.firstBone; b < a.numBones; ++b)
            {
                const int32_t gi = a.gameIndex[b];
                if (gi < 0 || uint32_t(gi) >= aCount || b >= visQ.size())
                    continue;
                const float* rq = game.At(uint32_t(gi)) + vti::kGameRotOffsetFloats;
                const float* sq = &s.pose[size_t(b) * 7 + 3];
                const float sg = (rq[0] * sq[0] + rq[1] * sq[1] + rq[2] * sq[2] + rq[3] * sq[3]) < 0.0f ? -1.0f : 1.0f;
                std::array<float, 4> v{};
                float n2 = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    v[size_t(k)] = rq[k] * (1.0f - w) + sg * sq[k] * w;
                    n2 += v[size_t(k)] * v[size_t(k)];
                }
                for (int k = 0; k < 4; ++k)
                    v[size_t(k)] /= std::sqrt(std::fmax(n2, 1e-12f));
                if (visHave)
                {
                    const float stepDeg = vtr::detail::QAngle(visQ[b].data(), v.data()) / kDegT;
                    r.maxVisRotDeg = std::max(r.maxVisRotDeg, stepDeg);
                    if (stallRun && pf == aStallFrom)
                        r.swapJumpDeg = std::max(r.swapJumpDeg, stepDeg);
                    if (afterRearm)
                        r.rearmMaxStepDeg = std::max(r.rearmMaxStepDeg, stepDeg);
                }
                visQ[b] = v;
                if (r.tFull < 0.0f && w >= 0.99f)
                    r.handInDevDeg = std::max(r.handInDevDeg, vtr::detail::QAngle(rq, v.data()) / kDegT);
            }
            visHave = true;
            if (r.tFull < 0.0f && w >= 0.99f)
                r.tFull = QpcSeconds(now - edge);
        }
        if ((aRideCapture || launched) && !(stallRun && launched)) // without ride captures the animation thread delivers frames only from the launch feed on
            vtr::FkSide(game.buf.data(), aCount, aRig, now, true);
    }
    const auto vt = VtCounts();
    r.handed = vt[1];
    r.noCapture = vt[2];
    r.holdFrames = float(vtr::g_counters.rearmHoldFrames.load() - holdBase);
    r.rearms = Field(Status(), "jumpOutRearms");
    r.ok = launched;
    return r;
}
// r6 launch (game log 2026-11-04 06:07:27, jumpout_stand): the jump-out edge as the game runs it. The ride frames are DENIED (the vehicle hold withdraws
// the pose-write permission for the whole ride; the mounted pop filter and the ride capture run on them), the game swaps the seated pose for its flight
// pose Q at the workspot drop 2 rows (36 ms, recorded 43 ms) before the launch edge, and the permission returns only after the edge (recorded 11 ms).
// RunJumpOutHandIn above never denies and measures the records before FkSide, so it cannot see the animation frame between the edge and the
// permission. Here every frame is measured after FkSide: the visible pose = the records the call left (a denied frame, or a held / written one), or
// on an allowed frame the published snapshot blended onto them by its weight (BlendIntoRecords). aNormal: an ordinary unmount (no launch) of the same
// ride for the bit-identity control.
struct JumpOutDeniedRes
{
    bool ok = false;
    int launchRow = -1, frames = 0, edgeFrame = -1, lateFrames = 0;
    bool lateEdge = false;
    float worstStepDeg = 0.0f;  // largest visible bone rotation step per frame, from the workspot drop to 0.3 s after the edge
    float worstEdgeDeg = 0.0f;  // ... on the edge frame and the denied frame after it
    float worst50Deg = 0.0f;    // largest visible bone rotation change per 50 ms (3 frames) with the window end in the first 0.2 s after the edge
    float worstPelvisCm = 0.0f; // largest visible pelvis height step per frame from the edge frame on
    float handed = -1.0f, pins = 0.0f, holdFrames = 0.0f, holdEnds = 0.0f, wMin = 2.0f;
    std::string Line() const
    {
        char b[420];
        std::snprintf(b, sizeof b, "ok %d launch row %d frames %d (late %d) visible step max %.1f deg (edge + denied frame %.1f), worst per 50 ms %.1f deg, pelvis step %.1f cm, "
                      "handed %.0f pins %.0f hold frames %.0f hold ends %.0f w min %.2f",
                      ok ? 1 : 0, launchRow, frames, lateFrames, worstStepDeg, worstEdgeDeg, worst50Deg, worstPelvisCm, handed, pins, holdFrames, holdEnds, wMin);
        return b;
    }
};
JumpOutDeniedRes RunJumpOutDenied(uint64_t aRig, uint32_t aCount, bool aNormal = false, int aDeniedAfterEdge = 1)
{
    JumpOutDeniedRes r;
    const auto k0 = vtr::SnapCounters();
    SetWeaponUpper(false);
    struct Restore
    {
        ~Restore() { SetWeaponUpper(true); }
    } restore;
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a, s;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones || a.gameIndex[a.firstBone] < 0)
        return r;
    const uint32_t hips = a.firstBone;
    const float seatDz = 0.08f - a.pose[size_t(hips) * 7 + 2]; // the seated ride: Hips 0.08 m (below kStandHipsZ: the exit latch never arms)
    Recs game(aCount);
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow();
    std::vector<std::array<std::array<float, 4>, vti::kMaxMMBones>> hist; // the visible rotations of the last frames
    std::vector<float> histZ;
    std::vector<int> histEdge;
    int sinceEdge = -1;
    bool launched = false;
    const auto seated = [&] { FillRecs(game, a, [](uint32_t) { return 0.0f; }, [&](uint32_t b) { return b == hips ? seatDz : 0.0f; }); };
    const auto flight = [&] {
        FillRecs(game, a, [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? 60.0f : 30.0f; }, [&](uint32_t b) { return b == hips ? seatDz - 0.40f : 0.0f; });
    };
    // one animation frame: FkSide on the live records, then the visible pose
    int64_t lastNow = 0;
    const auto animFrame = [&](bool aAllows) {
        const int64_t now = QpcNow();
        const bool late = lastNow != 0 && QpcSeconds(now - lastNow) > 0.025f; // a descheduled test thread: not a 60 Hz step
        lastNow = now;
        if (late)
        {
            ++r.lateFrames;
            if (r.frames > 0 && sinceEdge <= aDeniedAfterEdge + 2) // (the drop frames before the edge: sinceEdge -1)
                r.lateEdge = true; // the frames around the edge were not 60 Hz: the result says nothing about the edge
        }
        const uint32_t n = vtr::FkSide(game.buf.data(), aCount, aRig, now, aAllows);
        const bool blend = aAllows && n == 0 && ReadSnap(s) && utpp::g_hot.mmActive.load();
        const float w = blend ? s.weight : 0.0f;
        if (aAllows && launched)
            r.wMin = std::min(r.wMin, w);
        std::array<std::array<float, 4>, vti::kMaxMMBones> vis{};
        for (uint32_t b = a.firstBone; b < a.numBones && b < vti::kMaxMMBones; ++b)
        {
            const int32_t gi = a.gameIndex[b];
            if (gi < 0 || uint32_t(gi) >= aCount)
                continue;
            const float* rq = game.At(uint32_t(gi)) + vti::kGameRotOffsetFloats;
            std::array<float, 4> v{rq[0], rq[1], rq[2], rq[3]};
            if (w > 0.0f)
            {
                const float* sq = &s.pose[size_t(b) * 7 + 3];
                const float sg = (rq[0] * sq[0] + rq[1] * sq[1] + rq[2] * sq[2] + rq[3] * sq[3]) < 0.0f ? -1.0f : 1.0f;
                for (int k = 0; k < 4; ++k)
                    v[size_t(k)] = rq[k] * (1.0f - w) + sg * sq[k] * w;
            }
            float n2 = 0.0f;
            for (float x : v)
                n2 += x * x;
            for (float& x : v)
                x /= std::sqrt(std::fmax(n2, 1e-12f));
            vis[b] = v;
        }
        float z = game.At(uint32_t(a.gameIndex[hips]))[2];
        if (w > 0.0f)
            z += (s.pose[size_t(hips) * 7 + 2] - z) * w;
        const bool measure = !hist.empty() && !late;
        if (measure)
        {
            float step = 0.0f;
            for (uint32_t b = a.firstBone; b < a.numBones && b < vti::kMaxMMBones; ++b)
                if (a.gameIndex[b] >= 0)
                    step = std::max(step, vtr::detail::QAngle(hist.back()[b].data(), vis[b].data()) / kDegT);
            r.worstStepDeg = std::max(r.worstStepDeg, step);
            if (sinceEdge >= 0) // (from the edge frame on; the drop frames before it carry the fixture's own 0.4 m pelvis step)
                r.worstPelvisCm = std::max(r.worstPelvisCm, std::fabs(z - histZ.back()) * 100.0f);
            if (sinceEdge >= 0 && sinceEdge <= aDeniedAfterEdge)
                r.worstEdgeDeg = std::max(r.worstEdgeDeg, step);
            if (hist.size() >= 3 && sinceEdge >= 0 && sinceEdge <= 12)
            {
                float d50 = 0.0f;
                for (uint32_t b = a.firstBone; b < a.numBones && b < vti::kMaxMMBones; ++b)
                    if (a.gameIndex[b] >= 0)
                        d50 = std::max(d50, vtr::detail::QAngle(hist[hist.size() - 3][b].data(), vis[b].data()) / kDegT);
                r.worst50Deg = std::max(r.worst50Deg, d50);
            }
        }
        hist.push_back(vis);
        histZ.push_back(z);
        ++r.frames;
    };
    for (int i = 0; i < 130; ++i) // ride: denied frames, the seated game pose (the mounted pop filter and the ride capture run on them; 2.2 s: the mount transition, <= kGlideEndSeconds 1.5 s, is over)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        Feed(in);
        seated();
        animFrame(false);
    }
    r.frames = 0;
    r.lateFrames = 0;
    {
        const auto lastVis = hist.back();
        const float lastZ = histZ.back();
        hist.assign(1, lastVis);
        histZ.assign(1, lastZ);
    }
    if (aNormal)
    {
        // an ordinary unmount: the workspot drops (the game flight pose), the vehicle bit falls on the ground three frames later, no launch; the
        // permission returns one frame after the edge as in the recording
        int edge = -1;
        for (int i = 0; i < 40; ++i)
        {
            SpinUntil(next);
            next += frame;
            in.psmBlock = i < 3 ? kVeh : 0u;
            in.locoState = i < 3 ? 0 : 1;
            in.velWorld[2] = 0.0f;
            Feed(in);
            if (i == 3)
                edge = r.frames;
            sinceEdge = edge >= 0 ? r.frames - edge : -1;
            flight();
            animFrame(i >= 3 + aDeniedAfterEdge);
        }
        r.edgeFrame = edge;
    }
    else
    {
        const int n = int(std::size(kJoRows));
        for (int i = 9; i < n && r.frames < 40; ++i) // from the workspot drop: rows 9, 10 drop, 11 launch
        {
            const JoRow& row = kJoRows[i];
            SpinUntil(next);
            next += frame;
            in.dt = 1.0f / 60.0f;
            in.velWorld[2] = row.vz;
            in.locoState = row.loco;
            in.psmBlock = row.psm;
            in.onGround = row.onGround != 0;
            in.distToGround = row.dist;
            in.psmFall = row.psmFall;
            Feed(in);
            if (!launched && Field(Status(), "jumpOuts") >= 1.0f)
            {
                launched = true;
                r.launchRow = i;
                r.edgeFrame = r.frames;
            }
            sinceEdge = launched ? r.frames - r.edgeFrame : -1;
            if ((row.psm & kWs) != 0)
                seated();
            else
                flight();
            animFrame(launched && sinceEdge >= aDeniedAfterEdge);
        }
    }
    const auto k1 = vtr::SnapCounters();
    r.handed = VtCounts()[1];
    r.pins = float(k1.exitHoldPins - k0.exitHoldPins);
    r.holdFrames = float(k1.exitHoldFrames - k0.exitHoldFrames);
    r.holdEnds = float(k1.exitHoldEnds - k0.exitHoldEnds);
    r.ok = aNormal ? r.edgeFrame >= 0 : launched;
    return r;
}
// 2026-10-28 11-49-37 mounted pop filter: the game's seated / exit pose jumps in single frames while MM is at weight 0 (48.596 arms 86 deg,
// 48.707 back 71 deg). Ride with the vehicle bit in TPP, the animation thread (FkSide) every frame at 60 Hz with the given permission
// (false = utpp_camera's vehicle hold), the game pose S, from frame 30 the upper body +86 / legs +45 deg in one frame, from frame 37 the
// upper body back to +15 deg (-71 deg), at frame 56 the legs +20 deg more (below the pop threshold: untouched).
struct RidePopRes
{
    bool ok = false;
    float maxVisStepDeg = 0.0f, rawPopDeg = 0.0f, endDevDeg = -1.0f, pops = 0.0f;
    bool untouchedBefore = true, untouchedSmall = true;
    std::string Line() const
    {
        char b[260];
        std::snprintf(b, sizeof b, "ok %d game pop %.1f deg, visible step max %.1f deg, records untouched before the pop %d and at the 20 deg step %d, "
                      "visible vs game 0.3 s after the last pop %.2f deg, pops %.0f", ok ? 1 : 0, rawPopDeg, maxVisStepDeg,
                      untouchedBefore ? 1 : 0, untouchedSmall ? 1 : 0, endDevDeg, pops);
        return b;
    }
};
RidePopRes RunRidePops(uint64_t aRig, uint32_t aCount, bool aAllows)
{
    RidePopRes r;
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones || a.gameIndex[a.firstBone] < 0)
        return r;
    const uint64_t popBase = vtr::g_counters.pops.load();
    Recs game(aCount);
    std::vector<uint8_t> raw, prevVis;
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow();
    for (int i = 0; i < 80; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        Feed(in);
        const float up = i >= 37 ? 15.0f : (i >= 30 ? 86.0f : 0.0f);
        const float leg = i >= 56 ? 65.0f : (i >= 30 ? 45.0f : 0.0f);
        FillRecs(game, a, [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? up : leg; }, [](uint32_t) { return 0.0f; });
        raw = game.buf;
        vtr::FkSide(game.buf.data(), aCount, aRig, QpcNow(), aAllows);
        if (i >= 20 && i < 30 && game.buf != raw)
            r.untouchedBefore = false;
        if (i == 56 && game.buf != raw)
            r.untouchedSmall = false;
        if (i >= 20 && !prevVis.empty())
            for (uint32_t b = a.firstBone; b < a.numBones; ++b)
            {
                const int32_t gi = a.gameIndex[b];
                if (gi < 0 || uint32_t(gi) >= aCount)
                    continue;
                const float* v = reinterpret_cast<const float*>(game.buf.data() + size_t(gi) * vti::kGameBoneStride);
                const float* pv = reinterpret_cast<const float*>(prevVis.data() + size_t(gi) * vti::kGameBoneStride);
                const float* rv = reinterpret_cast<const float*>(raw.data() + size_t(gi) * vti::kGameBoneStride);
                if (i != 56) // the 20 deg step is the game's own motion (untouched, checked above)
                    r.maxVisStepDeg = std::max(r.maxVisStepDeg, RecAngleDeg(pv, v));
                if (i == 30)
                    r.rawPopDeg = std::max(r.rawPopDeg, RecAngleDeg(pv, rv));
                if (i == 55)
                    r.endDevDeg = std::max(r.endDevDeg, RecAngleDeg(rv, v));
            }
        prevVis = game.buf;
    }
    r.pops = float(vtr::g_counters.pops.load() - popBase);
    r.ok = true;
    return r;
}
struct RideRes
{
    bool ok = false;
    std::array<float, 4> vt = {-1.0f, -1.0f, -1.0f, -1.0f};
    int midWrites = 0, midRoot = 0, midFeeds = 0;
    float maxRootM = 0.0f;
    std::string Line() const
    {
        char b[220];
        std::snprintf(b, sizeof b, "ok %d vehTrans mounts %.0f handed %.0f no capture %.0f aborts %.0f; mid-ride feeds %d with a transition write %d "
                      "with a root offset %d (max %.2f m)",
                      ok ? 1 : 0, vt[0], vt[1], vt[2], vt[3], midFeeds, midWrites, midRoot, maxRootM);
        return b;
    }
};
// review fix (vehdll MMDriver:3307): mount, ride 1.5 s; from 0.5 s the car moves 0.30 m per feed (18 m/s); mid-ride an in-car
// scene (3 feeds), a same-rig re-arm and a settings re-push; then the unmount. The mount transition must run once: no write and
// no root offset mid-ride (the vehicle's motion is no workspot jump), the capture runs again after the scene, the exit is
// handed in.
RideRes RunRideAborts(void* aRigPtr, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RideRes r;
    const uint64_t rig = reinterpret_cast<uint64_t>(aRigPtr);
    const uint32_t count = uint32_t(aHashes.size());
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones)
        return r;
    Recs game(count);
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow(), cut = 0;
    for (int i = 0; i < 96; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        if (i >= 30)
            in.capsulePos[0] += 0.30f;
        if (i >= 35 && i < 38)
            in.psmBlock |= 0x2u; // in-car scene (kPsmScene)
        if (i == 45)
            OnArmed(aRigPtr, aHashes, aParents); // the game re-arms: animated object / tracks changed, the same rig
        if (i == 50)
            SetMountCut(true); // settings re-push, the same value
        if (i == 95)
        {
            in.psmBlock = kVeh;
            in.locoState = 1;
        }
        Feed(in);
        const int64_t now = QpcNow();
        if (i == 0)
            cut = now;
        FillRecs(game, a, [](uint32_t) { return 0.0f; }, [](uint32_t) { return 0.0f; });
        const uint32_t n = vtr::FkSide(game.buf.data(), count, rig, now, true);
        if (QpcSeconds(now - cut) > 0.45f)
        {
            ++r.midFeeds;
            r.midWrites += n > 0 ? 1 : 0;
            float ro[3] = {};
            if (vtr::RootOffset(now, ro))
            {
                ++r.midRoot;
                r.maxRootM = std::max(r.maxRootM, std::hypot(ro[0], ro[1]));
            }
        }
    }
    for (int i = 0; i < 10; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = kWs | Det(0);
        in.locoState = 0;
        Feed(in);
        FillRecs(game, a, [](uint32_t) { return 0.0f; }, [](uint32_t) { return 0.0f; });
        vtr::FkSide(game.buf.data(), count, rig, QpcNow(), true);
    }
    r.vt = VtCounts();
    r.ok = true;
    return r;
}
// review fix (vehdll VehicleTransition:366): the seated game legs are 40 deg (UpLeg) / 10 deg (Leg) off the MM pose, the upper
// body 50 deg. Through the real driver (roles from the game names of the frozen snapshot) the whole leg chain is on the game pose
// after the hold; the per-bone fallback kept the 10 deg shin offset.
struct LegRes
{
    bool ok = false;
    float shinMaxDeg = 0.0f, thighMaxDeg = 0.0f, upperMaxDeg = 0.0f;
    int offsetFrames = 0;
    std::string Line() const
    {
        char b[200];
        std::snprintf(b, sizeof b, "ok %d offset frames %d max off the game pose: shin %.2f thigh %.2f upper body %.2f deg", ok ? 1 : 0,
                      offsetFrames, shinMaxDeg, thighMaxDeg, upperMaxDeg);
        return b;
    }
};
LegRes RunMountLegs(uint64_t aRig, const std::vector<uint64_t>& aHashes)
{
    LegRes r;
    const uint32_t count = uint32_t(aHashes.size());
    FeedInput in = MountFeed();
    for (int i = 0; i < 120; ++i)
        Feed(in);
    static vti::PoseSnapshot a;
    if (!ReadSnap(a) || a.weight < 0.99f || !a.numBones)
        return r;
    const uint64_t hUp[2] = {vti::CNameHash("LeftUpLeg"), vti::CNameHash("RightUpLeg")};
    const uint64_t hLeg[2] = {vti::CNameHash("LeftLeg"), vti::CNameHash("RightLeg")};
    const auto hashOf = [&](uint32_t b) -> uint64_t {
        const int32_t g = a.gameIndex[b];
        return g >= 0 && uint32_t(g) < aHashes.size() ? aHashes[size_t(g)] : 0ull;
    };
    const auto isUp = [&](uint32_t b) { return hashOf(b) == hUp[0] || hashOf(b) == hUp[1]; };
    const auto isLeg = [&](uint32_t b) { return hashOf(b) == hLeg[0] || hashOf(b) == hLeg[1]; };
    const auto rot = [&](uint32_t b) { return vti::SnapshotUpperBone(a, b) ? 50.0f : (isUp(b) ? 40.0f : (isLeg(b) ? 10.0f : 0.0f)); };
    Recs game(count), ref(count);
    // car seat: the Hips 0.55 m (below vehtrans::kStandHipsZ: the 25 deg leg rule, not the standing bike entry cap)
    const uint32_t hips = a.firstBone;
    const float seatDz = 0.55f - a.pose[size_t(hips) * 7 + 2];
    FillRecs(ref, a, rot, [&](uint32_t b) { return b == hips ? seatDz : 0.0f; });
    const int64_t frame = FrameTicks();
    int64_t next = QpcNow(), cut = 0;
    for (int i = 0; i < 20; ++i)
    {
        SpinUntil(next);
        next += frame;
        in.psmBlock = i == 0 ? kVeh : (kVeh | kWs);
        in.locoState = i == 0 ? 1 : 0;
        Feed(in);
        const int64_t now = QpcNow();
        if (i == 0)
            cut = now;
        game.buf = ref.buf;
        vtr::FkSide(game.buf.data(), count, aRig, now, true);
        const float t = QpcSeconds(now - cut);
        if (i < int(vtr::kMountHoldFrames) || t >= vtr::kMountSeconds)
            continue;
        ++r.offsetFrames;
        for (uint32_t b = a.firstBone; b < a.numBones; ++b)
        {
            const int32_t g = a.gameIndex[b];
            if (g < 0 || uint32_t(g) >= count)
                continue;
            const float d = RecAngleDeg(game.At(uint32_t(g)), ref.At(uint32_t(g)));
            if (isLeg(b))
                r.shinMaxDeg = std::max(r.shinMaxDeg, d);
            else if (isUp(b))
                r.thighMaxDeg = std::max(r.thighMaxDeg, d);
            else if (vti::SnapshotUpperBone(a, b))
                r.upperMaxDeg = std::max(r.upperMaxDeg, d);
        }
    }
    r.ok = true;
    return r;
}
bool FkSideNoop(uint64_t aRig, uint32_t aCount)
{
    static vti::PoseSnapshot a;
    if (!ReadSnap(a))
        return false;
    Recs game(aCount);
    FillRecs(game, a, [](uint32_t) { return 20.0f; }, [](uint32_t) { return 0.0f; });
    const std::vector<uint8_t> before = game.buf;
    const uint32_t n = vtr::FkSide(game.buf.data(), aCount, aRig, QpcNow(), true);
    return n == 0 && game.buf == before;
}
std::string Line(const VxRes& r)
{
    char b[300];
    std::snprintf(b, sizeof b, "feeds %zu wUp %.3f s minAfterUp %.2f mb max %.2f mounted w max %.2f stale w %.2f->%.2f held %d air ep %.0f lands %.0f diag %zu",
                  r.w.size(), FirstAt(r.w, 0.99f), MinAfterUp(r, 999), MaxOf(r.mb), r.mountedWMax, r.staleW0, r.staleW1, r.staleHeld ? 1 : 0,
                  r.airEpisodes, r.airLands, r.diag.size());
    return b;
}
} // namespace vx

// Production order of a mount in UTPP third person: MountEvent (the MountingEvent) -> up to 0.2 s of feeds without the vehicle bit ->
// the vehicle bit. utpp_camera's vehicle hold denies the pose write 5-10 ms after the cut, i.e. before the first vehicle feed, and
// vehtrans::FkSide then ends the mount transition unless the mounted-TPP flag is already set. Returns the flag after the event.
bool MountTppFlagAfterEvent(bool aTpp)
{
    FeedInput in;
    in.tppActive = aTpp;
    in.locoState = 1;
    in.dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i)
        Feed(in);
    MountEvent();
    for (int i = 0; i < 12; ++i)
        Feed(in);
    const bool flag = utpp::mm::vehtrans::g_mountTpp.load();
    utpp::mm::vehtrans::Abort();
    return flag;
}

void SectionVehExit(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionVehExit");
    (void)argc; // the first database only (argv[1])
    using namespace vx;
    const SimClockScope simClock; // r4 landing: the injected sim clock follows the wall time
    const std::string path = argv[1];
    std::string name = path.substr(path.find_last_of("\\/") + 1);
    name = name.substr(0, name.rfind('.'));
    const std::wstring wp(path.begin(), path.end());
    const auto fresh = [&] {
        const bool ok = LoadDatabase(wp, name);
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a(); // hold on, mask on, 0.6.0 (mount cut) on
        SetUpperMaskStates(7);
        SetAir(0);
        SetAirRobust(true);
        SetLogger(&VxLog);
        return ok;
    };
    bool loaded = true;
    std::string all;
    const auto run = [&](const char* aName, const std::vector<VxStep>& aSteps, bool aVehicle = true) {
        loaded = fresh() && loaded;
        const VxRes r = Run(aSteps, aVehicle);
        std::printf("[INFO] vehexit [%s] %s: %s\n", name.c_str(), aName, Line(r).c_str());
        return r;
    };
    // ---- T1 (car exit, the long variant 228.6-229.95): exit workspot + NotInBase 0, loco 0 for 1.30 s, then stand
    {
        const VxRes t1 = run("T1 workspot 1.30 s", {VxStep{.seconds = 1.30f, .psm = kWs | Det(0), .loco = 0}, VxStep{.seconds = 1.0f}});
        Check("vehexit [" + name + "] CAR-1 T1: exit workspot (latched at the unmount) 1.30 s - MM back by 0.25 s, never below 0.5 after, upper body on MM",
              loaded && FirstAt(t1.w, 0.99f) > 0.0f && FirstAt(t1.w, 0.99f) <= 0.25f + 0.034f && MinAfterUp(t1, 1) >= 0.5f && MaxOf(t1.mb) <= 0.0f &&
                  t1.mountedWMax == 0.0f,
              Line(t1));
        // EXITDIAG: at least one line from the unmount, at most 40, the first at +0.00 s, an end line, status exitDiag=
        bool first0 = !t1.diag.empty() && t1.diag.front().find("UTPP EXITDIAG +0.00s ") != std::string::npos;
        bool hasEnd = !t1.diag.empty() && t1.diag.back().find(" end") != std::string::npos;
        Check("vehexit [" + name + "] CAR-1 EXITDIAG: lines from the falling edge (+0.00 s), one per state change, an end line, <= 40, status exitDiag=",
              first0 && hasEnd && t1.diag.size() >= 2 && t1.diag.size() <= 40 && Field(t1.status, "exitDiag") == float(t1.diag.size()),
              std::to_string(t1.diag.size()) + " lines" + (t1.diag.empty() ? std::string() : " first: " + t1.diag.front()));
        for (size_t i = 0; i < t1.diag.size() && i < 6; ++i)
            std::printf("[INFO] vehexit [%s] T1 %s\n", name.c_str(), t1.diag[i].c_str());
    }
    // ---- T2 (bike, the short variant): NotInBase 0, loco 0 for 0.35 s, unarmed: legs and upper body on MM
    {
        const VxRes t2 = run("T2 bike detail 0 0.35 s", {VxStep{.seconds = 0.35f, .psm = Det(0), .loco = 0}, VxStep{.seconds = 1.0f}});
        Check("vehexit [" + name + "] CAR-1 T2: bike NotInBase 0.35 s - MM back by 0.25 s, upper body on MM (maskBlend 0) while the exit mask covers the gate",
              loaded && FirstAt(t2.w, 0.99f) > 0.0f && FirstAt(t2.w, 0.99f) <= 0.25f + 0.034f && MaxOf(t2.mb) <= 0.0f && t2.mask.size() > 5 &&
                  t2.mask[5] == 4u,
              Line(t2));
    }
    // ---- T4: the feed goes stale for 0.5 s right after the unmount (exit workspot latched): held, not released
    {
        const VxRes t4 = run("T4 stale 0.5 s", {VxStep{.seconds = 0.10f, .psm = kWs | Det(0), .loco = 0}, VxStep{.seconds = 0.5f, .stale = true},
                                                VxStep{.seconds = 0.5f, .psm = kWs | Det(0), .loco = 0}, VxStep{.seconds = 0.5f}});
        Check("vehexit [" + name + "] CAR-1 T4: stale feed 0.5 s after the unmount - held (published weight not lower, holdPoseActive), MM back after",
              loaded && t4.staleW0 > 0.0f && t4.staleW1 >= t4.staleW0 && t4.staleHeld && t4.w.back() >= 0.99f, Line(t4));
    }
    // ---- negative controls (they must keep releasing)
    {
        const VxRes seat = run("N seat switch", {VxStep{.seconds = 0.20f, .psm = kWs | Det(0), .loco = 0}, VxStep{.seconds = 0.4f, .psm = kVeh | kWs, .loco = 0},
                                                 VxStep{.seconds = 1.0f}});
        float seatW = 0.0f;
        for (size_t i = 0; i < seat.w.size(); ++i)
            if (seat.stepOf[i] == 1)
                seatW = std::max(seatW, seat.w[i]);
        Check("vehexit [" + name + "] CAR-1 negative: the vehicle bit again 0.2 s after the unmount (seat switch) cuts the body at once (weight 0 on every vehicle feed)",
              loaded && seatW == 0.0f && seat.w.back() >= 0.99f, Line(seat) + " seat w max " + std::to_string(seatW));
        bool relOk = true;
        std::string relL;
        const std::pair<const char*, uint32_t> rel[] = {{"scene", 0x2u}, {"swim", 0x4u}, {"knockdown 29", Det(29)}, {"felled 31", Det(31)},
                                                        {"takedown", 0x10u}, {"ladder 10", Det(10)}};
        for (const auto& [rn, bits] : rel)
        {
            const VxRes x = run(rn, {VxStep{.seconds = 0.30f, .psm = kWs | Det(0), .loco = 0}, VxStep{.seconds = 0.6f, .psm = bits, .loco = 0}});
            const float wEnd = WAtStepEnd(x, 1);
            relOk = relOk && wEnd <= 0.01f;
            relL += std::string(rn) + " wEnd " + std::to_string(wEnd) + " ";
        }
        Check("vehexit [" + name + "] CAR-1 negative: scene / swim / knockdown 29 / felled 31 / takedown / ladder inside the exit window still release (weight 0)",
              loaded && relOk, relL);
        // a NEW workspot: inside the window after the latched one ended, and at +1.0 s after a normal exit; on foot without a vehicle edge
        const VxRes nw = run("N new workspot in the window", {VxStep{.seconds = 0.30f, .psm = kWs | Det(0), .loco = 0},
                                                             VxStep{.seconds = 0.20f, .psm = Det(0), .loco = 0},
                                                             VxStep{.seconds = 0.6f, .psm = kWs | Det(0), .loco = 0}});
        const VxRes nl = run("N new workspot +1.0 s", {VxStep{.seconds = 1.0f}, VxStep{.seconds = 0.6f, .psm = kWs | Det(0), .loco = 0}});
        const VxRes nf = run("N on-foot workspot", {VxStep{.seconds = 1.0f}, VxStep{.seconds = 0.6f, .psm = kWs | Det(0), .loco = 0}}, false);
        Check("vehexit [" + name + "] CAR-1 negative: a new workspot (rising edge inside the window, at +1.0 s, on foot without a vehicle edge) releases",
              loaded && WAtStepEnd(nw, 2) <= 0.01f && WAtStepEnd(nl, 1) <= 0.01f && WAtStepEnd(nf, 1) <= 0.01f,
              "in-window " + std::to_string(WAtStepEnd(nw, 2)) + " +1.0 s " + std::to_string(WAtStepEnd(nl, 1)) + " on foot " +
                  std::to_string(WAtStepEnd(nf, 1)));
        Check("vehexit [" + name + "] CAR-1 negative: no EXITDIAG line without a vehicle-bit edge (on foot)", nf.diag.empty() && Field(nf.status, "exitDiag") == 0.0f,
              std::to_string(nf.diag.size()));
    }
    // ---- CAR-2: jump-out at 7 m/s (vz +2), Fall 14 for 0.6 s, RegularLand 23, Stand - with the latched exit workspot and without
    {
        bool okWs = true, okNo = true;
        std::string l;
        for (int v = 0; v < 2; ++v)
        {
            const uint32_t ws = v == 0 ? kWs : 0u;
            const VxRes c = run(v == 0 ? "CAR-2 jump-out (exit workspot)" : "CAR-2 jump-out (no workspot)",
                                {VxStep{.seconds = 0.6f, .psm = ws | Det(14), .loco = 0, .vz = 2.0f, .g = 16.0f, .speed = 7.0f, .airborne = true},
                                 VxStep{.seconds = 0.15f, .psm = Det(23), .loco = 0, .speed = 4.0f}, VxStep{.seconds = 1.0f, .speed = 1.0f}});
            int ownFall = 0;
            for (size_t i = 0; i < c.w.size(); ++i)
                if (c.stepOf[i] == 0)
                    ownFall += c.owns[i];
            const bool ok = FirstAt(c.w, 0.99f) > 0.0f && FirstAt(c.w, 0.99f) <= 0.25f + 0.034f && MinAfterUp(c, 2) >= 0.99f && MaxOf(c.mb) <= 0.0f &&
                            ownFall >= 30 && c.airEpisodes >= 1.0f && c.airLands >= 1.0f;
            (v == 0 ? okWs : okNo) = ok;
            l += std::string(v == 0 ? " ws: " : " no ws: ") + Line(c) + " air-owned fall feeds " + std::to_string(ownFall);
        }
        Check("vehexit [" + name + "] CAR-2: jump-out at 7 m/s with the latched exit workspot - MM / air own the fall and the landing (w >= 0.99 by 0.25 s, "
                                     "no game-pose frame after, air episode + landing, upper body on MM)",
              loaded && okWs, l);
        Check("vehexit [" + name + "] CAR-2: jump-out without a workspot - the same (regression guard)", loaded && okNo, l);
        // VeryHardLand 25 after the exit: the game's land animation (release); on foot 25 stays held as before
        const VxRes vh = run("CAR-2 VeryHardLand 25", {VxStep{.seconds = 0.3f, .psm = kWs | Det(14), .loco = 0, .vz = -6.0f, .speed = 7.0f, .airborne = true},
                                                       VxStep{.seconds = 0.6f, .psm = Det(25), .loco = 0}});
        const VxRes vf = run("on-foot VeryHardLand 25", {VxStep{.seconds = 0.5f}, VxStep{.seconds = 0.3f, .psm = Det(14), .loco = 0, .vz = -6.0f, .airborne = true},
                                                         VxStep{.seconds = 0.6f, .psm = Det(25), .loco = 0}}, false);
        Check("vehexit [" + name + "] CAR-2: VeryHardLand 25 inside the exit window releases to the game land; on foot it stays held (unchanged)",
              loaded && WAtStepEnd(vh, 1) <= 0.01f && WAtStepEnd(vf, 2) >= 0.99f,
              "exit " + std::to_string(WAtStepEnd(vh, 1)) + " on foot " + std::to_string(WAtStepEnd(vf, 2)));
    }
    // ---- J4 (verification): bike unmount + ballistic arc vz0 +6 m/s for 1.0 s (Cyberpunk gravity ~16 m/s^2), then land and stand.
    // Hypothesis (b) "the exit-window mask keeps the game upper body over an MM ground query": the mask must not show during the
    // flight (apex frames with |vz| < 1 included) and the air area must own the flight from the 2nd airborne feed.
    {
        bool ok = true;
        std::string l;
        for (uint32_t d : {0u, 14u})
        {
            const VxRes j = run(d == 0 ? "J4 arc detail 0" : "J4 arc fall 14",
                                {VxStep{.seconds = 0.75f, .psm = Det(d), .loco = 0, .vz = 6.0f, .g = 16.0f, .speed = 2.0f, .airborne = true},
                                 VxStep{.seconds = 0.15f, .psm = Det(23), .loco = 0, .speed = 1.0f}, VxStep{.seconds = 1.0f}});
            bool ownFrom2 = true;
            int air = 0;
            for (size_t i = 0; i < j.w.size(); ++i)
                if (j.stepOf[i] == 0)
                {
                    if (air >= 1)
                        ownFrom2 = ownFrom2 && j.owns[i] == 1;
                    ++air;
                }
            ok = ok && ownFrom2 && MaxOf(j.mb) <= 0.0f && FirstAt(j.w, 0.99f) > 0.0f && FirstAt(j.w, 0.99f) <= 0.25f + 0.034f && j.airLands >= 1.0f;
            l += std::string(" detail ") + std::to_string(d) + ": " + Line(j) + " owned from the 2nd airborne feed " + (ownFrom2 ? "1" : "0");
        }
        Check("vehexit [" + name + "] J4: unmount + ballistic arc (vz0 +6, 0.75 s) - air owns from the 2nd airborne feed, no exit mask in the flight, "
                                     "a landing, MM by 0.25 s",
              loaded && ok, l);
    }
    // ---- JUMPOUT (2026-10-28, recording 01-22-53 video 68.8-70.1 s): the bike jump-out replayed from the recorded feed rows
    {
        loaded = fresh() && loaded;
        const JumpOutRes jo = RunRecordedJumpOut();
        std::printf("[INFO] vehexit [%s] JUMPOUT recorded: %s\n", name.c_str(), jo.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT: recorded bike jump-out - the vehicle bit after the workspot drop is released at the launch (one "
                                     "exit edge, one EXITDIAG start), MM at full weight within 0.25 s of the launch (no animation thread here: no capture, the live exit capture gives up and the normal fade-in runs) and never below 0.99 after, "
                                     "air owns the whole flight, no upper mask, nothing during the ride",
              loaded && jo.launch >= 0 && jo.land > jo.launch && jo.rideWMax == 0.0f && jo.tUp > 0.0f && jo.tUp <= 0.25f + 0.034f &&
                  jo.wMinAfterUp >= 0.99f && jo.ownedFlight && jo.mbMax <= 0.0f && jo.exitWindows == 1.0f && jo.diagStarts == 1 &&
                  jo.jumpOuts == 1.0f && jo.epochDelta == 1 && jo.flightCuts == 0.0f && !jo.activeAtEnd &&
                  jo.exitFlagBefore == 0.0f && jo.exitFlagFlight == 1.0f && jo.exitFlagEnd == 0.0f &&
                  jo.slewFrames >= 15.0f && jo.slewFrames <= 45.0f,
              jo.Line() + " jumpOutExit flag before/flight/end " + std::to_string(jo.exitFlagBefore) + "/" + std::to_string(jo.exitFlagFlight) + "/" + std::to_string(jo.exitFlagEnd));
        // same-rig re-arms right after the launch and in the falling half (the game re-arms around unmounts): latch, weight and window kept
        loaded = fresh() && loaded;
        const JumpOutRes jr = RunRecordedJumpOut([&](int aRow) {
            if (aRow == 12 || aRow == 60)
                OnArmed(aRig, aHashes, aParents);
        });
        std::printf("[INFO] vehexit [%s] JUMPOUT recorded + re-arms: %s\n", name.c_str(), jr.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT: same-rig re-arms after the launch and after the apex keep the release (no mount cut, one exit "
                                     "window, weight never below 0.99 once up)",
              loaded && jr.tUp > 0.0f && jr.tUp <= 0.25f + 0.034f && jr.wMinAfterUp >= 0.99f && jr.exitWindows == 1.0f && jr.flightCuts == 0.0f &&
                  jr.jumpOuts == 1.0f && jr.epochDelta == 1,
              jr.Line());
        // r3 landing (log 2026-11-03 23:26, car jump-out): ONE ground-contact feed in the descent (the capsule grazing the car body at -10 m/s;
        // the game reports Stand, gate 0) closes the exit window; the jump-out state must outlive it until the real touchdown has lasted
        // 0.10 s, so the chooser keeps the jump-out flight (no Start clip, jump-out landing threshold)
        constexpr int kBlipRow = 86;
        loaded = fresh() && loaded;
        const JumpOutRes jb = RunRecordedJumpOut({}, [](int aRow, FeedInput& aIn) {
            if (aRow == kBlipRow)
            {
                aIn.onGround = true;
                aIn.locoState = 1;
                aIn.psmBlock = 0;
            }
        });
        bool heldThrough = jb.flagRow.size() > size_t(kBlipRow) + 6 && jb.windowRow[size_t(kBlipRow)] == 0.0f && jb.windowRow[size_t(kBlipRow) - 1] > 0.0f;
        for (size_t k = size_t(kBlipRow); k <= size_t(kBlipRow) + 5; ++k)
            heldThrough = heldThrough && jb.flagRow[k] == 1.0f; // rows 86..91: false contact, 4 airborne rows, the touchdown row
        std::printf("[INFO] vehexit [%s] JUMPOUT false contact: window before/at blip %.2f/%.2f, flag rows %zu..%zu = %.0f %.0f %.0f %.0f %.0f %.0f, end %.0f\n", name.c_str(),
                    double(jb.windowRow[size_t(kBlipRow) - 1]), double(jb.windowRow[size_t(kBlipRow)]), size_t(kBlipRow), size_t(kBlipRow) + 5,
                    double(jb.flagRow[size_t(kBlipRow)]), double(jb.flagRow[size_t(kBlipRow) + 1]), double(jb.flagRow[size_t(kBlipRow) + 2]),
                    double(jb.flagRow[size_t(kBlipRow) + 3]), double(jb.flagRow[size_t(kBlipRow) + 4]), double(jb.flagRow[size_t(kBlipRow) + 5]),
                    double(jb.exitFlagEnd));
        Check("vehexit [" + name + "] JUMPOUT false contact: a one-feed ground contact in the descent closes the exit window, the jump-out state stays through the "
                                     "real touchdown (rows 86..91) and ends once the ground contact has lasted 0.10 s",
              loaded && jb.launch >= 0 && heldThrough && jb.exitFlagEnd == 0.0f && jb.jumpOuts == 1.0f && jb.exitWindows == 1.0f,
              jb.Line());
        // a flight that never touches down: the hold ends 0.5 s after the window closed (the rows after the blip stay airborne, 0.59 s)
        loaded = fresh() && loaded;
        const JumpOutRes jc = RunRecordedJumpOut({}, [](int aRow, FeedInput& aIn) {
            if (aRow == kBlipRow)
            {
                aIn.onGround = true;
                aIn.locoState = 1;
                aIn.psmBlock = 0;
            }
            else if (aRow > kBlipRow)
            {
                aIn.onGround = false;
                aIn.velWorld[0] = aIn.velWorld[1] = 0.0f;
                aIn.velWorld[2] = -10.5f;
                aIn.psmBlock = 0x61002000u;
            }
        });
        const size_t endRow = jc.flagRow.size() - 1;
        float tAfter = 0.0f;
        size_t releasedAt = 0;
        for (size_t k = size_t(kBlipRow) + 1; k < jc.flagRow.size() && releasedAt == 0; ++k)
        {
            tAfter += k < std::size(kJoRows) ? kJoRows[k].dt : kJoRows[std::size(kJoRows) - 1].dt;
            if (jc.flagRow[k] == 0.0f)
                releasedAt = k;
        }
        std::printf("[INFO] vehexit [%s] JUMPOUT no touchdown: flag released at row %zu (%.3f s after the blip), end row %zu\n", name.c_str(), releasedAt, double(tAfter), endRow);
        Check("vehexit [" + name + "] JUMPOUT false contact, no touchdown: the held jump-out state ends by the 0.5 s cap (not before 0.4 s, not kept to the end of the run)",
              loaded && jc.flagRow.size() > size_t(kBlipRow) + 20 && jc.flagRow[size_t(kBlipRow) + 15] == 1.0f && releasedAt != 0 && tAfter >= 0.4f && tAfter <= 0.56f,
              "released at row " + std::to_string(releasedAt) + ", " + std::to_string(tAfter) + " s after the blip");
        // V2 side of the same false contact: the v2 step must receive the exit window and the jump-out flag through the false contact and the
        // touchdown (rows 86..91), otherwise the chooser loses its jump-out flight even though the host kept the state (MotionHost.cpp MotionStepLocked)
        loaded = fresh() && loaded;
        g_motionSeen.clear();
        SetMotionHooks(&kCaptureMotionHooks);
        const JumpOutRes jv = RunRecordedJumpOut({}, [](int aRow, FeedInput& aIn) {
            g_motionRow = aRow;
            if (aRow == kBlipRow)
            {
                aIn.onGround = true;
                aIn.locoState = 1;
                aIn.psmBlock = 0;
            }
        });
        SetMotionHooks(nullptr);
        bool motionHeld = true;
        std::string motionRows;
        for (int k = kBlipRow; k <= kBlipRow + 5; ++k)
        {
            const auto it = g_motionSeen.find(k);
            const bool ok = it != g_motionSeen.end() && it->second.window == 1 && it->second.jumpOut == 1;
            motionHeld = motionHeld && ok;
            motionRows += it == g_motionSeen.end() ? " -" : (ok ? " 1" : " 0");
        }
        std::printf("[INFO] vehexit [%s] JUMPOUT false contact, v2 side: window and jump-out flag at the v2 step in rows %d..%d:%s\n", name.c_str(), kBlipRow, kBlipRow + 5, motionRows.c_str());
        Check("vehexit [" + name + "] JUMPOUT false contact: the motion step receives the exit window and the jump-out flag through the false contact and the touchdown (rows 86..91)",
              loaded && jv.launch >= 0 && motionHeld, motionRows);
        // a feed dropout around the touchdown: the Tick stale path must not drop the jump-out state when the exit window runs out without feeds (the move case
        // held the pose 0.6-0.8 s at the landing); the dropout starts in the descent with the window still open
        // r3b landing: from the landing approach on (row 84) Tick feeds the touchdown itself (see the landing dropout checks below); the stale path with the window running
        // out is still what a silence earlier in the descent (row 70, touchdown 0.33 s away) runs through, so both stay in this group
        for (const auto& drop : {std::make_pair(84, 0.85f), std::make_pair(84, 1.2f), std::make_pair(70, 1.5f)})
        {
            const int kDropRow = drop.first;
            const float dropS = drop.second;
            loaded = fresh() && loaded;
            const JumpOutRes jd = RunRecordedJumpOut([dropS, kDropRow](int aRow) {
                if (aRow != kDropRow)
                    return;
                LARGE_INTEGER f0, fq, fn;
                QueryPerformanceFrequency(&fq);
                QueryPerformanceCounter(&f0);
                do
                {
                    Sleep(5);
                    Tick();
                    QueryPerformanceCounter(&fn);
                } while (float(double(fn.QuadPart - f0.QuadPart) / double(fq.QuadPart)) < dropS);
            });
            bool kept = jd.land > kDropRow && jd.flagRow.size() > size_t(jd.land);
            for (int k = kDropRow; kept && k <= jd.land; ++k)
                kept = jd.flagRow[size_t(k)] == 1.0f;
            std::printf("[INFO] vehexit [%s] JUMPOUT feed dropout %.2f s at row %d: flag rows %d..%d %s, land row %d, end %.0f\n", name.c_str(), double(dropS), kDropRow, kDropRow, jd.land,
                        kept ? "all 1" : "not all 1", jd.land, double(jd.exitFlagEnd));
            Check("vehexit [" + name + "] JUMPOUT feed dropout " + std::to_string(int(dropS * 100.0f)) + (kDropRow == 84 ? "" : " (row " + std::to_string(kDropRow) + ")") +
                      " cs in the descent: the jump-out state survives the stale exit window to the touchdown and ends after it",
                  loaded && jd.launch >= 0 && kept && jd.exitFlagEnd == 0.0f, jd.Line());
        }
    }
    // ---- r3b landing (game log 2026-11-04 02:06:36 jump-out with the vehicle at 8 m/s; feed dump t 296.6-297.7): the PSM landing state after a very fast fall gives no
    // locomotion update at all - the last air feed is 0.01 s above the floor (vz -12.9), the next feed comes 0.65 s later in Stand with zero velocity. The gate kept its last
    // (air) value, the MM clock stalled and the landing clip started 0.65 s late. Tick() feeds the touchdown itself once the predicted touchdown is overdue: the gate opens,
    // the v2 step runs and AirState lands at the contact; the real feeds that follow confirm the touchdown and end the jump-out state.
    {
        int landRow = -1;
        for (int k = 0; k < int(std::size(kJoRows)) && landRow < 0; ++k)
            if (kJoRows[k].onGround != 0)
                landRow = k;
        struct DropProbe
        {
            int motionBefore = -1, motionAfter = -1, flaggedBefore = -1, flaggedAfter = -1;
            float lands0 = -1.0f, lands1 = -1.0f, episodes = -1.0f, feeds = -1.0f, held = -1.0f, feedsMid = -1.0f;
            uint32_t gate = 0xFFFFu;
        };
        auto runDrop = [&](int aDropRow, float aDropS, const std::function<void(int, FeedInput&)>& aMod = {}) {
            DropProbe pr;
            g_landMotionCalls = g_landMotionFlagged = 0;
            SetMotionHooks(&kCountMotionHooks);
            const JumpOutRes jr = RunRecordedJumpOut([&](int aRow) {
                if (aRow != aDropRow)
                    return;
                pr.motionBefore = g_landMotionCalls;
                pr.flaggedBefore = g_landMotionFlagged;
                pr.lands0 = Field(Status(), "airLands");
                if (aDropS > 0.55f)
                {
                    TickFor(0.5f);
                    pr.feedsMid = Field(Status(), "landDropoutFeeds");
                    TickFor(aDropS - 0.5f);
                }
                else
                    TickFor(aDropS);
                const std::string st = Status();
                pr.gate = HexField(st, "gate");
                pr.motionAfter = g_landMotionCalls;
                pr.flaggedAfter = g_landMotionFlagged;
                pr.lands1 = Field(st, "airLands");
                pr.episodes = Field(st, "landDropouts");
                pr.feeds = Field(st, "landDropoutFeeds");
                pr.held = Field(st, "jumpOutExit");
            }, aMod);
            SetMotionHooks(nullptr);
            return std::make_pair(jr, pr);
        };
        loaded = fresh() && loaded;
        const auto landDrop = runDrop(landRow, 0.65f);
        const JumpOutRes& jl = landDrop.first;
        const DropProbe& pl = landDrop.second;
        std::printf("[INFO] vehexit [%s] JUMPOUT landing dropout 0.65 s before the touchdown row %d: Tick landing episodes %.0f feeds %.0f, raw gate during 0x%X, motion steps %d (flagged %d), "
                    "airLands %.0f -> %.0f, jump-out state %.0f during, end %.0f\n",
                    name.c_str(), landRow, double(pl.episodes), double(pl.feeds), pl.gate, pl.motionAfter - pl.motionBefore, pl.flaggedAfter - pl.flaggedBefore, double(pl.lands0),
                    double(pl.lands1), double(pl.held), double(jl.exitFlagEnd));
        Check("vehexit [" + name + "] JUMPOUT landing dropout: Tick feeds the touchdown once (one episode, at least 5 synthetic feeds)",
              loaded && landRow > 0 && jl.launch >= 0 && pl.episodes == 1.0f && pl.feeds >= 5.0f, "episodes " + std::to_string(pl.episodes) + " feeds " + std::to_string(pl.feeds));
        // the real silent landing gap (~0.65 s of sim time, game log 2026-11-04 02:06:36) is continued whole: the synthetic feeds are still arriving in the last 0.15 s
        // of the gap (a 0.25 s wall-clock bound would stop them 0.4 s early and freeze the body again)
        Check("vehexit [" + name + "] JUMPOUT landing dropout 0.65 s of sim time: Tick feeds the whole gap (feeds keep arriving between 0.50 s and 0.65 s, one episode)",
              loaded && landRow > 0 && jl.launch >= 0 && pl.episodes == 1.0f && pl.feedsMid >= 1.0f && pl.feeds >= pl.feedsMid + 3.0f,
              "feeds at 0.50 s " + std::to_string(pl.feedsMid) + ", at 0.65 s " + std::to_string(pl.feeds));
        Check("vehexit [" + name + "] JUMPOUT landing dropout: the gate is open and the MM clock runs through the silent landing (raw gate 0, at least 5 motion steps, all with the jump-out flags)",
              loaded && pl.gate == 0u && pl.motionAfter - pl.motionBefore >= 5 && pl.flaggedAfter - pl.flaggedBefore == pl.motionAfter - pl.motionBefore,
              "gate " + std::to_string(pl.gate) + " motion steps " + std::to_string(pl.motionAfter - pl.motionBefore) + " flagged " + std::to_string(pl.flaggedAfter - pl.flaggedBefore));
        Check("vehexit [" + name + "] JUMPOUT landing dropout: AirState lands during the silence; the jump-out state stays for the real feeds to confirm and ends after the touchdown",
              loaded && pl.lands1 > pl.lands0 && pl.held == 1.0f && jl.exitFlagEnd == 0.0f && jl.land == landRow, "lands " + std::to_string(pl.lands0) + " -> " + std::to_string(pl.lands1));
        // the stand jump-out of the same log (feed dump t 158.410-158.527) reports the Fall detail 14 (0x610F2040) on the last air rows: the silent landing must not inherit it
        {
            loaded = fresh() && loaded;
            const auto det = runDrop(landRow, 0.65f, [landRow](int aRow, FeedInput& aIn) {
                if (aRow >= landRow - 3 && aRow < landRow)
                    aIn.psmBlock = 0x610F2040u;
            });
            const DropProbe& pd = det.second;
            std::printf("[INFO] vehexit [%s] JUMPOUT landing dropout after Fall-detail rows: Tick landing episodes %.0f feeds %.0f, raw gate during 0x%X, v2 steps %d, airLands %.0f -> %.0f\n",
                        name.c_str(), double(pd.episodes), double(pd.feeds), pd.gate, pd.motionAfter - pd.motionBefore, double(pd.lands0), double(pd.lands1));
            Check("vehexit [" + name + "] JUMPOUT landing dropout after Fall-detail rows (0x610F2040): the synthetic landing carries no detail - gate open, motion steps, AirState lands",
                  loaded && det.first.launch >= 0 && pd.episodes == 1.0f && pd.gate == 0u && pd.motionAfter - pd.motionBefore >= 5 && pd.lands1 > pd.lands0,
                  "episodes " + std::to_string(pd.episodes) + " gate " + std::to_string(pd.gate) + " motion steps " + std::to_string(pd.motionAfter - pd.motionBefore));
        }
        // a pause / menu starting in the last 0.2 s of the fall looks like the silent landing state to a wall clock (no feed). The continuation is measured in game SIM
        // time, which stands still in a pause: no synthetic feed, the episode does not progress, and on the resume the episode continues normally
        {
            // (a1) the pause starts inside a running episode (0.2 s of sim-time silence were already continued)
            loaded = fresh() && loaded;
            float lands1a = -1.0f, lands1b = -1.0f, feedsMid = -1.0f, feedsFrozen = -1.0f, feedsPaused = -1.0f, feedsResume = -1.0f, winBefore = -1.0f, winAfter = -1.0f, epsPaused = -1.0f, epsResume = -1.0f;
            const JumpOutRes jp = RunRecordedJumpOut([&](int aRow) {
                if (aRow != landRow)
                    return;
                winBefore = Field(Status(), "exitWindow");
                TickFor(0.2f); // the sim clock follows the wall time: the synthetic feeds run
                const std::string stMid = Status();
                winAfter = Field(stMid, "exitWindow");
                feedsMid = Field(stMid, "landDropoutFeeds");
                lands1a = Field(stMid, "airLands");
                SimSetMode(SimMode::Frozen);
                TickFor(0.05f); // the sim time between the last sample and the freeze is real: the Tick right after the freeze may still feed it
                feedsFrozen = Field(Status(), "landDropoutFeeds");
                TickFor(2.0f); // 2 s of wall time, no sim time
                const std::string stP = Status();
                feedsPaused = Field(stP, "landDropoutFeeds");
                epsPaused = Field(stP, "landDropouts");
                SimSetMode(SimMode::Follow);
                TickFor(0.2f); // resume: the same episode continues
                const std::string stR = Status();
                feedsResume = Field(stR, "landDropoutFeeds");
                epsResume = Field(stR, "landDropouts");
                lands1b = Field(stR, "airLands");
            });
            SimSetMode(SimMode::Follow);
            std::printf("[INFO] vehexit [%s] JUMPOUT pause 2 s (sim clock frozen) inside the landing episode, row %d: synthetic feeds %.0f before the pause, %.0f after it, %.0f after the resume, episodes %.0f/%.0f, "
                        "exit window %.2f -> %.2f\n",
                        name.c_str(), landRow, double(feedsMid), double(feedsPaused), double(feedsResume), double(epsPaused), double(epsResume), double(winBefore), double(winAfter));
            Check("vehexit [" + name + "] JUMPOUT pause (sim clock frozen, 2 s of wall time) inside the landing episode: no synthetic feed during the pause, the same episode continues on the resume",
                  loaded && jp.launch >= 0 && feedsMid >= 1.0f && feedsPaused == feedsFrozen && epsPaused == 1.0f && feedsResume >= feedsPaused + 3.0f && epsResume == 1.0f,
                  "feeds " + std::to_string(feedsMid) + " -> " + std::to_string(feedsFrozen) + " -> " + std::to_string(feedsPaused) + " -> " + std::to_string(feedsResume) + " episodes " + std::to_string(epsPaused) + "/" + std::to_string(epsResume));
            Check("vehexit [" + name + "] JUMPOUT pause inside the landing episode: after the resume the landing completes once (AirState landed before the pause and does not land a second time, the jump-out state ends after the real touchdown row)",
                  loaded && jp.launch >= 0 && lands1a >= 1.0f && lands1b == lands1a && jp.land == landRow && jp.exitFlagEnd == 0.0f,
                  "airLands " + std::to_string(lands1a) + " -> " + std::to_string(lands1b) + " land row " + std::to_string(jp.land) + " end " + std::to_string(jp.exitFlagEnd));
            Check("vehexit [" + name + "] JUMPOUT landing dropout: the synthetic feeds keep the exit window open (release:veryHardLand still possible on the next real feed)",
                  loaded && jp.launch >= 0 && feedsMid >= 1.0f && winBefore > 0.0f && std::fabs(winAfter - winBefore) <= 0.011f,
                  "window " + std::to_string(winBefore) + " -> " + std::to_string(winAfter) + " feeds " + std::to_string(feedsMid));
            // (a2) the pause starts at the landing row, before the touchdown is overdue: nothing is fed for the whole pause, no episode starts; the resume starts it normally
            loaded = fresh() && loaded;
            float lands2a = -1.0f, lands2b = -1.0f, feedsP2 = -1.0f, epsP2 = -1.0f, feedsR2 = -1.0f, epsR2 = -1.0f;
            const JumpOutRes jq = RunRecordedJumpOut([&](int aRow) {
                if (aRow != landRow)
                    return;
                lands2a = Field(Status(), "airLands");
                SimSetMode(SimMode::Frozen);
                TickFor(2.0f);
                const std::string stP = Status();
                feedsP2 = Field(stP, "landDropoutFeeds");
                epsP2 = Field(stP, "landDropouts");
                SimSetMode(SimMode::Follow);
                TickFor(0.3f);
                const std::string stR = Status();
                feedsR2 = Field(stR, "landDropoutFeeds");
                epsR2 = Field(stR, "landDropouts");
                lands2b = Field(stR, "airLands");
            });
            SimSetMode(SimMode::Follow);
            std::printf("[INFO] vehexit [%s] JUMPOUT pause 2 s (sim clock frozen) at the landing approach row %d: synthetic feeds %.0f / episodes %.0f during the pause, %.0f / %.0f after the resume\n",
                        name.c_str(), landRow, double(feedsP2), double(epsP2), double(feedsR2), double(epsR2));
            Check("vehexit [" + name + "] JUMPOUT pause (sim clock frozen, 2 s of wall time) at the landing approach: zero synthetic feeds during the pause, the episode starts normally on the resume",
                  loaded && jq.launch >= 0 && feedsP2 == 0.0f && epsP2 == 0.0f && feedsR2 >= 5.0f && epsR2 == 1.0f,
                  "pause feeds " + std::to_string(feedsP2) + " episodes " + std::to_string(epsP2) + ", resume feeds " + std::to_string(feedsR2) + " episodes " + std::to_string(epsR2));
            Check("vehexit [" + name + "] JUMPOUT pause at the landing approach: after the resume the landing completes (AirState lands, the jump-out state ends after the real touchdown row)",
                  loaded && jq.launch >= 0 && lands2b > lands2a && jq.land == landRow && jq.exitFlagEnd == 0.0f,
                  "airLands " + std::to_string(lands2a) + " -> " + std::to_string(lands2b) + " land row " + std::to_string(jq.land) + " end " + std::to_string(jq.exitFlagEnd));
            // (c) the sim clock is unavailable (the call fails): the continuation is bounded by 0.25 s of wall time after the last real feed (a longer gap may be a pause)
            loaded = fresh() && loaded;
            float feedsA = -1.0f, feedsB = -1.0f, epsC = -1.0f;
            const JumpOutRes jc = RunRecordedJumpOut([&](int aRow) {
                if (aRow != landRow)
                    return;
                SimSetMode(SimMode::Unavailable);
                TickFor(0.5f);
                feedsA = Field(Status(), "landDropoutFeeds");
                TickFor(0.7f);
                const std::string st2 = Status();
                feedsB = Field(st2, "landDropoutFeeds");
                epsC = Field(st2, "landDropouts");
            });
            SimSetMode(SimMode::Follow);
            std::printf("[INFO] vehexit [%s] JUMPOUT pause 1.2 s without a sim clock at the landing approach row %d: synthetic feeds %.0f after 0.5 s, %.0f after 1.2 s, episodes %.0f\n", name.c_str(),
                        landRow, double(feedsA), double(feedsB), double(epsC));
            Check("vehexit [" + name + "] JUMPOUT sim clock unavailable: the wall-clock fallback feeds the touchdown for 0.25 s only (feeds after 0.5 s == after 1.2 s, one episode)",
                  loaded && jc.launch >= 0 && epsC == 1.0f && feedsA >= 1.0f && feedsB == feedsA,
                  "feeds " + std::to_string(feedsA) + " -> " + std::to_string(feedsB) + " episodes " + std::to_string(epsC));
        }
        // the same silence in mid-flight (descent, touchdown more than 0.4 s away): a hitch or a pause is no landing - nothing is fed
        for (const int dropRow : {20, 60})
        {
            loaded = fresh() && loaded;
            const auto mid = runDrop(dropRow, 0.5f);
            std::printf("[INFO] vehexit [%s] JUMPOUT silence 0.5 s at row %d (not the landing approach): Tick landing episodes %.0f feeds %.0f\n", name.c_str(), dropRow,
                        double(mid.second.episodes), double(mid.second.feeds));
            Check("vehexit [" + name + "] JUMPOUT silence at row " + std::to_string(dropRow) + " (mid-flight, no landing predicted): Tick feeds nothing",
                  loaded && mid.first.launch >= 0 && mid.second.episodes == 0.0f && mid.second.feeds == 0.0f,
                  "episodes " + std::to_string(mid.second.episodes) + " feeds " + std::to_string(mid.second.feeds));
        }
    }
    // ---- P2 (CAR-5 / MOTO-3 mount pop, MOTO-6 / CAR-1 step 5 exit hand-in): mm/VehicleTransition.hpp through the real driver.
    // The animation thread is simulated: every frame the game records are set to the game pose and passed through
    // vehtrans::FkSide (what the Mapping / FK write paths call), at real 60 Hz timing (the transition runs on QPC time).
    {
        loaded = fresh() && loaded;
        const MountRes m = RunMount(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()));
        std::printf("[INFO] vehexit [%s] P2 mount: %s\n", name.c_str(), m.Line().c_str());
        Check("vehexit [" + name + "] P2 mount (CAR-5 / MOTO-3): no visible per-bone jump > 15 deg at the cut (game entry pose 50 deg off the MM pose "
                                     "on the upper body), transition frames written, nothing written 0.30 s after the jump sample",
              loaded && m.ok && m.cutJumpDeg <= 15.0f && m.maxJumpDeg <= 15.0f && m.writtenFrames >= 3 && m.writtenLate == 0 && m.diffDeg >= 40.0f,
              m.Line());
        Check("vehexit [" + name + "] P2 mount: an entity jump of 0.8 m / 30 deg right after the cut becomes a root offset = -(jump), 0 by 0.30 s",
              loaded && m.ok && std::fabs(m.rootDist - 0.8f) <= 0.05f && std::fabs(m.rootYaw + 30.0f) <= 1.0f && !m.rootLate, m.Line());
        loaded = fresh() && loaded;
        const bool flagTpp = MountTppFlagAfterEvent(true);
        loaded = fresh() && loaded;
        const bool flagFpp = MountTppFlagAfterEvent(false);
        Check("vehexit [" + name + "] P2 mount event before the vehicle bit (TPP): the vehtrans mounted-TPP flag is set at the cut and stays set while only "
                                     "the MountingEvent latch holds it (a CET pose-write denial before the first vehicle feed must not end the transition); "
                                     "FPP: not set",
              loaded && flagTpp && !flagFpp, std::string("tpp=") + (flagTpp ? "1" : "0") + " fpp=" + (flagFpp ? "1" : "0"));
        loaded = fresh() && loaded;
        const ExitHandRes e = RunExitHandIn(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()));
        std::printf("[INFO] vehexit [%s] P2 exit: %s\n", name.c_str(), e.Line().c_str());
        Check("vehexit [" + name + "] P2 exit (MOTO-6 / CAR-1 step 5): capture of the standing exit pose, MM at weight 1 on the edge frame, no visible "
                                     "frame within 0.3 s at the squat pelvis height, status vehTrans handed 1",
              loaded && e.ok && e.wEdge >= 1.0f && e.nearSquat == 0 && e.frames >= 15 && e.handed == 1.0f && e.jumpOutFlag == 0.0f && e.slewFrames == 0.0f, e.Line());
        // ---- 2026-10-28 bike jump-out: the exit edge is the launch (JumpOutLatch); is the exit handed in from the capture?
        loaded = fresh() && loaded;
        const JumpOutHandRes jh = RunJumpOutHandIn(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()));
        std::printf("[INFO] vehexit [%s] JUMPOUT hand-in: %s\n", name.c_str(), jh.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT hand-in: with a ride capture at the launch edge the exit is handed in (status vehTrans handed 1), MM at weight 1 on the "
                                     "launch feed, visible rotation step <= 15 deg per frame and the hand-in start within 3 deg of the game pose",
              loaded && jh.ok && jh.wEdge >= 1.0f && jh.wMin >= 0.99f && jh.handed == 1.0f && jh.maxVisRotDeg <= 15.0f && jh.handInDevDeg <= 3.0f,
              jh.Line());
        // no ride capture reached the edge (the real game: every exit ran without a hand-in): the animation thread delivers the live game pose of
        // the launch frame, MM goes to weight 1 on the next feed with the inertialized offset game pose -> MM: no fade-in through the game flight
        // pose and no snap (the visible pelvis never steps more than 7 cm per frame)
        loaded = fresh() && loaded;
        const JumpOutHandRes jl = RunJumpOutHandIn(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()), false);
        std::printf("[INFO] vehexit [%s] JUMPOUT live capture: %s\n", name.c_str(), jl.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT live capture: without a ride capture the exit is handed in from the game pose of the launch frame - MM at weight >= 0.99 "
                                     "within 0.1 s of the edge, visible rotation step <= 15 deg per frame and the hand-in start within 3 deg of the game pose the player saw (the step cap follows the hand-in), visible pelvis step <= 7 cm per frame (a 40 cm offset decaying with half-life 0.08 s steps 5.8 cm, a snap 40 cm), status vehTrans handed 1 / no capture 1",
              loaded && jl.ok && jl.handed == 1.0f && jl.noCapture == 1.0f && jl.tFull >= 0.0f && jl.tFull <= 0.10f && jl.maxVisStepCm <= 7.0f &&
                  jl.maxVisRotDeg <= 15.0f && jl.handInDevDeg <= 3.0f,
              jl.Line());
        // 2026-10-28 11-49-37: the game replaced the animated object in the flight (0.33 s after the launch, 6 frames until the same-rig
        // re-arm): those frames show the game's flight pose (the swap jump is the game's, not ours); the re-arm captures the new object
        // and hands in from it - no frame jumps to MM. The published weight stays (the re-arm keeps it; the hold skips the MM blend on
        // the animation thread instead of a weight dip), so the 'weight >= 0.99 after a re-arm' expectation of the checks above holds.
        loaded = fresh() && loaded;
        const JumpOutHandRes jq = RunJumpOutHandIn(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()), false, 20, 6,
                                                   [&] { OnArmed(aRig, aHashes, aParents); });
        std::printf("[INFO] vehexit [%s] JUMPOUT re-arm in the flight: %s | %s\n", name.c_str(), jq.Line().c_str(), jq.RearmLine().c_str());
        // The step is the angular speed of the clip times the frame time (about 15.01 deg at 60 Hz), so the limit sits a little above 15 deg.
        Check("vehexit [" + name + "] JUMPOUT re-arm in the flight (recording 11-49-37): after 6 frames of the game pose on a replaced object the "
                                     "same-rig re-arm hands in from a live capture of it - visible rotation step <= 15.25 deg per frame from the re-arm on "
                                     "(the swap itself >= 40 deg), weight never below 0.99, at least one re-arm hold frame, jumpOutRearms 1",
              loaded && jq.ok && jq.swapJumpDeg >= 40.0f && jq.rearmMaxStepDeg <= 15.25f && jq.wMinAfterRearm >= 0.99f && jq.holdFrames >= 1.0f &&
                  jq.rearms == 1.0f,
              jq.Line() + " | " + jq.RearmLine());
        // r6 launch: the jump-out edge with the denied ride the game runs. The animation frame between the edge and the permission must not show the raw
        // game flight pose (pop filter and ride capture end at the edge): the pinned edge capture holds the shown pose until the hand-in is published.
        // Mutation 30 (no pin) shows the raw pose on that frame, a step of 0.6 x the pop-filter offset away from the shown pose and back at the hand-in.
        // (a real-time 60 Hz replay: a run whose frames around the edge were descheduled says nothing about the edge and is repeated, up to 4 times)
        const auto runDenied = [&](bool aNormal) {
            JumpOutDeniedRes res;
            for (int attempt = 0; attempt < 4; ++attempt)
            {
                loaded = fresh() && loaded;
                res = RunJumpOutDenied(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()), aNormal);
                if (!res.lateEdge)
                    break;
            }
            return res;
        };
        const JumpOutDeniedRes jd = runDenied(false);
        std::printf("[INFO] vehexit [%s] JUMPOUT denied ride: %s\n", name.c_str(), jd.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT launch (r6, recording 06:07:27): the denied ride, the workspot drop 2 frames before the edge and the permission one "
                                     "frame after it - the edge capture is pinned (1 pin, 1 hold end), the visible rotation step on the edge frames <= 15 deg and "
                                     "per 50 ms <= 45 deg, MM at weight >= 0.99 from the first allowed frame, status vehTrans handed 1",
              loaded && jd.ok && !jd.lateEdge && jd.handed == 1.0f && jd.pins == 1.0f && jd.holdEnds == 1.0f && jd.worstEdgeDeg <= 15.0f && jd.worst50Deg <= 45.0f &&
                  jd.wMin >= 0.99f,
              jd.Line());
        // the same ride and the same denied frames with an ordinary unmount: no pin, no hold frame (bit-identical to the code before)
        const JumpOutDeniedRes jnorm = runDenied(true);
        std::printf("[INFO] vehexit [%s] normal unmount, denied ride: %s\n", name.c_str(), jnorm.Line().c_str());
        Check("vehexit [" + name + "] JUMPOUT launch control: an ordinary unmount of the same seated ride pins nothing (0 pins, 0 hold frames, 0 hold ends)",
              loaded && jnorm.ok && jnorm.pins == 0.0f && jnorm.holdFrames == 0.0f && jnorm.holdEnds == 0.0f, jnorm.Line());
        // mounted pop filter, with the vehicle hold's denial (the game) and with the permission
        for (const bool allows : {false, true})
        {
            loaded = fresh() && loaded;
            const RidePopRes rp = RunRidePops(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size()), allows);
            std::printf("[INFO] vehexit [%s] ride pops (permission %d): %s\n", name.c_str(), allows ? 1 : 0, rp.Line().c_str());
            Check("vehexit [" + name + "] ride pops (recording 11-49-37, permission " + std::to_string(allows ? 1 : 0) + "): the game's 86 / 71 deg one-frame "
                                         "jumps of the seated pose are inertialized (visible step <= 15 deg), the game pose untouched before them and at a 20 deg "
                                         "step, back on the game pose (<= 0.5 deg) 0.3 s after the last pop, 2 pops",
                  loaded && rp.ok && rp.rawPopDeg >= 80.0f && rp.maxVisStepDeg <= 15.0f && rp.untouchedBefore && rp.untouchedSmall &&
                      rp.endDevDeg >= 0.0f && rp.endDevDeg <= 0.5f && rp.pops == 2.0f,
                  rp.Line());
        }
        // ---- review fixes (vehicle batch review round F1)
        const uint64_t rigU = reinterpret_cast<uint64_t>(aRig);
        const uint32_t rigN = uint32_t(aHashes.size());
        // vehdll MMDriver:3307: aborts mid-ride never restart the mount transition
        loaded = fresh() && loaded;
        const RideRes ride = RunRideAborts(aRig, aHashes, aParents);
        std::printf("[INFO] vehexit [%s] P2 ride aborts: %s\n", name.c_str(), ride.Line().c_str());
        Check("vehexit [" + name + "] P2 ride (review vehdll MMDriver:3307): an in-car scene, a same-rig re-arm and a settings re-push mid-ride at "
                                     "18 m/s - one mount transition, no transition write and no root offset mid-ride, the exit still handed in",
              loaded && ride.ok && ride.vt[0] == 1.0f && ride.midFeeds >= 40 && ride.midWrites == 0 && ride.midRoot == 0 && ride.vt[1] == 1.0f &&
                  ride.vt[2] == 0.0f,
              ride.Line());
        // vehdll MMDriver:2612: a same-rig re-arm 0.13 s after the unmount / in the exit state keeps the hand-in and the weight
        loaded = fresh() && loaded;
        const ExitHandRes ra = RunExitHandIn(rigU, rigN, [&](int aPhase, int aI) {
            if (aPhase == 1 && aI == 8)
                OnArmed(aRig, aHashes, aParents);
        });
        std::printf("[INFO] vehexit [%s] P2 exit + re-arm +0.13 s: %s\n", name.c_str(), ra.Line().c_str());
        Check("vehexit [" + name + "] P2 exit (review vehdll MMDriver:2612): a same-rig re-arm 0.13 s after the unmount keeps the hand-in and the "
                                     "weight (never below 0.99 within 0.3 s, no frame at the squat), status vehTransKept 1",
              loaded && ra.ok && ra.wEdge >= 1.0f && ra.wMin >= 0.99f && ra.nearSquat == 0 && ra.handed == 1.0f && ra.kept == 1.0f, ra.Line());
        loaded = fresh() && loaded;
        const ExitHandRes rb = RunExitHandIn(rigU, rigN, [&](int aPhase, int aI) {
            if (aPhase == 0 && aI == 26)
                OnArmed(aRig, aHashes, aParents);
        });
        std::printf("[INFO] vehexit [%s] P2 exit state re-arm: %s\n", name.c_str(), rb.Line().c_str());
        Check("vehexit [" + name + "] P2 exit: a same-rig re-arm in the exit state (vehicle bit still set, 0.17 s before the unmount) - the exit is "
                                     "still captured and handed in",
              loaded && rb.ok && rb.wEdge >= 1.0f && rb.nearSquat == 0 && rb.handed == 1.0f && rb.kept == 1.0f, rb.Line());
        // negative: another rig inside the exit window resets as before (weight 0 at once)
        static int s_otherRig = 0;
        loaded = fresh() && loaded;
        const ExitHandRes rc = RunExitHandIn(rigU, rigN, [&](int aPhase, int aI) {
            if (aPhase == 1 && aI == 8)
                OnArmed(&s_otherRig, aHashes, aParents);
        });
        std::printf("[INFO] vehexit [%s] P2 exit + other rig: %s\n", name.c_str(), rc.Line().c_str());
        Check("vehexit [" + name + "] P2 exit negative: a re-arm with another rig inside the exit window still resets the weight (0 at once)",
              loaded && rc.ok && rc.wMin <= 0.0f && rc.kept == 0.0f, rc.Line());
        // on foot: a same-rig re-arm resets the weight as before (identity)
        loaded = fresh() && loaded;
        {
            FeedInput in = MountFeed();
            for (int i = 0; i < 120; ++i)
                Feed(in);
            vti::PoseSnapshot s0, s1;
            const bool before = ReadSnap(s0) && s0.weight >= 0.99f && utpp::g_hot.mmActive.load();
            OnArmed(aRig, aHashes, aParents);
            const bool after = ReadSnap(s1) && s1.weight == 0.0f && !utpp::g_hot.mmActive.load();
            Check("vehexit [" + name + "] P2 on foot: a same-rig re-arm resets the weight to 0 as before (no vehicle state, vehTransKept 0)",
                  loaded && before && after && Field(Status(), "vehTransKept") == 0.0f,
                  "before " + std::to_string(s0.weight) + " after " + std::to_string(s1.weight));
        }
        // vehdll VehicleTransition:366: the production roles (game names of the frozen snapshot's map) - the whole bent leg chain
        loaded = fresh() && loaded;
        const LegRes legs = RunMountLegs(rigU, aHashes);
        std::printf("[INFO] vehexit [%s] P2 mount legs: %s\n", name.c_str(), legs.Line().c_str());
        Check("vehexit [" + name + "] P2 mount (review vehdll VehicleTransition:366): seated legs 40 deg (UpLeg) / 10 deg (Leg) off - the whole leg "
                                     "chain on the game pose after the hold (shin and thigh < 0.5 deg), the upper body keeps its offsets",
              loaded && legs.ok && legs.offsetFrames >= 3 && legs.shinMaxDeg < 0.5f && legs.thighMaxDeg < 0.5f && legs.upperMaxDeg > 5.0f, legs.Line());
        // on foot: no transition, no capture, FkSide leaves the records untouched
        loaded = fresh() && loaded;
        const VxRes f = Run({VxStep{.seconds = 0.5f, .speed = 1.0f}}, false);
        Check("vehexit [" + name + "] P2 on foot: vehtrans not wanted, FkSide writes nothing, status vehTrans=0/0/0/0",
              loaded && !utpp::mm::vehtrans::Wanted() && f.status.find(" vehTrans=0/0/0/0 vehTransActive=0") != std::string::npos &&
                  FkSideNoop(reinterpret_cast<uint64_t>(aRig), uint32_t(aHashes.size())),
              f.status.substr(f.status.find(" vehTrans="), 40));
    }
    SetAirRobust(false);
    SetAir(3);
    SetLogger(&TraceLog);
}

// 0.6.3 (Section063) turn rate - test 2026-10-12 (Locomotion body turn, a former test database): "run turns play strangely slowed
// down, the character slides in slow motion". Night traces (the recorded log camera_whip_180 / flick_180_run):
// Run_Turn_R_180_Rfoot f=21 cost 83.6 |v| 3.80 rate 0.70, Run_Turn_L_135_Rfoot rate 0.81 (stick held: rate = loop rate = game /
// database loop speed x clip factor) while the capsule kept 3.8 m/s through the turn; offset root at its 30 cm clamp, foot
// corrections 20 cm. Game model: production switches (0.5.9 capsule sync / catch-up / trajectory calibration, 0.6.0), game-like
// capsule (g_game059); whips with the velocity turning at constant speed (Locomotion in the traces), a flick through the stick
// centre. Modes: 0 = turn rate off (0.6.2), 1 = on. Turn segment = plant of a turn-class clip (root below 65 % of the clip's peak
// root speed; measured here from the database, independent of the driver's weight).
std::map<std::string, std::pair<int, int>> TurnSegments(const utpp::mm::Database& aDb)
{
    using namespace utpp::mm::steer;
    std::map<std::string, std::pair<int, int>> out;
    const auto clips = aDb.Clips();
    const float fps = aDb.Fps() > 0.0f ? aDb.Fps() : 30.0f;
    for (uint32_t c = 0; c < clips.size(); ++c)
    {
        const std::string name(aDb.ClipName(c));
        if (!IsTurnClass(DriverClipClass(name, ClassifyClip(name))))
            continue;
        std::vector<float> sp;
        for (uint32_t f = 0; f < clips[c].numFrames; ++f)
        {
            const auto* r = aDb.Root(clips[c].firstFrame + f);
            sp.push_back(r ? std::sqrt(r->dx * r->dx + r->dy * r->dy) * fps : 0.0f);
        }
        if (sp.empty())
            continue;
        const float peak = *std::max_element(sp.begin(), sp.end());
        if (peak < 1.0f)
            continue;
        int a = -1, b = -1;
        for (int f = 0; f < int(sp.size()); ++f)
            if (sp[size_t(f)] < 0.65f * peak)
            {
                if (a < 0)
                    a = f;
                b = f;
            }
        if (a >= 0)
            out[name] = {a, b};
    }
    return out;
}

std::string Line063(const Metrics& m)
{
    char buf[320];
    std::snprintf(buf, sizeof buf,
                  "turnSeg %d fr rate min %.2f mean %.2f low(<0.9) %d | corrMax %.1f cm (seg %.1f) offSat24 %.2f s offMax %.1f cm (seg %.1f) costMax %.1f trans %d",
                  m.turnSegFrames, m.turnSegFrames ? m.turnSegRateMin : 0.0f, m.TurnSegRateMean(), m.turnSegLow, m.corrMax, m.turnSegCorrMax,
                  m.offSat24, m.offMax, m.turnSegOffMax, m.costMax, m.transitions);
    return buf;
}

void Section063(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("Section063");
    using S = Step;
    {
        // 0.6.4: shipped default max stays 1.1 (the in-game 1.4 comes from the CET setting); 1.4 must be accepted too.
        const bool okDef = SetTurnRateParams(0.9f, 1.4f) && SetTurnRateParams(0.9f, 1.1f);
        const bool bad = !SetTurnRateParams(std::nanf(""), 1.4f) && !SetTurnRateParams(0.4f, 1.4f) && !SetTurnRateParams(1.5f, 1.4f) &&
                         !SetTurnRateParams(0.9f, 2.5f);
        Check("0.6.4 turn rate params: 0.9 / 1.4 and the shipped 0.9 / 1.1 accepted, NaN / min < 0.5 / min > max / max > 2 rejected", okDef && bad);
        ApplyTurnRateEnv();
    }
    // run whip 180 (camera_whip_180: stick forward, the target swings 180 deg, the capsule turns its velocity at constant speed)
    const std::vector<Step> w180 = {S{.name = "accel", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "cruise", .seconds = 2.0f, .stick = 1.0f},
                                    S{.name = "whip", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .rotateVel = true},
                                    S{.name = "run_on", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "stop", .seconds = 2.0f, .stick = 0.0f}};
    // run whip 135 to the right
    const std::vector<Step> w135 = {S{.name = "accel", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "cruise", .seconds = 2.0f, .stick = 1.0f},
                                    S{.name = "whip", .seconds = 2.0f, .stick = 1.0f, .angleDeg = -135.0f, .rotateVel = true},
                                    S{.name = "run_on", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "stop", .seconds = 2.0f, .stick = 0.0f}};
    // run flick 180 through the stick centre (2 frames at 0; the capsule brakes through zero)
    const std::vector<Step> f180 = {S{.name = "accel", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "cruise", .seconds = 2.0f, .stick = 1.0f},
                                    S{.name = "whip", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .zeroFrames = 2},
                                    S{.name = "run_on", .seconds = 1.5f, .stick = 1.0f},
                                    S{.name = "stop", .seconds = 2.0f, .stick = 0.0f}};
    const std::vector<Step>* sc[3] = {&w180, &w135, &f180};
    const char* scName[3] = {"run_whip_180", "run_whip_135", "run_flick_180"};
    std::string table;
    int dbs = 0, dbsTurn = 0, negSlow = 0;
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        std::map<std::string, std::pair<int, int>> segs;
        std::string err;
        {
            utpp::mm::Database tdb; // released before the driver loads its copy
            if (tdb.LoadFromFile(argv[di], kExpectedRigHash, err))
                segs = TurnSegments(tdb);
        }
        const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
        if (segs.empty() || !LoadDatabase(p, name))
        {
            Check("0.6.3 database loaded (turn segments measured): " + name, false, argv[di] + std::string(" ") + err);
            continue;
        }
        ++dbs;
        g_turnSeg = segs;
        OnArmed(aRig, aHashes, aParents);
        g_game059 = true;
        std::map<std::string, Metrics> r[2][3];
        std::string st;
        ConfigureMode(2);
        Set059(true, true, true);
        Set060(true);
        (void)RunScenario(*sc[0], -82.0f, true); // warm-up (driver filters after the database load), as in Section059
        for (int v = 0; v < 2; ++v)
        {
            ConfigureMode(2);
            Set059(true, true, true);
            Set060(true);
            SetTurnRate(v == 1);
            for (int k = 0; k < 3; ++k)
            {
                r[v][k] = RunScenario(*sc[k], -82.0f, true);
                const Metrics& wm = r[v][k].at("whip");
                std::printf("[INFO] 0.6.3 %s %s %s/whip: %s\n", name.c_str(), v ? "on" : "off", scName[k], Line063(wm).c_str());
                std::printf("[INFO] 0.6.3 %s %s %s/whip picks: %s\n", name.c_str(), v ? "on" : "off", scName[k], wm.transList.c_str());
                const Metrics& ro = r[v][k].at("run_on");
                std::printf("[INFO] 0.6.3 %s %s %s/run_on: %s\n", name.c_str(), v ? "on" : "off", scName[k], Line063(ro).c_str());
            }
            if (v == 1)
                st = Status();
        }
        g_game059 = false;
        g_turnSegNames.clear();
        for (const auto& kv : g_turnSeg)
            g_turnSegNames.insert(kv.first);
        g_turnSeg.clear();
        ConfigureMode(2); // turn rate / 0.5.9 / 0.6.0 switches off again for the other sections
        // aggregate over the whip / run_on steps
        int seg[2] = {}, low[2] = {};
        float sum[2] = {}, rmin[2] = {9, 9}, corr[2] = {}, sat[2] = {}, offMax[2] = {}, cost[2] = {};
        int trans[2] = {}, capOver[2] = {};
        bool sameBefore = true;
        std::string beforeDiff;
        for (int v = 0; v < 2; ++v)
            for (int k = 0; k < 3; ++k)
            {
                for (const char* stp : {"whip", "run_on"})
                {
                    const Metrics& m = r[v][k].at(stp);
                    seg[v] += m.turnSegFrames;
                    low[v] += m.turnSegLow;
                    sum[v] += m.turnSegRateSum;
                    if (m.turnSegFrames > 0)
                        rmin[v] = std::min(rmin[v], m.turnSegRateMin);
                    corr[v] = std::max(corr[v], m.corrMax);
                    sat[v] += m.offSat24;
                    offMax[v] = std::max(offMax[v], m.offMax);
                    cost[v] = std::max(cost[v], m.visCostMax);
                    trans[v] += m.transitions;
                    capOver[v] += m.capOver;
                }
                if (v == 1)
                    for (const char* stp : {"accel", "cruise"})
                    {
                        const Metrics& a = r[0][k].at(stp);
                        const Metrics& b = r[1][k].at(stp);
                        bool turnClip = false; // a turn-class clip in the run-in is legitimately raised
                        for (const std::string& cl : b.clips)
                            turnClip = turnClip || g_turnSegNames.count(cl) > 0;
                        if (!turnClip && (a.rateMin != b.rateMin || a.rateMax != b.rateMax || a.corrMax != b.corrMax ||
                                          a.transitions != b.transitions || a.offMax != b.offMax))
                        {
                            sameBefore = false;
                            beforeDiff += std::string(scName[k]) + "/" + stp + ": " + a.Line() + " -> " + b.Line() + "; ";
                        }
                    }
            }
        const float mean[2] = {seg[0] ? sum[0] / float(seg[0]) : 0.0f, seg[1] ? sum[1] / float(seg[1]) : 0.0f};
        char line[512];
        for (int v = 0; v < 2; ++v)
        {
            std::snprintf(line, sizeof line,
                          "  %-28s %-3s turnSeg %4d fr rate min %.2f mean %.2f low %3d | corrMax %5.1f cm offSat24 %.2f s offMax %5.1f cm visCostMax %6.1f capOver %d trans %d\n",
                          name.c_str(), v ? "on" : "off", seg[v], seg[v] ? rmin[v] : 0.0f, mean[v], low[v], corr[v], sat[v], offMax[v], cost[v],
                          capOver[v], trans[v]);
            table += line;
        }
        {
            const size_t q = st.find(" turnRateOn=");
            std::printf("[INFO] 0.6.3 %s status:%s\n", name.c_str(), q == std::string::npos ? " (none)" : st.substr(q, 140).c_str());
        }
        Check("0.6.3 [" + name + "] status reports the turn rate (on, raised in the whips)",
              st.find("turnRateOn=1") != std::string::npos && Field(st, "turnRateFeeds") > 0.0f);
        Check("0.6.3 [" + name + "] run-in (accel / cruise) without turn-class clips unchanged: the turn rate acts only on turn-class clips",
              sameBefore, beforeDiff);
        if (seg[1] == 0)
        {
            std::printf("[INFO] 0.6.3 %s: no turn-class plant frames in the whips (no checks on the rate)\n", name.c_str());
            continue;
        }
        ++dbsTurn;
        if (seg[0] > 0 && mean[0] < 0.85f)
            ++negSlow;
        Check("0.6.3 [" + name + "] turn plants at run speed: playback rate mean >= 0.95, >= 85 % of the frames at >= 0.9 (0.6.2 mean " +
                  std::to_string(mean[0]) + ")",
              mean[1] >= 0.95f && float(low[1]) <= 0.15f * float(seg[1]),
              "mean " + std::to_string(mean[1]) + " min " + std::to_string(rmin[1]) + " low " + std::to_string(low[1]) + "/" +
                  std::to_string(seg[1]));
        // the whip model itself gives 14.5-15.9 cm foot corrections with 0.6.2 (the capsule turns its velocity at 360 deg/s at a
        // constant speed while any clip brakes into the plant); the turn rate must not add slip and must shorten the time the
        // body is dragged at the offset clamp
        Check("0.6.3 [" + name + "] foot slip bounded: max foot-lock correction <= 16.5 cm and not above 0.6.2 + 1 cm; offset >= 24 cm "
                                 "not longer than 0.6.2",
              corr[1] <= 16.5f && corr[1] <= corr[0] + 1.0f && sat[1] <= sat[0] + 1e-4f,
              "corr " + std::to_string(corr[0]) + " -> " + std::to_string(corr[1]) + " offSat24 " + std::to_string(sat[0]) + " -> " +
                  std::to_string(sat[1]));
        Check("0.6.3 [" + name + "] selections above the cost cap not more than 0.6.2, transitions not above 0.6.2 + 15 %",
              capOver[1] <= capOver[0] && float(trans[1]) <= float(trans[0]) * 1.15f + 1.0f,
              "capOver " + std::to_string(capOver[0]) + " -> " + std::to_string(capOver[1]) + " trans " + std::to_string(trans[0]) + " -> " + std::to_string(trans[1]));
    }
    std::printf("[INFO] 0.6.3 turn-rate table (whip + run_on steps of the 3 scenarios):\n%s", table.c_str());
    Check("0.6.3 negative control: turn rate off (0.6.2) plays the turn plants at a mean rate < 0.85 (>= 1 database with turn plants)",
          negSlow >= 1 || dbsTurn == 0, std::to_string(negSlow) + "/" + std::to_string(dbsTurn));
    Check("0.6.3 turn-class plants reached in the whips on at least one database", dbsTurn >= 1 || dbs == 0,
          std::to_string(dbsTurn) + "/" + std::to_string(dbs));
    SetTurnRate(false);
}

// ================================================================ loco C0 + C1a (2026-10-13)
// C1a crouch clip-set exclusion (UTPPSetMMCrouchIdleExclude, default on; the older sections run with it off and keep their numbers).
// The default exclusion list is empty; the section drives the mechanism with one motion tag (kC1aTag: the crouch stop clips, which
// the standstill hold re-enters about once per second). crouch_idle_hold: crouch, stick released 10 s, body turned +-30 deg in place, idle again - with the exclusion none of the
// listed clips is picked and the standstill rule holds. Calibration digest identical with / without the exclusion; fallback when no
// crouch idle loop would remain; exact tag names only; a runtime toggle gives the data of a fresh load (loco P0 recompute hook).
// Databases: the arguments.
// C0 feet diagnostics (UTPPSetMMFeetDiag, default on): one "MMTRACE feet" line per TRANS / tick line, per-foot counters adding
// up to the totals, one "footstep" line per drift unlock, selections identical with the diagnostics off.
void ConfigureShippedC1a()
{
    ConfigureMode(2);
    Set059(true, true, true);
    Set060(true);
    SetUpperMask(true);
    SetHoldPose(true);
    SetTurnRate(true);
}

uint64_t CrouchMaskOfDigest(const std::string& aDigest)
{
    const size_t p = aDigest.find("masks=");
    const size_t s = p == std::string::npos ? p : aDigest.find('/', p);
    return s == std::string::npos ? 0ull : std::strtoull(aDigest.c_str() + s + 1, nullptr, 16);
}

// motion clip names start with "M_Neutral_" (the package databases hold nothing else); a clip of any other name is a non-motion clip.
bool IsAddedClipName(const std::string& aClip)
{
    return !aClip.empty() && aClip.rfind("M_Neutral_", 0) != 0;
}
// aList: clip names separated by spaces (a name may carry a suffix after the name)
bool ListHasAddedClip(const std::string& aList)
{
    size_t p = 0;
    while (p < aList.size())
    {
        size_t e = aList.find(' ', p);
        if (e == std::string::npos)
            e = aList.size();
        if (e > p && IsAddedClipName(aList.substr(p, e - p)))
            return true;
        p = e + 1;
    }
    return false;
}

const char* const kC1aTag = "PSD_Dense_Crouch_Walk_Stops"; // motion tag the section lists in the crouch exclusion
// speed-0 motion tag for the calibration check (the exclusion must not move the speed calibration; the crouch stops carry speed and do)
const char* const kC1aCalTag = "PSD_Dense_Crouch_TurnInPlace";
std::set<std::string> g_c1aExcluded;                        // names of the clips carrying kC1aTag (filled per database)

struct HoldC1a
{
    int exclPicks = 0;     // selections (TRANS) of the clips of the listed tag after c_enter
    int idleSwitches = 0;  // standstill selections (idle / stop class: the standstill rule's set) whose source family differs from
                           // the previous one (from the clip playing at the end of c_enter)
    int famChanges = 0;    // status clip changes between source families after c_enter
    int idleStill = 0;     // non-idle / non-stop selections at standstill after c_enter
    std::string endClip;   // clip playing at the end of c_hold
    std::string idles;     // idle selections after c_enter
    // c_hold (10 s, no input): selections as "clip@frame xN", their count and the shortest gap between two of them (s). The
    // motion crouch standstill re-enters the tail of M_Neutral_Crouch_Stop_* about once per second (also at loco P0); a much
    // shorter gap would be a visible pose loop.
    std::string holdPicks;
    int holdTrans = 0;
    float holdMinGap = -1.0f; // < 0: fewer than two selections
};

HoldC1a RunHoldC1a()
{
    using S = Step;
    const std::vector<Step> hold = {S{.name = "c_enter", .seconds = 1.5f, .stick = 0.0f, .loco = 2},
                                    S{.name = "c_hold", .seconds = 10.0f, .stick = 0.0f, .loco = 2},
                                    S{.name = "c_pan_l", .seconds = 1.0f, .stick = 0.0f, .loco = 2, .bodyRate = 30.0f},
                                    S{.name = "c_pan_r", .seconds = 2.0f, .stick = 0.0f, .loco = 2, .bodyRate = -30.0f},
                                    S{.name = "c_pan_back", .seconds = 1.0f, .stick = 0.0f, .loco = 2, .bodyRate = 30.0f},
                                    S{.name = "c_hold2", .seconds = 4.0f, .stick = 0.0f, .loco = 2}};
    const size_t t0 = g_trans.size();
    const float c0 = g_clock;
    const auto r = RunScenario(hold, 98.0f, true);
    HoldC1a h;
    for (const auto& kv : r)
        if (kv.first != "c_enter")
        {
            h.famChanges += kv.second.famChanges;
            h.idleStill += kv.second.idleStill;
        }
    h.endClip = r.at("c_hold").endClip;
    const auto still = [](const std::string& c) { const std::string k = ClassName(c); return k == "Idle" || k == "Stop"; };
    std::string prevIdle = still(r.at("c_enter").endClip) ? r.at("c_enter").endClip : std::string();
    for (size_t k = t0; k < g_trans.size() && k < g_transTime.size(); ++k)
    {
        if (g_transTime[k] - c0 < 1.5f - 1e-4f)
            continue;
        const std::string& cl = g_trans[k].first;
        if (g_c1aExcluded.count(cl) != 0)
            ++h.exclPicks;
        if (!still(cl))
            continue;
        if (!prevIdle.empty() && utpp::mm::ClipFamily(prevIdle) != utpp::mm::ClipFamily(cl))
            ++h.idleSwitches;
        if (ClassName(cl) == "Idle" && h.idles.size() < 400 && (h.idles.empty() || cl != prevIdle))
            h.idles += cl + " ";
        prevIdle = cl;
    }
    {
        std::vector<std::pair<std::string, int>> picks;
        float last = -1.0f;
        for (size_t k = t0; k < g_trans.size() && k < g_transTime.size(); ++k)
        {
            const float t = g_transTime[k] - c0;
            if (t < 1.5f - 1e-4f || t >= 11.5f - 1e-4f)
                continue;
            const std::string key = g_trans[k].first + "@" + std::to_string(k < g_transFrame.size() ? g_transFrame[k] : -1);
            auto it = std::find_if(picks.begin(), picks.end(), [&](const auto& p) { return p.first == key; });
            if (it == picks.end())
                picks.emplace_back(key, 1);
            else
                ++it->second;
            if (last >= 0.0f && (h.holdMinGap < 0.0f || t - last < h.holdMinGap))
                h.holdMinGap = t - last;
            last = t;
            ++h.holdTrans;
        }
        for (const auto& p : picks)
            h.holdPicks += p.first + " x" + std::to_string(p.second) + " ";
    }
    return h;
}

void SectionC0C1a(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionC0C1a");
    std::vector<std::string> paths;
    for (int di = 1; di < argc; ++di)
        paths.push_back(argv[di]);
    int negEx = 0, negDbs = 0, tagDbDone = 0, feetDone = 0;
    for (const std::string& path : paths)
    {
        std::string name = path;
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        // name oracle from the file: the clips of the listed motion tag, the motion crouch idle loop
        utpp::mm::Database tdb;
        std::string err;
        if (!tdb.LoadFromFile(path.c_str(), kExpectedRigHash, err))
        {
            Check("C1a database read: " + name, false, err);
            continue;
        }
        std::vector<uint32_t> exClips;
        int32_t motionIdle = -1;
        uint64_t tagBit = 0;
        for (uint32_t t = 0; t < tdb.NumTags() && t < 64; ++t)
            if (tdb.TagName(t) == kC1aTag)
                tagBit = 1ull << t;
        g_c1aExcluded.clear();
        for (uint32_t c = 0; c < tdb.NumClips(); ++c)
        {
            const std::string_view n = tdb.ClipName(c);
            if (tagBit != 0 && (tdb.Clips()[c].tagMask & tagBit) != 0)
            {
                exClips.push_back(c);
                g_c1aExcluded.insert(std::string(n));
            }
            if (n == "M_Neutral_Crouch_Idle_Loop")
                motionIdle = int32_t(c);
        }
        const std::wstring wp(path.begin(), path.end());
        // ---- exclusion off: fresh load
        ConfigureShippedC1a();
        SetCrouchIdleExclude(false);
        SetCrouchExcludeTags("");
        g_exclLines.clear();
        if (!LoadDatabase(wp, name))
        {
            Check("C1a database loaded: " + name, false, path);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a();
        const std::string digOff = DerivedClassDataDigest(), calOff = CalibrationDigest();
        const bool noLineOff = g_exclLines.empty();
        const HoldC1a hOff = RunHoldC1a();
        // ---- calibration reference: a speed-0 tag listed (clips without root speed leave the speed calibration alone), fresh load
        SetCrouchExcludeTags(kC1aCalTag);
        SetCrouchIdleExclude(true);
        if (!LoadDatabase(wp, name))
        {
            Check("C1a database loaded: " + name, false, path);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a();
        const std::string calSpeed0 = CalibrationDigest();
        // ---- exclusion on (the listed motion tag): fresh load
        SetCrouchExcludeTags(kC1aTag);
        SetCrouchIdleExclude(true);
        g_exclLines.clear();
        if (!LoadDatabase(wp, name))
        {
            Check("C1a database loaded: " + name, false, path);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a();
        const std::string digOn = DerivedClassDataDigest(), calOn = CalibrationDigest();
        const std::string stOn = Status();
        const std::string loadLine = g_exclLines.empty() ? std::string() : g_exclLines.front();
        const HoldC1a hOn = RunHoldC1a();
        const uint64_t maskOff = CrouchMaskOfDigest(digOff), maskOn = CrouchMaskOfDigest(digOn);
        int exAllowedOn = 0;
        for (uint32_t c : exClips)
            exAllowedOn += (tdb.Clips()[c].tagMask & maskOn) ? 1 : 0;
        const bool motionIdleOn = motionIdle >= 0 && (tdb.Clips()[size_t(motionIdle)].tagMask & maskOn) != 0;
        const int exclClips = int(Field(stOn, "exclClips") + 0.5f);
        const bool skipped = Field(stOn, "exclSkipped") > 0.5f, active = Field(stOn, "exclActive") > 0.5f;
        std::printf("[INFO] C1a %s: listed-tag clips %zu motion set idle %d | off: exclPicks %d idleSwitches %d famChanges %d idleStill %d end %s | "
                    "idles %s\n",
                    name.c_str(), exClips.size(), motionIdle >= 0 ? 1 : 0, hOff.exclPicks, hOff.idleSwitches, hOff.famChanges, hOff.idleStill,
                    hOff.endClip.c_str(), hOff.idles.c_str());
        std::printf("[INFO] C1a %s: on: exclPicks %d idleSwitches %d famChanges %d idleStill %d end %s | idles %s\n", name.c_str(),
                    hOn.exclPicks, hOn.idleSwitches, hOn.famChanges, hOn.idleStill, hOn.endClip.c_str(), hOn.idles.c_str());
        std::printf("[INFO] C1a %s: c_hold 10 s selections off: %d (min gap %.2f s) %s| on: %d (min gap %.2f s) %s\n", name.c_str(),
                    hOff.holdTrans, double(hOff.holdMinGap), hOff.holdPicks.c_str(), hOn.holdTrans, double(hOn.holdMinGap),
                    hOn.holdPicks.c_str());
        std::printf("[INFO] C1a %s: crouch mask off 0x%llX on 0x%llX exclClips %d skipped %d | %s\n", name.c_str(),
                    static_cast<unsigned long long>(maskOff), static_cast<unsigned long long>(maskOn), exclClips, skipped ? 1 : 0,
                    loadLine.c_str());
        Check("C1a [" + name + "] calibration unchanged by the crouch exclusion of a speed-0 tag (speed calibration, family rates incl. "
                                "per-clip factors, capsule profile, bands, speed maps)",
              calSpeed0 == calOff, calSpeed0 == calOff ? std::string() : "off " + calOff + " | speed-0 tag " + calSpeed0);
        std::printf("[INFO] C1a %s: calibration with the crouch stops excluded %s the calibration without exclusion\n", name.c_str(),
                    calOn == calOff ? "equals" : "differs from");
        if (!exClips.empty() && motionIdle >= 0)
        {
            ++negDbs;
            negEx += hOff.exclPicks + hOff.idleSwitches > 0 ? 1 : 0;
            Check("C1a [" + name + "] exclusion on: the " + std::to_string(exClips.size()) +
                      " clips of the listed Neutral tag out of the crouch set, the Neutral crouch idle in, logged once at the load",
                  active && !skipped && exclClips == int(exClips.size()) && exAllowedOn == 0 && motionIdleOn && noLineOff &&
                      g_exclLines.size() >= 1 && loadLine.find("(load): on") != std::string::npos,
                  "exclClips " + std::to_string(exclClips) + " allowed " + std::to_string(exAllowedOn) + " | " + loadLine);
            const std::string endClass = ClassName(hOn.endClip);
            // the 10 s standstill (c_hold) with the exclusion re-enters the tail of the motion crouch stop about once per second
            // (the motion idle loop is not selected either way - INFO line above): no re-entry loop faster than 0.5 s
            Check("C1a [" + name + "] crouch_idle_hold: no clip of the listed tag picked, no standstill family switch, standstill rule "
                                    "kept (Neutral idle / stop clip playing), no 10 s standstill re-entry within 0.5 s",
                  hOn.exclPicks == 0 && hOn.idleSwitches == 0 && hOn.famChanges == 0 && hOn.idleStill == 0 &&
                      (endClass == "Idle" || endClass == "Stop") &&
                      utpp::mm::ClipFamily(hOn.endClip) == utpp::mm::ClipFamily("M_Neutral_Crouch_Idle_Loop") &&
                      (hOn.holdTrans < 2 || hOn.holdMinGap >= 0.5f),
                  "on: end " + hOn.endClip + " idles " + hOn.idles + "hold " + hOn.holdPicks + "| off: exclPicks " +
                      std::to_string(hOff.exclPicks) + " switches " + std::to_string(hOff.idleSwitches) + " hold " + hOff.holdPicks);
        }
        else
            Check("C1a [" + name + "] the database has the listed Neutral tag " + kC1aTag + " and the Neutral crouch idle loop", false,
                  std::to_string(exClips.size()) + " clips, idle " + std::to_string(motionIdle));
        {
            // runtime toggle = fresh load (loco P0 recompute hook)
            SetCrouchIdleExclude(false);
            const std::string dT0 = DerivedClassDataDigest();
            SetCrouchIdleExclude(true);
            const std::string dT1 = DerivedClassDataDigest();
            Check("C1a [" + name + "] runtime toggle off / on gives the class data of a fresh load without / with the exclusion",
                  dT0 == digOff && dT1 == digOn, dT0 == digOff ? dT1 : dT0);
        }
        if (tagDbDone == 0 && !exClips.empty() && motionIdle >= 0)
        {
            ++tagDbDone;
            const auto crouchMask = [] { return CrouchMaskOfDigest(DerivedClassDataDigest()); };
            const bool sub = SetCrouchExcludeTags("Crouch_Idle");
            const std::string s1 = Status();
            const uint64_t m1 = crouchMask();
            const bool one = SetCrouchExcludeTags(std::string(" ") + kC1aTag + " ");
            const std::string s2 = Status(), t2 = CrouchExcludeTags();
            const std::string both2 = std::string(kC1aTag) + ",PSD_Dense_Crouch_Idles";
            const bool both = SetCrouchExcludeTags(both2);
            const std::string s3 = Status();
            const uint64_t m3 = crouchMask();
            const std::string l3 = g_exclLines.empty() ? std::string() : g_exclLines.back();
            const bool bad = !SetCrouchExcludeTags("Crouch Tag;Idle") && CrouchExcludeTags() == both2;
            const bool def = SetCrouchExcludeTags("") && CrouchExcludeTags().empty();
            const std::string s4 = Status();
            Check("C1a [" + name + "] exact tag names: 'Crouch_Idle' matches no tag (Neutral PSD_Dense_Crouch_Idles kept), one name "
                                    "-> its clips, Neutral idle tag listed too -> skipped (no crouch idle loop left), invalid list rejected, "
                                    "'' = default (empty list, nothing excluded)",
                  sub && Field(s1, "exclActive") < 0.5f && Field(s1, "exclMissing") > 0.5f && m1 == maskOff && one && t2 == kC1aTag &&
                      int(Field(s2, "exclClips") + 0.5f) == int(exClips.size()) && both && Field(s3, "exclSkipped") > 0.5f && m3 == maskOff &&
                      l3.find("skipped") != std::string::npos && bad && def && int(Field(s4, "exclClips") + 0.5f) == 0 &&
                      Field(s4, "exclActive") < 0.5f,
                  l3);
        }
        if (feetDone == 0 && motionIdle >= 0)
        {
            ++feetDone;
            using S = Step;
            const std::vector<Step> fs = {S{.name = "f_idle", .seconds = 1.0f, .stick = 0.0f, .loco = 2},
                                          S{.name = "f_start", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
                                          S{.name = "f_dash", .seconds = 0.4f, .stick = 1.0f, .loco = 2, .speed = 8.0f},
                                          S{.name = "f_walk", .seconds = 1.0f, .stick = 1.0f, .loco = 2},
                                          S{.name = "f_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2},
                                          S{.name = "f_turn", .seconds = 1.5f, .stick = 1.0f, .angleDeg = 120.0f, .loco = 2},
                                          S{.name = "f_stop2", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
            std::vector<std::string> seq[2];
            size_t feetN[2] = {0, 0}, stepN[2] = {0, 0};
            int mainN[2] = {0, 0};
            std::string st[2];
            for (int v = 0; v < 2; ++v)
            {
                // same start for both runs: fresh load, the switches, one warm-up run with the diagnostics off
                if (!LoadDatabase(wp, name))
                    break;
                OnArmed(aRig, aHashes, aParents);
                SetEnabled(true);
                ConfigureShippedC1a();
                SetFeetDiag(false);
                (void)RunScenario(fs, 98.0f, true);
                SetFeetDiag(v == 1);
                const size_t t0 = g_trans.size(), f0 = g_feetLines.size(), s0 = g_footstepLines.size();
                const int m0 = g_mainTraceLines;
                (void)RunScenario(fs, 98.0f, true);
                for (size_t k = t0; k < g_trans.size(); ++k)
                {
                    char b[64];
                    std::snprintf(b, sizeof b, " f=%d c=%.3f", k < g_transFrame.size() ? g_transFrame[k] : -1, double(g_trans[k].second));
                    seq[v].push_back(g_trans[k].first + b);
                }
                feetN[v] = g_feetLines.size() - f0;
                stepN[v] = g_footstepLines.size() - s0;
                mainN[v] = g_mainTraceLines - m0;
                st[v] = Status();
            }
            const std::string& so = st[1];
            const float dL = Field(so, "driftL"), dR = Field(so, "driftR");
            // loco C1b: with replant on (UTPP_LOCO_C1A_ALL run: shipped default crouch) the replant steps are foot events as well
            const float sL = Field(so, "stepsL"), sR = Field(so, "stepsR");
            const bool replantOn = FootReplantMask() != 0;
            bool parse = feetN[1] > 0;
            for (size_t k = g_feetLines.size() - feetN[1]; k < g_feetLines.size(); ++k)
            {
                const std::string& l = g_feetLines[k];
                const size_t pl = l.find(" L="), pr = l.find(" R=");
                parse = parse && pl != std::string::npos && pr != std::string::npos && std::strchr("PRFS", l[pl + 3]) &&
                        std::strchr("PRFS", l[pr + 3]) && l.find(" offL=") != std::string::npos && l.find(" stepsR=") != std::string::npos &&
                        l.find(" crouchExcl=") != std::string::npos;
            }
            // (the second run starts fresh: SetEnabled re-inits the matcher and the foot locker, the counters cover that run only)
            bool steps = stepN[1] == size_t(dL + dR + (replantOn ? sL + sR : 0.0f) + 0.5f);
            size_t sideN[2] = {0, 0}, driftN = 0; // footstep lines per side=L / side=R (must match the per-foot counters)
            for (size_t k = g_footstepLines.size() - stepN[1]; k < g_footstepLines.size(); ++k)
            {
                const bool drift = g_footstepLines[k].find(" reason=drift ") != std::string::npos;
                const bool step = g_footstepLines[k].find(" reason=radius ") != std::string::npos ||
                                  g_footstepLines[k].find(" reason=clamp ") != std::string::npos ||
                                  g_footstepLines[k].find(" reason=tidy ") != std::string::npos ||
                                  g_footstepLines[k].find(" reason=handover ") != std::string::npos;
                steps = steps && ((drift && Field(g_footstepLines[k], "dist") >= 25.0f) || (replantOn && step));
                driftN += drift ? 1 : 0;
                sideN[0] += g_footstepLines[k].find(" side=L ") != std::string::npos ? 1 : 0;
                sideN[1] += g_footstepLines[k].find(" side=R ") != std::string::npos ? 1 : 0;
            }
            const bool sides = sideN[0] == size_t(dL + (replantOn ? sL : 0.0f) + 0.5f) && sideN[1] == size_t(dR + (replantOn ? sR : 0.0f) + 0.5f) &&
                               driftN == size_t(dL + dR + 0.5f);
            std::printf("[INFO] C0 %s feet: TRANS %zu/%zu main lines %d/%d feet lines %zu/%zu footstep lines %zu/%zu | %s\n", name.c_str(),
                        seq[0].size(), seq[1].size(), mainN[0], mainN[1], feetN[0], feetN[1], stepN[0], stepN[1],
                        g_feetLines.empty() ? "" : g_feetLines.back().c_str());
            if (!g_footstepLines.empty())
                std::printf("[INFO] C0 %s last footstep: %s\n", name.c_str(), g_footstepLines.back().c_str());
            {
                const size_t p = so.find(" feet=\"");
                std::printf("[INFO] C0 %s status:%s\n", name.c_str(), p == std::string::npos ? " (none)" : so.substr(p).c_str());
            }
            std::string firstDiff;
            for (size_t k = 0; k < std::max(seq[0].size(), seq[1].size()) && firstDiff.empty(); ++k)
                if (k >= seq[0].size() || k >= seq[1].size() || seq[0][k] != seq[1][k])
                    firstDiff = "#" + std::to_string(k) + " off " + (k < seq[0].size() ? seq[0][k] : "-") + " | on " +
                                (k < seq[1].size() ? seq[1][k] : "-");
            Check("C0 [" + name + "] feet diagnostics: selections identical with the diagnostics off / on; one feet line per MMTRACE "
                                  "TRANS / tick line (none when off); lines parse",
                  seq[0] == seq[1] && !seq[1].empty() && feetN[0] == 0 && stepN[0] == 0 && feetN[1] == size_t(mainN[1]) && parse,
                  firstDiff);
            Check("C0 [" + name + "] per-foot counters add up to the totals (slips, drift unlocks; replant steps: 0 with replant off, the "
                                  "steps total with replant on), status segment only with the per-foot part when on",
                  Field(so, "slipsL") + Field(so, "slipsR") == Field(so, "footSlips") && dL + dR == Field(so, "flDriftUnlocks") &&
                      (replantOn ? sL + sR == Field(so, "steps") : sL + sR == 0.0f) && st[0].find(" slipsL=") == std::string::npos &&
                      st[0].find("diag=0") != std::string::npos && so.find("diag=1") != std::string::npos &&
                      Field(st[0], "flDriftUnlocks") == Field(so, "flDriftUnlocks"),
                  "slips " + std::to_string(Field(so, "slipsL")) + "+" + std::to_string(Field(so, "slipsR")) + " / " +
                      std::to_string(Field(so, "footSlips")) + " drift " + std::to_string(dL) + "+" + std::to_string(dR) + " / " +
                      std::to_string(Field(so, "flDriftUnlocks")));
            Check("C0 [" + name + "] footstep lines: one per drift unlock (reason=drift, dist >= 25 cm), side=L / side=R counts equal "
                                  "the per-foot drift counters, at least one in the dash",
                  steps && sides && stepN[1] >= 1,
                  std::to_string(stepN[1]) + " lines (side=L " + std::to_string(sideN[0]) + ", side=R " + std::to_string(sideN[1]) +
                      "), drift L " + std::to_string(dL) + " R " + std::to_string(dR));
            SetFeetDiag(true);
        }
    }
    Check("C1a negative control (exclusion off = loco P0): crouch_idle_hold picks clips of the listed tag / switches the standstill "
          "family (some database)",
          negDbs >= 1 && negEx >= 1, std::to_string(negEx) + "/" + std::to_string(negDbs) + " databases");
    // back to the switches of the older sections
    SetCrouchExcludeTags("");
    ConfigureMode(2);
}

// ================================================================ loco C1b (2026-10-13): plant / replant (UTPPSetMMFootReplant)
// Real driver, shipped switches, feet diagnostics on. Per feed (g_feedProbe): the published snapshot's toe bones (model space by FK)
// placed in the world with the integrated capsule position and the fed player yaw - the same world frame as the foot locker - and
// the feet phases / IK weights / corrections of the status feet segment. Planted toe = phase P on this and the previous feed at IK
// weight 1. Scenarios: crouch start / stop in 8 directions relative to the body, the crouch idle hold (exclusion off),
// stance changes while planted, crouch walk / pivot / turn / arc (correction p95 against replant off),
// stand scenarios (replant crouch-only: published pose identical to replant off, feed for feed).
int g_replantMain = 0; // replant mask of the older sections (0; UTPP_LOCO_C1A_ALL=1: the shipped 2)
// m/s, displayed toe world speed in P / S: regression cap (review fix; not the design's absolute bound). Measured with replant crouch:
// crouch start / stop x8 4.5-5.1 m/s in the C1b-only run, 5.1-6.1 m/s in the full suite (driver state left by the older sections);
// replant off 3.0-3.9 m/s
constexpr double kC1bToeCap = 7.5;

struct FeetProbe
{
    int32_t toe[2] = {-1, -1};
    std::vector<int16_t> parents;
    double px = 0.0, py = 0.0;
    bool havePrev = false;
    double prevW[2][2] = {};
    char prevPhase[2] = {'F', 'F'};
    float prevWeight[2] = {0.0f, 0.0f};
    std::vector<double> planted;  // m/s, planted toe world speed
    double plantedMax = 0.0;      // m/s
    double lockedMax = 0.0;       // m/s, toe world speed in any locked phase (P / S) on both feeds at weight 1
    std::vector<double> corr;     // cm, corrections of locked feet while the capsule moves (>= 0.3 m/s)
    double corrMax = 0.0;         // cm, any locked foot
    int simultaneous = 0;         // feeds with both feet in a step (phase S)
    int stepFeeds = 0;            // feeds with a foot in phase S
    std::vector<uint64_t> hashes; // published pose per feed
    std::vector<float> stepsAt;   // cumulative replant steps (status) per feed
    void Clear()
    {
        px = py = 0.0;
        havePrev = false;
        prevPhase[0] = prevPhase[1] = 'F';
        prevWeight[0] = prevWeight[1] = 0.0f;
        planted.clear();
        plantedMax = lockedMax = corrMax = 0.0;
        corr.clear();
        simultaneous = stepFeeds = 0;
        hashes.clear();
        stepsAt.clear();
    }
};
FeetProbe g_fp;

float FieldAfter(const std::string& s, size_t aFrom, const char* key)
{
    if (aFrom == std::string::npos)
        return 0.0f;
    const std::string k = std::string(" ") + key + "=";
    const size_t p = s.find(k, aFrom);
    return p == std::string::npos ? 0.0f : float(std::atof(s.c_str() + p + k.size()));
}
char PhaseAfter(const std::string& s, size_t aFrom, const char* key)
{
    if (aFrom == std::string::npos)
        return '?';
    const std::string k = std::string(" ") + key + "=";
    const size_t p = s.find(k, aFrom);
    return p == std::string::npos || p + k.size() >= s.size() ? '?' : s[p + k.size()];
}

void C1bProbe(const FeedInput& aIn, const std::string& aStatus)
{
    using namespace utpp::mm;
    using namespace utpp::mm::integ;
    FeetProbe& fp = g_fp;
    const double dt = std::fmin(double(aIn.dt), 0.25);
    fp.px += double(aIn.velWorld[0]) * dt;
    fp.py += double(aIn.velWorld[1]) * dt;
    static PoseSnapshot snap{};
    if (!utpp::g_mmPose.Read(snap))
        return;
    {
        uint64_t h = 1469598103934665603ull;
        const auto mix = [&](const void* p, size_t n)
        {
            const auto* b = static_cast<const uint8_t*>(p);
            for (size_t i = 0; i < n; ++i)
            {
                h ^= b[i];
                h *= 1099511628211ull;
            }
        };
        mix(&snap.weight, sizeof snap.weight);
        mix(snap.pose, sizeof(float) * 7 * std::min<uint32_t>(snap.numBones, kMaxMMBones));
        mix(&snap.upperScale, sizeof snap.upperScale);
        fp.hashes.push_back(h);
    }
    const size_t feet = aStatus.find(" feet=\"");
    fp.stepsAt.push_back(FieldAfter(aStatus, feet, "stepsL") + FieldAfter(aStatus, feet, "stepsR"));
    const char ph[2] = {PhaseAfter(aStatus, feet, "L"), PhaseAfter(aStatus, feet, "R")};
    const float w[2] = {FieldAfter(aStatus, feet, "wL"), FieldAfter(aStatus, feet, "wR")};
    const float c[2] = {FieldAfter(aStatus, feet, "corrL"), FieldAfter(aStatus, feet, "corrR")};
    const double yaw = double(aIn.playerYawDeg) * double(kDegT);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double speed = std::hypot(double(aIn.velWorld[0]), double(aIn.velWorld[1]));
    if (ph[0] == 'S' || ph[1] == 'S')
        ++fp.stepFeeds;
    if (ph[0] == 'S' && ph[1] == 'S')
        ++fp.simultaneous;
    for (int s = 0; s < 2; ++s)
    {
        if (fp.toe[s] < 0 || uint32_t(fp.toe[s]) >= snap.numBones)
            continue;
        // model-space toe by FK of the published local pose
        int32_t chain[64];
        int n = 0;
        for (int32_t b = fp.toe[s]; b >= 0 && n < 64; b = fp.parents[size_t(b)])
            chain[n++] = b;
        math::Vec3 p{};
        math::Quat q{};
        for (int i = n - 1; i >= 0; --i)
        {
            const float* l = &snap.pose[size_t(chain[i]) * 7];
            p = p + math::QuatRotate(q, math::Vec3{l[0], l[1], l[2]});
            q = math::QuatMul(q, math::Quat{l[3], l[4], l[5], l[6]});
        }
        const double wx = fp.px + cy * double(p.x) - sy * double(p.y);
        const double wy = fp.py + sy * double(p.x) + cy * double(p.y);
        const bool locked = ph[s] == 'P' || ph[s] == 'S';
        if (locked)
        {
            fp.corrMax = std::max(fp.corrMax, double(c[s]));
            if (speed >= 0.3)
                fp.corr.push_back(double(c[s]));
        }
        if (fp.havePrev && dt > 0.0 && w[s] >= 1.0f && fp.prevWeight[s] >= 1.0f)
        {
            const double v = std::hypot(wx - fp.prevW[s][0], wy - fp.prevW[s][1]) / dt;
            if (locked && (fp.prevPhase[s] == 'P' || fp.prevPhase[s] == 'S'))
                fp.lockedMax = std::max(fp.lockedMax, v);
            if (ph[s] == 'P' && fp.prevPhase[s] == 'P')
            {
                fp.planted.push_back(v);
                fp.plantedMax = std::max(fp.plantedMax, v);
            }
        }
        fp.prevW[s][0] = wx;
        fp.prevW[s][1] = wy;
        fp.prevPhase[s] = ph[s];
        fp.prevWeight[s] = w[s];
    }
    fp.havePrev = true;
}

double Pct(std::vector<double> v, double aP)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(aP * double(v.size() - 1) + 0.5))];
}

struct C1bRun
{
    std::vector<double> planted, corr;
    double plantedMax = 0, lockedMax = 0, corrMax = 0;
    int simultaneous = 0, stepFeeds = 0;
    float steps = 0, drift = 0, slips = 0, handovers = 0, simSteps = 0, maxLift = 0;
    float reasons[4] = {0, 0, 0, 0}; // radius, clamp, tidy, handover
    std::vector<uint64_t> hashes;
    std::string trans;
    size_t stepLine0 = 0, feetLine0 = 0; // g_footstepLines / g_feetLines before the probed run
    std::string segSteps;                // replant steps per scenario step ("name:n ")
    std::string Line() const
    {
        char b[420];
        std::snprintf(b, sizeof b,
                      "planted p95 %.2f max %.1f cm/s, locked max %.2f m/s, corr p95 %.1f max %.1f cm, steps %.0f (r %.0f c %.0f t %.0f h %.0f), "
                      "handovers %.0f, drift %.0f, slips %.0f, simult %d/%.0f, lift %.1f cm",
                      Pct(planted, 0.95) * 100.0, plantedMax * 100.0, lockedMax, Pct(corr, 0.95), corrMax, double(steps),
                      double(reasons[0]), double(reasons[1]), double(reasons[2]), double(reasons[3]), double(handovers), double(drift),
                      double(slips), simultaneous, double(simSteps), double(maxLift));
        return b;
    }
};

// One measured run from the same start for every mode: fresh database load, the shipped switches, a warm-up run of the same steps
// (as SectionC0C1a), then the probed run.
struct C1bSetup
{
    std::wstring path;
    std::string name;
    const std::vector<uint64_t>* hashes = nullptr;
    const std::vector<int16_t>* parents = nullptr;
    void* rig = nullptr;
};

C1bRun RunC1b(const C1bSetup& aSu, const std::vector<Step>& aSteps, float aBodyYaw, int32_t aMask, bool aExclude = true)
{
    C1bRun r;
    if (!LoadDatabase(aSu.path, aSu.name))
        return r;
    OnArmed(aSu.rig, *aSu.hashes, *aSu.parents);
    SetEnabled(true);
    ConfigureShippedC1a();
    SetCrouchIdleExclude(aExclude);
    SetFeetDiag(true);
    SetFootReplant(aMask);
    (void)RunScenario(aSteps, aBodyYaw, true); // warm-up (driver state after the load)
    g_fp.Clear();
    g_feedProbe = &C1bProbe;
    const size_t t0 = g_trans.size();
    r.stepLine0 = g_footstepLines.size();
    r.feetLine0 = g_feetLines.size();
    (void)RunScenario(aSteps, aBodyYaw, true); // fresh matcher / foot locker
    g_feedProbe = nullptr;
    r.planted = g_fp.planted;
    r.corr = g_fp.corr;
    r.plantedMax = g_fp.plantedMax;
    r.lockedMax = g_fp.lockedMax;
    r.corrMax = g_fp.corrMax;
    r.simultaneous = g_fp.simultaneous;
    r.stepFeeds = g_fp.stepFeeds;
    r.hashes = g_fp.hashes;
    {
        // replant steps per scenario step (the probe sees the step feeds in order; frames as RunScenario)
        size_t at = 0;
        for (const Step& s : aSteps)
        {
            const size_t frames = size_t(s.seconds * 60.0f + 0.5f);
            const float a = at > 0 && at - 1 < g_fp.stepsAt.size() ? g_fp.stepsAt[at - 1] : 0.0f;
            at += frames;
            const float b = at > 0 && at - 1 < g_fp.stepsAt.size() ? g_fp.stepsAt[at - 1] : a;
            char seg[96];
            std::snprintf(seg, sizeof seg, "%s:%.0f ", s.name.c_str(), double(b - a));
            r.segSteps += seg;
        }
    }
    const std::string st = Status();
    const size_t feet = st.find(" feet=\"");
    r.steps = FieldAfter(st, feet, "stepsL") + FieldAfter(st, feet, "stepsR");
    r.drift = FieldAfter(st, feet, "driftL") + FieldAfter(st, feet, "driftR");
    r.slips = FieldAfter(st, feet, "slipsL") + FieldAfter(st, feet, "slipsR");
    r.handovers = FieldAfter(st, feet, "handovers");
    r.simSteps = FieldAfter(st, feet, "simSteps");
    r.maxLift = FieldAfter(st, feet, "maxLift");
    r.reasons[0] = FieldAfter(st, feet, "stepsRadius");
    r.reasons[1] = FieldAfter(st, feet, "stepsClamp");
    r.reasons[2] = FieldAfter(st, feet, "stepsTidy");
    r.reasons[3] = FieldAfter(st, feet, "stepsHandover");
    for (size_t k = t0; k < g_trans.size(); ++k)
        r.trans += g_trans[k].first + "@" + std::to_string(k < g_transFrame.size() ? g_transFrame[k] : -1) + ";";
    return r;
}

void SectionC1b(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionC1b");
    using S = Step;
    {
        // diagnostic tuning: UTPP_LOCO_REPLANT=<radius>,<stopsRadius>,<ratio>,<stopsRatio>,<stepSpeed>,<stopsStepSpeed>,<lift>
        char e[160] = {};
        float a[7] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_REPLANT", e, sizeof e) > 0 &&
            sscanf_s(e, "%f,%f,%f,%f,%f,%f,%f", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6]) == 7)
            std::printf("[INFO] C1b replant params %s -> %d\n", e, SetFootReplantParams(a[0], a[1], a[2], a[3], a[4], a[5], a[6]) ? 1 : 0);
    }
    std::vector<std::string> paths;
    for (int di = 1; di < argc; ++di)
        paths.push_back(argv[di]);
    // crouch start / stop in 8 directions relative to the body: each start turns the target by rel[d] from the previous one (the
    // body faces the previous target after the stop)
    const float rel[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -135.0f, -90.0f, -45.0f};
    std::vector<Step> dirs = {S{.name = "c_idle", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    for (int d = 0; d < 8; ++d)
    {
        dirs.push_back(S{.name = "c_start" + std::to_string(d), .seconds = 2.0f, .stick = 1.0f, .angleDeg = rel[d], .loco = 2});
        dirs.push_back(S{.name = "c_stop" + std::to_string(d), .seconds = 2.0f, .stick = 0.0f, .loco = 2});
    }
    const std::vector<Step> hold = {S{.name = "c_enter", .seconds = 1.5f, .stick = 0.0f, .loco = 2},
                                    S{.name = "c_hold", .seconds = 10.0f, .stick = 0.0f, .loco = 2},
                                    S{.name = "c_pan_l", .seconds = 1.0f, .stick = 0.0f, .loco = 2, .bodyRate = 30.0f},
                                    S{.name = "c_pan_r", .seconds = 2.0f, .stick = 0.0f, .loco = 2, .bodyRate = -30.0f},
                                    S{.name = "c_pan_back", .seconds = 1.0f, .stick = 0.0f, .loco = 2, .bodyRate = 30.0f},
                                    S{.name = "c_hold2", .seconds = 4.0f, .stick = 0.0f, .loco = 2}};
    const std::vector<Step> stance = {S{.name = "s_idle", .seconds = 1.5f, .stick = 0.0f, .loco = 1},
                                      S{.name = "c_down", .seconds = 2.0f, .stick = 0.0f, .loco = 2},
                                      S{.name = "s_up", .seconds = 2.0f, .stick = 0.0f, .loco = 1},
                                      S{.name = "c_down2", .seconds = 2.0f, .stick = 0.0f, .loco = 2, .bodyRate = 20.0f},
                                      S{.name = "s_up2", .seconds = 2.0f, .stick = 0.0f, .loco = 1}};
    const std::vector<Step> walk = {S{.name = "c_idle", .seconds = 1.0f, .stick = 0.0f, .loco = 2},
                                    S{.name = "c_walk", .seconds = 3.0f, .stick = 1.0f, .loco = 2},
                                    S{.name = "c_pivot", .seconds = 2.5f, .stick = 1.0f, .angleDeg = 180.0f, .loco = 2},
                                    S{.name = "c_turn90", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 90.0f, .loco = 2},
                                    S{.name = "c_arc", .seconds = 3.0f, .stick = 1.0f, .angleRate = 60.0f, .loco = 2},
                                    S{.name = "c_half", .seconds = 2.0f, .stick = 0.6f, .angleDeg = -135.0f, .loco = 2},
                                    S{.name = "c_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    const std::vector<Step> stand = {S{.name = "s_idle", .seconds = 1.0f, .stick = 0.0f},
                                     S{.name = "s_start", .seconds = 2.5f, .stick = 1.0f},
                                     S{.name = "s_flick", .seconds = 2.5f, .stick = 1.0f, .angleDeg = 180.0f, .zeroFrames = 2},
                                     S{.name = "s_arc", .seconds = 3.0f, .stick = 1.0f, .angleRate = 90.0f},
                                     S{.name = "s_slow", .seconds = 2.0f, .stick = 0.47f, .wobble = 0.07f},
                                     S{.name = "s_sprint", .seconds = 2.0f, .stick = 1.0f, .loco = 3},
                                     S{.name = "s_stop", .seconds = 2.0f, .stick = 0.0f}};
    const std::vector<Step> trace = {S{.name = "c_idle", .seconds = 1.5f, .stick = 0.0f, .loco = 2},
                                     S{.name = "c_start", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 90.0f, .loco = 2},
                                     S{.name = "c_stop", .seconds = 2.0f, .stick = 0.0f, .loco = 2},
                                     S{.name = "s_up", .seconds = 1.5f, .stick = 0.0f, .loco = 1}};
    int dbs = 0, negPlanted = 0, crouchDbs = 0;
    std::string walkTable;
    for (const std::string& path : paths)
    {
        std::string name = path;
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        utpp::mm::Database tdb;
        std::string err;
        if (!tdb.LoadFromFile(path.c_str(), kExpectedRigHash, err))
        {
            Check("C1b database read: " + name, false, err);
            continue;
        }
        bool crouchClips = false;
        for (uint32_t c = 0; c < tdb.NumClips() && !crouchClips; ++c)
            crouchClips = tdb.ClipName(c).find("Crouch_Start") != std::string_view::npos;
        g_fp.toe[0] = g_fp.toe[1] = -1;
        g_fp.parents.assign(tdb.NumBones(), -1);
        for (uint32_t b = 0; b < tdb.NumBones(); ++b)
        {
            g_fp.parents[b] = int16_t(tdb.BoneParent(b));
            if (tdb.BoneName(b) == "LeftToeBase")
                g_fp.toe[0] = int32_t(b);
            if (tdb.BoneName(b) == "RightToeBase")
                g_fp.toe[1] = int32_t(b);
        }
        C1bSetup su;
        su.path = std::wstring(path.begin(), path.end());
        su.name = name;
        su.hashes = &aHashes;
        su.parents = &aParents;
        su.rig = aRig;
        ++dbs;
        if (crouchClips)
        {
            ++crouchDbs;
            // ---- (A) crouch start / stop x8
            {
                const C1bRun off = RunC1b(su, dirs, -82.0f, 0), on = RunC1b(su, dirs, -82.0f, 2);
                std::printf("[INFO] C1b %s crouch start / stop x8 off: %s\n", name.c_str(), off.Line().c_str());
                std::printf("[INFO] C1b %s crouch start / stop x8 crouch: %s | steps per segment: %s\n", name.c_str(), on.Line().c_str(),
                            on.segSteps.c_str());
                negPlanted += Pct(off.planted, 0.95) > Pct(on.planted, 0.95) + 0.01 ? 1 : 0;
                // review fix: displayed toe world speed in P / S (locked max): the step criterion is relative to the animated toe
                // (hurry x3), so it exceeds replant off (crouch 4.5-6.1 vs 3.0-3.9 m/s); regression cap 7.5 m/s, not the design's
                // absolute bound (deviation reported)
                Check("C1b [" + name + "] crouch start / stop x8 (replant crouch): planted toe p95 <= 1 cm/s, never two feet stepping, "
                                        "foot correction <= 15 cm, lift <= 3 cm, locked toe world speed <= 7.5 m/s (regression cap), "
                                        "selections identical to replant off",
                      Pct(on.planted, 0.95) <= 0.01 && on.simultaneous == 0 && on.simSteps == 0.0f && on.corrMax <= 15.0 &&
                          on.maxLift <= 3.05f && on.steps > 0.0f && on.lockedMax <= kC1bToeCap && on.trans == off.trans &&
                          !on.trans.empty(),
                      on.Line() + " | off locked max " + std::to_string(off.lockedMax) + " m/s");
            }
            // ---- (B) crouch idle hold, exclusion off, c_hold of loco C1a
            {
                const C1bRun off = RunC1b(su, hold, 98.0f, 0, false), on = RunC1b(su, hold, 98.0f, 2, false);
                std::printf("[INFO] C1b %s crouch idle hold (exclusion off) off: %s | crouch: %s\n", name.c_str(), off.Line().c_str(),
                            on.Line().c_str());
                Check("C1b [" + name + "] crouch idle hold (exclusion off), replant crouch: planted toe p95 <= "
                                        "1 cm/s, never two feet stepping, correction <= 15 cm, locked toe world speed <= 7.5 m/s (regression "
                                        "cap), selections identical to replant off",
                      Pct(on.planted, 0.95) <= 0.01 && on.simultaneous == 0 && on.simSteps == 0.0f && on.corrMax <= 15.0 &&
                          on.lockedMax <= kC1bToeCap && on.trans == off.trans,
                      on.Line());
            }
            // ---- (C) stance changes while planted (stand idle -> crouch -> stand -> crouch, stick released; replant crouch only)
            {
                const C1bRun off = RunC1b(su, stance, 98.0f, 0), on = RunC1b(su, stance, 98.0f, 2);
                std::printf("[INFO] C1b %s stance changes while planted off: %s | crouch: %s\n", name.c_str(), off.Line().c_str(),
                            on.Line().c_str());
                Check("C1b [" + name + "] stance changes while planted (replant crouch): never two feet stepping, no drift unlock more than "
                                        "replant off, correction <= 15 cm, locked toe world speed <= 7.5 m/s (regression cap), selections "
                                        "identical",
                      on.simultaneous == 0 && on.simSteps == 0.0f && on.drift <= off.drift && on.corrMax <= 15.0 &&
                          on.lockedMax <= kC1bToeCap && on.trans == off.trans,
                      on.Line() + " | off " + off.Line());
            }
            // ---- (D) crouch walk / pivot / turn / arc: correction p95 not worse than replant off
            {
                const C1bRun off = RunC1b(su, walk, -82.0f, 0), on = RunC1b(su, walk, -82.0f, 2);
                char b[400];
                std::snprintf(b, sizeof b,
                              "  %-28s corr p95 off %.2f crouch %.2f cm | max off %.1f crouch %.1f cm | planted p95 off %.1f crouch %.1f cm/s | "
                              "steps %.0f drift %.0f/%.0f\n",
                              name.c_str(), Pct(off.corr, 0.95), Pct(on.corr, 0.95), off.corrMax, on.corrMax, Pct(off.planted, 0.95) * 100.0,
                              Pct(on.planted, 0.95) * 100.0, double(on.steps), double(off.drift), double(on.drift));
                walkTable += b;
                walkTable += "    steps per segment: " + on.segSteps + "\n";
                Check("C1b [" + name + "] crouch walk / pivot / turn / arc (replant crouch): correction p95 not above replant off, max <= "
                                        "15 cm, never two feet stepping, selections identical",
                      Pct(on.corr, 0.95) <= Pct(off.corr, 0.95) + 1e-6 && on.corrMax <= 15.0 && on.simultaneous == 0 && on.trans == off.trans,
                      on.Line() + " | off " + off.Line());
            }
            // ---- (G) review fix: crouch hold states in the shipped configuration (hold pose on): sprint -> crouch slide entry at
            // 10.5 m/s -> slide (loco 0 + PSM detail, 8.3 m/s, the stance bit is off: handover) -> crouch walk; crouch walk -> dodge
            // (loco 0, 6 m/s) -> crouch walk. Replant crouch: correction not above replant off (+ 0.5 cm), no more drift unlocks,
            // never two feet stepping, selections identical
            {
                const std::vector<Step> slide = {S{.name = "g_sprint", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
                                                 S{.name = "g_entry", .seconds = 0.1f, .stick = 1.0f, .loco = 2, .speed = 10.5f},
                                                 S{.name = "g_slide", .seconds = 1.3f, .stick = 1.0f, .loco = 0, .psm = 0x40u | (6u << 16),
                                                   .speed = 8.3f},
                                                 S{.name = "g_crouch", .seconds = 1.5f, .stick = 1.0f, .loco = 2}};
                const std::vector<Step> dodge = {S{.name = "g_cwalk", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
                                                 S{.name = "g_dodge", .seconds = 0.4f, .stick = 1.0f, .loco = 0, .psm = 0x40u, .speed = 6.0f},
                                                 S{.name = "g_cwalk2", .seconds = 1.5f, .stick = 1.0f, .loco = 2}};
                // driver state not reset by a database load (end of the previous scenario) can change the sprint selections of
                // these runs (seen on a former test database): one discarded replant-off run first, so both measured runs start
                // after the same scenario
                (void)RunC1b(su, slide, -82.0f, 0);
                const C1bRun so = RunC1b(su, slide, -82.0f, 0), sn = RunC1b(su, slide, -82.0f, 2);
                (void)RunC1b(su, dodge, -82.0f, 0);
                const C1bRun dof = RunC1b(su, dodge, -82.0f, 0), don = RunC1b(su, dodge, -82.0f, 2);
                std::printf("[INFO] C1b %s crouch slide off: %s | crouch: %s | steps per segment: %s\n", name.c_str(), so.Line().c_str(),
                            sn.Line().c_str(), sn.segSteps.c_str());
                std::printf("[INFO] C1b %s crouch dodge off: %s | crouch: %s | steps per segment: %s\n", name.c_str(), dof.Line().c_str(),
                            don.Line().c_str(), don.segSteps.c_str());
                Check("C1b [" + name + "] crouch slide / dodge (shipped hold pose, replant crouch): correction not above replant off "
                                        "(+0.5 cm), no more drift unlocks, never two feet stepping, selections identical",
                      !so.hashes.empty() && sn.corrMax <= so.corrMax + 0.5 && sn.drift <= so.drift && sn.simultaneous == 0 &&
                          sn.simSteps == 0.0f && sn.trans == so.trans && !dof.hashes.empty() && don.corrMax <= dof.corrMax + 0.5 &&
                          don.drift <= dof.drift && don.simultaneous == 0 && don.simSteps == 0.0f && don.trans == dof.trans,
                      "slide corr max " + std::to_string(sn.corrMax) + " / off " + std::to_string(so.corrMax) + " cm, drift " +
                          std::to_string(sn.drift) + " / " + std::to_string(so.drift) + "; dodge corr max " + std::to_string(don.corrMax) +
                          " / off " + std::to_string(dof.corrMax) + " cm, drift " + std::to_string(don.drift) + " / " +
                          std::to_string(dof.drift));
            }
            // ---- (F) trace: one footstep line per replant step with its reason, feet lines with the replant fields
            {
                const C1bRun on = RunC1b(su, trace, -82.0f, 2);
                const size_t lines = g_footstepLines.size() - on.stepLine0; // the probed run's lines
                size_t reasonLines = 0, driftLines = 0;
                for (size_t k = on.stepLine0; k < g_footstepLines.size(); ++k)
                {
                    const std::string& l = g_footstepLines[k];
                    driftLines += l.find(" reason=drift ") != std::string::npos ? 1 : 0;
                    reasonLines += (l.find(" reason=radius ") != std::string::npos || l.find(" reason=clamp ") != std::string::npos ||
                                    l.find(" reason=tidy ") != std::string::npos || l.find(" reason=handover ") != std::string::npos)
                                       ? 1
                                       : 0;
                }
                bool fields = g_feetLines.size() > on.feetLine0, phaseS = false;
                for (size_t k = on.feetLine0; k < g_feetLines.size(); ++k)
                {
                    fields = fields && g_feetLines[k].find(" replant=2 active=") != std::string::npos &&
                             g_feetLines[k].find(" liftR=") != std::string::npos;
                    phaseS = phaseS || g_feetLines[k].find(" L=S ") != std::string::npos || g_feetLines[k].find(" R=S ") != std::string::npos;
                }
                const std::string st = Status();
                Check("C1b [" + name + "] trace / status: one footstep line per replant step (reason radius / clamp / tidy / handover), feet "
                                        "lines with the replant fields, status replant=2",
                      lines == reasonLines + driftLines && reasonLines == size_t(on.steps + 0.5f) && driftLines == size_t(on.drift + 0.5f) &&
                          fields && st.find(" replant=2 replantActive=") != std::string::npos,
                      std::to_string(reasonLines) + " step lines / " + std::to_string(on.steps) + " steps, " + std::to_string(driftLines) +
                          " drift lines, phase S in a feet line " + std::to_string(phaseS ? 1 : 0));
            }
        }
        // ---- (E) stand path: replant crouch-only publishes the replant-off pose feed for feed (and 'all' changes it).
        // Review fix: standing replant (mask 1 / 3) is experimental and console-only (not offered in CET): in run starts / 180 deg
        // flicks the waiting foot of the one-foot-at-a-time rule is stretched beyond the 15 cm foot-correction guard of 0.5.9 (20-23 cm,
        // replant off 10.6-14.3 cm) with up to one drift unlock. Gated here: planted toes still, never two feet stepping, lift <= 3 cm,
        // selections identical; the corrections are reported (INFO), not held to the guard.
        {
            const C1bRun off = RunC1b(su, stand, -82.0f, 0), crouch = RunC1b(su, stand, -82.0f, 2), all = RunC1b(su, stand, -82.0f, 3);
            size_t firstDiff = off.hashes.size();
            for (size_t k = 0; k < std::min(off.hashes.size(), crouch.hashes.size()); ++k)
                if (off.hashes[k] != crouch.hashes[k])
                {
                    firstDiff = k;
                    break;
                }
            size_t allDiff = 0;
            for (size_t k = 0; k < std::min(off.hashes.size(), all.hashes.size()); ++k)
                allDiff += off.hashes[k] != all.hashes[k] ? 1 : 0;
            std::printf("[INFO] C1b %s stand path: replant all: %s | %zu of %zu feeds differ from replant off\n", name.c_str(),
                        all.Line().c_str(), allDiff, all.hashes.size());
            Check("C1b [" + name + "] stand path bit-identical: replant crouch publishes the replant-off pose on every stand / sprint feed "
                                    "(replant all changes it)",
                  !off.hashes.empty() && off.hashes.size() == crouch.hashes.size() && firstDiff == off.hashes.size() && crouch.steps == 0.0f &&
                      allDiff > 0 && all.steps > 0.0f,
                  "feeds " + std::to_string(off.hashes.size()) + " first difference " +
                      (firstDiff == off.hashes.size() ? std::string("none") : std::to_string(firstDiff)) + ", replant all: " +
                      std::to_string(allDiff) + " feeds differ");
            std::printf("[INFO] C1b %s stand path (experimental standing replant, console only): correction max %.1f cm (replant off "
                        "%.1f cm, 15 cm guard %s), drift unlocks %.0f (off %.0f)\n",
                        name.c_str(), all.corrMax, off.corrMax, all.corrMax <= 15.0 ? "kept" : "EXCEEDED", double(all.drift),
                        double(off.drift));
            Check("C1b [" + name + "] experimental standing replant (mask 3): planted toe p95 <= 1 cm/s, never two feet stepping, lift <= "
                                    "3 cm, selections identical to replant off (corrections INFO: beyond the 15 cm guard)",
                  Pct(all.planted, 0.95) <= 0.01 && all.simultaneous == 0 && all.simSteps == 0.0f && all.maxLift <= 3.05f &&
                      all.trans == off.trans && !all.trans.empty(),
                  all.Line());
        }
    }
    std::printf("[INFO] C1b crouch walk / pivot / turn / arc correction table:\n%s", walkTable.c_str());
    Check("C1b negative control: replant off moves planted toes in the crouch starts / stops (planted p95 above replant crouch + 1 cm/s) "
          "on some database",
          negPlanted >= 1, std::to_string(negPlanted) + "/" + std::to_string(crouchDbs));
    {
        // natives: mask / params validation (previous kept), defaults back
        const bool m = !SetFootReplant(-1) && !SetFootReplant(4) && SetFootReplant(3) && FootReplantMask() == 3 && SetFootReplant(2);
        const bool p = !SetFootReplantParams(0.2f, 0.14f, 0.2f, 0.75f, 1.0f, 2.5f, 0.03f) &&
                       !SetFootReplantParams(0.12f, 0.14f, std::nanf(""), 0.75f, 1.0f, 2.5f, 0.03f) &&
                       !SetFootReplantParams(0.12f, 0.14f, 0.2f, 0.75f, 0.1f, 2.5f, 0.03f) &&
                       SetFootReplantParams(0.10f, 0.14f, 0.2f, 0.75f, 1.0f, 2.5f, 0.03f);
        Check("C1b natives: replant mask 0..3 only, params radius <= 0.15 / NaN / step speed < 0.2 rejected, valid accepted", m && p);
    }
    // back to the defaults and the switches of the older sections
    {
        const utpp::mm::FootReplantConfig d{};
        SetFootReplantParams(d.radius, d.radiusStops, d.ratio, d.ratioStops, d.stepSpeed, d.stepSpeedStops, d.lift);
    }
    SetFootReplant(g_replantMain);
    SetFeetDiag(g_replantMain != 0);
    SetCrouchIdleExclude(g_replantMain != 0);
    ConfigureMode(2);
}

// ---- stage standstill idle (UTPPSetMMIdleHold, default on; the older sections run with it off and keep their numbers)
// In game / batch 1 verification the standstill never reached the looping idle: at the end of the stop clip the end-of-clip search
// (playing frame excluded) re-entered the stop's own tail (M_Neutral_Crouch_Stop_FR_Rtfoot f=153 every 0.96 s) - a small pop
// every second. Scenario per stance (stand / crouch): move 2 s, stick released 10 s, start 1.5 s; fresh load, shipped switches,
// warm-up run, probed run; idle hold off (= batch 1) vs on. On: zero stop selections after the first idle-loop pick, the idle
// loop plays (frames advance, wrap when it is shorter than the time left), planted feet (planted toe speed, max lock correction
// in the still window) not worse than off, the start picks a start clip of the stance within the off latency.
struct IdleHoldRun
{
    int stopPicks = 0;          // stop-class selections in the still window
    int stopAfterIdle = 0;      // ... after the first idle-loop selection
    int idleLoopPicks = 0;      // idle-loop selections in the still window
    float firstIdleAt = -1.0f;  // s into the still window
    std::string firstIdle;
    int loopFeeds = 0, loopAdvances = 0, loopWraps = 0; // still-window feeds playing an idle loop after its first pick
    int loopFrames = 0;         // length of the last idle loop played
    float loopTimeLeft = 0.0f;  // s of the still window left after the first idle-loop pick
    double plantedMax = 0.0, lockedMax = 0.0, corrMax = 0.0; // still window (feet probe)
    double plantedLate = 0.0, lockedLate = 0.0, corrLate = 0.0; // ... from 1 s into it (settled standstill)
    float startLatency = -1.0f; // s from the start step to the first start-class selection
    std::string startClip, firstStartPick, endStillClip, picks;
    float holdSearches = 0.0f;  // status idleHoldSearches (on)
    // tail gate: still-window feeds whose previously playing clip was a non-loop, non-idle clip with more than the gate's 12
    // frames (+2 feed margin) left, and those of them where the stop exclusion was applied anyway (idleHoldSearches advanced)
    int gateFeeds = 0, gateViolations = 0;
};

struct IdleHoldProbeState
{
    float c0 = 0.0f, still0 = 0.0f, still1 = 0.0f;
    bool inStill = false, done = false, late = false;
    // time from the feed count (the suite clock g_clock is a large float by then: its += dt drift moved a selection across the
    // still / start boundary in the full run)
    int feeds = 0;
    size_t transSeen = 0;
    std::vector<float> transT; // feed time of each TRANS from the probed run's first one
    double early[3] = {0.0, 0.0, 0.0};
    const utpp::mm::Database* db = nullptr;
    int32_t loopClip = -1, lastFrame = -1;
    float lastHold = -1.0f; // status idleHoldSearches of the previous still-window feed
    IdleHoldRun* run = nullptr;
    bool idlePicked = false;
};
IdleHoldProbeState g_ih;

int32_t ClipIndexOf(const utpp::mm::Database& aDb, const std::string& aName)
{
    for (uint32_t c = 0; c < aDb.NumClips(); ++c)
        if (aDb.ClipName(c) == aName)
            return int32_t(c);
    return -1;
}
bool IsIdleLoop(const utpp::mm::Database& aDb, int32_t aClip)
{
    return aClip >= 0 && (aDb.Clips()[size_t(aClip)].flags & utpp::mm::kClipLoop) != 0 &&
           utpp::mm::steer::ClassifyClip(aDb.ClipName(uint32_t(aClip))) == utpp::mm::steer::kClassIdle;
}

void IdleHoldProbe(const FeedInput& aIn, const std::string& aStatus)
{
    C1bProbe(aIn, aStatus);
    IdleHoldProbeState& p = g_ih;
    ++p.feeds;
    const float t = float(p.feeds) / 60.0f;
    for (; p.transSeen < g_trans.size(); ++p.transSeen)
        p.transT.push_back(t);
    if (!p.inStill && !p.done && t > p.still0 + 1e-4f)
    {
        p.inStill = true; // feet maxima of the still window only (positions / phases carry on)
        g_fp.plantedMax = g_fp.lockedMax = g_fp.corrMax = 0.0;
    }
    if (!p.inStill)
        return;
    if (!p.late && t > p.still0 + 1.0f + 1e-4f)
    {
        p.late = true; // settled window: maxima of the first second kept aside
        p.early[0] = g_fp.plantedMax;
        p.early[1] = g_fp.lockedMax;
        p.early[2] = g_fp.corrMax;
        g_fp.plantedMax = g_fp.lockedMax = g_fp.corrMax = 0.0;
    }
    const int32_t clip = ClipIndexOf(*p.db, ClipOf(aStatus));
    const int32_t frame = int32_t(Field(aStatus, "clipFrame") + 0.5f);
    p.idlePicked = p.idlePicked || IsIdleLoop(*p.db, clip);
    if (p.idlePicked && IsIdleLoop(*p.db, clip))
    {
        ++p.run->loopFeeds;
        const int32_t n = int32_t(p.db->Clips()[size_t(clip)].numFrames);
        if (clip == p.loopClip && p.lastFrame >= 0)
        {
            if (frame > p.lastFrame)
                ++p.run->loopAdvances;
            else if (frame < p.lastFrame && frame <= 3 && p.lastFrame >= n - 4) // played through its end into the start
                ++p.run->loopWraps;
        }
        p.run->loopFrames = int(n);
    }
    // tail gate: the exclusion decision of this feed reads the clip / frame that played after the previous one
    const float hold = Field(aStatus, "idleHoldSearches");
    if (p.loopClip >= 0 && p.lastHold >= 0.0f)
    {
        const utpp::mm::ClipRecord& pc = p.db->Clips()[size_t(p.loopClip)];
        if ((pc.flags & utpp::mm::kClipLoop) == 0 &&
            utpp::mm::steer::ClassifyClip(p.db->ClipName(uint32_t(p.loopClip))) != utpp::mm::steer::kClassIdle &&
            int32_t(pc.numFrames) - 1 - p.lastFrame > 12 + 2)
        {
            ++p.run->gateFeeds;
            if (hold > p.lastHold + 0.5f)
                ++p.run->gateViolations;
        }
    }
    p.lastHold = hold;
    p.loopClip = clip;
    p.lastFrame = frame;
    p.run->endStillClip = ClipOf(aStatus);
    if (t >= p.still1 - 0.5f / 60.0f)
    {
        p.inStill = false;
        p.done = true;
        p.run->plantedLate = g_fp.plantedMax;
        p.run->lockedLate = g_fp.lockedMax;
        p.run->corrLate = g_fp.corrMax;
        p.run->plantedMax = std::max(p.early[0], g_fp.plantedMax);
        p.run->lockedMax = std::max(p.early[1], g_fp.lockedMax);
        p.run->corrMax = std::max(p.early[2], g_fp.corrMax);
    }
}

// aKind 0: after a stop (move 2 s, release); 1: after a stance change at a standstill (2 s still in the other stance - the batch 1
// reproduction: stand idle -> crouch re-entered M_Neutral_Crouch_Stop_FR_Rtfoot f=153 every 0.96 s)
IdleHoldRun RunIdleHold(const C1bSetup& aSu, const utpp::mm::Database& aDb, int aLoco, int aKind, bool aHold)
{
    using S = Step;
    const float moveS = 2.0f, stillS = 10.0f;
    const std::vector<Step> steps = {aKind == 0 ? S{.name = "ih_move", .seconds = moveS, .stick = 0.82f, .loco = aLoco}
                                               : S{.name = "ih_move", .seconds = moveS, .stick = 0.0f, .loco = aLoco == 2 ? 1 : 2},
                                     S{.name = "ih_still", .seconds = stillS, .stick = 0.0f, .loco = aLoco},
                                     S{.name = "ih_start", .seconds = 1.5f, .stick = 0.82f, .loco = aLoco}};
    IdleHoldRun r;
    if (!LoadDatabase(aSu.path, aSu.name))
        return r;
    OnArmed(aSu.rig, *aSu.hashes, *aSu.parents);
    SetEnabled(true);
    ConfigureShippedC1a();
    SetCrouchIdleExclude(true);
    SetFeetDiag(true);
    SetFootReplant(2);
    SetIdleHold(aHold);
    (void)RunScenario(steps, -82.0f, true); // warm-up (driver state after the load)
    g_fp.Clear();
    g_ih = IdleHoldProbeState{};
    g_ih.db = &aDb;
    g_ih.run = &r;
    // RunScenario settles 90 idle feeds (no probe) before the steps; their selections count at the first step feed
    g_ih.c0 = g_clock;
    g_ih.transSeen = g_trans.size();
    g_ih.still0 = moveS;
    g_ih.still1 = moveS + stillS;
    g_feedProbe = &IdleHoldProbe;
    const size_t t0 = g_trans.size();
    (void)RunScenario(steps, -82.0f, true);
    g_feedProbe = nullptr;
    for (size_t k = t0; k < g_trans.size() && k - t0 < g_ih.transT.size(); ++k)
    {
        const float t = g_ih.transT[k - t0];
        const std::string& cl = g_trans[k].first;
        const int32_t ci = ClipIndexOf(aDb, cl);
        const uint8_t cls = utpp::mm::steer::ClassifyClip(cl);
        if (t > moveS + 1e-4f && t < moveS + stillS - 1e-4f)
        {
            if (r.picks.size() < 600)
                r.picks += cl + "@" + std::to_string(k < g_transFrame.size() ? g_transFrame[k] : -1) + " ";
            if (cls == utpp::mm::steer::kClassStop)
            {
                ++r.stopPicks;
                if (r.firstIdleAt >= 0.0f)
                    ++r.stopAfterIdle;
            }
            if (IsIdleLoop(aDb, ci))
            {
                ++r.idleLoopPicks;
                if (r.firstIdleAt < 0.0f)
                {
                    r.firstIdleAt = t - moveS;
                    r.firstIdle = cl;
                    r.loopTimeLeft = stillS - r.firstIdleAt;
                }
            }
        }
        if (t > moveS + stillS + 1e-4f && r.startLatency < 0.0f && ClassName(cl) == "Start")
        {
            r.startLatency = t - moveS - stillS;
            r.startClip = cl;
        }
        if (t > moveS + stillS + 1e-4f && r.firstStartPick.empty())
            r.firstStartPick = cl;
    }
    const std::string st = Status();
    r.holdSearches = Field(st, "idleHoldSearches");
    return r;
}

void SectionIdleHold(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionIdleHold");
    (void)argc; // C4100
    // databases: utpp_base and the package default utpp_base_dense_full_posture (next to the first argument)
    std::vector<std::string> paths;
    {
        std::string dir = argv[1];
        const size_t sl = dir.find_last_of("\\/");
        dir = sl == std::string::npos ? std::string() : dir.substr(0, sl + 1);
        for (const char* n : {"utpp_base.utppmm", "utpp_base_dense_full_posture.utppmm"})
            if (GetFileAttributesA((dir + n).c_str()) != INVALID_FILE_ATTRIBUTES)
                paths.push_back(dir + n);
    }
    Check("idle hold: databases v7c and default present", paths.size() == 2, std::to_string(paths.size()));
    int negDbs = 0;
    for (const std::string& path : paths)
    {
        std::string name = path.substr(path.find_last_of("\\/") + 1);
        name = name.substr(0, name.rfind('.'));
        utpp::mm::Database tdb;
        std::string err;
        if (!tdb.LoadFromFile(path.c_str(), kExpectedRigHash, err))
        {
            Check("idle hold database read: " + name, false, err);
            continue;
        }
        g_fp.toe[0] = g_fp.toe[1] = -1;
        g_fp.parents.assign(tdb.NumBones(), -1);
        for (uint32_t b = 0; b < tdb.NumBones(); ++b)
        {
            g_fp.parents[b] = int16_t(tdb.BoneParent(b));
            if (tdb.BoneName(b) == "LeftToeBase")
                g_fp.toe[0] = int32_t(b);
            if (tdb.BoneName(b) == "RightToeBase")
                g_fp.toe[1] = int32_t(b);
        }
        C1bSetup su;
        su.path = std::wstring(path.begin(), path.end());
        su.name = name;
        su.hashes = &aHashes;
        su.parents = &aParents;
        su.rig = aRig;
        bool negHere = false;
        for (int sc = 0; sc < 4; ++sc)
        {
            const int loco = (sc & 1) ? 2 : 1, kind = sc >> 1;
            const char* sn = kind == 0 ? (loco == 2 ? "crouch stop" : "stand stop") : (loco == 2 ? "stand->crouch" : "crouch->stand");
            const IdleHoldRun off = RunIdleHold(su, tdb, loco, kind, false);
            const IdleHoldRun on = RunIdleHold(su, tdb, loco, kind, true);
            for (const auto* rr : {&off, &on})
                std::printf("[INFO] idle hold %s %s %s: still picks %s| stop %d (after idle %d) idle loops %d first %s @%.2f s, loop feeds %d "
                            "adv %d wraps %d (len %d, %.2f s left), planted max %.1f cm/s locked max %.2f m/s corr max %.1f cm (from 1 s: %.1f cm/s "
                            "%.2f m/s %.1f cm), end %s, "
                            "start %s %.3f s (first %s), holdSearches %.0f, gate feeds %d (excluded %d)\n",
                            name.c_str(), sn, rr == &off ? "off" : "on", rr->picks.c_str(), rr->stopPicks, rr->stopAfterIdle,
                            rr->idleLoopPicks, rr->firstIdle.c_str(), double(rr->firstIdleAt), rr->loopFeeds, rr->loopAdvances,
                            rr->loopWraps, rr->loopFrames, double(rr->loopTimeLeft), rr->plantedMax * 100.0, rr->lockedMax,
                            rr->corrMax, rr->plantedLate * 100.0, rr->lockedLate, rr->corrLate, rr->endStillClip.c_str(), rr->startClip.c_str(), double(rr->startLatency),
                            rr->firstStartPick.c_str(), double(rr->holdSearches), rr->gateFeeds, rr->gateViolations);
            negHere = negHere || off.stopPicks >= 3 || off.idleLoopPicks == 0;
            const std::string tag = "idle hold [" + name + " " + sn + "] ";
            // the loop wraps when it is shorter than the still time left after its pick (1 s margin), else it keeps advancing
            const bool wrapExpected = on.loopFrames > 0 && float(on.loopFrames) / 30.0f < on.loopTimeLeft - 1.0f;
            Check(tag + "on: idle loop reached, zero stop selections after the first idle-loop pick, the loop plays (frames advance"
                        ", wraps when shorter than the time left), still window ends on an idle",
                  on.idleLoopPicks >= 1 && on.stopAfterIdle == 0 && on.loopAdvances >= int(0.8f * 30.0f * std::fmin(on.loopTimeLeft, 4.0f)) &&
                      (!wrapExpected || on.loopWraps >= 1) && ClassName(on.endStillClip) == "Idle" && on.holdSearches > 0.0f,
                  "picks " + on.picks + "| first " + on.firstIdle + " adv " + std::to_string(on.loopAdvances) + " wraps " +
                      std::to_string(on.loopWraps) + " end " + on.endStillClip);
            // planted feet: not worse than off (+5 % / 0.5 cm tolerance for float noise; a toe speed below 0.1 m/s = 1.7 mm per feed is
            // no jump - the idle loop's own weight shift moves a planted toe ~5 cm/s) from 1 s into the still window; after a stop also
            // over the whole window (after a stance change the first second is the stance transition itself: on goes straight to the
            // idle loop, off through a stop clip - INFO line, not compared)
            const auto notWorse = [](double a, double b, double c, double a0, double b0, double c0)
            { return (a <= a0 * 1.05 + 0.005 || a < 0.1) && (b <= b0 * 1.05 + 0.005 || b < 0.1) && c <= c0 + 0.5; };
            Check(tag + "on: planted feet not worse than off (planted / locked toe speed, max lock correction; settled standstill" +
                      (kind == 0 ? std::string(" and the whole still window)") : std::string(")")),
                  notWorse(on.plantedLate, on.lockedLate, on.corrLate, off.plantedLate, off.lockedLate, off.corrLate) &&
                      (kind != 0 || notWorse(on.plantedMax, on.lockedMax, on.corrMax, off.plantedMax, off.lockedMax, off.corrMax)),
                  "from 1 s: planted " + std::to_string(on.plantedLate) + " / " + std::to_string(off.plantedLate) + " locked " +
                      std::to_string(on.lockedLate) + " / " + std::to_string(off.lockedLate) + " corr " + std::to_string(on.corrLate) +
                      " / " + std::to_string(off.corrLate) + " | whole: planted " + std::to_string(on.plantedMax) + " / " +
                      std::to_string(off.plantedMax) + " corr " + std::to_string(on.corrMax) + " / " + std::to_string(off.corrMax));
            // tail gate: a stop (or other non-loop, non-idle) clip with more than 12 frames left is never cut by the stop exclusion;
            // after a stop the still window must contain such feeds (the stop clip plays on at the standstill), else it is vacuous
            if (kind == 0)
                Check(tag + "on: no stop exclusion while a non-loop non-idle clip has more than 12 frames left (tail gate)",
                      on.gateFeeds > 0 && on.gateViolations == 0,
                      "gate feeds " + std::to_string(on.gateFeeds) + " excluded " + std::to_string(on.gateViolations));
            const bool crouchStart = on.startClip.find("Crouch") != std::string::npos;
            Check(tag + "on: the start from the idle picks a start clip of the stance within the off latency",
                  on.startLatency >= 0.0f && off.startLatency >= 0.0f && on.startLatency <= off.startLatency + 1.0f / 60.0f + 1e-4f &&
                      crouchStart == (loco == 2),
                  "on " + on.startClip + " " + std::to_string(on.startLatency) + " s | off " + off.startClip + " " +
                      std::to_string(off.startLatency) + " s");
        }
        negDbs += negHere ? 1 : 0;
    }
    Check("idle hold: off (= batch 1) shows the stop-tail standstill on every database (negative control)",
          negDbs == int(paths.size()) && negDbs > 0, std::to_string(negDbs));
    {
        // setter / status: off = no idleHold fields (batch 1b status text), on = fields
        SetIdleHold(false);
        const bool h0 = IdleHold();
        const std::string s0 = Status();
        SetIdleHold(true);
        const std::string s1 = Status();
        Check("idle hold: native toggle, status fields only while on",
              !h0 && IdleHold() && s0.find(" idleHold=") == std::string::npos && s1.find(" idleHold=1") != std::string::npos);
    }
    g_fp.toe[0] = g_fp.toe[1] = -1;
    SetIdleHold(g_replantMain != 0);
    SetFootReplant(g_replantMain);
    SetFeetDiag(g_replantMain != 0);
    SetCrouchIdleExclude(g_replantMain != 0);
    ConfigureMode(2);
}

// ================================================================ batch 2 knee fix (2026-10-13): UTPPSetMMKneeFix
// Report: the right knee "breaks" on starts. Offline analysis (2026-10-13): the StanceGuard knee swivel (forward pole, no
// stance gate) pushed the knee inward in crouch, its 80 ms filter chasing a target that jumps with the raw swing; standing turn starts
// broke in the foot-lock two-bone IK (pole = animated knee projected onto the new hip-target line, ill-conditioned for an almost
// straight leg with a lateral lock offset). Per feed (real driver, shipped switches, v6): both knees of the published snapshot and
// of the matcher output (raw = animation + inertialization) by FK. Break frame (60 Hz, first 1.0 s after each start press, weight
// >= 0.5): published |swivel change| > 8 deg/frame (thigh-shin flex > 20 deg on both frames, raw and published) OR |lateral knee
// offset change| > 2.5 cm/frame OR bend-plane deviation from raw > 20 deg. Added break = published break on a frame where the raw
// knee does not break (the raw animation's own fast swings stay in both modes). Fix on (mask 3) vs off (mask 0) from the same start.
// Batch 5: "on" = the shipped mask 7 (bit 4 planted swivel hold); a second suite on utpp_base compares mask 7 with 3.
// Diagnostics: UTPP_KNEE_TRACE=1 (one line per break frame), UTPP_KNEE_ATTR=nofl|nosg (foot lock / stance guard off),
// UTPP_KNEE_ONMASK=<0..7> (the v6 "on" mask), UTPP_KNEE_DB=<path> (the bit 4 suite database), UTPP_KNEE_CFG=1 (driver status
// before each run).
namespace knee
{
using utpp::mm::BoneTransform;
using utpp::mm::math::Quat;
using utpp::mm::math::Vec3;
constexpr float kR2D = 57.2957795f;
constexpr float kSwBreak = 8.0f, kLatBreak = 2.5f, kHingeBreak = 20.0f, kFlexBent = 20.0f;

struct G
{
    Vec3 p;
    Quat q;
};
struct Leg
{
    float flex = 0.0f, sw = 0.0f, lat = 0.0f; // deg, deg (knee offset about the hip-ankle axis from pelvis forward), cm (outward)
    float reach = 0.0f;                       // |hip - ankle| / (thigh + shin) (batch 5 diagnostics)
    bool bent = false;                        // bend plane defined (flex > 10 deg)
    Vec3 nl{};                                // bend-plane normal in the thigh frame
};
float Len(Vec3 v) { return std::sqrt(utpp::mm::math::Dot(v, v)); }
Vec3 Nrm(Vec3 v)
{
    const float l = Len(v);
    return l > 1e-9f ? v * (1.0f / l) : Vec3{};
}
float WrapD(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

struct Probe
{
    int32_t chain[2][3] = {{-1, -1, -1}, {-1, -1, -1}}; // UpLeg, Leg, Foot per side (0 left, 1 right)
    std::vector<int16_t> parents;
    std::vector<std::string> stepNames;
    std::vector<int> stepFrames;
    std::string scen;   // batch 5 diagnostics: scenario name / mask of the run (UTPP_KNEE_TRACE=1 lines)
    int32_t mask = 0;
    bool trace = false;
    int fi = 0;
    uint64_t lastSeq = 0;
    bool havePrev = false;
    Leg prevPub[2], prevRaw[2];
    std::vector<float> raw;
    std::vector<G> m;
    // results (window frames)
    int brk[2] = {0, 0}, added[2] = {0, 0}, rawBrk[2] = {0, 0}, hinge[2] = {0, 0}, frames = 0;
    float maxSw[2] = {0.0f, 0.0f}, maxLat[2] = {0.0f, 0.0f}, maxSwRaw[2] = {0.0f, 0.0f}, maxLatRaw[2] = {0.0f, 0.0f};
    void Clear()
    {
        fi = 0;
        lastSeq = 0;
        havePrev = false;
        for (int s = 0; s < 2; ++s)
        {
            brk[s] = added[s] = rawBrk[s] = hinge[s] = 0;
            maxSw[s] = maxLat[s] = maxSwRaw[s] = maxLatRaw[s] = 0.0f;
        }
        frames = 0;
    }
    void FK(const float* aPose, uint32_t aBones)
    {
        m.assign(aBones, G{});
        for (uint32_t b = 0; b < aBones; ++b)
        {
            const float* l = &aPose[size_t(b) * 7];
            const Vec3 lp{l[0], l[1], l[2]};
            const Quat lq{l[3], l[4], l[5], l[6]};
            const int p = b < parents.size() ? parents[b] : -1;
            if (p < 0 || uint32_t(p) >= b)
                m[b] = G{lp, lq};
            else
                m[b] = G{m[size_t(p)].p + utpp::mm::math::QuatRotate(m[size_t(p)].q, lp), utpp::mm::math::QuatMul(m[size_t(p)].q, lq)};
        }
    }
    Leg Measure(int s) const
    {
        using namespace utpp::mm::math;
        Leg r;
        const Vec3 a = m[size_t(chain[s][0])].p, b = m[size_t(chain[s][1])].p, k = m[size_t(chain[s][2])].p;
        const Vec3 thigh = b - a, shin = k - b;
        const float lt = Len(thigh), ls = Len(shin);
        r.flex = std::acos(std::clamp(Dot(Nrm(thigh), Nrm(shin)), -1.0f, 1.0f)) * kR2D;
        r.reach = lt + ls > 1e-6f ? Len(k - a) / (lt + ls) : 0.0f;
        const Vec3 axis = Nrm(k - a);
        const Vec3 off = thigh - axis * Dot(thigh, axis);
        const float sgn = s == 1 ? 1.0f : -1.0f;
        const Vec3 rr = m[size_t(chain[1][0])].p - m[size_t(chain[0][0])].p;
        const Vec3 fwdP = Nrm(Cross(Vec3{0.0f, 0.0f, 1.0f}, rr));
        const Vec3 F = Nrm(fwdP - axis * Dot(fwdP, axis));
        const Vec3 L = Cross(F, axis * -1.0f);
        const Vec3 offn = Nrm(off);
        r.sw = std::atan2(sgn * Dot(offn, L), Dot(offn, F)) * kR2D;
        Vec3 latP = rr;
        latP.z = 0.0f;
        r.lat = sgn * Dot(off, Nrm(latP)) * 100.0f;
        const Vec3 n = Cross(thigh, shin);
        const float nl = Len(n);
        if (nl > std::sin(10.0f / kR2D) * lt * ls)
        {
            r.bent = true;
            r.nl = QuatRotate(QuatInv(m[size_t(chain[s][0])].q), n * (1.0f / nl));
        }
        return r;
    }
};
Probe g_kp;

void FeedProbe(const FeedInput& aIn, const std::string& aStatus)
{
    using namespace utpp::mm::integ;
    C1bProbe(aIn, aStatus); // planted / locked toe speed, lock corrections (g_fp)
    Probe& k = g_kp;
    const int fi = k.fi++;
    int step = -1, inStep = 0;
    for (int acc = 0, i = 0; i < int(k.stepFrames.size()); ++i)
    {
        if (fi < acc + k.stepFrames[size_t(i)])
        {
            step = i;
            inStep = fi - acc;
            break;
        }
        acc += k.stepFrames[size_t(i)];
    }
    static PoseSnapshot snap{};
    if (!utpp::g_mmPose.Read(snap) || !MatcherPoseForTest(k.raw) || k.chain[1][2] < 0 || snap.numBones * 7 > k.raw.size() ||
        uint32_t(k.chain[1][2]) >= snap.numBones)
    {
        k.havePrev = false;
        return;
    }
    const bool ran = snap.sequence != k.lastSeq;
    k.lastSeq = snap.sequence;
    Leg pub[2], raw[2];
    k.FK(snap.pose, snap.numBones);
    for (int s = 0; s < 2; ++s)
        pub[s] = k.Measure(s);
    k.FK(k.raw.data(), snap.numBones);
    for (int s = 0; s < 2; ++s)
        raw[s] = k.Measure(s);
    const bool window = step >= 0 && k.stepNames[size_t(step)].rfind("start", 0) == 0 && inStep <= 60;
    if (window && k.havePrev && ran && Field(aStatus, "weight") >= 0.5f)
    {
        ++k.frames;
        for (int s = 0; s < 2; ++s)
        {
            const bool bent = pub[s].flex > kFlexBent && raw[s].flex > kFlexBent && k.prevPub[s].flex > kFlexBent &&
                              k.prevRaw[s].flex > kFlexBent;
            const float dSw = bent ? std::fabs(WrapD(pub[s].sw - k.prevPub[s].sw)) : 0.0f;
            const float dSwRaw = bent ? std::fabs(WrapD(raw[s].sw - k.prevRaw[s].sw)) : 0.0f;
            const float dLat = std::fabs(pub[s].lat - k.prevPub[s].lat), dLatRaw = std::fabs(raw[s].lat - k.prevRaw[s].lat);
            float h = 0.0f;
            if (pub[s].bent && raw[s].bent)
                h = std::acos(std::clamp(utpp::mm::math::Dot(pub[s].nl, raw[s].nl), -1.0f, 1.0f)) * kR2D;
            const bool rawBreak = dSwRaw > kSwBreak || dLatRaw > kLatBreak;
            const bool pubBreak = dSw > kSwBreak || dLat > kLatBreak || h > kHingeBreak;
            k.brk[s] += pubBreak ? 1 : 0;
            k.added[s] += pubBreak && !rawBreak ? 1 : 0;
            k.rawBrk[s] += rawBreak ? 1 : 0;
            k.hinge[s] += h > kHingeBreak ? 1 : 0;
            k.maxSw[s] = std::max(k.maxSw[s], dSw);
            k.maxLat[s] = std::max(k.maxLat[s], dLat);
            k.maxSwRaw[s] = std::max(k.maxSwRaw[s], dSwRaw);
            k.maxLatRaw[s] = std::max(k.maxLatRaw[s], dLatRaw);
            // batch 5 diagnostics (UTPP_KNEE_TRACE=1): one line per published break frame - clip, lock state, reach, StanceGuard
            if (k.trace && pubBreak)
            {
                const char sd = s == 0 ? 'L' : 'R';
                const std::string st = s == 0 ? "L" : "R";
                const size_t fp = aStatus.find(" " + st + "=");
                const char ph = fp == std::string::npos ? '?' : aStatus[fp + 3];
                std::printf("[KTRACE] %s m%d f%d %c %s dSw %.1f raw %.1f dLat %.1f raw %.1f h %.1f flex %.0f/%.0f reach %.3f/%.3f "
                            "clip %s@%d ph %c off %.1f corr %.1f w %.2f c %.2f swivel %.1f push %.1f\n",
                            k.scen.c_str(), k.mask, fi, sd, rawBreak ? "rawB" : "ADD", double(dSw), double(dSwRaw), double(dLat),
                            double(dLatRaw), double(h), double(pub[s].flex), double(raw[s].flex), double(pub[s].reach),
                            double(raw[s].reach), ClipOf(aStatus).c_str(), int(Field(aStatus, "clipFrame")), ph,
                            double(Field(aStatus, ("off" + st).c_str())), double(Field(aStatus, ("corr" + st).c_str())),
                            double(Field(aStatus, ("w" + st).c_str())), double(Field(aStatus, ("c" + st).c_str())),
                            double(Field(aStatus, ("swivel" + st).c_str())), double(Field(aStatus, ("stancePush" + st).c_str())));
            }
        }
    }
    for (int s = 0; s < 2; ++s)
    {
        k.prevPub[s] = pub[s];
        k.prevRaw[s] = raw[s];
    }
    k.havePrev = true;
}

struct Run
{
    int brk[2] = {0, 0}, added[2] = {0, 0}, rawBrk[2] = {0, 0}, hinge[2] = {0, 0}, frames = 0;
    float maxSw[2] = {0, 0}, maxLat[2] = {0, 0}, maxSwRaw[2] = {0, 0}, maxLatRaw[2] = {0, 0};
    double plantedMax = 0, lockedMax = 0, corrMax = 0;
    std::vector<double> planted;
    int Brk() const { return brk[0] + brk[1]; }
    int Added() const { return added[0] + added[1]; }
    void Add(const Run& o)
    {
        for (int s = 0; s < 2; ++s)
        {
            brk[s] += o.brk[s];
            added[s] += o.added[s];
            rawBrk[s] += o.rawBrk[s];
            hinge[s] += o.hinge[s];
            maxSw[s] = std::max(maxSw[s], o.maxSw[s]);
            maxLat[s] = std::max(maxLat[s], o.maxLat[s]);
            maxSwRaw[s] = std::max(maxSwRaw[s], o.maxSwRaw[s]);
            maxLatRaw[s] = std::max(maxLatRaw[s], o.maxLatRaw[s]);
        }
        frames += o.frames;
        plantedMax = std::max(plantedMax, o.plantedMax);
        lockedMax = std::max(lockedMax, o.lockedMax);
        corrMax = std::max(corrMax, o.corrMax);
        planted.insert(planted.end(), o.planted.begin(), o.planted.end());
    }
    std::string Line() const
    {
        char b[400];
        std::snprintf(b, sizeof b,
                      "breaks %d (L %d R %d), added %d (L %d R %d), raw %d, hinge %d, frames %d | max dSw L %.1f R %.1f deg/f (raw %.1f %.1f), "
                      "max dLat L %.1f R %.1f cm/f (raw %.1f %.1f) | planted p95 %.1f max %.1f cm/s, locked max %.2f m/s, corr max %.1f cm",
                      Brk(), brk[0], brk[1], Added(), added[0], added[1], rawBrk[0] + rawBrk[1], hinge[0] + hinge[1], frames,
                      double(maxSw[0]), double(maxSw[1]), double(maxSwRaw[0]), double(maxSwRaw[1]), double(maxLat[0]),
                      double(maxLat[1]), double(maxLatRaw[0]), double(maxLatRaw[1]), Pct(planted, 0.95) * 100.0, plantedMax * 100.0,
                      lockedMax, corrMax);
        return b;
    }
};

Run RunOne(const C1bSetup& aSu, const std::vector<Step>& aSteps, float aBodyYaw, int32_t aMask)
{
    Run r;
    if (!LoadDatabase(aSu.path, aSu.name))
        return r;
    OnArmed(aSu.rig, *aSu.hashes, *aSu.parents);
    SetEnabled(true);
    ConfigureShippedC1a();
    SetCrouchIdleExclude(true);
    SetFeetDiag(true);
    SetFootReplant(2);
    SetIdleHold(true);
    SetKneeFix(aMask);
    // batch 5 attribution diagnostics (UTPP_KNEE_ATTR, default none): nofl = foot lock off, nosg = stance guard off
    {
        char ea[8] = {};
        if (GetEnvironmentVariableA("UTPP_KNEE_ATTR", ea, sizeof ea) > 0)
        {
            if (std::string(ea) == "nofl")
                SetFootLock(false);
            if (std::string(ea) == "nosg")
                (void)SetStanceGuard(false, 3.0f);
        }
        char et[4] = {};
        g_kp.trace = GetEnvironmentVariableA("UTPP_KNEE_TRACE", et, sizeof et) > 0 && et[0] == '1';
        g_kp.mask = aMask;
        // diagnostics: UTPP_KNEE_CFG=1 prints the driver status (switches) before each run
        char ec[4] = {};
        if (GetEnvironmentVariableA("UTPP_KNEE_CFG", ec, sizeof ec) > 0 && ec[0] == '1')
            std::printf("[KCFG] %s m%d %s\n", g_kp.scen.c_str(), aMask, Status().c_str());
    }
    (void)RunScenario(aSteps, aBodyYaw, true); // warm-up (driver state after the load)
    g_fp.Clear();
    g_kp.Clear();
    g_kp.stepNames.clear();
    g_kp.stepFrames.clear();
    for (const Step& s : aSteps)
    {
        g_kp.stepNames.push_back(s.name);
        g_kp.stepFrames.push_back(int(s.seconds * 60.0f + 0.5f));
    }
    g_feedProbe = &FeedProbe;
    (void)RunScenario(aSteps, aBodyYaw, true);
    g_feedProbe = nullptr;
    for (int s = 0; s < 2; ++s)
    {
        r.brk[s] = g_kp.brk[s];
        r.added[s] = g_kp.added[s];
        r.rawBrk[s] = g_kp.rawBrk[s];
        r.hinge[s] = g_kp.hinge[s];
        r.maxSw[s] = g_kp.maxSw[s];
        r.maxLat[s] = g_kp.maxLat[s];
        r.maxSwRaw[s] = g_kp.maxSwRaw[s];
        r.maxLatRaw[s] = g_kp.maxLatRaw[s];
    }
    r.frames = g_kp.frames;
    r.plantedMax = g_fp.plantedMax;
    r.lockedMax = g_fp.lockedMax;
    r.corrMax = g_fp.corrMax;
    r.planted = g_fp.planted;
    return r;
}
} // namespace knee

void SectionKnee(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionKnee");
    (void)argc;
    using S = Step;
    std::string v6 = argv[1], v7c;
    {
        const size_t sl = v6.find_last_of("\\/");
        const std::string dir = sl == std::string::npos ? std::string() : v6.substr(0, sl + 1);
        v6 = dir + "utpp_base.utppmm";
        // batch 5: the installed motion-only database next to argv[1]; UTPP_KNEE_DB=<path> tests another one (e.g. an installed copy)
        v7c = dir + "utpp_base.utppmm";
        char ed[1024] = {};
        const DWORD n = GetEnvironmentVariableA("UTPP_KNEE_DB", ed, sizeof ed);
        if (n > 0 && n < sizeof ed)
            v7c = ed;
    }
    utpp::mm::Database tdb;
    std::string err;
    const bool dbOk = tdb.LoadFromFile(v6.c_str(), kExpectedRigHash, err);
    // ctest shards (--units=knee.<cell>): the section is a grid of 3 sub-suites (v6 mask on vs off, v7c mask 7 vs 3, v6 mask 7 vs 3) x
    // 2 groups of scenario families; a cell runs every scenario of its families for its sub-suite, so each family aggregate that a
    // Check reads is complete inside one cell. Cells: v6a v6b v7a v7b v6ma v6mb; "knee" = all of them in the original order.
    // Group a: crouch start, stand restart, crouch / stand steady gait (26 scenarios); group b: stand start, stand turn start,
    // crouch restart (24). Checks that exist once (database reads, native range) print in the first / last cell only; a failure
    // prints in every cell that hits it.
    const auto groupOf = [](const std::string& aFam) {
        return (aFam == "crouch start" || aFam == "stand restart" || aFam == "crouch steady gait" || aFam == "stand steady gait") ? 'a' : 'b';
    };
    const auto cellOn = [&](const char* aPhase, const std::string& aFam) {
        return SubUnitOn("knee", (std::string(aPhase) + groupOf(aFam)).c_str());
    };
    if (!dbOk || SubUnitOn("knee", "v6a"))
        Check("knee fix: database utpp_base read", dbOk, err);
    if (!dbOk)
        return;
    {
        static const char* kNames[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"}, {"RightUpLeg", "RightLeg", "RightFoot"}};
        knee::g_kp.parents.assign(tdb.NumBones(), -1);
        g_fp.toe[0] = g_fp.toe[1] = -1;
        g_fp.parents.assign(tdb.NumBones(), -1);
        for (uint32_t b = 0; b < tdb.NumBones(); ++b)
        {
            knee::g_kp.parents[b] = int16_t(tdb.BoneParent(b));
            g_fp.parents[b] = int16_t(tdb.BoneParent(b));
            for (int s = 0; s < 2; ++s)
                for (int j = 0; j < 3; ++j)
                    if (tdb.BoneName(b) == kNames[s][j])
                        knee::g_kp.chain[s][j] = int32_t(b);
            if (tdb.BoneName(b) == "LeftToeBase")
                g_fp.toe[0] = int32_t(b);
            if (tdb.BoneName(b) == "RightToeBase")
                g_fp.toe[1] = int32_t(b);
        }
    }
    C1bSetup su;
    su.path = std::wstring(v6.begin(), v6.end());
    su.name = "base_only";
    su.hashes = &aHashes;
    su.parents = &aParents;
    su.rig = aRig;
    // scenarios (as the 2026-10-13 analysis): start / stop from an idle in 8 directions relative to the body (stand and crouch, full
    // stick and walk stick), and a second start right after a stop (crouch stop 0.3 / 0.6 s, stand 0.4 s) in 4 directions
    struct Scen
    {
        std::string fam, name;
        std::vector<Step> steps;
    };
    std::vector<Scen> scens;
    const float rel[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, -135.0f, -90.0f, -45.0f};
    struct Fam
    {
        const char* tag;
        int loco;
        float stick;
    };
    const Fam fams[4] = {{"s_run", 1, 1.0f}, {"s_walk", 1, 0.7f}, {"c_run", 2, 1.0f}, {"c_walk", 2, 0.7f}};
    for (const Fam& f : fams)
        for (int d = 0; d < 8; ++d)
        {
            Scen sc;
            const bool turn = std::fabs(rel[d]) >= 90.0f;
            sc.fam = f.loco == 2 ? "crouch start" : (turn ? "stand turn start" : "stand start (0 / +-45)");
            sc.name = std::string(f.tag) + "_d" + std::to_string(int(rel[d]));
            sc.steps = {S{.name = "idle", .seconds = 2.0f, .stick = 0.0f, .loco = f.loco},
                        S{.name = "start", .seconds = 1.5f, .stick = f.stick, .angleDeg = rel[d], .loco = f.loco},
                        S{.name = "stop", .seconds = 1.5f, .stick = 0.0f, .loco = f.loco}};
            scens.push_back(sc);
        }
    const float rel2[4] = {0.0f, 90.0f, 180.0f, -90.0f};
    for (int loco = 1; loco <= 2; ++loco)
        for (float stopT : (loco == 2 ? std::vector<float>{0.3f, 0.6f} : std::vector<float>{0.4f}))
            for (int d = 0; d < 4; ++d)
            {
                Scen sc;
                char nm[64];
                std::snprintf(nm, sizeof nm, "%s_restart%.0fms_d%d", loco == 2 ? "c" : "s", double(stopT) * 1000.0, int(rel2[d]));
                sc.fam = loco == 2 ? "crouch restart" : "stand restart";
                sc.name = nm;
                sc.steps = {S{.name = "idle", .seconds = 2.0f, .stick = 0.0f, .loco = loco},
                            S{.name = "start", .seconds = 1.5f, .stick = 1.0f, .loco = loco},
                            S{.name = "stop", .seconds = stopT, .stick = 0.0f, .loco = loco},
                            S{.name = "start2", .seconds = 1.5f, .stick = 1.0f, .angleDeg = rel2[d], .loco = loco},
                            S{.name = "stop2", .seconds = 1.5f, .stick = 0.0f, .loco = loco}};
                scens.push_back(sc);
            }
    // steady gait (review: bit 1 also mutes the crouch swivel in the loops, not only on starts): 1.5 s into a straight run / walk,
    // the next 1.0 s is measured (step "start_steady": the probe window is the first 60 frames of a step named start*)
    for (int loco = 1; loco <= 2; ++loco)
        for (float stick : {1.0f, 0.7f})
            for (float ang : (loco == 2 ? std::vector<float>{0.0f, 90.0f} : std::vector<float>{0.0f}))
            {
                Scen sc;
                char nm[64];
                std::snprintf(nm, sizeof nm, "%s_%s_steady_d%d", loco == 2 ? "c" : "s", stick > 0.9f ? "run" : "walk", int(ang));
                sc.fam = loco == 2 ? "crouch steady gait" : "stand steady gait";
                sc.name = nm;
                sc.steps = {S{.name = "idle", .seconds = 2.0f, .stick = 0.0f, .loco = loco},
                            S{.name = "go", .seconds = 1.5f, .stick = stick, .angleDeg = ang, .loco = loco},
                            S{.name = "start_steady", .seconds = 1.2f, .stick = stick, .loco = loco},
                            S{.name = "stop", .seconds = 1.0f, .stick = 0.0f, .loco = loco}};
                scens.push_back(sc);
            }
    std::map<std::string, knee::Run> famOn, famOff;
    std::vector<std::string> famOrder; // families in order of first appearance in scens (all of them, whatever cell runs)
    for (const Scen& sc : scens)
        if (std::find(famOrder.begin(), famOrder.end(), sc.fam) == famOrder.end())
            famOrder.push_back(sc.fam);
    int32_t onMask = 7; // batch 5: the shipped default (bits 1, 2 and 4)
    {
        char em[4] = {}; // diagnostics: UTPP_KNEE_ONMASK=<0..7> replaces the "on" mask
        if (GetEnvironmentVariableA("UTPP_KNEE_ONMASK", em, sizeof em) > 0)
            onMask = std::atoi(em);
    }
    for (const Scen& sc : scens)
    {
        if (!cellOn("v6", sc.fam))
            continue;
        knee::g_kp.scen = sc.name;
        const knee::Run on = knee::RunOne(su, sc.steps, -82.0f, onMask);
        const knee::Run off = knee::RunOne(su, sc.steps, -82.0f, 0);
        std::printf("[INFO] knee fix %s on : %s\n", sc.name.c_str(), on.Line().c_str());
        std::printf("[INFO] knee fix %s off: %s\n", sc.name.c_str(), off.Line().c_str());
        famOn[sc.fam].Add(on);
        famOff[sc.fam].Add(off);
    }
    for (const std::string& f : famOrder)
    {
        if (!cellOn("v6", f))
            continue;
        const knee::Run& on = famOn[f];
        const knee::Run& off = famOff[f];
        std::printf("[INFO] knee fix family %s: on  %s\n", f.c_str(), on.Line().c_str());
        std::printf("[INFO] knee fix family %s: off %s\n", f.c_str(), off.Line().c_str());
        const std::string tag = "knee fix [" + f + "] ";
        const std::string det = "on: " + on.Line() + " | off: " + off.Line();
        // break frames the leg post-processing adds to the raw animation: at least halved in the crouch and standing turn families
        // (the families of the report), not more elsewhere. All published break frames: not more than off in the target
        // families; elsewhere at most 1 more, and only on frames where the raw knee itself breaks (published breaks that are not
        // added <= raw breaks: the published knee follows the raw swing a frame earlier / later - full suite, stand start 0 / +-45:
        // 17 vs 16 with 16 raw breaks and the same 1 added)
        const bool target = f == "crouch start" || f == "crouch restart" || f == "stand turn start";
        const int rawAll = on.rawBrk[0] + on.rawBrk[1];
        Check(tag + (target ? "added knee break frames (published break, raw knee steady) at least halved vs fix off, total breaks not more"
                            : "added knee break frames not more than fix off, total breaks at most 1 more (raw-following only)"),
              on.frames > 0 && on.frames == off.frames &&
                  (target ? 2 * on.Added() <= off.Added() && off.Added() >= 4 && on.Brk() <= off.Brk()
                          : on.Added() <= off.Added() && on.Brk() <= off.Brk() + 1 && on.Brk() - on.Added() <= rawAll),
              det);
        // foot locks not worse (5 % / 0.5 cm tolerance for float noise; a toe speed below 0.1 m/s is no jump)
        const auto notWorse = [](double a, double a0) { return a <= a0 * 1.05 + 0.005 || a < 0.1; };
        Check(tag + "foot locks not worse than fix off (planted / locked toe speed, max lock correction)",
              notWorse(on.plantedMax, off.plantedMax) && notWorse(on.lockedMax, off.lockedMax) && on.corrMax <= off.corrMax + 0.5,
              det);
    }
    // batch 5 knee fix bit 4 (StanceGuard swivel slowed while the foot is planted) on the installed motion-only database: the standing
    // turn starts (Reface / Start clips turning 90-180 deg) broke the knee on 25 frames (15 added) with mask 3 - all from the stance
    // guard knee swivel (stance guard off: 0 added; foot lock off: unchanged). Mask 7 vs 3 from the same start: added break frames
    // at least halved and fewer breaks in the standing turn starts; elsewhere at most 1 more added and 2 more breaks (standing
    // restarts 11 -> 12); foot locks not worse (the swivel keeps the ankle: identical lock numbers expected).
    if (SubUnitOn("knee", "v7a") || SubUnitOn("knee", "v7b"))
    {
        utpp::mm::Database cdb;
        std::string cerr;
        const bool cOk = cdb.LoadFromFile(v7c.c_str(), kExpectedRigHash, cerr);
        bool sameRig = cOk && cdb.NumBones() == tdb.NumBones();
        for (uint32_t b = 0; sameRig && b < cdb.NumBones(); ++b)
            sameRig = cdb.BoneName(b) == tdb.BoneName(b) && cdb.BoneParent(b) == tdb.BoneParent(b);
        if (!sameRig || SubUnitOn("knee", "v7a"))
            Check("knee fix bit 4: database " + v7c + " read, same bones as v6 (probe indices)", sameRig, cerr);
        if (sameRig)
        {
            C1bSetup sc7 = su;
            sc7.path = std::wstring(v7c.begin(), v7c.end());
            sc7.name = "base_only";
            std::map<std::string, knee::Run> f7, f3;
            for (const Scen& sc : scens)
            {
                if (!cellOn("v7", sc.fam))
                    continue;
                knee::g_kp.scen = "v7c " + sc.name;
                const knee::Run r7 = knee::RunOne(sc7, sc.steps, -82.0f, 7);
                const knee::Run r3 = knee::RunOne(sc7, sc.steps, -82.0f, 3);
                std::printf("[INFO] knee fix v7c %s m7: %s\n", sc.name.c_str(), r7.Line().c_str());
                std::printf("[INFO] knee fix v7c %s m3: %s\n", sc.name.c_str(), r3.Line().c_str());
                f7[sc.fam].Add(r7);
                f3[sc.fam].Add(r3);
            }
            for (const std::string& f : famOrder)
            {
                if (!cellOn("v7", f))
                    continue;
                const knee::Run& a = f7[f];
                const knee::Run& b = f3[f];
                std::printf("[INFO] knee fix v7c family %s: m7 %s\n", f.c_str(), a.Line().c_str());
                std::printf("[INFO] knee fix v7c family %s: m3 %s\n", f.c_str(), b.Line().c_str());
                const std::string tag = "knee fix bit 4 v7c [" + f + "] ";
                const std::string det = "m7: " + a.Line() + " | m3: " + b.Line();
                const bool target = f == "stand turn start";
                // batch 5 repair: the numbers depend on the driver state the section starts from (stance guard swivel, body yaw,
                // offsets survive LoadDatabase / OnArmed; the warm-up run does not wash the chain out): UTPP_LOCO_ONLYKNEE=1 (cold
                // start) gave m7 11 / added 1 (R 0) vs m3 25 / 15 (R 10), the full suite m7 22 / added 6 (R 6) vs m3 31 / 14 (R 12).
                // Target family: also fewer added break frames on the right knee (the knee of the report).
                Check(tag + (target ? "added knee break frames at least halved vs mask 3 (right knee fewer), fewer break frames"
                                    : "added knee break frames at most 1 more than mask 3, break frames at most 2 more"),
                      a.frames > 0 && a.frames == b.frames &&
                          (target ? 2 * a.Added() <= b.Added() && b.Added() >= 4 && a.Brk() < b.Brk() && a.added[1] < b.added[1]
                                  : a.Added() <= b.Added() + 1 && a.Brk() <= b.Brk() + 2),
                      det);
                const auto notWorse = [](double x, double x0) { return x <= x0 * 1.05 + 0.005 || x < 0.1; };
                Check(tag + "foot locks not worse than mask 3 (planted / locked toe speed, max lock correction)",
                      notWorse(a.plantedMax, b.plantedMax) && notWorse(a.lockedMax, b.lockedMax) && a.corrMax <= b.corrMax + 0.5, det);
            }
        }
    }
    // batch 5 repair: bit 4 on v6 (mask 7 vs 3; the v6 suite above compares 7 with 0). The accepted v6 cost of the 0.3 gain is
    // small (cold start: standing restarts 5 -> 7, turn starts 10 -> 11); a full hold (gain 0) doubled the restarts (5 -> 11).
    // Bound: per family at most 3 more break frames and 3 more added than mask 3 (mutation gain 0 fails it: cold start stand restart
    // 11 vs 5, full suite stand turn start 28 / 15 added vs 22 / 14). Runs after the v7c suite (the earlier runs keep their driver
    // state chain and numbers).
    if (onMask == 7)
    {
        std::map<std::string, knee::Run> g7, g3;
        for (const Scen& sc : scens)
        {
            if (!cellOn("v6m", sc.fam))
                continue;
            knee::g_kp.scen = "v6m " + sc.name;
            const knee::Run r7 = knee::RunOne(su, sc.steps, -82.0f, 7);
            const knee::Run r3 = knee::RunOne(su, sc.steps, -82.0f, 3);
            std::printf("[INFO] knee fix v6 %s m7: %s\n", sc.name.c_str(), r7.Line().c_str());
            std::printf("[INFO] knee fix v6 %s m3: %s\n", sc.name.c_str(), r3.Line().c_str());
            g7[sc.fam].Add(r7);
            g3[sc.fam].Add(r3);
        }
        for (const std::string& f : famOrder)
        {
            if (!cellOn("v6m", f))
                continue;
            const knee::Run& a = g7[f];
            const knee::Run& b = g3[f];
            std::printf("[INFO] knee fix v6 family %s: m7 %s\n", f.c_str(), a.Line().c_str());
            std::printf("[INFO] knee fix v6 family %s: m3 %s\n", f.c_str(), b.Line().c_str());
            Check("knee fix bit 4 v6 [" + f + "] mask 7 vs 3: at most 3 more break frames and 3 more added",
                  a.frames > 0 && a.frames == b.frames && a.Brk() <= b.Brk() + 3 && a.Added() <= b.Added() + 3,
                  "m7: " + a.Line() + " | m3: " + b.Line());
        }
    }
    if (SubUnitOn("knee", "v6mb")) // once per suite (the last cell)
    {
        // native: range 0..7 (batch 5), default 7, status field only while not 0
        const int32_t m0 = KneeFixMask();
        const bool bad = !SetKneeFix(-1) && !SetKneeFix(8) && KneeFixMask() == m0;
        SetKneeFix(0);
        const std::string s0 = Status();
        SetKneeFix(7);
        const std::string s7 = Status();
        Check("knee fix: native range 0..7 (invalid rejected, previous kept), status kneeFix= only while not 0",
              bad && KneeFixMask() == 7 && s0.find(" kneeFix=") == std::string::npos && s7.find(" kneeFix=7") != std::string::npos);
    }
    g_fp.toe[0] = g_fp.toe[1] = -1;
    SetKneeFix(g_replantMain != 0 ? 7 : 0);
    SetIdleHold(g_replantMain != 0);
    SetFootReplant(g_replantMain);
    SetFeetDiag(g_replantMain != 0);
    SetCrouchIdleExclude(g_replantMain != 0);
    ConfigureMode(2);
}

// ---- B2 strafe S1 (2026-10-13): aim strafe on the installed database (utpp_base, next to argv[1]) ----
// Aim game model: V aims (PSM UpperBody 6 + AimWalk, locoState 0; crouch: locoState 2 + UpperBody 6), upper-body mask on with
// states 7 (weapon in hand), hold pose on; V's yaw follows the camera through the aim align (starts beyond 12 deg, aligns down to
// 2 deg at <= 1080 deg/s); the capsule moves body-relative to the raw stick at 1.65 m/s (vanilla accel 24 / decel 20 m/s^2);
// Locomotion inactive (the reds 0.6.1 fallback target body + stick angle is fed, the driver must ignore it). Not aiming: the
// camera-relative stick, the body turned by UTPP's command (as RunScenario).
namespace strafet
{
struct AimStep
{
    std::string name;
    float seconds = 1.0f;
    float angle = 0.0f;     // stick direction relative to the body (aim) / camera (no aim), deg CCW from forward (90 = left)
    float stick = 1.0f;     // |stick|
    float camRate = 0.0f;   // deg/s mouse pan
    float camFlick = 0.0f;  // deg camera jump at the step start
    float angleRate = 0.0f; // deg/s stick sweep
    int loco = 1;           // 1 stand, 2 crouch, 3 sprint (no aim)
    bool aim = true;
    bool cruise = false;    // transitions/s and steady rate criteria
    float speed = 3.85f;    // m/s full-stick speed while not aiming
};
struct AimMetrics
{
    int moveFrames = 0, agree = 0, turnPicks = 0, strafeFrames = 0, picks = 0, cruiseTrans = 0, rateErrN = 0, rateOut = 0;
    float cruiseSec = 0.0f, corrMax = 0.0f, offSatMax = 0.0f, yawStepMax = 0.0f, rateMin = 9.0f, rateMax = 0.0f, rateErrMax = 0.0f;
    float offLatency = -1.0f, btLatency = -1.0f;
    bool onAtRelease = false; // strafe=1 on the frame before the aim release (set with offLatency)
    float steadyCorrMax = 0.0f; // foot-lock correction max over the last second of the cruise steps (steady strafe)
    int steadyN = 0, steadyAgree = 0;
    // B2 strafe S4 jog (only with g_jogF > 0): native AimJogTarget answers, capsule speed at the profile, run loops while cruising
    int jogQ = 0, jogOn = 0, jogWrongOn = 0, jogMissing = 0, steadyRunLoop = 0, steadyWalkLoop = 0, steadyLoop = 0, capN = 0, releaseJogOn = 0;
    int jogLit = 0; // jogOn frames compared against the literal design numbers (JogLiteral)
    float jogErrMax = 0.0f, capErrMax = 0.0f;
    std::string turnList;
    float Agree() const { return moveFrames > 0 ? float(agree) / float(moveFrames) : 1.0f; }
    void Add(const AimMetrics& o)
    {
        moveFrames += o.moveFrames;
        agree += o.agree;
        turnPicks += o.turnPicks;
        strafeFrames += o.strafeFrames;
        picks += o.picks;
        cruiseTrans += o.cruiseTrans;
        rateErrN += o.rateErrN;
        rateOut += o.rateOut;
        cruiseSec += o.cruiseSec;
        corrMax = std::max(corrMax, o.corrMax);
        offSatMax = std::max(offSatMax, o.offSatMax);
        yawStepMax = std::max(yawStepMax, o.yawStepMax);
        rateMin = std::min(rateMin, o.rateMin);
        rateMax = std::max(rateMax, o.rateMax);
        rateErrMax = std::max(rateErrMax, o.rateErrMax);
        steadyCorrMax = std::max(steadyCorrMax, o.steadyCorrMax);
        steadyN += o.steadyN;
        steadyAgree += o.steadyAgree;
        jogQ += o.jogQ;
        jogOn += o.jogOn;
        jogWrongOn += o.jogWrongOn;
        jogMissing += o.jogMissing;
        jogLit += o.jogLit;
        steadyRunLoop += o.steadyRunLoop;
        steadyWalkLoop += o.steadyWalkLoop;
        steadyLoop += o.steadyLoop;
        capN += o.capN;
        releaseJogOn += o.releaseJogOn;
        jogErrMax = std::max(jogErrMax, o.jogErrMax);
        capErrMax = std::max(capErrMax, o.capErrMax);
        if (turnList.size() < 300)
            turnList += o.turnList;
    }
    std::string Line() const
    {
        char b[520];
        std::snprintf(b, sizeof b,
                      "dir agree %.1f %% (%d/%d) turnPicks %d picks %d strafeFrames %d cruise trans/s %.2f footlock max %.1f cm offSat/step max "
                      "%.2f s rootYaw/step max %.1f deg rate %.2f..%.2f (out %d) steady rate err max %.1f %% (n %d) | steady cruise: footlock max %.1f cm, "
                      "agree %.1f %% (%d)",
                      100.0 * Agree(), agree, moveFrames, turnPicks, picks, strafeFrames, cruiseSec > 0 ? cruiseTrans / cruiseSec : 0.0f, corrMax,
                      offSatMax, yawStepMax, rateMin, rateMax, rateOut, 100.0 * rateErrMax, rateErrN, steadyCorrMax,
                      steadyN > 0 ? 100.0 * steadyAgree / steadyN : 100.0, steadyN);
        return b;
    }
    std::string JogLine() const
    {
        char b[512];
        std::snprintf(b, sizeof b,
                      "jog target: queries %d on %d (vs literal F x 1 / 0.70 / 0.60: %d, err max %.4f m/s, on while not expected %d, missing "
                      "while expected %d, on at the aim release %d) | reds-model capsule vs literal err max %.1f %% (n %d) | steady clips run "
                      "gait %d walk gait %d, facing-fixed loops %d (n %d)",
                      jogQ, jogOn, jogLit, jogErrMax, jogWrongOn, jogMissing, releaseJogOn, 100.0 * capErrMax, capN, steadyRunLoop,
                      steadyWalkLoop, steadyLoop, steadyN);
        return b;
    }
};
struct ClipInfo
{
    std::vector<float> cumYawDeg, ownSpeed;
};
ClipInfo g_ci;
bool g_dbgSteps = false; // UTPP_STRAFE_DBG=1: one line per step
utpp::mm::strafe::StrafeProfiles g_prof; // test-side strafe profiles (speed map S(theta)) from the driver's state tag masks
bool g_haveProf = false;
const utpp::mm::Database* g_db = nullptr;
// B2 strafe S4 jog: g_jogF > 0 = the reds UTPPUpdateAimSpeed model (MaxSpeed multiplier from AimJogTarget) drives the aim capsule
// and the run gait expectations; 0 = the S1 runs (AimJogTarget never queried)
float g_jogF = 0.0f;
// S4: expected jog target from the literal design numbers (F : side : back = 1 : 0.70 : 0.60 at 0 / 90 / 135-180 deg, forward diagonal
// 1), independent of strafe::JogProfile / JogTargetSpeed; |stick| <= 0.15 -> F. < 0 = stick angle not a multiple of 45 deg.
float JogLiteral(float aF, float aAngleDeg, float aStick)
{
    if (aStick <= 0.15f)
        return aF;
    const float k = std::fabs(WrapDeg(aAngleDeg)) / 45.0f;
    const int i = int(std::lround(k));
    if (std::fabs(k - float(i)) > 1e-3f || i < 0 || i > 4)
        return -1.0f;
    static constexpr float kLit[5] = {1.0f, 1.0f, 0.70f, 0.60f, 0.60f};
    return aF * kLit[i];
}
// B2 strafe S2: per-feed probe (status after the feed, the step, strafe=1 flag); nullptr = nothing (the S1 runs)
std::function<void(const std::string&, const AimStep&, bool)> g_chestProbe;
std::string StatusFrom(const char* aKey)
{
    const std::string st = Status();
    const size_t at = st.find(aKey);
    return at == std::string::npos ? std::string("(no ") + aKey + ")" : st.substr(at);
}

bool TurnPick(uint32_t aClip)
{
    using namespace utpp::mm;
    const std::string_view n = g_db->ClipName(aClip);
    const uint8_t base = steer::DriverClipClass(n, steer::ClassifyClip(n), true);
    if (aClip < g_ci.cumYawDeg.size() && g_ci.cumYawDeg[aClip] >= 10.0f)
        return true;
    if (strafe::IsStrafeTurnBaseClass(base))
        return true;
    if (g_jogF > 0.0f) // S4 jog: the run band is the strafe band (stand)
    {
        const int sc = steer::SpeedClassOfName(n);
        return base == steer::kClassPattern && sc != 0 && sc != 2;
    }
    return base == steer::kClassPattern && steer::SpeedClassOfName(n) >= 2; // off the walk band
}

// aAllAim: metrics on every aiming frame (negative control with strafe off) instead of the strafe=1 frames
AimMetrics RunAim(const std::vector<AimStep>& aSteps, float aDt, bool aAllAim = false)
{
    using namespace utpp::mm;
    AimMetrics total;
    SetEnabled(true); // fresh matcher state
    float cam = 0.0f, body = 0.0f, vx = 0.0f, vy = 0.0f, angle = 0.0f;
    bool aligning = false, prevAim = true, prevFlag = false;
    const float fps = g_db->Fps() > 0.0f ? g_db->Fps() : 30.0f;
    const uint32_t kGunAimWalk = ((0u + 1u) << 24) | ((6u + 1u) << 13) | 0x40u | ((2u + 1u) << 16);
    const uint32_t kCrouchAim = ((0u + 1u) << 24) | ((6u + 1u) << 13);
    int prevClip = -2;
    float sinceChange = 0.0f;
    float jogMul = 1.0f; // S4 reds model: MaxSpeed multiplier (1 = no modifier)
    for (const AimStep& st : aSteps)
    {
        AimMetrics m;
        cam += st.camFlick;
        angle = st.angle;
        const int frames = int(st.seconds / aDt + 0.5f);
        float yawSum = 0.0f, offSat = 0.0f;
        const bool release = prevAim && !st.aim;
        for (int i = 0; i < frames; ++i)
        {
            cam = WrapDeg(cam + st.camRate * aDt);
            angle += st.angleRate * aDt;
            const bool moving = st.stick > 0.15f;
            if (st.aim)
            {
                const float e = WrapDeg(cam - body);
                if (!aligning && std::fabs(e) > 12.0f)
                    aligning = true;
                if (aligning)
                {
                    body = WrapDeg(body + std::clamp(e, -1080.0f * aDt, 1080.0f * aDt));
                    if (std::fabs(WrapDeg(cam - body)) <= 2.0f)
                        aligning = false;
                }
            }
            else
            {
                aligning = false;
                float baseYaw = body;
                if (moving)
                    baseYaw = body + std::clamp(WrapDeg(cam + angle - body), -540.0f * aDt, 540.0f * aDt);
                body = WrapDeg(BodyYawCommand(body, baseYaw));
            }
            const float dirDeg = st.aim ? body + angle : cam + angle;
            const float want = (st.aim ? 1.65f * jogMul : st.speed) * std::fmin(st.stick, 1.0f);
            {
                const float dx = -std::sin(dirDeg * kDegT), dy = std::cos(dirDeg * kDegT);
                float along = vx * dx + vy * dy;
                float px = vx - along * dx, py = vy - along * dy;
                const float pl = std::sqrt(px * px + py * py);
                const float pk = pl > 1e-6f ? std::fmax(0.0f, pl - 20.0f * aDt) / pl : 0.0f;
                px *= pk;
                py *= pk;
                if (!moving)
                    along = along > 0.0f ? std::fmax(0.0f, along - 20.0f * aDt) : std::fmin(0.0f, along + 20.0f * aDt);
                else if (along < 0.0f || along > want)
                    along = along < 0.0f ? std::fmin(0.0f, along + 20.0f * aDt) : std::fmax(want, along - 20.0f * aDt);
                else
                    along = std::fmin(want, along + 24.0f * aDt);
                vx = px + along * dx;
                vy = py + along * dy;
            }
            FeedInput in;
            in.velWorld[0] = vx;
            in.velWorld[1] = vy;
            const float mag = std::fmin(st.stick, 1.0f);
            if (st.aim)
            {
                in.move[0] = -std::sin(angle * kDegT) * mag;
                in.move[1] = std::cos(angle * kDegT) * mag;
                in.locoState = st.loco == 2 ? 2 : 0;
                in.psmBlock = st.loco == 2 ? kCrouchAim : kGunAimWalk;
            }
            else
            {
                in.move[1] = mag;
                in.locoState = st.loco;
            }
            in.cameraYawDeg = cam;
            in.playerYawDeg = body;
            in.tppActive = true;
            in.dt = aDt;
            in.haveTarget = moving;
            in.targetYawDeg = WrapDeg(dirDeg);
            g_clock += aDt;
            Feed(in);
            const std::string status = Status();
            const bool sFlag = Field(status, "strafe") > 0.5f;
            const bool sOn = aAllAim ? st.aim : sFlag;
            if (g_jogF > 0.0f)
            {
                // reds UTPPUpdateAimSpeed: AimWalk + UpperBody Aim (stand aim feeds), native target, 0.02 hysteresis, kept on release
                const float t = AimJogTarget(in.move[0], in.move[1]);
                const bool aimWalk = st.aim && st.loco != 2;
                const float sState = Field(status, "strafeState");
                const bool expect = aimWalk && sState > 0.5f && sState < 1.5f;
                ++m.jogQ;
                if (t > 0.0f)
                {
                    ++m.jogOn;
                    const float lit = JogLiteral(g_jogF, angle, std::fmin(st.stick, 1.0f));
                    if (lit >= 0.0f)
                    {
                        ++m.jogLit;
                        m.jogErrMax = std::max(m.jogErrMax, std::fabs(t - lit));
                    }
                }
                if (t > 0.0f && !expect)
                    ++m.jogWrongOn;
                if (t <= 0.0f && expect)
                    ++m.jogMissing;
                if (release && i == 0 && t > 0.0f)
                    ++m.releaseJogOn;
                if (!aimWalk || t <= 0.0f)
                    jogMul = 1.0f;
                else if (moving || jogMul == 1.0f)
                {
                    const float w = std::clamp(t / 1.65f, 0.5f, 2.5f);
                    if (std::fabs(w - jogMul) > 0.02f)
                        jogMul = w;
                }
            }
            if (g_chestProbe)
                g_chestProbe(status, st, sFlag);
            if (sFlag)
                ++m.strafeFrames;
            if (release && i == 0)
                m.onAtRelease = prevFlag; // strafe=1 on the last aiming frame (the release latency is not vacuous)
            prevFlag = sFlag;
            const int clip = int(Field(status, "clip") + 0.5f);
            const int cf = int(Field(status, "clipFrame") + 0.5f);
            const float rate = Field(status, "rate");
            const float w = Field(status, "weight");
            const float cap = std::sqrt(vx * vx + vy * vy);
            if (release && m.offLatency < 0.0f && !sOn)
                m.offLatency = float(i) * aDt;
            if (release && m.btLatency < 0.0f && Field(status, "btFed") > 0.5f)
                m.btLatency = float(i + 1) * aDt;
            if (clip != prevClip && prevClip != -2 && clip >= 0)
            {
                if (st.cruise)
                    ++m.cruiseTrans;
                if (sOn)
                {
                    ++m.picks;
                    if (TurnPick(uint32_t(clip)))
                    {
                        ++m.turnPicks;
                        m.turnList += std::string(" ") + std::string(g_db->ClipName(uint32_t(clip)));
                    }
                }
            }
            sinceChange = clip != prevClip ? 0.0f : sinceChange + aDt;
            prevClip = clip;
            if (st.cruise)
                m.cruiseSec += aDt;
            if (!sOn || clip < 0 || uint32_t(clip) >= g_db->NumClips())
                continue;
            const ClipRecord& cr = g_db->Clips()[size_t(clip)];
            // played root yaw (matcher time = rate x real time)
            if (const RootDelta* rd = g_db->Root(cr.firstFrame + uint32_t(std::clamp(cf, 0, int(cr.numFrames) - 1))))
                yawSum += rd->dyaw / kDegT * rate * aDt * fps;
            m.yawStepMax = std::max(m.yawStepMax, std::fabs(yawSum));
            m.corrMax = std::max({m.corrMax, Field(status, "corrL"), Field(status, "corrR")});
            const bool steady = st.cruise && i >= frames - int(1.0f / aDt);
            if (steady)
                m.steadyCorrMax = std::max({m.steadyCorrMax, Field(status, "corrL"), Field(status, "corrR")});
            if (g_jogF > 0.0f && steady && st.aim && st.loco != 2 && moving)
            {
                // the test's copy of reds UTPPUpdateAimSpeed drives the capsule: this checks the multiplier clamp / hysteresis
                // model against the literal numbers, not a game capsule
                const float exp = JogLiteral(g_jogF, angle, 1.0f) * std::fmin(st.stick, 1.0f);
                if (exp > 0.0f)
                {
                    m.capErrMax = std::max(m.capErrMax, std::fabs(cap - exp) / exp);
                    ++m.capN;
                }
                // gait of the playing clip (v6: the motion start clips carry the loop cycle, so a straight cruise keeps playing the
                // start); facing-fixed loops counted separately
                const int sc = steer::SpeedClassOfName(g_db->ClipName(uint32_t(clip)));
                m.steadyRunLoop += sc == 2 ? 1 : 0;
                m.steadyWalkLoop += sc == 1 ? 1 : 0;
                m.steadyLoop += (size_t(clip) < g_prof.loopOwnSpeed.size() && g_prof.loopOwnSpeed[size_t(clip)] > 0.0f) ? 1 : 0;
            }
            const float ox = Field(status, "offX"), oy = Field(status, "offY");
            if (std::sqrt(ox * ox + oy * oy) >= 24.9f)
                offSat += aDt;
            m.offSatMax = std::max(m.offSatMax, offSat);
            if (w > 0.5f && moving)
            {
                m.rateMin = std::min(m.rateMin, rate);
                m.rateMax = std::max(m.rateMax, rate);
                if (rate < 0.6f - 1e-3f || rate > 1.4f + 1e-3f)
                    ++m.rateOut;
            }
            if (cap >= 0.5f && w > 0.5f && moving)
            {
                ++m.moveFrames;
                double sx = 0.0, sy = 0.0;
                for (int k = -1; k <= 1; ++k)
                {
                    const int f = std::clamp(cf + k, 0, int(cr.numFrames) - 1);
                    if (const RootDelta* r = g_db->Root(cr.firstFrame + uint32_t(f)))
                    {
                        sx += r->dx;
                        sy += r->dy;
                    }
                }
                const float rootSpeed = float(std::sqrt(sx * sx + sy * sy) / 3.0) * fps;
                const float b = body * kDegT;
                const float cy = vx * -std::sin(b) + vy * std::cos(b), cx = vx * std::cos(b) + vy * std::sin(b);
                const float thC = std::atan2(-cx, cy) / kDegT, thR = std::atan2(-float(sx), float(sy)) / kDegT;
                const bool ok = rootSpeed >= 0.3f && std::fabs(WrapDeg(thR - thC)) <= 30.0f;
                if (ok)
                    ++m.agree;
                if (steady)
                {
                    ++m.steadyN;
                    m.steadyAgree += ok ? 1 : 0;
                }
                // steady rate law (cruise, last second, >= 0.3 s after a clip change: the 0.1 s smoothing settled): a facing-fixed loop
                // plays at capsule / its own speed, any other clip at aim point / S(theta) (per-direction loop speed map)
                if (st.cruise && i >= frames - int(1.0f / aDt) && sinceChange >= 0.3f && st.aim && cap >= 1.0f && g_haveProf)
                {
                    const float own = size_t(clip) < g_prof.loopOwnSpeed.size() ? g_prof.loopOwnSpeed[size_t(clip)] : 0.0f;
                    // S4 jog (stand): run gait, game point F x JogProfile(theta)
                    const bool jogG = g_jogF > 0.0f && st.loco != 2;
                    const float sMap = g_prof.table.Speed(st.loco == 2 ? 1 : 0, jogG ? 1 : 0, WrapDeg(angle) * kDegT);
                    const float pt = jogG ? g_jogF * strafe::JogProfile(WrapDeg(angle) * kDegT) : 1.65f;
                    const float exp = std::clamp(own > 0.3f ? cap / own : (sMap > 0.05f ? pt / sMap : 1.0f), 0.6f, 1.4f);
                    m.rateErrMax = std::max(m.rateErrMax, std::fabs(rate - exp) / exp);
                    ++m.rateErrN;
                }
            }
        }
        prevAim = st.aim;
        if (g_dbgSteps)
        {
            const std::string stE = Status();
            const int ec = int(Field(stE, "clip") + 0.5f);
            std::printf("[DBG] strafe step %-16s agree %4d/%4d corr %.1f offSat %.2f rate %.2f..%.2f end %s f=%d rate=%.2f own=%.2f |v|=%.2f strafe=%d trans %d\n",
                        st.name.c_str(), m.agree, m.moveFrames, m.corrMax, m.offSatMax, m.rateMin, m.rateMax,
                        ec >= 0 && uint32_t(ec) < g_db->NumClips() ? std::string(g_db->ClipName(uint32_t(ec))).c_str() : "-",
                        int(Field(stE, "clipFrame")), Field(stE, "rate"), ec >= 0 && size_t(ec) < g_ci.ownSpeed.size() ? g_ci.ownSpeed[size_t(ec)] : 0.0f,
                        std::sqrt(vx * vx + vy * vy), int(Field(stE, "strafe") + 0.5f), m.picks);
        }
        total.Add(m);
        if (release)
        {
            total.offLatency = m.offLatency;
            total.btLatency = m.btLatency;
            total.onAtRelease = m.onAtRelease;
        }
    }
    return total;
}
} // namespace strafet

void SectionStrafe(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionStrafe");
    (void)argc;
    using namespace strafet;
    using A = AimStep;
    std::string v6 = argv[1];
    {
        const size_t sl = v6.find_last_of("\\/");
        v6 = (sl == std::string::npos ? std::string() : v6.substr(0, sl + 1)) + "utpp_base.utppmm";
    }
    static utpp::mm::Database tdb;
    std::string err;
    const bool dbOk = tdb.LoadFromFile(v6.c_str(), kExpectedRigHash, err);
    Check("B2 strafe: database utpp_base read", dbOk, err);
    if (!dbOk)
        return;
    g_db = &tdb;
    {
        char ed[4] = {};
        g_dbgSteps = GetEnvironmentVariableA("UTPP_STRAFE_DBG", ed, sizeof ed) > 0 && ed[0] == '1';
        char ep[4] = {};
        const bool noPat = GetEnvironmentVariableA("UTPP_STRAFE_NOPAT", ep, sizeof ep) > 0 && ep[0] == '1';
        SetStrafePatterns(!noPat); // diagnostic A/B: patterns out of the strafe set
        if (noPat)
            std::printf("[INFO] B2 strafe: UTPP_STRAFE_NOPAT=1 - patterns excluded while strafing (diagnostic)\n");
    }
    {
        const float fps = tdb.Fps() > 0.0f ? tdb.Fps() : 30.0f;
        g_ci.cumYawDeg.assign(tdb.NumClips(), 0.0f);
        g_ci.ownSpeed.assign(tdb.NumClips(), 0.0f);
        for (uint32_t c = 0; c < tdb.NumClips(); ++c)
        {
            const utpp::mm::ClipRecord& cr = tdb.Clips()[c];
            float acc = 0.0f, mx = 0.0f;
            double ss = 0.0;
            for (uint32_t f = 0; f < cr.numFrames; ++f)
                if (const utpp::mm::RootDelta* r = tdb.Root(cr.firstFrame + f))
                {
                    acc += r->dyaw;
                    mx = std::fmax(mx, std::fabs(acc));
                    ss += std::sqrt(double(r->dx) * r->dx + double(r->dy) * r->dy);
                }
            g_ci.cumYawDeg[c] = mx / kDegT;
            if (tdb.ClipName(c).find("_Loop") != std::string_view::npos && g_ci.cumYawDeg[c] <= 0.05f && cr.numFrames > 0)
                g_ci.ownSpeed[c] = float(ss / cr.numFrames) * fps;
        }
    }
    const std::wstring p(v6.begin(), v6.end());
    if (!LoadDatabase(p, "strafe_f"))
    {
        Check("B2 strafe: database loaded", false, v6);
        return;
    }
    OnArmed(aRig, aHashes, aParents);
    {
        // test-side profiles with the driver's state tag masks (stand / crouch) for the rate-law expectation S(theta)
        const std::string dg = DerivedClassDataDigest();
        const size_t mp = dg.find("masks=");
        if (mp != std::string::npos)
        {
            char* e1 = nullptr;
            const unsigned long long ms = std::strtoull(dg.c_str() + mp + 6, &e1, 16);
            const unsigned long long mc = (e1 && *e1 == '/') ? std::strtoull(e1 + 1, nullptr, 16) : 0ull;
            std::vector<uint8_t> cls(tdb.NumClips());
            for (uint32_t c = 0; c < tdb.NumClips(); ++c)
                cls[c] = utpp::mm::steer::DriverClipClass(tdb.ClipName(c), utpp::mm::steer::ClassifyClip(tdb.ClipName(c)), true);
            utpp::mm::strafe::ComputeStrafeProfiles(tdb, cls, ms, mc, utpp::mm::strafe::StrafeConfig{}, g_prof);
            g_haveProf = g_prof.avail[0];
        }
        Check("B2 strafe: test-side speed map built from the driver's state tag masks", g_haveProf, dg.substr(0, 80));
    }
    SetUpperMask(true);
    SetUpperMaskStates(7);
    SetHoldPose(true);
    SetStrafe(true);
    std::printf("[INFO] B2 strafe status: %s\n", StatusFrom(" strafe=").c_str());

    // scenarios
    std::vector<A> dirs;
    for (int k = 0; k < 8; ++k)
    {
        const float a = 45.0f * float(k);
        const std::string n = "dir" + std::to_string(int(a));
        dirs.push_back(A{.name = n + "_idle", .seconds = 0.5f, .angle = a, .stick = 0.0f});
        dirs.push_back(A{.name = n + "_start", .seconds = 0.6f, .angle = a});
        dirs.push_back(A{.name = n + "_cruise", .seconds = 2.0f, .angle = a, .cruise = true});
        dirs.push_back(A{.name = n + "_stop", .seconds = 0.8f, .angle = a, .stick = 0.0f});
    }
    std::vector<A> sweep = {A{.name = "sw_start", .seconds = 0.6f, .angle = 0.0f}};
    for (int k = 1; k <= 12; ++k)
        sweep.push_back(A{.name = "swL" + std::to_string(15 * k), .seconds = 0.5f, .angle = 15.0f * float(k)});
    sweep.push_back(A{.name = "sw_mid", .seconds = 0.6f, .angle = 0.0f});
    for (int k = 1; k <= 12; ++k)
        sweep.push_back(A{.name = "swR" + std::to_string(15 * k), .seconds = 0.5f, .angle = -15.0f * float(k)});
    sweep.push_back(A{.name = "sw_stop", .seconds = 0.8f, .stick = 0.0f});
    const std::vector<A> rev = {A{.name = "LL", .seconds = 1.5f, .angle = 90.0f},   A{.name = "RL", .seconds = 1.5f, .angle = -90.0f},
                                A{.name = "F", .seconds = 1.5f, .angle = 0.0f},     A{.name = "B", .seconds = 1.5f, .angle = 180.0f},
                                A{.name = "FL", .seconds = 1.5f, .angle = 45.0f},   A{.name = "BR", .seconds = 1.5f, .angle = -135.0f},
                                A{.name = "rv_stop", .seconds = 0.8f, .stick = 0.0f}};
    const std::vector<A> pans = {A{.name = "pan_start", .seconds = 0.6f, .angle = 90.0f},
                                 A{.name = "pan90", .seconds = 2.0f, .angle = 90.0f, .camRate = 90.0f},
                                 A{.name = "pan180", .seconds = 2.0f, .angle = 90.0f, .camRate = 180.0f},
                                 A{.name = "pan_stop", .seconds = 0.8f, .stick = 0.0f}};
    const std::vector<A> idlePans = {A{.name = "ip_idle", .seconds = 0.5f, .stick = 0.0f},
                                     A{.name = "ip60", .seconds = 2.0f, .stick = 0.0f, .camRate = 60.0f},
                                     A{.name = "ip180", .seconds = 1.0f, .stick = 0.0f, .camRate = 180.0f},
                                     A{.name = "ip_flick", .seconds = 1.0f, .stick = 0.0f, .camFlick = 40.0f}};
    const std::vector<A> sprintAim = {A{.name = "sprint", .seconds = 2.0f, .angle = 0.0f, .loco = 3, .aim = false, .speed = 8.25f},
                                      A{.name = "aim_from_sprint", .seconds = 2.0f, .angle = 0.0f},
                                      A{.name = "aim_side", .seconds = 1.5f, .angle = 90.0f},
                                      A{.name = "release_run", .seconds = 1.5f, .angle = 90.0f, .loco = 1, .aim = false}};
    const std::vector<A> crouch = {A{.name = "c_idle", .seconds = 0.6f, .stick = 0.0f, .loco = 2},
                                   A{.name = "c_L", .seconds = 1.5f, .angle = 90.0f, .loco = 2, .cruise = true},
                                   A{.name = "c_R", .seconds = 1.5f, .angle = -90.0f, .loco = 2, .cruise = true},
                                   A{.name = "c_B", .seconds = 1.5f, .angle = 180.0f, .loco = 2, .cruise = true},
                                   A{.name = "c_FL", .seconds = 1.5f, .angle = 45.0f, .loco = 2, .cruise = true},
                                   A{.name = "c_stop", .seconds = 0.8f, .stick = 0.0f, .loco = 2}};
    struct Sc
    {
        const char* name;
        const std::vector<A>* steps;
    };
    const Sc scs[] = {{"8 directions", &dirs}, {"15 deg sweep", &sweep}, {"reversals", &rev}, {"mouse pans", &pans},
                      {"idle aim pans", &idlePans}, {"aim from sprint / release into run", &sprintAim}, {"crouch aim", &crouch}};
    AimMetrics on, offAll, stand;
    float offLat = -1.0f, btLat = -1.0f;
    bool onAtRelease = false;
    int sprintStrafeFrames = 0;
    for (int pass = 0; pass < 2; ++pass)
    {
        SetStrafe(pass == 0);
        for (const Sc& sc : scs)
        {
            const AimMetrics r = RunAim(*sc.steps, 1.0f / 60.0f, pass == 1);
            std::printf("[INFO] B2 strafe %s [%s] 60 Hz: %s%s%s\n", pass == 0 ? "ON " : "OFF", sc.name, r.Line().c_str(),
                        r.turnList.empty() ? "" : " | turn picks:", r.turnList.c_str());
            if (pass == 0)
            {
                on.Add(r);
                if (sc.steps != &crouch)
                    stand.Add(r);
                if (sc.steps == &sprintAim)
                {
                    offLat = r.offLatency;
                    btLat = r.btLatency;
                    onAtRelease = r.onAtRelease;
                    sprintStrafeFrames = r.strafeFrames;
                }
            }
            else if (sc.steps == &dirs || sc.steps == &idlePans || sc.steps == &rev)
                offAll.Add(r);
        }
    }
    SetStrafe(true);
    std::printf("[INFO] B2 strafe search time (patterns in the strafe set): %s\n", StatusFrom(" strafeSearches=").c_str());
    AimMetrics dirsOn, pansOn, standSteady, offDirs;
    {
        // per-scenario ON metrics again for the scoped checks (same deterministic runs as above)
        SetStrafe(true);
        dirsOn = RunAim(dirs, 1.0f / 60.0f);
        pansOn = RunAim(pans, 1.0f / 60.0f);
        SetStrafe(false);
        offDirs = RunAim(dirs, 1.0f / 60.0f, true);
        SetStrafe(true);
    }
    // S1 criteria (tests_offline, strafe ON)
    Check("B2 strafe [v6] 60 Hz: 8 directions (start / cruise / stop) + mouse pans - playing root direction within +-30 deg of the "
          "capsule (body frame) on >= 90 % of moving frames",
          dirsOn.Agree() >= 0.9f && pansOn.Agree() >= 0.9f && dirsOn.moveFrames > 500,
          dirsOn.Line() + " | pans " + std::to_string(pansOn.Agree()));
    Check("B2 strafe [v6]: 0 StrafeTurn / Turn / Reface / TurnInPlace (ROOT >= 10 deg, turn class, off-band pattern) picks while strafe=1, "
          "every scenario (moving and idle aim pans)",
          on.turnPicks == 0 && on.picks > 20, on.Line() + (on.turnList.empty() ? "" : " |" + on.turnList));
    // engagement guards (review): these checks must not pass vacuously with strafe never engaged (0/0 frames)
    const bool engaged = on.strafeFrames > 1000 && on.moveFrames > 500;
    Check("B2 strafe [v6]: played root yaw per step <= 10 deg (strafe engaged)", engaged && on.yawStepMax <= 10.0f, on.Line());
    Check("B2 strafe [v6]: steady strafe (last second of the 8 cruise directions, stand): foot-lock correction max <= 15 cm, "
          "direction agreement >= 90 %",
          dirsOn.steadyCorrMax <= 15.0f && dirsOn.steadyN > 200 && dirsOn.steadyAgree >= 0.9f * float(dirsOn.steadyN), dirsOn.Line());
    Check("B2 strafe [v6]: <= 2.5 transitions/s while cruising (strafe engaged)",
          engaged && on.cruiseSec > 0.0f && on.cruiseTrans / on.cruiseSec <= 2.5f, on.Line());
    Check("B2 strafe [v6]: playback rate within [0.6, 1.4] with the stick held; steady rate = capsule / own loop speed or aim point / "
          "S(theta) within 5 %",
          on.rateOut == 0 && on.rateErrN > 100 && on.rateErrMax <= 0.05f, on.Line());
    Check("B2 strafe [v6]: aim released while moving -> strafe=0 within 0.15 s + 1 frame, body turn (btFed=1) within 0.2 s",
          onAtRelease && sprintStrafeFrames > 100 && offLat >= 0.0f && offLat <= 0.15f + 1.0f / 60.0f + 1e-4f && btLat >= 0.0f &&
              btLat <= 0.2f + 1e-4f,
          "strafe off after " + std::to_string(offLat) + " s, btFed after " + std::to_string(btLat) + " s, strafe=1 before release " +
              std::to_string(onAtRelease ? 1 : 0) + ", strafe frames " + std::to_string(sprintStrafeFrames));
    Check("B2 strafe [v6]: strafe engaged on the aim feeds (stand and crouch)", stand.strafeFrames > 1000 && on.strafeFrames > stand.strafeFrames,
          std::to_string(stand.strafeFrames) + " / " + std::to_string(on.strafeFrames));
    Check("B2 strafe [v6]: strafe ON beats the 0.6.5 path on every aim scenario set (agreement, offset saturation, turn picks)",
          engaged && offAll.moveFrames > 500 && dirsOn.strafeFrames > 500 && on.Agree() > offAll.Agree() + 0.3f &&
              dirsOn.offSatMax < offDirs.offSatMax && on.turnPicks < offAll.turnPicks,
          "agree " + std::to_string(on.Agree()) + " vs " + std::to_string(offAll.Agree()) + ", offSat " + std::to_string(dirsOn.offSatMax) +
              " vs " + std::to_string(offDirs.offSatMax));
    // design targets not met by S1 (reported, not hidden; see the stage report): the whole-scenario numbers including the
    // start / stop transitions with the vanilla 24 m/s^2 capsule (the 0.6.5 path has the same foot-lock clamp there), the
    // 15 deg sweep / reversals (pattern / pivot segments), crouch-aim (v6 crouch cardinals: 30 searchable frames)
    {
        const bool agreeAll = on.Agree() >= 0.9f, flAll = on.corrMax <= 15.0f, satAll = on.offSatMax <= 0.3f;
        std::printf("[INFO] B2 strafe S1 design target overall direction agreement >= 90 %%: %s (%.1f %%; 0.6.5 path %.1f %%)\n",
                    agreeAll ? "met" : "NOT MET", 100.0 * on.Agree(), 100.0 * offAll.Agree());
        std::printf("[INFO] B2 strafe S1 design target foot-lock max <= 15 cm over every frame (starts / stops included): %s (%.1f cm; 0.6.5 "
                    "path %.1f cm)\n",
                    flAll ? "met" : "NOT MET", on.corrMax, offAll.corrMax);
        std::printf("[INFO] B2 strafe S1 design target offset root saturated <= 0.3 s per step: %s (%.2f s; 0.6.5 path %.2f s)\n",
                    satAll ? "met" : "NOT MET", on.offSatMax, offAll.offSatMax);
    }
    // ---- B2 strafe S2 chest align (default on): the published pose with ChestAlign on against the same deterministic runs with it
    // off. Test-side ChestAlign (same reference clips as the driver) measures the twist of the PUBLISHED chest (Spine model
    // rotation about Z vs the stance's forward-walk reference) independently of the driver's diagnostics.
    {
        using namespace utpp::mm;
        constexpr float kRad2DegS2 = 180.0f / math::kPi;
        ChestAlign meas;
        bool refOk = meas.Init(tdb);
        const char* refName[2] = {"M_Neutral_Walk_Loop_F", "M_Neutral_Crouch_Loop_F"};
        for (int si = 0; si < 2 && refOk; ++si)
        {
            const int32_t c = ChestAlign::FindClip(tdb, refName[si]);
            refOk = c >= 0;
            if (refOk)
            {
                const ClipRecord& cr = tdb.Clips()[size_t(c)];
                refOk = meas.BuildReference(tdb, si, cr.firstFrame, (cr.loopPeriod > 0 && cr.loopPeriod <= cr.numFrames) ? cr.loopPeriod : cr.numFrames);
            }
        }
        Check("B2 strafe S2 [v6]: test-side chest references (stand / crouch forward loops)", refOk);
        const std::string st0 = Status();
        Check("B2 strafe S2 [v6]: driver chest align on by default, ready, stand / crouch references built, config 0.5 / 60",
              ChestAlignEnabled() && Field(st0, "chestAlign") > 0.5f && Field(st0, "chestReady") > 0.5f &&
                  st0.find(" chestRef=1/1") != std::string::npos && st0.find(" chestCfg=0.50/60.0") != std::string::npos,
              StatusFrom(" chestAlign="));
        struct Rec
        {
            std::vector<BoneTransform> pose;
            bool strafe = false, valid = false, steady = false;
            int stance = 0;
            float w = 0.0f, dev = 0.0f, corr = 0.0f, low = 0.0f, blend = 0.0f;
        };
        auto capture = [&](std::vector<Rec>& aOut)
        {
            aOut.clear();
            g_chestProbe = [&aOut, last = (const AimStep*)nullptr, idx = 0](const std::string& aStatus, const AimStep& aSt, bool aStrafe) mutable
            {
                static utpp::mm::integ::PoseSnapshot snap{};
                Rec r;
                idx = &aSt == last ? idx + 1 : 0;
                last = &aSt;
                // steady: the last second of a cruise step (60 Hz runs)
                r.steady = aSt.cruise && idx >= int(aSt.seconds * 60.0f + 0.5f) - 60;
                if (utpp::g_mmPose.Read(snap))
                {
                    r.pose.resize(snap.numBones);
                    for (uint32_t b = 0; b < snap.numBones; ++b)
                    {
                        const float* q = &snap.pose[size_t(b) * 7];
                        r.pose[b].p = {q[0], q[1], q[2]};
                        r.pose[b].q = {q[3], q[4], q[5], q[6]};
                    }
                }
                r.strafe = aStrafe;
                r.stance = aSt.loco == 2 ? ChestAlign::kCrouch : ChestAlign::kStand;
                r.valid = Field(aStatus, "chestValid") > 0.5f;
                r.w = Field(aStatus, "chestW");
                r.dev = Field(aStatus, "chestDev");
                r.corr = Field(aStatus, "chestCorr");
                r.low = Field(aStatus, "chestLow");
                r.blend = Field(aStatus, "chestBlend");
                aOut.push_back(std::move(r));
            };
        };
        struct Run
        {
            const char* name;
            const std::vector<A>* steps;
        };
        // aim on / off at walk / idle (no running: deterministic across runs): strafe entries (mask fade-in, one-frame lag) and
        // strafe-off frames = the weight-0 frames
        const std::vector<A> toggles = {A{.name = "t_free", .seconds = 0.5f, .stick = 0.0f, .aim = false},
                                        A{.name = "t_aim_idle", .seconds = 0.6f, .stick = 0.0f},
                                        A{.name = "t_free2", .seconds = 0.6f, .stick = 0.0f, .aim = false},
                                        A{.name = "t_aim_left", .seconds = 1.2f, .angle = 90.0f},
                                        A{.name = "t_free3", .seconds = 0.8f, .stick = 0.0f, .aim = false},
                                        A{.name = "t_aim_back", .seconds = 1.2f, .angle = 180.0f},
                                        A{.name = "t_free4", .seconds = 0.8f, .stick = 0.0f, .aim = false},
                                        A{.name = "t_c_aim", .seconds = 1.2f, .angle = -90.0f, .loco = 2},
                                        A{.name = "t_c_free", .seconds = 0.8f, .stick = 0.0f, .loco = 2, .aim = false}};
        const Run runs[] = {{"8 directions", &dirs}, {"reversals", &rev}, {"crouch aim", &crouch}, {"aim from sprint", &sprintAim},
                            {"aim toggles", &toggles}};
        SetStrafe(true);
        float p95On[2] = {0, 0}, p95Off[2] = {0, 0}, meanOn[2] = {0, 0}, meanOff[2] = {0, 0}, maxOn[2] = {0, 0}, maxOff[2] = {0, 0};
        std::vector<float> devOn[2], devOff[2], steadyOn[2], steadyOff[2], devK0[2], lagErr[2];
        int ctlDiff = 0, ctlFrames = 0, nondet = 0;
        int frames = 0, legsDiff = 0, w0Frames = 0, w0Strafe = 0, w0Diff = 0, spineChanged = 0, strafeFull = 0, sizeMismatch = 0;
        int validFrames = 0, nonStrafeValid = 0, blendDiagFrames = 0;
        float diagErrMax = 0.0f;
        for (const Run& run : runs)
        {
            // RunAim restarts the matcher only (body yaw, weights, mask blend, offset root carry over): every compared run follows
            // a run of the same scenario. Order: warm-up off, ON (after off), off (after ON), control off (after off) - the
            // ON / off pair differs only by ChestAlign, the off / control pair shows that ChestAlign leaks nothing into the state.
            std::vector<Rec> offR, onR, ctlR, k0R;
            SetChestAlign(false);
            RunAim(*run.steps, 1.0f / 60.0f);
            RunAim(*run.steps, 1.0f / 60.0f); // two warm-ups: slow leg filters carry over two runs back
            SetChestAlign(true);
            capture(onR);
            RunAim(*run.steps, 1.0f / 60.0f);
            SetChestAlign(false);
            capture(offR);
            RunAim(*run.steps, 1.0f / 60.0f);
            capture(ctlR);
            RunAim(*run.steps, 1.0f / 60.0f);
            SetChestAlign(true);
            // swayKeep 0 (console param): the whole twist is cancelled, the residual is only the filter lag
            SetChestAlignParams(0.0f, 60.0f);
            capture(k0R);
            RunAim(*run.steps, 1.0f / 60.0f);
            SetChestAlignParams(0.5f, 60.0f);
            g_chestProbe = nullptr;
            for (size_t i = 0; i < std::min(k0R.size(), onR.size()); ++i)
            {
                const Rec& r = k0R[i];
                if (r.strafe && r.valid && r.w >= 0.999f && !r.pose.empty())
                    devK0[r.stance].push_back(std::fabs(meas.Deviation(r.pose, r.stance) * kRad2DegS2));
            }
            // per-frame determinism: the driver carries state across runs that never repeats (sinceStance, the adaptive rate
            // factor; seen on the sprint run of the full suite): a frame whose two chest-off runs differ is excluded from the
            // ON / off comparisons (counted and reported)
            std::vector<char> det(offR.size(), 0);
            for (size_t i = 0; i < std::min(offR.size(), ctlR.size()); ++i)
            {
                ++ctlFrames;
                det[i] = (offR[i].pose.size() == ctlR[i].pose.size() &&
                          std::memcmp(offR[i].pose.data(), ctlR[i].pose.data(), offR[i].pose.size() * sizeof(BoneTransform)) == 0)
                             ? 1
                             : 0;
                ctlDiff += det[i] ? 0 : 1;
            }
            if (onR.size() != offR.size())
            {
                ++sizeMismatch;
                continue;
            }
            for (size_t i = 0; i < onR.size(); ++i)
            {
                const Rec& a = onR[i];
                const Rec& b = offR[i];
                if (a.pose.size() != b.pose.size() || a.pose.empty())
                {
                    ++sizeMismatch;
                    continue;
                }
                if (!det[i])
                {
                    ++nondet;
                    continue;
                }
                if (a.strafe != b.strafe)
                {
                    ++sizeMismatch;
                    continue;
                }
                ++frames;
                validFrames += a.valid ? 1 : 0;
                nonStrafeValid += (a.valid && !a.strafe) ? 1 : 0;
                for (size_t k = 0; k < a.pose.size(); ++k)
                    if (int32_t(k) != meas.SpineBone() && std::memcmp(&a.pose[k], &b.pose[k], sizeof(BoneTransform)) != 0)
                    {
                        if (g_dbgSteps && legsDiff < 12)
                            std::printf("[DBG] chest legs diff run %s frame %zu bone %zu %.*s dp %.6f %.6f %.6f dq %.6f %.6f %.6f %.6f w %.3f strafe %d\n", run.name, i, k,
                                        int(tdb.BoneName(uint32_t(k)).size()), tdb.BoneName(uint32_t(k)).data(), a.pose[k].p.x - b.pose[k].p.x,
                                        a.pose[k].p.y - b.pose[k].p.y, a.pose[k].p.z - b.pose[k].p.z, a.pose[k].q.x - b.pose[k].q.x,
                                        a.pose[k].q.y - b.pose[k].q.y, a.pose[k].q.z - b.pose[k].q.z, a.pose[k].q.w - b.pose[k].q.w, a.w, a.strafe ? 1 : 0);
                        ++legsDiff;
                        break;
                    }
                const bool spineSame = std::memcmp(&a.pose[size_t(meas.SpineBone())], &b.pose[size_t(meas.SpineBone())], sizeof(BoneTransform)) == 0;
                if (!(a.w > 0.0f))
                {
                    ++w0Frames;
                    w0Strafe += a.strafe ? 1 : 0;
                    w0Diff += spineSame ? 0 : 1;
                }
                else
                    spineChanged += spineSame ? 0 : 1;
                if (!a.strafe || a.w < 0.999f || !a.valid)
                    continue;
                ++strafeFull;
                const float dOn = meas.Deviation(a.pose, a.stance) * kRad2DegS2;
                const float dOff = meas.Deviation(b.pose, b.stance) * kRad2DegS2;
                devOn[a.stance].push_back(std::fabs(dOn));
                devOff[a.stance].push_back(std::fabs(dOff));
                if (a.steady)
                {
                    steadyOn[a.stance].push_back(std::fabs(dOn));
                    steadyOff[a.stance].push_back(std::fabs(dOff));
                }
                meanOn[a.stance] += dOn;
                // correction error beyond the deliberately kept sway: output - swayKeep x (input - low-passed input)
                lagErr[a.stance].push_back(std::fabs((a.dev + a.corr) - 0.5f * (a.dev - a.low)));
                meanOff[a.stance] += dOff;
                // driver diagnostics: input twist (status, 2 decimals) = the twist of the chest-off pose; output = input + correction.
                // chest_repair: during a stance-switch blend (0 < chestBlend < 1) the driver measures against the blended reference
                if (a.blend > 0.0f && a.blend < 1.0f)
                {
                    auto blended = [&](const std::vector<BoneTransform>& aP)
                    {
                        const float dS = meas.Deviation(aP, ChestAlign::kStand), dC = meas.Deviation(aP, ChestAlign::kCrouch);
                        return math::WrapAngle(dS + a.blend * math::WrapAngle(dC - dS)) * kRad2DegS2;
                    };
                    ++blendDiagFrames;
                    diagErrMax = std::max({diagErrMax, std::fabs(a.dev - blended(b.pose)), std::fabs(a.dev + a.corr - blended(a.pose))});
                }
                else
                    diagErrMax = std::max({diagErrMax, std::fabs(a.dev - dOff), std::fabs(a.dev + a.corr - dOn)});
            }
        }
        auto p95 = [](std::vector<float> v)
        {
            if (v.empty())
                return 0.0f;
            std::sort(v.begin(), v.end());
            return v[std::min(v.size() - 1, size_t(0.95 * double(v.size())))];
        };
        for (int si = 0; si < 2; ++si)
        {
            p95On[si] = p95(devOn[si]);
            p95Off[si] = p95(devOff[si]);
            maxOn[si] = devOn[si].empty() ? 0.0f : *std::max_element(devOn[si].begin(), devOn[si].end());
            maxOff[si] = devOff[si].empty() ? 0.0f : *std::max_element(devOff[si].begin(), devOff[si].end());
            const float n = float(std::max<size_t>(devOn[si].size(), 1));
            meanOn[si] /= n;
            meanOff[si] /= n;
            std::printf("[INFO] B2 strafe S2 chest twist [%s] 60 Hz (%zu full-weight strafe frames): |twist| p95 %.1f deg (S2 off %.1f), max %.1f "
                        "(off %.1f), mean %.1f (off %.1f)\n",
                        si == 0 ? "stand: 8 directions + reversals" : "crouch aim", devOn[si].size(), p95On[si], p95Off[si], maxOn[si],
                        maxOff[si], meanOn[si], meanOff[si]);
            std::printf("[INFO] B2 strafe S2 chest twist [%s] steady cruise (last second, %zu frames): |twist| p95 %.1f deg (S2 off %.1f)\n",
                        si == 0 ? "stand" : "crouch", steadyOn[si].size(), p95(steadyOn[si]), p95(steadyOff[si]));
            std::printf("[INFO] B2 strafe S2 chest twist [%s] swayKeep 0 (console param): |twist| p95 %.1f deg (%zu frames); default "
                        "swayKeep 0.5: correction error beyond the kept sway |output - 0.5 x (input - low)| p95 %.1f deg\n",
                        si == 0 ? "stand" : "crouch", p95(devK0[si]), devK0[si].size(), p95(lagErr[si]));
        }
        std::printf("[INFO] B2 strafe S2 chest: determinism control (chest off after chest ON vs chest off after chest off): %d / %d "
                    "frames differ (excluded from the ON / off comparisons: %d)\n",
                    ctlDiff, ctlFrames, nondet);
        std::printf("[INFO] B2 strafe S2 chest: %d frames, %d valid (%d outside strafe), %d at weight 0 (%d strafing), %d with the Spine "
                    "changed, legs / hips differing on %d, diagnostics vs measured max err %.2f deg, mismatched runs %d\n",
                    frames, validFrames, nonStrafeValid, w0Frames, w0Strafe, spineChanged, legsDiff, diagErrMax, sizeMismatch);
        std::printf("[INFO] B2 strafe S2 chest: %d full-weight frames inside a stance-switch reference blend (chest_repair)\n", blendDiagFrames);
        // Design target "status chestDev after correction <= 10 deg p95 in strafe": met by the mechanism with swayKeep 0 (the whole
        // twist cancelled, residual = filter lag); the default swayKeep 0.5 keeps half the step sway around the low-passed mean on
        // purpose (v6 side / back loops swing +-10..21 deg), so its p95 is reported against the target, not gated.
        {
            const bool met = p95On[0] <= 10.0f && p95On[1] <= 10.0f;
            std::printf("[INFO] B2 strafe S2 design target chest twist after the correction p95 <= 10 deg with the default swayKeep 0.5: %s "
                        "(stand %.1f, crouch %.1f; swayKeep 0: stand %.1f, crouch %.1f; S1 baseline stand %.1f, crouch %.1f)\n",
                        met ? "met" : "NOT MET (kept sway)", p95On[0], p95On[1], p95(devK0[0]), p95(devK0[1]), p95Off[0], p95Off[1]);
        }
        Check("B2 strafe S2 [v6]: swayKeep 0 - chest twist after the correction p95 <= 10 deg while strafing (stand and crouch; S1 "
              "baseline > 30 deg)",
              devK0[0].size() > 500 && devK0[1].size() > 100 && p95(devK0[0]) <= 10.0f && p95(devK0[1]) <= 10.0f && p95Off[0] > 30.0f &&
                  p95Off[1] > 30.0f,
              "stand p95 " + std::to_string(p95(devK0[0])) + ", crouch p95 " + std::to_string(p95(devK0[1])));
        Check("B2 strafe S2 [v6]: default swayKeep 0.5 - correction error beyond the kept sway p95 <= 10 deg, twist p95 below half the S1 "
              "baseline, mean twist within 2 deg (stand and crouch)",
              devOn[0].size() > 500 && devOn[1].size() > 100 && p95(lagErr[0]) <= 10.0f && p95(lagErr[1]) <= 10.0f &&
                  p95On[0] < 0.5f * p95Off[0] && p95On[1] < 0.5f * p95Off[1] && std::fabs(meanOn[0]) <= 2.0f && std::fabs(meanOn[1]) <= 2.0f,
              "stand p95 " + std::to_string(p95On[0]) + " (off " + std::to_string(p95Off[0]) + ", beyond sway " + std::to_string(p95(lagErr[0])) +
                  "), crouch p95 " + std::to_string(p95On[1]) + " (off " + std::to_string(p95Off[1]) + ", beyond sway " +
                  std::to_string(p95(lagErr[1])) + ")");
        // timing variants (filters integrate the real dt): 30 / 144 Hz, 8 directions, stand
        for (const float hz : {30.0f, 144.0f})
        {
            std::vector<Rec> hzOn, hzOff;
            SetChestAlign(false);
            RunAim(dirs, 1.0f / hz);
            RunAim(dirs, 1.0f / hz);
            SetChestAlign(true);
            capture(hzOn);
            RunAim(dirs, 1.0f / hz);
            SetChestAlign(false);
            capture(hzOff);
            RunAim(dirs, 1.0f / hz);
            SetChestAlign(true);
            g_chestProbe = nullptr;
            std::vector<float> a, b;
            int legs = 0;
            for (size_t i = 0; i < std::min(hzOn.size(), hzOff.size()); ++i)
            {
                const Rec& x = hzOn[i];
                const Rec& y = hzOff[i];
                if (x.pose.size() != y.pose.size() || x.pose.empty())
                    continue;
                for (size_t k = 0; k < x.pose.size(); ++k)
                    if (int32_t(k) != meas.SpineBone() && std::memcmp(&x.pose[k], &y.pose[k], sizeof(BoneTransform)) != 0)
                    {
                        ++legs;
                        break;
                    }
                if (!x.strafe || !x.valid || x.w < 0.999f)
                    continue;
                a.push_back(std::fabs(meas.Deviation(x.pose, x.stance) * kRad2DegS2));
                b.push_back(std::fabs(meas.Deviation(y.pose, y.stance) * kRad2DegS2));
            }
            char nm[32];
            std::snprintf(nm, sizeof nm, "%.0f Hz", static_cast<double>(hz));
            std::printf("[INFO] B2 strafe S2 chest twist [stand: 8 directions] %s (%zu frames): |twist| p95 %.1f deg (S2 off %.1f)\n", nm,
                        a.size(), p95(a), p95(b));
            Check(std::string("B2 strafe S2 [v6] ") + nm + ": 8 directions - chest twist p95 below half the S1 baseline, bones but the Spine "
                                                      "bit-identical",
                  a.size() > 200 && p95(a) < 0.5f * p95(b) && legs == 0,
                  std::to_string(p95(a)) + " vs " + std::to_string(p95(b)) + ", legs differing " + std::to_string(legs));
        }
        // chest_repair: crouch toggles WHILE AIMING (the S2 runs above start already crouched). The driver stance flips on the
        // input frame, the matcher pose crosses to the crouch loops over several frames: the published chest twist (vs the
        // fixed STAND reference) must move between its settled stand / crouch values without overshooting. A / B against the
        // old snap switch (SetChestAlignStanceBlend(0), C++ test hook) and chest align off.
        {
            const std::vector<A> ctog = {A{.name = "ct_aim", .seconds = 1.2f, .angle = 0.0f},
                                         A{.name = "ct_c1", .seconds = 1.5f, .angle = 0.0f, .loco = 2},
                                         A{.name = "ct_s1", .seconds = 1.5f, .angle = 0.0f},
                                         A{.name = "ct_c2", .seconds = 1.5f, .angle = 90.0f, .loco = 2},
                                         A{.name = "ct_s2", .seconds = 1.5f, .angle = 90.0f},
                                         A{.name = "ct_ci", .seconds = 1.5f, .stick = 0.0f, .loco = 2},
                                         A{.name = "ct_si", .seconds = 1.5f, .stick = 0.0f},
                                         A{.name = "ct_stop", .seconds = 0.5f, .stick = 0.0f, .aim = false}};
            const math::Quat rs = meas.Reference(ChestAlign::kStand);
            struct TogM
            {
                float overshoot = 0.0f;
                int toggles = 0, blendFrames = 0, strafeLost = 0;
                std::string per;
            };
            auto measure = [&](const std::vector<Rec>& aR)
            {
                TogM m;
                std::vector<float> o(aR.size(), 0.0f);
                for (size_t i = 0; i < aR.size(); ++i)
                    if (!aR[i].pose.empty())
                        o[i] = ChestAlign::TwistZ(math::QuatMul(math::QuatNormalize(meas.Model(aR[i].pose, meas.SpineBone()).q), math::QuatInv(rs))) *
                               kRad2DegS2;
                // step boundaries (60 Hz): frame index where each step starts
                std::vector<int> start;
                int acc = 0;
                for (const A& st : ctog)
                {
                    start.push_back(acc);
                    acc += int(st.seconds * 60.0f + 0.5f);
                }
                for (size_t i = 0; i < aR.size(); ++i)
                    m.blendFrames += (aR[i].valid && aR[i].blend > 0.0f && aR[i].blend < 1.0f) ? 1 : 0;
                // toggles: steps 1..6 (each a stance flip with aim held); band = settled tails (last 0.3 s) of the step before and
                // of this step; overshoot = excursion outside the band over this step
                for (size_t k = 1; k + 1 < ctog.size(); ++k)
                {
                    const int b1 = start[k], e1 = start[k + 1];
                    if (e1 > int(aR.size()) || b1 < 18)
                        break;
                    float lo = 1e9f, hi = -1e9f;
                    for (int i = b1 - 18; i < b1; ++i)
                    {
                        lo = std::min(lo, o[size_t(i)]);
                        hi = std::max(hi, o[size_t(i)]);
                    }
                    for (int i = e1 - 18; i < e1; ++i)
                    {
                        lo = std::min(lo, o[size_t(i)]);
                        hi = std::max(hi, o[size_t(i)]);
                    }
                    float ov = 0.0f;
                    for (int i = b1; i < e1; ++i)
                    {
                        m.strafeLost += aR[size_t(i)].strafe ? 0 : 1;
                        ov = std::max({ov, o[size_t(i)] - hi, lo - o[size_t(i)]});
                    }
                    ++m.toggles;
                    m.overshoot = std::max(m.overshoot, ov);
                    char b[48];
                    std::snprintf(b, sizeof b, "%s%s %.1f", m.per.empty() ? "" : ",", ctog[k].name.c_str(), static_cast<double>(ov));
                    m.per += b;
                }
                return m;
            };
            std::vector<Rec> tOn, tSnap, tOff;
            SetChestAlign(true);
            RunAim(ctog, 1.0f / 60.0f);
            RunAim(ctog, 1.0f / 60.0f);
            capture(tOn);
            RunAim(ctog, 1.0f / 60.0f);
            const bool snapSet = SetChestAlignStanceBlend(0.0f);
            capture(tSnap);
            RunAim(ctog, 1.0f / 60.0f);
            const bool restored = SetChestAlignStanceBlend(0.8f);
            SetChestAlign(false);
            capture(tOff);
            RunAim(ctog, 1.0f / 60.0f);
            SetChestAlign(true);
            g_chestProbe = nullptr;
            const TogM mOn = measure(tOn), mSnap = measure(tSnap), mOff = measure(tOff);
            // reference-switch artifact (the review finding): chest yaw that moves AHEAD of the pose. Window = the frames after the
            // stance flip while the published chest-off pose has not left the old stance yet (test-side Hips-height crouch
            // fraction < 0.1 going down / > 0.9 going up, at most 0.5 s); lead = max |(out - out_before) - (off - off_before)|.
            // The overshoot above is dominated by the matcher's own transition twist (chest off shows it too) and the devLow
            // lag, so it is reported, not gated.
            auto lead = [&](const std::vector<Rec>& aR, std::string& aPer)
            {
                auto tw = [&](const Rec& r)
                {
                    return r.pose.empty() ? 0.0f
                                          : ChestAlign::TwistZ(math::QuatMul(math::QuatNormalize(meas.Model(r.pose, meas.SpineBone()).q), math::QuatInv(rs))) *
                                                kRad2DegS2;
                };
                float worst = -1.0f;
                int acc = 0;
                for (size_t k = 0; k + 1 < ctog.size(); ++k)
                {
                    acc += int(ctog[k].seconds * 60.0f + 0.5f);
                    const int s = acc; // first frame of step k + 1
                    if (k + 1 >= ctog.size() - 1 || s < 1 || s + 30 > int(std::min(aR.size(), tOff.size())))
                        break;
                    const bool toCrouch = ctog[k + 1].loco == 2;
                    const float ob = tw(aR[size_t(s - 1)]), fb = tw(tOff[size_t(s - 1)]);
                    float l = 0.0f;
                    int n = 0;
                    for (int i = s; i < s + 30; ++i)
                    {
                        const float h = tOff[size_t(i)].pose.empty() ? 0.5f : meas.PoseCrouchFraction(tOff[size_t(i)].pose);
                        if (!std::isfinite(h) || (toCrouch ? h >= 0.1f : h <= 0.9f))
                            break;
                        ++n;
                        l = std::max(l, std::fabs((tw(aR[size_t(i)]) - ob) - (tw(tOff[size_t(i)]) - fb)));
                    }
                    worst = std::max(worst, l);
                    char b[48];
                    std::snprintf(b, sizeof b, "%s%s %.1f/%d", aPer.empty() ? "" : ",", ctog[k + 1].name.c_str(), static_cast<double>(l), n);
                    aPer += b;
                }
                return worst;
            };
            std::string leadPerOn, leadPerSnap;
            const float leadOn = lead(tOn, leadPerOn), leadSnap = lead(tSnap, leadPerSnap);
            if (g_dbgSteps) // UTPP_STRAFE_DBG=1: per-frame chest twist of the three runs
                for (size_t i = 0; i < std::min({tOn.size(), tSnap.size(), tOff.size()}); ++i)
                {
                    auto tw = [&](const Rec& r)
                    {
                        return r.pose.empty() ? 0.0f
                                              : ChestAlign::TwistZ(math::QuatMul(math::QuatNormalize(meas.Model(r.pose, meas.SpineBone()).q),
                                                                                 math::QuatInv(rs))) *
                                                    kRad2DegS2;
                    };
                    std::printf("[DBG] ctog %zu st %d on %.2f snap %.2f off %.2f blend %.3f dev %.2f low %.2f corr %.2f | snap dev %.2f low %.2f "
                                "corr %.2f hipsC %.3f\n",
                                i, tOn[i].stance, tw(tOn[i]), tw(tSnap[i]), tw(tOff[i]), tOn[i].blend, tOn[i].dev, tOn[i].low, tOn[i].corr,
                                tSnap[i].dev, tSnap[i].low, tSnap[i].corr,
                                tOff[i].pose.empty() ? -1.0f : meas.PoseCrouchFraction(tOff[i].pose));
                }
            const bool badRejected = !SetChestAlignStanceBlend(-1.0f) && !SetChestAlignStanceBlend(std::nanf("")) && !SetChestAlignStanceBlend(5.5f);
            std::printf("[INFO] B2 strafe S2 chest crouch toggles while aiming (60 Hz, %d toggles): chest yaw overshoot beyond the settled stand / "
                        "crouch values max %.1f deg [%s]; snap switch (old) %.1f deg [%s]; chest off %.1f deg; blend frames %d (snap %d), "
                        "strafe lost %d\n",
                        mOn.toggles, mOn.overshoot, mOn.per.c_str(), mSnap.overshoot, mSnap.per.c_str(), mOff.overshoot, mOn.blendFrames,
                        mSnap.blendFrames, mOn.strafeLost);
            std::printf("[INFO] B2 strafe S2 chest crouch toggles while aiming: chest yaw ahead of the pose (before the pose leaves the old "
                        "stance) max %.1f deg [%s]; snap switch (old) %.1f deg [%s]\n",
                        leadOn, leadPerOn.c_str(), leadSnap, leadPerSnap.c_str());
            Check("B2 strafe S2 [v6] chest_repair: crouch toggles while aiming - chest yaw ahead of the pose <= 4 deg and below half the old "
                  "snap switch, overshoot not above the snap switch; stance blend used, snap = 0 blend frames; stanceBlend setter "
                  "range-checked",
                  mOn.toggles == 6 && mOn.strafeLost == 0 && leadOn >= 0.0f && leadOn <= 4.0f && leadOn < 0.5f * leadSnap &&
                      mOn.overshoot <= mSnap.overshoot && mOn.blendFrames > 0 && mSnap.blendFrames == 0 && snapSet && restored &&
                      badRejected && Status().find(" chestStanceBlend=0.80") != std::string::npos,
                  "lead " + std::to_string(leadOn) + " (snap " + std::to_string(leadSnap) + "), overshoot " + std::to_string(mOn.overshoot) +
                      " (snap " + std::to_string(mSnap.overshoot) + ", off " + std::to_string(mOff.overshoot) + "), blend frames " +
                      std::to_string(mOn.blendFrames));
        }
        Check("B2 strafe S2 [v6]: legs / hips / every bone but the Spine bit-identical to chest align off (every published frame)",
              frames > 2000 && sizeMismatch == 0 && legsDiff == 0,
              std::to_string(legsDiff) + " / " + std::to_string(frames) + " (non-deterministic frames excluded: " + std::to_string(nondet) + ")");
        Check("B2 strafe S2 [v6]: weight 0 (strafe off, mask blend 0) -> the published pose bit-identical, Spine included; weight > 0 "
              "changes the Spine",
              w0Frames > 0 && w0Strafe > 0 && w0Diff == 0 && spineChanged > 500,
              "w0 " + std::to_string(w0Frames) + " (strafing " + std::to_string(w0Strafe) + ", differing " + std::to_string(w0Diff) +
                  "), spine changed " + std::to_string(spineChanged));
        Check("B2 strafe S2 [v6]: diagnostics only while strafing; status chestDev = measured input twist, chestDev + chestCorr = "
              "measured output twist (0.05 deg)",
              nonStrafeValid == 0 && strafeFull > 500 && diagErrMax <= 0.05f, std::to_string(diagErrMax));
        // natives: range / NaN rejected (previous kept), switch
        {
            const bool bad = !SetChestAlignParams(std::nanf(""), 60.0f) && !SetChestAlignParams(0.5f, std::nanf("")) &&
                             !SetChestAlignParams(-0.01f, 60.0f) && !SetChestAlignParams(1.01f, 60.0f) && !SetChestAlignParams(0.5f, -1.0f) &&
                             !SetChestAlignParams(0.5f, 90.5f);
            const std::string sKeep = Status();
            const bool good = SetChestAlignParams(0.0f, 90.0f) && Status().find(" chestCfg=0.00/90.0") != std::string::npos &&
                              SetChestAlignParams(0.5f, 60.0f);
            SetChestAlign(false);
            const std::string sOff = Status();
            SetChestAlign(true);
            Check("B2 strafe S2: UTPPSetMMChestAlignParams rejects NaN / out of range (previous kept), accepts 0..1 / 0..90; "
                  "UTPPSetMMChestAlign switch",
                  bad && good && sKeep.find(" chestCfg=0.50/60.0") != std::string::npos && Field(sOff, "chestAlign") < 0.5f &&
                      ChestAlignEnabled(),
                  StatusFrom(" chestCfg="));
        }
    }
    // timing variants (direction / foot-lock metrics)
    for (const float hz : {30.0f, 144.0f})
    {
        SetStrafe(true);
        const AimMetrics r = RunAim(dirs, 1.0f / hz);
        char nm[64];
        std::snprintf(nm, sizeof nm, "%.0f Hz", static_cast<double>(hz));
        std::printf("[INFO] B2 strafe ON  [8 directions] %s: %s\n", nm, r.Line().c_str());
        Check(std::string("B2 strafe [v6] ") + nm + ": 8 directions - root direction agreement >= 90 %, 0 turn picks",
              r.Agree() >= 0.9f && r.turnPicks == 0 && r.moveFrames > 100, r.Line());
        std::printf("[INFO] B2 strafe S1 design target %s steady foot-lock <= 15 cm: %s (%.1f cm; 60 Hz %.1f cm)\n", nm,
                    r.steadyCorrMax <= 15.0f ? "met" : "NOT MET", r.steadyCorrMax, dirsOn.steadyCorrMax);
    }
    // ---- B2 strafe S4 jog while aiming (CET bodyturn.aimJog default ON, bodyturn.aimJogSpeed default 3.3 in 3.3..3.85; native
    // default jogF 0 = the S1 walk gait above, until CET sends jogF). The test capsule follows the test's copy of reds UTPPUpdateAimSpeed (MaxSpeed
    // multiplier = AimJogTarget / 1.65 while AimWalk + Aim): capsule speeds are model values, not game measurements.
    {
        SetStrafe(true);
        const std::string digest0 = StrafeDataDigest();
        const std::string st0 = Status();
        const float t0 = AimJogTarget(0.0f, 1.0f);
        Check("B2 strafe S4: jog off by default (jogF 0): AimJogTarget 0, no aimJogT status segment",
              t0 == 0.0f && st0.find(" aimJogT=") == std::string::npos, StatusFrom(" strafeCfg="));
        const bool bad = !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 1.65f) && !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 3.0f) &&
                         !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 3.29f) && !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 6.1f) &&
                         !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, std::nanf("")) && !SetStrafeParams(false, 10.0f, 30.0f, 0.15f, -1.0f);
        Check("B2 strafe S4: UTPPSetMMStrafeParams rejects jogF NaN / < 0 / outside 3.3..6 (1.65, 3.0, 3.29: run loops below the 0.6 rate "
              "floor) (previous kept)",
              bad && StrafeDataDigest() == digest0 && Status().find(" aimJogT=") == std::string::npos);
        // 8 directions with an 8 s cruise: the v6 motion Run_Start_* clips carry ~2 s of run cycle at rate 0.6-0.7, so the 2 s cruise
        // of `dirs` never leaves them; the steady second of a long cruise shows the facing-fixed run loops
        std::vector<A> dirsLong;
        for (int k = 0; k < 8; ++k)
        {
            const float a = 45.0f * float(k);
            const std::string n = "jl" + std::to_string(int(a));
            dirsLong.push_back(A{.name = n + "_idle", .seconds = 0.5f, .angle = a, .stick = 0.0f});
            dirsLong.push_back(A{.name = n + "_start", .seconds = 0.6f, .angle = a});
            dirsLong.push_back(A{.name = n + "_cruise", .seconds = 8.0f, .angle = a, .cruise = true});
            dirsLong.push_back(A{.name = n + "_stop", .seconds = 0.8f, .angle = a, .stick = 0.0f});
        }
        AimMetrics jogAll, jogRev, jogSprint, jogCrouch;
        for (const float F : {3.85f, 3.3f})
        {
            char nm[32];
            std::snprintf(nm, sizeof nm, "jog %.2f", static_cast<double>(F));
            Check(std::string("B2 strafe S4: SetStrafeParams jogF ") + nm + " accepted", SetStrafeParams(false, 10.0f, 30.0f, 0.15f, F));
            g_jogF = F;
            const AimMetrics d = RunAim(dirs, 1.0f / 60.0f);
            std::printf("[INFO] B2 strafe S4 %s [8 directions] 60 Hz: %s\n", nm, d.Line().c_str());
            std::printf("[INFO] B2 strafe S4 %s [8 directions] 60 Hz: %s\n", nm, d.JogLine().c_str());
            const AimMetrics dl = RunAim(dirsLong, 1.0f / 60.0f);
            std::printf("[INFO] B2 strafe S4 %s [8 directions, 8 s cruise] 60 Hz: %s\n", nm, dl.Line().c_str());
            std::printf("[INFO] B2 strafe S4 %s [8 directions, 8 s cruise] 60 Hz: %s\n", nm, dl.JogLine().c_str());
            if (F == 3.85f)
            {
                jogRev = RunAim(rev, 1.0f / 60.0f);
                jogSprint = RunAim(sprintAim, 1.0f / 60.0f);
                jogCrouch = RunAim(crouch, 1.0f / 60.0f);
                const std::pair<const char*, const AimMetrics*> extra[] = {
                    {"reversals", &jogRev}, {"aim from sprint / release", &jogSprint}, {"crouch aim", &jogCrouch}};
                for (const auto& e : extra)
                {
                    std::printf("[INFO] B2 strafe S4 jog 3.85 [%s] 60 Hz: %s\n", e.first, e.second->Line().c_str());
                    std::printf("[INFO] B2 strafe S4 jog 3.85 [%s] 60 Hz: %s\n", e.first, e.second->JogLine().c_str());
                }
                const std::string sj = Status();
                Check("B2 strafe S4 jog 3.85: status aimJogT segment + strafeCfg jogF 3.85",
                      sj.find(" aimJogT=") != std::string::npos && Field(sj, "aimJogQ") > 100.0f &&
                          sj.find("/3.85 strafeSet=") != std::string::npos,
                      StatusFrom(" aimJogT="));
            }
            jogAll.Add(d);
            jogAll.Add(dl);
            // per-F checks
            const std::string tag = std::string("B2 strafe S4 [v6] ") + nm + " 60 Hz 8 directions: ";
            Check(tag + "AimJogTarget = literal F x 1 / 0.70 / 0.60 (forward / side / back) on every Active stand aim frame (1e-4 m/s), 0 "
                  "otherwise",
                  d.jogOn > 500 && d.jogLit == d.jogOn && d.jogMissing == 0 && d.jogWrongOn == 0 && d.jogErrMax <= 1e-4f &&
                      dl.jogLit == dl.jogOn && dl.jogMissing == 0 && dl.jogWrongOn == 0 && dl.jogErrMax <= 1e-4f,
                  d.JogLine());
            Check(tag + "reds-model capsule (1.65 x clamp(target / 1.65, 0.5, 2.5), hysteresis 0.02) reaches the literal speeds within 3 % "
                  "(the multiplier clamp does not cut the profile)",
                  d.capN > 200 && d.capErrMax <= 0.03f && dl.capN > 200 && dl.capErrMax <= 0.03f, d.JogLine());
            Check(tag + "0 turn picks, root direction agreement >= 90 %, playback rate within [0.6, 1.4]",
                  d.turnPicks == 0 && d.Agree() >= 0.9f && d.rateOut == 0 && d.moveFrames > 500 && dl.turnPicks == 0 && dl.Agree() >= 0.9f &&
                      dl.rateOut == 0,
                  d.Line() + " | 8 s: " + dl.Line());
            Check(tag + "steady rate = capsule / own loop speed or F x profile / S_run(theta) within 5 % (rate floor not reached)",
                  d.rateErrN > 100 && d.rateErrMax <= 0.05f && dl.rateErrN > 100 && dl.rateErrMax <= 0.05f && d.rateMin >= 0.6f &&
                      dl.rateMin >= 0.6f,
                  d.Line() + " | 8 s: " + dl.Line());
            Check(tag + "2 s cruise plays run-gait clips (the Neutral Run_Start_* cycle, no walk-gait clip)",
                  d.steadyN > 200 && d.steadyRunLoop == d.steadyN && d.steadyWalkLoop == 0, d.JogLine());
            Check(tag + "8 s cruise: the steady second plays facing-fixed run strafe loops (>= 80 % of the steady frames, no walk-gait clip)",
                  dl.steadyN > 200 && dl.steadyLoop >= int(0.8f * float(dl.steadyN)) && dl.steadyWalkLoop == 0 && dl.steadyRunLoop == dl.steadyN,
                  dl.JogLine());
            Check(tag + "steady foot-lock correction max <= 15 cm, direction agreement >= 90 % (2 s and 8 s cruise)",
                  d.steadyCorrMax <= 15.0f && d.steadyN > 200 && d.steadyAgree >= 0.9f * float(d.steadyN) && dl.steadyCorrMax <= 15.0f &&
                      dl.steadyAgree >= 0.9f * float(dl.steadyN),
                  d.Line() + " | 8 s: " + dl.Line());
        }
        g_jogF = 3.85f;
        SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 3.85f);
        Check("B2 strafe S4 jog 3.85: reversals - 0 turn picks, target = literal, rate within [0.6, 1.4]",
              jogRev.turnPicks == 0 && jogRev.jogMissing == 0 && jogRev.jogWrongOn == 0 && jogRev.jogLit == jogRev.jogOn &&
                  jogRev.jogErrMax <= 1e-4f && jogRev.rateOut == 0 && jogRev.jogOn > 300,
              jogRev.Line() + " | " + jogRev.JogLine());
        Check("B2 strafe S4 jog 3.85: aim released -> AimJogTarget 0 on the first non-aim feed (multiplier removed); sprint / run "
              "without aim never jog",
              jogSprint.releaseJogOn == 0 && jogSprint.jogWrongOn == 0 && jogSprint.jogOn > 100, jogSprint.JogLine());
        Check("B2 strafe S4 jog 3.85: crouch aim keeps the walk gait (AimJogTarget 0, crouch strafe engaged)",
              jogCrouch.jogOn == 0 && jogCrouch.strafeFrames > 200 && jogCrouch.jogWrongOn == 0, jogCrouch.JogLine());
        for (const float hz : {30.0f, 144.0f})
        {
            const AimMetrics r = RunAim(dirs, 1.0f / hz);
            char nm[64];
            std::snprintf(nm, sizeof nm, "%.0f Hz", static_cast<double>(hz));
            std::printf("[INFO] B2 strafe S4 jog 3.85 [8 directions] %s: %s\n", nm, r.Line().c_str());
            std::printf("[INFO] B2 strafe S4 jog 3.85 [8 directions] %s: %s\n", nm, r.JogLine().c_str());
            Check(std::string("B2 strafe S4 [v6] jog 3.85 ") + nm + ": 8 directions - target = literal, reds-model capsule within 3 %, 0 "
                  "turn picks, agreement >= 90 %, rate within [0.6, 1.4], steady foot-lock <= 15 cm",
                  r.jogMissing == 0 && r.jogWrongOn == 0 && r.jogLit == r.jogOn && r.jogErrMax <= 1e-4f && r.capErrMax <= 0.03f &&
                      r.capN > 50 && r.turnPicks == 0 && r.Agree() >= 0.9f && r.rateOut == 0 && r.steadyCorrMax <= 15.0f &&
                      r.moveFrames > 100,
                  r.Line() + " | " + r.JogLine());
        }
        // strafe switch off -> no jog even with jogF set
        SetStrafe(false);
        const float tOff = AimJogTarget(0.0f, 1.0f);
        SetStrafe(true);
        // back to the walk gait: the strafe class set / tables return to the jog-off state
        g_jogF = 0.0f;
        const bool back = SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 0.0f);
        Check("B2 strafe S4: strafe off -> AimJogTarget 0; jogF 0 again -> strafe data digest identical to before the jog runs, "
              "aimJogT segment gone",
              tOff == 0.0f && back && StrafeDataDigest() == digest0 && Status().find(" aimJogT=") == std::string::npos);
        std::printf("[INFO] B2 strafe S4 jog totals (8 directions, 2 s + 8 s cruise, x 2 speeds): %s | %s\n", jogAll.Line().c_str(),
                    jogAll.JogLine().c_str());
    }
    SetStrafe(false);
    std::printf("[INFO] B2 strafe OFF control (8 directions + reversals + idle pans, all aim frames): %s\n", offAll.Line().c_str());
    Check("B2 strafe negative control (strafe off = 0.6.5): the aim feeds show the reported problem (root-turning picks > 0 or "
          "direction agreement < 60 %)",
          offAll.turnPicks > 0 || offAll.Agree() < 0.6f, offAll.Line());
    Check("B2 strafe off: never engages (0 strafe frames), the 0.6.5 path", offAll.strafeFrames == 0, std::to_string(offAll.strafeFrames));
    SetUpperMask(false);
    SetHoldPose(false);
    SetStrafePatterns(true);
    g_db = nullptr;
}

// ================================================================ AIR HOLD stage (2026-10-13)
// Jumps / drops / landings through the real MMDriver (hold mode = mm.air auto in this build), v6 and v7b. Game model: ground
// capsule as RunAim (non-aim) / aim walk; in the air the planar velocity is kept (air control turns it towards a changed stick at
// 90 deg/s), vz from take-off with gUp 19.2 / gDown 16 m/s^2, the body yaw frozen; PSM detail 18 rising / 14 falling (jump), 14
// (drop), 23 for 0.15 s at the touchdown (locoState 0 in the air and while landing); UTPPFeedAir fields every feed (onGround,
// raycast distance to the floor while UTPPMMAirWantsProbe, previous-frame PSM.Fall).
namespace airt
{
enum Kind
{
    kJump,
    kDrop,
    kBlip,
    kClimb,
    kDodge,
    kSlide,
    kRelease,
    kKnock // hold repair: jump detail 18 for aSc.blip s while on the ground (car hit / knockdown; 1 feed = a grounded jump edge)
};
struct AirSc
{
    std::string name;
    Kind kind = kJump;
    float speed = 3.85f;     // planar target speed on the ground (m/s)
    int loco = 1;            // 1 stand, 3 sprint
    float stick = 1.0f;      // |stick| (0 = standing)
    float pre = 1.6f;        // s on the ground before the event
    float vz0 = 6.2f;        // jump take-off vz (m/s)
    float dropH = 0.0f;      // drop height (m)
    float stickTurn = 0.0f;  // deg stick change at the apex (mid-air)
    float airVelTurnRate = 90.0f; // deg/s: velocity responds more slowly than an instant stick change
    float blip = 0.0f;       // s detail-14 blip
    float post = 1.0f;       // s after the touchdown
    bool aim = false;        // aim walk (strafe) before / after, aim held in the air
    bool tppOff = false;     // hold repair: FPP (TPP off = an external release reason) for the whole run
    bool steer = false;      // AIR STEER stage: the game applies UTPPMMBodyYawCommand in the air too (CET executor; mm.airSteer)
    bool steerOffMid = false; // AIR STEER repair: SetAirSteer(false) in the air (after the apex, once >= 10 deg turned)
};
struct AirRes
{
    int airFrames = 0, ownedAir = 0, notOwnedAfterStart = 0, searchesOwned = 0, transOwned = 0, turnPicksOwned = 0, locksAir = 0;
    int btOwned = 0, searchesAir = 0, turnPicksAir = 0, transAir = 0, locksAirAll = 0;
    float firstOwnedAfter = -1.0f; // s after leaving the ground
    float wMinAir = 1.0f;
    int rateN = 0, rateOk = 0;
    float rateRatioMin = 9.0f, rateRatioMax = 0.0f;
    float corrAfter = 0.0f;
    double toeAfter = 0.0;
    float corrPeakAt = -1.0f, toePeakAt = -1.0f;
    float corrAtToePeak = 0.0f;
    float contactAtToePeak = 0.0f, lockAtToePeak = 0.0f;
    float speedAtToePeak = 0.0f, facingAtToePeak = 0.0f;
    int picksAtToePeak = 0;
    float toePublishedPeak = 0.0f;
    int stopPicksAfter = 0, picksAfter = 0;
    int replantWindowFeeds = 0, replantWindowOn = 0, replantAfterFeeds = 0, replantAfterOn = 0;
    int replantBeforeOn = 0, replantAirOn = 0, replantStatusMissing = 0;
    int episodes = 0, takeoffs = 0, falls = 0, handbacks = 0, resets = 0, mmair = 0;
    bool strafeBefore = false;
    float strafeOffLatency = -1.0f, strafeBackLatency = -1.0f;
    int strafeOwned = 0, ownedAfterRelease = 0;
    float wAfterRelease = 1.0f;
    std::vector<uint64_t> hashes;
    std::string picks;
    int addedAfter = 0; // non-motion clips picked within 0.6 s after the touchdown
    // hold repair (kKnock): owned feeds on the ground, their window (s after the event start), searches while owned on the
    // ground, strafe off / back (s after the event start)
    int ownedGround = 0, searchesOwnedGround = 0, ownedGroundRuns = 0;
    int probeWants = 0; // UTPPMMAirWantsProbe true while airborne
    float ownedGroundFirst = -1.0f, ownedGroundLast = -1.0f, strafeOffGround = -1.0f, strafeBackGround = -1.0f;
    // AIR CLIPS stage (filled while g_clipMetrics): forced air clips (UTPP MMAIRCLIP: action, clip, entry, ttl, s after the event
    // start), touchdown time, matcher Hips z (database local, above the capsule) on owned airborne feeds, clip at the end
    struct ClipEv
    {
        std::string action, clip;
        float t = 0.0f, ttl = -1.0f;
        int entry = -1, touchF = -1; // AIR CLIPS repair: clip-local entry frame, clip-local touch frame (MMAIRCLIP touch=)
    };
    std::vector<ClipEv> clipEvs;
    float touchAt = -1.0f;
    // AIR CLIPS repair: clip / clip-local frame right after the touchdown feed; Jump_Loop_Fall playback while owned in the air
    // (feeds, wraps = clip frame went back, longest run of feeds on one clip frame); airQuery overrides during the run
    std::string touchClip;
    int touchClipFrame = -1;
    int fallLoopFeeds = 0, fallLoopWraps = 0, fallLoopStreak = 0, fallLoopNumFrames = -1;
    int queryOverrides = 0;
    int landRealigns = 0; // driver-only overshot-land -> touch-frame alignment count
    int landBlends = 0;
    float landBlendFirstHipsErr = 0.0f;
    float hipsMin = 9.0f, hipsMax = -9.0f;
    std::string endClip;
    std::string seq; // every clip change from the event start to the end (status clip=)
    // AIR STEER stage: body yaw in the air (largest |rate| deg/s, total turn deg, |target - body| deg at the touchdown and at
    // the end), steer commands while airborne
    float bodyRateMaxAir = 0.0f, bodyTurnAir = 0.0f, errTouch = -1.0f, errEnd = -1.0f;
    float bodyVelGapMaxAir = 0.0f, bodyVelGapMaxAfterApex = 0.0f;
    int steerFedAir = 0;
    // AIR STEER repair: mid-air off toggle done, body turn after it (deg), airSteerBlocked delta (armed feeds without the hold)
    bool steerOffDone = false;
    float bodyTurnAfterOff = 0.0f;
    int steerBlocked = 0;
};
int g_mmairLines = 0;
bool g_clipMetrics = false;  // AIR CLIPS stage
float g_curEvT = -1.0f;      // RunAir: s since the event start of the feed being processed
std::vector<AirRes::ClipEv> g_clipEvs;
int g_fallLoopNumFrames = -1; // AIR CLIPS repair: numFrames of the last forced fall loop clip
std::vector<std::string> g_mmairSample;
// Temporary Ground/Feet peak trace. Enabled only for v7c adverse steering at 60 Hz, including first lock and handback.
bool g_airFootPeakTrace = false;
float g_airFootPeakTouchTime = -1.0f;
void AirLog(const char* aLine)
{
    if (g_airFootPeakTrace && g_airFootPeakTouchTime >= 0.10f && g_airFootPeakTouchTime <= 0.44f &&
        (std::strstr(aLine, "UTPP MMTRACE footstep ") || std::strstr(aLine, "UTPP MMTRACE TRANS ") ||
         std::strstr(aLine, "UTPP MMTRACE feet ")))
    {
        std::printf("[PEAKFOOT-EVENT] touch+%.6f %s\n", g_airFootPeakTouchTime, aLine);
    }
    if (g_clipMetrics && std::strstr(aLine, "UTPP MMAIRCLIP "))
    {
        const std::string l = aLine;
        AirRes::ClipEv e;
        auto tok = [&](const char* k) {
            const size_t q = l.find(k);
            if (q == std::string::npos)
                return std::string();
            const size_t a = q + std::strlen(k), b = l.find(' ', a);
            return l.substr(a, b == std::string::npos ? std::string::npos : b - a);
        };
        e.action = tok(" action=");
        e.clip = tok(" clip=");
        e.ttl = float(std::atof(tok(" ttl=").c_str()));
        e.entry = std::atoi(tok(" entry=").c_str()); // "entry/numFrames"
        {
            const std::string en = tok(" entry=");
            const size_t sl = en.find('/');
            if (e.action == "fallLoop" && sl != std::string::npos)
                g_fallLoopNumFrames = std::atoi(en.c_str() + sl + 1);
        }
        const std::string tf = tok(" touch=");
        e.touchF = tf.empty() ? -1 : std::atoi(tf.c_str());
        e.t = g_curEvT;
        g_clipEvs.push_back(e);
    }
    if (std::strstr(aLine, "UTPP MMAIR "))
    {
        ++g_mmairLines;
        if (g_mmairSample.size() < 8)
            g_mmairSample.push_back(aLine);
    }
    TraceLog(aLine);
}
uint64_t PubHash()
{
    using namespace utpp::mm::integ;
    static PoseSnapshot snap{};
    if (!utpp::g_mmPose.Read(snap))
        return 0;
    uint64_t h = 1469598103934665603ull;
    const auto mix = [&](const void* p, size_t n)
    {
        const auto* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i)
        {
            h ^= b[i];
            h *= 1099511628211ull;
        }
    };
    mix(&snap.weight, sizeof snap.weight);
    mix(snap.pose, sizeof(float) * 7 * std::min<uint32_t>(snap.numBones, kMaxMMBones));
    return h;
}
bool TurnishName(const std::string& aName)
{
    using namespace utpp::mm::steer;
    const uint8_t k = DriverClipClass(aName, ClassifyClip(aName));
    return IsTurnClass(k) || k == kClassTurnInPlace;
}

// aFeedAir false: no UTPPFeedAir fields (an older script; the air area infers the ground from the PSM detail)
AirRes RunAir(const AirSc& aSc, float aHz, bool aFeedAir = true)
{
    using namespace utpp::mm;
    AirRes r;
    const bool peakTrace = g_airFootPeakTrace;
    SetEnabled(true); // fresh matcher state
    const float dt = 1.0f / aHz;
    float cam = 0.0f, body = 0.0f, vx = 0.0f, vy = 0.0f, z = 0.0f, vz = 0.0f, floorZ = 0.0f, angle = 0.0f;
    const uint32_t kAimBits = ((0u + 1u) << 24) | ((6u + 1u) << 13);
    const std::string st0 = Status();
    const float ep0 = Field(st0, "airEpisodes"), to0 = Field(st0, "airTakeoffs"), fa0 = Field(st0, "airFalls");
    const float hb0 = Field(st0, "airHandbacks"), rs0 = Field(st0, "airResets");
    const float sb0 = Field(st0, "airSteerBlocked"); // AIR STEER repair
    const float al0 = Field(st0, "airLandAlign");
    const float ab0 = Field(st0, "airLandBlend");
    const int mmair0 = g_mmairLines;
    float prevSearches = Field(st0, "searches"), prevTrans = Field(st0, "transitions");
    std::string prevClip = ClipOf(st0);
    enum Ph
    {
        Pre,
        Event,
        Post
    } ph = Pre;
    float tPh = 0.0f, evT = 0.0f, sinceTouch = -1.0f, sinceLeave = -1.0f, sinceRelease = -1.0f;
    float sinceEvent = -1.0f; // hold repair (kKnock metrics)
    bool prevOwnedGround = false;
    bool airborne = false, apexDone = false, touched = false, everOwned = false, released = false;
    int detail = -1, prevFall = 0, landLeft = 0;
    float vzPrev = 0.0f;
    bool prevStrafe = false;
    int flPrevFrame = -1, flRun = 0; // AIR CLIPS repair (fall loop metrics)
    const float query0 = Field(st0, "airQuery");
    g_fallLoopNumFrames = -1;
    for (int guard = 0; guard < int(20.0f * aHz); ++guard)
    {
        // ---- game model
        const bool moving = aSc.stick > 0.15f;
        if (ph == Pre && tPh >= aSc.pre)
        {
            ph = Event;
            evT = 0.0f;
            switch (aSc.kind)
            {
            case kJump:
            case kRelease:
                airborne = true;
                vz = aSc.vz0;
                detail = 18;
                break;
            case kDrop:
                airborne = true;
                vz = 0.0f;
                floorZ = z - aSc.dropH;
                detail = 14;
                break;
            case kBlip:
                airborne = true;
                vz = -0.5f;
                floorZ = z - 0.5f * aSc.blip;
                detail = 14;
                break;
            case kClimb:
                airborne = true;
                vz = 2.5f;
                detail = 8;
                break;
            case kDodge:
                airborne = true;
                vz = 1.6f;
                detail = 7;
                break;
            case kSlide:
                detail = 5;
                break;
            case kKnock:
                detail = 18;
                break;
            }
            sinceLeave = airborne ? 0.0f : -1.0f;
            sinceEvent = 0.0f;
        }
        if (ph == Event)
        {
            evT += dt;
            switch (aSc.kind)
            {
            case kJump:
            case kDrop:
            case kRelease:
                if (airborne && !released)
                {
                    vz -= (vz > 0.0f ? 19.2f : 16.0f) * dt;
                    z += vz * dt;
                    if (aSc.kind != kDrop)
                        detail = vz > 0.0f ? 18 : 14;
                    if (!apexDone && vz <= 0.0f)
                    {
                        apexDone = true;
                        angle += aSc.stickTurn;
                        if (aSc.kind == kRelease)
                        {
                            released = true; // ladder grab at the apex: hangs 0.5 s, then stands on the ladder top
                            detail = 10;
                            vz = 0.0f;
                            sinceRelease = 0.0f;
                        }
                    }
                    if (z <= floorZ)
                    {
                        z = floorZ;
                        vz = 0.0f;
                        airborne = false;
                        touched = true;
                        detail = 23;
                        landLeft = int(0.15f * aHz + 0.5f);
                    }
                }
                else if (released)
                {
                    sinceRelease += dt;
                    vx = vy = 0.0f;
                    if (sinceRelease >= 0.5f && airborne)
                    {
                        airborne = false;
                        floorZ = z;
                        detail = -1;
                        ph = Post;
                        tPh = 0.0f;
                    }
                }
                break;
            case kBlip:
                z += vz * dt;
                if (evT >= aSc.blip)
                {
                    airborne = false;
                    z = floorZ;
                    vz = 0.0f;
                    detail = -1;
                    ph = Post;
                    tPh = 0.0f;
                }
                break;
            case kClimb:
                z += vz * dt;
                if (evT >= 0.6f)
                {
                    detail = 9;
                    vz = 1.0f;
                }
                if (evT >= 0.9f)
                {
                    airborne = false;
                    floorZ = z;
                    vz = 0.0f;
                    detail = -1;
                    ph = Post;
                    tPh = 0.0f;
                }
                break;
            case kDodge:
                if (evT >= 0.15f)
                    vz = -1.6f;
                z += vz * dt;
                if (evT >= 0.3f)
                {
                    airborne = false;
                    z = floorZ;
                    vz = 0.0f;
                    detail = -1;
                    ph = Post;
                    tPh = 0.0f;
                }
                break;
            case kSlide:
                if (evT >= 1.0f)
                {
                    detail = -1;
                    ph = Post;
                    tPh = 0.0f;
                }
                break;
            case kKnock:
                if (evT >= aSc.blip + 1.5f * dt) // aSc.blip 0: exactly one feed with detail 18
                {
                    detail = -1;
                    ph = Post;
                    tPh = 0.0f;
                }
                break;
            }
            if (touched && ph == Event)
            {
                ph = Post;
                tPh = 0.0f;
                sinceTouch = 0.0f;
                g_fp.Clear();
            }
        }
        if (ph == Post && landLeft == 0 && detail == 23)
            detail = -1;
        // planar capsule
        const float dirDeg = aSc.aim ? body + angle : cam + angle;
        if (airborne && !released)
        {
            // air control: the planar velocity turns towards a changed stick at 90 deg/s, speed kept
            const float sp = std::sqrt(vx * vx + vy * vy);
            if (sp > 0.1f && moving)
            {
                const float cur = std::atan2(-vx, vy) / kDegT;
                const float e = std::clamp(WrapDeg(dirDeg - cur), -aSc.airVelTurnRate * dt, aSc.airVelTurnRate * dt);
                const float nd = (cur + e) * kDegT;
                vx = -std::sin(nd) * sp;
                vy = std::cos(nd) * sp;
            }
            if (aSc.steer && !aSc.aim)
            {
                // AIR STEER stage: the executor commands every frame while UTPPMMBodyTurnActive
                if (aSc.steerOffMid && !r.steerOffDone && apexDone && r.bodyTurnAir >= 10.0f)
                {
                    SetAirSteer(false); // AIR STEER repair: the setting switched off in the air
                    r.steerOffDone = true;
                }
                const float b0 = body;
                if (BodyTurnActive())
                    body = WrapDeg(BodyYawCommand(body, body));
                const float step = std::fabs(WrapDeg(body - b0));
                r.bodyRateMaxAir = std::max(r.bodyRateMaxAir, step / dt);
                r.bodyTurnAir += step;
                if (sp >= 0.5f && BodyTurnActive())
                {
                    const float heading = std::atan2(-vx, vy) / kDegT;
                    const float gap = std::fabs(WrapDeg(body - heading));
                    r.bodyVelGapMaxAir = std::max(r.bodyVelGapMaxAir, gap);
                    if (apexDone)
                        r.bodyVelGapMaxAfterApex = std::max(r.bodyVelGapMaxAfterApex, gap);
                }
                if (r.steerOffDone)
                    r.bodyTurnAfterOff += step;
            }
        }
        else if (!released)
        {
            float want = (aSc.aim ? 1.65f : (aSc.kind == kSlide && ph == Event ? 8.0f : aSc.speed)) * std::fmin(aSc.stick, 1.0f);
            if (aSc.kind == kKnock && ph == Event && !aSc.aim)
                want = 0.0f; // knocked down: the capsule brakes to a stop
            const float dx = -std::sin(dirDeg * kDegT), dy = std::cos(dirDeg * kDegT);
            float along = vx * dx + vy * dy;
            float px = vx - along * dx, py = vy - along * dy;
            const float pl = std::sqrt(px * px + py * py);
            const float pk = pl > 1e-6f ? std::fmax(0.0f, pl - 20.0f * dt) / pl : 0.0f;
            px *= pk;
            py *= pk;
            if (!moving)
                along = along > 0.0f ? std::fmax(0.0f, along - 20.0f * dt) : std::fmin(0.0f, along + 20.0f * dt);
            else if (along < 0.0f || along > want)
                along = along < 0.0f ? std::fmin(0.0f, along + 20.0f * dt) : std::fmax(want, along - 20.0f * dt);
            else
                along = std::fmin(want, along + 24.0f * dt);
            vx = px + along * dx;
            vy = py + along * dy;
            // body (ground only; frozen in the air)
            if (aSc.aim)
                body = cam;
            else
            {
                float baseYaw = body;
                if (moving)
                    baseYaw = body + std::clamp(WrapDeg(cam + angle - body), -540.0f * dt, 540.0f * dt);
                body = WrapDeg(BodyYawCommand(body, baseYaw));
            }
        }
        // ---- feed
        FeedInput in;
        in.velWorld[0] = vx;
        in.velWorld[1] = vy;
        in.velWorld[2] = airborne ? vz : 0.0f;
        const float mag = std::fmin(aSc.stick, 1.0f);
        if (aSc.aim)
        {
            in.move[0] = -std::sin(angle * kDegT) * mag;
            in.move[1] = std::cos(angle * kDegT) * mag;
        }
        else
            in.move[1] = mag;
        const bool special = detail >= 0;
        in.locoState = (special || aSc.aim) ? 0 : aSc.loco;
        uint32_t psm = 0;
        if (aSc.aim)
            psm = kAimBits | 0x40u | ((uint32_t(special ? detail : 2) + 1u) << 16);
        else if (special)
            psm = 0x40u | ((uint32_t(detail) + 1u) << 16);
        in.psmBlock = psm;
        in.cameraYawDeg = cam;
        in.playerYawDeg = body;
        in.tppActive = !aSc.tppOff;
        in.dt = dt;
        in.haveTarget = moving;
        in.targetYawDeg = WrapDeg(dirDeg);
        if (aFeedAir)
        {
            in.haveAir = true;
            in.onGround = !airborne;
            in.distToGround = -2.0f;
            const bool wantsProbe = airborne && AirWantsProbe();
            r.probeWants += wantsProbe ? 1 : 0;
            if (wantsProbe)
                in.distToGround = released ? 0.0f : std::fmax(0.0f, z - floorZ);
            in.psmFall = prevFall;
        }
        g_clock += dt;
        if (peakTrace)
            g_airFootPeakTouchTime = sinceTouch;
        g_curEvT = sinceEvent; // AIR CLIPS stage (MMAIRCLIP time stamps)
        if (g_clipMetrics && touched && r.touchAt < 0.0f)
            r.touchAt = sinceEvent;
        Feed(in);
        prevFall = !airborne ? 0 : (vz < -12.0f ? 3 : (vz < -9.0f ? 2 : (vz < 0.0f ? 1 : 0)));
        if (landLeft > 0)
            --landLeft;
        // ---- metrics
        const std::string st = Status();
        r.hashes.push_back(PubHash());
        // Ground/Feet: observe the real FootLocker mode, not a synthetic timer or a test-side policy.
        // The confirmed touchdown feed has contactScale=0; measure after the 0.08 s ramp,
        // not on the feed that physically landed but intentionally still has no lock.
        const bool hasReplantStatus = st.find(" replantActive=") != std::string::npos;
        if (aSc.name == "hold_touch_replant_window" || aSc.name == "ground_only_replant_off")
            r.replantStatusMissing += hasReplantStatus ? 0 : 1;
        const bool replant = hasReplantStatus && Field(st, "replantActive") > 0.5f;
        if (ph == Pre)
            r.replantBeforeOn += replant ? 1 : 0;
        if (airborne)
            r.replantAirOn += replant ? 1 : 0;
        if (sinceTouch >= 0.15f && sinceTouch <= 0.40f)
        {
            ++r.replantWindowFeeds;
            r.replantWindowOn += replant ? 1 : 0;
        }
        if (sinceTouch > 0.55f && sinceTouch <= 0.75f)
        {
            ++r.replantAfterFeeds;
            r.replantAfterOn += replant ? 1 : 0;
        }
        const bool owned = AirOwnsForTest();
        const float searches = Field(st, "searches"), trans = Field(st, "transitions");
        const std::string clip = ClipOf(st);
        const bool picked = clip != prevClip && !clip.empty();
        const bool sFlag = Field(st, "strafe") > 0.5f;
        if (peakTrace && sinceTouch >= 0.10f && sinceTouch <= 0.44f)
        {
            const size_t feet = st.find(" feet=\"");
            const char l = PhaseAfter(st, feet, "L"), rr = PhaseAfter(st, feet, "R");
            // The normal feet status exposes offsets and ankle corrections, but not reach error / clamped flags.
            // Do not report their absence as zeros: err/clamp below are explicitly N/A (no driver changes in this test).
            std::printf("[PEAKFOOT] touch+%.6f airMode=HOLD clip=%s id=%d local=%d picked=%d from=%s searches=%.0f trans=%.0f "
                        "L=%c c=%.2f w=%.2f corr=%.2fcm off=%.2fcm err=N/A clamp=N/A "
                        "R=%c c=%.2f w=%.2f corr=%.2fcm off=%.2fcm err=N/A clamp=N/A "
                        "lock=%d/%d replant=%.0f steps=%.0f rate=%.3f airBlend=%.3f speed=%.3f bodyYaw=%.2f velYaw=%.2f\n",
                        sinceTouch, clip.c_str(), int(Field(st, "clip")), int(Field(st, "clipFrame")), picked ? 1 : 0,
                        prevClip.c_str(), searches, trans, l, FieldAfter(st, feet, "cL"), FieldAfter(st, feet, "wL"),
                        FieldAfter(st, feet, "corrL"), FieldAfter(st, feet, "offL"),
                        rr, FieldAfter(st, feet, "cR"), FieldAfter(st, feet, "wR"), FieldAfter(st, feet, "corrR"),
                        FieldAfter(st, feet, "offR"),
                        int(Field(st, "lockL")), int(Field(st, "lockR")), Field(st, "replantActive"), FieldAfter(st, feet, "stepsL") + FieldAfter(st, feet, "stepsR"),
                        Field(st, "rate"), Field(st, "airLandBlend"), std::hypot(vx, vy), body, std::atan2(-vx, vy) / kDegT);
        }
        if (ph == Pre && aSc.aim)
            r.strafeBefore = sFlag;
        if (g_clipMetrics) // AIR CLIPS stage
        {
            if (sinceEvent >= 0.0f && picked)
            {
                char tb[32];
                std::snprintf(tb, sizeof tb, "@%.2f", sinceEvent);
                r.seq += " " + clip + tb;
            }
            if (airborne && !released && owned)
            {
                static std::vector<float> mp;
                if (MatcherPoseForTest(mp) && mp.size() >= 3 * 7)
                {
                    r.hipsMin = std::min(r.hipsMin, mp[2 * 7 + 2]);
                    r.hipsMax = std::max(r.hipsMax, mp[2 * 7 + 2]);
                }
            }
            r.endClip = clip;
            // AIR CLIPS repair: touchdown alignment and fall loop playback
            if (touched && r.touchClipFrame < 0)
            {
                r.touchClip = clip;
                r.touchClipFrame = int(Field(st, "clipFrame"));
            }
            if (airborne && !released && owned && clip.find("Jump_Loop_Fall") != std::string::npos)
            {
                const int cf = int(Field(st, "clipFrame"));
                ++r.fallLoopFeeds;
                if (flPrevFrame >= 0 && cf < flPrevFrame)
                    ++r.fallLoopWraps;
                flRun = (flPrevFrame >= 0 && cf == flPrevFrame) ? flRun + 1 : 1;
                r.fallLoopStreak = std::max(r.fallLoopStreak, flRun);
                flPrevFrame = cf;
            }
            else
                flPrevFrame = -1;
    r.queryOverrides = int(Field(st, "airQuery") - query0);
            r.fallLoopNumFrames = g_fallLoopNumFrames;
        }
        if (aSc.steer) // AIR STEER stage
        {
            const float err = std::fabs(WrapDeg(dirDeg - body));
            if (touched && r.errTouch < 0.0f)
                r.errTouch = err;
            r.errEnd = err;
            if (airborne && !released && Field(st, "airSteerFed") > 0.5f)
                ++r.steerFedAir;
        }
        if (airborne && !released)
        {
            ++r.airFrames;
            r.searchesAir += int(searches - prevSearches);
            r.transAir += int(trans - prevTrans);
            if (picked && TurnishName(clip))
                ++r.turnPicksAir;
            if (Field(st, "lockL") > 0.5f || Field(st, "lockR") > 0.5f)
                ++r.locksAirAll; // any airborne frame (the mm.air=off comparison)
            if (owned)
            {
                if (!everOwned)
                    r.firstOwnedAfter = sinceLeave;
                everOwned = true;
                ++r.ownedAir;
                r.searchesOwned += int(searches - prevSearches);
                r.transOwned += int(trans - prevTrans);
                if (picked)
                {
                    r.picks += " " + clip;
                    if (TurnishName(clip))
                        ++r.turnPicksOwned;
                }
                if (Field(st, "lockL") > 0.5f || Field(st, "lockR") > 0.5f)
                    ++r.locksAir;
                if (Field(st, "btFed") > 0.5f)
                    ++r.btOwned;
                if (sFlag)
                    ++r.strafeOwned;
                r.wMinAir = std::min(r.wMinAir, Field(st, "weight"));
                const float lr = Field(st, "loopRate");
                if (lr > 0.05f && Field(st, "turnRateActive") < 0.5f)
                {
                    const float k = Field(st, "rate") / lr;
                    ++r.rateN;
                    r.rateOk += std::fabs(k - 0.5f) <= 0.03f ? 1 : 0;
                    r.rateRatioMin = std::min(r.rateRatioMin, k);
                    r.rateRatioMax = std::max(r.rateRatioMax, k);
                }
            }
            else if (everOwned)
                ++r.notOwnedAfterStart;
            if (aSc.aim && r.strafeOffLatency < 0.0f && !sFlag && prevStrafe)
                r.strafeOffLatency = sinceLeave;
        }
        if (aSc.kind == kKnock && sinceEvent >= 0.0f)
        {
            if (owned && !airborne)
            {
                ++r.ownedGround;
                r.searchesOwnedGround += int(searches - prevSearches);
                if (r.ownedGroundFirst < 0.0f)
                    r.ownedGroundFirst = sinceEvent;
                r.ownedGroundLast = sinceEvent;
                if (!prevOwnedGround)
                    ++r.ownedGroundRuns;
            }
            if (aSc.aim && r.strafeOffGround < 0.0f && prevStrafe && !sFlag)
                r.strafeOffGround = sinceEvent;
            if (aSc.aim && r.strafeOffGround >= 0.0f && r.strafeBackGround < 0.0f && sFlag)
                r.strafeBackGround = sinceEvent;
            prevOwnedGround = owned && !airborne;
        }
        if (sinceEvent >= 0.0f)
            sinceEvent += dt;
        if (released)
        {
            r.ownedAfterRelease += owned ? 1 : 0;
            r.wAfterRelease = Field(st, "weight");
        }
        if (sinceTouch >= 0.0f)
        {
            if (sinceTouch <= 0.6f)
            {
                const float corr = std::max(Field(st, "corrL"), Field(st, "corrR"));
                if (corr > r.corrAfter)
                {
                    r.corrAfter = corr;
                    r.corrPeakAt = sinceTouch;
                }
                C1bProbe(in, st);
                // The locked-toe metric is a discrete speed after publication, not necessarily IK sliding.
                // A true slip additionally needs sustained nonzero motion while the lock remains planted;
                // preserve this separate raw peak for diagnosis rather than weakening HandBackOk.
                r.toePublishedPeak = std::max(r.toePublishedPeak, float(g_fp.lockedMax));
                if (g_fp.lockedMax > r.toeAfter)
                {
                    r.toeAfter = g_fp.lockedMax;
                    r.toePeakAt = sinceTouch;
                    r.corrAtToePeak = corr;
                    r.contactAtToePeak = std::max(Field(st, "cL"), Field(st, "cR"));
                    r.lockAtToePeak = std::max(Field(st, "lockL"), Field(st, "lockR"));
                    r.speedAtToePeak = std::sqrt(vx * vx + vy * vy);
                    r.facingAtToePeak = std::fabs(WrapDeg(body - std::atan2(-vx, vy) / kDegT));
                    r.picksAtToePeak = r.picksAfter;
                }
                if (picked)
                {
                    ++r.picksAfter;
                    if (IsAddedClipName(clip))
                        ++r.addedAfter;
                    if (moving && steer::ClassifyClip(clip) == steer::kClassStop)
                        ++r.stopPicksAfter;
                }
            }
            if (aSc.aim && r.strafeBackLatency < 0.0f && sFlag)
                r.strafeBackLatency = sinceTouch;
            sinceTouch += dt;
        }
        if (sinceLeave >= 0.0f)
            sinceLeave += dt;
        prevStrafe = sFlag;
        prevSearches = searches;
        prevTrans = trans;
        prevClip = clip;
        vzPrev = vz;
        tPh += dt;
        if (ph == Post && tPh >= aSc.post)
            break;
    }
    (void)vzPrev;
    r.toeAfter = g_fp.lockedMax;
    const std::string st1 = Status();
    r.episodes = int(Field(st1, "airEpisodes") - ep0);
    r.takeoffs = int(Field(st1, "airTakeoffs") - to0);
    r.falls = int(Field(st1, "airFalls") - fa0);
    r.handbacks = int(Field(st1, "airHandbacks") - hb0);
    r.steerBlocked = int(Field(st1, "airSteerBlocked") - sb0); // AIR STEER repair
    r.landRealigns = int(Field(st1, "airLandAlign") - al0);
    r.landBlends = int(Field(st1, "airLandBlend") - ab0);
    r.landBlendFirstHipsErr = Field(st1, "airBlendHipsErr");
    r.resets = int(Field(st1, "airResets") - rs0);
    r.mmair = g_mmairLines - mmair0;
    return r;
}
std::string Line(const AirRes& r)
{
    char b[640];
    std::snprintf(b, sizeof b,
                  "air %d owned %d (first after %.3f s, gaps %d) searches %d/%d trans %d/%d turnPicks %d/%d (owned/air) locks %d bt %d "
                  "wMin %.2f rate x%.2f..%.2f (%d/%d at 0.5) | after touchdown: corr %.1f cm toe %.2f m/s picks %d stop %d | ep %d to %d "
                  "falls %d hb %d resets %d mmair %d",
                  r.airFrames, r.ownedAir, r.firstOwnedAfter, r.notOwnedAfterStart, r.searchesOwned, r.searchesAir, r.transOwned, r.transAir,
                  r.turnPicksOwned, r.turnPicksAir, r.locksAir, r.btOwned, r.wMinAir, r.rateN ? r.rateRatioMin : 0.0f,
                  r.rateN ? r.rateRatioMax : 0.0f, r.rateOk, r.rateN, r.corrAfter, r.toeAfter, r.picksAfter, r.stopPicksAfter, r.episodes,
                  r.takeoffs, r.falls, r.handbacks, r.resets, r.mmair);
    return b;
}
// the air hold criteria on airborne frames (jumps / drops)
bool HoldAirOk(const AirRes& r, float aMaxFirst)
{
    return r.airFrames > 0 && r.ownedAir > 0 && r.firstOwnedAfter >= 0.0f && r.firstOwnedAfter <= aMaxFirst && r.notOwnedAfterStart == 0 &&
           r.searchesOwned == 0 && r.transOwned == 0 && r.turnPicksOwned == 0 && r.locksAir == 0 && r.btOwned == 0 && r.wMinAir >= 0.9f;
}
bool HandBackOk(const AirRes& r)
{
    return r.handbacks == 1 && r.resets == 0 && r.corrAfter <= 15.0f && r.toeAfter <= kC1bToeCap && r.stopPicksAfter == 0;
}
} // namespace airt

// ================================================================ CLIMB RELEASE stage (2026-10-13)
// PSM Climb 8 / Vault 9 through the real MMDriver (hold pose on, mm.air hold): with mm.climbRelease on (default) and v2 not
// owning the traversal (no hooks / the main-branch stub) the body is released to the game animation - travRelease, the MM weight
// fades to 0 with the normal gate fade during 8 / 9 and comes back after; off (or v2 owning: hook returns true) = held as before.
namespace travt
{
enum Kind
{
    kVaultWalk, // walk 1.6 m/s, vault (detail 9) 0.8 s with |vz| 1, walk on
    kJumpClimb  // run 3.85 m/s, jump (detail 18, vz 5.5) 0.25 s, climb (detail 8) 1.2 s with vz +1.2 off the ground, walk on
};
struct TravRes
{
    std::vector<uint64_t> hashes;
    std::vector<float> w;
    int evFirst = -1, evLast = -1; // first / last feed with detail 8 / 9
    float wPre = 0.0f;             // weight on the feed before the first 8 / 9 feed
    float wMinEv = 1.0f;           // min weight over the 8 / 9 feeds
    float zeroAfter = -1.0f;       // s from the first 8 / 9 feed until the weight is 0
    float backAfter = -1.0f;       // s from the last 8 / 9 feed until the weight is >= 0.99 again
    float wEnd = 0.0f;
    int relFeeds = 0, relOutside = 0, evFeeds = 0;
    float eps = 0.0f, owned = 0.0f; // travReleaseEpisodes / travOwnedFeeds deltas
    int gateLines = 0, onLines = 0, offLines = 0;
};
int g_gateTravLines = 0, g_travOnLines = 0, g_travOffLines = 0;
int g_travInfoLines = 0; // TRAV-1 D: always-on 'UTPP trav: released to game animation' lines (counted, never reset by RunTrav)
void TravLog(const char* aLine)
{
    if (std::strstr(aLine, "UTPP MMGATE ") && std::strstr(aLine, " release=0x1 travRelease"))
        ++g_gateTravLines;
    if (std::strstr(aLine, "UTPP MMTRAV travRelease on "))
        ++g_travOnLines;
    if (std::strstr(aLine, "UTPP MMTRAV travRelease off "))
        ++g_travOffLines;
    if (std::strstr(aLine, "UTPP trav: released to game animation (detail="))
        ++g_travInfoLines;
    airt::AirLog(aLine);
}
bool g_ownsReturn = false;
bool OwnsHook() { return g_ownsReturn; }
const MotionHooks kOwnsHooks{nullptr, nullptr, nullptr, nullptr, nullptr, &OwnsHook}; // ownsTraversal only (no v2 pose)
// TRAV-1 (vehicle batch): travActive stub (v2 traversal active, no v2 pose) - releases un-owned 8 / 9 whatever mm.climbRelease says
bool g_travActiveReturn = true;
bool TravActiveHook() { return g_travActiveReturn; }
const MotionHooks kTravActiveHooks{nullptr, nullptr, nullptr, nullptr, nullptr, &OwnsHook, nullptr, &TravActiveHook};
std::string TravLine(const TravRes& r)
{
    char b[260];
    std::snprintf(b, sizeof b, "ev %d..%d wPre %.2f wMinEv %.3f zeroAfter %.3f backAfter %.3f wEnd %.2f rel %d/%d outside %d eps %.0f owned %.0f "
                  "lines gate %d on %d off %d",
                  r.evFirst, r.evLast, r.wPre, r.wMinEv, r.zeroAfter, r.backAfter, r.wEnd, r.relFeeds, r.evFeeds, r.relOutside, r.eps, r.owned,
                  r.gateLines, r.onLines, r.offLines);
    return b;
}
// aStopEv > 0: return right after that many 8 / 9 feeds (mid-episode; the no-feed Tick checks continue from there)
TravRes RunTrav(Kind aKind, float aHz, int aStopEv = -1)
{
    TravRes r;
    const float dt = 1.0f / aHz;
    const std::string st0 = Status();
    const float ep0 = Field(st0, "travReleaseEpisodes"), ow0 = Field(st0, "travOwnedFeeds");
    g_gateTravLines = g_travOnLines = g_travOffLines = 0;
    const float pre = 1.5f;
    const float evLen = aKind == kVaultWalk ? 0.8f : 1.45f;
    const float total = pre + evLen + 2.5f;
    const float walk = aKind == kVaultWalk ? 1.6f : 3.85f;
    float vy = 0.0f, vz = 0.0f;
    const int n = int(std::lround(total * aHz));
    for (int i = 0; i < n; ++i)
    {
        const float t = float(i) * dt;
        int detail = -1;
        bool airborne = false;
        float want = walk;
        if (t >= pre && t < pre + evLen)
        {
            const float e = t - pre;
            if (aKind == kVaultWalk)
            {
                detail = 9;
                airborne = true;
                vz = e < 0.4f ? 1.0f : -1.0f;
                want = 2.0f;
            }
            else if (e < 0.25f)
            {
                detail = 18;
                airborne = true;
                vz = 5.5f - 19.2f * e;
            }
            else
            {
                detail = 8;
                airborne = true;
                vz = 1.2f;
                want = 0.0f;
            }
        }
        else
        {
            vz = 0.0f;
            if (t >= pre + evLen)
                want = 1.6f; // walk on after the event
        }
        if (!airborne || detail != 18)
            vy = vy < want ? std::fmin(want, vy + 24.0f * dt) : std::fmax(want, vy - 20.0f * dt);
        FeedInput in;
        in.velWorld[0] = 0.0f;
        in.velWorld[1] = vy;
        in.velWorld[2] = airborne ? vz : 0.0f;
        in.move[1] = 1.0f;
        in.locoState = detail >= 0 ? 0 : 1;
        in.psmBlock = detail >= 0 ? (0x40u | ((uint32_t(detail) + 1u) << 16)) : 0u;
        in.cameraYawDeg = 0.0f;
        in.playerYawDeg = 0.0f;
        in.tppActive = true;
        in.dt = dt;
        in.haveTarget = true;
        in.targetYawDeg = 0.0f;
        in.haveAir = true;
        in.onGround = !airborne;
        in.distToGround = -2.0f;
        in.psmFall = 0;
        g_clock += dt;
        Feed(in);
        const std::string st = Status();
        const float w = Field(st, "weight");
        const bool rel = Field(st, "travRelease") > 0.5f;
        r.hashes.push_back(airt::PubHash());
        r.w.push_back(w);
        const bool ev = detail == 8 || detail == 9;
        if (ev)
        {
            if (r.evFirst < 0)
            {
                r.evFirst = i;
                r.wPre = i > 0 ? r.w[size_t(i) - 1] : 0.0f;
            }
            r.evLast = i;
            ++r.evFeeds;
            r.wMinEv = std::min(r.wMinEv, w);
            r.relFeeds += rel ? 1 : 0;
            if (r.zeroAfter < 0.0f && w <= 0.0f)
                r.zeroAfter = float(i - r.evFirst) * dt;
        }
        else
        {
            r.relOutside += rel ? 1 : 0;
            if (r.evLast >= 0 && r.backAfter < 0.0f && w >= 0.99f)
                r.backAfter = float(i - r.evLast) * dt;
        }
        r.wEnd = w;
        if (aStopEv > 0 && r.evFeeds >= aStopEv)
            break;
    }
    const std::string st1 = Status();
    r.eps = Field(st1, "travReleaseEpisodes") - ep0;
    r.owned = Field(st1, "travOwnedFeeds") - ow0;
    r.gateLines = g_gateTravLines;
    r.onLines = g_travOnLines;
    r.offLines = g_travOffLines;
    return r;
}
uint64_t FeedCalls() // exact (Field is a float)
{
    const std::string s = Status();
    const size_t p = s.find(" feedCalls=");
    return p == std::string::npos ? 0ull : std::strtoull(s.c_str() + p + 11, nullptr, 10);
}
// review repair: no feeds mid-episode (the script stops feeding while MM is disabled: menu / pause) - Tick only, real time
struct TickRes
{
    float wStart = 0.0f, wEnd = 0.0f, wMax = 0.0f;
    float zeroAfter = -1.0f; // s of ticking until the weight is 0
    float ticks = 0.0f;      // holdPoseTicks delta
    bool active = false;     // holdPoseActive=1 at the end
};
TickRes TickDisabled(float aSeconds)
{
    TickRes r;
    const std::string st0 = Status();
    r.wStart = r.wMax = Field(st0, "weight");
    const float t0 = Field(st0, "holdPoseTicks");
    SetEnabled(false);
    LARGE_INTEGER fq, a, now;
    QueryPerformanceFrequency(&fq);
    QueryPerformanceCounter(&a);
    for (;;)
    {
        Sleep(10);
        Tick();
        QueryPerformanceCounter(&now);
        const float el = float(double(now.QuadPart - a.QuadPart) / double(fq.QuadPart));
        const float w = Field(Status(), "weight");
        r.wMax = std::max(r.wMax, w);
        if (r.zeroAfter < 0.0f && w <= 0.0f)
            r.zeroAfter = el;
        r.wEnd = w;
        if (el >= aSeconds)
            break;
    }
    const std::string st1 = Status();
    r.ticks = Field(st1, "holdPoseTicks") - t0;
    r.active = st1.find(" holdPoseActive=1") != std::string::npos;
    SetEnabled(true);
    return r;
}
std::string TickLine(const TickRes& r)
{
    char b[160];
    std::snprintf(b, sizeof b, "w %.2f -> %.2f (max %.2f) zeroAfter %.3f holdTicks %.0f active %d", r.wStart, r.wEnd, r.wMax, r.zeroAfter,
                  r.ticks, r.active ? 1 : 0);
    return b;
}
} // namespace travt

void SectionAir(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionAir");
    (void)argc;
    using namespace airt;
    std::vector<std::string> paths;
    {
        std::string dir = argv[1];
        const size_t sl = dir.find_last_of("\\/");
        dir = sl == std::string::npos ? std::string() : dir.substr(0, sl + 1);
        // the game runs the motion-only utpp_base (the package database)
        for (const char* n : {"utpp_base.utppmm"})
            if (GetFileAttributesA((dir + n).c_str()) != INVALID_FILE_ATTRIBUTES)
                paths.push_back(dir + n);
    }
    Check("air hold: database utpp_base present", paths.size() == 1, std::to_string(paths.size()));
    // toes for the post-landing toe speed (C1bProbe; same rig in every database)
    g_fp.toe[0] = g_fp.toe[1] = -1;
    g_fp.parents.assign(aParents.begin(), aParents.end());
    for (size_t b = 0; b < aHashes.size(); ++b)
    {
        if (aHashes[b] == utpp::mm::integ::CNameHash("LeftToeBase"))
            g_fp.toe[0] = int32_t(b);
        if (aHashes[b] == utpp::mm::integ::CNameHash("RightToeBase"))
            g_fp.toe[1] = int32_t(b);
    }
    SetLogger(&AirLog);
    const AirSc runJump{.name = "jump_run"};
    const AirSc standJump{.name = "jump_stand", .speed = 0.0f, .stick = 0.0f, .vz0 = 5.5f};
    const AirSc sprintJump{.name = "jump_sprint", .speed = 8.2f, .loco = 3};
    const AirSc stickJump{.name = "jump_stick_change", .stickTurn = 90.0f};
    const AirSc drop2{.name = "drop_2m", .kind = kDrop, .speed = 3.85f, .dropH = 2.0f};
    const AirSc drop6{.name = "drop_6m", .kind = kDrop, .speed = 3.85f, .dropH = 6.0f};
    const AirSc climb{.name = "climb_vault", .kind = kClimb, .speed = 1.6f};
    const AirSc dodge{.name = "dodge", .kind = kDodge};
    const AirSc slide{.name = "slide", .kind = kSlide, .speed = 8.2f, .loco = 3};
    const AirSc release{.name = "release_ladder_midair", .kind = kRelease};
    const AirSc aimJump{.name = "aim_strafe_jump", .speed = 1.65f, .pre = 2.0f, .vz0 = 5.5f, .post = 1.5f, .aim = true};
    // hold repair: grounded jump-detail edges (air feed onGround true)
    const AirSc knock{.name = "knockdown_detail18_ground", .kind = kKnock, .speed = 8.2f, .loco = 3, .pre = 2.5f, .blip = 3.0f, .post = 1.5f};
    const AirSc aimEdge{.name = "aim_strafe_jumpedge_ground", .kind = kKnock, .speed = 1.65f, .pre = 2.0f, .blip = 0.0f, .post = 1.5f, .aim = true};
    int negSearches = 0, negTurns = 0, negLocks = 0;
    for (const std::string& path : paths)
    {
        std::string name = path.substr(path.find_last_of("\\/") + 1);
        name = name.substr(0, name.rfind('.'));
        const std::wstring wp(path.begin(), path.end());
        if (!LoadDatabase(wp, name))
        {
            Check("air hold: database loaded " + name, false, path);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a();
        SetFeetDiag(true);
        SetUpperMaskStates(7);
        SetStrafe(true);
        SetAir(2); // AIR CLIPS stage: the Data databases have .airz sidecars now -> the hold checks run with mm.air=hold
        {
            const std::string st = Status();
            const size_t p = st.find(" air=");
            std::printf("[INFO] air hold [%s] status:%s\n", name.c_str(), p == std::string::npos ? " (none)" : st.substr(p, 330).c_str());
            Check("air hold [" + name + "]: status air segment (mm.air=hold, table loaded)", p != std::string::npos &&
                  st.find(" air=hold airReason=hold:table") != std::string::npos);
        }
        // Ground/Feet regression: enable only the CROUCH replant mask (2). Stand/Sprint still require the
        // physical AIR Hold touchdown override. Mask 2 also exposes replantActive in the status for all feeds
        // (mask 0 hides this key entirely), so the test can observe effective FootLocker state.
        {
            const int savedReplant = FootReplantMask();
            SetFootReplant(2);
            const AirSc timed{.name = "hold_touch_replant_window", .post = 0.85f};
            bool timedOk = true, groundOk = true, offOk = true;
            std::string failures;
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                SetAir(2);
                const AirRes landed = RunAir(timed, hz);
                const bool on = landed.handbacks == 1 && landed.episodes == 1 && landed.replantStatusMissing == 0 &&
                                landed.replantBeforeOn == 0 &&
                                landed.replantAirOn == 0 && landed.replantWindowFeeds > 0 &&
                                landed.replantWindowOn == landed.replantWindowFeeds && landed.replantAfterFeeds > 0 &&
                                landed.replantAfterOn == 0;
                timedOk &= on;
                const AirSc onlyGround{.name = "ground_only_replant_off", .kind = kSlide, .post = 0.85f};
                const AirRes ground = RunAir(onlyGround, hz);
                groundOk &= ground.episodes == 0 && ground.replantStatusMissing == 0 && ground.replantBeforeOn == 0 &&
                            ground.replantWindowOn == 0 &&
                            ground.replantAfterOn == 0;
                SetAir(3);
                const AirRes airOff = RunAir(timed, hz);
                offOk &= airOff.episodes == 0 && airOff.replantStatusMissing == 0 && airOff.replantBeforeOn == 0 && airOff.replantAirOn == 0 &&
                         airOff.replantWindowOn == 0 && airOff.replantAfterOn == 0;
                if (!on || !groundOk || !offOk)
                    failures += " @" + std::to_string(int(hz)) + " hold=" + std::to_string(landed.replantWindowOn) + "/" +
                                std::to_string(landed.replantWindowFeeds) + " after=" + std::to_string(landed.replantAfterOn) +
                                "/" + std::to_string(landed.replantAfterFeeds);
            }
            Check("air hold [" + name + "] touchdown replant: ON after contact ramp (0.15..0.40 s), OFF after 0.55 s (30/60/144 Hz)",
                  timedOk, failures);
            Check("air hold [" + name + "] ground-only and AIR OFF: no temporary stand replant at 30/60/144 Hz",
                  groundOk && offOk, failures);
            SetFootReplant(savedReplant);
            SetAir(2);
        }
        // ---- jumps at 30 / 60 / 144 Hz
        bool jumpsOk = true, handOk = true, rateOk = true, traceOk = true;
        int addedAlt = 0;
        std::string jumpLines;
        for (const AirSc* sc : {&runJump, &standJump, &sprintJump, &stickJump})
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                const AirRes r = RunAir(*sc, hz);
                std::printf("[INFO] air hold [%s] %s %.0f Hz: %s%s\n", name.c_str(), sc->name.c_str(), hz, Line(r).c_str(),
                            r.picks.empty() ? "" : (" | owned picks:" + r.picks).c_str());
                const bool ok = HoldAirOk(r, 1.5f / hz) && r.episodes == 1 && r.takeoffs == 1;
                const bool hb = HandBackOk(r);
                // stride rate x0.5 on >= 90 % of the owned frames with a loop rate (the standing jump plays the idle at rate 1 x 0.5)
                const bool ro = sc->stick <= 0.15f || (r.rateN > 0 && r.rateOk * 10 >= r.rateN * 9);
                jumpsOk = jumpsOk && ok;
                handOk = handOk && hb;
                rateOk = rateOk && ro;
                traceOk = traceOk && r.mmair >= 2;
                addedAlt += r.addedAfter + (ListHasAddedClip(r.picks) ? 1 : 0);
                if (!ok || !hb || !ro)
                    jumpLines += " " + sc->name + "@" + std::to_string(int(hz)) + (ok ? "" : ":air") + (hb ? "" : ":handback") + (ro ? "" : ":rate");
            }
        Check("air hold [" + name + "] running / standing / sprint / stick-change jumps at 30 / 60 / 144 Hz: owned from the take-off feed "
                                    "to the touchdown, 0 searches / transitions / turn-pivot-reface picks, 0 locked feet, no body-turn input, "
                                    "MM weight >= 0.9 in the air",
              jumpsOk, jumpLines);
        Check("air hold [" + name + "] jumps: one hand-back at the touchdown, post-landing foot correction <= 15 cm, locked toe <= " +
                  std::to_string(kC1bToeCap).substr(0, 3) + " m/s, no stop clip while the stick is held",
              handOk, jumpLines);
        Check("air hold [" + name + "] jumps: stride rate x0.5 of the loop rate in the air (>= 90 % of the owned frames within 0.03)", rateOk,
              jumpLines);
        Check("air hold [" + name + "] jumps: UTPP MMAIR trace lines on the phase changes (>= 2 per jump)", traceOk, jumpLines);
        if (name.find("base_only") != std::string::npos)
            Check("air hold [" + name + "] motion set-only database: no non-Neutral clip picked in the air or within 0.6 s after a landing",
                  addedAlt == 0, std::to_string(addedAlt));
        // ---- drops
        for (const AirSc* sc : {&drop2, &drop6})
        {
            const AirRes r = RunAir(*sc, 60.0f);
            std::printf("[INFO] air hold [%s] %s 60 Hz: %s\n", name.c_str(), sc->name.c_str(), Line(r).c_str());
            Check("air hold [" + name + "] " + sc->name + ": ledge fall after minAir (owned within 0.15 s, to the touchdown), hold criteria, "
                                                           "one hand-back, clean landing",
                  HoldAirOk(r, 0.15f) && r.falls == 1 && r.episodes == 1 && HandBackOk(r), Line(r));
        }
        // ---- detail-14 blips 0.04 .. 0.32 s, climb / vault, dodge, slide: no episode
        {
            std::string bl;
            bool ok = true;
            for (float d : {0.04f, 0.08f, 0.16f, 0.24f, 0.32f})
            {
                AirSc b{.name = "blip", .kind = kBlip, .blip = d, .post = 0.5f};
                const AirRes r = RunAir(b, 60.0f);
                ok = ok && r.episodes == 0 && r.ownedAir == 0 && r.handbacks == 0 && r.resets == 0;
                char s[96];
                std::snprintf(s, sizeof s, " %.2fs:ep%d/own%d", d, r.episodes, r.ownedAir);
                bl += s;
            }
            Check("air hold [" + name + "] detail-14 blips 0.04-0.32 s (|vz| 0.5): no air episode, nothing owned", ok, bl);
            const AirRes rc = RunAir(climb, 60.0f), rd = RunAir(dodge, 60.0f), rs = RunAir(slide, 60.0f);
            Check("air hold [" + name + "] climb / vault (vz 2.5 off the ground), dodge (vz +-1.6), slide: no air episode",
                  rc.episodes == 0 && rc.ownedAir == 0 && rd.episodes == 0 && rd.ownedAir == 0 && rs.episodes == 0 && rs.ownedAir == 0,
                  "climb " + Line(rc) + " | dodge " + Line(rd) + " | slide " + Line(rs));
        }
        // ---- release detail (ladder) mid-air
        {
            const AirRes r = RunAir(release, 60.0f);
            std::printf("[INFO] air hold [%s] %s: %s wAfterRelease %.2f\n", name.c_str(), release.name.c_str(), Line(r).c_str(), r.wAfterRelease);
            Check("air hold [" + name + "] ladder (release detail 10) grabbed mid-air: Reset, nothing owned on the ladder, no hand-back",
                  r.resets == 1 && r.ownedAfterRelease == 0 && r.handbacks == 0 && r.episodes == 1, Line(r));
        }
        // ---- aim strafe, jump, land, aim strafe again
        {
            const AirRes r = RunAir(aimJump, 60.0f);
            std::printf("[INFO] air hold [%s] %s: %s strafe before %d off after %.3f s back after %.3f s strafe owned %d\n", name.c_str(),
                        aimJump.name.c_str(), Line(r).c_str(), r.strafeBefore ? 1 : 0, r.strafeOffLatency, r.strafeBackLatency, r.strafeOwned);
            Check("air hold [" + name + "] aim strafe then jump: strafe on before, off within 0.15 s + 2 frames in the air, back within 0.5 s "
                                        "after the landing; hold criteria in the air",
                  r.strafeBefore && r.strafeOffLatency >= 0.0f && r.strafeOffLatency <= 0.15f + 2.0f / 60.0f && r.strafeBackLatency >= 0.0f &&
                      r.strafeBackLatency <= 0.5f && HoldAirOk(r, 1.5f / 60.0f) && r.handbacks == 1,
                  Line(r));
        }
        // ---- hold repair: grounded jump-detail edge with the air feed (onGround true). Knockdown / car hit reported as detail 18
        // for 3 s after a sprint: one take-off that never leaves the ground, aborted after abortTime 0.25 s (one hand-back, no
        // second episode while detail 18 lasts, no search while owned)
        {
            bool ok = true;
            std::string kl;
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                const AirRes r = RunAir(knock, hz);
                const float dt = 1.0f / hz;
                const float ownedS = r.ownedGround * dt;
                std::printf("[INFO] air hold [%s] %s %.0f Hz: %s | owned on the ground %d feeds (%.3f s, %.3f..%.3f s after the edge, runs %d, "
                            "searches %d)\n",
                            name.c_str(), knock.name.c_str(), hz, Line(r).c_str(), r.ownedGround, ownedS, r.ownedGroundFirst,
                            r.ownedGroundLast, r.ownedGroundRuns, r.searchesOwnedGround);
                const bool o = r.takeoffs == 1 && r.episodes == 1 && r.handbacks == 1 && r.resets == 0 && r.ownedGroundRuns == 1 &&
                               r.ownedGroundFirst == 0.0f && ownedS > 0.0f && ownedS <= 0.25f + 1.5f * dt &&
                               r.ownedGroundLast <= 0.25f + 1.5f * dt && r.searchesOwnedGround == 0 && r.airFrames == 0;
                ok = ok && o;
                if (!o)
                    kl += " " + std::to_string(int(hz)) + "Hz:" + Line(r);
            }
            Check("air hold [" + name + "] knockdown / car hit as detail 18 on the ground (air feed onGround) at 30 / 60 / 144 Hz: one take-off "
                                        "from the edge feed, aborted within 0.25 s + 1 frame (one hand-back, no reset), no second episode "
                                        "while detail 18 lasts, no search while owned",
                  ok, kl);
        }
        // ---- hold repair: AirOwns > Strafe. Aim strafe, ONE feed with jump detail 18 on the ground (then aim walk again, |vz| 0):
        // strafe::Wanted is true again from the next feed, only the air ownership (g_airOwnsPrev, ~0.25 s until the abort) turns
        // strafe off; it comes back after the hand-back
        {
            const AirRes r = RunAir(aimEdge, 60.0f);
            std::printf("[INFO] air hold [%s] %s: %s | owned on the ground %d feeds (%.3f..%.3f s) strafe before %d off at %.3f s back at "
                        "%.3f s\n",
                        name.c_str(), aimEdge.name.c_str(), Line(r).c_str(), r.ownedGround, r.ownedGroundFirst, r.ownedGroundLast,
                        r.strafeBefore ? 1 : 0, r.strafeOffGround, r.strafeBackGround);
            Check("air hold [" + name + "] aim strafe + one grounded jump-edge feed: strafe yields to the air ownership (off within 0.15 s + "
                                        "2 frames while owned), back within 0.6 s after the edge; one take-off, one hand-back",
                  r.strafeBefore && r.ownedGround > 0 && r.strafeOffGround >= 0.0f && r.strafeOffGround <= r.ownedGroundLast + 1.0f / 60.0f &&
                      r.strafeOffGround <= 0.15f + 2.0f / 60.0f && r.strafeBackGround > r.strafeOffGround && r.strafeBackGround <= 0.6f &&
                      r.takeoffs == 1 && r.handbacks == 1 && r.resets == 0,
                  Line(r));
        }
        // ---- hold repair: FPP (TPP off = external release) running jump: no take-off (also not on the grounded edge), no Reset,
        // no MMAIR line and no raycast probe request; control: the TPP jump requests probes while descending
        {
            AirSc fpp = runJump;
            fpp.name = "jump_run_fpp";
            fpp.tppOff = true;
            const AirRes r = RunAir(fpp, 60.0f);
            const AirRes c = RunAir(runJump, 60.0f);
            std::printf("[INFO] air hold [%s] %s: %s probeWants %d | TPP control probeWants %d\n", name.c_str(), fpp.name.c_str(), Line(r).c_str(),
                        r.probeWants, c.probeWants);
            Check("air hold [" + name + "] FPP jump (release): no take-off / episode / Reset / MMAIR line, no probe request (TPP control "
                                        "requests probes)",
                  r.airFrames > 0 && r.takeoffs == 0 && r.episodes == 0 && r.resets == 0 && r.mmair == 0 && r.ownedAir == 0 &&
                      r.probeWants == 0 && c.probeWants > 0,
                  Line(r) + " probeWants " + std::to_string(r.probeWants) + "/" + std::to_string(c.probeWants));
        }
        // ---- no UTPPFeedAir (older script / stale air feed): the ground state is inferred from the PSM detail
        {
            const AirRes r = RunAir(runJump, 60.0f, false);
            std::printf("[INFO] air hold [%s] jump_run without the air feed: %s\n", name.c_str(), Line(r).c_str());
            Check("air hold [" + name + "] running jump without UTPPFeedAir (detail-inferred ground): hold criteria, one hand-back",
                  HoldAirOk(r, 1.5f / 60.0f) && r.episodes == 1 && HandBackOk(r), Line(r));
        }
        // ---- mm.air = off: exactly the previous path. Compared runs start from the same state: database reloaded, the section's
        // configuration, a 2 s ground warm-up (the driver keeps filters / controllers across runs)
        {
            const AirSc warm{.name = "warm_up", .kind = kSlide, .pre = 2.0f, .post = 0.5f};
            const auto fresh = [&](int32_t aMode) {
                const bool ok = LoadDatabase(wp, name);
                OnArmed(aRig, aHashes, aParents);
                SetEnabled(true);
                ConfigureShippedC1a();
                SetFeetDiag(true);
                SetUpperMaskStates(7);
                SetStrafe(true);
                SetAir(aMode);
                (void)RunAir(warm, 60.0f);
                return ok;
            };
            bool loaded = fresh(3);
            const std::string st = Status();
            const AirRes a = RunAir(stickJump, 60.0f, true);
            loaded = fresh(3) && loaded;
            const AirRes b = RunAir(stickJump, 60.0f, false);
            Check("air off [" + name + "]: no status segment, no MMAIR line, no air ownership; the air feed is inert (published pose of the "
                                       "jump bit-identical with / without UTPPFeedAir)",
                  loaded && st.find(" air=") == std::string::npos && a.mmair == 0 && a.ownedAir == 0 && a.episodes == 0 &&
                      a.hashes == b.hashes && !a.hashes.empty(),
                  Line(a));
            negSearches += a.searchesAir;
            negTurns += a.turnPicksAir;
            negLocks += a.locksAirAll;
            std::printf("[INFO] air off [%s] jump_stick_change 60 Hz (negative control): %s | while airborne: %d turn-class picks, %d feeds with "
                        "a locked foot of %d\n",
                        name.c_str(), Line(a).c_str(), a.turnPicksAir, a.locksAirAll, a.airFrames);
            // ground run + slide (no air event): auto and off publish the same poses
            const AirSc only{.name = "ground_run_slide", .kind = kSlide, .pre = 2.0f, .post = 1.0f};
            loaded = fresh(3) && loaded;
            const AirRes off = RunAir(only, 60.0f);
            loaded = fresh(0) && loaded;
            const AirRes on = RunAir(only, 60.0f);
            Check("air auto vs off [" + name + "]: ground run + slide (no air) bit-identical (published pose every feed)",
                  loaded && off.hashes == on.hashes && !on.hashes.empty() && on.episodes == 0, std::to_string(on.hashes.size()) + " feeds");
            // hold repair: knockdown as detail 18 on the ground with the air feed, auto vs off (the only grounded air effect: the
            // aborted take-off window)
            {
                loaded = fresh(3) && loaded;
                const AirRes koff = RunAir(knock, 60.0f);
                loaded = fresh(0) && loaded;
                const AirRes kon = RunAir(knock, 60.0f);
                int nd = 0, first = -1, last = -1;
                const size_t n = std::min(koff.hashes.size(), kon.hashes.size());
                for (size_t i = 0; i < n; ++i)
                    if (koff.hashes[i] != kon.hashes[i])
                    {
                        ++nd;
                        if (first < 0)
                            first = int(i);
                        last = int(i);
                    }
                const int ev = int(std::lround(knock.pre * 60.0f));
                std::printf("[INFO] air auto vs off [%s] %s 60 Hz: %zu / %zu feeds, %d differ (first %+.3f s, last %+.3f s from the edge); "
                            "off searches %d trans %d | auto %s\n",
                            name.c_str(), knock.name.c_str(), kon.hashes.size(), koff.hashes.size(), nd, first < 0 ? 0.0f : (first - ev) / 60.0f,
                            last < 0 ? 0.0f : (last - ev) / 60.0f, koff.searchesAir, koff.transAir, Line(kon).c_str());
                Check("air auto vs off [" + name + "]: knockdown detail 18 on the ground: identical before the edge, same feed count",
                      loaded && kon.hashes.size() == koff.hashes.size() && (first < 0 || first >= ev), std::to_string(nd) + " differ");
            }
        }
        // ---- CLIMB RELEASE stage: Climb 8 / Vault 9 -> game animation unless v2 owns the traversal (mm.climbRelease)
        {
            using namespace travt;
            const bool holdPrev = Field(Status(), "holdPose") > 0.5f;
            const uint64_t feedCalls0 = FeedCalls();
            const AirSc warm{.name = "warm_up", .kind = kSlide, .pre = 2.0f, .post = 0.5f};
            const auto freshT = [&](bool aRelease, const MotionHooks* aHooks, bool aOwns) {
                const bool ok = LoadDatabase(wp, name);
                OnArmed(aRig, aHashes, aParents);
                SetEnabled(true);
                ConfigureShippedC1a();
                SetFeetDiag(true);
                SetUpperMaskStates(7);
                SetStrafe(true);
                SetAir(2);
                SetHoldPose(true);
                SetClimbRelease(aRelease);
                g_ownsReturn = aOwns;
                SetMotionHooks(aHooks);
                (void)RunAir(warm, 60.0f);
                return ok;
            };
            SetLogger(&TravLog);
            g_travInfoLines = 0;
            bool loaded = freshT(true, nullptr, false);
            const std::string st = Status();
            Check("climb release [" + name + "]: status climbRelease=1 by default-on setting, travRelease=0 on the ground",
                  loaded && st.find(" climbRelease=1 travRelease=0 releaseReasons=0x0") != std::string::npos);
            bool onOk = true, offOk = true, travTraceOk = true;
            std::string onL, offL;
            for (travt::Kind k : {kVaultWalk, kJumpClimb})
                for (float hz : {30.0f, 60.0f})
                {
                    const char* kn = k == kVaultWalk ? "vault" : "jump_climb";
                    SetClimbRelease(true);
                    const TravRes a = RunTrav(k, hz);
                    std::printf("[INFO] climb release [%s] %s %.0f Hz on: %s\n", name.c_str(), kn, hz, TravLine(a).c_str());
                    // released: MM had the body before, travRelease on every 8 / 9 feed only, the weight reaches 0 with the gate
                    // fade (within 0.5 s) and is back (>= 0.99) within 1.5 s after the episode
                    const bool ok = a.evFeeds > 0 && a.wPre >= 0.99f && a.relFeeds == a.evFeeds && a.relOutside == 0 && a.wMinEv <= 0.0f &&
                                    a.zeroAfter >= 0.0f && a.zeroAfter <= 0.5f && a.backAfter >= 0.0f && a.backAfter <= 1.5f && a.wEnd >= 0.99f &&
                                    a.eps == 1.0f && a.owned == 0.0f;
                    onOk = onOk && ok;
                    travTraceOk = travTraceOk && a.gateLines >= 1 && a.onLines == 1 && a.offLines == 1;
                    if (!ok)
                        onL += std::string(" ") + kn + "@" + std::to_string(int(hz)) + ": " + TravLine(a);
                    SetClimbRelease(false);
                    const TravRes b = RunTrav(k, hz);
                    std::printf("[INFO] climb release [%s] %s %.0f Hz off: %s\n", name.c_str(), kn, hz, TravLine(b).c_str());
                    const bool okb = b.evFeeds > 0 && b.wMinEv >= 0.99f && b.relFeeds == 0 && b.relOutside == 0 && b.eps == 0.0f &&
                                     b.onLines == 0 && b.gateLines == 0;
                    offOk = offOk && okb;
                    if (!okb)
                        offL += std::string(" ") + kn + "@" + std::to_string(int(hz)) + ": " + TravLine(b);
                }
            Check("climb release [" + name + "] on: vault (9) / jump into a climb (8) at 30 / 60 Hz - travRelease on every 8 / 9 feed only, "
                                          "MM weight 1 -> 0 within 0.5 s (gate fade), back to >= 0.99 within 1.5 s after, one episode",
                  onOk, onL);
            Check("climb release [" + name + "] on: MMGATE line carries 'release=0x1 travRelease', one MMTRAV on / off line per episode", travTraceOk);
            Check("climb release [" + name + "] TRAV-1 D: one always-on 'UTPP trav: released to game animation' line per release episode",
                  g_travInfoLines == 4, std::to_string(g_travInfoLines) + " lines for 4 released episodes");
            Check("climb release [" + name + "] off: 8 / 9 held as before (weight >= 0.99, no release reason, no trace)", offOk, offL);
            // bit-identical: off == on with v2 owning the traversal (hook true); on with the stub hook (false) == on without hooks
            bool idOk = true, stubOk = true, ownOk = true;
            std::string idL;
            for (travt::Kind k : {kVaultWalk, kJumpClimb})
            {
                loaded = freshT(false, nullptr, false) && loaded;
                const TravRes off = RunTrav(k, 60.0f);
                loaded = freshT(true, &kOwnsHooks, true) && loaded;
                const TravRes own = RunTrav(k, 60.0f);
                loaded = freshT(true, &kOwnsHooks, false) && loaded;
                const TravRes stub = RunTrav(k, 60.0f);
                loaded = freshT(true, nullptr, false) && loaded;
                const TravRes on = RunTrav(k, 60.0f);
                idOk = idOk && off.hashes == own.hashes && !off.hashes.empty();
                stubOk = stubOk && stub.hashes == on.hashes && on.hashes != off.hashes;
                ownOk = ownOk && own.owned == float(own.evFeeds) && own.relFeeds == 0 && own.eps == 0.0f && stub.owned == 0.0f && stub.eps == 1.0f;
                idL += std::string(" ") + (k == kVaultWalk ? "vault" : "jump_climb") + " own: " + TravLine(own) + " | stub: " + TravLine(stub);
            }
            std::printf("[INFO] climb release [%s] identity runs:%s\n", name.c_str(), idL.c_str());
            Check("climb release [" + name + "]: setting off bit-identical to on with motion owning the traversal (published pose + weight every feed)",
                  loaded && idOk, idL);
            Check("climb release [" + name + "]: OwnsTraversal false (stub hook) bit-identical to no motion hooks; released differs from off",
                  loaded && stubOk);
            Check("climb release [" + name + "]: motion owning -> no release reason on any 8 / 9 feed (travOwnedFeeds = 8 / 9 feeds)", ownOk, idL);
            // TRAV-1 (vehicle batch, P1a; video 2026-10-22 hood vaults: a held ground pose walks through the car): with v2 traversal
            // active (MotionHooks::travActive) mm.climbRelease off no longer holds a Climb / Vault that v2 does not play - released exactly
            // as with the setting on (same pose + weight every feed), one always-on info line per episode. v2 owning keeps the body
            // (== off); travActive false keeps the legacy v1 hold (== off, no hooks). Mutation 4 (aMotionActive dropped) fails this.
            {
                bool actOk = true, actOwnOk = true, inactOk = true;
                std::string tl;
                for (travt::Kind k : {kVaultWalk, kJumpClimb})
                {
                    const char* kn = k == kVaultWalk ? "vault" : "jump_climb";
                    loaded = freshT(true, nullptr, false) && loaded;
                    const TravRes on = RunTrav(k, 60.0f);
                    loaded = freshT(false, nullptr, false) && loaded;
                    const TravRes off = RunTrav(k, 60.0f);
                    g_travActiveReturn = true;
                    loaded = freshT(false, &kTravActiveHooks, false) && loaded;
                    const int info0 = g_travInfoLines;
                    const TravRes act = RunTrav(k, 60.0f);
                    const int infoN = g_travInfoLines - info0;
                    loaded = freshT(false, &kTravActiveHooks, true) && loaded;
                    const TravRes own = RunTrav(k, 60.0f);
                    g_travActiveReturn = false;
                    loaded = freshT(false, &kTravActiveHooks, false) && loaded;
                    const TravRes ina = RunTrav(k, 60.0f);
                    g_travActiveReturn = true;
                    actOk = actOk && act.evFeeds > 0 && act.hashes == on.hashes && act.relFeeds == act.evFeeds && act.relOutside == 0 &&
                            act.wMinEv <= 0.0f && act.eps == 1.0f && act.owned == 0.0f && infoN == 1;
                    actOwnOk = actOwnOk && own.hashes == off.hashes && own.relFeeds == 0 && own.eps == 0.0f && own.owned == float(own.evFeeds);
                    inactOk = inactOk && ina.hashes == off.hashes && ina.relFeeds == 0 && ina.eps == 0.0f && ina.owned == 0.0f;
                    tl += std::string(" ") + kn + " active: " + TravLine(act) + " info " + std::to_string(infoN) + " | owned: " + TravLine(own) +
                          " | inactive: " + TravLine(ina);
                }
                std::printf("[INFO] climb release [%s] TRAV-1 travActive runs:%s\n", name.c_str(), tl.c_str());
                Check("climb release [" + name + "] TRAV-1: setting off + motion traversal active (travActive) releases an un-owned 8 / 9 exactly as "
                                                 "the setting on (pose + weight every feed), one 'released to game animation' info line",
                      loaded && actOk, tl);
                Check("climb release [" + name + "] TRAV-1: setting off + travActive + motion owning == setting off (held, owned on every 8 / 9 feed)",
                      loaded && actOwnOk, tl);
                Check("climb release [" + name + "] TRAV-1: setting off + travActive false == setting off without hooks (legacy base hold)",
                      loaded && inactOk, tl);
            }
            // review repair: feeds stop mid-episode (MM disabled, Tick only, 0.4 s real time). On: the Tick keeps the last feed's
            // release reason (no hold, the weight fades to 0). Setting turned off mid-episode: the reason is cleared at once (status)
            // and the Tick holds the body as before this stage.
            {
                bool tOk = true, tOffOk = true;
                std::string tL;
                for (travt::Kind k : {kVaultWalk, kJumpClimb})
                {
                    const char* kn = k == kVaultWalk ? "vault" : "jump_climb";
                    loaded = freshT(true, nullptr, false) && loaded;
                    const TravRes a = RunTrav(k, 60.0f, 2);
                    const bool relA = Field(Status(), "travRelease") > 0.5f;
                    const TickRes ta = TickDisabled(0.4f);
                    tOk = tOk && relA && a.evFeeds == 2 && ta.zeroAfter >= 0.0f && ta.wEnd <= 0.0f && ta.wMax <= ta.wStart && ta.ticks == 0.0f &&
                          !ta.active;
                    loaded = freshT(true, nullptr, false) && loaded;
                    const TravRes b = RunTrav(k, 60.0f, 2);
                    const bool relB = Field(Status(), "travRelease") > 0.5f;
                    SetClimbRelease(false);
                    const std::string stOff = Status();
                    const bool cleared = stOff.find(" climbRelease=0 travRelease=0 releaseReasons=0x0") != std::string::npos;
                    const TickRes tb = TickDisabled(0.4f);
                    SetClimbRelease(true);
                    tOffOk = tOffOk && relB && b.evFeeds == 2 && cleared && tb.ticks > 0.0f && tb.active && tb.zeroAfter < 0.0f &&
                             tb.wEnd >= tb.wStart;
                    tL += std::string(" ") + kn + " on: " + TickLine(ta) + " | off mid-episode (cleared " + (cleared ? "1" : "0") +
                          "): " + TickLine(tb);
                }
                std::printf("[INFO] climb release [%s] no-feed ticks:%s\n", name.c_str(), tL.c_str());
                Check("climb release [" + name + "] no feeds mid-episode (MM disabled, Tick only): released - no hold tick, weight to 0", tOk,
                      tL);
                Check("climb release [" + name + "] setting off mid-episode: release reason cleared at once, the Tick holds the body", tOffOk,
                      tL);
            }
            // review repair: the global feed count drives the every-2nd-feed air probe (AirWantsProbe); keep its parity as it was
            // before this block so the later sections see the same probe phase as without the climb release checks
            if ((FeedCalls() & 1u) != (feedCalls0 & 1u))
            {
                FeedInput pad;
                pad.velWorld[1] = 1.6f;
                pad.move[1] = 1.0f;
                pad.locoState = 1;
                pad.tppActive = true;
                pad.dt = 1.0f / 60.0f;
                pad.haveTarget = true;
                pad.haveAir = true;
                pad.onGround = true;
                pad.distToGround = -2.0f;
                g_clock += pad.dt;
                Feed(pad);
            }
            SetMotionHooks(nullptr);
            SetClimbRelease(true);
            SetHoldPose(holdPrev);
            SetLogger(&AirLog);
        }
        SetAir(0);
    }
    Check("negative control mm.air=off: the matcher searches and feet lock while airborne in the stick-change jump (some database)",
          negSearches > 0 && negLocks > 0,
          std::to_string(negSearches) + " searches, " + std::to_string(negTurns) + " turn-class picks, " + std::to_string(negLocks) +
              " feeds with a locked foot");
    for (const std::string& l : g_mmairSample)
        std::printf("[INFO] air hold trace sample: %s\n", l.c_str());
    SetLogger(&TraceLog);
    SetAir(3); // back to the older sections' setting
    SetStrafe(false);
    SetFeetDiag(false);
    SetHoldPose(false);
    SetUpperMask(false);
}

// AIR CLIPS stage (2026-10-13): mm.air=auto with the .airz sidecars next to v6 / v7b / utpp_base -> clips mode. Air clip
// sequence (forced clips from the UTPP MMAIRCLIP trace), TTL error at the land entry, heavy / light lands, the F_Off ledge drop, the
// matcher Hips height above the capsule in the air (root_z removed: no double vertical), hand-back to ground clips. A hard link of
// utpp_base without a sidecar checks auto -> hold (noTable).
void SectionAirClips(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionAirClips");
    (void)argc;
    using namespace airt;
    std::string dir = argv[1];
    {
        const size_t sl = dir.find_last_of("\\/");
        dir = sl == std::string::npos ? std::string() : dir.substr(0, sl + 1);
    }
    std::vector<std::string> paths;
    for (const char* n : {"utpp_base"})
        if (GetFileAttributesA((dir + n + ".utppmm").c_str()) != INVALID_FILE_ATTRIBUTES &&
            GetFileAttributesA((dir + n + ".airz").c_str()) != INVALID_FILE_ATTRIBUTES)
            paths.push_back(dir + n + ".utppmm");
    Check("air clips: database utpp_base with .airz sidecar present", paths.size() == 1, std::to_string(paths.size()));
    SetLogger(&AirLog);
    g_clipMetrics = true;
    const AirSc runJump{.name = "jump_run"};
    const AirSc standJump{.name = "jump_stand", .speed = 0.0f, .stick = 0.0f, .vz0 = 5.5f};
    const AirSc sprintJump{.name = "jump_sprint", .speed = 8.2f, .loco = 3};
    const AirSc stickJump{.name = "jump_stick_change", .stickTurn = 90.0f};
    const AirSc drop2{.name = "drop_2m_walkoff", .kind = kDrop, .speed = 3.85f, .dropH = 2.0f};
    const AirSc drop6{.name = "drop_6m", .kind = kDrop, .speed = 3.85f, .dropH = 6.0f};
    auto runOne = [&](const AirSc& aSc, float aHz) {
        g_clipEvs.clear();
        AirRes r = RunAir(aSc, aHz);
        r.clipEvs = g_clipEvs;
        return r;
    };
    auto evLine = [](const AirRes& r) {
        std::string s;
        char b[200];
        for (const AirRes::ClipEv& e : r.clipEvs)
        {
            std::snprintf(b, sizeof b, " %s:%s@%.3f(ttl %.2f)", e.action.c_str(), e.clip.c_str(), e.t, e.ttl);
            s += b;
        }
        std::snprintf(b, sizeof b, " | touch@%.3f hipsZ %.3f..%.3f end %s", r.touchAt, r.hipsMin, r.hipsMax, r.endClip.c_str());
        return s + b;
    };
    // TTL error at the land entry (s): (touchdown - land entry) - predicted ttl; 9 = no land entry before the touchdown
    auto landTtlErr = [](const AirRes& r) {
        for (const AirRes::ClipEv& e : r.clipEvs)
            if (e.action == "land" && r.touchAt >= 0.0f && e.t < r.touchAt)
                return (r.touchAt - e.t) - e.ttl;
        return 9.0f;
    };
    auto has = [](const AirRes& r, const char* aAction, const char* aSub) {
        for (const AirRes::ClipEv& e : r.clipEvs)
            if (e.action == aAction && (!aSub || e.clip.find(aSub) != std::string::npos))
                return true;
        return false;
    };
    // forced air clips are never non-motion clips; on the motion-only database no clip around the episode is either
    auto addedAlt = [](const AirRes& r, bool aMotionOnly) {
        int n = 0;
        for (const AirRes::ClipEv& e : r.clipEvs)
            n += IsAddedClipName(e.clip) ? 1 : 0;
        if (aMotionOnly)
            n += int(ListHasAddedClip(r.seq));
        return n;
    };
    // every clip change while airborne and owned is a forced air clip (no ground / turn / pivot clip in the air)
    auto airClean = [](const AirRes& r) {
        return r.searchesOwned == 0 && r.turnPicksOwned == 0 && r.picks.find("Jump_") != std::string::npos &&
               [&] {
                   size_t a = 0;
                   const std::string& p = r.picks;
                   while (a < p.size())
                   {
                       const size_t b = p.find(' ', a + 1);
                       const std::string t = p.substr(a + 1, b == std::string::npos ? std::string::npos : b - a - 1);
                       if (!t.empty() && t.find("Jump_") == std::string::npos)
                           return false;
                       if (b == std::string::npos)
                           break;
                       a = b;
                   }
                   return true;
               }();
    };
    // the hold criteria with forced transitions allowed: every owned airborne transition is one of the forced air clips
    auto clipsAirOk = [](const AirRes& r, float aMaxFirst) {
        int forced = 0;
        for (const AirRes::ClipEv& e : r.clipEvs)
            forced += (r.touchAt < 0.0f || e.t < r.touchAt) ? 1 : 0;
        return r.airFrames > 0 && r.ownedAir > 0 && r.firstOwnedAfter >= 0.0f && r.firstOwnedAfter <= aMaxFirst && r.notOwnedAfterStart == 0 &&
               r.searchesOwned == 0 && r.transOwned <= forced && r.turnPicksOwned == 0 && r.locksAir == 0 && r.btOwned == 0 && r.wMinAir >= 0.9f;
    };
    auto hipsOk = [](const AirRes& r) { return r.hipsMin >= 0.30f && r.hipsMax <= 1.40f && r.hipsMax >= r.hipsMin; };
    auto endGround = [](const AirRes& r) { return !r.endClip.empty() && r.endClip.find("Jump_") == std::string::npos; };
    for (const std::string& path : paths)
    {
        std::string name = path.substr(path.find_last_of("\\/") + 1);
        name = name.substr(0, name.rfind('.'));
        const std::wstring wp(path.begin(), path.end());
        if (!LoadDatabase(wp, name))
        {
            Check("air clips: database loaded " + name, false, path);
            continue;
        }
        OnArmed(aRig, aHashes, aParents);
        SetEnabled(true);
        ConfigureShippedC1a();
        SetFeetDiag(true);
        SetUpperMaskStates(7);
        SetStrafe(true);
        SetAir(0);
        {
            const std::string st = Status();
            const size_t p = st.find(" air=");
            std::printf("[INFO] air clips [%s] status:%s\n", name.c_str(), p == std::string::npos ? " (none)" : st.substr(p, 420).c_str());
            Check("air clips [" + name + "]: status air=clips airReason=auto:table, table fields appended", p != std::string::npos &&
                  st.find(" air=clips airReason=auto:table") != std::string::npos && st.find(" airTable=") != std::string::npos &&
                  Field(st, "airLoop") > 0.5f);
        }
        const bool motionOnly = name.find("base_only") != std::string::npos;
        // ---- jumps at 30 / 60 / 144 Hz: Start -> (fall loop) -> Land before the touchdown, TTL error <= 0.05 s
        bool seqOk = true, ttlOk = true, airOk = true, hipOk = true, hbOk = true, heavyOk = true;
        int ng = 0;
        float worstTtl = 0.0f;
        std::string bad;
        int minEntryAll = 999, worstAlign = 0, aligned = 0; // AIR CLIPS repair
        int landRealignsAll = 0;
        int landBlendsAll = 0;
        float worstFirstHipsErr = 0.0f;
        constexpr int kAlignMax = 2;                        // clip frames: round(TTL * 30) + TTL error <= 0.05 s
        std::string repBad;
        for (const AirSc* sc : {&runJump, &standJump, &sprintJump, &stickJump})
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                const AirRes r = runOne(*sc, hz);
                landRealignsAll += r.landRealigns;
                landBlendsAll += r.landBlends;
                if (r.landBlends > 0)
                    worstFirstHipsErr = std::max(worstFirstHipsErr, r.landBlendFirstHipsErr);
                const float te = landTtlErr(r);
                std::printf("[INFO] air clips [%s] %s %.0f Hz: TTL err %+.3f s |%s | %s\n", name.c_str(), sc->name.c_str(), hz, te,
                            evLine(r).c_str(), Line(r).c_str());
                const bool sq = !r.clipEvs.empty() && r.clipEvs.front().action == "takeoff" && r.clipEvs.front().clip.find("_Start_") != std::string::npos &&
                                has(r, "land", "_Land_") && r.episodes == 1 && r.takeoffs == 1;
                const bool tt = std::fabs(te) <= 0.05f;
                const bool ao = clipsAirOk(r, 1.5f / hz) && airClean(r);
                const bool ho = hipsOk(r);
                const bool hb = HandBackOk(r) && endGround(r);
                const bool hv = !has(r, "land", "Heavy"); // flat jumps land light
                worstTtl = std::max(worstTtl, std::fabs(te));
                seqOk = seqOk && sq;
                ttlOk = ttlOk && tt;
                airOk = airOk && ao;
                hipOk = hipOk && ho;
                hbOk = hbOk && hb;
                heavyOk = heavyOk && hv;
                ng += addedAlt(r, motionOnly);
                // AIR CLIPS repair: Start / Land entries never at clip frame 0; the land clip's touch frame is reached at the
                // touchdown (clip-local frame after the touchdown feed - touch frame, in clip frames)
                {
                    int minEntry = 999, align = 999;
                    for (const AirRes::ClipEv& e : r.clipEvs)
                    {
                        if (e.action == "takeoff" || e.action == "land")
                            minEntry = std::min(minEntry, e.entry);
                        if (e.action == "land" && r.touchAt >= 0.0f && e.t < r.touchAt && e.clip == r.touchClip && e.touchF >= 0)
                            align = r.touchClipFrame - e.touchF;
                    }
                    std::printf("[INFO] air clips [%s] %s %.0f Hz: min Start/Land entry %d, touch alignment %+d clip frames (at the "
                                "touchdown %s f%d), query overrides %d\n",
                                name.c_str(), sc->name.c_str(), hz, minEntry, align, r.touchClip.c_str(), r.touchClipFrame, r.queryOverrides);
                    // align 999: no land clip at the touchdown (turn on landing: handed back to ground MM at the touchdown)
                    minEntryAll = std::min(minEntryAll, minEntry);
                    if (align != 999)
                    {
                        worstAlign = std::max(worstAlign, std::abs(align));
                        ++aligned;
                    }
                    if (minEntry < 1 || (align != 999 && std::abs(align) > kAlignMax))
                        repBad += " " + sc->name + "@" + std::to_string(int(hz)) + ":entry" + std::to_string(minEntry) + ":align" +
                                  std::to_string(align);
                }
                if (!sq || !tt || !ao || !ho || !hb || !hv)
                    bad += " " + sc->name + "@" + std::to_string(int(hz)) + (sq ? "" : ":seq") + (tt ? "" : ":ttl") + (ao ? "" : ":air") +
                           (ho ? "" : ":hips") + (hb ? "" : ":handback") + (hv ? "" : ":heavy");
            }
        Check("air clips [" + name + "] running / standing / sprint / stick-change jumps at 30 / 60 / 144 Hz: a Jump Start clip at the take-off, "
                                     "a Land clip before the touchdown, one episode",
              seqOk, bad);
        char tb[64];
        std::snprintf(tb, sizeof tb, "worst %.3f s", worstTtl);
        Check("air clips [" + name + "] jumps: TTL error at the land entry <= 0.05 s (30 / 60 / 144 Hz)", ttlOk, std::string(tb) + bad);
        Check("air clips [" + name + "] jumps: owned in the air, 0 searches, only forced Jump clips (no ground / turn / pivot clip), no "
                                     "locked foot, weight >= 0.9",
              airOk, bad);
        Check("air clips [" + name + "] jumps: matcher Hips z above the capsule stays in 0.30..1.40 m in the air (root_z removed, no double "
                                     "vertical)",
              hipOk, bad);
        Check("air clips [" + name + "] jumps: one hand-back, post-landing foot correction <= 15 cm, locked toe <= cap, ground clip at the "
                                     "end; flat jumps land light",
              hbOk && heavyOk, bad);
        // ---- AIR CLIPS repair: entry frames, touch alignment, hand-back query override
        Check("air clips [" + name + "] jumps: Start / Land clips never entered at clip frame 0; the land clip is at its touch frame at the "
                                     "touchdown (|clip frame - touch| <= 2) at 30 / 60 / 144 Hz",
              minEntryAll >= 1 && aligned >= 9 && worstAlign <= kAlignMax,
              "min entry " + std::to_string(minEntryAll) + " aligned " + std::to_string(aligned) + " worst " + std::to_string(worstAlign) + repBad);
        Check("air clips [" + name + "] genuine touchdown: predicted Land never >2 frames past touch after driver alignment "
                                     "(30 / 60 / 144 Hz); at most one corrective ForcePlay per jump",
              worstAlign <= kAlignMax && landRealignsAll <= 12,
              "corrective realigns " + std::to_string(landRealignsAll) + " worst align " + std::to_string(worstAlign));
        Check("air clips [" + name + "] Recover -> ground HandBack: at most one 0.18 s pre-IK blend per jump "
                                     "(30 / 60 / 144 Hz, without affecting matcher searches)",
              landBlendsAll <= 12 && worstFirstHipsErr <= 1.0e-4f,
              "land blends " + std::to_string(landBlendsAll) + " first-frame Hips mismatch " + std::to_string(worstFirstHipsErr));
        // ---- AIR CLIPS repair: release mid-air (ladder grab at the apex: Reset while the Start clip carries root_z): the first
        // search takes its bone features from the last clean ground frame (one airQuery override)
        {
            const AirSc relJump{.name = "jump_ladder_grab_release", .kind = kRelease};
            const AirRes r = runOne(relJump, 60.0f);
            std::printf("[INFO] air clips [%s] %s 60 Hz: query overrides %d |%s | %s\n", name.c_str(), relJump.name.c_str(), r.queryOverrides,
                        evLine(r).c_str(), Line(r).c_str());
            Check("air clips [" + name + "] release mid-air (Reset from a clip frame with root_z): the first search takes its bone features "
                                         "from a clean ground frame (one airQuery override), one Reset",
                  has(r, "takeoff", "_Start_") && r.resets == 1 && r.queryOverrides == 1, Line(r));
        }
        // ---- AIR CLIPS repair: 140 m walk-off: F_Off -> Jump_Loop_Fall for longer than the clip (loop override in the driver: the
        // clip wraps and never sits on one frame), heavy land, hand-back
        {
            const AirSc drop20{.name = "drop_140m_walkoff", .kind = kDrop, .speed = 3.85f, .dropH = 140.0f};
            const AirRes r = runOne(drop20, 60.0f);
            const float te = landTtlErr(r);
            const float loopS = r.fallLoopFeeds / 60.0f, clipS = r.fallLoopNumFrames > 0 ? (r.fallLoopNumFrames - 1) / 30.0f : -1.0f;
            std::printf("[INFO] air clips [%s] %s 60 Hz: TTL err %+.3f s | fall loop %d feeds (%.3f s, clip %.3f s) wraps %d longest "
                        "same-frame run %d feeds |%s | %s\n",
                        name.c_str(), drop20.name.c_str(), te, r.fallLoopFeeds, loopS, clipS, r.fallLoopWraps, r.fallLoopStreak,
                        evLine(r).c_str(), Line(r).c_str());
            ng += addedAlt(r, motionOnly);
            Check("air clips [" + name + "] drop_140m walk-off: Jump_Loop_Fall plays longer than the clip and loops (wraps >= 1, never more "
                                         "than 3 feeds on one frame), Heavy land before the touchdown (TTL error <= 0.05 s), Hips band, hand-back",
                  has(r, "fallLoop", "Jump_Loop_Fall") && clipS > 0.0f && loopS > clipS && r.fallLoopWraps >= 1 && r.fallLoopStreak <= 3 &&
                      has(r, "land", "_Heavy") && std::fabs(te) <= 0.05f && clipsAirOk(r, 0.15f) && airClean(r) && hipsOk(r) &&
                      HandBackOk(r) && endGround(r),
                  evLine(r));
        }
        // ---- drops: 2 m walk-off (F_Off drop clip), 6 m (heavy land)
        {
            const AirRes r2 = runOne(drop2, 60.0f), r6 = runOne(drop6, 60.0f);
            const float te2 = landTtlErr(r2), te6 = landTtlErr(r6);
            std::printf("[INFO] air clips [%s] %s 60 Hz: TTL err %+.3f s |%s | %s\n", name.c_str(), drop2.name.c_str(), te2, evLine(r2).c_str(),
                        Line(r2).c_str());
            std::printf("[INFO] air clips [%s] %s 60 Hz: TTL err %+.3f s |%s | %s\n", name.c_str(), drop6.name.c_str(), te6, evLine(r6).c_str(),
                        Line(r6).c_str());
            ng += addedAlt(r2, motionOnly) + addedAlt(r6, motionOnly);
            Check("air clips [" + name + "] drop_2m walk-off ledge: F_Off drop clip first, then a light Land before the touchdown (TTL error "
                                         "<= 0.05 s), hold criteria, hand-back",
                  !r2.clipEvs.empty() && r2.clipEvs.front().action == "off" && r2.clipEvs.front().clip.find("_Off_") != std::string::npos &&
                      has(r2, "land", "_Light") && std::fabs(te2) <= 0.05f && clipsAirOk(r2, 0.15f) && airClean(r2) && hipsOk(r2) &&
                      HandBackOk(r2) && endGround(r2),
                  evLine(r2));
            Check("air clips [" + name + "] drop_6m: Heavy land clip before the touchdown (TTL error <= 0.05 s), hold criteria, Hips band, "
                                         "hand-back",
                  has(r6, "land", "_Heavy") && !has(r6, "land", "_Light") && std::fabs(te6) <= 0.05f && clipsAirOk(r6, 0.15f) && airClean(r6) &&
                      hipsOk(r6) && HandBackOk(r6) && endGround(r6),
                  evLine(r6));
        }
        Check("air clips [" + name + "]: no non-Neutral clip forced" + (motionOnly ? " or picked around the air episodes" : ""), ng == 0,
              std::to_string(ng));
        // ---- mm.air=hold and off with the table loaded: no air clip is forced
        {
            SetAir(2);
            const AirRes h = runOne(runJump, 60.0f);
            SetAir(3);
            const AirRes o = runOne(runJump, 60.0f);
            SetAir(0);
            Check("air clips [" + name + "]: mm.air=hold / off with the table loaded force no air clip (hold criteria / no ownership)",
                  h.clipEvs.empty() && HoldAirOk(h, 1.5f / 60.0f) && HandBackOk(h) && o.clipEvs.empty() && o.ownedAir == 0,
                  "hold " + Line(h) + " | off " + Line(o));
        }
        // ---- AIR CLIPS repair: the table reaches the matcher (root_z subtraction, fall loop override) only under auto / clips
        {
            std::string rz;
            bool ok = true;
            for (int32_t m : {0, 1, 2, 3, 0})
            {
                SetAir(m);
                const std::string st = Status();
                const bool want = m <= 1;
                const bool rzOn = Field(st, "airRootZ") > 0.5f, lo = Field(st, "airLoopOv") > 0.5f;
                // mm.air=off prints no air segment at all (the 1cf5307 status text): the fields read 0 there
                ok = ok && rzOn == want && lo == want && (m == 3) == (st.find(" airRootZ=") == std::string::npos);
                rz += " " + std::to_string(m) + ":" + (rzOn ? "1" : "0") + (lo ? "1" : "0");
            }
            std::printf("[INFO] air clips [%s] mm.air 0/1/2/3/0 -> airRootZ airLoopOv:%s\n", name.c_str(), rz.c_str());
            Check("air clips [" + name + "]: root_z / fall loop override applied to the matcher under auto / clips only, removed under hold / "
                                         "off and re-applied (status airRootZ / airLoopOv)",
                  ok, rz);
        }
        // ---- AIR CLIPS repair: knockdown / car hit as detail 18 on the ground (capsule not rising): clips mode only arms the
        // take-off (no Start clip, no ownership); published poses identical to mm.air=off
        {
            const AirSc knock{.name = "knockdown_detail18_ground", .kind = kKnock, .speed = 8.2f, .loco = 3, .pre = 2.5f, .blip = 3.0f, .post = 1.5f};
            const AirSc warm{.name = "warm_up", .kind = kSlide, .pre = 2.0f, .post = 0.5f};
            const auto fresh = [&](int32_t aMode) {
                const bool ok = LoadDatabase(wp, name);
                OnArmed(aRig, aHashes, aParents);
                SetEnabled(true);
                ConfigureShippedC1a();
                SetFeetDiag(true);
                SetUpperMaskStates(7);
                SetStrafe(true);
                SetAir(aMode);
                (void)RunAir(warm, 60.0f);
                return ok;
            };
            bool loaded = fresh(3);
            const AirRes koff = runOne(knock, 60.0f);
            loaded = fresh(0) && loaded;
            const AirRes kon = runOne(knock, 60.0f);
            int nd = 0;
            for (size_t i = 0; i < std::min(koff.hashes.size(), kon.hashes.size()); ++i)
                nd += koff.hashes[i] != kon.hashes[i] ? 1 : 0;
            std::printf("[INFO] air clips [%s] %s 60 Hz auto vs off: %zu / %zu feeds, %d differ | auto %s\n", name.c_str(), knock.name.c_str(),
                        kon.hashes.size(), koff.hashes.size(), nd, Line(kon).c_str());
            Check("air clips [" + name + "]: knockdown detail 18 on the ground (not rising): no take-off, no Start clip, no ownership; "
                                         "published poses identical to mm.air=off",
                  loaded && kon.hashes.size() == koff.hashes.size() && nd == 0 && kon.takeoffs == 0 && kon.episodes == 0 &&
                      kon.clipEvs.empty() && kon.ownedGround == 0,
                  std::to_string(nd) + " differ " + Line(kon));
            SetAir(0);
        }
    }
    // ---- no table: a hard link of utpp_base without its sidecar -> auto runs hold
    {
        const std::string src = dir + "utpp_base.utppmm";
        char exe[MAX_PATH] = {};
        GetModuleFileNameA(nullptr, exe, MAX_PATH);
        std::string ldir = exe;
        ldir = ldir.substr(0, ldir.find_last_of("\\/") + 1) + "air_notable\\";
        CreateDirectoryA(ldir.c_str(), nullptr);
        const std::string dst = ldir + "utpp_base.utppmm";
        // AIR CLIPS repair: a private copy (no hard link: a write through it would change the Data original, and a link fails
        // across volumes). An older hard link is removed first (deleting a link name leaves the Data file intact).
        std::string copyErr;
        {
            const HANDLE h = CreateFileA(dst.c_str(), 0, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, 0, nullptr);
            if (h != INVALID_HANDLE_VALUE)
            {
                BY_HANDLE_FILE_INFORMATION fi{};
                const bool linked = GetFileInformationByHandle(h, &fi) && fi.nNumberOfLinks > 1;
                CloseHandle(h);
                if (linked && !DeleteFileA(dst.c_str()))
                    copyErr = "cannot remove the old hard link " + dst;
            }
            WIN32_FILE_ATTRIBUTE_DATA sa{}, da{};
            const bool haveSrc = GetFileAttributesExA(src.c_str(), GetFileExInfoStandard, &sa) != 0;
            const bool same = GetFileAttributesExA(dst.c_str(), GetFileExInfoStandard, &da) != 0 && haveSrc &&
                              sa.nFileSizeHigh == da.nFileSizeHigh && sa.nFileSizeLow == da.nFileSizeLow;
            if (copyErr.empty() && !same && (!haveSrc || !CopyFileA(src.c_str(), dst.c_str(), FALSE)))
                copyErr = "copy " + src + " -> " + dst + " failed (error " + std::to_string(GetLastError()) + ")";
        }
        if (!copyErr.empty())
            std::printf("[INFO] air clips [no table] %s\n", copyErr.c_str());
        const bool noSidecar = GetFileAttributesA((ldir + "utpp_base.airz").c_str()) == INVALID_FILE_ATTRIBUTES;
        const std::wstring wp(dst.begin(), dst.end());
        bool ok = copyErr.empty() && noSidecar && LoadDatabase(wp, "utpp_base_notable");
        if (ok)
        {
            OnArmed(aRig, aHashes, aParents);
            SetEnabled(true);
            ConfigureShippedC1a();
            SetFeetDiag(true);
            SetUpperMaskStates(7);
            SetStrafe(true);
            SetAir(0);
        }
        const std::string st = Status();
        const AirRes r = ok ? runOne(runJump, 60.0f) : AirRes{};
        std::printf("[INFO] air clips [no table] jump_run 60 Hz: %s\n", Line(r).c_str());
        Check("air clips: utpp_base without .airz -> status air=hold airReason=auto:noTable, hold criteria, no air clip forced",
              ok && st.find(" air=hold airReason=auto:noTable") != std::string::npos && st.find(" airTable=") == std::string::npos &&
                  r.clipEvs.empty() && HoldAirOk(r, 1.5f / 60.0f) && HandBackOk(r),
              copyErr.empty() ? Line(r) : copyErr);
    }
    g_clipMetrics = false;
    SetLogger(&TraceLog);
    SetAir(3); // back to the older sections' setting
    SetStrafe(false);
    SetFeetDiag(false);
    SetHoldPose(false);
    SetUpperMask(false);
}

// ================================================================ AIR STEER stage (2026-10-13)
// mm.airSteer through the real MMDriver, mm.air=hold, the three air databases. Runs after SectionAirClips, so every older section
// (all with mm.airSteer off, the default) keeps its numbers bit for bit.
void SectionAirSteer(int argc, char** argv, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    BeginSection("SectionAirSteer");
    (void)argc;
    using namespace airt;
    std::vector<std::string> paths;
    {
        std::string dir = argv[1];
        const size_t sl = dir.find_last_of("\\/");
        dir = sl == std::string::npos ? std::string() : dir.substr(0, sl + 1);
        for (const char* n : {"utpp_base.utppmm"})
            if (GetFileAttributesA((dir + n).c_str()) != INVALID_FILE_ATTRIBUTES)
                paths.push_back(dir + n);
    }
    Check("air steer: database utpp_base present", paths.size() == 1, std::to_string(paths.size()));
    g_fp.toe[0] = g_fp.toe[1] = -1;
    g_fp.parents.assign(aParents.begin(), aParents.end());
    for (size_t b = 0; b < aHashes.size(); ++b)
    {
        if (aHashes[b] == utpp::mm::integ::CNameHash("LeftToeBase"))
            g_fp.toe[0] = int32_t(b);
        if (aHashes[b] == utpp::mm::integ::CNameHash("RightToeBase"))
            g_fp.toe[1] = int32_t(b);
    }
    SetLogger(&AirLog);
    for (const std::string& path : paths)
    {
        std::string name = path.substr(path.find_last_of("\\/") + 1);
        name = name.substr(0, name.rfind('.'));
        const std::wstring wp(path.begin(), path.end());
        // ---- AIR STEER: jump with a 90 deg stick change at the apex, the game applying the body-yaw command in
        // the air (CET executor). On: the body yaw follows velocity with <= 35 deg lead toward the stick, still no turn intent / turn
        // clips / searches (hold criteria), one clean hand-back, the ground body turn finishes the turn after the landing. Off
        // (default): the executor sees no active body turn in the air (body frozen as before); toggling on / off leaves no state
        // (published pose bit-identical to a fresh off run); on without a target (standing jump) = off bit for bit.
        {
            const AirSc steerJump{.name = "jump_stick_change_steer", .stickTurn = 90.0f, .steer = true};
            const AirSc steerStand{.name = "jump_stand_steer", .speed = 0.0f, .stick = 0.0f, .vz0 = 5.5f, .steer = true};
            const AirSc warmS{.name = "warm_up", .kind = kSlide, .pre = 2.0f, .post = 0.5f};
            const auto freshS = [&](bool aSteer) {
                const bool ok = LoadDatabase(wp, name);
                OnArmed(aRig, aHashes, aParents);
                SetEnabled(true);
                ConfigureShippedC1a();
                SetFeetDiag(true);
                SetUpperMaskStates(7);
                SetStrafe(true);
                SetAir(2);
                SetHoldPose(true); // the game defaults (mm.holdPose / upper mask on): the jump gate is held, g_effGate 0
                SetUpperMask(true);
                SetAirSteer(aSteer);
                (void)RunAir(warmS, 60.0f); // same driver / controller state before every compared run (as the air off check)
                return ok;
            };
            bool steerOk = true, steerHold = true, steerHand = true;
            std::string steerLines;
            bool loadedS = true;
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                loadedS = freshS(true) && loadedS;
                const AirRes r = RunAir(steerJump, hz);
                std::printf("[INFO] air steer [%s] %s %.0f Hz: %s | body in the air: rate max %.1f deg/s, turn %.1f deg, fed %d/%d; "
                            "err touch %.1f end %.1f deg%s\n",
                            name.c_str(), steerJump.name.c_str(), hz, Line(r).c_str(), r.bodyRateMaxAir, r.bodyTurnAir, r.steerFedAir,
                            r.airFrames, r.errTouch, r.errEnd, r.picks.empty() ? "" : (" | owned picks:" + r.picks).c_str());
                // actual velocity turns at 90 deg/s and the body may lead it by <= 35 deg. The body continues the
                // ground turn after touching down, with no air matcher turns or searches.
                const bool ok = r.bodyRateMaxAir <= 200.5f && r.bodyTurnAir >= 10.0f && r.bodyVelGapMaxAir <= 35.5f &&
                                r.errTouch >= 0.0f && r.errTouch < 80.0f &&
                                r.errEnd >= 0.0f && r.errEnd <= 3.0f && r.steerFedAir > 0;
                const bool ho = HoldAirOk(r, 1.5f / hz) && r.episodes == 1 && r.takeoffs == 1 && r.turnPicksAir == 0;
                const bool hb = HandBackOk(r);
                steerOk = steerOk && ok;
                steerHold = steerHold && ho;
                steerHand = steerHand && hb;
                if (!ok || !ho || !hb)
                    steerLines += " @" + std::to_string(int(hz)) + (ok ? "" : ":follow") + (ho ? "" : ":hold") + (hb ? "" : ":handback");
            }
            Check("air steer [" + name + "] jump with stick change, mm.airSteer on, 30 / 60 / 144 Hz: body follows capsule velocity "
                                         "with <= 35 deg lead and <= 200 deg/s, ground turn completes after landing",
                  loadedS && steerOk, steerLines);
            // Negative control: turn the stick by 150 degrees at the apex, but turn the actual velocity
            // only 15 degrees/second. The previous stick-only steering spun the body long before the capsule.
            bool adverseOk = true;
            std::string adverseLines;
            const AirSc adverse{.name = "jump_stick_150_velocity_slow", .stickTurn = 150.0f,
                                .airVelTurnRate = 15.0f, .steer = true};
            for (float hz : {30.0f, 60.0f, 144.0f})
            {
                // High-volume, per-feed foot diagnostics are opt-in. Preserve quiet, order-identical
                // stdout in the ordinary full suite while retaining a deterministic repro on demand.
                char footTrace[8] = {};
                g_airFootPeakTrace = name == "utpp_base" && hz == 60.0f &&
                                     GetEnvironmentVariableA("UTPP_AIR_FOOT_TRACE", footTrace, sizeof footTrace) > 0 && footTrace[0] == '1';
                g_airFootPeakTouchTime = -1.0f;
                const bool ld = freshS(true);
                const AirRes a = RunAir(adverse, hz);
                g_airFootPeakTrace = false;
                const bool fedOk = a.steerFedAir > 0;
                const bool gapOk = a.bodyVelGapMaxAfterApex <= 35.5f;
                const bool rateOk = a.bodyRateMaxAir <= 200.5f;
                const bool holdOk = HoldAirOk(a, 1.5f / hz);
                const bool handOk = HandBackOk(a);
                const bool endOk = a.errEnd >= 0.0f && a.errEnd <= 3.0f;
                const bool ok = ld && fedOk && gapOk && rateOk && holdOk && handOk && endOk;
                adverseOk = adverseOk && ok;
                if (!ok)
                    adverseLines += " @" + std::to_string(int(hz)) + " gap=" + std::to_string(a.bodyVelGapMaxAfterApex) +
                                    " maxRate=" + std::to_string(a.bodyRateMaxAir) + " end=" + std::to_string(a.errEnd) +
                                    " corrPeak@=" + std::to_string(a.corrPeakAt) + " toePeak@=" + std::to_string(a.toePeakAt) +
                                    " toePeakCorr=" + std::to_string(a.corrAtToePeak) + " c/lock=" + std::to_string(a.contactAtToePeak) +
                                    "/" + std::to_string(a.lockAtToePeak) + " speed=" + std::to_string(a.speedAtToePeak) +
                                    " facingGap=" + std::to_string(a.facingAtToePeak) + " picks=" + std::to_string(a.picksAtToePeak) +
                                    " loaded/fed/gap/rate/hold/hand/end=" + std::to_string(int(ld)) + "/" + std::to_string(int(fedOk)) + "/" +
                                    std::to_string(int(gapOk)) + "/" + std::to_string(int(rateOk)) + "/" + std::to_string(int(holdOk)) +
                                    "/" + std::to_string(int(handOk)) + "/" + std::to_string(int(endOk)) + " | " + Line(a);
            }
            Check("air steer [" + name + "] adverse 150 deg stick / 15 deg/s velocity (30/60/144 Hz): no airborne body spin "
                                         "more than 35 deg away from actual travel heading, clean base landing handback",
                  adverseOk, adverseLines);
            // Optional diagnostic-only A/B sweep. It does not add checks or change any production defaults.
            // Every variant is reloaded/warmed independently; restore the baseline for subsequent AIR tests.
            char sweepEnv[8] = {};
            if (name == "utpp_base" && GetEnvironmentVariableA("UTPP_AIR_FOOT_SWEEP", sweepEnv, sizeof sweepEnv) > 0 &&
                sweepEnv[0] == '1')
            {
                const auto sweep = [&](const char* label, int replantMask, bool catchup) {
                    const bool loadedSweep = freshS(true);
                    SetFootReplant(replantMask);
                    SetOffsetCatchup(catchup);
                    const AirRes s = RunAir(adverse, 60.0f);
                    std::printf("[FOOTSWEEP] %s loaded=%d replant=%d catchup=%d corrAfter=%.2fcm toeAfter=%.3fm/s "
                                "corrPeak@=%.6f toePeak@=%.6f picksAfter=%d handbacks=%d resets=%d endClip=%s | %s\n",
                                label, loadedSweep ? 1 : 0, FootReplantMask(), catchup ? 1 : 0, s.corrAfter, s.toeAfter,
                                s.corrPeakAt, s.toePeakAt, s.picksAfter, s.handbacks, s.resets, s.endClip.c_str(), Line(s).c_str());
                };
                const int savedReplant = FootReplantMask();
                sweep("baseline-shipped-catchup", savedReplant, true);
                sweep("stand-replant", savedReplant | 1, true);
                sweep("catchup-off", savedReplant, false);
                SetFootReplant(savedReplant);
                SetOffsetCatchup(true);
                (void)freshS(true);
            }
            Check("air steer [" + name + "] jump with stick change, mm.airSteer on: hold criteria in the air (0 searches / transitions / "
                                         "turn picks, no body-turn input, no locked feet), 0 turn clips picked while airborne",
                  steerHold, steerLines);
            Check("air steer [" + name + "] jump with stick change, mm.airSteer on: one clean hand-back at the touchdown", steerHand, steerLines);
            // off: body frozen in the air (the executor sees no active body turn); toggle leaves no state; no target = off
            bool loaded = freshS(false);
            const AirRes off = RunAir(steerJump, 60.0f);
            loaded = freshS(true) && loaded;
            SetAirSteer(false);
            const AirRes tog = RunAir(steerJump, 60.0f);
            loaded = freshS(false) && loaded;
            const AirRes stOff = RunAir(steerStand, 60.0f);
            loaded = freshS(true) && loaded;
            const AirRes stOn = RunAir(steerStand, 60.0f);
            std::printf("[INFO] air steer [%s] off: body turn in the air %.1f deg, fed %d; toggled: %.1f deg, pose %s; standing on: fed %d, "
                        "pose %s\n", name.c_str(), off.bodyTurnAir, off.steerFedAir, tog.bodyTurnAir, tog.hashes == off.hashes ? "same" : "DIFF",
                        stOn.steerFedAir, stOn.hashes == stOff.hashes ? "same" : "DIFF");
            Check("air steer [" + name + "] mm.airSteer off: body yaw frozen in the air (no active body turn), on -> off toggle bit-identical "
                                         "to a fresh off run, on without a stick target (standing jump) bit-identical to off",
                  loaded && off.bodyTurnAir == 0.0f && off.steerFedAir == 0 && tog.hashes == off.hashes && !off.hashes.empty() &&
                      stOn.hashes == stOff.hashes && !stOn.hashes.empty() && stOn.steerFedAir == 0,
                  Line(off));
            // ---- AIR STEER repair (review): (a) the setting switched off in the air: the body stops at once (no steering after the
            // toggle, airSteerFed cleared), the landing / hand-back stay clean and the ground turn still finishes (follow-law rate
            // carried over to the ground law); (b) without the hold pose (UTPPSetMMHoldPose off) the air area does not own the
            // body, the setting has no effect (never armed, airSteerBlocked 0, body frozen); (c) mm.air=auto (clips, the game default with .airz) + steering: the body
            // follows, only forced Jump clips / no turn clip in the air, a Land clip before the touchdown, one clean hand-back
            {
                const AirSc steerMid{.name = "jump_stick_change_steer_off_mid", .stickTurn = 90.0f, .steer = true, .steerOffMid = true};
                bool midOk = true, blkOk = true, clipOk = true;
                std::string midLines;
                for (float hz : {30.0f, 60.0f, 144.0f})
                {
                    bool ld = freshS(true);
                    const AirRes m = RunAir(steerMid, hz);
                    const bool fedClear = Field(Status(), "airSteerFed") < 0.5f;
                    std::printf("[INFO] air steer [%s] off in the air %.0f Hz: toggled %d, turn %.1f deg (after the toggle %.1f), err end "
                                "%.1f deg, fed after %d | %s\n",
                                name.c_str(), hz, m.steerOffDone ? 1 : 0, m.bodyTurnAir, m.bodyTurnAfterOff, m.errEnd, fedClear ? 0 : 1,
                                Line(m).c_str());
                    const bool ok1 = ld && m.steerOffDone && m.steerBlocked == 0 && m.bodyTurnAfterOff == 0.0f && m.bodyTurnAir >= 10.0f && fedClear &&
                                     HoldAirOk(m, 1.5f / hz) && m.turnPicksAir == 0 && HandBackOk(m) && m.errEnd >= 0.0f && m.errEnd <= 3.0f;
                    midOk = midOk && ok1;
                    // (b) hold pose off
                    ld = freshS(true);
                    SetHoldPose(false);
                    const AirRes b = RunAir(steerJump, hz);
                    std::printf("[INFO] air steer [%s] hold pose off %.0f Hz: turn %.1f deg, fed %d, blocked %d | %s\n", name.c_str(), hz,
                                b.bodyTurnAir, b.steerFedAir, b.steerBlocked, Line(b).c_str());
                    const bool ok2 = ld && b.bodyTurnAir == 0.0f && b.steerFedAir == 0 && b.steerBlocked == 0 && b.ownedAir == 0;
                    blkOk = blkOk && ok2;
                    SetHoldPose(true);
                    // (c) mm.air=auto -> clips
                    ld = freshS(true);
                    SetAir(0);
                    const bool clipsMode = Status().find(" air=clips airReason=auto:table") != std::string::npos;
                    g_clipMetrics = true;
                    g_clipEvs.clear();
                    AirRes c = RunAir(steerJump, hz);
                    c.clipEvs = g_clipEvs;
                    g_clipMetrics = false;
                    bool land = false, onlyJump = c.picks.find("Jump_") != std::string::npos;
                    for (const AirRes::ClipEv& e : c.clipEvs)
                        land = land || (e.action == "land" && e.clip.find("_Land_") != std::string::npos && c.touchAt >= 0.0f && e.t < c.touchAt);
                    {
                        size_t a = 0;
                        const std::string& p = c.picks;
                        while (a < p.size())
                        {
                            const size_t e = p.find(' ', a + 1);
                            const std::string tk = p.substr(a + 1, e == std::string::npos ? std::string::npos : e - a - 1);
                            if (!tk.empty() && tk.find("Jump_") == std::string::npos)
                                onlyJump = false;
                            if (e == std::string::npos)
                                break;
                            a = e;
                        }
                    }
                    std::printf("[INFO] air steer [%s] clips %.0f Hz: mode %s, rate max %.1f deg/s, turn %.1f deg, err touch %.1f end %.1f, "
                                "land clip %d, only Jump clips owned %d | %s\n",
                                name.c_str(), hz, clipsMode ? "clips" : "NOT-clips", c.bodyRateMaxAir, c.bodyTurnAir, c.errTouch, c.errEnd,
                                land ? 1 : 0, onlyJump ? 1 : 0, Line(c).c_str());
                    const bool ok3 = ld && clipsMode && c.bodyRateMaxAir <= 200.5f && c.bodyTurnAir >= 10.0f &&
                                     c.bodyVelGapMaxAir <= 35.5f && c.steerFedAir > 0 && c.steerBlocked == 0 &&
                                     c.turnPicksAir == 0 && c.searchesOwned == 0 && c.turnPicksOwned == 0 && c.locksAir == 0 && onlyJump &&
                                     land && c.episodes == 1 && c.takeoffs == 1 && HandBackOk(c) && c.errEnd >= 0.0f && c.errEnd <= 3.0f;
                    clipOk = clipOk && ok3;
                    SetAir(2);
                    if (!ok1 || !ok2 || !ok3)
                        midLines += " @" + std::to_string(int(hz)) + (ok1 ? "" : ":offMid") + (ok2 ? "" : ":holdOff") + (ok3 ? "" : ":clips");
                }
                Check("air steer [" + name + "] mm.airSteer switched off in the air (30 / 60 / 144 Hz): the body stops turning at once, "
                                             "airSteerFed cleared, hold criteria, one clean hand-back, ground turn done after the landing",
                      midOk, midLines);
                Check("air steer [" + name + "] mm.airSteer on with the hold pose off: the air area does not own the body, no effect in "
                                             "the air (never armed, airSteerBlocked 0, body frozen)",
                      blkOk, midLines);
                Check("air steer [" + name + "] mm.airSteer on with mm.air=auto (air clips): body follows <= 200 deg/s, only forced Jump "
                                             "clips in the air (no turn clip / search / locked foot), a Land clip before the touchdown, one "
                                             "clean hand-back, ground turn done",
                      clipOk, midLines);
            }
            freshS(false);
        }
    }
    SetAirSteer(false);
    SetLogger(&TraceLog);
    SetAir(3); // back to the older sections' setting
    SetStrafe(false);
    SetFeetDiag(false);
    SetHoldPose(false);
    SetUpperMask(false);
}

namespace
{
// b5 testorder: the suite's base settings between sections (main: once before the first database load; BeginSection: again
// before every section, so a section starts from the same settings alone (UTPP_LOCO_ONLY*) and inside the full suite)
void ApplySuiteBaseSettings(bool aPrint)
{
    // loco C0 / C1a / C1b switches off outside SectionC0C1a / SectionC1b (the older sections keep their numbers); diagnostic
    // UTPP_LOCO_C1A_ALL=1 keeps the shipped defaults (crouch idle exclusion, feet diagnostics, replant crouch) in every section
    {
        char ea[4] = {};
        const bool all = GetEnvironmentVariableA("UTPP_LOCO_C1A_ALL", ea, sizeof ea) > 0 && ea[0] == '1';
        SetCrouchIdleExclude(all);
        SetIdleHold(all); // stage standstill idle: off outside SectionIdleHold as well
        SetFeetDiag(all);
        g_replantMain = all ? 2 : 0;
        SetFootReplant(g_replantMain);
        SetKneeFix(all ? 7 : 0); // batch 2 knee fix: off outside SectionKnee (the older sections keep their numbers); batch 5: shipped 7
        // B2 strafe S1: off outside SectionStrafe (the older sections keep their numbers); diagnostic UTPP_LOCO_STRAFE_ALL=1
        // keeps the shipped default (strafe on) in every section
        char es[4] = {};
        const bool strafeAll = GetEnvironmentVariableA("UTPP_LOCO_STRAFE_ALL", es, sizeof es) > 0 && es[0] == '1';
        SetStrafe(strafeAll);
        if (strafeAll && aPrint)
            std::printf("[INFO] UTPP_LOCO_STRAFE_ALL=1: strafe ON (shipped default) in every section\n");
        // AIR HOLD stage: mm.air off outside SectionAir (the older sections keep their numbers: their jump / knockdown steps feed
        // LocomotionDetailed 18 without an air feed); diagnostic UTPP_LOCO_AIR_ALL=1 keeps the shipped default (auto) everywhere
        char eair2[4] = {};
        const bool airAll = GetEnvironmentVariableA("UTPP_LOCO_AIR_ALL", eair2, sizeof eair2) > 0 && eair2[0] == '1';
        SetAir(airAll ? 0 : 3);
        if (airAll && aPrint)
            std::printf("[INFO] UTPP_LOCO_AIR_ALL=1: mm.air auto (shipped default) in every section\n");
    }
}

// b5 testorder: the second half of the base settings (main: after the first database load and SetEnabled, as before)
void ApplySuiteSwitchesAfterLoad()
{
    SetCapsuleSync(false); // 0.5.9 switches off outside Section059 (the older sections keep their 0.5.8 numbers)
    SetOffsetCatchup(false);
    SetTrajCal(false);
    SetMeleeGate(false); // 0.6.0 switches off outside Section060
    SetMountCut(false);
    SetStanceClipMinSpeed(0.0f);
    SetUpperMask(false); // 0.6.2 switches off outside Section062
    SetHoldPose(false);
    SetTurnRate(false); // 0.6.3 switch off outside Section063
    SetCommandDtFromFeed(true); // the simulation runs faster than real time
    SetLogger(&TraceLog);
    SetTrace(true);
}

// b5 testorder: every Section* starts here. Marker line "==== SECTION <name>" (order comparison: a section run alone through
// UTPP_LOCO_ONLY* must print the same [INFO] / [PASS] lines as inside the full suite), the test-side state of the previous
// sections cleared, the suite base settings, then the driver's session state (LoadDatabase resets it again).
void BeginSection(const char* aName)
{
    std::printf("==== SECTION %s\n", aName);
    g_trans.clear();
    g_transFrame.clear();
    g_transTime.clear();
    g_transW.clear();
    g_transV.clear();
    g_transMv.clear();
    g_clock = 0.0f;
    g_infoLines.clear();
    g_feetLines.clear();
    g_footstepLines.clear();
    g_exclLines.clear();
    g_mainTraceLines = 0;
    g_turnSeg.clear();
    g_turnSegNames.clear();
    g_game059 = false;
    g_feedProbe = nullptr;
    strafet::g_chestProbe = nullptr;
    airt::g_mmairLines = 0;
    airt::g_clipMetrics = false;
    airt::g_curEvT = -1.0f;
    airt::g_clipEvs.clear();
    airt::g_fallLoopNumFrames = -1;
    airt::g_mmairSample.clear();
    travt::g_gateTravLines = travt::g_travOnLines = travt::g_travOffLines = 0;
    travt::g_ownsReturn = false;
    ApplySuiteBaseSettings(false);
    SetEnabled(true);
    ApplySuiteSwitchesAfterLoad();
    ResetSessionState();
    {
        // b5 testorder repair: settings the base above does not restore (air steer, root rate / steering configs, knee fix, chest
        // align, transition / cost cap, ...) must be restored by the section that changed them. Every section start compares these
        // status fields with the first section start: a leak FAILS the full suite (no line when equal, so the alone-vs-full
        // comparison and the check count are unchanged).
        static const char* const kKeys[] = {"blendTime", "footLock", "bodyTurn", "offHalflife", "offLimit", "accelMatch", "rootRate",
                                            "rootRateCfg", "stopSync", "slowWalk", "steerFollow", "steerCfg", "dwell", "curation",
                                            "stanceGuard", "stanceSmooth", "turnCommit", "speedCap", "costCap", "turnSettle",
                                            "capsuleSync", "offCatchup", "trajCal", "meleeGate", "mountCut", "upperMask", "holdPose",
                                            "turnRate", "airSteer", "strafe", "chestAlign", "kneeFix", "idleHold", "feetDiag",
                                            "replant", "crouchExcl", "tagFilter", "allowedStates", "swapReset"};
        const std::string st = Status();
        std::string fp;
        for (const char* k : kKeys)
        {
            const std::string key = std::string(" ") + k + "=";
            const size_t p = st.find(key);
            std::string v = "-";
            if (p != std::string::npos)
            {
                const size_t b = p + key.size();
                const size_t e = st[b] == '"' ? st.find('"', b + 1) + 1 : st.find(' ', b);
                v = st.substr(b, e == std::string::npos ? std::string::npos : e - b);
            }
            fp += std::string(k) + "=" + v + " ";
        }
        static std::string s_first, s_firstName;
        if (s_first.empty())
        {
            s_first = fp;
            s_firstName = aName;
        }
        else if (fp != s_first)
        {
            std::string diff;
            for (size_t a = 0, b = 0; a < fp.size() && b < s_first.size();)
            {
                const size_t ea = fp.find(' ', a), eb = s_first.find(' ', b);
                if (fp.compare(a, ea - a, s_first, b, eb - b) != 0)
                    diff += s_first.substr(b, eb - b) + " -> " + fp.substr(a, ea - a) + "; ";
                a = ea + 1;
                b = eb + 1;
            }
            Check(std::string("testorder: section ") + aName + " starts with the settings of the first section (" + s_firstName +
                      "): a previous section left a setting changed",
                  false, diff);
        }
    }
    // diagnostic UTPP_LOCO_SECTION_STATUS=1: the driver status at the section start (settings + session state; diff a section's
    // line between an alone run and the full suite to find what still carries over)
    char ess[4] = {};
    if (GetEnvironmentVariableA("UTPP_LOCO_SECTION_STATUS", ess, sizeof ess) > 0 && ess[0] == '1')
        std::printf("SECTION_STATUS %s %s\n", aName, Status().c_str());
}

} // namespace

namespace
{
void RunUnit(const char* aName, void (*aFn)(int, char**, const std::vector<uint64_t>&, const std::vector<int16_t>&, void*), int argc, char** argv,
             const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents, void* aRig)
{
    if (!UnitOn(aName))
        return;
    const DWORD t0 = GetTickCount();
    aFn(argc, argv, aHashes, aParents, aRig);
    if (TimingOn())
        std::printf("[TIME] unit %s: %.1f s\n", aName, double(GetTickCount() - t0) / 1000.0);
}
} // namespace

int main(int argc, char** argv)
{
    // "--units=a,b" is removed from argv (the sections read the extra database arguments from argv[2..])
    for (int i = 1; i < argc;)
    {
        if (std::strncmp(argv[i], "--units=", 8) == 0)
        {
            g_unitFilter = true;
            const std::string list = argv[i] + 8;
            for (size_t a = 0; a <= list.size();)
            {
                size_t b = list.find(',', a);
                if (b == std::string::npos)
                    b = list.size();
                if (b > a)
                    g_units.insert(list.substr(a, b - a));
                a = b + 1;
            }
            for (int j = i; j + 1 < argc; ++j)
                argv[j] = argv[j + 1];
            --argc;
        }
        else
            ++i;
    }
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_loco_test <database.utppmm> [extra.utppmm ...] [--units=main,057,...]\n");
        return 2;
    }
    // ---- unit: stick filter
    if (UnitOn("main")) // the two unit checks run once (shard "main")
    {
        utpp::mm::steer::StickFilter f;
        float x = 0, y = 0.16f, tgt = 10.0f;
        bool have = true;
        const bool h1 = f.Update(x, y, have, tgt, 1 / 60.0f);
        x = 0;
        y = 0.12f;
        have = true;
        const bool h2 = f.Update(x, y, have, tgt, 1 / 60.0f);
        const bool lifted = h2 && std::fabs(y - 0.16f) < 1e-6f;
        x = 0;
        y = 0.0f;
        have = false;
        float tgt2 = 0.0f;
        const bool h3 = f.Update(x, y, have, tgt2, 1 / 60.0f);
        const bool bridged = h3 && have && tgt2 == 10.0f && y > 0.15f;
        bool h4 = true;
        for (int i = 0; i < 10; ++i)
        {
            x = 0;
            y = 0;
            have = false;
            h4 = f.Update(x, y, have, tgt2, 1 / 60.0f);
        }
        Check("stick filter: held above 0.05, lifted to 0.16, 0.12 s release bridged with the last target, then released",
              h1 && lifted && bridged && !h4 && y == 0.0f && !have);
    }
    // ---- unit: start rules
    if (UnitOn("main"))
    {
        using namespace utpp::mm::steer;
        CurationInput ci;
        ci.input = true;
        ci.haveTarget = true;
        ci.absErr = 170.0f * kDegT;
        ci.capsule = 0.2f;
        ci.moving = true;
        ci.sinceInput = 0.1f;
        CurationConfig on, off;
        off.startRules = false;
        const uint32_t a = ExcludedClasses(ci, on), b = ExcludedClasses(ci, off);
        ci.capsule = 0.5f;
        ci.sinceInput = 3.0f;
        const uint32_t c = ExcludedClasses(ci, on), d = ExcludedClasses(ci, off);
        ci.speedClass = 2;
        const uint32_t e = ExcludedClasses(ci, on);
        Check("start rules: standstill pivots / turns out, slow held walk without stops, sprint starts only when sprinting; off = 0.5.2",
              (a & kPivotMask) == kPivotMask && (b & kPivotMask) == 0 && (c & Bit(kClassStop)) && !(d & Bit(kClassStop)) &&
                  (e & Bit(kClassStartSprint)) && ClassifyClip("M_Neutral_Sprint_Start_F_Lfoot") == kClassStartSprint &&
                  ClassifyClip("M_Neutral_Walk_Stop_F_Lfoot") == kClassStop);
    }

    ApplySuiteBaseSettings(true);
    const std::wstring path(argv[1], argv[1] + std::strlen(argv[1]));
    if (!LoadDatabase(path, "test"))
    {
        Check("database loaded", false, argv[1]);
        std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
        return 1;
    }
    std::printf("[INFO] %s\n", SpeedCalibrationSummary().c_str());
    utpp::mm::Database db;
    std::string err;
    db.LoadFromFile(argv[1], kExpectedRigHash, err);
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(utpp::mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    utpp::g_hot.armed.store(true);
    OnArmed(&rig, hashes, parents);
    SetEnabled(true);
    ApplySuiteSwitchesAfterLoad();
    {
        // loco C1b diagnostic: UTPP_LOCO_ONLYC1B=1 runs only the C1b section
        char eb[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYC1B", eb, sizeof eb) > 0 && eb[0] == '1')
        {
            SectionC1b(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // AIR HOLD stage diagnostic: UTPP_LOCO_ONLYAIR=1 runs only the air section
        char eair[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYAIR", eair, sizeof eair) > 0 && eair[0] == '1')
        {
            SectionAir(argc, argv, hashes, parents, &rig);
            SectionAirClips(argc, argv, hashes, parents, &rig); // AIR CLIPS stage
            SectionAirSteer(argc, argv, hashes, parents, &rig); // AIR STEER stage
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // B2 strafe diagnostic: UTPP_LOCO_ONLYSTRAFE=1 runs only the strafe section
        char es[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYSTRAFE", es, sizeof es) > 0 && es[0] == '1')
        {
            SectionStrafe(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // batch 2 knee fix diagnostic: UTPP_LOCO_ONLYKNEE=1 runs only the knee fix section
        char ek[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYKNEE", ek, sizeof ek) > 0 && ek[0] == '1')
        {
            SectionKnee(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // stage standstill idle diagnostic: UTPP_LOCO_ONLYIDLE=1 runs only the idle hold section
        char ei[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYIDLE", ei, sizeof ei) > 0 && ei[0] == '1')
        {
            SectionIdleHold(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // loco C0 / C1a diagnostic: UTPP_LOCO_ONLYC1A=1 runs only the C0 / C1a section
        char ec[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYC1A", ec, sizeof ec) > 0 && ec[0] == '1')
        {
            SectionC0C1a(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // vehicle batch W1: UTPP_LOCO_ONLYVEHEXIT=1 runs only the vehicle exit section (tests/vehexit.cmake mutation builds)
        char evx[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLYVEHEXIT", evx, sizeof evx) > 0 && evx[0] == '1')
        {
            SectionVehExit(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // 0.6.3 diagnostic: UTPP_LOCO_ONLY063=1 runs only the 0.6.3 section
        char e63[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY063", e63, sizeof e63) > 0 && e63[0] == '1')
        {
            Section063(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // 0.6.2 diagnostic: UTPP_LOCO_ONLY062=1 runs only the 0.6.2 section
        char e62[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY062", e62, sizeof e62) > 0 && e62[0] == '1')
        {
            Section062(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // 0.6.0 diagnostic: UTPP_LOCO_ONLY060=1 runs only the 0.6.0 section
        char e6[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY060", e6, sizeof e6) > 0 && e6[0] == '1')
        {
            Section060(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        // 0.5.7 diagnostic: UTPP_LOCO_ONLY057=1 runs only the 0.5.7 section
        char e7[4] = {};
        char e8[4] = {};
        char e9[4] = {};
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY059", e9, sizeof e9) > 0 && e9[0] == '1')
        {
            Section059(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY058", e8, sizeof e8) > 0 && e8[0] == '1')
        {
            Section058(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
        if (GetEnvironmentVariableA("UTPP_LOCO_ONLY057", e7, sizeof e7) > 0 && e7[0] == '1')
        {
            Section057(argc, argv, hashes, parents, &rig);
            Shutdown();
            std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
            return g_fail == 0 ? 0 : 1;
        }
    }

    const DWORD mainT0 = GetTickCount();
    if (UnitOn("main")) // unit "main": the inline scenarios up to the closing brace before Section057 (not re-indented)
    {
    // scenarios (camera -82 deg, stick angles relative to the camera as in the gamepad harness)
    const std::vector<Step> slowWalk = {{"ramp_in", 1.0f, 0.45f, 0.0f}, {"half_stick", 6.0f, 0.47f, -1.0f, 0.07f}, {"stop", 2.5f, 0.0f}};
    const std::vector<Step> flick = {{"accel", 1.5f, 1.0f}, {"cruise", 2.5f, 1.0f}, {"flick", 2.5f, 1.0f, -1.0f, 0.0f, 180.0f, 0.0f, 2},
                                     {"cruise_back", 1.5f, 1.0f}, {"stop", 2.5f, 0.0f}};
    const std::vector<Step> turnStart = {{"walk", 2.0f, 0.87f}, {"arc_right", 3.0f, 0.87f, -1.0f, 0.0f, 0.0f, -120.0f},
                                         {"stop", 2.5f, 0.0f}};
    const std::vector<Step> arcs = {{"accel", 2.0f, 1.0f}, {"arc_60", 4.0f, 1.0f, -1.0f, 0.0f, 0.0f, 60.0f},
                                    {"arc_90", 4.0f, 1.0f, -1.0f, 0.0f, 0.0f, 90.0f},
                                    {"arc_120", 3.0f, 1.0f, -1.0f, 0.0f, 0.0f, 120.0f}, {"stop", 2.5f, 0.0f}};

    // 0.5.4: modes 0.5.2 / 0.5.3 (speed bands off, reference only) / 0.5.4; the 0.5.3 checks below compare 0.5.2 with the
    // current state (every fix on = 0.5.4), so the 0.5.3 criteria must still hold with the speed bands
    std::map<std::string, std::map<std::string, Metrics>> res[3];
    for (int mode = 0; mode < 3; ++mode)
    {
        ConfigureMode(mode);
        res[mode]["slow"] = RunScenario(slowWalk, -160.0f);
        res[mode]["flick"] = RunScenario(flick, 150.0f);
        res[mode]["turnstart"] = RunScenario(turnStart, 98.0f);
        res[mode]["arcs"] = RunScenario(arcs, -82.0f);
        for (const auto& sc : res[mode])
            for (const auto& stp : sc.second)
                std::printf("[INFO] %s %s/%s: %s\n", kModeName[mode], sc.first.c_str(), stp.first.c_str(), stp.second.Line().c_str());
    }
    const auto& o = res[0];
    const auto& n = res[2];
    auto cls = [](const Metrics& m, const char* c) { auto it = m.classes.find(c); return it == m.classes.end() ? 0 : it->second; };

    // slow walk with the half stick
    const Metrics& sw0 = o.at("slow").at("half_stick");
    const Metrics& sw1 = n.at("slow").at("half_stick");
    Check("negative control 0.5.2: slow half-stick walk selects stop / start clips with the stick held",
          cls(sw0, "Stop") + cls(sw0, "Start") > 0, sw0.ClassesString());
    // Limits are set for the shipped database (motion clips only): one start clip is part of a slow half-stick walk there.
    Check("0.5.3 slow walk: no stop / arc / turn-in-place clips and at most one start clip while the stick is held",
          cls(sw1, "Stop") + cls(sw1, "Arc") + cls(sw1, "TurnInPlace") == 0 && cls(sw1, "Start") <= 1, sw1.ClassesString());
    Check("0.5.3 slow walk: <= 0.7 transitions/s and fewer than 0.5.2", sw1.transitions / sw1.seconds <= 0.7f && sw1.transitions < sw0.transitions,
          std::to_string(sw1.transitions) + " vs " + std::to_string(sw0.transitions));
    Check("0.5.3 slow walk: foot-lock correction not above 0.5.2 (INFO: data-limited, see report)", sw1.corrMax <= sw0.corrMax + 0.5f,
          std::to_string(sw1.corrMax) + " vs " + std::to_string(sw0.corrMax));

    // flick 180 while running
    const Metrics& fl0 = o.at("flick").at("flick");
    const Metrics& fl1 = n.at("flick").at("flick");
    Check("negative control 0.5.2: flick through the stick centre selects a stop clip", cls(fl0, "Stop") > 0, fl0.ClassesString());
    // Limit set for the shipped database (motion clips only): the offset stays saturated 0.57 s there.
    Check("0.5.3 flick: offset saturated <= 0.6 s", fl1.offSat <= 0.6f, std::to_string(fl1.offSat));
    {
        // 0.5.8: the flick's selection cost depends on the loop phase at the flick (0.5.8 starts / loop bands change the phase
        // reached in the chained run: Run_Turn_L_180 entered at f=24 cost 49.8 instead of f=21 cost 23.0 from the same loop
        // family). Phase sweep from a fresh state, cruise shortened by 0, 0.1 .. 0.5 s: highest / mean flick cost and stop clips,
        // 0.5.2 / 0.5.7 (0.5.8 switches off) / current. The sweep showed the single-run criterion "max cost below 0.5.2" to be
        // phase luck already in 0.5.7 (0.5.7 sweep max 70.4 mean 47.2 vs 0.5.2 max 53.7 mean 41.1 on a former test database, where
        // 0.5.2 picks a stop clip at every phase): the cost criterion is now "worst phase not above 0.5.7"; the single run is INFO.
        float sw[3] = {0.0f, 0.0f, 0.0f}, mean[3] = {0.0f, 0.0f, 0.0f}, sat[3] = {0.0f, 0.0f, 0.0f}, satMean[3] = {0.0f, 0.0f, 0.0f};
        int stops[3] = {0, 0, 0};
        for (int mi = 0; mi < 3; ++mi)
        {
            ConfigureMode(mi == 0 ? 0 : 2);
            if (mi == 1)
                Set058(false);
            for (int ph = 0; ph < 6; ++ph)
            {
                std::vector<Step> fl = flick;
                fl[1].seconds = 2.5f - 0.1f * float(ph);
                const auto rr = RunScenario(fl, 150.0f, true);
                sw[mi] = std::max(sw[mi], rr.at("flick").costMax);
                mean[mi] += rr.at("flick").costMax / 6.0f;
                sat[mi] = std::max(sat[mi], rr.at("flick").offSat);
                satMean[mi] += rr.at("flick").offSat / 6.0f;
                stops[mi] += cls(rr.at("flick"), "Stop");
            }
        }
        ConfigureMode(2);
        std::printf("[INFO] flick single run (chained): current cost %.1f vs 0.5.2 %.1f\n", fl1.costMax, fl0.costMax);
        Check("0.5.3 flick: no stop clip (chained run and 6 loop phases), worst-phase cost not above 0.5.7 (0.5.8 switches off)",
              cls(fl1, "Stop") == 0 && stops[2] == 0 && sw[2] <= sw[1] + 1e-3f,
              fl1.ClassesString() + " | sweep 0.5.2 max " + std::to_string(sw[0]) + " stops " + std::to_string(stops[0]) + " | 0.5.7 max " +
                  std::to_string(sw[1]) + " mean " + std::to_string(mean[1]) + " stops " + std::to_string(stops[1]) + " | current max " +
                  std::to_string(sw[2]) + " mean " + std::to_string(mean[2]));
        std::printf("[INFO] 0.5.8 flick phase sweep (6 phases): 0.5.2 max %.1f mean %.1f stops %d | 0.5.7 max %.1f mean %.1f stops %d | current max %.1f mean %.1f stops %d\n",
                    sw[0], mean[0], stops[0], sw[1], mean[1], stops[1], sw[2], mean[2], stops[2]);
        std::printf("[INFO] 0.5.8 flick phase sweep offset saturation: 0.5.2 max %.2f mean %.2f | 0.5.7 max %.2f mean %.2f | current max %.2f mean %.2f s%c",
                    sat[0], satMean[0], sat[1], satMean[1], sat[2], satMean[2], 10);
    }
    const Metrics& cr0 = o.at("flick").at("cruise");
    const Metrics& cr1 = n.at("flick").at("cruise");
    Check("negative control 0.5.2: straight run with the mixed database keeps the offset root far out (rate 0.80 on Neutral clips)",
          cr0.offMax >= 15.0f, std::to_string(cr0.offMax));
    // The 0.5.2 mode behaves well on the shipped database, so a comparison with it says nothing there: absolute limit instead.
    Check("0.5.3 cruise: steady offset root (last 1 s) at most 0.25 cm, not saturated",
          cr1.offLast <= 0.25f && cr1.offSat <= 0.05f,
          std::to_string(cr1.offLast) + " vs " + std::to_string(cr0.offLast) + " cm (max " + std::to_string(cr1.offMax) + ")");
    const Metrics& ac0 = o.at("flick").at("accel");
    const Metrics& ac1 = n.at("flick").at("accel");
    std::printf("[INFO] 0.5.3 accel from a 128 deg gap: %s vs 0.5.2 %s\n", ac1.ClassesString().c_str(), ac0.ClassesString().c_str());

    // invariant over every 0.5.3 step: no pivot / moving turn selected below 0.8 m/s capsule (in game 0.5.2:
    // Walk_Turn_R_180 at 0.25 m/s, a walk pivot at 0.05 m/s)
    int slowPivots = 0;
    for (const auto& sc : n)
        for (const auto& stp : sc.second)
            slowPivots += stp.second.pivotTurnSlow;
    Check("0.5.3: no pivot / moving-turn clip selected below 0.8 m/s in any scenario", slowPivots == 0, std::to_string(slowPivots));

    // arcs must not get worse
    for (const char* a : {"arc_60", "arc_90", "arc_120"})
    {
        const Metrics& a0 = o.at("arcs").at(a);
        const Metrics& a1 = n.at("arcs").at(a);
        Check(std::string("arcs ") + a + ": not worse (transitions, offset saturation, cost max + 1)",
              a1.transitions <= a0.transitions + 1 && a1.offSat <= std::max(a0.offSat, 0.05f) && a1.costMax <= a0.costMax + 1.0f,
              a1.Line() + " | 0.5.2 " + a0.Line());
    }
    // ================================================================ 0.5.4 (play recording, 0.5.2 trace)
    {
        using namespace utpp::mm::steer;
        CurationInput ci;
        ci.input = true;
        ci.haveTarget = true;
        ci.animLed = true;
        ci.absErr = 170.0f * kDegT;
        ci.capsule = 3.4f;
        ci.moving = true;
        ci.sinceInput = 3.0f;
        ci.speedClass = 2;
        ci.speedBand = 2;
        CurationConfig off, on;
        on.speedBands = true;
        const uint32_t a = ExcludedClasses(ci, off), b = ExcludedClasses(ci, on), bc = ExcludedCurrentClasses(ci, on);
        const bool refaceOk = SpeedVariantClass("M_Neutral_Sprint_Reface_Start_F_L_090", kClassReface) == kClassRefaceSprint &&
                              SpeedVariantClass("M_Neutral_Walk_Reface_Start_B_L_180", kClassReface) == kClassRefaceWalk &&
                              SpeedVariantClass("M_Neutral_Run_Loop_F", kClassNone) == kClassNone && (b & Bit(kClassRefaceWalk)) &&
                              (b & Bit(kClassRefaceSprint)) && !(b & Bit(kClassRefaceRun)) && !(a & kRefaceVariantMask) &&
                              WithVariants(Bit(kClassReface)) == (Bit(kClassReface) | kRefaceVariantMask);
        ci.speedBand = 1;
        ci.speedClass = 1;
        ci.capsule = 1.5f;
        const uint32_t c = ExcludedClasses(ci, on);
        ci.speedBand = 3;
        ci.speedClass = 3;
        ci.capsule = 8.0f;
        ci.pivotFallback = true;
        const uint32_t d = ExcludedClasses(ci, on);
        ci.input = false;
        const uint32_t f = ExcludedClasses(ci, on);
        Check("0.5.4 unit: idle class; run band excludes walk pivots / turns (0.5.3 allowed them), walk band keeps walk ones, "
              "sprint without sprint pivots falls back to run pivots only; idle out (also as current) while held and moving",
              ClassifyClip("X_Crouch_Idle_Loop_CrouchIdle02LookingAround") == kClassIdle &&
                  ClassifyClip("M_Neutral_Stand_Idle_Loop") == kClassIdle && ClassifyClip("X_Stand_Idle_Var_Female1A04Look") == kClassIdle &&
                  ClassifyClip("M_Neutral_Crouch_Idle_Turn_090_L") == kClassTurnInPlace &&
                  ClassifyClip("X_Walk_Pivot_L_180_Female1B15WalkTurnAround180") == kClassPivotWalk &&
                  ClassifyClip("M_Neutral_Walk_Turn_R_180_Rfoot") == kClassPivotWalk && !(a & Bit(kClassPivotWalk)) &&
                  !(a & Bit(kClassIdle)) && (b & Bit(kClassPivotWalk)) && (b & Bit(kClassTurnWalk)) && !(b & Bit(kClassPivotRun)) &&
                  !(b & Bit(kClassTurnRun)) && (b & Bit(kClassIdle)) && (bc & Bit(kClassIdle)) &&
                  !(c & (Bit(kClassPivotWalk) | Bit(kClassTurnWalk))) && (c & Bit(kClassPivotRun)) && !(d & Bit(kClassPivotRun)) &&
                  !(d & Bit(kClassPivotSprint)) && (d & Bit(kClassTurnRun)) && (d & Bit(kClassPivotWalk)) && !(f & Bit(kClassIdle)) &&
                  refaceOk);
        // band tracker: hysteresis, down-hold, no down-step during an animation-led turn
        utpp::mm::BandEdges ed;
        ed.walkRun = 3.0f;
        ed.runSprint = 6.0f;
        utpp::mm::SpeedBandTracker t;
        const float dt = 1.0f / 60.0f;
        const int b0 = t.Update(2.0f, true, false, dt, ed);
        const int b1 = t.Update(3.1f, true, false, dt, ed); // inside the hysteresis: stays walk
        const int b2 = t.Update(3.3f, true, false, dt, ed); // above 3.24: run
        int b3 = 0, b4 = 0, b5 = 0;
        for (int i = 0; i < 30; ++i)
            b3 = t.Update(2.9f, true, false, dt, ed); // inside the hysteresis: stays run
        for (int i = 0; i < 60; ++i)
            b4 = t.Update(1.0f, true, true, dt, ed); // decelerating through a turn: held
        for (int i = 0; i < 15; ++i)
            b5 = t.Update(1.0f, true, false, dt, ed); // 0.25 s < downHold
        int b6 = 0;
        for (int i = 0; i < 10; ++i)
            b6 = t.Update(1.0f, true, false, dt, ed);
        Check("0.5.4 unit: speed band hysteresis / down-hold / held through an animation-led turn",
              b0 == 1 && b1 == 1 && b2 == 2 && b3 == 2 && b4 == 2 && b5 == 2 && b6 == 1,
              std::to_string(b0) + std::to_string(b1) + std::to_string(b2) + std::to_string(b3) + std::to_string(b4) +
                  std::to_string(b5) + std::to_string(b6));
    }

    // ================================================================ 0.5.6 units (play 0.5.5, a former test database)
    {
        using namespace utpp::mm::integ;
        GateDebounceConfig on, off;
        off.holdSeconds = 0.0f;
        const float dt = 1.0f / 60.0f;
        const uint32_t soft = kGateLocoState | kGatePsmBlock;
        // single blip every 6 frames (game: 1 sample / 0.107 s) -> never closes; with the debounce off -> closes each time
        GateDebounce a, b;
        int closedA = 0, closedB = 0;
        for (int i = 0; i < 120; ++i)
        {
            const bool blip = i % 6 == 5;
            closedA += a.Update(blip ? soft : 0u, blip ? uint32_t(kPsmDetailed) : 0u, 0.0f, dt, on) != 0 ? 1 : 0;
            closedB += b.Update(blip ? soft : 0u, blip ? uint32_t(kPsmDetailed) : 0u, 0.0f, dt, off) != 0 ? 1 : 0;
        }
        // sustained soft closure (slide): closes after holdSeconds and stays closed; reopens with the raw gate
        GateDebounce c;
        int closeFrame = -1;
        for (int i = 0; i < 30; ++i)
            if (c.Update(soft, kPsmDetailed, 0.0f, dt, on) != 0 && closeFrame < 0)
                closeFrame = i;
        const bool stays = c.Update(soft, kPsmDetailed, 0.0f, dt, on) != 0;
        const bool reopens = c.Update(0u, 0u, 0.0f, dt, on) == 0;
        // real gates at once: vehicle, workspot, scene + detailed, TPP off, jump (soft but vz 3.5), fall (vz -6)
        GateDebounce d;
        const bool veh = d.Update(kGatePsmBlock, kPsmVehicle, 0.0f, dt, on) != 0;
        d.Reset();
        const bool ws = d.Update(kGatePsmBlock, kPsmWorkspot, 0.0f, dt, on) != 0;
        d.Reset();
        const bool scene = d.Update(kGatePsmBlock | kGateLocoState, kPsmScene | kPsmDetailed, 0.0f, dt, on) != 0;
        d.Reset();
        const bool tpp = d.Update(kGateTppInactive, 0u, 0.0f, dt, on) != 0;
        d.Reset();
        const bool jump = d.Update(soft, kPsmDetailed, 3.5f, dt, on) != 0;
        d.Reset();
        const bool fall = d.Update(soft, kPsmDetailed, -6.0f, dt, on) != 0;
        Check("0.5.6 unit: gate debounce - single-sample loco/detailed blips every 0.1 s never close (0.5.5: every blip closes), a "
              "sustained soft closure closes after 0.06 s and stays closed, vehicle / workspot / scene / TPP off / jump / fall close at once",
              closedA == 0 && closedB == 20 && closeFrame >= 3 && closeFrame <= 5 && stays && reopens && veh && ws && scene && tpp &&
                  jump && fall && a.Suppressed() == 20 && a.Confirmed() == 0 && PsmBlockBits(0x40u | (6u << 16)) == 0x40u &&
                  PsmDetailOf(0x40u | (6u << 16)) == 5 && PsmDetailOf(0x40u) == -1,
              "blipCloses " + std::to_string(closedA) + "/" + std::to_string(closedB) + " closeFrame " + std::to_string(closeFrame));
    }
    {
        using namespace utpp::mm::steer;
        const bool names =
            DriverClipClass("X_Transition_Run_Female1C27CrouchToRun1", kClassStanceTransition) == kClassStanceFromCrouch &&
            DriverClipClass("X_Transition_Run_Female1C26RunToCrouch", kClassStanceTransition) == kClassStanceToCrouch &&
            DriverClipClass("X_Transition_Walk_Female1B24WalkToCrouch", kClassStanceTransition) == kClassStanceToCrouch &&
            DriverClipClass("X_Transition_Sprint_CrouchedToSprinting", kClassStanceTransition) == kClassStanceFromCrouch &&
            DriverClipClass("M_Neutral_Transition_Stand_to_Crouch", kClassStanceTransition) == kClassStanceToCrouch &&
            DriverClipClass("M_Neutral_Transition_Crouch_to_Stand", kClassStanceTransition) == kClassStanceFromCrouch &&
            DriverClipClass("X_Transition_Walk_Female1A07Crouch", kClassStanceTransition) == kClassStanceTransition &&
            ClassifyClip("X_Crouch_Loop_F_CrouchWalkRight") == kClassNone &&
            DriverClipClass("X_Crouch_Loop_F_CrouchWalkRight", kClassNone) == kClassLateral &&
            DriverClipClass("X_Run_Loop_F_LeftStrafe", kClassNone) == kClassLateral &&
            DriverClipClass("X_Run_Move_L_Female1C24SideStepLeft", kClassNone) == kClassLateral &&
            DriverClipClass("M_Neutral_Crouch_Loop_F", kClassNone) == kClassNone &&
            DriverClipClass("M_Neutral_Walk_Loop_F_L_20", kClassNone) == kClassNone &&
            DriverClipClass("M_Neutral_Run_Turn_L_135_Lfoot", ClassifyClip("M_Neutral_Run_Turn_L_135_Lfoot")) == kClassTurnRun &&
            DriverClipClass("X_Walk_Turn_L_090_Female1B09WalkTurnLeft90", ClassifyClip("X_Walk_Turn_L_090_Female1B09WalkTurnLeft90")) ==
                kClassTurnWalk &&
            DriverClipClass("M_Neutral_Walk_Reface_Start_B_L_180", kClassReface) == kClassRefaceWalk;
        CurationInput ci;
        ci.input = true;
        ci.capsule = 2.7f;
        ci.stance = 2;
        ci.forward = true;
        ci.lateralOk = true;
        ci.sinceStance = 0.1f; // inside the 0.5.2 window
        CurationConfig on, off;
        on.crouchRules = true;
        const uint32_t a = WithVariants(ExcludedClasses(ci, on)), ac = WithVariants(ExcludedCurrentClasses(ci, on));
        const uint32_t b = WithVariants(ExcludedClasses(ci, off));
        ci.stance = 1;
        const uint32_t c = WithVariants(ExcludedClasses(ci, on));
        ci.capsule = 10.0f;
        const uint32_t d = WithVariants(ExcludedClasses(ci, on));
        ci.capsule = 2.7f;
        ci.lateralOk = false;
        const uint32_t e = WithVariants(ExcludedClasses(ci, on));
        ci.lateralOk = true;
        ci.forward = false;
        const uint32_t f = WithVariants(ExcludedClasses(ci, on));
        ci.sinceStance = 5.0f;
        const uint32_t g = WithVariants(ExcludedClasses(ci, on));
        const bool rules = (a & Bit(kClassStanceFromCrouch)) && !(a & Bit(kClassStanceToCrouch)) && (ac & Bit(kClassStanceFromCrouch)) &&
                           (a & Bit(kClassLateral)) && (ac & Bit(kClassLateral)) && !(b & (kStanceVariantMask | Bit(kClassLateral))) &&
                           (c & Bit(kClassStanceToCrouch)) && !(c & Bit(kClassStanceFromCrouch)) &&
                           (d & kStanceVariantMask) == kStanceVariantMask && !(e & Bit(kClassLateral)) && !(f & Bit(kClassLateral)) &&
                           (g & kStanceVariantMask) == kStanceVariantMask;
        Check("0.5.6 unit: stance-transition direction / lateral classes from the in-game names; crouched -> no CrouchTo* (also as current), "
              "standing -> no *ToCrouch, none above 6 m/s, lateral out only for forward motion with a non-lateral forward loop; "
              "0.5.2 window covers both directions; off = 0.5.5",
              names && rules);
    }

    using S = Step;
    const std::vector<Step> crouchMove = {
        S{.name = "crouch_idle", .seconds = 1.0f, .stick = 0.0f, .loco = 2},
        S{.name = "crouch_start", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
        S{.name = "crouch_circle", .seconds = 3.0f, .stick = 1.0f, .angleRate = 220.0f, .loco = 2},
        S{.name = "crouch_zig", .seconds = 0.6f, .stick = 1.0f, .angleDeg = 70.0f, .loco = 2},
        S{.name = "crouch_zag", .seconds = 0.6f, .stick = 1.0f, .angleDeg = -110.0f, .loco = 2},
        S{.name = "crouch_zig2", .seconds = 0.6f, .stick = 1.0f, .angleDeg = 90.0f, .loco = 2},
        S{.name = "crouch_straight", .seconds = 2.0f, .stick = 1.0f, .loco = 2},
        S{.name = "crouch_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    const std::vector<Step> runFlick = {
        S{.name = "run_accel", .seconds = 1.5f, .stick = 1.0f},
        S{.name = "run_cruise", .seconds = 2.0f, .stick = 1.0f},
        S{.name = "run_flick180", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 180.0f, .zeroFrames = 2},
        S{.name = "run_back", .seconds = 1.5f, .stick = 1.0f},
        S{.name = "run_turn135", .seconds = 1.5f, .stick = 1.0f, .angleDeg = -135.0f},
        S{.name = "run_turn120", .seconds = 1.5f, .stick = 1.0f, .angleDeg = 120.0f},
        S{.name = "run_stop", .seconds = 2.0f, .stick = 0.0f}};
    std::vector<Step> runFlickGame = runFlick;
    for (Step& s : runFlickGame)
        s.rotateVel = true;
    const std::vector<Step> walkTurns = {
        S{.name = "walk", .seconds = 2.5f, .stick = 0.75f},
        S{.name = "walk_turn180", .seconds = 2.5f, .stick = 0.75f, .angleDeg = 180.0f},
        S{.name = "walk_turn135", .seconds = 2.5f, .stick = 0.75f, .angleDeg = -135.0f},
        S{.name = "walk_turn90", .seconds = 2.0f, .stick = 0.75f, .angleDeg = 90.0f},
        S{.name = "walk_stop", .seconds = 2.0f, .stick = 0.0f}};
    const std::vector<Step> sprint = {
        S{.name = "sprint_accel", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
        S{.name = "sprint_cruise", .seconds = 2.0f, .stick = 1.0f, .loco = 3},
        S{.name = "sprint_turn120", .seconds = 2.0f, .stick = 1.0f, .angleDeg = 120.0f, .loco = 3},
        S{.name = "sprint_stop", .seconds = 2.0f, .stick = 0.0f}};

    int negWalkAtRun = 0;
    // ---- 0.5.6 scenarios from a recorded play log (a former test database)
    // A: 02:54:08-02:54:24 - run, crouch at a run, crouch walk, stand up, crouch again, straight crouch walk (CrouchToRun
    //    chosen right after entering crouch, a sideways crouch loop for 3 s of straight crouch walking)
    const std::vector<Step> seqA = {S{.name = "a1_run", .seconds = 2.0f, .stick = 1.0f, .loco = 1},
                                    S{.name = "a2_crouch_in", .seconds = 0.8f, .stick = 1.0f, .loco = 2},
                                    S{.name = "a3_crouch_walk", .seconds = 3.0f, .stick = 1.0f, .loco = 2},
                                    S{.name = "a4_stand_up", .seconds = 1.4f, .stick = 1.0f, .loco = 1},
                                    S{.name = "a5_crouch_again", .seconds = 0.8f, .stick = 1.0f, .loco = 2},
                                    S{.name = "a6_crouch_straight", .seconds = 4.0f, .stick = 1.0f, .loco = 2},
                                    S{.name = "a7_stop", .seconds = 1.5f, .stick = 0.0f, .loco = 2}};
    // B: 02:54:08-02:54:17 / 02:52:15-02:52:19 - sprint, slide entry (crouch at 10-11 m/s), slide (loco 0 + kPsmDetailed for
    //    1.3 s), then crouch walk with a single-sample loco-0 / kPsmDetailed blip every ~0.1 s for ~2.1 s (w = 0.00 in game)
    const std::vector<Step> seqB = {
        S{.name = "b1_sprint", .seconds = 2.5f, .stick = 1.0f, .loco = 3},
        S{.name = "b2_slide_entry", .seconds = 0.1f, .stick = 1.0f, .loco = 2, .speed = 10.5f},
        S{.name = "b3_slide", .seconds = 1.3f, .stick = 1.0f, .loco = 0, .psm = 0x40u | (6u << 16), .speed = 8.3f},
        S{.name = "b4_crouch_blips", .seconds = 2.1f, .stick = 1.0f, .loco = 2, .blinkEvery = 6, .expectOpen = true},
        S{.name = "b5_crouch_walk", .seconds = 1.5f, .stick = 1.0f, .loco = 2, .expectOpen = true}};
    // C: real gates must still close at once (vehicle, workspot, jump with vertical speed); a sustained dodge-like soft
    //    closure closes after the debounce
    const std::vector<Step> seqC = {
        S{.name = "c1_walk", .seconds = 2.0f, .stick = 0.75f, .loco = 1, .expectOpen = true},
        S{.name = "c2_jump", .seconds = 0.5f, .stick = 0.75f, .loco = 0, .psm = 0x40u | (19u << 16), .vz = 3.5f},
        S{.name = "c3_walk", .seconds = 1.5f, .stick = 0.75f, .loco = 1},
        S{.name = "c4_vehicle", .seconds = 1.0f, .stick = 0.0f, .loco = 1, .psm = 0x1u},
        S{.name = "c5_walk", .seconds = 1.5f, .stick = 0.75f, .loco = 1},
        S{.name = "c6_workspot", .seconds = 1.0f, .stick = 0.0f, .loco = 1, .psm = 0x8u},
        S{.name = "c7_walk", .seconds = 1.5f, .stick = 0.75f, .loco = 1},
        S{.name = "c8_dodge", .seconds = 0.4f, .stick = 0.75f, .loco = 0, .psm = 0x40u},
        S{.name = "c9_walk", .seconds = 1.0f, .stick = 0.75f, .loco = 1}};
    float negBlipLow = 0.0f;
    std::string table056;
    for (int di = 1; di < argc; ++di)
    {
        std::string name = argv[di];
        {
            const size_t sl = name.find_last_of("\\/");
            if (sl != std::string::npos)
                name = name.substr(sl + 1);
            const size_t dot = name.rfind('.');
            if (dot != std::string::npos)
                name = name.substr(0, dot);
        }
        if (di > 1)
        {
            const std::wstring p(argv[di], argv[di] + std::strlen(argv[di]));
            g_infoLines.clear();
            if (!LoadDatabase(p, name))
            {
                Check("0.5.4 database loaded: " + name, false, argv[di]);
                continue;
            }
            OnArmed(&rig, hashes, parents);
            SetEnabled(true);
        }
        const std::string cal = SpeedCalibrationSummary();
        std::printf("[INFO] 0.5.4 db %s: %s\n", name.c_str(), cal.c_str());
        std::map<std::string, std::map<std::string, Metrics>> rr[3];
        std::string st054;
        for (int mode = 0; mode < 3; ++mode)
        {
            ConfigureMode(mode);
            rr[mode]["crouch"] = RunScenario(crouchMove, 98.0f);
            rr[mode]["runflick"] = RunScenario(runFlick, -82.0f);
            {
                // diagnostic: UTPP_LOCO_FRESH_FLICK=1 runs the game-like run flick from a fresh matcher state
                char fe[4] = {};
                const bool fresh = GetEnvironmentVariableA("UTPP_LOCO_FRESH_FLICK", fe, sizeof fe) > 0 && fe[0] == '1';
                rr[mode]["runflick_game"] = RunScenario(runFlickGame, -82.0f, fresh);
            }
            rr[mode]["walkturns"] = RunScenario(walkTurns, -82.0f);
            rr[mode]["sprint"] = RunScenario(sprint, -82.0f);
            if (mode == 2)
            {
                st054 = Status();
            }
            for (const auto& sc : rr[mode])
            {
                std::printf("[INFO] %s %s %s TOTAL: %s\n", name.c_str(), kModeName[mode], sc.first.c_str(), Line054(Total(sc.second)).c_str());
                for (const auto& stp : sc.second)
                    std::printf("[INFO]   %s %s %s/%s: %s\n", name.c_str(), kModeName[mode], sc.first.c_str(), stp.first.c_str(),
                                Line054(stp.second).c_str());
            }
        }
        for (const std::string& l : g_infoLines)
            if (l.find("speed bands") != std::string::npos || l.find("WARNING") != std::string::npos)
                std::printf("[INFO] %s log: %s\n", name.c_str(), l.c_str());
        const Metrics c3 = Total(rr[1]["crouch"]), c4 = Total(rr[2]["crouch"]);
        const Metrics r3 = Total(rr[1]["runflick"]), r4 = Total(rr[2]["runflick"]);
        const Metrics g3 = Total(rr[1]["runflick_game"]), g4 = Total(rr[2]["runflick_game"]);
        const Metrics w3 = Total(rr[1]["walkturns"]), w4 = Total(rr[2]["walkturns"]);
        const Metrics s4 = Total(rr[2]["sprint"]);
        negWalkAtRun += r3.walkAtRun + g3.walkAtRun + Total(rr[1]["sprint"]).walkAtRun + r3.sprintAtRun + g3.sprintAtRun;
        // idle: none selected above 0.5 m/s with the stick held; a playing idle may last until the next search (search
        // interval in matcher time, up to ~0.4 s real at the 0.25 slow rate) - at most 0.5 s over the whole scenario and
        // at most 10 % of 0.5.3
        Check("0.5.4 [" + name + "] crouch movement: no idle / idle-turn clip selected while moving with the stick held, idle playing "
                                 "while moving <= 0.5 s (<= 10 % of 0.5.3)",
              c4.idleMoving == 0 && c4.tipMoving == 0 && c4.idleMovingTime <= 0.5f &&
                  (c3.idleMovingTime <= 0.0f || c4.idleMovingTime <= 0.1f * c3.idleMovingTime),
              Line054(c4) + " | 0.5.3 " + Line054(c3));
        Check("0.5.4 [" + name + "] run flick 180 / 135 / 120 (both capsule models): no walk / sprint-class pivot / turn / reface at a "
                                 "run, no run-class one in a walk",
              r4.walkAtRun == 0 && g4.walkAtRun == 0 && r4.runAtWalk == 0 && g4.runAtWalk == 0 && w4.runAtWalk == 0 &&
                  r4.sprintAtRun == 0 && g4.sprintAtRun == 0,
              Line054(g4) + " | 0.5.3 " + Line054(g3) + " | brake model 0.5.4 " + Line054(r4) + " | 0.5.3 " + Line054(r3));
        Check("0.5.4 [" + name + "] slow walk turns: walk band throughout, walk pivots / turns not excluded more than in 0.5.3, still "
                                 "selected when 0.5.3 selected them",
              w4.bandMax == 1 && w4.walkAllowedLed >= w3.walkAllowedLed && w4.walkTurns >= std::min(w3.walkTurns, 1),
              Line054(w4) + " | 0.5.3 " + Line054(w3));
        // Trade-off (reported): without the walk-class clips the run flick uses Run_Turn_180 / Run_Reface (motion data
        // decelerates in place) - lower-matching candidates, the offset root may saturate longer than with the walk clips of
        // 0.5.3. Guard against a gross regression only; the measured deltas are printed and reported.
        Check("0.5.4 [" + name + "] run flick (game-like capsule): no gross regression vs 0.5.3 (cost max <= 2x + 5, offset saturation <= +1 s)",
              g4.costMax <= g3.costMax * 2.0f + 5.0f && g4.offSat <= g3.offSat + 1.0f,
              "cost " + std::to_string(g4.costMax) + " vs " + std::to_string(g3.costMax) + ", sat " + std::to_string(g4.offSat) + " vs " +
                  std::to_string(g3.offSat) + " (INFO brake model: cost " + std::to_string(r4.costMax) + " vs " + std::to_string(r3.costMax) +
                  ", sat " + std::to_string(r4.offSat) + " vs " + std::to_string(r3.offSat) + ")");
        {
            const size_t p = st054.find(" speedBands=");
            std::printf("[INFO] %s status:%s\n", name.c_str(), p == std::string::npos ? " (none)" : st054.substr(p).c_str());
        }
        {
            // sprint data: status sprintData = 0 exactly when the calibration's sprint speed is a fallback ('f'), and a
            // WARNING is logged then; no walk pivots / turns at a sprint
            const size_t p = cal.find("sprint ");
            const size_t q = p == std::string::npos ? p : cal.find('(', p);
            const bool fallback = q != std::string::npos && q + 1 < cal.size() && cal[q + 1] == 'f';
            const bool flag = Field(st054, "sprintData") > 0.5f;
            bool warned = false;
            for (const std::string& l : g_infoLines)
                warned = warned || (l.find("WARNING") != std::string::npos && l.find("no sprint") != std::string::npos);
            Check("0.5.4 [" + name + "] sprint data reported (sprintData=" + std::string(flag ? "1" : "0") + ", warning when missing), "
                                     "no walk pivots / turns at a sprint",
                  flag == !fallback && (!fallback || warned || di == 1) && s4.walkAtRun == 0, Line054(s4));
        }
        // ================================================================ 0.5.6 (0.5.5 = both 0.5.6 switches off)
        {
            std::map<std::string, Metrics> a[2], b[2], c[2];
            for (int v = 0; v < 2; ++v)
            {
                Configure055(v == 1);
                a[v] = RunScenario(seqA, 98.0f);
                b[v] = RunScenario(seqB, 98.0f);
                c[v] = RunScenario(seqC, -82.0f);
                for (auto* sc : {&a[v], &b[v], &c[v]})
                    for (const auto& stp : *sc)
                        std::printf("[INFO] %s %s %s: %s\n", name.c_str(), v ? "0.5.6" : "0.5.5", stp.first.c_str(), Line056(stp.second).c_str());
            }
            const std::string st056 = Status();
            {
                const size_t p = st056.find(" gateDebounce=");
                std::printf("[INFO] %s status:%s\n", name.c_str(), p == std::string::npos ? " (none)" : st056.substr(p).c_str());
            }
            const Metrics a0 = Total(a[0]), a1 = Total(a[1]), b0 = Total(b[0]), b1 = Total(b[1]);
            const Metrics& blip0 = b[0].at("b4_crouch_blips");
            const Metrics& blip1 = b[1].at("b4_crouch_blips");
            const Metrics& walk1 = b[1].at("b5_crouch_walk");
            negBlipLow = std::max(negBlipLow, blip0.wLowTime);
            // lateral negative control: only databases where 0.5.6 applies the rule (crouch lateralOk), see seqD below
            const bool lateralOk = Field(st056, "lateralOk") > 0.0f; // stand flag; crouch flag after the '/'
            const bool crouchLateralOk = st056.find("lateralOk=1/1") != std::string::npos || st056.find("lateralOk=0/1") != std::string::npos;
            (void)lateralOk;
            Check("0.5.6 [" + name + "] crouch walk with single-sample loco-0 / detailed blips every 0.1 s after a slide: MM weight back "
                                     "(< 0.5 for <= 0.45 s = reopen hold + fade-in, >= 0.99 over the last 1.5 s), none lost in the following walk",
                  blip1.wLowTime <= 0.45f && blip1.wMinLast >= 0.99f && walk1.wLowTime <= 0.0f,
                  Line056(blip1) + " | 0.5.5 " + Line056(blip0));
            const Metrics& sl0 = b[0].at("b3_slide");
            const Metrics& sl1 = b[1].at("b3_slide");
            const Metrics& j1 = c[1].at("c2_jump");
            const Metrics& v1 = c[1].at("c4_vehicle");
            const Metrics& w1 = c[1].at("c6_workspot");
            const Metrics& d1 = c[1].at("c8_dodge");
            const Metrics& v0 = c[0].at("c4_vehicle");
            Check("0.5.6 [" + name + "] real gates: vehicle / workspot / jump (vz 3.5) close on the first sample and reach weight 0 as in 0.5.5; "
                                     "sustained slide / dodge close within 0.06 s + 2 frames",
                  j1.closeLatency == 0.0f && v1.closeLatency == 0.0f && w1.closeLatency == 0.0f && v1.zeroLatency >= 0.0f &&
                      v1.zeroLatency <= v0.zeroLatency + 1e-4f && w1.zeroLatency >= 0.0f && sl1.closeLatency >= 0.0f &&
                      sl1.closeLatency <= 0.06f + 2.0f / 60.0f && d1.closeLatency >= 0.0f && d1.closeLatency <= 0.06f + 2.0f / 60.0f &&
                      sl1.zeroLatency >= 0.0f && sl1.zeroLatency <= sl0.zeroLatency + 0.06f + 2.0f / 60.0f,
                  "jump " + Line056(j1) + " | vehicle " + Line056(v1) + " | workspot " + Line056(w1) + " | dodge " + Line056(d1) +
                      " | slide " + Line056(sl1) + " (0.5.5 slide close " + std::to_string(sl0.closeLatency) + " zero " +
                      std::to_string(sl0.zeroLatency) + ")");
            Check("0.5.6 [" + name + "] crouch toggles / slide entry: no stance transition against the current stance, none above 6 m/s",
                  a1.wrongStance == 0 && a1.stanceFast == 0 && b1.wrongStance == 0 && b1.stanceFast == 0,
                  "A " + Line056(a1) + " | B " + Line056(b1) + " | 0.5.5 A " + Line056(a0) + " | 0.5.5 B " + Line056(b0));
            Check("0.5.6 [" + name + "] straight crouch walk: no lateral loop selected for forward motion when the crouch set has a "
                                     "non-lateral forward loop (crouch lateralOk=" + std::string(crouchLateralOk ? "1" : "0") + ")",
                  !crouchLateralOk || (a1.lateralFwd == 0 && b1.lateralFwd == 0 && a1.lateralTime <= 0.0f),
                  "A " + Line056(a1) + " | 0.5.5 A " + Line056(a0));
            {
                const Metrics& cs0 = a[0].at("a6_crouch_straight");
                const Metrics& cs1 = a[1].at("a6_crouch_straight");
                char row[640];
                std::snprintf(row, sizeof row,
                              "%s | blips wLow %.2f -> %.2f s | slide close %.3f -> %.3f s | wrongStance %d -> %d | stanceFast %d -> %d | "
                              "rightStance %d -> %d | lateralFwd %d -> %d (%.2f -> %.2f s) | crouch straight offSat %.2f -> %.2f s offMax %.1f -> %.1f cm "
                              "rate %.2f..%.2f -> %.2f..%.2f | clips %zu -> %zu\n",
                              name.c_str(), blip0.wLowTime, blip1.wLowTime, sl0.closeLatency, sl1.closeLatency, a0.wrongStance + b0.wrongStance,
                              a1.wrongStance + b1.wrongStance, a0.stanceFast + b0.stanceFast, a1.stanceFast + b1.stanceFast,
                              a0.rightStance + b0.rightStance, a1.rightStance + b1.rightStance, a0.lateralFwd + b0.lateralFwd,
                              a1.lateralFwd + b1.lateralFwd, a0.lateralTime + b0.lateralTime, a1.lateralTime + b1.lateralTime, cs0.offSat,
                              cs1.offSat, cs0.offMax, cs1.offMax, cs0.rateMin, cs0.rateMax, cs1.rateMin, cs1.rateMax, cs0.clips.size(),
                              cs1.clips.size());
                table056 += row;
                std::string l0, l1;
                for (const std::string& cl : cs0.clips)
                    l0 += cl + " ";
                for (const std::string& cl : cs1.clips)
                    l1 += cl + " ";
                std::printf("[INFO] %s crouch straight clips 0.5.5: %s| 0.5.6: %s\n", name.c_str(), l0.c_str(), l1.c_str());
            }
        }
    }
    std::printf("[INFO] 0.5.6 before -> after (0.5.5 -> 0.5.6):\n%s", table056.c_str());
    Check("negative control 0.5.5: single-sample blips keep the MM weight below 0.5 for >= 1.5 s of the 2.1 s crouch walk (some database)",
          negBlipLow >= 1.5f, std::to_string(negBlipLow));
    // (0.5.5 negative controls for a stance transition against the current stance / a lateral crouch loop for forward crouch walking and the
    // 0.5.3 idle-while-moving control need clips of non-package sources; the package database does not reproduce them)
    Check("negative control 0.5.3 (speed bands off): a walk / sprint-class pivot / turn / reface is selected at a run (some database)",
          negWalkAtRun > 0,
          std::to_string(negWalkAtRun));

    // start with a 180 deg gap (crouch_walk_turns 'walk' step; the harness did not enter crouch). 2026-10-07: run from a
    // fresh matcher state on the first database (in the chained sequence its motion-only path depended on the end state of
    // the run flick and flipped between offset saturation 0.30 / 0.62 s with unrelated rate changes)
    {
        const std::wstring p1(argv[1], argv[1] + std::strlen(argv[1]));
        if (LoadDatabase(p1, "test"))
        {
            OnArmed(&rig, hashes, parents);
            ConfigureMode(2);
            const Metrics ts1 = RunScenario(turnStart, 98.0f, true).at("walk");
            Check("0.5.3 start with a 180 deg gap (fresh state): cost <= 15, offset saturated <= 0.5 s", ts1.costMax <= 15.0f && ts1.offSat <= 0.5f,
                  ts1.Line());
        }
        else
            Check("turnstart: database reloaded", false, argv[1]);
    }
    }
    if (UnitOn("main") && TimingOn())
        std::printf("[TIME] unit main: %.1f s\n", double(GetTickCount() - mainT0) / 1000.0);
    RunUnit("057", Section057, argc, argv, hashes, parents, &rig);           // 0.5.7
    RunUnit("058", Section058, argc, argv, hashes, parents, &rig);           // 0.5.8
    RunUnit("059", Section059, argc, argv, hashes, parents, &rig);           // 0.5.9
    RunUnit("060", Section060, argc, argv, hashes, parents, &rig);           // 0.6.0
    RunUnit("062", Section062, argc, argv, hashes, parents, &rig);           // 0.6.2
    RunUnit("063", Section063, argc, argv, hashes, parents, &rig);           // 0.6.3
    RunUnit("c0c1a", SectionC0C1a, argc, argv, hashes, parents, &rig);       // loco C0 / C1a
    RunUnit("c1b", SectionC1b, argc, argv, hashes, parents, &rig);           // loco C1b
    RunUnit("idle", SectionIdleHold, argc, argv, hashes, parents, &rig);     // stage standstill idle
    RunUnit("knee", SectionKnee, argc, argv, hashes, parents, &rig);         // batch 2 knee fix
    RunUnit("strafe", SectionStrafe, argc, argv, hashes, parents, &rig);     // B2 strafe S1
    RunUnit("air", SectionAir, argc, argv, hashes, parents, &rig);           // AIR HOLD stage
    RunUnit("airclips", SectionAirClips, argc, argv, hashes, parents, &rig); // AIR CLIPS stage
    RunUnit("airsteer", SectionAirSteer, argc, argv, hashes, parents, &rig); // AIR STEER stage (after every other section: they keep their numbers)
    RunUnit("vehexit", SectionVehExit, argc, argv, hashes, parents, &rig);   // vehicle batch W1 (last: the earlier sections keep their numbers)
    // shard: the testorder leak check of the single-process run happens at the start of the next section; here the last unit of the
    // shard is checked the same way (settings reset, fingerprint compared with the first section of this process; prints only on a leak)
    if (g_unitFilter)
        BeginSection("ShardEnd");
    Shutdown();
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// Native get-up through the REAL v1 host (src/MotionHost.cpp Feed + the v2 seam, MotionDriver.cpp glue, no RED4ext): the ragdoll pose store is
// filled like PoseHook does, UTPPRagdollGetUpPlan / Begin run on the woman bundle with a synthetic utpp_getup_woman.utpptrav (4 montages that
// morph from the lying pose to a standing one), and the published snapshots are observed feed by feed.
// Usage: utpp_ragdoll_getup_host_test <data dir> <v1 database>
// Checks: the file loads (avail=ok); Plan picks the clip (Back) with a small rms; Begin takes the body (episode handed over, GetUpOwns);
// the FIRST get-up feed already publishes at weight 1 with the gate open (the v1 gate does not abort the run with 'v1-gate'); the phases
// Blend -> Play -> HandOff -> Idle on feed time; the run ends 'done' without an abort; a capsule that jumps 0.17 m up / 0.1 m sideways in the
// middle of the blend does not move the world Hips (world anchoring through the live feed); an abort hands back without a pose pop.
// Release continuity (r13b): the script teleports the entity to the plan target in the Begin call; the feed samples show the new frame one feed
// later and a publish can come before the first Step. World Hips / Head must stay continuous (<= 1 cm) from the first publish after the release,
// whatever the feed shows (stale pre-teleport sample, no teleport at all), and the first publish after Begin must carry the get-up pose.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/mm/MotionAdapter.hpp"
#include "../src/mm/MiniJson.hpp"
#include "../src/ragdoll/GetUpClips.hpp"
#include "../src/ragdoll/GetUpPlayer.hpp"
#include "../src/ragdoll/RdLastPose.hpp"

#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace gut
{
#define main UnusedGetUpMain
#include "RagdollGetUpTest.cpp"
#undef main
} // namespace gut

namespace
{
namespace rag = utpp::ragdoll;

struct Sample
{
    float w = -1.0f;
    int state = 0;
    float hipsLocal[3] = {0, 0, 0};
    float hipsWorld[3] = {0, 0, 0};
    bool stepped = false;
};

struct RunOut
{
    int planCode = -99;
    float planRms = -1.0f;
    bool begin = false;
    std::vector<Sample> s;
    std::string statusEnd;
    bool ownsAfter = true;
    // release continuity: the stand-in publish between Begin and the first feed (the engine frame is the teleport target from the Begin call on)
    bool gapApplied = false;
    float gapHipsW[3] = {0, 0, 0}, gapHeadW[3] = {0, 0, 0}, snapHipsW[3] = {0, 0, 0}, snapHeadW[3] = {0, 0, 0};
    uint64_t beginSeqBefore = 0, beginSeqAfter = 0, tickSeqAfter = 0;
    float beginWeight = -1.0f;
    float tgtYaw = 0.0f;
    bool hitchApplied = false;
    int hitchState = -9;
    std::vector<std::array<float, 3>> headW; // per feed: world Head (engine frame)
};

constexpr float kDt = 1.0f / 60.0f;
int g_body = 0; // TR-dll male parity: 0 woman, 1 man (--body man); the synthetic clip set / bundle / skeleton follow it

// The driver's steady clock (lift slew between publishes, wall watchdog, yield timers) runs on feed time in this test: one feed = kDt, whatever
// the machine load. On the real clock the two runs compared below get different publish gaps, hence a different floor lift (3 m/s), and the
// world Hips of the two runs differ by up to the lift step of a stalled frame although the anchoring is exact.
struct SimClock
{
    SimClock() { motiondriver::TestSetSteadyClockNs(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); }
    ~SimClock() { motiondriver::TestSetSteadyClockNs(-1); }
    SimClock(const SimClock&) = delete;
    SimClock& operator=(const SimClock&) = delete;
};

// who shows what after the release: the ENGINE frame is the one the entity is rendered in, the SAMPLE is what the feed reports
enum class Tele
{
    SameAtOnce,  // the script's teleport reaches the engine and the feed together (the plain run)
    StaleSample, // engine at the target from the Begin call, the feed still carries the pre-teleport sample for aStale feeds
    Never        // no teleport at all: engine and feed stay at the pre frame
};

RunOut RunGetUp(const Rig& aRig, bool aCapsuleJump, int aAbortAtFeed, Tele aTele = Tele::SameAtOnce, int aStale = 0, bool aGap = false, int aHitchMs = 0)
{
    RunOut r;
    const SimClock simClock;
    motiondriver::SetEnabled(false);
    Check(ResetBase(aRig), "real host (re)initialized");
    motionhost::SetCapsuleSync(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    (void)motiondriver::Step({}, 0, kDt);
    motiondriver::WaitLoader();
    motiondriver::RagdollSetGetUp(true);

    // engine frame (what is rendered) and feed sample (what the capsule reports), position + yaw rad; both start at the pre-teleport entity
    float ec[3] = {0.0f, 0.0f, 0.0f}, sc[3] = {0.0f, 0.0f, 0.0f};
    float ey = 0.0f, sy = 0.0f;
    float floorZ = 0.0f; // world floor height
    const auto worldOf = [&](const float* aLocal, float* aOut) {
        float rl[3];
        rag::gu::RotZ(ey, aLocal, rl);
        for (int k = 0; k < 3; ++k)
            aOut[k] = ec[k] + rl[k];
    };
    const auto headOf = [&](const mm::integ::PoseSnapshot& aSnap, float* aOut) {
        gut::Pose p(aSnap.pose, aSnap.pose + size_t(gut::kN) * 7);
        const gut::Fk fk = gut::DoFk(p);
        worldOf(gut::Joint(fk, rag::kJHead), aOut);
    };
    const auto feed = [&](uint32_t aPsm) {
        motionhost::FeedInput f;
        f.dt = kDt;
        f.tppActive = true;
        f.haveAir = true;
        f.onGround = true;
        f.distToGround = std::fmax(0.0f, sc[2] - floorZ); // the world floor stays at floorZ: a capsule that is moved up by the jump has that much ground below
                                                          // it, so the floor safety net (zGround = -distToGround in model space) sees the same world floor in all runs
        f.locoState = mm::integ::kLocoStand;
        f.psmBlock = aPsm;
        f.haveExtra = true;
        f.capsulePos[0] = sc[0];
        f.capsulePos[1] = sc[1];
        f.capsulePos[2] = sc[2];
        f.playerYawDeg = sy / mm::integ::kDegToRad;
        motiondriver::FeedAirFromInput(f);
        g_log = HookLog{};
        motiondriver::TestAdvanceSteadyClockNs(static_cast<int64_t>(double(kDt) * 1e9)); // one feed of simulated time
        motionhost::Feed(f);
        Sample s;
        mm::integ::PoseSnapshot snap;
        std::array<float, 3> hw{0.0f, 0.0f, 0.0f};
        if (g_mmPose.Read(snap))
        {
            s.w = snap.weight;
            const float* h = &snap.pose[size_t(2) * 7];
            worldOf(h, s.hipsWorld);
            for (int k = 0; k < 3; ++k)
                s.hipsLocal[k] = h[k];
            headOf(snap, hw.data());
        }
        r.headW.push_back(hw);
        s.state = motiondriver::RagdollGetUpState();
        s.stepped = g_log.stepResult;
        return s;
    };
    const uint32_t kKnock = mm::integ::kPsmDetailed | (30u << mm::integ::kPsmDetailShift); // LocomotionDetailed 29 as the script injects it
    for (int i = 0; i < 90; ++i)
        feed(0u);
    for (int i = 0; i < 20; ++i)
        feed(kKnock);
    r.headW.clear();

    // the ragdoll pose store as PoseHook fills it: records of the lying pose (clip B placed at an anchor), db bone i = game bone i
    gut::Rng rng;
    const gut::ClipSet set = gut::MakeClipSet(g_body == 0);
    gut::Scene sc0{{0.10f, 0.20f, 0.0f}, 0.5, {0.0f, 0.0f, 0.0f}, 0.0};
    gut::Pose snapPose;
    gut::MakeSnapshot(set.local[0], sc0, rng, 0.0, 0.0, snapPose);
    static uint32_t dst[rag::RdLastPose::kMaxBones];
    static uint8_t data[rag::RdLastPose::kMaxBones][rag::RdLastPose::kStride];
    uint32_t n = 0;
    for (uint32_t i = 2; i < gut::kN; ++i, ++n)
    {
        dst[n] = i;
        float rec[12] = {};
        std::memcpy(rec, &snapPose[size_t(i) * 7], 3 * sizeof(float));
        std::memcpy(rec + 4, &snapPose[size_t(i) * 7 + 3], 4 * sizeof(float));
        rec[8] = rec[9] = rec[10] = 1.0f;
        std::memcpy(data[n], rec, sizeof rec);
    }
    LARGE_INTEGER hz{}, now{};
    QueryPerformanceFrequency(&hz);
    QueryPerformanceCounter(&now);
    rag::LastPose().Clear();
    rag::LastPose().Write(1, dst, data, n, now.QuadPart, 645);
    const float pos[3] = {0, 0, 0}, rot[4] = {0, 0, 0, 1};
    r.planCode = motiondriver::RagdollGetUpPlan(pos, rot, now.QuadPart, hz.QuadPart, false, 1.0f);
    float tgt[4];
    motiondriver::RagdollGetUpTarget(tgt);
    r.planRms = tgt[3];
    mm::integ::PoseSnapshot before;
    const uint64_t seqBeforeBegin = g_mmPose.Read(before) ? before.sequence : 0;
    r.begin = r.planCode >= 0 && motiondriver::RagdollGetUpBeginPublish(std::nanf("")); // as the script native: Begin, then the v1 publish
    if (!r.begin)
    {
        r.statusEnd = motiondriver::RagdollGetUpStatus();
        return r;
    }
    // the script's Teleport(player, feet, rot) in the same call: feet = anchor (xy + ground z), rot = the plan yaw
    float fw[2];
    motiondriver::RagdollGetUpForward(fw);
    r.tgtYaw = std::atan2(-fw[0], fw[1]);
    floorZ = tgt[2];
    // the ragdoll snapshot in the world (pre frame: entity at the origin, yaw 0), the truth every frame from the release on must stay on
    {
        const gut::Fk fk = gut::DoFk(snapPose);
        const float* hh = gut::Joint(fk, rag::kJHips);
        const float* hd = gut::Joint(fk, rag::kJHead);
        for (int k = 0; k < 3; ++k)
        {
            r.snapHipsW[k] = hh[k];
            r.snapHeadW[k] = hd[k];
        }
    }
    if (aTele != Tele::Never)
    {
        for (int k = 0; k < 3; ++k)
            ec[k] = tgt[k];
        ey = r.tgtYaw;
        if (aTele == Tele::SameAtOnce)
        {
            for (int k = 0; k < 3; ++k)
                sc[k] = tgt[k];
            sy = r.tgtYaw;
        }
    }
    if (aGap)
    {
        // the real v1 publish right after Begin (motionhost::PublishGetUpGap through RagdollGetUpBeginPublish), no stand-in: the snapshot the FK hook would read
        mm::integ::PoseSnapshot after;
        const bool haveAfter = g_mmPose.Read(after);
        r.beginSeqBefore = seqBeforeBegin;
        r.beginSeqAfter = haveAfter ? after.sequence : 0;
        r.beginWeight = haveAfter ? after.weight : -1.0f;
        if (haveAfter)
        {
            const float* h = &after.pose[size_t(2) * 7];
            r.gapApplied = after.sequence > seqBeforeBegin && after.weight >= 0.99f && std::fabs(h[0]) < 5.0f && std::fabs(h[1]) < 5.0f;
            worldOf(h, r.gapHipsW);
            headOf(after, r.gapHeadW);
        }
        // the plain Tick of the real host between Begin and the first feed publishes nothing more (the feed drives the weight)
        motionhost::Tick();
        mm::integ::PoseSnapshot afterTick;
        r.tickSeqAfter = g_mmPose.Read(afterTick) ? afterTick.sequence : 0;
        if (aHitchMs > 0)
        {
            // a long frame before the first Step: further publishes without a Step (stand-in snapshot through the apply hook, the clock runs on);
            // the first-Step grace of 0.3 s still applies
            static mm::integ::PoseSnapshot gap;
            gap = mm::integ::PoseSnapshot{};
            gap.rig = RigPtr();
            gap.weight = 1.0f;
            gap.numBones = gut::kN;
            gap.firstBone = 2;
            (void)WApply(gap);
            motiondriver::TestAdvanceSteadyClockNs(static_cast<int64_t>(aHitchMs) * 1000000);
            r.hitchApplied = WApply(gap);
            r.hitchState = motiondriver::RagdollGetUpState();
        }
    }
    for (int i = 0; i < 230; ++i)
    {
        if (aCapsuleJump && i == 8)
        {
            ec[0] += 0.1f; // (inside the 0.15 m off-anchor limit: the capsule is at the anchor after the teleport) the entity is moved (a late teleport, the end-of-episode drop, a push): the body must stay in the world
            ec[2] += 0.17f;
            sc[0] += 0.1f;
            sc[2] += 0.17f;
        }
        if (aTele == Tele::StaleSample && i == aStale)
        {
            for (int k = 0; k < 3; ++k)
                sc[k] = ec[k]; // the feed catches up with the teleport
            sy = ey;
        }
        if (i == aAbortAtFeed)
            motiondriver::GetUpAbort("test-abort");
        r.s.push_back(feed(kKnock));
    }
    r.statusEnd = motiondriver::RagdollGetUpStatus();
    r.ownsAfter = rag::GetUpOwns();
    return r;
}

float Dist3(const float* a, const float* b) { return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])); }
} // namespace

int main(int argc, char** argv)
{
    // TR-dll male parity: an optional trailing "--body man|woman" (default woman) selects the bundle, skeleton, synthetic clip set and real get-up file.
    int body = 0;
    std::vector<char*> args(argv, argv + argc);
    for (size_t i = 1; i + 1 < args.size(); ++i)
        if (!std::strcmp(args[i], "--body"))
        {
            body = !std::strcmp(args[i + 1], "man") ? 1 : 0;
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i), args.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            break;
        }
    argc = static_cast<int>(args.size());
    argv = args.data();
    const char* const bodyName = body == 1 ? "man" : "woman";
    const std::string skeleton = std::string("player_") + bodyName + "_skeleton";
    if (argc != 3 && argc != 4)
    {
        std::puts("usage: utpp_ragdoll_getup_host_test <data dir> <base database> [<dir with the real utpp_getup_<body>.utpptrav>] [--body man|woman]");
        return 2;
    }
    const std::string data = argv[1];
    motiondriver::Paths paths = DataPaths(data, bodyName);
    Rig rig;
    if (!RigFromDb(paths.files.db, rig))
    {
        std::printf("FAIL %s database / rig\n", bodyName);
        return 3;
    }
    // synthetic clip file: lying pose -> standing pose over 70 frames (2.3 s), all four montages
    g_body = body;
    const gut::ClipSet set = gut::MakeClipSet(body == 0);
    gut::Pose lying[rag::kGuClipCount];
    for (int c = 0; c < rag::kGuClipCount; ++c)
        lying[c] = set.local[c];
    const gut::Pose stand = gut::MakeStandPose(set);
    // one file per test binary: the five get-up host tests start at once under ctest -j and used to overwrite / read each other's file in the
    // cwd (a mutation test that stops early leaves its own file behind; the next run overwrites it)
    char exePath[MAX_PATH];
    GetModuleFileNameA(nullptr, exePath, MAX_PATH);
    std::string exeStem(exePath);
    exeStem = exeStem.substr(exeStem.find_last_of("\\/") + 1);
    exeStem = exeStem.substr(0, exeStem.find_last_of('.'));
    const std::string gpath = std::string("utpp_getup_host_test_") + bodyName + "_" + exeStem + ".utpptrav";
    struct RemoveAtExit
    {
        std::string path;
        ~RemoveAtExit() // a scanner may still hold the new file for a moment: retry
        {
            for (int i = 0; i < 40 && !DeleteFileA(path.c_str()) && GetLastError() != ERROR_FILE_NOT_FOUND; ++i)
                Sleep(50);
        }
    } removeGetUpFile{gpath};
    {
        std::ofstream o(gpath, std::ios::binary);
        o << gut::MakeFile(skeleton.c_str(), 70, true, -1, lying, &stand, true);
    }
    paths.getup = gpath;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(body, paths);
    motiondriver::SetRigBody(body);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &motiondriver::Frame, &motiondriver::OwnsTraversal,
                               &motiondriver::OwnsSlide};
    motionhost::SetMotionHooks(&hooks);
    motiondriver::SetGetUpGapPublisher(&motionhost::PublishGetUpGap); // as Install: the v1 publish right after Begin

    // the REAL file of the bake (A) must load against the real woman database (bone names, parents, rig): avail=ok
    const std::string realName = std::string("/utpp_getup_") + bodyName + ".utpptrav";
    if (argc == 4 && std::ifstream(std::string(argv[3]) + realName).good())
    {
        motiondriver::Paths real = paths;
        real.getup = std::string(argv[3]) + realName;
        motiondriver::SetPathsOverride(body, real);
        motiondriver::SetEnabled(false);
        Check(ResetBase(rig), "real host initialized (real get-up file)");
        motiondriver::SetEnabled(true);
        (void)motiondriver::Step({}, 0, kDt);
        motiondriver::WaitLoader();
        motiondriver::RagdollSetGetUp(true);
        const std::string st = motiondriver::RagdollGetUpStatus();
        std::printf("INFO real file status: %s\n", st.c_str());
        Check(st.find("avail=ok") != std::string::npos,
              std::string("A's real utpp_getup_") + bodyName + ".utpptrav loads in the real " + bodyName + " bundle (avail=ok)", st);
        motiondriver::SetPathsOverride(body, paths);
        motiondriver::SetEnabled(false);
    }
    else
        std::puts("SKIP real get-up file not given / not present");
    const RunOut base = RunGetUp(rig, false, -1);
    std::printf("INFO status: %s\n", base.statusEnd.c_str());
    Check(base.statusEnd.find("avail=ok") != std::string::npos, std::string("the synthetic get-up file loads in the ") + bodyName + " bundle (avail=ok)", base.statusEnd);
    Check(base.planCode == 0 && base.planRms >= 0.0f && base.planRms < 0.05f, "Plan classifies the lying pose as the Back clip with a small rms",
          "code " + std::to_string(base.planCode) + " rms " + std::to_string(base.planRms));
    Check(base.begin, "Begin takes the body", base.statusEnd);
    if (!base.begin || base.s.size() < 200)
    {
        std::printf("host: %d PASS %d FAIL\n", g_pass, g_fail);
        return 1;
    }
    // first get-up feed: published at weight 1 with the v1 gate open, run alive
    Check(base.s[0].stepped && base.s[0].w >= 0.99f && base.s[0].state == 1,
          "the first get-up feed publishes at weight 1 with the gate open (no 'v1-gate' abort)",
          "stepped " + std::to_string(base.s[0].stepped) + " w " + std::to_string(base.s[0].w) + " state " + std::to_string(base.s[0].state));
    bool allW = true, allStepped = true;
    int firstPlay = -1, firstHand = -1, firstIdle = -1;
    for (size_t i = 0; i < base.s.size(); ++i)
    {
        const Sample& s = base.s[i];
        if (s.state > 0)
        {
            allW = allW && s.w >= 0.99f;
            allStepped = allStepped && s.stepped;
        }
        if (s.state == 2 && firstPlay < 0) firstPlay = int(i);
        if (s.state == 3 && firstHand < 0) firstHand = int(i);
        if (s.state == 0 && firstIdle < 0) firstIdle = int(i);
    }
    char d[200];
    std::snprintf(d, sizeof d, "play@%d (%.2fs) handoff@%d (%.2fs) idle@%d (%.2fs)", firstPlay, firstPlay * double(kDt), firstHand,
                  firstHand * double(kDt), firstIdle, firstIdle * double(kDt));
    std::printf("INFO phases: %s\n", d);
    Check(allW && allStepped, "every get-up feed publishes at weight 1 and motion steps it", d);
    Check(firstPlay >= 17 && firstPlay <= 20 && firstHand >= 145 && firstHand <= 150 && firstIdle >= firstHand + 17 && firstIdle <= firstHand + 20,
          "phases on feed time: Blend 0.3 s (clip at half rate), Play to the HandOff marker (clip 2.3 s = 2.45 s), HandOff 0.3 s, then Idle", d);
    Check(base.statusEnd.find("lastEnd=done") != std::string::npos && base.statusEnd.find("aborts=0") != std::string::npos && !base.ownsAfter,
          "the run ends 'done' without an abort and the owner flag is cleared", base.statusEnd);
    // continuity of the published hips (local, entity frame) feed to feed during the blend
    float maxBlendStep = 0.0f;
    for (int i = 1; i < 18; ++i)
        maxBlendStep = std::fmax(maxBlendStep, Dist3(base.s[size_t(i)].hipsLocal, base.s[size_t(i - 1)].hipsLocal));
    std::printf("INFO max hips step in the blend %.4f m\n", static_cast<double>(maxBlendStep));
    Check(maxBlendStep < 0.05f, "blend: the published Hips move smoothly (< 5 cm per feed)", std::to_string(maxBlendStep));

    // world anchoring: the capsule jumps at feed 8; the world Hips follow the same path as without the jump
    const RunOut jump = RunGetUp(rig, true, -1);
    float worst = 0.0f;
    for (size_t i = 0; i < std::min(base.s.size(), jump.s.size()) && int(i) < firstHand; ++i)
    {
        // world Hips = capsule + local (yaw 0): the jump run's capsule is (+0.3, 0, +0.17) from feed 8 on
        float a[3] = {base.s[i].hipsWorld[0], base.s[i].hipsWorld[1], base.s[i].hipsWorld[2]};
        float b[3] = {jump.s[i].hipsWorld[0], jump.s[i].hipsWorld[1], jump.s[i].hipsWorld[2]};
        worst = std::fmax(worst, Dist3(a, b));
    }
    std::printf("INFO world Hips difference with a capsule jump: %.5f m\n", static_cast<double>(worst));
    Check(worst < 0.01f, "world anchoring: a capsule jump of 0.17 m up / 0.1 m sideways does not move the world Hips (< 1 cm)", std::to_string(worst));

    // release continuity (r13b): the entity is teleported to the plan target in the Begin call; world Hips / Head stay on the ragdoll pose from the
    // first publish on, whatever the feed shows
    {
        const RunOut stale = RunGetUp(rig, false, -1, Tele::StaleSample, 1, true);   // a publish before the first feed + one stale feed
        const RunOut stale2 = RunGetUp(rig, false, -1, Tele::StaleSample, 2, false); // two stale feeds
        const RunOut never = RunGetUp(rig, false, -1, Tele::Never, 0, false);        // the teleport never reaches the feed
        const auto worldDiff = [](const RunOut& a, const RunOut& b, size_t aFrom, size_t aTo, float& aHips, float& aHead) {
            aHips = aHead = 0.0f;
            for (size_t i = aFrom; i < aTo && i < a.s.size() && i < b.s.size(); ++i)
            {
                aHips = std::fmax(aHips, Dist3(a.s[i].hipsWorld, b.s[i].hipsWorld));
                aHead = std::fmax(aHead, Dist3(a.headW[i].data(), b.headW[i].data()));
            }
        };
        std::printf("INFO real v1 host right after Begin: publish sequence %llu -> %llu (Tick: %llu), published weight %.2f\n",
                    static_cast<unsigned long long>(stale.beginSeqBefore), static_cast<unsigned long long>(stale.beginSeqAfter),
                    static_cast<unsigned long long>(stale.tickSeqAfter), static_cast<double>(stale.beginWeight));
        const float gapHips = Dist3(stale.gapHipsW, stale.snapHipsW), gapHead = Dist3(stale.gapHeadW, stale.snapHeadW);
        std::printf("INFO gap publish: applied=%d world Hips %.4f m / Head %.4f m from the ragdoll pose (target yaw %.1f deg)\n", stale.gapApplied ? 1 : 0,
                    static_cast<double>(gapHips), static_cast<double>(gapHead), static_cast<double>(stale.tgtYaw / mm::integ::kDegToRad));
        Check(stale.begin && stale.s.size() >= 200, "release continuity runs: Begin takes the body");
        Check(stale.gapApplied && gapHips <= 0.01f && gapHead <= 0.01f,
              "the real base publish right after Begin (sequence advances, weight 1, no Step yet) carries the get-up pose: world Hips / Head on the ragdoll pose (<= 1 cm), no game-pose frame",
              "applied " + std::to_string(stale.gapApplied) + " hips " + std::to_string(gapHips) + " head " + std::to_string(gapHead));
        float h1 = 0.0f, d1 = 0.0f, h2 = 0.0f, d2 = 0.0f, h3 = 0.0f, d3 = 0.0f;
        worldDiff(stale, base, 0, 40, h1, d1);
        worldDiff(stale2, base, 0, 40, h2, d2);
        worldDiff(never, base, 5, 40, h3, d3);
        std::printf("INFO world difference to the plain run, feeds 0..39: stale x1 %.4f / %.4f, stale x2 %.4f / %.4f, no teleport (from feed 5) %.4f / %.4f (Hips / Head)\n",
                    static_cast<double>(h1), static_cast<double>(d1), static_cast<double>(h2), static_cast<double>(d2), static_cast<double>(h3),
                    static_cast<double>(d3));
        Check(h1 <= 0.01f && d1 <= 0.01f,
              "a stale pre-teleport feed sample does not re-express the pose: world Hips / Head continuous (<= 1 cm) over the first 40 feeds",
              std::to_string(h1) + " / " + std::to_string(d1));
        Check(h2 <= 0.01f && d2 <= 0.01f, "two stale feed samples: world Hips / Head continuous (<= 1 cm)", std::to_string(h2) + " / " + std::to_string(d2));
        float stepMax = 0.0f;
        for (size_t i = 1; i < 40 && i < stale.s.size(); ++i)
            stepMax = std::fmax(stepMax, Dist3(stale.s[i].hipsWorld, stale.s[i - 1].hipsWorld));
        stepMax = std::fmax(stepMax, Dist3(stale.s[0].hipsWorld, stale.gapHipsW));
        Check(stepMax < 0.05f, "release: every world Hips step from the gap publish to feed 39 is below 5 cm", std::to_string(stepMax));
        Check(never.s.size() > 30 && never.s[10].state > 0 && h3 <= 0.01f && d3 <= 0.01f,
              "no teleport at all: after the bounded fallback the body stays on the ragdoll pose through the live sample (run alive, world Hips / Head <= 1 cm)",
              "state " + std::to_string(never.s.size() > 10 ? never.s[10].state : -9) + " " + std::to_string(h3) + " / " + std::to_string(d3));
        Check(stale.statusEnd.find(" gap=1 ") != std::string::npos, "status reports the gap publish", stale.statusEnd);
        // a 0.2 s hitch before the first Step: the run survives (grace 0.3 s before the first Step-driven pose) and goes on to its normal end
        const RunOut hitch = RunGetUp(rig, false, -1, Tele::StaleSample, 1, true, 200);
        Check(hitch.begin && hitch.hitchState == 1 && hitch.s.size() > 100 && hitch.s[0].state == 1 && hitch.s[0].stepped &&
                  hitch.statusEnd.find("aborts=0") != std::string::npos,
              "a 0.2 s hitch between the gap publish and the first Step does not end the run (first-Step grace)",
              "state " + std::to_string(hitch.hitchState) + " " + hitch.statusEnd);
    }

    // abort: ends with state -1, the owner flag cleared, the tail blends instead of snapping
    const RunOut ab = RunGetUp(rig, false, 40);
    float jumpAtAbort = 0.0f;
    if (ab.s.size() > 45)
        jumpAtAbort = Dist3(ab.s[40].hipsLocal, ab.s[39].hipsLocal);
    float diffToMm = 0.0f;
    if (ab.s.size() > 60)
        diffToMm = Dist3(ab.s[60].hipsLocal, ab.s[39].hipsLocal);
    std::printf("INFO abort: step at the abort feed %.4f m, hips distance get-up(39) -> mm(60) %.4f m, status %s\n",
                static_cast<double>(jumpAtAbort), static_cast<double>(diffToMm), ab.statusEnd.c_str());
    Check(ab.s.size() > 60 && ab.s[41].state == -1 && !ab.ownsAfter && ab.statusEnd.find("lastEnd=test-abort") != std::string::npos,
          "abort: state -1 after the abort, owner flag cleared, reason logged", ab.statusEnd);
    Check(diffToMm < 0.02f || jumpAtAbort < 0.6f * diffToMm, "abort: the get-up pose fades into MM (no single-feed snap of the whole difference)",
          std::to_string(jumpAtAbort) + " of " + std::to_string(diffToMm));

    // abort in the middle of the HandOff blend: the tail starts from the pose that was SHOWN, so no frame steps more than the blend does
    {
        const int at = firstHand + 9;
        const RunOut mid = RunGetUp(rig, false, at);
        float baseStep = 0.0f, midStep = 0.0f;
        for (int i = firstHand; i < firstHand + 18 && size_t(i + 1) < base.s.size(); ++i)
            baseStep = std::fmax(baseStep, Dist3(base.s[size_t(i + 1)].hipsLocal, base.s[size_t(i)].hipsLocal));
        for (int i = at - 2; i < at + 14 && size_t(i + 1) < mid.s.size(); ++i)
            midStep = std::fmax(midStep, Dist3(mid.s[size_t(i + 1)].hipsLocal, mid.s[size_t(i)].hipsLocal));
        std::printf("INFO abort in HandOff: max step %.4f m (the plain hand-off blend steps %.4f m)\n", static_cast<double>(midStep),
                    static_cast<double>(baseStep));
        Check(mid.s.size() > size_t(at + 14) && mid.s[size_t(at)].state == -1 && !mid.ownsAfter, "abort in HandOff: state -1, owner flag cleared", mid.statusEnd);
        Check(midStep <= baseStep * 1.6f + 0.01f, "abort in HandOff: no frame steps more than the hand-off blend does (the tail starts from the shown pose)",
              std::to_string(midStep) + " vs " + std::to_string(baseStep));
    }

    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("getup host: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

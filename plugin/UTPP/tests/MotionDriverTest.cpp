// port: MotionDriver unit test in isolation (src/MotionDriver.cpp without MotionHost.cpp / RED4ext; report 6.10).
// Usage: utpp_motion_driver_unit_test <data dir>   (<data>\utpp_motion_<body>.utppmm + .chooser.json,
//        <data>\motion\utpp_overlays_<body>.utppov + utpp_style_weights_<body>.txt, <data>\traversal\utpp_traversal_<body>.utpptrav;
//        motion-derived, outside git; a missing file makes the test FAIL).
// Checks: data layout of PathsForBody; mm.motionDriver off -> no load, Step / ApplyToSnapshot no-ops, snapshot bytes untouched (v1 path);
// body unknown / missing file / database of the other body / live topology mismatch / not armed -> adapter fallback reason and no
// v2 pose; woman + man load lazily on the worker, a valid feed gives a finite v2 pose through ApplyToSnapshot with the v2 bone
// map; closed gate (g_effGate != 0), PSM release detail and overlay release -> no v2 pose; body switch reloads (only the
// active body stays loaded); status line fields; toggle off after running -> snapshot untouched; 8c MotionSetFrame pose is a
// caller-owned copy (deterministic use-after-swap check + thread stress with concurrent reloads / body switches).
// 8g trav task A: motiondriver::OwnsTraversal (lock-free flag for the runtime's v1 gate) across accept (set at FeedTraversal,
// before any Step; montage from the .utpptrav version 2 poses), episode end (detail leaves 8 after the handback), reject (gate),
// montage missing, no traversal data, yield mid-episode, toggle off, body switch, re-arm; 10: fault clears it.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/MotionDriver.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/MotionTraversal.hpp" // trav 8g (StartResult)
#include "TestFailFast.hpp"

#include <windows.h>
#include <bcrypt.h>
#pragma comment(lib, "bcrypt")

using namespace utpp;
using mm::adapter::Fallback;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const char* aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat, aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote(std::string("FAIL ") + aWhat + (aDetail.empty() ? "" : " | ") + aDetail);
}

// Independent SHA-256 of a whole file (own read + CNG), compared with the driver's worker-thread status sha=.
std::string FileSha256(const std::string& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return {};
    std::vector<char> buf(size_t(1) << 20);
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    unsigned char d[32] = {};
    bool ok = BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0) == 0;
    ok = ok && BCryptCreateHash(alg, &hash, nullptr, 0, nullptr, 0, 0) == 0;
    while (ok && f)
    {
        f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const std::streamsize got = f.gcount();
        if (got > 0)
            ok = BCryptHashData(hash, reinterpret_cast<PUCHAR>(buf.data()), static_cast<ULONG>(got), 0) == 0;
    }
    ok = ok && BCryptFinishHash(hash, d, sizeof d, 0) == 0;
    if (hash)
        BCryptDestroyHash(hash);
    if (alg)
        BCryptCloseAlgorithmProvider(alg, 0);
    if (!ok)
        return {};
    std::string s;
    char h[3];
    for (unsigned char c : d)
    {
        std::snprintf(h, sizeof h, "%02x", c);
        s += h;
    }
    return s;
}

struct Rig
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    uint32_t bones = 0;
};

// The armed rig of a test = the database bones themselves (name hash + parent), as the live player rig must match them.
bool RigFromDb(const std::string& aPath, Rig& aRig)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 32)
        return false;
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
    mm::motion::MotionDatabase db;
    std::string err;
    if (!db.LoadFromMemory(bytes, rig, err))
        return false;
    aRig.bones = db.NumBones();
    for (uint32_t i = 0; i < db.NumBones(); ++i)
    {
        aRig.hashes.push_back(mm::integ::CNameHash(db.BoneName(i)));
        aRig.parents.push_back(static_cast<int16_t>(db.BoneParent(i)));
    }
    return true;
}

motiondriver::Paths DataPaths(const std::string& aData, const char* aBody)
{
    const std::string b(aBody);
    motiondriver::Paths p;
    p.files.db = aData + "\\utpp_motion_" + b + ".utppmm";
    p.files.chooser = aData + "\\utpp_motion_" + b + ".chooser.json";
    p.files.overlay = aData + "\\motion\\utpp_overlays_" + b + ".utppov";
    p.files.style = aData + "\\motion\\utpp_style_weights_" + b + ".txt";
    p.traversal = aData + "\\traversal\\utpp_traversal_" + b + ".utpptrav";
    return p;
}

bool Exists(const std::string& aPath) { return std::ifstream(aPath).good(); }

// walk forward at ~1.6 m/s, stick forward, actor + camera yaw 0, stand, 60 Hz
motionhost::FeedInput WalkFeed()
{
    motionhost::FeedInput f;
    f.velWorld[1] = 1.6f;
    f.move[1] = 1.0f;
    f.locoState = mm::integ::kLocoStand;
    f.tppActive = true;
    f.dt = 1.0f / 60.0f;
    return f;
}

mm::integ::PoseSnapshot MarkedSnapshot(uint64_t aRig)
{
    mm::integ::PoseSnapshot s{};
    s.weight = 0.75f;
    s.numBones = 3;
    s.firstBone = 2;
    s.rig = aRig;
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
        s.gameIndex[i] = 1000 + static_cast<int32_t>(i);
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones * 7; ++i)
        s.pose[i] = 0.001f * static_cast<float>(i);
    s.upperScale = 0.5f;
    s.upperBits[0] = 0xF0F0F0F0u;
    return s;
}

bool SameBytes(const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

// motionfix item 2: rows of the real 31-column feed dump (the recorded log, header '# cols t dt vx vy
// vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step haveTarget targetYawDeg capX capY capZ effGate
// holdEligible gateRaw upperBody detailed onGround distGround psmFall walk').
struct DumpRow
{
    float t = 0, dt = 0, v[3] = {}, mv[2] = {}, yaw = 0, cam = 0, tgtYaw = 0;
    int32_t loco = 0, upper = -1;
    uint32_t psm = 0;
    bool haveTgt = false, motionOwned = false;
};

std::vector<DumpRow> ReadDump(const std::string& aPath)
{
    std::vector<DumpRow> rows;
    std::ifstream f(aPath);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::vector<std::string> c;
        size_t p = 0;
        while (p < line.size())
        {
            const size_t q = line.find(' ', p);
            c.push_back(line.substr(p, q == std::string::npos ? std::string::npos : q - p));
            if (q == std::string::npos)
                break;
            p = q + 1;
        }
        if (c.size() < 31)
            continue;
        DumpRow r;
        r.t = std::strtof(c[0].c_str(), nullptr);
        r.dt = std::strtof(c[1].c_str(), nullptr);
        for (int k = 0; k < 3; ++k)
            r.v[k] = std::strtof(c[2 + k].c_str(), nullptr);
        r.mv[0] = std::strtof(c[5].c_str(), nullptr);
        r.mv[1] = std::strtof(c[6].c_str(), nullptr);
        r.yaw = std::strtof(c[7].c_str(), nullptr);
        r.cam = std::strtof(c[8].c_str(), nullptr);
        r.loco = static_cast<int32_t>(std::strtol(c[9].c_str(), nullptr, 10));
        r.psm = static_cast<uint32_t>(std::strtoul(c[10].c_str(), nullptr, 10));
        r.motionOwned = c[12] == "-";
        r.haveTgt = c[17] == "1";
        r.tgtYaw = std::strtof(c[18].c_str(), nullptr);
        r.upper = static_cast<int32_t>(std::strtol(c[25].c_str(), nullptr, 10));
        rows.push_back(r);
    }
    return rows;
}

mm::math::Quat SnapQ(const mm::integ::PoseSnapshot& s, uint32_t b)
{
    const float* d = &s.pose[size_t(b) * 7];
    return {d[3], d[4], d[5], d[6]};
}

// model-space rotation / position of the snapshot bones (database order = rig order of the test, parents from the rig);
// bones below kFirstBone (never written) are identity at the origin
void SnapModel(const mm::integ::PoseSnapshot& s, const Rig& aR, std::vector<mm::math::Quat>& aQ, std::vector<mm::math::Vec3>& aP)
{
    aQ.assign(aR.bones, mm::math::Quat{});
    aP.assign(aR.bones, mm::math::Vec3{});
    for (uint32_t b = motiondriver::kFirstBone; b < aR.bones && b < s.numBones; ++b)
    {
        const int p = aR.parents[b];
        const mm::math::Quat q = SnapQ(s, b);
        const float* d = &s.pose[size_t(b) * 7];
        const mm::math::Vec3 t{d[0], d[1], d[2]};
        if (p < 0)
        {
            aQ[b] = q;
            aP[b] = t;
        }
        else
        {
            aQ[b] = mm::math::QuatNormalize(mm::math::QuatMul(aQ[p], q));
            aP[b] = aP[p] + mm::math::QuatRotate(aQ[p], t);
        }
    }
}

std::string StatusField(const std::string& aStatus, const char* aKey)
{
    const size_t p = aStatus.find(aKey);
    if (p == std::string::npos)
        return {};
    const size_t s = p + std::strlen(aKey);
    const size_t e = aStatus.find(' ', s);
    return aStatus.substr(s, e == std::string::npos ? std::string::npos : e - s);
}

// Steps aN frames; returns the number of frames with a v2 pose applied (all finite, bone map = 0..n-1 of the rig)
int RunFrames(int aN, uint64_t aRig, const Rig& aR, uint32_t aEffGate, uint32_t aPsmBlock, bool& aFinite, bool& aMapOk)
{
    int applied = 0;
    aFinite = true;
    aMapOk = true;
    motionhost::FeedInput f = WalkFeed();
    f.psmBlock = aPsmBlock;
    for (int i = 0; i < aN; ++i)
    {
        const bool stepped = motiondriver::Step(f, aEffGate, f.dt);
        mm::integ::PoseSnapshot s = MarkedSnapshot(aRig);
        const bool ap = motiondriver::ApplyToSnapshot(s);
        if (stepped != ap)
            aMapOk = false; // a ready pose must reach the snapshot, and only then
        if (!ap)
            continue;
        ++applied;
        if (s.numBones != aR.bones || s.firstBone != motiondriver::kFirstBone || s.weight != 0.75f || s.rig != aRig)
            aMapOk = false;
        for (uint32_t b = 0; b < s.numBones; ++b)
        {
            // rig = database bones in order: the map is the identity from kFirstBone on (bones below are never written)
            const int32_t want = b >= motiondriver::kFirstBone ? static_cast<int32_t>(b) : -1;
            if (s.gameIndex[b] != want)
                aMapOk = false;
            for (int k = 0; k < 7; ++k)
                if (!std::isfinite(s.pose[size_t(b) * 7 + size_t(k)]))
                    aFinite = false;
            const float* q = &s.pose[size_t(b) * 7 + 3];
            const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (b >= motiondriver::kFirstBone && std::fabs(len - 1.0f) > 1e-3f) // [P] float quaternion normalisation
                aFinite = false;
        }
    }
    return applied;
}

// trav task A: one Step standing still with a PSM detail (-1 = none); true = a v2 pose
bool TravStep(int aDetail, uint32_t aEffGate = 0)
{
    motionhost::FeedInput f = WalkFeed();
    f.velWorld[1] = 0.0f;
    f.move[1] = 0.0f;
    if (aDetail >= 0)
        f.psmBlock = mm::integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << mm::integ::kPsmDetailShift);
    return motiondriver::Step(f, aEffGate, f.dt);
}
// climb event 0.6 m ahead (+Y), ledge aTop m above the feet at the origin, facing +Y
motiondriver::TraversalEvent ClimbEvent(float aAhead, float aTop)
{
    motiondriver::TraversalEvent e;
    e.action = 8;
    e.topPoint[1] = aAhead;
    e.topPoint[2] = aTop;
    e.collisionNormal[1] = -1.0f;
    e.topExtent = 0.8f;
    e.obstacleEnd[1] = aAhead + 0.8f;
    e.obstacleEnd[2] = aTop;
    e.behindPoint[1] = aAhead + 1.1f;
    e.inputDirection[1] = 1.0f;
    e.ownerForward[1] = 1.0f;
    e.climbSpeedModifier = 1.0f;
    return e;
}

// one Step + Apply with a marked snapshot: true when nothing was produced and the snapshot bytes are untouched
bool NoMotionPose(uint64_t aRig, uint32_t aEffGate, uint32_t aPsmBlock)
{
    motionhost::FeedInput f = WalkFeed();
    f.psmBlock = aPsmBlock;
    const bool stepped = motiondriver::Step(f, aEffGate, f.dt);
    const mm::integ::PoseSnapshot ref = MarkedSnapshot(aRig);
    mm::integ::PoseSnapshot s = ref;
    const bool ap = motiondriver::ApplyToSnapshot(s);
    return !stepped && !ap && SameBytes(s, ref);
}

// snapshot in the v1 layout of a rig = database bones (maps to the v2 bones from kFirstBone on): identity rotations, zero translation
mm::integ::PoseSnapshot BaseLikeSnapshot(uint64_t aRig, uint32_t aBones)
{
    mm::integ::PoseSnapshot s{};
    s.weight = 0.75f;
    s.numBones = aBones;
    s.firstBone = motiondriver::kFirstBone;
    s.rig = aRig;
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
        s.gameIndex[i] = (i >= motiondriver::kFirstBone && i < aBones) ? static_cast<int32_t>(i) : -1;
    for (uint32_t i = 0; i < aBones; ++i)
        s.pose[size_t(i) * 7 + 6] = 1.0f;
    return s;
}
// largest |difference| of the pose floats of bones kFirstBone..aBones-1
float MaxPoseDiff(const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b, uint32_t aBones)
{
    float m = 0.0f;
    for (size_t k = size_t(motiondriver::kFirstBone) * 7; k < size_t(aBones) * 7; ++k)
        m = std::fmax(m, std::fabs(a.pose[k] - b.pose[k]));
    return m;
}

// review minor 4 child (--exit-without-shutdown <data>): a finished load leaves the loader thread joinable; returning from main
// without Shutdown must exit cleanly (static destruction detaches it; a joinable std::thread would std::terminate -> abort).
int ExitWithoutShutdownChild(const std::string& aData)
{
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT); // a failure exits with code 3, no dialog
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f = WalkFeed();
    (void)motiondriver::Step(f, 0, f.dt); // starts the worker
    for (int i = 0; i < 3000 && motiondriver::Loads() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // the worker function has returned; the thread stays joinable
    std::printf("CHILD loads=%llu, returning from main without Shutdown\n", static_cast<unsigned long long>(motiondriver::Loads()));
    std::fflush(stdout);
    return motiondriver::Loads() == 1 ? 0 : 4;
}
// P3 RED: A owns the loader's moved thread, B's Shutdown sees an empty handle.
// The pause is inside the test-only hook, after releasing g_m and before A joins.
int ConcurrentShutdownChild(const std::string& aData, bool aFirstShutdown = false)
{
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetEnabled(true);
    const auto f = WalkFeed();
    motiondriver::TestWorkerPause(true);
    (void)motiondriver::Step(f, 0, f.dt); // actual worker is parked before its real LoadBundle, without holding g_m
    motiondriver::TestWaitLoaderPauseAfterMove(true);
    std::atomic<bool> aReturned{false}, bReturned{false};
    std::thread a([&] {
        if (aFirstShutdown)
            motiondriver::Shutdown();
        else
            motiondriver::WaitLoader();
        aReturned.store(true, std::memory_order_release);
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while ((!motiondriver::TestWaitLoaderMoved() || !motiondriver::TestWorkerBlocked()) && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool moved = motiondriver::TestWaitLoaderMoved() && motiondriver::TestWorkerBlocked();
    std::thread b;
    if (moved)
        b = std::thread([&] { motiondriver::Shutdown(); bReturned.store(true, std::memory_order_release); });
    // Wait for B to return while A is provably parked before join, capped to 2 s.
    const auto bDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (moved && !bReturned.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < bDeadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool aCompletedAtObservation = aReturned.load(std::memory_order_acquire);
    const bool premature = moved && bReturned.load(std::memory_order_acquire) && !aCompletedAtObservation;
    const bool workerAliveAtReturn = premature && motiondriver::TestWorkerBlocked();
    const bool shutdownEntered = motiondriver::TestShutdownEntries() >= (aFirstShutdown ? 2u : 1u);
    motiondriver::TestWorkerResume();
    motiondriver::TestWaitLoaderResume(); // cleanup unconditionally, before either join
    a.join();
    if (b.joinable())
        b.join();
    motiondriver::TestWaitLoaderPauseAfterMove(false);
    motiondriver::TestWorkerPause(false);
    int32_t body = -1;
    const bool lateBundle = motiondriver::LoadedBody(body);
    motiondriver::Shutdown(); // stable postcondition even after the intentionally unsafe ordering
    const auto loadsBeforeRestart = motiondriver::Loads();
    (void)motiondriver::Step(f, 0, f.dt);
    motiondriver::WaitLoader();
    const bool staysOff = !motiondriver::Enabled() && motiondriver::Loads() == loadsBeforeRestart && !motiondriver::LoadedBody(body);
    motiondriver::SetEnabled(true);
    (void)motiondriver::Step(f, 0, f.dt);
    motiondriver::WaitLoader();
    const bool restarted = motiondriver::Loads() == loadsBeforeRestart + 1 && motiondriver::LoadedBody(body) && body == 0;
    motiondriver::Shutdown();
    std::printf("P3_RACE moved=%d shutdownBeforeJoin=%d waitACompletedAtObservation=%d finalA=%d finalB=%d\n",
                moved ? 1 : 0, premature ? 1 : 0, aCompletedAtObservation ? 1 : 0,
                aReturned.load() ? 1 : 0, bReturned.load() ? 1 : 0);
    std::fflush(stdout);
    std::printf("P3_LIVE workerBlockedAtShutdownReturn=%d shutdownEntered=%d bundleAfterShutdown=%d\n",
                workerAliveAtReturn ? 1 : 0, shutdownEntered ? 1 : 0, lateBundle ? 1 : 0);
    std::printf("P3_RESTART firstShutdown=%d staysOff=%d restarted=%d\n", aFirstShutdown ? 1 : 0, staysOff ? 1 : 0, restarted ? 1 : 0);
    if (!moved)
        return 3; // inconclusive precondition, not a race assertion
    return !shutdownEntered ? 3 : (premature || lateBundle || !staysOff || !restarted ? 1 : 0);
}
// runs this exe with the child arguments; returns the exit code (-1 = could not start / timeout)
long RunSelfChild(const std::string& aArgs)
{
    char exe[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exe, MAX_PATH);
    std::string cmd = std::string("\"") + exe + "\" " + aArgs;
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    if (!CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &pi))
        return -1;
    const DWORD w = WaitForSingleObject(pi.hProcess, 120000);
    DWORD code = 0;
    GetExitCodeProcess(pi.hProcess, &code);
    if (w != WAIT_OBJECT_0)
        TerminateProcess(pi.hProcess, 99);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return w == WAIT_OBJECT_0 ? static_cast<long>(code) : -1;
}

std::vector<std::string> g_logLines; // motiondriver::SetLogger sink
void CaptureLog(const char* aLine) { g_logLines.emplace_back(aLine); }
size_t LogCount(const char* aNeedle)
{
    size_t n = 0;
    for (const auto& l : g_logLines)
        n += l.find(aNeedle) != std::string::npos ? 1 : 0;
    return n;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_motion_driver_unit_test <data dir>\n");
        return 2;
    }
    if (argc >= 3 && std::string(argv[1]) == "--exit-without-shutdown")
        return ExitWithoutShutdownChild(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--p3-concurrent-shutdown")
        return ConcurrentShutdownChild(argv[2]);
    if (argc >= 3 && std::string(argv[1]) == "--p3-two-shutdowns")
        return ConcurrentShutdownChild(argv[2], true);
    const std::string data = argv[1];
    {
        // G2 / G5 native settings (data-free): walk toggle speed validation + the lock-free target gate, Tier2 Slow flag
        Check(motiondriver::WalkSpeedSetting() == 0.0f && motiondriver::WalkSpeedTarget() == 0.0f, "G2: walk speed default 0 (off)");
        Check(!motiondriver::SetWalkSpeed(std::numeric_limits<float>::quiet_NaN()) && !motiondriver::SetWalkSpeed(-0.1f) && !motiondriver::SetWalkSpeed(3.6f) &&
                  !motiondriver::SetWalkSpeed(std::numeric_limits<float>::infinity()) && motiondriver::WalkSpeedSetting() == 0.0f,
              "G2: SetWalkSpeed rejects NaN / negative / over 3.5 / inf and keeps the previous value");
        Check(motiondriver::SetWalkSpeed(2.0f) && motiondriver::WalkSpeedSetting() == 2.0f, "G2: SetWalkSpeed(2.0) stored");
        Check(motiondriver::WalkSpeedTarget() == 0.0f, "G2: target answers 0 while motion is not driving third-person locomotion");
        Check(motiondriver::SetWalkSpeed(3.5f) && motiondriver::SetWalkSpeed(0.0f) && motiondriver::WalkSpeedSetting() == 0.0f, "G2: 3.5 accepted, 0 switches it off");
        // minimum analog walk speed (CET mm.minWalkSpeed): DLL default 0 (off), 0 .. 1.6, the multiplier 1 while v2 does not drive
        Check(motiondriver::MinAnalogWalkSetting() == 0.0f && motiondriver::AnalogFloorMul(0.0f, 0.09f, 1, false) == 1.0f, "minimum analog walk default 0 (off)");
        Check(!motiondriver::SetMinAnalogWalk(std::numeric_limits<float>::quiet_NaN()) && !motiondriver::SetMinAnalogWalk(-0.1f) &&
                  !motiondriver::SetMinAnalogWalk(1.7f) && !motiondriver::SetMinAnalogWalk(std::numeric_limits<float>::infinity()) &&
                  motiondriver::MinAnalogWalkSetting() == 0.0f,
              "SetMinAnalogWalk rejects NaN / negative / over 1.6 / inf and keeps the previous value");
        Check(motiondriver::SetMinAnalogWalk(0.82f) && motiondriver::MinAnalogWalkSetting() == 0.82f, "SetMinAnalogWalk(0.82) stored");
        Check(motiondriver::AnalogFloorMul(0.0f, 0.09f, 1, false) == 1.0f, "floor multiplier 1 while motion is not driving third-person locomotion");
        Check(motiondriver::SetMinAnalogWalk(1.6f) && motiondriver::SetMinAnalogWalk(0.0f) && motiondriver::MinAnalogWalkSetting() == 0.0f,
              "minimum analog walk: 1.6 accepted, 0 switches it off");
        Check(!motiondriver::Tier2SlowFed(), "G5: Tier2 Slow flag starts false");
        motiondriver::FeedTier2Slow(true);
        Check(motiondriver::Tier2SlowFed(), "G5: Tier2 Slow flag fed");
        motiondriver::FeedTier2Slow(false);
        Check(!motiondriver::Tier2SlowFed(), "G5: Tier2 Slow flag cleared");
    }
    Check(RunSelfChild("--p3-concurrent-shutdown \"" + data + "\"") == 0,
          "P3 concurrent WaitLoader/Shutdown: joins live worker, no late bundle, restart works");
    Check(RunSelfChild("--p3-two-shutdowns \"" + data + "\"") == 0,
          "P3 two concurrent Shutdown calls: both wait, stay disabled, restart works");
    const motiondriver::Paths pw = DataPaths(data, "woman");
    const motiondriver::Paths pm = DataPaths(data, "man");
    for (const motiondriver::Paths* p : {&pw, &pm})
        for (const std::string* f : {&p->files.db, &p->files.chooser, &p->files.overlay, &p->files.style, &p->traversal})
            Check(Exists(*f), "data file present", *f);
    Rig rw, rm;
    // (results computed before Check: argument evaluation order is unspecified, the detail string must see the result)
    const bool rwOk = RigFromDb(pw.files.db, rw);
    const bool rmOk = RigFromDb(pm.files.db, rm);
    Check(rwOk && rw.bones > motiondriver::kFirstBone, "woman rig from the database", std::to_string(rw.bones));
    Check(rmOk && rm.bones > motiondriver::kFirstBone, "man rig from the database", std::to_string(rm.bones));
    const uint64_t kRigPtr = 0x1234567800ull;

    // ---- 1. data layout next to the DLL
    {
        const motiondriver::Paths p = motiondriver::PathsForBody(mm::adapter::Body::Man, "C:\\g\\plugins\\UTPP");
        Check(p.files.db == "C:\\g\\plugins\\UTPP\\data\\utpp_motion_man.utppmm" &&
                  p.files.chooser == "C:\\g\\plugins\\UTPP\\data\\utpp_motion_man.chooser.json" &&
                  p.files.overlay == "C:\\g\\plugins\\UTPP\\data\\utpp_overlays_man.utppov" &&
                  p.files.style == "C:\\g\\plugins\\UTPP\\data\\utpp_style_weights_man.txt" &&
                  p.traversal == "C:\\g\\plugins\\UTPP\\data\\utpp_traversal_man.utpptrav",
              "layout <dll dir>\\data\\ (man)", p.files.db + " " + p.traversal);
        const motiondriver::Paths u = motiondriver::PathsForBody(mm::adapter::Body::Unknown, "C:\\g");
        Check(u.files.db.empty() && u.traversal.empty(), "layout: unknown body -> no paths");
    }

    // ---- 2. default off: v1 untouched, nothing loaded
    motiondriver::SetDataDir(data + "\\does-not-exist");
    Check(!motiondriver::Enabled(), "mm.motionDriver default off");
    Check(motiondriver::Reason() == Fallback::NotRequested, "off -> not-requested", mm::adapter::FallbackName(motiondriver::Reason()));
    Check(motiondriver::SetRigBody(0), "SetRigBody(0) accepted");
    motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
    Check(NoMotionPose(kRigPtr, 0, 0), "off: Step false, snapshot bytes untouched");
    Check(motiondriver::Loads() == 0, "off: no load started", std::to_string(motiondriver::Loads()));
    Check(!motiondriver::SetRigBody(2) && !motiondriver::SetRigBody(-1), "SetRigBody rejects values other than 0 / 1");
    Check(!motiondriver::SetOverlayInputs(2, 0, false, false, 42), "overlay protocol2 rejects invalid resolved Aim pose");
    Check(motiondriver::SetOverlayInputs(2, 0, false, false, 3), "overlay protocol2 accepts atomic Ready1H / Aim2H");
    Check(motiondriver::SetOverlayInputs(2, 0, false, false, 2), "overlay protocol2 accepts manual Aim1H");
    Check(!motiondriver::SetOverlayInputs(42, 0, false, false) && !motiondriver::SetOverlayInputs(0, 7, false, false),
          "SetOverlayInputs rejects pose / aim out of range");
    Check(motiondriver::SetOverlayInputs(9, 0, false, false) && motiondriver::SetOverlayInputs(0, 0, false, false),
          "SetOverlayInputs accepts Neutral Injured 9 and Default");

    // ---- 3. missing file (layout dir without data) -> db-missing, no pose, no crash
    motiondriver::SetLogger(&CaptureLog);
    motiondriver::SetEnabled(true);
    Check(NoMotionPose(kRigPtr, 0, 0), "missing data: no motion pose (load started)");
    motiondriver::WaitLoader();
    Check(NoMotionPose(kRigPtr, 0, 0), "missing data: no motion pose after the load");
    Check(motiondriver::Reason() == Fallback::DbMissing, "missing file -> db-missing", mm::adapter::FallbackName(motiondriver::Reason()));
    Check(StatusField(motiondriver::Status(), "reason=") == "db-missing", "status reason=db-missing", motiondriver::Status());
    (void)NoMotionPose(kRigPtr, 0, 0);
    Check(LogCount("fallback=db-missing") == 1, "6.10.6 db-missing logged exactly once over 3 skipped steps",
          std::to_string(LogCount("fallback=db-missing")) + " of " + std::to_string(g_logLines.size()));

    // ---- 4. database of the other body -> rig-mismatch (not db-invalid)
    {
        motiondriver::Paths wrong = pw;
        wrong.files.db = pm.files.db;
        const uint64_t before = motiondriver::Loads();
        motiondriver::SetPathsOverride(0, wrong); // a path change reloads the active body on the next Step
        (void)NoMotionPose(kRigPtr, 0, 0);
        motiondriver::WaitLoader();
        (void)NoMotionPose(kRigPtr, 0, 0);
        motiondriver::WaitLoader();
        Check(motiondriver::Loads() == before + 1, "path override -> one reload", std::to_string(motiondriver::Loads() - before));
        Check(motiondriver::Reason() == Fallback::RigMismatch, "man database for the woman body -> rig-mismatch",
              mm::adapter::FallbackName(motiondriver::Reason()));
        Check(NoMotionPose(kRigPtr, 0, 0), "rig-mismatch: no motion pose");
    }

    // ---- 5. woman: lazy load, not armed -> rig-mismatch, armed -> v2 pose
    motiondriver::SetPathsOverride(0, pw);
    motiondriver::SetPathsOverride(1, pm);
    motiondriver::OnDisarmed();
    motiondriver::SetRigBody(1);
    motiondriver::SetRigBody(0);
    const uint64_t loads0 = motiondriver::Loads();
    (void)NoMotionPose(kRigPtr, 0, 0); // starts the woman load
    motiondriver::WaitLoader();
    (void)NoMotionPose(kRigPtr, 0, 0);
    motiondriver::WaitLoader();
    int32_t lb = -1;
    bool have = motiondriver::LoadedBody(lb);
    Check(have && lb == 0, "woman bundle loaded", std::to_string(lb));
    Check(motiondriver::Reason() == Fallback::RigMismatch, "woman loaded, not armed -> rig-mismatch",
          mm::adapter::FallbackName(motiondriver::Reason()));
    Check(NoMotionPose(kRigPtr, 0, 0), "not armed: no motion pose");
    motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
    Check(motiondriver::Reason() == Fallback::None, "woman armed -> motion active", mm::adapter::FallbackName(motiondriver::Reason()));
    bool finite = false, mapOk = false;
    int n = RunFrames(120, kRigPtr, rw, 0, 0, finite, mapOk);
    Check(n == 120, "woman: 120 / 120 frames give a motion pose", std::to_string(n));
    Check(finite, "woman: pose finite, quaternions unit");
    Check(mapOk, "woman: motion bone map + numBones / firstBone, base weight / rig kept");
    {
        const std::string st = motiondriver::Status();
        const double p95 = std::atof(StatusField(st, "feedP95us=").c_str());
        Check(StatusField(st, "enabled=") == "1" && StatusField(st, "body=") == "woman" &&
                  StatusField(st, "db=").rfind("loaded", 0) == 0 && StatusField(st, "reason=") == "none" && p95 > 0.0,
              "status: enabled body db reason feed p95", st);
        const std::string pick = StatusField(st, "pick=clip:");
        Check(!pick.empty() && std::atoi(pick.c_str()) >= 0, "status: last pick is a clip", pick);
        const std::string want = FileSha256(pw.files.db);
        Check(want.size() == 64 && StatusField(st, "sha=") == want, "status sha= = SHA-256 of the woman database file",
              StatusField(st, "sha=") + " vs " + want);
        std::printf("INFO woman feed p95 %.1f us (budget %.0f us)\n", p95, static_cast<double>(mm::adapter::Budget::kFeedP95Us));
    }
    // a Tick republish without a feed re-applies the last v2 pose (hold)
    {
        mm::integ::PoseSnapshot s = MarkedSnapshot(kRigPtr);
        Check(motiondriver::ApplyToSnapshot(s) && s.numBones == rw.bones, "republish without a feed keeps the motion pose");
    }
    // snapshot of another rig pointer (stale arm) is not touched
    {
        const mm::integ::PoseSnapshot ref = MarkedSnapshot(kRigPtr + 8);
        mm::integ::PoseSnapshot s = ref;
        Check(!motiondriver::ApplyToSnapshot(s) && SameBytes(s, ref), "snapshot of another rig untouched");
    }

    // ---- 6. yields: closed gate, PSM release detail, overlay release (motionfix item 5: no AirOwns yield, v2 owns air)
    Check(NoMotionPose(kRigPtr, 1u, 0), "g_effGate != 0 -> no motion pose (motion never opens the gate)");
    (void)RunFrames(2, kRigPtr, rw, 0, 0, finite, mapOk);
    {
        const uint32_t detail10 = (10u + 1u) << mm::integ::kPsmDetailShift; // PsmDetailReleases(10)
        Check(mm::integ::PsmDetailReleases(10) && NoMotionPose(kRigPtr, 0, detail10), "PSM release detail 10 -> no motion pose");
        const uint32_t detail31 = (31u + 1u) << mm::integ::kPsmDetailShift;
        Check(NoMotionPose(kRigPtr, 0, detail31), "PSM release detail 31 -> no motion pose");
    }
    motiondriver::SetOverlayInputs(1, 2, true, false); // Rifle, Aiming, release (melee / body carry)
    Check(NoMotionPose(kRigPtr, 0, 0), "overlay release -> no motion pose");
    Check(StatusField(motiondriver::Status(), "skip=") == "overlay-release", "status skip=overlay-release", motiondriver::Status());
    motiondriver::SetOverlayInputs(1, 2, false, false); // Rifle aiming, not released
    n = RunFrames(30, kRigPtr, rw, 0, 0, finite, mapOk);
    Check(n == 30 && finite && mapOk, "release off (Rifle Aiming) -> motion pose again", std::to_string(n));
    motiondriver::SetOverlayInputs(0, 0, false, true); // Default + injured -> Injured overlay
    n = RunFrames(30, kRigPtr, rw, 0, 0, finite, mapOk);
    Check(n == 30 && finite && mapOk, "injured (Default -> Injured overlay) -> motion pose", std::to_string(n));
    motiondriver::SetOverlayInputs(0, 0, false, false);

    // ---- 7. live topology mismatch -> rig-mismatch
    {
        Rig bad = rw;
        bad.parents[motiondriver::kFirstBone + 3] = static_cast<int16_t>(motiondriver::kFirstBone + 5);
        motiondriver::OnArmed(kRigPtr, bad.hashes, bad.parents);
        Check(motiondriver::Reason() == Fallback::RigMismatch || motiondriver::Reason() == Fallback::BoneMap,
              "live rig with a different parent -> rig-mismatch / bone-map", mm::adapter::FallbackName(motiondriver::Reason()));
        Check(NoMotionPose(kRigPtr, 0, 0), "topology mismatch: no motion pose");
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        Check(motiondriver::Reason() == Fallback::None, "re-armed with the right rig -> motion active",
              mm::adapter::FallbackName(motiondriver::Reason()));
    }

    // ---- 8. body switch reloads (only the active body stays loaded)
    motiondriver::SetRigBody(1);
    Check(NoMotionPose(kRigPtr, 0, 0), "body switch: no pose until the man bundle is in");
    motiondriver::WaitLoader();
    (void)NoMotionPose(kRigPtr, 0, 0);
    motiondriver::WaitLoader();
    have = motiondriver::LoadedBody(lb);
    Check(have && lb == 1, "man bundle loaded (woman released)", std::to_string(lb));
    Check(motiondriver::Loads() == loads0 + 2, "one load per body", std::to_string(motiondriver::Loads() - loads0));
    // the woman and man rigs may share bone names + parents: the live topology check cannot tell them apart (INFO only)
    std::printf("INFO man bundle with the woman rig still armed: %s\n", mm::adapter::FallbackName(motiondriver::Reason()));
    motiondriver::OnArmed(kRigPtr, rm.hashes, rm.parents);
    n = RunFrames(60, kRigPtr, rm, 0, 0, finite, mapOk);
    Check(n == 60 && finite && mapOk, "man armed -> motion pose", std::to_string(n));
    Check(StatusField(motiondriver::Status(), "body=") == "man", "status body=man", motiondriver::Status());
    motiondriver::SetRigBody(0);
    (void)NoMotionPose(kRigPtr, 0, 0);
    motiondriver::WaitLoader();
    have = motiondriver::LoadedBody(lb);
    Check(have && lb == 0 && motiondriver::Loads() == loads0 + 3, "switch back reloads the woman",
          std::to_string(lb) + " loads " + std::to_string(motiondriver::Loads() - loads0));
    motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
    n = RunFrames(10, kRigPtr, rw, 0, 0, finite, mapOk);
    Check(n == 10 && finite && mapOk, "woman again -> motion pose", std::to_string(n));

    // ---- 8h. motionfix item 4 (note, male V test 2026-10-13: the man bundle load left no log line, only a
    // transient first-frame fallback=db-missing; LogReasonOnceLocked hid every later reason): a load start / load done line per
    // bundle load (5 so far: missing data, other body's db, woman, man, woman), a fallback line on every reason change (also a
    // reason seen before, and back to none), rate bounded; status + MOTION trace carry body / bundle / rig identity / bone map.
    {
        const auto lastWith = [](const char* aNeedle) -> std::string {
            for (size_t i = g_logLines.size(); i-- > 0;)
                if (g_logLines[i].find(aNeedle) != std::string::npos)
                    return g_logLines[i];
            return {};
        };
        const size_t loadsNow = static_cast<size_t>(motiondriver::Loads());
        Check(loadsNow == loads0 + 3 && LogCount("UTPP mm.motionDriver load start #") == loadsNow &&
                  LogCount("UTPP mm.motionDriver load done #") == loadsNow,
              "8h a load start + a load done line for every bundle load",
              "loads " + std::to_string(loadsNow) + " start " + std::to_string(LogCount("UTPP mm.motionDriver load start #")) + " done " +
                  std::to_string(LogCount("UTPP mm.motionDriver load done #")));
        const std::string ms = lastWith(("load start #" + std::to_string(loads0 + 2) + " body=man why=body-switch from=woman").c_str());
        const std::string md = lastWith(("load done #" + std::to_string(loads0 + 2) + " body=man installed").c_str());
        Check(!ms.empty() && ms.find("utpp_motion_man.utppmm") != std::string::npos, "8h man load start line (body switch, db path)", ms);
        Check(!md.empty() && md.find(" bones=" + std::to_string(rm.bones) + " ") != std::string::npos &&
                  md.find("(body) db=1 chooser=1 matcher=1 ov=1") != std::string::npos && md.find(" dbBytes=0 ") == std::string::npos,
              "8h man load done line: installed, bones, sizes, rig identity of the body, every part loaded", md);
        Check(LogCount("fallback=db-missing cause=loading body=man bundle=woman loading:man") == 1,
              "8h body switch: the transient db-missing while the man bundle loads is logged with cause=loading (db-missing was "
              "logged before, in 3)",
              lastWith("fallback=db-missing"));
        Check(LogCount("fallback=none") >= 3 && lastWith("UTPP mm.motionDriver fallback=").find("fallback=none") != std::string::npos,
              "8h fallback=none (motion path) logged on every return to motion, last reason line = none",
              std::to_string(LogCount("fallback=none")) + " " + lastWith("UTPP mm.motionDriver fallback="));
        // a reason already seen this session (rig-mismatch in 4 / 5 / 7) is logged again on its next change
        (void)RunFrames(70, kRigPtr, rw, 0, 0, finite, mapOk); // > 1 s of feed time: a fresh rate window
        Rig bad = rw;
        bad.parents[motiondriver::kFirstBone + 3] = static_cast<int16_t>(motiondriver::kFirstBone + 5);
        const size_t mis0 = LogCount("fallback=rig-mismatch") + LogCount("fallback=bone-map");
        const size_t none0 = LogCount("fallback=none");
        motiondriver::OnArmed(kRigPtr, bad.hashes, bad.parents);
        (void)NoMotionPose(kRigPtr, 0, 0);
        (void)NoMotionPose(kRigPtr, 0, 0);
        const size_t mis1 = LogCount("fallback=rig-mismatch") + LogCount("fallback=bone-map");
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        const size_t reason0 = LogCount("UTPP mm.motionDriver fallback=");
        n = RunFrames(60, kRigPtr, rw, 0, 0, finite, mapOk);
        const size_t reason1 = LogCount("UTPP mm.motionDriver fallback=");
        Check(mis0 >= 1 && mis1 == mis0 + 1, "8h rig-mismatch seen before is logged again on its next change (once for 2 steps)",
              std::to_string(mis0) + " -> " + std::to_string(mis1));
        Check(n == 60 && reason1 == reason0 + 1 && LogCount("fallback=none") == none0 + 1,
              "8h re-arm: one fallback=none line, none more over 60 motion frames (no per-frame logging)",
              std::to_string(n) + " frames, reason lines +" + std::to_string(reason1 - reason0));
        // rate bound [P]: a reason flapping every feed (bad / good rig, 80 feeds = 1.33 s) writes at most 4 lines per 1 s
        const size_t flap0 = LogCount("UTPP mm.motionDriver fallback=");
        for (int i = 0; i < 40; ++i)
        {
            motiondriver::OnArmed(kRigPtr, bad.hashes, bad.parents);
            (void)NoMotionPose(kRigPtr, 0, 0);
            motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
            (void)RunFrames(1, kRigPtr, rw, 0, 0, finite, mapOk);
        }
        const size_t flapLines = LogCount("UTPP mm.motionDriver fallback=") - flap0;
        motiondriver::OnArmed(kRigPtr, bad.hashes, bad.parents);
        for (int i = 0; i < 70; ++i) // 1.17 s on the bad rig: the window ends, the differing reason is written
            (void)NoMotionPose(kRigPtr, 0, 0);
        const std::string held = lastWith("UTPP mm.motionDriver fallback=");
        Check(flapLines >= 2 && flapLines <= 8, "8h 80 flapping feeds (1.33 s): 2..8 reason lines (4 per 1 s window) [P]",
              std::to_string(flapLines));
        Check(held.find("fallback=none") == std::string::npos && held.find("suppressed=0 ") == std::string::npos,
              "8h after the flapping the held reason is written with the suppressed count", held);
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        n = RunFrames(10, kRigPtr, rw, 0, 0, finite, mapOk);
        Check(n == 10 && lastWith("UTPP mm.motionDriver fallback=").find("fallback=none") != std::string::npos,
              "8h re-armed: motion pose, last reason line none", lastWith("UTPP mm.motionDriver fallback="));
        // status: bundle state, rig identity, live rig, bone map
        const std::string st = motiondriver::Status();
        const std::string mapWant =
            "1:" + std::to_string(rw.bones - motiondriver::kFirstBone) + "/" + std::to_string(rw.bones) + "/miss0/par0";
        char rigHex[32];
        std::snprintf(rigHex, sizeof(rigHex), "0x%llx", static_cast<unsigned long long>(kRigPtr));
        Check(StatusField(st, " bundle=") == "woman/ready/idle" && StatusField(st, " map=") == mapWant &&
                  StatusField(st, " rig=") == rigHex && StatusField(st, " gameBones=") == std::to_string(rw.bones) &&
                  StatusField(st, "rigId=").find("(body)") != std::string::npos,
              "8h status: bundle woman/ready/idle, rigId of the body, armed rig, game bones, bone map", st);
        // MOTION trace: body / bundle / rig / rigId / map fields
        (void)motiondriver::SetWeaponAction(2);
        motiondriver::SetTrace(true);
        const size_t tr0 = LogCount("UTPP MOTION ");
        n = RunFrames(3, kRigPtr, rw, 0, 0, finite, mapOk);
        const std::string tl = lastWith("UTPP MOTION ");
        Check(n == 3 && LogCount("UTPP MOTION ") > tr0 &&
                  tl.find(std::string(" body=woman bundle=woman rig=") + rigHex + " rigId=0x") != std::string::npos &&
                  tl.find(" map=1:" + std::to_string(rw.bones - motiondriver::kFirstBone) + "/" + std::to_string(rw.bones)) !=
                      std::string::npos,
              "8h MOTION trace line carries body / bundle / rig / rigId / bone map", tl);
        Check(tl.find(" act=2 reloadUpper=") != std::string::npos,
              "8h MOTION trace marks the PSM.Weapon Reload action and upper-body blend", tl);
        const size_t reloadTraceCount = LogCount("UTPP MOTION ");
        (void)motiondriver::SetWeaponAction(8);
        n = RunFrames(1, kRigPtr, rw, 0, 0, finite, mapOk);
        const std::string shootTrace = lastWith("UTPP MOTION ");
        Check(n == 1 && LogCount("UTPP MOTION ") > reloadTraceCount &&
                  shootTrace.find(" act=8 reloadUpper=") != std::string::npos &&
                  shootTrace.find(" chg") != std::string::npos,
              "8h MOTION trace emits an immediate changed line for PSM.Weapon Shoot", shootTrace);
        motiondriver::SetTrace(false);
        (void)motiondriver::SetWeaponAction(0);
    }

    // ---- 8b. commit A handovers on a snapshot with the v2 game bones (MarkedSnapshot maps to no v2 bone, so it never blends):
    // toggle off -> v2 -> v1 handover, toggle on -> v1 -> v2 hand-in, both over kDefaultHardOutTime of feed time.
    {
        auto baseLike = [&]() {
            mm::integ::PoseSnapshot s{};
            s.weight = 0.75f;
            s.numBones = rw.bones;
            s.firstBone = motiondriver::kFirstBone;
            s.rig = kRigPtr;
            for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
                s.gameIndex[i] = (i >= motiondriver::kFirstBone && i < rw.bones) ? static_cast<int32_t>(i) : -1;
            for (uint32_t i = 0; i < rw.bones; ++i)
                s.pose[size_t(i) * 7 + 6] = 1.0f; // identity rotation, zero translation
            return s;
        };
        // expected blend frames, same float clock as MotionDriver StepLocked (decrement by dt, blend while left > 0)
        const float dt = WalkFeed().dt;
        auto framesWhileLeft = [&](float aLeft) {
            int k = 0;
            while ((aLeft = std::fmax(0.0f, aLeft - dt)) > 0.0f)
                ++k;
            return k;
        };
        const int wantHo = framesWhileLeft(motionhost::kDefaultHardOutTime);     // toggle off: the first Step already decrements
        const int wantHi = 1 + framesWhileLeft(motionhost::kDefaultHardOutTime); // hand-in: the arming Step sets left = T after the decrement
        const unsigned long long ho0 = std::strtoull(StatusField(motiondriver::Status(), "handover=").c_str(), nullptr, 10);
        motiondriver::SetEnabled(false);
        int changed = 0, firstUnchanged = -1;
        bool contiguous = true;
        for (int i = 0; i < wantHo + 3; ++i)
        {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, 0, f.dt);
            const mm::integ::PoseSnapshot ref = baseLike();
            mm::integ::PoseSnapshot s = ref;
            const bool ap = motiondriver::ApplyToSnapshot(s);
            const bool ch = !SameBytes(s, ref);
            contiguous = contiguous && !st && !ap && (!ch || firstUnchanged < 0);
            if (ch)
                ++changed;
            else if (firstUnchanged < 0)
                firstUnchanged = i;
        }
        const unsigned long long ho1 = std::strtoull(StatusField(motiondriver::Status(), "handover=").c_str(), nullptr, 10);
        Check(changed == wantHo && contiguous && ho1 - ho0 == static_cast<unsigned long long>(wantHo),
              "8b toggle off: motion -> base handover blends the held motion pose into the base snapshot, then base as is",
              "changed " + std::to_string(changed) + "/" + std::to_string(wantHo) + " handover= +" + std::to_string(ho1 - ho0) +
                  (contiguous ? "" : " (not contiguous / Step or Apply true)"));
        const unsigned long long hi0 = std::strtoull(StatusField(motiondriver::Status(), "handin=").c_str(), nullptr, 10);
        motiondriver::SetEnabled(true);
        bool firstIsHeld = false, allApplied = true;
        for (int i = 0; i < wantHi + 3; ++i)
        {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, 0, f.dt);
            const mm::integ::PoseSnapshot held = baseLike();
            mm::integ::PoseSnapshot s = held;
            allApplied = motiondriver::ApplyToSnapshot(s) && st && allApplied;
            if (i == 0) // smoothstep 0: the published pose is the last v1 snapshot on every mapped v2 bone (kFirstBone on)
                firstIsHeld = std::memcmp(&s.pose[size_t(motiondriver::kFirstBone) * 7], &held.pose[size_t(motiondriver::kFirstBone) * 7],
                                          sizeof(float) * 7 * (rw.bones - motiondriver::kFirstBone)) == 0;
        }
        const unsigned long long hi1 = std::strtoull(StatusField(motiondriver::Status(), "handin=").c_str(), nullptr, 10);
        Check(allApplied && firstIsHeld && hi1 - hi0 == static_cast<unsigned long long>(wantHi),
              "8b toggle on: base -> motion hand-in starts from the last base snapshot, blends for kDefaultHardOutTime",
              std::string("first frame = held base ") + (firstIsHeld ? "yes" : "no") + ", handin= +" + std::to_string(hi1 - hi0) + "/" +
                  std::to_string(wantHi) + (allApplied ? "" : " (a frame without a motion pose)"));

        // review minor A: a long weight-0 span (vehicle / menu / gate closure with the rig armed: v1 publishes weight 0 and its
        // pipeline pose stays the pre-span one) must not leave the pre-span snapshot as the hand-in start pose.
        // marked v1 pose: translation x = aX on every v2-mapped bone (identity rotation)
        auto baseMarked = [&](float aWeight, float aX) {
            mm::integ::PoseSnapshot s = baseLike();
            s.weight = aWeight;
            for (uint32_t i = motiondriver::kFirstBone; i < rw.bones; ++i)
                s.pose[size_t(i) * 7] = aX;
            return s;
        };
        auto mappedEq = [&](const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b) {
            return std::memcmp(&a.pose[size_t(motiondriver::kFirstBone) * 7], &b.pose[size_t(motiondriver::kFirstBone) * 7],
                               sizeof(float) * 7 * (rw.bones - motiondriver::kFirstBone)) == 0;
        };
        auto handinNow = [&]() { return std::strtoull(StatusField(motiondriver::Status(), "handin=").c_str(), nullptr, 10); };
        // gate closed: v2 yields (v2 -> v1 handover), v1 publishes the pre-span pose A at weight > 0 until the handover is
        // over, then aSpan feeds at weight 0 (pose B: what v1's pipeline still holds)
        auto closeAndSpan = [&](int aSpan) {
            for (int i = 0; i < wantHo + 3; ++i)
            {
                motionhost::FeedInput f = WalkFeed();
                (void)motiondriver::Step(f, 1u, f.dt);
                mm::integ::PoseSnapshot s = baseMarked(0.75f, 0.5f); // A
                (void)motiondriver::ApplyToSnapshot(s);
            }
            for (int i = 0; i < aSpan; ++i)
            {
                motionhost::FeedInput f = WalkFeed();
                (void)motiondriver::Step(f, 1u, f.dt);
                mm::integ::PoseSnapshot s = baseMarked(0.0f, 0.5f); // B = A (v1 matcher stands still while the gate is closed)
                (void)motiondriver::ApplyToSnapshot(s);
            }
        };
        const int span = static_cast<int>(120.0f / dt); // 2 minutes of feeds at weight 0
        // (1) re-entry straight after the span: no hand-in from the stale pre-span pose (the v1 weight fade-in is the transition)
        closeAndSpan(span);
        const unsigned long long hiA0 = handinNow();
        bool reentryA = true, staleA = false;
        for (int i = 0; i < wantHi + 3; ++i)
        {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, 0, f.dt);
            mm::integ::PoseSnapshot s = baseMarked(0.02f, 0.25f); // v1 fades in again: current pose C
            reentryA = motiondriver::ApplyToSnapshot(s) && st && reentryA;
            if (i == 0)
                staleA = mappedEq(s, baseMarked(0.75f, 0.5f));
        }
        const unsigned long long hiA1 = handinNow();
        Check(reentryA && !staleA && hiA1 == hiA0,
              "8b review minor A: motion re-entry after a 2 min weight-0 span: no hand-in from the pre-span pose",
              std::string("first frame = pre-span pose ") + (staleA ? "yes" : "no") + ", handin= +" + std::to_string(hiA1 - hiA0) +
                  "/0" + (reentryA ? "" : " (a frame without a motion pose)"));
        // (2) the span, then v1 publishes the current pose C at weight > 0 (gate still closed), then v2 re-enters: the hand-in
        // starts from C (not from the pre-span A, not from the re-entry snapshot D)
        closeAndSpan(span);
        {
            motionhost::FeedInput f = WalkFeed();
            (void)motiondriver::Step(f, 1u, f.dt);
            mm::integ::PoseSnapshot s = baseMarked(0.02f, 0.25f); // C
            (void)motiondriver::ApplyToSnapshot(s);
        }
        const unsigned long long hiB0 = handinNow();
        bool reentryB = true, fromC = false;
        for (int i = 0; i < wantHi + 3; ++i)
        {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, 0, f.dt);
            mm::integ::PoseSnapshot s = baseMarked(0.04f, 0.125f); // D
            reentryB = motiondriver::ApplyToSnapshot(s) && st && reentryB;
            if (i == 0)
                fromC = mappedEq(s, baseMarked(0.02f, 0.25f));
        }
        const unsigned long long hiB1 = handinNow();
        Check(reentryB && fromC && hiB1 - hiB0 == static_cast<unsigned long long>(wantHi),
              "8b review minor A: weight-0 span, base publish, motion re-entry: hand-in from the current base pose",
              std::string("first frame = current base ") + (fromC ? "yes" : "no") + ", handin= +" + std::to_string(hiB1 - hiB0) + "/" +
                  std::to_string(wantHi) + (reentryB ? "" : " (a frame without a motion pose)"));
    }

    // ---- 8k. feed budget: a trip pauses the motion pose for a span of feed time (5 s, doubled after each resumed window up to 60 s,
    // back to 5 s after a clean window of Budget::kWindow feeds), hands over / in through the same blends as a toggle, and decides on
    // the smaller of the wall time and the thread's own time of a feed. Feed times come from the test seam (TestSetFeedTimesUs).
    {
        using mm::adapter::Budget;
        const uint32_t fb = motiondriver::kFirstBone;
        auto baseSnap = [&]() {
            mm::integ::PoseSnapshot s{};
            s.weight = 0.75f;
            s.numBones = rw.bones;
            s.firstBone = fb;
            s.rig = kRigPtr;
            for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
                s.gameIndex[i] = (i >= fb && i < rw.bones) ? static_cast<int32_t>(i) : -1;
            for (uint32_t i = 0; i < rw.bones; ++i)
                s.pose[size_t(i) * 7 + 6] = 1.0f;
            return s;
        };
        // largest translation difference over the mapped bones (no quaternion sign ambiguity)
        auto transDiff = [&](const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b) {
            float m = 0.0f;
            for (uint32_t i = fb; i < rw.bones; ++i)
                for (int k = 0; k < 3; ++k)
                    m = std::fmax(m, std::fabs(a.pose[size_t(i) * 7 + k] - b.pose[size_t(i) * 7 + k]));
            return m;
        };
        auto counter = [&](const char* aField) { return std::strtoull(StatusField(motiondriver::Status(), aField).c_str(), nullptr, 10); };
        struct Fed
        {
            bool step = false, budget = false;
            mm::integ::PoseSnapshot pub{};
        };
        auto feed = [&](float aDt) {
            motionhost::FeedInput f = WalkFeed();
            f.dt = aDt;
            Fed r;
            r.step = motiondriver::Step(f, 0, aDt);
            r.budget = std::strcmp(motiondriver::LastSkipReason(), "budget") == 0;
            r.pub = baseSnap();
            (void)motiondriver::ApplyToSnapshot(r.pub);
            return r;
        };
        // motion feeds until the first Step without a motion pose (that Step is consumed); aMotion = motion feeds before it
        auto runUntilSkip = [&](float aDt, int aMax, int& aMotion, Fed* aLast = nullptr) {
            aMotion = 0;
            for (int i = 0; i < aMax; ++i)
            {
                Fed r = feed(aDt);
                if (!r.step)
                {
                    if (aLast)
                        *aLast = r;
                    return r.budget;
                }
                ++aMotion;
            }
            return false;
        };
        // skipped feeds of a pause after its first one, up to the Step that runs the motion pose again (consumed, returned in aResume)
        auto runPause = [&](float aDt, int aMax, int& aPaused, Fed* aResume = nullptr) {
            aPaused = 1; // the Step runUntilSkip consumed
            for (int i = 0; i < aMax; ++i)
            {
                Fed r = feed(aDt);
                if (r.step)
                {
                    if (aResume)
                        *aResume = r;
                    return true;
                }
                if (!r.budget)
                    return false;
                ++aPaused;
            }
            return false;
        };
        // the pause covers aPaused skipped Steps; it ends on the next Step: (aPaused + 1) x dt = pause length, within one feed (+ float
        // rounding of the accumulated clock)
        auto pauseOk = [](int aPaused, float aDt, float aWant) {
            return std::fabs(static_cast<float>(aPaused + 1) * aDt - aWant) <= 1.01f * aDt;
        };
        const float dt60 = 1.0f / 60.0f;
        const unsigned long long trips0 = counter("budgetTrips=");
        const size_t tripLines0 = LogCount("UTPP mm.motionDriver budget trip #"), resumeLines0 = LogCount("UTPP mm.motionDriver budget resume #");
        const size_t budgetLines0 = LogCount("UTPP mm.motionDriver budget ");
        // A. a fresh window
        motiondriver::TestSetFeedTimesUs(100.0f, 100.0f);
        motiondriver::SetEnabled(false);
        motiondriver::SetEnabled(true);
        for (int i = 0; i < 30; ++i)
            (void)feed(dt60);
        // B. the wall time is long (the thread was preempted), the thread's own time short: no trip
        motiondriver::TestSetFeedTimesUs(800.0f, 100.0f);
        int motion = 0;
        const bool skippedB = runUntilSkip(dt60, 2 * static_cast<int>(Budget::kWindow) + 100, motion);
        {
            const std::string st = motiondriver::Status();
            Check(!skippedB && motion == 2 * static_cast<int>(Budget::kWindow) + 100 && counter("budgetTrips=") == trips0 &&
                      std::fabs(std::atof(StatusField(st, "feedP95us=").c_str()) - 100.0) < 0.5 &&
                      std::fabs(std::atof(StatusField(st, "wallP95us=").c_str()) - 800.0) < 0.5,
                  "8k wall time 800 us, own thread time 100 us over 1300 feeds: no trip, feedP95us = thread time, wallP95us = wall time",
                  "motion " + std::to_string(motion) + " feedP95us=" + StatusField(st, "feedP95us=") + " wallP95us=" +
                      StatusField(st, "wallP95us=") + " budgetTrips=" + StatusField(st, "budgetTrips="));
        }
        // C. both long: trip, a 5 s pause with the handover, the resume with the hand-in, the window restarts empty
        const float lastMotionToBase = [&]() {
            Fed r = feed(dt60);
            return transDiff(r.pub, baseSnap());
        }();
        motiondriver::TestSetFeedTimesUs(800.0f, 800.0f);
        const unsigned long long ho0 = counter("handover="), hi0 = counter("handin=");
        Fed prev = feed(dt60), first{};
        int motionC = 0;
        bool skippedC = false;
        for (int i = 0; i < 200; ++i, ++motionC)
        {
            Fed r = feed(dt60);
            if (!r.step)
            {
                first = r;
                skippedC = r.budget;
                break;
            }
            prev = r;
        }
        const std::string stPause = motiondriver::Status();
        Check(skippedC && first.budget && counter("budgetTrips=") == trips0 + 1 && LogCount("UTPP mm.motionDriver budget trip #") == tripLines0 + 1,
              "8k wall and thread time 800 us: the window trips (one trip line), the motion pose pauses (skip=budget)",
              "motion feeds before the pause " + std::to_string(motionC) + " | " + stPause.substr(0, 200));
        Check(StatusField(stPause, "reason=") == "budget" && StatusField(stPause, "skip=") == "budget" &&
                  std::fabs(std::atof(StatusField(stPause, "budgetCoolS=").c_str()) - Budget::kPauseFirstS) < 0.01 &&
                  std::atof(StatusField(stPause, "budgetPauseS=").c_str()) > 0.0,
              "8k status during the pause: reason=budget skip=budget budgetCoolS=5 budgetPauseS > 0",
              "reason=" + StatusField(stPause, "reason=") + " skip=" + StatusField(stPause, "skip=") + " budgetCoolS=" +
                  StatusField(stPause, "budgetCoolS=") + " budgetPauseS=" + StatusField(stPause, "budgetPauseS="));
        const std::string tripLine = [&]() {
            for (size_t i = g_logLines.size(); i-- > 0;)
                if (g_logLines[i].find("budget trip #") != std::string::npos)
                    return g_logLines[i];
            return std::string();
        }();
        Check(tripLine.find(" p95us=800.0 wallP95us=800.0 limit=500 pause=5.0s next=10s") != std::string::npos &&
                  tripLine.find("feedP95us=") == std::string::npos,
              "8k trip line: p95 values, limit, pause, next pause (no status token feedP95us=)", tripLine);
        // pause and resume: the published pose never steps by more than half the motion-to-base distance in one frame
        float maxStep = transDiff(first.pub, prev.pub);
        int changed = transDiff(first.pub, baseSnap()) > 0.0f ? 1 : 0;
        bool contiguous = true;
        int paused = 1;
        Fed resume{};
        bool resumed = false;
        prev = first;
        for (int i = 0; i < 2000 && !resumed; ++i)
        {
            Fed r = feed(dt60);
            maxStep = std::fmax(maxStep, transDiff(r.pub, prev.pub));
            if (r.step)
            {
                resume = r;
                resumed = true;
                break;
            }
            const bool ch = transDiff(r.pub, baseSnap()) > 0.0f;
            contiguous = contiguous && r.budget && (!ch || changed == paused);
            changed += ch ? 1 : 0;
            ++paused;
            prev = r;
        }
        const unsigned long long ho1 = counter("handover=");
        int wantHo = 0;
        for (float left = motionhost::kDefaultHardOutTime; (left = std::fmax(0.0f, left - dt60)) > 0.0f;)
            ++wantHo;
        Check(contiguous && changed >= wantHo && changed <= wantHo + 1 && ho1 - ho0 == static_cast<unsigned long long>(changed),
              "8k pause: the held motion pose hands over to the base pose over kDefaultHardOutTime (same handover as a toggle), then base",
              "changed " + std::to_string(changed) + " (want " + std::to_string(wantHo) + ".." + std::to_string(wantHo + 1) +
                  ") handover= +" + std::to_string(ho1 - ho0) + (contiguous ? "" : " (not contiguous)"));
        Check(resumed && pauseOk(paused, dt60, Budget::kPauseFirstS) && LogCount("UTPP mm.motionDriver budget resume #") == resumeLines0 + 1,
              "8k the pause ends by itself after 5 s of feed time (one resume line)",
              std::string(resumed ? "resumed" : "NOT resumed") + " after " + std::to_string(paused) + " skipped feeds");
        // hand-in: the first frame after the resume is the published base pose on every mapped bone, the blend runs over the toggle time
        int hiFrames = 0;
        bool handinFirstHeld = resumed && transDiff(resume.pub, baseSnap()) == 0.0f, allMotion = resumed;
        prev = resume;
        for (int i = 0; i < 20 && resumed; ++i)
        {
            Fed r = feed(dt60);
            maxStep = std::fmax(maxStep, transDiff(r.pub, prev.pub));
            allMotion = allMotion && r.step;
            prev = r;
        }
        hiFrames = static_cast<int>(counter("handin=") - hi0);
        const int wantHi = 1 + wantHo;
        Check(resumed && handinFirstHeld && allMotion && hiFrames == wantHi,
              "8k resume: the motion pose hands in from the published base pose (same hand-in as a toggle on)",
              std::string("first frame = base ") + (handinFirstHeld ? "yes" : "no") + " handin= +" + std::to_string(hiFrames) + "/" +
                  std::to_string(wantHi) + (allMotion ? "" : " (a frame without a motion pose)"));
        Check(resumed && lastMotionToBase > 0.05f && maxStep <= 0.5f * lastMotionToBase,
              "8k no single-frame pose step at the pause or the resume (largest frame step <= half the motion-to-base distance)",
              "largest step " + std::to_string(maxStep) + " m, motion-to-base " + std::to_string(lastMotionToBase) + " m");
        // D. the same load keeps tripping: every trip follows a full fresh window (600 feeds) and doubles the pause up to 60 s
        // (the resume above already ran 21 motion feeds of the new window)
        {
            const float want[] = {10.0f, 20.0f, 40.0f, 60.0f, 60.0f};
            std::string detail;
            bool ok = true;
            int already = 21;
            for (float w : want)
            {
                int m = 0, p = 0;
                const bool s = runUntilSkip(dt60, 700, m);
                const bool r = s && runPause(dt60, 5000, p);
                ok = ok && s && r && m + already == static_cast<int>(Budget::kWindow) && pauseOk(p, dt60, w);
                detail += std::to_string(m + already) + " feeds -> " + std::to_string(static_cast<float>(p + 1) * dt60) + " s; ";
                already = 1; // the resume Step is the first feed of the next window
            }
            Check(ok, "8k repeated trips: each after a fresh 600-feed window, pauses 10 / 20 / 40 / 60 / 60 s (first 5 s above)", detail);
        }
        // E. a clean window after a resume resets the pause to 5 s
        {
            motiondriver::TestSetFeedTimesUs(100.0f, 100.0f);
            int m = 0, p = 0;
            const bool cleanSkip = runUntilSkip(dt60, static_cast<int>(Budget::kWindow) + 50, m);
            const double cool = std::atof(StatusField(motiondriver::Status(), "budgetCoolS=").c_str());
            motiondriver::TestSetFeedTimesUs(800.0f, 800.0f);
            const bool s = runUntilSkip(dt60, 200, m);
            const bool r = s && runPause(dt60, 5000, p);
            Check(!cleanSkip && std::fabs(cool - Budget::kPauseFirstS) < 0.01 && s && r && pauseOk(p, dt60, Budget::kPauseFirstS),
                  "8k a full window without a trip resets the pause to 5 s",
                  "budgetCoolS after the clean window " + std::to_string(cool) + ", next pause " + std::to_string(static_cast<float>(p + 1) * dt60) + " s");
        }
        // F. the pause runs on feed time: 5 s at 30 / 60 / 120 Hz (window first cleaned at the same rate)
        {
            std::string detail;
            bool ok = true;
            for (float hz : {30.0f, 60.0f, 120.0f})
            {
                const float dt = 1.0f / hz;
                motiondriver::TestSetFeedTimesUs(100.0f, 100.0f);
                int m = 0, p = 0;
                const bool cleanSkip = runUntilSkip(dt, static_cast<int>(Budget::kWindow) + 50, m);
                motiondriver::TestSetFeedTimesUs(800.0f, 800.0f);
                const bool s = runUntilSkip(dt, 200, m);
                const bool r = s && runPause(dt, 5000, p);
                ok = ok && !cleanSkip && s && r && pauseOk(p, dt, Budget::kPauseFirstS);
                detail += std::to_string(static_cast<int>(hz)) + " Hz: " + std::to_string(p) + " skipped feeds = " +
                          std::to_string(static_cast<float>(p + 1) * dt) + " s; ";
            }
            Check(ok, "8k the 5 s pause is frame-rate independent (30 / 60 / 120 Hz, within one feed)", detail);
        }
        const unsigned long long tripsAll = counter("budgetTrips=") - trips0;
        const size_t lines = LogCount("UTPP mm.motionDriver budget ") - budgetLines0;
        Check(tripsAll == 10 && lines == 2 * tripsAll, "8k one trip line and one resume line per trip, nothing per frame",
              std::to_string(tripsAll) + " trips, " + std::to_string(lines) + " budget lines");
        // back to measured feed times and a fresh window for the sections below
        motiondriver::TestSetFeedTimesUs(-1.0f, -1.0f);
        motiondriver::SetEnabled(false);
        motiondriver::SetEnabled(true);
        for (int i = 0; i < 30; ++i)
            (void)feed(dt60);
    }

    // ---- 8c. MotionSetFrame pose lifetime (commit A review): Frame copies the v2 pose into the caller's buffer under the v2 lock, so a
    // bundle swap / free by the worker (reload, body switch) after Frame returned cannot leave the caller with a dangling pointer.
    // Instrumented (TestKeepRetiredPoses): a replaced bundle's pose buffer is NaN-filled and kept alive, so a pointer into the old
    // bundle reads NaN (defined, detectable) instead of freed memory. (a) deterministic: Frame, then a forced reload of the same
    // body completes, then the frame is read; (b) thread stress: frames on this thread while a second thread reloads / switches
    // the body; a race cannot be forced deterministically, (a) is the deterministic proof, (b) checks the fix under contention.
    {
        motiondriver::TestKeepRetiredPoses(true);
        int32_t baseStackIndex[mm::integ::kMaxMMBones];
        for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
            baseStackIndex[i] = (i >= motiondriver::kFirstBone && i < rw.bones) ? static_cast<int32_t>(i) : -1;
        std::array<mm::BoneTransform, mm::integ::kMaxMMBones> buf{};
        auto frameNow = [&](motionhost::MotionSetFrame& aF) {
            motionhost::FeedInput f = WalkFeed();
            if (!motiondriver::Step(f, 0, f.dt))
                return false;
            aF = motionhost::MotionSetFrame{};
            aF.buf = buf.data();
            aF.bufCap = static_cast<uint32_t>(buf.size());
            return motiondriver::Frame(baseStackIndex, rw.bones, aF);
        };
        auto allFinite = [](const mm::BoneTransform* aP, uint32_t aN) {
            for (uint32_t i = 0; i < aN; ++i)
                if (!std::isfinite(aP[i].p.x) || !std::isfinite(aP[i].p.y) || !std::isfinite(aP[i].p.z) ||
                    !std::isfinite(aP[i].q.x) || !std::isfinite(aP[i].q.y) || !std::isfinite(aP[i].q.z) || !std::isfinite(aP[i].q.w))
                    return false;
            return true;
        };
        // no caller buffer -> refused (never a pointer into the bundle)
        {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, 0, f.dt);
            motionhost::MotionSetFrame nb{};
            Check(st && !motiondriver::Frame(baseStackIndex, rw.bones, nb) && nb.pose == nullptr, "8c Frame without a caller buffer -> false, no pointer");
            motionhost::MotionSetFrame small{};
            small.buf = buf.data();
            small.bufCap = rw.bones - 1;
            Check(!motiondriver::Frame(baseStackIndex, rw.bones, small) && small.pose == nullptr, "8c Frame with bufCap < bones -> false");
        }
        // (a) deterministic use-after-swap
        motionhost::MotionSetFrame fr{};
        const bool got = frameNow(fr);
        std::vector<mm::BoneTransform> want(fr.pose, fr.pose + (got ? fr.numBones : 0));
        const size_t retired0 = motiondriver::TestRetiredPoses();
        const uint64_t loadsA = motiondriver::Loads();
        motiondriver::SetPathsOverride(0, pw); // same files: reload of the running body on the next Step
        {
            motionhost::FeedInput f = WalkFeed();
            (void)motiondriver::Step(f, 0, f.dt); // starts the load (v2 keeps running on the old bundle meanwhile)
        }
        motiondriver::WaitLoader(); // the worker swapped the bundle: the old pose buffer is retired (NaN) now
        const bool swapped = motiondriver::Loads() == loadsA + 1 && motiondriver::TestRetiredPoses() == retired0 + 1;
        const bool same = got && fr.numBones == rw.bones && std::memcmp(fr.pose, want.data(), sizeof(mm::BoneTransform) * want.size()) == 0;
        Check(got && fr.pose == buf.data(), "8c Frame pose = the caller's buffer (copy, not a pointer into the bundle)");
        Check(swapped && same && allFinite(fr.pose, fr.numBones),
              "8c frame read after a completed bundle swap: still the copied pose (a pointer into the old bundle reads NaN)",
              std::string("swapped ") + (swapped ? "yes" : "no") + ", same " + (same ? "yes" : "no"));
        // (b) thread stress: >= 2000 frame iterations on this thread while the other thread reloads / switches the body
        std::atomic<bool> stop{false};
        std::atomic<int> switches{0};
        std::thread other([&]() {
            int k = 0;
            while (!stop.load())
            {
                const uint64_t l = motiondriver::Loads();
                if ((k % 3) == 2)
                {
                    motiondriver::SetRigBody(1); // body switch: the man load replaces the woman bundle, then back
                    while (!stop.load() && motiondriver::Loads() == l)
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    motiondriver::SetRigBody(0);
                }
                else
                    motiondriver::SetPathsOverride(0, pw); // reload of the running body
                const uint64_t l2 = motiondriver::Loads();
                while (!stop.load() && motiondriver::Loads() == l2)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                ++k;
                switches.store(k);
            }
        });
        const size_t retiredS0 = motiondriver::TestRetiredPoses();
        int iters = 0, frames = 0, bad = 0, notOwned = 0;
        const auto t0 = std::chrono::steady_clock::now();
        // [P] 2000 iterations and >= 4 completed reloads / switches; 120 s cap (a load takes seconds)
        while ((iters < 2000 || switches.load() < 4) &&
               std::chrono::steady_clock::now() - t0 < std::chrono::seconds(120))
        {
            ++iters;
            motionhost::MotionSetFrame f2{};
            if (!frameNow(f2))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }
            ++frames;
            if (f2.pose != buf.data())
                ++notOwned;
            const std::vector<mm::BoneTransform> first(f2.pose, f2.pose + f2.numBones);
            std::this_thread::yield(); // widen the window between Frame and the read (the worker may swap here)
            if (!allFinite(f2.pose, f2.numBones) ||
                std::memcmp(first.data(), f2.pose, sizeof(mm::BoneTransform) * first.size()) != 0)
                ++bad;
        }
        stop.store(true);
        other.join();
        motiondriver::WaitLoader();
        const size_t retiredS = motiondriver::TestRetiredPoses() - retiredS0;
        std::printf("INFO 8c stress: %d iterations, %d frames, %d reloads / switches, %zu bundles retired, %d bad reads\n", iters,
                    frames, switches.load(), retiredS, bad);
        Check(iters >= 2000 && switches.load() >= 4 && retiredS >= 4 && frames > 0,
              "8c stress ran with concurrent bundle swaps (>= 2000 iterations, >= 4 swaps)",
              std::to_string(iters) + " it, " + std::to_string(switches.load()) + " swaps, " + std::to_string(frames) + " frames");
        Check(bad == 0 && notOwned == 0, "8c stress: every frame pose is the caller's copy, finite and stable across the swaps",
              std::to_string(bad) + " bad, " + std::to_string(notOwned) + " not owned");
        motiondriver::TestKeepRetiredPoses(false);
        // back to the running woman for the sections below
        motiondriver::SetRigBody(0);
        (void)NoMotionPose(kRigPtr, 0, 0);
        motiondriver::WaitLoader();
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        n = RunFrames(10, kRigPtr, rw, 0, 0, finite, mapOk);
        Check(n == 10 && finite && mapOk, "8c after the stress: woman -> motion pose", std::to_string(n));
    }

    // ---- 8d. batch 1b walk toggle: FeedInput.walkToggled (+ haveExtra) -> MotionFeed.wantsToWalk -> motion walk gait. Full stick at
    // 1.6 m/s: walk toggle on -> the pick comes from a *Walk* database, off -> not (full stick = run gait, GaitFromInput);
    // walkToggled without haveExtra (older script) = off.
    {
        std::vector<std::string> dbNames;
        {
            std::ifstream fdb(pw.files.db, std::ios::binary);
            std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(fdb)), std::istreambuf_iterator<char>());
            uint64_t rig = 0;
            if (bytes.size() >= 32)
                std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
            mm::motion::MotionDatabase db;
            std::string err;
            if (db.LoadFromMemory(bytes, rig, err))
                for (uint32_t i = 0; i < db.NumDatabases(); ++i)
                    dbNames.emplace_back(db.DbName(i));
        }
        auto pickDb = [&](bool aHaveExtra, bool aWalk, int& aWalkPicks, int& aFrames) {
            aWalkPicks = aFrames = 0;
            std::string last;
            for (int i = 0; i < 120; ++i)
            {
                motionhost::FeedInput f = WalkFeed();
                f.haveExtra = aHaveExtra;
                f.walkToggled = aWalk;
                if (!motiondriver::Step(f, 0, f.dt))
                    continue;
                mm::integ::PoseSnapshot s = MarkedSnapshot(kRigPtr);
                (void)motiondriver::ApplyToSnapshot(s);
                if (i < 60) // the second second: the gait has settled
                    continue;
                const std::string pick = StatusField(motiondriver::Status(), "pick=");
                const size_t d = pick.find("/db");
                const int db = d == std::string::npos ? -1 : std::atoi(pick.c_str() + d + 3);
                ++aFrames;
                if (db >= 0 && static_cast<size_t>(db) < dbNames.size())
                {
                    last = dbNames[size_t(db)];
                    if (last.find("Walk") != std::string::npos)
                        ++aWalkPicks;
                }
            }
            return last;
        };
        int wWalk = 0, nWalk = 0, wOff = 0, nOff = 0, wOld = 0, nOld = 0;
        const std::string dbWalk = pickDb(true, true, wWalk, nWalk);
        const std::string dbOff = pickDb(true, false, wOff, nOff);
        const std::string dbOld = pickDb(false, true, wOld, nOld);
        std::printf("INFO 8d walk toggle: on %d/%d walk-db frames (last %s), off %d/%d (last %s), no extra %d/%d (last %s)\n", wWalk,
                    nWalk, dbWalk.c_str(), wOff, nOff, dbOff.c_str(), wOld, nOld, dbOld.c_str());
        Check(!dbNames.empty() && nWalk == 60 && wWalk == nWalk, "8d walk toggle on: every settled pick from a Walk database",
              std::to_string(wWalk) + "/" + std::to_string(nWalk) + " last " + dbWalk);
        Check(nOff == 60 && wOff == 0, "8d walk toggle off (full stick): no pick from a Walk database",
              std::to_string(wOff) + "/" + std::to_string(nOff) + " last " + dbOff);
        Check(nOld == 60 && wOld == 0, "8d walkToggled without haveExtra (older script) = off",
              std::to_string(wOld) + "/" + std::to_string(nOld) + " last " + dbOld);
    }

    // ---- 8e. review minor 5: body switch A -> B -> A before B is installed keeps the A bundle mapped (no re-arm needed);
    // (i) no Step in between, (ii) a Step on B started the B load, which is dropped when it completes.
    {
        bool fin = false, mo = false;
        const int n0 = RunFrames(5, kRigPtr, rw, 0, 0, fin, mo);
        const uint64_t l0 = motiondriver::Loads();
        motiondriver::SetRigBody(1);
        motiondriver::SetRigBody(0);
        const int nA = RunFrames(10, kRigPtr, rw, 0, 0, fin, mo);
        Check(n0 == 5 && nA == 10 && fin && mo && motiondriver::Loads() == l0,
              "8e A -> B -> A (no step on B): motion pose at once, no reload, no re-arm",
              std::to_string(nA) + "/10, reason " + mm::adapter::FallbackName(motiondriver::Reason()));
        motiondriver::SetRigBody(1);
        (void)NoMotionPose(kRigPtr, 0, 0); // starts the man load
        motiondriver::SetRigBody(0);         // back before the man bundle is in
        motiondriver::WaitLoader();          // the man load completes and is dropped (body changed while loading)
        int32_t lb2 = -1;
        const bool have2 = motiondriver::LoadedBody(lb2);
        const int nB = RunFrames(10, kRigPtr, rw, 0, 0, fin, mo);
        Check(have2 && lb2 == 0 && nB == 10 && fin && mo && motiondriver::Loads() == l0 + 1,
              "8e A -> B (load started) -> A: B load dropped, woman bundle mapped, motion pose without re-arm",
              std::to_string(nB) + "/10, loaded " + std::to_string(lb2) + ", loads +" + std::to_string(motiondriver::Loads() - l0) +
                  ", reason " + mm::adapter::FallbackName(motiondriver::Reason()));
    }

    // ---- 8f. review major 3: re-entry during a running v2 -> v1 handover hands in from the snapshot that was PUBLISHED last
    // (the handover blend), not from the pure held v2 pose: the first re-entry publish equals the previous publish (smoothstep 0)
    // on every v2 bone. (i) yield (gate) + re-entry, (ii) toggle off + toggle on (SetEnabled(true) mid-handover).
    {
        const float dt = WalkFeed().dt;
        auto feed = [&](uint32_t aGate, mm::integ::PoseSnapshot& aOut) {
            motionhost::FeedInput f = WalkFeed();
            const bool st = motiondriver::Step(f, aGate, dt);
            aOut = BaseLikeSnapshot(kRigPtr, rw.bones);
            const bool ap = motiondriver::ApplyToSnapshot(aOut);
            return st && ap;
        };
        static mm::integ::PoseSnapshot held{}, pub{}, entry{}; // large: off the stack
        for (int variant = 0; variant < 2; ++variant)
        {
            bool running = true;
            for (int i = 0; i < 30; ++i) // v2 runs, the hand-in of the previous section is over
                running = feed(0, held) && running;
            if (variant == 1)
                motiondriver::SetEnabled(false);
            // 5 handover publishes (0.12 s at 60 Hz: s ~ 0.9 on the last one), each without a v2 pose
            bool yielded = true;
            for (int i = 0; i < 5; ++i)
                yielded = !feed(variant == 0 ? 1u : 0u, pub) && yielded;
            const float blendVsHeld = MaxPoseDiff(pub, held, rw.bones);
            const float blendVsBase = MaxPoseDiff(pub, BaseLikeSnapshot(kRigPtr, rw.bones), rw.bones);
            if (variant == 1)
                motiondriver::SetEnabled(true);
            const bool entered = feed(0, entry);
            const float jump = MaxPoseDiff(entry, pub, rw.bones);
            const char* name = variant == 0 ? "8f yield mid-handover -> re-entry" : "8f toggle off -> on mid-handover";
            Check(running && yielded && entered && blendVsHeld > 1e-3f && blendVsBase > 1e-4f,
                  (std::string(name) + ": precondition (motion ran, handover blend differs from the held motion and from base)").c_str(),
                  "blend vs held " + std::to_string(blendVsHeld) + ", vs base " + std::to_string(blendVsBase));
            Check(jump <= 1e-6f, (std::string(name) + ": first re-entry publish = last published blend (continuous)").c_str(),
                  "max |d| " + std::to_string(jump) + " (pure held motion would be " + std::to_string(blendVsHeld) + ")");
        }
    }

    // ---- 8g. trav task A: OwnsTraversal. Traversal data = the version 2 file + the decision table (the montage is forced
    // with TestForceTraversal: Mantle stand, not a clip of the v2 db -> played from the version 2 poses)
    {
        const std::string mantle = "AM_M_Neutral_Traversal_Mantle_1_0_stand_F_Lfoot";
        motiondriver::Paths pt = pw;
        pt.traversal = data + "\\traversal\\utpp_traversal_woman_motion.utpptrav";
        pt.traversalTable = data + "\\traversal\\traversal_table.json";
        motiondriver::SetPathsOverride(0, pt);
        (void)TravStep(-1); // reload with the traversal data
        motiondriver::WaitLoader();
        (void)TravStep(-1);
        const motiondriver::TraversalProbe p0 = motiondriver::ProbeTraversal();
        Check(p0.playable == 34 && p0.playableFromPoses == 20, "8g woman bundle: 34 playable montages, 20 from the version 2 poses",
              std::to_string(p0.playable) + " / " + std::to_string(p0.playableFromPoses));
        for (int i = 0; i < 10; ++i)
            (void)TravStep(-1);
        // (a) accept -> true at FeedTraversal, through the montage and the MM part of the episode; detail leaves 8 -> false
        motiondriver::TestForceTraversal(mantle, false);
        const bool before = motiondriver::OwnsTraversal();
        const bool fed = motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const bool atEvent = motiondriver::OwnsTraversal();
        const motiondriver::TraversalProbe pa = motiondriver::ProbeTraversal();
        int ownedSteps = 0, poseSteps = 0, steps = 0;
        bool handback = false;
        for (int i = 0; i < 240 && !(handback && steps > 0); ++i)
        {
            poseSteps += TravStep(8) ? 1 : 0;
            ownedSteps += motiondriver::OwnsTraversal() ? 1 : 0;
            ++steps;
            handback = motiondriver::ProbeTraversal().handbacks > pa.handbacks;
        }
        for (int i = 0; i < 5; ++i) // MM after the handback, detail still 8
        {
            poseSteps += TravStep(8) ? 1 : 0;
            ownedSteps += motiondriver::OwnsTraversal() ? 1 : 0;
            ++steps;
        }
        const motiondriver::TraversalProbe pb = motiondriver::ProbeTraversal();
        (void)TravStep(-1);
        const bool afterEnd = motiondriver::OwnsTraversal();
        char d[320];
        std::snprintf(d, sizeof d, "before %d, event %d (accepted +%llu result %d from poses %d montage %s), owned %d/%d steps (motion pose %d), handback %d, montage frames %llu, after detail left 8: %d",
                      before ? 1 : 0, atEvent ? 1 : 0, static_cast<unsigned long long>(pa.accepted - p0.accepted), pa.lastResult,
                      pa.fromPoseFile ? 1 : 0, pa.montage, ownedSteps, steps, poseSteps, handback ? 1 : 0,
                      static_cast<unsigned long long>(pb.frames - pa.frames), afterEnd ? 1 : 0);
        Check(fed && !before && atEvent && pa.fromPoseFile && pb.frames > pa.frames + 5,
              "8g task A accept: false before, true right after FeedTraversal (no Step yet), montage played from the version 2 poses", d);
        Check(handback && ownedSteps == steps && poseSteps == steps,
              "8g task A: true on every Step of the episode (montage + MM after the handback, detail 8)", d);
        Check(!afterEnd, "8g task A episode end: false once the detail leaves 8", d);
        // (l) 2026-10-30 body yaw offset (mm.bodyYawOffset 8) through one episode with a provided root frame (offset-root context):
        // weight 1 / ok standing before the climb; 0 / trav on every montage Step and on the handback Step; 0 / trav while the handback
        // blend runs; then a fade-in no faster than dt / 0.25 s per Step (the provided root yaw moves by <= 8 deg x dt / 0.25 s per feed,
        // no snap), back to 1 / ok
        {
            auto boStep = [](int aDetail) {
                motionhost::FeedInput f = WalkFeed();
                f.velWorld[1] = 0.0f;
                f.move[1] = 0.0f;
                f.haveRootContext = true;
                f.nativeRootEnabled = true;
                if (aDetail >= 0)
                    f.psmBlock = mm::integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << mm::integ::kPsmDetailShift);
                (void)motiondriver::Step(f, 0, f.dt);
                return f.dt;
            };
            Check(motiondriver::SetBodyYawOffset(8.0f), "8g body offset: SetBodyYawOffset(8) accepted");
            for (int i = 0; i < 40; ++i)
                (void)boStep(-1);
            const float wPre = motiondriver::BodyYawOffsetWeight();
            const std::string whyPre = motiondriver::BodyYawOffsetReason();
            motiondriver::TestForceTraversal(mantle, false);
            const motiondriver::TraversalProbe pl0 = motiondriver::ProbeTraversal();
            const bool lFed = motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
            int montage = 0, montageBad = 0;
            bool hb = false;
            float wHb = -1.0f;
            std::string whyHb;
            for (int i = 0; i < 240 && !hb; ++i)
            {
                (void)boStep(8);
                hb = motiondriver::ProbeTraversal().handbacks > pl0.handbacks;
                const float w = motiondriver::BodyYawOffsetWeight();
                const std::string why = motiondriver::BodyYawOffsetReason();
                if (hb)
                {
                    wHb = w;
                    whyHb = why;
                }
                else
                {
                    ++montage;
                    montageBad += !(w == 0.0f && why == "trav");
                }
            }
            int travAfter = 0, travBad = 0, decreases = 0;
            float prev = wHb, maxRise = 0.0f, dtMax = 0.0f;
            for (int i = 0; i < 60; ++i) // 5 MM Steps with the detail still 8, then standing (detail left 8)
            {
                dtMax = std::fmax(dtMax, boStep(i < 5 ? 8 : -1));
                const float w = motiondriver::BodyYawOffsetWeight();
                if (motiondriver::BodyYawOffsetReason() == "trav")
                {
                    ++travAfter;
                    travBad += w != 0.0f;
                }
                maxRise = std::fmax(maxRise, w - prev);
                decreases += w < prev;
                prev = w;
            }
            const float wEnd = motiondriver::BodyYawOffsetWeight();
            const std::string whyEnd = motiondriver::BodyYawOffsetReason();
            motiondriver::SetBodyYawOffset(0.0f);
            for (int i = 0; i < 20; ++i)
                (void)boStep(-1); // faded out again (the next cases run with the offset idle)
            char dl[400];
            std::snprintf(dl, sizeof dl, "before %.3f/%s, fed %d, montage Steps %d (bad %d), handback %d at %.3f/%s, trav Steps after %d (bad %d), "
                          "max rise %.4f per Step (bound %.4f), decreases %d, end %.3f/%s",
                          static_cast<double>(wPre), whyPre.c_str(), lFed ? 1 : 0, montage, montageBad, hb ? 1 : 0, static_cast<double>(wHb),
                          whyHb.c_str(), travAfter, travBad, static_cast<double>(maxRise), static_cast<double>(dtMax / 0.25f + 1e-4f), decreases,
                          static_cast<double>(wEnd), whyEnd.c_str());
            Check(wPre == 1.0f && whyPre == "ok", "8g body offset: standing with a root frame before the climb -> weight 1, reason ok", dl);
            Check(lFed && montage > 5 && montageBad == 0, "8g body offset: weight 0, reason trav on every montage Step", dl);
            Check(hb && wHb == 0.0f && whyHb == "trav" && travAfter > 0 && travBad == 0,
                  "8g body offset: weight 0, reason trav on the handback Step and while the handback blend runs", dl);
            Check(maxRise <= dtMax / 0.25f + 1e-4f && decreases == 0 && wEnd == 1.0f && whyEnd == "ok",
                  "8g body offset: after the handback the weight fades in (<= dt / 0.25 s per Step: no root yaw snap), back to 1 / ok", dl);
        }
        // (b) reject (gate on, ledge 4 m away: the warp would teleport) -> false at the event and on the detail-8 Steps
        motiondriver::TestForceTraversal(mantle, true);
        (void)motiondriver::FeedTraversal(ClimbEvent(4.0f, 1.0f));
        const motiondriver::TraversalProbe pr = motiondriver::ProbeTraversal();
        const bool rejEvent = motiondriver::OwnsTraversal();
        int rejOwned = 0, rejPose = 0;
        for (int i = 0; i < 10; ++i)
        {
            rejPose += TravStep(8) ? 1 : 0;
            rejOwned += motiondriver::OwnsTraversal() ? 1 : 0;
        }
        const std::string rejSkip = StatusField(motiondriver::Status(), "skip=");
        (void)TravStep(-1);
        Check(pr.lastResult == static_cast<int32_t>(utpp::mm::traversal::v2::StartResult::Rejected) && !rejEvent && rejOwned == 0 &&
                  rejPose == 0 && rejSkip == "trav-yield",
              "8g task A reject: plan Rejected -> false at the event and on every detail-8 Step (trav-yield)",
              "result " + std::to_string(pr.lastResult) + " owns " + std::to_string(rejEvent ? 1 : 0) + " owned " + std::to_string(rejOwned) +
                  " pose " + std::to_string(rejPose) + " skip " + rejSkip);
        // (c) montage missing -> MontageFailed, false
        motiondriver::TestForceTraversal("AM_not_a_montage", false);
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const motiondriver::TraversalProbe pm0 = motiondriver::ProbeTraversal();
        const bool missEvent = motiondriver::OwnsTraversal();
        (void)TravStep(8);
        const bool missStep = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(pm0.lastResult == static_cast<int32_t>(utpp::mm::traversal::v2::StartResult::MontageFailed) && !missEvent && !missStep,
              "8g task A montage missing: MontageFailed -> false", "result " + std::to_string(pm0.lastResult));
        motiondriver::TestForceTraversal(mantle, false);
        // review minor 5: the live tweaks of the event (UTPPFeedTraversal from GetStaticFloatParameterDefault) are the ones the
        // plan uses; no tweaks / non-finite values -> the 2.31 defaults. Ledge 1.0 m: Low by default (mid 1.2), Mid with 0.8.
        {
            motiondriver::TraversalEvent tw = ClimbEvent(0.6f, 1.0f);
            tw.haveTweaks = true;
            tw.highThreshold = 1.5f;
            tw.midThreshold = 0.8f;
            const float vd[3] = {2.0f, 1.5f, 0.9f}, hd[3] = {0.5f, 0.4f, 0.25f};
            for (int k = 0; k < 3; ++k)
            {
                tw.verticalDuration[k] = vd[k];
                tw.horizontalDuration[k] = hd[k];
            }
            tw.forwardStep = 0.45f;
            (void)motiondriver::FeedTraversal(tw);
            const motiondriver::TraversalProbe pT5 = motiondriver::ProbeTraversal();
            (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f)); // no tweaks: defaults
            const motiondriver::TraversalProbe pD5 = motiondriver::ProbeTraversal();
            motiondriver::TraversalEvent bad = tw;
            bad.midThreshold = std::numeric_limits<float>::quiet_NaN();
            bad.verticalDuration[2] = -1.0f;
            bad.horizontalDuration[2] = std::numeric_limits<float>::infinity();
            bad.forwardStep = 0.0f;
            (void)motiondriver::FeedTraversal(bad); // each bad value -> its default (mid 1.2 -> Low, 0.3 + 0.1, step 0.3)
            const motiondriver::TraversalProbe pB5 = motiondriver::ProbeTraversal();
            (void)TravStep(-1);
            char dt5[300];
            std::snprintf(dt5, sizeof dt5, "live: class %d dur %.3f dest y %.3f; none: class %d dur %.3f dest y %.3f; bad: class %d dur %.3f dest y %.3f",
                          pT5.climbClass, static_cast<double>(pT5.gameDuration), static_cast<double>(pT5.destination[1]), pD5.climbClass,
                          static_cast<double>(pD5.gameDuration), static_cast<double>(pD5.destination[1]), pB5.climbClass,
                          static_cast<double>(pB5.gameDuration), static_cast<double>(pB5.destination[1]));
            Check(pT5.climbClass == 1 && std::fabs(pT5.gameDuration - 1.9f) < 1e-4f && std::fabs(pT5.destination[1] - 1.05f) < 1e-4f &&
                      pD5.climbClass == 2 && std::fabs(pD5.gameDuration - 0.4f) < 1e-4f && std::fabs(pD5.destination[1] - 0.9f) < 1e-4f &&
                      pB5.climbClass == 2 && std::fabs(pB5.gameDuration - 0.4f) < 1e-4f && std::fabs(pB5.destination[1] - 0.9f) < 1e-4f,
                  "8g review minor 5: plan uses the event's live climb tweaks (class, duration, destination); missing / invalid -> 2.31 defaults",
                  dt5);
        }
        // (d) yield mid-episode (closed gate on a montage Step) -> false, rest of the episode yields
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        (void)TravStep(8);
        (void)TravStep(8);
        const bool yPre = motiondriver::OwnsTraversal();
        (void)TravStep(8, 1u); // g_effGate != 0: v2 skips
        const bool yGate = motiondriver::OwnsTraversal();
        const bool yNext = TravStep(8);
        const bool yNextOwns = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(yPre && !yGate && !yNext && !yNextOwns, "8g task A yield mid-episode: a skipped Step ends the ownership, the episode yields",
              std::to_string(yPre) + std::to_string(yGate) + std::to_string(yNext) + std::to_string(yNextOwns));
        // (e) toggle off while the montage plays -> false
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        (void)TravStep(8);
        const bool tPre = motiondriver::OwnsTraversal();
        motiondriver::SetEnabled(false);
        const bool tOff = motiondriver::OwnsTraversal();
        motiondriver::SetEnabled(true);
        (void)TravStep(8);
        const bool tOn = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(tPre && !tOff && !tOn, "8g task A toggle off: false (and stays false after toggling on: the event is dropped)",
              std::to_string(tPre) + std::to_string(tOff) + std::to_string(tOn));
        // (f) body switch with an accepted plan not started yet -> false
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const bool bPre = motiondriver::OwnsTraversal();
        motiondriver::SetRigBody(1);
        const bool bSwitch = motiondriver::OwnsTraversal();
        // review minor 4: an event while the new body's bundle is not loaded (Step: fallback Loading) is not owned
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const bool bLoading = motiondriver::OwnsTraversal();
        motiondriver::SetRigBody(0); // back before any Step: the woman bundle is mapped again
        (void)TravStep(8);
        const bool bBack = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(bPre && !bSwitch && !bBack, "8g task A body switch: false (the plan of the other body is dropped)",
              std::to_string(bPre) + std::to_string(bSwitch) + std::to_string(bBack));
        Check(!bLoading, "8g review minor 4: event while the body's bundle is loading (fallback Loading) -> false",
              std::to_string(bLoading));
        // (g) re-arm while playing -> false
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        (void)TravStep(8);
        const bool aPre = motiondriver::OwnsTraversal();
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        const bool aArm = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(aPre && !aArm, "8g task A re-arm: false", std::to_string(aPre) + std::to_string(aArm));
        // (i)-(k) next, OwnsTraversal contract (main dc221fa): the v1 gate reads it BEFORE v2's Step, so a deferred event
        // (not planned in FeedTraversal: here v2 skips on an overlay release; was the removed SetAirOwns) is owned while v2 can still
        // plan it. (i) planned and played at the first detail-8 Step after the release ended, owned throughout
        motiondriver::SetOverlayInputs(0, 0, true, false); // motionfix item 5: overlay release defers the event (SetAirOwns removed)
        const motiondriver::TraversalProbe pd0 = motiondriver::ProbeTraversal();
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const motiondriver::TraversalProbe pd1 = motiondriver::ProbeTraversal();
        const bool dEvent = motiondriver::OwnsTraversal(); // review minor 4: false while the air path owns the body (Step would skip)
        motiondriver::SetOverlayInputs(0, 0, false, false);
        const bool dAirEnd = motiondriver::OwnsTraversal(); // re-evaluated: v2 can plan it now -> true before the first detail-8 Step
        int dOwned = 0, dPose = 0;
        for (int i = 0; i < 10; ++i)
        {
            dPose += TravStep(8) ? 1 : 0;
            dOwned += motiondriver::OwnsTraversal() ? 1 : 0;
        }
        const motiondriver::TraversalProbe pd2 = motiondriver::ProbeTraversal();
        (void)TravStep(-1);
        const bool dEnd = motiondriver::OwnsTraversal();
        char dd[256];
        std::snprintf(dd, sizeof dd, "plans at the event +%llu, owns at the event %d / after the air ownership %d, plans +%llu accepted +%llu, montage frames +%llu, owned %d/10 (v2 pose %d), after detail left 8: %d",
                      static_cast<unsigned long long>(pd1.plans - pd0.plans), dEvent ? 1 : 0, dAirEnd ? 1 : 0,
                      static_cast<unsigned long long>(pd2.plans - pd0.plans), static_cast<unsigned long long>(pd2.accepted - pd0.accepted),
                      static_cast<unsigned long long>(pd2.frames - pd1.frames), dOwned, dPose, dEnd ? 1 : 0);
        Check(pd1.plans == pd0.plans && !dEvent && dAirEnd && pd2.plans == pd0.plans + 1 && pd2.accepted == pd0.accepted + 1 &&
                  pd2.frames >= pd1.frames + 10 && dOwned == 10 && dPose == 10 && !dEnd,
              "8g contract deferred plan: not planned at the event (air owned, false: Step would skip), true once the air ownership ends, planned + played at the first detail-8 Step, true throughout",
              dd);
        // (j) deferred event that ages out (0.67 s of skipped Steps on detail -1, > kTravEventMaxAge) -> false, never planned
        motiondriver::SetOverlayInputs(0, 0, true, false); // motionfix item 5: overlay release defers the event (SetAirOwns removed)
        const motiondriver::TraversalProbe pj0 = motiondriver::ProbeTraversal();
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const bool jEvent = motiondriver::OwnsTraversal();
        for (int i = 0; i < 40; ++i)
            (void)TravStep(-1);
        const bool jAged = motiondriver::OwnsTraversal();
        motiondriver::SetOverlayInputs(0, 0, false, false);
        int jOwned = 0;
        for (int i = 0; i < 3; ++i)
        {
            (void)TravStep(8);
            jOwned += motiondriver::OwnsTraversal() ? 1 : 0;
        }
        const motiondriver::TraversalProbe pj1 = motiondriver::ProbeTraversal();
        (void)TravStep(-1);
        Check(!jEvent && !jAged && jOwned == 0 && pj1.plans == pj0.plans && pj1.frames == pj0.frames,
              "8g contract deferred event aged out: false at the event (air owned), false after kTravEventMaxAge, never planned",
              std::to_string(jEvent) + std::to_string(jAged) + " owned " + std::to_string(jOwned) + "/3 plans +" +
                  std::to_string(pj1.plans - pj0.plans));
        // (k) the event's detail arrives on a skipped Step (air still owned): v2 cannot start the montage -> given up, false,
        // the episode yields (no late plan)
        motiondriver::SetOverlayInputs(0, 0, true, false); // motionfix item 5: overlay release defers the event (SetAirOwns removed)
        const motiondriver::TraversalProbe pk0 = motiondriver::ProbeTraversal();
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const bool kEvent = motiondriver::OwnsTraversal();
        (void)TravStep(8); // skipped: overlay-release
        const bool kSkip = motiondriver::OwnsTraversal();
        motiondriver::SetOverlayInputs(0, 0, false, false);
        int kOwned = 0, kPose = 0;
        for (int i = 0; i < 3; ++i)
        {
            kPose += TravStep(8) ? 1 : 0;
            kOwned += motiondriver::OwnsTraversal() ? 1 : 0;
        }
        const std::string kSkipWhy = StatusField(motiondriver::Status(), "skip=");
        const motiondriver::TraversalProbe pk1 = motiondriver::ProbeTraversal();
        (void)TravStep(-1);
        Check(!kEvent && !kSkip && kOwned == 0 && kPose == 0 && kSkipWhy == "trav-yield" && pk1.plans == pk0.plans,
              "8g contract deferred event, detail on a skipped Step: given up (false, trav-yield, no late plan)",
              std::to_string(kEvent) + std::to_string(kSkip) + " owned " + std::to_string(kOwned) + "/3 pose " + std::to_string(kPose) +
                  " skip " + kSkipWhy + " plans +" + std::to_string(pk1.plans - pk0.plans));
        // (h) bundle without traversal data -> MontageFailed, false (then the traversal paths again for 10)
        motiondriver::Paths pn = pw;
        pn.traversal.clear();
        pn.traversalTable.clear();
        motiondriver::SetPathsOverride(0, pn);
        (void)TravStep(-1);
        motiondriver::WaitLoader();
        (void)TravStep(-1);
        const motiondriver::TraversalProbe pn0 = motiondriver::ProbeTraversal();
        (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
        const motiondriver::TraversalProbe pn1 = motiondriver::ProbeTraversal();
        const bool noData = motiondriver::OwnsTraversal();
        (void)TravStep(8);
        const bool noDataStep = motiondriver::OwnsTraversal();
        (void)TravStep(-1);
        Check(pn1.plans == pn0.plans + 1 && pn1.lastResult == static_cast<int32_t>(utpp::mm::traversal::v2::StartResult::MontageFailed) &&
                  !noData && !noDataStep,
              "8g task A no traversal data: MontageFailed -> false", "result " + std::to_string(pn1.lastResult));
        motiondriver::SetPathsOverride(0, pt);
        (void)TravStep(-1);
        motiondriver::WaitLoader();
        (void)TravStep(-1);
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
    }

    // ---- 9. toggle off after running -> v1 untouched (also the held pose)
    motiondriver::SetEnabled(false);
    Check(NoMotionPose(kRigPtr, 0, 0), "toggle off: Step false, snapshot bytes untouched");
    {
        const mm::integ::PoseSnapshot ref = MarkedSnapshot(kRigPtr);
        mm::integ::PoseSnapshot s = ref;
        Check(!motiondriver::ApplyToSnapshot(s) && SameBytes(s, ref), "toggle off: republish untouched");
    }
    Check(StatusField(motiondriver::Status(), "enabled=") == "0" && motiondriver::Reason() == Fallback::NotRequested,
          "toggle off: status enabled=0, reason not-requested", motiondriver::Status());
    // v1-only cost of the apply hook with mm.motionDriver off (the default; Main.cpp installs the hooks always): lock + handover check +
    // the commit A copy of the v1 snapshot (start pose of a later hand-in), once per v1 publish. [P] bound: 1% of the v2 feed
    // p95 budget per publish.
    {
        motionhost::FeedInput f = WalkFeed();
        for (int i = 0; i < 12; ++i) // the toggle-off handover is over (kDefaultHardOutTime at 60 Hz)
            (void)motiondriver::Step(f, 0, f.dt);
        mm::integ::PoseSnapshot s = MarkedSnapshot(kRigPtr);
        constexpr int kN = 20000;
        bool anyApplied = false;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kN; ++i)
        {
            s.pose[7] = static_cast<float>(i); // not loop-invariant
            anyApplied = motiondriver::ApplyToSnapshot(s) || anyApplied;
        }
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / kN;
        const double bound = 0.01 * static_cast<double>(mm::adapter::Budget::kFeedP95Us);
        std::printf("INFO mm.motionDriver off: apply hook %.3f us per base publish (%d calls, sizeof(PoseSnapshot) %zu B)\n", us, kN,
                    sizeof(mm::integ::PoseSnapshot));
        Check(!anyApplied && us <= bound, "mm.motionDriver off: apply hook cost per base publish <= 1% of the motion feed budget [P]",
              std::to_string(us) + " us, bound " + std::to_string(bound) + " us");
    }

    // ---- 12. motionfix item 2: aim offset wiring + mm.aimGameUpper + measurement on the real in-game dump
    // (2026-10-13 female V test, PSM UpperBody Aim rows). Checks:
    //  12a) Pistol1H Aiming, aim target 30 deg left of the entity vs straight ahead: published Spine3 model heading differs
    //       by 30 x AimingWeight +- 1 deg [P float + per-frame base motion]; status ao= counts AO frames;
    //  12b) aimGameUpper on + aiming: snapshot keeps the incoming upperScale (0.5) with the v2 upper bits (BuildUpperMask,
    //       the fixture's 3-bone v1 map differs) and the AO is skipped; off: upperScale 1;
    //  12c) measurement (INFO + sanity checks): crosshair error proxy = angle between the WeaponRight barrel (+Y, aimofs
    //       default axis) and the direction weapon -> aim target (60 m along camera yaw, pitch 0 / +-20: the dump has no
    //       pitch), and the spine twist (Spine3 vs Hips heading minus the same in the 12a idle aim straight ahead), AO off
    //       vs AO on.
    //  Mutations UTPP_MOTION_DRIVER_FIX_MUTATION 7 (handback ignored) and 8 (target not forwarded) fail 12a / 12b.
    {
        motiondriver::SetEnabled(true);
        (void)motiondriver::SetRigBody(0);
        motiondriver::OnArmed(kRigPtr, rw.hashes, rw.parents);
        motiondriver::SetAimGameUpper(false);
        motiondriver::SetAimTarget(false, false, nullptr);
        bool fin = false, mok = false;
        (void)RunFrames(5, kRigPtr, rw, 0, 0, fin, mok);
        motiondriver::WaitLoader();
        (void)RunFrames(5, kRigPtr, rw, 0, 0, fin, mok);
        int sp3 = -1, hips = -1, wr = -1;
        {
            std::ifstream f(pw.files.db, std::ios::binary);
            std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            uint64_t rig = 0;
            if (bytes.size() >= 32)
                std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
            mm::motion::MotionDatabase db;
            std::string err;
            if (db.LoadFromMemory(bytes, rig, err))
                for (uint32_t i = 0; i < db.NumBones(); ++i)
                {
                    const std::string_view nm = db.BoneName(i);
                    sp3 = nm == "Spine3" ? static_cast<int>(i) : sp3;
                    hips = nm == "Hips" ? static_cast<int>(i) : hips;
                    wr = nm == "WeaponRight" ? static_cast<int>(i) : wr;
                }
        }
        Check(sp3 >= 0 && hips >= 0, "12 database bones Spine3 / Hips", std::to_string(sp3) + " / " + std::to_string(hips) +
                                                                             " WeaponRight " + std::to_string(wr));
        Check(motiondriver::SetOverlayInputs(2, 2, false, false), "12 overlay inputs Pistol1H / Aiming");
        auto heading = [](const mm::math::Quat& q, int ax) {
            const mm::math::Vec3 a = ax == 0 ? mm::math::Vec3{1, 0, 0} : ax == 1 ? mm::math::Vec3{0, 1, 0} : mm::math::Vec3{0, 0, 1};
            const mm::math::Vec3 v = mm::math::QuatRotate(q, a);
            return std::atan2(-v.x, v.y) * 57.29578f;
        };
        auto wrap = [](float d) {
            while (d > 180.0f)
                d -= 360.0f;
            while (d < -180.0f)
                d += 360.0f;
            return d;
        };
        auto target = [](float aYawDeg, float aPitchDeg, float* aT) {
            const float y = aYawDeg / 57.29578f, p = aPitchDeg / 57.29578f;
            aT[0] = -std::sin(y) * std::cos(p) * 60.0f;
            aT[1] = std::cos(y) * std::cos(p) * 60.0f;
            aT[2] = 1.4f + std::sin(p) * 60.0f;
        };
        auto stepIdle = [&](int aN, mm::integ::PoseSnapshot& aS) {
            bool ap = false;
            for (int i = 0; i < aN; ++i)
            {
                motionhost::FeedInput f = WalkFeed();
                f.velWorld[1] = 0.0f;
                f.move[1] = 0.0f;
                (void)motiondriver::Step(f, 0, f.dt);
                aS = MarkedSnapshot(kRigPtr);
                ap = motiondriver::ApplyToSnapshot(aS);
            }
            return ap;
        };
        std::vector<mm::math::Quat> q12Ref; // 12a idle Pistol1H aim, target straight ahead: reference of the 12c twist
        // 12a) the most horizontal axis of Spine3 in the published pose is used for the heading
        {
            float t0[3], t30[3];
            target(0.0f, 0.0f, t0);
            target(30.0f, 0.0f, t30);
            mm::integ::PoseSnapshot s0, s30;
            motiondriver::SetAimTarget(true, true, t0);
            const bool a0 = stepIdle(90, s0) && stepIdle(1, s0);
            motiondriver::SetAimTarget(true, true, t30);
            const bool a30 = stepIdle(1, s30);
            std::vector<mm::math::Quat> q0, q30;
            std::vector<mm::math::Vec3> p0, p30;
            SnapModel(s0, rw, q0, p0);
            SnapModel(s30, rw, q30, p30);
            q12Ref = q0;
            int ax = 0;
            float best = 1.0f;
            for (int k = 0; k < 3 && sp3 >= 0; ++k)
            {
                const mm::math::Vec3 a = k == 0 ? mm::math::Vec3{1, 0, 0} : k == 1 ? mm::math::Vec3{0, 1, 0} : mm::math::Vec3{0, 0, 1};
                const float z = std::fabs(mm::math::QuatRotate(q0[sp3], a).z);
                if (z < best)
                {
                    best = z;
                    ax = k;
                }
            }
            const std::string st = motiondriver::Status();
            const float w = std::strtof(StatusField(st, " aoW=").c_str(), nullptr);
            const float d = sp3 >= 0 ? wrap(heading(q30[sp3], ax) - heading(q0[sp3], ax)) : 0.0f;
            Check(a0 && a30 && std::fabs(d - 30.0f * w) <= 1.0f, "12a aim target 30 deg left -> published Spine3 heading +30 x AimingWeight",
                  "d " + std::to_string(d) + " w " + std::to_string(w) + " status" + st.substr(st.find(" aim=")));
            motiondriver::SetAimTarget(false, false, nullptr);
            // Fix batch 2 (AO): a lost target no longer ends the aim offset in one frame. The last angles are latched and fade
            // out with the Aiming weight (Pistol1H Aiming here, weight 1: hold 0.35 s, gone at 0.6 s): still ~30 deg one frame
            // later, none after 0.75 s.
            mm::integ::PoseSnapshot sOff1, sOff;
            (void)stepIdle(1, sOff1);
            std::vector<mm::math::Quat> qOff1;
            std::vector<mm::math::Vec3> pOff1;
            SnapModel(sOff1, rw, qOff1, pOff1);
            const float dOff1 = sp3 >= 0 ? wrap(heading(qOff1[sp3], ax) - heading(q0[sp3], ax)) : 0.0f;
            Check(std::fabs(dOff1 - d) <= 1.0f, "12a aim target lost -> the spine yaw is latched for the next frame, no one-frame drop",
                  "d " + std::to_string(d) + " one frame after " + std::to_string(dOff1));
            (void)stepIdle(44, sOff);
            std::vector<mm::math::Quat> qOff;
            std::vector<mm::math::Vec3> pOff;
            SnapModel(sOff, rw, qOff, pOff);
            const float dOff = sp3 >= 0 ? wrap(heading(qOff[sp3], ax) - heading(q0[sp3], ax)) : 99.0f;
            Check(std::fabs(dOff) <= 1.0f, "12a aim target off -> no spine yaw after the fade (heading vs target ahead)", std::to_string(dOff));
        }
        // 12b) aimGameUpper
        {
            float t30[3];
            target(30.0f, 0.0f, t30);
            motiondriver::SetAimTarget(true, true, t30);
            mm::integ::PoseSnapshot sOn, sOffU;
            motiondriver::SetAimGameUpper(true);
            const bool aOn = stepIdle(2, sOn);
            const std::string st = motiondriver::Status();
            motiondriver::SetAimGameUpper(false);
            const bool aOff = stepIdle(1, sOffU);
            bool bitsSpine3 = sp3 >= 0 && ((sOn.upperBits[sp3 >> 5] >> (sp3 & 31)) & 1u) != 0;
            bool bitsHips = hips >= 0 && ((sOn.upperBits[hips >> 5] >> (hips & 31)) & 1u) != 0;
            Check(aOn && sOn.upperScale == 0.5f && bitsSpine3 && !bitsHips && StatusField(st, " gameUpper=") == "1",
                  "12b aimGameUpper on + aiming: incoming upperScale kept, motion upper bits (Spine3 set, Hips clear)",
                  std::to_string(sOn.upperScale) + st.substr(st.find(" aim=")));
            Check(aOff && sOffU.upperScale == 1.0f, "12b aimGameUpper off: weapon overlay -> upperScale 1",
                  std::to_string(sOffU.upperScale));
            motiondriver::SetAimTarget(false, false, nullptr);
        }
        // 13) motionfix item 6: PoseSnapshot::overlayArms (both hands on the weapon), woman, idle, aimGameUpper off
        {
            auto arms = [&](int aPose, int aAim, int aFrames, bool aHandback) {
                (void)motiondriver::SetOverlayInputs(aPose, aAim, false, false);
                motiondriver::SetAimGameUpper(aHandback);
                mm::integ::PoseSnapshot s;
                const bool ap = stepIdle(aFrames, s);
                return ap ? static_cast<int>(s.overlayArms) : -1;
            };
            Check(arms(3, 2, 150, false) == 1, "13a Pistol2H / Aiming steady -> overlayArms 1");
            const int swFirst = arms(1, 2, 1, false);
            Check(swFirst == 0, "13b Pistol2H -> Rifle: first frame (overlay crossfade running) -> overlayArms 0",
                  std::to_string(swFirst));
            Check(arms(1, 2, 150, false) == 1, "13b Rifle / Aiming steady -> overlayArms 1");
            Check(arms(2, 2, 150, false) == 0, "13c Pistol1H / Aiming (left hand off the weapon) -> overlayArms 0");
            Check(arms(3, 2, 150, true) == 0, "13d Pistol2H / Aiming with the upper body handed back to the game -> overlayArms 0");
            Check(arms(3, 2, 150, false) == 1, "13d handback off again -> overlayArms 1");
            Check(arms(0, 0, 400, false) == 0, "13e Default overlay, Relaxed -> overlayArms 0");
            Check(arms(3, 0, 400, false) == 0, "13e Pistol2H / Relaxed -> overlayArms 0");
            (void)arms(2, 2, 150, false); // back to the Pistol1H / Aiming state of 12
        }
        // 12c) measurement on the real dump
        // Reload owns only the upper body even when both user game-upper settings are off. Shoot is carried as a
        // distinct action but must not replace the authored grip until an additive recoil transfer is implemented.
        {
            motiondriver::SetAimGameUpper(false);
            motiondriver::SetWeaponUpper(false);
            Check(motiondriver::SetOverlayInputs(2, 1, false, false), "12d Pistol1H Ready for reload handback");
            // The previous block left the overlay Aiming: flush that aim state so this reload starts from Ready (the
            // aim-interrupting reload has its own ramp, 12d2 below).
            {
                mm::integ::PoseSnapshot sFlush;
                (void)stepIdle(2, sFlush);
            }
            Check(!motiondriver::SetWeaponAction(3) && motiondriver::SetWeaponAction(2),
                  "12d weapon action validates PSM.Weapon state");
            mm::integ::PoseSnapshot sStart, sReload, sReturn, sDone;
            const bool bStart = stepIdle(1, sStart);
            const bool bReload = stepIdle(20, sReload);
            const bool maskedSpine = sp3 >= 0 && ((sReload.upperBits[sp3 >> 5] >> (sp3 & 31)) & 1u) != 0;
            const bool maskedHips = hips >= 0 && ((sReload.upperBits[hips >> 5] >> (hips & 31)) & 1u) != 0;
            const bool maskedWeapon = wr >= 0 && ((sReload.upperBits[wr >> 5] >> (wr & 31)) & 1u) != 0;
            Check(bStart && bReload && sStart.upperScale > 0.0f && sStart.upperScale < 1.0f &&
                  sReload.upperScale == 0.0f && !maskedSpine && !maskedHips && maskedWeapon,
                  "12d Reload fades game arms/hands/weapon in while MM/Neutral torso and legs remain",
                  std::to_string(sStart.upperScale) + " spine=" + std::to_string(maskedSpine) + " weapon=" + std::to_string(maskedWeapon));
            Check(motiondriver::SetWeaponAction(0), "12d Reload exit accepted");
            const bool bReturn = stepIdle(1, sReturn);
            const bool bDone = stepIdle(12, sDone);
            Check(bReturn && bDone && sReturn.upperScale > 0.0f && sReturn.upperScale < 1.0f &&
                  sDone.upperScale == 1.0f, "12d Reload exit fades back to authored pose",
                  std::to_string(sReturn.upperScale));
            // 12d2) a reload that interrupts ADS blends the game arms in over .30s (not .12s) and returns like 12d.
            {
                (void)motiondriver::SetWeaponAction(0);
                Check(motiondriver::SetOverlayInputs(2, 2, false, false), "12d2 Pistol1H Aiming before the reload");
                mm::integ::PoseSnapshot sAim, sEdge, sMid, sFull;
                (void)stepIdle(30, sAim);
                Check(motiondriver::SetOverlayInputs(2, 1, false, false), "12d2 game drops the aim flag with the reload");
                Check(motiondriver::SetWeaponAction(2), "12d2 reload edge from aiming");
                const bool bEdge = stepIdle(1, sEdge);
                const bool bMid = stepIdle(11, sMid);   // about .20s after the edge: the old ramp was already native here
                const bool bFull = stepIdle(20, sFull); // about .53s
                Check(bEdge && bMid && bFull && sEdge.upperScale > 0.9f && sMid.upperScale > 0.05f && sFull.upperScale == 0.0f,
                      "12d2 aim-interrupting reload: arms not yet native at .20s, fully native by .53s",
                      std::to_string(sEdge.upperScale) + " / " + std::to_string(sMid.upperScale) + " / " + std::to_string(sFull.upperScale));
                (void)motiondriver::SetWeaponAction(0);
                mm::integ::PoseSnapshot sOut;
                (void)stepIdle(14, sOut);
                Check(sOut.upperScale == 1.0f, "12d2 exit still returns the authored pose within .23s", std::to_string(sOut.upperScale));
            }
            // 12d3) a RIFLE reload keeps the former .12s ramp even when it interrupts ADS: the .30s ramp, the aim hold
            //       and the slide roll hold are measured on Nue pistols only.
            {
                (void)motiondriver::SetWeaponAction(0);
                Check(motiondriver::SetOverlayInputs(1, 2, false, false), "12d3 Rifle Aiming before the reload");
                mm::integ::PoseSnapshot sAim, sEdge, sMid;
                (void)stepIdle(30, sAim);
                Check(motiondriver::SetOverlayInputs(1, 1, false, false), "12d3 game drops the aim flag with the reload");
                Check(motiondriver::SetWeaponAction(2), "12d3 rifle reload edge from aiming");
                const bool bEdge = stepIdle(1, sEdge);
                const bool bMid = stepIdle(11, sMid); // about .20s after the edge
                Check(bEdge && bMid && sMid.upperScale == 0.0f,
                      "12d3 rifle reload from aiming: arms fully native at .20s (former .12s ramp)",
                      std::to_string(sEdge.upperScale) + " / " + std::to_string(sMid.upperScale));
                (void)motiondriver::SetWeaponAction(0);
                mm::integ::PoseSnapshot sOut;
                (void)stepIdle(14, sOut);
                Check(motiondriver::SetOverlayInputs(2, 1, false, false), "12d3 back to Pistol1H Ready");
            }
            // 12d5) Fix batch 2 (AO) x reload contacts: the aim offset goes down with the hand-over of the arms to the game (the
            //       driver sets OverlayFrameInputs::aoScale while the reload envelope runs). An ADS reload keeps the aim pose for
            //       the 0.30 s aim hold: the AO share of the published Spine3 heading (30 deg to the target, minus the same
            //       scenario without a target) is 30 deg at the aim, still ~28 at the edge, about half of it half way through the
            //       ramp and none once the game owns the arms (the reload contacts then meet a torso the AO no longer tilts).
            //       Mutation UTPP_POSEV2_AO_MUTATION 8 keeps the AO and fails.
            if (sp3 >= 0 && hips >= 0 && q12Ref.size() > static_cast<size_t>(sp3))
            {
                int ax = 0;
                float best = 1.0f;
                for (int k = 0; k < 3; ++k)
                {
                    const mm::math::Vec3 a = k == 0 ? mm::math::Vec3{1, 0, 0} : k == 1 ? mm::math::Vec3{0, 1, 0} : mm::math::Vec3{0, 0, 1};
                    const float z = std::fabs(mm::math::QuatRotate(q12Ref[sp3], a).z);
                    if (z < best)
                    {
                        best = z;
                        ax = k;
                    }
                }
                auto spineYaw = [&](const mm::integ::PoseSnapshot& aS) {
                    std::vector<mm::math::Quat> q;
                    std::vector<mm::math::Vec3> p;
                    SnapModel(aS, rw, q, p);
                    return wrap(heading(q[sp3], ax) - heading(q[hips], ax)); // spine twist: the body turn toward the target cancels
                };
                // aim, edge, half way, full hand-over
                auto adsReload = [&](bool aTarget, float aOut[4]) {
                    (void)motiondriver::SetWeaponAction(0);
                    (void)motiondriver::SetOverlayInputs(2, 2, false, false);
                    float t30[3];
                    target(30.0f, 0.0f, t30);
                    motiondriver::SetAimTarget(aTarget, aTarget, aTarget ? t30 : nullptr);
                    mm::integ::PoseSnapshot sAim, sEdge, sHalf, sFull, sOut;
                    bool ok = stepIdle(90, sAim);
                    ok = motiondriver::SetOverlayInputs(2, 1, false, false) && ok;
                    ok = motiondriver::SetWeaponAction(2) && ok;
                    ok = stepIdle(1, sEdge) && ok;
                    ok = stepIdle(8, sHalf) && ok;  // about .15 s: half of the .30 s ramp
                    ok = stepIdle(25, sFull) && ok; // about .58 s: the game owns the arms
                    aOut[0] = spineYaw(sAim);
                    aOut[1] = spineYaw(sEdge);
                    aOut[2] = spineYaw(sHalf);
                    aOut[3] = spineYaw(sFull);
                    motiondriver::SetAimTarget(false, false, nullptr);
                    (void)motiondriver::SetWeaponAction(0);
                    ok = stepIdle(60, sOut) && ok;
                    (void)motiondriver::SetOverlayInputs(2, 1, false, false);
                    ok = stepIdle(120, sOut) && ok; // back to Ready, the next scenario starts from the same state
                    return ok;
                };
                float withAo[4]{}, noAo[4]{};
                const bool okRuns = adsReload(true, withAo) && adsReload(false, noAo);
                float d[4];
                for (int k = 0; k < 4; ++k)
                    d[k] = wrap(withAo[k] - noAo[k]);
                Check(okRuns && d[0] > 25.0f && d[1] > 23.0f && d[2] > 8.0f && d[2] < 24.0f && std::fabs(d[3]) <= 3.0f,
                      "12d5 ADS reload: the AO (30 deg) goes down with the hand-over of the arms and is gone when the game owns them",
                      "aim " + std::to_string(d[0]) + " edge " + std::to_string(d[1]) + " half " + std::to_string(d[2]) + " full " + std::to_string(d[3]));
                // 12d6) the exit tail: a reload that ends after 0.20 s (inside the aim hold, the CET target stays) hands the arms back over
                //       0.20 s (g_reloadUpperWeight 0.65 -> 0). The AO comes back with them: the published spine twist (minus the same
                //       scenario without a target) changes by <= 6 deg per frame, not by the 18 deg of the hidden part in the frame
                //       the reload action ends (the first gate). Mutation UTPP_POSEV2_AO_MUTATION 13 is that gate.
                auto shortReload = [&](bool aTarget, float aOut[31]) {
                    (void)motiondriver::SetWeaponAction(0);
                    (void)motiondriver::SetOverlayInputs(2, 2, false, false);
                    float t30[3];
                    target(30.0f, 0.0f, t30);
                    motiondriver::SetAimTarget(aTarget, aTarget, aTarget ? t30 : nullptr);
                    mm::integ::PoseSnapshot sAim, sOne, sOut;
                    bool ok = stepIdle(90, sAim);
                    ok = motiondriver::SetOverlayInputs(2, 1, false, false) && ok;
                    ok = motiondriver::SetWeaponAction(2) && ok;
                    ok = stepIdle(12, sOne) && ok; // the reload lasts 0.20 s only
                    aOut[0] = spineYaw(sOne);
                    ok = motiondriver::SetWeaponAction(0) && ok;
                    for (int i = 1; i <= 30; ++i)
                    {
                        ok = stepIdle(1, sOne) && ok;
                        aOut[i] = spineYaw(sOne);
                    }
                    motiondriver::SetAimTarget(false, false, nullptr);
                    ok = stepIdle(60, sOut) && ok;
                    ok = stepIdle(120, sOut) && ok;
                    return ok;
                };
                float tailWith[31]{}, tailNo[31]{};
                const bool okTail = shortReload(true, tailWith) && shortReload(false, tailNo);
                float tailStep = 0.0f, prevD = wrap(tailWith[0] - tailNo[0]);
                for (int i = 1; i <= 30; ++i)
                {
                    const float dd = wrap(tailWith[i] - tailNo[i]);
                    tailStep = std::max(tailStep, std::fabs(dd - prevD));
                    prevD = dd;
                }
                Check(okTail && tailStep <= 6.0f,
                      "12d6 a reload that ends after 0.20 s: the AO returns with the arms over the 0.20 s exit tail (<= 6 deg per frame)",
                      "max one-frame change " + std::to_string(tailStep) + " deg, after 30 frames " + std::to_string(prevD));
            }
            // 12d4) the contact-correction episode of an authored reload is classified once, on the reload edge, and the decision
            //       is kept for the whole reload. It must read the PSM stance of the edge feed (ChooserState::stanceRaw), not the
            //       0.15 s debounced chooser stance (fix/slide-exit): crouch -> stand and R inside the filter time admits the
            //       correction; stand -> crouch and R inside the filter time does not (the debounced stance would admit a ~.35 s
            //       pulse in the crouch); a plain standing reload is unchanged. Read from the published snapshot
            //       (reloadContactsEligible / reloadContactsWeight) of the real driver. Mutation
            //       UTPP_RELOAD_EPISODE_STANCE_MUTATION=1 (tests/reloadfix.cmake) reads cs.stance and MUST fail 12d4b / 12d4c.
            {
                struct Run
                {
                    int eligible = 0;
                    float maxWeight = 0.0f, endWeight = 0.0f;
                    bool applied = true, authored = false;
                };
                // aState: 0 Stand 1, 1 Crouch 3, 2 CrouchSprint 30 (a stand for the body). P1 (fix batch 2): a clean Stand <-> Crouch edge reaches
                // the chooser on its own feed, so the debounced stance this test needs to differ from the raw one only exists on an edge that
                // keeps the filter: Crouch 3 <-> CrouchSprint 30.
                const auto feedStep = [&](int aState, mm::integ::PoseSnapshot& aS) {
                    motionhost::FeedInput f = WalkFeed();
                    f.velWorld[1] = f.move[1] = 0.0f;
                    f.locoState = aState == 1 ? mm::integ::kLocoCrouch : mm::integ::kLocoStand;
                    f.psmBlock = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn; // drawn pistol: the reload domain
                    if (aState == 2)
                        f.psmBlock |= mm::integ::kPsmDetailed | (uint32_t(30 + 1) << mm::integ::kPsmDetailShift);
                    (void)motiondriver::Step(f, 0, f.dt);
                    aS = MarkedSnapshot(kRigPtr);
                    return motiondriver::ApplyToSnapshot(aS);
                };
                // aFrom: stance held for 1 s (the filter settles); aGap frames of aTo (still inside the 0.15 s filter: the
                // edge feed is frame aGap + 1 of the change, <= .133 s); then the reload edge and 40 reload frames (.67 s).
                const auto scenario = [&](int aFrom, int aTo, int aGap) {
                    Run r;
                    mm::integ::PoseSnapshot s;
                    (void)motiondriver::SetWeaponAction(0);
                    for (int i = 0; i < 60; ++i)
                        r.applied = feedStep(aFrom, s) && r.applied;
                    for (int i = 0; i < aGap; ++i)
                        r.applied = feedStep(aTo, s) && r.applied;
                    (void)motiondriver::SetWeaponAction(2);
                    for (int i = 0; i < 40; ++i)
                    {
                        r.applied = feedStep(aTo, s) && r.applied;
                        r.eligible += s.reloadContactsEligible ? 1 : 0;
                        r.maxWeight = std::fmax(r.maxWeight, s.reloadContactsWeight);
                        r.endWeight = s.reloadContactsWeight;
                    }
                    r.authored = StatusField(motiondriver::Status(), " reloadAuthored=") == "1";
                    (void)motiondriver::SetWeaponAction(0);
                    return r;
                };
                const auto desc = [](const Run& r) {
                    return "eligible " + std::to_string(r.eligible) + " max " + std::to_string(r.maxWeight) + " end " +
                           std::to_string(r.endWeight) + (r.authored ? " authored" : " NOT-authored") +
                           (r.applied ? "" : " NOT-applied");
                };
                motiondriver::SetRecoilWeapon(1204, true); // supported Nue pistol: SetAuthoredReload is accepted
                motiondriver::SetAuthoredReload(true);
                const Run stand = scenario(0, 0, 0);   // control: standing reload, no stance change
                const Run crouch = scenario(1, 1, 0);  // control: reload in the crouch
                const Run toStand3 = scenario(1, 2, 3); // crouch -> CrouchSprint (stand), R .067 s into the change
                const Run toStand6 = scenario(1, 2, 6); // crouch -> CrouchSprint (stand), R .117 s into the change
                const Run toCrouch3 = scenario(2, 1, 3);
                const Run toCrouch6 = scenario(2, 1, 6);
                const auto admitted = [](const Run& r) { return r.applied && r.authored && r.eligible >= 25 && r.endWeight > 0.9f; };
                const auto refused = [](const Run& r) { return r.applied && r.authored && r.eligible == 0 && r.maxWeight == 0.0f; };
                Check(admitted(stand), "12d4a standing authored reload without a stance change: correction admitted (unchanged)", desc(stand));
                Check(refused(crouch), "12d4a reload in the crouch: correction not admitted (unchanged)", desc(crouch));
                Check(admitted(toStand3) && admitted(toStand6),
                      "12d4b crouch -> stand, R inside the 0.15 s stance filter: correction admitted for the whole reload",
                      "3 frames: " + desc(toStand3) + " | 6 frames: " + desc(toStand6));
                Check(refused(toCrouch3) && refused(toCrouch6),
                      "12d4c stand -> crouch, R inside the 0.15 s stance filter: correction never admitted (no pulse)",
                      "3 frames: " + desc(toCrouch3) + " | 6 frames: " + desc(toCrouch6));
                mm::integ::PoseSnapshot sSettle;
                for (int i = 0; i < 60; ++i)
                    (void)feedStep(0, sSettle); // stand again, the reload tail is over
                motiondriver::SetAuthoredReload(false);
                motiondriver::SetRecoilWeapon(0, false); // back to the default (no recoil weapon) for the blocks below
            }
            // 12d7) rifle reloads (overlay Rifle, no authored clip) get the native contact correction in every ground stance and gait
            //       (reload_contacts_runtime::ReloadCorrectionPose): standing, walking, crouched and crouch-walking reloads are admitted for
            //       the whole reload, a stance / gait change inside the reload never lowers the weight, a reload in the air is never
            //       admitted; walking / crouched PISTOL reloads stay at 0 (the stationary rule, unchanged). Mutations (tests/reloadfix.cmake):
            //       UTPP_RELOAD_RIFLE_MUTATION=1 (rifle path off) MUST fail 12d7a, =3 (the stationary-only rule of 88910e9) MUST fail 12d7b/c,
            //       =4 (rifle slide frames excluded, e00bb09) MUST fail 12d7f. Rifle reloads on slide frames (PSM Slide, no slide bundle here)
            //       are admitted, pistol slide reloads are not (12d7g).
            {
                struct Run
                {
                    int eligible = 0;
                    float end = 0.0f, maxDrop = 0.0f;
                    bool applied = true;
                };
                // aMode: 0 stand still, 1 stand walking, 2 crouch still, 3 crouch walking, 4 in the air (PSM Fall),
                //       5 sliding (PSM Slide on the ground: the slide frame of ReloadCorrectionPose without the slide bundle).
                const auto step = [&](int aMode, mm::integ::PoseSnapshot& aS) {
                    motionhost::FeedInput f = WalkFeed();
                    if (aMode == 0 || aMode == 2)
                        f.velWorld[1] = f.move[1] = 0.0f;
                    f.locoState = (aMode == 2 || aMode == 3 || aMode == 5) ? mm::integ::kLocoCrouch : mm::integ::kLocoStand;
                    f.psmBlock = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
                    if (aMode == 4)
                        f.psmBlock |= mm::integ::kPsmDetailed | (uint32_t(14 + 1) << mm::integ::kPsmDetailShift); // Fall: MovementMode InAir
                    if (aMode == 5)
                        f.psmBlock |= mm::integ::kPsmDetailed | (uint32_t(5 + 1) << mm::integ::kPsmDetailShift); // Slide
                    (void)motiondriver::Step(f, 0, f.dt);
                    aS = MarkedSnapshot(kRigPtr);
                    return motiondriver::ApplyToSnapshot(aS);
                };
                // 60 frames of the first mode, the reload edge, 40 reload frames with aModeAt(frame), the reload end and a standing settle.
                const auto reload = [&](auto aModeAt) {
                    Run r;
                    mm::integ::PoseSnapshot s;
                    (void)motiondriver::SetWeaponAction(0);
                    for (int i = 0; i < 60; ++i)
                        r.applied = step(aModeAt(0), s) && r.applied;
                    (void)motiondriver::SetWeaponAction(2);
                    float last = 0.0f;
                    for (int i = 0; i < 40; ++i)
                    {
                        r.applied = step(aModeAt(i), s) && r.applied;
                        r.eligible += s.reloadContactsEligible && !s.reloadAuthoredClip ? 1 : 0;
                        r.maxDrop = std::fmax(r.maxDrop, last - s.reloadContactsWeight);
                        last = r.end = s.reloadContactsWeight;
                    }
                    (void)motiondriver::SetWeaponAction(0);
                    for (int i = 0; i < 60; ++i)
                        r.applied = step(0, s) && r.applied; // the reload tail is over
                    return r;
                };
                const auto desc = [](const Run& r) {
                    return "eligible " + std::to_string(r.eligible) + " end " + std::to_string(r.end) + " max drop " +
                           std::to_string(r.maxDrop) + (r.applied ? "" : " NOT-applied");
                };
                const auto admitted = [](const Run& r) { return r.applied && r.eligible >= 25 && r.end > 0.9f && r.maxDrop <= 1.0e-6f; };
                const auto refused = [](const Run& r) { return r.applied && r.eligible == 0 && r.end == 0.0f; };
                Check(motiondriver::SetOverlayInputs(1, 1, false, false), "12d7 Rifle Ready for the reload");
                const Run stand = reload([](int) { return 0; });
                const Run walk = reload([](int) { return 1; });
                const Run crouch = reload([](int) { return 2; });
                const Run crouchWalk = reload([](int) { return 3; });
                const Run changes = reload([](int k) { return k < 12 ? 0 : (k < 22 ? 1 : (k < 32 ? 3 : 2)); }); // stand -> walk -> crouch walk -> crouch
                const Run air = reload([](int) { return 4; });
                const Run slide = reload([](int) { return 5; });
                const Run slideMid = reload([](int k) { return k < 12 ? 1 : (k < 28 ? 5 : 3); }); // walk -> slide -> crouch walk
                Check(admitted(stand), "12d7a standing rifle reload: contact correction admitted (no authored clip)", desc(stand));
                Check(admitted(walk) && admitted(crouch) && admitted(crouchWalk),
                      "12d7b walking / crouched / crouch-walking rifle reload: contact correction admitted for the whole reload",
                      "walk " + desc(walk) + " | crouch " + desc(crouch) + " | crouch walk " + desc(crouchWalk));
                Check(admitted(changes), "12d7c rifle reload with stand -> walk -> crouch walk -> crouch inside it: the weight never drops",
                      desc(changes));
                Check(refused(air), "12d7d rifle reload in the air: correction never admitted", desc(air));
                Check(admitted(slide) && admitted(slideMid),
                      "12d7f rifle reload on slide frames (PSM Slide): correction admitted for the whole reload, a slide inside it never lowers the weight",
                      "slide " + desc(slide) + " | walk -> slide -> crouch walk " + desc(slideMid));
                Check(motiondriver::SetOverlayInputs(2, 1, false, false), "12d7 back to Pistol1H Ready");
                const Run pistolWalk = reload([](int) { return 1; });
                const Run pistolCrouch = reload([](int) { return 2; });
                const Run pistolSlide = reload([](int) { return 5; });
                Check(refused(pistolWalk) && refused(pistolCrouch), "12d7e walking / crouched pistol reload: correction never admitted (unchanged)",
                      "walk " + desc(pistolWalk) + " | crouch " + desc(pistolCrouch));
                Check(refused(pistolSlide), "12d7g pistol reload on slide frames: correction never admitted (unchanged)", desc(pistolSlide));
                mm::integ::PoseSnapshot sSettle;
                (void)stepIdle(2, sSettle);
            }
            Check(motiondriver::SetWeaponAction(8), "12d Shoot action accepted");
            mm::integ::PoseSnapshot sShoot;
            const bool bShoot = stepIdle(2, sShoot);
            Check(bShoot && sShoot.upperScale == 1.0f && StatusField(motiondriver::Status(), " action=") == "8",
                  "12d Shoot is diagnosed without full upper-body handback");
            motiondriver::SetWeaponAction(0);
            motiondriver::SetWeaponUpper(true);
        }
        // First hipfire with no admitted transfer must yield a REAL snapshot action mask,
        // including when v1/native weapon-upper is OFF and the incoming scale is exactly one.
        // Revolver fix (recording 2026-10-25): only a weapon with an authored recoil profile (the route) may hand the arms to the native
        // shot while it waits for the transfer; a Pistol2H without a profile (Overture) keeps the authored arms. pass 0 = no profile,
        // pass 1 = the Nue profile (the original native fallback behaviour).
        for (int hipPass = 0; hipPass < 2; ++hipPass)
        {
            motiondriver::SetWeaponUpper(false);
            motiondriver::SetAimGameUpper(false);
            motiondriver::SetShootTransferEnabled(true);
            motiondriver::SetRecoilProfile(hipPass == 1 ? 1206 : 0, hipPass == 1 ? 1 : 0);
            motiondriver::SetOverlayInputs(2, 0, false, false, 3);
            motiondriver::SetWeaponAction(0);
            auto hipStep = [&](int count, mm::integ::PoseSnapshot& snapshot) {
                bool applied = false;
                for (int i=0;i<count;++i)
                {
                    motionhost::FeedInput feed = WalkFeed();
                    feed.velWorld[1] = feed.move[1] = 0.0f;
                    feed.psmBlock = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
                    motiondriver::Step(feed, 0, feed.dt);
                    snapshot = MarkedSnapshot(kRigPtr);
                    snapshot.upperScale = 1.0f;
                    applied = motiondriver::ApplyToSnapshot(snapshot);
                }
                return applied;
            };
            mm::integ::PoseSnapshot before, first, post, optOut;
            hipStep(3, before);
            motiondriver::SetWeaponAction(8);
            const bool firstOk = hipStep(1, first);
            const auto bit = [](const mm::integ::PoseSnapshot& snap, int bone) {
                return bone >= 0 && ((snap.upperBits[bone >> 5] >> (bone & 31)) & 1u) != 0;
            };
            if (hipPass == 0)
                Check(firstOk && first.upperScale == 1.0f && first.traceAiming == 0 &&
                      StatusField(motiondriver::Status(), " hipFirePose=") == "1" && StatusField(motiondriver::Status(), " shootRoute=") == "0",
                      "12e hipfire first burst of a Pistol2H without a recoil profile (revolver): no native hand-over, authored arms");
            else
                Check(firstOk && first.upperScale == 0.0f && bit(first, wr) && !bit(first, sp3) && !bit(first, hips) &&
                      first.traceAiming == 0 && StatusField(motiondriver::Status(), " hipFirePose=") == "1",
                      "12e hipfire first burst: actual native action mask despite weaponUpper OFF; no native ADS");
            motiondriver::SetWeaponAction(0);
            Check(hipStep(1, post) && post.upperScale == 1.0f,
                  "12e hipfire postshot hold returns authored upper, no permanent native fallback");
            motiondriver::SetShootTransferEnabled(false);
            motiondriver::SetWeaponAction(8);
            Check(hipStep(1, optOut) && optOut.upperScale == 1.0f,
                  "12e hipfire both native options OFF respected in actual snapshot");
            motiondriver::SetWeaponAction(0);
            motiondriver::SetRecoilProfile(0, 0);
            motiondriver::SetWeaponUpper(true);
        }
        // 12c) measurement on the real dump
        const std::string dumpPath = data + "\\..\\Research\\logs\\game\\v2-test-20261013\\utpp-feed-2026-10-13-22-53-08.txt";
        const std::vector<DumpRow> rows = ReadDump(dumpPath);
        Check(rows.size() > 1000, "12c real dump read", dumpPath + " rows " + std::to_string(rows.size()));
        struct Acc
        {
            int n = 0;
            double sum = 0, max = 0, twistMax = 0;
            std::vector<float> e;
        };
        const char* modeName[4] = {"AO off (pre-fix)", "AO on pitch 0", "AO on pitch +20", "AO on pitch -20"};
        const float modePitch[4] = {0.0f, 0.0f, 20.0f, -20.0f};
        Acc acc[4];
        int spans = 0;
        for (int mode = 0; mode < 4 && wr >= 0 && sp3 >= 0 && hips >= 0; ++mode)
        {
            bool inSpan = false;
            float spanStart = 0.0f;
            std::vector<mm::math::Quat> ref;
            int axS = 0, axH = 0; // most horizontal axis of Spine3 / Hips in the span's first measured frame
            for (size_t i = 0; i < rows.size(); ++i)
            {
                const DumpRow& r = rows[i];
                const bool aim = r.upper == 6;
                if (!aim)
                {
                    inSpan = false;
                    continue;
                }
                if (!inSpan)
                {
                    inSpan = true;
                    spanStart = r.t;
                    spans += mode == 0 ? 1 : 0;
                    ref.clear();
                }
                motionhost::FeedInput f;
                std::memcpy(f.velWorld, r.v, sizeof f.velWorld);
                std::memcpy(f.move, r.mv, sizeof f.move);
                f.cameraYawDeg = r.cam;
                f.playerYawDeg = r.yaw;
                f.locoState = r.loco;
                f.tppActive = true;
                f.dt = r.dt > 0.0f && r.dt < 0.1f ? r.dt : 1.0f / 60.0f;
                f.haveTarget = r.haveTgt;
                f.targetYawDeg = r.tgtYaw;
                f.psmBlock = r.psm;
                float tg[3];
                target(wrap(r.cam - r.yaw), modePitch[mode], tg);
                motiondriver::SetAimTarget(mode > 0, true, tg);
                (void)motiondriver::Step(f, 0, f.dt);
                mm::integ::PoseSnapshot s = MarkedSnapshot(kRigPtr);
                if (!motiondriver::ApplyToSnapshot(s) || r.t - spanStart < 0.5f) // Relaxed -> Ready -> Aiming (0.2 + 0.2 s)
                    continue;
                std::vector<mm::math::Quat> q;
                std::vector<mm::math::Vec3> p;
                SnapModel(s, rw, q, p);
                const mm::math::Vec3 barrel = mm::math::QuatRotate(q[wr], mm::math::Vec3{0, 1, 0});
                const mm::math::Vec3 to{tg[0] - p[wr].x, tg[1] - p[wr].y, tg[2] - p[wr].z};
                const float ln = std::sqrt(to.x * to.x + to.y * to.y + to.z * to.z);
                const float c = (barrel.x * to.x + barrel.y * to.y + barrel.z * to.z) / (ln > 0 ? ln : 1.0f);
                const float err = std::acos(std::fmax(-1.0f, std::fmin(1.0f, c))) * 57.29578f;
                if (ref.empty())
                {
                    ref = q12Ref.empty() ? q : q12Ref;
                    float bs = 2.0f, bh = 2.0f;
                    for (int k = 0; k < 3; ++k)
                    {
                        const mm::math::Vec3 a = k == 0 ? mm::math::Vec3{1, 0, 0} : k == 1 ? mm::math::Vec3{0, 1, 0} : mm::math::Vec3{0, 0, 1};
                        const float zs = std::fabs(mm::math::QuatRotate(ref[sp3], a).z), zh = std::fabs(mm::math::QuatRotate(ref[hips], a).z);
                        if (zs < bs)
                        {
                            bs = zs;
                            axS = k;
                        }
                        if (zh < bh)
                        {
                            bh = zh;
                            axH = k;
                        }
                    }
                }
                const float tw = std::fabs(wrap(wrap(heading(q[sp3], axS) - heading(ref[sp3], axS)) - wrap(heading(q[hips], axH) - heading(ref[hips], axH))));
                acc[mode].n++;
                acc[mode].sum += err;
                acc[mode].max = std::fmax(acc[mode].max, err);
                acc[mode].twistMax = std::fmax(acc[mode].twistMax, tw);
                acc[mode].e.push_back(err);
            }
            motiondriver::SetAimTarget(false, false, nullptr);
            std::sort(acc[mode].e.begin(), acc[mode].e.end());
            const auto pct = [&](double k) { return acc[mode].e.empty() ? 0.0f : acc[mode].e[size_t(k * double(acc[mode].e.size() - 1))]; };
            std::printf("INFO 12c %s: %d aim frames (%d spans): crosshair error proxy mean %.2f median %.2f p95 %.2f max %.2f deg; spine twist (Spine3 vs Hips heading, minus 12a idle aim ahead) max %.1f deg\n",
                        modeName[mode], acc[mode].n, spans, acc[mode].n ? acc[mode].sum / acc[mode].n : 0.0, static_cast<double>(pct(0.5)),
                        static_cast<double>(pct(0.95)), acc[mode].max, acc[mode].twistMax);
        }
        Check(wr >= 0 && acc[0].n > 100 && acc[1].n == acc[0].n, "12c measurement ran on the aim spans of the dump",
              std::to_string(acc[0].n) + " / " + std::to_string(acc[1].n) + " frames");
        (void)motiondriver::SetOverlayInputs(0, 0, false, false);
        motiondriver::SetAimGameUpper(false);
    }

    // ---- 10. 6.10.6 / commit A item 3: an exception inside Step fails closed to v1 for the session, cause logged once
    motiondriver::SetEnabled(true);
    n = RunFrames(10, kRigPtr, rw, 0, 0, finite, mapOk);
    Check(n == 10 && finite && mapOk, "fault precondition: re-enabled woman -> motion pose", std::to_string(n));
    const size_t faultLogs0 = LogCount("fallback=fault");
    // trav task A: an accepted traversal (forced montage, version 2 poses) is owned until the fault
    (void)motiondriver::FeedTraversal(ClimbEvent(0.6f, 1.0f));
    (void)TravStep(8);
    const bool faultPre = motiondriver::OwnsTraversal();
    motiondriver::TestThrowInNextStep();
    Check(NoMotionPose(kRigPtr, 0, 0), "fault: the throwing Step returns false, no exception escapes, snapshot untouched (cut)");
    Check(faultPre && !motiondriver::OwnsTraversal(), "10 task A fault: OwnsTraversal true before, false after the fault",
          std::to_string(faultPre ? 1 : 0));
    Check(motiondriver::Reason() == Fallback::Fault && StatusField(motiondriver::Status(), " fault=") == "fault:step-exception",
          "fault: reason fault, first cause fault:step-exception", motiondriver::Status());
    bool stillBase = true;
    for (int i = 0; i < 5; ++i)
        stillBase = NoMotionPose(kRigPtr, 0, 0) && stillBase;
    Check(stillBase, "fault: 5 later steps stay on base (no motion pose, snapshot untouched)");
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    Check(NoMotionPose(kRigPtr, 0, 0) && motiondriver::Reason() == Fallback::Fault,
          "fault: toggle off / on keeps base for the session", mm::adapter::FallbackName(motiondriver::Reason()));
    Check(faultLogs0 == 0 && LogCount("fallback=fault") == 1, "fault logged exactly once over 7 faulted steps",
          std::to_string(faultLogs0) + " -> " + std::to_string(LogCount("fallback=fault")));

    // ---- 11. review minor 4: a process that loaded a bundle and returns from main without Shutdown exits cleanly (child)
    {
        const long code = RunSelfChild("--exit-without-shutdown \"" + data + "\"");
        Check(code == 0, "11 load then exit without Shutdown: clean exit (loader thread detached at static destruction)",
              "child exit code " + std::to_string(code) + " (3 = abort / std::terminate)");
    }

    motiondriver::Shutdown();
    int32_t after = -1;
    Check(!motiondriver::LoadedBody(after), "shutdown frees the bundle");
    std::printf("MotionDriverTest: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

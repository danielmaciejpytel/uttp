// port: MotionDriver integration test through the REAL v1 MMDriver (src/MotionHost.cpp + src/MotionDriver.cpp, no RED4ext / game).
// Usage: utpp_motion_driver_integ_test [--replay-csv <utpp_motion_replay csv, woman db>] <data dir> <v1 database.utppmm> <feed> [feed ...]
//   data dir = <root>\Data (utpp_motion_<body>.utppmm + .chooser.json, overlay\, traversal\; motion-derived, outside git),
//   feeds = Build\test-data\replay\*.feed (tools/replay/extract_feeds.py). A missing file makes the test FAIL.
//   Checks 1, 3 (overlay release, yield mid-feed) and 4 run each scenario in a fresh child process of this exe (--scenario ...):
//   the v1 runtime state survives SetEnabled / LoadDatabase / re-arm, so only fresh processes give comparable v1 runs.
//   --replay-csv: the woman picks must also equal the replay executable's own rows (motionclip, clipT).
// The hooks are the plugin's (MotionDriverNatives.cpp Install: {Step, ApplyToSnapshot, OnArmed, OnDisarmed}) wrapped only to observe
// the arguments / results; the published snapshot is read back through g_mmPose like the FK detour (FkHook.cpp
// ApplyMotionMatching: weight > 0, rig, age <= kDetourStaleSeconds).
// Checks:
//  1 mm.motionDriver off with the hooks installed -> every published v1 snapshot (pose, bone map, weight, upper mask; publishTicks / sequence
//    excluded) bit-identical to the same feed without hooks (precondition: two runs without hooks are identical).
//  2 mm.motionDriver on, woman then man -> a v2 pose published on every frame v2 is allowed to run (v1 g_effGate 0, no PSM release detail),
//    finite, unit quaternions, rig bones of the v2 database; picks equal the utpp_motion_replay adapter path (FeedReplayMotion.cpp
//    RunFeed: MakeMotionFeed -> MapToChooser with the committed visual root yaw (3b845df) -> BuildQueryTrajectory -> ChooserMatcher)
//    for the same frames; feed p95 <= adapter::Budget::kFeedP95Us.
//  3 release frames (overlay release, PSM release detail 10 / 31 in the feed) -> no v2 pose, published snapshot = v1 run.
//  4 missing data / database of the other body -> fallback reason in Status, published snapshots = v1 run.
//  5 toggling mm.motionDriver and switching the body mid-feed -> no crash, a v2 pose is only published on frames where v2 stepped, and the
//    FK consumer never uses a snapshot older than kDetourStaleSeconds.
//  7 review majors 1 / 2 (in process, real MMDriver): the v1 matcher stands still while v2 owns the feeds; the first v1 update
//    after the span (toggle off) searches afresh without a stale inertialization offset and leaves the stale clip; the start bind
//    and the long-way start turn (read from the frozen v1 diagnostics) are released while v2 owns.
//  6 batch 1b feed dump (mm/FeedDump.hpp, on in every child): mm.motionDriver off with hooks -> dump lines identical to no hooks (wall-time
//    columns t / baseMicros masked); v2 owning the feed (tail) -> one line per feed, raw input columns = the v1 run, and the v1 columns
//    are '-' exactly on the v2-owned feeds (status tail= count; the v1 matcher did not run there). Format unchanged (31 columns).
//  8 trav (in process): synthetic climb / vault starts (FeedTraversal + PSM detail 8 / 9 feeds) -> montage pose, hands at
//    the ledge (plan), root path within 0.30 m, continuous handback with JustTraversed; rejected plan -> v2 yields (see the block).
//    Traversal data = Data\traversal\utpp_traversal_<body>_motion.utpptrav (FORMAT_TRAVERSAL version 2: + poses). 8j task A:
//    motiondriver::OwnsTraversal across accept / reject / stale event / episode end. 8i: every one of the 34 montages, woman and man,
//    forced through the real path (TestForceTraversal): accepted, played (db clip or version 2 poses), owned, pose = montage,
//    published FK hand at the FrontLedge end frame on the game's hand point, handback, ownership ends with the episode.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/MotionDriver.hpp"
#include "../src/State.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/MotionAdapter.hpp"
#include "../src/mm/GamePoseCapture.hpp"
#include "../src/mm/MotionTraversal.hpp" // trav check 8
#include "MotionFixRealFrames.hpp" // motionfix 9a
#include "TestFailFast.hpp" // mutation early exit (UTPP_TEST_STOP_ON), called from Check()

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

using namespace utpp;
using namespace utpp::mm;
using namespace utpp::mm::motion;
using adapter::Fallback;

namespace
{
int g_pass = 0, g_fail = 0;
void AbortChildPool(); // kills the runner's children and removes their files; defined with the child pool below
// The decision of UtppFailFastNote (TestFailFast.hpp), mirrored: same variable, same space-joined FAIL lines, same regex. Needed to clean up the
// children of a runner BEFORE the immediate exit of a mutation test (std::_Exit runs no destructor).
bool FailFastWouldStop(const std::string& aFailLine)
{
    static const std::string s_expr = [] {
        char buf[4096];
        const DWORD n = GetEnvironmentVariableA("UTPP_TEST_STOP_ON", buf, sizeof(buf));
        return n > 0 && n < sizeof(buf) ? std::string(buf, n) : std::string();
    }();
    if (s_expr.empty())
        return false;
    static std::string s_seen;
    static bool s_bad = false;
    static std::regex s_rx = [] {
        try { return std::regex(s_expr); } catch (...) { s_bad = true; return std::regex(); }
    }();
    if (s_bad)
        return false;
    if (!s_seen.empty())
        s_seen += ' ';
    s_seen += aFailLine;
    return std::regex_search(s_seen, s_rx);
}
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk) // a mutation test (UTPP_TEST_STOP_ON set by ctest) stops here once the FAIL lines seen so far satisfy its expectation
    {
        const std::string line = "FAIL " + aWhat + (aDetail.empty() ? "" : " | ") + aDetail;
        if (FailFastWouldStop(line))
            AbortChildPool();
        UtppFailFastNote(line);
    }
}

// ------------------------------------------------------------------ feed file (same columns as FeedReplayMotion.cpp LoadFeed)
struct Frame
{
    float t = 0, dt = 0, vx = 0, vy = 0, vz = 0, mvx = 0, mvy = 0, yawDeg = 0, camDeg = 0;
    int loco = 0, trans = 0;
    uint32_t psm = 0; // raw psmBlock; unsigned: the dual-wield flag (bit 31) exceeds INT_MAX
    // Optional extra feed fields (the 31-column game feed dump: Locomotion world target, capsule, walk toggle, air feed). Unset (false) for the
    // 17-column .feed files: ToInput then leaves the FeedInput defaults, so those runs are unchanged.
    bool haveTarget = false, haveExtra = false, haveAir = false, walk = false, onGround = true;
    float targetYawDeg = 0, cap[3] = {0, 0, 0}, distGround = -2.0f;
    int psmFall = 0;
};
struct FeedFile
{
    std::string name;
    std::vector<Frame> fr;
};
bool LoadFeed(const std::string& aPath, FeedFile& f)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    f.name = aPath.substr(aPath.find_last_of("/\\") + 1);
    std::string line, baseClip, step;
    int baseFlags = 0;
    float baseCost = 0, baseMicros = 0;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream s(line);
        Frame r;
        s >> r.t >> r.dt >> r.vx >> r.vy >> r.vz >> r.mvx >> r.mvy >> r.yawDeg >> r.camDeg >> r.loco >> r.psm >> r.trans >> baseClip >> baseFlags >>
            baseCost >> baseMicros >> step;
        if (!s)
            return false;
        f.fr.push_back(r);
    }
    return !f.fr.empty();
}
motionhost::FeedInput ToInput(const Frame& r)
{
    motionhost::FeedInput in;
    in.velWorld[0] = r.vx;
    in.velWorld[1] = r.vy;
    in.velWorld[2] = r.vz;
    in.move[0] = r.mvx;
    in.move[1] = r.mvy;
    in.cameraYawDeg = r.camDeg;
    in.playerYawDeg = r.yawDeg;
    in.locoState = r.loco;
    in.tppActive = true;
    in.dt = r.dt;
    in.psmBlock = static_cast<uint32_t>(r.psm);
    in.haveTarget = r.haveTarget;
    in.targetYawDeg = r.targetYawDeg;
    in.haveExtra = r.haveExtra;
    in.walkToggled = r.walk;
    std::memcpy(in.capsulePos, r.cap, sizeof(in.capsulePos));
    in.haveAir = r.haveAir;
    in.onGround = r.onGround;
    in.distToGround = r.distGround;
    in.psmFall = r.psmFall;
    return in;
}

// ------------------------------------------------------------------ rig = database bones (live player rig must match them)
struct Rig
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    uint32_t bones = 0;
};
bool RigFromDb(const std::string& aPath, Rig& aRig, std::unique_ptr<mm::motion::MotionDatabase>* aKeep = nullptr)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 32)
        return false;
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
    auto db = std::make_unique<mm::motion::MotionDatabase>();
    std::string err;
    if (!db->LoadFromMemory(bytes, rig, err))
        return false;
    aRig.bones = db->NumBones();
    for (uint32_t i = 0; i < db->NumBones(); ++i)
    {
        aRig.hashes.push_back(mm::integ::CNameHash(db->BoneName(i)));
        aRig.parents.push_back(static_cast<int16_t>(db->BoneParent(i)));
    }
    if (aKeep)
        *aKeep = std::move(db);
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
    p.traversal = aData + "\\traversal\\utpp_traversal_" + b + "_motion.utpptrav"; // trav: format 2 (+ poses)
    p.traversalTable = aData + "\\traversal\\traversal_table.json"; // UTPP-MOTION SEAM trav
    return p;
}
bool Exists(const std::string& aPath) { return std::ifstream(aPath).good(); }

// ------------------------------------------------------------------ observed hooks (same functions as Install's kHooks)
struct HookLog
{
    bool stepCalled = false, stepResult = false, applyResult = false;
    uint32_t effGate = 0;
    // The committed visual root of this feed (world yaw, deg), read from the frame the driver hands to the leg tail; see WFrame.
    float actorYawDeg = 0, visualYawDeg = 0;
    bool haveVisual = false;
    // The foot contacts the driver hands to the leg tail with this feed (MotionSetFrame::contact: the live blend of the displayed pose,
    // what FootLocker plants the feet with); all 0 without a frame (raw hooks, v2 not stepped).
    float tailContact[2] = {0, 0};
    bool haveContact = false;
};
HookLog g_log;
std::vector<mm::integ::PoseSnapshot>* g_capture = nullptr; // diagnostics: every published snapshot of a run
bool WStep(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    g_log.stepCalled = true;
    g_log.effGate = aEffGate;
    g_log.actorYawDeg = aRaw.playerYawDeg;
    g_log.stepResult = motiondriver::Step(aRaw, aEffGate, aDt);
    return g_log.stepResult;
}
// Frame hook = the real motiondriver::Frame, observed only. Since 3b845df ("Fix locomotion root ownership", 2026-10-17) the runtime feeds the
// chooser mapping with the COMMITTED visual root (MotionDriver.cpp: ctx.rootYawDeg = b.root.PreviousVisualWorldYaw() / kDegToRad, the
// value rootframe::Processor::Commit stored on the previous v2 step) instead of an integral of the clip ROOT dyaw. The frame handed to
// the leg tail carries exactly that state: rootOffsetYaw = the native offset root yaw of the step just committed, so the visual world
// yaw after this step = Wrap(Wrap(actor yaw) + rootOffsetYaw) (Processor::Prepare / Commit, same float / double steps; the orientation
// warp, which would add to rootOffsetYaw, is off in these runs). The reference picks use it for the NEXT step, like the runtime.
bool WFrame(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    const bool ok = motiondriver::Frame(aBaseGameIndex, aBaseBones, aOut);
    if (ok && aOut.providedRoot)
    {
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        const float actor = static_cast<float>(std::remainder(static_cast<double>(g_log.actorYawDeg * mm::integ::kDegToRad), kTwoPi));
        const float visual = static_cast<float>(std::remainder(static_cast<double>(actor) + static_cast<double>(aOut.rootOffsetYaw), kTwoPi));
        g_log.visualYawDeg = visual / mm::integ::kDegToRad;
        g_log.haveVisual = true;
    }
    if (ok)
    {
        g_log.tailContact[0] = aOut.contact[0];
        g_log.tailContact[1] = aOut.contact[1];
        g_log.haveContact = true;
    }
    return ok;
}
mm::integ::PoseSnapshot g_pre; // commit A metrics: the snapshot as PublishLocked built it, before the v2 apply hook (source pose)
bool g_havePre = false;
bool WApply(mm::integ::PoseSnapshot& aSnap)
{
    g_pre = aSnap;
    g_havePre = true;
    const bool r = motiondriver::ApplyToSnapshot(aSnap);
    g_log.applyResult = g_log.applyResult || r;
    return r;
}
// ownsTraversal = the real motiondriver::OwnsTraversal, as Install's kHooks: the v1 climb release gate (mm.climbRelease) asks v2 (8k);
// travActive = the real motiondriver::TravActive (TRAV-1, vehicle batch: v2 traversal active releases what v2 does not own even with
// mm.climbRelease off, 8k / 8l setting-off cases). ownsSlide stays null here (the slide tests cover it).
const motionhost::MotionHooks kHooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrame, &motiondriver::OwnsTraversal, nullptr,
                            &motiondriver::TravActive};
// pre-commit-A path for the foot-slide comparison: no frame hook -> v1 matcher runs, the raw v2 pose is published
const motionhost::MotionHooks kHooksRaw{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, nullptr};

// ------------------------------------------------------------------ published snapshot (FK detour view)
mm::integ::PoseSnapshot g_read; // large: off the stack
bool ReadPublished() { return g_mmPose.Read(g_read); }
uint64_t RigPtr();
uint64_t SnapHash(const mm::integ::PoseSnapshot& aS)
{
    mm::integ::PoseSnapshot s = aS;
    s.publishTicks = 0; // wall clock
    s.sequence = 0;     // global publish counter
    s.rig = s.rig == RigPtr() ? 1u : 2u; // RigPtr() = address of this exe's g_rigDummy: differs between two builds (v1 identity vs a reference exe)
    // Pose identity fields, not pose data: host poses carry the reported body and every pose its source. They are hashed as the layout
    // without them (host body -1, source 0), so pinned digests (kBarrelLoopOffDigest) keep covering the pose bytes unchanged.
    if (s.source != mm::integ::kPoseSourceDriver)
        s.bodyType = -1;
    s.source = 0;
    // 2026-11-09 melee full body / aim tilt: lowerScale and meleeAimWeight were appended to PoseSnapshot. At their defaults (1 / 0) the
    // hash covers the layout before them (the bytes up to lowerScale + the former tail padding, zero), so digests pinned before the
    // change (kBarrelLoopOffDigest) stay comparable; other values are mixed in after it.
    constexpr size_t kTail = offsetof(mm::integ::PoseSnapshot, lowerScale);
    constexpr size_t kAlign = alignof(mm::integ::PoseSnapshot);
    constexpr size_t kOldSize = (kTail + kAlign - 1) / kAlign * kAlign;
    const auto* p = reinterpret_cast<const uint8_t*>(&s);
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < kOldSize; ++i)
        h = (h ^ (i < kTail ? p[i] : 0u)) * 1099511628211ull;
    if (!(s.lowerScale == 1.0f && s.meleeAimWeight == 0.0f))
    {
        const auto* q = reinterpret_cast<const uint8_t*>(&s.lowerScale);
        const auto* r = reinterpret_cast<const uint8_t*>(&s.meleeAimWeight);
        for (size_t i = 0; i < sizeof(float); ++i)
            h = (h ^ q[i]) * 1099511628211ull;
        for (size_t i = 0; i < sizeof(float); ++i)
            h = (h ^ r[i]) * 1099511628211ull;
    }
    return h;
}
int64_t QpcNow()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}
double AgeSeconds(int64_t aTicks)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return static_cast<double>(QpcNow() - aTicks) / static_cast<double>(f.QuadPart);
}

// "-pid<N>" in a recoil case name: the profile id of the case (RecoilProfiles.hpp), 0 = not given
// "<translation>/<rotation>" of the Status recoilEff= field (m / rad)
static bool ParseEff(const std::string& aValue, double& aT, double& aR)
{
    char* end = nullptr;
    aT = std::strtod(aValue.c_str(), &end);
    if (end == aValue.c_str() || *end != '/') return false;
    const char* rest = end + 1;
    aR = std::strtod(rest, &end);
    return end != rest;
}
static int32_t PidOf(const std::string& aCase)
{
    const size_t at = aCase.find("-pid");
    return at == std::string::npos ? 0 : std::atoi(aCase.c_str() + at + 4);
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

// ------------------------------------------------------------------ v1 driver setup (identical before every run)
std::wstring g_baseDb;
uint8_t g_rigDummy[16];
const Rig* g_armRig = nullptr;
bool ResetBase(const Rig& aRig)
{
    motionhost::SetEnabled(false);
    motionhost::OnDisarmed();
    if (!motionhost::LoadDatabase(g_baseDb, "default"))
        return false;
    motionhost::SetCommandDtFromFeed(true);
    g_hot.armed.store(true, std::memory_order_release); // as Arming.cpp after a successful arm (gate input "mapped")
    motionhost::OnArmed(g_rigDummy, aRig.hashes, aRig.parents);
    motionhost::SetEnabled(true);
    g_armRig = &aRig;
    return true;
}
uint64_t RigPtr() { return reinterpret_cast<uint64_t>(&g_rigDummy[0]); }

struct FrameOut
{
    uint64_t hash = 0;
    bool published = false, stepCalled = false, stepped = false, applied = false;
    bool motionPoseOk = true; // frames with a v2 pose: the published snapshot passes MotionPoseOk (finite, unit quats, v2 rig bones)
    std::string motionWhy;
    uint32_t effGate = 0;
    int32_t clip = -1, db = -1;
    float clipTime = 0;
    // committed visual root yaw after this feed (world, deg; WFrame) - the root yaw the chooser mapping sees on the next v2 step
    float visualYawDeg = 0;
    bool haveVisual = false;
    // foot slide metric inputs: world position of both toes (FootLocker frame, see RunFeed) and the contact the leg tail was given
    bool toeOk[2] = {false, false}, haveContact = false;
    float toeW[2][2] = {{0, 0}, {0, 0}}, tailContact[2] = {0, 0};
    // commit A metrics (deg / us / m): max per-bone angular step of the published pose and of the source pose (pre-apply),
    // max angle published(anchor frame) -> source, feed cost, toe slide per contact frame (-1 = no contact pair)
    float stepP = 0, stepS = 0, theta = 0, feedUs = 0, slide[2] = {-1, -1};
    // the published pose bytes differ from the source pose (pre-apply): a blend / v2 pose changed the snapshot on this frame
    bool changedBySrc = false;
};
// ------------------------------------------------------------------ commit A metrics
// max over mapped bones (same game bone in both snapshots) of the rotation angle between two snapshots, degrees
float MaxAngleDeg(const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b)
{
    if (a.rig != b.rig)
        return 0.0f;
    const uint32_t n = std::min({a.numBones, b.numBones, mm::integ::kMaxMMBones});
    float m = 0.0f;
    for (uint32_t i = std::max(a.firstBone, b.firstBone); i < n; ++i)
    {
        if (a.gameIndex[i] < 0 || a.gameIndex[i] != b.gameIndex[i])
            continue;
        const float* p = &a.pose[size_t(i) * 7 + 3];
        const float* q = &b.pose[size_t(i) * 7 + 3];
        const float d = std::fabs(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
        m = std::max(m, 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f);
    }
    return m;
}
// ------------------------------------------------------------------ trav reference poses (independent of the driver)
// Montage pose at montage time aT: the database clip when the probe played one, else the .utpptrav version 2 P lines of the
// montage (by name), pose bones mapped to the database bones by name; frame lerp + quaternion nlerp. Database bone order.
bool TravRefPose(const mm::motion::MotionDatabase& aDb, const utpp::mm::traversal::TraversalFile& aFile, int32_t aClip,
                 const char* aMontage, float aT, std::vector<BoneTransform>& aOut)
{
    aOut.assign(aDb.NumBones(), BoneTransform{});
    if (aClip >= 0)
        return mm::adapter::SamplePose(aDb, aClip, aT, aOut.data(), aDb.NumBones());
    const utpp::mm::traversal::TravMontage* m = aFile.Find(aMontage);
    if (!m || !m->HasPose())
        return false;
    const size_t nb = aFile.poseBones.size();
    const double ft = std::clamp(static_cast<double>(aT) * m->fps, 0.0, static_cast<double>(m->frames - 1));
    const size_t f0 = static_cast<size_t>(ft), f1 = std::min(f0 + 1, static_cast<size_t>(m->frames - 1));
    const double u = ft - static_cast<double>(f0);
    for (uint32_t i = 0; i < aDb.NumBones(); ++i)
    {
        size_t k = nb;
        for (size_t j = 0; j < nb; ++j)
            if (aFile.poseBones[j] == aDb.BoneName(i))
                k = j;
        if (k == nb)
            return false;
        const float* a = &m->pose[(f0 * nb + k) * 7];
        const float* b = &m->pose[(f1 * nb + k) * 7];
        double q[4], len = 0, dot = 0;
        for (int c = 0; c < 4; ++c)
            dot += double(a[3 + c]) * b[3 + c];
        for (int c = 0; c < 4; ++c)
        {
            const double bc = dot < 0 ? -b[3 + c] : b[3 + c];
            q[c] = a[3 + c] + (bc - a[3 + c]) * u;
            len += q[c] * q[c];
        }
        len = std::sqrt(len);
        aOut[i].p = {static_cast<float>(a[0] + (b[0] - a[0]) * u), static_cast<float>(a[1] + (b[1] - a[1]) * u),
                     static_cast<float>(a[2] + (b[2] - a[2]) * u)};
        aOut[i].q = {static_cast<float>(q[0] / len), static_cast<float>(q[1] / len), static_cast<float>(q[2] / len),
                     static_cast<float>(q[3] / len)};
    }
    return true;
}
// mean rotation angle (deg) over bones 2.. between a published snapshot and a reference pose (same db bone order)
float MeanAngleToRef(const std::vector<BoneTransform>& aRef, const mm::integ::PoseSnapshot& s)
{
    float m = 0.0f;
    int n = 0;
    for (uint32_t i = 2; i < std::min<uint32_t>(s.numBones, static_cast<uint32_t>(aRef.size())); ++i, ++n)
    {
        const float* q = &s.pose[size_t(i) * 7 + 3];
        const float d = std::fabs(q[0] * aRef[i].q.x + q[1] * aRef[i].q.y + q[2] * aRef[i].q.z + q[3] * aRef[i].q.w);
        m += 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f;
    }
    return n ? m / static_cast<float>(n) : 999.0f;
}
// model-space position of bone aBone of a published snapshot, bones 0 / 1 = the entity (identity), parents of the database
bool PublishedBoneModel(const mm::integ::PoseSnapshot& aS, const mm::motion::MotionDatabase& aDb, int aBone, float aOut[3])
{
    if (aBone < 2 || static_cast<uint32_t>(aBone) >= aS.numBones || aS.numBones > aDb.NumBones())
        return false;
    std::vector<std::array<float, 7>> w(aS.numBones);
    for (uint32_t i = 0; i < aS.numBones; ++i)
    {
        const int par = aDb.BoneParent(i);
        const float* l = &aS.pose[size_t(i) * 7];
        if (i < 2 || par < 2)
        {
            const float id[7] = {0, 0, 0, 0, 0, 0, 1};
            const float* src = i < 2 ? id : l;
            std::copy(src, src + 7, w[i].begin());
            continue;
        }
        const float* pw = w[static_cast<size_t>(par)].data();
        const float* q = pw + 3;
        const float tx = 2.0f * (q[1] * l[2] - q[2] * l[1]), ty = 2.0f * (q[2] * l[0] - q[0] * l[2]),
                    tz = 2.0f * (q[0] * l[1] - q[1] * l[0]);
        w[i][0] = pw[0] + l[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
        w[i][1] = pw[1] + l[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
        w[i][2] = pw[2] + l[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
        const float* b = l + 3;
        w[i][3] = q[3] * b[0] + q[0] * b[3] + q[1] * b[2] - q[2] * b[1];
        w[i][4] = q[3] * b[1] - q[0] * b[2] + q[1] * b[3] + q[2] * b[0];
        w[i][5] = q[3] * b[2] + q[0] * b[1] - q[1] * b[0] + q[2] * b[3];
        w[i][6] = q[3] * b[3] - q[0] * b[0] - q[1] * b[1] - q[2] * b[2];
    }
    for (int k = 0; k < 3; ++k)
        aOut[k] = w[static_cast<size_t>(aBone)][k];
    return true;
}
struct Xf
{
    float p[3] = {0, 0, 0};
    float q[4] = {0, 0, 0, 1};
};
void QRot(const float* q, const float* v, float* o)
{
    // o = q v q*
    const float tx = 2.0f * (q[1] * v[2] - q[2] * v[1]), ty = 2.0f * (q[2] * v[0] - q[0] * v[2]), tz = 2.0f * (q[0] * v[1] - q[1] * v[0]);
    o[0] = v[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
    o[1] = v[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
    o[2] = v[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
}
Xf Mul(const Xf& a, const float* local)
{
    Xf r;
    QRot(a.q, local, r.p);
    for (int k = 0; k < 3; ++k)
        r.p[k] += a.p[k];
    const float* b = local + 3;
    r.q[0] = a.q[3] * b[0] + a.q[0] * b[3] + a.q[1] * b[2] - a.q[2] * b[1];
    r.q[1] = a.q[3] * b[1] - a.q[0] * b[2] + a.q[1] * b[3] + a.q[2] * b[0];
    r.q[2] = a.q[3] * b[2] + a.q[0] * b[1] - a.q[1] * b[0] + a.q[2] * b[3];
    r.q[3] = a.q[3] * b[3] - a.q[0] * b[0] - a.q[1] * b[1] - a.q[2] * b[2];
    return r;
}
// model space (character frame) position of bone aBone of a snapshot (database bone order = rig order here)
bool ModelPos(const mm::integ::PoseSnapshot& s, const Rig& aR, int32_t aBone, float* aOut)
{
    int32_t path[64];
    int n = 0;
    for (int32_t b = aBone; b >= 0 && n < 64; b = aR.parents[size_t(b)])
    {
        if (static_cast<uint32_t>(b) >= s.numBones)
            return false;
        path[n++] = b;
    }
    Xf g;
    for (int i = n - 1; i >= 0; --i)
        g = Mul(g, &s.pose[size_t(path[i]) * 7]);
    std::memcpy(aOut, g.p, sizeof(g.p));
    return true;
}
struct MetricCtx
{
    const Rig* rig = nullptr;
    const mm::motion::MotionDatabase* db = nullptr; // v2 database of the run (CONT of the pick)
    int32_t toe[2] = {-1, -1};
    size_t anchor = ~size_t(0); // frame whose published pose is the held start pose (switch frame - 1)
};
MetricCtx* g_metrics = nullptr;
float ContactAt(const mm::motion::MotionDatabase& db, int32_t aClip, float aClipTime, int aSide)
{
    if (aClip < 0 || static_cast<uint32_t>(aClip) >= db.NumClips())
        return 0.0f;
    const auto& cl = db.Clip(static_cast<uint32_t>(aClip));
    const float fps = db.Header().fps > 0.0f ? db.Header().fps : 30.0f;
    const float local = std::fmax(0.0f, aClipTime * fps);
    const uint32_t f0 = cl.firstFrame + std::min(static_cast<uint32_t>(local), cl.numFrames - 1);
    const uint32_t f1 = (f0 + 1 < cl.firstFrame + cl.numFrames) ? f0 + 1 : f0;
    const float a = local - std::floor(local);
    const float c0 = aSide ? db.Contact(f0).r : db.Contact(f0).l, c1 = aSide ? db.Contact(f1).r : db.Contact(f1).l;
    return c0 + (c1 - c0) * a;
}

// One feed frame through the real Feed; aOnFrame(i) runs before Feed (toggles, body switch, overlay inputs).
bool MotionPoseOk(const mm::integ::PoseSnapshot& s, const Rig& aR, std::string& aWhy);
template<typename F>
std::vector<FrameOut> RunFeed(const FeedFile& aF, F&& aOnFrame)
{
    std::vector<FrameOut> out;
    out.reserve(aF.fr.size());
    static mm::integ::PoseSnapshot prevP, prevS, anchorP;
    bool havePrevP = false, havePrevS = false, haveAnchor = false, prevContact[2] = {false, false};
    float prevToe[2][2] = {{0, 0}, {0, 0}}, charX = 0, charY = 0;
    for (size_t i = 0; i < aF.fr.size(); ++i)
    {
        aOnFrame(i);
        g_log = HookLog{};
        g_havePre = false;
        const int64_t t0 = QpcNow();
        motionhost::Feed(ToInput(aF.fr[i]));
        const int64_t t1 = QpcNow();
        FrameOut o;
        o.published = ReadPublished();
        if (g_metrics)
        {
            LARGE_INTEGER fq;
            QueryPerformanceFrequency(&fq);
            o.feedUs = static_cast<float>(static_cast<double>(t1 - t0) * 1e6 / static_cast<double>(fq.QuadPart));
            if (o.published && havePrevP)
                o.stepP = MaxAngleDeg(g_read, prevP);
            if (g_havePre && havePrevS)
                o.stepS = MaxAngleDeg(g_pre, prevS);
            if (o.published && g_havePre)
                o.changedBySrc = std::memcmp(g_read.pose, g_pre.pose, sizeof(g_read.pose)) != 0 ||
                                 std::memcmp(g_read.gameIndex, g_pre.gameIndex, sizeof(g_read.gameIndex)) != 0;
            if (haveAnchor && g_havePre)
                o.theta = MaxAngleDeg(anchorP, g_pre);
            if (o.published)
            {
                prevP = g_read;
                havePrevP = true;
                if (i == g_metrics->anchor)
                {
                    anchorP = g_read;
                    haveAnchor = true;
                }
            }
            if (g_havePre)
            {
                prevS = g_pre;
                havePrevS = true;
            }
        }
        o.hash = o.published ? SnapHash(g_read) : 0;
        if (g_capture)
            g_capture->push_back(g_read);
        o.stepCalled = g_log.stepCalled;
        o.stepped = g_log.stepResult;
        o.applied = g_log.applyResult;
        o.effGate = g_log.effGate;
        o.haveVisual = g_log.haveVisual && o.stepped;
        o.visualYawDeg = g_log.visualYawDeg;
        o.haveContact = g_log.haveContact && o.stepped;
        o.tailContact[0] = g_log.tailContact[0];
        o.tailContact[1] = g_log.tailContact[1];
        if (o.applied)
            o.motionPoseOk = o.published && g_armRig && MotionPoseOk(g_read, *g_armRig, o.motionWhy);
        const std::string st = motiondriver::Status();
        // "pick=clip:<clip>@<t>/db<db>/cost<c>" (MotionDriver.cpp Status)
        const std::string pk = StatusField(st, "pick=clip:");
        const size_t at = pk.find('@'), sl = pk.find("/db");
        if (at != std::string::npos && sl != std::string::npos && sl > at)
        {
            o.clip = std::atoi(pk.substr(0, at).c_str());
            o.clipTime = static_cast<float>(std::atof(pk.substr(at + 1, sl - at - 1).c_str()));
            o.db = std::atoi(pk.substr(sl + 3).c_str());
        }
        if (g_metrics && g_metrics->db && g_metrics->rig)
        {
            // toe slide in the FootLocker world frame (capsule integrated from the feed velocity, actor yaw) while the CONT of
            // the v2 pick is >= 0.5 [P] on this and the previous frame
            const Frame& fr = aF.fr[i];
            const float dt = (std::isfinite(fr.dt) && fr.dt > 0.0f) ? std::fmin(fr.dt, 0.25f) : 0.0f;
            charX += fr.vx * dt;
            charY += fr.vy * dt;
            const float yaw = fr.yawDeg * 0.0174532925f, c = std::cos(yaw), sn = std::sin(yaw);
            for (int side = 0; side < 2; ++side)
            {
                // toe world position on every published v2 frame (the parent applies the contact of its choice, see check A)
                float mt[3];
                if (o.stepped && o.published && g_metrics->toe[side] >= 0 && ModelPos(g_read, *g_metrics->rig, g_metrics->toe[side], mt))
                {
                    o.toeOk[side] = true;
                    o.toeW[side][0] = charX + c * mt[0] - sn * mt[1];
                    o.toeW[side][1] = charY + sn * mt[0] + c * mt[1];
                }
                const bool contact = o.stepped && o.published && ContactAt(*g_metrics->db, o.clip, o.clipTime, side) >= 0.5f;
                float m[3], w[2] = {0, 0};
                const bool ok = contact && g_metrics->toe[side] >= 0 && ModelPos(g_read, *g_metrics->rig, g_metrics->toe[side], m);
                if (ok)
                {
                    w[0] = charX + c * m[0] - sn * m[1];
                    w[1] = charY + sn * m[0] + c * m[1];
                    if (prevContact[side])
                        o.slide[side] = std::hypot(w[0] - prevToe[side][0], w[1] - prevToe[side][1]);
                    prevToe[side][0] = w[0];
                    prevToe[side][1] = w[1];
                }
                prevContact[side] = ok;
            }
        }
        out.push_back(o);
    }
    return out;
}
auto kNoop = [](size_t) {};

// first differing field of two snapshots (diagnostics for the identity checks)
std::string FirstDiff(const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b)
{
    if (a.weight != b.weight)
        return "weight " + std::to_string(a.weight) + " vs " + std::to_string(b.weight);
    if (a.numBones != b.numBones || a.firstBone != b.firstBone || a.rig != b.rig)
        return "header";
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
        if (a.gameIndex[i] != b.gameIndex[i])
            return "gameIndex " + std::to_string(i);
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones * 7; ++i)
        if (std::memcmp(&a.pose[i], &b.pose[i], 4) != 0)
            return "pose bone " + std::to_string(i / 7) + " comp " + std::to_string(i % 7) + " " + std::to_string(a.pose[i]) + " vs " +
                   std::to_string(b.pose[i]);
    if (a.upperScale != b.upperScale)
        return "upperScale";
    if (a.overlayArms != b.overlayArms)
        return "overlayArms";
    return std::memcmp(a.upperBits, b.upperBits, sizeof(a.upperBits)) != 0 ? "upperBits" : "pad / none";
}

// The v2 pose in the published snapshot: finite, unit quaternions from kFirstBone, bone map from the v2 database rig
bool MotionPoseOk(const mm::integ::PoseSnapshot& s, const Rig& aR, std::string& aWhy)
{
    if (s.numBones != aR.bones || s.firstBone != motiondriver::kFirstBone || s.rig != RigPtr())
    {
        aWhy = "bones " + std::to_string(s.numBones) + " first " + std::to_string(s.firstBone);
        return false;
    }
    for (uint32_t b = 0; b < s.numBones; ++b)
    {
        for (int k = 0; k < 7; ++k)
            if (!std::isfinite(s.pose[size_t(b) * 7 + size_t(k)]))
            {
                aWhy = "non-finite bone " + std::to_string(b);
                return false;
            }
        const float* q = &s.pose[size_t(b) * 7 + 3];
        const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (b >= motiondriver::kFirstBone && std::fabs(len - 1.0f) > 1e-3f) // [P] float quaternion normalisation (as MotionDriverTest)
        {
            aWhy = "quat len bone " + std::to_string(b);
            return false;
        }
    }
    return true;
}

// ------------------------------------------------------------------ reference picks: utpp_motion_replay adapter path
// FeedReplayMotion.cpp RunFeed (l. 349-427): MakeMotionFeed (AirFeed{}) -> MapContext {root yaw; previous valid pick's Pivots tag} ->
// MapToChooser -> BuildQueryTrajectory -> ChooserMatcher. Root yaw = the one the runtime hands the mapping since 3b845df: the visual
// root committed by the previous v2 step (taken from the driver's frame, FrameOut::visualYawDeg), seeded with the actor yaw;
// FeedReplayMotion.cpp still integrates the clip ROOT dyaw (pre-3b845df model), so its csv rows agree only where both are equal.
// Restarted (fresh state, seeded again) at every frame where the driver re-entered after a yield, as MotionDriver Step does.
struct RefPick
{
    int32_t clip = -1, db = -1;
    float clipTime = 0;
};
std::vector<RefPick> ReferencePicks(const mm::motion::MotionDatabase& db, const chooser::Table& table, const FeedFile& aF,
                                    const std::vector<FrameOut>& aRun)
{
    std::vector<RefPick> out(aF.fr.size());
    std::unique_ptr<adapter::MotionAdapter> drv;
    adapter::MapState ms;
    const adapter::QueryTimes qt = adapter::BuildQueryTimes(db);
    std::vector<mm::motion::TrajSample> traj(adapter::kQueryTrajMax);
    float rootYawDeg = 0;
    int32_t prevDb = -1;
    bool running = false;
    const float fps = db.Header().fps > 0.0f ? db.Header().fps : 30.0f;
    for (size_t i = 0; i < aF.fr.size(); ++i)
    {
        if (!aRun[i].stepped)
        {
            running = false;
            continue;
        }
        const Frame& r = aF.fr[i];
        if (!running)
        {
            drv = std::make_unique<adapter::MotionAdapter>();
            std::string err;
            mm::motion::MatcherConfig cfg;
            if (!drv->Init(&db, &table, cfg, motiondriver::kFirstBone, err))
                return {};
            ms = adapter::MapState{};
            rootYawDeg = r.yawDeg;
            prevDb = -1;
            running = true;
        }
        const float vel[3] = {r.vx, r.vy, r.vz};
        const float mv[2] = {r.mvx, r.mvy};
        const float dt = (std::isfinite(r.dt) && r.dt > 0.0f) ? std::fmin(r.dt, 0.25f) : 0.0f;
        const adapter::MotionFeed feed = adapter::MakeMotionFeed(vel, mv, dt, static_cast<uint32_t>(r.psm), r.loco, adapter::AirFeed{}, r.camDeg, r.yawDeg);
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = rootYawDeg;
        ctx.currentDbPivots = prevDb >= 0 && adapter::DatabaseHasPivotsTag(db.DbName(static_cast<uint32_t>(prevDb)));
        adapter::MapConfig mc;
        mc.stanceFilterS = adapter::kStanceFilterSeconds; // MotionDriver Step (fix/slide-exit): the reference mirrors the runtime
        mc.stanceHoldInDodge = true;
        mc.stanceCleanEdge = true; // P1 (fix batch 2): the runtime bypasses the filter on a clean Stand <-> Crouch edge
        mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
        mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
        const mm::motion::ChooserState cs = adapter::MapToChooser(feed, mc, ms, ctx);
        const uint32_t nq = adapter::BuildQueryTrajectory(ms, qt, traj.data(), adapter::kQueryTrajMax);
        if (nq == 0)
            continue;
        const bool ok = drv->Update(dt, traj.data(), nq, cs);
        const mm::motion::MatchResult& m = drv->Last();
        if (!ok || m.clip < 0 || m.database < 0)
            continue;
        prevDb = m.database;
        // Root yaw for the next step, as the runtime (MotionDriver.cpp, 3b845df): the committed visual root of this step (aRun[i].visualYawDeg)
        // when the native root ran; only without it the integral of the playing frame's ROOT dyaw (the fallback of the same assignment).
        if (aRun[i].haveVisual)
            rootYawDeg = aRun[i].visualYawDeg;
        else if (m.frame < db.NumFrames())
            rootYawDeg = motiontraj::NormalizeAxis(rootYawDeg + db.Root(m.frame).dyaw * fps * dt / (3.14159265358979f / 180.0f));
        out[i] = RefPick{m.clip, m.database, m.clipTime};
    }
    return out;
}
// ------------------------------------------------------------------ child process scenarios (fresh v1 state per run)
struct ScenarioOut
{
    std::vector<FrameOut> fr;
    std::string status;
    std::vector<std::string> dump; // check 6: feed dump lines (t / baseMicros masked), header excluded
};
std::wstring Wide(const std::string& s) { return std::wstring(s.begin(), s.end()); }
// A temp file of a runner child: <scenario>_<pid>_<n>.txt, unique per child (the runners start several children at once).
std::string OutPathFor(const char* aScenario)
{
    static unsigned s_serial = 0;
    char exe[MAX_PATH];
    GetModuleFileNameA(nullptr, exe, MAX_PATH);
    std::string dir(exe);
    dir = dir.substr(0, dir.find_last_of("\\/"));
    CreateDirectoryA((dir + "\\motion_adapter_integ_tmp").c_str(), nullptr);
    return dir + "\\motion_adapter_integ_tmp\\" + aScenario + "_" + std::to_string(GetCurrentProcessId()) + "_" + std::to_string(s_serial++) + ".txt";
}
// Deletes a child's files: the output, the arms csv of the weapon-upper children and the feed dump folder of the scenario children.
void RemoveChildFiles(const std::string& aOut)
{
    DeleteFileA(aOut.c_str());
    DeleteFileA((aOut + ".arms.csv").c_str());
    const std::string dump = aOut + ".dump";
    WIN32_FIND_DATAA fd;
    if (HANDLE h = FindFirstFileA((dump + "\\*").c_str(), &fd); h != INVALID_HANDLE_VALUE)
    {
        do
        {
            if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
                DeleteFileA((dump + "\\" + fd.cFileName).c_str());
        } while (FindNextFileA(h, &fd));
        FindClose(h);
        RemoveDirectoryA(dump.c_str());
    }
}
// Leftovers of runners that were stopped early (a mutation test exits at its first expected FAIL while children still run): entries of the
// temp folder not written for an hour. Live children rewrite their files within a few minutes (the child timeout is 5 minutes).
void SweepStaleChildFiles()
{
    char exe[MAX_PATH];
    GetModuleFileNameA(nullptr, exe, MAX_PATH);
    std::string dir(exe);
    dir = dir.substr(0, dir.find_last_of("\\/")) + "\\motion_adapter_integ_tmp";
    FILETIME nowFt;
    GetSystemTimeAsFileTime(&nowFt);
    ULARGE_INTEGER now;
    now.LowPart = nowFt.dwLowDateTime;
    now.HighPart = nowFt.dwHighDateTime;
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA((dir + "\\*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE)
        return;
    do
    {
        const std::string name = fd.cFileName;
        if (name == "." || name == "..")
            continue;
        ULARGE_INTEGER t;
        t.LowPart = fd.ftLastWriteTime.dwLowDateTime;
        t.HighPart = fd.ftLastWriteTime.dwHighDateTime;
        if (now.QuadPart < t.QuadPart || now.QuadPart - t.QuadPart < 36000000000ULL) // 1 h in 100 ns
            continue;
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
        {
            WIN32_FIND_DATAA sub;
            if (HANDLE hs = FindFirstFileA((dir + "\\" + name + "\\*").c_str(), &sub); hs != INVALID_HANDLE_VALUE)
            {
                do
                {
                    if (!(sub.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
                        DeleteFileA((dir + "\\" + name + "\\" + sub.cFileName).c_str());
                } while (FindNextFileA(hs, &sub));
                FindClose(hs);
            }
            RemoveDirectoryA((dir + "\\" + name).c_str());
        }
        else
            DeleteFileA((dir + "\\" + name).c_str());
    } while (FindNextFileA(h, &fd));
    FindClose(h);
}

// The runners' children (one process of this exe each) run through a pool: up to min(8, hardware threads / 2) at a time (UTPP_TEST_CHILD_JOBS
// overrides), started in submission order the moment they are added, collected in ANY order by Wait(id). A runner submits all its children
// first and then evaluates them in its original order, so the printed Check lines are the sequential run's lines. Per child semantics are
// the sequential ones: a 5 minute limit counted from its own start (then it is killed and reports exit code STILL_ACTIVE = 259, as the old
// WaitForSingleObject timeout + GetExitCodeProcess did), exit code 1 when it could not be started. Every child is in one job object that kills
// it when this process ends, also on the immediate exit of a mutation test (UtppFailFastNote), so no child outlives its runner.
// The children run below normal priority: the runner's own thread keeps its wall-clock budget checks (feed p95) when the machine is busy.
class ChildPool
{
public:
    static ChildPool& Get()
    {
        static ChildPool s_pool;
        return s_pool;
    }
    size_t Add(std::wstring aCmd, std::string aOut, bool aLaunch = true, DWORD aTimeoutMs = 300000)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        Job j;
        j.cmd = std::move(aCmd);
        j.out = std::move(aOut);
        j.launch = aLaunch;
        j.timeoutMs = aTimeoutMs;
        m_jobs.push_back(std::move(j));
        Pump();
        return m_jobs.size() - 1;
    }
    // Blocks until child aId finished (or timed out); returns its exit code. The other children keep running meanwhile.
    DWORD Wait(size_t aId)
    {
        for (;;)
        {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                Pump();
                if (m_jobs[aId].done)
                    return m_jobs[aId].code;
            }
            Sleep(10);
        }
    }
    // Kills every running child, starts no more and removes the files of all launched ones (the immediate exit of a mutation test).
    void KillAndClean()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_next = m_jobs.size();
        for (size_t i : m_running)
        {
            TerminateProcess(m_jobs[i].process, 1);
            WaitForSingleObject(m_jobs[i].process, 5000);
            CloseHandle(m_jobs[i].process);
            m_jobs[i].process = nullptr;
            m_jobs[i].done = true;
        }
        m_running.clear();
        for (const Job& j : m_jobs)
            if (j.launch)
                RemoveChildFiles(j.out);
    }
    ~ChildPool()
    {
        m_stop = true;
        if (m_monitor.joinable())
            m_monitor.join();
        KillAndClean();
        if (m_group)
            CloseHandle(m_group);
        g_pool = nullptr;
    }
    static ChildPool* g_pool; // the pool when one exists (a runner without children never creates it)

private:
    struct Job
    {
        std::wstring cmd;
        std::string out;
        DWORD timeoutMs = 300000;
        bool launch = true;
        bool done = false;
        HANDLE process = nullptr;
        ULONGLONG startTick = 0;
        DWORD code = 1;
    };
    ChildPool()
    {
        const unsigned n = std::thread::hardware_concurrency() / 2;
        m_max = std::min(8u, std::max(2u, n));
        char env[16];
        if (const DWORD en = GetEnvironmentVariableA("UTPP_TEST_CHILD_JOBS", env, sizeof(env)); en > 0 && en < sizeof(env))
            if (const int v = std::atoi(env); v >= 1)
                m_max = static_cast<size_t>(std::min(v, 60));
        SweepStaleChildFiles();
        m_group = CreateJobObjectW(nullptr, nullptr);
        if (m_group)
        {
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
            info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            SetInformationJobObject(m_group, JobObjectExtendedLimitInformation, &info, sizeof(info));
        }
        g_pool = this;
        // a monitor thread keeps the slots filled while the runner is busy in-process (Pump only runs inside Add / Wait otherwise)
        m_monitor = std::thread([this] {
            while (!m_stop)
            {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    Pump();
                }
                Sleep(20);
            }
        });
    }
    void Pump()
    {
        for (size_t k = 0; k < m_running.size();)
        {
            Job& j = m_jobs[m_running[k]];
            bool fin = WaitForSingleObject(j.process, 0) == WAIT_OBJECT_0;
            if (fin)
                GetExitCodeProcess(j.process, &j.code);
            else if (GetTickCount64() - j.startTick >= j.timeoutMs)
            {
                TerminateProcess(j.process, 1);
                j.code = STILL_ACTIVE;
                fin = true;
            }
            if (fin)
            {
                CloseHandle(j.process);
                j.process = nullptr;
                j.done = true;
                m_running.erase(m_running.begin() + static_cast<std::ptrdiff_t>(k));
            }
            else
                ++k;
        }
        while (m_next < m_jobs.size() && m_running.size() < m_max)
        {
            const size_t id = m_next++;
            Job& j = m_jobs[id];
            STARTUPINFOW si{};
            si.cb = sizeof(si);
            PROCESS_INFORMATION pi{};
            if (!j.launch ||
                !CreateProcessW(nullptr, j.cmd.data(), nullptr, nullptr, FALSE, CREATE_SUSPENDED | BELOW_NORMAL_PRIORITY_CLASS, nullptr, nullptr, &si, &pi))
            {
                j.done = true; // not started: exit code 1, no output
                j.launch = false;
                continue;
            }
            if (m_group)
                AssignProcessToJobObject(m_group, pi.hProcess);
            ResumeThread(pi.hThread);
            CloseHandle(pi.hThread);
            j.process = pi.hProcess;
            j.startTick = GetTickCount64();
            m_running.push_back(id);
        }
    }
    std::vector<Job> m_jobs;
    std::vector<size_t> m_running;
    size_t m_next = 0;
    size_t m_max = 8;
    HANDLE m_group = nullptr;
    std::mutex m_mutex;
    std::thread m_monitor;
    std::atomic<bool> m_stop{false};
};
ChildPool* ChildPool::g_pool = nullptr;
void AbortChildPool()
{
    if (ChildPool::g_pool)
        ChildPool::g_pool->KillAndClean();
}

struct ChildRef
{
    size_t id = 0;
    std::string out;
};
// "<this exe>" <aMode> "<data>" "<db>" "<out>"<aTail>; the tail starts with a space (e.g. L" 60 v2-hipfire-rifle"). aLaunch false = the child
// is not started (its precondition, e.g. a missing database, failed): exit code 1 and no output, as before.
ChildRef SubmitModeChildAt(const std::string& aOut, const wchar_t* aMode, const std::string& aData, const std::string& aDb,
                           const std::wstring& aTail = L"", bool aLaunch = true, DWORD aTimeoutMs = 300000)
{
    wchar_t exe[MAX_PATH];
    GetModuleFileNameW(nullptr, exe, MAX_PATH);
    DeleteFileA(aOut.c_str());
    const std::wstring cmd = L"\"" + std::wstring(exe) + L"\" " + aMode + L" \"" + Wide(aData) + L"\" \"" + Wide(aDb) + L"\" \"" + Wide(aOut) + L"\"" + aTail;
    return ChildRef{ChildPool::Get().Add(cmd, aOut, aLaunch, aTimeoutMs), aOut};
}
ChildRef SubmitModeChild(const char* aTag, const wchar_t* aMode, const std::string& aData, const std::string& aDb,
                         const std::wstring& aTail = L"", bool aLaunch = true, DWORD aTimeoutMs = 300000)
{
    return SubmitModeChildAt(OutPathFor(aTag), aMode, aData, aDb, aTail, aLaunch, aTimeoutMs);
}
ChildRef SubmitScenario(const char* aScenario, const std::string& aData, const std::string& aBaseStack, const std::string& aFeed)
{
    wchar_t exe[MAX_PATH];
    GetModuleFileNameW(nullptr, exe, MAX_PATH);
    const std::string out = OutPathFor(aScenario);
    DeleteFileA(out.c_str());
    const std::wstring cmd = L"\"" + std::wstring(exe) + L"\" --scenario " + Wide(aScenario) + L" \"" + Wide(aData) + L"\" \"" + Wide(aBaseStack) +
                             L"\" \"" + Wide(aFeed) + L"\" \"" + Wide(out) + L"\"";
    return ChildRef{ChildPool::Get().Add(cmd, out), out};
}
// Waits for a child submitted by SubmitModeChild/At, returns its exit code (the caller reads and removes its files).
DWORD WaitChild(const ChildRef& aRef) { return ChildPool::Get().Wait(aRef.id); }
// The results of a scenario child (the old RunChild): false when the child failed or wrote no frame.
bool CollectScenario(const ChildRef& aRef, ScenarioOut& aOut)
{
    const DWORD code = WaitChild(aRef);
    const std::string& out = aRef.out;
    std::ifstream in(out);
    std::string line;
    aOut = ScenarioOut{};
    while (std::getline(in, line))
    {
        if (line.rfind("STATUS ", 0) == 0)
        {
            aOut.status = line.substr(7);
            continue;
        }
        if (line.rfind("DUMP ", 0) == 0)
        {
            aOut.dump.push_back(line.substr(5));
            continue;
        }
        std::istringstream s(line);
        FrameOut o;
        int p = 0, c = 0, st = 0, ap = 0, ch = 0;
        int tk0 = 0, tk1 = 0, hc = 0;
        s >> o.hash >> p >> c >> st >> ap >> o.stepP >> o.stepS >> o.theta >> o.feedUs >> o.slide[0] >> o.slide[1] >> ch >> tk0 >> tk1 >>
            o.toeW[0][0] >> o.toeW[0][1] >> o.toeW[1][0] >> o.toeW[1][1] >> hc >> o.tailContact[0] >> o.tailContact[1];
        o.toeOk[0] = tk0 != 0;
        o.toeOk[1] = tk1 != 0;
        o.haveContact = hc != 0;
        o.changedBySrc = ch != 0;
        o.published = p != 0;
        o.stepCalled = c != 0;
        o.stepped = st != 0;
        o.applied = ap != 0;
        aOut.fr.push_back(o);
    }
    in.close();
    DeleteFileA(out.c_str());
    return code == 0 && !aOut.fr.empty();
}

// Child: fresh process, the scenario's setup, the whole feed through motionhost::Feed, per-frame results to aOut.
int ChildMain(const std::string& aScenario, const std::string& aData, const std::string& aBaseStack, const std::string& aFeed, const std::string& aOut)
{
    g_baseDb = Wide(aBaseStack);
    FeedFile f;
    Rig rw;
    const motiondriver::Paths pw = DataPaths(aData, "woman"), pm = DataPaths(aData, "man");
    std::unique_ptr<mm::motion::MotionDatabase> wdb;
    if (!LoadFeed(aFeed, f) || !RigFromDb(pw.files.db, rw, &wdb))
        return 3;
    MetricCtx mc;
    mc.rig = &rw;
    mc.db = wdb.get();
    for (uint32_t b = 0; b < wdb->NumBones(); ++b)
    {
        if (wdb->BoneName(b) == "LeftToeBase")
            mc.toe[0] = static_cast<int32_t>(b);
        if (wdb->BoneName(b) == "RightToeBase")
            mc.toe[1] = static_cast<int32_t>(b);
    }
    mc.anchor = f.fr.size() / 2 - 1;
    g_metrics = &mc;
    motiondriver::Paths p0 = pw;
    if (aScenario == "dbmissing")
        p0.files.db = aData + "\\does-not-exist\\utpp_motion_woman.utppmm";
    else if (aScenario == "rigmismatch")
        p0.files.db = pm.files.db;
    motiondriver::SetPathsOverride(0, p0);
    motiondriver::SetPathsOverride(1, pm);
    motiondriver::SetRigBody(0);
    motionhost::SetMotionHooks(aScenario == "nohooks" ? nullptr : (aScenario == "raw" ? &kHooksRaw : &kHooks));
    if (!ResetBase(rw))
        return 4;
    const bool motionOn = aScenario != "nohooks" && aScenario != "off";
    if (motionOn)
    {
        // load the bundle before the run (the first Step starts the worker; motiondriver only, v1 state untouched)
        motiondriver::SetEnabled(true);
        (void)motiondriver::Step(ToInput(f.fr.front()), 1u, f.fr.front().dt);
        motiondriver::WaitLoader();
        motiondriver::SetEnabled(false);
        if (aScenario != "handin") // handin: v1 until the switch frame, then mm.motionDriver on
            motiondriver::SetEnabled(true);
    }
    if (aScenario == "release")
        motiondriver::SetOverlayInputs(0, 0, true, false);
    // check 6: batch 1b feed dump on for the whole feed (folder next to the child's output file)
    const std::string dumpDir = aOut + ".dump\\";
    CreateDirectoryA(dumpDir.c_str(), nullptr);
    motionhost::SetFeedDumpDir(Wide(dumpDir));
    const bool dumpOn = motionhost::SetFeedDump(true);
    const size_t half = f.fr.size() / 2;
    const std::vector<FrameOut> r = RunFeed(f, [&](size_t i) {
        if (aScenario == "yield" && i == half)
            motiondriver::SetOverlayInputs(0, 0, true, false);
        if (aScenario == "handin" && i == half)
            motiondriver::SetEnabled(true);
        if (aScenario == "toggleoff" && i == half)
            motiondriver::SetEnabled(false);
    });
    FILE* o = nullptr;
    if (fopen_s(&o, aOut.c_str(), "w") != 0 || !o)
        return 5;
    for (const FrameOut& x : r)
        std::fprintf(o, "%llu %d %d %d %d %.6f %.6f %.6f %.3f %.7f %.7f %d %d %d %.7f %.7f %.7f %.7f %d %.6f %.6f\n",
                     static_cast<unsigned long long>(x.hash), x.published ? 1 : 0,
                     x.stepCalled ? 1 : 0, x.stepped ? 1 : 0, x.applied ? 1 : 0, x.stepP, x.stepS, x.theta, x.feedUs, x.slide[0], x.slide[1],
                     x.changedBySrc ? 1 : 0, x.toeOk[0] ? 1 : 0, x.toeOk[1] ? 1 : 0, x.toeW[0][0], x.toeW[0][1], x.toeW[1][0], x.toeW[1][1],
                     x.haveContact ? 1 : 0, x.tailContact[0], x.tailContact[1]);
    std::fprintf(o, "STATUS %s\n", motiondriver::Status().c_str());
    motionhost::SetFeedDump(false);
    (void)motionhost::FeedDumpFlushForTest(5000);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown(); // joins the dump writer (file closed)
    if (dumpOn)
    {
        // FeedDumpStatus() = "<feed dump status ... file=<path>> poseTrace on=.. frames=.. file=<path>" (since eb9ada7 the pose-trace dumper
        // status follows). The dump path is what lies between the FIRST "file=" and the " poseTrace " separator; it contains spaces
        // ("Ultimate Third Person Project"), so it cannot be cut at a space. Taking the rest of the string made the file unopenable
        // and every dump empty (0 lines), which let "off hooks == no hooks" pass 0 == 0.
        const std::string st = motionhost::FeedDumpStatus();
        const size_t fp = st.find("file=");
        const size_t pe = fp == std::string::npos ? std::string::npos : st.find(" poseTrace ", fp);
        const std::string path = fp == std::string::npos ? std::string()
                                 : st.substr(fp + 5, pe == std::string::npos ? std::string::npos : pe - (fp + 5));
        std::ifstream din(path);
        std::string dl;
        while (std::getline(din, dl))
        {
            if (dl.empty() || dl[0] == '#')
                continue;
            // mask the wall-time columns: 0 t, 15 baseMicros (FeedDump.hpp kHeader)
            std::istringstream ds(dl);
            std::string c, masked;
            for (int k = 0; ds >> c; ++k)
                masked += (k ? " " : "") + ((k == 0 || k == 15) ? std::string("T") : c);
            std::fprintf(o, "DUMP %s\n", masked.c_str());
        }
        din.close();
        DeleteFileA(path.c_str());
    }
    RemoveDirectoryA(dumpDir.c_str());
    std::fclose(o);
    g_metrics = nullptr;
    return 0;
}
// 7e batch 2 (1cf5307) child: v2 leg context on v2-owned feeds. utpp_a former test database (strafe profiles + chest references),
// upper mask on (states 7), hold pose, strafe on, chest align / knee fix defaults; crouch aim strafe left (locoState 2, PSM
// UpperBody Aim) with mm.motionDriver off until the v1 strafe mode and ChestAlign run, then mm.motionDriver on for 90 more identical feeds.
// Line 1 of aOut: counts. Exit 0 = all checks hold, 3 = precondition (no v1 strafe / chest, or v2 never owned), 4 = a v2 frame
// without the crouch stance or the crouch swivel gate, 5 = ChestAlign on a v2 frame, 6 = strafe mode on after the first v2 frame,
// 7 = after mm.motionDriver off (10 feeds) no handback feed, or strafe not back on by the feed after the handback feed.
int LegCtxChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut)
{
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(aData, "man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    motiondriver::SetEnabled(false);
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) || !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motionhost::SetUpperMask(true);
    motionhost::SetUpperMaskStates(7);
    motionhost::SetHoldPose(true);
    motionhost::SetStrafe(true);
    const float dt = 1.0f / 60.0f;
    const uint32_t kCrouchAim = ((0u + 1u) << 24) | ((6u + 1u) << 13); // as the loco suite's B2 strafe section
    float vx = 0.0f;
    auto make = [&]() {
        motionhost::FeedInput in;
        vx = std::fmax(-1.2f, vx - 6.0f * dt); // strafe left (body forward +y, yaw 0)
        in.velWorld[0] = vx;
        in.move[0] = -1.0f;
        in.locoState = 2;
        in.psmBlock = kCrouchAim;
        in.cameraYawDeg = 0.0f;
        in.playerYawDeg = 0.0f;
        in.tppActive = true;
        in.haveTarget = true;
        in.targetYawDeg = 90.0f;
        in.dt = dt;
        return in;
    };
    int baseStackStrafe = 0, baseChest = 0;
    for (int i = 0; i < 90; ++i)
    {
        motionhost::Feed(make());
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        baseStackStrafe += p.strafeOn ? 1 : 0;
        baseChest += p.chestOut ? 1 : 0;
    }
    const motionhost::MotionSeamProbe pre = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(true);
    motiondriver::WaitLoader();
    int owned = 0, badStance = 0, noSwivel = 0, chest = 0, strafeAfter = 0, firstOwned = -1;
    for (int i = 0; i < 90; ++i)
    {
        motionhost::Feed(make());
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        if (p.legMotionSet)
        {
            ++owned;
            if (firstOwned < 0)
                firstOwned = i;
            badStance += p.legStance != 2 ? 1 : 0;
            noSwivel += p.swivelSuppressed ? 0 : 1;
            chest += p.chestOut ? 1 : 0;
        }
        if (firstOwned >= 0 && i > firstOwned)
            strafeAfter += p.strafeOn ? 1 : 0;
    }
    const motionhost::MotionSeamProbe post = motionhost::ProbeMotionSeam();
    // handback phase: mm.motionDriver off, 10 more identical feeds. The handback feed (first v1-owned, handbacks + 1) still sees the held
    // matcher in UpdateStrafeLocked (one-feed exit lag, documented at the gate); strafe must re-enter by the feed after it.
    motiondriver::SetEnabled(false);
    int handbackFeed = -1, strafeAtHandback = -1, reentry = -1;
    for (int i = 0; i < 10; ++i)
    {
        motionhost::Feed(make());
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        if (handbackFeed < 0 && !p.legMotionSet && p.handbacks > post.handbacks)
        {
            handbackFeed = i;
            strafeAtHandback = p.strafeOn ? 1 : 0;
        }
        if (handbackFeed >= 0 && reentry < 0 && p.strafeOn)
            reentry = i - handbackFeed;
    }
    char d[600];
    std::snprintf(d, sizeof d,
                  "base 90 feeds: strafe on %d chest %d (strafe at switch %d); motion on 90 feeds: owned %d (first %d), stance != crouch %d, "
                  "swivel gate off %d, chest align %d, strafe on after the first owned feed %d, strafe exits %llu -> %llu; motion off: "
                  "handback feed %d, strafe on at it %d, strafe re-entry %d feeds after it",
                  baseStackStrafe, baseChest, pre.strafeOn ? 1 : 0, owned, firstOwned, badStance, noSwivel, chest, strafeAfter,
                  static_cast<unsigned long long>(pre.strafeExits), static_cast<unsigned long long>(post.strafeExits), handbackFeed,
                  strafeAtHandback, reentry);
    std::ofstream(aOut) << d << "\n";
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    if (baseStackStrafe == 0 || baseChest == 0 || !pre.strafeOn || owned < 60)
        return 3;
    if (badStance || noSwivel)
        return 4;
    if (chest)
        return 5;
    if (strafeAfter)
        return 6;
    if (handbackFeed < 0 || reentry < 0 || reentry > 1)
        return 7;
    return 0;
}

// 7f review minor B child: aim jog speed while v2 owns. utpp_a former test database, upper mask on (states 7), hold pose, strafe on,
// jogF 3.3 (CET aimJog default); stand gun aim walk forward (locoState 0, PSM AimWalk + UpperBody Aim) at the jog capsule speed:
// 90 feeds mm.motionDriver off, 90 feeds mm.motionDriver on, 10 feeds mm.motionDriver off (handback). After every feed AimJogTarget(0, 1) (the reds query).
// Line 1 of aOut: counts. Exit 0 = all checks hold, 3 = precondition (v1 never answered the jog, or v2 owned < 60 feeds),
// 8 = a v2-owned feed (after the first) answered other than 3.3 m/s (strafe ForceOff dropped the jog), 9 = a feed of the handback
// phase answered other than 3.3 m/s (flip at the handback).
int AimJogChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut)
{
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(aData, "man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    motiondriver::SetEnabled(false);
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) || !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motionhost::SetUpperMask(true);
    motionhost::SetUpperMaskStates(7);
    motionhost::SetHoldPose(true);
    motionhost::SetStrafe(true);
    const float kJog = 3.3f;
    if (!motionhost::SetStrafeParams(false, 10.0f, 30.0f, 0.15f, kJog))
    {
        std::ofstream(aOut) << "SetStrafeParams jogF 3.3 rejected\n";
        return 3;
    }
    const float dt = 1.0f / 60.0f;
    const uint32_t kGunAimWalk = ((0u + 1u) << 24) | ((6u + 1u) << 13) | 0x40u | ((2u + 1u) << 16); // as the loco suite's S4 jog
    float vy = 0.0f;
    auto make = [&]() {
        motionhost::FeedInput in;
        vy = std::fmin(kJog, vy + 6.0f * dt); // forward (body forward +y, yaw 0) up to the jog speed
        in.velWorld[1] = vy;
        in.move[1] = 1.0f;
        in.locoState = 0; // AimWalk: unsupported locomotion state = standing aim
        in.psmBlock = kGunAimWalk;
        in.cameraYawDeg = 0.0f;
        in.playerYawDeg = 0.0f;
        in.tppActive = true;
        in.haveTarget = true;
        in.targetYawDeg = 0.0f;
        in.dt = dt;
        return in;
    };
    auto jogOk = [&](float t) { return std::fabs(t - kJog) <= 1e-4f; };
    int baseJog = 0;
    for (int i = 0; i < 90; ++i)
    {
        motionhost::Feed(make());
        baseJog += jogOk(motionhost::AimJogTarget(0.0f, 1.0f)) ? 1 : 0;
    }
    const float tPre = motionhost::AimJogTarget(0.0f, 1.0f);
    motiondriver::SetEnabled(true);
    motiondriver::WaitLoader();
    int owned = 0, firstOwned = -1, ownedBad = 0;
    float ownedMin = 1e9f;
    for (int i = 0; i < 90; ++i)
    {
        motionhost::Feed(make());
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        const float t = motionhost::AimJogTarget(0.0f, 1.0f);
        if (p.legMotionSet)
        {
            ++owned;
            if (firstOwned < 0)
                firstOwned = i;
        }
        if (firstOwned >= 0 && i > firstOwned)
        {
            ownedBad += jogOk(t) ? 0 : 1;
            ownedMin = std::fmin(ownedMin, t);
        }
    }
    const motionhost::MotionSeamProbe post = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(false);
    int handbackFeed = -1, hbBad = 0;
    float hbMin = 1e9f;
    for (int i = 0; i < 10; ++i)
    {
        motionhost::Feed(make());
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        const float t = motionhost::AimJogTarget(0.0f, 1.0f);
        if (handbackFeed < 0 && !p.legMotionSet && p.handbacks > post.handbacks)
            handbackFeed = i;
        hbBad += jogOk(t) ? 0 : 1;
        hbMin = std::fmin(hbMin, t);
    }
    char d[400];
    std::snprintf(d, sizeof d,
                  "base 90 feeds: jog 3.3 answers %d (at switch %.2f); motion on 90 feeds: owned %d (first %d), owned feeds after the first "
                  "without 3.3 %d (min %.2f); motion off 10 feeds: handback feed %d, feeds without 3.3 %d (min %.2f)",
                  baseJog, static_cast<double>(tPre), owned, firstOwned, ownedBad, static_cast<double>(ownedMin), handbackFeed, hbBad,
                  static_cast<double>(hbMin));
    std::ofstream(aOut) << d << "\n";
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    if (baseJog == 0 || !jogOk(tPre) || owned < 60 || handbackFeed < 0)
        return 3;
    if (ownedBad)
        return 8;
    if (hbBad)
        return 9;
    return 0;
}

// 7h merge repair (b4 review): air Clips mode (v1 database with its .airz) x v2. Exit 0 ok, 3 precondition (no .airz / Clips not
// active / v2 did not own), 10 v1 air clip forced while v2 owns or on a handback feed, 11 the reset after a v2 handback latched
// AirState's pre-span frame as the query bone frame, 12 handback from a v1 air clip frame (root_z) without the clean query frame,
// 13 v1 air steering armed during v2 ownership / first handback, 14 reset while still airborne permitted new foot contacts.
int AirClipsChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut)
{
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(aData, "man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    motiondriver::SetEnabled(false);
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) || !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motionhost::SetHoldPose(true);
    auto synthAir = [](bool aAir, float aVz, int aDetail) {
        motionhost::FeedInput in;
        in.locoState = aAir ? 0 : mm::integ::kLocoStand;
        in.psmBlock = aDetail >= 0 ? (0x40u | ((uint32_t(aDetail) + 1u) << 16)) : 0u;
        in.velWorld[2] = aAir ? aVz : 0.0f;
        in.tppActive = true;
        in.dt = 1.0f / 60.0f;
        in.haveAir = true;
        in.onGround = !aAir;
        in.distToGround = -2.0f;
        return in;
    };
    auto ground = [&](int n) {
        for (int i = 0; i < n; ++i)
            motionhost::Feed(synthAir(false, 0.0f, -1));
    };
    float vz = 3.0f;
    auto airFeed = [&]() {
        motionhost::Feed(synthAir(true, vz, vz > 0.0f ? 18 : 14));
        vz -= (vz > 0.0f ? 19.2f : 16.0f) / 60.0f;
    };
    // A: v1 alone - a standing jump forces air clips (Clips mode really active)
    ground(60);
    const motionhost::MotionSeamProbe a0 = motionhost::ProbeMotionSeam();
    vz = 3.0f;
    for (int i = 0; i < 24; ++i)
        airFeed();
    const motionhost::MotionSeamProbe a1 = motionhost::ProbeMotionSeam();
    ground(60);
    // B: v2 owns from the ground through the jump; handback mid-air, then the AirState reset feed
    motiondriver::SetEnabled(true);
    motiondriver::WaitLoader();
    ground(20);
    const motionhost::MotionSeamProbe b0 = motionhost::ProbeMotionSeam();
    size_t bOwnedAir = 0, bBadAirSteer = 0;
    vz = 3.0f;
    for (int i = 0; i < 24; ++i)
    {
        airFeed();
        const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
        bOwnedAir += (p.legMotionSet && p.airOwns) ? 1 : 0;
        const std::string status = motionhost::Status();
        bBadAirSteer += p.legMotionSet && StatusField(status, "airSteerFed=") != "0" ? 1 : 0;
    }
    const motionhost::MotionSeamProbe b1 = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(false);
    airFeed(); // handback feed
    const motionhost::MotionSeamProbe bHb = motionhost::ProbeMotionSeam();
    const bool bHbSteerQuiet = StatusField(motionhost::Status(), "airSteerFed=") == "0";
    airFeed(); // AirState Reset feed
    const motionhost::MotionSeamProbe bRs = motionhost::ProbeMotionSeam();
    const bool bRsAirborne = bRs.legAirborne && bRs.legContactScale == 0.0f;
    ground(60);
    // C: v1 jump plays an air clip (frame carries root_z), v2 takes over mid-air, handback mid-air: the fresh search takes its bone
    // features from the last clean ground frame
    vz = 3.0f;
    bool cDirty = false;
    for (int i = 0; i < 24 && !cDirty; ++i)
    {
        airFeed();
        cDirty = motionhost::ProbeMotionSeam().frameRootZ;
    }
    const motionhost::MotionSeamProbe c0 = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(true);
    size_t cOwned = 0;
    for (int i = 0; i < 4; ++i)
    {
        airFeed();
        cOwned += motionhost::ProbeMotionSeam().legMotionSet ? 1 : 0;
    }
    const motionhost::MotionSeamProbe c1 = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(false);
    airFeed(); // handback feed
    const motionhost::MotionSeamProbe cHb = motionhost::ProbeMotionSeam();
    ground(60);
    char d[900];
    std::snprintf(d, sizeof d,
                  "airTable %d | A base jump: air clips %llu -> %llu | B motion owned air feeds %zu, clips %llu -> %llu -> hb %llu -> reset %llu, "
                  "handbacks %llu -> %llu, resets %u -> %u, hb frame %d rootZ %d, query overrides hb %llu -> reset %llu (frame %d, pre-span "
                  "frame %d) | C dirty %d (frame %d), v2 owned %zu/4, clips %llu -> %llu -> hb %llu, handback %llu -> %llu, searched %d, "
                  "query overrides %llu -> %llu (frame %d) | motion bad air steer %zu, hb quiet %d, reset airborne %d scale %.2f",
                  a0.airTableValid ? 1 : 0, static_cast<unsigned long long>(a0.airPlays), static_cast<unsigned long long>(a1.airPlays),
                  bOwnedAir, static_cast<unsigned long long>(b0.airPlays), static_cast<unsigned long long>(b1.airPlays),
                  static_cast<unsigned long long>(bHb.airPlays), static_cast<unsigned long long>(bRs.airPlays),
                  static_cast<unsigned long long>(b1.handbacks), static_cast<unsigned long long>(bHb.handbacks), bHb.airResets, bRs.airResets,
                  bHb.frame, bHb.frameRootZ ? 1 : 0, static_cast<unsigned long long>(bHb.airQueryOverrides),
                  static_cast<unsigned long long>(bRs.airQueryOverrides), bRs.airQueryFrame, b0.frame, cDirty ? 1 : 0, c0.frame, cOwned,
                  static_cast<unsigned long long>(c0.airPlays), static_cast<unsigned long long>(c1.airPlays),
                  static_cast<unsigned long long>(cHb.airPlays), static_cast<unsigned long long>(c1.handbacks),
                  static_cast<unsigned long long>(cHb.handbacks), cHb.searched ? 1 : 0, static_cast<unsigned long long>(c1.airQueryOverrides),
                  static_cast<unsigned long long>(cHb.airQueryOverrides), cHb.airQueryFrame, bBadAirSteer, bHbSteerQuiet ? 1 : 0,
                  bRs.legAirborne ? 1 : 0, static_cast<double>(bRs.legContactScale));
    std::ofstream(aOut) << d << "\n";
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    if (!a0.airTableValid || a1.airPlays <= a0.airPlays || bOwnedAir < 10 || bHb.handbacks != b1.handbacks + 1 ||
        bRs.airResets != bHb.airResets + 1 || bHb.frameRootZ || !cDirty || cOwned < 3 || cHb.handbacks != c1.handbacks + 1)
        return 3;
    if (b1.airPlays != b0.airPlays || bHb.airPlays != b1.airPlays || c1.airPlays != c0.airPlays || cHb.airPlays != c1.airPlays)
        return 10;
    if (bRs.airQueryOverrides != bHb.airQueryOverrides)
        return 11;
    if (!cHb.searched || cHb.airQueryOverrides != c1.airQueryOverrides + 1 || cHb.airQueryFrame < 0)
        return 12;
    if (bBadAirSteer != 0 || !bHbSteerQuiet)
        return 13;
    if (!bRsAirborne)
        return 14;
    return 0;
}

// 7i: real published upperScale across pre-armed aim mask, v2 ownership and v1 handback.
// The aim reason remains active across ownership, then clears on an open ground gate.
// This checks easing direction and final bounds, without assuming a precise floating-point blend curve.
int MaskHandoverChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut,
                          bool aAimExit = false, int aHz = 60, bool aLegacy = false, bool aBaseStack = false,
                          const std::string& aWeaponCase = "", bool aPerturbControl = false)
{
    const bool fixtureAim = aWeaponCase.find("aim-pistol") != std::string::npos;
    motionhost::SetAimUpperFade(!aLegacy);
    motiondriver::SetAimUpperFade(!aLegacy);
    motionhost::SetWeaponUpper(aWeaponCase != "off");
    motiondriver::SetWeaponUpper(aWeaponCase != "off");
    // fix2/hip: v2-nue-recoil-*-slide cases need a bundle that declares the slide section (the plain v2 databases do not)
    const bool slideCase = (aWeaponCase.find("nue-recoil-") == 0 || aWeaponCase.find("hipfire-") == 0) &&
                           aWeaponCase.find("-slide") != std::string::npos;
    const auto bodyPaths = [&](const char* aBody) {
        motiondriver::Paths p = DataPaths(aData, aBody);
        if (slideCase)
        {
            const std::string stem = aData + (std::string(aBody) == "man" ? "\\utpp_motion_man_knee_slide" : "\\utpp_motion_woman_knee_slide");
            p.files.db = stem + ".utppmm";
            p.files.chooser = stem + ".chooser.json";
        }
        return p;
    };
    motiondriver::SetPathsOverride(0, bodyPaths("woman"));
    motiondriver::SetPathsOverride(1, bodyPaths("man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(aAimExit || !aWeaponCase.empty() ? 2 : 0, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    motiondriver::SetEnabled(false);
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) ||
        !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motionhost::SetUpperMask(true);
    motionhost::SetUpperMaskStates(7);
    motionhost::SetHoldPose(true);
    const float dt = 1.0f / static_cast<float>(aHz);
    uint32_t weaponBits = 0, extraPsm = 0;
    bool airborne = false;
    bool recoilTppActive = true;
    // fix2/hip slide feed, as MotionDriverSlideHostTest: PSM detail 5 on the ground, 10.8 m/s decaying 2.75 m/s^2 to 2.73 m/s
    bool sliding = false, forceCrouch = false;
    float slideSpeed = 10.8f;
    int moveOverride = -1; // hipfire-route-* cases: 0 = no motion, 1 = moving, -1 = by the case name
    // fix2/hip: the nue-recoil cases run on a simulated recoil clock (one frame = dt), not on QPC + Sleep: the 0.1 s shot pulse, the
    // 0.8 s recoil clip and the admission ramp are wall-time windows, so a loaded machine (Sleep(4) taking 180 ms) used to lose shots
    bool simClock = false;
    auto feed = [&](bool aim) {
        aim = aim || fixtureAim;
        if (simClock) motiondriver::TestAdvanceRecoilClock(dt);
        motionhost::FeedInput in;
        in.locoState = airborne ? 0 : ((forceCrouch || aWeaponCase.find("-crouch") != std::string::npos) ? mm::integ::kLocoCrouch : mm::integ::kLocoStand);
        in.tppActive = recoilTppActive;
        in.dt = dt;
        in.haveAir = true;
        in.onGround = !airborne;
        in.velWorld[2] = airborne ? -4.0f : 0.0f;
        if (aWeaponCase.find("nue-recoil-") == 0 || aWeaponCase.find("hipfire-") == 0)
        {
            const bool stillNow = moveOverride >= 0 ? moveOverride == 0 : aWeaponCase.find("-still") != std::string::npos;
            if(!stillNow) { in.velWorld[1] = 2.5f; in.move[1] = 1.0f; }
        }
        in.distToGround = airborne ? 2.0f : -2.0f;
        in.psmBlock = (aim ? ((6u + 1u) << 13) : (aWeaponCase.empty() ? 0u : ((aWeaponCase == "reload" ? 4u : 1u) << 13))) |
                      weaponBits | extraPsm;
        if (airborne) in.psmBlock |= mm::integ::kPsmDetailed | ((14u + 1u) << 16);
        if (sliding && !airborne)
        {
            in.locoState = 0;
            in.velWorld[1] = slideSpeed;
            slideSpeed = std::max(2.73f, slideSpeed - 2.75f * dt);
            in.psmBlock |= mm::integ::kPsmDetailed | ((5u + 1u) << mm::integ::kPsmDetailShift);
        }
        if (!aWeaponCase.empty()) motiondriver::FeedAirFromInput(in); // same explicit physical feed as the native host bridge
        motionhost::Feed(in);
        return ReadPublished() ? g_read.upperScale : -1.0f;
    };
    auto valid = [](float v) { return std::isfinite(v) && v >= 0.0f && v <= 1.0f; };
    for (int i = 0; i < 45; ++i)
        feed(false);
    // Focused child-only authored-recoil integration. Uses the actual driver
    // shot API and QPC; Sleep advances real sampler time independently of feed dt.
    // Run v2-nue-recoil-{ads,hipfire}-{1h,2h} (optional -man / -air / -slide), not the full suite.
    if (aWeaponCase.find("nue-recoil-") == 0)
    {
        // nue-recoil-<ads|hipfire>-<1h|2h>[-air|-slide][-man]: the Nue pistol (profile 1). The extra tokens "-liberty" (Pistol2H /
        // Liberty profile 2) and "-rifle" (Rifle overlay / Ajax profile 12) run the same block with another profile: the recoil is
        // owned by that profile's curves and the authored reload stays Nue-only.
        const bool rifleOverlay = aWeaponCase.find("-rifle") != std::string::npos;
        const bool libertyProfile = aWeaponCase.find("-liberty") != std::string::npos;
        const int32_t profileId = PidOf(aWeaponCase) ? PidOf(aWeaponCase) : (rifleOverlay ? 12 : (libertyProfile ? 2 : 1));
        const bool nueProfile = profileId == 1;
        const bool oneHand = aWeaponCase.find("1h") != std::string::npos;
        const int poseId = rifleOverlay ? 1 : (oneHand ? 2 : 3);
        const bool aim = aWeaponCase.find("ads") != std::string::npos;
        const bool man = aWeaponCase.find("-man") != std::string::npos;
        const bool airCase = aWeaponCase.find("-air") != std::string::npos;
        Rig sourceRig; std::unique_ptr<mm::motion::MotionDatabase> sourceDb;
        if (!RigFromDb(bodyPaths(man?"man":"woman").files.db,sourceRig,&sourceDb)) return 34;
        motiondriver::SetRigBody(man?1:0);
        motiondriver::SetOverlayInputs(poseId,aim?2:1,false,false);
        motiondriver::SetAimGameUpper(false);
        motionhost::SetWeaponUpper(false); motiondriver::SetWeaponUpper(false);
        motiondriver::SetGripEnabled(true); motiondriver::SetShootTransferEnabled(true);
        motiondriver::SetSlide(slideCase);
        motiondriver::SetEnabled(true);
        motiondriver::Step(motionhost::FeedInput{},0,dt); motiondriver::WaitLoader();
        // A slide case on a bundle without the slide section would pass vacuously on the ground feed: fail instead.
        if (slideCase && !motiondriver::SlideBundleDeclared()) return 36;
        motionhost::OnArmed(&rig,sourceRig.hashes,sourceRig.parents);
        weaponBits=mm::integ::kPsmWeaponKnown|mm::integ::kPsmWeaponDrawn;
        motiondriver::SetRecoilProfile(700,profileId); // = SetRecoilWeapon(700,true) for the Nue
        simClock=true;motiondriver::TestSetRecoilClock(1000.0);
        for(int i=0;i<2*aHz;++i)feed(aim);
        const auto evaluations=[]() {
            const auto value=StatusField(motiondriver::Status(),"shootEvaluations=");
            return value.empty()?uint64_t(0):static_cast<uint64_t>(std::stoull(value));
        };
        const auto applied=[]() {return StatusField(motiondriver::Status(),"shootApplied=")=="1";};
        const auto recoilCleared=[]() {
            const auto status=motiondriver::Status();
            return StatusField(status,"nueRecoilOwns=")=="0" && StatusField(status,"nueRecoilActive=")=="0" &&
                StatusField(status,"nueRecoilAdmitted=")=="0" && StatusField(status,"nueRecoilPending=")=="0";
        };
        const bool curvesReady=StatusField(motiondriver::Status(),"recoilProfileReady=")=="1" &&
            StatusField(motiondriver::Status(),"recoilProfile=")==std::to_string(profileId);
        // recoilEff= is the translation (m) / rotation (rad) of the recoil the arms were given on the last frame
        const auto effective=[]() {
            const std::string value=StatusField(motiondriver::Status(),"recoilEff=");
            double t=0,r=0;if(!ParseEff(value,t,r))return std::pair<double,double>{-1,-1};
            return std::pair<double,double>{t,r};
        };
        float pollScaleMin=1.0f; // reach: share of the kick the IK could follow on the frames where it was applied
        const auto poll=[&](int frames,bool wantOwned) {
            int count=0;bool owned=true;
            for(int i=0;i<frames;++i){feed(aim);count+=applied()?1:0;
                if(applied())pollScaleMin=std::min(pollScaleMin,std::stof(StatusField(motiondriver::Status(),"shootScale=")));
                owned=owned && motionhost::ProbeMotionSeam().legMotionSet && ReadPublished() && g_read.upperScale>.99f &&
                    StatusField(motiondriver::Status(),"nueRecoilOwns=")=="1";
            }
            return std::pair<int,bool>{count,!wantOwned||owned};
        };
        motiondriver::SetWeaponAction(8);
        const uint64_t before=evaluations();
        motiondriver::FeedRecoilShot(700,aim);
        const auto initial=poll(24,true);
        const bool fired=curvesReady && initial.first>=3 && initial.second && evaluations()>before;
        const bool visualAim=aim ? StatusField(motiondriver::Status(),"hipFirePose=")=="0" :
                                  StatusField(motiondriver::Status(),"hipFirePose=")=="1";
        // fix2/hip: the first Step after a shot can come late (frame hitch, loaded machine): the shot pulse must survive its first two
        // Steps whatever the wall time says, or the shot is left to the native action fallback (FPP arms). Settle with the clip over,
        // fire, let 0.3 s pass before the first Step, and the recoil must still be applied and owned.
        motiondriver::SetWeaponAction(0);
        for(int i=0;i<4;++i)feed(aim);
        motiondriver::TestAdvanceRecoilClock(1.0);
        feed(aim);feed(aim);
        motiondriver::SetWeaponAction(8);
        const uint64_t hitchBefore=evaluations();
        motiondriver::FeedRecoilShot(700,aim);
        motiondriver::TestAdvanceRecoilClock(0.3);
        const auto hitchPoll=poll(12,true);
        const bool hitchOk=hitchPoll.first>=3 && hitchPoll.second && evaluations()>hitchBefore;
        // The upper pose and actual recoil must survive the physical air gate,
        // while the leg context remains airborne (no planted-foot correction).
        // Exercise entry with an existing shot, a fresh shot while airborne,
        // and landing; all existing weapon/reload/TPP safety checks still run below.
        bool airPath=true;
        std::ostringstream airDiagnostics;
        if(airCase)
        {
            const auto phase=[&](const char* name,int frames,bool expectAir) {
                int appliedFrames=0,ownedFrames=0,aimFrames=0,airLegFrames=0;
                float minUpper=1.0f,maxContact=0.0f;
                std::string firstFailure;
                for(int i=0;i<frames;++i)
                {
                    feed(aim);
                    const auto seam=motionhost::ProbeMotionSeam();
                    const auto status=motiondriver::Status();
                    const bool published=ReadPublished();
                    const float upper=published?g_read.upperScale:-1.0f;
                    const bool own=seam.legMotionSet && published && valid(upper) && upper>.99f &&
                        StatusField(status,"nueRecoilOwns=")=="1";
                    const bool visual=StatusField(status,"hipFirePose=")==(aim?"0":"1");
                    // Physics marks the first airborne feed immediately; the legacy
                    // AirState decision can retain Ground for one debounce frame.
                    // Assert the active v2 air mode and actual leg safety instead.
                    const bool airLeg=seam.legMotionSet && motiondriver::AirDiagnostics().movementMode==1 &&
                        seam.legAirborne && seam.legContactScale==0.0f;
                    appliedFrames+=applied()?1:0;
                    ownedFrames+=own?1:0;aimFrames+=visual?1:0;airLegFrames+=airLeg?1:0;
                    minUpper=std::fmin(minUpper,upper);
                    maxContact=std::fmax(maxContact,seam.legContactScale);
                    if(firstFailure.empty() && (!own || !visual || (expectAir && !airLeg)))
                        firstFailure=" frame="+std::to_string(i)+" status="+status;
                }
                const bool ok=appliedFrames>=3 && ownedFrames==frames && aimFrames==frames &&
                    (!expectAir || airLegFrames==frames);
                airDiagnostics<<"phase="<<name<<" frames="<<frames<<" applied="<<appliedFrames
                    <<" owned="<<ownedFrames<<" visualAim="<<aimFrames<<" airLegs="<<airLegFrames
                    <<" minUpper="<<minUpper<<" maxContact="<<maxContact<<" ok="<<ok<<firstFailure<<"\n";
                return ok;
            };
            motiondriver::FeedRecoilShot(700,aim);
            const bool groundShot=poll(1,true).second && applied();
            airborne=true;
            const bool carry=phase("ground-shot-to-air",12,true);
            motiondriver::SetWeaponAction(0);feed(aim);
            motiondriver::SetWeaponAction(8);
            const uint64_t airBefore=evaluations();
            motiondriver::FeedRecoilShot(700,aim);
            const bool newShot=phase("shot-in-air",12,true) && evaluations()>airBefore;
            airborne=false;
            const bool land=phase("air-shot-to-land",12,false);
            airPath=groundShot && carry && newShot && land;
            airDiagnostics<<"groundShot="<<groundShot<<" airPath="<<airPath<<"\n";
        }
        // fix2/hip P8: the v2 slide (chooser MovementMode 2) must keep the authored weapon recoil and the MM arms for
        // every shot. Without it each slide shot resets the recoil and the native action fallback publishes upperScale 0.
        bool slidePath=true;
        std::ostringstream slideDiagnostics;
        if(slideCase)
        {
            const auto slideFrameCount=[]() {
                const auto value=StatusField(motiondriver::Status(),"slideFrames=");
                return value.empty()?uint64_t(0):static_cast<uint64_t>(std::stoull(value));
            };
            // aSlideFrames: leading frames fed as the slide, the rest as crouch (aSlideFrames < aFrames).
            // aShot: recoil must be applied and owned; otherwise only the full MM arm ownership is required.
            const auto phase=[&](const char* name,int aFrames,int aSlideFrames,bool aShot) {
                int appliedFrames=0,ownedFrames=0,upperFrames=0,aimFrames=0,slideModeFrames=0;
                float minUpper=1.0f;
                std::string firstFailure;
                const uint64_t slideBefore=slideFrameCount();
                for(int i=0;i<aFrames;++i)
                {
                    if(i==aSlideFrames){sliding=false;forceCrouch=true;}
                    feed(aim);
                    const auto seam=motionhost::ProbeMotionSeam();
                    const auto status=motiondriver::Status();
                    const bool published=ReadPublished();
                    const float upper=published?g_read.upperScale:-1.0f;
                    const bool upperOk=seam.legMotionSet && published && valid(upper) && upper>.99f;
                    const bool own=upperOk && StatusField(status,"nueRecoilOwns=")=="1";
                    const bool visual=StatusField(status,"hipFirePose=")==(aim?"0":"1");
                    const bool slideMode=motiondriver::AirDiagnostics().movementMode==2;
                    appliedFrames+=applied()?1:0;
                    ownedFrames+=own?1:0;upperFrames+=upperOk?1:0;aimFrames+=visual?1:0;
                    slideModeFrames+=(i<aSlideFrames && slideMode)?1:0;
                    minUpper=std::fmin(minUpper,upper);
                    if(firstFailure.empty() && (!(aShot?own:upperOk) || !visual || (i<aSlideFrames && !slideMode)))
                        firstFailure=" frame="+std::to_string(i)+" status="+status;
                }
                const uint64_t slideDelta=slideFrameCount()-slideBefore;
                const bool ok=(aShot?(appliedFrames>=3 && ownedFrames==aFrames):upperFrames==aFrames) &&
                    aimFrames==aFrames && slideModeFrames==aSlideFrames &&
                    slideDelta>=static_cast<uint64_t>(aSlideFrames);
                slideDiagnostics<<"phase="<<name<<" frames="<<aFrames<<" slideFed="<<aSlideFrames<<" applied="<<appliedFrames
                    <<" owned="<<ownedFrames<<" upperOk="<<upperFrames<<" visualAim="<<aimFrames<<" slideMode="<<slideModeFrames
                    <<" slideFrames+="<<slideDelta<<" minUpper="<<minUpper<<" ok="<<ok<<firstFailure<<"\n";
                return ok;
            };
            motiondriver::FeedRecoilShot(700,aim);
            const bool groundShot=poll(1,true).second && applied();
            sliding=true;slideSpeed=10.8f;
            const bool carry=phase("ground-shot-to-slide",12,12,true);
            motiondriver::SetWeaponAction(0);feed(aim);
            motiondriver::SetWeaponAction(8);
            const uint64_t slideShotBefore=evaluations();
            motiondriver::FeedRecoilShot(700,aim);
            const bool newShot=phase("shot-in-slide",12,12,true) && evaluations()>slideShotBefore;
            motiondriver::SetWeaponAction(0);
            const bool between=phase("between-shots-in-slide",10,10,false);
            motiondriver::SetWeaponAction(8);
            motiondriver::FeedRecoilShot(700,aim);
            const bool toCrouch=phase("slide-shot-to-crouch",12,4,true);
            forceCrouch=false;
            slidePath=groundShot && carry && newShot && between && toCrouch;
            slideDiagnostics<<"groundShot="<<groundShot<<" slidePath="<<slidePath<<"\n";
        }
        // Same-weapon rapid retrigger is a real event, not a fabricated FK sample.
        const uint64_t retriggerBefore=evaluations();
        motiondriver::FeedRecoilShot(700,aim);
        const auto retrigger=poll(12,true);
        const bool repeated=retrigger.first>=3 && retrigger.second && evaluations()>retriggerBefore;
        // Review B: a hip <-> ADS switch inside a burst keeps the running kick. The next round selects the other clip: its curve starts
        // from the offset the running one has reached, instead of the offset being deleted in one frame (pre-fix: 13-25 cm with the
        // larger profiles, 5 cm with the Nue).
        // rise(): feed frames until the kick the arms were given is at least aMin (the small profiles need up to 0.4 s)
        const auto rise=[&](double aMin) {
            double now=effective().first;
            for(int i=0;i<aHz*2/5 && now<aMin;++i){feed(aim);now=effective().first;}
            return now;
        };
        motiondriver::FeedRecoilShot(700,aim);feed(aim);
        const double switchBefore=rise(.0008);
        motiondriver::FeedRecoilShot(700,!aim);
        feed(aim);
        const double switchAfter=effective().first;
        // the frame of the switch carries the offset of the running clip (a clip can swing 2x between two frames; a deleted offset is ~0)
        const bool switchKeeps=switchBefore>.0008 && switchAfter>.15*switchBefore && switchAfter>.0003;
        for(int i=0;i<aHz/3;++i)feed(aim);
        motiondriver::SetRecoilProfile(701,profileId);
        const uint64_t staleBefore=evaluations();
        motiondriver::FeedRecoilShot(700,aim);
        const auto stale=poll(4,false);
        const bool staleCleared=stale.first==0 && evaluations()==staleBefore && recoilCleared();
        motiondriver::FeedRecoilShot(701,aim);
        const bool recovered=poll(12,true).first>=3;
        motiondriver::SetShootTransferEnabled(false);
        const uint64_t offBefore=evaluations();
        motiondriver::FeedRecoilShot(701,aim);
        const bool off=poll(4,false).first==0 && evaluations()==offBefore && recoilCleared();
        motiondriver::SetShootTransferEnabled(true);
        const bool noReplay=poll(4,false).first==0 && recoilCleared();
        motiondriver::FeedRecoilShot(701,aim);
        const bool resumed=poll(12,true).first>=3;
        // Publishing a fresh rig mapping must not replay the old shot/carry.
        motionhost::OnArmed(&rig,sourceRig.hashes,sourceRig.parents);
        const uint64_t remapBefore=evaluations();
        const bool remapCleared=poll(4,false).first==0 && evaluations()==remapBefore && recoilCleared();
        motiondriver::SetRecoilWeapon(702,false);
        motiondriver::FeedRecoilShot(702,aim);
        const bool unsupported=poll(4,false).first==0 && recoilCleared();
        // Every guard is exercised AFTER a newly admitted real shot, so a
        // disabled/missing curve cannot make these clear assertions vacuous.
        uint32_t guardEpoch=710;
        const auto armGuard=[&]() {
            recoilTppActive=true;
            motiondriver::SetOverlayInputs(poseId,aim?2:1,false,false);
            motiondriver::SetWeaponAction(0);
            for(int i=0;i<4;++i)feed(aim); // restore any preceding TPP/release gate
            motiondriver::SetRecoilProfile(++guardEpoch,profileId);
            motiondriver::SetWeaponAction(8);
            motiondriver::FeedRecoilShot(guardEpoch,aim);
            const auto active=poll(12,true);
            return active.first>=3 && active.second;
        };
        bool guardsArmed=armGuard();
        const bool rejectedResolved=!motiondriver::SetOverlayInputs(poseId,aim?2:1,false,false,999);
        const bool invalidResolved=rejectedResolved && recoilCleared() && poll(4,false).first==0;
        guardsArmed=armGuard() && guardsArmed;
        const bool rejectedAim=!motiondriver::SetOverlayInputs(poseId,999,false,false);
        const bool invalidAim=rejectedAim && recoilCleared() && poll(4,false).first==0;
        guardsArmed=armGuard() && guardsArmed;
        recoilTppActive=false;
        const bool tppClear=poll(4,false).first==0 && recoilCleared();
        guardsArmed=armGuard() && guardsArmed;
        motiondriver::SetOverlayInputs(poseId,aim?2:1,true,false);
        const bool releaseClear=recoilCleared() && poll(4,false).first==0;
        guardsArmed=armGuard() && guardsArmed;
        // Review B: a reload that starts while the kick is running fades it over the 0.12 s upper-body entry instead of deleting it in
        // the first reload frame. One frame later the kick is still owned and its exit weight has only started to fall; 0.2 s later it is gone and every owner flag is clear.
        motiondriver::SetWeaponAction(0);
        for(int i=0;i<4;++i)feed(aim);
        motiondriver::SetRecoilProfile(++guardEpoch,profileId);
        motiondriver::SetWeaponAction(8);
        motiondriver::FeedRecoilShot(guardEpoch,aim);feed(aim);
        const double reloadBefore=rise(.0008);
        motiondriver::SetWeaponAction(2);
        feed(aim);
        const double reloadAfter=effective().first;
        // the first reload frame: the recoil is still owned and its exit weight has only started to fall (a deleted kick: not owned)
        const double reloadExitW=std::atof(StatusField(motiondriver::Status(),"recoilExitW=").c_str());
        const bool reloadFades=reloadBefore>.0008 && StatusField(motiondriver::Status(),"nueRecoilOwns=")=="1" && reloadExitW<1.0 && reloadExitW>.5;
        for(int i=0;i<aHz/5+2;++i)feed(aim);
        const bool reloadClear=reloadFades && recoilCleared() && poll(4,false).first==0;
        motiondriver::SetAuthoredReload(true);feed(aim);
        const bool stationaryReloadCase=!man && aWeaponCase.find("-still")!=std::string::npos &&
            aWeaponCase.find("-crouch")==std::string::npos;
        // The authored reload exists for the Nue only: every other profile must leave SetAuthoredReload(true) refused.
        const bool authoredOn=StatusField(motiondriver::Status(),"reloadAuthored=")==((man||!nueProfile)?"0":"1") &&
            ReadPublished() && g_read.reloadAuthoredClip==(nueProfile && stationaryReloadCase);
        motiondriver::SetAuthoredReload(false);feed(aim);
        const bool authoredOff=StatusField(motiondriver::Status(),"reloadAuthored=")=="0";
        motiondriver::SetRecoilWeapon(999,false);motiondriver::SetAuthoredReload(true);feed(aim);
        const bool unsupportedAuthored=StatusField(motiondriver::Status(),"reloadAuthored=")=="0";
        // the IK follows at least 70% of the kick on every applied frame (the Nue is the reference and is not gated here)
        const bool reachOk=nueProfile || pollScaleMin>=.7f;
        const std::string finalStatus=motiondriver::Status();
        std::ofstream(aOut)<<"case="<<aWeaponCase<<" hz="<<aHz<<" moving="<<(aWeaponCase.find("-still")==std::string::npos)<<" curvesReady="<<curvesReady<<" fired="<<fired
            <<" visualAim="<<visualAim<<" hitch="<<hitchOk<<" airCase="<<airCase<<" airPath="<<airPath<<" slideCase="<<slideCase<<" slidePath="<<slidePath<<" repeated="<<repeated<<" stale="<<staleCleared
            <<" recovered="<<recovered<<" off="<<off<<" noReplay="<<noReplay<<" resumed="<<resumed
            <<" remap="<<remapCleared<<" unsupported="<<unsupported<<" guardsArmed="<<guardsArmed
            <<" invalidResolved="<<invalidResolved<<" invalidAim="<<invalidAim<<" tppClear="<<tppClear
            <<" pollScaleMin="<<pollScaleMin<<" releaseClear="<<releaseClear<<" reloadClear="<<reloadClear<<" reloadFades="<<reloadFades<<" reloadEff="<<reloadBefore<<"/"<<reloadAfter<<" switchKeeps="<<switchKeeps<<" switchEff="<<switchBefore<<"/"<<switchAfter<<" initialApplied="<<initial.first
            <<" authoredOn="<<authoredOn<<" authoredOff="<<authoredOff<<" unsupportedAuthored="<<unsupportedAuthored
            <<" status="<<finalStatus<<"\n"<<airDiagnostics.str()<<slideDiagnostics.str();
        motiondriver::TestSetRecoilClock(-1.0);
        motiondriver::SetEnabled(false);motionhost::SetMotionHooks(nullptr);motiondriver::Shutdown();motionhost::Shutdown();
        return fired&&visualAim&&hitchOk&&reachOk&&airPath&&slidePath&&repeated&&switchKeeps&&staleCleared&&recovered&&off&&noReplay&&resumed&&remapCleared&&unsupported&&
            guardsArmed&&invalidResolved&&invalidAim&&tppClear&&releaseClear&&reloadClear&&
            authoredOn&&authoredOff&&unsupportedAuthored?0:35;
    }
    // P9 (recording 2026-10-21 12:15): hipfire with a weapon that has NO recoil route (rifle, SMG, shotgun, any pistol but the Nue
    // preset outside the native FK stance) keeps the authored Aim arms (upperScale 1) through the shot and the 0.75 s hold. Before the
    // fix the native first-burst fallback published upperScale 0 (FPP arms, ~100 degrees off the MM shoulders) for every act=8 frame
    // and snapped back in one frame (log: 41/41 rifle episodes, |d upper| = 1.00 at the start and the end of every shot).
    // hipfire-<rifle|pistol2h-other|pistol1h-other|nue2h>[-still|-crouch|-air|-slide][-man]. There is no FeedRecoilShot for the
    // "other" weapons (the reds binding reports the Nue preset only). Controls: hipfire-nue2h (the authored route owns the arms) and
    // hipfire-pistol2h-other-still (Pistol2H standing still keeps its native FK route, so the fallback may still engage there).
    if (aWeaponCase.find("hipfire-") == 0)
    {
        const bool rifle = aWeaponCase.find("rifle") != std::string::npos;
        const bool oneHand = aWeaponCase.find("pistol1h") != std::string::npos;
        const bool nueControl = aWeaponCase.find("nue2h") != std::string::npos;
        // hipfire-<rifle|pistol2h|pistol1h>-profile: the weapon HAS an authored recoil profile (rifle: Ajax 12, pistols: Liberty 2),
        // so, like the Nue, its recoil episode owns the arms: upperScale 1, recoil owner, applied transfers growing.
        const bool profileCase = aWeaponCase.find("-profile") != std::string::npos;
        // profile of the case: Ajax for a rifle, Liberty for a pistol; "-masamune" (rifle, 8 cm after the budget, burst rifle, the review's
        // worst case) and "-quasar" (revolver); "-pid<N>" selects any profile id
        const int32_t profileId = PidOf(aWeaponCase) ? PidOf(aWeaponCase) :
                                  aWeaponCase.find("masamune") != std::string::npos ? 15 : aWeaponCase.find("quasar") != std::string::npos ? 11 :
                                  (rifle ? 12 : 2);
        // hipfire-<rifle|pistol2h|pistol1h>-badprofile: an id outside the table fails closed = a weapon without a profile (behaviour A)
        const bool badProfile = aWeaponCase.find("-badprofile") != std::string::npos;
        // hipfire-<pistol1h|pistol2h>-retired: the retired id 9 (the Overture) fails closed the same way
        const bool retiredProfile = aWeaponCase.find("-retired") != std::string::npos;
        const bool authoredControl = nueControl || profileCase;
        const bool man = aWeaponCase.find("-man") != std::string::npos;
        const bool airCase = aWeaponCase.find("-air") != std::string::npos;
        // Revolver fix (recording 2026-10-25, Overture): the legacy native FK stance (Pistol2H, stand, no motion) is NOT a route any
        // more. Without an authored recoil profile no weapon waits for a transfer: the support arm stayed on the authored Aim pose, the
        // native shot (upperScale 0, arm 93-142 degrees away in one frame) never takes it. hipfire-pistol2h-other-still is the
        // regression of that revolver case (UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION=3 restores the old route and must fail it).
        const bool nativeFkRoute = false;
        const bool expectRoute = authoredControl || nativeFkRoute;
        const uint32_t shotEpoch = nueControl ? 700 : (profileCase ? 702 : 701);
        Rig sourceRig; std::unique_ptr<mm::motion::MotionDatabase> sourceDb;
        if (!RigFromDb(bodyPaths(man ? "man" : "woman").files.db, sourceRig, &sourceDb)) return 34;
        motiondriver::SetRigBody(man ? 1 : 0);
        motiondriver::SetOverlayInputs(rifle ? 1 : (oneHand ? 2 : 3), 1, false, false);
        motiondriver::SetAimGameUpper(false);
        motionhost::SetWeaponUpper(false); motiondriver::SetWeaponUpper(false);
        motiondriver::SetGripEnabled(true); motiondriver::SetShootTransferEnabled(true);
        motiondriver::SetSlide(slideCase);
        motiondriver::SetEnabled(true);
        motiondriver::Step(motionhost::FeedInput{}, 0, dt); motiondriver::WaitLoader();
        if (slideCase && !motiondriver::SlideBundleDeclared()) return 36; // a slide case on a bundle without the slide section would be vacuous
        motionhost::OnArmed(&rig, sourceRig.hashes, sourceRig.parents);
        weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
        if (profileCase) motiondriver::SetRecoilProfile(shotEpoch, profileId);
        else if (badProfile) motiondriver::SetRecoilProfile(shotEpoch, 99);
        else if (retiredProfile) motiondriver::SetRecoilProfile(shotEpoch, 9);
        else motiondriver::SetRecoilWeapon(shotEpoch, nueControl);
        // The recoil weapons (Nue control and the profile cases) run on the simulated recoil clock (one feed = dt), like the nue-recoil
        // block: the 0.1 s shot pulse, the clip and the admission ramp are wall-time windows, so a loaded machine lost shots under Sleep(4).
        if (authoredControl) { simClock = true; motiondriver::TestSetRecoilClock(1000.0); }
        for (int i = 0; i < 2 * aHz; ++i) feed(false);
        if (airCase) { airborne = true; for (int i = 0; i < aHz / 2; ++i) feed(false); }
        if (slideCase) { sliding = true; slideSpeed = 10.8f; for (int i = 0; i < aHz / 6; ++i) feed(false); }
        const auto frames = [&](double aSeconds) { return std::max(1, static_cast<int>(std::lround(aSeconds * aHz))); };
        // P9 review C: the route is decided once per shot (kept from the act=8 edge, it can only fall). Pistol2H that is not the Nue preset:
        //   hipfire-route-pulse: stands still on the edge (native FK route), a 0.05 s micro-step in the middle of the shot flips the live
        //     gate 1 -> 0 -> 1: the arms may switch FPP -> authored once, never back (4 switches without the latch);
        //   hipfire-route-stop: moves on the edge (no route), stops in the middle: the live gate rises 0 -> 1, the held route must stay 0 and
        //     the authored arms must not give way to the FPP shot (2 switches without the latch).
        if (aWeaponCase.find("hipfire-route-") == 0)
        {
            const bool pulse = aWeaponCase.find("pulse") != std::string::npos;
            moveOverride = pulse ? 0 : 1;
            for (int i = 0; i < aHz; ++i) feed(false); // the movement state settles on the edge value
            const int pulseAt = frames(0.05), pulseLen = frames(0.05);
            std::ostringstream trace;
            int flips = 0, liveFalls = 0, liveRises = 0, heldRises = 0;
            float prevUpper = -1.0f, minUpper = 1.0f;
            bool prevLive = false, prevHeld = false, havePrev = false;
            const int shotFrames = frames(0.217), holdFrames = frames(0.9);
            for (int i = -frames(0.1); i < shotFrames + holdFrames; ++i)
            {
                if (i == 0) motiondriver::SetWeaponAction(8);
                if (i == shotFrames) motiondriver::SetWeaponAction(0);
                if (pulse && i == pulseAt) moveOverride = 1;
                if (pulse && i == pulseAt + pulseLen) moveOverride = 0;
                if (!pulse && i == pulseAt) moveOverride = 0;
                feed(false);
                const std::string status = motiondriver::Status();
                const float upper = ReadPublished() ? g_read.upperScale : -1.0f;
                const bool live = StatusField(status, "shootRouteLive=") == "1", held = StatusField(status, "shootRoute=") == "1";
                if (prevUpper >= 0.0f && upper >= 0.0f && std::fabs(upper - prevUpper) > 0.5f) ++flips;
                if (i >= 0 && i < shotFrames && havePrev)
                {
                    if (prevLive && !live) ++liveFalls;
                    if (!prevLive && live) ++liveRises;
                    if (i >= 1 && !prevHeld && held) ++heldRises; // (the edge frame sets the route)
                }
                if (i >= 0) minUpper = std::fmin(minUpper, upper);
                trace << (i < 0 ? 'p' : (i < shotFrames ? 'S' : 'h')) << ":" << upper << "/" << (live ? 1 : 0) << (held ? 1 : 0) << " ";
                prevUpper = upper; prevLive = live; prevHeld = held; havePrev = true;
            }
            // Revolver fix: this weapon (Pistol2H, no authored recoil profile) has no route in any movement state, so a micro-step or a stop
            // inside the shot changes nothing: the gate never opens, the arms never switch and stay authored (upperScale 1) for the whole
            // shot and hold. (Before the fix the standing state opened the native FK route and the arms switched to the native shot.)
            const bool pass = liveFalls == 0 && liveRises == 0 && heldRises == 0 && flips == 0 && minUpper > 0.99f;
            std::ofstream(aOut) << "case=" << aWeaponCase << " hz=" << aHz << " pass=" << pass << " flips=" << flips << " liveFalls=" << liveFalls
                << " liveRises=" << liveRises << " heldRises=" << heldRises << " minUpper=" << minUpper << "\ntrace=" << trace.str() << "\n";
            motiondriver::SetEnabled(false); motionhost::SetMotionHooks(nullptr); motiondriver::Shutdown(); motionhost::Shutdown();
            return pass ? 0 : 37;
        }
        // pulseFrames > 0: a shot pulse every that many frames while the action stays 8 (full-auto: PSM.Weapon 8 is held across shots)
        struct Seg { const char* name; int action; int frames; int hipFrames; int pulseFrames = 0; };
        // act=8 frames and the first 0.70 s of the hold must show the hipfire pose; the tail after the hold expiry only needs the arms.
        std::vector<Seg> segs;
        segs.push_back({"ready", 0, frames(0.3), 0});
        segs.push_back({"shot", 8, frames(0.217), frames(0.217)});            // median rifle shot of the recording
        segs.push_back({"hold", 0, frames(0.9), frames(0.7)});                // HipFirePoseHold 0.75 s
        for (int i = 0; i < 4; ++i)
        {
            segs.push_back({"burst-shot", 8, frames(0.1), frames(0.1)});
            segs.push_back({"burst-gap", 0, frames(0.1), frames(0.1)});
        }
        segs.push_back({"tail", 0, frames(1.0), frames(0.6)});
        segs.push_back({"shot2-no-hold", 8, frames(0.217), frames(0.217)});   // first shot after the hold expired (17/41 of the recording)
        segs.push_back({"hold2", 0, frames(0.9), frames(0.7)});
        // Full-auto with a recoil profile: one second of held action 8 with a shot pulse every 83 ms (~12 rounds/s), the clip is
        // retriggered on every round. The arms stay MM-owned and continuous, the kick keeps being applied, then the hold expires.
        if (profileCase)
        {
            segs.push_back({"auto-fire", 8, frames(1.0), frames(1.0), std::max(1, static_cast<int>(std::lround(0.083 * aHz)))});
            segs.push_back({"auto-hold", 0, frames(0.9), frames(0.7)});
        }
        std::ostringstream diag;
        bool allOk = true;
        float prevUpper = -1.0f;
        int raiseFrames = 0;
        int aoRaise = 0;
        float prevAo = 0.0f, aoStep = 0.0f; // the steady Ready before the shot has aoW 0
        int totalHip = 0, totalShotFrames = 0, routeFrames = 0, nativeFrames = 0, appliedFrames = 0, modeFailures = 0, ownsFrames = 0;
        // recoil given to the arms per frame (Status recoilEff= translation m / rotation rad) and the share of it the IK reached
        double prevEffT = -1, prevEffR = -1, effStepT = 0, effStepR = 0, effMaxT = 0, effMaxR = 0;
        float scaleMin = 1.0f;
        float globalMinUpper = 1.0f, globalMaxStep = 0.0f;
        std::string firstFailure;
        const auto evaluationCount = []() {
            const auto value = StatusField(motiondriver::Status(), "shootEvaluations=");
            return value.empty() ? uint64_t(0) : static_cast<uint64_t>(std::stoull(value));
        };
        const uint64_t evaluationsBefore = evaluationCount();
        for (const Seg& seg : segs)
        {
            motiondriver::SetWeaponAction(seg.action);
            if (seg.action == 8 && authoredControl) motiondriver::FeedRecoilShot(shotEpoch, false);
            int ok = 0, hip = 0, route = 0, native = 0, applied = 0, owns = 0;
            float minUpper = 1.0f, maxStep = 0.0f, segScaleMin = 1.0f;
            std::string scaleTrace;
            for (int i = 0; i < seg.frames; ++i)
            {
                if (seg.pulseFrames > 0 && i > 0 && i % seg.pulseFrames == 0) motiondriver::FeedRecoilShot(shotEpoch, false);
                feed(false);
                const auto seam = motionhost::ProbeMotionSeam();
                const std::string status = motiondriver::Status();
                // P9 C: the first shot comes out of a steady Ready; count the frames until the hands pose (aimPoseW) reaches 80%.
                // P9 B: the AO alpha (aoW) takes the raise with the hands (80% within 0.1 s + 1.5 frames, the second shot of a burst has the
                // full sweep); its step per frame stays under dt / 0.1 (60 deg yaw: 600 deg/s < AoJumpDeg(dt))
                if (std::string(seg.name) == "shot")
                {
                    const std::string poseW = StatusField(status, "aimPoseW="), aoW = StatusField(status, "aoW=");
                    if (raiseFrames == 0 && !poseW.empty() && std::stof(poseW) >= 0.8f) raiseFrames = i + 1;
                    if (!aoW.empty())
                    {
                        const float ao = std::stof(aoW); // %.2f in the status: allow its rounding
                        if (aoRaise == 0 && ao >= 0.8f) aoRaise = i + 1;
                        if (prevAo >= 0.0f) aoStep = std::max(aoStep, ao - prevAo);
                        prevAo = ao;
                    }
                }
                const bool published = ReadPublished();
                const float upper = published ? g_read.upperScale : -1.0f;
                const bool owned = seam.legMotionSet && published && valid(upper);
                const bool hipPose = StatusField(status, "hipFirePose=") == "1";
                const bool routeNow = StatusField(status, "shootRoute=") == "1";
                const bool modeOk = !airCase && !slideCase ? true :
                    motiondriver::AirDiagnostics().movementMode == (slideCase ? 2 : 1);
                const float step = prevUpper >= 0.0f && upper >= 0.0f ? std::fabs(upper - prevUpper) : 0.0f;
                prevUpper = upper;
                minUpper = std::fmin(minUpper, upper);
                maxStep = std::fmax(maxStep, step);
                hip += hipPose ? 1 : 0;
                route += routeNow ? 1 : 0;
                native += (owned && upper < 0.5f) ? 1 : 0;
                applied += StatusField(status, "shootApplied=") == "1" ? 1 : 0;
                owns += StatusField(status, "nueRecoilOwns=") == "1" ? 1 : 0;
                {
                    double effT = 0, effR = 0;
                    if (ParseEff(StatusField(status, "recoilEff="), effT, effR))
                    {
                        if (prevEffT >= 0) { effStepT = std::max(effStepT, std::fabs(effT - prevEffT)); effStepR = std::max(effStepR, std::fabs(effR - prevEffR)); }
                        prevEffT = effT; prevEffR = effR; effMaxT = std::max(effMaxT, effT); effMaxR = std::max(effMaxR, effR);
                    }
                    if (StatusField(status, "shootApplied=") == "1")
                    {
                        const float sc = std::stof(StatusField(status, "shootScale="));
                        scaleMin = std::min(scaleMin, sc);
                        segScaleMin = std::min(segScaleMin, sc);
                        if (sc < 0.9f && scaleTrace.size() < 120) scaleTrace += std::to_string(i) + ":" + std::to_string(sc).substr(0, 4) + "/" + StatusField(status, "recoilEff=").substr(0, 14) + " ";
                    }
                }
                modeFailures += modeOk ? 0 : 1;
                // the arms: published, bounded, continuous; for the weapons without a route also fully authored (upperScale 1)
                const bool armsOk = owned && (nativeFkRoute || (step <= 0.011f && upper > 0.99f));
                // R3 (combat probe 2026-10-24, shoot_jump): in the air no hip-fire hold is kept after a Shoot (the jump clip owns the arms);
                // the air cases now require the pose on the act=8 frames and its ABSENCE on every action-0 frame (was: 0.7 s of hold).
                const bool hipOk = (airCase && seg.action == 0) ? !hipPose : (i >= seg.hipFrames || hipPose);
                const bool routeOk = seg.action != 8 || routeNow == expectRoute;
                const bool frameOk = armsOk && hipOk && routeOk && modeOk;
                ok += frameOk ? 1 : 0;
                if (!frameOk && firstFailure.empty())
                    firstFailure = std::string(seg.name) + " frame=" + std::to_string(i) + " upper=" + std::to_string(upper) +
                        " hipPose=" + std::to_string(hipPose) + " route=" + std::to_string(routeNow) + " status=" + status;
            }
            const bool segOk = ok == seg.frames;
            allOk = allOk && segOk;
            globalMinUpper = std::fmin(globalMinUpper, minUpper);
            globalMaxStep = std::fmax(globalMaxStep, maxStep);
            if (seg.action == 8) { totalShotFrames += seg.frames; routeFrames += route; nativeFrames += native; appliedFrames += applied; ownsFrames += owns; }
            totalHip += hip;
            diag << "phase=" << seg.name << " action=" << seg.action << " frames=" << seg.frames << " ok=" << ok << " hipPose=" << hip
                 << " route=" << route << " nativeFrames=" << native << " applied=" << applied << " minUpper=" << minUpper
                 << " maxStep=" << maxStep << " segOk=" << segOk << " scaleMin=" << segScaleMin << (scaleTrace.empty() ? "" : " lowScale[frame:scale/eff]=" + scaleTrace) << "\n";
        }
        // Weapon without a route: no frame of any shot may hand the arms to FPP and no recoil transfer may be reported as applied.
        const bool noNative = expectRoute || (nativeFrames == 0 && appliedFrames == 0 && routeFrames == 0);
        // Control (Pistol2H standing still): the native warm-up fallback is still engaged where a route exists.
        const bool fallbackKept = !nativeFkRoute || nativeFrames > 0;
        // Control (native FK route): the route stays open on every act=8 frame, so the P9 gate cannot over-reach.
        const bool routeOpen = !expectRoute || routeFrames == totalShotFrames;
        const bool supportedFlag = StatusField(motiondriver::Status(), "recoilSupported=") == (authoredControl ? "1" : "0");
        // A weapon with a profile: the profile is the one sent (and its files loaded), the recoil episode owns the arms on every
        // shot frame (upperScale 1 is already required by armsOk) and transfers are applied (the evaluator solved the kick).
        const uint64_t evaluationsGrown = evaluationCount() - evaluationsBefore;
        const bool profileOk = !profileCase ||
            (StatusField(motiondriver::Status(), "recoilProfile=") == std::to_string(profileId) &&
             StatusField(motiondriver::Status(), "recoilProfileReady=") == "1" && ownsFrames == totalShotFrames &&
             appliedFrames >= 3 && evaluationsGrown >= 3 &&
             // the burst never moves the weapon more than 3 cm / 5 degrees between two frames, stays under the series cap and the
             // reach solver follows at least 70% of the kick on every frame (the Nova / Overture / Burya clips, a hammer-cock motion
             // 7-10 cm sideways, got 3-30% and flickered between frames = the bisection of 9 solves per frame the review measured)
             effStepT <= 0.03 && effStepR <= 5.0 * 3.14159265 / 180.0 && effMaxT <= 0.126 && effMaxR <= 0.63 && scaleMin >= 0.7f);
        const std::string finalStatus = motiondriver::Status();
        // The hip shot raises the weapon out of Ready in hipFireReadyToAiming (0.1 s): 80% within 0.1 s + 1.5 frames. The pre-C raise
        // (readyToAiming 0.2 s) needs ~0.16 s + 1 frame, which fails this bound at 30, 60 and 144 Hz.
        const bool raiseOk = raiseFrames > 0 && static_cast<float>(raiseFrames) * dt <= 0.1f + 1.5f * dt;
        // The AO alpha follows the hands: 80% within 0.1 s + 1.5 frames, at most dt / 0.1 per frame (+ 0.011 for the %.2f rounding of the status).
        const bool aoOk = aoRaise > 0 && static_cast<float>(aoRaise) * dt <= 0.1f + 1.5f * dt && aoStep <= dt / 0.1f + 0.011f;
        const bool pass = allOk && noNative && routeOpen && fallbackKept && supportedFlag && modeFailures == 0 && raiseOk && aoOk && profileOk;
        std::ofstream(aOut) << "case=" << aWeaponCase << " hz=" << aHz << " pass=" << pass << " allOk=" << allOk << " noNative=" << noNative
            << " routeOpen=" << routeOpen << " fallbackKept=" << fallbackKept << " raiseOk=" << raiseOk << " raiseFrames=" << raiseFrames << " aoOk=" << aoOk << " aoStep=" << aoStep << " aoRaise=" << aoRaise << " supportedFlag=" << supportedFlag
            << " profileOk=" << profileOk << " effStepCm=" << effStepT * 100 << " effStepDeg=" << effStepR * 57.29578 << " effMaxCm=" << effMaxT * 100
            << " effMaxDeg=" << effMaxR * 57.29578 << " scaleMin=" << scaleMin << " ownsFrames=" << ownsFrames << " evaluationsGrown=" << evaluationsGrown << " minUpper=" << globalMinUpper
            << " maxStep=" << globalMaxStep << " shotFrames=" << totalShotFrames << " routeFrames=" << routeFrames
            << " nativeFrames=" << nativeFrames << " appliedFrames=" << appliedFrames << " modeFailures=" << modeFailures
            << " hipPoseFrames=" << totalHip << " firstFailure=[" << firstFailure << "]\n" << diag.str()
            << "status=" << finalStatus << "\n";
        motiondriver::TestSetRecoilClock(-1.0);
        motiondriver::SetEnabled(false); motionhost::SetMotionHooks(nullptr); motiondriver::Shutdown(); motionhost::Shutdown();
        return pass ? 0 : 37;
    }
    if (aWeaponCase == "reload-mask" || aWeaponCase == "reload-mask-man")
    {
        const bool man = aWeaponCase == "reload-mask-man";
        Rig sourceRig;
        std::unique_ptr<mm::motion::MotionDatabase> sourceDb;
        if (!RigFromDb(DataPaths(aData, man ? "man" : "woman").files.db, sourceRig, &sourceDb)) return 3;
        motiondriver::SetRigBody(man ? 1 : 0);
        motiondriver::SetOverlayInputs(2, 1, false, false);
        motiondriver::SetAimGameUpper(false);
        motionhost::SetWeaponUpper(false); motiondriver::SetWeaponUpper(false);
        motiondriver::SetEnabled(true);
        motiondriver::Step(motionhost::FeedInput{}, 0, dt); motiondriver::WaitLoader();
        motionhost::OnArmed(&rig, sourceRig.hashes, sourceRig.parents);
        weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
        for (int i = 0; i < aHz; ++i) feed(false);
        auto masked = [&](const auto& database, std::string_view name) {
            for (uint32_t i = 0; i < database.NumBones(); ++i)
                if (database.BoneName(i) == name)
                    return mm::integ::SnapshotUpperBone(g_read, i);
            return false;
        };
        motiondriver::SetWeaponAction(2);
        for (int i = 0; i < aHz / 2; ++i) feed(false);
        const bool reload = motionhost::ProbeMotionSeam().legMotionSet && g_read.upperScale < 0.01f &&
            !masked(*sourceDb, "Spine3") && masked(*sourceDb, "RightHand") && masked(*sourceDb, "WeaponRight");
        motiondriver::SetWeaponAction(0);
        motiondriver::SetOverlayInputs(2, 2, false, false);
        // Runtime regression: entering ADS during the reload tail must not let
        // the v1 aim mask drive the v2 action weight back toward native FPP.
        bool reloadAimTail = true;
        float previousTail = g_read.upperScale;
        for (int i = 0; i < aHz / 2; ++i)
        {
            const float current = feed(i >= aHz / 20);
            reloadAimTail = reloadAimTail && valid(current) &&
                current + 0.0001f >= previousTail && current - previousTail <= dt / 0.20f + 0.002f;
            previousTail = current;
        }
        reloadAimTail = reloadAimTail && previousTail > 0.999f;
        motiondriver::SetAimGameUpper(true);
        for (int i = 0; i < aHz / 2; ++i) feed(true);
        const bool aim = motionhost::ProbeMotionSeam().legMotionSet && g_read.upperScale < 0.01f &&
            masked(*sourceDb, "Spine3") && masked(*sourceDb, "RightHand") && !masked(*sourceDb, "Hips");
        motiondriver::SetEnabled(false);
        for (int i = 0; i < aHz / 2; ++i) feed(true);
        const bool v1 = !motionhost::ProbeMotionSeam().legMotionSet && g_read.upperScale < 0.01f &&
            masked(db, "Spine3") && masked(db, "RightHand") && !masked(db, "Hips");
        std::ofstream(aOut) << "case=" << aWeaponCase << " hz=" << aHz << " reload=" << reload
                           << " reloadAimTail=" << reloadAimTail << " aim=" << aim << " base=" << v1 << "\n";
        motionhost::SetMotionHooks(nullptr); motiondriver::Shutdown(); motionhost::Shutdown();
        return !reload ? 30 : !aim ? 31 : !v1 ? 32 : !reloadAimTail ? 33 : 0;
    }
    if (aWeaponCase == "grip-pistol" || aWeaponCase == "grip-rifle" ||
        aWeaponCase == "grip-man-pistol" || aWeaponCase == "grip-man-rifle" ||
        aWeaponCase == "grip-aim-pistol" || aWeaponCase == "grip-man-aim-pistol")
    {
        const bool man = aWeaponCase.find("grip-man-") == 0;
        Rig sourceRig;
        std::unique_ptr<mm::motion::MotionDatabase> sourceDb;
        if (!RigFromDb(DataPaths(aData, man ? "man" : "woman").files.db, sourceRig, &sourceDb)) return 3;
        hashes = sourceRig.hashes; parents = sourceRig.parents;
        motiondriver::SetRigBody(man ? 1 : 0);
        motiondriver::SetOverlayInputs(aWeaponCase.find("rifle") != std::string::npos ? 1 : 3, fixtureAim ? 2 : 1, false, false);
        if (fixtureAim)
        {
            motiondriver::SetAimGameUpper(false);
            const float target[3] = {-5.0f, 8.660254f, 11.4f}; // yaw30, pitch45 relative to the aim origin
            motiondriver::SetAimTarget(true, true, target);
        }
        motiondriver::SetEnabled(true);
        motiondriver::Step(motionhost::FeedInput{}, 0, dt); motiondriver::WaitLoader();
        motionhost::OnArmed(&rig, hashes, parents);
        weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
        motionhost::SetWeaponUpper(false); motiondriver::SetWeaponUpper(false);
        for (int i = 0; i < (fixtureAim ? 2 * aHz : 45); ++i) feed(false); // settle the actual overlay before constructing a reachable reference grip
        if (!ReadPublished() || g_read.numBones != sourceDb->NumBones()) return 24;
        std::vector<uint8_t> records(size_t(sourceDb->NumBones()) * mm::integ::kGameBoneStride, 0);
        for (uint32_t i = 0; i < sourceDb->NumBones(); ++i)
        {
            float* rec = reinterpret_cast<float*>(records.data() + size_t(i) * mm::integ::kGameBoneStride);
            const float* p = g_read.pose + i * 7;
            rec[0] = p[0]; rec[1] = p[1]; rec[2] = p[2];
            rec[4] = p[3]; rec[5] = p[4]; rec[6] = p[5]; rec[7] = p[6];
        }
        auto capture = [&] {
            LARGE_INTEGER now; QueryPerformanceCounter(&now);
            return mm::integ::CaptureGamePoseSample(records.data(), sourceDb->NumBones(), reinterpret_cast<uint64_t>(&rig), now.QuadPart);
        };
        // Authored two-hand grip: Pistol2H / Pistol1H and, since bbcc18a (2026-10-15), the Rifle too (motionpose::MayUseAuthoredTwoHandGrip):
        // gameGripWanted is false for all of them, nobody captures the game pose and the left hand follows the authored grip (source 2).
        // The game-pose route (source 1, captured >= 3, responds to the changed game target) is only reachable on a handback.
        const bool authored = aWeaponCase.find("pistol") != std::string::npos || aWeaponCase.find("rifle") != std::string::npos;
        int captured = 0, supplied = 0, applied = 0, clamped = 0, owned = 0, sourceOk = 0;
        float maxGripError = 0.0f;
        for (int i = 0; i < 45; ++i)
        {
            captured += capture() ? 1 : 0; feed(false);
            const auto p = motiondriver::ProbeGrip();
            owned += motionhost::ProbeMotionSeam().legMotionSet ? 1 : 0;
            sourceOk += p.source == (authored ? 2u : 1u) ? 1 : 0;
            supplied += p.supplied ? 1 : 0; applied += p.applied ? 1 : 0;
            clamped += p.clamped ? 1 : 0;
            if (p.applied) maxGripError = std::max(maxGripError, p.error);
        }
        const std::string aimStatus = motiondriver::Status();
        const bool aimPath = !fixtureAim || (aimStatus.find("aim=tgt11 aiming=1") != std::string::npos &&
                                            aimStatus.find("aoYaw=30.0") != std::string::npos);
        int leftHand = -1;
        for (uint32_t i = 0; i < sourceDb->NumBones(); ++i)
            if (sourceDb->BoneName(i) == "LeftHand") leftHand = static_cast<int>(i);
        if (leftHand < 0) return 25;
        const auto beforePerturb = g_read;
        if (!aPerturbControl)
            reinterpret_cast<float*>(records.data() + size_t(leftHand) * mm::integ::kGameBoneStride)[0] += 0.01f;
        // Export matching frame indices for an independent perturbed/control process comparison.
        // Comparing to the earlier pose also includes normal animation time progression.
        std::ofstream perturbFrames(aOut + ".arms.csv");
        perturbFrames.precision(9);
        for (int i = 0; i < 5; ++i)
        {
            capture(); feed(false);
            const auto p = motiondriver::ProbeGrip();
            if (!p.applied || p.clamped || p.error > 0.001f) return 26;
            perturbFrames << i;
            for (uint32_t b = 0; b < sourceDb->NumBones(); ++b)
                if (sourceDb->BoneName(b) == "LeftArm" || sourceDb->BoneName(b) == "LeftForeArm" || sourceDb->BoneName(b) == "LeftHand")
                    for (int j = 3; j < 7; ++j) perturbFrames << ',' << g_read.pose[b * 7 + j];
            perturbFrames << '\n';
        }
        bool changed = false;
        for (uint32_t i = 0; i < sourceDb->NumBones(); ++i)
            if (sourceDb->BoneName(i) == "LeftArm" || sourceDb->BoneName(i) == "LeftForeArm" || sourceDb->BoneName(i) == "LeftHand")
                for (int j = 3; j < 7; ++j)
                    changed = changed || std::fabs(beforePerturb.pose[i * 7 + j] - g_read.pose[i * 7 + j]) > 1e-4f;
        mm::integ::GamePoseSample old;
        if (!mm::integ::g_gamePoseSamples.Read(old))
        {
            if (!authored) return 27;
            mm::integ::PoseSnapshot map;
            if (!mm::integ::g_gamePoseReadMap.Read(map)) return 27;
            LARGE_INTEGER now; QueryPerformanceCounter(&now);
            old.rig = map.rig; old.generation = map.sequence; old.count = map.numBones; old.ticks = now.QuadPart;
        }
        auto rejectedOrIgnored = [&] {
            const auto p = motiondriver::ProbeGrip();
            return authored ? p.source == 2 && p.supplied && p.applied && p.error <= 0.001f : !p.supplied;
        };
        LARGE_INTEGER frequency; QueryPerformanceFrequency(&frequency);
        auto stale = old; stale.ticks -= frequency.QuadPart;
        mm::integ::g_gamePoseSamples.TryPublish(stale); feed(false);
        const bool staleRejected = rejectedOrIgnored();
        auto otherRig = old; ++otherRig.rig;
        mm::integ::g_gamePoseSamples.TryPublish(otherRig); feed(false);
        const bool rigRejected = rejectedOrIgnored();
        auto future = old; future.ticks += frequency.QuadPart;
        mm::integ::g_gamePoseSamples.TryPublish(future); feed(false);
        const bool futureRejected = rejectedOrIgnored();
        capture(); feed(false);
        motiondriver::OnArmed(reinterpret_cast<uint64_t>(&rig), hashes, parents);
        feed(false);
        const bool rearmRejected = rejectedOrIgnored();
        const bool freshCapture = capture(); feed(false);
        const bool recovered = motiondriver::ProbeGrip().supplied;
        motiondriver::SetGripEnabled(false); feed(false);
        const bool toggleOff = !capture() && !motiondriver::ProbeGrip().supplied && !motiondriver::ProbeGrip().applied;
        motiondriver::SetGripEnabled(true); feed(false);
        const bool toggleCapture = capture(); feed(false);
        const bool toggleOn = (authored ? !toggleCapture : toggleCapture) && motiondriver::ProbeGrip().supplied && motiondriver::ProbeGrip().applied;
        auto captureTimes = [&] {
            std::vector<double> us;
            us.reserve(2000);
            for (int i = 0; i < 2000; ++i)
            {
                const auto begin = std::chrono::steady_clock::now();
                capture();
                us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count());
            }
            std::sort(us.begin(), us.end());
            return std::array<double, 3>{us[1000], us[1900], us[1980]};
        };
        const auto captureOnUs = captureTimes();
        weaponBits = mm::integ::kPsmWeaponKnown; feed(false);
        const bool emptyRejected = !motiondriver::ProbeGrip().supplied && !motiondriver::ProbeGrip().applied;
        const bool emptyCaptureSkipped = !capture();
        weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn; feed(false);
        motiondriver::SetEnabled(false);
        const bool disabledCaptureSkipped = !capture();
        const auto captureOffUs = captureTimes();
        std::ofstream(aOut) << "grip=" << aWeaponCase << " hz=" << aHz << " captured=" << captured << " supplied=" << supplied
                           << " applied=" << applied << " rearmRejected=" << rearmRejected << " fresh=" << freshCapture
                           << " recovered=" << recovered << " emptyRejected=" << emptyRejected
                           << " owned=" << owned << " sourceOk=" << sourceOk << " authored=" << authored << " clamped=" << clamped << " maxGripError=" << maxGripError
                           << " changed=" << changed << " control=" << aPerturbControl << " stale=" << staleRejected << " rig=" << rigRejected
                           << " future=" << futureRejected << " emptySkip=" << emptyCaptureSkipped
                           << " disabledSkip=" << disabledCaptureSkipped << " toggleOff=" << toggleOff << " toggleOn=" << toggleOn << " aimPath=" << aimPath
                           << " captureOnUs=" << captureOnUs[0] << "," << captureOnUs[1] << "," << captureOnUs[2]
                           << " captureOffUs=" << captureOffUs[0] << "," << captureOffUs[1] << "," << captureOffUs[2] << "\n";
        if (fixtureAim) std::ofstream(aOut, std::ios::app) << "aimStatus=" << aimStatus << "\n";
        return (authored ? captured == 0 : captured >= 3) && supplied >= 3 && applied >= 3 && owned >= 3 && sourceOk >= 3 && clamped == 0 &&
               std::isfinite(maxGripError) && maxGripError <= 0.001f &&
               // Authored independence is checked by the parent against matching control frames,
               // not an earlier animation pose. Rifle must still respond to the changed game target.
               (authored || (aPerturbControl ? !changed : changed)) && staleRejected && rigRejected && futureRejected &&
               rearmRejected && (authored ? !freshCapture : freshCapture) && recovered && emptyRejected && emptyCaptureSkipped && disabledCaptureSkipped && toggleOff && toggleOn && aimPath ? 0 : 23;
    }
    if (!aWeaponCase.empty())
    {
        motiondriver::SetEnabled(!aBaseStack);
        motionhost::SetAir(2); // held-air path: the legs remain owned, game weapon pose must survive
        weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
        int owned = 0;
        for (int i = 0; i < static_cast<int>(std::ceil(0.4f / dt)); ++i)
        {
            feed(false);
            owned += motionhost::ProbeMotionSeam().legMotionSet ? 1 : 0;
        }
        airborne = aWeaponCase == "air";
        if (aWeaponCase == "unknown") weaponBits = mm::integ::kPsmWeaponDrawn;
        if (aWeaponCase == "empty") weaponBits = mm::integ::kPsmWeaponKnown;
        if (aWeaponCase == "release") extraPsm = mm::integ::kPsmScene;
        float final = -1.0f, first = -1.0f;
        int phaseOwned = 0, airLegs = 0;
        for (int i = 0; i < static_cast<int>(std::ceil(0.4f / dt)); ++i)
        {
            final = feed(false);
            if (i == 0) first = final;
            const auto seam = motionhost::ProbeMotionSeam();
            phaseOwned += seam.legMotionSet ? 1 : 0;
            airLegs += seam.legAirborne && seam.legContactScale <= 0.01f ? 1 : 0;
        }
        const float weight = ReadPublished() ? g_read.weight : -1.0f;
        bool hipsLower = false, spineUpper = false;
        for (uint32_t i = 0; i < g_read.numBones; ++i)
        {
            const int32_t gi = g_read.gameIndex[i];
            if (gi < 0 || static_cast<size_t>(gi) >= hashes.size()) continue;
            const bool upper = (g_read.upperBits[i / 32] & (1u << (i % 32))) != 0;
            if (hashes[static_cast<size_t>(gi)] == mm::integ::CNameHash("Hips")) hipsLower = !upper;
            if (hashes[static_cast<size_t>(gi)] == mm::integ::CNameHash("Spine1")) spineUpper = upper;
        }
        const bool held = aWeaponCase == "held" || aWeaponCase == "air" || aWeaponCase == "reload";
        const bool released = aWeaponCase == "release";
        std::ofstream(aOut) << "case=" << aWeaponCase << " hz=" << aHz << " base=" << aBaseStack << " owned=" << owned
                            << " phaseOwned=" << phaseOwned << " airLegs=" << airLegs << " first=" << first
                            << " upperScale=" << final << " weight=" << weight << " hipsLower=" << hipsLower << " spineUpper=" << spineUpper << "\n";
        motiondriver::SetEnabled(false); motionhost::SetMotionHooks(nullptr); motiondriver::Shutdown(); motionhost::Shutdown();
        if ((!aBaseStack && owned < 3) || (aBaseStack && owned != 0)) return 3;
        if (airborne && airLegs < 3) return 21;
        if ((aWeaponCase == "empty" || aWeaponCase == "unknown") && first >= 0.3f) return 22;
        return released ? (weight <= 0.05f && phaseOwned == 0 ? 0 : 20) :
               (weight > 0.95f && (held ? final >= 0.0f && final < 0.05f && hipsLower && spineUpper : final > 0.95f) ? 0 : 19);
    }
    float pre = 1.0f;
    bool bounds = true, maskDown = true;
    for (int i = 0; i < 20; ++i)
    {
        const float v = feed(true);
        bounds &= valid(v);
        maskDown &= v <= pre + 1e-5f;
        pre = v;
    }
    const bool armed = pre < 0.05f;
    motiondriver::SetEnabled(!aBaseStack);
    motiondriver::WaitLoader();
    int owned = 0;
    bool motionMaskStable = true;
    for (int i = 0; i < 25; ++i)
    {
        const float v = feed(true);
        const auto p = motionhost::ProbeMotionSeam();
        owned += p.legMotionSet ? 1 : 0;
        bounds &= valid(v);
        if (p.legMotionSet)
            motionMaskStable &= v <= 0.05f;
    }
    if (aAimExit)
    {
        motiondriver::SetOverlayInputs(2, 0, false, false); // keep the pistol drawn, exit aim while v2 still owns
        float previous = ReadPublished() ? g_read.upperScale : pre;
        float first = -1.0f, maxStep = 0.0f, at200ms = 1.0f;
        int ownedRelease = 0;
        bool monotonic = true;
        for (int i = 0; i < static_cast<int>(std::ceil(0.3f / dt)); ++i)
        {
            const float v = feed(false);
            bounds &= valid(v);
            if (i == 0) first = v;
            if ((i + 1) * dt <= 0.2f + dt * 0.5f) at200ms = v;
            maxStep = std::fmax(maxStep, v - previous);
            monotonic &= v + 1e-5f >= previous;
            previous = v;
            ownedRelease += motionhost::ProbeMotionSeam().legMotionSet ? 1 : 0;
        }
        std::ofstream(aOut) << "hz=" << aHz << " legacy=" << aLegacy << " base=" << aBaseStack << " owned=" << owned << " releaseOwned=" << ownedRelease
                            << " first=" << first << " at200ms=" << at200ms << " final=" << previous
                            << " maxStep=" << maxStep << " bounds=" << bounds << " monotonic=" << monotonic << "\n";
        motiondriver::SetEnabled(false); motionhost::SetMotionHooks(nullptr); motiondriver::Shutdown(); motionhost::Shutdown();
        if (!armed || (!aBaseStack && (owned < 15 || ownedRelease < 3)) || (aBaseStack && (owned != 0 || ownedRelease != 0))) return 3;
        if (aLegacy)
            return bounds && motionMaskStable && first > 0.99f && previous > 0.99f ? 0 : 18;
        return bounds && motionMaskStable && monotonic && first < 0.06f && at200ms < 0.99f && previous > 0.99f
               && maxStep <= 1.6f * dt / 0.25f + 1e-5f ? 0 : 17;
    }
    const auto before = motionhost::ProbeMotionSeam();
    motiondriver::SetEnabled(false);
    const float hb = feed(true);
    const auto after = motionhost::ProbeMotionSeam();
    const bool handback = after.handbacks == before.handbacks + 1 && !after.legMotionSet;
    bounds &= valid(hb);
    bool easedUp = true;
    float previous = hb, first = -1.0f;
    for (int i = 0; i < 20; ++i)
    {
        const float v = feed(false);
        if (i == 0)
            first = v;
        bounds &= valid(v);
        easedUp &= v + 1e-5f >= previous && v - previous <= 0.35f;
        previous = v;
    }
    std::ofstream(aOut) << "pre=" << pre << " owned=" << owned << " hb=" << hb << " handback=" << handback
                        << " firstRelease=" << first << " final=" << previous << " bounds=" << bounds
                        << " maskDown=" << maskDown << " motionStable=" << motionMaskStable << " easedUp=" << easedUp << "\n";
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    if (!armed || owned < 15 || !handback)
        return 3;
    if (!bounds || !maskDown || !motionMaskStable || hb > 0.05f)
        return 15;
    if (!easedUp || first < hb - 1e-5f || previous < 0.95f)
        return 16;
    return 0;
}
} // namespace

// motionfix items 1 + 4 child (fresh process): the real in-game frames of MotionFixRealFrames.hpp (side run, toward-camera
// run, crouch walk toward camera; Locomotion mv (0, 1) + haveTarget / targetYawDeg) through the real MMDriver + MotionDriver hooks
// with the MOTION trace on (captured logger). Each window: 90 feeds, the first 30 = warm-up. The trace state at a feed = the
// last MOTION line written up to it (lines come on every change of db / clip / IsPivoting / IsStarting, so the last line holds
// the state of every later feed until the next line). Line 1 of aOut: counts. Exit 0 = all hold; 3 = precondition (v2 owned
// < 55 of the 60 checked feeds of a window, or no line); 4 = pivot=1 on a checked feed; 5 = a *Pivots db / *Pivot* clip on a
// checked feed; 6 = the v2 pick clip (Status) changed on a feed without a line for the new clip on that feed; 7 = a line with
// the trace off; 8 = no periodic (2 Hz) line, or two periodic lines closer than 0.45 s of feed time;
// 9 = no native body turn on v2-owned run feeds; 10 = no bounded yaw command toward the target.
std::vector<std::string> g_v2fixLines;
// vehicle batch (TRAV-1 D): the always-on "UTPP trav:" plan line (8v)
std::vector<std::string> g_travInfoLines;
void TravInfoCapture(const char* aLine)
{
    if (aLine != nullptr && std::strncmp(aLine, "UTPP trav: ", 11) == 0)
        g_travInfoLines.emplace_back(aLine);
}
void MotionFixCapture(const char* aLine)
{
    if (aLine != nullptr && std::strncmp(aLine, "UTPP MOTION ", 12) == 0)
        g_v2fixLines.emplace_back(aLine);
}
int MotionFixChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut)
{
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(aData, "man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetLogger(&MotionFixCapture);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) || !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motiondriver::SetTrace(true);
    auto toIn = [](const motionfix::RealFrame& r) {
        motionhost::FeedInput in;
        in.velWorld[0] = r.vx;
        in.velWorld[1] = r.vy;
        in.velWorld[2] = r.vz;
        in.move[0] = r.mvx;
        in.move[1] = r.mvy;
        in.cameraYawDeg = r.camDeg;
        in.playerYawDeg = r.yawDeg;
        in.locoState = r.loco;
        in.tppActive = true;
        in.dt = r.dt;
        in.psmBlock = r.psm;
        in.haveTarget = r.haveTarget;
        in.targetYawDeg = r.targetYawDeg;
        return in;
    };
    auto field = [](const std::string& aLine, const char* aKey) {
        const size_t p = aLine.find(aKey);
        if (p == std::string::npos)
            return std::string();
        const size_t s = p + std::strlen(aKey);
        return aLine.substr(s, aLine.find(' ', s) - s);
    };
    const motionfix::RealFrame* wins[3] = {motionfix::kSideRun, motionfix::kTowardRun, motionfix::kCrouchToward};
    int code = 0, owned[3] = {0, 0, 0}, bodyActive = 0, pivotFeeds = 0, pivotDbFeeds = 0, clipChanges = 0, clipChangeNoLine = 0, periodic = 0,
        periodicTooClose = 0;
    std::string bad;
    int32_t prevClip = -1;
    float clock = 0.0f, lastPeriodicT = -1.0f;
    for (int w = 0; w < 3; ++w)
        for (int i = 0; i < 90; ++i)
        {
            const size_t before = g_v2fixLines.size();
            g_log = HookLog{};
            motionhost::Feed(toIn(wins[w][i]));
            clock += wins[w][i].dt;
            const bool stepped = g_log.stepResult;
            const std::string pk = StatusField(motiondriver::Status(), "pick=clip:");
            const int32_t clip = stepped ? std::atoi(pk.substr(0, pk.find('@')).c_str()) : -1;
            const bool newLine = g_v2fixLines.size() > before;
            if (newLine && g_v2fixLines.back().find(" chg") == std::string::npos)
            {
                ++periodic;
                if (lastPeriodicT >= 0.0f && clock - lastPeriodicT < 0.45f)
                    ++periodicTooClose;
                lastPeriodicT = clock;
            }
            if (stepped && prevClip >= 0 && clip != prevClip)
            {
                ++clipChanges;
                const std::string want = "clip=" + std::to_string(clip) + ":";
                if (!newLine || g_v2fixLines.back().find(want) == std::string::npos)
                    ++clipChangeNoLine;
            }
            if (stepped)
                prevClip = clip;
            if (i < 30 || !stepped || g_v2fixLines.empty())
                continue;
            ++owned[w];
            if (w == 1 && motionhost::BodyTurnActive())
                ++bodyActive;
            const std::string& l = g_v2fixLines.back();
            if (field(l, " pivot=") != "0")
            {
                ++pivotFeeds;
                if (bad.empty())
                    bad = l;
            }
            const std::string dbn = field(l, " db="), cln = field(l, " clip=");
            if (dbn.find("Pivots") != std::string::npos || cln.find("Pivot") != std::string::npos)
            {
                ++pivotDbFeeds;
                if (bad.empty())
                    bad = l;
            }
        }
    const size_t linesOn = g_v2fixLines.size();
    motiondriver::SetTrace(false);
    for (int i = 0; i < 30; ++i)
        motionhost::Feed(toIn(motionfix::kTowardRun[i]));
    const size_t linesOff = g_v2fixLines.size() - linesOn;
    // The checked run window has nearly converged by feed 30. Replay its first, misaligned frame after warm-up
    // to prove that the native executor receives an actual bounded yaw step while v2 owns the feed.
    const auto& turn = motionfix::kTowardRun[0];
    motionhost::Feed(toIn(turn));
    const bool turnActive = motionhost::BodyTurnActive();
    const float command = motionhost::BodyYawCommand(turn.yawDeg, turn.yawDeg);
    const float error = std::remainder(turn.targetYawDeg - turn.yawDeg, 360.0f);
    const float step = std::remainder(command - turn.yawDeg, 360.0f);
    const bool bodyCommand = turnActive && std::isfinite(command) && step * error > 0.0f && std::fabs(step) <= std::fabs(error);
    {
        std::ofstream o(aOut);
        o << "owned " << owned[0] << "/" << owned[1] << "/" << owned[2] << " of 60, bodyActive " << bodyActive << "/60, bodyCommand " << (bodyCommand ? 1 : 0)
          << " step " << step << " error " << error << ", lines " << linesOn << ", pivotFeeds "
          << pivotFeeds << ", pivotDbFeeds " << pivotDbFeeds << ", clipChanges " << clipChanges << " noLine " << clipChangeNoLine
          << ", periodic " << periodic << " tooClose " << periodicTooClose << ", linesTraceOff " << linesOff
          << (bad.empty() ? std::string() : ", first bad: " + bad) << "\n";
        for (const std::string& l : g_v2fixLines)
            o << l << "\n";
    }
    if (owned[0] < 55 || owned[1] < 55 || owned[2] < 55 || linesOn == 0)
        code = 3;
    else if (pivotFeeds > 0)
        code = 4;
    else if (pivotDbFeeds > 0)
        code = 5;
    else if (clipChanges == 0 || clipChangeNoLine > 0)
        code = 6;
    else if (linesOff > 0)
        code = 7;
    else if (periodic == 0 || periodicTooClose > 0)
        code = 8;
    else if (bodyActive < 55)
        code = 9;
    else if (!bodyCommand)
        code = 10;
    return code;
}

int main(int argc, char** argv)
{
    if (argc == 5 && std::string(argv[1]) == "--motionfix-child")
        return MotionFixChildMain(argv[2], argv[3], argv[4]);
    if (argc == 7 && std::string(argv[1]) == "--scenario")
        return ChildMain(argv[2], argv[3], argv[4], argv[5], argv[6]);
    if (argc == 5 && std::string(argv[1]) == "--legctx-child")
        return LegCtxChildMain(argv[2], argv[3], argv[4]);
    if (argc == 5 && std::string(argv[1]) == "--aimjog-child")
        return AimJogChildMain(argv[2], argv[3], argv[4]);
    if (argc == 5 && std::string(argv[1]) == "--airclips-child")
        return AirClipsChildMain(argv[2], argv[3], argv[4]);
    if (argc == 5 && std::string(argv[1]) == "--maskhandover-child")
        return MaskHandoverChildMain(argv[2], argv[3], argv[4]);
    if ((argc == 6 || argc == 7) && std::string(argv[1]) == "--mask-exit-child")
        return MaskHandoverChildMain(argv[2], argv[3], argv[4], true, std::stoi(argv[5]),
                                    argc == 7 && std::string(argv[6]) == "legacy", argc == 7 && std::string(argv[6]) == "v1");
    if ((argc == 7 || argc == 8) && std::string(argv[1]) == "--weapon-upper-child")
        return MaskHandoverChildMain(argv[2], argv[3], argv[4], false, std::stoi(argv[5]), false,
                                    std::string(argv[6]).find("v1-") == 0, std::string(argv[6]).substr(3),
                                    argc == 8 && std::string(argv[7]) == "perturb-control");
    // optional trailing --shard <i>/<n> (sharded ctest entries, tests/motiondriverinteg.cmake): shard 0 runs everything up to the tail
    // children (section 1, every in-process section, in the original order: they share the process-global v1 / v2 state) plus its
    // share of the tail units; shards 1 .. n-1 run only their share of the tail units (7e - 7i, the pistol aim exits and the weapon
    // upper / grip cases per frame rate), which are independent child processes. Without the argument everything runs, as before.
    int shardIdx = 0, shardCnt = 1;
    if (argc >= 3 && std::string(argv[argc - 2]) == "--shard")
    {
        int i = -1, n = 0;
        if (std::sscanf(argv[argc - 1], "%d/%d", &i, &n) != 2 || n < 1 || i < 0 || i >= n)
        {
            std::printf("usage: ... --shard <i>/<n> with 0 <= i < n\n");
            return 2;
        }
        shardIdx = i;
        shardCnt = n;
        argc -= 2;
    }
    const bool coreShard = shardIdx == 0;
    int tailUnit = 0; // the tail units are numbered in their original order; unit u belongs to shard u % n
    auto mineTail = [&](int aUnit) { return aUnit % shardCnt == shardIdx; };
    // optional --replay-csv <csv of utpp_motion_replay --db <woman v2> run in the same build>: the woman reference picks must equal
    // the replay's own motionclip / clipT rows (FeedReplayMotion.cpp csv columns feed,t,...,motionclip,...,clipT)
    std::vector<char*> av(argv, argv + argc);
    std::map<std::string, std::pair<std::string, float>> csvRows;
    std::string csvPath;
    if (av.size() > 2 && std::string(av[1]) == "--replay-csv")
    {
        csvPath = av[2];
        av.erase(av.begin() + 1, av.begin() + 3);
        argc = static_cast<int>(av.size());
        argv = av.data();
        std::ifstream in(csvPath);
        std::string line;
        std::getline(in, line); // header
        while (std::getline(in, line))
        {
            std::vector<std::string> c;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ','))
                c.push_back(cell);
            if (c.size() >= 17)
                csvRows[c[0] + "|" + c[1]] = {c[9], static_cast<float>(std::atof(c[14].c_str()))};
        }
        Check(!csvRows.empty(), "replay csv rows loaded", csvPath + " " + std::to_string(csvRows.size()));
    }
    if (argc < 4)
    {
        std::printf("usage: utpp_motion_driver_integ_test [--replay-csv <csv>] <data dir> <base database.utppmm> <feed> [feed ...]\n");
        return 2;
    }
    if (argc < 4)
    {
        std::printf("usage: utpp_motion_driver_integ_test <data dir> <base database.utppmm> <feed> [feed ...]\n");
        return 2;
    }
    const std::string data = argv[1];
    {
        const std::string v1 = argv[2];
        g_baseDb.assign(v1.begin(), v1.end());
        Check(Exists(v1), "base database present", v1);
    }
    const motiondriver::Paths pw = DataPaths(data, "woman"), pm = DataPaths(data, "man");
    for (const motiondriver::Paths* p : {&pw, &pm})
        for (const std::string* f : {&p->files.db, &p->files.chooser, &p->files.overlay, &p->files.style, &p->traversal})
            Check(Exists(*f), "data file present", *f);
    std::vector<FeedFile> feeds;
    for (int i = 3; i < argc; ++i)
    {
        FeedFile f;
        const bool ok = LoadFeed(argv[i], f);
        Check(ok, "feed loaded", std::string(argv[i]) + " frames " + std::to_string(f.fr.size()));
        if (ok)
            feeds.push_back(std::move(f));
    }
    Rig rw, rm;
    std::unique_ptr<mm::motion::MotionDatabase> dbw, dbm;
    const bool rwOk = RigFromDb(pw.files.db, rw, &dbw), rmOk = RigFromDb(pm.files.db, rm, &dbm);
    Check(rwOk && rw.bones == 73, "woman motion rig 73 bones", std::to_string(rw.bones));
    Check(rmOk && rm.bones == 73, "man motion rig 73 bones", std::to_string(rm.bones));
    chooser::Table tw, tm;
    {
        std::string e1, e2;
        const bool a = tw.LoadFile(pw.files.chooser.c_str(), e1), b = tm.LoadFile(pm.files.chooser.c_str(), e2);
        Check(a && b, "chooser tables loaded", e1 + " " + e2);
    }
    if (g_fail)
    {
        std::printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
        return 1;
    }
    motiondriver::SetPathsOverride(0, pw);
    motiondriver::SetPathsOverride(1, pm);

    // ---------------------------------------------------------------- 1, 3, 4: v1 identity scenarios, each in a fresh child process
    // (v1 runtime state survives SetEnabled / LoadDatabase / re-arm, so two in-process runs are not comparable)
    // Every child of this runner (these scenarios, 7e - 7i, the pistol aim exits and the weapon upper / grip cases at the end) is submitted to
    // the child pool here and starts at once, up to the pool's bound, while the in-process sections run; each block below waits for its own
    // children at the position where it used to start them, so the Check lines keep their order and messages.
    struct ScenarioKids
    {
        ChildRef ref, ref2, off, rel, yield, miss, other, tail, raw, handin, toggleoff;
    };
    std::vector<ScenarioKids> scenarioKids;
    for (size_t fi = 0; fi < feeds.size() && coreShard; ++fi)
    {
        const std::string feedPath = argv[3 + fi];
        ScenarioKids k;
        k.ref = SubmitScenario("nohooks", argv[1], argv[2], feedPath);
        k.ref2 = SubmitScenario("nohooks", argv[1], argv[2], feedPath);
        k.off = SubmitScenario("off", argv[1], argv[2], feedPath);
        k.rel = SubmitScenario("release", argv[1], argv[2], feedPath);
        k.yield = SubmitScenario("yield", argv[1], argv[2], feedPath);
        k.miss = SubmitScenario("dbmissing", argv[1], argv[2], feedPath);
        k.other = SubmitScenario("rigmismatch", argv[1], argv[2], feedPath);
        k.tail = SubmitScenario("tail", argv[1], argv[2], feedPath);
        k.raw = SubmitScenario("raw", argv[1], argv[2], feedPath);
        k.handin = SubmitScenario("handin", argv[1], argv[2], feedPath);
        k.toggleoff = SubmitScenario("toggleoff", argv[1], argv[2], feedPath);
        scenarioKids.push_back(std::move(k));
    }
    const std::string v6Db = data + "\\utpp_base.utppmm";
    // tail units 0 - 4: 7e, 9a, 7f, 7h, 7i
    const bool mine7e = mineTail(tailUnit++), mine9a = mineTail(tailUnit++), mine7f = mineTail(tailUnit++), mine7h = mineTail(tailUnit++),
               mine7i = mineTail(tailUnit++);
    ChildRef kid7e, kid9a, kid7f, kid7h, kid7i;
    if (mine7e)
        kid7e = SubmitModeChild("legctx7e", L"--legctx-child", data, v6Db, L"", Exists(v6Db));
    if (mine9a)
        kid9a = SubmitModeChild("motionfix9a", L"--motionfix-child", data, argv[2]);
    if (mine7f)
        kid7f = SubmitModeChild("aimjog7f", L"--aimjog-child", data, v6Db, L"", Exists(v6Db));
    if (mine7h)
        kid7h = SubmitModeChild("airclips7h", L"--airclips-child", data, v6Db, L"", Exists(v6Db));
    if (mine7i)
        kid7i = SubmitModeChild("mask7i", L"--maskhandover-child", data, v6Db, L"", Exists(v6Db));
    struct HzKid
    {
        int hz;
        std::string variant;
        ChildRef kid, control; // control: only the authored grip cases (perturb-control twin)
        bool authored = false;
    };
    std::vector<HzKid> exitKids, weaponKids;
    for (int hz : {30, 60, 144}) // tail units 6 - 8
    {
        const bool mine = mineTail(tailUnit++);
        for (const char* variant : {"v2", "legacy", "v1"})
        {
            if (!mine)
                continue;
            HzKid k{hz, variant, {}, {}, false};
            k.kid = SubmitModeChild("aimexit", L"--mask-exit-child", data, v6Db, L" " + std::to_wstring(hz) + L" " + Wide(variant), Exists(v6Db));
            exitKids.push_back(std::move(k));
        }
    }
    for (int hz : {30, 60, 144}) // tail units 9 - 11
    {
        const bool mine = mineTail(tailUnit++);
        for (const char* variant : {"v1-held", "v2-held", "v1-air", "v2-air", "v1-reload", "v2-reload",
                                    "v1-off", "v2-off", "v1-empty", "v2-empty", "v1-unknown", "v2-unknown", "v1-release", "v2-release",
                                    "v2-grip-pistol", "v2-grip-rifle", "v2-grip-man-pistol", "v2-grip-man-rifle",
                                    "v2-grip-aim-pistol", "v2-grip-man-aim-pistol", "v2-reload-mask", "v2-reload-mask-man",
                                    "v2-nue-recoil-hipfire-2h-slide", "v2-nue-recoil-hipfire-1h-slide",
                                    "v2-nue-recoil-hipfire-2h-slide-man"})
        {
            if (!mine)
                continue;
            HzKid k{hz, variant, {}, {}, false};
            const std::wstring tail = L" " + std::to_wstring(hz) + L" " + Wide(variant);
            k.kid = SubmitModeChild("weaponupper", L"--weapon-upper-child", data, v6Db, tail, Exists(v6Db));
            const bool grip = k.variant.find("grip") != std::string::npos;
            k.authored = grip && (k.variant.find("pistol") != std::string::npos || k.variant.find("rifle") != std::string::npos); // Rifle: authored since bbcc18a
            if (k.authored) // the perturb-control twin of the same case; its arms csv is compared with the main child's
                k.control = SubmitModeChildAt(k.kid.out + ".control", L"--weapon-upper-child", data, v6Db, tail + L" perturb-control");
            weaponKids.push_back(std::move(k));
        }
    }
    if (coreShard) // section 1 and every in-process section (up to the tail children below), in their original order
    {
    for (size_t fi = 0; fi < feeds.size(); ++fi)
    {
        const FeedFile& f = feeds[fi];
        ScenarioKids& kids = scenarioKids[fi];
        ScenarioOut ref, ref2, off, rel, yield, miss, other, tail, raw, handin, toggleoff;
        const bool ok = CollectScenario(kids.ref, ref) && CollectScenario(kids.ref2, ref2) &&
                        CollectScenario(kids.off, off) && CollectScenario(kids.rel, rel) &&
                        CollectScenario(kids.yield, yield) && CollectScenario(kids.miss, miss) &&
                        CollectScenario(kids.other, other) && CollectScenario(kids.tail, tail) &&
                        CollectScenario(kids.raw, raw) && CollectScenario(kids.handin, handin) &&
                        CollectScenario(kids.toggleoff, toggleoff);
        Check(ok && ref.fr.size() == f.fr.size(), "child scenarios ran " + f.name, std::to_string(ref.fr.size()));
        if (!ok || ref.fr.size() != f.fr.size())
            continue;
        auto same = [&](const ScenarioOut& s, size_t from, size_t& v2) {
            size_t n = 0;
            v2 = 0;
            for (size_t i = from; i < ref.fr.size() && i < s.fr.size(); ++i)
            {
                n += s.fr[i].hash == ref.fr[i].hash && s.fr[i].published;
                v2 += s.fr[i].stepped || s.fr[i].applied;
            }
            return n;
        };
        const size_t n = ref.fr.size();
        size_t v2 = 0, visible = 0;
        for (const FrameOut& o : ref.fr)
            visible += o.published;
        const size_t s2 = same(ref2, 0, v2);
        Check(s2 == n && visible == n, "1 precondition: two fresh base runs without hooks identical " + f.name,
              std::to_string(s2) + "/" + std::to_string(n));
        size_t calls = 0;
        for (const FrameOut& o : off.fr)
            calls += o.stepCalled;
        const size_t s1 = same(off, 0, v2);
        Check(s1 == n && v2 == 0 && calls == n, "1 mm.motionDriver off, hooks installed: base snapshots bit-identical " + f.name,
              std::to_string(s1) + "/" + std::to_string(n) + " step calls " + std::to_string(calls) + " motion frames " + std::to_string(v2));
        {
            // 6 batch 1b feed dump
            auto col = [](const std::string& aLine, int aK) {
                std::istringstream ds(aLine);
                std::string c;
                for (int k = 0; ds >> c; ++k)
                    if (k == aK)
                        return c;
                return std::string();
            };
            auto cols = [](const std::string& aLine) {
                std::istringstream ds(aLine);
                std::string c;
                int k = 0;
                while (ds >> c)
                    ++k;
                return k;
            };
            bool width = !ref.dump.empty();
            for (const std::string& l : ref.dump)
                width = width && cols(l) == 31;
            Check(ref.dump.size() == n && ref.dump == ref2.dump && width,
                  "6 precondition: feed dump one line per feed, 31 columns, two runs without hooks identical (t / baseMicros masked) " + f.name,
                  std::to_string(ref.dump.size()) + "/" + std::to_string(n));
            size_t firstDiff = 0;
            while (firstDiff < ref.dump.size() && firstDiff < off.dump.size() && ref.dump[firstDiff] == off.dump[firstDiff])
                ++firstDiff;
            Check(!ref.dump.empty() && off.dump == ref.dump, "6 mm.motionDriver off, hooks installed: feed dump identical to no hooks " + f.name,
                  std::to_string(off.dump.size()) + " lines, first diff " + std::to_string(firstDiff));
            size_t rawSame = 0, dashT = 0, dashR = 0;
            for (size_t i = 0; i < tail.dump.size() && i < ref.dump.size(); ++i)
            {
                bool eq = true;
                for (int k = 1; k <= 10 && eq; ++k) // dt vx vy vz mvx mvy yawDeg camDeg loco psm (raw FeedInput)
                    eq = col(tail.dump[i], k) == col(ref.dump[i], k);
                rawSame += eq;
                dashT += col(tail.dump[i], 11) == "-";
                dashR += col(ref.dump[i], 11) == "-";
            }
            const size_t owned = static_cast<size_t>(std::max(0, std::atoi(StatusField(tail.status, "tail=").c_str())));
            Check(tail.dump.size() == n && rawSame == n && owned > 0 && dashT == dashR + owned,
                  "6 v2 owns the feed: dump has every feed with the raw input, v1 columns '-' exactly on the v2-owned feeds " + f.name,
                  std::to_string(tail.dump.size()) + "/" + std::to_string(n) + " raw " + std::to_string(rawSame) + " '-' " +
                      std::to_string(dashT) + " = " + std::to_string(dashR) + " + tail " + std::to_string(owned));
        }
        {
            // v1-only cost of the installed hooks with mm.motionDriver off (INFO: child processes, wall-clock noise; the [P] bound on the
            // apply hook itself is in the unit test)
            auto pct = [](const ScenarioOut& s, size_t q) {
                std::vector<float> us;
                for (const FrameOut& o : s.fr)
                    us.push_back(o.feedUs);
                std::sort(us.begin(), us.end());
                return us.empty() ? 0.0f : us[std::min(us.size() - 1, (us.size() * q) / 100)];
            };
            std::printf("INFO whole Feed, mm.motionDriver off with hooks vs no hooks %s: p50 %.1f / %.1f us, p95 %.1f / %.1f us\n", f.name.c_str(),
                        pct(off, 50), pct(ref, 50), pct(off, 95), pct(ref, 95));
        }
        const size_t s3 = same(rel, 0, v2);
        Check(s3 == n && v2 == 0 && StatusField(rel.status, "skip=") == "overlay-release",
              "3 overlay release (melee / carry) whole feed: no motion pose, snapshots = base " + f.name,
              std::to_string(s3) + "/" + std::to_string(n) + " motion " + std::to_string(v2) + " skip=" + StatusField(rel.status, "skip="));
        size_t before = 0;
        for (size_t i = 0; i < n / 2; ++i)
            before += yield.fr[i].applied;
        // handover (MotionDriver g_hoLeft): the yield frame shows the held v2 pose, then it blends into v1 over
        // motionhost::kDefaultHardOutTime of feed time (same clock as MotionDriver StepLocked); afterwards the snapshots are v1's.
        size_t ho = 1;
        {
            float left = motionhost::kDefaultHardOutTime;
            for (size_t i = n / 2 + 1; i < n; ++i)
            {
                const float dt = f.fr[i].dt;
                if (std::isfinite(dt) && dt > 0.0f)
                    left = std::fmax(0.0f, left - std::fmin(dt, 0.25f));
                if (left <= 0.0f)
                    break;
                ++ho;
            }
        }
        // blend presence: on every handover frame the published pose differs from the v1 source pose built by PublishLocked
        // (pre-apply). Comparing against the never-v2 reference run cannot fail: after commit A v1 continues from the tailed
        // v2 state, so those frames differ from the reference with or without a blend.
        size_t hoDiff = 0;
        for (size_t i = n / 2; i < n / 2 + ho && i < n; ++i)
            hoDiff += yield.fr[i].published && yield.fr[i].changedBySrc;
        // commit A: while v2 owns a feed the v1 search does not run and the v1 foot lock / stance guard process the v2 pose, so
        // after the handback v1 continues from that state (not from a never-v2 run): every frame after the handover is a
        // published v1 snapshot (no v2 step / apply) and the v1 matcher moves the pose (hash changes on most frames).
        v2 = 0;
        size_t pubAfter = 0, moving = 0;
        for (size_t i = n / 2 + ho; i < n && i < yield.fr.size(); ++i)
        {
            pubAfter += yield.fr[i].published;
            v2 += yield.fr[i].stepped || yield.fr[i].applied;
            moving += i > 0 && yield.fr[i].hash != yield.fr[i - 1].hash;
        }
        const size_t after = n - n / 2 - ho;
        Check(before > 0 && pubAfter == after && v2 == 0 && moving * 2 > after,
              "3 motion runs, then overlay release mid-feed: base snapshots once the handover is over " + f.name,
              "motion frames before " + std::to_string(before) + ", handover frames " + std::to_string(ho) + ", base published after " +
                  std::to_string(pubAfter) + "/" + std::to_string(after) + " moving " + std::to_string(moving) + " motion after " +
                  std::to_string(v2));
        Check(ho >= 2 && hoDiff == ho, "3 handover after the yield: held motion pose blending into base (not the base source yet) " + f.name,
              std::to_string(hoDiff) + "/" + std::to_string(ho) + " handover frames differ from the base source pose");
        // ---- commit A (6.10.1 item 3, 6.10.6): handover continuity, foot lock on v2 frames, feed cost
        // Continuity, both directions (handin v1 -> v2 by toggle on, yield v2 -> v1 by release, toggleoff v2 -> v1 by toggle):
        // published P_i = nlerp(H, S_i, s_i) with H = P at the switch frame - 1 (held), S_i = the source pose before the apply hook,
        // s_i = smoothstep over kDefaultHardOutTime (MotionDriver BlendFromHeld). Per frame of the window and per bone:
        //   step(P) <= k * (max(own step of S_i, own step of the held side just before the switch) + ds_i * theta_i) + 0.01 deg
        // theta_i = max angle H -> S_i, ds_i = s_i - s_{i-1}; k = sqrt(2) = worst nlerp angular-rate factor for shortest-arc
        // quaternions (|lerp| >= cos 45 deg) [derived]. A hard cut has step = theta at ds = 1 and fails when theta is large.
        for (const auto& sc : {std::make_pair(&handin, "handin v1->v2 (toggle on)"), std::make_pair(&yield, "yield v2->v1 (release)"),
                               std::make_pair(&toggleoff, "toggle off v2->v1")})
        {
            const ScenarioOut& r = *sc.first;
            const bool toggle = sc.first == &toggleoff;
            const size_t half = n / 2;
            if (r.fr.size() != n || half < 2)
            {
                Check(false, std::string("A continuity ") + sc.second + " " + f.name, "frames " + std::to_string(r.fr.size()));
                continue;
            }
            const float T = motionhost::kDefaultHardOutTime;
            auto smooth = [&](float left) {
                const float a = 1.0f - std::fmin(1.0f, std::fmax(0.0f, left) / T);
                return a * a * (3.0f - 2.0f * a);
            };
            const float heldOwn = r.fr[half - 1].stepP;
            float left = T, sPrev = 0.0f, worstMargin = 1e9f, maxStep = 0.0f, maxTheta = 0.0f, cutStep = 0.0f, cutBound = 0.0f;
            size_t frames = 0, bad = 0, blendFrames = 0, blendChanged = 0;
            for (size_t i = half; i < n; ++i)
            {
                const float dt = std::isfinite(f.fr[i].dt) && f.fr[i].dt > 0.0f ? std::fmin(f.fr[i].dt, 0.25f) : 0.0f;
                if (i > half || toggle) // handin / yield start at left = T on the switch frame; toggle off decrements first
                    left = std::fmax(0.0f, left - dt);
                const float sI = smooth(left);
                // own motion of the source: on the switch frame the source itself jumps v1 <-> v2 (stepS = theta there), so
                // only the held side's own step counts; afterwards consecutive source poses of the same side (stepS)
                const float own = i == half ? heldOwn : std::max(r.fr[i].stepS, heldOwn);
                const float bound = 1.41421356f * (own + (sI - sPrev) * r.fr[i].theta) + 0.01f;
                if (i == half)
                {
                    cutStep = r.fr[i].theta; // what a hard cut would show on the switch frame
                    cutBound = bound;
                }
                if (left > 0.0f)
                {
                    ++blendFrames;
                    blendChanged += r.fr[i].published && r.fr[i].changedBySrc;
                }
                bad += r.fr[i].stepP > bound;
                worstMargin = std::fmin(worstMargin, bound - r.fr[i].stepP);
                maxStep = std::fmax(maxStep, r.fr[i].stepP);
                maxTheta = std::fmax(maxTheta, r.fr[i].theta);
                ++frames;
                sPrev = sI;
                if (left <= 0.0f)
                    break;
            }
            char d[320];
            std::snprintf(d, sizeof d, "window %zu frames, max step %.2f deg, max theta %.2f deg, hard-cut step %.2f deg vs switch-frame bound %.2f deg, worst margin %.2f deg, over %zu",
                          frames, maxStep, maxTheta, cutStep, cutBound, worstMargin, bad);
            Check(frames >= 2 && bad == 0, std::string("A continuity ") + sc.second + ": step <= own + inertializer bound " + f.name, d);
            // the bound must reject a hard cut on this window, else the check above proves nothing here
            Check(cutStep > cutBound, std::string("A continuity ") + sc.second + ": a hard cut would exceed the switch-frame bound " + f.name, d);
            Check(blendFrames >= 2 && blendChanged == blendFrames,
                  std::string("A continuity ") + sc.second + ": every blend frame differs from the source pose " + f.name,
                  std::to_string(blendChanged) + "/" + std::to_string(blendFrames));
        }
        {
            // foot lock on v2 frames: toe slide while the foot is planted, with the leg tail vs the raw v2 pose (pre-commit-A path).
            // "Planted" = the contact the driver hands to the leg tail with the same feed (MotionSetFrame::contact, tail run, frame-aligned
            // for both runs), >= 0.5 on this and the previous frame; the toe is measured in the FootLocker frame (capsule integrated
            // from the feed velocity, actor yaw - the visual root offset is already in the published pose). It used to be the CONT of
            // the picked clip. Since the live crossfade of the outgoing clips (bca1b73) and the visual root (3b845df) the displayed
            // pose is a blend of several clips and a re-pick every 0.15-0.3 s (flick_180_run: SpinTransition 180 loop under this
            // open-loop feed) changes the picked clip's CONT before the shown feet follow it: the pick said "right foot planted"
            // while the tail was told 0.05 (feed 420), so the old metric counted swing-foot motion as slide, in the raw run as much as
            // in the tail run (flick_180_run: tail 2.382 cm vs raw 2.309 cm). The lock is driven by the published contact, so that is
            // the contact the slide has to be judged on. The criterion is unchanged (tail < raw, same n); the pick-contact figure stays
            // in the detail as INFO.
            double sumT = 0, sumR = 0, pickT = 0, pickR = 0;
            size_t nT = 0, nR = 0, pickNT = 0, pickNR = 0, tailFrames = 0, contactFrames = 0;
            const size_t frames = std::min({n, tail.fr.size(), raw.fr.size()});
            auto plantedSlide = [&](const ScenarioOut& run, double& aSum, size_t& aCount) {
                bool prevPlanted[2] = {false, false};
                float prevToe[2][2] = {{0, 0}, {0, 0}};
                for (size_t i = 0; i < frames; ++i)
                    for (int k = 0; k < 2; ++k)
                    {
                        const bool planted = tail.fr[i].haveContact && run.fr[i].toeOk[k] && tail.fr[i].tailContact[k] >= 0.5f;
                        if (planted && prevPlanted[k])
                        {
                            aSum += std::hypot(run.fr[i].toeW[k][0] - prevToe[k][0], run.fr[i].toeW[k][1] - prevToe[k][1]);
                            ++aCount;
                        }
                        prevPlanted[k] = planted;
                        prevToe[k][0] = run.fr[i].toeW[k][0];
                        prevToe[k][1] = run.fr[i].toeW[k][1];
                    }
            };
            plantedSlide(tail, sumT, nT);
            plantedSlide(raw, sumR, nR);
            for (size_t i = 0; i < frames; ++i)
            {
                contactFrames += tail.fr[i].haveContact;
                for (int k = 0; k < 2; ++k)
                {
                    if (tail.fr[i].slide[k] >= 0.0f)
                        pickT += tail.fr[i].slide[k], ++pickNT;
                    if (raw.fr[i].slide[k] >= 0.0f)
                        pickR += raw.fr[i].slide[k], ++pickNR;
                }
            }
            for (const FrameOut& o : tail.fr)
                tailFrames += o.applied;
            const double mT = nT ? 100.0 * sumT / double(nT) : -1.0, mR = nR ? 100.0 * sumR / double(nR) : -1.0;
            char d[400];
            std::snprintf(d, sizeof d,
                          "mean toe slide per planted frame (tail contact): tail %.3f cm (n %zu), raw %.3f cm (n %zu); total tail %.1f cm raw %.1f cm; "
                          "INFO pick-CONT metric tail %.3f cm (n %zu) raw %.3f cm (n %zu); tail frames with a contact %zu; status tail=%s",
                          mT, nT, mR, nR, 100.0 * sumT, 100.0 * sumR, pickNT ? 100.0 * pickT / double(pickNT) : -1.0, pickNT,
                          pickNR ? 100.0 * pickR / double(pickNR) : -1.0, pickNR, contactFrames, StatusField(tail.status, "tail=").c_str());
            Check(nT > 20 && nR > 20 && mT < mR && contactFrames > 0 && std::atoi(StatusField(tail.status, "tail=").c_str()) > 0 &&
                      StatusField(tail.status, "layoutSkips=") == "0",
                  "A motion frames go through the leg tail: foot slide during contacts lower than the raw motion pose " + f.name, d);
            std::vector<float> us;
            for (const FrameOut& o : tail.fr)
                if (o.applied)
                    us.push_back(o.feedUs);
            std::sort(us.begin(), us.end());
            const float p50 = us.empty() ? 1e9f : us[std::min(us.size() - 1, (us.size() * 50) / 100)];
            const float p95 = us.empty() ? 1e9f : us[std::min(us.size() - 1, (us.size() * 95) / 100)];
            const float p99 = us.empty() ? 1e9f : us[std::min(us.size() - 1, (us.size() * 99) / 100)];
            const float maxUs = us.empty() ? 1e9f : us.back();
            std::printf("INFO whole Feed, v2 with leg tail %s: n %zu, p50 %.1f us, p95 %.1f us, p99 %.1f us, max %.1f us\n",
                        f.name.c_str(), us.size(), p50, p95, p99, maxUs);
            Check(!us.empty() && p95 <= adapter::Budget::kFeedP95Us,
                  "A whole Feed with motion + leg tail p95 <= adapter::Budget::kFeedP95Us " + f.name,
                  "p95 " + std::to_string(p95) + " us over " + std::to_string(us.size()) + " motion frames, budget " +
                      std::to_string(adapter::Budget::kFeedP95Us));
        }
        for (const auto& c : {std::make_pair(&miss, "db-missing"), std::make_pair(&other, "rig-mismatch")})
        {
            const size_t s5 = same(*c.first, 0, v2);
            Check(StatusField(c.first->status, "reason=") == c.second, std::string("4 ") + c.second + ": reason in status " + f.name,
                  c.first->status);
            Check(s5 == n && v2 == 0, std::string("4 ") + c.second + ": no motion pose, snapshots = base " + f.name,
                  std::to_string(s5) + "/" + std::to_string(n));
        }
    }

    // ---------------------------------------------------------------- 2. mm.motionDriver on, woman then man
    motionhost::SetMotionHooks(&kHooks);
    for (int body = 0; body <= 1; ++body)
    {
        const Rig& rig = body == 0 ? rw : rm;
        const mm::motion::MotionDatabase& db = body == 0 ? *dbw : *dbm;
        const chooser::Table& table = body == 0 ? tw : tm;
        const char* bn = body == 0 ? "woman" : "man";
        motiondriver::SetRigBody(body);
        motiondriver::SetOverlayInputs(0, 0, false, false);
        for (size_t fi = 0; fi < feeds.size(); ++fi)
        {
            const FeedFile& f = feeds[fi];
            ResetBase(rig);
            motiondriver::SetEnabled(false);
            motiondriver::SetEnabled(true);
            // load the body's bundle before the measured run (first Step starts the worker)
            {
                motionhost::FeedInput warm = ToInput(f.fr.front());
                motionhost::Feed(warm);
                motiondriver::WaitLoader();
                ResetBase(rig);
                motiondriver::SetEnabled(false);
                motiondriver::SetEnabled(true);
            }
            int32_t lb = -1;
            Check(motiondriver::LoadedBody(lb) && lb == body && motiondriver::Reason() == Fallback::None,
                  std::string("2 ") + bn + " bundle loaded, armed, no fallback " + f.name, adapter::FallbackName(motiondriver::Reason()));
            const std::vector<FrameOut> run = RunFeed(f, kNoop);
            size_t allowed = 0, stepped = 0, published = 0, poseOk = 0, gateClosed = 0, gateClosedNoMotion = 0;
            std::string why;
            for (size_t i = 0; i < run.size(); ++i)
            {
                const int det = mm::integ::PsmDetailOf(static_cast<uint32_t>(f.fr[i].psm));
                const bool canRun = run[i].effGate == 0 && !(det >= 0 && mm::integ::PsmDetailReleases(det));
                if (!canRun)
                {
                    ++gateClosed;
                    gateClosedNoMotion += !run[i].stepped && !run[i].applied;
                    continue;
                }
                ++allowed;
                stepped += run[i].stepped;
                published += run[i].applied;
            }
            // every frame with a v2 pose: the snapshot the FK detour reads passed MotionPoseOk in RunFeed
            for (const FrameOut& o : run)
                if (o.applied)
                {
                    poseOk += o.motionPoseOk;
                    if (!o.motionPoseOk && why.empty())
                        why = o.motionWhy;
                }
            Check(allowed > 0 && stepped == allowed && published == allowed,
                  std::string("2 ") + bn + " motion pose on every allowed frame " + f.name,
                  std::to_string(stepped) + "/" + std::to_string(published) + "/" + std::to_string(allowed) + " (stepped/published/allowed)");
            Check(gateClosedNoMotion == gateClosed, std::string("2 ") + bn + " no motion pose on gate-closed / PSM-release frames " + f.name,
                  std::to_string(gateClosedNoMotion) + "/" + std::to_string(gateClosed));
            Check(published > 0 && poseOk == published, std::string("2 ") + bn + " every published motion pose finite, unit quats, 73 bones " + f.name,
                  std::to_string(poseOk) + "/" + std::to_string(published) + " " + why);
            const std::vector<RefPick> ref = ReferencePicks(db, table, f, run);
            size_t cmp = 0, eq = 0, firstDiff = SIZE_MAX;
            for (size_t i = 0; i < run.size() && i < ref.size(); ++i)
            {
                if (!run[i].stepped)
                    continue;
                ++cmp;
                const bool same = run[i].clip == ref[i].clip && run[i].db == ref[i].db && std::fabs(run[i].clipTime - ref[i].clipTime) < 0.006f;
                eq += same;
                if (!same && firstDiff == SIZE_MAX)
                    firstDiff = i;
            }
            std::string d = std::to_string(eq) + "/" + std::to_string(cmp);
            if (firstDiff != SIZE_MAX)
                d += " first diff frame " + std::to_string(firstDiff) + " driver clip " + std::to_string(run[firstDiff].clip) + "@" +
                     std::to_string(run[firstDiff].clipTime) + " db" + std::to_string(run[firstDiff].db) + " replay clip " +
                     std::to_string(ref[firstDiff].clip) + "@" + std::to_string(ref[firstDiff].clipTime) + " db" + std::to_string(ref[firstDiff].db);
            Check(cmp > 0 && eq == cmp && !ref.empty(), std::string("2 ") + bn + " picks = utpp_motion_replay adapter path " + f.name, d);
            if (body == 0 && !csvRows.empty())
            {
                // the driver's picks against the replay executable's own rows (woman db); rows exist only for frames with a pose
                size_t rows = 0, rowEq = 0, firstBad = SIZE_MAX;
                for (size_t i = 0; i < run.size(); ++i)
                {
                    char t[32];
                    std::snprintf(t, sizeof t, "%.3f", static_cast<double>(f.fr[i].t));
                    const auto it = csvRows.find(f.name + "|" + t);
                    if (it == csvRows.end() || run[i].clip < 0 || static_cast<uint32_t>(run[i].clip) >= db.NumClips())
                        continue;
                    ++rows;
                    const bool same = std::string(db.ClipName(static_cast<uint32_t>(run[i].clip))) == it->second.first &&
                                      std::fabs(run[i].clipTime - it->second.second) < 0.006f; // status %.2f vs csv %.3f
                    rowEq += same;
                    if (!same && firstBad == SIZE_MAX)
                        firstBad = i;
                }
                Check(rows + 30 >= run.size() && rowEq == rows, std::string("2 woman picks = utpp_motion_replay csv rows ") + f.name,
                      std::to_string(rowEq) + "/" + std::to_string(rows) + " of " + std::to_string(run.size()) + " frames" +
                          (firstBad != SIZE_MAX ? " first diff frame " + std::to_string(firstBad) : std::string()));
            }
            const std::string st = motiondriver::Status();
            const double p95 = std::atof(StatusField(st, "feedP95us=").c_str());
            Check(p95 > 0.0 && p95 <= adapter::Budget::kFeedP95Us, std::string("2 ") + bn + " feed p95 within Budget::kFeedP95Us " + f.name,
                  StatusField(st, "feedP95us=") + " us");
        }
    }

    // ---------------------------------------------------------------- 3. release frames -> no v2 pose, v1 behaviour
    {
        const FeedFile& f = feeds.front();
        motiondriver::SetRigBody(0);
        ResetBase(rw);
        motiondriver::SetEnabled(true);
        motionhost::Feed(ToInput(f.fr.front()));
        motiondriver::WaitLoader();
        ResetBase(rw);
        motiondriver::SetEnabled(false);
        motiondriver::SetEnabled(true);
        // PSM release detail injected on frames 100..159 (detail 10, then 31): no v2 pose on those frames
        for (int detail : {10, 31})
        {
            FeedFile g = f;
            const uint32_t packed = static_cast<uint32_t>(detail + 1) << mm::integ::kPsmDetailShift;
            for (size_t i = 100; i < 160 && i < g.fr.size(); ++i)
                g.fr[i].psm = static_cast<int>((static_cast<uint32_t>(g.fr[i].psm) & ~mm::integ::kPsmDetailFieldMask) | packed);
            ResetBase(rw);
            motiondriver::SetEnabled(false);
            motiondriver::SetEnabled(true);
            const std::vector<FrameOut> r = RunFeed(g, kNoop);
            size_t inWin = 0, noMotion = 0, after = 0;
            for (size_t i = 100; i < 160 && i < r.size(); ++i)
            {
                ++inWin;
                noMotion += !r[i].stepped && !r[i].applied;
            }
            for (size_t i = 160; i < r.size(); ++i)
                after += r[i].stepped;
            Check(inWin == 60 && noMotion == inWin && after > 0,
                  "3 PSM release detail " + std::to_string(detail) + ": no motion pose in the window, motion again after it",
                  std::to_string(noMotion) + "/" + std::to_string(inWin) + " after " + std::to_string(after));
        }
    }

    // ---------------------------------------------------------------- 5. toggles / body switch mid-feed, stale snapshots
    {
        const FeedFile& f = feeds.front();
        motiondriver::SetRigBody(0);
        ResetBase(rw);
        motiondriver::SetEnabled(true);
        motionhost::Feed(ToInput(f.fr.front()));
        motiondriver::WaitLoader();
        ResetBase(rw);
        motiondriver::SetEnabled(false);
        motiondriver::SetEnabled(true);
        size_t offFrames = 0, offMotion = 0, onWoman = 0, onWomanMotion = 0, motionWithoutStep = 0, wrongBody = 0;
        bool off = false;
        int body = 0;
        const std::vector<FrameOut> r = RunFeed(f, [&](size_t i) {
            if (i % 37 == 0 && i > 0)
            {
                off = !off;
                motiondriver::SetEnabled(!off);
            }
            if (i % 101 == 50)
            {
                body ^= 1;
                motiondriver::SetRigBody(body); // the armed rig stays the woman one: the man body must fall back, never publish
            }
        });
        // replay the toggle schedule to classify frames
        off = false;
        body = 0;
        for (size_t i = 0; i < r.size(); ++i)
        {
            if (i % 37 == 0 && i > 0)
                off = !off;
            if (i % 101 == 50)
                body ^= 1;
            if (r[i].applied && !r[i].stepped)
                ++motionWithoutStep;
            if (body == 1 && r[i].applied)
                ++wrongBody;
            if (off)
            {
                ++offFrames;
                offMotion += r[i].applied || r[i].stepped;
            }
            else if (body == 0)
            {
                ++onWoman;
                onWomanMotion += r[i].applied;
            }
        }
        Check(motionWithoutStep == 0, "5 toggles: a motion pose only on frames where motion stepped", std::to_string(motionWithoutStep));
        Check(wrongBody == 0, "5 body switch to man on the woman rig: never a motion pose (fallback)", std::to_string(wrongBody));
        Check(offFrames > 0 && offMotion == 0, "5 toggled off: no motion pose", std::to_string(offMotion) + "/" + std::to_string(offFrames));
        Check(onWomanMotion > 0, "5 toggled on with the woman body: motion poses again",
              std::to_string(onWomanMotion) + "/" + std::to_string(onWoman) + " (frames right after a toggle / switch may still be loading)");
        // stale: no feed for longer than kDetourStaleSeconds -> the FK consumer rejects the last snapshot by age
        motiondriver::SetRigBody(0);
        ResetBase(rw);
        motiondriver::SetEnabled(false);
        motiondriver::SetEnabled(true);
        for (size_t i = 0; i < 120 && i < f.fr.size(); ++i)
            motionhost::Feed(ToInput(f.fr[i]));
        motiondriver::SetEnabled(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(motionhost::kDetourStaleSeconds * 1000.0f) + 100));
        const bool rd = ReadPublished();
        const double age = rd ? AgeSeconds(g_read.publishTicks) : 0.0;
        Check(rd && age > motionhost::kDetourStaleSeconds, "5 no feed: last snapshot older than kDetourStaleSeconds (FK detour skips it)",
              std::to_string(age) + " s");
        g_log = HookLog{};
        motionhost::Tick(); // a republish (hold) after the toggle off: v1 pose, fresh
        const bool rd2 = ReadPublished();
        Check(!g_log.applyResult && rd2 && (g_read.weight <= 0.0f || AgeSeconds(g_read.publishTicks) <= motionhost::kDetourStaleSeconds),
              "5 Tick after toggle off: no motion pose re-applied, any visible snapshot is fresh",
              "weight " + std::to_string(rd2 ? g_read.weight : -1.0f));
    }

    // ---------------------------------------------------------------- 7. review majors 1 / 2: v1 matcher / steering around a v2 span
    {
        motiondriver::SetRigBody(0);
        motiondriver::SetOverlayInputs(0, 0, false, false);
        auto warmMotion = [&](const motionhost::FeedInput& aIn) {
            ResetBase(rw);
            motiondriver::SetEnabled(true);
            motionhost::Feed(aIn);
            motiondriver::WaitLoader();
            ResetBase(rw);
            motiondriver::SetEnabled(false);
        };
        // 7a major 1: idle_then_start_sideways (idle 0..3 s, start right 3..6 s): v1 alone until feed 150 (idle), v2 owns feeds
        // 150..299 (the start / run), toggle off before feed 300 = handback while running
        const FeedFile* fs = &feeds.front();
        for (const FeedFile& f : feeds)
            if (f.name.find("idle_then_start") != std::string::npos)
                fs = &f;
        if (fs->fr.size() > 320)
        {
            warmMotion(ToInput(fs->fr.front()));
            motionhost::MotionSeamProbe pStale{}, pLast{}, pBack{};
            size_t owned = 0;
            for (size_t i = 0; i <= 300; ++i)
            {
                if (i == 150)
                    motiondriver::SetEnabled(true);
                if (i == 300)
                    motiondriver::SetEnabled(false);
                motionhost::Feed(ToInput(fs->fr[i]));
                const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
                if (i == 149)
                    pStale = p;
                if (i >= 150 && i < 300)
                {
                    owned += p.motionOwned && p.updates == pStale.updates ? 1 : 0;
                    pLast = p;
                }
                if (i == 300)
                    pBack = p;
            }
            char d[320];
            std::snprintf(d, sizeof d, "stale clip %d frame %d upd %llu | span owned %zu/150 clip %d frame %d upd %llu", pStale.clip,
                          pStale.frame, static_cast<unsigned long long>(pStale.updates), owned, pLast.clip, pLast.frame,
                          static_cast<unsigned long long>(pLast.updates));
            Check(pStale.ready && owned >= 140 && pLast.clip == pStale.clip && pLast.frame == pStale.frame,
                  "7a precondition: base matcher frozen (clip / frame / updates) while motion owns the feeds " + fs->name, d);
            std::snprintf(d, sizeof d, "handback: upd %llu searched %d offset %.4f rad / %.4f m clip %d (stale %d) handbacks %llu -> %llu",
                          static_cast<unsigned long long>(pBack.updates), pBack.searched ? 1 : 0,
                          static_cast<double>(pBack.maxOffsetAngle), static_cast<double>(pBack.maxOffsetDist), pBack.clip, pStale.clip,
                          static_cast<unsigned long long>(pStale.handbacks), static_cast<unsigned long long>(pBack.handbacks));
            Check(pBack.updates == pStale.updates + 1 && !pBack.motionOwned && pBack.handbacks == pStale.handbacks + 1,
                  "7a handback: one base update on the toggle-off feed, counted once " + fs->name, d);
            Check(pBack.searched && pBack.maxOffsetAngle == 0.0f && pBack.maxOffsetDist == 0.0f && pBack.clip != pStale.clip,
                  "7a handback: fresh base search, no stale inertialization offset, stale (idle) clip left " + fs->name, d);
        }
        else
            Check(false, "7a feed long enough", fs->name);

        // 7b major 2: synthetic start towards a target behind / beside the body (fixed-rate body yaw through BodyYawCommand) until the
        // v1 start bind engages; then v2 takes the feeds: bind sign and long-way direction must be released while v2 owns.
        const float targets[] = {180.0f, -150.0f, 150.0f, 135.0f, -135.0f, 90.0f, -90.0f};
        bool bound = false;
        float tgtUsed = 0.0f;
        int bindSign = 0;
        size_t ownedFeeds = 0, heldBind = 0, heldDir = 0;
        for (float tgt : targets)
        {
            if (bound)
                break;
            warmMotion(ToInput(fs->fr.front()));
            float yaw = 0.0f, vx = 0.0f, vy = 0.0f;
            auto synth = [&](int aI) {
                motionhost::FeedInput in;
                const float dt = 1.0f / 60.0f;
                const bool moving = aI >= 60;
                if (moving)
                {
                    const float sp = std::fmin(3.5f, std::sqrt(vx * vx + vy * vy) + 8.0f * dt);
                    vx = -std::sin(tgt * 0.0174532925f) * sp;
                    vy = std::cos(tgt * 0.0174532925f) * sp;
                }
                in.velWorld[0] = vx;
                in.velWorld[1] = vy;
                in.move[1] = moving ? 1.0f : 0.0f;
                in.playerYawDeg = yaw;
                in.locoState = mm::integ::kLocoStand;
                in.tppActive = true;
                in.dt = dt;
                in.haveTarget = moving;
                in.targetYawDeg = tgt;
                return in;
            };
            for (int i = 0; i < 160 && !bound; ++i)
            {
                float e = tgt - yaw;
                e -= 360.0f * std::floor((e + 180.0f) / 360.0f);
                const float baseYaw = i >= 60 ? yaw + std::fmax(-540.0f / 60.0f, std::fmin(540.0f / 60.0f, e)) : yaw;
                yaw = motionhost::BodyYawCommand(yaw, baseYaw);
                motionhost::Feed(synth(i));
                const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
                if (p.bindSign != 0)
                {
                    bound = true;
                    tgtUsed = tgt;
                    bindSign = p.bindSign;
                    motiondriver::SetEnabled(true); // v2 takes the next feeds while the v1 start segment is bound
                    for (int k = 1; k <= 20; ++k)
                    {
                        e = tgt - yaw;
                        e -= 360.0f * std::floor((e + 180.0f) / 360.0f);
                        yaw = motionhost::BodyYawCommand(yaw, yaw + std::fmax(-9.0f, std::fmin(9.0f, e)));
                        motionhost::Feed(synth(i + k));
                        const motionhost::MotionSeamProbe q = motionhost::ProbeMotionSeam();
                        if (!q.motionOwned)
                            continue;
                        ++ownedFeeds;
                        heldBind += q.bindSign != 0 ? 1 : 0;
                        heldDir += q.bodyFollowDir != 0 ? 1 : 0;
                    }
                    motiondriver::SetEnabled(false);
                }
            }
        }
        Check(bound && ownedFeeds >= 15, "7b precondition: base start bind engaged, then motion owned the following feeds",
              "target " + std::to_string(tgtUsed) + " bind " + std::to_string(bindSign) + " owned " + std::to_string(ownedFeeds) + "/20");
        Check(bound && heldBind == 0 && heldDir == 0, "7b start bind / long-way start turn released while motion owns the feeds",
              "bind held on " + std::to_string(heldBind) + ", follow dir on " + std::to_string(heldDir) + " of " +
                  std::to_string(ownedFeeds) + " motion feeds");

        // 7c verifier info 1: the handback feed computes the steering exclusions before its fresh search - the turn commit, the
        // settle window and the long-way follow dir must not come from the stale pre-span clip there. Body yaw held at 0 with the
        // target ~175 deg behind (|error| >= 160 deg the whole time): v1 alone until it commits a turn clip, then v2 owns 8 feeds,
        // toggle off = handback feed.
        auto synthFixed = [&](int aI, float aTgt, int aLoco, bool aStick) {
            motionhost::FeedInput in;
            const float dt = 1.0f / 60.0f;
            const bool moving = aStick && aI >= 60;
            in.velWorld[0] = moving ? -std::sin(aTgt * 0.0174532925f) * 1.5f : 0.0f;
            in.velWorld[1] = moving ? std::cos(aTgt * 0.0174532925f) * 1.5f : 0.0f;
            in.move[1] = moving ? 1.0f : 0.0f;
            in.playerYawDeg = 0.0f;
            in.locoState = aLoco;
            in.tppActive = true;
            in.dt = dt;
            in.haveTarget = moving;
            in.targetYawDeg = aTgt;
            return in;
        };
        {
            bool committed = false, preDir = false;
            float tgtC = 0.0f;
            motionhost::MotionSeamProbe pPre{}, pBack{};
            size_t owned = 0;
            for (float tgt : {175.0f, -175.0f, 165.0f, -165.0f})
            {
                if (committed)
                    break;
                warmMotion(ToInput(fs->fr.front()));
                for (int i = 0; i < 200 && !committed; ++i)
                {
                    motionhost::Feed(synthFixed(i, tgt, mm::integ::kLocoStand, true));
                    pPre = motionhost::ProbeMotionSeam();
                    if (!pPre.turnCommitted)
                        continue;
                    committed = true;
                    tgtC = tgt;
                    preDir = pPre.bodyFollowDir != 0;
                    motiondriver::SetEnabled(true);
                    for (int k = 1; k <= 8; ++k)
                    {
                        motionhost::Feed(synthFixed(i + k, tgt, mm::integ::kLocoStand, true));
                        owned += motionhost::ProbeMotionSeam().motionOwned ? 1 : 0;
                    }
                    motiondriver::SetEnabled(false);
                    motionhost::Feed(synthFixed(i + 9, tgt, mm::integ::kLocoStand, true));
                    pBack = motionhost::ProbeMotionSeam();
                }
            }
            char d[320];
            std::snprintf(d, sizeof d, "target %.0f committed %d (follow dir %d) owned %zu/8 | handback %llu -> %llu: committed %d settling %d follow dir %d",
                          static_cast<double>(tgtC), committed ? 1 : 0, pPre.bodyFollowDir, owned,
                          static_cast<unsigned long long>(pPre.handbacks), static_cast<unsigned long long>(pBack.handbacks),
                          pBack.turnCommitted ? 1 : 0, pBack.settling ? 1 : 0, pBack.bodyFollowDir);
            Check(committed && owned == 8 && pBack.handbacks == pPre.handbacks + 1,
                  "7c precondition: base turn commit engaged with |error| >= 160 deg, motion owned 8 feeds, then one handback", d);
            Check(committed && !pBack.turnCommitted && !pBack.settling && pBack.bodyFollowDir == 0,
                  std::string("7c handback feed: no turn commit / settle window / long-way follow dir from the stale clip") +
                      (preDir ? "" : " (follow dir was 0 before the span: not exercised)"),
                  d);
        }

        // 7g merge b4 (air x v2): standing jump while v2 owns the feeds. AirState runs on every feed, so a v2 frame in the air gets
        // the air leg context (airborne, contactScale 0: no foot plants in the air) like a v1 frame; the frozen v1 matcher plays no
        // air clip; toggle off mid-air = handback: the AirState episode is reset on the next feed (Ground, lockout until landed).
        {
            auto synthAir = [](bool aAir, float aVz, int aDetail) {
                motionhost::FeedInput in;
                in.locoState = aAir ? 0 : mm::integ::kLocoStand;
                in.psmBlock = aDetail >= 0 ? (0x40u | ((uint32_t(aDetail) + 1u) << 16)) : 0u;
                in.velWorld[2] = aAir ? aVz : 0.0f;
                in.tppActive = true;
                in.dt = 1.0f / 60.0f;
                in.haveAir = true;
                in.onGround = !aAir;
                in.distToGround = -2.0f;
                return in;
            };
            warmMotion(ToInput(fs->fr.front()));
            for (int i = 0; i < 60; ++i)
                motionhost::Feed(synthAir(false, 0.0f, -1));
            motiondriver::SetEnabled(true);
            for (int i = 0; i < 20; ++i)
                motionhost::Feed(synthAir(false, 0.0f, -1));
            const motionhost::MotionSeamProbe pGround = motionhost::ProbeMotionSeam();
            size_t airMotion = 0, badScale = 0, ownedNoAir = 0, badAirSteer = 0, badBoundary = 0;
            float worstScale = 0.0f;
            float vz = 3.0f;
            for (int i = 0; i < 24; ++i)
            {
                motionhost::Feed(synthAir(true, vz, vz > 0.0f ? 18 : 14));
                vz -= (vz > 0.0f ? 19.2f : 16.0f) / 60.0f;
                const motionhost::MotionSeamProbe p = motionhost::ProbeMotionSeam();
                if (p.legMotionSet && StatusField(motionhost::Status(), "airSteerFed=") != "0")
                    ++badAirSteer;
                if (!p.legMotionSet)
                    continue;
                if (!p.airOwns)
                {
                    ++ownedNoAir;
                    continue;
                }
                ++airMotion;
                if (!p.legAirborne || p.legContactScale != 0.0f)
                {
                    ++badScale;
                    worstScale = std::fmax(worstScale, p.legContactScale);
                }
            }
            const motionhost::MotionSeamProbe pAir = motionhost::ProbeMotionSeam();
            motiondriver::SetEnabled(false);
            motionhost::Feed(synthAir(true, vz, 14)); // handback feed (v1 owns again, still in the air)
            const motionhost::MotionSeamProbe pHb = motionhost::ProbeMotionSeam();
            const bool hbSteerQuiet = StatusField(motionhost::Status(), "airSteerFed=") == "0";
            // The first v1-owned handback is still physically airborne. The reset happens on
            // the NEXT feed, so this boundary must not expose stale v1 ground contacts/replant.
            if (pHb.legMotionSet || !pHb.legAirborne || pHb.legContactScale != 0.0f)
                ++badBoundary;
            vz -= 16.0f / 60.0f;
            motionhost::Feed(synthAir(true, vz, 14));
            const motionhost::MotionSeamProbe pAfter = motionhost::ProbeMotionSeam();
            if (pAfter.legMotionSet || !pAfter.legAirborne || pAfter.legContactScale != 0.0f)
                ++badBoundary;
            for (int i = 0; i < 30; ++i)
                motionhost::Feed(synthAir(false, 0.0f, -1));
            char d[600];
            std::snprintf(d, sizeof d,
                          "motion air feeds %zu (motion owned without air ownership %zu), bad %zu (worst contactScale %.2f), air clips %llu -> %llu | "
                          "handback %llu -> %llu, phase %d -> %d, resets %u -> %u, contactScale after %.2f, leg airborne %d, motion bad steer %zu, hb steer quiet %d",
                          airMotion, ownedNoAir, badScale, static_cast<double>(worstScale), static_cast<unsigned long long>(pGround.airPlays),
                          static_cast<unsigned long long>(pAir.airPlays), static_cast<unsigned long long>(pAir.handbacks),
                          static_cast<unsigned long long>(pHb.handbacks), pHb.airPhase, pAfter.airPhase, pHb.airResets, pAfter.airResets,
                          static_cast<double>(pAfter.legContactScale), pAfter.legAirborne ? 1 : 0, badAirSteer,
                          hbSteerQuiet ? 1 : 0);
            Check(pGround.legMotionSet && airMotion >= 10, "7g precondition: motion owned >= 10 feeds of a standing jump with the air area owning them", d);
            Check(airMotion >= 10 && badScale == 0, "7g motion frames in the air: LegContext airborne, contactScale 0 (no foot plants)", d);
            Check(airMotion >= 10 && pAir.airPlays == pGround.airPlays, "7g no base air clip forced on the frozen base matcher while motion owns", d);
            Check(pHb.handbacks == pAir.handbacks + 1 && pAfter.airResets == pHb.airResets + 1 && pAfter.airPhase == 0,
                  "7g handback mid-air: AirState episode reset on the next feed (Ground)", d);
            Check(airMotion >= 10 && badAirSteer == 0 && hbSteerQuiet,
                  "7g v2-owned air and first v1 handback never arm v1 air steering", d);
            Check(airMotion >= 10 && badBoundary == 0 && hbSteerQuiet,
                  "7g boundary: first v2->v1 handback and next airborne reset keep zero contacts, no stale ground replant or air yaw", d);
            Check(pAfter.airResets == pHb.airResets + 1 && pAfter.legAirborne && pAfter.legContactScale == 0.0f,
                  "7g reset mid-air: no FootLocker contact-scale replant before confirmed landing", d);
        }
        // 8 UTPP-MOTION SEAM trav: synthetic game climb / vault starts through the real MMDriver path (Feed with a PSM
        // detail 8 / 9 span, event through motiondriver::FeedTraversal as the UTPPFeedTraversal native). The obstacle is chosen offline
        // with motiontrav::PlanTraversal on the same data (table + the .utpptrav montages that are clips of the woman v2 db): the first
        // Ok climb of a grid and the first non-Ok vault. Checks: 8a plan accepted, v2 owns every detail-8 feed until the handback,
        // the published pose is the montage clip (database clip of the montage sequence at the driver's pose time) once the entry
        // blend is over; 8b FrontLedge contact of the plan at the game's hand point (independent: the event's hand); 8c warped
        // root steps <= 0.30 m per feed and the root ends at HorizontalDestination = topPoint + fwd * 0.3 (LT:2727, tweak 0.3);
        // 8d handback: JustTraversed, MM owns the next feed, published per-feed angular step at the handback within the montage's
        // own per-feed steps (+ margin); 8e rejected plan -> no v2 pose on the detail-9 feeds (skip trav-yield), v2 back after.
        {
            namespace motiontrav = utpp::mm::traversal::v2;
            namespace trv = utpp::mm::traversal;
            const std::string tablePath = data + "\\traversal\\traversal_table.json";
            std::string tj, err;
            {
                std::ifstream tf(tablePath, std::ios::binary);
                std::stringstream ss;
                ss << tf.rdbuf();
                tj = ss.str();
            }
            trv::DecisionTable table;
            trv::TraversalFile full, play;
            std::vector<int32_t> playClip;
            const bool dataOk = table.Load(tj, err) && trv::LoadTraversalFile(DataPaths(data, "woman").traversal, full, err);
            // the driver's playable set: montages whose sequence is a v2 db clip, else those with version 2 poses
            for (const auto& m : full.montages)
            {
                int32_t clip = -1;
                for (uint32_t c = 0; c < dbw->Header().numClips; ++c)
                    if (dbw->ClipName(c) == m.sequence && dbw->Clip(c).numFrames > 1)
                        clip = static_cast<int32_t>(c);
                if (clip < 0 && !m.HasPose())
                    continue;
                play.montages.push_back(m);
                playClip.push_back(clip);
            }
            play.version = full.version;
            play.poseBones = full.poseBones;
            Check(dataOk && full.version == 2 && play.montages.size() == 34,
                  "8 traversal data (table + woman montages: motion db clips or version 2 poses, all 34)",
                  err + " playable " + std::to_string(play.montages.size()));
            const trv::V3 start{0.0, 0.0, 0.0};
            auto obstacle = [&](motiontrav::GameAction aA, double aDist, double aH, double aExt) {
                motiontrav::GameObstacle o;
                o.action = aA;
                o.topPoint = {0.0, aDist, aH};
                o.collisionNormal = {0.0, -1.0, 0.0};
                o.topExtent = aExt;
                o.obstacleEnd = {0.0, aDist + aExt, aH};
                o.behindPoint = {0.0, aDist + aExt + 0.3, 0.0};
                o.inputDirection = {0.0, 1.0, 0.0};
                o.hasHands = aA == motiontrav::GameAction::Climb && aH > 1.2;
                o.leftHand = {-0.25, aDist, aH};
                o.rightHand = {0.25, aDist, aH};
                return o;
            };
            motiontrav::CapsuleState cap0;
            cap0.pos = start;
            cap0.yaw = 0.0;
            bool foundOk = false, foundBad = false;
            motiontrav::GameObstacle okO, badO;
            double okSpeed = 0.0, badSpeed = 0.0;
            for (double sp : {1.5, 0.0, 3.0})
                for (double h = 0.9; h <= 2.01 && !foundOk; h += 0.1)
                    for (double d = 0.4; d <= 1.21 && !foundOk; d += 0.1)
                    {
                        motiontrav::CapsuleState c = cap0;
                        c.speed = sp;
                        const motiontrav::GameObstacle o = obstacle(motiontrav::GameAction::Climb, d, h, 0.8);
                        if (motiontrav::PlanTraversal(table, play, o, c, motiontrav::GameTweaks{}, motiontrav::Config{}).result == motiontrav::StartResult::Ok)
                        {
                            foundOk = true;
                            okO = o;
                            okSpeed = sp;
                        }
                    }
            bool badRejected = false; // prefer a gate rejection (Rejected) over other non-Ok results
            for (double h = 0.6; h <= 1.61 && !badRejected; h += 0.1)
                for (double d = 0.4; d <= 1.21 && !badRejected; d += 0.2)
                {
                    motiontrav::CapsuleState c = cap0;
                    c.speed = 3.5;
                    const motiontrav::GameObstacle o = obstacle(motiontrav::GameAction::Vault, d, h, 0.4);
                    const motiontrav::Plan p = motiontrav::PlanTraversal(table, play, o, c, motiontrav::GameTweaks{}, motiontrav::Config{});
                    if (p.result == motiontrav::StartResult::Rejected || (p.result != motiontrav::StartResult::Ok && !foundBad))
                    {
                        foundBad = true;
                        badRejected = p.result == motiontrav::StartResult::Rejected;
                        badO = o;
                        badSpeed = 3.5;
                    }
                }
            Check(foundOk, "8 precondition: an accepted climb plan exists on the woman data (offline grid)");
            auto toEvent = [](const motiontrav::GameObstacle& o, const trv::V3& aPos, double aSpeed) {
                motiondriver::TraversalEvent e;
                e.action = static_cast<int32_t>(o.action);
                auto put = [](float* d, const trv::V3& v) {
                    d[0] = static_cast<float>(v.x);
                    d[1] = static_cast<float>(v.y);
                    d[2] = static_cast<float>(v.z);
                };
                put(e.topPoint, o.topPoint);
                put(e.collisionNormal, o.collisionNormal);
                e.topExtent = static_cast<float>(o.topExtent);
                put(e.behindPoint, o.behindPoint);
                put(e.obstacleEnd, o.obstacleEnd);
                put(e.inputDirection, o.inputDirection);
                e.hasHands = o.hasHands;
                put(e.leftHand, o.leftHand);
                put(e.rightHand, o.rightHand);
                put(e.playerPos, aPos);
                e.ownerForward[0] = 0.0f;
                e.ownerForward[1] = 1.0f;
                e.ownerForward[2] = 0.0f;
                e.speed = static_cast<float>(aSpeed);
                e.climbSpeedModifier = 1.0f;
                return e;
            };
            const float dt = 1.0f / 60.0f;
            float capPos[3] = {0, 0, 0};
            // The game's feed (main 57968d8): every feed is UTPPFeedFromLocomotion of the active PSM state - the walk-in from
            // the stand state (UTPPLocoStateId "stand" = 1), the climb from the ClimbEvents.OnUpdate wrap and the vault from the
            // LocomotionEventsTransition wrap (state names "climb" / "vault" -> UTPPLocoStateId 0 = kLocoOther), each right after
            // UTPPFeedLocomotionExtra (entity position) and UTPPFeedAir (IsOnGround false while climbing / vaulting, not probed
            // -2; Main.cpp forwards it with motiondriver::FeedAirFromInput before motionhost::Feed). The event itself is UTPPFeedTraversal
            // from the ClimbEvents / VaultEvents.OnEnter wrap, before the first detail-8 / 9 feed.
            float feedYawDeg = 0.0f;
            auto feedAt = [&](int aDetail, const float* aVel) {
                motionhost::FeedInput in;
                for (int k = 0; k < 3; ++k)
                {
                    in.velWorld[k] = aVel[k];
                    capPos[k] += aVel[k] * dt;
                }
                in.move[1] = aVel[1] > 0.1f ? 1.0f : 0.0f;
                in.playerYawDeg = feedYawDeg;
                in.cameraYawDeg = feedYawDeg;
                const bool trav = aDetail == 8 || aDetail == 9;
                in.locoState = trav ? mm::integ::kLocoOther : mm::integ::kLocoStand;
                in.tppActive = true;
                in.dt = dt;
                in.haveExtra = true;
                std::memcpy(in.capsulePos, capPos, sizeof capPos);
                in.haveAir = true;
                in.onGround = !trav;
                in.distToGround = -2.0f;
                in.psmFall = 0;
                if (aDetail >= 0)
                    in.psmBlock = mm::integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << mm::integ::kPsmDetailShift);
                g_log = HookLog{};
                motiondriver::FeedAirFromInput(in);
                motionhost::Feed(in);
                return g_log.stepResult;
            };
            auto clipPoseAngle = [&](const motiondriver::TraversalProbe& aP, float aT, const mm::integ::PoseSnapshot& s) {
                // mean angle over bones 2.. between the published snapshot and the montage pose (db clip or version 2 poses,
                // same db bone order); the published pose went through the overlay layering and the v1 leg tail, so it is
                // compared relatively (8a)
                std::vector<BoneTransform> ref;
                if (!TravRefPose(*dbw, full, aP.clip, aP.montage, aT, ref))
                    return 999.0f;
                return MeanAngleToRef(ref, s);
            };
            if (foundOk)
            {
                warmMotion(ToInput(fs->fr.front()));
                motiondriver::SetRigBody(0);
                motiondriver::SetEnabled(true);
                const float walk[3] = {0.0f, static_cast<float>(okSpeed), 0.0f}; // approach at the plan's speed
                const float still[3] = {0.0f, 0.0f, 0.0f};
                int mmOwned = 0;
                for (int i = 0; i < 90; ++i)
                    mmOwned += feedAt(-1, walk) ? 1 : 0;
                const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                motiontrav::GameObstacle o = okO;
                o.topPoint = o.topPoint + p0;
                o.obstacleEnd = o.obstacleEnd + p0;
                o.behindPoint = o.behindPoint + p0;
                o.leftHand = o.leftHand + p0;
                o.rightHand = o.rightHand + p0;
                const motiondriver::TraversalProbe pr0 = motiondriver::ProbeTraversal();
                const bool ownsBefore = motiondriver::OwnsTraversal();
                const std::string baseSt0 = motionhost::Status(); // 8k: v1 climb release counters before the accepted climb
                const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, okSpeed));
                const bool ownsAtEvent = motiondriver::OwnsTraversal(); // before any Feed: the v1 gate of the next feed sees it
                std::vector<bool> owns;
                // independent game values: climb class / duration (tweaks 2.31, LT:2730-2751, [P] sum) and HorizontalDestination
                const double hgt = o.topPoint.z - p0.z;
                const int cls = hgt > 1.6 ? 0 : (hgt > 1.2 ? 1 : 2);
                const double vD[3] = {1.0, 0.6, 0.3}, hD[3] = {0.3, 0.2, 0.1};
                const double gameT = vD[cls] + hD[cls];
                const trv::V3 dest{o.topPoint.x, o.topPoint.y + 0.3, o.topPoint.z};
                const int climbFeeds = static_cast<int>(std::ceil(gameT / dt)) + 6;
                std::vector<motiondriver::TraversalProbe> pr;
                std::vector<bool> owned;
                std::vector<mm::integ::PoseSnapshot> snaps;
                float maxPoseErr = 0.0f, sumAt = 0.0f, sumAlt = 0.0f;
                int poseChecked = 0, poseCloser = 0;
                float warpOffMax = 0.0f, warpErrMax = 0.0f, warpNoOffMax = 0.0f, handBestDf = 1e9f, handCap[3] = {0, 0, 0};
                int warpBones = 0;
                mm::integ::PoseSnapshot handSnap{};
                for (int i = 0; i < climbFeeds; ++i)
                {
                    // capsule along the game path: start -> destination over the game climb time
                    const float a = static_cast<float>(std::min(1.0, (i + 1) * dt / gameT));
                    float v[3] = {0, 0, 0};
                    for (int k = 0; k < 3; ++k)
                    {
                        const float target = static_cast<float>((k == 0 ? p0.x : k == 1 ? p0.y : p0.z) +
                                                                a * ((k == 0 ? dest.x : k == 1 ? dest.y : dest.z) -
                                                                     (k == 0 ? p0.x : k == 1 ? p0.y : p0.z)));
                        v[k] = (target - capPos[k]) / dt;
                    }
                    owned.push_back(feedAt(8, v));
                    owns.push_back(motiondriver::OwnsTraversal());
                    pr.push_back(motiondriver::ProbeTraversal());
                    const mm::integ::PoseSnapshot* s = motiondriver::LastMotionSnapshot();
                    snaps.push_back(s ? *s : mm::integ::PoseSnapshot{});
                    if (s && pr.back().active && pr.back().frames - pr0.frames > 12) // entry blend (0.2 s = 12 feeds) over
                    {
                        // the montage at the driver's pose time must match better than the same clip 0.25 s off and than the
                        // MM pose published just before the traversal
                        const float tc = pr.back().poseTime;
                        const float eAt = clipPoseAngle(pr.back(), tc, *s);
                        const float eAlt = std::min(clipPoseAngle(pr.back(), std::max(0.0f, tc - 0.25f), *s),
                                                    clipPoseAngle(pr.back(), tc + 0.25f, *s));
                        maxPoseErr = std::max(maxPoseErr, eAt);
                        sumAt += eAt;
                        sumAlt += eAlt;
                        poseCloser += eAt < eAlt ? 1 : 0;
                        ++poseChecked;
                        // 8f warp root on the PUBLISHED bones: root children = montage clip at the pose time + (plan root -
                        // capsule) (entity yaw 0 here, so the character frame = world); the offset is the driver probe, the
                        // clip pose and the published snapshot are independent of it
                        std::vector<BoneTransform> ref;
                        if (TravRefPose(*dbw, full, pr.back().clip, pr.back().montage, tc, ref))
                        {
                            const float* off = pr.back().offset;
                            warpOffMax = std::max(warpOffMax, std::hypot(std::hypot(off[0], off[1]), off[2]));
                            for (uint32_t bi = 2; bi < std::min(s->numBones, dbw->NumBones()); ++bi)
                                if (dbw->BoneParent(bi) >= 0 && dbw->BoneParent(bi) < 2)
                                {
                                    const float* pp = &s->pose[size_t(bi) * 7];
                                    const float w = std::hypot(std::hypot(pp[0] - (ref[bi].p.x + off[0]), pp[1] - (ref[bi].p.y + off[1])),
                                                               pp[2] - (ref[bi].p.z + off[2]));
                                    const float wo = std::hypot(std::hypot(pp[0] - ref[bi].p.x, pp[1] - ref[bi].p.y), pp[2] - ref[bi].p.z);
                                    warpErrMax = std::max(warpErrMax, w);
                                    warpNoOffMax = std::max(warpNoOffMax, wo);
                                    ++warpBones;
                                }
                        }
                        // 8g published hand (forward kinematics of the published snapshot, bones 0 / 1 = the entity at the
                        // capsule) at the feed closest to the FrontLedge window end frame
                        const float fps = dbw->Header().fps > 0.0f ? dbw->Header().fps : 30.0f;
                        const float df = std::fabs(tc * fps - static_cast<float>(pr.back().frontEndFrame));
                        if (pr.back().frontEndFrame >= 0 && df < handBestDf)
                        {
                            handBestDf = df;
                            handSnap = *s;
                            std::memcpy(handCap, capPos, sizeof handCap);
                        }
                    }
                }
                // after the span: detail back to stand, standing still
                for (int i = 0; i < 20; ++i)
                {
                    owned.push_back(feedAt(-1, still));
                    owns.push_back(motiondriver::OwnsTraversal());
                    pr.push_back(motiondriver::ProbeTraversal());
                    const mm::integ::PoseSnapshot* s = motiondriver::LastMotionSnapshot();
                    snaps.push_back(s ? *s : mm::integ::PoseSnapshot{});
                }
                const motiondriver::TraversalProbe& pl = pr.back();
                size_t trav = 0, hb = pr.size();
                bool ownedAll = true;
                float maxStep = 0.0f, maxMontageAng = 0.0f;
                for (size_t i = 0; i < pr.size(); ++i)
                {
                    const bool isTrav = pr[i].frames > (i ? pr[i - 1].frames : pr0.frames);
                    if (isTrav)
                    {
                        ++trav;
                        ownedAll = ownedAll && owned[i];
                        if (i > 0 && pr[i - 1].frames > pr0.frames)
                            maxStep = std::max(maxStep, std::hypot(pr[i].root[0] - pr[i - 1].root[0], pr[i].root[1] - pr[i - 1].root[1]));
                        if (i > 0 && pr[i].frames - pr0.frames > 18)
                            maxMontageAng = std::max(maxMontageAng, MaxAngleDeg(snaps[i - 1], snaps[i]));
                    }
                    if (hb == pr.size() && pr[i].justTraversed)
                        hb = i;
                }
                const size_t lastTrav = hb > 0 ? hb - 1 : 0;
                const float endErr = hb < pr.size()
                                         ? static_cast<float>(std::sqrt(std::pow(pr[lastTrav].root[0] - dest.x, 2) +
                                                                        std::pow(pr[lastTrav].root[1] - dest.y, 2) +
                                                                        std::pow(pr[lastTrav].root[2] - dest.z, 2)))
                                         : 999.0f;
                const float hbAng = hb < pr.size() && hb > 0 ? MaxAngleDeg(snaps[hb - 1], snaps[hb]) : 999.0f;
                std::string seq; // diagnostics: per-feed max angular step around the handback (worst bone)
                for (size_t i = hb >= 3 ? hb - 3 : 1; i < std::min(pr.size(), hb + 4); ++i)
                {
                    const mm::integ::PoseSnapshot& a = snaps[i - 1];
                    const mm::integ::PoseSnapshot& c = snaps[i];
                    float m = 0.0f;
                    int wb = -1;
                    for (uint32_t k = 2; k < std::min({a.numBones, c.numBones, mm::integ::kMaxMMBones}); ++k)
                    {
                        const float* q0 = &a.pose[size_t(k) * 7 + 3];
                        const float* q1 = &c.pose[size_t(k) * 7 + 3];
                        const float dd = std::fabs(q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]);
                        const float ang = 2.0f * std::acos(std::min(1.0f, dd)) * 57.2957795f;
                        if (ang > m)
                        {
                            m = ang;
                            wb = static_cast<int>(k);
                        }
                    }
                    char t[96];
                    std::snprintf(t, sizeof t, " [%zu %.1f b%d %s g%d/%d]", i, static_cast<double>(m), wb,
                                  wb >= 0 ? std::string(dbw->BoneName(static_cast<uint32_t>(wb))).c_str() : "-", a.gameIndex[wb >= 0 ? wb : 0],
                                  c.gameIndex[wb >= 0 ? wb : 0]);
                    seq += t;
                }
                std::printf("INFO 8d steps around the handback:%s\n", seq.c_str());
                const float handErr = std::hypot(std::hypot(pl.handContact[0] - pl.handTarget[0], pl.handContact[1] - pl.handTarget[1]),
                                                 pl.handContact[2] - pl.handTarget[2]);
                auto dist3 = [](const float* a, const trv::V3& b) {
                    return std::sqrt(std::pow(a[0] - b.x, 2) + std::pow(a[1] - b.y, 2) + std::pow(a[2] - b.z, 2));
                };
                // without hand data (review major 3) the root is centred on topPoint and the hand keeps the clip's lateral
                // offset: the target lies on the horizontal line through topPoint perpendicular to the facing
                // (-collisionNormal), at most 0.5 m to the side (half the arm span)
                auto offLine = [&](const float* a) {
                    const double fx = -o.collisionNormal.x, fy = -o.collisionNormal.y, fl = std::hypot(fx, fy);
                    const double dx = a[0] - o.topPoint.x, dy = a[1] - o.topPoint.y, dz = a[2] - o.topPoint.z;
                    const double along = (dx * fx + dy * fy) / fl, side = (dx * fy - dy * fx) / fl;
                    return std::fabs(side) <= 0.5 ? std::hypot(along, dz) : 1e9;
                };
                const double tgtErr = std::min({dist3(pl.handTarget, o.leftHand), dist3(pl.handTarget, o.rightHand),
                                                o.hasHands ? 1e9 : offLine(pl.handTarget)});
                char d[512];
                std::snprintf(d, sizeof d,
                              "mm feeds %d/90, event %d, h %.2f class %d gameT %.2f, accepted %llu result %d clip %d, trav frames %zu owned %d, "
                              "pose err %.2f deg over %d feeds, hand contact-target %.3f m target-event hand %.3f m, root step max %.3f m, "
                              "end-dest %.3f m, handback feed %zu jt %d hbAng %.2f deg montage step max %.2f deg, handbacks %llu, status %s",
                              mmOwned, fed ? 1 : 0, hgt, cls, gameT, static_cast<unsigned long long>(pl.accepted - pr0.accepted),
                              pl.lastResult, pl.clip, trav, ownedAll ? 1 : 0, static_cast<double>(maxPoseErr), poseChecked,
                              static_cast<double>(handErr), tgtErr, static_cast<double>(maxStep), static_cast<double>(endErr), hb,
                              hb < pr.size() ? 1 : 0, static_cast<double>(hbAng), static_cast<double>(maxMontageAng),
                              static_cast<unsigned long long>(pl.handbacks - pr0.handbacks), StatusField(motiondriver::Status(), "trav=").c_str());
                Check(fed && pl.accepted == pr0.accepted + 1 && pl.lastResult == 0 && trav >= 10 && ownedAll && mmOwned >= 80,
                      "8a climb: plan accepted, motion owns every montage feed", d);
                std::printf("INFO 8a pose vs montage: checked %d, closer than +-0.25 s %d, mean angle at %.2f / off %.2f deg, max mean %.2f\n",
                            poseChecked, poseCloser, static_cast<double>(poseChecked ? sumAt / poseChecked : 0.0f),
                            static_cast<double>(poseChecked ? sumAlt / poseChecked : 0.0f), static_cast<double>(maxPoseErr));
                Check(poseChecked >= 5 && poseCloser == poseChecked && maxPoseErr <= 8.0f,
                      "8a published pose follows the montage clip at the driver pose time (mean angle <= 8 deg, closer than +-0.25 s)", d);
                Check(handErr <= 0.05f && tgtErr <= 0.01, "8b FrontLedge contact at the game's hand point (plan)", d);
                Check(maxStep <= 0.30f && endErr <= 0.30f, "8c root steps <= 0.30 m, root ends at HorizontalDestination (<= 0.30 m)", d);
                Check(hb < pr.size() && owned[hb] && pl.handbacks == pr0.handbacks + 1 && hbAng <= 20.0f,
                      "8d handback: JustTraversed, MM owns the next feed, continuous (max bone step <= 20 deg)", d);
                char dw[320];
                std::snprintf(dw, sizeof dw, "root-child bones checked %d, |plan root - capsule| max %.3f m, published - (clip + offset) max %.4f m, "
                              "published - clip (no offset) max %.3f m", warpBones, static_cast<double>(warpOffMax),
                              static_cast<double>(warpErrMax), static_cast<double>(warpNoOffMax));
                Check(warpBones >= 5 && warpOffMax >= 0.20f && warpErrMax <= 0.05f,
                      "8f published root children = montage clip + (plan root - capsule) (warp root on screen, <= 0.05 m, offset >= 0.20 m)", dw);
                // 8g: hand bone of the FrontLedge window of the played montage, forward kinematics on the published pose
                std::string handBone;
                if (const trv::TravMontage* pm8 = full.Find(pl.montage))
                    for (const auto& w : pm8->windows)
                        if (w.target == "FrontLedge" && handBone.empty())
                            handBone = w.contactBone;
                int hb2 = -1;
                for (uint32_t i = 0; i < dbw->NumBones() && !handBone.empty(); ++i)
                    if (dbw->BoneName(i) == handBone)
                        hb2 = static_cast<int>(i);
                float handPubErr = 999.0f, handPub[3] = {0, 0, 0};
                if (hb2 >= 2 && handBestDf < 1e9f && static_cast<uint32_t>(hb2) < handSnap.numBones)
                {
                    std::vector<std::array<float, 7>> wld(handSnap.numBones); // world p (3) + q (x y z w)
                    auto qmul = [](const float* a, const float* b, float* o) {
                        o[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
                        o[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
                        o[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
                        o[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
                    };
                    auto qrot = [](const float* q, const float* v, float* o) {
                        const float tx = 2.0f * (q[1] * v[2] - q[2] * v[1]), ty = 2.0f * (q[2] * v[0] - q[0] * v[2]),
                                    tz = 2.0f * (q[0] * v[1] - q[1] * v[0]);
                        o[0] = v[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
                        o[1] = v[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
                        o[2] = v[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
                    };
                    for (uint32_t i = 0; i < handSnap.numBones; ++i)
                    {
                        const int par = dbw->BoneParent(i);
                        const float* l = &handSnap.pose[size_t(i) * 7];
                        if (i < 2 || par < 2) // bones 0 / 1: the entity (identity, at the capsule)
                        {
                            const float id[7] = {0, 0, 0, 0, 0, 0, 1};
                            const float* src = i < 2 ? id : l;
                            std::copy(src, src + 7, wld[i].begin());
                            continue;
                        }
                        const float* pwp = wld[static_cast<size_t>(par)].data();
                        float r[3];
                        qrot(pwp + 3, l, r);
                        for (int k = 0; k < 3; ++k)
                            wld[i][k] = pwp[k] + r[k];
                        qmul(pwp + 3, l + 3, wld[i].data() + 3);
                    }
                    for (int k = 0; k < 3; ++k)
                        handPub[k] = handCap[k] + wld[static_cast<size_t>(hb2)][k];
                    handPubErr = std::hypot(std::hypot(handPub[0] - pl.handTarget[0], handPub[1] - pl.handTarget[1]),
                                            handPub[2] - pl.handTarget[2]);
                }
                char dh[320];
                std::snprintf(dh, sizeof dh, "bone %s (%d), frame distance %.2f, published hand (%.3f %.3f %.3f) vs game hand target (%.3f %.3f %.3f): %.3f m",
                              handBone.c_str(), hb2, static_cast<double>(handBestDf), handPub[0], handPub[1], handPub[2],
                              pl.handTarget[0], pl.handTarget[1], pl.handTarget[2], static_cast<double>(handPubErr));
                Check(handPubErr <= 0.05f, "8g published hand at the FrontLedge end frame on the game's hand point (FK, <= 0.05 m)", dh);
                // 8j task A: OwnsTraversal across the accepted climb. climbFeeds detail-8 feeds (montage + MM after the handback
                // while the detail stays 8), then 20 detail-stand feeds (episode over)
                size_t ownsSpan = 0, ownsAfter = 0;
                for (size_t i = 0; i < owns.size(); ++i)
                    (i < static_cast<size_t>(climbFeeds) ? ownsSpan : ownsAfter) += owns[i] ? 1 : 0;
                char dj[256];
                std::snprintf(dj, sizeof dj, "before %d, right after FeedTraversal %d, detail-8 feeds owned %zu/%d (montage %zu, handback feed %zu), after the episode %zu/20, montage %s from poses %d",
                              ownsBefore ? 1 : 0, ownsAtEvent ? 1 : 0, ownsSpan, climbFeeds, trav, hb, ownsAfter, pl.montage,
                              pl.fromPoseFile ? 1 : 0);
                Check(!ownsBefore && ownsAtEvent && ownsSpan == static_cast<size_t>(climbFeeds) && ownsAfter == 0,
                      "8j task A accept: OwnsTraversal false before, true from FeedTraversal (before the next Feed) through the episode, false after it", dj);
                // 8m real game feed (main 57968d8 ClimbEvents.OnUpdate wrap): the montage starts on the FIRST detail-8 feed after the
                // OnEnter event (TravStartLocked: montage frame 1 on feed 0), and OwnsTraversal is cleared on the very first feed
                // that leaves detail 8 (episode end), not later
                {
                    const size_t endFeed = static_cast<size_t>(climbFeeds);
                    const bool startFirst = !pr.empty() && pr[0].active && pr[0].frames == pr0.frames + 1 && owned[0];
                    const bool clearedAtEnd = owns.size() > endFeed && owns[endFeed - 1] && !owns[endFeed];
                    char dm[200];
                    std::snprintf(dm, sizeof dm, "feed 0: active %d montage frames +%llu motion %d; owns on the last detail-8 feed %d, on the first stand feed %d",
                                  !pr.empty() && pr[0].active ? 1 : 0,
                                  pr.empty() ? 0ull : static_cast<unsigned long long>(pr[0].frames - pr0.frames),
                                  !owned.empty() && owned[0] ? 1 : 0, owns.size() > endFeed && owns[endFeed - 1] ? 1 : 0,
                                  owns.size() > endFeed && owns[endFeed] ? 1 : 0);
                    Check(startFirst && clearedAtEnd,
                          "8m real feed: montage starts on the first detail-8 feed after OnEnter, OwnsTraversal cleared on the first feed after the episode", dm);
                }
                // 8k climbRelease x OwnsTraversal (real hooks): mm.climbRelease on (default), v2 owns the accepted climb -> the v1
                // gate counts every detail-8 feed as v2-owned and never releases the body to the game animation
                {
                    const std::string baseSt1 = motionhost::Status();
                    auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                    const unsigned long long ownedD = cnt(baseSt1, " travOwnedFeeds=") - cnt(baseSt0, " travOwnedFeeds=");
                    const unsigned long long relD = cnt(baseSt1, " travReleaseFeeds=") - cnt(baseSt0, " travReleaseFeeds=");
                    const unsigned long long epD = cnt(baseSt1, " travReleaseEpisodes=") - cnt(baseSt0, " travReleaseEpisodes=");
                    const bool on = baseSt1.find(" climbRelease=1 ") != std::string::npos;
                    char dk[200];
                    std::snprintf(dk, sizeof dk, "climbRelease on %d, base travOwnedFeeds +%llu / %d detail-8 feeds, travReleaseFeeds +%llu, travReleaseEpisodes +%llu",
                                  on ? 1 : 0, ownedD, climbFeeds, relD, epD);
                    Check(on && ownedD == static_cast<unsigned long long>(climbFeeds) && relD == 0 && epD == 0,
                          "8k climbRelease x OwnsTraversal accept: base gate sees motion owning every detail-8 feed, no travRelease", dk);
                }
            }
            // vehicle batch TRAV-1: the rejected vault also with mm.climbRelease off - v2 traversal active (MotionHooks::travActive) releases it to the game animation all the same (mutation UTPP_VEHEXIT_MUTATION=4: v1 holds instead)
            for (const bool climbRel : {true, false})
            {
                const std::string relTag = climbRel ? std::string() : std::string(" [mm.climbRelease off]");
                motionhost::SetClimbRelease(climbRel);
                if (foundBad)
                {
                    warmMotion(ToInput(fs->fr.front()));
                    motiondriver::SetRigBody(0);
                    motiondriver::SetEnabled(true);
                    capPos[0] = capPos[1] = capPos[2] = 0.0f;
                    const float run[3] = {0.0f, 3.5f, 0.0f};
                    for (int i = 0; i < 60; ++i)
                        feedAt(-1, run);
                    const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                    motiontrav::GameObstacle o = badO;
                    o.topPoint = o.topPoint + p0;
                    o.obstacleEnd = o.obstacleEnd + p0;
                    o.behindPoint = o.behindPoint + p0;
                    const motiondriver::TraversalProbe pr0 = motiondriver::ProbeTraversal();
                    const std::string baseSt0 = motionhost::Status(); // 8k reject: v1 climb release counters before the rejected vault
                    const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, badSpeed));
                    const bool ownsAtEvent = motiondriver::OwnsTraversal();
                    int motionFeeds = 0, travFeeds = 0, ownsFeeds = 0;
                    std::string skip;
                    for (int i = 0; i < 40; ++i)
                    {
                        motionFeeds += feedAt(9, run) ? 1 : 0;
                        ownsFeeds += motiondriver::OwnsTraversal() ? 1 : 0;
                        if (i == 20)
                            skip = StatusField(motiondriver::Status(), "skip=");
                    }
                    travFeeds = static_cast<int>(motiondriver::ProbeTraversal().frames - pr0.frames);
                    bool baseReleased = false; // 8k: v1 released the rejected vault to the game (mm.climbRelease on)
                    {
                        // 8k climbRelease x OwnsTraversal reject: v2 yields -> the v1 gate releases every detail-9 feed, one episode
                        const std::string baseSt1 = motionhost::Status();
                        auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                        const unsigned long long ownedD = cnt(baseSt1, " travOwnedFeeds=") - cnt(baseSt0, " travOwnedFeeds=");
                        const unsigned long long relD = cnt(baseSt1, " travReleaseFeeds=") - cnt(baseSt0, " travReleaseFeeds=");
                        const unsigned long long epD = cnt(baseSt1, " travReleaseEpisodes=") - cnt(baseSt0, " travReleaseEpisodes=");
                        char dk[200];
                        std::snprintf(dk, sizeof dk, "base travOwnedFeeds +%llu, travReleaseFeeds +%llu / 40 detail-9 feeds, travReleaseEpisodes +%llu",
                                      ownedD, relD, epD);
                        baseReleased = relD == 40;
                        Check(ownedD == 0 && relD == 40 && epD == 1,
                              std::string("8k climbRelease x OwnsTraversal reject: motion does not own -> base releases all 40 detail-9 feeds (one episode)") + relTag, dk);
                    }
                    int after = 0;
                    for (int i = 0; i < 20; ++i)
                        after += feedAt(-1, run) ? 1 : 0;
                    const motiondriver::TraversalProbe pl = motiondriver::ProbeTraversal();
                    char d[320];
                    std::snprintf(d, sizeof d, "event %d result %d (%s) yields %llu, v2 feeds on detail 9: %d/40, montage frames %d, skip '%s', v2 after: %d/20",
                                  fed ? 1 : 0, pl.lastResult, motiontrav::ToString(static_cast<motiontrav::StartResult>(pl.lastResult)),
                                  static_cast<unsigned long long>(pl.yields - pr0.yields), motionFeeds, travFeeds, skip.c_str(), after);
                    Check(fed && pl.lastResult != 0 && pl.yields == pr0.yields + 1 && motionFeeds == 0 && travFeeds == 0 &&
                              // the v1 climb release closes the gate on detail 9 first -> v2 skips with 'gate' before its trav-yield
                              skip == (baseReleased ? "gate" : "trav-yield"),
                          std::string("8e rejected plan: motion yields every detail-9 feed of that traversal (game / base path)") + relTag, d);
                    Check(after >= 15, std::string("8e after the traversal motion owns the feeds again") + relTag, d);
                    Check(fed && !ownsAtEvent && ownsFeeds == 0 && pl.plans == pr0.plans + 1,
                          std::string("8j task A reject: OwnsTraversal false after the rejected plan and on every detail-9 feed") + relTag,
                          "owns at event " + std::to_string(ownsAtEvent ? 1 : 0) + ", owned feeds " + std::to_string(ownsFeeds) + "/40, plans +" +
                              std::to_string(pl.plans - pr0.plans));
                }
                else
                    Check(false, std::string("8e precondition: a non-Ok vault plan exists (offline grid)") + relTag);
                motionhost::SetClimbRelease(true);
            }
            // 8v vehicle batch (TRAV-1 B / D, travActive): the obstacle kind of the event reaches the plan through FeedTraversal. An
            // accepted vault (offline grid, kind 0) fed with kind 1 (vehicle) is Unsupported('vehicle'): not owned, v2 yields every
            // detail-9 feed; the same vault with kind 0 is owned again. One "UTPP trav:" info line per event. TravActive is true
            // while v2 is enabled with traversal data and false once it is switched off.
            {
                bool vFound = false;
                motiontrav::GameObstacle okV;
                double vSpeed = 0.0;
                for (double sp : {3.5, 5.0})
                    for (double h = 0.6; h <= 1.21 && !vFound; h += 0.1)
                        for (double dd = 0.4; dd <= 1.21 && !vFound; dd += 0.2)
                        {
                            motiontrav::CapsuleState c = cap0;
                            c.speed = sp;
                            const motiontrav::GameObstacle o = obstacle(motiontrav::GameAction::Vault, dd, h, 0.4);
                            if (motiontrav::PlanTraversal(table, play, o, c, motiontrav::GameTweaks{}, motiontrav::Config{}).result == motiontrav::StartResult::Ok)
                            {
                                vFound = true;
                                okV = o;
                                vSpeed = sp;
                            }
                        }
                Check(vFound, "8v precondition: an accepted vault plan exists on the woman data (offline grid)");
                if (vFound)
                {
                    warmMotion(ToInput(fs->fr.front()));
                    motiondriver::SetRigBody(0);
                    motiondriver::SetEnabled(true);
                    g_travInfoLines.clear();
                    motiondriver::SetLogger(&TravInfoCapture);
                    const float run[3] = {0.0f, static_cast<float>(vSpeed), 0.0f};
                    auto vaultOnce = [&](int32_t aKind, bool& aOwnsAtEvent, int& aMotionFeeds, motiondriver::TraversalProbe& aPr) {
                        capPos[0] = capPos[1] = capPos[2] = 0.0f;
                        for (int i = 0; i < 60; ++i)
                            feedAt(-1, run);
                        const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                        motiontrav::GameObstacle o = okV;
                        o.topPoint = o.topPoint + p0;
                        o.obstacleEnd = o.obstacleEnd + p0;
                        o.behindPoint = o.behindPoint + p0;
                        motiondriver::TraversalEvent ev = toEvent(o, p0, vSpeed);
                        ev.obstacleKind = aKind;
                        const bool fed = motiondriver::FeedTraversal(ev);
                        aOwnsAtEvent = motiondriver::OwnsTraversal();
                        aPr = motiondriver::ProbeTraversal();
                        aMotionFeeds = 0;
                        for (int i = 0; i < 40; ++i)
                            aMotionFeeds += feedAt(9, run) ? 1 : 0;
                        for (int i = 0; i < 20; ++i)
                            feedAt(-1, run);
                        return fed;
                    };
                    const bool activeOn = motiondriver::TravActive();
                    const motiondriver::TraversalProbe q0 = motiondriver::ProbeTraversal();
                    bool ownsVeh = true, ownsStat = false;
                    int motionVeh = -1, motionStat = -1;
                    motiondriver::TraversalProbe pv, ps;
                    const bool fedVeh = vaultOnce(motiondriver::kObstacleVehicle, ownsVeh, motionVeh, pv);
                    const size_t linesVeh = g_travInfoLines.size();
                    const std::string lineVeh = linesVeh ? g_travInfoLines.back() : std::string("-");
                    const bool fedStat = vaultOnce(motiondriver::kObstacleStatic, ownsStat, motionStat, ps);
                    const std::string lineStat = g_travInfoLines.size() > linesVeh ? g_travInfoLines.back() : std::string("-");
                    motiondriver::SetLogger(nullptr);
                    char d[900];
                    std::snprintf(d, sizeof d, "vehicle: fed %d result %s(%s) kind %d owns %d v2 detail-9 feeds %d/40 | static: fed %d result %s(%s) owns %d v2 feeds %d/40 | lines %zu: '%s' / '%s'",
                                  fedVeh ? 1 : 0, motiontrav::ToString(static_cast<motiontrav::StartResult>(pv.lastResult)), pv.reason, pv.kind, ownsVeh ? 1 : 0, motionVeh,
                                  fedStat ? 1 : 0, motiontrav::ToString(static_cast<motiontrav::StartResult>(ps.lastResult)), ps.reason, ownsStat ? 1 : 0, motionStat,
                                  g_travInfoLines.size(), lineVeh.c_str(), lineStat.c_str());
                    Check(fedVeh && pv.lastResult == static_cast<int32_t>(motiontrav::StartResult::Unsupported) && std::string(pv.reason) == "vehicle" &&
                              pv.kind == 1 && !ownsVeh && motionVeh == 0,
                          "8v vehicle kind: a vault over a vehicle (kind 1) is Unsupported(vehicle), not owned, motion yields every detail-9 feed", d);
                    Check(fedStat && ps.lastResult == static_cast<int32_t>(motiontrav::StartResult::Ok) && std::string(ps.reason).empty() && ps.kind == 0 &&
                              ownsStat && motionStat > 0,
                          "8v vehicle kind control: the same vault over static geometry (kind 0) is an owned Ok plan again", d);
                    Check(g_travInfoLines.size() == 2 && ps.infoLines == q0.infoLines + 2 &&
                              lineVeh.find(" action=9 ") != std::string::npos && lineVeh.find(" kind=1 ") != std::string::npos &&
                              lineVeh.find(" result=Unsupported(vehicle) ") != std::string::npos && lineStat.find(" kind=0 ") != std::string::npos &&
                              lineStat.find(" decision=Hurdle ") != std::string::npos && lineStat.find(" result=Ok(-) montage=") != std::string::npos,
                          "8v TRAV-1 D: one always-on 'UTPP trav:' info line per traversal event (action / kind / decision / result / montage)", d);
                    {
                        // TRAV-1 A + B end to end: the vault over a vehicle with mm.climbRelease off (the saved value) - v2 does not
                        // own it and v2 traversal is active, so the v1 gate releases all 40 detail-9 feeds to the game's vault animation
                        motionhost::SetClimbRelease(false);
                        const std::string s0 = motionhost::Status();
                        bool ownsOff = true;
                        int motionOff = -1;
                        motiondriver::TraversalProbe po;
                        const bool fedOff = vaultOnce(motiondriver::kObstacleVehicle, ownsOff, motionOff, po);
                        const std::string s1 = motionhost::Status();
                        motionhost::SetClimbRelease(true);
                        auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                        const unsigned long long relD = cnt(s1, " travReleaseFeeds=") - cnt(s0, " travReleaseFeeds=");
                        const unsigned long long epD = cnt(s1, " travReleaseEpisodes=") - cnt(s0, " travReleaseEpisodes=");
                        const unsigned long long ownedD = cnt(s1, " travOwnedFeeds=") - cnt(s0, " travOwnedFeeds=");
                        char dv[220];
                        std::snprintf(dv, sizeof dv, "fed %d result %s(%s) owns %d v2 feeds %d/40, v1 travReleaseFeeds +%llu episodes +%llu owned +%llu",
                                      fedOff ? 1 : 0, motiontrav::ToString(static_cast<motiontrav::StartResult>(po.lastResult)), po.reason, ownsOff ? 1 : 0, motionOff,
                                      relD, epD, ownedD);
                        Check(fedOff && std::string(po.reason) == "vehicle" && !ownsOff && motionOff == 0 && relD == 40 && epD == 1 && ownedD == 0,
                              "8v vehicle vault with mm.climbRelease off: base releases all 40 detail-9 feeds to the game vault (one episode)", dv);
                    }
                    motiondriver::SetEnabled(false);
                    const bool activeOff = motiondriver::TravActive();
                    motiondriver::SetEnabled(true);
                    Check(activeOn && !activeOff, "8v travActive: true with motion enabled and the woman traversal data, false with mm.motionDriver off",
                          "on " + std::to_string(activeOn ? 1 : 0) + " off " + std::to_string(activeOff ? 1 : 0));
                }
            }
            // 8h stale event: the accepted climb event, then 0.6 s (> kTravEventMaxAge) of feeds on which v2 is skipped early
            // (PSM detail 10 = psm-release), then detail 8: the event has aged out -> no plan, no montage frame
            if (foundOk)
            {
                warmMotion(ToInput(fs->fr.front()));
                motiondriver::SetRigBody(0);
                motiondriver::SetEnabled(true);
                capPos[0] = capPos[1] = capPos[2] = 0.0f;
                const float walk[3] = {0.0f, static_cast<float>(okSpeed), 0.0f};
                const float still[3] = {0.0f, 0.0f, 0.0f};
                for (int i = 0; i < 60; ++i)
                    feedAt(-1, walk);
                const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                motiontrav::GameObstacle o = okO;
                o.topPoint = o.topPoint + p0;
                o.obstacleEnd = o.obstacleEnd + p0;
                o.behindPoint = o.behindPoint + p0;
                o.leftHand = o.leftHand + p0;
                o.rightHand = o.rightHand + p0;
                const motiondriver::TraversalProbe pr0 = motiondriver::ProbeTraversal();
                const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, okSpeed));
                const bool ownsAtEvent = motiondriver::OwnsTraversal();
                int relMotion = 0;
                std::string skip;
                for (int i = 0; i < 36; ++i) // 36 / 60 s = 0.6 s
                {
                    relMotion += feedAt(10, still) ? 1 : 0;
                    if (i == 0)
                        skip = StatusField(motiondriver::Status(), "skip=");
                }
                const bool ownsAged = motiondriver::OwnsTraversal();
                int climbMotion = 0, climbOwns = 0;
                for (int i = 0; i < 20; ++i)
                {
                    climbMotion += feedAt(8, still) ? 1 : 0;
                    climbOwns += motiondriver::OwnsTraversal() ? 1 : 0;
                }
                const motiondriver::TraversalProbe pl = motiondriver::ProbeTraversal();
                for (int i = 0; i < 10; ++i)
                    feedAt(-1, still);
                char d[320];
                std::snprintf(d, sizeof d, "event %d, v2 on detail-10 feeds %d/36 (skip '%s'), plans %llu accepted %llu, montage frames %llu, v2 on detail 8 %d/20, owns at event %d / after 0.6 s %d / on detail 8 %d/20",
                              fed ? 1 : 0, relMotion, skip.c_str(), static_cast<unsigned long long>(pl.plans - pr0.plans),
                              static_cast<unsigned long long>(pl.accepted - pr0.accepted),
                              static_cast<unsigned long long>(pl.frames - pr0.frames), climbMotion, ownsAtEvent ? 1 : 0, ownsAged ? 1 : 0,
                              climbOwns);
                // the plan is made at the event (task A: OwnsTraversal for the next v1 Feed); it then ages on the skipped Steps
                Check(fed && relMotion == 0 && skip == "psm-release" && pl.plans == pr0.plans + 1 && pl.accepted == pr0.accepted + 1 &&
                          pl.frames == pr0.frames,
                      "8h an accepted plan ages on skipped Steps: stale after kTravEventMaxAge, never played later", d);
                Check(ownsAtEvent && !ownsAged && climbOwns == 0,
                      "8j task A stale: OwnsTraversal true at the event, false once the plan aged out (and on the later detail-8 feeds)", d);
            }
            // 8l next, OwnsTraversal contract (main dc221fa) through the real v1 gate: the accepted climb event arrives
            // while v2 skips on an overlay release (deferred, not planned in FeedTraversal; was the removed SetAirOwns). The v1
            // climb release reads OwnsTraversal before v2's Step: it must already be true, so v1 does not release the first
            // detail-8 feed (a release closes the gate, v2 skips 'gate' and never plans); v2 plans at that Step and plays it.
            // vehicle batch TRAV-1: 8l again with mm.climbRelease off (v2 traversal active: the same contract, nothing released before the plan)
            for (const bool climbRel : {true, false})
            {
                const std::string relTag8l = climbRel ? std::string() : std::string(" [mm.climbRelease off]");
                motionhost::SetClimbRelease(climbRel);
                if (foundOk)
                {
                    warmMotion(ToInput(fs->fr.front()));
                    motiondriver::SetRigBody(0);
                    motiondriver::SetEnabled(true);
                    capPos[0] = capPos[1] = capPos[2] = 0.0f;
                    const float walk[3] = {0.0f, static_cast<float>(okSpeed), 0.0f};
                    const float still[3] = {0.0f, 0.0f, 0.0f};
                    for (int i = 0; i < 60; ++i)
                        feedAt(-1, walk);
                    const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                    motiontrav::GameObstacle o = okO;
                    o.topPoint = o.topPoint + p0;
                    o.obstacleEnd = o.obstacleEnd + p0;
                    o.behindPoint = o.behindPoint + p0;
                    o.leftHand = o.leftHand + p0;
                    o.rightHand = o.rightHand + p0;
                    const motiondriver::TraversalProbe pr0 = motiondriver::ProbeTraversal();
                    const std::string baseSt0 = motionhost::Status();
                    motiondriver::SetOverlayInputs(0, 0, true, false); // motionfix item 5: overlay release defers the event (SetAirOwns removed)
                    const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, okSpeed));
                    const motiondriver::TraversalProbe pr1 = motiondriver::ProbeTraversal();
                    const bool ownsAtEvent = motiondriver::OwnsTraversal(); // review minor 4: false while the air path owns the body
                    motiondriver::SetOverlayInputs(0, 0, false, false); // the release ends before the first detail-8 feed
                    const bool ownsAfterAir = motiondriver::OwnsTraversal(); // deferred event re-evaluated: v2 can plan it now
                    int motionFeeds = 0, ownsFeeds = 0;
                    for (int i = 0; i < 20; ++i)
                    {
                        motionFeeds += feedAt(8, still) ? 1 : 0;
                        ownsFeeds += motiondriver::OwnsTraversal() ? 1 : 0;
                    }
                    const motiondriver::TraversalProbe pl = motiondriver::ProbeTraversal();
                    const std::string baseSt1 = motionhost::Status();
                    for (int i = 0; i < 10; ++i)
                        feedAt(-1, still);
                    auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                    const unsigned long long ownedD = cnt(baseSt1, " travOwnedFeeds=") - cnt(baseSt0, " travOwnedFeeds=");
                    const unsigned long long relD = cnt(baseSt1, " travReleaseFeeds=") - cnt(baseSt0, " travReleaseFeeds=");
                    char d[320];
                    std::snprintf(d, sizeof d, "event %d, plans at the event +%llu, owns at the event (release) %d / after the release %d, plans +%llu accepted +%llu, montage frames +%llu, v2 feeds %d/20, owned %d/20, v1 travOwnedFeeds +%llu travReleaseFeeds +%llu",
                                  fed ? 1 : 0, static_cast<unsigned long long>(pr1.plans - pr0.plans), ownsAtEvent ? 1 : 0, ownsAfterAir ? 1 : 0,
                                  static_cast<unsigned long long>(pl.plans - pr0.plans), static_cast<unsigned long long>(pl.accepted - pr0.accepted),
                                  static_cast<unsigned long long>(pl.frames - pr0.frames), motionFeeds, ownsFeeds, ownedD, relD);
                    Check(fed && pr1.plans == pr0.plans && !ownsAtEvent && ownsAfterAir && pl.plans == pr0.plans + 1 &&
                              pl.accepted == pr0.accepted + 1 && pl.frames >= pr0.frames + 20 && motionFeeds == 20 && ownsFeeds == 20 &&
                              ownedD == 20 && relD == 0,
                          std::string("8l climbRelease x deferred plan: not owned while the overlay release holds, owned once it ends (before the first detail-8 feed), base never releases, motion plans at the first detail-8 feed and plays it") + relTag8l, d);
                }
                motionhost::SetClimbRelease(true);
            }
            // 8k accept with mm.climbRelease off (vehicle batch TRAV-1): an accepted climb stays owned by v2 on every detail-8 feed, the
            // v1 gate never releases it (v2 traversal active only releases what v2 does not own) - no flash to the game animation
            if (foundOk)
            {
                warmMotion(ToInput(fs->fr.front()));
                motiondriver::SetRigBody(0);
                motiondriver::SetEnabled(true);
                motionhost::SetClimbRelease(false);
                capPos[0] = capPos[1] = capPos[2] = 0.0f;
                const float walk[3] = {0.0f, static_cast<float>(okSpeed), 0.0f};
                const float still[3] = {0.0f, 0.0f, 0.0f};
                for (int i = 0; i < 60; ++i)
                    feedAt(-1, walk);
                const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                motiontrav::GameObstacle o = okO;
                o.topPoint = o.topPoint + p0;
                o.obstacleEnd = o.obstacleEnd + p0;
                o.behindPoint = o.behindPoint + p0;
                o.leftHand = o.leftHand + p0;
                o.rightHand = o.rightHand + p0;
                const std::string baseSt0 = motionhost::Status();
                const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, okSpeed));
                const bool ownsAtEvent = motiondriver::OwnsTraversal();
                int motionFeeds = 0, ownsFeeds = 0;
                for (int i = 0; i < 20; ++i)
                {
                    motionFeeds += feedAt(8, still) ? 1 : 0;
                    ownsFeeds += motiondriver::OwnsTraversal() ? 1 : 0;
                }
                const std::string baseSt1 = motionhost::Status();
                for (int i = 0; i < 10; ++i)
                    feedAt(-1, still);
                motionhost::SetClimbRelease(true);
                auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                const unsigned long long ownedD = cnt(baseSt1, " travOwnedFeeds=") - cnt(baseSt0, " travOwnedFeeds=");
                const unsigned long long relD = cnt(baseSt1, " travReleaseFeeds=") - cnt(baseSt0, " travReleaseFeeds=");
                const bool off = baseSt1.find(" climbRelease=0 ") != std::string::npos;
                char d[260];
                std::snprintf(d, sizeof d, "climbRelease off %d, event %d, owns at the event %d, v2 feeds %d/20, owned %d/20, v1 travOwnedFeeds +%llu travReleaseFeeds +%llu",
                              off ? 1 : 0, fed ? 1 : 0, ownsAtEvent ? 1 : 0, motionFeeds, ownsFeeds, ownedD, relD);
                Check(off && fed && ownsAtEvent && motionFeeds == 20 && ownsFeeds == 20 && ownedD == 20 && relD == 0,
                      "8k climbRelease off x OwnsTraversal accept: motion owns every detail-8 feed, base never releases (travReleaseFeeds 0)", d);
            }
            // 8n review minor 4: a climb with a melee weapon / fists / body carry / game-pose weapon (SetOverlayInputs release):
            // Step would skip 'overlay-release' on every detail-8 feed, so the event must not be owned (deferral gated like
            // Step): OwnsTraversal false at the event and on the first detail-8 feed, the v1 gate releases from that feed on
            if (foundOk)
            {
                warmMotion(ToInput(fs->fr.front()));
                motiondriver::SetRigBody(0);
                motiondriver::SetEnabled(true);
                capPos[0] = capPos[1] = capPos[2] = 0.0f;
                const float walk[3] = {0.0f, static_cast<float>(okSpeed), 0.0f};
                const float still[3] = {0.0f, 0.0f, 0.0f};
                for (int i = 0; i < 60; ++i)
                    feedAt(-1, walk);
                motiondriver::SetOverlayInputs(0, 0, true, false); // melee / fists equipped: overlay release
                feedAt(-1, still);
                const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                motiontrav::GameObstacle o = okO;
                o.topPoint = o.topPoint + p0;
                o.obstacleEnd = o.obstacleEnd + p0;
                o.behindPoint = o.behindPoint + p0;
                o.leftHand = o.leftHand + p0;
                o.rightHand = o.rightHand + p0;
                const motiondriver::TraversalProbe pr0 = motiondriver::ProbeTraversal();
                const std::string baseSt0 = motionhost::Status();
                const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, okSpeed));
                const bool ownsAtEvent = motiondriver::OwnsTraversal();
                feedAt(8, still);
                const bool ownsFirst = motiondriver::OwnsTraversal();
                const std::string baseSt1 = motionhost::Status();
                int ownsLater = 0;
                for (int i = 0; i < 10; ++i)
                {
                    feedAt(8, still);
                    ownsLater += motiondriver::OwnsTraversal() ? 1 : 0;
                }
                const motiondriver::TraversalProbe pl = motiondriver::ProbeTraversal();
                for (int i = 0; i < 5; ++i)
                    feedAt(-1, still);
                motiondriver::SetOverlayInputs(0, 0, false, false);
                auto cnt = [](const std::string& s, const char* k) { return std::strtoull(StatusField(s, k).c_str(), nullptr, 10); };
                const unsigned long long ownedD = cnt(baseSt1, " travOwnedFeeds=") - cnt(baseSt0, " travOwnedFeeds=");
                const unsigned long long relD = cnt(baseSt1, " travReleaseFeeds=") - cnt(baseSt0, " travReleaseFeeds=");
                char d[260];
                std::snprintf(d, sizeof d, "event %d, owns at the event %d / after the first detail-8 feed %d / later %d/10, v1 on the first feed: travOwnedFeeds +%llu travReleaseFeeds +%llu, montage frames +%llu",
                              fed ? 1 : 0, ownsAtEvent ? 1 : 0, ownsFirst ? 1 : 0, ownsLater, ownedD, relD,
                              static_cast<unsigned long long>(pl.frames - pr0.frames));
                Check(fed && !ownsAtEvent && !ownsFirst && ownsLater == 0 && ownedD == 0 && relD == 1 && pl.frames == pr0.frames,
                      "8n melee-equipped climb: not owned at the event nor on the first detail-8 feed, base releases it at once", d);
            }
            // Climb run through the real path with the game feed (event, detail-8 feeds with the capsule on the game path start ->
            // HorizontalDestination over the game climb time, then 20 stand feeds). Returns per feed: v2 owned, probe, snapshot,
            // capsule position. Shared by 8o / 8p.
            struct ClimbRun
            {
                bool fed = false;
                motiondriver::TraversalProbe pr0;
                std::vector<bool> owned;
                std::vector<motiondriver::TraversalProbe> pr;
                std::vector<mm::integ::PoseSnapshot> snaps;
                std::vector<std::array<float, 3>> caps;
                int climbFeeds = 0;
            };
            auto runClimb = [&](const motiontrav::GameObstacle& aO, const trv::V3& aP0, double aSpeed, const trv::V3& aDest, double aGameT,
                                int aDetail) {
                ClimbRun r;
                r.pr0 = motiondriver::ProbeTraversal();
                r.fed = motiondriver::FeedTraversal(toEvent(aO, aP0, aSpeed));
                r.climbFeeds = static_cast<int>(std::ceil(aGameT / dt)) + 6;
                const float still[3] = {0.0f, 0.0f, 0.0f};
                for (int i = 0; i < r.climbFeeds + 20; ++i)
                {
                    float v[3] = {0, 0, 0};
                    if (i < r.climbFeeds)
                    {
                        const double a = std::min(1.0, (i + 1) * dt / aGameT);
                        const double tgt[3] = {aP0.x + a * (aDest.x - aP0.x), aP0.y + a * (aDest.y - aP0.y), aP0.z + a * (aDest.z - aP0.z)};
                        for (int k = 0; k < 3; ++k)
                            v[k] = static_cast<float>((tgt[k] - capPos[k]) / dt);
                    }
                    r.owned.push_back(feedAt(i < r.climbFeeds ? aDetail : -1, i < r.climbFeeds ? v : still));
                    r.pr.push_back(motiondriver::ProbeTraversal());
                    const mm::integ::PoseSnapshot* sn = motiondriver::LastMotionSnapshot();
                    r.snaps.push_back(sn ? *sn : mm::integ::PoseSnapshot{});
                    r.caps.push_back({capPos[0], capPos[1], capPos[2]});
                }
                return r;
            };
            auto boneOf = [&](const std::string& aName) {
                for (uint32_t i = 0; i < dbw->NumBones(); ++i)
                    if (dbw->BoneName(i) == aName)
                        return static_cast<int>(i);
                return -1;
            };
            // published FK hand (world) at the montage feed closest to the FrontLedge window end frame; -1 = not found
            auto publishedFrontHand = [&](const ClimbRun& aR, float aOut[3], std::string& aBone) {
                const float fps = dbw->Header().fps > 0.0f ? dbw->Header().fps : 30.0f;
                float best = 1e9f;
                int bi = -1;
                for (size_t i = 0; i < aR.pr.size(); ++i)
                    if (aR.pr[i].active && aR.pr[i].frontEndFrame >= 0 && aR.pr[i].frames > aR.pr0.frames)
                    {
                        const float df = std::fabs(aR.pr[i].poseTime * fps - static_cast<float>(aR.pr[i].frontEndFrame));
                        if (df < best)
                        {
                            best = df;
                            bi = static_cast<int>(i);
                        }
                    }
                if (bi < 0)
                    return -1.0f;
                aBone.clear();
                if (const trv::TravMontage* pm = full.Find(aR.pr[static_cast<size_t>(bi)].montage))
                    for (const auto& w : pm->windows)
                        if (w.target == "FrontLedge" && aBone.empty())
                            aBone = w.contactBone;
                float m[3];
                if (!PublishedBoneModel(aR.snaps[static_cast<size_t>(bi)], *dbw, boneOf(aBone), m))
                    return -1.0f;
                for (int k = 0; k < 3; ++k)
                    aOut[k] = aR.caps[static_cast<size_t>(bi)][static_cast<size_t>(k)] + m[k];
                return best;
            };
            // 8o review major 2: approach off the wall normal (capsule / entity yaw 0, wall facing yaw a = 0, +-0.3, +-0.5 rad,
            // hands on the ledge 0.25 m either side). The warped plan root turns owner yaw -> facing over the warp windows; the
            // published pose must turn with it: published FK hand at the FrontLedge end frame on the game's hand point
            // (<= 0.05 m) for every offset. Climb with hand data (ledge above midThreshold, game m_shouldIkHands).
            {
                const double offs[5] = {0.0, 0.3, -0.3, 0.5, -0.5};
                auto yawObstacle = [&](double aYaw, double aDist, double aH, const trv::V3& aP0) {
                    motiontrav::GameObstacle o;
                    o.action = motiontrav::GameAction::Climb;
                    const trv::V3 face{-std::sin(aYaw), std::cos(aYaw), 0.0}, right{std::cos(aYaw), std::sin(aYaw), 0.0};
                    o.topPoint = aP0 + trv::V3{0.0, aDist, aH};
                    o.collisionNormal = face * -1.0;
                    o.topExtent = 0.8;
                    o.obstacleEnd = o.topPoint + face * 0.8;
                    o.behindPoint = o.topPoint + face * 1.1;
                    o.behindPoint.z = aP0.z;
                    o.inputDirection = face;
                    o.hasHands = true;
                    o.leftHand = o.topPoint + right * -0.25;
                    o.rightHand = o.topPoint + right * 0.25;
                    return o;
                };
                // offline: the first (speed, height, distance) whose plans are Ok for all offsets (table + gate)
                bool found = false;
                double fSp = 0, fH = 0, fD = 0;
                for (double sp : {1.5, 0.0, 3.0})
                    for (double h = 1.3; h <= 2.01 && !found; h += 0.1)
                        for (double dd = 0.4; dd <= 1.21 && !found; dd += 0.1)
                        {
                            bool all = true;
                            for (double a : offs)
                            {
                                motiontrav::CapsuleState c = cap0;
                                c.speed = sp;
                                all = all && motiontrav::PlanTraversal(table, play, yawObstacle(a, dd, h, start), c, motiontrav::GameTweaks{},
                                                                 motiontrav::Config{}).result == motiontrav::StartResult::Ok;
                            }
                            if (all)
                            {
                                found = true;
                                fSp = sp;
                                fH = h;
                                fD = dd;
                            }
                        }
                Check(found, "8o precondition: a hand-data climb accepted (table + gate) at every yaw offset 0 / +-0.3 / +-0.5 rad");
                if (found)
                {
                    std::string det;
                    float worst = 0.0f, worstHb = 0.0f;
                    int okN = 0;
                    for (double a : offs)
                    {
                        warmMotion(ToInput(fs->fr.front()));
                        motiondriver::SetRigBody(0);
                        motiondriver::SetEnabled(true);
                        capPos[0] = capPos[1] = capPos[2] = 0.0f;
                        const float walk[3] = {0.0f, static_cast<float>(fSp), 0.0f};
                        for (int i = 0; i < 60; ++i)
                            feedAt(-1, walk);
                        const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                        const motiontrav::GameObstacle o = yawObstacle(a, fD, fH, p0);
                        const double hgt = fH;
                        const int cls = hgt > 1.6 ? 0 : (hgt > 1.2 ? 1 : 2);
                        const double vD[3] = {1.0, 0.6, 0.3}, hD[3] = {0.3, 0.2, 0.1};
                        const trv::V3 dest{o.topPoint.x, o.topPoint.y + 0.3, o.topPoint.z}; // owner forward +Y * forwardStep 0.3
                        const ClimbRun r = runClimb(o, p0, fSp, dest, vD[cls] + hD[cls], 8);
                        float hand[3] = {0, 0, 0};
                        std::string bone;
                        const float df = publishedFrontHand(r, hand, bone);
                        const motiondriver::TraversalProbe& pl = r.pr.back();
                        // independent target: the event's hand of that contact bone
                        const trv::V3 want = bone == "LeftHand" ? o.leftHand : o.rightHand;
                        const float handErrO = df < 0.0f ? 999.0f
                                                    : static_cast<float>(std::sqrt(std::pow(hand[0] - want.x, 2) + std::pow(hand[1] - want.y, 2) +
                                                                                   std::pow(hand[2] - want.z, 2)));
                        float maxYaw = 0.0f;
                        for (const auto& q : r.pr)
                            if (q.active)
                                maxYaw = std::max(maxYaw, std::fabs(q.rootYaw));
                        worst = std::max(worst, handErrO);
                        // verifier round 2: the handback after the turned root is continuous too (max bone step <= 20 deg, as 8d)
                        size_t hb = r.pr.size();
                        for (size_t i = 1; i < r.pr.size() && hb == r.pr.size(); ++i)
                            if (r.pr[i].justTraversed)
                                hb = i;
                        const float hbAng = hb < r.pr.size() ? MaxAngleDeg(r.snaps[hb - 1], r.snaps[hb]) : 999.0f;
                        worstHb = std::max(worstHb, hbAng);
                        okN += r.fed && pl.accepted == r.pr0.accepted + 1 && handErrO <= 0.05f && hbAng <= 20.0f ? 1 : 0;
                        char t[260];
                        std::snprintf(t, sizeof t, " [yaw %+.1f: %s %s df %.2f hand %.3f m, max |root yaw| %.2f, handback step %.2f deg]", a, pl.montage,
                                      bone.c_str(), static_cast<double>(df), static_cast<double>(handErrO), static_cast<double>(maxYaw),
                                      static_cast<double>(hbAng));
                        det += t;
                    }
                    std::printf("INFO 8o speed %.1f h %.1f d %.1f:%s\n", fSp, fH, fD, det.c_str());
                    Check(okN == 5,
                          "8o approach off the wall normal (0 / +-0.3 / +-0.5 rad): published FK hand on the game's hand point (<= 0.05 m), continuous handback (<= 20 deg)",
                          "worst hand " + std::to_string(worst) + " m, worst handback step " + std::to_string(worstHb) + " deg" + det);
                    // 8q verifier round 2 (handback turn cut by a skip): the 0.5 rad climb, an overlay release 2 feeds after the
                    // handback for 4 feeds (the handback turn still running), then v2 again: the re-entry starts a fresh blend
                    // (inertializer reset, hand-in from the published pose), so no handback turn is applied on any feed after it;
                    // positive control: the handback feed itself applies the turn (|turn| > 0.1 rad). Mode 1 (attempt 5): the
                    // release starts on the feed right after the last montage frame, so the handback feed itself is skipped (Skip
                    // gives the episode up, handbackNext survives): the stale handback on the re-entry must not start the turn
                    // either (the published pose then is the v1 / handover pose); positive control = mode 0's turn, same setup
                    float q0Turn = 0.0f;
                    for (int mode = 0; mode < 2; ++mode)
                    {
                        const double a = 0.5;
                        warmMotion(ToInput(fs->fr.front()));
                        motiondriver::SetRigBody(0);
                        motiondriver::SetEnabled(true);
                        capPos[0] = capPos[1] = capPos[2] = 0.0f;
                        const float walk[3] = {0.0f, static_cast<float>(fSp), 0.0f};
                        for (int i = 0; i < 60; ++i)
                            feedAt(-1, walk);
                        const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                        const motiontrav::GameObstacle o = yawObstacle(a, fD, fH, p0);
                        const int cls = fH > 1.6 ? 0 : (fH > 1.2 ? 1 : 2);
                        const double vD[3] = {1.0, 0.6, 0.3}, hD[3] = {0.3, 0.2, 0.1};
                        const double gameT = vD[cls] + hD[cls];
                        const trv::V3 dest{o.topPoint.x, o.topPoint.y + 0.3, o.topPoint.z};
                        const bool fed = motiondriver::FeedTraversal(toEvent(o, p0, fSp));
                        const int climbFeeds = static_cast<int>(std::ceil(gameT / dt)) + 6;
                        const float still[3] = {0.0f, 0.0f, 0.0f};
                        int hb = -1, rel = -1, relEnd = -1, reFeeds = 0, reOwnedMotion = 0, endFeed = -1;
                        bool prevActive = false;
                        float hbTurn = 0.0f, reTurn = 0.0f;
                        const auto motionFrames = [] { // published v2 frames (MM pick or traversal frame)
                            const std::string s = StatusField(motiondriver::Status(), " frames=");
                            return s.empty() ? 0ull : std::stoull(s);
                        };
                        for (int i = 0; i < climbFeeds + 40; ++i)
                        {
                            float v[3] = {0, 0, 0};
                            if (i < climbFeeds)
                            {
                                const double u = std::min(1.0, (i + 1) * dt / gameT);
                                const double tgt[3] = {p0.x + u * (dest.x - p0.x), p0.y + u * (dest.y - p0.y), p0.z + u * (dest.z - p0.z)};
                                for (int k = 0; k < 3; ++k)
                                    v[k] = static_cast<float>((tgt[k] - capPos[k]) / dt);
                            }
                            if (rel < 0 && (mode == 0 ? hb >= 0 && i == hb + 2 : endFeed >= 0 && i == endFeed + 1))
                            {
                                rel = i;
                                // mode 1: released until the detail has left 8 (a detail-8 yield feed would consume the
                                // stale handback), so v2 re-enters on a stand feed with handbackNext still set
                                relEnd = mode == 0 ? rel + 4 : std::max(rel + 4, climbFeeds + 2);
                                (void)motiondriver::SetOverlayInputs(0, 0, true, false);
                            }
                            if (rel >= 0 && i == relEnd)
                                (void)motiondriver::SetOverlayInputs(0, 0, false, false);
                            const unsigned long long steps0 = motionFrames();
                            (void)feedAt(i < climbFeeds ? 8 : -1, i < climbFeeds ? v : still);
                            const bool stepped = motionFrames() > steps0;
                            const motiondriver::TraversalProbe q = motiondriver::ProbeTraversal();
                            if (hb < 0 && q.justTraversed)
                            {
                                hb = i;
                                hbTurn = q.handbackYaw;
                            }
                            if (endFeed < 0 && prevActive && !q.active && !q.justTraversed)
                                endFeed = i; // the last montage frame was published on this feed (handbackNext set)
                            prevActive = q.active;
                            if (rel >= 0 && i >= relEnd)
                            {
                                ++reFeeds;
                                reOwnedMotion += stepped ? 1 : 0;
                                if (stepped)
                                    reTurn = std::max(reTurn, std::fabs(q.handbackYaw));
                            }
                        }
                        char t[260];
                        std::snprintf(t, sizeof t, "mode %d: fed %d last montage feed %d, handback feed %d turn %.3f rad, release %d..%d, re-entry feeds %d (motion frames %d), max turn after %.4f rad",
                                      mode, fed ? 1 : 0, endFeed, hb, static_cast<double>(hbTurn), rel, relEnd, reFeeds, reOwnedMotion, static_cast<double>(reTurn));
                        std::printf("INFO 8q %s\n", t);
                        if (mode == 0)
                        {
                            q0Turn = hbTurn;
                            Check(fed && hb >= 0 && std::fabs(hbTurn) > 0.1f && reOwnedMotion > 0 && reTurn == 0.0f,
                                  "8q handback turn cut by a skip (overlay release during the blend) is not resumed on the re-entry", t);
                        }
                        else
                            Check(fed && endFeed >= 0 && rel == endFeed + 1 && std::fabs(q0Turn) > 0.1f && reOwnedMotion > 0 && reTurn == 0.0f,
                                  "8q handback feed itself skipped (overlay release right after the last montage frame): the re-entry starts no handback turn", t);
                    }
                }
            }
            // 8p review major 3 (+ verifier round 2 finding 3): traversals without hand data, as the game sends them (low climb:
            // m_shouldIkHands false; vault: never), through the real path, each approached 0 / +-0.3 / +-0.5 rad off the wall
            // normal (entity yaw 0, obstacle turned about the capsule start, as 8o). Per run: the played root (probe = the warp
            // root published on the root children, 8f) stays within a source-derived lateral bound of the centred line through
            // topPoint (capsule start / game destination off the line + the montage's own lateral root excursion over the played
            // frames + 2 cm); the vault root lands at the game Destination (obstacleEnd.xy, max(behindPoint.z, player z)) +
            // inputDirection * vault.forwardStep 0.5 (LT:3058-3063, <= 0.02 m); the handback is v2-owned and continuous, measured
            // directly on both rotation and position: (i) max bone step <= 20 deg (as 8d); (ii) root snap: the played root at the
            // last montage feed vs the entity at the handback feed (the MM pose is rooted at the entity, so this distance is the
            // slide the handback has to make) <= the capsule's largest per-feed step + 2 cm; (iii) the published Hips (world =
            // capsule + FK of the snapshot) per-feed horizontal step over the handback blend (kTravBlendOut 0.2 s) <= its largest
            // step while the montage played + 2 cm.
            {
                const double offs[5] = {0.0, 0.3, -0.3, 0.5, -0.5};
                auto turned = [](const motiontrav::GameObstacle& aO, double aYaw, const trv::V3& aP0) {
                    // obstacle relative to the start (origin), turned by aYaw about it, then moved to the capsule start aP0
                    motiontrav::GameObstacle o = aO;
                    auto pt = [&](const trv::V3& v) { return aP0 + trv::RotZ(v, aYaw); };
                    o.topPoint = pt(aO.topPoint);
                    o.obstacleEnd = pt(aO.obstacleEnd);
                    o.behindPoint = pt(aO.behindPoint);
                    o.leftHand = pt(aO.leftHand);
                    o.rightHand = pt(aO.rightHand);
                    o.collisionNormal = trv::RotZ(aO.collisionNormal, aYaw);
                    o.inputDirection = trv::RotZ(aO.inputDirection, aYaw);
                    return o;
                };
                // aYaw = entity (owner) yaw - wall facing: before the first warp window the played root moves along the owner
                // yaw, so it leaves the centred line by up to (clip root travel start -> first window start) * sin|aYaw|; over
                // the warp windows the root heading turns by |aYaw| while the translation warp is spread along the window, so
                // the path bows off its chord by at most (window travel / 2) * sin(|aYaw| / 2) (worst case: the whole turn at
                // the middle of the travel)
                auto lateralRun = [&](const ClimbRun& aR, const motiontrav::GameObstacle& aO, const trv::V3& aP0, const trv::V3& aDest,
                                      double aYaw, double& aBound, double& aPre, double& aWin) {
                    // facing = -collisionNormal, lateral axis perpendicular to it
                    const double fx = -aO.collisionNormal.x, fy = -aO.collisionNormal.y, fl = std::hypot(fx, fy);
                    const double lx = fy / fl, ly = -fx / fl;
                    auto lat = [&](double x, double y) { return (x - aO.topPoint.x) * lx + (y - aO.topPoint.y) * ly; };
                    double clipLat = 0.0;
                    aPre = aWin = 0.0;
                    const motiondriver::TraversalProbe& pa = aR.pr.front();
                    if (const trv::TravMontage* pm = full.Find(pa.montage))
                    {
                        const size_t s0 = static_cast<size_t>(std::max(pa.startFrame, 0));
                        for (size_t f = s0; f < pm->root.size() && f <= static_cast<size_t>(pa.endFrame); ++f)
                            clipLat = std::max(clipLat, std::fabs(trv::RotZ(pm->root[f].pos - pm->root[s0].pos, -pm->root[s0].yaw).x));
                        int w0 = pa.endFrame, w1 = static_cast<int>(s0);
                        for (const auto& w : pm->windows)
                        {
                            w0 = std::min(w0, w.startFrame);
                            w1 = std::max(w1, w.endFrame);
                        }
                        if (w0 > static_cast<int>(s0) && static_cast<size_t>(w0) < pm->root.size())
                            aPre = std::hypot(pm->root[static_cast<size_t>(w0)].pos.x - pm->root[s0].pos.x,
                                              pm->root[static_cast<size_t>(w0)].pos.y - pm->root[s0].pos.y);
                        aWin = 0.0;
                        for (size_t f = static_cast<size_t>(std::max(w0, static_cast<int>(s0))) + 1;
                             f < pm->root.size() && f <= static_cast<size_t>(std::min(w1, pa.endFrame)); ++f)
                            aWin += std::hypot(pm->root[f].pos.x - pm->root[f - 1].pos.x, pm->root[f].pos.y - pm->root[f - 1].pos.y);
                    }
                    else
                        clipLat = -1e9; // montage unknown: the bound fails the check
                    aBound = std::max(std::fabs(lat(aP0.x, aP0.y)), std::fabs(lat(aDest.x, aDest.y))) + clipLat +
                             aPre * std::fabs(std::sin(aYaw)) + 0.5 * aWin * std::sin(0.5 * std::fabs(aYaw)) + 0.02;
                    double m = 0.0;
                    for (const auto& q : aR.pr)
                        if (q.active)
                            m = std::max(m, std::fabs(lat(q.root[0], q.root[1])));
                    return m;
                };
                auto handbackOf = [&](const ClimbRun& aR, size_t& aHb, float& aAng) {
                    aHb = aR.pr.size();
                    for (size_t i = 0; i < aR.pr.size(); ++i)
                        if (aR.pr[i].justTraversed)
                        {
                            aHb = i;
                            break;
                        }
                    aAng = aHb < aR.pr.size() && aHb > 0 ? MaxAngleDeg(aR.snaps[aHb - 1], aR.snaps[aHb]) : 999.0f;
                    return aHb < aR.pr.size() && aHb > 0 && aR.owned[aHb];
                };
                const int hipsBone = boneOf("Hips");
                const size_t blendFeeds = static_cast<size_t>(std::ceil(0.2 / dt)) + 1; // kTravBlendOut 0.2 s
                // one hand-less traversal (obstacle relative to the start, see turned) through the real path; appends the detail
                auto runHandless = [&](const motiontrav::GameObstacle& aRel, double aSp, double aYaw, std::string& aDet, double& aWorstLat,
                                       double& aWorstLand, double& aWorstSnap, double& aWorstHips) {
                    const bool vault = aRel.action == motiontrav::GameAction::Vault;
                    warmMotion(ToInput(fs->fr.front()));
                    motiondriver::SetRigBody(0);
                    motiondriver::SetEnabled(true);
                    capPos[0] = capPos[1] = capPos[2] = 0.0f;
                    const float walk[3] = {0.0f, static_cast<float>(aSp), 0.0f};
                    for (int i = 0; i < 60; ++i)
                        feedAt(-1, walk);
                    const trv::V3 p0{capPos[0], capPos[1], capPos[2]};
                    const motiontrav::GameObstacle o = turned(aRel, aYaw, p0);
                    trv::V3 dest;
                    double gameT = 1.0;
                    if (vault)
                    {
                        // independent game Destination (LT:3058-3063)
                        dest = {o.obstacleEnd.x + o.inputDirection.x * 0.5, o.obstacleEnd.y + o.inputDirection.y * 0.5,
                                std::max(o.behindPoint.z, p0.z) + o.inputDirection.z * 0.5};
                        // vault: no game duration in script; the capsule follows a straight path over the montage's own time
                        motiontrav::CapsuleState c = cap0;
                        c.pos = p0;
                        c.speed = aSp;
                        const motiontrav::Plan off = motiontrav::PlanTraversal(table, play, o, c, motiontrav::GameTweaks{}, motiontrav::Config{});
                        gameT = off.montage ? (off.endFrame - off.startFrame) / off.montage->fps : 1.0;
                    }
                    else
                    {
                        const double hgt = o.topPoint.z - p0.z;
                        const int cls = hgt > 1.6 ? 0 : (hgt > 1.2 ? 1 : 2);
                        const double vD[3] = {1.0, 0.6, 0.3}, hD[3] = {0.3, 0.2, 0.1};
                        dest = {o.topPoint.x, o.topPoint.y + 0.3, o.topPoint.z}; // owner forward +Y * forwardStep 0.3
                        gameT = vD[cls] + hD[cls];
                    }
                    const ClimbRun r = runClimb(o, p0, aSp, dest, gameT, vault ? 9 : 8);
                    double latBound = 0.0, pre = 0.0, win = 0.0;
                    const double latMax = lateralRun(r, o, p0, dest, aYaw, latBound, pre, win);
                    size_t hb = 0;
                    float hbAng = 0.0f;
                    const bool hbOk = handbackOf(r, hb, hbAng);
                    const size_t lastTrav = hbOk ? hb - 1 : 0;
                    const float* lr = r.pr[lastTrav].root;
                    const double land = vault ? std::sqrt(std::pow(lr[0] - dest.x, 2) + std::pow(lr[1] - dest.y, 2) + std::pow(lr[2] - dest.z, 2))
                                              : 0.0;
                    // (ii) root snap at the handback, bound = the capsule's largest per-feed step (game path) + 2 cm
                    double capStep = 0.0;
                    for (size_t i = 1; i < r.caps.size(); ++i)
                        capStep = std::max(capStep, std::sqrt(std::pow(r.caps[i][0] - r.caps[i - 1][0], 2) + std::pow(r.caps[i][1] - r.caps[i - 1][1], 2) +
                                                              std::pow(r.caps[i][2] - r.caps[i - 1][2], 2)));
                    const double snapBound = capStep + 0.02;
                    const double snap = hbOk ? std::sqrt(std::pow(lr[0] - r.caps[hb][0], 2) + std::pow(lr[1] - r.caps[hb][1], 2) + std::pow(lr[2] - r.caps[hb][2], 2))
                                             : 999.0;
                    // (iii) published Hips world position, per-feed horizontal step: montage play vs handback blend
                    auto hipsW = [&](size_t i, double aOut[3]) {
                        float m[3];
                        if (!PublishedBoneModel(r.snaps[i], *dbw, hipsBone, m))
                            return false;
                        for (int k = 0; k < 3; ++k)
                            aOut[k] = static_cast<double>(r.caps[i][static_cast<size_t>(k)]) + m[k];
                        return true;
                    };
                    bool hipsOk = hipsBone >= 0 && hbOk;
                    double playStep = 0.0, hbStep = 0.0;
                    for (size_t i = 1; hipsOk && i < r.pr.size() && i < hb + blendFeeds; ++i)
                    {
                        const bool inPlay = i < hb && r.pr[i].active && r.pr[i - 1].active;
                        if (!inPlay && i < hb)
                            continue;
                        double h0[3], h1[3];
                        if (!hipsW(i - 1, h0) || !hipsW(i, h1))
                        {
                            hipsOk = false;
                            break;
                        }
                        const double st = std::hypot(h1[0] - h0[0], h1[1] - h0[1]);
                        (inPlay ? playStep : hbStep) = std::max(inPlay ? playStep : hbStep, st);
                    }
                    const double hipsBound = playStep + 0.02;
                    const bool ok = r.fed && r.pr.back().accepted == r.pr0.accepted + 1 && latMax <= latBound && (!vault || land <= 0.02) &&
                                    hbOk && hbAng <= 20.0f && snap <= snapBound && hipsOk && playStep > 0.0 && hbStep <= hipsBound;
                    aWorstLat = std::max(aWorstLat, latMax - latBound);
                    aWorstLand = std::max(aWorstLand, land);
                    aWorstSnap = std::max(aWorstSnap, snap - snapBound);
                    aWorstHips = std::max(aWorstHips, hbStep - hipsBound);
                    char t[560];
                    std::snprintf(t, sizeof t,
                                  " [yaw %+.1f: event %d accepted +%llu %s, |root lateral| %.3f m (bound %.3f, travel pre-window %.3f / windows %.3f)%s%.3f, handback feed %zu owned %d step %.2f deg, root snap %.3f m (bound %.3f), Hips step blend %.3f / play %.3f m%s]",
                                  aYaw, r.fed ? 1 : 0, static_cast<unsigned long long>(r.pr.back().accepted - r.pr0.accepted), r.pr.back().montage,
                                  latMax, latBound, pre, win, vault ? ", landing - Destination " : ", -", land, hb, hbOk ? 1 : 0, static_cast<double>(hbAng), snap,
                                  snapBound, hbStep, playStep, ok ? "" : " FAIL");
                    aDet += t;
                    return ok;
                };
                // (a) low climb without hand data (ledge <= 1.2 m: obstacle() leaves hasHands false), Ok at every offset
                bool cFound = false;
                motiontrav::GameObstacle cO;
                double cSp = 0.0;
                for (double sp : {okSpeed, 1.5, 0.0, 3.0})
                    for (double h = 0.9; h <= 1.21 && !cFound; h += 0.1)
                        for (double dd = 0.4; dd <= 1.21 && !cFound; dd += 0.1)
                        {
                            const motiontrav::GameObstacle o = obstacle(motiontrav::GameAction::Climb, dd, h, 0.8);
                            bool all = !o.hasHands;
                            for (double a : offs)
                            {
                                motiontrav::CapsuleState c = cap0;
                                c.speed = sp;
                                all = all && motiontrav::PlanTraversal(table, play, turned(o, a, start), c, motiontrav::GameTweaks{}, motiontrav::Config{}).result ==
                                                 motiontrav::StartResult::Ok;
                            }
                            if (all)
                            {
                                cFound = true;
                                cO = o;
                                cSp = sp;
                            }
                        }
                Check(cFound && foundOk && !okO.hasHands,
                      "8p precondition: the 8a climb has no hand data (low climb) and a hand-less low climb is accepted at every yaw offset 0 / +-0.3 / +-0.5 rad");
                if (cFound)
                {
                    std::string det;
                    double wLat = -1e9, wLand = 0.0, wSnap = -1e9, wHips = -1e9;
                    int okN = 0;
                    for (double a : offs)
                        okN += runHandless(cO, cSp, a, det, wLat, wLand, wSnap, wHips) ? 1 : 0;
                    char hd[200];
                    std::snprintf(hd, sizeof hd, "speed %.1f h %.1f d %.1f, worst margins: lateral %+.3f m, root snap %+.3f m, Hips step %+.3f m;", cSp,
                                  cO.topPoint.z, cO.topPoint.y, wLat, wSnap, wHips);
                    std::printf("INFO 8p low climb %s%s\n", hd, det.c_str());
                    Check(okN == 5,
                          "8p low climb without hand data, approach 0 / +-0.3 / +-0.5 rad: no lateral root slide beyond the source bound, v2-owned handback without bone or root snap",
                          std::string(hd) + det);
                }
                // (b) a vault without hand data accepted by the table + gate at every offset (offline grid on the same data)
                bool vFound = false;
                motiontrav::GameObstacle vO;
                double vSp = 0.0;
                for (double sp : {3.5, 5.0})
                    for (double h = 0.8; h <= 1.21 && !vFound; h += 0.1)
                        for (double dd = 0.5; dd <= 1.01 && !vFound; dd += 0.1)
                            for (double ext : {0.3, 0.1})
                            {
                                if (vFound)
                                    break;
                                motiontrav::GameObstacle o = obstacle(motiontrav::GameAction::Vault, dd, h, ext);
                                o.hasHands = false;
                                bool all = true;
                                for (double a : offs)
                                {
                                    motiontrav::CapsuleState c = cap0;
                                    c.speed = sp;
                                    all = all && motiontrav::PlanTraversal(table, play, turned(o, a, start), c, motiontrav::GameTweaks{}, motiontrav::Config{}).result ==
                                                     motiontrav::StartResult::Ok;
                                }
                                if (all)
                                {
                                    vFound = true;
                                    vO = o;
                                    vSp = sp;
                                }
                            }
                Check(vFound, "8p precondition: a vault without hand data is accepted on the woman data at every yaw offset 0 / +-0.3 / +-0.5 rad (offline grid)");
                if (vFound)
                {
                    std::string det;
                    double wLat = -1e9, wLand = 0.0, wSnap = -1e9, wHips = -1e9;
                    int okN = 0;
                    for (double a : offs)
                        okN += runHandless(vO, vSp, a, det, wLat, wLand, wSnap, wHips) ? 1 : 0;
                    char hd[220];
                    std::snprintf(hd, sizeof hd, "speed %.1f h %.1f d %.1f ext %.1f, worst: lateral margin %+.3f m, landing %.3f m, root snap margin %+.3f m, Hips step margin %+.3f m;",
                                  vSp, vO.topPoint.z, vO.topPoint.y, vO.topExtent, wLat, wLand, wSnap, wHips);
                    std::printf("INFO 8p vault %s%s\n", hd, det.c_str());
                    Check(okN == 5,
                          "8p vault without hand data, approach 0 / +-0.3 / +-0.5 rad: no lateral root slide beyond the source bound, lands at the game Destination, v2-owned handback without bone or root snap",
                          std::string(hd) + det);
                }
            }
            // 8i trav task B: every montage of the version 2 file, woman and man, through the real path. Each montage is
            // forced (TestForceTraversal: no decision table, gate off; integ target only) on an obstacle built from the montage's
            // own FrontLedge contact so the plan starts at frame 0 (obstacle at least 0.2 m above the feet: the catch montages
            // start above their ledge); the capsule follows the plan root one feed behind; 120 Hz feeds. Per montage: accepted,
            // played from the db clip or the version 2 poses (the 20 not in the db), every montage feed v2-owned with
            // OwnsTraversal true, published pose = montage pose after the entry blend (mean angle <= 8 deg), published FK hand
            // at the FrontLedge end frame on the game's hand point (<= 0.05 m), handback, OwnsTraversal false after the episode.
            for (int body = 0; body <= 1; ++body)
            {
                const char* bn = body ? "man" : "woman";
                const Rig& rig = body ? rm : rw;
                const mm::motion::MotionDatabase& db = body ? *dbm : *dbw;
                trv::TraversalFile tf;
                std::string terr;
                const bool tfOk = trv::LoadTraversalFile(DataPaths(data, bn).traversal, tf, terr);
                size_t inDb = 0;
                for (const auto& m : tf.montages)
                    for (uint32_t c = 0; c < db.Header().numClips; ++c)
                        if (db.ClipName(c) == m.sequence && db.Clip(c).numFrames > 1)
                        {
                            ++inDb;
                            break;
                        }
                const float dtI = 1.0f / 120.0f;
                float cap[3] = {0, 0, 0};
                auto feedI = [&](int aDetail, const float* aVel) {
                    motionhost::FeedInput in;
                    for (int k = 0; k < 3; ++k)
                    {
                        in.velWorld[k] = aVel[k];
                        cap[k] += aVel[k] * dtI;
                    }
                    in.move[1] = std::hypot(aVel[0], aVel[1]) > 0.1f ? 1.0f : 0.0f;
                    const bool trav = aDetail == 8 || aDetail == 9; // game feed as feedAt (climb / vault state -> loco 0, airborne)
                    in.locoState = trav ? mm::integ::kLocoOther : mm::integ::kLocoStand;
                    in.tppActive = true;
                    in.dt = dtI;
                    in.haveExtra = true;
                    std::memcpy(in.capsulePos, cap, sizeof cap);
                    in.haveAir = true;
                    in.onGround = !trav;
                    in.distToGround = -2.0f;
                    if (aDetail >= 0)
                        in.psmBlock = mm::integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << mm::integ::kPsmDetailShift);
                    g_log = HookLog{};
                    motiondriver::FeedAirFromInput(in);
                    motionhost::Feed(in);
                    return g_log.stepResult;
                };
                const float zero[3] = {0, 0, 0};
                motiondriver::SetRigBody(body);
                motiondriver::SetOverlayInputs(0, 0, false, false);
                ResetBase(rig);
                motiondriver::SetEnabled(true);
                feedI(-1, zero);
                motiondriver::WaitLoader();
                feedI(-1, zero);
                const motiondriver::TraversalProbe pl0 = motiondriver::ProbeTraversal();
                char dp[200];
                std::snprintf(dp, sizeof dp, "%s: file %d (%s) montages %zu, version %d, db clips %zu, driver playable %u (from poses %u)", bn,
                              tfOk ? 1 : 0, terr.c_str(), tf.montages.size(), tf.version, inDb, pl0.playable, pl0.playableFromPoses);
                Check(tfOk && tf.montages.size() == 34 && pl0.playable == 34 && pl0.playableFromPoses == 34 - inDb && inDb < 34,
                      std::string("8i ") + bn + ": the driver can play all 34 montages (db clip or version 2 poses)", dp);
                int nAcc = 0, nPlayed = 0, nPose = 0, nHand = 0, nHb = 0, nOwn = 0, nPoses = 0;
                float worstPose = 0.0f, worstHand = 0.0f;
                std::string bad;
                for (const trv::TravMontage& m : tf.montages)
                {
                    const bool climb = m.action == "Mantle" || m.action == "ClimbStart" ||
                                       m.name.find("_Catch_Mantle_") != std::string::npos ||
                                       m.name.find("_Catch_Cliff_") != std::string::npos;
                    int fw = -1, bw = -1;
                    for (size_t k = 0; k < m.windows.size(); ++k)
                    {
                        if (m.windows[k].target == "FrontLedge" && fw < 0)
                            fw = static_cast<int>(k);
                        if (m.windows[k].target == "BackLedge" && bw < 0)
                            bw = static_cast<int>(k);
                    }
                    if (fw < 0)
                    {
                        bad += " " + m.name + "(no FrontLedge)";
                        continue;
                    }
                    const trv::RootKey& r0 = m.root.front();
                    auto contact = [&](int aW) {
                        const trv::TravWindow& w = m.windows[static_cast<size_t>(aW)];
                        const trv::RootKey& re = m.root[static_cast<size_t>(w.endFrame)];
                        return trv::RotZ(re.pos + trv::RotZ(w.offset, re.yaw) - r0.pos, -r0.yaw); // capsule frame (yaw 0)
                    };
                    motiondriver::TestForceTraversal(m.name, false);
                    motiondriver::SetEnabled(false);
                    ResetBase(rig);
                    motiondriver::SetEnabled(true);
                    cap[0] = cap[1] = cap[2] = 0.0f;
                    for (int i = 0; i < 12; ++i)
                        feedI(-1, zero);
                    trv::V3 fc = contact(fw);
                    fc.z = std::max(fc.z, 0.2);
                    const double ext = bw >= 0 ? std::max(0.1, contact(bw).y - contact(fw).y) : 0.4;
                    motiontrav::GameObstacle o;
                    o.action = climb ? motiontrav::GameAction::Climb : motiontrav::GameAction::Vault;
                    o.topPoint = {fc.x, fc.y, fc.z};
                    o.collisionNormal = {0.0, -1.0, 0.0};
                    o.topExtent = ext;
                    o.obstacleEnd = {fc.x, fc.y + ext, fc.z};
                    o.behindPoint = {fc.x, fc.y + ext + 0.3, 0.0};
                    o.inputDirection = {0.0, 1.0, 0.0};
                    // the game's hand point = the montage's own FrontLedge contact (both hands: the window's contact bone is checked)
                    o.hasHands = true;
                    o.leftHand = o.topPoint;
                    o.rightHand = o.topPoint;
                    const trv::V3 startI{cap[0], cap[1], cap[2]};
                    const motiondriver::TraversalProbe q0 = motiondriver::ProbeTraversal();
                    const bool fed = motiondriver::FeedTraversal(toEvent(o, startI, 0.0));
                    const motiondriver::TraversalProbe q1 = motiondriver::ProbeTraversal();
                    const bool accepted = fed && q1.accepted == q0.accepted + 1 && q1.lastResult == 0 && m.name == q1.montage;
                    nAcc += accepted ? 1 : 0;
                    nPoses += accepted && q1.fromPoseFile ? 1 : 0;
                    const int detail = climb ? 8 : 9;
                    const int entryFeeds = static_cast<int>(std::ceil(0.2f / dtI)) + 2;
                    int frames = 0, framesOwned = 0, posesChecked = 0, hbFeed = -1;
                    float poseMax = 0.0f, handDf = 1e9f, handErr = 999.0f, handVec[3] = {0, 0, 0}, handOff = 0.0f, planErr = 999.0f;
                    motiondriver::TraversalProbe last = q1;
                    std::vector<BoneTransform> ref;
                    const int maxFeeds = static_cast<int>(m.frames / m.fps / 0.5 / dtI) + 40;
                    for (int i = 0; i < maxFeeds && hbFeed < 0; ++i)
                    {
                        float v[3] = {0, 0, 0};
                        if (last.frames > q1.frames) // follow the plan root one feed behind
                            for (int k = 0; k < 3; ++k)
                                v[k] = (last.root[k] - cap[k]) / dtI;
                        const bool own = feedI(detail, v);
                        const bool owns = motiondriver::OwnsTraversal();
                        const motiondriver::TraversalProbe pc = motiondriver::ProbeTraversal();
                        if (pc.frames > last.frames)
                        {
                            ++frames;
                            framesOwned += own && owns ? 1 : 0;
                            const mm::integ::PoseSnapshot* sn = motiondriver::LastMotionSnapshot();
                            if (sn && frames > entryFeeds && TravRefPose(db, tf, pc.clip, pc.montage, pc.poseTime, ref))
                            {
                                poseMax = std::max(poseMax, MeanAngleToRef(ref, *sn));
                                ++posesChecked;
                                const float df = std::fabs(pc.poseTime * static_cast<float>(m.fps) - static_cast<float>(pc.frontEndFrame));
                                int hand = -1;
                                for (uint32_t bi = 0; bi < db.NumBones(); ++bi)
                                    if (db.BoneName(bi) == m.windows[static_cast<size_t>(fw)].contactBone)
                                        hand = static_cast<int>(bi);
                                float hp[3];
                                if (df < handDf && PublishedBoneModel(*sn, db, hand, hp))
                                {
                                    handDf = df;
                                    for (int k = 0; k < 3; ++k)
                                        handVec[k] = cap[k] + hp[k] - pc.handTarget[k];
                                    handErr = std::hypot(std::hypot(handVec[0], handVec[1]), handVec[2]);
                                    handOff = std::hypot(std::hypot(pc.offset[0], pc.offset[1]), pc.offset[2]);
                                    planErr = std::hypot(std::hypot(pc.handContact[0] - pc.handTarget[0], pc.handContact[1] - pc.handTarget[1]),
                                                         pc.handContact[2] - pc.handTarget[2]);
                                }
                            }
                        }
                        if (pc.handbacks > q1.handbacks)
                            hbFeed = i;
                        last = pc;
                    }
                    for (int i = 0; i < 4; ++i)
                        feedI(detail, zero);
                    const bool ownsInEpisode = motiondriver::OwnsTraversal();
                    for (int i = 0; i < 6; ++i)
                        feedI(-1, zero);
                    const bool ownsAfter = motiondriver::OwnsTraversal();
                    const bool played = accepted && frames >= 5 && framesOwned == frames;
                    const bool poseOk = posesChecked >= 3 && poseMax <= 8.0f;
                    const bool handOk = handDf <= 1.0f && handErr <= 0.05f;
                    const bool ownOk = ownsInEpisode && !ownsAfter;
                    nPlayed += played ? 1 : 0;
                    nPose += poseOk ? 1 : 0;
                    nHand += handOk ? 1 : 0;
                    nHb += hbFeed >= 0 ? 1 : 0;
                    nOwn += ownOk ? 1 : 0;
                    worstPose = std::max(worstPose, posesChecked ? poseMax : 999.0f);
                    worstHand = std::max(worstHand, handErr);
                    std::printf("INFO 8i %s %s: %s acc %d poses %d frames %d owned %d, pose max %.2f deg (%d), hand %.3f m (df %.2f; published - target %.3f %.3f %.3f, plan contact - target %.3f, |plan root - capsule| %.3f), handback %d, owns in/after %d/%d, start %d end %d\n",
                                bn, m.name.c_str(), climb ? "climb" : "vault", accepted ? 1 : 0, q1.fromPoseFile ? 1 : 0, frames,
                                framesOwned, static_cast<double>(poseMax), posesChecked, static_cast<double>(handErr),
                                static_cast<double>(handDf), static_cast<double>(handVec[0]), static_cast<double>(handVec[1]),
                                static_cast<double>(handVec[2]), static_cast<double>(planErr), static_cast<double>(handOff),
                                hbFeed >= 0 ? 1 : 0, ownsInEpisode ? 1 : 0, ownsAfter ? 1 : 0, q1.startFrame, q1.endFrame);
                    if (!(played && poseOk && handOk && hbFeed >= 0 && ownOk))
                        bad += " " + m.name;
                }
                motiondriver::TestForceTraversal("", true);
                motiondriver::SetEnabled(false);
                char di[640];
                std::snprintf(di, sizeof di, "%s: accepted %d/34 (from poses %d, expected %zu), played+owned %d, pose %d (worst %.2f deg), hand %d (worst %.3f m), handback %d, ownership %d; failing:%s",
                              bn, nAcc, nPoses, 34 - inDb, nPlayed, nPose, static_cast<double>(worstPose), nHand,
                              static_cast<double>(worstHand), nHb, nOwn, bad.empty() ? " none" : bad.substr(0, 400).c_str());
                Check(nAcc == 34 && nPoses == static_cast<int>(34 - inDb) && nPlayed == 34,
                      std::string("8i ") + bn + ": all 34 montages accepted and played through the real path, v2-owned (db clip or version 2 poses)", di);
                Check(nPose == 34, std::string("8i ") + bn + ": published pose = montage pose after the entry blend (all 34, mean angle <= 8 deg)", di);
                Check(nHand == 34, std::string("8i ") + bn + ": published FK hand at the FrontLedge end frame on the game's hand point (all 34, <= 0.05 m)", di);
                Check(nHb == 34 && nOwn == 34, std::string("8i ") + bn + ": handback and OwnsTraversal true in / false after the episode (all 34)", di);
            }
            motiondriver::SetEnabled(false);
        }
    }

    } // coreShard

    // ---------------------------------------------------------------- 7e batch 2 (1cf5307): v2 leg context (child)
    // fresh process (LegCtxChildMain): v1 crouch aim strafe with ChestAlign, then v2 owns the feeds: the v2 LegContext carries the
    // crouch stance to StanceGuard (crouch swivel gate), ChestAlign never runs on a v2 frame, the strafe mode leaves after the
    // first v2-owned feed
    if (mine7e)
    {
        const DWORD code = WaitChild(kid7e);
        std::string d;
        {
            std::ifstream in(kid7e.out);
            std::getline(in, d);
        }
        DeleteFileA(kid7e.out.c_str());
        d = v6Db + ": " + d + " (child exit " + std::to_string(code) + ")";
        Check(code == 0 || code >= 4, "7e precondition: base crouch aim strafe with ChestAlign, then motion owned >= 60 of 90 feeds", d);
        Check(code != 4 && code != 3, "7e motion frames: LegContext stance crouch reaches StanceGuard (crouch swivel gate on)", d);
        Check(code != 5 && code != 3, "7e motion frames: no ChestAlign", d);
        Check(code != 6 && code != 3, "7e strafe mode off from the second v2-owned feed on (strafe not mapped to v2)", d);
        Check(code == 0, "7e handback: strafe re-enters by the feed after the handback feed (one-feed exit lag at most)", d);
    }

    // ---------------------------------------------------------------- 9a motionfix items 1 + 4 (child, MotionFixChildMain)
    // real in-game Locomotion frames (mv (0, 1) + target) through the real MMDriver + MotionDriver with the MOTION trace captured:
    // no IsPivoting and no Pivots db / clip on the checked feeds (item 1), a trace line on every v2 clip change, 2 Hz periodic
    // lines, none with the trace off (item 4)
    if (mine9a)
    {
        const DWORD code = WaitChild(kid9a);
        std::string d, l;
        {
            std::ifstream in(kid9a.out);
            std::getline(in, d);
            int shown = 0;
            while (std::getline(in, l) && shown < 6)
            {
                std::printf("  9a trace: %s\n", l.c_str());
                ++shown;
            }
        }
        DeleteFileA(kid9a.out.c_str());
        d += " (child exit " + std::to_string(code) + ")";
        std::printf("9a motionfix: %s\n", d.c_str());
        Check(code != 3 && code != 1, "9a precondition: motion owns >= 55 of the 60 checked feeds of each real window, trace lines written", d);
        Check(code != 4 && code != 3 && code != 1, "9a item 1: no IsPivoting on the real side / toward-camera run and crouch walk feeds", d);
        Check(code != 5 && code != 4 && code != 3 && code != 1, "9a item 1: motion picks no *Pivots db / *Pivot* clip on those feeds", d);
        Check(code == 0 || code == 7 || code == 8, "9a item 4: a MOTION trace line for the new clip on every motion clip change", d);
        Check(code == 0 || code == 8, "9a item 4: no MOTION line with the trace off", d);
        Check(code == 0, "9a item 4: periodic 2 Hz lines (>= 0.45 s apart)", d);
        Check(code != 9 && code != 3 && code != 1, "9a Locomotion: native body turn remains available on v2-owned toward-camera run feeds", d);
        Check(code != 10 && code != 9 && code != 3 && code != 1, "9a Locomotion: v2-owned feeds issue bounded yaw commands toward the movement target", d);
    }

    // ---------------------------------------------------------------- 7f review minor B: aim jog while v2 owns (child)
    // fresh process (AimJogChildMain): stand gun aim walk forward with jogF 3.3; the strafe mode is forced off while v2 owns, the
    // jog capsule speed (AimJogTarget) is not: 3.3 m/s on every v2-owned feed and through the handback (no flip to vanilla 1.65)
    if (mine7f)
    {
        const DWORD code = WaitChild(kid7f);
        std::string d;
        {
            std::ifstream in(kid7f.out);
            std::getline(in, d);
        }
        DeleteFileA(kid7f.out.c_str());
        d = v6Db + ": " + d + " (child exit " + std::to_string(code) + ")";
        Check(code == 0 || code >= 8, "7f precondition: base stand aim jog answers 3.3, then motion owned >= 60 of 90 feeds, handback seen", d);
        Check(code != 8 && code != 3, "7f v2-owned aim run keeps the 3.3 m/s jog target (strafe ForceOff does not drop it)", d);
        Check(code == 0, "7f handback: jog target stays 3.3 m/s through the handback (no flip)", d);
    }

    // ---------------------------------------------------------------- 7h merge repair (b4 review): air Clips mode x v2 (child)
    // fresh process (AirClipsChildMain) on a v1 database with its .airz (Clips mode; 7g runs Hold): no v1 air clip forced while v2
    // owns or on the handback feed; the AirState reset after a mid-air handback does not latch the pre-span frame as the query
    // bone frame; a handback from a v1 air clip frame (root_z) searches with the last clean ground frame's bone features
    if (mine7h)
    {
        const DWORD code = WaitChild(kid7h);
        std::string d;
        {
            std::ifstream in(kid7h.out);
            std::getline(in, d);
        }
        DeleteFileA(kid7h.out.c_str());
        d = v6Db + ": " + d + " (child exit " + std::to_string(code) + ")";
        Check(code == 0 || code >= 10, "7h precondition: Clips mode (base jump forces air clips), motion owned the air feeds, handbacks seen", d);
        Check(code != 10 && code != 3, "7h no base air clip forced while motion owns or on the handback feed (Clips mode)", d);
        Check(code != 11 && code != 3 && code != 10, "7h AirState reset after a motion handback: no stale pre-span query bone frame", d);
        Check(code == 0, "7h handback from a base air clip frame (root_z): fresh search with the clean ground frame's bone features", d);
        Check(code != 13 && code != 3, "7h v2-owned feeds and first handback: no v1 air-steer command armed", d);
        Check(code != 14 && code != 3, "7h mid-air reset: LegContext remains airborne with zero foot contacts", d);
    }

    // 7i: separate process keeps the pre-arm mask timeline independent of 7g/7h.
    if (mine7i)
    {
        const DWORD code = WaitChild(kid7i);
        std::string d;
        { std::ifstream in(kid7i.out); std::getline(in, d); }
        DeleteFileA(kid7i.out.c_str());
        Check(code != 3 && code != 1, "7i precondition: mask armed and v2->v1 handback observed", d);
        Check(code != 15 && code != 3 && code != 1, "7i published upperScale bounded and masked through motion ownership", d);
        Check(code == 0, "7i published upperScale eases upward after unmasked handback", d);
    }

    for (const HzKid& k : exitKids)
    {
        const DWORD code = WaitChild(k.kid);
        std::string detail;
        { std::ifstream input(k.kid.out); std::getline(input, detail); }
        DeleteFileA(k.kid.out.c_str());
        Check(code == 0, "Batch6 pistol aim exit " + k.variant + " at " + std::to_string(k.hz) +
              "Hz: policy ON eases over 0.25s, OFF keeps the legacy motion cut", detail);
    }
    for (const HzKid& k : weaponKids)
    {
        const std::string& variant = k.variant;
        const int hz = k.hz;
        const std::string& out = k.kid.out;
        const DWORD code = WaitChild(k.kid);
        std::string detail;
        { std::ifstream input(out); std::getline(input, detail); }
        const bool grip = variant.find("grip") != std::string::npos;
        const bool authored = k.authored;
        bool paired = true;
        if (authored)
        {
            const std::string& controlOut = k.control.out;
            const DWORD controlCode = WaitChild(k.control);
            std::ifstream perturbed(out + ".arms.csv"), control(controlOut + ".arms.csv");
            std::string p, c;
            int frames = 0;
            paired = controlCode == 0 && perturbed.good() && control.good();
            while (std::getline(perturbed, p))
            {
                paired = paired && std::getline(control, c) && p == c && std::count(p.begin(), p.end(), ',') == 12;
                ++frames;
            }
            paired = paired && frames == 5 && !std::getline(control, c);
            detail += " pairedFrames=" + std::to_string(frames) + " pairedEqual=" + std::to_string(paired);
            perturbed.close();
            control.close();
            DeleteFileA(controlOut.c_str()); DeleteFileA((controlOut + ".arms.csv").c_str());
        }
        DeleteFileA(out.c_str()); DeleteFileA((out + ".arms.csv").c_str());
        Check(code == 0 && paired, "weapon upper/grip " + variant + " at " + std::to_string(hz) +
              (variant.find("nue-recoil") != std::string::npos ?
               "Hz: motion slide keeps authored weapon recoil, MM arms (upperScale 1) and the hipfire pose for every shot" :
               variant.find("reload-mask") != std::string::npos ? "Hz: Reload arm mask restores torso ownership for Aim and base" :
               grip ? "Hz: source-specific IK, paired authored independence, rearm and holster contracts" :
               "Hz: held/reload/AIR mask only the upper body; OFF/empty/unknown/release keep their contracts"), detail);
    }
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    std::printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

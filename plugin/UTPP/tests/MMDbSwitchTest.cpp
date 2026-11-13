// UTPP 0.5.1 offline test: runtime motion matching database switching through the real MMDriver (no game).
// Usage: utpp_mm_dbswitch_test <default.utppmm> <alternative.utppmm> [extra.utppmm ...]
// A temporary folder is laid out like the plugin folder (utpp.utppmm + db\*.utppmm). Checked: listing / name
// validation, load + SHA, switch with fade-out -> swap at weight 0 -> fade-in, state reset, failure keeps the active
// database (corrupt file, missing name), no-op for the active name, switch while MM is disabled (Tick path).
// Extra databases are only reported (INFO: would they load / pass the bone-set check against the default).
// loco P0: for every extra database, the mask-derived class data after a default -> extra switch equals a fresh load; the
// matcher's class set after the recompute is switched on while walking; the stale counter / status segment.

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
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/State.hpp"
#include "../src/mm/BaseMatcher.hpp"
#include "../src/mm/SpeedCalibration.hpp"

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

namespace
{
int g_pass = 0, g_fail = 0;
void Check(const char* aName, bool aOk, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
}

float PublishedWeight()
{
    static utpp::mm::integ::PoseSnapshot snap; // large, keep off the stack
    return utpp::g_mmPose.Read(snap) ? snap.weight : -1.0f;
}

void FeedFrames(int aFrames, float* aMaxWeight = nullptr, float* aMinWeight = nullptr, float aSpeed = 1.5f, float aStick = 1.0f)
{
    utpp::motionhost::FeedInput in;
    in.velWorld[1] = aSpeed;
    in.move[1] = aStick;
    in.locoState = 1;
    in.tppActive = true;
    in.dt = 1.0f / 60.0f;
    for (int i = 0; i < aFrames; ++i)
    {
        utpp::motionhost::Feed(in);
        const float w = PublishedWeight();
        if (aMaxWeight && w > *aMaxWeight)
            *aMaxWeight = w;
        if (aMinWeight && w < *aMinWeight)
            *aMinWeight = w;
    }
}

// 0.5.2: the calibration the driver must install for a database file (same masks as ComputeStateTagMasksLocked)
utpp::mm::SpeedCalibration DeriveFile(const std::string& aPath)
{
    using namespace utpp::mm;
    Database db;
    std::string err;
    db.LoadFromFile(aPath.c_str(), utpp::motionhost::kExpectedRigHash, err);
    const uint64_t excl = BaseMatcher::TagMaskContaining(db, "_Lands_") | BaseMatcher::TagMaskContaining(db, "FromTraversal");
    return DeriveSpeedCalibration(db, BaseMatcher::TagMaskContaining(db, "Stand_") & ~excl,
                                  BaseMatcher::TagMaskContaining(db, "Crouch_") & ~excl);
}

bool MapEquals(int aLoco, const utpp::mm::SpeedPoints& aExpect)
{
    float g[3], d[3];
    int32_t n = 0;
    if (!utpp::motionhost::GetSpeedMap(aLoco, g, d, n) || n != aExpect.count)
        return false;
    for (int i = 0; i < n; ++i)
        if (std::fabs(g[i] - aExpect.game[i]) > 1e-5f || std::fabs(d[i] - aExpect.db[i]) > 1e-5f)
            return false;
    return true;
}

std::string MapString(int aLoco)
{
    float g[3], d[3];
    int32_t n = 0;
    utpp::motionhost::GetSpeedMap(aLoco, g, d, n);
    std::string out;
    for (int i = 0; i < n; ++i)
        out += std::to_string(g[i]) + ">" + std::to_string(d[i]) + " ";
    return out;
}

std::string Field(const std::string& aStatus, const char* aKey)
{
    const std::string k = std::string(" ") + aKey + "=";
    const size_t p = aStatus.find(k);
    if (p == std::string::npos)
        return {};
    size_t e = aStatus.find(' ', p + k.size());
    if (aStatus[p + k.size()] == '"')
        e = aStatus.find('"', p + k.size() + 1) + 1;
    return aStatus.substr(p + k.size(), e - p - k.size());
}
} // namespace

int main(int argc, char** argv)
{
    namespace fs = std::filesystem;
    using namespace utpp::motionhost;
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_dbswitch_test <default.utppmm> <alternative.utppmm> [extra ...]\n");
        return 2;
    }
    const fs::path dir = fs::temp_directory_path() / "utpp_dbswitch_test";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir / "data");
    fs::copy_file(argv[1], dir / "data" / "utpp_base.utppmm");
    fs::copy_file(argv[2], dir / "data" / "posture.utppmm");
    {
        std::ofstream sc(dir / "data" / "posture.json", std::ios::binary); // 0.5.2 valid sidecar
        sc << "{ \"walk\": 2.0, \"run\": 5.0, \"sprint\": 7.0, \"crouch\": 2.2 }";
    }
    {
        std::ofstream sc(dir / "data" / "utpp_base.json", std::ios::binary); // 0.5.2 broken sidecar of the default database
        sc << "{ \"walk\": ";
    }
    {
        std::ofstream bad(dir / "data" / "bad.utppmm", std::ios::binary);
        bad << "not a utppmm database";
    }
    {
        std::ofstream ignored(dir / "data" / "bad name!.utppmm", std::ios::binary);
        ignored << "x";
    }
    {
        std::ofstream ignored(dir / "data" / "notes.txt", std::ios::binary);
        ignored << "x";
    }
    for (const char* motion : {"utpp_motion_woman.utppmm", "utpp_motion_man.utppmm"}) // per-body motion data: not selectable
    {
        std::ofstream ignored(dir / "data" / motion, std::ios::binary);
        ignored << "x";
    }

    // names
    Check("name validation", IsValidDatabaseName("test_db-full.2") && !IsValidDatabaseName("") &&
                                 !IsValidDatabaseName("../x") && !IsValidDatabaseName("a\\b") &&
                                 !IsValidDatabaseName(".hidden") && !IsValidDatabaseName(std::string(65, 'a')));

    SetDatabaseDir(dir.wstring());
    Check("list: default + valid db names sorted, invalid / other files and the per-body motion data ignored",
          ListDatabases() == "default;bad;posture",
          ListDatabases());
    Check("request before any load: unknown name rejected", !RequestDatabase("missing"));

    const bool loaded = LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default");
    Check("load default", loaded && DatabaseName() == "default" && DatabaseSha().size() == 64, DatabaseSha());
    const utpp::mm::SpeedCalibration calDefault = DeriveFile(argv[1]);
    Check("calibration at load: derived from the default database (stand, sprint, crouch maps)",
          MapEquals(1, calDefault.stand) && MapEquals(3, calDefault.stand) && MapEquals(2, calDefault.crouch), MapString(1));
    Check("calibration at load: broken sidecar rejected and reported", SpeedCalibrationSummary().find("sidecar REJECTED") != std::string::npos &&
          Field(Status(), "speedCal").find("REJECTED") != std::string::npos, SpeedCalibrationSummary());

    // fake armed rig = database bones (names / parents)
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
    static int rigDummy = 0;
    utpp::g_hot.armed.store(true);
    OnArmed(&rigDummy, hashes, parents);
    SetEnabled(true);
    SetHoldPose(false); // 0.6.2: the 0.5.1 path (fade-out, swap at weight 0, fade-in); the hold-pose switch is checked below
    float maxW = 0.0f;
    FeedFrames(60, &maxW);
    Check("MM fades in on the default database", PublishedWeight() >= 0.999f, std::to_string(PublishedWeight()));

    // switch default -> posture
    Check("request posture queued", RequestDatabase("posture"));
    WaitDatabaseWorker();
    Check("worker done: pending, still the default active", DatabaseSwitchStatus() == "pending posture" && DatabaseName() == "default",
          DatabaseSwitchStatus());
    int framesToSwap = 0;
    float maxDuring = 0.0f;
    float prev = PublishedWeight();
    bool monotonic = true;
    while (DatabaseName() != "posture" && framesToSwap < 120)
    {
        FeedFrames(1);
        const float w = PublishedWeight();
        if (w > prev + 1e-6f)
            monotonic = false;
        prev = w;
        maxDuring = std::max(maxDuring, w);
        ++framesToSwap;
    }
    Check("fade-out before the swap (weight never rises), swap within 0.2 s", DatabaseName() == "posture" && monotonic &&
          framesToSwap <= 12, std::to_string(framesToSwap) + " frames");
    Check("weight 0 at the swap", PublishedWeight() == 0.0f);
    const std::string st = Status();
    Check("status: name, sha prefix, switch ok, 1 switch", Field(st, "dbName") == "posture" && Field(st, "dbSha").size() == 12 &&
          Field(st, "dbSwitch") == "\"ok posture\"" && Field(st, "dbSwitches") == "1",
          Field(st, "dbName") + " " + Field(st, "dbSha") + " " + Field(st, "dbSwitch"));
    Check("bone map rebuilt after the switch", Field(st, "map").find("ok") != std::string::npos, Field(st, "map"));
    {
        float g[3], d[3];
        int32_t n = 0;
        GetSpeedMap(1, g, d, n);
        float gc[3], dc[3];
        int32_t nc = 0;
        GetSpeedMap(2, gc, dc, nc);
        Check("switch applies the new calibration (posture.json sidecar 2.0 / 5.0 / 7.0, crouch 2.2)",
              n == 3 && d[0] == 2.0f && d[1] == 5.0f && d[2] == 7.0f && nc == 1 && dc[0] == 2.2f &&
                  SpeedCalibrationSummary().find("| sidecar") != std::string::npos,
              SpeedCalibrationSummary());
    }
    FeedFrames(90);
    Check("MM fades back in on the new database", PublishedWeight() >= 0.999f, std::to_string(PublishedWeight()));
    Check("different file -> different sha", DatabaseSha() != std::string() && Field(Status(), "dbSha") != "");

    // failure keeps the active database
    Check("corrupt database queued", RequestDatabase("bad"));
    WaitDatabaseWorker();
    float minW = 2.0f;
    FeedFrames(30, nullptr, &minW);
    Check("corrupt database: FAILED, active kept, MM not interrupted",
          DatabaseSwitchStatus().rfind("FAILED bad:", 0) == 0 && DatabaseName() == "posture" && minW >= 0.999f,
          DatabaseSwitchStatus());
    Check("missing database rejected", !RequestDatabase("missing") && DatabaseSwitchStatus().find("file not found") != std::string::npos);
    Check("invalid name rejected", !RequestDatabase("..\\utpp"));
    Check("active name = no-op ok", RequestDatabase("posture") && DatabaseSwitchStatus() == "ok posture");

    // switch while MM is disabled: Tick performs the swap
    SetEnabled(false);
    Check("request default while disabled", RequestDatabase("default"));
    WaitDatabaseWorker();
    for (int i = 0; i < 100 && DatabaseName() != "default"; ++i)
    {
        Sleep(10);
        Tick();
    }
    Check("disabled: swapped by Tick", DatabaseName() == "default" && DatabaseSwitchStatus() == "ok default");
    Check("switch back: default calibration restored", MapEquals(1, calDefault.stand) && MapEquals(2, calDefault.crouch), MapString(1));
    SetEnabled(true);
    FeedFrames(90);
    Check("re-enabled: MM runs on the default database", PublishedWeight() >= 0.999f);

    // unarmed switch (no rig): map rebuilt at the next arming
    OnDisarmed();
    utpp::g_hot.armed.store(false);
    Check("unarmed switch queued", RequestDatabase("posture"));
    WaitDatabaseWorker();
    for (int i = 0; i < 100 && DatabaseName() != "posture"; ++i)
    {
        Sleep(10);
        Tick();
    }
    Check("unarmed: swapped", DatabaseName() == "posture");
    utpp::g_hot.armed.store(true);
    OnArmed(&rigDummy, hashes, parents);
    FeedFrames(90);
    Check("re-armed: mapped and running", PublishedWeight() >= 0.999f && Field(Status(), "map").find("ok") != std::string::npos);

    // 0.6.2 hold last pose: a switch in TPP keeps the body - swap at full weight on the next feed, the held pose stays published
    SetHoldPose(true);
    {
        Check("hold: request default queued", RequestDatabase("default"));
        WaitDatabaseWorker();
        float minHold = 2.0f;
        int frames = 0;
        while (DatabaseName() != "default" && frames < 120)
        {
            FeedFrames(1, nullptr, &minHold);
            ++frames;
        }
        FeedFrames(30, nullptr, &minHold);
        const std::string hs = Status();
        Check("hold: switch at full weight (published weight never below 1), swapped on the first feed, MM continues on the new database",
              DatabaseName() == "default" && frames <= 2 && minHold >= 0.999f && Field(hs, "holdSwaps") == "1" &&
                  DatabaseSwitchStatus() == "ok default",
              std::to_string(frames) + " frames, min w " + std::to_string(minHold) + ", holdSwaps " + Field(hs, "holdSwaps"));
    }
    SetHoldPose(false);

    // extra databases: INFO only
    for (int a = 3; a < argc; ++a)
    {
        const fs::path src = argv[a];
        const std::string name = "extra" + std::to_string(a - 2);
        fs::copy_file(src, dir / "data" / (name + ".utppmm"), fs::copy_options::overwrite_existing);
        RequestDatabase(name);
        WaitDatabaseWorker();
        double maxFeedMs = 0.0; // longest Feed call = the swap (InitMatcher on the game thread)
        for (int i = 0; i < 100 && DatabaseSwitchStatus().rfind("pending", 0) == 0; ++i)
        {
            LARGE_INTEGER t0, t1, fq;
            QueryPerformanceFrequency(&fq);
            QueryPerformanceCounter(&t0);
            FeedFrames(1);
            QueryPerformanceCounter(&t1);
            maxFeedMs = std::max(maxFeedMs, double(t1.QuadPart - t0.QuadPart) * 1000.0 / double(fq.QuadPart));
        }
        if (DatabaseName() == name)
        {
            const utpp::mm::SpeedCalibration ce = DeriveFile(src.string());
            Check(("switch to " + src.filename().string() + " installs its own calibration").c_str(),
                  MapEquals(1, ce.stand) && MapEquals(3, ce.stand) && MapEquals(2, ce.crouch), SpeedCalibrationSummary());
        }
        std::printf("[INFO] %s (%s): %s, active=%s, longest game-thread feed during the switch %.1f ms\n",
                    src.filename().string().c_str(), name.c_str(), DatabaseSwitchStatus().c_str(), DatabaseName().c_str(),
                    maxFeedMs);
    }

    // loco P0: the data derived from the state tag masks (0.5.8 loop bands + availability, crouch lateral rule; with the clip
    // classes, matcher class sets and turn profiles) after a default -> X switch equals a fresh load of X. 0.6.5b built the loop
    // bands in the matcher init, before the masks (after a switch: the previous database's tag bits; in game default -> v6).
    // Reference = load + UTPPSetMMEnabled(true) (matcher init with the database's own masks in place). Negative control:
    // UTPPSetMMDerivedRecompute(false) = the 0.6.5b order -> stale data after the switch on >= 1 database; switching it back on
    // recomputes the active database at once.
    {
        int extras = 0, staleOff = 0, resetExercised = 0;
        std::string staleNames, resetNames;
        for (int a = 3; a < argc; ++a)
        {
            const fs::path src = argv[a];
            const std::string name = "extra" + std::to_string(a - 2);
            const std::string file = src.filename().string();
            ++extras;
            std::string sw[2]; // 0 = recompute on, 1 = off (0.6.5b order)
            std::string swStrafe; // B2 strafe data after the switch (recompute on)
            std::string swChest;  // B2 strafe S2 chest references after the switch (recompute on)
            long long staleDelta = -1; // derivedStale counted by the switch (recompute on)
            bool offSegment = true;    // Status carries the P0 segment while off (must not: 0.6.5b text)
            const auto staleCount = []
            {
                const std::string v = Field(Status(), "derivedStale");
                return v.empty() ? -1LL : std::stoll(v);
            };
            for (int off = 0; off < 2; ++off)
            {
                SetDerivedRecompute(off == 0);
                LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default");
                OnArmed(&rigDummy, hashes, parents);
                SetEnabled(true); // in game: CET enables MM after the default load, then the worker switch arrives
                FeedFrames(30);
                const long long stale0 = staleCount();
                RequestDatabase(name);
                WaitDatabaseWorker();
                for (int i = 0; i < 120 && DatabaseName() != name; ++i)
                    FeedFrames(1);
                sw[off] = DatabaseName() == name ? DerivedClassDataDigest() : "switch failed: " + DatabaseSwitchStatus();
                if (off == 0)
                {
                    staleDelta = stale0 < 0 ? -1 : staleCount() - stale0;
                    swStrafe = DatabaseName() == name ? StrafeDataDigest() : "switch failed";
                    swChest = DatabaseName() == name ? ChestAlignDigest() : "switch failed";
                }
                else
                    offSegment = Status().find(" derivedRecompute=") != std::string::npos;
            }
            // Runtime path: a slow walk (1 m/s, stick 0.3: walk band only) on the stale data puts a banded class set into the
            // matcher (built from the stale loop bands, the stale run loops off band); switching the recompute back on must leave
            // the matcher on the class set the fresh tables give (the reset to the base set), at once and after further feeds
            // (ApplyClassSetLocked only rebuilds when the set / band changes).
            FeedFrames(60, nullptr, nullptr, 1.0f, 0.3f);
            const ClassSetProbe pre = ProbeClassSet();
            SetDerivedRecompute(true); // back on: the active database (switched with the 0.6.5b order) is recomputed at once
            const ClassSetProbe post = ProbeClassSet(pre.set, pre.band);
            const std::string onAgain = DerivedClassDataDigest();
            FeedFrames(30, nullptr, nullptr, 1.0f, 0.3f);
            const ClassSetProbe post2 = ProbeClassSet();
            {
                char d[200];
                std::snprintf(d, sizeof d,
                              "before set %d band %d held=expect %d | after set %d band %d held=expect %d | +30 feeds set %d band "
                              "%d held=expect %d",
                              pre.set, pre.band, pre.held == pre.expectActive ? 1 : 0, post.set, post.band,
                              post.held == post.expectActive ? 1 : 0, post2.set, post2.band, post2.held == post2.expectActive ? 1 : 0);
                Check(("loco P0: " + file + " recompute switched back on while walking: the matcher holds the class set of the fresh "
                       "tables (at once and after 30 feeds)").c_str(),
                      pre.ready && post.ready && post2.ready && pre.held == pre.expectActive && post.held == post.expectActive &&
                          post2.held == post2.expectActive,
                      d);
                if (pre.band > 0 && post.expectAt != pre.held)
                {
                    ++resetExercised; // the banded set held before the recompute is stale under the fresh tables
                    resetNames += " " + file;
                }
            }
            LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default"); // the default database's masks in place
            const bool loadedFresh = LoadDatabase(src.wstring(), name);
            const std::string fresh = DerivedClassDataDigest();
            const std::string freshStrafe = StrafeDataDigest();
            const std::string freshChest = ChestAlignDigest();
            SetEnabled(true);
            const std::string ref = DerivedClassDataDigest();
            const std::string refStrafe = StrafeDataDigest();
            // B2 strafe S1: the strafe profiles (availability, sectors, per-direction loop speeds, class set 3) are derived from the
            // state tag masks as well: a default -> X switch gives the tables of a fresh load of X
            Check(("B2 strafe: default -> " + file + " switch and fresh load give the same strafe availability / tables / class set").c_str(),
                  loadedFresh && swStrafe == refStrafe && freshStrafe == refStrafe,
                  "switch " + swStrafe + " | fresh " + freshStrafe + " | ref " + refStrafe);
            std::printf("[INFO] B2 strafe %s: %s\n", file.c_str(), refStrafe.c_str());
            // B2 strafe S2: the chest references are rebuilt per database (load / switch), never the previous database's
            const std::string refChest = ChestAlignDigest();
            Check(("B2 strafe S2: default -> " + file + " switch and fresh load give the same chest align references").c_str(),
                  loadedFresh && swChest == refChest && freshChest == refChest,
                  "switch " + swChest + " | fresh " + freshChest + " | ref " + refChest);
            std::printf("[INFO] B2 strafe S2 chest %s: %s\n", file.c_str(), refChest.c_str());
            Check(("loco P0: default -> " + file + " switch and fresh load give the reference class data (loop bands, availability, "
                   "lateral rule, class sets)").c_str(),
                  loadedFresh && sw[0] == ref && fresh == ref, "switch " + sw[0] + " | fresh " + fresh + " | ref " + ref);
            Check(("loco P0: " + file + " recompute switched back on after an 0.6.5b-order switch: reference data at once").c_str(),
                  onAgain == ref, onAgain);
            // the off digest differs from the reference only in the loop bands: derivedStale counts the switch iff they were stale
            // (the lateral rule held then is the previous database's, rebuilt by the calibration: not counted)
            Check(("loco P0: " + file + " derivedStale counts the switch iff the loop bands were stale; no status segment while off")
                      .c_str(),
                  staleDelta == (sw[1] != ref ? 1 : 0) && !offSegment,
                  "delta " + std::to_string(staleDelta) + " offSegment " + std::to_string(offSegment ? 1 : 0));
            if (sw[1] != ref)
            {
                ++staleOff;
                staleNames += " " + file;
            }
            std::printf("[INFO] loco P0 %s: recompute off (0.6.5b order) after the switch: %s\n", file.c_str(),
                        sw[1] == ref ? "same data (masks coincide)" : ("STALE " + sw[1]).c_str());
        }
        Check("loco P0 negative control: recompute off (0.6.5b order) leaves stale class data after the switch on >= 1 database",
              extras == 0 || staleOff >= 1, std::to_string(staleOff) + "/" + std::to_string(extras) + staleNames);
        Check("loco P0: the runtime reset is exercised (a banded class set built from stale loop bands was active) on >= 1 database",
              extras == 0 || resetExercised >= 1, std::to_string(resetExercised) + "/" + std::to_string(extras) + resetNames);
    }

    // AIR CLIPS repair: the .airz sidecar through the switch path (worker read, parse in TrySwapLocked, InitMatcher applies it).
    // First extra database with a <name>.airz next to it: default (no table) -> airA (table) -> bad (failed: airA and its table
    // kept) -> airB (same database, corrupt sidecar: badTable, matcher without root_z) -> airA again (table back) -> default.
    {
        fs::path airSrc;
        for (int a = 3; a < argc && airSrc.empty(); ++a)
        {
            fs::path p = argv[a];
            if (fs::exists(fs::path(p).replace_extension(".airz")))
                airSrc = p;
        }
        if (airSrc.empty())
            std::printf("[INFO] air sidecar switch: no extra database with a .airz sidecar (skipped)\n");
        else
        {
            fs::copy_file(airSrc, dir / "data" / "airA.utppmm", fs::copy_options::overwrite_existing);
            fs::copy_file(fs::path(airSrc).replace_extension(".airz"), dir / "data" / "airA.airz", fs::copy_options::overwrite_existing);
            fs::copy_file(airSrc, dir / "data" / "airB.utppmm", fs::copy_options::overwrite_existing);
            {
                std::ofstream bad(dir / "data" / "airB.airz", std::ios::binary);
                bad << "not an airz table";
            }
            SetDerivedRecompute(true);
            LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default");
            OnArmed(&rigDummy, hashes, parents);
            SetEnabled(true);
            FeedFrames(30);
            const auto air = [](const std::string& s) {
                return Field(s, "air") + " " + Field(s, "airReason") + " table " + Field(s, "airTable") + " rootZ " + Field(s, "airRootZ") +
                       " loopOv " + Field(s, "airLoopOv");
            };
            const auto switchTo = [&](const char* aName) {
                if (!RequestDatabase(aName))
                    return false;
                WaitDatabaseWorker();
                for (int i = 0; i < 120 && DatabaseSwitchStatus().rfind("pending", 0) == 0; ++i)
                    FeedFrames(1);
                FeedFrames(10);
                return true;
            };
            const std::string s0 = Status();
            const bool okA = switchTo("airA") && DatabaseName() == "airA";
            const std::string sA = Status();
            const bool qBad = switchTo("bad");
            const std::string sBad = Status();
            const std::string badInfo = std::string(qBad ? "queued" : "rejected") + ", active " + DatabaseName() + ", " + DatabaseSwitchStatus();
            const bool okB = switchTo("airB") && DatabaseName() == "airB";
            const std::string sB = Status();
            const bool okA2 = switchTo("airA") && DatabaseName() == "airA";
            const std::string sA2 = Status();
            const bool okD = switchTo("default") && DatabaseName() == "default";
            const std::string sD = Status();
            std::printf("[INFO] air sidecar switch (%s): default %s | airA %s | bad %s (%s) | airB %s | airA %s | default %s\n",
                        airSrc.filename().string().c_str(), air(s0).c_str(), air(sA).c_str(), air(sBad).c_str(), DatabaseSwitchStatus().c_str(),
                        air(sB).c_str(), air(sA2).c_str(), air(sD).c_str());
            const auto withTable = [](const std::string& s) {
                return Field(s, "air") == "clips" && Field(s, "airReason") == "auto:table" && !Field(s, "airTable").empty() &&
                       Field(s, "airRootZ") == "1" && Field(s, "airLoopOv") == "1";
            };
            const auto noTable = [](const std::string& s, const char* aReason) {
                return Field(s, "air") == "hold" && Field(s, "airReason").rfind(aReason, 0) == 0 && Field(s, "airTable").empty() &&
                       Field(s, "airRootZ").empty();
            };
            Check("air sidecar switch: default (no .airz) -> auto:noTable; switch to a database with its .airz -> table read by the worker, "
                  "applied to the matcher (airRootZ / airLoopOv 1)",
                  noTable(s0, "auto:noTable") && okA && withTable(sA), air(s0) + " | " + air(sA));
            Check("air sidecar switch: a failed switch keeps the active database and its table",
                  qBad && badInfo.find(", active airA, FAILED bad") != std::string::npos && withTable(sBad), badInfo + " | " + air(sBad));
            Check("air sidecar switch: corrupt .airz -> switch ok, hold with auto:badTable(..), no table in the matcher; switching back "
                  "restores the table; default again -> noTable",
                  okB && noTable(sB, "auto:badTable") && okA2 && withTable(sA2) && okD && noTable(sD, "auto:noTable"),
                  air(sB) + " | " + air(sA2) + " | " + air(sD));
        }
    }

    // b5 testorder repair: (1) LoadDatabase itself starts a fresh session (no ResetSessionState call: the loco suite's BeginSection
    // calls it, this check does not, so removing the reset from LoadDatabase fails here); (2) UTPPSetMMSwapReset on (default): a
    // swap clears the old database's playback state (swapPost 0); off: 0.7.0 swap, that state carries over (swapPost != 0).
    {
        const auto fresh = [](const std::string& s) {
            return Field(s, "feedCalls") == "0" && Field(s, "sinceStance") == "999.00" && Field(s, "stanceChanges") == "0" &&
                   Field(s, "rootRateActive") == "0" && Field(s, "committed") == "0" && Field(s, "settling") == "0" &&
                   Field(s, "swivelL") == "0.0" && Field(s, "swivelR") == "0.0" && Field(s, "gateCloses") == "0" &&
                   Field(s, "chestValid") != "1";
        };
        const auto sess = [](const std::string& s) {
            return "feedCalls " + Field(s, "feedCalls") + " sinceStance " + Field(s, "sinceStance") + " stanceChanges " +
                   Field(s, "stanceChanges") + " rootRateActive " + Field(s, "rootRateActive") + " committed " + Field(s, "committed") +
                   " swivel " + Field(s, "swivelL") + "/" + Field(s, "swivelR") + " gateCloses " + Field(s, "gateCloses");
        };
        LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default");
        OnArmed(&rigDummy, hashes, parents);
        SetEnabled(true);
        SetHoldPose(false);
        FeedFrames(90);
        FeedFrames(30, nullptr, nullptr, 0.0f, 0.0f); // stop: stance / stick history
        FeedFrames(60, nullptr, nullptr, 3.5f, 1.0f);
        const std::string before = Status();
        LoadDatabase((dir / "data" / "utpp_base.utppmm").wstring(), "default");
        OnArmed(&rigDummy, hashes, parents);
        const std::string after = Status();
        std::printf("[INFO] testorder LoadDatabase session reset: before %s | after %s\n", sess(before).c_str(), sess(after).c_str());
        Check("testorder: LoadDatabase alone (no ResetSessionState) starts a fresh session (feed / stance / RootRate / turn / StanceGuard "
              "state and counters of the previous session gone)",
              Field(before, "feedCalls") == "180" && !fresh(before) && fresh(after), sess(before) + " | " + sess(after));

        const auto switchTo = [&](const char* aName) {
            if (!RequestDatabase(aName))
                return false;
            WaitDatabaseWorker();
            for (int i = 0; i < 120 && DatabaseSwitchStatus().rfind("pending", 0) == 0; ++i)
                FeedFrames(1, nullptr, nullptr, 3.5f, 1.0f);
            return DatabaseName() == aName;
        };
        const auto swapInfo = [](const std::string& s) {
            return "swapReset " + Field(s, "swapReset") + " swapResets " + Field(s, "swapResets") + " carry " + Field(s, "swapCarry") +
                   " post " + Field(s, "swapPost");
        };
        SetEnabled(true);
        FeedFrames(120, nullptr, nullptr, 3.5f, 1.0f);
        const std::string s0 = Status();
        const bool okOn = switchTo("posture");
        const std::string sOn = Status();
        SetSwapReset(false);
        FeedFrames(120, nullptr, nullptr, 3.5f, 1.0f);
        const bool okOff = switchTo("default");
        const std::string sOff = Status();
        SetSwapReset(true);
        std::printf("[INFO] testorder swap reset: start %s | on -> posture %s | off -> default %s\n", swapInfo(s0).c_str(),
                    swapInfo(sOn).c_str(), swapInfo(sOff).c_str());
        const auto n = [](const std::string& v) { return v.empty() ? -1ll : std::stoll(v, nullptr, 0); };
        Check("testorder: swap reset on (default): the swap clears the old database's playback state (carry != 0, post 0, swapResets +1)",
              Field(s0, "swapReset") == "1" && okOn && n(Field(sOn, "swapCarry")) > 0 && n(Field(sOn, "swapPost")) == 0 &&
                  n(Field(sOn, "swapResets")) == n(Field(s0, "swapResets")) + 1,
              swapInfo(s0) + " | " + swapInfo(sOn));
        Check("testorder: swap reset off (0.7.0 swap): the playback state carries over (post != 0, swapResets unchanged)",
              okOff && Field(sOff, "swapReset") == "0" && n(Field(sOff, "swapPost")) > 0 &&
                  n(Field(sOff, "swapResets")) == n(Field(sOn, "swapResets")),
              swapInfo(sOff));
    }

    Shutdown();
    fs::remove_all(dir, ec);
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// Offline simulation test of the motion matching runtime core (Phase 4a, not shipped).
// Usage: utpp_mm_sim_test <database.utppmm>      Exit code 0 = all checks passed.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/BaseMatcher.hpp"
#include "../src/mm/AirTable.hpp" // AIR CLIPS stage

using namespace utpp::mm;

// ---------------------------------------------------------------- allocation counter (verifies no allocations in Update)
static std::atomic<bool> g_countAllocs{false};
static std::atomic<uint64_t> g_allocs{0};

static void* CountedAlloc(std::size_t n)
{
    if (g_countAllocs.load(std::memory_order_relaxed))
        g_allocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1))
        return p;
    throw std::bad_alloc();
}
void* operator new(std::size_t n) { return CountedAlloc(n); }
void* operator new[](std::size_t n) { return CountedAlloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

// ---------------------------------------------------------------- helpers
static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0)
{
    char buf[512];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d, e, f);
    return buf;
}

static constexpr std::string_view kBoneNames[73] = {
    "Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg",
    "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder", "Neck", "LeftArm",
    "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head", "LeftHand", "RightHand", "LeftHandEnd",
    "WeaponLeft", "RightHandEnd", "WeaponRight", "LeftHeel", "RightHeel", "LeftEye", "RightEye",
    "LeftInHandThumb", "LeftInHandIndex", "LeftInHandMiddle", "LeftInHandRing", "LeftInHandPinky",
    "RightInHandThumb", "RightInHandIndex", "RightInHandMiddle", "RightInHandRing", "RightInHandPinky",
    "LeftToeBase", "RightToeBase", "LeftHandThumb1", "LeftHandIndex1", "LeftHandMiddle1", "LeftHandRing1",
    "LeftHandPinky1", "RightHandThumb1", "RightHandIndex1", "RightHandMiddle1", "RightHandRing1",
    "RightHandPinky1", "LeftHandThumb2", "LeftHandIndex2", "LeftHandMiddle2", "LeftHandRing2", "LeftHandPinky2",
    "RightHandThumb2", "RightHandIndex2", "RightHandMiddle2", "RightHandRing2", "RightHandPinky2",
    "LeftHandIndex3", "LeftHandMiddle3", "LeftHandRing3", "LeftHandPinky3", "RightHandIndex3",
    "RightHandMiddle3", "RightHandRing3", "RightHandPinky3"};
static constexpr int32_t kBoneParents[73] = {
    -1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24, 11, 12, 22, 22,
    23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 29, 30, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 56, 57, 58, 59, 61, 62, 63, 64};

static const Database* g_db = nullptr;
static uint64_t g_tagIdle = 0, g_tagWalkLoop = 0, g_tagRunLoop = 0, g_tagWalkStart = 0, g_tagWalkStop = 0, g_tagTurn = 0;

static uint64_t ClipTags(int32_t aClip) { return aClip >= 0 ? g_db->Clips()[size_t(aClip)].tagMask : 0; }
static std::string ClipName(int32_t aClip) { return std::string(g_db->ClipName(static_cast<uint32_t>(aClip))); }

static double Percentile(std::vector<double> aV, double aP)
{
    if (aV.empty())
        return 0.0;
    std::sort(aV.begin(), aV.end());
    const size_t i = std::min(aV.size() - 1, static_cast<size_t>(aP * double(aV.size() - 1) + 0.5));
    return aV[i];
}

static bool PoseFinite(const Pose& aP)
{
    for (const BoneTransform& b : aP.bones)
        if (!math::IsFinite(b.p) || !math::IsFinite(b.q))
            return false;
    return true;
}

static uint64_t HashBytes(const void* aData, size_t aSize, uint64_t aH)
{
    return Fnv1a64(static_cast<const uint8_t*>(aData), aSize, aH);
}

static MatcherInput Walk(float aVx, float aVy)
{
    MatcherInput in;
    in.desiredVelocity[0] = aVx;
    in.desiredVelocity[1] = aVy;
    in.facingMode = FacingMode::Direction;
    const float l = std::sqrt(aVx * aVx + aVy * aVy);
    if (l > 1e-3f)
    {
        in.desiredFacing[0] = aVx / l;
        in.desiredFacing[1] = aVy / l;
    }
    else
    {
        in.desiredFacing[0] = 0.0f;
        in.desiredFacing[1] = 0.0f; // keep current facing
    }
    return in;
}

// Start/stop script: 0 (2 s) -> walk speed (4 s) -> run speed (4 s) -> 0 (5 s), forward.
static constexpr double kStartScriptEnd = 15.0;
static float StartScriptSpeed(double t, float aWalk, float aRun)
{
    if (t < 2.0)
        return 0.0f;
    if (t < 6.0)
        return aWalk;
    if (t < 10.0)
        return aRun;
    return 0.0f;
}

// Mean root speed (m/s) of a clip from the ROOT section.
static float ClipRootSpeed(const Database& aDb, std::string_view aName)
{
    for (uint32_t c = 0; c < aDb.NumClips(); ++c)
    {
        if (aDb.ClipName(c) != aName)
            continue;
        const ClipRecord& r = aDb.Clips()[c];
        double s = 0.0;
        for (uint32_t i = 0; i < r.numFrames; ++i)
            s += std::hypot(aDb.Root(r.firstFrame + i)->dx, aDb.Root(r.firstFrame + i)->dy);
        return static_cast<float>(s / r.numFrames * aDb.Fps());
    }
    return 0.0f;
}

// Mixed script for determinism / performance (period 20 s): walks in several directions, run, stop, yaw-rate turns.
static MatcherInput MixedScript(double t)
{
    const double u = std::fmod(t, 20.0);
    if (u < 2.0)
        return Walk(0.0f, 0.0f);
    if (u < 5.0)
        return Walk(0.0f, 1.5f);
    if (u < 7.0)
        return Walk(-1.06f, 1.06f); // 45 deg left, still in the character frame -> keeps turning
    if (u < 10.0)
        return Walk(0.0f, 3.75f);
    if (u < 12.0)
        return Walk(0.0f, 0.0f);
    if (u < 14.0)
    {
        MatcherInput in;
        in.desiredVelocity[1] = 1.5f;
        in.facingMode = FacingMode::YawRate;
        in.yawRate = 0.8f;
        return in;
    }
    if (u < 16.0)
        return Walk(1.5f, 0.0f); // strafe right
    if (u < 17.0)
        return Walk(0.0f, -1.5f); // backwards
    return Walk(0.0f, 0.0f);
}

struct RunStats
{
    uint64_t hash = 0xCBF29CE484222325ull;
    uint64_t transitions = 0, searches = 0;
    bool finite = true;
};

// ---------------------------------------------------------------- main
// ================================================================ AIR CLIPS stage (2026-10-13): matcher runtime APIs
// queryBoneFrame, ForcePlay (+ per-transition half-life), SetRootZ, SetLoopOverride / IsLoopClip. Synthetic part on the
// test database; real part on <db> <airz> argument pairs (v6 / v7c + sidecar; skipped and reported when missing).
static uint64_t PoseHash(const Pose& aP)
{
    return HashBytes(aP.bones.data(), aP.bones.size() * sizeof(BoneTransform), 1469598103934665603ull);
}

static int32_t FirstNonLoopClip(const Database& aDb, uint32_t aMinFrames)
{
    for (uint32_t c = 0; c < aDb.NumClips(); ++c)
        if ((aDb.Clips()[c].flags & kClipLoop) == 0 && aDb.Clips()[c].numFrames >= aMinFrames)
            return int32_t(c);
    return -1;
}

static void AirClipsSyntheticTests(const Database& db)
{
    const float dt = 1.0f / 60.0f;
    MatcherConfig cfg;
    // (h1) defaults / zero table bit-identical
    {
        BaseMatcher a, b;
        bool ok = a.Init(db, cfg) && b.Init(db, cfg);
        std::vector<float> zeros(db.NumFrames(), 0.0f);
        ok = ok && b.SetRootZ(zeros) && b.HasRootZ();
        uint64_t ha = 0, hb = 0;
        for (int i = 0; ok && i < 1200; ++i)
        {
            const MatcherInput in = MixedScript(i * double(dt));
            a.Update(dt, in);
            b.Update(dt, in);
            ha = HashBytes(a.Output().bones.data(), a.Output().bones.size() * sizeof(BoneTransform), ha);
            hb = HashBytes(b.Output().bones.data(), b.Output().bones.size() * sizeof(BoneTransform), hb);
        }
        Expect("(h1) air APIs: zero root_z table + default inputs are bit-identical to the plain matcher (20 s mixed script)",
               ok && ha == hb && a.Diagnostics().transitions == b.Diagnostics().transitions);
    }
    const int32_t nl = FirstNonLoopClip(db, 20);
    Expect("(h) non-loop clip >= 20 frames present", nl >= 0);
    if (nl < 0)
        return;
    const ClipRecord& nc = db.Clips()[size_t(nl)];
    // (h2) ForcePlay: exact frame, +1 transition, inertialized with the given half-life, outside the allowed tag set
    {
        MatcherConfig c2 = cfg;
        c2.allowedTagMask = g_tagIdle ? g_tagIdle : ~0ull;
        BaseMatcher m;
        bool ok = m.Init(db, c2);
        for (int i = 0; ok && i < 60; ++i)
            m.Update(dt, Walk(0.0f, 0.0f));
        const uint64_t tr0 = m.Diagnostics().transitions;
        int32_t nl2 = -1; // non-loop clip outside the idle tag set
        for (uint32_t c = 0; c < db.NumClips() && nl2 < 0; ++c)
            if ((db.Clips()[c].flags & kClipLoop) == 0 && db.Clips()[c].numFrames >= 20 && (db.Clips()[c].tagMask & g_tagIdle) == 0)
                nl2 = int32_t(c);
        const ClipRecord& nc2 = db.Clips()[size_t(nl2 >= 0 ? nl2 : nl)];
        const int32_t dst = int32_t(nc2.firstFrame + nc2.numFrames / 2);
        const bool outside = nl2 >= 0 && !m.ClipAllowed(nl2);
        ok = ok && m.ForcePlay(dst, 0.15f);
        const MatcherDiagnostics d0 = m.Diagnostics();
        const bool exact = d0.frame == dst && d0.transitions == tr0 + 1 && std::fabs(m.TransitionHalflife() - 0.15f) < 1e-6f;
        const float off0 = d0.maxOffsetDist;
        BaseMatcher slow; // same history, ForcePlay with a 0.6 s half-life
        bool okSlow = slow.Init(db, c2);
        for (int i = 0; okSlow && i < 60; ++i)
            slow.Update(dt, Walk(0.0f, 0.0f));
        okSlow = okSlow && slow.ForcePlay(dst, 0.6f);
        MatcherInput hold = Walk(0.0f, 0.0f);
        hold.suppressSearch = true;
        for (int i = 0; i < 4; ++i) // 0.6 s: 4 half-lives of 0.15 s, 1 of 0.6 s
        {
            m.Update(0.15f, hold);
            slow.Update(0.15f, hold);
        }
        const float off1 = m.Diagnostics().maxOffsetDist;
        const float offSlow = slow.Diagnostics().maxOffsetDist;
        const bool decays = okSlow && std::isfinite(off0) && off0 > 0.0f && std::isfinite(off1) && off1 < off0 && off1 < 0.5f * offSlow;
        const bool played = m.Diagnostics().clip == nl2;
        const bool bad = !m.ForcePlay(-1) && !m.ForcePlay(int32_t(db.NumFrames()));
        Expect("(h2) ForcePlay: exact frame, transitions +1, half-life 0.15 s (0.6 s later offset < half of a 0.6 s half-life run), plays outside the allowed tags",
               ok && exact && decays && played && outside && bad,
               Fmt("off0 %.4f off1 %.4f (hl 0.6: %.4f) outside %d", off0, off1, offSlow, outside ? 1 : 0));
        // next search transition goes back to the configured half-life
        MatcherInput go = Walk(0.0f, 0.0f);
        for (int i = 0; i < 120 && m.Diagnostics().transitions == tr0 + 1; ++i)
            m.Update(dt, go);
        Expect("(h2b) a search transition after ForcePlay restores the configured half-life",
               m.Diagnostics().transitions > tr0 + 1 && m.TransitionHalflife() == cfg.inertializationHalflife);
    }
    // (h3) suppressSearch: no searches, non-loop end holds; (h4) loop override end-of-clip + wrap
    {
        MatcherConfig c3 = cfg;
        c3.searchInterval = 1000.0f;
        auto run = [&](bool aOverride, bool aSuppress, int aFrames, int32_t& aLast, uint64_t& aSearches, uint64_t& aEoc, bool& aWrapped,
                       bool aVary = true) {
            BaseMatcher m;
            if (!m.Init(db, c3))
                return false;
            m.ForcePlay(int32_t(nc.firstFrame + nc.numFrames - 6));
            if (aOverride)
                m.SetLoopOverride(nl, true);
            const uint64_t s0 = m.Diagnostics().searches, e0 = m.Diagnostics().endOfClipSearches;
            aWrapped = false;
            int32_t prev = m.Diagnostics().frame;
            for (int i = 0; i < aFrames * 2; ++i)
            {
                MatcherInput in = aVary ? Walk(i % 40 < 20 ? 0.0f : 3.0f, i % 30 < 15 ? 4.0f : 0.0f) : Walk(0.0f, 0.0f);
                in.suppressSearch = aSuppress;
                m.Update(1.0f / 60.0f, in);
                const int32_t f = m.Diagnostics().frame;
                if (m.Diagnostics().clip == nl && f < prev)
                    aWrapped = true;
                prev = f;
            }
            aLast = m.Diagnostics().frame;
            aSearches = m.Diagnostics().searches - s0;
            aEoc = m.Diagnostics().endOfClipSearches - e0;
            return m.IsLoopClip(nl) == aOverride;
        };
        int32_t last = -1;
        uint64_t s = 0, e = 0;
        bool wrapped = false;
        bool ok = run(false, true, 240, last, s, e, wrapped);
        Expect("(h3) suppressSearch: 4 s of changing input -> 0 searches, the non-loop end holds its last frame",
               ok && s == 0 && e == 0 && last == int32_t(nc.firstFrame + nc.numFrames - 1), Fmt("searches %.0f last %.0f", double(s), double(last)));
        ok = run(true, true, int(nc.numFrames) * 2 + 300, last, s, e, wrapped);
        Expect("(h4) SetLoopOverride: the clip keeps playing past its end (wraps to firstFrame+1), no hold", ok && wrapped && s == 0);
        ok = run(false, false, 60, last, s, e, wrapped, false);
        const uint64_t eocPlain = e;
        bool ok2 = run(true, false, 60, last, s, e, wrapped, false);
        Expect("(h4b) IsLoopClip shared by the end-of-clip test: override -> 0 end-of-clip searches (plain clip > 0)",
               ok && ok2 && eocPlain > 0 && e == 0, Fmt("plain %.0f override %.0f", double(eocPlain), double(e)));
    }
    // (h5) SetRootZ subtracts the lerped value from the Hips p.z only (inertialization off: output = sample)
    {
        MatcherConfig c5 = cfg;
        c5.inertializationHalflife = 0.0f;
        BaseMatcher a, b;
        bool ok = a.Init(db, c5) && b.Init(db, c5);
        std::vector<float> rz(db.NumFrames());
        for (uint32_t f = 0; f < db.NumFrames(); ++f)
            rz[f] = 0.01f * float(f % 97);
        ok = ok && b.SetRootZ(rz) && !b.SetRootZ(std::span<const float>(rz.data(), rz.size() - 1));
        a.ForcePlay(int32_t(nc.firstFrame));
        b.ForcePlay(int32_t(nc.firstFrame));
        float maxErr = 0.0f;
        bool othersSame = true;
        const uint32_t h = c5.firstBodyBone;
        for (int i = 0; ok && i < int(nc.numFrames) * 2; ++i)
        {
            MatcherInput in = Walk(0.0f, 0.0f);
            in.suppressSearch = true;
            a.Update(1.0f / 73.0f, in);
            b.Update(1.0f / 73.0f, in);
            const MatcherDiagnostics& d = b.Diagnostics();
            const int32_t f0 = d.frame;
            const ClipRecord& cr = db.Clips()[size_t(d.clip)];
            const int32_t f1 = (uint32_t(f0) + 1 < cr.firstFrame + cr.numFrames) ? f0 + 1 : f0;
            const float al = f1 == f0 ? 0.0f : std::min(std::max(d.alpha, 0.0f), 1.0f);
            const float expect = a.Output().bones[h].p.z - (rz[size_t(f0)] + (rz[size_t(f1)] - rz[size_t(f0)]) * al);
            maxErr = std::max(maxErr, std::fabs(b.Output().bones[h].p.z - expect));
            for (uint32_t k = 0; k < db.NumBones(); ++k)
                if (k != h && std::memcmp(&a.Output().bones[k], &b.Output().bones[k], sizeof(BoneTransform)) != 0)
                    othersSame = false;
            othersSame = othersSame && a.Output().bones[h].p.x == b.Output().bones[h].p.x && a.Output().bones[h].p.y == b.Output().bones[h].p.y;
        }
        Expect("(h5) SetRootZ: Hips p.z = raw - lerp(root_z) (<= 1e-5 m), every other value bit-identical; wrong size rejected",
               ok && maxErr <= 1e-5f && othersSame, Fmt("maxErr %.7f", maxErr));
    }
    // (h6) queryBoneFrame: bone features of the latched frame, trajectory features unchanged, latch cleared by the search
    {
        BaseMatcher a, b;
        bool ok = a.Init(db, cfg) && b.Init(db, cfg);
        const int32_t qf = int32_t(nc.firstFrame + 3);
        MatcherInput in = Walk(0.0f, 1.0f);
        a.Update(dt, in);
        in.queryBoneFrame = qf;
        b.Update(dt, in);
        std::vector<float> want(db.NumFeatures());
        db.DenormalizeFeatures(db.Features(uint32_t(qf)), want);
        std::vector<uint8_t> traj(db.NumFeatures(), 0);
        for (const GroupRecord& g : db.Groups())
            if (g.kind == uint32_t(GroupKind::TrajectoryPosition) || g.kind == uint32_t(GroupKind::TrajectoryDirection))
                for (uint32_t j = g.firstFeature; j < g.firstFeature + g.numFeatures && j < traj.size(); ++j)
                    traj[j] = 1;
        float boneErr = 0.0f, trajErr = 0.0f;
        const auto qa = a.LastRawQuery(), qb = b.LastRawQuery();
        for (size_t j = 0; ok && j < want.size(); ++j)
        {
            if (traj[j])
                trajErr = std::max(trajErr, std::fabs(qa[j] - qb[j]));
            else
                boneErr = std::max(boneErr, std::fabs(qb[j] - want[j]));
        }
        Expect("(h6) queryBoneFrame: bone features from the given frame, trajectory features unchanged, latch cleared after the search",
               ok && b.Diagnostics().searchedThisUpdate && boneErr <= 1e-6f && trajErr == 0.0f && b.PendingQueryBoneFrame() == -1,
               Fmt("boneErr %.7f trajErr %.7f", boneErr, trajErr));
    }
}

static bool ReadFileBytes(const char* aPath, std::vector<uint8_t>& aOut)
{
    FILE* f = nullptr;
    if (fopen_s(&f, aPath, "rb") != 0 || !f)
        return false;
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    aOut.resize(n > 0 ? size_t(n) : 0);
    const bool ok = n > 0 && std::fread(aOut.data(), 1, aOut.size(), f) == aOut.size();
    std::fclose(f);
    return ok;
}

static void AirClipsRealTests(const char* aDbPath, const char* aAirzPath)
{
    std::vector<uint8_t> bytes, airz;
    if (!ReadFileBytes(aDbPath, bytes) || !ReadFileBytes(aAirzPath, airz))
    {
        std::printf("[NOT RUN] (h7) real air clips: %s / %s missing\n", aDbPath, aAirzPath);
        return;
    }
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + offsetof(Header, rigHash), 8);
    Database db;
    std::string err;
    const bool loaded = db.LoadFromMemory(bytes, rig, err);
    const auto sha = airtab::Sha256::Of(bytes);
    bytes.clear();
    bytes.shrink_to_fit();
    airtab::AirTable t;
    const bool parsed = loaded && t.Parse(airz, db, sha, err);
    const std::string tag = std::string("(h7 ") + (std::strrchr(aDbPath, '\\') ? std::strrchr(aDbPath, '\\') + 1 : aDbPath) + ") ";
    Expect((tag + "database + .airz load").c_str(), parsed, err);
    if (!parsed)
        return;
    MatcherConfig cfg;
    cfg.inertializationHalflife = 0.0f;
    cfg.searchInterval = 1000.0f;
    Expect((tag + "Hips bone = firstBodyBone").c_str(), t.HipsBone() == int32_t(cfg.firstBodyBone));
    float minZ = 1e9f, maxZ = -1e9f, maxVz = 0.0f;
    int32_t worstClip = -1, worstFrame = -1;
    float maxVz0 = 0.0f; // frame 0 -> 1 segment (reported only)
    uint32_t clipsChecked = 0;
    BaseMatcher m;
    bool ok = m.Init(db, cfg) && m.SetRootZ(t.RootZ());
    for (const airtab::ClipMeta& meta : t.Meta())
    {
        if (meta.status != airtab::ClipStatus::Applied || !ok)
            continue;
        ++clipsChecked;
        m.ForcePlay(int32_t(meta.firstFrame));
        float prevZ = m.Output().bones[cfg.firstBodyBone].p.z;
        for (uint32_t f = 1; f < meta.numFrames; ++f)
        {
            MatcherInput in;
            in.suppressSearch = true;
            m.Update(1.0f / 30.0f, in);
            const float z = m.Output().bones[cfg.firstBodyBone].p.z;
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);
            if (f == 1) // the first segment of a few motion land clips jumps (frame 0 data): the driver never enters at frame 0
                maxVz0 = std::max(maxVz0, std::fabs(z - prevZ) * 30.0f);
            else if (std::fabs(z - prevZ) * 30.0f > maxVz)
            {
                maxVz = std::fabs(z - prevZ) * 30.0f;
                worstClip = meta.clip;
                worstFrame = int32_t(f);
            }
            prevZ = z;
        }
    }
    std::printf("%sair clips %u (applied), corrected Hips z %.3f..%.3f m, max |dz/dt| %.2f m/s (%s frame %d; frame 0->1 segments %.2f m/s)\n", tag.c_str(),
                clipsChecked, double(minZ), double(maxZ), double(maxVz), worstClip >= 0 ? std::string(db.ClipName(uint32_t(worstClip))).c_str() : "-",
                worstFrame, double(maxVz0));
    Expect((tag + "every applied air clip: corrected Hips z in [0.30, 1.40] m and |dz/dt| <= 8 m/s from frame 1 on (heavy-land compression at touch ~6.8 m/s; an uncorrected root drop is 10-19 m/s)").c_str(),
           ok && clipsChecked >= 50 && minZ >= airtab::kHipsMin && maxZ <= airtab::kHipsMax && maxVz <= 8.0f,
           Fmt("clips %.0f z %.3f..%.3f vz %.2f", clipsChecked, minZ, maxZ, maxVz));
    // ground clip (walk / run loop): bit-identical with and without the table
    {
        int32_t g = -1;
        for (uint32_t c = 0; c < db.NumClips() && g < 0; ++c)
            if (db.ClipName(c).find("Run_Loop_F") != std::string_view::npos && !t.MetaOfClip(int32_t(c)))
                g = int32_t(c);
        BaseMatcher a, b;
        bool same = g >= 0 && a.Init(db, cfg) && b.Init(db, cfg) && b.SetRootZ(t.RootZ());
        float rzMax = 0.0f;
        if (same)
        {
            const ClipRecord& cr = db.Clips()[size_t(g)];
            for (uint32_t f = 0; f < cr.numFrames; ++f)
                rzMax = std::max(rzMax, std::fabs(t.RootZAt(cr.firstFrame + f)));
            a.ForcePlay(int32_t(cr.firstFrame));
            b.ForcePlay(int32_t(cr.firstFrame));
            for (int i = 0; i < 200 && same; ++i)
            {
                MatcherInput in;
                in.suppressSearch = true;
                a.Update(1.0f / 60.0f, in);
                b.Update(1.0f / 60.0f, in);
                same = PoseHash(a.Output()) == PoseHash(b.Output());
            }
        }
        Expect((tag + "ground Run_Loop_F: root_z 0 and output bit-identical with the table").c_str(), same && rzMax == 0.0f);
    }
    // fall loop: loop override (only when the table verified it) plays continuously
    if (t.FallLoopClip() >= 0)
    {
        const ClipRecord& cr = db.Clips()[size_t(t.FallLoopClip())];
        BaseMatcher f;
        bool okL = f.Init(db, cfg) && f.SetRootZ(t.RootZ()) && f.SetLoopOverride(t.FallLoopClip(), t.LoopOk());
        f.ForcePlay(int32_t(cr.firstFrame));
        int advanced = 0;
        int32_t prev = f.Diagnostics().frame;
        for (int i = 0; okL && i < 400; ++i)
        {
            MatcherInput in;
            in.suppressSearch = true;
            f.Update(1.0f / 30.0f, in);
            advanced += f.Diagnostics().frame != prev ? 1 : 0;
            prev = f.Diagnostics().frame;
        }
        Expect((tag + "Jump_Loop_Fall: LoopOk and > 300 frames of continuous playback with the override").c_str(),
               okL && t.LoopOk() && advanced >= 395, Fmt("advanced %.0f loopDeg %.3f", advanced, t.LoopMaxDeg()));
    }
    // ForcePlay Run loop -> F_Start_Run at liftoff - 2: inertializer offset < 0.15 m (table applied)
    {
        const airtab::PickResult pr = t.Pick(airtab::Kind::Start, airtab::Dir::F, airtab::Band::Run, airtab::Weight::None, airtab::Foot::L);
        int32_t run = -1;
        for (uint32_t c = 0; c < db.NumClips() && run < 0; ++c)
            if (db.ClipName(c).find("Run_Loop_F") != std::string_view::npos && !t.MetaOfClip(int32_t(c)))
                run = int32_t(c);
        MatcherConfig ci;
        ci.searchInterval = 1000.0f;
        BaseMatcher m2;
        bool okI = pr.clip >= 0 && run >= 0 && m2.Init(db, ci) && m2.SetRootZ(t.RootZ());
        float off = 1e9f;
        if (okI)
        {
            const airtab::ClipMeta& sm = t.Meta()[size_t(pr.meta)];
            m2.ForcePlay(int32_t(db.Clips()[size_t(run)].firstFrame + 10));
            for (int i = 0; i < 30; ++i)
            {
                MatcherInput in;
                in.suppressSearch = true;
                m2.Update(1.0f / 60.0f, in);
            }
            okI = m2.ForcePlay(int32_t(sm.firstFrame) + std::max(sm.liftoff - 2, 0), 0.15f);
            off = m2.Diagnostics().maxOffsetDist;
        }
        Expect((tag + "Run_Loop_F -> F_Start_Run at liftoff-2: inertializer max offset < 0.15 m").c_str(), okI && off < 0.15f,
               Fmt("offset %.3f m", off));
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_sim_test <database.utppmm>\n");
        return 2;
    }
    Database db;
    std::string err;
    const bool loaded = db.LoadFromFile(argv[1], ComputeRigHash(kBoneNames, kBoneParents), err);
    Expect("load database", loaded, err);
    if (!loaded)
        return 1;
    g_db = &db;
    for (uint32_t t = 0; t < db.NumTags(); ++t)
    {
        const std::string_view n = db.TagName(t);
        const uint64_t bit = 1ull << t;
        if (n == "PSD_Dense_Stand_Idles")
            g_tagIdle = bit;
        else if (n == "PSD_Dense_Stand_Walk_Loops")
            g_tagWalkLoop = bit;
        else if (n == "PSD_Dense_Stand_Run_Loops")
            g_tagRunLoop = bit;
        else if (n == "PSD_Dense_Stand_Walk_Starts")
            g_tagWalkStart = bit;
        else if (n == "PSD_Dense_Stand_Walk_Stops")
            g_tagWalkStop = bit;
        else if (n == "PSD_Dense_Stand_TurnInPlace")
            g_tagTurn = bit;
    }
    Expect("expected tags present", g_tagIdle && g_tagWalkLoop && g_tagRunLoop && g_tagWalkStart && g_tagWalkStop && g_tagTurn);
    const uint32_t nf = db.NumFeatures();
    const float fps = db.Fps();
    const float kDt60 = 1.0f / 60.0f;

    // trajectory group layout
    uint32_t tpFirst = 0, tdFirst = 0, nh = 0;
    int32_t horizons[4] = {};
    for (const GroupRecord& g : db.Groups())
    {
        if (g.kind == uint32_t(GroupKind::TrajectoryPosition))
        {
            tpFirst = g.firstFeature;
            nh = g.numHorizons;
            std::memcpy(horizons, g.horizonFrames, sizeof horizons);
        }
        else if (g.kind == uint32_t(GroupKind::TrajectoryDirection))
            tdFirst = g.firstFeature;
    }
    Expect("trajectory groups 3 horizons at 10/20/30 frames",
           nh == 3 && horizons[0] == 10 && horizons[1] == 20 && horizons[2] == 30 && tpFirst == 15 && tdFirst == 21);

    // ================================================================ (c0) feature convention vs baker
    // Integrate ROOT deltas in playback order (loops: N-1 -> 1) and feed the samples through the runtime's feature
    // function; must reproduce the baked trajectory features of every searchable frame.
    {
        std::vector<float> raw(nf);
        float maxPos = 0.0f, maxDir = 0.0f;
        uint32_t n = 0;
        for (uint32_t i = 0; i < db.NumFrames(); ++i)
        {
            if (!(db.FrameFlagsOf(i) & kFrameSearchable))
                continue;
            const ClipRecord& c = db.Clips()[size_t(db.ClipOfFrame(i))];
            float x = 0.0f, y = 0.0f, yaw = 0.0f;
            float fx[4], fy[4], fyaw[4];
            uint32_t f = i, k = 0;
            for (int32_t s = 1; s <= horizons[nh - 1]; ++s)
            {
                const RootDelta* r = db.Root(f);
                float rx = 0.0f, ry = 0.0f;
                math::Rot2(yaw, r->dx, r->dy, rx, ry);
                x += rx;
                y += ry;
                yaw += r->dyaw;
                const uint32_t local = f - c.firstFrame;
                f = (local + 1 < c.numFrames) ? f + 1 : ((c.flags & kClipLoop) ? c.firstFrame + 1 : f);
                if (s == horizons[k])
                {
                    fx[k] = x;
                    fy[k] = y;
                    fyaw[k] = yaw;
                    ++k;
                }
            }
            float pos[8], dir[8];
            TrajectoryFeaturesFromSamples(0.0f, 0.0f, 0.0f, fx, fy, fyaw, nh, pos, dir);
            db.DenormalizeFeatures(db.Features(i), raw);
            for (uint32_t j = 0; j < 2 * nh; ++j)
            {
                maxPos = std::max(maxPos, std::fabs(pos[j] - raw[tpFirst + j]));
                maxDir = std::max(maxDir, std::fabs(dir[j] - raw[tdFirst + j]));
            }
            ++n;
        }
        Expect("(c0) runtime trajectory feature formula + ROOT playback order reproduce baked features", maxPos < 2e-3f && maxDir < 2e-3f,
               Fmt("%.0f searchable frames, max |pos err| = %.2e m, max |dir err| = %.2e", n, maxPos, maxDir));
    }

    // ================================================================ (c1) straight walk loop: predicted trajectory vs baked
    {
        int32_t clip = -1;
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (db.ClipName(c) == "M_Neutral_Walk_Loop_F")
                clip = static_cast<int32_t>(c);
        Expect("(c1) clip M_Neutral_Walk_Loop_F present", clip >= 0);
        if (clip >= 0)
        {
            const uint32_t frame = db.Clips()[size_t(clip)].firstFrame + 30;
            std::vector<float> raw(nf), q(nf);
            db.DenormalizeFeatures(db.Features(frame), raw);
            // its own future root motion: average velocity over the last horizon (1 s)
            const float T = static_cast<float>(horizons[nh - 1]) / fps;
            const float vx = raw[tpFirst + 2 * (nh - 1)] / T, vy = raw[tpFirst + 2 * (nh - 1) + 1] / T;
            Trajectory tr;
            tr.Init(horizons, nh, fps, 0.27f, 0.27f);
            tr.SetState(vx, vy, 0.0f, 0.0f, 0.0f);
            TrajectoryInput ti;
            ti.desiredVelocity[0] = vx;
            ti.desiredVelocity[1] = vy;
            ti.desiredFacing[0] = 0.0f;
            ti.desiredFacing[1] = 1.0f;
            tr.Update(0.0f, ti);
            float maxPos = 0.0f, maxDir = 0.0f;
            for (uint32_t j = 0; j < 2 * nh; ++j)
            {
                maxPos = std::max(maxPos, std::fabs(tr.PositionFeatures()[j] - raw[tpFirst + j]));
                maxDir = std::max(maxDir, std::fabs(tr.DirectionFeatures()[j] - raw[tdFirst + j]));
                raw[tpFirst + j] = tr.PositionFeatures()[j];
                raw[tdFirst + j] = tr.DirectionFeatures()[j];
            }
            db.NormalizeQuery(raw, q);
            const SearchResult r = db.Search(q);
            float ownCost = 0.0f;
            for (uint32_t j = 0; j < nf; ++j)
                ownCost += (q[j] - db.Features(frame)[j]) * (q[j] - db.Features(frame)[j]);
            const int32_t rc = r.index >= 0 ? db.ClipOfFrame(static_cast<uint32_t>(r.index)) : -1;
            std::printf("  walk loop frame %u: v=(%.3f, %.3f) m/s, predicted vs baked trajectory max err pos=%.3f m dir=%.3f; "
                        "search -> %s[%d] cost=%.4f (own frame cost %.4f)\n",
                        frame, double(vx), double(vy), double(maxPos), double(maxDir), rc >= 0 ? ClipName(rc).c_str() : "-",
                        rc >= 0 ? r.index - int32_t(db.Clips()[size_t(rc)].firstFrame) : -1, double(r.cost), double(ownCost));
            Expect("(c1) steady straight walk: predicted trajectory close to the baked one", maxPos < 0.15f && maxDir < 0.1f,
                   Fmt("max pos err %.3f m, max dir err %.3f", maxPos, maxDir));
            Expect("(c1) query from own pose + predicted trajectory picks a walk loop with low cost",
                   rc >= 0 && (ClipTags(rc) & g_tagWalkLoop) && (db.Clips()[size_t(rc)].flags & kClipLoop) && r.cost < 0.25f,
                   Fmt("cost %.4f", r.cost));
        }
    }

    MatcherConfig baseCfg;

    // ================================================================ (a) determinism with variable dt
    auto runMixed = [&](const MatcherConfig& aCfg, double aSeconds, uint32_t aSeed) {
        RunStats s;
        BaseMatcher mm;
        if (!mm.Init(db, aCfg))
        {
            s.finite = false;
            return s;
        }
        uint32_t lcg = aSeed;
        double t = 0.0;
        while (t < aSeconds)
        {
            lcg = lcg * 1664525u + 1013904223u;
            const float dt = 1.0f / 120.0f + (1.0f / 30.0f - 1.0f / 120.0f) * float(lcg >> 8) / float(1u << 24);
            mm.Update(dt, MixedScript(t));
            t += dt;
            const Pose& p = mm.Output();
            s.hash = HashBytes(p.bones.data(), p.bones.size() * sizeof(BoneTransform), s.hash);
            const int32_t fr = mm.Diagnostics().frame;
            s.hash = HashBytes(&fr, sizeof fr, s.hash);
            s.finite = s.finite && PoseFinite(p);
        }
        s.transitions = mm.Diagnostics().transitions;
        s.searches = mm.Diagnostics().searches;
        return s;
    };
    {
        const RunStats r1 = runMixed(baseCfg, 40.0, 12345u);
        const RunStats r2 = runMixed(baseCfg, 40.0, 12345u);
        Expect("(a) determinism: two runs (40 s, variable dt 1/120..1/30) bitwise-identical output",
               r1.hash == r2.hash && r1.finite && r2.finite && r1.transitions == r2.transitions,
               Fmt("hash 0x%08x%08x, searches %.0f, transitions %.0f", double(uint32_t(r1.hash >> 32)), double(uint32_t(r1.hash)),
                   double(r1.searches), double(r1.transitions)));
    }

    // ================================================================ (b) idle
    {
        BaseMatcher mm;
        const bool ok = mm.Init(db, baseCfg);
        bool allIdle = ok, finite = ok;
        float maxSpeed = 0.0f;
        std::string firstNonIdle;
        for (int i = 0; ok && i < 600; ++i)
        {
            mm.Update(kDt60, Walk(0.0f, 0.0f));
            const MatcherDiagnostics& d = mm.Diagnostics();
            if (!(ClipTags(d.clip) & g_tagIdle))
            {
                if (allIdle)
                    firstNonIdle = ClipName(d.clip) + " at t=" + std::to_string((i + 1) * kDt60);
                allIdle = false;
            }
            finite = finite && PoseFinite(mm.Output());
            maxSpeed = std::max(maxSpeed, std::hypot(d.simVelocity[0], d.simVelocity[1]));
        }
        const MatcherDiagnostics& d = mm.Diagnostics();
        Expect("(b) idle: 10 s zero input stays in Idle-tagged clips, finite output", ok && allIdle && finite && maxSpeed == 0.0f,
               (ok ? "clip " + ClipName(d.clip) : std::string("init failed")) + ", searches " + std::to_string(d.searches) +
                   ", transitions " + std::to_string(d.transitions) + (firstNonIdle.empty() ? "" : ", first non-idle: " + firstNonIdle));
    }

    // ================================================================ (c2)+(d) start / walk / run / stop, with and without inertialization
    struct Trace
    {
        std::vector<int32_t> clip;
        std::vector<uint8_t> transitioned;
        std::vector<Pose> poses;
        std::vector<float> consumedSpeed;
        uint64_t transitions = 0, searches = 0, forced = 0, endOfClip = 0;
        bool finite = true;
    };
    auto runStart = [&](const MatcherConfig& aCfg, float aWalk, float aRun, Trace& aOut) {
        BaseMatcher mm;
        if (!mm.Init(db, aCfg))
            return false;
        const int n = int(kStartScriptEnd * 60.0 + 0.5);
        aOut.poses.reserve(size_t(n));
        for (int i = 0; i < n; ++i)
        {
            const double t = i * double(kDt60);
            mm.Update(kDt60, Walk(0.0f, StartScriptSpeed(t, aWalk, aRun)));
            const MatcherDiagnostics& d = mm.Diagnostics();
            aOut.clip.push_back(d.clip);
            aOut.transitioned.push_back(d.transitionedThisUpdate ? 1 : 0);
            aOut.poses.push_back(mm.Output());
            aOut.consumedSpeed.push_back(std::hypot(d.consumedRoot[0], d.consumedRoot[1]) / kDt60);
            aOut.finite = aOut.finite && PoseFinite(mm.Output());
        }
        aOut.transitions = mm.Diagnostics().transitions;
        aOut.searches = mm.Diagnostics().searches;
        aOut.forced = mm.Diagnostics().forcedSearches;
        aOut.endOfClip = mm.Diagnostics().endOfClipSearches;
        return true;
    };
    // Walk / run speeds of the data (motion loops): asserted run. The brief's nominal speeds (1.5 / 3.75 m/s) lie between
    // the walk and run loop speeds of this database; that run is reported (and checked for finiteness / walk entry) only.
    const float dataWalk = ClipRootSpeed(db, "M_Neutral_Walk_Loop_F");
    const float dataRun = ClipRootSpeed(db, "M_Neutral_Run_Loop_F");
    std::printf("  database loop root speeds: walk %.3f m/s, run %.3f m/s\n", double(dataWalk), double(dataRun));
    MatcherConfig offCfg = baseCfg;
    offCfg.inertializationHalflife = 0.0f;
    constexpr double kDeg = 180.0 / 3.14159265358979323846;

    auto scenario = [&](const char* aLabel, float aWalk, float aRun, bool aAssertTags) {
        Trace on, off;
        const bool okOn = runStart(baseCfg, aWalk, aRun, on);
        const bool okOff = runStart(offCfg, aWalk, aRun, off);
        if (!okOn || !okOff)
        {
            Expect("(c2) init", false);
            return;
        }
        std::printf("  [%s] walk %.2f m/s, run %.2f m/s; clip sequence (t: clip):\n", aLabel, double(aWalk), double(aRun));
        int32_t prev = -2;
        for (size_t i = 0; i < on.clip.size(); ++i)
            if (on.clip[i] != prev)
            {
                std::printf("    %6.3f s  %s\n", double(i + 1) * kDt60, ClipName(on.clip[i]).c_str());
                prev = on.clip[i];
            }
        auto frac = [&](double t0, double t1, uint64_t mask, bool loopOnly) {
            int a = 0, n = 0;
            for (size_t i = 0; i < on.clip.size(); ++i)
            {
                const double t = double(i + 1) * kDt60;
                if (t >= t0 && t < t1)
                {
                    ++n;
                    const bool isLoop = (db.Clips()[size_t(on.clip[i])].flags & kClipLoop) != 0;
                    a += ((ClipTags(on.clip[i]) & mask) && (!loopOnly || isLoop)) ? 1 : 0;
                }
            }
            return n ? double(a) / n : 0.0;
        };
        auto avgSpeed = [&](double t0, double t1) {
            double s = 0.0;
            int n = 0;
            for (size_t i = 0; i < on.consumedSpeed.size(); ++i)
            {
                const double t = double(i + 1) * kDt60;
                if (t >= t0 && t < t1)
                {
                    s += on.consumedSpeed[i];
                    ++n;
                }
            }
            return n ? s / n : 0.0;
        };
        int32_t firstMove = -1;
        for (size_t i = size_t(2.0 / kDt60); i < on.clip.size(); ++i)
            if (on.transitioned[i])
            {
                firstMove = on.clip[i];
                break;
            }
        const double idle0 = frac(0.0, 2.0, g_tagIdle, false);
        const double walkLate = frac(4.5, 6.0, g_tagWalkLoop, true);
        const double runLate = frac(8.0, 10.0, g_tagRunLoop, true);
        const double stopAny = frac(10.0, 12.5, g_tagWalkStop | g_tagWalkLoop | g_tagIdle, false);
        const double idleEnd = frac(14.0, 15.0, g_tagIdle | g_tagWalkStop, false);
        std::printf("    fractions: idle[0,2)=%.2f walk LOOP clip[4.5,6)=%.2f run LOOP clip[8,10)=%.2f stop|walk|idle[10,12.5)=%.2f "
                    "idle|stop[14,15)=%.2f\n",
                    idle0, walkLate, runLate, stopAny, idleEnd);
        std::printf("    consumed anim root speed (m/s): walk[4.5,6)=%.2f run[8,10)=%.2f idle[14,15)=%.2f | searches=%llu forced=%llu "
                    "endOfClip=%llu transitions=%llu\n",
                    avgSpeed(4.5, 6.0), avgSpeed(8.0, 10.0), avgSpeed(14.0, 15.0), static_cast<unsigned long long>(on.searches),
                    static_cast<unsigned long long>(on.forced), static_cast<unsigned long long>(on.endOfClip),
                    static_cast<unsigned long long>(on.transitions));
        const std::string L = std::string("(c2) [") + aLabel + "] ";
        Expect((L + "idle before input").c_str(), idle0 >= 0.99, Fmt("%.2f", idle0));
        Expect((L + "first transition after the walk command enters a walk start or walk loop").c_str(),
               firstMove >= 0 && (ClipTags(firstMove) & (g_tagWalkStart | g_tagWalkLoop)), firstMove >= 0 ? ClipName(firstMove) : std::string("-"));
        if (aAssertTags)
        {
            const double walkTagged = frac(4.5, 6.0, g_tagWalkLoop | g_tagWalkStart, false);
            const double walkSpeed = avgSpeed(4.5, 6.0), runSpeed = avgSpeed(8.0, 10.0);
            Expect((L + "walk phase: walk start/loop clips at the commanded speed (+-10%), reaches a walk LOOP clip").c_str(),
                   walkTagged >= 0.99 && frac(2.0, 6.0, g_tagWalkLoop, true) > 0.0 && std::fabs(walkSpeed - aWalk) <= 0.1 * aWalk,
                   Fmt("walk-tagged %.2f, loop clip %.2f of [4.5,6) s, consumed %.2f m/s", walkTagged, walkLate, walkSpeed));
            Expect((L + "run phase settles in a run LOOP clip at the commanded speed (+-10%)").c_str(),
                   runLate >= 0.8 && std::fabs(runSpeed - aRun) <= 0.1 * aRun, Fmt("%.2f of [8,10) s, consumed %.2f m/s", runLate, runSpeed));
            Expect((L + "stop phase uses stop / walk / idle clips").c_str(), stopAny >= 0.5, Fmt("%.2f of [10,12.5) s", stopAny));
            Expect((L + "ends idle (or finishing a stop)").c_str(), idleEnd >= 0.99, Fmt("%.2f of [14,15) s", idleEnd));
        }
        Expect((L + "finite output").c_str(), on.finite && off.finite);

        // (d) per-update change of every bone local rotation / position (bones 2..72), inertialized vs hard cut
        struct Jumps
        {
            double maxAllRot = 0, maxTransRot = 0, maxAllPos = 0, maxTransPos = 0, maxNaturalRot = 0;
            size_t maxAllBone = 0, maxAllUpdate = 0, maxTransBone = 0;
            std::vector<double> rot;
        };
        auto measure = [&](const Trace& tr) {
            Jumps j;
            for (size_t i = 1; i < tr.poses.size(); ++i)
            {
                double mr = 0, mp = 0;
                size_t mb = 0;
                for (size_t b = 2; b < tr.poses[i].bones.size(); ++b)
                {
                    const BoneTransform& a = tr.poses[i - 1].bones[b];
                    const BoneTransform& c = tr.poses[i].bones[b];
                    const double ang = double(math::QuatAngleBetween(a.q, c.q));
                    if (ang > mr)
                    {
                        mr = ang;
                        mb = b;
                    }
                    const math::Vec3 d = c.p - a.p;
                    mp = std::max(mp, double(std::sqrt(math::Dot(d, d))));
                }
                j.rot.push_back(mr);
                if (mr > j.maxAllRot)
                {
                    j.maxAllRot = mr;
                    j.maxAllBone = mb;
                    j.maxAllUpdate = i;
                }
                j.maxAllPos = std::max(j.maxAllPos, mp);
                if (tr.transitioned[i])
                {
                    if (mr > j.maxTransRot)
                    {
                        j.maxTransRot = mr;
                        j.maxTransBone = mb;
                    }
                    j.maxTransPos = std::max(j.maxTransPos, mp);
                }
                else
                    j.maxNaturalRot = std::max(j.maxNaturalRot, mr);
            }
            return j;
        };
        const Jumps jOn = measure(on), jOff = measure(off);
        auto bone = [&](size_t b) { return std::string(db.BoneName(uint32_t(b))); };
        std::printf("    per-update (60 Hz) max bone-local change, bones 2..72:\n"
                    "      inertialized : all rot %.2f deg (%s, t=%.3f s) pos %.4f m | transition updates rot %.2f deg (%s) pos %.4f m | "
                    "non-transition rot %.2f deg | p99 rot %.2f deg\n",
                    jOn.maxAllRot * kDeg, bone(jOn.maxAllBone).c_str(), double(jOn.maxAllUpdate + 1) * kDt60, jOn.maxAllPos,
                    jOn.maxTransRot * kDeg, bone(jOn.maxTransBone).c_str(), jOn.maxTransPos, jOn.maxNaturalRot * kDeg,
                    Percentile(jOn.rot, 0.99) * kDeg);
        std::printf("      hard cut     : all rot %.2f deg (%s, t=%.3f s) pos %.4f m | transition updates rot %.2f deg (%s) pos %.4f m | "
                    "non-transition rot %.2f deg | p99 rot %.2f deg\n",
                    jOff.maxAllRot * kDeg, bone(jOff.maxAllBone).c_str(), double(jOff.maxAllUpdate + 1) * kDt60, jOff.maxAllPos,
                    jOff.maxTransRot * kDeg, bone(jOff.maxTransBone).c_str(), jOff.maxTransPos, jOff.maxNaturalRot * kDeg,
                    Percentile(jOff.rot, 0.99) * kDeg);
        const std::string D = std::string("(d) [") + aLabel + "] ";
        Expect((D + "same transition sequence with and without inertialization").c_str(),
               on.clip == off.clip && on.transitioned == off.transitioned);
        Expect((D + "inertialized transition jumps smaller than hard cuts").c_str(),
               jOn.maxTransRot < jOff.maxTransRot && jOn.maxTransPos < jOff.maxTransPos);
        // A transition must not change any bone faster than the animation data itself does between two output updates.
        Expect((D + "inertialized transition-update change <= largest natural (non-transition) change of the clips").c_str(),
               jOn.maxTransRot <= jOff.maxNaturalRot, Fmt("%.2f deg vs %.2f deg", jOn.maxTransRot * kDeg, jOff.maxNaturalRot * kDeg));
    };
    scenario("data speeds", dataWalk, dataRun, true);
    scenario("brief speeds", 1.5f, 3.75f, false);

    // ================================================================ (e) performance + no allocations
    {
        BaseMatcher mm;
        const bool ok = mm.Init(db, baseCfg);
        std::vector<double> tUpd, tSearch;
        tUpd.reserve(3600);
        tSearch.reserve(3600);
        bool finite = ok;
        g_allocs = 0;
        for (int i = 0; ok && i < 3600; ++i)
        {
            const MatcherInput in = MixedScript(i * double(kDt60));
            g_countAllocs = true;
            const auto t0 = std::chrono::steady_clock::now();
            mm.Update(kDt60, in);
            const auto t1 = std::chrono::steady_clock::now();
            g_countAllocs = false;
            tUpd.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            if (mm.Diagnostics().searchedThisUpdate)
                tSearch.push_back(mm.Diagnostics().lastSearchMicros);
            finite = finite && PoseFinite(mm.Output());
        }
        const uint64_t allocs = g_allocs.load();
        std::printf("  Update() us over 60 s @ 60 Hz (3600 calls): median=%.2f p99=%.2f max=%.2f | search us (%zu searches): median=%.2f "
                    "p99=%.2f max=%.2f | transitions=%llu\n",
                    Percentile(tUpd, 0.5), Percentile(tUpd, 0.99), Percentile(tUpd, 1.0), tSearch.size(), Percentile(tSearch, 0.5),
                    Percentile(tSearch, 0.99), Percentile(tSearch, 1.0), static_cast<unsigned long long>(mm.Diagnostics().transitions));
        Expect("(e) no heap allocations inside Update()", ok && allocs == 0, std::to_string(allocs) + " allocations");
        Expect("(e) median Update() < 300 us", ok && Percentile(tUpd, 0.5) < 300.0, Fmt("%.2f us", Percentile(tUpd, 0.5)));
        Expect("(e) finite output over 60 s mixed script", finite);
    }

    // ================================================================ (f) tag filter + filtered search exactness
    {
        MatcherConfig cfg = baseCfg;
        cfg.allowedTagMask = g_tagIdle | g_tagWalkLoop | g_tagWalkStart | g_tagWalkStop | g_tagTurn; // no run loops
        BaseMatcher mm;
        const bool ok = mm.Init(db, cfg);
        bool neverRun = ok;
        for (int i = 0; ok && i < 600; ++i)
        {
            mm.Update(kDt60, Walk(0.0f, i < 60 ? 0.0f : 3.75f));
            neverRun = neverRun && !(ClipTags(mm.Diagnostics().clip) & g_tagRunLoop);
        }
        Expect("(f) tag allow-mask without Run_Loops never plays a run loop at run speed", neverRun,
               ok ? "ends in " + ClipName(mm.Diagnostics().clip) : std::string("init failed"));

        std::mt19937 rng(0xF117E5u);
        std::normal_distribution<float> gauss(0.0f, 1.0f);
        std::uniform_int_distribution<uint32_t> pick(0, db.NumFrames() - 1);
        uint32_t mism = 0, outside = 0;
        const auto ranges = mm.AllowedRanges();
        for (int k = 0; ok && k < 3000; ++k)
        {
            const auto src = db.Features(pick(rng));
            std::vector<float> q(src.begin(), src.end());
            for (float& v : q)
                v += 0.5f * gauss(rng);
            const int32_t curr = (k % 2) ? static_cast<int32_t>(pick(rng)) : -1;
            const SearchResult a = db.SearchInRanges(q, ranges, curr, 10, (k % 3) ? 0.0f : 0.2f);
            const SearchResult b = db.SearchBruteForceInRanges(q, ranges, curr, 10, (k % 3) ? 0.0f : 0.2f);
            if (a.index != b.index || std::memcmp(&a.cost, &b.cost, sizeof(float)) != 0)
                ++mism;
            if (curr < 0 && a.index >= 0 && !(ClipTags(db.ClipOfFrame(static_cast<uint32_t>(a.index))) & cfg.allowedTagMask))
                ++outside;
        }
        Expect("(f) filtered AABB search == filtered brute force, results inside the filter", ok && mism == 0 && outside == 0,
               std::to_string(mism) + " mismatches, " + std::to_string(outside) + " outside, " + std::to_string(ranges.size()) + " ranges");
    }

    // ================================================================ (g) robustness: non-finite input / dt
    {
        BaseMatcher mm;
        bool ok = mm.Init(db, baseCfg);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();
        for (int i = 0; ok && i < 300; ++i)
        {
            MatcherInput in = Walk(0.0f, 1.5f);
            if (i % 7 == 0)
                in.desiredVelocity[0] = nan;
            if (i % 11 == 0)
                in.desiredFacing[1] = inf;
            if (i % 13 == 0)
                in.yawRate = nan, in.facingMode = FacingMode::YawRate;
            const float dt = (i % 17 == 0) ? nan : (i % 19 == 0) ? 5.0f : (i % 23 == 0) ? -1.0f : kDt60;
            mm.Update(dt, in);
            ok = ok && PoseFinite(mm.Output());
        }
        MatcherConfig bad = baseCfg;
        bad.searchInterval = 0.0f;
        BaseMatcher mm2;
        const bool rejected = !mm2.Init(db, bad);
        bad = baseCfg;
        bad.allowedTagMask = 0;
        const bool rejectedEmpty = !mm2.Init(db, bad);
        Database empty;
        const bool rejectedDb = !mm2.Init(empty, baseCfg);
        Expect("(g) NaN/inf input and NaN/negative/huge dt keep the output finite; invalid config / empty filter / empty db rejected",
               ok && rejected && rejectedEmpty && rejectedDb);
    }

    // AIR CLIPS stage: matcher runtime APIs (synthetic on this database) + real air clips on <db> <airz> argument pairs
    AirClipsSyntheticTests(db);
    for (int i = 2; i + 1 < argc; i += 2)
        AirClipsRealTests(argv[i], argv[i + 1]);
    if (argc < 4)
        std::printf("[NOT RUN] (h7) real air clips: no <db> <airz> arguments\n");

    std::printf("SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

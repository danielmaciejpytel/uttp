// Offline tests of the Phase 4b game integration helpers (src/mm/GameIntegration.hpp), not shipped.
// Usage: utpp_mm_integration_test [database.utppmm]   Exit code 0 = all checks passed.
// With a database, the bone map is also checked against the database's own names (synthetic game rig built from them).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/GameIntegration.hpp"
#include "../src/mm/GamePoseCapture.hpp"
#include "../src/mm/NativeShootTransfer.hpp"
#include "../src/mm/BaseMatcher.hpp"

using namespace utpp::mm;
using namespace utpp::mm::integ;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d);
    return buf;
}

static bool Near(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// ---------------------------------------------------------------------------------------------- yaw convention
static void TestYaw()
{
    float maxErr = 0.0f;
    // yaw 0 faces +Y; +90 deg (CCW) faces -X; -90 faces +X; 180 faces -Y.
    struct Case
    {
        float deg, fx, fy;
    } cases[] = {{0, 0, 1}, {90, -1, 0}, {-90, 1, 0}, {180, 0, -1}, {45, -0.70710678f, 0.70710678f}};
    for (const Case& c : cases)
    {
        float fx, fy;
        ForwardFromYaw(c.deg * kDegToRad, fx, fy);
        maxErr = std::fmax(maxErr, std::fmax(std::fabs(fx - c.fx), std::fabs(fy - c.fy)));
        const float back = YawFromForward(c.fx, c.fy);
        maxErr = std::fmax(maxErr, std::fabs(WrapPi(back - c.deg * kDegToRad)));
    }
    Expect("yaw convention: forward = (-sin, cos), YawFromForward inverse", maxErr < 1e-6f, Fmt("max err %.3g", maxErr));
    Expect("YawFromForward degenerate -> 0", YawFromForward(0, 0) == 0.0f && YawFromForward(NAN, 1) == 0.0f);

    // World velocity along the character's forward -> (0, |v|); along right -> (|v|, 0); for random yaws.
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> U(-3.14159f, 3.14159f), S(0.1f, 8.0f);
    float err = 0.0f;
    for (int i = 0; i < 1000; ++i)
    {
        const float yaw = U(rng), speed = S(rng);
        float fx, fy;
        ForwardFromYaw(yaw, fx, fy);
        const float rx = fy, ry = -fx; // right = (cos, sin) = (fy, -fx)
        float cx, cy;
        WorldToCharacter(fx * speed, fy * speed, yaw, cx, cy);
        err = std::fmax(err, std::fabs(cx) + std::fabs(cy - speed));
        WorldToCharacter(rx * speed, ry * speed, yaw, cx, cy);
        err = std::fmax(err, std::fabs(cx - speed) + std::fabs(cy));
        // length preserved for an arbitrary vector
        const float vx = U(rng), vy = U(rng);
        WorldToCharacter(vx, vy, yaw, cx, cy);
        err = std::fmax(err, std::fabs(std::sqrt(cx * cx + cy * cy) - std::sqrt(vx * vx + vy * vy)));
    }
    Expect("world->character: forward->(0,v), right->(v,0), length kept (1000 random yaws)", err < 1e-5f, Fmt("max err %.3g", err));
    float cx, cy;
    WorldToCharacter(0.0f, 2.0f, 90.0f * kDegToRad, cx, cy); // facing -X, moving +Y world = moving to the character's right
    Expect("world->character: yaw 90, world +Y velocity = character right", Near(cx, 2.0f, 1e-6f) && Near(cy, 0.0f, 1e-6f),
           Fmt("(%.6f, %.6f)", cx, cy));
    Expect("WrapPi", Near(WrapPi(3.5f * kPiF), -0.5f * kPiF, 1e-5f) && Near(WrapPi(-kPiF), kPiF, 1e-6f));
}

// ---------------------------------------------------------------------------------------------- gating + weight
static void TestGateAndWeight()
{
    GateInput g;
    g.mmEnabled = g.dbLoaded = g.mapped = g.tppActive = g.matcherReady = true;
    g.locoState = kLocoStand;
    Expect("gate open for stand + all conditions", ComputeGate(g) == 0);
    GateInput h = g;
    h.locoState = kLocoCrouch;
    Expect("gate open: crouch allowed by default (0.2.3)", ComputeGate(h) == 0);
    h = g;
    h.locoState = kLocoOther;
    Expect("gate closed: non-locomotion state", ComputeGate(h) == kGateLocoState);
    h = g;
    h.locoState = 77;
    Expect("gate closed: out-of-range state", ComputeGate(h) == kGateLocoState);
    h = g;
    h.mmEnabled = false;
    h.tppActive = false;
    h.fault = true;
    Expect("gate reasons accumulate", ComputeGate(h) == (kGateMMDisabled | kGateTppInactive | kGateFault));
    Expect("gate default-constructed: disabled|db|map|tpp|state(0)|matcher",
           ComputeGate(GateInput{}) == (kGateMMDisabled | kGateDbNotLoaded | kGateNotMapped | kGateTppInactive | kGateLocoState | kGateMatcherNotReady));

    WeightFollower w;
    float t = 0.0f, prev = 0.0f, w50 = 0.0f;
    bool monotonic = true;
    while (t < 0.25f - 1e-6f)
    {
        const float v = w.Update(1.0f, 1.0f / 120.0f, 0.25f);
        t += 1.0f / 120.0f;
        monotonic = monotonic && v >= prev;
        prev = v;
        if (std::fabs(t - 0.125f) < 0.5f / 120.0f)
            w50 = v;
    }
    w.Update(1.0f, 1.0f / 120.0f, 0.25f);
    Expect("weight: monotonic smoothstep fade-in, 0.5 at half time, 1 after blend time",
           monotonic && Near(w50, 0.5f, 0.02f) && w.Weight() == 1.0f, Fmt("w(half)=%.4f w(end)=%.4f", w50, w.Weight()));
    for (int i = 0; i < 40; ++i)
        w.Update(0.0f, 1.0f / 120.0f, 0.25f);
    Expect("weight: fade-out to exactly 0", w.Weight() == 0.0f);
    w.Update(1.0f, 0.01f, 0.0f);
    Expect("weight: blend time 0 snaps", w.Weight() == 1.0f);
    w.Update(1.0f, NAN, 0.25f);
    Expect("weight: non-finite dt ignored", w.Weight() == 1.0f);
}


// 0.3.2 (Phase 5.2): PSM block gate signal and the gate fader (hard fade-out, reopen hold), 60 fps.
static void TestGateFader()
{
    GateInput g;
    g.mmEnabled = g.dbLoaded = g.mapped = g.tppActive = g.matcherReady = true;
    g.locoState = kLocoStand;
    GateInput h = g;
    h.psmBlock = kPsmVehicle;
    Expect("psm block: vehicle closes the gate with kGatePsmBlock only", ComputeGate(h) == kGatePsmBlock);
    h.psmBlock = kPsmNoBlackboard;
    Expect("psm block: missing blackboard closes the gate (fail closed)", ComputeGate(h) == kGatePsmBlock);
    h.psmBlock = kPsmDetailed | kPsmScene;
    h.locoState = kLocoOther;
    Expect("psm block accumulates with the loco-state reason", ComputeGate(h) == (kGatePsmBlock | kGateLocoState));

    const float dt = 1.0f / 60.0f;
    const FadeTimes ft{0.25f, 0.12f, 0.10f};
    auto framesUntil = [&](GateFader& f, uint32_t gate, bool wantOne, int maxFrames, bool* monotonic) {
        float prev = f.Weight();
        bool mono = true;
        for (int i = 1; i <= maxFrames; ++i)
        {
            const float w = f.Update(gate, dt, ft);
            mono = mono && (wantOne ? w >= prev : w <= prev);
            prev = w;
            if (wantOne ? w == 1.0f : w == 0.0f)
            {
                if (monotonic)
                    *monotonic = mono;
                return i;
            }
        }
        if (monotonic)
            *monotonic = mono;
        return -1;
    };

    GateFader f;
    bool mono = false;
    const int nIn = framesUntil(f, 0, true, 100, &mono);
    Expect("fader: first fade-in has no hold, ~0.25 s, monotonic", mono && nIn >= 14 && nIn <= 16, Fmt("frames=%.0f", nIn));
    // vehicle mount / scene / workspot: hard fade-out
    const int nOutHard = framesUntil(f, kGatePsmBlock, false, 100, &mono);
    Expect("fader: PSM block fades out in hardOut 0.12 s, monotonic", mono && nOutHard >= 7 && nOutHard <= 8,
           Fmt("frames=%.0f", nOutHard));
    // block clears: weight stays 0 during the reopen hold, then fades in
    int zeroFrames = 0;
    for (int i = 0; i < 5; ++i) // 0.083 s < reopenHold
        zeroFrames += f.Update(0, dt, ft) == 0.0f ? 1 : 0;
    Expect("fader: weight 0 during the reopen hold (0.10 s)", zeroFrames == 5 && f.HoldPending());
    const int nReopen = framesUntil(f, 0, true, 100, &mono);
    Expect("fader: after the hold fade-in completes in ~0.25 s", mono && nReopen >= 15 && nReopen <= 17,
           Fmt("frames=%.0f", nReopen));
    // soft closure (MM switched off by the user): normal blend time
    const int nOutSoft = framesUntil(f, kGateMMDisabled, false, 100, &mono);
    Expect("fader: soft closure fades out in blendTime 0.25 s", mono && nOutSoft >= 14 && nOutSoft <= 16,
           Fmt("frames=%.0f", nOutSoft));
    Expect("fader: soft closure sets no reopen hold", (framesUntil(f, 0, true, 100, nullptr) <= 16) && !f.HoldPending());
    // dash/jump/landing (loco state) with a short stand window in between: no fade-in flicker
    framesUntil(f, kGateLocoState, false, 100, nullptr);
    float maxW = 0.0f;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        for (int i = 0; i < 3; ++i) // 0.05 s "stand" (< reopen hold)
            maxW = std::fmax(maxW, f.Update(0, dt, ft));
        for (int i = 0; i < 6; ++i)
            maxW = std::fmax(maxW, f.Update(kGateLocoState, dt, ft));
    }
    Expect("fader: open windows shorter than the hold keep MM at 0 (no flicker)", maxW == 0.0f, Fmt("maxW=%.3f", maxW));
    // hard fade-out keeps its speed when the reason turns soft mid-fade
    framesUntil(f, 0, true, 100, nullptr);
    f.Update(kGatePsmBlock, dt, ft);
    const int nMixed = framesUntil(f, kGateMMDisabled, false, 100, nullptr);
    Expect("fader: hard fade-out continues at hardOut after the reason changes", nMixed >= 6 && nMixed <= 7,
           Fmt("frames=%.0f", nMixed));
    // stale feed (locomotion PSM stopped, e.g. vehicle/scene without feed) is a hard closure with a hold
    framesUntil(f, 0, true, 100, nullptr);
    const int nStale = framesUntil(f, kGateStaleFeed, false, 100, nullptr);
    f.Update(0, dt, ft);
    Expect("fader: stale feed fades out in hardOut and sets the hold", nStale >= 7 && nStale <= 8 && f.HoldPending());
    // zero times snap; non-finite dt ignored
    GateFader z;
    const FadeTimes zt{0.0f, 0.0f, 0.0f};
    z.Update(0, dt, zt);
    const float zIn = z.Weight();
    z.Update(kGatePsmBlock, dt, zt);
    const float zOut = z.Weight();
    z.Update(0, dt, zt);
    Expect("fader: zero times snap (in, out, no hold)", zIn == 1.0f && zOut == 0.0f && z.Weight() == 1.0f);
    z.Update(kGatePsmBlock, NAN, ft);
    Expect("fader: non-finite dt does not move the weight", z.Weight() == 1.0f);
}

// ---------------------------------------------------------------------------------------------- bone map
static void TestBoneMap(const Database* aDb)
{
    // synthetic: db Root, Trajectory, Hips, Spine, LeftUpLeg ; game rig shuffled with extra bones
    const char* dbNames[] = {"Root", "Trajectory", "Hips", "Spine", "LeftUpLeg"};
    const int32_t dbPar[] = {-1, 0, 1, 2, 2};
    const char* gameNames[] = {"Root", "Extra", "Trajectory", "Hips", "LeftUpLeg", "Spine", "Extra2"};
    const int16_t gamePar[] = {-1, 0, 0, 2, 3, 3, 5};
    std::vector<uint64_t> dh, gh;
    for (const char* n : dbNames)
        dh.push_back(CNameHash(n));
    for (const char* n : gameNames)
        gh.push_back(CNameHash(n));
    BoneMapResult r = BuildBoneMap(dh, dbPar, gh, gamePar, 2);
    Expect("bone map: by name, Root/Trajectory never mapped",
           r.ok && r.mapped == 3 && r.gameIndex[0] == -1 && r.gameIndex[1] == -1 && r.gameIndex[2] == 3 && r.gameIndex[3] == 5 &&
               r.gameIndex[4] == 4,
           Fmt("mapped=%g hips=%g spine=%g", r.mapped, r.gameIndex[2], r.gameIndex[3]));

    std::vector<uint64_t> gh2 = gh;
    gh2[5] = CNameHash("NotSpine");
    r = BuildBoneMap(dh, dbPar, gh2, gamePar, 2);
    Expect("bone map: missing name fails closed (no bone mapped)", !r.ok && r.missing == 1 && r.firstMissing == 3 && r.mapped == 0 &&
                                                                       r.gameIndex[2] == -1);
    int16_t badPar[] = {-1, 0, 0, 2, 3, 4, 5}; // Spine parented to LeftUpLeg
    r = BuildBoneMap(dh, dbPar, gh, badPar, 2);
    Expect("bone map: parent mismatch fails closed", !r.ok && r.parentMismatch == 1 && r.firstParentMismatch == 3 && r.mapped == 0);
    r = BuildBoneMap(dh, dbPar, gh, {}, 2);
    Expect("bone map: without parents (no check) still maps by name", r.ok && r.mapped == 3);
    Expect("CNameHash: None/empty = 0, FNV-1a 64", CNameHash("None") == 0 && CNameHash("") == 0 &&
                                                       CNameHash("a") == 0xAF63DC4C8601EC8Cull);

    if (!aDb)
        return;
    // Real database names against a synthetic 619-bone game rig that contains them at shuffled positions.
    const uint32_t nb = aDb->NumBones();
    std::vector<uint64_t> dbh(nb);
    std::vector<int32_t> dbp(nb);
    for (uint32_t b = 0; b < nb; ++b)
    {
        dbh[b] = CNameHash(aDb->BoneName(b));
        dbp[b] = aDb->BoneParent(b);
    }
    const uint32_t ng = 619;
    std::vector<uint64_t> g(ng);
    std::vector<int16_t> gp(ng, -1);
    std::vector<int32_t> pos(nb);
    for (uint32_t i = 0; i < ng; ++i)
        g[i] = CNameHash("filler_" + std::to_string(i));
    for (uint32_t b = 0; b < nb; ++b)
        pos[b] = static_cast<int32_t>((b * 7 + 13) % ng);
    for (uint32_t b = 0; b < nb; ++b)
    {
        g[size_t(pos[b])] = dbh[b];
        gp[size_t(pos[b])] = static_cast<int16_t>(dbp[b] >= 0 ? pos[size_t(dbp[b])] : -1);
    }
    r = BuildBoneMap(dbh, dbp, g, gp, 2);
    bool idx = r.ok;
    for (uint32_t b = 2; b < nb && idx; ++b)
        idx = r.gameIndex[b] == pos[b];
    Expect("bone map: database 73 bones vs synthetic 619-bone rig", idx && r.mapped == nb - 2,
           Fmt("mapped=%g of %g", r.mapped, nb - 2));
}

// ---------------------------------------------------------------------------------------------- seqlock buffer
static void TestSeqlock()
{
    static SeqlockBuffer<PoseSnapshot> buf;
    static PoseSnapshot s{}, out{};
    Expect("seqlock: read before publish fails", !buf.Read(out));
    s.weight = 0.75f;
    s.numBones = 3;
    s.rig = 0x1234;
    s.sequence = 1;
    for (uint32_t i = 0; i < kMaxMMBones * 7; ++i)
        s.pose[i] = static_cast<float>(i);
    buf.Publish(s);
    Expect("seqlock: publish/consume roundtrip bitwise", buf.Read(out) && std::memcmp(&out, &s, sizeof s) == 0);
    s.sequence = 2;
    s.weight = 0.5f;
    buf.Publish(s);
    Expect("seqlock: latest value wins", buf.Read(out) && out.sequence == 2 && out.weight == 0.5f);
    buf.TestBeginWriteOnLatest();
    Expect("seqlock: slot being written -> read fails (no torn data)", !buf.Read(out));
    buf.TestEndWriteOnLatest();
    Expect("seqlock: readable again after write completes", buf.Read(out) && out.sequence == 2);

    // Concurrent stress: writer publishes snapshots whose every word encodes the sequence; readers verify consistency.
    static SeqlockBuffer<PoseSnapshot> cbuf;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> reads{0}, torn{0}, inconsistent{0};
    auto reader = [&]
    {
        PoseSnapshot r{};
        while (!stop.load(std::memory_order_relaxed))
        {
            if (!cbuf.Read(r))
            {
                torn.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            reads.fetch_add(1, std::memory_order_relaxed);
            const float expect = static_cast<float>(r.sequence % 100000);
            bool ok = r.weight == expect;
            for (uint32_t i = 0; i < kMaxMMBones * 7 && ok; i += 37)
                ok = r.pose[i] == expect;
            if (!ok)
                inconsistent.fetch_add(1, std::memory_order_relaxed);
        }
    };
    std::thread r1(reader), r2(reader);
    PoseSnapshot w{};
    for (uint64_t seq = 1; seq <= 200000; ++seq)
    {
        w.sequence = seq;
        const float v = static_cast<float>(seq % 100000);
        w.weight = v;
        for (float& p : w.pose)
            p = v;
        cbuf.Publish(w);
    }
    stop = true;
    r1.join();
    r2.join();
    Expect("seqlock: 200000 publishes vs 2 concurrent readers, 0 inconsistent snapshots",
           inconsistent.load() == 0 && reads.load() > 0,
           Fmt("reads=%g failedReads=%g inconsistent=%g", double(reads.load()), double(torn.load()), double(inconsistent.load())));
}

// ---------------------------------------------------------------------------------------------- blend
struct GameBone
{
    float p[4], q[4], s[4];
};
static_assert(sizeof(GameBone) == 48);

static void NormalizeQ(float* q)
{
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int k = 0; k < 4; ++k)
        q[k] /= n;
}

static void TestBlend()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> U(-1.0f, 1.0f);
    const uint32_t count = 619;
    std::vector<GameBone> game(count), orig;
    for (GameBone& b : game)
    {
        for (int k = 0; k < 4; ++k)
        {
            b.p[k] = U(rng);
            b.q[k] = U(rng);
            b.s[k] = 1.0f + 0.1f * U(rng);
        }
        NormalizeQ(b.q);
    }
    static PoseSnapshot snap{};
    snap.numBones = 73;
    snap.firstBone = 2;
    for (uint32_t b = 0; b < kMaxMMBones; ++b)
        snap.gameIndex[b] = b < 73 ? static_cast<int32_t>((b * 5 + 3) % count) : -1;
    snap.gameIndex[0] = 0; // must still not be written (below firstBone)
    snap.gameIndex[1] = 1;
    snap.gameIndex[72] = static_cast<int32_t>(count + 5); // out of range -> skipped
    for (uint32_t b = 0; b < 73; ++b)
    {
        float* m = &snap.pose[size_t(b) * 7];
        for (int k = 0; k < 7; ++k)
            m[k] = U(rng);
        NormalizeQ(m + 3);
        // half of the bones: MM quaternion in the opposite hemisphere of the game's
        if (b % 2 == 0 && snap.gameIndex[b] >= 0 && uint32_t(snap.gameIndex[b]) < count)
        {
            const float* gq = game[size_t(snap.gameIndex[b])].q;
            const float d = gq[0] * m[3] + gq[1] * m[4] + gq[2] * m[5] + gq[3] * m[6];
            if (d > 0.0f)
                for (int k = 3; k < 7; ++k)
                    m[k] = -m[k];
        }
    }
    orig = game;

    uint32_t n = BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), count, snap, 0.0f);
    Expect("blend: weight 0 -> game pose bitwise identical", n == 0 && std::memcmp(game.data(), orig.data(), count * 48) == 0);
    n = BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), count, snap, -1.0f);
    Expect("blend: negative weight -> untouched", n == 0 && std::memcmp(game.data(), orig.data(), count * 48) == 0);

    n = BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), count, snap, 1.0f);
    float maxP = 0.0f, maxQ = 0.0f;
    bool scaleKept = true, othersKept = true, signAligned = true;
    std::vector<uint8_t> touched(count, 0);
    for (uint32_t b = 2; b < 72; ++b)
    {
        const uint32_t g = uint32_t(snap.gameIndex[b]);
        touched[g] = 1;
        const float* m = &snap.pose[size_t(b) * 7];
        for (int k = 0; k < 3; ++k)
            maxP = std::fmax(maxP, std::fabs(game[g].p[k] - m[k]));
        const float d = game[g].q[0] * m[3] + game[g].q[1] * m[4] + game[g].q[2] * m[5] + game[g].q[3] * m[6];
        maxQ = std::fmax(maxQ, 1.0f - std::fabs(d));
        const float dg = game[g].q[0] * orig[g].q[0] + game[g].q[1] * orig[g].q[1] + game[g].q[2] * orig[g].q[2] + game[g].q[3] * orig[g].q[3];
        signAligned = signAligned && dg >= 0.0f;
        scaleKept = scaleKept && std::memcmp(game[g].s, orig[g].s, 16) == 0 && game[g].p[3] == orig[g].p[3];
    }
    for (uint32_t i = 0; i < count; ++i)
        if (!touched[i])
            othersKept = othersKept && std::memcmp(&game[i], &orig[i], 48) == 0;
    Expect("blend: weight 1 -> MM pose (pos max err, 1-|dot|)", maxP <= 1e-6f && maxQ <= 1e-6f && n == 70,
           Fmt("written=%g maxPos=%.3g maxQ=%.3g", n, maxP, maxQ));
    Expect("blend: weight 1 quaternion sign aligned to the game hemisphere", signAligned);
    Expect("blend: scale and pos.w kept; Root/Trajectory/unmapped/out-of-range bones untouched", scaleKept && othersKept &&
                                                                                                     std::memcmp(&game[0], &orig[0], 48) == 0 &&
                                                                                                     std::memcmp(&game[1], &orig[1], 48) == 0);

    // weight 0.5: shortest path. Rotation angle game->result == result->mm (nlerp midpoint), position = midpoint.
    game = orig;
    BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), count, snap, 0.5f);
    float maxMid = 0.0f, maxAngDiff = 0.0f, maxNorm = 0.0f;
    bool shortest = true;
    for (uint32_t b = 2; b < 72; ++b)
    {
        const uint32_t g = uint32_t(snap.gameIndex[b]);
        const float* m = &snap.pose[size_t(b) * 7];
        for (int k = 0; k < 3; ++k)
            maxMid = std::fmax(maxMid, std::fabs(game[g].p[k] - 0.5f * (orig[g].p[k] + m[k])));
        auto absdot = [](const float* a, const float* c) { return std::fabs(a[0] * c[0] + a[1] * c[1] + a[2] * c[2] + a[3] * c[3]); };
        const float a1 = 2.0f * std::acos(std::fmin(1.0f, absdot(orig[g].q, game[g].q)));
        const float a2 = 2.0f * std::acos(std::fmin(1.0f, absdot(game[g].q, m + 3)));
        const float total = 2.0f * std::acos(std::fmin(1.0f, absdot(orig[g].q, m + 3)));
        maxAngDiff = std::fmax(maxAngDiff, std::fabs(a1 - a2));
        shortest = shortest && (a1 + a2) <= total + 1e-3f; // on the short arc
        maxNorm = std::fmax(maxNorm, std::fabs(std::sqrt(game[g].q[0] * game[g].q[0] + game[g].q[1] * game[g].q[1] +
                                                          game[g].q[2] * game[g].q[2] + game[g].q[3] * game[g].q[3]) - 1.0f));
    }
    Expect("blend: weight 0.5 -> position midpoint, rotation halfway on the SHORT arc (sign fix), unit quats",
           maxMid < 1e-6f && maxAngDiff < 1e-3f && shortest && maxNorm < 1e-6f,
           Fmt("pos %.3g angDiff %.3g rad norm %.3g", maxMid, maxAngDiff, maxNorm));

    // non-finite MM bone -> skipped
    game = orig;
    static PoseSnapshot bad{};
    bad = snap;
    bad.pose[size_t(5) * 7 + 3] = NAN;
    BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), count, bad, 1.0f);
    const uint32_t g5 = uint32_t(snap.gameIndex[5]);
    Expect("blend: non-finite MM bone left untouched", std::memcmp(&game[g5], &orig[g5], 48) == 0);
    Expect("blend: null bone buffer -> 0", BlendIntoRecords(nullptr, count, snap, 1.0f) == 0);
}

// ---------------------------------------------------------------------------------------------- end-to-end (db)
static void TestEndToEnd(const Database& aDb)
{
    BaseMatcher m;
    MatcherConfig cfg;
    Expect("e2e: matcher init", m.Init(aDb, cfg));
    // A player at yaw 90 deg walking world -X at 2 m/s = character forward 2 m/s.
    float cx, cy;
    WorldToCharacter(-2.0f, 0.0f, 90.0f * kDegToRad, cx, cy);
    MatcherInput in;
    in.desiredVelocity[0] = cx;
    in.desiredVelocity[1] = cy;
    in.facingMode = FacingMode::YawRate;
    in.yawRate = 0.0f;
    for (int i = 0; i < 180; ++i)
        m.Update(1.0f / 60.0f, in);
    float vx, vy;
    m.GetTrajectory().LocalVelocity(vx, vy);
    const std::string_view clip = aDb.ClipName(uint32_t(m.Diagnostics().clip));
    Expect("e2e: world -X at yaw 90 -> forward walk, simulated velocity ~(0, 2)",
           Near(cx, 0.0f, 1e-5f) && Near(cy, 2.0f, 1e-5f) && Near(vx, 0.0f, 0.05f) && Near(vy, 2.0f, 0.05f),
           Fmt("char (%.3f, %.3f) sim (%.3f, %.3f)", cx, cy, vx, vy) + " clip=" + std::string(clip));

    static PoseSnapshot snap{};
    snap.numBones = aDb.NumBones();
    snap.firstBone = 2;
    snap.weight = 1.0f;
    for (uint32_t b = 0; b < kMaxMMBones; ++b)
        snap.gameIndex[b] = b < aDb.NumBones() ? static_cast<int32_t>(b) : -1;
    const auto& bones = m.Output().bones;
    for (uint32_t b = 0; b < aDb.NumBones(); ++b)
    {
        float* d = &snap.pose[size_t(b) * 7];
        d[0] = bones[b].p.x, d[1] = bones[b].p.y, d[2] = bones[b].p.z;
        d[3] = bones[b].q.x, d[4] = bones[b].q.y, d[5] = bones[b].q.z, d[6] = bones[b].q.w;
    }
    std::vector<GameBone> game(aDb.NumBones());
    for (GameBone& g : game)
        g = GameBone{{0, 0, 0, 1}, {0, 0, 0, 1}, {1, 1, 1, 1}};
    const uint32_t n = BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), aDb.NumBones(), snap, 1.0f);
    float maxP = 0.0f;
    for (uint32_t b = 2; b < aDb.NumBones(); ++b)
        maxP = std::fmax(maxP, std::fabs(game[b].p[2] - bones[b].p.z));
    Expect("e2e: matcher output -> snapshot -> records at weight 1", n == aDb.NumBones() - 2 && maxP <= 1e-6f,
           Fmt("written %g maxErr %.3g", n, maxP));
}

// ---------------------------------------------------------------------------------------------- 0.6.2 upper-body mask / hold
static void TestUpperMask(const Database* aDb)
{
    // names of the rig (the player skeleton listing) when no database is given
    static const char* kRig[73] = {"Root", "Trajectory", "Hips", "Spine", "Spine1", "Spine2", "Spine3", "LeftUpLeg", "RightUpLeg", "LeftLeg",
                                   "RightLeg", "LeftFoot", "RightFoot", "LeftHeel", "RightHeel", "LeftToeBase", "RightToeBase", "Neck",
                                   "Neck1", "Head", "LeftEye", "RightEye", "LeftShoulder", "RightShoulder", "LeftArm", "RightArm",
                                   "LeftForeArm", "RightForeArm", "LeftHand", "RightHand", "LeftHandEnd", "RightHandEnd", "WeaponLeft",
                                   "WeaponRight", "LeftInHandThumb", "LeftHandThumb1", "LeftHandThumb2", "LeftInHandIndex",
                                   "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3", "LeftInHandMiddle", "LeftHandMiddle1",
                                   "LeftHandMiddle2", "LeftHandMiddle3", "LeftInHandRing", "LeftHandRing1", "LeftHandRing2",
                                   "LeftHandRing3", "LeftInHandPinky", "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3",
                                   "RightInHandThumb", "RightHandThumb1", "RightHandThumb2", "RightInHandIndex", "RightHandIndex1",
                                   "RightHandIndex2", "RightHandIndex3", "RightInHandMiddle", "RightHandMiddle1", "RightHandMiddle2",
                                   "RightHandMiddle3", "RightInHandRing", "RightHandRing1", "RightHandRing2", "RightHandRing3",
                                   "RightInHandPinky", "RightHandPinky1", "RightHandPinky2", "RightHandPinky3", "Torso_Dummy"};
    const uint32_t nb = aDb ? aDb->NumBones() : 72u;
    auto nameAt = [&](uint32_t b) -> std::string_view { return aDb ? aDb->BoneName(b) : std::string_view(kRig[b]); };
    const UpperMask m = BuildUpperMask(nb, 2, nameAt);
    bool lowerOk = true, upperOk = true;
    int lowerNamed = 0, upperNamed = 0, w26 = -1, w28 = -1;
    std::string lowerList;
    for (uint32_t b = 2; b < nb; ++b)
    {
        const std::string_view n = nameAt(b);
        const bool lower = n == "Hips" || n == "Spine" || n.find("UpLeg") != std::string_view::npos ||
                           ((n.find("Leg") != std::string_view::npos || n.find("Foot") != std::string_view::npos ||
                             n.find("Heel") != std::string_view::npos || n.find("ToeBase") != std::string_view::npos) &&
                            n.find("Hand") == std::string_view::npos);
        if (lower)
        {
            lowerOk = lowerOk && !m.Upper(b);
            ++lowerNamed;
            lowerList += std::string(n) + " ";
        }
        else
        {
            upperOk = upperOk && m.Upper(b);
            ++upperNamed;
        }
        if (n == "WeaponLeft")
            w26 = int(b);
        if (n == "WeaponRight")
            w28 = int(b);
    }
    Expect("mask: kMaskUpperFromGame - Hips, Spine, legs, feet (12) by MM; Spine1..3, neck, head, arms, hands, fingers, weapons by the game",
           m.ok && lowerOk && upperOk && m.lower == 12u && lowerNamed == 12 && int(m.upper) == upperNamed && w26 >= 0 && w28 >= 0 &&
               m.Upper(uint32_t(w26)) && m.Upper(uint32_t(w28)) && !m.Upper(0) && !m.Upper(1),
           Fmt("lower %g upper %g w26 %g w28 %g", m.lower, m.upper, w26, w28) + " [" + lowerList + "]");
    if (aDb)
        Expect("mask: database bone indices = rig (WeaponLeft 26, WeaponRight 28)", w26 == 26 && w28 == 28, Fmt("%g %g", w26, w28));

    const UpperMask action = BuildWeaponActionMask(nb, 2, nameAt);
    bool actionArms = true, actionTorso = true, actionWeapons = true;
    int leftHand = -1, rightHand = -1, leftFingerRoot = -1, rightFingerRoot = -1, spine3 = -1, head = -1;
    for (uint32_t b = 2; b < nb; ++b)
    {
        const std::string_view n = nameAt(b);
        const bool selected = action.Upper(b);
        if (n == "LeftHand") { leftHand = static_cast<int>(b); actionArms = actionArms && selected; }
        if (n == "RightHand") { rightHand = static_cast<int>(b); actionArms = actionArms && selected; }
        if (n == "LeftInHandThumb") { leftFingerRoot = static_cast<int>(b); actionArms = actionArms && selected; }
        if (n == "RightInHandThumb") { rightFingerRoot = static_cast<int>(b); actionArms = actionArms && selected; }
        if (n == "Spine3") { spine3 = static_cast<int>(b); actionTorso = actionTorso && !selected; }
        if (n == "Head") { head = static_cast<int>(b); actionTorso = actionTorso && !selected; }
        if (n == "WeaponLeft" || n == "WeaponRight") actionWeapons = actionWeapons && selected;
    }
    Expect("weapon action mask: both hands and weapon attachments come from the same game action pose",
           action.ok && leftHand >= 0 && rightHand >= 0 && leftFingerRoot >= 0 && rightFingerRoot >= 0 && actionArms && actionWeapons &&
               action.Upper(uint32_t(w26)) && action.Upper(uint32_t(w28)));
    Expect("weapon action mask: Spine3 and Head stay on MM/Neutral to reduce FPP torso deformation in TPP",
           action.ok && spine3 >= 0 && head >= 0 && actionTorso && !action.Upper(uint32_t(spine3)) && !action.Upper(uint32_t(head)));
    Expect("weapon action mask: malformed rig fails closed", !BuildWeaponActionMask(4, 2, [](uint32_t b) -> std::string_view {
               return b == 2 ? "Hips" : b == 3 ? "WeaponRight" : "Foo";
           }).ok);

    // fail closed: no Spine1 (unknown rig) -> no mask
    const UpperMask bad = BuildUpperMask(4, 2, [](uint32_t b) -> std::string_view { return b == 2 ? "Hips" : "Foo"; });
    Expect("mask: unknown rig (no Spine1) -> not ok, no bits (full close as 0.6.1)", !bad.ok && bad.bits[0] == 0u && bad.bits[1] == 0u);

    // blend with the mask (identity map)
    static PoseSnapshot snap{};
    snap = PoseSnapshot{};
    snap.numBones = nb;
    snap.firstBone = 2;
    for (uint32_t b = 0; b < kMaxMMBones; ++b)
        snap.gameIndex[b] = b < nb ? int32_t(b) : -1;
    for (uint32_t b = 0; b < nb; ++b)
    {
        float* p = &snap.pose[size_t(b) * 7];
        p[0] = 1.0f, p[1] = 2.0f, p[2] = 3.0f, p[3] = 0.0f, p[4] = 0.0f, p[5] = 0.70710678f, p[6] = 0.70710678f;
    }
    for (uint32_t i = 0; i < kMaskWords; ++i)
        snap.upperBits[i] = m.bits[i];
    std::vector<GameBone> game(nb), orig(nb);
    for (GameBone& g : orig)
        g = GameBone{{0, 0, 0, 1}, {0, 0, 0, 1}, {1, 1, 1, 1}};
    auto run = [&](float aScale, float aWeight, uint32_t& aWritten) {
        game = orig;
        snap.upperScale = aScale;
        aWritten = BlendIntoRecords(reinterpret_cast<uint8_t*>(game.data()), nb, snap, aWeight);
    };
    uint32_t n = 0;
    run(0.0f, 1.0f, n);
    bool upperKept = true, lowerWritten = true;
    for (uint32_t b = 2; b < nb; ++b)
    {
        const bool same = std::memcmp(&game[b], &orig[b], sizeof(GameBone)) == 0;
        if (m.Upper(b))
            upperKept = upperKept && same;
        else
            lowerWritten = lowerWritten && game[b].p[0] == 1.0f && game[b].p[2] == 3.0f;
    }
    Expect("mask blend: upperScale 0 - upper bones bitwise game pose (incl. WeaponLeft / WeaponRight), lower bones MM, count = lower",
           upperKept && lowerWritten && n == m.lower, Fmt("written %g", n));
    run(0.5f, 1.0f, n);
    Expect("mask blend: upperScale 0.5 at weight 1 - upper bone midpoint, lower bone MM",
           Near(game[uint32_t(w28)].p[0], 0.5f, 1e-6f) && Near(game[2].p[0], 1.0f, 1e-6f) && n == nb - 2, Fmt("%g", game[uint32_t(w28)].p[0]));
    run(NAN, 1.0f, n);
    Expect("mask blend: non-finite upperScale = full body (0.6.1)", n == nb - 2 && Near(game[uint32_t(w28)].p[0], 1.0f, 1e-6f));
    snap.upperBits[0] = snap.upperBits[1] = snap.upperBits[2] = 0u;
    run(0.0f, 1.0f, n);
    Expect("mask blend: no upper bits = full body whatever the scale (default snapshot = 0.6.1)", n == nb - 2);
    const PoseSnapshot def{};
    Expect("mask blend: snapshot defaults upperScale 1, no bits", def.upperScale == 1.0f && def.upperBits[0] == 0u);
}

static void TestUpperMaskDecision()
{
    UpperMaskInput in;
    in.meleeActive = true;
    in.gate = kGateMelee;
    UpperMaskDecision d = DecideUpperMask(in);
    Expect("mask decision: melee gate alone -> mask (melee), kGateMelee covered", d.reasons == kMaskStateMelee && d.covered == kGateMelee);
    // fists block as logged: melee + AimWalk soft bits + UpperBody Aim
    in.gate = kGateMelee | kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = kPsmDetailAimWalk;
    in.upperBody = kPsmUpperBodyAim;
    d = DecideUpperMask(in);
    Expect("mask decision: fists block (0x510, AimWalk, UpperBody Aim) -> mask melee|aim, all covered",
           d.reasons == (kMaskStateMelee | kMaskStateAim) && d.covered == in.gate);
    in.gate |= kGateTppInactive;
    Expect("mask decision: + TPP off (aim in FPP) -> no mask", DecideUpperMask(in).reasons == 0u);
    in = UpperMaskInput{};
    in.upperBody = kPsmUpperBodyAim;
    in.gate = kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = 18; // Jump
    Expect("mask decision: aim during a jump (detail Jump) -> no mask", DecideUpperMask(in).reasons == 0u);
    in.psmDetail = kPsmDetailAimWalk;
    in.vz = 3.0f;
    Expect("mask decision: AimWalk with |vz| 3 -> no mask", DecideUpperMask(in).reasons == 0u);
    in.vz = 0.0f;
    in.states = kMaskStateMelee;
    Expect("mask decision: states = melee only -> aim not masked", DecideUpperMask(in).reasons == 0u);
    in = UpperMaskInput{};
    in.exitWindow = true;
    in.gate = kGateLocoState;
    Expect("mask decision: exit window, locoState 0 -> mask (exit)", DecideUpperMask(in).reasons == kMaskStateExit);
    in.gate = 0;
    Expect("mask decision: exit window, open gate -> mask (exit), nothing covered", DecideUpperMask(in).reasons == kMaskStateExit &&
                                                                                       DecideUpperMask(in).covered == 0u);
    in.gate = kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = 14; // Fall
    Expect("mask decision: exit window + Fall -> no mask", DecideUpperMask(in).reasons == 0u);
    in.psm = kPsmVehicle;
    in.gate = kGatePsmBlock;
    Expect("mask decision: exit window + vehicle bit -> no mask", DecideUpperMask(in).reasons == 0u);
    in = UpperMaskInput{};
    in.meleeActive = true;
    for (uint32_t g : {uint32_t(kGateOverSpeed), uint32_t(kGateStaleFeed), uint32_t(kGateFault), uint32_t(kGatePsmBlock)})
    {
        in.gate = kGateMelee | g;
        in.psm = g == kGatePsmBlock ? uint32_t(kPsmScene) : 0u;
        Expect(Fmt("mask decision: melee + gate 0x%g -> no mask (full close)", double(g)).c_str(), DecideUpperMask(in).reasons == 0u);
    }
    // fade
    UpperMaskBlend b;
    float w = b.Update(true, false, false, 1.0f / 60.0f, kMaskFadeSeconds);
    Expect("mask fade: nothing visible -> snaps to the mask", w == 1.0f);
    b.Reset();
    int frames = 0;
    while (b.Update(true, false, true, 1.0f / 60.0f, kMaskFadeSeconds) < 1.0f && frames < 100)
        ++frames;
    Expect("mask fade: in over 0.15 s (9 frames at 60 Hz)", frames + 1 == 9, Fmt("%g frames", frames + 1));
    const float held = b.Update(false, false, true, 0.1f, kMaskFadeSeconds);
    Expect("mask fade: not wanted with the gate closed -> held", held == 1.0f);
    b.Update(false, true, true, 0.075f, kMaskFadeSeconds);
    Expect("mask fade: not wanted, gate open -> fades out (half way after 0.075 s)", Near(b.Progress(), 0.5f, 1e-5f));
    b.Update(false, true, false, 0.0f, kMaskFadeSeconds);
    Expect("mask fade: not visible, not wanted -> 0", b.Weight() == 0.0f);
    // AIR-held v1 pose has effective gate open even when raw PSM still reports Jump/Fall.
    // Only an explicit AIR ownership signal may release a stale upper-body mask during that raw closure.
    b.Reset();
    b.Update(true, false, false, 0.0f, kMaskFadeSeconds);
    const float airHeld = b.Update(false, false, true, 0.075f, kMaskFadeSeconds);
    Expect("mask fade: raw gate closed without AIR ownership still holds previous upper mask", airHeld == 1.0f);
    const float airHalf = b.Update(false, false, true, 0.075f, kMaskFadeSeconds, true);
    Expect("mask fade: AIR-owned base effective gate releases stale upper mask smoothly", Near(airHalf, 0.5f, 1e-5f));
    const float airFull = b.Update(false, false, true, 0.075f, kMaskFadeSeconds, true);
    Expect("mask fade: AIR-owned base reaches full MM upper body", airFull == 0.0f);
    b.Update(true, false, false, 0.0f, kMaskFadeSeconds);
    Expect("mask fade: legacy raw gate holds upper mask after AIR handback", b.Update(false, false, true, 0.075f, kMaskFadeSeconds) == 1.0f);
}

static void TestHoldEligible()
{
    HoldInput h;
    h.tpp = true;
    h.poseOk = true;
    Expect("hold: open gate, TPP -> eligible", HoldEligible(h));
    for (uint32_t g : {uint32_t(kGateMMDisabled), uint32_t(kGateNotMapped), uint32_t(kGateLocoState), uint32_t(kGateStaleFeed),
                       uint32_t(kGateOverSpeed), uint32_t(kGateMelee)})
    {
        h.gate = g;
        Expect(Fmt("hold: gate 0x%g held", double(g)).c_str(), HoldEligible(h));
    }
    h.gate = kGatePsmBlock;
    for (uint32_t p : {uint32_t(kPsmPhotoOrMenu), uint32_t(kPsmReaction), uint32_t(kPsmNoBlackboard), uint32_t(kPsmDetailed)})
    {
        h.psm = p;
        h.psmDetail = p == kPsmDetailed ? 18 : -1;
        Expect(Fmt("hold: PSM 0x%g held (menu, stagger, no blackboard, jump)", double(p)).c_str(), HoldEligible(h));
    }
    for (uint32_t p : {uint32_t(kPsmVehicle), uint32_t(kPsmScene), uint32_t(kPsmSwimming), uint32_t(kPsmWorkspot), uint32_t(kPsmTakedown),
                       uint32_t(kPsmCarry), uint32_t(kPsmVitals)})
    {
        h.psm = p;
        Expect(Fmt("hold: PSM 0x%g released", double(p)).c_str(), !HoldEligible(h));
    }
    h.psm = kPsmDetailed;
    for (int det : {10, 11, 12, 13, 29, 31})
    {
        h.psmDetail = det;
        Expect(Fmt("hold: detail %g (ladder / knockdown / felled) released", det).c_str(), !HoldEligible(h));
    }
    h = HoldInput{};
    h.tpp = true;
    h.poseOk = true;
    h.gate = kGateTppInactive;
    Expect("hold: TPP off gate -> released", !HoldEligible(h));
    h.gate = kGateFault;
    Expect("hold: fault -> released", !HoldEligible(h));
    h.gate = kGateMMDisabled;
    h.tpp = false;
    Expect("hold: not in TPP -> released", !HoldEligible(h));
    h.tpp = true;
    h.poseOk = false;
    Expect("hold: no valid MM pose -> released", !HoldEligible(h));
    h.poseOk = true;
    h.on = false;
    Expect("hold: switch off -> released", !HoldEligible(h));
    // fader helpers
    GateFader f;
    FadeTimes t;
    f.Update(0, 1.0f, t);
    f.Update(kGatePsmBlock, 1.0f, t); // closed, hold armed
    f.ClearHold();
    const float w1 = f.Update(0, 1.0f / 60.0f, t);
    Expect("fader: ClearHold - the next open sample fades in at once (no reopen hold)", w1 > 0.0f && !f.HoldPending());
    f.Update(kGatePsmBlock, 1.0f, t);
    f.Fill();
    Expect("fader: Fill -> weight 1, target 1, no hold", f.Weight() == 1.0f && f.Target() == 1.0f && !f.HoldPending());
}

// Vehicle batch W1 (CAR-1 / CAR-2 / MOTO-6): the vehicle exit window rules of HoldEligible and DecideUpperMask, as a table.
// Outside the window every case equals the pre-batch rule (checked against an explicit copy of it).
static bool LegacyHoldEligible(const HoldInput& aIn)
{
    if (!aIn.on || !aIn.tpp || !aIn.poseOk)
        return false;
    if ((aIn.gate & ~kHoldGateMask) != 0)
        return false;
    if ((aIn.psm & kPsmReleaseMask) != 0)
        return false;
    if ((aIn.psm & kPsmDetailed) != 0 && PsmDetailReleases(aIn.psmDetail))
        return false;
    return aIn.release == 0;
}

static void TestExitWindowRules()
{
    // ---- outside the window: identical to the pre-batch rule on a grid of inputs (exitLatchedPsm ignored without the window)
    {
        int diff = 0, n = 0;
        const uint32_t gates[] = {0u, kGateLocoState, kGateStaleFeed, kGateOverSpeed, kGatePsmBlock, kGateTppInactive, kGateFault, kGateMelee};
        const uint32_t psms[] = {0u, kPsmVehicle, kPsmScene, kPsmSwimming, kPsmWorkspot, kPsmTakedown, kPsmCarry, kPsmVitals, kPsmDetailed,
                                 kPsmWorkspot | kPsmDetailed, kPsmPhotoOrMenu, kPsmReaction};
        for (uint32_t g : gates)
            for (uint32_t p : psms)
                for (int det : {-1, 0, 6, 10, 14, 18, 23, 24, 25, 29, 31})
                    for (uint32_t latch : {0u, uint32_t(kPsmWorkspot)})
                        for (bool tpp : {false, true})
                        {
                            HoldInput h;
                            h.tpp = tpp;
                            h.poseOk = true;
                            h.gate = g;
                            h.psm = p;
                            h.psmDetail = det;
                            h.exitLatchedPsm = latch; // no window: must be ignored
                            ++n;
                            diff += HoldEligible(h) != LegacyHoldEligible(h) ? 1 : 0;
                        }
        Expect("exit window: outside the window HoldEligible == the pre-batch rule on every grid case", diff == 0, Fmt("%g of %g differ", diff, n));
    }
    // ---- inside the window
    HoldInput h;
    h.tpp = true;
    h.poseOk = true;
    h.exitWindow = true;
    h.exitLatchedPsm = kPsmWorkspot;
    h.gate = kGatePsmBlock | kGateLocoState;
    h.psm = kPsmWorkspot | kPsmDetailed;
    h.psmDetail = 0;
    Expect("exit window: latched exit workspot + NotInBase 0 -> held (CAR-1 T1)", HoldEligible(h));
    h.exitLatchedPsm = 0;
    Expect("exit window: a workspot NOT latched at the edge (new rising edge) -> released", !HoldEligible(h) &&
                                                                                           std::string(HoldBlocker(h)) == "release:workspot");
    h.exitWindow = false;
    h.exitLatchedPsm = kPsmWorkspot;
    Expect("exit window: latched workspot after the window -> released (on-foot rule)", !HoldEligible(h));
    h.exitWindow = true;
    for (int det : {0, 6, 14, 18, 23, 24})
    {
        h.psm = kPsmWorkspot | kPsmDetailed;
        h.psmDetail = det;
        Expect(Fmt("exit window: latched workspot + detail %g (NotInBase / SlideFall / Fall / Jump / lands) -> held", det).c_str(), HoldEligible(h));
    }
    for (uint32_t g : {uint32_t(kGateStaleFeed), uint32_t(kGateLocoState), uint32_t(kGateOverSpeed)})
    {
        h.gate = kGatePsmBlock | g;
        h.psm = kPsmWorkspot | kPsmDetailed;
        h.psmDetail = 0;
        Expect(Fmt("exit window: latched workspot + gate 0x%g (stale / loco state / over-speed) -> held", double(g)).c_str(), HoldEligible(h));
    }
    h.gate = kGatePsmBlock;
    for (uint32_t p : {uint32_t(kPsmVehicle), uint32_t(kPsmScene), uint32_t(kPsmSwimming), uint32_t(kPsmTakedown), uint32_t(kPsmCarry),
                       uint32_t(kPsmVitals)})
    {
        h.psm = p | kPsmWorkspot;
        Expect(Fmt("exit window: PSM 0x%g (vehicle again / scene / swim / takedown / carry / vitals) still releases", double(p)).c_str(),
               !HoldEligible(h));
    }
    h.psm = kPsmWorkspot | kPsmDetailed;
    for (int det : {10, 11, 12, 13, 25, 29, 31})
    {
        h.psmDetail = det;
        Expect(Fmt("exit window: detail %g (ladder / VeryHardLand / knockdown / felled) still releases", det).c_str(), !HoldEligible(h));
    }
    h.psmDetail = 25;
    Expect("exit window: VeryHardLand 25 blocker named", std::string(HoldBlocker(h)) == "release:veryHardLand");
    h.exitWindow = false;
    h.psm = kPsmDetailed;
    Expect("exit window: VeryHardLand 25 on foot stays held (unchanged)", HoldEligible(h));
    h.gate = kGateTppInactive;
    Expect("exit window: blocker names - TPP off gate", std::string(HoldBlocker(h)) == "gate");
    Expect("exit window: ExitReleaseMask without the window == kPsmReleaseMask", ExitReleaseMask(false, kPsmWorkspot) == kPsmReleaseMask &&
                                                                                ExitReleaseMask(true, 0u) == kPsmReleaseMask);
    Expect("exit window: ExitReleaseMask with a latched workspot drops only the workspot bit",
           ExitReleaseMask(true, kPsmWorkspot) == (kPsmReleaseMask & ~uint32_t(kPsmWorkspot)));

    // ---- DecideUpperMask: TPP exit window keeps the upper body on MM (MOTO-6 / CAR-1 step 3), FPP and weapons unchanged
    UpperMaskInput in;
    in.exitWindow = true;
    in.gate = kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = 0;
    in.tpp = true;
    UpperMaskDecision d = DecideUpperMask(in);
    Expect("upper mask: TPP exit window, NotInBase 0 -> exit reason, gate covered, upper body on MM (upperFromGame false)",
           d.reasons == kMaskStateExit && d.covered == in.gate && !d.upperFromGame);
    in.tpp = false;
    d = DecideUpperMask(in);
    Expect("upper mask: FPP (tpp false) exit window -> kMaskStateExit with the game upper body (0.6.2)",
           d.reasons == kMaskStateExit && d.covered == in.gate && d.upperFromGame);
    in.tpp = true;
    in.weaponAction = true;
    d = DecideUpperMask(in);
    Expect("upper mask: TPP exit window with a weapon in hand -> the game upper body", d.reasons == kMaskStateExit && d.upperFromGame);
    in.weaponAction = false;
    in.meleeActive = true;
    in.gate |= kGateMelee;
    d = DecideUpperMask(in);
    Expect("upper mask: TPP exit window + melee -> the game upper body (melee reason)", (d.reasons & kMaskStateMelee) != 0 && d.upperFromGame);
    in = UpperMaskInput{};
    in.upperBody = kPsmUpperBodyAim;
    in.gate = kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = kPsmDetailAimWalk;
    in.tpp = true;
    d = DecideUpperMask(in);
    Expect("upper mask: TPP aim walk (no exit window) -> the game upper body, unchanged", d.reasons == kMaskStateAim && d.upperFromGame);
    in = UpperMaskInput{};
    in.exitWindow = true;
    in.tpp = true;
    in.gate = kGateLocoState | kGatePsmBlock;
    in.psm = kPsmDetailed;
    in.psmDetail = 0;
    in.vz = 3.0f;
    Expect("upper mask: exit window in flight (|vz| 3) -> no mask (J4 / CAR-2: the air area owns the body)", DecideUpperMask(in).reasons == 0u);
    in.psm = kPsmDetailed | kPsmWorkspot;
    in.vz = 0.0f;
    Expect("upper mask: exit window + workspot -> no mask (the hold decides)", DecideUpperMask(in).reasons == 0u);
}

// 2026-10-28 bike jump-out: JumpOutLatch (GameIntegration.hpp) as a table. The positive sequence is the recorded one (feed
// utpp-feed-2026-10-28-01-22-39 t 80.29-81.12: ride 0x9, the workspot drop at vz 0, launch 9.21 m/s 64 ms later, the vehicle bit
// for 0.78 s, then the bit falls at vz -2.48); the negatives are the vehicle-only states that must keep the vehicle bit.
static void TestJumpOutLatch()
{
    constexpr uint32_t kRide = kPsmVehicle | kPsmWorkspot;
    constexpr float dt = 1.0f / 60.0f;
    // ---- recorded jump-out
    {
        JumpOutLatch j;
        bool rideKept = true;
        for (int i = 0; i < 30; ++i)
            rideKept = !j.Update(kRide, 0.0f, true, dt) && rideKept;
        const bool drop = j.Update(kPsmVehicle, 0.0f, true, 0.064f); // first vehicle-only feed: vz 0 (a normal unmount looks the same)
        const bool launch = j.Update(kPsmVehicle, 9.21f, true, 0.032f);
        bool held = true;
        float vz = 9.21f;
        for (int i = 0; i < 44; ++i) // 0.73 s more of the flight: vz falls through 0 (and below kJumpOutRiseMs) - the latch stays
        {
            vz -= 16.0f * dt;
            held = j.Update(kPsmVehicle, vz, true, dt) && held;
        }
        const bool off = j.Update(0u, -2.48f, true, dt);
        Expect("jump-out: recorded flight - ride feeds keep the vehicle bit, the vz 0 drop feed too, the 9.21 m/s launch releases it",
               rideKept && !drop && launch, Fmt("drop %g launch %g", drop ? 1.0 : 0.0, launch ? 1.0 : 0.0));
        Expect("jump-out: the latch holds through the whole flight (vz below the rise threshold, past the arm window) until the raw bit falls",
               held && !off && !j.Active() && j.Fired() == 1u, Fmt("held %g fired %g", held ? 1.0 : 0.0, double(j.Fired())));
    }
    // ---- negatives: these keep the vehicle bit
    const auto any = [](JumpOutLatch& j, uint32_t aPsm, float aVz, bool aTpp, int aN) {
        bool r = false;
        for (int i = 0; i < aN; ++i)
            r = j.Update(aPsm, aVz, aTpp, dt) || r;
        return r;
    };
    {
        JumpOutLatch j;
        any(j, kRide, 0.0f, true, 30);
        const bool normal = any(j, kPsmVehicle, 0.0f, true, 2) || any(j, 0u, 0.0f, true, 1);
        Expect("jump-out: normal unmount (2 vehicle-only feeds at vz 0, as in all 18 recorded unmounts) keeps the vehicle bit", !normal);
    }
    {
        JumpOutLatch j;
        const bool ramp = any(j, kRide, 6.0f, true, 40);
        Expect("jump-out: a ramp jump while mounted (vehicle + workspot, vz 6) keeps the vehicle bit", !ramp);
    }
    {
        JumpOutLatch j;
        any(j, kRide, 0.0f, true, 30);
        any(j, kPsmVehicle, 0.0f, true, 18); // 0.30 s vehicle-only on the ground
        const bool late = j.Update(kPsmVehicle, 9.0f, true, dt);
        Expect("jump-out: a rise after the arm window (0.30 s after the workspot drop) keeps the vehicle bit", !late);
    }
    {
        JumpOutLatch j;
        const bool noRide = any(j, kPsmVehicle, 5.0f, true, 10); // vehicle bit before any workspot (MountingEvent start)
        Expect("jump-out: a rising vehicle-only feed without a ride before it keeps the vehicle bit", !noRide);
    }
    {
        JumpOutLatch j;
        any(j, kRide, 0.0f, true, 30);
        const bool knock = any(j, kPsmVehicle | kPsmDetailed, 3.67f, true, 3); // crash ejection 2026-10-25 23:40 (knockdown 29)
        JumpOutLatch k;
        any(k, kRide, 0.0f, true, 30);
        const bool fpp = any(k, kPsmVehicle, 9.2f, false, 3);
        Expect("jump-out: a crash ejection (LocomotionDetailed bit, knockdown) and an FPP flight keep the vehicle bit", !knock && !fpp);
    }
    {
        JumpOutLatch j;
        any(j, kRide, 0.0f, true, 30);
        j.Update(kPsmVehicle, 0.0f, true, dt);
        const bool fired = j.Update(kPsmVehicle, 9.2f, true, dt);
        const bool remount = j.Update(kRide, 0.0f, true, dt);
        const bool after = j.Update(kPsmVehicle, 0.0f, true, dt);
        Expect("jump-out: the workspot back after a release (re-mount / seat switch) ends the latch - the vehicle bit cuts again",
               fired && !remount && !after && !j.Active());
    }
}

int main(int argc, char** argv)
{
    {
        GamePoseSample s{}; s.rig = 42; s.generation = 7; s.count = 2; s.ticks = 100;
        Expect("game pose sample admits matching rig generation and bounded age", FreshGamePoseSample(s, 42, 7, 2, 110, 10));
        Expect("game pose sample rejects stale data", !FreshGamePoseSample(s, 42, 7, 2, 111, 10));
        Expect("game pose sample rejects a different rig", !FreshGamePoseSample(s, 43, 7, 2, 110, 10));
        Expect("game pose sample rejects rearmed generation", !FreshGamePoseSample(s, 42, 8, 2, 110, 10));
        Expect("game pose sample rejects layout mismatch and future timestamp", !FreshGamePoseSample(s, 42, 7, 3, 110, 10) && !FreshGamePoseSample(s, 42, 7, 2, 99, 10));
        GamePoseSample shot = s; shot.ticks = 107;
        Expect("Shoot transfer admits fresh pre-event and post-event samples",
               native_shoot::AdmitSamples(s, shot, 42, 7, 2, 105, 110, 10));
        Expect("Shoot transfer rejects absent, old and rearmed baselines",
               !native_shoot::AdmitSamples(s, shot, 42, 7, 2, 0, 110, 10) &&
               !native_shoot::AdmitSamples(s, shot, 42, 7, 2, 111, 112, 10) &&
               !native_shoot::AdmitSamples(s, shot, 42, 8, 2, 105, 110, 10));
        shot.ticks = 104;
        Expect("Shoot transfer rejects a sample from before the action event",
               !native_shoot::AdmitSamples(s, shot, 42, 7, 2, 105, 110, 10));
        native_shoot::DeltaHold hold;
        ov::Xf first{}; first.t = {0.005f, 0.0f, 0.0f};
        ov::Xf recovered{}; recovered.t = {0.008f, 0.0f, 0.0f};
        ov::Xf accepted{};
        const bool initial = hold.Resolve(true, true, first, 42, 7, 100, 10, accepted);
        const bool dropout = hold.Resolve(true, false, {}, 42, 7, 105, 10, accepted);
        const bool heldFirst = accepted.t.x == first.t.x;
        const bool resumed = hold.Resolve(true, true, recovered, 42, 7, 107, 10, accepted);
        Expect("Shoot transfer holds a brief missing sample and accepts the next fresh delta",
               initial && dropout && heldFirst && resumed && accepted.t.x == recovered.t.x);
        const bool oldGeneration = hold.Resolve(true, false, {}, 42, 8, 108, 10, accepted);
        Expect("Shoot transfer rejects a rig-generation transition", !oldGeneration);
        hold.Resolve(true, true, first, 42, 7, 200, 10, accepted);
        float fade = 0.0f;
        const bool decaying = hold.Resolve(true, false, {}, 42, 7, 215, 10, accepted, &fade);
        Expect("Shoot transfer fades held delta after 100 ms", decaying && fade > 0.45f && fade < 0.55f);
        const bool expired = hold.Resolve(true, false, {}, 42, 7, 220, 10, accepted, &fade);
        Expect("Shoot transfer returns to authored pose after 200 ms", !expired && fade == 0.0f);
        hold.Resolve(true, true, first, 42, 7, 300, 10, accepted);
        hold.Reset(); // Shoot -> Reload / changed weapon
        native_shoot::EpisodeGate gate;
        float activation = 0.0f;
        gate.Begin(300);
        const bool awaitingFirst = gate.Accept(false, true, 301, 10, 5, activation);
        const bool staysPending = gate.pending && !gate.admitted && activation == 0.0f;
        const bool nextFkSample = gate.Accept(true, false, 302, 10, 5, activation);
        const bool startsAtZero = activation == 0.0f;
        const bool ramping = gate.Accept(true, false, 304, 10, 5, activation) && activation > 0.0f && activation < 1.0f;
        const bool full = gate.Accept(true, false, 307, 10, 5, activation) && activation == 1.0f;
        Expect("Shoot waits for asynchronous post-event FK sample and ramps transfer smoothly",
               !awaitingFirst && staysPending && nextFkSample && startsAtZero && ramping && full);
        gate.Begin(400);
        const bool invalidFirst = gate.Accept(false, false, 401, 10, 5, activation);
        const bool lateAfterInvalid = gate.Accept(true, false, 402, 10, 5, activation);
        gate.Begin(500);
        const bool timeout = gate.Accept(false, true, 511, 10, 5, activation);
        const bool lateAfterTimeout = gate.Accept(true, false, 512, 10, 5, activation);
        Expect("Shoot invalid data and timeout cannot activate transfer later in the episode",
               !invalidFirst && !lateAfterInvalid && !timeout && !lateAfterTimeout);
        gate.Begin(600);
        const bool admittedFirst = gate.Accept(true, false, 600, 10, 5, activation);
        const bool briefGap = gate.Accept(false, false, 601, 10, 5, activation);
        gate.Reset(); // Shoot -> Reload
        const bool afterReload = gate.Accept(true, false, 602, 10, 5, activation);
        Expect("Shoot admitted episode tolerates short gaps and resets on Reload",
               admittedFirst && briefGap && !afterReload && activation == 0.0f);
        for (int hz : {30, 60, 144})
        {
            // Event arrives immediately after the preceding FK. The next driver update
            // still sees that old sample; one update later the new FK is available.
            gate.Begin(1000000);
            const int64_t step = 1000000 / hz;
            gate.Accept(false, true, 1000001, 100000, 50000, activation);
            const bool delayed = gate.Accept(true, false, 1000000 + step, 100000, 50000, activation);
            const bool zeroStart = activation == 0.0f;
            const bool settled = gate.Accept(true, false, 1050000 + step, 100000, 50000, activation);
            Expect(("Shoot admission handles event-before-FK order at " + std::to_string(hz) + " Hz").c_str(),
                   delayed && zeroStart && settled && activation == 1.0f);
        }
        Expect("Shoot transfer clears its hold on Reload or weapon change",
               !hold.Resolve(true, false, {}, 42, 7, 301, 10, accepted));
        const math::Quat yaw = gik::AxisAngle({0, 0, 1}, math::kPi / 2);
        const math::Quat recoil = gik::AxisAngle({0, 1, 0}, math::kPi / 18);
        ov::Xf source{yaw, {5.0f, 2.0f, 1.0f}};
        ov::Xf sourceFired{math::QuatMul(yaw, recoil), {5.0f, 2.02f, 1.0f}};
        ov::Xf motion;
        const bool motionOk = native_shoot::RelativeMotion(source, sourceFired, motion);
        Expect("Shoot recoil is measured in the source weapon axes",
               motionOk && gik::Len(motion.t - math::Vec3{.02f, 0, 0}) < 0.00001f &&
               ov::AngleDeg(motion.r, recoil) < 0.001f);
        const ov::Xf authoredWeapon{{}, {.5f, .4f, 1.0f}}, authoredHand{{}, {.45f, .4f, 1.0f}};
        const auto handTarget = native_shoot::RightHandTarget(motion, authoredWeapon, authoredHand);
        const auto expected = authoredWeapon.t + math::Vec3{.02f, 0, 0} + math::QuatRotate(recoil, {-.05f, 0, 0});
        Expect("Shoot transfer pivots around the authored weapon and preserves its hand attachment",
               gik::Len(handTarget.t - expected) < 0.00001f && ov::AngleDeg(handTarget.r, recoil) < 0.001f);
        source.t = source.t + math::Vec3{100, 20, -3};
        sourceFired.t = sourceFired.t + math::Vec3{100, 20, -3};
        ov::Xf shifted;
        Expect("Shoot transfer is independent of the FPP weapon's model-space position",
               native_shoot::RelativeMotion(source, sourceFired, shifted) &&
               gik::Len(shifted.t - motion.t) < 0.00001f && ov::AngleDeg(shifted.r, motion.r) < 0.001f);
        source.r = {0, 0, 0, 0};
        Expect("Shoot transfer rejects a degenerate source orientation",
               !native_shoot::RelativeMotion(source, sourceFired, shifted));
    }
    {
        struct Sample { uint32_t words[16]; };
        GamePoseMailbox<Sample> mailbox;
        Sample unread{};
        Expect("game pose mailbox rejects reads before the first sample", !mailbox.Read(unread));
        std::atomic<uint32_t> bad{0}, accepted{0}, done{0};
        std::vector<std::thread> writers;
        for (uint32_t id = 1; id <= 4; ++id)
            writers.emplace_back([&, id] {
                for (uint32_t i = 0; i < 20000; ++i)
                {
                    Sample s; for (auto& word : s.words) word = id * 100000 + i;
                    if (mailbox.TryPublish(s)) accepted.fetch_add(1);
                }
                done.fetch_add(1);
            });
        uint32_t reads = 0;
        while (done.load() != 4)
        {
            Sample s;
            if (mailbox.Read(s))
            {
                ++reads;
                for (auto word : s.words) if (word != s.words[0]) bad.fetch_add(1);
            }
        }
        for (auto& writer : writers) writer.join();
        Sample final{};
        Expect("game pose mailbox multiple writers never publish mixed samples", accepted.load() > 0 && reads > 0 && bad.load() == 0 && mailbox.Read(final));
    }
    {
        float records[24] = {};
        records[0] = 10.0f; records[7] = 1.0f;
        records[12] = 20.0f; records[19] = 1.0f;
        PoseSnapshot map{}; map.numBones = 2; map.gameIndex[0] = 1; map.gameIndex[1] = 0;
        float out[14]; for (float& v : out) v = -99.0f;
        auto capture = [&] { return CaptureMappedGamePose(reinterpret_cast<uint8_t*>(records), 2, map, out, 2); };
        Expect("game pose capture follows the explicit map and quaternion record offset", capture() && out[0] == 20.0f && out[7] == 10.0f && out[6] == 1.0f);
        float before[14]; std::memcpy(before, out, sizeof out);
        map.gameIndex[1] = -1;
        Expect("game pose capture rejects missing mappings without partial output", !capture() && std::memcmp(before, out, sizeof out) == 0);
        map.gameIndex[1] = 1;
        Expect("game pose capture rejects duplicate mappings", !capture());
        map.gameIndex[1] = 2;
        Expect("game pose capture rejects out-of-range mappings", !capture());
        map.gameIndex[1] = 0; records[7] = 0.0f;
        Expect("game pose capture rejects a zero quaternion", !capture());
        records[7] = 1.0f; records[0] = std::numeric_limits<float>::quiet_NaN();
        Expect("game pose capture rejects non-finite transforms", !capture());
        records[0] = 10.0f;
        Expect("game pose capture rejects insufficient capacity", !CaptureMappedGamePose(reinterpret_cast<uint8_t*>(records), 2, map, out, 1));
        Expect("game pose capture rejects null source", !CaptureMappedGamePose(nullptr, 2, map, out, 2));
    }
    const uint32_t weapon = kPsmWeaponKnown | kPsmWeaponDrawn;
    Expect("weapon diagnostic bits never close a ground gate", PsmBlockBits(weapon) == 0);
    Expect("weapon diagnostic bits preserve scene closure", PsmBlockBits(weapon | kPsmScene) == kPsmScene);
    Expect("weapon diagnostic decode needs known and drawn", PsmWeaponKnown(weapon) && PsmWeaponDrawn(weapon));
    Expect("unknown/empty weapon never fabricates a held item", !PsmWeaponDrawn(kPsmWeaponDrawn) && !PsmWeaponDrawn(kPsmWeaponKnown));
    const uint32_t oldFields = ((6u+1u)<<13) | ((14u+1u)<<16) | ((3u+1u)<<24) | kPsmDetailed;
    Expect("weapon diagnostics leave UpperBody, detail and melee decoders unchanged", PsmUpperBodyOf(weapon|oldFields) == 6 &&
           PsmDetailOf(weapon|oldFields) == 14 && PsmMeleeStateOf(weapon|oldFields) == 3 && PsmBlockBits(weapon|oldFields) == kPsmDetailed);
    TestYaw();
    TestGateAndWeight();
    TestGateFader();
    Database db;
    bool haveDb = false;
    if (argc >= 2)
    {
        std::string err;
        haveDb = db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, err);
        Expect("load database (rig hash 0x72CB314EB01F562D)", haveDb, err);
    }
    else
    {
        std::printf("[SKIP] database-backed checks (no path given)\n");
    }
    TestBoneMap(haveDb ? &db : nullptr);
    TestSeqlock();
    TestBlend();
    if (haveDb)
        TestEndToEnd(db);
    TestUpperMask(haveDb ? &db : nullptr);
    TestUpperMaskDecision();
    TestHoldEligible();
    TestExitWindowRules(); // vehicle batch W1
    TestJumpOutLatch();    // 2026-10-28 bike jump-out
    std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

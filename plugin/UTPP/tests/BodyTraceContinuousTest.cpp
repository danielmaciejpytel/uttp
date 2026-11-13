// Continuous helper-bone capture (src/mm/BodyTraceContinuous.hpp, Capture::Refill/Live in BodyDeformerTrace.hpp).
//   A) fast row formatter == the printf rows of the explicit flush (all 185 bones, both record types, special floats);
//   B) Capture / PostHelperCapture::Refill: tops the budget up without re-arming, never after Cancel, ring and dropped counter untouched;
//   C) controller state machine with fake streams / files / clock:
//      idle without enable / map, arm when a map arrives, steady 60 Hz producer + 4 ms writer ticks = every frame written, no gap at the
//      200-frame budget, writer behind = frames dropped and counted (stats == ring counter), outside cancel = re-arm, rig change =
//      close + new file pair (same map again = no-op), post flag change, clear map, size cap (stops, no reopen, restart on enable),
//      open failure retry, write failure stop, off / shutdown, stats log every 10 s;
//   D) throughput of the formatter (informational: rows/s, MB/s, microseconds per frame).
// Mutations UTPP_BTC_MUTATION 1 (no refill), 2 (dropped frames not accumulated), 3 (an identical map counts as a new rig), 4 (no size cap)
// must each print a FAIL line.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <type_traits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../src/mm/BodyTraceContinuous.hpp"

using namespace utpp::mm::bodytrace;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
template <class... A> std::string F(const char* aFmt, A... a)
{
    char b[512];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

// ---- A) formatter ------------------------------------------------------------------------------------------------------------------
float RandomFloat(std::mt19937_64& aRng, bool aExotic)
{
    const int kind = static_cast<int>(aRng() % 10);
    if (kind == 0) return 0.0f;
    if (kind == 1) return -0.0f;
    if (kind == 2 && aExotic) return std::numeric_limits<float>::infinity();
    if (kind == 3 && aExotic) return std::numeric_limits<float>::quiet_NaN(); // the value Copy() writes for an unmapped bone
    if (kind == 4) return std::ldexp(1.0f, static_cast<int>(aRng() % 60) - 30);
    if (kind == 5) return static_cast<float>(static_cast<int>(aRng() % 2001) - 1000) * 0.001f;
    uint32_t bits = static_cast<uint32_t>(aRng());
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    if (!aExotic && !std::isfinite(f)) return 1.5f;
    return f;
}
std::string RefLegacyRows(const Frame& f)
{
    std::string out;
    char b[1024];
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        int len = std::snprintf(b, sizeof(b), "%lld,%llu,%u,%llu,%llu,%u,%s,%d", f.ticks, static_cast<unsigned long long>(f.desc), f.thread,
                                static_cast<unsigned long long>(f.rig), static_cast<unsigned long long>(f.generation), f.rigCount,
                                kNames[n].data(), f.index[n]);
        out.append(b, static_cast<size_t>(len));
        for (uint32_t s = 0; s < 2; ++s)
            for (uint32_t v = 0; v < 12; ++v)
            {
                len = std::snprintf(b, sizeof(b), ",%.9g", static_cast<double>(f.stage[s][n][v]));
                out.append(b, static_cast<size_t>(len));
            }
        out.push_back('\n');
    }
    return out;
}
std::string RefPostRows(const PostHelperFrame& f)
{
    std::string out;
    char b[1024];
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        int len = std::snprintf(b, sizeof(b), "%lld,%llu,%u,%llu,%llu,%u,%llu,%llu,%lld,%lld,%s,%d", f.ticks, static_cast<unsigned long long>(f.desc),
                                f.thread, static_cast<unsigned long long>(f.rig), static_cast<unsigned long long>(f.generation), f.rigCount,
                                static_cast<unsigned long long>(f.receiptGeneration), static_cast<unsigned long long>(f.receiptCycle),
                                f.receiptStartedTicks, f.receiptLastTicks, kNames[n].data(), f.index[n]);
        out.append(b, static_cast<size_t>(len));
        for (uint32_t v = 0; v < 12; ++v)
        {
            len = std::snprintf(b, sizeof(b), ",%.9g", static_cast<double>(f.local[n][v]));
            out.append(b, static_cast<size_t>(len));
        }
        out.push_back('\n');
    }
    return out;
}
void Formatter()
{
    std::mt19937_64 rng(12345);
    auto f = std::make_unique<Frame>();
    auto p = std::make_unique<PostHelperFrame>();
    int mismatchLegacy = 0, mismatchPost = 0;
    size_t bytes = 0;
    for (int round = 0; round < 40; ++round)
    {
        const bool exotic = round % 2 == 0;
        f->ticks = static_cast<int64_t>(rng() >> 2); f->desc = rng(); f->thread = static_cast<uint32_t>(rng()); f->rig = rng();
        f->generation = rng() % 1000; f->rigCount = static_cast<uint32_t>(rng() % 400);
        p->ticks = static_cast<int64_t>(rng() >> 2); p->desc = rng(); p->thread = static_cast<uint32_t>(rng()); p->rig = rng();
        p->generation = rng() % 1000; p->rigCount = static_cast<uint32_t>(rng() % 400); p->receiptGeneration = rng() % 99;
        p->receiptCycle = rng() % 99; p->receiptStartedTicks = static_cast<int64_t>(rng() >> 3); p->receiptLastTicks = static_cast<int64_t>(rng() >> 3);
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            f->index[n] = p->index[n] = static_cast<int32_t>(rng() % 300) - 1;
            for (uint32_t v = 0; v < 12; ++v)
            {
                f->stage[0][n][v] = RandomFloat(rng, exotic);
                f->stage[1][n][v] = RandomFloat(rng, exotic);
                p->local[n][v] = RandomFloat(rng, exotic);
            }
        }
        ByteBuf b;
        AppendFrameRows(*f, b);
        const std::string got(b.Data(), b.Size()), ref = RefLegacyRows(*f);
        bytes += got.size();
        if (got != ref)
            ++mismatchLegacy;
        b.Clear();
        AppendFrameRows(*p, b);
        if (std::string(b.Data(), b.Size()) != RefPostRows(*p))
            ++mismatchPost;
    }
    Check(mismatchLegacy == 0, F("A: legacy rows identical to printf (%d of 40 random frames differ, %zu bytes)", mismatchLegacy, bytes));
    Check(mismatchPost == 0, F("A: post-helper rows identical to printf (%d of 40 differ)", mismatchPost));
    // 185 rows per frame, the indefinite NaN and the fixed edge values
    Frame e;
    for (uint32_t n = 0; n < kMaxBones; ++n)
        for (uint32_t v = 0; v < 12; ++v)
            e.stage[0][n][v] = e.stage[1][n][v] = 0.0f;
    e.stage[0][0][0] = -std::numeric_limits<float>::quiet_NaN();
    e.stage[0][0][1] = std::numeric_limits<float>::denorm_min();
    e.stage[0][0][2] = 3.4028235e38f;
    e.stage[0][0][3] = 1.0e-5f;
    e.stage[0][0][4] = 123456789.0f;
    ByteBuf b;
    AppendFrameRows(e, b);
    size_t lines = 0;
    for (size_t i = 0; i < b.Size(); ++i)
        lines += b.Data()[i] == '\n';
    Check(lines == kMaxBones && std::string(b.Data(), b.Size()) == RefLegacyRows(e), F("A: edge values (indefinite NaN, denormal, max, 1e-05) and %zu rows", lines));
}

// ---- B) Refill -------------------------------------------------------------------------------------------------------------------
template <class Cap, class Push> void RefillTest(const char* aTag, Push aPush)
{
    auto cap = std::make_unique<Cap>();
    Map map;
    map.rig = 77; map.rigCount = 2; map.index[0] = 0; map.index[1] = 1;
    Check(!cap->Live() && !cap->Refill(4, 2), F("B %s: not live before Arm; Refill refused", aTag));
    Check(cap->Arm(map, 4) && cap->Live() && cap->Active(), F("B %s: Arm -> live + active", aTag));
    Check(!cap->Refill(4, 2), F("B %s: Refill above the threshold does nothing", aTag));
    int pushed = 0;
    for (int i = 0; i < 6; ++i)
        pushed += aPush(*cap) ? 1 : 0;
    Check(pushed == 4 && !cap->Active() && cap->Live(), F("B %s: budget 4 exhausted (pushed %d): inactive but still live", aTag, pushed));
    Check(cap->Refill(4, 2) && cap->Active(), F("B %s: Refill reactivates without Arm (ring not drained)", aTag));
    int more = 0;
    for (int i = 0; i < 6; ++i)
        more += aPush(*cap) ? 1 : 0;
    Check(more == 4, F("B %s: second budget of 4 frames captured (%d)", aTag, more));
    Check(cap->Cancel() && !cap->Live() && !cap->Active(), F("B %s: Cancel -> not live", aTag));
    Check(!cap->Refill(4, 2) && !cap->Active(), F("B %s: Refill refused after Cancel", aTag));
    uint32_t popped = 0;
    typename std::conditional<std::is_same<Cap, Capture>::value, Frame, PostHelperFrame>::type fr;
    while (cap->Pop(fr))
        ++popped;
    Check(popped == 8, F("B %s: all 8 frames still in the ring after the refill (%u)", aTag, popped));
}

// ---- C) controller with fakes ----------------------------------------------------------------------------------------------------
class FakeStream final : public ITraceStream
{
public:
    bool live = false;
    uint32_t remaining = 0, pending = 0;
    uint64_t dropped = 0, produced = 0, lostInactive = 0, arms = 0, refills = 0;
    uint64_t bytesPerFrame = 100;
    bool failArm = false;
    bool Live() const override { return live; }
    bool Arm(const Map& aMap, uint32_t aBudget) override
    {
        if (failArm || pending || remaining)
            return false;
        map = aMap; live = true; remaining = aBudget; dropped = 0; ++arms;
        return true;
    }
    bool Refill(uint32_t aBudget, uint32_t aBelow) override
    {
        if (!live || remaining >= aBelow)
            return false;
        remaining = aBudget; ++refills;
        return true;
    }
    bool Cancel() override { live = false; remaining = 0; return true; }
    uint64_t Dropped() const override { return dropped; }
    uint32_t Drain(uint32_t aMax, IByteSink& aSink, bool* aFailed) override
    {
        uint32_t n = 0;
        std::string blob(static_cast<size_t>(bytesPerFrame), 'x');
        while (n < aMax && pending)
        {
            --pending; ++n;
            if (!aSink.Write(blob.data(), blob.size()) && aFailed)
                *aFailed = true;
        }
        return n;
    }
    // the FK thread: one frame per call
    void Produce()
    {
        ++produced;
        if (!live || !remaining)
        {
            ++lostInactive;
            return;
        }
        if (pending >= kRingSize)
        {
            ++dropped;
            return;
        }
        ++pending;
        --remaining;
    }
    Map map;
};

struct FakeSink final : IByteSink
{
    uint64_t bytes = 0;
    bool fail = false;
    bool Write(const char*, size_t n) override
    {
        if (fail)
            return false;
        bytes += n;
        return true;
    }
    uint64_t Bytes() const override { return bytes; }
};

struct FakeEnv final : IEnv
{
    int64_t now = 0;
    std::vector<std::string> logs;
    int opens = 0, closes = 0, failOpen = 0;
    bool open = false, lastPost = false;
    Map lastMap;
    std::vector<Totals> closed;
    FakeSink sinks[2];
    int64_t NowMs() override { return now; }
    void Log(const std::string& a) override { logs.push_back(a); }
    bool Open(const Map& aMap, const Topology&, bool aPost, uint32_t) override
    {
        if (failOpen > 0)
        {
            --failOpen;
            return false;
        }
        ++opens; open = true; lastMap = aMap; lastPost = aPost;
        sinks[0] = FakeSink{}; sinks[1] = FakeSink{};
        return true;
    }
    IByteSink* Sink(int aStream) override { return open ? &sinks[aStream] : nullptr; }
    void Close(const Totals& aTotals) override { ++closes; open = false; closed.push_back(aTotals); }
    std::string Describe() override { return "fake.csv"; }
    int Count(const char* aNeedle) const
    {
        int c = 0;
        for (const auto& l : logs)
            c += l.find(aNeedle) != std::string::npos;
        return c;
    }
};

Map MakeMap(uint64_t aRig, uint32_t aCount = 4)
{
    Map m;
    m.rig = aRig; m.rigCount = aCount;
    for (uint32_t i = 0; i < aCount && i < kMaxBones; ++i)
        m.index[i] = static_cast<int32_t>(i);
    return m;
}
std::shared_ptr<const Topology> Topo() { return std::make_shared<const Topology>(); }

struct Rig
{
    FakeEnv env;
    FakeStream legacy, post;
    ContinuousController::Config cfg;
    std::unique_ptr<ContinuousController> c;
    explicit Rig(const ContinuousController::Config& aCfg = {}) : cfg(aCfg) { c = std::make_unique<ContinuousController>(env, legacy, post, cfg); }
    // advances the clock by aMs; the FK thread produces at aHz, the writer ticks every aTickMs
    void Run(int64_t aMs, double aHz, int aTickMs, bool aWriter = true)
    {
        const int64_t end = env.now + aMs;
        double nextFrame = static_cast<double>(env.now);
        while (env.now < end)
        {
            env.now += 1;
            while (aHz > 0 && static_cast<double>(env.now) >= nextFrame)
            {
                legacy.Produce();
                if (post.live)
                    post.Produce();
                nextFrame += 1000.0 / aHz;
            }
            if (aWriter && env.now % aTickMs == 0)
                c->Tick();
        }
    }
};

void Controller()
{
    {
        Rig r;
        r.Run(1000, 60, 4);
        Check(r.env.opens == 0 && !r.legacy.live, "C: not enabled -> nothing opened or armed");
        r.c->Enable(true);
        r.Run(1000, 60, 4);
        Check(r.env.opens == 0 && !r.legacy.live, "C: enabled without a map -> nothing opened (waits for the rig)");
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(20, 60, 4);
        Check(r.env.opens == 1 && r.legacy.live && r.legacy.arms == 1 && !r.post.live && r.legacy.map.rig == 1,
              F("C: map arrives -> one session opened, legacy armed with that map (opens %d, arms %llu)", r.env.opens,
                static_cast<unsigned long long>(r.legacy.arms)));
        Check(!r.c->Enable(true), "C: Enable(true) twice is idempotent (no state change)");
        r.Run(1000, 60, 4);
        Check(r.env.opens == 1, "C: still one session after another second");
    }
    {   // steady state: 60 s, 60 Hz FK, 4 ms writer ticks
        Rig r;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), true, Topo());
        r.Run(60000, 60, 4);
        r.c->Shutdown();
        const auto s = r.c->GetStats();
        const uint64_t prod = r.legacy.produced;
        Check(r.legacy.lostInactive <= 1 && r.legacy.dropped == 0,
              F("C: 60 s at 60 Hz: no frame lost to a re-arm gap (produced %llu, lost while inactive %llu, dropped %llu, refills %llu)",
                static_cast<unsigned long long>(prod), static_cast<unsigned long long>(r.legacy.lostInactive),
                static_cast<unsigned long long>(r.legacy.dropped), static_cast<unsigned long long>(r.legacy.refills)));
        Check(s.lifetimeFrames[0] + r.legacy.pending >= prod - 2 && s.lifetimeFrames[0] <= prod,
              F("C: every produced frame was written (written %llu of %llu)", static_cast<unsigned long long>(s.lifetimeFrames[0]),
                static_cast<unsigned long long>(prod)));
        Check(s.lifetimeFrames[1] > 0 && r.env.closed.size() == 1 && r.env.closed[0].bytes[1] == s.lifetimeFrames[1] * 100,
              "C: post-helper stream drained into its own file");
        Check(r.env.Count("rows=") >= 6 && r.env.Count("rows=") <= 8, F("C: stats line every 10 s (%d lines in 60 s incl. the final one)", r.env.Count("rows=")));
    }
    {   // writer behind: 400 frames with no tick, then catch up
        Rig r;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(20, 0, 4);
        r.legacy.remaining = 400; // a large budget so the ring (256) is the limit
        for (int i = 0; i < 400; ++i)
            r.legacy.Produce();
        r.Run(100, 0, 4);
        const auto s = r.c->GetStats();
        Check(r.legacy.dropped == 144 && s.dropped[0] == 144 && s.frames[0] == 256,
              F("C: writer behind: ring counter %llu, controller dropped %llu (expected 144), written %llu (256)",
                static_cast<unsigned long long>(r.legacy.dropped), static_cast<unsigned long long>(s.dropped[0]),
                static_cast<unsigned long long>(s.frames[0])));
        r.c->Shutdown();
        Check(r.env.closed.size() == 1 && r.env.closed[0].dropped[0] == 144, "C: dropped frames reported in the file trailer totals");
        Check(r.env.Count("dropped=144") >= 1, "C: dropped count appears in the log line");
    }
    {   // outside cancel (explicit flush / disarm) -> re-arm for the same rig
        Rig r;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(2000, 60, 4);
        r.legacy.Cancel();
        r.Run(200, 60, 4);
        Check(r.legacy.live && r.legacy.arms == 2 && r.c->GetStats().rearms == 1 && r.env.opens == 1,
              F("C: cancelled from outside -> armed again in the same file (arms %llu, opens %d)", static_cast<unsigned long long>(r.legacy.arms), r.env.opens));
    }
    {   // rig change
        Rig r;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(2000, 60, 4);
        Check(!r.c->SetMap(MakeMap(1), false, Topo()), "C: identical map is a no-op");
        r.Run(500, 60, 4);
        Check(r.env.opens == 1 && r.env.closes == 0, F("C: identical map -> no new file pair (opens %d closes %d)", r.env.opens, r.env.closes));
        r.c->SetMap(MakeMap(2), false, Topo());
        r.Run(200, 60, 4);
        Check(r.env.opens == 2 && r.env.closes == 1 && r.legacy.map.rig == 2 && r.env.lastMap.rig == 2 && r.legacy.arms == 2,
              F("C: new rig -> old file closed, new pair opened, armed with the new map (opens %d closes %d arms %llu)", r.env.opens, r.env.closes,
                static_cast<unsigned long long>(r.legacy.arms)));
        Check(r.env.closed[0].frames[0] > 100, "C: the old file got its frames before closing");
        r.c->SetMap(MakeMap(2), true, Topo());
        r.Run(200, 60, 4);
        Check(r.env.opens == 3 && r.env.lastPost, "C: post flag change = new file pair with the post file");
        r.c->ClearMap();
        r.Run(100, 60, 4);
        Check(r.env.closes == 3 && !r.legacy.live && !r.post.live && !r.env.open, "C: rig gone -> files closed, captures cancelled");
        r.c->SetMap(MakeMap(5), false, Topo());
        r.Run(100, 60, 4);
        Check(r.env.opens == 4 && r.legacy.live, "C: new rig after a gap -> a new session");
    }
    {   // size cap
        ContinuousController::Config cfg;
        cfg.maxBytes = 5000; // 50 frames of 100 bytes
        Rig r(cfg);
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(5000, 60, 4);
        const auto s = r.c->GetStats();
        Check(s.capped && !s.running && r.env.Count("size cap") >= 1 && r.env.closes == 1 && !r.legacy.live && r.env.closed[0].bytes[0] < 5000 + 40 * 100,
              F("C: size cap stops the capture and closes the file (bytes %llu)", static_cast<unsigned long long>(r.env.closed.empty() ? 0 : r.env.closed[0].bytes[0])));
        r.Run(2000, 60, 4);
        Check(r.env.opens == 1, "C: capped -> never reopens by itself");
        r.c->Enable(false);
        r.c->Enable(true);
        r.Run(200, 60, 4);
        Check(r.env.opens == 2 && r.c->GetStats().running, "C: off + on starts a new capture after the cap");
    }
    {   // open failure and write failure
        Rig r;
        r.env.failOpen = 2;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), false, Topo());
        r.Run(1000, 60, 4);
        Check(r.env.opens == 0 && r.c->GetStats().openFails == 1, F("C: open failure -> retry only after the back-off (fails %llu)", static_cast<unsigned long long>(r.c->GetStats().openFails)));
        r.Run(5000, 60, 4);
        Check(r.env.opens == 0 && r.c->GetStats().openFails == 2, "C: second failure after 5 s");
        r.Run(5500, 60, 4);
        Check(r.env.opens == 1 && r.c->GetStats().running, "C: third attempt succeeds");
        r.env.sinks[0].fail = true;
        r.Run(300, 60, 4);
        Check(r.c->GetStats().failed && !r.c->GetStats().running && r.env.closes == 1 && r.env.Count("write failure") >= 1, "C: write failure ends the session and is logged");
        r.Run(2000, 60, 4);
        Check(r.env.opens == 1, "C: failed -> no reopen loop");
    }
    {   // off / shutdown
        Rig r;
        r.c->Enable(true);
        r.c->SetMap(MakeMap(1), true, Topo());
        r.Run(1000, 60, 4);
        r.c->Enable(false);
        r.Run(100, 60, 4);
        Check(r.env.closes == 1 && !r.legacy.live && !r.post.live && r.legacy.pending == 0 && r.env.Count("stopped (off)") == 1, "C: off -> final drain, files closed, captures cancelled");
        r.c->Shutdown();
        r.c->Shutdown();
        Check(r.env.closes == 1, "C: Shutdown after off is a no-op");
        Rig q;
        q.c->Enable(true);
        q.c->SetMap(MakeMap(1), false, Topo());
        q.Run(500, 60, 4);
        q.c->Shutdown();
        Check(q.env.closes == 1 && q.env.Count("stopped (shutdown)") == 1, "C: Shutdown while running closes the files");
    }
}

// ---- D) throughput -------------------------------------------------------------------------------------------------------------------
void Throughput()
{
    auto f = std::make_unique<Frame>();
    std::mt19937_64 rng(99);
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        f->index[n] = static_cast<int32_t>(n);
        for (uint32_t v = 0; v < 12; ++v)
        {
            f->stage[0][n][v] = static_cast<float>(rng() % 20001) * 0.0001f - 1.0f;
            f->stage[1][n][v] = static_cast<float>(rng() % 20001) * 0.0001f - 1.0f;
        }
    }
    f->ticks = 4750377550594;
    f->rig = 140737488355328ull; f->desc = 140737488355328ull; f->generation = 3; f->rigCount = 294; f->thread = 1234;
    ByteBuf b;
    const int frames = 500;
    const auto t0 = std::chrono::steady_clock::now();
    size_t bytes = 0;
    for (int i = 0; i < frames; ++i)
    {
        b.Clear();
        AppendFrameRows(*f, b);
        bytes += b.Size();
    }
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double usPerFrame = sec * 1e6 / frames;
    std::printf("INFO D: %d frames formatted: %.1f us/frame, %.0f MB/s, %.0f KB/frame -> at 60 Hz %.1f MB/s of file, formatter load %.2f%% of one core\n", frames, usPerFrame,
                static_cast<double>(bytes) / sec / 1e6, static_cast<double>(bytes) / frames / 1e3, static_cast<double>(bytes) / frames * 60 / 1e6,
                usPerFrame * 60 / 1e4);
    Check(usPerFrame < 3000.0, F("D: one frame (185 rows x 24 values) formats in %.1f us (< 3 ms)", usPerFrame));
}
} // namespace

int main()
{
    Formatter();
    RefillTest<Capture>("legacy", [](Capture& c) {
        auto f = std::make_unique<Frame>();
        uint8_t bones[2 * 48] = {};
        if (!c.Begin(*f, bones, 2, 77, 5, 10, 1))
            return false;
        return c.End(*f, bones, 2, 77, 5);
    });
    RefillTest<PostHelperCapture>("post", [](PostHelperCapture& c) {
        uint8_t bones[2 * 48] = {};
        return c.Capture(bones, 2, 77, 5, 10, 1, 1, 1, 5, 6);
    });
    Controller();
    Throughput();
    std::printf("body trace continuous: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

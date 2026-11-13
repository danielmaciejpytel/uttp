#pragma once
// Continuous capture of ALL helper bones (kMaxBones = 185, both stages) - controller + fast CSV row formatter. Pure C++ (no game types,
// no Win32): the production environment (files, writer thread, logger) lives in src/BodyTraceRuntime.cpp, the tests drive the same
// controller with fake streams / sinks / clock (tests/BodyTraceContinuousTest.cpp).
//
// Native: UTPPBodyPoseTraceContinuous(on: Bool) -> Bool (Main.cpp, scripts/utpp.reds). While on, a background writer thread (never the
// game / FK thread; the FK hot path of BodyDeformerTrace.hpp stays allocation- and IO-free) keeps the Capture rings armed (budget 200 per
// refill, topped up without a gap via Capture::Refill) and drains them to
//   <game>\red4ext\logs\utpp-bodytrace-<yyyy-mm-dd-hh-mm-ss>.csv            (before_mm / after_mm_and_aim, 185 bones x 24 values)
//   <game>\red4ext\logs\utpp-bodytrace-posthelper-<yyyy-mm-dd-hh-mm-ss>.csv (post-helper apply stage, 12 values; only when the helper apply
//                                                                              is enabled)
// with the same header, column names and row format as the explicit UTPPBodyPoseTraceArm / Flush files (existing analysis scripts such as
// tools\analysis\replay_body_skin.py read them unchanged). One file pair per rig: a rig change (player entity swap, re-arm) closes the
// files and opens a new pair, so every file has exactly one `# rig_bone` topology. Frames the writer cannot take (ring full) are counted by
// the capture (`dropped`) and reported every 10 s as one log line with rows / frames / dropped / bytes / file; a hard size cap per file
// (default 6 GB) stops the capture with a log line. The explicit natives refuse to arm while the continuous mode owns the captures.
#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>

#include "BodyDeformerTrace.hpp"

namespace utpp::mm::bodytrace
{
// ---- fast row formatting (identical text to the printf rows of the explicit flush: "%lld,%llu,%u,...,%s,%d" then ",%.9g" values) --------
class ByteBuf
{
public:
    void Clear() { m_n = 0; }
    const char* Data() const { return m_d.data(); }
    size_t Size() const { return m_n; }
    // Room for aN more bytes; returns the write pointer.
    char* Reserve(size_t aN)
    {
        if (m_n + aN > m_d.size())
            m_d.resize(std::max(m_d.size() * 2, m_n + aN));
        return m_d.data() + m_n;
    }
    void Commit(size_t aN) { m_n += aN; }
    void Putc(char c) { *Reserve(1) = c; Commit(1); }
    void Put(const char* s, size_t n) { std::memcpy(Reserve(n), s, n); Commit(n); }
    void PutInt(long long v) { char* p = Reserve(24); Commit(static_cast<size_t>(std::to_chars(p, p + 24, v).ptr - p)); }
    void PutUInt(unsigned long long v) { char* p = Reserve(24); Commit(static_cast<size_t>(std::to_chars(p, p + 24, v).ptr - p)); }
    // %.9g of the float (the explicit flush printed double(float) with %.9g: same value, same digits).
    void PutFloat9(float v)
    {
        char* p = Reserve(32);
        // a signalling NaN prints as nan(snan) from a float but as nan through the double conversion printf does: convert NaNs first
        const auto r = v != v ? std::to_chars(p, p + 32, static_cast<double>(v), std::chars_format::general, 9)
                              : std::to_chars(p, p + 32, v, std::chars_format::general, 9);
        Commit(r.ec == std::errc{} ? static_cast<size_t>(r.ptr - p) : 0);
    }

private:
    std::vector<char> m_d = std::vector<char>(1u << 17);
    size_t m_n = 0;
};

// Appends the 185 rows of one legacy frame (stage 0 values 0-11 then stage 1 values 0-11 per row).
inline void AppendFrameRows(const Frame& f, ByteBuf& out)
{
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        out.PutInt(f.ticks); out.Putc(',');
        out.PutUInt(f.desc); out.Putc(',');
        out.PutUInt(f.thread); out.Putc(',');
        out.PutUInt(f.rig); out.Putc(',');
        out.PutUInt(f.generation); out.Putc(',');
        out.PutUInt(f.rigCount); out.Putc(',');
        out.Put(kNames[n].data(), kNames[n].size()); out.Putc(',');
        out.PutInt(f.index[n]);
        for (uint32_t s = 0; s < 2; ++s)
            for (uint32_t v = 0; v < 12; ++v)
            {
                out.Putc(',');
                out.PutFloat9(f.stage[s][n][v]);
            }
        out.Putc('\n');
    }
}
// Appends the 185 rows of one post-helper frame.
inline void AppendFrameRows(const PostHelperFrame& f, ByteBuf& out)
{
    for (uint32_t n = 0; n < kMaxBones; ++n)
    {
        out.PutInt(f.ticks); out.Putc(',');
        out.PutUInt(f.desc); out.Putc(',');
        out.PutUInt(f.thread); out.Putc(',');
        out.PutUInt(f.rig); out.Putc(',');
        out.PutUInt(f.generation); out.Putc(',');
        out.PutUInt(f.rigCount); out.Putc(',');
        out.PutUInt(f.receiptGeneration); out.Putc(',');
        out.PutUInt(f.receiptCycle); out.Putc(',');
        out.PutInt(f.receiptStartedTicks); out.Putc(',');
        out.PutInt(f.receiptLastTicks); out.Putc(',');
        out.Put(kNames[n].data(), kNames[n].size()); out.Putc(',');
        out.PutInt(f.index[n]);
        for (uint32_t v = 0; v < 12; ++v)
        {
            out.Putc(',');
            out.PutFloat9(f.local[n][v]);
        }
        out.Putc('\n');
    }
}

// ---- seams (production = real captures / files / thread, tests = fakes) ------------------------------------------------------------
class IByteSink
{
public:
    virtual ~IByteSink() = default;
    virtual bool Write(const char* aData, size_t aSize) = 0; // false = IO failure
    virtual uint64_t Bytes() const = 0;                      // bytes accepted so far
};

class ITraceStream // one capture ring
{
public:
    virtual ~ITraceStream() = default;
    virtual bool Live() const = 0;                          // armed and not cancelled (Arm succeeded, Cancel not called since)
    virtual bool Arm(const Map& aMap, uint32_t aBudget) = 0; // fails while undrained frames / an active session exist
    virtual bool Refill(uint32_t aBudget, uint32_t aBelow) = 0;
    virtual bool Cancel() = 0;                              // false = producer busy right now
    virtual uint64_t Dropped() const = 0;                   // since the last Arm
    // Pops up to aMax frames, formats them and writes them to aSink. Returns the number of frames popped (all of them are counted even
    // when the sink failed; *aFailed is set then).
    virtual uint32_t Drain(uint32_t aMax, IByteSink& aSink, bool* aFailed) = 0;
};

struct Topology // rig bone hashes and parents of the mapped rig (written as `# rig_bone` header lines by the environment)
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
};

struct Totals
{
    uint64_t frames[2] = {0, 0}, dropped[2] = {0, 0}, bytes[2] = {0, 0};
    bool failed = false;
};

class IEnv
{
public:
    virtual ~IEnv() = default;
    virtual int64_t NowMs() = 0;
    virtual void Log(const std::string& aLine) = 0;
    // Creates the file(s) for this rig (headers written). aPost = also the post-helper file. false = failed (logged by the environment).
    virtual bool Open(const Map& aMap, const Topology& aTopology, bool aPost, uint32_t aBudget) = 0;
    virtual IByteSink* Sink(int aStream) = 0; // 0 = legacy, 1 = post-helper; nullptr when not open
    virtual void Close(const Totals& aTotals) = 0; // trailer + close
    virtual std::string Describe() = 0;            // files for the log lines
};

inline bool SameMap(const Map& a, const Map& b)
{
    return a.rig == b.rig && a.rigCount == b.rigCount && a.index == b.index;
}

// Fixed-rate (writer thread) state machine. Control path: Enable / SetMap / ClearMap (any thread, short mutex). Writer thread: Tick.
class ContinuousController
{
public:
    struct Config
    {
        uint32_t budget = kMaxFrames;       // frames per refill (Capture limit)
        uint32_t refillBelow = kMaxFrames / 2; // refill when fewer frames remain
        uint32_t drainBatch = 32;           // frames per stream per Tick before re-checking the rest
        uint64_t maxBytes = 6ull << 30;     // hard cap per file
        int64_t statsPeriodMs = 10000;
        int64_t retryMs = 5000;             // after a failed Open
        std::string logPrefix = "UTPP body trace continuous"; // start of every log line of the controller
    };
    struct Stats
    {
        uint64_t frames[2] = {0, 0}, dropped[2] = {0, 0};
        uint64_t lifetimeFrames[2] = {0, 0}, lifetimeDropped[2] = {0, 0}; // all finished files (frames / dropped of the running file are above)
        uint64_t arms = 0, sessions = 0, openFails = 0, rearms = 0;
        bool running = false, capped = false, failed = false;
    };

    ContinuousController(IEnv& aEnv, ITraceStream& aLegacy, ITraceStream& aPost, const Config& aCfg)
        : m_env(aEnv), m_cfg(aCfg)
    {
        m_stream[0] = &aLegacy;
        m_stream[1] = &aPost;
    }

    // ---- control path ----
    bool Enable(bool aOn) // returns true when the state changed (idempotent)
    {
        std::lock_guard lock(m_mutex);
        if (m_want == aOn)
            return false;
        m_want = aOn;
        if (aOn)
            m_restart = true; // a new session clears cap / failure
        return true;
    }
    bool Enabled() const { std::lock_guard lock(m_mutex); return m_want; }
    // Publishes the map of the CURRENT rig. An identical map (same rig, count, indices, post flag) is a no-op; a different one makes the
    // writer close the files and open a new pair.
    bool SetMap(const Map& aMap, bool aPost, std::shared_ptr<const Topology> aTopology)
    {
        std::lock_guard lock(m_mutex);
#if !(defined(UTPP_BTC_MUTATION) && UTPP_BTC_MUTATION == 3) // mutation 3: an identical map counts as a new rig
        if (m_mapValid && m_post == aPost && SameMap(m_map, aMap))
            return false;
#endif
        m_map = aMap; m_post = aPost; m_topology = std::move(aTopology); m_mapValid = true;
        ++m_mapGen;
        return true;
    }
    void ClearMap()
    {
        std::lock_guard lock(m_mutex);
        if (!m_mapValid)
            return;
        m_mapValid = false;
        ++m_mapGen;
    }

    // ---- writer thread ----
    // One iteration; returns the number of frames drained (the thread sleeps only when 0).
    uint32_t Tick()
    {
        bool want, valid, post, restart;
        uint64_t gen;
        Map map;
        std::shared_ptr<const Topology> topo;
        {
            std::lock_guard lock(m_mutex);
            want = m_want; valid = m_mapValid; post = m_post; gen = m_mapGen; map = m_map; topo = m_topology;
            restart = m_restart;
            m_restart = false;
        }
        uint32_t drained = 0;
        if (restart)
        {
            if (m_session)
                drained += EndSession("restart");
            m_capped = m_failed = false;
            m_retryAt = 0;
        }
        if (!want || !valid)
        {
            if (m_session)
                drained += EndSession(!want ? "off" : "no rig");
            return drained;
        }
        if (m_capped || m_failed)
            return drained;
        if (m_session && gen != m_armedGen)
            drained += EndSession("rig changed");
        if (!m_session)
        {
            const int64_t now = m_env.NowMs();
            if (m_retryAt && now < m_retryAt)
                return drained;
            if (!StartSession(map, topo ? *topo : Topology{}, post, gen))
            {
                m_retryAt = m_env.NowMs() + m_cfg.retryMs;
                return drained;
            }
            m_nextStats = m_env.NowMs() + m_cfg.statsPeriodMs;
            return drained;
        }
        for (int s = 0; s < 2; ++s)
        {
            if (!m_active[s])
                continue;
            drained += DrainStream(s, m_cfg.drainBatch);
            if (!m_stream[s]->Live())
            {
                // cancelled from outside (explicit flush / disarm): drain what is left and arm again for the same rig
                Absorb(s);
                if (CancelWithRetry(s))
                {
                    drained += DrainAll(s);
                    Absorb(s);
                    if (m_stream[s]->Arm(m_map, m_cfg.budget))
                    {
                        ++m_stats.rearms;
                        m_last[s] = 0;
                    }
                }
            }
            else
            {
#if !(defined(UTPP_BTC_MUTATION) && UTPP_BTC_MUTATION == 1) // mutation 1: the budget is never topped up (a gap every 200 frames)
                m_stream[s]->Refill(m_cfg.budget, m_cfg.refillBelow);
#endif
            }
        }
        const IByteSink* sink0 = m_env.Sink(0);
        const IByteSink* sink1 = m_active[1] ? m_env.Sink(1) : nullptr;
        if (m_failed)
            drained += EndSession("write failure");
#if defined(UTPP_BTC_MUTATION) && UTPP_BTC_MUTATION == 4 // mutation 4: no size cap
        else if (sink0 && sink0->Bytes() == ~0ull)
#else
        else if ((sink0 && sink0->Bytes() >= m_cfg.maxBytes) || (sink1 && sink1->Bytes() >= m_cfg.maxBytes))
#endif
        {
            m_capped = true;
            m_env.Log(m_cfg.logPrefix + ": size cap reached, stopping (" + std::to_string(m_cfg.maxBytes) + " bytes per file)");
            drained += EndSession("size cap");
        }
        else if (m_env.NowMs() >= m_nextStats)
        {
            m_nextStats = m_env.NowMs() + m_cfg.statsPeriodMs;
            LogStats();
        }
        return drained;
    }
    // Final drain + close (DLL unload / off); safe to call when idle.
    void Shutdown()
    {
        { std::lock_guard lock(m_mutex); m_want = false; }
        if (m_session)
            EndSession("shutdown");
    }
    Stats GetStats() const
    {
        Stats s = m_stats;
        s.running = m_session;
        s.capped = m_capped;
        s.failed = m_failed;
        return s;
    }

private:
    uint32_t DrainStream(int aS, uint32_t aMax)
    {
        IByteSink* sink = m_env.Sink(aS);
        if (!sink)
            return 0;
        bool failed = false;
        const uint32_t n = m_stream[aS]->Drain(aMax, *sink, &failed);
        m_stats.frames[aS] += n;
        if (failed)
            m_failed = true;
        Absorb(aS);
        return n;
    }
    uint32_t DrainAll(int aS)
    {
        uint32_t total = 0;
        for (int i = 0; i < 16; ++i) // the ring holds kRingSize frames
        {
            const uint32_t n = DrainStream(aS, kRingSize);
            total += n;
            if (n == 0)
                break;
        }
        return total;
    }
    void Absorb(int aS) // dropped frames are reset by Arm: accumulate the deltas
    {
        const uint64_t d = m_stream[aS]->Dropped();
#if defined(UTPP_BTC_MUTATION) && UTPP_BTC_MUTATION == 2 // mutation 2: dropped frames are not accumulated
        m_last[aS] = d;
        return;
#endif
        if (d >= m_last[aS])
            m_stats.dropped[aS] += d - m_last[aS];
        m_last[aS] = d;
    }
    bool CancelWithRetry(int aS)
    {
        for (int i = 0; i < 64; ++i)
            if (m_stream[aS]->Cancel())
                return true;
        return false;
    }
    bool StartSession(const Map& aMap, const Topology& aTopo, bool aPost, uint64_t aGen)
    {
        if (!m_env.Open(aMap, aTopo, aPost, m_cfg.budget))
        {
            ++m_stats.openFails;
            return false;
        }
        m_map = aMap;
        m_active[0] = true;
        m_active[1] = aPost;
        for (int s = 0; s < 2; ++s)
        {
            m_last[s] = 0;
            if (!m_active[s])
                continue;
            // leftovers of an earlier session may still sit in the ring (cancelled elsewhere): they belong to no file of this session
            CancelWithRetry(s);
            DiscardRing(s);
            if (!m_stream[s]->Arm(aMap, m_cfg.budget))
            {
                m_env.Log(m_cfg.logPrefix + ": arm failed (stream " + std::to_string(s) + ")");
                for (int c = 0; c < 2; ++c)
                    if (m_active[c])
                        CancelWithRetry(c);
                m_active[0] = m_active[1] = false;
                Totals t;
                m_env.Close(t);
                ++m_stats.openFails;
                return false;
            }
            ++m_stats.arms;
        }
        m_armedGen = aGen;
        m_session = true;
        ++m_stats.sessions;
        m_env.Log(m_cfg.logPrefix + ": started " + m_env.Describe());
        return true;
    }
    void DiscardRing(int aS)
    {
        struct Null : IByteSink
        {
            bool Write(const char*, size_t) override { return true; }
            uint64_t Bytes() const override { return 0; }
        } null;
        bool f = false;
        for (int i = 0; i < 16 && m_stream[aS]->Drain(kRingSize, null, &f) > 0; ++i)
        {
        }
    }
    uint32_t EndSession(const char* aWhy)
    {
        uint32_t drained = 0;
        for (int s = 0; s < 2; ++s)
            if (m_active[s])
            {
                CancelWithRetry(s);
                drained += DrainAll(s);
                Absorb(s);
            }
        Totals t;
        for (int s = 0; s < 2; ++s)
        {
            t.frames[s] = m_stats.frames[s];
            t.dropped[s] = m_stats.dropped[s];
            const IByteSink* sink = m_env.Sink(s);
            t.bytes[s] = sink ? sink->Bytes() : 0;
        }
        t.failed = m_failed;
        LogStats();
        m_env.Log(m_cfg.logPrefix + ": stopped (" + aWhy + ") " + m_env.Describe());
        m_env.Close(t);
        m_session = false;
        m_active[0] = m_active[1] = false;
        // per-file counters restart with the next file pair
        for (int s = 0; s < 2; ++s)
        {
            m_stats.lifetimeFrames[s] += m_stats.frames[s];
            m_stats.lifetimeDropped[s] += m_stats.dropped[s];
            m_stats.frames[s] = 0;
            m_stats.dropped[s] = 0;
            m_last[s] = 0;
        }
        return drained;
    }
    void LogStats()
    {
        const IByteSink* s0 = m_env.Sink(0);
        const IByteSink* s1 = m_env.Sink(1);
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "rows=%llu frames=%llu dropped=%llu postRows=%llu postFrames=%llu postDropped=%llu bytes=%llu postBytes=%llu",
                      static_cast<unsigned long long>(m_stats.frames[0] * kMaxBones), static_cast<unsigned long long>(m_stats.frames[0]),
                      static_cast<unsigned long long>(m_stats.dropped[0]), static_cast<unsigned long long>(m_stats.frames[1] * kMaxBones),
                      static_cast<unsigned long long>(m_stats.frames[1]), static_cast<unsigned long long>(m_stats.dropped[1]),
                      static_cast<unsigned long long>(s0 ? s0->Bytes() : 0), static_cast<unsigned long long>(s1 ? s1->Bytes() : 0));
        m_env.Log(m_cfg.logPrefix + ": " + buf + " " + m_env.Describe());
    }

    IEnv& m_env;
    Config m_cfg;
    ITraceStream* m_stream[2] = {nullptr, nullptr};

    mutable std::mutex m_mutex; // control path <-> writer thread
    bool m_want = false, m_restart = false, m_mapValid = false, m_post = false;
    Map m_map;
    std::shared_ptr<const Topology> m_topology;
    uint64_t m_mapGen = 0;

    // writer thread only
    bool m_session = false, m_capped = false, m_failed = false;
    bool m_active[2] = {false, false};
    uint64_t m_armedGen = 0;
    uint64_t m_last[2] = {0, 0};
    int64_t m_nextStats = 0, m_retryAt = 0;
    Stats m_stats;
};

// ---- production adapters for the real rings -----------------------------------------------------------------------------------------
template <class Cap, class FrameT>
class CaptureStream final : public ITraceStream
{
public:
    explicit CaptureStream(Cap& aCap) : m_cap(aCap), m_frame(std::make_unique<FrameT>()) {}
    bool Live() const override { return m_cap.Live(); }
    bool Arm(const Map& aMap, uint32_t aBudget) override { return m_cap.Arm(aMap, aBudget); }
    bool Refill(uint32_t aBudget, uint32_t aBelow) override { return m_cap.Refill(aBudget, aBelow); }
    bool Cancel() override { return m_cap.Cancel(); }
    uint64_t Dropped() const override { return m_cap.Dropped(); }
    uint32_t Drain(uint32_t aMax, IByteSink& aSink, bool* aFailed) override
    {
        uint32_t n = 0;
        while (n < aMax && m_cap.Pop(*m_frame))
        {
            ++n;
            m_buf.Clear();
            AppendFrameRows(*m_frame, m_buf);
            if (!aSink.Write(m_buf.Data(), m_buf.Size()) && aFailed)
                *aFailed = true;
        }
        return n;
    }

private:
    Cap& m_cap;
    std::unique_ptr<FrameT> m_frame;
    ByteBuf m_buf;
};
} // namespace utpp::mm::bodytrace

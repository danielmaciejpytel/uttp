#pragma once
// Compact pose capture, session layer. Pure C++ (no game types, no Win32): files, clock and compression are injected, so the offline tests
// run the whole thing with in-memory files. The block format is in mm/PoseCaptureFormat.hpp; the production environment (real files, the
// Windows Compression API, writer thread) is in src/PoseCaptureRuntime.cpp.
//
// The controller of the CSV capture (mm/BodyTraceContinuous.hpp, ContinuousController) is reused as the state machine: it keeps the capture
// rings armed and topped up without a gap, drains them from the writer thread, counts frames the ring could not take (`dropped`), enforces the
// hard size cap and closes / opens a file when the rig changes. This file adds the environment it drives (PoseEnv) and the ring adapter that
// hands every frame to the environment as a plain record.
//
// PoseEnv writes <folder>\<prefix><yyyy-mm-dd-hh-mm-ss>.utpppose:
//   keepMinutes == 0  prefix utpp-pose-: one file per rig for the whole session (size cap applies per file); never removed by the mod;
//   keepMinutes  > 0  prefix utpp-pose-ring-: segment files of about one minute; a segment older than the window is deleted as soon as it is
//                     closed, so the files present always cover at least the last keepMinutes minutes. Every segment is readable on its own
//                     (own header with the topology, blocks start with a key frame).
//                     The first file of such a capture also removes ring segments (utpp-pose-ring-*) of earlier runs older than the window
//                     (PoseEnv::Open); files of the timed mode (utpp-pose-*) are never touched.
// Frames are collected into blocks (blockFrames frames or blockMaxMs milliseconds, whichever comes first), each block is quantized, delta
// coded and compressed (PoseCaptureFormat.hpp) and appended to the file. A pending block is written when the file is closed.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "BodyTraceContinuous.hpp"
#include "PoseCaptureFormat.hpp"

namespace utpp::mm::posecapture
{
using bodytrace::IByteSink;
using bodytrace::IEnv;
using bodytrace::ITraceStream;
using bodytrace::Map;
using bodytrace::Topology;
using bodytrace::Totals;

// ---- seams ------------------------------------------------------------------------------------------------------------------------------
class IPoseFile
{
public:
    virtual ~IPoseFile() = default;
    virtual bool Write(const uint8_t* aData, size_t aSize) = 0; // false = IO failure
    virtual bool Close() = 0;
};
class IPoseStorage
{
public:
    virtual ~IPoseStorage() = default;
    virtual std::unique_ptr<IPoseFile> Create(const std::string& aFileName) = 0; // exclusive create; nullptr = exists / failed
    virtual bool Remove(const std::string& aFileName) = 0;
    // Removes plain files of the capture folder named <prefix>*<extension> whose last write is more than aOlderThanMs ago. Called once, before the
    // first Create of a session: no file of a running session exists yet. Errors are ignored.
    virtual void RemoveStale(const std::string& aPrefix, const std::string& aExtension, int64_t aOlderThanMs)
    {
        (void)aPrefix;
        (void)aExtension;
        (void)aOlderThanMs;
    }
    virtual std::string DisplayPath(const std::string& aFileName) const = 0;      // path relative to the game folder (for logs and results)
};
struct HostTime
{
    std::string stamp = "1970-01-01-00-00-00"; // local time yyyy-mm-dd-hh-mm-ss (file name)
    uint16_t utc[7] = {1970, 1, 1, 0, 0, 0, 0}; // year month day hour minute second millisecond
    int64_t qpcHz = 10000000, qpcTicks = 0;
};
class IPoseHost
{
public:
    virtual ~IPoseHost() = default;
    virtual int64_t NowMs() = 0; // monotonic
    virtual HostTime Time() = 0;
    virtual void Log(const std::string& aLine) = 0;
};

// One frame as the ring adapter hands it over: MetaCount(stream) int64 fields, then kMaxBones * StageCount(stream) * 12 floats
// (bone slot n, stage s, value v at (n * stages + s) * 12 + v; slots outside the rig hold nan).
inline size_t RecordBytes(uint32_t aStream) { return sizeof(int64_t) * MetaCount(aStream) + sizeof(float) * kMaxBones * StageCount(aStream) * 12; }

class PoseEnv final : public IEnv
{
public:
    struct Config
    {
        uint32_t keepMinutes = 0;       // 0 = one file per rig for the whole session
        int64_t segmentMs = 60000;      // segment length when keepMinutes > 0
        uint32_t blockFrames = 300;
        int64_t blockMaxMs = 5000;
        std::string filePrefix = "utpp-pose-";         // timed mode (keepMinutes 0): the files the player sends, never removed by the mod
        std::string ringPrefix = "utpp-pose-ring-";    // permanent mode (keepMinutes > 0): ring segments, the only files the mod removes
        std::string fileExtension = ".utpppose";
    };
    using DroppedProbe = std::function<void(uint64_t (&aDropped)[2])>;

    PoseEnv(IPoseHost& aHost, IPoseStorage& aStorage, ICompressor& aComp, const Config& aCfg)
        : m_host(aHost), m_storage(aStorage), m_comp(aComp), m_cfg(aCfg)
    {
        m_sink[0] = std::make_unique<StreamSink>(*this, kStreamLegacy);
        m_sink[1] = std::make_unique<StreamSink>(*this, kStreamPostHelper);
    }
    ~PoseEnv() override
    {
        if (m_open)
            Close(Totals{});
    }
    // Called from the writer thread: current dropped-frame counters of the running file (segment trailers carry the delta).
    void SetDroppedProbe(DroppedProbe aProbe) { m_probe = std::move(aProbe); }

    // ---- IEnv ----
    int64_t NowMs() override { return m_host.NowMs(); }
    void Log(const std::string& aLine) override { m_host.Log(aLine); }
    bool Open(const Map& aMap, const Topology& aTopo, bool aPost, uint32_t aBudget) override
    {
        m_map = aMap;
        m_topo = aTopo;
        m_post = aPost;
        m_budget = aBudget;
        for (uint64_t& d : m_droppedAttributed)
            d = 0;
        // Permanent mode only: segments left by earlier runs of the game would otherwise stay forever (the ring prunes this session's files).
        // The timed mode (keepMinutes 0) produces the files the player sends: it never deletes earlier ones.
        if (m_cfg.keepMinutes > 0 && !m_purged)
        {
            m_purged = true;
            m_storage.RemoveStale(m_cfg.ringPrefix, m_cfg.fileExtension, static_cast<int64_t>(m_cfg.keepMinutes) * 60000);
        }
        return OpenFile();
    }
    IByteSink* Sink(int aStream) override { return m_open && aStream >= 0 && aStream < 2 && (aStream == 0 || m_post) ? m_sink[aStream].get() : nullptr; }
    void Close(const Totals& aTotals) override
    {
        if (!m_open)
            return;
        FlushAll();
        Trailer t;
        t.failed = aTotals.failed || m_failed;
        for (int s = 0; s < 2; ++s)
        {
            t.frames[s] = m_segFrames[s];
            t.dropped[s] = aTotals.dropped[s] >= m_droppedAttributed[s] ? aTotals.dropped[s] - m_droppedAttributed[s] : 0;
        }
        CloseFile(t);
        m_open = false;
        Prune(m_host.NowMs());
    }
    std::string Describe() override
    {
        return m_files.empty() ? std::string("file=") : "file=" + m_files.back().display;
    }

    // ---- results (call from the control thread only after the writer thread has been joined) ----
    // Display paths of the files of this session that still exist, oldest first.
    std::vector<std::string> SessionFiles() const
    {
        std::vector<std::string> out;
        for (const auto& f : m_files)
            out.push_back(f.display);
        return out;
    }
    uint64_t TotalBytes() const
    {
        uint64_t n = m_currentBytes;
        if (m_cfg.keepMinutes > 0)
            for (const auto& f : m_files)
                if (!f.open)
                    n += f.bytes;
        return n;
    }
    bool Failed() const { return m_failed; }

private:
    struct FileRef
    {
        std::string name, display;
        int64_t closedMs = 0;
        uint64_t bytes = 0;
        bool open = false;
    };
    struct StreamBuf
    {
        std::vector<int64_t> meta;
        std::vector<float> values;
        uint32_t frames = 0;
        int64_t firstMs = 0;
    };
    class StreamSink final : public IByteSink
    {
    public:
        StreamSink(PoseEnv& aEnv, uint32_t aStream) : m_env(aEnv), m_stream(aStream) {}
        bool Write(const char* aData, size_t aSize) override { return m_env.AddFrame(m_stream, reinterpret_cast<const uint8_t*>(aData), aSize); }
        uint64_t Bytes() const override { return m_stream == kStreamLegacy ? m_env.TotalBytes() : 0; } // one shared file: counted once

    private:
        PoseEnv& m_env;
        uint32_t m_stream;
    };

    bool OpenFile()
    {
        const HostTime now = m_host.Time();
        std::unique_ptr<IPoseFile> file;
        std::string name;
        for (int suffix = 0; suffix < 100 && !file; ++suffix)
        {
            name = (m_cfg.keepMinutes > 0 ? m_cfg.ringPrefix : m_cfg.filePrefix) + now.stamp + (suffix ? "-" + std::to_string(suffix + 1) : std::string()) + m_cfg.fileExtension;
            file = m_storage.Create(name);
        }
        if (!file)
        {
            m_host.Log("UTPP pose capture: cannot create the capture file in " + m_storage.DisplayPath(std::string()));
            return false;
        }
        FileHeader h;
        h.algorithm = m_comp.Algorithm();
        h.segmentIndex = m_segmentIndex++;
        h.maxFrames = m_budget;
        h.qpcHz = now.qpcHz;
        h.armTicks = now.qpcTicks;
        for (int i = 0; i < 7; ++i)
            h.utc[i] = now.utc[i];
        h.rig = m_map.rig;
        h.rigCount = m_map.rigCount;
        h.hasStream[1] = m_post;
        h.hashes = m_topo.hashes;
        h.parents = m_topo.parents;
        h.gameIndex.assign(m_map.index.begin(), m_map.index.end());
        m_columns[0] = h.Columns(kStreamLegacy);
        m_columns[1] = h.Columns(kStreamPostHelper);
        std::vector<uint8_t> bytes;
        AppendFileHeader(bytes, h, m_map.index);
        if (!file->Write(bytes.data(), bytes.size()))
        {
            file->Close();
            m_storage.Remove(name);
            m_host.Log("UTPP pose capture: cannot write the capture file " + m_storage.DisplayPath(name));
            return false;
        }
        m_file = std::move(file);
        m_currentBytes = bytes.size();
        m_segStartMs = m_host.NowMs();
        m_failed = false;
        for (int s = 0; s < 2; ++s)
        {
            m_segFrames[s] = 0;
            m_buf[s] = StreamBuf{};
        }
        FileRef ref;
        ref.name = name;
        ref.display = m_storage.DisplayPath(name);
        ref.open = true;
        m_files.push_back(ref);
        m_open = true;
        return true;
    }
    void CloseFile(const Trailer& aTrailer)
    {
        if (!m_file)
            return;
        std::vector<uint8_t> bytes;
        AppendTrailer(bytes, aTrailer);
        if (!m_file->Write(bytes.data(), bytes.size()))
            m_failed = true;
        m_currentBytes += bytes.size();
        if (!m_file->Close())
            m_failed = true;
        m_file.reset();
        if (!m_files.empty())
        {
            m_files.back().open = false;
            m_files.back().closedMs = m_host.NowMs();
            m_files.back().bytes = m_currentBytes;
        }
        m_currentBytes = 0;
    }
    void Prune(int64_t aNowMs)
    {
        if (m_cfg.keepMinutes == 0)
            return;
#if defined(UTPP_PCF_MUTATION) && UTPP_PCF_MUTATION == 2 // mutation 2: old segments are never removed
        return;
#endif
        const int64_t horizon = aNowMs - static_cast<int64_t>(m_cfg.keepMinutes) * 60000;
        for (size_t i = 0; i < m_files.size();)
        {
            if (!m_files[i].open && m_files[i].closedMs < horizon)
            {
                m_storage.Remove(m_files[i].name);
                m_files.erase(m_files.begin() + static_cast<std::ptrdiff_t>(i));
            }
            else
                ++i;
        }
    }
    bool AddFrame(uint32_t aStream, const uint8_t* aRecord, size_t aSize)
    {
        if (!m_open || !m_file || m_failed || aSize != RecordBytes(aStream))
            return false;
        StreamBuf& b = m_buf[aStream];
        const uint32_t metaCount = MetaCount(aStream), stages = StageCount(aStream);
        if (b.frames == 0)
            b.firstMs = m_host.NowMs();
        const size_t metaAt = b.meta.size();
        b.meta.resize(metaAt + metaCount);
        std::memcpy(b.meta.data() + metaAt, aRecord, sizeof(int64_t) * metaCount);
        const uint8_t* values = aRecord + sizeof(int64_t) * metaCount;
        size_t at = b.values.size();
        b.values.resize(at + m_columns[aStream]);
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            if (m_map.index[n] < 0)
                continue;
            std::memcpy(b.values.data() + at, values + size_t(n) * stages * 12 * sizeof(float), size_t(stages) * 12 * sizeof(float));
            at += size_t(stages) * 12;
        }
        ++b.frames;
        ++m_segFrames[aStream];
        const int64_t now = m_host.NowMs();
        if (b.frames >= m_cfg.blockFrames || now - b.firstMs >= m_cfg.blockMaxMs)
            FlushBlock(aStream);
        if (m_cfg.keepMinutes > 0 && !m_failed && now - m_segStartMs >= m_cfg.segmentMs)
            Roll();
        return !m_failed;
    }
    void FlushBlock(uint32_t aStream)
    {
        StreamBuf& b = m_buf[aStream];
        if (b.frames == 0 || !m_file)
            return;
        EncodeBlockPayload(aStream, b.frames, m_columns[aStream], b.meta.data(), b.values.data(), m_raw);
        std::vector<uint8_t> out;
        if (!AppendDataBlock(out, aStream, b.frames, m_raw, m_comp, m_scratch) || !m_file->Write(out.data(), out.size()))
            m_failed = true;
        else
            m_currentBytes += out.size();
        b.meta.clear();
        b.values.clear();
        b.frames = 0;
    }
    void FlushAll()
    {
        for (uint32_t s = 0; s < kStreamCount; ++s)
            FlushBlock(s);
    }
    // Closes the running segment (trailer with its frame count and the dropped frames since the previous one) and opens the next.
    void Roll()
    {
        FlushAll();
        uint64_t dropped[2] = {m_droppedAttributed[0], m_droppedAttributed[1]};
        if (m_probe)
            m_probe(dropped);
        Trailer t;
        t.failed = m_failed;
        for (int s = 0; s < 2; ++s)
        {
            t.frames[s] = m_segFrames[s];
            t.dropped[s] = dropped[s] >= m_droppedAttributed[s] ? dropped[s] - m_droppedAttributed[s] : 0;
            m_droppedAttributed[s] = dropped[s];
        }
        CloseFile(t);
        m_open = false;
        Prune(m_host.NowMs());
        if (!OpenFile())
            m_failed = true;
    }

    IPoseHost& m_host;
    IPoseStorage& m_storage;
    ICompressor& m_comp;
    Config m_cfg;
    DroppedProbe m_probe;
    std::unique_ptr<StreamSink> m_sink[2];

    Map m_map;
    Topology m_topo;
    bool m_post = false, m_open = false, m_failed = false, m_purged = false;
    uint32_t m_budget = 0, m_segmentIndex = 0;
    uint32_t m_columns[2] = {0, 0};
    std::unique_ptr<IPoseFile> m_file;
    std::vector<FileRef> m_files;
    uint64_t m_currentBytes = 0;
    int64_t m_segStartMs = 0;
    uint64_t m_segFrames[2] = {0, 0};
    uint64_t m_droppedAttributed[2] = {0, 0};
    StreamBuf m_buf[2];
    std::vector<uint8_t> m_raw, m_scratch;
};

// ---- ring adapters (drain a capture ring into PoseEnv records) ------------------------------------------------------------------------------
inline void FillRecord(const bodytrace::Frame& f, std::vector<uint8_t>& aRecord)
{
    aRecord.resize(RecordBytes(kStreamLegacy));
    const int64_t meta[4] = {f.ticks, static_cast<int64_t>(f.desc), static_cast<int64_t>(f.thread), static_cast<int64_t>(f.generation)};
    std::memcpy(aRecord.data(), meta, sizeof(meta));
    uint8_t* dst = aRecord.data() + sizeof(meta);
    for (uint32_t n = 0; n < kMaxBones; ++n)
        for (uint32_t s = 0; s < 2; ++s)
        {
            std::memcpy(dst, f.stage[s][n], 12 * sizeof(float));
            dst += 12 * sizeof(float);
        }
}
inline void FillRecord(const bodytrace::PostHelperFrame& f, std::vector<uint8_t>& aRecord)
{
    aRecord.resize(RecordBytes(kStreamPostHelper));
    const int64_t meta[8] = {f.ticks,
                             static_cast<int64_t>(f.desc),
                             static_cast<int64_t>(f.thread),
                             static_cast<int64_t>(f.generation),
                             static_cast<int64_t>(f.receiptGeneration),
                             static_cast<int64_t>(f.receiptCycle),
                             f.receiptStartedTicks,
                             f.receiptLastTicks};
    std::memcpy(aRecord.data(), meta, sizeof(meta));
    std::memcpy(aRecord.data() + sizeof(meta), f.local, sizeof(float) * kMaxBones * 12);
}

template <class Cap, class FrameT>
class PoseCaptureStream final : public ITraceStream
{
public:
    explicit PoseCaptureStream(Cap& aCap) : m_cap(aCap), m_frame(std::make_unique<FrameT>()) {}
    bool Live() const override { return m_cap.Live(); }
    bool Arm(const Map& aMap, uint32_t aBudget) override
    {
        const bool ok = m_cap.Arm(aMap, aBudget);
        if (ok)
            m_gaps = Gaps{};
        return ok;
    }
    // A ring whose budget ran out while it was live recorded nothing until this refill: that gap is a loss the ring does not count itself.
    bool Refill(uint32_t aBudget, uint32_t aBelow) override
    {
        if (m_cap.Live() && !m_cap.Active())
            ++m_gaps.pending;
        return m_cap.Refill(aBudget, aBelow);
    }
    bool Cancel() override { return m_cap.Cancel(); }
    // Frames the ring refused, plus the frames missing in budget gaps (sized from the timestamps around the gap; one at least, until the
    // gap shows in the drained frames). Never decreases while the ring stays armed.
    uint64_t Dropped() const override { return m_cap.Dropped() + m_gaps.lost + m_gaps.pending; }
    uint32_t Drain(uint32_t aMax, IByteSink& aSink, bool* aFailed) override
    {
        uint32_t n = 0;
        while (n < aMax && m_cap.Pop(*m_frame))
        {
            ++n;
            m_gaps.Observe(m_frame->ticks);
            FillRecord(*m_frame, m_record);
            if (!aSink.Write(reinterpret_cast<const char*>(m_record.data()), m_record.size()) && aFailed)
                *aFailed = true;
        }
        return n;
    }

private:
    // Estimates the frames lost in a budget gap: the first timestamp step above 2.5 x the running average after the gap was noticed.
    struct Gaps
    {
        uint64_t pending = 0, lost = 0;
        int64_t prevTicks = 0;
        double average = 0.0;
        void Observe(int64_t aTicks)
        {
            if (prevTicks > 0)
            {
                const double step = static_cast<double>(aTicks - prevTicks);
                if (pending > 0 && average > 0.0 && step > 2.5 * average)
                {
                    lost += static_cast<uint64_t>(std::max(1.0, std::floor(step / average + 0.5) - 1.0));
                    --pending;
                }
                else if (step > 0.0)
                    average = average > 0.0 ? average + (step - average) / 16.0 : step;
            }
            prevTicks = aTicks;
        }
    };
    Cap& m_cap;
    std::unique_ptr<FrameT> m_frame;
    std::vector<uint8_t> m_record;
    Gaps m_gaps;
};

// Controller settings of the compact capture: the budget is topped up on every pass (the writer thread may be busy compressing a block
// for a while: the full budget is the slack before the rings stop recording), the size cap is sized for this format.
inline bodytrace::ContinuousController::Config CompactControllerConfig()
{
    bodytrace::ContinuousController::Config c;
    c.budget = bodytrace::kMaxFrames;
    c.refillBelow = bodytrace::kMaxFrames;
    c.maxBytes = 2ull << 30;
    c.logPrefix = "UTPP pose capture";
    return c;
}
} // namespace utpp::mm::posecapture

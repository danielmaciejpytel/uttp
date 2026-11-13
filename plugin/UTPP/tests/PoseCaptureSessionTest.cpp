// Compact pose capture, session layer (src/mm/PoseCaptureSession.hpp) driven by the real capture rings and the CSV capture's controller, with
// in-memory files, a fake clock and a fake compressor. Single-threaded: the test plays the FK hook (Begin / End) and the writer thread (Tick).
//   A) a whole session (keepMinutes 0): one file, every frame (both streams) with exact ticks and values within tolerance, trailer counts,
//      blocks cut by frame count and by time;
//   B) keepMinutes > 0: ~1 minute segments, old segments deleted as soon as they are older than the window, every retained segment decodes on
//      its own (own header, key frame, trailer), the window is covered up to the last frame, file list == files on disk;
//   C) rig change: the file is closed (trailer) and a new one opens with the new topology; the same map again changes nothing;
//   D) hard size cap: capture stops with one log line, the file stays valid;
//   E) lost frames: a writer stall shorter than the budget loses nothing; a longer one is counted (ring overflow or a budget gap) and the file
//      trailer carries the same number;
//   G) stale files of earlier runs: ring segments (utpp-pose-ring-*) removed when a keepMinutes > 0 capture starts; files of the timed mode
//      (utpp-pose-*) never removed by either mode; others untouched;
//   F) log lines carry the compact capture's own prefix and never an absolute path.
// Mutation UTPP_PCF_MUTATION 2 (old segments are never removed) must print a FAIL line.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/PoseCaptureSession.hpp"

using namespace utpp::mm::posecapture;
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

struct FakeHost final : IPoseHost
{
    int64_t nowMs = 100000;
    std::vector<std::string> logs;
    int64_t NowMs() override { return nowMs; }
    HostTime Time() override
    {
        HostTime t;
        const int64_t s = nowMs / 1000;
        char b[48];
        std::snprintf(b, sizeof(b), "2026-11-10-%02d-%02d-%02d", static_cast<int>((s / 3600) % 24), static_cast<int>((s / 60) % 60), static_cast<int>(s % 60));
        t.stamp = b;
        t.utc[3] = static_cast<uint16_t>((s / 3600) % 24);
        t.utc[4] = static_cast<uint16_t>((s / 60) % 60);
        t.utc[5] = static_cast<uint16_t>(s % 60);
        t.utc[6] = static_cast<uint16_t>(nowMs % 1000);
        t.qpcTicks = nowMs * 10000;
        return t;
    }
    void Log(const std::string& aLine) override { logs.push_back(aLine); }
    int Count(const std::string& aNeedle) const
    {
        int n = 0;
        for (const auto& l : logs)
            n += l.find(aNeedle) != std::string::npos;
        return n;
    }
};
struct MemFile final : IPoseFile
{
    std::vector<uint8_t>* data;
    explicit MemFile(std::vector<uint8_t>* aData) : data(aData) {}
    bool Write(const uint8_t* aData, size_t aSize) override { data->insert(data->end(), aData, aData + aSize); return true; }
    bool Close() override { return true; }
};
struct MemStorage final : IPoseStorage
{
    std::map<std::string, std::vector<uint8_t>> files;
    std::unique_ptr<IPoseFile> Create(const std::string& aName) override
    {
        if (files.count(aName))
            return nullptr;
        return std::make_unique<MemFile>(&files[aName]);
    }
    bool Remove(const std::string& aName) override { return files.erase(aName) > 0; }
    std::map<std::string, int64_t> ageMs; // age of files that existed before the capture started (others count as brand new)
    int staleCalls = 0;
    void RemoveStale(const std::string& aPrefix, const std::string& aExt, int64_t aOlderThanMs) override
    {
        ++staleCalls;
        for (auto it = files.begin(); it != files.end();)
        {
            const std::string& n = it->first;
            const auto a = ageMs.find(n);
            const bool match = n.size() >= aPrefix.size() + aExt.size() && n.compare(0, aPrefix.size(), aPrefix) == 0 &&
                               n.compare(n.size() - aExt.size(), aExt.size(), aExt) == 0;
            if (match && a != ageMs.end() && a->second > aOlderThanMs)
                it = files.erase(it);
            else
                ++it;
        }
    }
    std::string DisplayPath(const std::string& aName) const override { return "red4ext\\logs\\" + aName; }
};

constexpr uint32_t kRigCount = 60;
constexpr uint32_t kMappedBones = 8;

float Gen(uint64_t aFrame, uint32_t aRigBone, uint32_t aV, uint32_t aStage)
{
    return static_cast<float>(aRigBone) + 0.1f * static_cast<float>(aV) + 0.003f * static_cast<float>(aFrame % 200) * static_cast<float>(1 + aStage);
}

struct Session
{
    FakeHost host;
    MemStorage storage;
    StoreCompressor store;
    Capture cap;
    PostHelperCapture post;
    PoseCaptureStream<Capture, Frame> legacy{cap};
    PoseCaptureStream<PostHelperCapture, PostHelperFrame> postStream{post};
    ICompressor* comp = &store;
    std::unique_ptr<PoseEnv> env;
    std::unique_ptr<ContinuousController> ctrl;
    Map map;
    std::shared_ptr<Topology> topo;
    std::vector<uint8_t> bytesBegin = std::vector<uint8_t>(size_t(kRigCount) * 48), bytesEnd = std::vector<uint8_t>(size_t(kRigCount) * 48);
    uint64_t frameNo = 0;
    int64_t ticks = 6000000000000ll;
    std::vector<int64_t> producedTicks;
    bool postEnabled = false;

    void Start(uint32_t aKeepMinutes, bool aPost, uint32_t aBlockFrames = 30, int64_t aBlockMaxMs = 5000, uint64_t aMaxBytes = 0, uint64_t aRig = 1)
    {
        PoseEnv::Config cfg;
        cfg.keepMinutes = aKeepMinutes;
        cfg.blockFrames = aBlockFrames;
        cfg.blockMaxMs = aBlockMaxMs;
        env = std::make_unique<PoseEnv>(host, storage, *comp, cfg);
        auto cc = CompactControllerConfig();
        if (aMaxBytes)
            cc.maxBytes = aMaxBytes;
        ctrl = std::make_unique<ContinuousController>(*env, legacy, postStream, cc);
        env->SetDroppedProbe([this](uint64_t (&aDropped)[2]) {
            const auto s = ctrl->GetStats();
            aDropped[0] = s.dropped[0];
            aDropped[1] = s.dropped[1];
        });
        postEnabled = aPost;
        SetRig(aRig, aPost);
        ctrl->Enable(true);
        ctrl->Tick(); // starts the session, arms the rings
    }
    void SetRig(uint64_t aRig, bool aPost)
    {
        map = Map{};
        map.rig = aRig;
        map.rigCount = kRigCount;
        for (uint32_t i = 0; i < kMappedBones; ++i)
            map.index[i * 20 % kMaxBones] = static_cast<int32_t>(i * 3);
        topo = std::make_shared<Topology>();
        for (uint32_t i = 0; i < kRigCount; ++i)
        {
            topo->hashes.push_back(aRig * 1000003ull + i);
            topo->parents.push_back(static_cast<int16_t>(i == 0 ? -1 : i - 1));
        }
        ctrl->SetMap(map, aPost, topo);
    }
    // One FK call pair; advances the fake clock by dtMs.
    bool Produce(int64_t aDtMs = 16)
    {
        host.nowMs += aDtMs;
        ticks += aDtMs * 10000;
        for (uint32_t i = 0; i < kRigCount; ++i)
            for (uint32_t v = 0; v < 12; ++v)
            {
                const float b = Gen(frameNo, i, v, 0), e = Gen(frameNo, i, v, 1);
                std::memcpy(&bytesBegin[size_t(i) * 48 + v * 4], &b, 4);
                std::memcpy(&bytesEnd[size_t(i) * 48 + v * 4], &e, 4);
            }
        static Frame frame;
        const bool begun = cap.Begin(frame, bytesBegin.data(), kRigCount, map.rig, 77, ticks, 1000 + static_cast<uint32_t>(frameNo % 3));
        const bool ended = begun && cap.End(frame, bytesEnd.data(), kRigCount, map.rig, 77);
        if (postEnabled)
            post.Capture(bytesEnd.data(), kRigCount, map.rig, 77, ticks, 1000, 5, 6, ticks - 100, ticks - 50);
        if (ended)
            producedTicks.push_back(ticks);
        ++frameNo;
        return ended;
    }
    void Run(uint32_t aFrames)
    {
        for (uint32_t i = 0; i < aFrames; ++i)
        {
            Produce();
            ctrl->Tick();
        }
    }
    void Stop()
    {
        ctrl->Shutdown();
        ctrl->Tick();
    }
    DecodedFile Read(const std::string& aDisplay)
    {
        const std::string name = aDisplay.substr(aDisplay.find_last_of('\\') + 1);
        const auto& d = storage.files.at(name);
        return DecodeFile(d.data(), d.size(), *comp);
    }
};

bool NoAbsolutePath(const std::vector<std::string>& aLogs)
{
    for (const auto& l : aLogs)
        if (l.find(":\\") != std::string::npos || l.find(":/") != std::string::npos || l.find("\\\\") != std::string::npos)
            return false;
    return true;
}

void WholeSession()
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    s.Start(0, true, 30, 5000);
    const size_t headerBytes = s.storage.files.begin()->second.size(); // the header is written when the file opens
    s.Run(100);
    Check(s.storage.files.size() == 1, "A: keepMinutes 0 creates one file");
    // blocks written during the run: 3 full blocks of 30 frames per stream
    Check(s.storage.files.begin()->second.size() > headerBytes + 20000, "A: full blocks are written while the capture runs (not only at the end)");
    s.Stop();
    const auto files = s.env->SessionFiles();
    Check(files.size() == 1 && files[0].rfind("red4ext\\logs\\utpp-pose-", 0) == 0 && files[0].find(".utpppose") != std::string::npos,
          "A: result path is relative and named utpp-pose-<stamp>.utpppose (" + (files.empty() ? std::string("none") : files[0]) + ")");
    const DecodedFile d = s.Read(files[0]);
    Check(d.headerOk && !d.truncated && !d.corrupt, "A: file decodes");
    Check(d.trailer.present && d.trailer.frames[0] == 100 && d.trailer.frames[1] == 100 && d.trailer.dropped[0] == 0 && !d.trailer.failed,
          F("A: trailer frames %llu / %llu, dropped %llu", static_cast<unsigned long long>(d.trailer.frames[0]), static_cast<unsigned long long>(d.trailer.frames[1]),
            static_cast<unsigned long long>(d.trailer.dropped[0])));
    Check(d.FramesOf(0) == 100 && d.FramesOf(1) == 100, F("A: %llu legacy + %llu post-helper frames in the blocks", static_cast<unsigned long long>(d.FramesOf(0)), static_cast<unsigned long long>(d.FramesOf(1))));
    // timestamps exact, in order, equal to the produced ones
    std::vector<int64_t> got;
    std::vector<uint32_t> sizes;
    for (const auto& b : d.blocks)
        if (b.stream == 0)
        {
            sizes.push_back(b.frames);
            for (uint32_t f = 0; f < b.frames; ++f)
                got.push_back(b.meta[size_t(f) * 4]);
        }
    Check(got == s.producedTicks, "A: every timestamp equals the one produced (exact QPC ticks)");
    Check(sizes.size() == 4 && sizes[0] == 30 && sizes[3] == 10, F("A: legacy blocks cut by frame count: %zu blocks (30,30,30,10)", sizes.size()));
    // values
    double worst = 0.0;
    size_t frameAt = 0;
    for (const auto& b : d.blocks)
    {
        if (b.stream != 0)
            continue;
        for (uint32_t f = 0; f < b.frames; ++f, ++frameAt)
        {
            uint32_t col = 0;
            for (uint32_t n = 0; n < kMaxBones; ++n)
                if (s.map.index[n] >= 0)
                    for (uint32_t st = 0; st < 2; ++st)
                        for (uint32_t v = 0; v < 12; ++v, ++col)
                            worst = std::max(worst, static_cast<double>(std::fabs(b.values[size_t(f) * b.columns + col] - Gen(frameAt, static_cast<uint32_t>(s.map.index[n]), v, st))));
        }
    }
    Check(worst < 2e-5, F("A: values within the quantization error (worst %.2e)", worst));
    Check(d.header.hasStream[1] && d.header.rigCount == kRigCount && d.header.hashes.size() == kRigCount && d.header.hashes[5] == 1 * 1000003ull + 5, "A: header has the post-helper stream and the rig topology");
    Check(s.host.Count("UTPP pose capture: started file=red4ext\\logs\\utpp-pose-") == 1 && s.host.Count("UTPP body trace continuous") == 0, "F: log lines use the compact capture's prefix and a relative file path");
    Check(NoAbsolutePath(s.host.logs), "F: no absolute path in any log line");
}

void BlockByTime()
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    s.Start(0, false, 1000, 5000);
    const size_t headerBytes = s.storage.files.begin()->second.size();
    for (int i = 0; i < 4; ++i)
    {
        s.Produce(2500); // one frame every 2.5 s: the 5 s limit cuts the block at the third frame
        s.ctrl->Tick();
    }
    const size_t sizeBefore = s.storage.files.begin()->second.size();
    s.Stop();
    const DecodedFile d = s.Read(s.env->SessionFiles()[0]);
    Check(d.FramesOf(0) == 4 && d.blocks.size() == 2 && d.blocks[0].frames == 3, F("A: block cut by time: blocks %zu (3 + 1 frames)", d.blocks.size()));
    Check(sizeBefore > headerBytes + 1000, "A: the time-cut block is already in the file before the capture stops");
}

void Segments()
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    s.Start(2, false, 30, 5000);
    for (uint32_t i = 0; i < 300 * 60; ++i) // 5 minutes at 60 Hz (16 ms steps)
    {
        s.Produce(16);
        s.ctrl->Tick();
    }
    const int64_t now = s.host.nowMs;
    s.Stop();
    const auto files = s.env->SessionFiles();
    Check(s.storage.files.size() == files.size() && files.size() >= 2, F("B: file list == files on disk (%zu)", files.size()));
    Check(files.size() <= 4, F("B: segments older than the 2-minute window are deleted (%zu left of ~5)", files.size()));
    int64_t firstTick = INT64_MAX, lastTick = 0;
    bool selfContained = true, ordered = true;
    uint32_t prevIndex = 0;
    bool firstSeg = true;
    uint64_t frames = 0;
    for (const auto& f : files)
    {
        const DecodedFile d = s.Read(f);
        selfContained = selfContained && d.headerOk && !d.truncated && !d.corrupt && d.trailer.present && !d.blocks.empty() && d.header.hashes.size() == kRigCount;
        if (!firstSeg)
            ordered = ordered && d.header.segmentIndex > prevIndex;
        firstSeg = false;
        prevIndex = d.header.segmentIndex;
        frames += d.FramesOf(0);
        for (const auto& b : d.blocks)
        {
            firstTick = std::min(firstTick, b.meta[0]);
            lastTick = std::max(lastTick, b.meta[size_t(b.frames - 1) * 4]);
        }
        // a key frame: the first frame of a segment decodes without any other file (values near Gen)
        const auto& b0 = d.blocks.front();
        if (std::fabs(b0.values[0] - Gen(0, 0, 0, 0)) > 100.0f)
            selfContained = false;
    }
    Check(selfContained, "B: every retained segment decodes on its own (header + topology, blocks, trailer)");
    Check(ordered, "B: segment indices increase along the retained files");
    const int64_t lastProduced = s.producedTicks.back();
    Check(lastTick == lastProduced, "B: the retained files run up to the last frame");
    const double windowSeconds = static_cast<double>(lastTick - firstTick) / 1e7;
    Check(windowSeconds >= 120.0, F("B: retained files cover %.1f s (at least the 120 s window)", windowSeconds));
    Check(windowSeconds <= 190.0, F("B: retained files do not keep much more than the window (%.1f s)", windowSeconds));
    (void)now;
    (void)frames;
}

// Earlier runs leave files behind: the permanent mode (keepMinutes > 0) removes only its OWN ring segments (utpp-pose-ring-*) older than the
// window when it starts; files of the timed mode (utpp-pose-*, the ones the player sends) are never removed by either mode; other names stay.
void StaleFiles(uint32_t aKeepMinutes)
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    const std::string ringStale = "utpp-pose-ring-2026-11-09-10-00-00.utpppose", ringFresh = "utpp-pose-ring-2026-11-10-09-58-00.utpppose";
    const std::string timedStale = "utpp-pose-2026-11-09-10-00-00.utpppose";
    const std::string otherExt = "utpp-pose-ring-2026-11-09-10-00-00.csv", otherName = "notes.utpppose";
    for (const std::string& n : {ringStale, ringFresh, timedStale, otherExt, otherName})
        s.storage.files[n] = {1, 2, 3};
    s.storage.ageMs[ringStale] = 3ll * 3600 * 1000;
    s.storage.ageMs[ringFresh] = 4ll * 60 * 1000; // younger than the 10-minute window
    s.storage.ageMs[timedStale] = 3ll * 3600 * 1000;
    s.storage.ageMs[otherExt] = 3ll * 3600 * 1000;
    s.storage.ageMs[otherName] = 3ll * 3600 * 1000;
    s.Start(aKeepMinutes, false, 30, 5000);
    s.Run(40);
    s.Stop();
    const std::string tag = aKeepMinutes ? "G: permanent mode: " : "G: timed mode: ";
    Check(s.storage.staleCalls == (aKeepMinutes ? 1 : 0), tag + F("stale cleanup runs %d time(s)", s.storage.staleCalls));
    Check((s.storage.files.count(ringStale) == 0) == (aKeepMinutes > 0), tag + (aKeepMinutes ? "the stale ring segment of an earlier run is removed" : "the earlier ring segment is kept"));
    Check(s.storage.files.count(timedStale) == 1, tag + "a stale file of the timed mode survives (the mod never removes it)");
    Check(s.storage.files.count(ringFresh) == 1, tag + "a ring segment inside the window is kept");
    Check(s.storage.files.count(otherExt) == 1 && s.storage.files.count(otherName) == 1, tag + "files that do not match the capture name are kept");
    const auto sessionFiles = s.env->SessionFiles();
    const std::string expectPrefix = aKeepMinutes ? "red4ext\\logs\\utpp-pose-ring-" : "red4ext\\logs\\utpp-pose-2";
    Check(sessionFiles.size() == 1 && sessionFiles[0].rfind(expectPrefix, 0) == 0 && s.storage.files.size() == (aKeepMinutes ? 5u : 6u),
          tag + F("this run's file is named with the %s prefix (%zu files in the folder)", aKeepMinutes ? "ring" : "timed", s.storage.files.size()));
}

void RigChange()
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    s.Start(0, false, 30, 5000);
    s.Run(40);
    s.ctrl->SetMap(s.map, false, s.topo); // the same map again: a no-op
    s.Run(10);
    Check(s.storage.files.size() == 1, "C: the same map again keeps the file");
    s.SetRig(2, false);
    s.host.nowMs += 1500; // the new file gets its own stamp
    s.Run(1);
    s.Run(25);
    s.Stop();
    const auto files = s.env->SessionFiles();
    Check(files.size() == 2, F("C: a rig change opens a second file (%zu)", files.size()));
    if (files.size() != 2)
        return;
    const DecodedFile a = s.Read(files[0]), b = s.Read(files[1]);
    Check(a.headerOk && b.headerOk && a.header.rig == 1 && b.header.rig == 2 && a.header.hashes[0] != b.header.hashes[0], "C: each file carries its own rig and topology");
    Check(a.trailer.present && b.trailer.present && a.trailer.frames[0] == 50 && b.trailer.frames[0] >= 25, F("C: frames per file %llu / %llu", static_cast<unsigned long long>(a.trailer.frames[0]), static_cast<unsigned long long>(b.trailer.frames[0])));
    Check(b.header.segmentIndex == a.header.segmentIndex + 1, "C: the second file follows in segmentIndex");
}

void SizeCap()
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    s.Start(0, false, 10, 5000, 3000);
    uint64_t guard = 0;
    while (s.ctrl->GetStats().capped == false && guard++ < 2000)
        s.Run(1);
    Check(s.ctrl->GetStats().capped, "D: the size cap stops the capture");
    Check(s.host.Count("size cap reached") == 1, "D: one log line for the cap");
    const auto files = s.env->SessionFiles();
    Check(files.size() == 1, "D: one file");
    if (!files.empty())
    {
        const DecodedFile d = s.Read(files[0]);
        Check(d.headerOk && d.trailer.present && !d.truncated && d.FramesOf(0) > 0, F("D: the capped file is valid (%llu frames, trailer present)", static_cast<unsigned long long>(d.FramesOf(0))));
        const uint64_t before = s.storage.files.begin()->second.size();
        s.Run(50);
        Check(s.storage.files.begin()->second.size() == before, "D: nothing is written after the cap");
    }
}

// The fake FK thread keeps producing while the writer thread is stuck in a (slow) compression call.
class StallCompressor final : public ICompressor
{
public:
    std::function<void()> onCompress;
    uint32_t Algorithm() const override { return 99; }
    bool Compress(const uint8_t* aSrc, size_t aSize, std::vector<uint8_t>& aOut) override
    {
        if (onCompress)
            onCompress();
        aOut.assign(aSrc, aSrc + aSize);
        aOut.push_back(0); // not smaller: stored as is
        return true;
    }
    bool Decompress(const uint8_t*, size_t, size_t, std::vector<uint8_t>&) override { return false; }
};

void LostFrames(uint32_t aStallFrames, bool aExpectLoss)
{
    auto sessionHolder = std::make_unique<Session>(); // the rings are megabytes: heap
    Session& s = *sessionHolder;
    StallCompressor stall;
    s.comp = &stall;
    s.Start(0, false, 20, 5000);
    s.Run(10);
    bool armed = true;
    stall.onCompress = [&] {
        if (!armed)
            return;
        armed = false;
        for (uint32_t i = 0; i < aStallFrames; ++i) // frames produced while the writer cannot drain
            s.Produce();
    };
    s.Run(15); // the 20th frame completes a block -> the compression stalls the writer
    s.Run(60); // writer catches up
    s.Stop();
    const auto stats = s.ctrl->GetStats();
    const uint64_t dropped = stats.lifetimeDropped[0];
    if (aExpectLoss)
    {
        const uint64_t refused = s.frameNo - s.producedTicks.size();
        Check(dropped > 0, F("E: a stall of %u frames (more than the budget) is counted as lost frames (%llu)", aStallFrames, static_cast<unsigned long long>(dropped)));
        Check(dropped + 2 >= refused && dropped <= refused + 2, F("E: the count matches the frames refused during the stall (%llu counted, %llu refused)", static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(refused)));
    }
    else
        Check(dropped == 0, F("E: a stall of %u frames loses nothing (%llu counted)", aStallFrames, static_cast<unsigned long long>(dropped)));
    // frames the producer got into the rings == frames in the file + frames never drained; the counted loss explains the missing ones
    const uint64_t produced = s.producedTicks.size();
    const uint64_t written = stats.lifetimeFrames[0];
    Check(written == produced, F("E: every frame the rings accepted reached the writer (%llu of %llu)", static_cast<unsigned long long>(written), static_cast<unsigned long long>(produced)));
    if (aExpectLoss)
    {
        const uint64_t attempted = s.frameNo;
        Check(attempted > produced, F("E: frames the rings refused during the stall: %llu", static_cast<unsigned long long>(attempted - produced)));
    }
    // the file trailer carries the same lost-frame count
    const auto files = s.env->SessionFiles();
    if (!files.empty() && s.storage.files.size() == 1)
    {
        const auto& bytes = s.storage.files.begin()->second;
        // the test compressor cannot decode: read the trailer from the file end (kind 2, u32 flags, 2 x u64 frames, 2 x u64 dropped = 37 bytes)
        const size_t tail = bytes.size() >= 37 ? bytes.size() - 37 : 0;
        ByteReader r(bytes.data() + tail, 37);
        const uint8_t kind = r.U8();
        r.U32();
        r.U64(); r.U64();
        const uint64_t trailerDropped = r.U64();
        Check(kind == 2 && trailerDropped == dropped, F("E: trailer dropped == controller count (%llu)", static_cast<unsigned long long>(trailerDropped)));
    }
}
} // namespace

int main()
{
    WholeSession();
    BlockByTime();
    Segments();
    StaleFiles(10);
    StaleFiles(0);
    RigChange();
    SizeCap();
    LostFrames(100, false);
    LostFrames(300, true);
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

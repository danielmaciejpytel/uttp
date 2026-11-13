// End-to-end test of the production compact pose capture (src/PoseCaptureRuntime.cpp): the real capture rings, a producer thread playing the
// FK hook (Begin / End at ~120 Hz, 185 bones, both stages + the post-helper ring), the real writer thread, real files and the Windows
// Compression API (XPRESS with Huffman).
//   - Start / Stop / Status, idempotent Start, refusal before Init; the result is a '|' list of paths RELATIVE to the game folder
//   - the file decodes (header: clock, rig, topology, bone names; blocks; trailer), every frame the rings accepted is in it, exact ticks,
//     nothing dropped at this rate, values within the quantization error, the post-helper stream is in the same file
//   - a rig change closes the file and opens a second one (both listed)
//   - coexistence: the text capture (BodyTraceRuntime) and the compact capture refuse each other, in both orders
//   - a session without a rig writes no file (Stop returns an empty string)
// Usage: utpp_pose_capture_runtime_test <scratch folder>   (the files are left there)
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../src/BodyTraceRuntime.hpp"
#include "../src/PoseCaptureCompression.hpp"
#include "../src/PoseCaptureRuntime.hpp"

namespace fs = std::filesystem;
using namespace utpp::mm::bodytrace;
namespace pcf = utpp::mm::posecapture;

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
    char b[640];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

std::mutex g_logMutex;
std::vector<std::string> g_logs;
void Log(const char* aLine)
{
    std::lock_guard lock(g_logMutex);
    g_logs.push_back(aLine);
}
int CountLogs(const char* aNeedle)
{
    std::lock_guard lock(g_logMutex);
    int c = 0;
    for (const auto& l : g_logs)
        c += l.find(aNeedle) != std::string::npos;
    return c;
}

constexpr uint32_t kRigCount = 185;
std::atomic<uint64_t> g_rig{1};
std::atomic<bool> g_stop{false};
std::mutex g_tickMutex;
std::vector<int64_t> g_capturedTicks;  // ticks of the frames the legacy ring accepted
std::atomic<uint64_t> g_postCaptured{0};

Map MakeMap(uint64_t aRig)
{
    std::vector<std::string_view> names(kNames.begin(), kNames.end());
    Map m;
    Resolve(names, aRig, m);
    return m;
}
float BoneValue(size_t aFloat) { return static_cast<float>(aFloat % 977) * 0.01f - 3.0f; }
void Producer(double aHz)
{
    std::vector<uint8_t> bones(kRigCount * 48, 0), bones2(kRigCount * 48, 0);
    for (size_t i = 0; i < bones.size() / 4; ++i)
    {
        const float v = BoneValue(i), w = v + 0.5f;
        std::memcpy(bones.data() + i * 4, &v, 4);
        std::memcpy(bones2.data() + i * 4, &w, 4);
    }
    auto frame = std::make_unique<Frame>();
    const auto start = std::chrono::steady_clock::now();
    uint64_t n = 0;
    while (!g_stop.load())
    {
        const auto due = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(static_cast<double>(n) / aHz));
        while (std::chrono::steady_clock::now() < due && !g_stop.load())
            std::this_thread::yield();
        ++n;
        const uint64_t rig = g_rig.load();
        const int64_t ticks = 6000000000000ll + static_cast<int64_t>(n) * 83333;
        if (g_capture.Active() && g_capture.Begin(*frame, bones.data(), kRigCount, rig, 0xD0, ticks, 7) &&
            g_capture.End(*frame, bones2.data(), kRigCount, rig, 0xD0))
        {
            std::lock_guard lock(g_tickMutex);
            g_capturedTicks.push_back(ticks);
        }
        if (g_postHelperCapture.Active() && g_postHelperCapture.Capture(bones2.data(), kRigCount, rig, 0xD0, ticks, 7, 1, 1, ticks - 5, ticks - 1))
            ++g_postCaptured;
    }
}
void SleepMs(int aMs) { std::this_thread::sleep_for(std::chrono::milliseconds(aMs)); }

std::vector<std::string> Split(const std::string& aList)
{
    std::vector<std::string> out;
    size_t at = 0;
    while (at <= aList.size() && !aList.empty())
    {
        const size_t bar = aList.find('|', at);
        out.push_back(aList.substr(at, bar == std::string::npos ? std::string::npos : bar - at));
        if (bar == std::string::npos)
            break;
        at = bar + 1;
    }
    return out;
}
std::vector<uint8_t> ReadAll(const fs::path& aPath)
{
    std::ifstream in(aPath, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
bool HasAbsolutePath(const std::string& aText)
{
    return aText.find(":\\") != std::string::npos || aText.find(":/") != std::string::npos || aText.find("\\\\") != std::string::npos;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_pose_capture_runtime_test <scratch folder>\n");
        return 2;
    }
    const fs::path game = fs::absolute(argv[1]);
    const fs::path logs = game / "red4ext" / "logs";
    std::error_code ec;
    fs::remove_all(game, ec);
    fs::create_directories(logs);
    utpp::posecapture::WindowsCompressor reader(pcf::kAlgorithmXpress);
    Check(reader.Valid(), "the Windows compression API is available (XPRESS with Huffman)");

    std::vector<uint64_t> hashes(kRigCount);
    std::vector<int16_t> parents(kRigCount);
    for (uint32_t i = 0; i < kRigCount; ++i)
    {
        hashes[i] = 0xABCD0000ull + i;
        parents[i] = i == 0 ? -1 : static_cast<int16_t>(i - 1);
    }

    Check(!utpp::posecaptureruntime::Start(0), "not initialised -> Start refused");
    Check(utpp::posecaptureruntime::Status() == "poseCapture state=off", "Status before any session: off");
    utpp::posecaptureruntime::Init(&Log, logs.wstring());
    utpp::bodytraceruntime::Init(&Log, logs.wstring());
    Check(!utpp::posecaptureruntime::Start(-1) && !utpp::posecaptureruntime::Start(100000), "a negative or absurd window is refused");
    std::thread producer(Producer, 120.0);

    // ---- session 1: one rig, post-helper stream on ----
    Check(utpp::posecaptureruntime::Start(0) && utpp::posecaptureruntime::Start(0) && utpp::posecaptureruntime::Enabled(), "Start twice: recording, idempotent");
    Check(utpp::posecaptureruntime::Status().find("state=recording") != std::string::npos, "Status: recording");
    Check(!utpp::posecaptureruntime::Start(5) && utpp::posecaptureruntime::Enabled() && CountLogs("another window is running") == 1,
          "Start with another window while recording is refused (Stop first), the running capture is untouched");
    Check(!utpp::bodytraceruntime::Enable(true) && !utpp::bodytraceruntime::Enabled(), "the text capture is refused while the compact capture runs");
    Check(CountLogs("refused, the compact pose capture is running") == 1, "  ... with a log line");
    SleepMs(200);
    Check(fs::is_empty(logs), "recording without a rig: no file yet");
    utpp::posecaptureruntime::PublishMap(MakeMap(1), true, hashes, parents);
    SleepMs(3500);
    const std::string status = utpp::posecaptureruntime::Status();
    Check(status.find("file=red4ext\\logs\\utpp-pose-") != std::string::npos && !HasAbsolutePath(status), "Status names the file relative to the game folder: " + status.substr(0, 200));
    const std::string list1 = utpp::posecaptureruntime::Stop();
    const std::vector<std::string> files1 = Split(list1);
    Check(files1.size() == 1 && files1[0].rfind("red4ext\\logs\\utpp-pose-", 0) == 0 && files1[0].size() > 8 && files1[0].substr(files1[0].size() - 9) == ".utpppose" &&
              !HasAbsolutePath(list1),
          "Stop returns one relative path: " + list1);
    Check(!utpp::posecaptureruntime::Enabled() && utpp::posecaptureruntime::Status().find("state=stopped") != std::string::npos, "stopped");
    size_t ticksAfterSession1 = 0;
    {
        std::lock_guard lock(g_tickMutex);
        ticksAfterSession1 = g_capturedTicks.size();
    }
    if (files1.size() == 1)
    {
        const fs::path path = game / files1[0];
        Check(fs::exists(path), "the listed file exists under the game folder");
        const auto bytes = ReadAll(path);
        const pcf::DecodedFile d = pcf::DecodeFile(bytes.data(), bytes.size(), reader);
        Check(d.headerOk && !d.corrupt && !d.truncated && d.trailer.present && !d.trailer.failed, F("file decodes: header, %zu blocks, trailer (%zu bytes)", d.blocks.size(), bytes.size()));
        Check(d.header.algorithm == pcf::kAlgorithmXpress && d.header.rig == MakeMap(1).rig && d.header.rigCount == kRigCount && d.header.hasStream[1] && d.header.qpcHz > 0 &&
                  d.header.utc[0] >= 2026,
              "header: algorithm, rig, post-helper stream, clock, UTC start");
        Check(d.header.hashes == hashes && d.header.parents == parents && d.header.names.size() == kMaxBones && d.header.names[0] == "Spine3", "header: rig topology and bone names");
        std::vector<int64_t> got;
        for (const auto& b : d.blocks)
            if (b.stream == 0)
                for (uint32_t f = 0; f < b.frames; ++f)
                    got.push_back(b.meta[size_t(f) * 4]);
        std::vector<int64_t> want;
        {
            std::lock_guard lock(g_tickMutex);
            want.assign(g_capturedTicks.begin(), g_capturedTicks.begin() + static_cast<std::ptrdiff_t>(std::min(ticksAfterSession1, g_capturedTicks.size())));
        }
        // the producer kept running while Stop closed the file: the file holds a prefix of what was accepted, in order, exactly
        const bool prefix = got.size() <= want.size() && std::equal(got.begin(), got.end(), want.begin()) && got.size() + 3 >= want.size();
        Check(prefix && got.size() > 300, F("%zu frames in the file, %zu accepted by the ring: same ticks in the same order", got.size(), want.size()));
        Check(d.trailer.dropped[0] == 0 && d.FramesOf(1) > 300, F("nothing dropped at 120 Hz (trailer dropped %llu), %llu post-helper frames in the same file", static_cast<unsigned long long>(d.trailer.dropped[0]), static_cast<unsigned long long>(d.FramesOf(1))));
        double worst = 0.0;
        for (const auto& b : d.blocks)
            if (b.stream == 0 && b.frames > 0)
            {
                // every bone slot is mapped in this rig: column c = bone slot c / 24, stage (c / 12) % 2, value c % 12
                for (uint32_t c = 0; c < b.columns; c += 7)
                {
                    const uint32_t slot = c / 24, stage = (c / 12) % 2, v = c % 12;
                    const float expect = BoneValue(size_t(slot) * 12 + v) + (stage ? 0.5f : 0.0f);
                    worst = std::max(worst, static_cast<double>(std::fabs(b.values[c] - expect)));
                }
            }
        Check(worst < 1e-3, F("values (constant pose) within the quantization error (worst %.2e)", worst));
    }

    // ---- coexistence in the other order, then off / on again ----
    Check(utpp::bodytraceruntime::Enable(true), "the text capture starts once the compact capture has stopped");
    Check(!utpp::posecaptureruntime::Start(0), "the compact capture is refused while the text capture runs");
    Check(CountLogs("refused, the text capture of the helper bones is running") == 1, "  ... with a log line");
    Check(utpp::bodytraceruntime::Enable(false), "the text capture stops");

    // ---- session 2: rig change = second file ----
    g_rig = 1;
    Check(utpp::posecaptureruntime::Start(0), "second session starts");
    utpp::posecaptureruntime::PublishMap(MakeMap(1), false, hashes, parents);
    SleepMs(1800);
    g_rig = 2;
    utpp::posecaptureruntime::PublishMap(MakeMap(2), false, hashes, parents);
    SleepMs(1800);
    const std::string list2 = utpp::posecaptureruntime::Stop();
    const std::vector<std::string> files2 = Split(list2);
    Check(files2.size() == 2 && files2[0] != files2[1], "a rig change gives a second file: " + list2);
    if (files2.size() == 2)
    {
        const auto a = ReadAll(game / files2[0]), b = ReadAll(game / files2[1]);
        const pcf::DecodedFile da = pcf::DecodeFile(a.data(), a.size(), reader), db = pcf::DecodeFile(b.data(), b.size(), reader);
        Check(da.headerOk && db.headerOk && da.trailer.present && db.trailer.present && da.header.rig == MakeMap(1).rig && db.header.rig == MakeMap(2).rig &&
                  db.header.segmentIndex == da.header.segmentIndex + 1,
              "both files are complete, each with its own rig, indexed in order");
        Check(da.FramesOf(0) > 150 && db.FramesOf(0) > 150 && !da.header.hasStream[1], F("frames per file %llu / %llu, no post-helper stream when it was off", static_cast<unsigned long long>(da.FramesOf(0)), static_cast<unsigned long long>(db.FramesOf(0))));
    }

    // ---- session 3: no rig = no file ----
    const size_t before = std::distance(fs::directory_iterator(logs), fs::directory_iterator());
    Check(utpp::posecaptureruntime::Start(1), "session with a window starts");
    SleepMs(300);
    Check(utpp::posecaptureruntime::Stop().empty() && std::distance(fs::directory_iterator(logs), fs::directory_iterator()) == static_cast<std::ptrdiff_t>(before),
          "a session without a rig writes no file; Stop returns an empty string");

    g_stop = true;
    producer.join();
    utpp::posecaptureruntime::Shutdown();
    {
        std::lock_guard lock(g_logMutex);
        bool absolute = false;
        for (const auto& l : g_logs)
            absolute = absolute || HasAbsolutePath(l);
        Check(!absolute, "no log line carries an absolute path");
    }
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

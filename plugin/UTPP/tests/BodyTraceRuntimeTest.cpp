// End-to-end test of the production continuous capture (src/BodyTraceRuntime.cpp + the real g_capture / g_postHelperCapture rings):
// a producer thread plays the FK hook (Begin/End at ~120 Hz, 185 bones), the writer thread drains into real CSV files.
//   - files: utpp-bodytrace-<yyyy-mm-dd-hh-mm-ss>.csv + utpp-bodytrace-posthelper-<..>.csv, header + `# rig_bone` topology + columns + rows
//   - every captured frame is in the file (185 rows each), no frame lost to a re-arm gap, nothing dropped at this rate
//   - a rig change (new map) closes the pair (trailer with frames / dropped) and opens a new one with the new rig
//   - off / on again, idempotent calls, the 10 s statistics line (period shortened is not possible here: checked in the unit test), unload
// Usage: utpp_body_trace_runtime_test <scratch folder>   (created if missing; the files are left there)
#include <algorithm>
#include <atomic>
#include <cstring>
#include <memory>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../src/BodyTraceRuntime.hpp"

namespace fs = std::filesystem;
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
std::atomic<uint64_t> g_calls{0}, g_captured{0}, g_postCaptured{0};

Map MakeMap(uint64_t aRig)
{
    std::vector<std::string_view> names(kNames.begin(), kNames.end());
    Map m;
    Resolve(names, aRig, m);
    return m;
}
void Producer(double aHz)
{
    std::vector<uint8_t> bones(kRigCount * 48, 0);
    for (size_t i = 0; i < bones.size(); i += 4)
    {
        const float v = static_cast<float>(i % 977) * 0.01f - 3.0f;
        std::memcpy(bones.data() + i, &v, 4);
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
        const int64_t ticks = 1000000 + static_cast<int64_t>(n) * 10;
        ++g_calls;
        if (g_capture.Active() && g_capture.Begin(*frame, bones.data(), kRigCount, rig, 0xD0, ticks, 7) &&
            g_capture.End(*frame, bones.data(), kRigCount, rig, 0xD0))
            ++g_captured;
        if (g_postHelperCapture.Active() && g_postHelperCapture.Capture(bones.data(), kRigCount, rig, 0xD0, ticks, 7, 1, 1, ticks - 5, ticks - 1))
            ++g_postCaptured;
    }
}

struct FileInfo
{
    std::string name;
    uint64_t rows = 0, bytes = 0;
    int rigBones = 0, comments = 0;
    std::string trailer, columns;
    uint64_t firstTicks = 0, lastTicks = 0, rig = 0;
};
FileInfo Scan(const fs::path& aPath)
{
    FileInfo fi;
    fi.name = aPath.filename().string();
    fi.bytes = fs::file_size(aPath);
    std::ifstream in(aPath, std::ios::binary);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;
        if (line[0] == '#')
        {
            ++fi.comments;
            if (line.rfind("# rig_bone", 0) == 0)
                ++fi.rigBones;
            if (line.rfind("# frames=", 0) == 0)
                fi.trailer = line;
            continue;
        }
        if (line.rfind("ticks,", 0) == 0)
        {
            fi.columns = line;
            continue;
        }
        ++fi.rows;
        if (fi.rows == 1)
        {
            fi.firstTicks = std::strtoull(line.c_str(), nullptr, 10);
            size_t pos = 0;
            for (int c = 0; c < 3; ++c)
                pos = line.find(',', pos) + 1;
            fi.rig = std::strtoull(line.c_str() + pos, nullptr, 10);
        }
        fi.lastTicks = std::strtoull(line.c_str(), nullptr, 10);
    }
    return fi;
}
std::vector<fs::path> Files(const fs::path& aDir, const char* aPrefix)
{
    std::vector<fs::path> v;
    for (const auto& e : fs::directory_iterator(aDir))
        if (e.path().filename().string().rfind(aPrefix, 0) == 0)
            v.push_back(e.path());
    std::sort(v.begin(), v.end());
    return v;
}
void SleepMs(int aMs) { std::this_thread::sleep_for(std::chrono::milliseconds(aMs)); }
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_body_trace_runtime_test <scratch folder>\n");
        return 2;
    }
    const fs::path dir = fs::absolute(argv[1]);
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);

    std::vector<uint64_t> hashes(kRigCount);
    std::vector<int16_t> parents(kRigCount);
    for (uint32_t i = 0; i < kRigCount; ++i)
    {
        hashes[i] = 0xABCD0000ull + i;
        parents[i] = i == 0 ? -1 : static_cast<int16_t>(i - 1);
    }

    Check(!utpp::bodytraceruntime::Enable(true), "not initialised -> Enable(true) refused");
    utpp::bodytraceruntime::Init(&Log, dir.wstring());
    Check(!utpp::bodytraceruntime::Enabled(), "initialised but off");
    std::thread producer(Producer, 120.0);

    Check(utpp::bodytraceruntime::Enable(true) && utpp::bodytraceruntime::Enable(true) && utpp::bodytraceruntime::Enabled(), "Enable(true) twice: on, idempotent");
    SleepMs(200);
    Check(Files(dir, "utpp-bodytrace-").empty(), "enabled without a map: no file yet");
    const uint64_t calls0 = g_calls.load();
    utpp::bodytraceruntime::PublishMap(MakeMap(1), true, hashes, parents);
    SleepMs(3000);
    const uint64_t capturedRig1 = g_captured.load(), postRig1 = g_postCaptured.load(), offeredRig1 = g_calls.load() - calls0;
    utpp::bodytraceruntime::PublishMap(MakeMap(1), true, hashes, parents); // identical: ignored
    SleepMs(100);
    Check(Files(dir, "utpp-bodytrace-").size() == 2, "identical map published again: still one pair of files (legacy + post-helper)");

    // rig change while capturing
    g_rig = 2;
    utpp::bodytraceruntime::PublishMap(MakeMap(2), true, hashes, parents);
    SleepMs(2500);
    Check(utpp::bodytraceruntime::Enable(false) && !utpp::bodytraceruntime::Enabled(), "Enable(false): off, writer thread joined");
    const uint64_t callsEnd = g_calls.load(), capturedEnd = g_captured.load();
    Check(utpp::bodytraceruntime::Enable(false), "Enable(false) twice: idempotent");

    // second session (same second -> numeric suffix, never overwriting)
    g_rig = 3;
    Check(utpp::bodytraceruntime::Enable(true), "off -> on again");
    utpp::bodytraceruntime::PublishMap(MakeMap(3), false, hashes, parents);
    SleepMs(1500);
    utpp::bodytraceruntime::ClearMap();
    SleepMs(200);
    utpp::bodytraceruntime::Shutdown();
    g_stop = true;
    producer.join();

    const auto legacy = Files(dir, "utpp-bodytrace-2");
    const auto post = Files(dir, "utpp-bodytrace-posthelper-");
    Check(legacy.size() == 3 && post.size() == 2, F("files: %zu legacy (rig 1, rig 2, rig 3) + %zu post-helper (rig 1, rig 2)", legacy.size(), post.size()));
    std::printf("INFO files in %s\n", dir.string().c_str());
    uint64_t rowsTotal = 0;
    bool namesOk = true;
    for (const auto& p : legacy)
    {
        const std::string n = p.filename().string(); // utpp-bodytrace-2026-10-25-12-00-00[-2].csv
        namesOk = namesOk && n.size() >= 34 && n[19] == '-' && n.find(".csv") != std::string::npos;
        const FileInfo fi = Scan(p);
        rowsTotal += fi.rows;
        std::printf("INFO %s: rows %llu (%llu frames) bytes %llu rig_bone lines %d rig %llu trailer '%s'\n", fi.name.c_str(), static_cast<unsigned long long>(fi.rows),
                    static_cast<unsigned long long>(fi.rows / kMaxBones), static_cast<unsigned long long>(fi.bytes), fi.rigBones,
                    static_cast<unsigned long long>(fi.rig), fi.trailer.c_str());
        Check(fi.rows % kMaxBones == 0 && fi.rows > 0 && fi.rigBones == static_cast<int>(kRigCount) && !fi.trailer.empty() &&
                  fi.columns.find("s1_11") != std::string::npos && fi.trailer.find("dropped=0") != std::string::npos && fi.trailer.find("failed=0") != std::string::npos,
              F("%s: whole frames (185 rows each), topology, column header, trailer without drops / failure", fi.name.c_str()));
    }
    Check(namesOk, "file names follow utpp-bodytrace-<yyyy-mm-dd-hh-mm-ss>[-n].csv");
    if (legacy.size() == 3)
    {
        const FileInfo a = Scan(legacy[0]), b = Scan(legacy[1]), c = Scan(legacy[2]);
        Check(a.rig == 1 && b.rig == 2 && c.rig == 3, F("each file records one rig (%llu, %llu, %llu)", static_cast<unsigned long long>(a.rig),
                                                          static_cast<unsigned long long>(b.rig), static_cast<unsigned long long>(c.rig)));
        const uint64_t framesFile = (a.rows + b.rows + c.rows) / kMaxBones;
        // frames produced while a map was valid and the rig matched; producer counts only successful End()
        Check(framesFile == g_captured.load(), F("every captured frame is in a file: producer captured %llu, files hold %llu", static_cast<unsigned long long>(g_captured.load()),
                                                static_cast<unsigned long long>(framesFile)));
        const double ratio = static_cast<double>(capturedRig1) / static_cast<double>(std::max<uint64_t>(offeredRig1, 1));
        Check(ratio >= 0.97, F("rig 1 phase (3 s): %llu of %llu offered frames captured (%.1f%%) - no gap at the 200-frame budget", static_cast<unsigned long long>(capturedRig1), static_cast<unsigned long long>(offeredRig1), ratio * 100.0));
    }
    if (post.size() == 2)
    {
        const FileInfo p1 = Scan(post[0]), p2 = Scan(post[1]);
        Check(p1.rows % kMaxBones == 0 && p2.rows % kMaxBones == 0 && (p1.rows + p2.rows) / kMaxBones == g_postCaptured.load() &&
                  p1.columns.find("local_11") != std::string::npos && p1.trailer.find("dropped=0") != std::string::npos,
              F("post-helper files hold every captured post frame (%llu)", static_cast<unsigned long long>(g_postCaptured.load())));
    }
    (void)postRig1; (void)callsEnd; (void)capturedEnd;
    Check(!g_capture.Active() && !g_postHelperCapture.Active() && !g_capture.Live(), "after unload: captures cancelled");
    Check(CountLogs("started") == 3 && CountLogs("stopped") == 3, F("log: 3 starts / 3 stops (%d / %d)", CountLogs("started"), CountLogs("stopped")));
    std::printf("body trace runtime: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

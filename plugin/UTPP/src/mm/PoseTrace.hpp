#pragma once
// Bounded three-stage FK-input capture. Animation callbacks only copy into a fixed
// ring; a separate writer owns all formatting and file IO. Off by default: the diagnostic logs use the compact pose recording; this text
// CSV starts only from motionhost::SetFeedDump(true) when the opt-in marker file is present (see MotionHost.cpp).
#include "../CommonUtil.hpp"
#include "GameIntegration.hpp"
#include <Windows.h>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <thread>

namespace utpp::mm::posetrace
{
inline constexpr uint64_t kMaxFrames = 108000; // 30 minutes at 60 Hz; ring[64] RAM stays unchanged
inline constexpr size_t kWriteBufferBytes = 64 * 1024;

struct Frame
{
    int64_t ticks = 0;
    uint64_t desc = 0;
    uint32_t thread = 0, count = 0;
    integ::PoseSnapshot snap;
    float stage[3][integ::kMaxMMBones][12]; // complete 48-byte local records, including scale
};
inline bool Capture(Frame& f, int stage, const uint8_t* bones, uint32_t count)
{
    if (stage < 0 || stage >= 3 || !bones || !f.snap.numBones || f.snap.numBones > integ::kMaxMMBones) return false;
    uint32_t mapped = 0;
    for (uint32_t b = 0; b < f.snap.numBones; ++b)
    {
        const int32_t g = f.snap.gameIndex[b];
        if (g < 0 || uint32_t(g) >= count)
        {
            for (float& value : f.stage[stage][b]) value = std::numeric_limits<float>::quiet_NaN();
            continue;
        }
        std::memcpy(f.stage[stage][b], bones + size_t(g) * integ::kGameBoneStride, 48);
        ++mapped;
    }
    return mapped != 0;
}
class Dumper
{
public:
    ~Dumper() { Stop(); }
    bool Enabled() const { return on.load(std::memory_order_relaxed); }
    const std::filesystem::path& Path() const { return path; }
    void Push(const Frame& f)
    {
        if (!Enabled()) return;
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1); return; }
        if (!Enabled()) { busy.clear(std::memory_order_release); return; }
        const auto w = write.load(std::memory_order_relaxed);
        if (w - read.load(std::memory_order_acquire) >= 64) dropped.fetch_add(1);
        else { ring[w % 64] = f; write.store(w + 1, std::memory_order_release); }
        busy.clear(std::memory_order_release);
    }
    void Stop()
    {
        on.store(false, std::memory_order_release);
        // A callback which observed on=true may still hold the producer flag.
        while (busy.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        busy.clear(std::memory_order_release);
        finish.store(true, std::memory_order_release);
        if (worker.joinable())
        {
            worker.join();
            // no frame reached the trace (the skeleton hook did not feed it): drop the header-only file. A failed writer keeps
            // its file so that Status() (failed=1) and the file itself still show the cause.
            std::error_code ec;
            if (read.load() == 0 && !failed.load() && !path.empty()) std::filesystem::remove(path, ec);
        }
    }
    bool Start(const std::wstring& folder)
    {
        Stop();
        write.store(0); read.store(0); dropped.store(0); failed.store(false);
        LARGE_INTEGER q{}, hz{}; QueryPerformanceCounter(&q); QueryPerformanceFrequency(&hz);
        path = std::filesystem::path(folder) / (L"utpp-pose-" + std::to_wstring(q.QuadPart) + L".csv");
        FILE* file = nullptr;
        if (_wfopen_s(&file, path.c_str(), L"wx") || !file) return false;
        std::fprintf(file, "# qpc_hz=%lld max_frames=%llu stages=game,blend,aim local_records=pos4,quat4,scale4\n",
                     hz.QuadPart, static_cast<unsigned long long>(kMaxFrames));
        std::fprintf(file, "ticks,desc,thread,rig,seq,publish,action,overlay,aiming,clip,weight,upper,bone,gameIndex,masked");
        for (int s = 0; s < 3; ++s) for (int k = 0; k < 12; ++k) std::fprintf(file, ",s%d_%d", s, k);
        for (int k = 0; k < 7; ++k) std::fprintf(file, ",mm_%d", k);
        std::fprintf(file, "\n");
        if (std::fflush(file)) { std::fclose(file); return false; }
        finish.store(false); on.store(true, std::memory_order_release);
        try { worker = std::thread([this, file]() { Run(file); }); }
        catch (...) { on.store(false); std::fclose(file); return false; }
        return true;
    }
    std::string Status() const
    {
        return "poseTrace on=" + std::to_string(Enabled()) + " frames=" + std::to_string(read.load()) +
            " dropped=" + std::to_string(dropped.load()) + " failed=" + std::to_string(failed.load()) +
            " file=" + util::RelativeToGame(path.wstring()); // game-relative: no user name in the status text
    }
private:
    void Run(FILE* file)
    {
        char output[kWriteBufferBytes];
        char row[2048];
        size_t outputUsed = 0;
        uint64_t written = 0;
        bool ioOk = true;
        auto lastFlush = std::chrono::steady_clock::now();
        const auto flushBuffered = [&]() {
            if (outputUsed)
            {
                const size_t requested = outputUsed;
                if (std::fwrite(output, 1, requested, file) != requested) ioOk = false;
                outputUsed = 0;
            }
            if (ioOk && std::fflush(file)) ioOk = false;
            if (!ioOk) { failed.store(true); on.store(false); }
            lastFlush = std::chrono::steady_clock::now();
        };
        for (;;)
        {
            auto r = read.load(std::memory_order_relaxed);
            if (r == write.load(std::memory_order_acquire))
            {
                if (finish.load(std::memory_order_acquire)) break;
                if (outputUsed && std::chrono::steady_clock::now() - lastFlush >= std::chrono::milliseconds(250))
                    flushBuffered();
                std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue;
            }
            const Frame& f = ring[r % 64];
            if (written < kMaxFrames && ioOk)
            {
                for (uint32_t b = 0; b < f.snap.numBones; ++b)
                {
                    char* p = row;
                    char* const end = row + sizeof(row);
                    bool first = true, rowOk = true;
                    const auto addInt = [&](auto value) {
                        if (!rowOk) return;
                        if (!first) *p++ = ',';
                        const auto result = std::to_chars(p, end, value);
                        if (result.ec != std::errc{}) rowOk = false;
                        else { p = result.ptr; first = false; }
                    };
                    const auto addFloat = [&](double value) {
                        if (!rowOk) return;
                        if (!first) *p++ = ',';
                        const auto result = std::to_chars(p, end, value, std::chars_format::general, 9);
                        if (result.ec != std::errc{}) rowOk = false;
                        else { p = result.ptr; first = false; }
                    };
                    addInt(f.ticks); addInt(f.desc); addInt(f.thread); addInt(f.snap.rig); addInt(f.snap.sequence);
                    addInt(f.snap.publishTicks); addInt(f.snap.traceAction); addInt(f.snap.traceOverlay);
                    addInt(f.snap.traceAiming); addInt(f.snap.traceClip); addFloat(f.snap.weight); addFloat(f.snap.upperScale);
                    addInt(b); addInt(f.snap.gameIndex[b]); addInt(integ::SnapshotUpperBone(f.snap, b) ? 1 : 0);
                    for (int s = 0; s < 3; ++s)
                        for (int k = 0; k < 12; ++k) addFloat(f.stage[s][b][k]);
                    for (int k = 0; k < 7; ++k) addFloat(f.snap.pose[b * 7 + k]);
                    if (!rowOk || p >= end) { ioOk = false; break; }
                    *p++ = '\n';
                    const size_t rowBytes = static_cast<size_t>(p - row);
                    if (rowBytes > sizeof(output) - outputUsed)
                    {
                        flushBuffered();
                        if (!ioOk) break;
                    }
                    std::memcpy(output + outputUsed, row, rowBytes);
                    outputUsed += rowBytes;
                }
                if (!ioOk) { failed.store(true); on.store(false); }
                ++written;
                if (written == kMaxFrames) on.store(false, std::memory_order_release);
            }
            read.store(r + 1, std::memory_order_release);
            if (std::ferror(file)) { ioOk = false; failed.store(true); on.store(false); }
        }
        flushBuffered();
        if (ioOk)
            std::fprintf(file, "# frames=%llu dropped=%llu failed=%d\n", written, dropped.load(), int(failed.load()));
        if (std::fclose(file)) failed.store(true);
    }
    Frame ring[64]{};
    std::atomic<uint64_t> write{0}, read{0}, dropped{0};
    std::atomic<bool> on{false}, finish{true}, failed{false};
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    std::thread worker;
    std::filesystem::path path;
};
inline Dumper g_dump;
}

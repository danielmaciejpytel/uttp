#pragma once
// Ring and file of the reload diagnostic record (ReloadTrace.hpp). The pose hook copies one record into a fixed ring (a try-flag: a busy or
// full ring drops the record and counts it, the hook never waits); a writer thread stores the records unchanged in
// utpp-reload-<local date-time>.bin next to the other diagnostic files. Off (the default) the hook pays one relaxed load.
// File: a 64-byte header, then whole records (kRecordBytes each) in push order. The writer flushes at least every 250 ms, so a killed game
// loses at most the last quarter second; the reader accepts a truncated last record.
#include "../CommonUtil.hpp"
#include "ReloadTrace.hpp"
#include <Windows.h>
#include <share.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>

namespace utpp::mm::reload_trace
{
inline constexpr char kMagic[8] = {'U', 'T', 'P', 'P', 'R', 'L', 'D', 'T'};
inline constexpr uint64_t kMaxRecords = 216000; // one hour of reload frames at 60 Hz (~93 MB)
inline constexpr uint32_t kRingSize = 256;      // ~4 s of frames at 60 Hz (~110 KB)
inline constexpr int kTailFrames = 30;          // frames recorded after the last reload-related frame (the hand-back exit)

struct Header
{
    char magic[8];
    uint32_t format;
    uint32_t recordBytes;
    int64_t qpcHz;
    int64_t qpcAtOpen;
    uint16_t localTime[8]; // year, month, day, hour, minute, second, millisecond, 0 (local time at qpcAtOpen)
    uint8_t reserved[16];
};
static_assert(sizeof(Header) == 64, "header layout is mirrored by the reader");

class Ring
{
public:
    ~Ring() { Stop(); }
    bool Enabled() const { return on.load(std::memory_order_relaxed); }
    // Pose hook: one record, never blocks.
    // Every attempt gets a push number (Record::pushNumber, 24 bits, little endian), dropped ones included: a gap in the file = drops.
    void Push(const Record& r)
    {
        if (!Enabled()) return;
        const uint32_t number = pushes.fetch_add(1, std::memory_order_relaxed);
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1, std::memory_order_relaxed); return; }
        if (Enabled())
        {
            const uint64_t w = write.load(std::memory_order_relaxed);
            if (w - read.load(std::memory_order_acquire) >= kRingSize) dropped.fetch_add(1, std::memory_order_relaxed);
            else
            {
                Record& slot = ring[w % kRingSize];
                slot = r;
                slot.pushNumber[0] = uint8_t(number); slot.pushNumber[1] = uint8_t(number >> 8); slot.pushNumber[2] = uint8_t(number >> 16);
                write.store(w + 1, std::memory_order_release);
            }
        }
        busy.clear(std::memory_order_release);
    }
    // Reload-related frame (true) or not: returns whether this hook call records (a reload frame or inside the tail after one).
    bool Wanted(bool reloadFrame)
    {
        if (!Enabled()) return false;
        if (reloadFrame) { tail.store(kTailFrames, std::memory_order_relaxed); return true; }
        int t = tail.load(std::memory_order_relaxed);
        while (t > 0 && !tail.compare_exchange_weak(t, t - 1, std::memory_order_relaxed)) {}
        return t > 0;
    }
    // Control path (game thread). folder: the diagnostic log folder.
    bool Start(const std::wstring& folder)
    {
        Stop();
        write.store(0); read.store(0); dropped.store(0); stored.store(0); failed.store(false); tail.store(0); pushes.store(0);
        Header h{};
        std::memcpy(h.magic, kMagic, sizeof(kMagic));
        h.format = kFormat; h.recordBytes = kRecordBytes;
        LARGE_INTEGER q{}, hz{}; QueryPerformanceCounter(&q); QueryPerformanceFrequency(&hz);
        SYSTEMTIME lt{}; GetLocalTime(&lt);
        h.qpcHz = hz.QuadPart; h.qpcAtOpen = q.QuadPart;
        const uint16_t t[8] = {lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond, lt.wMilliseconds, 0};
        std::memcpy(h.localTime, t, sizeof(t));
        // never overwrite an earlier record: a second start within the same second gets -2, -3, ... (like the feed dump)
        FILE* file = nullptr;
        for (int k = 1; k <= 9 && !file; ++k)
        {
            wchar_t name[64];
            swprintf_s(name, L"utpp-reload-%04u-%02u-%02u-%02u-%02u-%02u%s.bin", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute,
                       lt.wSecond, k == 1 ? L"" : (std::wstring(L"-") + std::to_wstring(k)).c_str());
            path = std::filesystem::path(folder) / name;
            std::error_code ec;
            if (std::filesystem::exists(path, ec) || ec) continue;
            // shared for reading: the harness copies the file while the game writes it
            file = _wfsopen(path.c_str(), L"wb", _SH_DENYWR);
        }
        if (!file) return false;
        if (std::fwrite(&h, sizeof(h), 1, file) != 1 || std::fflush(file)) { std::fclose(file); return false; }
        finish.store(false); on.store(true, std::memory_order_release);
        try { writer = std::thread([this, file]() { Run(file); }); }
        catch (...) { on.store(false); std::fclose(file); return false; }
        return true;
    }
    void Stop()
    {
        on.store(false, std::memory_order_release);
        while (busy.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        busy.clear(std::memory_order_release);
        finish.store(true, std::memory_order_release);
        if (writer.joinable()) writer.join();
    }
    std::string Status() const
    {
        return "reloadRecord on=" + std::to_string(Enabled()) + " records=" + std::to_string(stored.load()) +
               " dropped=" + std::to_string(dropped.load()) + " failed=" + std::to_string(failed.load()) +
               " file=" + util::RelativeToGame(path.wstring()); // game-relative: no user name in the status text
    }
    const std::filesystem::path& Path() const { return path; }
    uint64_t Stored() const { return stored.load(); }
    uint64_t Dropped() const { return dropped.load(); }

private:
    void Run(FILE* file)
    {
        bool ok = true;
        auto lastFlush = std::chrono::steady_clock::now();
        bool pending = false;
        for (;;)
        {
            const uint64_t r = read.load(std::memory_order_relaxed);
            if (r == write.load(std::memory_order_acquire))
            {
                if (finish.load(std::memory_order_acquire)) break;
                if (pending && std::chrono::steady_clock::now() - lastFlush >= std::chrono::milliseconds(250))
                {
                    if (std::fflush(file)) ok = false;
                    pending = false; lastFlush = std::chrono::steady_clock::now();
                }
                if (!ok) { failed.store(true); on.store(false); }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            if (ok && stored.load(std::memory_order_relaxed) < kMaxRecords)
            {
                if (std::fwrite(&ring[r % kRingSize], sizeof(Record), 1, file) != 1) ok = false;
                else { stored.fetch_add(1, std::memory_order_relaxed); pending = true; }
                if (stored.load(std::memory_order_relaxed) >= kMaxRecords) on.store(false, std::memory_order_release);
            }
            read.store(r + 1, std::memory_order_release);
            if (!ok) { failed.store(true); on.store(false); }
        }
        if (std::fflush(file) || std::fclose(file)) failed.store(true);
    }
    Record ring[kRingSize]{};
    std::atomic<uint64_t> write{0}, read{0}, dropped{0}, stored{0};
    std::atomic<int> tail{0};
    std::atomic<uint32_t> pushes{0};
    std::atomic<bool> on{false}, finish{true}, failed{false};
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    std::thread writer;
    std::filesystem::path path;
};
inline Ring g_ring;
} // namespace utpp::mm::reload_trace

// Reload diagnostic record (src/mm/ReloadTrace.hpp, ReloadTraceRing.hpp): record layout, ring and file encoding.
// The layout numbers below are also checked by the reader's test (tools/gametest/test_reload_trace.py).
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "../src/mm/ReloadTraceRing.hpp"
#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <vector>
using namespace utpp::mm;
static int g_pass = 0, g_fail = 0;
static void Check(bool ok, const char* what)
{
    (ok ? g_pass : g_fail)++;
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
}
int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    using reload_trace::Record;
    // 1) layout (mirrored by the reader)
    Check(sizeof(Record) == 432 && sizeof(reload_trace::Header) == 64, "record 432 bytes, header 64 bytes");
    Check(offsetof(Record, ticks) == 0 && offsetof(Record, sequence) == 8 && offsetof(Record, flags) == 16 && offsetof(Record, mask) == 20 &&
              offsetof(Record, path) == 24 && offsetof(Record, action) == 28 && offsetof(Record, snapWeight) == 32 &&
              offsetof(Record, corrIn) == 48 && offsetof(Record, holdAmp) == 76 && offsetof(Record, stockShift) == 92 &&
              offsetof(Record, clavGame) == 100 && offsetof(Record, clavOut) == 172 && offsetof(Record, jointReqDeg) == 196 &&
              offsetof(Record, jointMaxDeg) == 220 && offsetof(Record, jointLimited) == 224 && offsetof(Record, shoulder) == 228 &&
              offsetof(Record, guide) == 300 && offsetof(Record, elbowSolve) == 312 && offsetof(Record, handOutRot) == 348 &&
              offsetof(Record, reachClavDeg) == 364 && offsetof(Record, anat) == 372,
          "field offsets as in the reader");
    // 2) Clear: every float NaN, integers 0
    {
        Record r;
        r.flags = 7; r.jointLimited = 5; r.snapWeight = 1.0f;
        reload_trace::Clear(r);
        bool nan = std::isnan(r.snapWeight) && std::isnan(r.anat[4][2]) && std::isnan(r.jointMaxDeg) && std::isnan(r.clavOut[1][2]);
        Check(nan && r.flags == 0 && r.jointLimited == 0 && r.ticks == 0 && r.path == reload_trace::kPathNone,
              "Clear: floats NaN (not run), flags / limited bits 0");
    }
    // 3) ring off: nothing wanted, nothing stored
    auto ring = std::make_unique<reload_trace::Ring>();
    {
        Record r; reload_trace::Clear(r);
        ring->Push(r);
        Check(!ring->Enabled() && !ring->Wanted(true) && ring->Stored() == 0 && ring->Dropped() == 0, "off: no record is wanted or stored");
    }
    // 4) tail: a reload frame, then kTailFrames more frames, then nothing
    std::filesystem::create_directories(argv[1]);
    for (const auto& e : std::filesystem::directory_iterator(argv[1]))
        if (e.path().filename().wstring().rfind(L"utpp-reload-", 0) == 0) std::filesystem::remove(e.path());
    const std::wstring folder = std::filesystem::path(argv[1]).wstring();
    if (!ring->Start(folder)) { std::printf("FAIL start\n"); return 1; }
    {
        int wanted = 0;
        ring->Wanted(true);
        for (int k = 0; k < 40; ++k) wanted += ring->Wanted(false) ? 1 : 0;
        Check(wanted == reload_trace::kTailFrames, "tail: the frames after the last reload frame are recorded for the tail only");
    }
    // 5) encoding: records come out byte for byte in push order (a full ring drops and counts, never blocks)
    std::vector<Record> pushed;
    for (int k = 0; k < 600; ++k)
    {
        Record r; reload_trace::Clear(r);
        r.ticks = 1000 + k; r.sequence = uint64_t(k); r.path = uint8_t(k % 4); r.exit = uint8_t(k % 11);
        r.corrIn = float(k) * 0.01f; r.clavOut[1][2] = -float(k); r.anat[4][2] = 75.0f + float(k);
        r.flags = uint32_t(k) * 3u; r.jointLimited = uint32_t(k & 63);
        ring->Push(r);
        pushed.push_back(r);
        if (k % 50 == 49) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto path = ring->Path();
    ring->Stop();
    const uint64_t stored = ring->Stored(), dropped = ring->Dropped();
    std::ifstream in(path, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    reload_trace::Header h{};
    bool headerOk = bytes.size() >= sizeof(h);
    if (headerOk) std::memcpy(&h, bytes.data(), sizeof(h));
    headerOk = headerOk && std::memcmp(h.magic, reload_trace::kMagic, 8) == 0 && h.format == reload_trace::kFormat &&
               h.recordBytes == sizeof(Record) && h.qpcHz > 0 && h.qpcAtOpen > 0 && h.localTime[0] >= 2020;
    Check(headerOk, "header: magic, format, record size, clock and local time");
    const size_t n = headerOk ? (bytes.size() - sizeof(h)) / sizeof(Record) : 0;
    bool exact = headerOk && (bytes.size() - sizeof(h)) % sizeof(Record) == 0 && n == stored && stored + dropped == pushed.size();
    size_t next = 0;
    bool numbered = true;
    for (size_t i = 0; exact && i < n; ++i)
    {
        Record r;
        std::memcpy(&r, bytes.data() + sizeof(h) + i * sizeof(Record), sizeof(r));
        while (next < pushed.size() && pushed[next].ticks != r.ticks) ++next; // dropped records are skipped, the order is kept
        // the push number is the index of the attempt (drops included); the rest of the record is as pushed
        const uint32_t number = uint32_t(r.pushNumber[0]) | uint32_t(r.pushNumber[1]) << 8 | uint32_t(r.pushNumber[2]) << 16;
        numbered = numbered && number == uint32_t(next);
        std::memset(r.pushNumber, 0, sizeof(r.pushNumber));
        exact = next < pushed.size() && std::memcmp(&r, &pushed[next], sizeof(r)) == 0;
        ++next;
    }
    Check(numbered, "push number: every stored record carries the index of its push attempt (a gap = dropped records)");
    std::printf("  pushed %zu, stored %llu, dropped %llu, file %zu bytes\n", pushed.size(), static_cast<unsigned long long>(stored),
                static_cast<unsigned long long>(dropped), bytes.size());
    Check(exact && stored >= 256, "encoding: the stored records equal the pushed ones byte for byte, in order; stored + dropped = pushed");
    // 6) never overwrites: a restart (also within the same second) opens a new file, the earlier record is kept
    {
        const bool again = ring->Start(folder);
        Check(again && ring->Path() != path && std::filesystem::file_size(path) == bytes.size(),
              "restart: a new file (suffix -2 within the same second), the earlier record is kept");
        ring->Stop();
        Record r; reload_trace::Clear(r);
        ring->Push(r);
        Check(!ring->Enabled(), "stopped: off again");
    }
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

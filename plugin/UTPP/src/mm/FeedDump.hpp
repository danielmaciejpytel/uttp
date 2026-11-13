#pragma once
// batch 1b (2026-10-13) motion matching feed dump (UTPPSetMMFeedDump / CET setting mm.feedDump, default off).
// One line per MMDriver Feed() call. The column order is a contract with the motion port (its .feed reader takes the first 17
// columns, as written by tools/replay/extract_feeds.py on branch port; the others are appended) - see kHeader:
//   t            real time since the dump was enabled (s)
//   dt           frame dt of the feed
//   vx vy vz     capsule world velocity (m/s, z up)
//   mvx mvy      raw stick (no dead zone, before the 0.5.3 stick filter)
//   yawDeg camDeg  player yaw / camera yaw (deg)
//   loco         locomotion state as fed (before the debounce / hold conditioning)
//   psm          raw UTPPPsmBlockMask (incl. the UpperBody / LocomotionDetailed / MeleeWeapon diagnostic fields), decimal
//   trans        1 on the feed whose matcher update made a v1 transition
//   baseClip baseFlags   clip played after the update (spaces / control characters -> '_') and its frame
//   baseCost baseMicros best cost and wall time (us) of the last v1 search (as MMTRACE cost= / sUs=)
//   step         game-test step name: unknown to the plugin, always '-'
//   haveTarget targetYawDeg   Locomotion world target (as fed)
//   capX capY capZ            capsule (player entity) world position (UTPPFeedLocomotionExtra)
//   effGate holdEligible gateRaw   gate the MM weight follows (after the upper-body mask and the hold), hold-last-pose
//                eligibility, gate before the debounce (decimal bit masks)
//   upperBody detailed        PSM.UpperBody / PSM.LocomotionDetailed decoded from psm
//   onGround distGround psmFall   air state: '-' until the air feed exists (air hold stage)
//   walk         PSM permanent bool WalkToggled (UTPPFeedLocomotionExtra), 0/1
// '-' = unknown (also any non-finite float). Feed floats are printed with 9 significant digits (exact float round trip).
//
// Threads: Push (game thread; MMDriver calls it under its mutex, so there is one producer at a time) copies one fixed-size
// Record into a preallocated single-producer / single-consumer ring - no allocation, no lock, no file I/O. A full ring drops the
// record and counts it (Push never waits for the writer). The writer thread (started by Start) creates
// <dir>\utpp-feed-<yyyy-mm-dd-hh-mm-ss>.txt (local time; "-2" .. "-99" appended when the name exists), writes the header line, and
// then about every second (earlier when the ring is half full) and on Stop formats and writes every pending record. Stop does not
// wait (the writer drains, closes the file and exits); Shutdown and the next Start join it. The destructor (DLL detach without an
// unload, process exit) never joins a live writer under the loader lock: a writer already terminated by the process exit has its
// remaining records written from the destructor, a live one gets up to 2 s to finish and is then detached.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <thread>

namespace utpp::mm::feeddump
{
inline constexpr char kHeader[] = "# cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step haveTarget "
                                  "targetYawDeg capX capY capZ effGate holdEligible gateRaw upperBody detailed onGround distGround "
                                  "psmFall walk";
inline constexpr int kColumns = 31;
inline constexpr size_t kClipChars = 95;     // clip name bytes kept (a longer name is cut)
inline constexpr size_t kMaxLine = 1024;     // FormatLine always fits a record into this many bytes
inline constexpr uint32_t kDefaultRing = 4096; // records (~27 s at 150 fps without a drain)

struct Record
{
    double t = 0.0;
    float dt = 0.0f;
    float vel[3] = {0.0f, 0.0f, 0.0f};
    float move[2] = {0.0f, 0.0f};
    float yawDeg = 0.0f;
    float camDeg = 0.0f;
    int32_t loco = 0;
    uint32_t psm = 0;
    bool v1 = false;          // the matcher ran on this feed (false: trans baseClip baseFlags baseCost baseMicros = '-')
    bool trans = false;
    int32_t baseFlags = -1;         // < 0: '-'
    bool baseSearched = false;  // the matcher has searched since its init (false: baseCost baseMicros = '-')
    float baseCost = 0.0f;
    double baseMicros = 0.0;
    char clip[kClipChars + 1] = {}; // "" = '-'
    bool haveTarget = false;
    float targetYawDeg = 0.0f;
    bool haveCapsule = false; // false: capX capY capZ = '-'
    float capsule[3] = {0.0f, 0.0f, 0.0f};
    uint32_t effGate = 0;
    bool holdEligible = false;
    uint32_t gateRaw = 0;
    int32_t upperBody = -1;   // < 0: '-'
    int32_t detailed = -1;    // < 0: '-'
    int8_t onGround = -1;     // < 0: '-'
    float distGround = std::numeric_limits<float>::quiet_NaN(); // non-finite: '-'
    int32_t psmFall = -1;     // < 0: '-'
    int8_t walk = -1;         // < 0: '-'
};

inline void SetClip(Record& aRec, std::string_view aName)
{
    const size_t n = std::min(aName.size(), kClipChars);
    if (n > 0)
        std::memcpy(aRec.clip, aName.data(), n);
    aRec.clip[n] = 0;
}

namespace detail
{
class LineOut
{
public:
    LineOut(char* aBuf, size_t aSize) : m_buf(aBuf), m_size(aSize) {}

    void Text(std::string_view aText)
    {
        Sep();
        Put(aText.data(), aText.size());
    }
    void Dash() { Text("-"); }
    void Flag(bool aOn) { Text(aOn ? "1" : "0"); }
    void Int(long long aValue)
    {
        char b[32];
        Formatted(b, std::snprintf(b, sizeof b, "%lld", aValue));
    }
    void UInt(unsigned long long aValue)
    {
        char b[32];
        Formatted(b, std::snprintf(b, sizeof b, "%llu", aValue));
    }
    void Real(double aValue, const char* aFormat)
    {
        if (!std::isfinite(aValue))
        {
            Dash();
            return;
        }
        char b[64];
        int n = std::snprintf(b, sizeof b, aFormat, aValue);
        if (n <= 0 || n >= static_cast<int>(sizeof b))
            n = std::snprintf(b, sizeof b, "%.9g", aValue); // a huge value in a fixed-point format: still one short token
        Formatted(b, n);
    }
    // clip names: empty -> '-', spaces / control / non-ASCII bytes -> '_' (one whitespace-free token)
    void Clip(const char* aName)
    {
        if (!aName || !aName[0])
        {
            Dash();
            return;
        }
        Sep();
        for (size_t i = 0; i < kClipChars && aName[i] != 0; ++i)
        {
            const unsigned char c = static_cast<unsigned char>(aName[i]);
            const char out = (c <= 0x20 || c >= 0x7F) ? '_' : static_cast<char>(c);
            Put(&out, 1);
        }
    }
    size_t Finish()
    {
        Put("\n", 1);
        if (!m_ok || m_len >= m_size)
            return 0;
        m_buf[m_len] = 0;
        return m_len;
    }

private:
    void Sep()
    {
        if (m_cols++ > 0)
            Put(" ", 1);
    }
    void Formatted(const char* aText, int aLen)
    {
        Sep();
        if (aLen <= 0 || aLen >= 64)
        {
            m_ok = false;
            return;
        }
        Put(aText, static_cast<size_t>(aLen));
    }
    void Put(const char* aData, size_t aLen)
    {
        if (!m_ok || m_len + aLen >= m_size)
        {
            m_ok = false;
            return;
        }
        std::memcpy(m_buf + m_len, aData, aLen);
        m_len += aLen;
    }

    char* m_buf;
    size_t m_size;
    size_t m_len = 0;
    int m_cols = 0;
    bool m_ok = true;
};

inline std::string Narrow(const std::wstring& aWide)
{
    if (aWide.empty())
        return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, aWide.c_str(), static_cast<int>(aWide.size()), nullptr, 0, nullptr, nullptr);
    std::string out(n > 0 ? static_cast<size_t>(n) : 0u, '\0');
    if (n > 0)
        WideCharToMultiByte(CP_UTF8, 0, aWide.c_str(), static_cast<int>(aWide.size()), out.data(), n, nullptr, nullptr);
    return out;
}
} // namespace detail

// One dump line incl. '\n' (NUL-terminated) into aBuf; returns its length without the NUL, 0 when it did not fit (kMaxLine always
// fits). Exactly kColumns space-separated tokens.
inline size_t FormatLine(const Record& aRec, char* aBuf, size_t aSize)
{
    constexpr const char* kF = "%.9g"; // float round trip
    detail::LineOut o(aBuf, aSize);
    o.Real(aRec.t, "%.6f");
    o.Real(aRec.dt, kF);
    o.Real(aRec.vel[0], kF);
    o.Real(aRec.vel[1], kF);
    o.Real(aRec.vel[2], kF);
    o.Real(aRec.move[0], kF);
    o.Real(aRec.move[1], kF);
    o.Real(aRec.yawDeg, kF);
    o.Real(aRec.camDeg, kF);
    o.Int(aRec.loco);
    o.UInt(aRec.psm);
    if (aRec.v1)
    {
        o.Flag(aRec.trans);
        o.Clip(aRec.clip);
    }
    else
    {
        o.Dash();
        o.Dash();
    }
    if (aRec.v1 && aRec.baseFlags >= 0)
        o.Int(aRec.baseFlags);
    else
        o.Dash();
    if (aRec.v1 && aRec.baseSearched)
    {
        o.Real(aRec.baseCost, "%.4f");
        o.Real(aRec.baseMicros, "%.1f");
    }
    else
    {
        o.Dash();
        o.Dash();
    }
    o.Dash(); // step
    o.Flag(aRec.haveTarget);
    o.Real(aRec.targetYawDeg, kF);
    for (int i = 0; i < 3; ++i)
    {
        if (aRec.haveCapsule)
            o.Real(aRec.capsule[i], kF);
        else
            o.Dash();
    }
    o.UInt(aRec.effGate);
    o.Flag(aRec.holdEligible);
    o.UInt(aRec.gateRaw);
    if (aRec.upperBody >= 0)
        o.Int(aRec.upperBody);
    else
        o.Dash();
    if (aRec.detailed >= 0)
        o.Int(aRec.detailed);
    else
        o.Dash();
    if (aRec.onGround >= 0)
        o.Flag(aRec.onGround != 0);
    else
        o.Dash();
    o.Real(aRec.distGround, kF);
    if (aRec.psmFall >= 0)
        o.Int(aRec.psmFall);
    else
        o.Dash();
    if (aRec.walk >= 0)
        o.Flag(aRec.walk != 0);
    else
        o.Dash();
    return o.Finish();
}

// "utpp-feed-<yyyy-mm-dd-hh-mm-ss>.txt"; aIndex >= 2 appends "-<aIndex>" (name already taken).
inline std::wstring FileName(const SYSTEMTIME& aTime, int aIndex = 1)
{
    wchar_t b[96];
    if (aIndex >= 2)
        swprintf_s(b, L"utpp-feed-%04u-%02u-%02u-%02u-%02u-%02u-%d.txt", unsigned(aTime.wYear), unsigned(aTime.wMonth), unsigned(aTime.wDay),
                   unsigned(aTime.wHour), unsigned(aTime.wMinute), unsigned(aTime.wSecond), aIndex);
    else
        swprintf_s(b, L"utpp-feed-%04u-%02u-%02u-%02u-%02u-%02u.txt", unsigned(aTime.wYear), unsigned(aTime.wMonth), unsigned(aTime.wDay),
                   unsigned(aTime.wHour), unsigned(aTime.wMinute), unsigned(aTime.wSecond));
    return b;
}

enum class PushResult
{
    Off,       // not accepting (stopped, never started, or the writer failed) - not counted
    Ok,
    Dropped,   // ring full: dropped and counted
    FirstDrop, // the first drop of this session (the caller logs one line)
};

class Dumper
{
public:
    explicit Dumper(uint32_t aRing = kDefaultRing)
    {
        uint32_t cap = 2;
        while (cap < aRing && cap < (1u << 24))
            cap <<= 1;
        m_cap = cap;
    }
    Dumper(const Dumper&) = delete;
    Dumper& operator=(const Dumper&) = delete;

    ~Dumper()
    {
        if (!m_thread.joinable())
            return;
        // no mutex here: a writer terminated by the process exit may have died holding one
        m_accept.store(false, std::memory_order_release);
        m_stop.store(true, std::memory_order_release);
        m_cv.notify_all();
        const HANDLE th = static_cast<HANDLE>(m_thread.native_handle());
        if (WaitForSingleObject(th, 0) == WAIT_OBJECT_0)
        {
            m_thread.join(); // already gone: returns at once
            LastChanceFlush();
            return;
        }
        const ULONGLONG deadline = GetTickCount64() + 2000;
        while (!m_writerDone.load(std::memory_order_acquire) && GetTickCount64() < deadline)
            Sleep(1);
        m_thread.detach();
    }

    // Game thread (Start / Stop / Push serialized by the caller). False: empty directory, ring allocation or thread start failed.
    // aStamp = file name time (nullptr: local time now).
    bool Start(const std::wstring& aDir, const SYSTEMTIME* aStamp = nullptr)
    {
        if (m_accept.load(std::memory_order_acquire))
            return true;
        JoinWriter(); // previous session (normally finished already)
        if (aDir.empty())
            return false;
        if (!m_ring)
        {
            m_ring.reset(new (std::nothrow) Record[m_cap]);
            if (!m_ring)
                return false;
        }
        m_dir = aDir;
        if (m_dir.back() != L'\\' && m_dir.back() != L'/')
            m_dir += L'\\';
        if (aStamp)
            m_stamp = *aStamp;
        else
            GetLocalTime(&m_stamp);
        {
            std::lock_guard lk(m_infoMutex);
            m_path = m_dir + FileName(m_stamp);
            m_error.clear();
        }
        m_head.store(0, std::memory_order_relaxed);
        m_tail.store(0, std::memory_order_relaxed);
        m_pushed.store(0, std::memory_order_relaxed);
        m_dropped.store(0, std::memory_order_relaxed);
        m_written.store(0, std::memory_order_relaxed);
        m_failed.store(false, std::memory_order_relaxed);
        m_stop.store(false, std::memory_order_relaxed);
        m_wake.store(false, std::memory_order_relaxed);
        m_writerDone.store(false, std::memory_order_relaxed);
        m_dropLogged = false;
        m_file = INVALID_HANDLE_VALUE;
        m_accept.store(true, std::memory_order_release);
        try
        {
            m_thread = std::thread(&Dumper::WriterMain, this);
        }
        catch (...)
        {
            m_accept.store(false, std::memory_order_release);
            return false;
        }
        ++m_sessions;
        return true;
    }

    // Stops accepting; the writer writes what is pending, closes the file and exits (not waited for).
    void Stop()
    {
        m_accept.store(false, std::memory_order_release);
        if (!m_thread.joinable())
            return;
        {
            std::lock_guard lk(m_cvMutex);
            m_stop.store(true, std::memory_order_release);
        }
        m_cv.notify_all();
    }

    // Stop + wait for the writer (unload, tests).
    void Shutdown()
    {
        Stop();
        JoinWriter();
    }

    PushResult Push(const Record& aRec)
    {
        if (!m_accept.load(std::memory_order_acquire) || m_failed.load(std::memory_order_relaxed))
            return PushResult::Off;
        const uint64_t head = m_head.load(std::memory_order_relaxed);
        const uint64_t tail = m_tail.load(std::memory_order_acquire);
        if (head - tail >= m_cap)
        {
            m_dropped.fetch_add(1, std::memory_order_relaxed);
            m_totalDropped.fetch_add(1, std::memory_order_relaxed);
            if (!m_dropLogged)
            {
                m_dropLogged = true;
                return PushResult::FirstDrop;
            }
            return PushResult::Dropped;
        }
        m_ring[static_cast<size_t>(head & (m_cap - 1))] = aRec;
        m_head.store(head + 1, std::memory_order_release);
        m_pushed.fetch_add(1, std::memory_order_relaxed);
        m_totalPushed.fetch_add(1, std::memory_order_relaxed);
        if (head + 1 - tail == m_cap / 2)
        {
            m_wake.store(true, std::memory_order_release); // early drain; without the lock a missed wake waits <= 1 s
            m_cv.notify_all();
        }
        return PushResult::Ok;
    }

    bool Accepting() const { return m_accept.load(std::memory_order_acquire); }
    bool Failed() const { return m_failed.load(std::memory_order_acquire); }
    uint64_t Pushed() const { return m_pushed.load(std::memory_order_relaxed); }
    uint64_t Dropped() const { return m_dropped.load(std::memory_order_relaxed); }
    uint64_t Written() const { return m_written.load(std::memory_order_acquire); }
    uint64_t Sessions() const { return m_sessions; }
    uint32_t Capacity() const { return m_cap; }
    std::wstring Path() const
    {
        std::lock_guard lk(m_infoMutex);
        return m_path;
    }
    std::string Error() const
    {
        std::lock_guard lk(m_infoMutex);
        return m_error;
    }
    // "on= lines= written= dropped= sessions= totalLines= totalDropped= ring= error= file=<path>" (no quotes; the path is last).
    std::string Status() const
    {
        std::string err, path;
        {
            std::lock_guard lk(m_infoMutex);
            err = m_error.empty() ? std::string("-") : m_error;
            path = detail::Narrow(m_path);
        }
        char b[384];
        std::snprintf(b, sizeof b, "on=%d lines=%llu written=%llu dropped=%llu sessions=%llu totalLines=%llu totalDropped=%llu ring=%u error=%s file=",
                      Accepting() ? 1 : 0, static_cast<unsigned long long>(Pushed()), static_cast<unsigned long long>(Written()),
                      static_cast<unsigned long long>(Dropped()), static_cast<unsigned long long>(m_sessions),
                      static_cast<unsigned long long>(m_totalPushed.load(std::memory_order_relaxed)),
                      static_cast<unsigned long long>(m_totalDropped.load(std::memory_order_relaxed)), m_cap, err.c_str());
        return std::string(b) + (path.empty() ? std::string("-") : path);
    }

    // Test hooks: a paused writer does not drain (except on stop) - the ring fills and Push drops; TestWake drains now.
    void TestPauseWriter(bool aPause) { m_pause.store(aPause, std::memory_order_release); }
    void TestWake()
    {
        {
            std::lock_guard lk(m_cvMutex);
            m_wake.store(true, std::memory_order_release);
        }
        m_cv.notify_all();
    }

private:
    static constexpr size_t kWriteBuffer = 64 * 1024;

    void JoinWriter()
    {
        if (m_thread.joinable())
            m_thread.join();
    }

    void Fail(const char* aWhat, DWORD aCode)
    {
        char b[96];
        std::snprintf(b, sizeof b, "%s_%lu", aWhat, static_cast<unsigned long>(aCode));
        {
            std::lock_guard lk(m_infoMutex);
            m_error = b;
        }
        m_failed.store(true, std::memory_order_release);
        m_accept.store(false, std::memory_order_release);
    }

    static bool WriteAll(HANDLE aFile, const char* aData, size_t aLen, DWORD& aErr)
    {
        while (aLen > 0)
        {
            DWORD done = 0;
            const DWORD chunk = static_cast<DWORD>(std::min<size_t>(aLen, 1u << 20));
            if (!WriteFile(aFile, aData, chunk, &done, nullptr) || done == 0)
            {
                aErr = GetLastError();
                return false;
            }
            aData += done;
            aLen -= done;
        }
        return true;
    }

    // Formats and writes every pending record (writer thread, or the destructor once the writer is gone).
    bool Drain(HANDLE aFile, char* aBuf)
    {
        uint64_t tail = m_tail.load(std::memory_order_relaxed);
        const uint64_t head = m_head.load(std::memory_order_acquire);
        size_t used = 0;
        uint64_t lines = 0;
        DWORD err = 0;
        while (tail != head)
        {
            if (used + kMaxLine > kWriteBuffer)
            {
                if (!WriteAll(aFile, aBuf, used, err))
                {
                    Fail("WriteFile", err);
                    return false;
                }
                m_written.fetch_add(lines, std::memory_order_release);
                used = 0;
                lines = 0;
            }
            used += FormatLine(m_ring[static_cast<size_t>(tail & (m_cap - 1))], aBuf + used, kWriteBuffer - used);
            ++lines;
            ++tail;
            m_tail.store(tail, std::memory_order_release); // slot free for the producer
        }
        if (used > 0 && !WriteAll(aFile, aBuf, used, err))
        {
            Fail("WriteFile", err);
            return false;
        }
        m_written.fetch_add(lines, std::memory_order_release);
        return true;
    }

    void WriterMain()
    {
        HANDLE file = INVALID_HANDLE_VALUE;
        std::wstring path;
        DWORD err = 0;
        for (int i = 1; i <= 99 && file == INVALID_HANDLE_VALUE; ++i)
        {
            path = m_dir + FileName(m_stamp, i);
            file = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file == INVALID_HANDLE_VALUE)
            {
                err = GetLastError();
                if (err != ERROR_FILE_EXISTS && err != ERROR_ALREADY_EXISTS)
                    break;
            }
        }
        {
            std::lock_guard lk(m_infoMutex);
            m_path = path;
        }
        std::unique_ptr<char[]> buf(new (std::nothrow) char[kWriteBuffer]);
        if (file == INVALID_HANDLE_VALUE || !buf)
        {
            if (file != INVALID_HANDLE_VALUE)
                CloseHandle(file);
            Fail(file == INVALID_HANDLE_VALUE ? "CreateFileW" : "alloc", err);
            m_writerDone.store(true, std::memory_order_release);
            return;
        }
        m_file = file;
        std::string header(kHeader);
        header += '\n';
        bool ok = WriteAll(file, header.data(), header.size(), err);
        if (!ok)
            Fail("WriteFile", err);
        while (ok)
        {
            {
                std::unique_lock lk(m_cvMutex);
                m_cv.wait_for(lk, std::chrono::seconds(1), [this] {
                    return m_stop.load(std::memory_order_acquire) || m_wake.load(std::memory_order_acquire);
                });
                m_wake.store(false, std::memory_order_relaxed);
            }
            const bool stop = m_stop.load(std::memory_order_acquire);
            if (stop || !m_pause.load(std::memory_order_acquire))
                ok = Drain(file, buf.get());
            if (stop)
                break;
        }
        m_file = INVALID_HANDLE_VALUE;
        CloseHandle(file);
        m_writerDone.store(true, std::memory_order_release);
    }

    void LastChanceFlush()
    {
        if (m_file == INVALID_HANDLE_VALUE)
            return;
        std::unique_ptr<char[]> buf(new (std::nothrow) char[kWriteBuffer]);
        if (buf)
            Drain(m_file, buf.get());
        CloseHandle(m_file);
        m_file = INVALID_HANDLE_VALUE;
    }

    uint32_t m_cap = kDefaultRing;
    std::unique_ptr<Record[]> m_ring;
    std::atomic<uint64_t> m_head{0}; // producer
    std::atomic<uint64_t> m_tail{0}; // consumer
    std::atomic<bool> m_accept{false};
    std::atomic<bool> m_stop{false};
    std::atomic<bool> m_wake{false};
    std::atomic<bool> m_pause{false};
    std::atomic<bool> m_failed{false};
    std::atomic<bool> m_writerDone{false};
    std::atomic<uint64_t> m_pushed{0}, m_dropped{0}, m_written{0}, m_totalPushed{0}, m_totalDropped{0};
    bool m_dropLogged = false; // producer only
    uint64_t m_sessions = 0;   // Start only
    std::wstring m_dir;
    SYSTEMTIME m_stamp{};
    HANDLE m_file = INVALID_HANDLE_VALUE; // writer thread (read by the destructor only after the writer is gone)
    mutable std::mutex m_infoMutex;       // m_path / m_error
    std::wstring m_path;
    std::string m_error;
    std::mutex m_cvMutex;
    std::condition_variable m_cv;
    std::thread m_thread;
};
} // namespace utpp::mm::feeddump

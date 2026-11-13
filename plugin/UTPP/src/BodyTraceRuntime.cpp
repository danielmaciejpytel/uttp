#include "BodyTraceRuntime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>

#include <Windows.h>

#include "CommonUtil.hpp"
#include "mm/BodyTraceContinuous.hpp"
#include "mm/CaptureOwner.hpp"

namespace utpp::bodytraceruntime
{
namespace
{
using namespace mm::bodytrace;

int64_t NowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Buffered file: 4 MB stdio buffer, flushed at least every 2 s so a crash loses only the tail.
class FileSink final : public IByteSink
{
public:
    ~FileSink() override { Close(); }
    bool Create(const std::filesystem::path& aPath)
    {
        Close();
        FILE* f = nullptr;
        if (_wfopen_s(&f, aPath.c_str(), L"wbx") != 0 || !f) // exclusive create: never overwrite an earlier capture
            return false;
        m_f = f;
        m_path = aPath;
        std::setvbuf(m_f, nullptr, _IOFBF, 4u << 20);
        m_bytes = 0;
        m_failed = false;
        m_lastFlush = NowMs();
        return true;
    }
    bool Write(const char* aData, size_t aSize) override
    {
        if (!m_f || m_failed)
            return false;
        if (std::fwrite(aData, 1, aSize, m_f) != aSize)
        {
            m_failed = true;
            return false;
        }
        m_bytes += aSize;
        const int64_t now = NowMs();
        if (now - m_lastFlush >= 2000)
        {
            m_lastFlush = now;
            if (std::fflush(m_f) != 0)
            {
                m_failed = true;
                return false;
            }
        }
        return true;
    }
    uint64_t Bytes() const override { return m_bytes; }
    FILE* File() { return m_f; }
    bool IsOpen() const { return m_f != nullptr; }
    bool Failed() const { return m_failed; }
    const std::filesystem::path& Path() const { return m_path; }
    bool Close()
    {
        if (!m_f)
            return true;
        const bool ok = std::fclose(m_f) == 0 && !m_failed;
        m_f = nullptr;
        return ok;
    }

private:
    FILE* m_f = nullptr;
    std::filesystem::path m_path;
    uint64_t m_bytes = 0;
    int64_t m_lastFlush = 0;
    bool m_failed = false;
};

class FileEnv final : public IEnv
{
public:
    LogFn log = nullptr;
    std::wstring folder;
    std::wstring root; // game folder: log lines show paths relative to it (never an absolute path)

    std::string Rel(const std::filesystem::path& aPath) const { return util::RelativeForLog(aPath.wstring(), root); }

    int64_t NowMs() override { return utpp::bodytraceruntime::NowMs(); }
    void Log(const std::string& aLine) override
    {
        if (log)
            log(aLine.c_str());
    }
    bool Open(const Map& aMap, const Topology& aTopo, bool aPost, uint32_t aBudget) override
    {
        if (folder.empty())
        {
            Log("UTPP body trace continuous: no log folder");
            return false;
        }
        SYSTEMTIME lt{};
        GetLocalTime(&lt);
        char stamp[32];
        std::snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u-%02u-%02u-%02u", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond);
        bool ok = false;
        std::filesystem::path legacyPath, postPath;
        for (int suffix = 0; suffix < 100 && !ok; ++suffix)
        {
            const std::string tail = suffix ? "-" + std::to_string(suffix + 1) : "";
            legacyPath = std::filesystem::path(folder) / ("utpp-bodytrace-" + std::string(stamp) + tail + ".csv");
            postPath = std::filesystem::path(folder) / ("utpp-bodytrace-posthelper-" + std::string(stamp) + tail + ".csv");
            if (!m_sink[0].Create(legacyPath))
                continue;
            if (aPost && !m_sink[1].Create(postPath))
            {
                m_sink[0].Close();
                std::error_code ec;
                std::filesystem::remove(legacyPath, ec);
                continue;
            }
            ok = true;
        }
        if (!ok)
        {
            Log("UTPP body trace continuous: cannot create the capture files in " + util::RelativeForLog(folder, root));
            return false;
        }
        LARGE_INTEGER ticks{}, hz{};
        QueryPerformanceCounter(&ticks);
        QueryPerformanceFrequency(&hz);
        SYSTEMTIME utc{};
        GetSystemTime(&utc);
        char head[512];
        std::snprintf(head, sizeof(head),
                      "# qpc_hz=%lld arm_ticks=%lld utc=%04u-%02u-%02uT%02u:%02u:%02u.%03uZ max_frames=%u continuous=1 stages=before_mm,after_mm_and_aim scope=main_rig_helpers\n",
                      hz.QuadPart, ticks.QuadPart, utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond, utc.wMilliseconds, aBudget);
        Header(m_sink[0], head, aTopo, "ticks,desc,thread,rig,generation,rigCount,name,gameIndex", 24, "s");
        if (aPost)
        {
            std::snprintf(head, sizeof(head),
                          "# qpc_hz=%lld arm_ticks=%lld utc=%04u-%02u-%02uT%02u:%02u:%02u.%03uZ max_frames=%u continuous=1 stage=post_helper_apply_after_commit scope=main_rig_helpers; correlate by ticks,desc,rig,thread with legacy file\n",
                          hz.QuadPart, ticks.QuadPart, utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond, utc.wMilliseconds, aBudget);
            Header(m_sink[1], head, aTopo,
                   "ticks,desc,thread,rig,generation,rigCount,receiptGeneration,receiptCycle,receiptStartedTicks,receiptLastTicks,name,gameIndex", 12, "local_");
        }
        m_open[0] = true;
        m_open[1] = aPost;
        m_rig = aMap.rig;
        return true;
    }
    IByteSink* Sink(int aStream) override { return aStream >= 0 && aStream < 2 && m_open[aStream] ? &m_sink[aStream] : nullptr; }
    void Close(const Totals& aTotals) override
    {
        for (int s = 0; s < 2; ++s)
        {
            if (!m_open[s])
                continue;
            if (m_sink[s].File())
                std::fprintf(m_sink[s].File(), "# frames=%llu dropped=%llu failed=%d\n", static_cast<unsigned long long>(aTotals.frames[s]),
                             static_cast<unsigned long long>(aTotals.dropped[s]), (aTotals.failed || m_sink[s].Failed()) ? 1 : 0);
            m_last[s] = m_sink[s].Path();
            m_sink[s].Close();
            m_open[s] = false;
        }
    }
    std::string Describe() override
    {
        std::string d = "file=";
        const bool o = m_open[0];
        d += Rel(o ? m_sink[0].Path() : m_last[0]);
        if (o ? m_open[1] : !m_last[1].empty())
            d += " postFile=" + Rel(o ? m_sink[1].Path() : m_last[1]);
        return d;
    }

private:
    static void Header(FileSink& aSink, const char* aHead, const Topology& aTopo, const char* aColumns, int aValues, const char* aPrefix)
    {
        FILE* f = aSink.File();
        if (!f)
            return;
        std::fputs(aHead, f);
        // Owned rig topology (offline skinning needs these parents; the mesh joint nodes cannot substitute for them).
        for (size_t i = 0; i < aTopo.hashes.size(); ++i)
            std::fprintf(f, "# rig_bone index=%u hash=%016llX parent=%d\n", static_cast<unsigned>(i), static_cast<unsigned long long>(aTopo.hashes[i]),
                         i < aTopo.parents.size() ? static_cast<int>(aTopo.parents[i]) : -1);
        std::fputs(aColumns, f);
        if (std::strcmp(aPrefix, "s") == 0)
        {
            for (int stage = 0; stage < 2; ++stage)
                for (int v = 0; v < 12; ++v)
                    std::fprintf(f, ",s%d_%d", stage, v);
        }
        else
            for (int v = 0; v < aValues; ++v)
                std::fprintf(f, ",%s%d", aPrefix, v);
        std::fputc('\n', f);
        std::fflush(f);
    }

    FileSink m_sink[2];
    bool m_open[2] = {false, false};
    std::filesystem::path m_last[2];
    uint64_t m_rig = 0;
};

struct Runtime
{
    FileEnv env;
    CaptureStream<Capture, Frame> legacy{g_capture};
    CaptureStream<PostHelperCapture, PostHelperFrame> post{g_postHelperCapture};
    ContinuousController controller{env, legacy, post, ContinuousController::Config{}};
    std::thread thread;
    std::mutex cvMutex;
    std::condition_variable cv;
    bool stop = false;
    bool inited = false;
};

std::mutex g_controlMutex; // Init / Enable / Shutdown (control path)
std::unique_ptr<Runtime> g_rt;

void ThreadMain(Runtime* aRt)
{
    for (;;)
    {
        {
            std::lock_guard lock(aRt->cvMutex);
            if (aRt->stop)
                break;
        }
        if (aRt->controller.Tick() == 0)
        {
            std::unique_lock lock(aRt->cvMutex);
            aRt->cv.wait_for(lock, std::chrono::milliseconds(3), [&] { return aRt->stop; });
        }
    }
    aRt->controller.Shutdown(); // final drain + close
}

void StopThreadLocked()
{
    if (!g_rt || !g_rt->thread.joinable())
        return;
    {
        std::lock_guard lock(g_rt->cvMutex);
        g_rt->stop = true;
    }
    g_rt->cv.notify_all();
    g_rt->thread.join();
}
} // namespace

void Init(LogFn aLog, const std::wstring& aFolder)
{
    std::lock_guard lock(g_controlMutex);
    if (!g_rt)
        g_rt = std::make_unique<Runtime>();
    g_rt->env.log = aLog;
    g_rt->env.folder = aFolder;
    g_rt->env.root = util::GameRootOfLogFolder(aFolder);
    g_rt->inited = true;
}

bool Enable(bool aOn)
{
    std::lock_guard lock(g_controlMutex);
    if (aOn)
    {
        if (!g_rt || !g_rt->inited || g_rt->env.folder.empty())
            return false;
        if (g_rt->thread.joinable())
            return true; // idempotent
        if (!mm::bodytrace::AcquireRings(mm::bodytrace::RingOwner::TextCapture))
        {
            g_rt->env.Log("UTPP body trace continuous: refused, the compact pose capture is running");
            return false;
        }
        g_rt->stop = false;
        g_rt->controller.Enable(true);
        g_rt->thread = std::thread(ThreadMain, g_rt.get());
        return true;
    }
    if (g_rt)
    {
        g_rt->controller.Enable(false);
        StopThreadLocked();
        mm::bodytrace::ReleaseRings(mm::bodytrace::RingOwner::TextCapture);
    }
    return true;
}

bool Enabled()
{
    std::lock_guard lock(g_controlMutex);
    return g_rt && g_rt->thread.joinable();
}

void PublishMap(const Map& aMap, bool aPost, std::vector<uint64_t> aHashes, std::vector<int16_t> aParents)
{
    std::lock_guard lock(g_controlMutex);
    if (!g_rt || !g_rt->thread.joinable())
        return;
    auto topo = std::make_shared<Topology>();
    topo->hashes = std::move(aHashes);
    topo->parents = std::move(aParents);
    g_rt->controller.SetMap(aMap, aPost, std::move(topo));
}

void ClearMap()
{
    std::lock_guard lock(g_controlMutex);
    if (g_rt)
        g_rt->controller.ClearMap();
}

std::string Status()
{
    std::lock_guard lock(g_controlMutex);
    if (!g_rt)
        return "bodyPoseTraceContinuous on=0";
    const auto s = g_rt->controller.GetStats();
    return "bodyPoseTraceContinuous on=" + std::to_string(g_rt->thread.joinable() ? 1 : 0) + " running=" + std::to_string(s.running ? 1 : 0) +
           " frames=" + std::to_string(s.frames[0]) + " dropped=" + std::to_string(s.dropped[0]) + " sessions=" + std::to_string(s.sessions) +
           " capped=" + std::to_string(s.capped ? 1 : 0) + " failed=" + std::to_string(s.failed ? 1 : 0);
}

void Shutdown()
{
    std::lock_guard lock(g_controlMutex);
    if (g_rt)
    {
        g_rt->controller.Enable(false);
        StopThreadLocked();
        mm::bodytrace::ReleaseRings(mm::bodytrace::RingOwner::TextCapture);
    }
}
} // namespace utpp::bodytraceruntime

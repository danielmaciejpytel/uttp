#include "PoseCaptureRuntime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>

#include <Windows.h>

#include "CommonUtil.hpp"
#include "PoseCaptureCompression.hpp"
#include "mm/CaptureOwner.hpp"
#include "mm/PoseCaptureSession.hpp"

namespace utpp::posecaptureruntime
{
namespace
{
using namespace mm::bodytrace;
using namespace mm::posecapture;

// Measured on a real recording (60 frames/s, 185 bones): XPRESS with Huffman coding writes 4.5-7 MB per minute and costs about 2-3 ms of
// writer-thread time per second of recording (worst block 22 ms); LZMS gives the same size at 25 times the time.
constexpr uint32_t kAlgorithm = kAlgorithmXpress;
constexpr int32_t kMaxKeepMinutes = 120;

class DiskFile final : public IPoseFile
{
public:
    explicit DiskFile(FILE* aFile) : m_f(aFile) { std::setvbuf(m_f, nullptr, _IOFBF, 1u << 20); }
    ~DiskFile() override { Close(); }
    bool Write(const uint8_t* aData, size_t aSize) override
    {
        if (!m_f || m_failed)
            return false;
        if (std::fwrite(aData, 1, aSize, m_f) != aSize)
            m_failed = true;
        // whole writes are blocks (seconds apart): hand every one to the OS so a crash of the game loses at most the pending block
        else if (std::fflush(m_f) != 0)
            m_failed = true;
        return !m_failed;
    }
    bool Close() override
    {
        if (!m_f)
            return !m_failed;
        const bool ok = std::fclose(m_f) == 0 && !m_failed;
        m_f = nullptr;
        return ok;
    }

private:
    FILE* m_f;
    bool m_failed = false;
};

class DiskStorage final : public IPoseStorage
{
public:
    std::wstring folder, root;
    std::unique_ptr<IPoseFile> Create(const std::string& aFileName) override
    {
        FILE* f = nullptr;
        // exclusive create: never overwrite an earlier capture
        if (_wfopen_s(&f, (std::filesystem::path(folder) / aFileName).c_str(), L"wbx") != 0 || !f)
            return nullptr;
        return std::make_unique<DiskFile>(f);
    }
    bool Remove(const std::string& aFileName) override
    {
        std::error_code ec;
        return std::filesystem::remove(std::filesystem::path(folder) / aFileName, ec) && !ec;
    }
    void RemoveStale(const std::string& aPrefix, const std::string& aExtension, int64_t aOlderThanMs) override
    {
        // Only plain files of this folder (no recursion, no links), named <prefix>*<extension>, last written more than the window ago.
        try
        {
            namespace fs = std::filesystem;
            const std::wstring prefix(aPrefix.begin(), aPrefix.end()), ext(aExtension.begin(), aExtension.end());
            std::error_code ec;
            const auto now = fs::file_time_type::clock::now();
            const auto minAge = std::chrono::milliseconds(aOlderThanMs);
            for (fs::directory_iterator it(folder, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec))
            {
                try
                {
                    const std::wstring name = it->path().filename().wstring();
                    if (name.size() < prefix.size() + ext.size() || name.compare(0, prefix.size(), prefix) != 0 ||
                        _wcsicmp(name.c_str() + (name.size() - ext.size()), ext.c_str()) != 0)
                        continue;
                    std::error_code fec;
                    const auto st = it->symlink_status(fec);
                    const DWORD attrs = GetFileAttributesW(it->path().c_str());
                    if (fec || !fs::is_regular_file(st) || attrs == INVALID_FILE_ATTRIBUTES || (attrs & FILE_ATTRIBUTE_REPARSE_POINT) != 0)
                        continue;
                    const auto written = fs::last_write_time(it->path(), fec);
                    if (fec || now - written <= minAge)
                        continue;
                    fs::remove(it->path(), fec); // a locked file stays
                }
                catch (...)
                {
                }
            }
        }
        catch (...)
        {
        }
    }
    std::string DisplayPath(const std::string& aFileName) const override
    {
        return util::RelativeForLog(aFileName.empty() ? folder : (std::filesystem::path(folder) / aFileName).wstring(), root);
    }
};

class Host final : public IPoseHost
{
public:
    LogFn log = nullptr;
    int64_t NowMs() override
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    HostTime Time() override
    {
        HostTime t;
        SYSTEMTIME lt{}, utc{};
        GetLocalTime(&lt);
        GetSystemTime(&utc);
        char stamp[32];
        std::snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u-%02u-%02u-%02u", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond);
        t.stamp = stamp;
        t.utc[0] = utc.wYear; t.utc[1] = utc.wMonth; t.utc[2] = utc.wDay; t.utc[3] = utc.wHour;
        t.utc[4] = utc.wMinute; t.utc[5] = utc.wSecond; t.utc[6] = utc.wMilliseconds;
        t.qpcHz = util::QpcFrequency();
        t.qpcTicks = util::QpcTicks();
        return t;
    }
    void Log(const std::string& aLine) override
    {
        if (log)
            log(aLine.c_str());
    }
};

struct Session
{
    Host host;
    DiskStorage storage;
    utpp::posecapture::WindowsCompressor compressor{kAlgorithm};
    PoseEnv env;
    PoseCaptureStream<Capture, Frame> legacy{g_capture};
    PoseCaptureStream<PostHelperCapture, PostHelperFrame> post{g_postHelperCapture};
    ContinuousController controller;
    int32_t keepMinutes = 0;
    std::thread thread;
    std::mutex cvMutex;
    std::condition_variable cv;
    bool stop = false;
    bool running = false;
    // snapshot for Status (written by the writer thread, read under snapMutex / as atomics)
    std::atomic<uint64_t> frames{0}, dropped{0}, bytes{0};
    std::atomic<bool> capped{false}, failed{false};
    std::mutex snapMutex;
    std::string currentFile;
    uint32_t fileCount = 0;

    Session(LogFn aLog, const std::wstring& aFolder, const std::wstring& aRoot, int32_t aKeepMinutes)
        : env(host, storage, compressor, MakeConfig(aKeepMinutes)),
          controller(env, legacy, post, CompactControllerConfig()),
          keepMinutes(aKeepMinutes)
    {
        host.log = aLog;
        storage.folder = aFolder;
        storage.root = aRoot;
        // segment trailers carry the frames lost since the previous segment: the running file's counters
        env.SetDroppedProbe([this](uint64_t (&aDropped)[2]) {
            const auto s = controller.GetStats();
            aDropped[0] = s.dropped[0];
            aDropped[1] = s.dropped[1];
        });
    }
    static PoseEnv::Config MakeConfig(int32_t aKeepMinutes)
    {
        PoseEnv::Config c;
        c.keepMinutes = static_cast<uint32_t>(aKeepMinutes);
        return c;
    }
    void Snapshot()
    {
        const auto s = controller.GetStats();
        frames.store(s.lifetimeFrames[0] + s.frames[0]);
        dropped.store(s.lifetimeDropped[0] + s.dropped[0] + s.lifetimeDropped[1] + s.dropped[1]);
        bytes.store(env.TotalBytes());
        capped.store(s.capped);
        failed.store(s.failed || env.Failed());
        const std::string describe = env.Describe();
        std::lock_guard lock(snapMutex);
        currentFile = describe.rfind("file=", 0) == 0 ? describe.substr(5) : describe;
        fileCount = static_cast<uint32_t>(env.SessionFiles().size());
    }
};

std::mutex g_controlMutex; // Init / Start / Stop / PublishMap (control path)
LogFn g_log = nullptr;
std::wstring g_folder, g_root;
std::unique_ptr<Session> g_session;

void ThreadMain(Session* aS)
{
    int64_t nextSnapshot = 0;
    for (;;)
    {
        {
            std::lock_guard lock(aS->cvMutex);
            if (aS->stop)
                break;
        }
        const uint32_t drained = aS->controller.Tick();
        const int64_t now = aS->host.NowMs();
        if (now >= nextSnapshot)
        {
            nextSnapshot = now + 500;
            aS->Snapshot();
        }
        if (drained == 0)
        {
            std::unique_lock lock(aS->cvMutex);
            aS->cv.wait_for(lock, std::chrono::milliseconds(3), [&] { return aS->stop; });
        }
    }
    aS->controller.Shutdown(); // final drain, pending block, trailer
    aS->Snapshot();
}

void StopThreadLocked()
{
    if (!g_session || !g_session->thread.joinable())
        return;
    {
        std::lock_guard lock(g_session->cvMutex);
        g_session->stop = true;
    }
    g_session->cv.notify_all();
    g_session->thread.join();
    g_session->running = false;
    ReleaseRings(RingOwner::CompactCapture);
}

std::string JoinFiles(const std::vector<std::string>& aFiles)
{
    std::string out;
    for (const auto& f : aFiles)
    {
        if (!out.empty())
            out += '|';
        out += f;
    }
    return out;
}
} // namespace

void Init(LogFn aLog, const std::wstring& aFolder)
{
    std::lock_guard lock(g_controlMutex);
    g_log = aLog;
    g_folder = aFolder;
    g_root = util::GameRootOfLogFolder(aFolder);
}

bool Start(int32_t aKeepMinutes)
{
    std::lock_guard lock(g_controlMutex);
    if (g_session && g_session->running)
    {
        // already recording: the same window changes nothing; another window needs a Stop first (the files of a session have one window)
        if (g_session->keepMinutes == aKeepMinutes)
            return true;
        if (g_log)
            g_log("UTPP pose capture: refused, a capture with another window is running (stop it first)");
        return false;
    }
    if (g_folder.empty() || aKeepMinutes < 0 || aKeepMinutes > kMaxKeepMinutes)
        return false;
    if (!AcquireRings(RingOwner::CompactCapture))
    {
        if (g_log)
            g_log("UTPP pose capture: refused, the text capture of the helper bones is running");
        return false;
    }
    auto session = std::make_unique<Session>(g_log, g_folder, g_root, aKeepMinutes);
    if (!session->compressor.Valid())
    {
        ReleaseRings(RingOwner::CompactCapture);
        if (g_log)
            g_log("UTPP pose capture: the Windows compression API is not available");
        return false;
    }
    g_session = std::move(session);
    g_session->controller.Enable(true);
    g_session->running = true;
    g_session->thread = std::thread(ThreadMain, g_session.get());
    return true;
}

std::string Stop()
{
    std::lock_guard lock(g_controlMutex);
    if (!g_session)
        return {};
    StopThreadLocked();
    return JoinFiles(g_session->env.SessionFiles());
}

std::string Status()
{
    std::lock_guard lock(g_controlMutex);
    if (!g_session)
        return "poseCapture state=off";
    Session& s = *g_session;
    std::string file;
    uint32_t files = 0;
    {
        std::lock_guard snap(s.snapMutex);
        file = s.currentFile;
        files = s.fileCount;
    }
    return std::string("poseCapture state=") + (s.running ? "recording" : "stopped") + " keep=" + std::to_string(s.keepMinutes) +
           " frames=" + std::to_string(s.frames.load()) + " dropped=" + std::to_string(s.dropped.load()) + " bytes=" + std::to_string(s.bytes.load()) +
           " files=" + std::to_string(files) + " capped=" + std::to_string(s.capped.load() ? 1 : 0) + " failed=" + std::to_string(s.failed.load() ? 1 : 0) +
           " file=" + file;
}

bool Enabled()
{
    std::lock_guard lock(g_controlMutex);
    return g_session && g_session->running;
}

void PublishMap(const Map& aMap, bool aPost, std::vector<uint64_t> aHashes, std::vector<int16_t> aParents)
{
    std::lock_guard lock(g_controlMutex);
    if (!g_session || !g_session->running)
        return;
    auto topo = std::make_shared<Topology>();
    topo->hashes = std::move(aHashes);
    topo->parents = std::move(aParents);
    g_session->controller.SetMap(aMap, aPost, std::move(topo));
}

void ClearMap()
{
    std::lock_guard lock(g_controlMutex);
    if (g_session && g_session->running)
        g_session->controller.ClearMap();
}

void Shutdown()
{
    std::lock_guard lock(g_controlMutex);
    if (g_session)
        StopThreadLocked();
}
} // namespace utpp::posecaptureruntime

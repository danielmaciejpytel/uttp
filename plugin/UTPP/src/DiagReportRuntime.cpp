#include "DiagReportRuntime.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <system_error>
#include <vector>

#include <Windows.h>

#include "CommonUtil.hpp"
#include "DiagReport.hpp"

namespace utpp::diagreportruntime
{
namespace
{
namespace fs = std::filesystem;

std::string FileNameUtf8(const fs::path& aPath) { return util::WideToUtf8(aPath.filename().wstring()); }

std::vector<diagreport::FileEntry> ListFiles(const std::wstring& aFolder, const std::wstring& aRoot)
{
    std::vector<diagreport::FileEntry> out;
    std::error_code ec;
    const auto now = fs::file_time_type::clock::now();
    for (fs::directory_iterator it(fs::path(aFolder), ec), end; !ec && it != end; it.increment(ec))
    {
        std::error_code e1, e2, e3;
        if (!it->is_regular_file(e1))
            continue;
        diagreport::FileEntry f;
        f.name = FileNameUtf8(it->path());
        if (!diagreport::IsModFile(f.name))
            continue;
        f.bytes = static_cast<uint64_t>(it->file_size(e2));
        const auto written = it->last_write_time(e3);
        if (e2 || e3)
            continue;
        f.ageSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - written).count();
        f.relPath = util::RelativeForLog(it->path().wstring(), aRoot);
        out.push_back(std::move(f));
    }
    return out;
}
} // namespace

std::string WriteReport(const std::wstring& aLogFolder, const std::string& aModVersion, const std::string& aGameVersion, const std::string& aBody)
{
    if (aLogFolder.empty())
        return {};
    const std::wstring root = util::GameRootOfLogFolder(aLogFolder);
    SYSTEMTIME lt{};
    GetLocalTime(&lt);
    char stamp[32], date[32];
    std::snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u-%02u-%02u-%02u", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond);
    std::snprintf(date, sizeof(date), "%04u-%02u-%02u %02u:%02u:%02u", lt.wYear, lt.wMonth, lt.wDay, lt.wHour, lt.wMinute, lt.wSecond);

    const std::vector<diagreport::FileEntry> all = ListFiles(aLogFolder, root);
    for (int suffix = 0; suffix < 100; ++suffix)
    {
        const std::string name = std::string("utpp-report-") + stamp + (suffix ? "-" + std::to_string(suffix + 1) : std::string()) + ".txt";
        const fs::path path = fs::path(aLogFolder) / name;
        FILE* f = nullptr;
        if (_wfopen_s(&f, path.c_str(), L"wbx") != 0 || !f) // exclusive create: never overwrite an earlier report
            continue;
        const std::string rel = util::RelativeForLog(path.wstring(), root);
        diagreport::Info info;
        info.modVersion = aModVersion;
        info.gameVersion = aGameVersion;
        info.date = date;
        info.body = aBody;
        const std::string text = diagreport::Compose(info, rel, diagreport::SelectRecentFiles(all, diagreport::kRecentSeconds, name));
        const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
        const bool closed = std::fclose(f) == 0;
        if (!ok || !closed)
        {
            std::error_code ec;
            fs::remove(path, ec);
            return {};
        }
        return rel;
    }
    return {};
}
} // namespace utpp::diagreportruntime

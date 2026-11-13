#pragma once
// Diagnostic report file written for players (UTPPDiagWriteReport): pure text logic, no Win32. The production side (listing the log folder,
// writing the file) is in src/DiagReportRuntime.cpp. The report never contains an absolute path: file entries are paths relative to the
// game folder.
//
//   UTPP diagnostic report
//   mod: UTPP 0.1.0
//   game: Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)
//   date: 2026-11-11 04:20:11
//
//   <body from the script: settings, what the player did, ...>
//
//   Zip the files listed below and send them to the mod author.
//   red4ext\logs\utpp-report-2026-11-11-04-20-11.txt  (this file)
//   red4ext\logs\utpp-pose-2026-11-11-04-12-09.utpppose  (7.9 MB)
//   ...
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "PathScrub.hpp"

namespace utpp::diagreport
{
inline constexpr int64_t kRecentSeconds = 30 * 60;
inline constexpr size_t kMaxBodyBytes = 64 * 1024;

struct FileEntry
{
    std::string name;        // file name inside the log folder (utpp-...)
    std::string relPath;     // path relative to the game folder, shown in the report
    uint64_t bytes = 0;
    int64_t ageSeconds = 0;  // now - last write; negative = in the future (clock skew), counted as recent
};

struct Info
{
    std::string modName = "UTPP";
    std::string modVersion;
    std::string gameVersion;  // empty = unknown
    std::string date;         // local time, "yyyy-mm-dd hh:mm:ss"
    std::string body;         // text passed by the script
};

inline std::string Lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}
// The mod's own files: utpp-* (case-insensitive: RED4ext names its log file after the plugin, in whatever case it was given).
inline bool IsModFile(const std::string& aName) { return Lower(aName).rfind("utpp-", 0) == 0; }

// Files to list: the mod's files changed within aWindowSeconds, the report being written (aExclude) left out (it is listed separately), by name.
inline std::vector<FileEntry> SelectRecentFiles(const std::vector<FileEntry>& aAll, int64_t aWindowSeconds, const std::string& aExclude)
{
    std::vector<FileEntry> out;
    for (const auto& f : aAll)
        if (IsModFile(f.name) && f.name != aExclude && f.ageSeconds <= aWindowSeconds)
            out.push_back(f);
    std::sort(out.begin(), out.end(), [](const FileEntry& a, const FileEntry& b) { return Lower(a.name) < Lower(b.name); });
    return out;
}

inline std::string HumanSize(uint64_t aBytes)
{
    char b[48];
    if (aBytes >= (1ull << 20))
        std::snprintf(b, sizeof(b), "%.1f MB", static_cast<double>(aBytes) / (1024.0 * 1024.0));
    else if (aBytes >= 1024)
        std::snprintf(b, sizeof(b), "%.0f KB", static_cast<double>(aBytes) / 1024.0);
    else
        std::snprintf(b, sizeof(b), "%llu bytes", static_cast<unsigned long long>(aBytes));
    return b;
}

// The text comes from a script: whatever absolute path it may carry is removed (pathscrub::RedactAbsolutePaths), so the report never
// exposes the Windows user name or the install folder.
using pathscrub::RedactAbsolutePaths;

// aSelfRelPath = the report's own path (listed first, as "this file").
inline std::string Compose(const Info& aInfo, const std::string& aSelfRelPath, const std::vector<FileEntry>& aFiles)
{
    const std::string body = RedactAbsolutePaths(aInfo.body.size() > kMaxBodyBytes ? aInfo.body.substr(0, kMaxBodyBytes) + "\n[text cut]" : aInfo.body);
    std::string out;
    out += aInfo.modName + " diagnostic report\n";
    out += "mod: " + aInfo.modName + " " + aInfo.modVersion + "\n";
    out += "game: " + (aInfo.gameVersion.empty() ? std::string("unknown") : aInfo.gameVersion) + "\n";
    out += "date: " + aInfo.date + "\n\n";
    out += body;
    if (!body.empty() && body.back() != '\n')
        out += '\n';
    out += "\nZip the files listed below and send them to the mod author.\n";
    out += aSelfRelPath + "  (this file)\n";
    for (const auto& f : aFiles)
        out += f.relPath + "  (" + HumanSize(f.bytes) + ")\n";
    return out;
}
} // namespace utpp::diagreport

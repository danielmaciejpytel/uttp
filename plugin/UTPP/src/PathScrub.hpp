#pragma once
// Absolute paths must never reach a log line or a report: they contain the Windows user name or the game's install folder. Pure text logic (no
// Win32, no RED4ext) so it is unit tested; used by the shared log sink (PluginLog.hpp) and by the diagnostic report (DiagReport.hpp).
//   StripGameRoot       "<game root>\red4ext\logs\x.log"   ->  "red4ext\logs\x.log"   (the root is registered once at load: SetGameRoot)
//   RedactAbsolutePaths any other drive path ("C:\..", "D:/..") or network path ("\\server\..") -> "<path removed>"
// The sources of log lines already print relative paths (util::RelativeForLog in CommonUtil.hpp); this is the safety net behind them.
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace utpp::pathscrub
{
inline std::string& GameRootRef()
{
    static std::string root;
    return root;
}
// UTF-8, any slash type; trailing separators are dropped.
inline void SetGameRoot(std::string aRoot)
{
    while (!aRoot.empty() && (aRoot.back() == '\\' || aRoot.back() == '/'))
        aRoot.pop_back();
    GameRootRef() = std::move(aRoot);
}

inline bool IsSep(char c) { return c == '\\' || c == '/'; }
inline char FoldChar(char c) { return c == '/' ? '\\' : static_cast<char>(std::tolower(static_cast<unsigned char>(c))); }

// Removes "<root><separator>" wherever the root is followed by a separator (case-insensitive, / or \ as one).
inline std::string StripGameRoot(const std::string& aText, const std::string& aRoot)
{
    if (aRoot.empty() || aText.size() <= aRoot.size())
        return aText;
    std::string out;
    out.reserve(aText.size());
    size_t i = 0;
    while (i < aText.size())
    {
        bool match = i + aRoot.size() < aText.size() && IsSep(aText[i + aRoot.size()]);
        for (size_t k = 0; match && k < aRoot.size(); ++k)
            match = FoldChar(aText[i + k]) == FoldChar(aRoot[k]);
        if (match)
        {
            i += aRoot.size() + 1;
            continue;
        }
        out += aText[i++];
    }
    return out;
}

// Drive and network paths up to the end of the line, the next quote, bracket, bar, semicolon or comma (paths may hold spaces: "Steam Library",
// "Program Files"; the text after a path on the same line may go with it: privacy over completeness).
inline std::string RedactAbsolutePaths(const std::string& aText)
{
    auto stop = [](char c) { return c == '\t' || c == '\r' || c == '\n' || c == '"' || c == '\'' || c == '<' || c == '>' || c == '|' || c == ';' || c == ','; };
    std::string out;
    size_t i = 0;
    while (i < aText.size())
    {
        const bool boundary = i == 0 || !std::isalnum(static_cast<unsigned char>(aText[i - 1]));
        const bool drive = boundary && i + 2 < aText.size() && std::isalpha(static_cast<unsigned char>(aText[i])) && aText[i + 1] == ':' && IsSep(aText[i + 2]);
        const bool unc = boundary && i + 2 < aText.size() && aText[i] == '\\' && aText[i + 1] == '\\' && std::isalnum(static_cast<unsigned char>(aText[i + 2]));
        if (drive || unc)
        {
            out += "<path removed>";
            while (i < aText.size() && !stop(aText[i]))
                ++i;
            continue;
        }
        out += aText[i++];
    }
    return out;
}

inline std::string ScrubLine(const std::string& aLine) { return RedactAbsolutePaths(StripGameRoot(aLine, GameRootRef())); }

// printf-style formatting of one log line, then the scrub. The line is formatted here (not by the logger) so the scrub sees the final text.
template <typename... Args>
std::string FormatLine(const char* aFmt, Args... aArgs)
{
    std::string line;
    const int n = std::snprintf(nullptr, 0, aFmt, aArgs...);
    if (n > 0)
    {
        line.resize(static_cast<size_t>(n) + 1);
        std::snprintf(line.data(), line.size(), aFmt, aArgs...);
        line.resize(static_cast<size_t>(n));
    }
    return ScrubLine(line);
}
} // namespace utpp::pathscrub

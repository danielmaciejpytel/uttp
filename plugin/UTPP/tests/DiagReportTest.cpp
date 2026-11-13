// Diagnostic report (src/DiagReport.hpp, src/DiagReportRuntime.cpp) and the path helpers used to keep absolute paths out of everything the DLL writes
// (src/CommonUtil.hpp: GameRootOfLogFolder, RelativeForLog).
//   A) which files count as the mod's own; the 30-minute filter on an injected list (boundaries, clock skew, the report itself, sort order);
//   B) the composed text: header lines, the script's text, the one instruction sentence, "(this file)", sizes; absolute paths in the script's
//      text are removed, relative paths stay; an oversized text is cut;
//   C) path helpers: relative to the game folder (case, slashes, UTF-8), a path outside it shows the file name only, never a drive or a user folder;
//   E) the log sink's scrub (src/PathScrub.hpp, used by PluginLog.hpp): the game folder shows as a relative path, any other drive or network path is removed,
//      ordinary text (times, URLs, percent signs) stays;
//   D) the production writer on a scratch game folder: file name, exclusive create (a second report in the same second does not overwrite), the
//      listing of recent mod files only, nothing but relative paths in the file.
// Usage: utpp_diag_report_test <scratch folder>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/CommonUtil.hpp"
#include "../src/DiagReport.hpp"
#include "../src/DiagReportRuntime.hpp"
#include "../src/PathScrub.hpp"

namespace fs = std::filesystem;
using namespace utpp::diagreport;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
bool Has(const std::string& aText, const std::string& aNeedle) { return aText.find(aNeedle) != std::string::npos; }
bool HasAbsolutePath(const std::string& aText)
{
    for (size_t i = 0; i + 2 < aText.size(); ++i)
        if (std::isalpha(static_cast<unsigned char>(aText[i])) && aText[i + 1] == ':' && (aText[i + 2] == '\\' || aText[i + 2] == '/') &&
            (i == 0 || !std::isalnum(static_cast<unsigned char>(aText[i - 1]))))
            return true;
    return aText.find("\\\\") != std::string::npos;
}
FileEntry Entry(const std::string& aName, uint64_t aBytes, int64_t aAge)
{
    FileEntry e;
    e.name = aName;
    e.relPath = "red4ext\\logs\\" + aName;
    e.bytes = aBytes;
    e.ageSeconds = aAge;
    return e;
}
std::string ReadAll(const fs::path& aPath)
{
    std::ifstream in(aPath, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void Selection()
{
    Check(IsModFile("utpp-2026-11-10-12-00-00.log") && IsModFile("UTPP-2026.log") && IsModFile("utpp-pose-x.utpppose") && !IsModFile("other.log") && !IsModFile("xutpp-a.log") &&
              !IsModFile("utpp.log"),
          "A: utpp-* files only (any case)");
    const std::vector<FileEntry> all = {Entry("utpp-b.log", 10, 10),           Entry("utpp-a.log", 10, 1799),        Entry("utpp-edge.log", 10, 1800),
                                        Entry("utpp-old.log", 10, 1801),       Entry("utpp-skew.log", 10, -50),      Entry("notes.txt", 10, 5),
                                        Entry("utpp-report-x.txt", 10, 3),     Entry("UTPP-c.log", 10, 20)};
    const auto picked = SelectRecentFiles(all, kRecentSeconds, "utpp-report-x.txt");
    std::string names;
    for (const auto& f : picked)
        names += f.name + " ";
    Check(names == "utpp-a.log utpp-b.log UTPP-c.log utpp-edge.log utpp-skew.log ", "A: last 30 minutes (1800 s inclusive, a future timestamp counts), others and the report itself out, by name: " + names);
    Check(SelectRecentFiles({}, kRecentSeconds, "").empty(), "A: an empty folder lists nothing");
}

void Composition()
{
    Info info;
    info.modVersion = "0.1.0";
    info.gameVersion = "Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)";
    info.date = "2026-11-11 04:20:11";
    info.body = "settings: shoulder=0\nlast action: reload";
    const std::vector<FileEntry> files = {Entry("utpp-2026.log", 524288 * 3, 5), Entry("utpp-pose-a.utpppose", 8 * 1024 * 1024 + 300000, 40), Entry("utpp-tiny.txt", 700, 1)};
    const std::string t = Compose(info, "red4ext\\logs\\utpp-report-2026-11-11-04-20-11.txt", files);
    Check(t.rfind("UTPP diagnostic report\nmod: UTPP 0.1.0\ngame: Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)\ndate: 2026-11-11 04:20:11\n\n", 0) == 0, "B: header lines (mod and version, game version, date)");
    Check(Has(t, "settings: shoulder=0\nlast action: reload\n"), "B: the script's text is in the report");
    const std::string sentence = "Zip the files listed below and send them to the mod author.";
    Check(t.find(sentence) != std::string::npos && t.find(sentence) == t.rfind(sentence), "B: the instruction sentence appears once");
    const size_t at = t.find(sentence);
    Check(at != std::string::npos && t.substr(at).find("red4ext\\logs\\utpp-report-2026-11-11-04-20-11.txt  (this file)\n") != std::string::npos, "B: the report itself is listed after the sentence as (this file)");
    Check(Has(t, "red4ext\\logs\\utpp-2026.log  (1.5 MB)\n") && Has(t, "red4ext\\logs\\utpp-pose-a.utpppose  (8.3 MB)\n") && Has(t, "red4ext\\logs\\utpp-tiny.txt  (700 bytes)\n"), "B: files with relative paths and sizes");
    Check(!HasAbsolutePath(t), "B: no absolute path in the report");
    Info unknown;
    unknown.modVersion = "0.1.0";
    Check(Has(Compose(unknown, "r", {}), "game: unknown\n"), "B: an unknown game version is written as unknown");

    Info dirty = info;
    dirty.body = "path C:\\Users\\bob\\AppData\\x.txt and D:/Games/Cyberpunk 2077/red4ext/f.log; share \\\\server\\share\\f.txt; rel red4ext\\logs\\a.log; \"E:\\q\\r\" end; c:x not a path";
    const std::string r = RedactAbsolutePaths(dirty.body);
    Check(r == "path <path removed>; share <path removed>; rel red4ext\\logs\\a.log; \"<path removed>\" end; c:x not a path",
          "B: drive and network paths in the script's text are removed, relative paths stay: " + r);
    Check(!HasAbsolutePath(Compose(dirty, "r", {})), "B: Compose removes them");
    Info big = info;
    big.body = std::string(kMaxBodyBytes + 5000, 'x');
    const std::string bt = Compose(big, "r", {});
    Check(bt.size() < kMaxBodyBytes + 1000 && Has(bt, "[text cut]"), "B: an oversized text is cut");
}

void PathHelpers()
{
    using namespace utpp::util;
    const std::wstring root = L"E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077";
    Check(GameRootOfLogFolder(L"E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077\\red4ext\\logs\\") == root && GameRootOfLogFolder(L"E:/Steam Library/steamapps/common/Cyberpunk 2077/red4ext/logs") == root,
          "C: game folder = two levels above red4ext\\logs (trailing and forward slashes accepted)");
    Check(GameRootOfLogFolder(L"logs").empty() && GameRootOfLogFolder(L"").empty(), "C: a folder without two parents has no game root");
    Check(RelativeForLog(root + L"\\red4ext\\logs\\utpp-a.csv", root) == "red4ext\\logs\\utpp-a.csv", "C: a path inside the game folder is shown relative");
    Check(RelativeForLog(L"e:/steam library/STEAMAPPS/common/cyberpunk 2077/red4ext/logs/x.log", root) == "red4ext\\logs\\x.log", "C: case-insensitive, forward slashes");
    Check(RelativeForLog(L"C:\\Users\\bob\\AppData\\Local\\Temp\\utpp-b.csv", root) == "utpp-b.csv", "C: a path outside the game folder shows the file name only");
    Check(RelativeForLog(L"E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077 Extra\\x.log", root) == "x.log", "C: a sibling folder with the same prefix is not inside the game folder");
    Check(RelativeForLog(L"D:\\x\\y.log", L"") == "y.log" && RelativeForLog(L"y.log", root) == "y.log", "C: without a root, or with a bare name, only the file name");
    const std::string u8 = RelativeForLog(root + L"\\red4ext\\logs\\utpp-\u017c\u00f3\u0142w.log", root);
    Check(u8 == "red4ext\\logs\\utpp-\xC5\xBC\xC3\xB3\xC5\x82w.log", "C: non-ASCII names are UTF-8");
    Check(RelativeForLog(L"C:\\Users\\bob\\x.log", L"C:\\Users\\bob") == "x.log", "C: a file directly in the root shows its name");
}

void Scrub()
{
    using namespace utpp::pathscrub;
    SetGameRoot("E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077\\");
    Check(FormatLine("UTPP MM feed dump on: %s", "E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077\\red4ext\\logs\\utpp-feed-x.txt") == "UTPP MM feed dump on: red4ext\\logs\\utpp-feed-x.txt",
          "E: a path in the game folder is shown relative in the log line");
    Check(FormatLine("db %s ok=%d", "e:/steam library/steamapps/common/cyberpunk 2077/red4ext/plugins/UTPP/db/a.utppmm", 1) == "db red4ext/plugins/UTPP/db/a.utppmm ok=1",
          "E: case-insensitive, forward slashes kept after the root");
    Check(FormatLine("loaded %s;", "C:\\Users\\bob\\AppData\\Local\\Temp\\y.utppmm") == "loaded <path removed>;", "E: any other drive path is removed");
    Check(FormatLine("share %s, x", "\\\\nas\\mods\\y.txt") == "share <path removed>, x", "E: a network path is removed");
    Check(FormatLine("E:\\Steam Library\\steamapps\\common\\Cyberpunk 2077 Extra\\x.log failed") == "<path removed>", "E: a sibling folder with the same prefix is not the game folder: redacted");
    Check(FormatLine("t=12:30:15 url=https://example.org/a a:b %d%%", 7) == "t=12:30:15 url=https://example.org/a a:b 7%", "E: lines without paths are unchanged (times, URLs, percent signs)");
    Check(FormatLine("100%%") == "100%" && FormatLine("plain") == "plain", "E: formatting without arguments");
    const std::string longText(5000, 'x');
    Check(FormatLine("%s!", longText.c_str()) == longText + "!", "E: a long line is not cut");
    SetGameRoot("");
    Check(FormatLine("%s", "E:\\Steam Library\\x\\a.log") == "<path removed>", "E: without a registered root every drive path is removed");
}

void Writer(const fs::path& aScratch)
{
    std::error_code ec;
    fs::remove_all(aScratch, ec);
    const fs::path game = aScratch / "Cyberpunk 2077";
    const fs::path logs = game / "red4ext" / "logs";
    fs::create_directories(logs);
    const auto write = [&](const std::string& aName, const std::string& aText, int aAgeMinutes) {
        std::ofstream(logs / aName, std::ios::binary) << aText;
        fs::last_write_time(logs / aName, fs::file_time_type::clock::now() - std::chrono::minutes(aAgeMinutes));
    };
    write("utpp-2026-11-11-04-00-00.log", std::string(2048, 'x'), 1);
    write("utpp-pose-2026-11-11-04-10-00.utpppose", std::string(3 * 1024 * 1024, 'p'), 10);
    write("utpp-bodytrace-old.csv", "old", 120);
    write("RED4ext-notes.txt", "not ours", 1);
    const std::string rel = utpp::diagreportruntime::WriteReport(logs.wstring(), "0.1.0", "Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)", "body line from the script\nsecond line");
    Check(rel.rfind("red4ext\\logs\\utpp-report-", 0) == 0 && rel.size() > 4 && rel.substr(rel.size() - 4) == ".txt" && !HasAbsolutePath(rel), "D: returns the report's path relative to the game folder: " + rel);
    const std::string text = ReadAll(game / rel);
    Check(Has(text, "UTPP diagnostic report\nmod: UTPP 0.1.0\ngame: Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)\ndate: 20"), "D: header with mod, game version and date");
    Check(Has(text, "body line from the script\nsecond line\n"), "D: the script's text");
    Check(Has(text, "red4ext\\logs\\utpp-2026-11-11-04-00-00.log  (2 KB)") && Has(text, "red4ext\\logs\\utpp-pose-2026-11-11-04-10-00.utpppose  (3.0 MB)"), "D: recent mod files are listed with relative path and size");
    Check(!Has(text, "utpp-bodytrace-old.csv") && !Has(text, "RED4ext-notes.txt"), "D: a file from two hours ago and a file that is not the mod's are not listed");
    Check(Has(text, rel + "  (this file)") && !HasAbsolutePath(text) && Has(text, "Zip the files listed below and send them to the mod author."), "D: the report lists itself, the instruction is there, no absolute path anywhere");
    const std::string rel2 = utpp::diagreportruntime::WriteReport(logs.wstring(), "0.1.0", "", "again");
    Check(!rel2.empty() && rel2 != rel && ReadAll(game / rel) == text, "D: a second report never overwrites the first (" + rel2 + ")");
    Check(Has(ReadAll(game / rel2), "game: unknown\n") && Has(ReadAll(game / rel2), rel + "  ("), "D: the second report lists the first as a recent file");
    Check(utpp::diagreportruntime::WriteReport(L"", "0.1.0", "", "x").empty() && utpp::diagreportruntime::WriteReport((aScratch / "missing" / "red4ext" / "logs").wstring(), "0.1.0", "", "x").empty(),
          "D: an empty or missing log folder gives an empty result, no crash");
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_diag_report_test <scratch folder>\n");
        return 2;
    }
    Selection();
    Composition();
    PathHelpers();
    Scrub();
    Writer(fs::absolute(argv[1]));
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

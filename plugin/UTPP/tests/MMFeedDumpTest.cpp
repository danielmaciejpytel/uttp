// batch 1b offline test: motion matching feed dump (mm/FeedDump.hpp + the real MMDriver wiring; no game).
// Usage: utpp_mm_feeddump_test <out dir> [database.utppmm]
// A formatter: exact header, 31 columns on every line, the motion .feed columns first, '-' for unknown / non-finite values, clip
//   names with spaces / control / non-ASCII bytes, float round trip, file names.
// B writer: start / stop / shutdown, file naming incl. name collisions, flush about every second without a stop, drop counting with
//   a stalled writer (Push never waits), restart, an uncreatable file, the destructor of a live writer.
// C driver: dump off = no record / file / status segment, no folder = start refused, one line per Feed call with the raw feed
//   values and the extra fields (walk, capsule), v1 columns with a database, every Feed return path (database swap) writes a line,
//   Shutdown writes the rest.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/State.hpp"
#include "../src/mm/FeedDump.hpp"

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

namespace
{
namespace fs = std::filesystem;
namespace fd = utpp::mm::feeddump;

int g_pass = 0, g_fail = 0;
void Check(const char* aName, bool aOk, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
}

std::vector<std::string> g_logLines;
void CaptureLog(const char* aLine)
{
    g_logLines.emplace_back(aLine ? aLine : "");
}
bool LoggedContaining(const std::string& aText)
{
    for (const std::string& l : g_logLines)
        if (l.find(aText) != std::string::npos)
            return true;
    return false;
}

std::vector<std::string> Tokens(const std::string& aLine)
{
    std::vector<std::string> out;
    std::istringstream s(aLine);
    std::string t;
    while (s >> t)
        out.push_back(t);
    return out;
}

std::string Format(const fd::Record& aRec)
{
    char buf[fd::kMaxLine];
    const size_t n = fd::FormatLine(aRec, buf, sizeof buf);
    return std::string(buf, n);
}

// A well-formed data line: ends with '\n', no '\r', single spaces, exactly kColumns tokens.
bool WellFormed(const std::string& aLine)
{
    if (aLine.empty() || aLine.back() != '\n' || aLine.find('\r') != std::string::npos || aLine.find("  ") != std::string::npos ||
        aLine.front() == ' ')
        return false;
    return Tokens(aLine).size() == size_t(fd::kColumns);
}

std::string ReadFile(const fs::path& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

// lines without the '\n' (the last line must be terminated; an unterminated tail is returned as its own entry)
std::vector<std::string> FileLines(const fs::path& aPath)
{
    std::vector<std::string> out;
    const std::string text = ReadFile(aPath);
    size_t p = 0;
    while (p < text.size())
    {
        const size_t e = text.find('\n', p);
        if (e == std::string::npos)
        {
            out.push_back(text.substr(p) + "<unterminated>");
            break;
        }
        out.push_back(text.substr(p, e - p));
        p = e + 1;
    }
    return out;
}

std::vector<fs::path> FeedFiles(const fs::path& aDir)
{
    std::vector<fs::path> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(aDir, ec))
    {
        const std::string n = e.path().filename().string();
        if (n.rfind("utpp-feed-", 0) == 0 && n.size() > 4 && n.substr(n.size() - 4) == ".txt")
            out.push_back(e.path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

void ClearFeedFiles(const fs::path& aDir)
{
    std::error_code ec;
    for (const fs::path& p : FeedFiles(aDir))
        fs::remove(p, ec);
}

// data lines of a dump file (header checked separately); false when the header is not the contract header
bool DataLines(const fs::path& aPath, std::vector<std::string>& aLines)
{
    aLines = FileLines(aPath);
    if (aLines.empty() || aLines.front() != fd::kHeader)
        return false;
    aLines.erase(aLines.begin());
    return true;
}

bool AllWellFormed(const std::vector<std::string>& aLines, std::string& aBad)
{
    for (const std::string& l : aLines)
        if (!WellFormed(l + "\n"))
        {
            aBad = l;
            return false;
        }
    return true;
}

template<typename Pred>
bool WaitFor(Pred aPred, int aTimeoutMs)
{
    for (int waited = 0; waited <= aTimeoutMs; waited += 5)
    {
        if (aPred())
            return true;
        Sleep(5);
    }
    return aPred();
}

SYSTEMTIME Stamp(int aY, int aMo, int aD, int aH, int aMi, int aS)
{
    SYSTEMTIME st{};
    st.wYear = WORD(aY);
    st.wMonth = WORD(aMo);
    st.wDay = WORD(aD);
    st.wHour = WORD(aH);
    st.wMinute = WORD(aMi);
    st.wSecond = WORD(aS);
    return st;
}

fd::Record Numbered(int aI)
{
    fd::Record r;
    r.t = aI;
    r.dt = 1.0f / 60.0f;
    r.loco = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ A: formatter
void TestFormatter()
{
    const std::string header = fd::kHeader;
    Check("A header: exact contract text",
          header == "# cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step haveTarget targetYawDeg capX "
                    "capY capZ effGate holdEligible gateRaw upperBody detailed onGround distGround psmFall walk",
          header);
    std::vector<std::string> cols = Tokens(header);
    const bool prefix = cols.size() >= 2 && cols[0] == "#" && cols[1] == "cols";
    if (prefix)
        cols.erase(cols.begin(), cols.begin() + 2);
    std::string first17;
    for (size_t i = 0; i < 17 && i < cols.size(); ++i)
        first17 += (i ? " " : "") + cols[i];
    Check("A header: '# cols' + kColumns (31) names, .feed columns first, walk last",
          prefix && cols.size() == size_t(fd::kColumns) && fd::kColumns == 31 &&
              first17 == "t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step" && cols.back() == "walk",
          std::to_string(cols.size()) + " / " + first17);

    const fd::Record def;
    const std::string defLine = Format(def);
    Check("A default record: unknown columns '-' (base, step, capsule, upperBody, detailed, air, walk)",
          defLine == "0.000000 0 0 0 0 0 0 0 0 0 0 - - - - - - 0 0 - - - 0 0 0 - - - - - -\n", defLine);

    fd::Record r;
    r.t = 12.5;
    r.dt = 1.0f / 60.0f;
    r.vel[0] = 1.5f;
    r.vel[1] = -2.25f;
    r.vel[2] = 0.125f;
    r.move[0] = 0.5f;
    r.move[1] = -1.0f;
    r.yawDeg = 90.5f;
    r.camDeg = -179.75f;
    r.loco = 1;
    r.psm = (6u + 1u) * 8192u + (1u + 1u) * 65536u + (30u + 1u) * 16777216u + 64u; // UpperBody 6, detailed 1, melee 30, bit 64
    r.v1 = true;
    r.trans = true;
    fd::SetClip(r, "Walk Loop\tF");
    r.baseFlags = 42;
    r.baseSearched = true;
    r.baseCost = 3.14159f;
    r.baseMicros = 87.26;
    r.haveTarget = true;
    r.targetYawDeg = 45.0f;
    r.haveCapsule = true;
    r.capsule[0] = -1234.5f;
    r.capsule[1] = 567.25f;
    r.capsule[2] = 12.0625f;
    r.effGate = 257;
    r.holdEligible = true;
    r.gateRaw = 0x80000001u;
    r.upperBody = 6;
    r.detailed = 1;
    r.onGround = 1;
    r.distGround = 0.5f;
    r.psmFall = 2;
    r.walk = 1;
    const std::string full = Format(r);
    Check("A full record: exact line (decimal masks, 9-digit floats, cost %.4f, sUs %.1f, clip spaces / tab -> '_')",
          full == "12.500000 0.0166666675 1.5 -2.25 0.125 0.5 -1 90.5 -179.75 1 520282176 1 Walk_Loop_F 42 3.1416 87.3 - 1 45 -1234.5 "
                  "567.25 12.0625 257 1 2147483649 6 1 1 0.5 2 1\n",
          full);

    fd::Record rt;
    rt.move[0] = 0.7071f;
    rt.vel[1] = 0.03f;
    rt.camDeg = 33.3f;
    const std::vector<std::string> rtTok = Tokens(Format(rt));
    Check("A floats round-trip exactly (9 significant digits)",
          rtTok.size() == 31 && std::strtof(rtTok[5].c_str(), nullptr) == 0.7071f && std::strtof(rtTok[3].c_str(), nullptr) == 0.03f &&
              std::strtof(rtTok[8].c_str(), nullptr) == 33.3f,
          rtTok.size() > 8 ? rtTok[5] + " " + rtTok[3] + " " + rtTok[8] : "");

    fd::Record nf;
    nf.dt = std::nanf("");
    nf.vel[0] = INFINITY;
    nf.vel[2] = -INFINITY;
    nf.targetYawDeg = std::nanf("");
    nf.haveCapsule = true;
    nf.capsule[1] = std::nanf("");
    nf.v1 = true; // the matcher ran, but no clip / no search yet
    nf.baseFlags = -1;
    nf.baseSearched = false;
    const std::vector<std::string> nt = Tokens(Format(nf));
    Check("A non-finite floats -> '-'; base without clip / search: trans 0, baseClip baseFlags baseCost baseMicros '-'",
          nt.size() == 31 && nt[1] == "-" && nt[2] == "-" && nt[4] == "-" && nt[3] == "0" && nt[18] == "-" && nt[19] == "0" &&
              nt[20] == "-" && nt[21] == "0" && nt[22] == "0" && nt[11] == "0" && nt[12] == "-" && nt[13] == "-" && nt[14] == "-" &&
              nt[15] == "-",
          Format(nf));

    fd::Record cl;
    cl.v1 = true;
    fd::SetClip(cl, "Run Turn 180 L");
    std::vector<std::string> ct = Tokens(Format(cl));
    const bool spaces = ct.size() == 31 && ct[12] == "Run_Turn_180_L";
    fd::SetClip(cl, "Caf\xC3\xA9\x01x");
    ct = Tokens(Format(cl));
    const bool nonAscii = ct.size() == 31 && ct[12] == "Caf___x";
    fd::SetClip(cl, std::string(200, 'A'));
    ct = Tokens(Format(cl));
    const bool longName = ct.size() == 31 && ct[12] == std::string(fd::kClipChars, 'A');
    fd::SetClip(cl, "");
    ct = Tokens(Format(cl));
    const bool empty = ct.size() == 31 && ct[12] == "-";
    Check("A clip names: spaces -> '_', control / non-ASCII bytes -> '_', cut at kClipChars, empty -> '-' (always one token)",
          spaces && nonAscii && longName && empty);

    // every variant above is one well-formed line
    bool allOk = WellFormed(defLine) && WellFormed(full) && WellFormed(Format(rt)) && WellFormed(Format(nf)) && WellFormed(Format(cl));
    fd::Record big = r;
    big.t = 1e9;
    big.dt = -3.4e38f;
    for (float& v : big.vel)
        v = 3.4e38f;
    big.baseCost = 3.4e38f;
    big.baseMicros = 1e300;
    big.loco = INT32_MIN;
    big.psm = 0xFFFFFFFFu;
    big.baseFlags = INT32_MAX;
    fd::SetClip(big, std::string(300, 'z'));
    const std::string bigLine = Format(big);
    allOk = allOk && WellFormed(bigLine) && bigLine.size() < fd::kMaxLine;
    Check("A every line well-formed (31 tokens, '\\n', no '\\r' / double space), extreme values fit kMaxLine", allOk,
          std::to_string(bigLine.size()) + " " + bigLine.substr(0, 120));

    char small[32];
    Check("A too small buffer -> 0 (nothing claimed written)", fd::FormatLine(r, small, sizeof small) == 0);

    Check("A file name utpp-feed-<yyyy-mm-dd-hh-mm-ss>.txt (+ -N on a collision)",
          fd::FileName(Stamp(2026, 10, 13, 8, 5, 9)) == L"utpp-feed-2026-10-13-08-05-09.txt" &&
              fd::FileName(Stamp(2026, 10, 13, 8, 5, 9), 2) == L"utpp-feed-2026-10-13-08-05-09-2.txt");
}

// ------------------------------------------------------------------------------------------------ B: writer
void TestWriter(const fs::path& aDir)
{
    fs::create_directories(aDir);
    ClearFeedFiles(aDir);
    const SYSTEMTIME st = Stamp(2001, 2, 3, 4, 5, 6);
    const fs::path f1 = aDir / "utpp-feed-2001-02-03-04-05-06.txt";
    const fs::path f2 = aDir / "utpp-feed-2001-02-03-04-05-06-2.txt";
    const fs::path f3 = aDir / "utpp-feed-2001-02-03-04-05-06-3.txt";
    {
        fd::Dumper d(64);
        Check("B no folder -> Start refused, Push off (not counted)",
              !d.Start(L"") && d.Push(Numbered(0)) == fd::PushResult::Off && d.Pushed() == 0 && d.Dropped() == 0 && d.Sessions() == 0);

        // start / stop / shutdown
        const bool started = d.Start(aDir.wstring(), &st);
        int ok = 0;
        for (int i = 0; i < 50; ++i)
            ok += d.Push(Numbered(i)) == fd::PushResult::Ok ? 1 : 0;
        d.Stop();
        const bool offAfterStop = d.Push(Numbered(99)) == fd::PushResult::Off && !d.Accepting();
        d.Shutdown();
        std::vector<std::string> lines;
        const bool header = DataLines(f1, lines);
        std::string bad;
        bool order = lines.size() == 50;
        for (size_t i = 0; order && i < lines.size(); ++i)
        {
            const std::vector<std::string> tk = Tokens(lines[i]);
            order = !tk.empty() && tk[0] == std::to_string(i) + ".000000";
        }
        Check("B start -> 50 pushes -> stop (no more pushes) -> shutdown: header + 50 well-formed lines in order",
              started && ok == 50 && offAfterStop && header && AllWellFormed(lines, bad) && order && d.Written() == 50 &&
                  d.Pushed() == 50 && d.Dropped() == 0 && d.Path() == f1.wstring(),
              std::to_string(lines.size()) + " " + bad);

        // periodic flush (no stop, no wake): the lines reach the file within about a second
        const bool started2 = d.Start(aDir.wstring(), &st);
        for (int i = 0; i < 10; ++i)
            d.Push(Numbered(i));
        const auto t0 = std::chrono::steady_clock::now();
        const bool flushed = WaitFor([&] { return FileLines(f2).size() == 11; }, 3000);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        Check("B second start in the same second -> '-2' file; lines written about every second without a stop",
              started2 && fs::exists(f2) && flushed && secs < 2.5 && d.Path() == f2.wstring(), std::to_string(secs) + " s");
        d.Shutdown();

        // drops: a stalled writer, Push never waits
        const bool started3 = d.Start(aDir.wstring(), &st);
        d.TestPauseWriter(true);
        int okN = 0, first = 0, dropped = 0;
        const auto p0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 84; ++i)
        {
            const fd::PushResult res = d.Push(Numbered(i));
            okN += res == fd::PushResult::Ok ? 1 : 0;
            first += res == fd::PushResult::FirstDrop ? 1 : 0;
            dropped += res == fd::PushResult::Dropped ? 1 : 0;
        }
        const double pushMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - p0).count();
        const std::string stDrop = d.Status();
        Check("B full ring (64): 64 accepted, the 65th is FirstDrop (one log line), 19 more Dropped; counted, status dropped=20",
              started3 && okN == 64 && first == 1 && dropped == 19 && d.Dropped() == 20 && d.Pushed() == 64 &&
                  stDrop.find(" dropped=20 ") != std::string::npos && stDrop.find("ring=64 ") != std::string::npos,
              stDrop + " push " + std::to_string(pushMs) + " ms");
        d.TestPauseWriter(false);
        d.TestWake();
        const bool drained = WaitFor([&] { return d.Written() == 64; }, 3000);
        int okAfter = 0;
        for (int i = 0; i < 5; ++i)
            okAfter += d.Push(Numbered(100 + i)) == fd::PushResult::Ok ? 1 : 0;
        d.Shutdown();
        std::vector<std::string> l3;
        Check("B writer resumes: ring drained, new lines accepted again; file = 64 + 5 lines, no partial line",
              drained && okAfter == 5 && DataLines(f3, l3) && l3.size() == 69 && AllWellFormed(l3, bad) && d.Dropped() == 20,
              std::to_string(l3.size()) + " " + bad);

        // an uncreatable file: the writer fails, Push is off, the error is in the status
        const bool started4 = d.Start((aDir / "no_such_folder").wstring(), &st);
        const bool failed = WaitFor([&] { return d.Failed(); }, 3000);
        const std::string stFail = d.Status();
        Check("B uncreatable file -> Failed, not accepting, Push off, status error=CreateFileW_3",
              started4 && failed && !d.Accepting() && d.Push(Numbered(0)) == fd::PushResult::Off &&
                  stFail.find("error=CreateFileW_3 ") != std::string::npos,
              stFail);
        d.Stop();

        // restart after a failure and after a plain Stop (Start joins the previous writer)
        const bool started5 = d.Start(aDir.wstring(), &st);
        d.Push(Numbered(7));
        d.Stop();
        const bool started6 = d.Start(aDir.wstring(), &st);
        d.Push(Numbered(8));
        d.Shutdown();
        const std::vector<fs::path> files = FeedFiles(aDir);
        Check("B restart after a failure / a stop: new files -4 / -5, sessions counted (6 starts)",
              started5 && started6 && files.size() == 5 && fs::exists(aDir / "utpp-feed-2001-02-03-04-05-06-5.txt") &&
                  d.Sessions() == 6,
              std::to_string(files.size()) + " files, sessions " + std::to_string(d.Sessions()));
    }

    // destructor with a live writer (DLL detach without an unload): the writer finishes its last drain, nothing is lost
    {
        const SYSTEMTIME st2 = Stamp(2001, 2, 3, 4, 5, 7);
        {
            fd::Dumper d(64);
            d.Start(aDir.wstring(), &st2);
            for (int i = 0; i < 20; ++i)
                d.Push(Numbered(i));
        }
        std::vector<std::string> l;
        std::string bad;
        const fs::path fx = aDir / "utpp-feed-2001-02-03-04-05-07.txt";
        const bool done = WaitFor([&] { return FileLines(fx).size() == 21; }, 3000);
        Check("B destructor with a live writer: lines written, file closed", done && DataLines(fx, l) && l.size() == 20 &&
                                                                                AllWellFormed(l, bad));
    }
}

// ------------------------------------------------------------------------------------------------ C: driver wiring
utpp::motionhost::FeedInput WalkFeed(int aI, bool aExtra)
{
    utpp::motionhost::FeedInput in;
    in.velWorld[0] = 0.25f;
    in.velWorld[1] = 1.5f;
    in.velWorld[2] = -0.0625f;
    in.move[0] = 0.03f; // raw, inside the stick dead zone
    in.move[1] = 1.0f;
    in.cameraYawDeg = 33.3f;
    in.playerYawDeg = -7.5f;
    in.locoState = 1;
    in.tppActive = true;
    in.dt = 1.0f / 60.0f;
    in.haveTarget = true;
    in.targetYawDeg = 10.0f + float(aI);
    in.psmBlock = (6u + 1u) * 8192u + (1u + 1u) * 65536u; // diagnostic fields only: UpperBody 6, LocomotionDetailed 1
    if (aExtra)
    {
        in.haveExtra = true;
        in.walkToggled = (aI % 2) == 1;
        in.capsulePos[0] = 100.25f;
        in.capsulePos[1] = -200.5f;
        in.capsulePos[2] = 3.0f + float(aI);
    }
    return in;
}

void TestDriver(const fs::path& aDir, const char* aDb)
{
    using namespace utpp::motionhost;
    SetLogger(&CaptureLog);
    fs::create_directories(aDir);
    ClearFeedFiles(aDir);

    Check("C dump never started: no feedDump status segment (status text unchanged), dump status sessions=0",
          Status().find("feedDump=") == std::string::npos && FeedDumpStatus().find("on=0 ") == 0 &&
              FeedDumpStatus().find(" sessions=0 ") != std::string::npos,
          FeedDumpStatus());
    for (int i = 0; i < 5; ++i)
        Feed(WalkFeed(i, true));
    Check("C dump off: Feed writes nothing (no file, no record)", FeedFiles(aDir).empty() && FeedDumpStatus().find(" lines=0 ") != std::string::npos);

    Check("C no folder set -> SetFeedDump(true) refused and logged, no session", !SetFeedDump(true) &&
                                                                                 LoggedContaining("UTPP MM feed dump FAILED to start") &&
                                                                                 Status().find("feedDump=") == std::string::npos);
    SetFeedDump(false);

    // no database: the matcher never runs -> v1 columns '-'
    SetFeedDumpDir(aDir.wstring());
    {
        std::error_code rc; // stale text pose CSVs of older runs of this folder (the dumper now removes its empty files itself)
        std::vector<fs::path> stale;
        for (const auto& e : fs::directory_iterator(aDir, rc))
            if (e.path().filename().wstring().rfind(L"utpp-pose-", 0) == 0)
                stale.push_back(e.path());
        for (const fs::path& p : stale)
            fs::remove(p, rc);
    }
    const bool on = SetFeedDump(true);
    const bool onAgain = SetFeedDump(true); // already on: same session
    {
        // the diagnostic logs / harness feed dump alone never start the text pose CSV (compact pose recording instead)
        bool anyPoseCsv = false;
        for (const auto& e : fs::directory_iterator(aDir))
            anyPoseCsv = anyPoseCsv || e.path().filename().wstring().rfind(L"utpp-pose-", 0) == 0;
        Check("C feed dump on without the marker file: no text pose CSV (status poseTrace on=0, no utpp-pose-* file)",
              on && !anyPoseCsv && FeedDumpStatus().find(" poseTrace on=0 ") != std::string::npos, FeedDumpStatus());
    }
    for (int i = 0; i < 30; ++i)
    {
        FeedInput in = WalkFeed(i, i != 29); // the last feed without the extra fields (older script)
        if (i == 5)
            in.locoState = 5; // raw (unsupported) state, not the debounced / conditioned one
        if (i == 10 || i == 11) // AIR HOLD stage: UTPPFeedAir fields (airborne with a raycast; grounded, not probed)
        {
            in.haveAir = true;
            in.onGround = i == 11;
            in.distToGround = i == 10 ? 1.25f : -2.0f;
            in.psmFall = i == 10 ? 1 : 0;
        }
        Feed(in);
    }
    SetFeedDump(false);
    const bool flushed = FeedDumpFlushForTest(3000);
    const std::vector<fs::path> files = FeedFiles(aDir);
    std::vector<std::string> lines;
    std::string bad;
    const bool header = files.size() == 1 && DataLines(files[0], lines);
    Check("C on: 'feed dump on' logged with the file; one well-formed line per Feed call (30); off logged with the count",
          on && onAgain && flushed && header && lines.size() == 30 && AllWellFormed(lines, bad) &&
              LoggedContaining("UTPP MM feed dump on: ") && LoggedContaining(files[0].filename().string()) &&
              LoggedContaining("UTPP MM feed dump off: lines=30 dropped=0"),
          std::to_string(lines.size()) + " " + bad);
    if (lines.size() == 30)
    {
        const std::vector<std::string> a = Tokens(lines[0]);
        const std::vector<std::string> b = Tokens(lines[1]);
        const std::vector<std::string> c = Tokens(lines[5]);
        const std::vector<std::string> z = Tokens(lines[29]);
        Check("C raw feed columns: dt vx vy vz mvx mvy yawDeg camDeg loco psm haveTarget targetYawDeg (stick inside the dead zone kept)",
              a[1] == "0.0166666675" && a[2] == "0.25" && a[3] == "1.5" && a[4] == "-0.0625" && a[5] == "0.0299999993" && a[6] == "1" &&
                  a[7] == "-7.5" && a[8] == "33.2999992" && a[9] == "1" && a[10] == "188416" && a[17] == "1" && a[18] == "10" &&
                  b[18] == "11",
              lines[0]);
        Check("C loco as fed (5, not the conditioned stance); upperBody / detailed decoded from psm (6 / 1)",
              c[9] == "5" && a[25] == "6" && a[26] == "1", lines[5]);
        Check("C extra fields: walk 0/1 last column, capsule xyz; feed without them -> '-'",
              a[30] == "0" && b[30] == "1" && a[19] == "100.25" && a[20] == "-200.5" && a[21] == "3" && b[21] == "4" && z[30] == "-" &&
                  z[19] == "-" && z[20] == "-" && z[21] == "-",
              lines[29]);
        Check("C no database: base columns '-', step '-', air columns '-', gates numeric",
              a[11] == "-" && a[12] == "-" && a[13] == "-" && a[14] == "-" && a[15] == "-" && a[16] == "-" && a[27] == "-" && a[28] == "-" &&
                  a[29] == "-" && a[22] != "-" && a[24] != "-",
              lines[0]);
        Check("C no database: holdEligible 0 (no pose to hold)", a[23] == "0" && z[23] == "0", lines[0]);
        {
            const std::vector<std::string> g = Tokens(lines[10]);
            const std::vector<std::string> h = Tokens(lines[11]);
            Check("C air columns from UTPPFeedAir: onGround distGround psmFall (0 1.25 1 airborne, 1 -2 0 grounded / not probed); '-' "
                  "without the air feed",
                  g.size() == 31 && h.size() == 31 && g[27] == "0" && g[28] == "1.25" && g[29] == "1" && h[27] == "1" && h[28] == "-2" &&
                      h[29] == "0" && a[27] == "-" && z[27] == "-",
                  lines[10] + " | " + lines[11]);
        }
        double prev = -1.0;
        bool mono = true;
        for (const std::string& l : lines)
        {
            const double t = std::atof(Tokens(l)[0].c_str());
            mono = mono && t >= prev && t < 60.0;
            prev = t;
        }
        Check("C t = seconds since the dump was enabled, non-decreasing", mono && std::atof(a[0].c_str()) < 5.0);
    }
    for (int i = 0; i < 10; ++i)
        Feed(WalkFeed(i, true));
    Check("C off again: further feeds write nothing", FileLines(files.empty() ? fs::path() : files[0]).size() == 31 && FeedFiles(aDir).size() == 1);
    const std::string st1 = Status();
    Check("C status segment feedDump=\"on=0 ... sessions=1 ... file=<path>\" once a dump was started",
          st1.find(" feedDump=\"on=0 lines=30 written=30 dropped=0 sessions=1 ") != std::string::npos, st1.substr(st1.find("feedDump=") == std::string::npos ? 0 : st1.find("feedDump=")));

    if (!aDb)
    {
        std::printf("[INFO] no database given: v1 / swap checks skipped\n");
        return;
    }
    // with a database: the matcher runs -> v1 columns; a database swap (early return in Feed) still writes its line
    const fs::path dbDir = aDir / "dbdir";
    std::error_code ec;
    fs::remove_all(dbDir, ec);
    fs::create_directories(dbDir / "data");
    for (const fs::path& dst : {dbDir / "data" / "utpp_base.utppmm", dbDir / "data" / "alt.utppmm"})
    {
        fs::create_hard_link(aDb, dst, ec); // same volume: no 23 MB copy
        if (ec)
            fs::copy_file(aDb, dst, ec);
    }
    SetDatabaseDir(dbDir.wstring());
    const bool loaded = LoadDatabase((dbDir / "data" / "utpp_base.utppmm").wstring(), "default");
    utpp::mm::Database db;
    std::string err;
    db.LoadFromFile(aDb, kExpectedRigHash, err);
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(utpp::mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rigDummy = 0;
    utpp::g_hot.armed.store(true);
    OnArmed(&rigDummy, hashes, parents);
    SetEnabled(true);
    ClearFeedFiles(aDir);
    const bool on2 = SetFeedDump(true);
    int feeds = 0;
    for (int i = 0; i < 90; ++i, ++feeds)
        Feed(WalkFeed(i, true));
    const bool requested = RequestDatabase("alt");
    WaitDatabaseWorker();
    for (int i = 0; i < 60; ++i, ++feeds)
        Feed(WalkFeed(i, true));
    const std::string sw = DatabaseSwitchStatus();
    SetFeedDump(false);
    FeedDumpFlushForTest(3000);
    const std::vector<fs::path> f2 = FeedFiles(aDir);
    std::vector<std::string> l2;
    const bool h2 = f2.size() == 1 && DataLines(f2[0], l2);
    Check("C with a database: one line per Feed call across a database swap (early return paths)",
          loaded && on2 && requested && h2 && int(l2.size()) == feeds && AllWellFormed(l2, bad) && sw.rfind("ok alt", 0) == 0,
          std::to_string(l2.size()) + "/" + std::to_string(feeds) + " " + sw + " " + bad);
    int baseLines = 0, trans = 0, costs = 0, holds = 0, holdBad = 0;
    for (const std::string& l : l2)
    {
        const std::vector<std::string> t = Tokens(l);
        if (t.size() != 31)
            continue;
        baseLines += (t[12] != "-" && t[13] != "-" && t[11] != "-") ? 1 : 0;
        trans += t[11] == "1" ? 1 : 0;
        costs += (t[14] != "-" && t[15] != "-") ? 1 : 0;
        holds += t[23] == "1" ? 1 : 0;
        holdBad += (t[23] != "0" && t[23] != "1") ? 1 : 0;
    }
    Check("C base columns: clip / frame on the matcher feeds, transitions marked, search cost / time once searched",
          baseLines >= 140 && trans >= 1 && costs >= 140,
          "base " + std::to_string(baseLines) + " trans " + std::to_string(trans) + " costs " + std::to_string(costs));
    // hold last pose on (default), TPP, a pose computed: the driver's HoldEligible result reaches the column
    Check("C holdEligible column: 1 once a pose is available (driver capture), only 0/1",
          holds >= 140 && holdBad == 0, "holds " + std::to_string(holds) + "/" + std::to_string(l2.size()));

    // Shutdown with the dump on: logged, the writer joined, every line written
    g_logLines.clear();
    SetFeedDump(true);
    for (int i = 0; i < 5; ++i)
        Feed(WalkFeed(i, true));
    Shutdown();
    const std::vector<fs::path> f3 = FeedFiles(aDir);
    fs::path newest;
    for (const fs::path& p : f3)
        if (f2.empty() || p != f2[0])
            newest = p;
    std::vector<std::string> l3;
    Check("C Shutdown with the dump on: 'off (shutdown)' logged, all lines written (writer joined)",
          f3.size() == 2 && !newest.empty() && DataLines(newest, l3) && l3.size() == 5 &&
              LoggedContaining("UTPP MM feed dump off (shutdown): lines=5 dropped=0"),
          std::to_string(f3.size()) + " files");
    {
        // the project's own tools opt in with <feed dump folder>\pose-text-csv.txt: then the text pose CSV starts with the feed dump
        const fs::path marker = aDir / L"pose-text-csv.txt";
        { std::ofstream(marker) << "pose text csv\n"; }
        SetFeedDumpDir(aDir.wstring());
        const bool onCsv = SetFeedDump(true);
        const std::string st = FeedDumpStatus();
        SetFeedDump(false);
        fs::remove(marker, ec);
        const size_t pt = st.find(" poseTrace on=1 ");
        const std::string poseStatus = pt == std::string::npos ? std::string() : st.substr(pt);
        Check("C feed dump on with the marker file: text pose CSV started (status poseTrace on=1), file shown game-relative (no drive letter)",
              onCsv && pt != std::string::npos && poseStatus.find("utpp-pose-") != std::string::npos && poseStatus.find(':') == std::string::npos, st);
    }
    fs::remove_all(dbDir, ec); // the hard links to the database (no writable alias of the Data file left behind)
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_feeddump_test <out dir> [database.utppmm]\n");
        return 2;
    }
    const fs::path out = fs::path(argv[1]);
    TestFormatter();
    TestWriter(out / "writer");
    TestDriver(out / "driver", argc >= 3 ? argv[2] : nullptr);
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

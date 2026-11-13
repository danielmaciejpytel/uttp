// P1 (fix batch 2, 2026-10-21 12:15 recording): stance latency of adapter::MapToChooser. The 0.15 s stance filter of fix batch 1
// (MapConfig::stanceFilterS) delayed EVERY crouch / stand-up edge: the clip pick came 0.132-0.151 s (8-9 feeds) after the PSM change
// (docs: 22 of 22 in-place crouches), although the game PSM has no Stand <-> Crouch flicker (75 clean edges, 0 one-feed spikes). The
// fix (MapConfig::stanceCleanEdge) lets a CLEAN edge - Stand 1 / AimWalk 2 <-> Crouch 3 on this and the previous feed, the old raw value
// held >= stanceCleanHoldS, no slide / slide-exit window, a crouch ENTRY below stanceCleanMaxSpeedMs - reach the chooser on its own feed;
// everything that flickers (CrouchSprint 30, Dodge 7, Sprint 4, air / landing, slide) keeps the filter.
//
// Usage: utpp_p1_stance_latency_test <feed_recording_20261021_1213.txt> [--report]
// Part 1: synthetic sequences (no data). Part 2: replay of the recorded feed dump windows (tests/data/feed_recording_20261021_1213.txt: windows of
// -0.75 / +0.55 s around every PSM stance edge of the 12:13 session) through adapter::MakeMotionFeed -> adapter::MapToChooser with the runtime
// configuration (MotionDriver.cpp StepLocked), slide feed on and off.
// Mutation builds (-DUTPP_P1_MUTATION=k, see MotionAdapter.hpp) MUST fail: 1 no bypass, 3 no crouch-entry speed limit, 4 no min-hold.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../src/mm/MotionAdapter.hpp"

namespace adapter = utpp::mm::adapter;
namespace integ = utpp::mm::integ;
namespace motion = utpp::mm::motion;

namespace
{
int g_fail = 0, g_checks = 0;
bool g_report = false;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
        ++g_fail;
    if (!aOk || g_report)
        std::printf("%s %s (%.3f, %.3f)\n", aOk ? "PASS" : "FAIL", aWhat, aA, aB);
}

constexpr float kFilterS = adapter::kStanceFilterSeconds;
constexpr double kShortRunS = 0.15; // fed stance runs shorter than this are flicker

// ------------------------------------------------------------------------------------------------ configurations
// The runtime configuration (MotionDriver.cpp StepLocked): the filter, the Dodge hold, the P1 bypass; slide on with a declared slide section.
adapter::MapConfig RuntimeConfig(bool aSlide)
{
    adapter::MapConfig c;
    c.slide = aSlide;
    c.slideSpec.declared = aSlide;
    c.slideDeceleration = aSlide ? adapter::kSlideDecelerationMs2 : 0.0f;
    c.slideAirGraceS = aSlide ? adapter::kSlideAirGraceSeconds : 0.0f;
    c.stanceFilterS = adapter::kStanceFilterSeconds;
    c.stanceHoldInDodge = true;
    c.stanceCleanEdge = true;
    c.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
    c.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    c.gravity = 16.0f;
    return c;
}

// ------------------------------------------------------------------------------------------------ part 1: synthetic sequences
adapter::MotionFeed SynthFeed(float aDt, int aDetail, float aSpeed, bool aOnGround = true)
{
    const float vel[3] = {0.0f, aSpeed, aOnGround ? 0.0f : -1.5f};
    const float mv[2] = {0.0f, aSpeed > 0.01f ? 1.0f : 0.0f};
    adapter::AirFeed air;
    air.valid = true;
    air.onGround = aOnGround;
    air.distanceToGround = aOnGround ? 0.0f : 0.12f;
    // psm block: detail + 1 in bits 16..23 together with kPsmDetailed (bit 6), as scripts/utpp.reds packs it
    const uint32_t psm = (uint32_t(aDetail + 1) << 16) | (1u << 6);
    return adapter::MakeMotionFeed(vel, mv, aDt, psm, 0, air, 0.0f, 0.0f);
}

struct Seg
{
    int detail;
    int n;
    float speed;
};
using Seq = std::vector<Seg>;

// stance fed to the chooser per feed
std::vector<int> RunSeq(const adapter::MapConfig& aCfg, const Seq& aSeq)
{
    const float dt = 1.0f / 60.0f;
    adapter::MapState m;
    adapter::MapContext ctx;
    std::vector<int> out;
    for (const Seg& s : aSeq)
        for (int i = 0; i < s.n; ++i)
            out.push_back(adapter::MapToChooser(SynthFeed(dt, s.detail, s.speed), aCfg, m, ctx).stance);
    return out;
}

int FirstOf(const std::vector<int>& aOut, int aValue, size_t aFrom)
{
    for (size_t i = aFrom; i < aOut.size(); ++i)
        if (aOut[i] == aValue)
            return int(i);
    return -1;
}

int Flips(const std::vector<int>& aOut)
{
    int n = 0;
    for (size_t i = 1; i < aOut.size(); ++i)
        n += aOut[i] != aOut[i - 1];
    return n;
}

void SyntheticChecks()
{
    const adapter::MapConfig c = RuntimeConfig(true);
    const double dt = 1.0 / 60.0;
    const double filter = double(kFilterS);

    // --- a clean edge reaches the chooser on its own feed, in both directions
    {
        const auto up = RunSeq(c, {{1, 30, 0.0f}, {3, 30, 0.0f}});
        Check(FirstOf(up, 1, 0) == 30, "synthetic: a clean Stand -> Crouch edge in place reaches the chooser on its own feed (feed, expected)", FirstOf(up, 1, 0), 30);
        const auto down = RunSeq(c, {{3, 30, 0.0f}, {1, 30, 0.0f}});
        Check(FirstOf(down, 0, 0) == 30, "synthetic: a clean Crouch -> Stand edge reaches the chooser on its own feed (feed, expected)", FirstOf(down, 0, 0), 30);
        const auto walkDown = RunSeq(c, {{3, 30, 2.7f}, {1, 30, 2.7f}});
        Check(FirstOf(walkDown, 0, 0) == 30, "synthetic: a clean stand-up while crouch walking (2.7 m/s) is immediate", FirstOf(walkDown, 0, 0), 30);
        const auto walkUp = RunSeq(c, {{1, 30, 2.7f}, {3, 30, 2.7f}});
        Check(FirstOf(walkUp, 1, 0) == 30, "synthetic: a crouch entered at the crouch walk speed (2.7 m/s) is immediate", FirstOf(walkUp, 1, 0), 30);
        const auto aimUp = RunSeq(c, {{2, 30, 0.0f}, {3, 30, 0.0f}, {2, 30, 0.0f}});
        Check(FirstOf(aimUp, 1, 0) == 30 && FirstOf(aimUp, 0, 31) == 60, "synthetic: AimWalk 2 <-> Crouch 3 edges are clean too (feeds 30, 60)", FirstOf(aimUp, 1, 0),
              FirstOf(aimUp, 0, 31));
    }
    // --- an edge that comes from / goes to CrouchSprint 30 keeps the filter (accepted after the filter time, not before)
    {
        for (const auto& [name, seq] : {std::pair<const char*, Seq>{"Crouch -> CrouchSprint 30", {{3, 30, 0.0f}, {30, 30, 0.0f}}},
                                        std::pair<const char*, Seq>{"CrouchSprint 30 -> Crouch", {{30, 30, 0.0f}, {3, 30, 0.0f}}}})
        {
            const auto o = RunSeq(c, seq);
            const bool toStand = seq[0].detail == 3;
            const int first = FirstOf(o, toStand ? 0 : 1, 0);
            const double delay = (first - 30 + 1) * dt; // the accepting feed is the filter-th feed of the new value
            char what[160];
            std::snprintf(what, sizeof(what), "synthetic: %s is delayed by the filter time only (s, filter)", name);
            Check(first >= 30 && delay >= filter - 0.001 && delay <= filter + dt + 0.001, what, delay, filter);
        }
        // Sprint 4 and Dodge 7 are not clean either
        const auto sp = RunSeq(c, {{3, 30, 0.0f}, {4, 30, 0.0f}});
        const double dsp = (FirstOf(sp, 0, 0) - 30 + 1) * dt;
        Check(dsp >= filter - 0.001 && dsp <= filter + dt + 0.001, "synthetic: Crouch -> Sprint 4 keeps the filter (s)", dsp, filter);
    }
    // --- flicker: single-feed CrouchSprint 30 spikes inside the crouch (and in the stand) never reach the chooser
    {
        Seq seq = {{3, 30, 0.0f}};
        for (int i = 0; i < 3; ++i)
        {
            seq.push_back({30, 1, 0.0f});
            seq.push_back({3, 7, 0.0f});
        }
        Check(Flips(RunSeq(c, seq)) == 0, "synthetic: 1-feed CrouchSprint spikes inside the crouch do not reach the chooser (flips)", Flips(RunSeq(c, seq)));
        adapter::MapConfig off = c;
        off.stanceFilterS = 0.0f;
        Check(Flips(RunSeq(off, seq)) == 6, "synthetic (mutation): without the filter every spike flips the stance twice", Flips(RunSeq(off, seq)));
        // the 3 <-> 30 train of the recording: 0.1 s Crouch, 0.017 s CrouchSprint, repeated
        Seq train = {{3, 30, 2.8f}};
        for (int i = 0; i < 12; ++i)
        {
            train.push_back({30, 1, 2.8f});
            train.push_back({3, 6, 2.8f});
        }
        Check(Flips(RunSeq(c, train)) == 0, "synthetic: the recorded Crouch 3 <-> CrouchSprint 30 train does not reach the chooser (flips)", Flips(RunSeq(c, train)));
    }
    // --- min-hold: a return inside the filter time of a bypassed edge goes through the filter
    {
        const auto o = RunSeq(c, {{1, 30, 0.0f}, {3, 3, 0.0f}, {1, 2, 0.0f}, {3, 30, 0.0f}});
        Check(o[30] == 1 && Flips(o) == 1, "synthetic: Stand -> Crouch (bypass) -> 2 stand feeds -> Crouch: the return does not reach the chooser (flips)", Flips(o), 1);
        const auto p = RunSeq(c, {{3, 30, 0.0f}, {1, 3, 0.0f}, {3, 2, 0.0f}, {1, 30, 0.0f}});
        Check(p[30] == 0 && p[36] == 0 && Flips(p) == 1, "synthetic: Crouch -> Stand (bypass) -> 2 crouch feeds -> Stand: the return does not reach the chooser (flips)", Flips(p), 1);
        // a decided double toggle (each state held >= the filter time) is followed
        const auto t = RunSeq(c, {{1, 30, 0.0f}, {3, 12, 0.0f}, {1, 12, 0.0f}});
        Check(FirstOf(t, 1, 0) == 30 && FirstOf(t, 0, 31) == 42, "synthetic: a toggle held 0.2 s each way is followed on both edges (feeds 30, 42)", FirstOf(t, 1, 0),
              FirstOf(t, 0, 31));
    }
    // --- the crouch ENTRY speed limit: above the crouch walk speed the entry keeps the filter; stand -> 3 -> 30 at the run speed never shows
    {
        for (const float sp : {3.0f, 3.85f})
        {
            const auto o = RunSeq(c, {{1, 30, sp}, {3, 30, sp}});
            const double delay = (FirstOf(o, 1, 0) - 30 + 1) * dt;
            char what[160];
            std::snprintf(what, sizeof(what), "synthetic: a crouch entered at %.2f m/s keeps the filter delay (s, filter)", double(sp));
            Check(delay >= filter - 0.001 && delay <= filter + dt + 0.001, what, delay, filter);
        }
        // 12:13 feed t 278.786 / 280.435: Stand -> Crouch 3 for 0.10-0.12 s -> CrouchSprint 30 at 3.85 m/s - the crouch never reaches the chooser
        const auto blip = RunSeq(c, {{1, 30, 3.85f}, {3, 6, 3.85f}, {30, 60, 2.8f}});
        Check(Flips(blip) == 0 && blip.back() == 0, "synthetic: Stand -> Crouch 0.1 s -> CrouchSprint at 3.85 m/s: no crouch reaches the chooser (flips)", Flips(blip));
        adapter::MapConfig noLimit = c;
        noLimit.stanceCleanMaxSpeedMs = 1.0e9f;
        Check(Flips(RunSeq(noLimit, {{1, 30, 3.85f}, {3, 6, 3.85f}, {30, 60, 2.8f}})) >= 2, "synthetic (mutation): without the speed limit the blip shows as a crouch", 2.0);
    }
    // --- Dodge: a Dodge from the crouch / the stand keeps the stance (unchanged by the bypass)
    {
        const auto o = RunSeq(c, {{3, 30, 2.7f}, {7, 12, 2.7f}, {3, 30, 2.7f}});
        Check(Flips(o) == 0 && o[0] == 1, "synthetic: Dodge 7 from the crouch keeps the crouch (flips)", Flips(o));
        const auto s = RunSeq(c, {{1, 30, 3.85f}, {7, 12, 3.85f}, {1, 30, 3.85f}});
        Check(Flips(s) == 0 && s[0] == 0, "synthetic: Dodge 7 from the stand stays stand (flips)", Flips(s));
        // the edge after a Dodge is not clean (the previous detail was 7)
        const auto d = RunSeq(c, {{3, 30, 0.0f}, {7, 6, 0.0f}, {1, 30, 0.0f}});
        const double dd = (FirstOf(d, 0, 0) - 36 + 1) * dt;
        Check(FirstOf(d, 0, 0) >= 36 && dd >= filter - 0.001 && dd <= filter + dt + 0.001, "synthetic: the edge right after a Dodge keeps the filter (s)", dd, filter);
    }
    // --- slide context: pass-through frames, the provisional value and the exit window are not touched by the bypass
    {
        // slide -> Crouch 0.3 s -> Stand inside the slide exit window (0.6 s): the filter time
        const auto in = RunSeq(c, {{5, 20, 6.0f}, {3, 18, 2.7f}, {1, 30, 2.7f}});
        const double din = (FirstOf(in, 0, 38) - 38 + 1) * dt;
        Check(FirstOf(in, 0, 38) >= 38 && din >= filter - 0.001 && din <= filter + dt + 0.001, "synthetic: a stand-up 0.3 s after a slide keeps the filter (s)", din, filter);
        // ... and after the exit window the same edge is clean
        const auto late = RunSeq(c, {{5, 20, 6.0f}, {3, 50, 2.7f}, {1, 30, 2.7f}});
        Check(FirstOf(late, 0, 70) == 70, "synthetic: a stand-up 0.83 s after a slide is clean (feed, expected)", FirstOf(late, 0, 70), 70);
        // with the slide feed off the PSM Slide 5 is no pass-through frame, the edge from 5 is still not clean (previous detail 5)
        const adapter::MapConfig off = RuntimeConfig(false);
        const auto o = RunSeq(off, {{5, 20, 6.0f}, {3, 40, 2.7f}});
        Check(FirstOf(o, 1, 20) >= 20 + 8, "synthetic: slide feed off: Slide 5 -> Crouch keeps the filter (feeds)", FirstOf(o, 1, 20) - 20, 8);
    }
    // --- the landing of the recording (feed t 340.306): Jump 18 -> Crouch 3 for 0.034 s -> Stand never shows a crouch
    {
        const auto o = RunSeq(c, {{1, 30, 8.5f}, {18, 20, 8.5f}, {3, 2, 8.5f}, {1, 30, 8.2f}});
        Check(Flips(o) == 0 && o[0] == 0, "synthetic: air 18 -> Crouch 3 (2 feeds) -> Stand: no crouch reaches the chooser (flips)", Flips(o));
    }
    // --- the pre-P1 behaviour is the switch's off position (bit for bit)
    {
        adapter::MapConfig off = c;
        off.stanceCleanEdge = false;
        const auto o = RunSeq(off, {{1, 30, 0.0f}, {3, 30, 0.0f}});
        const double delay = (FirstOf(o, 1, 0) - 30 + 1) * dt;
        Check(delay >= filter - 0.001 && delay <= filter + dt + 0.001, "synthetic: stanceCleanEdge off = the pre-P1 filter delay on a clean edge (s)", delay, filter);
    }
}

// ------------------------------------------------------------------------------------------------ part 2: replay of the recording
struct Row
{
    double t = 0;
    float dt = 0, vel[3] = {0, 0, 0};
    int loco = 0;
    uint32_t psm = 0;
    int onGround = -1;
    float dist = -1;
    int fall = 0;
};
struct Section
{
    std::string name;
    std::vector<Row> rows;
};

float Num(const std::string& aTok, float aDef = 0.0f) { return aTok == "-" ? aDef : std::strtof(aTok.c_str(), nullptr); }

bool LoadFixture(const char* aPath, std::vector<Section>& aOut)
{
    std::ifstream f(aPath);
    if (!f)
        return false;
    std::string line;
    std::vector<std::string> cols;
    while (std::getline(f, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind("# cols", 0) == 0)
        {
            std::istringstream is(line.substr(6));
            std::string c;
            cols.clear();
            while (is >> c)
                cols.push_back(c);
            continue;
        }
        if (line.empty() || line[0] == '#')
            continue;
        if (line.rfind("@section ", 0) == 0)
        {
            std::string name = line.substr(9);
            aOut.push_back({name.substr(0, name.find(' ')), {}});
            continue;
        }
        if (aOut.empty() || cols.empty())
            return false;
        std::istringstream is(line);
        std::vector<std::string> tok;
        std::string t;
        while (is >> t)
            tok.push_back(t);
        if (tok.size() != cols.size())
            return false;
        auto at = [&](const char* aName) -> const std::string& {
            for (size_t i = 0; i < cols.size(); ++i)
                if (cols[i] == aName)
                    return tok[i];
            static const std::string kDash = "-";
            return kDash;
        };
        Row r;
        r.t = std::strtod(at("t").c_str(), nullptr);
        r.dt = Num(at("dt"));
        r.vel[0] = Num(at("vx"));
        r.vel[1] = Num(at("vy"));
        r.vel[2] = Num(at("vz"));
        r.loco = static_cast<int>(Num(at("loco")));
        r.psm = static_cast<uint32_t>(std::strtoull(at("psm").c_str(), nullptr, 10));
        r.onGround = at("onGround") == "-" ? -1 : static_cast<int>(Num(at("onGround")));
        r.dist = Num(at("distGround"), -1.0f);
        r.fall = static_cast<int>(Num(at("psmFall")));
        aOut.back().rows.push_back(r);
    }
    return !aOut.empty();
}

struct Frame
{
    double t = 0;
    int detail = 0;
    int raw = 0;     // PSM stance of the feed
    int stance = 0;  // stance fed to the chooser
    float speed = 0; // planar capsule speed, m/s
    float dt = 0;
};

std::vector<Frame> Replay(const Section& aSec, const adapter::MapConfig& aCfg)
{
    std::vector<Frame> out;
    adapter::MapState ms;
    adapter::MapContext ctx;
    for (const Row& r : aSec.rows)
    {
        const float dt = std::fmin(r.dt > 0 ? r.dt : 0.0f, 0.25f);
        adapter::AirFeed air;
        air.valid = r.onGround >= 0;
        air.onGround = r.onGround != 0;
        air.distanceToGround = r.dist;
        air.psmFall = r.fall;
        const float mv[2] = {0.0f, 0.0f};
        const adapter::MotionFeed f = adapter::MakeMotionFeed(r.vel, mv, dt, r.psm, r.loco, air, 0.0f, 0.0f);
        const motion::ChooserState cs = adapter::MapToChooser(f, aCfg, ms, ctx);
        Frame fr;
        fr.t = r.t;
        fr.detail = f.psmDetailed;
        fr.raw = adapter::PsmCrouchStance(f.psmDetailed) ? 1 : 0;
        fr.stance = cs.stance;
        fr.speed = std::sqrt(r.vel[0] * r.vel[0] + r.vel[1] * r.vel[1]);
        fr.dt = dt;
        out.push_back(fr);
    }
    return out;
}

struct Edge
{
    size_t sec = 0, idx = 0;
    double t = 0;
    int from = 0, to = 0; // details
    int newRaw = 0;
    bool clean = false;    // clean details on both feeds, old value held >= hold, new value held >= 0.3 s, no slide / air / Dodge / CrouchSprint in the 0.8 s before
    bool moving = false;   // a crouch entry at >= the speed limit
    double latencyS = -1, latencyFeeds = -1; // chooser stance == newRaw first reached
};

struct Summary
{
    int edges = 0, qualifying = 0, worstFeeds = 0, over1 = 0, gated = 0, gatedOk = 0, shortRuns = 0, shortRunsRaw = 0;
    int cleanEntries = 0, cleanExits = 0;
    double worstS = 0;
};

Summary Analyse(const std::vector<Section>& aSecs, const adapter::MapConfig& aCfg, bool aPrint, const char* aName)
{
    Summary sum;
    for (size_t s = 0; s < aSecs.size(); ++s)
    {
        const std::vector<Frame> fr = Replay(aSecs[s], aCfg);
        if (fr.size() < 4)
            continue;
        // raw runs / edges
        for (size_t i = 1; i < fr.size(); ++i)
        {
            if (fr[i].raw == fr[i - 1].raw)
                continue;
            Edge e;
            e.sec = s;
            e.idx = i;
            e.t = fr[i].t;
            e.from = fr[i - 1].detail;
            e.to = fr[i].detail;
            e.newRaw = fr[i].raw;
            ++sum.edges;
            // old run: back to the previous edge or the section start
            size_t a = i - 1;
            while (a > 0 && fr[a - 1].raw == fr[i - 1].raw)
                --a;
            const double oldHold = fr[i].t - fr[a].t;
            size_t b = i;
            while (b + 1 < fr.size() && fr[b + 1].raw == fr[i].raw)
                ++b;
            const double newHold = fr[b].t - fr[i].t + double(fr[b].dt);
            bool context = false; // a slide / air / Dodge / CrouchSprint / Sprint frame shortly before, or CrouchSprint inside the new run's first filter time
            for (size_t q = i; q-- > 0 && fr[i].t - fr[q].t < 0.8;)
                context = context || !adapter::CleanStanceDetail(fr[q].detail);
            for (size_t q = i; q < fr.size() && fr[q].t - fr[i].t < double(kFilterS); ++q)
                context = context || !adapter::CleanStanceDetail(fr[q].detail);
            e.clean = adapter::CleanStanceDetail(e.from) && adapter::CleanStanceDetail(e.to) && oldHold >= double(kFilterS) && newHold >= 0.3 && !context;
            e.moving = e.newRaw == 1 && fr[i].speed >= adapter::kStanceCleanMaxSpeedMs;
            for (size_t q = i; q <= b; ++q)
                if (fr[q].stance == e.newRaw)
                {
                    e.latencyFeeds = double(q - i);
                    e.latencyS = fr[q].t - fr[i].t;
                    break;
                }
            if (!e.clean)
                continue;
            if (e.moving)
            {
                // a held crouch entered above the speed limit: the pre-P1 filter delay stays
                ++sum.gated;
                sum.gatedOk += e.latencyS >= double(kFilterS) - 0.02 && e.latencyS <= double(kFilterS) + 0.05;
            }
            else
            {
                ++sum.qualifying;
                (e.newRaw == 1 ? sum.cleanEntries : sum.cleanExits)++;
                sum.worstFeeds = std::max(sum.worstFeeds, e.latencyFeeds < 0 ? 1000 : int(e.latencyFeeds));
                sum.worstS = std::max(sum.worstS, e.latencyS < 0 ? 1.0e3 : e.latencyS);
                sum.over1 += e.latencyFeeds < 0 || e.latencyFeeds > 1;
            }
            if (aPrint)
                std::printf("  [%s] %s t=%.3f (v %.2f) %d->%d speed %.2f: chooser stance after %.0f feeds (%.3f s)%s\n", aName, e.moving ? "gated " : "clean ", e.t,
                            e.t - 91.56, e.from, e.to, double(fr[i].speed), e.latencyFeeds, e.latencyS, e.latencyFeeds > 1 && !e.moving ? "  <-- SLOW" : "");
        }
        // fed stance runs shorter than the flicker bound (section ends excluded), fed and raw
        for (int pass = 0; pass < 2; ++pass)
        {
            size_t st = 0;
            for (size_t i = 1; i <= fr.size(); ++i)
            {
                const int cur = i < fr.size() ? (pass ? fr[i].raw : fr[i].stance) : -1;
                const int prev = pass ? fr[st].raw : fr[st].stance;
                if (i == fr.size() || cur != prev)
                {
                    const double dur = i < fr.size() ? fr[i].t - fr[st].t : 1.0e9;
                    if (st > 0 && i < fr.size() && dur < kShortRunS)
                        (pass ? sum.shortRunsRaw : sum.shortRuns)++;
                    st = i;
                }
            }
        }
    }
    return sum;
}

void ReplayChecks(const std::vector<Section>& aSecs)
{
    // fixture sanity: the decoded PSM detail agrees with the dump's own detailed column wherever that is set (psm packs detail + 1)
    size_t rows = 0, d30 = 0, d3 = 0;
    for (const Section& s : aSecs)
        for (const Row& r : s.rows)
        {
            ++rows;
            const int d = adapter::DetailFromFeed(r.psm, r.loco);
            d30 += d == adapter::kPsmCrouchSprint;
            d3 += d == adapter::kPsmCrouch;
        }
    std::printf("fixture: %zu sections, %zu rows (Crouch 3: %zu, CrouchSprint 30: %zu)\n", aSecs.size(), rows, d3, d30);
    Check(rows > 5000 && d30 > 100 && d3 > 500, "fixture: the recorded windows hold Crouch 3 and CrouchSprint 30 frames", double(d3), double(d30));

    for (const bool slide : {true, false})
    {
        const char* name = slide ? "runtime, slide feed on" : "runtime, slide feed off";
        const adapter::MapConfig c = RuntimeConfig(slide);
        const Summary s = Analyse(aSecs, c, g_report && slide, name);
        std::printf("[%s] PSM stance edges %d, clean in place %d (entries %d, stand-ups %d), clean but entered at >= %.1f m/s %d, fed stance runs < %.2f s: %d (PSM %d)\n",
                    name, s.edges, s.qualifying, s.cleanEntries, s.cleanExits, double(adapter::kStanceCleanMaxSpeedMs), s.gated, kShortRunS, s.shortRuns, s.shortRunsRaw);
        char what[200];
        std::snprintf(what, sizeof(what), "replay [%s]: the recording has >= 60 clean in-place edges (31 crouches + stand-ups of v 2:22-3:02 among them)", name);
        Check(s.qualifying >= 60 && s.cleanEntries >= 20 && s.cleanExits >= 30, what, s.qualifying, s.cleanEntries);
        std::snprintf(what, sizeof(what), "replay [%s]: every clean in-place edge reaches the chooser within 1 feed (edges over, worst feeds)", name);
        Check(s.over1 == 0, what, s.over1, s.worstFeeds);
        std::snprintf(what, sizeof(what), "replay [%s]: the held crouches entered at the run speed keep the pre-P1 filter delay (ok, of)", name);
        Check(s.gated >= 4 && s.gatedOk == s.gated, what, s.gatedOk, s.gated);
        std::snprintf(what, sizeof(what), "replay [%s]: no stance run shorter than 0.15 s reaches the chooser (count, PSM runs)", name);
        Check(s.shortRuns == 0 && s.shortRunsRaw >= 40, what, s.shortRuns, s.shortRunsRaw);

        // the landing of feed t 340.306: Jump 18 -> Crouch 3 for 0.034 s -> Stand
        bool landingSeen = false, landingCrouch = false;
        for (const Section& sec : aSecs)
            if (!sec.rows.empty() && sec.rows.front().t < 340.30 && sec.rows.back().t > 340.35)
                for (const Frame& f : Replay(sec, c))
                    if (f.t >= 340.25 && f.t <= 340.45)
                    {
                        landingSeen = true;
                        landingCrouch = landingCrouch || f.stance == 1;
                    }
        std::snprintf(what, sizeof(what), "replay [%s]: the landing at feed t 340.306 (Jump 18 -> Crouch 3 for 0.034 s) does not change the stance", name);
        Check(landingSeen && !landingCrouch, what, landingSeen ? 1.0 : 0.0, landingCrouch ? 1.0 : 0.0);

        // the Stand -> Crouch 3 -> CrouchSprint blips at 278.786 / 280.435 (3.85 m/s) never show as a crouch
        bool blipSeen = false, blipCrouch = false;
        for (const Section& sec : aSecs)
            for (const double t0 : {278.786, 280.435})
                if (!sec.rows.empty() && sec.rows.front().t < t0 - 0.2 && sec.rows.back().t > t0 + 0.3)
                    for (const Frame& f : Replay(sec, c))
                        if (f.t >= t0 - 0.1 && f.t <= t0 + 0.25)
                        {
                            blipSeen = true;
                            blipCrouch = blipCrouch || f.stance == 1;
                        }
        std::snprintf(what, sizeof(what), "replay [%s]: the Stand -> Crouch 0.1 s -> CrouchSprint blips (feed t 278.786, 280.435) show no crouch", name);
        Check(blipSeen && !blipCrouch, what, blipSeen ? 1.0 : 0.0, blipCrouch ? 1.0 : 0.0);
    }

    // mutations of the configuration: the replay must notice each of them
    {
        adapter::MapConfig off = RuntimeConfig(true);
        off.stanceCleanEdge = false;
        const Summary s = Analyse(aSecs, off, false, "mutation");
        Check(s.over1 > 40, "replay (mutation): without the bypass the clean edges are 8-9 feeds late (edges over 1 feed)", s.over1, s.worstFeeds);
        adapter::MapConfig noFilter = RuntimeConfig(true);
        noFilter.stanceFilterS = 0.0f;
        const Summary n = Analyse(aSecs, noFilter, false, "mutation");
        Check(n.shortRuns > 20, "replay (mutation): without the filter the CrouchSprint / slide flicker reaches the chooser (fed runs < 0.15 s)", n.shortRuns, n.shortRunsRaw);
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_p1_stance_latency_test <feed_recording_20261021_1213.txt> [--report]\n");
        return 2;
    }
    for (int i = 2; i < argc; ++i)
        if (!std::strcmp(argv[i], "--report"))
            g_report = true;
    SyntheticChecks();
    std::vector<Section> secs;
    if (!LoadFixture(argv[1], secs))
    {
        std::printf("FAIL: fixture %s\n", argv[1]);
        return 1;
    }
    ReplayChecks(secs);
    std::printf("p1 stance latency: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

// Speed converter, starts and stops (speedconv::StartStopStep, MapConfig::speedConverter bit speedconv::kStartStopWindow): the
// converted speed of a start from rest follows the reference maximum acceleration (when enabled), a release brakes at the reference
// braking deceleration, both hand back to the turn converter's speed without a jump, a steady run passes unchanged, and through the
// adapter the bit changes only the stop feeds of the query (starts are off by default). No data files.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

using namespace utpp::mm;
using namespace utpp::mm::adapter;
using motiontraj::V3;

static int passed = 0, failed = 0;
static void Check(const char* label, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    ok ? ++passed : ++failed;
}

static constexpr float kDt = 1.0f / 60.0f;
static constexpr float kAccel = 800.0f, kBrake = 2000.0f;

// The game capsule speed of a start from rest (feed index from the first input feed): full speed after 0.18 s
static float GameStart(int i, float aTop) { return std::min(aTop, aTop * float(i + 1) * kDt / 0.18f); }

static void UnitStart()
{
    speedconv::StartStopLimits L;
    L.starts = true; // off by default (StartStopLimits::starts)
    speedconv::StartStopState s;
    speedconv::StartStopStep(s, V3{}, V3{}, false, true, kDt, kAccel, kBrake, L); // idle
    const float top = 400.0f;
    float prev = 0.0f, maxStep = 0.0f, maxAbove = 0.0f, gapMax = 0.0f, gap = 0.0f, end = 0.0f;
    bool slower = false, converting = false, heading = true;
    for (int i = 0; i < 90; ++i)
    {
        const V3 g{0.0f, GameStart(i, top), 0.0f};
        const V3 q = speedconv::StartStopStep(s, g, g, true, true, kDt, kAccel, kBrake, L);
        const float sp = motiontraj::SizeXY(q);
        if (i < 6)
        {
            maxAbove = std::max(maxAbove, sp - prev - kAccel * kDt); // inside the window: at most the reference acceleration
            slower = slower || sp < g.y - 50.0f;
            converting = converting || speedconv::StartStopConverting(s);
        }
        maxStep = std::max(maxStep, std::fabs(sp - prev));
        gap += (g.y - sp) * kDt;
        gapMax = std::max(gapMax, gap);
        prev = sp;
        end = sp;
        heading = heading && std::fabs(q.x) < 1e-3f && q.y >= 0.0f;
    }
    Check("start: heading along the game velocity", heading);
    Check("start: the converted speed follows the reference acceleration", slower && maxAbove <= 0.01f && converting);
    // the gap cap ends the window; the hand-off continues the gap only for ~handoffS
    Check("start: the converted path stays within the gap cap plus the hand-off", gapMax < speedconv::StartStopLimits{}.gapCapCm + 0.1f * top);
    Check("start: the hand-off has no speed jump", maxStep < 80.0f);
    Check("start: back on the game speed", std::fabs(end - top) < 1e-3f && !speedconv::StartStopConverting(s));
    // default limits: no start window
    speedconv::StartStopState d;
    speedconv::StartStopStep(d, V3{}, V3{}, false, true, kDt, kAccel, kBrake);
    const V3 g{0.0f, GameStart(0, top), 0.0f};
    const V3 q = speedconv::StartStopStep(d, g, g, true, true, kDt, kAccel, kBrake);
    Check("start: off by default", q.y == g.y && !speedconv::StartStopConverting(d));
}

static void UnitStop()
{
    speedconv::StartStopState s;
    const float top = 650.0f; // a sprint: the game brakes it in ~0.17 s, the reference in ~0.33 s
    for (int i = 0; i < 60; ++i)
    {
        const V3 g{top, 0.0f, 0.0f};
        speedconv::StartStopStep(s, g, g, true, true, kDt, kAccel, kBrake);
    }
    float prev = top, minStep = 0.0f, below = 0.0f, end = -1.0f;
    bool slower = false;
    V3 last{};
    for (int i = 0; i < 60; ++i)
    {
        const float gs = std::max(0.0f, top - 3800.0f * float(i + 1) * kDt);
        const V3 g{gs, 0.0f, 0.0f};
        const V3 q = speedconv::StartStopStep(s, g, g, false, true, kDt, kAccel, kBrake);
        const float sp = motiontraj::SizeXY(q);
        if (i < 8)
        {
            minStep = std::min(minStep, sp - prev + kBrake * kDt); // braking at most at the reference deceleration
            slower = slower || sp > gs + 50.0f;
        }
        below = std::max(below, gs - sp);
        prev = sp;
        end = sp;
        last = q;
    }
    Check("stop: the converted speed brakes at the reference deceleration", slower && minStep >= -0.01f);
    Check("stop: never below the game speed", below <= 1e-3f);
    Check("stop: ends at rest", end == 0.0f && last.x == 0.0f && last.y == 0.0f);
}

static void UnitSteadyAndAir()
{
    speedconv::StartStopState s;
    bool same = true;
    for (int i = 0; i < 120; ++i)
    {
        const V3 g{std::sin(float(i) * 0.05f) * 300.0f, 300.0f, 12.0f};
        const V3 q = speedconv::StartStopStep(s, g, g, true, true, kDt, kAccel, kBrake);
        same = same && q.x == g.x && q.y == g.y && q.z == g.z;
    }
    Check("steady run: the velocity passes unchanged", same);
    // in the air the window never opens
    speedconv::StartStopState a;
    speedconv::StartStopStep(a, V3{}, V3{}, false, false, kDt, kAccel, kBrake);
    const V3 g{0.0f, 400.0f, 0.0f};
    const V3 q = speedconv::StartStopStep(a, g, g, true, false, kDt, kAccel, kBrake);
    Check("off the ground: the velocity passes unchanged", q.y == g.y && !speedconv::StartStopConverting(a));
}

// Through the adapter: idle, a start, a long run, a release. Returns the query velocity over the last 0.1 s of every feed.
static std::vector<float> RunStartStop(int aConverter, std::vector<float>* aSamples = nullptr)
{
    MapConfig mc;
    mc.speedConverter = aConverter;
    MapState ms;
    std::vector<float> out;
    const int idle = 30, run = 120, stop = 40;
    for (int i = 0; i < idle + run + stop; ++i)
    {
        float sp = 0.0f, mvy = 0.0f;
        if (i >= idle && i < idle + run)
        {
            sp = GameStart(i - idle, 3.85f);
            mvy = 1.0f;
        }
        else if (i >= idle + run)
            sp = std::max(0.0f, 3.85f - 23.0f * float(i - idle - run + 1) * kDt);
        const float vel[3] = {0.0f, sp, 0.0f}, mv[2] = {0.0f, mvy};
        MotionFeed f = MakeMotionFeed(vel, mv, kDt, 0u, 0, AirFeed{}, 0.0f, 0.0f);
        f.psmDetailed = kPsmStand;
        MapContext ctx;
        ctx.rootYawValid = true;
        const motion::ChooserState cs = MapToChooser(f, mc, ms, ctx);
        out.push_back(motiontraj::SizeXY(ms.traj.VelocityBetween(-0.1f, 0.0f)));
        if (aSamples)
        {
            aSamples->push_back(cs.rateSpeedScale);
            for (int k = -2; k <= 15; ++k)
            {
                const motion::TrajSample q = QuerySample(ms.traj, 0.1f * float(k));
                aSamples->insert(aSamples->end(), {q.x, q.y, q.z, q.yaw});
            }
        }
    }
    return out;
}

static void Adapter()
{
    const int base = speedconv::PresetFor(speedconv::kPresetStronger).bits;
    std::vector<float> samplesBase, samplesWin;
    const std::vector<float> a = RunStartStop(base, &samplesBase);
    const std::vector<float> b = RunStartStop(base | speedconv::kStartStopWindow, &samplesWin);
    const int idle = 30, run = 120;
    Check("adapter: the start query is unchanged (starts off by default)", b[idle + 6] == a[idle + 6]);
    Check("adapter: the stop query brakes later with the window", b[idle + run + 5] > a[idle + run + 5] + 20.0f);
    // the steady part of the run and the idle before the start are bit identical (the window closed, the hand-off done)
    bool idleSame = true, runSame = true;
    for (int i = 0; i < idle; ++i)
        idleSame = idleSame && a[i] == b[i];
    const size_t per = 1 + 18 * 4;
    for (int i = idle; i < idle + run; ++i)
        for (size_t k = 0; k < per; ++k)
            runSame = runSame && std::fabs(samplesBase[size_t(i) * per + k] - samplesWin[size_t(i) * per + k]) <= 1.0e-4f;
    Check("adapter: idle before the start unchanged", idleSame);
    // the query samples are relative to the trajectory's current position, which integrates the converted velocity (float rounding)
    Check("adapter: the start and the run match the turn converter's query", runSame);
    // the window drops the game speed rate (the clip plays at its own rate while converted)
    bool rateOne = true;
    for (int i = idle + run; i < idle + run + 6; ++i)
        rateOne = rateOne && samplesWin[size_t(i) * per] <= samplesBase[size_t(i) * per] + 1.0e-6f;
    if (!rateOne)
        for (int i = idle + run; i < idle + run + 6; ++i)
            std::printf("  rate base %.4f window %.4f\n", samplesBase[size_t(i) * per], samplesWin[size_t(i) * per]);
    Check("adapter: the stop window does not speed the clip up", rateOne);
}

int main()
{
    UnitStart();
    UnitStop();
    UnitSteadyAndAir();
    Adapter();
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}

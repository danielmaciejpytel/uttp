// Guards of the capsule speed play rate (MapConfig::capsuleRateGuards, capsuleRateBlendS; caprate:: in mm/MotionAdapter.hpp): the
// direction ratio of a strafe, the dodge at the mapped query rate, the ratio of a start at the speed it accelerates to, crouched loops
// only, and the blend of the scale
// between feeds. Unit checks of each helper plus a run through MapToChooser. No data files.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace utpp::mm;
using namespace utpp::mm::adapter;

static int passed = 0, failed = 0;
static void Check(const char* label, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    ok ? ++passed : ++failed;
}

// A straight run along world +y at the game run speed (3.85 m/s after a 0.5 s ramp), aDodgeFrom..aDodgeTo feeds in the dodge state.
// Returns ChooserState::rateSpeedScale and isStarting of every feed.
struct Run
{
    std::vector<float> rate;
    std::vector<bool> starting;
};
static Run RunStraight(const MapConfig& aMc, int aFeeds, int aDodgeFrom = -1, int aDodgeTo = -1)
{
    const float dt = 1.0f / 60.0f;
    MapState ms;
    Run out;
    for (int i = 0; i < aFeeds; ++i)
    {
        const float sp = std::fmin(1.0f, float(i) / 30.0f) * 3.85f;
        const float vel[3] = {0.0f, sp, 0.0f}, mv[2] = {0.0f, 1.0f};
        MotionFeed f = MakeMotionFeed(vel, mv, dt, 0u, 0, AirFeed{}, 0.0f, 0.0f);
        f.psmDetailed = i >= aDodgeFrom && i < aDodgeTo ? kPsmDodge : kPsmStand;
        MapContext ctx;
        ctx.rootYawValid = true;
        const motion::ChooserState cs = MapToChooser(f, aMc, ms, ctx);
        out.rate.push_back(cs.rateSpeedScale);
        out.starting.push_back(cs.isStarting);
    }
    return out;
}

int main()
{
    // ---- shipped defaults
    {
        const MapConfig d;
        Check("defaults: capsule speed play rate on with every guard but the start one and the 0.2 s blend",
              d.capsuleSpeedRate == CapsuleRateMode::Both && d.capsuleRateScope == 0 && d.capsuleRateGuards == (caprate::kAll & ~(caprate::kStart | caprate::kRefaceStarts)) &&
                  d.capsuleRateBlendS == caprate::kBlendS && caprate::kBlendS == 0.2f);
    }
    // ---- kDirection: forward / direction reference speed (run 500 / back 300 / side 350 cm/s)
    {
        Check("direction ratio: a back strafe at the run caps 500 / 300", std::fabs(caprate::DirectionFactor(500.0f, 300.0f) - 500.0f / 300.0f) < 1e-6f);
        Check("direction ratio: a side strafe 500 / 350", std::fabs(caprate::DirectionFactor(500.0f, 350.0f) - 500.0f / 350.0f) < 1e-6f);
        Check("direction ratio: forward (direction speed = forward speed) keeps 1", caprate::DirectionFactor(500.0f, 500.0f) == 1.0f);
        Check("direction ratio: a faster direction or an invalid speed keeps 1",
              caprate::DirectionFactor(500.0f, 700.0f) == 1.0f && caprate::DirectionFactor(0.0f, 300.0f) == 1.0f &&
                  caprate::DirectionFactor(500.0f, std::nanf("")) == 1.0f);
        // the shooter back run: the forward map gives 0.77, the back strafe needs the capsule over the capped prediction
        Check("direction ratio: applied to the scale only with the direction guard",
              std::fabs(caprate::Target(0.77f, caprate::kDirection, false, 500.0f / 300.0f) - 0.77f * 500.0f / 300.0f) < 1e-6f &&
                  caprate::Target(0.77f, 0, false, 500.0f / 300.0f) == 0.77f);
    }
    // ---- kDodge / kStart: the mapped query rate (scale 1)
    {
        Check("dodge: the dodge and its decay play at the mapped query rate", caprate::Target(1.17f, caprate::kAll, true, 1.0f) == 1.0f);
        Check("dodge: without the dodge guard the scale stays", caprate::Target(1.17f, caprate::kAll & ~caprate::kDodge, true, 1.0f) == 1.17f);
        Check("off: scale 1 stays 1 under every guard", caprate::Target(1.0f, caprate::kAll, false, 500.0f / 300.0f) == 1.0f);
        // a sprint start (prediction at 700 cm/s) at a chooser speed of 260 cm/s reads the sprint ratio
        Check("start: a start reads the ratio at the speed its prediction accelerates to", caprate::RatioSpeed(260.0f, caprate::kAll, true, 700.0f) == 700.0f);
        Check("start: outside a start, or without the start guard, the chooser speed",
              caprate::RatioSpeed(260.0f, caprate::kAll, false, 700.0f) == 260.0f &&
                  caprate::RatioSpeed(260.0f, caprate::kAll & ~caprate::kStart, true, 700.0f) == 260.0f);
        Check("start: a prediction below the chooser speed keeps the chooser speed", caprate::RatioSpeed(650.0f, caprate::kAll, true, 500.0f) == 650.0f);
    }
    // ---- kCrouch: crouched, loop clips only
    {
        Check("crouch: crouched picks take the scale on loop clips only (scope 1)", caprate::Scope(0, caprate::kAll, 1) == 1);
        Check("crouch: standing keeps the configured scope", caprate::Scope(0, caprate::kAll & ~caprate::kRefaceStarts, 0) == 0 &&
                                                               caprate::Scope(2, caprate::kAll, 0) == 2 && caprate::Scope(1, caprate::kAll, 0) == 1);
        Check("reface starts: standing with scope 0 leaves the reface start clips at the mapped rate (scope 3)",
              caprate::Scope(0, caprate::kAll, 0) == 3 && caprate::Scope(0, caprate::kAll & ~caprate::kRefaceStarts, 0) == 0);
        Check("crouch: without the crouch guard the configured scope", caprate::Scope(0, caprate::kAll & ~caprate::kCrouch, 1) == 0);
    }
    // ---- blend: first order, no step, exact target in the end
    {
        const float dt = 1.0f / 60.0f;
        float s = 0.77f, firstStep = 0.0f;
        int feeds = 0;
        for (; feeds < 600 && s != 1.0f; ++feeds)
        {
            const float n = caprate::Blend(s, 1.0f, dt, caprate::kBlendS);
            if (feeds == 0)
                firstStep = n - s;
            s = n;
        }
        Check("blend: a guard toggle moves the scale by <= dt / 0.2 s of the gap per feed (no step)",
              firstStep > 0.0f && firstStep <= (1.0f - 0.77f) * dt / 0.2f + 1e-6f);
        Check("blend: the scale reaches exactly 1 (the crouch loop rule takes over again)", s == 1.0f && feeds > 30 && feeds < 300);
        Check("blend: time constant 0 = the target at once", caprate::Blend(0.77f, 1.0f, dt, 0.0f) == 1.0f);
        Check("blend: a 30 Hz feed moves twice the 60 Hz share",
              std::fabs((caprate::Blend(0.77f, 1.0f, 2.0f * dt, 0.2f) - 0.77f) - 2.0f * (caprate::Blend(0.77f, 1.0f, dt, 0.2f) - 0.77f)) < 1e-5f);
    }
    // ---- through MapToChooser: a run start and a dodge on a straight run
    {
        MapConfig guarded;
        guarded.capsuleRateGuards = caprate::kAll;
        MapConfig plain;
        plain.capsuleRateGuards = 0;
        plain.capsuleRateBlendS = 0.0f;
        MapConfig noStart; // every guard but the start one, same blend
        noStart.capsuleRateGuards = caprate::kAll & ~caprate::kStart;
        const Run g = RunStraight(guarded, 150), p = RunStraight(plain, 150), n = RunStraight(noStart, 150);
        // the steady run: the same scale as the unguarded rate (forward, no guard acts, the blend has settled)
        Check("chooser: a steady forward run keeps the unguarded scale", std::fabs(g.rate.back() - p.rate.back()) < 1e-4f && p.rate.back() < 0.9f);
        // the start phase: with the start guard the scale is closer to the run ratio (the speed the start accelerates to) than without it
        // (the walk ratio of the first feeds); both share the blend
        int startFeeds = 0;
        float startGap = 0.0f, noStartGap = 0.0f;
        const float runRatio = p.rate.back();
        for (size_t i = 0; i < g.rate.size(); ++i)
            if (g.starting[i])
            {
                ++startFeeds;
                startGap += std::fabs(g.rate[i] - runRatio);
                noStartGap += std::fabs(n.rate[i] - runRatio);
            }
        std::printf("  start: %d feeds, summed gap to the run ratio %.4f with the start guard, %.4f without\n", startFeeds, double(startGap), double(noStartGap));
        Check("chooser: a run start plays at the run ratio, not at the walk ratio of its first feeds", startFeeds > 0 && startGap < noStartGap);
        float step = 0.0f;
        for (size_t i = 1; i < g.rate.size(); ++i)
            step = std::fmax(step, std::fabs(g.rate[i] - g.rate[i - 1]));
        Check("chooser: no scale step above the blend share of one feed after the start", step <= 0.3f / 12.0f + 1e-4f);
        const Run gd = RunStraight(guarded, 150, 100, 130);
        Check("chooser: a dodge moves the scale toward 1 gradually, not in one feed",
              gd.rate[99] < 0.9f && gd.rate[100] < gd.rate[99] + 0.03f && gd.rate[129] > 0.98f);
    }
    std::printf("%d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

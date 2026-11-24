// Speed converter, terrain part (speedconv_terrain.hpp, kStepHold): a stair step dip of the game speed keeps the speed before it for at
// most kHoldS and then follows the game speed down; a dip without a step and an ineligible feed keep the game velocity bit for bit.
// Prints one PASS / FAIL line per check; exit code = number of failures.
#include <cmath>
#include <cstdio>
#include "../src/mm/SpeedConverter.hpp"
#include "../src/mm/speedconv_terrain.hpp"

namespace sc = utpp::mm::speedconv;
using utpp::mm::motiontraj::SizeXY;
using utpp::mm::motiontraj::V3;

static int g_fail = 0;
static void Check(bool aOk, const char* aName, double aValue)
{
    std::printf("%s %s value=%.3f\n", aOk ? "PASS" : "FAIL", aName, aValue);
    if (!aOk)
        ++g_fail;
}

static constexpr float kDt = 1.0f / 60.0f;

// a straight run at 238 cm/s for 0.5 s on level ground (capsule z 10 m), then the given dip profile
struct Run
{
    sc::StepHoldState s;
    float z = 10.0f;
    float t = 0.0f;
    V3 Feed(float aSpeed, float aDz = 0.0f, bool aEligible = true)
    {
        z += aDz;
        t += kDt;
        return sc::StepHoldVelocity(s, V3{0.0f, aSpeed, -25.0f}, z, true, aEligible, kDt);
    }
    void Steady(float aSeconds = 0.5f)
    {
        for (float e = 0.0f; e < aSeconds; e += kDt)
            Feed(238.0f);
    }
};

int main()
{
    {
        // ineligible / unknown height: the game velocity unchanged, bit for bit
        sc::StepHoldState s;
        const V3 g{12.5f, -230.25f, -40.0f};
        const V3 o = sc::StepHoldVelocity(s, g, 1.0f, true, false, kDt);
        const V3 o2 = sc::StepHoldVelocity(s, g, 1.0f, false, true, kDt);
        Check(o.x == g.x && o.y == g.y && o.z == g.z && o2.x == g.x && o2.y == g.y && o2.z == g.z, "identity_ineligible", 0.0);
    }
    {
        // step up of 10 cm with the speed dip in the same feed: the dip is held at the speed before it, z of the game kept
        Run r;
        r.Steady();
        float minHeld = 1.0e9f;
        bool zKept = true;
        V3 o = r.Feed(100.0f, 0.10f);
        minHeld = std::fmin(minHeld, SizeXY(o));
        zKept = zKept && o.z == -25.0f;
        for (int k = 0; k < 12; ++k) // 0.2 s inside the hold
        {
            o = r.Feed(100.0f);
            minHeld = std::fmin(minHeld, SizeXY(o));
            zKept = zKept && o.z == -25.0f;
        }
        Check(minHeld > 230.0f && zKept, "step_dip_held", minHeld);
        // the game speed comes back: the game velocity again
        o = r.Feed(238.0f);
        Check(std::fabs(SizeXY(o) - 238.0f) < 1.0e-3f, "recovered_game_speed", SizeXY(o));
    }
    {
        // the dip starts 0.1 s before the step (the capsule presses on the riser first): held from the step on
        Run r;
        r.Steady();
        for (int k = 0; k < 6; ++k)
            r.Feed(90.0f);
        const V3 o = r.Feed(90.0f, 0.12f);
        Check(SizeXY(o) > 230.0f, "dip_before_step_held", SizeXY(o));
    }
    {
        // a dip without a step (a stop, a wall): the game velocity
        Run r;
        r.Steady();
        float maxOut = 0.0f;
        for (int k = 0; k < 20; ++k)
            maxOut = std::fmax(maxOut, SizeXY(r.Feed(60.0f)));
        Check(std::fabs(maxOut - 60.0f) < 1.0e-3f, "no_step_dip_game_speed", maxOut);
        // a step long after the dip started (more than kStepWindowS) is no step dip either
        const V3 o = r.Feed(60.0f, 0.10f);
        Check(std::fabs(SizeXY(o) - 60.0f) < 1.0e-3f, "late_step_not_held", SizeXY(o));
    }
    {
        // a turn through a step dip (a pivot meeting a stair step): the game velocity
        Run r;
        r.Steady();
        const V3 o = sc::StepHoldVelocity(r.s, V3{150.0f, 40.0f, 0.0f}, r.z + 0.10f, true, true, kDt);
        Check(std::fabs(SizeXY(o) - SizeXY(V3{150.0f, 40.0f, 0.0f})) < 1.0e-3f, "turning_dip_game_speed", SizeXY(o));
    }
    {
        // a start from rest over a step: no dip (the speed before it is below kMinHoldSpeedCmS)
        Run r;
        for (int k = 0; k < 30; ++k)
            r.Feed(60.0f);
        const V3 o = r.Feed(30.0f, 0.10f);
        Check(std::fabs(SizeXY(o) - 30.0f) < 1.0e-3f, "slow_dip_game_speed", SizeXY(o));
    }
    {
        // a long step dip: after kHoldS the held speed follows the game speed down at kReleaseDecelCmS2 and reaches it
        Run r;
        r.Steady();
        r.Feed(80.0f, 0.10f);
        float atHoldEnd = 0.0f, prev = 1.0e9f, maxDrop = 0.0f;
        float tDip = 0.0f;
        float last = 0.0f;
        for (int k = 0; k < 90; ++k) // 1.5 s
        {
            tDip += kDt;
            const float v = SizeXY(r.Feed(80.0f));
            if (tDip > sc::kHoldS + 2.0f * kDt && prev < 1.0e8f)
                maxDrop = std::fmax(maxDrop, (prev - v) / kDt);
            if (tDip <= sc::kHoldS)
                atHoldEnd = v;
            prev = v;
            last = v;
        }
        Check(atHoldEnd > 230.0f, "hold_until_limit", atHoldEnd);
        Check(maxDrop <= sc::kReleaseDecelCmS2 * 1.01f, "release_rate_limited", maxDrop);
        Check(std::fabs(last - 80.0f) < 1.0e-3f, "release_reaches_game_speed", last);
    }
    {
        // through the converter: the held query keeps the stall rule from resetting to the dipped game velocity
        Run r;
        sc::State cs;
        sc::Limits lim;
        lim.friction = 2.0f;
        lim.maxTurnCos = std::cos(150.0f / 57.29578f);
        float minQ = 1.0e9f;
        for (int k = 0; k < 30; ++k)
            sc::Step(cs, r.Feed(238.0f), true, true, kDt, lim, false, false);
        V3 q = sc::Step(cs, r.Feed(90.0f, 0.10f), true, true, kDt, lim, false, false);
        minQ = std::fmin(minQ, SizeXY(q));
        for (int k = 0; k < 10; ++k)
        {
            q = sc::Step(cs, r.Feed(90.0f), true, true, kDt, lim, false, false);
            minQ = std::fmin(minQ, SizeXY(q));
        }
        Check(minQ > 230.0f && cs.stallSpeed <= 0.0f, "converter_no_stall_on_step", minQ);
    }
    std::printf("%s speedconv terrain (%d failed)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail;
}

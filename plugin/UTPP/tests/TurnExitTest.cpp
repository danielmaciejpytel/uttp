// Quick straightening after turns (mm/TurnExit.hpp, setting mm.fastTurnExit): the end-of-turn detector, the arc to straight loop pair
// and the exit rate of the body follow law (mm/BodyTurn.hpp).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

#include "mm/BodyTurn.hpp"
#include "mm/TurnExit.hpp"

namespace te = utpp::mm::turnexit;

namespace
{
int g_fail = 0;
void Check(bool aOk, const char* aWhat, double aValue = 0.0)
{
    std::printf("%s %s (%.4f)\n", aOk ? "PASS" : "FAIL", aWhat, aValue);
    g_fail += aOk ? 0 : 1;
}

struct Trace
{
    int exitsWhileTurning = 0;
    float openAfterStopS = -1.0f, closeAfterStopS = -1.0f, rate = 0.0f;
    uint64_t exits = 0;
};

// run straight 1 s, turn at aRate for aTurnS, run straight 1 s; aEveryOther: the heading moves on every second feed only (mouse
// counts at a high frame rate)
Trace Turn(float aHz, float aRate, float aTurnS, bool aEveryOther = false)
{
    te::Detector d;
    Trace out;
    const float dt = 1.0f / aHz;
    const int a = int(aHz), b = a + int(aTurnS * aHz + 0.5f), n = b + int(aHz);
    float yaw = 170.0f; // wraps past 180 on the way
    for (int i = 0; i < n; ++i)
    {
        if (i >= a && i < b)
        {
            if (!aEveryOther)
                yaw += aRate * dt;
            else if ((i - a) % 2 == 1)
                yaw += 2.0f * aRate * dt;
        }
        d.Step(dt, true, std::remainder(yaw, 360.0f));
        if (i < b && d.Exits() > 0)
            ++out.exitsWhileTurning;
        if (i >= b && d.Active() && out.openAfterStopS < 0.0f)
        {
            out.openAfterStopS = float(i - b + 1) * dt;
            out.rate = d.TurnRateDegS();
        }
        if (i >= b && !d.Active() && out.openAfterStopS >= 0.0f && out.closeAfterStopS < 0.0f)
            out.closeAfterStopS = float(i - b + 1) * dt;
    }
    out.exits = d.Exits();
    return out;
}
} // namespace

int main()
{
    for (const float hz : {30.0f, 60.0f, 144.0f})
        for (const float rate : {60.0f, 150.0f, -150.0f, 300.0f})
        {
            const Trace t = Turn(hz, rate, 0.6f);
            char what[160];
            std::snprintf(what, sizeof what, "%.0f Hz, %.0f deg/s: no exit while the turn holds", hz, rate);
            Check(t.exitsWhileTurning == 0, what, t.exitsWhileTurning);
            std::snprintf(what, sizeof what, "%.0f Hz, %.0f deg/s: one exit within 70 ms of the end of the turn", hz, rate);
            Check(t.exits == 1 && t.openAfterStopS >= 0.0f && t.openAfterStopS <= 0.07f, what, t.openAfterStopS);
            std::snprintf(what, sizeof what, "%.0f Hz, %.0f deg/s: the exit carries the turn rate within 15 %%", hz, rate);
            Check(std::fabs(t.rate - std::fabs(rate)) <= 0.15f * std::fabs(rate), what, t.rate);
            std::snprintf(what, sizeof what, "%.0f Hz, %.0f deg/s: the window closes 0.5 s later", hz, rate);
            Check(std::fabs(t.closeAfterStopS - t.openAfterStopS - 0.5f) <= 1.5f / hz, what, t.closeAfterStopS - t.openAfterStopS);
        }
    {
        const Trace t = Turn(144.0f, 120.0f, 1.0f, true);
        Check(t.exitsWhileTurning == 0 && t.exits == 1, "144 Hz, heading steps on every second feed: one exit, only at the end",
              double(t.exits));
    }
    {
        const Trace t = Turn(60.0f, 35.0f, 1.5f);
        Check(t.exits == 0, "a turn slower than the arm rate never opens the exit", double(t.exits));
    }
    {
        // a direction key pressed while running: the heading jumps in one feed
        te::Detector d;
        for (int i = 0; i < 180; ++i)
            d.Step(1.0f / 60.0f, true, i < 60 ? 0.0f : (i < 120 ? 90.0f : -90.0f));
        Check(d.Exits() == 0, "heading jumps (direction keys) never open the exit", double(d.Exits()));
    }
    {
        // a new turn during the window closes it at once; leaving ordinary running forgets everything
        te::Detector d;
        float yaw = 0.0f;
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 40; ++i)
            d.Step(dt, true, yaw += 150.0f * dt);
        for (int i = 0; i < 6; ++i)
            d.Step(dt, true, yaw);
        const bool open = d.Active();
        for (int i = 0; i < 4; ++i)
            d.Step(dt, true, yaw -= 150.0f * dt);
        Check(open && !d.Active(), "a new turn closes the window", open);
        for (int i = 0; i < 30; ++i)
            d.Step(dt, true, yaw -= 150.0f * dt);
        d.Step(dt, false, yaw);
        for (int i = 0; i < 10; ++i)
            d.Step(dt, true, yaw);
        Check(!d.Active(), "an ineligible feed forgets the turn", d.Active());
    }
    {
        using std::string_view;
        const string_view f = "M_Neutral_Run_Loop_F";
        Check(te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_F_L_20"), f) &&
                  te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_F_R_20"), f) &&
                  te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_FL"), f) &&
                  te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_FR"), f),
              "arc loops pair with the straight loop of the gait");
        Check(!te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_LL"), f) &&
                  !te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_F"), f) &&
                  !te::ArcToStraightLoop(string_view("M_Neutral_Walk_Loop_FL"), f) &&
                  !te::ArcToStraightLoop(string_view("M_Neutral_Run_Loop_FL"), string_view("M_Neutral_Run_Loop_FR")) &&
                  !te::ArcToStraightLoop(string_view("M_Neutral_Run_Pivot_F"), f),
              "no other pair skips the dwell");
    }
    {
        using std::string_view;
        Check(te::StraightForwardLoop(string_view("M_Neutral_Run_Loop_F")) &&
                  te::StraightForwardLoop(string_view("M_Neutral_Sprint_Loop_F")) &&
                  !te::StraightForwardLoop(string_view("M_Neutral_Run_Loop_FL")) &&
                  !te::StraightForwardLoop(string_view("M_Neutral_Run_Loop_F_L_20")) &&
                  !te::StraightForwardLoop(string_view("M_Neutral_Transition_Run_to_Sprint_Rfoot")),
              "only the straight forward loops are kept");
    }
    {
        // the follow law: exit rate 0 is the former command bit for bit; with the exit rate the error of a stopped 150 deg/s turn
        // closes within 100 ms and never passes the target
        using utpp::mm::bt::BodyYawController;
        constexpr float kRad = 3.14159265f / 180.0f;
        const float dt = 1.0f / 60.0f;
        BodyYawController a, b, c;
        float ya = 0.0f, yb = 0.0f, yc = 0.0f, target = 0.0f;
        bool same = true;
        for (int i = 0; i < 60; ++i)
        {
            target += 150.0f * kRad * dt;
            ya = a.Command(ya, target, dt, true);
            yb = b.Command(yb, target, dt, true, 0.0f, 0.0f, te::kBodyFollowS);
            yc = c.Command(yc, target, dt, true);
            same = same && std::memcmp(&ya, &yb, sizeof ya) == 0;
        }
        Check(same, "follow law: exit rate 0 is bit-identical");
        const float lag = (target - yc) / kRad;
        float t = 0.0f, past = 0.0f, tc = -1.0f;
        for (int i = 0; i < 30; ++i)
        {
            yc = c.Command(yc, target, dt, true, 0.0f, 150.0f * te::kBodyRateGain * kRad, te::kBodyFollowS);
            t += dt;
            past = std::fmax(past, (yc - target) / kRad);
            if (tc < 0.0f && std::fabs(target - yc) < 0.5f * kRad)
                tc = t;
        }
        Check(lag > 15.0f && tc >= 0.0f && tc <= 0.1f + 1e-4f, "exit rate: the remaining lag closes within 100 ms", tc);
        Check(past <= 1e-3f, "exit rate: never past the target", past);
    }
    std::printf("turn exit: %d failed\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}

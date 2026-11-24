// Speed converter, gait ramp (speedconv::GaitStep, MapConfig::speedConverter bit kGaitRamp): a gait change ramps the converted speed
// along the reference acceleration / braking, the ramp hands off to the game speed when it reaches it or after kGaitRampMaxS, held
// speeds and inactive feeds pass unchanged, the heading of the converted velocity is kept.
// Mutations (UTPP_SPEED_CONVERTER_MUTATION): 20 = no gait ramp; 21 = a slow-down ramps at the acceleration; 22 = no time hand-off;
// 23 = a ramp also starts inside a speed change; 24 = no confirmation (a stop's walk gait ramps).
#include <cmath>
#include <cstdio>
#include "../src/mm/speedconv_gait.hpp"

namespace
{
using utpp::mm::motiontraj::V3;
namespace sc = utpp::mm::speedconv;
int g_fail = 0;
void Check(bool aOk, const char* aWhat, double aGot, double aWant)
{
    if (!aOk)
    {
        ++g_fail;
        std::printf("FAIL %s: got %.3f want %.3f\n", aWhat, aGot, aWant);
    }
}
float Speed(V3 v) { return std::sqrt(v.x * v.x + v.y * v.y); }
constexpr float kDt = 1.0f / 60.0f;
constexpr float kAccel = 800.0f, kBraking = 500.0f;
} // namespace

int main()
{
    // held run: unchanged
    {
        sc::GaitState s;
        const V3 run{0.0f, 500.0f, 0.0f};
        float worst = 0.0f;
        for (int i = 0; i < 60; ++i)
            worst = std::fmax(worst, std::fabs(Speed(sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking)) - 500.0f));
        Check(worst < 1e-3f, "held run unchanged", worst, 0.0);
    }
    // walk 200 -> run 500 in one feed: the speed rises at 800 cm/s^2 and hands off after ~0.375 s
    {
        sc::GaitState s;
        const V3 walk{200.0f, 0.0f, 0.0f}, run{500.0f, 0.0f, 0.0f};
        for (int i = 0; i < 30; ++i)
            sc::GaitStep(s, walk, walk, 0, true, kDt, kAccel, kBraking);
        V3 out{};
        for (int i = 0; i < 6; ++i) // the ramp is used once the change is confirmed (0.1 s), from the speed before the change
            out = sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking);
        Check(std::fabs(Speed(out) - (200.0f + kAccel * 6 * kDt)) < 0.5f, "walk->run ramps after the confirmation", Speed(out),
              200.0f + kAccel * 6 * kDt);
        Check(std::fabs(out.y) < 1e-4f && out.x > 0.0f, "walk->run heading kept", out.y, 0.0);
        int feeds = 6;
        while (s.ramping && feeds < 200)
        {
            out = sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking);
            ++feeds;
        }
        const float t = feeds * kDt;
        Check(std::fabs(t - 0.375f) < 0.05f, "walk->run ramp time", t, 0.375);
        Check(std::fabs(Speed(out) - 500.0f) < 1e-3f, "walk->run hand-off at the game speed", Speed(out), 500.0);
    }
    // sprint 700 -> run 500 (sprint release): braking 500 cm/s^2 -> 0.4 s
    {
        sc::GaitState s;
        const V3 sprint{0.0f, 700.0f, 0.0f}, run{0.0f, 500.0f, 0.0f};
        for (int i = 0; i < 30; ++i)
            sc::GaitStep(s, sprint, sprint, 2, true, kDt, 300.0f, kBraking);
        V3 out{};
        int feeds = 0;
        float mid = 0.0f;
        do
        {
            out = sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking);
            if (++feeds == 12)
                mid = Speed(out);
        } while (s.ramping && feeds < 200);
        Check(std::fabs(mid - (700.0f - kBraking * 12 * kDt)) < 1.0f, "sprint release brakes at the input braking", mid,
              700.0f - kBraking * 12 * kDt);
        Check(std::fabs(feeds * kDt - 0.4f) < 0.05f, "sprint release ramp time", feeds * kDt, 0.4);
    }
    // inactive feed (air / no input) stops the ramp and passes the velocity
    {
        sc::GaitState s;
        const V3 walk{200.0f, 0.0f, 0.0f}, run{500.0f, 0.0f, 0.0f};
        for (int i = 0; i < 30; ++i)
            sc::GaitStep(s, walk, walk, 0, true, kDt, kAccel, kBraking);
        sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking);
        const V3 out = sc::GaitStep(s, run, run, 1, false, kDt, kAccel, kBraking);
        Check(!s.ramping && std::fabs(Speed(out) - 500.0f) < 1e-3f, "inactive feed hands off", Speed(out), 500.0);
    }
    // a game speed that keeps moving away hands off after kGaitRampMaxS
    {
        sc::GaitState s;
        const V3 walk{200.0f, 0.0f, 0.0f};
        for (int i = 0; i < 30; ++i)
            sc::GaitStep(s, walk, walk, 0, true, kDt, kAccel, kBraking);
        int feeds = 0;
        float game = 400.0f; // a step, then a game speed rising faster than the ramp
        do
        {
            game += 900.0f * kDt; // faster than the ramp
            const V3 g{game, 0.0f, 0.0f};
            sc::GaitStep(s, g, g, 1, true, kDt, kAccel, kBraking);
            ++feeds;
        } while (s.ramping && feeds < 600);
        Check(std::fabs(feeds * kDt - sc::kGaitRampMaxS) < 0.05f, "time hand-off", feeds * kDt, sc::kGaitRampMaxS);
    }
    // a stop: the stick passes the walk gait for a few feeds before the input is released -> no ramp is used
    {
        sc::GaitState s;
        const V3 run{500.0f, 0.0f, 0.0f};
        for (int i = 0; i < 30; ++i)
            sc::GaitStep(s, run, run, 1, true, kDt, kAccel, kBraking);
        float worst = 0.0f;
        for (int i = 1; i <= 4; ++i)
        {
            const V3 g{500.0f - 50.0f * i, 0.0f, 0.0f};
            worst = std::fmax(worst, std::fabs(Speed(sc::GaitStep(s, g, g, 0, i < 4, kDt, kAccel, kBraking)) - Speed(g)));
        }
        Check(worst < 1e-3f, "stop walk gait not ramped", worst, 0.0);
    }
    // a gait change inside a start (the game speed still rising) does not ramp
    {
        sc::GaitState s;
        float game = 50.0f;
        bool ramped = false;
        for (int i = 0; i < 40; ++i)
        {
            game += 1200.0f * kDt; // a start faster than the reference acceleration
            const V3 g{game, 0.0f, 0.0f};
            const V3 out = sc::GaitStep(s, g, g, game > 250.0f ? 1 : 0, true, kDt, kAccel, kBraking);
            ramped = ramped || std::fabs(Speed(out) - game) > 1e-3f;
        }
        Check(!ramped, "no ramp inside a start", ramped ? 1.0 : 0.0, 0.0);
    }
    if (g_fail)
        std::printf("%d check(s) failed\n", g_fail);
    else
        std::printf("speedconv gait: all checks passed\n");
    return g_fail ? 1 : 0;
}

// Speed converter through jumps and landings (speedconv::LandingFeed, speedconv::SeedLanding; MapConfig::speedConverter kLanding).
// The feeds are driven the way MotionAdapter drives them: LandingFeed first, then Step with the landing scope or the regular ground
// scope. No data files.
#include "../src/mm/speedconv_air.hpp"
#include <cmath>
#include <cstdio>

using namespace utpp::mm;
using motiontraj::V3;

static int passed = 0, failed = 0;
static void Check(const char* label, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    ok ? ++passed : ++failed;
}

static float HeadingDeg(V3 v) { return std::atan2(v.y, v.x) * 57.2957795f; }

namespace
{
constexpr float kDt = 1.0f / 60.0f;
const float kMaxTurnCos = std::cos(150.0f / motiontraj::kRad2Deg);

// one feed as the adapter runs it (stronger preset: speed not shaped, not direction only)
struct Feeder
{
    speedconv::State s;
    speedconv::LandingState l;
    speedconv::Limits lim;
    bool lastScope = false;
    Feeder()
    {
        lim.friction = 2.0f;
        lim.maxTurnCos = kMaxTurnCos;
    }
    V3 input{1.0f, 0.0f, 0.0f}; // input acceleration (direction)
    V3 Feed(V3 aGame, bool aAir, bool aGround = true, bool aRegular = false, bool aInput = true)
    {
        lastScope = speedconv::LandingFeed(l, s, aGame, aInput ? input : V3{}, aAir, aGround, aRegular, kDt, lim.maxTurnCos);
        const bool active = (lastScope && aInput) || aRegular;
        return speedconv::Step(s, aGame, aInput, active, kDt, lim, false, false);
    }
    void Fly(V3 aVel, float aSeconds)
    {
        for (float t = 0.0f; t < aSeconds; t += kDt)
            Feed(aVel, true, false);
    }
};
} // namespace

int main()
{
    // an air-control turn: the flight goes along +x, the capsule lands moving along +y (90 deg); the converted velocity keeps the
    // flight heading at touch-down and turns along the friction arc, at the game ground speed
    {
        Feeder f;
        f.input = V3{0.0f, 1.0f, 0.0f}; // the player steers to +y in the air
        f.Feed(V3{500.0f, 0.0f, 0.0f}, false, true, true); // running on the ground before the jump
        f.Fly(V3{500.0f, 0.0f, -100.0f}, 0.4f);
        const V3 land{0.0f, 400.0f, -300.0f};
        const V3 v = f.Feed(land, false);
        Check("landing: the landing scope converts the touch-down feed", f.lastScope);
        const float h = HeadingDeg(v);
        Check("landing: touch-down keeps the flight heading (turn part way, 2-45 deg)", h > 2.0f && h < 45.0f);
        const float sp = std::hypot(v.x, v.y);
        Check("landing: touch-down speed is at most the game ground speed (not the flight speed), less only by the turn friction",
              sp <= 400.5f && sp > 300.0f);
        Check("landing: the vertical velocity is the game's", v.z == land.z);
        V3 w = v;
        for (int i = 0; i < 17; ++i) // through the 0.3 s window
            w = f.Feed(V3{0.0f, 400.0f, 0.0f}, false);
        Check("landing: 0.3 s after touch-down the heading has turned further (still converted)", HeadingDeg(w) > h + 5.0f && f.lastScope);
        // the regular ground scope takes over without a restart (the state continues)
        const V3 r = f.Feed(V3{0.0f, 400.0f, 0.0f}, false, true, true);
        Check("landing: the regular scope closes the landing scope", !f.lastScope);
        Check("landing: the regular scope continues from the landing state (no restart at the game heading)",
              HeadingDeg(r) > HeadingDeg(w) && HeadingDeg(r) < 89.0f);
    }
    // the capsule snaps its speed on touch-down (1300 -> 600 cm/s): no stall reset, the converted velocity is the game's
    {
        Feeder f;
        f.Fly(V3{1300.0f, 0.0f, -200.0f}, 0.3f);
        const V3 v = f.Feed(V3{600.0f, 0.0f, 0.0f}, false);
        Check("landing snap: converted = game velocity (aligned), no stall", std::fabs(v.x - 600.0f) < 0.01f && std::fabs(v.y) < 0.01f &&
                                                                                f.s.stallSpeed == 0.0f);
        const V3 n = f.Feed(V3{620.0f, 0.0f, 0.0f}, false);
        Check("landing snap: the next feed follows the game speed", std::fabs(n.x - 620.0f) < 0.01f);
    }
    // touch-down spike (recorded: Female weapon session, running jump): the flight and the input point along (97, 213), the game
    // reports two feeds of a reversed velocity on the touch-down (the ground snap), then the run along the flight again: the converted
    // velocity stays along the flight at the game speed (no pivot), and the run continues without a snap
    {
        const float flight = HeadingDeg(V3{97.0f, 213.0f, 0.0f});
        Feeder f;
        f.input = V3{97.0f, 213.0f, 0.0f};
        f.Fly(V3{75.0f, 160.0f, -150.0f}, 0.2f);
        const V3 a = f.Feed(V3{-91.0f, -216.0f, -298.0f}, false);
        const V3 b = f.Feed(V3{-60.0f, -142.0f, -25.0f}, false);
        const V3 c = f.Feed(V3{97.0f, 213.0f, -77.0f}, false);
        Check("landing spike: the reversed touch-down feeds convert along the flight heading",
              std::fabs(HeadingDeg(a) - flight) < 2.0f && std::fabs(HeadingDeg(b) - flight) < 2.0f && f.l.spikes == 2);
        Check("landing spike: at the game ground speed", std::fabs(std::hypot(a.x, a.y) - std::hypot(91.0f, 216.0f)) < 1.0f);
        Check("landing spike: the run after the spike continues along the flight heading at the game speed",
              std::fabs(HeadingDeg(c) - flight) < 2.0f && std::fabs(std::hypot(c.x, c.y) - std::hypot(97.0f, 213.0f)) < 1.0f);
        // the same touch-down with the input turned along the game velocity is a real turn: not replaced
        Feeder g;
        g.input = V3{-91.0f, -216.0f, 0.0f};
        g.Fly(V3{75.0f, 160.0f, -150.0f}, 0.2f);
        const V3 d = g.Feed(V3{-91.0f, -216.0f, -298.0f}, false);
        Check("landing spike: a turned input is a real turn (the game velocity beyond the largest turn)",
              g.l.spikes == 0 && std::fabs(d.x + 91.0f) < 0.01f);
    }
    // a reversal beyond the converter's largest turn keeps the game velocity
    {
        Feeder f;
        f.input = V3{-1.0f, 0.0f, 0.0f}; // the player reverses in the air
        f.Fly(V3{500.0f, 0.0f, 0.0f}, 0.3f);
        const V3 v = f.Feed(V3{-400.0f, 0.0f, 0.0f}, false);
        Check("landing reversal: beyond the largest turn the game velocity", std::fabs(v.x + 400.0f) < 0.01f);
    }
    // the contact flag flickering (one air feed on the ground) is no landing; a standing landing passes the game velocity
    {
        Feeder f;
        f.Feed(V3{300.0f, 0.0f, 0.0f}, false, true, true);
        f.Feed(V3{300.0f, 50.0f, 0.0f}, true, false);
        f.Feed(V3{0.0f, 300.0f, 0.0f}, false, true, false);
        Check("flicker: a 1-feed air phase opens no landing scope", !f.lastScope);
        Feeder g;
        g.Fly(V3{0.0f, 0.0f, -200.0f}, 0.5f);
        const V3 v = g.Feed(V3{}, false, true, false, false);
        Check("standing landing: no input passes the game velocity", v.x == 0.0f && v.y == 0.0f);
    }
    // the scope ends after kLandingMaxWindowS without the regular scope, and leaving the ground (a slide) clears it
    {
        Feeder f;
        f.Fly(V3{500.0f, 0.0f, 0.0f}, 0.3f);
        f.Feed(V3{0.0f, 500.0f, 0.0f}, false);
        int feeds = 0;
        while (f.lastScope && feeds < 200)
        {
            f.Feed(V3{0.0f, 500.0f, 0.0f}, false);
            ++feeds;
        }
        Check("landing scope ends after 1 s at the latest", feeds * kDt > 0.9f && feeds * kDt < 1.1f);
        Feeder g;
        g.Fly(V3{500.0f, 0.0f, 0.0f}, 0.3f);
        g.Feed(V3{0.0f, 500.0f, 0.0f}, false);
        g.Feed(V3{0.0f, 500.0f, 0.0f}, false, false);
        Check("a slide (not eligible) clears the landing scope", !g.lastScope && !g.l.window);
    }
    std::printf("SpeedConv_air: %d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}

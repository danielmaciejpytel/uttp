// Fixed stick speed (CET mm.stickSpeed "fixed", MapConfig::stickSpeedFixed): adapter::FixedStickFraction (the speed the script holds the
// capsule at, with its x10 multiplier cap) and adapter::MapToChooser with the mode on / off: the predicted far speed is the held speed
// (stand walk below the stick 0.7, stand run from 0.7, the crouch speed in a crouch), the minimum analog walk speed is off there, and
// outside the gate (walk toggle, sprint) the mode changes nothing. No data files.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../src/mm/MotionAdapter.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
static int passed = 0, failed = 0;
#define CHECK(c, ...) do { if (c) ++passed; else { ++failed; std::printf("FAIL " __VA_ARGS__); std::printf("\n"); } } while (0)

struct Run
{
    std::vector<float> q; // every frame: chooser gait / state / speed and the query samples t -0.2 .. 1.5 s
    float farCmS = 0.f;
    int gait = -1;
};
// 60 Hz: 0.5 s rest, a 0.4 s capsule ramp to aSpeedMs along +Y with the stick (0, aStick), 3 s steady.
static Run RunSteady(bool aFixed, float aFloorMs, float aSpeedMs, float aStick, int aPsm, bool aWalkToggle = false)
{
    const float dt = 1.f / 60.f;
    adapter::MapConfig mc;
    mc.stickSpeedFixed = aFixed;
    adapter::SetAnalogFloor(mc, aFloorMs);
    if (aWalkToggle)
        mc.gameWalkTogglePoint = 1.6f;
    adapter::MapState ms;
    Run out;
    const int rest = 30, n = rest + int(3.4f / dt);
    for (int i = 0; i < n; ++i)
    {
        const float t = float(i - rest) * dt;
        const bool on = i >= rest;
        const float vel[3] = {0.f, on ? aSpeedMs * std::fmin(1.f, t / 0.4f) : 0.f, 0.f}, mv[2] = {0.f, on ? aStick : 0.f};
        adapter::MotionFeed f = adapter::MakeMotionFeed(vel, mv, dt, 0u, 0, adapter::AirFeed{}, 0.f, 0.f);
        f.psmDetailed = aPsm;
        f.wantsToWalk = aWalkToggle;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        const ChooserState cs = adapter::MapToChooser(f, mc, ms, ctx);
        out.q.insert(out.q.end(), {float(cs.gait), float(cs.movementState), cs.isStarting ? 1.f : 0.f, cs.speed2dCmS});
        for (int k = -2; k <= 15; ++k)
        {
            const TrajSample s = adapter::QuerySample(ms.traj, 0.1f * float(k));
            out.q.insert(out.q.end(), {s.x, s.y, s.z, s.yaw});
        }
        out.gait = cs.gait;
    }
    const TrajSample a = adapter::QuerySample(ms.traj, 1.4f), b = adapter::QuerySample(ms.traj, 1.5f);
    out.farCmS = std::hypot(b.x - a.x, b.y - a.y) * 100.f / 0.1f;
    return out;
}
static bool Same(const Run& a, const Run& b)
{
    return a.q.size() == b.q.size() && std::memcmp(a.q.data(), b.q.data(), a.q.size() * sizeof(float)) == 0;
}

int main()
{
    const adapter::MapConfig c;
    CHECK(!c.stickSpeedFixed, "MapConfig default: analog (stickSpeedFixed false)");

    // ---- the held fraction (of the stance's full-stick speed)
    const float walkFrac = adapter::kFixedStickWalkMs / 3.84f;
    CHECK(std::fabs(adapter::FixedStickFraction(c, 0, adapter::kPsmStand, 0.4f) - walkFrac * 3.85f / 3.85f) < 1e-5f,
          "stand |mv| 0.40: walk fraction 1.6 / 3.84 (%.5f)", adapter::FixedStickFraction(c, 0, adapter::kPsmStand, 0.4f));
    CHECK(adapter::FixedStickFraction(c, 0, adapter::kPsmStand, 0.7f) == 1.f && adapter::FixedStickFraction(c, 0, adapter::kPsmStand, 0.95f) == 1.f,
          "stand |mv| 0.70 / 0.95: the full-stick run");
    CHECK(adapter::FixedStickFraction(c, 1, adapter::kPsmCrouch, 0.3f) == 1.f, "crouch |mv| 0.30: the full-stick crouch speed");
    {
        const float fc = adapter::FixedStickFraction(c, 1, adapter::kPsmCrouch, 0.1f);
        const float want = 10.f * adapter::AnalogFloorStickFraction(0.1f);
        CHECK(std::fabs(fc - want) < 1e-5f && fc < 0.7f, "crouch |mv| 0.10: capped at 10 x the stick's own fraction (%.4f, want %.4f)", fc, want);
        const float fw = adapter::FixedStickFraction(c, 0, adapter::kPsmStand, 0.04f);
        CHECK(std::fabs(fw - 10.f * adapter::AnalogFloorStickFraction(0.04f)) < 1e-5f && fw < walkFrac,
              "stand |mv| 0.04: capped at 10 x the stick's own fraction (%.4f)", fw);
    }

    // ---- MapToChooser: the predicted far speed is the held speed (the capsule runs at it in fixed mode)
    struct Case { float stick; int psm; float ms; int stance; const char* name; };
    const Case cases[] = {{0.4f, adapter::kPsmStand, 3.85f * walkFrac, 0, "stand walk |mv| 0.40"},
                          {0.2f, adapter::kPsmStand, 3.85f * walkFrac, 0, "stand walk |mv| 0.20"},
                          {0.8f, adapter::kPsmStand, 3.85f, 0, "stand run |mv| 0.80"},
                          {0.5f, adapter::kPsmCrouch, 2.7f, 1, "crouch |mv| 0.50"},
                          {0.1f, adapter::kPsmCrouch, 2.7f * 10.f * adapter::AnalogFloorStickFraction(0.1f), 1, "crouch |mv| 0.10 (capped)"}};
    for (const Case& k : cases)
    {
        const float mapped = k.ms * 100.f * adapter::SpeedScaleFor(c, k.stance, k.ms);
        const Run fixed = RunSteady(true, 0.f, k.ms, k.stick, k.psm);
        const Run analog = RunSteady(false, 0.f, k.ms, k.stick, k.psm);
        CHECK(std::fabs(fixed.farCmS - mapped) < 3.f, "%s fixed: predicted far speed %.1f cm/s = the held speed %.1f cm/s", k.name, fixed.farCmS, mapped);
        if (k.stick < 0.7f)
            CHECK(analog.farCmS < mapped - 20.f, "%s analog: the prediction follows the stick (%.1f cm/s, held %.1f)", k.name, analog.farCmS, mapped);
        CHECK(fixed.gait == (k.stance == 1 ? analog.gait : (k.stick < adapter::kFixedStickSplit ? 0 : 1)), "%s: gait split at the stick (%d)", k.name, fixed.gait);
    }

    // ---- the minimum analog walk speed is off in fixed mode (the script removes its multiplier there)
    {
        const float v = 3.85f * walkFrac;
        CHECK(Same(RunSteady(true, 0.82f, v, 0.3f, adapter::kPsmStand), RunSteady(true, 0.f, v, 0.3f, adapter::kPsmStand)),
              "fixed |mv| 0.30: the floor 0.82 changes nothing, every frame");
    }
    // ---- outside the gate the mode changes nothing: walk toggle, sprint
    CHECK(Same(RunSteady(true, 0.f, 1.0f, 0.5f, adapter::kPsmStand, true), RunSteady(false, 0.f, 1.0f, 0.5f, adapter::kPsmStand, true)),
          "walk toggle |mv| 0.50: fixed == analog, every frame");
    CHECK(Same(RunSteady(true, 0.f, 8.2f, 1.0f, adapter::kPsmSprint), RunSteady(false, 0.f, 8.2f, 1.0f, adapter::kPsmSprint)),
          "sprint: fixed == analog, every frame");

    std::printf("FixedStickSpeed: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

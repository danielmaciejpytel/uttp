// Minimum analog walk speed (CET mm.minWalkSpeed): adapter::AnalogWalkFloorMul (the MaxSpeed multiplier the script applies), its stick
// AnalogFloorStickFor (MapConfig::analogFloorStick, the same extended curve both ways) and adapter::MapToChooser with the floor on / off:
// identical at and above the floor stick (Stand, Crouch), and the predicted cap is the speed the lifted capsule really reaches. No data files.
// Mutations (analogfloor.cmake, each must fail its own check): 1 = no 1.03 threshold (the |mv| 0.45 check), 2 = no driver lift (the
// |mv| 0.090 prediction check), 3 = no x4 cap in the driver lift (the |mv| 0.040 prediction check), 4 = the stand-run fraction and cap
// under the walk toggle (the walk |mv| 0.150 prediction check), 5 = the stand-run cap 4 under the walk toggle (the walk |mv| 0.090 check).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>
#include "../src/mm/MotionAdapter.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
static int passed = 0, failed = 0;
#define CHECK(c, ...) do { if (c) ++passed; else { ++failed; std::printf("FAIL " __VA_ARGS__); std::printf("\n"); } } while (0)

// One MapToChooser run (60 Hz): 0.5 s rest, a 0.4 s ramp of the capsule to aSpeedMs along +Y with the stick (0, aStick), 3 s steady. Every
// frame keeps the chooser inputs and the query trajectory (t -0.2 .. 1.5 s); farCmS = predicted speed between 1.4 and 1.5 s of the last frame.
struct Frame
{
    int gait = 0, movementState = 0;
    bool isStarting = false, isPivoting = false;
    float speed2dCmS = 0.f;
    float fut[3] = {}, cur[3] = {};
    std::vector<float> q; // x, y, z, yaw per query sample
};
struct Run
{
    std::vector<Frame> frames;
    float farCmS = 0.f;
    bool moving = false;
};
static Run RunSteady(float aFloorMs, float aSpeedMs, float aStick, int aPsm, float aWalkPoint = 0.f)
{
    const float dt = 1.f / 60.f;
    adapter::MapConfig mc;
    adapter::SetAnalogFloor(mc, aFloorMs);
    mc.gameWalkTogglePoint = aWalkPoint; // > 0: the walk toggle is held on (MotionFeed::wantsToWalk) with this walk speed
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
        f.wantsToWalk = aWalkPoint > 0.f;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        const ChooserState cs = adapter::MapToChooser(f, mc, ms, ctx);
        Frame fr;
        fr.gait = cs.gait;
        fr.movementState = cs.movementState;
        fr.isStarting = cs.isStarting;
        fr.isPivoting = cs.isPivoting;
        fr.speed2dCmS = cs.speed2dCmS;
        fr.fut[0] = ms.st.trjFuture.x, fr.fut[1] = ms.st.trjFuture.y, fr.fut[2] = ms.st.trjFuture.z;
        fr.cur[0] = ms.st.trjCurrent.x, fr.cur[1] = ms.st.trjCurrent.y, fr.cur[2] = ms.st.trjCurrent.z;
        for (int k = -2; k <= 15; ++k)
        {
            const TrajSample q = adapter::QuerySample(ms.traj, 0.1f * float(k));
            fr.q.insert(fr.q.end(), {q.x, q.y, q.z, q.yaw});
        }
        out.frames.push_back(fr);
        out.moving = cs.movementState == 1;
    }
    const TrajSample a = adapter::QuerySample(ms.traj, 1.4f), b = adapter::QuerySample(ms.traj, 1.5f);
    out.farCmS = std::hypot(b.x - a.x, b.y - a.y) * 100.f / 0.1f;
    return out;
}
static bool Same(const Run& a, const Run& b)
{
    if (a.frames.size() != b.frames.size())
        return false;
    for (size_t i = 0; i < a.frames.size(); ++i)
    {
        const Frame &x = a.frames[i], &y = b.frames[i];
        if (x.gait != y.gait || x.movementState != y.movementState || x.isStarting != y.isStarting || x.isPivoting != y.isPivoting ||
            x.speed2dCmS != y.speed2dCmS || std::memcmp(x.fut, y.fut, sizeof x.fut) != 0 || std::memcmp(x.cur, y.cur, sizeof x.cur) != 0 ||
            x.q != y.q)
            return false;
    }
    return true;
}

int main()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float fracMin = adapter::AnalogFloorFraction(0.82f); // default setting
    CHECK(std::fabs(fracMin - 0.82f / 3.84f) < 1e-6f, "fracMin of 0.82 m/s = 0.82 / 3.84 (%.5f)", fracMin);
    CHECK(adapter::AnalogFloorFraction(0.f) == 0.f && adapter::AnalogFloorFraction(nan) == 0.f && adapter::AnalogFloorFraction(-0.1f) == 0.f &&
              adapter::AnalogFloorFraction(1.7f) == 0.f,
          "AnalogFloorFraction: 0 / NaN / negative / over 1.6 -> 0 (off)");

    // ---- exactly 1: released stick, at / above the floor stick, the 0.45 cluster (1.03 threshold), off, NaN
    CHECK(adapter::AnalogWalkFloorMul(0.f, fracMin) == 1.f && adapter::AnalogWalkFloorMul(adapter::kStickReleased, fracMin) == 1.f,
          "released stick (0, kStickReleased) -> 1");
    for (const float s : {0.455f, 0.5f, 0.55f, 0.64f, 0.73f, 0.82f, 1.0f, 1.2f})
        CHECK(adapter::AnalogWalkFloorMul(s, fracMin) == 1.f, "|mv| %.3f at / above the floor -> 1 (%.4f)", s, adapter::AnalogWalkFloorMul(s, fracMin));
    CHECK(adapter::AnalogWalkFloorMul(0.45f, fracMin) == 1.f, "|mv| 0.45 (measured cluster just under 0.455) -> 1, not ~1.01 (%.4f)",
          adapter::AnalogWalkFloorMul(0.45f, fracMin));
    CHECK(adapter::AnalogWalkFloorMul(0.09f, 0.f) == 1.f && adapter::AnalogWalkFloorMul(0.27f, adapter::AnalogFloorFraction(0.f)) == 1.f,
          "setting 0 (off) -> 1");
    CHECK(adapter::AnalogWalkFloorMul(nan, fracMin) == 1.f && adapter::AnalogWalkFloorMul(0.09f, nan) == 1.f &&
              adapter::AnalogWalkFloorMul(std::numeric_limits<float>::infinity(), fracMin) == 1.f,
          "NaN / inf input -> 1");

    // ---- measured points: |mv| 0.09 -> 0.24 m/s (x3.4 -> 0.82), |mv| 0.27 -> 0.41 m/s (x2.0)
    const float m09 = adapter::AnalogWalkFloorMul(0.09f, fracMin), m27 = adapter::AnalogWalkFloorMul(0.27f, fracMin);
    CHECK(std::fabs(m09 - 0.82f / 0.24f) < 0.01f, "|mv| 0.09 -> ~3.42 (%.4f)", m09);
    CHECK(m27 > 1.9f && m27 < 2.05f, "|mv| 0.27 -> ~2.0 (%.4f)", m27);

    // ---- decreasing on (0.09, 0.455), the lifted fraction is fracMin (+-1 %), never above 4
    float prev = 1e9f;
    bool decreasing = true, lifted = true, capped = true;
    float worst = 0.f;
    for (int i = 0; i <= 400; ++i)
    {
        const float s = 0.09f + (0.455f - 0.09f) * float(i) / 400.f;
        const float m = adapter::AnalogWalkFloorMul(s, fracMin);
        decreasing = decreasing && m <= prev;
        prev = m;
        if (m > 1.f)
        {
            const float err = std::fabs(adapter::AnalogFloorStickFraction(s) * m / fracMin - 1.f);
            worst = std::fmax(worst, err);
            lifted = lifted && err <= 0.01f;
        }
    }
    for (int i = 0; i <= 1000; ++i)
    {
        const float s = float(i) / 1000.f;
        for (const float ms : {0.1f, 0.82f, 1.6f})
            capped = capped && adapter::AnalogWalkFloorMul(s, adapter::AnalogFloorFraction(ms)) <= adapter::kAnalogFloorMulMax;
    }
    CHECK(decreasing, "non-increasing over (0.09, 0.455)");
    CHECK(lifted, "frac(|mv|) x mul = fracMin within 1 %% (worst %.4f)", worst);
    CHECK(capped, "never above 4 (0..1 stick, settings 0.1 / 0.82 / 1.6)");
    CHECK(adapter::AnalogWalkFloorMul(0.03f, fracMin) == adapter::kAnalogFloorMulMax, "below the measured range the cap 4 holds (|mv| 0.03)");

    // ---- the floor stick of the setting (MapConfig::analogFloorStick)
    CHECK(adapter::AnalogFloorStickFor(0.82f) == 0.455f, "AnalogFloorStickFor(0.82) = 0.455 exactly (%.6f)", adapter::AnalogFloorStickFor(0.82f));
    CHECK(adapter::AnalogFloorStickFor(0.f) == 0.f && adapter::AnalogFloorStickFor(nan) == 0.f && adapter::AnalogFloorStickFor(2.f) == 0.f,
          "AnalogFloorStickFor: off / NaN / out of range -> 0");
    // the same extended curve both ways (also below its 0.18 point, settings < 0.30 m/s)
    for (const float ms : {0.1f, 0.2f, 0.24f, 0.3f, 0.41f, 0.6f, 1.2f, 1.6f})
    {
        const float s = adapter::AnalogFloorStickFor(ms);
        CHECK(std::fabs(adapter::AnalogFloorStickFraction(s) * 3.84f - ms) < 1e-4f, "AnalogFloorStickFor(%.2f) = %.4f lands on the extended curve", ms, s);
    }
    CHECK(adapter::AnalogFloorStickFor(0.24f) == 0.09f, "AnalogFloorStickFor(0.24) = the measured 0.09 point (%.6f)", adapter::AnalogFloorStickFor(0.24f));

    // ---- MapToChooser (production path: MapToChooserCore through EffectiveMapConfig): floor on vs off
    for (const int psm : {adapter::kPsmStand, adapter::kPsmCrouch})
    {
        const char* stanceName = psm == adapter::kPsmCrouch ? "crouch" : "stand";
        const int stance = psm == adapter::kPsmCrouch ? 1 : 0;
        adapter::MapConfig off;
        const float full = adapter::GameFullStickSpeed(off, stance, false);
        // identity: |mv| at and above the floor stick, the capsule at the curve speed of the stick -> every frame bit for bit
        for (const float s : {0.455f, 0.55f, 0.64f, 0.73f, 0.82f, 1.0f})
        {
            const float v = full * adapter::GameStickSpeedFraction(s);
            const Run a = RunSteady(0.f, v, s, psm), b = RunSteady(0.82f, v, s, psm);
            CHECK(Same(a, b), "%s |mv| %.3f (%.2f m/s): MapToChooser floor on == floor off, every frame", stanceName, s, v);
        }
        // the lifted capsule: steady at the reachable speed, the predicted far speed (the cap) is that speed, not the floor's
        for (const float s : {0.09f, 0.04f})
        {
            const float reach = adapter::AnalogFloorLiftFraction(s, adapter::AnalogFloorFraction(0.82f));
            const float v = full * std::fmin(adapter::AnalogFloorFraction(0.82f), adapter::kAnalogFloorMulMax * adapter::AnalogFloorStickFraction(s));
            adapter::MapConfig mc;
            const float mapped = v * 100.f * adapter::SpeedScaleFor(mc, stance, v);
            const float floorMapped = full * adapter::AnalogFloorFraction(0.82f) * 100.f * adapter::SpeedScaleFor(mc, stance, full * adapter::AnalogFloorFraction(0.82f));
            const Run r = RunSteady(0.82f, v, s, psm);
            CHECK(r.moving && std::fabs(r.farCmS - mapped) < 3.f,
                  "%s |mv| %.3f floor: predicted far speed %.1f cm/s = the reachable capsule speed %.1f cm/s (floor %.1f, lift fraction %.4f)",
                  stanceName, s, r.farCmS, mapped, floorMapped, reach);
        }
    }

    // walk toggle (G, walk speed 1.6 m/s): the floor is the same m/s against the walk speed, with the cap of the same reachable m/s
    {
        const float walk = 1.6f, fw = adapter::AnalogFloorFractionWalk(0.82f, walk), capW = adapter::AnalogFloorMulMaxWalk(walk);
        CHECK(std::fabs(fw - 0.82f / 1.6f) < 1e-6f, "walk: fraction 0.82 / 1.6 (%.5f)", fw);
        CHECK(adapter::AnalogFloorFractionWalk(1.6f, 1.2f) == 1.f && adapter::AnalogFloorFractionWalk(0.f, walk) == 0.f &&
                  adapter::AnalogFloorFractionWalk(0.82f, 0.f) == 0.f && adapter::AnalogFloorFractionWalk(nan, walk) == 0.f &&
                  adapter::AnalogFloorFractionWalk(0.82f, nan) == 0.f,
              "walk: fraction capped at 1, 0 when off / invalid");
        CHECK(std::fabs(capW - 4.f * 3.84f / 1.6f) < 1e-4f && adapter::AnalogFloorMulMaxWalk(3.84f) == adapter::kAnalogFloorMulMax &&
                  adapter::AnalogFloorMulMaxWalk(0.2f) == adapter::kAnalogFloorMulMaxWalk && adapter::AnalogFloorMulMaxWalk(nan) == adapter::kAnalogFloorMulMax,
              "walk: cap 4 x 3.84 / walk (%.3f), bounded 4 .. 10", capW);
        // the capsule reaches the floor m/s: walk speed x fraction(stick) x multiplier (|mv| 0.15 and 0.3), and the same reachable m/s as the
        // stand run below the measured range (|mv| 0.04)
        for (const float s : {0.15f, 0.3f})
        {
            const float v = walk * adapter::AnalogFloorStickFraction(s) * adapter::AnalogWalkFloorMul(s, fw, capW);
            CHECK(std::fabs(v - 0.82f) < 0.01f, "walk |mv| %.2f: capsule %.3f m/s = floor 0.82", s, v);
        }
        const float vw = walk * adapter::AnalogFloorStickFraction(0.04f) * adapter::AnalogWalkFloorMul(0.04f, fw, capW);
        const float vr = 3.84f * adapter::AnalogFloorStickFraction(0.04f) * adapter::AnalogWalkFloorMul(0.04f, adapter::AnalogFloorFraction(0.82f));
        CHECK(std::fabs(vw - vr) < 1e-3f, "walk |mv| 0.04: capped at the stand run's reachable m/s (%.3f vs %.3f)", vw, vr);
        CHECK(adapter::AnalogWalkFloorMul(0.8f, fw, capW) == 1.f, "walk |mv| 0.8 (above the walk floor stick %.3f) -> 1", adapter::AnalogFloorStickForFraction(fw));
        // the driver under the walk map predicts the lifted walk speed (EffectiveMapConfig): far speed = the mapped floor speed
        adapter::MapConfig wc;
        wc.gameWalkTogglePoint = walk;
        for (const float s : {0.15f, 0.09f})
        {
            const float v = walk * std::fmin(fw, capW * adapter::AnalogFloorStickFraction(s));
            adapter::MapConfig e = wc;
            e.gameStandPoints[0] = walk;
            e.walkMapActive = true;
            const float mapped = v * 100.f * adapter::SpeedScaleFor(e, 0, v);
            const Run r = RunSteady(0.82f, v, s, adapter::kPsmStand, walk);
            CHECK(r.moving && std::fabs(r.farCmS - mapped) < 3.f, "walk |mv| %.3f floor: predicted far speed %.1f cm/s = the lifted walk speed %.1f cm/s",
                  s, r.farCmS, mapped);
        }
        // identity at / above the walk floor stick: floor on == floor off under the walk toggle
        for (const float s : {0.8f, 1.0f})
        {
            const float v = walk * adapter::GameStickSpeedFraction(s);
            CHECK(Same(RunSteady(0.f, v, s, adapter::kPsmStand, walk), RunSteady(0.82f, v, s, adapter::kPsmStand, walk)),
                  "walk |mv| %.2f: MapToChooser floor on == floor off, every frame", s);
        }
    }

    std::printf("AnalogWalkFloor: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

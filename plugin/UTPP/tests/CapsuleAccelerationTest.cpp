#include "../src/mm/MotionAdapter.hpp"
#include <cstdio>
#include <limits>
using namespace utpp::mm;
namespace {
int pass = 0, fail = 0;
void Check(bool ok, const char* name) { (ok ? pass : fail)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", name); }
bool Near(float a, float b, float tolerance = .02f) { return std::fabs(a-b) <= tolerance; }
float ReferenceDistance(const adapter::MapConfig& c, int stance, float speed, float cap, float end)
{
    double result = 0;
    constexpr int steps = 12000;
    const float dt = end / steps;
    for (int i = 0; i < steps; ++i) {
        const float v = std::min(cap, speed + c.capsuleAcceleration * (i+.5f)*dt);
        result += v * 100.f * adapter::SpeedScaleFor(c, stance, v) * dt;
    }
    return float(result);
}
adapter::MotionFeed Feed(float fps = 60.f)
{
    adapter::MotionFeed f;
    f.dt = 1.f/fps;
    f.psmDetailed = adapter::kPsmStand;
    return f;
}
void Warm(adapter::MapState& state, const adapter::MapConfig& c, adapter::MotionFeed f)
{
    f.move[0] = f.move[1] = 0;
    adapter::MapToChooser(f, c, state, {});
}
bool SameForecast(const adapter::MapState& a, const adapter::MapState& b, float dt)
{
    for (float time : {.1f, .3f, .5f}) {
        const auto pa = a.traj.PositionAt(dt + time), pb = b.traj.PositionAt(dt + time);
        if (!Near(pa.x, pb.x) || !Near(pa.y, pb.y) || !Near(pa.z, pb.z)) return false;
    }
    return true;
}
}
int main()
{
    const float native = accel::AppliedAcceleration(24.f, 3.91f/24.f, 3.91f);
    Check(Near(native, 3.91f, 1.e-5f), "observed base times applied multiplier, never target, supplies physical acceleration");
    Check(Near(accel::AppliedAcceleration(24, 3.91f/24, 3.75f), 3.91f), "REDscript retention tolerance keeps actual acceleration instead of intended target");
    Check(accel::AppliedAcceleration(24, 1, 3.91f) == 0, "first modifier insertion invalidates pre-change observation");
    Check(accel::AppliedAcceleration(24, .16f, 12) == 0, "modifier replacement invalidates old observation");
    Check(accel::AppliedAcceleration(24, .16f, 0) == 0, "modifier removal invalidates old observation");
    Check(accel::AppliedAcceleration(24, 1, 23.8f) == 0, "no existing modifier cannot use small-delta retention branch");
    Check(accel::AppliedAcceleration(24, 1, 0) == 24, "unmodified observed native acceleration admitted");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (float value : {0.f, -1.f, nan, std::numeric_limits<float>::infinity()})
        Check(accel::AppliedAcceleration(value, .16f, 3.91f) == 0, "invalid native stat fails closed");
    Check(accel::FreshAcceleration(native, .1f, .2f, true) == native, "same-domain fresh sample admitted");
    Check(accel::FreshAcceleration(native, .1f, .2f, false) == 0, "loco or PSM domain change rejects sample");
    for (float age : {-.01f, .201f, nan})
        Check(accel::FreshAcceleration(native, age, .2f, true) == 0, "future stale or invalid sample age rejected");

    adapter::MapConfig c;
    c.capsuleAcceleration = native;
    c.capsuleBrakingDeceleration = 24;
    adapter::MapConfig fallback = c; fallback.capsuleAcceleration = 0;
    // Recorded start ramp: 0.200753 s at 3.91 predicts .784944 m/s;
    // log release speed .789 m/s. Exact query integration crosses map knots.
    Check(Near(native*.200753f, .789f, .005f), "recorded first ramp agrees with actually applied stat");
    for (float fps : {30.f, 60.f, 144.f}) for (float speed : {0.f, .789f, 1.59f, 1.61f}) {
        auto f = Feed(fps); f.velWorld[1] = speed;
        adapter::MapState state; Warm(state, c, f); f.move[1] = 1;
        adapter::MapToChooser(f, c, state, {});
        Check(state.capsuleAccelModel == native, "first full-input ramp uses observed model at 30/60/144Hz");
        const auto origin = state.traj.PositionAt(f.dt);
        for (float t : {.1f, .2f, .5f, 1.5f})
            Check(Near(state.traj.PositionAt(f.dt+t).y-origin.y,
                       ReferenceDistance(c, 0, speed, 3.85f, t)), "trajectory distance matches independent mapped physical integral across knots and speed cap");
    }
    for (int stance : {adapter::kPsmCrouch, adapter::kPsmSprint}) {
        auto f = Feed(); f.psmDetailed = stance;
        adapter::MapState state; Warm(state, c, f); f.move[1] = 1;
        adapter::MapToChooser(f, c, state, {});
        const bool crouch = stance == adapter::kPsmCrouch;
        Check(state.capsuleAccelModel == native && Near(state.traj.PositionAt(f.dt+.5f).y-state.traj.PositionAt(f.dt).y,
            ReferenceDistance(c, crouch ? 1 : 0, 0, crouch ? 2.7f : 8.2f, .5f)), "stable crouch and sprint retain their own physical caps/calibration");
    }
    {
        auto f = Feed(); adapter::MapConfig uniform = c; uniform.speedScale = .7f;
        adapter::MapState state; Warm(state, uniform, f); f.move[1] = 1;
        adapter::MapToChooser(f, uniform, state, {});
        Check(state.capsuleAccelModel == native && Near(state.traj.PositionAt(f.dt+.5f).y,
            ReferenceDistance(uniform, 0, 0, 3.85f, .5f)), "explicit uniform speed calibration remains supported");
    }
    for (int guard = 0; guard < 9; ++guard) {
        auto f = Feed(); adapter::MapState state, old; Warm(state, c, f); Warm(old, fallback, f);
        f.move[1] = 1;
        if (guard == 0) f.move[1] = .5f;
        if (guard == 1) { f.psmDetailed = adapter::kPsmFall; f.air.valid = true; f.air.onGround = false; }
        if (guard == 2) f.aiming = true;
        if (guard == 3) f.playerYawDeg = 90;
        if (guard == 4) f.velWorld[1] = -1;
        if (guard == 5) f.dt = 0;
        if (guard == 6) { f.cameraYawDeg = 10; f.playerYawDeg = 10; }
        if (guard == 7) { f.velWorld[1] = 1; f.move[1] = 0; }
        if (guard == 8) f.velWorld[1] = 3.85f;
        adapter::MapToChooser(f, c, state, {}); adapter::MapToChooser(f, fallback, old, {});
        Check(state.capsuleAccelModel == 0 && SameForecast(state, old, f.dt),
              "analog air strafe pivot reversal dt0 camera-turn released or capped velocity preserve existing forecast");
        if (guard == 7) Check(state.capsuleBrakeModel == 24, "accepted native braking remains authoritative after release");
    }
    {
        auto f = Feed(); adapter::MapConfig blend = c; blend.stanceSpeedBlendTime = .2f;
        adapter::MapState state; Warm(state, blend, f); f.move[1] = 1; f.psmDetailed = adapter::kPsmCrouch;
        adapter::MapToChooser(f, blend, state, {});
        Check(state.capsuleAccelModel == 0, "stance calibration transition rejects scalar full-speed model");
    }
    {
        auto below = Feed(); below.velWorld[1] = 3.84999f;
        auto capped = below; capped.velWorld[1] = 3.85f;
        adapter::MapState a, b; Warm(a, c, below); Warm(b, c, capped);
        below.move[1] = capped.move[1] = 1;
        adapter::MapToChooser(below, c, a, {}); adapter::MapToChooser(capped, c, b, {});
        Check(a.capsuleAccelModel == native && b.capsuleAccelModel == 0 && SameForecast(a, b, capped.dt),
              "reaching speed cap is continuous when returning to existing steady-speed prediction");
        a.traj.Reset();
        Check(a.traj.CapsuleAcceleration() == 0, "trajectory reset clears active acceleration diagnostics");
    }
    {
        auto f = Feed(); adapter::MapState state, old; Warm(state, c, f); Warm(old, fallback, f);
        f.move[1] = 1;
        adapter::MapToChooser(f, c, state, {}); adapter::MapToChooser(f, fallback, old, {});
        const float at2 = state.traj.PositionAt(f.dt+.2f).y, at5 = state.traj.PositionAt(f.dt+.5f).y;
        std::printf("Recorded-start model: .2s %.6f->%.6fcm; .5s %.6f->%.6fcm\n",
                    old.traj.PositionAt(f.dt+.2f).y, at2, old.traj.PositionAt(f.dt+.5f).y, at5);
        Check(Near(at2, 9.775f) && std::fabs(old.traj.PositionAt(f.dt+.5f).y-at5)>10,
              "baseline first-ramp discrepancy removed without changing physical acceleration");
    }
    std::printf("CapsuleAccelerationTest: %d PASS, %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

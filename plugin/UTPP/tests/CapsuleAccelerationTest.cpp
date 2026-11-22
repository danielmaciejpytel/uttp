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
    // Capsule forecast (MapConfig::capsuleForecast): the applied-acceleration ramp beyond the straight full-stick domain.
    adapter::MapConfig forecast = c; forecast.capsuleForecast = true;
    {
        // a straight full-stick start is the same forecast with the option on
        auto f = Feed(); adapter::MapState on, off; Warm(on, forecast, f); Warm(off, c, f);
        f.move[1] = 1;
        adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, c, off, {});
        bool same = on.capsuleAccelModel == off.capsuleAccelModel;
        for (int k = 0; k <= motiontraj::kPredictionCount; ++k) {
            const auto a = on.traj.Prediction(k).pos, b = off.traj.Prediction(k).pos;
            same = same && a.x == b.x && a.y == b.y && a.z == b.z;
        }
        Check(same, "forecast on: straight full-stick start bit-identical to off");
    }
    for (float fps : {30.f, 60.f, 144.f}) {
        // body facing 90 deg away from the input: the capsule ramps along the input at the applied acceleration
        auto f = Feed(fps); f.playerYawDeg = 90;
        adapter::MapState on, off; Warm(on, forecast, f); Warm(off, c, f);
        f.move[1] = 1;
        adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, c, off, {});
        const auto origin = on.traj.PositionAt(f.dt);
        bool ok = on.capsuleAccelModel == native && off.capsuleAccelModel == 0;
        for (float t : {.1f, .3f, .7f, 1.5f}) { // sample times (between samples the trajectory interpolates linearly)
            const auto p = on.traj.PositionAt(f.dt+t);
            ok = ok && Near(p.y-origin.y, ReferenceDistance(c, 0, 0, 3.85f, t)) && Near(p.x-origin.x, 0.f);
        }
        Check(ok, "forecast on: start facing away follows the input at the applied acceleration (30/60/144 Hz)");
    }
    {
        // partial stick: the ramp keeps the applied acceleration up to the steady speed of that stick (the off forecast's cap)
        auto f = Feed(); adapter::MapState on, off; Warm(on, forecast, f); Warm(off, c, f);
        f.move[1] = .5f;
        adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, c, off, {});
        const auto origin = on.traj.PositionAt(f.dt);
        const float vOn = on.traj.VelocityBetween(f.dt+1.4f, f.dt+1.5f).y, vOff = off.traj.VelocityBetween(f.dt+1.4f, f.dt+1.5f).y;
        const float d1 = on.traj.PositionAt(f.dt+.1f).y-origin.y;
        std::printf("Partial-stick forecast: terminal %.2f (off %.2f) cm/s, first .1s %.3f cm\n", vOn, vOff, d1);
        Check(on.capsuleAccelModel == native && Near(d1, ReferenceDistance(c, 0, 0, 3.85f, .1f)) && Near(vOn, vOff, .5f),
              "forecast on: partial stick ramps at the applied acceleration to the cap of that stick");
        auto fast = f; fast.velWorld[1] = 3.f; // already above the steady speed of the half stick
        adapter::MapState a, b; Warm(a, forecast, fast); Warm(b, c, fast);
        adapter::MapToChooser(fast, forecast, a, {}); adapter::MapToChooser(fast, c, b, {});
        Check(a.capsuleAccelModel == 0 && SameForecast(a, b, fast.dt), "forecast on: capsule above the stick's cap keeps the existing forecast");
    }
    for (float fps : {30.f, 60.f, 144.f}) {
        // turning input: the camera turned since the last frame (controller yaw rate, idle generation clamp 100 deg/s); the ramp follows
        // the input turned by that rate. Independent integration: yaw(t) = yaw0 + rate t, speed min(cap, a t), mapped like the replay.
        auto f = Feed(fps); adapter::MapState on, off; Warm(on, forecast, f); Warm(off, c, f);
        f.move[1] = 1; f.cameraYawDeg = 1; f.playerYawDeg = 1;
        adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, c, off, {});
        const float rate = std::min(100.f, 1.f * fps); // deg/s
        float wx = 0, wy = 0; adapter::StickToWorld(0, 1, 1, wx, wy);
        const float yaw0 = std::atan2(wy, wx);
        const auto origin = on.traj.PositionAt(f.dt);
        bool ok = on.capsuleAccelModel == native && off.capsuleAccelModel == 0 && Near(on.traj.ControllerYawRate(), rate, .01f);
        for (float end : {.3f, .7f}) { // sample times
            double x = 0, y = 0;
            constexpr int steps = 14000;
            const float dt = end / steps;
            for (int i = 0; i < steps; ++i) {
                const float t = (i+.5f)*dt, v = std::min(3.85f, native*t);
                const float yaw = yaw0 + rate * t / 57.2957795f, q = v * 100.f * adapter::SpeedScaleFor(c, 0, v) * dt;
                x += std::cos(yaw) * q; y += std::sin(yaw) * q;
            }
            const auto p = on.traj.PositionAt(f.dt+end);
            std::printf("Turning forecast %gHz %.2fs: (%.2f, %.2f) reference (%.2f, %.2f) cm\n", fps, end, p.x-origin.x, p.y-origin.y, x, y);
            // the forecast steps 0.1 s along the mid-step direction (a chord of the turning, accelerating path): within 0.3 cm or 1 %
            const float tol = std::max(.3f, .01f * float(std::sqrt(x*x + y*y)));
            ok = ok && Near(p.x-origin.x, float(x), tol) && Near(p.y-origin.y, float(y), tol);
        }
        Check(ok, "forecast on: turning input follows the controller yaw rate at the applied acceleration (30/60/144 Hz)");
    }
    {
        // the option off keeps every guarded case on the existing forecast (the guard loop above); on, the air, aiming and dt 0 still do
        for (int guard = 0; guard < 3; ++guard) {
            auto f = Feed(); adapter::MapState on, old; Warm(on, forecast, f); Warm(old, fallback, f);
            f.move[1] = 1;
            if (guard == 0) { f.psmDetailed = adapter::kPsmFall; f.air.valid = true; f.air.onGround = false; }
            if (guard == 1) f.aiming = true;
            if (guard == 2) f.dt = 0;
            adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, fallback, old, {});
            Check(on.capsuleAccelModel == 0 && SameForecast(on, old, f.dt), "forecast on: air, aiming and dt 0 keep the existing forecast");
        }
    }
    const auto sameBits = [](const adapter::MapState& a, const adapter::MapState& b) {
        bool same = a.capsuleAccelModel == b.capsuleAccelModel;
        for (int k = 0; k <= motiontraj::kPredictionCount; ++k) {
            const auto pa = a.traj.Prediction(k).pos, pb = b.traj.Prediction(k).pos;
            same = same && pa.x == pb.x && pa.y == pb.y && pa.z == pb.z;
        }
        return same;
    };
    for (int guard = 0; guard < 4; ++guard) {
        // option off: facing away, partial stick, turning input and a partial-stick turn are bit for bit the forecast without a
        // native acceleration (the only forecast these inputs had before the option)
        auto f = Feed(); adapter::MapState off, old; Warm(off, c, f); Warm(old, fallback, f);
        f.move[1] = 1;
        if (guard == 0) f.playerYawDeg = 90;
        if (guard == 1) f.move[1] = .5f;
        if (guard >= 2) { f.cameraYawDeg = 10; f.playerYawDeg = 10; }
        if (guard == 3) f.move[1] = .5f;
        adapter::MapToChooser(f, c, off, {}); adapter::MapToChooser(f, fallback, old, {});
        Check(off.capsuleAccelModel == 0 && sameBits(off, old), "forecast off: facing away, partial stick and turning inputs unchanged bit for bit");
    }
    {
        // the cap held up by the minimum analog speed (raw cap below the query cap) and a cap the map cannot reach keep the existing forecast
        const float walkCap = 200.f;
        const auto ok = adapter::CapsuleTurningStartPrediction(c, 0, 0, walkCap, walkCap);
        const auto floor = adapter::CapsuleTurningStartPrediction(c, 0, 0, walkCap, walkCap - .2f);
        const auto unreachable = adapter::CapsuleTurningStartPrediction(c, 0, 0, 1.e5f, 1.e5f);
        const auto invalid = adapter::CapsuleTurningStartPrediction(c, 0, 0, nan, nan);
        Check(ok.Active() && ok.turning && Near(ok.QuerySpeed(ok.maxSpeed), walkCap, .1f), "forecast: inverse map finds the game speed of the query cap");
        Check(floor.acceleration == 0 && !floor.Active(), "forecast: cap from the minimum analog speed keeps the existing forecast");
        Check(unreachable.acceleration == 0 && invalid.acceleration == 0, "forecast: unreachable or invalid cap keeps the existing forecast");
        // only starts: below the start fraction of the steady speed the ramp runs, at or above it the existing forecast stays
        const float hi = ok.maxSpeed, frac = adapter::kCapsuleForecastStartFraction;
        const auto start = adapter::CapsuleTurningStartPrediction(c, 0, (frac - .02f) * hi, walkCap, walkCap);
        const auto loop = adapter::CapsuleTurningStartPrediction(c, 0, (frac + .02f) * hi, walkCap, walkCap);
        Check(start.Active() && loop.acceleration == 0, "forecast: start fraction admits starts and keeps loops on the existing forecast");
    }
    for (int stance : {adapter::kPsmStand, adapter::kPsmCrouch}) for (float scale : {0.f, .7f}) {
        // partial stick through the chooser: crouch and the explicit uniform calibration ramp to the off forecast's terminal speed
        auto f = Feed(); f.psmDetailed = stance;
        adapter::MapConfig on = forecast, off = c; on.speedScale = off.speedScale = scale;
        adapter::MapState a, b; Warm(a, on, f); Warm(b, off, f);
        f.move[1] = .5f;
        adapter::MapToChooser(f, on, a, {}); adapter::MapToChooser(f, off, b, {});
        const float vOn = a.traj.VelocityBetween(f.dt+1.4f, f.dt+1.5f).y, vOff = b.traj.VelocityBetween(f.dt+1.4f, f.dt+1.5f).y;
        std::printf("Partial-stick forecast stance %d scale %.1f: terminal %.2f (off %.2f) cm/s\n", stance, scale, vOn, vOff);
        Check(a.capsuleAccelModel == native && b.capsuleAccelModel == 0 && Near(vOn, vOff, .5f),
              "forecast on: partial stick in crouch and with uniform calibration ramps to the cap of that stick");
    }
    {
        // the partial-stick branch while turning: the ramp follows the controller yaw rate (the path bends towards the turn)
        auto f = Feed(); adapter::MapState on, still; Warm(on, forecast, f); Warm(still, forecast, f);
        f.move[1] = .5f;
        auto turn = f; turn.cameraYawDeg = 1; turn.playerYawDeg = 1;
        adapter::MapToChooser(turn, forecast, on, {}); adapter::MapToChooser(f, forecast, still, {});
        const auto o = on.traj.PositionAt(turn.dt), p = on.traj.PositionAt(turn.dt+.7f);
        const auto so = still.traj.PositionAt(f.dt), sp = still.traj.PositionAt(f.dt+.7f);
        const float len = std::hypot(p.x-o.x, p.y-o.y), slen = std::hypot(sp.x-so.x, sp.y-so.y);
        // the input itself is turned by 1 deg; the bend beyond that comes from the yaw rate (60 deg/s for .7 s)
        const float bendDeg = std::fabs(std::atan2(p.x-o.x, p.y-o.y) - std::atan2(sp.x-so.x, sp.y-so.y)) * 57.2957795f;
        std::printf("Partial-stick turning forecast: %.2f cm (straight %.2f), bend %.2f deg\n", len, slen, bendDeg);
        Check(on.capsuleAccelModel == native && len < slen && len > .9f * slen && bendDeg > 10.f,
              "forecast on: partial stick with a turning input ramps along the turning direction");
    }
    for (float speed : {2.f, 3.7f}) {
        // full-stick camera turn while moving: the moving generation data clamps the controller yaw rate to 0, so the turning ramp
        // never applies to a run or a sprint and the forecast is the one of the option off bit for bit
        auto f = Feed(); f.velWorld[1] = speed;
        adapter::MapState on, off; Warm(on, forecast, f); Warm(off, c, f);
        f.move[1] = 1; f.cameraYawDeg = 1; f.playerYawDeg = 1;
        adapter::MapToChooser(f, forecast, on, {}); adapter::MapToChooser(f, c, off, {});
        Check(on.traj.ControllerYawRate() == 0 && sameBits(on, off), "forecast on: camera turn while moving at full stick is the forecast of the option off");
    }
    std::printf("CapsuleAccelerationTest: %d PASS, %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

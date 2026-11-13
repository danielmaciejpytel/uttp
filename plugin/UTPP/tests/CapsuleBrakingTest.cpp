#include "../src/mm/MotionAdapter.hpp"
#include <cstdio>
#include <limits>
using namespace utpp::mm;
namespace {
int pass = 0, fail = 0;
void Check(bool ok, const char* name) { (ok ? pass : fail)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", name); }
bool Near(float a, float b, float tolerance = .015f) { return std::fabs(a - b) <= tolerance; }
// Independent numerical integration of the production speed calibration, not
// the new closed-form braking integral. Only the physical scalar law is shared.
float ReferenceDistance(const adapter::MapConfig& c, float weight, float speed, float end)
{
    const int steps = 20000;
    const float dt = end / steps;
    double distance = 0;
    for (int i = 0; i < steps; ++i) {
        const float v = std::max(0.f, speed - c.capsuleBrakingDeceleration * (i + .5f) * dt);
        distance += v * 100.f * adapter::BlendedSpeedScaleFor(c, weight, v) * dt;
    }
    return float(distance);
}
}
int main()
{
    adapter::MapConfig c; c.capsuleBrakingDeceleration = 24;
    for (float weight : {0.f, .4f, 1.f}) for (float speed : {0.f, .3f, 1.59f, 1.61f, 3.74f, 3.85f, 4.86f, 8.1f, 8.3f}) {
        const auto p = adapter::CapsuleStopPrediction(c, weight, speed);
        Check(Near(p.QuerySpeedAt(0), speed * 100 * adapter::BlendedSpeedScaleFor(c, weight, speed)), "initial query speed preserves stand/crouch/blended calibration");
        for (float t : {.025f, .1f, .2f, .5f})
            Check(Near(p.DistanceBetween(0, t), ReferenceDistance(c, weight, speed, t)), "mapped CP stop distance matches independent integration across map knots");
        const float stop = speed / 24;
        Check(p.QuerySpeedAt(stop + .001f) == 0 && p.DistanceBetween(stop + .001f, 2) == 0, "no velocity or extra displacement after physical stop");
    }
    c.speedScale = .7f;
    const auto uniform = adapter::CapsuleStopPrediction(c, .5f, 4.86f);
    Check(Near(uniform.DistanceBetween(0, 1), .5f * 4.86f * 4.86f / 24 * 70), "explicit uniform calibration remains honored");
    c.speedScale = 0;
    // Real landing speeds from session022647, with the normal prediction clock
    // offset by the nonzero frame dt (not just a synthetic dt=0 initial sample).
    for (float fps : {30.f, 60.f, 144.f})
    for (float speed : {3.740220f, 4.299251f, 3.999081f, 4.770181f, 4.858243f}) {
        adapter::MotionFeed f; f.dt = 1.f / fps; f.psmDetailed = adapter::kPsmStand; f.velWorld[0] = speed;
        adapter::MapState state; adapter::MapContext context;
        adapter::MapToChooser(f, c, state, context);
        for (float t : {.1f, .2f, .3f, .5f})
            Check(Near(state.traj.PositionAt(f.dt + t).x - state.traj.PositionAt(f.dt).x,
                ReferenceDistance(c, 0, speed, t)), "30/60/144Hz prediction clock preserves integrated stop for recorded landing speed");
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Check(brake::FreshDeceleration(24, .1f, .2f) == 24, "fresh stat admitted");
    for (float age : {-.01f, .201f, nan}) Check(brake::FreshDeceleration(24, age, .2f) == 0, "future/stale/invalid timestamp fails closed");
    for (float value : {0.f, -1.f, nan}) Check(brake::FreshDeceleration(value, 0, .2f) == 0, "unusable stat fails closed");

    adapter::MapContext context;
    for (int stance : {adapter::kPsmStand, adapter::kPsmCrouch}) {
        adapter::MotionFeed f; f.psmDetailed = stance; f.velWorld[0] = 4.86f; f.dt = 0;
        adapter::MapState state;
        adapter::MapToChooser(f, c, state, context);
        Check(state.capsuleBrakeModel == 24, "real adapter selects native no-input grounded braking");
        const auto p = adapter::CapsuleStopPrediction(c, stance == adapter::kPsmCrouch ? 1.f : 0.f, 4.86f);
        for (float t : {.1f, .2f, .3f, .4f})
            Check(Near(state.traj.PositionAt(t).x, p.DistanceBetween(0, t)), "trajectory prediction points follow physical stop and same speed map");
        Check(Near(state.traj.PositionAt(.3f).x, state.traj.PositionAt(.5f).x), "ground trajectory stays stopped after 4.86/24 seconds");
        f.velWorld[2] = 3.5f;
        adapter::MapState slope;
        adapter::MapToChooser(f, c, slope, context);
        Check(Near(slope.traj.PositionAt(.2f).x, state.traj.PositionAt(.2f).x), "grounded vertical velocity does not shorten horizontal stop direction");
        f.velWorld[2] = 0;
        adapter::MapConfig legacy = c; legacy.capsuleBrakingDeceleration = 0;
        adapter::MapState old;
        adapter::MapToChooser(f, legacy, old, context);
        Check(std::fabs(old.traj.PositionAt(.3f).x - state.traj.PositionAt(.3f).x) > 5, "legacy Neutral model reproduces meaningful physical stop mismatch");
        f.move[1] = 1;
        adapter::MapState moving, movingOld;
        adapter::MapToChooser(f, c, moving, context); adapter::MapToChooser(f, legacy, movingOld, context);
        Check(moving.capsuleBrakeModel == 0 && Near(moving.traj.PositionAt(.4f).x, movingOld.traj.PositionAt(.4f).x), "held-input motion unchanged");
        f.move[1] = adapter::kStickReleased + .0001f;
        adapter::MapState edge, edgeOld;
        adapter::MapToChooser(f, c, edge, context); adapter::MapToChooser(f, legacy, edgeOld, context);
        Check(edge.capsuleBrakeModel == 0 && Near(edge.traj.PositionAt(.4f).x, edgeOld.traj.PositionAt(.4f).x), "input just above existing deadzone remains on held-input model");
        f.move[1] = adapter::kStickReleased;
        adapter::MapState deadzone;
        adapter::MapToChooser(f, c, deadzone, context);
        Check(deadzone.capsuleBrakeModel == 24, "existing input deadzone remains released");
        f.move[1] = 0; f.psmDetailed = adapter::kPsmFall; f.air.valid = true; f.air.onGround = false;
        adapter::MapState airborne, airborneOld;
        adapter::MapToChooser(f, c, airborne, context); adapter::MapToChooser(f, legacy, airborneOld, context);
        Check(airborne.capsuleBrakeModel == 0 && Near(airborne.traj.PositionAt(.4f).x, airborneOld.traj.PositionAt(.4f).x), "air motion unchanged even with native stat");
        f.psmDetailed = stance; f.air.onGround = true;
        adapter::MapToChooser(f, c, airborne, context);
        Check(airborne.capsuleBrakeModel == 24, "landing admits native ground braking immediately");
        for (float invalid : {0.f, -1.f, nan}) {
            adapter::MapConfig absent = c; absent.capsuleBrakingDeceleration = invalid;
            adapter::MapState fallback, reference;
            adapter::MapToChooser(f, absent, fallback, context); adapter::MapToChooser(f, legacy, reference, context);
            Check(fallback.capsuleBrakeModel == 0 && Near(fallback.traj.PositionAt(.4f).x, reference.traj.PositionAt(.4f).x), "missing/stale/invalid native model preserves Neutral fallback");
        }
    }
    std::printf("CapsuleBraking: %d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

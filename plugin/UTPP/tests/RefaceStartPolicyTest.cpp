#include "../src/mm/RefaceStart.hpp"
#include "../src/mm/BodyTurn.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace rs = utpp::mm::refacestart;
namespace bt = utpp::mm::bt;
static int checks = 0, failures = 0;
static void Check(bool value, const char* name, int hz)
{
    ++checks;
    if (!value) { ++failures; std::printf("FAIL %d Hz: %s\n", hz, name); }
}
static bool Near(float a, float b, float eps = 0.00001f) { return std::fabs(a - b) <= eps; }
static rs::Episode Armed(float dt, int sign = 1)
{
    rs::Episode e;
    e.Step(dt, true, true, true, 0, 0, sign * 179.5f * rs::kDeg, 1);
    return e;
}

static void Lifecycle(int hz)
{
    const float dt = 1.0f / hz, target = 179.5f * rs::kDeg;
    auto e = Armed(dt);
    Check(e.direction == 1 && e.clip == -1, "initial left direction", hz);
    Check(Armed(dt, -1).direction == -1, "initial right direction", hz);
    e.Selected(397, 1);
    Check(e.clip == 397 && e.direction == 1, "matching selected clip commits", hz);
    e.Step(dt, true, true, true, 3.85f, 0, -179.5f * rs::kDeg, 1);
    Check(e.direction == 1 && e.clip == 397, "180-degree wrap and acceleration preserve episode", hz);
    e.Step(dt, true, true, true, 3.85f, 0, target + 19 * rs::kDeg, 1);
    Check(e.direction == 1, "goal variation below 20 degrees preserves episode", hz);
    e.Step(dt, true, true, true, 3.85f, 0, target + 21 * rs::kDeg, 1);
    Check(!e.direction && e.clip == -1, "changed goal releases episode", hz);
    e.Step(dt, true, true, true, 0, 0, target, 1);
    Check(!e.direction, "held input cannot rearm after release", hz);
    e.Step(dt, true, false, true, 0, 0, target, 1);
    e.Step(dt, true, true, true, 0, 0, target, 1);
    Check(e.direction == 1, "new input edge rearms", hz);

    e = Armed(dt); e.Step(dt, true, false, true, 0, 0, target, 1);
    Check(!e.direction, "input release", hz);
    e = Armed(dt); e.Step(dt, true, true, false, 0, 0, target, 1);
    Check(!e.direction, "start completed", hz);
    e = Armed(dt); e.Step(dt, false, true, true, 0, 0, target, 1);
    Check(!e.direction, "ineligible air or crouch releases", hz);
    e.Step(dt, true, true, true, 0, 0, target, 1);
    Check(!e.direction, "landing or stance return does not consume held input as new edge", hz);
    e = Armed(dt); e.Step(dt, true, true, true, 0, 0, target, 2);
    Check(!e.direction, "gait change releases", hz);
    e = Armed(dt); e.Step(0.251f, true, true, true, 0, 0, target, 1);
    Check(!e.direction, "feed gap releases", hz);
    e = Armed(dt);
    for (int n = 0; n < hz * 2 + 2; ++n) e.Step(dt, true, true, true, 0, 0, target, 1);
    Check(!e.direction, "two second TTL releases", hz);
    e.Reset(); e.Step(0, true, true, true, 0, 0, target, 1);
    Check(!e.direction && !e.previousInput, "zero dt does not consume initial edge", hz);
    e.Step(dt, true, true, true, 0, 0, target, 1);
    Check(e.direction == 1, "first real frame admits after zero dt", hz);
    e.Selected(397, 1); const float elapsed = e.elapsed;
    e.Step(0, true, true, true, 0, 0, target, 1);
    Check(e.direction == 1 && e.clip == 397 && e.previousInput && e.elapsed == elapsed,
          "zero dt preserves valid episode without advancing time", hz);
    for (int cancel = 0; cancel < 5; ++cancel)
    {
        e = Armed(dt); e.Selected(397, 1);
        e.Step(0, cancel != 4, cancel != 0, cancel != 1, 0, 0,
               cancel == 3 ? target + 21 * rs::kDeg : target, cancel == 2 ? 2 : 1);
        Check(!e.direction && e.clip == -1, "zero dt still cancels on release/start/goal/gait/eligibility", hz);
    }

    e.Reset(); e.Step(dt, true, true, true, .501f, 0, target, 1);
    Check(!e.direction, "already moving cannot admit", hz);
    e.Reset(); e.Step(dt, true, true, true, .5f, 0, target, 1);
    Check(e.direction == 1, "0.5 m/s admission boundary", hz);
    e.Reset(); e.Step(dt, true, true, true, 0, 0, 90 * rs::kDeg, 1);
    Check(!e.direction, "normal 90-degree turn unaffected", hz);
    e.Reset(); e.Step(dt, true, true, true, 0, 0, target, 0);
    Check(!e.direction, "walk not admitted", hz);
    e.Reset(); e.Step(dt, true, true, true, 0, 0, target, 2);
    Check(e.direction == 1, "sprint can admit", hz);
    e = Armed(dt); e.Selected(399, -1);
    Check(!e.direction && e.clip == -1, "opposite selected direction releases", hz);
    e = Armed(dt); e.Selected(-1, 1);
    Check(!e.direction && e.clip == -1, "invalid selected clip releases", hz);
    e = Armed(dt); e.Selected(20, 0);
    Check(!e.direction, "non-reface selection releases", hz);
    e = Armed(dt); e.Reset();
    Check(!e.direction && e.clip == -1 && !e.previousInput, "session reset clears edge and clip", hz);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int fault = 0; fault < 5; ++fault)
    {
        e = Armed(dt);
        e.Step(fault == 0 ? nan : fault == 1 ? -dt : dt, true, true, true,
               fault == 2 ? nan : 0, fault == 3 ? nan : 0, fault == 4 ? nan : target, 1);
        Check(!e.direction && e.clip == -1, "invalid feed releases safely", hz);
    }
}

struct TurnResult { float finalError, travel, atHalfSecond; };
static TurnResult Controller(int hz, int sign)
{
    const float dt = 1.0f / hz, desired = sign * 179.5f * rs::kDeg;
    rs::Episode e = Armed(dt, sign);
    bt::BodyYawController control;
    float actor = 0, travel = 0, atHalfSecond = 0;
    // The shortest-angle answer flips at +/-180 before the capsule has rotated.
    const float jittered = -desired;
    const float protectedTarget = rs::ControllerTarget(actor, jittered, e.direction);
    bt::BodyYawController unprotected;
    const float wrong = bt::Wrap(unprotected.Command(actor, jittered, dt) - actor);
    const float protectedStep = bt::Wrap(control.Command(actor, protectedTarget, dt) - actor);
    Check(wrong * sign < 0 && protectedStep * sign > 0,
          "regression: controller and selected reface agree across ambiguous half-turn", hz);
    control.Reset();
    bool allForward = true, allFinite = true, rateBounded = true;
    for (int frame = 0; frame < hz * 2; ++frame)
    {
        const float time = static_cast<float>(frame) / hz;
        const float goal = time < .1f && (frame % 2) ? jittered : desired;
        e.Step(dt, true, true, time < 1.5f, frame ? 3.85f : 0, actor, goal, 1);
        control.FeedAnimRate(time < .4f ? sign * 260.0f * rs::kDeg : 0, dt);
        const float next = control.Command(actor, rs::ControllerTarget(actor, goal, e.direction), dt);
        const float step = bt::Wrap(next - actor);
        allForward = allForward && step * sign >= -0.00001f;
        allFinite = allFinite && std::isfinite(next);
        rateBounded = rateBounded && std::fabs(step) <= control.Config().maxRate * dt + .00001f;
        travel += std::fabs(step);
        actor = next;
        if (frame + 1 == hz / 2) atHalfSecond = actor;
    }
    Check(allForward && allFinite && rateBounded, "whole turn monotonic finite and rate bounded", hz);
    Check(std::fabs(bt::Wrap(desired - actor)) < .1f * rs::kDeg, "whole turn converges", hz);
    Check(travel < 190.0f * rs::kDeg, "whole turn has no added revolution", hz);

    // A delayed capsule sample already beyond the goal must use a short correction.
    const float overshot = bt::Wrap(desired + sign * 1.0f * rs::kDeg);
    const float correctedTarget = rs::ControllerTarget(overshot, desired, sign);
    control.Reset(); control.FeedAnimRate(sign * 260.0f * rs::kDeg, dt);
    const float correction = bt::Wrap(control.Command(overshot, correctedTarget, dt) - overshot);
    Check(Near(correctedTarget, desired) && correction * sign < 0 && std::fabs(correction) <= 1.001f * rs::kDeg,
          "overshoot corrects one degree and never forces a full turn", hz);
    Check(Near(rs::ControllerTarget(0, desired, 0), desired) &&
          Near(rs::ControllerTarget(0, desired, 2), desired), "inactive or invalid direction leaves target unchanged", hz);
    return {std::fabs(bt::Wrap(desired - actor)), travel, atHalfSecond};
}

int main()
{
    std::array<TurnResult, 3> left{}, right{};
    const std::array<int, 3> rates{30, 60, 144};
    for (size_t i = 0; i < rates.size(); ++i)
    {
        Lifecycle(rates[i]); left[i] = Controller(rates[i], 1); right[i] = Controller(rates[i], -1);
        Check(std::fabs(left[i].travel - right[i].travel) < .01f * rs::kDeg,
              "mirrored directions have equivalent travel", rates[i]);
        std::printf("%d Hz: left/right travel %.4f/%.4f deg, final error %.6f/%.6f deg, at 0.5 s %.4f/%.4f deg\n",
                    rates[i], left[i].travel / rs::kDeg, right[i].travel / rs::kDeg,
                    left[i].finalError / rs::kDeg, right[i].finalError / rs::kDeg,
                    left[i].atHalfSecond / rs::kDeg, right[i].atHalfSecond / rs::kDeg);
    }
    for (size_t i = 0; i < 2; ++i)
        Check(std::fabs(left[i].atHalfSecond - left[2].atHalfSecond) < 5.0f * rs::kDeg,
              "frame-rate variation at 0.5 seconds stays under 5 degrees", rates[i]);
    std::printf("RefaceStartPolicy: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

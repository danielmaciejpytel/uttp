#include "../src/mm/MotionSteering.hpp"
#include <cstdio>
#include <limits>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 1e-5f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
int main()
{
    using motionSteer::Inputs;
    using motionSteer::Steering;
    Steering s;
    Inputs i;
    i.dt = .016f;
    i.rootSpeedMs = 1;
    i.animationYawDelta = .02f;
    i.targetWorldYaw = 1;
    i.predictedAuthoredYaw = .5f;
    i.alpha = 0;
    C("alpha0 leaves authored delta and state unchanged",
      s.Update(i).yawDelta == i.animationYawDelta && s.AngularVelocity() == 0);
    i.alpha = 1;
    i.enabled = false;
    C("disabled bypass", !s.Update(i).active && s.AngularVelocity() == 0);
    i.enabled = true;
    i.rootSpeedMs = .1f;
    C("exact10cm per second threshold inactive", !s.Update(i).active);
    i.rootSpeedMs = .10001f;
    C("above10cm per second activates", s.Update(i).active);
    s.Reset();
    i.rootSpeedMs = 1;
    i.turnInPlace = true;
    auto hi = s.Update(i);
    C("authored scaling upper clamp", Near(hi.authoredRatio, 1.5f));
    i.targetWorldYaw = -1;
    auto lo = s.Update(i);
    C("opposing target does not invert authored turn", Near(lo.authoredRatio, .5f) && lo.yawDelta > 0);
    i.animationYawDelta = -.02f;
    i.predictedAuthoredYaw = -.5f;
    i.targetWorldYaw = -1;
    C("negative authored turn ratio keeps sign", Near(s.Update(i).authoredRatio, 1.5f));
    i.predictedAuthoredYaw = 0;
    C("zero predicted turn bypasses division", s.Update(i).authoredRatio == 1);
    i.predictedAuthoredYaw = math::kPi / 180.f;
    C("exact1deg prediction threshold does not scale", s.Update(i).authoredRatio == 1);
    // Independent closed form first critically-damped step from identity/zero velocity.
    s.Reset();
    i = {};
    i.dt = .02f;
    i.rootSpeedMs = 1;
    i.targetWorldYaw = .8f;
    i.havePrediction = false;
    i.animationYawDelta = 0;
    const auto first = s.Update(i);
    const double y = 2 * std::log(2.) / .2;
    const double oracle = .8 * (1 - (1 + y * .02) * std::exp(-y * .02));
    C("procedural spring first step matches independent exponential oracle",
      Near(first.proceduralYaw, float(oracle), .0006f));
    C("angular velocity units are radians per second", s.AngularVelocity() > 0 && s.AngularVelocity() < 2);
    Steering tip;
    i.turnInPlace = true;
    const auto negligible = tip.Update(i);
    C("TIP procedural1e6 differs from general .2",
      std::fabs(negligible.proceduralYaw) < 1e-6 && first.proceduralYaw > .001);
    i.rootSpeedMs = 0;
    C("TIP steering permits stationary authored turn", tip.Update(i).active);
    i.turnInPlace = false;
    C("general steering rejects stationary", !s.Update(i).active);
    s.Reset();
    i = {};
    i.dt = .016f;
    i.turnInPlace = true;
    i.rootSpeedMs = 1;
    i.predictedAuthoredYaw = 170 * math::kPi / 180;
    i.targetWorldYaw = -170 * math::kPi / 180;
    i.animationYawDelta = .05f;
    C("plusminus180 chooses nearest authored endpoint branch", Near(s.Update(i).authoredRatio, 190.f / 170.f));
    const float v = s.AngularVelocity();
    i.dt = 0;
    auto zero = s.Update(i);
    C("dt0 exact authored no-op and state continuity",
      zero.yawDelta == i.animationYawDelta && s.AngularVelocity() == v);
    i.dt = .02f;
    i.alpha = .5f;
    i.targetWorldYaw = 1;
    i.predictedAuthoredYaw = .5f;
    C("alpha scales authored ratio from identity", Near(s.Update(i).authoredRatio, 1.25f));
    Steering a, b;
    for (int n = 0; n < 60; ++n)
    {
        Inputs f;
        f.dt = .013f;
        f.rootSpeedMs = .5f;
        f.previousVisualWorldYaw = .01f * n;
        f.targetWorldYaw = .8f + .015f * n;
        f.predictedAuthoredYaw = .12f;
        f.animationYawDelta = .003f;
        auto x = a.Update(f);
        f.previousVisualWorldYaw += 2.3f;
        f.targetWorldYaw += 2.3f;
        auto z = b.Update(f);
        C("global yaw-frame invariance",
          Near(x.yawDelta, z.yawDelta) && Near(a.AngularVelocity(), b.AngularVelocity()));
    }
    s.Reset();
    C("reset alpha uninitialized output zero", s.Alpha() == 0);
    C("first true dt0 snaps one", s.UpdateEnable(true, 0) == 1);
    s.Reset();
    C("first false dt0 snaps zero", s.UpdateEnable(false, 0) == 0);
    C("subsequent fade half50ms", Near(s.UpdateEnable(true, .05f), .5f));
    C("subsequent fade endpoint100ms", s.UpdateEnable(true, .05f) == 1);
    C("target reversal dt0 preserves current output", s.UpdateEnable(false, 0) == 1);
    C("fade out50ms", Near(s.UpdateEnable(false, .05f), .5f));
    C("fade out endpoint", s.UpdateEnable(false, .05f) == 0);
    s.Reset();
    s.UpdateEnable(false, 0);
    C("partial rising .04 seconds", Near(s.UpdateEnable(true, .04f), .4f));
    C("partial reversal uses full .1 new range oracle", Near(s.UpdateEnable(false, .02f), .4f * (1.f - .02f / .1f)));
    C("second reversal rebases from .32 oracle", Near(s.UpdateEnable(true, .025f), .32f + (1.f - .32f) * .25f));
    const float held = s.Alpha();
    C("NaN dt leaves alpha and target unchanged",
      s.UpdateEnable(false, std::numeric_limits<float>::quiet_NaN()) == held);
    C("negative dt leaves alpha and target unchanged", s.UpdateEnable(false, -.01f) == held);
    C("resume original target after invalid inputs", Near(s.UpdateEnable(true, .025f), .32f + (1.f - .32f) * .5f));
    s.Reset();
    C("invalid first dt does not initialize", s.UpdateEnable(false, -.01f) == 0 && s.UpdateEnable(true, 0) == 1);
    s.Reset();
    C("explicit reinitialize takes first actual false", s.UpdateEnable(false, .016f) == 0);
    i.dt = .01f;
    i.targetWorldYaw = std::numeric_limits<float>::quiet_NaN();
    C("nonfinite target rejected", !s.Update(i).valid);
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

#include "../src/mm/MotionOrientationWarp.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

using namespace utpp::mm::motionow;
static int pass = 0, fail = 0;
static void C(const char *name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 1.0e-4f)
{
    return std::isfinite(a) && std::fabs(a - b) <= e;
}

int main()
{
    Inputs in;
    State state;
    in.dt = 1.0f / 60.0f;
    in.rootdeltaX = 0.01f;
    in.worldlocomotionX = 0.0f;
    in.worldlocomotionY = 1.0f;
    in.curvealpha = 0.0f;
    auto out = Step(state, in);
    C("alpha-zero is valid but inactive", out.valid && !out.active);
    C("alpha-zero leaves planar root delta and yaw intact", Near(out.warpedRootdeltaX, in.rootdeltaX) && Near(out.warpedRootdeltaY, in.rootdeltaY) && Near(out.rootdeltaYaw, in.rootdeltaYaw));
    C("alpha-zero leaves per-player interpolation state untouched", state.firstUpdate && Near(state.actualAngle, 0.0f));

    state = {};
    in.curvealpha = 1.0f;
    in.rootdeltaX = 0.0005f; // 0.05 m/s, below the 0.10 m/s threshold
    in.rootdeltaY = 0.0f;
    out = Step(state, in);
    C("minimum root-motion speed guard suppresses target", out.active && Near(out.targetAngle, 0.0f));
    C("speed guard preserves translation", Near(out.warpedRootdeltaX, in.rootdeltaX) && Near(out.warpedRootdeltaY, in.rootdeltaY));

    state = {};
    in.rootdeltaX = 0.01f;
    in.rootdeltaY = 0.0f;
    in.worldlocomotionX = -1.0f;
    in.worldlocomotionY = 0.0f;
    out = Step(state, in);
    C("inverse-direction guard changes 180-degree target to zero", Near(out.targetAngle, 0.0f));

    state = {};
    in.worldlocomotionX = 0.0f;
    in.worldlocomotionY = 1.0f;
    in.rootdeltaYaw = 0.37f;
    out = Step(state, in);
    const float inputMagnitude = std::hypot(in.rootdeltaX, in.rootdeltaY);
    const float outputMagnitude = std::hypot(out.warpedRootdeltaX, out.warpedRootdeltaY);
    C("90-degree planar target is computed", Near(out.targetAngle, 0.5f * 3.14159265358979323846f));
    C("warping preserves planar root-motion magnitude", Near(inputMagnitude, outputMagnitude));
    C("root-motion yaw is unchanged", Near(out.rootdeltaYaw, in.rootdeltaYaw));
    C("first relevant frame snaps pose angle; distribution alpha is applied", Near(out.actualPoseAngle, 0.5f * out.targetAngle));
    C("planar translation rotates toward locomotion", std::fabs(out.warpedRootdeltaX) < 1.0e-5f && out.warpedRootdeltaY > 0.0f);

    // On subsequent frames the pose angle is interpolated, while the root delta translation uses the current target angle.
    in.worldlocomotionX = 1.0f;
    in.worldlocomotionY = 0.0f;
    in.rootdeltaYaw = 0.0f;
    out = Step(state, in);
    C("subsequent frame interpolates toward changed target", out.targetAngle == 0.0f && out.actualPoseAngle > 0.0f && out.actualPoseAngle < 0.5f * 0.5f * 3.14159265358979323846f);

    // UE weights ActualOrientationAngleRad in-place; the next frame starts from this weighted value.
    state = {};
    in.worldlocomotionX = 0.0f;
    in.worldlocomotionY = 1.0f;
    in.curvealpha = 1.0f;
    (void)Step(state, in);
    in.curvealpha = 0.5f;
    out = Step(state, in);
    const float fadedState = state.actualAngle;
    in.curvealpha = 1.0f;
    out = Step(state, in);
    C("curve fade alpha weights stored angle and affects next-frame interpolation", Near(fadedState, 0.25f * 3.14159265358979323846f) && state.actualAngle > fadedState && state.actualAngle < 0.5f * 3.14159265358979323846f);

    // UE's scalar FInterpTo takes the long numeric route across the +/-179-degree boundary.
    state = {};
    const float rad = 3.14159265358979323846f / 180.0f;
    const float boundaryExpected = 179.0f * rad + (-179.0f * rad - 179.0f * rad) * (in.dt * kRotationInterpSpeed);
    C("plus-minus 179 boundary follows scalar UE interpolation oracle", Near(ScalarInterpTo(179.0f * rad, -179.0f * rad, in.dt, kRotationInterpSpeed), boundaryExpected));
    in.curvealpha = 1.0f;
    in.rootdeltaYaw = 0.0f;
    in.worldlocomotionX = std::cos(179.0f * rad);
    in.worldlocomotionY = std::sin(179.0f * rad);
    (void)Step(state, in);
    in.worldlocomotionX = std::cos(-179.0f * rad);
    in.worldlocomotionY = std::sin(-179.0f * rad);
    out = Step(state, in);
    C("135-degree guard still bounds graph target around boundary", Near(out.targetAngle, 1.0f * rad, 1.0e-3f));

    // Root-motion angular change counteracts pose interpolation; a 10-degree delta is below the pivot cutoff.
    state = {};
    in.rootdeltaX = 0.01f;
    in.rootdeltaY = 0.0f;
    in.rootdeltaYaw = 10.0f * 3.14159265358979323846f / 180.0f;
    in.worldlocomotionX = 1.0f;
    in.worldlocomotionY = 0.0f;
    (void)Step(state, in);
    in.rootdeltaYaw = 0.0f;
    out = Step(state, in);
    C("root-motion yaw change is counter-compensated", state.actualAngle < -0.05f && state.counterAngle < -0.01f && out.actualPoseAngle < 0.0f);

    // Below threshold, UE holds the prior direction but still records this frame's root-motion yaw.
    state = {};
    in.rootdeltaX = 0.01f;
    in.rootdeltaY = 0.0f;
    in.dt = 1.0f / 60.0f;
    in.worldlocomotionX = 1.0f;
    in.worldlocomotionY = 0.0f;
    in.rootdeltaYaw = 0.4f;
    (void)Step(state, in);
    const float priorDirectionX = state.prevRootDirectionX;
    const float priorDirectionY = state.prevRootDirectionY;
    in.dt = 0.1f;
    in.rootdeltaX = 0.001f; // 0.01 m/s: below threshold
    in.rootdeltaYaw = 0.2f;
    (void)Step(state, in);
    C("below-threshold frame retains previous root direction", Near(state.prevRootDirectionX, priorDirectionX) && Near(state.prevRootDirectionY, priorDirectionY));
    C("below-threshold frame still records root-motion delta yaw", Near(state.prevRootRotationYaw, 0.2f));

    // The caller's explicit CP fallback follows the actual CP convention: forward is +Y.
    state = {};
    in.dt = 1.0f / 60.0f;
    in.rootdeltaX = 0.0f;
    in.rootdeltaY = 0.01f;
    in.rootdeltaYaw = 0.0f;
    in.worldlocomotionX = 0.0f;
    in.worldlocomotionY = 0.0f;
    in.previousVisualWorldYaw = 0.0f;
    in.fallbackLocalLocomotionX = 0.0f;
    in.fallbackLocalLocomotionY = 1.0f;
    out = Step(state, in);
    C("zero locomotion uses explicit CP forward fallback +Y", Near(out.targetAngle, 0.0f));

    state = {};
    in.rootdeltaX = 0.01f;
    in.rootdeltaY = 0.0f;
    in.fallbackLocalLocomotionX = 0.0f;
    in.fallbackLocalLocomotionY = 0.0f;
    out = Step(state, in);
    C("all-zero explicit fallback leaves target unchanged at zero", Near(out.targetAngle, 0.0f));
    C("bounded remainder handles huge finite angles", std::isfinite(WrapRadians(1.0e30f)) && std::fabs(WrapRadians(1.0e30f)) <= 3.14159265358979323846f);

    in.rootdeltaX = std::numeric_limits<float>::quiet_NaN();
    out = Step(state, in);
    C("non-finite input fails closed", !out.valid && std::isnan(out.warpedRootdeltaX));

    std::printf("Summary: %d passed, %d failed\n", pass, fail);
    return fail == 0 ? 0 : 1;
}

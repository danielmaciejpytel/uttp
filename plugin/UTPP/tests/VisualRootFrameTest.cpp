#include "../src/mm/VisualRootFrame.hpp"
#include <cstdio>
#include <cstring>
#include <limits>
#include <initializer_list>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 2e-5f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
int main()
{
    using visualroot::Aim;
    using visualroot::Frame;
    math::Vec3 out{9, 8, 7};
    Frame f{.2f, -.1f, math::kPi / 2};
    C("90degree pivot with translation inverse",
      visualroot::InversePoint(f, {.2f, .9f, 2}, out) && Near(out.x, 1) && Near(out.y, 0) && Near(out.z, 2));
    math::Vec3 actor;
    C("90degree point forward reconstruction",
      visualroot::Transform(f, out, actor) && Near(actor.x, .2f) && Near(actor.y, .9f));
    for (float yaw : {-9.f, -math::kPi, -1.f, 0.f, 1.f, math::kPi, 15.f})
        for (float distance : {.2f, 2.f, 60.f})
        {
            Frame frame{-.27f, .18f, yaw};
            math::Vec3 target{distance * .4f, -distance * .8f, 1.4f + distance * .3f}, local, reconstructed;
            C("actor target inverse then root reconstructs original",
              visualroot::InversePoint(frame, target, local) && visualroot::Transform(frame, local, reconstructed) &&
                  Near(target.x, reconstructed.x, 2e-5f) && Near(target.y, reconstructed.y, 2e-5f) &&
                  Near(target.z, reconstructed.z));
            // Independent homogeneous inverse: dot with actor-space visual right/forward.
            const double c = std::cos(double(yaw)), s = std::sin(double(yaw));
            const double dx = double(target.x) - frame.x, dy = double(target.y) - frame.y;
            C("inverse agrees with independent orthonormal basis projection",
              Near(local.x, float(c * dx + s * dy), 2e-5f) && Near(local.y, float(-s * dx + c * dy), 2e-5f));
            Aim aim;
            bool ok = visualroot::ComputeAim(frame, target, aim);
            const double a = aim.yawDeg * 3.14159265358979323846 / 180, b = aim.pitchDeg * 3.14159265358979323846 / 180;
            // A +Y-forward aim ray, transformed as a vector using independent world basis.
            const double rx = -std::sin(a) * std::cos(b), ry = std::cos(a) * std::cos(b), rz = std::sin(b);
            const double wx = c * rx - s * ry, wy = s * rx + c * ry;
            const double tz = double(target.z) - 1.4;
            const double length = std::sqrt(dx * dx + dy * dy + tz * tz);
            const double dot = (wx * dx + wy * dy + rz * tz) / length;
            C("visual chest aim ray collinear with original actor target", ok && dot > 1 - 1e-10);
        }
    Aim before{4, 5}, aim = before;
    Frame bad{0, 0, std::numeric_limits<float>::infinity()};
    math::Vec3 sentinel = out;
    C("invalid root preserves point output",
      !visualroot::InversePoint(bad, {1, 2, 3}, out) && std::memcmp(&sentinel, &out, sizeof out) == 0);
    C("invalid root preserves aim output",
      !visualroot::ComputeAim(bad, {1, 2, 3}, aim) && aim.yawDeg == before.yawDeg && aim.pitchDeg == before.pitchDeg);
    C("nonfinite target rejected", !visualroot::ComputeAim({}, {std::numeric_limits<float>::quiet_NaN(), 2, 3}, aim));
    C("nonfinite chest rejected", !visualroot::ComputeAim({}, {1, 2, 3}, aim, std::numeric_limits<float>::infinity()));
    C("target at visual chest has no direction", !visualroot::ComputeAim({.2f, -.1f, 2}, {.2f, -.1f, 1.4f}, aim));
    C("vertical target remains valid", visualroot::ComputeAim({}, {0, 0, 3}, aim) && Near(aim.pitchDeg, 90));
    Aim left, right;
    visualroot::ComputeAim({.1f, .2f, math::kPi - 1e-5f}, {2, 5, 1.6f}, left);
    visualroot::ComputeAim({.1f, .2f, -math::kPi + 1e-5f}, {2, 5, 1.6f}, right);
    C("plusminuspi wrap preserves continuous aim direction",
      std::fabs(std::remainder(double(left.yawDeg - right.yawDeg), 360.)) < .002 &&
          Near(left.pitchDeg, right.pitchDeg));
    C("large finite root yaw wraps safely",
      visualroot::ComputeAim({0, 0, std::numeric_limits<float>::max()}, {60, -60, 2}, aim) &&
          std::isfinite(aim.yawDeg));
    // Explicit opposite camera target bounds, including rear target sign and translation.
    for (float sign : {-1.f, 1.f})
        C("camera target plusminus60metres accepted",
          visualroot::ComputeAim({.3f, -.2f, 1.2f}, {60 * sign, -60 * sign, 60 * sign}, aim));
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

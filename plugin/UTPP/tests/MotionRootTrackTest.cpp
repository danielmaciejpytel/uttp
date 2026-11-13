#include "../src/mm/MotionRootTrack.hpp"
#include <cstdio>
#include <limits>
#include <memory>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void Check(bool ok, const char *label) {
  ++(ok ? pass : fail);
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
}
static bool Near(double a, double b, double eps = 1e-7) {
  return std::fabs(a - b) <= eps;
}
static bool Near(root2::Delta a, root2::Delta b, double eps = 1e-7) {
  return Near(a.x, b.x, eps) && Near(a.y, b.y, eps) &&
         Near(a.yaw, b.yaw, eps) &&
         Near(a.advancedSeconds, b.advancedSeconds, eps);
}
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  auto db = std::make_unique<motion::MotionDatabase>();
  std::string err;
  Check(db->LoadFromFile(argv[1], motion::kRigIdentityWoman, err),
        "actual DB load");
  if (fail)
    return 2;
  const double fps = db->Header().fps;
  int nonloop = -1, loop = -1;
  for (uint32_t i = 0; i < db->NumClips(); ++i) {
    const auto &c = db->Clip(i);
    if (c.numFrames > 20) {
      if (c.flags & motion::kClipLoop)
        loop = i;
      else
        nonloop = i;
    }
  }
  Check(loop >= 0 && nonloop >= 0, "actual loop and nonloop fixtures");
  if (fail)
    return 2;
  root2::Delta out;
  Check(root2::Extract(*db, nonloop, .05, 0, out) && Near(out, {}),
        "zero duration exact identity");
  const auto &c = db->Clip(nonloop);
  const auto &key = db->Root(c.firstFrame + 2);
  root2::Delta expected{key.dx, key.dy, key.dyaw, 1 / fps};
  Check(root2::Extract(*db, nonloop, 2 / fps, 1 / fps, out) &&
            Near(out, expected),
        "full key equals ROOT step");
  // Independent endpoints in frame-2 coordinates: P(u)=u*dP, R(u)=Rz(u*dyaw).
  const double u = .25, v = .75, theta = -u * key.dyaw;
  expected = {
      std::cos(theta) * key.dx * (v - u) - std::sin(theta) * key.dy * (v - u),
      std::sin(theta) * key.dx * (v - u) + std::cos(theta) * key.dy * (v - u),
      key.dyaw * (v - u), (v - u) / fps};
  Check(root2::Extract(*db, nonloop, (2 + u) / fps, (v - u) / fps, out) &&
            Near(out, expected),
        "fractional extraction agrees with inverse absolute endpoints");
  for (int clip : {nonloop, loop})
    for (double t : {.013, .107, .253}) {
      root2::Delta first, second, whole;
      bool ok = root2::Extract(*db, clip, t, .173, first) &&
                root2::Extract(*db, clip, t + .173, .327, second) &&
                root2::Extract(*db, clip, t, .5, whole);
      Check(ok && Near(root2::Compose(first, second), whole, 2e-6),
            "off-grid intervals compose without a frame phase shift");
    }
  const double end = double(c.numFrames - 1) / fps;
  Check(
      root2::Extract(*db, nonloop, end, .5, out) && Near(out, {}),
      "stopped nonloop has zero displacement despite duplicate last ROOT key");
  root2::Delta crossing, tail;
  Check(root2::Extract(*db, nonloop, end - .01, .5, crossing) &&
            root2::Extract(*db, nonloop, end - .01, .01, tail) &&
            Near(crossing, tail) && Near(crossing.advancedSeconds, .01),
        "nonloop crossing consumes only real final interval");
  const auto &lc = db->Clip(loop);
  const double period = double(lc.loopPeriod) / fps;
  root2::Delta a, b, d;
  Check(root2::Extract(*db, loop, period - .01, .5, d) &&
            root2::Extract(*db, loop, period - .01, .01, a) &&
            root2::Extract(*db, loop, 0, .49, b) &&
            Near(root2::Compose(a, b), d, 2e-6),
        "loop wrap composes tail and next cycle");
  Check(root2::Extract(*db, loop, period * 2 + .123, .5, a) &&
            root2::Extract(*db, loop, .123, .5, b) && Near(a, b, 2e-6),
        "loop extraction independent of cycle index");
  // Independent yaw prefix: quarter frame to integer frame 8 on a nonloop.
  double yaw = 0;
  for (int k = 1; k < 8; ++k)
    yaw += db->Root(c.firstFrame + k).dyaw;
  yaw -= .25 * db->Root(c.firstFrame + 1).dyaw;
  Check(root2::Extract(*db, nonloop, 1.25 / fps, 6.75 / fps, out) &&
            Near(out.yaw, yaw),
        "predicted authored yaw equals independent weighted key sum");
  out = {1, 2, 3, 4};
  const auto saved = out;
  Check(!root2::Extract(*db, -1, 0, .5, out) && Near(out, saved),
        "invalid clip preserves output");
  Check(!root2::Extract(*db, nonloop, -1, .5, out) && Near(out, saved),
        "negative start rejected");
  Check(!root2::Extract(*db, nonloop, 0, -1, out) && Near(out, saved),
        "negative duration rejected");
  Check(!root2::Extract(*db, nonloop, 0,
                        std::numeric_limits<double>::infinity(), out) &&
            Near(out, saved),
        "nonfinite duration rejected");
  Check(
      !root2::Extract(*db, loop, std::numeric_limits<double>::max(), .5, out) &&
          Near(out, saved),
      "finite loop start overflowing frame cursor preserves output");
  Check(!root2::Extract(*db, nonloop, std::numeric_limits<double>::max(), .5,
                        out) &&
            Near(out, saved),
        "finite nonloop start overflowing frame cursor preserves output");
  Check(!root2::Extract(*db, loop, period - .01, 10.001, out) &&
            Near(out, saved),
        "invalid loop extraction duration preserves caller output");
  std::printf("%d PASS %d FAIL\n", pass, fail);
  return fail ? 1 : 0;
}

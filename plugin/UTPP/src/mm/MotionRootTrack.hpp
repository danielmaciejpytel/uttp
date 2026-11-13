// Piecewise planar extraction from the baked ROOT track. Independent of engine
// APIs. ROOT[i] is frame i -> i+1 in frame i axes; nonloop's duplicated last
// step is an extrapolation hint and MUST NOT be played beyond the asset
// endpoint.
#pragma once
#include "MotionDatabase.hpp"
#include <algorithm>
#include <cmath>

namespace utpp::mm::root2 {
struct Delta {
  double x = 0, y = 0, yaw = 0, advancedSeconds = 0;
};
inline Delta Compose(const Delta &a, const Delta &b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw,
          a.advancedSeconds + b.advancedSeconds};
}
// Positive animation seconds, deliberately independent of real dt/play rate.
// Used for the source Steering node's .5 ANIMATION-second lookahead.
// Interpolating absolute key positions is linear; fractional local deltas
// must therefore be rotated out of the fraction's starting orientation.
inline bool Extract(const motion::MotionDatabase &db, int32_t clip, double startSeconds,
                    double durationSeconds, Delta &output) {
  if (!db.Loaded() || clip < 0 || uint32_t(clip) >= db.NumClips() ||
      !std::isfinite(startSeconds) || startSeconds < 0 ||
      !std::isfinite(durationSeconds) || durationSeconds < 0 ||
      durationSeconds > 10)
    return false;
  const auto &cl = db.Clip(uint32_t(clip));
  const double fps = db.Header().fps;
  if (cl.numFrames == 0 || !(std::isfinite(fps) && fps > 0))
    return false;
  const bool loop = (cl.flags & motion::kClipLoop) && cl.loopPeriod > 0;
  const double end = loop ? cl.loopPeriod : cl.numFrames - 1;
  if (end > cl.numFrames || end < 0)
    return false;
  Delta result;
  if (end == 0 || durationSeconds == 0) {
    output = result;
    return true;
  }
  double cursor = startSeconds * fps;
  double remaining = durationSeconds * fps;
  // Finite seconds can overflow on conversion to frame units. Reject before
  // fmod/floor or any floating-to-integer conversion; preserve caller output.
  if (!std::isfinite(cursor) || !std::isfinite(remaining))
    return false;
  cursor = loop ? std::fmod(cursor, end) : std::min(cursor, end);
  if (!loop)
    remaining = std::min(remaining, end - cursor);
  for (uint32_t steps = 0; remaining > 1e-10; ++steps) {
    if (steps >= 65536)
      return false;
    if (cursor >= end - 1e-10) {
      if (!loop)
        break;
      cursor = 0;
    }
    const double integer = std::floor(cursor);
    const auto index = uint32_t(integer);
    if (index >= cl.numFrames || (!loop && index + 1 >= cl.numFrames))
      return false;
    const double fraction = cursor - integer;
    const double frames = std::min({remaining, 1 - fraction, end - cursor});
    if (!(frames > 0))
      return false;
    const auto &key = db.Root(cl.firstFrame + index);
    if (!std::isfinite(key.dx) || !std::isfinite(key.dy) ||
        !std::isfinite(key.dyaw))
      return false;
    const double angle = -fraction * key.dyaw;
    const double dx = key.dx * frames, dy = key.dy * frames;
    const Delta part{std::cos(angle) * dx - std::sin(angle) * dy,
                     std::sin(angle) * dx + std::cos(angle) * dy,
                     key.dyaw * frames, frames / fps};
    result = Compose(result, part);
    cursor += frames;
    remaining -= frames;
  }
  if (!std::isfinite(result.x) || !std::isfinite(result.y) ||
      !std::isfinite(result.yaw))
    return false;
  output = result;
  return true;
}
} // namespace utpp::mm::root2

#pragma once

#include "MotionDatabase.hpp"
#include "Steering.hpp"
#include <algorithm>
#include <cmath>

namespace utpp::mm::cancelstart
{
// Only the observed early standing run-start cancellation. Do not reinterpret
// regular stops, reface footwork, stance changes, airborne or shot transitions.
struct State
{
    int mode = 0, movement = 0, stance = 0, gait = 0;
    float speed = 0;
    bool starting = false, special = false, explicitWalk = false;
};
inline bool SupportAllowed(const State& state)
{
    // VariableSpeedWalkRun maps released input to Walk, even after a Run start.
    // This inferred idle gait is distinct from an explicit walk-toggle request.
    return state.mode == 0 && state.movement == 0 && state.stance == 0 &&
        (state.gait == 0 || state.gait == 1) && !state.special && !state.explicitWalk;
}
inline bool Candidate(const State& before, const State& after, bool nonLoop,
                      int sourceClass, float phase, int destinationClass, bool forced)
{
    return !forced && nonLoop && before.mode == 0 && before.movement == 1 && before.stance == 0 &&
        before.gait == 1 && before.starting && !before.special && !before.explicitWalk && SupportAllowed(after) &&
        std::isfinite(after.speed) && after.speed >= 0 && after.speed <= 150 &&
        std::isfinite(phase) && phase >= 0 && phase <= .35f && sourceClass == steer::kClassStart &&
        (destinationClass == steer::kClassStop || destinationClass == steer::kClassIdle);
}
inline float Contact(const motion::MotionDatabase& db, int32_t clip, float time, int side)
{
    if (clip < 0 || uint32_t(clip) >= db.NumClips() || !std::isfinite(time) || (side != 0 && side != 1)) return 0;
    const auto& record = db.Clip(uint32_t(clip));
    if (!record.numFrames) return 0;
    const float phase = std::clamp(time * db.Header().fps, 0.f, float(record.numFrames - 1));
    const uint32_t first = uint32_t(phase), second = std::min(first + 1, record.numFrames - 1);
    const auto& a = db.Contact(record.firstFrame + first);
    const auto& b = db.Contact(record.firstFrame + second);
    const float x = side ? a.r : a.l, y = side ? b.r : b.l;
    return x + (y - x) * (phase - first);
}
struct Plan { int support = -1; float duration = 0; };
inline Plan Resolve(const motion::MotionDatabase& db, int32_t source, float sourceTime, float sourceRate,
                    int32_t destination, float destinationTime, float destinationRate,
                    float displayedLeft, float displayedRight, float normalDuration)
{
    Plan result{-1, normalDuration};
    if (!std::isfinite(displayedLeft) || !std::isfinite(displayedRight) ||
        !std::isfinite(sourceRate) || !std::isfinite(destinationRate) || sourceRate <= 0 || destinationRate <= 0 ||
        !std::isfinite(normalDuration) || normalDuration <= 0) return result;
    // Select from the pose that was actually displayed, before inserting the new
    // stop. Ambiguous double support is not enough to invent a supporting foot.
    if (std::fabs(displayedLeft - displayedRight) < .1f) return result;
    const int side = displayedLeft > displayedRight ? 0 : 1;
    if (std::max(displayedLeft, displayedRight) < .85f ||
        Contact(db, source, sourceTime, side) < .7f || Contact(db, destination, destinationTime, side) < .7f)
        return result;
    // Finish the old accelerating start before the shared authored support is
    // handed over. Keep a bounded smooth crossfade rather than cutting its pose.
    float sharedSupport = .2f;
    for (int step = 1; step <= 24; ++step)
    {
        const float t = float(step) / 120.f;
        if (Contact(db, source, sourceTime + t * sourceRate, side) < .7f ||
            Contact(db, destination, destinationTime + t * destinationRate, side) < .7f)
        { sharedSupport = t; break; }
    }
    result.support = side;
    result.duration = std::min(normalDuration, std::clamp(sharedSupport, .12f, .2f));
    return result;
}
} // namespace utpp::mm::cancelstart

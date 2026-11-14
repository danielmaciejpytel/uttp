#pragma once

#include <cmath>
#include <cstdint>

namespace utpp::mm::refacestart
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg = kPi / 180.0f;
inline float Angle(float value) { return std::remainder(value, 2.0f * kPi); }

// One ordinary idle -> near-backward start. The matcher and body controller
// share its initial side; neither can change it merely by crossing +/-180.
struct Episode
{
    int direction = 0;
    int32_t clip = -1;
    bool previousInput = false;
    float target = 0, elapsed = 0;
    int gait = -1;

    void Clear() { direction = 0; clip = -1; elapsed = 0; }
    void Reset() { *this = {}; }
    void Step(float dt, bool eligible, bool input, bool starting, float speed,
              float actorYaw, float targetYaw, int currentGait)
    {
        if (!std::isfinite(dt) || dt < 0 || dt > .25f || !eligible ||
            !std::isfinite(speed) || !std::isfinite(actorYaw) || !std::isfinite(targetYaw))
        {
            Clear();
            previousInput = input;
            return;
        }
        if (direction)
        {
            elapsed += dt;
            if (!input || !starting || currentGait != gait || elapsed > 2.0f ||
                std::fabs(Angle(targetYaw - target)) > 20.0f * kDeg)
                Clear();
        }
        if (dt == 0) return; // allow cancellation, but do not consume a new edge
        const bool pressed = input && !previousInput;
        previousInput = input;
        if (!direction && pressed && starting && speed <= .5f && currentGait >= 1)
        {
            const float error = Angle(targetYaw - actorYaw);
            if (std::fabs(error) >= 120.0f * kDeg)
            {
                direction = error >= 0 ? 1 : -1;
                target = targetYaw;
                gait = currentGait;
            }
        }
    }
    void Selected(int32_t selectedClip, int selectedDirection)
    {
        if (!direction) return;
        if (selectedDirection != direction || selectedClip < 0)
            Clear();
        else
            clip = selectedClip;
    }
};

// Protect only the ambiguous half-turn. Near the destination ordinary shortest
// error wins, so overshooting cannot turn a small correction into a full circle.
inline float ControllerTarget(float actor, float target, int direction)
{
    if (direction != 1 && direction != -1) return target;
    const float error = Angle(target - actor);
    if (error * direction < 0 && std::fabs(error) >= 150.0f * kDeg)
        return Angle(actor + direction * 179.0f * kDeg);
    return target;
}
} // namespace utpp::mm::refacestart

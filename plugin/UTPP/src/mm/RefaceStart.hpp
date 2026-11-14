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
    bool pending = false; // a press while moving waits for the matcher's pick on the same feed
    float target = 0, elapsed = 0;
    int gait = -1;

    void Clear() { direction = 0; clip = -1; elapsed = 0; pending = false; }
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
            // A gait change alone keeps the side: the wanted direction is still
            // the one the start was admitted for, so the faster variant of the
            // start must turn the same way. Dropping to rest ends the episode.
            if (!input || !starting || currentGait < 1 || elapsed > 2.0f ||
                std::fabs(Angle(targetYaw - target)) > 20.0f * kDeg)
                Clear();
            else
                gait = currentGait;
        }
        if (dt == 0) return; // allow cancellation, but do not consume a new edge
        const bool pressed = input && !previousInput;
        previousInput = input;
        pending = false;
        if (!direction && pressed && starting && currentGait >= 1)
        {
            const float error = Angle(targetYaw - actorYaw);
            if (std::fabs(error) >= 120.0f * kDeg)
            {
                target = targetYaw;
                gait = currentGait;
                elapsed = 0;
                // From rest the side is the sign of the error. A new press while
                // still moving (the stick pressed again during a stop) takes the
                // side of the reface start the matcher picks on this feed, so
                // the body turns the way that clip turns.
                if (speed <= .5f)
                    direction = error >= 0 ? 1 : -1;
                else
                    pending = true;
            }
        }
    }
    void Selected(int32_t selectedClip, int selectedDirection)
    {
        if (!direction)
        {
            if (pending && selectedClip >= 0 && (selectedDirection == 1 || selectedDirection == -1))
            {
                direction = selectedDirection;
                clip = selectedClip;
            }
            pending = false;
            return;
        }
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

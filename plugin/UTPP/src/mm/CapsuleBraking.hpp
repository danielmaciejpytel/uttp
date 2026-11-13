#pragma once
// Native capsule stop prediction in game units, mapped to the existing MM query
// units. The stat is a model input, not a measured deceleration or friction fit.
#include <algorithm>
#include <cmath>
#include "PlaybackRate.hpp"

namespace utpp::mm::brake
{
inline float FreshDeceleration(float value, float age, float maxAge)
{
    return std::isfinite(value) && value > 0.0f && std::isfinite(age) &&
                   age >= 0.0f && age <= maxAge ? value : 0.0f;
}

// Integral of the existing piecewise-linear speed map from zero to gameSpeed.
// Input is m/s; output map values are cm/s. Integrating before dividing by the
// physical deceleration gives query distance in cm, including a partial final
// prediction step instead of adding another full 0.1 s of motion after stopping.
inline float MapIntegral(const SpeedPoints& map, float gameSpeed)
{
    if (!(gameSpeed > 0.0f)) return 0.0f;
    float result = 0.0f, previous = 0.0f;
    for (int i = 0; i < map.count; ++i)
    {
        const float end = std::min(gameSpeed, map.game[i]);
        result += 0.5f * (MapSpeedPoints(map, previous) + MapSpeedPoints(map, end)) * (end - previous);
        if (gameSpeed <= end) return result;
        previous = end;
    }
    return result + 0.5f * (MapSpeedPoints(map, previous) + MapSpeedPoints(map, gameSpeed)) * (gameSpeed - previous);
}

struct Prediction
{
    float deceleration = 0.0f; // physical game m/s^2; zero keeps the original motion predictor
    float initialSpeed = 0.0f; // physical horizontal game m/s
    float crouchWeight = 0.0f; // same current calibration blend as MapToChooser
    SpeedPoints stand{0, {}, {}}, crouch{0, {}, {}};

    bool Active() const
    {
        return std::isfinite(deceleration) && deceleration > 0.0f &&
               std::isfinite(initialSpeed) && initialSpeed >= 0.0f;
    }
    float GameSpeedAt(float time) const
    {
        return std::max(0.0f, initialSpeed - deceleration * std::max(0.0f, time));
    }
    float QuerySpeed(float speed) const
    {
        const float a = MapSpeedPoints(stand, speed), b = MapSpeedPoints(crouch, speed);
        return a + (b - a) * crouchWeight;
    }
    float QuerySpeedAt(float time) const { return QuerySpeed(GameSpeedAt(time)); }
    float DistanceBetween(float from, float to) const
    {
        if (!Active() || !(to > from)) return 0.0f;
        const float high = GameSpeedAt(from), low = GameSpeedAt(to);
        const float a = MapIntegral(stand, high) - MapIntegral(stand, low);
        const float b = MapIntegral(crouch, high) - MapIntegral(crouch, low);
        return std::max(0.0f, (a + (b - a) * crouchWeight) / deceleration);
    }
};
} // namespace utpp::mm::brake

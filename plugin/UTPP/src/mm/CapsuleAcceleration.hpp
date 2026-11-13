#pragma once
// Physical, already-applied native acceleration; this changes the MM forecast,
// never the game's acceleration stat or the accepted native braking path.
#include "CapsuleBraking.hpp"

namespace utpp::mm::accel
{
// UTPPUpdateAccel reads the stat BEFORE possibly changing our modifier. Only
// retain that observation if its existing modifier will survive the call. The
// requested target is used to reject an obsolete sample, never as a measurement.
inline float AppliedAcceleration(float base, float applied, float requestedTarget)
{
    if (!std::isfinite(base) || !(base > 0.f) || !std::isfinite(applied) ||
        !(applied > 0.f && applied <= 1.f) || !std::isfinite(requestedTarget)) return 0.f;
    const float want = requestedTarget > 0.f && base > requestedTarget ?
        std::clamp(requestedTarget / base, .05f, 1.f) : 1.f;
    // Same branches/tolerance as scripts/utpp.reds UTPPUpdateAccel. applied==1
    // can mean no modifier object, so it cannot use the modifier-retention branch.
    const bool unchanged = want >= .999f ? applied == 1.f :
        applied < 1.f && std::fabs(want - applied) < .02f;
    const float effective = base * applied;
    return unchanged && std::isfinite(effective) ? effective : 0.f;
}

inline float FreshAcceleration(float value, float age, float maxAge, bool sameDomain)
{
    return sameDomain ? brake::FreshDeceleration(value, age, maxAge) : 0.f;
}

struct Prediction
{
    float acceleration = 0.f; // applied physical game m/s^2, not query acceleration
    float initialSpeed = 0.f;
    float maxSpeed = 0.f;     // physical speed cap; only straight full-stick forecasts
    float crouchWeight = 0.f;
    SpeedPoints stand{0, {}, {}}, crouch{0, {}, {}};

    bool Active() const
    {
        return std::isfinite(acceleration) && acceleration > 0.f &&
            std::isfinite(initialSpeed) && initialSpeed >= 0.f &&
            std::isfinite(maxSpeed) && maxSpeed > initialSpeed;
    }
    float GameSpeedAt(float time) const
    {
        return std::min(maxSpeed, initialSpeed + acceleration * std::max(0.f, time));
    }
    float QuerySpeed(float speed) const
    {
        const float a = MapSpeedPoints(stand, speed), b = MapSpeedPoints(crouch, speed);
        return a + (b - a) * crouchWeight;
    }
    float QuerySpeedAt(float time) const { return QuerySpeed(GameSpeedAt(time)); }
    float DistanceBetween(float from, float to) const
    {
        if (!Active() || !(to > from)) return 0.f;
        from = std::max(0.f, from);
        to = std::max(from, to);
        const float low = GameSpeedAt(from), high = GameSpeedAt(to);
        const float a = brake::MapIntegral(stand, high) - brake::MapIntegral(stand, low);
        const float b = brake::MapIntegral(crouch, high) - brake::MapIntegral(crouch, low);
        const float capTime = (maxSpeed - initialSpeed) / acceleration;
        const float atCap = std::max(0.f, to - std::max(from, capTime));
        return std::max(0.f, (a + (b - a) * crouchWeight) / acceleration) + atCap * QuerySpeed(maxSpeed);
    }
};
} // namespace utpp::mm::accel

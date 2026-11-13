// Pure same-frame planar root/aim coordinate conversions. No pose or grip mutation.
#pragma once
#include "MMMath.hpp"
#include <cmath>
namespace utpp::mm::visualroot
{
struct Frame
{
    float x = 0, y = 0, yaw = 0;
}; // visual origin/orientation in actor coordinates
struct Aim
{
    float yawDeg = 0, pitchDeg = 0;
};
inline bool Valid(Frame f)
{
    return std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.yaw);
}
inline bool Transform(Frame f, math::Vec3 visual, math::Vec3 &actor)
{
    if (!Valid(f) || !math::IsFinite(visual))
        return false;
    const double angle = std::remainder(double(f.yaw), 2 * 3.14159265358979323846);
    const double c = std::cos(angle), s = std::sin(angle);
    const math::Vec3 candidate{float(double(f.x) + c * visual.x - s * visual.y),
                               float(double(f.y) + s * visual.x + c * visual.y), visual.z};
    if (!math::IsFinite(candidate))
        return false;
    actor = candidate;
    return true;
}
inline bool InversePoint(Frame f, math::Vec3 actor, math::Vec3 &visual)
{
    if (!Valid(f) || !math::IsFinite(actor))
        return false;
    const double angle = std::remainder(double(f.yaw), 2 * 3.14159265358979323846);
    const double c = std::cos(angle), s = std::sin(angle), x = double(actor.x) - f.x, y = double(actor.y) - f.y;
    const math::Vec3 candidate{float(c * x + s * y), float(-s * x + c * y), actor.z};
    if (!math::IsFinite(candidate))
        return false;
    visual = candidate;
    return true;
}
inline bool ComputeAim(Frame f, math::Vec3 pointActor, Aim &result, float chestZ = 1.4f)
{
    if (!std::isfinite(chestZ))
        return false;
    math::Vec3 target;
    if (!InversePoint(f, pointActor, target))
        return false;
    const double x = target.x, y = target.y, z = double(target.z) - chestZ, h = std::hypot(x, y);
    if (h <= 1e-4 && std::fabs(z) <= 1e-4)
        return false;
    const Aim candidate{float(std::atan2(-x, y) * 180 / 3.14159265358979323846),
                        float(std::atan2(z, h) * 180 / 3.14159265358979323846)};
    if (!std::isfinite(candidate.yawDeg) || !std::isfinite(candidate.pitchDeg))
        return false;
    result = candidate;
    return true;
}
} // namespace utpp::mm::visualroot

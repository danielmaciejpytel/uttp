#pragma once
// UTPP quaternion helpers. Pure, allocation-free, no SDK dependencies so the same code is used by
// the FK detour (hot path) and by the offline self-test (tests/QuatSelfTest.cpp).
//
// Layout matches the CP2077 bone record rotation: float4 (x, y, z, w).
// Convention: Hamilton product. ApplyLocalRotation computes q' = q * r, i.e. r is applied about the
// bone's LOCAL axis (the axis expressed in the bone's own frame), then renormalizes.

#include <cmath>
#include <cstdint>

namespace utpp::quat
{
struct Q
{
    float x, y, z, w;
};

constexpr float kPi = 3.14159265358979323846f;

inline Q Mul(const Q& a, const Q& b)
{
    return Q{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// axis: 0 = X, 1 = Y, 2 = Z. Returns false (and leaves aOut untouched) for an invalid axis or a
// non-finite angle.
inline bool AxisAngleDeg(int32_t aAxis, float aDeg, Q& aOut)
{
    if (!std::isfinite(aDeg))
        return false;
    const float half = aDeg * (kPi / 360.0f);
    const float s = std::sin(half);
    const float c = std::cos(half);
    switch (aAxis)
    {
    case 0:
        aOut = Q{s, 0.0f, 0.0f, c};
        return true;
    case 1:
        aOut = Q{0.0f, s, 0.0f, c};
        return true;
    case 2:
        aOut = Q{0.0f, 0.0f, s, c};
        return true;
    default:
        return false;
    }
}

// Returns false (and leaves aQ untouched) for a zero-length or non-finite quaternion.
inline bool Normalize(Q& aQ)
{
    const float n2 = aQ.x * aQ.x + aQ.y * aQ.y + aQ.z * aQ.z + aQ.w * aQ.w;
    if (!std::isfinite(n2) || !(n2 > 1e-12f))
        return false;
    const float inv = 1.0f / std::sqrt(n2);
    aQ.x *= inv;
    aQ.y *= inv;
    aQ.z *= inv;
    aQ.w *= inv;
    return true;
}

// In-place: aQ4 (x,y,z,w) = normalize(aQ4 * axisAngle(aAxis, aDeg)). Returns false and leaves the
// memory untouched if any input is invalid or the result cannot be normalized.
inline bool ApplyLocalRotation(float* aQ4, int32_t aAxis, float aDeg)
{
    Q r;
    if (!AxisAngleDeg(aAxis, aDeg, r))
        return false;
    const Q q{aQ4[0], aQ4[1], aQ4[2], aQ4[3]};
    Q out = Mul(q, r);
    if (!Normalize(out))
        return false;
    aQ4[0] = out.x;
    aQ4[1] = out.y;
    aQ4[2] = out.z;
    aQ4[3] = out.w;
    return true;
}
} // namespace utpp::quat

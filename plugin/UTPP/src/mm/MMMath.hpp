// UTPP motion matching runtime - small vector / quaternion / spring helpers. Header-only, allocation-free.
//
// Quaternions are stored x, y, z, w (CP2077 / .utppmm layout), Hamilton product. Planar yaw is CCW about +Z.
// Springs, quaternion log/exp and the inertialization primitives are re-implementations of Daniel Holden's
// "Motion-Matching" reference code (orangeduck, MIT License, Copyright (c) 2021 Daniel Holden
// Motion-Matching: common.h fast_negexpf, quat.h quat_log/quat_exp/quat_abs, spring.h
// halflife_to_damping / simple_spring_damper_exact / decay_spring_damper_exact, controller.cpp
// simulation_positions_update). Converted to x,y,z,w storage and to a planar (x, y, yaw) simulation.
#pragma once

#include <cmath>
#include <cstdint>

namespace utpp::mm::math
{
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kLn2 = 0.69314718056f;

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return {a.x * s, a.y * s, a.z * s}; }
inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline Vec3 Lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }
inline bool IsFinite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

struct Quat
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

inline Quat QuatMul(const Quat& a, const Quat& b)
{
    return Quat{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat QuatInv(const Quat& q) { return Quat{-q.x, -q.y, -q.z, q.w}; } // unit quaternions only
inline Quat QuatNeg(const Quat& q) { return Quat{-q.x, -q.y, -q.z, -q.w}; }
inline Quat QuatAbs(const Quat& q) { return q.w < 0.0f ? QuatNeg(q) : q; }
inline float QuatDot(const Quat& a, const Quat& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline bool IsFinite(const Quat& q) { return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w); }

// Normalizes; returns identity for zero-length / non-finite input.
inline Quat QuatNormalize(const Quat& q)
{
    const float n2 = QuatDot(q, q);
    if (!std::isfinite(n2) || !(n2 > 1e-20f))
        return Quat{};
    const float inv = 1.0f / std::sqrt(n2);
    return Quat{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

inline Vec3 QuatRotate(const Quat& q, Vec3 v)
{
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = 2.0f * Cross(u, v);
    return v + q.w * t + Cross(u, t);
}

// orangeduck quat_exp / quat_log (x,y,z,w storage).
inline Quat QuatExp(Vec3 v, float eps = 1e-8f)
{
    const float halfangle = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (halfangle < eps)
        return QuatNormalize(Quat{v.x, v.y, v.z, 1.0f});
    const float c = std::cos(halfangle);
    const float s = std::sin(halfangle) / halfangle;
    return Quat{s * v.x, s * v.y, s * v.z, c};
}

inline Vec3 QuatLog(const Quat& q, float eps = 1e-8f)
{
    const float length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (length < eps)
        return Vec3{q.x, q.y, q.z};
    const float halfangle = std::acos(std::fmin(std::fmax(q.w, -1.0f), 1.0f));
    return Vec3{q.x, q.y, q.z} * (halfangle / length);
}

inline Quat QuatFromScaledAngleAxis(Vec3 v) { return QuatExp(v * 0.5f); }
inline Vec3 QuatToScaledAngleAxis(const Quat& q) { return 2.0f * QuatLog(q); }

// Angular velocity (parent space, rad/s) taking aCurr to aNext in aDt: log(abs(next * inv(curr))) * 2 / dt.
inline Vec3 QuatAngularVelocity(const Quat& aNext, const Quat& aCurr, float aInvDt)
{
    return QuatToScaledAngleAxis(QuatAbs(QuatMul(aNext, QuatInv(aCurr)))) * aInvDt;
}

// Shortest-path normalized lerp.
inline Quat QuatNlerpShortest(const Quat& a, Quat b, float t)
{
    if (QuatDot(a, b) < 0.0f)
        b = QuatNeg(b);
    return QuatNormalize(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t});
}

// Angle (radians, 0..pi) between two unit rotations.
inline float QuatAngleBetween(const Quat& a, const Quat& b)
{
    const float d = std::fabs(QuatDot(a, b));
    return 2.0f * std::acos(std::fmin(d, 1.0f));
}

// ---- planar helpers (x, y in metres, yaw CCW about +Z)

inline void Rot2(float aYaw, float aX, float aY, float& aOutX, float& aOutY)
{
    const float c = std::cos(aYaw);
    const float s = std::sin(aYaw);
    aOutX = c * aX - s * aY;
    aOutY = s * aX + c * aY;
}

// Wraps to (-pi, pi].
inline float WrapAngle(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::remainder(a, 2.0f * kPi);
    return a <= -kPi ? a + 2.0f * kPi : a;
}

// ---- springs (orangeduck spring.h)

inline float FastNegExp(float x) { return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x); }
inline float HalflifeToDamping(float aHalflife, float eps = 1e-5f) { return (4.0f * kLn2) / (aHalflife + eps); }

// Critically damped spring of a scalar towards a fixed goal (exact for constant goal over dt).
inline void SimpleSpringDamperExact(float& x, float& v, float aGoal, float aHalflife, float dt)
{
    const float y = HalflifeToDamping(aHalflife) / 2.0f;
    const float j0 = x - aGoal;
    const float j1 = v + j0 * y;
    const float eydt = FastNegExp(y * dt);
    x = eydt * (j0 + j1 * dt) + aGoal;
    v = eydt * (v - j1 * y * dt);
}

inline void DecaySpringDamperExact(Vec3& x, Vec3& v, float aHalflife, float dt)
{
    const float y = HalflifeToDamping(aHalflife) / 2.0f;
    const Vec3 j1 = v + x * y;
    const float eydt = FastNegExp(y * dt);
    x = eydt * (x + j1 * dt);
    v = eydt * (v - j1 * y * dt);
}

inline void DecaySpringDamperExact(Quat& x, Vec3& v, float aHalflife, float dt)
{
    const float y = HalflifeToDamping(aHalflife) / 2.0f;
    const Vec3 j0 = QuatToScaledAngleAxis(x);
    const Vec3 j1 = v + j0 * y;
    const float eydt = FastNegExp(y * dt);
    x = QuatFromScaledAngleAxis(eydt * (j0 + j1 * dt));
    v = eydt * (v - j1 * y * dt);
}

// orangeduck controller.cpp simulation_positions_update (velocity spring with acceleration state), planar, one axis.
// Exact for a constant desired velocity over dt.
inline void SimulationAxisUpdate(float& p, float& v, float& a, float aDesiredV, float aHalflife, float dt)
{
    const float y = HalflifeToDamping(aHalflife) / 2.0f;
    const float j0 = v - aDesiredV;
    const float j1 = a + j0 * y;
    const float eydt = FastNegExp(y * dt);
    const float pPrev = p;
    p = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) + (j1 / (y * y)) + j0 / y + aDesiredV * dt + pPrev;
    v = eydt * (j0 + j1 * dt) + aDesiredV;
    a = eydt * (a - j1 * y * dt);
}

} // namespace utpp::mm::math

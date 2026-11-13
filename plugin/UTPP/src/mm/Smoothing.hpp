// Smoothing primitives of the UE 5.8 blend space weights (Core/Public/Math/UnrealMathUtility.h 1553-1559, 1608-1648: InvExpApprox /
// CriticallyDampedSmoothing; FMath::FInterpConstantTo). Header-only, no game hooks. Used by LeanPack.hpp (the DLL) and BlendSpace.hpp (offline tests).
#pragma once
#include <algorithm>

namespace utpp::mm::bs {

inline float InvExpApprox(float x) {
    return 1.f / (1.f + 1.00746054f * x + 0.45053901f * x * x + 0.25724632f * x * x * x);
}

// UE FMath::CriticallyDampedSmoothing (target rate 0).
inline void CriticallyDampedSmoothing(float& value, float& rate, float target, float dt, float smoothingTime) {
    if (smoothingTime > 1e-4f) {
        const float w = 2.f / smoothingTime, e = InvExpApprox(w * dt), x0 = value - target;
        const float b = (rate + x0 * w) * dt;
        value = target + (x0 + b) * e;
        rate = (rate - b * w) * e;
    } else if (dt > 0.f) { rate = (target - value) / dt; value = target; }
    else { value = target; rate = 0.f; }
}

inline float FInterpConstantTo(float cur, float target, float dt, float speed) {
    const float d = target - cur;
    if (d * d < 1e-8f) return target;
    const float step = speed * dt;
    return cur + std::clamp(d, -step, step);
}

}  // namespace utpp::mm::bs

// UTPP port report 4.2 #34: DeadBlending as the UE 5.8 FAnimNode_DeadBlending node does it. Header-only.
// References (UE 5.8 Engine/Source/Runtime/Engine):
//   Classes/Animation/AnimNode_DeadBlending.h 56-133: DefaultBlendDuration 0.25, ExtrapolationHalfLife 1.0,
//     HalfLifeMin 0.05, HalfLifeMax 1.0, MaximumTranslationVelocity 500 (cm/s), MaximumRotationVelocity 360 deg/s.
//   Private/Animation/AnimNode_DeadBlending.cpp
//     148-182 ExtrapolateTranslation / ExtrapolateRotation: x + v/c * (1 - InvExpApprox(c t)), c = ln2 / halfLife
//     227-245 ComputeDecayHalfLifeFromDiffAndVelocity: clamp(HalfLife * diff / v, Min, Max) per component
//     360-422 InitFrom: source pose + finite-difference velocity (prev->curr), clamped to the maxima
//     520-600 Apply: alpha = 1 - BlendOption(t / duration); out = lerp(dst, extrapolated src, alpha)
//                    (rotation: RotationDiff = extrap * dst^-1, shortest arc, scaled rotation vector * alpha)
//   Private/AlphaBlend.cpp 244-247: Cubic = CubicInterp(0,0,1,0,t), HermiteCubic = SmoothStep -> both 3t^2 - 2t^3.
// Units: translation in the caller's unit (cm in UE; pass maxTranslationVelocity accordingly, 5 m/s for CP77 metres).
// Scales are not extrapolated (UTPP poses carry no scale); curves use ExtrapolateCurve semantics via Extrapolate1.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace utpp::mm::deadblend {

struct V3 { float x = 0, y = 0, z = 0; };
struct Q4 { float x = 0, y = 0, z = 0, w = 1; };
struct BoneTR { V3 t; Q4 r; };

inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float Len(V3 a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
inline Q4 Mul(Q4 a, Q4 b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Q4 Inv(Q4 q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Q4 Shortest(Q4 q) { return q.w < 0 ? Q4{-q.x, -q.y, -q.z, -q.w} : q; }
inline V3 ToRotVec(Q4 q) {
    q = Shortest(q);
    const float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-8f) return {2 * q.x, 2 * q.y, 2 * q.z};
    const float a = 2.f * std::atan2(s, q.w);
    return {q.x / s * a, q.y / s * a, q.z / s * a};
}
inline Q4 FromRotVec(V3 v) {
    const float a = Len(v);
    if (a < 1e-8f) return {v.x * 0.5f, v.y * 0.5f, v.z * 0.5f, 1.f};
    const float s = std::sin(a * 0.5f) / a;
    return {v.x * s, v.y * s, v.z * s, std::cos(a * 0.5f)};
}

enum class BlendOption { Linear, Cubic, HermiteCubic };
inline float AlphaToBlendOption(float t, BlendOption o) {
    t = std::clamp(t, 0.f, 1.f);
    return o == BlendOption::Linear ? t : t * t * (3.f - 2.f * t);
}
inline float InvExpApprox(float x) {
    return 1.f / (1.f + 1.00746054f * x + 0.45053901f * x * x + 0.25724632f * x * x * x);
}
constexpr float kLn2 = 0.69314718056f;

inline float DecayHalfLife(float diff, float vel, float hl, float hlMin, float hlMax) {
    const float v = std::fabs(vel) < 1e-8f ? (vel < 0 ? -1e-8f : 1e-8f) : vel;   // ClipMagnitudeToGreaterThanEpsilon
    return std::clamp(hl * (diff / v), hlMin, hlMax);
}
// 1D extrapolation (also ExtrapolateCurve).
inline float Extrapolate1(float x, float v, float t, float halfLife) {
    if (v * v <= 1e-8f) return x;
    const float c = kLn2 / std::max(halfLife, 1e-8f);
    return x + (v / c) * (1.f - InvExpApprox(c * t));
}

struct Settings {
    float blendDuration = 0.25f;
    float halfLife = 1.0f, halfLifeMin = 0.05f, halfLifeMax = 1.0f;
    float maxTranslationVelocity = 500.f;        // per second, caller units
    float maxRotationVelocityDeg = 360.f;
    BlendOption blend = BlendOption::Linear;      // node default; motion AO switch uses Cubic (report 3.12)
};

class DeadBlender {
public:
    // Start a transition: source pose this frame + previous frame (dt apart) and the destination pose this frame.
    void Start(const std::vector<BoneTR>& srcCurr, const std::vector<BoneTR>& srcPrev, float dt,
               const std::vector<BoneTR>& dstNow, const Settings& s) {
        s_ = s; time_ = 0.f; active_ = true;
        const size_t n = srcCurr.size();
        src_ = srcCurr; vt_.assign(n, {}); vr_.assign(n, {}); hlT_.assign(n, {}); hlR_.assign(n, {});
        dir_.assign(n, {});
        for (size_t i = 0; i < n; ++i) {
            if (dt > 1e-8f && i < srcPrev.size()) {
                V3 vt = (srcCurr[i].t - srcPrev[i].t) * (1.f / dt);
                V3 vr = ToRotVec(Mul(srcCurr[i].r, Inv(srcPrev[i].r))) * (1.f / dt);
                const float lt = Len(vt), lr = Len(vr), mr = s.maxRotationVelocityDeg * 3.14159265358979f / 180.f;
                if (lt > s.maxTranslationVelocity) vt = vt * (s.maxTranslationVelocity / lt);
                if (lr > mr) vr = vr * (mr / lr);
                vt_[i] = vt; vr_[i] = vr;
            }
            const V3 dT = (i < dstNow.size() ? dstNow[i].t : srcCurr[i].t) - srcCurr[i].t;
            const V3 dR = ToRotVec(Mul(i < dstNow.size() ? dstNow[i].r : srcCurr[i].r, Inv(srcCurr[i].r)));
            hlT_[i] = {DecayHalfLife(dT.x, vt_[i].x, s.halfLife, s.halfLifeMin, s.halfLifeMax),
                       DecayHalfLife(dT.y, vt_[i].y, s.halfLife, s.halfLifeMin, s.halfLifeMax),
                       DecayHalfLife(dT.z, vt_[i].z, s.halfLife, s.halfLifeMin, s.halfLifeMax)};
            hlR_[i] = {DecayHalfLife(dR.x, vr_[i].x, s.halfLife, s.halfLifeMin, s.halfLifeMax),
                       DecayHalfLife(dR.y, vr_[i].y, s.halfLife, s.halfLifeMin, s.halfLifeMax),
                       DecayHalfLife(dR.z, vr_[i].z, s.halfLife, s.halfLifeMin, s.halfLifeMax)};
        }
    }
    bool Active() const { return active_; }
    float Time() const { return time_; }

    // Advance by dt, then blend the extrapolated source over dst in place. First call with dt = 0 gives the source.
    void Apply(std::vector<BoneTR>& pose, float dt) {
        if (!active_) return;
        time_ += dt;
        const float alpha = 1.f - AlphaToBlendOption(time_ / std::max(s_.blendDuration, 1e-8f), s_.blend);
        for (size_t i = 0; i < pose.size() && i < src_.size(); ++i) {
            const V3 et = Extrap3(src_[i].t, vt_[i], hlT_[i]);
            const Q4 er = Mul(FromRotVec(Extrap3({}, vr_[i], hlR_[i])), src_[i].r);
            if (alpha <= 0.f) continue;
            pose[i].t = pose[i].t + (et - pose[i].t) * alpha;
            Q4 diff = Mul(er, Inv(pose[i].r));
            const float d = diff.x * dir_[i].x + diff.y * dir_[i].y + diff.z * dir_[i].z + diff.w * dir_[i].w;
            if (d < 0) diff = {-diff.x, -diff.y, -diff.z, -diff.w};   // EnforceShortestArcWith(previous direction)
            dir_[i] = diff;
            pose[i].r = Mul(FromRotVec(ToRotVecRaw(diff) * alpha), pose[i].r);
        }
        if (time_ >= s_.blendDuration) active_ = false;
    }

private:
    V3 Extrap3(V3 x, V3 v, V3 hl) const {
        return {Extrapolate1(x.x, v.x, time_, hl.x), Extrapolate1(x.y, v.y, time_, hl.y), Extrapolate1(x.z, v.z, time_, hl.z)};
    }
    static V3 ToRotVecRaw(Q4 q) {   // no shortest-arc flip (direction already enforced)
        const float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (s < 1e-8f) return {2 * q.x, 2 * q.y, 2 * q.z};
        const float a = 2.f * std::atan2(s, q.w);
        return {q.x / s * a, q.y / s * a, q.z / s * a};
    }
    Settings s_;
    float time_ = 0.f;
    bool active_ = false;
    std::vector<BoneTR> src_;
    std::vector<V3> vt_, vr_, hlT_, hlR_;
    std::vector<Q4> dir_;
};

}  // namespace utpp::mm::deadblend

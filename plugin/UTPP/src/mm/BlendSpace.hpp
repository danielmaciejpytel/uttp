// UTPP port report 4.2 #33: blend space sample weights as UE 5.8 UBlendSpace / UBlendSpace1D compute them.
// Header-only, no game hooks. References (UE 5.8 Engine/Source/Runtime/Engine/Private/Animation):
//   BlendSpace.cpp 2138-2145  GetNormalizedBlendInput (clamp to Min..Max; WrapInput not used by motion BS)
//   BlendSpace.cpp 2904-2980  FBlendSpaceData::GetSamples1D (segments, end clamp)
//   BlendSpace.cpp 2982-3100  FBlendSpaceData::GetSamples2D (containing triangle, FMath::GetBaryCentric2D)
//   BlendSpaceHelpers.cpp 57-105 / 141-195  Delaunay + AdjustEdgeDirections (Tangential: hypotenuse up-slope iff
//                                            mid.X * mid.Y < 0 in raw sample space)
//   BlendSpace.cpp 1951-1954, 2156-2172  SmoothWeight: FInterpConstantTo or CriticallyDampedSmoothing with
//                                            smoothing time 1/(e*speed) (bTargetWeightInterpolationEaseInOut)
//   Core/Public/Math/UnrealMathUtility.h 1553-1559, 1608-1648  InvExpApprox / CriticallyDampedSmoothing
// 2D support is limited to samples forming a full rectilinear grid (true for BS_Neutral_AO_Stand: yaw -90..90 step 45
// x pitch -90/0/90). For such a grid every Delaunay cell is an ambiguous rectangle, so UE's result is "split every
// cell by the diagonal the Tangential rule wants" -- which is what Build() reproduces. Anything else fails closed.
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "Smoothing.hpp"

namespace utpp::mm::bs {

struct Sample { float x = 0.f, y = 0.f; std::string anim; };
struct Weighted { int sample = -1; float weight = 0.f; };

struct BlendSpaceDef {
    std::string name;
    int dims = 1;
    float min[2] = {0.f, 0.f}, max[2] = {1.f, 1.f};
    float speedPerSec = 0.f;      // TargetWeightInterpolationSpeedPerSec
    bool easeInOut = true;        // bTargetWeightInterpolationEaseInOut
    std::vector<Sample> samples;
};

class BlendSpaceEval {
public:
    bool Build(const BlendSpaceDef& d) {
        def_ = d; xs_.clear(); ys_.clear(); grid_.clear(); ok_ = false;
        if (d.samples.empty()) return false;
        for (const auto& s : d.samples) { xs_.push_back(s.x); if (d.dims == 2) ys_.push_back(s.y); }
        auto uniq = [](std::vector<float>& v) { std::sort(v.begin(), v.end()); v.erase(std::unique(v.begin(), v.end()), v.end()); };
        uniq(xs_);
        if (d.dims == 1) { ok_ = xs_.size() == d.samples.size(); return ok_; }
        uniq(ys_);
        if (xs_.size() < 2 || ys_.size() < 2 || xs_.size() * ys_.size() != d.samples.size()) return false;
        grid_.assign(xs_.size() * ys_.size(), -1);
        for (int i = 0; i < (int)d.samples.size(); ++i) {
            const int ix = int(std::lower_bound(xs_.begin(), xs_.end(), d.samples[i].x) - xs_.begin());
            const int iy = int(std::lower_bound(ys_.begin(), ys_.end(), d.samples[i].y) - ys_.begin());
            int& slot = grid_[iy * xs_.size() + ix];
            if (slot >= 0) return false;
            slot = i;
        }
        ok_ = true;
        return true;
    }
    bool Ok() const { return ok_; }

    // Raw target weights (no smoothing). Weights <= 1e-5 (ZERO_ANIMWEIGHT_THRESH) are dropped like UE.
    std::vector<Weighted> Evaluate(float inX, float inY = 0.f) const {
        std::vector<Weighted> out;
        if (!ok_) return out;
        const float px = std::clamp(inX, def_.min[0], def_.max[0]);
        auto push = [&out](int s, float w) { if (w > 1e-5f) out.push_back({s, w}); };
        if (def_.dims == 1) {
            auto idxOf = [&](float x) { for (int i = 0; i < (int)def_.samples.size(); ++i) if (def_.samples[i].x == x) return i; return -1; };
            if (px <= xs_.front()) { push(idxOf(xs_.front()), 1.f); return out; }
            if (px >= xs_.back()) { push(idxOf(xs_.back()), 1.f); return out; }
            for (size_t k = 0; k + 1 < xs_.size(); ++k)
                if (px >= xs_[k] && px <= xs_[k + 1]) {
                    const float f = std::clamp((px - xs_[k]) / (xs_[k + 1] - xs_[k]), 0.f, 1.f);
                    push(idxOf(xs_[k]), 1.f - f); push(idxOf(xs_[k + 1]), f);
                    return out;
                }
            return out;
        }
        const float py = std::clamp(inY, def_.min[1], def_.max[1]);
        const int nx = (int)xs_.size();
        int cx = int(std::upper_bound(xs_.begin(), xs_.end(), px) - xs_.begin()) - 1;
        int cy = int(std::upper_bound(ys_.begin(), ys_.end(), py) - ys_.begin()) - 1;
        cx = std::clamp(cx, 0, nx - 2); cy = std::clamp(cy, 0, (int)ys_.size() - 2);
        const float x0 = xs_[cx], x1 = xs_[cx + 1], y0 = ys_[cy], y1 = ys_[cy + 1];
        const float u = (px - x0) / (x1 - x0), v = (py - y0) / (y1 - y0);
        const int s00 = grid_[cy * nx + cx], s10 = grid_[cy * nx + cx + 1];
        const int s01 = grid_[(cy + 1) * nx + cx], s11 = grid_[(cy + 1) * nx + cx + 1];
        const float mx = 0.5f * (x0 + x1), my = 0.5f * (y0 + y1);
        const bool upSlope = mx * my < 0.f;   // Tangential (BlendSpaceHelpers.cpp:164)
        if (upSlope) {                         // diagonal (x0,y0)-(x1,y1)
            if (u >= v) { push(s00, 1.f - u); push(s10, u - v); push(s11, v); }
            else        { push(s00, 1.f - v); push(s01, v - u); push(s11, u); }
        } else {                               // diagonal (x1,y0)-(x0,y1)
            if (u + v <= 1.f) { push(s00, 1.f - u - v); push(s10, u); push(s01, v); }
            else              { push(s10, 1.f - v); push(s01, 1.f - u); push(s11, u + v - 1.f); }
        }
        return out;
    }

private:
    BlendSpaceDef def_;
    std::vector<float> xs_, ys_;
    std::vector<int> grid_;
    bool ok_ = false;
};

// Per-sample target-weight smoothing (UBlendSpace::InterpolateWeightOfSampleData + SmoothWeight), one channel.
// Per-bone overrides (neck_01 4/s on the AO) = a second WeightSmoother with that speed.
class WeightSmoother {
public:
    void Reset(size_t numSamples) { w_.assign(numSamples, 0.f); r_.assign(numSamples, 0.f); primed_ = false; }
    // Returns smoothed, renormalized weights per sample index.
    const std::vector<float>& Update(const std::vector<Weighted>& target, float dt, float speed, bool easeInOut) {
        std::vector<float> t(w_.size(), 0.f);
        for (const auto& s : target) if (s.sample >= 0 && s.sample < (int)t.size()) t[s.sample] = s.weight;
        if (!primed_ || speed <= 0.f) { w_ = t; std::fill(r_.begin(), r_.end(), 0.f); primed_ = true; }
        else {
            const float st = 1.f / (2.71828182845904523536f * speed);
            for (size_t i = 0; i < w_.size(); ++i) {
                if (easeInOut) CriticallyDampedSmoothing(w_[i], r_[i], t[i], dt, st);
                else w_[i] = FInterpConstantTo(w_[i], t[i], dt, speed);
                if (w_[i] <= 1e-5f) { w_[i] = 0.f; r_[i] = 0.f; }
            }
        }
        float sum = 0.f; for (float v : w_) sum += v;
        out_ = w_;
        if (sum > 1e-6f) for (float& v : out_) v /= sum;
        return out_;
    }
private:
    std::vector<float> w_, r_, out_;
    bool primed_ = false;
};

}  // namespace utpp::mm::bs

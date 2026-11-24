#pragma once
// Speed converter, trajectory prediction (MapConfig::speedConverter bit kTrajectory, with kQuery): while the converted velocity differs
// from the game velocity (a turn the converter is still curving through), the future samples of the query trajectory are predicted the
// way the reference character movement would move on from the converted state: the converter itself (speedconv::Step: its friction,
// acceleration and braking) is integrated over the prediction horizon in short sub-steps, chasing the velocity the capsule keeps (the
// input direction, turned by the controller yaw rate of the prediction, at the current game speed, or, through a start (TrajectoryRamp),
// at the speed of the plain prediction in that interval, so the start ramp is kept). The plain prediction steps the
// ground movement in 0.1 s steps at the gait friction and the reference maximum speed, so it reaches the input direction sooner than the
// converted history that follows it does. History samples, sample times, heights (floor pass) and facings stay as they are. The
// converted prediction is blended in by the divergence of the converted and the game velocity, so a held loop (converted = game) keeps
// the plain prediction bit for bit.
#include <algorithm>
#include <cmath>
#include "MotionTrajectory.hpp"
#include "SpeedConverter.hpp"

namespace utpp::mm::speedconv
{
// MapConfig::speedConverter bit: the query prediction from the converted state (needs kQuery)
inline constexpr int kTrajectory = 256;

inline constexpr int kTrajSubsteps = 6;            // sub-steps per prediction interval (0.1 s -> 1/60 s, one game feed)
inline constexpr float kTrajFullAngleDeg = 5.0f;   // heading divergence (converted vs game) with the full converted prediction
inline constexpr float kTrajFullSpeedCmS = 25.0f;  // speed divergence with the full converted prediction
// below this share of the prediction speed cap the capsule is still starting (the plain prediction's ramp is chased); at or above it the
// game speed is held (a steady loop, a slower gait or an analog speed sits below the cap, which the plain prediction would drift to)
inline constexpr float kTrajStartFraction = 0.5f;

// The plain prediction's speed is chased (a start): a capsule start forecast shapes it, or the game speed is below kTrajStartFraction of
// the prediction speed cap (both cm/s on the query scale).
inline bool TrajectoryRamp(bool aStartForecast, float aGameCmS, float aCapCmS)
{
    return aStartForecast || (std::isfinite(aGameCmS) && std::isfinite(aCapCmS) && aGameCmS < kTrajStartFraction * aCapCmS);
}

// Weight of the converted prediction from the divergence of the converted and the game velocity (horizontal): 0 when they agree, 1 from
// kTrajFullAngleDeg of heading or kTrajFullSpeedCmS of speed difference, linear in between.
inline float TrajectoryWeight(motiontraj::V3 aConverted, motiontraj::V3 aGame)
{
#if UTPP_SPEED_CONVERTER_MUTATION == 30
    return 0.0f; // mutation 30: the plain prediction always
#endif
    const float sc = motiontraj::SizeXY(aConverted), sg = motiontraj::SizeXY(aGame);
    if (!std::isfinite(sc) || !std::isfinite(sg))
        return 0.0f;
    float w = std::fabs(sc - sg) / kTrajFullSpeedCmS;
    if (sc > motiontraj::kHasVelocity && sg > motiontraj::kHasVelocity)
    {
        const float err = std::fabs(motiontraj::NormalizeAxis(motiontraj::YawOf(aGame) - motiontraj::YawOf(aConverted)));
        w = std::max(w, err / kTrajFullAngleDeg);
    }
    return std::clamp(w, 0.0f, 1.0f);
}

// Horizontal offsets of the prediction samples from the current position (aOut[0] = 0, aOut[k] after k intervals): the converter state
// aState (copied, not changed) stepped kTrajSubsteps times per interval towards the velocity the capsule keeps. aGame: the game velocity
// of this feed (query scale, cm/s); aInputDir: the input direction (zero = the game velocity direction); aRotStepDeg: the controller yaw
// turn per prediction interval (the input turns with it, like the plain prediction); aSpeed[k]: the speed chased in interval k (cm/s);
// the remaining arguments as Step.
inline void PredictConvertedOffsets(const State& aState, motiontraj::V3 aGame, motiontraj::V3 aInputDir, float aRotStepDeg,
                                    const float (&aSpeed)[motiontraj::kPredictionCount], bool aInput, const Limits& aLimits,
                                    bool aShapeSpeed, bool aDirectionOnly, motiontraj::V3 (&aOut)[motiontraj::kPredictionCount + 1])
{
    using motiontraj::V3;
    State s = aState;
#if UTPP_SPEED_CONVERTER_MUTATION == 31
    Reset(s, aGame); // mutation 31: the prediction starts from the game velocity instead of the converted state
#endif
    const V3 dir0 = motiontraj::SafeNormal(motiontraj::IsZero(aInputDir) ? V3{aGame.x, aGame.y, 0.0f} : V3{aInputDir.x, aInputDir.y, 0.0f});
    const float h = motiontraj::kPredictionInterval / float(kTrajSubsteps);
    V3 p{};
    aOut[0] = p;
    for (int k = 0; k < motiontraj::kPredictionCount; ++k)
    {
        for (int j = 0; j < kTrajSubsteps; ++j)
        {
            const V3 dir = motiontraj::RotateYaw(dir0, aRotStepDeg * (float(k) + (float(j) + 0.5f) / float(kTrajSubsteps)));
            const V3 v = Step(s, dir * aSpeed[k], aInput, true, h, aLimits, aShapeSpeed, aDirectionOnly);
            p = p + V3{v.x, v.y, 0.0f} * h;
        }
        aOut[k + 1] = p;
    }
}

// The hook of one feed, after Trajectory::Update: blends the horizontal prediction positions of aTraj towards the converted prediction
// by TrajectoryWeight. aState is the converter state after this feed's Step (its velocity is the converted query velocity);
// aPlainSpeed: a start (TrajectoryRamp: the plain prediction's speed per interval is chased, else the game speed). Returns
// the weight applied (0 = the trajectory is unchanged).
inline float ApplyConvertedPrediction(motiontraj::Trajectory& aTraj, const State& aState, motiontraj::V3 aGame, motiontraj::V3 aInputDir,
                                      bool aInput, const Limits& aLimits, bool aShapeSpeed, bool aDirectionOnly, bool aPlainSpeed)
{
    if (!aState.valid || !aInput)
        return 0.0f;
    const float w = TrajectoryWeight(aState.vel, aGame);
    if (!(w > 0.0f))
        return 0.0f;
    // the speed chased per interval: the plain prediction's through a start (its ramp), else the game speed the capsule holds (the plain
    // prediction would drift to the reference gait cap)
#if UTPP_SPEED_CONVERTER_MUTATION == 32
    aPlainSpeed = false; // mutation 32: the current game speed over the whole horizon, also through a start
#endif
    float speed[motiontraj::kPredictionCount];
    for (int k = 0; k < motiontraj::kPredictionCount; ++k)
    {
        const motiontraj::V3 a = aTraj.Prediction(k).pos, b = aTraj.Prediction(k + 1).pos;
        speed[k] = aPlainSpeed ? motiontraj::SizeXY(b - a) / motiontraj::kPredictionInterval : motiontraj::SizeXY(aGame);
    }
    motiontraj::V3 off[motiontraj::kPredictionCount + 1];
    PredictConvertedOffsets(aState, aGame, aInputDir, aTraj.ControllerYawRate() * motiontraj::kPredictionInterval, speed, aInput, aLimits,
                            aShapeSpeed, aDirectionOnly, off);
    const motiontraj::V3 p0 = aTraj.Prediction(0).pos;
    motiontraj::V3 xy[motiontraj::kPredictionCount + 1];
    for (int k = 0; k <= motiontraj::kPredictionCount; ++k)
    {
        const motiontraj::V3 plain = aTraj.Prediction(k).pos;
        const motiontraj::V3 conv = p0 + off[k];
        xy[k] = motiontraj::V3{plain.x + (conv.x - plain.x) * w, plain.y + (conv.y - plain.y) * w, plain.z};
    }
    aTraj.SetPredictionXY(xy);
    return w;
}
} // namespace utpp::mm::speedconv

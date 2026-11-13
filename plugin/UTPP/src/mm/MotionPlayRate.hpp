// Wanted play rate of ordinary (non-event) playback.
// Reference behaviour: PoseSearchLibrary::CalculateWantedPlayRate and
// PoseSearchFeatureChannel_Trajectory::GetEstimatedSpeedRatio.
#pragma once

#include "MotionDatabase.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace utpp::mm::rate2
{
struct Result
{
    float wanted = 1.0f, ratio = 1.0f, querySpeed = 0.0f, poseSpeed = 0.0f;
    uint32_t channels = 0;
    bool valid = true;
};

inline Result Calculate(const motion::MotionDatabase& db, uint32_t database, uint32_t localRow,
                        const float* rawQuery, float minRate, float maxRate,
                        float trajectorySpeedMultiplier = 1.0f)
{
    Result out;
    const auto invalid = [] { Result r; r.valid = false; return r; };
    if (!db.Loaded() || database >= db.NumDatabases() || !db.DatabaseUsable(database))
        return invalid();
    const auto& d = db.Db(database);
    if (localRow >= d.numRows || !std::isfinite(minRate) || !std::isfinite(maxRate) ||
        minRate <= 1e-4f || minRate > maxRate)
        return invalid();
    if (std::fabs(maxRate - minRate) <= 1e-4f)
    {
        if (!std::isfinite(trajectorySpeedMultiplier)) return invalid();
        out.wanted = std::fabs(trajectorySpeedMultiplier) > 1e-4f
            ? minRate / trajectorySpeedMultiplier : minRate;
        return std::isfinite(out.wanted) && out.wanted > 0.0f ? out : invalid();
    }
    // UE leaves literal 1 for an absent cached query/trajectory parent (no clamp).
    if (!rawQuery) return out;
    const auto& schema = db.Schema(d.schema);
    const float* features = db.Features(database, localRow);
    const float* mean = db.NormMean(d.norm);
    const float* scale = db.NormScale(d.norm);
    // Current motion schemas have one trajectory parent, flattened into SCHC flags.
    // Multiple independent trajectory parents are not represented by this format.
    bool haveTrajectory = false;
    for (uint32_t i = 0; i < schema.numChannels; ++i)
    {
        const auto& c = db.Channel(schema.firstChannel + i);
        if (!(c.flags & motion::kChanTrajectory)) continue;
        haveTrajectory = true;
        if (c.kind != 2 || (c.flags & motion::kChanNormalizeOutput)) continue;
        float querySquare = 0.0f, poseSquare = 0.0f;
        for (uint32_t k = 0; k < c.cardinality; ++k)
        {
            const uint32_t o = c.dataOffset + k;
            if (o >= d.dims || !std::isfinite(scale[o]) || scale[o] <= 0.0f ||
                !std::isfinite(rawQuery[o])) return invalid();
            // DFEA = (raw - mean) * scale. Scale already contains channel weights.
            const float pose = features[o] / scale[o] + mean[o];
            querySquare += rawQuery[o] * rawQuery[o];
            poseSquare += pose * pose;
        }
        out.querySpeed += std::sqrt(querySquare);
        out.poseSpeed += std::sqrt(poseSquare);
        ++out.channels;
    }
    if (!std::isfinite(out.querySpeed) || !std::isfinite(out.poseSpeed)) return invalid();
    if (!haveTrajectory) return out;
    // UE's 1e-4 cm/s denominator guard in this format's metres/second.
    if (out.poseSpeed > 1e-6f) out.ratio = out.querySpeed / out.poseSpeed;
    out.wanted = std::clamp(out.ratio, minRate, maxRate);
    return out;
}
} // namespace utpp::mm::rate2

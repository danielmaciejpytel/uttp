#pragma once

#include "GameIntegration.hpp"

namespace utpp::mm::integ
{
// Animation callbacks may have multiple writers. Admit one with a single atomic operation;
// a busy caller drops its sample rather than waiting. Payload readers retain bounded seqlock reads.
template<typename T>
class GamePoseMailbox
{
public:
    bool TryPublish(const T& aValue)
    {
        if (m_writer.test_and_set(std::memory_order_acquire)) return false;
        m_buffer.Publish(aValue);
        m_writer.clear(std::memory_order_release);
        return true;
    }
    bool Read(T& aOut) const { return m_buffer.Read(aOut); }
private:
    std::atomic_flag m_writer = ATOMIC_FLAG_INIT;
    SeqlockBuffer<T> m_buffer;
};

// Called before BlendIntoRecords. Copies only mapped local transforms; no game pointers escape.
// The caller owns rig/thread admission. Missing mappings or invalid values reject the entire sample.
// aUnmappedBelow: bones below this index (Root / Trajectory of a v2 snapshot: gameIndex -1, never written or read by the vehicle
// transition, which works from firstBone) may be unmapped; they are stored as identity instead of rejecting the whole sample.
inline bool CaptureMappedGamePose(const uint8_t* aBones, uint32_t aCount, const PoseSnapshot& aMap,
                                  float* aOut, uint32_t aCapacity, uint32_t aUnmappedBelow = 0)
{
    if (!aBones || !aOut || !aMap.numBones || aMap.numBones > kMaxMMBones || aCapacity < aMap.numBones)
        return false;
    float pending[kMaxMMBones * 7];
    for (uint32_t b = 0; b < aMap.numBones; ++b)
    {
        const int32_t g = aMap.gameIndex[b];
        if (b < aUnmappedBelow && (g < 0 || static_cast<uint32_t>(g) >= aCount))
        {
            float* id = pending + b * 7;
            id[0] = id[1] = id[2] = id[3] = id[4] = id[5] = 0.0f;
            id[6] = 1.0f;
            continue;
        }
        if (g < 0 || static_cast<uint32_t>(g) >= aCount)
            return false;
        for (uint32_t other = 0; other < b; ++other)
            if (aMap.gameIndex[other] == g)
                return false;
        float record[8];
        std::memcpy(record, aBones + size_t(g) * kGameBoneStride, sizeof record);
        float* dst = pending + b * 7;
        for (uint32_t i = 0; i < 3; ++i) dst[i] = record[i];
        float norm = 0.0f;
        for (uint32_t i = 0; i < 4; ++i)
        {
            dst[3 + i] = record[kGameRotOffsetFloats + i];
            norm += dst[3 + i] * dst[3 + i];
        }
        for (uint32_t i = 0; i < 7; ++i)
            if (!std::isfinite(dst[i])) return false;
        if (!std::isfinite(norm) || norm < 1e-8f) return false;
    }
    std::memcpy(aOut, pending, size_t(aMap.numBones) * 7 * sizeof(float));
    return true;
}

struct GamePoseSample
{
    uint64_t rig = 0, generation = 0;
    int64_t ticks = 0;
    uint32_t count = 0, pad = 0;
    float pose[kMaxMMBones * 7] = {};
};
inline SeqlockBuffer<PoseSnapshot> g_gamePoseReadMap;
inline GamePoseMailbox<GamePoseSample> g_gamePoseSamples;
inline std::atomic<bool> g_gamePoseCaptureWanted{false};

inline bool CaptureGamePoseSample(const uint8_t* aBones, uint32_t aCount, uint64_t aRig, int64_t aTicks)
{
    if (!g_gamePoseCaptureWanted.load(std::memory_order_relaxed)) return false;
    PoseSnapshot map;
    if (!g_gamePoseReadMap.Read(map) || !map.rig || map.rig != aRig || !map.sequence || aTicks <= 0)
        return false;
    GamePoseSample sample;
    sample.rig = aRig; sample.generation = map.sequence; sample.ticks = aTicks; sample.count = map.numBones;
    if (!CaptureMappedGamePose(aBones, aCount, map, sample.pose, kMaxMMBones)) return false;
    return g_gamePoseSamples.TryPublish(sample);
}

inline bool FreshGamePoseSample(const GamePoseSample& aSample, uint64_t aRig, uint64_t aGeneration,
                                uint32_t aCount, int64_t aNow, int64_t aMaxAge)
{
    return aRig && aGeneration && aSample.rig == aRig && aSample.generation == aGeneration &&
           aSample.count == aCount && aCount > 0 && aCount <= kMaxMMBones && aSample.ticks > 0 &&
           aNow >= aSample.ticks && aMaxAge > 0 && aNow - aSample.ticks <= aMaxAge;
}
}

// UTPP 0.5.4 - speed bands for pivot / moving-turn clips. Header-only, deterministic, no Windows / game dependency
// (shared with the offline tests).
//
// In game (0.5.2, 2026-10-07 01:44-01:47, default / a former test database) walk-class pivots and turns were
// chosen while running: a non-package clip at 3.85 m/s (cost 98), a non-package clip (cost 66-112),
// M_Neutral_Walk_Turn_R_180_Rfoot at 3.05-3.86 m/s. The 0.3.8 rule allowed the own speed class AND the next slower one
// (fixed game thresholds 2.6 / 6 m/s); the 0.5.3 start rules only cover standstill (< 0.8 m/s).
//
// Band of the motion = class whose forward loops the current speed maps to:
//   - speed = max(intent, capsule) in game m/s, mapped into database m/s through the stand speed map (per database,
//     SpeedCalibration) -> the band edges are database speeds: edge(walk, run) = midpoint between the fastest forward
//     walk loop and the slowest forward run loop (when they do not overlap; otherwise between the class speeds), same for
//     run / sprint;
//   - hysteresis: up when above edge * (1 + hysteresis), down when below edge * (1 - hysteresis) for downHold s; no
//     down-step while an animation-led turn is in progress with the stick held (a flick decelerates the capsule through
//     the walk band mid-turn); without stick input the band follows the speed at once (pivots / turns are excluded
//     then anyway; the next start begins in the band of its own speed).
// Reface starts are speed-classed too (steer::SpeedVariantClass): while the capsule moves (>= refaceSpeed) only the
// band's own reface clips stay candidates.
// Data fallback: a band whose class has no pivot (or no moving-turn) clips in the stand clip set allows the next slower
// class of that kind (a former test database: no sprint pivots -> run pivots at a sprint). A walk band never falls back.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>

#include "PlaybackRate.hpp"
#include "SpeedCalibration.hpp"
#include "Steering.hpp"

namespace utpp::mm
{
struct BandEdges
{
    float walkRun = 3.83f;   // database m/s
    float runSprint = 6.56f; // database m/s
    bool sprintData = true;  // the database has measured sprint loops (false: sprint speed is a fallback)
};

inline float BandEdge(const SpeedClassResult& a, const SpeedClassResult& b)
{
    const bool measured = (a.source == 'm' || a.source == 'a') && (b.source == 'm' || b.source == 'a') && a.clips > 0 && b.clips > 0;
    if (measured && a.maxSpeed < b.minSpeed)
        return 0.5f * (a.maxSpeed + b.minSpeed);
    return 0.5f * (a.speed + b.speed);
}

inline BandEdges ComputeBandEdges(const SpeedCalibration& aCal)
{
    BandEdges e;
    e.walkRun = BandEdge(aCal.cls[kSpeedWalk], aCal.cls[kSpeedRun]);
    e.runSprint = std::fmax(BandEdge(aCal.cls[kSpeedRun], aCal.cls[kSpeedSprint]), e.walkRun + 0.1f);
    e.sprintData = (aCal.cls[kSpeedSprint].source == 'm' || aCal.cls[kSpeedSprint].source == 'a' || aCal.cls[kSpeedSprint].source == 'o') &&
                   (aCal.cls[kSpeedSprint].clips > 0 || aCal.cls[kSpeedSprint].source == 'o');
    return e;
}

struct BandConfig
{
    float hysteresis = 0.08f; // relative to the edge
    float downHold = 0.35f;   // s below the lower edge before a down-step
};

class SpeedBandTracker
{
public:
    void Reset() { *this = SpeedBandTracker{}; }
    // aDbSpeed: max(intent, capsule) mapped into database m/s. aHeld: stick held; aTurning: animation-led turn in progress.
    int Update(float aDbSpeed, bool aHeld, bool aTurning, float aDt, const BandEdges& e, const BandConfig& c = {})
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
        const float v = std::isfinite(aDbSpeed) ? aDbSpeed : 0.0f;
        const float up = 1.0f + c.hysteresis, dn = 1.0f - c.hysteresis;
        const int raw = v < e.walkRun ? 1 : (v < e.runSprint ? 2 : 3);
        if (m_band == 0)
        {
            m_band = raw;
            m_below = 0.0f;
            return m_band;
        }
        if (!aHeld) // no pivots / turns without input: follow the speed (a new start begins in the band of its speed)
        {
            m_below = 0.0f;
            if (raw != m_band)
                ++m_changes;
            m_band = raw;
            return m_band;
        }
        // up-steps at once
        int target = m_band;
        while (target < 3 && v > (target == 1 ? e.walkRun : e.runSprint) * up)
            ++target;
        if (target > m_band)
        {
            m_band = target;
            m_below = 0.0f;
            ++m_changes;
            return m_band;
        }
        // down-steps after downHold below the lower edge (not during an animation-led turn)
        const float lower = (m_band == 3 ? e.runSprint : (m_band == 2 ? e.walkRun : 0.0f)) * dn;
        if (m_band > 1 && v < lower && !aTurning)
        {
            m_below += dt;
            if (m_below >= c.downHold)
            {
                m_band = raw; // v < lower edge -> raw < m_band
                m_below = 0.0f;
                ++m_changes;
            }
        }
        else
            m_below = 0.0f;
        return m_band;
    }
    int Band() const { return m_band; }
    uint64_t Changes() const { return m_changes; }

private:
    int m_band = 0;
    float m_below = 0.0f;
    uint64_t m_changes = 0;
};

// Pivot / moving-turn clips per speed class (1 walk, 2 run, 3 sprint) in a clip set; [0] unused.
struct BandAvailability
{
    int pivots[4] = {0, 0, 0, 0};
    int turns[4] = {0, 0, 0, 0};
    bool PivotFallback(int aBand) const { return aBand >= 2 && aBand <= 3 && pivots[aBand] == 0; }
    bool TurnFallback(int aBand) const { return aBand >= 2 && aBand <= 3 && turns[aBand] == 0; }
    std::string Summary() const
    {
        char buf[96];
        std::snprintf(buf, sizeof buf, "pivots w%d r%d s%d turns w%d r%d s%d", pivots[1], pivots[2], pivots[3], turns[1], turns[2], turns[3]);
        return buf;
    }
};

// aClasses: steer::ClassifyClip per clip; aTagMasks: clip tag masks; aSetMask: clip set (stand) tag mask.
inline BandAvailability ComputeBandAvailability(std::span<const uint8_t> aClasses, std::span<const uint64_t> aTagMasks, uint64_t aSetMask)
{
    using namespace steer;
    BandAvailability a;
    for (size_t i = 0; i < aClasses.size() && i < aTagMasks.size(); ++i)
    {
        if (!(aTagMasks[i] & aSetMask))
            continue;
        switch (aClasses[i])
        {
        case kClassPivotWalk: ++a.pivots[1]; break;
        case kClassPivotRun: ++a.pivots[2]; break;
        case kClassPivotSprint: ++a.pivots[3]; break;
        case kClassTurnWalk: ++a.turns[1]; break;
        case kClassTurnRun: ++a.turns[2]; break;
        case kClassTurnSprint: ++a.turns[3]; break;
        default: break;
        }
    }
    return a;
}
} // namespace utpp::mm

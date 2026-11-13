// UTPP 0.5.9 (Phase 5.3): capsule <-> animation synchronisation helpers. Header-only, C++20.
//
// 1. Capsule acceleration profile (UTPPSetMMCapsuleSync): measured per database at load (like SpeedCalibration) from the root
//    speed profiles of the forward start clips ("_<Walk|Run|Sprint|Crouch>_Start_F" + end or '_') and stop clips
//    ("_<cls>_Stop_F"). Start: 10 -> 90 % of the clip's peak speed, mean acceleration 0.8 * peak / (t90 - t10). Stop: from the
//    first frame below 90 % of the peak (after the peak) to the first frame below 10 %, mean deceleration. Database m/s^2 are
//    converted to the game with the speed map: a_game = a_db * (G / D)^2 (G game speed of the class, D calibrated database loop
//    speed; playback rate G / D applied to time and distance); the 0.3.3 defaults (
//    3.75 / 12.2 / 6.96 m/s^2 for the motion dense database). Median over the clips of a class; classes without clips keep the
//    0.5.8 defaults (acceleration) / no change (deceleration).
// 2. Trajectory spring calibration (UTPPSetMMTrajCal): the predicted velocity spring is a critically damped spring (half-life h,
//    50 % of a step after 1.21 h); the capsule ramps linearly at a (50 % after v / 2a). Matching the 50 % time gives the
//    real-time half-life h = 0.413 * v / a; the trajectory runs in matcher time, so h_matcher = h * rate (rate = G / D).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "SpeedCalibration.hpp"

namespace utpp::mm
{
struct CapsuleClassProfile
{
    float accelDb = 0.0f, decelDb = 0.0f;   // database m/s^2 (median), 0 = not measured
    float accelGame = 0.0f, decelGame = 0.0f; // game m/s^2 (speed map), 0 = not measured
    int startClips = 0, stopClips = 0;
    float startT90 = 0.0f;                  // s database time (median)
};

struct CapsuleProfile
{
    CapsuleClassProfile cls[kSpeedClassCount];
    // per locomotion state (0 none, 1 stand, 2 crouch, 3 sprint): game targets, 0 = keep the 0.5.8 value / vanilla
    float accel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float decel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float trajHalflife[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // matcher-time velocity half-life, 0 = configured default

    std::string Summary() const
    {
        char buf[512];
        int n = 0;
        for (int k = 0; k < kSpeedClassCount && n >= 0 && n < int(sizeof buf); ++k)
            n += std::snprintf(buf + n, sizeof buf - size_t(n), "%s start %.2f(%d) stop %.2f(%d) | ", kSpeedClassName[k],
                               double(cls[k].accelGame), cls[k].startClips, double(cls[k].decelGame), cls[k].stopClips);
        if (n >= 0 && n < int(sizeof buf))
            std::snprintf(buf + n, sizeof buf - size_t(n), "accel %.2f/%.2f/%.2f decel %.2f/%.2f/%.2f trajHl %.3f/%.3f/%.3f",
                          double(accel[1]), double(accel[2]), double(accel[3]), double(decel[1]), double(decel[2]),
                          double(decel[3]), double(trajHalflife[1]), double(trajHalflife[2]), double(trajHalflife[3]));
        return buf;
    }
};

// "_<cls>_<kind>_F" followed by the end or '_' (excludes FL / FR / F_R_090 reface variants are not Start_F_<digit>... names
// with "Reface" / "Turn" / "Pivot" are skipped explicitly).
inline bool IsForwardKindName(std::string_view aName, std::string_view aCls, std::string_view aKind)
{
    if (aName.find("Reface") != std::string_view::npos || aName.find("Turn") != std::string_view::npos ||
        aName.find("Pivot") != std::string_view::npos)
        return false;
    std::string pat = "_";
    pat += aCls;
    pat += "_";
    pat += aKind;
    pat += "_F";
    size_t p = aName.find(pat);
    while (p != std::string_view::npos)
    {
        const size_t e = p + pat.size();
        if (e == aName.size() || aName[e] == '_')
            return true;
        p = aName.find(pat, p + 1);
    }
    return false;
}

// Smoothed (3-frame) planar root speed per frame (database m/s), loop-duplicate frames skipped.
inline std::vector<float> ClipSpeedProfile(const Database& aDb, uint32_t aClip)
{
    std::vector<float> raw;
    const auto clips = aDb.Clips();
    if (aClip >= clips.size() || aDb.Fps() <= 0.0f)
        return raw;
    const ClipRecord& c = clips[aClip];
    for (uint32_t f = c.firstFrame; f < c.firstFrame + c.numFrames; ++f)
    {
        if (aDb.FrameFlagsOf(f) & kFrameLoopDuplicate)
            continue;
        const RootDelta* r = aDb.Root(f);
        if (!r)
            continue;
        raw.push_back(std::sqrt(r->dx * r->dx + r->dy * r->dy) * aDb.Fps());
    }
    std::vector<float> s(raw.size(), 0.0f);
    for (size_t i = 0; i < raw.size(); ++i)
    {
        const size_t a = i > 0 ? i - 1 : 0, b = std::min(raw.size() - 1, i + 1);
        float sum = 0.0f;
        for (size_t j = a; j <= b; ++j)
            sum += raw[j];
        s[i] = sum / float(b - a + 1);
    }
    return s;
}

// Start: mean acceleration 10 -> 90 % of the peak (db m/s^2); 0 if not measurable. aT90 = s from the clip start.
inline float StartAccelDb(const std::vector<float>& v, float aFps, float& aT90)
{
    aT90 = 0.0f;
    if (v.size() < 4 || aFps <= 0.0f)
        return 0.0f;
    const float peak = *std::max_element(v.begin(), v.end());
    if (peak < 0.3f)
        return 0.0f;
    int i10 = -1, i90 = -1;
    for (size_t i = 0; i < v.size(); ++i)
    {
        if (i10 < 0 && v[i] >= 0.1f * peak)
            i10 = int(i);
        if (i90 < 0 && v[i] >= 0.9f * peak)
        {
            i90 = int(i);
            break;
        }
    }
    if (i10 < 0 || i90 <= i10)
        return 0.0f;
    aT90 = float(i90) / aFps;
    return 0.8f * peak / (float(i90 - i10) / aFps);
}

// Stop: mean deceleration from the first frame below 90 % of the peak (after the peak) to the first frame below 10 %.
inline float StopDecelDb(const std::vector<float>& v, float aFps)
{
    if (v.size() < 4 || aFps <= 0.0f)
        return 0.0f;
    const size_t ip = size_t(std::max_element(v.begin(), v.end()) - v.begin());
    const float peak = v[ip];
    if (peak < 0.3f)
        return 0.0f;
    int ihi = -1, ilo = -1;
    for (size_t i = ip; i < v.size(); ++i)
    {
        if (ihi < 0 && v[i] < 0.9f * peak)
            ihi = int(i);
        if (ihi >= 0 && v[i] < 0.1f * peak)
        {
            ilo = int(i);
            break;
        }
    }
    if (ihi < 0 || ilo <= ihi)
        return 0.0f;
    return 0.8f * peak / (float(ilo - ihi) / aFps);
}

// aStandMask / aCrouchMask: clip tag masks of the stand and crouch sets (driver state masks); aCal: the database's speed
// calibration (game / database loop speeds).
inline CapsuleProfile DeriveCapsuleProfile(const Database& aDb, uint64_t aStandMask, uint64_t aCrouchMask, const SpeedCalibration& aCal)
{
    CapsuleProfile p;
    const auto clips = aDb.Clips();
    const float gameSpeed[kSpeedClassCount] = {kGameStandPoints[0], kGameStandPoints[1], kGameStandPoints[2], kGameCrouchPoint};
    for (int k = 0; k < kSpeedClassCount; ++k)
    {
        const uint64_t mask = k == kSpeedCrouch ? aCrouchMask : aStandMask;
        std::vector<float> acc, dec, t90s;
        for (uint32_t i = 0; i < clips.size(); ++i)
        {
            if (!(clips[i].tagMask & mask))
                continue;
            const std::string_view name = aDb.ClipName(i);
            const bool start = IsForwardKindName(name, kSpeedClassName[k], "Start");
            const bool stop = !start && IsForwardKindName(name, kSpeedClassName[k], "Stop");
            if (!start && !stop)
                continue;
            const std::vector<float> v = ClipSpeedProfile(aDb, i);
            if (start)
            {
                float t90 = 0.0f;
                const float a = StartAccelDb(v, aDb.Fps(), t90);
                if (a > 0.0f && std::isfinite(a))
                {
                    acc.push_back(a);
                    t90s.push_back(t90);
                }
            }
            else
            {
                const float d = StopDecelDb(v, aDb.Fps());
                if (d > 0.0f && std::isfinite(d))
                    dec.push_back(d);
            }
        }
        CapsuleClassProfile& c = p.cls[k];
        const float D = aCal.cls[k].speed;
        const float r = D > 0.1f ? gameSpeed[k] / D : 0.0f;
        if (!acc.empty() && r > 0.0f)
        {
            c.accelDb = Median(acc);
            c.startT90 = Median(t90s);
            c.startClips = int(acc.size());
            c.accelGame = c.accelDb * r * r;
        }
        if (!dec.empty() && r > 0.0f)
        {
            c.decelDb = Median(dec);
            c.stopClips = int(dec.size());
            c.decelGame = c.decelDb * r * r;
        }
    }
    // states: stand = Run class (game run 3.85 is the stand top speed), crouch = Crouch, sprint = Sprint
    const int stateClass[4] = {-1, kSpeedRun, kSpeedCrouch, kSpeedSprint};
    for (int s = 1; s < 4; ++s)
    {
        const CapsuleClassProfile& c = p.cls[stateClass[s]];
        if (c.accelGame > 0.0f)
            p.accel[s] = std::clamp(c.accelGame, 1.5f, 30.0f);
        if (c.decelGame > 0.0f)
            p.decel[s] = std::clamp(c.decelGame, 3.0f, 40.0f); // the driver applies a 10 m/s^2 gameplay floor
        const float G = s == 2 ? kGameCrouchPoint : (s == 3 ? kGameStandPoints[2] : kGameStandPoints[1]);
        const float D = aCal.cls[stateClass[s]].speed;
        const float a = p.accel[s];
        if (a > 0.0f && D > 0.1f)
            p.trajHalflife[s] = std::clamp(0.413f * G / a * (G / D), 0.08f, 0.6f);
    }
    return p;
}
} // namespace utpp::mm

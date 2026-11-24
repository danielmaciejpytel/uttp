// UTPP motion matching runtime - steps fitted to stair treads (option mm.stairSteps, default off). Header-only, deterministic,
// allocation-free.
//
// On a stair flight a loop clip keeps its own step length (a walk step is about 1.1 m, a run step about 1.7 m), so the feet land
// wherever the step ends: on a tread, on a nose or between two treads. This option fits the step to the treads instead: the step
// length becomes a whole number of treads and the step rate rises by the same factor, so the movement speed stays the game's.
//   - Detector: the ground traces of the feet (ball and heel, mm/GroundFeetRuntime.hpp) are kept as world points for windowS seconds.
//     Projected on the travel direction they show the stair profile: neighbouring points (at most pairMaxDs apart along the travel)
//     whose heights differ by minRise .. maxRise straddle a nose. Two or more noses of the same rise sign give the tread depth
//     (median spacing); a least-squares fit of the staircase z = z0 + rise * floor((s - phase) / tread) over the points refines the
//     tread and gives the phase of the noses. The fit is used when its residual stays under maxRms and the newest points (recentS)
//     still span a rise (V is on the flight, not on the landing after it).
//   - Fit of the step: the world step length L of the playing loop (capsule speed x clip step time / play rate) is shortened to n
//     treads, n the smallest whole number with n x tread >= minScale x L (the stride warp floor of the stairs); the play rate rises
//     by L / (n x tread) (the cadence), the stride warp (StrideWarp.hpp, scale = capsule speed / clip root speed) then shortens the
//     step by the same factor. A step shorter than minScale x L is not reachable (the knees and the shin skinning of a step shorter
//     than about half of the clip's own) and a step is never lengthened: no whole n fits = no change.
//   - The weight fades in / out over weightTime; the matcher ramps the cadence in on a slower time constant than the stride warp, so
//     the step shortens before the rate leaves it behind.
//   - Landing spot: the n x tread fit keeps the landing spot from step to step but does not choose it. At every landing the matcher
//     reads the landing foot's distance from the nearest tread centre (TreadPhaseError, FeedOut::phaseErr) and quickens or slows the
//     next step by a part of it, so the following foot lands nearer the centre; no foot is moved sideways or along the stairs.
// Off (the default) or without stairs: no point is stored, the weight is 0 and the cadence stays exactly 1 (bit for bit).
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#ifndef UTPP_STAIR_MIN_SCALE
// Shortest step on stairs as a fraction of the clip's own (Config::minScale); a build may override it for comparison.
#define UTPP_STAIR_MIN_SCALE 0.5f
#endif

namespace utpp::mm::stairsteps
{

struct Config
{
    float minRise = 0.06f;    // m, smallest stair rise
    float maxRise = 0.30f;    // m, largest stair rise
    float minTread = 0.18f;   // m, smallest tread depth
    float maxTread = 0.55f;   // m, largest tread depth
    float pairMaxDs = 0.15f;  // m, neighbouring points at most this far apart along the travel straddle a nose
    float clusterGap = 0.08f; // m, nose candidates closer than this are one nose
    float windowS = 1.5f;     // s, oldest point used
    float behindM = 1.6f;     // m, points used behind the capsule (along the travel)
    float aheadM = 1.0f;      // m, ... and ahead of it
    float minSpacing = 0.03f; // m, a trace slot stores a new point after moving at least this far
    float maxRms = 0.03f;     // m, largest staircase fit residual
    float recentS = 0.4f;     // s, the newest points must span a rise
    float minSpeed = 0.5f;    // m/s, slowest travel the detector reads (direction of travel)
    float fitPeriodS = 0.05f; // s between two fits
    float lostS = 0.3f;       // s without a valid fit before the weight falls
    float weightTime = 0.3f;  // s, full-scale rise / fall of the weight
    float minScale = UTPP_STAIR_MIN_SCALE; // shortest step on stairs (fraction of the clip's own)
};

struct Point
{
    float x = 0.0f, y = 0.0f, z = 0.0f; // world
    float t = 0.0f;                     // detector clock (s)
};

// Stair flight under V along the travel direction (dirX, dirY): tread depth, signed rise (> 0 up), a world point on a nose line.
struct Estimate
{
    bool valid = false;
    float tread = 0.0f, rise = 0.0f;
    float dirX = 0.0f, dirY = 1.0f;
    float noseX = 0.0f, noseY = 0.0f;
    float rms = 0.0f;
    int points = 0;
    int noses = 0;
};

// Nose line nearest to the world point (x, y) along the estimate's direction: signed distance (m) of the point past that nose
// (0 .. tread: the point is on the tread above an upward nose / below a downward one).
inline float PastNose(const Estimate& e, float x, float y)
{
    if (!e.valid || !(e.tread > 0.0f))
        return 0.0f;
    const float s = (x - e.noseX) * e.dirX + (y - e.noseY) * e.dirY;
    return s - e.tread * std::floor(s / e.tread);
}

// Whole treads per step and the cadence (play rate factor, >= 1) for a step of length aStepLen (m) on treads of aTread (m): the
// smallest n with n x tread >= minScale x step and n x tread <= step; a previous n (aPrev) is kept while it still fits and the smaller
// one only just does (within 3 %), so a step length near a boundary does not switch between two counts. treads 0 = no fit.
struct StrideFit
{
    int treads = 0;
    float cadence = 1.0f;
};
inline StrideFit FitStride(float aStepLen, float aTread, float aMinScale, int aPrev = 0)
{
    StrideFit r;
    if (!std::isfinite(aStepLen) || !std::isfinite(aTread) || !std::isfinite(aMinScale) || !(aStepLen > 0.0f) || !(aTread > 0.0f) ||
        !(aMinScale > 0.0f) || aMinScale > 1.0f)
        return r;
    const int nMin = std::max(1, int(std::ceil(aMinScale * aStepLen / aTread - 1.0e-4f)));
    const int nMax = int(std::floor(aStepLen / aTread + 1.0e-4f));
    if (nMin > nMax)
        return r;
    int n = nMin;
    if (aPrev == nMin + 1 && aPrev <= nMax && float(nMin) * aTread < 1.03f * aMinScale * aStepLen)
        n = aPrev;
    r.treads = n;
    r.cadence = std::fmax(1.0f, aStepLen / (float(n) * aTread));
    return r;
}

class Detector
{
public:
    static constexpr int kCapacity = 128;
    static constexpr int kSlots = 4; // ball left / right, heel left / right

    void Reset()
    {
        m_count = 0;
        m_head = 0;
        for (bool& h : m_haveSlot)
            h = false;
    }
    int Count() const { return m_count; }

    // One trace hit of slot aSlot (0..3) at detector time aT; stored when the slot moved at least minSpacing since its last point.
    void Add(const Config& c, int aSlot, float aX, float aY, float aZ, float aT)
    {
        if (aSlot < 0 || aSlot >= kSlots || !std::isfinite(aX) || !std::isfinite(aY) || !std::isfinite(aZ) || !std::isfinite(aT))
            return;
        if (m_haveSlot[aSlot])
        {
            const float dx = aX - m_slot[aSlot].x, dy = aY - m_slot[aSlot].y;
            if (dx * dx + dy * dy < c.minSpacing * c.minSpacing && std::fabs(aZ - m_slot[aSlot].z) < 0.5f * c.minRise)
                return;
        }
        const Point p{aX, aY, aZ, aT};
        m_slot[aSlot] = p;
        m_haveSlot[aSlot] = true;
        m_pts[m_head] = p;
        m_head = (m_head + 1) % kCapacity;
        m_count = std::min(m_count + 1, kCapacity);
    }

    // Staircase fit of the stored points around the capsule (aCapX, aCapY) along the unit travel direction (aDirX, aDirY) at time aNow.
    Estimate Fit(const Config& c, float aCapX, float aCapY, float aDirX, float aDirY, float aNow) const
    {
        Estimate e;
        e.dirX = aDirX;
        e.dirY = aDirY;
        float s[kCapacity], z[kCapacity];
        int n = 0;
        float recentLo = 1.0e9f, recentHi = -1.0e9f;
        for (int k = 0; k < m_count; ++k)
        {
            const Point& p = m_pts[k];
            if (aNow - p.t > c.windowS)
                continue;
            const float sp = (p.x - aCapX) * aDirX + (p.y - aCapY) * aDirY;
            if (sp < -c.behindM || sp > c.aheadM)
                continue;
            // insertion by s (at most kCapacity points)
            int i = n++;
            while (i > 0 && s[i - 1] > sp)
            {
                s[i] = s[i - 1];
                z[i] = z[i - 1];
                --i;
            }
            s[i] = sp;
            z[i] = p.z;
            if (aNow - p.t <= c.recentS)
            {
                recentLo = std::fmin(recentLo, p.z);
                recentHi = std::fmax(recentHi, p.z);
            }
        }
        e.points = n;
        if (n < 6 || !(recentHi - recentLo >= c.minRise))
            return e;
        // nose candidates: neighbouring points along the travel with a rise between them
        float cs[kCapacity], cr[kCapacity];
        int nc = 0, up = 0;
        for (int i = 0; i + 1 < n; ++i)
        {
            const float ds = s[i + 1] - s[i], dz = z[i + 1] - z[i];
            if (ds <= c.pairMaxDs && std::fabs(dz) >= c.minRise && std::fabs(dz) <= c.maxRise)
            {
                cs[nc] = 0.5f * (s[i] + s[i + 1]);
                cr[nc] = dz;
                up += dz > 0.0f ? 1 : 0;
                ++nc;
            }
        }
        if (nc < 2)
            return e;
        const int sign = 2 * up >= nc ? 1 : -1;
        const int same = sign > 0 ? up : nc - up;
        if (5 * same < 4 * nc) // at least 80 % of the candidates rise the same way
            return e;
        // rise: median of the same-sign candidates; noses: candidates merged within clusterGap
        float rs[kCapacity];
        int nr = 0;
        float noses[kCapacity];
        int nn = 0;
        float sum = 0.0f;
        int cnt = 0;
        float last = -1.0e9f;
        for (int k = 0; k < nc; ++k)
        {
            if ((cr[k] > 0.0f) != (sign > 0))
                continue;
            rs[nr++] = std::fabs(cr[k]);
            if (cnt > 0 && cs[k] - last > c.clusterGap)
            {
                noses[nn++] = sum / float(cnt);
                sum = 0.0f;
                cnt = 0;
            }
            sum += cs[k];
            ++cnt;
            last = cs[k];
        }
        if (cnt > 0)
            noses[nn++] = sum / float(cnt);
        e.noses = nn;
        if (nn < 2)
            return e;
        std::sort(rs, rs + nr);
        const float rise = float(sign) * rs[nr / 2];
        float sp[kCapacity];
        int ns = 0;
        for (int k = 0; k + 1 < nn; ++k)
        {
            const float d = noses[k + 1] - noses[k];
            if (d >= c.minTread && d <= c.maxTread)
                sp[ns++] = d;
        }
        if (ns < 1)
            return e;
        std::sort(sp, sp + ns);
        const float t0 = sp[ns / 2];
        // least-squares staircase: tread around t0, phase over one tread
        float bestCost = 1.0e30f, bestT = t0, bestP = 0.0f;
        auto cost = [&](float aT, float aP) {
            double sr = 0.0, sr2 = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const float r = z[i] - rise * std::floor((s[i] - aP) / aT);
                sr += r;
                sr2 += double(r) * r;
            }
            return float(sr2 - sr * sr / n);
        };
        for (int dt = -3; dt <= 3; ++dt)
        {
            const float t = t0 + 0.01f * float(dt);
            if (t < c.minTread || t > c.maxTread)
                continue;
            for (float p = 0.0f; p < t; p += 0.02f)
            {
                const float k = cost(t, p);
                if (k < bestCost)
                    bestCost = k, bestT = t, bestP = p;
            }
        }
        // fine grid around it; the cost of a staircase is flat between two neighbouring points, so the middle of the cheapest cells
        // (not the first one) is taken: sparse points (a run at 60 Hz, 6 cm apart) otherwise left the noses up to 5 cm off
        {
            float costs[9][13];
            float fineBest = 1.0e30f;
            for (int dt = -4; dt <= 4; ++dt)
                for (int dp = -6; dp <= 6; ++dp)
                {
                    const float t = bestT + 0.0025f * float(dt);
                    costs[dt + 4][dp + 6] = t >= c.minTread && t <= c.maxTread ? cost(t, bestP + 0.005f * float(dp)) : 1.0e30f;
                    fineBest = std::fmin(fineBest, costs[dt + 4][dp + 6]);
                }
            const float tie = fineBest + std::fmax(1.0e-6f, 1.0e-3f * fineBest);
            double sumT = 0.0, sumP = 0.0;
            int ties = 0;
            for (int dt = -4; dt <= 4; ++dt)
                for (int dp = -6; dp <= 6; ++dp)
                    if (costs[dt + 4][dp + 6] <= tie)
                    {
                        sumT += 0.0025 * dt;
                        sumP += 0.005 * dp;
                        ++ties;
                    }
            if (ties > 0 && fineBest <= bestCost)
            {
                bestT += float(sumT / ties);
                bestP += float(sumP / ties);
                bestCost = cost(bestT, bestP);
            }
        }
        e.rms = std::sqrt(std::fmax(bestCost, 0.0f) / float(n));
        if (!(e.rms <= c.maxRms))
            return e;
        e.valid = true;
        e.tread = bestT;
        e.rise = rise;
        e.noseX = aCapX + aDirX * bestP;
        e.noseY = aCapY + aDirY * bestP;
        return e;
    }

private:
    Point m_pts[kCapacity];
    int m_head = 0, m_count = 0;
    Point m_slot[kSlots];
    bool m_haveSlot[kSlots] = {false, false, false, false};
};

// Distance (m) of a point from the nearest tread centre along the flight (the travel direction of the estimate), within [-tread / 2,
// tread / 2]; > 0 = past the centre. The tread centres lie half a tread from the noses.
inline float TreadPhaseError(const Estimate& aEst, float aX, float aY)
{
    if (!aEst.valid || !(aEst.tread > 1.0e-3f))
        return 0.0f;
    const float s = (aX - aEst.noseX) * aEst.dirX + (aY - aEst.noseY) * aEst.dirY - 0.5f * aEst.tread;
    return s - aEst.tread * std::round(s / aEst.tread);
}

// The stair flight handed to the ground feet stage (animation thread) with the ground traces of a frame (GroundFeet Samples): the
// estimate, its weight and the treads per step of the playing fit. valid 0 = no flight (the option off: always 0).
struct Flight
{
    uint32_t valid = 0;
    float weight = 0.0f;
    float tread = 0.0f, rise = 0.0f;
    float dirX = 0.0f, dirY = 1.0f;
    float noseX = 0.0f, noseY = 0.0f;
    int32_t treads = 0;
};
static_assert(sizeof(Flight) % 4 == 0);

// Swing of a foot over a stair flight (the ground feet stage, option mm.stairSteps): a foot that leaves a tread moves to the tread it
// will land on along one smooth curve over the swing, from the moment it lifts off, instead of keeping the tread it left until it is
// about to land. Pure functions of the flight, positions along it (s, m, from the flight's nose line along its direction) and heights
// (world, m).
namespace swing
{
inline float Along(const Flight& f, float aX, float aY) { return (aX - f.noseX) * f.dirX + (aY - f.noseY) * f.dirY; }
inline float TreadIndex(const Flight& f, float aS) { return std::floor(aS / f.tread); }
// Minimum-jerk blend 0 -> 1 (zero rate and acceleration at both ends).
inline float MinJerk(float aU)
{
    const float u = std::clamp(aU, 0.0f, 1.0f);
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}
// Smooth maximum of two heights: max(a, b) once they are more than aEps apart, a when they are equal, without a kink in between.
inline float SmoothMax(float aA, float aB, float aEps)
{
    const float d = aA - aB;
    return 0.5f * (aA + aB + std::sqrt(d * d + aEps * aEps) - aEps);
}
// Height of the line through the nose tips of a flight going up at s, for a support foot on tread aKs at height aHs: a toe kept above
// it clears every nose it passes.
inline float NoseLineUp(const Flight& f, float aS, float aHs, float aKs) { return aHs + f.rise * (aS / f.tread - aKs); }
// Tread height under s of a flight going down, each drop at a nose spread over aRamp treads after the nose (continuous in s): a heel
// or ball kept above it does not cut into the tread it is leaving.
inline float TreadDownRamped(const Flight& f, float aS, float aHs, float aKs, float aRamp)
{
    const float q = aS / f.tread, k = std::floor(q);
    const float frac = q - k;
    const float back = aRamp > 1.0e-3f ? 1.0f - std::clamp(frac / aRamp, 0.0f, 1.0f) : 0.0f;
    return aHs + f.rise * (k - aKs) - f.rise * back;
}

struct Plan
{
    float h0 = 0.0f;    // world height of the tread the foot left (its planting plane at the lift-off)
    float h1 = 0.0f;    // world height of the tread it lands on
    float s0 = 0.0f;    // mid-sole along the flight at the lift-off
    float sLand = 0.0f; // mid-sole along the flight at the landing
    float hs = 0.0f;    // support foot's plane height (world) and its tread index (heights of the other treads are relative to it)
    float ks = 0.0f;
    bool anchored = false; // the landing is the fit's tread count past a standing support foot (kept; else the clip may move it)
};

struct Config
{
    float toe = 0.07f;        // m, toe tip ahead of the ball along the foot
    float clearance = 0.03f;  // m, toe clearance above the noses at mid swing (0 at the lift-off and the landing)
    float rampIn = 0.2f;      // swing share over which the nose clearance comes in (the toe of a foot that leaves a tread)
    float downRamp = 0.1f;    // treads past the nose over which a drop is spread for the heel / ball going down
    float eps = 0.015f;       // m, smooth maximum width
    float minSwing = 0.15f;   // m, shortest planned swing along the flight
    float follow = 22.0f;     // rad/s, critically damped follow of a lower ground height (a higher one is taken at once)
    float sinkMin = 0.3f;     // m/s, ball sink rate (clip) above which its landing is predicted
    float predictMax = 0.25f; // s, longest predicted time to the landing
};

// Plan at the lift-off: aS0 = the swinging foot's mid-sole along the flight, aH0 its plane height; aSs / aHs = the support foot's
// mid-sole and plane height, aSupport = that foot stands now (a walk). With a standing support the landing is f.treads treads past it
// (the fit of the playing step), or as far ahead of it as the swinging foot is behind it (no fit); without one (the flight phase of a
// run) it is two steps (2 x f.treads treads) past the tread the swinging foot left. False = no plan (a flight without a tread, no fit
// in the flight phase or a swing too short).
inline bool PlanSwing(const Config& c, const Flight& f, float aS0, float aH0, float aSs, float aHs, bool aSupport, Plan& aOut)
{
    if (!f.valid || !(f.tread > 1.0e-3f) || !std::isfinite(f.rise) || !std::isfinite(aS0) || !std::isfinite(aH0) ||
        (aSupport && (!std::isfinite(aSs) || !std::isfinite(aHs))))
        return false;
    Plan p;
    p.h0 = aH0;
    p.s0 = aS0;
    float kLand = 0.0f;
    if (!aSupport)
    {
        if (f.treads <= 0)
            return false;
        p.hs = aH0;
        p.ks = TreadIndex(f, aS0);
        p.sLand = aS0 + 2.0f * float(f.treads) * f.tread;
        kLand = p.ks + 2.0f * float(f.treads);
    }
    else
    {
        p.hs = aHs;
        p.ks = TreadIndex(f, aSs);
        if (f.treads > 0)
        {
            p.sLand = aSs + float(f.treads) * f.tread;
            kLand = p.ks + float(f.treads);
            p.anchored = true;
        }
        else
        {
            p.sLand = aSs + std::fmax(aSs - aS0, 0.0f);
            kLand = TreadIndex(f, p.sLand);
        }
    }
    if (!(p.sLand - p.s0 >= c.minSwing))
        return false;
    p.h1 = p.hs + f.rise * (kLand - p.ks);
    aOut = p;
    return true;
}

// One step of the swing ground height aG (rate aGv) towards aTarget: a higher target is taken at once (the foot never goes into a
// tread), a lower one is followed critically damped at aOmega (exact step, stable for any aDt), so a released requirement does not
// drop the foot in one frame.
inline void Follow(float& aG, float& aGv, float aTarget, float aOmega, float aDt)
{
    if (!std::isfinite(aG) || !std::isfinite(aGv) || !(aDt > 0.0f) || !(aOmega > 0.0f) || aTarget >= aG)
    {
        aGv = (std::isfinite(aG) && aDt > 0.0f) ? std::fmax((aTarget - aG) / aDt, 0.0f) : 0.0f;
        aG = aTarget;
        return;
    }
    const float e0 = aG - aTarget, b = aGv + aOmega * e0, k = std::exp(-aOmega * aDt);
    aG = aTarget + (e0 + b * aDt) * k;
    aGv = (aGv - aOmega * b * aDt) * k;
    if (aG < aTarget)
    {
        aG = aTarget;
        aGv = 0.0f;
    }
}

// Landing seen from the clip: a foot whose ball is aAbove over the clip's floor and sinking at aVAbove (m/s) lands in about
// aAbove / -aVAbove; with its mid-sole at aS moving at aVs along the flight (m/s) it lands near aS + aVs x that time, on the tread
// under that point. The plan's landing then moves there (a step longer or shorter than the fit). False = no prediction (the foot is
// not sinking fast enough or not moving along the flight); the plan stays.
inline bool Retarget(const Config& c, const Flight& f, Plan& p, float aS, float aVs, float aAbove, float aVAbove)
{
    if (!(f.tread > 1.0e-3f) || !std::isfinite(aS) || !std::isfinite(aVs) || !std::isfinite(aAbove) || !std::isfinite(aVAbove) ||
        !(aVAbove < -c.sinkMin) || !(aVs > 0.0f) || p.anchored)
        return false;
    const float sPred = aS + aVs * std::clamp(std::fmax(aAbove, 0.0f) / -aVAbove, 0.0f, c.predictMax);
    p.sLand = std::fmax(sPred, aS + 0.01f);
    p.h1 = p.hs + f.rise * (TreadIndex(f, sPred) - p.ks);
    return true;
}

// Shape of the planned glide over the progress: linear, or front-loaded (1 - (1 - u)^2: the foot heads for the lower tread right
// after the lift-off and settles onto it).
inline float GlideShape(float aU, bool aFront)
{
    const float u = std::clamp(aU, 0.0f, 1.0f);
    return aFront ? 1.0f - (1.0f - u) * (1.0f - u) : u;
}

// One step of the planned glide: the height aPath moves towards the landing tread by the share of the glide left that the mid-sole
// covered since the last step (aSPrev -> aS; front-loaded with aFront); it reaches the landing tread at the landing whatever the
// plan's later changes, without a jump of its own.
inline float PathStep(const Plan& p, float aPath, float aSPrev, float aS, bool aFront)
{
    if (!std::isfinite(aPath))
        return p.h1;
    const float span = p.sLand - p.s0;
    if (!(span > 0.01f) || !(p.sLand - aSPrev > 0.01f))
        return p.h1;
    const float e0 = GlideShape((aSPrev - p.s0) / span, aFront), e1 = GlideShape((aS - p.s0) / span, aFront);
    const float rem = 1.0f - e0;
    if (!(rem > 1.0e-4f))
        return p.h1;
    return aPath + (p.h1 - aPath) * std::clamp((e1 - e0) / rem, 0.0f, 1.0f);
}

// Swing progress (0 at the lift-off, 1 at the landing) of the mid-sole at aS.
inline float Progress(const Plan& p, float aS) { return std::clamp((aS - p.s0) / (p.sLand - p.s0), 0.0f, 1.0f); }

// Ground height (world) under a swinging foot: the planned glide aPath from the tread it left to the tread it lands on (PathStep; NaN =
// linear in the progress aU), raised where the foot needs it. Going up the toe tip (aSToe along the flight, its own height aToeAbove above the clip's
// floor) stays aClearance above the nose line, capped at the landing tread; going down the heel and the ball (aSHeel / aSBall, their
// own heights aHeelAbove / aBallAbove) stay above the tread they leave (each drop spread over downRamp treads), at most the tread the
// foot left. Continuous in the positions and the progress.
inline float SwingGround(const Config& c, const Flight& f, const Plan& p, float aPath, float aU, float aSToe, float aToeAbove,
                         float aSHeel, float aHeelAbove, float aSBall, float aBallAbove)
{
    const float u = std::clamp(aU, 0.0f, 1.0f);
    const float path = std::isfinite(aPath) ? aPath : p.h0 + (p.h1 - p.h0) * u;
    float req = path;
    if (f.rise > 0.0f)
    {
        const float line = std::fmin(NoseLineUp(f, aSToe, p.hs, p.ks), p.h1) + c.clearance * 4.0f * u * (1.0f - u) - aToeAbove;
        const float in = c.rampIn > 1.0e-3f ? MinJerk(u / c.rampIn) : 1.0f;
        req = p.h0 + (line - p.h0) * in;
    }
    else if (f.rise < 0.0f)
    {
        const float heel = TreadDownRamped(f, aSHeel, p.hs, p.ks, c.downRamp) - aHeelAbove;
        const float ball = TreadDownRamped(f, aSBall, p.hs, p.ks, c.downRamp) - aBallAbove;
        req = std::fmin(std::fmax(heel, ball), p.h0);
    }
    if (!std::isfinite(req))
        return path;
    return SmoothMax(path, req, c.eps);
}
} // namespace swing

// Game-thread state: the detector fed by the ground traces and read by the feed.
struct FeedOut
{
    float treadM = 0.0f; // m, 0 = no stairs
    float weight = 0.0f; // [0, 1]
    float minScale = UTPP_STAIR_MIN_SCALE;
    float phaseErr[2] = {0.0f, 0.0f};  // m, TreadPhaseError of each foot this frame (0 = left, 1 = right)
    bool phaseOk[2] = {false, false}; // the foot has a trace this frame
};

class Tracker
{
public:
    explicit Tracker(const Config& aCfg = {}) : m_cfg(aCfg) {}
    const Config& Cfg() const { return m_cfg; }
    void SetMinScale(float aMinScale)
    {
        if (std::isfinite(aMinScale) && aMinScale > 0.2f && aMinScale <= 1.0f)
            m_cfg.minScale = aMinScale;
    }
    void Reset()
    {
        m_det.Reset();
        m_time = 0.0f;
        m_sinceFit = 1.0e9f;
        m_sinceValid = 1.0e9f;
        m_weight = 0.0f;
        m_est = Estimate{};
        m_footOk[0] = m_footOk[1] = false;
    }
    // The traces of one frame (world hits; hit false = none).
    void AddTraces(const bool (&aHit)[2], const float (&aPos)[2][3], const bool (&aHeelHit)[2], const float (&aHeelPos)[2][3])
    {
        for (int k = 0; k < 2; ++k)
        {
            if (aHit[k])
                m_det.Add(m_cfg, k, aPos[k][0], aPos[k][1], aPos[k][2], m_time);
            if (aHeelHit[k])
                m_det.Add(m_cfg, 2 + k, aHeelPos[k][0], aHeelPos[k][1], aHeelPos[k][2], m_time);
            // the foot's middle (between ball and heel; the ball alone without a heel hit)
            m_footOk[k] = aHit[k];
            if (aHit[k])
            {
                m_foot[k][0] = aHeelHit[k] ? 0.5f * (aPos[k][0] + aHeelPos[k][0]) : aPos[k][0];
                m_foot[k][1] = aHeelHit[k] ? 0.5f * (aPos[k][1] + aHeelPos[k][1]) : aPos[k][1];
            }
        }
    }
    // One feed: capsule position and velocity (m, m/s, world); aGround = ground locomotion (else the stairs fade out).
    FeedOut Update(float aDt, bool aHaveCapsule, float aCapX, float aCapY, float aVelX, float aVelY, bool aGround)
    {
        FeedOut out;
        out.minScale = m_cfg.minScale;
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.25f) : 0.0f;
        m_time += dt;
        m_sinceFit += dt;
        m_sinceValid += dt;
        const float speed = std::hypot(aVelX, aVelY);
        const bool moving = aHaveCapsule && aGround && std::isfinite(aCapX) && std::isfinite(aCapY) && std::isfinite(speed) &&
                            speed >= m_cfg.minSpeed;
        if (moving && m_sinceFit >= m_cfg.fitPeriodS && m_det.Count() > 0)
        {
            m_sinceFit = 0.0f;
            const Estimate e = m_det.Fit(m_cfg, aCapX, aCapY, aVelX / speed, aVelY / speed, m_time);
            if (e.valid)
            {
                m_est = e;
                m_sinceValid = 0.0f;
                ++m_valid;
            }
            ++m_fits;
        }
        const bool on = moving && m_est.valid && m_sinceValid <= m_cfg.lostS;
        const float step = m_cfg.weightTime > 0.0f ? dt / m_cfg.weightTime : 1.0f;
        m_weight = std::clamp(on ? m_weight + step : m_weight - step, 0.0f, 1.0f);
        if (m_weight <= 0.0f)
        {
            m_weight = 0.0f;
            if (!on)
                m_est = Estimate{};
        }
        if (m_weight > 0.0f && m_est.valid)
        {
            out.treadM = m_est.tread;
            out.weight = m_weight;
            for (int k = 0; k < 2; ++k)
            {
                out.phaseOk[k] = m_footOk[k];
                out.phaseErr[k] = m_footOk[k] ? TreadPhaseError(m_est, m_foot[k][0], m_foot[k][1]) : 0.0f;
            }
        }
        return out;
    }
    const Estimate& Current() const { return m_est; }
    float Weight() const { return m_weight; }
    uint64_t Fits() const { return m_fits; }
    uint64_t ValidFits() const { return m_valid; }

private:
    Config m_cfg;
    Detector m_det;
    float m_time = 0.0f, m_sinceFit = 1.0e9f, m_sinceValid = 1.0e9f, m_weight = 0.0f;
    Estimate m_est;
    uint64_t m_fits = 0, m_valid = 0;
    float m_foot[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // last traced foot middle (x, y) per side
    bool m_footOk[2] = {false, false};
};

namespace rt
{
inline std::atomic<bool> g_on{false}; // the menu default (mm.stairSteps)
inline Tracker g_tracker;             // game thread only (ground traces and the feed)
inline std::atomic<float> g_cadence{1.0f}, g_tread{0.0f}, g_weight{0.0f}; // diagnostics of the last feed
inline std::atomic<int32_t> g_treads{0};
inline std::atomic<float> g_landErr{0.0f}; // m, phase error of the foot that landed in the last feed (NaN = no landing)

inline bool Enabled() { return g_on.load(std::memory_order_relaxed); }
inline void SetEnabled(bool aOn)
{
    if (g_on.exchange(aOn, std::memory_order_acq_rel) != aOn)
        g_tracker.Reset();
}
// Game thread, with the traces of a frame (GroundFeetRuntime PublishSamples).
inline void AddTraces(const bool (&aHit)[2], const float (&aPos)[2][3], const bool (&aHeelHit)[2], const float (&aHeelPos)[2][3])
{
    if (Enabled())
        g_tracker.AddTraces(aHit, aPos, aHeelHit, aHeelPos);
}
// Game thread, once per feed. Off: no stairs.
inline FeedOut Update(float aDt, bool aHaveCapsule, float aCapX, float aCapY, float aVelX, float aVelY, bool aGround)
{
    if (!Enabled())
        return FeedOut{};
    const FeedOut o = g_tracker.Update(aDt, aHaveCapsule, aCapX, aCapY, aVelX, aVelY, aGround);
    g_tread.store(o.treadM, std::memory_order_relaxed);
    g_weight.store(o.weight, std::memory_order_relaxed);
    return o;
}
// Game thread, with the traces of a frame: the flight of the last feed for the ground feet stage (valid 0 = none; off: always none).
inline Flight CurrentFlight()
{
    Flight f;
    if (!Enabled())
        return f;
    const Estimate& e = g_tracker.Current();
    const float w = g_tracker.Weight();
    if (!e.valid || !(w > 0.0f) || !(e.tread > 1.0e-3f))
        return f;
    f.valid = 1;
    f.weight = w;
    f.tread = e.tread;
    f.rise = e.rise;
    f.dirX = e.dirX;
    f.dirY = e.dirY;
    f.noseX = e.noseX;
    f.noseY = e.noseY;
    f.treads = g_treads.load(std::memory_order_relaxed);
    return f;
}
} // namespace rt

} // namespace utpp::mm::stairsteps

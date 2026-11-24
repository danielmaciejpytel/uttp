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
        for (int dp = -4; dp <= 4; ++dp)
        {
            const float p = bestP + 0.005f * float(dp);
            const float k = cost(bestT, p);
            if (k < bestCost)
                bestCost = k, bestP = p;
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

// Game-thread state: the detector fed by the ground traces and read by the feed.
struct FeedOut
{
    float treadM = 0.0f; // m, 0 = no stairs
    float weight = 0.0f; // [0, 1]
    float minScale = UTPP_STAIR_MIN_SCALE;
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
};

namespace rt
{
inline std::atomic<bool> g_on{false}; // the menu default (mm.stairSteps)
inline Tracker g_tracker;             // game thread only (ground traces and the feed)
inline std::atomic<float> g_cadence{1.0f}, g_tread{0.0f}, g_weight{0.0f}; // diagnostics of the last feed
inline std::atomic<int32_t> g_treads{0};

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
} // namespace rt

} // namespace utpp::mm::stairsteps

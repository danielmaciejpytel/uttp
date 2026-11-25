#pragma once
// Quick straightening after a turn (setting mm.fastTurnExit, default off). Header-only, allocation-free, deterministic (feed dt only).
//
// While V runs, the input heading is the world heading the stick asks for (camera yaw plus stick angle), so a mouse turn and a stick
// turn look the same. A sustained turn of that heading that comes to an end opens a short exit window; entering a turn and holding it
// change nothing. During the window the host turns the body the rest of the way at the turn's own rate or faster (BodyTurn follow law),
// and the motion driver stops curving the query velocity (speed converter), lets the lean settle quickly and lets a straight loop
// replace an arc loop without waiting out the loop dwell.
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace utpp::mm::turnexit
{
struct Params
{
    float armRateDegS = 45.0f;   // input heading rate that counts as turning
    float armTimeS = 0.12f;      // turning this long arms the exit (a key press that flips the heading at once stays below it)
    float armGapS = 0.10f;       // a pause longer than this (without an exit) forgets the turn
    float stopRateDegS = 20.0f;  // the turn has ended below this rate ...
    float stopFraction = 0.25f;  // ... or below this fraction of the turn's own rate
    float stopWindowS = 0.03f;   // window of the end-of-turn rate (mouse input arrives in uneven steps)
    float rateWindowS = 0.15f;   // window of the turn's own rate
    float rateMemoryS = 1.0f;    // time constant of the decay of the turn's own rate
    float holdS = 0.5f;          // length of the exit window
    float maxDtS = 0.1f;         // a longer feed gap forgets everything
};

class Detector
{
public:
    explicit Detector(const Params& aParams = {}) : m_p(aParams) {}

    void Reset()
    {
        m_n = 0;
        m_head = 0;
        m_time = 0.0;
        m_yaw = 0.0f;
        m_turnS = m_gapS = 0.0f;
        m_turnRate = 0.0f;
        m_turnSign = 0.0f;
        m_left = 0.0f;
        m_exitRate = 0.0f;
    }

    // One feed. aEligible: running with the stick held in the ordinary locomotion (anything else forgets the turn); aHeadingDeg: the
    // input heading in the world (deg, any wrap).
    void Step(float aDt, bool aEligible, float aHeadingDeg)
    {
        if (!aEligible || !std::isfinite(aHeadingDeg) || !std::isfinite(aDt) || !(aDt > 0.0f) || aDt > m_p.maxDtS)
        {
            Reset();
            return;
        }
        if (m_n == 0)
            m_yaw = aHeadingDeg;
        else
            m_yaw += std::remainder(aHeadingDeg - m_yaw, 360.0f); // unwrapped
        m_time += double(aDt);
        m_ring[m_head] = Sample{m_time, m_yaw};
        m_head = (m_head + 1) % kRing;
        m_n = std::min(m_n + 1, kRing);
        if (m_left > 0.0f)
            m_left = std::max(0.0f, m_left - aDt);

        float shortRate = 0.0f, longRate = 0.0f;
        if (!Rate(m_p.stopWindowS, shortRate))
            return;
        Rate(m_p.rateWindowS, longRate);
        const float mag = std::fabs(shortRate);
        if (mag >= m_p.armRateDegS)
        {
            const float sign = shortRate > 0.0f ? 1.0f : -1.0f;
            if (m_turnSign != sign)
            {
                m_turnSign = sign;
                m_turnS = 0.0f;
                m_turnRate = 0.0f;
            }
            m_turnS += aDt;
            m_gapS = 0.0f;
            // the turn's own rate: the long window, but not read down by the part of the window before the turn or after its end
            // (a slow decay in time, the same at every frame rate)
            m_turnRate = std::max(std::fabs(longRate), m_turnRate * std::max(0.0f, 1.0f - aDt / m_p.rateMemoryS));
            m_left = 0.0f; // a new turn closes the window at once
        }
        const bool ended = mag < std::max(m_p.stopRateDegS, m_p.stopFraction * m_turnRate);
        if (m_turnS >= m_p.armTimeS && ended)
        {
            m_left = m_p.holdS;
            m_exitRate = m_turnRate;
            ++m_exits;
            m_turnS = 0.0f;
            m_turnSign = 0.0f;
            m_turnRate = 0.0f;
        }
        else if (mag < m_p.armRateDegS)
        {
            m_gapS += aDt;
            if (m_gapS > m_p.armGapS)
            {
                m_turnS = 0.0f;
                m_turnSign = 0.0f;
                m_turnRate = 0.0f;
            }
        }
    }

    bool Active() const { return m_left > 0.0f; }
    // rate (deg/s) of the turn whose end opened the window
    float TurnRateDegS() const { return Active() ? m_exitRate : 0.0f; }
    uint64_t Exits() const { return m_exits; }

private:
    struct Sample
    {
        double t = 0.0;
        float yaw = 0.0f;
    };
    static constexpr int kRing = 64; // 0.15 s up to ~420 feeds per second; above that the windows are shorter

    // heading rate over the newest window of at least aWindow seconds (or the whole ring when it is shorter); false = under 2 samples
    bool Rate(float aWindow, float& aOut) const
    {
        if (m_n < 2)
            return false;
        const Sample& now = m_ring[(m_head + kRing - 1) % kRing];
        const Sample* ref = nullptr;
        for (int k = 1; k < m_n; ++k)
        {
            ref = &m_ring[(m_head + kRing - 1 - k) % kRing];
            if (now.t - ref->t >= double(aWindow) - 1.0e-6)
                break;
        }
        const double span = now.t - ref->t;
        if (!(span > 0.0))
            return false;
        aOut = static_cast<float>(double(now.yaw - ref->yaw) / span);
        return true;
    }

    Params m_p;
    Sample m_ring[kRing]{};
    int m_n = 0, m_head = 0;
    double m_time = 0.0;
    float m_yaw = 0.0f;
    float m_turnS = 0.0f, m_gapS = 0.0f, m_turnRate = 0.0f, m_turnSign = 0.0f;
    float m_left = 0.0f, m_exitRate = 0.0f;
    uint64_t m_exits = 0;
};

// Body follow law inside the window: at least kBodyRateGain times the turn's rate, or the remaining error over kBodyFollowS.
inline constexpr float kBodyRateGain = 1.5f;
inline constexpr float kBodyFollowS = 0.05f;
// Drawn root inside the window: it rides the capsule and takes back the offset it kept against it at this half-life (s)
// (MotionOffsetRoot Step::followCapsule).
inline constexpr float kRootHalfLifeS = 0.04f;
// Orientation warp inside the window (owarp::Params): no new pelvis or torso correction, the applied one returns at this smoothing
// (s) and rate bound (deg/s).
inline constexpr float kWarpReleaseTcS = 0.05f;
inline constexpr float kWarpReleaseRateDegS = 240.0f;
template <class WarpParams>
inline void ReleaseWarp(WarpParams& aParams)
{
    aParams.maxPelvisDeg = aParams.maxTorsoDeg = 0.0f;
    aParams.pelvisTc = aParams.torsoTc = kWarpReleaseTcS;
    aParams.maxPelvisRate = aParams.maxTorsoRate = kWarpReleaseRateDegS;
}
// Lean inside the window: low pass of the lateral acceleration (s) and time scale of the blend space weight smoothing.
inline constexpr float kLeanAccelTcS = 0.03f;
inline constexpr float kLeanBlendTimeScale = 6.0f;

// "..._Loop_F": a straight forward loop. Inside the window a playing one is kept while the stick state does not change.
template <class Str>
inline bool StraightForwardLoop(const Str& aPlaying)
{
    const auto at = aPlaying.rfind("_Loop_");
    return at != Str::npos && aPlaying.size() == at + 7 && aPlaying[at + 6] == 'F';
}

// An arc loop (the running arcs: 20 degree leans and the forward diagonals) and the straight forward loop of the same gait:
// "..._Run_Loop_F_L_20" / "..._Run_Loop_FR" -> "..._Run_Loop_F". Only this pair skips the loop dwell inside the window.
template <class Str>
inline bool ArcToStraightLoop(const Str& aPlaying, const Str& aCandidate)
{
    const auto at = aPlaying.rfind("_Loop_");
    if (at == Str::npos)
        return false;
    const auto side = aPlaying.substr(at + 6);
    if (!(side == "F_L_20" || side == "F_R_20" || side == "FL" || side == "FR"))
        return false;
    return aCandidate.size() == at + 7 && aCandidate.substr(0, at + 6) == aPlaying.substr(0, at + 6) && aCandidate[at + 6] == 'F';
}
} // namespace utpp::mm::turnexit

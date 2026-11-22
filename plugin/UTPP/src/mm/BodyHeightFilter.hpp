// UTPP motion matching runtime - body height smoothing on uneven ground. Header-only, C++20, deterministic, allocation-free.
//
// On rough ground the game snaps the capsule to the floor every frame; the body is drawn at the capsule height, so it hops by a few
// centimetres while the camera pivot smooths the same height. This stage smooths the drawn height of the pelvis only:
//   - BodyHeightFilter: a second-order tracker on the capsule height the body is drawn with. The velocity estimate is low-passed
//     (velTau), the position follows the measured height around the velocity prediction (posTau), so a constant slope passes with
//     zero lag; both in closed form of dt. A frame whose dt is above hitchFactor x the running median dt does not update the
//     velocity (its catch-up is handled as position). The offset (filtered - measured) is clamped to [minOffset, maxOffset] and the
//     tracker state with it (no wind-up). While the stage is not allowed (air, vehicle, scenes, hold, traversal, stance outside
//     stand .. sprint) the offset fades to zero with fadeTau; a height step above resetStep (teleport, stairs warp) or a long gap
//     restarts the tracker at the measured height (offset 0 at once). On a moving platform (elevator; the flag held platformHold s
//     after it drops) the stage passes the height through: the tracker holds no velocity, the offset of the entry is carried and fades
//     linearly to 0 (also when the flag ends before it reached 0), and the tracker fades in from 0 after the ride on top of it - the
//     output stays continuous through a flag flicker.
//   - BodyHeightLegs: applies an offset to a local pose (7 floats per bone: p.xyz, q.xyzw, database order). The pelvis (common
//     parent of both UpLegs) moves by the offset along model z; both legs are solved again with the shared two-bone solver
//     (mm/LegIk.hpp) to the ankle positions the pose had before, so the feet stay where they were (model space) and only the knee
//     bend changes. The foot keeps its model-space rotation.
// Model space: character frame, z up (the frame is turned about z only, so model z is world z).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

#include "Inertializer.hpp"
#include "LegIk.hpp"
#include "MMMath.hpp"

namespace utpp::mm::bodyheight
{

struct Config
{
    float posTau = 0.05f;      // s, position time constant around the velocity prediction
    float velTau = 0.08f;      // s, velocity low-pass time constant
    float minOffset = -0.04f;  // m, lowest offset (body below the capsule height)
    float maxOffset = 0.005f;  // m, highest offset (legs are nearly straight at run contact)
    float fadeTau = 0.08f;     // s, fade of the offset when the stage opens / closes
    float resetStep = 0.5f;    // m, a height step above this restarts the tracker
    float maxDt = 0.25f;       // s, a longer gap restarts the tracker
    float hitchFactor = 2.0f;  // dt above this x the running median dt does not update the velocity
    float platformRate = 0.12f; // m/s, linear fade of the offset to 0 when a moving platform starts carrying V (0.4 cm per frame at 30 Hz)
    float platformHold = 0.3f;  // s, the platform flag is held this long after it drops (a flicker at the cabin's start / stop)
};

inline bool ValidConfig(const Config& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    return pos(c.posTau) && pos(c.velTau) && pos(c.fadeTau) && pos(c.resetStep) && pos(c.maxDt) && pos(c.hitchFactor) && pos(c.platformRate) &&
           pos(c.platformHold) &&
           std::isfinite(c.minOffset) && std::isfinite(c.maxOffset) && c.minOffset <= 0.0f && c.maxOffset >= 0.0f;
}

struct FilterDiag
{
    uint64_t steps = 0;    // tracker updates
    uint64_t resets = 0;   // restarts (step / gap / non-finite input)
    uint64_t hitches = 0;  // frames whose velocity update was skipped
    uint64_t clamps = 0;   // frames whose offset hit a bound
    uint64_t platform = 0; // frames on a moving platform (no smoothing)
};

class Filter
{
public:
    explicit Filter(const Config& aCfg = {}) : m_cfg(ValidConfig(aCfg) ? aCfg : Config{}) {}

    void Reset()
    {
        m_init = false;
        m_p = m_v = m_z = 0.0f;
        m_fade = 0.0f;
        m_offset = 0.0f;
        m_carry = 0.0f;
        m_platT = 0.0f;
        m_onPlat = false;
        m_dtCount = 0;
        m_dtNext = 0;
    }

    // One update with the time since the previous update (s), the measured drawn height (m) and whether the stage may act now.
    // Returns the offset to add to the pelvis height (m). A non-positive / non-finite dt leaves the state and returns the last offset.
    // aPlatform: a moving platform (elevator) carries V - no smoothing; held platformHold s after it drops. The tracker follows the
    // measured height with zero velocity (the cabin's motion never enters the velocity estimate), the offset of the entry is carried and
    // fades linearly to 0 (platformRate, also after the ride until it is 0) and the tracker fades in again from 0 after the ride on top of
    // it: no step in the output when the flag starts or ends.
    float Step(float aDt, float aZ, bool aAllowed, bool aPlatform = false)
    {
        if (!std::isfinite(aZ))
        {
            if (m_init)
                ++m_diag.resets;
            Reset();
            return 0.0f;
        }
        if (!m_init)
        {
            Start(aZ);
            return 0.0f;
        }
        if (!(aDt > 0.0f) || !std::isfinite(aDt))
            return m_offset;
        if (aPlatform)
            m_platT = m_cfg.platformHold;
        else
            m_platT = std::fmax(m_platT - aDt, 0.0f);
        const bool platform = aPlatform || m_platT > 0.0f;
        if (platform && !m_onPlat)
            m_carry = m_offset; // the output of the last frame goes on as the carried offset
        m_onPlat = platform;
        if (m_carry != 0.0f)
        {
            // linear fade to exactly 0 (0.33 s from the -4 cm bound)
            const float mag = std::fmax(std::fabs(m_carry) - m_cfg.platformRate * aDt, 0.0f);
            m_carry = m_carry < 0.0f ? -mag : mag;
        }
        if (platform)
        {
            ++m_diag.platform;
            PushDt(aDt);
            m_p = m_z = aZ;
            m_v = 0.0f;
            m_fade = 0.0f;
            m_offset = m_carry;
            return m_offset;
        }
        if (std::fabs(aZ - m_z) > m_cfg.resetStep || aDt > m_cfg.maxDt)
        {
            ++m_diag.resets;
            const float fade = m_fade;
            Start(aZ);
            m_fade = fade; // the gate fade is independent of the tracker restart
            return 0.0f;
        }
        ++m_diag.steps;
        const float median = MedianDt();
        const bool hitch = median > 0.0f && aDt > m_cfg.hitchFactor * median;
        PushDt(aDt);
        if (!hitch)
        {
            const float vraw = (aZ - m_z) / aDt;
            const float kv = std::exp(-aDt / m_cfg.velTau);
            m_v = vraw + (m_v - vraw) * kv;
        }
        else
            ++m_diag.hitches;
        const float pred = m_p + m_v * aDt;
        const float kp = std::exp(-aDt / m_cfg.posTau);
        float o = (pred - aZ) * kp;
        if (o < m_cfg.minOffset || o > m_cfg.maxOffset)
        {
            o = std::clamp(o, m_cfg.minOffset, m_cfg.maxOffset);
            ++m_diag.clamps;
        }
        m_p = aZ + o;
        m_z = aZ;
        const float target = aAllowed ? 1.0f : 0.0f;
        m_fade = target + (m_fade - target) * std::exp(-aDt / m_cfg.fadeTau);
        if (!aAllowed && m_fade < 1e-3f)
            m_fade = 0.0f;
        else if (aAllowed && m_fade > 0.999f)
            m_fade = 1.0f;
        m_offset = o * m_fade;
        if (m_carry != 0.0f) // after a ride: the carried offset fades out under the tracker fading in
            m_offset = std::clamp(m_offset + m_carry, m_cfg.minOffset, m_cfg.maxOffset);
        return m_offset;
    }

    float Offset() const { return m_offset; }
    float Fade() const { return m_fade; }
    bool Started() const { return m_init; }
    const FilterDiag& Diagnostics() const { return m_diag; }
    const Config& Cfg() const { return m_cfg; }

private:
    static constexpr int kDtRing = 15;

    void Start(float aZ)
    {
        m_init = true;
        m_p = m_z = aZ;
        m_v = 0.0f;
        m_offset = 0.0f;
        m_fade = 0.0f;
        m_carry = 0.0f;
    }
    void PushDt(float aDt)
    {
        m_dt[size_t(m_dtNext)] = aDt;
        m_dtNext = (m_dtNext + 1) % kDtRing;
        if (m_dtCount < kDtRing)
            ++m_dtCount;
    }
    float MedianDt() const
    {
        if (m_dtCount < 3)
            return 0.0f;
        std::array<float, kDtRing> s{};
        std::copy(m_dt.begin(), m_dt.begin() + m_dtCount, s.begin());
        std::nth_element(s.begin(), s.begin() + m_dtCount / 2, s.begin() + m_dtCount);
        return s[size_t(m_dtCount / 2)];
    }

    Config m_cfg;
    bool m_init = false;
    float m_p = 0.0f, m_v = 0.0f, m_z = 0.0f;
    float m_fade = 0.0f;
    float m_offset = 0.0f;
    float m_carry = 0.0f;  // offset carried through a moving platform ride (fades linearly to 0)
    float m_platT = 0.0f;  // s, platform flag hold left
    bool m_onPlat = false; // the last step ran the platform branch
    std::array<float, kDtRing> m_dt{};
    int m_dtCount = 0, m_dtNext = 0;
    FilterDiag m_diag;
};

// Leg chains of a database topology (indices into the local pose); trivially copyable so it can be published to the animation threads.
struct LegRig
{
    static constexpr int kMaxDepth = 16;
    uint32_t numBones = 0;      // 0 = not valid
    int32_t pelvis = -1;        // common parent of both UpLegs
    int32_t pathLen = 0;        // ancestors of the pelvis' children down to the pelvis, root first (path[pathLen - 1] = pelvis)
    int32_t path[kMaxDepth] = {};
    int32_t chain[2][3] = {{-1, -1, -1}, {-1, -1, -1}}; // UpLeg, Leg, Foot (left, right)
    int32_t toe[2] = {-1, -1};  // toe below the foot (-1: the ankle stands in)
    int32_t toePathLen[2] = {0, 0};
    int32_t toePath[2][4] = {};

    bool Valid() const { return numBones != 0; }

    // aDb: any database type with Loaded / NumBones / BoneParent / BoneName. False (rig invalid) if a bone or the chain is missing.
    template <class Db>
    bool Init(const Db& aDb)
    {
        *this = LegRig{};
        if (!aDb.Loaded())
            return false;
        static constexpr std::string_view kNames[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"}, {"RightUpLeg", "RightLeg", "RightFoot"}};
        static constexpr std::string_view kToes[2] = {"LeftToeBase", "RightToeBase"};
        const uint32_t nb = aDb.NumBones();
        auto find = [&](std::string_view aName) {
            for (uint32_t b = 0; b < nb; ++b)
                if (aDb.BoneName(b) == aName)
                    return static_cast<int32_t>(b);
            return int32_t(-1);
        };
        for (uint32_t b = 0; b < nb; ++b)
            if (aDb.BoneParent(b) >= static_cast<int32_t>(b))
                return false; // parents precede children
        int32_t pel[2] = {-1, -1};
        for (int s = 0; s < 2; ++s)
        {
            for (int k = 0; k < 3; ++k)
                if ((chain[s][k] = find(kNames[s][k])) < 0)
                    return false;
            if (aDb.BoneParent(uint32_t(chain[s][1])) != chain[s][0] || aDb.BoneParent(uint32_t(chain[s][2])) != chain[s][1])
                return false;
            pel[s] = aDb.BoneParent(uint32_t(chain[s][0]));
            const int32_t t = find(kToes[s]);
            int32_t tmp[4];
            int32_t n = 0;
            bool below = false;
            for (int32_t b = t; b >= 0 && n < 4; b = aDb.BoneParent(uint32_t(b)))
            {
                if (b == chain[s][2])
                {
                    below = true;
                    break;
                }
                tmp[n++] = b;
            }
            if (below)
            {
                toe[s] = t;
                toePathLen[s] = n;
                for (int32_t i = 0; i < n; ++i)
                    toePath[s][i] = tmp[n - 1 - i];
            }
        }
        if (pel[0] < 0 || pel[0] != pel[1])
            return false;
        int32_t tmp[kMaxDepth];
        int32_t n = 0;
        for (int32_t b = pel[0]; b >= 0; b = aDb.BoneParent(uint32_t(b)))
        {
            if (n >= kMaxDepth)
                return false;
            tmp[n++] = b;
        }
        pathLen = n;
        for (int32_t i = 0; i < n; ++i)
            path[i] = tmp[n - 1 - i];
        pelvis = pel[0];
        numBones = nb;
        return true;
    }
};

struct ApplyResult
{
    bool applied = false; // the pelvis moved
    int solved = 0;       // legs solved again (0..2)
    int clamped = 0;      // legs whose ankle target was beyond the reach
    bool raiseLimited = false; // a raise was cut to the reach slack of the straighter leg
    float offset = 0.0f;  // m, offset applied (a raise at most the reach slack)
    float ankleError = 0.0f; // m, largest ankle distance from where it was (after the solve)
};

namespace detail
{
inline BoneTransform Load(const float* aPose, int32_t b)
{
    const float* v = aPose + size_t(b) * 7;
    return BoneTransform{math::Vec3{v[0], v[1], v[2]}, math::Quat{v[3], v[4], v[5], v[6]}};
}
inline BoneTransform Mul(const BoneTransform& aParent, const BoneTransform& aLocal)
{
    return BoneTransform{aParent.p + math::QuatRotate(aParent.q, aLocal.p), math::QuatMul(aParent.q, aLocal.q)};
}
inline float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
} // namespace detail

// Moves the pelvis of aPose (numBones x 7 floats) by aOffset along model z and solves both legs back to their ankles. An offset of
// zero, an invalid rig or a pose of another bone count leaves aPose untouched. aMaxExtension / aSoftExtension: reach fractions of the
// leg length (FootLocker defaults). A raise is limited to the height at which the straighter leg reaches aSoftExtension of its length
// (the solver is exact below it), so the feet never lift with the pelvis; a lowering needs no bound (the knees bend more).
inline ApplyResult ApplyOffset(const LegRig& aRig, float* aPose, uint32_t aNumBones, float aOffset, float aMaxExtension = 0.995f,
                               float aSoftExtension = 0.97f)
{
    using detail::Load;
    using detail::Mul;
    ApplyResult r;
    if (!aRig.Valid() || !aPose || aNumBones != aRig.numBones || !std::isfinite(aOffset) || aOffset == 0.0f)
        return r;
    for (int32_t i = 0; i < aRig.pathLen; ++i)
        for (int k = 0; k < 7; ++k)
            if (!std::isfinite(aPose[size_t(aRig.path[i]) * 7 + size_t(k)]))
                return r;
    // pelvis parent and pelvis (model space), before the move
    BoneTransform par{};
    for (int32_t i = 0; i + 1 < aRig.pathLen; ++i)
        par = Mul(par, Load(aPose, aRig.path[i]));
    const BoneTransform pel0 = Mul(par, Load(aPose, aRig.pelvis));
    math::Vec3 ankle[2], toe[2];
    for (int s = 0; s < 2; ++s)
    {
        const BoneTransform ga = Mul(pel0, Load(aPose, aRig.chain[s][0]));
        const BoneTransform gb = Mul(ga, Load(aPose, aRig.chain[s][1]));
        const BoneTransform gc = Mul(gb, Load(aPose, aRig.chain[s][2]));
        BoneTransform gl = gc;
        for (int32_t i = 0; i < aRig.toePathLen[s]; ++i)
            gl = Mul(gl, Load(aPose, aRig.toePath[s][i]));
        ankle[s] = gc.p;
        toe[s] = gl.p;
        if (!math::IsFinite(ankle[s]) || !math::IsFinite(toe[s]))
            return r;
        if (aOffset > 0.0f)
        {
            // largest raise o with |hip + (0, 0, o) - ankle| <= soft x leg length: o^2 + 2 dz o + lac^2 - S^2 = 0
            const math::Vec3 d = ga.p - gc.p;
            const float lac2 = math::Dot(d, d);
            const float S = aSoftExtension * (detail::Len(gb.p - ga.p) + detail::Len(gc.p - gb.p));
            const float disc = d.z * d.z - lac2 + S * S;
            const float slack = S * S > lac2 && disc > 0.0f ? std::fmax(0.0f, -d.z + std::sqrt(disc)) : 0.0f;
            if (slack < aOffset)
            {
                aOffset = slack;
                r.raiseLimited = true;
            }
        }
    }
    if (!(aOffset != 0.0f))
        return r;
    r.offset = aOffset;
    float* pp = aPose + size_t(aRig.pelvis) * 7;
    const math::Vec3 d = math::QuatRotate(math::QuatInv(par.q), math::Vec3{0.0f, 0.0f, aOffset});
    pp[0] += d.x;
    pp[1] += d.y;
    pp[2] += d.z;
    r.applied = true;
    const BoneTransform pel1 = Mul(par, Load(aPose, aRig.pelvis));
    for (int s = 0; s < 2; ++s)
    {
        const BoneTransform ga = Mul(pel1, Load(aPose, aRig.chain[s][0]));
        const BoneTransform gb = Mul(ga, Load(aPose, aRig.chain[s][1]));
        const BoneTransform gc = Mul(gb, Load(aPose, aRig.chain[s][2]));
        const legik::TwoBoneSolution sol =
            legik::SolveTwoBone(pel1, ga, gb, gc, ankle[s], toe[s], aMaxExtension, aSoftExtension, true, true);
        if (sol.clamped)
            ++r.clamped;
        if (sol.status != legik::TwoBoneStatus::kOk || !math::IsFinite(sol.ankle))
        {
            r.ankleError = std::max(r.ankleError, detail::Len(gc.p - ankle[s])); // the leg follows the pelvis (fail safe)
            continue;
        }
        for (int k = 0; k < 3; ++k)
        {
            float* q = aPose + size_t(aRig.chain[s][k]) * 7 + 3;
            q[0] = sol.local[k].x;
            q[1] = sol.local[k].y;
            q[2] = sol.local[k].z;
            q[3] = sol.local[k].w;
        }
        ++r.solved;
        r.ankleError = std::max(r.ankleError, detail::Len(sol.ankle - ankle[s]));
    }
    return r;
}

} // namespace utpp::mm::bodyheight

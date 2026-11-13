// UTPP head lead (opt-in, CET mm.headLead): a runtime correction of the Neck / Neck1 / Head yaw while V turns at run speed.
// Header-only, C++20, no game types, no allocation. Unit test: tests/HeadLeadTest.cpp; offline replay of the film
// 12-43-58: tests/HeadLeadReplay.cpp.
//
// Problem (film 12-43-58, 3-stage pose capture): the motion run clips point head and chest "forward" while the body follows the
// velocity and lags it by 30-35 deg, so the clip picked for a left circle (M_Neutral_Run_Loop_FL) leaves the head turned about
// 32 deg to the RIGHT of Spine3 (against the turn); the right circle clips (FR / F_R_20) leave it ~16 deg to the right (into
// the turn). motion turns the head INTO the turn by 15-25 deg relative to the shoulders (additive BS_Relaxed_Lean_Head,
// disabled while the aim offset is active).
//
// Algorithm (all angles degrees, yaw about the model vertical axis, model space Z up, + = counter-clockwise from above =
// V turning LEFT, same as atan2(-x, y) of GameIntegration.hpp):
//   Step (feed rate): heading = atan2(vy, vx) of the world velocity; rate = smoothed d(heading)/dt (+ left, - right);
//                     lead = clamp(gain * rate, +-maxLead);  weight w -> turnWeight(|rate|) * speedWeight(speed) while the gate is open
//                     (no aiming / weapon action / air / crouch / blocked), linear fade in/out >= 0.15 s.
//   Apply (publish rate, on the local pose BEFORE the aim offset / finish stages): measures
//                     m = yaw(Head local Z) - yaw(Spine3 local Z) - restBias   (0 = the idle pose; <0 = head turned right)
//                     mf = low-passed m (keeps the natural per-step sway of the clip, removes only its bias),
//                     corr = clamp(deadband(lead - mf), +-maxCorr) * w, rate limited, applied as a model-space rotation about Z,
//                     distributed Neck 0.25 / Neck1 0.25 / Head 0.5 (positions, scales and the eye / face children untouched).
// Straight running (|rate| below the ramp) has weight 0: bit-identical pose.
//
// Reference of the lead (2026-10-19, in-game report "circling left -> the head goes slightly to the right"): the first version
// measured the head against Spine3. That is not the direction of the run: the actor yaw lags the velocity by 30-40 deg (strafe
// style run clips FL / FR), so Spine3 itself points 35-45 deg off the path - to the RIGHT of it in a left circle, to the LEFT of
// it in a right circle. A head that is "+lead" from Spine3 still ended -4..-7 deg (left circle) / +48 deg (right circle: no
// correction at all, FR already has the head right of Spine3) off the path (captures 22-47 / 22-48, stage 1; the frames of the
// database pose and of the game records are identical, only the reference was wrong). With Signals.yawValid the lead is
// therefore measured against the travel direction in the actor frame: pathRel = atan2(vy, vx) - 90 - actorYaw (deg, + left; the
// pose is in the actor model space, +Y forward, so AxisYaw(Head) is the direction the face looks, 0 = forward), headVsPath =
// AxisYaw(Head) - pathRel (0 = the face looks along the path), target headVsPath = lead. pathBlend 0 gives the Spine3 law again
// (bit-identical to the first version); maxTwistDeg bounds how far the path law may turn the head from Spine3 (twistBias
// removed), so a clip with a strongly twisted torso cannot ask for an anatomically absurd neck. The window is centred on
// head-vs-Spine3 == twistBiasDeg (1 deg, what the run clips hold), NOT on the idle pose (restBiasDeg 23.39, measurement only):
// the idle centre made the window [-26.6, +73.4] vs Spine3 and clipped right circles in 37% of the frames, left circles in 0%.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace utpp::mm::headlead
{
constexpr float kRad = 3.14159265358979323846f / 180.0f;
constexpr float kDeg = 180.0f / 3.14159265358979323846f;
constexpr int32_t kMaxChain = 32;

inline float Wrap(float a)
{
    a = std::fmod(a + 180.0f, 360.0f);
    if (a < 0.0f)
        a += 360.0f;
    return a - 180.0f;
}
inline float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float Smooth01(float x, float a, float b)
{
    if (!(b > a))
        return x >= b ? 1.0f : 0.0f;
    const float t = Clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
inline float Alpha(float dt, float tc) { return tc > 1.0e-4f ? 1.0f - std::exp(-dt / tc) : 1.0f; }
inline float Approach(float v, float target, float maxStep)
{
    const float d = target - v;
    return std::fabs(d) <= maxStep ? target : v + (d > 0.0f ? maxStep : -maxStep);
}

struct Params
{
    float leadDegAtRef = 14.0f; // lead (deg) produced by refRate (native tuning 1)
    float refRate = 120.0f;     // deg/s heading rate the leadDegAtRef refers to
    float maxLeadDeg = 22.0f;   // native tuning 2: lead clamp
    float rateOn0 = 25.0f, rateOn1 = 70.0f;   // |heading rate| (deg/s) ramp of the weight: 0 below on0, 1 above on1
    float speedOn0 = 0.8f, speedOn1 = 1.6f;   // m/s ramp of the weight (low speed / idle: 0)
    float rateTc = 0.12f;       // heading rate smoothing
    float measureTc = 0.35f;    // low pass of the measured head-vs-Spine3 yaw (keeps the clip's step sway)
    float fadeIn = 0.25f, fadeOut = 0.20f; // seconds, >= 0.15
    float deadbandDeg = 3.0f;
    float maxCorrDeg = 80.0f;   // bound of one demanded correction (the path law is further bounded by maxTwistDeg)
    float corrTc = 0.06f;
    float maxCorrRate = 300.0f; // deg/s
    float restBiasDeg = 23.39f; // yaw(Head Z) - yaw(Spine3 Z) of the idle pose (motion female rig, film 12-43-58); measurement only
    // yaw(Head Z) - yaw(Spine3 Z) that the twist window (+-maxTwistDeg) is centred on: the run clips keep the head ~1 deg off Spine3
    // (session 2026-10-20: 23.39 shifted the window to [-26.6, +73.4] vs Spine3 and clipped the right circles in 37% of the frames,
    // left 0%). Measured for the female rig only; the same value is used for the male body (no measurement). Window only.
    float twistBiasDeg = 1.0f;
    float pathBlend = 1.0f;     // 0 = lead against Spine3 (first version), 1 = lead against the travel direction (needs Signals.yawValid)
    float pathTc = 0.10f;       // smoothing of the travel direction relative to the actor forward
    float maxTwistDeg = 50.0f;  // |head vs Spine3 - twistBiasDeg| the path law may ask for
    float share[3] = {0.25f, 0.25f, 0.5f}; // Neck, Neck1, Head
    float Gain() const { return refRate > 1.0f ? leadDegAtRef / refRate : 0.0f; }
};

inline bool ValidTuning(float aLeadDegAtRef, float aMaxLeadDeg)
{
    return std::isfinite(aLeadDegAtRef) && std::isfinite(aMaxLeadDeg) && aLeadDegAtRef >= 0.0f && aLeadDegAtRef <= 45.0f &&
           aMaxLeadDeg >= 0.0f && aMaxLeadDeg <= 45.0f;
}

struct Signals
{
    float vx = 0.0f, vy = 0.0f; // world velocity, m/s (Z up)
    bool aiming = false;        // aim / ADS
    bool weaponBusy = false;    // reload / shoot / hipfire pose owner
    bool air = false;
    bool crouch = false;
    bool blocked = false;       // traversal, release, anything that gives the body back to the game
    float yawDeg = 0.0f;        // actor yaw (FeedInput.playerYawDeg; + = left, forward(psi) = (-sin psi, cos psi))
    bool yawValid = false;      // false: the lead is measured against Spine3 (first version)
};

// Bone indices (database order) of the root-to-Head chain.
struct Skeleton
{
    bool ok = false;
    int32_t len = 0;
    int32_t chain[kMaxChain] = {};
    int32_t spine3 = -1, neck = -1, neck1 = -1, head = -1; // database bone indices
};

template<class NameFn, class ParentFn>
inline Skeleton Resolve(uint32_t aCount, NameFn aName, ParentFn aParent)
{
    Skeleton s;
    auto find = [&](std::string_view n) -> int32_t {
        for (uint32_t i = 0; i < aCount; ++i)
            if (std::string_view(aName(i)) == n)
                return static_cast<int32_t>(i);
        return -1;
    };
    s.spine3 = find("Spine3");
    s.neck = find("Neck");
    s.neck1 = find("Neck1");
    s.head = find("Head");
    if (s.spine3 < 0 || s.neck < 0 || s.neck1 < 0 || s.head < 0)
        return s;
    int32_t rev[kMaxChain];
    int32_t n = 0;
    for (int32_t b = s.head; b >= 0 && n < kMaxChain; b = static_cast<int32_t>(aParent(static_cast<uint32_t>(b))))
        rev[n++] = b;
    if (n >= kMaxChain)
        return s;
    for (int32_t i = 0; i < n; ++i)
        s.chain[i] = rev[n - 1 - i];
    s.len = n;
    int32_t pS = -1, pN = -1, pN1 = -1, pH = -1;
    for (int32_t i = 0; i < n; ++i)
    {
        if (s.chain[i] == s.spine3) pS = i;
        if (s.chain[i] == s.neck) pN = i;
        if (s.chain[i] == s.neck1) pN1 = i;
        if (s.chain[i] == s.head) pH = i;
    }
    s.ok = pS >= 0 && pN > pS && pN1 > pN && pH > pN1;
    return s;
}

// The runtime path: bone names / parents of the v2 database (aDb.BoneName / BoneParent); the chain is the one the published pose
// is written with (database bone order = snapshot order = game rig order, mapped by name).
template<class Db>
inline Skeleton ResolveDb(const Db& aDb, uint32_t aCount)
{
    return Resolve(aCount, [&](uint32_t i) { return aDb.BoneName(i); }, [&](uint32_t i) { return aDb.BoneParent(i); });
}

struct Quat
{
    float x = 0, y = 0, z = 0, w = 1;
};
inline Quat Mul(const Quat& a, const Quat& b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat Conj(const Quat& a) { return {-a.x, -a.y, -a.z, a.w}; }
inline bool Norm(Quat& q)
{
    const float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(n2) || n2 < 1.0e-12f)
        return false;
    const float k = 1.0f / std::sqrt(n2);
    q = {q.x * k, q.y * k, q.z * k, q.w * k};
    return true;
}
// Yaw (deg, about +Z, CCW positive) of the bone-local +Z axis rotated by the model quaternion.
inline float AxisYaw(const Quat& q)
{
    const float vx = 2.0f * (q.x * q.z + q.y * q.w);
    const float vy = 2.0f * (q.y * q.z - q.x * q.w);
    return std::atan2(vy, vx) * kDeg;
}

struct Telemetry
{
    float lead = 0.0f, weight = 0.0f, measured = 0.0f, corr = 0.0f, rate = 0.0f;
    float path = 0.0f, headVsPath = 0.0f; // travel direction vs actor forward / head vs travel direction (low passed); path law only
    bool pathLaw = false;
    bool applied = false;
    bool twistClamped = false; // the twist window changed the demanded correction (> 0.5 deg) this frame
};

class HeadLead
{
public:
    void Reset()
    {
        m_rate = 0.0f; m_prevH = 0.0f; m_havePrev = false; m_lead = 0.0f; m_w = 0.0f; m_mf = 0.0f; m_mInit = false;
        m_corr = 0.0f; m_path = 0.0f; m_pathValid = false; m_hpf = 0.0f; m_hInit = false; m_tel = Telemetry{};
    }

    // Feed rate. aDt in seconds (non-finite or <= 0: nothing advances).
    void Step(const Params& aP, const Signals& aS, float aDt)
    {
        if (!std::isfinite(aDt) || aDt <= 0.0f)
            return;
        const float dt = std::fmin(aDt, 0.25f);
        const float vx = std::isfinite(aS.vx) ? aS.vx : 0.0f, vy = std::isfinite(aS.vy) ? aS.vy : 0.0f;
        const float speed = std::sqrt(vx * vx + vy * vy);
        if (speed >= 0.5f)
        {
            const float h = std::atan2(vy, vx) * kDeg;
            if (m_havePrev)
            {
                const float raw = Clamp(Wrap(h - m_prevH) / dt, -720.0f, 720.0f);
                m_rate += (raw - m_rate) * Alpha(dt, aP.rateTc);
            }
            m_prevH = h;
            m_havePrev = true;
            if (aS.yawValid && std::isfinite(aS.yawDeg))
            {
                const float inst = Wrap(h - 90.0f - aS.yawDeg); // travel direction vs actor forward, + = left of forward
                m_path = m_pathValid ? Wrap(m_path + Wrap(inst - m_path) * Alpha(dt, aP.pathTc)) : inst;
                m_pathValid = true;
            }
        }
        else
        {
            m_havePrev = false;
            m_rate *= std::exp(-dt / 0.1f);
        }
        if (!aS.yawValid)
            m_pathValid = false;
        m_lead = Clamp(aP.Gain() * m_rate, -aP.maxLeadDeg, aP.maxLeadDeg);
        const bool gate = !(aS.aiming || aS.weaponBusy || aS.air || aS.crouch || aS.blocked);
        const float target = gate ? Smooth01(std::fabs(m_rate), aP.rateOn0, aP.rateOn1) * Smooth01(speed, aP.speedOn0, aP.speedOn1) : 0.0f;
        const float fade = std::fmax(target > m_w ? aP.fadeIn : aP.fadeOut, 0.15f);
        m_w = Approach(m_w, target, dt / fade);
        m_tel.lead = m_lead;
        m_tel.weight = m_w;
        m_tel.rate = m_rate;
    }

    // Publish rate: aPose = 7 floats per database bone (p.xyz, q.xyzw local). Returns the total yaw correction applied (deg).
    float Apply(const Params& aP, const Skeleton& aSk, float* aPose, uint32_t aBones, float aDt)
    {
        m_tel.applied = false;
        if (!aPose || !aSk.ok || !std::isfinite(aDt) || aDt <= 0.0f)
            return 0.0f;
        const float dt = std::fmin(aDt, 0.25f);
        Quat local[kMaxChain];
        Quat model[kMaxChain];
        Quat run;
        int32_t iS = -1, iH = -1;
        for (int32_t i = 0; i < aSk.len; ++i)
        {
            const int32_t b = aSk.chain[i];
            if (b < 0 || static_cast<uint32_t>(b) >= aBones)
                return 0.0f;
            const float* q = aPose + static_cast<size_t>(b) * 7 + 3;
            local[i] = {q[0], q[1], q[2], q[3]};
            if (!Norm(local[i]))
                return 0.0f;
            run = i == 0 ? local[i] : Mul(run, local[i]);
            model[i] = run;
            if (b == aSk.spine3) iS = i;
            if (b == aSk.head) iH = i;
        }
        const float m = Wrap(AxisYaw(model[iH]) - AxisYaw(model[iS]) - aP.restBiasDeg);
        if (!std::isfinite(m))
            return 0.0f;
        if (!m_mInit)
        {
            m_mf = m;
            m_mInit = true;
        }
        else
            m_mf = Wrap(m_mf + Wrap(m - m_mf) * Alpha(dt, aP.measureTc));
        const bool pathLaw = aP.pathBlend > 0.0f && m_pathValid;
        float err = m_lead - m_mf;
        if (pathLaw)
        {
            const float hv = Wrap(AxisYaw(model[iH]) - m_path); // head vs travel direction (0 = the face looks along the path)
            m_hpf = m_hInit ? Wrap(m_hpf + Wrap(hv - m_hpf) * Alpha(dt, aP.measureTc)) : hv;
            m_hInit = true;
            const float b = Clamp(aP.pathBlend, 0.0f, 1.0f);
            err = (1.0f - b) * (m_lead - m_mf) + b * (m_lead - m_hpf);
        }
        const float ae = std::fabs(err);
        err = ae <= aP.deadbandDeg ? 0.0f : (err > 0.0f ? 1.0f : -1.0f) * std::fmin(ae - aP.deadbandDeg, aP.maxCorrDeg);
        m_tel.twistClamped = false;
        if (pathLaw) // the torso of the clip may be 40 deg off the path: never ask the neck for more than maxTwist from Spine3
        {
            // window centred on head-vs-Spine3 == twistBiasDeg (NOT on the idle pose): tw = yaw(Head) - yaw(Spine3) - twistBias
            const float tw = m_mf + (aP.restBiasDeg - aP.twistBiasDeg);
            const float cl = Clamp(err, -(aP.maxTwistDeg + tw), aP.maxTwistDeg - tw);
            m_tel.twistClamped = std::fabs(cl - err) > 0.5f;
            err = cl;
        }
        const float want = err * m_w;
        const float step = Clamp((want - m_corr) * Alpha(dt, aP.corrTc), -aP.maxCorrRate * dt, aP.maxCorrRate * dt);
        m_corr += step;
        if (std::fabs(m_corr) < 1.0e-4f && want == 0.0f)
            m_corr = 0.0f;
        m_tel.measured = m;
        m_tel.corr = m_corr;
        m_tel.pathLaw = pathLaw;
        m_tel.path = pathLaw ? m_path : 0.0f;
        m_tel.headVsPath = pathLaw ? m_hpf : 0.0f;
        if (m_corr == 0.0f)
            return 0.0f;
        run = Quat{};
        for (int32_t i = 0; i < aSk.len; ++i)
        {
            const int32_t b = aSk.chain[i];
            Quat l = local[i];
            const float share = b == aSk.neck ? aP.share[0] : (b == aSk.neck1 ? aP.share[1] : (b == aSk.head ? aP.share[2] : 0.0f));
            if (share != 0.0f)
            {
                const float h = 0.5f * m_corr * share * kRad;
                const Quat qz{0.0f, 0.0f, std::sin(h), std::cos(h)};
                const Quat parent = i == 0 ? Quat{} : run;
                Quat n = Mul(Mul(Mul(Conj(parent), qz), parent), l);
                if (Norm(n))
                {
                    l = n;
                    float* q = aPose + static_cast<size_t>(b) * 7 + 3;
                    q[0] = l.x; q[1] = l.y; q[2] = l.z; q[3] = l.w;
                }
            }
            run = i == 0 ? l : Mul(run, l);
        }
        m_tel.applied = true;
        return m_corr;
    }

    const Telemetry& Tel() const { return m_tel; }
    float Lead() const { return m_lead; }
    float Weight() const { return m_w; }
    float Corr() const { return m_corr; }
    float Rate() const { return m_rate; }

private:
    float m_rate = 0.0f, m_prevH = 0.0f, m_lead = 0.0f, m_w = 0.0f, m_mf = 0.0f, m_corr = 0.0f, m_path = 0.0f, m_hpf = 0.0f;
    bool m_havePrev = false, m_mInit = false, m_pathValid = false, m_hInit = false;
    Telemetry m_tel;
};
} // namespace utpp::mm::headlead

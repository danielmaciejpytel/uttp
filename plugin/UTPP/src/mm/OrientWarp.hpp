// UTPP orientation warp (opt-in, CET mm.orientWarp): a runtime correction of the pelvis and torso yaw of the v2 pose while V
// runs a circle, so that hips / legs and shoulders face along the PATH of motion instead of away from it.
// Header-only, C++20, no game types, no allocation. Unit test: tests/OrientWarpTest.cpp; offline replay of the 2026-10-19 captures:
// tests/OrientWarpReplay.cpp.
//
// Problem (analysis 2026-10-19, session 23:51, run circles 3.85 m/s, 170 deg/s):
// the shoulders end up -33 deg (left circle, + = left) / +33 deg (right circle) off the path, i.e. OUTSIDE the turn, the pelvis
// -14 / +5 deg, and the neck is twisted 47 deg against the shoulders (head lead looks along the path). Three terms add up:
//   - lag of the actor yaw behind the path (follow law, followTime 0.15): -22 deg,
//   - the visual root yaw offset (MotionOffsetRoot, MMTRACE offYaw -19 left / +19 right): rotates Hips and the whole torso rigidly to
//     the outside of the turn,
//   - the clip's own yaw (F_L_20 / F_R_20 shoulders +8.8 / -8.9 deg into the turn).
// The rigid root yaw offset also turns the clip's inward lean (F_L_20 +5.4, F_R_20 -9.3 deg) into an OUTWARD lean (-10 / -6 deg).
// motion (ABP_SandboxCharacter 1.0: orientation warping node, RotationInterpSpeed 8, DistributedBoneOrientationAlpha 0.5)
// compensates the difference between the character yaw and the path: the pelvis / legs are warped towards the path and the rest is
// distributed over the spine bones (weights 0.2 / 0.2 / 0.2 / 0.4 of Spine .. Spine3, the same weights as motionPose::Warp), so the
// shoulders face the path. mm/MotionOrientationWarp.hpp + MotionOrientationPose.hpp already port that node for the ROOT MOTION direction
// (half angle on the pelvis, chest unchanged); what stays is the residual that the actor lag and the visual root yaw leave.
//
// Design (smallest clean one; every term is a measured bias, so it is clip independent: FL / FR / F / F_L_20 / F_R_20):
//   Step (feed rate)    heading = atan2(vy, vx) of the world velocity, rate = smoothed d(heading)/dt, pathRel = heading - 90 - actorYaw
//                       (travel direction in the actor frame, + = left of forward, same as HeadLead). Weight w -> turnWeight(|rate|)
//                       * speedWeight(speed) while the gate is open (no aiming / weapon action / air / crouch / slide / traversal /
//                       blocked), linear fade in / out >= 0.15 s. Below the turn ramp (straight running) and below the speed ramp
//                       (< 0.8 m/s) w = 0 and every output is exactly 0: the pose is bit-identical.
//   Measure (feed rate, on the v2 pose BEFORE the leg tail and before this layer, so there is no feedback)
//                       pelvisYaw = yaw of the line RightUpLeg -> LeftUpLeg, shoulderYaw = yaw of RightShoulder -> LeftShoulder, both
//                       from the forward kinematics of the pose in the actor model space plus the visual root yaw (the offset root
//                       rotates the pose by it later); vsPath = yaw - pathRel (0 = facing along the path). Two cascaded low passes
//                       (measureTc each, ~ -19 dB at the 1.5 Hz run cadence) remove the natural per-step sway (shoulders +-15 deg)
//                       and keep only the bias, which is what is corrected.
//   Pelvis (a)          extra visual root yaw dp = -(pelvisVsPath - target) * gain, deadband, clamped to +-maxPelvisDeg and so that
//                       |root yaw + dp| <= maxRootYawDeg (the visual root is never pushed outside the turn: the correction is
//                       measured against the path, with the sign of the turn only for the optional 'into the turn' targets). It is
//                       applied as an addition to the provided root frame yaw that the host adopts (MotionSetFrame.rootOffsetYaw): Hips and
//                       the whole body rotate about the visual root point, the processor state (steering, orientation warp, aim
//                       frame) is untouched. The legs do NOT follow the pelvis (motion: the feet are IK targets, the warp turns the
//                       pelvis): Apply counter-rotates LeftUpLeg / RightUpLeg by -legHold * dp in the Hips frame, so thighs, knees
//                       and feet keep their model-space direction and only the hip joints move with the pelvis (+-10 cm * sin(dp), ~3 cm).
//                       Without this the whole foot trajectory turns by dp against the ground and the foot locker has to absorb
//                       3.85 m/s * sin(dp) of lateral slide (offline replay of the real tail: contact toe speed 0.22 -> 0.78 m/s).
//   Torso (b)           the residual twist tau = (shoulderTarget - (shoulderVsPath + dp)) * gain is distributed over Spine .. Spine3 as
//                       a rotation about the model vertical axis (weights 0.2 / 0.2 / 0.2 / 0.4, positions, scales and all other
//                       bones untouched); the shoulder target is 'shoulderIntoDeg' into the turn; the twist between pelvis and Spine3
//                       is bounded by maxTwistDeg (50 deg), tau by maxTorsoDeg; deadband, smoothing (tc) and a rate limit as the head
//                       lead. The head is not touched: HeadLead (applied later on the published pose) keeps measuring the head against
//                       the path, so the neck twist shrinks by what the torso now takes.
//   Gates (driver)      off for aiming, weapon actions (reload / shot / hipfire), a drawn weapon, air, crouch, slide, traversal and
//                       release, and when the root frame of the host is not in use (no provided root yaw to add to).
// OFF (setting) never constructs a call: the pose path is bit-identical. ON with w = 0 (straight running, idle, start / stop, aim,
// reload) returns exactly 0 / 0 and leaves the pose untouched.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>

#include "HeadLead.hpp"

namespace utpp::mm::orientwarp
{
using headlead::Alpha;
using headlead::Approach;
using headlead::Clamp;
using headlead::Conj;
using headlead::kDeg;
using headlead::kMaxChain;
using headlead::kRad;
using headlead::Mul;
using headlead::Norm;
using headlead::Quat;
using headlead::Smooth01;
using headlead::Wrap;

struct Params
{
    float rateOn0 = 25.0f, rateOn1 = 70.0f;   // |heading rate| (deg/s) ramp of the weight: 0 below on0, 1 above on1
    float rateOff0 = 240.0f, rateOff1 = 360.0f; // ... and back to 0 above off1 (pirouettes / hairpins are not a circle)
    float errOff0 = 55.0f, errOff1 = 95.0f;   // |pelvis or shoulders vs path| (deg) fade-out: lateral / backward motion, starts, pivots
    float speedOn0 = 0.8f, speedOn1 = 1.6f;   // m/s ramp of the weight (low speed / idle: 0)
    float rateTc = 0.12f;                     // heading rate smoothing
    float pathTc = 0.10f;                     // smoothing of the travel direction relative to the actor forward
    float fadeIn = 0.25f, fadeOut = 0.20f;    // seconds, both >= 0.15 (floor in Step)
    float measureTc = 0.25f;                  // each of the two cascaded low passes of the measured pelvis / shoulder yaw vs path
    float deadbandDeg = 1.0f;
    // (a) pelvis: extra visual root yaw
    float pelvisGain = 1.0f;                  // share of the pelvis-vs-path error removed
    float pelvisIntoDeg = 5.0f;               // target pelvis vs path, + = into the turn (the lean of the trunk needs the azimuth into the turn)
    float maxPelvisDeg = 30.0f;               // bound of the extra yaw
    float maxRootYawDeg = 45.0f;              // bound of |visual root yaw + extra| (the offset root is never pushed that far)
    float legHold = 1.0f;                     // share of the extra root yaw undone on the legs (1 = thighs / feet keep their direction)
    float hipsPosHold = 1.0f;                 // share of the extra root yaw undone on the Hips POSITION (the host rotates Hips p about the
                                              // root origin: without this the pelvis swings sideways by |p| * sin(dp) and drags the legs along)
    float legPosHold = 0.0f;                  // share of that undone also on the hip joint positions (1 = legs fixed in model space)
    float pelvisTc = 0.12f;                   // smoothing of the applied extra yaw
    float maxPelvisRate = 70.0f;              // deg/s
    // (b) torso: twist over Spine .. Spine3
    float shoulderIntoDeg = 8.0f;             // target shoulders vs path, + = into the turn (the head lead turns the head ~20 deg further)
    float torsoGain = 1.0f;                   // share of the residual shoulder error removed
    float maxTwistDeg = 50.0f;                // |Spine3 - pelvis| after the correction
    float maxTorsoDeg = 45.0f;                // bound of the twist added over the spine
    float torsoTc = 0.12f;
    float maxTorsoRate = 100.0f;              // deg/s
    float share[4] = {0.2f, 0.2f, 0.2f, 0.4f}; // Spine, Spine1, Spine2, Spine3 (motionPose::kMappedCounterWeights)
};

struct Signals
{
    float vx = 0.0f, vy = 0.0f; // world velocity, m/s (Z up)
    bool aiming = false;        // aim / ADS
    bool weaponBusy = false;    // reload / shoot / hipfire pose owner, drawn weapon
    bool air = false;
    bool crouch = false;        // crouch stance or slide
    bool blocked = false;       // traversal, release, setting off: anything that gives the body back to the game
    float yawDeg = 0.0f;        // actor yaw (FeedInput.playerYawDeg; + = left, forward(psi) = (-sin psi, cos psi))
    bool yawValid = false;      // false: no path reference, weight 0
};

// Bone indices (database order) of the pelvis / shoulder lines and the Spine chain.
struct Skeleton
{
    bool ok = false;
    int32_t hips = -1, spine[4] = {-1, -1, -1, -1}, lsh = -1, rsh = -1, lup = -1, rup = -1;
    int32_t len = 0; // root-to-Spine3 chain (apply)
    int32_t hipsPos = -1;                              // position of Hips in chain
    bool legsOk = false;                               // both up legs are direct children of Hips (leg counter-rotation possible)
    int32_t chain[kMaxChain] = {};
    int32_t spinePos[4] = {-1, -1, -1, -1};            // position of Spine .. Spine3 in chain
    int32_t tlen[4] = {0, 0, 0, 0};                    // root-to-bone chains of LeftUpLeg, RightUpLeg, LeftShoulder, RightShoulder
    int32_t tchain[4][kMaxChain] = {};
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
    s.hips = find("Hips");
    s.spine[0] = find("Spine");
    s.spine[1] = find("Spine1");
    s.spine[2] = find("Spine2");
    s.spine[3] = find("Spine3");
    s.lup = find("LeftUpLeg");
    s.rup = find("RightUpLeg");
    s.lsh = find("LeftShoulder");
    s.rsh = find("RightShoulder");
    if (s.hips < 0 || s.lup < 0 || s.rup < 0 || s.lsh < 0 || s.rsh < 0)
        return s;
    for (int32_t k = 0; k < 4; ++k)
        if (s.spine[k] < 0)
            return s;
    auto chainOf = [&](int32_t bone, int32_t* out, int32_t& len) -> bool {
        int32_t rev[kMaxChain];
        int32_t n = 0;
        for (int32_t b = bone; b >= 0 && n < kMaxChain; b = static_cast<int32_t>(aParent(static_cast<uint32_t>(b))))
            rev[n++] = b;
        if (n >= kMaxChain)
            return false;
        for (int32_t i = 0; i < n; ++i)
            out[i] = rev[n - 1 - i];
        len = n;
        return true;
    };
    if (!chainOf(s.spine[3], s.chain, s.len))
        return s;
    for (int32_t k = 0; k < 4; ++k)
        for (int32_t i = 0; i < s.len; ++i)
            if (s.chain[i] == s.spine[k])
                s.spinePos[k] = i;
    // the four spine bones must follow each other on the chain (Spine < Spine1 < Spine2 < Spine3)
    if (s.spinePos[0] < 0 || s.spinePos[1] <= s.spinePos[0] || s.spinePos[2] <= s.spinePos[1] || s.spinePos[3] <= s.spinePos[2])
        return s;
    const int32_t ends[4] = {s.lup, s.rup, s.lsh, s.rsh};
    for (int32_t k = 0; k < 4; ++k)
        if (!chainOf(ends[k], s.tchain[k], s.tlen[k]))
            return s;
    for (int32_t i = 0; i < s.len; ++i)
        if (s.chain[i] == s.hips)
            s.hipsPos = i;
    s.legsOk = s.hipsPos >= 0 && s.tlen[0] >= 2 && s.tlen[1] >= 2 && s.tchain[0][s.tlen[0] - 2] == s.hips && s.tchain[1][s.tlen[1] - 2] == s.hips;
    s.ok = true;
    return s;
}

template<class Db>
inline Skeleton ResolveDb(const Db& aDb, uint32_t aCount)
{
    return Resolve(aCount, [&](uint32_t i) { return aDb.BoneName(i); }, [&](uint32_t i) { return aDb.BoneParent(i); });
}

struct Telemetry
{
    float weight = 0.0f, rate = 0.0f, path = 0.0f;
    float pelvisVsPath = 0.0f, shouldersVsPath = 0.0f; // low passed, before this layer (actor frame + visual root yaw)
    float rootDeg = 0.0f, torsoDeg = 0.0f;             // applied extra root yaw / torso twist
    bool measured = false;
};

class OrientWarp
{
public:
    void Reset()
    {
        m_rate = 0.0f; m_prevH = 0.0f; m_havePrev = false; m_w = 0.0f;
        m_path = 0.0f; m_pathValid = false;
        m_p1 = m_p2 = m_s1 = m_s2 = 0.0f; m_fInit = false;
        m_dp = 0.0f; m_tau = 0.0f;
        m_tel = Telemetry{};
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
        const bool gate = m_pathValid && !(aS.aiming || aS.weaponBusy || aS.air || aS.crouch || aS.blocked);
        const float ar = std::fabs(m_rate);
        const float target = gate ? Smooth01(ar, aP.rateOn0, aP.rateOn1) * (1.0f - Smooth01(ar, aP.rateOff0, aP.rateOff1)) *
                                        Smooth01(speed, aP.speedOn0, aP.speedOn1)
                                  : 0.0f;
        const float fade = std::fmax(target > m_w ? aP.fadeIn : aP.fadeOut, 0.15f);
        m_w = Approach(m_w, target, dt / fade);
        m_tel.weight = m_w;
        m_tel.rate = m_rate;
        m_tel.path = m_pathValid ? m_path : 0.0f;
    }

    // Measure and update the applied corrections. aPose = 7 floats per database bone (p.xyz, q.xyzw local), the v2 pose BEFORE the
    // visual root yaw is applied; aRootYawDeg = that yaw (rootframe Frame().yaw in degrees, 0 when none). Returns the extra root yaw
    // (deg); TorsoDeg() is the twist for Apply. Always advances the correction towards its (possibly zero) target.
    float Measure(const Params& aP, const Skeleton& aSk, const float* aPose, uint32_t aBones, float aRootYawDeg, float aDt)
    {
        m_tel.measured = false;
        if (!aPose || !aSk.ok || !std::isfinite(aDt) || aDt <= 0.0f || !std::isfinite(aRootYawDeg))
            return Relax(aP, aDt);
        const float dt = std::fmin(aDt, 0.25f);
        float py = 0.0f, sy = 0.0f;
        if (!LineYaw(aSk, aPose, aBones, 0, 1, py) || !LineYaw(aSk, aPose, aBones, 2, 3, sy))
            return Relax(aP, aDt);
        if (!m_pathValid)
            return Relax(aP, aDt); // no reference: hold the filters, take the corrections back to 0
        const float pv = Wrap(py + aRootYawDeg - m_path);
        const float sv = Wrap(sy + aRootYawDeg - m_path);
        if (!m_fInit)
        {
            m_p1 = m_p2 = pv;
            m_s1 = m_s2 = sv;
            m_fInit = true;
        }
        else
        {
            const float a = Alpha(dt, aP.measureTc);
            m_p1 = Wrap(m_p1 + Wrap(pv - m_p1) * a);
            m_p2 = Wrap(m_p2 + Wrap(m_p1 - m_p2) * a);
            m_s1 = Wrap(m_s1 + Wrap(sv - m_s1) * a);
            m_s2 = Wrap(m_s2 + Wrap(m_s1 - m_s2) * a);
        }
        const float sgn = m_rate >= 0.0f ? 1.0f : -1.0f; // + into the turn = towards the side of the turn
        // a pose that is far off the path (lateral / backward loops, starts, pivots) is not a circle bias: fade the correction out
        const float errW = 1.0f - Smooth01(std::fmax(std::fabs(m_p2), std::fabs(m_s2)), aP.errOff0, aP.errOff1);
        const float w = m_w * errW;
        // pelvis: bring the pelvis line onto the path (target 'into' the turn), bounded, the root never pushed too far
        float ep = Wrap(sgn * aP.pelvisIntoDeg - m_p2);
        ep = Deadband(ep, aP.deadbandDeg) * aP.pelvisGain;
        float wantP = Clamp(ep, -aP.maxPelvisDeg, aP.maxPelvisDeg);
        wantP = Clamp(wantP, -aP.maxRootYawDeg - aRootYawDeg, aP.maxRootYawDeg - aRootYawDeg);
        wantP *= w;
        m_dp += Clamp((wantP - m_dp) * Alpha(dt, aP.pelvisTc), -aP.maxPelvisRate * dt, aP.maxPelvisRate * dt);
        if (std::fabs(m_dp) < 2.0e-3f && wantP == 0.0f)
            m_dp = 0.0f;
        // torso: the residual shoulder error after the (applied) pelvis correction, twist to the pelvis bounded
        const float twist0 = Wrap(m_s2 - m_p2);
        float es = Wrap(sgn * aP.shoulderIntoDeg - (m_s2 + m_dp));
        es = Deadband(es, aP.deadbandDeg) * aP.torsoGain;
        float wantT = Clamp(es, -aP.maxTorsoDeg, aP.maxTorsoDeg);
        wantT = Clamp(wantT, -aP.maxTwistDeg - twist0, aP.maxTwistDeg - twist0);
        wantT *= w;
        m_tau += Clamp((wantT - m_tau) * Alpha(dt, aP.torsoTc), -aP.maxTorsoRate * dt, aP.maxTorsoRate * dt);
        if (std::fabs(m_tau) < 2.0e-3f && wantT == 0.0f)
            m_tau = 0.0f;
        m_tel.measured = true;
        m_tel.pelvisVsPath = m_p2;
        m_tel.shouldersVsPath = m_s2;
        m_tel.rootDeg = m_dp;
        m_tel.torsoDeg = m_tau;
        return m_dp;
    }

    // Rotates Spine .. Spine3 about the model vertical axis by TorsoDeg() (shares) and counter-rotates the two up legs by
    // -legHold * RootDeg() (the caller adds RootDeg() to the visual root yaw, which turns Hips and everything below), in place.
    // Returns the twist applied (deg).
    float Apply(const Params& aP, const Skeleton& aSk, float* aPose, uint32_t aBones) const
    {
        return ApplyAngles(aP, aSk, aPose, aBones, m_dp, m_tau);
    }

    // The same edit for given angles: aRootDeg = extra visual root yaw the caller adds to the root frame (legs / Hips position turned
    // back by legHold / hipsPosHold of it), aTorsoDeg = twist over Spine .. Spine3. Used by Apply and by the pelvis yaw trim
    // (PelvisTrim below: root +trim, torso -trim, so only the pelvis turns). Returns aTorsoDeg (0 = nothing done).
    static float ApplyAngles(const Params& aP, const Skeleton& aSk, float* aPose, uint32_t aBones, float aRootDeg, float aTorsoDeg)
    {
        const float dp = aRootDeg, tau = aTorsoDeg;
        if ((tau == 0.0f && dp == 0.0f) || !aPose || !aSk.ok || !std::isfinite(dp) || !std::isfinite(tau))
            return 0.0f;
        Quat local[kMaxChain];
        for (int32_t i = 0; i < aSk.len; ++i)
        {
            const int32_t b = aSk.chain[i];
            if (b < 0 || static_cast<uint32_t>(b) >= aBones)
                return 0.0f;
            const float* q = aPose + static_cast<size_t>(b) * 7 + 3;
            local[i] = {q[0], q[1], q[2], q[3]};
            if (!Norm(local[i]))
                return 0.0f;
        }
        Quat run;
        for (int32_t i = 0; i < aSk.len; ++i)
        {
            Quat l = local[i];
            float share = 0.0f;
            for (int32_t k = 0; k < 4; ++k)
                if (aSk.spinePos[k] == i)
                    share = aP.share[k];
            if (i == aSk.hipsPos && dp != 0.0f && aP.hipsPosHold != 0.0f)
            {
                // Hips local position (Root frame): the host turns it by dp about the root origin, so turn it back here
                const float hh = 0.5f * (-aP.hipsPosHold * dp) * kRad;
                const Quat parent = i == 0 ? Quat{} : run;
                const Quat kh = Mul(Mul(Conj(parent), Quat{0.0f, 0.0f, std::sin(hh), std::cos(hh)}), parent);
                float* p = aPose + static_cast<size_t>(aSk.chain[i]) * 7;
                float r[3];
                Rotate(kh, p, r);
                p[0] = r[0]; p[1] = r[1]; p[2] = r[2];
            }
            if (i == aSk.hipsPos && aSk.legsOk && dp != 0.0f && aP.legHold != 0.0f)
                CounterRotateLegs(aSk, aPose, aBones, run_after(run, l, i), -aP.legHold * dp, aP.legPosHold);
            if (share != 0.0f && tau != 0.0f)
            {
                const float h = 0.5f * tau * share * kRad;
                const Quat qz{0.0f, 0.0f, std::sin(h), std::cos(h)};
                const Quat parent = i == 0 ? Quat{} : run;
                Quat n = Mul(Mul(Mul(Conj(parent), qz), parent), l);
                if (Norm(n))
                {
                    l = n;
                    float* q = aPose + static_cast<size_t>(aSk.chain[i]) * 7 + 3;
                    q[0] = l.x; q[1] = l.y; q[2] = l.z; q[3] = l.w;
                }
            }
            run = i == 0 ? l : Mul(run, l);
        }
        return tau;
    }

    const Telemetry& Tel() const { return m_tel; }
    float Weight() const { return m_w; }
    float RootDeg() const { return m_dp; }
    float TorsoDeg() const { return m_tau; }
    bool Idle() const { return m_w == 0.0f && m_dp == 0.0f && m_tau == 0.0f; }

    // Yaw (deg, CCW positive from +Y, model space) of the line from bone aA to bone aB of the target chains (0 = LeftUpLeg, 1 = RightUpLeg,
    // 2 = LeftShoulder, 3 = RightShoulder) as the FACING of a left-right pair: aA is the left bone, aB the right one. Exposed for tests.
    static bool LineYaw(const Skeleton& aSk, const float* aPose, uint32_t aBones, int32_t aLeft, int32_t aRight, float& aOut)
    {
        float l[3], r[3];
        if (!Fk(aSk, aPose, aBones, aLeft, l) || !Fk(aSk, aPose, aBones, aRight, r))
            return false;
        const float vx = l[0] - r[0], vy = l[1] - r[1];
        if (!std::isfinite(vx) || !std::isfinite(vy) || vx * vx + vy * vy < 1.0e-10f)
            return false;
        aOut = Wrap(std::atan2(-vx, vy) * kDeg - 90.0f);
        return true;
    }

private:
    static Quat run_after(const Quat& aRun, const Quat& aLocal, int32_t aIndex) { return aIndex == 0 ? aLocal : Mul(aRun, aLocal); }
    // up legs: local = conj(H) * Rz(angle) * H * local, H = model rotation of Hips (the legs keep their model direction when Hips turns)
    static void CounterRotateLegs(const Skeleton& aSk, float* aPose, uint32_t aBones, const Quat& aHipsModel, float aDeg, float aPosHold)
    {
        const float h = 0.5f * aDeg * kRad;
        const Quat qz{0.0f, 0.0f, std::sin(h), std::cos(h)};
        const Quat k = Mul(Mul(Conj(aHipsModel), qz), aHipsModel);
        const float hp = h * aPosHold;
        const Quat kp = Mul(Mul(Conj(aHipsModel), Quat{0.0f, 0.0f, std::sin(hp), std::cos(hp)}), aHipsModel);
        for (const int32_t b : {aSk.lup, aSk.rup})
        {
            if (b < 0 || static_cast<uint32_t>(b) >= aBones)
                continue;
            float* q = aPose + static_cast<size_t>(b) * 7 + 3;
            Quat l{q[0], q[1], q[2], q[3]};
            if (!Norm(l))
                continue;
            Quat n = Mul(k, l);
            if (Norm(n))
            {
                q[0] = n.x; q[1] = n.y; q[2] = n.z; q[3] = n.w;
                if (aPosHold != 0.0f) // hip joint position in the Hips frame, turned back by the same share
                {
                    float* p = aPose + static_cast<size_t>(b) * 7;
                    float r[3];
                    Rotate(kp, p, r);
                    p[0] = r[0]; p[1] = r[1]; p[2] = r[2];
                }
            }
        }
    }
    float Relax(const Params& aP, float aDt)
    {
        // no measurement: the corrections return to 0 at the same limited rate (never a jump)
        if (std::isfinite(aDt) && aDt > 0.0f)
        {
            const float dt = std::fmin(aDt, 0.25f);
            m_dp += Clamp((0.0f - m_dp) * Alpha(dt, aP.pelvisTc), -aP.maxPelvisRate * dt, aP.maxPelvisRate * dt);
            m_tau += Clamp((0.0f - m_tau) * Alpha(dt, aP.torsoTc), -aP.maxTorsoRate * dt, aP.maxTorsoRate * dt);
            if (std::fabs(m_dp) < 2.0e-3f)
                m_dp = 0.0f;
            if (std::fabs(m_tau) < 2.0e-3f)
                m_tau = 0.0f;
        }
        m_tel.rootDeg = m_dp;
        m_tel.torsoDeg = m_tau;
        return m_dp;
    }
    static float Deadband(float aE, float aDb)
    {
        const float ae = std::fabs(aE);
        return ae <= aDb ? 0.0f : (aE > 0.0f ? 1.0f : -1.0f) * (ae - aDb);
    }
    static void Rotate(const Quat& q, const float* v, float* o)
    {
        // v + 2 w (u x v) + 2 u x (u x v)
        const float cx = q.y * v[2] - q.z * v[1], cy = q.z * v[0] - q.x * v[2], cz = q.x * v[1] - q.y * v[0];
        o[0] = v[0] + 2.0f * (q.w * cx + q.y * cz - q.z * cy);
        o[1] = v[1] + 2.0f * (q.w * cy + q.z * cx - q.x * cz);
        o[2] = v[2] + 2.0f * (q.w * cz + q.x * cy - q.y * cx);
    }
    // Model-space position of target bone aT (0..3) by the forward kinematics of the local pose along its root chain.
    static bool Fk(const Skeleton& aSk, const float* aPose, uint32_t aBones, int32_t aT, float* aOut)
    {
        float pos[3] = {0.0f, 0.0f, 0.0f};
        Quat run;
        for (int32_t i = 0; i < aSk.tlen[aT]; ++i)
        {
            const int32_t b = aSk.tchain[aT][i];
            if (b < 0 || static_cast<uint32_t>(b) >= aBones)
                return false;
            const float* p = aPose + static_cast<size_t>(b) * 7;
            Quat q{p[3], p[4], p[5], p[6]};
            if (!Norm(q))
                return false;
            float r[3];
            Rotate(run, p, r);
            pos[0] += r[0]; pos[1] += r[1]; pos[2] += r[2];
            run = i == 0 ? q : Mul(run, q);
        }
        aOut[0] = pos[0]; aOut[1] = pos[1]; aOut[2] = pos[2];
        return std::isfinite(pos[0]) && std::isfinite(pos[1]) && std::isfinite(pos[2]);
    }

    float m_rate = 0.0f, m_prevH = 0.0f, m_w = 0.0f, m_path = 0.0f;
    float m_p1 = 0.0f, m_p2 = 0.0f, m_s1 = 0.0f, m_s2 = 0.0f, m_dp = 0.0f, m_tau = 0.0f;
    bool m_havePrev = false, m_pathValid = false, m_fInit = false;
    Telemetry m_tel;
};

// Run pelvis yaw trim (CET mm.pelvisYawTrim, deg, + = left, default 0 = nothing). The motion forward loops carry the pelvis about
// 5 deg to the right of the travel direction (database means: Run_Loop_F -4.7, Walk_Loop_F -4.7, Sprint_Loop_F -4.8; chest about 0,
// head forward; the source FBX shows the same values, in game -5 .. -6.7). The trim is added to the visual root yaw of the leg-tail
// frame while the torso is twisted back by the same angle and the legs keep their direction (ApplyAngles(root +t, torso -t)): only
// the pelvis turns, chest, head and feet stay. The matcher never sees it (the processor state and the bundle pose are untouched; it
// edits the frame copy only, like the orientation warp). Weight: ground forward locomotion (speed ramp, travel direction within
// fwdOff0 .. fwdOff1 of the actor forward), 0 while aiming, with a weapon drawn / weapon action, in the air, crouching / sliding or
// blocked (traversal, release, no root frame); linear fades. The caller scales it by (1 - orientation warp weight): in run circles
// the warp already brings the pelvis onto the path.
inline constexpr float kPelvisTrimMaxDeg = 10.0f;
inline bool ValidPelvisTrim(float aDeg) { return std::isfinite(aDeg) && aDeg >= -kPelvisTrimMaxDeg && aDeg <= kPelvisTrimMaxDeg; }

struct TrimParams
{
    float speedOn0 = 0.8f, speedOn1 = 1.6f; // m/s ramp (idle / slow: 0)
    float fwdOff0 = 30.0f, fwdOff1 = 60.0f; // |travel direction vs actor forward| (deg): full below, 0 above (strafe / back: no trim)
    float pathTc = 0.10f;
    float fadeIn = 0.30f, fadeOut = 0.20f;  // s
};

class PelvisTrim
{
public:
    void Reset() { m_w = 0.0f; m_path = 0.0f; m_pathValid = false; m_motion = "idle"; }

    // Feed rate (feed dt: replays stay deterministic). Same Signals as the orientation warp.
    void Step(const TrimParams& aP, const Signals& aS, float aDt)
    {
        if (!std::isfinite(aDt) || aDt <= 0.0f)
            return;
        const float dt = std::fmin(aDt, 0.25f);
        const float vx = std::isfinite(aS.vx) ? aS.vx : 0.0f, vy = std::isfinite(aS.vy) ? aS.vy : 0.0f;
        const float speed = std::sqrt(vx * vx + vy * vy);
        if (speed >= 0.5f && aS.yawValid && std::isfinite(aS.yawDeg))
        {
            const float inst = Wrap(std::atan2(vy, vx) * kDeg - 90.0f - aS.yawDeg); // + = left of the actor forward
            m_path = m_pathValid ? Wrap(m_path + Wrap(inst - m_path) * Alpha(dt, aP.pathTc)) : inst;
            m_pathValid = true;
        }
        else if (speed < 0.5f || !aS.yawValid)
            m_pathValid = false;
        const bool gate = m_pathValid && !(aS.aiming || aS.weaponBusy || aS.air || aS.crouch || aS.blocked);
        const float target = gate ? Smooth01(speed, aP.speedOn0, aP.speedOn1) * (1.0f - Smooth01(std::fabs(m_path), aP.fwdOff0, aP.fwdOff1)) : 0.0f;
        // diagnostics (trace pt=): what holds the target below 1 when no gate signal does
        m_motion = !m_pathValid ? "idle" : speed < aP.speedOn1 ? "slow" : std::fabs(m_path) > aP.fwdOff0 ? "dir" : "ok";
        const float fade = std::fmax(target > m_w ? aP.fadeIn : aP.fadeOut, 0.05f);
        m_w = Approach(m_w, target, dt / fade);
    }
    float Weight() const { return m_w; }
    // "idle" (no travel direction / < 0.5 m/s), "slow" (below speedOn1), "dir" (travel off the forward window), "ok"; gate signals are
    // reported by the caller (they come first)
    const char* MotionReason() const { return m_motion; }
    // effective trim (deg) for the user value aTrimDeg; 0 for an invalid value or weight 0
    float Deg(float aTrimDeg) const { return ValidPelvisTrim(aTrimDeg) ? m_w * aTrimDeg : 0.0f; }

private:
    float m_w = 0.0f, m_path = 0.0f;
    bool m_pathValid = false;
    const char* m_motion = "idle";
};

// Body yaw offset (CET mm.bodyYawOffset, deg, + = left, default 0 = nothing). The camera sits over the right shoulder, so V reads
// as turned to the right; the offset turns the WHOLE character (pelvis, torso, head, legs and feet as one rigid pose) about the
// visual root by a constant yaw. The caller only adds it to the visual root yaw of the leg-tail frame (no ApplyAngles: no leg hold,
// no torso counter twist); the entity / capsule yaw, the movement direction, the matcher / processor state and the bundle pose are
// untouched. Weight: 1 on foot in every stance and gait (idle, walk, run, sprint, crouch, slide, air), weapon holstered or drawn;
// 0 while aiming, during a weapon action (shot, reload, hip-fire pose, melee) so the aim pose points at the crosshair, and while
// blocked (traversal, release, no root frame); linear fades (feed dt). Signals.air / crouch / velocity / yaw are not used.
inline constexpr float kBodyYawOffsetMaxDeg = 20.0f;
inline bool ValidBodyYawOffset(float aDeg) { return std::isfinite(aDeg) && aDeg >= -kBodyYawOffsetMaxDeg && aDeg <= kBodyYawOffsetMaxDeg; }

struct BodyOffsetParams
{
    float fadeIn = 0.25f, fadeOut = 0.25f; // s
};

class BodyYawOffset
{
public:
    void Reset() { m_w = 0.0f; }

    // Feed rate (feed dt: replays stay deterministic).
    void Step(const BodyOffsetParams& aP, const Signals& aS, float aDt)
    {
        if (!std::isfinite(aDt) || aDt <= 0.0f)
            return;
        const float dt = std::fmin(aDt, 0.25f);
        const float target = (aS.aiming || aS.weaponBusy || aS.blocked) ? 0.0f : 1.0f;
        const float fade = std::fmax(target > m_w ? aP.fadeIn : aP.fadeOut, 0.05f);
        m_w = Approach(m_w, target, dt / fade);
    }
    float Weight() const { return m_w; }
    // effective offset (deg) for the user value aOffsetDeg; 0 for an invalid value or weight 0
    float Deg(float aOffsetDeg) const { return ValidBodyYawOffset(aOffsetDeg) ? m_w * aOffsetDeg : 0.0f; }

private:
    float m_w = 0.0f;
};
} // namespace utpp::mm::orientwarp

// UTPP motion matching runtime - motion lateral-acceleration LEAN layer (opt-in, CET mm.lean / mm.leanHead).
// Header-only, C++20, no game types, no allocation per frame (all buffers are members).
// (the pack Data\utpp_lean_<body>.bin is baked by tools/mmbake/leanbake.py). Unit test: tests/LeanRuntimeTest.cpp;
// offline replay of the 2026-10-19 captures: tests/LeanReplay.cpp.
//
// What it does (motion SandboxCharacter_Mover_ABP Update_AdditiveLean + AdditiveLeans, see LeanPack.hpp for the ABP notes):
//   lat   = clamp(a_right / MapRangeClamped(speed 200..320 -> 500..800 cm/s^2), -1, 1), a_right = acceleration component to the RIGHT
//           of the velocity heading; (x, y) = lat * dirScale[MovementDirection] (F: (lat, 0)); the blend spaces Relaxed run / walk
//           lean (5 samples each) and the head lean (13 samples) give sample weights which are smoothed like UBlendSpace (1.2 / 1.2 /
//           1.75 per s); body = walk -> run crossfade by speed (200..320 cm/s, interp 10/s) as mesh space rotation offset additive,
//           head additive on top; weight = gate * (1 - Disable_AdditiveLeans curve of the playing clip, interp 5/s).
//
// Differences to the ABP, all measured on the real captures:
//   * The ABP takes lat from (Velocity - VelocityLastFrame) / dt of the CMC. The v2 feed velocity ('velWorld') changes in STEPS every
//     2-3 frames, so the same difference is bimodal (+-1 spikes between zeros, mean +-0.49) and the smoothed lean would settle at
//     half the real value. The lateral acceleration is therefore low-passed (time constant accelTc ~0.14 s, time weighted so the mean
//     is exact) BEFORE the clamp: circles at 3.85 m/s give |lat| ~0.97, straight running < 0.03.
//   * MovementDirection is always F (99.4% of the moving frames of the real sessions are within 60 deg of the body axis).
//   * Gates (all fade linearly in gateFadeS = 0.3 s): v2 active and not released, no traversal / slide frame, OnGround + Moving, no
//     crouch / air, no aiming / reload / hipfire / weapon action / drawn weapon (the conservative gate of the orientation warp);
//     lat ramps in over speed 0.8 .. 1.6 m/s so a start / stop never leans.
//   * The lean additive moves the Hips 14 cm sideways and 5 cm down and swings the legs, so the ankles end up 22 .. 30 cm away from
//     where the gait put them (replay of the 23:51 circles: planted toe speed 0.27 -> 1.0 m/s in the final pose, 4 x the foot locker
//     correction). motion pins the feet with its foot IK after the lean layer; Apply does the same with a two-bone leg IK
//     (legik::SolveTwoBone, thigh / shin / foot rotations only) back onto the model-space ankles and foot rotations of the unleaned
//     pose (Params::anchorFeet), so the gait, the contacts and the foot locker see the same feet as without the lean.
//   * The head layer (setting leanHead, default off) is applied only when asked; it replaces the HeadLead layer of the driver
//     (never summed with it: the head lead is a closed loop on the path angle and would fight the lean head).
// OFF (no call) and ON with weight 0 (straight running, idle, aim, reload, ...): Idle() is true, Apply is never called, the pose is
// bit-identical.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "LeanPack.hpp"
#include "LegIk.hpp"
#include "OverlayFile.hpp"

namespace utpp::mm::leanrt
{
namespace ln = utpp::mm::lean;

static_assert(ln::kRigIdentityWoman == ov::kRigIdentityWomanOv && ln::kRigIdentityMan == ov::kRigIdentityManOv,
              "lean pack rig identities must equal the overlay / database rig identities");
static_assert(ln::kNumBones <= ov::kMaxBones, "lean pack bones must fit an overlay pose");

struct Params
{
    float scale = 1.0f;     // body lean amount: multiplier of the apply alpha (1 = motion strength)
    float headScale = 1.0f; // head lean amount
    float accelTc = 0.14f;  // s, low pass of the lateral acceleration (time weighted)
    float speedRamp0 = 0.8f, speedRamp1 = 1.6f; // m/s, lat ramp (start / stop never leans)
    float gateFadeS = 0.3f; // s, linear fade of the gate (PARM bool_blend_s / state_blend_s)
    float maxAccel = 60.0f; // m/s^2, clamp of one raw lateral acceleration sample (teleports; a lower clamp biases the mean of the spiky feed low)
    float dtMax = 0.1f;     // s, longer feed gaps reset the estimator
    float staleS = 0.2f;    // s, a velocity unchanged for longer than this has no acceleration
    bool anchorFeet = true; // legs: two-bone IK back onto the model-space ankles of the unleaned pose (see Apply)
    float maxExtension = 0.995f, softExtension = 0.97f; // anchor IK reach (FootLocker values)
};

// Native range check (UTPPSetMotionLeanParams): finite and 0 .. 2.
inline bool ScaleValid(float aScale) { return std::isfinite(aScale) && aScale >= 0.0f && aScale <= 2.0f; }

struct Signals // one per v2 feed
{
    float vx = 0.0f, vy = 0.0f; // world velocity m/s (feed 'velWorld')
    bool velValid = false;
    bool moving = false;        // cs.movementState == Moving && cs.movementMode == OnGround
    bool aiming = false, weaponBusy = false, air = false, crouch = false, slide = false, blocked = false;
    bool head = false;          // head lean wanted (setting mm.leanHead)
    int direction = ln::kDirF;  // E_MovementDirection
    int curveClip = -1;         // pack CLIP index of the played clip (Disable_AdditiveLeans) or -1
    float clipSeconds = 0.0f;   // time inside that clip
};

struct Telemetry
{
    float speed = 0.0f, aLat = 0.0f, lat = 0.0f, x = 0.0f, y = 0.0f, walkRun = 0.0f, disable = 0.0f, gate = 0.0f, headGate = 0.0f;
    float weight = 0.0f; // body apply alpha before scale (gate * (1 - disable))
    int tri[3] = {-1, -1, -1};
};

inline float FInterpTo(float aCur, float aTarget, float aDt, float aSpeed)
{
    if (!(aSpeed > 0.0f))
        return aTarget;
    const float d = aTarget - aCur;
    if (d * d < 1e-10f)
        return aTarget;
    return aCur + d * std::clamp(aDt * aSpeed, 0.0f, 1.0f);
}

class LeanRuntime
{
public:
    // Layers by id ("run" and "walk" are required, "head" optional). False = the pack cannot drive the lean.
    bool Bind(const ln::LeanPack& aPack)
    {
        m_run = m_walk = m_head = -1;
        Reset();
        if (!aPack.Ok())
            return false;
        m_run = aPack.FindLayer("run");
        m_walk = aPack.FindLayer("walk");
        m_head = aPack.FindLayer("head");
        if (m_run < 0 || m_walk < 0 || aPack.LayerAt(m_run).dims != 2 || aPack.LayerAt(m_walk).dims != 2)
        {
            m_run = m_walk = m_head = -1;
            return false;
        }
        if (m_head >= 0 && aPack.LayerAt(m_head).dims != 2)
            m_head = -1;
        // leg bones for the foot anchor (missing = no anchor, the lean still works)
        m_legsOk = true;
        static const char* const kLeg[2][4] = {{"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase"},
                                               {"RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"}};
        for (int side = 0; side < 2; ++side)
        {
            int idx[4] = {-1, -1, -1, -1};
            for (int k = 0; k < ln::kNumBones; ++k)
                for (int j = 0; j < 4; ++j)
                    if (aPack.BoneName(k) == kLeg[side][j])
                        idx[j] = k;
            m_up[side] = idx[0];
            m_shin[side] = idx[1];
            m_foot[side] = idx[2];
            m_toe[side] = idx[3];
            m_legsOk = m_legsOk && idx[0] >= 0 && idx[1] >= 0 && idx[2] >= 0 && aPack.Skel().parent[idx[0]] >= 0 &&
                       aPack.Skel().parent[idx[1]] == idx[0] && aPack.Skel().parent[idx[2]] == idx[1];
        }
        return true;
    }
    bool HasHead() const { return m_head >= 0; }

    void Reset()
    {
        m_runSt.Reset();
        m_walkSt.Reset();
        m_headSt.Reset();
        m_runOk = m_walkOk = m_headOk = false;
        m_haveVel = false;
        m_pvx = m_pvy = 0.0f;
        m_axHeld = m_ayHeld = m_sinceChange = 0.0f;
        m_aLP = 0.0f;
        m_gate = m_headGate = 0.0f;
        m_want = m_headWant = false;
        m_walkRun = 0.0f;
        m_haveWalkRun = false;
        m_disable = 0.0f;
        m_haveDisable = false;
        m_tel = Telemetry{};
    }

    // Nothing runs and Apply must not be called: the pose path is untouched.
    bool Idle() const { return !m_want && !m_headWant && m_gate <= 0.0f && m_headGate <= 0.0f; }
    float Weight() const { return m_gate * (1.0f - m_disable); }
    float HeadWeight() const { return m_headGate * (1.0f - m_disable); }
    const Telemetry& Tel() const { return m_tel; }
    // The head layer is in use (asked for, bound and not faded out): the driver blocks its own head lead meanwhile.
    bool HeadActive() const { return m_head >= 0 && (m_headWant || m_headGate > 0.0f); }

    // Feed rate: estimator, gates, blend space weights, crossfade and curve. Cheap; allocation free.
    void Step(const ln::LeanPack& aPack, const Params& aP, const Signals& aS, float aDt)
    {
        float dt = aDt;
        if (!std::isfinite(dt) || dt <= 0.0f)
            dt = 1.0f / 60.0f;
        const bool gap = dt > aP.dtMax;
        dt = std::min(dt, aP.dtMax);

        // ---- lateral acceleration estimator (low pass of the unclamped component to the right of the heading)
        const bool vOk = aS.velValid && std::isfinite(aS.vx) && std::isfinite(aS.vy);
        const float vx = vOk ? aS.vx : 0.0f, vy = vOk ? aS.vy : 0.0f;
        const float speed = std::sqrt(vx * vx + vy * vy);
        float hx = 0.0f, hy = 0.0f;
        const bool heading = speed > 0.05f;
        if (heading)
        {
            hx = vx / speed;
            hy = vy / speed;
        }
        if (!vOk || gap || !m_haveVel)
        {
            m_aLP = 0.0f; // no previous velocity (first feed, gap, invalid): no acceleration
            m_axHeld = m_ayHeld = 0.0f;
            m_sinceChange = 0.0f;
            m_pvx = vx;
            m_pvy = vy;
        }
        else
        {
            // The feed velocity changes in steps every 2-3 frames: the acceleration is the change over the time since the LAST CHANGE
            // (piecewise constant, mean exact), not over one frame (spikes between zeros). A source that changes every frame gives
            // exactly the ABP frame difference. Unchanged for longer than staleS = constant velocity = no acceleration.
            m_sinceChange += dt;
            if (std::fabs(vx - m_pvx) > 1e-6f || std::fabs(vy - m_pvy) > 1e-6f)
            {
                const float span = std::max(m_sinceChange, 0.001f);
                m_axHeld = (vx - m_pvx) / span;
                m_ayHeld = (vy - m_pvy) / span;
                m_pvx = vx;
                m_pvy = vy;
                m_sinceChange = 0.0f;
            }
            else if (m_sinceChange > aP.staleS)
                m_axHeld = m_ayHeld = 0.0f;
            const float aRight = heading ? std::clamp(m_axHeld * hy - m_ayHeld * hx, -aP.maxAccel, aP.maxAccel) : 0.0f;
            const float k = aP.accelTc > 1e-4f ? 1.0f - std::exp(-dt / aP.accelTc) : 1.0f;
            m_aLP += (aRight - m_aLP) * k;
        }
        m_haveVel = vOk;
        if (!std::isfinite(m_aLP))
            m_aLP = 0.0f;

        // ---- gates (linear fade)
        m_want = !aS.blocked && aS.moving && !aS.crouch && !aS.air && !aS.aiming && !aS.weaponBusy && !aS.slide && vOk;
        m_headWant = m_want && aS.head && m_head >= 0;
        const float fadeStep = aP.gateFadeS > 1e-4f ? dt / aP.gateFadeS : 1.0f;
        m_gate += std::clamp((m_want ? 1.0f : 0.0f) - m_gate, -fadeStep, fadeStep);
        m_headGate += std::clamp((m_headWant ? 1.0f : 0.0f) - m_headGate, -fadeStep, fadeStep);
        if (m_gate < 1e-4f && !m_want)
            m_gate = 0.0f;
        if (m_headGate < 1e-4f && !m_headWant)
            m_headGate = 0.0f;

        // ---- Disable_AdditiveLeans (curve of the played clip, interp 5/s) and the walk -> run crossfade (interp 10/s)
        float curve = 0.0f;
        if (aS.curveClip >= 0 && aS.curveClip < aPack.NumClips() && std::isfinite(aS.clipSeconds))
            curve = std::clamp(aPack.DisableCurveAt(aS.curveClip, aS.clipSeconds), 0.0f, 1.0f);
        const ln::Params& pp = aPack.P();
        m_disable = m_haveDisable ? FInterpTo(m_disable, curve, dt, pp.disableCurveInterpPerS) : curve;
        m_haveDisable = true;
        const float wr = ln::WalkRunAlpha(pp, speed);
        m_walkRun = m_haveWalkRun ? FInterpTo(m_walkRun, wr, dt, pp.xfadeInterpPerS) : wr;
        m_haveWalkRun = true;

        // ---- blend space input and weights (only while the lean is wanted or still fading; idle resets so re-entry snaps)
        float lat = 0.0f, x = 0.0f, y = 0.0f;
        if (m_want || m_gate > 0.0f || m_headWant || m_headGate > 0.0f)
        {
            const float ramp = std::clamp((speed - aP.speedRamp0) / std::max(aP.speedRamp1 - aP.speedRamp0, 1e-4f), 0.0f, 1.0f);
            // right-hand acceleration vector (hy, -hx) * aLP: LateralAccelAmount projects it back to aLP and applies the ABP map + clamp
            lat = ln::LateralAccelAmount(pp, vx, vy, m_aLP * hy, -m_aLP * hx) * ramp;
            ln::LeanInput(pp, aS.direction, lat, x, y);
            if (!std::isfinite(x) || !std::isfinite(y))
                x = y = 0.0f;
            const float* wr2 = m_runSt.Update(aPack, m_run, x, y, dt);
            const float* ww2 = m_walkSt.Update(aPack, m_walk, x, y, dt);
            m_runOk = wr2 != nullptr; // Evaluate / Update failure = lean 0 for that layer
            m_walkOk = ww2 != nullptr;
            if (m_head >= 0)
                m_headOk = m_headSt.Update(aPack, m_head, x, y, dt) != nullptr;
            else
                m_headOk = false;
            m_tel.tri[0] = m_runSt.tri;
            m_tel.tri[1] = m_walkSt.tri;
            m_tel.tri[2] = m_headSt.tri;
        }
        else
        {
            m_runSt.Reset();
            m_walkSt.Reset();
            m_headSt.Reset();
            m_runOk = m_walkOk = m_headOk = false;
        }
        m_tel.speed = speed;
        m_tel.aLat = m_aLP;
        m_tel.lat = lat;
        m_tel.x = x;
        m_tel.y = y;
        m_tel.walkRun = m_walkRun;
        m_tel.disable = m_disable;
        m_tel.gate = m_gate;
        m_tel.headGate = m_headGate;
        m_tel.weight = Weight();
    }

    // Publishes the lean on a LOCAL pose of the overlay skeleton (same rig as the pack: bone names / parents checked by
    // PackMatchesOverlay). Body: walk / run crossfade as one additive; head on top. Returns true when anything was applied.
    bool Apply(const ln::LeanPack& aPack, const Params& aP, ov::Pose& aPose)
    {
        if (!aPack.Ok() || aPack.Skel().n != ln::kNumBones)
            return false;
        bool applied = false;
        const bool anchor = aP.anchorFeet && m_legsOk;
        ov::Vec3 prePos[ov::kMaxBones];
        ov::Quat preRot[ov::kMaxBones];
        ov::Quat toePre[2] = {};
        if (anchor)
        {
            ov::MeshPositions(aPack.Skel(), aPose, prePos, preRot);
            for (int side = 0; side < 2; ++side)
                if (m_toe[side] >= 0)
                    toePre[side] = aPose.b[m_toe[side]].r;
        }
        const float bodyAlpha = Weight() * aP.scale;
        if (bodyAlpha > ov::kZeroWeight && (m_runOk || m_walkOk))
        {
            const float a = std::clamp(m_walkRun, 0.0f, 1.0f);
            float wWalk = m_walkOk ? 1.0f - a : 0.0f, wRun = m_runOk ? a : 0.0f;
            if (m_walkOk && m_runOk)
            {
                // both layers: crossfade
            }
            else if (m_walkOk)
                wWalk = 1.0f; // only the walk set is valid: use it alone
            else
                wRun = 1.0f;
            const ov::Additive* smp[2] = {nullptr, nullptr};
            float wt[2] = {0.0f, 0.0f};
            int n = 0;
            if (wWalk > ov::kZeroWeight)
            {
                aPack.BlendAdditive(m_walk, m_walkSt.out, m_walkAdd);
                smp[n] = &m_walkAdd;
                wt[n++] = wWalk;
            }
            if (wRun > ov::kZeroWeight)
            {
                aPack.BlendAdditive(m_run, m_runSt.out, m_runAdd);
                smp[n] = &m_runAdd;
                wt[n++] = wRun;
            }
            if (n == 1)
                ov::ApplyAdditive(aPack.Skel(), aPose, *smp[0], bodyAlpha);
            else if (n == 2)
            {
                ov::BlendAdditives(smp, wt, 2, m_body);
                ov::ApplyAdditive(aPack.Skel(), aPose, m_body, bodyAlpha);
            }
            applied = applied || n > 0;
        }
        const float headAlpha = HeadWeight() * aP.headScale;
        if (headAlpha > ov::kZeroWeight && m_headOk && m_head >= 0)
        {
            aPack.BlendAdditive(m_head, m_headSt.out, m_headAdd);
            ov::ApplyAdditive(aPack.Skel(), aPose, m_headAdd, headAlpha);
            applied = true;
        }
        if (anchor && applied)
            AnchorFeet(aPack, aP, prePos, preRot, toePre, aPose);
        return applied;
    }

private:
    // Two-bone IK of both legs onto the unleaned model-space ankles (aPrePos / aPreRot = mesh space of the pose before the additives);
    // the foot keeps its unleaned model-space rotation, the toe its unleaned local rotation. A failed solve leaves that leg leaned.
    void AnchorFeet(const ln::LeanPack& aPack, const Params& aP, const ov::Vec3* aPrePos, const ov::Quat* aPreRot, const ov::Quat* aToePre,
                    ov::Pose& aPose) const
    {
        ov::Vec3 pos[ov::kMaxBones];
        ov::Quat rot[ov::kMaxBones];
        ov::MeshPositions(aPack.Skel(), aPose, pos, rot);
        for (int side = 0; side < 2; ++side)
        {
            const int up = m_up[side], shin = m_shin[side], foot = m_foot[side], toe = m_toe[side];
            const int hp = aPack.Skel().parent[up];
            const utpp::mm::BoneTransform gp{pos[hp], rot[hp]}, ga{pos[up], rot[up]}, gb{pos[shin], rot[shin]},
                gc{pos[foot], aPreRot[foot]};
            const ov::Vec3 toeP = toe >= 0 ? aPrePos[toe] : aPrePos[foot] + math::QuatRotate(aPreRot[foot], ov::Vec3{0.0f, 0.1f, 0.0f});
            const legik::TwoBoneSolution s =
                legik::SolveTwoBone(gp, ga, gb, gc, aPrePos[foot], toeP, aP.maxExtension, aP.softExtension, true, true);
            if (s.status != legik::TwoBoneStatus::kOk)
                continue;
            aPose.b[up].r = s.local[0];
            aPose.b[shin].r = s.local[1];
            aPose.b[foot].r = s.local[2];
            if (toe >= 0)
                aPose.b[toe].r = aToePre[side];
        }
    }

    int m_run = -1, m_walk = -1, m_head = -1;
    int m_up[2] = {-1, -1}, m_shin[2] = {-1, -1}, m_foot[2] = {-1, -1}, m_toe[2] = {-1, -1};
    bool m_legsOk = false;
    ln::LayerState m_runSt, m_walkSt, m_headSt;
    bool m_runOk = false, m_walkOk = false, m_headOk = false;
    bool m_haveVel = false;
    float m_pvx = 0.0f, m_pvy = 0.0f, m_aLP = 0.0f;
    float m_axHeld = 0.0f, m_ayHeld = 0.0f, m_sinceChange = 0.0f;
    float m_gate = 0.0f, m_headGate = 0.0f;
    bool m_want = false, m_headWant = false;
    float m_walkRun = 0.0f, m_disable = 0.0f;
    bool m_haveWalkRun = false, m_haveDisable = false;
    Telemetry m_tel;
    ov::Additive m_walkAdd, m_runAdd, m_body, m_headAdd;
};

// Fail-soft identity check at load: bone count, names and parents of the pack equal the overlay skeleton the pose evaluator uses.
inline bool PackMatchesOverlay(const ln::LeanPack& aPack, const ov::OverlayFile& aOv, std::string& aWhy)
{
    if (!aPack.Ok())
    {
        aWhy = "pack not loaded";
        return false;
    }
    if (!aOv.Ok() || aOv.NumBones() != ln::kNumBones || aOv.Skel().n != ln::kNumBones)
    {
        aWhy = "overlay bone count differs";
        return false;
    }
    for (int i = 0; i < ln::kNumBones; ++i)
    {
        if (aOv.BoneName(i) != aPack.BoneName(i))
        {
            aWhy = "bone name differs at " + std::to_string(i);
            return false;
        }
        if (aOv.Skel().parent[i] != aPack.Skel().parent[i])
        {
            aWhy = "bone parent differs at " + std::to_string(i);
            return false;
        }
    }
    return true;
}

// Database clip -> pack curve table by NAME with a frame count check (a mismatch leaves that clip unmuted). Db: NumClips(),
// ClipName(i), ClipFrames(i). Returns the number of mapped clips; aMismatch counts same-name clips with another frame count.
template <class GetName, class GetFrames>
inline int BuildClipMap(const ln::LeanPack& aPack, uint32_t aNumClips, GetName aName, GetFrames aFrames, std::vector<int32_t>& aMap,
                        int& aMismatch)
{
    aMap.assign(aNumClips, -1);
    aMismatch = 0;
    int mapped = 0;
    if (!aPack.Ok())
        return 0;
    for (uint32_t c = 0; c < aNumClips; ++c)
    {
        const int k = aPack.FindClip(aName(c));
        if (k < 0)
            continue;
        if (aPack.ClipAt(k).count != static_cast<int>(aFrames(c)))
        {
            ++aMismatch;
            continue;
        }
        aMap[c] = k;
        ++mapped;
    }
    return mapped;
}

} // namespace utpp::mm::leanrt

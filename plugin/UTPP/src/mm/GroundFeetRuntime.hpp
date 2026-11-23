// UTPP motion matching runtime - feet on uneven ground (mm/GroundFeet.hpp), shared state between the game thread and the animation
// threads. Header-only, lock-free, allocation-free.
//   - game thread (feed / script / setting): SetEnabled, PublishGate, ProbeFromPose (at every publish: the balls of the published
//     feet, the points the script traces under next), ProbePoint (model -> world for the script), PublishSamples (the traces);
//   - animation thread: StepAndApply (one stage update per frame, reused by every other call of the same frame; one leg solve for the
//     body height offset and the ground plan). An inactive plan returns false and the caller runs the body height stage alone, so the
//     pose is bit for bit what it was without this stage.
// Off by default (mm.groundFeet). Off: no state is stepped, no trace is asked for and the snapshot is not touched.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "BodyHeightRuntime.hpp"
#include "GameIntegration.hpp"
#include "GroundFeet.hpp"

namespace utpp::mm::groundfeet::rt
{

inline std::atomic<bool> g_on{false}; // off by default (the menu default, motionhost g_groundFeetOn)
// bit 0: the stage may act (feed gate); bit 1: a moving platform carries V; bits 8..31: reset serial
inline std::atomic<uint32_t> g_gate{0};
inline integ::SeqlockBuffer<Samples> g_samples;
inline integ::SeqlockBuffer<Plan> g_plan;           // plan of the last stage update (reused by the other calls of a frame)
inline std::atomic<float> g_bias{0.0f};              // m, calibration (Config::bias)
inline constexpr double kSameFrameSeconds = 0.0005;
inline constexpr double kSampleMaxAge = 0.25;         // s, older traces are not used

struct Stepper
{
    Stage stage;
    int64_t lastTicks = 0;
    uint32_t serial = 0;
    float bias = 0.0f;
};
inline Stepper g_stepper;
inline std::atomic_flag g_stepLock = ATOMIC_FLAG_INIT;

// diagnostics (feed dump / trace)
inline std::atomic<uint64_t> g_steps{0}, g_applied{0}, g_identity{0}, g_noSamples{0}, g_kneeLimits{0}, g_clamps{0}, g_sudden{0}, g_rejected{0};
inline std::atomic<float> g_lift[2] = {0.0f, 0.0f};  // m, ground height applied at the ball (last apply)
inline std::atomic<float> g_tilt[2] = {0.0f, 0.0f};  // rad
inline std::atomic<float> g_planeH[2] = {0.0f, 0.0f}; // m, planting plane height under the trace (model, last step)
inline std::atomic<float> g_weight[2] = {0.0f, 0.0f};
inline std::atomic<float> g_pelvis{0.0f};            // m, ground pelvis offset (last step)
inline std::atomic<float> g_lastUs{0.0f}, g_maxUs{0.0f};

inline bool Enabled() { return g_on.load(std::memory_order_relaxed); }

inline void BumpSerial() { g_gate.fetch_add(1u << 8, std::memory_order_acq_rel); }

inline void SetEnabled(bool aOn)
{
    if (g_on.exchange(aOn, std::memory_order_acq_rel) != aOn)
        BumpSerial();
}

inline void SetBias(float aBias)
{
    if (std::isfinite(aBias) && std::fabs(aBias) < 0.2f)
        g_bias.store(aBias, std::memory_order_relaxed);
}

// Game thread, every publish while the stage is on.
inline void PublishGate(bool aAllowed, bool aReset, bool aPlatform = false)
{
    const uint32_t cur = g_gate.load(std::memory_order_relaxed);
    const uint32_t serial = (cur >> 8) + (aReset ? 1u : 0u);
    g_gate.store((serial << 8) | (aAllowed ? 1u : 0u) | (aPlatform ? 2u : 0u), std::memory_order_release);
}
inline bool GateOpen() { return (g_gate.load(std::memory_order_acquire) & 3u) == 1u; }

// ------------------------------------------------------------------------------------------------ probe points (game thread only)
struct Probe
{
    bool valid = false;
    math::Vec3 ball[2];         // model space, the published pose
    float world[2][2] = {};     // the last world point handed out per foot
    bool haveWorld[2] = {false, false};
};
inline Probe g_probe;

// Game thread, at a publish: the balls of the published feet.
inline void ProbeFromPose(const float* aPose, uint32_t aNumBones)
{
    bodyheight::LegRig rig;
    PoseFrame f;
    if (!Enabled() || !bodyheight::rt::g_rig.Read(rig) || !Gather(rig, aPose, aNumBones, f))
    {
        g_probe.valid = false;
        return;
    }
    g_probe.valid = true;
    for (int s = 0; s < 2; ++s)
        g_probe.ball[s] = f.leg[s].ball;
}

// World point (x, y) to trace under for foot aSide with the capsule the pose will be drawn with (aCap, aYaw rad: world = cap + Rz(yaw)
// model). A foot that moved since the last point is led by one more step of that motion (a swinging foot; at most 0.3 m).
inline bool ProbePoint(int aSide, float aCapX, float aCapY, float aYaw, float& aOutX, float& aOutY)
{
    if (aSide < 0 || aSide > 1 || !g_probe.valid || !std::isfinite(aCapX) || !std::isfinite(aCapY) || !std::isfinite(aYaw))
        return false;
    float wx = 0.0f, wy = 0.0f;
    math::Rot2(aYaw, g_probe.ball[aSide].x, g_probe.ball[aSide].y, wx, wy);
    wx += aCapX;
    wy += aCapY;
    float lx = 0.0f, ly = 0.0f;
    if (g_probe.haveWorld[aSide])
    {
        lx = wx - g_probe.world[aSide][0];
        ly = wy - g_probe.world[aSide][1];
        const float l = std::sqrt(lx * lx + ly * ly);
        if (l < 0.01f || !std::isfinite(l))
            lx = ly = 0.0f;
        else if (l > 0.3f)
            lx *= 0.3f / l, ly *= 0.3f / l;
    }
    g_probe.world[aSide][0] = wx;
    g_probe.world[aSide][1] = wy;
    g_probe.haveWorld[aSide] = true;
    aOutX = wx + lx;
    aOutY = wy + ly;
    return true;
}

// Game thread: the traces of this frame in world space (hit points and normals) with the capsule they were taken with, stored in its
// model frame.
inline void PublishSamples(const bool (&aHit)[2], const float (&aPos)[2][3], const float (&aNormal)[2][3], float aCapX, float aCapY,
                           float aCapZ, float aYaw, int64_t aTicks)
{
    Samples s;
    s.serial = g_gate.load(std::memory_order_relaxed) >> 8;
    s.yaw = aYaw;
    s.capZ = aCapZ;
    s.ticks = aTicks;
    for (int k = 0; k < 2; ++k)
    {
        FootSample& f = s.foot[k];
        f.hit = aHit[k] ? 1u : 0u;
        math::Rot2(-aYaw, aPos[k][0] - aCapX, aPos[k][1] - aCapY, f.x, f.y);
        f.z = aPos[k][2];
        math::Rot2(-aYaw, aNormal[k][0], aNormal[k][1], f.nx, f.ny);
        f.nz = aNormal[k][2];
    }
    g_samples.Publish(s);
}

// ------------------------------------------------------------------------------------------------ animation thread
inline void StoreMax(std::atomic<float>& aMax, float aValue)
{
    float prev = aMax.load(std::memory_order_relaxed);
    while (aValue > prev && !aMax.compare_exchange_weak(prev, aValue, std::memory_order_relaxed))
    {
    }
}

// One stage update per frame and the apply on the local copy of the published pose. aDrawn: the drawn world position of the player
// (x, y valid with aHaveXY; z required). aBodyOffset: the body height offset of this frame (0 when that stage is off). True when the
// pose was changed by the combined solve; false = nothing done here (the caller applies the body height offset alone).
inline bool StepAndApply(integ::PoseSnapshot& aSnap, int64_t aNow, int64_t aTicksPerSecond, bool aHaveZ, bool aHaveXY,
                         const float (&aDrawn)[3], float aBodyOffset)
{
    if (!Enabled())
        return false;
    bodyheight::LegRig rig;
    if (!(aSnap.weight > 0.0f) || integ::SnapshotLowerScale(aSnap) < 1.0f || !bodyheight::rt::g_rig.Read(rig) || !rig.Valid() ||
        aSnap.numBones != rig.numBones)
        return false;
    Plan plan;
    bool havePlan = false;
    if (!g_stepLock.test_and_set(std::memory_order_acquire))
    {
        Stepper& s = g_stepper;
        const uint32_t gate = g_gate.load(std::memory_order_acquire);
        const float bias = g_bias.load(std::memory_order_relaxed);
        if ((gate >> 8) != s.serial || bias != s.bias)
        {
            s.serial = gate >> 8;
            s.bias = bias;
            Config c = s.stage.Cfg();
            c.bias = bias;
            s.stage.SetConfig(c);
            s.stage.Reset();
            s.lastTicks = 0;
        }
        const double dt = s.lastTicks != 0 && aTicksPerSecond > 0 ? double(aNow - s.lastTicks) / double(aTicksPerSecond) : 0.0;
        if (!aHaveZ || !std::isfinite(aDrawn[2]))
        {
            s.stage.Reset();
            s.lastTicks = 0;
            g_plan.Publish(Plan{});
        }
        else if (s.lastTicks == 0 || dt >= kSameFrameSeconds)
        {
            Samples smp;
            const bool have = g_samples.Read(smp) && smp.ticks != 0 && aTicksPerSecond > 0 &&
                              double(aNow - smp.ticks) / double(aTicksPerSecond) <= kSampleMaxAge && (smp.serial == (gate >> 8));
            if (!have)
                g_noSamples.fetch_add(1, std::memory_order_relaxed);
            const StageDiag before = s.stage.Diagnostics();
            plan = s.stage.Step(static_cast<float>(dt), have ? &smp : nullptr, (gate & 1u) != 0, (gate & 2u) != 0, aDrawn, aHaveXY,
                                aBodyOffset, rig, aSnap.pose, aSnap.numBones);
            const StageDiag& after = s.stage.Diagnostics();
            g_sudden.fetch_add(after.sudden - before.sudden, std::memory_order_relaxed);
            g_rejected.fetch_add(after.rejected - before.rejected, std::memory_order_relaxed);
            s.lastTicks = aNow;
            g_steps.fetch_add(1, std::memory_order_relaxed);
            g_plan.Publish(plan);
            havePlan = true;
            for (int k = 0; k < 2; ++k)
            {
                g_planeH[k].store(plan.foot[k].h, std::memory_order_relaxed);
                g_weight[k].store(plan.foot[k].w, std::memory_order_relaxed);
            }
            g_pelvis.store(plan.pelvis, std::memory_order_relaxed);
        }
        g_stepLock.clear(std::memory_order_release);
    }
    if (!havePlan && !g_plan.Read(plan))
        return false;
    if (!plan.active)
    {
        g_identity.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    const ApplyResult r = Apply(Config{}, rig, aSnap.pose, aSnap.numBones, plan, aBodyOffset); // the apply does not read the bias
    if (!r.applied)
    {
        g_identity.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    g_applied.fetch_add(1, std::memory_order_relaxed);
    g_kneeLimits.fetch_add(uint64_t(r.kneeLimited), std::memory_order_relaxed);
    g_clamps.fetch_add(uint64_t(r.clamped), std::memory_order_relaxed);
    for (int k = 0; k < 2; ++k)
    {
        g_lift[k].store(r.lift[k], std::memory_order_relaxed);
        g_tilt[k].store(r.tilt[k], std::memory_order_relaxed);
    }
    return true;
}

} // namespace utpp::mm::groundfeet::rt

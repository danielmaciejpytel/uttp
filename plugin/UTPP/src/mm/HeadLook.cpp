// UTPP head look runtime, see mm/HeadLookRuntime.hpp and mm/HeadLook.hpp.
// Hot path (Apply): atomics + one seqlock read + the layer on stack arrays; no allocation, no locks, no logging.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <span>

#include "GameIntegration.hpp"
#include "HeadLook.hpp"
#include "HeadLookRuntime.hpp"
#include "Overlay.hpp"

namespace utpp::headlook
{
namespace
{
using namespace utpp::mm;
namespace core = utpp::mm::headlook;

struct TargetMsg
{
    float target[3] = {0, 0, 0};
    float weight = 0.0f;
    float chest = 0.5f;
    float maxYawDeg = core::kDefaultMaxYawDeg;
    int32_t mode = 0;
    uint32_t on = 0;
    uint32_t valid = 0;
    int64_t ticks = 0;
    uint64_t seq = 0;
};
static_assert(sizeof(TargetMsg) % 4 == 0);

struct RigMsg
{
    core::Rig rig;
    uint64_t rigPtr = 0;
};
static_assert(sizeof(RigMsg) % 4 == 0);

integ::SeqlockBuffer<TargetMsg> g_target;
integ::SeqlockBuffer<RigMsg> g_rig;
uint64_t g_targetSeq = 0; // game thread
std::atomic<bool> g_rigOk{false};
std::atomic<uint32_t> g_rigFk{0};
std::atomic<bool> g_targetOn{false}; // the last SetTarget asked for a look (quick idle check of the hot path)
std::atomic<int64_t> g_gateClosedTicks{0}; // last FK call with a fresh target and a closed gate (the game owns the head)

// detour-side state (single entrant via g_busy)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
core::Layer g_layer;
int64_t g_lastStep = 0;
std::atomic<bool> g_dirty{false}; // the layer is not at rest

// last applied output (radians) for a concurrent animation job, plus diagnostics
std::atomic<float> g_outChest{0}, g_outYaw{0}, g_outPitch{0}, g_outEyeYaw{0}, g_outEyePitch{0};
std::atomic<float> g_gazeYaw{0}, g_gazePitch{0}, g_headYaw{0}, g_headPitch{0}, g_gate{0};
std::atomic<int64_t> g_outTicks{0};
std::atomic<uint32_t> g_flags{0};
std::atomic<int32_t> g_mode{0}, g_why{0};
std::atomic<uint64_t> g_calls{0}, g_applied{0}, g_busySkips{0}, g_busyReapplied{0}, g_fails{0}, g_targetCalls{0}, g_targetInvalid{0},
    g_gapResets{0}, g_chestHeld{0};

enum Why : int32_t
{
    kWhyIdle = 0,   // nothing to do (no target, at rest)
    kWhyLook = 1,   // a target is followed
    kWhyStale = 2,  // target older than kMaxTargetAge
    kWhyGate = 3,   // MM / melee gate closed
    kWhyRig = 4,    // rig not resolved / FK rig differs
    kWhySolve = 5,  // FK of the source pose failed
    kWhyNoTarget = 6,
};
const char* WhyName(int32_t aWhy)
{
    switch (aWhy)
    {
    case kWhyIdle: return "idle";
    case kWhyLook: return "look";
    case kWhyStale: return "stale";
    case kWhyGate: return "gate";
    case kWhyRig: return "rig";
    case kWhySolve: return "solve";
    case kWhyNoTarget: return "none";
    }
    return "?";
}

constexpr float kRadToDeg = 57.2957795f;

double Freq()
{
    static const double f = [] {
        LARGE_INTEGER q;
        QueryPerformanceFrequency(&q);
        return q.QuadPart > 0 ? static_cast<double>(q.QuadPart) : 1.0;
    }();
    return f;
}

int64_t Now()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

float Age(int64_t aNow, int64_t aThen)
{
    if (aThen == 0)
        return 1e9f;
    return static_cast<float>(static_cast<double>(aNow - aThen) / Freq());
}

// Caller owns g_busy.
void ResetLocked()
{
    g_layer.Reset();
    g_lastStep = 0;
    g_outChest.store(0.0f, std::memory_order_relaxed);
    g_outYaw.store(0.0f, std::memory_order_relaxed);
    g_outPitch.store(0.0f, std::memory_order_relaxed);
    g_outEyeYaw.store(0.0f, std::memory_order_relaxed);
    g_outEyePitch.store(0.0f, std::memory_order_relaxed);
    g_outTicks.store(0, std::memory_order_relaxed);
    g_dirty.store(false, std::memory_order_relaxed);
}

// The overlays whose arms do not hold a weapon: Spine3 may carry a share of the turn (arms and helpers under it follow).
bool ChestAllowedFor(int32_t aOverlay)
{
    return aOverlay == static_cast<int32_t>(utpp::mm::ov::PoseOverlay::Default) ||
           aOverlay == static_cast<int32_t>(utpp::mm::ov::PoseOverlay::Injured);
}
} // namespace

void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RigMsg m;
    m.rig = core::ResolveRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    m.rigPtr = reinterpret_cast<uint64_t>(aRig);
    g_rig.Publish(m);
    g_rigOk.store(m.rig.ok != 0, std::memory_order_release);
    g_rigFk.store(m.rig.fkCount, std::memory_order_relaxed);
}

void OnDisarmed()
{
    RigMsg m;
    g_rig.Publish(m);
    g_rigOk.store(false, std::memory_order_release);
    g_rigFk.store(0, std::memory_order_relaxed);
}

bool RigOk() { return g_rigOk.load(std::memory_order_acquire); }

bool CanAct()
{
    if (!g_rigOk.load(std::memory_order_acquire))
        return false;
    const int64_t closed = g_gateClosedTicks.load(std::memory_order_relaxed);
    return closed == 0 || Age(Now(), closed) > 0.5f;
}

bool SetTarget(bool aOn, bool aValid, const float aTargetModel[3], float aWeight, int32_t aMode, float aChest, float aMaxYawDeg)
{
    g_targetCalls.fetch_add(1, std::memory_order_relaxed);
    TargetMsg m;
    if (aOn && !(std::isfinite(aWeight) && aWeight >= 0.0f && aWeight <= 1.0f && core::ValidMode(aMode) &&
                 core::ValidParams(aMaxYawDeg, aChest)))
        return false;
    m.on = (aOn && aWeight > 0.0f) ? 1u : 0u;
    m.weight = m.on ? aWeight : 0.0f;
    m.mode = m.on ? aMode : 0;
    m.chest = aChest;
    m.maxYawDeg = aMaxYawDeg;
    if (m.on && aValid && aTargetModel && std::isfinite(aTargetModel[0]) && std::isfinite(aTargetModel[1]) &&
        std::isfinite(aTargetModel[2]))
    {
        m.valid = 1;
        m.target[0] = aTargetModel[0];
        m.target[1] = aTargetModel[1];
        m.target[2] = aTargetModel[2];
    }
    else if (m.on)
        g_targetInvalid.fetch_add(1, std::memory_order_relaxed);
    m.ticks = Now();
    m.seq = ++g_targetSeq;
    g_target.Publish(m);
    g_targetOn.store(m.on != 0, std::memory_order_release);
    return true;
}

void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, int32_t aOverlay,
           int64_t aNowTicks)
{
    // the common case (head look off, layer at rest): two relaxed loads
    if (!g_targetOn.load(std::memory_order_relaxed) && !g_dirty.load(std::memory_order_relaxed))
        return;
    g_calls.fetch_add(1, std::memory_order_relaxed);
    RigMsg rm;
    if (!aBones || !g_rigOk.load(std::memory_order_relaxed) || !g_rig.Read(rm) || !rm.rig.ok ||
        rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || rm.rig.fkCount > aCount)
    {
        g_why.store(kWhyRig, std::memory_order_relaxed);
        return;
    }
    if (g_busy.test_and_set(std::memory_order_acquire))
    {
        // another animation job owns the layer state: the same (fresh) output as its last call, no step
        g_busySkips.fetch_add(1, std::memory_order_relaxed);
        const int64_t at = g_outTicks.load(std::memory_order_relaxed);
        if (at != 0 && Age(aNowTicks, at) <= core::kMaxTargetAge)
        {
            core::Output out;
            out.chestYaw = g_outChest.load(std::memory_order_relaxed);
            out.yaw = g_outYaw.load(std::memory_order_relaxed);
            out.pitch = g_outPitch.load(std::memory_order_relaxed);
            out.eyeYaw = g_outEyeYaw.load(std::memory_order_relaxed);
            out.eyePitch = g_outEyePitch.load(std::memory_order_relaxed);
            if ((out.chestYaw != 0.0f || out.yaw != 0.0f || out.pitch != 0.0f || out.eyeYaw != 0.0f || out.eyePitch != 0.0f) &&
                core::Layer::WriteOutput(aBones, aCount, rm.rig, out))
                g_busyReapplied.fetch_add(1, std::memory_order_relaxed);
        }
        return;
    }
    TargetMsg t;
    const bool haveTarget = g_target.Read(t);
    const float gate = core::LayerGate(aMMWeight, aUpperScale, aMeleeWeight);
    const bool fresh = haveTarget && t.on != 0 && Age(aNowTicks, t.ticks) <= core::kMaxTargetAge;
    // a gap longer than the target age since the last step (the FK detour skipped this layer: early return of the snapshot apply,
    // perspective denial, MM weight 0 ...) starts from rest instead of resuming old angles
    if (g_lastStep != 0 && !(Age(aNowTicks, g_lastStep) <= core::kMaxTargetAge))
    {
        ResetLocked();
        g_gapResets.fetch_add(1, std::memory_order_relaxed);
        if (!fresh)
        {
            g_why.store(kWhyIdle, std::memory_order_relaxed);
            g_busy.clear(std::memory_order_release);
            return;
        }
    }
    core::Input in;
    in.targetValid = fresh && t.valid != 0;
    in.target = core::Vec3{t.target[0], t.target[1], t.target[2]};
    in.weight = fresh ? t.weight : 0.0f;
    in.mode = t.mode;
    in.chest = t.chest;
    in.maxYawDeg = t.maxYawDeg;
    in.gate = gate;
    in.chestAllowed = ChestAllowedFor(aOverlay);
    in.dt = g_lastStep != 0 ? Age(aNowTicks, g_lastStep) : 0.0f;
    g_lastStep = aNowTicks;
    core::Output out;
    const bool ok = g_layer.Run(aBones, aCount, rm.rig, in, out);
    if (!ok)
        g_fails.fetch_add(1, std::memory_order_relaxed);
    else if (out.wrote)
        g_applied.fetch_add(1, std::memory_order_relaxed);
    g_outChest.store(ok ? out.chestYaw : 0.0f, std::memory_order_relaxed);
    g_outYaw.store(ok ? out.yaw : 0.0f, std::memory_order_relaxed);
    g_outPitch.store(ok ? out.pitch : 0.0f, std::memory_order_relaxed);
    g_outEyeYaw.store(ok ? out.eyeYaw : 0.0f, std::memory_order_relaxed);
    g_outEyePitch.store(ok ? out.eyePitch : 0.0f, std::memory_order_relaxed);
    g_outTicks.store(aNowTicks, std::memory_order_relaxed);
    g_gazeYaw.store(out.gazeYaw * kRadToDeg, std::memory_order_relaxed);
    g_gazePitch.store(out.gazePitch * kRadToDeg, std::memory_order_relaxed);
    g_headYaw.store(out.headYaw * kRadToDeg, std::memory_order_relaxed);
    g_headPitch.store(out.headPitch * kRadToDeg, std::memory_order_relaxed);
    g_gate.store(gate, std::memory_order_relaxed);
    if (fresh && !(gate > 0.0f))
        g_gateClosedTicks.store(aNowTicks, std::memory_order_relaxed);
    g_flags.store(out.flags, std::memory_order_relaxed);
    g_mode.store(out.mode, std::memory_order_relaxed);
    if (!in.chestAllowed && out.wrote)
        g_chestHeld.fetch_add(1, std::memory_order_relaxed);
    g_why.store(!ok ? kWhySolve : (!fresh ? (haveTarget && t.on != 0 ? kWhyStale : kWhyNoTarget) : (gate > 0.0f ? kWhyLook : kWhyGate)),
                std::memory_order_relaxed);
    g_dirty.store(!g_layer.Idle(), std::memory_order_relaxed);
    // The step clock restarts only when the layer is at rest AND nothing is wanted. With a wanted target the springs start from rest:
    // dropping the clock here made every step dt = 0, so they never left rest (first in-game run 2026-11-13: gaze -97 deg, applied 0).
    if (g_layer.Idle() && !(in.targetValid && in.weight > 0.0f && gate > 0.0f))
        g_lastStep = 0;
    g_busy.clear(std::memory_order_release);
}

std::string Status()
{
    const int64_t now = Now();
    TargetMsg t;
    const bool haveTarget = g_target.Read(t);
    char buf[800];
    std::snprintf(buf, sizeof(buf),
                  "headlook rig=%d fk=%u on=%d valid=%d mode=%d w=%.2f chest=%.2f maxYaw=%.0f age=%.3f targetCalls=%llu targetInvalid=%llu "
                  "why=%s gate=%.2f gaze=%.1f/%.1f head=%.1f/%.1f applied chest=%.1f yaw=%.1f pitch=%.1f eyes=%.1f/%.1f flags=0x%X "
                  "calls=%llu appliedCalls=%llu busy=%llu reapplied=%llu fails=%llu gapResets=%llu chestHeld=%llu",
                  g_rigOk.load() ? 1 : 0, g_rigFk.load(), haveTarget && t.on ? 1 : 0, haveTarget && t.valid ? 1 : 0,
                  haveTarget ? t.mode : 0, static_cast<double>(haveTarget ? t.weight : 0.0f),
                  static_cast<double>(haveTarget ? t.chest : 0.0f), static_cast<double>(haveTarget ? t.maxYawDeg : 0.0f),
                  static_cast<double>(haveTarget ? Age(now, t.ticks) : -1.0f), static_cast<unsigned long long>(g_targetCalls.load()),
                  static_cast<unsigned long long>(g_targetInvalid.load()), WhyName(g_why.load()), static_cast<double>(g_gate.load()),
                  static_cast<double>(g_gazeYaw.load()), static_cast<double>(g_gazePitch.load()),
                  static_cast<double>(g_headYaw.load()), static_cast<double>(g_headPitch.load()),
                  static_cast<double>(g_outChest.load() * kRadToDeg), static_cast<double>(g_outYaw.load() * kRadToDeg),
                  static_cast<double>(g_outPitch.load() * kRadToDeg), static_cast<double>(g_outEyeYaw.load() * kRadToDeg),
                  static_cast<double>(g_outEyePitch.load() * kRadToDeg), static_cast<unsigned>(g_flags.load()),
                  static_cast<unsigned long long>(g_calls.load()), static_cast<unsigned long long>(g_applied.load()),
                  static_cast<unsigned long long>(g_busySkips.load()), static_cast<unsigned long long>(g_busyReapplied.load()),
                  static_cast<unsigned long long>(g_fails.load()), static_cast<unsigned long long>(g_gapResets.load()),
                  static_cast<unsigned long long>(g_chestHeld.load()));
    return buf;
}
} // namespace utpp::headlook

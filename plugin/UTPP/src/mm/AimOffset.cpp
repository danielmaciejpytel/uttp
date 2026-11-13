// UTPP 0.6.3 aim offset runtime, see mm/AimOffsetRuntime.hpp and mm/AimOffset.hpp.
// Hot path (Apply): atomics + two seqlock reads + the solver on stack arrays; no allocation, no locks, no logging.

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

#include "AimOffset.hpp"
#include "AimOffsetRuntime.hpp"
#include "GameIntegration.hpp"

namespace utpp::aimofs
{
namespace
{
using namespace utpp::mm;

struct TargetMsg
{
    float target[3] = {0, 0, 0};
    uint32_t enabled = 0;
    uint32_t valid = 0;
    uint32_t pad = 0;
    int64_t ticks = 0;
    uint64_t seq = 0;
};
static_assert(sizeof(TargetMsg) % 4 == 0);

struct RigMsg
{
    aim::Rig rig;
    uint64_t rigPtr = 0;
};
static_assert(sizeof(RigMsg) % 4 == 0);

integ::SeqlockBuffer<TargetMsg> g_target;
integ::SeqlockBuffer<RigMsg> g_rig;
uint64_t g_targetSeq = 0; // game thread

std::atomic<float> g_maxYaw{30.0f}, g_maxPitch{25.0f}, g_smoothHz{12.0f};
std::atomic<int32_t> g_axis{aim::kDefaultBarrelAxis};
std::atomic<int32_t> g_maskStates{int32_t(integ::kMaskStatesAll)};
std::atomic<int64_t> g_feedTicks{0};
std::atomic<uint32_t> g_feedPsm{0};
std::atomic<bool> g_feedTpp{false};
std::atomic<int64_t> g_meleeTicks{0}; // last feed with melee bits (0 = never)
std::atomic<bool> g_rigOk{false};
std::atomic<uint32_t> g_rigFk{0};

// detour-side state (single entrant via g_busy)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
aim::Controller g_ctrl;
int64_t g_lastApply = 0;

// diagnostics
std::atomic<uint64_t> g_calls{0}, g_applied{0}, g_skipped{0}, g_resets{0}, g_targetCalls{0}, g_targetInvalid{0}, g_busySkips{0};
std::atomic<uint32_t> g_lastGuard{aim::kGuardNoTarget};
std::atomic<float> g_errYaw{0}, g_errPitch{0}, g_cmdYaw{0}, g_cmdPitch{0}, g_resYaw{0}, g_resPitch{0}, g_errBefore{0},
    g_errAfter{0}, g_lhErr{0}, g_weight{0};
std::atomic<uint32_t> g_twoHand{0}, g_ikOk{0};

int64_t Now()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

double Freq()
{
    static const double f = [] {
        LARGE_INTEGER q;
        QueryPerformanceFrequency(&q);
        return q.QuadPart > 0 ? static_cast<double>(q.QuadPart) : 1.0;
    }();
    return f;
}

float Age(int64_t aNow, int64_t aThen)
{
    if (aThen == 0)
        return 1e9f;
    return static_cast<float>(static_cast<double>(aNow - aThen) / Freq());
}

constexpr float kRadToDeg = 57.2957795f;
} // namespace

void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RigMsg m;
    m.rig = aim::ResolveRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
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

void OnFeed(bool aTpp, uint32_t aPsmBlockMask)
{
    const int64_t now = Now();
    g_feedTpp.store(aTpp, std::memory_order_relaxed);
    g_feedPsm.store(aPsmBlockMask, std::memory_order_relaxed);
    if (integ::PsmMeleeBits(aPsmBlockMask) != 0)
        g_meleeTicks.store(now, std::memory_order_relaxed);
    g_feedTicks.store(now, std::memory_order_release);
}

void SetMaskStates(int32_t aBits)
{
    g_maskStates.store(aBits, std::memory_order_relaxed);
}

void SetTarget(bool aEnabled, bool aValid, const float aTargetModel[3])
{
    g_targetCalls.fetch_add(1, std::memory_order_relaxed);
    TargetMsg m;
    m.enabled = aEnabled ? 1u : 0u;
    m.valid = 0;
    if (aEnabled && aValid && aTargetModel && std::isfinite(aTargetModel[0]) && std::isfinite(aTargetModel[1]) &&
        std::isfinite(aTargetModel[2]))
    {
        m.valid = 1;
        m.target[0] = aTargetModel[0];
        m.target[1] = aTargetModel[1];
        m.target[2] = aTargetModel[2];
    }
    else if (aEnabled)
        g_targetInvalid.fetch_add(1, std::memory_order_relaxed);
    m.ticks = Now();
    m.seq = ++g_targetSeq;
    g_target.Publish(m);
}

float TargetDistance()
{
    return aim::Params{}.targetDistance;
}

bool SetParams(float aMaxYawDeg, float aMaxPitchDeg, float aSmoothHz)
{
    if (!aim::ValidParams(aMaxYawDeg, aMaxPitchDeg, aSmoothHz))
        return false;
    g_maxYaw.store(aMaxYawDeg, std::memory_order_relaxed);
    g_maxPitch.store(aMaxPitchDeg, std::memory_order_relaxed);
    g_smoothHz.store(aSmoothHz, std::memory_order_relaxed);
    return true;
}

bool SetAxis(int32_t aAxis)
{
    if (!aim::ValidAxis(aAxis))
        return false;
    g_axis.store(aAxis, std::memory_order_relaxed);
    return true;
}

void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, int64_t aNowTicks)
{
    g_calls.fetch_add(1, std::memory_order_relaxed);
    if (g_busy.test_and_set(std::memory_order_acquire))
    {
        g_busySkips.fetch_add(1, std::memory_order_relaxed);
        g_lastGuard.store(aim::kGuardBusy, std::memory_order_relaxed);
        return;
    }
    TargetMsg t;
    const bool haveTarget = g_target.Read(t);
    RigMsg rm;
    const bool haveRig = g_rig.Read(rm);

    aim::GuardInput gi;
    gi.targetEnabled = haveTarget && t.enabled != 0;
    gi.targetValid = haveTarget && t.valid != 0;
    gi.targetAge = haveTarget ? Age(aNowTicks, t.ticks) : 1e9f;
    gi.feedAge = Age(aNowTicks, g_feedTicks.load(std::memory_order_acquire));
    gi.tpp = g_feedTpp.load(std::memory_order_relaxed);
    gi.psm = g_feedPsm.load(std::memory_order_relaxed);
    gi.maskStates = g_maskStates.load(std::memory_order_relaxed);
    const int64_t melee = g_meleeTicks.load(std::memory_order_relaxed);
    gi.sinceMelee = melee != 0 ? Age(aNowTicks, melee) : -1.0f;
    gi.upperScale = aUpperScale;
    gi.mmWeight = aMMWeight;
    gi.rigOk = haveRig && rm.rig.ok != 0 && rm.rigPtr == reinterpret_cast<uint64_t>(aRig) && rm.rig.fkCount <= aCount && aBones;
    uint32_t guard = aim::ComputeGuard(gi);
    aim::Params p;
    p.maxYawDeg = g_maxYaw.load(std::memory_order_relaxed);
    p.maxPitchDeg = g_maxPitch.load(std::memory_order_relaxed);
    p.smoothHz = g_smoothHz.load(std::memory_order_relaxed);
    p.barrelAxis = g_axis.load(std::memory_order_relaxed);
    if (p.maxYawDeg == 0.0f && p.maxPitchDeg == 0.0f)
        guard |= aim::kGuardDisabled;

    if (guard != 0)
    {
        if (g_ctrl.Has())
            g_resets.fetch_add(1, std::memory_order_relaxed);
        g_ctrl.Reset();
        g_lastApply = 0;
        g_skipped.fetch_add(1, std::memory_order_relaxed);
        g_lastGuard.store(guard, std::memory_order_relaxed);
        g_busy.clear(std::memory_order_release);
        return;
    }
    const float dt = g_lastApply != 0 ? Age(aNowTicks, g_lastApply) : 0.0f;
    if (dt > aim::kMaxTargetAge)
        g_ctrl.Reset(); // long gap (pause, loading): start again from 0
    g_lastApply = aNowTicks;
    const float weight = aim::GamePoseShare(aMMWeight, aUpperScale);
    aim::Result r;
    const mm::math::Vec3 target{t.target[0], t.target[1], t.target[2]};
    if (!g_ctrl.Apply(aBones, aCount, rm.rig, p, target, weight, dt, r))
    {
        g_lastApply = 0;
        g_skipped.fetch_add(1, std::memory_order_relaxed);
        g_lastGuard.store(aim::kGuardSolve, std::memory_order_relaxed);
        g_busy.clear(std::memory_order_release);
        return;
    }
    g_applied.fetch_add(1, std::memory_order_relaxed);
    g_lastGuard.store(0, std::memory_order_relaxed);
    g_errYaw.store(r.errYaw * kRadToDeg, std::memory_order_relaxed);
    g_errPitch.store(r.errPitch * kRadToDeg, std::memory_order_relaxed);
    g_cmdYaw.store(r.cmdYaw * kRadToDeg, std::memory_order_relaxed);
    g_cmdPitch.store(r.cmdPitch * kRadToDeg, std::memory_order_relaxed);
    g_resYaw.store(r.resYaw * kRadToDeg, std::memory_order_relaxed);
    g_resPitch.store(r.resPitch * kRadToDeg, std::memory_order_relaxed);
    g_errBefore.store(r.errBefore * kRadToDeg, std::memory_order_relaxed);
    g_errAfter.store(r.errAfter * kRadToDeg, std::memory_order_relaxed);
    g_lhErr.store(r.leftHandErr, std::memory_order_relaxed);
    g_weight.store(weight, std::memory_order_relaxed);
    g_twoHand.store(r.twoHanded ? 1u : 0u, std::memory_order_relaxed);
    g_ikOk.store(r.leftIkOk ? 1u : 0u, std::memory_order_relaxed);
    g_busy.clear(std::memory_order_release);
}

std::string Status()
{
    const int64_t now = Now();
    TargetMsg t;
    const bool haveTarget = g_target.Read(t);
    const int32_t axis = g_axis.load(std::memory_order_relaxed);
    const char axisName[] = {axis < 0 ? '-' : '+', static_cast<char>('X' + ((axis < 0 ? -axis : axis) - 1)), 0};
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "aimofs rig=%d fk=%u axis=%s maxYaw=%.1f maxPitch=%.1f smoothHz=%.1f target=%d valid=%d age=%.3f "
                  "targetCalls=%llu targetInvalid=%llu feedAge=%.3f tpp=%d upperBody=%d maskStates=0x%X guard=0x%X calls=%llu "
                  "applied=%llu skipped=%llu resets=%llu busy=%llu err=%.2f/%.2f cmd=%.2f/%.2f res=%.2f/%.2f errBefore=%.2f "
                  "errAfter=%.2f w=%.2f twoHand=%u ik=%u lhErr=%.4f",
                  g_rigOk.load() ? 1 : 0, g_rigFk.load(), axisName, static_cast<double>(g_maxYaw.load()),
                  static_cast<double>(g_maxPitch.load()), static_cast<double>(g_smoothHz.load()),
                  haveTarget && t.enabled ? 1 : 0, haveTarget && t.valid ? 1 : 0,
                  static_cast<double>(haveTarget ? Age(now, t.ticks) : -1.0f),
                  static_cast<unsigned long long>(g_targetCalls.load()), static_cast<unsigned long long>(g_targetInvalid.load()),
                  static_cast<double>(Age(now, g_feedTicks.load())), g_feedTpp.load() ? 1 : 0,
                  integ::PsmUpperBodyOf(g_feedPsm.load()), static_cast<unsigned>(g_maskStates.load()),
                  static_cast<unsigned>(g_lastGuard.load()), static_cast<unsigned long long>(g_calls.load()),
                  static_cast<unsigned long long>(g_applied.load()), static_cast<unsigned long long>(g_skipped.load()),
                  static_cast<unsigned long long>(g_resets.load()), static_cast<unsigned long long>(g_busySkips.load()),
                  static_cast<double>(g_errYaw.load()), static_cast<double>(g_errPitch.load()),
                  static_cast<double>(g_cmdYaw.load()), static_cast<double>(g_cmdPitch.load()),
                  static_cast<double>(g_resYaw.load()), static_cast<double>(g_resPitch.load()),
                  static_cast<double>(g_errBefore.load()), static_cast<double>(g_errAfter.load()),
                  static_cast<double>(g_weight.load()), g_twoHand.load(), g_ikOk.load(), static_cast<double>(g_lhErr.load()));
    return buf;
}
} // namespace utpp::aimofs

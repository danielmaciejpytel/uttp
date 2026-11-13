// UTPP melee aim tilt runtime, see mm/MeleeAimRuntime.hpp and mm/MeleeAim.hpp.
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

#include "GameIntegration.hpp"
#include "MeleeAim.hpp"
#include "MeleeAimRuntime.hpp"
#include "MeleeBaseRuntime.hpp"

namespace utpp::meleeaim
{
namespace
{
using namespace utpp::mm;
namespace core = utpp::mm::meleeaim;

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
    core::Rig rig;
    uint64_t rigPtr = 0;
};
static_assert(sizeof(RigMsg) % 4 == 0);

integ::SeqlockBuffer<TargetMsg> g_target;
integ::SeqlockBuffer<TargetMsg> g_lock; // the game's melee target (feet, model space), UTPPSetMeleeAimTarget
integ::SeqlockBuffer<RigMsg> g_rig;
uint64_t g_targetSeq = 0; // game thread
uint64_t g_lockSeq = 0;   // game thread

std::atomic<float> g_maxUp{0.0f}, g_maxDown{0.0f}, g_smoothHz{core::kDefaultSmoothHz}; // DLL default: off
std::atomic<bool> g_rigOk{false};
std::atomic<uint32_t> g_rigFk{0};

// detour-side state (single entrant via g_busy)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
core::Controller g_ctrl;
bool g_hitLatch = false; // hysteresis of the hit <-> cam source switch (core::SelectTarget), cleared with the controller
int64_t g_lastStep = 0;
std::atomic<bool> g_dirty{false}; // the controller ran since the last reset

enum Source : int32_t
{
    kSrcIdle = 0,    // no melee weight
    kSrcCam = 1,     // fresh camera / crosshair aim point
    kSrcStale = 2,   // aim point older than kMaxTargetAge
    kSrcNone = 3,    // no aim point (never sent / disabled)
    kSrcInvalid = 4, // player transform unavailable
    kSrcNear = 5,    // aim point too close to the chest / non-finite
    kSrcRig = 6,     // rig not resolved / FK rig differs
    kSrcOff = 7,     // params 0 / 0
    kSrcSolve = 8,   // FK of the source pose failed
    kSrcLock = 9,    // the game's melee target (round 3)
    kSrcHit = 10,    // camera target point within melee reach (crosshair hit)
};
const char* SourceName(int32_t aSrc)
{
    switch (aSrc)
    {
    case kSrcIdle: return "idle";
    case kSrcCam: return "cam";
    case kSrcStale: return "stale";
    case kSrcNone: return "none";
    case kSrcInvalid: return "invalid";
    case kSrcNear: return "near";
    case kSrcRig: return "rig";
    case kSrcOff: return "off";
    case kSrcSolve: return "solve";
    case kSrcLock: return "lock";
    case kSrcHit: return "hit";
    }
    return "?";
}

// diagnostics (degrees) + the last applied angles (radians) for a concurrent animation job
std::atomic<float> g_appPitch{0}, g_appYaw{0}, g_tgtPitch{0}, g_tgtYaw{0}, g_rawPitch{0}, g_rawYaw{0}, g_weight{0};
std::atomic<uint32_t> g_flags{0};
std::atomic<int32_t> g_src{kSrcIdle};
std::atomic<int64_t> g_appTicks{0};
std::atomic<uint64_t> g_calls{0}, g_applied{0}, g_busySkips{0}, g_busyReapplied{0}, g_solveFails{0}, g_targetCalls{0},
    g_targetInvalid{0}, g_lockCalls{0}, g_lockValid{0}, g_gapResets{0}, g_offFades{0};
uint64_t g_loggedCalls = 0; // game thread (FormatLogLine)

constexpr float kRadToDeg = 57.2957795f;

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

// Caller owns g_busy.
void ResetLocked()
{
    g_ctrl.Reset();
    g_hitLatch = false;
    g_lastStep = 0;
    g_appPitch.store(0.0f, std::memory_order_relaxed);
    g_appYaw.store(0.0f, std::memory_order_relaxed);
    g_weight.store(0.0f, std::memory_order_relaxed);
    g_appTicks.store(0, std::memory_order_relaxed);
    g_dirty.store(false, std::memory_order_relaxed);
}

void ResetIfDirty(int32_t aSrc)
{
    if (g_dirty.load(std::memory_order_relaxed) && !g_busy.test_and_set(std::memory_order_acquire))
    {
        ResetLocked();
        g_src.store(aSrc, std::memory_order_relaxed);
        g_busy.clear(std::memory_order_release);
    }
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

void SetTarget(bool aEnabled, bool aValid, const float aTargetModel[3])
{
    g_targetCalls.fetch_add(1, std::memory_order_relaxed);
    TargetMsg m;
    m.enabled = aEnabled ? 1u : 0u;
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

void SetLockTarget(bool aValid, const float aFeetModel[3])
{
    g_lockCalls.fetch_add(1, std::memory_order_relaxed);
    TargetMsg m;
    m.enabled = 1;
    if (aValid && aFeetModel && std::isfinite(aFeetModel[0]) && std::isfinite(aFeetModel[1]) && std::isfinite(aFeetModel[2]))
    {
        m.valid = 1;
        m.target[0] = aFeetModel[0];
        m.target[1] = aFeetModel[1];
        m.target[2] = aFeetModel[2];
        g_lockValid.fetch_add(1, std::memory_order_relaxed);
    }
    m.ticks = Now();
    m.seq = ++g_lockSeq;
    g_lock.Publish(m);
}

bool SetParams(float aMaxUpDeg, float aMaxDownDeg, float aSmoothHz)
{
    if (!core::ValidParams(aMaxUpDeg, aMaxDownDeg, aSmoothHz))
        return false;
    g_maxUp.store(aMaxUpDeg, std::memory_order_relaxed);
    g_maxDown.store(aMaxDownDeg, std::memory_order_relaxed);
    g_smoothHz.store(aSmoothHz, std::memory_order_relaxed);
    return true;
}

bool Active()
{
    return g_appPitch.load(std::memory_order_relaxed) != 0.0f || g_appYaw.load(std::memory_order_relaxed) != 0.0f;
}

void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, int64_t aNowTicks)
{
    const float weight = core::TiltWeight(aMeleeWeight, aMMWeight, aUpperScale);
    if (!(weight > 0.0f))
    {
        ResetIfDirty(kSrcIdle); // every non-melee frame: one relaxed load
        return;
    }
    g_calls.fetch_add(1, std::memory_order_relaxed);
    core::Params p;
    p.maxUpDeg = g_maxUp.load(std::memory_order_relaxed);
    p.maxDownDeg = g_maxDown.load(std::memory_order_relaxed);
    p.smoothHz = g_smoothHz.load(std::memory_order_relaxed);
    // "Melee aim tilt" turned off: the angles fade to level at smoothHz (no pop mid-strike); idle once level
    const bool enabled = core::Enabled(p);
    if (!enabled && !g_dirty.load(std::memory_order_relaxed))
    {
        g_src.store(kSrcOff, std::memory_order_relaxed);
        return;
    }
    RigMsg rm;
    if (!aBones || !g_rig.Read(rm) || !rm.rig.ok || rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || rm.rig.fkCount > aCount)
    {
        ResetIfDirty(kSrcRig);
        g_src.store(kSrcRig, std::memory_order_relaxed);
        return;
    }
    if (g_busy.test_and_set(std::memory_order_acquire))
    {
        // another animation job owns the smoothing state: the same (fresh) angles as its last call, no step
        g_busySkips.fetch_add(1, std::memory_order_relaxed);
        const int64_t at = g_appTicks.load(std::memory_order_relaxed);
        const float ap = g_appPitch.load(std::memory_order_relaxed), ay = g_appYaw.load(std::memory_order_relaxed);
        if (at != 0 && Age(aNowTicks, at) <= core::kMaxTargetAge && (ap != 0.0f || ay != 0.0f) &&
            core::ApplyTilt(aBones, aCount, rm.rig, ap, ay))
            g_busyReapplied.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    TargetMsg t, lk;
    const bool haveTarget = g_target.Read(t);
    const bool haveLock = g_lock.Read(lk);
    const bool camFresh = haveTarget && t.enabled != 0 && t.valid != 0 && Age(aNowTicks, t.ticks) <= core::kMaxTargetAge;
    const bool lockFresh = haveLock && lk.valid != 0 && Age(aNowTicks, lk.ticks) <= core::kMaxLockAge;
    core::Target tg;
    int32_t src = kSrcCam;
    if (!enabled)
        src = kSrcOff; // target 0: fade out
    else if (!camFresh && !lockFresh)
        src = (!haveTarget || t.enabled == 0) ? kSrcNone : (t.valid == 0 ? kSrcInvalid : kSrcStale);
    else
    {
        aim::Xf model[aim::kMaxFkBones];
        if (!aim::ComputeModel(aBones, aCount, rm.rig.parents, rm.rig.fkCount, model))
            src = kSrcSolve;
        else
        {
            // round 3: the game's melee target, else the crosshair hit within reach, else the camera ray (neutral pitch removed)
            core::AimInput ai;
            ai.lockValid = lockFresh;
            ai.lockFeet = core::Vec3{lk.target[0], lk.target[1], lk.target[2]};
            ai.camValid = camFresh;
            ai.camPoint = core::Vec3{t.target[0], t.target[1], t.target[2]};
            core::TargetSource ts = core::kTgtNone;
            tg = core::SelectTarget(model[rm.rig.Chest()].p, ai, p, &ts, &g_hitLatch);
            src = ts == core::kTgtLock ? kSrcLock : ts == core::kTgtHit ? kSrcHit : ts == core::kTgtCam ? kSrcCam : kSrcNear;
        }
    }
    if (src != kSrcCam && src != kSrcNear && src != kSrcLock && src != kSrcHit)
        tg.flags |= core::kFlagNoTarget;
    // a gap longer than the target age since the last step (the FK detour skipped this layer: early return of the snapshot
    // apply, perspective denial, MM weight 0 ...) starts from level instead of resuming old angles
    if (g_lastStep != 0 && !(Age(aNowTicks, g_lastStep) <= core::kMaxTargetAge))
    {
        g_ctrl.Reset();
        g_hitLatch = false;
        g_lastStep = 0;
        g_gapResets.fetch_add(1, std::memory_order_relaxed);
    }
    const float dt = g_lastStep != 0 ? Age(aNowTicks, g_lastStep) : 0.0f;
    g_lastStep = aNowTicks;
    g_ctrl.Step(tg.pitch, tg.yaw, tg.valid, dt, p.smoothHz);
    const float ap = g_ctrl.Pitch() * weight, ay = g_ctrl.Yaw() * weight;
    bool capped = false, ok = true;
    if (ap != 0.0f || ay != 0.0f)
        ok = core::ApplyTilt(aBones, aCount, rm.rig, ap, ay, &capped);
    if (!ok)
        g_solveFails.fetch_add(1, std::memory_order_relaxed);
    else if (ap != 0.0f || ay != 0.0f)
        g_applied.fetch_add(1, std::memory_order_relaxed);
    g_appPitch.store(ok ? ap : 0.0f, std::memory_order_relaxed);
    g_appYaw.store(ok ? ay : 0.0f, std::memory_order_relaxed);
    g_appTicks.store(aNowTicks, std::memory_order_relaxed);
    g_tgtPitch.store(tg.pitch * kRadToDeg, std::memory_order_relaxed);
    g_tgtYaw.store(tg.yaw * kRadToDeg, std::memory_order_relaxed);
    g_rawPitch.store(tg.rawPitch * kRadToDeg, std::memory_order_relaxed);
    g_rawYaw.store(tg.rawYaw * kRadToDeg, std::memory_order_relaxed);
    g_weight.store(weight, std::memory_order_relaxed);
    g_flags.store(tg.flags | (capped ? uint32_t(core::kFlagJoint) : 0u), std::memory_order_relaxed);
    g_src.store(ok ? src : int32_t(kSrcSolve), std::memory_order_relaxed);
    g_dirty.store(true, std::memory_order_relaxed);
    if (!enabled)
    {
        g_offFades.fetch_add(1, std::memory_order_relaxed);
        if (std::fabs(g_ctrl.Pitch()) < 1e-4f && std::fabs(g_ctrl.Yaw()) < 1e-4f)
            ResetLocked(); // level: idle (no more calls until the setting is on again)
    }
    g_busy.clear(std::memory_order_release);
}

bool FormatLogLine(char* aBuf, size_t aLen)
{
    if (!aBuf || aLen == 0)
        return false;
    const uint64_t calls = g_calls.load(std::memory_order_relaxed);
    if (calls == g_loggedCalls)
        return false; // no melee-weighted FK call since the last line
    g_loggedCalls = calls;
    const int head = std::snprintf(aBuf, aLen,
                  "UTPP MELEEAIM src=%s tiltW=%.2f tgtPitch=%.1f tgtYaw=%.1f rawPitch=%.1f rawYaw=%.1f pitch=%.1f yaw=%.1f "
                  "flags=0x%X maxUp=%.0f maxDown=%.0f hz=%.0f calls=%llu applied=%llu busy=%llu reapplied=%llu fails=%llu",
                  SourceName(g_src.load(std::memory_order_relaxed)), static_cast<double>(g_weight.load()),
                  static_cast<double>(g_tgtPitch.load()), static_cast<double>(g_tgtYaw.load()),
                  static_cast<double>(g_rawPitch.load()), static_cast<double>(g_rawYaw.load()),
                  static_cast<double>(g_appPitch.load() * kRadToDeg), static_cast<double>(g_appYaw.load() * kRadToDeg),
                  static_cast<unsigned>(g_flags.load()), static_cast<double>(g_maxUp.load()), static_cast<double>(g_maxDown.load()),
                  static_cast<double>(g_smoothHz.load()), static_cast<unsigned long long>(calls),
                  static_cast<unsigned long long>(g_applied.load()), static_cast<unsigned long long>(g_busySkips.load()),
                  static_cast<unsigned long long>(g_busyReapplied.load()), static_cast<unsigned long long>(g_solveFails.load()));
    // the chest base correction (mm/MeleeBase.hpp) runs just before the tilt: its state rides on the same line
    if (head > 0 && size_t(head) < aLen)
        utpp::meleebase::AppendLog(aBuf, aLen, size_t(head));
    return true;
}

std::string Status()
{
    const int64_t now = Now();
    TargetMsg t, lk;
    const bool haveTarget = g_target.Read(t);
    const bool haveLock = g_lock.Read(lk);
    char buf[900];
    std::snprintf(buf, sizeof(buf),
                  "meleeaim rig=%d fk=%u maxUp=%.1f maxDown=%.1f smoothHz=%.1f maxYaw=%.0f target=%d valid=%d age=%.3f "
                  "targetCalls=%llu targetInvalid=%llu lock=%d lockAge=%.3f lockCalls=%llu lockValid=%llu src=%s tiltW=%.2f "
                  "tgt=%.1f/%.1f raw=%.1f/%.1f applied=%.1f/%.1f flags=0x%X calls=%llu appliedCalls=%llu busy=%llu reapplied=%llu "
                  "fails=%llu gapResets=%llu offFades=%llu",
                  g_rigOk.load() ? 1 : 0, g_rigFk.load(), static_cast<double>(g_maxUp.load()), static_cast<double>(g_maxDown.load()),
                  static_cast<double>(g_smoothHz.load()), static_cast<double>(core::kMaxYawDeg), haveTarget && t.enabled ? 1 : 0,
                  haveTarget && t.valid ? 1 : 0, static_cast<double>(haveTarget ? Age(now, t.ticks) : -1.0f),
                  static_cast<unsigned long long>(g_targetCalls.load()), static_cast<unsigned long long>(g_targetInvalid.load()),
                  haveLock && lk.valid ? 1 : 0, static_cast<double>(haveLock ? Age(now, lk.ticks) : -1.0f),
                  static_cast<unsigned long long>(g_lockCalls.load()), static_cast<unsigned long long>(g_lockValid.load()),
                  SourceName(g_src.load()), static_cast<double>(g_weight.load()), static_cast<double>(g_tgtPitch.load()),
                  static_cast<double>(g_tgtYaw.load()), static_cast<double>(g_rawPitch.load()), static_cast<double>(g_rawYaw.load()),
                  static_cast<double>(g_appPitch.load() * kRadToDeg), static_cast<double>(g_appYaw.load() * kRadToDeg),
                  static_cast<unsigned>(g_flags.load()), static_cast<unsigned long long>(g_calls.load()),
                  static_cast<unsigned long long>(g_applied.load()), static_cast<unsigned long long>(g_busySkips.load()),
                  static_cast<unsigned long long>(g_busyReapplied.load()), static_cast<unsigned long long>(g_solveFails.load()),
                  static_cast<unsigned long long>(g_gapResets.load()), static_cast<unsigned long long>(g_offFades.load()));
    return buf;
}
} // namespace utpp::meleeaim

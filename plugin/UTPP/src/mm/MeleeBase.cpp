// UTPP melee chest base correction runtime, see mm/MeleeBase.hpp. Hot path (Apply): atomics + a seqlock read + the table match on stack
// data; no allocation, no locks, no logging.

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
#include "MeleeBase.hpp"
#include "MeleeBaseRuntime.hpp"

namespace utpp::meleebase
{
namespace
{
using namespace utpp::mm;
namespace core = utpp::mm::meleebase;
using core::Vec3;

struct RigMsg
{
    core::Rig rig;
    uint64_t rigPtr = 0;
};
static_assert(sizeof(RigMsg) % 4 == 0);

integ::SeqlockBuffer<RigMsg> g_rig;
std::atomic<bool> g_rigOk{false};

// detour-side state (single entrant via g_busy)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
core::Gate g_gate;
core::Hints g_hints;
int64_t g_lastStep = 0;
float g_enableW = 0.0f;       // eased fists / standing flag (0..1)
bool g_enableFresh = true;    // the next call after a reset snaps g_enableW to the flag
std::atomic<bool> g_dirty{false};

// last applied rotation vector (radians) for a concurrent animation job, and diagnostics
std::atomic<float> g_appX{0}, g_appY{0}, g_appZ{0}, g_weight{0}, g_gateValue{0}, g_rms{0};
std::atomic<float> g_rawYaw{0}, g_rawPitch{0}, g_rawRoll{0}, g_limYaw{0}, g_limPitch{0}, g_limRoll{0};
std::atomic<int64_t> g_appTicks{0};
std::atomic<int32_t> g_lastBody{-1}, g_lastClip{-1}, g_lastFrame{-1};
std::atomic<uint32_t> g_flags{0};
std::atomic<int32_t> g_lastNb{-1};
std::atomic<float> g_lastT{0};
std::atomic<bool> g_lastMatched{false};
std::atomic<uint64_t> g_calls{0}, g_matched{0}, g_applied{0}, g_noMatch{0}, g_busySkips{0}, g_busyReapplied{0}, g_solveFails{0},
    g_limited{0}, g_lowerChecks{0}, g_lowerKeptOnMM{0}, g_noBody{0}, g_matchedBody[2] = {{0}, {0}};
std::atomic<float> g_lastEngineHipsZ{0}, g_lastMMHipsZ{0};

// Key bone rotations of the game pose, read before the MM blend (CaptureEngineKeys) and consumed by the Apply of the same FK call (same
// thread, nothing in between touches them).
struct EngineKeys
{
    uint64_t rigPtr = 0;
    uint32_t valid = 0;
    core::Quat key[core::kKeyBones];
};
thread_local EngineKeys t_engine;
// share of the melee full pose in this FK call (SetFullPoseShare; consumed by Apply)
thread_local float t_fullPoseShare = 0.0f;

// lower body guard: the latched decision (hysteresis) and its eased weight (one episode of the melee full body)
std::atomic<bool> g_keepLatch{false};
std::atomic<float> g_keepK{0.0f};
std::atomic<int64_t> g_keepTicks{0};

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
    g_gate.Reset();
    g_hints = core::Hints{};
    g_lastStep = 0;
    g_enableW = 0.0f;
    g_enableFresh = true;
    g_appX.store(0.0f, std::memory_order_relaxed);
    g_appY.store(0.0f, std::memory_order_relaxed);
    g_appZ.store(0.0f, std::memory_order_relaxed);
    g_weight.store(0.0f, std::memory_order_relaxed);
    g_gateValue.store(0.0f, std::memory_order_relaxed);
    g_appTicks.store(0, std::memory_order_relaxed);
    g_dirty.store(false, std::memory_order_relaxed);
}

void ResetIfDirty()
{
    if (g_dirty.load(std::memory_order_relaxed) && !g_busy.test_and_set(std::memory_order_acquire))
    {
        ResetLocked();
        g_busy.clear(std::memory_order_release);
    }
}

// Arm / disarm (game thread): hard reset of everything the animation threads keep (waits the few microseconds an FK call holds g_busy).
void HardReset()
{
    while (g_busy.test_and_set(std::memory_order_acquire))
    {
    }
    ResetLocked();
    g_busy.clear(std::memory_order_release);
    g_keepLatch.store(false, std::memory_order_relaxed);
    g_keepK.store(0.0f, std::memory_order_relaxed);
    g_keepTicks.store(0, std::memory_order_relaxed);
}
} // namespace

int64_t TicksPerSecond() { return static_cast<int64_t>(Freq()); }

void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RigMsg m;
    m.rig = core::ResolveRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    m.rigPtr = reinterpret_cast<uint64_t>(aRig);
    g_rig.Publish(m);
    g_rigOk.store(m.rig.ok != 0, std::memory_order_release);
    HardReset(); // new rig: hints, gate, enable ease and the guard latch start clean (the armed body persists: it is set by the script)
}

void OnDisarmed()
{
    RigMsg m;
    g_rig.Publish(m);
    g_rigOk.store(false, std::memory_order_release);
    HardReset();
}

void CaptureEngineKeys(const uint8_t* aBones, uint32_t aCount, void* aRig, float aMeleeWeight)
{
    t_engine.valid = 0;
    RigMsg rm;
    if (!aBones || !(aMeleeWeight > 0.0f) || !g_rigOk.load(std::memory_order_acquire) || !g_rig.Read(rm) || !rm.rig.ok ||
        rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || rm.rig.fkCount > aCount)
        return;
    if (!core::ReadKeys(aBones, aCount, rm.rig, t_engine.key))
        return;
    t_engine.rigPtr = reinterpret_cast<uint64_t>(aRig);
    t_engine.valid = 1;
}

bool PeekEngineKeys(void* aRig, mm::math::Quat* aOut)
{
    if (!aOut || t_engine.valid == 0 || t_engine.rigPtr != reinterpret_cast<uint64_t>(aRig))
        return false;
    for (int i = 0; i < core::kKeyBones; ++i)
        aOut[i] = t_engine.key[i];
    return true;
}

void SetFullPoseShare(float aShare) { t_fullPoseShare = std::isfinite(aShare) ? aim::Clampf(aShare, 0.0f, 1.0f) : 0.0f; }

void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, float aMeleeWeight, uint32_t aEnabled,
           int64_t aNowTicks)
{
    // the melee full pose already wrote the clip's lower body with this share: the chest correction acts on the rest only
    const float fullPoseShare = t_fullPoseShare;
    t_fullPoseShare = 0.0f;
    // the game pose's key bones of this call, when CaptureEngineKeys ran for it (taken once: a stale capture is never used twice)
    const bool haveEngineKeys = t_engine.valid != 0 && t_engine.rigPtr == reinterpret_cast<uint64_t>(aRig);
    t_engine.valid = 0;
    const float weight = meleeaim::TiltWeight(aMeleeWeight, aMMWeight, aUpperScale);
    if (!(weight > 0.0f) || (aEnabled == 0 && !g_dirty.load(std::memory_order_relaxed)))
    {
        ResetIfDirty(); // every non-melee frame: one relaxed load
        return;
    }
    g_calls.fetch_add(1, std::memory_order_relaxed);
    const int32_t body = core::ArmedBody();
    RigMsg rm;
    if (body < 0)
    {
        g_noBody.fetch_add(1, std::memory_order_relaxed); // the script has not told the body yet: no table, no correction
        return;
    }
    if (!aBones || !g_rig.Read(rm) || !rm.rig.ok || rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || rm.rig.fkCount > aCount)
    {
        ResetIfDirty();
        return;
    }
    if (g_busy.test_and_set(std::memory_order_acquire))
    {
        // another animation job owns the state: the same (fresh) rotation as its last call, no step
        g_busySkips.fetch_add(1, std::memory_order_relaxed);
        const int64_t at = g_appTicks.load(std::memory_order_relaxed);
        const Vec3 rot{g_appX.load(std::memory_order_relaxed), g_appY.load(std::memory_order_relaxed), g_appZ.load(std::memory_order_relaxed)};
        if (at != 0 && Age(aNowTicks, at) <= core::kMaxGapSeconds && core::ApplyChestRotation(aBones, aCount, rm.rig, rot))
            g_busyReapplied.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // a gap since the last step (this layer was skipped: early return of the snapshot apply, perspective denial, teleport ...) starts clean
    if (g_lastStep != 0 && !(Age(aNowTicks, g_lastStep) <= core::kMaxGapSeconds))
    {
        g_gate.Reset();
        g_hints = core::Hints{};
        g_enableFresh = true;
    }
    const float dt = g_lastStep != 0 ? Age(aNowTicks, g_lastStep) : 0.0f;
    g_lastStep = aNowTicks;
    // the fists / standing flag is a TARGET weight: eased over kEnableSeconds both ways (a flip must not remove or add a 20-30 deg rotation in one frame)
    {
        const float target = aEnabled != 0 ? 1.0f : 0.0f;
        if (g_enableFresh)
        {
            g_enableW = target;
            g_enableFresh = false;
        }
        else
        {
            const float step = (dt > core::kMaxDt ? core::kMaxDt : dt) / core::kEnableSeconds;
            g_enableW = target > g_enableW ? std::fmin(target, g_enableW + step) : std::fmax(target, g_enableW - step);
        }
    }
    const float wEff = weight * g_enableW * (1.0f - fullPoseShare);
    if (!(wEff > 0.0f) && aEnabled == 0)
    {
        // eased all the way out: nothing to apply, but the state stays (gate, smoothed rotation, row hint, dirty) so that the flag coming
        // back eases in from where it was instead of snapping a fresh 20-30 deg rotation in; the mask dropping resets it (ResetIfDirty)
        g_appX.store(0.0f, std::memory_order_relaxed); // a concurrent job must not re-apply the last rotation meanwhile
        g_appY.store(0.0f, std::memory_order_relaxed);
        g_appZ.store(0.0f, std::memory_order_relaxed);
        g_busy.clear(std::memory_order_release);
        return;
    }
    core::StepInfo si;
    const bool ok = core::CorrectChest(aBones, aCount, rm.rig, body, g_hints, g_gate, wEff, dt, &si, haveEngineKeys ? t_engine.key : nullptr);
    if (si.body >= 0)
    {
        g_rawYaw.store(si.raw.yaw * core::kRadToDeg, std::memory_order_relaxed);
        g_rawPitch.store(si.raw.pitch * core::kRadToDeg, std::memory_order_relaxed);
        g_rawRoll.store(si.raw.roll * core::kRadToDeg, std::memory_order_relaxed);
        g_limYaw.store(si.lim.yaw * core::kRadToDeg, std::memory_order_relaxed);
        g_limPitch.store(si.lim.pitch * core::kRadToDeg, std::memory_order_relaxed);
        g_limRoll.store(si.lim.roll * core::kRadToDeg, std::memory_order_relaxed);
        g_rms.store(si.rmsDeg, std::memory_order_relaxed);
        g_lastBody.store(si.body, std::memory_order_relaxed);
        g_lastClip.store(int32_t(si.clip), std::memory_order_relaxed);
        g_lastFrame.store(si.frame, std::memory_order_relaxed);
        g_lastNb.store(si.neighbour, std::memory_order_relaxed);
        g_lastT.store(si.t, std::memory_order_relaxed);
        if (si.flags != 0 && si.matched)
            g_limited.fetch_add(1, std::memory_order_relaxed);
    }
    g_lastMatched.store(si.matched, std::memory_order_relaxed);
    if (si.matched)
    {
        g_matched.fetch_add(1, std::memory_order_relaxed);
        g_matchedBody[body].fetch_add(1, std::memory_order_relaxed);
    }
    else
        g_noMatch.fetch_add(1, std::memory_order_relaxed);
    if (!ok)
        g_solveFails.fetch_add(1, std::memory_order_relaxed);
    else if (aim::Length(si.applied) > 1e-6f)
        g_applied.fetch_add(1, std::memory_order_relaxed);
    g_appX.store(si.applied.x, std::memory_order_relaxed);
    g_appY.store(si.applied.y, std::memory_order_relaxed);
    g_appZ.store(si.applied.z, std::memory_order_relaxed);
    g_appTicks.store(aNowTicks, std::memory_order_relaxed);
    g_weight.store(wEff, std::memory_order_relaxed);
    g_gateValue.store(si.gate, std::memory_order_relaxed);
    g_flags.store(si.flags, std::memory_order_relaxed);
    g_dirty.store(true, std::memory_order_relaxed);
    g_busy.clear(std::memory_order_release);
}

float LowerBodyKeepWeight(const uint8_t* aBones, uint32_t aCount, void* aRig, const mm::integ::PoseSnapshot& aSnap, int64_t aNowTicks)
{
    RigMsg rm;
    if (!aBones || !g_rig.Read(rm) || !rm.rig.ok || rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || uint32_t(rm.rig.hips) >= aCount)
        return g_keepK.load(std::memory_order_relaxed);
    const uint32_t nb = aSnap.numBones <= integ::kMaxMMBones ? aSnap.numBones : integ::kMaxMMBones;
    for (uint32_t b = aSnap.firstBone; b < nb; ++b)
    {
        if (aSnap.gameIndex[b] != rm.rig.hips)
            continue;
        const float mmZ = aSnap.pose[size_t(b) * 7 + 2];
        const float engineZ = aim::ReadLocal(aBones, uint32_t(rm.rig.hips)).p.z;
        g_lowerChecks.fetch_add(1, std::memory_order_relaxed);
        g_lastEngineHipsZ.store(engineZ, std::memory_order_relaxed);
        g_lastMMHipsZ.store(mmZ, std::memory_order_relaxed);
        const bool keep = core::KeepLowerOnMM(g_keepLatch.load(std::memory_order_relaxed), engineZ, mmZ);
        g_keepLatch.store(keep, std::memory_order_relaxed);
        const float target = keep ? 1.0f : 0.0f;
        // the decision is eased like the legs' own fade (kEnableSeconds): a flip mid-episode moves the legs over 0.2 s, not in one frame;
        // the first call of an episode snaps (the melee full body's own fade is just starting)
        const int64_t last = g_keepTicks.load(std::memory_order_relaxed);
        float k = g_keepK.load(std::memory_order_relaxed);
        const float age = Age(aNowTicks, last);
        if (last == 0 || !(age <= core::kMaxGapSeconds))
            k = target;
        else
        {
            const float step = (age > core::kMaxDt ? core::kMaxDt : age) / core::kEnableSeconds;
            k = target > k ? std::fmin(target, k + step) : std::fmax(target, k - step);
        }
        g_keepTicks.store(aNowTicks, std::memory_order_relaxed);
        g_keepK.store(k, std::memory_order_relaxed);
        if (k > 0.0f)
            g_lowerKeptOnMM.fetch_add(1, std::memory_order_relaxed);
        return k;
    }
    return g_keepK.load(std::memory_order_relaxed);
}

float LowerBodyReset()
{
    g_keepLatch.store(false, std::memory_order_relaxed);
    g_keepK.store(0.0f, std::memory_order_relaxed);
    g_keepTicks.store(0, std::memory_order_relaxed);
    return 0.0f;
}

Counters GetCounters()
{
    Counters c;
    c.calls = g_calls.load();
    c.matched = g_matched.load();
    c.matchedFemale = g_matchedBody[0].load();
    c.matchedMale = g_matchedBody[1].load();
    c.noBody = g_noBody.load();
    c.applied = g_applied.load();
    return c;
}

LastStep GetLastStep()
{
    LastStep s;
    s.body = g_lastBody.load();
    s.clip = g_lastClip.load();
    s.frame = g_lastFrame.load();
    s.neighbour = g_lastNb.load();
    s.t = g_lastT.load();
    s.rmsDeg = g_rms.load();
    s.gate = g_gateValue.load();
    s.weight = g_weight.load();
    s.applied[0] = g_appX.load();
    s.applied[1] = g_appY.load();
    s.applied[2] = g_appZ.load();
    s.raw[0] = g_rawYaw.load();
    s.raw[1] = g_rawPitch.load();
    s.raw[2] = g_rawRoll.load();
    s.lim[0] = g_limYaw.load();
    s.lim[1] = g_limPitch.load();
    s.lim[2] = g_limRoll.load();
    s.flags = g_flags.load();
    s.matched = g_lastMatched.load();
    return s;
}

size_t AppendLog(char* aBuf, size_t aLen, size_t aUsed)
{
    if (!aBuf || aUsed >= aLen)
        return 0;
    const int32_t body = g_lastBody.load(std::memory_order_relaxed), clip = g_lastClip.load(std::memory_order_relaxed);
    const core::TableData& t = body == 1 ? core::MaleTable() : core::FemaleTable();
    const char* clipName = (body >= 0 && clip >= 0 && uint32_t(clip) < t.clipCount) ? t.clipNames[clip] : "-";
    const int n = std::snprintf(
        aBuf + aUsed, aLen - aUsed,
        " | BASE body=%d clip=%s f=%d rms=%.1f gate=%.2f w=%.2f raw=%.0f/%.0f/%.0f lim=%.0f/%.0f/%.0f flags=0x%X matched=%llu none=%llu "
        "applied=%llu limited=%llu fails=%llu noBody=%llu lowerKept=%llu/%llu hipsZ=%.2f/%.2f",
        core::ArmedBody(), clipName, g_lastFrame.load(), static_cast<double>(g_rms.load()), static_cast<double>(g_gateValue.load()),
        static_cast<double>(g_weight.load()), static_cast<double>(g_rawYaw.load()), static_cast<double>(g_rawPitch.load()),
        static_cast<double>(g_rawRoll.load()), static_cast<double>(g_limYaw.load()), static_cast<double>(g_limPitch.load()),
        static_cast<double>(g_limRoll.load()), static_cast<unsigned>(g_flags.load()), static_cast<unsigned long long>(g_matched.load()),
        static_cast<unsigned long long>(g_noMatch.load()), static_cast<unsigned long long>(g_applied.load()),
        static_cast<unsigned long long>(g_limited.load()), static_cast<unsigned long long>(g_solveFails.load()),
        static_cast<unsigned long long>(g_noBody.load()), static_cast<unsigned long long>(g_lowerKeptOnMM.load()),
        static_cast<unsigned long long>(g_lowerChecks.load()), static_cast<double>(g_lastEngineHipsZ.load()),
        static_cast<double>(g_lastMMHipsZ.load()));
    return n > 0 ? size_t(n) : 0;
}

std::string Status()
{
    char buf[760];
    buf[0] = 0;
    size_t used = std::snprintf(buf, sizeof(buf), "meleebase rig=%d calls=%llu", g_rigOk.load() ? 1 : 0,
                                static_cast<unsigned long long>(g_calls.load()));
    if (used < sizeof(buf))
        AppendLog(buf, sizeof(buf), used);
    return buf;
}
} // namespace utpp::meleebase

// UTPP melee full pose runtime, see mm/MeleePose.hpp. Hot path (Apply): atomics + a seqlock read + the table match on stack data; no
// allocation, no locks, no logging.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <span>

#include "GameIntegration.hpp"
#include "MeleeBaseRuntime.hpp"
#include "MeleeBodyId.hpp"
#include "MeleePose.hpp"
#include "MeleePoseGuard.hpp"
#include "MeleePoseRuntime.hpp"

namespace utpp::meleepose
{
namespace
{
using namespace utpp::mm;
namespace core = utpp::mm::meleepose;

struct RigMsg
{
    mm::meleebase::Rig keyRig;
    core::LowerRig lower;
    uint64_t rigPtr = 0;
};

integ::SeqlockBuffer<RigMsg> g_rig;
std::atomic<bool> g_rigOk{false};
core::PoseTables g_tables[2];   // written once by Load (before any FK call), read-only afterwards
std::atomic<bool> g_loaded[2] = {{false}, {false}};
std::atomic<uint64_t> g_weapon{0};
std::atomic<bool> g_enabled{true};
const uint64_t kFistsHash = integ::CNameHash("w_melee_fists");

// detour-side state (single entrant via g_busy)
std::atomic_flag g_busy = ATOMIC_FLAG_INIT;
core::State g_state;
int32_t g_stateBody = -1;
int64_t g_lastTicks = 0;

// the last write, for a concurrent animation job of the same frame
struct LastWrite
{
    core::LowerPose pose;
    float weight = 0.0f;
    int64_t ticks = 0;
};
integ::SeqlockBuffer<LastWrite> g_last;

std::atomic<uint64_t> g_calls{0}, g_written{0}, g_held{0}, g_relocks{0}, g_noTable{0}, g_noWeapon{0}, g_busySkips{0};
std::atomic<uint64_t> g_frozen{0}, g_toBase{0};
std::atomic<int32_t> g_meleeState{-1}; // PSM melee weapon state of the last feed (-1 = not reported)
std::atomic<float> g_capsuleSpeed{0.0f}; // planar capsule speed of the last feed (m/s)

// borders (feet plant, Hips height in the fades): detour side, owned through g_busy like g_state
core::BorderState g_border;
int64_t g_borderTicks = 0;
std::atomic<bool> g_borderActive{false};
std::atomic<uint64_t> g_borderCalls{0}, g_planted{0}, g_steps{0}, g_hipsLimited{0}, g_tailCalls{0};
uint64_t g_loggedCalls = 0, g_loggedGuardCalls = 0, g_loggedBorderCalls = 0; // game thread (FormatLogLine)
std::atomic<int64_t> g_costTicks{0}, g_costMax{0};
std::atomic<int32_t> g_lastClip{-1};
std::atomic<float> g_lastWeight{0.0f}, g_lastRms{0.0f}, g_lastClock{0.0f}, g_lastHipsOffset{0.0f};
std::atomic<int32_t> g_lastBody{-1};
std::atomic<bool> g_lastMatched{false}, g_lastHeld{false};

// reference pose guard (mm/MeleePoseGuard.hpp): its own rig message, state and entrant flag
struct GuardRigMsg
{
    core::GuardRig rig;
    uint64_t rigPtr = 0;
};
integ::SeqlockBuffer<GuardRigMsg> g_guardRig;
std::atomic<bool> g_guardRigOk{false};
std::atomic_flag g_guardBusy = ATOMIC_FLAG_INIT;
core::GuardState g_guard;
std::atomic<uint64_t> g_guardCalls{0}, g_guardHeld{0}, g_guardPassed{0}, g_guardBusySkips{0};

void GuardReset()
{
    while (g_guardBusy.test_and_set(std::memory_order_acquire))
    {
    }
    g_guard.have = false;
    g_guard.held = 0;
    g_guard.ticks = 0;
    g_guardBusy.clear(std::memory_order_release);
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

// held melee weapon states: hold, charged hold, block, targeting, deflect (a held block reports deflect from its start); the clip clock
// stands there while the upper body is still
bool HoldState(int32_t aState) { return aState >= 6 && aState <= 10; }

void HardReset()
{
    while (g_busy.test_and_set(std::memory_order_acquire))
    {
    }
    g_state = core::State{};
    g_stateBody = -1;
    g_lastTicks = 0;
    g_border = core::BorderState{};
    g_borderTicks = 0;
    g_borderActive.store(false, std::memory_order_relaxed);
    g_last.Publish(LastWrite{});
    g_busy.clear(std::memory_order_release);
}

void Track(int64_t aStart)
{
    LARGE_INTEGER e;
    QueryPerformanceCounter(&e);
    const int64_t d = e.QuadPart - aStart;
    g_costTicks.fetch_add(d, std::memory_order_relaxed);
    int64_t m = g_costMax.load(std::memory_order_relaxed);
    while (d > m && !g_costMax.compare_exchange_weak(m, d, std::memory_order_relaxed))
    {
    }
}
} // namespace

void Load(const std::wstring& aDataDir)
{
    const wchar_t* names[2] = {L"utpp_melee_pose_female.bin", L"utpp_melee_pose_male.bin"};
    for (int b = 0; b < 2; ++b)
    {
        std::ifstream f(aDataDir + names[b], std::ios::binary);
        std::vector<uint8_t> bytes;
        if (f)
            bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        const bool ok = !bytes.empty() && core::ParseTables(std::move(bytes), g_tables[b]) && g_tables[b].body == uint32_t(b);
        g_loaded[b].store(ok, std::memory_order_release);
    }
}

void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    RigMsg m;
    m.keyRig = mm::meleebase::ResolveRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    m.lower = core::ResolveLowerRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    m.rigPtr = reinterpret_cast<uint64_t>(aRig);
    g_rig.Publish(m);
    g_rigOk.store(m.keyRig.ok != 0 && m.lower.ok != 0, std::memory_order_release);
    HardReset();
    GuardRigMsg g;
    g.rig = core::ResolveGuardRig(std::span<const uint64_t>(aHashes), std::span<const int16_t>(aParents));
    g.rigPtr = reinterpret_cast<uint64_t>(aRig);
    g_guardRig.Publish(g);
    g_guardRigOk.store(g.rig.ok != 0, std::memory_order_release);
    GuardReset();
}

void OnDisarmed()
{
    g_rig.Publish(RigMsg{});
    g_rigOk.store(false, std::memory_order_release);
    HardReset();
    g_guardRig.Publish(GuardRigMsg{});
    g_guardRigOk.store(false, std::memory_order_release);
    GuardReset();
}

void GuardEngine(uint8_t* aBones, uint32_t aCount, void* aRig, float aMeleeWeight, int64_t aNowTicks)
{
    const bool active = std::isfinite(aMeleeWeight) && aMeleeWeight > 0.0f && aBones && g_guardRigOk.load(std::memory_order_acquire);
    if (!active)
    {
        if (g_guard.have && !g_guardBusy.test_and_set(std::memory_order_acquire))
        {
            g_guard.have = false; // the melee reason ended: the next episode starts without a stored pose
            g_guard.held = 0;
            g_guardBusy.clear(std::memory_order_release);
        }
        return;
    }
    GuardRigMsg gm;
    if (!g_guardRig.Read(gm) || gm.rigPtr != reinterpret_cast<uint64_t>(aRig))
        return;
    if (g_guardBusy.test_and_set(std::memory_order_acquire))
    {
        g_guardBusySkips.fetch_add(1, std::memory_order_relaxed); // another animation job owns the stored pose: this call is left alone
        return;
    }
    g_guardCalls.fetch_add(1, std::memory_order_relaxed);
    const float age = g_guard.have ? Age(aNowTicks, g_guard.ticks) : 1e9f;
    const core::GuardResult r = core::GuardStep(g_guard, gm.rig, aBones, aCount, true, aNowTicks, age);
    if (r == core::GuardResult::Held)
        g_guardHeld.fetch_add(1, std::memory_order_relaxed);
    else if (r == core::GuardResult::Passed)
        g_guardPassed.fetch_add(1, std::memory_order_relaxed);
    g_guardBusy.clear(std::memory_order_release);
}

void SetWeapon(uint64_t aVariableHash) { g_weapon.store(aVariableHash, std::memory_order_relaxed); }
void SetMeleeState(int32_t aState, float aPlanarSpeed)
{
    g_meleeState.store(aState, std::memory_order_relaxed);
    g_capsuleSpeed.store(std::isfinite(aPlanarSpeed) ? aPlanarSpeed : 0.0f, std::memory_order_relaxed);
}

namespace
{
// Borders after the layer stopped writing (the incoming pose of the following calls): a planted foot steps to it or is released, the
// Hips height offset decays. Ends by itself (BorderState::active = false).
void BorderTail(uint8_t* aBones, uint32_t aCount, void* aRig, int64_t aNowTicks)
{
    if (!g_borderActive.load(std::memory_order_relaxed) || !aBones)
        return;
    if (g_busy.test_and_set(std::memory_order_acquire))
        return;
    RigMsg rm;
    if (!g_rig.Read(rm) || rm.rigPtr != reinterpret_cast<uint64_t>(aRig) || !(Age(aNowTicks, g_borderTicks) <= mm::meleebase::kMaxGapSeconds))
    {
        g_border = core::BorderState{};
        g_borderActive.store(false, std::memory_order_relaxed);
        g_busy.clear(std::memory_order_release);
        return;
    }
    const float dt = g_borderTicks != 0 ? Age(aNowTicks, g_borderTicks) : 0.0f;
    g_borderTicks = aNowTicks;
    core::BorderInfo bi;
    core::BorderStep(g_border, aBones, aCount, rm.lower, nullptr, false, 0.0f, dt, &bi,
                     g_capsuleSpeed.load(std::memory_order_relaxed) > core::kCapsuleMoveSpeed);
    g_tailCalls.fetch_add(1, std::memory_order_relaxed);
    if (bi.planted > 0)
        g_planted.fetch_add(1, std::memory_order_relaxed);
    if (bi.stepStarted)
        g_steps.fetch_add(1, std::memory_order_relaxed);
    if (bi.hipsLimited)
        g_hipsLimited.fetch_add(1, std::memory_order_relaxed);
    g_borderActive.store(g_border.active, std::memory_order_relaxed);
    g_busy.clear(std::memory_order_release);
}
} // namespace
void SetEnabled(bool aOn) { g_enabled.store(aOn, std::memory_order_relaxed); }

void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aLowerScale, int64_t aNowTicks)
{
    utpp::meleebase::SetFullPoseShare(0.0f);
    const float fade = std::isfinite(aLowerScale) ? 1.0f - aim::Clampf(aLowerScale, 0.0f, 1.0f) : 0.0f;
    if (!(fade > 0.0f) || !g_enabled.load(std::memory_order_relaxed) || !g_rigOk.load(std::memory_order_acquire) || !aBones)
    {
        if (g_lastTicks != 0 && !g_busy.test_and_set(std::memory_order_acquire))
        {
            g_state = core::State{}; // the standing gate / switch closed: the next episode starts clean
            g_lastTicks = 0;
            g_busy.clear(std::memory_order_release);
        }
        if (g_rigOk.load(std::memory_order_acquire))
            BorderTail(aBones, aCount, aRig, aNowTicks);
        return;
    }
    LARGE_INTEGER start;
    QueryPerformanceCounter(&start);
    g_calls.fetch_add(1, std::memory_order_relaxed);
    const int32_t body = mm::meleebase::ArmedBody();
    RigMsg rm;
    if (body < 0 || body > 1 || !g_loaded[body].load(std::memory_order_acquire))
    {
        g_noTable.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (!g_rig.Read(rm) || rm.rigPtr != reinterpret_cast<uint64_t>(aRig))
        return;
    for (int k = 0; k < core::kLowerBones; ++k)
        if (uint32_t(rm.lower.bone[k]) >= aCount)
            return;
    const core::PoseTables& t = g_tables[body];
    if (g_busy.test_and_set(std::memory_order_acquire))
    {
        // another animation job owns the state: the same pose as its last call (when fresh), no step
        g_busySkips.fetch_add(1, std::memory_order_relaxed);
        LastWrite lw;
        if (g_last.Read(lw) && lw.ticks != 0 && Age(aNowTicks, lw.ticks) <= 0.05f && lw.weight > 0.0f)
        {
            const float w = lw.weight * fade;
            if (core::WriteLower(aBones, aCount, rm.lower, lw.pose, w))
                utpp::meleebase::SetFullPoseShare(w);
        }
        return;
    }
    if (g_stateBody != body || (g_lastTicks != 0 && !(Age(aNowTicks, g_lastTicks) <= mm::meleebase::kMaxGapSeconds)))
    {
        g_state = core::State{};
        g_stateBody = body;
        g_lastTicks = 0;
    }
    if (g_borderTicks != 0 && !(Age(aNowTicks, g_borderTicks) <= mm::meleebase::kMaxGapSeconds))
    {
        g_border = core::BorderState{}; // a gap (no FK calls): the plants are stale
        g_borderTicks = 0;
    }
    const float dt = g_lastTicks != 0 ? Age(aNowTicks, g_lastTicks) : 0.0f;
    g_lastTicks = aNowTicks;
    uint64_t wh = g_weapon.load(std::memory_order_relaxed);
    const core::Weapon* weapon = t.Find(wh != 0 ? wh : kFistsHash);
    if (!weapon)
        g_noWeapon.fetch_add(1, std::memory_order_relaxed);
    // the key bones of the game pose of this call (read before the MM blend by the chest correction's capture)
    math::Quat key[mm::meleebase::kKeyBones];
    const bool haveKeys = utpp::meleebase::PeekEngineKeys(aRig, key) || mm::meleebase::ReadKeys(aBones, aCount, rm.keyRig, key);
    core::LowerPose pose;
    core::StepInfo si;
    const float w = core::Step(g_state, t, weapon, haveKeys ? key : nullptr, 1.0f, dt, pose, &si,
                               HoldState(g_meleeState.load(std::memory_order_relaxed)));
    // the full-body fade multiplies the write (the state keeps easing on its own timeline)
    const float wEff = w * fade;
    core::LowerModel pre;
    const bool havePre = wEff > 0.0f && !g_border.active && core::ComputeLowerModel(aBones, aCount, rm.lower, pre);
    bool wrote = false;
    if (wEff > 0.0f && core::WriteLower(aBones, aCount, rm.lower, pose, wEff))
    {
        wrote = true;
        g_written.fetch_add(1, std::memory_order_relaxed);
        utpp::meleebase::SetFullPoseShare(wEff);
    }
    // borders: feet planted / short steps and the Hips height speed limit, on the written pose (and after it, see BorderTail)
    {
        const float bdt = g_borderTicks != 0 ? Age(aNowTicks, g_borderTicks) : 0.0f;
        g_borderTicks = aNowTicks;
        core::BorderInfo bi;
        const bool border = core::BorderStep(g_border, aBones, aCount, rm.lower, havePre ? &pre : nullptr, wrote, wEff, bdt, &bi,
                                             g_capsuleSpeed.load(std::memory_order_relaxed) > core::kCapsuleMoveSpeed);
        g_lastHipsOffset.store(bi.hipsOffset, std::memory_order_relaxed);
        if (border)
        {
            g_borderCalls.fetch_add(1, std::memory_order_relaxed);
            if (bi.planted > 0)
                g_planted.fetch_add(1, std::memory_order_relaxed);
            if (bi.stepStarted)
                g_steps.fetch_add(1, std::memory_order_relaxed);
            if (bi.hipsLimited)
                g_hipsLimited.fetch_add(1, std::memory_order_relaxed);
        }
        g_borderActive.store(g_border.active, std::memory_order_relaxed);
    }
    if (si.frozen)
        g_frozen.fetch_add(1, std::memory_order_relaxed);
    if (si.toBase)
        g_toBase.fetch_add(1, std::memory_order_relaxed);
    if (si.held)
        g_held.fetch_add(1, std::memory_order_relaxed);
    if (si.relocked)
        g_relocks.fetch_add(1, std::memory_order_relaxed);
    g_lastClip.store(si.clip, std::memory_order_relaxed);
    g_lastWeight.store(wEff, std::memory_order_relaxed);
    g_lastRms.store(si.rmsDeg, std::memory_order_relaxed);
    g_lastClock.store(si.clockRow, std::memory_order_relaxed);
    g_lastBody.store(body, std::memory_order_relaxed);
    g_lastMatched.store(si.matched, std::memory_order_relaxed);
    g_lastHeld.store(si.held, std::memory_order_relaxed);
    g_last.Publish(LastWrite{pose, w, aNowTicks});
    g_busy.clear(std::memory_order_release);
    Track(start.QuadPart);
}

Counters GetCounters()
{
    Counters c;
    c.calls = g_calls.load();
    c.written = g_written.load();
    c.held = g_held.load();
    c.relocks = g_relocks.load();
    c.noTable = g_noTable.load();
    c.noWeapon = g_noWeapon.load();
    c.busy = g_busySkips.load();
    c.guardCalls = g_guardCalls.load();
    c.guardHeld = g_guardHeld.load();
    c.guardPassed = g_guardPassed.load();
    c.guardBusy = g_guardBusySkips.load();
    c.frozen = g_frozen.load();
    c.toBase = g_toBase.load();
    c.borderCalls = g_borderCalls.load();
    c.tailCalls = g_tailCalls.load();
    c.planted = g_planted.load();
    c.steps = g_steps.load();
    c.hipsLimited = g_hipsLimited.load();
    const double us = 1e6 / Freq();
    c.meanMicros = c.calls ? double(g_costTicks.load()) * us / double(c.calls) : 0.0;
    c.maxMicros = double(g_costMax.load()) * us;
    return c;
}

LastStep GetLastStep()
{
    LastStep s;
    s.body = g_lastBody.load();
    s.clip = g_lastClip.load();
    s.clockRow = g_lastClock.load();
    s.weight = g_lastWeight.load();
    s.rmsDeg = g_lastRms.load();
    s.hipsOffset = g_lastHipsOffset.load();
    s.matched = g_lastMatched.load();
    s.held = g_lastHeld.load();
    return s;
}

bool FormatLogLine(char* aBuf, size_t aLen)
{
    if (!aBuf || aLen == 0)
        return false;
    const uint64_t calls = g_calls.load(std::memory_order_relaxed), guardCalls = g_guardCalls.load(std::memory_order_relaxed),
                   borderCalls = g_borderCalls.load(std::memory_order_relaxed) + g_tailCalls.load(std::memory_order_relaxed);
    if (calls == g_loggedCalls && guardCalls == g_loggedGuardCalls && borderCalls == g_loggedBorderCalls)
        return false; // nothing ran since the last line
    g_loggedCalls = calls;
    g_loggedGuardCalls = guardCalls;
    g_loggedBorderCalls = borderCalls;
    const Counters c = GetCounters();
    const int n = std::snprintf(
        aBuf, aLen,
        "UTPP MELEEPOSE calls=%llu written=%llu held=%llu relocks=%llu frozen=%llu toBase=%llu busy=%llu noTable=%llu noWeapon=%llu "
        "border=%llu tail=%llu planted=%llu steps=%llu hipsLimited=%llu guardCalls=%llu guardReplaced=%llu guardPassed=%llu "
        "guardBusy=%llu w=%.2f state=%d",
        static_cast<unsigned long long>(c.calls), static_cast<unsigned long long>(c.written), static_cast<unsigned long long>(c.held),
        static_cast<unsigned long long>(c.relocks), static_cast<unsigned long long>(c.frozen), static_cast<unsigned long long>(c.toBase),
        static_cast<unsigned long long>(c.busy), static_cast<unsigned long long>(c.noTable), static_cast<unsigned long long>(c.noWeapon),
        static_cast<unsigned long long>(c.borderCalls), static_cast<unsigned long long>(c.tailCalls),
        static_cast<unsigned long long>(c.planted), static_cast<unsigned long long>(c.steps),
        static_cast<unsigned long long>(c.hipsLimited), static_cast<unsigned long long>(c.guardCalls),
        static_cast<unsigned long long>(c.guardHeld), static_cast<unsigned long long>(c.guardPassed),
        static_cast<unsigned long long>(c.guardBusy), static_cast<double>(g_lastWeight.load()), g_meleeState.load());
    return n > 0 && size_t(n) < aLen;
}

std::string Status()
{
    const Counters c = GetCounters();
    const int32_t body = mm::meleebase::ArmedBody();
    const int32_t clip = g_lastClip.load();
    const char* name = (body >= 0 && body <= 1 && g_loaded[body].load() && clip >= 0 && size_t(clip) < g_tables[body].clipNames.size())
        ? g_tables[body].clipNames[size_t(clip)]
        : "-";
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "meleepose on=%d tables=%d/%d rig=%d calls=%llu written=%llu held=%llu relocks=%llu noTable=%llu noWeapon=%llu busy=%llu "
                  "clip=%s w=%.2f rms=%.1f cost=%.1f/%.1fus refguard rig=%d calls=%llu held=%llu passed=%llu busy=%llu",
                  g_enabled.load() ? 1 : 0, g_loaded[0].load() ? 1 : 0, g_loaded[1].load() ? 1 : 0, g_rigOk.load() ? 1 : 0,
                  static_cast<unsigned long long>(c.calls), static_cast<unsigned long long>(c.written), static_cast<unsigned long long>(c.held),
                  static_cast<unsigned long long>(c.relocks), static_cast<unsigned long long>(c.noTable),
                  static_cast<unsigned long long>(c.noWeapon), static_cast<unsigned long long>(c.busy), name,
                  static_cast<double>(g_lastWeight.load()), static_cast<double>(g_lastRms.load()), c.meanMicros, c.maxMicros,
                  g_guardRigOk.load() ? 1 : 0, static_cast<unsigned long long>(c.guardCalls), static_cast<unsigned long long>(c.guardHeld),
                  static_cast<unsigned long long>(c.guardPassed), static_cast<unsigned long long>(c.guardBusy));
    return buf;
}
} // namespace utpp::meleepose

// UTPP vehicle exit guard of the player's TPP representation (2026-10-28, diag-20261028-c9 E2).
//
// Symptom: ~0.47 s into every vehicle exit with the UTPP third-person body the player's TPP representation is deactivated
// for ~0.1 s (TppHead slot tpp -> empty, the vehicle graph on its first-person branch for 6 frames). It follows the vehicle
// camera's active perspective turning FPP (the exit camera is FPP); no script sends the deactivation.
//
// Evidence (static analysis of the 2.50 Cyberpunk2077.exe, 2.50; RVAs):
//  1. TPPRepresentationComponent class registration (0x12A11B4) registers its event handlers; the handler for
//     gameDeactivateTPPRepresentationEvent (class slot 0x3BC64D8, registration 0x171008C) is 0x2AA7794: it unequips the
//     TPP items and sets the anim feature TPPRepresentation.IsActive = false (0x18A2EA8(comp, 0)). The feature setter is
//     reached only from that handler (false) and the Prepare handler 0x9621A0, so every drop is this event.
//  2. Event vtable 0x3649C50 (GetType 0x1952B80 -> slot 0x3BC64D8); the only code that stores it is the ctor 0x2178B70,
//     called by the RTTI construct callback 0x2AA6C60 (scripts' new) and the native factory 0x2BACF50. The factory has
//     exactly five call sites: photo mode (0x2E50088, 0x2E551A0), the vehicle cinematic camera (0x21CB190, only undoes its
//     own activation), the quest node "Player TPP Representation" (0x2CB8790) and the cold chunk 0x2542308 of
//     0xC32D84 below.
//  3. 0xC32D84 (kRvaSync): vehicle camera component sync of the representation: wanted byte +0x130, applied byte +0x131;
//     if they differ and the check 0x193A240(this) does not defer: wanted != 0 -> ActivateTPPRepresentationEvent
//     (0x192A3A8 -> ctor 0x21CC720, vtable GetType -> Activate slot 0x3BC6518), wanted == 0 -> je 0x2542308 ->
//     factory 0x2BACF50 (Deactivate); the event is queued on [this+0x60]; then applied = wanted.
//     Wanted is written by 0xC33DB8 (if changed: store, tail-jump into the sync), called from the per-frame perspective
//     decision 0xC33BBC (perspective mode from 0xC33DCC -> wanted 1 for the third-person modes, else 0 via the cold chunk
//     0x2496B1C), and zeroed by the release path 0x2BAE644 (which then calls the sync). The per-frame update 0xC32B80
//     calls the sync every frame.
// So the exit drop is the vehicle camera's wanted 1 -> 0 at the end of its TPP -> FPP switch, sent by kRvaSync.
//
// Fix: detour kRvaSync. Only while the exit window is open (SetWindow, opened by CET utpp_body only while UTPP wants the
// TPP body; hard cap kMaxWindowMs) and only where the game itself would send the deactivation now (wanted 0 != applied,
// the vanilla defer check false), the detour does what the sync does minus the event: applied = wanted. The vehicle camera
// then treats the representation as released (no pending deactivation fires after the window); a later wanted = 1
// (next mount / TPP ride camera) activates again as usual. Every other call runs the game's sync unchanged.
// Install is fail-closed: prologue, defer call, deactivation branch and factory call are checked; any mismatch = no hook.
// Hot path: relaxed atomics, no locks, no allocation, no logging; field access under SEH.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Stack.hpp>

#include "../CommonUtil.hpp"
#include "../PluginLog.hpp"
#include "TppRepGuard.hpp"
#include "TppRepGuardCore.hpp"

namespace utpp::tpprepguard
{
namespace
{
using Sync_t = void (*)(void*);
using Defer_t = bool (*)(void*);

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;

Sync_t g_origSync = nullptr;
Defer_t g_defer = nullptr;
void* g_targetSync = nullptr;

std::atomic<bool> g_installed{false};
std::atomic<bool> g_open{false};
std::atomic<uint64_t> g_openedAtMs{0};
std::atomic<uint64_t> g_windows{0};      // windows opened
std::atomic<uint64_t> g_skipped{0};      // deactivations not sent (applied = wanted)
std::atomic<uint64_t> g_deferred{0};     // wanted 0 in the window, but the game deferred (passed through)
std::atomic<uint64_t> g_lateDeact{0};    // wanted 0 != applied while the window was open but past the cap (passed)
std::atomic<uint64_t> g_activations{0};  // wanted 1 != applied in the window (passed)
std::atomic<uint64_t> g_sehFails{0};

std::mutex g_coldMutex; // cold state below (script thread / load)
std::string g_installWhy = "not installed (Install not called)";
Gate g_gate; // mirror of g_open / g_openedAtMs for GateSet (under g_coldMutex)

using pluginlog::LogInfo;
using pluginlog::LogWarn;

bool SehRead(const uint8_t* aThis, uint8_t& aWanted, uint8_t& aApplied)
{
    __try
    {
        aWanted = aThis[kSyncWantedOff];
        aApplied = aThis[kSyncAppliedOff];
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SehDefer(void* aThis, bool& aDefer)
{
    __try
    {
        aDefer = g_defer(aThis);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SehWriteApplied(uint8_t* aThis, uint8_t aValue)
{
    __try
    {
        aThis[kSyncAppliedOff] = aValue;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

void DetourSync(void* aThis)
{
    if (aThis && g_open.load(std::memory_order_acquire))
    {
        auto* self = static_cast<uint8_t*>(aThis);
        uint8_t wanted = 0, applied = 0;
        if (!SehRead(self, wanted, applied))
        {
            g_sehFails.fetch_add(1, std::memory_order_relaxed);
        }
        else if (wanted != applied)
        {
            const bool active = GateActive(true, g_openedAtMs.load(std::memory_order_relaxed), GetTickCount64());
            if (wanted != 0)
                g_activations.fetch_add(1, std::memory_order_relaxed);
            else if (!active)
                g_lateDeact.fetch_add(1, std::memory_order_relaxed);
            else
            {
                bool defer = true;
                if (!SehDefer(aThis, defer))
                    g_sehFails.fetch_add(1, std::memory_order_relaxed);
                else if (Decide(active, wanted, applied, defer) == Decision::SkipDeactivate)
                {
                    if (SehWriteApplied(self, wanted))
                    {
                        g_skipped.fetch_add(1, std::memory_order_relaxed);
                        return;
                    }
                    g_sehFails.fetch_add(1, std::memory_order_relaxed);
                }
                else
                    g_deferred.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
    g_origSync(aThis);
}

using util::SehBytesEqual;

// rel32 branch / call at aAt (opcode bytes aOpLen) -> target RVA, or 0 when unreadable / opcode differs
uint32_t SehRelTarget(const uint8_t* aBase, uint32_t aAt, const uint8_t* aOp, size_t aOpLen)
{
    __try
    {
        if (std::memcmp(aBase + aAt, aOp, aOpLen) != 0)
            return 0;
        int32_t rel = 0;
        std::memcpy(&rel, aBase + aAt + aOpLen, 4);
        return static_cast<uint32_t>(static_cast<int64_t>(aAt) + static_cast<int64_t>(aOpLen) + 4 + rel);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
}

std::string Hex(uint32_t aV)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%X", aV);
    return buf;
}

void Native_SetWindow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool live = SetWindow(on); // logs one line per window edge
    if (aOut)
        *aOut = live;
}

void Native_Status(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(Status().c_str());
}

// Empty string = all sites match.
std::string CheckSites(const uint8_t* aBase)
{
    static constexpr uint8_t kCall[] = {0xE8};
    static constexpr uint8_t kJe[] = {0x0F, 0x84};
    if (!SehBytesEqual(aBase + kRvaSync, kSyncPrologue, sizeof(kSyncPrologue)))
        return "sync prologue @0xC32D84 mismatch";
    uint32_t t = SehRelTarget(aBase, kRvaDeferCall, kCall, sizeof(kCall));
    if (t != kRvaDefer)
        return "defer call @0xC32DB7 -> " + Hex(t) + " (expected 0x193A240)";
    if (!SehBytesEqual(aBase + kRvaDefer, kDeferPrologue, sizeof(kDeferPrologue)))
        return "defer check prologue @0x193A240 mismatch";
    if (!SehBytesEqual(aBase + kRvaDeactBranch, kDeactBranch, sizeof(kDeactBranch)))
        return "deactivation branch @0xC32DC4 mismatch";
    t = SehRelTarget(aBase, kRvaDeactBranch + 10, kJe, sizeof(kJe));
    if (t != kRvaDeactChunk)
        return "deactivation branch -> " + Hex(t) + " (expected 0x2542308)";
    t = SehRelTarget(aBase, kRvaDeactChunk, kCall, sizeof(kCall));
    if (t != kRvaDeactFactory)
        return "deactivation chunk call -> " + Hex(t) + " (expected the Deactivate factory 0x2BACF50)";
    return {};
}
} // namespace

void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    std::lock_guard lock(g_coldMutex);
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !g_sdk || !g_sdk->hooking)
    {
        g_installWhy = "no image base / hooking API";
        LogWarn("UTPP tpprepguard: hook not installed (%s); vehicle exit guard inert", g_installWhy.c_str());
        return;
    }
    const std::string bad = CheckSites(base);
    if (!bad.empty())
    {
        g_installWhy = bad;
        LogWarn("UTPP tpprepguard: hook not installed (%s); vehicle exit guard inert", g_installWhy.c_str());
        return;
    }
    g_defer = reinterpret_cast<Defer_t>(base + kRvaDefer);
    uint8_t* target = base + kRvaSync;
    if (!g_sdk->hooking->Attach(g_handle, target, reinterpret_cast<void*>(&DetourSync),
                                reinterpret_cast<void**>(&g_origSync)) ||
        !g_origSync)
    {
        g_origSync = nullptr;
        g_installWhy = "hooking->Attach failed on the vehicle camera representation sync @0xC32D84";
        LogWarn("UTPP tpprepguard: %s; vehicle exit guard inert", g_installWhy.c_str());
        return;
    }
    g_targetSync = target;
    g_installed.store(true, std::memory_order_release);
    g_installWhy = "installed (vehicle camera representation sync @0xC32D84)";
    LogInfo("UTPP tpprepguard: hook attached (vehicle camera representation sync @%p); window closed until the script opens it",
            static_cast<void*>(target));
}

void Shutdown()
{
    std::lock_guard lock(g_coldMutex);
    g_open.store(false);
    g_gate = Gate{};
    if (g_sdk && g_sdk->hooking && g_targetSync)
        LogInfo("UTPP tpprepguard unload: detach %s", g_sdk->hooking->Detach(g_handle, g_targetSync) ? "ok" : "FAILED");
    g_targetSync = nullptr;
    g_installed.store(false);
}

bool SetWindow(bool aOn)
{
    const uint64_t now = GetTickCount64();
    bool opened = false;
    bool closed = false;
    {
        std::lock_guard lock(g_coldMutex);
        const bool wasOpen = g_gate.open;
        opened = GateSet(g_gate, aOn, now);
        closed = !aOn && wasOpen;
        if (opened)
        {
            g_openedAtMs.store(g_gate.openedAtMs, std::memory_order_relaxed);
            g_open.store(true, std::memory_order_release);
            g_windows.fetch_add(1, std::memory_order_relaxed);
        }
        else if (closed)
        {
            g_open.store(false, std::memory_order_release);
        }
    }
    const bool live = g_installed.load(std::memory_order_acquire);
    if (opened || closed) // one line per window edge
        LogInfo("UTPPSetVehicleExitRepGuard(%d) hook=%d skipped=%llu deferred=%llu late=%llu", aOn ? 1 : 0, live ? 1 : 0,
                static_cast<unsigned long long>(g_skipped.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(g_deferred.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(g_lateDeact.load(std::memory_order_relaxed)));
    return live;
}

void RegisterNatives()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const RED4ext::CBaseFunction::Flags flags = {.isNative = true, .isStatic = true};
    auto* f0 = RED4ext::CGlobalFunction::Create("UTPPSetVehicleExitRepGuard", "UTPPSetVehicleExitRepGuard", &Native_SetWindow);
    f0->flags = flags;
    f0->AddParam("Bool", "on");
    f0->SetReturnType("Bool");
    rtti->RegisterFunction(f0);
    auto* f1 = RED4ext::CGlobalFunction::Create("UTPPVehicleExitRepGuardStatus", "UTPPVehicleExitRepGuardStatus", &Native_Status);
    f1->flags = flags;
    f1->SetReturnType("String");
    rtti->RegisterFunction(f1);
}

bool Installed()
{
    return g_installed.load(std::memory_order_acquire);
}

std::string Status()
{
    std::string why;
    {
        std::lock_guard lock(g_coldMutex);
        why = g_installWhy;
    }
    const uint64_t now = GetTickCount64();
    const bool open = g_open.load(std::memory_order_acquire);
    const uint64_t at = g_openedAtMs.load(std::memory_order_relaxed);
    const bool active = GateActive(open, at, now);
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "tpprepguard hook=%d open=%d active=%d openFor=%.2f windows=%llu skipped=%llu deferred=%llu late=%llu "
                  "activations=%llu sehFails=%llu install=\"%s\"",
                  g_installed.load() ? 1 : 0, open ? 1 : 0, active ? 1 : 0,
                  open && now >= at ? static_cast<double>(now - at) / 1000.0 : 0.0,
                  static_cast<unsigned long long>(g_windows.load()), static_cast<unsigned long long>(g_skipped.load()),
                  static_cast<unsigned long long>(g_deferred.load()), static_cast<unsigned long long>(g_lateDeact.load()),
                  static_cast<unsigned long long>(g_activations.load()), static_cast<unsigned long long>(g_sehFails.load()),
                  why.c_str());
    return buf;
}
} // namespace utpp::tpprepguard

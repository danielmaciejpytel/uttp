// UTPPTestHost: test-only RED4ext plugin for automated test runs (test installs only, never part of a release package).
//
// Inert after every game start. The global native UTPPTestBackgroundInput(on: Bool) -> String switches background pad
// input on or off at run time; UTPPTestBackgroundInputStatus() -> String reports the counters.
//
// While on:
//   * the pad gate of the engine input update (TestHostCore.hpp) is a 2-byte nop, so the queued virtual / real pad
//     events reach the game while another window is in front;
//   * while the game window is NOT really the foreground window (the foreground window belongs to another process),
//     the game's own imports that could touch the desktop are made harmless through its import table:
//       ClipCursor(rect)      -> no-op (ClipCursor(NULL) always passes),
//       SetCursorPos          -> no-op,
//       SetCursor(NULL)       -> no-op (no hidden cursor over other windows),
//       SetForegroundWindow   -> refused (returns FALSE),
//       SetFocus / SetActiveWindow -> refused unless a mouse button is held (a click into the game window),
//       GetCursorPos          -> the last position read while the game was in front (no reaction to the user's mouse);
//   * XInputGetState is counted (calls, connected pads, input outside the dead zone; game in front / behind), and so are
//     the GetForegroundWindow calls of the pad gate itself (the result is never changed): the counters tell whether the
//     input update runs at all while the game is behind and whether the pad reports anything there.
// No window message is swallowed or rewritten (no window subclass): the focus loss handling of the game is unchanged.
// Keyboard and mouse: the game registers raw input without the background flag and does not import GetAsyncKeyState, so
// keystrokes and mouse input in other windows never reach it; nothing is hooked for that.
// With the game really in front every hook passes the call through unchanged. Switching off restores the gate bytes and
// the import table entries.
//
// Log: the plugin's own RED4ext log, every line prefixed "[bg-input]": state changes, every refusal reason, the first
// blocked calls of each hook, foreground changes and a counter summary once per second while on.

#include <Windows.h>
#include <intrin.h>

#include <RED4ext/RED4ext.hpp>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "TestHostCore.hpp"

namespace
{
namespace th = utpp::testhost;

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;

void Log(bool aWarn, const char* aFmt, ...)
{
    if (!g_sdk || !g_sdk->logger)
        return;
    char buf[1024];
    int n = std::snprintf(buf, sizeof(buf), "[bg-input] ");
    va_list ap;
    va_start(ap, aFmt);
    std::vsnprintf(buf + n, sizeof(buf) - static_cast<size_t>(n), aFmt, ap);
    va_end(ap);
    if (aWarn)
        g_sdk->logger->Warn(g_handle, buf);
    else
        g_sdk->logger->Info(g_handle, buf);
}

std::mutex g_switchMutex;              // UTPPTestBackgroundInput / unload
std::atomic<bool> g_on{false};         // read by every hook
bool g_gatePatched = false;            // g_switchMutex
uint8_t* g_gate = nullptr;             // g_switchMutex
std::atomic<uint64_t> g_onSince{0};    // GetTickCount64 at switch-on
std::atomic<uint64_t> g_nextSummary{0};
std::atomic<int> g_lastFg{-1};         // 1 = game in front, 0 = another process, -1 = unknown
std::atomic<uint64_t> g_fgChanges{0};
std::atomic<uint64_t> g_lastCursor{0}; // packed POINT (x low, y high) from the last pass-through GetCursorPos
std::atomic<bool> g_haveCursor{false};
std::atomic<uintptr_t> g_gateReturn{0}; // return address of the pad gate's GetForegroundWindow call
std::atomic<uint64_t> g_gateRunsFg{0}, g_gateRunsBg{0}, g_otherFgCalls{0};

bool ReallyForeground()
{
    HWND fg = GetForegroundWindow();
    if (!fg)
        return false;
    DWORD pid = 0;
    GetWindowThreadProcessId(fg, &pid);
    return pid == GetCurrentProcessId();
}

void NoteForeground(bool aFg)
{
    const int now = aFg ? 1 : 0;
    const int prev = g_lastFg.exchange(now);
    if (prev != now && prev != -1)
    {
        const uint64_t n = ++g_fgChanges;
        if (n <= 200)
            Log(false, "foreground -> %s (change %llu)", aFg ? "game" : "other process", static_cast<unsigned long long>(n));
    }
}

// True while switched on and the game is not really the foreground window: the desktop-touching calls are refused.
bool Blocking()
{
    if (!g_on.load(std::memory_order_relaxed))
        return false;
    const bool fg = ReallyForeground();
    NoteForeground(fg);
    return !fg;
}

bool MouseButtonDown()
{
    return (GetAsyncKeyState(VK_LBUTTON) & 0x8000) || (GetAsyncKeyState(VK_RBUTTON) & 0x8000) ||
           (GetAsyncKeyState(VK_MBUTTON) & 0x8000);
}

// ------------------------------------------------------------------------------------------------ import table hooks
enum HookId
{
    kXInputGetState,
    kClipCursor,
    kSetCursorPos,
    kSetCursor,
    kSetForegroundWindow,
    kSetFocus,
    kSetActiveWindow,
    kGetCursorPos,
    kGetForegroundWindow,
    kHookCount
};

struct Hook
{
    const char* dll;
    const char* name;
    void* detour;
    std::atomic<void*> next{nullptr};
    void** slot = nullptr;
    bool installed = false;
    std::atomic<uint64_t> calls{0};
    std::atomic<uint64_t> blocked{0};
};

Hook g_hooks[kHookCount];

template<class F> F Next(HookId aId)
{
    return reinterpret_cast<F>(g_hooks[aId].next.load(std::memory_order_acquire));
}

void CountBlocked(HookId aId)
{
    const uint64_t n = ++g_hooks[aId].blocked;
    if (n <= 5 || n == 50 || n == 500 || n % 5000 == 0)
        Log(false, "%s blocked (game not in front), %llu so far", g_hooks[aId].name, static_cast<unsigned long long>(n));
}

void MaybeSummary();

// XInput gamepad state (layout of XINPUT_STATE).
struct PadState
{
    DWORD packet;
    WORD buttons;
    BYTE leftTrigger;
    BYTE rightTrigger;
    SHORT lx, ly, rx, ry;
};

struct PadCounters
{
    std::atomic<uint64_t> calls{0}, connected{0}, active{0};
};
PadCounters g_padFg, g_padBg;
std::atomic<uint64_t> g_padFirstBgActiveLogged{0};

DWORD WINAPI Hook_XInputGetState(DWORD aUser, PadState* aState)
{
    ++g_hooks[kXInputGetState].calls;
    const DWORD r = Next<DWORD(WINAPI*)(DWORD, PadState*)>(kXInputGetState)(aUser, aState);
    if (g_on.load(std::memory_order_relaxed))
    {
        const bool fg = ReallyForeground();
        NoteForeground(fg);
        PadCounters& c = fg ? g_padFg : g_padBg;
        ++c.calls;
        if (r == ERROR_SUCCESS && aState)
        {
            ++c.connected;
            if (th::PadActive(aState->buttons, aState->leftTrigger, aState->rightTrigger, aState->lx, aState->ly, aState->rx,
                              aState->ry))
            {
                ++c.active;
                if (!fg && g_padFirstBgActiveLogged.exchange(1) == 0)
                    Log(false, "first pad input read with the game behind another window: pad %lu buttons=0x%04X lt=%u rt=%u "
                               "lx=%d ly=%d rx=%d ry=%d",
                        aUser, aState->buttons, aState->leftTrigger, aState->rightTrigger, aState->lx, aState->ly, aState->rx,
                        aState->ry);
            }
        }
        MaybeSummary();
    }
    return r;
}

BOOL WINAPI Hook_ClipCursor(const RECT* aRect)
{
    ++g_hooks[kClipCursor].calls;
    if (aRect && Blocking())
    {
        CountBlocked(kClipCursor);
        return TRUE;
    }
    return Next<BOOL(WINAPI*)(const RECT*)>(kClipCursor)(aRect);
}

BOOL WINAPI Hook_SetCursorPos(int aX, int aY)
{
    ++g_hooks[kSetCursorPos].calls;
    if (Blocking())
    {
        CountBlocked(kSetCursorPos);
        return TRUE;
    }
    return Next<BOOL(WINAPI*)(int, int)>(kSetCursorPos)(aX, aY);
}

HCURSOR WINAPI Hook_SetCursor(HCURSOR aCursor)
{
    ++g_hooks[kSetCursor].calls;
    if (!aCursor && Blocking())
    {
        CountBlocked(kSetCursor);
        return GetCursor();
    }
    return Next<HCURSOR(WINAPI*)(HCURSOR)>(kSetCursor)(aCursor);
}

BOOL WINAPI Hook_SetForegroundWindow(HWND aWnd)
{
    ++g_hooks[kSetForegroundWindow].calls;
    if (Blocking())
    {
        CountBlocked(kSetForegroundWindow);
        return FALSE;
    }
    return Next<BOOL(WINAPI*)(HWND)>(kSetForegroundWindow)(aWnd);
}

HWND WINAPI Hook_SetFocus(HWND aWnd)
{
    ++g_hooks[kSetFocus].calls;
    if (Blocking() && !MouseButtonDown())
    {
        CountBlocked(kSetFocus);
        return nullptr;
    }
    return Next<HWND(WINAPI*)(HWND)>(kSetFocus)(aWnd);
}

HWND WINAPI Hook_SetActiveWindow(HWND aWnd)
{
    ++g_hooks[kSetActiveWindow].calls;
    if (Blocking() && !MouseButtonDown())
    {
        CountBlocked(kSetActiveWindow);
        return nullptr;
    }
    return Next<HWND(WINAPI*)(HWND)>(kSetActiveWindow)(aWnd);
}

BOOL WINAPI Hook_GetCursorPos(POINT* aPoint)
{
    ++g_hooks[kGetCursorPos].calls;
    if (aPoint && g_haveCursor.load() && Blocking())
    {
        const uint64_t packed = g_lastCursor.load();
        aPoint->x = static_cast<LONG>(static_cast<int32_t>(packed & 0xFFFFFFFFull));
        aPoint->y = static_cast<LONG>(static_cast<int32_t>(packed >> 32));
        CountBlocked(kGetCursorPos);
        return TRUE;
    }
    const BOOL r = Next<BOOL(WINAPI*)(POINT*)>(kGetCursorPos)(aPoint);
    if (r && aPoint)
    {
        g_lastCursor.store(static_cast<uint32_t>(aPoint->x) | (static_cast<uint64_t>(static_cast<uint32_t>(aPoint->y)) << 32));
        g_haveCursor.store(true);
    }
    return r;
}

// Counts only; the game always gets the real foreground window.
HWND WINAPI Hook_GetForegroundWindow()
{
    ++g_hooks[kGetForegroundWindow].calls;
    const HWND r = Next<HWND(WINAPI*)()>(kGetForegroundWindow)();
    if (g_on.load(std::memory_order_relaxed))
    {
        if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == g_gateReturn.load(std::memory_order_relaxed))
        {
            DWORD pid = 0;
            if (r)
                GetWindowThreadProcessId(r, &pid);
            ++(pid == GetCurrentProcessId() ? g_gateRunsFg : g_gateRunsBg);
        }
        else
            ++g_otherFgCalls;
        MaybeSummary();
    }
    return r;
}

void InitHookTable()
{
    struct Row
    {
        HookId id;
        const char* dll;
        const char* name;
        void* detour;
    };
    const Row rows[] = {
        {kXInputGetState, "XINPUT9_1_0.dll", "XInputGetState", reinterpret_cast<void*>(&Hook_XInputGetState)},
        {kClipCursor, "USER32.dll", "ClipCursor", reinterpret_cast<void*>(&Hook_ClipCursor)},
        {kSetCursorPos, "USER32.dll", "SetCursorPos", reinterpret_cast<void*>(&Hook_SetCursorPos)},
        {kSetCursor, "USER32.dll", "SetCursor", reinterpret_cast<void*>(&Hook_SetCursor)},
        {kSetForegroundWindow, "USER32.dll", "SetForegroundWindow", reinterpret_cast<void*>(&Hook_SetForegroundWindow)},
        {kSetFocus, "USER32.dll", "SetFocus", reinterpret_cast<void*>(&Hook_SetFocus)},
        {kSetActiveWindow, "USER32.dll", "SetActiveWindow", reinterpret_cast<void*>(&Hook_SetActiveWindow)},
        {kGetCursorPos, "USER32.dll", "GetCursorPos", reinterpret_cast<void*>(&Hook_GetCursorPos)},
        {kGetForegroundWindow, "USER32.dll", "GetForegroundWindow", reinterpret_cast<void*>(&Hook_GetForegroundWindow)},
    };
    for (const Row& r : rows)
    {
        g_hooks[r.id].dll = r.dll;
        g_hooks[r.id].name = r.name;
        g_hooks[r.id].detour = r.detour;
    }
}

bool WriteSlot(void** aSlot, void* aValue)
{
    DWORD old = 0;
    if (!VirtualProtect(aSlot, sizeof(void*), PAGE_READWRITE, &old))
        return false;
    InterlockedExchangePointer(aSlot, aValue);
    DWORD tmp = 0;
    VirtualProtect(aSlot, sizeof(void*), old, &tmp);
    return true;
}

int InstallHooks(uint8_t* aBase, uint64_t aSize)
{
    int ok = 0;
    for (Hook& h : g_hooks)
    {
        h.calls = 0;
        h.blocked = 0;
        if (h.installed)
        {
            ++ok;
            continue;
        }
        const uint32_t rva = th::FindImportSlot(aBase, aSize, h.dll, h.name);
        if (!rva)
        {
            Log(true, "hook %s: not imported by the game executable, left alone", h.name);
            continue;
        }
        void** slot = reinterpret_cast<void**>(aBase + rva);
        void* current = *slot;
        if (current == h.detour)
        {
            h.slot = slot;
            h.installed = true;
            ++ok;
            continue;
        }
        h.next.store(current, std::memory_order_release);
        if (!WriteSlot(slot, h.detour))
        {
            Log(true, "hook %s: VirtualProtect failed (error %lu), left alone", h.name, GetLastError());
            continue;
        }
        h.slot = slot;
        h.installed = true;
        ++ok;
        Log(false, "hook %s: import slot RVA 0x%X, previous target %p", h.name, rva, current);
    }
    return ok;
}

void RemoveHooks()
{
    for (Hook& h : g_hooks)
    {
        if (!h.installed || !h.slot)
            continue;
        DWORD old = 0;
        if (!VirtualProtect(h.slot, sizeof(void*), PAGE_READWRITE, &old))
        {
            Log(true, "hook %s: VirtualProtect failed on removal (error %lu); stays installed and passes every call through",
                h.name, GetLastError());
            continue;
        }
        void* prev = InterlockedCompareExchangePointer(h.slot, h.next.load(), h.detour);
        DWORD tmp = 0;
        VirtualProtect(h.slot, sizeof(void*), old, &tmp);
        if (prev == h.detour)
        {
            h.installed = false;
            h.slot = nullptr;
        }
        else
        {
            Log(true, "hook %s: the import slot was changed by another module after the switch-on; the hook stays in the "
                      "chain and passes every call through",
                h.name);
        }
    }
}

// ------------------------------------------------------------------------------------------------ gate patch
bool ExeVersionOk(std::string& aWhy)
{
    wchar_t path[MAX_PATH * 2] = {};
    const DWORD len = GetModuleFileNameW(nullptr, path, static_cast<DWORD>(std::size(path)));
    if (len == 0 || len >= std::size(path))
    {
        aWhy = "GetModuleFileNameW failed";
        return false;
    }
    DWORD dummy = 0;
    const DWORD size = GetFileVersionInfoSizeW(path, &dummy);
    std::vector<uint8_t> data(size ? size : 1);
    VS_FIXEDFILEINFO* ffi = nullptr;
    UINT ffiLen = 0;
    if (!size || !GetFileVersionInfoW(path, 0, size, data.data()) ||
        !VerQueryValueW(data.data(), L"\\", reinterpret_cast<void**>(&ffi), &ffiLen) || !ffi || ffiLen < sizeof(*ffi))
    {
        aWhy = "no readable exe version resource";
        return false;
    }
    if (ffi->dwFileVersionMS != th::kExeFileVerMS || ffi->dwFileVersionLS != th::kExeFileVerLS)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "exe file version %u.%u.%u.%u is not 3.0.88.46235 (2.50)", HIWORD(ffi->dwFileVersionMS),
                      LOWORD(ffi->dwFileVersionMS), HIWORD(ffi->dwFileVersionLS), LOWORD(ffi->dwFileVersionLS));
        aWhy = buf;
        return false;
    }
    if (g_sdk && g_sdk->runtime)
    {
        const auto& v = *g_sdk->runtime;
        if (!(v.major == 2 && ((v.minor == 5 && v.patch == 0) || (v.minor == 50 && v.patch == 0))))
        {
            aWhy = "runtime product version is not 2.50";
            return false;
        }
    }
    return true;
}

bool SetGate(uint8_t* aGate, const uint8_t* aBytes)
{
    DWORD old = 0;
    if (!VirtualProtect(aGate, 2, PAGE_EXECUTE_READWRITE, &old))
        return false;
    SHORT value = 0;
    std::memcpy(&value, aBytes, 2);
    InterlockedExchange16(reinterpret_cast<SHORT volatile*>(aGate), value);   // RVA 0x20849C6 is 2-byte aligned
    DWORD tmp = 0;
    VirtualProtect(aGate, 2, old, &tmp);
    FlushInstructionCache(GetCurrentProcess(), aGate, 2);
    return aGate[0] == aBytes[0] && aGate[1] == aBytes[1];
}

const char* GateText()
{
    if (!g_gate)
        return "unchecked";
    switch (th::ClassifyGate(g_gate - th::kGateOffset))
    {
    case th::GateState::Original:
        return "original";
    case th::GateState::Patched:
        return "open";
    default:
        return "changed";
    }
}

std::string Counters()
{
    char buf[900];
    auto c = [](HookId i) { return static_cast<unsigned long long>(g_hooks[i].calls.load()); };
    auto b = [](HookId i) { return static_cast<unsigned long long>(g_hooks[i].blocked.load()); };
    int hooks = 0;
    for (const Hook& h : g_hooks)
        hooks += h.installed ? 1 : 0;
    std::snprintf(buf, sizeof(buf),
                  "gate=%s hooks=%d/%d fg=%d gateRuns[fg=%llu bg=%llu] pad[bg calls=%llu conn=%llu active=%llu] pad[fg "
                  "calls=%llu conn=%llu active=%llu] fgChanges=%llu otherForegroundCalls=%llu blocked/calls clip=%llu/%llu setCursorPos=%llu/%llu setCursor=%llu/%llu "
                  "setForeground=%llu/%llu setFocus=%llu/%llu setActive=%llu/%llu getCursorPos=%llu/%llu onFor=%llus",
                  GateText(), hooks, static_cast<int>(kHookCount), ReallyForeground() ? 1 : 0,
                  static_cast<unsigned long long>(g_gateRunsFg.load()), static_cast<unsigned long long>(g_gateRunsBg.load()),
                  static_cast<unsigned long long>(g_padBg.calls.load()), static_cast<unsigned long long>(g_padBg.connected.load()),
                  static_cast<unsigned long long>(g_padBg.active.load()), static_cast<unsigned long long>(g_padFg.calls.load()),
                  static_cast<unsigned long long>(g_padFg.connected.load()),
                  static_cast<unsigned long long>(g_padFg.active.load()), static_cast<unsigned long long>(g_fgChanges.load()),
                  static_cast<unsigned long long>(g_otherFgCalls.load()), b(kClipCursor), c(kClipCursor), b(kSetCursorPos),
                  c(kSetCursorPos), b(kSetCursor), c(kSetCursor), b(kSetForegroundWindow), c(kSetForegroundWindow),
                  b(kSetFocus), c(kSetFocus), b(kSetActiveWindow), c(kSetActiveWindow), b(kGetCursorPos), c(kGetCursorPos),
                  static_cast<unsigned long long>(g_on.load() ? (GetTickCount64() - g_onSince.load()) / 1000 : 0));
    return buf;
}

void MaybeSummary()
{
    const uint64_t now = GetTickCount64();
    uint64_t due = g_nextSummary.load();
    if (now < due || !g_nextSummary.compare_exchange_strong(due, now + 1000))
        return;
    Log(false, "summary %s", Counters().c_str());
}

void ResetCounters()
{
    for (PadCounters* p : {&g_padFg, &g_padBg})
    {
        p->calls = 0;
        p->connected = 0;
        p->active = 0;
    }
    g_padFirstBgActiveLogged = 0;
    g_fgChanges = 0;
    g_lastFg = -1;
    g_haveCursor = false;
    g_gateRunsFg = 0;
    g_gateRunsBg = 0;
    g_otherFgCalls = 0;
}

std::string SwitchOn()
{
    std::lock_guard<std::mutex> lock(g_switchMutex);
    if (g_on.load())
        return "UTPPTestHost on (already) " + Counters();
    auto refuse = [](const std::string& aWhy) {
        Log(true, "switch-on refused: %s", aWhy.c_str());
        return "UTPPTestHost refused: " + aWhy;
    };
    std::string why;
    if (!ExeVersionOk(why))
        return refuse(why);
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base)
        return refuse("no game module");
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    const uint64_t imageSize = nt->OptionalHeader.SizeOfImage;
    if (static_cast<uint64_t>(th::kGateBlockRva) + sizeof(th::kGateBlock) > imageSize)
        return refuse("image smaller than the gate site");
    uint8_t* block = base + th::kGateBlockRva;
    const th::GateState state = th::ClassifyGate(block);
    if (state == th::GateState::Unknown)
    {
        char hex[3 * sizeof(th::kGateBlock) + 1] = {};
        for (size_t i = 0; i < sizeof(th::kGateBlock); ++i)
            std::snprintf(hex + 3 * i, 4, "%02X ", block[i]);
        return refuse(std::string("gate bytes at RVA 0x20849AA differ: ") + hex);
    }
    const uint32_t callSlot = th::RipCallSlot(block + th::kForegroundCallOffset, th::kGateBlockRva + th::kForegroundCallOffset);
    const uint32_t importSlot = th::FindImportSlot(base, imageSize, "USER32.dll", "GetForegroundWindow");
    if (callSlot != th::kForegroundSlotRva || importSlot != th::kForegroundSlotRva)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "foreground call slot 0x%X / import slot 0x%X, expected 0x%X", callSlot, importSlot,
                      th::kForegroundSlotRva);
        return refuse(buf);
    }
    g_gate = block + th::kGateOffset;
    g_gateReturn = reinterpret_cast<uintptr_t>(block + th::kForegroundCallOffset + 6);   // RVA 0x20849BF
    ResetCounters();
    const int hooks = InstallHooks(base, imageSize);
    POINT cursor{};
    if (GetCursorPos(&cursor))   // the frozen cursor position when the game is already behind at switch-on
    {
        g_lastCursor.store(static_cast<uint32_t>(cursor.x) | (static_cast<uint64_t>(static_cast<uint32_t>(cursor.y)) << 32));
        g_haveCursor.store(true);
    }
    g_onSince = GetTickCount64();
    g_nextSummary = 0;
    g_on.store(true);
    if (!SetGate(g_gate, th::kGatePatched))
    {
        g_on.store(false);
        RemoveHooks();
        return refuse("gate write failed (error " + std::to_string(GetLastError()) + ")");
    }
    g_gatePatched = true;
    Log(false, "switched ON: gate RVA 0x%X %02X %02X -> %02X %02X (was %s), %d/%d import hooks, game %s", th::kGateRva,
        th::kGateOriginal[0], th::kGateOriginal[1], th::kGatePatched[0], th::kGatePatched[1],
        state == th::GateState::Patched ? "already open" : "original", hooks, static_cast<int>(kHookCount),
        ReallyForeground() ? "in front" : "behind another window");
    return "UTPPTestHost on " + Counters();
}

std::string SwitchOff(const char* aWhy)
{
    std::lock_guard<std::mutex> lock(g_switchMutex);
    if (!g_on.load() && !g_gatePatched)
        return "UTPPTestHost off (already)";
    const std::string last = Counters();
    bool restored = true;
    if (g_gatePatched && g_gate)
    {
        restored = SetGate(g_gate, th::kGateOriginal);
        g_gatePatched = !restored;
    }
    g_on.store(false);
    RemoveHooks();
    Log(!restored, "switched OFF (%s): gate %s; totals %s", aWhy, restored ? "restored to 75 0F" : "RESTORE FAILED",
        last.c_str());
    return std::string(restored ? "UTPPTestHost off " : "UTPPTestHost off gate-restore-failed ") + last;
}

// ------------------------------------------------------------------------------------------------ natives
void Native_BackgroundInput(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const std::string s = on ? SwitchOn() : SwitchOff("script");
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

void Native_BackgroundInputStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    const std::string s = std::string(g_on.load() ? "UTPPTestHost on " : "UTPPTestHost off ") + Counters();
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

void RegisterTypes()
{
}

void PostRegisterTypes()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const RED4ext::CBaseFunction::Flags flags = {.isNative = true, .isStatic = true};
    auto* f1 = RED4ext::CGlobalFunction::Create("UTPPTestBackgroundInput", "UTPPTestBackgroundInput", &Native_BackgroundInput);
    f1->flags = flags;
    f1->AddParam("Bool", "on");
    f1->SetReturnType("String");
    rtti->RegisterFunction(f1);
    auto* f2 = RED4ext::CGlobalFunction::Create("UTPPTestBackgroundInputStatus", "UTPPTestBackgroundInputStatus",
                                                &Native_BackgroundInputStatus);
    f2->flags = flags;
    f2->SetReturnType("String");
    rtti->RegisterFunction(f2);
    Log(false, "natives registered: UTPPTestBackgroundInput(Bool) -> String, UTPPTestBackgroundInputStatus() -> String");
}
} // namespace

RED4EXT_C_EXPORT bool RED4EXT_CALL Main(RED4ext::v1::PluginHandle aHandle, RED4ext::v1::EMainReason aReason,
                                        const RED4ext::v1::Sdk* aSdk)
{
    switch (aReason)
    {
    case RED4ext::v1::EMainReason::Load:
    {
        g_handle = aHandle;
        g_sdk = aSdk;
        InitHookTable();
        auto* rtti = RED4ext::CRTTISystem::Get();
        rtti->AddRegisterCallback(&RegisterTypes);
        rtti->AddPostRegisterCallback(&PostRegisterTypes);
        Log(false, "loaded, background pad input OFF (test installs only; switch with UTPPTestBackgroundInput)");
        break;
    }
    case RED4ext::v1::EMainReason::Unload:
        SwitchOff("unload");
        break;
    }
    return true;
}

RED4EXT_C_EXPORT void RED4EXT_CALL Query(RED4ext::v1::PluginInfo* aInfo)
{
    aInfo->name = L"UTPPTestHost";
    aInfo->author = L"Daniel Maciej Pytel";
    aInfo->version = RED4EXT_V1_SEMVER(0, 1, 0);
    aInfo->runtime = RED4EXT_V1_RUNTIME_VERSION_2_50;
    aInfo->sdk = RED4EXT_V1_SDK_VERSION_CURRENT;
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_1;
}

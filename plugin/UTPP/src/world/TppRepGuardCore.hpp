// UTPP vehicle exit guard of the player's TPP representation: code sites and the pure gating logic (no game, no Windows API;
// offline tests in tests/TppRepGuardTest.cpp). Evidence and the hook: TppRepGuard.cpp.
#pragma once

#include <cstdint>

namespace utpp::tpprepguard
{
// ---- 2.50 code sites (Cyberpunk2077.exe 3.0.88.46235; static analysis of the 2.50 exe, see TppRepGuard.cpp) ----
// Vehicle camera component: sync of the player's TPP representation (this = component; byte +0x130 = wanted,
// +0x131 = applied; [this+0x60] = the entity that receives the event).
inline constexpr uint32_t kRvaSync = 0xC32D84;
// mov [rsp+8],rbx; mov [rsp+10h],rdi; push rbp; mov rbp,rsp; sub rsp,40h; mov al,[rcx+131h]; mov rbx,rcx;
// cmp [rcx+130h],al; jne +10h
inline constexpr uint8_t kSyncPrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x7C, 0x24, 0x10, 0x55, 0x48,
                                            0x8B, 0xEC, 0x48, 0x83, 0xEC, 0x40, 0x8A, 0x81, 0x31, 0x01, 0x00, 0x00,
                                            0x48, 0x8B, 0xD9, 0x38, 0x81, 0x30, 0x01, 0x00, 0x00, 0x75};
inline constexpr uint32_t kSyncWantedOff = 0x130;
inline constexpr uint32_t kSyncAppliedOff = 0x131;
// call rel32 -> kRvaDefer (the vanilla "not now" check; true = the sync returns without any change)
inline constexpr uint32_t kRvaDeferCall = 0xC32DB7;
inline constexpr uint32_t kRvaDefer = 0x193A240;
// push rbx; sub rsp,20h; mov rcx,[rcx+150h]
inline constexpr uint8_t kDeferPrologue[] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0x89, 0x50, 0x01, 0x00, 0x00};
// mov rdi,[rbx+60h]; cmp [rbx+130h],al (al = 0); je rel32 -> kRvaDeactChunk (the wanted == 0 branch)
inline constexpr uint32_t kRvaDeactBranch = 0xC32DC4;
inline constexpr uint8_t kDeactBranch[] = {0x48, 0x8B, 0x7B, 0x60, 0x38, 0x83, 0x30, 0x01, 0x00, 0x00, 0x0F, 0x84};
inline constexpr uint32_t kRvaDeactChunk = 0x2542308;
// call rel32 -> kRvaDeactFactory (new DeactivateTPPRepresentationEvent, queued on [this+0x60] right after)
inline constexpr uint32_t kRvaDeactFactory = 0x2BACF50;
// Offline-only identity chain (tests): factory -> event ctor -> vtable whose GetType returns the class slot that
// TPPRepresentationComponent registers its Deactivate handler for.
inline constexpr uint32_t kRvaDeactFactoryCtorCall = 0x2BACF9D;
inline constexpr uint32_t kRvaDeactCtor = 0x2178B70;
inline constexpr uint32_t kRvaDeactCtorVtLea = 0x2178B81; // lea rcx,[rip+X] -> kRvaDeactVtable
inline constexpr uint32_t kRvaDeactVtable = 0x3649C50;
inline constexpr uint32_t kRvaDeactGetType = 0x1952B80; // mov rax,[rip+X] -> kRvaDeactTypeSlot; ret
inline constexpr uint32_t kRvaDeactTypeSlot = 0x3BC64D8;

// ---- gating (pure) ----
// One exit window: opened by Set(true), closed by Set(false). It is active for at most kMaxWindowMs after it opened;
// calling Set(true) again while it is open does not extend it, and after that cap only Set(false) re-arms it, so a caller
// that stops (or keeps saying "on") cannot leave the guard on.
inline constexpr uint64_t kMaxWindowMs = 3000;

struct Gate
{
    bool open = false;
    uint64_t openedAtMs = 0;
};

// Returns true when this call opened a new window.
inline bool GateSet(Gate& aGate, bool aOn, uint64_t aNowMs)
{
    if (!aOn)
    {
        aGate.open = false;
        return false;
    }
    // an expired window that was never closed (CET reload, script error) must not block the next exit
    if (aGate.open && aNowMs >= aGate.openedAtMs && aNowMs - aGate.openedAtMs < kMaxWindowMs)
        return false;
    aGate.open = true;
    aGate.openedAtMs = aNowMs;
    return true;
}

inline bool GateActive(bool aOpen, uint64_t aOpenedAtMs, uint64_t aNowMs)
{
    return aOpen && aNowMs >= aOpenedAtMs && aNowMs - aOpenedAtMs < kMaxWindowMs;
}

inline bool GateActive(const Gate& aGate, uint64_t aNowMs)
{
    return GateActive(aGate.open, aGate.openedAtMs, aNowMs);
}

enum class Decision : uint8_t
{
    Pass,          // run the game's sync unchanged
    SkipDeactivate // do what the sync does minus the event: applied = wanted (0), no DeactivateTPPRepresentationEvent
};

// aWanted / aApplied: the component bytes (+0x130 / +0x131). aDefer: the vanilla "not now" check (only consulted by the
// caller when this returns SkipDeactivate for aDefer = false; with aDefer = true the game itself changes nothing).
inline Decision Decide(bool aGateActive, uint8_t aWanted, uint8_t aApplied, bool aDefer)
{
    if (!aGateActive || aWanted == aApplied || aWanted != 0 || aDefer)
        return Decision::Pass;
    return Decision::SkipDeactivate;
}
} // namespace utpp::tpprepguard

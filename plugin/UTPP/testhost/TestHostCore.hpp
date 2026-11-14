// UTPPTestHost core: the background pad input site of the 2.50 executable and pure helpers (no Windows API, no RED4ext),
// shared by the test-only host plugin (TestHost.cpp) and its offline site test (tests/TestHostSiteTest.cpp).
//
// The engine's per-frame input update (RVA 0x5D5414) calls RVA 0x20849A0, which hands the pad events queued by the pad
// device thread to the game only while the game window is the foreground window:
//
//   020849AA  80 B9 A8 17 00 00 00   cmp  byte ptr [rcx+0x17A8], 0      ; start-up option: no pad events at all
//   020849B1  48 8B FA               mov  rdi, rdx
//   020849B4  48 8B D9               mov  rbx, rcx
//   020849B7  75 1E                  jne  0x20849D7
//   020849B9  FF 15 A1 F2 F6 00      call [GetForegroundWindow]         ; IAT slot RVA 0x2FF3C60
//   020849BF  48 3B 83 D0 01 00 00   cmp  rax, [rbx+0x1D0]              ; the game window
//   020849C6  75 0F                  jne  0x20849D7                     ; <- gate: skip the pad events when inactive
//   020849C8  48 8B 8B E8 01 00 00   mov  rcx, [rbx+0x1E8]              ; pad manager
//   020849CF  48 8B D7               mov  rdx, rdi
//   020849D2  E8 6D 03 55 FE         call 0x5D4D44                      ; move the queued pad events into the frame's input
//
// While switched on, the 2-byte jne becomes a 2-byte nop (66 90): the queued pad events reach the game whatever window is
// in front. With the game really in front the jump is never taken, so the patched and unpatched code behave the same.
// The focus loss handler (WM_ACTIVATE inactive / WM_KILLFOCUS -> one input reset on the next frame) is left alone.
#pragma once

#include <cstdint>
#include <cstring>

namespace utpp::testhost
{
// Game executable 2.50: file version 3.0.88.46235.
constexpr uint32_t kExeFileVerMS = (3u << 16) | 0u;
constexpr uint32_t kExeFileVerLS = (88u << 16) | 46235u;

constexpr uint32_t kGateBlockRva = 0x20849AA;
constexpr uint8_t kGateBlock[] = {
    0x80, 0xB9, 0xA8, 0x17, 0x00, 0x00, 0x00,       // cmp byte ptr [rcx+0x17A8], 0
    0x48, 0x8B, 0xFA,                               // mov rdi, rdx
    0x48, 0x8B, 0xD9,                               // mov rbx, rcx
    0x75, 0x1E,                                     // jne +0x1E
    0xFF, 0x15, 0xA1, 0xF2, 0xF6, 0x00,             // call [rip+0xF6F2A1] (GetForegroundWindow)
    0x48, 0x3B, 0x83, 0xD0, 0x01, 0x00, 0x00,       // cmp rax, [rbx+0x1D0]
    0x75, 0x0F,                                     // jne +0x0F   (gate)
    0x48, 0x8B, 0x8B, 0xE8, 0x01, 0x00, 0x00,       // mov rcx, [rbx+0x1E8]
    0x48, 0x8B, 0xD7,                               // mov rdx, rdi
    0xE8, 0x6D, 0x03, 0x55, 0xFE,                   // call 0x5D4D44
};
constexpr uint32_t kGateOffset = 0x1C;                        // offset of the gate jne inside kGateBlock
constexpr uint32_t kGateRva = kGateBlockRva + kGateOffset;    // 0x20849C6
constexpr uint8_t kGateOriginal[2] = {0x75, 0x0F};            // jne +0x0F
constexpr uint8_t kGatePatched[2] = {0x66, 0x90};             // 2-byte nop (one instruction: safe to swap back at any time)
constexpr uint32_t kForegroundCallOffset = 0x0F;              // the call [rip+disp32] inside kGateBlock
constexpr uint32_t kForegroundSlotRva = 0x2FF3C60;            // USER32!GetForegroundWindow IAT slot

static_assert(kGateRva == 0x20849C6, "gate RVA");
static_assert(sizeof(kGateBlock) == 0x2D, "gate block length");

enum class GateState
{
    Original,   // the game's own bytes (switch off)
    Patched,    // the 2-byte nop (switch on)
    Unknown,    // anything else: never written
};

// The block at kGateBlockRva compared with kGateBlock, the gate bytes allowed to be either the original or the patch.
inline GateState ClassifyGate(const uint8_t* aBlock)
{
    for (uint32_t i = 0; i < sizeof(kGateBlock); ++i)
    {
        if (i == kGateOffset || i == kGateOffset + 1)
            continue;
        if (aBlock[i] != kGateBlock[i])
            return GateState::Unknown;
    }
    const uint8_t* g = aBlock + kGateOffset;
    if (g[0] == kGateOriginal[0] && g[1] == kGateOriginal[1])
        return GateState::Original;
    if (g[0] == kGatePatched[0] && g[1] == kGatePatched[1])
        return GateState::Patched;
    return GateState::Unknown;
}

// Target RVA of the rip-relative call [rip+disp32] at aCallRva (FF 15 disp32), 0 when the bytes are not that call.
inline uint32_t RipCallSlot(const uint8_t* aCall, uint32_t aCallRva)
{
    if (aCall[0] != 0xFF || aCall[1] != 0x15)
        return 0;
    int32_t disp = 0;
    std::memcpy(&disp, aCall + 2, 4);
    return static_cast<uint32_t>(static_cast<int64_t>(aCallRva) + 6 + disp);
}

// Pad input outside the usual dead zones (XInput gamepad fields).
inline bool PadActive(uint16_t aButtons, uint8_t aLeftTrigger, uint8_t aRightTrigger, int16_t aLX, int16_t aLY, int16_t aRX,
                      int16_t aRY)
{
    auto outside = [](int16_t v, int dz) { return v > dz || v < -dz; };
    return aButtons != 0 || aLeftTrigger > 30 || aRightTrigger > 30 || outside(aLX, 7849) || outside(aLY, 7849) ||
           outside(aRX, 8689) || outside(aRY, 8689);
}

// Case-insensitive ASCII compare (import DLL names).
inline bool SameAsciiNoCase(const char* aA, const char* aB)
{
    for (; *aA && *aB; ++aA, ++aB)
    {
        char a = *aA, b = *aB;
        if (a >= 'A' && a <= 'Z')
            a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z')
            b = static_cast<char>(b - 'A' + 'a');
        if (a != b)
            return false;
    }
    return *aA == *aB;
}

// IAT slot RVA of aDll!aName in a PE image laid out at its RVAs (a loaded module or a section-mapped file copy);
// 0 = not imported by name. aSize bounds every read.
inline uint32_t FindImportSlot(const uint8_t* aImage, uint64_t aSize, const char* aDll, const char* aName)
{
    auto rd32 = [&](uint64_t o) -> uint32_t {
        uint32_t v = 0;
        if (o + 4 <= aSize)
            std::memcpy(&v, aImage + o, 4);
        return v;
    };
    auto rd64 = [&](uint64_t o) -> uint64_t {
        uint64_t v = 0;
        if (o + 8 <= aSize)
            std::memcpy(&v, aImage + o, 8);
        return v;
    };
    auto str = [&](uint64_t o) -> const char* {
        if (o >= aSize)
            return nullptr;
        for (uint64_t i = o; i < aSize && i < o + 256; ++i)
            if (aImage[i] == 0)
                return reinterpret_cast<const char*>(aImage + o);
        return nullptr;
    };
    if (aSize < 0x400 || aImage[0] != 'M' || aImage[1] != 'Z')
        return 0;
    const uint32_t pe = rd32(0x3C);
    if (rd32(pe) != 0x00004550)
        return 0;
    const uint64_t opt = pe + 24;
    const uint32_t impRva = rd32(opt + 112 + 8);   // data directory 1 (imports), PE32+
    if (!impRva)
        return 0;
    for (uint64_t d = impRva; d + 20 <= aSize; d += 20)
    {
        const uint32_t ilt = rd32(d), nameRva = rd32(d + 12), iat = rd32(d + 16);
        if (!nameRva && !iat)
            break;
        const char* dll = str(nameRva);
        if (!dll || !SameAsciiNoCase(dll, aDll))
            continue;
        const uint32_t names = ilt ? ilt : iat;
        for (uint32_t k = 0; k < 4096; ++k)
        {
            const uint64_t t = rd64(names + 8ull * k);
            if (!t)
                break;
            if (t & (1ull << 63))
                continue;   // by ordinal
            const char* n = str((t & 0x7FFFFFFFull) + 2);
            if (n && std::strcmp(n, aName) == 0)
                return static_cast<uint32_t>(iat + 8ull * k);
        }
    }
    return 0;
}
} // namespace utpp::testhost

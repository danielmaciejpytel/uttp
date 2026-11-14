// Offline test for the background pad input site of the test-only host plugin (testhost/TestHostCore.hpp) against a COPY
// of the 2.50 game executable (never the installed game file). No game, no RED4ext, no Windows API.
// Usage: utpp_testhost_site_test <Cyberpunk2077.exe copy>
//   positive: the gate block matches once in .text at RVA 0x20849AA, its call goes through the GetForegroundWindow import
//             slot, the pad event call targets RVA 0x5D4D44, every hooked import resolves, patch / restore round-trips;
//   negative: changed bytes, a different gate instruction or an unknown import -> no match.

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

#include "../testhost/TestHostCore.hpp"

using namespace utpp::testhost;

namespace
{
int g_fail = 0;
int g_pass = 0;

void Check(bool aOk, const char* aName)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aName);
    }
}

template <class T> T Rd(const std::vector<uint8_t>& aF, size_t aOff)
{
    T v{};
    if (aOff + sizeof(T) <= aF.size())
        std::memcpy(&v, aF.data() + aOff, sizeof(T));
    return v;
}

// The file laid out at its RVAs (sections zero-filled to VirtualSize); .text range returned through the out params.
bool Load(const char* aPath, std::vector<uint8_t>& aMem, uint32_t& aTextRva, uint32_t& aTextSize)
{
    std::ifstream in(aPath, std::ios::binary);
    if (!in)
        return false;
    std::vector<uint8_t> f((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (f.size() < 0x400 || f[0] != 'M' || f[1] != 'Z')
        return false;
    const uint32_t pe = Rd<uint32_t>(f, 0x3C);
    if (Rd<uint32_t>(f, pe) != 0x00004550)
        return false;
    const uint16_t ns = Rd<uint16_t>(f, pe + 6);
    const uint16_t optSize = Rd<uint16_t>(f, pe + 20);
    const uint32_t imageSize = Rd<uint32_t>(f, pe + 24 + 56);
    const uint32_t headers = Rd<uint32_t>(f, pe + 24 + 60);
    aMem.assign(imageSize, 0);
    std::memcpy(aMem.data(), f.data(), std::min<size_t>({headers, f.size(), aMem.size()}));
    aTextRva = aTextSize = 0;
    for (uint16_t i = 0; i < ns; ++i)
    {
        const size_t o = pe + 24 + optSize + 40u * i;
        char name[9] = {};
        std::memcpy(name, f.data() + o, 8);
        const uint32_t vsize = Rd<uint32_t>(f, o + 8), va = Rd<uint32_t>(f, o + 12);
        const uint32_t rawSize = Rd<uint32_t>(f, o + 16), raw = Rd<uint32_t>(f, o + 20);
        const size_t n = std::min<size_t>(std::min(rawSize, vsize), raw < f.size() ? f.size() - raw : 0);
        if (static_cast<size_t>(va) + n <= aMem.size())
            std::memcpy(aMem.data() + va, f.data() + raw, n);
        if (std::strcmp(name, ".text") == 0)
        {
            aTextRva = va;
            aTextSize = vsize;
        }
    }
    return aTextSize != 0;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_testhost_site_test <Cyberpunk2077.exe copy>\nFAIL: no executable\n");
        return 2;
    }
    std::vector<uint8_t> img;
    uint32_t textRva = 0, textSize = 0;
    if (!Load(argv[1], img, textRva, textSize))
    {
        std::printf("FAIL: cannot load %s\n", argv[1]);
        return 2;
    }
    const uint8_t* block = img.data() + kGateBlockRva;

    // the site as shipped
    Check(ClassifyGate(block) == GateState::Original, "gate block matches at RVA 0x20849AA with the original jne");
    Check(kGateRva % 2 == 0, "gate RVA is 2-byte aligned (single interlocked 16-bit write)");
    size_t hits = 0;
    uint32_t hitRva = 0;
    const uint8_t* text = img.data() + textRva;
    for (const uint8_t* p = text; (p = std::search(p, text + textSize, std::begin(kGateBlock), std::end(kGateBlock))) !=
                                  text + textSize;
         ++p)
    {
        ++hits;
        hitRva = static_cast<uint32_t>(p - img.data());
    }
    Check(hits == 1 && hitRva == kGateBlockRva, "gate block occurs exactly once in .text");
    Check(RipCallSlot(block + kForegroundCallOffset, kGateBlockRva + kForegroundCallOffset) == kForegroundSlotRva,
          "the gate's call goes through the GetForegroundWindow slot");
    Check(FindImportSlot(img.data(), img.size(), "USER32.dll", "GetForegroundWindow") == kForegroundSlotRva,
          "import table: USER32!GetForegroundWindow slot");
    Check(FindImportSlot(img.data(), img.size(), "user32.DLL", "GetForegroundWindow") == kForegroundSlotRva,
          "import lookup ignores the DLL name case");
    // the gate jne and the start-up option jne both skip to the same instruction after the pad event call
    Check(kGateRva + 2 + kGateBlock[kGateOffset + 1] == kGateBlockRva + sizeof(kGateBlock), "gate jne skips the pad event call");
    Check(kGateBlockRva + 0x0D + 2 + kGateBlock[0x0E] == kGateBlockRva + sizeof(kGateBlock), "option jne skips the same block");
    int32_t rel = 0;
    std::memcpy(&rel, block + sizeof(kGateBlock) - 4, 4);
    Check(kGateBlockRva + sizeof(kGateBlock) + static_cast<int64_t>(rel) == 0x5D4D44, "pad event call targets RVA 0x5D4D44");

    // every import the plugin hooks resolves to its 2.50 slot; no GetAsyncKeyState import (keyboard state of other windows)
    struct Imp
    {
        const char* dll;
        const char* name;
        uint32_t slot;
    };
    const Imp imps[] = {
        {"XINPUT9_1_0.dll", "XInputGetState", 0x2FF3F80}, {"USER32.dll", "ClipCursor", 0x2FF3C50},
        {"USER32.dll", "SetCursorPos", 0x2FF3BD0},        {"USER32.dll", "SetCursor", 0x2FF3C40},
        {"USER32.dll", "SetForegroundWindow", 0x2FF3BE8}, {"USER32.dll", "SetFocus", 0x2FF3BF0},
        {"USER32.dll", "SetActiveWindow", 0x2FF3CF8},     {"USER32.dll", "GetCursorPos", 0x2FF3C38},
    };
    for (const Imp& i : imps)
    {
        char name[96];
        std::snprintf(name, sizeof(name), "import slot %s!%s", i.dll, i.name);
        Check(FindImportSlot(img.data(), img.size(), i.dll, i.name) == i.slot, name);
    }
    Check(FindImportSlot(img.data(), img.size(), "USER32.dll", "GetAsyncKeyState") == 0, "no GetAsyncKeyState import");
    Check(FindImportSlot(img.data(), img.size(), "USER32.dll", "NoSuchFunction") == 0, "unknown import -> 0");
    Check(FindImportSlot(img.data(), img.size(), "NOSUCH.dll", "ClipCursor") == 0, "unknown DLL -> 0");

    // patch / restore on a copy of the block
    std::vector<uint8_t> copy(block, block + sizeof(kGateBlock));
    std::memcpy(copy.data() + kGateOffset, kGatePatched, 2);
    Check(ClassifyGate(copy.data()) == GateState::Patched, "patched gate is recognised");
    std::memcpy(copy.data() + kGateOffset, kGateOriginal, 2);
    Check(std::equal(copy.begin(), copy.end(), block), "restore gives the original bytes");
    copy[kGateOffset] = 0x74;   // je instead of jne
    Check(ClassifyGate(copy.data()) == GateState::Unknown, "different gate instruction -> unknown");
    copy[kGateOffset] = kGateOriginal[0];
    copy[0x17] ^= 0x01;         // ModRM of the cmp rax, [rbx+0x1D0]
    Check(ClassifyGate(copy.data()) == GateState::Unknown, "changed block byte -> unknown");
    copy[0x17] ^= 0x01;
    copy[0x11] ^= 0x10;         // call through another slot
    Check(RipCallSlot(copy.data() + kForegroundCallOffset, kGateBlockRva + kForegroundCallOffset) != kForegroundSlotRva,
          "changed call displacement -> other slot");

    // pad dead zones
    Check(!PadActive(0, 0, 0, 0, 0, 0, 0), "neutral pad");
    Check(!PadActive(0, 30, 30, 7849, -7849, 8689, -8689), "inside dead zones");
    Check(PadActive(0, 0, 0, 0, 32767, 0, 0), "left stick up");
    Check(PadActive(0x1000, 0, 0, 0, 0, 0, 0), "button A");
    Check(PadActive(0, 0, 255, 0, 0, 0, 0), "right trigger");

    std::printf("utpp_testhost_site_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

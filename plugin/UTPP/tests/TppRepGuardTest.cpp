// Offline test of the vehicle exit guard of the TPP representation (src/world/TppRepGuardCore.hpp): the gating logic and,
// given a copy of the 2.50 executable (argv[1]), the code sites and the identity chain of the deactivation event.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

#include "world/TppRepGuardCore.hpp"

using namespace utpp::tpprepguard;

namespace
{
int g_pass = 0;
int g_fail = 0;

void Check(bool aOk, const char* aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat);
    }
}

void GateTests()
{
    Gate g;
    Check(!GateActive(g, 1000), "closed gate is inactive");
    Check(GateSet(g, true, 1000) && GateActive(g, 1000), "on opens, active at once");
    Check(GateActive(g, 1000 + kMaxWindowMs - 1), "active just before the cap");
    Check(!GateSet(g, true, 2500) && g.openedAtMs == 1000, "repeat on while open does not reopen / extend");
    Check(!GateActive(g, 1000 + kMaxWindowMs), "inactive at the cap (3 s after the opening, refreshes ignored)");
    Check(GateSet(g, true, 5000) && GateActive(g, 5000) && g.openedAtMs == 5000,
          "on after the cap without an off (lost off after a reload): a new window opens");
    Check(!GateSet(g, false, 5100) && !g.open && !GateActive(g, 5100), "off closes");
    Check(GateSet(g, true, 6000) && GateActive(g, 6500), "next window after an off: active again");
    GateSet(g, false, 6600);
    Check(!GateActive(g, 6700), "off inside the window -> inactive at once");
    Check(!GateActive(true, 9000, 8999), "clock before the opening -> inactive");
}

void DecideTests()
{
    // the deactivation the game would send now (wanted 0, applied 1, no defer) is skipped only with the gate active
    Check(Decide(true, 0, 1, false) == Decision::SkipDeactivate, "gate active, wanted 0 != applied 1 -> skip");
    Check(Decide(false, 0, 1, false) == Decision::Pass, "gate inactive -> pass");
    Check(Decide(true, 1, 0, false) == Decision::Pass, "activation always passes");
    Check(Decide(true, 0, 0, false) == Decision::Pass, "nothing to do (wanted == applied) -> pass");
    Check(Decide(true, 1, 1, false) == Decision::Pass, "already active -> pass");
    Check(Decide(true, 0, 1, true) == Decision::Pass, "the game defers -> pass (the game changes nothing either)");
    Check(Decide(true, 0, 2, false) == Decision::SkipDeactivate, "applied any non-zero, wanted 0 -> skip");
}

// After a skip the detour writes applied = wanted: the next sync in the same window and after it has nothing to send.
void SequenceTest()
{
    Gate g;
    uint8_t wanted = 1, applied = 1;
    int sent = 0;
    auto sync = [&](uint64_t aNow) {
        const Decision d = Decide(GateActive(g, aNow), wanted, applied, false);
        if (d == Decision::SkipDeactivate)
        {
            applied = wanted;
            return;
        }
        if (wanted != applied)
        {
            if (wanted == 0)
                ++sent;
            applied = wanted;
        }
    };
    GateSet(g, true, 0);       // exit start
    for (uint64_t t = 0; t < 470; t += 16)
        sync(t);
    wanted = 0;                // the camera reaches FPP at 0.47 s
    for (uint64_t t = 470; t < 2000; t += 16)
        sync(t);
    GateSet(g, false, 2000);   // window end
    for (uint64_t t = 2000; t < 4000; t += 16)
        sync(t);
    Check(sent == 0 && applied == 0, "exit: no deactivation sent in or after the window, the camera state is released");
    wanted = 1;                // next mount with the TPP camera
    sync(5000);
    Check(applied == 1, "next mount activates as usual");
    wanted = 0;                // TPP -> FPP during a ride, no window
    sync(6000);
    Check(sent == 1, "ride perspective switch outside the window -> deactivation sent");
}

bool LoadImage(const char* aExe, std::vector<uint8_t>& aMem, uint32_t& aTextVa, uint32_t& aTextSize)
{
    std::ifstream in(aExe, std::ios::binary);
    if (!in)
        return false;
    std::vector<uint8_t> f((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (f.size() < 0x400)
        return false;
    auto rd32 = [&](size_t o) { uint32_t v = 0; if (o + 4 <= f.size()) std::memcpy(&v, f.data() + o, 4); return v; };
    auto rd16 = [&](size_t o) { uint16_t v = 0; if (o + 2 <= f.size()) std::memcpy(&v, f.data() + o, 2); return v; };
    const uint32_t pe = rd32(0x3C);
    if (rd32(pe) != 0x00004550)
        return false;
    const uint16_t ns = rd16(pe + 6);
    const uint16_t optSize = rd16(pe + 20);
    aMem.assign(rd32(pe + 24 + 56), 0);
    aTextVa = aTextSize = 0;
    for (uint16_t i = 0; i < ns; ++i)
    {
        const size_t o = pe + 24 + optSize + 40u * i;
        char name[9] = {};
        std::memcpy(name, f.data() + o, 8);
        const uint32_t vsize = rd32(o + 8), va = rd32(o + 12), rawSize = rd32(o + 16), raw = rd32(o + 20);
        size_t n = rawSize < vsize ? rawSize : vsize;
        if (raw + n > f.size())
            n = f.size() - raw;
        if (va + n <= aMem.size())
            std::memcpy(aMem.data() + va, f.data() + raw, n);
        if (std::strcmp(name, ".text") == 0)
        {
            aTextVa = va;
            aTextSize = vsize;
        }
    }
    return aTextSize != 0;
}

uint32_t RelTarget(const std::vector<uint8_t>& aMem, uint32_t aAt, uint32_t aOpLen)
{
    int32_t rel = 0;
    std::memcpy(&rel, aMem.data() + aAt + aOpLen, 4);
    return static_cast<uint32_t>(static_cast<int64_t>(aAt) + aOpLen + 4 + rel);
}

int CountIn(const std::vector<uint8_t>& aMem, uint32_t aVa, uint32_t aSize, const uint8_t* aPat, size_t aLen)
{
    int n = 0;
    for (uint32_t i = aVa; i + aLen <= aVa + aSize; ++i)
        if (aMem[i] == aPat[0] && std::memcmp(aMem.data() + i, aPat, aLen) == 0)
            ++n;
    return n;
}

void SiteTests(const char* aExe)
{
    std::vector<uint8_t> m;
    uint32_t tva = 0, tsz = 0;
    if (!LoadImage(aExe, m, tva, tsz))
    {
        Check(false, "exe copy loads (PE, .text)");
        return;
    }
    Check(std::memcmp(m.data() + kRvaSync, kSyncPrologue, sizeof(kSyncPrologue)) == 0, "sync prologue @0xC32D84");
    Check(CountIn(m, tva, tsz, kSyncPrologue, sizeof(kSyncPrologue)) == 1, "sync prologue unique in .text");
    Check(m[kRvaDeferCall] == 0xE8 && RelTarget(m, kRvaDeferCall, 1) == kRvaDefer, "defer call @0xC32DB7 -> 0x193A240");
    Check(std::memcmp(m.data() + kRvaDefer, kDeferPrologue, sizeof(kDeferPrologue)) == 0, "defer check prologue");
    Check(std::memcmp(m.data() + kRvaDeactBranch, kDeactBranch, sizeof(kDeactBranch)) == 0, "deactivation branch @0xC32DC4");
    Check(RelTarget(m, kRvaDeactBranch + 10, 2) == kRvaDeactChunk, "je -> cold chunk 0x2542308");
    Check(m[kRvaDeactChunk] == 0xE8 && RelTarget(m, kRvaDeactChunk, 1) == kRvaDeactFactory, "chunk call -> factory 0x2BACF50");
    // identity: factory -> ctor -> vtable -> GetType -> the class slot of gameDeactivateTPPRepresentationEvent
    Check(m[kRvaDeactFactoryCtorCall] == 0xE8 && RelTarget(m, kRvaDeactFactoryCtorCall, 1) == kRvaDeactCtor,
          "factory calls the event ctor 0x2178B70");
    Check(m[kRvaDeactCtorVtLea] == 0x48 && m[kRvaDeactCtorVtLea + 1] == 0x8D && m[kRvaDeactCtorVtLea + 2] == 0x0D &&
              RelTarget(m, kRvaDeactCtorVtLea, 3) == kRvaDeactVtable,
          "ctor stores vtable 0x3649C50");
    uint64_t vt0 = 0;
    std::memcpy(&vt0, m.data() + kRvaDeactVtable, 8);
    Check(vt0 == 0x140000000ull + kRvaDeactGetType, "vtable[0] = GetType 0x1952B80");
    Check(m[kRvaDeactGetType] == 0x48 && m[kRvaDeactGetType + 1] == 0x8B && m[kRvaDeactGetType + 2] == 0x05 &&
              RelTarget(m, kRvaDeactGetType, 3) == kRvaDeactTypeSlot && m[kRvaDeactGetType + 7] == 0xC3,
          "GetType returns the class slot 0x3BC64D8");
    // the class registration that fills the slot names it gameDeactivateTPPRepresentationEvent: lea rdx,[name] at
    // 0x17100DB, mov [slot],rdi at 0x171012C
    Check(RelTarget(m, 0x17100DB, 3) < m.size() &&
              std::strcmp(reinterpret_cast<const char*>(m.data() + RelTarget(m, 0x17100DB, 3)),
                          "gameDeactivateTPPRepresentationEvent") == 0,
          "registration 0x171008C names gameDeactivateTPPRepresentationEvent");
    Check(m[0x171012C] == 0x48 && m[0x171012D] == 0x89 && m[0x171012E] == 0x3D && RelTarget(m, 0x171012C, 3) == kRvaDeactTypeSlot,
          "registration stores the class into slot 0x3BC64D8");
}
} // namespace

int main(int argc, char** argv)
{
    GateTests();
    DecideTests();
    SequenceTest();
    if (argc > 1)
        SiteTests(argv[1]);
    else
        std::printf("note: no exe copy given, code site checks skipped\n");
    std::printf("tpprepguard test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

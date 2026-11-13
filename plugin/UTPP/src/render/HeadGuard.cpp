// UTPP HeadGuard (Phase 5N step 5b), game side. See render/HeadGuardCore.hpp (mechanism, safety rules, 2.31 site).
// Resolve() locates the built-in near-camera compare by our own signatures and
// validation (fail-closed: status FAILED, nothing written); UTPPHeadGuardSetSite can still supply a candidate for
// in-game experiments. Everything is game thread only.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "../world/LootHeadLog.hpp"
#include "HeadGuard.hpp"

namespace utpp::headguard
{
namespace
{
using loothead::LogF;

enum class Mode
{
    None,
    WriteConst,   // the (single-referenced) constant is rewritten in place
    RedirectDisp, // the instruction's disp32 points at a UTPP-owned float
};

struct State
{
    bool killFile = false;
    bool validated = false;
    bool enabled = false;
    bool killed = false;
    bool patchFailed = false;
    std::string patchWhy;

    // Proven site.
    bool siteReady = false;
    bool builtin = false;
    Mode mode = Mode::None;
    hg::Site site{"", 0, 0.0f, hg::kDefaultTppValue};
    std::string siteWhy = "unset (Resolve not run)";
    uint8_t* insn = nullptr;
    size_t insnLen = 0;
    // RedirectDisp
    uint8_t* dispAddr = nullptr;
    int32_t origDisp = 0;
    int32_t newDisp = 0;
    float* cell = nullptr; // UTPP-owned float near the image
    // WriteConst
    uint8_t* constAddr = nullptr;
    uint32_t constOrig = 0;
    uint32_t constNow = 0; // bits currently written while patched

    bool patched = false;
    uint64_t applies = 0;
    uint64_t restores = 0;
    uint64_t ticks = 0;
};

State* g = nullptr; // heap, never freed

State& S()
{
    if (!g)
        g = new State();
    return *g;
}

const char* ModeName(Mode aMode)
{
    return aMode == Mode::WriteConst ? "const" : (aMode == Mode::RedirectDisp ? "redirect" : "none");
}

uint32_t Bits(float aV)
{
    uint32_t b;
    std::memcpy(&b, &aV, 4);
    return b;
}

struct Section
{
    uint8_t* begin = nullptr;
    size_t size = 0;
    DWORD characteristics = 0;
};

uint8_t* ImageBase()
{
    return reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
}

bool FindSection(const void* aAddr, Section& aOut)
{
    auto* base = ImageBase();
    if (!base)
        return false;
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    const auto* sec = IMAGE_FIRST_SECTION(nt);
    for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; ++i)
    {
        uint8_t* b = base + sec[i].VirtualAddress;
        const size_t sz = sec[i].Misc.VirtualSize;
        if (!aAddr ? (std::memcmp(sec[i].Name, ".text", 6) == 0)
                   : (static_cast<const uint8_t*>(aAddr) >= b && static_cast<const uint8_t*>(aAddr) < b + sz))
        {
            aOut.begin = b;
            aOut.size = sz;
            aOut.characteristics = sec[i].Characteristics;
            return true;
        }
    }
    return false;
}

bool ReadFloatGuarded(const float* aAddr, float& aOut)
{
    __try
    {
        aOut = *static_cast<const volatile float*>(aAddr);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// ImageView callback: aligned float inside a readable, non-executable section of the loaded image.
bool ReadImageF32(void*, uint64_t aRva, float& aOut)
{
    const auto* p = reinterpret_cast<const float*>(ImageBase() + aRva);
    Section sec;
    if ((aRva & 3) != 0 || !FindSection(p, sec) || (sec.characteristics & IMAGE_SCN_MEM_EXECUTE) ||
        !(sec.characteristics & IMAGE_SCN_MEM_READ) || reinterpret_cast<const uint8_t*>(p) + 4 > sec.begin + sec.size)
        return false;
    return ReadFloatGuarded(p, aOut);
}

// Returns the number of matches (0..2) in aHits; aFault on an access violation.
size_t ScanGuarded(const Section& aText, const int16_t* aPat, size_t aN, size_t (&aHits)[2], bool& aFault)
{
    aFault = false;
    __try
    {
        return hg::ScanRaw(aText.begin, aText.size, aPat, aN, aHits, 2);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aFault = true;
        return 0;
    }
}

size_t CountRefsGuarded(const Section& aText, uint64_t aTextRva, uint64_t aTargetRva, bool& aFault)
{
    aFault = false;
    __try
    {
        return hg::CountRipRefs(aText.begin, aText.size, aTextRva, aTargetRva, 4);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aFault = true;
        return 0;
    }
}

bool FindBuiltinGuarded(const hg::ImageView& aView, hg::NearSite& aOut, const char*& aWhy, bool& aFault)
{
    aFault = false;
    __try
    {
        return hg::FindNearCameraSite(aView, aOut, aWhy);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aFault = true;
        aWhy = "access violation while scanning";
        return false;
    }
}

// Page within +-2 GB of aNear (searched downwards then upwards in 64 KB steps).
float* AllocCellNear(const uint8_t* aNear)
{
    SYSTEM_INFO si{};
    GetSystemInfo(&si);
    const uint64_t gran = si.dwAllocationGranularity ? si.dwAllocationGranularity : 0x10000;
    const uint64_t center = reinterpret_cast<uint64_t>(aNear) & ~(gran - 1);
    const uint64_t span = 0x70000000ull; // 1.75 GB
    const uint64_t lo = center > span ? center - span : gran;
    const uint64_t hi = center + span;
    for (int dir = 0; dir < 2; ++dir)
    {
        for (uint64_t a = dir == 0 ? center - gran : center + gran; dir == 0 ? a > lo : a < hi;
             a = dir == 0 ? a - gran : a + gran)
        {
            MEMORY_BASIC_INFORMATION mbi{};
            if (VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi)) != sizeof(mbi))
                continue;
            if (mbi.State != MEM_FREE)
                continue;
            void* p = VirtualAlloc(reinterpret_cast<void*>(a), static_cast<SIZE_T>(gran), MEM_RESERVE | MEM_COMMIT,
                                   PAGE_READWRITE);
            if (p)
                return static_cast<float*>(p);
        }
    }
    return nullptr;
}

const char* PatchWhy(hg::PatchResult aR)
{
    switch (aR)
    {
    case hg::PatchResult::Mismatch:
        return "bytes changed (another writer?)";
    case hg::PatchResult::Unaligned:
        return "target not in an aligned block";
    default:
        return "compare-exchange failed";
    }
}

// 4-byte compare-exchange at aAddr under a temporary protection change. aCode: flush the instruction cache.
bool Write4(uint8_t* aAddr, bool aCode, uint32_t aExpectOld, uint32_t aNew, std::string& aWhy)
{
    const State& s = S();
    DWORD old = 0;
    if (!VirtualProtect(aAddr, 4, aCode ? PAGE_EXECUTE_READWRITE : PAGE_READWRITE, &old))
    {
        aWhy = "VirtualProtect failed";
        return false;
    }
    hg::PatchResult r = hg::PatchResult::Raced;
    __try
    {
        r = aCode ? hg::Patch4(aAddr, aExpectOld, aNew) : hg::PatchConst4(aAddr, aExpectOld, aNew);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        r = hg::PatchResult::Raced;
    }
    DWORD tmp = 0;
    VirtualProtect(aAddr, 4, old, &tmp);
    if (aCode)
        FlushInstructionCache(GetCurrentProcess(), s.insn, s.insnLen);
    if (r != hg::PatchResult::Ok)
    {
        aWhy = PatchWhy(r);
        return false;
    }
    return true;
}

bool WriteApply(std::string& aWhy)
{
    State& s = S();
    if (s.mode == Mode::WriteConst)
    {
        const uint32_t v = Bits(s.site.value);
        if (!Write4(s.constAddr, false, s.constOrig, v, aWhy))
            return false;
        s.constNow = v;
        return true;
    }
    uint32_t e, n;
    std::memcpy(&e, &s.origDisp, 4);
    std::memcpy(&n, &s.newDisp, 4);
    return Write4(s.dispAddr, true, e, n, aWhy);
}

bool WriteRestore(std::string& aWhy)
{
    State& s = S();
    if (s.mode == Mode::WriteConst)
        return Write4(s.constAddr, false, s.constNow, s.constOrig, aWhy);
    uint32_t e, n;
    std::memcpy(&e, &s.newDisp, 4);
    std::memcpy(&n, &s.origDisp, 4);
    return Write4(s.dispAddr, true, e, n, aWhy);
}

void Apply()
{
    State& s = S();
    std::string why;
    if (WriteApply(why))
    {
        s.patched = true;
        ++s.applies;
        LogF(0, "UTPP HeadGuard: near-camera threshold %.4f -> %.4f (%s) at %p", static_cast<double>(s.site.expected),
             static_cast<double>(s.site.value), ModeName(s.mode), s.insn);
    }
    else
    {
        s.patchFailed = true;
        s.patchWhy = why;
        LogF(2, "UTPP HeadGuard: apply FAILED (%s) - off for the session", why.c_str());
    }
}

void Restore(const char* aWhy)
{
    State& s = S();
    if (!s.patched)
        return;
    std::string why;
    if (WriteRestore(why))
    {
        s.patched = false;
        ++s.restores;
        LogF(0, "UTPP HeadGuard: original threshold restored (%s)", aWhy);
    }
    else
    {
        s.patchFailed = true;
        s.patchWhy = why;
        LogF(2, "UTPP HeadGuard: restore FAILED (%s)", why.c_str());
    }
}

// Arms a decoded site: picks WriteConst when the constant has exactly one RIP-relative reference in .text,
// otherwise RedirectDisp when the disp32 can be swapped atomically; anything else is refused.
bool Arm(uint8_t* aInsn, const hg::RipInsn& aRi, const float* aTarget, size_t aRefs,
         const hg::Site& aSite, bool aBuiltin, std::string& aWhy)
{
    State& s = S();
    Mode mode = Mode::None;
    int32_t rel = 0;
    if (aRefs == 1)
        mode = Mode::WriteConst;
    else if (hg::DispCasPossible(reinterpret_cast<uintptr_t>(aInsn + aRi.dispPos)))
    {
        if (!s.cell)
            s.cell = AllocCellNear(aInsn);
        if (!s.cell || !hg::Rel32(reinterpret_cast<uint64_t>(aInsn), aRi.len, reinterpret_cast<uint64_t>(s.cell), rel))
        {
            aWhy = "no reachable memory for the UTPP constant";
            return false;
        }
        mode = Mode::RedirectDisp;
    }
    else
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "constant has %zu RIP references and disp32 is not atomically writable", aRefs);
        aWhy = buf;
        return false;
    }
    s.mode = mode;
    s.site = aSite;
    s.builtin = aBuiltin;
    s.insn = aInsn;
    s.insnLen = aRi.len;
    s.dispAddr = aInsn + aRi.dispPos;
    s.origDisp = aRi.disp;
    s.newDisp = rel;
    s.constAddr = reinterpret_cast<uint8_t*>(const_cast<float*>(aTarget));
    float cur = 0.0f;
    ReadFloatGuarded(aTarget, cur);
    s.constOrig = Bits(cur);
    s.constNow = s.constOrig;
    if (mode == Mode::RedirectDisp)
        *s.cell = aSite.value;
    s.siteReady = true;
    s.patchFailed = false;
    s.patchWhy.clear();
    return true;
}

hg::Inputs MakeInputs(bool aTpp)
{
    const State& s = S();
    hg::Inputs in;
    in.siteReady = s.siteReady;
    in.enabled = s.enabled;
    in.killed = s.killed;
    in.tpp = aTpp;
    in.killFile = s.killFile;
    in.validated = s.validated;
    in.patchFailed = s.patchFailed;
    return in;
}
} // namespace

void Init(bool aKillFile)
{
    State& s = S();
    s.killFile = s.killFile || aKillFile;
}

void Resolve(bool aGameValidated)
{
    State& s = S();
    s.validated = aGameValidated;
    if (s.patched)
        return;
    if (s.killFile)
    {
        s.siteWhy = "skipped (kill file)";
        LogF(1, "UTPP HeadGuard: %s; validated=%d", s.siteWhy.c_str(), aGameValidated ? 1 : 0);
        return;
    }
    Section text;
    auto* base = ImageBase();
    if (!base || !FindSection(nullptr, text) || !(text.characteristics & IMAGE_SCN_MEM_EXECUTE))
    {
        s.siteWhy = "FAILED (.text not found)";
        LogF(2, "UTPP HeadGuard: %s - inert", s.siteWhy.c_str());
        return;
    }
    hg::ImageView view;
    view.text = text.begin;
    view.textSize = text.size;
    view.textRva = static_cast<uint64_t>(text.begin - base);
    view.readF32 = &ReadImageF32;
    hg::NearSite ns;
    const char* why = nullptr;
    bool fault = false;
    const ULONGLONG t0 = GetTickCount64();
    const bool found = FindBuiltinGuarded(view, ns, why, fault);
    const ULONGLONG ms = GetTickCount64() - t0;
    if (!found)
    {
        s.siteWhy = std::string("FAILED (") + (why ? why : "?") + ")";
        LogF(2, "UTPP HeadGuard: built-in near-camera site %s - inert, nothing patched (%llu ms)", s.siteWhy.c_str(),
             static_cast<unsigned long long>(ms));
        return;
    }
    hg::Site site;
    site.pattern = std::string("builtin:") + hg::kNearSigs[ns.sig].name;
    site.insnOffset = 0;
    site.expected = hg::kNearSqExpected;
    site.value = s.site.value; // kDefaultTppValue unless UTPPHeadGuardSetValue ran earlier
    uint8_t* insn = text.begin + ns.insnOff;
    const auto* target = reinterpret_cast<const float*>(base + ns.constRva);
    std::string armWhy;
    if (!Arm(insn, ns.insn, target, ns.constRefs, site, true, armWhy))
    {
        s.siteWhy = "FAILED (" + armWhy + ")";
        LogF(2, "UTPP HeadGuard: built-in near-camera site %s - inert", s.siteWhy.c_str());
        return;
    }
    char buf[240];
    std::snprintf(buf, sizeof(buf), "builtin %s: compare at RVA 0x%llX, threshold RVA 0x%llX = %.6g (refs %zu), mode %s",
                  hg::kNearSigs[ns.sig].name, static_cast<unsigned long long>(view.textRva + ns.insnOff),
                  static_cast<unsigned long long>(ns.constRva), static_cast<double>(ns.constValue), ns.constRefs,
                  ModeName(s.mode));
    s.siteWhy = buf;
    LogF(1, "UTPP HeadGuard: site OK - %s; TPP value %.4f; validated=%d (%llu ms)", buf,
         static_cast<double>(s.site.value), aGameValidated ? 1 : 0, static_cast<unsigned long long>(ms));
}

bool SetSite(const hg::Site& aSite, std::string& aWhy)
{
    State& s = S();
    if (s.patched)
    {
        aWhy = "patched: disable first";
        return false;
    }
    if (s.killed || s.killFile || !s.validated)
    {
        aWhy = "killed / kill file / game not validated";
        return false;
    }
    hg::Pattern pat;
    if (!hg::ParsePattern(aSite.pattern, pat))
    {
        aWhy = "bad pattern";
        return false;
    }
    if (aSite.insnOffset < -64 || aSite.insnOffset > 64 || !hg::ValidSiteValue(aSite.expected) ||
        !hg::ValidSiteValue(aSite.value))
    {
        aWhy = "bad offset / value";
        return false;
    }
    Section text;
    if (!FindSection(nullptr, text) || !(text.characteristics & IMAGE_SCN_MEM_EXECUTE))
    {
        aWhy = ".text not found";
        return false;
    }
    bool fault = false;
    size_t hits[2] = {};
    const size_t nHits = ScanGuarded(text, pat.bytes.data(), pat.bytes.size(), hits, fault);
    if (fault || nHits != 1)
    {
        aWhy = fault ? "scan fault" : (nHits == 0 ? "pattern not found" : "pattern not unique");
        return false;
    }
    const int64_t at = static_cast<int64_t>(hits[0]) + aSite.insnOffset;
    if (at < 0 || static_cast<size_t>(at) + 16 > text.size)
    {
        aWhy = "instruction outside .text";
        return false;
    }
    uint8_t* insn = text.begin + at;
    hg::RipInsn ri;
    if (!hg::DecodeSseRip(insn, 16, ri))
    {
        aWhy = "instruction is not a scalar-float RIP-relative m32 operation";
        return false;
    }
    const auto* target = reinterpret_cast<const float*>(insn + ri.len + ri.disp);
    Section tsec;
    if (!FindSection(target, tsec) || (tsec.characteristics & IMAGE_SCN_MEM_EXECUTE) ||
        (reinterpret_cast<uintptr_t>(target) & 3) != 0)
    {
        aWhy = "operand is not an aligned constant in a data section of the image";
        return false;
    }
    float cur = 0.0f;
    if (!ReadFloatGuarded(target, cur) || !hg::FloatMatches(cur, aSite.expected))
    {
        char buf[120];
        std::snprintf(buf, sizeof(buf), "constant is %.6g, expected %.6g", static_cast<double>(cur),
                      static_cast<double>(aSite.expected));
        aWhy = buf;
        return false;
    }
    auto* base = ImageBase();
    const uint64_t textRva = static_cast<uint64_t>(text.begin - base);
    const uint64_t targetRva = static_cast<uint64_t>(reinterpret_cast<const uint8_t*>(target) - base);
    const size_t refs = CountRefsGuarded(text, textRva, targetRva, fault);
    if (fault)
    {
        aWhy = "reference scan fault";
        return false;
    }
    if (!Arm(insn, ri, target, refs, aSite, false, aWhy))
        return false;
    char buf[220];
    std::snprintf(buf, sizeof(buf), "candidate at .text+0x%llX (op 0x%02X%s), constant %.6g (refs %zu), mode %s",
                  static_cast<unsigned long long>(at), ri.opcode, ri.vex ? " VEX" : "", static_cast<double>(cur), refs,
                  ModeName(s.mode));
    s.siteWhy = buf;
    LogF(1, "UTPP HeadGuard: site set: %s; value while TPP %.6g", buf, static_cast<double>(aSite.value));
    return true;
}

bool SetValue(float aValue)
{
    State& s = S();
    if (!hg::ValidSiteValue(aValue))
        return false;
    if (s.patched && s.mode == Mode::WriteConst)
    {
        std::string why;
        const uint32_t v = Bits(aValue);
        if (!Write4(s.constAddr, false, s.constNow, v, why))
        {
            LogF(2, "UTPP HeadGuard: SetValue write FAILED (%s)", why.c_str());
            return false;
        }
        s.constNow = v;
    }
    s.site.value = aValue;
    if (s.cell)
    {
        long bits;
        std::memcpy(&bits, &aValue, 4);
        _InterlockedExchange(reinterpret_cast<volatile long*>(s.cell), bits);
    }
    return true;
}

void Tick(bool aTpp)
{
    State& s = S();
    ++s.ticks;
    const bool want = hg::WantPatched(MakeInputs(aTpp));
    if (want && !s.patched)
        Apply();
    else if (!want && s.patched)
        Restore(s.killed ? "killed" : (aTpp ? "disabled" : "not TPP"));
}

void Shutdown()
{
    if (!g)
        return;
    g->killed = true;
    Restore("unload");
}

void SetEnabled(bool aOn)
{
    S().enabled = aOn;
    LogF(0, "UTPPHeadGuardSetEnabled(%d) site=%d killed=%d", aOn ? 1 : 0, S().siteReady ? 1 : 0,
         S().killed ? 1 : 0);
}

void Kill()
{
    S().killed = true;
    Restore("killed");
    LogF(1, "UTPPHeadGuardKill(): vanilla threshold for the rest of the session");
}

bool Active()
{
    return S().patched;
}

uint32_t Reasons(bool aTpp)
{
    return hg::Reasons(MakeInputs(aTpp));
}

std::string Status(bool aTpp)
{
    const State& s = S();
    char buf[900];
    std::snprintf(buf, sizeof(buf),
                  "UTPP HeadGuard site=%d (%s) mode=%s enabled=%d killed=%d killFile=%d validated=%d reasons=0x%X "
                  "tpp=%d patched=%d applies=%llu restores=%llu value=%.4f failed=%d%s%s ticks=%llu",
                  s.siteReady ? 1 : 0, s.siteWhy.c_str(), ModeName(s.mode), s.enabled ? 1 : 0, s.killed ? 1 : 0,
                  s.killFile ? 1 : 0, s.validated ? 1 : 0,
                  hg::Reasons(MakeInputs(aTpp)), aTpp ? 1 : 0, s.patched ? 1 : 0,
                  static_cast<unsigned long long>(s.applies), static_cast<unsigned long long>(s.restores),
                  static_cast<double>(s.site.value), s.patchFailed ? 1 : 0, s.patchWhy.empty() ? "" : " ",
                  s.patchWhy.c_str(), static_cast<unsigned long long>(s.ticks));
    return buf;
}
} // namespace utpp::headguard

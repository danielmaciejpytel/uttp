// Offline test for the built-in HeadGuard site (render/HeadGuardCore.hpp) against a COPY of the game executable
// (Extracted\reverse\exe250\Cyberpunk2077.exe = game 2.5, never the installed game file; 2.31 values in the comments below).
// No game, no RED4ext, no Windows API.
// Usage: utpp_headguard_site_test <Cyberpunk2077.exe copy>
//   positive: the signature resolves uniquely to the 2.5 compare, constants 0.2025 / 0.04, 1/131072 scale, one
//             RIP reference -> WriteConst; apply + restore on a memory copy round-trips byte-exact;
//   negative: mutated / duplicated code, changed or already-patched constant, shared constant -> FAILED, no write.

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "../src/render/HeadGuardCore.hpp"

using namespace utpp;

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

struct Sec
{
    char name[9] = {};
    uint32_t va = 0, vsize = 0, raw = 0, rawSize = 0, chars = 0;
};

// The file as loaded: sections copied to their RVAs (zero-filled to VirtualSize).
struct Image
{
    std::vector<uint8_t> mem;
    std::vector<Sec> secs;
    const Sec* text = nullptr;
};

template <class T> T Rd(const std::vector<uint8_t>& aF, size_t aOff)
{
    T v{};
    if (aOff + sizeof(T) <= aF.size())
        std::memcpy(&v, aF.data() + aOff, sizeof(T));
    return v;
}

bool Load(const char* aPath, Image& aImg)
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
    aImg.mem.assign(imageSize, 0);
    for (uint16_t i = 0; i < ns; ++i)
    {
        const size_t o = pe + 24 + optSize + 40u * i;
        Sec s;
        std::memcpy(s.name, f.data() + o, 8);
        s.vsize = Rd<uint32_t>(f, o + 8);
        s.va = Rd<uint32_t>(f, o + 12);
        s.rawSize = Rd<uint32_t>(f, o + 16);
        s.raw = Rd<uint32_t>(f, o + 20);
        s.chars = Rd<uint32_t>(f, o + 36);
        const size_t n = std::min<size_t>(std::min(s.rawSize, s.vsize), f.size() - s.raw);
        if (s.va + n <= aImg.mem.size())
            std::memcpy(aImg.mem.data() + s.va, f.data() + s.raw, n);
        aImg.secs.push_back(s);
    }
    for (const Sec& s : aImg.secs)
        if (std::strcmp(s.name, ".text") == 0)
            aImg.text = &s;
    return aImg.text != nullptr;
}

constexpr uint32_t kExec = 0x20000000;
constexpr uint32_t kRead = 0x40000000;

bool ReadF32(void* aCtx, uint64_t aRva, float& aOut)
{
    const Image& img = *static_cast<const Image*>(aCtx);
    if (aRva & 3)
        return false;
    for (const Sec& s : img.secs)
        if (aRva >= s.va && aRva + 4 <= static_cast<uint64_t>(s.va) + s.vsize)
        {
            if ((s.chars & kExec) || !(s.chars & kRead))
                return false;
            std::memcpy(&aOut, img.mem.data() + aRva, 4);
            return true;
        }
    return false;
}

hg::ImageView View(Image& aImg)
{
    hg::ImageView v;
    v.text = aImg.mem.data() + aImg.text->va;
    v.textSize = aImg.text->vsize;
    v.textRva = aImg.text->va;
    v.ctx = &aImg;
    v.readF32 = &ReadF32;
    return v;
}

bool Find(Image& aImg, hg::NearSite& aOut, std::string& aWhy)
{
    const char* why = nullptr;
    const bool ok = hg::FindNearCameraSite(View(aImg), aOut, why);
    aWhy = why ? why : "";
    return ok;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_headguard_site_test <Cyberpunk2077.exe copy>\nFAIL: no executable\n");
        return 2;
    }
    Image img;
    if (!Load(argv[1], img))
    {
        std::printf("FAIL: cannot load %s\n", argv[1]);
        return 2;
    }
    const std::vector<uint8_t> pristine = img.mem;
    const uint64_t textVa = img.text->va;

    // ---- Positive: 2.5.
    hg::NearSite ns;
    std::string why;
    const bool found = Find(img, ns, why);
    Check(found, "2.5: built-in near-camera site found");
    if (!found)
    {
        std::printf("why: %s\n", why.c_str());
        std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
        return 1;
    }
    const uint64_t insnRva = textVa + ns.insnOff;
    std::printf("site: sig=%s compare RVA 0x%llX threshold RVA 0x%llX = %.9g inner RVA 0x%llX scale RVA 0x%llX refs=%zu "
                "disp@0x%llX\n",
                hg::kNearSigs[ns.sig].name, static_cast<unsigned long long>(insnRva),
                static_cast<unsigned long long>(ns.constRva), static_cast<double>(ns.constValue),
                static_cast<unsigned long long>(ns.innerRva), static_cast<unsigned long long>(ns.scaleRva),
                ns.constRefs, static_cast<unsigned long long>(insnRva + ns.insn.dispPos));
    Check(ns.sig == 0, "2.5: tightest signature wins");
    // 2.31 (Extracted\reverse\exe231): compare 0x52932A, threshold 0x31EF144, inner 0x31EF030 (moved in 2.5, same code)
    Check(insnRva == 0x51C25A, "2.5: compare at RVA 0x51C25A (CameraSystem PlayerAimUpdateTick listener update)");
    Check(ns.insn.opcode == 0x2F && !ns.insn.vex && ns.insn.len == 7, "2.5: comiss xmm, [rip+disp32]");
    Check(ns.constRva == 0x37B9AB4 && ns.innerRva == 0x37B99A0, "2.5: threshold / inner constants in .rdata");
    Check(hg::FloatMatches(ns.constValue, 0.2025f), "2.5: threshold is 0.45 m squared");
    Check(ns.constRefs == 1, "2.5: threshold constant has exactly one RIP reference (WriteConst allowed)");
    Check(!hg::DispCasPossible(static_cast<uintptr_t>(insnRva + ns.insn.dispPos)),
          "2.5: disp32 straddles a 16-byte block (RedirectDisp impossible, WriteConst needed)");

    // Each signature alone is unique in 2.5 (robustness when the tighter one breaks).
    for (int i = 0; i < hg::kNearSigCount; ++i)
    {
        size_t hits[2] = {};
        const size_t n = hg::ScanRaw(img.mem.data() + textVa, img.text->vsize, hg::kNearSigs[i].bytes,
                                     hg::kNearSigs[i].n, hits, 2);
        const std::string name = std::string("2.5: signature '") + hg::kNearSigs[i].name + "' unique at the compare";
        Check(n == 1 && hits[0] + hg::kNearSigs[i].insnOffset == ns.insnOff, name.c_str());
    }

    // WriteConst round trip on the memory copy (what the game side does under VirtualProtect).
    {
        uint8_t* c = img.mem.data() + ns.constRva;
        uint32_t orig;
        std::memcpy(&orig, c, 4);
        const uint32_t zero = 0;
        Check(hg::PatchConst4(c, orig, zero) == hg::PatchResult::Ok, "writeconst: apply 0.0");
        Check(hg::PatchConst4(c, orig, zero) == hg::PatchResult::Mismatch, "writeconst: second apply refused");
        hg::NearSite again;
        std::string w;
        Check(!Find(img, again, w), "writeconst: patched constant -> FAILED");
        std::printf("  patched constant: %s\n", w.c_str());
        Check(hg::PatchConst4(c, zero, orig) == hg::PatchResult::Ok, "writeconst: restore");
        Check(img.mem == pristine, "writeconst: image byte-exact after restore");
        Check(hg::PatchConst4(c + 1, orig, zero) == hg::PatchResult::Unaligned, "writeconst: unaligned refused");
    }

    // Value semantics: 0 makes every squared distance >= threshold ("far" branch).
    {
        const float d2[] = {0.0f, 0.01f, 0.04f, 0.1f, 0.2025f, 4.0f};
        bool far = true;
        for (float v : d2)
            far = far && !(v < hg::kDefaultTppValue);
        Check(far, "value: threshold 0 -> jb never taken for d2 >= 0");
    }

    // ---- Negative: each mutation must make the resolver fail (nothing to patch), then the copy is restored.
    struct Mut
    {
        const char* name;
        int64_t off; // from the compare
        uint8_t value;
    };
    const Mut muts[] = {
        {"neg: jb -> jae after first compare", 7, 0x73},
        {"neg: first comiss -> ucomiss", 1, 0x2E},
        {"neg: second compare register differs", 18, 0x1D},
        {"neg: ja -> jbe after second compare", 23, 0x76},
        {"neg: first comiss modrm not RIP-relative", 2, 0x55},
    };
    for (const Mut& m : muts)
    {
        uint8_t& b = img.mem[textVa + ns.insnOff + m.off];
        const uint8_t old = b;
        b = m.value;
        hg::NearSite x;
        std::string w;
        Check(!Find(img, x, w), m.name);
        std::printf("  %s: %s\n", m.name, w.c_str());
        b = old;
    }
    {
        // Changed constants (other build / other mod).
        float* t = reinterpret_cast<float*>(img.mem.data() + ns.constRva);
        float* inner = reinterpret_cast<float*>(img.mem.data() + ns.innerRva);
        const float ot = *t, oi = *inner;
        *t = 0.25f;
        hg::NearSite x;
        std::string w;
        Check(!Find(img, x, w), "neg: threshold constant 0.25 -> FAILED");
        *t = ot;
        *inner = 0.05f;
        Check(!Find(img, x, w), "neg: inner constant 0.05 -> FAILED");
        *inner = oi;
    }
    {
        // Missing FixedPoint scale load (constant changed).
        float* sc = reinterpret_cast<float*>(img.mem.data() + ns.scaleRva);
        const float o = *sc;
        *sc = 1.0f / 65536.0f;
        hg::NearSite x;
        std::string w;
        Check(!Find(img, x, w), "neg: no 1/131072 scale before the compare -> FAILED");
        std::printf("  scale: %s\n", w.c_str());
        *sc = o;
    }
    {
        // Duplicate the compare block elsewhere in .text (scratch area of the copy) -> both signatures non-unique.
        const size_t len = 36 + 8;
        const size_t dst = 0x100000;
        std::vector<uint8_t> saved(img.mem.begin() + textVa + dst, img.mem.begin() + textVa + dst + len);
        std::memcpy(img.mem.data() + textVa + dst, img.mem.data() + textVa + ns.insnOff - 12, len);
        hg::NearSite x;
        std::string w;
        const bool f = Find(img, x, w);
        Check(!f && w == "signature not unique", "neg: duplicated compare -> not unique -> FAILED");
        std::memcpy(img.mem.data() + textVa + dst, saved.data(), len);
    }
    {
        // Other RIP-relative accesses to the threshold constant -> WriteConst must be refused (refs > 1).
        struct Extra
        {
            const char* name;
            uint8_t op[4];
            int64_t delta; // target = constant + delta
            size_t expect;
        };
        const Extra extras[] = {
            {"refs: second movss of the constant counted", {0xF3, 0x0F, 0x10, 0x05}, 0, 2},
            {"refs: movups covering the constant (16 B from -8) counted", {0x0F, 0x10, 0x05, 0x90}, -8, 2},
            {"refs: movss of the neighbour 4 B below ignored", {0xF3, 0x0F, 0x10, 0x05}, -4, 1},
        };
        const size_t dst = 0x100000;
        for (const Extra& e : extras)
        {
            std::vector<uint8_t> saved(img.mem.begin() + textVa + dst, img.mem.begin() + textVa + dst + 8);
            uint8_t* p = img.mem.data() + textVa + dst;
            const size_t opLen = e.op[3] == 0x05 ? 4 : 3; // modrm included
            std::memcpy(p, e.op, opLen);
            const int64_t end = static_cast<int64_t>(textVa + dst + opLen + 4);
            const int32_t disp = static_cast<int32_t>(static_cast<int64_t>(ns.constRva) + e.delta - end);
            std::memcpy(p + opLen, &disp, 4);
            const size_t refs = hg::CountRipRefs(img.mem.data() + textVa, img.text->vsize, textVa, ns.constRva, 4);
            Check(refs == e.expect, e.name);
            if (refs != e.expect)
                std::printf("  refs=%zu expected %zu\n", refs, e.expect);
            std::memcpy(p, saved.data(), 8);
        }
    }
    Check(img.mem == pristine, "image byte-exact after all negative tests");
    {
        hg::NearSite x;
        std::string w;
        Check(Find(img, x, w) && x.insnOff == ns.insnOff, "re-scan after restore finds the same site");
    }
    // Empty / truncated views.
    {
        hg::ImageView v;
        hg::NearSite x;
        const char* w = nullptr;
        Check(!hg::FindNearCameraSite(v, x, w), "neg: empty view");
        hg::ImageView t = View(img);
        t.textSize = ns.insnOff + 20; // second compare cut off
        Check(!hg::FindNearCameraSite(t, x, w), "neg: truncated .text");
        hg::ImageView nr = View(img);
        nr.readF32 = nullptr;
        Check(!hg::FindNearCameraSite(nr, x, w), "neg: unreadable constants");
    }

    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

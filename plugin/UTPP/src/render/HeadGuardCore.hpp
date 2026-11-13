// UTPP HeadGuard core (Phase 5N step 5b): pure logic, no game / Windows API (offline-tested in tests/LootHeadTest.cpp
// and tests/HeadGuardSiteTest.cpp against a copy of the 2.31 executable).
// Mechanism: the engine hides / dissolves the head when the camera is close; UTPP lowers that near-camera threshold in
// TPP and restores it in FPP. The built-in signatures and checks below were derived from our own disassembly of
// Cyberpunk2077.exe 2.31.
//
// Two fail-closed patch mechanisms for ONE scalar-float RIP-relative operand of the game's .text section:
//   * WriteConst (preferred): the constant the instruction reads is rewritten in place with one aligned 4-byte
//     compare-exchange - ONLY when a full .text scan finds exactly one RIP-relative instruction referencing it
//     (pooled constants shared with other code are never written). No code byte changes.
//   * RedirectDisp: the instruction's disp32 is redirected to a UTPP-owned float (within +-2 GB of the image) with one
//     interlocked 8- or 16-byte compare-exchange of the aligned block containing it (refused if no such block exists).
// Site acceptance: pattern (hex, "??" wildcards) matches EXACTLY ONCE in .text; the instruction decodes as a whitelisted
// scalar-float SSE/AVX m32 [rip+disp32] form; the constant equals the expected value.
//
// 2.31 built-in site (2026-10-07, "HeadGuard site"): the CameraSystem "PlayerAimUpdateTick"
// per-listener update (RVA 0x529274 in 2.31) converts two FixedPoint WorldPositions (x 1/131072) - the active camera
// (gameCameraSystem+0x4C0) and a tracked placed component - into metres, computes the squared distance and classifies
// it with   comiss d2, [0.2025]  (= 0.45 m squared)  and  comiss d2, [0.04]  (= 0.2 m squared);  the two resulting
// flags are stored on the listener and applied to two component groups. A threshold of 0 makes every distance
// "far" (the state used when the camera is > 0.45 m away), i.e. the near-camera hide/swap never triggers.
// In 2.31 the 0.2025 constant is referenced by this instruction only and its disp32 straddles a 16-byte boundary,
// so the built-in site uses WriteConst.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <intrin.h>

namespace utpp::hg
{
struct Pattern
{
    std::vector<int16_t> bytes; // -1 = wildcard
};

inline int HexNibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

// "F3 0F 10 05 ?? ?? ?? ??" (single '?' also accepted). 4..64 tokens, at least 4 fixed bytes.
inline bool ParsePattern(const std::string& aText, Pattern& aOut)
{
    Pattern p;
    size_t i = 0;
    int fixed = 0;
    while (i < aText.size())
    {
        while (i < aText.size() && (aText[i] == ' ' || aText[i] == '\t'))
            ++i;
        if (i >= aText.size())
            break;
        size_t j = i;
        while (j < aText.size() && aText[j] != ' ' && aText[j] != '\t')
            ++j;
        const std::string tok = aText.substr(i, j - i);
        i = j;
        if (tok == "?" || tok == "??")
        {
            p.bytes.push_back(-1);
            continue;
        }
        if (tok.size() != 2)
            return false;
        const int hi = HexNibble(tok[0]), lo = HexNibble(tok[1]);
        if (hi < 0 || lo < 0)
            return false;
        p.bytes.push_back(static_cast<int16_t>(hi * 16 + lo));
        ++fixed;
    }
    if (p.bytes.size() < 4 || p.bytes.size() > 64 || fixed < 4)
        return false;
    aOut = p;
    return true;
}

// Match offsets written to aOut (at most aMaxOut); returns the number written. No allocation (usable inside __try).
inline size_t ScanRaw(const uint8_t* aData, size_t aSize, const int16_t* aPat, size_t aN, size_t* aOut, size_t aMaxOut)
{
    if (!aData || !aPat || aN == 0 || aSize < aN || aMaxOut == 0)
        return 0;
    size_t anchor = 0; // first fixed byte
    while (anchor < aN && aPat[anchor] < 0)
        ++anchor;
    if (anchor == aN)
        return 0;
    const uint8_t first = static_cast<uint8_t>(aPat[anchor]);
    size_t count = 0;
    for (size_t i = 0; i + aN <= aSize; ++i)
    {
        if (aData[i + anchor] != first)
            continue;
        bool ok = true;
        for (size_t k = 0; k < aN && ok; ++k)
            ok = aPat[k] < 0 || aData[i + k] == static_cast<uint8_t>(aPat[k]);
        if (ok)
        {
            aOut[count++] = i;
            if (count >= aMaxOut)
                break;
        }
    }
    return count;
}

// All match offsets (stops after aMaxHits).
inline std::vector<size_t> Scan(const uint8_t* aData, size_t aSize, const Pattern& aP, size_t aMaxHits)
{
    std::vector<size_t> hits(aMaxHits);
    hits.resize(ScanRaw(aData, aSize, aP.bytes.data(), aP.bytes.size(), hits.data(), aMaxHits));
    return hits;
}

struct RipInsn
{
    size_t len = 0;     // instruction length
    size_t dispPos = 0; // offset of disp32 from the instruction start
    int32_t disp = 0;
    uint8_t opcode = 0;
    uint8_t reg = 0; // ModRM.reg (+8 with REX.R / VEX.R)
    bool vex = false;
};

// Decodes the instruction at aCode (aAvail readable bytes). Only scalar-float m32 RIP-relative forms are accepted.
inline bool DecodeSseRip(const uint8_t* aCode, size_t aAvail, RipInsn& aOut)
{
    if (!aCode || aAvail < 7)
        return false;
    size_t p = 0;
    bool f3 = false;
    uint8_t opcode = 0;
    bool vex = false;
    bool rexR = false;
    if (aCode[0] == 0xC5)
    {
        // 2-byte VEX: C5 [R vvvv L pp] opcode modrm. pp: 00 none, 10 F3. L must be 0 (scalar).
        const uint8_t v = aCode[1];
        const uint8_t pp = v & 0x03;
        const bool l = (v & 0x04) != 0;
        if (l || (pp != 0x00 && pp != 0x02))
            return false;
        f3 = pp == 0x02;
        rexR = (v & 0x80) == 0; // VEX.R is stored inverted
        opcode = aCode[2];
        p = 3;
        vex = true;
    }
    else
    {
        if (aCode[p] == 0xF3)
        {
            f3 = true;
            ++p;
        }
        if (aCode[p] >= 0x40 && aCode[p] <= 0x4F)
        {
            if (aCode[p] & 0x08) // REX.W: not a scalar-float form we accept
                return false;
            rexR = (aCode[p] & 0x04) != 0;
            ++p;
        }
        if (aCode[p] != 0x0F)
            return false;
        opcode = aCode[p + 1];
        p += 2;
    }
    bool allowed = false;
    if (f3)
        allowed = opcode == 0x10 || opcode == 0x58 || opcode == 0x59 || opcode == 0x5C || opcode == 0x5D ||
                  opcode == 0x5E || opcode == 0x5F;
    else
        allowed = opcode == 0x2E || opcode == 0x2F;
    if (!allowed)
        return false;
    if (p + 5 > aAvail)
        return false;
    const uint8_t modrm = aCode[p];
    if ((modrm & 0xC7) != 0x05) // mod 00, rm 101 = [rip + disp32]
        return false;
    ++p;
    int32_t disp = 0;
    std::memcpy(&disp, aCode + p, 4);
    aOut.dispPos = p;
    aOut.len = p + 4;
    aOut.disp = disp;
    aOut.opcode = opcode;
    aOut.reg = static_cast<uint8_t>(((modrm >> 3) & 7) + (rexR ? 8 : 0));
    aOut.vex = vex;
    return true;
}

inline bool FloatMatches(float aActual, float aExpected)
{
    if (!std::isfinite(aActual) || !std::isfinite(aExpected))
        return false;
    uint32_t a, e;
    std::memcpy(&a, &aActual, 4);
    std::memcpy(&e, &aExpected, 4);
    if (a == e)
        return true;
    const float scale = std::fmax(std::fabs(aExpected), 1e-6f);
    return std::fabs(aActual - aExpected) <= 1e-6f * scale;
}

// rel32 for an instruction at aInsnAddr (length aLen) to reach aTarget; false when out of range.
inline bool Rel32(uint64_t aInsnAddr, size_t aLen, uint64_t aTarget, int32_t& aOut)
{
    const int64_t rel = static_cast<int64_t>(aTarget) - static_cast<int64_t>(aInsnAddr + aLen);
    if (rel < INT32_MIN || rel > INT32_MAX)
        return false;
    aOut = static_cast<int32_t>(rel);
    return true;
}

// Writes 4 bytes at aAddr only if they currently equal aExpectOld, with one interlocked compare-exchange of the aligned
// 8-byte (or, when the 4 bytes straddle, 16-byte) block that contains them. The memory must be writable.
enum class PatchResult
{
    Ok = 0,
    Mismatch = 1,  // current bytes != expected old
    Unaligned = 2, // no aligned 8/16-byte block contains the 4 bytes
    Raced = 3,     // compare-exchange lost repeatedly
};

inline PatchResult Patch4(uint8_t* aAddr, uint32_t aExpectOld, uint32_t aNew)
{
    const uintptr_t a = reinterpret_cast<uintptr_t>(aAddr);
    const uintptr_t q = a & ~static_cast<uintptr_t>(7);
    const unsigned off8 = static_cast<unsigned>(a - q);
    if (off8 <= 4)
    {
        auto* qp = reinterpret_cast<volatile long long*>(q);
        for (int attempt = 0; attempt < 8; ++attempt)
        {
            const long long cur = *qp;
            uint32_t now;
            std::memcpy(&now, reinterpret_cast<const uint8_t*>(&cur) + off8, 4);
            if (now != aExpectOld)
                return PatchResult::Mismatch;
            long long next = cur;
            std::memcpy(reinterpret_cast<uint8_t*>(&next) + off8, &aNew, 4);
            if (_InterlockedCompareExchange64(qp, next, cur) == cur)
                return PatchResult::Ok;
        }
        return PatchResult::Raced;
    }
    const uintptr_t o = a & ~static_cast<uintptr_t>(15);
    const unsigned off16 = static_cast<unsigned>(a - o);
    if (off16 > 12)
        return PatchResult::Unaligned;
    auto* op = reinterpret_cast<long long*>(o);
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        alignas(16) long long cur[2];
        cur[0] = reinterpret_cast<volatile long long*>(op)[0];
        cur[1] = reinterpret_cast<volatile long long*>(op)[1];
        uint32_t now;
        std::memcpy(&now, reinterpret_cast<const uint8_t*>(cur) + off16, 4);
        if (now != aExpectOld)
            return PatchResult::Mismatch;
        alignas(16) long long next[2] = {cur[0], cur[1]};
        std::memcpy(reinterpret_cast<uint8_t*>(next) + off16, &aNew, 4);
        alignas(16) long long cmp[2] = {cur[0], cur[1]};
        if (_InterlockedCompareExchange128(op, next[1], next[0], cmp))
            return PatchResult::Ok;
    }
    return PatchResult::Raced;
}

// One interlocked 4-byte compare-exchange of an aligned 32-bit value (WriteConst). The memory must be writable.
inline PatchResult PatchConst4(uint8_t* aAddr, uint32_t aExpectOld, uint32_t aNew)
{
    if ((reinterpret_cast<uintptr_t>(aAddr) & 3) != 0)
        return PatchResult::Unaligned;
    long expect, next;
    std::memcpy(&expect, &aExpectOld, 4);
    std::memcpy(&next, &aNew, 4);
    const long prev = _InterlockedCompareExchange(reinterpret_cast<volatile long*>(aAddr), next, expect);
    return prev == expect ? PatchResult::Ok : PatchResult::Mismatch;
}

// True when the 4 disp bytes at aAddr lie inside an aligned 8- or 16-byte block (RedirectDisp is possible).
inline bool DispCasPossible(uintptr_t aAddr)
{
    return (aAddr & 7) <= 4 || (aAddr & 15) <= 12;
}

// True when a scalar-float m32 instruction (DecodeSseRip whitelist) has its ModRM at aText[aModrm]: such an access
// reads exactly 4 bytes, so a target 4+ bytes below the constant cannot overlap it.
inline bool ScalarM32At(const uint8_t* aText, size_t aSize, size_t aModrm)
{
    for (size_t off = 2; off <= 4; ++off)
    {
        if (aModrm < off)
            break;
        RipInsn ri;
        if (DecodeSseRip(aText + aModrm - off, aSize - (aModrm - off), ri) && ri.dispPos == off + 1)
            return true;
    }
    return false;
}

// Number of RIP-relative operands in aText that may touch the 4 bytes at aTargetRva (stops after aMax + 1).
// A candidate is any byte that looks like a ModRM with mod=00 rm=101 followed by disp32. Counted:
//   * exact references (instruction ends after the disp, or after a 1-byte immediate);
//   * accesses starting 1..3 bytes above / below the target (always overlapping);
//   * accesses starting 4..15 bytes below the target (a 16-byte vector load would cover it) unless the instruction
//     decodes as a scalar 4-byte access.
// Over-counting (random byte matches) is possible and only makes the caller refuse (fail-closed); a reference that
// is not RIP-relative (e.g. via a pointer table) is not counted.
inline size_t CountRipRefs(const uint8_t* aText, size_t aSize, uint64_t aTextRva, uint64_t aTargetRva, size_t aMax)
{
    if (!aText || aSize < 5)
        return 0;
    size_t count = 0;
    for (size_t i = 0; i + 5 <= aSize; ++i)
    {
        if ((aText[i] & 0xC7) != 0x05)
            continue;
        int32_t disp;
        std::memcpy(&disp, aText + i + 1, 4);
        // rel = aTargetRva - (target of an instruction without immediate)
        const int64_t rel = static_cast<int64_t>(aTargetRva) - static_cast<int64_t>(aTextRva + i + 5) - disp;
        bool hit = rel >= -3 && rel <= 3;
        if (!hit && rel >= 4 && rel <= 15)
            hit = !ScalarM32At(aText, aSize, i);
        if (hit)
        {
            if (++count > aMax)
                break;
        }
    }
    return count;
}

// ---------------------------------------------------------------------------------------------
// Built-in near-camera site (see the header comment). Allocation-free (usable inside __try).
constexpr float kNearSqExpected = 0.2025f;           // (0.45 m)^2: the threshold that is patched
constexpr float kInnerSqExpected = 0.04f;            // (0.2 m)^2: second compare, validated but never written
constexpr float kFixedPointScale = 1.0f / 131072.0f; // WorldPosition FixedPoint -> metres
constexpr float kDefaultTppValue = 0.0f;             // squared metres; 0 = never "near" while third person
constexpr size_t kScaleWindow = 0x100;               // bytes before the compare searched for the scale load
constexpr size_t kSecondCompareDelta = 16;           // second comiss starts 16 bytes after the first

// Own signatures, tightest first; wildcards cover registers, rel8 branches and disp32 so a rebuilt executable with
// shifted addresses still matches. insnOffset = first comiss relative to the match.
inline constexpr int16_t kNearSig0[] = {0xF3, 0x0F, 0x58, -1,   0xF3, 0x0F, 0x59, -1,   0xF3, 0x0F, 0x58, -1,
                                        0x0F, 0x2F, -1,   -1,   -1,   -1,   -1,   0x72, -1,   -1,   -1,   -1,
                                        0xB2, 0x01, 0xEB, -1,   0x0F, 0x2F, -1,   -1,   -1,   -1,   -1,   0x77};
inline constexpr int16_t kNearSig1[] = {0x0F, 0x2F, -1, -1, -1, -1, -1, 0x72, -1, -1, -1, -1,
                                        -1,   -1,   -1, -1, 0x0F, 0x2F, -1, -1, -1, -1, -1, 0x77};

struct NearSig
{
    const int16_t* bytes;
    size_t n;
    size_t insnOffset;
    const char* name;
};

inline constexpr NearSig kNearSigs[] = {
    {kNearSig0, sizeof(kNearSig0) / sizeof(kNearSig0[0]), 12, "dist2+compare"},
    {kNearSig1, sizeof(kNearSig1) / sizeof(kNearSig1[0]), 0, "compare-pair"},
};
constexpr int kNearSigCount = static_cast<int>(sizeof(kNearSigs) / sizeof(kNearSigs[0]));

// Read-only view of the loaded (or file-mapped) image. readF32 must return false for RVAs outside a readable,
// non-executable section of the image.
struct ImageView
{
    const uint8_t* text = nullptr;
    size_t textSize = 0;
    uint64_t textRva = 0;
    void* ctx = nullptr;
    bool (*readF32)(void* aCtx, uint64_t aRva, float& aOut) = nullptr;
};

struct NearSite
{
    int sig = -1;          // index into kNearSigs
    size_t insnOff = 0;    // first comiss, offset into .text
    RipInsn insn;          // decoded first comiss
    uint64_t constRva = 0; // 0.2025
    float constValue = 0.0f;
    uint64_t innerRva = 0; // 0.04
    size_t constRefs = 0;  // RIP-relative accesses that may touch constRva (CountRipRefs); WriteConst needs 1
    uint64_t scaleRva = 0; // 1/131072 load before the compare
};

inline bool ReadView(const ImageView& aV, uint64_t aRva, float& aOut)
{
    return aV.readF32 && aV.readF32(aV.ctx, aRva, aOut);
}

// Validates the instruction at aInsnOff as the near-camera compare. aWhy receives a static string on failure.
inline bool ValidateNearCompare(const ImageView& aV, size_t aInsnOff, NearSite& aOut, const char*& aWhy)
{
    if (aInsnOff + kSecondCompareDelta + 16 > aV.textSize)
    {
        aWhy = "compare too close to the end of .text";
        return false;
    }
    const uint8_t* c = aV.text + aInsnOff;
    RipInsn a, b;
    if (!DecodeSseRip(c, 16, a) || a.vex || a.opcode != 0x2F)
    {
        aWhy = "first instruction is not comiss xmm, [rip+disp32]";
        return false;
    }
    if (c[a.len] != 0x72)
    {
        aWhy = "first compare is not followed by jb";
        return false;
    }
    if (!DecodeSseRip(c + kSecondCompareDelta, 16, b) || b.vex || b.opcode != 0x2F || b.reg != a.reg ||
        c[kSecondCompareDelta + b.len] != 0x77)
    {
        aWhy = "second compare (comiss same register, ja) missing";
        return false;
    }
    const uint64_t insnRva = aV.textRva + aInsnOff;
    const uint64_t constRva = static_cast<uint64_t>(static_cast<int64_t>(insnRva + a.len) + a.disp);
    const uint64_t innerRva =
        static_cast<uint64_t>(static_cast<int64_t>(insnRva + kSecondCompareDelta + b.len) + b.disp);
    float v1 = 0.0f, v2 = 0.0f;
    if (!ReadView(aV, constRva, v1) || !ReadView(aV, innerRva, v2))
    {
        aWhy = "compare operands are not readable data constants of the image";
        return false;
    }
    if (!FloatMatches(v1, kNearSqExpected))
    {
        aWhy = "threshold constant is not 0.2025 (changed build, or already patched by another mod)";
        return false;
    }
    if (!FloatMatches(v2, kInnerSqExpected))
    {
        aWhy = "inner constant is not 0.04";
        return false;
    }
    // The compared value is a squared distance between FixedPoint world positions: a movss of 1/131072 precedes it.
    uint64_t scaleRva = 0;
    const size_t from = aInsnOff > kScaleWindow ? aInsnOff - kScaleWindow : 0;
    for (size_t k = from; k < aInsnOff && !scaleRva; ++k)
    {
        if (aV.text[k] != 0xF3 && aV.text[k] != 0xC5)
            continue;
        RipInsn m;
        if (!DecodeSseRip(aV.text + k, aInsnOff - k, m) || m.opcode != 0x10 || k + m.len > aInsnOff)
            continue;
        const uint64_t t = static_cast<uint64_t>(static_cast<int64_t>(aV.textRva + k + m.len) + m.disp);
        float sv = 0.0f;
        if (ReadView(aV, t, sv) && sv == kFixedPointScale)
            scaleRva = t;
    }
    if (!scaleRva)
    {
        aWhy = "no FixedPoint scale (1/131072) load before the compare";
        return false;
    }
    aOut.insnOff = aInsnOff;
    aOut.insn = a;
    aOut.constRva = constRva;
    aOut.constValue = v1;
    aOut.innerRva = innerRva;
    aOut.scaleRva = scaleRva;
    aOut.constRefs = CountRipRefs(aV.text, aV.textSize, aV.textRva, constRva, 4);
    return true;
}

// Scans the built-in signatures (tightest first). Success needs a unique match and a full validation.
inline bool FindNearCameraSite(const ImageView& aV, NearSite& aOut, const char*& aWhy)
{
    aWhy = "no signature matched";
    if (!aV.text || aV.textSize == 0)
    {
        aWhy = "no .text";
        return false;
    }
    for (int i = 0; i < kNearSigCount; ++i)
    {
        const NearSig& sg = kNearSigs[i];
        size_t hits[2] = {};
        const size_t n = ScanRaw(aV.text, aV.textSize, sg.bytes, sg.n, hits, 2);
        if (n == 0)
            continue;
        if (n > 1)
        {
            aWhy = "signature not unique";
            continue;
        }
        NearSite site;
        const char* why = nullptr;
        if (!ValidateNearCompare(aV, hits[0] + sg.insnOffset, site, why))
        {
            aWhy = why;
            continue;
        }
        site.sig = i;
        aOut = site;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------
// Site description and activation decision.
struct Site
{
    std::string pattern;
    int32_t insnOffset = 0; // instruction start relative to the pattern match
    float expected = 0.0f;  // value of the original constant
    float value = 0.0f;     // value used while active (third person)
};

inline bool ValidSiteValue(float aV)
{
    return std::isfinite(aV) && std::fabs(aV) <= 1.0e6f;
}

enum Reason : uint32_t
{
    kNoSite = 1u << 0,       // no proven site (built-in signature FAILED / skipped, no runtime candidate)
    kDisabled = 1u << 1,     // user setting off (default)
    kKilled = 1u << 2,       // session kill switch
    // bit 3 unused
    kNotTpp = 1u << 4,       // heartbeat stale or first person (FPP needs the vanilla threshold)
    kKillFile = 1u << 5,     // utpp_headguard.disable
    kNotValidated = 1u << 6, // UTPP's 2.31 validation failed
    kPatchFailed = 1u << 7,  // a write failed (latched for the session)
};

struct Inputs
{
    bool siteReady = false;
    bool enabled = false;
    bool killed = false;
    bool tpp = false;
    bool killFile = false;
    bool validated = false;
    bool patchFailed = false;
};

inline uint32_t Reasons(const Inputs& aIn)
{
    uint32_t r = 0;
    if (!aIn.siteReady)
        r |= kNoSite;
    if (!aIn.enabled)
        r |= kDisabled;
    if (aIn.killed)
        r |= kKilled;
    if (!aIn.tpp)
        r |= kNotTpp;
    if (aIn.killFile)
        r |= kKillFile;
    if (!aIn.validated)
        r |= kNotValidated;
    if (aIn.patchFailed)
        r |= kPatchFailed;
    return r;
}

inline bool WantPatched(const Inputs& aIn)
{
    return Reasons(aIn) == 0;
}
} // namespace utpp::hg

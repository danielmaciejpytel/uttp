// UTPP native locomotion input (vehicle approach walk): pure logic, no Windows / RED4ext dependency (offline-testable).
//
// What this covers: the function-level hook on the engine's per-frame player locomotion input reader (2.5 exe RVA 0x6485E0,
// called from exactly one place, 0x6491B2). The reader fetches MoveX / MoveY from the input snapshot, clamps the vector to unit
// length, stores it at this+0x90 / this+0x94, writes the anim variables LocomotionInputDirectionX / Y and mirrors the vector into
// a per-player record (+0xAC / +0xB0). The script natives that the move override feeds (GetActionValue / GetInputHeading) never
// pass through it, which is why V did not move during the first approach test (the PSM and the motion matching followed the
// override, the capsule did not).
//
// This header holds what must be provable offline:
//   * VerifyReader: the byte-level proof of the reader, its single caller, the shared input lookup (proven from the already
//     hooked GetActionValue handler) and the anim variable block. The hook is attached only when every required check passes
//     (fail closed). The caller supplies a byte reader (RVA -> bytes) so tests can feed the real exe or mutated copies.
//   * Arming: a time-limited arm flag. The detour overwrites the hardware vector only while armed AND the move override is
//     live; a stale arm can never stick (TTL <= kMaxArmTtlSec).
//   * ClampUnit: the unit-length clamp of the override vector (same rule as the reader's own clamp).
//
// Evidence status: the reader and its call sites are OBSERVED in the 2.5 exe
// (sha256 d36e74c4...fab4). That the capsule translation consumes +0x90 / +0x94 (or the anim variables) is NOT proven: the
// first armed run decides it (dpos in the approach tick log). The +0xAC / +0xB0 record is not rewritten (reaching it needs the
// owner's vtable chain +0x28/+0x1E0 and a ref-counted handle copied from locals of the reader; not replicated).

#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace utpp::freemove::nativein
{
// ---- 2.5 retail exe (RVAs), all OBSERVED in the 2.50 Cyberpunk2077.exe ----
constexpr uint64_t kRvaHandlerActionValue = 0x648778; // GetActionValue native handler (anchor, proven by the slot swap)
constexpr uint64_t kRvaReader = 0x6485E0;             // per-frame locomotion input reader (hook target)
constexpr uint64_t kRvaReaderCall = 0x6491B2;         // its only caller (E8 rel32)
constexpr uint64_t kRvaLookup = 0x648ECC;             // input snapshot lookup (key -> float), NOT hooked
constexpr uint64_t kRvaAnimKeyX = 0x41D74A0;          // global CName hash LocomotionInputDirectionX
constexpr uint64_t kRvaAnimKeyY = 0x41D74A8;          // global CName hash LocomotionInputDirectionY
constexpr uint64_t kRvaAnimSet = 0xB12138;            // bool(uint64 key, float value [xmm1], char flag [r8b], void* container [r9])
constexpr uint32_t kOffInputX = 0x90;                 // reader this+0x90: unit-clamped MoveX
constexpr uint32_t kOffInputY = 0x94;                 // reader this+0x94: unit-clamped MoveY
constexpr uint32_t kOffSub = 0x30;                    // embedded sub-object (vtable slot 0 / 1 used by the reader)
constexpr uint32_t kOffSubField = 0x10;               // slot-0 result +0x10 != 0 selects the snapshot (stick) branch
constexpr uint32_t kOffHeadObj = 0x38;                // slot-0 result +0x38: the referenced object (null = the reader bails out)
constexpr uint32_t kOffHeadRef = 0x40;                // slot-0 result +0x40: ref block whose first int is the strong count (0 = bail)
constexpr uint32_t kOffContainerPtr = 0x58;           // slot-1 result +0x58 -> anim variable block
constexpr uint32_t kOffContainerAdd = 0xC060;         // + this = the container passed to kRvaAnimSet
constexpr uint64_t kHashMoveX = 0xE386BCF96B1AB2A4ull;
constexpr uint64_t kHashMoveY = 0xE386BDF96B1AB457ull;

constexpr double kMaxArmTtlSec = 0.5;

// ---- byte proof ----
struct Pat
{
    uint64_t rva;
    const uint8_t* bytes;
    size_t len;
    int rel0Off; // offset of a rel32 / rip-rel32 (ignored in the byte compare, checked against rel0Target), -1 = none
    uint64_t rel0Target;
    int rel1Off;
    uint64_t rel1Target;
};

// Reader prologue (mov rax,rsp; mov [rax+8],rbx; mov [rax+10h],rbp; mov [rax+18h],rsi; push rdi; sub rsp,40h): the first 15
// bytes have no rip-relative operand and no branch, so a standard trampoline hook is valid.
constexpr uint8_t kReaderPrologue[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68,
                                       0x10, 0x48, 0x89, 0x70, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x40};
// Reader early-out (0x6485F4..0x648656): sub = this+0x30; head = sub.vtable[0](sub); bail to the exit when [head+0x40] is null,
// the strong count *[head+0x40] is 0 (or, after the ref is taken, [head+0x38] is null). The detour runs after the reader has
// released its own ref, so it re-applies this test before it writes anything (no write on frames the reader skipped).
constexpr uint8_t kGuardA[] = {0x48, 0x8D, 0x59, 0x30, 0x0F, 0x29, 0x70, 0xE8, 0x48, 0x8B, 0x03, 0x48, 0x8B, 0xF1, 0x48, 0x8B,
                               0xCB, 0xFF, 0x10, 0x0F, 0x57, 0xC0, 0x4C, 0x8B, 0xC0, 0xF3, 0x0F, 0x7F, 0x44, 0x24, 0x20, 0x48,
                               0x8B, 0x40, 0x40, 0x48, 0x85, 0xC0, 0x0F, 0x84, 0x18, 0x01, 0x00, 0x00, 0x8B, 0x00, 0x85, 0xC0,
                               0x0F, 0x84, 0x0E, 0x01, 0x00, 0x00};
constexpr uint8_t kGuardB[] = {0x49, 0x8B, 0x50, 0x40, 0x8D, 0x48, 0x01, 0xF0, 0x0F, 0xB1, 0x0A, 0x49, 0x8B, 0x40, 0x40, 0x0F,
                               0x85, 0x2A, 0x01, 0x00, 0x00, 0x48, 0x89, 0x44, 0x24, 0x28, 0x49, 0x8B, 0x40, 0x38, 0x48, 0x89,
                               0x44, 0x24, 0x20, 0x48, 0x85, 0xC0, 0x0F, 0x84, 0xE2, 0x00, 0x00, 0x00};
// Only caller: mov rcx,rsi / call reader / mov rcx,[rsi+0D8h] / test rcx,rcx. After the call the caller uses callee-saved RSI
// only, so a normal detour (volatile registers free) is safe here.
constexpr uint8_t kCallerSite[] = {0x48, 0x8B, 0xCE, 0xE8, 0, 0, 0, 0, 0x48, 0x8B, 0x8E, 0xD8, 0x00, 0x00, 0x00, 0x48, 0x85, 0xC9};
// In the GetActionValue handler (already the proven anchor): mov rcx,[rdi+98h]; test rcx,rcx; jz; mov rdx,[rsp+30h]; call lookup.
constexpr uint8_t kHandlerLookupCall[] = {0x48, 0x8B, 0x8F, 0x98, 0x00, 0x00, 0x00, 0x48, 0x85, 0xC9,
                                          0x74, 0x0A, 0x48, 0x8B, 0x54, 0x24, 0x30, 0xE8, 0, 0, 0, 0};
constexpr uint8_t kLookupPrologue[] = {0x48, 0x8B, 0x41, 0x08, 0x8B, 0x49, 0x14, 0x48, 0x8D, 0x0C, 0x49, 0x48, 0xC1, 0xE1, 0x04};
// movabs rdx,MoveX / mov rcx,r8 / call lookup
constexpr uint8_t kMoveXSite[] = {0x48, 0xBA, 0xA4, 0xB2, 0x1A, 0x6B, 0xF9, 0xBC, 0x86, 0xE3, 0x49, 0x8B, 0xC8, 0xE8, 0, 0, 0, 0};
// lea rdi,[rsi+90h] / movabs rdx,MoveY / movss [rdi],xmm2 / mov rcx,r8 / call lookup
constexpr uint8_t kMoveYSite[] = {0x48, 0x8D, 0xBE, 0x90, 0x00, 0x00, 0x00, 0x48, 0xBA, 0x57, 0xB4, 0x1A, 0x6B, 0xF9, 0xBD, 0x86,
                                  0xE3, 0xF3, 0x0F, 0x11, 0x17, 0x49, 0x8B, 0xC8, 0xE8, 0, 0, 0, 0};
// movss [rsi+94h],xmm0
constexpr uint8_t kStoreY[] = {0xF3, 0x0F, 0x11, 0x86, 0x94, 0x00, 0x00, 0x00};
// stick branch selector: mov rax,[rbx]; mov rcx,rbx; call [rax]; mov r8,[rax+10h]; test r8,r8; jz fallback
constexpr uint8_t kBranch[] = {0x48, 0x8B, 0x03, 0x48, 0x8B, 0xCB, 0xFF, 0x10, 0x4C, 0x8B, 0x40, 0x10, 0x4D, 0x85, 0xC0, 0x0F, 0x84};
// anim variable X: call [rax+8] (owner) / movss xmm1,[rdi] / mov r8b,1 / mov rcx,[keyX] / mov r9,[rax+58h]; add r9,0C060h / call set
constexpr uint8_t kAnimX[] = {0x48, 0x8B, 0x03, 0x48, 0x8B, 0xCB, 0xFF, 0x50, 0x08, 0xF3, 0x0F, 0x10, 0x0F, 0x41, 0xB0, 0x01,
                              0x48, 0x8B, 0x0D, 0, 0, 0, 0, 0x4C, 0x8B, 0x48, 0x58, 0x49, 0x81, 0xC1, 0x60, 0xC0, 0x00, 0x00,
                              0xE8, 0, 0, 0, 0};
constexpr uint8_t kAnimY[] = {0x48, 0x8B, 0x03, 0x48, 0x8B, 0xCB, 0xFF, 0x50, 0x08, 0xF3, 0x0F, 0x10, 0x8E, 0x94, 0x00, 0x00,
                              0x00, 0x41, 0xB0, 0x01, 0x48, 0x8B, 0x0D, 0, 0, 0, 0, 0x4C, 0x8B, 0x48, 0x58, 0x49, 0x81, 0xC1,
                              0x60, 0xC0, 0x00, 0x00, 0xE8, 0, 0, 0, 0};
constexpr uint8_t kAnimSetPrologue[] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x41, 0x8B, 0x91, 0x00, 0x0C, 0x00, 0x00, 0x48, 0x8B, 0xC1};

inline constexpr Pat kPatReader{kRvaReader, kReaderPrologue, sizeof(kReaderPrologue), -1, 0, -1, 0};
inline constexpr Pat kPatGuardA{0x6485F4, kGuardA, sizeof(kGuardA), -1, 0, -1, 0};
inline constexpr Pat kPatGuardB{0x64862A, kGuardB, sizeof(kGuardB), -1, 0, -1, 0};
inline constexpr Pat kPatCaller{kRvaReaderCall - 3, kCallerSite, sizeof(kCallerSite), 4, kRvaReader, -1, 0};
inline constexpr Pat kPatHandlerCall{0x6487CF, kHandlerLookupCall, sizeof(kHandlerLookupCall), 18, kRvaLookup, -1, 0};
inline constexpr Pat kPatLookup{kRvaLookup, kLookupPrologue, sizeof(kLookupPrologue), -1, 0, -1, 0};
inline constexpr Pat kPatMoveX{0x64866B, kMoveXSite, sizeof(kMoveXSite), 14, kRvaLookup, -1, 0};
inline constexpr Pat kPatMoveY{0x648680, kMoveYSite, sizeof(kMoveYSite), 25, kRvaLookup, -1, 0};
inline constexpr Pat kPatStoreY{0x6486A8, kStoreY, sizeof(kStoreY), -1, 0, -1, 0};
inline constexpr Pat kPatBranch{0x648656, kBranch, sizeof(kBranch), -1, 0, -1, 0};
inline constexpr Pat kPatAnimX{0x6486C1, kAnimX, sizeof(kAnimX), 19, kRvaAnimKeyX, 35, kRvaAnimSet};
inline constexpr Pat kPatAnimY{0x6486E8, kAnimY, sizeof(kAnimY), 23, kRvaAnimKeyY, 39, kRvaAnimSet};
inline constexpr Pat kPatAnimSet{kRvaAnimSet, kAnimSetPrologue, sizeof(kAnimSetPrologue), -1, 0, -1, 0};

struct Verdict
{
    bool ok = false;     // the reader hook may be attached
    bool animOk = false; // the anim variable rewrite (kRvaAnimSet call chain) is proven as well
    const char* why = "not run";
};

// read(rva, dst, len) -> bool copies image bytes. A failed read or any mismatch fails the pattern.
template<typename Read>
bool CheckPat(const Pat& aPat, Read&& aRead)
{
    uint8_t buf[64];
    if (aPat.len == 0 || aPat.len > sizeof(buf) || !aRead(aPat.rva, buf, aPat.len))
        return false;
    for (size_t i = 0; i < aPat.len; ++i)
    {
        const bool inRel0 = aPat.rel0Off >= 0 && i >= static_cast<size_t>(aPat.rel0Off) && i < static_cast<size_t>(aPat.rel0Off) + 4;
        const bool inRel1 = aPat.rel1Off >= 0 && i >= static_cast<size_t>(aPat.rel1Off) && i < static_cast<size_t>(aPat.rel1Off) + 4;
        if (!inRel0 && !inRel1 && buf[i] != aPat.bytes[i])
            return false;
    }
    auto relOk = [&](int aOff, uint64_t aTarget)
    {
        if (aOff < 0)
            return true;
        int32_t disp = 0;
        std::memcpy(&disp, buf + aOff, sizeof(disp));
        const int64_t next = static_cast<int64_t>(aPat.rva) + aOff + 4;
        return next + disp == static_cast<int64_t>(aTarget);
    };
    return relOk(aPat.rel0Off, aPat.rel0Target) && relOk(aPat.rel1Off, aPat.rel1Target);
}

// aHandlerRva = RVA of the GetActionValue handler found by the slot proof (the anchor); aHashX / aHashY = the CName hashes the
// game resolved for MoveX / MoveY (must equal the constants baked into the call sites).
template<typename Read>
Verdict VerifyReader(uint64_t aHandlerRva, uint64_t aHashX, uint64_t aHashY, Read&& aRead)
{
    Verdict v;
    if (aHandlerRva != kRvaHandlerActionValue)
    {
        v.why = "GetActionValue handler is not at the expected RVA (different game build)";
        return v;
    }
    if (aHashX != kHashMoveX || aHashY != kHashMoveY)
    {
        v.why = "MoveX / MoveY CName hash differs from the call-site constants";
        return v;
    }
    struct Step
    {
        const Pat* pat;
        const char* why;
    };
    const Step required[] = {
        {&kPatHandlerCall, "anchor handler lookup call does not target the expected lookup"},
        {&kPatLookup, "lookup prologue mismatch"},
        {&kPatReader, "reader prologue mismatch"},
        {&kPatGuardA, "reader early-out (head / ref block test) mismatch"},
        {&kPatGuardB, "reader early-out (ref take / object test) mismatch"},
        {&kPatCaller, "reader caller site mismatch (not a single known caller)"},
        {&kPatMoveX, "reader MoveX lookup site mismatch"},
        {&kPatMoveY, "reader MoveY lookup site mismatch"},
        {&kPatStoreY, "reader +0x94 store mismatch"},
    };
    for (const Step& s : required)
    {
        if (!CheckPat(*s.pat, aRead))
        {
            v.why = s.why;
            return v;
        }
    }
    v.ok = true;
    v.why = "reader proven";
    const Step anim[] = {
        {&kPatBranch, "stick branch selector mismatch"},
        {&kPatAnimX, "anim variable X block mismatch"},
        {&kPatAnimY, "anim variable Y block mismatch"},
        {&kPatAnimSet, "anim variable setter prologue mismatch"},
    };
    for (const Step& s : anim)
    {
        if (!CheckPat(*s.pat, aRead))
        {
            v.why = s.why; // reader hook still allowed (hardware vector overwrite only); anim rewrite off
            return v;
        }
    }
    v.animOk = true;
    return v;
}

// The reader's early-out test on the slot-0 result: ref block present, strong count non-zero, object present. aCount is the
// strong count read from the ref block (the caller reads memory; this keeps the rule testable offline).
constexpr bool ReaderWouldRun(bool aHasRefBlock, int32_t aStrongCount, bool aHasObject)
{
    return aHasRefBlock && aStrongCount != 0 && aHasObject;
}

// ---- arming ----
// Armed only on the game thread by the approach step (every Running step refreshes it), dropped by every other result,
// cancel and begin. Read in the detour: one relaxed atomic load.
class Arming
{
public:
    // False (and disarmed) on a non-finite time or a non-positive TTL.
    bool Arm(double aNow, double aTtlSec)
    {
        if (!std::isfinite(aNow) || !std::isfinite(aTtlSec) || !(aTtlSec > 0.0))
        {
            Disarm();
            return false;
        }
        m_until.store(aNow + (aTtlSec > kMaxArmTtlSec ? kMaxArmTtlSec : aTtlSec), std::memory_order_relaxed);
        return true;
    }
    void Disarm() { m_until.store(0.0, std::memory_order_relaxed); }
    bool Armed(double aNow) const
    {
        const double until = m_until.load(std::memory_order_relaxed);
        return until > 0.0 && aNow < until;
    }

private:
    std::atomic<double> m_until{0.0};
};

// Instance filter. One call site does not mean one object: the reader may run once per locomotion component. The first
// applying call of an arm session latches its instance; every later call of another instance is refused. Release() ends the
// session (called on every unarmed reader call). Track() counts the distinct instances seen on armed calls (cumulative, capped
// at kMaxTracked) so one armed run shows whether the reader is multi-instance. Safe from any thread (atomics only).
class InstanceLatch
{
public:
    static constexpr size_t kMaxTracked = 16;

    void Track(const void* aSelf)
    {
        for (size_t i = 0; i < kMaxTracked; ++i)
        {
            const void* cur = m_seen[i].load(std::memory_order_relaxed);
            if (cur == aSelf)
                return;
            if (!cur)
            {
                const void* expected = nullptr;
                if (m_seen[i].compare_exchange_strong(expected, aSelf, std::memory_order_relaxed) || expected == aSelf)
                    return;
            }
        }
    }
    size_t Distinct() const
    {
        size_t n = 0;
        for (size_t i = 0; i < kMaxTracked; ++i)
            n += m_seen[i].load(std::memory_order_relaxed) ? 1 : 0;
        return n;
    }
    // True when aSelf is (or just became) the latched instance. A refused call is counted.
    bool Admit(const void* aSelf)
    {
        const void* expected = nullptr;
        if (m_latched.compare_exchange_strong(expected, aSelf, std::memory_order_relaxed))
        {
            m_latches.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        if (expected == aSelf)
            return true;
        m_refused.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    void Release()
    {
        if (m_latched.load(std::memory_order_relaxed))
            m_latched.store(nullptr, std::memory_order_relaxed);
    }
    const void* Latched() const { return m_latched.load(std::memory_order_relaxed); }
    uint64_t Latches() const { return m_latches.load(std::memory_order_relaxed); }
    uint64_t Refused() const { return m_refused.load(std::memory_order_relaxed); }

private:
    std::atomic<const void*> m_latched{nullptr};
    std::atomic<const void*> m_seen[kMaxTracked]{};
    std::atomic<uint64_t> m_latches{0};
    std::atomic<uint64_t> m_refused{0};
};

// Overwrite decision: armed + a live override + no apply fault latched.
constexpr bool ShouldApply(bool aArmed, bool aOverrideLive, bool aFaulted)
{
    return aArmed && aOverrideLive && !aFaulted;
}

// Unit-length clamp of the override vector (the reader's own rule: scale down when |v| > 1, never up). Non-finite -> (0, 0)
// and false.
inline bool ClampUnit(float& aX, float& aY)
{
    if (!std::isfinite(aX) || !std::isfinite(aY))
    {
        aX = 0.0f;
        aY = 0.0f;
        return false;
    }
    const float len2 = aX * aX + aY * aY;
    if (len2 > 1.0f)
    {
        const float inv = 1.0f / std::sqrt(len2);
        aX *= inv;
        aY *= inv;
    }
    return true;
}
} // namespace utpp::freemove::nativein

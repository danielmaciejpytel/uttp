// UTPP Locomotion hook + natives (Phase 5N step 4a). Designed from the game's RTTI / RED4ext.SDK.
//
// Why a native hook (evidence, vanilla 2.31 scripts): the PSM reads the move input only through the natives
// StateGameScriptInterface.GetActionValue(n"MoveX"/n"MoveY") (DefaultTransition.GetMovementInputActionValue,
// meleeTransitions, slide/back-input checks) and GetInputHeading() (SprintDecisions, Dodge* DodgeDirection, swimming,
// ladder, low gravity) - 23 call sites in 6 files. redscript cannot wrap native functions, and wrapping every caller
// would duplicate vanilla decision code. One detour per native covers all callers and touches nothing else.
//
// Hook point: the member-native handler table slot [regIndex] the engine dispatches through. After proving the class
// + functions (native, non-static, Float return, parameter list), the table/dispatcher layout and the handlers
// (inside the game image, executable, not shared with any other native of the class), the two slots are swapped
// with one atomic compare-exchange each (restored on unload). Any failure -> not installed (inert).
//
// Table layout (second standalone run 2026-10-07: code -6 with
// handlerTable=0 / handler=0): RED4ext.SDK CBaseFunction::GetHandler treats the address-library entry
// CBaseFunction_Handlers (0x5A7D28A9) as a POINTER to the table and dereferences it. In 2.31 that entry IS the table:
// .data RVA 0x33E6E50, 0x8000 entries (slot 0 is null -> the SDK reading gives a null "table"). Evidence from
// the 2.31 Cyberpunk2077.exe (byte-identical to the retail exe):
//   rtti::Function::InternalCallNative (hash 0xA3961FF0, RVA 0x14619C), non-static branch @0x146251:
//     call [rax+18h] (GetRegIndex); mov ecx,eax; lea rax,[imageBase]; mov rsi,[rax+rcx*8+33E6E50h]; ...; call rsi
//   (the static branch reads a second table at 0x33A6E50 the same way);
//   rtti::NativeMemberFunction ctor (RVA 0x3CDF14): idx = [0x3426E50] + 0xFF; [0x3426E50]++; table[idx] = handler;
//     this->regIndex(+0xB8) = idx  (counter directly after the 0x40000-byte table);
//   the table RVA appears as an operand exactly twice in .text: that dispatcher read and that constructor write.
// So the dispatcher re-reads the slot on every call and nothing caches it: swapping the slot is the complete hook.
// The proof re-checks this layout at runtime (dispatcher bytes, counter, regIndex field) - fail-closed.
//
// Timing (first standalone run 2026-10-07: code -5 at PostRegisterTypes):
// the engine registers native class functions by name + handler only (like RED4ext CClassFunction::Create); their
// return type and parameters are filled in when the script runtime loads final.redscripts, which happens AFTER
// PostRegisterTypes. So PostRegister only registers the natives, runs the cheap checks and logs the pre-script RTTI
// facts; the full proof + attach run on the first Running frame (main menu: scripts bound, no player PSM ticking).
//
// Native input reader (vehicle approach walk): the capsule does not follow the script-side override, because the engine's
// per-frame locomotion input reader (2.5 exe RVA 0x6485E0, single caller) takes MoveX / MoveY from the input snapshot, not
// through these natives. A function-level hook on that reader (input/NativeInputCore.hpp: byte proof, fail closed) calls the
// original first; unarmed it only counts and samples the hardware vector, armed (approach running and the move override
// live) it overwrites the unit-clamped vector at this+0x90 / +0x94 and re-writes the two anim variables.
//
// Hot path (detours): atomics, one opcode-handler call to peek a constant CName argument, no logging, no allocation,
// no locks. The argument peek is SEH-guarded; a fault latches the kill switch for the session.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Stack.hpp>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "../CommonUtil.hpp"
#include "../PluginLog.hpp"
#include "ApproachCore.hpp"
#include "Locomotion.hpp"
#include "LocomotionCore.hpp"
#include "NativeInputCore.hpp"

namespace utpp::locomotion
{
namespace
{
using Handler_t = void (*)(void*, RED4ext::CStackFrame&, void*, RED4ext::rtti::IType*);

// redscript / REDengine bytecode opcode of a CName constant (Nop 0 ... F64Const 13, NameConst 14; consistent with
// Codeware's New = 48 / Cast = 90). Anything else is passed through untouched (counted as nonConstArgs).
constexpr uint8_t kOpNameConst = 14;

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;

freemove::Core g_core;
approach::Approach g_approach; // vehicle approach walk (game thread only: natives)
approach::Scene g_approachScene;
bool g_approachSceneValid = false;
approach::Params g_approachParams;
freemove::InstallResult g_install = freemove::InstallResult::NoClass;
bool g_initDone = false;
bool g_validated = false; // UTPP 2.31 validation result passed to OnPostRegister
std::atomic<bool> g_runningDone{false};
bool g_killFile = false;

Handler_t g_origActionValue = nullptr;
Handler_t g_origInputHeading = nullptr;
Handler_t* g_slotActionValue = nullptr;  // swapped slot (null = not swapped)
Handler_t* g_slotInputHeading = nullptr;
Handler_t* g_table = nullptr;            // proven member-native handler table (null = not proven)
uint32_t g_regCount = 0;                 // registration counter read during the proof
uint32_t g_regActionValue = 0;
uint32_t g_regInputHeading = 0;

uint64_t g_hMoveX = 0;
uint64_t g_hMoveY = 0;

// Native input reader hook (function level on the 2.5 locomotion input reader; see the header comment).
namespace nin = freemove::nativein;
using Reader_t = void (*)(void*);
using AnimSet_t = bool (*)(uint64_t, float, char, void*); // 0xB12138: (key, value [xmm1], flag [r8b], container [r9])
Reader_t g_origReader = nullptr;
uint8_t* g_readerTarget = nullptr;
std::atomic<bool> g_readerAttached{false};
bool g_animRewriteOk = false; // anim variable call chain proven at install (read-only afterwards)
AnimSet_t g_animSet = nullptr;
const uint64_t* g_animKeyX = nullptr;
const uint64_t* g_animKeyY = nullptr;
nin::Arming g_nativeArm; // armed by the approach step only
nin::InstanceLatch g_nativeInst; // the override applies to one reader instance per arm session
std::atomic<uint64_t> g_nativeReaderBail{0}; // applying frames on which the reader itself bailed out (no write made)
std::atomic<bool> g_nativeFault{false};
std::atomic<uint64_t> g_readerCalls{0};
std::atomic<uint64_t> g_nativeApplied{0};
std::atomic<uint64_t> g_nativeAnimWrites{0};
std::atomic<uint64_t> g_nativeFallbackBranch{0}; // armed frames where the reader took the non-snapshot branch (no anim rewrite)
std::atomic<uint64_t> g_nativeFaults{0};
std::atomic<uint32_t> g_hwXBits{0}; // last hardware vector the reader produced (this+0x90 / +0x94 after the original)
std::atomic<uint32_t> g_hwYBits{0};
float g_lastCmdX = 0.0f; // last stick command of the approach step (game thread)
float g_lastCmdY = 0.0f;

std::atomic<uint64_t> g_peekFaults{0};
std::atomic<uint64_t> g_nonConstArgs{0};
std::atomic<uint64_t> g_avCalls{0};

using pluginlog::LogInfo;
using pluginlog::LogWarn;

double Now() { return util::QpcSeconds(); }

// ---------------------------------------------------------------------------------------------
// Hot path.

struct FrameSave
{
    char* code;
    void* data;
    RED4ext::rtti::IType* dataType;
    uint8_t currentParam;
    bool useDirectData;
};

void SaveFrame(const RED4ext::CStackFrame& aF, FrameSave& aS)
{
    aS.code = aF.code;
    aS.data = aF.data;
    aS.dataType = aF.dataType;
    aS.currentParam = aF.currentParam;
    aS.useDirectData = aF.useDirectData;
}

void RestoreFrame(RED4ext::CStackFrame& aF, const FrameSave& aS)
{
    aF.code = aS.code;
    aF.data = aS.data;
    aF.dataType = aS.dataType;
    aF.currentParam = aS.currentParam;
    aF.useDirectData = aS.useDirectData;
}

// Evaluates the first argument only when it is a CName constant (pure: copies the inline constant), then the caller
// rewinds the frame so the original handler reads the argument exactly as without the hook.
bool PeekNameUnguarded(RED4ext::CStackFrame& aF, uint64_t& aOut)
{
    if (!aF.code || static_cast<uint8_t>(*aF.code) != kOpNameConst)
        return false;
    RED4ext::CName name;
    RED4ext::GetParameter(&aF, &name);
    aOut = name.hash;
    return true;
}

__declspec(noinline) bool PeekNameGuarded(RED4ext::CStackFrame& aF, uint64_t& aOut)
{
    FrameSave save;
    SaveFrame(aF, save);
    bool ok = false;
    __try
    {
        ok = PeekNameUnguarded(aF, aOut);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        ok = false;
        g_peekFaults.fetch_add(1, std::memory_order_relaxed);
        g_core.Kill();
    }
    RestoreFrame(aF, save);
    return ok;
}

void DetourActionValue(void* aCtx, RED4ext::CStackFrame& aFrame, void* aOut, RED4ext::rtti::IType* aType)
{
    g_avCalls.fetch_add(1, std::memory_order_relaxed);
    uint64_t name = 0;
    bool peeked = false;
    if (!g_core.Killed())
    {
        peeked = PeekNameGuarded(aFrame, name);
        if (!peeked)
            g_nonConstArgs.fetch_add(1, std::memory_order_relaxed);
    }
    g_origActionValue(aCtx, aFrame, aOut, aType);
    if (!peeked || !aOut || (name != g_hMoveX && name != g_hMoveY))
        return;
    auto* value = static_cast<float*>(aOut);
    *value = g_core.FilterAxis(name == g_hMoveX ? freemove::Axis::X : freemove::Axis::Y, *value, Now());
}

void DetourInputHeading(void* aCtx, RED4ext::CStackFrame& aFrame, void* aOut, RED4ext::rtti::IType* aType)
{
    g_origInputHeading(aCtx, aFrame, aOut, aType);
    if (!aOut)
        return;
    auto* value = static_cast<float*>(aOut);
    *value = g_core.FilterHeading(*value, Now());
}

float BitsToFloat(uint32_t aBits)
{
    float f;
    std::memcpy(&f, &aBits, sizeof(f));
    return f;
}

uint32_t FloatToBits(float aValue)
{
    uint32_t b;
    std::memcpy(&b, &aValue, sizeof(b));
    return b;
}

__declspec(noinline) bool SampleReaderInput(const void* aSelf, float& aX, float& aY)
{
    __try
    {
        const auto* base = static_cast<const uint8_t*>(aSelf);
        aX = *reinterpret_cast<const float*>(base + nin::kOffInputX);
        aY = *reinterpret_cast<const float*>(base + nin::kOffInputY);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Overwrites the reader's vector and (when proven and the reader took the snapshot branch) the two anim variables through the
// reader's own setter with the owner/container chain the reader itself uses. aAnim: both anim variables written.
// aFallback: the reader took the non-snapshot branch (anim variables untouched). aBail: the reader's own early-out test (ref
// block present, strong count non-zero, object present) fails, i.e. the reader skipped this frame: nothing is written (the
// detour runs after the reader has released its own ref; the test is a guard, not a pin). False = fault.
__declspec(noinline) bool ApplyReaderInput(void* aSelf, float aX, float aY, bool& aAnim, bool& aFallback, bool& aBail)
{
    aAnim = false;
    aFallback = false;
    aBail = false;
    __try
    {
        auto* base = static_cast<uint8_t*>(aSelf);
        using Slot_t = uint8_t* (*)(void*);
        void* sub = base + nin::kOffSub;
        void** vtable = *static_cast<void***>(sub);
        const uint8_t* head = reinterpret_cast<Slot_t>(vtable[0])(sub);
        const int32_t* refBlock = head ? *reinterpret_cast<const int32_t* const*>(head + nin::kOffHeadRef) : nullptr;
        const bool hasObject = head && *reinterpret_cast<void* const*>(head + nin::kOffHeadObj) != nullptr;
        if (!nin::ReaderWouldRun(refBlock != nullptr, refBlock ? *refBlock : 0, hasObject))
        {
            aBail = true;
            return true;
        }
        *reinterpret_cast<float*>(base + nin::kOffInputX) = aX;
        *reinterpret_cast<float*>(base + nin::kOffInputY) = aY;
        if (!g_animRewriteOk)
            return true;
        if (!*reinterpret_cast<void* const*>(head + nin::kOffSubField))
        {
            aFallback = true; // the reader derives its vector from the anim variables here: nothing to rewrite
            return true;
        }
        const uint8_t* owner = reinterpret_cast<Slot_t>(vtable[1])(sub);
        const uint64_t keyX = *g_animKeyX;
        const uint64_t keyY = *g_animKeyY;
        uint8_t* block = owner ? *reinterpret_cast<uint8_t* const*>(owner + nin::kOffContainerPtr) : nullptr;
        if (!block || keyX == 0 || keyY == 0)
            return true;
        void* container = block + nin::kOffContainerAdd;
        g_animSet(keyX, aX, 1, container);
        g_animSet(keyY, aY, 1, container);
        aAnim = true;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

void DetourReader(void* aSelf)
{
    g_origReader(aSelf);
    g_readerCalls.fetch_add(1, std::memory_order_relaxed);
    if (!aSelf)
        return;
    float hx = 0.0f, hy = 0.0f;
    if (SampleReaderInput(aSelf, hx, hy))
    {
        g_hwXBits.store(FloatToBits(hx), std::memory_order_relaxed);
        g_hwYBits.store(FloatToBits(hy), std::memory_order_relaxed);
    }
    const double now = Now();
    float x = 0.0f, y = 0.0f, heading = 0.0f;
    const bool armed = g_nativeArm.Armed(now);
    if (!armed)
    {
        g_nativeInst.Release(); // the arm session ended: the next one latches afresh
        return;
    }
    g_nativeInst.Track(aSelf);
    const bool live = g_core.GetMoveOverride(now, x, y, heading);
    if (!nin::ShouldApply(armed, live, g_nativeFault.load(std::memory_order_relaxed)))
        return;
    if (!g_nativeInst.Admit(aSelf))
        return; // another reader instance than the one latched for this arm session
    nin::ClampUnit(x, y);
    bool anim = false, fallback = false, bail = false;
    if (!ApplyReaderInput(aSelf, x, y, anim, fallback, bail))
    {
        g_nativeFaults.fetch_add(1, std::memory_order_relaxed);
        g_nativeFault.store(true, std::memory_order_relaxed);
        g_nativeArm.Disarm();
        return;
    }
    if (bail)
    {
        g_nativeReaderBail.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    g_nativeApplied.fetch_add(1, std::memory_order_relaxed);
    if (anim)
        g_nativeAnimWrites.fetch_add(1, std::memory_order_relaxed);
    if (fallback)
        g_nativeFallbackBranch.fetch_add(1, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------
// Resolution / proof (game thread, PostRegister).

bool InGameImage(const void* aPtr)
{
    const auto* base = reinterpret_cast<const uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !aPtr)
        return false;
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    const auto* p = static_cast<const uint8_t*>(aPtr);
    return p >= base && p < base + nt->OptionalHeader.SizeOfImage;
}

bool Executable(const void* aPtr)
{
    MEMORY_BASIC_INFORMATION mbi{};
    if (!aPtr || VirtualQuery(aPtr, &mbi, sizeof(mbi)) != sizeof(mbi) || mbi.State != MEM_COMMIT)
        return false;
    const DWORD exec = PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    return (mbi.Protect & exec) != 0;
}

uint64_t Rva(const void* aPtr)
{
    if (!InGameImage(aPtr))
        return 0;
    return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(aPtr) -
                                 reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr)));
}

bool ReadSlot(Handler_t* aTable, uint32_t aIndex, void*& aOut)
{
    __try
    {
        aOut = reinterpret_cast<void*>(aTable[aIndex]);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aOut = nullptr;
        return false;
    }
}

constexpr uint32_t kHashHandlerTable = RED4ext::Detail::AddressHashes::CBaseFunction_Handlers; // 0x5A7D28A9
constexpr uint32_t kHashInternalCallNative = 0xA3961FF0; // rtti::Function::InternalCallNative (2744524784)
constexpr size_t kDispatcherScanBytes = 0x130;

// RED4ext address library without the SDK's terminate-on-miss behaviour (0 = unknown hash).
uintptr_t ResolveHash(uint32_t aHash)
{
    using Resolve_t = uintptr_t (*)(uint32_t);
    const HMODULE red4ext = GetModuleHandleW(L"RED4ext.dll");
    const auto fn = red4ext ? reinterpret_cast<Resolve_t>(GetProcAddress(red4ext, "RED4ext_ResolveAddress")) : nullptr;
    return fn ? fn(aHash) : 0;
}

bool Writable(const void* aPtr)
{
    MEMORY_BASIC_INFORMATION mbi{};
    if (!aPtr || VirtualQuery(aPtr, &mbi, sizeof(mbi)) != sizeof(mbi) || mbi.State != MEM_COMMIT)
        return false;
    const DWORD rw = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    return (mbi.Protect & rw) != 0 && (mbi.Protect & PAGE_GUARD) == 0;
}

bool ReadBytes(const void* aSrc, void* aDst, size_t aLen)
{
    __try
    {
        std::memcpy(aDst, aSrc, aLen);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Observed table facts (both candidate readings are reported).
struct TableInfo
{
    uintptr_t entry = 0;      // address-library entry 0x5A7D28A9
    void* sdkDeref = nullptr; // what RED4ext.SDK GetHandler uses as the table (*entry)
    uintptr_t dispatcher = 0; // rtti::Function::InternalCallNative
    freemove::DispatchScan scan;
    bool entryInImage = false;
    bool entryWritable = false;
    bool tailInImage = false; // counter after the table is inside the image
    bool countRead = false;
    uint32_t count = 0;
    bool proven = false;
};

TableInfo ProveTable()
{
    TableInfo t;
    t.entry = ResolveHash(kHashHandlerTable);
    t.dispatcher = ResolveHash(kHashInternalCallNative);
    auto* entry = reinterpret_cast<uint8_t*>(t.entry);
    t.entryInImage = InGameImage(entry);
    t.entryWritable = t.entryInImage && Writable(entry);
    if (t.entryInImage)
        ReadBytes(entry, &t.sdkDeref, sizeof(t.sdkDeref));
    uint8_t* tail = entry ? entry + static_cast<size_t>(freemove::kHandlerTableEntries) * sizeof(Handler_t) : nullptr;
    t.tailInImage = t.entryInImage && InGameImage(tail) && InGameImage(tail + sizeof(uint32_t) - 1);
    if (t.tailInImage)
        t.countRead = ReadBytes(tail, &t.count, sizeof(t.count));
    if (t.entryInImage && InGameImage(reinterpret_cast<void*>(t.dispatcher)) &&
        InGameImage(reinterpret_cast<void*>(t.dispatcher + kDispatcherScanBytes - 1)) &&
        Executable(reinterpret_cast<void*>(t.dispatcher)))
    {
        uint8_t code[kDispatcherScanBytes];
        if (ReadBytes(reinterpret_cast<void*>(t.dispatcher), code, sizeof(code)))
            t.scan = freemove::ScanDispatcher(code, sizeof(code), Rva(reinterpret_cast<void*>(t.dispatcher)), Rva(entry));
    }
    t.proven = t.entryInImage && t.entryWritable && t.tailInImage && t.countRead && t.scan.tableReads == 1 &&
               t.count > 0 && t.count < freemove::kHandlerTableEntries;
    return t;
}

Handler_t* HandlerTable()
{
    return g_table;
}

Handler_t* SlotOf(uint32_t aReg)
{
    Handler_t* table = HandlerTable();
    return table && aReg < freemove::kHandlerTableEntries ? table + aReg : nullptr;
}

void* HandlerOf(RED4ext::CBaseFunction* aFn)
{
    if (!aFn || !aFn->flags.isNative || aFn->flags.isStatic)
        return nullptr;
    Handler_t* slot = SlotOf(aFn->GetRegIndex());
    void* h = nullptr;
    return slot && ReadSlot(slot, 0, h) ? h : nullptr;
}

// Atomic slot swap (expected -> replacement). Only on a slot proven writable; no protection change.
bool SwapSlot(Handler_t* aSlot, Handler_t aExpected, Handler_t aReplacement)
{
    if (!aSlot || !Writable(aSlot))
        return false;
    __try
    {
        void* prev = InterlockedCompareExchangePointer(reinterpret_cast<void* volatile*>(aSlot),
                                                       reinterpret_cast<void*>(aReplacement),
                                                       reinterpret_cast<void*>(aExpected));
        return prev == reinterpret_cast<void*>(aExpected);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool TypeIs(RED4ext::CProperty* aProp, const char* aName)
{
    return aProp && aProp->type && aProp->type->GetName() == RED4ext::CName(aName);
}

const char* NameStr(RED4ext::CName aName)
{
    if (aName.IsNone())
        return "<none>";
    const char* str = aName.ToString();
    return str ? str : "<?>";
}

std::string PropTypeStr(const RED4ext::CProperty* aProp)
{
    if (!aProp)
        return "<null>";
    if (!aProp->type)
        return "<null type>";
    return NameStr(aProp->type->GetName());
}

// Which class in the parent chain declares aFn (CClass::GetFunction walks parents).
const char* OwnerName(RED4ext::CClass* aCls, const RED4ext::CBaseFunction* aFn)
{
    for (auto* c = aCls; c; c = c->parent)
    {
        for (auto* f : c->funcs)
            if (f == aFn)
                return NameStr(c->name);
        for (auto* f : c->staticFuncs)
            if (f == aFn)
                return NameStr(c->name);
    }
    return "<?>";
}

// One log line with everything the proof looked at (observed values, not expectations).
std::string Describe(const char* aName, RED4ext::CClass* aCls, RED4ext::CClassFunction* aFn,
                     const freemove::FunctionFacts& aF, void* aHandler, uint32_t aReg)
{
    char buf[768];
    if (!aFn)
    {
        std::snprintf(buf, sizeof(buf), "%s: NOT FOUND in %s or its parents", aName,
                      aCls ? NameStr(aCls->name) : "<no class>");
        return buf;
    }
    uint32_t rawFlags = 0;
    static_assert(sizeof(rawFlags) == sizeof(aFn->flags));
    std::memcpy(&rawFlags, &aFn->flags, sizeof(rawFlags));
    std::string params;
    for (uint32_t i = 0; i < aFn->params.size(); ++i)
    {
        const RED4ext::CProperty* p = aFn->params[i];
        if (!params.empty())
            params += ", ";
        if (!p)
        {
            params += "<null>";
            continue;
        }
        params += NameStr(p->name);
        params += ':';
        params += PropTypeStr(p);
        if (p->flags.isOut)
            params += "[out]";
        if (p->flags.isOptional)
            params += "[opt]";
    }
    const std::string ret = PropTypeStr(aFn->returnType);
    std::snprintf(buf, sizeof(buf),
                  "%s: fn=%p owner=%s full=%s short=%s flags=0x%08X native=%d static=%d final=%d const=%d reg=%u "
                  "regField=%u regOk=%d ret=%s params=%u(%s) locals=%u handler=%p rva=0x%llX image=%d exec=%d "
                  "unique=%d%s",
                  aName, static_cast<void*>(aFn), OwnerName(aCls, aFn), NameStr(aFn->fullName),
                  NameStr(aFn->shortName), rawFlags, aFn->flags.isNative ? 1 : 0, aFn->flags.isStatic ? 1 : 0,
                  aFn->flags.isFinal ? 1 : 0, aFn->flags.isConst ? 1 : 0, aReg, aFn->regIndex, aF.regOk ? 1 : 0,
                  ret.c_str(),
                  static_cast<unsigned>(aFn->params.size()), params.c_str(), static_cast<unsigned>(aFn->localVars.size()),
                  aHandler, static_cast<unsigned long long>(Rva(aHandler)), aF.handlerInImage ? 1 : 0,
                  aF.handlerExecutable ? 1 : 0, aF.handlerUnique ? 1 : 0,
                  freemove::LooksUnbound(aF) ? " [unbound: no return type / params yet]" : "");
    return buf;
}

RED4ext::CClassFunction* Collect(RED4ext::CClass* aCls, const char* aName, freemove::FunctionFacts& aFacts,
                                 void*& aHandler, uint32_t& aReg)
{
    aHandler = nullptr;
    auto* fn = aCls->GetFunction(RED4ext::CName(aName));
    if (!fn)
        return nullptr;
    aFacts.found = true;
    aFacts.isNative = fn->flags.isNative != 0;
    aFacts.isStatic = fn->flags.isStatic != 0;
    aFacts.hasReturnType = fn->returnType != nullptr;
    aFacts.returnsFloat = TypeIs(fn->returnType, "Float");
    aFacts.paramCount = static_cast<int>(fn->params.size());
    aFacts.firstParamIsCName = fn->params.size() >= 1 && TypeIs(fn->params[0], "CName");
    aFacts.firstParamByValue =
        fn->params.size() >= 1 && fn->params[0] && !fn->params[0]->flags.isOut && !fn->params[0]->flags.isOptional;
    if (!aFacts.isNative || aFacts.isStatic)
        return fn;
    aReg = fn->GetRegIndex();
    aFacts.regOk = HandlerTable() && fn->regIndex == aReg && freemove::RegPlausible(aReg, g_regCount);
    if (!aFacts.regOk)
        return fn;
    aHandler = HandlerOf(fn);
    aFacts.handlerInImage = InGameImage(aHandler);
    aFacts.handlerExecutable = Executable(aHandler);
    // Unique: no other native method of the class (or its parents) dispatches to the same handler.
    bool unique = aHandler != nullptr;
    for (auto* c = aCls; c && unique; c = c->parent)
    {
        for (auto* other : c->funcs)
        {
            if (other && other != fn && other->flags.isNative && !other->flags.isStatic && HandlerOf(other) == aHandler)
            {
                unique = false;
                break;
            }
        }
    }
    aFacts.handlerUnique = unique;
    return fn;
}

// RTTI read only (no writes): class / function facts plus one diagnostic line per item.
struct Resolved
{
    void* hAv = nullptr;
    void* hIh = nullptr;
    std::string classLine;
    std::string tableLine;
    std::string avLine;
    std::string ihLine;
};

void Resolve(freemove::InstallFacts& aFacts, Resolved& aOut)
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const char* via = "gamestateMachineGameScriptInterface";
    RED4ext::CClass* cls = rtti ? rtti->GetClass("gamestateMachineGameScriptInterface") : nullptr;
    if (!cls && rtti)
    {
        via = "StateGameScriptInterface";
        cls = rtti->GetClass("StateGameScriptInterface");
    }
    aFacts.classFound = cls != nullptr;
    char buf[512];
    if (!cls)
    {
        std::snprintf(buf, sizeof(buf),
                      "class: NOT FOUND (rtti=%p, tried gamestateMachineGameScriptInterface, StateGameScriptInterface)",
                      static_cast<void*>(rtti));
        aOut.classLine = buf;
        return;
    }
    std::snprintf(buf, sizeof(buf), "class: %s (lookup '%s') cls=%p parent=%s funcs=%u staticFuncs=%u",
                  NameStr(cls->name), via, static_cast<void*>(cls),
                  cls->parent ? NameStr(cls->parent->name) : "<none>", static_cast<unsigned>(cls->funcs.size()),
                  static_cast<unsigned>(cls->staticFuncs.size()));
    aOut.classLine = buf;
    const TableInfo t = ProveTable();
    g_table = t.proven ? reinterpret_cast<Handler_t*>(t.entry) : nullptr;
    g_regCount = t.count;
    aFacts.tableProven = t.proven;
    std::snprintf(buf, sizeof(buf),
                  "table: entry(0x5A7D28A9)=%p rva=0x%llX image=%d writable=%d | sdkDeref(*entry)=%p | "
                  "dispatcher(0xA3961FF0)=%p rva=0x%llX reads=%d tableReads=%d otherDisp=0x%X | count@+0x40000=%u "
                  "read=%d | proven=%d (array = entry itself, %u slots)",
                  reinterpret_cast<void*>(t.entry), static_cast<unsigned long long>(Rva(reinterpret_cast<void*>(t.entry))),
                  t.entryInImage ? 1 : 0, t.entryWritable ? 1 : 0, t.sdkDeref, reinterpret_cast<void*>(t.dispatcher),
                  static_cast<unsigned long long>(Rva(reinterpret_cast<void*>(t.dispatcher))), t.scan.reads,
                  t.scan.tableReads, t.scan.firstOtherDisp, t.count, t.countRead ? 1 : 0, t.proven ? 1 : 0,
                  freemove::kHandlerTableEntries);
    aOut.tableLine = buf;
    uint32_t regAv = 0, regIh = 0;
    auto* fnAv = Collect(cls, "GetActionValue", aFacts.actionValue, aOut.hAv, regAv);
    auto* fnIh = Collect(cls, "GetInputHeading", aFacts.inputHeading, aOut.hIh, regIh);
    aFacts.handlersDistinct = aOut.hAv && aOut.hIh && aOut.hAv != aOut.hIh;
    g_regActionValue = regAv;
    g_regInputHeading = regIh;
    aOut.avLine = Describe("GetActionValue", cls, fnAv, aFacts.actionValue, aOut.hAv, regAv);
    aOut.ihLine = Describe("GetInputHeading", cls, fnIh, aFacts.inputHeading, aOut.hIh, regIh);
}

// aWarn: failure diagnostics; otherwise informational (pre-script snapshot / successful install).
void LogResolved(const char* aPhase, const Resolved& aR, const freemove::InstallFacts& aF, bool aWarn)
{
    auto line = [&](const std::string& aText)
    {
        if (aText.empty())
            return;
        if (aWarn)
            LogWarn("UTPP Locomotion facts [%s] %s", aPhase, aText.c_str());
        else
            LogInfo("UTPP Locomotion facts [%s] %s", aPhase, aText.c_str());
    };
    line(aR.classLine);
    line(aR.tableLine);
    line(aR.avLine);
    line(aR.ihLine);
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "check: expected GetActionValue(actionName:CName)->Float and GetInputHeading()->Float, native=1 "
                  "static=0; signatureOk av=%d ih=%d tableProven=%d handlerOk av=%d ih=%d distinct=%d",
                  freemove::SignatureOk(aF.actionValue, 1, true) ? 1 : 0, aF.tableProven ? 1 : 0,
                  freemove::SignatureOk(aF.inputHeading, 0, false) ? 1 : 0, freemove::HandlerOk(aF.actionValue) ? 1 : 0,
                  freemove::HandlerOk(aF.inputHeading) ? 1 : 0, aF.handlersDistinct ? 1 : 0);
    line(buf);
}

std::wstring KillFilePath()
{
    const std::wstring dir = util::PluginDir();
    return dir.empty() ? std::wstring() : dir + L"utpp_locomotion.disable";
}

freemove::InstallFacts EarlyFacts(bool aGameValidated)
{
    freemove::InstallFacts facts;
    facts.killFile = g_killFile;
    facts.gameValidated = aGameValidated;
    return facts;
}

// Function-level hook on the native locomotion input reader. Anchored on the proven GetActionValue handler (the anchor must sit
// at the expected RVA), every site verified from the running image (input/NativeInputCore.hpp); any mismatch -> not attached
// (the approach walk then refuses to start). The hook only counts / samples while unarmed.
void InstallReader()
{
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !g_sdk || !g_sdk->hooking)
    {
        LogWarn("UTPP Locomotion native input reader: not hooked (no image base / hooking API)");
        return;
    }
    auto read = [&](uint64_t aRva, uint8_t* aDst, size_t aLen)
    {
        return aLen > 0 && InGameImage(base + aRva) && InGameImage(base + aRva + aLen - 1) && ReadBytes(base + aRva, aDst, aLen);
    };
    const uint64_t handlerRva = Rva(reinterpret_cast<const void*>(g_origActionValue));
    const nin::Verdict v = nin::VerifyReader(handlerRva, g_hMoveX, g_hMoveY, read);
    if (!v.ok)
    {
        LogWarn("UTPP Locomotion native input reader: not hooked (%s; anchor rva 0x%llX); approach walk refused", v.why,
                static_cast<unsigned long long>(handlerRva));
        return;
    }
    bool animOk = v.animOk;
    uint64_t keys[2] = {0, 0};
    if (animOk)
    {
        // the keys are CName hashes the engine fills at start-up; read again on every use (zero = skip the anim rewrite)
        animOk = read(nin::kRvaAnimKeyX, reinterpret_cast<uint8_t*>(&keys[0]), sizeof(uint64_t)) &&
                 read(nin::kRvaAnimKeyY, reinterpret_cast<uint8_t*>(&keys[1]), sizeof(uint64_t)) &&
                 Executable(base + nin::kRvaAnimSet);
    }
    uint8_t* target = base + nin::kRvaReader;
    if (!Executable(target))
    {
        LogWarn("UTPP Locomotion native input reader: not hooked (target not executable)");
        return;
    }
    g_animRewriteOk = animOk;
    g_animSet = reinterpret_cast<AnimSet_t>(base + nin::kRvaAnimSet);
    g_animKeyX = reinterpret_cast<const uint64_t*>(base + nin::kRvaAnimKeyX);
    g_animKeyY = reinterpret_cast<const uint64_t*>(base + nin::kRvaAnimKeyY);
    if (!g_sdk->hooking->Attach(g_handle, target, reinterpret_cast<void*>(&DetourReader), reinterpret_cast<void**>(&g_origReader)) ||
        !g_origReader)
    {
        g_origReader = nullptr;
        g_animRewriteOk = false;
        LogWarn("UTPP Locomotion native input reader: not hooked (hooking->Attach failed @0x%llX); approach walk refused",
                static_cast<unsigned long long>(nin::kRvaReader));
        return;
    }
    g_readerTarget = target;
    g_readerAttached.store(true, std::memory_order_release);
    LogInfo("UTPP Locomotion native input reader: hooked @%p (rva 0x%llX, known call site verified, anim variable rewrite %s); "
            "counts and samples only until an approach arms it",
            static_cast<void*>(target), static_cast<unsigned long long>(nin::kRvaReader),
            animOk ? "proven" : "NOT proven (hardware vector overwrite only)");
}

// First Running frame: script runtime initialised, natives carry their script signatures. Full proof + attach.
void Install(bool aGameValidated)
{
    freemove::InstallFacts facts = EarlyFacts(aGameValidated);
    Resolved res;
    // RTTI is only touched when the cheap decisions allow it.
    if (freemove::DecideEarly(facts) == freemove::InstallResult::Pending)
        Resolve(facts, res);

    g_install = freemove::DecideInstall(facts);
    void* hAv = res.hAv;
    void* hIh = res.hIh;
    if (g_install == freemove::InstallResult::Ok)
    {
        // Originals first: the detours may run on another thread the moment a slot is swapped.
        g_origActionValue = reinterpret_cast<Handler_t>(hAv);
        g_origInputHeading = reinterpret_cast<Handler_t>(hIh);
        Handler_t* slotAv = SlotOf(g_regActionValue);
        Handler_t* slotIh = SlotOf(g_regInputHeading);
        if (!SwapSlot(slotAv, g_origActionValue, &DetourActionValue))
        {
            g_install = freemove::InstallResult::AttachFailed;
        }
        else
        {
            g_slotActionValue = slotAv;
            if (!SwapSlot(slotIh, g_origInputHeading, &DetourInputHeading))
            {
                if (SwapSlot(slotAv, &DetourActionValue, g_origActionValue))
                    g_slotActionValue = nullptr;
                g_install = freemove::InstallResult::AttachFailed;
            }
            else
            {
                g_slotInputHeading = slotIh;
            }
        }
    }
    const bool installed = g_install == freemove::InstallResult::Ok;
    g_core.SetInstalled(installed);
    if (installed)
        InstallReader();
    if (installed)
    {
        LogInfo("UTPP Locomotion: handler slots swapped on first Running frame (GetActionValue reg %u orig %p rva 0x%llX, "
                "GetInputHeading reg %u orig %p rva 0x%llX); default disabled",
                g_regActionValue, hAv, static_cast<unsigned long long>(Rva(hAv)), g_regInputHeading, hIh,
                static_cast<unsigned long long>(Rva(hIh)));
        LogResolved("running", res, facts, false);
    }
    else
    {
        LogWarn("UTPP Locomotion not installed: code %d (%s)%s", static_cast<int>(g_install),
                freemove::InstallResultName(g_install),
                g_slotActionValue ? " - GetActionValue slot could NOT be restored (slot changed by another module)" : "");
        LogResolved("running", res, facts, true);
    }
}

// ---------------------------------------------------------------------------------------------
// Natives.

std::string Status()
{
    const double now = Now();
    const freemove::Params p = g_core.GetParams();
    char buf[2048];
    std::snprintf(buf, sizeof(buf),
                  "UTPP Locomotion install=%d (%s) active=%d reasons=0x%X enabled=%d killed=%d blockMask=0x%X "
                  "hasInput=%d rawX=%.3f rawY=%.3f rawAngle=%.1f axisReads=%llu axisRewrites=%llu headingReads=%llu "
                  "headingRewrites=%llu beats=%llu activations=%llu avCalls=%llu nonConstArgs=%llu peekFaults=%llu "
                  "regAV=%u regIH=%u stale=%.2f deadzone=%.2f tppDist=%.2f overrideSets=%llu overrideReads=%llu overrideActive=%d",
                  static_cast<int>(g_install), freemove::InstallResultName(g_install), g_core.Active(now) ? 1 : 0,
                  g_core.Reasons(now), g_core.Enabled() ? 1 : 0, g_core.Killed() ? 1 : 0,
                  g_core.BlockMask(), g_core.HasInput(now) ? 1 : 0,
                  static_cast<double>(g_core.RawAxis(freemove::Axis::X)), static_cast<double>(g_core.RawAxis(freemove::Axis::Y)),
                  static_cast<double>(g_core.RawAngle(now)), static_cast<unsigned long long>(g_core.AxisReads()),
                  static_cast<unsigned long long>(g_core.AxisRewrites()),
                  static_cast<unsigned long long>(g_core.HeadingReads()),
                  static_cast<unsigned long long>(g_core.HeadingRewrites()),
                  static_cast<unsigned long long>(g_core.Beats()), static_cast<unsigned long long>(g_core.Activations()),
                  static_cast<unsigned long long>(g_avCalls.load()), static_cast<unsigned long long>(g_nonConstArgs.load()),
                  static_cast<unsigned long long>(g_peekFaults.load()), g_regActionValue, g_regInputHeading, p.staleSec,
                  static_cast<double>(p.deadzone), static_cast<double>(p.tppCameraDist),
                  static_cast<unsigned long long>(g_core.OverrideSets()),
                  static_cast<unsigned long long>(g_core.OverrideAxisReads()), g_core.OverrideActive(now) ? 1 : 0);
    const size_t used = std::strlen(buf);
    std::snprintf(buf + used, sizeof(buf) - used,
                  " rdAttached=%d rdAnimOk=%d rdCalls=%llu rdHwX=%.3f rdHwY=%.3f rdArmed=%d nativeApplied=%llu rdAnimWrites=%llu "
                  "rdFallbackBranch=%llu rdFaults=%llu rdFault=%d rdDistinctThis=%zu rdLatches=%llu rdOtherInstRefused=%llu "
                  "rdReaderBail=%llu",
                  g_readerAttached.load() ? 1 : 0, g_animRewriteOk ? 1 : 0, static_cast<unsigned long long>(g_readerCalls.load()),
                  static_cast<double>(BitsToFloat(g_hwXBits.load())), static_cast<double>(BitsToFloat(g_hwYBits.load())),
                  g_nativeArm.Armed(now) ? 1 : 0, static_cast<unsigned long long>(g_nativeApplied.load()),
                  static_cast<unsigned long long>(g_nativeAnimWrites.load()),
                  static_cast<unsigned long long>(g_nativeFallbackBranch.load()),
                  static_cast<unsigned long long>(g_nativeFaults.load()), g_nativeFault.load() ? 1 : 0,
                  g_nativeInst.Distinct(), static_cast<unsigned long long>(g_nativeInst.Latches()),
                  static_cast<unsigned long long>(g_nativeInst.Refused()),
                  static_cast<unsigned long long>(g_nativeReaderBail.load()));
    return buf;
}

void N_Status(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(Status().c_str());
}

void N_SetEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    g_core.SetEnabled(on);
    LogInfo("UTPPLocomotionSetEnabled(%d) install=%d killed=%d", on ? 1 : 0, static_cast<int>(g_install),
            g_core.Killed() ? 1 : 0);
}

void N_Kill(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    g_core.Kill();
    LogWarn("UTPPLocomotionKill(): Locomotion pass-through for the rest of the session");
}

void N_SetHold(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    g_core.SetHold(on);
}

void N_SetBlockMask(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t mask = static_cast<int32_t>(freemove::kDefaultBlockMask);
    RED4ext::GetParameter(aFrame, &mask);
    aFrame->code++;
    g_core.SetBlockMask(static_cast<uint32_t>(mask));
    LogInfo("UTPPLocomotionSetBlockMask(0x%X)", g_core.BlockMask());
}

void N_SetParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float stale = 0.25f, deadzone = 0.15f, tppDist = 0.6f;
    RED4ext::GetParameter(aFrame, &stale);
    RED4ext::GetParameter(aFrame, &deadzone);
    RED4ext::GetParameter(aFrame, &tppDist);
    aFrame->code++;
    freemove::Params p = g_core.GetParams();
    p.staleSec = stale;
    p.deadzone = deadzone;
    p.tppCameraDist = tppDist;
    const bool ok = g_core.SetParams(p);
    LogInfo("UTPPLocomotionSetParams(%.3f, %.3f, %.3f) -> %d", static_cast<double>(stale), static_cast<double>(deadzone),
            static_cast<double>(tppDist), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_Heartbeat(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool ownerTpp = false;
    float camDist = 0.0f;
    int32_t bits = 0;
    RED4ext::GetParameter(aFrame, &ownerTpp);
    RED4ext::GetParameter(aFrame, &camDist);
    RED4ext::GetParameter(aFrame, &bits);
    aFrame->code++;
    g_core.Heartbeat(Now(), ownerTpp, camDist, static_cast<uint32_t>(bits));
}

void N_Active(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = g_core.Active(Now());
}

void N_Reasons(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = static_cast<int32_t>(g_core.Reasons(Now()));
}

void N_HasInput(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = g_core.HasInput(Now());
}

void N_RawAngle(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = g_core.RawAngle(Now());
}

// Script thread (the PSM dodge decision, utpp_locomotion.reds): the DodgeDirection to store (LocomotionCore DodgeHeading).
void N_DodgeHeading(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float game = 0.0f;
    RED4ext::GetParameter(aFrame, &game);
    aFrame->code++;
    const float served = g_core.DodgeHeading(game, Now());
    if (aOut)
        *aOut = served;
}

// ---- move override / vehicle approach walk (src/input/LocomotionCore.hpp override, src/input/ApproachCore.hpp planner) ----
// Game thread only. The approach walk is a pure consumer of the override channel: every step it writes a short-lived stick.

void N_SetMoveOverride(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float x = 0.0f, y = 0.0f, heading = 0.0f, ttl = 0.0f;
    RED4ext::GetParameter(aFrame, &x);
    RED4ext::GetParameter(aFrame, &y);
    RED4ext::GetParameter(aFrame, &heading);
    RED4ext::GetParameter(aFrame, &ttl);
    aFrame->code++;
    g_core.SetMoveOverride(x, y, heading, static_cast<double>(ttl), Now());
}

void N_ClearMoveOverride(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    g_core.ClearMoveOverride();
}

void N_OverrideReady(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = g_core.OverrideAllowed();
}

void N_ApproachSetScene(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float cx = 0, cy = 0, hx = 0, hy = 0, hingeX = 0, hingeY = 0, len = 0, closedDeg = 0, openDeg = 0;
    bool arcValid = false;
    RED4ext::GetParameter(aFrame, &cx);
    RED4ext::GetParameter(aFrame, &cy);
    RED4ext::GetParameter(aFrame, &hx);
    RED4ext::GetParameter(aFrame, &hy);
    RED4ext::GetParameter(aFrame, &arcValid);
    RED4ext::GetParameter(aFrame, &hingeX);
    RED4ext::GetParameter(aFrame, &hingeY);
    RED4ext::GetParameter(aFrame, &len);
    RED4ext::GetParameter(aFrame, &closedDeg);
    RED4ext::GetParameter(aFrame, &openDeg);
    aFrame->code++;
    g_approachSceneValid = false;
    if (!(std::isfinite(cx) && std::isfinite(cy) && std::isfinite(hx) && std::isfinite(hy) && hx > 0.3f && hy > 0.5f))
        return;
    g_approachScene = approach::Scene{};
    g_approachScene.box = approach::BoxPoly(cx, cy, hx, hy, 0.35f);
    if (arcValid && std::isfinite(hingeX) && std::isfinite(hingeY) && std::isfinite(len) && std::isfinite(closedDeg) &&
        std::isfinite(openDeg) && len > 0.3f && len < 3.0f)
    {
        g_approachScene.sector = approach::SectorPoly({hingeX, hingeY}, len, closedDeg, openDeg, 8.0f);
        g_approachScene.haveSector = true;
    }
    g_approachSceneValid = true;
}

// 1 started, 0 already at the spot (mount at once), < 0: -1 invalid input, -2 no path, -3 too far, -4 scene missing,
// -5 override channel not available (hook not installed / kill switch / native input reader hook not attached or faulted).
void N_ApproachBegin(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    float px = 0, py = 0, yaw = 0, tx = 0, ty = 0, tyaw = 0;
    RED4ext::GetParameter(aFrame, &px);
    RED4ext::GetParameter(aFrame, &py);
    RED4ext::GetParameter(aFrame, &yaw);
    RED4ext::GetParameter(aFrame, &tx);
    RED4ext::GetParameter(aFrame, &ty);
    RED4ext::GetParameter(aFrame, &tyaw);
    aFrame->code++;
    int32_t r;
    g_approach.Cancel();
    g_core.ClearMoveOverride();
    g_nativeArm.Disarm();
    if (!g_approachSceneValid)
        r = -4;
    else if (!g_core.OverrideAllowed() || !g_readerAttached.load(std::memory_order_acquire) ||
             g_nativeFault.load(std::memory_order_relaxed))
        r = -5;
    else
        r = static_cast<int32_t>(g_approach.Begin(g_approachScene, g_approachParams, {px, py}, yaw, {tx, ty}, tyaw));
    if (aOut)
        *aOut = r;
}

// Result codes: approach::Result (0 running, 1 arrived, 2 timeout near, 3 timeout, 4 input abort, 5 no motion, 6 near spot). The override is
// refreshed while running and cleared on any other result.
void N_ApproachStep(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    float px = 0, py = 0, yaw = 0, ref = 0, dt = 0;
    bool yawControl = false;
    RED4ext::GetParameter(aFrame, &px);
    RED4ext::GetParameter(aFrame, &py);
    RED4ext::GetParameter(aFrame, &yaw);
    RED4ext::GetParameter(aFrame, &ref);
    RED4ext::GetParameter(aFrame, &dt);
    RED4ext::GetParameter(aFrame, &yawControl);
    aFrame->code++;
    const double now = Now();
    approach::Command cmd;
    const approach::Result r =
        g_approach.Step({px, py}, yaw, ref, dt, yawControl, g_core.RealMagnitude(now), g_core.RealAngle(now), cmd);
    if (r == approach::Result::Running)
    {
        g_lastCmdX = cmd.stickX;
        g_lastCmdY = cmd.stickY;
        if (!g_core.SetMoveOverride(cmd.stickX, cmd.stickY, cmd.headingDeg, static_cast<double>(g_approachParams.ttlS), now) ||
            !g_nativeArm.Arm(now, static_cast<double>(g_approachParams.ttlS)))
        {
            g_approach.Cancel(); // channel lost (kill switch): never keep walking blind
            g_core.ClearMoveOverride();
            g_nativeArm.Disarm();
            if (aOut)
                *aOut = static_cast<int32_t>(approach::Result::Timeout);
            return;
        }
    }
    else
    {
        g_core.ClearMoveOverride();
        g_nativeArm.Disarm();
    }
    if (aOut)
        *aOut = static_cast<int32_t>(r);
}

void N_ApproachCancel(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    g_approach.Cancel();
    g_core.ClearMoveOverride();
    g_nativeArm.Disarm();
}

void N_ApproachActive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = g_approach.Active();
}

void N_ApproachInfo(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    char tail[320];
    std::snprintf(tail, sizeof(tail), " result=%s residual=%.2f t=%.2f gain=%.2f passed=%d headingErr=%.1fdeg(n=%d) cruise=%.2fm/s overrideSets=%llu overrideReads=%llu nativeApplied=%llu",
                  approach::ResultName(g_approach.Last()), static_cast<double>(g_approach.Residual()),
                  static_cast<double>(g_approach.Elapsed()), static_cast<double>(g_approach.Gain()), g_approach.Passed() ? 1 : 0,
                  static_cast<double>(g_approach.HeadingErrMeanDeg()), g_approach.HeadingErrSamples(),
                  static_cast<double>(g_approach.CruiseSpeedMean()),
                  static_cast<unsigned long long>(g_core.OverrideSets()),
                  static_cast<unsigned long long>(g_core.OverrideAxisReads()),
                  static_cast<unsigned long long>(g_nativeApplied.load()));
    if (aOut)
        *aOut = RED4ext::CString((g_approach.Describe() + tail).c_str());
}

// The 0.1 s tick log line of the approach walk (script passes time, vehicle-frame position and the distance moved since the
// previous line): the last stick command, the native overwrite counter and the reader's hardware vector are added here.
void N_ApproachTickInfo(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    float t = 0.0f, px = 0.0f, py = 0.0f, dpos = 0.0f;
    RED4ext::GetParameter(aFrame, &t);
    RED4ext::GetParameter(aFrame, &px);
    RED4ext::GetParameter(aFrame, &py);
    RED4ext::GetParameter(aFrame, &dpos);
    aFrame->code++;
    // ref = reference yaw of the stick (vehicle frame), yaw = body yaw, velYaw = measured velocity yaw, spotErr = body yaw error vs the
    // clip start yaw, ph = 0 walk / 1 turn-first hold / 2 facing, stick = commanded magnitude, spd = smoothed measured speed (m/s)
    char buf[400];
    std::snprintf(buf, sizeof(buf),
                  "[utpp_vehicle_approach] tick t=%.2f cmd=(%.2f,%.2f) pos=(%.2f,%.2f) dpos=%.3f ref=%.0f yaw=%.0f velYaw=%.0f spotErr=%.0f "
                  "ph=%d stick=%.2f spd=%.2f nativeApplied=%llu hw=(%.2f,%.2f) rdCalls=%llu",
                  static_cast<double>(t), static_cast<double>(g_lastCmdX), static_cast<double>(g_lastCmdY),
                  static_cast<double>(px), static_cast<double>(py), static_cast<double>(dpos), static_cast<double>(g_approach.RefYaw()),
                  static_cast<double>(g_approach.BodyYaw()), static_cast<double>(g_approach.VelYaw()), static_cast<double>(g_approach.YawErr()),
                  g_approach.Phase(), static_cast<double>(g_approach.StickOut()), static_cast<double>(g_approach.Speed()),
                  static_cast<unsigned long long>(g_nativeApplied.load()), static_cast<double>(BitsToFloat(g_hwXBits.load())),
                  static_cast<double>(BitsToFloat(g_hwYBits.load())), static_cast<unsigned long long>(g_readerCalls.load()));
    if (aOut)
        *aOut = RED4ext::CString(buf);
}

template<typename T>
RED4ext::CGlobalFunction* Make(const char* aName, RED4ext::ScriptingFunction_t<T> aFn)
{
    auto* f = RED4ext::CGlobalFunction::Create(aName, aName, aFn);
    f->flags = {.isNative = true, .isStatic = true};
    return f;
}

void RegisterNatives()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
        return;
    auto* f = Make("UTPPLocomotionStatus", &N_Status);
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionSetEnabled", &N_SetEnabled);
    f->AddParam("Bool", "on");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionKill", &N_Kill);
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionSetHold", &N_SetHold);
    f->AddParam("Bool", "on");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionSetBlockMask", &N_SetBlockMask);
    f->AddParam("Int32", "mask");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionSetParams", &N_SetParams);
    f->AddParam("Float", "staleSeconds");
    f->AddParam("Float", "deadzone");
    f->AddParam("Float", "tppCameraDistance");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionHeartbeat", &N_Heartbeat);
    f->AddParam("Bool", "ownerSaysTpp");
    f->AddParam("Float", "cameraDistance2D");
    f->AddParam("Int32", "blockBits");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionActive", &N_Active);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionReasons", &N_Reasons);
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionHasInput", &N_HasInput);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionRawAngle", &N_RawAngle);
    f->SetReturnType("Float");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionDodgeHeading", &N_DodgeHeading);
    f->AddParam("Float", "gameHeading");
    f->SetReturnType("Float");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionSetMoveOverride", &N_SetMoveOverride);
    f->AddParam("Float", "x");
    f->AddParam("Float", "y");
    f->AddParam("Float", "headingDeg");
    f->AddParam("Float", "ttlSeconds");
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionClearMoveOverride", &N_ClearMoveOverride);
    rtti->RegisterFunction(f);

    f = Make("UTPPLocomotionOverrideReady", &N_OverrideReady);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachSetScene", &N_ApproachSetScene);
    f->AddParam("Float", "boxCx");
    f->AddParam("Float", "boxCy");
    f->AddParam("Float", "boxHx");
    f->AddParam("Float", "boxHy");
    f->AddParam("Bool", "arcValid");
    f->AddParam("Float", "hingeX");
    f->AddParam("Float", "hingeY");
    f->AddParam("Float", "arcLength");
    f->AddParam("Float", "arcClosedDeg");
    f->AddParam("Float", "arcOpenDeg");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachBegin", &N_ApproachBegin);
    f->AddParam("Float", "px");
    f->AddParam("Float", "py");
    f->AddParam("Float", "yawDeg");
    f->AddParam("Float", "spotX");
    f->AddParam("Float", "spotY");
    f->AddParam("Float", "spotYawDeg");
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachStep", &N_ApproachStep);
    f->AddParam("Float", "px");
    f->AddParam("Float", "py");
    f->AddParam("Float", "yawDeg");
    f->AddParam("Float", "refYawDeg");
    f->AddParam("Float", "dt");
    f->AddParam("Bool", "yawControl");
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachCancel", &N_ApproachCancel);
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachActive", &N_ApproachActive);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachInfo", &N_ApproachInfo);
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPApproachTickInfo", &N_ApproachTickInfo);
    f->AddParam("Float", "t");
    f->AddParam("Float", "px");
    f->AddParam("Float", "py");
    f->AddParam("Float", "dpos");
    f->SetReturnType("String");
    rtti->RegisterFunction(f);
}
} // namespace

void Init(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    g_hMoveX = RED4ext::CName("MoveX").hash;
    g_hMoveY = RED4ext::CName("MoveY").hash;
    const std::wstring kill = KillFilePath();
    g_killFile = !kill.empty() && GetFileAttributesW(kill.c_str()) != INVALID_FILE_ATTRIBUTES;
    g_initDone = true;
    if (g_killFile)
        LogWarn("UTPP Locomotion: kill file utpp_locomotion.disable present - hook will not be installed");
}

void OnPostRegister(bool aGameValidated)
{
    RegisterNatives();
    if (!g_initDone)
    {
        g_install = freemove::InstallResult::NoClass;
        LogWarn("UTPP Locomotion: Init() not called, hook not installed");
        return;
    }
    g_validated = aGameValidated;
    freemove::InstallFacts facts = EarlyFacts(aGameValidated);
    g_install = freemove::DecideEarly(facts);
    if (g_install != freemove::InstallResult::Pending)
    {
        LogWarn("UTPP Locomotion not installed: code %d (%s)", static_cast<int>(g_install),
                freemove::InstallResultName(g_install));
        return;
    }
    // Diagnostic snapshot only (RTTI read, no decision): shows whether the natives are still unbound here.
    Resolved res;
    Resolve(facts, res);
    LogInfo("UTPP Locomotion: install deferred to the first Running frame (code %d); pre-script RTTI snapshot follows",
            static_cast<int>(g_install));
    LogResolved("post-register", res, facts, false);
}

// A hook or handler slot that could not be removed still points into this module: keep the DLL mapped so a native unload
// or reload cannot leave the game calling freed code.
static void PinThisModule(const char* aWhy)
{
    HMODULE pinned = nullptr;
    const BOOL ok = GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                                       reinterpret_cast<LPCWSTR>(&PinThisModule), &pinned);
    LogWarn("UTPP Locomotion: %s; module %s", aWhy, ok ? "pinned in memory" : "pin FAILED");
}

void OnRunning()
{
    if (g_runningDone.exchange(true, std::memory_order_acq_rel))
        return;
    if (!g_initDone || g_install != freemove::InstallResult::Pending)
        return;
    Install(g_validated);
}

uint32_t CurrentReasons()
{
    return g_core.Reasons(Now());
}

bool LastDodgeHeading(uint64_t& aCalls, float& aServed, float& aEngine, bool& aRewritten, double& aAgeS)
{
    freemove::Core::DodgeHeadingRecord r;
    if (!g_core.LastDodgeHeading(r))
        return false;
    aCalls = r.calls;
    aServed = r.served;
    aEngine = r.engine;
    aRewritten = r.rewritten;
    aAgeS = Now() - r.at;
    return true;
}

void Shutdown()
{
    g_core.Kill();
    g_core.SetInstalled(false);
    g_nativeArm.Disarm();
    if (g_readerAttached.exchange(false) && g_sdk && g_sdk->hooking && g_readerTarget)
    {
        const bool detached = g_sdk->hooking->Detach(g_handle, g_readerTarget);
        LogInfo("UTPP Locomotion native input reader: detach %s", detached ? "ok" : "FAILED");
        if (!detached)
            PinThisModule("reader hook still installed");
    }
    g_readerTarget = nullptr;
    // Restore only if the slot still holds our detour (another module may have swapped it after us).
    if (g_slotInputHeading && SwapSlot(g_slotInputHeading, &DetourInputHeading, g_origInputHeading))
        g_slotInputHeading = nullptr;
    if (g_slotActionValue && SwapSlot(g_slotActionValue, &DetourActionValue, g_origActionValue))
        g_slotActionValue = nullptr;
    if (g_slotInputHeading || g_slotActionValue)
    {
        LogWarn("UTPP Locomotion: handler slot restore skipped (slot no longer ours)");
        PinThisModule("handler slot still points into this module");
    }
}
} // namespace utpp::locomotion

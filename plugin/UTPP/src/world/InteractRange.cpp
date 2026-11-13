// UTPP InteractRange, game side. See world/InteractRangeCore.hpp for the mechanism and the evidence.
// Same resource handling as LootRange (world/LootRange.cpp): the managed descriptors are requested from the resource
// loader once the feature is wanted (the loader de-duplicates by path, so these are the instances the interaction
// components use), walked once (listed layers only), the LookAt leaves captured; every tick each slot steps towards its
// third-person target class or back to vanilla. All raw memory access is SEH-guarded; a fault marks the slot foreign.
// Vehicle descriptors additionally get their morphed seat groups into the compiled non-look-at list
// (InteractRangeCore.hpp StepList) while the morph is complete; the list is restored to its vanilla members before
// the predicates revert.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/Memory/Allocators.hpp>
#include <RED4ext/RED4ext.hpp>
#include <RED4ext/ResourceLoader.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "InteractRange.hpp"
#include "LootHeadLog.hpp"

namespace utpp::interactrange
{
namespace
{
using loothead::LogF;

constexpr double kMinGap = 0.1; // s between vtable swap and payload write

struct Descriptor
{
    const interact::DescriptorSpec* spec = nullptr;
    std::vector<uint64_t> tags;
    RED4ext::SharedPtr<RED4ext::ResourceToken<>>* token = nullptr; // heap, never freed (no engine calls at exit)
    bool requested = false;
    bool walked = false;
    bool failed = false;
    uint32_t layers = 0;
    uint32_t leaves = 0;
    std::string note;
    uint64_t res = 0;         // InteractionDescriptorResource instance (kept loaded by 'token')
    interact::ListPatch list; // compiled non-look-at list (vehicle descriptors)
};

struct State
{
    bool killFile = false;
    bool validated = false;
    bool resolved = false;
    std::string resolveWhy = "not resolved";
    std::string alwaysSameWhy = "not checked";
    bool listOk = false; // resource layout leaves 0xF0..0x100 to native data (RTTI); per-descriptor proof in ProbeList
    std::string listWhy = "not checked";
    interact::CompiledLayout layout;
    bool enabled = false;
    bool killed = false;
    interact::Vtables vt;
    interact::Config cfg;
    loot::DescriptorOffsets offs;
    std::vector<Descriptor> descriptors;
    std::vector<interact::Slot> slots;
    std::vector<size_t> slotDescriptor; // index into descriptors, parallel to slots
    uint64_t ticks = 0;
    uint64_t applies = 0;
    uint64_t reverts = 0;
    bool loadsIssued = false;
};

State* g = nullptr; // heap, never freed

State& S()
{
    if (!g)
        g = new State();
    return *g;
}

// ---------------------------------------------------------------------------------------------
// SEH-guarded raw access (no C++ objects with destructors inside __try).

bool SafeRead64(const void* aAddr, uint64_t& aOut)
{
    __try
    {
        aOut = *static_cast<const volatile uint64_t*>(aAddr);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SafeRead32(const void* aAddr, uint32_t& aOut)
{
    __try
    {
        aOut = *static_cast<const volatile uint32_t*>(aAddr);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SafeCopy(void* aDst, const void* aSrc, size_t aLen)
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

bool SafeXchg64(void* aAddr, uint64_t aValue)
{
    if ((reinterpret_cast<uintptr_t>(aAddr) & 7) != 0)
        return false;
    __try
    {
        _InterlockedExchange64(static_cast<volatile long long*>(aAddr), static_cast<long long>(aValue));
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SafeXchg32(void* aAddr, uint32_t aValue)
{
    if ((reinterpret_cast<uintptr_t>(aAddr) & 3) != 0)
        return false;
    __try
    {
        _InterlockedExchange(static_cast<volatile long*>(aAddr), static_cast<long>(aValue));
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool SafeInc32(void* aAddr)
{
    if (!aAddr || (reinterpret_cast<uintptr_t>(aAddr) & 3) != 0)
        return false;
    __try
    {
        _InterlockedIncrement(static_cast<volatile long*>(aAddr));
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// IAllocator::AllocAligned (vtable slot 1) of the allocator object {vtable}; the call 0x270174 makes for DynArray
// growth.
uint64_t SafeAllocAligned(uint64_t aVtable, uint64_t aBytes, uint32_t aAlign)
{
    uint64_t holder = aVtable;
    auto* alloc = reinterpret_cast<const RED4ext::Memory::IAllocator*>(&holder);
    RED4ext::Memory::AllocationResult r{};
    __try
    {
        r = alloc->AllocAligned(aBytes, aAlign);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
    return r.memory && r.size >= aBytes ? reinterpret_cast<uint64_t>(r.memory) : 0;
}

bool InGameImage(uint64_t aAddr);

struct GameListMemory final : interact::IListMemory
{
    bool R64(uint64_t aAddr, uint64_t& aOut) override
    {
        return aAddr && (aAddr & 7) == 0 && SafeRead64(reinterpret_cast<const void*>(aAddr), aOut);
    }
    bool R32(uint64_t aAddr, uint32_t& aOut) override
    {
        return aAddr && (aAddr & 3) == 0 && SafeRead32(reinterpret_cast<const void*>(aAddr), aOut);
    }
    bool W64(uint64_t aAddr, uint64_t aValue) override { return SafeXchg64(reinterpret_cast<void*>(aAddr), aValue); }
    bool W32(uint64_t aAddr, uint32_t aValue) override { return SafeXchg32(reinterpret_cast<void*>(aAddr), aValue); }
    bool IncStrong(uint64_t aRefCount) override { return SafeInc32(reinterpret_cast<void*>(aRefCount)); }
    uint64_t Alloc(uint64_t aAllocator, uint64_t aBytes, uint32_t aAlign) override
    {
        return SafeAllocAligned(aAllocator, aBytes, aAlign);
    }
    bool InImage(uint64_t aAddr) override { return InGameImage(aAddr); }
};

struct GameMemory final : loot::IObjectMemory
{
    bool Read(void* aObj, uint32_t aOff, uint64_t& aOut) override
    {
        return SafeRead64(static_cast<uint8_t*>(aObj) + aOff, aOut);
    }
    bool Write(void* aObj, uint32_t aOff, uint64_t aValue) override
    {
        return SafeXchg64(static_cast<uint8_t*>(aObj) + aOff, aValue);
    }
};

struct GameReader final : loot::IDescriptorReader
{
    bool Ptr(const void* aBase, uint32_t aOff, void*& aOut) override
    {
        uint64_t v = 0;
        if (!aBase || !SafeRead64(static_cast<const uint8_t*>(aBase) + aOff, v))
            return false;
        aOut = reinterpret_cast<void*>(v);
        return true;
    }
    bool U32(const void* aBase, uint32_t aOff, uint32_t& aOut) override
    {
        return aBase && SafeRead32(static_cast<const uint8_t*>(aBase) + aOff, aOut);
    }
    bool U64(const void* aBase, uint32_t aOff, uint64_t& aOut) override
    {
        return aBase && SafeRead64(static_cast<const uint8_t*>(aBase) + aOff, aOut);
    }
    bool IsLookAt(void* aObj) override
    {
        uint64_t vt = 0;
        return aObj && (reinterpret_cast<uintptr_t>(aObj) & 7) == 0 && SafeRead64(aObj, vt) && vt == S().vt.lookAt;
    }
};

// ---------------------------------------------------------------------------------------------
// RTTI proof.

bool PropAt(RED4ext::CClass* aCls, const char* aProp, uint32_t aExpected, std::string& aWhy)
{
    auto* p = aCls ? aCls->GetProperty(RED4ext::CName(aProp)) : nullptr;
    if (!p)
    {
        aWhy = std::string("property missing: ") + (aCls ? aCls->GetName().ToString() : "?") + "." + aProp;
        return false;
    }
    if (p->valueOffset != aExpected)
    {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "property %s.%s at 0x%X, expected 0x%X", aCls->GetName().ToString(), aProp,
                      p->valueOffset, aExpected);
        aWhy = buf;
        return false;
    }
    return true;
}

bool VtableOf(RED4ext::CClass* aCls, uint64_t& aOut)
{
    void* inst = aCls->CreateInstance(true); // kept for the session (<= 64 bytes), never released
    if (!inst)
        return false;
    return SafeRead64(inst, aOut) && aOut != 0;
}

bool InGameImage(uint64_t aAddr)
{
    const auto* base = reinterpret_cast<const uint8_t*>(GetModuleHandleW(nullptr));
    if (!base)
        return false;
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    const uint64_t b = reinterpret_cast<uint64_t>(base);
    return aAddr >= b && aAddr < b + nt->OptionalHeader.SizeOfImage;
}

bool ResolveRtti(std::string& aWhy)
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
    {
        aWhy = "no RTTI system";
        return false;
    }
    auto cls = [&](const char* aName) { return rtti->GetClass(RED4ext::CName(aName)); };
    RED4ext::CClass* base = cls("gameinteractionsIPredicateType");
    RED4ext::CClass* lookAt = cls("gameinteractionsLookAtPredicate");
    RED4ext::CClass* window = cls("gameinteractionsDistanceFromScreenCenterPredicate");
    RED4ext::CClass* res = cls("gameinteractionsInteractionDescriptorResource");
    RED4ext::CClass* hsDef = cls("gameinteractionsCHotSpotDefinition");
    RED4ext::CClass* linked = cls("gameinteractionsCLinkedLayersDefinition");
    RED4ext::CClass* layer = cls("gameinteractionsCHotSpotLayerDefinition");
    RED4ext::CClass* area = cls("gameinteractionsCHotSpotAreaFilterDefinition");
    RED4ext::CClass* functor = cls("gameinteractionsCFunctorDefinition");
    RED4ext::CClass* pred = cls("gameinteractionsCPredicateDefinition");
    if (!base || !lookAt || !window || !res || !hsDef || !linked || !layer || !area || !functor || !pred)
    {
        aWhy = "interaction RTTI classes missing";
        return false;
    }
    if (!lookAt->IsA(base) || !window->IsA(base))
    {
        aWhy = "predicate classes do not derive from gameinteractionsIPredicateType";
        return false;
    }
    if (lookAt->GetSize() != 0x40 || window->GetSize() != 0x40 || lookAt->GetAlignment() > 16 ||
        window->GetAlignment() > 16)
    {
        aWhy = "predicate sizes differ from 0x40";
        return false;
    }
    const auto& o = S().offs;
    if (!PropAt(lookAt, "testTarget", 0x30, aWhy) || !PropAt(lookAt, "testType", 0x34, aWhy) ||
        !PropAt(lookAt, "stopOnTransparent", 0x38, aWhy) || !PropAt(window, "height", 0x30, aWhy) ||
        !PropAt(window, "width", 0x34, aWhy) || !PropAt(window, "curvature", 0x38, aWhy) ||
        !PropAt(window, "maxPriorityBoundsFactor", 0x3C, aWhy) || !PropAt(res, "definition", o.resDefinition, aWhy) ||
        !PropAt(hsDef, "layersDefinition", o.defLayers, aWhy) ||
        !PropAt(linked, "layersDefinitions", o.linkedLayers, aWhy) || !PropAt(layer, "tag", o.layerTag, aWhy) ||
        !PropAt(layer, "areaFilterDefinition", o.layerAreaFilter, aWhy) || !PropAt(area, "functor", o.areaFunctor, aWhy) ||
        !PropAt(functor, "predicate", o.functorPredicate, aWhy) || !PropAt(pred, "predicateType", o.predType, aWhy) ||
        !PropAt(pred, "functor1DataDefinition", o.predFunctor1, aWhy) ||
        !PropAt(pred, "functor2DataDefinition", o.predFunctor2, aWhy))
        return false;
    interact::Vtables vt;
    if (!VtableOf(lookAt, vt.lookAt) || !VtableOf(window, vt.window) || vt.lookAt == vt.window ||
        !InGameImage(vt.lookAt) || !InGameImage(vt.window))
    {
        aWhy = "predicate vtables not proven (instance creation or image range)";
        return false;
    }

    // AlwaysSame (vehicle proximity mode). Any mismatch only disables that mode (vehicles then use the window).
    S().alwaysSameWhy = "ok";
    RED4ext::CClass* same = cls("gameinteractionsAlwaysSamePredicate");
    std::string why;
    uint64_t sameVt = 0;
    if (!same)
        why = "class missing";
    else if (!same->IsA(base))
        why = "not an IPredicateType";
    else if (same->GetSize() < 0x34 || same->GetSize() > 0x40 || same->GetAlignment() > 16)
        why = "unexpected size";
    else if (!PropAt(same, "priority", 0x30, why))
        ;
    else if (!VtableOf(same, sameVt) || !InGameImage(sameVt) || sameVt == vt.lookAt || sameVt == vt.window)
        why = "vtable not proven";
    else
    {
        uint64_t fn = 0;
        uint8_t code[sizeof(interact::kAlwaysSameEvalCode)] = {};
        if (!SafeRead64(reinterpret_cast<const void*>(sameVt + 8ull * interact::kEvalSlot), fn) || !InGameImage(fn) ||
            !SafeCopy(code, reinterpret_cast<const void*>(fn), sizeof(code)))
            why = "evaluate slot unreadable";
        else if (!interact::IsAlwaysSameEval(code, sizeof(code)))
            why = "evaluate code differs from 'return priority'";
    }
    if (why.empty())
        vt.alwaysSame = sameVt;
    else
        S().alwaysSameWhy = why;

    // Compiled non-look-at list: resource+0xF0 must be native-only storage (no RTTI property over 0xF0..0x100) inside
    // the instance. Any mismatch only disables the list patch; each descriptor is proven again in ProbeList.
    {
        const uint32_t lo = S().layout.resCompiled, hi = lo + 16;
        std::string lw;
        if (res->GetSize() < hi)
            lw = "resource class smaller than 0x100";
        for (RED4ext::CClass* c = res; c && lw.empty(); c = c->parent)
            for (const RED4ext::CProperty* p : c->props)
            {
                const uint32_t ps = p && p->type ? p->type->GetSize() : 0;
                if (p && p->valueOffset < hi && p->valueOffset + (ps ? ps : 1) > lo)
                {
                    lw = std::string("RTTI property over 0xF0: ") + p->name.ToString();
                    break;
                }
            }
        S().listOk = lw.empty();
        S().listWhy = lw.empty() ? "ok" : lw;
    }
    S().vt = vt;
    return true;
}

// ---------------------------------------------------------------------------------------------

loot::Inputs MakeInputs(bool aTpp)
{
    const State& s = S();
    loot::Inputs in;
    in.resolved = s.resolved;
    in.enabled = s.enabled;
    in.killed = s.killed;
    in.tpp = aTpp;
    in.killFile = s.killFile;
    in.validated = s.validated;
    in.haveSlots = !s.slots.empty();
    return in;
}

const char* ShortName(const char* aPath)
{
    const char* slash = std::strrchr(aPath, '\\');
    return slash ? slash + 1 : aPath;
}

std::string ListLine(const Descriptor& aD)
{
    GameListMemory lm;
    const interact::ListProbe p = interact::ProbeList(lm, S().layout, S().offs, aD.res);
    return interact::ListSummary(p, aD.list);
}

// One list step for a descriptor; logs every state change (one line < 500 chars).
void StepDescriptorList(Descriptor& aD, const std::vector<uint64_t>& aWant, double aNow)
{
    if (!aD.res || aD.list.managed.empty())
        return;
    const interact::ListState before = aD.list.state;
    const uint32_t g0 = aD.list.grows, a0 = aD.list.adds, r0 = aD.list.removes, x0 = aD.list.replaced;
    GameListMemory lm;
    interact::StepList(lm, S().layout, S().offs, aD.res, aD.list, aWant, aNow, kMinGap);
    if (aD.list.state != before || aD.list.grows != g0 || aD.list.adds != a0 || aD.list.removes != r0 ||
        aD.list.replaced != x0)
        LogF(aD.list.state == interact::ListState::Foreign ? 1 : 0, "UTPP InteractRange %s: %s",
             ShortName(aD.spec->path), ListLine(aD).c_str());
    // Foreign is final, so this is logged once per descriptor.
    if (before != interact::ListState::Foreign && aD.list.state == interact::ListState::Foreign &&
        !aD.list.pinned.empty())
        LogF(1, "UTPP InteractRange %s: %zu seat group(s) could not be taken out of the non-look-at list; their "
                "predicates stay morphed for the rest of the session",
             ShortName(aD.spec->path), aD.list.pinned.size());
}

// Slot whose group may still be in the non-look-at list after a failed list patch: kept morphed (never reverted).
bool SlotPinned(size_t aSlot)
{
    const State& s = S();
    return interact::Pins(s.descriptors[s.slotDescriptor[aSlot]].list,
                          reinterpret_cast<uint64_t>(s.slots[aSlot].groupObj));
}

// Managed groups of a descriptor whose captured LookAt leaves are all morphed (empty unless aWant).
std::vector<uint64_t> MorphedGroups(size_t aDi, bool aWant)
{
    std::vector<uint64_t> out;
    State& s = S();
    if (!aWant)
        return out;
    for (uint64_t grp : s.descriptors[aDi].list.managed)
    {
        bool all = true, any = false;
        for (size_t i = 0; i < s.slots.size(); ++i)
            if (s.slotDescriptor[i] == aDi && reinterpret_cast<uint64_t>(s.slots[i].groupObj) == grp)
            {
                any = true;
                all = all && s.slots[i].s.state == loot::SlotState::Morphed;
            }
        if (any && all)
            out.push_back(grp);
    }
    return out;
}

void RevertLists(double aNow)
{
    for (auto& d : S().descriptors)
        if (d.list.state == interact::ListState::Added)
            StepDescriptorList(d, {}, aNow);
}

// Returns the number of predicates kept morphed because their group is pinned in the list.
uint32_t RevertAll(const char* aWhy, double aNow = 0.0)
{
    State& s = S();
    RevertLists(aNow); // list back to its vanilla members before the predicates are restored
    GameMemory mem;
    uint32_t n = 0, kept = 0;
    for (size_t i = 0; i < s.slots.size(); ++i)
    {
        auto& slot = s.slots[i];
        const bool was = slot.s.state == loot::SlotState::Morphed || slot.s.state == loot::SlotState::VtableSwapped;
        if (was && SlotPinned(i))
        {
            ++kept;
            continue;
        }
        if (interact::RevertInteractSlot(mem, slot, s.vt) && was)
            ++n;
    }
    if (n)
    {
        s.reverts += n;
        LogF(0, "UTPP InteractRange: %u predicate(s) restored (%s)", n, aWhy);
    }
    return kept;
}

void IssueLoads()
{
    State& s = S();
    if (s.loadsIssued)
        return;
    s.loadsIssued = true;
    auto* loader = RED4ext::ResourceLoader::Get();
    for (auto& d : s.descriptors)
    {
        if (d.requested)
            continue;
        d.requested = true;
        if (!loader)
        {
            d.failed = true;
            d.note = "no resource loader";
            continue;
        }
        d.token = new RED4ext::SharedPtr<RED4ext::ResourceToken<>>(
            loader->LoadAsync<RED4ext::CResource>(RED4ext::ResourcePath(d.spec->path)));
        if (!d.token || !*d.token)
        {
            d.failed = true;
            d.note = "LoadAsync returned no token";
        }
    }
}

void WalkLoaded()
{
    State& s = S();
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* resCls = rtti ? rtti->GetClass(RED4ext::CName("gameinteractionsInteractionDescriptorResource")) : nullptr;
    GameReader reader;
    GameMemory mem;
    for (size_t di = 0; di < s.descriptors.size(); ++di)
    {
        Descriptor& d = s.descriptors[di];
        if (d.walked || d.failed || !d.token || !*d.token)
            continue;
        RED4ext::ResourceToken<>* tok = d.token->GetPtr();
        if (tok->IsFailed())
        {
            d.failed = true;
            d.note = "load failed";
            LogF(1, "UTPP InteractRange: descriptor load failed: %s", d.spec->path);
            continue;
        }
        if (!tok->IsLoaded())
            continue;
        d.walked = true;
        RED4ext::CResource* res = tok->resource.GetPtr();
        if (!res || !resCls || res->GetType() != resCls)
        {
            d.failed = true;
            d.note = "not an InteractionDescriptorResource";
            LogF(1, "UTPP InteractRange: %s is not an InteractionDescriptorResource", d.spec->path);
            continue;
        }
        const interact::GroupWalk w = interact::WalkGroupsTags(reader, s.offs, res, d.tags.data(), d.tags.size());
        d.layers = w.layers;
        d.res = reinterpret_cast<uint64_t>(res);
        uint32_t added = 0;
        for (const auto& gl : w.groups)
        {
            size_t captured = 0;
            for (void* leaf : gl.leaves)
            {
                bool known = false;
                for (const auto& sl : s.slots)
                    known = known || sl.s.obj == leaf;
                if (known)
                    continue;
                interact::Slot slot;
                slot.group = d.spec->group;
                slot.groupObj = gl.group;
                if (loot::CaptureSlot(mem, leaf, loot::VtablePair{s.vt.lookAt, s.vt.window}, slot.s))
                {
                    s.slots.push_back(slot);
                    s.slotDescriptor.push_back(di);
                    ++added;
                    ++captured;
                }
            }
            // Only vehicle seat groups go into the non-look-at list; device / talk descriptors are probed read-only.
            // A group qualifies only when every LookAt leaf in all of its layers is captured here (morphed with it).
            if (d.spec->group == interact::Group::Vehicle && s.listOk)
            {
                if (const char* why = interact::ListIneligible(gl, captured))
                    LogF(1, "UTPP InteractRange %s: group %p not added to the non-look-at list (%s)",
                         ShortName(d.spec->path), gl.group, why);
                else
                    d.list.managed.push_back(reinterpret_cast<uint64_t>(gl.group));
            }
        }
        d.leaves = added;
        if (w.readError || w.truncated)
            d.note = w.readError ? "read error during walk" : "walk truncated";
        LogF(w.readError ? 1 : 0, "UTPP InteractRange: %s (%s): layers %u, LookAt leaves %u captured%s%s", d.spec->path,
             interact::GroupName(d.spec->group), w.layers, added, w.readError ? " (read error)" : "",
             w.truncated ? " (truncated)" : "");
        // Read-only probe of the load-time classification (all managed descriptors; device / talk = controls).
        LogF(0, "UTPP InteractRange %s: %s", ShortName(d.spec->path), ListLine(d).c_str());
    }
}
} // namespace

void Init(bool aKillFile)
{
    State& s = S();
    s.killFile = s.killFile || aKillFile;
    if (!s.descriptors.empty())
        return;
    size_t n = 0;
    const interact::DescriptorSpec* specs = interact::Descriptors(n);
    for (size_t i = 0; i < n; ++i)
    {
        Descriptor d;
        d.spec = &specs[i];
        for (size_t k = 0; k < interact::TagCount(specs[i]); ++k)
            d.tags.push_back(RED4ext::CName(specs[i].tags[k]).hash);
        s.descriptors.push_back(d);
    }
}

void Resolve(bool aGameValidated)
{
    State& s = S();
    s.validated = aGameValidated;
    if (s.killFile || !aGameValidated)
    {
        s.resolveWhy = s.killFile ? "kill file" : "game not validated";
        LogF(1, "UTPP InteractRange not resolved: %s", s.resolveWhy.c_str());
        return;
    }
    std::string why;
    s.resolved = ResolveRtti(why);
    s.resolveWhy = s.resolved ? "ok" : why;
    if (s.resolved)
        LogF(s.vt.alwaysSame ? 0 : 1,
             "UTPP InteractRange: resolved (LookAt vt 0x%llX, window vt 0x%llX, AlwaysSame vt 0x%llX: %s; "
             "seat list patch: %s); default disabled",
             static_cast<unsigned long long>(s.vt.lookAt), static_cast<unsigned long long>(s.vt.window),
             static_cast<unsigned long long>(s.vt.alwaysSame),
             s.vt.alwaysSame ? "vehicle proximity available" : s.alwaysSameWhy.c_str(), s.listWhy.c_str());
    else
        LogF(1, "UTPP InteractRange not resolved: %s", why.c_str());
}

void Tick(double aNow, bool aTpp)
{
    State& s = S();
    ++s.ticks;
    if (!s.resolved)
        return;
    const loot::Inputs in = MakeInputs(aTpp);
    const bool want = loot::WantMorph(in);
    if (want || s.loadsIssued)
    {
        IssueLoads();
        WalkLoaded();
    }
    if (!want)
    {
        RevertAll(s.killed ? "killed" : (!s.enabled ? "disabled" : "not TPP"), aNow);
        return;
    }
    // List first: a group leaves the list before any of its leaves is reverted (mode change) in the same tick.
    const bool listMode = s.listOk && interact::ListPatchMode(s.cfg.vehicleMode);
    for (size_t di = 0; di < s.descriptors.size(); ++di)
    {
        Descriptor& d = s.descriptors[di];
        if (d.list.state == interact::ListState::Added)
        {
            std::vector<uint64_t> keep = MorphedGroups(di, listMode);
            // Keep only groups whose leaves stay morphed under the current target class (no mode switch pending).
            bool stays = listMode;
            for (size_t i = 0; i < s.slots.size() && stays; ++i)
                if (s.slotDescriptor[i] == di && s.slots[i].s.state == loot::SlotState::Morphed)
                    stays = interact::TargetFor(s.slots[i], true, s.vt, s.cfg).vt == s.slots[i].morphVt;
            if (!stays)
                keep.clear();
            if (keep != d.list.added)
                StepDescriptorList(d, keep, aNow);
        }
    }
    GameMemory mem;
    for (size_t i = 0; i < s.slots.size(); ++i)
    {
        auto& slot = s.slots[i];
        if (slot.s.state == loot::SlotState::Morphed && SlotPinned(i))
            continue; // group possibly still listed: no mode change / revert for this predicate
        const loot::SlotState before = slot.s.state;
        interact::StepInteractSlot(mem, slot, true, s.vt, s.cfg, aNow, kMinGap);
        if (before != loot::SlotState::Morphed && slot.s.state == loot::SlotState::Morphed)
            ++s.applies;
        if (before != loot::SlotState::Vanilla && slot.s.state == loot::SlotState::Vanilla)
            ++s.reverts;
        if (slot.s.state == loot::SlotState::Foreign && before != loot::SlotState::Foreign)
            LogF(1, "UTPP InteractRange: predicate %p became foreign (unexpected vtable / access failure) - left alone",
                 slot.s.obj);
    }
    // Then add the groups whose leaves are now completely morphed (Morphed implies the current target class).
    if (!listMode)
        return;
    for (size_t di = 0; di < s.descriptors.size(); ++di)
    {
        Descriptor& d = s.descriptors[di];
        if (d.list.managed.empty() || d.list.state == interact::ListState::Foreign)
            continue;
        const std::vector<uint64_t> addSet = MorphedGroups(di, true);
        if (addSet.empty() || (d.list.state == interact::ListState::Added && addSet == d.list.added))
            continue;
        StepDescriptorList(d, addSet, aNow);
    }
}

void Shutdown()
{
    if (!g)
        return;
    g->killed = true;
    RevertAll("unload");
}

void SetEnabled(bool aOn)
{
    S().enabled = aOn;
    LogF(0, "UTPPInteractRangeSetEnabled(%d) resolved=%d killed=%d", aOn ? 1 : 0, S().resolved ? 1 : 0,
         S().killed ? 1 : 0);
}

void Kill()
{
    S().killed = true;
    const uint32_t kept = RevertAll("killed");
    if (kept)
        LogF(1, "UTPPInteractRangeKill(): vanilla interaction look-at restored for the rest of the session, except %u "
                "predicate(s) of seat group(s) that could not be taken out of the non-look-at list",
             kept);
    else
        LogF(1, "UTPPInteractRangeKill(): vanilla interaction look-at restored for the rest of the session");
}

bool SetScale(float aScale)
{
    loot::WindowParams w;
    if (!loot::ScaledWindow(aScale, w))
        return false;
    S().cfg.window = w;
    return true;
}

bool SetVehicleMode(int32_t aMode)
{
    if (!interact::ValidVehicleMode(aMode))
        return false;
    S().cfg.vehicleMode = static_cast<interact::VehicleMode>(aMode);
    return true;
}

bool SetVehiclePriority(float aPriority)
{
    if (!interact::ValidPriority(aPriority))
        return false;
    S().cfg.vehiclePriority = aPriority;
    return true;
}

bool Active()
{
    for (const auto& sl : S().slots)
        if (sl.s.state == loot::SlotState::Morphed)
            return true;
    return false;
}

uint32_t Reasons(bool aTpp)
{
    return loot::Reasons(MakeInputs(aTpp));
}

std::string Status(double aNow, bool aTpp)
{
    const State& s = S();
    uint32_t morphed = 0, swapped = 0, vanilla = 0, foreign = 0, prox = 0;
    for (const auto& sl : s.slots)
    {
        switch (sl.s.state)
        {
        case loot::SlotState::Morphed:
            ++morphed;
            prox += sl.morphVt != 0 && sl.morphVt == s.vt.alwaysSame ? 1u : 0u;
            break;
        case loot::SlotState::VtableSwapped: ++swapped; break;
        case loot::SlotState::Vanilla: ++vanilla; break;
        case loot::SlotState::Foreign: ++foreign; break;
        }
    }
    std::string desc;
    for (const auto& d : s.descriptors)
    {
        char buf[220];
        std::snprintf(buf, sizeof(buf), "\n [%s %s %s L%u x%u%s%s]", ShortName(d.spec->path),
                      interact::GroupName(d.spec->group),
                      d.failed ? "FAILED" : (d.walked ? "walked" : (d.requested ? "loading" : "idle")), d.layers, d.leaves,
                      d.note.empty() ? "" : " ", d.note.c_str());
        desc += buf;
        if (d.res) // live read-only probe of the compiled non-look-at list
            desc += " " + ListLine(d);
    }
    char buf[900];
    std::snprintf(buf, sizeof(buf),
                  "UTPP InteractRange resolved=%d (%s) alwaysSame=%s list=%s enabled=%d killed=%d killFile=%d "
                  "reasons=0x%X tpp=%d "
                  "slots=%zu morphed=%u (proximity %u) swapped=%u vanilla=%u foreign=%u applies=%llu reverts=%llu "
                  "vehicleMode=%d priority=%.2f window=%.2fx%.2f c%.2f p%.2f ticks=%llu t=%.1f descriptors:",
                  s.resolved ? 1 : 0, s.resolveWhy.c_str(), s.vt.alwaysSame ? "ok" : s.alwaysSameWhy.c_str(),
                  s.listWhy.c_str(), s.enabled ? 1 : 0, s.killed ? 1 : 0, s.killFile ? 1 : 0,
                  loot::Reasons(MakeInputs(aTpp)), aTpp ? 1 : 0,
                  s.slots.size(), morphed, prox, swapped, vanilla, foreign, static_cast<unsigned long long>(s.applies),
                  static_cast<unsigned long long>(s.reverts), static_cast<int>(s.cfg.vehicleMode),
                  static_cast<double>(s.cfg.vehiclePriority), static_cast<double>(s.cfg.window.width),
                  static_cast<double>(s.cfg.window.height), static_cast<double>(s.cfg.window.curvature),
                  static_cast<double>(s.cfg.window.maxPriorityBoundsFactor), static_cast<unsigned long long>(s.ticks), aNow);
    return std::string(buf) + desc;
}

std::string DescribeComponent(RED4ext::IScriptable* aComponent)
{
    if (!aComponent)
        return "no interaction component";
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* cls = rtti ? rtti->GetClass(RED4ext::CName("gameinteractionsComponent")) : nullptr;
    RED4ext::CClass* type = aComponent->GetType();
    if (!cls || !type || !type->IsA(cls))
        return std::string("not an InteractionComponent (") + (type ? type->GetName().ToString() : "?") + ")";
    RED4ext::CProperty* prop = cls->GetProperty(RED4ext::CName("definitionResource"));
    if (!prop || !prop->type || prop->type->GetType() != RED4ext::rtti::ERTTIType::ResourceReference)
        return "InteractionComponent.definitionResource missing or not a resource reference";
    uint64_t hash = 0;
    if (!SafeRead64(reinterpret_cast<const uint8_t*>(aComponent) + prop->valueOffset, hash))
        return "definitionResource unreadable";
    // Ref = {ResourcePath path @0, SharedPtr<ResourceToken> token @8}; ResourceToken.resource (Handle) @0x28.
    uint64_t compToken = 0, compRes = 0;
    const bool tokenRead = SafeRead64(reinterpret_cast<const uint8_t*>(aComponent) + prop->valueOffset + 8, compToken);
    if (tokenRead && compToken)
        SafeRead64(reinterpret_cast<const void*>(compToken + 0x28), compRes);
    char head[96];
    std::snprintf(head, sizeof(head), "descriptor 0x%016llX", static_cast<unsigned long long>(hash));
    std::string out = head;
    const State& s = S();
    for (size_t di = 0; di < s.descriptors.size(); ++di)
    {
        const Descriptor& d = s.descriptors[di];
        if (RED4ext::ResourcePath(d.spec->path).hash != hash)
            continue;
        uint32_t morphed = 0, total = 0;
        for (size_t i = 0; i < s.slots.size(); ++i)
            if (s.slotDescriptor[i] == di)
            {
                ++total;
                morphed += s.slots[i].s.state == loot::SlotState::Morphed ? 1u : 0u;
            }
        out += std::string(" = ") + d.spec->path + " (UTPP interact " + interact::GroupName(d.spec->group) + ", layers";
        for (size_t k = 0; k < interact::TagCount(*d.spec); ++k)
            out += std::string(" ") + d.spec->tags[k];
        // Same resource instance as the one walked by UTPP = the morphed predicate objects are the component's.
        uint64_t ourToken = 0, ourRes = 0;
        if (d.token && *d.token)
        {
            ourToken = reinterpret_cast<uint64_t>(d.token->GetPtr());
            ourRes = reinterpret_cast<uint64_t>(d.token->GetPtr()->resource.GetPtr());
        }
        const char* same = !compToken ? "component token not set"
                           : !ourRes  ? "UTPP copy not loaded yet"
                           : (compToken == ourToken || compRes == ourRes) ? "same resource instance"
                                                                           : "DIFFERENT resource instance";
        char buf[160];
        std::snprintf(buf, sizeof(buf), "; LookAt leaves %u, morphed %u; %s)", total, morphed, same);
        out += buf;
    }
    size_t nl = 0;
    const char* const* loot = interact::LootDescriptorPaths(nl);
    for (size_t i = 0; i < nl; ++i)
        if (RED4ext::ResourcePath(loot[i]).hash == hash)
            out += std::string(" = ") + loot[i] + " (UTPP loot range: Loot layer)";
    if (out.find(" = ") == std::string::npos)
        out += hash ? " (not managed by UTPP)" : " (empty: definition inline or overridden)";
    return out;
}
} // namespace utpp::interactrange

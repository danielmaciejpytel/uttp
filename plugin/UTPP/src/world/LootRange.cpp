// UTPP LootRange (Phase 5N step 5a), game side. See world/LootRangeCore.hpp for the mechanism and the evidence.
// Built from the 2.31 interaction descriptors (WolvenKit JSON export) and RED4ext.SDK.
//
// Flow (game thread only):
//   Resolve (PostRegister): RTTI proof - LookAtPredicate and DistanceFromScreenCenterPredicate exist, both derive from
//     gameinteractionsIPredicateType, both are 0x40 bytes, the payload properties sit at the offsets the core expects,
//     every walked property has the offset the core uses; vtables are taken from fresh instances of both classes.
//   Tick: when wanted for the first time, the loot descriptors are requested from the resource loader (the tokens are
//     kept, so the resources stay resident and are the same instances the interaction components use - the loader
//     de-duplicates tokens by path); loaded descriptors are walked once (Loot layers only), LookAt leaves captured;
//     every tick each slot steps towards morphed (TPP) or vanilla (anything else).
// All raw memory access is SEH-guarded; a fault marks the slot foreign (never written again).

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/ResourceLoader.hpp>

#include <string>
#include <vector>

#include "LootHeadLog.hpp"
#include "LootRange.hpp"

namespace utpp::lootrange
{
namespace
{
using loothead::LogF;

constexpr double kMinGap = 0.1; // s between vtable swap and payload write (in-flight LookAt calls have returned)

const char* const kDefaultDescriptors[] = {
    "base\\gameplay\\containers\\generic_container_interaction.interaction",
    "base\\gameplay\\containers\\generic_singular_container_interaction.interaction",
    "base\\gameplay\\interactions\\npc\\default_puppet_interaction.interaction",
    "base\\gameplay\\interactions\\npc\\mechanical_puppet_interaction.interaction",
};

struct Descriptor
{
    std::string path;
    RED4ext::SharedPtr<RED4ext::ResourceToken<>>* token = nullptr; // heap, never freed (no engine calls at exit)
    bool requested = false;
    bool walked = false;
    bool failed = false;
    uint32_t lootLayers = 0;
    uint32_t leaves = 0;
    std::string note;
};

struct State
{
    bool killFile = false;
    bool validated = false;
    bool resolved = false;
    std::string resolveWhy = "not resolved";
    bool enabled = false;
    bool killed = false;
    loot::VtablePair vt;
    loot::DescriptorOffsets offs;
    uint64_t lootTag = 0;
    loot::WindowParams window;
    std::vector<Descriptor> descriptors;
    std::vector<loot::Slot> slots;
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
    void* inst = aCls->CreateInstance(true); // kept for the session (64 bytes), never released
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
        char buf[160];
        std::snprintf(buf, sizeof(buf), "predicate sizes LookAt 0x%X DistanceFromScreenCenter 0x%X (expected 0x40)",
                      lookAt->GetSize(), window->GetSize());
        aWhy = buf;
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
    // Size 0x40 + the four DistanceFromScreenCenter properties at 0x30/0x34/0x38/0x3C: the swapped payload is exactly
    // 0x30..0x40 (LookAt bytes 0x39..0x3F are padding and are restored from the captured copy anyway).
    loot::VtablePair vt;
    if (!VtableOf(lookAt, vt.lookAt) || !VtableOf(window, vt.window) || vt.lookAt == vt.window ||
        !InGameImage(vt.lookAt) || !InGameImage(vt.window))
    {
        aWhy = "predicate vtables not proven (instance creation or image range)";
        return false;
    }
    S().vt = vt;
    S().lootTag = RED4ext::CName("Loot").hash;
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

void RevertAll(const char* aWhy)
{
    State& s = S();
    GameMemory mem;
    uint32_t n = 0;
    for (auto& slot : s.slots)
    {
        const bool was = slot.state == loot::SlotState::Morphed || slot.state == loot::SlotState::VtableSwapped;
        if (loot::RevertSlot(mem, slot, s.vt) && was)
            ++n;
    }
    if (n)
    {
        s.reverts += n;
        LogF(0, "UTPP LootRange: %u predicate(s) restored (%s)", n, aWhy);
    }
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
            loader->LoadAsync<RED4ext::CResource>(RED4ext::ResourcePath(d.path.c_str())));
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
    for (auto& d : s.descriptors)
    {
        if (d.walked || d.failed || !d.token || !*d.token)
            continue;
        RED4ext::ResourceToken<>* tok = d.token->GetPtr();
        if (tok->IsFailed())
        {
            d.failed = true;
            d.note = "load failed";
            LogF(1, "UTPP LootRange: descriptor load failed: %s", d.path.c_str());
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
            LogF(1, "UTPP LootRange: %s is not an InteractionDescriptorResource", d.path.c_str());
            continue;
        }
        const loot::WalkResult w = loot::WalkDescriptor(reader, s.offs, res, s.lootTag);
        d.lootLayers = w.lootLayers;
        uint32_t added = 0;
        for (void* leaf : w.leaves)
        {
            bool known = false;
            for (const auto& sl : s.slots)
                known = known || sl.obj == leaf;
            if (known)
                continue;
            loot::Slot slot;
            if (loot::CaptureSlot(mem, leaf, s.vt, slot))
            {
                s.slots.push_back(slot);
                ++added;
            }
        }
        d.leaves = added;
        if (w.readError || w.truncated)
            d.note = w.readError ? "read error during walk" : "walk truncated";
        LogF(w.readError ? 1 : 0, "UTPP LootRange: %s: Loot layers %u, LookAt leaves %u captured%s%s", d.path.c_str(),
             w.lootLayers, added, w.readError ? " (read error)" : "", w.truncated ? " (truncated)" : "");
    }
}
} // namespace

void Init(bool aKillFile)
{
    State& s = S();
    s.killFile = s.killFile || aKillFile;
    if (!s.descriptors.empty())
        return;
    for (const char* p : kDefaultDescriptors)
    {
        Descriptor d;
        d.path = p;
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
        LogF(1, "UTPP LootRange not resolved: %s", s.resolveWhy.c_str());
        return;
    }
    std::string why;
    s.resolved = ResolveRtti(why);
    s.resolveWhy = s.resolved ? "ok" : why;
    if (s.resolved)
        LogF(0, "UTPP LootRange: resolved (LookAt vt 0x%llX, DistanceFromScreenCenter vt 0x%llX); default disabled",
             static_cast<unsigned long long>(s.vt.lookAt), static_cast<unsigned long long>(s.vt.window));
    else
        LogF(1, "UTPP LootRange not resolved: %s", why.c_str());
}

void Tick(double aNow, bool aTpp)
{
    State& s = S();
    ++s.ticks;
    if (!s.resolved)
        return;
    loot::Inputs in = MakeInputs(aTpp);
    const bool want = loot::WantMorph(in);
    // Resources are only requested once the feature is wanted (enabled, TPP, not killed).
    if (want || s.loadsIssued)
    {
        IssueLoads();
        WalkLoaded();
    }
    if (!want)
    {
        RevertAll(s.killed ? "killed" : (!s.enabled ? "disabled" : "not TPP"));
        return;
    }
    GameMemory mem;
    for (auto& slot : s.slots)
    {
        const loot::SlotState before = slot.state;
        loot::StepSlot(mem, slot, s.vt, true, s.window, aNow, kMinGap);
        if (before != loot::SlotState::Morphed && slot.state == loot::SlotState::Morphed)
            ++s.applies;
        if (slot.state == loot::SlotState::Foreign && before != loot::SlotState::Foreign)
            LogF(1, "UTPP LootRange: predicate %p became foreign (unexpected vtable / access failure) - left alone",
                 slot.obj);
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
    LogF(0, "UTPPLootRangeSetEnabled(%d) resolved=%d killed=%d", aOn ? 1 : 0, S().resolved ? 1 : 0,
         S().killed ? 1 : 0);
}

void Kill()
{
    S().killed = true;
    RevertAll("killed");
    LogF(1, "UTPPLootRangeKill(): vanilla look-at restored for the rest of the session");
}

bool SetWindow(const loot::WindowParams& aW)
{
    if (!loot::ValidWindow(aW))
        return false;
    S().window = aW;
    return true;
}

bool AddDescriptor(const std::string& aPath)
{
    State& s = S();
    if (aPath.size() < 13 || aPath.size() > 260 || aPath.find(".interaction") == std::string::npos ||
        s.descriptors.size() >= 32)
        return false;
    for (const auto& d : s.descriptors)
        if (d.path == aPath)
            return true;
    Descriptor d;
    d.path = aPath;
    s.descriptors.push_back(d);
    s.loadsIssued = false; // the new one is requested on the next wanted tick
    return true;
}

bool Active()
{
    for (const auto& sl : S().slots)
        if (sl.state == loot::SlotState::Morphed)
            return true;
    return false;
}

uint32_t Reasons(double, bool aTpp)
{
    return loot::Reasons(MakeInputs(aTpp));
}

std::string Status(double aNow, bool aTpp)
{
    const State& s = S();
    uint32_t morphed = 0, swapped = 0, vanilla = 0, foreign = 0;
    for (const auto& sl : s.slots)
    {
        switch (sl.state)
        {
        case loot::SlotState::Morphed: ++morphed; break;
        case loot::SlotState::VtableSwapped: ++swapped; break;
        case loot::SlotState::Vanilla: ++vanilla; break;
        case loot::SlotState::Foreign: ++foreign; break;
        }
    }
    std::string desc;
    for (const auto& d : s.descriptors)
    {
        const size_t slash = d.path.find_last_of('\\');
        char buf[200];
        std::snprintf(buf, sizeof(buf), " [%s %s L%u x%u%s%s]", slash == std::string::npos ? d.path.c_str() : d.path.c_str() + slash + 1,
                      d.failed ? "FAILED" : (d.walked ? "walked" : (d.requested ? "loading" : "idle")), d.lootLayers,
                      d.leaves, d.note.empty() ? "" : " ", d.note.c_str());
        desc += buf;
    }
    char buf[700];
    std::snprintf(buf, sizeof(buf),
                  "UTPP LootRange resolved=%d (%s) enabled=%d killed=%d killFile=%d reasons=0x%X tpp=%d slots=%zu "
                  "morphed=%u swapped=%u vanilla=%u foreign=%u applies=%llu reverts=%llu window=%.2fx%.2f c%.2f p%.2f "
                  "ticks=%llu t=%.1f descriptors:",
                  s.resolved ? 1 : 0, s.resolveWhy.c_str(), s.enabled ? 1 : 0, s.killed ? 1 : 0,
                  s.killFile ? 1 : 0, loot::Reasons(MakeInputs(aTpp)), aTpp ? 1 : 0,
                  s.slots.size(), morphed, swapped, vanilla, foreign, static_cast<unsigned long long>(s.applies),
                  static_cast<unsigned long long>(s.reverts), static_cast<double>(s.window.width),
                  static_cast<double>(s.window.height), static_cast<double>(s.window.curvature),
                  static_cast<double>(s.window.maxPriorityBoundsFactor), static_cast<unsigned long long>(s.ticks), aNow);
    return std::string(buf) + desc;
}
} // namespace utpp::lootrange

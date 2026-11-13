// UTPP LootRange core (Phase 5N step 5a): pure logic, no game / Windows dependencies (offline-tested in
// tests/LootHeadTest.cpp). Designed from the game's own interaction descriptors
// (WolvenKit export of 2.31 *.interaction) and RED4ext.SDK RTTI layouts.
//
// Mechanism:
//   The "Loot" layer of the container / NPC body descriptors gates the prompt with
//       ContainedInShapes(player, sphere 2 m)  AND  LookAt(Entity, Interaction|Targeting)
//   LookAt is evaluated along the first-person view, which misses the target under an orbit camera. CDPR's own loot
//   drop descriptors gate the same layer with DistanceFromScreenCenter(width 0.65, height 0.65, curvature 1,
//   maxPriorityBoundsFactor 0.4) [generic_loot_drop: AND VisibleTarget; generic_big_loot_drop: OR LookAt], i.e. a
//   screen-space window, which follows whatever camera renders the frame.
//   UTPP morphs ONLY the LookAt leaves of the Loot layers of those descriptors into the engine's own
//   DistanceFromScreenCenterPredicate (same RTTI size 0x40, payload 0x30..0x40) while third person is active, and
//   restores the exact original bytes otherwise. The 2 m sphere around the player stays untouched (no range increase).
//
// Write ordering (all writes are aligned 8-byte stores, so a concurrent reader sees old or new, never half):
//   apply  : (1) vtable -> DistanceFromScreenCenter; LookAt payload ints read as floats are denormals (empty window)
//            (2) >= minGap later (all in-flight LookAt calls have returned): payload <- floats.
//   revert : payload <- saved ints (DistanceFromScreenCenter reads denormals = empty window), then vtable -> LookAt.
//            Immediate (no in-flight reader can see a wrong combination for its own class).
// Any unexpected vtable seen before a write marks the slot foreign: it is never written again.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace utpp::loot
{
// ---------------------------------------------------------------------------------------------
// Third-person gate shared by LootRange and HeadGuard. Fed by a heartbeat from the player state machine
// (scripts/utpp_loothead.reds). Stale heartbeat -> not TPP (fail safe: vanilla behaviour).
struct GateParams
{
    double staleSec = 0.5;      // heartbeat older than this -> FPP/unknown
    float tppCameraDist = 0.6f; // active camera this far (m, 3D) from the first-person camera -> third person
};

inline bool ValidGateParams(const GateParams& aP)
{
    return std::isfinite(aP.staleSec) && aP.staleSec >= 0.05 && aP.staleSec <= 5.0 && std::isfinite(aP.tppCameraDist) &&
           aP.tppCameraDist >= 0.1f && aP.tppCameraDist <= 5.0f;
}

class TppGate
{
public:
    void Heartbeat(double aNow, bool aOwnerSaysTpp, float aCameraDistance)
    {
        m_last = aNow;
        m_owner = aOwnerSaysTpp;
        m_dist = aCameraDistance;
        ++m_beats;
    }
    bool Fresh(double aNow) const
    {
        return m_beats > 0 && aNow >= m_last && aNow - m_last <= m_params.staleSec;
    }
    bool Tpp(double aNow) const
    {
        if (!Fresh(aNow))
            return false;
        if (m_owner)
            return true;
        return std::isfinite(m_dist) && m_dist >= m_params.tppCameraDist;
    }
    bool SetParams(const GateParams& aP)
    {
        if (!ValidGateParams(aP))
            return false;
        m_params = aP;
        return true;
    }
    const GateParams& Params() const { return m_params; }
    uint64_t Beats() const { return m_beats; }
    float Distance() const { return m_dist; }
    bool Owner() const { return m_owner; }

private:
    GateParams m_params;
    double m_last = 0.0;
    bool m_owner = false;
    float m_dist = 0.0f;
    uint64_t m_beats = 0;
};

// ---------------------------------------------------------------------------------------------
// Window parameters written into the morphed predicate (vanilla loot drop values by default).
struct WindowParams
{
    float width = 0.65f;
    float height = 0.65f;
    float curvature = 1.0f;
    float maxPriorityBoundsFactor = 0.4f;
};

inline bool ValidWindow(const WindowParams& aW)
{
    auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    return in(aW.width, 0.05f, 2.0f) && in(aW.height, 0.05f, 2.0f) && in(aW.curvature, 0.1f, 4.0f) &&
           in(aW.maxPriorityBoundsFactor, 0.0f, 1.0f);
}

// User scale 0.25..2.5 applied to width/height of the vanilla loot drop window.
inline bool ScaledWindow(float aScale, WindowParams& aOut)
{
    if (!std::isfinite(aScale) || aScale < 0.25f || aScale > 2.5f)
        return false;
    WindowParams w;
    w.width = std::fmin(2.0f, 0.65f * aScale);
    w.height = std::fmin(2.0f, 0.65f * aScale);
    aOut = w;
    return ValidWindow(aOut);
}

inline uint32_t FloatBits(float aV)
{
    uint32_t u;
    std::memcpy(&u, &aV, sizeof(u));
    return u;
}

inline uint64_t PackPair(float aLo, float aHi)
{
    return static_cast<uint64_t>(FloatBits(aLo)) | (static_cast<uint64_t>(FloatBits(aHi)) << 32);
}

// Payload layout of DistanceFromScreenCenterPredicate (verified against RTTI at runtime):
// 0x30 height, 0x34 width, 0x38 curvature, 0x3C maxPriorityBoundsFactor.
inline uint64_t WindowQ30(const WindowParams& aW) { return PackPair(aW.height, aW.width); }
inline uint64_t WindowQ38(const WindowParams& aW) { return PackPair(aW.curvature, aW.maxPriorityBoundsFactor); }

// ---------------------------------------------------------------------------------------------
// Memory access abstraction for one predicate object (game: SEH-guarded interlocked access; tests: plain buffer).
// Offsets: 0x00 vtable, 0x30 / 0x38 payload qwords.
struct IObjectMemory
{
    virtual ~IObjectMemory() = default;
    virtual bool Read(void* aObj, uint32_t aOff, uint64_t& aOut) = 0;
    virtual bool Write(void* aObj, uint32_t aOff, uint64_t aValue) = 0;
};

enum class SlotState : uint8_t
{
    Vanilla = 0,
    VtableSwapped = 1, // DistanceFromScreenCenter vtable, original payload (empty window)
    Morphed = 2,
    Foreign = 3, // unexpected vtable / failed access: never touched again
};

inline const char* SlotStateName(SlotState aS)
{
    switch (aS)
    {
    case SlotState::Vanilla: return "vanilla";
    case SlotState::VtableSwapped: return "swapped";
    case SlotState::Morphed: return "morphed";
    case SlotState::Foreign: return "foreign";
    }
    return "?";
}

struct Slot
{
    void* obj = nullptr;
    uint64_t savedQ30 = 0;
    uint64_t savedQ38 = 0;
    SlotState state = SlotState::Vanilla;
    double since = 0.0;
    uint64_t appliedQ30 = 0;
    uint64_t appliedQ38 = 0;
};

struct VtablePair
{
    uint64_t lookAt = 0;
    uint64_t window = 0; // DistanceFromScreenCenterPredicate
};

// Captures the original payload. False when the object is not a LookAt predicate (by vtable) or unreadable.
inline bool CaptureSlot(IObjectMemory& aMem, void* aObj, const VtablePair& aVt, Slot& aOut)
{
    uint64_t vt = 0, q30 = 0, q38 = 0;
    if (!aObj || (reinterpret_cast<uintptr_t>(aObj) & 7) != 0)
        return false;
    if (!aMem.Read(aObj, 0x00, vt) || vt != aVt.lookAt || !aMem.Read(aObj, 0x30, q30) || !aMem.Read(aObj, 0x38, q38))
        return false;
    aOut = Slot{};
    aOut.obj = aObj;
    aOut.savedQ30 = q30;
    aOut.savedQ38 = q38;
    return true;
}

namespace detail
{
inline bool ExpectVt(IObjectMemory& aMem, Slot& aS, uint64_t aExpected)
{
    uint64_t vt = 0;
    if (!aMem.Read(aS.obj, 0x00, vt) || vt != aExpected)
    {
        aS.state = SlotState::Foreign;
        return false;
    }
    return true;
}
inline bool W(IObjectMemory& aMem, Slot& aS, uint32_t aOff, uint64_t aV)
{
    if (!aMem.Write(aS.obj, aOff, aV))
    {
        aS.state = SlotState::Foreign;
        return false;
    }
    return true;
}
} // namespace detail

// Immediate restore (ints first, then vtable). Safe at any time. Returns true when the slot is vanilla afterwards.
// Generic form: aTarget = vtable of the engine predicate class the LookAt object was morphed into (the screen window
// for loot; InteractRange also uses gameinteractionsAlwaysSamePredicate, see InteractRangeCore.hpp).
inline bool RevertSlotTo(IObjectMemory& aMem, Slot& aS, uint64_t aLookAt, uint64_t aTarget)
{
    switch (aS.state)
    {
    case SlotState::Vanilla:
        return true;
    case SlotState::Foreign:
        return false;
    case SlotState::Morphed:
        if (!detail::ExpectVt(aMem, aS, aTarget) || !detail::W(aMem, aS, 0x38, aS.savedQ38) ||
            !detail::W(aMem, aS, 0x30, aS.savedQ30) || !detail::W(aMem, aS, 0x00, aLookAt))
            return false;
        aS.state = SlotState::Vanilla;
        return true;
    case SlotState::VtableSwapped:
        if (!detail::ExpectVt(aMem, aS, aTarget) || !detail::W(aMem, aS, 0x00, aLookAt))
            return false;
        aS.state = SlotState::Vanilla;
        return true;
    }
    return false;
}

inline bool RevertSlot(IObjectMemory& aMem, Slot& aS, const VtablePair& aVt)
{
    return RevertSlotTo(aMem, aS, aVt.lookAt, aVt.window);
}

// One step towards the wanted state. Apply is two-phase (minGap seconds between vtable and payload).
// Generic form: morph into class aTarget with payload qwords aQ30 / aQ38 (offsets 0x30 / 0x38 of the 0x40-byte object).
inline void StepSlotTo(IObjectMemory& aMem, Slot& aS, uint64_t aLookAt, uint64_t aTarget, bool aWant, uint64_t aQ30,
                       uint64_t aQ38, double aNow, double aMinGap);

inline void StepSlot(IObjectMemory& aMem, Slot& aS, const VtablePair& aVt, bool aWant, const WindowParams& aW,
                     double aNow, double aMinGap)
{
    StepSlotTo(aMem, aS, aVt.lookAt, aVt.window, aWant, WindowQ30(aW), WindowQ38(aW), aNow, aMinGap);
}

inline void StepSlotTo(IObjectMemory& aMem, Slot& aS, uint64_t aLookAt, uint64_t aTarget, bool aWant, uint64_t aQ30,
                       uint64_t aQ38, double aNow, double aMinGap)
{
    if (aS.state == SlotState::Foreign)
        return;
    if (!aWant)
    {
        RevertSlotTo(aMem, aS, aLookAt, aTarget);
        return;
    }
    const uint64_t q30 = aQ30, q38 = aQ38;
    switch (aS.state)
    {
    case SlotState::Vanilla:
        if (!detail::ExpectVt(aMem, aS, aLookAt))
            return;
        {
            // The payload must still be the captured original (no other writer).
            uint64_t c30 = 0, c38 = 0;
            if (!aMem.Read(aS.obj, 0x30, c30) || !aMem.Read(aS.obj, 0x38, c38) || c30 != aS.savedQ30 ||
                c38 != aS.savedQ38)
            {
                aS.state = SlotState::Foreign;
                return;
            }
        }
        if (!detail::W(aMem, aS, 0x00, aTarget))
            return;
        aS.state = SlotState::VtableSwapped;
        aS.since = aNow;
        return;
    case SlotState::VtableSwapped:
        if (aNow - aS.since < aMinGap)
            return;
        if (!detail::ExpectVt(aMem, aS, aTarget) || !detail::W(aMem, aS, 0x30, q30) || !detail::W(aMem, aS, 0x38, q38))
            return;
        aS.state = SlotState::Morphed;
        aS.appliedQ30 = q30;
        aS.appliedQ38 = q38;
        return;
    case SlotState::Morphed:
        if (aS.appliedQ30 == q30 && aS.appliedQ38 == q38)
            return;
        if (!detail::ExpectVt(aMem, aS, aTarget) || !detail::W(aMem, aS, 0x30, q30) || !detail::W(aMem, aS, 0x38, q38))
            return;
        aS.appliedQ30 = q30;
        aS.appliedQ38 = q38;
        return;
    case SlotState::Foreign:
        return;
    }
}

// ---------------------------------------------------------------------------------------------
// Descriptor walk: InteractionDescriptorResource -> definition.layersDefinition[] (CLinkedLayersDefinition)
// -> layersDefinitions[] (CHotSpotLayerDefinition, tag == Loot) -> areaFilterDefinition -> functor
// -> predicate {predicateType, functor1, functor2} (recursive). Offsets verified against RTTI at runtime.
struct DescriptorOffsets
{
    uint32_t resDefinition = 0x40;      // InteractionDescriptorResource.definition (inline CHotSpotDefinition)
    uint32_t defLayers = 0x00;          // CHotSpotDefinition.layersDefinition (DynArray<Handle>)
    uint32_t linkedLayers = 0x50;       // CLinkedLayersDefinition.layersDefinitions (DynArray<Handle>)
    uint32_t layerTag = 0x48;           // CHotSpotLayerDefinition.tag (CName)
    uint32_t layerAreaFilter = 0x60;    // CHotSpotLayerDefinition.areaFilterDefinition (Handle)
    uint32_t areaFunctor = 0x90;        // CHotSpotAreaFilterDefinition.functor (Handle)
    uint32_t functorPredicate = 0x30;   // CFunctorDefinition.predicate (inline CPredicateDefinition)
    uint32_t predType = 0x00;           // CPredicateDefinition.predicateType (Handle)
    uint32_t predFunctor1 = 0x18;       // CPredicateDefinition.functor1DataDefinition (Handle)
    uint32_t predFunctor2 = 0x28;       // CPredicateDefinition.functor2DataDefinition (Handle)
};

struct IDescriptorReader
{
    virtual ~IDescriptorReader() = default;
    virtual bool Ptr(const void* aBase, uint32_t aOff, void*& aOut) = 0;     // reads a pointer (Handle.instance etc.)
    virtual bool U32(const void* aBase, uint32_t aOff, uint32_t& aOut) = 0;
    virtual bool U64(const void* aBase, uint32_t aOff, uint64_t& aOut) = 0;
    virtual bool IsLookAt(void* aObj) = 0; // class identity (game: vtable of a fresh LookAtPredicate + RTTI type)
};

struct WalkResult
{
    std::vector<void*> leaves; // unique LookAt predicate objects inside Loot layers
    uint32_t lootLayers = 0;
    uint32_t nodes = 0;
    bool truncated = false;
    bool readError = false;
};

namespace detail
{
constexpr uint32_t kMaxArray = 64;
constexpr uint32_t kMaxDepth = 12;
constexpr uint32_t kMaxNodes = 512;

inline bool Array(IDescriptorReader& aR, const void* aBase, uint32_t aOff, void*& aEntries, uint32_t& aSize,
                  WalkResult& aRes)
{
    if (!aR.Ptr(aBase, aOff, aEntries) || !aR.U32(aBase, aOff + 0x0C, aSize))
    {
        aRes.readError = true;
        return false;
    }
    if (aSize > kMaxArray)
    {
        aRes.truncated = true;
        aSize = kMaxArray;
    }
    return aSize == 0 || aEntries != nullptr;
}

inline void Functor(IDescriptorReader& aR, const DescriptorOffsets& aO, void* aFunctor, uint32_t aDepth, WalkResult& aRes)
{
    if (!aFunctor)
        return;
    if (aDepth > kMaxDepth || ++aRes.nodes > kMaxNodes)
    {
        aRes.truncated = true;
        return;
    }
    const auto* pred = static_cast<const uint8_t*>(aFunctor) + aO.functorPredicate;
    void* type = nullptr;
    void* f1 = nullptr;
    void* f2 = nullptr;
    if (!aR.Ptr(pred, aO.predType, type) || !aR.Ptr(pred, aO.predFunctor1, f1) || !aR.Ptr(pred, aO.predFunctor2, f2))
    {
        aRes.readError = true;
        return;
    }
    if (type && aR.IsLookAt(type))
    {
        bool seen = false;
        for (void* l : aRes.leaves)
            seen = seen || l == type;
        if (!seen)
            aRes.leaves.push_back(type);
    }
    Functor(aR, aO, f1, aDepth + 1, aRes);
    Functor(aR, aO, f2, aDepth + 1, aRes);
}
} // namespace detail

// Generic form: collects the LookAt leaves of every layer whose tag is one of aTags[0..aTagCount) (WalkResult.lootLayers
// then counts all matched layers). WalkDescriptor below = the Loot layer only.
inline WalkResult WalkDescriptorTags(IDescriptorReader& aR, const DescriptorOffsets& aO, const void* aResource,
                                     const uint64_t* aTags, size_t aTagCount)
{
    WalkResult res;
    if (!aResource)
    {
        res.readError = true;
        return res;
    }
    const auto* def = static_cast<const uint8_t*>(aResource) + aO.resDefinition;
    void* linkedEntries = nullptr;
    uint32_t linkedCount = 0;
    if (!detail::Array(aR, def, aO.defLayers, linkedEntries, linkedCount, res))
        return res;
    for (uint32_t i = 0; i < linkedCount; ++i)
    {
        void* linked = nullptr;
        if (!aR.Ptr(linkedEntries, i * 16u, linked)) // Handle = {instance, refCount}
        {
            res.readError = true;
            return res;
        }
        if (!linked)
            continue;
        void* layerEntries = nullptr;
        uint32_t layerCount = 0;
        if (!detail::Array(aR, linked, aO.linkedLayers, layerEntries, layerCount, res))
            return res;
        for (uint32_t j = 0; j < layerCount; ++j)
        {
            void* layer = nullptr;
            uint64_t tag = 0;
            if (!aR.Ptr(layerEntries, j * 16u, layer))
            {
                res.readError = true;
                return res;
            }
            if (!layer)
                continue;
            if (!aR.U64(layer, aO.layerTag, tag))
            {
                res.readError = true;
                return res;
            }
            bool match = false;
            for (size_t k = 0; k < aTagCount && !match; ++k)
                match = aTags[k] == tag;
            if (!match)
                continue;
            ++res.lootLayers;
            void* area = nullptr;
            void* functor = nullptr;
            if (!aR.Ptr(layer, aO.layerAreaFilter, area))
            {
                res.readError = true;
                return res;
            }
            if (!area)
                continue;
            if (!aR.Ptr(area, aO.areaFunctor, functor))
            {
                res.readError = true;
                return res;
            }
            detail::Functor(aR, aO, functor, 0, res);
        }
    }
    return res;
}

inline WalkResult WalkDescriptor(IDescriptorReader& aR, const DescriptorOffsets& aO, const void* aResource,
                                 uint64_t aLootTagHash)
{
    return WalkDescriptorTags(aR, aO, aResource, &aLootTagHash, 1);
}

// ---------------------------------------------------------------------------------------------
// Activation decision (reasons bitmask, 0 = active).
enum Reason : uint32_t
{
    kNotResolved = 1u << 0,  // classes / offsets / vtables not proven
    kDisabled = 1u << 1,     // user setting off (default)
    kKilled = 1u << 2,       // session kill switch
    // bit 3 unused
    kNotTpp = 1u << 4,       // heartbeat stale or first person
    kKillFile = 1u << 5,     // utpp_lootrange.disable
    kNotValidated = 1u << 6, // UTPP's 2.31 validation failed
    kNoSlots = 1u << 7,      // no LookAt leaf captured (descriptors not loaded yet)
};

struct Inputs
{
    bool resolved = false;
    bool enabled = false;
    bool killed = false;
    bool tpp = false;
    bool killFile = false;
    bool validated = false;
    bool haveSlots = false;
};

inline uint32_t Reasons(const Inputs& aIn)
{
    uint32_t r = 0;
    if (!aIn.resolved)
        r |= kNotResolved;
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
    if (!aIn.haveSlots)
        r |= kNoSlots;
    return r;
}

// Morph wanted: everything except "no slots" must be clear.
inline bool WantMorph(const Inputs& aIn)
{
    return (Reasons(aIn) & ~static_cast<uint32_t>(kNoSlots)) == 0;
}
} // namespace utpp::loot

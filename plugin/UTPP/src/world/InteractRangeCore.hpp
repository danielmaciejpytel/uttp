// UTPP InteractRange core: pure logic, no game / Windows dependencies (offline-tested in tests/LootHeadTest.cpp).
// Third-person interaction reach for vehicles, devices / doors and NPC talk, built on the LootRange morph
// (LootRangeCore.hpp: a LookAt predicate object of a loaded *.interaction descriptor is turned into another engine
// predicate class of the same 0x40-byte storage while third person is active, original bytes restored otherwise).
//
// Evidence (2.31 WolvenKit export of the interaction layers):
//   vehicles  vehicle_device_interaction seat_front_left / seat_front_right / seat_back_left / seat_back_right,
//             motorcycle_device_interaction seat_front_left, bike / driver / passenger* _interaction Mount:
//                 ContainedInShapes(player, seat sphere r 1.0 .. 1.3 m) AND LookAt(Entity, Interaction)
//   devices   default_device / ladder_device / fake_door direct: ContainedInShapes(camera, r 2.85) AND LookAt AND
//             notSuppressed; door_interaction_descriptor / elevator_device layer1: ContainedInShapes AND LookAt(Targeting)
//   NPC talk  default_puppet ReturnTalk, mechanical_puppet GenericTalk / ReturnTalk: ContainedInShapes AND LookAt
//   (default_puppet GenericTalk already has a vanilla screen-window branch; loot drops / small and big loot already use
//    DistanceFromScreenCenter; ammo / throwables are proximity only -> not managed here.)
// LookAt follows the player's look ray. Under an orbit camera that ray rarely meets a vehicle next to V (head 1.5 m
// above the pivot, camera pitch near 0, the bike seat 45 deg below the eye line), so the seat prompt needs a
// "perfect" camera angle.
//
// Morph targets:
//   Vehicle  (default) gameinteractionsAlwaysSamePredicate (RTTI size 0x38, float priority at 0x30; its evaluate
//            virtual returns this->priority: 2.5 code at vtable slot 28 = mov eax,[rcx+30h]; mov [rdx],eax;
//            mov rax,rdx; ret - checked again at runtime before use). The seat layer is then gated only by its vanilla
//            seat sphere around V (no range increase). Fallback / option: the screen window below.
//   Device, Talk  gameinteractionsDistanceFromScreenCenterPredicate with the vanilla loot drop window (0.65 x 0.65,
//            curvature 1, maxPriorityBoundsFactor 0.4) times a user scale - the same window CDPR uses for dropped items.
// Payload while only the vtable is swapped (phase 1, <= minGap): the LookAt ints read as floats are 0 or denormals,
// i.e. an empty window / priority ~0, same as LootRange.
//
// Load-time look-at classification (static analysis of the 2.5 Cyberpunk2077.exe; RVAs relative to the image base):
//   InteractionDescriptorResource post-load (vtable 0x308d438 slot 5 = 0x8b1ab4) builds a refcounted "compiled" object
//   (0xD0 bytes, 0x8b19f0) held at resource+0xF0, then runs a visitor (vtable 0x308d3b8) over its groups:
//     compiled+0x00  DynArray<Handle<CLinkedLayersDefinition>> = copy of definition.layersDefinition (0x8b15a0
//                    copies the handles with 0x1a9580: same instances, strong count +1)
//     compiled+0xB0  entry[0] = {qword key, DynArray<Handle<CLinkedLayersDefinition>> list @+0xB8}; entry count
//                    at +0xC8 (always 1, 0x8b19f0). DynArray = {entries @0, capacity u32 @8, size u32 @0xC}; capacity 0 ->
//                    'entries' holds the allocator (its vtable pointer), otherwise the allocator qword follows the
//                    elements at entries + align8(capacity * 16) (realloc 0x270174; same as RED4ext.SDK DynArray).
//   Per group the visitor keeps a stack of {x = subtree contains a LookAtPredicate (GetType IsA, 0x8b00dc),
//   y = subtree passes without the LookAt}; leaf y = !isLookAt, the CPredicateDefinition binary operator combines with
//   AND / OR, unary operator 1 negates (pop / combine 0xf5fe04). Root of a layer's area-filter tree: flag = !x || y;
//   the flag is reset per layer (0x8b0274), so the group's last layer decides. 0x8b02c0 appends the group handle to the
//   visitor list when the flag is set; 0x8af0c8 stores a non-empty list into entry[0] with key = the compiled object.
//   => list = groups that are NOT look-at gated. Vehicle seat / Mount descriptors have none (key 0, empty list);
//   motorcycle_device_interaction lists PuppetClose / PuppetFar only.
//   The hotspot registration (0x8adfcc) hands a SharedPtr to this shared compiled object to the interaction manager;
//   the reader of the list was not located statically - the in-game check proves whether membership changes the prompt.
//   UTPP's predicate morph happens long after this one-time classification, so a morphed seat group stays "gated".
// List patch (vehicle groups only, in step with the morph, see StepList): a managed seat group is appended while all
// of its LookAt leaves are morphed and removed before they are restored. The list never shrinks below the vanilla
// members and is never freed under a reader (see StepList for the write ordering). Only groups whose LookAt leaves in
// all layers are captured qualify (ListIneligible). A group that cannot be taken out again when the patch fails keeps
// its predicates morphed for the session (ListPatch::pinned) instead of being listed with a restored LookAt.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "LootRangeCore.hpp"

namespace utpp::interact
{
enum class Group : uint8_t
{
    Vehicle = 0,
    Device = 1,
    Talk = 2,
};

inline const char* GroupName(Group aG)
{
    switch (aG)
    {
    case Group::Vehicle: return "vehicle";
    case Group::Device: return "device";
    case Group::Talk: return "talk";
    }
    return "?";
}

// Vehicle seat handling in third person.
enum class VehicleMode : int32_t
{
    Vanilla = 0,         // seat layers untouched
    Proximity = 1,       // LookAt -> AlwaysSame(priority) + seat group in the non-look-at list: seat sphere decides
                         // (default)
    Window = 2,          // LookAt -> screen window (same as devices) + seat group in the non-look-at list
    ProximityNoList = 3, // LookAt -> AlwaysSame only, classification list untouched (A/B check)
};

inline bool ValidVehicleMode(int32_t aM) { return aM >= 0 && aM <= 3; }

// Whether the managed seat groups are added to the compiled non-look-at list in this mode.
inline bool ListPatchMode(VehicleMode aM) { return aM == VehicleMode::Proximity || aM == VehicleMode::Window; }

constexpr size_t kMaxTags = 4;

struct DescriptorSpec
{
    const char* path;
    Group group;
    const char* tags[kMaxTags]; // nullptr-terminated when shorter
};

// Managed descriptors (depot paths as in the 2.31 / 2.5 archives) and the layers whose LookAt leaves are morphed.
// Deliberately NOT managed: Loot (LootRange), Grapple / takedown layers (combat), vehicle trunk / hood (quest-gated
// actions), default_puppet GenericTalk (vanilla already screen based).
inline const DescriptorSpec* Descriptors(size_t& aCount)
{
    static const DescriptorSpec kSpecs[] = {
        {"base\\gameplay\\vehicles\\vehicle_device_interaction.interaction", Group::Vehicle,
         {"seat_front_left", "seat_front_right", "seat_back_left", "seat_back_right"}},
        {"base\\gameplay\\vehicles\\motorcycle_device_interaction.interaction", Group::Vehicle, {"seat_front_left"}},
        {"base\\gameplay\\vehicles\\bike_interaction.interaction", Group::Vehicle, {"Mount"}},
        {"base\\gameplay\\vehicles\\driver_interaction.interaction", Group::Vehicle, {"Mount"}},
        {"base\\gameplay\\vehicles\\passenger_interaction.interaction", Group::Vehicle, {"Mount"}},
        {"base\\gameplay\\vehicles\\passenger_back_left_interaction.interaction", Group::Vehicle, {"Mount"}},
        {"base\\gameplay\\vehicles\\passenger_back_right_interaction.interaction", Group::Vehicle, {"Mount"}},
        {"base\\gameplay\\devices\\interactions\\default_device_interaction.interaction", Group::Device, {"direct"}},
        {"base\\gameplay\\devices\\interactions\\ladder_device_interaction.interaction", Group::Device, {"direct"}},
        {"base\\gameplay\\devices\\doors\\base\\fake_door_interaction.interaction", Group::Device, {"direct"}},
        {"base\\gameplay\\devices\\doors\\door_interaction_descriptor.interaction", Group::Device, {"layer1"}},
        {"base\\gameplay\\devices\\elevators\\elevator_device_interaction.interaction", Group::Device, {"layer1"}},
        {"base\\gameplay\\interactions\\npc\\default_puppet_interaction.interaction", Group::Talk, {"ReturnTalk"}},
        {"base\\gameplay\\interactions\\npc\\mechanical_puppet_interaction.interaction", Group::Talk,
         {"GenericTalk", "ReturnTalk"}},
    };
    aCount = sizeof(kSpecs) / sizeof(kSpecs[0]);
    return kSpecs;
}

inline size_t TagCount(const DescriptorSpec& aS)
{
    size_t n = 0;
    while (n < kMaxTags && aS.tags[n])
        ++n;
    return n;
}

// Descriptors handled by LootRange (Loot layer); listed here only so the console probe can name them.
inline const char* const* LootDescriptorPaths(size_t& aCount)
{
    static const char* const kPaths[] = {
        "base\\gameplay\\containers\\generic_container_interaction.interaction",
        "base\\gameplay\\containers\\generic_singular_container_interaction.interaction",
        "base\\gameplay\\interactions\\npc\\default_puppet_interaction.interaction",
        "base\\gameplay\\interactions\\npc\\mechanical_puppet_interaction.interaction",
    };
    aCount = sizeof(kPaths) / sizeof(kPaths[0]);
    return kPaths;
}

// ---------------------------------------------------------------------------------------------
// AlwaysSame predicate. Default = 1.0, the value of the vanilla AlwaysSame leaves (mechanical_puppet TakedownLayer).
constexpr float kDefaultVehiclePriority = 1.0f;

inline bool ValidPriority(float aP) { return std::isfinite(aP) && aP >= 0.05f && aP <= 1.0f; }

// Payload qword 0x30: low dword = float priority, high dword (AlwaysSame padding 0x34) = original LookAt testType.
inline uint64_t PriorityQ30(float aPriority, uint64_t aSavedQ30)
{
    return static_cast<uint64_t>(loot::FloatBits(aPriority)) | (aSavedQ30 & 0xFFFFFFFF00000000ull);
}

// Evaluate virtual of AlwaysSamePredicate on 2.5 (vtable slot kEvalSlot): returns the float at this+0x30.
constexpr uint32_t kEvalSlot = 28;
constexpr uint8_t kAlwaysSameEvalCode[9] = {0x8B, 0x41, 0x30, 0x89, 0x02, 0x48, 0x8B, 0xC2, 0xC3};

inline bool IsAlwaysSameEval(const uint8_t* aCode, size_t aLen)
{
    return aCode && aLen >= sizeof(kAlwaysSameEvalCode) &&
           std::memcmp(aCode, kAlwaysSameEvalCode, sizeof(kAlwaysSameEvalCode)) == 0;
}

// ---------------------------------------------------------------------------------------------
// Per-slot target selection and stepping.
struct Vtables
{
    uint64_t lookAt = 0;
    uint64_t window = 0;     // DistanceFromScreenCenterPredicate
    uint64_t alwaysSame = 0; // 0 = not proven -> vehicles use the window
};

struct Config
{
    loot::WindowParams window; // devices, talk, vehicles in Window mode
    VehicleMode vehicleMode = VehicleMode::Proximity;
    float vehiclePriority = kDefaultVehiclePriority;
};

struct Slot
{
    loot::Slot s;
    Group group = Group::Device;
    uint64_t morphVt = 0;     // class the object is (partly) morphed into; valid while s.state is swapped / morphed
    void* groupObj = nullptr; // CLinkedLayersDefinition holding the layer of this leaf
};

struct Target
{
    bool want = false;
    uint64_t vt = 0;
    uint64_t q30 = 0;
    uint64_t q38 = 0;
};

inline Target TargetFor(const Slot& aSlot, bool aWantAll, const Vtables& aVt, const Config& aCfg)
{
    Target t;
    if (!aWantAll)
        return t;
    if (aSlot.group == Group::Vehicle)
    {
        if (aCfg.vehicleMode == VehicleMode::Vanilla)
            return t;
        const bool prox =
            aCfg.vehicleMode == VehicleMode::Proximity || aCfg.vehicleMode == VehicleMode::ProximityNoList;
        if (prox && aVt.alwaysSame != 0 && ValidPriority(aCfg.vehiclePriority))
        {
            t.want = true;
            t.vt = aVt.alwaysSame;
            t.q30 = PriorityQ30(aCfg.vehiclePriority, aSlot.s.savedQ30);
            t.q38 = aSlot.s.savedQ38; // beyond the 0x38 bytes of AlwaysSame: left as captured
            return t;
        }
    }
    t.want = true;
    t.vt = aVt.window;
    t.q30 = loot::WindowQ30(aCfg.window);
    t.q38 = loot::WindowQ38(aCfg.window);
    return t;
}

// One step. A slot morphed into another class than the wanted one (mode change) is first restored to vanilla; the
// new target is applied from the next step on (two-phase again).
inline void StepInteractSlot(loot::IObjectMemory& aMem, Slot& aSlot, bool aWantAll, const Vtables& aVt,
                             const Config& aCfg, double aNow, double aMinGap)
{
    loot::Slot& s = aSlot.s;
    if (s.state == loot::SlotState::Foreign)
        return;
    const Target t = TargetFor(aSlot, aWantAll, aVt, aCfg);
    const bool morphed = s.state == loot::SlotState::VtableSwapped || s.state == loot::SlotState::Morphed;
    if (morphed && (!t.want || aSlot.morphVt != t.vt))
    {
        loot::RevertSlotTo(aMem, s, aVt.lookAt, aSlot.morphVt);
        return;
    }
    if (!t.want)
        return;
    loot::StepSlotTo(aMem, s, aVt.lookAt, t.vt, true, t.q30, t.q38, aNow, aMinGap);
    if (s.state == loot::SlotState::VtableSwapped || s.state == loot::SlotState::Morphed)
        aSlot.morphVt = t.vt;
}

inline bool RevertInteractSlot(loot::IObjectMemory& aMem, Slot& aSlot, const Vtables& aVt)
{
    if (aSlot.s.state == loot::SlotState::Vanilla)
        return true;
    return loot::RevertSlotTo(aMem, aSlot.s, aVt.lookAt, aSlot.morphVt);
}

// ---------------------------------------------------------------------------------------------
// Group-aware walk: same traversal and limits as loot::WalkDescriptorTags, but the LookAt leaves are reported per
// CLinkedLayersDefinition (the unit of the load-time classification). A leaf shared by two groups is kept in the first.
struct GroupLeaves
{
    void* group = nullptr;
    std::vector<void*> leaves;    // LookAt leaves of the matching layers (shared ones only in the first group)
    bool otherLookAt = false;     // a non-matching layer of this group holds a LookAt leaf (or could not be read)
    bool shared = false;          // a LookAt leaf of a matching layer was already reported for an earlier group
    bool truncated = false;       // a walk limit was hit while this group was walked (leaves may be missing)
};

// The load-time classification decides per group (all layers). A group may join the non-look-at list only when every
// LookAt leaf in all of its layers is one UTPP captured (and therefore morphs) as part of this group.
// Returns nullptr when eligible, otherwise the reason.
inline const char* ListIneligible(const GroupLeaves& aG, size_t aCaptured)
{
    if (aG.leaves.empty())
        return "no LookAt leaf of its own";
    if (aG.otherLookAt)
        return "LookAt in a layer that is not morphed";
    if (aG.shared)
        return "LookAt leaf shared with another group";
    if (aG.truncated)
        return "walk truncated";
    if (aCaptured != aG.leaves.size())
        return "not every LookAt leaf captured";
    return nullptr;
}

struct GroupWalk
{
    std::vector<GroupLeaves> groups; // only groups with at least one matching layer
    uint32_t layers = 0;             // matching layers
    bool truncated = false;
    bool readError = false;
    size_t LeafCount() const
    {
        size_t n = 0;
        for (const auto& g : groups)
            n += g.leaves.size();
        return n;
    }
};

inline GroupWalk WalkGroupsTags(loot::IDescriptorReader& aR, const loot::DescriptorOffsets& aO, const void* aResource,
                                const uint64_t* aTags, size_t aTagCount)
{
    GroupWalk out;
    loot::WalkResult tmp; // limits / error flags of the shared helpers
    if (!aResource)
    {
        out.readError = true;
        return out;
    }
    const auto* def = static_cast<const uint8_t*>(aResource) + aO.resDefinition;
    void* linkedEntries = nullptr;
    uint32_t linkedCount = 0;
    std::vector<void*> seen;
    auto finish = [&]() {
        out.truncated = out.truncated || tmp.truncated;
        out.readError = out.readError || tmp.readError;
        return out;
    };
    if (!loot::detail::Array(aR, def, aO.defLayers, linkedEntries, linkedCount, tmp))
        return finish();
    for (uint32_t i = 0; i < linkedCount; ++i)
    {
        void* linked = nullptr;
        if (!aR.Ptr(linkedEntries, i * 16u, linked))
        {
            out.readError = true;
            return finish();
        }
        if (!linked)
            continue;
        void* layerEntries = nullptr;
        uint32_t layerCount = 0;
        if (!loot::detail::Array(aR, linked, aO.linkedLayers, layerEntries, layerCount, tmp))
            return finish();
        GroupLeaves gl;
        gl.group = linked;
        bool matched = false;
        for (uint32_t j = 0; j < layerCount; ++j)
        {
            void* layer = nullptr;
            uint64_t tag = 0;
            if (!aR.Ptr(layerEntries, j * 16u, layer))
            {
                out.readError = true;
                return finish();
            }
            if (!layer)
                continue;
            if (!aR.U64(layer, aO.layerTag, tag))
            {
                out.readError = true;
                return finish();
            }
            bool match = false;
            for (size_t k = 0; k < aTagCount && !match; ++k)
                match = aTags[k] == tag;
            if (!match)
            {
                // Not morphed: only checked for LookAt leaves (separate limits; failures only make the group ineligible).
                if (gl.otherLookAt)
                    continue;
                void* oArea = nullptr;
                void* oFunctor = nullptr;
                if (!aR.Ptr(layer, aO.layerAreaFilter, oArea) || (oArea && !aR.Ptr(oArea, aO.areaFunctor, oFunctor)))
                {
                    gl.otherLookAt = true;
                    continue;
                }
                loot::WalkResult other;
                loot::detail::Functor(aR, aO, oFunctor, 0, other);
                gl.otherLookAt = !other.leaves.empty() || other.readError || other.truncated;
                continue;
            }
            matched = true;
            ++out.layers;
            void* area = nullptr;
            void* functor = nullptr;
            if (!aR.Ptr(layer, aO.layerAreaFilter, area))
            {
                out.readError = true;
                return finish();
            }
            if (!area)
                continue;
            if (!aR.Ptr(area, aO.areaFunctor, functor))
            {
                out.readError = true;
                return finish();
            }
            tmp.leaves.clear();
            loot::detail::Functor(aR, aO, functor, 0, tmp);
            for (void* leaf : tmp.leaves)
            {
                bool dup = false;
                for (void* s : seen)
                    dup = dup || s == leaf;
                if (dup)
                {
                    bool own = false;
                    for (void* l : gl.leaves)
                        own = own || l == leaf;
                    gl.shared = gl.shared || !own;
                    continue;
                }
                seen.push_back(leaf);
                gl.leaves.push_back(leaf);
            }
            if (tmp.readError)
                return finish();
        }
        if (matched)
        {
            gl.truncated = tmp.truncated; // sticky: once a limit is hit, later groups count as incomplete too
            out.groups.push_back(gl);
        }
    }
    return finish();
}

// ---------------------------------------------------------------------------------------------
// Compiled non-look-at list (layout and evidence in the header comment).
struct CompiledLayout
{
    uint32_t resCompiled = 0xF0; // InteractionDescriptorResource: SharedPtr<compiled> (instance pointer first)
    uint32_t groups = 0x00;      // compiled: DynArray<Handle<CLinkedLayersDefinition>> (copy of the definition)
    uint32_t key = 0xB0;         // compiled: entry[0].key (= compiled when the list was stored, else 0)
    uint32_t list = 0xB8;        // compiled: entry[0].list DynArray
    uint32_t entryCount = 0xC8;  // compiled: number of entries (1)
};

constexpr uint32_t kHandleSize = 16;     // Handle = {instance, refCount}
constexpr uint32_t kMaxListGroups = 64;  // sanity bound (descriptors have <= 8 groups)
constexpr uint32_t kMaxListCapacity = 256;
constexpr uint32_t kMaxListGrows = 8;    // per compiled object; each grow keeps the previous buffer alive

inline uint64_t Align8(uint64_t aV) { return (aV + 7) & ~7ull; }
// Buffer of a DynArray<Handle> with capacity aCap: elements, then the allocator qword (game realloc 0x270174 layout).
inline uint64_t ListBufferBytes(uint32_t aCap) { return Align8(static_cast<uint64_t>(aCap) * kHandleSize) + 8; }
inline uint64_t ListAllocatorSlot(uint64_t aData, uint32_t aCap)
{
    return aData + Align8(static_cast<uint64_t>(aCap) * kHandleSize);
}

// Raw memory access for the list (game: SEH-guarded aligned / interlocked access; tests: an arena with a write log).
struct IListMemory
{
    virtual ~IListMemory() = default;
    virtual bool R64(uint64_t aAddr, uint64_t& aOut) = 0;
    virtual bool R32(uint64_t aAddr, uint32_t& aOut) = 0;
    virtual bool W64(uint64_t aAddr, uint64_t aValue) = 0; // aligned single store
    virtual bool W32(uint64_t aAddr, uint32_t aValue) = 0; // aligned single store
    virtual bool IncStrong(uint64_t aRefCount) = 0;        // what a Handle copy does: lock inc of the strong count (+0)
    // Allocates through the allocator whose vtable pointer is aAllocator (IAllocator::AllocAligned, slot 1).
    // 0 = failed.
    virtual uint64_t Alloc(uint64_t aAllocator, uint64_t aBytes, uint32_t aAlign) = 0;
    virtual bool InImage(uint64_t aAddr) = 0; // inside the game executable image
};

struct GroupHandle
{
    uint64_t inst = 0;
    uint64_t rc = 0;
};

inline int IndexOfGroup(const std::vector<GroupHandle>& aV, uint64_t aInst)
{
    for (size_t i = 0; i < aV.size(); ++i)
        if (aV[i].inst == aInst)
            return static_cast<int>(i);
    return -1;
}

struct ListProbe
{
    bool ok = false;
    const char* why = "not probed";
    uint64_t comp = 0;
    uint64_t key = 0;
    uint64_t data = 0;
    uint32_t cap = 0;
    uint32_t size = 0;
    uint64_t allocator = 0;
    std::vector<GroupHandle> groups; // compiled+0x00 (proven equal to definition.layersDefinition)
    std::vector<GroupHandle> list;
};

// Read-only. Every layout assumption is checked against the live data: one entry; the compiled group array holds
// exactly the definition's group instances; the list is a sane DynArray whose allocator vtable lies in the game image,
// whose elements are those groups with a refcount block; key is 0 (empty list) or the compiled object itself.
inline ListProbe ProbeList(IListMemory& aM, const CompiledLayout& aL, const loot::DescriptorOffsets& aO, uint64_t aRes)
{
    ListProbe p;
    auto fail = [&](const char* aWhy) {
        p.ok = false;
        p.why = aWhy;
        return p;
    };
    if (!aRes || (aRes & 7) != 0)
        return fail("no resource");
    if (!aM.R64(aRes + aL.resCompiled, p.comp))
        return fail("compiled pointer unreadable");
    if (!p.comp || (p.comp & 7) != 0)
        return fail(p.comp ? "compiled misaligned" : "no compiled data");
    uint32_t entries = 0;
    if (!aM.R32(p.comp + aL.entryCount, entries) || entries != 1)
        return fail("compiled entry count != 1");
    if (!aM.R64(p.comp + aL.key, p.key))
        return fail("key unreadable");
    uint64_t gData = 0, dData = 0;
    uint32_t gSize = 0, dSize = 0;
    const uint64_t defArr = aRes + aO.resDefinition + aO.defLayers;
    if (!aM.R64(p.comp + aL.groups, gData) || !aM.R32(p.comp + aL.groups + 0xC, gSize) || !aM.R64(defArr, dData) ||
        !aM.R32(defArr + 0xC, dSize))
        return fail("group arrays unreadable");
    if (gSize == 0 || gSize > kMaxListGroups || gSize != dSize || !gData || !dData)
        return fail("compiled groups differ from the definition (count)");
    for (uint32_t i = 0; i < gSize; ++i)
    {
        GroupHandle h;
        uint64_t d = 0;
        if (!aM.R64(gData + i * kHandleSize, h.inst) || !aM.R64(gData + i * kHandleSize + 8, h.rc) ||
            !aM.R64(dData + i * kHandleSize, d))
            return fail("group handle unreadable");
        if (h.inst != d)
            return fail("compiled groups differ from the definition (instance)");
        p.groups.push_back(h);
    }
    if (!aM.R64(p.comp + aL.list, p.data) || !aM.R32(p.comp + aL.list + 8, p.cap) ||
        !aM.R32(p.comp + aL.list + 0xC, p.size))
        return fail("list unreadable");
    if (p.size > p.cap || p.cap > kMaxListCapacity)
        return fail("list size / capacity implausible");
    if (p.cap == 0)
        p.allocator = p.data;
    else if ((p.data & 7) != 0 || !aM.R64(ListAllocatorSlot(p.data, p.cap), p.allocator))
        return fail("list buffer unreadable");
    uint64_t allocFn = 0;
    if (!aM.InImage(p.allocator) || !aM.R64(p.allocator + 8, allocFn) || !aM.InImage(allocFn))
        return fail("list allocator not a game vtable");
    if (p.key != 0 && p.key != p.comp)
        return fail("list key is neither 0 nor the compiled object");
    if (p.key == 0 && p.size != 0)
        return fail("list key 0 with a non-empty list");
    for (uint32_t i = 0; i < p.size; ++i)
    {
        GroupHandle h;
        if (!aM.R64(p.data + i * kHandleSize, h.inst) || !aM.R64(p.data + i * kHandleSize + 8, h.rc))
            return fail("list element unreadable");
        if (IndexOfGroup(p.groups, h.inst) < 0 || !h.rc)
            return fail("list element is not a group handle of this descriptor");
        p.list.push_back(h);
    }
    p.ok = true;
    p.why = "ok";
    return p;
}

enum class ListState : uint8_t
{
    Vanilla = 0, // nothing of ours in the list
    Grown = 1,   // buffer replaced by a larger copy (same members); waiting minGap before the append
    Added = 2,   // our groups appended after the vanilla members
    Foreign = 3, // layout proof failed / unexpected content / failed write: never written again
};

inline const char* ListStateName(ListState aS)
{
    switch (aS)
    {
    case ListState::Vanilla: return "vanilla";
    case ListState::Grown: return "grown";
    case ListState::Added: return "added";
    case ListState::Foreign: return "foreign";
    }
    return "?";
}

struct ListPatch
{
    std::vector<uint64_t> managed; // groups whose LookAt leaves UTPP morphs (from the walk)
    ListState state = ListState::Vanilla;
    uint64_t comp = 0;             // compiled object the state refers to
    uint32_t base = 0;             // list size before our append (= vanilla members)
    std::vector<uint64_t> added;   // appended groups, in order
    std::vector<uint64_t> addedRc; // their refcount blocks (the handle UTPP wrote), same order
    uint64_t data = 0;             // list buffer the append went into
    bool keySet = false;           // key was 0 and set to the compiled object by us
    double since = 0.0;            // time of the last grow
    double removedAt = 0.0;        // time of the last remove (valid when removes > 0)
    // Groups that could not be taken out of the list when the patch became Foreign: possibly still listed, so their
    // LookAt predicates must stay morphed (a listed group with a restored LookAt would bypass the look-at gate).
    std::vector<uint64_t> pinned;
    std::vector<uint64_t> leaked;  // replaced buffers, never freed (a reader may still iterate one)
    std::vector<std::pair<uint64_t, uint64_t>> owned; // slot address -> group instance whose strong ref UTPP holds
    uint32_t grows = 0, adds = 0, removes = 0, replaced = 0;
    const char* why = "";
};

namespace detail
{
inline bool Foreign(ListPatch& aP, const char* aWhy)
{
    aP.state = ListState::Foreign;
    aP.why = aWhy;
    return false;
}

inline bool OwnsSlot(const ListPatch& aP, uint64_t aSlot, uint64_t aInst)
{
    for (const auto& o : aP.owned)
        if (o.first == aSlot && o.second == aInst)
            return true;
    return false;
}

inline void SetOwned(ListPatch& aP, uint64_t aSlot, uint64_t aInst)
{
    for (auto& o : aP.owned)
        if (o.first == aSlot)
        {
            o.second = aInst;
            return;
        }
    aP.owned.emplace_back(aSlot, aInst);
}

// Leaves Added for Foreign (aWhy). Three outcomes:
//   restore : the resource still holds the compiled object UTPP appended to (so it is alive) and the buffer, size and
//             tail slots are exactly what UTPP wrote -> size back to the vanilla count, then the key if UTPP set it and
//             it is still the compiled object (same order as a remove). Nothing pinned.
//   gone    : the list is readable and none of the added groups is in it any more -> no write, nothing pinned.
//   pinned  : anything else (object replaced / unreadable, other entries after ours, write failure) -> no write; the
//             added groups go to 'pinned' and their predicates must stay morphed.
// aMayWrite = false: only the gone / pinned decision (used after a failed write of the remove itself).
inline bool LeaveAdded(IListMemory& aM, const CompiledLayout& aL, uint64_t aRes, ListPatch& aP, bool aMayWrite,
                       const char* aWhy)
{
    std::vector<uint64_t> pin = aP.added;
    uint64_t comp = 0, data = 0;
    uint32_t cap = 0, size = 0;
    const bool alive = aRes && aP.comp && aM.R64(aRes + aL.resCompiled, comp) && comp == aP.comp;
    if (alive && aM.R64(comp + aL.list, data) && aM.R32(comp + aL.list + 8, cap) &&
        aM.R32(comp + aL.list + 0xC, size) && size <= cap && cap <= kMaxListCapacity && (cap == 0 || (data & 7) == 0))
    {
        const size_t n = aP.added.size();
        bool exact = aMayWrite && n != 0 && aP.addedRc.size() == n && data == aP.data && size == aP.base + n;
        for (size_t k = 0; k < n && exact; ++k)
        {
            const uint64_t slot = data + (aP.base + k) * kHandleSize;
            uint64_t inst = 0, rc = 0;
            exact = aM.R64(slot, inst) && aM.R64(slot + 8, rc) && inst == aP.added[k] && rc == aP.addedRc[k] &&
                    OwnsSlot(aP, slot, inst);
        }
        if (exact && aM.W32(comp + aL.list + 0xC, aP.base))
        {
            pin.clear();
            uint64_t key = 0;
            if (aP.keySet && aM.R64(comp + aL.key, key) && key == comp)
                aM.W64(comp + aL.key, 0); // a failure leaves key = compiled with the vanilla members: still not listed
            ++aP.removes;
        }
        else
        {
            bool readable = true;
            std::vector<uint64_t> listed;
            for (uint32_t i = 0; i < size && readable; ++i)
            {
                uint64_t inst = 0;
                readable = aM.R64(data + i * kHandleSize, inst);
                for (uint64_t g : aP.added)
                    if (inst == g)
                        listed.push_back(g);
            }
            if (readable)
                pin = listed;
        }
    }
    aP.pinned = pin;
    aP.added.clear();
    aP.addedRc.clear();
    aP.keySet = false;
    return Foreign(aP, aWhy);
}
} // namespace detail

// True when aGroup must keep its LookAt predicates morphed (see ListPatch::pinned).
inline bool Pins(const ListPatch& aP, uint64_t aGroup)
{
    for (uint64_t g : aP.pinned)
        if (g == aGroup)
            return true;
    return false;
}

// One step towards "aWant appended" (aWant = managed groups whose leaves are all morphed; empty = vanilla).
// Reader safety (a reader may load 'entries' and 'size' at different moments, on another thread):
//   grow   : a new buffer from the list's own allocator gets copies of the current elements (strong count +1 each,
//            as a Handle copy) and the allocator qword; then 'entries' is published, then 'capacity'. The old buffer
//            is never freed, so (old entries, size) and (new entries, size) both read valid handles.
//   append : (>= minGap after a grow) element written beyond 'size' (strong count +1), key set if it was 0, then
//            'size' is raised with one aligned store.
//   remove : 'size' back to the vanilla count first, then key restored. The element stays in place (still a valid
//            handle, its reference stays owned by UTPP), so a reader still using the old size reads valid data. A slot
//            is given a different handle only >= minGap after the last remove.
//   failure while added: detail::LeaveAdded (restore the vanilla size / key, or pin the added groups).
// The game itself writes the list only while the descriptor is loaded; UTPP keeps the descriptor loaded.
// Returns true when the list is in the wanted state afterwards.
inline bool StepList(IListMemory& aM, const CompiledLayout& aL, const loot::DescriptorOffsets& aO, uint64_t aRes,
                     ListPatch& aP, const std::vector<uint64_t>& aWant, double aNow, double aMinGap)
{
    if (aP.state == ListState::Foreign)
        return false;
    if (aWant.empty() && aP.state != ListState::Added)
        return true; // vanilla membership (a grown buffer holds the same members)
    ListProbe p = ProbeList(aM, aL, aO, aRes);
    if (aP.comp != 0 && p.comp != 0 && p.comp != aP.comp)
    {
        // Compiled data rebuilt: our additions went with the old object (released by its owner); start over.
        aP.state = ListState::Vanilla;
        aP.added.clear();
        aP.addedRc.clear();
        aP.owned.clear();
        aP.keySet = false;
        ++aP.replaced;
    }
    // Every failure while Added goes through LeaveAdded (restore / gone / pinned), never a silent half state.
    auto fail = [&](const char* aWhy, bool aMayWrite = true) {
        if (aP.state == ListState::Added)
            return detail::LeaveAdded(aM, aL, aRes, aP, aMayWrite, aWhy);
        return detail::Foreign(aP, aWhy);
    };
    if (!p.ok)
        return fail(p.why);
    aP.comp = p.comp;
    const uint64_t sizeAddr = p.comp + aL.list + 0xC;

    // Wanted additions = aWant minus vanilla members (members below the base when we appended).
    const uint32_t vanillaCount = aP.state == ListState::Added ? aP.base : p.size;
    std::vector<uint64_t> toAdd;
    for (uint64_t g : aWant)
    {
        if (IndexOfGroup(p.groups, g) < 0)
            return fail("wanted group not in this descriptor");
        bool member = false;
        for (uint32_t i = 0; i < vanillaCount && i < p.list.size(); ++i)
            member = member || p.list[i].inst == g;
        if (!member)
            toAdd.push_back(g);
    }

    if (aP.state == ListState::Added)
    {
        if (p.size != aP.base + aP.added.size())
            return fail("list size changed by another writer");
        for (size_t k = 0; k < aP.added.size(); ++k)
            if (p.list[aP.base + k].inst != aP.added[k])
                return fail("list tail changed by another writer");
        if (toAdd == aP.added)
            return true;
        if (!aM.W32(sizeAddr, aP.base))
            return fail("size write failed", false);
        if (aP.keySet && !aM.W64(p.comp + aL.key, 0))
            return fail("key restore failed", false); // size is already the vanilla count: gone, not pinned
        aP.state = ListState::Vanilla;
        aP.added.clear();
        aP.addedRc.clear();
        aP.keySet = false;
        aP.removedAt = aNow;
        ++aP.removes;
        return aWant.empty(); // a different set is appended on the next step
    }

    if (toAdd.empty())
    {
        aP.why = "already listed";
        return true;
    }
    const uint32_t need = p.size + static_cast<uint32_t>(toAdd.size());
    if (p.cap < need)
    {
        if (aP.grows >= kMaxListGrows)
            return fail("too many list grows");
        if (need > kMaxListCapacity)
            return fail("list would exceed the capacity bound");
        const uint64_t bytes = ListBufferBytes(need);
        const uint64_t buf = aM.Alloc(p.allocator, bytes, 8);
        if (!buf || (buf & 7) != 0)
            return fail("allocation failed");
        for (uint64_t q = 0; q < bytes; q += 8)
            if (!aM.W64(buf + q, 0))
                return fail("new buffer write failed");
        for (uint32_t i = 0; i < p.size; ++i)
        {
            if (!aM.IncStrong(p.list[i].rc) || !aM.W64(buf + i * kHandleSize, p.list[i].inst) ||
                !aM.W64(buf + i * kHandleSize + 8, p.list[i].rc))
                return fail("copy to the new buffer failed");
        }
        if (!aM.W64(ListAllocatorSlot(buf, need), p.allocator))
            return fail("allocator slot write failed");
        if (!aM.W64(p.comp + aL.list, buf) || !aM.W32(p.comp + aL.list + 8, need))
            return fail("publish failed");
        if (p.cap != 0)
            aP.leaked.push_back(p.data);
        aP.state = ListState::Grown;
        aP.since = aNow;
        ++aP.grows;
        return false;
    }
    if (aP.state == ListState::Grown && aNow - aP.since < aMinGap)
        return false;
    // Which slots need a new handle. A slot that was below 'size' before a recent remove may still be read by a reader
    // holding that old size: it is rewritten only minGap after the remove (same grace period as after a grow).
    std::vector<GroupHandle> handles;
    std::vector<bool> rewrite;
    bool anyRewrite = false;
    for (size_t k = 0; k < toAdd.size(); ++k)
    {
        const uint64_t slot = p.data + (p.size + k) * kHandleSize;
        const GroupHandle h = p.groups[static_cast<size_t>(IndexOfGroup(p.groups, toAdd[k]))];
        if (!h.rc)
            return fail("group handle without refcount");
        uint64_t ci = 0, cr = 0;
        if (!aM.R64(slot, ci) || !aM.R64(slot + 8, cr))
            return fail("slot unreadable");
        handles.push_back(h);
        rewrite.push_back(!(ci == h.inst && cr == h.rc && detail::OwnsSlot(aP, slot, h.inst)));
        anyRewrite = anyRewrite || rewrite.back();
    }
    if (anyRewrite && aP.removes > 0 && aNow - aP.removedAt < aMinGap)
    {
        aP.why = "waiting after a remove";
        return false;
    }
    for (size_t k = 0; k < toAdd.size(); ++k)
    {
        const uint64_t slot = p.data + (p.size + k) * kHandleSize;
        const GroupHandle h = handles[k];
        if (rewrite[k])
        {
            if (!aM.IncStrong(h.rc))
                return fail("refcount increment failed");
            detail::SetOwned(aP, slot, h.inst);
            if (!aM.W64(slot + 8, h.rc) || !aM.W64(slot, h.inst))
                return fail("element write failed");
        }
    }
    bool keySet = false;
    if (p.key == 0)
    {
        if (!aM.W64(p.comp + aL.key, p.comp))
            return fail("key write failed");
        keySet = true;
    }
    if (!aM.W32(sizeAddr, need))
        return fail("size write failed");
    aP.state = ListState::Added;
    aP.base = p.size;
    aP.added = toAdd;
    aP.addedRc.clear();
    for (const GroupHandle& h : handles)
        aP.addedRc.push_back(h.rc);
    aP.data = p.data;
    aP.keySet = keySet;
    aP.why = "";
    ++aP.adds;
    return true;
}

// Compact one-line description (< 300 chars): live list, managed groups in / out, patch state.
inline std::string ListSummary(const ListProbe& aP, const ListPatch& aPatch)
{
    char buf[320];
    const std::string pinned = aPatch.pinned.empty() ? "" : " pinned " + std::to_string(aPatch.pinned.size());
    if (!aP.ok)
    {
        std::snprintf(buf, sizeof(buf), "list ? (%s) patch %s%s%s%s", aP.why, ListStateName(aPatch.state),
                      aPatch.why[0] ? " " : "", aPatch.why, pinned.c_str());
        return buf;
    }
    std::string members, managed;
    for (const auto& h : aP.list)
        members += (members.empty() ? "g" : ",g") + std::to_string(IndexOfGroup(aP.groups, h.inst));
    for (uint64_t g : aPatch.managed)
    {
        bool in = false;
        for (const auto& h : aP.list)
            in = in || h.inst == g;
        managed += (managed.empty() ? "g" : " g") + std::to_string(IndexOfGroup(aP.groups, g)) + (in ? ":in" : ":out");
    }
    std::snprintf(buf, sizeof(buf), "list %u/%u key=%s groups %zu [%s] managed [%s] patch %s +%zu grows %u adds %u "
                                    "removes %u%s%s%s",
                  aP.size, aP.cap, aP.key == 0 ? "0" : "comp", aP.groups.size(), members.c_str(), managed.c_str(),
                  ListStateName(aPatch.state), aPatch.added.size(), aPatch.grows, aPatch.adds, aPatch.removes,
                  aPatch.why[0] ? " " : "", aPatch.why, pinned.c_str());
    return buf;
}
} // namespace utpp::interact

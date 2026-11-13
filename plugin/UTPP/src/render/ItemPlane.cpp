// UTPP item render plane (0.6.4): why the weapon is drawn through V's body in TPP and the universal fix.
//
// Evidence (2.31, Ghidra, exe 3.0.80.51928):
//  1. The player entity (player_wa_fpp.ent / player_ma_fpp.ent, also the _ep1 variants) has a gameAttachmentSlots component
//     whose animParams bind slots to the animation parameter "renderPlane" with function RenderingPlane:
//     WeaponRight, WeaponLeft, PersonalLink, SilverhandArm (+ "renderPlaneInspect" for slot 0).
//  2. gameAttachmentSlots attach (FUN_1407f9f54) walks animParams (DynArray at +0xE0, stride 0x18) for the slot the item is
//     attached to and creates, for each visual component of the item, a binding object (0x68 bytes, FUN_14046ce08) that
//     listens to the owner's animation parameters.
//  3. The animation parameter broadcast (job "Broadcast AnimParams" -> FUN_14015decc) runs whenever a parameter value changed
//     (or a new listener made it dirty); for a RenderingPlane binding FUN_14015e760 maps the float to a plane
//     (< 0.5 scene, < 1.5 background, else weapon; constants at RVA 0x31ef230 / 0x31ef494) and calls the component's plane
//     setter FUN_14015e874(component, plane) (int at component+0x138).
//  4. gameItemEventsPropagateRenderingPlane: ctor (FUN_1424f25f8) zeroes the int32 at +0x40 (= RPl_Scene); the ItemObject
//     handler (FUN_140c7c130) reads +0x40 and applies it to the item and its child items (FUN_140658438). So the UTPP 0.6.3
//     script events DID apply RPl_Scene, but the player's FPP animation graph keeps driving "renderPlane" = weapon (2) and every
//     change of the parameter re-applies RPl_Weapon to the held item through the binding of step 2. One-shot events lose.
//
// Fix: two detours, gated by a TPP override that the script sets (UTPPSetItemPlaneOverride):
//  - FUN_14015decc (broadcast body): marks the current thread as "inside an animation parameter broadcast" (depth counter);
//  - FUN_14015e874 (visual component plane setter): inside a broadcast, with the override on, RPl_Weapon (2) -> RPl_Scene (0).
//  Outside a broadcast (equip spawn, gameItemEventsPropagateRenderingPlane, quests) nothing changes, so the FPP restore event
//  (plane 2 via UTPPSetItemRenderPlane) still works. Only RPl_Weapon is rewritten, only while the override is on.
//
// 2026-11-01 registry + re-apply (one render-plane fix for V in TPP; pure policy / registry in ItemPlaneCore.hpp):
//  The detour alone latched: components set by the FPP graph before the override turned on kept RPl_Weapon until the parameter
//  changed again, and components rewritten to RPl_Scene kept it after the override turned off. Now every setter call inside a
//  broadcast records the component (its own weak reference, ISerializable::ref at +0x08) and the plane vanilla requested;
//  SetOverride (on a change) and UTPPItemPlaneReapply write core::PlaneFor(override, requested) to every live registered component
//  (force: 2 -> 0; vanilla: exactly the requested value). With the override off the detour writes exactly what it wrote before
//  (the registry only observes). Outside-broadcast writes are never rewritten and mark the entry so the re-apply leaves it alone.
//  Step 0 (2.50 exe, capstone, 2026-11-01): the setter at 0x208E78 reads only rcx (component) and edx (plane):
//  mov esi,[rcx+138h]; mov edi,edx; mov [rbx+138h],edi (raw value, 4 included); then vfunc +0x278 when the old or new value is
//  the sentinel 4 (rcx = component) and vfunc +0x280 when the value changed (rcx = component, rdx = plane on that path); the vfunc
//  implementations were NOT inspected. Its four direct call sites (E8/E9 scan of .text; indirect references not searched) pass
//  unrelated r8 / r9 (broadcast map 0xB6784B: r8 = the reference block of the weak handle it locked, r9 = the component;
//  0x209078 / 0x20A2B4: a stack pointer left by an earlier call; 0xD48262: whatever its caller passed), so no consistent r8 / r9
//  contract exists; the re-apply passes rcx / edx exactly like those call sites and nullptr for r8 / r9. The vanilla ItemObject
//  handler 0xA8469C reads the event plane (+0x40) into a deferred object built by 0xA84844 (vtable 0x30AFB78, plane at +0x40,
//  a pointer at +0x48). 0xD48250 is a step that calls setter(component, [ctx+40h]) when [ctx+48h] is 0 or equals the
//  component's +0x40 (name); linking it to that handler rests only on the matching field layout (no xref found: not in the
//  vtable, no E8/E9 callers, no .rdata pointer). Hypothesis: vanilla calls the setter directly with (component, plane) from
//  entity event processing on the game thread (not observed). The broadcast map locks the binding's weak handle (binding
//  +0x40 / +0x48, lock cmpxchg on the strong count) before the call: the same weak-reference pattern the registry uses.
//  Threads: the detour may run on job threads (thread_local depth); the registry is guarded by an SRW lock held only for the
//  probe / snapshot (never while a setter runs). The re-apply, prune and dump run only from script natives (game thread) and call
//  the trampoline g_origSetPlane under a thread_local "reapplying" guard, so writes it causes (also through vfunc +0x280) are
//  neither recorded nor counted.
//  Hot path: thread_local ints, relaxed atomics, one short SRW-locked probe of a preallocated table, no allocation, no logging.
//
// 2026-11-01 round 2 (render-plane audit; NG1 / SPECIAL-2 / dump):
//  - NG1, anim-param handler gate. After computing the parameters the broadcast also queues an entAnimParamsEvent on the owner
//    entity (0x53CA6B container, 0x53CB2A -> 0x1A67B0 -> queue at entity+0xD8); when the entity drains it, the event callbacks
//    0x20A240 (SkinnedMesh / garment: CName renderingPlaneAnimationParam at +0x238) and 0x209000 (MorphTargetSkinnedMesh: +0x2E8)
//    call the setter (0x20A2B4 / 0x209078) with NO broadcast frame. The same two functions are also called directly inside the
//    broadcast (0x53CBD9 / 0x53CC0B). Two more detours (++t_handlerDepth, call, --) make every write from either handler an
//    anim-param write: the setter gate is core::InAnimParam(broadcast depth, handler depth). No return-address whitelist (a
//    trampoline of another mod would break it). 2.50 bytes checked against the game exe (SHA-256 d36e74c4...fab4, capstone): both
//    prologues are 'mov [rsp+10h],rbx; push rdi; sub rsp,20h; mov rdi,rdx; lea r8,[rsp+30h]; mov rdx,[rcx+238h|2E8h]' (25 bytes,
//    nothing RIP-relative), the only E8/E9 callers are the two broadcast call sites above (the event registrations 0x1845DC5 /
//    0xFC11A7 use RIP-relative lea, so absolute-pointer scans find nothing; that does not matter for an inline detour). Signature
//    (rcx component, rdx event), r8 / r9 unused on entry; the detours forward all four registers and the original's rax. Each
//    handler hook is optional: a prologue mismatch or Attach failure leaves that handler unhooked (gate degrades to broadcast
//    depth only for it), logs why and keeps the rest of the fix.
//  - SPECIAL-2: outside writes that agree with the policy clear outsideLast (core NoteOutside); no native change beyond passing
//    the override.
//  - Dump: ownerChain column (player / item:<slot>:<tdbid> / child:<slot>/<slot>:<tdbid> / other) and DumpPlayer (every visual,
//    effect spawner and particles component of the player, the items in its attachment slots and their child items).

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/ScriptGameInstance.hpp>
#include <RED4ext/Scripting/Natives/entEntity.hpp>
#include <RED4ext/Scripting/Stack.hpp>
#include <RED4ext/Scripting/Utils.hpp>

#include "../CommonUtil.hpp"
#include "../PluginLog.hpp"
#include "ItemPlane.hpp"
#include "ItemPlaneCore.hpp"

namespace utpp::itemplane
{
namespace
{
constexpr uint32_t kRvaBroadcast = 0x53C978; // FUN_14015decc, animation parameter broadcast body
constexpr uint32_t kRvaSetPlane = 0x208E78;  // FUN_14015e874, IVisualComponent rendering plane setter
constexpr uint8_t kBroadcastPrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x55,
                                          0x57, 0x41, 0x54, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48,
                                          0x83, 0xEC, 0x70, 0x83, 0xB9, 0x84, 0x02, 0x00, 0x00, 0x00};
constexpr uint8_t kSetPlanePrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57,
                                         0x48, 0x83, 0xEC, 0x20, 0x8B, 0xB1, 0x38, 0x01, 0x00, 0x00, 0x8B,
                                         0xFA, 0x48, 0x8B, 0xD9, 0x83, 0xFE, 0x04, 0x74, 0x1F};
// Round 2 (NG1): the two entAnimParamsEvent handlers that call the setter (see the header). 25 bytes each, through the load of the
// renderingPlaneAnimationParam CName, so the check also pins which handler is hooked.
constexpr uint32_t kRvaSkinnedHandler = 0x20A240; // SkinnedMesh / garment components, param CName at +0x238
constexpr uint32_t kRvaMorphHandler = 0x209000;   // MorphTargetSkinnedMesh components, param CName at +0x2E8
constexpr uint8_t kSkinnedHandlerPrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0xFA,
                                               0x4C, 0x8D, 0x44, 0x24, 0x30, 0x48, 0x8B, 0x91, 0x38, 0x02, 0x00, 0x00};
constexpr uint8_t kMorphHandlerPrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0xFA,
                                             0x4C, 0x8D, 0x44, 0x24, 0x30, 0x48, 0x8B, 0x91, 0xE8, 0x02, 0x00, 0x00};
// 2.50 (T2 audit 2026-10-23): broadcast (2.31 0x15DECC), setter (0x15E874) and the float -> plane map (0x15E760 ->
// 0xB67788: < 0.5 scene, < 1.5 background, else weapon) are instruction-identical apart from call targets; the ItemObject
// handler (2.31 0xC7C130) is found once on 2.50 at 0xA8469C with the same "mov eax,[rdx+40h]"; ERenderingPlane is unchanged.
constexpr uint32_t kEventSize = 0x48;     // gameItemEventsPropagateRenderingPlane (RED4ext.SDK generated header; checked at runtime)
constexpr uint32_t kEventPlaneOff = 0x40; // int32 read by the ItemObject handler (2.31 FUN_140c7c130, 2.50 RVA 0xA8469C)
constexpr int32_t kPlaneScene = 0;
constexpr int32_t kPlaneWeapon = 2;

using Broadcast_t = void (*)(void*, void*, void*, void*);
using SetPlane_t = void (*)(void*, int32_t, void*, void*);
using Handler_t = uint64_t (*)(void*, void*, void*, void*); // (component, event); rax forwarded as is

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;

Broadcast_t g_origBroadcast = nullptr;
SetPlane_t g_origSetPlane = nullptr;
void* g_targetBroadcast = nullptr;
void* g_targetSetPlane = nullptr;
Handler_t g_origSkinnedHandler = nullptr;
Handler_t g_origMorphHandler = nullptr;
void* g_targetSkinnedHandler = nullptr;
void* g_targetMorphHandler = nullptr;
std::atomic<int> g_handlerHooks{0}; // 0..2 handler detours attached (Status gate= / handlerHooks=)

std::atomic<bool> g_installed{false};
std::atomic<bool> g_override{false};
std::atomic<uint64_t> g_broadcasts{0};    // broadcasts entered while the override was on
std::atomic<uint64_t> g_weaponInBc{0};    // RPl_Weapon writes seen inside a broadcast (any override state)
std::atomic<uint64_t> g_forced{0};        // RPl_Weapon -> RPl_Scene rewrites
std::atomic<uint64_t> g_eventsScene{0};   // UTPPSetItemRenderPlane(plane 0) queued
std::atomic<uint64_t> g_eventsWeapon{0};  // UTPPSetItemRenderPlane(plane 2) queued
std::atomic<uint64_t> g_eventsOther{0};   // other planes queued
std::atomic<uint64_t> g_eventFails{0};

// 2026-11-01 registry counters (Status: registry= reapplied= bgInBc= weaponOutBc= regDropped= noRef= pruned= reapplies=).
std::atomic<uint64_t> g_bgInBc{0};       // RPl_Background requested inside a broadcast (counted, not rewritten)
std::atomic<uint64_t> g_weaponOutBc{0};  // RPl_Weapon requested outside a broadcast (never rewritten)
std::atomic<uint64_t> g_regDropped{0};   // new components not registered: registry full
std::atomic<uint64_t> g_noRef{0};        // broadcast writes to a component without its own weak reference (not registered)
std::atomic<uint64_t> g_reapplied{0};    // setter calls made by the re-apply (planes actually changed)
std::atomic<uint64_t> g_pruned{0};       // dead entries removed
std::atomic<uint64_t> g_reapplies{0};    // re-apply passes run
std::atomic<uint64_t> g_handlerQueued{0}; // handler entries without a broadcast frame while the override was on (NG1 path)
std::atomic<uint64_t> g_orphanSkipped{0}; // re-apply writes skipped: component without an owner (detached; the game setter would crash)
std::atomic<uint64_t> g_orphanLogs{0};    // re-applies that skipped orphans (rate limit of the log line)

constexpr uint32_t kOffPlane = 0x138; // int32 rendering plane of the visual component (read by the setter)
constexpr uint32_t kOffName = 0x40;   // entIComponent name (CName)
constexpr uint32_t kOffOwner = 0x50;  // entIComponent owner entity (RED4ext.SDK entIComponent.hpp; used the same way in GameLook.cpp)
constexpr size_t kRegSlots = 2048;    // cap = 1536 live components (V's bound body / garment / item parts are far fewer; NPCs share it)
using Registry = core::Registry<kRegSlots>;

// Guarded by g_regLock (SRW: no allocation, no exceptions; held only for a probe or a snapshot, never while a setter runs).
SRWLOCK g_regLock = SRWLOCK_INIT;
Registry g_reg;

std::mutex g_coldMutex; // cold state below (game thread / load)
std::string g_installWhy = "not installed (Install not called)";
std::string g_lastFail;

thread_local int t_broadcastDepth = 0;
thread_local int t_handlerDepth = 0; // > 0 inside the Skinned / MorphTarget entAnimParamsEvent handler (round 2, NG1)
thread_local int t_reapplying = 0; // > 0 while the re-apply calls the trampoline: those writes are neither recorded nor counted

struct RegLock
{
    RegLock()
    {
        AcquireSRWLockExclusive(&g_regLock);
    }
    ~RegLock()
    {
        ReleaseSRWLockExclusive(&g_regLock);
    }
    RegLock(const RegLock&) = delete;
    RegLock& operator=(const RegLock&) = delete;
};

// The component's own weak reference (ISerializable::ref, set by the Handle ctor). Usable only when it points back at the
// component; otherwise (never held by a Handle) the component is written as before but not registered.
struct RefKey
{
    uintptr_t instance = 0;
    uintptr_t refCnt = 0;
};
RefKey ReadRef(void* aComponent)
{
    if (!aComponent)
        return {};
    auto* s = static_cast<RED4ext::ISerializable*>(aComponent);
    if (s->ref.instance != s || !s->ref.refCount)
        return {};
    return {reinterpret_cast<uintptr_t>(aComponent), reinterpret_cast<uintptr_t>(s->ref.refCount)};
}

RED4ext::RefCnt* AsRefCnt(uintptr_t aRefCnt)
{
    return reinterpret_cast<RED4ext::RefCnt*>(aRefCnt);
}

// Game thread, under g_regLock: removes entries whose strong count is 0 (the registry's weak reference keeps the block readable)
// and hands their weak references to aRelease (released by the caller after the lock).
size_t PruneLocked(std::vector<RED4ext::WeakHandle<RED4ext::ISerializable>>& aRelease)
{
    const size_t removed = g_reg.Prune([](const core::Entry& e) { return AsRefCnt(e.refCnt)->strongRefs == 0; },
                                       [&](const core::Entry& e) {
                                           aRelease.emplace_back();
                                           aRelease.back().instance = reinterpret_cast<RED4ext::ISerializable*>(e.instance);
                                           aRelease.back().refCount = AsRefCnt(e.refCnt);
                                       });
    if (removed)
        g_pruned.fetch_add(removed, std::memory_order_relaxed);
    return removed;
}

// Setter-detour reclaim (2026-11-01, review finding: a long TPP session without policy edges filled the registry with dead
// entries and new components, V's included, were dropped). core::Registry::RecordInside calls ReclaimInDetour when a NEW
// component would exceed the cap, under g_regLock. It only reads the reference-count blocks the registry's weak references keep
// readable, runs core Prune (no allocation) with a bounded batch and stores the removed entries in a per-thread fixed array; the
// weak references are released by ReleaseReclaimed AFTER the lock is dropped (DecWeakRef is the game's own handle destructor, the
// same call any WeakHandle makes on any game thread; it is never made under g_regLock). PruneLocked / SnapshotLive are not usable
// here: they grow a std::vector under the lock and are documented game-thread only.
constexpr size_t kReclaimBatch = 64;
struct ReclaimBatch
{
    core::Entry entries[kReclaimBatch];
    size_t count = 0;
};
thread_local ReclaimBatch t_reclaim;

size_t ReclaimInDetour()
{
    t_reclaim.count = 0;
    const size_t removed =
        g_reg.Prune([](const core::Entry& e) { return t_reclaim.count < kReclaimBatch && AsRefCnt(e.refCnt)->strongRefs == 0; },
                    [](const core::Entry& e) { t_reclaim.entries[t_reclaim.count++] = e; });
    if (removed)
        g_pruned.fetch_add(removed, std::memory_order_relaxed);
    return removed;
}

// Outside g_regLock: releases the weak references of the entries ReclaimInDetour removed.
void ReleaseReclaimed()
{
    const size_t n = t_reclaim.count;
    t_reclaim.count = 0;
    for (size_t i = 0; i < n; ++i)
    {
        RED4ext::WeakHandle<RED4ext::ISerializable> h; // destructor = DecWeakRef
        h.instance = reinterpret_cast<RED4ext::ISerializable*>(t_reclaim.entries[i].instance);
        h.refCount = AsRefCnt(t_reclaim.entries[i].refCnt);
    }
}

// A live entry pinned by a strong reference taken under the lock (released when the snapshot is destroyed, as the broadcast
// map releases the handle it locked).
struct Pinned
{
    RED4ext::Handle<RED4ext::ISerializable> handle;
    core::Entry entry;
};

// Game thread: prune, then pin every live entry (up to aMax). aRelease receives the pruned weak references.
void SnapshotLive(std::vector<Pinned>& aOut, size_t aMax, std::vector<RED4ext::WeakHandle<RED4ext::ISerializable>>& aRelease,
                  size_t* aSize)
{
    aOut.reserve(Registry::kMaxEntries < aMax ? Registry::kMaxEntries : aMax);
    RegLock lock;
    PruneLocked(aRelease);
    if (aSize)
        *aSize = g_reg.Size();
    g_reg.ForEach([&](const core::Entry& e) {
        if (aOut.size() >= aMax)
            return;
        if (!AsRefCnt(e.refCnt)->IncRefIfNotZero())
            return; // died since the prune
        aOut.emplace_back();
        aOut.back().handle.instance = reinterpret_cast<RED4ext::ISerializable*>(e.instance); // adopts the strong reference
        aOut.back().handle.refCount = AsRefCnt(e.refCnt);
        aOut.back().entry = e;
    });
}

int32_t PlaneOf(const RED4ext::ISerializable* aComponent)
{
    return *reinterpret_cast<const int32_t*>(reinterpret_cast<const uint8_t*>(aComponent) + kOffPlane);
}

// The owner entity of a component (entIComponent +0x50), null when the component is detached from its entity. Only for a
// component the caller holds a strong reference to (Pinned): the object and this field are then readable.
const void* OwnerOf(const RED4ext::ISerializable* aComponent)
{
    return *reinterpret_cast<const void* const*>(reinterpret_cast<const uint8_t*>(aComponent) + kOffOwner);
}

// Game thread only (script natives). Writes core::PlaneFor(override, requested) to every live registered component whose last
// write came from a broadcast, through the trampoline (the real setter), only where the plane differs. Returns the writes.
int32_t ReapplyNow()
{
    if (!g_installed.load(std::memory_order_acquire) || !g_origSetPlane)
        return 0;
    const bool force = g_override.load(std::memory_order_relaxed);
    std::vector<Pinned> live;
    std::vector<RED4ext::WeakHandle<RED4ext::ISerializable>> release;
    SnapshotLive(live, Registry::kMaxEntries, release, nullptr);
    release.clear(); // DecWeakRef outside the lock
    int32_t written = 0;
    int32_t orphans = 0;
    ++t_reapplying;
    for (const Pinned& p : live)
    {
        // The owner is read right before the call (game thread, the entry is pinned): the game's setter dereferences it, and an
        // earlier setter call of this loop may have detached a later component. A detached component is skipped, kept in the registry.
        const core::ReapplyStep step =
            core::PlanReapplyAttached(force, p.entry, PlaneOf(p.handle.instance), OwnerOf(p.handle.instance) != nullptr);
        if (step.orphan)
        {
            ++orphans;
            continue;
        }
        if (!step.write)
            continue;
        g_origSetPlane(p.handle.instance, step.plane, nullptr, nullptr);
        ++written;
    }
    --t_reapplying;
    g_reapplies.fetch_add(1, std::memory_order_relaxed);
    g_reapplied.fetch_add(static_cast<uint64_t>(written), std::memory_order_relaxed);
    if (orphans)
    {
        g_orphanSkipped.fetch_add(static_cast<uint64_t>(orphans), std::memory_order_relaxed);
        // One line per re-apply that skipped something (re-applies run on policy edges and explicit natives only); after the first
        // 16 only every 16th, so a loop of edges cannot flood the log.
        const uint64_t n = g_orphanLogs.fetch_add(1, std::memory_order_relaxed);
        if (n < 16 || (n % 16) == 0)
            pluginlog::LogWarn("UTPP itemplane: re-apply skipped %d component(s) without an owner (detached, kept in the registry; "
                               "force=%d written=%d skippedTotal=%llu)",
                               orphans, force ? 1 : 0, written,
                               static_cast<unsigned long long>(g_orphanSkipped.load(std::memory_order_relaxed)));
    }
    return written;
}

using pluginlog::LogInfo;
using pluginlog::LogWarn;

void DetourBroadcast(void* a1, void* a2, void* a3, void* a4)
{
    const bool on = g_override.load(std::memory_order_relaxed);
    if (on)
        g_broadcasts.fetch_add(1, std::memory_order_relaxed);
    ++t_broadcastDepth;
    g_origBroadcast(a1, a2, a3, a4);
    --t_broadcastDepth;
}

// Round 2 (NG1): the entAnimParamsEvent handlers. Every setter call they make carries an animation-parameter plane, also when the
// owner's queued event runs them outside the broadcast frame. Same pattern as DetourBroadcast; all registers and rax forwarded.
void CountQueuedHandler()
{
    if (t_broadcastDepth == 0 && g_override.load(std::memory_order_relaxed))
        g_handlerQueued.fetch_add(1, std::memory_order_relaxed);
}
uint64_t DetourSkinnedHandler(void* a1, void* a2, void* a3, void* a4)
{
    CountQueuedHandler();
    ++t_handlerDepth;
    const uint64_t r = g_origSkinnedHandler(a1, a2, a3, a4);
    --t_handlerDepth;
    return r;
}
uint64_t DetourMorphHandler(void* a1, void* a2, void* a3, void* a4)
{
    CountQueuedHandler();
    ++t_handlerDepth;
    const uint64_t r = g_origMorphHandler(a1, a2, a3, a4);
    --t_handlerDepth;
    return r;
}

void DetourSetPlane(void* aComponent, int32_t aPlane, void* a3, void* a4)
{
    if (t_reapplying > 0)
    {
        g_origSetPlane(aComponent, aPlane, a3, a4); // our own re-apply (incl. vfunc +0x280 propagation): untouched, unrecorded
        return;
    }
    // The plane decision is core::DetourPlane (identical to 0.6.4: inside a broadcast, with the override on, 2 -> 0; everything
    // else unchanged); the registry only observes. Round 2: "inside a broadcast" = core::InAnimParam (broadcast frame or one of
    // the anim-param event handlers; the handler depth stays 0 when its hook is not installed).
    const bool inBc = core::InAnimParam(t_broadcastDepth, t_handlerDepth);
    const RefKey key = ReadRef(aComponent);
    core::SetterOutcome o;
    {
        RegLock lock;
        o = core::OnSetterCall(g_reg, inBc, g_override.load(std::memory_order_relaxed), key.instance, key.refCnt, aPlane,
                               [&] { AsRefCnt(key.refCnt)->IncWeakRef(); }, ReclaimInDetour);
    }
    ReleaseReclaimed(); // weak references of entries reclaimed above (no-op unless a new component hit the cap)
    if (o.weaponInBc)
        g_weaponInBc.fetch_add(1, std::memory_order_relaxed);
    if (o.forced)
        g_forced.fetch_add(1, std::memory_order_relaxed);
    if (o.bgInBc)
        g_bgInBc.fetch_add(1, std::memory_order_relaxed);
    if (o.weaponOutBc)
        g_weaponOutBc.fetch_add(1, std::memory_order_relaxed);
    if (inBc && o.noRef)
        g_noRef.fetch_add(1, std::memory_order_relaxed);
    if (o.recorded && o.record == core::RecordResult::Dropped)
        g_regDropped.fetch_add(1, std::memory_order_relaxed);
    g_origSetPlane(aComponent, o.plane, a3, a4);
}

using util::HexAt;
using util::SehBytesEqual;
using util::SehCopy;

// Round 2 (NG1), called by Install under g_coldMutex after the broadcast and setter hooks are live. One optional handler hook:
// prologue check, then Attach. Returns 1 when attached; on failure nothing is installed for it and aWhy gets the reason.
int InstallHandler(uint8_t* aBase, uint32_t aRva, const uint8_t* aPrologue, size_t aLen, void* aDetour, Handler_t* aOrig,
                   void** aTarget, const char* aName, std::string& aWhy)
{
    uint8_t* target = aBase + aRva;
    char head[96];
    if (!SehBytesEqual(target, aPrologue, aLen))
    {
        std::snprintf(head, sizeof(head), "%s handler prologue @0x%X mismatch [", aName, aRva);
        aWhy += (aWhy.empty() ? "" : "; ") + std::string(head) + HexAt(target, aLen) + "]";
        return 0;
    }
    if (!g_sdk->hooking->Attach(g_handle, target, aDetour, reinterpret_cast<void**>(aOrig)) || !*aOrig)
    {
        *aOrig = nullptr;
        std::snprintf(head, sizeof(head), "hooking->Attach failed on the %s handler @0x%X", aName, aRva);
        aWhy += (aWhy.empty() ? "" : "; ") + std::string(head);
        return 0;
    }
    *aTarget = target;
    return 1;
}
} // namespace

void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    std::lock_guard lock(g_coldMutex);
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !g_sdk || !g_sdk->hooking)
    {
        g_installWhy = "no image base / hooking API";
        LogWarn("UTPP itemplane: hook not installed (%s); TPP weapon plane fix inert", g_installWhy.c_str());
        return;
    }
    uint8_t* bc = base + kRvaBroadcast;
    uint8_t* sp = base + kRvaSetPlane;
    if (!SehBytesEqual(bc, kBroadcastPrologue, sizeof(kBroadcastPrologue)))
    {
        g_installWhy = "broadcast prologue @0x53C978 mismatch [" + HexAt(bc, sizeof(kBroadcastPrologue)) + "]";
        LogWarn("UTPP itemplane: hook not installed (%s); TPP weapon plane fix inert", g_installWhy.c_str());
        return;
    }
    if (!SehBytesEqual(sp, kSetPlanePrologue, sizeof(kSetPlanePrologue)))
    {
        g_installWhy = "plane setter prologue @0x208E78 mismatch [" + HexAt(sp, sizeof(kSetPlanePrologue)) + "]";
        LogWarn("UTPP itemplane: hook not installed (%s); TPP weapon plane fix inert", g_installWhy.c_str());
        return;
    }
    // Setter first: the broadcast detour alone changes nothing; a setter detour without the broadcast one never rewrites.
    if (!g_sdk->hooking->Attach(g_handle, sp, reinterpret_cast<void*>(&DetourSetPlane),
                                reinterpret_cast<void**>(&g_origSetPlane)) ||
        !g_origSetPlane)
    {
        g_origSetPlane = nullptr;
        g_installWhy = "hooking->Attach failed on the plane setter @0x208E78";
        LogWarn("UTPP itemplane: %s; TPP weapon plane fix inert", g_installWhy.c_str());
        return;
    }
    g_targetSetPlane = sp;
    if (!g_sdk->hooking->Attach(g_handle, bc, reinterpret_cast<void*>(&DetourBroadcast),
                                reinterpret_cast<void**>(&g_origBroadcast)) ||
        !g_origBroadcast)
    {
        g_origBroadcast = nullptr;
        g_installWhy = "hooking->Attach failed on the broadcast @0x53C978 (setter hook left as pass-through)";
        LogWarn("UTPP itemplane: %s; TPP weapon plane fix inert", g_installWhy.c_str());
        return;
    }
    g_targetBroadcast = bc;
    g_installed.store(true, std::memory_order_release);
    // Round 2 (NG1): the anim-param event handlers, each optional (without it that handler's writes keep the broadcast-depth-only
    // gate, exactly as before).
    std::string handlerWhy;
    int hooks = InstallHandler(base, kRvaSkinnedHandler, kSkinnedHandlerPrologue, sizeof(kSkinnedHandlerPrologue),
                               reinterpret_cast<void*>(&DetourSkinnedHandler), &g_origSkinnedHandler, &g_targetSkinnedHandler,
                               "skinned", handlerWhy);
    hooks += InstallHandler(base, kRvaMorphHandler, kMorphHandlerPrologue, sizeof(kMorphHandlerPrologue),
                            reinterpret_cast<void*>(&DetourMorphHandler), &g_origMorphHandler, &g_targetMorphHandler, "morph",
                            handlerWhy);
    g_handlerHooks.store(hooks, std::memory_order_release);
    char handlers[64];
    std::snprintf(handlers, sizeof(handlers), "anim-param handlers %d/2", hooks);
    g_installWhy = std::string("installed (broadcast @0x53C978, plane setter @0x208E78, ") + handlers +
                   (handlerWhy.empty() ? std::string(")") : ": " + handlerWhy + ")");
    if (!handlerWhy.empty())
        LogWarn("UTPP itemplane: %s; those handler writes keep the broadcast-only gate", handlerWhy.c_str());
    LogInfo("UTPP itemplane: hooks attached (anim-param broadcast @%p, plane setter @%p, %s); override off until the script sets "
            "it",
            static_cast<void*>(bc), static_cast<void*>(sp), handlers);
}

void Shutdown()
{
    std::lock_guard lock(g_coldMutex);
    g_override.store(false);
    if (g_sdk && g_sdk->hooking)
    {
        if (g_targetSkinnedHandler)
            LogInfo("UTPP itemplane unload: skinned handler detach %s",
                    g_sdk->hooking->Detach(g_handle, g_targetSkinnedHandler) ? "ok" : "FAILED");
        if (g_targetMorphHandler)
            LogInfo("UTPP itemplane unload: morph handler detach %s",
                    g_sdk->hooking->Detach(g_handle, g_targetMorphHandler) ? "ok" : "FAILED");
        if (g_targetBroadcast)
            LogInfo("UTPP itemplane unload: broadcast detach %s",
                    g_sdk->hooking->Detach(g_handle, g_targetBroadcast) ? "ok" : "FAILED");
        if (g_targetSetPlane)
            LogInfo("UTPP itemplane unload: plane setter detach %s",
                    g_sdk->hooking->Detach(g_handle, g_targetSetPlane) ? "ok" : "FAILED");
    }
    g_targetBroadcast = nullptr;
    g_targetSetPlane = nullptr;
    g_targetSkinnedHandler = nullptr;
    g_targetMorphHandler = nullptr;
    g_handlerHooks.store(0);
    g_installed.store(false);
    // Hooks are gone; the weak references are deliberately leaked (no calls into the game's allocators at unload).
    RegLock regLock;
    g_reg.Forget();
}

bool SetOverride(bool aOn)
{
    const bool prev = g_override.exchange(aOn, std::memory_order_relaxed);
    const bool live = g_installed.load(std::memory_order_acquire);
    if (prev != aOn)
    {
        // Game thread (script native): re-apply the new policy to every registered component (removes the latch both ways).
        const int32_t written = ReapplyNow();
        size_t size = 0;
        {
            RegLock lock;
            size = g_reg.Size();
        }
        LogInfo("UTPPSetItemPlaneOverride(%d) hook=%d forced=%llu reapplied=%d registry=%zu", aOn ? 1 : 0, live ? 1 : 0,
                static_cast<unsigned long long>(g_forced.load(std::memory_order_relaxed)), written, size);
    }
    else if (live)
    {
        // No policy change: nothing is written; only dead entries are pruned. Rare: the script (SyncPolicy) deduplicates and calls
        // this only on a policy edge, so pruning in practice runs on edges, Reapply and Dump / DumpPlayer; between them the
        // setter detour reclaims dead entries itself when a new component would hit the cap (ReclaimInDetour).
        std::vector<RED4ext::WeakHandle<RED4ext::ISerializable>> release;
        RegLock lock;
        PruneLocked(release);
    }
    return live;
}

int32_t Reapply()
{
    return ReapplyNow();
}

namespace
{
// ---- Round 2 (A3): read-only player tree for the dumps (game thread only; script natives) ----
// The player, the items in its attachment slots and their child items (weapon attachments, cyberware parts). Items come from
// TransactionSystem.GetItemInSlot(owner, slot) called through RTTI (the vanilla query; weapon.script asks a weapon for
// AttachmentSlots.DamageMod the same way). There is no native list of an entity's occupied slots, so the fixed list below is
// tried on every owner (an unknown or empty slot returns null). Coverage limit: an item in a slot missing from this list, or a
// part that is not spawned as its own item entity (stat mods, programs), is not listed. Depth 2 (player -> item -> child).
constexpr const char* kTreeSlots[] = {
    // equipment / body / held items of the player
    "WeaponRight", "WeaponLeft", "TppHead", "Head", "Face", "Eyes", "Chest", "Torso", "Legs", "Feet", "Outfit", "Underwear",
    "UnderwearTop", "UnderwearBottom", "RightArm", "LeftArm", "ArmsCyberwareGeneralSlot", "SilverhandArm", "PersonalLink",
    "Consumable", "Inspect", "Gem", "RightShoulder", "LeftShoulder", "ItemSlotGenericRanged", "ItemSlotGenericMelee",
    "ItemSlotHandgunRight", "ItemSlotHandgunLeft", "ItemSlotKnifeRight", "ItemSlotKnifeLeft", "ItemSlotKatana", "ItemSlotHammer",
    "ItemSlotSMG", "ItemSlotTechRifle", "ItemSlotSniperRifle",
    // weapon / cyberware parts spawned as child items
    "Scope", "ScopeRail", "PowerModule", "Magazine", "DamageMod", "Laser", "Cannon", "CannonLaser", "RailGun", "NanoWiresCable",
    "NanoWiresBattery", "MantisBladesEdge", "MantisBladesRotor", "StrongArmsKnuckles", "StrongArmsBattery",
    "ProjectileLauncherRound", "ProjectileLauncherWiring"};
constexpr size_t kTreeMaxNodes = 96;        // entities listed (player + items + children)
constexpr int kTreeMaxComponents = 512;     // components read per entity
constexpr uint32_t kOffParamSkinned = 0x238; // entSkinnedMeshComponent.renderingPlaneAnimationParam (SDK; read by 0x20A240)
constexpr uint32_t kOffParamMorph = 0x2E8;   // entMorphTargetSkinnedMeshComponent.renderingPlaneAnimationParam (read by 0x209000)

struct TreeNode
{
    RED4ext::Handle<RED4ext::IScriptable> entity; // strong while the dump runs
    std::string label;                            // player | item:<slot>:<tdbid> | child:<slot>/<slot>:<tdbid>
    std::string slot;                             // slot path ("" for the player)
};

struct PlayerTree
{
    std::vector<TreeNode> nodes; // [0] = the player when found
    std::string why = "ok";      // "ok" = items walked; else why only the player (or nothing) is listed
    size_t items = 0, children = 0;

    const char* LabelOf(const void* aEntity) const
    {
        for (const TreeNode& n : nodes)
            if (n.entity.instance == aEntity)
                return n.label.c_str();
        return nullptr;
    }
};

const char* NameOr(const RED4ext::CName& aName, const char* aFallback)
{
    const char* s = aName.ToString();
    return s ? s : aFallback;
}

RED4ext::CBaseFunction* FindStaticFunction(RED4ext::CClass* aCls, const char* aShortName)
{
    if (!aCls)
        return nullptr;
    const RED4ext::CName want(aShortName);
    for (RED4ext::CClassStaticFunction* f : aCls->staticFuncs)
        if (f && f->shortName == want)
            return f;
    return nullptr;
}

// GameInstance.GetTransactionSystem(game) through RTTI.
bool GetTransactions(RED4ext::Handle<RED4ext::IScriptable>& aTs, std::string& aWhy)
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CBaseFunction* fn = rtti ? FindStaticFunction(rtti->GetClass("ScriptGameInstance"), "GetTransactionSystem") : nullptr;
    if (!fn)
    {
        aWhy = "GameInstance.GetTransactionSystem not found";
        return false;
    }
    RED4ext::ScriptGameInstance gi;
    RED4ext::StackArgs_t args;
    args.emplace_back(nullptr, &gi);
    if (!RED4ext::ExecuteFunction(static_cast<void*>(nullptr), fn, &aTs, args) || !aTs)
    {
        aWhy = "GetTransactionSystem returned nothing";
        return false;
    }
    return true;
}

uint64_t ItemTdbid(RED4ext::IScriptable* aItem)
{
    RED4ext::CClass* cls = aItem ? aItem->GetType() : nullptr;
    RED4ext::CClassFunction* fn = cls ? cls->GetFunction("GetItemID") : nullptr;
    RED4ext::ItemID id{};
    if (!fn || !RED4ext::ExecuteFunction(aItem, fn, &id))
        return 0;
    return id.tdbid.value;
}

// Fills aTree (never throws, never writes anything to the game).
void CollectPlayerTree(PlayerTree& aTree)
{
    RED4ext::ScriptGameInstance gi;
    if (!gi.instance)
    {
        aTree.why = "no game instance";
        return;
    }
    RED4ext::Handle<RED4ext::IScriptable> player;
    if (!RED4ext::ExecuteGlobalFunction("GetPlayer;GameInstance", &player, gi) || !player)
    {
        aTree.why = "no player";
        return;
    }
    aTree.nodes.push_back({player, "player", ""});
    RED4ext::Handle<RED4ext::IScriptable> ts;
    if (!GetTransactions(ts, aTree.why))
        return;
    RED4ext::CClass* tsCls = ts.instance->GetType();
    RED4ext::CClassFunction* getItemInSlot = tsCls ? tsCls->GetFunction("GetItemInSlot") : nullptr;
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* itemCls = rtti ? rtti->GetClass("gameItemObject") : nullptr;
    if (!getItemInSlot || !itemCls)
    {
        aTree.why = "TransactionSystem.GetItemInSlot / gameItemObject not found";
        return;
    }
    std::vector<RED4ext::TweakDBID> slotIds;
    slotIds.reserve(std::size(kTreeSlots));
    for (const char* s : kTreeSlots)
        slotIds.emplace_back((std::string("AttachmentSlots.") + s).c_str());

    size_t levelBegin = 0;
    for (int depth = 1; depth <= 2; ++depth)
    {
        const size_t levelEnd = aTree.nodes.size();
        for (size_t o = levelBegin; o < levelEnd; ++o)
        {
            for (size_t s = 0; s < slotIds.size(); ++s)
            {
                RED4ext::Handle<RED4ext::IScriptable> owner = aTree.nodes[o].entity; // the call's ref<GameObject> argument
                RED4ext::TweakDBID slot = slotIds[s];
                RED4ext::Handle<RED4ext::IScriptable> item;
                RED4ext::StackArgs_t args;
                args.emplace_back(nullptr, &owner);
                args.emplace_back(nullptr, &slot);
                if (!RED4ext::ExecuteFunction(ts.instance, getItemInSlot, &item, args) || !item)
                    continue;
                RED4ext::CClass* cls = item.instance->GetType();
                if (!cls || !cls->IsA(itemCls) || aTree.LabelOf(item.instance))
                    continue; // not an item object, or already listed under another slot
                if (aTree.nodes.size() >= kTreeMaxNodes)
                {
                    aTree.why = "entity cap reached";
                    return;
                }
                char tdb[32];
                std::snprintf(tdb, sizeof(tdb), "0x%016llX", static_cast<unsigned long long>(ItemTdbid(item.instance)));
                TreeNode n;
                n.entity = item;
                if (depth == 1)
                {
                    n.slot = kTreeSlots[s];
                    n.label = "item:" + n.slot + ":" + tdb;
                    ++aTree.items;
                }
                else
                {
                    n.slot = aTree.nodes[o].slot + "/" + kTreeSlots[s];
                    n.label = "child:" + n.slot + ":" + tdb;
                    ++aTree.children;
                }
                aTree.nodes.push_back(std::move(n));
            }
        }
        levelBegin = levelEnd;
    }
}

// SEH-only (no destructors): copies up to aMax (instance, reference block) pairs of entEntity::components (DynArray of
// Handle<entIComponent> at +0xA0, RED4ext.SDK entEntity.hpp). -1 = read fault, -2 = implausible size.
struct RawHandle
{
    RED4ext::ISerializable* instance;
    RED4ext::RefCnt* refCount;
};
int SehCopyComponents(const RED4ext::ent::Entity* aEntity, RawHandle* aOut, int aMax)
{
    __try
    {
        const uint32_t n = aEntity->components.Size();
        if (n > 4096)
            return -2;
        const RED4ext::Handle<RED4ext::ent::IComponent>* data = aEntity->components.Data();
        int k = 0;
        for (uint32_t i = 0; i < n && k < aMax; ++i)
        {
            aOut[k].instance = data[i].instance;
            aOut[k].refCount = data[i].refCount;
            ++k;
        }
        return k;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}

// The live components of one entity, each pinned by a strong reference (released when the vector is destroyed).
int PinComponents(const RED4ext::IScriptable* aEntity, std::vector<RED4ext::Handle<RED4ext::ISerializable>>& aOut)
{
    static RawHandle raw[kTreeMaxComponents]; // game thread only
    const int n = SehCopyComponents(reinterpret_cast<const RED4ext::ent::Entity*>(aEntity), raw, kTreeMaxComponents);
    for (int i = 0; i < n; ++i)
    {
        if (!raw[i].instance || !raw[i].refCount || !raw[i].refCount->IncRefIfNotZero())
            continue;
        aOut.emplace_back();
        aOut.back().instance = raw[i].instance; // adopts the strong reference
        aOut.back().refCount = raw[i].refCount;
    }
    return n;
}

// The registry view of one component: "<requested>/<broadcast|outside>", "none" or "noref".
std::string RegistryNote(RED4ext::ISerializable* aComponent)
{
    const RefKey key = ReadRef(aComponent);
    if (!key.instance)
        return "noref";
    core::Entry e;
    bool found = false;
    {
        RegLock lock;
        if (const core::Entry* p = g_reg.Find(key.instance, key.refCnt))
        {
            e = *p;
            found = true;
        }
    }
    if (!found)
        return "none";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%d/%s", e.requested, e.outsideLast ? "outside" : "broadcast");
    return buf;
}
} // namespace

std::string Dump(int32_t aMax, const void* aPlayer)
{
    const size_t max = aMax > 0 ? static_cast<size_t>(aMax) : 0;
    // Round 2: the owner chain of every line (player / item / child item of the player, else other). Built before the snapshot.
    PlayerTree tree;
    CollectPlayerTree(tree);
    std::vector<Pinned> live;
    std::vector<RED4ext::WeakHandle<RED4ext::ISerializable>> release;
    size_t size = 0, peak = 0;
    if (g_installed.load(std::memory_order_acquire))
        SnapshotLive(live, max, release, &size);
    release.clear();
    {
        RegLock lock;
        peak = g_reg.Peak();
    }
    char buf[640];
    std::snprintf(buf, sizeof(buf),
                  "itemplane dump: hook=%d override=%d registry=%zu shown=%zu reapplied=%llu reapplies=%llu weaponInBroadcast=%llu "
                  "forcedScene=%llu bgInBc=%llu weaponOutBc=%llu regDropped=%llu noRef=%llu pruned=%llu registryPeak=%zu "
                  "handlerQueued=%llu orphanSkipped=%llu tree=\"%s\"",
                  g_installed.load() ? 1 : 0, g_override.load() ? 1 : 0, size, live.size(),
                  static_cast<unsigned long long>(g_reapplied.load()), static_cast<unsigned long long>(g_reapplies.load()),
                  static_cast<unsigned long long>(g_weaponInBc.load()), static_cast<unsigned long long>(g_forced.load()),
                  static_cast<unsigned long long>(g_bgInBc.load()), static_cast<unsigned long long>(g_weaponOutBc.load()),
                  static_cast<unsigned long long>(g_regDropped.load()), static_cast<unsigned long long>(g_noRef.load()),
                  static_cast<unsigned long long>(g_pruned.load()), peak,
                  static_cast<unsigned long long>(g_handlerQueued.load()),
                  static_cast<unsigned long long>(g_orphanSkipped.load()), tree.why.c_str());
    std::string out = buf;
    LogInfo("%s", buf);
    for (size_t i = 0; i < live.size(); ++i)
    {
        const Pinned& p = live[i];
        const auto* base = reinterpret_cast<const uint8_t*>(p.handle.instance);
        RED4ext::CClass* cls = p.handle.instance->GetType();
        const char* clsName = cls ? cls->GetName().ToString() : nullptr;
        const char* name = reinterpret_cast<const RED4ext::CName*>(base + kOffName)->ToString();
        const void* owner = *reinterpret_cast<void* const*>(base + kOffOwner);
        const char* chain = tree.LabelOf(owner);
        if (!chain)
            chain = (aPlayer && owner == aPlayer) ? "player" : "other";
        std::snprintf(buf, sizeof(buf), "  [%zu] class=%s name=%s requested=%d last=%s current=%d ownerChain=%s", i,
                      clsName ? clsName : "?", name ? name : "?", p.entry.requested, p.entry.outsideLast ? "outside" : "broadcast",
                      PlaneOf(p.handle.instance), chain);
        out += '\n';
        out += buf;
        LogInfo("%s", buf);
    }
    return out;
}

std::string DumpPlayer(int32_t aMax)
{
    const size_t max = aMax > 0 ? static_cast<size_t>(aMax) : 0;
    PlayerTree tree;
    CollectPlayerTree(tree);
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* entityCls = rtti ? rtti->GetClass("entEntity") : nullptr;
    RED4ext::CClass* visualCls = rtti ? rtti->GetClass("entIVisualComponent") : nullptr;
    RED4ext::CClass* skinnedCls = rtti ? rtti->GetClass("entSkinnedMeshComponent") : nullptr;
    RED4ext::CClass* morphCls = rtti ? rtti->GetClass("entMorphTargetSkinnedMeshComponent") : nullptr;
    RED4ext::CClass* spawnerCls = rtti ? rtti->GetClass("entEffectSpawnerComponent") : nullptr;
    RED4ext::CClass* particlesCls = rtti ? rtti->GetClass("entParticlesComponent") : nullptr;

    std::vector<std::string> lines;
    size_t rows = 0;
    char buf[512];
    for (const TreeNode& node : tree.nodes)
    {
        RED4ext::CClass* ecls = node.entity.instance->GetType();
        if (!entityCls || !ecls || !ecls->IsA(entityCls))
        {
            std::snprintf(buf, sizeof(buf), "PE owner=%s class=%s not-an-entity", node.label.c_str(),
                          ecls ? NameOr(ecls->GetName(), "?") : "?");
            lines.emplace_back(buf);
            continue;
        }
        const auto* ent = reinterpret_cast<const RED4ext::ent::Entity*>(node.entity.instance);
        std::vector<RED4ext::Handle<RED4ext::ISerializable>> comps;
        const int n = PinComponents(node.entity.instance, comps);
        std::snprintf(buf, sizeof(buf), "PE owner=%s class=%s app=%s components=%d", node.label.c_str(),
                      NameOr(ecls->GetName(), "?"), NameOr(ent->appearanceName, "?"), n);
        lines.emplace_back(buf);
        for (const auto& h : comps)
        {
            RED4ext::CClass* cls = h.instance->GetType();
            if (!cls)
                continue;
            const bool visual = visualCls && cls->IsA(visualCls);
            const bool spawner = spawnerCls && cls->IsA(spawnerCls);
            const bool particles = particlesCls && cls->IsA(particlesCls);
            if (!visual && !spawner && !particles)
                continue;
            ++rows;
            if (rows > max)
                continue; // counted, not shown
            const auto* base = reinterpret_cast<const uint8_t*>(h.instance);
            const char* param = "-";
            if (morphCls && cls->IsA(morphCls))
                param = NameOr(*reinterpret_cast<const RED4ext::CName*>(base + kOffParamMorph), "?");
            else if (skinnedCls && cls->IsA(skinnedCls))
                param = NameOr(*reinterpret_cast<const RED4ext::CName*>(base + kOffParamSkinned), "?");
            char cur[16] = "n/a"; // +0x138 exists only on entIVisualComponent
            if (visual)
                std::snprintf(cur, sizeof(cur), "%d", PlaneOf(h.instance));
            std::snprintf(buf, sizeof(buf), "PV owner=%s class=%s name=%s param=%s cur=%s reg=%s", node.label.c_str(),
                          NameOr(cls->GetName(), "?"), NameOr(*reinterpret_cast<const RED4ext::CName*>(base + kOffName), "?"),
                          param, cur, RegistryNote(h.instance).c_str());
            lines.emplace_back(buf);
        }
    }
    const int hooks = g_handlerHooks.load(std::memory_order_acquire);
    std::snprintf(buf, sizeof(buf),
                  "itemplane player dump: hook=%d override=%d gate=%s entities=%zu items=%zu children=%zu rows=%zu shown=%zu "
                  "tree=\"%s\"",
                  g_installed.load() ? 1 : 0, g_override.load() ? 1 : 0,
                  hooks >= 2 ? "bc+handler" : (hooks == 1 ? "bc+partial" : "bc-only"), tree.nodes.size(), tree.items,
                  tree.children, rows, rows < max ? rows : max, tree.why.c_str());
    std::string out = buf;
    LogInfo("%s", buf);
    for (const std::string& l : lines)
    {
        out += '\n';
        out += l;
        LogInfo("%s", l.c_str());
    }
    return out;
}

bool OverrideOn()
{
    return g_override.load(std::memory_order_relaxed);
}

bool Installed()
{
    return g_installed.load(std::memory_order_acquire);
}

bool QueuePlaneEvent(RED4ext::IScriptable* aItem, int32_t aPlane, std::string& aWhy)
{
    if (!aItem)
    {
        aWhy = "item null";
        return false;
    }
    if (aPlane < 0 || aPlane > 2)
    {
        aWhy = "plane out of range (0 scene, 1 background, 2 weapon)";
        return false;
    }
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::CClass* evCls = rtti ? rtti->GetClass("gameItemEventsPropagateRenderingPlane") : nullptr;
    if (!evCls)
    {
        aWhy = "class gameItemEventsPropagateRenderingPlane not found";
        return false;
    }
    if (evCls->GetSize() != kEventSize)
    {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "event size 0x%X != 0x%X", evCls->GetSize(), kEventSize);
        aWhy = buf;
        return false;
    }
    RED4ext::CClass* itemCls = aItem->GetType();
    RED4ext::CClassFunction* queue = itemCls ? itemCls->GetFunction("QueueEvent") : nullptr;
    RED4ext::CClassFunction* queueChildren = itemCls ? itemCls->GetFunction("QueueEventToChildItems") : nullptr;
    if (!queue || !queueChildren)
    {
        aWhy = "QueueEvent / QueueEventToChildItems not found on the item class (not an ItemObject?)";
        return false;
    }
    // One event for the item, one for its child items / parts (as the vanilla scripts do: separate instances).
    for (int i = 0; i < 2; ++i)
    {
        void* mem = evCls->CreateInstance(true);
        if (!mem)
        {
            aWhy = "CreateInstance failed";
            return false;
        }
        RED4ext::Handle<RED4ext::IScriptable> evt(static_cast<RED4ext::IScriptable*>(mem));
        *reinterpret_cast<int32_t*>(static_cast<uint8_t*>(mem) + kEventPlaneOff) = aPlane;
        RED4ext::StackArgs_t args;
        args.emplace_back(nullptr, &evt);
        if (!RED4ext::ExecuteFunction(aItem, i == 0 ? queue : queueChildren, nullptr, args))
        {
            aWhy = i == 0 ? "QueueEvent failed" : "QueueEventToChildItems failed";
            return false;
        }
    }
    aWhy.clear();
    return true;
}

void CountEvent(int32_t aPlane, bool aOk)
{
    if (!aOk)
    {
        g_eventFails.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (aPlane == kPlaneScene)
        g_eventsScene.fetch_add(1, std::memory_order_relaxed);
    else if (aPlane == kPlaneWeapon)
        g_eventsWeapon.fetch_add(1, std::memory_order_relaxed);
    else
        g_eventsOther.fetch_add(1, std::memory_order_relaxed);
}

std::string Status()
{
    std::string why;
    {
        std::lock_guard lock(g_coldMutex);
        why = g_installWhy;
    }
    size_t size = 0, peak = 0;
    {
        RegLock lock;
        size = g_reg.Size();
        peak = g_reg.Peak();
    }
    // Existing keys keep their order; the registry keys follow eventFails; the round-2 keys (gate= handlerHooks= handlerQueued=
    // registryPeak=) follow reapplies=; install="..." stays last.
    // gate: bc+handler = both anim-param handler hooks live (NG1 closed); bc+partial = one of them (its reason is in install=);
    // bc-only = none (the 0.6.4 broadcast-depth gate alone).
    const int hooks = g_handlerHooks.load(std::memory_order_acquire);
    const char* gate = hooks >= 2 ? "bc+handler" : (hooks == 1 ? "bc+partial" : "bc-only");
    char buf[1280];
    std::snprintf(buf, sizeof(buf),
                  "itemplane hook=%d override=%d broadcastsOn=%llu weaponInBroadcast=%llu forcedScene=%llu "
                  "eventsScene=%llu eventsWeapon=%llu eventsOther=%llu eventFails=%llu registry=%zu reapplied=%llu bgInBc=%llu "
                  "weaponOutBc=%llu regDropped=%llu noRef=%llu pruned=%llu reapplies=%llu orphanSkipped=%llu gate=%s "
                  "handlerHooks=%d handlerQueued=%llu registryPeak=%zu install=\"%s\"",
                  g_installed.load() ? 1 : 0, g_override.load() ? 1 : 0,
                  static_cast<unsigned long long>(g_broadcasts.load()),
                  static_cast<unsigned long long>(g_weaponInBc.load()), static_cast<unsigned long long>(g_forced.load()),
                  static_cast<unsigned long long>(g_eventsScene.load()),
                  static_cast<unsigned long long>(g_eventsWeapon.load()),
                  static_cast<unsigned long long>(g_eventsOther.load()),
                  static_cast<unsigned long long>(g_eventFails.load()), size,
                  static_cast<unsigned long long>(g_reapplied.load()), static_cast<unsigned long long>(g_bgInBc.load()),
                  static_cast<unsigned long long>(g_weaponOutBc.load()), static_cast<unsigned long long>(g_regDropped.load()),
                  static_cast<unsigned long long>(g_noRef.load()), static_cast<unsigned long long>(g_pruned.load()),
                  static_cast<unsigned long long>(g_reapplies.load()),
                  static_cast<unsigned long long>(g_orphanSkipped.load()), gate, hooks,
                  static_cast<unsigned long long>(g_handlerQueued.load()), peak, why.c_str());
    return buf;
}
} // namespace utpp::itemplane

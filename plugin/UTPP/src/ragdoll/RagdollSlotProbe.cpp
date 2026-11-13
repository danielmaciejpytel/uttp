// UTPP ragdoll R1f probe (2026-11-03): read-only view of the animation system's per-ragdoll pose slots.
//
// Evidence (static analysis of the 2.50 exe; RVAs):
//  - 0x286EDC = per-index body of the job "Animation System - update ragdolls", called as (ctx, i, ...) with *ctx = the animation
//    system A. R = *(A + 0xA6950 + i*8) is the i-th registered entRagdollComponent; its pose slot is S = A + 0x8A940 + i*0x30.
//  - While R+0x1DB (collect request) or float R+0x1D0 > 0, the job copies S.entry1 (ptr S+0x10, cap S+0x18, count S+0x1C, 32-byte
//    elements) into R's double pose buffer (R+0xD8 / R+0xE8). The activator needs both counts > 0. The animation side fills S.entry1;
//    in game V's buffers stay empty (R1d/R1e), so this probe reads S for V and for every other registered ragdoll (NPCs).
//
// The detour only stores A and the highest index seen (relaxed atomics) and calls the original. Nothing is written to the game.
// The dump runs on the script thread and reads under SEH: a stale A or a race with the job gives at worst an "unreadable" line.
//
// Production core (2026-11-03): the same detour identifies V's slot by owner entity (component +0x50 -> +0x48 == the player's
// EntityID hash, published natively once per second or by UTPPRagdollSetPlayer), caches index + component, re-validates both every
// job tick of that index and feeds the live overlay gate into the episode state machine (RagdollEpisode.hpp). The natives of the
// episode (Begin / Release / Abort / State / Pelvis ...) are registered below; the pose writers check ragdoll::GameOwned().

#include <Windows.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/Generated/ent/Entity.hpp>
#include <RED4ext/Scripting/Natives/Quaternion.hpp>
#include <RED4ext/Scripting/Natives/Vector4.hpp>
#include <RED4ext/Scripting/Stack.hpp>

#include "../CommonUtil.hpp"
#include "../PluginLog.hpp"
#include "../MotionDriver.hpp"
#include "../State.hpp"
#include "../mm/NativeReloadContactsRuntime.hpp"
#include "RagdollGuards.hpp"
#include "RdLastPose.hpp"
#include "RagdollSlotProbe.hpp"
#include "RdPoseTable.hpp"

// The owner EntityID is read at +0x48 of the entity the component's +0x50 points to; the SDK layout agrees.
static_assert(offsetof(RED4ext::ent::Entity, entityID) == 0x48);

namespace utpp::ragdollslot
{
namespace
{
constexpr uint32_t kRvaUpdateRagdoll = 0x286EDC;
// mov rax,rsp; mov [rax+18h],r8; push rbp/rbx/rsi/rdi/r12..r15; lea rbp,[rax-5Fh]
constexpr uint8_t kPrologue[] = {0x48, 0x8B, 0xC4, 0x4C, 0x89, 0x40, 0x18, 0x55, 0x53, 0x56, 0x57, 0x41,
                                 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x68, 0xA1};
constexpr size_t kComponentTable = 0xA6950; // A + i*8 -> entRagdollComponent*
constexpr size_t kSlotTable = 0x8A940;      // A + i*0x30 -> pose slot
constexpr size_t kSlotSize = 0x30;
constexpr uint32_t kMaxIndex = 4096; // sanity cap of the scanned range
constexpr int kMaxExamples = 6;

using Update_t = uint64_t (*)(void*, uint64_t, uint64_t, uint64_t);

const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;
Update_t g_orig = nullptr;
void* g_target = nullptr;

std::atomic<uint8_t*> g_animSys{nullptr};
std::atomic<uint32_t> g_maxIndex{0};
std::atomic<uint64_t> g_calls{0};
std::atomic<int32_t> g_selfIndex{-1};
std::atomic<const void*> g_selfComp{nullptr};
std::atomic<uint64_t> g_playerHash{0};      // the player's EntityID hash (0 = unknown)
std::atomic<uint64_t> g_cacheDrops{0}, g_cacheHits{0};
std::atomic<int64_t> g_qpcHz{0};
std::atomic<int64_t> g_idleSampleAt{0};
std::atomic<int32_t> g_pelvisIdx[4] = {{-1}, {-1}, {-1}, {-1}}; // Hips, Spine, LeftUpLeg, RightUpLeg
// V's player-tagged output descriptors have different bone counts (294 / 619 / 645 in game); the largest one carries the full body and
// the ragdoll (the merged rig the plugin arms on has 619 bones, so g_hot.playerBones is NOT the count to compare with).
std::atomic<uint32_t> g_pelvisBestCount{0};

ragdoll::RdKeep g_keep;
ragdoll::PelvisTracker g_pelvis;
// Per-rig Hips / Spine / LeftUpLeg / RightUpLeg indices of the player descriptors whose rig is not the merged one (see OnPlayerFk).
ragdoll::PelvisRigCache g_pelvisRigs;

int64_t QpcHz()
{
    int64_t hz = g_qpcHz.load(std::memory_order_relaxed);
    if (hz <= 0)
    {
        LARGE_INTEGER f{};
        QueryPerformanceFrequency(&f);
        hz = f.QuadPart;
        g_qpcHz.store(hz, std::memory_order_relaxed);
        g_keep.SetMaxAge(hz / 10); // an open evaluation older than 100 ms is never written back
    }
    return hz;
}
int64_t QpcNow()
{
    LARGE_INTEGER n{};
    QueryPerformanceCounter(&n);
    return n.QuadPart;
}
double NowS() { return static_cast<double>(QpcNow()) / static_cast<double>(QpcHz()); }

std::mutex g_coldMutex;
std::string g_installWhy = "not installed (Install not called)";

using pluginlog::LogInfo;
using pluginlog::LogWarn;

// Owner EntityID hash of a ragdoll component: R+0x50 is read by the activation job as an object whose +0x48 is the lookup key
// (owner entity, +0x48 = EntityID; the in-game probe confirmed selfIdOk=1 against V's own id). 0 = unreadable.
uint64_t SehOwnerHash(const void* aComp)
{
    __try
    {
        const uint8_t* entity = nullptr;
        std::memcpy(&entity, static_cast<const uint8_t*>(aComp) + 0x50, sizeof(entity));
        if (!entity)
            return 0;
        uint64_t hash = 0;
        std::memcpy(&hash, entity + 0x48, sizeof(hash));
        return hash;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
}

// EntityID hash of an entity object (entity+0x48, static_assert above). 0 = unreadable.
uint64_t SehEntityId(const void* aEntity)
{
    __try
    {
        uint64_t hash = 0;
        std::memcpy(&hash, static_cast<const uint8_t*>(aEntity) + 0x48, sizeof(hash));
        return hash;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
}

// Component pointer registered at index aIndex (nullptr = empty / unreadable).
const void* SehComponent(const uint8_t* aSys, uint32_t aIndex)
{
    __try
    {
        const void* comp = nullptr;
        std::memcpy(&comp, aSys + kComponentTable + static_cast<size_t>(aIndex) * 8, sizeof(comp));
        return comp;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return nullptr;
    }
}

// Overlay gate (slot+0x29) of index aIndex; false with aKnown = false when unreadable (not a "gate low" reading).
bool SehGate(const uint8_t* aSys, uint32_t aIndex, bool& aKnown)
{
    __try
    {
        const bool gate = aSys[kSlotTable + static_cast<size_t>(aIndex) * kSlotSize + 0x29] != 0;
        aKnown = true;
        return gate;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aKnown = false;
        return false;
    }
}

void DropSlotCache()
{
    g_selfIndex.store(-1, std::memory_order_release);
    g_selfComp.store(nullptr, std::memory_order_relaxed);
    g_cacheDrops.fetch_add(1, std::memory_order_relaxed);
}

// Job tick of index aIndex: finds V's slot by owner entity, keeps it cached, and feeds the live gate into the episode.
void TrackPlayerSlot(const uint8_t* aSys, uint32_t aIndex)
{
    const uint64_t player = g_playerHash.load(std::memory_order_relaxed);
    if (!player || !aSys || aIndex >= kMaxIndex)
        return;
    const int32_t cached = g_selfIndex.load(std::memory_order_acquire);
    if (cached >= 0)
    {
        if (static_cast<uint32_t>(cached) != aIndex)
            return; // another ragdoll: nothing to do once V's slot is known
        const void* comp = SehComponent(aSys, aIndex);
        if (!comp || comp != g_selfComp.load(std::memory_order_relaxed) || SehOwnerHash(comp) != player)
        {
            DropSlotCache(); // the table entry changed under the cache
            return;
        }
        g_cacheHits.fetch_add(1, std::memory_order_relaxed);
        bool known = false;
        const bool gate = SehGate(aSys, aIndex, known);
        ragdoll::g_episode.Tick(gate, NowS(), known);
        return;
    }
    const void* comp = SehComponent(aSys, aIndex);
    if (!comp || SehOwnerHash(comp) != player)
        return;
    g_selfComp.store(comp, std::memory_order_relaxed);
    g_selfIndex.store(static_cast<int32_t>(aIndex), std::memory_order_release);
}

uint64_t DetourUpdate(void* aCtx, uint64_t aIndex, uint64_t aA3, uint64_t aA4)
{
    // The original dereferences aCtx first thing as well; the index is the 32-bit job index.
    uint8_t* sys = nullptr;
    if (aCtx)
    {
        sys = *static_cast<uint8_t**>(aCtx);
        g_animSys.store(sys, std::memory_order_relaxed);
    }
    const uint32_t i = static_cast<uint32_t>(aIndex);
    if (i < kMaxIndex && i > g_maxIndex.load(std::memory_order_relaxed))
        g_maxIndex.store(i, std::memory_order_relaxed);
    g_calls.fetch_add(1, std::memory_order_relaxed);
    TrackPlayerSlot(sys, i);
    return g_orig(aCtx, aIndex, aA3, aA4);
}

using util::SehBytesEqual;

struct Slot
{
    uint32_t entry0Count;
    uint32_t entry1Cap;
    uint32_t entry1Count;
    uint32_t at20;
    uint8_t at28;
    uint8_t at29;
    bool entry1Ptr;
};

bool SehReadSlot(const uint8_t* aSys, uint32_t aIndex, const void*& aComp, Slot& aSlot)
{
    __try
    {
        std::memcpy(&aComp, aSys + kComponentTable + static_cast<size_t>(aIndex) * 8, sizeof(aComp));
        const uint8_t* s = aSys + kSlotTable + static_cast<size_t>(aIndex) * kSlotSize;
        uint64_t p = 0;
        std::memcpy(&aSlot.entry0Count, s + 0x0C, 4);
        std::memcpy(&p, s + 0x10, 8);
        std::memcpy(&aSlot.entry1Cap, s + 0x18, 4);
        std::memcpy(&aSlot.entry1Count, s + 0x1C, 4);
        std::memcpy(&aSlot.at20, s + 0x20, 4);
        aSlot.at28 = s[0x28];
        aSlot.at29 = s[0x29];
        aSlot.entry1Ptr = p != 0;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

std::string FormatSlot(uint32_t aIndex, const Slot& aS)
{
    char b[160];
    snprintf(b, sizeof(b), "i=%u e0n=%u e1ptr=%d e1cap=%u e1n=%u s20=%u s28=%u s29=%u", aIndex, aS.entry0Count, aS.entry1Ptr ? 1 : 0,
             aS.entry1Cap, aS.entry1Count, aS.at20, aS.at28, aS.at29);
    return b;
}

// R1l: raw 32-byte element aBone of the slot container at aOff (0x00 = entry0 physics pose, 0x10 = entry1 model-space anim pose),
// read only when aBone < its count. The element layout is not known yet, so all 8 floats are printed.
bool SehReadElem(const uint8_t* aSys, uint32_t aIndex, size_t aOff, uint32_t aBone, float (&aOut)[8])
{
    __try
    {
        const uint8_t* s = aSys + kSlotTable + static_cast<size_t>(aIndex) * kSlotSize;
        const uint8_t* p = nullptr;
        uint32_t count = 0;
        std::memcpy(&p, s + aOff, sizeof(p));
        std::memcpy(&count, s + aOff + 0x0C, 4);
        if (!p || aBone >= count)
            return false;
        std::memcpy(aOut, p + static_cast<size_t>(aBone) * 0x20, sizeof(aOut));
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

std::string FormatElems(const uint8_t* aSys, uint32_t aIndex)
{
    static constexpr uint32_t kBones[] = {2, 22, 11}; // player rig: Hips, Head, LeftFoot (NPC rig: index 2 is also Hips)
    std::string r;
    for (const size_t off : {size_t{0x00}, size_t{0x10}})
    {
        for (const uint32_t bone : kBones)
        {
            float f[8];
            char b[200];
            if (SehReadElem(aSys, aIndex, off, bone, f))
                snprintf(b, sizeof(b), " e%d[%u]=(%.3f,%.3f,%.3f,%.3f|%.3f,%.3f,%.3f,%.3f)", off == 0 ? 0 : 1, bone, f[0], f[1], f[2], f[3],
                         f[4], f[5], f[6], f[7]);
            else
                snprintf(b, sizeof(b), " e%d[%u]=n/a", off == 0 ? 0 : 1, bone);
            r += b;
        }
    }
    return r;
}

// One status line: episode state, keep counters (saves : restores 1 : n per frame, faults 0), thread instrumentation (threadDiff 0 =
// the merges of one descriptor stay on one thread), pelvis, speed, face-up. Game thread.
void LogKeepStatus(const char* aTag)
{
    const auto c = g_keep.Snapshot();
    ragdoll::PelvisSnapshot p;
    const bool pv = g_pelvis.Read(p, QpcNow(), QpcHz());
    LogInfo("[utpp_ragdoll] %s state=%d owner=%d gate=%d slot=%d saves=%llu restores=%llu skips=%llu (busy=%llu stale=%llu serial=%llu "
            "none=%llu poison=%llu full=%llu) faults=%llu bones=%u threadSame=%llu threadDiff=%llu saveTid=%u restoreTid=%u "
            "cacheHits=%llu cacheDrops=%llu pelvis=%s(%.3f,%.3f,%.3f) speed=%.2f faceUp=%d",
            aTag, static_cast<int>(ragdoll::g_episode.State()), ragdoll::GameOwned() ? 1 : 0, PlayerOverlayActive() ? 1 : 0,
            g_selfIndex.load(std::memory_order_relaxed), static_cast<unsigned long long>(c.saves),
            static_cast<unsigned long long>(c.restores), static_cast<unsigned long long>(c.skips),
            static_cast<unsigned long long>(c.skipBusy), static_cast<unsigned long long>(c.skipStale),
            static_cast<unsigned long long>(c.skipSerial), static_cast<unsigned long long>(c.skipNoSlot),
            static_cast<unsigned long long>(c.skipPoison), static_cast<unsigned long long>(c.skipFull),
            static_cast<unsigned long long>(c.faults), c.lastBones, static_cast<unsigned long long>(c.threadSame),
            static_cast<unsigned long long>(c.threadDiff), c.lastSaveThread, c.lastRestoreThread,
            static_cast<unsigned long long>(g_cacheHits.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_cacheDrops.load(std::memory_order_relaxed)), pv ? "ok" : "stale",
            pv ? static_cast<double>(p.pos[0]) : 0.0, pv ? static_cast<double>(p.pos[1]) : 0.0,
            pv ? static_cast<double>(p.pos[2]) : 0.0, pv ? static_cast<double>(p.speed) : 0.0, pv ? g_pelvis.FaceUp(p) : -1);
}

// Ragdoll R1o: one log line per filled row of the map / fk tables (RdPoseTable.hpp), written here so the 480-character
// UTPPLogLine limit does not cut them. Game thread; the rows are atomics, a row can mix two frames.
void LogPoseTables()
{
    LogKeepStatus("rdkeep");
    using utpp::rdpose::Row;
    auto f4 = [](const std::atomic<float> (&aV)[4], char (&aOut)[64]) {
        snprintf(aOut, sizeof(aOut), "(%.3f,%.3f,%.3f,%.3f)", aV[0].load(std::memory_order_relaxed),
                 aV[1].load(std::memory_order_relaxed), aV[2].load(std::memory_order_relaxed), aV[3].load(std::memory_order_relaxed));
    };
    char a[64], b[64], c[64], d[64], e[64];
    for (int i = 0; i < utpp::rdpose::kRows; ++i)
    {
        const Row& r = utpp::rdpose::g_map[i];
        const uint64_t key = r.key.load(std::memory_order_relaxed);
        if (!key)
            continue;
        f4(r.v[0], a);
        f4(r.v[1], b);
        f4(r.v[2], c);
        f4(r.v[3], d);
        f4(r.pre, e);
        LogInfo("[utpp_ragdoll] rdmap %d seq=%llu rig=%llX srcBones=%u desc=%llX out=%u graph=%llX calls=%llu hipsPre=%s hipsPos=%s "
                "hipsRot=%s headRot=%s lfootRot=%s g18=%llX g30=%llX g25a0=%llX g25b1=%llu g25c8=%llX hash=%llX", i,
                static_cast<unsigned long long>(r.seq.load()), static_cast<unsigned long long>(key),
                r.count.load(), static_cast<unsigned long long>(r.a.load()), r.aux.load(), static_cast<unsigned long long>(r.b.load()),
                static_cast<unsigned long long>(r.calls.load()), e, a, b, c, d, static_cast<unsigned long long>(r.f[0].load()),
                static_cast<unsigned long long>(r.f[1].load()), static_cast<unsigned long long>(r.f[2].load()),
                static_cast<unsigned long long>(r.f[3].load()), static_cast<unsigned long long>(r.f[4].load()),
                static_cast<unsigned long long>(r.f[5].load()));
    }
    for (int i = 0; i < utpp::rdpose::kRows; ++i)
    {
        const Row& r = utpp::rdpose::g_fk[i];
        const uint64_t key = r.key.load(std::memory_order_relaxed);
        if (!key)
            continue;
        f4(r.v[0], a);
        f4(r.v[1], b);
        f4(r.v[2], c);
        f4(r.pre, e);
        LogInfo("[utpp_ragdoll] rdfk %d seq=%llu desc=%llX rig=%llX tag=%llX count=%u matched=%u calls=%llu hipsPre=%s hips=%s head=%s "
                "lfoot=%s", i, static_cast<unsigned long long>(r.seq.load()), static_cast<unsigned long long>(key), static_cast<unsigned long long>(r.a.load()),
                static_cast<unsigned long long>(r.b.load()), r.count.load(), r.aux.load(),
                static_cast<unsigned long long>(r.calls.load()), e, a, b, c);
    }
}

std::string Dump(const void* aComp)
{
    if (!g_orig)
        return "rslot " + g_installWhy;
    uint8_t* sys = g_animSys.load(std::memory_order_relaxed);
    const uint32_t maxI = g_maxIndex.load(std::memory_order_relaxed);
    char head[128];
    snprintf(head, sizeof(head), "rslot calls=%llu max=%u", static_cast<unsigned long long>(g_calls.load(std::memory_order_relaxed)),
             maxI);
    std::string s(head);
    if (!sys)
        return s + " (job not seen yet)";
    std::string mine = " self=not-registered";
    std::string examples;
    int filled = 0, examplesShown = 0, unreadable = 0;
    for (uint32_t i = 0; i <= maxI; ++i)
    {
        const void* comp = nullptr;
        Slot slot{};
        if (!SehReadSlot(sys, i, comp, slot))
        {
            ++unreadable;
            continue;
        }
        if (slot.entry1Count > 0)
            ++filled;
        // Research probe: it reports any component it is handed, but only the KNOWN player's component may become the production
        // cache (it feeds the episode gate); an NPC's ragdoll must never turn into "V's gate".
        if (aComp && comp == aComp && ragdoll::CanBindSlotCache(g_playerHash.load(std::memory_order_relaxed), SehOwnerHash(comp)))
        {
            g_selfComp.store(comp, std::memory_order_relaxed);
            g_selfIndex.store(static_cast<int32_t>(i), std::memory_order_release);
        }
        if (aComp && comp == aComp)
            mine = " self{" + FormatSlot(i, slot) + FormatElems(sys, i) + "}";
        else if (comp && slot.entry1Count > 0 && examplesShown < kMaxExamples)
        {
            examples += " {" + FormatSlot(i, slot) + FormatElems(sys, i) + "}";
            ++examplesShown;
        }
    }
    char sum[96];
    snprintf(sum, sizeof(sum), " filled=%d/%u unreadable=%d", filled, maxI + 1, unreadable);
    LogPoseTables();
    return s + sum + mine + " others:" + (examples.empty() ? std::string(" none") : examples);
}

// aOther < 0: owner of aComp; otherwise owner of the aOther-th registered ragdoll component other than aComp (0 = none).
uint64_t EntityHash(const void* aComp, int32_t aOther)
{
    if (aOther < 0)
        return aComp ? SehOwnerHash(aComp) : 0;
    uint8_t* sys = g_animSys.load(std::memory_order_relaxed);
    if (!g_orig || !sys)
        return 0;
    const uint32_t maxI = g_maxIndex.load(std::memory_order_relaxed);
    int32_t seen = 0;
    for (uint32_t i = 0; i <= maxI; ++i)
    {
        const void* comp = nullptr;
        Slot slot{};
        if (!SehReadSlot(sys, i, comp, slot) || !comp || comp == aComp)
            continue;
        if (seen++ == aOther)
            return SehOwnerHash(comp);
    }
    return 0;
}

void SetPlayerHash(uint64_t aHash)
{
    if (g_playerHash.exchange(aHash, std::memory_order_acq_rel) == aHash)
        return;
    // A new session / a different player: nothing cached for the old one may survive.
    DropSlotCache();
    ragdoll::g_episode.Abort(NowS());
    g_keep.InvalidateAll();
    ragdoll::LastPose().Clear();
    LogInfo("[utpp_ragdoll] player entity id changed -> slot cache dropped, episode aborted");
}

void Native_EntityHash(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, uint64_t* aOut, int64_t)
{
    RED4ext::Handle<RED4ext::IScriptable> comp;
    int32_t other = -1;
    RED4ext::GetParameter(aFrame, &comp);
    RED4ext::GetParameter(aFrame, &other);
    aFrame->code++;
    const uint64_t h = EntityHash(comp ? comp.instance : nullptr, other);
    if (aOut)
        *aOut = h;
}

void Native_SlotDump(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    RED4ext::Handle<RED4ext::IScriptable> comp;
    RED4ext::GetParameter(aFrame, &comp);
    aFrame->code++;
    try // natives run inside the game's script VM: no exception may leave them
    {
        const std::string s = Dump(comp ? comp.instance : nullptr);
        if (aOut)
            *aOut = RED4ext::CString(s.c_str());
    }
    catch (...)
    {
    }
}
// ---- production natives (signatures: RagdollEpisode.hpp header comment) ----
void Native_SetPlayer(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    uint64_t hash = 0;
    RED4ext::GetParameter(aFrame, &hash);
    aFrame->code++;
    // The DLL's own read of the player entity is authoritative: a script value only counts while none is known, a different one
    // is logged and ignored (two writers must never flip the id and abort the episode every second).
    const uint64_t known = g_playerHash.load(std::memory_order_relaxed);
    if (known == 0)
        SetPlayerHash(hash);
    else if (known != hash)
        LogWarn("[utpp_ragdoll] UTPPRagdollSetPlayer(%llX) ignored: the native player id is %llX", static_cast<unsigned long long>(hash),
                static_cast<unsigned long long>(known));
}

void Native_Begin(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    // Refused without the hook, without a published player id or while UTPP is not armed on the player tracks (the keep store could not act).
    const bool allowed = g_orig != nullptr && g_playerHash.load(std::memory_order_relaxed) != 0 &&
                         utpp::g_hot.armed.load(std::memory_order_acquire);
    if (ragdoll::GetUpOwns())
        motiondriver::GetUpAbort("episode-begin"); // a new ragdoll while the get-up runs: the get-up ends NOW, not on the next Step (no mixed pose)
    const bool ok = ragdoll::g_episode.Begin(NowS(), allowed);
    if (ok)
        ragdoll::LastPose().Clear(); // every episode starts with an empty last-pose store (get-up)
    if (ok)
        LogInfo("[utpp_ragdoll] episode begin (state=%d)", static_cast<int>(ragdoll::g_episode.State()));
    else
        LogWarn("[utpp_ragdoll] episode begin refused (hook=%d player=%d armed=%d)", g_orig ? 1 : 0,
                g_playerHash.load(std::memory_order_relaxed) ? 1 : 0, utpp::g_hot.armed.load(std::memory_order_acquire) ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_Release(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    ragdoll::g_episode.Release(NowS());
    LogInfo("[utpp_ragdoll] episode release (state=%d)", static_cast<int>(ragdoll::g_episode.State()));
}

void Native_Abort(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    ragdoll::g_episode.Abort(NowS());
    g_keep.InvalidateAll();
    motiondriver::GetUpAbort("script-abort"); // a running native get-up ends too (no-op when idle)
    ragdoll::LastPose().Clear();
    LogInfo("[utpp_ragdoll] episode abort");
}

void Native_State(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = static_cast<int32_t>(ragdoll::g_episode.State());
}

void Native_Pelvis(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    aFrame->code++;
    ragdoll::PelvisSnapshot p;
    const bool ok = g_pelvis.Read(p, QpcNow(), QpcHz());
    if (aOut)
    {
        aOut->X = ok ? p.pos[0] : 0.0f;
        aOut->Y = ok ? p.pos[1] : 0.0f;
        aOut->Z = ok ? p.pos[2] : 0.0f;
        aOut->W = ok ? 1.0f : 0.0f;
    }
}

void Native_PelvisSpeed(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    aFrame->code++;
    ragdoll::PelvisSnapshot p;
    if (aOut)
        *aOut = g_pelvis.Read(p, QpcNow(), QpcHz()) ? p.speed : 0.0f;
}

// World pelvis speed / settle: the entity root follows the ragdoll horizontally (in-game 2026-11-03), so the speed is taken on
// entity position + Hips model position. Script threads, serialised by one mutex.
std::mutex g_settleMutex;
ragdoll::SmoothSpeed g_worldSpeed;
ragdoll::SettleDetector g_settle;
float g_lastWorldSpeed = 0.0f;
uint64_t g_settleEpisodes = 0;

// Steps the filter with the entity transform: world pelvis = entityPos + entityRot * Hips model position. false = no running episode /
// no fresh pelvis sample (speed 0 / last value).
bool StepWorldSpeed(const RED4ext::Vector4& aEntity, const RED4ext::Quaternion& aRot, float& aSpeed)
{
    std::lock_guard lock(g_settleMutex);
    const auto counters = ragdoll::g_episode.Counters();
    if (ragdoll::g_episode.State() == ragdoll::EpState::Idle)
    {
        g_worldSpeed.Reset();
        g_settle.Reset();
        g_lastWorldSpeed = 0.0f;
        aSpeed = 0.0f;
        return false;
    }
    if (counters.episodes != g_settleEpisodes) // a new episode: forget the old filter and hold timer
    {
        g_settleEpisodes = counters.episodes;
        g_worldSpeed.Reset();
        g_settle.Reset();
        g_settle.Begin(ragdoll::g_episode.StartedAt()); // the minimum / maximum clock runs from the episode start, not the first call
    }
    ragdoll::PelvisSnapshot p;
    if (!g_pelvis.Read(p, QpcNow(), QpcHz()))
    {
        aSpeed = g_lastWorldSpeed;
        return false;
    }
    // v' = v + 2r(u x v) + 2 u x (u x v), u = (i, j, k); a zero quaternion (script passed none) counts as identity.
    float qi = aRot.i, qj = aRot.j, qk = aRot.k, qr = aRot.r;
    if (qi == 0.0f && qj == 0.0f && qk == 0.0f && qr == 0.0f)
        qr = 1.0f;
    const float vx = p.pos[0], vy = p.pos[1], vz = p.pos[2];
    const float cx = qj * vz - qk * vy, cy = qk * vx - qi * vz, cz = qi * vy - qj * vx;
    const float dx = qj * cz - qk * cy, dy = qk * cx - qi * cz, dz = qi * cy - qj * cx;
    const float w[3] = {aEntity.X + vx + 2.0f * (qr * cx + dx), aEntity.Y + vy + 2.0f * (qr * cy + dy),
                        aEntity.Z + vz + 2.0f * (qr * cz + dz)};
    g_lastWorldSpeed = g_worldSpeed.Step(w, NowS());
    aSpeed = g_lastWorldSpeed;
    return true;
}

void Native_WorldSpeed(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    RED4ext::Vector4 entity{};
    RED4ext::Quaternion rot{};
    RED4ext::GetParameter(aFrame, &entity);
    RED4ext::GetParameter(aFrame, &rot);
    aFrame->code++;
    float speed = 0.0f;
    StepWorldSpeed(entity, rot, speed);
    if (aOut)
        *aOut = speed;
}

void Native_Settled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::Vector4 entity{};
    RED4ext::Quaternion rot{};
    RED4ext::GetParameter(aFrame, &entity);
    RED4ext::GetParameter(aFrame, &rot);
    aFrame->code++;
    float speed = 0.0f;
    bool settled = false;
    if (StepWorldSpeed(entity, rot, speed))
    {
        std::lock_guard lock(g_settleMutex);
        settled = g_settle.Update(NowS(), speed);
    }
    if (aOut)
        *aOut = settled;
}

void Native_FaceUp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    ragdoll::PelvisSnapshot p;
    if (aOut)
        *aOut = g_pelvis.Read(p, QpcNow(), QpcHz()) ? g_pelvis.FaceUp(p) : -1;
}
} // namespace

namespace
{
// The gate of V's cached slot. aKnown = false: no reading (no player id, no cached slot, the table entry changed, fault); the result
// is then false. A missing reading must not be taken as "the gate fell" by the episode's lockout (RagdollEpisode.hpp Tick).
bool ReadPlayerOverlay(bool& aKnown)
{
    aKnown = false;
    const uint8_t* sys = g_animSys.load(std::memory_order_relaxed);
    const int32_t i = g_selfIndex.load(std::memory_order_acquire);
    const void* self = g_selfComp.load(std::memory_order_relaxed);
    if (!sys || i < 0 || !self || g_playerHash.load(std::memory_order_relaxed) == 0)
        return false;
    __try
    {
        const void* comp = nullptr;
        std::memcpy(&comp, sys + kComponentTable + static_cast<size_t>(i) * 8, sizeof(comp));
        if (comp != self)
            return false;
        aKnown = true;
        return sys[kSlotTable + static_cast<size_t>(i) * kSlotSize + 0x29] != 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        aKnown = false;
        return false;
    }
}
} // namespace

bool PlayerOverlayActive()
{
    bool known = false;
    return ReadPlayerOverlay(known);
}

ragdoll::RdKeep& Keep() { return g_keep; }
ragdoll::PelvisTracker& Pelvis() { return g_pelvis; }

void PublishPlayerEntity(const void* aEntity)
{
    SetPlayerHash(aEntity ? SehEntityId(aEntity) : 0);
}

void OnDisarm()
{
    g_pelvisBestCount.store(0, std::memory_order_relaxed);
    g_pelvisRigs.Clear();
    ragdoll::g_episode.Abort(NowS());
    g_keep.InvalidateAll();
    ragdoll::LastPose().Clear();
    DropSlotCache();
}

void OnFrame()
{
    static bool s_wasOwned = false;
    static double s_lastLog = 0.0;
    const double now = NowS();
    if (g_orig)
    {
        bool known = false;
        const bool gate = ReadPlayerOverlay(known);
        ragdoll::g_episode.Tick(gate, now, known);
        if (ragdoll::GetUpOwns() && ragdoll::g_episode.Owns())
            motiondriver::GetUpAbort("episode-restart"); // the gate rose again without a Begin (a re-hit): same rule
    }
    const bool owned = ragdoll::GameOwned();
    if (owned != s_wasOwned)
    {
        s_wasOwned = owned;
        if (owned)
        {
            // Same reason as a perspective denial: an old reload correction must not be replayed when the episode ends.
            utpp::mm::reload_contacts_runtime::ResetContinuityOnDenial();
            s_lastLog = 0.0;
        }
        else
            g_keep.InvalidateAll(); // the episode is over: nothing may be restored any more
        LogInfo("[utpp_ragdoll] episode owner %s", owned ? "on" : "off");
    }
    if (ragdoll::g_episode.State() != ragdoll::EpState::Idle && now - s_lastLog >= 1.0)
    {
        s_lastLog = now;
        LogKeepStatus("episode");
    }
}

void SetPelvisBones(int32_t aHips, int32_t aSpine, int32_t aLeftUpLeg, int32_t aRightUpLeg)
{
    g_pelvisBestCount.store(0, std::memory_order_relaxed); // a new arm: learn the full-body descriptor again
    g_pelvisRigs.Clear();
    g_pelvisIdx[0].store(-1, std::memory_order_relaxed); // gate first
    g_pelvisIdx[1].store(aSpine, std::memory_order_relaxed);
    g_pelvisIdx[2].store(aLeftUpLeg, std::memory_order_relaxed);
    g_pelvisIdx[3].store(aRightUpLeg, std::memory_order_relaxed);
    g_pelvisIdx[0].store(aHips, std::memory_order_relaxed);
}

namespace
{
// Pelvis telemetry reads Hips / Spine / LeftUpLeg / RightUpLeg of the player's full output descriptor by bone index. V has several
// player-tagged descriptors with different rigs (619 / 645 bones in game), so the indices resolved on the merged rig (SetPelvisBones)
// are used only for that rig; any other rig is resolved by bone name once (g_pelvisRigs) and a rig that cannot be resolved is not
// sampled. All game-memory reads are POD-only SEH stages; false = not the player's descriptor / unreadable.
__declspec(noinline) bool SehPelvisCandidate(const uint8_t* aDesc, void* aRig, uint32_t& aCount, const uint8_t*& aBones)
{
    __try
    {
        const uint32_t count = *reinterpret_cast<const uint32_t*>(aDesc + utpp::kDescCountOff);
        if (count == 0)
            return false;
        // The full-body descriptor is the largest player-tagged one; a smaller one never samples once a larger one was seen.
        if (count < g_pelvisBestCount.load(std::memory_order_relaxed))
            return false;
        const uint64_t tag = *reinterpret_cast<const uint64_t*>(aDesc + utpp::kDescTagOff);
        const uint64_t ta = utpp::g_hot.trackA.load(std::memory_order_relaxed);
        const uint64_t tb = utpp::g_hot.trackB.load(std::memory_order_relaxed);
        const uint64_t pr = utpp::g_hot.playerRig.load(std::memory_order_relaxed);
        if (!((tag != 0 && (tag == ta || tag == tb)) || (pr != 0 && reinterpret_cast<uint64_t>(aRig) == pr)))
            return false;
        const uint8_t* bones = *reinterpret_cast<uint8_t* const*>(aDesc + utpp::kDescBonesOff);
        if (!bones)
            return false;
        aCount = count;
        aBones = bones;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Bone-name table of a rig object whose DynArray<CName> sits at aOffset (MetaRig 0x20, anim::Rig 0x50; the layout of the rig FK
// receives is probed, the one whose name count equals the descriptor's bone count wins).
__declspec(noinline) bool SehResolveRigNames(const void* aRig, size_t aOffset, uint32_t aCount, const uint64_t (&aWanted)[4], int32_t (&aOut)[4])
{
    __try
    {
        const uint8_t* base = static_cast<const uint8_t*>(aRig);
        const uint64_t* names = *reinterpret_cast<const uint64_t* const*>(base + aOffset);
        const uint32_t n = *reinterpret_cast<const uint32_t*>(base + aOffset + 8);
        if (!names || n != aCount)
            return false;
        return ragdoll::ResolvePelvisIndices(names, n, aWanted, aOut);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

__declspec(noinline) bool SehCopyPelvisBones(const uint8_t* aBones, uint32_t aCount, const int32_t (&aIdx)[4], float (&aOut)[4][3])
{
    __try
    {
        for (int i = 0; i < 4; ++i)
        {
            const int32_t k = aIdx[i];
            if (k < 0 || static_cast<uint32_t>(k) >= aCount)
                return false;
            const float* b = reinterpret_cast<const float*>(aBones + static_cast<size_t>(k) * utpp::kBoneStride);
            aOut[i][0] = b[0];
            aOut[i][1] = b[1];
            aOut[i][2] = b[2];
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Positions (model space, after FK) of Hips / Spine / LeftUpLeg / RightUpLeg of descriptor aDesc when it is the player's full output.
bool ReadPelvisBones(const uint8_t* aDesc, void* aRig, float (&aOut)[4][3])
{
    uint32_t count = 0;
    const uint8_t* bones = nullptr;
    if (!SehPelvisCandidate(aDesc, aRig, count, bones))
        return false;
    int32_t idx[4];
    const uint64_t pr = utpp::g_hot.playerRig.load(std::memory_order_relaxed);
    if (pr != 0 && reinterpret_cast<uint64_t>(aRig) == pr)
    {
        for (int i = 0; i < 4; ++i)
            idx[i] = g_pelvisIdx[i].load(std::memory_order_relaxed); // resolved by name on this very rig at arm time
    }
    else
    {
        static const uint64_t kWanted[4] = {RED4ext::CName("Hips").hash, RED4ext::CName("Spine").hash,
                                            RED4ext::CName("LeftUpLeg").hash, RED4ext::CName("RightUpLeg").hash};
        const auto resolve = [&](uint64_t aRigKey, uint32_t aCount, int32_t (&aIdx)[4]) {
            const void* rig = reinterpret_cast<const void*>(aRigKey);
            return SehResolveRigNames(rig, 0x20, aCount, kWanted, aIdx) || SehResolveRigNames(rig, 0x50, aCount, kWanted, aIdx);
        };
        if (g_pelvisRigs.Lookup(reinterpret_cast<uint64_t>(aRig), count, resolve, idx) != ragdoll::PelvisRigCache::Result::Ok)
            return false;
    }
    if (!SehCopyPelvisBones(bones, count, idx, aOut))
        return false;
    // Raise "best" only for a descriptor that really resolved: an unresolvable larger one must not silence the sampling of the rest.
    if (count > g_pelvisBestCount.load(std::memory_order_relaxed))
        g_pelvisBestCount.store(count, std::memory_order_relaxed);
    return true;
}
} // namespace

void OnPlayerFk(const uint8_t* aDesc, void* aRig, int64_t aTicks)
{
    if (!aDesc || g_pelvisIdx[0].load(std::memory_order_relaxed) < 0)
        return;
    const bool owned = ragdoll::GameOwned();
    const bool getUp = ragdoll::GetUpOwns(); // the get-up lies on the ground: its poses must not re-sign the standing reference
    const int64_t hz = QpcHz();
    if (!owned)
    {
        // Idle: a standing-pose sample 4 times a second (reference for the belly normal sign, and a fresh pelvis for the natives).
        const int64_t last = g_idleSampleAt.load(std::memory_order_relaxed);
        if (aTicks - last < hz / 4)
            return;
    }
    float b[4][3];
    if (!ReadPelvisBones(aDesc, aRig, b))
        return;
    if (!owned)
    {
        g_idleSampleAt.store(aTicks, std::memory_order_relaxed);
        if (!getUp)
            g_pelvis.SetReference(b[0], b[1], b[2], b[3]);
    }
    g_pelvis.Feed(b[0], b[1], b[2], b[3], aTicks, hz);
}

void RegisterNatives()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const RED4ext::CBaseFunction::Flags flags = {.isNative = true, .isStatic = true};
    auto* f0 = RED4ext::CGlobalFunction::Create("UTPPRagdollSlotDump", "UTPPRagdollSlotDump", &Native_SlotDump);
    f0->flags = flags;
    f0->AddParam("handle:IComponent", "comp");
    f0->SetReturnType("String");
    rtti->RegisterFunction(f0);
    auto* f1 = RED4ext::CGlobalFunction::Create("UTPPRagdollEntityHash", "UTPPRagdollEntityHash", &Native_EntityHash);
    f1->flags = flags;
    f1->AddParam("handle:IComponent", "comp");
    f1->AddParam("Int32", "other");
    f1->SetReturnType("Uint64");
    rtti->RegisterFunction(f1);

    auto* p0 = RED4ext::CGlobalFunction::Create("UTPPRagdollSetPlayer", "UTPPRagdollSetPlayer", &Native_SetPlayer);
    p0->flags = flags;
    p0->AddParam("Uint64", "entityIdHash");
    rtti->RegisterFunction(p0);
    auto* p1 = RED4ext::CGlobalFunction::Create("UTPPRagdollBegin", "UTPPRagdollBegin", &Native_Begin);
    p1->flags = flags;
    p1->SetReturnType("Bool");
    rtti->RegisterFunction(p1);
    auto* p2 = RED4ext::CGlobalFunction::Create("UTPPRagdollRelease", "UTPPRagdollRelease", &Native_Release);
    p2->flags = flags;
    rtti->RegisterFunction(p2);
    auto* p3 = RED4ext::CGlobalFunction::Create("UTPPRagdollAbort", "UTPPRagdollAbort", &Native_Abort);
    p3->flags = flags;
    rtti->RegisterFunction(p3);
    auto* p4 = RED4ext::CGlobalFunction::Create("UTPPRagdollState", "UTPPRagdollState", &Native_State);
    p4->flags = flags;
    p4->SetReturnType("Int32");
    rtti->RegisterFunction(p4);
    auto* p5 = RED4ext::CGlobalFunction::Create("UTPPRagdollPelvis", "UTPPRagdollPelvis", &Native_Pelvis);
    p5->flags = flags;
    p5->SetReturnType("Vector4");
    rtti->RegisterFunction(p5);
    auto* p6 = RED4ext::CGlobalFunction::Create("UTPPRagdollPelvisSpeed", "UTPPRagdollPelvisSpeed", &Native_PelvisSpeed);
    p6->flags = flags;
    p6->SetReturnType("Float");
    rtti->RegisterFunction(p6);
    auto* p7 = RED4ext::CGlobalFunction::Create("UTPPRagdollFaceUp", "UTPPRagdollFaceUp", &Native_FaceUp);
    p7->flags = flags;
    p7->SetReturnType("Int32");
    rtti->RegisterFunction(p7);
    auto* p8 = RED4ext::CGlobalFunction::Create("UTPPRagdollWorldSpeed", "UTPPRagdollWorldSpeed", &Native_WorldSpeed);
    p8->flags = flags;
    p8->AddParam("Vector4", "entityPos");
    p8->AddParam("Quaternion", "entityRot");
    p8->SetReturnType("Float");
    rtti->RegisterFunction(p8);
    auto* p9 = RED4ext::CGlobalFunction::Create("UTPPRagdollSettled", "UTPPRagdollSettled", &Native_Settled);
    p9->flags = flags;
    p9->AddParam("Vector4", "entityPos");
    p9->AddParam("Quaternion", "entityRot");
    p9->SetReturnType("Bool");
    rtti->RegisterFunction(p9);
}

void Install(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    std::lock_guard lock(g_coldMutex);
    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !g_sdk || !g_sdk->hooking)
    {
        g_installWhy = "not installed (no image base / hooking API)";
        LogWarn("UTPP ragdollslot: %s", g_installWhy.c_str());
        return;
    }
    uint8_t* target = base + kRvaUpdateRagdoll;
    if (!SehBytesEqual(target, kPrologue, sizeof(kPrologue)))
    {
        g_installWhy = "not installed (update-ragdolls prologue mismatch @0x286EDC)";
        LogWarn("UTPP ragdollslot: %s", g_installWhy.c_str());
        return;
    }
    if (!g_sdk->hooking->Attach(g_handle, target, reinterpret_cast<void*>(&DetourUpdate), reinterpret_cast<void**>(&g_orig)) ||
        !g_orig)
    {
        g_orig = nullptr;
        g_installWhy = "not installed (hooking->Attach failed @0x286EDC)";
        LogWarn("UTPP ragdollslot: %s", g_installWhy.c_str());
        return;
    }
    g_target = target;
    g_installWhy = "installed";
    LogInfo("UTPP ragdollslot: read-only hook attached (update ragdolls job body @%p)", static_cast<void*>(target));
}

void Shutdown()
{
    std::lock_guard lock(g_coldMutex);
    if (g_sdk && g_sdk->hooking && g_target)
        LogInfo("UTPP ragdollslot unload: detach %s", g_sdk->hooking->Detach(g_handle, g_target) ? "ok" : "FAILED");
    g_target = nullptr;
}
} // namespace utpp::ragdollslot

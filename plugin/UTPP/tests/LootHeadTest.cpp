// Offline test for Phase 5N step 5 (LootRange + HeadGuard pure logic). No game, no RED4ext.
//   utpp_loothead_test            -> exit 0 when every check passes
//   utpp_loothead_test --negative -> runs a deliberately wrong expectation and must FAIL (proves the harness detects)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "../src/render/HeadGuardCore.hpp"
#include "../src/world/InteractRangeCore.hpp"
#include "../src/world/LootRangeCore.hpp"

namespace
{
int g_pass = 0;
int g_fail = 0;

void Check(bool aOk, const char* aName, const std::string& aDetail = {})
{
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
}

using namespace utpp;

// ---------------------------------------------------------------------------------------------
// Fake predicate objects + memory with a write log.
constexpr uint64_t kVtLookAt = 0x1400100000ull;
constexpr uint64_t kVtWindow = 0x1400200000ull;
constexpr uint64_t kVtOther = 0x1400300000ull;

struct alignas(16) FakeObj
{
    uint64_t q[8] = {}; // 0x40 bytes
};

struct WriteRec
{
    void* obj;
    uint32_t off;
    uint64_t value;
};

struct FakeMem final : loot::IObjectMemory
{
    std::vector<WriteRec> log;
    bool failWrites = false;
    bool Read(void* aObj, uint32_t aOff, uint64_t& aOut) override
    {
        std::memcpy(&aOut, static_cast<uint8_t*>(aObj) + aOff, 8);
        return true;
    }
    bool Write(void* aObj, uint32_t aOff, uint64_t aValue) override
    {
        if (failWrites)
            return false;
        std::memcpy(static_cast<uint8_t*>(aObj) + aOff, &aValue, 8);
        log.push_back({aObj, aOff, aValue});
        return true;
    }
};

// LookAt payload as in the container descriptor: testTarget Entity(0), testType Interaction(1), stopOnTransparent 1.
FakeObj MakeLookAt()
{
    FakeObj o;
    o.q[0] = kVtLookAt;
    o.q[6] = 0x0000000100000000ull; // 0x30 testTarget=0, 0x34 testType=1
    o.q[7] = 0x0000000000000001ull; // 0x38 stopOnTransparent=1, padding 0
    return o;
}

float LoF(uint64_t q)
{
    float f;
    uint32_t u = static_cast<uint32_t>(q);
    std::memcpy(&f, &u, 4);
    return f;
}
float HiF(uint64_t q)
{
    float f;
    uint32_t u = static_cast<uint32_t>(q >> 32);
    std::memcpy(&f, &u, 4);
    return f;
}

// Replays a write log on a copy and checks after every write: a reader that dispatches through the current vtable
// sees, for LookAt, exactly the original payload; for the window class, either the configured window or a payload
// whose floats are all < 1e-30 (empty window). Mixed qwords (new q30 with old q38) are allowed for the window class
// only when both halves are "safe" (configured or tiny).
bool InvariantHolds(const FakeObj& aStart, const std::vector<WriteRec>& aLog, const loot::WindowParams& aW, std::string& aWhy)
{
    FakeObj o = aStart;
    const uint64_t origQ30 = aStart.q[0] == kVtLookAt ? aStart.q[6] : 0x0000000100000000ull;
    const uint64_t origQ38 = aStart.q[0] == kVtLookAt ? aStart.q[7] : 0x1ull;
    auto tinyOrCfg = [&](float v, float cfg) { return std::fabs(v) < 1e-30f || v == cfg; };
    for (size_t i = 0; i < aLog.size(); ++i)
    {
        std::memcpy(reinterpret_cast<uint8_t*>(&o) + aLog[i].off, &aLog[i].value, 8);
        if (o.q[0] == kVtLookAt)
        {
            if (o.q[6] != origQ30 || o.q[7] != origQ38)
            {
                aWhy = "LookAt vtable with non-original payload after write " + std::to_string(i);
                return false;
            }
        }
        else if (o.q[0] == kVtWindow)
        {
            if (!tinyOrCfg(LoF(o.q[6]), aW.height) || !tinyOrCfg(HiF(o.q[6]), aW.width) ||
                !tinyOrCfg(LoF(o.q[7]), aW.curvature) || !tinyOrCfg(HiF(o.q[7]), aW.maxPriorityBoundsFactor))
            {
                aWhy = "window vtable with a non-window, non-empty payload after write " + std::to_string(i);
                return false;
            }
        }
        else
        {
            aWhy = "unknown vtable";
            return false;
        }
    }
    return true;
}

void TestGate()
{
    loot::TppGate g;
    Check(!g.Tpp(1.0), "gate: no heartbeat -> not TPP");
    g.Heartbeat(1.0, true, 0.0f);
    Check(g.Tpp(1.2), "gate: owner says TPP, fresh -> TPP");
    Check(!g.Tpp(1.6), "gate: stale after 0.5 s -> not TPP");
    Check(!g.Tpp(0.9), "gate: time going backwards -> not TPP");
    g.Heartbeat(2.0, false, 0.05f);
    Check(!g.Tpp(2.1), "gate: FPP (camera 5 cm from eye) -> not TPP");
    g.Heartbeat(3.0, false, 2.4f);
    Check(g.Tpp(3.1), "gate: camera 2.4 m from eye -> TPP without owner");
    g.Heartbeat(4.0, false, std::nanf(""));
    Check(!g.Tpp(4.1), "gate: NaN distance without owner -> not TPP");
    loot::GateParams p;
    p.staleSec = 0.0;
    Check(!g.SetParams(p), "gate: stale 0 rejected");
    p.staleSec = 1.0;
    p.tppCameraDist = 1.0f;
    Check(g.SetParams(p), "gate: valid params accepted");
    g.Heartbeat(5.0, false, 0.8f);
    Check(!g.Tpp(5.9), "gate: 0.8 m < new threshold 1.0 -> not TPP");
}

void TestWindow()
{
    loot::WindowParams w;
    Check(loot::ValidWindow(w), "window: vanilla loot drop defaults valid (0.65/0.65/1/0.4)");
    Check(loot::ScaledWindow(1.0f, w) && w.width == 0.65f && w.height == 0.65f, "window: scale 1 = vanilla loot drop");
    Check(loot::ScaledWindow(2.0f, w) && std::fabs(w.width - 1.3f) < 1e-6f, "window: scale 2");
    Check(!loot::ScaledWindow(0.1f, w) && !loot::ScaledWindow(3.0f, w) && !loot::ScaledWindow(std::nanf(""), w),
          "window: scale out of range / NaN rejected");
    loot::WindowParams bad;
    bad.curvature = std::nanf("");
    Check(!loot::ValidWindow(bad), "window: NaN curvature rejected");
    Check(loot::WindowQ30(loot::WindowParams{}) == loot::PackPair(0.65f, 0.65f), "window: q30 = height | width<<32");
}

void TestSlots()
{
    const loot::VtablePair vt{kVtLookAt, kVtWindow};
    loot::WindowParams w;
    FakeMem mem;

    FakeObj notLookAt = MakeLookAt();
    notLookAt.q[0] = kVtOther;
    loot::Slot s0;
    Check(!loot::CaptureSlot(mem, &notLookAt, vt, s0), "slot: capture refuses a non-LookAt object");
    Check(!loot::CaptureSlot(mem, reinterpret_cast<uint8_t*>(&notLookAt) + 4, vt, s0), "slot: capture refuses unaligned");

    FakeObj o = MakeLookAt();
    const FakeObj start = o;
    loot::Slot s;
    Check(loot::CaptureSlot(mem, &o, vt, s), "slot: capture LookAt");
    loot::StepSlot(mem, s, vt, true, w, 10.0, 0.1);
    Check(s.state == loot::SlotState::VtableSwapped && o.q[0] == kVtWindow && o.q[6] == start.q[6] && o.q[7] == start.q[7],
          "slot: apply phase 1 swaps only the vtable");
    loot::StepSlot(mem, s, vt, true, w, 10.05, 0.1);
    Check(s.state == loot::SlotState::VtableSwapped && o.q[6] == start.q[6], "slot: payload not written before minGap");
    loot::StepSlot(mem, s, vt, true, w, 10.2, 0.1);
    Check(s.state == loot::SlotState::Morphed && LoF(o.q[6]) == 0.65f && HiF(o.q[6]) == 0.65f && LoF(o.q[7]) == 1.0f &&
              HiF(o.q[7]) == 0.4f,
          "slot: phase 2 writes the window payload");
    const size_t writesBefore = mem.log.size();
    loot::StepSlot(mem, s, vt, true, w, 10.3, 0.1);
    Check(mem.log.size() == writesBefore, "slot: no writes when already morphed with the same window");
    loot::WindowParams w2;
    loot::ScaledWindow(1.5f, w2);
    loot::StepSlot(mem, s, vt, true, w2, 10.4, 0.1);
    Check(s.state == loot::SlotState::Morphed && std::fabs(HiF(o.q[6]) - 0.975f) < 1e-6f, "slot: window change rewrites payload");
    loot::StepSlot(mem, s, vt, false, w2, 10.5, 0.1);
    Check(s.state == loot::SlotState::Vanilla && std::memcmp(&o, &start, sizeof(o)) == 0,
          "slot: revert restores the exact original 0x40 bytes");
    const size_t n = mem.log.size();
    Check(n >= 3 && mem.log[n - 3].off == 0x38 && mem.log[n - 2].off == 0x30 && mem.log[n - 1].off == 0x00,
          "slot: revert order payload 0x38, 0x30, then vtable");
    std::string why;
    // Note: the log covers the window change too; the invariant uses the final window for the check of tiny-or-config,
    // so replay the two halves separately.
    {
        FakeMem m2;
        FakeObj a = MakeLookAt();
        loot::Slot sa;
        loot::CaptureSlot(m2, &a, vt, sa);
        loot::StepSlot(m2, sa, vt, true, w, 0.0, 0.1);
        loot::StepSlot(m2, sa, vt, true, w, 0.2, 0.1);
        loot::StepSlot(m2, sa, vt, false, w, 0.3, 0.1);
        Check(InvariantHolds(MakeLookAt(), m2.log, w, why), "slot: reader invariant over apply + revert", why);
    }
    {
        FakeMem m3;
        FakeObj a = MakeLookAt();
        loot::Slot sa;
        loot::CaptureSlot(m3, &a, vt, sa);
        loot::StepSlot(m3, sa, vt, true, w, 0.0, 0.1);
        loot::StepSlot(m3, sa, vt, false, w, 0.05, 0.1); // revert from phase 1
        Check(sa.state == loot::SlotState::Vanilla && a.q[0] == kVtLookAt && m3.log.size() == 2,
              "slot: revert from phase 1 = vtable back only");
        Check(InvariantHolds(MakeLookAt(), m3.log, w, why), "slot: reader invariant over phase 1 + revert", why);
    }
    // Negative control for the invariant: a wrong order (payload first, then vtable) must be detected.
    {
        std::vector<WriteRec> wrong = {{nullptr, 0x30, loot::WindowQ30(w)}, {nullptr, 0x38, loot::WindowQ38(w)},
                                    {nullptr, 0x00, kVtWindow}};
        Check(!InvariantHolds(MakeLookAt(), wrong, w, why), "slot: invariant detects payload-before-vtable (negative control)");
    }
    // Foreign: someone else changed the vtable.
    {
        FakeMem m4;
        FakeObj a = MakeLookAt();
        loot::Slot sa;
        loot::CaptureSlot(m4, &a, vt, sa);
        a.q[0] = kVtOther;
        loot::StepSlot(m4, sa, vt, true, w, 0.0, 0.1);
        Check(sa.state == loot::SlotState::Foreign && m4.log.empty(), "slot: unexpected vtable -> foreign, no write");
        loot::StepSlot(m4, sa, vt, false, w, 1.0, 0.1);
        Check(m4.log.empty() && !loot::RevertSlot(m4, sa, vt), "slot: foreign slot is never written again");
    }
    {
        FakeMem m5;
        FakeObj a = MakeLookAt();
        loot::Slot sa;
        loot::CaptureSlot(m5, &a, vt, sa);
        a.q[6] ^= 1; // payload changed after capture
        loot::StepSlot(m5, sa, vt, true, w, 0.0, 0.1);
        Check(sa.state == loot::SlotState::Foreign && m5.log.empty(), "slot: payload changed since capture -> foreign");
    }
    {
        FakeMem m6;
        FakeObj a = MakeLookAt();
        loot::Slot sa;
        loot::CaptureSlot(m6, &a, vt, sa);
        loot::StepSlot(m6, sa, vt, true, w, 0.0, 0.1);
        m6.failWrites = true;
        loot::StepSlot(m6, sa, vt, true, w, 1.0, 0.1);
        Check(sa.state == loot::SlotState::Foreign, "slot: write failure -> foreign");
    }
}

// ---------------------------------------------------------------------------------------------
// Synthetic descriptor tree with the real offsets.
struct Arena
{
    std::vector<std::vector<uint8_t>*> blocks;
    ~Arena()
    {
        for (auto* b : blocks)
            delete b;
    }
    uint8_t* Alloc(size_t n)
    {
        auto* b = new std::vector<uint8_t>(n + 16, 0);
        blocks.push_back(b);
        auto p = reinterpret_cast<uintptr_t>(b->data());
        return reinterpret_cast<uint8_t*>((p + 15) & ~static_cast<uintptr_t>(15));
    }
};

void Put64(void* aBase, uint32_t aOff, uint64_t aV) { std::memcpy(static_cast<uint8_t*>(aBase) + aOff, &aV, 8); }
void PutPtr(void* aBase, uint32_t aOff, const void* aP) { Put64(aBase, aOff, reinterpret_cast<uint64_t>(aP)); }
void Put32(void* aBase, uint32_t aOff, uint32_t aV) { std::memcpy(static_cast<uint8_t*>(aBase) + aOff, &aV, 4); }

struct Reader final : loot::IDescriptorReader
{
    bool failAll = false;
    bool Ptr(const void* aBase, uint32_t aOff, void*& aOut) override
    {
        if (failAll || !aBase)
            return false;
        uint64_t v;
        std::memcpy(&v, static_cast<const uint8_t*>(aBase) + aOff, 8);
        aOut = reinterpret_cast<void*>(v);
        return true;
    }
    bool U32(const void* aBase, uint32_t aOff, uint32_t& aOut) override
    {
        if (failAll || !aBase)
            return false;
        std::memcpy(&aOut, static_cast<const uint8_t*>(aBase) + aOff, 4);
        return true;
    }
    bool U64(const void* aBase, uint32_t aOff, uint64_t& aOut) override
    {
        if (failAll || !aBase)
            return false;
        std::memcpy(&aOut, static_cast<const uint8_t*>(aBase) + aOff, 8);
        return true;
    }
    bool IsLookAt(void* aObj) override
    {
        uint64_t vt;
        std::memcpy(&vt, aObj, 8);
        return vt == kVtLookAt;
    }
};

constexpr uint64_t kTagLoot = 0x1111;
constexpr uint64_t kTagGrapple = 0x2222;
constexpr uint64_t kTagTalk = 0x3333;

struct Tree
{
    Arena arena;
    loot::DescriptorOffsets o;

    uint8_t* Pred(uint64_t aVt)
    {
        uint8_t* p = arena.Alloc(0x40);
        Put64(p, 0, aVt);
        return p;
    }
    // CFunctorDefinition with predicate {type, f1, f2}
    uint8_t* Functor(void* aType, void* aF1 = nullptr, void* aF2 = nullptr)
    {
        uint8_t* f = arena.Alloc(0x70);
        uint8_t* pd = f + o.functorPredicate;
        PutPtr(pd, o.predType, aType);
        PutPtr(pd, o.predFunctor1, aF1);
        PutPtr(pd, o.predFunctor2, aF2);
        return f;
    }
    uint8_t* Layer(uint64_t aTag, void* aFunctor)
    {
        uint8_t* area = arena.Alloc(0xA0);
        PutPtr(area, o.areaFunctor, aFunctor);
        uint8_t* l = arena.Alloc(0x90);
        Put64(l, o.layerTag, aTag);
        PutPtr(l, o.layerAreaFilter, area);
        return l;
    }
    uint8_t* HandleArray(const std::vector<void*>& aObjs)
    {
        uint8_t* a = arena.Alloc(16 * (aObjs.size() + 1));
        for (size_t i = 0; i < aObjs.size(); ++i)
            PutPtr(a, static_cast<uint32_t>(i * 16), aObjs[i]);
        return a;
    }
    uint8_t* Linked(const std::vector<void*>& aLayers)
    {
        uint8_t* ll = arena.Alloc(0x80);
        PutPtr(ll, o.linkedLayers, HandleArray(aLayers));
        Put32(ll, o.linkedLayers + 8, static_cast<uint32_t>(aLayers.size()));
        Put32(ll, o.linkedLayers + 12, static_cast<uint32_t>(aLayers.size()));
        return ll;
    }
    uint8_t* Resource(const std::vector<void*>& aLinked)
    {
        uint8_t* r = arena.Alloc(0x100);
        uint8_t* def = r + o.resDefinition;
        PutPtr(def, o.defLayers, HandleArray(aLinked));
        Put32(def, o.defLayers + 8, static_cast<uint32_t>(aLinked.size()));
        Put32(def, o.defLayers + 12, static_cast<uint32_t>(aLinked.size()));
        return r;
    }
};

void TestWalk()
{
    Reader rd;
    {
        // generic_container_interaction: Loot = ContainedInShapes AND f1(LookAt); LootGenerator etc. without LookAt;
        // a Grapple layer (default_puppet) with its own LookAt must NOT be collected.
        Tree t;
        uint8_t* lookLoot = t.Pred(kVtLookAt);
        uint8_t* shapes = t.Pred(kVtOther);
        uint8_t* lootLayer = t.Layer(kTagLoot, t.Functor(shapes, t.Functor(lookLoot)));
        uint8_t* genLayer = t.Layer(0x4444, t.Functor(t.Pred(kVtOther)));
        uint8_t* lookGrapple = t.Pred(kVtLookAt);
        uint8_t* grapple = t.Layer(kTagGrapple, t.Functor(t.Pred(kVtOther), t.Functor(lookGrapple)));
        uint8_t* res = t.Resource({t.Linked({grapple}), t.Linked({lootLayer, genLayer})});
        const loot::WalkResult w = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(w.leaves.size() == 1 && w.leaves[0] == lookLoot && w.lootLayers == 1 && !w.readError && !w.truncated,
              "walk: container Loot layer -> exactly its LookAt leaf (Grapple LookAt untouched)");
    }
    {
        // GenericTalk-like nesting (OR / AND) inside a Loot layer, shared leaf referenced twice, null handles.
        Tree t;
        uint8_t* look = t.Pred(kVtLookAt);
        uint8_t* inner = t.Functor(t.Pred(kVtOther), t.Functor(look), nullptr);
        uint8_t* layer = t.Layer(kTagLoot, t.Functor(t.Pred(kVtOther), inner, t.Functor(look)));
        uint8_t* talk = t.Layer(kTagTalk, t.Functor(t.Pred(kVtLookAt)));
        uint8_t* res = t.Resource({t.Linked({layer, talk}), nullptr});
        const loot::WalkResult w = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(w.leaves.size() == 1 && w.leaves[0] == look, "walk: nested tree + shared leaf deduplicated, null linked skipped");
    }
    {
        Tree t;
        uint8_t* layer = t.Layer(kTagLoot, nullptr);
        uint8_t* res = t.Resource({t.Linked({layer})});
        const loot::WalkResult w = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(w.leaves.empty() && w.lootLayers == 1 && !w.readError, "walk: Loot layer without functor -> nothing");
    }
    {
        // Deep chain beyond the depth limit -> truncated, no crash.
        Tree t;
        uint8_t* f = t.Functor(t.Pred(kVtLookAt));
        for (int i = 0; i < 20; ++i)
            f = t.Functor(t.Pred(kVtOther), f);
        uint8_t* res = t.Resource({t.Linked({t.Layer(kTagLoot, f)})});
        const loot::WalkResult w = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(w.truncated && w.leaves.empty(), "walk: depth limit -> truncated");
    }
    {
        Tree t;
        std::vector<void*> many(100, nullptr); // 100 handles (readable), only the first used
        many[0] = t.Linked({t.Layer(kTagLoot, t.Functor(t.Pred(kVtLookAt)))});
        uint8_t* res = t.Resource(many);
        const loot::WalkResult w = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(w.truncated && w.leaves.size() == 1, "walk: array size above the cap (64) truncated, readable part walked");
        Reader bad;
        bad.failAll = true;
        const loot::WalkResult w2 = loot::WalkDescriptor(bad, t.o, res, kTagLoot);
        Check(w2.readError && w2.leaves.empty(), "walk: read failure -> readError, nothing collected");
    }
}

void TestLootReasons()
{
    loot::Inputs in;
    in.resolved = in.enabled = in.tpp = in.validated = in.haveSlots = true;
    Check(loot::Reasons(in) == 0 && loot::WantMorph(in), "loot reasons: all good -> active");
    loot::Inputs noSlots = in;
    noSlots.haveSlots = false;
    Check(loot::WantMorph(noSlots) && loot::Reasons(noSlots) == loot::kNoSlots, "loot reasons: no slots yet still wants (loads)");
    const struct
    {
        bool loot::Inputs::*field;
        bool value;
        uint32_t bit;
        const char* name;
    } cases[] = {{&loot::Inputs::resolved, false, loot::kNotResolved, "not resolved"},
                 {&loot::Inputs::enabled, false, loot::kDisabled, "disabled"},
                 {&loot::Inputs::killed, true, loot::kKilled, "killed"},
                 {&loot::Inputs::tpp, false, loot::kNotTpp, "FPP"},
                 {&loot::Inputs::killFile, true, loot::kKillFile, "kill file"},
                 {&loot::Inputs::validated, false, loot::kNotValidated, "not validated"}};
    for (const auto& c : cases)
    {
        loot::Inputs x = in;
        x.*(c.field) = c.value;
        Check(!loot::WantMorph(x) && (loot::Reasons(x) & c.bit), (std::string("loot reasons: ") + c.name + " -> off").c_str());
    }
}

// ---------------------------------------------------------------------------------------------
// InteractRange (vehicles / devices / talk): same morph, per-group target class.
constexpr uint64_t kVtSame = 0x1400400000ull; // AlwaysSamePredicate

// Replays a write log: under LookAt the payload is the original; under the window class every float is tiny or the
// configured value; under AlwaysSame the priority (low dword of 0x30) is tiny or the configured priority.
bool InteractInvariantHolds(const FakeObj& aStart, const std::vector<WriteRec>& aLog, const loot::WindowParams& aW,
                            float aPriority, std::string& aWhy)
{
    FakeObj o = aStart;
    auto tinyOrCfg = [&](float v, float cfg) { return std::fabs(v) < 1e-30f || v == cfg; };
    for (size_t i = 0; i < aLog.size(); ++i)
    {
        std::memcpy(reinterpret_cast<uint8_t*>(&o) + aLog[i].off, &aLog[i].value, 8);
        if (o.q[0] == kVtLookAt)
        {
            if (o.q[6] != aStart.q[6] || o.q[7] != aStart.q[7])
            {
                aWhy = "LookAt vtable with non-original payload after write " + std::to_string(i);
                return false;
            }
        }
        else if (o.q[0] == kVtWindow)
        {
            if (!tinyOrCfg(LoF(o.q[6]), aW.height) || !tinyOrCfg(HiF(o.q[6]), aW.width) ||
                !tinyOrCfg(LoF(o.q[7]), aW.curvature) || !tinyOrCfg(HiF(o.q[7]), aW.maxPriorityBoundsFactor))
            {
                aWhy = "window vtable with a non-window payload after write " + std::to_string(i);
                return false;
            }
        }
        else if (o.q[0] == kVtSame)
        {
            if (!tinyOrCfg(LoF(o.q[6]), aPriority))
            {
                aWhy = "AlwaysSame vtable with a priority neither ~0 nor configured after write " + std::to_string(i);
                return false;
            }
        }
        else
        {
            aWhy = "unknown vtable";
            return false;
        }
    }
    return true;
}

interact::Slot CapturedSlot(FakeMem& aMem, FakeObj& aObj, interact::Group aGroup)
{
    interact::Slot sl;
    sl.group = aGroup;
    loot::CaptureSlot(aMem, &aObj, loot::VtablePair{kVtLookAt, kVtWindow}, sl.s);
    return sl;
}

void TestInteract()
{
    using interact::Group;
    using interact::StepInteractSlot;
    // Descriptor table: scope checks.
    size_t n = 0;
    const interact::DescriptorSpec* specs = interact::Descriptors(n);
    bool tagsOk = n >= 10, pathsUnique = true, vehicleTagsOk = true, groupsOk = true;
    int vehicles = 0, devices = 0, talk = 0;
    for (size_t i = 0; i < n; ++i)
    {
        const size_t tc = interact::TagCount(specs[i]);
        tagsOk = tagsOk && tc >= 1 && std::strstr(specs[i].path, ".interaction") != nullptr;
        for (size_t k = 0; k < tc; ++k)
        {
            const std::string t = specs[i].tags[k];
            tagsOk = tagsOk && t != "Loot" && t != "Grapple" && t != "trunk" && t != "hood" &&
                     t.find("akedown") == std::string::npos;
            if (specs[i].group == Group::Vehicle)
                vehicleTagsOk = vehicleTagsOk && (t == "Mount" || t.rfind("seat_", 0) == 0);
        }
        for (size_t j = i + 1; j < n; ++j)
            pathsUnique = pathsUnique && std::strcmp(specs[i].path, specs[j].path) != 0;
        vehicles += specs[i].group == Group::Vehicle ? 1 : 0;
        devices += specs[i].group == Group::Device ? 1 : 0;
        talk += specs[i].group == Group::Talk ? 1 : 0;
        if (std::strstr(specs[i].path, "default_puppet") && specs[i].group == Group::Talk)
            groupsOk = groupsOk && tc == 1 && std::strcmp(specs[i].tags[0], "ReturnTalk") == 0; // GenericTalk: vanilla window
    }
    Check(tagsOk && pathsUnique, "interact: descriptor table well formed, no Loot / Grapple / takedown / trunk / hood layers");
    Check(vehicleTagsOk && vehicles >= 5 && devices >= 3 && talk >= 1 && groupsOk,
          "interact: vehicle specs only seat_* / Mount, devices and talk present, default_puppet GenericTalk untouched");

    // Multi-tag walk (vehicle_device-like): two seats collected, trunk and a Loot layer not.
    {
        Reader rd;
        Tree t;
        constexpr uint64_t kSeatFL = 0x5101, kSeatFR = 0x5102, kTrunk = 0x5103;
        uint8_t* lFL = t.Pred(kVtLookAt);
        uint8_t* lFR = t.Pred(kVtLookAt);
        uint8_t* lTrunk = t.Pred(kVtLookAt);
        uint8_t* lLoot = t.Pred(kVtLookAt);
        uint8_t* res = t.Resource({t.Linked({t.Layer(kTrunk, t.Functor(t.Pred(kVtOther), t.Functor(lTrunk))),
                                             t.Layer(kSeatFL, t.Functor(t.Pred(kVtOther), t.Functor(lFL))),
                                             t.Layer(kSeatFR, t.Functor(t.Pred(kVtOther), t.Functor(lFR))),
                                             t.Layer(kTagLoot, t.Functor(t.Pred(kVtOther), t.Functor(lLoot)))})});
        const uint64_t tags[] = {kSeatFL, kSeatFR};
        const loot::WalkResult w = loot::WalkDescriptorTags(rd, t.o, res, tags, 2);
        Check(w.leaves.size() == 2 && w.leaves[0] == lFL && w.leaves[1] == lFR && w.lootLayers == 2 && !w.readError,
              "interact walk: seat layers collected, trunk / Loot LookAt untouched");
        const loot::WalkResult wl = loot::WalkDescriptor(rd, t.o, res, kTagLoot);
        Check(wl.leaves.size() == 1 && wl.leaves[0] == lLoot, "interact walk: single-tag wrapper = Loot only (LootRange unchanged)");
    }

    // Payload helpers and the AlwaysSame code proof.
    Check(interact::ValidPriority(0.5f) && interact::ValidPriority(1.0f) && !interact::ValidPriority(0.0f) &&
              !interact::ValidPriority(1.5f) && !interact::ValidPriority(std::nanf("")),
          "interact: priority range 0.05..1, NaN rejected");
    Check(interact::ValidVehicleMode(0) && interact::ValidVehicleMode(2) && interact::ValidVehicleMode(3) &&
              !interact::ValidVehicleMode(4) && !interact::ValidVehicleMode(-1),
          "interact: vehicle modes 0..3");
    Check(interact::ListPatchMode(interact::VehicleMode::Proximity) && interact::ListPatchMode(interact::VehicleMode::Window) &&
              !interact::ListPatchMode(interact::VehicleMode::Vanilla) &&
              !interact::ListPatchMode(interact::VehicleMode::ProximityNoList),
          "interact: list patch only in modes 1 / 2 (3 = morph only, A/B)");
    Check(interact::kDefaultVehiclePriority == 1.0f && interact::Config{}.vehiclePriority == 1.0f,
          "interact: default vehicle priority 1.0 (vanilla AlwaysSame value)");
    Check(LoF(interact::PriorityQ30(0.5f, 0x0000000100000000ull)) == 0.5f &&
              (interact::PriorityQ30(0.5f, 0x0000000100000000ull) >> 32) == 1u,
          "interact: AlwaysSame q30 = priority | original high dword");
    {
        uint8_t code[9];
        std::memcpy(code, interact::kAlwaysSameEvalCode, 9);
        const bool good = interact::IsAlwaysSameEval(code, 9);
        code[2] = 0x38; // reads another field
        Check(good && !interact::IsAlwaysSameEval(code, 9) && !interact::IsAlwaysSameEval(code, 4) &&
                  !interact::IsAlwaysSameEval(nullptr, 9),
              "interact: AlwaysSame evaluate-code proof (match / altered / short / null)");
    }
    // Phase 1 under AlwaysSame reads the LookAt testTarget int as the priority: 0 or a denormal (never a real value).
    for (uint32_t testTarget : {0u, 1u, 2u})
        Check(std::fabs(LoF(static_cast<uint64_t>(testTarget))) < 1e-30f,
              ("interact: phase-1 priority from testTarget " + std::to_string(testTarget) + " is ~0").c_str());

    const interact::Vtables vt{kVtLookAt, kVtWindow, kVtSame};
    const interact::Config cfg;
    const float prio = cfg.vehiclePriority;

    // Vehicle, proximity: two-phase apply into AlwaysSame, exact revert.
    {
        FakeMem mem;
        FakeObj o = MakeLookAt();
        const FakeObj start = o;
        interact::Slot sl = CapturedSlot(mem, o, Group::Vehicle);
        Check(sl.s.obj == &o, "interact vehicle: capture");
        StepInteractSlot(mem, sl, true, vt, cfg, 0.0, 0.1);
        Check(sl.s.state == loot::SlotState::VtableSwapped && o.q[0] == kVtSame && o.q[6] == start.q[6] && o.q[7] == start.q[7],
              "interact vehicle: phase 1 swaps only the vtable (AlwaysSame)");
        StepInteractSlot(mem, sl, true, vt, cfg, 0.05, 0.1);
        Check(o.q[6] == start.q[6], "interact vehicle: no payload before minGap");
        StepInteractSlot(mem, sl, true, vt, cfg, 0.2, 0.1);
        Check(sl.s.state == loot::SlotState::Morphed && LoF(o.q[6]) == prio && (o.q[6] >> 32) == (start.q[6] >> 32) &&
                  o.q[7] == start.q[7],
              "interact vehicle: phase 2 writes the priority only (0x38.. stays original)");
        const size_t w0 = mem.log.size();
        StepInteractSlot(mem, sl, true, vt, cfg, 0.3, 0.1);
        Check(mem.log.size() == w0, "interact vehicle: no writes while morphed with the same config");
        StepInteractSlot(mem, sl, false, vt, cfg, 0.4, 0.1);
        Check(sl.s.state == loot::SlotState::Vanilla && std::memcmp(&o, &start, sizeof(o)) == 0,
              "interact vehicle: not wanted (FPP) -> exact original 0x40 bytes");
        std::string why;
        Check(InteractInvariantHolds(start, mem.log, cfg.window, prio, why), "interact vehicle: reader invariant apply + revert",
              why);
    }
    // Vehicle without a proven AlwaysSame -> window fallback; Vanilla mode -> untouched.
    {
        FakeMem mem;
        FakeObj o = MakeLookAt();
        interact::Slot sl = CapturedSlot(mem, o, Group::Vehicle);
        const interact::Vtables noSame{kVtLookAt, kVtWindow, 0};
        StepInteractSlot(mem, sl, true, noSame, cfg, 0.0, 0.1);
        StepInteractSlot(mem, sl, true, noSame, cfg, 0.2, 0.1);
        Check(o.q[0] == kVtWindow && LoF(o.q[6]) == 0.65f, "interact vehicle: AlwaysSame not proven -> screen window");
        FakeMem m2;
        FakeObj o2 = MakeLookAt();
        interact::Slot s2 = CapturedSlot(m2, o2, Group::Vehicle);
        interact::Config van = cfg;
        van.vehicleMode = interact::VehicleMode::Vanilla;
        StepInteractSlot(m2, s2, true, vt, van, 0.0, 0.1);
        StepInteractSlot(m2, s2, true, vt, van, 0.2, 0.1);
        Check(m2.log.empty() && s2.s.state == loot::SlotState::Vanilla, "interact vehicle: mode Vanilla -> no write");
    }
    // Mode change while morphed: proximity -> window goes through vanilla first; the whole log keeps the invariant.
    {
        FakeMem mem;
        FakeObj o = MakeLookAt();
        const FakeObj start = o;
        interact::Slot sl = CapturedSlot(mem, o, Group::Vehicle);
        StepInteractSlot(mem, sl, true, vt, cfg, 0.0, 0.1);
        StepInteractSlot(mem, sl, true, vt, cfg, 0.2, 0.1);
        interact::Config win = cfg;
        win.vehicleMode = interact::VehicleMode::Window;
        StepInteractSlot(mem, sl, true, vt, win, 0.3, 0.1);
        Check(sl.s.state == loot::SlotState::Vanilla && std::memcmp(&o, &start, sizeof(o)) == 0,
              "interact mode change: first step restores the original");
        StepInteractSlot(mem, sl, true, vt, win, 0.4, 0.1);
        StepInteractSlot(mem, sl, true, vt, win, 0.6, 0.1);
        Check(sl.s.state == loot::SlotState::Morphed && o.q[0] == kVtWindow && LoF(o.q[6]) == 0.65f && sl.morphVt == kVtWindow,
              "interact mode change: then two-phase into the window");
        StepInteractSlot(mem, sl, false, vt, win, 0.7, 0.1);
        std::string why;
        Check(std::memcmp(&o, &start, sizeof(o)) == 0 && InteractInvariantHolds(start, mem.log, cfg.window, prio, why),
              "interact mode change: invariant over proximity -> vanilla -> window -> vanilla", why);
        // Revert from phase 1 of AlwaysSame (FPP right after entering TPP).
        FakeMem m2;
        FakeObj a = MakeLookAt();
        interact::Slot sa = CapturedSlot(m2, a, Group::Vehicle);
        StepInteractSlot(m2, sa, true, vt, cfg, 0.0, 0.1);
        Check(interact::RevertInteractSlot(m2, sa, vt) && a.q[0] == kVtLookAt && m2.log.size() == 2,
              "interact vehicle: revert from phase 1 = vtable back only");
    }
    // Device: window with the user scale; foreign detection still applies.
    {
        FakeMem mem;
        FakeObj o = MakeLookAt();
        interact::Slot sl = CapturedSlot(mem, o, Group::Device);
        interact::Config big = cfg;
        loot::ScaledWindow(1.5f, big.window);
        StepInteractSlot(mem, sl, true, vt, big, 0.0, 0.1);
        StepInteractSlot(mem, sl, true, vt, big, 0.2, 0.1);
        Check(o.q[0] == kVtWindow && std::fabs(HiF(o.q[6]) - 0.975f) < 1e-6f, "interact device: screen window x1.5");
        FakeMem m2;
        FakeObj a = MakeLookAt();
        interact::Slot sa = CapturedSlot(m2, a, Group::Device);
        a.q[0] = kVtOther; // e.g. another writer
        StepInteractSlot(m2, sa, true, vt, cfg, 0.0, 0.1);
        StepInteractSlot(m2, sa, false, vt, cfg, 0.1, 0.1);
        Check(sa.s.state == loot::SlotState::Foreign && m2.log.empty(),
              "interact device: unexpected vtable -> foreign, never written");
    }
    // Priority change on an already morphed seat: payload rewritten in place on the next step (no revert, no vtable
    // write), the original high dword kept; mode 3 morphs into AlwaysSame like mode 1.
    {
        FakeMem mem;
        FakeObj o = MakeLookAt();
        const FakeObj start = o;
        interact::Slot sl = CapturedSlot(mem, o, Group::Vehicle);
        interact::Config c = cfg;
        StepInteractSlot(mem, sl, true, vt, c, 0.0, 0.1);
        StepInteractSlot(mem, sl, true, vt, c, 0.2, 0.1);
        const size_t w0 = mem.log.size();
        c.vehiclePriority = 0.5f;
        StepInteractSlot(mem, sl, true, vt, c, 0.3, 0.1);
        bool onlyPayload = mem.log.size() > w0;
        for (size_t i = w0; i < mem.log.size(); ++i)
            onlyPayload = onlyPayload && mem.log[i].off != 0x00;
        Check(sl.s.state == loot::SlotState::Morphed && o.q[0] == kVtSame && LoF(o.q[6]) == 0.5f &&
                  (o.q[6] >> 32) == (start.q[6] >> 32) && onlyPayload,
              "interact vehicle: priority change applies to an already morphed seat (payload only)");
        std::string why;
        Check(InteractInvariantHolds(start, std::vector<WriteRec>(mem.log.begin(), mem.log.begin() + w0), c.window, 1.0f,
                                     why),
              "interact vehicle: reader invariant of the apply at the default priority 1.0", why);
        interact::Config m3 = cfg;
        m3.vehicleMode = interact::VehicleMode::ProximityNoList;
        FakeMem m2;
        FakeObj a = MakeLookAt();
        interact::Slot sa = CapturedSlot(m2, a, Group::Vehicle);
        StepInteractSlot(m2, sa, true, vt, m3, 0.0, 0.1);
        StepInteractSlot(m2, sa, true, vt, m3, 0.2, 0.1);
        Check(a.q[0] == kVtSame && LoF(a.q[6]) == 1.0f, "interact vehicle: mode 3 = AlwaysSame morph");
        // Switching 3 -> 1 keeps the morph (same target class), no writes.
        const size_t w1 = m2.log.size();
        StepInteractSlot(m2, sa, true, vt, cfg, 0.3, 0.1);
        Check(m2.log.size() == w1 && sa.s.state == loot::SlotState::Morphed, "interact vehicle: mode 3 -> 1 without rewrite");
    }
    // Negative control: AlwaysSame payload written before the vtable must break the invariant.
    {
        std::vector<WriteRec> wrong = {{nullptr, 0x30, interact::PriorityQ30(prio, MakeLookAt().q[6])},
                                    {nullptr, 0x00, kVtSame}};
        std::string why;
        Check(!InteractInvariantHolds(MakeLookAt(), wrong, cfg.window, prio, why),
              "interact: invariant detects payload-before-vtable (negative control)");
    }
}

// ---------------------------------------------------------------------------------------------
// InteractRange: compiled non-look-at list (layout of the 2.5 post-load data, see InteractRangeCore.hpp).
struct ListWrite
{
    uint64_t addr;
    uint32_t width; // 4 or 8 (IncStrong = 4)
    uint64_t value;
    uint64_t old;
    double time;    // clock of the step that wrote it
};

struct FakeListMem final : interact::IListMemory
{
    Arena arena;
    std::vector<ListWrite> log;
    double now = 0.0; // set by the test to the time passed to StepList
    uint64_t imgLo = 0, imgHi = 0;
    bool failAlloc = false;
    uint32_t allocs = 0;
    uint64_t lastAllocator = 0;
    bool R64(uint64_t a, uint64_t& o) override
    {
        if (!a)
            return false;
        std::memcpy(&o, reinterpret_cast<const void*>(a), 8);
        return true;
    }
    bool R32(uint64_t a, uint32_t& o) override
    {
        if (!a)
            return false;
        std::memcpy(&o, reinterpret_cast<const void*>(a), 4);
        return true;
    }
    bool W64(uint64_t a, uint64_t v) override
    {
        if (!a || (a & 7))
            return false;
        uint64_t old;
        std::memcpy(&old, reinterpret_cast<const void*>(a), 8);
        std::memcpy(reinterpret_cast<void*>(a), &v, 8);
        log.push_back({a, 8, v, old, now});
        return true;
    }
    bool W32(uint64_t a, uint32_t v) override
    {
        if (!a || (a & 3))
            return false;
        uint32_t old;
        std::memcpy(&old, reinterpret_cast<const void*>(a), 4);
        std::memcpy(reinterpret_cast<void*>(a), &v, 4);
        log.push_back({a, 4, v, old, now});
        return true;
    }
    bool IncStrong(uint64_t rc) override
    {
        uint32_t v = 0;
        if (!R32(rc, v))
            return false;
        return W32(rc, v + 1);
    }
    uint64_t Alloc(uint64_t aAllocator, uint64_t aBytes, uint32_t) override
    {
        if (failAlloc)
            return 0;
        ++allocs;
        lastAllocator = aAllocator;
        return reinterpret_cast<uint64_t>(arena.Alloc(static_cast<size_t>(aBytes)));
    }
    bool InImage(uint64_t a) override { return a >= imgLo && a < imgHi; }

    // Value of the 8 / 4 bytes at a after the first aT writes of the log (undo the later ones).
    uint64_t At(uint64_t a, uint32_t w, size_t aT) const
    {
        uint64_t v = 0;
        std::memcpy(&v, reinterpret_cast<const void*>(a), w);
        for (size_t i = log.size(); i > aT; --i)
            if (log[i - 1].addr == a && log[i - 1].width == w)
                v = log[i - 1].old;
        return v;
    }
};

// Synthetic descriptor: resource (definition groups), compiled object, refcount blocks, a fake image with the allocator
// vtable. aListMembers = indices of groups in the vanilla list (empty -> key 0, {allocator, 0, 0}).
struct ListFixture
{
    FakeListMem m;
    loot::DescriptorOffsets o;
    interact::CompiledLayout L;
    uint8_t* image = nullptr;
    uint64_t allocVt = 0;
    uint8_t* res = nullptr;
    uint8_t* comp = nullptr;
    std::vector<uint8_t*> groups;
    std::vector<uint8_t*> rcs;
    uint8_t* listBuf = nullptr;

    explicit ListFixture(size_t aGroups, const std::vector<int>& aListMembers)
    {
        image = m.arena.Alloc(0x100);
        m.imgLo = reinterpret_cast<uint64_t>(image);
        m.imgHi = m.imgLo + 0x100;
        allocVt = m.imgLo + 0x40;
        PutPtr(image, 0x48, image + 0x80); // allocator vtable slot 1 -> code inside the image
        for (size_t i = 0; i < aGroups; ++i)
        {
            groups.push_back(m.arena.Alloc(0x80));
            uint8_t* rc = m.arena.Alloc(8);
            Put32(rc, 0, 2); // definition + compiled copy
            rcs.push_back(rc);
        }
        res = m.arena.Alloc(0x100);
        comp = m.arena.Alloc(0xD0);
        auto handles = [&](const std::vector<int>& aIdx, uint32_t aCap) {
            uint8_t* a = m.arena.Alloc(interact::ListBufferBytes(aCap));
            for (size_t k = 0; k < aIdx.size(); ++k)
            {
                PutPtr(a, static_cast<uint32_t>(k * 16), groups[aIdx[k]]);
                PutPtr(a, static_cast<uint32_t>(k * 16 + 8), rcs[aIdx[k]]);
            }
            return a;
        };
        std::vector<int> all;
        for (size_t i = 0; i < aGroups; ++i)
            all.push_back(static_cast<int>(i));
        uint8_t* def = res + o.resDefinition + o.defLayers;
        PutPtr(def, 0, handles(all, static_cast<uint32_t>(aGroups)));
        Put32(def, 8, static_cast<uint32_t>(aGroups));
        Put32(def, 12, static_cast<uint32_t>(aGroups));
        PutPtr(res, L.resCompiled, comp);
        PutPtr(comp, L.groups, handles(all, static_cast<uint32_t>(aGroups)));
        Put32(comp, L.groups + 8, static_cast<uint32_t>(aGroups));
        Put32(comp, L.groups + 12, static_cast<uint32_t>(aGroups));
        Put32(comp, L.entryCount, 1);
        if (aListMembers.empty())
        {
            Put64(comp, L.list, allocVt); // capacity 0: allocator in 'entries'
        }
        else
        {
            const uint32_t n = static_cast<uint32_t>(aListMembers.size());
            listBuf = handles(aListMembers, n); // capacity == size (game growth 1, 2, 3, 4, 6, ...)
            Put64(listBuf, static_cast<uint32_t>(interact::Align8(n * 16ull)), allocVt);
            PutPtr(comp, L.list, listBuf);
            Put32(comp, L.list + 8, n);
            Put32(comp, L.list + 12, n);
            PutPtr(comp, L.key, comp);
            for (int i : aListMembers)
                Put32(rcs[i], 0, 3);
        }
    }
    uint64_t Res() const { return reinterpret_cast<uint64_t>(res); }
    uint64_t G(int i) const { return reinterpret_cast<uint64_t>(groups[i]); }
    uint32_t Rc(int i) const
    {
        uint32_t v;
        std::memcpy(&v, rcs[i], 4);
        return v;
    }
    interact::ListProbe Probe() { return interact::ProbeList(m, L, o, Res()); }
    uint64_t A(uint32_t aOff) const { return reinterpret_cast<uint64_t>(comp) + aOff; }

    // Reader race: entries read at one moment, size at another (both orders), the two loads less than aWindow seconds
    // apart (a reader iteration never spans minGap - the same assumption as the two-phase predicate morph). Every
    // element below that size must be a handle of this descriptor's groups with its own refcount block, in the memory
    // of the later moment. State t = after the first t writes: visible from write t-1 until write t.
    bool RaceInvariant(std::string& aWhy, double aWindow) const
    {
        const size_t n = m.log.size();
        auto firstSeen = [&](size_t t) { return t == 0 ? -1e9 : m.log[t - 1].time; };
        auto lastSeen = [&](size_t t) { return t == n ? 1e9 : m.log[t].time; };
        for (size_t t1 = 0; t1 <= n; ++t1)
            for (size_t t2 = t1; t2 <= n; ++t2)
                for (int order = 0; order < 2; ++order)
                    if (firstSeen(t2) - lastSeen(t1) < aWindow)
                {
                    const size_t te = order == 0 ? t1 : t2, ts = order == 0 ? t2 : t1;
                    const uint64_t e = m.At(A(L.list), 8, te);
                    const uint32_t s = static_cast<uint32_t>(m.At(A(L.list + 12), 4, ts));
                    for (uint32_t k = 0; k < s; ++k)
                    {
                        const uint64_t inst = m.At(e + k * 16, 8, t2), rc = m.At(e + k * 16 + 8, 8, t2);
                        bool okH = false;
                        for (size_t g = 0; g < groups.size(); ++g)
                            okH = okH || (inst == reinterpret_cast<uint64_t>(groups[g]) &&
                                          rc == reinterpret_cast<uint64_t>(rcs[g]));
                        if (!okH)
                        {
                            aWhy = "entries@" + std::to_string(te) + " size@" + std::to_string(ts) + " element " +
                                   std::to_string(k) + " invalid";
                            return false;
                        }
                    }
                }
        return true;
    }
    size_t WriteIndex(uint64_t aAddr, uint64_t aValue, size_t aFrom = 0) const
    {
        for (size_t i = aFrom; i < m.log.size(); ++i)
            if (m.log[i].addr == aAddr && m.log[i].value == aValue)
                return i;
        return SIZE_MAX;
    }
};

void TestInteractList()
{
    using interact::ListState;
    const double gap = 0.1;
    auto Step = [&](ListFixture& aF, interact::ListPatch& aP, const std::vector<uint64_t>& aWant, double aT) {
        aF.m.now = aT;
        return interact::StepList(aF.m, aF.L, aF.o, aF.Res(), aP, aWant, aT, gap);
    };
    // Group-aware walk: seat leaves per group, trunk / Loot untouched, shared leaf kept once.
    {
        Reader rd;
        Tree t;
        constexpr uint64_t kSeat = 0x6101, kTrunk = 0x6102, kPuppet = 0x6103;
        uint8_t* lSeat = t.Pred(kVtLookAt);
        uint8_t* lShared = t.Pred(kVtLookAt);
        uint8_t* g0 = t.Linked({t.Layer(kSeat, t.Functor(t.Pred(kVtOther), t.Functor(lSeat), t.Functor(lShared)))});
        uint8_t* g1 = t.Linked({t.Layer(kPuppet, t.Functor(t.Pred(kVtOther)))});
        uint8_t* g2 = t.Linked({t.Layer(kTrunk, t.Functor(t.Pred(kVtOther), t.Functor(t.Pred(kVtLookAt)))),
                                t.Layer(kSeat, t.Functor(t.Pred(kVtOther), t.Functor(lShared)))});
        uint8_t* res = t.Resource({g0, g1, nullptr, g2});
        const uint64_t tags[] = {kSeat};
        const interact::GroupWalk w = interact::WalkGroupsTags(rd, t.o, res, tags, 1);
        Check(w.groups.size() == 2 && w.groups[0].group == g0 && w.groups[0].leaves.size() == 2 &&
                  w.groups[0].leaves[0] == lSeat && w.groups[1].group == g2 && w.groups[1].leaves.empty() &&
                  w.layers == 2 && w.LeafCount() == 2 && !w.readError,
              "list walk: seat leaves per group, trunk / puppet untouched, shared leaf kept in the first group");
        const loot::WalkResult flat = loot::WalkDescriptorTags(rd, t.o, res, tags, 1);
        Check(flat.leaves.size() == w.LeafCount(), "list walk: same leaves as the flat walk");
        Reader bad;
        bad.failAll = true;
        Check(interact::WalkGroupsTags(bad, t.o, res, tags, 1).readError, "list walk: read failure reported");
        // List eligibility: every LookAt leaf of all layers must be captured as part of the group.
        Check(!w.groups[0].otherLookAt && !w.groups[0].shared && interact::ListIneligible(w.groups[0], 2) == nullptr,
              "list guard: seat group with all LookAt leaves captured is eligible");
        Check(interact::ListIneligible(w.groups[0], 1) != nullptr,
              "list guard: a LookAt leaf not captured -> group not added");
        Check(w.groups[1].otherLookAt && w.groups[1].shared && interact::ListIneligible(w.groups[1], 0) != nullptr,
              "list guard: trunk LookAt + shared seat leaf -> group not added");
    }
    {
        // Seat layer with its own leaf plus a non-matching layer: LookAt there -> ineligible; no LookAt there -> eligible.
        Reader rd;
        Tree t;
        constexpr uint64_t kSeat = 0x6101, kTrunk = 0x6102, kPuppet = 0x6103;
        uint8_t* lA = t.Pred(kVtLookAt);
        uint8_t* lB = t.Pred(kVtLookAt);
        uint8_t* gA = t.Linked({t.Layer(kSeat, t.Functor(t.Pred(kVtOther), t.Functor(lA))),
                                t.Layer(kTrunk, t.Functor(t.Pred(kVtOther), t.Functor(t.Pred(kVtLookAt))))});
        uint8_t* gB = t.Linked({t.Layer(kPuppet, t.Functor(t.Pred(kVtOther))),
                                t.Layer(kSeat, t.Functor(t.Pred(kVtOther), t.Functor(lB)))});
        uint8_t* res = t.Resource({gA, gB});
        const uint64_t tags[] = {kSeat};
        const interact::GroupWalk w = interact::WalkGroupsTags(rd, t.o, res, tags, 1);
        const bool shape = w.groups.size() == 2 && w.groups[0].leaves.size() == 1 && w.groups[0].leaves[0] == lA &&
                           w.groups[1].leaves.size() == 1 && w.groups[1].leaves[0] == lB && w.layers == 2;
        const char* whyA = shape ? interact::ListIneligible(w.groups[0], 1) : nullptr;
        Check(shape && whyA && std::strcmp(whyA, "LookAt in a layer that is not morphed") == 0,
              "list guard: tag-matched layer morphed but another layer of the group has a LookAt -> not added",
              whyA ? whyA : "eligible");
        Check(shape && !w.groups[1].otherLookAt && interact::ListIneligible(w.groups[1], 1) == nullptr,
              "list guard: other layer without LookAt -> eligible");
        const loot::WalkResult flat = loot::WalkDescriptorTags(rd, t.o, res, tags, 1);
        Check(flat.leaves.size() == w.LeafCount() && flat.lootLayers == w.layers,
              "list guard: other-layer scan does not change the matched walk");
    }

    // Probe on both vanilla shapes.
    {
        ListFixture moto(4, {1, 2}); // motorcycle_device: PuppetClose / PuppetFar listed, seat g0 not
        const interact::ListProbe p = moto.Probe();
        Check(p.ok && p.size == 2 && p.cap == 2 && p.key == reinterpret_cast<uint64_t>(moto.comp) &&
                  p.groups.size() == 4 && p.list[0].inst == moto.G(1) && p.allocator == moto.allocVt,
              "list probe: motorcycle-like list (2/2, key = compiled, allocator after the elements)", p.why);
        ListFixture car(6, {}); // vehicle_device: nothing listed
        const interact::ListProbe q = car.Probe();
        Check(q.ok && q.size == 0 && q.cap == 0 && q.key == 0 && q.allocator == car.allocVt,
              "list probe: car-like empty list (key 0, allocator in entries)", q.why);
    }

    // Motorcycle: grow (cap == size), then append after minGap, then remove; reader race invariant over the whole log.
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        lp.managed = {f.G(0)};
        const std::vector<uint64_t> want = {f.G(0)};
        uint8_t* oldBuf = f.listBuf;
        const bool r0 = Step(f, lp, want, 0.0);
        interact::ListProbe p = f.Probe();
        Check(!r0 && lp.state == ListState::Grown && p.ok && p.size == 2 && p.cap == 3 &&
                  p.data != reinterpret_cast<uint64_t>(oldBuf) && f.m.lastAllocator == f.allocVt && f.Rc(1) == 4 &&
                  f.Rc(2) == 4 && lp.leaked.size() == 1,
              "list moto: grow = new buffer from the list's allocator, same members, copies own a reference", p.why);
        uint64_t oldFirst = 0;
        std::memcpy(&oldFirst, oldBuf, 8);
        Check(oldFirst == f.G(1), "list moto: old buffer left intact (never freed)");
        Step(f, lp, want, 0.05);
        Check(lp.state == ListState::Grown && f.Probe().size == 2, "list moto: no append before minGap");
        const bool r2 = Step(f, lp, want, 0.2);
        p = f.Probe();
        Check(r2 && lp.state == ListState::Added && p.ok && p.size == 3 && p.list[2].inst == f.G(0) && f.Rc(0) == 3 &&
                  p.key == reinterpret_cast<uint64_t>(f.comp) && !lp.keySet && lp.base == 2,
              "list moto: seat group appended after the vanilla members (ref owned, key kept)", p.why);
        const size_t elem = f.WriteIndex(p.data + 2 * 16, f.G(0));
        const size_t size3 = f.WriteIndex(f.A(f.L.list + 12), 3);
        Check(elem < size3 && size3 != SIZE_MAX, "list moto: element written before the size is raised");
        const size_t w = f.m.log.size();
        Step(f, lp, want, 0.3);
        Check(f.m.log.size() == w, "list moto: no writes while added with the same set");
        Check(interact::ListSummary(f.Probe(), lp).find("list 3/3 key=comp groups 4 [g1,g2,g0] managed [g0:in] patch added") !=
                  std::string::npos,
              "list moto: summary", interact::ListSummary(f.Probe(), lp));
        const bool r4 = Step(f, lp, {}, 0.4);
        p = f.Probe();
        uint64_t slot2 = 0;
        std::memcpy(&slot2, reinterpret_cast<const void*>(p.data + 32), 8);
        Check(r4 && lp.state == ListState::Vanilla && p.ok && p.size == 2 && p.list[0].inst == f.G(1) &&
                  p.list[1].inst == f.G(2) && slot2 == f.G(0) && f.Rc(0) == 3,
              "list moto: remove = vanilla members again, element stays valid beyond size", p.why);
        // Re-add: no grow, no second reference.
        const uint32_t allocs = f.m.allocs;
        Step(f, lp, want, 0.5);
        Check(lp.state == ListState::Added && f.Probe().size == 3 && f.m.allocs == allocs && f.Rc(0) == 3,
              "list moto: re-add reuses the slot (no grow, reference already owned)");
        Step(f, lp, {}, 0.6);
        std::string why;
        Check(f.RaceInvariant(why, gap), "list moto: reader race invariant (entries / size read at different moments)", why);
    }

    // Negative controls for the race check: size raised before the element, and append without the grace period.
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        Step(f, lp, {f.G(0)}, 0.0); // grow
        f.m.now = 0.2;
        const interact::ListProbe p = f.Probe();
        f.m.W32(f.A(f.L.list + 12), 3);
        f.m.W64(p.data + 32, f.G(0));
        f.m.W64(p.data + 40, reinterpret_cast<uint64_t>(f.rcs[0]));
        std::string why;
        Check(!f.RaceInvariant(why, gap), "list race check: size-before-element is detected (negative control)", why);
        ListFixture g(4, {1, 2});
        interact::ListPatch lq;
        Step(g, lq, {g.G(0)}, 0.0); // grow
        g.m.now = 0.0;              // append in the same instant
        const interact::ListProbe q = g.Probe();
        g.m.W64(q.data + 32, g.G(0));
        g.m.W64(q.data + 40, reinterpret_cast<uint64_t>(g.rcs[0]));
        g.m.W32(g.A(g.L.list + 12), 3);
        Check(!g.RaceInvariant(why, gap), "list race check: append without the grace period is detected (negative control)",
              why);
    }

    // Car: empty list (key 0, capacity 0) -> grow from the allocator, key set before the size, restored after.
    {
        ListFixture f(6, {});
        interact::ListPatch lp;
        lp.managed = {f.G(1), f.G(2)};
        const std::vector<uint64_t> want = {f.G(1), f.G(2)};
        Step(f, lp, want, 0.0);
        interact::ListProbe p = f.Probe();
        Check(lp.state == ListState::Grown && p.ok && p.cap == 2 && p.size == 0 && p.allocator == f.allocVt &&
                  lp.leaked.empty(),
              "list car: grow from capacity 0 stores the allocator after the new buffer", p.why);
        Step(f, lp, want, 0.2);
        p = f.Probe();
        const size_t keyW = f.WriteIndex(f.A(f.L.key), reinterpret_cast<uint64_t>(f.comp));
        const size_t sizeW = f.WriteIndex(f.A(f.L.list + 12), 2);
        Check(lp.state == ListState::Added && p.ok && p.size == 2 && lp.keySet && keyW < sizeW && sizeW != SIZE_MAX,
              "list car: both seat groups appended, key = compiled written before the size", p.why);
        Step(f, lp, {}, 0.3);
        p = f.Probe();
        const size_t size0 = f.WriteIndex(f.A(f.L.list + 12), 0, sizeW);
        const size_t key0 = f.WriteIndex(f.A(f.L.key), 0, keyW);
        Check(lp.state == ListState::Vanilla && p.ok && p.size == 0 && p.key == 0 && size0 < key0 && key0 != SIZE_MAX,
              "list car: remove = size 0 then key 0 (vanilla membership and key)", p.why);
        std::string why;
        Check(f.RaceInvariant(why, gap), "list car: reader race invariant", why);
        // Mode change while added (a different set): remove first, re-add on the next step.
        Step(f, lp, want, 0.4);
        const bool rr = Step(f, lp, {f.G(1)}, 0.5);
        Check(!rr && lp.state == ListState::Vanilla && f.Probe().size == 0, "list car: changed set -> removed first");
        Step(f, lp, {f.G(1)}, 0.6);
        Check(lp.state == ListState::Added && f.Probe().size == 1 && f.Probe().list[0].inst == f.G(1),
              "list car: then the new set appended");
    }

    // Layout proof failures -> foreign, nothing written.
    {
        auto foreignNoWrite = [&](ListFixture& f, const char* aName) {
            interact::ListPatch lp;
            lp.managed = {f.G(0)};
            Step(f, lp, {f.G(0)}, 0.0);
            Step(f, lp, {f.G(0)}, 0.5);
            Check(lp.state == ListState::Foreign && f.m.log.empty() && f.m.allocs == 0, aName, lp.why);
        };
        {
            ListFixture f(4, {1});
            Put32(f.comp, f.L.entryCount, 2);
            foreignNoWrite(f, "list proof: entry count != 1 -> foreign");
        }
        {
            ListFixture f(4, {1});
            uint64_t data;
            std::memcpy(&data, f.comp + f.L.groups, 8);
            PutPtr(reinterpret_cast<void*>(data), 16, f.m.arena.Alloc(8)); // compiled group 1 != definition group 1
            foreignNoWrite(f, "list proof: compiled groups differ from the definition -> foreign");
        }
        {
            ListFixture f(4, {1});
            Put64(f.comp, f.L.key, 0x1234);
            foreignNoWrite(f, "list proof: key neither 0 nor compiled -> foreign");
        }
        {
            ListFixture f(4, {});
            Put64(f.comp, f.L.list, reinterpret_cast<uint64_t>(f.m.arena.Alloc(16))); // allocator outside the image
            foreignNoWrite(f, "list proof: allocator not a game vtable -> foreign");
        }
        {
            ListFixture f(4, {1});
            PutPtr(f.listBuf, 0, f.m.arena.Alloc(8)); // list element that is not a group of this descriptor
            foreignNoWrite(f, "list proof: foreign list element -> foreign");
        }
        {
            ListFixture f(4, {1});
            Put64(f.comp, f.L.key, 0); // non-empty list without the key
            foreignNoWrite(f, "list proof: size > 0 with key 0 -> foreign");
            Check(std::strcmp(f.Probe().why, "list key 0 with a non-empty list") == 0, "list proof: key-0 reason named",
                  f.Probe().why);
        }
        {
            ListFixture f(4, {1});
            Put64(f.res, f.L.resCompiled, 0);
            foreignNoWrite(f, "list proof: no compiled data -> foreign");
        }
        {
            ListFixture f(4, {1, 2});
            f.m.failAlloc = true;
            interact::ListPatch lp;
            Step(f, lp, {f.G(0)}, 0.0);
            Check(lp.state == ListState::Foreign && f.m.log.empty(), "list: allocation failure -> foreign, nothing published",
                  lp.why);
        }
        {
            ListFixture f(4, {1, 2});
            interact::ListPatch lp;
            Step(f, lp, {0x42}, 0.0);
            Check(lp.state == ListState::Foreign && f.m.log.empty(), "list: wanted group not in the descriptor -> foreign");
        }
    }

    // Another writer changes the list while added -> foreign, no further writes; compiled rebuilt -> start over.
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        const std::vector<uint64_t> want = {f.G(0)};
        Step(f, lp, want, 0.0);
        Step(f, lp, want, 0.2);
        Put32(f.comp, f.L.list + 12, 2); // someone shrank it
        const size_t w = f.m.log.size();
        Step(f, lp, {}, 0.3);
        Check(lp.state == ListState::Foreign && f.m.log.size() == w && lp.pinned.empty() && !interact::Pins(lp, f.G(0)),
              "list: size shrunk by another writer -> foreign, untouched, not pinned (seat group no longer listed)",
              lp.why);

        ListFixture g(4, {1, 2});
        interact::ListPatch lq;
        const std::vector<uint64_t> wantG = {g.G(0)};
        Step(g, lq, wantG, 0.0);
        Step(g, lq, wantG, 0.2);
        const bool addedBefore = lq.state == ListState::Added;
        ListFixture h(4, {1, 2}); // a rebuilt compiled object for the same resource
        for (size_t i = 0; i < 4; ++i)
        {
            // same group instances: copy g's definition into h's compiled groups and point g's resource at h's compiled
            uint64_t gd, hd;
            std::memcpy(&gd, g.res + g.o.resDefinition + g.o.defLayers, 8);
            std::memcpy(&hd, h.comp + h.L.groups, 8);
            std::memcpy(reinterpret_cast<void*>(hd + i * 16), reinterpret_cast<const void*>(gd + i * 16), 16);
        }
        uint64_t gd;
        std::memcpy(&gd, g.res + g.o.resDefinition + g.o.defLayers, 8);
        uint64_t hl;
        std::memcpy(&hl, h.comp + h.L.list, 8);
        std::memcpy(reinterpret_cast<void*>(hl), reinterpret_cast<const void*>(gd + 16), 32); // members g1, g2
        PutPtr(h.comp, h.L.key, h.comp);
        PutPtr(g.res, g.L.resCompiled, h.comp);
        Put64(h.listBuf, 32, g.allocVt); // allocator of the rebuilt list = the game vtable of g's image
        Step(g, lq, wantG, 0.3);
        Check(addedBefore && lq.replaced == 1 && lq.state == ListState::Grown &&
                  lq.comp == reinterpret_cast<uint64_t>(h.comp),
              "list: compiled object rebuilt -> state reset, grows again on the new object", lq.why);
    }

    // Failure while added, restore branch: size / buffer / tail exactly as written -> vanilla size, then key.
    {
        ListFixture f(6, {});
        interact::ListPatch lp;
        const std::vector<uint64_t> want = {f.G(1), f.G(2)};
        Step(f, lp, want, 0.0);
        Step(f, lp, want, 0.2);
        const bool added = lp.state == ListState::Added && lp.keySet;
        const size_t w0 = f.m.log.size();
        Step(f, lp, {0x42}, 0.3); // a group of another descriptor -> foreign
        const interact::ListProbe p = f.Probe();
        const size_t sizeW = f.WriteIndex(f.A(f.L.list + 12), 0, w0);
        const size_t keyW = f.WriteIndex(f.A(f.L.key), 0, w0);
        Check(added && lp.state == ListState::Foreign && lp.pinned.empty() && !interact::Pins(lp, f.G(1)) && p.ok &&
                  p.size == 0 && p.key == 0 && sizeW < keyW && keyW != SIZE_MAX && f.m.log.size() == w0 + 2,
              "list leave: foreign from added restores size 0 then key 0, nothing pinned", lp.why);
        std::string why;
        Check(f.RaceInvariant(why, gap), "list leave: reader race invariant over the restore", why);
        Check(interact::ListSummary(p, lp).find("patch foreign") != std::string::npos &&
                  interact::ListSummary(p, lp).find("pinned") == std::string::npos,
              "list leave: summary without pinned", interact::ListSummary(p, lp));
    }
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        Step(f, lp, {f.G(0)}, 0.0);
        Step(f, lp, {f.G(0)}, 0.2);
        Put32(f.comp, f.L.entryCount, 2); // layout proof fails, object and our tail unchanged
        Step(f, lp, {}, 0.3);
        uint32_t size = 0;
        uint64_t key = 0;
        std::memcpy(&size, f.comp + f.L.list + 12, 4);
        std::memcpy(&key, f.comp + f.L.key, 8);
        Check(lp.state == ListState::Foreign && lp.pinned.empty() && size == 2 && key == reinterpret_cast<uint64_t>(f.comp),
              "list leave: probe failure while added -> vanilla size restored, vanilla key kept", lp.why);
    }
    // Failure while added, pinned branch: cannot be restored safely -> no write, group pinned (predicates stay morphed).
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        Step(f, lp, {f.G(0)}, 0.0);
        Step(f, lp, {f.G(0)}, 0.2);
        // Another writer: new buffer [g1, g2, g0, g3], size 4.
        uint8_t* nb = f.m.arena.Alloc(interact::ListBufferBytes(4));
        const int order[] = {1, 2, 0, 3};
        for (uint32_t k = 0; k < 4; ++k)
        {
            PutPtr(nb, k * 16, f.groups[order[k]]);
            PutPtr(nb, k * 16 + 8, f.rcs[order[k]]);
        }
        Put64(nb, static_cast<uint32_t>(interact::Align8(64)), f.allocVt);
        PutPtr(f.comp, f.L.list, nb);
        Put32(f.comp, f.L.list + 8, 4);
        Put32(f.comp, f.L.list + 12, 4);
        const size_t w = f.m.log.size();
        Step(f, lp, {}, 0.3);
        Check(lp.state == ListState::Foreign && f.m.log.size() == w && lp.pinned.size() == 1 &&
                  interact::Pins(lp, f.G(0)) && !interact::Pins(lp, f.G(3)) && f.Probe().size == 4,
              "list leave: another writer appended after ours -> untouched, seat group pinned", lp.why);
        Check(interact::ListSummary(f.Probe(), lp).find("pinned 1") != std::string::npos, "list leave: summary names pinned",
              interact::ListSummary(f.Probe(), lp));
        Step(f, lp, {}, 0.4);
        Check(f.m.log.size() == w && interact::Pins(lp, f.G(0)), "list leave: pinned foreign patch is never written again");
    }
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        Step(f, lp, {f.G(0)}, 0.0);
        Step(f, lp, {f.G(0)}, 0.2);
        Put64(f.res, f.L.resCompiled, 0); // compiled object no longer reachable from the resource
        const size_t w = f.m.log.size();
        Step(f, lp, {}, 0.3);
        uint32_t size = 0;
        std::memcpy(&size, f.comp + f.L.list + 12, 4);
        Check(lp.state == ListState::Foreign && f.m.log.size() == w && interact::Pins(lp, f.G(0)) && size == 3,
              "list leave: compiled object unreachable -> no write, seat group pinned", lp.why);
    }

    // Tail changed by another writer (same size): our group gone -> not pinned; our group moved below base -> pinned.
    {
        ListFixture f(4, {1, 2});
        interact::ListPatch lp;
        Step(f, lp, {f.G(0)}, 0.0);
        Step(f, lp, {f.G(0)}, 0.2);
        const uint64_t data = f.Probe().data;
        PutPtr(reinterpret_cast<void*>(data), 32, f.groups[3]); // slot 2: g0 -> g3
        PutPtr(reinterpret_cast<void*>(data), 40, f.rcs[3]);
        const size_t w = f.m.log.size();
        Step(f, lp, {}, 0.3);
        Check(lp.state == ListState::Foreign && f.m.log.size() == w && lp.pinned.empty() && f.Probe().size == 3,
              "list leave: tail replaced by another writer -> untouched, not pinned", lp.why);

        ListFixture g(4, {1, 2});
        interact::ListPatch lq;
        Step(g, lq, {g.G(0)}, 0.0);
        Step(g, lq, {g.G(0)}, 0.2);
        const uint64_t gd = g.Probe().data;
        PutPtr(reinterpret_cast<void*>(gd), 16, g.groups[0]); // [g1, g0, g2]
        PutPtr(reinterpret_cast<void*>(gd), 24, g.rcs[0]);
        PutPtr(reinterpret_cast<void*>(gd), 32, g.groups[2]);
        PutPtr(reinterpret_cast<void*>(gd), 40, g.rcs[2]);
        const size_t wg = g.m.log.size();
        Step(g, lq, {}, 0.3);
        Check(lq.state == ListState::Foreign && g.m.log.size() == wg && interact::Pins(lq, g.G(0)),
              "list leave: our group moved below the vanilla count -> untouched, pinned", lq.why);
    }

    // Re-add into a slot freed by a remove: the slot gets a different handle only minGap after the remove.
    {
        ListFixture f(6, {});
        interact::ListPatch lp;
        Step(f, lp, {f.G(1), f.G(2)}, 0.0);
        Step(f, lp, {f.G(1), f.G(2)}, 0.2);
        const uint64_t data = f.Probe().data;
        Step(f, lp, {f.G(2)}, 0.3); // changed set -> removed (size 0, slot 0 still g1)
        const bool removed = lp.state == ListState::Vanilla && f.Probe().size == 0;
        const size_t w = f.m.log.size();
        const bool r1 = Step(f, lp, {f.G(2)}, 0.35);
        uint64_t s0 = 0;
        std::memcpy(&s0, reinterpret_cast<const void*>(data), 8);
        Check(removed && !r1 && lp.state == ListState::Vanilla && f.m.log.size() == w && s0 == f.G(1),
              "list re-add: slot not rewritten within minGap of the remove", lp.why);
        const bool r2 = Step(f, lp, {f.G(2)}, 0.45);
        const interact::ListProbe p = f.Probe();
        Check(r2 && lp.state == ListState::Added && p.ok && p.size == 1 && p.list[0].inst == f.G(2) &&
                  p.list[0].rc == reinterpret_cast<uint64_t>(f.rcs[2]) && p.data == data,
              "list re-add: slot rewritten after minGap", p.why);
        std::string why;
        Check(f.RaceInvariant(why, gap), "list re-add: reader race invariant (old size against the rewritten slot)", why);

        ListFixture g(6, {});
        interact::ListPatch lq;
        Step(g, lq, {g.G(1), g.G(2)}, 0.0);
        Step(g, lq, {g.G(1), g.G(2)}, 0.2);
        g.m.now = 0.3;
        interact::StepList(g.m, g.L, g.o, g.Res(), lq, {g.G(2)}, 0.3, 0.0); // remove
        interact::StepList(g.m, g.L, g.o, g.Res(), lq, {g.G(2)}, 0.3, 0.0); // re-add in the same instant
        Check(lq.state == ListState::Added && !g.RaceInvariant(why, gap),
              "list re-add: rewrite in the same instant is detected (negative control)", why);
    }

    // Probe summary on a failed proof stays short and names the reason.
    {
        ListFixture f(4, {1});
        Put32(f.comp, f.L.entryCount, 3);
        interact::ListPatch lp;
        const std::string s = interact::ListSummary(f.Probe(), lp);
        Check(s.find("compiled entry count != 1") != std::string::npos && s.size() < 300, "list summary: failure reason", s);
    }
}

// ---------------------------------------------------------------------------------------------
// HeadGuard.

void TestPattern()
{
    hg::Pattern p;
    Check(hg::ParsePattern("F3 0F 10 05 ?? ?? ?? ??", p) && p.bytes.size() == 8 && p.bytes[4] == -1 && p.bytes[0] == 0xF3,
          "pattern: parse with wildcards");
    Check(hg::ParsePattern("0f 2f 05 ? ? ? ? 76", p) && p.bytes.size() == 8, "pattern: lower case + single ?");
    Check(!hg::ParsePattern("F3 0F", p), "pattern: too short rejected");
    Check(!hg::ParsePattern("?? ?? ?? ?? ?? 0F", p), "pattern: fewer than 4 fixed bytes rejected");
    Check(!hg::ParsePattern("F3 0G 10 05", p) && !hg::ParsePattern("F30F1005", p), "pattern: bad tokens rejected");
    const uint8_t data[] = {0x90, 0xF3, 0x0F, 0x10, 0x05, 1, 2, 3, 4, 0x90, 0xF3, 0x0F, 0x10, 0x0D, 5, 6, 7, 8};
    hg::ParsePattern("F3 0F 10 05 ?? ?? ?? ??", p);
    auto hits = hg::Scan(data, sizeof(data), p, 4);
    Check(hits.size() == 1 && hits[0] == 1, "scan: unique match");
    Check(hg::ParsePattern("90 F3 0F 10 ?? ??", p), "pattern: 4 fixed + wildcards");
    hits = hg::Scan(data, sizeof(data), p, 4);
    Check(hits.size() == 2 && hits[0] == 0 && hits[1] == 9, "scan: two matches reported (not unique)");
    hg::ParsePattern("F3 0F 11 05", p);
    Check(hg::Scan(data, sizeof(data), p, 4).empty(), "scan: no match");
}

void TestDecode()
{
    hg::RipInsn ri;
    const uint8_t movss[] = {0xF3, 0x0F, 0x10, 0x05, 0x10, 0x00, 0x00, 0x00, 0x90};
    Check(hg::DecodeSseRip(movss, sizeof(movss), ri) && ri.len == 8 && ri.dispPos == 4 && ri.disp == 0x10,
          "decode: movss xmm0,[rip+0x10]");
    const uint8_t movssRex[] = {0xF3, 0x44, 0x0F, 0x10, 0x0D, 0xF0, 0xFF, 0xFF, 0xFF};
    Check(hg::DecodeSseRip(movssRex, sizeof(movssRex), ri) && ri.len == 9 && ri.disp == -16, "decode: movss xmm9 (REX.R)");
    const uint8_t comiss[] = {0x0F, 0x2F, 0x05, 1, 0, 0, 0, 0x76};
    Check(hg::DecodeSseRip(comiss, sizeof(comiss), ri) && ri.len == 7 && ri.opcode == 0x2F, "decode: comiss xmm0,[rip]");
    const uint8_t ucomiss[] = {0x0F, 0x2E, 0x0D, 1, 0, 0, 0};
    Check(hg::DecodeSseRip(ucomiss, sizeof(ucomiss), ri) && ri.opcode == 0x2E, "decode: ucomiss xmm1,[rip]");
    const uint8_t mulss[] = {0xF3, 0x0F, 0x59, 0x15, 1, 0, 0, 0};
    Check(hg::DecodeSseRip(mulss, sizeof(mulss), ri), "decode: mulss xmm2,[rip]");
    const uint8_t vmovss[] = {0xC5, 0xFA, 0x10, 0x05, 4, 0, 0, 0};
    Check(hg::DecodeSseRip(vmovss, sizeof(vmovss), ri) && ri.vex && ri.len == 8, "decode: vmovss (VEX)");
    const uint8_t vcomiss[] = {0xC5, 0xF8, 0x2F, 0x05, 4, 0, 0, 0};
    Check(hg::DecodeSseRip(vcomiss, sizeof(vcomiss), ri) && ri.vex, "decode: vcomiss (VEX)");
    const uint8_t movsd[] = {0xF2, 0x0F, 0x10, 0x05, 1, 0, 0, 0};
    Check(!hg::DecodeSseRip(movsd, sizeof(movsd), ri), "decode: movsd (double) rejected");
    const uint8_t comisd[] = {0x66, 0x0F, 0x2F, 0x05, 1, 0, 0, 0};
    Check(!hg::DecodeSseRip(comisd, sizeof(comisd), ri), "decode: comisd (double) rejected");
    const uint8_t rexw[] = {0xF3, 0x48, 0x0F, 0x10, 0x05, 1, 0, 0, 0};
    Check(!hg::DecodeSseRip(rexw, sizeof(rexw), ri), "decode: REX.W rejected");
    const uint8_t stack[] = {0xF3, 0x0F, 0x10, 0x44, 0x24, 0x20, 0x90, 0x90};
    Check(!hg::DecodeSseRip(stack, sizeof(stack), ri), "decode: [rsp+disp8] (not RIP) rejected");
    const uint8_t store[] = {0xF3, 0x0F, 0x11, 0x05, 1, 0, 0, 0};
    Check(!hg::DecodeSseRip(store, sizeof(store), ri), "decode: movss store (writes memory) rejected");
    const uint8_t vexL[] = {0xC5, 0xFE, 0x10, 0x05, 1, 0, 0, 0};
    Check(!hg::DecodeSseRip(vexL, sizeof(vexL), ri), "decode: VEX.L=1 rejected");
    Check(!hg::DecodeSseRip(movss, 6, ri), "decode: truncated buffer rejected");
}

void TestPatch()
{
    alignas(16) uint8_t buf[64];
    for (int off = 0; off < 16; ++off)
    {
        std::memset(buf, 0xAB, sizeof(buf));
        const uint32_t oldv = 0x11223344u, newv = 0xA1B2C3D4u;
        std::memcpy(buf + 16 + off, &oldv, 4);
        const hg::PatchResult r = hg::Patch4(buf + 16 + off, oldv, newv);
        uint32_t now;
        std::memcpy(&now, buf + 16 + off, 4);
        bool othersIntact = true;
        for (int i = 0; i < 64; ++i)
            if ((i < 16 + off || i >= 20 + off) && buf[i] != 0xAB)
                othersIntact = false;
        const bool expectOk = off <= 12 || (off % 8) <= 4;
        char name[96];
        std::snprintf(name, sizeof(name), "patch4: offset %d -> %s", off, expectOk ? "ok, neighbours intact" : "refused (unaligned)");
        if (expectOk)
            Check(r == hg::PatchResult::Ok && now == newv && othersIntact, name);
        else
            Check(r == hg::PatchResult::Unaligned && now == oldv, name);
    }
    alignas(16) uint8_t b2[16] = {};
    const uint32_t v = 5;
    std::memcpy(b2 + 4, &v, 4);
    Check(hg::Patch4(b2 + 4, 6, 7) == hg::PatchResult::Mismatch, "patch4: expected-old mismatch refused");
}

void TestFlow()
{
    // Fake image: code at 0x100 "movss xmm0,[rip+X]" reading a constant 0.35 at 0x800; UTPP cell at 0xC00.
    alignas(16) static uint8_t img[0x1000];
    std::memset(img, 0xCC, sizeof(img));
    const float threshold = 0.35f, tppValue = 0.05f;
    std::memcpy(img + 0x800, &threshold, 4);
    std::memcpy(img + 0xC00, &tppValue, 4);
    const size_t insnAt = 0x103;
    const uint8_t pre[] = {0x48, 0x8B, 0xC1}; // mov rax, rcx
    std::memcpy(img + 0x100, pre, 3);
    img[insnAt] = 0xF3;
    img[insnAt + 1] = 0x0F;
    img[insnAt + 2] = 0x10;
    img[insnAt + 3] = 0x05;
    int32_t disp = static_cast<int32_t>(0x800 - (insnAt + 8));
    std::memcpy(img + insnAt + 4, &disp, 4);
    img[insnAt + 8] = 0x0F;
    img[insnAt + 9] = 0x2F;
    img[insnAt + 10] = 0xC1; // comiss xmm0, xmm1
    uint8_t original[0x1000];
    std::memcpy(original, img, sizeof(img));

    hg::Site site{"48 8B C1 F3 0F 10 05 ?? ?? ?? ?? 0F 2F C1", 3, 0.35f, 0.05f};
    hg::Pattern p;
    Check(hg::ParsePattern(site.pattern, p), "flow: pattern");
    const auto hits = hg::Scan(img, sizeof(img), p, 2);
    Check(hits.size() == 1 && hits[0] == 0x100, "flow: unique match");
    hg::RipInsn ri;
    const bool dec = hg::DecodeSseRip(img + hits[0] + site.insnOffset, 16, ri);
    const size_t target = hits[0] + site.insnOffset + ri.len + ri.disp;
    float cur;
    std::memcpy(&cur, img + target, 4);
    Check(dec && target == 0x800 && hg::FloatMatches(cur, site.expected), "flow: operand resolves to the expected constant");
    hg::Site wrong = site;
    wrong.expected = 0.5f;
    Check(!hg::FloatMatches(cur, wrong.expected), "flow: wrong expected value -> inert");
    int32_t rel = 0;
    const uint64_t insnAddr = reinterpret_cast<uint64_t>(img + insnAt);
    Check(hg::Rel32(insnAddr, ri.len, reinterpret_cast<uint64_t>(img + 0xC00), rel), "flow: cell reachable");
    uint32_t o32, n32;
    std::memcpy(&o32, &ri.disp, 4);
    std::memcpy(&n32, &rel, 4);
    Check(hg::Patch4(img + insnAt + ri.dispPos, o32, n32) == hg::PatchResult::Ok, "flow: apply");
    hg::RipInsn after;
    hg::DecodeSseRip(img + insnAt, 16, after);
    const size_t newTarget = insnAt + after.len + after.disp;
    float seen;
    std::memcpy(&seen, img + newTarget, 4);
    Check(newTarget == 0xC00 && seen == tppValue && std::memcmp(img + 0x800, &threshold, 4) == 0,
          "flow: instruction now reads the UTPP cell; shared constant untouched");
    Check(hg::Patch4(img + insnAt + ri.dispPos, n32, o32) == hg::PatchResult::Ok && std::memcmp(img, original, sizeof(img)) == 0,
          "flow: restore -> image byte-identical to the original");
    Check(!hg::Rel32(0x100000000000ull, 8, 0x100000000000ull + 0x90000000ull, rel), "flow: target beyond +-2 GB refused");
}

void TestHgReasons()
{
    hg::Inputs in;
    in.siteReady = in.enabled = in.tpp = in.validated = true;
    Check(hg::WantPatched(in), "hg reasons: all good -> patched");
    hg::Inputs noSite = in;
    noSite.siteReady = false;
    Check(!hg::WantPatched(noSite) && hg::Reasons(noSite) == hg::kNoSite, "hg reasons: 2.31 default (no site) -> inert");
    hg::Inputs fpp = in;
    fpp.tpp = false;
    Check(!hg::WantPatched(fpp), "hg reasons: FPP -> original threshold");
    hg::Inputs failed = in;
    failed.patchFailed = true;
    Check(!hg::WantPatched(failed), "hg reasons: patch failure latched -> off");
    Check(hg::FloatMatches(0.35f, 0.35f) && !hg::FloatMatches(std::nanf(""), 0.35f) && hg::FloatMatches(1000.0f, 1000.0001f),
          "hg: float match (bitwise / 1e-6 relative / NaN)");
}
} // namespace

int main(int argc, char** argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const bool negative = argc > 1 && std::strcmp(argv[1], "--negative") == 0;
    if (negative)
    {
        // Wrong expectation: claims a LookAt-before-payload order is safe. Must fail.
        loot::WindowParams w;
        std::vector<WriteRec> wrong = {{nullptr, 0x30, loot::WindowQ30(w)}, {nullptr, 0x00, kVtWindow}};
        std::string why;
        Check(InvariantHolds(MakeLookAt(), wrong, w, why), "NEGATIVE: payload written while LookAt vtable active", why);
        std::printf("NEGATIVE pass=%d fail=%d\n", g_pass, g_fail);
        return g_fail == 0 ? 0 : 1;
    }
    TestGate();
    TestWindow();
    TestSlots();
    TestWalk();
    TestLootReasons();
    TestInteract();
    TestInteractList();
    TestPattern();
    TestDecode();
    TestPatch();
    TestFlow();
    TestHgReasons();
    std::printf("utpp_loothead_test pass=%d fail=%d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// ragdoll production core: the keep store (src/ragdoll/RdKeep.hpp). Offline, plain C++: no game, no SEH.
// Mutations (build with -DUTPP_RAGDOLL_KEEP_MUTATION=k, CMake utpp_ragdoll_keep_mut1/2/3):
//   1 the serial check is gone              -> "FAIL stale serial"
//   2 FK never closes the evaluation        -> "FAIL close evaluation"
//   3 no age cap                            -> "FAIL age cap"
//   4 a stale open slot is never reclaimed  -> "FAIL aged slot is reclaimed"
//   5 FK that loses the try-lock leaves the slot valid -> "FAIL a poisoned slot is reusable at once"
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "../src/ragdoll/RdKeep.hpp"

using utpp::ragdoll::RdKeep;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
}

constexpr uint64_t kDescA = 0x1F6DEAFA640, kDescB = 0x1F6E0D14AD8;

// Fills n records: dst = base + i, every byte of the record = tag (so one serial/tag per restored buffer is checkable).
uint32_t FillTag(uint32_t aN, uint8_t aTag, uint32_t (&aDst)[RdKeep::kMaxBones], uint8_t (&aData)[RdKeep::kMaxBones][RdKeep::kStride])
{
    for (uint32_t i = 0; i < aN; ++i)
    {
        aDst[i] = 10 + i;
        std::memset(aData[i], aTag, RdKeep::kStride);
    }
    return aN;
}

bool Save(RdKeep& k, uint64_t aDesc, uint8_t aTag, int64_t aNow = 0, uint32_t aThread = 1, uint32_t aN = 8, uint64_t* aSerial = nullptr)
{
    return k.Save(aDesc, aNow, aThread, [&](uint32_t (&d)[RdKeep::kMaxBones], uint8_t (&b)[RdKeep::kMaxBones][RdKeep::kStride]) {
        return FillTag(aN, aTag, d, b);
    }, aSerial);
}

// Restore into a local buffer; returns the tag of the first record (0 = nothing restored) and whether all records agree.
struct Got
{
    bool ok = false;
    uint8_t tag = 0;
    bool consistent = true;
    uint32_t n = 0;
};
Got Restore(RdKeep& k, uint64_t aDesc, int64_t aNow = 0, uint32_t aThread = 1, uint64_t aExpect = 0)
{
    Got g;
    g.ok = k.Restore(aDesc, aNow, aThread, [&](uint32_t aN, const uint32_t (&d)[RdKeep::kMaxBones],
                                                 const uint8_t (&b)[RdKeep::kMaxBones][RdKeep::kStride]) {
        g.n = aN;
        g.tag = aN ? b[0][0] : 0;
        for (uint32_t i = 0; i < aN; ++i)
        {
            if (d[i] != 10 + i)
                g.consistent = false;
            for (uint32_t j = 0; j < RdKeep::kStride; ++j)
                if (b[i][j] != g.tag)
                    g.consistent = false;
        }
    }, aExpect);
    return g;
}

std::unique_ptr<RdKeep> Make(int64_t aMaxAge = 0)
{
    auto k = std::make_unique<RdKeep>();
    k->SetMaxAge(aMaxAge);
    return k;
}

void TestBasic()
{
    auto k = Make();
    Check(!Restore(*k, kDescA).ok, "restore without save is skipped");
    Check(Save(*k, kDescA, 0x11), "save");
    const Got g = Restore(*k, kDescA);
    Check(g.ok && g.tag == 0x11 && g.n == 8 && g.consistent, "restore returns the saved records");
    const Got g2 = Restore(*k, kDescA);
    Check(g2.ok && g2.tag == 0x11, "several later merges restore the same evaluation");
    const auto c = k->Snapshot();
    Check(c.saves == 1 && c.restores == 2 && c.faults == 0 && c.lastBones == 8, "counters saves/restores/bones",
          "saves=" + std::to_string(c.saves) + " restores=" + std::to_string(c.restores));
    Check(!Restore(*k, 0x1234).ok, "unknown descriptor is skipped");
    Check(!k->Save(0, 0, 1, [&](auto& d, auto& b) { return FillTag(1, 1, d, b); }), "descriptor 0 is rejected");
}

void TestTwoDescriptors()
{
    auto k = Make();
    Save(*k, kDescA, 0xA1);
    Save(*k, kDescB, 0xB2);
    Check(Restore(*k, kDescA).tag == 0xA1 && Restore(*k, kDescB).tag == 0xB2, "trackA and trackB descriptors are independent");
    k->CloseEvaluation(kDescA);
    Check(!Restore(*k, kDescA).ok && Restore(*k, kDescB).tag == 0xB2, "FK of one descriptor does not close the other");
    Save(*k, kDescA, 0xA3);
    Check(Restore(*k, kDescA).tag == 0xA3 && Restore(*k, kDescB).tag == 0xB2, "a new save into A leaves B untouched");
}

void TestStaleSerial()
{
    auto k = Make();
    uint64_t s1 = 0, s2 = 0;
    Save(*k, kDescA, 0x01, 0, 1, 8, &s1);
    Save(*k, kDescA, 0x02, 0, 1, 8, &s2);
    Check(s2 > s1, "serials grow");
    const bool oldRestored = Restore(*k, kDescA, 0, 1, s1).ok;
    Check(!oldRestored, "stale serial");
    const Got fresh = Restore(*k, kDescA, 0, 1, s2);
    Check(fresh.ok && fresh.tag == 0x02, "matching serial restores");
    Check(Restore(*k, kDescA).tag == 0x02, "no serial restores the open evaluation");
    Check(k->Snapshot().skipSerial >= 1, "the serial skip is counted");
}

void TestSaveInProgress()
{
    auto k = Make();
    Save(*k, kDescA, 0x21);
    RdKeep::Slot* s = k->BeginSave(kDescA); // a save is in flight: the old evaluation is gone and the lock is held
    Check(s != nullptr, "second save begins");
    Check(!Restore(*k, kDescA).ok, "restore during a save in progress is skipped (no torn / old data)");
    std::memset(s->data[0], 0x22, RdKeep::kStride);
    s->dst[0] = 10;
    k->CommitSave(s, 1, 0, 1);
    const Got g = Restore(*k, kDescA);
    Check(g.ok && g.tag == 0x22 && g.n == 1, "after the commit the new evaluation restores");
    // a save that loses the lock against a restore in flight poisons the slot
    RdKeep::Slot* r = k->BeginRestore(kDescA, 0, 1);
    Check(r != nullptr, "restore holds the slot");
    Check(k->BeginSave(kDescA) == nullptr, "save against a restore in flight is skipped");
    k->EndRestore(r);
    Check(!Restore(*k, kDescA).ok, "poisoned slot restores nothing until the next commit");
    Check(Save(*k, kDescA, 0x23) && Restore(*k, kDescA).tag == 0x23, "next committed save clears the poison");
}

void TestClose()
{
    auto k = Make();
    Save(*k, kDescA, 0x31);
    k->CloseEvaluation(kDescA);
    Check(!Restore(*k, kDescA).ok, "close evaluation");
    Save(*k, kDescA, 0x32);
    RdKeep::Slot* r = k->BeginRestore(kDescA, 0, 1);
    k->CloseEvaluation(kDescA); // a restore in flight: the slot is poisoned instead
    k->EndRestore(r);
    Check(!Restore(*k, kDescA).ok, "FK against a restore in flight poisons the evaluation");
    Save(*k, kDescB, 0x33);
    k->InvalidateAll();
    Check(!Restore(*k, kDescB).ok, "InvalidateAll stops every restore");
    Check(Save(*k, kDescB, 0x34) && Restore(*k, kDescB).tag == 0x34, "a committed save after InvalidateAll works again");
}

void TestAgeCap()
{
    auto k = Make(100);
    Save(*k, kDescA, 0x41, 1000);
    Check(Restore(*k, kDescA, 1050).ok, "restore inside the age cap");
    const bool late = Restore(*k, kDescA, 1200).ok;
    Check(!late, "age cap");
    Check(k->Snapshot().skipStale == 1, "stale skip counted");
    Check(!Restore(*k, kDescA, 1060).ok, "an aged-out evaluation stays closed");
    Save(*k, kDescA, 0x42, 5000);
    Check(!Restore(*k, kDescA, 4000).ok, "a clock that went backwards is not trusted");
}

void TestFaults()
{
    auto k = Make();
    const bool saved = k->Save(kDescA, 0, 1, [&](auto&, auto&) -> uint32_t { throw std::runtime_error("torn"); });
    Check(!saved && k->Snapshot().faults == 1, "a faulting save counts a fault and keeps nothing");
    Check(!Restore(*k, kDescA).ok, "a torn save is never written back");
    Save(*k, kDescA, 0x51);
    const bool restored = k->Restore(kDescA, 0, 1, [&](uint32_t, const auto&, const auto&) { throw std::runtime_error("write"); });
    Check(!restored && k->Snapshot().faults == 2, "a faulting restore counts a fault");
    Check(!Restore(*k, kDescA).ok, "a faulting restore drops the evaluation");
    Check(Save(*k, kDescA, 0x52) && Restore(*k, kDescA).tag == 0x52, "the lock is not leaked by faults");
    RdKeep::Slot* s = k->BeginSave(kDescA);
    k->AbortSave(s); // SEH path of PoseHook
    Check(!Restore(*k, kDescA).ok && k->Snapshot().faults == 3, "AbortSave keeps nothing");
    Check(Save(*k, kDescA, 0x53), "slot usable after AbortSave");
}

void TestTableFull()
{
    auto k = Make();
    for (uint32_t i = 0; i < RdKeep::kSlots; ++i)
        Check(Save(*k, 0x1000 + i, uint8_t(i + 1)), "fill slot " + std::to_string(i));
    Check(!Save(*k, 0x2000, 0x77), "a fifth live descriptor is skipped");
    Check(k->Snapshot().skipFull == 1, "full skip counted");
    k->CloseEvaluation(0x1001);
    Check(Save(*k, 0x2000, 0x78) && Restore(*k, 0x2000).tag == 0x78, "a closed slot is reused");
    Check(Restore(*k, 0x1000).tag == 1 && Restore(*k, 0x1002).tag == 3, "the other evaluations survive the reuse");
}

void TestStaleSlotReclaim()
{
    // (a) four descriptors saved, their FK never comes (freed / replaced descriptors): after the age cap a new one gets a slot
    auto k = Make(100);
    for (uint32_t i = 0; i < RdKeep::kSlots; ++i)
        Save(*k, 0x1000 + i, uint8_t(i + 1), 1000);
    Check(!Save(*k, 0x2000, 0x77, 1050), "a young open slot is not reclaimed");
    Check(k->Snapshot().skipFull == 1, "full skip counted before the age cap");
    const bool reclaimed = Save(*k, 0x2000, 0x78, 1200);
    Check(reclaimed && Restore(*k, 0x2000, 1210).tag == 0x78, "aged slot is reclaimed", "reclaimed=" + std::to_string(reclaimed));

    // the one-argument BeginSave follows the latest time the store has seen (the game call site passes no clock)
    auto m = Make(100);
    for (uint32_t i = 0; i < RdKeep::kSlots; ++i)
        Save(*m, 0x1000 + i, uint8_t(i + 1), 1000);
    Restore(*m, 0x9999, 1300); // a restore of an unknown descriptor only advances the store's clock
    RdKeep::Slot* s = m->BeginSave(0x3000);
    Check(s != nullptr, "BeginSave without a clock reclaims by the latest seen time");
    if (s)
        m->AbortSave(s);

    // (b) FK lost the try-lock against a restore in flight: the slot is poisoned and must not park until the episode ends
    auto p = Make(100);
    for (uint32_t i = 0; i < RdKeep::kSlots; ++i)
        Save(*p, 0x1000 + i, uint8_t(i + 1), 1000);
    RdKeep::Slot* r = p->BeginRestore(0x1002, 1010, 1);
    p->CloseEvaluation(0x1002); // busy: poisoned
    p->EndRestore(r);
    Check(!Restore(*p, 0x1002, 1020).ok, "poisoned slot restores nothing");
    const bool reused = Save(*p, 0x4000, 0x44, 1030); // the other three are young and valid: only the poisoned slot can be taken
    Check(reused, "a poisoned slot is reusable at once", "reused=" + std::to_string(reused));
    Check(Restore(*p, 0x4000, 1040).tag == 0x44, "and carries the new evaluation");
}

void TestThreadInstrumentation()
{
    auto k = Make();
    Save(*k, kDescA, 0x61, 0, 7);
    Restore(*k, kDescA, 0, 7);
    Restore(*k, kDescA, 0, 7);
    Restore(*k, kDescA, 0, 9);
    const auto c = k->Snapshot();
    Check(c.threadSame == 2 && c.threadDiff == 1, "thread same / diff counted",
          "same=" + std::to_string(c.threadSame) + " diff=" + std::to_string(c.threadDiff));
    Check(c.lastSaveThread == 7 && c.lastRestoreThread == 9, "last save / restore thread ids recorded");
}

// Two real threads, each running frames on its own descriptor (save, three restores, FK close), plus a hammer thread that restores
// on both descriptors and a saver that races a second save into descriptor A. Whatever interleaving happens: every restored buffer
// carries exactly one serial, never a torn mix, and never the data of the other descriptor.
void TestThreads()
{
    auto k = Make();
    constexpr int kFrames = 20000;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> torn{0}, crossed{0}, restored{0}, sinks{0};
    // tag of a frame = 1..250; descriptor id folded into the high bit so that crossing is detectable
    auto frame = [&](uint64_t aDesc, uint8_t aBase, uint32_t aTid) {
        for (int f = 0; f < kFrames; ++f)
        {
            const uint8_t tag = uint8_t(aBase | (1 + (f % 100)));
            k->Save(aDesc, 0, aTid, [&](auto& d, auto& b) { return FillTag(40, tag, d, b); });
            for (int m = 0; m < 3; ++m)
            {
                const Got g = Restore(*k, aDesc, 0, aTid);
                if (g.ok)
                {
                    ++restored;
                    if (!g.consistent)
                        ++torn;
                    if ((g.tag & 0x80) != (aBase & 0x80))
                        ++crossed;
                }
            }
            k->CloseEvaluation(aDesc);
        }
    };
    std::thread ta(frame, kDescA, uint8_t(0x00), 100u);
    std::thread tb(frame, kDescB, uint8_t(0x80), 200u);
    std::thread hammer([&] {
        while (!stop.load())
            for (uint64_t desc : {kDescA, kDescB})
            {
                const Got g = Restore(*k, desc, 0, 300);
                if (g.ok)
                {
                    ++sinks;
                    if (!g.consistent)
                        ++torn;
                    if ((g.tag & 0x80) != (desc == kDescB ? 0x80 : 0))
                        ++crossed;
                }
            }
    });
    std::thread racer([&] {
        for (int f = 0; f < kFrames; ++f)
            k->Save(kDescA, 0, 400, [&](auto& d, auto& b) { return FillTag(40, uint8_t(1 + (f % 100)), d, b); });
    });
    ta.join();
    tb.join();
    racer.join();
    stop.store(true);
    hammer.join();
    Check(torn.load() == 0, "no torn restore under two interleaving threads", "torn=" + std::to_string(torn.load()));
    Check(crossed.load() == 0, "no restore carries the other descriptor's data", "crossed=" + std::to_string(crossed.load()));
    Check(restored.load() > 0, "threads restored something", "restored=" + std::to_string(restored.load()));
    const auto c = k->Snapshot();
    Check(c.faults == 0, "no faults under threads");
    std::printf("INFO threads: saves=%llu restores=%llu skips=%llu (busy=%llu poison=%llu none=%llu) threadDiff=%llu\n",
                static_cast<unsigned long long>(c.saves), static_cast<unsigned long long>(c.restores),
                static_cast<unsigned long long>(c.skips), static_cast<unsigned long long>(c.skipBusy),
                static_cast<unsigned long long>(c.skipPoison), static_cast<unsigned long long>(c.skipNoSlot),
                static_cast<unsigned long long>(c.threadDiff));
}
} // namespace

int main()
{
    TestBasic();
    TestTwoDescriptors();
    TestStaleSerial();
    TestSaveInProgress();
    TestClose();
    TestAgeCap();
    TestFaults();
    TestTableFull();
    TestStaleSlotReclaim();
    TestThreadInstrumentation();
    TestThreads();
    std::printf("RagdollKeepTest: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

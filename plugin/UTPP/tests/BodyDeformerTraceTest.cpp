#include "../src/mm/BodyDeformerTrace.hpp"
#include <cstdlib>
#include <cmath>
#include <memory>
#include <new>
#include <thread>
#include <vector>

static std::atomic<unsigned> allocationCount{0};
void* operator new(std::size_t size)
{
    allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(size ? size : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

#define CHECK(condition) do { if (!(condition)) std::abort(); } while (false)

int main()
{
    using namespace utpp::mm::bodytrace;
    CHECK(kNames.size() == kMaxBones && kMaxBones == 185);
    std::vector<std::string_view> fullCatalog(kNames.begin(), kNames.end());
    Map fullMap;
    CHECK(Resolve(fullCatalog, 456, fullMap));
    CHECK(fullMap.rigCount == kMaxBones);
    for (uint32_t i = 0; i < kMaxBones; ++i) CHECK(fullMap.index[i] == static_cast<int32_t>(i));
    CHECK(kNames[173] == "Head" && kNames[176] == "Neck1" &&
          kNames[179] == "WeaponLeft" && kNames[180] == "WeaponRight" && kNames[184] == "reference_joint");

    auto capture = std::make_unique<Capture>(); // large fixed ring kept off the test's stack
    auto postCapture = std::make_unique<PostHelperCapture>();
    std::vector<std::string_view> names{"Spine3", "LeftShoulder", "l_scapula_A_top_out_JNT"};
    Map map;
    CHECK(Resolve(names, 123, map));
    CHECK(map.rigCount == 3 && map.index[0] == 0 && map.index[1] == 1 && map.index[11] == 2);
    names.push_back("Spine3");
    Map duplicate;
    CHECK(!Resolve(names, 123, duplicate));
    float bones[3][12]{};
    bones[0][0] = 1; bones[1][0] = 2; bones[2][0] = 3;
    Map bad = map; bad.index[0] = 3;
    CHECK(!capture->Arm(bad));
    CHECK(!capture->Arm(map, 201));
    CHECK(capture->Arm(map, 2));
    CHECK(!capture->Arm(map, 1));
    Frame frame, out;
    auto ptr = reinterpret_cast<uint8_t*>(bones);
    CHECK(!capture->Begin(frame, ptr, 2, 123, 77, 1, 1));
    CHECK(!capture->Begin(frame, ptr, 3, 124, 77, 1, 1));
    const unsigned allocationBefore = allocationCount.load();
    CHECK(capture->Begin(frame, ptr, 3, 123, 77, 1, 1));
    bones[0][0] = 10;
    CHECK(!capture->End(frame, ptr, 3, 123, 78));
    CHECK(capture->End(frame, ptr, 3, 123, 77));
    CHECK(allocationCount.load() == allocationBefore);
    CHECK(capture->Pop(out));
    CHECK(out.stage[0][0][0] == 1 && out.stage[1][0][0] == 10);
    CHECK(out.stage[0][11][0] == 3 && out.stage[1][11][0] == 3);
    CHECK(std::isnan(out.stage[0][2][0]));
    CHECK(!capture->Pop(out));

    // Separate final-stage capture records only an explicit post-commit call;
    // it carries the same FK identity/ticks for joining without changing the
    // legacy two-stage Frame or CSV contract.
    CHECK(postCapture->Arm(map, 2));
    const unsigned postAllocationBefore = allocationCount.load();
    CHECK(postCapture->Capture(ptr, 3, 123, 77, 101, 1, 9, 22, 95, 100));
    CHECK(allocationCount.load() == postAllocationBefore);
    PostHelperFrame postOut;
    CHECK(postCapture->Pop(postOut));
    CHECK(postOut.rig == 123 && postOut.desc == 77 && postOut.ticks == 101 && postOut.thread == 1);
    CHECK(postOut.receiptGeneration == 9 && postOut.receiptCycle == 22);
    CHECK(postOut.receiptStartedTicks == 95 && postOut.receiptLastTicks == 100);
    CHECK(postOut.index[0] == 0 && postOut.index[11] == 2);
    CHECK(postOut.local[0][0] == bones[0][0] && postOut.local[11][0] == bones[2][0]);
    CHECK(std::isnan(postOut.local[2][0]));
    CHECK(!postCapture->Capture(ptr, 3, 123, 77, 100, 1, 9, 23, 96, 101)); // receipt newer than FK tick
    CHECK(postCapture->Capture(ptr, 3, 123, 77, 102, 1, 9, 23, 96, 101));
    CHECK(!postCapture->Active());
    CHECK(!postCapture->Capture(ptr, 3, 123, 77, 103, 1, 9, 24, 97, 102));
    CHECK(postCapture->Pop(postOut) && postOut.receiptCycle == 23);
    CHECK(!postCapture->Pop(postOut));
    CHECK(postCapture->Arm(map, 1));
    CHECK(postCapture->Cancel());
    CHECK(!postCapture->Capture(ptr, 3, 123, 77, 104, 1, 9, 25, 98, 103));
    CHECK(!postCapture->Pop(postOut));

    CHECK(capture->Begin(frame, ptr, 3, 123, 77, 2, 1));
    CHECK(capture->End(frame, ptr, 3, 123, 77));
    CHECK(!capture->Active());
    CHECK(!capture->Arm(map)); // drain first
    CHECK(capture->Pop(out));
    CHECK(capture->Arm(map, 1));
    CHECK(capture->Begin(frame, ptr, 3, 123, 77, 3, 1));
    CHECK(capture->Cancel());
    CHECK(capture->Arm(map, 1));
    CHECK(!capture->End(frame, ptr, 3, 123, 77)); // prior generation cannot publish
    CHECK(capture->Cancel());

    // Concurrent producers may drop, but every consumed pair must be internally complete.
    CHECK(capture->Arm(map, 200));
    std::thread a([&] { for (int i = 0; i < 1000; ++i) { Frame f;
        if (capture->Begin(f, ptr, 3, 123, 77, i + 1, 1)) capture->End(f, ptr, 3, 123, 77); } });
    std::thread b([&] { for (int i = 0; i < 1000; ++i) { Frame f;
        if (capture->Begin(f, ptr, 3, 123, 88, i + 1, 2)) capture->End(f, ptr, 3, 123, 88); } });
    a.join(); b.join();
    unsigned consumed = 0;
    while (capture->Pop(out)) { ++consumed; CHECK(out.stage[0][0][0] == 10);
        CHECK(out.stage[1][0][0] == 10); CHECK(out.desc == (out.thread == 1 ? 77 : 88)); }
    CHECK(consumed > 0 && consumed <= 200);
    return 0;
}

// Storage/lifetime tests only. Synthetic hash values with the observed
// 152-source/619-destination dimensions are NOT runtime semantic evidence.
#include "../src/mm/SemanticPoseDiscovery.hpp"
#include <cstdio>
#include <memory>
#include <vector>

using namespace utpp::mm::semanticpose;
static unsigned checks = 0, failures = 0;
static void Check(bool aOk, const char* aName)
{
    ++checks;
    if (!aOk) ++failures;
    std::printf("[%s] %s\n", aOk ? "PASS" : "FAIL", aName);
}

int main()
{
    auto capture = std::make_unique<Capture>();
    auto owned = std::make_unique<Record>();
    std::vector<uint64_t> source(152), destination(619);
    for (uint32_t i = 0; i < source.size(); ++i) source[i] = 10000 + i;
    for (uint32_t i = 0; i < destination.size(); ++i) destination[i] = 20000 + i;
    std::vector<uint32_t> pairs;
    for (uint32_t i = 1; i < source.size(); ++i)
    { pairs.push_back(i); pairs.push_back(i + 100); }
    Identity id;
    id.generation = 1; id.sourceRig = 10; id.destinationRig = 20;
    id.graph = 30; id.desc = 40; id.bones = 50; id.tag = 60;
    Check(capture->Observe(id, source, destination, pairs) == Result::Disabled, "discovery defaults disabled");
    Check(!capture->Start(0), "zero generation rejected");
    Check(capture->Start(1), "explicit census starts");
    Check(capture->Observe(id, source, destination, pairs) == Result::Stored, "bounded observed dimensions stored");
    uint32_t count = 0;
    Check(capture->Count(count) && count == 1, "owned census count available without engine pointers");
    Check(capture->Observe(id, source, destination, pairs) == Result::Duplicate, "same signature not stored twice");
    const uint64_t originalName = source[1];
    source[1] = 999999;
    Check(capture->Copy(0, *owned) && owned->sourceNames[1] == originalName,
          "control copy owns original names after engine-memory simulation changes");
    Check(capture->Observe(id, source, destination, pairs) == Result::Stored,
          "same pointers and table with changed names form a new signature (ABA evidence)");
    Check(capture->Copy(1, *owned) && owned->sourceNames[1] == 999999,
          "replacement-name evidence is independently owned");
    for (uint32_t i = 2; i < kMaxSignatures; ++i)
    {
        id.graph = 30 + i;
        Check(capture->Observe(id, source, destination, pairs) == Result::Stored, "remaining distinct signature stored");
    }
    id.graph = 100;
    Check(capture->Observe(id, source, destination, pairs) == Result::Full, "ninth signature rejected without eviction");
    Check(capture->Count(count) && count == kMaxSignatures, "overflow preserves exact bounded record count");
    Check(!capture->Copy(kMaxSignatures, *owned), "no record beyond bounded capacity");
    auto badPairs = pairs; badPairs[0] = 152;
    Check(capture->Observe(id, source, destination, badPairs) == Result::Invalid, "source pair out of bounds rejected");
    badPairs = pairs; badPairs[1] = 619;
    Check(capture->Observe(id, source, destination, badPairs) == Result::Invalid, "destination pair out of bounds rejected");
    badPairs = pairs; badPairs.pop_back();
    Check(capture->Observe(id, source, destination, badPairs) == Result::Invalid, "odd pair table rejected");
    std::vector<uint64_t> oversized(kMaxBones + 1, 42);
    Check(capture->Observe(id, oversized, destination, pairs) == Result::Invalid, "oversized source rejected rather than truncated");
    Check(capture->Observe(id, source, oversized, pairs) == Result::Invalid, "oversized destination rejected rather than truncated");
    std::vector<uint32_t> oversizedPairs((kMaxPairs + 1) * 2, 0);
    Check(capture->Observe(id, source, destination, oversizedPairs) == Result::Invalid, "oversized pair table rejected");
    capture->Stop();
    Check(capture->Observe(id, source, destination, pairs) == Result::Disabled, "stop disables engine reads");
    Check(capture->Copy(0, *owned) && owned->sourceNames[1] == originalName, "stop preserves owned evidence");
    Check(capture->Start(2), "new arming generation starts fresh capture");
    Check(capture->Observe(id, source, destination, pairs) == Result::StaleGeneration, "previous generation rejected");
    Check(!capture->Copy(0, *owned), "fresh census has no stale records");
    std::printf("Record=%zu bytes Capture=%zu bytes; %u checks, %u failures\n",
                sizeof(Record), sizeof(Capture), checks, failures);
    return failures ? 1 : 0;
}

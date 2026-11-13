#include "../src/mm/MappingCycleReceipt.hpp"

#include <array>
#include <cstdio>

namespace r = utpp::mm::mappingcycle;
using Bytes = utpp::mm::mappingreceipt::Result;

namespace
{
constexpr std::array<uint64_t, 5> kAnchors{101, 102, 103, 104, 105};
constexpr std::array<uint64_t, 2> kHelpers{201, 202};
constexpr std::array<uint64_t, 8> kDestinations{101, 102, 103, 104, 105, 201, 202, 301};

Bytes Evidence(uint32_t count, uint64_t equalMask)
{
    Bytes result;
    result.valid = true;
    result.compared = count;
    for (uint32_t i = 0; i < count; ++i)
        if (equalMask & (uint64_t{1} << i))
        {
            ++result.equal;
            result.equalPairs[0] |= uint64_t{1} << i;
        }
    return result;
}
r::Identity MapIdentity(uint32_t thread = 11)
{
    return {7, 1, 2, 3, 4, static_cast<uint32_t>(kDestinations.size()), thread, 0};
}
r::FKIdentity FK(uint32_t thread = 11)
{
    return {7, 1, 2, 3, 4, static_cast<uint32_t>(kDestinations.size()), thread};
}
bool Main(r::Plan& plan, r::State& state, uint32_t thread = 11, int64_t ticks = 100)
{
    std::array<uint32_t, 10> pairs{0,0,1,1,2,2,3,3,4,4};
    return r::Observe(plan, state, MapIdentity(thread), ticks, 50, kAnchors, pairs,
                      Evidence(5, 0x1f)) == r::ObserveResult::MainStarted;
}
}

int main()
{
    int checks = 0, failures = 0;
    auto check = [&](bool ok) { ++checks; if (!ok) ++failures; };

    r::Plan plan;
    check(r::Prepare(7, kDestinations, kAnchors, kHelpers, plan) == r::Error::None && plan.valid);
    check(plan.requiredCount == 2 && plan.requiredDestinations[0] == 5 && plan.requiredDestinations[1] == 6);
    auto missingAnchor = kAnchors; missingAnchor[4] = 999;
    check(r::Prepare(7, kDestinations, missingAnchor, kHelpers, plan) == r::Error::MissingAnchor);
    auto duplicateHelpers = kHelpers; duplicateHelpers[1] = duplicateHelpers[0];
    check(r::Prepare(7, kDestinations, kAnchors, duplicateHelpers, plan) == r::Error::Duplicate);
    check(r::Prepare(0, kDestinations, kAnchors, kHelpers, plan) == r::Error::Bounds);
    check(r::Prepare(7, kDestinations, kAnchors, kHelpers, plan) == r::Error::None);

    r::State state;
    check(Main(plan, state));
    const std::array<uint64_t, 1> helper1{kHelpers[0]};
    const std::array<uint32_t, 2> pair1{0, 5};
    check(r::Observe(plan, state, MapIdentity(), 110, 50, helper1, pair1, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    const std::array<uint64_t, 1> helper2{kHelpers[1]};
    const std::array<uint32_t, 2> pair2{0, 6};
    check(r::Observe(plan, state, MapIdentity(), 120, 50, helper2, pair2, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    auto receipt = r::ConsumeBeforeFK(state, FK(), 125, 50, true);
    const uint64_t completedCycle = receipt.cycle;
    check(receipt.status == r::ConsumeStatus::Complete && receipt.complete && completedCycle != 0 &&
          receipt.required == 2 && receipt.covered == 2 && receipt.equal == 2);
    check(r::ConsumeBeforeFK(state, FK(), 126, 50, true).status == r::ConsumeStatus::MissingMain);

    check(Main(plan, state));
    check(r::Observe(plan, state, MapIdentity(), 110, 50, helper1, pair1, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    check(r::Observe(plan, state, MapIdentity(), 111, 50, helper1, pair1, Evidence(1, 0)) ==
          r::ObserveResult::HelpersUpdated); // Last writer clears prior equality.
    check(r::Observe(plan, state, MapIdentity(), 112, 50, helper2, pair2, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    receipt = r::ConsumeBeforeFK(state, FK(), 115, 50, true);
    check(receipt.status == r::ConsumeStatus::Incomplete && receipt.covered == 2 && receipt.equal == 1 &&
          receipt.cycle > completedCycle);

    check(Main(plan, state));
    check(r::ConsumeBeforeFK(state, FK(), 105, 50, false).status == r::ConsumeStatus::Denied);
    check(r::ConsumeBeforeFK(state, FK(), 106, 50, true).status == r::ConsumeStatus::MissingMain);

    check(Main(plan, state));
    check(r::Observe(plan, state, MapIdentity(), 110, 50, helper1, pair1, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    receipt = r::ConsumeBeforeFK(state, FK(), 115, 50, true);
    check(receipt.status == r::ConsumeStatus::Incomplete && receipt.covered == 1 && receipt.required == 2);

    check(Main(plan, state));
    check(r::ConsumeBeforeFK(state, FK(12), 105, 50, true).status == r::ConsumeStatus::IdentityMismatch);
    check(!state.active);

    check(Main(plan, state));
    check(r::Observe(plan, state, MapIdentity(), 151, 50, helper1, pair1, Evidence(1, 1)) ==
          r::ObserveResult::Rejected);
    check(!state.active);

    check(Main(plan, state));
    const std::array<uint64_t, 2> wrongNameSource{kHelpers[0], 999};
    const std::array<uint32_t, 2> wrongNamePair{1, 5};
    check(r::Observe(plan, state, MapIdentity(), 110, 50, wrongNameSource, wrongNamePair,
                     Evidence(1, 1)) == r::ObserveResult::HelpersUpdated);
    check(r::ConsumeBeforeFK(state, FK(), 115, 50, true).status == r::ConsumeStatus::Incomplete);

    check(Main(plan, state));
    check(r::Observe(plan, state, MapIdentity(), 110, 50, helper1, pair1, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated);
    const std::array<uint64_t, 1> unknownSource{999};
    check(r::Observe(plan, state, MapIdentity(), 111, 50, unknownSource, pair1, Evidence(1, 1)) ==
          r::ObserveResult::HelpersUpdated); // Unknown source is still the last destination writer.
    receipt = r::ConsumeBeforeFK(state, FK(), 115, 50, true);
    check(receipt.status == r::ConsumeStatus::Incomplete && receipt.covered == 1 && receipt.equal == 0);

    check(Main(plan, state));
    const std::array<uint32_t, 2> unrelatedPair{0, 7};
    auto otherBuffer = MapIdentity(); otherBuffer.desc = 99;
    check(r::Observe(plan, state, otherBuffer, 110, 50, unknownSource, unrelatedPair,
                     Evidence(1, 1)) == r::ObserveResult::Rejected);
    check(!state.active);

    check(Main(plan, state));
    auto alias = Evidence(1, 1); alias.overlap = true;
    check(r::Observe(plan, state, MapIdentity(), 110, 50, helper1, pair1, alias) ==
          r::ObserveResult::Rejected);

    std::printf("MappingCycleReceipt: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

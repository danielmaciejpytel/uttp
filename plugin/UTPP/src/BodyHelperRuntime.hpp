#pragma once
// Local bundle loader and guarded TPP runtime bridge for the skin helper recompute.
#include "mm/MappingCycleReceipt.hpp"
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace utpp::bodyhelperruntime
{
struct Counters
{
    uint64_t evaluated=0,rejected=0,buildFailed=0,proposed=0,proposedRecords=0;
    // commitRejected = every Commit refusal; jumpRejected = the subset refused because a helper's local translation would move by more
    // than the sanity bound (mm/BodyHelperCommit.hpp kMaxTranslationDelta).
    uint64_t applied=0,appliedRecords=0,commitRejected=0,jumpRejected=0;
    // Hot-gate rejections by reason (mm/BodyHelperAdmission.hpp Reject, index 0 unused):
    // 1 ragdoll, 2 gate (disabled/fault/owner/armed/MM off), 3 identity, 4 perspective, 5 mapping-cycle receipt, 6 body.
    std::array<uint64_t,7> rejectReason{};
};

// Load reads the private <directoryDll>/skin_helpers folder: mode.txt plus one bundle per body in man/ and woman/
// (program.ir + binding.dat each). The bundles are admitted independently; a missing, broken or unapproved bundle of one body
// never disables the other (Load returns false only when no bundle is usable). PROBE only computes. APPLY is admitted only for
// a bundle whose exact file hashes are compiled into the DLL (mm/BodyHelperAdmission.hpp); mode.txt never authorises a bundle
// and structural validation alone never promotes an offline-only one.
bool Enabled() noexcept;
bool ApplyEnabled() noexcept;
bool Load(const std::wstring& directoryDll);

// Cold arm-time preparation, called while disarmed. Parent indices and names
// must describe the merged player rig in bone-buffer order. A plan is prepared for every loaded bundle the rig fits (at least the
// approved base bones, the bones the bundle reads and writes in the approved layout); Evaluate picks the one of the live body.
bool PrepareArm(uint64_t rig,std::span<const int16_t> parents,
                std::span<const uint64_t> names,
                std::span<const uint64_t> requiredHelpers);

// Builds a bounded proposal; APPLY performs guarded commit. Caller must pass the receipt
// consumed for this exact FK invocation after validating its descriptor/bones
// identity; Receipt itself does not carry those pointers.
bool Evaluate(uint8_t* desc,uint8_t* bones,uint32_t count,uint64_t rig,
              const mm::mappingcycle::Receipt& receipt,int64_t now) noexcept;
// Atomic fail-closed switch for callback fault handlers. Active storage remains
// retained until DLL shutdown, so a concurrent animation callback cannot UAF.
void Disable() noexcept;
Counters ReadCounters() noexcept;
// Cold path: owned + compensated-child hashes of the loaded approved bundle (empty otherwise). The mapping-cycle
// receipt uses it as the required-helper catalog when no early-pose experiment catalog exists (shipped default).
std::vector<uint64_t> RequiredCatalog();
std::string Status(); // cold path only
} // namespace utpp::bodyhelperruntime

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

// Coverage of the recompute by core writer (mm/CoreWriteRecord.hpp): every player FK call is counted once under its outcome, for each
// writer that wrote core bones in it, and under the row kNoWriter when no layer wrote. Outcome 0 = committed; 1..6 = the hot-gate
// rejection reasons (same numbering as Counters::rejectReason); then build failure, commit refusal, not evaluated (the call never
// reached Evaluate: no mapping-cycle policy for the rig, ragdoll, helpers disabled) and proposed only (PROBE).
enum class Outcome : uint32_t { Committed, Ragdoll, Gate, Identity, Perspective, Receipt, Body, Build, Commit, NotEvaluated, ProposedOnly, Count };
inline constexpr const char* kOutcomeNames[size_t(Outcome::Count)] = {
    "commit", "ragdoll", "gate", "identity", "perspective", "receipt", "body", "build", "commitrej", "noeval", "probe"};
inline constexpr size_t kWriterRows = 7; // mm::corewrite::kWriterCount writers + kNoWriter
inline constexpr size_t kNoWriter = 6;
struct Coverage
{
    uint64_t calls = 0; // player FK calls counted (evaluated or not)
    std::array<std::array<uint64_t,size_t(Outcome::Count)>,kWriterRows> byWriter{};
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
// identity; Receipt itself does not carry those pointers. The core write record of the calling thread (mm/CoreWriteRecord.hpp) says
// which layers wrote core bones in this call; the outcome is counted for each of them (Coverage).
bool Evaluate(uint8_t* desc,uint8_t* bones,uint32_t count,uint64_t rig,
              const mm::mappingcycle::Receipt& receipt,int64_t now) noexcept;
// A player FK call that did not reach Evaluate (aRecord = its core write record). No-op while the helpers are disabled.
void NoteNotEvaluated(uint32_t aRecord) noexcept;
Coverage ReadCoverage() noexcept;
// Atomic fail-closed switch for callback fault handlers. Active storage remains
// retained until DLL shutdown, so a concurrent animation callback cannot UAF.
void Disable() noexcept;
Counters ReadCounters() noexcept;
// Cold path: owned + compensated-child hashes of the loaded approved bundle (empty otherwise). The mapping-cycle
// receipt uses it as the required-helper catalog when no early-pose experiment catalog exists (shipped default).
std::vector<uint64_t> RequiredCatalog();
std::string Status(); // cold path only
} // namespace utpp::bodyhelperruntime

#pragma once

#include "mm/GameIntegration.hpp"
#include <cstdint>
#include <span>

namespace utpp::earlyruntime
{
enum class Owner : uint32_t { FK = 0, Mapping = 1 };
enum class Result : uint32_t { Applied, WrongOwner, NotReady, IdentityRejected, PoseRejected,
                               SemanticRejected, PerspectiveRejected, CallbackNoWrite };
constexpr uint32_t kMaxRigBones = 1024;

// Control-path only. Mode may change ONLY while g_hot.armed == false. The FK
// integration must check CurrentOwner()==Owner::FK, regardless of whether an
// individual early mapping invocation succeeds. There is no per-frame fallback.
bool SelectOwner(Owner aOwner);
Owner CurrentOwner();
// The owner chosen at load. CurrentOwner() follows it, except while a rig is
// armed for which the early arm step was not ready (see ArmOwnerForRig).
Owner RequestedOwner();

// Arm path, before g_hot.armed is published. With the requested owner Mapping
// it runs PrepareArm; when that is not ready for this rig, the FK owner takes
// this rig, so a pose still reaches the skeleton. The next arm of a ready rig
// returns to Mapping. Returns false (no change) while armed.
struct ArmChoice
{
    Owner owner = Owner::FK;
    bool fellBack = false;              // Mapping requested, FK chosen for this rig
    uint32_t missingAnchors = 0;        // main-source anchor names absent from the rig
    uint32_t missingHelpers = 0;        // catalog helper names absent from the rig
};
bool ArmOwnerForRig(bool aMappingHooked, uint64_t aMergedRig, uint32_t aCount, uint64_t aTrackA, uint64_t aTrackB,
                    std::span<const uint64_t> aDestinationNames, std::span<const uint64_t> aForbiddenHelperNames,
                    ArmChoice& aOut);

// Before publishing g_hot.armed=true. Hashes must be OWNED copies from the
// current merged player MetaRig and validated procedural-helper catalog.
// A source helper is a deformation-only name absent from the main source rig;
// do not classify the main rig's own Torso/arms control bones as such helpers.
bool PrepareArm(uint64_t aMergedRig, uint32_t aCount, uint64_t aTrackA, uint64_t aTrackB,
                std::span<const uint64_t> aDestinationNames,
                std::span<const uint64_t> aForbiddenHelperNames);
void Disarm();

struct MappingView
{
    uint8_t* desc = nullptr;
    uint8_t* bones = nullptr;
    uint64_t tag = 0;
    uint64_t sourceRig = 0; // animRig resource identity, deliberately NOT merged MetaRig
    uint32_t outputCount = 0;
    std::span<const uint64_t> sourceNames;
    std::span<const uint32_t> pairs; // uint32(source,destination), after original mapping
};

// The caller reads g_mmPose ONCE and passes that same snapshot here and to the
// apply callback. No secondary snapshot read or helper reset occurs here.
// The callback performs the existing capture/blend/AO pipeline exactly once,
// returning its written-bone count. Mapping hook must wrap all live reads and
// the callback in SEH. The callback must not reread g_mmPose.
using Apply = uint32_t (*)(void* aUser, uint8_t* aDesc, uint8_t* aBones, uint32_t aCount,
                          uint64_t aMergedRig, const mm::integ::PoseSnapshot& aSnapshot, int64_t aNow);
Result EvaluateMapping(const MappingView& aMapping, const mm::integ::PoseSnapshot& aSnapshot,
                       int64_t aNow, Apply aApply, void* aUser = nullptr);

struct Counters
{
    uint64_t calls = 0, notReady = 0, identityRejected = 0, poseRejected = 0;
    uint64_t semanticRejected = 0, perspectiveRejected = 0, applied = 0, callbackNoWrite = 0;
};
Counters ReadCounters();

// Health check between two samples of the counters: mapping jobs arrived but
// none was admitted, and no pose or perspective rejection explains it (weight
// 0, stale pose and first-person stretches stay quiet). True = the motion
// pose did not reach the skeleton in that interval.
bool MappingStalled(const Counters& aBefore, const Counters& aAfter);
} // namespace utpp::earlyruntime

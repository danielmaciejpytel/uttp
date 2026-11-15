#pragma once
// Per-frame diagnostic record of the reload contact correction (one record per pose hook call of a reload, see ReloadTraceRing.hpp for
// the ring and the file). Plain data only: the pose hook fills it on its own thread, the ring copies it, a writer thread stores it as is.
// The record is filled by reading the pose and the correction state; it never changes the pose (tests: ReloadRifleContactsTest section 14).
// Layout is fixed (the reader in tools/gametest/reload_trace.py mirrors it); a change of any field bumps kFormat.
#include <cstdint>
#include <cstring>

namespace utpp::mm::reload_trace
{
inline constexpr uint32_t kFormat = 1;

// Which call of the hook produced the record.
enum Path : uint8_t
{
    kPathNone = 0,     // reload-related frame without a correction call (not admitted, no config, observe-only)
    kPathFinish = 1,   // Finish (a native cycle was captured)
    kPathHold = 2,     // HoldOnly (eligible, no native cycle)
    kPathLockLost = 3, // another animation job owned the correction state: records left as handed in
};
// Where FinishCore left (the last stage reached).
enum Exit : uint8_t
{
    kExitNone = 0,          // FinishCore not entered (hold / none paths)
    kExitNotCaptured = 1,   // cycle / perspective / snapshot mismatch: records untouched
    kExitBadWeight = 2,     // non-finite or out-of-range correction weight
    kExitWeightZero = 3,    // correction weight 0: records untouched, Finish returns true
    kExitReadFail = 4,
    kExitDivergenceOut = 5, // divergence gate brought the weight to 0
    kExitPlaceFail = 6,     // target placement (transport / anchor / mask restore) failed
    kExitPlacementOut = 7,  // placement guard brought the weight to 0
    kExitRestoreFail = 8,
    kExitSolveReject = 9,   // clavicle reach / contact solve rejected (solveReject = native_reload_contacts::Reject)
    kExitAccepted = 10,
};
// Record::flags
enum Flag : uint32_t
{
    kFlagEligible = 1u << 0,        // snapshot: reloadContactsEligible
    kFlagAuthored = 1u << 1,        // snapshot: reloadAuthoredClip
    kFlagAdmitted = 1u << 2,        // Admit() for this hook owner
    kFlagHaveConfig = 1u << 3,
    kFlagCaptured = 1u << 4,        // native cycle captured
    kFlagPermitted = 1u << 5,       // perspective permission at the commit
    kFlagReachedSolver = 1u << 6,   // Finish: the frame reached the solver (the joint limiter / final limits run)
    kFlagWritten = 1u << 7,         // Finish / HoldOnly returned true
    kFlagReplayWritten = 1u << 8,   // RejectHold::OnReject wrote the replay
    kFlagReplayGirdle = 1u << 9,    // the held delta was taken against V's own girdle
    kFlagHoldDelta = 1u << 10,      // RejectHold has a delta (an accepted frame before)
    kFlagFinalLimit = 1u << 11,     // the final left-arm stage changed the records
    kFlagSwivel = 1u << 12,         // the left-arm swivel limiter changed the solve
    kFlagOwnerMapping = 1u << 13,   // pose hook owner: early mapping job (else the FK hook)
    kFlagAiming = 1u << 14,         // snapshot: traceAiming
    kFlagJointsValid = 1u << 15,    // joint limiter had a previous output to limit against
};
// Record::mask: snapshot upper-mask bits of the bones that matter for the reload hand-back.
enum MaskBit : uint32_t
{
    kMaskClavL = 1u << 0, kMaskClavR = 1u << 1, kMaskUpperL = 1u << 2, kMaskUpperR = 1u << 3,
    kMaskForeL = 1u << 4, kMaskForeR = 1u << 5, kMaskHandL = 1u << 6, kMaskHandR = 1u << 7,
    kMaskSpine3 = 1u << 8, kMaskHead = 1u << 9, kMaskWeaponR = 1u << 10,
};
// Anatomy rows of Record::anat (upper-arm twist, pronation: signed deg; wrist bend: deg).
enum AnatRow : int { kAnatOwn = 0, kAnatSolve = 1, kAnatSwivel = 2, kAnatJoints = 3, kAnatFinal = 4, kAnatRows = 5 };

// All floats are NaN when the stage did not run. Positions in model space (m), rotations as quaternions x, y, z, w.
struct Record
{
    int64_t ticks = 0;          // QPC of the hook call
    uint64_t sequence = 0;      // snapshot sequence
    uint32_t flags = 0;
    uint32_t mask = 0;
    uint8_t path = kPathNone, exit = kExitNone, solveReject = 0, overlay = 0;
    int8_t action = 0;
    uint8_t pushNumber[3]{}; // push number of the ring (24 bits, little endian; set by Ring::Push): a gap = dropped records
    // snapshot weights (published by the game thread for this frame)
    float snapWeight, upperScale, lowerScale, contactWeight;
    // FinishCore weights: argument, divergence gate, chest transport (lambda), placement factor raw / after its gate, final correction
    // weight, girdle weight (before the placement guard)
    float corrIn, divergence, lambda, placementRaw, placementGate, corrFinal, girdleWeight;
    // reject hold after this frame: amplitude, recorded weight, recorded girdle weight, own-girdle weight used by a replay
    float holdAmp, holdWeight, holdGirdleWeight, replayOwnWeight;
    float stockShift, leftArmShift;
    // clavicle local translations [left, right][x, y, z]: game (before the blend), as handed to the correction, V's own (snapshot), as written
    float clavGame[2][3], clavIn[2][3], clavOwn[2][3], clavOut[2][3];
    // joint limiter: requested change (deg) of left clavicle / upper arm / forearm, right clavicle / upper arm / forearm; the frame's limit
    float jointReqDeg[6];
    float jointMaxDeg;
    uint32_t jointLimited; // bit k: tracked bone k was limited
    // left arm solve: shoulder (after the clavicle reach step), the hand target (position / rotation) and the elbow guide (the game's
    // elbow position carried with the target); the game's own hand rotation (before transport) and V's chest (Spine3) rotation
    float shoulder[3], handTarget[3], handTargetRot[4], handGameRot[4], chestRot[4], guide[3];
    // left elbow after the solve, after the swivel limiter, as written; left hand rotation as written
    float elbowSolve[3], elbowSwivel[3], elbowOut[3], handOutRot[4];
    float reachClavDeg, swivelDeg;
    float anat[kAnatRows][3];
};
static_assert(sizeof(float) == 4, "record floats are 32-bit");
inline constexpr uint32_t kRecordBytes = 432;
static_assert(sizeof(Record) == kRecordBytes, "record layout is mirrored by the reader");

inline void Clear(Record& r)
{
    r = Record{};
    // every 4-byte lane from snapWeight to the end is a float except jointLimited
    uint8_t* const first = reinterpret_cast<uint8_t*>(&r.snapWeight);
    uint8_t* const end = reinterpret_cast<uint8_t*>(&r) + sizeof(Record);
    const uint32_t nanBits = 0x7FC00000u;
    for (uint8_t* p = first; p + 4 <= end; p += 4)
        std::memcpy(p, &nanBits, 4);
    r.jointLimited = 0;
}
} // namespace utpp::mm::reload_trace

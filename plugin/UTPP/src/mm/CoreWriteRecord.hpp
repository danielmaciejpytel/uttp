#pragma once
// Per-call record of which layer wrote core bones of the player's pose in the current FK pass (one FK detour call). Header-only
// (inline thread_local), so the FK hook, the vehicle and vault layers and the skin helper recompute share it without a link
// dependency. The FK detour clears it first, every layer that writes core bones marks its bit at the write, and the helper
// recompute of the same call (posediag::NoteBeforeFK, same thread) reads it. No atomics: one thread owns one FK call.
// Under the Mapping owner the main mapping job writes the motion pose (and the vehicle transition) before the FK call; its writers
// travel with the mapping-cycle receipt of the thread (PoseHook.cpp) and are added to the record of the FK call of that cycle.

#include <cstdint>

namespace utpp::mm::corewrite
{
enum Writer : uint32_t
{
    kDriver = 0,            // motion driver pose (v2) blended in by the FK detour (and the layers on top of it)
    kHost = 1,              // host pose (v1, including v2 -> v1 fades and the driver fallback) blended in by the FK detour
    kVehicleTransition = 2, // vehicle mount / exit transition, exit latch and its hold
    kNeckGuard = 3,         // seated neck cut
    kVaultTorso = 4,        // vault torso limiter
    kVehicleGuards = 5,     // mounted seat pose switch guard and pop filter
    kWriterCount = 6,
};
inline constexpr const char* kWriterNames[kWriterCount] = {"driver", "host", "vehtrans", "neck", "vault", "vehguard"};
inline constexpr uint32_t Bit(Writer aWriter) { return 1u << static_cast<uint32_t>(aWriter); }
inline constexpr uint32_t kWriterMask = (1u << kWriterCount) - 1u;
inline constexpr uint32_t kPlayerCall = 1u << 31; // the FK call was matched as the player's (set by the FK detour)
inline constexpr uint32_t kEvaluated = 1u << 30;  // the helper recompute ran on this call (set by bodyhelperruntime::Evaluate)
// Set by the FK detour before the layers run, only while the weight-0 writer recompute is switched on: a weight-0 writer (vehicle
// transition, vehicle guards, neck cut, vault torso) can write core bones in this call and the body is shown in third person.
inline constexpr uint32_t kZeroWindow = 1u << 29;
// With kZeroWindow: the vehicle camera shows the third-person body in this call (mounted third-person flag of the vehicle layer).
inline constexpr uint32_t kVehicleThirdPerson = 1u << 28;
// Writers that run at a published motion weight of 0 (their frames are not covered by the published-pose path) and the published ones.
inline constexpr uint32_t kZeroWriterMask = Bit(kVehicleTransition) | Bit(kNeckGuard) | Bit(kVaultTorso) | Bit(kVehicleGuards);
inline constexpr uint32_t kPublishedMask = Bit(kDriver) | Bit(kHost);

inline thread_local uint32_t t_record = 0;

inline void Begin() noexcept { t_record = 0; }
inline void Mark(Writer aWriter) noexcept { t_record |= Bit(aWriter); }
inline void MarkPlayerCall() noexcept { t_record |= kPlayerCall; }
inline void MarkEvaluated() noexcept { t_record |= kEvaluated; }
inline void MarkZeroWindow(bool aVehicleThirdPerson) noexcept { t_record |= kZeroWindow | (aVehicleThirdPerson ? kVehicleThirdPerson : 0u); }
inline uint32_t Current() noexcept { return t_record; }
// Mapping owner: the core writes happen in the main mapping job, before the FK call of the same cycle. The mapping hook records that
// job's writers and the weight-0 writer window it opened apart (Begin before the job, Restore after it) and hands them to the FK call
// of the same cycle (MarkEarly).
inline constexpr uint32_t kEarlyMask = kWriterMask | kZeroWindow | kVehicleThirdPerson;
// The part of a mapping job's record that is handed on: its writers, and its window only when a weight-0 writer wrote in that job (the
// FK call decides the window again itself; a job that wrote nothing at weight 0 must not keep the vehicle's third-person flag alive).
inline constexpr uint32_t EarlyPart(uint32_t aRecord) noexcept
{
    return (aRecord & kZeroWriterMask) ? (aRecord & kEarlyMask) : (aRecord & kWriterMask);
}
inline void Restore(uint32_t aRecord) noexcept { t_record = aRecord; }
inline void MarkEarly(uint32_t aRecord) noexcept { t_record |= aRecord & kEarlyMask; }
} // namespace utpp::mm::corewrite

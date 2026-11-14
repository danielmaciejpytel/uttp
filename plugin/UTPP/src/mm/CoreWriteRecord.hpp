#pragma once
// Per-call record of which layer wrote core bones of the player's pose in the current FK pass (one FK detour call). Header-only
// (inline thread_local), so the FK hook, the vehicle and vault layers and the skin helper recompute share it without a link
// dependency. The FK detour clears it first, every layer that writes core bones marks its bit at the write, and the helper
// recompute of the same call (posediag::NoteBeforeFK, same thread) reads it. No atomics: one thread owns one FK call.

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

inline thread_local uint32_t t_record = 0;

inline void Begin() noexcept { t_record = 0; }
inline void Mark(Writer aWriter) noexcept { t_record |= Bit(aWriter); }
inline void MarkPlayerCall() noexcept { t_record |= kPlayerCall; }
inline void MarkEvaluated() noexcept { t_record |= kEvaluated; }
inline uint32_t Current() noexcept { return t_record; }
} // namespace utpp::mm::corewrite

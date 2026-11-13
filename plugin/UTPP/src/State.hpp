#pragma once
// Shared lock-free state between the game thread (natives, arming, update callback) and the
// animation threads (FK detour). The detour only reads/writes these atomics.

#include <atomic>
#include <cstdint>

#include "mm/GameIntegration.hpp"
#include "mm/PerspectiveWriteGate.hpp"

namespace utpp
{
// Game exe facts (Cyberpunk2077.exe 2.50, file version 3.0.88.46235, imagebase 0x140000000; RVAs ported from 2.31).
// Audit 2026-10-23: the bodies of FK (2.31 0x4B6968),
// its caller (0x1D2DB8 -> 0x1B3E00, call at +0x478 in both) and the mapping fn (0x17DDB4) are instruction-identical
// on 2.50 apart from call targets, so every displacement below is unchanged.
constexpr uint32_t kRvaFK = 0x582B3C;       // void FK(PoseDesc* D, Rig* R)
constexpr uint32_t kRvaFKCall = 0x1B4278;   // E8 rel32 -> FK, inside 0x1B3E00(K, ctx) (2.31: 0x1D2DB8)
constexpr uint32_t kRvaMapping = 0x1B3C08;  // pose mapping fn (hooked read-only for the census / early pose)

// PoseDesc layout used by the detour (2.50 FK: mov r11,[rcx]; mov r8d,[rcx+0Ch]; bone index*48, rot +10h, scale +20h).
constexpr uint32_t kDescBonesOff = 0x00;  // bone buffer pointer
constexpr uint32_t kDescCountOff = 0x0C;  // uint32 bone count
constexpr uint32_t kDescTagOff = 0x18;    // desc[3] = output float-track buffer (mapping fn writes tracks there);
                                          // identifies the player when == trackA/trackB (VR port; runtime-proven on 2.31)
constexpr uint32_t kBoneStride = 48;      // pos float4, rot quat float4, scale float4
constexpr uint32_t kBoneRotOff = 0x10;

// C4324 (padding due to alignas) is the intended effect: counters on separate cache lines.
#pragma warning(push)
#pragma warning(disable : 4324)
struct alignas(64) Counter
{
    std::atomic<uint64_t> value{0};
};

struct HotState
{
    // Arming (written on the game thread; armed is published last with release).
    std::atomic<uint64_t> trackA{0};
    std::atomic<uint64_t> trackB{0};
    std::atomic<uint64_t> playerDesc{0}; // *(animObj+0x18): candidate full pose descriptor
    std::atomic<uint64_t> playerRig{0};  // *(animObj+0x08): metaRig of the player root component
    std::atomic<uint32_t> playerBones{0}; // metaRig bone count (FK D count is expected to equal it)
    // 0 = tag (D+0x18 == trackA/B), 1 = D == playerDesc, 2 = R == playerRig, 3 = any of them
    // 0.1.1 in-game: tag and rig matched every player FK call (2703/2703), desc never; default = tag.
    std::atomic<int32_t> matchMode{0};
    std::atomic<bool> armed{false};
    // Detailed diagnostics (UTPPSetMMTrace / mm.trace): timing of the detour's own work (ownTicks, ownTicksMax, playerFkTicks) and
    // the periodic reload-contacts funnel line. Off by default: the FK detour then takes one QPC reading, not three.
    std::atomic<bool> timing{false};

    // Test effect controls.
    std::atomic<bool> enabled{false};
    std::atomic<float> angleDeg{0.0f};
    std::atomic<int32_t> axis{1}; // default local Y (unverified choice; settable 0/1/2)
    std::atomic<int32_t> boneIndex{-1};

    // Fault latch: set once by the SEH handler, never cleared for the session.
    std::atomic<bool> fault{false};
    std::atomic<uint32_t> faultCode{0};

    // Diagnostics (relaxed, each on its own cache line).
    Counter totalCalls;
    Counter matchedCalls;
    Counter appliedCalls;
    Counter badIndexCalls;
    // Cost of our own detour work (everything except the original FK), in QueryPerformanceCounter ticks.
    Counter ownTicks;
    std::atomic<uint64_t> ownTicksMax{0};
    // Cost of the original FK for matched (player) calls, for scale.
    Counter playerFkTicks;
    std::atomic<uint32_t> lastMatchedCount{0};
    Counter matchTag;
    Counter matchDesc;
    Counter matchRig;
    // Sample of an FK call whose bone count equals the player's rig bone count.
    std::atomic<uint64_t> sampleDesc{0};
    std::atomic<uint64_t> sampleRig{0};
    std::atomic<uint64_t> sampleTag{0};
    Counter sameCountCalls;

    // Motion matching (Phase 4b). mmActive is a cheap hint (published weight > 0); the detour then reads the snapshot.
    std::atomic<bool> mmActive{false};
    mm::PerspectiveWriteGate perspectiveWrite;
    std::atomic<int64_t> perspectiveWriteMaxAge{0};
    std::atomic<int64_t> mmStaleTicks{0}; // snapshots older than this (QPC ticks) are ignored; 0 = no check
    Counter mmAppliedCalls;   // FK calls where the MM pose was blended
    Counter mmBonesWritten;   // total bones written
    Counter mmTornReads;      // snapshot read failed (writer active twice) -> game pose kept
    Counter mmStaleSkips;     // snapshot too old -> game pose kept
    Counter mmRigMismatch;    // FK rig != snapshot rig -> game pose kept
    // Reload contact correction funnel (diagnostics, 2026-10-25): where eligible reload frames stop.
    Counter rcSnapEligible;   // snapshot says reload contacts eligible (weight > 0 possible)
    Counter rcOverlayOk;      // ... and the overlay is a pistol one
    Counter rcOwnerOk;        // ... and the early owner is Mapping
    Counter rcHaveConfig;     // ... and a Prepared config is published
    Counter rcCaptured;       // ... and Capture() accepted (weight 1, Valid, finite records)
    Counter rcFinishWritten;  // Finish() wrote a correction or held replay
    Counter rcLockLost;       // continuity lock not obtained
    Counter rcHoldOnly;       // eligible reload without a native cycle (hold replay path)
};

#pragma warning(pop)

extern HotState g_hot;
// Latest motion matching pose + map + weight; written by the game thread (MMDriver), read by the FK detour.
extern mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

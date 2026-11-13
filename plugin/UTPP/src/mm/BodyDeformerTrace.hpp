#pragma once
// Explicit, bounded MAIN-rig helper capture. Arm/Pop run on the control path.
// Begin/End run around BlendIntoRecords; they never allocate, wait, log, or do IO.
// Continuous mode (UTPPBodyPoseTraceContinuous, mm/BodyTraceContinuous.hpp + BodyTraceRuntime.cpp): a writer thread is the single Pop consumer and tops the
// budget up with Refill (no Arm gap); Live() = armed and not cancelled. The hot path is unchanged.
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>

namespace utpp::mm::bodytrace
{
inline constexpr uint32_t kMaxBones = 185, kMaxFrames = 200, kRingSize = 256;
// Original24 diagnostics retain their slots. Additional names come from the verified retail
// female fullbody mesh joint list plus Root/heel ancestors (Build/experiments/BodyDeformationOffline).
// Names only: no mesh data or weights are embedded. Explicit capture remains capped at200.
inline constexpr std::array<std::string_view, kMaxBones> kNames = {
    "Spine3", "LeftShoulder", "RightShoulder", "LeftArm",
    "RightArm", "LeftForeArm", "RightForeArm", "l_SHL_0_JNT",
    "l_SHL_1_JNT", "r_SHL_0_JNT", "r_SHL_1_JNT", "l_scapula_A_top_out_JNT",
    "l_scapula_A_bot_out_JNT", "r_scapula_A_top_out_JNT", "r_scapula_A_bot_out_JNT", "l_deltoid_top_top_out_JNT",
    "l_deltoid_back_top_out_JNT", "r_deltoid_top_top_out_JNT", "r_deltoid_back_top_out_JNT", "l_trapezius_top_mscl_JNT",
    "r_trapezius_top_mscl_JNT", "l_latissimusDorsi_mscl_JNT", "r_latissimusDorsi_mscl_JNT", "Neck",
    "Hips", "Spine", "Spine1", "Spine2",
    "l_deltoid_front_top_out_JNT", "l_deltoid_front_bot_out_JNT", "l_deltoid_back_bot_out_JNT", "l_triceps_mscl_JNT",
    "l_butterfly_top_CRV_bot_out_JNT", "r_deltoid_front_top_out_JNT", "r_deltoid_front_bot_out_JNT", "r_deltoid_back_bot_out_JNT",
    "r_triceps_mscl_JNT", "r_butterfly_top_CRV_top_out_JNT", "l_sternocleidomastoid_bot_mscl_JNT", "r_sternocleidomastoid_bot_mscl_JNT",
    "l_chest_front_A_bot_out_JNT", "l_chest_front_B_top_out_JNT", "l_chest_front_B_bot_out_JNT", "l_chest_front_C_top_out_JNT",
    "l_chest_front_C_bot_out_JNT", "r_chest_front_A_bot_out_JNT", "r_chest_front_B_top_out_JNT", "r_chest_front_B_bot_out_JNT",
    "r_chest_front_C_top_out_JNT", "r_chest_front_C_bot_out_JNT", "LeftUpLeg", "LeftLeg",
    "LeftFoot", "LeftToeBase", "l_calf_0_JNT", "l_lowLeg_back_B_mscl_JNT",
    "l_lowLeg_back_A_mscl_JNT", "l_knee_mscl_JNT", "l_thigh_0_JNT", "l_thigh_1_JNT",
    "l_leg_back_B_mscl_JNT", "l_leg_back_A_mscl_JNT", "l_leg_groin_mscl_JNT", "RightUpLeg",
    "RightLeg", "RightFoot", "RightToeBase", "r_calf_0_JNT",
    "r_lowLeg_back_A_mscl_JNT", "r_lowLeg_back_B_mscl_JNT", "r_knee_mscl_JNT", "r_thigh_0_JNT",
    "r_thigh_1_JNT", "r_leg_back_B_mscl_JNT", "r_leg_back_A_mscl_JNT", "r_leg_groin_mscl_JNT",
    "l_leg_buttock_mscl_JNT", "r_leg_buttock_mscl_JNT", "l_deltoid_top_bot_out_JNT", "l_biceps_A_mscl_JNT",
    "l_Wrist_0_JNT", "l_Wrist_1_JNT", "l_Wrist_2_JNT", "l_wrist_A_mscl_JNT",
    "l_wrist_B_mscl_JNT", "l_wrist_C_mscl_JNT", "LeftHand", "LeftHandThumb1",
    "LeftHandThumb2", "l_thumb_side_mscl_JNT", "LeftInHandIndex", "LeftHandIndex1",
    "LeftHandIndex2", "LeftHandIndex3", "l_ind_knuckle_B_top_mscl_JNT", "l_ind_knuckle_top_mscl_JNT",
    "l_ind_knuckle_bot_mscl_JNT", "l_thumb_thumb_A_bot_mscl_JNT", "l_thumb_thumb_A_top_mscl_JNT", "LeftInHandMiddle",
    "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3", "l_mid_knuckle_B_top_mscl_JNT",
    "l_mid_knuckle_top_mscl_JNT", "l_mid_knuckle_bot_mscl_JNT", "LeftInHandRing", "LeftHandRing1",
    "LeftHandRing2", "LeftHandRing3", "l_ring_knuckle_B_top_mscl_JNT", "l_ring_knuckle_top_mscl_JNT",
    "l_ring_knuckle_bot_mscl_JNT", "LeftInHandPinky", "LeftHandPinky1", "LeftHandPinky2",
    "LeftHandPinky3", "l_pinky_knuckle_B_top_mscl_JNT", "l_pinky_knuckle_top_mscl_JNT", "l_pinky_knuckle_bot_mscl_JNT",
    "l_pinkyBase_A_bot_mscl_JNT", "l_elbow_bend_A_mscl_JNT", "l_elbow_bend_B_mscl_JNT", "l_elbow_back_mscl_JNT",
    "r_deltoid_top_bot_out_JNT", "r_biceps_A_mscl_JNT", "r_Wrist_0_JNT", "r_Wrist_1_JNT",
    "r_Wrist_2_JNT", "r_wrist_A_mscl_JNT", "r_wrist_B_mscl_JNT", "r_wrist_C_mscl_JNT",
    "RightHand", "RightHandThumb1", "RightHandThumb2", "r_thumb_side_mscl_JNT",
    "RightInHandIndex", "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
    "r_ind_knuckle_B_top_mscl_JNT", "r_ind_knuckle_top_mscl_JNT", "r_ind_knuckle_bot_mscl_JNT", "r_thumb_thumb_A_bot_mscl_JNT",
    "r_thumb_thumb_A_top_mscl_JNT", "RightInHandMiddle", "RightHandMiddle1", "RightHandMiddle2",
    "RightHandMiddle3", "r_mid_knuckle_B_top_mscl_JNT", "r_mid_knuckle_top_mscl_JNT", "r_mid_knuckle_bot_mscl_JNT",
    "RightInHandRing", "RightHandRing1", "RightHandRing2", "RightHandRing3",
    "r_ring_knuckle_B_top_mscl_JNT", "r_ring_knuckle_top_mscl_JNT", "r_ring_knuckle_bot_mscl_JNT", "RightInHandPinky",
    "RightHandPinky1", "RightHandPinky2", "RightHandPinky3", "r_pinky_knuckle_B_top_mscl_JNT",
    "r_pinky_knuckle_top_mscl_JNT", "r_pinky_knuckle_bot_mscl_JNT", "r_pinkyBase_A_bot_mscl_JNT", "r_elbow_bend_A_mscl_JNT",
    "r_elbow_bend_B_mscl_JNT", "r_elbow_back_mscl_JNT", "Root", "LeftHeel",
    "RightHeel",
    // Complete helper-program core/protected inputs for exact replay; keep existing slots stable.
    "Head", "LeftHandEnd", "LeftInHandThumb", "Neck1", "RightHandEnd", "RightInHandThumb", "WeaponLeft", "WeaponRight", "LeftEye", "RightEye", "Trajectory", "reference_joint"};

struct Map
{
    uint64_t rig = 0;
    uint32_t rigCount = 0;
    std::array<int32_t, kMaxBones> index{};
    Map() { index.fill(-1); }
};
// Call after ReadRigNamesAndParents, away from FK. Duplicate names reject mapping.
template<class Names>
inline bool Resolve(const Names& names, uint64_t rig, Map& out)
{
    Map pending;
    if (!rig || names.empty() || names.size() > 65535) return false;
    pending.rig = rig; pending.rigCount = static_cast<uint32_t>(names.size());
    uint32_t mapped = 0;
    for (uint32_t n = 0; n < kMaxBones; ++n)
        for (uint32_t i = 0; i < pending.rigCount; ++i)
            if (std::string_view(names[i]) == kNames[n])
            {
                if (pending.index[n] >= 0) return false;
                pending.index[n] = static_cast<int32_t>(i); ++mapped;
            }
    if (!mapped) return false;
    out = pending;
    return true;
}

struct Frame
{
    uint64_t generation = 0, rig = 0, desc = 0;
    int64_t ticks = 0;
    uint32_t thread = 0, rigCount = 0;
    std::array<int32_t, kMaxBones> index{};
    float stage[2][kMaxBones][12]{};
};

class Capture
{
public:
    // Returns false while an earlier capture is active or frames remain undrained.
    // Failed Arm never changes an existing session.
    bool Arm(const Map& map, uint32_t budget = kMaxFrames)
    {
        if (!map.rig || !map.rigCount || map.rigCount > 65535 || budget == 0 || budget > kMaxFrames) return false;
        uint32_t mapped = 0;
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            const int32_t i = map.index[n];
            if (i < -1 || (i >= 0 && uint32_t(i) >= map.rigCount)) return false;
            if (i >= 0)
            {
                ++mapped;
                for (uint32_t p = 0; p < n; ++p) if (map.index[p] == i) return false;
            }
        }
        if (!mapped || busy.test_and_set(std::memory_order_acquire)) return false;
        if (remaining || write.load(std::memory_order_acquire) != read.load(std::memory_order_acquire))
        { busy.clear(std::memory_order_release); return false; }
        config = map; ++generation; remaining = budget; dropped.store(0);
        live.store(true, std::memory_order_release);
        active.store(true, std::memory_order_release);
        busy.clear(std::memory_order_release);
        return true;
    }
    bool Active() const { return active.load(std::memory_order_acquire); }
    // Continuous mode (BodyTraceContinuous.hpp): armed and not cancelled since. Refill tops the budget up to aBudget when fewer than
    // aBelow frames remain, without re-arming (no gap, ring / generation / dropped counter unchanged). False when cancelled, not armed
    // or the producer holds the flag right now (retry later).
    bool Live() const { return live.load(std::memory_order_acquire); }
    bool Refill(uint32_t aBudget = kMaxFrames, uint32_t aBelow = kMaxFrames / 2)
    {
        if (aBudget == 0 || aBudget > kMaxFrames || !live.load(std::memory_order_acquire) ||
            busy.test_and_set(std::memory_order_acquire)) return false;
        bool ok = false;
        if (live.load(std::memory_order_acquire) && remaining < aBelow)
        { remaining = aBudget; active.store(true, std::memory_order_release); ok = true; }
        busy.clear(std::memory_order_release);
        return ok;
    }    uint64_t Dropped() const { return dropped.load(std::memory_order_relaxed); }
    bool Cancel()
    {
        live.store(false, std::memory_order_release);
        // Stop new Begin calls even when the producer currently owns the flag.
        active.store(false, std::memory_order_release);
        if (busy.test_and_set(std::memory_order_acquire)) return false;
        remaining = 0; active.store(false, std::memory_order_release); ++generation;
        busy.clear(std::memory_order_release); return true;
    }
    bool Begin(Frame& f, const uint8_t* bones, uint32_t count, uint64_t rig,
               uint64_t desc, int64_t ticks, uint32_t thread)
    {
        if (!Active() || !bones || !desc || ticks <= 0) return false;
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1); return false; }
        if (!remaining || rig != config.rig || count != config.rigCount)
        { busy.clear(std::memory_order_release); return false; }
        f.generation = generation; f.rig = rig; f.desc = desc; f.ticks = ticks;
        f.thread = thread; f.rigCount = count; f.index = config.index;
        // Keep only the cheap configuration copy protected; local record copy is private.
        busy.clear(std::memory_order_release);
        return Copy(f, 0, bones, count);
    }
    bool End(Frame& f, const uint8_t* bones, uint32_t count, uint64_t rig, uint64_t desc)
    {
        if (!Active() || f.rig != rig || f.desc != desc || !Copy(f, 1, bones, count)) return false;
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1); return false; }
        bool ok = false;
        if (remaining && Active() && f.generation == generation)
        {
            const uint64_t w = write.load(std::memory_order_relaxed);
            if (w - read.load(std::memory_order_acquire) < kRingSize)
            {
                ring[w % kRingSize] = f;
                write.store(w + 1, std::memory_order_release);
                if (!--remaining) active.store(false, std::memory_order_release);
                ok = true;
            }
            else dropped.fetch_add(1);
        }
        busy.clear(std::memory_order_release);
        return ok;
    }
    // Exactly one consumer; copies the frame before allowing its ring slot to be reused.
    bool Pop(Frame& out)
    {
        const uint64_t r = read.load(std::memory_order_relaxed);
        if (r == write.load(std::memory_order_acquire)) return false;
        out = ring[r % kRingSize];
        read.store(r + 1, std::memory_order_release); return true;
    }
private:
    static bool Copy(Frame& f, uint32_t stage, const uint8_t* bones, uint32_t count)
    {
        if (!bones || stage > 1 || count != f.rigCount || !count || count > 65535) return false;
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            const int32_t i = f.index[n];
            if (i < -1 || (i >= 0 && uint32_t(i) >= count)) return false;
            if (i < 0) for (float& x : f.stage[stage][n]) x = std::numeric_limits<float>::quiet_NaN();
            else std::memcpy(f.stage[stage][n], bones + size_t(i) * 48, 48);
        }
        return true;
    }
    Map config;
    uint64_t generation = 0;
    uint32_t remaining = 0;
    std::atomic<bool> active{false}, live{false};
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    std::atomic<uint64_t> read{0}, write{0}, dropped{0};
    std::array<Frame, kRingSize> ring;
};

// A separate, opt-in snapshot taken only after the guarded helper correction
// commits. It deliberately has its own record shape/ring and output file so the
// legacy before_mm/after_mm_and_aim CSV schema and semantics never change.
struct PostHelperFrame
{
    uint64_t generation = 0, rig = 0, desc = 0;
    uint64_t receiptGeneration = 0, receiptCycle = 0;
    int64_t ticks = 0, receiptStartedTicks = 0, receiptLastTicks = 0;
    uint32_t thread = 0, rigCount = 0;
    std::array<int32_t, kMaxBones> index{};
    float local[kMaxBones][12]{};
};

class PostHelperCapture
{
public:
    bool Arm(const Map& map, uint32_t budget = kMaxFrames)
    {
        if (!map.rig || !map.rigCount || map.rigCount > 65535 || budget == 0 || budget > kMaxFrames)
            return false;
        uint32_t mapped = 0;
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            const int32_t i = map.index[n];
            if (i < -1 || (i >= 0 && uint32_t(i) >= map.rigCount)) return false;
            if (i >= 0)
            {
                ++mapped;
                for (uint32_t p = 0; p < n; ++p) if (map.index[p] == i) return false;
            }
        }
        if (!mapped || busy.test_and_set(std::memory_order_acquire)) return false;
        if (remaining || write.load(std::memory_order_acquire) != read.load(std::memory_order_acquire))
        { busy.clear(std::memory_order_release); return false; }
        config = map; ++generation; remaining = budget; dropped.store(0);
        live.store(true, std::memory_order_release);
        active.store(true, std::memory_order_release);
        busy.clear(std::memory_order_release);
        return true;
    }
    bool Active() const { return active.load(std::memory_order_acquire); }
    // Continuous mode (BodyTraceContinuous.hpp): armed and not cancelled since. Refill tops the budget up to aBudget when fewer than
    // aBelow frames remain, without re-arming (no gap, ring / generation / dropped counter unchanged). False when cancelled, not armed
    // or the producer holds the flag right now (retry later).
    bool Live() const { return live.load(std::memory_order_acquire); }
    bool Refill(uint32_t aBudget = kMaxFrames, uint32_t aBelow = kMaxFrames / 2)
    {
        if (aBudget == 0 || aBudget > kMaxFrames || !live.load(std::memory_order_acquire) ||
            busy.test_and_set(std::memory_order_acquire)) return false;
        bool ok = false;
        if (live.load(std::memory_order_acquire) && remaining < aBelow)
        { remaining = aBudget; active.store(true, std::memory_order_release); ok = true; }
        busy.clear(std::memory_order_release);
        return ok;
    }    uint64_t Dropped() const { return dropped.load(std::memory_order_relaxed); }
    bool Cancel()
    {
        live.store(false, std::memory_order_release);
        active.store(false, std::memory_order_release);
        if (busy.test_and_set(std::memory_order_acquire)) return false;
        remaining = 0; active.store(false, std::memory_order_release); ++generation;
        busy.clear(std::memory_order_release); return true;
    }
    bool Capture(const uint8_t* bones, uint32_t count, uint64_t rig, uint64_t desc,
                 int64_t ticks, uint32_t thread, uint64_t receiptGeneration,
                 uint64_t receiptCycle, int64_t receiptStartedTicks, int64_t receiptLastTicks)
    {
        if (!Active() || !bones || !desc || ticks <= 0 || receiptGeneration == 0 ||
            receiptCycle == 0 || receiptStartedTicks <= 0 || receiptLastTicks < receiptStartedTicks ||
            receiptLastTicks > ticks)
            return false;
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1); return false; }
        if (!remaining || rig != config.rig || count != config.rigCount)
        { busy.clear(std::memory_order_release); return false; }
        PostHelperFrame frame;
        frame.generation = generation; frame.rig = rig; frame.desc = desc;
        frame.ticks = ticks; frame.thread = thread; frame.rigCount = count;
        frame.receiptGeneration = receiptGeneration; frame.receiptCycle = receiptCycle;
        frame.receiptStartedTicks = receiptStartedTicks; frame.receiptLastTicks = receiptLastTicks;
        frame.index = config.index;
        busy.clear(std::memory_order_release);
        if (count > 65535) return false;
        for (uint32_t n = 0; n < kMaxBones; ++n)
        {
            const int32_t i = frame.index[n];
            if (i < -1 || (i >= 0 && uint32_t(i) >= count)) return false;
            if (i < 0) for (float& x : frame.local[n]) x = std::numeric_limits<float>::quiet_NaN();
            else std::memcpy(frame.local[n], bones + size_t(i) * 48, 48);
        }
        if (busy.test_and_set(std::memory_order_acquire)) { dropped.fetch_add(1); return false; }
        bool ok = false;
        if (remaining && Active() && frame.generation == generation)
        {
            const uint64_t w = write.load(std::memory_order_relaxed);
            if (w - read.load(std::memory_order_acquire) < kRingSize)
            {
                ring[w % kRingSize] = frame;
                write.store(w + 1, std::memory_order_release);
                if (!--remaining) active.store(false, std::memory_order_release);
                ok = true;
            }
            else dropped.fetch_add(1);
        }
        busy.clear(std::memory_order_release);
        return ok;
    }
    bool Pop(PostHelperFrame& out)
    {
        const uint64_t r = read.load(std::memory_order_relaxed);
        if (r == write.load(std::memory_order_acquire)) return false;
        out = ring[r % kRingSize];
        read.store(r + 1, std::memory_order_release); return true;
    }
private:
    Map config;
    uint64_t generation = 0;
    uint32_t remaining = 0;
    std::atomic<bool> active{false}, live{false};
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    std::atomic<uint64_t> read{0}, write{0}, dropped{0};
    std::array<PostHelperFrame, kRingSize> ring;
};
inline Capture g_capture;
inline PostHelperCapture g_postHelperCapture;
}

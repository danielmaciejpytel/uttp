#pragma once
// motion idle breaks (report 2.5.1 / 3.18, stage 10.9 [P]): deterministic idle-break player for the MM driver.
//
// Sources (only the rule constants live here):
//  - ABP pseudo "State Controller": Idle Loop -> Idle Break when GetInstanceCurrentStateElapsedTime > 3 (strict,
//    AnimStateTransitionNode_6); Idle Break -> Idle Loop by the shared rules "General - Animation Almost Complete"
//    (IsAnimationAlmostComplete() && elapsed > 0; IsAnimationAlmostComplete = !looping && time remaining <= 0.75 s)
//    and "General - No Valid Anim" (NoValidAnim || BlendStackInputs.Loop). Idle Break leaves through the aliases
//    "Idle -> Locomotion" (IsMoving()), "Grounded -> In Air" (MovementMode == InAir) and "Re-Transition to Idle
//    Loop" (Stance != Stance_LastFrame; ShouldTurnInPlace() - the break rows carry no TurnInPlace tag). Every state
//    has bAlwaysResetOnEntry, so the 3 s timer restarts on each Idle Loop entry (after a break, after a transition).
//  - OnStateEntry_IdleBreak: SetBlendStackAnimFromChooser(IdleBreak, ForceBlend=true): first chooser result, its
//    StartTime / BlendTime; no result -> NoValidAnim. OnStateEntry_IdleLoop: Idle Loop row, BlendTime 0.75.
//  - CHT_AnimationsForStateMachine "Stand Idle Loops" / "Crouch Idle Loops" (choosers_rules.txt l. 15-25 / 612-623):
//    columns MultiEnum(StateMachineState), FloatRange(Get_AO_Yaw, inclusive), OutputStruct, RandomizeColumn (no
//    context binding, RepeatProbabilityMultiplier 1, all weights 1) -> one weighted-random row among the rows that
//    pass. 10 rows; M_Neutral_Stand_Idle_Break_v05 has no row and is never selected.
//  - Get_AO_Yaw = select(RotationMode, OrientToMovement: 0, Strafe: Get_AOValue().X); Get_AOValue lerps the AO
//    towards 0 by the Disable_AO curve (6 break clips carry it: Stand v03/v04/v05, Crouch v01/v04/v05).
//
// UTPP choices ([P], report 2.5.1 / stage 10.9 test): any player input and aiming also block / interrupt a break
// (motion interrupts only on IsMoving / InAir / stance change / turn in place); the RNG is a seeded splitmix64 (UE uses
// the unseeded global FMath::FRand, so only the distribution, not the sequence, can match); the break blend weight
// is linear over the blend time (Disable_AO is read from the blended pose in UE, so the effective value is
// weight * curve). Transition order per update follows FAnimNode_StateMachine: rules are evaluated on the values of
// the previous update, then the active state's elapsed time and the clip time advance by dt.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace utpp::mm::idlebreak
{
enum class Stance : uint8_t
{
    Stand = 0,
    Crouch = 1
};

struct Row
{
    std::string_view clip;
    Stance stance;
    bool hasAoRange; // FloatRange(Get_AO_Yaw) bound on this row; otherwise the row passes for any yaw
    float aoYawMin;
    float aoYawMax;
    float startTime;
    float blendTime;
    float weight;
};

// Chooser order (Stand Idle Loops rows 1-5, Crouch Idle Loops rows 1-5; row 0 of each is the Idle Loop).
inline constexpr Row kRows[] = {
    {"M_Neutral_Stand_Idle_Break_v06", Stance::Stand, false, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Stand_Idle_Break_v04", Stance::Stand, true, -180.0f, 10.0f, 0.6f, 0.5f, 1.0f},
    {"M_Neutral_Stand_Idle_Break_v03", Stance::Stand, true, -10.0f, 180.0f, 0.3f, 0.5f, 1.0f},
    {"M_Neutral_Stand_Idle_Break_v02", Stance::Stand, false, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Stand_Idle_Break_v01", Stance::Stand, false, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Crouch_Idle_Break_v01", Stance::Crouch, true, -10.0f, 180.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Crouch_Idle_Break_v02", Stance::Crouch, false, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Crouch_Idle_Break_v03", Stance::Crouch, false, 0.0f, 0.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Crouch_Idle_Break_v04", Stance::Crouch, true, -180.0f, 10.0f, 0.0f, 0.5f, 1.0f},
    {"M_Neutral_Crouch_Idle_Break_v05", Stance::Crouch, true, -180.0f, 10.0f, 0.0f, 0.5f, 1.0f},
};
inline constexpr int kRowCount = static_cast<int>(sizeof(kRows) / sizeof(kRows[0]));
inline constexpr std::string_view kUnlistedClip = "M_Neutral_Stand_Idle_Break_v05"; // exported, no chooser row
inline constexpr std::string_view kIdleLoopClip[2] = {"M_Neutral_Stand_Idle_Loop", "M_Neutral_Crouch_Idle_Loop"};

inline constexpr float kBreakAfterIdleSeconds = 3.0f; // Idle Loop elapsed > 3 (strict)
inline constexpr float kAlmostCompleteSeconds = 0.75f; // IsAnimationAlmostComplete: time remaining <= 0.75
inline constexpr float kIdleLoopBlendTime = 0.75f;     // Idle Loop chooser row BlendTime
inline constexpr float kDefaultInterruptBlendTime = 0.5f; // Get_MMBlendTime OnGround -> OnGround

// Deterministic RNG (splitmix64). NextFloat01 in [0, 1) with 24-bit resolution.
struct Rng
{
    uint64_t state = 0;
    explicit Rng(uint64_t aSeed = 1) : state(aSeed) {}
    uint64_t Next()
    {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    float NextFloat01() { return static_cast<float>(Next() >> 40) * (1.0f / 16777216.0f); }
};

// Get_AO_Yaw: 0 in OrientToMovement; in Strafe the AO yaw lerped to 0 by the current Disable_AO value.
inline float ChooserAoYaw(bool aStrafe, float aAoYawDeg, float aDisableAO)
{
    return aStrafe ? aAoYawDeg * (1.0f - std::clamp(aDisableAO, 0.0f, 1.0f)) : 0.0f;
}

inline bool RowPasses(const Row& aRow, Stance aStance, float aChooserAoYaw)
{
    if (aRow.stance != aStance)
        return false;
    return !aRow.hasAoRange || (aChooserAoYaw >= aRow.aoYawMin && aChooserAoYaw <= aRow.aoYawMax);
}

// FRandomizeColumn over the rows that pass the MultiEnum / FloatRange columns: weighted pick, one draw per call when
// at least one row passes. aAvailable (optional, kRowCount entries) removes rows whose clip is not loaded.
// Returns the kRows index or -1 (NoValidAnim).
inline int SelectRow(Stance aStance, float aChooserAoYaw, Rng& aRng, const bool* aAvailable = nullptr)
{
    float total = 0.0f;
    int last = -1;
    for (int i = 0; i < kRowCount; ++i)
        if ((!aAvailable || aAvailable[i]) && RowPasses(kRows[i], aStance, aChooserAoYaw) && kRows[i].weight > 0.0f)
        {
            total += kRows[i].weight;
            last = i;
        }
    if (last < 0)
        return -1;
    float r = aRng.NextFloat01() * total;
    for (int i = 0; i < kRowCount; ++i)
    {
        if ((aAvailable && !aAvailable[i]) || !RowPasses(kRows[i], aStance, aChooserAoYaw) || kRows[i].weight <= 0.0f)
            continue;
        if (r < kRows[i].weight)
            return i;
        r -= kRows[i].weight;
    }
    return last; // rounding
}

// Runtime data of one break clip (from the retarget output: duration = (num_frames - 1) / fps).
struct ClipData
{
    float duration = 0.0f;
    float fps = 30.0f;
    bool looping = false;
    std::vector<float> disableAO; // per-frame Disable_AO samples; empty = no curve (0)

    float DisableAOAt(float aTime) const
    {
        if (disableAO.empty() || fps <= 0.0f)
            return 0.0f;
        const float f = std::clamp(aTime * fps, 0.0f, static_cast<float>(disableAO.size() - 1));
        const size_t i0 = static_cast<size_t>(f);
        const size_t i1 = std::min(i0 + 1, disableAO.size() - 1);
        const float a = f - static_cast<float>(i0);
        return std::clamp(disableAO[i0] + (disableAO[i1] - disableAO[i0]) * a, 0.0f, 1.0f);
    }
};

enum class Event : uint8_t
{
    None,
    StartBreak,  // play `clip` from `startTime`, blend `blendTime` (0.5)
    EndBreak,    // back to the idle loop `clip`, blend 0.75 (break clip has <= 0.75 s left and blends out)
    Interrupt,   // break cut by movement / air / stance / turn / input / aim; the driver picks the next clip
    NoValidAnim  // no row passed: Idle Break -> Idle Loop at once, timer restarts
};

enum class Phase : uint8_t
{
    IdleLoop,
    Break
};

struct Inputs
{
    float dt = 0.0f;
    Stance stance = Stance::Stand;
    bool moving = false;          // motion IsMoving(): future velocity and acceleration non-zero
    bool inAir = false;           // MovementMode == InAir
    bool turnInPlace = false;     // ShouldTurnInPlace()
    bool idleLoopActive = true;   // false while the driver plays a transition / landing / turn clip (not Idle Loop)
    bool inputPressed = false;    // [P] any player input
    bool aiming = false;          // [P] aiming / weapon raised
    bool strafe = false;          // RotationMode == Strafe (Get_AO_Yaw uses the AO yaw), else OrientToMovement (0)
    float aoYawDeg = 0.0f;        // Get_AOValue yaw before Disable_AO
    float interruptBlendTime = kDefaultInterruptBlendTime; // blend-out of the break after Interrupt
};

struct Output
{
    Event event = Event::None;
    int row = -1;              // kRows index of the break for StartBreak / EndBreak / Interrupt / while playing
    std::string_view clip;     // StartBreak: break clip, EndBreak: idle loop of the stance, else empty
    float startTime = 0.0f;    // StartBreak
    float blendTime = 0.0f;    // StartBreak 0.5, EndBreak 0.75, Interrupt: interruptBlendTime
    Phase phase = Phase::IdleLoop;
    float idleTime = 0.0f;     // Idle Loop state elapsed time
    float breakClipTime = 0.0f; // time in the playing (or blending-out) break clip
    float breakWeight = 0.0f;  // linear weight of the break clip (blend in / blend out)
    float disableAO = 0.0f;    // breakWeight * Disable_AO(breakClipTime)
};

class Controller
{
public:
    explicit Controller(uint64_t aSeed = 1) : m_rng(aSeed) {}

    // Registers a loaded break clip; rows whose clip is never registered (or is looping / too short) are skipped.
    void SetClip(std::string_view aName, const ClipData& aData)
    {
        for (int i = 0; i < kRowCount; ++i)
            if (kRows[i].clip == aName)
            {
                m_clips[i] = aData;
                m_available[i] = !aData.looping && aData.duration > kRows[i].startTime;
            }
    }

    void Reset()
    {
        m_phase = Phase::IdleLoop;
        m_idle = 0.0f;
        m_row = -1;
        m_tailRow = -1;
        m_hasLastStance = false;
    }

    Phase GetPhase() const { return m_phase; }

    Output Update(const Inputs& aIn)
    {
        const float dt = std::isfinite(aIn.dt) ? std::max(0.0f, aIn.dt) : 0.0f;
        const bool stanceChanged = m_hasLastStance && aIn.stance != m_lastStance;
        m_lastStance = aIn.stance;
        m_hasLastStance = true;
        const bool blocked = aIn.moving || aIn.inAir || aIn.turnInPlace || stanceChanged || !aIn.idleLoopActive ||
                             aIn.inputPressed || aIn.aiming;
        Output out;

        // blend-out tail of a finished / interrupted break
        if (m_tailRow >= 0)
        {
            m_tailTime += dt;
            m_tailElapsed += dt;
            if (m_tailElapsed >= m_tailBlend)
                m_tailRow = -1;
        }

        // transition rules on the previous update's values
        if (m_phase == Phase::Break)
        {
            if (blocked)
            {
                out.event = Event::Interrupt;
                out.row = m_row;
                out.blendTime = std::max(0.0f, aIn.interruptBlendTime);
                StartTail(out.blendTime);
                m_idle = 0.0f;
            }
            else if (m_breakElapsed > 0.0f && m_clips[m_row].duration - m_clipTime <= kAlmostCompleteSeconds)
            {
                out.event = Event::EndBreak;
                out.row = m_row;
                out.clip = kIdleLoopClip[static_cast<int>(aIn.stance)];
                out.blendTime = kIdleLoopBlendTime;
                StartTail(kIdleLoopBlendTime);
                m_idle = 0.0f;
            }
        }
        else if (blocked)
        {
            m_idle = 0.0f;
        }
        else if (m_idle > kBreakAfterIdleSeconds)
        {
            const float yaw = ChooserAoYaw(aIn.strafe, aIn.aoYawDeg, CurrentDisableAO());
            const int row = SelectRow(aIn.stance, yaw, m_rng, m_available);
            m_idle = 0.0f;
            if (row < 0)
            {
                out.event = Event::NoValidAnim;
            }
            else
            {
                out.event = Event::StartBreak;
                out.row = row;
                out.clip = kRows[row].clip;
                out.startTime = kRows[row].startTime;
                out.blendTime = kRows[row].blendTime;
                m_phase = Phase::Break;
                m_row = row;
                m_clipTime = kRows[row].startTime;
                m_breakElapsed = 0.0f;
                m_tailRow = -1;
            }
        }

        // advance the active state
        if (m_phase == Phase::Break)
        {
            m_clipTime += dt;
            m_breakElapsed += dt;
        }
        else if (!blocked && out.event != Event::NoValidAnim)
        {
            m_idle += dt;
        }

        out.phase = m_phase;
        out.idleTime = m_idle;
        if (m_phase == Phase::Break)
        {
            if (out.row < 0)
                out.row = m_row;
            const float blend = kRows[m_row].blendTime;
            out.breakClipTime = m_clipTime;
            out.breakWeight = blend > 0.0f ? std::min(1.0f, m_breakElapsed / blend) : 1.0f;
            out.disableAO = out.breakWeight * m_clips[m_row].DisableAOAt(m_clipTime);
        }
        else if (m_tailRow >= 0)
        {
            out.breakClipTime = m_tailTime;
            out.breakWeight = m_tailBlend > 0.0f ? m_tailWeight0 * std::max(0.0f, 1.0f - m_tailElapsed / m_tailBlend) : 0.0f;
            out.disableAO = out.breakWeight * m_clips[m_tailRow].DisableAOAt(m_tailTime);
        }
        m_lastDisableAO = out.disableAO;
        return out;
    }

private:
    float CurrentDisableAO() const { return m_lastDisableAO; }

    void StartTail(float aBlend)
    {
        const float blendIn = kRows[m_row].blendTime;
        m_tailWeight0 = blendIn > 0.0f ? std::min(1.0f, m_breakElapsed / blendIn) : 1.0f;
        m_tailRow = aBlend > 0.0f ? m_row : -1;
        m_tailTime = m_clipTime;
        m_tailElapsed = 0.0f;
        m_tailBlend = aBlend;
        m_phase = Phase::IdleLoop;
        m_row = -1;
    }

    Rng m_rng;
    ClipData m_clips[kRowCount];
    bool m_available[kRowCount] = {};
    Phase m_phase = Phase::IdleLoop;
    float m_idle = 0.0f;
    int m_row = -1;
    float m_clipTime = 0.0f;
    float m_breakElapsed = 0.0f;
    int m_tailRow = -1;
    float m_tailTime = 0.0f;
    float m_tailElapsed = 0.0f;
    float m_tailBlend = 0.0f;
    float m_tailWeight0 = 0.0f;
    float m_lastDisableAO = 0.0f;
    Stance m_lastStance = Stance::Stand;
    bool m_hasLastStance = false;
};
} // namespace utpp::mm::idlebreak

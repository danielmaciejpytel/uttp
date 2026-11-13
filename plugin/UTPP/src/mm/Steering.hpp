// UTPP Phase 5.3c (0.3.5) - steering stability. Header-only, allocation-free per frame, deterministic.
//
// 1. Steering: body-turn hysteresis. The 0.3.4 body-turn path (5.1) let the animation lead every body turn and gave the
//    matcher the raw input target as the desired facing, so during continuous steering (arcs, slalom, circling the stick)
//    a 10-40 deg facing gap persisted and discrete turn / reface clips matched best (clip switch every 0.1-0.15 s in game,
//    2026-10-06 22:08-22:09 recording). Now:
//    - animation-led (0.3.4 law) only for sharp direction changes: |target - body| > enterAngle (or > startAngle while
//      the capsule is slower than startSpeed, i.e. a start towards a target beside / behind the body); it ends when
//      |target - body| < exitAngle;
//    - otherwise follow mode: the body follows the target with a rate-limited spring (BodyYawController follow law) and
//      the matcher's desired facing = predicted body yaw (target minus the steady follow lag targetRate * followTime) with
//      the trajectory curving at the filtered target turn rate (Trajectory turnRate).
// 2. Clip classes from clip names (motion Dense naming) and the per-search exclusion rule (curation):
//    Pattern (Box / Diamond / Hourglass / Prism: strafing direction patterns) never in normal locomotion; Pivot (Pivot,
//    moving Turn_*_180) only when |target - body| >= pivotAngle; MoveTurn (moving Turn_L/R_045/090/135) only while
//    animation-led; Reface (Reface_Start_*) only below refaceSpeed or while animation-led; TurnInPlace (Stand_Turn,
//    Idle_Turn) only below turnInPlaceSpeed. Without stick input pivots / moving turns are excluded too (stop).
// Yaw convention as BodyTurn.hpp: radians, CCW about +Z, wrapped with bt::Wrap.
#pragma once

#include <cctype>
#include <cmath>
#include <string>
#include <cstdint>
#include <string_view>

#include "BodyTurn.hpp"

namespace utpp::mm::steer
{
inline constexpr float kDeg = 3.14159265358979323846f / 180.0f;

// ------------------------------------------------------------------------------------------------ clip classes
enum ClipClass : uint8_t
{
    kClassNone = 0,
    kClassPattern = 1,
    kClassPivot = 2,
    kClassMoveTurn = 3,
    kClassReface = 4,
    kClassTurnInPlace = 5,
    // 0.3.6 (moving-state curation): stop / start / speed-transition clips
    kClassStop = 6,
    kClassStart = 7,
    kClassTransition = 8,
    // 0.3.8: pivots / moving turns split by speed class (prefix Walk_ / Run_ / Sprint_; crouch stays kClassPivot /
    // kClassMoveTurn). All rules that applied to kClassPivot / kClassMoveTurn apply to every variant (PivotMask/TurnMask).
    kClassPivotWalk = 9,
    kClassPivotRun = 10,
    kClassPivotSprint = 11,
    kClassTurnWalk = 12,
    kClassTurnRun = 13,
    kClassTurnSprint = 14,
    // 0.5.2: stance-change transitions (stand <-> crouch: motion Transition_Stand_to_Crouch / Crouch_to_Stand, female non-package
    // WalkToCrouch / RunToCrouch / CrouchToRun / CrouchToWalk, non-package CrouchToStand / CrouchedToSprinting). In game (0.5.1,
    // a former test database) they were chosen during ordinary stand / crouch movement (WalkToCrouch x8, CrouchToRun x5).
    // Allowed only within CurationConfig::stanceWindow after a real stance change (CurationInput::sinceStance).
    kClassStanceTransition = 15,
    // 0.5.3: sprint starts (Sprint_Start_*, Sprint_Reface_Start_* stay kClassReface) - same rules as kClassStart, plus
    // excluded while the motion is not a sprint (in game 0.5.2: Sprint_Start_F chosen while starting a run).
    kClassStartSprint = 16,
    // 0.5.3: curved walk / run clips ("_Arc_": non-package Walk_Arc_*_Move, non-package *_Arc_*_Loop). In game (0.5.2, half-stick
    // straight walk) a non-package clip played for seconds while walking straight. Allowed only while the target turns
    // (CurationInput::turnRate >= CurationConfig::arcTurnRate) or the stick is released.
    kClassArc = 17,
    // 0.5.4: idle clips (stand / crouch idle loops and variations, "_Idle_" without Idle_Turn_). In game (0.5.2,
    // a former test database, crouch at 2.73 m/s with the stick held) a non-package clip
    // was chosen and re-chosen for seconds (cost 4-110). Excluded while the stick is held and the capsule moves
    // (CurationConfig::speedBands, idleSpeed).
    kClassIdle = 18,
    // 0.5.4: speed variants of kClassReface (Walk_ / Run_ / Sprint_Reface_Start_*), assigned by the driver through
    // SpeedVariantClass (ClassifyClip keeps kClassReface). Every rule on kClassReface applies to them (the driver
    // propagates the bit with WithVariants); with speed bands, a moving reface (capsule >= refaceSpeed, animation-led turn) must match the
    // band (in game 0.5.2: Sprint_Reface_Start_F_L_090 and Walk_Reface_Start at 3.85 m/s).
    kClassRefaceWalk = 19,
    kClassRefaceRun = 20,
    kClassRefaceSprint = 21,
    // 0.5.6: direction variants of kClassStanceTransition (driver-side, StanceVariantClass; ClassifyClip keeps
    // kClassStanceTransition). In game (0.5.5, a former test database) a non-package clip (stands
    // the body up) was chosen right after ENTERING crouch (02:54:11.6, 02:54:19.5, 02:54:23.7) and at the slide entry at
    // 10-11 m/s (02:54:59.9, 02:55:09.5). Into-crouch clips only while crouched, out-of-crouch clips only while standing.
    kClassStanceToCrouch = 22,
    kClassStanceFromCrouch = 23,
    // 0.5.6: sideways locomotion by source name (Left / Right / SideStep / Strafe in a loop / move clip). The female {mdir}
    // comes from the root direction and the non-package crouch roots are rotated against the body: a non-package clip
    // CrouchWalkRight moves -36 deg off the hip facing, yet its root says "forward", so it was played for forward crouch
    // walking (02:54:20.2-02:54:23.0, rate 1.40, offset -15..-24 cm, body visibly sideways). Excluded while the desired
    // motion is forward (CurationInput::forward) if the stance has a non-lateral forward loop (CurationInput::lateralOk;
    // the driver sets it for crouch only - the non-package stand "strafe" loops run 6-13 deg off the hip facing).
    kClassLateral = 24,
    // 0.5.7 (driver-side, UTPPSetMMStartTurn): a turning clip (pivot / moving turn / reface / turn in place / start with a
    // root turn >= 30 deg) whose root turns AGAINST the body's turn direction (sign of target - body; the body command
    // always turns the shortest way). Assigned dynamically while an animation-led turn is wanted, excluded then (also as
    // the playing clip). Offline (0.5.6, back start with a 171 deg error): M_Neutral_Stand_Turn_180_R chosen for a left
    // turn -> no animation lead, body turned by the 180 deg/s fallback only (error 81 deg after 0.5 s).
    kClassTurnAgainst = 25,
    // 0.5.7 (driver-side, UTPPSetMMStartTurn): start clip with a slow root turn (>= 45 deg taking > 0.8 s from 10 to 90 %:
    // a non-package clip(Mirror) 95 deg over ~1.3 s, 0.5 s wind-up at < 0.5 m/s, hips at 88 % of the stand
    // height, trunk lean 28 deg). In game (0.5.6, 03:30:13.86 and 03:30:16.06) chosen for side starts at |v| 0 and played at
    // rate 0.40-0.81: ~0.8 s hunched wind-up. Excluded while an animation-led start / turn is wanted.
    kClassSlowTurnStart = 26,
    // 0.5.8 (driver-side, UTPPSetMMLoopBands): a stand / sprint loop / move or speed transition whose speed class (loops: name
    // Walk_ / Run_ / Sprint_; transitions: source of X_to_Y / XToY) is not the speed band of the query speed. Assigned
    // dynamically per band, excluded as a candidate. In game (0.5.7, a former test database) M_Neutral_Walk_Loop_F was chosen
    // at 7.76 m/s (05:11:24.887) and Transition_Walk_to_Run at a steady 3.85 m/s (offset -18..-25 cm).
    kClassOffBand = 27,
    // 0.6.0 (driver-side, UTPPSetMMStanceClipMinSpeed): an into-crouch transition from MOVING locomotion (female non-package
    // RunToCrouch / WalkToCrouch: the source decelerates while crouching - root 5.46 m/s slowing down). Every rule on
    // kClassStanceToCrouch applies (stance masks below); in addition it is a candidate only when the capsule speed at the
    // stand -> crouch change was >= the threshold and the stick >= 0.7 forward. In game (0.5.9, play session 11:44:34-11:46:16)
    // a non-package clip f=2..6 was chosen on every run -> crouch (|v| 3.80-3.86, mv=(0,1)) and read as
    // a "slow down and crouch" animation. In-place transitions (motion Transition_Stand_to_Crouch) keep kClassStanceToCrouch.
    kClassStanceMoveToCrouch = 28,
    // B2 strafe S1 (driver-side, UTPPSetMMStrafe): in the strafe class set (aim strafe, Strafe.hpp ComputeStrafeProfiles) a clip
    // whose ROOT turns >= rootTurnMaxDeg (10 deg) cumulatively, a turn / reface / turn-in-place / arc / slow-turn-start clip,
    // or a Pattern clip off the strafe gait band. Never a candidate nor kept while strafing (CurationInput::strafe).
    kClassStrafeTurn = 29,
};

inline uint32_t Bit(uint8_t c) { return 1u << c; }
inline constexpr uint32_t kPivotMask = (1u << kClassPivot) | (1u << kClassPivotWalk) | (1u << kClassPivotRun) | (1u << kClassPivotSprint);
inline constexpr uint32_t kTurnMask = (1u << kClassMoveTurn) | (1u << kClassTurnWalk) | (1u << kClassTurnRun) | (1u << kClassTurnSprint);
inline bool IsPivot(uint8_t c) { return ((kPivotMask >> c) & 1u) != 0; }
inline bool IsMoveTurn(uint8_t c) { return ((kTurnMask >> c) & 1u) != 0; }
inline constexpr uint32_t kStartMask = (1u << kClassStart) | (1u << kClassStartSprint);
inline constexpr uint32_t kRefaceVariantMask = (1u << kClassRefaceWalk) | (1u << kClassRefaceRun) | (1u << kClassRefaceSprint);
inline bool IsStart(uint8_t c) { return ((kStartMask >> c) & 1u) != 0; }
// Speed class of a clip name: 1 walk, 2 run, 3 sprint, 0 other (crouch, idle).
inline int SpeedClassOfName(std::string_view aName)
{
    if (aName.find("Sprint_") != std::string_view::npos)
        return 3;
    if (aName.find("Run_") != std::string_view::npos)
        return 2;
    if (aName.find("Walk_") != std::string_view::npos)
        return 1;
    return 0;
}

// 0.5.2: stand <-> crouch transition by name (case-insensitive): a Transition clip mentioning crouch, or any clip whose
// name joins crouch with "to" ("ToCrouch", "CrouchTo", "CrouchedTo", "Crouch_to_", "_to_Crouch").
inline bool IsStanceTransitionName(std::string_view aName)
{
    std::string low(aName);
    for (char& ch : low)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    auto has = [&](const char* s) { return low.find(s) != std::string::npos; };
    if (!has("crouch"))
        return false;
    if (has("transition"))
        return true;
    return has("tocrouch") || has("crouchto") || has("crouchedto") || has("crouch_to_") || has("_to_crouch");
}

inline uint8_t ClassifyClip(std::string_view aName)
{
    auto has = [&](std::string_view s) { return aName.find(s) != std::string_view::npos; };
    if (has("_Box_") || has("_Diamond_") || has("_Hourglass_") || has("_Prism_"))
        return kClassPattern;
    if (IsStanceTransitionName(aName))
        return kClassStanceTransition;
    if (has("_Arc_"))
        return kClassArc;
    if (has("Transition_"))
        return kClassTransition;
    if (has("Idle_Turn_") || has("Stand_Turn_"))
        return kClassTurnInPlace;
    if (has("Reface_"))
        return kClassReface;
    if (has("_Pivot_") || (has("_Turn_") && has("_180")))
    {
        const int sc = SpeedClassOfName(aName);
        return sc == 1 ? kClassPivotWalk : (sc == 2 ? kClassPivotRun : (sc == 3 ? kClassPivotSprint : kClassPivot));
    }
    if (has("_Turn_L_") || has("_Turn_R_"))
    {
        const int sc = SpeedClassOfName(aName);
        return sc == 1 ? kClassTurnWalk : (sc == 2 ? kClassTurnRun : (sc == 3 ? kClassTurnSprint : kClassMoveTurn));
    }
    if (has("_Stop"))
        return kClassStop;
    if (has("_Start_"))
        return SpeedClassOfName(aName) == 3 ? kClassStartSprint : kClassStart;
    if (has("_Idle_") || (aName.size() >= 5 && aName.substr(aName.size() - 5) == "_Idle")) // 0.5.4
        return kClassIdle;
    return kClassNone;
}

// 0.5.4: driver-side class with speed variants (reface); any other class is returned unchanged.
inline uint8_t SpeedVariantClass(std::string_view aName, uint8_t aClass)
{
    if (aClass != kClassReface)
        return aClass;
    const int sc = SpeedClassOfName(aName);
    return sc == 1 ? kClassRefaceWalk : (sc == 2 ? kClassRefaceRun : (sc == 3 ? kClassRefaceSprint : aClass));
}

// 0.5.6: direction of a stand <-> crouch transition from its name (case-insensitive): +1 into crouch ("ToCrouch",
// "_to_Crouch", motion Transition_Stand_to_Crouch), -1 out of crouch ("CrouchTo", "CrouchedTo", "Crouch_to_"), 0 unknown.
inline int StanceDirOfName(std::string_view aName)
{
    std::string low(aName);
    for (char& ch : low)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    const bool into = low.find("tocrouch") != std::string::npos || low.find("_to_crouch") != std::string::npos;
    const bool outOf = low.find("crouchto") != std::string::npos || low.find("crouchedto") != std::string::npos ||
                       low.find("crouch_to_") != std::string::npos;
    return into == outOf ? 0 : (into ? 1 : -1);
}

// 0.6.0: into-crouch transition from moving locomotion by name (case-insensitive): "RunToCrouch", "WalkToCrouch",
// "SprintToCrouch" or the "_to_Crouch" spelling with Run / Walk / Sprint as the source.
inline bool IsMovingToCrouchName(std::string_view aName)
{
    std::string low(aName);
    for (char& ch : low)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    for (const char* s : {"runtocrouch", "walktocrouch", "sprinttocrouch", "run_to_crouch", "walk_to_crouch", "sprint_to_crouch"})
        if (low.find(s) != std::string::npos)
            return true;
    return false;
}

// 0.5.6: sideways word in the clip name (source part): Left / Right / SideStep / Strafe / Sideways (case-sensitive words
// as in the non-package names; motion uses single letters L / R / Lfoot and never matches).
inline bool IsLateralName(std::string_view aName)
{
    for (const char* w : {"Left", "Right", "SideStep", "Sidestep", "Strafe", "Sideways"})
        if (aName.find(w) != std::string_view::npos)
            return true;
    return false;
}

// 0.5.7: direction tag of a loop / move clip = the token after "_Loop_" / "_Move_" ({mdir} of the bake: F, B, L, R, FL,
// FR, BL, BR, LL, LR, RL, RR); "" when the name has none.
inline std::string_view LoopDirTagOfName(std::string_view aName)
{
    for (std::string_view key : {std::string_view("_Loop_"), std::string_view("_Move_")})
    {
        const size_t p = aName.find(key);
        if (p == std::string_view::npos)
            continue;
        const size_t a = p + key.size();
        const size_t b = aName.find('_', a);
        return aName.substr(a, (b == std::string_view::npos ? aName.size() : b) - a);
    }
    return {};
}
// 0.5.7: a direction tag that is not forward (F) or a forward diagonal (FL / FR).
inline bool IsSideDirTag(std::string_view aTag)
{
    for (const char* t : {"L", "R", "B", "BL", "BR", "LL", "LR", "RL", "RR"})
        if (aTag == t)
            return true;
    return false;
}
// 0.5.7: crouch loop / move moving sideways or backwards by its direction tag. In game (0.5.6, a former test database,
// 03:29:47.75-03:29:55 and 03:29:55.68-03:30:02) a non-package clip played 7-8 s with the stick forward
// (rate 1.40, offset -18..-25 cm, body diagonal, legs crossing): its {mdir} is L (root moves +43 deg off the hip facing),
// so the 0.5.6 word rule (Left / Right / SideStep / Strafe) did not catch it.
inline bool IsCrouchSideTagName(std::string_view aName)
{
    return aName.find("Crouch_") != std::string_view::npos && IsSideDirTag(LoopDirTagOfName(aName));
}

// 0.5.6: driver-side clip class = SpeedVariantClass + stance-transition direction + lateral loops / moves.
// 0.5.7 (aCrouchDirTag): + crouch loops / moves with a sideways / backwards direction tag.
inline uint8_t DriverClipClass(std::string_view aName, uint8_t aClass, bool aCrouchDirTag = false)
{
    if (aClass == kClassStanceTransition)
    {
        const int d = StanceDirOfName(aName);
        return d > 0 ? kClassStanceToCrouch : (d < 0 ? kClassStanceFromCrouch : aClass);
    }
    if (aClass == kClassNone && (IsLateralName(aName) || (aCrouchDirTag && IsCrouchSideTagName(aName))))
        return kClassLateral;
    return SpeedVariantClass(aName, aClass);
}

// 0.5.7: stance transitions are searchable only in their first frames (local frame <= kStanceStartFrames, 0.2 s at 30 fps).
// In game (0.5.6) a non-package clip was entered at f=27 / 30 / 21 (after its crouching part) and
// re-selected 0.23 s after its first selection (03:29:46.94 f=1 -> 03:29:47.16 f=30).
inline constexpr int32_t kStanceStartFrames = 6;
inline constexpr uint32_t kStanceAllMask = (1u << kClassStanceTransition) | (1u << kClassStanceToCrouch) | (1u << kClassStanceFromCrouch) |
                                           (1u << kClassStanceMoveToCrouch); // 0.6.0
inline bool IsStanceClass(uint8_t c) { return ((kStanceAllMask >> c) & 1u) != 0; }
// 0.5.7: every clip class that turns the body while moving / starting (pivots, moving turns, refaces incl. speed variants).
inline constexpr uint32_t kTurnClassMask = kPivotMask | kTurnMask | (1u << kClassReface) | kRefaceVariantMask;
inline bool IsTurnClass(uint8_t c) { return ((kTurnClassMask >> c) & 1u) != 0; }
// 0.5.8: start segment = a clip that starts / refaces / turns in place from (near) standstill (UTPPSetMMStartBind).
inline bool IsStartSegmentClass(uint8_t c)
{
    return IsStart(c) || c == kClassReface || ((kRefaceVariantMask >> c) & 1u) != 0 || c == kClassTurnInPlace || c == kClassSlowTurnStart;
}
// 0.5.8 turn settle: after a committed turn clip passed 90 % of its root turn, other turn-class clips in the same direction
// stay out for kTurnSettleSeconds (real time) while the remaining error is on the same side (the body-turn fallback finishes
// it). In game (0.5.7, crouch) Crouch_Turn_L_135 f=33 for a 159 deg error reached its 90 % frame with 38 deg left and
// Crouch_Turn_L_045 was started 0.375 s after it (05:09:50.249 -> 50.624); R_090 entered at f=38 -> R_045 after 0.19 s.
inline constexpr float kTurnSettleSeconds = 0.35f;
// 0.5.8 start bind: same-direction commit window of a start segment and the stick change that releases it.
inline constexpr float kStartCommitSeconds = 0.5f;
inline constexpr float kStartReverseDeg = 90.0f;

// 0.5.8 loop speed bands: speed class of a stand / sprint loop / move (class None or lateral, "_Loop" / "_Move_", not crouch) by its
// name, or the SOURCE speed class of a speed transition (kClassTransition: Walk_to_Run / WalkToRun -> walk). 0 = not banded.
inline int LoopBandOfName(std::string_view aName, uint8_t aClass)
{
    if (aName.find("Crouch") != std::string_view::npos || aName.find("crouch") != std::string_view::npos)
        return 0;
    if (aClass == kClassTransition)
    {
        std::string low(aName);
        for (char& ch : low)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        auto src = [&](const char* w) {
            const std::string a = std::string(w) + "_to_", b = std::string(w) + "to";
            return low.find(a) != std::string::npos || low.find(b) != std::string::npos;
        };
        return src("sprint") ? 3 : (src("run") ? 2 : (src("walk") ? 1 : 0));
    }
    if (aClass != kClassNone && aClass != kClassLateral) // lateral (strafe / side-step) loops are banded as well
        return 0;
    if (aName.find("_Loop") == std::string_view::npos && aName.find("_Move_") == std::string_view::npos)
        return 0;
    return SpeedClassOfName(aName);
}
// 0.5.7 standstill idle (no stick or MM gated out, capsule below kStandIdleSpeed): candidates and the playing clip = idle and
// stop clips only (a stop selected while the capsule brakes plays to its end; the capsule brakes below 0.2 m/s in ~0.2 s, often
// before the first stop search). Every other class (loops / moves, starts, transitions, turns, arcs, 25+) is out.
inline constexpr float kStandIdleSpeed = 0.2f;
inline constexpr uint32_t kStandIdleExclude = ~((1u << kClassIdle) | (1u << kClassStop));
inline constexpr uint32_t kStandIdleExcludeCurrent = kStandIdleExclude;

struct CurationConfig
{
    float pivotAngle = 120.0f * kDeg;    // pivots only for a target error at least this large
    float refaceSpeed = 1.0f;            // m/s capsule, reface starts only below
    float turnInPlaceSpeed = 0.5f;       // m/s capsule, turn in place only below
    // 0.3.6 moving-state rules (CurationInput::moving): in game (0.3.5 keyboard slalom) Crouch_Stop was chosen at 2.7 m/s
    // with the stick held, Crouch_Start re-selected mid-run and Transition_Run_to_Walk / Sprint_to_Run while running steady.
    float stopSpeed = 1.0f;              // stick held + capsule above: no stop clips (also not kept as the playing clip)
    float startSpeed = 1.6f;             // stick held + capsule above + startWindow passed: no start clips
    float startWindow = 1.0f;            // s after the stick press during which starts / transitions stay allowed
    float transitionSpeedGap = 0.5f;     // m/s |intent - capsule| below which speed transitions are excluded
    // 0.5.2: stance-change transitions only this long after a stand <-> crouch change (s; < 0 = always allowed = 0.5.1)
    float stanceWindow = 0.6f;
    // 0.5.3 start / slow-walk rules (startRules, default on; off = 0.5.2). In game (a former test database, gamepad):
    // Walk_Turn_R_180 / a non-package clip chosen from standstill (cost 10.8-15.8, offset saturated), Walk_Stop / Walk_Start
    // while slowly walking with the stick held, Sprint_Start while starting a run.
    bool startRules = true;
    float pivotMinSpeed = 0.8f;          // m/s capsule: below, no pivots / moving turns (standstill: reface / turn in place)
    float arcTurnRate = 30.0f * kDeg;    // rad/s |target turn rate|: below, no arc clips while the stick is held
    float turnInPlaceMoving = 0.2f;      // m/s capsule: stick held and moving faster -> no turn-in-place clips (planted feet
                                         // while the capsule walks: 18-21 cm foot-lock corrections in the slow-walk replay)
    // 0.5.4 speed bands (UTPPSetMMSpeedBands; the driver turns it on, the struct default off keeps the 0.5.3 rule set for
    // callers that do not supply CurationInput::speedBand). In game (0.5.2/0.5.3 rule "own speed class or the next slower
    // one"): a non-package clip at 3.85 m/s, a non-package clip / M_Neutral_Walk_Turn_R_180 at 3.05-3.86 m/s.
    // With speedBand > 0 only pivots / moving turns of the band's own class are candidates (next slower class only when
    // the database has none of that kind for the band: CurationInput::pivotFallback / turnFallback); idle clips are out
    // while the stick is held and the capsule is faster than idleSpeed.
    bool speedBands = false;
    float idleSpeed = 0.5f;              // m/s capsule
    // 0.5.6 crouch rules (UTPPSetMMCrouchRules; the driver turns it on, struct default off = 0.5.5 rule set):
    // stance transitions only in the direction of the current stance (CurationInput::stance; also not kept as the playing
    // clip), none above stanceMaxSpeed (sprint -> crouch = slide entry: CrouchToRun at 10-11 m/s in game); lateral loops /
    // moves (kClassLateral) not for forward motion when the stance has a non-lateral forward loop.
    bool crouchRules = false;
    float stanceMaxSpeed = 6.0f;         // m/s capsule
};

struct CurationInput
{
    bool input = false;     // stick held
    bool haveTarget = false;// Locomotion body-turn target valid
    bool animLed = false;   // Steering mode
    float absErr = 0.0f;    // |target - body| (rad)
    float capsule = 0.0f;   // m/s
    // 0.3.6
    bool moving = false;    // moving-state rules on (UTPPSetMMCurationMoving)
    float sinceInput = 1e6f;// s since the stick was pressed / released
    float intent = 0.0f;    // desired game speed (m/s)
    // 0.3.8 large turns: only pivots / moving turns of this speed class or the next slower one (1 walk, 2 run, 3 sprint; 0 = any)
    int speedClass = 0;
    // 0.3.9: slow-walk gate active (capsule below the walk floor, steady): start clips excluded after the start window too
    bool slowWalk = false;
    // 0.5.2: s since the last real stance change (locomotion state stand/sprint <-> crouch)
    float sinceStance = 1e6f;
    // 0.5.3: |turn rate| of the steering target (rad/s, Steering::TargetRate)
    float turnRate = 0.0f;
    // 0.5.4: speed band of the motion (1 walk, 2 run, 3 sprint; 0 = none -> 0.3.8 speedClass rule), SpeedBands.hpp
    int speedBand = 0;
    bool pivotFallback = false; // the database has no pivots of the band's class: the next slower class is allowed
    bool turnFallback = false;  // same for moving turns
    // 0.5.6 (crouchRules)
    int stance = 0;             // 1 stand / sprint, 2 crouch, 0 other (no direction rule)
    bool forward = false;       // stick held and the desired motion within +-45 deg of the body forward
    bool lateralOk = false;     // the stance's clip set has a non-lateral forward loop (else lateral clips stay allowed)
    // B2 strafe S1: aim strafe mode (strafe class set active). The base exclusion is kClassStrafeTurn instead of kClassPattern
    // (facing-fixed Box / Diamond / Hourglass / Prism patterns become candidates), the haveTarget pivot / turn rules are
    // skipped (facing-fixed pivots allowed while moving), every other rule stays. false = the masks of 0.6.5 bit for bit.
    bool strafe = false;
};

// 0.3.8: speed class of the current motion from the capsule / intent speed (game m/s): walk < 2.6, run < 6, sprint.
inline int SpeedClassOfSpeed(float aSpeed) { return aSpeed < 2.6f ? 1 : (aSpeed < 6.0f ? 2 : 3); }

// 0.5.6 crouch rules: classes excluded both as candidates and as the playing clip (see CurationConfig::crouchRules).
inline uint32_t CrouchRuleClasses(const CurationInput& in, const CurationConfig& c)
{
    if (!c.crouchRules)
        return 0u;
    uint32_t m = 0u;
    if (in.stance == 2) // crouched: nothing that stands the body up
        m |= 1u << kClassStanceFromCrouch;
    else if (in.stance == 1) // standing / sprinting: nothing that crouches it
        m |= (1u << kClassStanceToCrouch) | (1u << kClassStanceMoveToCrouch);
    if (in.capsule > c.stanceMaxSpeed) // slide entry / sprint speed: no stance transition at all
        m |= (1u << kClassStanceTransition) | (1u << kClassStanceToCrouch) | (1u << kClassStanceFromCrouch) |
             (1u << kClassStanceMoveToCrouch);
    if (in.input && in.forward && in.lateralOk)
        m |= 1u << kClassLateral;
    return m;
}

// Candidate classes to exclude for this search.
inline uint32_t ExcludedClasses(const CurationInput& in, const CurationConfig& c = {})
{
    uint32_t m = in.strafe ? Bit(kClassStrafeTurn) : Bit(kClassPattern);
    if (!in.input)
        m |= kPivotMask | kTurnMask;
    else if (in.haveTarget && !in.strafe)
    {
        if (!(in.absErr >= c.pivotAngle))
            m |= kPivotMask;
        if (!in.animLed)
            m |= kTurnMask;
    }
    const bool bands = c.speedBands && in.speedBand > 0; // 0.5.4 replaces the 0.3.8 speed-class rule
    if (in.speedClass > 0 && !bands) // 0.3.8: pivots / turns of the other speed classes out
    {
        static constexpr uint8_t kPiv[4] = {0, kClassPivotWalk, kClassPivotRun, kClassPivotSprint};
        static constexpr uint8_t kTrn[4] = {0, kClassTurnWalk, kClassTurnRun, kClassTurnSprint};
        for (int k = 1; k <= 3; ++k) // own class and the next slower one (a pivot decelerates)
            if (k != in.speedClass && k != in.speedClass - 1)
                m |= Bit(kPiv[k]) | Bit(kTrn[k]);
    }
    if (bands) // 0.5.4: only the band's own class (next slower one only as a data fallback)
    {
        static constexpr uint8_t kPiv[4] = {0, kClassPivotWalk, kClassPivotRun, kClassPivotSprint};
        static constexpr uint8_t kTrn[4] = {0, kClassTurnWalk, kClassTurnRun, kClassTurnSprint};
        static constexpr uint8_t kRef[4] = {0, kClassRefaceWalk, kClassRefaceRun, kClassRefaceSprint};
        for (int k = 1; k <= 3; ++k)
        {
            if (!(k == in.speedBand || (k == in.speedBand - 1 && in.pivotFallback)))
                m |= Bit(kPiv[k]);
            if (!(k == in.speedBand || (k == in.speedBand - 1 && in.turnFallback)))
                m |= Bit(kTrn[k]);
            if (k != in.speedBand && !(in.capsule < c.refaceSpeed)) // moving reface: own band only (starts: any)
                m |= Bit(kRef[k]);
        }
    }
    if (c.speedBands && in.input && in.capsule > c.idleSpeed) // 0.5.4: no idle while moving with the stick held
        m |= Bit(kClassIdle);
    if (!(in.capsule < c.refaceSpeed) && !in.animLed) // reface: starts, or a sharp turn (databases without pivots)
        m |= Bit(kClassReface);
    if (!(in.capsule < c.turnInPlaceSpeed))
        m |= Bit(kClassTurnInPlace);
    if (c.stanceWindow >= 0.0f && !(in.sinceStance <= c.stanceWindow)) // 0.5.2
        m |= Bit(kClassStanceTransition);
    m |= CrouchRuleClasses(in, c); // 0.5.6
    if (c.startRules) // 0.5.3
    {
        if (in.input && in.capsule < c.pivotMinSpeed)
            m |= kPivotMask | kTurnMask;
        if (in.input && in.sinceInput > c.startWindow)
            m |= Bit(kClassStop); // stick held: no stop clips at any speed (slow walk)
        if (in.speedClass > 0 && in.speedClass < 3)
            m |= Bit(kClassStartSprint);
        if (in.input && !(std::fabs(in.turnRate) >= c.arcTurnRate))
            m |= Bit(kClassArc);
        if (in.input && in.capsule > c.turnInPlaceMoving)
            m |= Bit(kClassTurnInPlace);
    }
    if (in.moving)
    {
        if (in.input)
        {
            if (in.capsule > c.stopSpeed)
                m |= Bit(kClassStop);
            if (in.sinceInput > c.startWindow)
            {
                // 0.3.9: also in the slow-walk gate (capsule < 1.6): in game Walk_Start_F played ~7 s at rate 0.3
                if (in.capsule > c.startSpeed || in.slowWalk)
                    m |= kStartMask;
                if (std::fabs(in.intent - in.capsule) < c.transitionSpeedGap || in.slowWalk) // 0.3.9: + slow walk
                    m |= Bit(kClassTransition);
            }
        }
        else
            m |= Bit(kClassReface) | Bit(kClassTurnInPlace); // stopping: no turn intent without input
    }
    if (in.strafe) // B2 strafe S1, belt and braces: the strafe class set already maps these classes to kClassStrafeTurn
        m |= kTurnMask | Bit(kClassReface) | Bit(kClassTurnInPlace) | Bit(kClassArc) | Bit(kClassSlowTurnStart);
    return m;
}
// Classes whose playing clip is not kept either (patterns are never valid locomotion; 0.3.6: a stop clip while moving).
inline uint32_t ExcludedCurrentClasses(const CurationInput& in = {}, const CurationConfig& c = {})
{
    uint32_t m = in.strafe ? Bit(kClassStrafeTurn) : Bit(kClassPattern); // B2 strafe S1: patterns may be kept while strafing
    if (in.moving && in.input && in.capsule > c.stopSpeed)
        m |= Bit(kClassStop);
    if (c.startRules && in.input && in.sinceInput > c.startWindow) // 0.5.3: a playing stop clip is left while the stick is held
        m |= Bit(kClassStop);
    if (c.startRules && in.input && !(std::fabs(in.turnRate) >= c.arcTurnRate)) // 0.5.3: no arc clip kept on a straight path
        m |= Bit(kClassArc);
    if (c.startRules && in.input && in.capsule > c.turnInPlaceMoving) // 0.5.3: no turn in place kept while walking
        m |= Bit(kClassTurnInPlace);
    if (in.moving && in.input && in.slowWalk && in.sinceInput > c.startWindow) // 0.3.9: no start / transition kept in a slow walk
        m |= kStartMask | Bit(kClassTransition);
    if (c.speedBands && in.input && in.capsule > c.idleSpeed) // 0.5.4: a playing idle is left as soon as the capsule moves
        m |= Bit(kClassIdle);
    m |= CrouchRuleClasses(in, c); // 0.5.6: wrong-direction stance transitions / sideways loops are not kept either
    if (in.strafe) // B2 strafe S1 (see ExcludedClasses)
        m |= kTurnMask | Bit(kClassReface) | Bit(kClassTurnInPlace) | Bit(kClassArc) | Bit(kClassSlowTurnStart);
    return m;
}
// 0.5.4: for clip classes from SpeedVariantClass - every kClassReface rule applies to the reface speed variants too
// (the driver applies this to both masks; the unit-level masks above stay the 0.5.3 bit sets).
// 0.5.6: likewise every kClassStanceTransition rule (0.5.2 window) applies to both direction variants.
inline constexpr uint32_t kStanceVariantMask = (1u << kClassStanceToCrouch) | (1u << kClassStanceFromCrouch) |
                                               (1u << kClassStanceMoveToCrouch); // 0.6.0
inline uint32_t WithVariants(uint32_t aMask)
{
    if (aMask & Bit(kClassReface))
        aMask |= kRefaceVariantMask;
    if (aMask & Bit(kClassStanceTransition))
        aMask |= kStanceVariantMask;
    return aMask;
}

// ------------------------------------------------------------------------------------------------ stick filter (0.5.3)
// Conditions the raw Move input before the matcher logic (which treats |move| <= 0.15 as "released": stop intent,
// legacy facing). In game (gamepad half stick 0.4-0.55) Locomotion delivered |move| 0.15-0.36, so a slow walk flickered
// between "held" and "released" (Walk_Stop / Walk_Start mid-walk); a 180 deg flick passed through 0 for one sample and a
// stop clip was chosen before the turn (cost 25.9, offset saturated). Held = |move| > press; while held, small magnitudes
// are lifted to minMag (direction kept) and a release must last releaseHold s (the last held vector / world target stay).
struct StickFilterConfig
{
    float press = 0.05f;
    float release = 0.03f;
    float releaseHold = 0.12f; // s
    float minMag = 0.16f;      // just above the 0.15 "released" threshold of BodyTurn.hpp
};

class StickFilter
{
public:
    void Reset() { *this = StickFilter{}; }
    // In/out: move (x, y), haveTarget / targetDeg (Locomotion world target). Returns true while the stick counts as held.
    bool Update(float& aX, float& aY, bool& aHaveTarget, float& aTargetDeg, float aDt, const StickFilterConfig& c = {})
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.25f) : 0.0f;
        const float x = std::isfinite(aX) ? aX : 0.0f, y = std::isfinite(aY) ? aY : 0.0f;
        const float mag = std::sqrt(x * x + y * y);
        if (mag > c.press)
        {
            m_held = true;
            m_releaseTime = 0.0f;
            const float k = mag < c.minMag ? c.minMag / mag : 1.0f;
            aX = x * k;
            aY = y * k;
            m_x = aX;
            m_y = aY;
            if (aHaveTarget && std::isfinite(aTargetDeg))
            {
                m_haveTarget = true;
                m_target = aTargetDeg;
            }
            if (k > 1.0f)
                ++m_lifted;
            return true;
        }
        if (!m_held)
            return false;
        m_releaseTime += dt;
        if (m_releaseTime >= c.releaseHold)
        {
            m_held = false;
            m_haveTarget = false;
            return false;
        }
        if (mag > c.release)
        {
            aX = x * (c.minMag / mag);
            aY = y * (c.minMag / mag);
        }
        else
        {
            aX = m_x;
            aY = m_y;
        }
        if (!aHaveTarget && m_haveTarget)
        {
            aHaveTarget = true;
            aTargetDeg = m_target;
        }
        ++m_bridged;
        return true;
    }
    bool Held() const { return m_held; }
    uint64_t Bridged() const { return m_bridged; } // updates with a bridged release (debounce)
    uint64_t Lifted() const { return m_lifted; }   // updates with a lifted magnitude

private:
    bool m_held = false;
    float m_releaseTime = 0.0f;
    float m_x = 0.0f, m_y = 0.0f;
    bool m_haveTarget = false;
    float m_target = 0.0f;
    uint64_t m_bridged = 0, m_lifted = 0;
};

// ------------------------------------------------------------------------------------------------ steering mode
struct SteerConfig
{
    float enterAngle = 100.0f * kDeg;  // sharp change: animation-led above this error
    float exitAngle = 30.0f * kDeg;    // animation-led ends below this error
    float startAngle = 60.0f * kDeg;   // start (capsule < startSpeed): animation-led above this error
    float startSpeed = 1.0f;           // m/s
    float targetRateHalflife = 0.08f;  // s, smoothing of the target turn rate
    float maxTargetRate = 6.0f;        // rad/s
    float followTime = 0.15f;          // s, must match ControllerConfig::followTime (steady follow lag = rate * time)
    // Matcher-side curvature cap (rad/s MATCHER time). The database has no tight running circles (gentle loop arcs
    // *_Loop_F_L/R_20 at most); predicting the full turn rate (e.g. 90 deg/s real = 128 deg/s matcher time at rate 0.71)
    // made every candidate cost hundreds offline. The body yaw itself is driven externally (follow law), so the query asks
    // for a gentle arc in the turn direction and loops win. Applied to the trajectory curvature and to the measured
    // angular velocity seeded into the prediction while following.
    float maxCurve = 0.15f;
    // 0.3.6: largest facing gap (query facing - body, rad) asked of the matcher in follow mode; 0 = off (0.3.5). A medium
    // target jump (keyboard 45 deg, stick flick) put the whole jump into the query facing; only turn clips contain that, so
    // loops were selected at cost ~116 (game 0.3.5) and crouch fell back to stop clips (cost 230-265). The body still turns
    // by the follow law; the query asks for what loop / arc clips contain.
    float maxFacingGap = 8.0f * kDeg; // 0.3.6 default (UTPPSetMMSteerGap; sweep 8/12/20/30 deg: 8 lowest cost)
};

// 0.3.7 root-motion warp for the offset root: the animation root velocity (character frame, real time) is rotated
// towards the capsule velocity direction by at most aMaxAngle (rad). In game (0.3.6, keyboard slalom) the capsule velocity
// turns at once to the stick direction (velChar 28 deg off the body) while the body turns with a 0.2 s smoothing and
// the animation root goes straight along the body: the lateral error accumulated in the offset root up to the 25 cm clamp
// every 0.9 s ("floating"). Warping keeps only the speed mismatch in the offset (motion-style orientation warping of the
// root motion; the pose itself is unchanged). Below aMinSpeed (either) no warp.
inline void WarpToCapsule(float& aAx, float& aAy, float aCx, float aCy, float aMaxAngle, float aMinSpeed = 0.3f)
{
    const float la = std::sqrt(aAx * aAx + aAy * aAy), lc = std::sqrt(aCx * aCx + aCy * aCy);
    if (!(la > aMinSpeed) || !(lc > aMinSpeed) || !(aMaxAngle > 0.0f))
        return;
    float ang = std::atan2(aAx * aCy - aAy * aCx, aAx * aCx + aAy * aCy); // signed angle anim -> capsule (CCW +)
    ang = std::fmax(-aMaxAngle, std::fmin(aMaxAngle, ang));
    const float c = std::cos(ang), s = std::sin(ang);
    const float x = aAx * c - aAy * s, y = aAx * s + aAy * c;
    aAx = x;
    aAy = y;
}

// Curvature / angular velocity for the matcher (matcher time) in follow mode.
inline float MatcherCurve(float aRateRealTime, float aPlaybackRate, const SteerConfig& c)
{
    if (!std::isfinite(aRateRealTime) || !(aPlaybackRate > 0.0f))
        return 0.0f;
    const float w = aRateRealTime / aPlaybackRate;
    return std::fmax(-c.maxCurve, std::fmin(c.maxCurve, w));
}

inline bool ValidSteer(const SteerConfig& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    return pos(c.enterAngle) && pos(c.exitAngle) && c.exitAngle < c.enterAngle && pos(c.startAngle) &&
           std::isfinite(c.startSpeed) && c.startSpeed >= 0.0f && std::isfinite(c.targetRateHalflife) &&
           c.targetRateHalflife >= 0.0f && pos(c.maxTargetRate) && pos(c.followTime) && pos(c.maxCurve) &&
           std::isfinite(c.maxFacingGap) && c.maxFacingGap >= 0.0f && c.maxFacingGap <= 3.1416f;
}

class Steering
{
public:
    bool SetConfig(const SteerConfig& c)
    {
        if (!ValidSteer(c))
            return false;
        m_cfg = c;
        return true;
    }
    const SteerConfig& Config() const { return m_cfg; }

    void Reset()
    {
        m_valid = m_animLed = m_entered = m_exited = false;
        m_prevTarget = m_rate = 0.0f;
    }

    // Once per locomotion feed with the RAW world target (before the turn latch). Returns AnimLed().
    bool Update(bool aHave, float aTarget, float aBody, float aCapsule, float aDt)
    {
        m_entered = false;
        m_exited = false;
        if (!aHave || !std::isfinite(aTarget) || !std::isfinite(aBody))
        {
            m_valid = m_animLed = false;
            m_rate = 0.0f;
            return false;
        }
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? std::fmin(aDt, 0.1f) : 0.0f;
        if (m_valid && dt > 0.0f)
        {
            float raw = bt::Wrap(aTarget - m_prevTarget) / dt;
            raw = std::fmax(-m_cfg.maxTargetRate, std::fmin(m_cfg.maxTargetRate, raw));
            const float k = m_cfg.targetRateHalflife > 0.0f ? 1.0f - std::exp(-0.69314718f * dt / m_cfg.targetRateHalflife) : 1.0f;
            m_rate += (raw - m_rate) * k;
            if (!std::isfinite(m_rate))
                m_rate = 0.0f;
        }
        else if (!m_valid)
            m_rate = 0.0f;
        m_prevTarget = aTarget;
        m_valid = true;
        const float a = std::fabs(bt::Wrap(aTarget - aBody));
        m_absErr = a;
        const float cap = std::isfinite(aCapsule) ? aCapsule : 0.0f;
        if (!m_animLed)
        {
            if (a > m_cfg.enterAngle || (cap < m_cfg.startSpeed && a > m_cfg.startAngle))
            {
                m_animLed = true;
                m_entered = true;
                ++m_entries;
            }
        }
        else if (a < m_cfg.exitAngle)
        {
            m_animLed = false;
            m_exited = true;
            ++m_exits;
        }
        return m_animLed;
    }

    bool AnimLed() const { return m_animLed; }
    bool Entered() const { return m_entered; } // entered animation-led this update (dwell exemption)
    bool Exited() const { return m_exited; }   // 0.3.8: left animation-led this update (recovery: dwell exemption)
    float TargetRate() const { return m_rate; } // rad/s real time
    float AbsError() const { return m_absErr; }
    uint64_t Entries() const { return m_entries; }
    uint64_t Exits() const { return m_exits; }

    // Follow mode: predicted body yaw for the matcher = target minus the steady follow lag, never beyond the body side.
    float FacingYaw(float aTarget, float aBody) const
    {
        const float err = bt::Wrap(aTarget - aBody);
        float lead = m_rate * m_cfg.followTime;
        if (lead * err <= 0.0f)
            lead = 0.0f;
        else if (std::fabs(lead) > std::fabs(err))
            lead = err;
        if (!(m_cfg.maxFacingGap > 0.0f))
            return bt::Wrap(aTarget - lead); // 0.3.5
        const float rel = std::fmax(-m_cfg.maxFacingGap, std::fmin(m_cfg.maxFacingGap, bt::Wrap(err - lead)));
        return bt::Wrap(aBody + rel);
    }

private:
    SteerConfig m_cfg{};
    bool m_valid = false, m_animLed = false, m_entered = false, m_exited = false;
    float m_prevTarget = 0.0f, m_rate = 0.0f, m_absErr = 0.0f;
    uint64_t m_entries = 0, m_exits = 0;
};
} // namespace utpp::mm::steer

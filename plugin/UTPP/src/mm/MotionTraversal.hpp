#pragma once
// UTPP motion stage 10.8 prep - MotionTraversal: drives one motion traversal montage from a Cyberpunk 2077 climb / vault
// start (PSM Climb / Vault OnEnter). Header only, offline, no game hooks (the seam is not wired here).
//
// Game side (decompiled 2.31 redscript, cyberpunk/player/psm/locomotionTransitions.script = LT):
//  - obstacle = PlayerObstacleSystem.GetCurrentClimbInfo(owner): descResult.topPoint / collisionNormal / topNormal /
//    topExtent / behindPoint / left+rightHandData, obstacleEnd, inputDirection (orphans.script:58976, 27659);
//  - Climb (ClimbEvents.GetClimbParameter LT:2713): obstacleHeight = topPoint.Z - player.Z, High > highThreshold,
//    Mid > midThreshold, else Low; durations = ClimbSpeedModifier * vertical/horizontalDuration<Type> (LT:2750-2751);
//    HorizontalDestination = topPoint + ownerForward * forwardStep; hand IK target per hand = grabPointEnd +
//    0.5 * (grabPointStart - grabPointEnd) (LT:2767/2770);
//  - Vault (VaultEvents.GetVaultParameter LT:3046): landing = (obstacleEnd.xy, max(behindPoint.Z, player.Z)),
//    Destination = landing + inputDirection * forwardStep, depth = topExtent; no duration in script (native body).
//
// motion side: BaseTraversal.hpp DecisionTable (CHT_TraversalAnims) picks the action + montage set, TraversalFile.hpp
// supplies the baked root track and the FrontLedge / BackLedge / BackFloor warp windows, SolveWarp bends the root.
//
// Mapping [P] (offline design, to be confirmed in game):
//  - Climb -> motion Mantle (the game always ends on top): HasFrontLedge, no back ledge, depth = max(topExtent,
//    climbMinMantleDepthM) so the table's depth >= 59 cm Mantle row applies; any other action is Unsupported.
//  - Vault -> motion Hurdle: front + back ledge + back floor (VaultDecisions.FitTest guarantees a floor behind,
//    LT:2901), depth = topExtent, BackLedgeHeight = topPoint.Z - landing Z; Mantle / Vault results are Unsupported
//    (they would not follow the game capsule over the obstacle). Vehicle batch: a vault over a vehicle (kind 1, reason
//    "vehicle") and a Hurdle whose obstacleEnd lies more than hurdleMaxTrueDepthM past topPoint (reason "deep") are
//    Unsupported too, so the game's vault animation plays (Config::vehicleVaultToGame / hurdleMaxTrueDepthM).
//  - MovementMode Walking (1) on the ground, Falling (3) when airborne (catch montages).
//  - Montage: the first montage of the decision (row order) present in the file with solvable windows and a
//    FrontLedge window (motion picks by MotionMatch over the set; no pose history here).
//  - Warp targets: FrontLedge = the game's grab point of the window's contact hand, yaw facing -collisionNormal;
//    BackLedge = that hand point + facing * topExtent at top height; BackFloor = game landing point. Without hand data
//    (vault, low climb: the game IKs no hand) the ROOT is warped to the centred line through topPoint (the hand keeps the
//    clip's lateral offset from the root: topPoint + lateral part of the window offset) and BackFloor = gameDestination
//    (review major 3: forcing each hand onto topPoint slid the root 0.2-0.6 m sideways and snapped it at the handback). Climb adds one root window FrontLedge end -> arrival frame toward HorizontalDestination so the
//    capsule path ends where the game puts the capsule.
//  - Climb game time = ClimbSpeedModifier * (verticalDuration<Type> + horizontalDuration<Type>) [P]: the script
//    only stores the two durations separately (SetVerticalDuration / SetHorizontalDuration, LT:2750-2751); that the
//    native body plays them back to back (sum) is an assumption to confirm in game.
//  - Duration: Climb plays at rate = clip arrival time / game climb time (clamped to [minRate, maxRate]); Vault
//    plays at rate 1 (no game duration). The montage hands back to MM (justTraversed) at the arrival frame
//    (climb) / the last warp window end (vault).
//  - Start frame: the frame (up to the FrontLedge window start) whose distance to the front contact point matches the
//    capsule's (stand-in for the MotionMatch start time).
//  - Gate: a plan whose warped root steps > maxStepM horizontally (per clip frame or per gateDt tick), whose warp adds
//    > maxStepM per clip frame, or whose climb rate had to be clamped is Rejected (the game animation is kept).
// Units: metres, seconds, yaw radians about +Z. Facing convention of the baked .utpptrav root tracks (CP axes): yaw 0
// faces +Y (the clips move along +Y), forward(yaw) = (-sin yaw, cos yaw).
#include "TraversalFile.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace utpp::mm::traversal::v2
{
// Game action that started the traversal (gamePSMDetailedLocomotionStates Climb / Vault).
enum class GameAction : uint8_t
{
    Climb = 8,
    Vault = 9
};

// playerStateMachineLocomotion.climb / .vault static parameters (tweakdb.bin 2.31, read by
// tools/traversal/read_climb_tweaks.py). Runtime should pass the
// live values; the defaults are the 2.31 data, not tuning.
struct GameTweaks
{
    double highThreshold = 1.6;
    double midThreshold = 1.2;
    double verticalDuration[3] = {1.0, 0.6, 0.3}; // High, Mid, Low (CpClimbClass order)
    double horizontalDuration[3] = {0.3, 0.2, 0.1};
    double climbForwardStep = 0.3;
    double vaultForwardStep = 0.5;
};

// Obstacle data readable from redscript at Climb / Vault OnEnter (PlayerClimbInfo + GeometryDescriptionResult).
struct GameObstacle
{
    GameAction action = GameAction::Climb;
    V3 topPoint;        // descResult.topPoint (front edge, top surface)
    V3 collisionNormal; // descResult.collisionNormal (points away from the obstacle toward the player)
    V3 obstacleEnd;     // PlayerClimbInfo.obstacleEnd (vault)
    V3 behindPoint;     // descResult.behindPoint (vault)
    V3 inputDirection;  // PlayerClimbInfo.inputDirection (vault Destination direction, LT:3053/3063; used as given)
    double topExtent = 0; // descResult.topExtent (obstacle depth along the top)
    bool hasHands = false;
    V3 leftHand, rightHand; // grabPointEnd + 0.5 * (grabPointStart - grabPointEnd) per hand
    int kind = 0; // vehicle batch (TRAV-1 B): descResult.climbedEntity, 0 static geometry, 1 vehicle, 2 other entity
};

struct CapsuleState
{
    V3 pos;                       // feet position (DefaultTransition.GetPlayerPosition)
    double yaw = 0;               // owner facing
    double speed = 0;             // horizontal linear speed, m/s
    bool airborne = false;        // Locomotion == 5 / dodgeAir / fall
    double climbSpeedModifier = 1; // gamedataStatType.ClimbSpeedModifier (<= 0 -> 1 as in LT:2746-2749)
};

struct Config
{
    double climbMinMantleDepthM = 0.60; // [P] > 59 cm table edge
    // TR-dll: 0.5 -> 0.40. Field 2026-11: running Mid climbs (Mantle_1_0_run_*, ~0.33 s of clip between the aligned start and the arrival
    // against the game's 0.8 s Mid climb) want rate 0.37-0.45 and were all Rejected(rate); the clip then plays slower than the clip,
    // like the game's own climb does. 0.5 reproduces the old behaviour.
    double minRate = 0.40;              // [P]
    double maxRate = 4.0;               // [P]
    bool warp = true;                   // false = play the clip unwarped (test mutation)
    bool matchDuration = true;          // false = rate 1 for climbs (test mutation)
    bool alignStart = true;             // false = always start at frame 0 (test mutation)
    double backFloorClearM = 0.30;      // [P] vault landing at least this far past the back edge
    bool gate = true;                   // reject plans that break the no-teleport / duration criteria
    double gateDt = 1.0 / 30.0;         // [P] slowest game tick the gate simulates
    double maxStepM = 0.30;             // report 7, stage 10.8: horizontal root step and warp-added step per frame (woman rig)
    // male parity (TR-dll): maxStepM is defined for the woman rig (root length 1.0); the gate limit is maxStepM * RigScale so a
    // rig whose clips are baked smaller (man = woman * 0.944, the hip ratio of the two skeletons) gets the same relative limit
    // instead of the same absolute 0.30 m. false = the old fixed limit for every rig.
    // Measured (motion_traversal_test, motion_knee data): scaling the limit makes the man rig reject MORE (Low 108 -> 81, vault 630 -> 558 accepted),
    // because the warp stretches the smaller rig towards the same world geometry; it only equalises man with woman. Default off.
    bool scaleGateByRig = false;
    double manRigScale = 0.944;         // player_man_skeleton root / limb length relative to player_woman_skeleton
    // TR-dll: a running CLIMB start (capsule speed >= runSpeedMs, the table's run speed row = 250 cm/s) uses the larger limit maxStepRunM
    // (vaults keep maxStepM: their planar step never exceeded it, only the warp-added step did, and that was not investigated).
    // Measured (--diag of motion_traversal_test): the run Mantle clips hold the capsule nearly still while the clip runs in towards the ledge
    // (the capsule is closer than the clip's FrontLedge window start) and then cover the rest in the 3 arrival frames at 0.33-0.37 m per
    // 30 fps tick (10-11 m/s, ~2x the clip's own 5.4 m/s run), independent of height / distance / speed, so the fixed 0.30 m rejected every
    // running Low / Mid climb (field: climbs >= 3 m/s rejected 34 of 38). 0 = the fixed limit for every speed.
    double runSpeedMs = 2.5;
    double maxStepRunM = 0.40;
    // TR-dll vault routing (table untouched, so any installed utpp_traversal_table.json behaves the same):
    //  - wideVault: a game vault the table maps to Mantle (depth >= 59 cm) is evaluated again as "no back floor" (the table's Vault row,
    //    depth clamped under 59 cm) so the Vault_1_0 / Catch_Vault montages are reachable; their landing is a root window to the game
    //    Destination (the clips have no BackFloor window). DEFAULT OFF: the Vault_1_0 / Catch_Vault clips are vault-and-FALL clips (root z
    //    keeps dropping at free-fall speed, -6 .. -12 m by the clip end; at the frame the root passes the start height it still descends
    //    at ~5.7 m/s), the table's own "HasBackFloor = false" row, so the planned hand-back is mid-fall, not a touchdown. The route is kept
    //    behind the key for a drop-off vault / a later landing clip.
    //  - lowHurdle: a drop of lowHurdleMinM .. 50 cm behind the edge (table: no row, CheckFailed) is evaluated as 50 cm -> the Hurdle rows.
    bool wideVault = false;
    bool lowHurdle = true;
    double lowHurdleMinM = 0.10;
    bool centreRootWithoutHands = true; // false = pre-review hand-less targets: every hand on topPoint (test mutation)
    // vehicle batch (TRAV-1 B): a vault over a vehicle (GameObstacle::kind 1) is Unsupported ("vehicle"): motion has no
    // slide-over-hood montage and a Hurdle stretched over a car body passes the legs through it; the game's vault animation
    // plays instead (v1 gate release). Mantle / climb onto a vehicle is unchanged.
    bool vehicleVaultToGame = true;
    // vehicle batch (TRAV-1 C): a Hurdle whose true depth (obstacleEnd - topPoint along the facing) exceeds this is Unsupported
    // ("deep") even when topExtent fits the Hurdle row (fallback when climbedEntity is not filled) [P]
    double hurdleMaxTrueDepthM = 0.90;
    // trav tests only: non-empty = skip the decision table / action mapping and plan this montage of the file
    // (still needs solvable windows and a FrontLedge window); empty (default) = the table decides
    std::string forceMontage;
};

enum class StartResult : uint8_t
{
    Ok,
    BadInput,
    CheckFailed,   // table: no traversal for these inputs
    MontageFailed, // table action without montages / none of them in the file
    Unsupported,   // action does not fit the game action (e.g. Mantle for a vault)
    Rejected       // gate: the warped root would teleport or the climb time cannot be matched -> keep the game anim
};

inline const char* ToString(StartResult r)
{
    switch (r)
    {
    case StartResult::Ok: return "Ok";
    case StartResult::BadInput: return "BadInput";
    case StartResult::CheckFailed: return "CheckFailed";
    case StartResult::MontageFailed: return "MontageFailed";
    case StartResult::Unsupported: return "Unsupported";
    case StartResult::Rejected: return "Rejected";
    }
    return "?";
}

struct Plan
{
    StartResult result = StartResult::BadInput;
    const char* reason = ""; // static string: game-side rule behind an Unsupported plan ("vehicle", "deep") or the gate criterion behind a
                             // Rejected one ("rate", "step", "warp", = rejectReason); "" otherwise
    Decision decision;
    const TravMontage* montage = nullptr;
    CpClimbClass climbClass = CpClimbClass::Low;
    double obstacleHeight = 0;
    double landingZ = 0;
    std::vector<WarpWindow> windows;  // SolveWarp windows (baked windows + climb arrival window)
    std::vector<V3> windowTargets;    // world target per window (warp point target)
    std::vector<V3> windowOffsets;    // warp point offset per window (root space)
    std::vector<int> windowIgnoreZ;
    std::vector<RootKey> warped;      // world root per clip frame (frames before startFrame repeat the start)
    int startFrame = 0;               // montage start frame (distance alignment, motion MotionMatch start time)
    int endFrame = 0;                 // hand-back frame
    double endClipTime = 0;           // endFrame / fps
    double rate = 1;                  // pose time per game second
    double gameDuration = -1;         // climb: ClimbSpeedModifier * (vertical + horizontal) [P] sum, the script stores
                                      // them separately (LT:2750-2751); vault: -1 (not in data)
    bool rateClamped = false;
    double rateWanted = 1;            // clip time / game duration before the [minRate, maxRate] clamp (1 = no duration match)
    const char* rejectReason = "";    // static string: why the gate rejected the plan ("rate", "step", "warp"); "" when not rejected
    double gateStepLimit = 0;         // horizontal step limit the gate applied (Config::maxStepM scaled by the rig), metres
    double gatePlanarStep = 0;        // max horizontal root step per clip frame and per gateDt tick
    double gateWarpAdded = 0;         // max (warped - animated) root step per clip frame
    V3 gameDestination;               // HorizontalDestination (climb) / Destination (vault)
};

struct Frame
{
    RootKey root;       // world root after this tick
    V3 delta;           // root translation this tick
    double poseTime = 0; // montage time to sample the pose at
    bool done = false;  // montage handed back to MM
    bool justTraversed = false; // true on the hand-back tick only (MM chooser JustTraversed)
};

namespace motion_detail
{
// Root-length scale of the rig a traversal file was baked for, relative to the woman rig: "player_man_skeleton" -> manRigScale,
// anything else (woman / unknown) -> 1.
inline double RigScale(const TraversalFile& aFile, const Config& aCfg)
{
    if (!aCfg.scaleGateByRig)
        return 1.0;
    const std::string& n = aFile.rigName;
    return n.find("man") != std::string::npos && n.find("woman") == std::string::npos ? aCfg.manRigScale : 1.0;
}
// Horizontal step limit of the gate (m per clip frame / per gateDt tick): Config::maxStepM scaled by the rig; a running climb start (speed >=
// Config::runSpeedMs) may use the larger Config::maxStepRunM.
inline double StepLimit(const TraversalFile& aFile, const CapsuleState& aC, const Config& aCfg, bool aClimb)
{
    const double base = aCfg.maxStepM * RigScale(aFile, aCfg);
    if (!aClimb || aCfg.maxStepRunM <= 0.0 || !std::isfinite(aC.speed) || aC.speed < aCfg.runSpeedMs)
        return base;
    return std::max(base, aCfg.maxStepRunM);
}
inline V3 Horizontal(V3 v) { return {v.x, v.y, 0}; }
inline V3 Forward(double aYaw) { return {-std::sin(aYaw), std::cos(aYaw), 0}; }
inline double YawOf(V3 aDir) { return std::atan2(-aDir.x, aDir.y); }
inline V3 Normalized(V3 v)
{
    const double l = Len(v);
    return l > 1e-9 ? v * (1.0 / l) : V3{};
}
inline RootKey Sample(const std::vector<RootKey>& aKeys, double aFrame)
{
    if (aKeys.empty())
        return {};
    if (aFrame <= 0)
        return aKeys.front();
    const double last = static_cast<double>(aKeys.size() - 1);
    if (aFrame >= last)
        return aKeys.back();
    const size_t i = static_cast<size_t>(aFrame);
    const double a = aFrame - static_cast<double>(i);
    const RootKey& p = aKeys[i];
    const RootKey& q = aKeys[i + 1];
    return {p.pos + (q.pos - p.pos) * a, p.yaw + WrapPi(q.yaw - p.yaw) * a};
}
// First frame after aFrom whose root height reaches the top of the clip's climb (max z after aFrom - aTolM).
inline int ArrivalFrame(const TravMontage& aM, int aFrom, double aTolM)
{
    const size_t n = aM.root.size();
    double zMax = -1e9;
    for (size_t f = static_cast<size_t>(std::max(aFrom, 0)); f < n; ++f)
        zMax = std::max(zMax, aM.root[f].pos.z);
    for (size_t f = static_cast<size_t>(std::max(aFrom, 0)); f < n; ++f)
        if (aM.root[f].pos.z >= zMax - aTolM)
            return static_cast<int>(f);
    return static_cast<int>(n) - 1;
}
// First frame after the apex (highest root z at or after aFrom) whose root z is back within 3 cm of the clip's start height = the
// touchdown of a vault / drop clip; the last frame when the clip never comes back down.
inline int VaultLandingFrame(const TravMontage& aM, int aFrom)
{
    const size_t n = aM.root.size();
    if (n == 0)
        return 0;
    size_t apex = static_cast<size_t>(std::clamp(aFrom, 0, static_cast<int>(n) - 1));
    for (size_t f = apex; f < n; ++f)
        if (aM.root[f].pos.z > aM.root[apex].pos.z)
            apex = f;
    for (size_t f = apex; f < n; ++f)
        if (aM.root[f].pos.z <= aM.root.front().pos.z + 0.03)
            return static_cast<int>(f);
    return static_cast<int>(n) - 1;
}
} // namespace motion_detail

// Chooser inputs (motion units) for a game climb / vault start.
inline Inputs MakeGameInputs(const GameObstacle& aO, const CapsuleState& aC, const Config& aCfg, double& aHeightM,
                             double& aLandingZ)
{
    aHeightM = aO.topPoint.z - aC.pos.z;
    aLandingZ = std::max(aO.behindPoint.z, aC.pos.z);
    const int mode = aC.airborne ? 3 : 1; // EMovementMode Falling / Walking
    if (aO.action == GameAction::Climb)
        return MakeInputsFromMetres(true, false, false, aHeightM, std::max(aO.topExtent, aCfg.climbMinMantleDepthM), 0.0,
                                    aC.speed, mode);
    return MakeInputsFromMetres(true, true, true, aHeightM, aO.topExtent, aO.topPoint.z - aLandingZ, aC.speed, mode);
}

inline Plan PlanTraversal(const DecisionTable& aTable, const TraversalFile& aFile, const GameObstacle& aO,
                          const CapsuleState& aC, const GameTweaks& aT, const Config& aCfg)
{
    using namespace motion_detail;
    Plan p;
    const V3 facing = Normalized(Horizontal(aO.collisionNormal * -1.0));
    if (Len(facing) < 0.5 || !std::isfinite(aO.topPoint.x + aO.topPoint.y + aO.topPoint.z) || aO.topExtent < 0)
        return p; // BadInput
    const Inputs in = MakeGameInputs(aO, aC, aCfg, p.obstacleHeight, p.landingZ);
    if (p.obstacleHeight <= 0)
        return p;
    p.gateStepLimit = StepLimit(aFile, aC, aCfg, aO.action == GameAction::Climb);
    // TRAV-1 B (vehicle batch). Mutation UTPP_TRAVV2_MUTATION=1 (test builds only): no vehicle rule.
#if !defined(UTPP_TRAVV2_MUTATION) || UTPP_TRAVV2_MUTATION != 1
    if (aO.action == GameAction::Vault && aO.kind == 1 && aCfg.vehicleVaultToGame)
    {
        p.result = StartResult::Unsupported;
        p.reason = "vehicle";
        return p;
    }
#endif
    std::vector<std::string> forced;
    if (aCfg.forceMontage.empty())
    {
        Inputs tin = in;
        // lowHurdle: 10..50 cm behind the edge has no row (BackLedgeHeight >= 50 -> Hurdle, <= 10 -> Mantle): evaluate it as 50 cm
        if (aO.action == GameAction::Vault && aCfg.lowHurdle && in.backLedgeHeight >= aCfg.lowHurdleMinM * 100.0 &&
            in.backLedgeHeight < 50.0)
        {
            tin.backLedgeHeight = 50.0;
        }
        p.decision = aTable.Evaluate(tin);
        bool wide = false;
        if (aO.action == GameAction::Vault && aCfg.wideVault && p.decision.actionName == "Mantle" &&
            in.backLedgeHeight >= aCfg.lowHurdleMinM * 100.0)
        {
            // wide obstacle (table: Mantle): the table's Vault row (no back floor, depth under the 59 cm edge)
            Inputs vin = in;
            vin.hasBackFloor = false;
            vin.obstacleDepth = std::min(in.obstacleDepth, 58.9);
            const Decision dv = aTable.Evaluate(vin);
            if (dv.status == Status::Ok && dv.actionName == "Vault")
            {
                p.decision = dv;
                wide = true;
            }
        }
        if (p.decision.status == Status::CheckFailed)
        {
            p.result = StartResult::CheckFailed;
            return p;
        }
        const std::string want = aO.action == GameAction::Climb ? "Mantle" : (wide ? "Vault" : "Hurdle");
        if (p.decision.actionName != want)
        {
            p.result = StartResult::Unsupported;
            return p;
        }
        // TRAV-1 C (vehicle batch): the Hurdle row fits topExtent, but the obstacle reaches farther along the facing (a car
        // body whose top surface is reported short): a stretched Hurdle would pass the legs through it. Mutation
        // UTPP_TRAVV2_MUTATION=2 (test builds only): no depth rule.
#if !defined(UTPP_TRAVV2_MUTATION) || UTPP_TRAVV2_MUTATION != 2
        if (aO.action == GameAction::Vault && !wide)
        {
            const V3 span = Horizontal(aO.obstacleEnd - aO.topPoint);
            if (span.x * facing.x + span.y * facing.y > aCfg.hurdleMaxTrueDepthM)
            {
                p.result = StartResult::Unsupported;
                p.reason = "deep";
                return p;
            }
        }
#endif
    }
    else
        forced.push_back(aCfg.forceMontage);
    for (const auto& name : aCfg.forceMontage.empty() ? p.decision.montages : forced)
    {
        const TravMontage* m = aFile.Find(name);
        if (!m || !WindowsSolvable(*m) || m->root.size() < 2)
            continue;
        bool front = false;
        for (const auto& w : m->windows)
            front = front || w.target == "FrontLedge";
        if (front)
        {
            p.montage = m;
            break;
        }
    }
    if (!p.montage)
    {
        p.result = StartResult::MontageFailed;
        return p;
    }
    const TravMontage& m = *p.montage;
    const double faceYaw = YawOf(facing);
    const V3 ownerFwd = Forward(aC.yaw);
    p.climbClass = ClassifyCpObstacle(p.obstacleHeight, aT.highThreshold, aT.midThreshold);
    if (aO.action == GameAction::Climb)
    {
        p.gameDestination = aO.topPoint + ownerFwd * aT.climbForwardStep;
        p.gameDestination.z = aO.topPoint.z;
    }
    else // LT:3053/3063: Destination = landing + climbInfo.inputDirection * forwardStep (not the obstacle facing)
        p.gameDestination =
            V3{aO.obstacleEnd.x, aO.obstacleEnd.y, p.landingZ} + aO.inputDirection * aT.vaultForwardStep;
    const bool centred = !aO.hasHands && aCfg.centreRootWithoutHands;
    const V3 lateral{facing.y, -facing.x, 0}; // horizontal, perpendicular to the facing

    auto handPoint = [&](const TravWindow& w) {
        if (!aO.hasHands)
        {
            if (!centred)
                return aO.topPoint;
            // root on the centred line: the warp point keeps its lateral offset from the root (root facing the wall)
            const V3 off = RotZ(w.offset, faceYaw);
            return aO.topPoint + lateral * (off.x * lateral.x + off.y * lateral.y);
        }
        return w.contactBone == "LeftHand" ? aO.leftHand : aO.rightHand;
    };
    std::vector<RootKey> targets;
    int frontEnd = -1, frontWin = -1;
    V3 frontClipContact, frontTarget;
    double frontClipYaw = 0;
    for (const auto& w : m.windows)
    {
        V3 t;
        if (w.target == "FrontLedge")
        {
            t = handPoint(w);
            if (frontEnd < 0)
            {
                frontEnd = w.endFrame;
                frontWin = static_cast<int>(targets.size());
                const RootKey& re = m.root[static_cast<size_t>(w.endFrame)];
                frontClipContact = re.pos + RotZ(w.offset, re.yaw);
                frontClipYaw = re.yaw;
                frontTarget = t;
            }
        }
        else if (w.target == "BackLedge")
        {
            t = handPoint(w) + facing * aO.topExtent;
            t.z = aO.topPoint.z;
        }
        else if (centred) // BackFloor without hand data: the game's landing (Destination / HorizontalDestination)
            t = p.gameDestination;
        else // BackFloor [P]: the clip's own front-contact -> landing root vector, at least backFloorClearM past
        {    // the game's back edge (topExtent), at the game landing height
            const V3 rel = RotZ(m.root[static_cast<size_t>(w.endFrame)].pos - frontClipContact, faceYaw - frontClipYaw);
            t = frontTarget + Horizontal(rel);
            const double ahead = rel.x * facing.x + rel.y * facing.y;
            if (ahead < aO.topExtent + aCfg.backFloorClearM)
                t = t + facing * (aO.topExtent + aCfg.backFloorClearM - ahead);
            t.z = p.landingZ;
        }
        targets.push_back({t, faceYaw});
    }
    p.windows = MakeWarpWindows(m, targets);
    for (size_t i = 0; i < p.windows.size(); ++i)
    {
        p.windowTargets.push_back(p.windows[i].target);
        p.windowOffsets.push_back(p.windows[i].warpPointOffset);
        p.windowIgnoreZ.push_back(p.windows[i].ignoreZ ? 1 : 0);
    }
    if (aO.action == GameAction::Climb)
    {
        p.endFrame = std::max(ArrivalFrame(m, frontEnd, 0.03), frontEnd);
        const int lastEnd = p.windows.empty() ? 0 : p.windows.back().endFrame;
        if (p.endFrame > lastEnd && p.endFrame > frontEnd)
        {
            WarpWindow ww;
            ww.startFrame = std::max(lastEnd, frontEnd);
            ww.endFrame = p.endFrame;
            ww.target = p.gameDestination;
            ww.targetYaw = faceYaw;
            ww.warpYaw = false;
            p.windows.push_back(ww);
            p.windowTargets.push_back(ww.target);
            p.windowOffsets.push_back({});
            p.windowIgnoreZ.push_back(0);
        }
        const double mod = aC.climbSpeedModifier > 0 ? aC.climbSpeedModifier : 1.0;
        const int k = static_cast<int>(p.climbClass);
        // [P] sum: LT:2750-2751 only stores the two durations separately; back-to-back playback is assumed
        p.gameDuration = mod * (aT.verticalDuration[k] + aT.horizontalDuration[k]);
    }
    else
    {
        p.endFrame = p.windows.empty() ? m.frames - 1 : p.windows.back().endFrame;
        // TR-dll: a vault montage without a BackFloor window (Vault_1_0_*, Catch_Vault_*: the clip only has FrontLedge / BackLedge
        // windows and carries on across the obstacle) would hand back to MM in mid-air at the last window. Add a root window from
        // there to the clip's touchdown frame towards the game's landing (Destination) and hand back at the touchdown.
        bool floorWindow = false;
        for (const auto& w : m.windows)
            floorWindow = floorWindow || w.target == "BackFloor";
        if (!floorWindow && aCfg.wideVault && frontEnd >= 0)
        {
            const int lastEnd = std::max(p.endFrame, frontEnd);
            const int land = VaultLandingFrame(m, lastEnd);
            if (land > lastEnd)
            {
                WarpWindow ww;
                ww.startFrame = lastEnd;
                ww.endFrame = land;
                ww.target = p.gameDestination;
                ww.targetYaw = faceYaw;
                ww.warpYaw = false;
                p.windows.push_back(ww);
                p.windowTargets.push_back(ww.target);
                p.windowOffsets.push_back({});
                p.windowIgnoreZ.push_back(0);
                p.endFrame = land;
            }
        }
    }
    p.endFrame = std::clamp(p.endFrame, 1, m.frames - 1);
    // Start frame [P]: motion starts the montage where the pose search matches (start time from MotionMatch); here the
    // frame whose distance to the front contact point (along the clip facing at the window end) best matches the
    // capsule's distance to the game's contact point, searched up to the FrontLedge window start.
    p.startFrame = 0;
    if (aCfg.alignStart && frontWin >= 0)
    {
        const WarpWindow& fw = p.windows[static_cast<size_t>(frontWin)];
        const RootKey& re = m.root[static_cast<size_t>(fw.endFrame)];
        const V3 clipContact = re.pos + RotZ(fw.warpPointOffset, re.yaw);
        const V3 clipFwd = Forward(re.yaw);
        const V3 toTarget = fw.target - aC.pos;
        const double wantDist = toTarget.x * facing.x + toTarget.y * facing.y;
        double best = 1e9;
        for (int f = 0; f <= fw.startFrame; ++f)
        {
            const V3 d = clipContact - m.root[static_cast<size_t>(f)].pos;
            const double err = std::fabs(d.x * clipFwd.x + d.y * clipFwd.y - wantDist);
            if (err < best - 1e-9)
            {
                best = err;
                p.startFrame = f;
            }
        }
    }
    p.endClipTime = static_cast<double>(p.endFrame) / m.fps;
    p.rate = 1.0;
    if (p.gameDuration > 0 && aCfg.matchDuration)
    {
        const double r = (p.endClipTime - static_cast<double>(p.startFrame) / m.fps) / p.gameDuration;
        p.rateWanted = r;
        p.rate = std::clamp(r, aCfg.minRate, aCfg.maxRate);
        p.rateClamped = p.rate != r;
    }
    // solve on the clip from startFrame (start key re-based to the capsule), then pad the skipped frames
    const size_t s = static_cast<size_t>(p.startFrame);
    std::vector<RootKey> sub(m.root.begin() + static_cast<std::ptrdiff_t>(s), m.root.end());
    std::vector<WarpWindow> subWin;
    if (aCfg.warp)
        for (WarpWindow w : p.windows)
        {
            w.startFrame = std::max(w.startFrame - p.startFrame, 0);
            w.endFrame -= p.startFrame;
            if (w.endFrame > 0)
                subWin.push_back(w);
        }
    const RootKey start{aC.pos, aC.yaw};
    const std::vector<RootKey> solved = SolveWarp(sub, start, subWin);
    p.warped.assign(s, start);
    p.warped.insert(p.warped.end(), solved.begin(), solved.end());
    // gate metrics: per clip frame (start..end) and per gateDt tick at the play rate
    for (size_t f = s + 1; f < p.warped.size() && f <= static_cast<size_t>(p.endFrame); ++f)
    {
        const V3 d = p.warped[f].pos - p.warped[f - 1].pos;
        const V3 anim = RotZ(RotZ(m.root[f].pos - m.root[f - 1].pos, -m.root[f - 1].yaw), p.warped[f - 1].yaw);
        p.gatePlanarStep = std::max(p.gatePlanarStep, std::hypot(d.x, d.y));
        p.gateWarpAdded = std::max(p.gateWarpAdded, Len(d - anim));
    }
    {
        const double t0 = static_cast<double>(p.startFrame) / m.fps;
        RootKey prev = p.warped[s];
        for (double t = t0; t < p.endClipTime - 1e-9;)
        {
            t = std::min(t + aCfg.gateDt * p.rate, p.endClipTime);
            const RootKey r = Sample(p.warped, t * m.fps);
            p.gatePlanarStep = std::max(p.gatePlanarStep, std::hypot(r.pos.x - prev.pos.x, r.pos.y - prev.pos.y));
            prev = r;
        }
    }
    p.result = StartResult::Ok;
    if (aCfg.gate)
    {
        // reason in priority order rate > step > warp: the first failing criterion explains the rejection in the log
        if (p.rateClamped)
            p.rejectReason = "rate";
        else if (p.gatePlanarStep > p.gateStepLimit)
            p.rejectReason = "step";
        else if (p.gateWarpAdded > p.gateStepLimit)
            p.rejectReason = "warp";
        if (p.rejectReason[0])
        {
            p.result = StartResult::Rejected;
            p.reason = p.rejectReason; // the "UTPP trav:" line prints result=Rejected(<reason>)
        }
    }
    return p;
}

// Per-tick player of a planned traversal: root motion + pose time until the hand-back frame.
class Player
{
public:
    explicit Player(const Plan& aPlan) : m_plan(&aPlan)
    {
        if (!aPlan.warped.empty())
            m_root = aPlan.warped.front();
        if (aPlan.montage)
            m_time = static_cast<double>(aPlan.startFrame) / aPlan.montage->fps;
    }
    Frame Tick(double aDt)
    {
        Frame f;
        const Plan& p = *m_plan;
        if (p.result != StartResult::Ok || !p.montage || m_done)
        {
            f.root = m_root;
            f.poseTime = m_time;
            f.done = true;
            return f;
        }
        m_time = std::min(m_time + std::max(aDt, 0.0) * p.rate, p.endClipTime);
        const RootKey r = motion_detail::Sample(p.warped, m_time * p.montage->fps);
        f.delta = r.pos - m_root.pos;
        f.root = r;
        f.poseTime = m_time;
        m_root = r;
        if (m_time >= p.endClipTime - 1e-9)
        {
            m_done = true;
            f.done = true;
            f.justTraversed = true;
        }
        return f;
    }

private:
    const Plan* m_plan;
    RootKey m_root;
    double m_time = 0;
    bool m_done = false;
};

// World warp point of window i of the plan at its end frame minus its target (ignoreZ respected), metres.
inline double WindowContactError(const Plan& aP, size_t aI)
{
    if (aI >= aP.windows.size() || aP.warped.empty())
        return 1e9;
    const WarpWindow& w = aP.windows[aI];
    const RootKey& r = aP.warped[static_cast<size_t>(std::clamp(w.endFrame, 0, static_cast<int>(aP.warped.size()) - 1))];
    V3 e = r.pos + RotZ(aP.windowOffsets[aI], r.yaw) - aP.windowTargets[aI];
    if (aP.windowIgnoreZ[aI])
        e.z = 0;
    return Len(e);
}
} // namespace utpp::mm::traversal::v2

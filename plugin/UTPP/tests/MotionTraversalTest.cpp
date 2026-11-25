// UTPP motion stage 10.8 prep - offline test of src/mm/MotionTraversal.hpp (game climb / vault start -> motion montage,
// warped root motion, pose time, hand-back to MM). Data is motion-derived and outside git.
// Usage: utpp_motion_traversal_test <traversal_table.json> <utpp_traversal_woman.utpptrav> <utpp_traversal_man.utpptrav>
// Synthetic obstacles span the game's ranges (the traversal integration progress log):
//  - climb heights across Low / Mid / High (midThreshold 1.2, highThreshold 1.6 m) up to the motion Mantle limit 2.75 m,
//    standing / walking / running, facing offsets within inputAngleThreshold 30 deg, capsule 0.45-1.0 m from the edge
//    (capsuleRadius 0.4, climb detectionRange 1.0), with and without game hand grab points, plus airborne (catch);
//  - vault heights 0.5-1.2 m (motion Hurdle >= 50 cm back ledge height, <= 125 cm), depth 0.1-0.55 m (< 59 cm, game
//    maxExtent 2.1), landing 0 / +0.2 m (FitTest <= 0.40 m), speeds 3.5-7 m/s (vault minSpeed 3.5), distance within
//    the game's ObstacleLengthCheck range.
// Checks [report 7, stage 10.8 criteria]: contact hand <= 5 cm from the game's hand point at every FrontLedge /
// BackLedge window end; horizontal root step <= 0.30 m per game tick (60 and 30 fps) and per clip frame; warp-added
// root step <= 0.30 m per clip frame; climb: hand-back time = game climb time (ClimbSpeedModifier * (vertical +
// horizontal duration)) within one tick and the root ends on the game's HorizontalDestination (<= 2 cm); vault: plan
// Destination = game formula (inputDirection); expected time / destination / class are computed here from the tweak
// values and LT formulas (GameExpect), not read from the plan; accepted plans per class >= the recorded minimum
// (kMinAccepted); hand-back fires exactly once with justTraversed. Mutations (warp off, injected teleport, injected
// warp jump, duration match off, 1 m/frame drift) must make the corresponding check fail. Vehicle batch (VehicleRules): a vault
// over a vehicle -> Unsupported(vehicle), obstacleEnd > 0.90 m past topPoint -> Unsupported(deep); controls stay Ok. Build
// mutations UTPP_TRAVV2_MUTATION=1 / 2 (tests/motion_traversal.cmake) must fail them.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <array>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/MotionTraversal.hpp"

using namespace utpp::mm::traversal;
using namespace utpp::mm::traversal::v2;

static int g_pass = 0, g_fail = 0;
static Config g_baseCfg; // config of the grid cases
static bool g_diag = false; // --diag: one line per planned case (rig, case, montage, result, reason, wanted rate, steps, limit)
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}

static bool ReadAll(const char* aPath, std::string& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    aOut = ss.str();
    return true;
}

static constexpr double kContactMax = 0.05;   // [P] report 7, stage 10.8
static constexpr double kStepMax = 0.30;      // [P] report 7, stage 10.8
// TR-dll: a running CLIMB start (>= 2.5 m/s, the table's run row) is allowed 0.40 m per tick, everything slower and every vault the old 0.30 m;
// written out here from the design, not read from the plan.
static constexpr double kRunSpeed = 2.5;
static constexpr double kStepMaxRun = 0.40;
static double ExpectedStepLimit(double aSpeed, bool aClimb)
{
    return aClimb && aSpeed >= kRunSpeed ? kStepMaxRun : kStepMax;
}
static constexpr double kDestMax = 0.02;      // [P] climb end on HorizontalDestination
// Minimum accepted (Ok) plans per rig (woman, man) and class (climb High, Mid, Low, vault) out of 218 / 164 / 166 /
// 756 cases (vault: 378 with hand data + 378 without, as the game sends it): the counts of the run on 2026-10-13 after the
// next review fix (hand-less root centred, data traversal_table.json + utpptrav woman / man). A change that
// silently rejects more plans (gate or formula) fails here; raise the numbers when the acceptance improves.
// TR-dll: classes 4 / 5 = wide vault (depth 0.6-2.0 m -> Vault_1_0 montages) and low hurdle (drop 0.15-0.45 m); the first four are over the
// extended speed grid (0-10 m/s). Recorded after the TR-dll planner change (minRate 0.40, 0.40 m run-climb step limit, low hurdle on; wide
// vault is measured with its key on), as the minimum over the v1, v2 and motion_knee data files; `kLegacyAccepted` is the pre-change result of the original grid (speeds 0 / 1.5 / 4 m/s, original vault cases) that the
// all-new-keys-off configuration must still reproduce exactly.
// With the rejected fallback (Config::rejectedFallback: other montages / neighbouring table rows under the same gate) every grid
// case of these six classes has an accepted plan; before it: woman 504 / 378 / 378 / 588 / 279 / 159, man 504 / 379 / 378 / 624 / 288 / 165.
static constexpr int kMinAccepted[2][6] = {{506, 380, 382, 756, 324, 288}, {506, 380, 382, 756, 324, 288}};
// [min, max]: the man vault count is 624 on the v1 file (ctest) and 630 on the motion_knee file; everything else is exact
static constexpr int kLegacyAccepted[2][4] = {{216, 126, 108, 588}, {216, 126, 108, 624}};
static constexpr int kLegacyAcceptedMax[2][4] = {{216, 126, 108, 588}, {216, 126, 108, 630}};

// Game values used for the expected results, written out here (not taken from GameTweaks / the plan) so a wrong
// formula in MotionTraversal.hpp cannot hide itself. Source: a dump of the game climb tweaks (tweakdb.bin 2.31).
static constexpr double kGameHighThreshold = 1.6;            // climb.highThreshold
static constexpr double kGameMidThreshold = 1.2;             // climb.midThreshold
static constexpr double kGameVertical[3] = {1.0, 0.6, 0.3};  // climb.verticalDuration High / Mid / Low
static constexpr double kGameHorizontal[3] = {0.3, 0.2, 0.1}; // climb.horizontalDuration High / Mid / Low
static constexpr double kGameClimbForwardStep = 0.3;         // climb.forwardStep
static constexpr double kGameVaultForwardStep = 0.5;         // vault.forwardStep

// Expected game result for a case, from the game formulas (LT = decompiled 2.31 locomotionTransitions.script):
//  - climb class: height = topPoint.Z - player.Z, > highThreshold High (0), > midThreshold Mid (1), else Low (2)
//    (LT:2730-2743);
//  - climb time = ClimbSpeedModifier * (verticalDuration<Type> + horizontalDuration<Type>) (LT:2746-2751; the sum is
//    [P], the script stores the two separately);
//  - HorizontalDestination = topPoint + ownerForward * forwardStep (LT:2727), forward(yaw) = (-sin, cos) (+Y at 0);
//  - vault Destination = (obstacleEnd.xy, max(behindPoint.Z, player.Z)) + inputDirection * forwardStep (LT:3058-3063).
struct Expect
{
    int climbClass = -1; // -1 = vault
    double duration = -1;
    V3 dest;
};

static Expect GameExpect(const GameObstacle& aO, const CapsuleState& aC)
{
    Expect e;
    if (aO.action == GameAction::Climb)
    {
        const double h = aO.topPoint.z - aC.pos.z;
        e.climbClass = h > kGameHighThreshold ? 0 : (h > kGameMidThreshold ? 1 : 2);
        const double mod = aC.climbSpeedModifier > 0 ? aC.climbSpeedModifier : 1.0; // [P] stat default 1
        e.duration = mod * (kGameVertical[e.climbClass] + kGameHorizontal[e.climbClass]);
        e.dest = {aO.topPoint.x - std::sin(aC.yaw) * kGameClimbForwardStep,
                  aO.topPoint.y + std::cos(aC.yaw) * kGameClimbForwardStep, aO.topPoint.z};
    }
    else
    {
        const double z = std::max(aO.behindPoint.z, aC.pos.z);
        e.dest = {aO.obstacleEnd.x + aO.inputDirection.x * kGameVaultForwardStep,
                  aO.obstacleEnd.y + aO.inputDirection.y * kGameVaultForwardStep,
                  z + aO.inputDirection.z * kGameVaultForwardStep};
    }
    return e;
}

struct Metrics
{
    double contact = 0;    // max hand window contact error
    double planarTick = 0; // max horizontal root step per game tick
    double planarClip = 0; // max horizontal root step per clip frame (warped)
    double warpAdded = 0;  // max (warped step - animated step) per clip frame
    double durErr = 0;     // |hand-back time - expected game climb time| (climb)
    double destErr = 0;    // climb: root at hand-back vs expected HorizontalDestination
    double planDurErr = 0; // |plan gameDuration - expected| (climb)
    double planDestErr = 0; // |plan gameDestination - expected| (climb and vault)
    // review major 3 (no hand data): root distance from the centred line through topPoint along the facing, and its bound
    // from the sources only (capsule start / game destination off the line + the clip's own lateral root excursion)
    double lateral = 0, lateralBound = 1e9;
    double landErr = 0; // vault without hand data: root at the hand-back vs the game Destination
    int doneCount = 0;
    int ticks = 0;
    bool justTraversedOnDone = false;
};

static double PlanarLen(V3 d) { return std::hypot(d.x, d.y); }

static void ClipMetrics(const Plan& aP, const std::vector<RootKey>& aWarped, Metrics& aM)
{
    const auto& clip = aP.montage->root;
    for (size_t f = static_cast<size_t>(aP.startFrame) + 1; f < aWarped.size() && f <= static_cast<size_t>(aP.endFrame);
         ++f)
    {
        const V3 d = aWarped[f].pos - aWarped[f - 1].pos;
        const V3 anim = RotZ(RotZ(clip[f].pos - clip[f - 1].pos, -clip[f - 1].yaw), aWarped[f - 1].yaw);
        aM.planarClip = std::max(aM.planarClip, PlanarLen(d));
        aM.warpAdded = std::max(aM.warpAdded, Len(d - anim));
    }
}

static std::vector<Frame> Run(const Plan& aP, const Expect& aE, double aDt, Metrics& aM)
{
    std::vector<Frame> out;
    Player pl(aP);
    for (int i = 0; i < 2000; ++i)
    {
        const Frame f = pl.Tick(aDt);
        out.push_back(f);
        if (f.done)
            break;
    }
    // one extra tick after hand-back: no motion, no second justTraversed
    const Frame after = pl.Tick(aDt);
    for (const Frame& f : out)
    {
        aM.planarTick = std::max(aM.planarTick, PlanarLen(f.delta));
        aM.doneCount += f.justTraversed ? 1 : 0;
    }
    aM.doneCount += after.justTraversed ? 1 : 0;
    Check(after.done && Len(after.delta) == 0, "tick after hand-back moves the root");
    aM.ticks = static_cast<int>(out.size());
    aM.justTraversedOnDone = !out.empty() && out.back().done && out.back().justTraversed;
    if (aE.climbClass >= 0)
    {
        aM.durErr = std::fabs(aM.ticks * aDt - aE.duration);
        aM.destErr = out.empty() ? 1e9 : Len(out.back().root.pos - aE.dest);
    }
    else
        aM.landErr = out.empty() ? 1e9 : Len(out.back().root.pos - aE.dest);
    return out;
}

struct Case
{
    std::string tag;
    GameObstacle o;
    CapsuleState c;
};

static Case MakeClimb(double aH, double aSpeed, double aDist, double aYawOff, bool aHands, bool aAir)
{
    Case k;
    k.o.action = GameAction::Climb;
    // obstacle face across +Y at y = aDist from the capsule (feet at origin), normal -Y; capsule yaw 0 faces +Y
    // (.utpptrav convention), plus the yaw offset; left of +Y is -X
    k.c.pos = {0, 0, 0};
    k.c.yaw = aYawOff;
    k.c.speed = aSpeed;
    k.c.airborne = aAir;
    k.o.topPoint = {0.1, aDist, aH};
    k.o.collisionNormal = {0, -1, 0};
    k.o.topExtent = 1.5;
    k.o.obstacleEnd = {0.1, aDist + 1.5, aH};
    k.o.behindPoint = {0.1, aDist + 1.5, aH};
    k.o.inputDirection = {-std::sin(aYawOff), std::cos(aYawOff), 0};
    k.o.hasHands = aHands;
    k.o.leftHand = {0.1 - 0.22, aDist, aH};
    k.o.rightHand = {0.1 + 0.22, aDist, aH};
    char b[160];
    std::snprintf(b, sizeof b, "climb h %.2f v %.1f d %.2f yaw %+.2f%s%s", aH, aSpeed, aDist, aYawOff,
                  aHands ? " hands" : "", aAir ? " air" : "");
    k.tag = b;
    return k;
}

static Case MakeVault(double aH, double aDepth, double aLandDz, double aSpeed, double aDist, double aYawOff,
                      bool aHands = true)
{
    Case k;
    k.o.action = GameAction::Vault;
    k.c.pos = {0, 0, 0};
    k.c.yaw = aYawOff;
    k.c.speed = aSpeed;
    k.o.topPoint = {-0.05, aDist, aH};
    k.o.collisionNormal = {0, -1, 0};
    k.o.topExtent = aDepth;
    k.o.obstacleEnd = {-0.05, aDist + aDepth, aH};
    k.o.behindPoint = {-0.05, aDist + aDepth + 0.3, aLandDz};
    // stick input along the capsule facing (differs from -collisionNormal when yaw != 0, so a facing-based
    // Destination is caught by the plan destination check)
    k.o.inputDirection = {-std::sin(aYawOff), std::cos(aYawOff), 0};
    k.o.hasHands = aHands; // the game's vault sends no hand data (utpp_motion.reds VaultEvents wrap)
    k.o.leftHand = {-0.05 - 0.22, aDist, aH};
    k.o.rightHand = {-0.05 + 0.22, aDist, aH};
    char b[160];
    std::snprintf(b, sizeof b, "vault h %.2f depth %.2f land %+.2f v %.1f d %.2f yaw %+.2f%s", aH, aDepth, aLandDz,
                  aSpeed, aDist, aYawOff, aHands ? " hands" : "");
    k.tag = b;
    return k;
}

// Evaluate one case with one config; returns false if the plan did not start.
static bool Evaluate(const DecisionTable& aT, const TraversalFile& aF, const Case& aC, const Config& aCfg, Plan& aP,
                     Metrics& a60, Metrics& a30)
{
    aP = PlanTraversal(aT, aF, aC.o, aC.c, GameTweaks{}, aCfg);
    if (aP.result != StartResult::Ok)
        return false;
    // contact hand vs the game's hand point, computed here from the obstacle (independent of the plan's targets):
    // FrontLedge = grab point of the window's hand (topPoint without hand data), BackLedge = that point + facing *
    // topExtent at top height; facing = -collisionNormal (horizontal)
    const V3 facing{-aC.o.collisionNormal.x, -aC.o.collisionNormal.y, 0};
    const V3 lat{facing.y, -facing.x, 0}; // horizontal, perpendicular to the facing (facing is unit length here)
    auto latOf = [&](V3 aV) { return aV.x * lat.x + aV.y * lat.y; };
    for (size_t i = 0; i < aP.montage->windows.size(); ++i)
    {
        const TravWindow& w = aP.montage->windows[i];
        if (w.target == "BackFloor")
            continue;
        V3 want = !aC.o.hasHands ? aC.o.topPoint : (w.contactBone == "LeftHand" ? aC.o.leftHand : aC.o.rightHand);
        if (w.target == "BackLedge")
        {
            want = want + facing * aC.o.topExtent;
            want.z = aC.o.topPoint.z;
        }
        const RootKey& r = aP.warped[static_cast<size_t>(w.endFrame)];
        V3 e = r.pos + RotZ(w.offset, r.yaw) - want;
        if (!aC.o.hasHands) // no game hand point: the hand keeps its lateral place, checked along the facing / height only
            e = e - lat * latOf(e);
        if (w.ignoreZ)
            e.z = 0;
        a60.contact = std::max({a60.contact, Len(e), aC.o.hasHands ? WindowContactError(aP, i) : 0.0});
    }
    if (!aC.o.hasHands)
    {
        // review major 3: no lateral root slide. Bound from the sources: the farther of capsule start / game destination
        // from the centred line, plus the clip's own lateral root excursion (clip start facing) over the played frames,
        // plus 2 cm. The pre-review targets (each hand on topPoint) shift the root by the hand's 0.2-0.6 m lateral offset.
        const Expect ge = GameExpect(aC.o, aC.c);
        const auto& clip = aP.montage->root;
        const size_t s0 = static_cast<size_t>(aP.startFrame);
        double clipLat = 0;
        for (size_t f = s0; f < clip.size() && f <= static_cast<size_t>(aP.endFrame); ++f)
            clipLat = std::max(clipLat, std::fabs(RotZ(clip[f].pos - clip[s0].pos, -clip[s0].yaw).x));
        a60.lateralBound = std::max(std::fabs(latOf(aC.c.pos - aC.o.topPoint)), std::fabs(latOf(ge.dest - aC.o.topPoint))) +
                           clipLat + 0.02;
        for (size_t f = s0; f < aP.warped.size() && f <= static_cast<size_t>(aP.endFrame); ++f)
            a60.lateral = std::max(a60.lateral, std::fabs(latOf(aP.warped[f].pos - aC.o.topPoint)));
    }
    ClipMetrics(aP, aP.warped, a60);
    const Expect e = GameExpect(aC.o, aC.c);
    a60.planDestErr = Len(aP.gameDestination - e.dest);
    a60.planDurErr = e.climbClass >= 0 ? std::fabs(aP.gameDuration - e.duration) : 0.0;
    Run(aP, e, 1.0 / 60.0, a60);
    Run(aP, e, 1.0 / 30.0, a30);
    return true;
}

struct Totals
{
    int cases = 0, started = 0;
    std::map<std::string, int> alternatives; // accepted plans by Plan::alternative ("" = direct row evaluation)
    double contact = 0, planarTick = 0, planarClip = 0, warpAdded = 0, durErr60 = 0, destErr = 0, minRate = 1e9,
           maxRate = 0;
    int clamped = 0;
    int rejected = 0;
    double lateral = 0, landErr = 0; // review major 3 (no hand data)
    int noHands = 0;
    int accepted[6] = {0, 0, 0, 0, 0, 0}; // climb High, Mid, Low, vault, wide vault, low hurdle
    int total[6] = {0, 0, 0, 0, 0, 0};
    // TR-dll: per capsule speed (0.1 m/s key) outcome of the climb / vault cases: {planned, Ok, Rejected(rate), Rejected(step),
    // Rejected(warp), other (not Ok / Rejected)}
    std::map<int, std::array<int, 6>> bySpeed;
};

static void RunCase(const DecisionTable& aT, const TraversalFile& aF, const Case& aC, const char* aRig, bool aExpectStart,
                    Totals& aTot, int aVaultCls = 3, const Config* aCfgOverride = nullptr)
{
    Plan p;
    Metrics m60, m30;
    ++aTot.cases;
    const bool ok = Evaluate(aT, aF, aC, aCfgOverride ? *aCfgOverride : g_baseCfg, p, m60, m30);
    if (g_diag && aExpectStart)
        std::printf("DIAG %s %s -> %s %s(%s) cls %d rateWanted %.2f rate %.2f step %.3f warp %.3f limit %.3f start %d end %d\n", aRig,
                    aC.tag.c_str(), p.montage ? p.montage->name.c_str() : "-", ToString(p.result), p.rejectReason, static_cast<int>(p.climbClass),
                    p.rateWanted, p.rate, p.gatePlanarStep, p.gateWarpAdded, p.gateStepLimit, p.startFrame, p.endFrame);
    const std::string tag = std::string(aRig) + " " + aC.tag;
    if (!aExpectStart)
    {
        Check(!ok, tag + ": expected no traversal, got " + (p.montage ? p.montage->name : std::string("?")));
        return;
    }
    const bool climb = aC.o.action == GameAction::Climb;
    const Expect ex = GameExpect(aC.o, aC.c);
    const int cls = climb ? ex.climbClass : aVaultCls;
    const double lim = ExpectedStepLimit(aC.c.speed, climb);
    ++aTot.total[cls];
    {
        auto& row = aTot.bySpeed[static_cast<int>(std::lround(aC.c.speed * 10.0))];
        ++row[0];
        const std::string why = p.rejectReason ? p.rejectReason : "";
        if (p.result == StartResult::Ok)
            ++row[1];
        else if (p.result == StartResult::Rejected)
            ++row[why == "rate" ? 2 : why == "step" ? 3 : why == "warp" ? 4 : 5];
        else
            ++row[5];
        if (p.result == StartResult::Rejected)
            Check(!why.empty(), tag + ": Rejected plan without a reason");
        else
            Check(why.empty(), tag + ": non-rejected plan carries a reject reason");
    }
    if (p.result == StartResult::Rejected) // gate: game animation kept (counted, coverage checked in main)
    {
        ++aTot.rejected;
        return;
    }
    Check(ok, tag + ": no plan (" + ToString(p.result) + ", action " + p.decision.actionName + ")");
    if (!ok)
        return;
    ++aTot.started;
    ++aTot.accepted[cls];
    ++aTot.alternatives[std::string(p.alternative) + " " + (p.alternative[0] ? p.montage->name : std::string())];
    char b[512];
    std::snprintf(b, sizeof b,
                  "%s -> %s start %d rate %.2f: contact %.4f, planar tick60 %.4f tick30 %.4f clip %.4f, warp-added %.4f, "
                  "dur err %.4f (game %.2f s), dest err %.4f",
                  tag.c_str(), p.montage->name.c_str(), p.startFrame, p.rate, m60.contact, m60.planarTick, m30.planarTick,
                  m60.planarClip, m60.warpAdded, m60.durErr, p.gameDuration, m60.destErr);
    Check(m60.contact <= kContactMax, std::string(b) + " (hand contact > 5 cm)");
    Check(m60.planarTick <= lim && m30.planarTick <= lim && m60.planarClip <= lim,
          std::string(b) + " (horizontal root step > the speed-dependent limit)");
    Check(m60.warpAdded <= lim, std::string(b) + " (warp-added root step > the speed-dependent limit)");
    Check(m60.doneCount == 1 && m30.doneCount == 1 && m60.justTraversedOnDone && m30.justTraversedOnDone,
          std::string(b) + " (hand-back / justTraversed not exactly once)");
    Check(m60.planDestErr <= 1e-6, std::string(b) + " (plan gameDestination != game formula)");
    if (!aC.o.hasHands)
    {
        char l[96];
        std::snprintf(l, sizeof l, " lateral %.3f bound %.3f land %.3f", m60.lateral, m60.lateralBound, m60.landErr);
        Check(m60.lateral <= m60.lateralBound, std::string(b) + l + " (no hand data: lateral root slide off the centred line)");
        if (!climb)
            Check(m60.landErr <= kDestMax && m30.landErr <= kDestMax,
                  std::string(b) + l + " (no hand data: vault root does not land at the game Destination)");
        aTot.lateral = std::max(aTot.lateral, m60.lateral);
        aTot.landErr = std::max(aTot.landErr, std::max(m60.landErr, m30.landErr));
        ++aTot.noHands;
    }
    if (climb)
    {
        Check(p.montage->action == "Mantle" || p.montage->action == "ClimbStart" || p.montage->action == "Catch",
              std::string(b) + " (climb montage action " + p.montage->action + ")");
        Check(!p.rateClamped, std::string(b) + " (play rate clamped, duration cannot match)");
        Check(m60.durErr <= 1.0 / 60.0 + 1e-9 && m30.durErr <= 1.0 / 30.0 + 1e-9,
              std::string(b) + " (hand-back time != game climb time)");
        Check(m60.destErr <= kDestMax && m30.destErr <= kDestMax, std::string(b) + " (end != HorizontalDestination)");
        Check(m60.planDurErr <= 1e-9, std::string(b) + " (plan gameDuration != game climb time)");
        Check(static_cast<int>(p.climbClass) == ex.climbClass, std::string(b) + " (climb class)");
        aTot.durErr60 = std::max(aTot.durErr60, m60.durErr);
        aTot.destErr = std::max(aTot.destErr, std::max(m60.destErr, m30.destErr));
        aTot.minRate = std::min(aTot.minRate, p.rate);
        aTot.maxRate = std::max(aTot.maxRate, p.rate);
        aTot.clamped += p.rateClamped ? 1 : 0;
    }
    else
        Check(p.montage->action == "Hurdle" || p.montage->action == "Catch" ||
                  (aVaultCls == 4 && p.montage->action == "Vault"),
              std::string(b) + " (vault montage action " + p.montage->action + ")");
    aTot.contact = std::max(aTot.contact, m60.contact);
    aTot.planarTick = std::max({aTot.planarTick, m60.planarTick, m30.planarTick});
    aTot.planarClip = std::max(aTot.planarClip, m60.planarClip);
    aTot.warpAdded = std::max(aTot.warpAdded, m60.warpAdded);
}

static std::vector<Case> ClimbCases()
{
    std::vector<Case> v;
    const double hs[] = {0.6, 0.9, 1.2, 1.25, 1.45, 1.6, 1.65, 2.0, 2.4, 2.7};
    const double speeds[] = {0.0, 1.5, 4.0, 5.5, 7.0, 8.5, 10.0}; // TR-dll: running climbs up to 10 m/s (field: >= 3 m/s rejected 89%)
    const double dists[] = {0.45, 0.7, 1.0};
    const double yaws[] = {-0.5, 0.0, 0.5}; // inputAngleThreshold 30 deg = 0.52 rad
    for (double h : hs)
        for (double s : speeds)
            for (double d : dists)
                for (double y : yaws)
                    for (int hands = 0; hands < 2; ++hands)
                        v.push_back(MakeClimb(h, s, d, y, hands != 0, false));
    for (double h : {0.6, 0.9, 1.5, 1.9}) // airborne climb (Locomotion 5): catch montages
        for (double d : {0.45, 0.8})
            v.push_back(MakeClimb(h, 3.0, d, 0.0, true, true));
    return v;
}

static std::vector<Case> VaultCases()
{
    std::vector<Case> v;
    for (double h : {0.55, 0.8, 1.0, 1.2})
        for (double depth : {0.1, 0.3, 0.55})
            for (double land : {0.0, 0.2})
                for (double s : {3.5, 5.0, 7.0})
                {
                    // ObstacleLengthCheck: offset < 0.6 + (2.0 - 0.6) * min(1, v / 8.5) + 0.05
                    const double range = 0.6 + 1.4 * std::min(1.0, s / 8.5) + 0.05;
                    for (double d : {0.5, 0.5 * (0.5 + range)})
                        for (double y : {-0.3, 0.0, 0.3})
                            if (h - land >= 0.5)
                                for (bool hands : {true, false}) // false = what the game sends for a vault
                                    v.push_back(MakeVault(h, depth, land, s, d, y, hands));
                }
    return v;
}

// TR-dll: wide vault (obstacle depth 0.6-2.0 m, the game's maxExtent 2.1): the table says Mantle for depth >= 59 cm, the planner
// routes the game vault to the Vault_1_0 montages (landing window to the game Destination).
static std::vector<Case> WideVaultCases()
{
    std::vector<Case> v;
    for (double h : {0.6, 0.9, 1.1})
        for (double depth : {0.7, 1.2, 2.0})
            for (double s : {3.5, 5.0, 7.0})
                for (double d : {0.5, 1.0})
                    for (double y : {-0.3, 0.0, 0.3})
                        for (bool hands : {true, false})
                            v.push_back(MakeVault(h, depth, 0.0, s, d, y, hands));
    return v;
}

// TR-dll: low hurdle (10-50 cm behind the edge: no table row, evaluated as the Hurdle row) over a thin obstacle.
static std::vector<Case> LowHurdleCases()
{
    std::vector<Case> v;
    for (double h : {0.15, 0.25, 0.35, 0.45})
        for (double depth : {0.1, 0.3})
            for (double s : {3.5, 5.0, 7.0})
                for (double d : {0.5, 1.0})
                    for (double y : {-0.3, 0.0, 0.3})
                        for (bool hands : {true, false})
                            v.push_back(MakeVault(h, depth, 0.0, s, d, y, hands));
    return v;
}

// Vehicle batch (TRAV-1 B / C, diag_traversal TESTS 5 / 6): a vault over a vehicle and a vault whose obstacle reaches far past the
// reported top surface go to the game's vault animation (Unsupported, reason "vehicle" / "deep"); the same vault over static
// geometry, a fence and a climb onto a vehicle keep their motion plans. Mutations UTPP_TRAVV2_MUTATION=1 (no vehicle rule) and 2 (no
// depth rule) must fail the respective check (tests/motion_traversal.cmake utpp_motion_traversal_mut1 / mut2).
static void VehicleRules(const DecisionTable& aT, const TraversalFile& aF, const char* aRig)
{
    const std::string r(aRig);
    auto plan = [&](const Case& aC) { return PlanTraversal(aT, aF, aC.o, aC.c, GameTweaks{}, Config{}); };
    auto desc = [](const Plan& aP) {
        return std::string(ToString(aP.result)) + "(" + (aP.reason ? aP.reason : "") + ") " + (aP.montage ? aP.montage->name : "-");
    };
    // (5) depth 0.40, landing 0, 5 m/s, no hand data (what the game sends); height / capsule distance = the first of h 0.80 / 1.0 x
    // d 1.0 / 0.8 / 0.5 m whose static-geometry plan is accepted (the woman rig's gate rejects h 0.80), so the control is a real
    // motion Hurdle
    Case stat = MakeVault(0.80, 0.40, 0.0, 5.0, 1.0, 0.0, false);
    bool found = false;
    for (double h : {0.80, 1.0})
        for (double d : {1.0, 0.8, 0.5})
            if (!found)
            {
                stat = MakeVault(h, 0.40, 0.0, 5.0, d, 0.0, false);
                found = plan(stat).result == StartResult::Ok;
            }
    Case veh = stat;
    veh.o.kind = 1;
    const Plan pv = plan(veh), ps = plan(stat);
    std::printf("%s vehicle rule (%s): kind 1 -> %s, kind 0 -> %s\n", aRig, stat.tag.c_str(), desc(pv).c_str(), desc(ps).c_str());
    Check(pv.result == StartResult::Unsupported && std::string(pv.reason) == "vehicle",
          r + " (5) vault over a vehicle (kind 1) must be Unsupported(vehicle), got " + desc(pv));
    Check(ps.result == StartResult::Ok && ps.montage && ps.montage->action == "Hurdle" && std::string(ps.reason).empty(),
          r + " (5) control: the same vault over static geometry (kind 0) must stay an Ok Hurdle, got " + desc(ps));
    {
        Case other = stat; // another entity (kind 2) is not a vehicle
        other.o.kind = 2;
        const Plan po = plan(other);
        Check(po.result == ps.result && po.montage == ps.montage, r + " (5) kind 2 (other entity) must plan like kind 0, got " + desc(po));
        Config off;
        off.vehicleVaultToGame = false;
        const Plan pf = PlanTraversal(aT, aF, veh.o, veh.c, GameTweaks{}, off);
        Check(pf.result == ps.result && pf.montage == ps.montage, r + " (5) vehicleVaultToGame false: kind 1 must plan like kind 0, got " + desc(pf));
    }
    // climb (Mantle) onto a vehicle is unchanged (it looks right in game)
    {
        Case c0 = MakeClimb(1.45, 1.5, 0.7, 0.0, true, false);
        Case c1 = c0;
        c1.o.kind = 1;
        const Plan q0 = plan(c0), q1 = plan(c1);
        Check(q0.result == StartResult::Ok && q1.result == q0.result && q1.montage == q0.montage && q1.startFrame == q0.startFrame,
              r + " (5) climb onto a vehicle must plan exactly like onto static geometry, got " + desc(q1) + " vs " + desc(q0));
    }
    // (6) topExtent 0.40 (fits the Hurdle row) but obstacleEnd 1.60 m past topPoint along the facing: Unsupported(deep)
    {
        Case deep = stat;
        deep.o.obstacleEnd.y = deep.o.topPoint.y + 1.60;
        const Plan pd = plan(deep);
        std::printf("%s depth rule: obstacleEnd +1.60 -> %s\n", aRig, desc(pd).c_str());
        Check(pd.result == StartResult::Unsupported && std::string(pd.reason) == "deep",
              r + " (6) vault with obstacleEnd 1.60 m past topPoint must be Unsupported(deep), got " + desc(pd));
        Case edge = stat; // exactly at the threshold (0.90): not deep
        edge.o.obstacleEnd.y = edge.o.topPoint.y + 0.90;
        const Plan pe = plan(edge);
        Check(std::string(pe.reason) != "deep", r + " (6) obstacleEnd 0.90 m past topPoint is not 'deep', got " + desc(pe));
        // fence 0.30 / 0.30 stays an Ok Hurdle
        const Plan pfence = plan(MakeVault(1.0, 0.30, 0.0, 5.0, 0.8, 0.0, false));
        Check(pfence.result == StartResult::Ok && std::string(pfence.reason).empty(),
              r + " (6) fence (topExtent 0.30, obstacleEnd 0.30) must stay Ok, got " + desc(pfence));
    }
}

static std::string Describe(const Plan& aP)
{
    return std::string(ToString(aP.result)) + "(" + (aP.reason ? aP.reason : "") + ") " + (aP.montage ? aP.montage->name : "-") + " dec " +
           aP.decision.actionName;
}

// TR-dll vault routing: each new key off reproduces the old routing of the same case (the controls), on = the new route.
static void VaultRouting(const DecisionTable& aT, const TraversalFile& aF, const char* aRig)
{
    const std::string r(aRig);
    const Case wide = MakeVault(0.9, 1.2, 0.0, 5.0, 1.0, 0.0, false); // wide obstacle, 1.2 m deep, 0.9 m high
    const Case low = MakeVault(0.3, 0.2, 0.0, 5.0, 0.8, 0.0, false);  // 30 cm curb
    const Case flat = MakeVault(0.05, 0.3, 0.0, 5.0, 0.8, 0.0, false); // 5 cm: below lowHurdleMinM
    auto plan = [&](const Case& aC, const Config& aCfg) { return PlanTraversal(aT, aF, aC.o, aC.c, GameTweaks{}, aCfg); };
    Config off;
    off.wideVault = false;
    off.lowHurdle = false;
    Config wideOn;
    wideOn.wideVault = true;
    const Plan wOn = plan(wide, wideOn), wOff = plan(wide, Config{}); // wideVault is off by default
    std::printf("%s routing: wide on -> %s | off -> %s\n", aRig, Describe(wOn).c_str(), Describe(wOff).c_str());
    Check(wOn.montage && wOn.montage->action == "Vault" && wOn.decision.actionName == "Vault" &&
              (wOn.result == StartResult::Ok || wOn.result == StartResult::Rejected),
          r + " wide vault (depth 1.2 m) must route to a Vault montage, got " + Describe(wOn));
    Check(wOff.result == StartResult::Unsupported && wOff.decision.actionName == "Mantle" && !wOff.montage,
          r + " wideVault off (default): a wide vault must be Unsupported (table Mantle) as before, got " + Describe(wOff));
    Config offWide;
    offWide.wideVault = false;
    Config offLow;
    offLow.lowHurdle = false;
    const Plan lOn = plan(low, Config{}), lOff = plan(low, offLow), lWideOff = plan(low, offWide);
    std::printf("%s routing: low on -> %s | off -> %s\n", aRig, Describe(lOn).c_str(), Describe(lOff).c_str());
    Check(lOn.montage && lOn.montage->action == "Hurdle" && lOn.decision.actionName == "Hurdle",
          r + " low hurdle (30 cm curb) must route to a Hurdle montage, got " + Describe(lOn));
    Check(lOff.result == StartResult::CheckFailed && !lOff.montage,
          r + " lowHurdle off: a 30 cm drop must be CheckFailed (no table row) as before, got " + Describe(lOff));
    Check(lWideOff.montage == lOn.montage && lWideOff.result == lOn.result,
          r + " wideVault off must not change a low hurdle, got " + Describe(lWideOff));
    const Plan fl = plan(flat, Config{});
    Check(!fl.montage && fl.result == StartResult::Unsupported, r + " a 5 cm vault stays Unsupported (curb, game), got " + Describe(fl));
    // wide vault over a vehicle stays with the game (vehicle rule first); a hurdle keeps the 'deep' rule
    Case wideVeh = wide;
    wideVeh.o.kind = 1;
    const Plan wv = plan(wideVeh, wideOn);
    Check(wv.result == StartResult::Unsupported && std::string(wv.reason) == "vehicle",
          r + " wide vault over a vehicle must stay Unsupported(vehicle), got " + Describe(wv));
    // the Vault montage ends at its touchdown on the game's Destination, not at the last window (mid-air)
    if (wOn.montage && wOn.result == StartResult::Ok)
    {
        const GameObstacle& o = wide.o;
        const V3 dest{o.obstacleEnd.x + o.inputDirection.x * 0.5, o.obstacleEnd.y + o.inputDirection.y * 0.5,
                      std::max(o.behindPoint.z, wide.c.pos.z)};
        Metrics m;
        Run(wOn, GameExpect(o, wide.c), 1.0 / 60.0, m);
        Check(m.landErr <= kDestMax && wOn.endFrame > wOn.windows.front().endFrame + 3,
              r + " wide vault: the hand-back is at the touchdown on the game Destination (land err " + std::to_string(m.landErr) + ", end frame " +
                  std::to_string(wOn.endFrame) + ")");
        (void)dest;
    }
}

// TR-dll identity: with every new key off (old minRate, fixed step limit, no wide vault / low hurdle) the original grid (speeds 0 / 1.5 / 4
// m/s, original vault cases) must still give exactly the pre-change accepted counts (216 / 126 / 108 / 588 woman, 630 man): the changes are
// all behind keys and do not touch the old routing.
// Start window and lead foot (Config::clampStartWindow / startWindowFallback / leadFootRow, all off by default): over the whole grid
// (climbs, vaults, low hurdles, wide vaults with their key on)
//  - with the window key on, every started plan's start time lies inside the montage's sampling range (start frame / fps within
//    [min, max] + 1e-6 s, or, for a range holding no frame, at most half a frame from its centre) unless the fallback key marked it;
//  - accepted plans per class are not fewer than with all keys off for the variants that keep the fallback or only rank by foot;
//  - with a known lead foot, a start whose planted foot ranks worse (other foot, or neither / both) is allowed only when no other montage
//    of the same decision gives a better rank under the same start rule (checked by planning each of them with forceMontage).
static bool StartInWindow(const Plan& aP)
{
    const StartWindow* w = FindStartWindow(aP.montage->name);
    if (!w)
        return false;
    const double t = static_cast<double>(aP.startFrame) / aP.montage->fps;
    if (t >= w->minS - 1e-6 && t <= w->maxS + 1e-6)
        return true;
    const bool frameInside = std::ceil(w->minS * aP.montage->fps - 1e-6) <= std::floor(w->maxS * aP.montage->fps + 1e-6);
    return !frameInside && std::fabs(t - 0.5 * (w->minS + w->maxS)) <= 0.5 / aP.montage->fps + 1e-9;
}

static void StartWindowAndFoot(const DecisionTable& aT, const TraversalFile& aF, const char* aRig)
{
    struct Item
    {
        Case c;
        int cls;
        bool wide;
    };
    std::vector<Item> items;
    for (const Case& c : ClimbCases())
        items.push_back({c, GameExpect(c.o, c.c).climbClass, false});
    for (const Case& c : VaultCases())
        items.push_back({c, 3, false});
    for (const Case& c : WideVaultCases())
        items.push_back({c, 4, true});
    for (const Case& c : LowHurdleCases())
        items.push_back({c, 5, false});
    const std::string r(aRig);
    // variants: 0 keys off, 1 window only (no fallback), 2 window with fallback, 3 / 4 window with fallback + lead foot left / right,
    // 5 / 6 window only + lead foot left / right, 7 / 8 lead foot left / right without the window
    constexpr int kV = 9;
    auto windowOn = [](int v) { return v >= 1 && v <= 6; };
    auto fallbackOn = [](int v) { return v == 2 || v == 3 || v == 4; };
    auto footOf = [](int v) { return v < 3 ? 0 : ((v - 3) % 2 == 0 ? 1 : 2); };
    // the accepted set may not shrink: fallback variants and the foot-only variants (the window-only ones print their loss)
    auto accCheck = [&](int v) { return fallbackOn(v) || v == 7 || v == 8; };
    int acc[kV][6] = {}, inWin[kV] = {}, started[kV] = {}, rank[kV][3] = {}, outside[kV] = {};
    int rowRank[kV][2][3] = {}; // row-order starts (foot key off) ranked against a left / right lead foot, for the report
    int unjustified = 0;
    std::map<std::string, int> rejects[kV];
    for (const Item& it : items)
        for (int v = 0; v < kV; ++v)
        {
            Config cfg = g_baseCfg;
            cfg.wideVault = it.wide;
            cfg.clampStartWindow = windowOn(v);
            cfg.startWindowFallback = fallbackOn(v);
            cfg.leadFootRow = footOf(v) != 0;
            CapsuleState cs = it.c.c;
            cs.leadFoot = static_cast<uint8_t>(footOf(v));
            const Plan p = PlanTraversal(aT, aF, it.c.o, cs, GameTweaks{}, cfg);
            if (p.result != StartResult::Ok || !p.montage)
            {
                if (p.result == StartResult::Rejected)
                    ++rejects[v][std::string(p.rejectReason) + " " + (p.montage ? p.montage->name : std::string("-"))];
                continue;
            }
            ++acc[v][it.cls];
            ++started[v];
            const bool in = StartInWindow(p);
            inWin[v] += in ? 1 : 0;
            outside[v] += p.startOutsideWindow ? 1 : 0;
            Check(!p.startOutsideWindow || fallbackOn(v), r + " " + it.c.tag + ": fallback flag without the fallback key");
            if (windowOn(v) && !p.startOutsideWindow)
                Check(in, r + " " + it.c.tag + ": start frame " + std::to_string(p.startFrame) + " of " + p.montage->name +
                              " outside the sampling range");
            if (footOf(v) == 0)
            {
                for (uint8_t lf = 1; lf <= 2; ++lf)
                    ++rowRank[v][lf - 1][StartFootRank(*p.montage, p.startFrame, lf)];
                continue;
            }
            const int rk = StartFootRank(*p.montage, p.startFrame, cs.leadFoot);
            ++rank[v][rk];
            if (rk == 0)
                continue;
            // a worse-ranked start is allowed only when no montage of the decision gives a better rank under the same start rule
            bool justified = true;
            for (const auto& name : p.decision.montages)
            {
                if (name == p.montage->name || !aF.Find(name))
                    continue;
                Config f = cfg;
                f.leadFootRow = false;
                f.forceMontage = name;
                // the same start rule as the chosen plan: in-window plans are compared with in-window plans (the window outranks the foot)
                f.startWindowFallback = false;
                f.clampStartWindow = cfg.clampStartWindow && !p.startOutsideWindow;
                const Plan q = PlanTraversal(aT, aF, it.c.o, cs, GameTweaks{}, f);
                if (q.result == StartResult::Ok && q.montage && StartFootRank(*q.montage, q.startFrame, cs.leadFoot) < rk)
                    justified = false;
            }
            unjustified += justified ? 0 : 1;
            Check(justified, r + " " + it.c.tag + ": " + p.montage->name + " (foot rank " + std::to_string(rk) +
                                 ") chosen while a montage with a better start foot passes");
        }
    const char* vn[kV] = {"keys off",
                          "window only",
                          "window + fallback",
                          "window + fallback + left foot",
                          "window + fallback + right foot",
                          "window only + left foot",
                          "window only + right foot",
                          "left foot",
                          "right foot"};
    for (int v = 0; v < kV; ++v)
    {
        std::printf("%s start window [%s]: %d started, %d in range (%.1f %%), accepted High %d Mid %d Low %d vault %d wide %d low hurdle %d",
                    aRig, vn[v], started[v], inWin[v], 100.0 * inWin[v] / std::max(started[v], 1), acc[v][0], acc[v][1], acc[v][2],
                    acc[v][3], acc[v][4], acc[v][5]);
        if (fallbackOn(v))
            std::printf(", fallback (start past the window) %d", outside[v]);
        if (footOf(v) != 0)
            std::printf(", start foot: lead %d, unknown %d, other %d", rank[v][0], rank[v][1], rank[v][2]);
        else
            std::printf(", start foot vs a left / right lead: lead %d / %d, unknown %d / %d, other %d / %d", rowRank[v][0][0],
                        rowRank[v][1][0], rowRank[v][0][1], rowRank[v][1][1], rowRank[v][0][2], rowRank[v][1][2]);
        std::printf("\n");
        for (int k = 0; k < 6 && accCheck(v); ++k)
            Check(acc[v][k] >= acc[0][k], r + " " + vn[v] + " class " + std::to_string(k) + ": " + std::to_string(acc[v][k]) +
                                               " accepted < " + std::to_string(acc[0][k]) + " with the keys off");
        if (g_diag)
            for (const auto& [k, n] : rejects[v])
                std::printf("%s start window [%s]: rejected %d x %s\n", aRig, vn[v], n, k.c_str());
        if (footOf(v) != 0)
            Check(rank[v][0] > 0, r + " " + vn[v] + ": no start with the lead foot planted");
    }
    Check(unjustified == 0, r + ": lead foot ignored while a montage with a better start foot passes");
}

// Traversal starts recorded on the test course that the gate rejected (handed back to the game animation). The obstacle geometry
// is rebuilt from the course boxes and the recorded capsule position / facing / height / depth / speed:
//  - three Climbs onto a wall 2.5 m above a 1.0 m platform, started 0.1 s after a jump (capsule 0.55-0.68 m above the platform,
//    rising about 4 m/s): the airborne catch row is rejected (warp), a jump start must play the ground climb of the speed band;
//  - one running Vault over a 0.5 m deep box 1.2 m tall with a 1.0 m drop behind: both montages of the depth row are rejected
//    (warp), the other depth row's run Hurdle passes;
//  - two accepted vaults of the same run (controls: the new keys must not change them).
struct Station
{
    const char* tag;
    bool climb;
    V3 pos;
    double yawDeg, speed, h, faceY, faceDist, ext, depth, back;
    const char* want; // expected montage name prefix with the new keys
    const char* alt;  // expected Plan::alternative
};

static Case MakeStation(const Station& aS, bool aHands)
{
    Case k;
    const double yaw = aS.yawDeg * 3.14159265358979323846 / 180.0;
    const V3 fwd{-std::sin(yaw), std::cos(yaw), 0}, face{0, aS.faceY, 0};
    k.o.action = aS.climb ? GameAction::Climb : GameAction::Vault;
    k.o.topPoint = aS.pos + fwd * (aS.faceDist / (fwd.y * face.y));
    k.o.topPoint.z = aS.pos.z + aS.h;
    k.o.collisionNormal = face * -1.0;
    k.o.topExtent = aS.ext;
    k.o.obstacleEnd = k.o.topPoint + face * aS.depth;
    k.o.behindPoint = k.o.obstacleEnd + face * 0.1;
    k.o.behindPoint.z = k.o.topPoint.z - aS.back;
    k.o.inputDirection = fwd;
    k.o.hasHands = aHands;
    const V3 lat{face.y, -face.x, 0};
    k.o.leftHand = k.o.topPoint - lat * 0.22;
    k.o.rightHand = k.o.topPoint + lat * 0.22;
    k.c.pos = aS.pos;
    k.c.yaw = yaw;
    k.c.speed = aS.speed;
    if (aS.climb)
    {
        k.c.airborne = true;
        k.c.verticalSpeed = 4.0;
        k.c.haveTakeoff = true;
        k.c.takeoffZ = 378.1;
    }
    k.tag = std::string(aS.tag) + (aHands ? " hands" : "");
    return k;
}

static void RecordedStations(const DecisionTable& aT, const TraversalFile& aF, const char* aRig)
{
    static const Station kStations[] = {
        {"climb 2.5 m, jump start standing", true, {1581.339, 2208.627, 378.645}, 4.5, 0.6, 1.95, 1, 0.473, 3.40, 3.39, 1.95,
         "AM_M_Neutral_Traversal_Climb_Start_2_5_stand_F", "jumpStart"},
        {"climb 2.5 m, jump start running", true, {1581.055, 2208.142, 378.783}, -4.5, 4.5, 1.82, 1, 0.958, 3.40, 3.39, 1.82,
         "AM_M_Neutral_Traversal_Climb_Start_2_5_run_F", "jumpStart"},
        {"climb 2.5 m, jump start walking", true, {1580.864, 2208.552, 378.647}, 4.5, 2.4, 1.95, 1, 0.548, 3.40, 3.39, 1.95,
         "AM_M_Neutral_Traversal_Climb_Start_2_5_walk_F", "jumpStart"},
        {"running vault 1.2 m", false, {1581.486, 2201.097, 377.100}, 153.3, 4.8, 1.20, -1, 0.497, 0.57, 0.51, 1.00,
         "AM_M_Neutral_Traversal_Hurdle_1_0_run_F_Lfoot", "depthRow"},
        {"running vault 1.0 m thin (control)", false, {1580.472, 2187.168, 377.100}, 2.3, 3.9, 1.00, 1, 0.832, 0.21, 0.20, 1.00,
         "AM_M_Neutral_Traversal_Hurdle_1_0_run_F_Lfoot", ""},
        {"running vault 1.0 m deep (control)", false, {1580.397, 2191.572, 377.100}, -3.2, 3.8, 1.00, 1, 0.528, 0.50, 0.50, 1.00,
         "AM_M_Neutral_Traversal_Hurdle_1_0_run_F_V2_Lfoot", ""},
    };
    const std::string r(aRig);
    Config off;
    off.jumpStartAsGround = false;
    off.rejectedFallback = false;
    for (const Station& st : kStations)
        for (int hands = 0; hands < (st.climb ? 2 : 1); ++hands)
        {
            const Case k = MakeStation(st, hands != 0);
            const std::string tag = r + " " + k.tag;
            const Plan before = PlanTraversal(aT, aF, k.o, k.c, GameTweaks{}, off);
            const bool control = !st.alt[0];
            Check(before.result == (control ? StartResult::Ok : StartResult::Rejected),
                  tag + ": keys off -> " + ToString(before.result) + " (the recorded result)");
            Plan p;
            Metrics m60, m30;
            const bool ok = Evaluate(aT, aF, k, Config{}, p, m60, m30);
            std::printf("%s station [%s]: keys off %s(%s) %s -> %s(%s) %s alt %s, rate %.2f, warp-added %.3f / limit %.2f, "
                        "contact %.4f\n",
                        aRig, k.tag.c_str(), ToString(before.result), before.rejectReason,
                        before.montage ? before.montage->name.c_str() : "-", ToString(p.result), p.rejectReason,
                        p.montage ? p.montage->name.c_str() : "-", p.alternative, p.rate, p.gateWarpAdded, p.gateStepLimit,
                        m60.contact);
            Check(ok && p.montage, tag + ": not accepted");
            if (!ok || !p.montage)
                continue;
            Check(p.montage->name.rfind(st.want, 0) == 0, tag + ": montage " + p.montage->name + ", expected " + st.want);
            Check(std::string(p.alternative) == st.alt, tag + ": alternative '" + p.alternative + "', expected '" + st.alt + "'");
            if (control)
                Check(p.montage == before.montage && p.startFrame == before.startFrame && p.endFrame == before.endFrame,
                      tag + ": the new keys changed an accepted plan");
            const double lim = ExpectedStepLimit(k.c.speed, st.climb);
            Check(m60.contact <= kContactMax, tag + ": hand contact " + std::to_string(m60.contact));
            Check(m60.planarTick <= lim && m30.planarTick <= lim && m60.planarClip <= lim && m60.warpAdded <= lim,
                  tag + ": root step above the limit");
            Check(m60.doneCount == 1 && m60.justTraversedOnDone, tag + ": hand-back not exactly once");
            if (!control)
            {
                // the same start onto a vehicle (kind 1): neither alternative route, the recorded rejection / refusal stays
                Case veh = k;
                veh.o.kind = 1;
                const Plan v = PlanTraversal(aT, aF, veh.o, veh.c, GameTweaks{}, Config{});
                const Plan vOff = PlanTraversal(aT, aF, veh.o, veh.c, GameTweaks{}, off);
                Check(v.result == vOff.result && v.montage == vOff.montage && !v.alternative[0] && v.result != StartResult::Ok,
                      tag + ": the new keys changed a vehicle plan");
            }
            if (st.climb)
            {
                Check(m60.durErr <= 1.0 / 60.0 + 1e-9, tag + ": hand-back time differs from the game climb time");
                Check(m60.destErr <= kDestMax, tag + ": does not end on the game destination");
                // a capsule that is not rising (falling / unknown) keeps the airborne catch row as the direct evaluation
                CapsuleState fall = k.c;
                fall.verticalSpeed = -3.0;
                Config direct = Config{};
                direct.rejectedFallback = false;
                const Plan q = PlanTraversal(aT, aF, k.o, fall, GameTweaks{}, direct);
                Check(q.montage && q.montage->name.find("_Catch_") != std::string::npos,
                      tag + ": falling start does not take the catch row");
                const Plan fb = PlanTraversal(aT, aF, k.o, fall, GameTweaks{}, Config{});
                Check(fb.result == StartResult::Ok && std::string(fb.alternative) == "ground",
                      tag + ": rejected catch does not fall back to the ground rows (" + ToString(fb.result) + ")");
                // no takeoff height (older feed): the height above the capsule picks the same ground row
                CapsuleState noTakeoff = k.c;
                noTakeoff.haveTakeoff = false;
                const Plan n = PlanTraversal(aT, aF, k.o, noTakeoff, GameTweaks{}, Config{});
                Check(n.result == StartResult::Ok && n.montage && n.montage->name.rfind(st.want, 0) == 0,
                      tag + ": jump start without a takeoff height");
            }
        }
}

static void LegacyIdentity(const DecisionTable& aT, const TraversalFile& aF, const char* aRig, int aRigIdx)
{
    Config legacy;
    legacy.minRate = 0.5;
    legacy.maxStepRunM = 0.0;
    legacy.wideVault = false;
    legacy.lowHurdle = false;
    legacy.scaleGateByRig = false;
    legacy.jumpStartAsGround = false;
    legacy.rejectedFallback = false;
    int ok[4] = {0, 0, 0, 0};
    std::vector<Case> all;
    for (const Case& c : ClimbCases())
        if (c.c.speed <= 4.0)
            all.push_back(c);
    for (const Case& c : VaultCases())
        all.push_back(c);
    for (const Case& c : all)
    {
        const Plan p = PlanTraversal(aT, aF, c.o, c.c, GameTweaks{}, legacy);
        if (p.result != StartResult::Ok)
            continue;
        const Expect e = GameExpect(c.o, c.c);
        ++ok[c.o.action == GameAction::Climb ? e.climbClass : 3];
    }
    std::printf("%s legacy config (old keys, original grid): %d / %d / %d / %d accepted\n", aRig, ok[0], ok[1], ok[2], ok[3]);
    {
        // information only: the same old keys over the extended grid (speeds to 10 m/s) = the pre-change acceptance to compare kMinAccepted with
        int ext[4] = {0, 0, 0, 0}, extN[4] = {0, 0, 0, 0};
        std::vector<Case> grid = ClimbCases();
        for (const Case& c : VaultCases())
            grid.push_back(c);
        for (const Case& c : grid)
        {
            const Expect e = GameExpect(c.o, c.c);
            const int k = c.o.action == GameAction::Climb ? e.climbClass : 3;
            ++extN[k];
            ext[k] += PlanTraversal(aT, aF, c.o, c.c, GameTweaks{}, legacy).result == StartResult::Ok ? 1 : 0;
        }
        std::printf("%s legacy config (extended grid): %d/%d, %d/%d, %d/%d, %d/%d accepted of High, Mid, Low, vault\n", aRig, ext[0], extN[0],
                    ext[1], extN[1], ext[2], extN[2], ext[3], extN[3]);
    }
    for (int k = 0; k < 4; ++k)
        Check(ok[k] >= kLegacyAccepted[aRigIdx][k] && ok[k] <= kLegacyAcceptedMax[aRigIdx][k],
              std::string(aRig) + " legacy config class " + std::to_string(k) + ": " + std::to_string(ok[k]) + " accepted, expected " +
                  std::to_string(kLegacyAccepted[aRigIdx][k]) + ".." + std::to_string(kLegacyAcceptedMax[aRigIdx][k]));
}

static void Mutations(const DecisionTable& aT, const TraversalFile& aF)
{
    // reference cases that pass with the default config
    const Case climb = MakeClimb(1.45, 1.5, 0.7, 0.3, true, false);
    const Case vault = MakeVault(1.0, 0.3, 0.0, 5.0, 0.8, 0.0);
    Plan p;
    Metrics a, b;
    // M1: warp off -> contact error must exceed 5 cm, destination missed
    {
        Config cfg;
        cfg.warp = false;
        Metrics m60, m30;
        const bool ok = Evaluate(aT, aF, climb, cfg, p, m60, m30);
        Check(ok && m60.contact > kContactMax, "mutation warp off not detected by contact check (climb)");
        Check(ok && m60.destErr > kDestMax, "mutation warp off not detected by destination check (climb)");
        Metrics v60, v30;
        const bool okv = Evaluate(aT, aF, vault, cfg, p, v60, v30);
        Check(okv && v60.contact > kContactMax, "mutation warp off not detected by contact check (vault)");
        std::printf("mutation warp off: climb contact %.4f dest %.4f, vault contact %.4f\n", m60.contact, m60.destErr,
                    v60.contact);
    }
    // M2: duration match off -> hand-back time differs from the game climb time
    {
        Config cfg;
        cfg.matchDuration = false;
        Metrics m60, m30;
        const bool ok = Evaluate(aT, aF, climb, cfg, p, m60, m30);
        Check(ok && m60.durErr > 1.0 / 60.0, "mutation duration match off not detected");
        std::printf("mutation duration off: dur err %.4f s\n", m60.durErr);
    }
    // M3: teleport injected into the per-tick root track -> planar tick check must fail
    {
        Evaluate(aT, aF, climb, Config{}, p, a, b);
        Player pl(p);
        double maxStep = 0;
        RootKey prev = p.warped.front();
        for (int i = 0; i < 2000; ++i)
        {
            Frame f = pl.Tick(1.0 / 60.0);
            if (i == 10)
                f.root.pos.y += 0.5; // teleport
            maxStep = std::max(maxStep, PlanarLen(f.root.pos - prev.pos));
            prev = f.root;
            if (f.done)
                break;
        }
        Check(maxStep > kStepMax, "mutation teleport not detected by planar step check");
        std::printf("mutation teleport: max planar tick step %.4f\n", maxStep);
    }
    // M4: warp jump injected into the warped clip track -> warp-added check must fail
    {
        Evaluate(aT, aF, vault, Config{}, p, a, b);
        std::vector<RootKey> w = p.warped;
        for (size_t f = static_cast<size_t>(p.startFrame) + 3; f < w.size(); ++f)
            w[f].pos.z += 0.5;
        Metrics m;
        ClipMetrics(p, w, m);
        Check(m.warpAdded > kStepMax, "mutation warp jump not detected by warp-added check");
        std::printf("mutation warp jump: warp-added %.4f\n", m.warpAdded);
    }
    // M5: wrong ledge (target 1 m higher than the plan used) -> contact check must fail
    {
        Evaluate(aT, aF, climb, Config{}, p, a, b);
        Plan q = p;
        q.windowTargets[0].z += 1.0;
        Check(WindowContactError(q, 0) > kContactMax, "mutation wrong ledge height not detected");
    }
    // M6 review major 3: pre-review hand-less targets (every hand on topPoint) -> lateral root slide beyond the source bound
    // (low climb without hand data, as the game sends it) and the vault root off the game Destination
    {
        Config cfg;
        cfg.centreRootWithoutHands = false;
        cfg.gate = false; // the slide must be caught by the checks, not hidden by a gate rejection
        int caught = 0, ran = 0;
        for (const Case& k : {MakeClimb(0.9, 1.5, 0.7, 0.0, false, false), MakeClimb(0.9, 0.0, 0.45, 0.3, false, false),
                              MakeVault(1.0, 0.3, 0.0, 5.0, 0.8, 0.0, false), MakeVault(0.8, 0.1, 0.0, 3.5, 0.5, -0.3, false)})
        {
            Metrics m60, m30;
            if (!Evaluate(aT, aF, k, cfg, p, m60, m30))
                continue;
            ++ran;
            const bool isVault = k.o.action == GameAction::Vault;
            const bool bad = m60.lateral > m60.lateralBound || (isVault && m60.landErr > kDestMax);
            caught += bad ? 1 : 0;
            std::printf("mutation hand-less targets: %s lateral %.3f (bound %.3f) land %.3f -> %s\n", k.tag.c_str(), m60.lateral,
                        m60.lateralBound, m60.landErr, bad ? "caught" : "MISSED");
        }
        Check(ran == 4 && caught == ran, "mutation hand-less hands-on-topPoint not detected by the lateral / landing checks");
    }
}

int main(int argc, char** argv)
{
    bool badArg = argc < 4;
    for (int i = 4; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--diag")
            g_diag = true;
        else
            badArg = true;
    }
    if (badArg)
    {
        std::printf("usage: utpp_motion_traversal_test <traversal_table.json> <woman.utpptrav> <man.utpptrav> [--diag]\n");
        return 2;
    }
    std::string json, err;
    DecisionTable table;
    Check(ReadAll(argv[1], json), std::string("read ") + argv[1]);
    const bool tok = table.Load(json, err);
    Check(tok, "table load: " + err);
    TraversalFile files[2];
    const char* rigs[2] = {"woman", "man"};
    for (int i = 0; i < 2; ++i)
        Check(LoadTraversalFile(argv[i + 2], files[i], err), std::string("load ") + argv[i + 2] + ": " + err);
    if (!tok || files[0].montages.empty() || files[1].montages.empty())
    {
        std::printf("utpp_motion_traversal_test: %d passed, %d failed (data missing)\n", g_pass, g_fail + 1);
        return 1;
    }
    const auto climbs = ClimbCases();
    const auto vaults = VaultCases();
    const auto wideVaults = WideVaultCases();
    const auto lowHurdles = LowHurdleCases();
    for (int i = 0; i < 2; ++i)
    {
        Totals tc, tv;
        for (const Case& c : climbs)
            RunCase(table, files[i], c, rigs[i], true, tc);
        for (const Case& c : vaults)
            RunCase(table, files[i], c, rigs[i], true, tv);
        Config wideCfg; // wide vault is off by default (Vault_1_0 = vault-and-fall clips): the route is exercised with the key on
        wideCfg.wideVault = true;
        for (const Case& c : wideVaults)
            RunCase(table, files[i], c, rigs[i], true, tv, 4, &wideCfg);
        for (const Case& c : lowHurdles)
            RunCase(table, files[i], c, rigs[i], true, tv, 5);
        // outside the ranges: climb above the motion Mantle table (2.75 m), vault with < 10 cm back ledge height (the table's Mantle
        // row, Unsupported for a vault: a curb the game handles)
        RunCase(table, files[i], MakeClimb(3.0, 0.0, 0.6, 0.0, true, false), rigs[i], false, tc);
        RunCase(table, files[i], MakeVault(0.05, 0.3, 0.0, 5.0, 0.8, 0.0), rigs[i], false, tv);
        VehicleRules(table, files[i], rigs[i]);
        VaultRouting(table, files[i], rigs[i]);
        LegacyIdentity(table, files[i], rigs[i], i);
        StartWindowAndFoot(table, files[i], rigs[i]);
        RecordedStations(table, files[i], rigs[i]);
        const char* cn[6] = {"High", "Mid", "Low", "vault", "wide vault", "low hurdle"};
        for (int k = 0; k < 2; ++k)
            for (const auto& [alt, n] : (k == 0 ? tc : tv).alternatives)
                std::printf("%s %s grid, plan source [%s]: %d\n", rigs[i], k == 0 ? "climb" : "vault",
                            alt == " " ? "direct" : alt.c_str(), n);
        for (int k = 0; k < 6; ++k)
        {
            const Totals& t = k < 3 ? tc : tv;
            std::printf("%s %s: accepted %d of %d (minimum %d)\n", rigs[i], cn[k], t.accepted[k], t.total[k],
                        kMinAccepted[i][k]);
            Check(t.accepted[k] > 0, std::string(rigs[i]) + " " + cn[k] + ": no accepted traversal (coverage)");
            Check(t.accepted[k] >= kMinAccepted[i][k],
                  std::string(rigs[i]) + " " + cn[k] + ": fewer accepted traversals than the recorded minimum");
        }
        std::printf("%s climb: %d cases (%d started), max contact %.4f m, max planar step %.4f m/tick, %.4f m/clip "
                    "frame, max warp-added %.4f m, max dur err %.4f s, max dest err %.4f m, rate %.2f..%.2f, clamped "
                    "%d\n",
                    rigs[i], tc.cases, tc.started, tc.contact, tc.planarTick, tc.planarClip, tc.warpAdded, tc.durErr60,
                    tc.destErr, tc.minRate, tc.maxRate, tc.clamped);
        std::printf("%s vault: %d cases (%d started), max contact %.4f m, max planar step %.4f m/tick, %.4f m/clip "
                    "frame, max warp-added %.4f m\n",
                    rigs[i], tv.cases, tv.started, tv.contact, tv.planarTick, tv.planarClip, tv.warpAdded);
        for (int k = 0; k < 2; ++k)
            for (const auto& [sp, r] : (k == 0 ? tc : tv).bySpeed)
                std::printf("%s %s v %4.1f: %d cases, Ok %d, Rejected rate %d step %d warp %d, other %d\n", rigs[i],
                            k == 0 ? "climb" : "vault", sp / 10.0, r[0], r[1], r[2], r[3], r[4], r[5]);
        std::printf("%s no hand data: climb %d started (max lateral %.3f m), vault %d started (max lateral %.3f m, max land err "
                    "%.4f m)\n",
                    rigs[i], tc.noHands, tc.lateral, tv.noHands, tv.lateral, tv.landErr);
    }
    Mutations(table, files[0]);
    // M6: gate off -> the criteria above must catch the plans the gate rejects (the gate is load-bearing)
    {
        Config cfg;
        cfg.gate = false;
        int bad = 0, n = 0;
        auto all = ClimbCases();
        const auto vs = VaultCases();
        all.insert(all.end(), vs.begin(), vs.end());
        for (const Case& c : all)
        {
            Plan p;
            Metrics m60, m30;
            if (!Evaluate(table, files[0], c, cfg, p, m60, m30))
                continue;
            ++n;
            const double lim = ExpectedStepLimit(c.c.speed, c.o.action == GameAction::Climb);
            if (m60.contact > kContactMax || m60.planarTick > lim || m30.planarTick > lim ||
                m60.planarClip > lim || m60.warpAdded > lim ||
                (c.o.action == GameAction::Climb && (m60.durErr > 1.0 / 60.0 + 1e-9 || p.rateClamped)))
                ++bad;
        }
        Check(bad > 0, "mutation gate off: no criterion violation found (gate not load-bearing or checks blind)");
        std::printf("mutation gate off: %d of %d plans violate a criterion\n", bad, n);
    }
    std::printf("utpp_motion_traversal_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

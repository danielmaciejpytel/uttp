#pragma once
// UTPP motion matching runtime - offline per-frame v2 pose pipeline (port).
// Header-only, allocates only in Init(), no game types.
// Order = motion ABP_SandboxCharacter / ABP_LayerBlending:
//   (1) ChooserMatcher MatchResult -> live outgoing-clip Hermite crossfade (retail), or explicit legacy Quintic
//       policy for diagnostic callers. External traversal/raw poses retain their inertialized bridge -> base pose L;
//   (2) OverlayMachine (pose overlay by weapon class, Relaxed / Ready / Aiming, stand / crouch, idle / move, style)
//       -> B = blended Pose_Neutral_*, OP = blended overlay poses of the machine keys (.utppov), curves = weighted
//       Layering_* curves, dP = blended style deltas dP_<Style>_<Stance> with the per-clip style weight
//       -> ov::ComposeLayers (mesh-space / local-space additives by Layering_*_Add / Arm_*_LS) -> layered pose;
//       overlayWeight blends L -> layered (0 = L bit for bit, layering skipped);
//   (3) gik::ApplyGripIK after the layering (Enable_HandIK_L / R curves of the layered result);
//   (4) idlebreak::Controller is evaluated as a CHOOSER-LAYER input: its Output (StartBreak / EndBreak / Interrupt,
//       clip, start time, blend time) is returned to the caller for the next matcher update; the pose is not touched.
// Every output is finite-checked (non-finite -> base pose, Evaluate returns false). Evaluate wall time is kept in a
// fixed ring (Budget::kWindow) for the p95 report against adapter::Budget::kFeedP95Us.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "AimSpineYawFilter.hpp"
#include "BarrelPitchLoop.hpp"
#include "MotionBlendStack.hpp"
#include "MotionAdapter.hpp"
#include "FeminineShoulders.hpp"
#include "ForearmTwist.hpp"
#include "MotionOrientationFeet.hpp"
#include "GripIK.hpp"
#include "IdleBreaks.hpp"
#include "MeshArmClavicleLimit.hpp"
#include "NativeShootTransfer.hpp"
#include "Overlay.hpp"
#include "OverlayFile.hpp"
#include "SupportGrip.hpp"

namespace utpp::mm::motionpose
{
inline bool MayUseAuthoredTwoHandGrip(bool aEnabled, bool aHandback, bool aHaveWeapon, bool aDrawn,
                                     ov::PoseOverlay aOverlay)
{
    // Pistol1H may still contain a fading Pistol2H layer; Evaluate checks its actual weight.
    return aEnabled && !aHandback && aHaveWeapon && aDrawn &&
           (aOverlay == ov::PoseOverlay::Rifle || aOverlay == ov::PoseOverlay::Pistol2H ||
            aOverlay == ov::PoseOverlay::Pistol1H);
}

enum class BaseBlendPolicy { Quintic, LiveCrossfade };

struct OverlayFrameInputs;
struct BaseFrameView
{
    std::span<const BoneTransform> pose;
    motionstack::Signals signals;
    float dt = 0;
    bool rootReliable = false;
    float staticTailWeight = 0;
};
struct OverlayFrameInputs
{
    // Synchronous, allocation-free hook after live base sampling, before
    // overlay/AO/grip. Context lives only through Evaluate; the driver commits
    // proposed root state on success.
    bool (*preOverlay)(void *, const BaseFrameView &, OverlayFrameInputs &) = nullptr;
    void *preOverlayContext = nullptr;
    bool montageBase = false; // DB traversal clips use the external/raw bridge, not the MM live stack
    ov::OverlayInputs machine;            // weapon class -> overlay (ov::SelectOverlay), aim / stance / movement, style
    bool bodyWoman = false; // explicit body identity; style policies do not infer sex from an overlay
    // Idle-with-pistol stance policy of the male V (Pistol1H Relaxed standing keeps the locomotion lower body and the lowered right arm; the
    // same ov::PreserveIdlePistolLowerBody / IdlePistolRightArmStyleWeight rules as the female V, style Masculine). Independent of bodyWoman,
    // which also drives the feminine shoulder narrowing and must stay female-only.
    bool maleIdlePistolStance = false;
    float overlayWeight = 1.0f;           // 0 = output is the inertialized base pose exactly
    const gik::Grip* leftGrip = nullptr;  // WeaponRight-space left grip (cache or table); nullptr = no left IK
    const ov::Xf* rightTarget = nullptr;  // model-space RightHand target; nullptr = no right IK
    bool gripSkip = false;                // DefaultSlot montage or previous frame in air
    bool authoredTwoHandGrip = false;      // Pistol2H/Rifle: derive the grip from the composed authored pose, before AO
    // rifle-grip-stock 2026-11-10: game grip / stock data of the equipped long gun (SupportGrip.hpp, sgrip::Of(recoil profile id));
    // the driver passes it on the Rifle overlay only. nullptr = no data: the Rifle pose is untouched bit for bit (after the gate fell).
    const sgrip::Entry* supportGrip = nullptr;
    bool shootPrepare = false;            // last authored Pistol2H Aim pose may seed an opt-in Shoot action
    bool shootAction = false;             // current action is Shoot; Reload clears the cached authored pose
    bool shootOneHand = false;           // explicit verified asset route only; never inferred from overlay
    bool shootDeltaValid = false;         // true only for fresh, same-rig native Shoot sample
    ov::Xf shootDelta{};                  // native WeaponRight motion in its pre-Shoot weapon-local frame
    float shootWeight = 1.0f;            // 100 ms hold, then 100 ms fade to current authored pose
    bool runIdleBreaks = false;           // evaluate the idle-break controller (chooser input) this frame
    idlebreak::Inputs idle;               // dt is overwritten with Evaluate's dt
    // motionfix item 2: aim offset after the layering (report 3.11 OPB sweep, 3.12 spine rotation + hand fix).
    // Angles of the camera / aim target relative to the entity (root) frame; haveAim false = no aim offset this frame.
    bool haveAim = false;
    bool haveActorAimPoint = false;
    math::Vec3 actorAimPoint{};          // raw point, retained separately from derived
                                         // visual-local yaw/pitch
    bool capturePoseDiagnostics = false; // mm.trace only: low-rate deltas for composed pose / feminine / AO / hand IK
    float aimYawDeg = 0.0f;   // camera yaw - root yaw, + = to the left (about +Z, the ApplySpineYaw sense)
    float aimPitchDeg = 0.0f; // + = up
    // fix batch 2 (AO): the game owns the upper body (handback) -> the aim offset is not applied to a target any more and its
    // latched angles fade out in kAoHandbackFadeSeconds (a cut would pop the MM upper body while the handback envelope still
    // publishes its share). A lost target without it (haveAim false) fades out with the Aiming weight (AimSpineYawFilter).
    bool aoSuppress = false;
    // fix batch 2 (AO): share of the aim offset that is still allowed while the reload hands the hands to the game (see
    // AoReloadScale). 1 outside a reload; the driver sets it while a reload envelope runs.
    float aoScale = 1.0f;
    // Aim pitch plan 2026-10-30 B (BarrelPitchLoop.hpp): closed loop on the barrel pitch of the aim offset. Off by default
    // (the open-loop aim offset bit for bit) until the driver supplies aimPointVisual; without a valid point it is inactive.
    bool barrelLoop = false;
    bool haveAimPointVisual = false;
    math::Vec3 aimPointVisual{};                  // hit point P of the camera ray, pose (visual root) frame: +Y forward, Z up
    int32_t barrelAxis = aim::kDefaultBarrelAxis; // WeaponRight local barrel axis (aim::AxisVector: 1 X, 2 Y, 3 Z, < 0 opposite)
    float barrelMuzzle = aim::Params{}.muzzleOffset; // m along the barrel from WeaponRight to the muzzle
};

// motionfix item 2 [P]: total spine yaw clamp. motion ModifyBone spine rotation clamps the AO yaw to +-90 deg;
// the in-game test asks for +-60..70 so a facing error cannot become a
// 90 deg neck twist: 60 deg [P], spread over Spine..Spine3 (OverlayFile spineYawBones).
inline constexpr float kAimSpineYawClampDeg = 60.0f;

// Fix batch 2 (AO) x reload contacts: the stationary pistol reload puts the hands and the weapon on the absolute NATIVE model
// positions (NativeReloadContactsRuntime FinishCore) while Spine..Spine3 stay MM. An aim offset that keeps tilting the MM torso
// (43-46 deg at pitch -60, head 1.46 m against the native 1.65 m) puts the weapon 0.2-0.3 m above the MM head. The aim offset
// therefore goes down with the MM share of the upper body: the same expression as the driver reloadScale (1 - shaped upper
// weight of the game, kept up by the slide roll hold). aShapedUpperWeight is g_reloadUpperWeight (AuthoredFadeWeight for a
// verified authored clip), aRollHold SlideRollHold::hold. Applied to the live AO (ADS reload aim hold) and to the latched one,
// so the AO is already 0 when the game owns the arms: no jump at the end of the aim hold, no overhead window after it.
inline float AoReloadScale(float aShapedUpperWeight, float aRollHold)
{
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 8
    (void)aShapedUpperWeight;
    (void)aRollHold;
    return 1.0f; // mutation 8: the aim offset ignores the reload hand-over
#else
    return std::max(1.0f - ov::Clamp01(aShapedUpperWeight), ov::Clamp01(aRollHold));
#endif
}

// motionfix item 6: both hands on the weapon (PoseSnapshot::overlayArms) = LeftHand within this distance of WeaponRight in
// the published pose. Measured on the .utppov poses (mesh space, woman = man): two-hand poses Pistol2H 0.049-0.130 m (Relax_Run
// 0.554 is a lowered run pose with Hand IK 0), Rifle 0.144-0.219 m; Pistol1H 0.620-1.067 m. Bound = midpoint of the gap between
// the largest two-hand value 0.219 and the smallest one-hand value 0.620.
inline constexpr float kOverlayArmsMaxHandDist = 0.42f;

// motionfix item 6 [P]: share of Layering_Arm_L_Add kept on the free left arm of the one-handed pistol poses (Hand IK L 0).
// MotionPoseTest 8 sweep (Pistol1H aiming on M_Neutral_Run_Loop_F, 3 s settled): left upper arm direction z max woman / man
// 1.00: +0.032 / +0.240 (horizontal / above), 0.50: -0.145 / +0.019, 0.25: -0.182 / -0.057, 0.00: -0.211 / -0.117.
inline constexpr float kFreeArmAddScale = 0.25f;

// Feminine shoulder posture (FeminineShoulders.hpp): per-side socket displacement toward the spine at weight 1, and the time a
// side needs to return from 0 to 1 after its reach limit released (drops follow the limit at once: the hand contact stays exact).
#ifndef UTPP_FEM_AMOUNT
#define UTPP_FEM_AMOUNT 0.025f // offline sweeps override with -DUTPP_FEM_AMOUNT
#endif
inline constexpr float kFeminineShoulderAmount = UTPP_FEM_AMOUNT; // m per side (2026-11-02: was 0.020)
inline constexpr float kFeminineMaxAmount = femshoulder::kMaxAmountCm / 100.0f; // clamp for any setting: 0.05 m (was 0.040)
inline constexpr float kFeminineRiseSeconds = 0.25f;
#ifndef UTPP_FEM_SOFTSTART
#define UTPP_FEM_SOFTSTART 0.995f // the recoil / grip IK needs reach slack: 0.998 failed 7 recoil-reach cases (2026-11-02)
#endif
#ifndef UTPP_FEM_MAXREACH
#define UTPP_FEM_MAXREACH 0.999f
#endif
inline constexpr float kFeminineSoftStart = UTPP_FEM_SOFTSTART, kFeminineMaxReach = UTPP_FEM_MAXREACH;
#ifndef UTPP_FEM_UNARMED_SOFTSTART
#define UTPP_FEM_UNARMED_SOFTSTART 0.998f
#endif
inline constexpr float kFeminineUnarmedSoftStart = UTPP_FEM_UNARMED_SOFTSTART, kFeminineUnarmedMaxReach = 0.9995f;

struct FrameInfo
{
    bool finite = true;
    bool inertialActive = false;
    bool liveCrossfade = false;
    float inertialDuration = 0.0f; // motionfix item 7: effective duration of the running / last transition (s)
    bool layered = false;
    gik::GripResult grip;
    gik::Grip gripTarget;
    bool gripFromOverlay = false;
    bool shootApplied = false;            // opt-in Shoot delta accepted by the selected 1H/2H IK route
    float shootScale = 0.0f;              // 0..1 reachable fraction of the requested weapon motion
    idlebreak::Output idle;
    ov::Curves curves;
    float evalUs = 0.0f;
    // motionfix item 2: aim offset of this frame (0 / false when not applied)
    bool aoApplied = false;
    float aoWeight = 0.0f;   // OverlayMachine::AimingWeight
    float aoSweepU = 0.0f;   // SweepTimeFromPitch
    float aoYawDeg = 0.0f;   // clamped yaw spread over the spine bones (before the weight)
    float aoAppliedWeight = 0.0f; // fix batch 2: weight actually applied = min(aoWeight, latch release); < aoWeight while a lost target fades
    bool aoLatched = false;       // fix batch 2: the angles of this frame are the latched ones of a lost target
    // motionfix item 6: distance LeftHand -> WeaponRight (m) of the published layered pose, -1 = not evaluated; steady =
    // one overlay pose key, one aim state and one base pose carry all the weight (no crossfade running)
    float leftHandToWeapon = -1.0f;
    bool overlaySteady = false;
    struct PoseStageDelta
    {
        float rotDeg[11] = {}; // local quaternion delta angle from the immediately preceding stage
        float posM[11] = {};   // local translation delta in metres from the immediately preceding stage
    };
    bool poseDiagValid = false;
    PoseStageDelta composeDelta, feminineDelta, overlayDelta, aimDelta, gripDelta;
    // Feminine shoulder pass of this frame (every frame the weapon / unarmed layering ran for a woman body; width = upper-arm
    // socket distance along the Spine3 lateral before / after the pass).
    struct FemShoulder
    {
        bool ran = false, applied = false;
        femshoulder::Reason reason = femshoulder::Reason::None;
        float requested = 0.0f, weight[2] = {}, feasible[2] = {};
        float widthBefore = 0.0f, widthAfter = 0.0f;
        gik::Result arm[2]; // last SolveArm result per side (left, right) of a failed pass, for diagnostics
        float handError = 0.0f, weaponError = 0.0f;
        float elbowMove[2] = {}, elbowFlexDeg[2] = {}; // elbow evidence of the pass (femshoulder::Result)
        float drift[2] = {}; // socket drift toward the hand per side (m, femshoulder REACH DRIFT; 0 up to the default amount)
    } fem;
    ov::ClavicleLimitResult clavicleLimit; // mesh-space arm clavicle deviation of this frame before the limit
    float forearmTwistMovedDeg[2] = {};   // forearm roll moved into the hand on the output (left, right; ForearmTwist.hpp)
    // rifle-hand-fix 2026-11-09: support (left) hand of a 2H pose - hand twist about the forearm -> hand axis before / after the
    // elbow swivel of LimitHandTwistBySwivel and the applied swivel (deg).
    float supportTwistBeforeDeg = 0.0f, supportTwistAfterDeg = 0.0f, supportSwivelDeg = 0.0f;
    // rifle-grip-stock 2026-11-10 (SupportGrip.hpp): weight of the game grip on the handguard, weight of the stock pass, stock gap along
    // the barrel before the pass (m, + = stock in front of the shoulder) and the applied weapon move (m, < 0 = back toward the shoulder)
    float supportGripWeight = 0.0f, stockWeight = 0.0f, stockGapM = 0.0f, stockShiftM = 0.0f;
    float reachShiftM = 0.0f, reachShiftNeedM = 0.0f; // rifle-grip-stock: applied / needed (uncapped) weapon slide for the left reach
    float supportReleaseM = 0.0f; // rifle-grip-stock review round 5: left-hand release offset still to go (m, sgrip::FollowLeftHand)
    float lowerLift[2] = {};              // OVLEGS: vertical ankle lift (m) of KeepLowerFootHeight on the composed legs (left, right)
    // Aim pitch plan 2026-10-30 section 4 (values for the MOTION trace): sweep pitch in / commanded, barrel loop state.
    float aoPitchInDeg = 0.0f;        // filtered target pitch of the aim offset
    float aoPitchCmdDeg = 0.0f;       // pitch given to the sweep = aoPitchInDeg + barrelBiasDeg (loop active)
    bool barrelActive = false;        // the loop ran this frame (enabled with an aim point, or still fading out)
    bool barrelMeasured = false;      // barrel and muzzle -> P measured after the aim offset
    float barrelBiasDeg = 0.0f;       // applied bias (biasEst)
    float barrelPitchDeg = 0.0f;      // barrel elevation after the aim offset (before the residual)
    float barrelWantPitchDeg = 0.0f;  // elevation of muzzle -> P
    float barrelErrBeforeDeg = 0.0f;  // barrel - want, before the residual (+ = above P)
    float barrelErrAfterDeg = 0.0f;   // barrel - want, after the residual
    float barrelResidualDeg = 0.0f;   // applied residual rotation (residual x aim offset alpha), + = up
    float barrelTargetDist = 0.0f;    // |P - muzzle| (m)
    int32_t barrelAxis = 0;           // axis id used (aim::AxisVector)
    float barrelLoopWeight = 0.0f;    // aim offset alpha the loop worked with
    uint32_t barrelExcluded = 0;      // BarrelExclusion bits (no learning, residual fading)
    // seam C (MotionDriver): the aim inputs the evaluator worked with after the pre-overlay hook (pose frame; diagnostics, tests)
    bool haveAimPointVisual = false;
    math::Vec3 aimPointVisual{};
    float aimYawInDeg = 0.0f, aimPitchInDeg = 0.0f; // OverlayFrameInputs aimYawDeg / aimPitchDeg (valid with an aim point)
};

inline constexpr std::array<std::string_view, 11> kPoseDiagnosticBoneNames = {
    "Spine1", "Spine3", "LeftShoulder", "RightShoulder", "LeftArm", "RightArm", "LeftForeArm", "RightForeArm", "LeftHand",
    "RightHand", "WeaponRight"};

class PoseEvaluator
{
public:
    // aDb / aOv must outlive the evaluator. aStyle may be null (style weight table defaults). aFirstBone: first bone the
    // inertializer touches (MotionAdapter::Init passes the same value).
    bool Init(const motion::MotionDatabase* aDb, const ov::OverlayFile* aOv, const ov::StyleFamilyData* aStyle,
              uint32_t aFirstBone, std::string& aErr, BaseBlendPolicy aPolicy = BaseBlendPolicy::Quintic)
    {
        m_ready = false;
        if (!aDb || !aDb->Loaded() || !aOv || !aOv->Ok())
            return Fail(aErr, "database / overlay file not loaded");
        if (aDb->Header().rigHash != aOv->RigHash())
            return Fail(aErr, "rig hash mismatch between .utppmm and .utppov");
        m_db = aDb;
        m_ov = aOv;
        m_n = aDb->NumBones();
        const ov::Skeleton& sk = aOv->Skel();
        m_poseDiagBones.fill(-1);
        for (size_t d = 0; d < kPoseDiagnosticBoneNames.size(); ++d)
            for (int b = 0; b < sk.n; ++b)
                if (aOv->BoneName(b) == kPoseDiagnosticBoneNames[d])
                {
                    m_poseDiagBones[d] = b;
                    break;
                }
        m_dbToOv.assign(m_n, -1);
        for (uint32_t i = 0; i < m_n; ++i)
            for (int k = 0; k < sk.n; ++k)
                if (aOv->BoneName(k) == aDb->BoneName(i))
                {
                    m_dbToOv[i] = k;
                    break;
                }
        for (uint32_t i = 0; i < m_n; ++i)
            if (m_dbToOv[i] < 0)
                return Fail(aErr, "database bone missing in the overlay skeleton");
        m_raw.assign(m_n, BoneTransform{});
        m_prev.assign(m_n, BoneTransform{});
        m_base.assign(m_n, BoneTransform{});
        m_basePolicy = aPolicy;
        if (m_basePolicy == BaseBlendPolicy::LiveCrossfade)
        {
            // motion FastFeet+Root WeightFactor profile resolved to the CP core rig.
            // CP Hips is UE pelvis, not its separate fast root track.
            std::vector<float> factors(m_n, 1.0f);
            for (uint32_t b = 0; b < m_n; ++b)
            {
                const auto name = aDb->BoneName(b);
                if (name == "Root" || name == "Trajectory") factors[b] = 0.1f;
                for (const auto leg : {"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",
                                       "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"})
                    if (name == leg) factors[b] = 1.0f / 1.5f;
            }
            if (!m_stack.Init(*aDb, factors)) return Fail(aErr, "live crossfade init");
        }
        if (!m_inertial.Init(m_n, aFirstBone))
            return Fail(aErr, "inertializer init");
        m_inertial.SetMinDuration(adapter::kBlendFloorSeconds); // motionfix item 7: blend floor on every v2 blend
        m_L = std::make_unique<ov::Pose>();
        m_B = std::make_unique<ov::Pose>();
        m_OP = std::make_unique<ov::Pose>();
        m_OPLower = std::make_unique<ov::Pose>();
        m_lay = std::make_unique<ov::Pose>();
        m_tmp = std::make_unique<ov::Pose>();
        m_shootFallback = std::make_unique<ov::Pose>();
        m_dP = std::make_unique<ov::Additive>();
        m_dPs = std::make_unique<ov::Additive[]>(kStyleSlots);
        m_opSlots = std::make_unique<ov::Pose[]>(7);
        m_lowSlots = std::make_unique<ov::Pose[]>(13);
        m_torsoPoses = std::make_unique<ov::Pose[]>(4);
        // pose index per machine key and style delta index per (style, stance key): resolved once (FindOverlayPose /
        // FindDelta build strings)
        for (int k = 0; k < kPoseKeys; ++k)
        {
            ov::PoseOverlay o;
            ov::AimState a;
            bool cr, mv;
            ov::DecodePoseKey(k, o, a, cr, mv);
            m_poseOfKey[k] = m_ov->FindOverlayPose(o, a, cr, mv);
        }
        static const char* kStyleName[3] = {"Neutral", "Feminine", "Masculine"}; // ov::Style order
        static const char* kStance[3] = {"_Stand_Idle", "_Stand_Move", "_Crouch"};
        for (int s = 0; s < 3; ++s)
            for (int k = 0; k < 3; ++k)
                m_styleDelta[s][k] = s == 0 ? -1 : m_ov->FindDelta(std::string("dP_") + kStyleName[s] + kStance[k]);
        m_scratch = std::make_unique<ov::LayerScratch>();
        // OVLEGS (KeepLowerFootHeight): the foot lift of the composed legs; not ready (no lift) when the rig lacks the leg chains
        m_lowerReady = m_lowerWarp.Init(*aDb);
        m_lowerPose.assign(m_n, BoneTransform{});
        for (int side = 0; side < 2; ++side)
            for (int k = 0; k < 3; ++k)
            {
                static constexpr const char* kLeg[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"}, {"RightUpLeg", "RightLeg", "RightFoot"}};
                m_legDb[side][k] = -1;
                for (uint32_t i = 0; i < m_n; ++i)
                    if (aDb->BoneName(i) == kLeg[side][k])
                        m_legDb[side][k] = static_cast<int>(i);
                m_lowerReady = m_lowerReady && m_legDb[side][k] >= 0;
            }
        // motionfix item 2: AO sweeps "AO_<Overlay>_<Stand|Crouch>_Sweep" (31-frame mesh-space deltas, reference 15)
        // and the hand-fix masks (clavicles + ik_hand = LeftShoulder / RightShoulder / WeaponLeft / WeaponRight, single
        // bones: LBB_LS of the clavicles from the pre-AO pose, report 3.12)
        for (int o = 0; o < kOverlays; ++o)
            for (int c = 0; c < 2; ++c)
            {
                m_sweep[o][c] = -1;
                const char* on = o == static_cast<int>(ov::PoseOverlay::Rifle)      ? "Rifle"
                                 : o == static_cast<int>(ov::PoseOverlay::Pistol1H) ? "Pistol1H"
                                 : o == static_cast<int>(ov::PoseOverlay::Pistol2H) ? "Pistol2H"
                                                                                      : nullptr;
                if (on)
                    m_sweep[o][c] = m_ov->FindDelta(std::string("AO_") + on + (c ? "_Crouch_Sweep" : "_Stand_Sweep"));
            }
        m_aoPre = std::make_unique<ov::Pose>();
        m_aoAdd = std::make_unique<ov::Additive>();
        m_aoA = std::make_unique<ov::Additive>();
        m_aoB = std::make_unique<ov::Additive>();
        m_aoSlots = std::make_unique<ov::Additive[]>(6);
        m_clavL.Clear(sk.n);
        m_clavR.Clear(sk.n);
        m_spine3 = m_spine2 = -1;
        for (int k = 0; k < sk.n; ++k)
        {
            const std::string_view nm = aOv->BoneName(k);
            if (nm == "LeftShoulder" || nm == "WeaponLeft")
                m_clavL.w[k] = 1.0f;
            if (nm == "Spine3") m_spine3 = k;
            if (nm == "Spine2") m_spine2 = k;
            if (nm == "RightShoulder" || nm == "WeaponRight")
                m_clavR.w[k] = 1.0f;
        }
        // base poses B by BaseKey (crouch * 2 + moving)
        static const char* kBase[4] = {"Pose_Neutral_Stand_Idle", "Pose_Neutral_Stand_Move", "Pose_Neutral_Crouch_Idle",
                                       "Pose_Neutral_Crouch_Move"};
        m_basePoses = std::make_unique<ov::Pose[]>(4);
        for (int k = 0; k < 4; ++k)
        {
            int p = aOv->FindPose(kBase[k]);
            if (p < 0 && k >= 2)
                p = aOv->FindPose("Pose_Neutral_Crouch");
            if (p < 0)
                return Fail(aErr, kBase[k]);
            aOv->FramePose(aOv->PoseAt(p).firstFrame, m_basePoses[k]);
        }
        m_rig = gik::FindRig(sk.n, [&](int i) { return aOv->BoneName(i); });
        m_forearmTwist.Build(sk, m_rig.left.lower, m_rig.left.hand, m_rig.right.lower, m_rig.right.hand);
        m_shootClavL = m_shootClavR = -1;
        m_pocketBone = -1;
        for (int i = 0; i < sk.n; ++i)
        {
            if (aOv->BoneName(i) == "LeftShoulder") m_shootClavL = i;
            if (aOv->BoneName(i) == "RightShoulder") m_shootClavR = i;
            if (aOv->BoneName(i) == sgrip::kPocketBone) m_pocketBone = i;
        }
        m_supportFingers = sgrip::FindFingers(sk.n, [&](int i) { return aOv->BoneName(i); });
        m_supportGate.Reset();
        m_reachShift = 0.0f;
        m_reachShiftHave = false;
        m_routeW = 0.0f;
        m_routeWHave = false;
        m_release.Reset();
        if (aStyle)
            m_styleTable = aStyle->weights;
        else
            m_styleTable.SetDefaults();
        m_machine = ov::OverlayMachine{};
        m_idle = idlebreak::Controller{};
        m_aoFilter.Reset();
        m_barrel.Reset();
        m_times.fill(0.0f);
        m_timeCount = 0;
        m_havePrev = m_externalBase = m_usesLiveBase = false;
        m_femHave = false;
        m_femHold.Reset();
        m_ready = true;
        return true;
    }

    bool Ready() const { return m_ready; }
    // Feminine shoulder posture (FeminineShoulders.hpp). continuous = per-side weights that follow the reach limits and a
    // rise limiter (default); false = the pre-2026-11-02 all-or-nothing pass (offline baseline only). amount = m per side.
    void SetFeminineShoulders(float aAmountM, bool aContinuous = true, float aRiseSeconds = kFeminineRiseSeconds)
    {
        m_femAmount = std::isfinite(aAmountM) ? std::clamp(aAmountM, 0.0f, kFeminineMaxAmount) : kFeminineShoulderAmount;
        m_femContinuous = aContinuous;
        m_femRiseSeconds = std::isfinite(aRiseSeconds) && aRiseSeconds > 0.0f ? aRiseSeconds : kFeminineRiseSeconds;
    }
    float FeminineShoulderAmount() const { return m_femAmount; }
    void SetGraphInputs(const motionstack::GraphInputs &input)
    {
        m_stack.SetGraphInputs(input);
    }
    // air: the next blend request (if any) skips the interruption deficit (landing: back-to-back air re-picks
    // must not shorten the touchdown blend to 0). Stays armed until a request consumes it or it is disarmed.
    void SetIgnoreDeficitOnNextBlend(bool aOn) { m_ignoreDeficit = aOn; }
    bool IgnoreDeficitArmed() const { return m_ignoreDeficit; }
    void Reset()
    {
        m_inertial.Reset();
        m_stack.Reset();
        m_externalBase = m_usesLiveBase = false;
        m_ignoreDeficit = false;
        m_machine = ov::OverlayMachine{};
        m_idle = idlebreak::Controller{};
        m_aoFilter.Reset();
        m_barrel.Reset();
        m_havePrev = false;
        m_femHave = false;
        m_femHold.Reset();
        m_supportGate.Reset();
        m_reachShift = 0.0f;
        m_reachShiftHave = false;
        m_routeW = 0.0f;
        m_routeWHave = false;
        m_release.Reset();
    }

    // One frame. aOut receives NumBones() transforms (game-local, db bone order). Returns false when no pose could be
    // sampled (aOut untouched) or a stage produced a non-finite value (aOut = inertialized base pose).
    bool Evaluate(float aDt, const motion::MatchResult& aMatch, const OverlayFrameInputs& aIn, BoneTransform* aOut,
                  uint32_t aNumBones, FrameInfo* aInfo = nullptr)
    {
        return EvaluateImpl(aDt, aMatch, nullptr, aIn, aOut, aNumBones, aInfo);
    }
    // UTPP-MOTION SEAM trav: same pipeline with the raw pose given by the caller (NumBones() transforms, db bone order,
    // game-local) instead of the database clip sample: traversal montages that are not clips of the database (.utpptrav
    // version 2 poses). aMatch still carries blend / jumped for the inertializer. nullptr -> false.
    bool EvaluateWithPose(float aDt, const motion::MatchResult& aMatch, const BoneTransform* aRaw, const OverlayFrameInputs& aIn,
                          BoneTransform* aOut, uint32_t aNumBones, FrameInfo* aInfo = nullptr)
    {
        return aRaw != nullptr && EvaluateImpl(aDt, aMatch, aRaw, aIn, aOut, aNumBones, aInfo);
    }

private:
    bool EvaluateImpl(float aDt, const motion::MatchResult& aMatch, const BoneTransform* aRaw, const OverlayFrameInputs&input,
                      BoneTransform* aOut, uint32_t aNumBones, FrameInfo* aInfo)
    {
        auto aIn = input; // same-frame target conversion never mutates the caller's
                          // actor-space inputs
        const auto t0 = std::chrono::steady_clock::now();
        FrameInfo info;
        if (!m_ready || !aOut || aNumBones != m_n)
            return false;
        const bool external = aRaw != nullptr || aIn.montageBase;
        const bool live = m_basePolicy == BaseBlendPolicy::LiveCrossfade && !external;
        if (live)
        {
            auto selected = aMatch;
            if (m_havePrev && m_externalBase)
            {
                // No future DB timeline exists for an external montage. Preserve
                // its last emitted base as a static tail on the MM handback.
                if (!m_stack.SeedPose(m_base)) return false;
                selected.jumped = true;
            }
            else if (!m_havePrev || (selected.jumped && !selected.blend.valid))
                m_stack.Reset();
            if (!m_stack.Step(selected, aDt)) return false;
            std::copy_n(m_stack.Output(), m_n, m_base.data());
            if (selected.blend.valid) m_ignoreDeficit = false;
            info.liveCrossfade = true;
            info.inertialDuration = m_stack.At(0).duration; // common transition diagnostic
        }
        else
        {
            if (aRaw != nullptr)
                std::memcpy(m_raw.data(), aRaw, sizeof(BoneTransform) * m_n);
            else if (!adapter::SamplePose(*m_db, aMatch.clip, aMatch.clipTime, m_raw.data(), m_n))
                return false;
            if (m_basePolicy == BaseBlendPolicy::LiveCrossfade && m_havePrev && !m_externalBase)
            {
                // Seed the unchanged traversal inertializer with the last actual
                // stack output, never with a stale incoming raw clip sample.
                m_inertial.Reset();
                m_inertial.Update(m_base.data(), nullptr, 0.0f, m_prev.data());
            }
            if (aMatch.blend.valid && aMatch.blend.blendTime > 0.0f)
            {
                m_inertial.Request(aMatch.blend.blendTime, m_ignoreDeficit);
                m_ignoreDeficit = false;
            }
            const bool continuous = m_havePrev && !aMatch.jumped &&
                (m_basePolicy == BaseBlendPolicy::Quintic || m_externalBase);
            if (!m_inertial.Update(m_raw.data(), continuous ? m_prev.data() : nullptr, aDt, m_base.data()))
                m_base = m_raw;
            std::swap(m_prev, m_raw);
            info.inertialActive = m_inertial.Active();
            info.inertialDuration = m_inertial.Duration();
        }
        m_externalBase = external;
        m_usesLiveBase = live;
        m_havePrev = true;
        if (live && aIn.preOverlay &&
            !aIn.preOverlay(
                aIn.preOverlayContext,
                BaseFrameView{m_base, m_stack.OutputSignals(), aDt, m_stack.RootReliable(), m_stack.StaticTailWeight()},
                aIn))
            return false;
        info.haveAimPointVisual = aIn.haveAimPointVisual;
        info.aimPointVisual = aIn.aimPointVisual;
        info.aimYawInDeg = aIn.aimYawDeg;
        info.aimPitchInDeg = aIn.aimPitchDeg;
        // (4) idle breaks: chooser-layer input only
        if (aIn.runIdleBreaks)
        {
            idlebreak::Inputs ii = aIn.idle;
            ii.dt = aDt;
            info.idle = m_idle.Update(ii);
        }
        // (2) overlay layering
        m_machine.Update(aIn.machine, aDt);
        const float w = ov::Clamp01(aIn.overlayWeight);
        bool ok = true;
        if (w > 0.0f)
        {
            ToOv(m_base.data(), *m_L);
            info.curves = BuildLayerInputs();
            const float idleLower = m_machine.LowerIdlePistolWeight(); // idle weight of the B2 keys
            auto scaleLower = [&](ov::Curves& aCurves) { // the legs / pelvis weights of the evaluator: info.curves and the stance sides below
                ov::PreserveIdlePistolLowerBody(m_machine, aIn.bodyWoman, aCurves, idleLower);
                ov::PreserveIdlePistolLowerBody(m_machine, aIn.maleIdlePistolStance, aCurves, idleLower, ov::Style::Masculine);
#if !(defined(UTPP_AIRLEGS_MUTATION) && UTPP_AIRLEGS_MUTATION == 3) // mutation 3: airW is not applied by the evaluator
                ov::FadeLowerBodyInAir(aIn.machine.airW, aCurves); // P6 K1: legs follow the jump clip, not the weapon pose
#endif
            };
            scaleLower(info.curves);
#if !(defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 4) // mutation 4: MotionPose does not apply jumpOutArmW
            ov::FadeArmsInJumpOutFlight(aIn.machine.jumpOutArmW, info.curves); // r9: the arms of a jump-out flight follow the jump clip, not the weapon pose
#endif
            ov::LayerInputs li;
            li.L = m_L.get();
            li.B = m_B.get();
            li.OP = m_haveOP ? m_OP.get() : nullptr;
            li.OPLower = m_haveOP ? m_OPLower.get() : nullptr; // P1 B2: legs / pelvis follow the stance alpha of B
            if (m_haveOP && m_torsoOn) // CR-4: spine / head per stance side during a stance fade of B
            {
                for (int k = 0; k < 2; ++k)
                {
                    li.torso[k].OP = &m_torsoPoses[k];
                    li.torso[k].B = &m_torsoPoses[2 + k];
                    li.torso[k].spine = m_torsoCurves[k][0];
                    li.torso[k].spineAdd = m_torsoCurves[k][1];
                    li.torso[k].head = m_torsoCurves[k][2];
                    li.torso[k].headAdd = m_torsoCurves[k][3];
                    ov::Curves lower; // OVLEGS: the legs / pelvis weights of the side, with the same idle / air factors as info.curves
                    lower.v[ov::kLegs] = m_torsoCurves[k][4];
                    lower.v[ov::kPelvis] = m_torsoCurves[k][5];
                    scaleLower(lower);
                    li.torso[k].legs = lower.v[ov::kLegs];
                    li.torso[k].pelvis = lower.v[ov::kPelvis];
                }
                li.torsoCrouch = m_torsoCrouch;
            }
            li.dP = m_haveDP ? m_dP.get() : nullptr;
            li.dPWeight = m_machine.StyleWeight(m_styleTable);
            li.curves = info.curves;
            li.styleArmR = ov::IdlePistolRightArmStyleWeight(m_machine, aIn.bodyWoman) *
                           ov::IdlePistolRightArmStyleWeight(m_machine, aIn.maleIdlePistolStance, ov::Style::Masculine); // x 1.0f: bit-identical for either body alone
            const ov::Skeleton& sk = m_ov->Skel();
            ov::ComposeLayers(sk, m_ov->Masks(), li, *m_scratch, *m_lay);
            KeepLowerFootHeight(li, info);
            // Mesh-space arms put the whole locomotion Spine3 deviation into the clavicle (Turn_180 aim-align, heavy
            // landing, slide: 59-84 deg -> shoulder skin balloon). MEASUREMENT ONLY: the runtime limit (180 deg) never engages
            // (MeshArmClavicleLimit.hpp STATUS); the deviation goes to FrameInfo::clavicleLimit and the trace.
            if (li.OP && li.B)
                info.clavicleLimit = ov::LimitMeshArmClavicles(sk, *m_scratch, m_shootClavL, m_shootClavR,
                                                               ov::kMeshArmClavicleRuntimeLimitDeg, ov::kMeshArmClavicleSoftDeg, *m_lay);
            if (w < 1.0f)
            {
                ov::Blend(*m_L, *m_lay, w, *m_tmp);
                std::memcpy(m_lay->b, m_tmp->b, sizeof(ov::Xf) * static_cast<size_t>(sk.n));
            }
            ov::Xf feminineBefore[kPoseDiagnosticBoneNames.size()];
            if (aIn.capturePoseDiagnostics)
            {
                CaptureStageDelta(*m_L, *m_lay, info.composeDelta);
                CaptureStageBones(*m_lay, feminineBefore);
            }
            // Contact-preserving feminine posture, before authored grip/AO/action bases.
            // Both hands and weapon attachments must survive the complete bilateral solve.
            if (aIn.bodyWoman && m_rig.Valid(sk))
            {
                // A target change does not instantly replace the composed pose.
                // Keep partial reach admission while an actual 1H or unarmed
                // pose contributes. Brief unarmed starts/stops can straighten
                // an arm past the full correction's reach limit; rejecting the
                // whole bilateral correction then changes socket width by 4 cm.
                bool composedPistol1H = false;
                bool composedUnarmed = false;
                float composedSum = 0.0f, composedOther = 0.0f; // weights of all composed poses / of the non-Default (weapon-class) ones
                for (int i = 0; i < m_machine.poses.count; ++i)
                {
                    if (m_machine.poses.w[i] > 0.0f)
                    {
                        composedSum += m_machine.poses.w[i];
                        if (m_machine.poses.key[i] / 12 != static_cast<int>(ov::PoseOverlay::Default))
                            composedOther += m_machine.poses.w[i];
                    }
                    composedPistol1H = composedPistol1H || (m_machine.poses.w[i] > 0.0f &&
                        m_machine.poses.key[i] / 12 == static_cast<int>(ov::PoseOverlay::Pistol1H));
                    composedUnarmed = composedUnarmed || (m_machine.poses.w[i] > 0.0f &&
                        m_machine.poses.key[i] / 12 == static_cast<int>(ov::PoseOverlay::Default));
                }
                femshoulder::Result posture;
                // Bounded posture pass keeps exact contacts near extension; global grip tuning stays unchanged.
                gik::Params contactParams;
                contactParams.softStart = kFeminineSoftStart;
                contactParams.maxReach = kFeminineMaxReach;
                // Purely unarmed layering has no recoil / grip IK behind it that needs reach slack: the arms may stay exact
                // closer to full extension there, which keeps the posture through arm swings (fewer reach cliffs). The window
                // fades with the weapon-class share of the composed poses (drawing a weapon must not switch it in one frame).
                {
                    const float weaponShare = composedSum > 0.0f ? ov::Clamp01(composedOther / composedSum) : 1.0f;
                    contactParams.softStart = kFeminineUnarmedSoftStart * (1.0f - weaponShare) + kFeminineSoftStart * weaponShare; // exact at both ends
                    contactParams.maxReach = kFeminineUnarmedMaxReach * (1.0f - weaponShare) + kFeminineMaxReach * weaponShare;
                }
                const float requested = m_machine.styles.WeightOf(static_cast<int>(ov::Style::Feminine)) * w;
                info.fem.ran = true;
                info.fem.widthBefore = femshoulder::SocketWidth(sk, *m_lay, m_rig, m_spine3, m_shootClavL, m_shootClavR);
                if (m_femContinuous)
                {
                    // Rise limiter: the previous applied weight of each side plus rate * dt. No history (first frame / Reset)
                    // = unlimited, so a fresh start shows the posture at once.
                    float limit[2] = {1.0f, 1.0f};
                    const float step = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.1f) / m_femRiseSeconds : 0.0f; // dt 0 re-evaluates the same frame
                    if (m_femHave)
                        for (int side = 0; side < 2; ++side) limit[side] = m_femPrev[side] + step;
                    femshoulder::Options options;
                    options.amount = m_femAmount;
                    options.maxWeight = limit;
                    options.hold = &m_femHold; // RETURN HOLD (amount > 2.5 cm per side only; dll-shoulder-brake 2026-11-09)
                    options.dt = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.1f) : 0.0f;
                    femshoulder::ApplyContinuous(sk, *m_lay, *m_tmp, m_rig, m_spine3, m_shootClavL, m_shootClavR,
                                                 requested, contactParams, posture, options);
                    // Remember what is shown. A pass that could not run keeps nothing: the next rise starts from 0.
                    for (int side = 0; side < 2; ++side) m_femPrev[side] = posture.applied ? posture.weight[side] : 0.0f;
                    if (!posture.applied) m_femHold.Reset();
                    m_femHave = true;
                }
                else
                    femshoulder::Apply(sk, *m_lay, *m_tmp, m_rig, m_spine3, m_shootClavL, m_shootClavR, requested,
                                       contactParams, posture, composedPistol1H || composedUnarmed);
                info.fem.applied = posture.applied;
                info.fem.reason = posture.reason;
                info.fem.requested = requested;
                info.fem.arm[0] = posture.left; info.fem.arm[1] = posture.right;
                info.fem.handError = posture.handError; info.fem.weaponError = posture.weaponError;
                for (int side = 0; side < 2; ++side)
                {
                    info.fem.elbowMove[side] = posture.elbowMove[side];
                    info.fem.elbowFlexDeg[side] = posture.elbowFlexDeg[side];
                    info.fem.drift[side] = posture.applied ? posture.drift[side] : 0.0f;
                    info.fem.weight[side] = posture.applied ? (m_femContinuous ? posture.weight[side] : posture.appliedWeight) : 0.0f;
                    info.fem.feasible[side] = posture.feasible[side];
                }
                info.fem.widthAfter = femshoulder::SocketWidth(sk, *m_lay, m_rig, m_spine3, m_shootClavL, m_shootClavR);
            }
            if (aIn.capturePoseDiagnostics)
                CaptureStageDelta(feminineBefore, *m_lay, info.feminineDelta);
            // Recoil is a weapon-local delta on THIS frame's authored arms.
            // Caching the pre-shot arms freezes Ready while the machine advances into Aim
            // and would also freeze locomotion once moving recoil is admitted.
            if (aIn.capturePoseDiagnostics)
            {
                CaptureStageDelta(*m_L, *m_lay, info.overlayDelta);
                info.poseDiagValid = true;
            }
            // A one-handed game's free left hand is not a two-handed pistol grip.
            // Capture the authored relative contact before AO, then let it follow the right-hand weapon.
            bool twoHandPoseActive = false;
            for (int i = 0; i < m_machine.poses.count; ++i)
                twoHandPoseActive = twoHandPoseActive ||
                    (m_machine.poses.w[i] > 0.005f &&
                     (m_machine.poses.key[i] / 12 == static_cast<int>(ov::PoseOverlay::Pistol2H) ||
                      m_machine.poses.key[i] / 12 == static_cast<int>(ov::PoseOverlay::Rifle)));
            // Keep the authored contact while a 2H pose fades out to 1H. Dropping IK on the input switch
            // itself releases an almost-full 2H pose in one frame and snaps the left upper arm.
            // A verified 1H asset may only drive a settled 1H pose. During the
            // 2H->1H fade its authored contact still owns both hands.
            const bool oneHandShoot = aIn.shootOneHand &&
                aIn.machine.overlay == ov::PoseOverlay::Pistol1H && !twoHandPoseActive;
            const bool shootActive = aIn.shootDeltaValid && aIn.shootWeight > ov::kZeroWeight &&
                !aIn.gripSkip && (aIn.shootOneHand ? oneHandShoot : aIn.authoredTwoHandGrip) &&
                m_rig.Valid(sk) && m_shootClavL >= 0 && m_shootClavR >= 0;
            const bool authored = aIn.authoredTwoHandGrip && twoHandPoseActive;
            gik::Grip authoredGrip;
            if (authored && m_rig.Valid(sk))
                authoredGrip = gik::CacheGripFromGamePose(sk, *m_lay, m_rig);
            // rifle-grip-stock 2026-11-10 (SupportGrip.hpp): the long gun's own game grip on the handguard and the stock in the shoulder.
            // Weights = route gate (0.2 s, so a holster / hand-back / weapon switch never jumps) x the composed weight of the Rifle poses
            // (grip) / of the Rifle Aim poses (stock: only the aim pose is shouldered; in the overlay's Ready / Relaxed / sprint carry the
            // butt sits 10-14 cm above / beside the shoulder, a pose change, not a contact fix - offline measurement 2026-11-10). Applied after the aim offset,
            // on the final aimed pose; the authored relative grip above is captured before the weapon moves, so the left hand follows it.
            const sgrip::Entry* sgEntry = nullptr;
            float supportW = 0.0f, stockW = 0.0f, sgRifleW = 0.0f, sgReachApplied = 0.0f, sgStockShift = 0.0f;
            bool sgRelease = false, sgSlide = false; // the gate falls with an entry that is not this frame's weapon data (sgrip::Release)
#if !(defined(UTPP_SUPPORTGRIP_MUTATION) && UTPP_SUPPORTGRIP_MUTATION == 1) // mutation 1: no support grip / stock pass (the build before)
            if (m_rig.Valid(sk))
            {
                float wRifle = 0.0f, wShoulder = 0.0f;
                for (int i = 0; i < m_machine.poses.count; ++i)
                {
                    ov::PoseOverlay o; ov::AimState a; bool c, mv;
                    ov::DecodePoseKey(m_machine.poses.key[i], o, a, c, mv);
                    if (o != ov::PoseOverlay::Rifle || !(m_machine.poses.w[i] > 0.0f))
                        continue;
                    wRifle += m_machine.poses.w[i];
                    if (a == ov::AimState::Aiming)
                        wShoulder += m_machine.poses.w[i];
                }
                sgRifleW = wRifle;
                // gate time from the hand's way (release / regrip <= sgrip::kMaxHandSpeed, >= 0.2 s), with the entry the gate moves: while
                // it falls with its last entry under other weapon data (another weapon, nullptr), that last entry (review round 5)
                const sgrip::Gate& gate = m_supportGate;
                const bool fallingOld = gate.have && gate.g > 0.0f && gate.last && aIn.supportGrip != gate.last;
                const sgrip::Entry* travelEntry = fallingOld ? gate.last : (aIn.supportGrip ? aIn.supportGrip : gate.last);
                m_gateSeconds = travelEntry ? sgrip::GateSeconds(sgrip::GateTravel(sk, *m_lay, m_rig, *travelEntry)) : sgrip::kGateSeconds;
                sgEntry = m_supportGate.Step(aIn.supportGrip, authored && aIn.supportGrip != nullptr, aDt, m_gateSeconds);
                if (!sgEntry)
                {
                    m_routeW = 0.0f; // gate closed
                    m_release.ResetSlides(); // the left-hand follower may still run (sgrip::FollowLeftHand)
                }
                if (sgEntry)
                {
                    // route weight: gate x Rifle pose weight, the product rate-limited like the gate (an overlay switch, e.g. holster
                    // from Aim, drops the Rifle weight faster than the hand may leave the handguard) and never above the gate (the
                    // entry ends with it)
                    const float target = m_supportGate.g * ov::Clamp01(wRifle);
                    const float step = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.1f) / m_gateSeconds : 0.0f;
                    m_routeW = m_routeWHave ? std::fmin(m_routeW + step, std::fmax(m_routeW - step, target)) : target;
                    m_routeW = std::fmin(m_routeW, m_supportGate.g);
                    m_routeWHave = true;
                    supportW = m_routeW;
                    stockW = sgEntry->handFirst ? 0.0f : m_supportGate.g * ov::Clamp01(wShoulder); // hand priority: no stock pass
                    sgRelease = aIn.supportGrip != sgEntry;
                    if (sgRelease)
                    {
                        // release (sgrip::Release): no stock pass and no grip of the old entry; the slide fraction f falls with the route
                        // weight, never rises (supportW = the weight left: diagnostics and the male swivel ramp)
                        m_release.f = m_release.slideValid && m_release.routeW > ov::kZeroWeight
                            ? std::fmin(m_release.f, ov::Clamp01(m_routeW / m_release.routeW)) : 0.0f;
                        stockW = 0.0f;
                        supportW = m_release.f * m_release.routeW;
                    }
                }
            }
#else
            (void)stockW, (void)sgSlide, (void)sgReachApplied, (void)sgStockShift;
#endif
            // motionfix item 2: aim offset (sweep + spine yaw + hand fix), alpha = AimingWeight
#if !defined(UTPP_POSEV2_AO_MUTATION) || UTPP_POSEV2_AO_MUTATION != 1
            ov::Xf aimBefore[kPoseDiagnosticBoneNames.size()];
            if (aIn.capturePoseDiagnostics)
                CaptureStageBones(*m_lay, aimBefore);
            // aim pitch plan 2026-10-30 B: closed barrel pitch loop on the sweep input + bounded residual (BarrelPitchLoop.hpp);
            // not engaged (loop off or no aim point, nothing left to fade) = the open-loop aim offset bit for bit
            const bool barrelLoop = BarrelLoopBegin(aIn, info, aDt);
            ApplyAimOffset(sk, aIn, info, aDt, barrelLoop);
            if (barrelLoop)
                ApplyBarrelLoop(sk, aIn, info, aDt, shootActive);
            if (aIn.capturePoseDiagnostics)
                CaptureStageDelta(aimBefore, *m_lay, info.aimDelta);
#endif
#if !(defined(UTPP_SUPPORTGRIP_MUTATION) && UTPP_SUPPORTGRIP_MUTATION == 1)
            if (sgEntry && sgRelease)
            {
                // release (sgrip::Release, review round 5): none of the old entry's geometry on this pose (it may hold another weapon). The
                // weapon's last slide (reach shift + stock pass) falls with f; the left hand: sgrip::FollowLeftHand after the grip IK.
                info.stockWeight = 0.0f;
                const float f = m_release.f;
                m_reachShift = m_release.reach * f;
                m_reachShiftHave = true;
                const float slide = (m_release.reach + m_release.stock) * f;
                sgSlide = std::fabs(slide) > 1e-5f;
                if (std::fabs(slide) > 1e-5f && sgrip::SlideWeaponBack(sk, *m_lay, m_rig, slide, m_gripParams))
                {
                    info.reachShiftM = m_reachShift;
                    info.stockShiftM = -m_release.stock * f;
                }
            }
            else if (sgEntry)
            {
                info.stockWeight = stockW;
                if (stockW > ov::kZeroWeight && m_pocketBone >= 0)
                {
                    sgrip::StockResult stock;
                    sgrip::ApplyStock(sk, *m_lay, m_rig, m_pocketBone, aIn.bodyWoman ? sgrip::kPocketWoman : sgrip::kPocketMan, *sgEntry,
                                      stockW, m_gripParams, stock);
                    info.stockGapM = stock.gap;
                    info.stockShiftM = stock.shift;
                    if (stock.applied)
                        sgStockShift = stock.shift;
                }
                // Reach-limited grip: the largest weight (<= supportW) whose hand target the left arm reaches inside the soft reach band,
                // so the hand never floats short of the handguard and the coupled IK below never has to move the weapon for it.
                // Reach shift: the weapon slides back along its barrel just enough for the left arm to reach the handguard (<= the
                // weapon's maxShift: 3 cm, Hercules 10 cm; rate-limited to the cap in the gate's time (>= 0.2 s, hand <= 0.6 m/s); 0 when
                // the hand already reaches). Remaining need -> reach-limited weight below.
                float shiftWant = 0.0f;
                if (authored && authoredGrip.valid && supportW > ov::kZeroWeight)
                {
                    info.reachShiftNeedM = sgrip::ReachShiftNeeded(sk, *m_lay, m_rig, authoredGrip, *sgEntry, supportW, m_gripParams.softStart);
                    shiftWant = std::fmin(info.reachShiftNeedM, sgEntry->maxShift);
                    // review round 5: a stock pass weapon (Masamune, Ajax, Saratoga) slides back only until its stock sits
                    // sgrip::kStockGapTolerance behind the shoulder pocket; the rest of the reach goes to the reach-limited weight
                    if (stockW > ov::kZeroWeight && m_pocketBone >= 0)
                    {
                        const float gap = sgrip::StockGap(sk, *m_lay, m_rig, m_pocketBone,
                                                          aIn.bodyWoman ? sgrip::kPocketWoman : sgrip::kPocketMan, *sgEntry);
                        if (std::isfinite(gap))
                            shiftWant = std::fmin(shiftWant, std::fmax(0.0f, gap + sgrip::kStockGapTolerance));
                    }
                }
                if (std::isfinite(aDt) && aDt > 0.0f)
                {
                    const float rate = std::fmin(aDt, 0.1f) * sgEntry->maxShift / m_gateSeconds;
                    m_reachShift = m_reachShiftHave ? std::fmin(m_reachShift + rate, std::fmax(m_reachShift - rate, shiftWant)) : shiftWant;
                    m_reachShiftHave = true;
                }
                if (m_reachShift > 1e-5f && sgrip::SlideWeaponBack(sk, *m_lay, m_rig, m_reachShift, m_gripParams))
                {
                    info.reachShiftM = m_reachShift;
                    sgReachApplied = m_reachShift;
                }
                if (authored && authoredGrip.valid && supportW > ov::kZeroWeight)
                {
                    supportW = sgrip::ReachLimitedWeight(sk, *m_lay, m_rig, authoredGrip, *sgEntry, supportW, m_gripParams.softStart);
                    // ... and whose hand rotation the support-hand swivel limit below can still bring to its 85 deg (both bodies)
                    supportW = sgrip::TwistLimitedWeight(sk, *m_lay, m_rig, m_forearmTwist, authoredGrip, *sgEntry, supportW, m_gripParams,
                                                         ov::kSupportHandTwistLimitDeg, ov::kSupportHandMaxSwivelDeg);
                    authoredGrip = sgrip::BlendGrip(authoredGrip, *sgEntry, supportW);
                }
                // slides of this frame for a release (sgrip::Release)
                m_release.routeW = m_routeW;
                m_release.reach = sgReachApplied;
                m_release.stock = -sgStockShift;
                m_release.f = 1.0f;
                m_release.slideValid = m_routeW > ov::kZeroWeight;
            }
            else
                m_reachShift = 0.0f; // gate closed: the shift has decayed with the route weight
            info.supportGripWeight = supportW;
#endif
            // (3) grip IK after the layering
            if (m_rig.Valid(sk) && !aIn.gripSkip)
            {
                const gik::Grip* left = oneHandShoot ? nullptr :
                    (authored ? (authoredGrip.valid ? &authoredGrip : nullptr) : aIn.leftGrip);
                if (left && left->valid) info.gripTarget = *left;
                info.gripFromOverlay = authored && authoredGrip.valid;
                gik::GripInputs gi = gik::InputsFromCurves(info.curves, left, aIn.rightTarget);
                gi.coupleWeaponHands = info.gripFromOverlay;
                if (oneHandShoot) gi.handIkL = 0;
                ov::Xf gripBefore[kPoseDiagnosticBoneNames.size()];
                if (aIn.capturePoseDiagnostics)
                    CaptureStageBones(*m_lay, gripBefore);
                bool actionApplied = false;
                bool shootBaseReady = false;
                gik::GripResult shootBaseGrip;
                if (shootActive && (oneHandShoot || authoredGrip.valid))
                {
                    // The activation endpoint must match the ordinary zero-weight
                    // path: same-frame AO AND its normal authored contact pass.
                    // A pre-AO or pre-grip base would introduce another hand jump.
                    *m_shootFallback = *m_lay;
                    shootBaseReady = gik::ApplyGripIK(sk, *m_shootFallback, m_rig, gi,
                                                      m_gripParams, shootBaseGrip);
                    math::Quat rhRot;
                    math::Vec3 rhPos;
                    gik::ModelOf(sk, *m_lay, m_rig.right.hand, rhRot, rhPos);
                    const ov::Xf rightModel{rhRot, rhPos};
                    ov::Xf weaponModel;
                    gik::ModelOf(sk, *m_lay, m_rig.weaponRight, weaponModel.r, weaponModel.t);
                    ov::Xf baselineRight;
                    gik::ModelOf(sk, *m_shootFallback, m_rig.right.hand, baselineRight.r, baselineRight.t);
                    const float activation = ov::Clamp01(aIn.shootWeight);
                    gik::GripResult candidate;
                    const auto solveMotion = [&](float scale) {
                        // Solve the reachable recoil first. Fading only this delta
                        // still activates soft-reach IK abruptly even near zero.
                        const ov::Xf weighted = ov::BlendXf(ov::Xf{}, aIn.shootDelta,
                                                          scale);
                        const ov::Xf target = native_shoot::RightHandTarget(weighted, weaponModel, rightModel);
                        gik::GripInputs shot = gi;
                        shot.rightTarget = &target;
                        shot.handIkR = 1.0f;
                        shot.handIkL = oneHandShoot ? 0.0f : 1.0f;
                        if (oneHandShoot) shot.leftGrip = nullptr;
                        shot.coupleWeaponHands = false;
                        std::memcpy(m_tmp->b, m_lay->b, sizeof(ov::Xf) * static_cast<size_t>(sk.n));
                        if (!shootBaseReady ||
                            !gik::ApplyGripIK(sk, *m_tmp, m_rig, shot, m_gripParams, candidate) ||
                            !candidate.right.ok || (!oneHandShoot && !candidate.left.ok) || candidate.right.handErr > 0.01f ||
                            (!oneHandShoot && candidate.left.handErr > 0.01f) ||
                            !native_shoot::BlendSolvedArms(sk, m_rig, *m_shootFallback, *m_tmp,
                                                           activation, *m_tmp)) return false;
                        // The free arm is not part of a 1H shot, including at full
                        // weight. Copy exactly rather than normalizing equal quats.
                        if (oneHandShoot)
                            for (int bone : {m_rig.left.upper, m_rig.left.lower, m_rig.left.hand})
                                m_tmp->b[bone] = m_shootFallback->b[bone];
                        // Blending rotations is not linear hand motion. Recheck
                        // both final contacts rather than reusing pre-blend errors.
                        ov::Xf finalRight, finalLeft, finalWeapon;
                        gik::ModelOf(sk, *m_tmp, m_rig.right.hand, finalRight.r, finalRight.t);
                        if (!oneHandShoot)
                        {
                            gik::ModelOf(sk, *m_tmp, m_rig.left.hand, finalLeft.r, finalLeft.t);
                            gik::ModelOf(sk, *m_tmp, m_rig.weaponRight, finalWeapon.r, finalWeapon.t);
                        }
                        const ov::Xf fadedTarget = ov::BlendXf(baselineRight, target, activation);
                        candidate.right.handErr = gik::Len(finalRight.t - fadedTarget.t);
                        candidate.left.handErr = oneHandShoot ? 0.0f : gik::Len(finalLeft.t - (finalWeapon.t +
                            math::QuatRotate(finalWeapon.r, authoredGrip.t)));
                        return std::isfinite(candidate.right.handErr) && candidate.right.handErr <= 0.01f &&
                            (oneHandShoot || (std::isfinite(candidate.left.handErr) && candidate.left.handErr <= 0.01f));
                    };
                    float reachable = 1.0f;
                    bool solved = solveMotion(reachable);
                    // A reachable part of the recoil is preferable to dropping the whole
                    // action to the current overlay on one frame. Bounded search; no limb
                    // stretching, allocation, or relaxation of the 1 cm contact criterion.
                    if (!solved && solveMotion(0.0f))
                    {
                        float high = 1.0f; reachable = 0.0f;
                        for (int attempt = 0; attempt < 6; ++attempt)
                        {
                            const float middle = (reachable + high) * 0.5f;
                            if (solveMotion(middle)) reachable = middle;
                            else high = middle;
                        }
                        solved = solveMotion(reachable);
                    }
                    if (solved)
                    {
                        std::memcpy(m_lay->b, m_tmp->b, sizeof(ov::Xf) * static_cast<size_t>(sk.n));
                        info.grip = candidate;
                        info.shootApplied = true;
                        info.shootScale = reachable;
                        actionApplied = true;
                    }
                }
                if (!actionApplied)
                {
                    if (shootBaseReady)
                    {
                        *m_lay = *m_shootFallback;
                        info.grip = shootBaseGrip;
                    }
                    else
                        gik::ApplyGripIK(sk, *m_lay, m_rig, gi, m_gripParams, info.grip);
                }
                // rifle-grip-stock: a fading route (holster / hand-back: no authored contact any more) keeps the left hand on the game grip
                // with the falling weight instead of dropping it in one frame; then the game grip fingers. Only with the route's own weapon
                // data (not in a release) and on a pose with Rifle weight (review round 5: e.g. not on the Pistol1H poses).
                const bool onGrip = left && left->valid;
                const bool routeOn = sgEntry && !sgRelease && m_routeW > ov::kZeroWeight && sgRifleW > 0.0f;
                if (routeOn && supportW > ov::kZeroWeight)
                {
                    if (!onGrip && !oneHandShoot)
                    {
                        math::Quat rw;
                        math::Vec3 pw;
                        gik::ModelOf(sk, *m_lay, m_rig.weaponRight, rw, pw);
                        const math::Vec3 tp = pw + math::QuatRotate(rw, sgEntry->t);
                        const math::Quat tr = math::QuatNormalize(math::QuatMul(rw, sgEntry->r));
                        gik::SolveArm(sk, *m_lay, m_rig.left, tp, &tr, supportW, m_gripParams, info.grip.left);
                    }
                    // the fingers close as far as the hand is on the grip (Enable_HandIK_L of the layered curves on the authored route)
                    sgrip::ApplyFingers(*m_lay, m_supportFingers, *sgEntry, supportW * (onGrip ? ov::Clamp01(gi.handIkL) : 1.0f));
                }
#if !(defined(UTPP_SUPPORTGRIP_MUTATION) && UTPP_SUPPORTGRIP_MUTATION == 1)
                // release (review round 5): the left hand leaves the handguard from where it was, <= sgrip::kMaxHandSpeed over its target
                // (sgrip::FollowLeftHand); anchors only when the route drops with its gate open, otherwise the pose stays bit for bit
                info.supportReleaseM = sgrip::FollowLeftHand(sk, *m_lay, m_rig, m_spine3, m_supportFingers, routeOn,
                                                             sgEntry != nullptr && !routeOn && m_release.prevApplied,
                                                             info.gripFromOverlay, sgSlide, aDt, m_gripParams, m_release);
#endif
                if (aIn.capturePoseDiagnostics)
                    CaptureStageDelta(gripBefore, *m_lay, info.gripDelta);
            }
            else
            {
                m_release.prevValid = m_release.active = false; // no grip IK this frame (grip skip / no rig): nothing to follow
                m_release.prevApplied = false;
            }
            // Game forearm convention (ForearmTwist.hpp): pure-hinge forearm, roll in the hand. The overlay compose can
            // leave up to ~125 deg of roll in the forearm (Pistol2H relaxed idle), which the hand-driven twist helpers of
            // the deformation rig cannot spread -> pinched elbow. Hand / weapon / finger model transforms are unchanged.
#if !(defined(UTPP_FOREARMTWIST_MUTATION) && UTPP_FOREARMTWIST_MUTATION == 1) // mutation 1: pass not called
            ov::MoveForearmTwistToHands(m_forearmTwist, *m_lay, info.forearmTwistMovedDeg);
#endif
            // rifle-hand-fix 2026-11-09, support hand of the two-handed overlay poses (Pistol2H, Rifle): the motion Pistol2H
            // Relaxed / Ready poses leave 105-130 deg of twist in the support hand once the roll sits there (above). An elbow swivel
            // brings it to kSupportHandTwistLimitDeg, the top of the game's own Female V support hands (ForearmTwist.hpp). Hand, weapon
            // and finger model transforms stay exact; below the limit the pose is untouched bit for bit. Weight = composed weight of the
            // 2H poses only (no switch on the grip route, so a reload hand-over cannot pop it). Female V only (limit calibrated on the
            // game's Female V poses; male untouched bit for bit). UTPP_SUPPORTHAND_LEGACY (tests / digest re-recording only, never in
            // the DLL): pass compiled out = the build before.
#if !defined(UTPP_SUPPORTHAND_LEGACY)
            if (m_rig.Valid(sk) && aIn.bodyWoman)
            {
                float w2H = 0.0f;
                for (int i = 0; i < m_machine.poses.count; ++i)
                {
                    const int overlay = m_machine.poses.key[i] / 12;
                    if (m_machine.poses.w[i] > 0.0f &&
                        (overlay == static_cast<int>(ov::PoseOverlay::Rifle) || overlay == static_cast<int>(ov::PoseOverlay::Pistol2H)))
                        w2H += m_machine.poses.w[i];
                }
                const ov::HandTwistLimitResult support = ov::LimitHandTwistBySwivel(*m_lay, m_rig.left.upper, m_rig.left.lower,
                    m_rig.left.hand, m_forearmTwist.childCount[0], ov::kSupportHandTwistLimitDeg, ov::kSupportHandMaxSwivelDeg,
                    ov::Clamp01(w2H));
                info.supportTwistBeforeDeg = support.twistBeforeDeg;
                info.supportTwistAfterDeg = support.twistAfterDeg;
                info.supportSwivelDeg = support.swivelDeg;
            }
#if !(defined(UTPP_SUPPORTGRIP_MUTATION) && UTPP_SUPPORTGRIP_MUTATION == 1)
            // rifle-grip-stock 2026-11-10: the game grip on the handguard (above) brings the game's hand rotation to the Male V support
            // hand too (up to 134 deg of twist offline). The same swivel limit, on the male body only with the weight of that route
            // (supportW), so every other male pose stays bit for bit. Steep ramp (full at supportW 0.25): the limit does nothing under 85 deg
            // and is continuous there, and a partial route weight (reach-limited grip 0.77-0.96) must not leave 86-88 deg.
            else if (m_rig.Valid(sk) && supportW > ov::kZeroWeight)
            {
                const ov::HandTwistLimitResult support = ov::LimitHandTwistBySwivel(*m_lay, m_rig.left.upper, m_rig.left.lower,
                    m_rig.left.hand, m_forearmTwist.childCount[0], ov::kSupportHandTwistLimitDeg, ov::kSupportHandMaxSwivelDeg,
                    ov::Clamp01(supportW * 4.0f));
                info.supportTwistBeforeDeg = support.twistBeforeDeg;
                info.supportTwistAfterDeg = support.twistAfterDeg;
                info.supportSwivelDeg = support.swivelDeg;
            }
#endif
#endif
            FromOv(*m_lay, aOut);
            info.layered = true;
            if (m_rig.left.hand >= 0 && m_rig.weaponRight >= 0)
            {
                ov::MeshPositions(m_ov->Skel(), *m_lay, m_meshPos, m_meshRot);
                const math::Vec3 d = {m_meshPos[m_rig.left.hand].x - m_meshPos[m_rig.weaponRight].x,
                                      m_meshPos[m_rig.left.hand].y - m_meshPos[m_rig.weaponRight].y,
                                      m_meshPos[m_rig.left.hand].z - m_meshPos[m_rig.weaponRight].z};
                info.leftHandToWeapon = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            }
            info.overlaySteady = m_machine.poses.count == 1 && m_machine.aimW.count == 1 && m_machine.bases.count == 1;
        }
        else
        {
            m_aoFilter.Reset(); // no layering this frame: no aim offset either, nothing to carry over
            m_barrel.ResetApplied(); // the learned biases stay
            std::memcpy(aOut, m_base.data(), sizeof(BoneTransform) * m_n);
        }
        for (uint32_t i = 0; i < m_n && ok; ++i)
            ok = Finite(aOut[i]);
        if (!ok)
            std::memcpy(aOut, m_base.data(), sizeof(BoneTransform) * m_n);
        info.finite = ok;
        const float us = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
        info.evalUs = us;
        m_times[m_timeCount % m_times.size()] = us;
        ++m_timeCount;
        if (aInfo)
            *aInfo = info;
        return ok;
    }

public:
    // p95 of the last min(count, Budget::kWindow) Evaluate times (us). Allocation-free (fixed scratch).
    float EvalP95Us()
    {
        const size_t n = std::min<size_t>(m_timeCount, m_times.size());
        if (n == 0)
            return 0.0f;
        std::copy(m_times.begin(), m_times.begin() + static_cast<std::ptrdiff_t>(n), m_sorted.begin());
        const size_t k = static_cast<size_t>(std::ceil(0.95 * static_cast<double>(n))) - 1;
        std::nth_element(m_sorted.begin(), m_sorted.begin() + static_cast<std::ptrdiff_t>(k),
                         m_sorted.begin() + static_cast<std::ptrdiff_t>(n));
        return m_sorted[k];
    }

    uint32_t NumBones() const { return m_n; }
    std::span<const BoneTransform> BasePose() const { return m_base; }
    bool UsesLiveBase() const { return m_usesLiveBase; }
    const motionstack::Signals& BaseSignals() const { return m_stack.OutputSignals(); }
    int CancelledStartSupport() const { return m_usesLiveBase ? m_stack.CancelledStartSupport() : -1; }
    const ov::OverlayMachine& Machine() const { return m_machine; }
    const ov::Skeleton& Skel() const { return m_ov->Skel(); }
    const gik::Rig& GripRig() const { return m_rig; }
    int OvBone(uint32_t aDbBone) const { return m_dbToOv[aDbBone]; }
    void ToOv(const BoneTransform* aIn, ov::Pose& aOut) const
    {
        // scale is not part of the .utppmm pose: keep the overlay rest scale of the base pose (B of the stand idle key)
        aOut = m_basePoses[0];
        for (uint32_t i = 0; i < m_n; ++i)
        {
            ov::Xf& x = aOut.b[m_dbToOv[i]];
            x.r = aIn[i].q;
            x.t = aIn[i].p;
        }
    }
    void FromOv(const ov::Pose& aIn, BoneTransform* aOut) const
    {
        for (uint32_t i = 0; i < m_n; ++i)
        {
            const ov::Xf& x = aIn.b[m_dbToOv[i]];
            aOut[i].q = x.r;
            aOut[i].p = x.t;
        }
    }

private:
    void CaptureStageBones(const ov::Pose& aPose, ov::Xf (&aOut)[kPoseDiagnosticBoneNames.size()]) const
    {
        for (size_t i = 0; i < kPoseDiagnosticBoneNames.size(); ++i)
        {
            const int b = m_poseDiagBones[i];
            aOut[i] = b >= 0 ? aPose.b[b] : ov::Xf{};
        }
    }
    void CaptureStageDelta(const ov::Xf (&aBefore)[kPoseDiagnosticBoneNames.size()], const ov::Pose& aAfter,
                           FrameInfo::PoseStageDelta& aOut) const
    {
        for (size_t i = 0; i < kPoseDiagnosticBoneNames.size(); ++i)
        {
            const int b = m_poseDiagBones[i];
            if (b < 0)
                continue;
            const ov::Xf& after = aAfter.b[b];
            aOut.rotDeg[i] = ov::AngleDeg(aBefore[i].r, after.r);
            const math::Vec3 delta = after.t - aBefore[i].t;
            aOut.posM[i] = std::sqrt(math::Dot(delta, delta));
        }
    }
    void CaptureStageDelta(const ov::Pose& aBefore, const ov::Pose& aAfter, FrameInfo::PoseStageDelta& aOut) const
    {
        ov::Xf before[kPoseDiagnosticBoneNames.size()];
        CaptureStageBones(aBefore, before);
        CaptureStageDelta(before, aAfter, aOut);
    }

    // motionfix item 2 (report 3.11 / 3.12, motion ABP order): on the layered pose m_lay
    //   (a) OPB Aiming: the overlay's AO sweep at u = SweepTimeFromPitch(pitch) as a mesh-space additive, alpha = AimingWeight;
    //   (b) spine rotation (Enable_SpineRotation branch; the Aim poses carry SpineRot = 1): clamp(yaw, +-kAimSpineYawClampDeg)
    //       spread over Spine..Spine3 about mesh +Z, scaled by AimingWeight;
    //   (c) hand fix: LBB_LS restores the clavicles + ik_hand bones from the pre-AO pose with Layering_Hand_L / R.
    // Fix batch 2 (AimSpineYawFilter.hpp): the angles come through the filter. The side of the +-60 clamp is kept while the
    // target is behind the back and the applied yaw moves at most kAoYawRateDegPerSec (no +60 -> -60 jump at +-180); a lost
    // target (haveAim false) keeps its last angles and fades out with the Aiming weight of the machine (alpha = min(a, release),
    // zero within kAoLatchMaxAgeSeconds) instead of ending the whole AO in one frame. aoSuppress (handback) fades it in 0.15 s.
    void ApplyAimOffset(const ov::Skeleton& aSk, const OverlayFrameInputs& aIn, FrameInfo& aInfo, float aDt, bool aBarrel)
    {
        const float a = ov::Clamp01(m_machine.AimingWeight());
        aInfo.aoWeight = a;
        // The external (traversal montage) base never carries a latch: it had no AO without a target before.
        if (aIn.montageBase && !aIn.haveAim)
        {
            m_aoFilter.Reset();
            m_barrel.ResetApplied(); // the learned biases stay
            return;
        }
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 3
        constexpr float kYawClampDeg = 360.0f; // mutation 3: no clamp
#else
        constexpr float kYawClampDeg = kAimSpineYawClampDeg;
#endif
        AimSpineYawFilter::Output ao;
        const bool have = m_aoFilter.Step(aIn.haveAim, a, aIn.aimYawDeg, aIn.aimPitchDeg, aDt, kYawClampDeg,
                                          aIn.machine.overlay == ov::PoseOverlay::Rifle ? 1 : 0, aIn.aoSuppress, ao);
        if (!have || a <= ov::kZeroWeight)
            return;
        float alpha = ao.release < 1.0f ? std::min(a, ao.release) : a; // = a bit for bit while a target drives the AO
        if (aIn.aoScale < 1.0f)
        {
            // Reload hand-over (AoReloadScale): the weight of the AO is capped by the MM share of the arms, min(a, release, s). ONE formula
            // for the live AO and the latched one, so the weight is continuous where the target is lost (a * s for the live AO and
            // min(a, s) for the latch jumped by up to 0.25 of the AO: a = 0.5, s = 0.6 gave 0.30 -> 0.50); not release * s either, that
            // would square the hand-over curve. A LATCHED AO is also used up (a cancelled or short reload cannot bring it back).
            const float s = ov::Clamp01(aIn.aoScale);
            if (!ao.live)
                m_aoFilter.CapRelease(s);
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 17
            alpha = ao.live ? alpha * s : std::min(alpha, s); // mutation 17: the product for the live AO, the minimum for the latch
#else
            alpha = std::min(alpha, s);
#endif
        }
        if (!(alpha > ov::kZeroWeight))
            return; // the reload has handed the upper body over: no AO, the layered pose stays untouched
        aInfo.aoAppliedWeight = alpha;
        aInfo.aoLatched = !ao.live;
        std::memcpy(m_aoPre->b, m_lay->b, sizeof(ov::Xf) * static_cast<size_t>(aSk.n));
        aInfo.aoPitchInDeg = aInfo.aoPitchCmdDeg = ao.pitchDeg;
        if (aBarrel)
            aInfo.aoPitchCmdDeg = ao.pitchDeg + m_barrelCmd; // B2: the bias of the barrel loop on the sweep input
        aInfo.aoSweepU = ov::SweepTimeFromPitch(aInfo.aoPitchCmdDeg);
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 4
        aInfo.aoSweepU = 0.5f; // mutation 4: pitch ignored
#endif
        const ov::Additive* sweepSlots[6];
        float sweepWeights[6];
        int sweepIds[6];
        int sweepCount = 0;
        for (int k = 0; k < m_machine.poses.count; ++k)
        {
            const int key = m_machine.poses.key[k];
            const int o = key / 12;
            const int d = o >= 0 && o < kOverlays ? m_sweep[o][(key & 2) != 0 ? 1 : 0] : -1;
            int existing = 0;
            while (existing < sweepCount && sweepIds[existing] != d) ++existing;
            if (existing < sweepCount)
            {
                sweepWeights[existing] += m_machine.poses.w[k];
                continue;
            }
            sweepIds[sweepCount] = d;
            ov::Additive& slot = m_aoSlots[sweepCount];
            if (d < 0)
                slot.SetIdentity(aSk.n, true);
            else
            {
                const int frames = static_cast<int>(m_ov->DeltaAt(d).numFrames);
                const float f = aInfo.aoSweepU * static_cast<float>(frames - 1);
                const int i0 = std::min(frames - 1, static_cast<int>(f));
                const int i1 = std::min(frames - 1, i0 + 1);
                m_ov->DeltaFrame(d, i0, true, *m_aoA);
                m_ov->DeltaFrame(d, i1, true, *m_aoB);
                const ov::Additive* s[2] = {m_aoA.get(), m_aoB.get()};
                const float t = f - static_cast<float>(i0);
                const float w[2] = {1.0f - t, t};
                ov::BlendAdditives(s, w, 2, slot);
            }
            sweepSlots[sweepCount] = &slot;
            sweepWeights[sweepCount++] = m_machine.poses.w[k];
        }
        if (sweepCount > 0)
        {
            const ov::Additive* sweep = sweepSlots[0];
            if (sweepCount > 1)
            {
                ov::BlendAdditives(sweepSlots, sweepWeights, sweepCount, *m_aoAdd);
                sweep = m_aoAdd.get();
            }
            ov::ApplyAdditive(aSk, *m_lay, *sweep, alpha);
        }
        const float yaw = ao.yawDeg; // clamped to +-kAimSpineYawClampDeg by the filter
        aInfo.aoYawDeg = yaw;
        const ov::Regions& r = m_ov->Masks();
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 3
        ov::ApplySpineYaw(aSk, *m_lay, r.spineYawBones, r.spineYawCount, alpha * yaw * math::kPi / 180.0f, 2.0f * math::kPi);
#else
        ov::ApplySpineYaw(aSk, *m_lay, r.spineYawBones, r.spineYawCount, alpha * yaw * math::kPi / 180.0f,
                          kAimSpineYawClampDeg * math::kPi / 180.0f);
#endif
#if !defined(UTPP_POSEV2_AO_MUTATION) || UTPP_POSEV2_AO_MUTATION != 2
        ov::RestoreHandsAfterAO(aSk, *m_aoPre, m_clavL, m_clavR, aInfo.curves, *m_lay);
#endif
        aInfo.aoApplied = true;
    }

    // Aim pitch plan 2026-10-30 B (BarrelPitchLoop.hpp). Slots of this frame (overlay x stance of the machine pose keys x
    // style) and the bias command for the sweep. False = bypass (loop off or no aim point and nothing left to fade out).
    bool BarrelLoopBegin(const OverlayFrameInputs& aIn, FrameInfo& aInfo, float aDt)
    {
#if defined(UTPP_POSEV2_AO_MUTATION) && UTPP_POSEV2_AO_MUTATION == 18
        (void)aIn;
        (void)aInfo;
        (void)aDt;
        return false; // mutation 18: the loop never applies (open-loop aim offset)
#else
        const bool run = aIn.barrelLoop && aIn.haveAimPointVisual && math::IsFinite(aIn.aimPointVisual);
        if (!run && m_barrel.Idle())
            return false;
        m_barrelSlotCount = 0;
        for (int k = 0; k < m_machine.poses.count; ++k)
        {
            const int key = m_machine.poses.key[k];
            const int o = key / 12;
            if (o < 0 || o >= kOverlays)
                continue;
            for (int j = 0; j < m_machine.styles.count && m_barrelSlotCount < kBarrelSlotMax; ++j)
            {
                const int st = m_machine.styles.key[j];
                const float w = m_machine.poses.w[k] * m_machine.styles.w[j];
                if (st < 0 || st >= 3 || !(w > 0.0f))
                    continue;
                m_barrelSlots[m_barrelSlotCount] = ((o * 2) + ((key & 2) != 0 ? 1 : 0)) * 3 + st;
                m_barrelW[m_barrelSlotCount++] = w;
            }
        }
        m_barrelCmd = m_barrel.Command(m_barrelSlots, m_barrelW, m_barrelSlotCount, run, aDt);
        aInfo.barrelActive = true;
        aInfo.barrelBiasDeg = m_barrelCmd;
        aInfo.barrelAxis = aIn.barrelAxis;
        return true;
#endif
    }

    // B1 measure after the aim offset, B2 learn, B3 residual about the lateral axis of the barrel (Spine2 / Spine3 share above
    // kBarrelSpineStartDeg, the rest at RightArm), B4 exclusions. The left hand is re-solved by the grip IK that follows.
    void ApplyBarrelLoop(const ov::Skeleton& aSk, const OverlayFrameInputs& aIn, FrameInfo& aInfo, float aDt, bool aShot)
    {
        const bool run = aIn.barrelLoop && aIn.haveAimPointVisual && math::IsFinite(aIn.aimPointVisual);
        uint32_t ex = run ? 0u : static_cast<uint32_t>(kBarrelExNoPoint);
        if (m_barrel.ShotExcluded(aShot, aDt))
            ex |= kBarrelExShot;
        if (aIn.aoScale < 1.0f)
            ex |= kBarrelExReload;
        if (aIn.aoSuppress)
            ex |= kBarrelExSuppress;
        if (aInfo.aoLatched)
            ex |= kBarrelExLatch;
        if (!aInfo.aoApplied || m_rig.weaponRight < 0)
        {
            // no aim offset this frame (its weight is ~0 at the edge): the residual ends with it
            m_barrel.ClearResidual();
            aInfo.barrelExcluded = ex | kBarrelExFade;
            return;
        }
        const float alpha = ov::Clamp01(aInfo.aoAppliedWeight);
        aInfo.barrelLoopWeight = alpha;
        if (alpha < kBarrelLearnMinAlpha)
            ex |= kBarrelExFade;
        math::Quat wr;
        math::Vec3 wt;
        gik::ModelOf(aSk, *m_lay, m_rig.weaponRight, wr, wt);
        BarrelMeasure m;
        // without a point (fading out) only the barrel direction is needed: any point ahead of the barrel
        const math::Vec3 point = run ? aIn.aimPointVisual : wt + math::QuatRotate(wr, aim::AxisVector(aIn.barrelAxis)) * 10.0f;
        const bool measured = MeasureBarrel(wr, wt, aIn.barrelAxis, aIn.barrelMuzzle, point, m);
        const float targetW = measured && run && std::fabs(m.errDeg) <= kBarrelMaxErrDeg ? BarrelTargetWeight(m.dist) : 0.0f;
        if (run)
            m_barrel.SetTargetWeight(targetW); // the next frame's bias command; without a point the bias fades anyway
        if (!(targetW >= 1.0f))
            ex |= kBarrelExTarget;
        if (measured && run)
        {
            aInfo.barrelMeasured = true;
            aInfo.barrelPitchDeg = m.barrelPitchDeg;
            aInfo.barrelWantPitchDeg = m.wantPitchDeg;
            aInfo.barrelErrBeforeDeg = aInfo.barrelErrAfterDeg = m.errDeg;
            aInfo.barrelTargetDist = m.dist;
        }
        aInfo.barrelExcluded = ex;
        if (ex == 0)
            m_barrel.Learn(m_barrelSlots, m_barrelW, m_barrelSlotCount, m.errDeg, aDt);
        // residual of this frame (BarrelLoopBegin stepped it); its next target integrates the error left after it
        const float residual = m_barrel.Residual() * alpha;
        aInfo.barrelResidualDeg = residual;
        // fade bits -> 0; shot / latch only -> frozen at its value (no pumping with the shots); none -> integrate
        m_barrel.SetResidualTarget((ex & kBarrelExFadeMask) != 0 ? 0.0f
                                   : ex != 0                     ? m_barrel.Residual()
                                                                 : m_barrel.Residual() - m.errDeg);
        math::Quat q;
        if (residual == 0.0f || !measured || m_rig.right.upper < 0)
            return;
        float armDeg, spineDeg;
        SplitBarrelResidual(residual, armDeg, spineDeg);
        if (spineDeg != 0.0f && m_spine2 >= 0 && m_spine3 >= 0 &&
            aim::YawPitchRotation(m.barrel, 0.0f, 0.5f * spineDeg * aim::kDegF, q))
        {
            RotateBoneMesh(aSk, m_spine2, q);
            RotateBoneMesh(aSk, m_spine3, q);
        }
        else
            armDeg = residual;
        if (!aim::YawPitchRotation(m.barrel, 0.0f, armDeg * aim::kDegF, q))
            return;
        RotateBoneMesh(aSk, m_rig.right.upper, q);
        if (aInfo.barrelMeasured)
        {
            gik::ModelOf(aSk, *m_lay, m_rig.weaponRight, wr, wt);
            BarrelMeasure after;
            if (MeasureBarrel(wr, wt, aIn.barrelAxis, aIn.barrelMuzzle, point, after))
            {
                aInfo.barrelErrAfterDeg = after.errDeg;
                if (ex == 0)
                    m_barrel.SetResidualTarget(m_barrel.Residual() - after.errDeg);
            }
        }
    }

    // Model-space rotation aQ of a bone about its own joint (the subtree follows): new model rotation = aQ * model rotation.
    void RotateBoneMesh(const ov::Skeleton& aSk, int aBone, const math::Quat& aQ)
    {
        math::Quat mr, pr{};
        math::Vec3 mp, pp;
        gik::ModelOf(aSk, *m_lay, aBone, mr, mp);
        const int p = aSk.parent[aBone];
        if (p >= 0)
            gik::ModelOf(aSk, *m_lay, p, pr, pp);
        const math::Quat nm = math::QuatMul(aQ, mr);
        m_lay->b[aBone].r = math::QuatNormalize(p < 0 ? nm : math::QuatMul(math::QuatInv(pr), nm));
    }

    static bool Fail(std::string& aErr, const char* aWhy)
    {
        aErr = aWhy;
        return false;
    }
    static bool Finite(const BoneTransform& b)
    {
        return std::isfinite(b.p.x) && std::isfinite(b.p.y) && std::isfinite(b.p.z) && std::isfinite(b.q.x) &&
               std::isfinite(b.q.y) && std::isfinite(b.q.z) && std::isfinite(b.q.w);
    }

    // B, OP, dP and curves from the machine crossfades (allocation-free: fixed arrays of pointers / weights).
    ov::Curves BuildLayerInputs()
    {
        const ov::OverlayMachine& m = m_machine;
        m_torsoOn = false; // CR-4: set by BuildTorsoSides
        // B
        const ov::Pose* bp[4];
        float bw[4];
        int nb = 0;
        for (int i = 0; i < m.bases.count; ++i)
        {
            bp[nb] = &m_basePoses[m.bases.key[i] & 3];
            bw[nb++] = m.bases.w[i];
        }
        if (nb == 0)
            *m_B = m_basePoses[0];
        else
            ov::BlendPosesWeighted(bp, bw, nb, *m_B);
        // OP + curves
        ov::Curves cur;
        const ov::Pose* op[6];
        float ow[6];
        int no = 0;
        float wNone = 0.0f;
        float freeArmW = 0.0f;
        for (int i = 0; i < m.poses.count; ++i)
        {
            const int key = m.poses.key[i];
            const int p = key >= 0 && key < kPoseKeys ? m_poseOfKey[key] : -1;
            if (p < 0)
            {
                wNone += m.poses.w[i];
                continue;
            }
            m_ov->FramePose(m_ov->PoseAt(p).firstFrame, m_opSlots[no]);
            ov::Curves c;
            m_ov->FrameCurves(p, m_ov->PoseAt(p).firstFrame, c);
            cur.Accumulate(c, m.poses.w[i]);
            {
                ov::PoseOverlay ko;
                ov::AimState ka;
                bool kc, km;
                ov::DecodePoseKey(key, ko, ka, kc, km);
                if (ko == ov::PoseOverlay::Pistol1H)
                    freeArmW += m.poses.w[i];
            }
            op[no] = &m_opSlots[no];
            ow[no++] = m.poses.w[i];
        }
        // motionfix item 6 (male chicken wing): the free left arm of the one-handed pistol poses (Hand IK L 0) takes the
        // full locomotion swing (Layering_Arm_L_Add x D_LS(B -> L)) on top of an aim pose that already holds the arm out and
        // back, so it swings up to / above horizontal; the add is scaled by kFreeArmAddScale there (see MotionPoseTest 8)
        // The scale fades out linearly with the Hand IK L curve (full at 0, none from 0.5): a hard < 0.5 switch made the arm add jump
        // 0.25 -> 1 inside the Ready1H -> Aim2H crossfade (MotionPoseTest transition step limit).
        if (freeArmW > 0.0f)
            cur.v[ov::kArmLAdd] *= 1.0f - freeArmW * (1.0f - m_freeArmScale) * (1.0f - std::min(1.0f, 2.0f * std::max(0.0f, cur[ov::kHandIKL])));
        m_haveOP = no > 0;
        if (m_haveOP)
        {
            // keys without an overlay pose (Default / Game) contribute L (the layering of L over itself is L)
            if (wNone > 0.0f)
            {
                m_opSlots[no] = *m_L;
                op[no] = &m_opSlots[no];
                ow[no++] = wNone;
            }
            ov::BlendPosesWeighted(op, ow, no, *m_OP);
            // P1 B2: the legs / pelvis regions (OP + (L - B)) take pose and layer weights split by the stance alpha of B (see
            // OverlayMachine::LowerBodyKeys): a stance change, then a move / aim change inside the 1 s of the base fade, no longer leaves
            // (OP - B) with the stand / crouch difference. Everything else above keeps the pose crossfade (the spine / head regions are
            // composed per stance side during a stance fade, CR-4: BuildTorsoSides).
            ov::OverlayMachine::LowerKeys lk;
            m.LowerBodyKeys(lk);
            const ov::Pose* lop[13];
            float low[13];
            int nl = 0;
            float lowNone = 0.0f;
            ov::Curves curLow;
            for (int i = 0; i < lk.count; ++i)
            {
                const int key = lk.key[i];
                const int p = key >= 0 && key < kPoseKeys ? m_poseOfKey[key] : -1;
                if (p < 0)
                {
                    lowNone += lk.w[i];
                    continue;
                }
                m_ov->FramePose(m_ov->PoseAt(p).firstFrame, m_lowSlots[nl]);
                ov::Curves c;
                m_ov->FrameCurves(p, m_ov->PoseAt(p).firstFrame, c);
                curLow.Accumulate(c, lk.w[i]);
                lop[nl] = &m_lowSlots[nl];
                low[nl++] = lk.w[i];
            }
            if (lowNone > 0.0f)
            {
                m_lowSlots[nl] = *m_L;
                lop[nl] = &m_lowSlots[nl];
                low[nl++] = lowNone;
            }
            if (nl > 0)
                ov::BlendPosesWeighted(lop, low, nl, *m_OPLower);
            else
                *m_OPLower = *m_OP;
            cur.v[ov::kLegs] = curLow.v[ov::kLegs];
            cur.v[ov::kPelvis] = curLow.v[ov::kPelvis];
#if defined(UTPP_LOCOSTANCE_MUTATION) && UTPP_LOCOSTANCE_MUTATION == 2
            // mutation 2 (CR-4): the spine / head regions take one inner on OPLower with the B-synced curves (the cross term)
            cur.v[ov::kSpine] = curLow.v[ov::kSpine];
            cur.v[ov::kSpineAdd] = curLow.v[ov::kSpineAdd];
            cur.v[ov::kHead] = curLow.v[ov::kHead];
            cur.v[ov::kHeadAdd] = curLow.v[ov::kHeadAdd];
#endif
            cur.present |= curLow.present;
            BuildTorsoSides();
        }
        // The stance/movement booleans are the new targets, not the current
        // blended pose. Use the base weights as well as the style weights:
        // selecting dP from the target immediately snapped the whole style
        // layer while B and OP were still at their previous stance.
        const ov::Additive* ds[kStyleSlots];
        float dw[kStyleSlots];
        int nd = 0;
        for (int i = 0; i < m.styles.count; ++i)
        {
            const int s = m.styles.key[i];
            for (int j = 0; j < m.bases.count; ++j)
            {
                const float weight = m.styles.w[i] * m.bases.w[j];
                if (!(weight > 0.0f)) continue;
                const int key = m.bases.key[j] & 3;
                const int stance = (key & 2) ? 2 : (key & 1);
                const int d = s > 0 && s < 3 ? m_styleDelta[s][stance] : -1;
                if (d < 0)
                    m_dPs[nd].SetIdentity(m_ov->Skel().n, true);
                else
                    m_ov->DeltaFrame(d, 0, true, m_dPs[nd]);
                ds[nd] = &m_dPs[nd];
                dw[nd++] = weight;
            }
        }
        m_haveDP = nd > 0;
        if (m_haveDP)
        {
            ov::BlendAdditives(ds, dw, nd, *m_dP);
            m_dP->meshSpace = true;
        }
        return cur;
    }

    // Model-space height (m) of one bone of a local overlay pose: FK along the parents of the overlay skeleton.
    float OvBoneZ(const ov::Pose& aPose, int aBone) const
    {
        const ov::Skeleton& sk = m_ov->Skel();
        int chain[ov::kMaxBones];
        int n = 0;
        for (int b = aBone; b >= 0 && n < ov::kMaxBones; b = sk.parent[b])
            chain[n++] = b;
        math::Quat q{0, 0, 0, 1};
        math::Vec3 p{0, 0, 0};
        for (int i = n; i-- > 0;)
        {
            p = p + math::QuatRotate(q, aPose.b[chain[i]].t);
            q = math::QuatNormalize(math::QuatMul(q, aPose.b[chain[i]].r));
        }
        return p.z;
    }

    // OVLEGS (loco/ovlegs, recording 2026-10-31 22:55, crouch -> stand with a drawn Pistol2H). Composing the legs per stance side (ComposeLayers,
    // lowerSides) removes the cross term, but each side is still OP_side + D_MS(B_side -> L) and L is not near B_side while the stance fades: the
    // stand side evaluated on the crouched L carries the stand overlay's bone-local leg offset onto a thigh that is pitched forward, where it is
    // a vertical offset instead of a horizontal one (the stand side alone put the left ankle 7.6 cm below L at 69.46, weighted 1 - s = 0.26 and
    // c_stand 1: 2.0 of the 2.5 cm of dip that remained). The composed ankles then stood 2.5 - 3.0 cm below the base pose, which keeps its feet
    // on the layers (BlendStack::KeepFootHeight), for the 1 s of the fade. The overlay's own share of the ankle height is known to first order:
    // the settled offset of each side, FK(OP_side) - FK(B_side), weighted by that side's leg layer weight and the crouch share. This lifts the
    // composed ankle to base + that expectation, never lowers it (the same two-bone, rotation-only lift as BlendStack::KeepFootHeight, through
    // motionFeet::Warp), only while the stance sides are set (0 < s < 1: outside a fade ComposeLayers and the pose are untouched, bit for bit) and
    // only while the leg layer is on (idle pistol / air factors in the side weights). The lift ramps in and out over the first and the last
    // 12 % of the fade (kOvLegsRamp, from either end: the crouch share runs 1 -> 0 into a stand and 0 -> 1 into a crouch) so it is 0 where the
    // sides switch on and off; the first-order offset is not exact (~0.6 cm off the stand side's real settled offset), and a crouch overlay
    // pose with a leg layer would otherwise step it in or out with the sides.
    static constexpr float kOvLegsRamp = 0.12f;
    void KeepLowerFootHeight(const ov::LayerInputs& aLi, FrameInfo& aInfo)
    {
        aInfo.lowerLift[0] = aInfo.lowerLift[1] = 0.0f;
#if defined(UTPP_OVLEGS_MUTATION) && UTPP_OVLEGS_MUTATION >= 1
        (void)aLi; // mutations 1, 2: no foot lift
        return;
#else
        if (!m_lowerReady || !m_haveOP || !m_torsoOn || !aLi.OP || !aLi.B || !aLi.torso[0].OP || !aLi.torso[1].OP)
            return;
        const float s = ov::Clamp01(aLi.torsoCrouch);
        const float c0 = aLi.torso[0].legs, c1 = aLi.torso[1].legs;
        const float gate = ov::Clamp01(std::fmax(c0, c1)) * ov::Clamp01(std::fmin(s, 1.0f - s) / kOvLegsRamp);
        if (!(gate > 0.0f))
            return;
        motionFeet::Targets targets{};
        targets.valid = true;
        targets.relative = true;
        targets.warpWeight = 1.0f;
        bool any = false;
        for (int side = 0; side < 2; ++side)
        {
            const int foot = m_dbToOv[m_legDb[side][2]];
            const float d0 = OvBoneZ(*aLi.torso[0].OP, foot) - OvBoneZ(*aLi.torso[0].B, foot);
            const float d1 = OvBoneZ(*aLi.torso[1].OP, foot) - OvBoneZ(*aLi.torso[1].B, foot);
            const float target = OvBoneZ(*aLi.L, foot) + (1.0f - s) * c0 * d0 + s * c1 * d1;
            const float dz = gate * std::fmax(0.0f, target - OvBoneZ(*m_lay, foot));
            if (!std::isfinite(dz))
                return;
            aInfo.lowerLift[side] = dz;
            targets.feet[side].p = {0.0f, 0.0f, dz};
            any = any || dz > 1.0e-6f;
        }
        if (!any)
            return;
        FromOv(*m_lay, m_lowerPose.data());
        const auto result = m_lowerWarp.Apply(m_lowerPose, targets);
        if (!result.applied)
            return; // a failing solve leaves the composed pose untouched
        for (int side = 0; side < 2; ++side) // only the rotations of the three leg bones: the solve changes nothing else
            for (int k = 0; k < 3; ++k)
                m_lay->b[m_dbToOv[m_legDb[side][k]]].r = m_lowerPose[static_cast<size_t>(m_legDb[side][k])].q;
#endif
    }

    // CR-4 (loco/stance, ov::ComposeLayers): during a stance fade of B (crouch share 0 < s < 1) the spine / head regions are composed per
    // stance side - the stance-free groups of the pose crossfade on their stand (key) or crouch (key | 2) pose with that side's curves, and
    // the base pose of that side (the `bases` entries of the stance, renormalised). A group without an overlay pose (Default / Game keys:
    // holster / draw inside the fade) is L in that side's pose with no curve weight, as in OP / OPLower above, so the sides stay on for the
    // whole fade: dropping to the crossfade path for such a frame stepped the torso 2.2 deg in one frame (mutation 3). Off (m_torsoOn
    // false: spine / head take OP and the crossfade curves, bit for bit the previous path) outside a fade. Reuses m_lowSlots (free once
    // OPLower is built).
    void BuildTorsoSides()
    {
        m_torsoOn = false;
        ov::OverlayMachine::StanceGroups g;
        m_machine.StanceGroupsOf(g);
        m_torsoCrouch = g.crouch;
        if (!(g.crouch > 0.0f && g.crouch < 1.0f) || g.count == 0)
            return;
        for (int k = 0; k < 2; ++k)
        {
            const ov::Pose* op[6];
            float ow[6];
            int no = 0;
            ov::Curves c;
            for (int j = 0; j < g.count; ++j)
            {
                if (!(g.w[j] > 0.0f))
                    continue;
                const int key = k ? (g.key[j] | 2) : g.key[j];
                const int p = key >= 0 && key < kPoseKeys ? m_poseOfKey[key] : -1;
                if (p < 0)
                {
#if defined(UTPP_LOCOSTANCE_MUTATION) && UTPP_LOCOSTANCE_MUTATION == 3
                    return; // mutation 3: a keyless group drops the spine / head to the crossfade path mid-fade
#else
                    op[no] = m_L.get(); // the layering of L over itself is L; no curve weight
                    ow[no++] = g.w[j];
                    continue;
#endif
                }
                m_ov->FramePose(m_ov->PoseAt(p).firstFrame, m_lowSlots[k * 6 + no]);
                ov::Curves ck;
                m_ov->FrameCurves(p, m_ov->PoseAt(p).firstFrame, ck);
                c.Accumulate(ck, g.w[j]);
                op[no] = &m_lowSlots[k * 6 + no];
                ow[no++] = g.w[j];
            }
            if (no == 0)
                return;
            ov::BlendPosesWeighted(op, ow, no, m_torsoPoses[k]);
            float sum = 0.0f;
            for (int j = 0; j < no; ++j)
                sum += ow[j];
            const float inv = sum > 0.0f ? 1.0f / sum : 0.0f;
            m_torsoCurves[k][0] = c.v[ov::kSpine] * inv;
            m_torsoCurves[k][1] = c.v[ov::kSpineAdd] * inv;
            m_torsoCurves[k][2] = c.v[ov::kHead] * inv;
            m_torsoCurves[k][3] = c.v[ov::kHeadAdd] * inv;
            m_torsoCurves[k][4] = c.v[ov::kLegs] * inv;
            m_torsoCurves[k][5] = c.v[ov::kPelvis] * inv;
            const ov::Pose* bp[4];
            float bw[4];
            int nb = 0;
            for (int i = 0; i < m_machine.bases.count; ++i)
                if (((m_machine.bases.key[i] & 2) != 0) == (k == 1) && m_machine.bases.w[i] > 0.0f)
                {
                    bp[nb] = &m_basePoses[m_machine.bases.key[i] & 3];
                    bw[nb++] = m_machine.bases.w[i];
                }
            if (nb == 0)
                return;
            ov::BlendPosesWeighted(bp, bw, nb, m_torsoPoses[2 + k]);
        }
        m_torsoOn = true;
    }

    const motion::MotionDatabase* m_db = nullptr;
    const ov::OverlayFile* m_ov = nullptr;
    uint32_t m_n = 0;
    std::vector<int> m_dbToOv;
    motionFeet::Warp m_lowerWarp; // OVLEGS: KeepLowerFootHeight
    bool m_lowerReady = false;
    std::vector<BoneTransform> m_lowerPose;
    int m_legDb[2][3]{{-1, -1, -1}, {-1, -1, -1}}; // database bone of the thigh / shin / ankle per side
    std::array<int, kPoseDiagnosticBoneNames.size()> m_poseDiagBones{};
    std::vector<BoneTransform> m_raw, m_prev, m_base;
    QuinticInertializer m_inertial;
    motionstack::BlendStack m_stack;
    BaseBlendPolicy m_basePolicy = BaseBlendPolicy::Quintic;
    bool m_externalBase = false, m_usesLiveBase = false;
    bool m_ignoreDeficit = false;
    std::unique_ptr<ov::Pose> m_L, m_B, m_OP, m_OPLower, m_lay, m_tmp, m_shootFallback;
    int m_shootClavL = -1, m_shootClavR = -1;
    femshoulder::ReturnHold m_femHold; // RETURN HOLD state of the feminine pass (FeminineShoulders.hpp)
    float m_reachShift = 0.0f;          // rifle-grip-stock: rate-limited reach shift of the weapon (m, SupportGrip.hpp)
    float m_gateSeconds = 0.2f;         // rifle-grip-stock: gate / reach-shift time of this frame (sgrip::GateSeconds)
    float m_routeW = 0.0f;              // rifle-grip-stock: rate-limited route weight (gate x Rifle pose weight)
    bool m_routeWHave = false;
    bool m_reachShiftHave = false;
    sgrip::Gate m_supportGate;          // rifle-grip-stock: route gate of the game grip / stock pass (SupportGrip.hpp)
    sgrip::Release m_release;           // rifle-grip-stock: release state while the gate falls under other weapon data (review round 5)
    sgrip::FingerBones m_supportFingers{};
    int m_pocketBone = -1;              // sgrip::kPocketBone (RightArm)
#ifdef UTPP_FEM_DEFAULT_LEGACY // offline identity proof only: the pre-2026-11-02 pass (build with the 0.995 / 0.999 reach macros too)
    float m_femAmount = 0.020f, m_femRiseSeconds = kFeminineRiseSeconds, m_femPrev[2] = {};
    bool m_femContinuous = false, m_femHave = false;
#else
    float m_femAmount = kFeminineShoulderAmount, m_femRiseSeconds = kFeminineRiseSeconds, m_femPrev[2] = {};
    bool m_femContinuous = true, m_femHave = false;
#endif
    std::unique_ptr<ov::Pose[]> m_basePoses;
    std::unique_ptr<ov::Pose[]> m_opSlots; // 6 machine keys + L for the keyless weight
    std::unique_ptr<ov::Pose[]> m_lowSlots; // 12 stance-split keys (OverlayMachine::LowerBodyKeys) + L
    // CR-4 (BuildTorsoSides): overlay pose of the stand / crouch side (0, 1), base pose of the stand / crouch side (2, 3); curves per side
    // (spine, spine add, head, head add, legs, pelvis); m_torsoOn = the sides are valid this frame
    std::unique_ptr<ov::Pose[]> m_torsoPoses;
    float m_torsoCurves[2][6]{};
    float m_torsoCrouch = 0.0f;
    bool m_torsoOn = false;
    static constexpr int kPoseKeys = static_cast<int>(ov::PoseOverlay::Count) * 12;
    int m_poseOfKey[kPoseKeys]{};
    int m_styleDelta[3][3]{};
    static constexpr int kStyleSlots = 3 * 4; // style x blended base key
    std::unique_ptr<ov::Additive> m_dP;
    std::unique_ptr<ov::Additive[]> m_dPs;
    std::unique_ptr<ov::LayerScratch> m_scratch;
    int m_spine3 = -1;
    ov::OverlayMachine m_machine;
    ov::StyleWeightTable m_styleTable;
    idlebreak::Controller m_idle;
    gik::Rig m_rig;
    ov::ForearmTwistRig m_forearmTwist;
    gik::Params m_gripParams;
    bool m_haveOP = false, m_haveDP = false;
    math::Vec3 m_meshPos[ov::kMaxBones];
    math::Quat m_meshRot[ov::kMaxBones];
#if defined(UTPP_POSEV2_ARM_MUTATION) && UTPP_POSEV2_ARM_MUTATION == 1
    float m_freeArmScale = 1.0f; // mutation 1: full locomotion swing on the free arm
#else
    float m_freeArmScale = kFreeArmAddScale;
#endif
    static constexpr int kOverlays = static_cast<int>(ov::PoseOverlay::Count);
    int m_sweep[kOverlays][2]{};
    AimSpineYawFilter m_aoFilter; // fix batch 2 (AO): spine yaw continuity + latch of the last aim angles
    BarrelPitchLoop m_barrel;     // aim pitch plan 2026-10-30 B: closed barrel pitch loop (bias per slot + residual)
    static constexpr int kBarrelSlotMax = 6 * 3; // machine pose keys x styles
    int m_barrelSlots[kBarrelSlotMax]{};
    float m_barrelW[kBarrelSlotMax]{};
    int m_barrelSlotCount = 0;
    float m_barrelCmd = 0.0f;
    int m_spine2 = -1;
    std::unique_ptr<ov::Pose> m_aoPre;
    std::unique_ptr<ov::Additive> m_aoAdd, m_aoA, m_aoB;
    std::unique_ptr<ov::Additive[]> m_aoSlots;
    ov::BoneWeights m_clavL, m_clavR;
    std::array<float, adapter::Budget::kWindow> m_times{};
    std::array<float, adapter::Budget::kWindow> m_sorted{};
    uint64_t m_timeCount = 0;
    bool m_havePrev = false;
    bool m_ready = false;
};
} // namespace utpp::mm::motionpose

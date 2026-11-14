#pragma once
// Same-call adapter. Caller publishes Prepared on the cold arming/configuration
// path and reads ONE immutable copy before Capture/Finish. No cached game poses.
#include "NativeReloadClavicleReach.hpp"
#include <atomic>
#include "GameIntegration.hpp"
#include "Overlay.hpp"

namespace utpp::mm::reload_contacts_runtime
{
// Finish is reached only for the explicit TPP pistol-reload capture policy.
// Keep the left native guide unchanged; cap only the right native guide
// turn that caused the measured P2H kink. Other consumers default to pi.
inline constexpr float kPistolReloadRightGuideTurnCapRad=math::kPi*0.25f;
// Engine animation blends can publish near-unit locals. Normalize this adapter's
// private copy consistently; strict solver inputs and all other consumers stay unchanged.
inline constexpr float kMaxNativeQuatNormSquaredError=0.01f;
// Verified authored reload clips: lowest hand-to-shoulder distance (fraction of the arm length) the contact solve
// may reach before the GripIK fold clamp rejects the frame. Recorded native poses reach 0.13 (elbow ~15deg); .12 keeps
// a margin without admitting a tighter fold than any native pose; 0 = solver default .20.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==3
inline constexpr float kAuthoredContactMinReach=0.0f; // mutation 3: default fold clamp
#else
inline constexpr float kAuthoredContactMinReach=0.12f;
#endif
// Native pose divergence guard. The contact targets are native MODEL positions, correct only while the native body
// stands where the MM body stands. After a jump / slide / crouch the native graph can leave the player squatting
// (hips ~.34m against MM ~.95m, 21.10 01:56): the targets then put the hands at the MM hips. From
// kTorsoTransportStartM of pelvis distance the targets are carried in the Spine3 frame onto the MM torso (lambda is
// binary: a partial transport leaves a residual offset d*(1-lambda) that crosses the arm reach and flips between
// Reject and accept). The correction WEIGHT is what moves continuously: it fades to 0 over
// [kTorsoFadeStartM, kTorsoTransportStartM] (healthy recordings stay below .28m, 03:37 crouch-stop reaches .39m) and
// returns to 1 over [kTorsoTransportStartM, kTorsoTransportFullM], so the output is the authored baseline at the switch.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && (UTPP_RELOAD_CONTACT_MUTATION==4 || UTPP_RELOAD_CONTACT_MUTATION==6)
inline constexpr float kTorsoFadeStartM=1.0e9f; // mutation 4 / 6: no divergence handling at all
inline constexpr float kTorsoReleaseM=0.0f;
inline constexpr float kTorsoTransportStartM=2.0e9f;
inline constexpr float kTorsoTransportFullM=3.0e9f;
#else
inline constexpr float kTorsoFadeStartM=0.36f;
inline constexpr float kTorsoReleaseM=0.30f; // transport is kept until the pelvis distance is back below this (healthy < .28)
inline constexpr float kTorsoTransportStartM=0.42f;
inline constexpr float kTorsoTransportFullM=0.60f;
#endif
// Mutation 19: the correction weight is not faded around the anchor <-> transport switch (the transport engages with the correction at full weight).
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==19
inline constexpr bool kTorsoGateEnabled=false;
#else
inline constexpr bool kTorsoGateEnabled=true;
#endif
// Rifle reloads (overlay Rifle, no authored clip). Offline go/no-go on recording 2026-10-25 (rifle-gonogo, the production
// Capture / Finish path replayed on the recorded rifle frames): every eligible stationary frame accepted, contact error 0. The
// native rifle choreography is carried in the Spine3 frame: anchored in model space the native hand targets lag a turning MM
// torso (left hand 19.5cm p50 / 56.8cm max off the native chest-relative position while turning, 30.9cm p50 in a run), in
// the chest frame they stay on it (0.0 p50). Compile-time switch, like kReloadCorrectionMoving: there is no runtime setting
// of the reload contact correction. Mutation 1 (UTPP_RELOAD_RIFLE_MUTATION) turns the rifle path off (pistols only),
// 2 keeps the rifle on the pistol placement (no forced chest-frame transport), 3 keeps the stationary-only rule for rifles
// (ReloadContactFade.hpp kReloadContactsRifleAnyStance: rifles are corrected in every ground stance and gait).
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==1
inline constexpr bool kReloadContactsRifle=false;
#else
inline constexpr bool kReloadContactsRifle=true;
#endif
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==2
inline constexpr bool kRifleChestTransport=false;
#else
inline constexpr bool kRifleChestTransport=true;
#endif
// Overlays (ov::PoseOverlay values, PoseSnapshot::traceOverlay / the driver's g_pose) whose reloads get the contact correction.
inline constexpr bool ReloadContactsOverlay(int32_t overlay)
{
    return overlay==static_cast<int32_t>(ov::PoseOverlay::Pistol1H) || overlay==static_cast<int32_t>(ov::PoseOverlay::Pistol2H) ||
           (kReloadContactsRifle && overlay==static_cast<int32_t>(ov::PoseOverlay::Rifle));
}
// Pose hook that writes the player's pose this session (EarlyPoseRuntime Owner): the FK hook (default) or the early mapping job.
enum class HookOwner : uint8_t { FK, Mapping };
// Overlays whose game-clip reloads get the contact correction under the default FK owner. A family is listed only when its offline
// replay keeps the reload under the anatomy limits on both bodies; every other overlay keeps the game's reload arms. Build switch
// UTPP_RELOAD_FK_OWNER_RIFLE=1 lists the Rifle overlay. Off by default: on recorded rifle reloads the correction adds single-frame
// arm steps at some entries and exits (one cause: a shot right after the reload leaves the reload domain and the correction fade
// resets to zero within one frame), and the male body has not been measured.
#if defined(UTPP_RELOAD_FK_OWNER_RIFLE) && UTPP_RELOAD_FK_OWNER_RIFLE==1
inline constexpr bool kFkOwnerRifle=true;
#else
inline constexpr bool kFkOwnerRifle=false;
#endif
inline constexpr bool kFkOwnerPistol=false;
inline constexpr bool FkOwnerOverlay(int32_t overlay)
{
    return (kFkOwnerRifle && kReloadContactsRifle && overlay==static_cast<int32_t>(ov::PoseOverlay::Rifle)) ||
           (kFkOwnerPistol && (overlay==static_cast<int32_t>(ov::PoseOverlay::Pistol1H) ||
                               overlay==static_cast<int32_t>(ov::PoseOverlay::Pistol2H)));
}
// Admission of the contact solve for one hook call (FkHook ApplyPoseSnapshot). The mapping owner keeps its rule (every reload overlay,
// authored clips included); the FK owner corrects only the listed overlays and never a purpose-built (authored) reload.
inline bool Admit(const integ::PoseSnapshot& s,HookOwner owner)
{
    if(!s.reloadContactsEligible || !ReloadContactsOverlay(s.traceOverlay))return false;
    if(owner==HookOwner::Mapping)return true;
    return !s.reloadAuthoredClip && FkOwnerOverlay(s.traceOverlay);
}
// Rifle out without a correction this frame: the published weapon / left hand is remembered for the start of the next episode
// (ObservePublished) by the owners that correct rifle reloads.
inline bool ObservesRifle(int32_t overlay,HookOwner owner)
{
    return kReloadContactsRifle && overlay==static_cast<int32_t>(ov::PoseOverlay::Rifle) &&
           (owner==HookOwner::Mapping || FkOwnerOverlay(overlay));
}
struct Prepared
{
    uint64_t rig=0;
    ov::Skeleton skeleton{};
    gik::Rig arms{};
    int32_t gameIndex[ov::kMaxBones]{};
    // Explicit cold proof only: unmapped synthetic DB roots have identity local
    // transforms and no missing game ancestor. Never infer this from bone count.
    uint32_t identityRoots[(ov::kMaxBones+31)/32]{};
    // Bone index of "Head" (-1 = unknown: the placement guard below is skipped). Set by the cold arming path.
    int head=-1;
    // Bone index of "Hips": the top bone this adapter may move (Root / Trajectory above it are never written by
    // BlendIntoRecords). -1 = derive it from the parent chain (TopBoneOf).
    int hips=-1;
};
inline integ::SeqlockBuffer<Prepared> g_config;
struct Cycle
{
    ov::Pose native{};
    const uint8_t* records=nullptr;
    uint32_t count=0;
    uint64_t sequence=0,rig=0;
    bool valid=false;
};
inline bool IdentityRoot(const Prepared& p,int i)
{ return ((p.identityRoots[i>>5]>>(i&31))&1u)!=0; }
// Plausibility guard on the PLACEMENT the contact solve is about to reproduce (model space is Z up). The native FPP
// arms are bound to the camera, so a diverged native torso can still ask for a weapon inside the head or a hand below
// the pelvis of the MM body (21.10 01:56 k2/k4/k8/k9/k10: weapon 1.4-9cm from the head, hands 7-12cm under the hips).
// Healthy recordings (19.10 11:49, 20.10 10:14, 21.10 03:37 stationary) stay above 13cm / 2.8cm, i.e. inside f == 1.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && (UTPP_RELOAD_CONTACT_MUTATION==5 || UTPP_RELOAD_CONTACT_MUTATION==6)
inline constexpr bool kPlacementGuardEnabled=false; // mutation 5 / 6: guard off
#else
inline constexpr bool kPlacementGuardEnabled=true;
#endif
inline constexpr float kWeaponHeadMinM=0.06f,kWeaponHeadFullM=0.12f;
// Left hand under the pelvis of the MM body. The guard stops a diverged native from putting the hand at the MM thighs / knees. Since the targets are carried onto
// the MM shoulders (anchor) or into the Spine3 frame (transport) a hand under the hips is no longer a sign of a diverged native but the plain geometry of a
// MM torso that leans further forward than the native one: R19 k3/k4 (MM torso 28-30deg against the native 11deg) bring the native belt hand up to 10.6cm under
// the MM hips for ~.2s, and the magazine rides in the frame of the WEAPON (animgraph, no write path in the runtime), so the hand can neither be lifted nor
// the weapon moved without tearing the hand off the magazine (a lifted arm: up to 13cm; a lifted target: pistol 14cm above its native height). The guard
// therefore starts at 14cm under the hips (full weight above -.14m, zero at -.20m). Mutation 18 restores the old +-2cm band: the correction is cut there.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==18
inline constexpr float kHandOverHipsMinM=-0.02f,kHandOverHipsFullM=0.02f;
#else
inline constexpr float kHandOverHipsMinM=-0.20f,kHandOverHipsFullM=-0.14f;
#endif
inline float Smooth01(float x){x=std::clamp(x,0.0f,1.0f);return x*x*(3.0f-2.0f*x);}
// Top bone of the pelvis chain. The production map (RemapLocked -> BuildBoneMap(..., 0)) maps Root and Trajectory too,
// so a climb that only stops at an unmapped bone would end at Root; stop below the skeleton root instead.
inline int TopBoneOf(const Prepared& p,int bone)
{
    if(p.hips>=0 && p.hips<p.skeleton.n)return p.hips;
    for(int step=0;step<p.skeleton.n;++step)
    {
        const int par=p.skeleton.parent[bone];
        if(par<0 || p.gameIndex[par]<0 || p.skeleton.parent[par]<0)return bone;
        bone=par;
    }
    return -1;
}
inline float PlacementWeight(const Prepared& p,const ov::Pose& authored,const ov::Pose& target)
{
    if constexpr(kPlacementGuardEnabled)
    {
        if(p.head<0 || p.head>=p.skeleton.n)return 1.0f;
        const auto& sk=p.skeleton;
        const int shoulder=sk.parent[p.arms.left.upper];
        const int spine3=shoulder>=0 ? sk.parent[shoulder] : -1;
        const int hips=spine3>=0 ? TopBoneOf(p,spine3) : -1;
        if(hips<0)return 1.0f;
        const float weaponHead=gik::Len(native_reload::Model(sk,target,p.arms.weaponRight).t-native_reload::Model(sk,authored,p.head).t);
        const float handOverHips=native_reload::Model(sk,target,p.arms.left.hand).t.z-native_reload::Model(sk,authored,hips).t.z;
        if(!std::isfinite(weaponHead) || !std::isfinite(handOverHips))return 1.0f;
        return std::min(Smooth01((weaponHead-kWeaponHeadMinM)/(kWeaponHeadFullM-kWeaponHeadMinM)),
                        Smooth01((handOverHips-kHandOverHipsMinM)/(kHandOverHipsFullM-kHandOverHipsMinM)));
    }
    return 1.0f;
}
inline float TorsoTransportLambda(float pelvisDistance)
{
    return (std::isfinite(pelvisDistance) && pelvisDistance>=kTorsoTransportStartM) ? 1.0f : 0.0f;
}
// Continuous correction gate around the transport switch: 1 -> 0 on the way in, 0 -> 1 on the way out.
inline float TorsoDivergenceGate(float pelvisDistance)
{
    if(!std::isfinite(pelvisDistance))return 1.0f;
    const float gate=pelvisDistance<kTorsoTransportStartM
        ? 1.0f-Smooth01((pelvisDistance-kTorsoFadeStartM)/(kTorsoTransportStartM-kTorsoFadeStartM))
        : Smooth01((pelvisDistance-kTorsoTransportStartM)/(kTorsoTransportFullM-kTorsoTransportStartM));
    return kTorsoGateEnabled ? gate : 1.0f;
}
inline ov::Xf InverseXf(const ov::Xf& a)
{
    const auto inv=math::QuatInv(a.r);
    ov::Xf out=a;out.r=inv;out.t=math::QuatRotate(inv,a.t*-1.0f);return out;
}
// Carry the native pose onto the MM torso: translate/rotate the top-level bone so that the native Spine3 frame moves
// by `lambda` of the way onto the authored (MM) Spine3 frame. Returns false when the skeleton has no such chain.
inline bool TransportNativeTorso(const Prepared& p,const ov::Pose& authored,ov::Pose& target,float lambda)
{
    const auto& sk=p.skeleton;
    const int shoulder=sk.parent[p.arms.left.upper];
    if(shoulder<0)return false;
    const int spine3=sk.parent[shoulder];
    if(spine3<0)return false;
    const int top=TopBoneOf(p,spine3);
    if(top<0 || top==spine3)return false;
    const auto natS3=native_reload::Model(sk,target,spine3);
    const auto mmS3=native_reload::Model(sk,authored,spine3);
    const auto wanted=native_reload::Blend(natS3,mmS3,lambda);
    const auto moved=native_reload::Compose(native_reload::Compose(wanted,InverseXf(natS3)),native_reload::Model(sk,target,top));
    const ov::Xf parentModel=sk.parent[top]>=0 ? native_reload::Model(sk,target,sk.parent[top]) : ov::Xf{};
    const auto local=native_reload::Relative(parentModel,moved);
    if(!math::IsFinite(local.t) || !math::IsFinite(local.r))return false;
    target.b[top].t=local.t;target.b[top].r=local.r;
    return true;
}
// Shoulder anchor of the native contact targets. The native hand / weapon targets are authored for the NATIVE body (first person
// choreography on a standing, upright V): applied as absolute model positions they keep their height and distance while the MM body
// stands lower and leans further forward (recorded 21.10 12:15, healthy standing reloads: MM shoulders 7.6cm lower and 3cm back,
// hips 6.3cm lower and 8.7cm back). The weapon then sat 8-27cm higher above the MM head than in the native pose (P3: "pistol over
// the head") and the shoulder-to-hand distance of the arms changed, which folded the left elbow ~20deg tighter (N6: 159-163deg
// against 139deg in the native pose). The whole target is therefore carried rigidly by the offset between the native and the MM
// shoulder mid points (the arm roots): every shoulder-to-hand vector, hand-to-weapon binding and finger pose of the native
// reload stays as authored, only the body they belong to changes. The weight multiplies the offset; 0 reproduces the absolute
// placement of fix batch 1 (mutation 17). The correction weight is applied before (the target is a blend of the baseline and
// the native pose), so the offset is zero when the correction is off and the output stays continuous.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==17
inline constexpr float kReloadContactShoulderAnchor=0.0f; // mutation 17: absolute native placement
#else
inline constexpr float kReloadContactShoulderAnchor=1.0f;
#endif
// Translation applied to the top bone of the pelvis chain so that the native target moves rigidly by `offset` (model space).
inline bool ShiftNativeTarget(const Prepared& p,ov::Pose& target,int top,const math::Vec3& offset)
{
    const auto& sk=p.skeleton;
    auto moved=native_reload::Model(sk,target,top);
    moved.t=moved.t+offset;
    const ov::Xf parentModel=sk.parent[top]>=0 ? native_reload::Model(sk,target,sk.parent[top]) : ov::Xf{};
    const auto local=native_reload::Relative(parentModel,moved);
    if(!math::IsFinite(local.t))return false;
    target.b[top].t=local.t;
    return true;
}
// Carry `target` (the blended native pose) onto the MM shoulders. `roots` is the MM pose the solver will work on, i.e. with the masked
// shoulder / arm channels already restored (their sockets differ from the records: 15.45cm against 17.4cm in the authored clips).
// The shift is rigid: the hand-to-weapon relations of the native reload (the new magazine is carried in the frame of the weapon and
// cannot be moved by the runtime) are never changed, whatever the height of the left hand over the MM hips (see kHandOverHips*).
// Returns false when nothing was changed (anchor off, or a skeleton without the Spine3 / pelvis chain: the absolute native placement
// stays, exactly as before).
inline bool AnchorNativeTarget(const Prepared& p,const ov::Pose& roots,ov::Pose& target)
{
    if constexpr(kReloadContactShoulderAnchor>0.0f)
    {
        const auto& sk=p.skeleton;
        const int shoulder=sk.parent[p.arms.left.upper];
        if(shoulder<0)return false;
        const int spine3=sk.parent[shoulder];
        if(spine3<0)return false;
        const int top=TopBoneOf(p,spine3);
        if(top<0 || top==spine3)return false;
        const auto mid=[&](const ov::Pose& q){
            return (native_reload::Model(sk,q,p.arms.left.upper).t+native_reload::Model(sk,q,p.arms.right.upper).t)*0.5f;};
        const math::Vec3 offset=(mid(roots)-mid(target))*kReloadContactShoulderAnchor;
        if(!math::IsFinite(offset))return false;
        return ShiftNativeTarget(p,target,top,offset);
    }
    else return false;
}
// Stock clearance of the carried rifle. The rifle path keeps the native weapon placement relative to the native chest (Spine3
// frame), but the native placement is FPP choreography: the stock sinks into the body there too, and the MM chest / shoulder geometry
// differs (MM shoulder sockets ~6cm further forward). Recorded 2026-10-28 01:22:53 (Sidewinder Items.mq008_golden_knuckledusters): the
// stock passes through the top / back of the right shoulder while the magazine is changed. Offline (recording 2026-10-26 11:28, 41
// Sidewinder reloads, final pose skinned on the female body mesh): the stock edges sit 3.4cm (p50) / 5.6cm (p05) inside the body
// during the reload, against +1.0cm (p50) outside in the MM rifle poses without a reload. The clearance carries the whole native
// target (both hands, both weapon bindings, so the magazine stays in the left hand) rigidly out of the body capsules below, bounded,
// reach-aware and weighted like the correction. Mutation 5 (UTPP_RELOAD_RIFLE_MUTATION) turns it off.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==5
inline constexpr bool kRifleStockClearance=false; // mutation 5: the stock may sink into the MM body
#else
inline constexpr bool kRifleStockClearance=true;
#endif
// Stock model in the WeaponRight frame (weapon origin = grip; mesh +Y forward, +Z up, confirmed on the recorded hands: left hand
// +0.22m along +Y). The Sidewinder stock mesh spans 0.004..0.324m behind the origin, -0.071..+0.056m vertical, +-0.018m lateral (CR2W
// boundingBox of w_rifle_assault__nokota_copperhead__rcv1_01_stock_01); its top and bottom edge lines are sampled from kRifleStockNearM
// to kRifleStockFarM. One stock model serves every weapon of the Rifle overlay (the runtime has no mesh data).
inline constexpr math::Vec3 kRifleStockBack{0.0f,-1.0f,0.0f}; // weapon backward axis in the WeaponRight frame
inline constexpr math::Vec3 kRifleStockUp{0.0f,0.0f,1.0f};    // weapon up axis in the WeaponRight frame
inline constexpr float kRifleStockNearM=0.10f,kRifleStockFarM=0.32f;
inline constexpr float kRifleStockEdgeM[2]={0.035f,-0.050f}; // top / bottom edge lines (the stock is ~0.02m thicker than the lines)
inline constexpr int kRifleStockSamplesPerEdge=5,kRifleStockSamples=2*kRifleStockSamplesPerEdge;
// MM body model, built from the pose the solver works on (chest frame: origin neck base, x right, y forward, z up):
//  - torso (chest, breasts, clavicle region): a height field along the chest forward axis - back and front surface of the female
//    body mesh t0_000_pwa_base__full (rigid-ancestor skinning with 203 recorded MM reload poses, median; torso and clavicle faces) on a
//    3cm grid. A point is inside when back < y < front; the only way out is forward (the body continues sideways).
//    -1 = no body in that column.
//  - capsules: 0 right trapezius / shoulder top (neck base -> towards the right arm socket), 1 right upper arm (below the socket ->
//    elbow), 2 neck (neck base -> head).
inline constexpr int kTorsoRows=15,kTorsoCols=15;
inline constexpr float kTorsoX0=-0.21f,kTorsoDX=0.03f,kTorsoZ0=0.06f,kTorsoDZ=-0.03f;
inline constexpr float kTorsoFrontM[kTorsoRows][kTorsoCols]={
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.072f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,0.077f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.081f,0.073f,0.075f,-1.0f,-1.0f,-1.0f,0.075f,0.073f,0.084f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.084f,0.079f,0.085f,0.089f,0.088f,0.089f,0.085f,0.080f,0.086f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.088f,0.098f,0.104f,0.107f,0.108f,0.107f,0.104f,0.098f,0.088f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.103f,0.124f,0.132f,0.128f,0.124f,0.128f,0.132f,0.124f,0.103f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.121f,0.153f,0.156f,0.148f,0.135f,0.148f,0.156f,0.153f,0.120f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.122f,0.156f,0.162f,0.153f,0.140f,0.153f,0.162f,0.156f,0.121f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.075f,0.142f,0.150f,0.147f,0.145f,0.147f,0.150f,0.142f,0.075f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.057f,0.117f,0.137f,0.145f,0.144f,0.145f,0.137f,0.117f,0.057f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,0.109f,0.128f,0.135f,0.135f,0.135f,0.127f,0.108f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,0.093f,0.114f,0.123f,0.125f,0.126f,0.115f,0.091f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.118f,0.103f,0.119f,0.131f,0.133f,0.128f,0.116f,0.107f,0.100f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.123f,0.108f,0.121f,0.134f,0.141f,0.144f,0.137f,0.130f,0.128f,0.112f,-1.0f,-1.0f,-1.0f},
};
inline constexpr float kTorsoBackM[kTorsoRows][kTorsoCols]={
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.016f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,0.010f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.008f,-0.024f,-0.034f,-1.0f,-1.0f,-1.0f,-0.034f,-0.025f,-0.012f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.017f,-0.032f,-0.041f,-0.046f,-0.046f,-0.046f,-0.042f,-0.033f,-0.020f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.020f,-0.036f,-0.044f,-0.049f,-0.048f,-0.050f,-0.044f,-0.038f,-0.024f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.022f,-0.038f,-0.045f,-0.050f,-0.047f,-0.050f,-0.046f,-0.040f,-0.027f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.025f,-0.041f,-0.046f,-0.050f,-0.045f,-0.049f,-0.046f,-0.042f,-0.028f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.020f,-0.038f,-0.043f,-0.045f,-0.038f,-0.045f,-0.043f,-0.038f,-0.022f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.009f,-0.032f,-0.040f,-0.043f,-0.035f,-0.042f,-0.039f,-0.032f,-0.009f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.009f,-0.026f,-0.038f,-0.040f,-0.031f,-0.039f,-0.036f,-0.025f,0.011f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-0.019f,-0.033f,-0.037f,-0.029f,-0.034f,-0.029f,-0.014f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-0.017f,-0.031f,-0.038f,-0.032f,-0.036f,-0.027f,-0.009f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.012f,-0.016f,-0.032f,-0.040f,-0.037f,-0.038f,-0.030f,-0.013f,0.006f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.014f,0.000f,-0.020f,-0.032f,-0.041f,-0.041f,-0.037f,-0.029f,-0.017f,0.004f,-1.0f,-1.0f,-1.0f},
};
// Torso column at (x right, z up) of the chest frame: bilinear back / front; false outside the grid or next to a column without body.
inline bool TorsoColumn(float x,float z,float& back,float& front)
{
    const float gx=(x-kTorsoX0)/kTorsoDX,gz=(z-kTorsoZ0)/kTorsoDZ;
    if(!(gx>=0.0f) || !(gz>=0.0f) || gx>float(kTorsoCols-1) || gz>float(kTorsoRows-1))return false;
    const int ix=std::min(int(gx),kTorsoCols-2),iz=std::min(int(gz),kTorsoRows-2);
    const float fx=gx-float(ix),fz=gz-float(iz);
    float f[2][2],b[2][2];
    for(int a=0;a<2;++a)for(int c=0;c<2;++c)
    {
        f[a][c]=kTorsoFrontM[iz+a][ix+c];b[a][c]=kTorsoBackM[iz+a][ix+c];
        if(f[a][c]<=-0.5f || b[a][c]<=-0.5f)return false;
    }
    front=(f[0][0]*(1-fx)+f[0][1]*fx)*(1-fz)+(f[1][0]*(1-fx)+f[1][1]*fx)*fz;
    back=(b[0][0]*(1-fx)+b[0][1]*fx)*(1-fz)+(b[1][0]*(1-fx)+b[1][1]*fx)*fz;
    return front>back;
}
inline constexpr int kStockCapsules=3,kStockParts=4; // parts: 0 trapezius, 1 upper arm, 2 neck, 3 torso
inline constexpr float kStockCapsuleRadiusM[kStockCapsules]={0.035f,0.045f,0.050f};
// The shoulder pocket (the front of the arm socket, where the butt rests when aiming) stays free: the trapezius capsule ends short of the
// socket, the upper arm capsule starts below it (the MM Rifle aim pose would otherwise read as 1.4cm inside the upper arm).
inline constexpr float kTrapeziusEnd=0.80f,kUpperArmStart=0.25f;
// Clearance kept between the stock edge lines and the body (the rifle poses without a reload keep +1.0cm p50 on the mesh).
inline constexpr float kStockClearanceM=0.01f;
// Largest shift of the native target (bounded: the native choreography stays recognisable) and the arm reach it may use (below the
// soft reach clamp of the solver, gik::Params::softStart .95, so a shifted target is still met exactly).
inline constexpr float kStockShiftMaxM=0.12f,kStockReachFraction=0.93f,kStockFoldFraction=0.25f; // fold: solver minReach .20
inline constexpr int kStockShiftIterations=4;
struct StockCapsule{math::Vec3 a,b;float r=0;};
struct StockBody
{
    StockCapsule caps[kStockCapsules];math::Vec3 neck,right,fwd,up;
    // Right arm as the solver will pose it: MM socket, upper / lower length, target hand and elbow guide (native elbow).
    math::Vec3 socket,hand,guide;float upper=0,lower=0;bool arm=false;
};
// Elbow of a two-bone arm from socket to hand (lengths a, b) bent towards guide (the plane the contact solver orients to).
inline math::Vec3 SolvedElbow(math::Vec3 socket,math::Vec3 hand,math::Vec3 guide,float a,float b)
{
    const math::Vec3 sh=hand-socket;
    const float d=std::clamp(gik::Len(sh),1.0e-4f,a+b-1.0e-4f);
    const math::Vec3 u=sh*(1.0f/std::max(gik::Len(sh),1.0e-4f));
    math::Vec3 v=guide-socket;v=v-u*math::Dot(v,u);
    const float lv=gik::Len(v);
    if(!(lv>1.0e-4f))return guide;
    v=v*(1.0f/lv);
    const float c=std::clamp((a*a+d*d-b*b)/(2.0f*a*d),-1.0f,1.0f);
    return socket+u*(a*c)+v*(a*std::sqrt(std::max(0.0f,1.0f-c*c)));
}
// Chest frame of a pose: up = Spine3 -> neck base, right = the Spine3 lateral axis (+Z of the bone, the side of the right arm),
// forward = up x right. Returns false when degenerate.
inline bool ChestFrame(const ov::Skeleton& sk,const ov::Pose& pose,int spine3,int neck,math::Vec3& right,math::Vec3& fwd,math::Vec3& up)
{
    const auto s3=native_reload::Model(sk,pose,spine3);
    up=native_reload::Model(sk,pose,neck).t-s3.t;
    const float lu=gik::Len(up);
    if(!(lu>1.0e-4f))return false;
    up=up*(1.0f/lu);
    right=math::QuatRotate(s3.r,math::Vec3{0.0f,0.0f,1.0f});
    right=right-up*math::Dot(right,up);
    const float lr=gik::Len(right);
    if(!(lr>1.0e-4f))return false;
    right=right*(1.0f/lr);
    fwd=math::Cross(up,right);
    return math::IsFinite(fwd);
}
// Neck base: the child of Spine3 on the way to the head (-1 = unknown).
inline int NeckBaseOf(const Prepared& p)
{
    const auto& sk=p.skeleton;
    const int shoulder=sk.parent[p.arms.left.upper];
    const int spine3=shoulder>=0 ? sk.parent[shoulder] : -1;
    if(spine3<0 || p.head<0 || p.head>=sk.n)return -1;
    int b=p.head;
    for(int step=0;step<sk.n && b>=0;++step){if(sk.parent[b]==spine3)return b;b=sk.parent[b];}
    return -1;
}
// Body model of `body`; the right hand and the elbow guide come from `elbowPose` (the solver puts the hand on the native target and bends
// the arm towards the native elbow: the upper arm capsule ends at that solved elbow; the measurement passes the same pose twice).
inline bool StockBodyOf(const Prepared& p,const ov::Pose& body,const ov::Pose& elbowPose,StockBody& out)
{
    const auto& sk=p.skeleton;
    const int neck=NeckBaseOf(p);
    const int shoulder=sk.parent[p.arms.left.upper];
    const int spine3=shoulder>=0 ? sk.parent[shoulder] : -1;
    if(neck<0 || spine3<0 || !ChestFrame(sk,body,spine3,neck,out.right,out.fwd,out.up))return false;
    out.neck=native_reload::Model(sk,body,neck).t;
    const math::Vec3 arm=native_reload::Model(sk,body,p.arms.right.upper).t,elbow=native_reload::Model(sk,elbowPose,p.arms.right.lower).t;
    out.caps[0]={out.neck,out.neck+(arm-out.neck)*kTrapeziusEnd,kStockCapsuleRadiusM[0]};
    out.caps[1]={arm,elbow,kStockCapsuleRadiusM[1]};
    out.socket=arm;out.hand=native_reload::Model(sk,elbowPose,p.arms.right.hand).t;out.guide=elbow;
    out.upper=gik::Len(body.b[p.arms.right.lower].t);out.lower=gik::Len(body.b[p.arms.right.hand].t);
    out.arm=out.upper>1.0e-3f && out.lower>1.0e-3f && math::IsFinite(out.hand);
    if(out.arm)out.caps[1].b=SolvedElbow(out.socket,out.hand,out.guide,out.upper,out.lower);
    out.caps[1].a=arm+(out.caps[1].b-arm)*kUpperArmStart;
    const math::Vec3 head=native_reload::Model(sk,body,p.head).t;
    out.caps[2]={out.neck,head,kStockCapsuleRadiusM[2]};
    return math::IsFinite(out.neck) && math::IsFinite(arm) && math::IsFinite(elbow) && math::IsFinite(head);
}
inline math::Vec3 ClosestOnSegment(math::Vec3 a,math::Vec3 b,math::Vec3 x)
{
    const math::Vec3 ab=b-a;const float l2=math::Dot(ab,ab);
    if(!(l2>1.0e-10f))return a;
    return a+ab*std::clamp(math::Dot(x-a,ab)/l2,0.0f,1.0f);
}
inline void StockPoints(const ov::Xf& weapon,math::Vec3 out[kRifleStockSamples])
{
    for(int e=0;e<2;++e)for(int k=0;k<kRifleStockSamplesPerEdge;++k)
    {
        const float d=kRifleStockNearM+(kRifleStockFarM-kRifleStockNearM)*float(k)/float(kRifleStockSamplesPerEdge-1);
        out[e*kRifleStockSamplesPerEdge+k]=weapon.t+math::QuatRotate(weapon.r,kRifleStockBack*d+kRifleStockUp*kRifleStockEdgeM[e]);
    }
}
// Torso depth of a point (distance to the nearer of back / front along the forward axis, 0 = outside) and its forward exit distance
// (front + margin - y; 0 = clear by the margin).
inline void TorsoDepth(const StockBody& b,math::Vec3 x,float margin,float& depth,float& exit)
{
    depth=0;exit=0;
    const math::Vec3 d=x-b.neck;
    float back,front;
    if(!TorsoColumn(math::Dot(d,b.right),math::Dot(d,b.up),back,front))return;
    const float y=math::Dot(d,b.fwd);
    if(y>back && y<front+margin)
    {
        exit=front+margin-y;
        depth=std::max(0.0f,std::min(front-y,y-back));
    }
}
// Depth (m) of the stock edge lines inside each body part of `pose` (weapon and body from the same pose, no clearance margin);
// out[kStockParts] = max of all. 0 = clear.
inline void StockDepths(const Prepared& p,const ov::Pose& pose,float out[kStockParts+1])
{
    for(int c=0;c<=kStockParts;++c)out[c]=0;
    StockBody body;
    if(!StockBodyOf(p,pose,pose,body))return;
    math::Vec3 pts[kRifleStockSamples];StockPoints(native_reload::Model(p.skeleton,pose,p.arms.weaponRight),pts);
    for(const auto& x:pts)
    {
        for(int c=0;c<kStockCapsules;++c)
        {
            const float d=body.caps[c].r-gik::Len(x-ClosestOnSegment(body.caps[c].a,body.caps[c].b,x));
            if(std::isfinite(d))out[c]=std::max(out[c],d);
        }
        float depth,exit;TorsoDepth(body,x,0.0f,depth,exit);
        if(std::isfinite(depth))out[kStockCapsules]=std::max(out[kStockCapsules],depth);
    }
    for(int c=0;c<kStockParts;++c)out[kStockParts]=std::max(out[kStockParts],out[c]);
}
// Largest depth (m) of the stock points `pts` (+ `shift`) in the body model, with `margin` added to every surface (0 = clear).
inline float StockBodyDepth(const StockBody& b,const math::Vec3* pts,int n,math::Vec3 shift,float margin)
{
    float worst=0;
    for(int i=0;i<n;++i)
    {
        const math::Vec3 x=pts[i]+shift;
        for(int c=0;c<kStockCapsules;++c)
        {
            StockCapsule cap=b.caps[c];
            if(c==1 && b.arm) // the arm follows the shifted hand
            {cap.b=SolvedElbow(b.socket,b.hand+shift,b.guide+shift,b.upper,b.lower);cap.a=b.socket+(cap.b-b.socket)*kUpperArmStart;}
            worst=std::max(worst,cap.r+margin-gik::Len(x-ClosestOnSegment(cap.a,cap.b,x)));
        }
        const math::Vec3 d=x-b.neck;
        float back,front;
        if(TorsoColumn(math::Dot(d,b.right),math::Dot(d,b.up),back,front))
        {
            const float y=math::Dot(d,b.fwd);
            worst=std::max(worst,std::min(front+margin-y,y-(back-margin)));
        }
    }
    return std::isfinite(worst) ? worst : 0.0f;
}
// ---- Left arm of the carried rifle against the torso (rifle reloads, rec 2026-10-28 05:03: forearm 6.5-9.4cm / wrist 2.1-4.7cm inside the
// chest of the Sidewinder reloads, wrist 7.6cm inside the belly of the Kyubi reloads, with the stock clearance releasing its shift while the
// native hand was still there). Torso height field of the female body mesh with the same generator as the stock table above but 20 rows
// (neck base down to z = -0.51: chest, belly, belt), used only for the left hand / forearm points of the carried target.
// Mutation 7 (UTPP_RELOAD_RIFLE_MUTATION) drops the left arm clearance, 8 the left hand smoothing, 9 lets the stock shift ignore the left arm.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==7
inline constexpr bool kLeftArmClearance=false;
#else
inline constexpr bool kLeftArmClearance=true;
#endif
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==8
inline constexpr bool kLeftHandSmoothing=false;
#else
inline constexpr bool kLeftHandSmoothing=true;
#endif
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==9
inline constexpr bool kStockShiftGuardsLeftArm=false;
#else
inline constexpr bool kStockShiftGuardsLeftArm=true;
#endif
inline constexpr int kArmTorsoRows=20;
inline constexpr float kArmTorsoFrontM[kArmTorsoRows][kTorsoCols]={
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.071f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,0.073f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.080f,0.073f,0.075f,-1.0f,-1.0f,-1.0f,0.075f,0.073f,0.083f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.084f,0.079f,0.085f,0.089f,0.088f,0.089f,0.085f,0.080f,0.085f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.088f,0.098f,0.104f,0.107f,0.108f,0.107f,0.104f,0.098f,0.089f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.103f,0.124f,0.132f,0.128f,0.124f,0.128f,0.132f,0.124f,0.104f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.121f,0.153f,0.156f,0.148f,0.135f,0.148f,0.156f,0.153f,0.121f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.122f,0.156f,0.162f,0.153f,0.140f,0.153f,0.162f,0.156f,0.122f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.075f,0.142f,0.150f,0.147f,0.145f,0.147f,0.150f,0.142f,0.075f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.057f,0.117f,0.137f,0.145f,0.144f,0.145f,0.137f,0.117f,0.057f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,0.109f,0.128f,0.135f,0.135f,0.135f,0.127f,0.108f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,0.093f,0.114f,0.123f,0.125f,0.126f,0.115f,0.091f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.118f,0.103f,0.119f,0.131f,0.133f,0.128f,0.116f,0.107f,0.100f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.123f,0.108f,0.121f,0.134f,0.141f,0.144f,0.137f,0.130f,0.128f,0.112f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.136f,0.121f,0.132f,0.140f,0.146f,0.148f,0.140f,0.139f,0.140f,0.127f,0.113f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.115f,0.132f,0.140f,0.146f,0.151f,0.151f,0.143f,0.144f,0.145f,0.135f,0.132f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.126f,0.147f,0.153f,0.154f,0.155f,0.154f,0.144f,0.150f,0.155f,0.148f,0.135f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.142f,0.166f,0.171f,0.166f,0.161f,0.152f,0.147f,0.161f,0.170f,0.167f,0.154f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.160f,0.182f,0.186f,0.180f,0.153f,0.135f,0.147f,0.175f,0.183f,0.180f,0.169f,0.162f,-1.0f},
};
inline constexpr float kArmTorsoBackM[kArmTorsoRows][kTorsoCols]={
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.017f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,0.010f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.008f,-0.024f,-0.034f,-1.0f,-1.0f,-1.0f,-0.034f,-0.025f,-0.012f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.017f,-0.032f,-0.041f,-0.046f,-0.046f,-0.046f,-0.042f,-0.033f,-0.020f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.020f,-0.036f,-0.044f,-0.049f,-0.048f,-0.050f,-0.044f,-0.038f,-0.024f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.022f,-0.038f,-0.045f,-0.050f,-0.047f,-0.050f,-0.046f,-0.040f,-0.027f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.025f,-0.041f,-0.046f,-0.050f,-0.045f,-0.049f,-0.046f,-0.042f,-0.027f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.020f,-0.038f,-0.043f,-0.045f,-0.038f,-0.045f,-0.043f,-0.038f,-0.022f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-0.009f,-0.032f,-0.040f,-0.043f,-0.035f,-0.042f,-0.039f,-0.032f,-0.009f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.009f,-0.026f,-0.038f,-0.040f,-0.031f,-0.039f,-0.036f,-0.025f,0.011f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-0.019f,-0.033f,-0.037f,-0.029f,-0.034f,-0.029f,-0.014f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,-1.0f,-0.017f,-0.031f,-0.038f,-0.032f,-0.036f,-0.027f,-0.009f,-1.0f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,-1.0f,0.012f,-0.016f,-0.032f,-0.040f,-0.037f,-0.038f,-0.030f,-0.013f,0.006f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.014f,0.000f,-0.020f,-0.032f,-0.041f,-0.041f,-0.037f,-0.029f,-0.017f,0.004f,-1.0f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.013f,-0.005f,-0.022f,-0.031f,-0.038f,-0.039f,-0.038f,-0.031f,-0.020f,0.000f,0.012f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.034f,-0.005f,-0.021f,-0.033f,-0.039f,-0.039f,-0.039f,-0.036f,-0.026f,-0.004f,0.013f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.032f,-0.008f,-0.023f,-0.038f,-0.043f,-0.040f,-0.038f,-0.044f,-0.032f,-0.008f,0.023f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.031f,-0.009f,-0.023f,-0.042f,-0.046f,-0.035f,-0.041f,-0.047f,-0.038f,-0.011f,0.022f,-1.0f,-1.0f},
    {-1.0f,-1.0f,0.026f,-0.002f,-0.022f,-0.039f,-0.042f,-0.023f,-0.035f,-0.044f,-0.036f,-0.013f,0.031f,0.031f,-1.0f},
};
// Torso column of the left-arm model (same grid as TorsoColumn, 20 rows).
inline bool ArmTorsoColumn(float x,float z,float& back,float& front)
{
    const float gx=(x-kTorsoX0)/kTorsoDX,gz=(z-kTorsoZ0)/kTorsoDZ;
    if(!(gx>=0.0f) || !(gz>=0.0f) || gx>float(kTorsoCols-1) || gz>float(kArmTorsoRows-1))return false;
    const int ix=std::min(int(gx),kTorsoCols-2),iz=std::min(int(gz),kArmTorsoRows-2);
    const float fx=gx-float(ix),fz=gz-float(iz);
    float f[2][2],b[2][2];
    for(int a=0;a<2;++a)for(int c=0;c<2;++c)
    {
        f[a][c]=kArmTorsoFrontM[iz+a][ix+c];b[a][c]=kArmTorsoBackM[iz+a][ix+c];
        if(f[a][c]<=-0.5f || b[a][c]<=-0.5f)return false;
    }
    front=(f[0][0]*(1-fx)+f[0][1]*fx)*(1-fz)+(f[1][0]*(1-fx)+f[1][1]*fx)*fz;
    back=(b[0][0]*(1-fx)+b[0][1]*fx)*(1-fz)+(b[1][0]*(1-fx)+b[1][1]*fx)*fz;
    return front>back;
}
// Left hand: wrist, hand centre (6cm along the forearm direction) and forearm middle, each with the radius of the limb.
inline constexpr int kLeftArmPoints=3;
inline constexpr float kLeftArmRadiusM[kLeftArmPoints]={0.035f,0.035f,0.040f};
inline void LeftArmPoints(const Prepared& p,const ov::Pose& pose,math::Vec3 out[kLeftArmPoints])
{
    const auto& sk=p.skeleton;
    const math::Vec3 wrist=native_reload::Model(sk,pose,p.arms.left.hand).t,fore=native_reload::Model(sk,pose,p.arms.left.lower).t;
    math::Vec3 d=wrist-fore;const float l=gik::Len(d);
    d=l>1.0e-5f ? d*(1.0f/l) : math::Vec3{0.0f,0.0f,0.0f};
    out[0]=wrist;out[1]=wrist+d*0.06f;out[2]=(wrist+fore)*0.5f;
}
// Largest penetration (m, limb radius included; <= 0: clear) of the left arm points (+ shift) in the torso of `b` (the MM body).
inline float LeftArmTorsoDepth(const StockBody& b,const math::Vec3* pts,math::Vec3 shift)
{
    float worst=-1.0f;
    for(int i=0;i<kLeftArmPoints;++i)
    {
        const math::Vec3 d=pts[i]+shift-b.neck;
        float back,front;
        if(ArmTorsoColumn(math::Dot(d,b.right),math::Dot(d,b.up),back,front))
        {
            const float y=math::Dot(d,b.fwd),r=kLeftArmRadiusM[i];
            worst=std::max(worst,std::min(front+r-y,y-(back-r)));
        }
    }
    return std::isfinite(worst) ? worst : -1.0f;
}
// Translation (model space) that carries the stock of `target` out of the body model of `body` (+ kStockClearanceM): the shortest
// shift along the chest forward, up or forward-up axis (searched in kStockSearchStepM steps up to kStockShiftMaxM) that clears every
// stock point. A stock that runs through the torso (in at the front, out at the back) cannot be cleared within the bound - a partial
// forward shift would carry the butt from just under the back into the middle of the body (recording 2026-10-26 k23/k36: 2.9 -> 7.0cm
// on the mesh; a partial shift that lowers the modelled depth also went deeper on the mesh) - such a stock is left as it is.
// Never downwards (a lowered target drops the left hand under the hips: crouch walk, recording 2026-10-26 k33). The shift is then
// shortened (bisection) until both target hands stay inside [kStockFoldFraction, kStockReachFraction] of the arm length from the MM
// arm sockets of `body` (no Reach / fold Contact reject) and the placement guard of `authored` (weapon at the head, left hand under
// the hips) is not lowered. Zero when the stock is clear or the geometry is missing.
inline constexpr float kStockSearchStepM=0.005f;
inline math::Vec3 StockClearanceOffset(const Prepared& p,const ov::Pose& body,const ov::Pose& authored,const ov::Pose& target)
{
    const math::Vec3 none{};
    StockBody model;
    if(!StockBodyOf(p,body,target,model))return none;
    const auto& sk=p.skeleton;
    math::Vec3 pts[kRifleStockSamples];StockPoints(native_reload::Model(sk,target,p.arms.weaponRight),pts);
    // Engaged only by an actual penetration (a stock that merely comes within the margin is left alone: in the MM rifle poses the stock
    // rests close beside the neck / arm), then cleared to the margin.
    if(!(StockBodyDepth(model,pts,kRifleStockSamples,none,0.0f)>0.0f))return none;
    const float diagonal=0.70710678f;
    const math::Vec3 dirs[3]={model.fwd,model.up,(model.fwd+model.up)*diagonal};
    const int steps=int(kStockShiftMaxM/kStockSearchStepM+0.5f);
    math::Vec3 offset{};bool cleared=false;
    // The shift carries the left arm too: a direction that drives the left hand / forearm deeper into the torso (an upward shift of the
    // stock out of the trapezius, rec 2026-10-28 05:03 Sidewinder: forearm 6.5 -> 9.4cm) is skipped, unless nothing else clears the stock.
    math::Vec3 leftPts[kLeftArmPoints];LeftArmPoints(p,target,leftPts);
    const float leftBase=LeftArmTorsoDepth(model,leftPts,none);
    for(int pass=0;pass<2 && !cleared;++pass)
    {
        if(pass==0 && !kStockShiftGuardsLeftArm)continue;
        for(int k=1;k<=steps && !cleared;++k)
            for(const auto& d:dirs)
                if(!(StockBodyDepth(model,pts,kRifleStockSamples,d*(kStockSearchStepM*float(k)),kStockClearanceM)>0.0f) &&
                   (pass==1 || LeftArmTorsoDepth(model,leftPts,d*(kStockSearchStepM*float(k)))<=leftBase+0.001f))
                {offset=d*(kStockSearchStepM*float(k));cleared=true;break;}
    }
    if(!cleared)return none;
    if(!(gik::Len(offset)>1.0e-5f) || !math::IsFinite(offset))return none;
    const int top=TopBoneOf(p,sk.parent[sk.parent[p.arms.left.upper]]);
    const float guard=PlacementWeight(p,authored,target);
    const auto reachOk=[&](float s){
        for(const gik::ArmChain* arm:{&p.arms.left,&p.arms.right})
        {
            const float length=gik::Len(body.b[arm->lower].t)+gik::Len(body.b[arm->hand].t);
            const math::Vec3 hand=native_reload::Model(sk,target,arm->hand).t+offset*s;
            const float reach=gik::Len(hand-native_reload::Model(sk,body,arm->upper).t);
            if(reach>kStockReachFraction*length || reach<kStockFoldFraction*length)return false;
        }
        if(s>0.0f && guard>0.0f)
        {
            ov::Pose shifted=target;
            if(top<0 || !ShiftNativeTarget(p,shifted,top,offset*s))return false;
            if(PlacementWeight(p,authored,shifted)<guard-1.0e-4f)return false;
        }
        return true;
    };
    if(reachOk(1.0f))return offset;
    if(!reachOk(0.0f))return none; // the native target is already beyond the reach: leave it to the solver's own guard
    float lo=0.0f,hi=1.0f;
    for(int k=0;k<10;++k){const float m=0.5f*(lo+hi);(reachOk(m) ? lo : hi)=m;}
    return offset*lo;
}
// Frame-to-frame continuity of the clearance: the shortest clearing shift can change direction between frames (forward <-> up); the
// applied shift (chest-frame coordinates, so it rides with the body) moves at most kStockShiftRateMps towards the wanted one. State
// lives across FK calls of one reload like the other Continuity members; a gap above .25s or a rig change starts at the raw value.
// Mutation 6 (UTPP_RELOAD_RIFLE_MUTATION) applies the raw shift.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==6
inline constexpr float kStockShiftRateMps=1.0e9f;
#else
inline constexpr float kStockShiftRateMps=0.30f; // 5mm per frame at 60fps
#endif
struct StockShiftState
{
    bool valid=false;
    uint64_t rig=0;
    int64_t lastTicks=0;
    math::Vec3 local{},last{};
    void Reset(){valid=false;local={};last={};}
    // wanted / result in chest-frame coordinates (right, forward, up). Repeated calls within one tick return the first result.
    math::Vec3 Step(math::Vec3 wanted,uint64_t rigId,int64_t ticks,int64_t ticksPerSecond)
    {
        if(!valid || rig!=rigId || ticks<=0 || ticksPerSecond<=0 || ticks<lastTicks || (ticks-lastTicks)>ticksPerSecond/4)
        {valid=true;rig=rigId;lastTicks=ticks;local=wanted;return local;}
        if(ticks==lastTicks)return local;
        const float dt=std::min(float(double(ticks-lastTicks)/double(ticksPerSecond)),.05f);
        lastTicks=ticks;
        const math::Vec3 delta=wanted-local;
        const float len=gik::Len(delta),step=kStockShiftRateMps*dt;
        local=len>step ? local+delta*(step/len) : wanted;
        return local;
    }
};
// Left arm adjustment of the rifle target: (b) torso clearance of the native left hand / forearm and (c) smoothing of the left hand relative to the
// weapon. Both are applied as ONE translation of the target's left clavicle, which carries LeftArm, LeftForeArm, LeftHand, WeaponLeft and the
// fingers rigidly: the hand-to-WeaponLeft binding and the elbow guide of the contact solve stay as authored, only the left hand moves against the
// weapon / chest by the offset (the weapon, the right arm and the stock clearance are untouched).
//  (b) The native left hand is placed against the FPP body: it passes through the chest / belly of the MM body (Kyubi reload: wrist held 7.6cm
//      inside the belly for 13 frames; Sidewinder: forearm 6.8cm inside the chest). The shortest shift forward / forward-outward / forward-up
//      (5mm steps, kLeftArmClearMaxM) that clears wrist, hand centre and forearm middle (limb radius + kLeftArmClearMarginM) is rate-limited
//      (rise kLeftArmRiseMps, fall kLeftArmFallMps in chest-frame coordinates) and follows the NATIVE hand only: unlike the stock shift it does not
//      release because the stock became clear. Mutation 10 applies it raw (no rate limit).
//  (c) The native FPP choreography snaps the left hand (Kyubi 114.019: 20cm in one frame, 65 -> 39cm from the weapon grip) and sweeps it at 5-12 m/s
//      between pouch and weapon. In the weapon frame (WeaponRight) the hand follows the native one with a speed cap (kLeftHandMaxSpeedMps), an
//      acceleration cap (kLeftHandMaxAccelMps2) and a hard lag bound (kLeftHandMaxLagM). While the hand rides with the weapon or rests on it
//      (grip, magazine insertion) its weapon-frame position is static and the filter returns it unchanged, so those contacts stay exact.
// Weighted like the correction (offsets scale with the correction weight); bounded by the arm reach (kStockReachFraction / kStockFoldFraction).
inline constexpr float kLeftArmClearMaxM=0.10f,kLeftArmClearMarginM=0.004f;
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==10
inline constexpr float kLeftArmRiseMps=1.0e9f,kLeftArmFallMps=1.0e9f; // mutation 10: the clearance offset is applied raw
#else
inline constexpr float kLeftArmRiseMps=1.0f,kLeftArmFallMps=0.25f;
#endif
// Residual clearance of the followed hand: a new entry into the torso is pushed out fast (2 m/s: 3.3cm per frame, 1.3x the old rate; a faster push pops the hand by the whole penetration, 9cm in rec 05:03 ep 18),
// only the release is rate limited (mutation 18 restores the 1.5 m/s rise: the forearm stayed 3-6cm inside the chest for 2 frames, rec 08:16).
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==18
inline constexpr float kLeftArmPostRiseMps=1.5f,kLeftArmPostFallMps=0.75f;
#else
inline constexpr float kLeftArmPostRiseMps=2.0f,kLeftArmPostFallMps=0.75f;
#endif
// Entry / exit ramp of the correction (the weight moves the hand from the MM baseline to the native placement in .12 s: up to 8.8cm per frame
// in the Spine3 frame, rec 2026-10-28 05:03). The follower starts an episode at the BASELINE hand (weapon frame of the MM pose) and acts fully
// while the weight rises or holds, so the ramp-in is a speed / acceleration limited glide from the baseline to the native hand instead of a jump.
// The lag contribution (follower - blended target) is multiplied by a follow scale that falls CONTINUOUSLY to zero together with the correction
// weight from the start of a fall (scale = weight / weight at the start of the fall, smoothed), whatever the frame rate, so the last frame of the
// exit ramp (the records then show the baseline) does not pop by the lag; it recovers within .1 s when the weight rises again. A new episode (the
// correction weight reached 0) starts every follower from the baseline again (ResetFollowers).
// Mutation 11 starts the follower at the blended hand again, 14 restores the old fade (lag at full scale until the weight is below .15), 15 keeps the
// follower state across weight 0 (a new episode within .25s resumes the old one), 16 drops the contact catch-up of the left hand.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==11
inline constexpr bool kLeftHandEntryFromBaseline=false;
#else
inline constexpr bool kLeftHandEntryFromBaseline=true;
#endif
// The episode starts from the pose the game PUBLISHED in the previous frames (the records as handed in: native masked shoulders / arms blended
// with the MM pose, moving with the gait), not from the MM snapshot `roots` (masked channels restored): in a run / crouch walk the two differ
// by 4-7cm and the first frame of the ramp jumped by that (rec 08:16: 7-8.5cm). Mutation 17 starts from `roots` again.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==17
inline constexpr bool kEntryFromPublished=false;
#else
inline constexpr bool kEntryFromPublished=true;
#endif
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==14
inline constexpr bool kFollowScaleFromFallStart=false;
#else
inline constexpr bool kFollowScaleFromFallStart=true;
#endif
inline constexpr float kLeftHandFollowWeight=0.15f; // only the mutation-14 fade
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==15
inline constexpr bool kResetFollowersAtZero=false;
#else
inline constexpr bool kResetFollowersAtZero=true;
#endif
// Left hand at the weapon (grip / magazine well): the native hand is within kLeftHandContactM of WeaponRight and nearly static there (< .5 m/s in
// the weapon frame). The follower then catches up fast (speed cap kLeftHandContactSpeedMps, acceleration x kLeftHandContactAccelScale), so the lag of an
// earlier snap never keeps the hand off the magazine / grip for more than a few frames.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==16
inline constexpr float kLeftHandContactM=0.0f;
#else
inline constexpr float kLeftHandContactM=0.30f;
#endif
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==19
inline constexpr float kLeftHandContactSpeedMps=0.5f,kLeftHandContactMaxSpeedMps=9.0f,kLeftHandContactAccelScale=4.0f; // mutation 19: the old 9 m/s
#else
inline constexpr float kLeftHandContactSpeedMps=0.5f,kLeftHandContactMaxSpeedMps=3.0f,kLeftHandContactAccelScale=4.0f; // 3 m/s: 5cm per frame at 60 fps
#endif
inline constexpr float kLeftHandMaxSpeedMps=3.0f,kLeftHandRampSpeedMps=2.2f,kLeftHandMaxAccelMps2=45.0f,kLeftHandSoftLagM=0.10f,kLeftHandCatchUpPerS=3.0f;
// The pose the game PUBLISHED on the frames before the correction started (rifle out, no reload yet): weapon in the chest frame, left hand in the
// weapon frame, remembered by ObservePublished (FkHook, read-only, no allocation). The game's own upper body envelope jumps the published arms /
// weapon by 7-8.5cm on the first frame of the reload action in a run / crouch walk (rec 08:16 ep 1 / 3: the correction output is within 1cm of the
// published pose of that frame, so the jump is in the published pose itself). An episode that starts within .25s of an observed frame starts
// its followers from THAT pose: the jump is inertialised (speed / acceleration capped like the rest of the ramp). Mutation 20 starts from the
// pose of the first correction frame again.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==20
inline constexpr bool kStartFromPreviousFrame=false;
#else
inline constexpr bool kStartFromPreviousFrame=true;
#endif
struct PublishedStart
{
    bool valid=false;uint64_t rig=0;int64_t ticks=0;math::Vec3 weapon{},hand{};
    void Reset(){valid=false;}
    bool Fresh(uint64_t rigId,int64_t now,int64_t tps)const
    { return kStartFromPreviousFrame && valid && rig==rigId && tps>0 && now>=ticks && (now-ticks)<=tps/4; }
};
struct LeftArmState
{
    // clearance offset in chest-frame coordinates (right, forward, up)
    bool clearValid=false;uint64_t clearRig=0;int64_t clearTicks=0;math::Vec3 clear{};
    // residual clearance of the final (smoothed) hand position, chest frame
    bool postValid=false;uint64_t postRig=0;int64_t postTicks=0;math::Vec3 post{};
    // hand in the weapon frame
    bool handValid=false;uint64_t handRig=0;int64_t handTicks=0;math::Vec3 hand{},vel{},lag{};
    // direction of the correction weight (per tick: a repeated call in the same tick keeps it)
    float prevWeight=0.0f,followScale=1.0f,fallStart=1.0f;int64_t weightTicks=0;bool falling=false;
    math::Vec3 xPrev{};
    void Reset(){clearValid=false;postValid=false;handValid=false;clear={};post={};hand={};vel={};lag={};prevWeight=0.0f;followScale=1.0f;fallStart=1.0f;weightTicks=0;falling=false;xPrev={};}
    static bool Stale(bool valid,uint64_t stateRig,int64_t last,uint64_t rigId,int64_t ticks,int64_t tps)
    { return !valid || stateRig!=rigId || ticks<=0 || tps<=0 || ticks<last || (ticks-last)>tps/4; }
    math::Vec3 StepClear(math::Vec3 wanted,uint64_t rigId,int64_t ticks,int64_t tps)
    {
        if(Stale(clearValid,clearRig,clearTicks,rigId,ticks,tps)){clearValid=true;clearRig=rigId;clearTicks=ticks;clear=wanted;return clear;}
        if(ticks==clearTicks)return clear;
        const float dt=std::min(float(double(ticks-clearTicks)/double(tps)),.05f);
        clearTicks=ticks;
        const math::Vec3 delta=wanted-clear;
        const float len=gik::Len(delta);
        // growing clearance (the arm is going into the body) may move faster than the release
        const float rate=gik::Len(wanted)>gik::Len(clear) ? kLeftArmRiseMps : kLeftArmFallMps,step=rate*dt;
        clear=len>step ? clear+delta*(step/len) : wanted;
        return clear;
    }
    math::Vec3 StepPost(math::Vec3 wanted,uint64_t rigId,int64_t ticks,int64_t tps)
    {
        if(Stale(postValid,postRig,postTicks,rigId,ticks,tps)){postValid=true;postRig=rigId;postTicks=ticks;post=wanted;return post;}
        if(ticks==postTicks)return post;
        const float dt=std::min(float(double(ticks-postTicks)/double(tps)),.05f);
        postTicks=ticks;
        const math::Vec3 delta=wanted-post;
        const float len=gik::Len(delta);
        const float rate=gik::Len(wanted)>gik::Len(post) ? kLeftArmPostRiseMps : kLeftArmPostFallMps,step=rate*dt;
        post=len>step ? post+delta*(step/len) : wanted;
        return post;
    }
    // Acceleration / speed limited follower: the hand moves towards x with at most kLeftHandMaxSpeedMps, changes its velocity by at most
    // kLeftHandMaxAccelMps2 and brakes in time (v <= sqrt(2 a d)), so it neither overshoots nor chatters; the hard lag bound keeps it kLeftHandMaxLagM
    // behind x at most. A resting / uniformly slow x (hand on the weapon) is followed with a lag of v^2 / 2a (1.5 m/s: 2.5cm), a static one exactly.
    // Follow scale of the lag (see the constants above); stepped once per tick, a repeated call in the same tick returns the same value.
    float FollowScale(float weight,uint64_t rigId,int64_t ticks,int64_t tps)
    {
        if(ticks==weightTicks)return followScale;
        if(Stale(handValid,handRig,handTicks,rigId,ticks,tps))
        {prevWeight=weight;weightTicks=ticks;falling=false;followScale=1.0f;fallStart=weight;return followScale;}
        const float dt=std::min(float(double(ticks-weightTicks)/double(tps)),.05f);
        const bool was=falling;
        falling=weight<prevWeight-1.0e-4f;
        if(!kFollowScaleFromFallStart)followScale=falling ? Smooth01(weight/kLeftHandFollowWeight) : 1.0f;
        else if(falling)
        {
            if(!was)fallStart=std::max(prevWeight,1.0e-3f);
            followScale=std::min(followScale,Smooth01(weight/fallStart));
        }
        else followScale=std::min(1.0f,followScale+dt/0.1f);
        prevWeight=weight;weightTicks=ticks;
        return followScale;
    }
    // x: the hand in the weapon frame of the blended target; start: where an episode begins (the baseline hand in the weapon frame of the MM pose).
    math::Vec3 StepHand(math::Vec3 x,math::Vec3 start,uint64_t rigId,int64_t ticks,int64_t tps,float catchUp=kLeftHandCatchUpPerS,
                       float maxSpeed=kLeftHandMaxSpeedMps,float maxAccel=kLeftHandMaxAccelMps2,float contactRadius=0.0f)
    {
        if(Stale(handValid,handRig,handTicks,rigId,ticks,tps)){handValid=true;handRig=rigId;handTicks=ticks;hand=start;vel={};lag=hand-x;xPrev=x;return x+lag;}
        // A second call within one tick (place() re-runs with a lowered correction weight, a different target) keeps the LAG of the first one,
        // not its absolute position.
        if(ticks==handTicks)return x+lag;
        const float dt=std::min(float(double(ticks-handTicks)/double(tps)),.05f);
        handTicks=ticks;
        if(contactRadius>0.0f && gik::Len(x)<=contactRadius && gik::Len(x-xPrev)<kLeftHandContactSpeedMps*dt)
        {maxSpeed=kLeftHandContactMaxSpeedMps;maxAccel*=kLeftHandContactAccelScale;catchUp=0.0f;} // 3 m/s = 5cm per frame at 60 fps
        xPrev=x;
        const math::Vec3 e=x-hand;const float dist=gik::Len(e);
        if(!(dist>1.0e-6f)){vel={};hand=x;lag={};return hand;}
        const math::Vec3 dir=e*(1.0f/dist);
        // Beyond kLeftHandSoftLagM the speed cap grows with the lag (a native snap / sweep much faster than the cap is caught up smoothly, no hard bound)
        const float cap=maxSpeed+std::max(0.0f,dist-kLeftHandSoftLagM)*catchUp;
        const float speed=std::min({cap,std::sqrt(2.0f*maxAccel*dist),dist/dt});
        math::Vec3 dv=dir*speed-vel;const float dvLen=gik::Len(dv),dvMax=maxAccel*dt;
        if(dvLen>dvMax)dv=dv*(dvMax/dvLen);
        vel=vel+dv;
        math::Vec3 out=hand+vel*dt;
        if(math::Dot(x-out,e)<=0.0f){out=x;vel={};} // arrived (or passed) the target within this step
        hand=out;lag=out-x;
        return hand;
    }
};
// Shortest clearing shift of the left arm points of `target` out of the torso of `model` (model space; zero when clear / unknown).
inline math::Vec3 LeftArmClearanceOffset(const Prepared& p,const ov::Pose& target,const StockBody& model,math::Vec3 base={0.0f,0.0f,0.0f})
{
    const math::Vec3 none{};
    math::Vec3 pts[kLeftArmPoints];LeftArmPoints(p,target,pts);
    for(auto& x:pts)x=x+base;
    if(!(LeftArmTorsoDepth(model,pts,none)>0.0f))return none;
    const float diagonal=0.70710678f;
    const math::Vec3 dirs[3]={model.fwd,(model.fwd-model.right)*diagonal,(model.fwd+model.up)*diagonal};
    const int steps=int(kLeftArmClearMaxM/kStockSearchStepM+0.5f);
    math::Vec3 offset{};bool cleared=false;
    for(int k=1;k<=steps && !cleared;++k)
        for(const auto& d:dirs)
            if(LeftArmTorsoDepth(model,pts,d*(kStockSearchStepM*float(k)))<=-kLeftArmClearMarginM)
            {offset=d*(kStockSearchStepM*float(k));cleared=true;break;}
    if(!cleared)return none;
    return math::IsFinite(offset) ? offset : none;
}
// Weapon follower (rifle). The ramp of the correction moves the whole carried target (weapon + both hands) from the MM baseline to the native
// placement in .12 s (up to 8.8cm per frame in the Spine3 frame, rec 2026-10-28 05:03), and the native FPP choreography snaps it in the hold as well.
// The weapon in the chest frame is followed with the same speed / acceleration limited follower as the left hand (the episode starts at the
// baseline weapon, the lag is faded out on the last frames of a falling weight), applied as ONE rigid translation of the top bone (like the
// stock shift: both hands keep their place in the weapon frame, the magazine contact is untouched). Bounded by the arm reach of both arms.
// Mutation 12 turns it off.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==12
inline constexpr bool kWeaponFollower=false;
#else
inline constexpr bool kWeaponFollower=true;
#endif
inline constexpr float kWeaponCatchUpPerS=8.0f,kWeaponMaxSpeedMps=1.8f,kWeaponMaxAccelMps2=25.0f; // the weapon (ramp of .5m) is caught up less aggressively than the hand
inline float FollowWeapon(const Prepared& p,const ov::Pose& body,const ov::Pose& startPose,ov::Pose& target,int top,LeftArmState* state,uint64_t rigId,
                          int64_t ticks,int64_t tps,float weight,const PublishedStart* pub=nullptr)
{
    if constexpr(!kWeaponFollower)return 0.0f;
    if(!state || top<0 || !(weight>0.0f))return 0.0f;
    const auto& sk=p.skeleton;
    StockBody frame;
    if(!StockBodyOf(p,body,target,frame))return 0.0f;
    const auto local=[&](math::Vec3 v){const math::Vec3 d=v-frame.neck;return math::Vec3{math::Dot(d,frame.right),math::Dot(d,frame.fwd),math::Dot(d,frame.up)};};
    const math::Vec3 weapon0=native_reload::Model(sk,target,p.arms.weaponRight).t;
    const math::Vec3 x=local(weapon0);
    math::Vec3 start=x;
    if(kLeftHandEntryFromBaseline && weight<0.999f)
    {
        const math::Vec3 b=local(native_reload::Model(sk,startPose,p.arms.weaponRight).t);
        if(math::IsFinite(b))start=b;
        if(pub && pub->Fresh(rigId,ticks,tps) && math::IsFinite(pub->weapon))start=pub->weapon; // the previous published frame
    }
    const math::Vec3 s=state->StepHand(x,start,rigId,ticks,tps,kWeaponCatchUpPerS,kWeaponMaxSpeedMps,kWeaponMaxAccelMps2);
    const float scale=state->FollowScale(weight,rigId,ticks,tps);
    const math::Vec3 lag=(s-x)*scale;
    math::Vec3 offset=frame.right*lag.x+frame.fwd*lag.y+frame.up*lag.z;
    if(!math::IsFinite(offset) || !(gik::Len(offset)>1.0e-6f))return 0.0f;
    const auto reachOk=[&](float f){
        for(const gik::ArmChain* arm:{&p.arms.left,&p.arms.right})
        {
            const float length=gik::Len(body.b[arm->lower].t)+gik::Len(body.b[arm->hand].t);
            const math::Vec3 socket=native_reload::Model(sk,body,arm->upper).t,hand=native_reload::Model(sk,target,arm->hand).t;
            const float reach0=gik::Len(hand-socket),reach=gik::Len(hand+offset*f-socket);
            if(reach>std::max(kStockReachFraction*length,reach0) || reach<std::min(kStockFoldFraction*length,reach0))return false;
        }
        return true;
    };
    float scaleReach=1.0f;
    if(!reachOk(1.0f))
    {
        float lo=0.0f,hi=1.0f;
        for(int k=0;k<10;++k){const float m=0.5f*(lo+hi);(reachOk(m) ? lo : hi)=m;}
        scaleReach=lo;
    }
    offset=offset*scaleReach;
    if(!(gik::Len(offset)>1.0e-6f) || !ShiftNativeTarget(p,target,top,offset))return 0.0f;
    return gik::Len(offset);
}
// Applies the left arm adjustment to `target` (translation of the left clavicle). `body` = the MM pose the solver works on (masked channels
// restored). Returns the length of the applied offset (m); 0 = nothing changed.
inline float AdjustLeftArm(const Prepared& p,const ov::Pose& body,const ov::Pose& startPose,ov::Pose& target,LeftArmState* state,uint64_t rigId,
                           int64_t ticks,int64_t tps,float weight,const PublishedStart* pub=nullptr)
{
    const auto& sk=p.skeleton;
    const int clavicle=sk.parent[p.arms.left.upper];
    const int spine3=clavicle>=0 ? sk.parent[clavicle] : -1;
    if(spine3<0 || !(weight>0.0f))return 0.0f;
    StockBody frame;
    if(!StockBodyOf(p,body,target,frame))return 0.0f;
    const math::Vec3 hand0=native_reload::Model(sk,target,p.arms.left.hand).t;
    math::Vec3 total{};
    if constexpr(kLeftArmClearance)
    {
        math::Vec3 wanted=LeftArmClearanceOffset(p,target,frame);
        if(state)
        {
            const math::Vec3 local=state->StepClear(math::Vec3{math::Dot(wanted,frame.right),math::Dot(wanted,frame.fwd),math::Dot(wanted,frame.up)},
                                                    rigId,ticks,tps);
            wanted=frame.right*local.x+frame.fwd*local.y+frame.up*local.z;
        }
        total=wanted*weight;
    }
    if constexpr(kLeftHandSmoothing)
    {
        if(state)
        {
            const auto wr=native_reload::Model(sk,target,p.arms.weaponRight);
            const math::Vec3 x=math::QuatRotate(math::QuatInv(wr.r),hand0+total-wr.t);
                    // (on the ramp the hand follower is slower, kLeftHandRampSpeedMps: with the weapon follower the sum stays <= 4 m/s in the Spine3 frame)
            // episode start: the hand the game published (records as handed in) in its weapon frame
            math::Vec3 start=x;
            if(kLeftHandEntryFromBaseline && weight<0.999f)
            {
                const auto wb=native_reload::Model(sk,startPose,p.arms.weaponRight);
                const math::Vec3 hb=math::QuatRotate(math::QuatInv(wb.r),native_reload::Model(sk,startPose,p.arms.left.hand).t-wb.t);
                if(math::IsFinite(hb))start=hb;
            }
            const math::Vec3 s=state->StepHand(x,start,rigId,ticks,tps,kLeftHandCatchUpPerS,weight<0.999f ? kLeftHandRampSpeedMps : kLeftHandMaxSpeedMps,kLeftHandMaxAccelMps2,kLeftHandContactM);
            const float scale=state->FollowScale(weight,rigId,ticks,tps);
            total=total+math::QuatRotate(wr.r,s-x)*scale;
        }
    }
    if constexpr(kLeftArmClearance)
    {
        // The followed hand can cut through the torso on its way to a native hand that rests at the belly: the penetration left at the final
        // position is cleared too (own rate limit; it does not feed back into the follower).
        math::Vec3 wanted=LeftArmClearanceOffset(p,target,frame,total);
        if(state)
        {
            const math::Vec3 local=state->StepPost(math::Vec3{math::Dot(wanted,frame.right),math::Dot(wanted,frame.fwd),math::Dot(wanted,frame.up)},
                                                   rigId,ticks,tps);
            wanted=frame.right*local.x+frame.fwd*local.y+frame.up*local.z;
        }
        total=total+wanted*weight;
    }
    if(!math::IsFinite(total) || !(gik::Len(total)>1.0e-6f))return 0.0f;
    // Arm reach: the shifted hand stays inside [kStockFoldFraction, kStockReachFraction] of the arm length from the MM left arm socket (when
    // the native hand is inside that range itself): no Reach / fold Contact reject because of this adjustment.
    const float length=gik::Len(body.b[p.arms.left.lower].t)+gik::Len(body.b[p.arms.left.hand].t);
    const math::Vec3 socket=native_reload::Model(sk,body,p.arms.left.upper).t;
    // A native hand already outside that range (extended arm: the solver soft clamp owns it) may not be taken further out, but the adjustment
    // is not dropped (an all-or-nothing switch would pop the hand by the whole offset).
    const float reach0=gik::Len(hand0-socket);
    const float reachHi=std::max(kStockReachFraction*length,reach0),reachLo=std::min(kStockFoldFraction*length,reach0);
    const auto reachOk=[&](float s){
        const float reach=gik::Len(hand0+total*s-socket);
        return reach<=reachHi && reach>=reachLo;
    };
    if(!std::isfinite(reach0))return 0.0f;
    float scale=1.0f;
    if(!reachOk(1.0f))
    {
        float lo=0.0f,hi=1.0f;
        for(int k=0;k<10;++k){const float m=0.5f*(lo+hi);(reachOk(m) ? lo : hi)=m;}
        scale=lo;
    }
    total=total*scale;
    const auto parent=native_reload::Model(sk,target,spine3);
    const math::Vec3 local=math::QuatRotate(math::QuatInv(parent.r),total);
    if(!math::IsFinite(local))return 0.0f;
    target.b[clavicle].t=target.b[clavicle].t+local;
    return gik::Len(total);
}
// The placement factor is recomputed every frame from the current native pose. It may only fall AT ONCE (a weapon that
// closes in on the head must never be let nearer than the raw factor allows: 21.10 01:56 k4 went 12.9 -> 3.5cm with a
// falling-rate limit) and recovers at most 1/.20s, so a weapon leaving the head zone cannot snap the correction back on.
// State lives across FK calls of one reload (single FK thread); a gap above .25s, a rig change or missing time starts a
// new episode at the raw value.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==7
inline constexpr bool kPlacementSmoothing=false; // mutation 7: raw factor, no limited recovery
#else
inline constexpr bool kPlacementSmoothing=true;
#endif
struct PlacementGate
{
    float value=1;
    int64_t lastTicks=0;
    uint64_t rig=0;
    void Reset(){value=1;lastTicks=0;rig=0;}
    float Step(float raw,uint64_t rigId,int64_t ticks,int64_t ticksPerSecond)
    {
        raw=std::isfinite(raw) ? std::clamp(raw,0.0f,1.0f) : 1.0f;
        if constexpr(kPlacementSmoothing)
        {
            if(ticks<=0 || ticksPerSecond<=0 || rigId!=rig || lastTicks<=0 || ticks<lastTicks ||
               (ticks-lastTicks)>ticksPerSecond/4)
            {value=raw;rig=rigId;lastTicks=ticks;return value;}
            const float dt=std::min(float(double(ticks-lastTicks)/double(ticksPerSecond)),.05f);
            if(ticks>lastTicks)
            {
                value=std::min(raw,value+dt/.20f);
                lastTicks=ticks;
            }
            else value=std::min(value,raw);
            return value;
        }
        return raw;
    }
};
// Time-limited, latched divergence handling. The stateless functions above change the hand target 4-10x faster than
// the native pelvis moves (the correction weight is faded over only .06m of pelvis distance) and re-cross the arm reach
// boundary on the way back (standing up). The state machine below latches the transport (engaged) with hysteresis -
// engage only when the pelvis is beyond kTorsoTransportStartM AND the gate has faded to 0, release only below
// kTorsoReleaseM AND after the gate has faded to 0 again - and moves the gate at most 1/kTorsoGateSeconds, so the
// output is always continuous: baseline at every switch of the latch.
inline constexpr float kTorsoGateSeconds=.25f;
struct DivergenceState
{
    bool engaged=false;
    float gate=1;
    int64_t lastTicks=0;
    uint64_t rig=0;
    void Reset(){engaged=false;gate=1;lastTicks=0;rig=0;}
    void Step(float pelvisDistance,uint64_t rigId,int64_t ticks,int64_t ticksPerSecond,float& lambdaOut,float& gateOut)
    {
        if(!std::isfinite(pelvisDistance)){lambdaOut=0;gateOut=1;return;}
        const bool stale=rigId!=rig || lastTicks<=0 || ticks<=0 || ticksPerSecond<=0 || ticks<lastTicks ||
                         (ticks-lastTicks)>ticksPerSecond/4;
        if(stale)
        {
            rig=rigId;lastTicks=ticks;
            engaged=TorsoTransportLambda(pelvisDistance)>0.0f;
            gate=TorsoDivergenceGate(pelvisDistance);
        }
        else if(ticks>lastTicks)
        {
            const float dt=std::min(float(double(ticks-lastTicks)/double(ticksPerSecond)),.05f);
            lastTicks=ticks;
            const float target=engaged ? (pelvisDistance>=kTorsoReleaseM ? 1.0f : 0.0f)
                                       : (pelvisDistance>=kTorsoFadeStartM ? 0.0f : 1.0f);
            const float step=dt/kTorsoGateSeconds;
            gate=std::clamp(target,gate-step,gate+step);
            if(!engaged && gate<=0.01f && TorsoTransportLambda(pelvisDistance)>0.0f)engaged=true;
            else if(engaged && gate<=0.01f && pelvisDistance<kTorsoReleaseM)engaged=false;
        }
        lambdaOut=engaged ? 1.0f : 0.0f;gateOut=kTorsoGateEnabled ? gate : 1.0f;
    }
};
// Cold-path builder shared by MotionDriver::RemapLocked and the offline tests, so the tests see the production topology
// (Root and Trajectory mapped, no identity roots). gameIndex = the cold readMap (firstBone 0).
template<class ParentOf,class NameAt>
inline bool FillPrepared(Prepared& out,uint64_t rig,uint32_t numBones,const int32_t* gameIndex,ParentOf&& parentOf,NameAt&& nameAt)
{
    out=Prepared{};
    if(numBones==0 || numBones>uint32_t(ov::kMaxBones))return false;
    out.rig=rig;
    out.skeleton.n=static_cast<int>(numBones);
    for(uint32_t i=0;i<numBones;++i)
    {
        out.skeleton.parent[i]=static_cast<int16_t>(parentOf(i));
        out.gameIndex[i]=gameIndex[i];
        const auto name=nameAt(i);
        if(name=="Head")out.head=static_cast<int>(i);
        if(name=="Hips")out.hips=static_cast<int>(i);
    }
    out.arms=gik::FindRig(out.skeleton.n,[&](int i){return nameAt(static_cast<uint32_t>(i));});
    return out.skeleton.Valid() && out.arms.Valid(out.skeleton);
}
inline bool Valid(const Prepared& p,const integ::PoseSnapshot& s,uint32_t count)
{
    if(!p.rig || p.rig!=s.rig || !p.skeleton.Valid() ||
       p.skeleton.n>int(integ::kMaxMMBones) || s.numBones!=uint32_t(p.skeleton.n) ||
       p.arms.weaponLeft>=p.skeleton.n || !p.arms.Valid(p.skeleton))return false;
    for(int b:{p.arms.left.upper,p.arms.left.lower,p.arms.left.hand,
               p.arms.right.upper,p.arms.right.lower,p.arms.right.hand,
               int(p.skeleton.parent[p.arms.left.upper]),int(p.skeleton.parent[p.arms.right.upper])})
        if(b<0 || !integ::SnapshotUpperBone(s,uint32_t(b)))return false;
    for(int i=0;i<p.skeleton.n;++i)
    {
        const int g=p.gameIndex[i];
        // Snapshot intentionally omits Root/Trajectory; cold readMap includes them.
        if(uint32_t(i)>=s.firstBone && g!=s.gameIndex[i])return false;
        if(g<0)
        {
            if(!IdentityRoot(p,i) || uint32_t(i)>=s.firstBone)return false;
            const int par=p.skeleton.parent[i];
            if(par>=0 && !IdentityRoot(p,par))return false;
        }
        else
        {
            if(uint32_t(g)>=count || IdentityRoot(p,i))return false;
            for(int j=0;j<i;++j)if(p.gameIndex[j]==g)return false;
        }
    }
    return true;
}
inline bool Read(const Prepared& p,const uint8_t* records,ov::Pose& out)
{
    out.n=p.skeleton.n;
    for(int i=0;i<out.n;++i)
    {
        if(p.gameIndex[i]<0){out.b[i]=ov::Xf{};continue;}
        float r[12];std::memcpy(r,records+size_t(p.gameIndex[i])*48,sizeof(r));
        for(float f:r)if(!std::isfinite(f))return false;
        if(std::fabs(r[8]-1)>1e-4f || std::fabs(r[9]-1)>1e-4f ||
           std::fabs(r[10]-1)>1e-4f)return false;
        const math::Quat rotation{r[4],r[5],r[6],r[7]};
        const float normSquared=math::QuatDot(rotation,rotation);
        if(!std::isfinite(normSquared) || normSquared<=0.0f ||
           std::fabs(normSquared-1.0f)>kMaxNativeQuatNormSquaredError)return false;
        out.b[i].t={r[0],r[1],r[2]};out.b[i].r=math::QuatNormalize(rotation);
        out.b[i].s={1,1,1};
    }
    return true;
}
// eligible is an explicit SAME snapshot policy: fully weighted TPP, stationary,
// drawn pistol reload OR its explicit action0 fade tail; caller checks perspective before
// Finish. No native ADS, camera, release-tail or other weapon policy is changed.
inline bool Capture(const Prepared& p,const integ::PoseSnapshot& s,
                    uint8_t* records,uint32_t count,bool eligible,Cycle& cycle)
{
    cycle.valid=false;
    if(!eligible || !records || (s.traceAction!=2 && s.traceAction!=0) || !std::isfinite(s.weight) ||
       s.weight<1-ov::kZeroWeight || !Valid(p,s,count) || !Read(p,records,cycle.native))return false;
    cycle.records=records;cycle.count=count;cycle.sequence=s.sequence;cycle.rig=s.rig;
    cycle.valid=true;return true;
}
// Clearance of an OUTPUT pose (hold replay, ramp-in): the raw placement guard above only judges the solver target. A faded /
// ramped output is baseline + amplitude * (delta recorded against an OLDER baseline), which can carry the weapon into
// the head zone although neither endpoint is there (21.10 01:56 k4: 3.5cm, raw guard 12.9cm). 1 = clear.
// Wider band than the target guard: a replayed / ramped / rate-limited output lags the target by a few frames.
// The band is RELATIVE to `refM`, the weapon-head distance of the previous frame's final output: an output that does not come
// nearer to the head than the previous one (up to the band's upper end) keeps its full amplitude. With a fixed band every
// single rejected frame of a healthy reload at 10-18cm cut the replay (to ~.3 at 13cm, ~.55 at 15cm) and the next accepted
// frame (no clearance on that path) put it back to 1: 1.0 -> .3 -> 1.0 in three frames (review of 7902462). Without a
// reference (episode start) the full band applies. Mutation 13 = the fixed band.
inline constexpr float kClearanceMinM=0.10f,kClearanceFullM=0.18f,kClearanceEpsM=0.005f;
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==13
inline constexpr bool kClearanceRelative=false; // mutation 13: fixed 10-18cm band
#else
inline constexpr bool kClearanceRelative=true;
#endif
// Weapon-head distance of a pose; < 0 when unknown.
inline float WeaponHeadM(const Prepared& p,const ov::Pose& pose)
{
    if(p.head<0 || p.head>=p.skeleton.n)return -1.0f;
    const float d=gik::Len(native_reload::Model(p.skeleton,pose,p.arms.weaponRight).t-native_reload::Model(p.skeleton,pose,p.head).t);
    return std::isfinite(d) ? d : -1.0f;
}
inline float HeadClearance(const Prepared& p,const ov::Pose& pose,float refM=-1.0f)
{
    if constexpr(kPlacementGuardEnabled)
    {
        const float d=WeaponHeadM(p,pose);
        if(d<0.0f)return 1.0f;
        float full=kClearanceFullM;
        if constexpr(kClearanceRelative)
        {
            if(refM>0.0f)full=std::clamp(refM,kClearanceMinM+kClearanceEpsM,kClearanceFullM);
        }
        return Smooth01((d-kClearanceMinM)/(full-kClearanceMinM));
    }
    return 1.0f;
}
// Continuity of the correction across rejected frames. When the solve of a frame is rejected (Reach / Pole / Solve /
// Contact / Binding) the records keep the baseline, which switches the whole overlay off - and the next accepted frame
// switches it back on - within one frame (forearm / upper-arm pops of 60-170deg in the recordings). One amplitude `amp`
// scales the correction delta (candidate - baseline, per arm-chain bone): a rejected frame replays the last accepted
// delta at `amp`, falling at most 1/kRejectFadeSeconds; an accepted frame after a rejection rises at most
// 1/kRejectFadeSeconds from where `amp` stood (healthy episodes: always accepted, amp == 1, output untouched).
// State lives across FK calls of one reload (single FK thread); a different rig, a gap above .25s or a denied
// perspective starts a new episode (accepted: amp 1, as before; rejected: amp 0, nothing to replay).
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==8
inline constexpr bool kRejectFade=false; // mutation 8: a rejected frame shows the baseline at once
#else
inline constexpr bool kRejectFade=true;
#endif
inline constexpr float kRejectFadeSeconds=.12f;
struct RejectHold
{
    bool valid=false,haveDelta=false;
    uint64_t rig=0;
    int64_t lastTicks=0;
    float weight=0,amp=1;
    float headM=-1.0f; // weapon-head distance of the previous frame's final output (reference of HeadClearance); < 0 unknown
    int n=0;
    int bone[8]{};
    math::Quat dq[8]{};
    math::Vec3 dt[8]{};
    void Reset(){valid=false;haveDelta=false;n=0;amp=1;headM=-1.0f;}
    static int Chains(const Prepared& p,int* out)
    {
        int c=0;
        if(!p.skeleton.Valid() || !p.arms.Valid(p.skeleton))return 0; // Prepared{} (published by a remap) has upper == -1
        for(const gik::ArmChain* arm:{&p.arms.left,&p.arms.right})
        {
            const int clavicle=p.skeleton.parent[arm->upper];
            if(clavicle>=0)out[c++]=clavicle;
            out[c++]=arm->upper;out[c++]=arm->lower;out[c++]=arm->hand;
        }
        return c;
    }
    // Time bookkeeping shared by accepted / rejected frames. Returns false and starts a new episode when stale.
    bool Advance(uint64_t rigId,int64_t ticks,int64_t ticksPerSecond,float direction)
    {
        const bool stale=!valid || rig!=rigId || ticks<=0 || ticksPerSecond<=0 || ticks<lastTicks ||
                         (ticks-lastTicks)>ticksPerSecond/4;
        if(stale){valid=true;rig=rigId;lastTicks=ticks;headM=-1.0f;return false;}
        if(ticks>lastTicks)
        {
            const float step=std::min(float(double(ticks-lastTicks)/double(ticksPerSecond)),.05f)/kRejectFadeSeconds;
            amp=std::clamp(amp+direction*step,0.0f,1.0f);
            lastTicks=ticks;
        }
        return true;
    }
    // Accepted frame: remember the full delta; returns the amplitude to apply to the candidate (1 = exact).
    float OnAccept(const Prepared& p,const ov::Pose& baseline,const ov::Pose& candidate,uint64_t rigId,
                   int64_t ticks,int64_t ticksPerSecond,float w)
    {
        if constexpr(kRejectFade)
        {
            if(!Advance(rigId,ticks,ticksPerSecond,+1.0f))amp=1.0f;
            int bones[8];const int c=Chains(p,bones);
            n=0;
            for(int k=0;k<c;++k)
            {
                const int b=bones[k];
                bone[n]=b;
                dq[n]=math::QuatNormalize(math::QuatMul(candidate.b[b].r,math::QuatInv(baseline.b[b].r)));
                dt[n]=candidate.b[b].t-baseline.b[b].t;
                ++n;
            }
            haveDelta=w>0;weight=w;
            return amp;
        }
        return 1.0f;
    }
    // Rejected frame (or a frame without a native cycle): baseline in, baseline + amp * last delta out.
    bool OnReject(const Prepared& p,const integ::PoseSnapshot& s,uint8_t* records,uint32_t count,
                  int64_t ticks,int64_t ticksPerSecond,float currentWeight)
    {
        if constexpr(kRejectFade)
        {
            if(!Advance(s.rig,ticks,ticksPerSecond,-1.0f)){amp=0.0f;haveDelta=false;return false;}
            if(!haveDelta || amp<=0 || !(weight>0) || !std::isfinite(currentWeight) || !Valid(p,s,count))return false;
            float f=amp*std::clamp(currentWeight/weight,0.0f,1.0f);
            ov::Pose base;
            if(!Read(p,records,base))return false;
            ov::Xf out[8];
            for(int pass=0;pass<2;++pass)
            {
                for(int k=0;k<n;++k)
                {
                    const int b=bone[k];
                    out[k]=base.b[b];
                    const auto full=math::QuatNormalize(math::QuatMul(dq[k],base.b[b].r));
                    out[k].r=math::QuatNlerpShortest(base.b[b].r,full,f);
                    out[k].t=base.b[b].t+dt[k]*f;
                    if(!math::IsFinite(out[k].r) || !math::IsFinite(out[k].t))return false;
                }
                if(pass==1)break;
                ov::Pose probe=base;
                for(int k=0;k<n;++k){probe.b[bone[k]].r=out[k].r;probe.b[bone[k]].t=out[k].t;}
                const float clearance=HeadClearance(p,probe,headM);
                if(clearance>=1.0f)break;
                f*=clearance; // the replay must not carry the weapon nearer to the head zone than the previous output
            }
            for(int k=0;k<n;++k) // all validation precedes the first write
            {
                const int b=bone[k];
                if(uint32_t(b)<s.firstBone || p.gameIndex[b]<0)continue;
                uint8_t* rec=records+size_t(p.gameIndex[b])*48;
                const float tv[3]={out[k].t.x,out[k].t.y,out[k].t.z},qv[4]={out[k].r.x,out[k].r.y,out[k].r.z,out[k].r.w};
                std::memcpy(rec,tv,sizeof(tv));std::memcpy(rec+16,qv,sizeof(qv));
            }
            return true;
        }
        return false;
    }
};
// Joint-space speed limit on the arm chain of an ACCEPTED frame. The solver output is a pure function of the targets
// and of three ill-conditioned inputs (hand-axis direction near the shoulder, elbow-plane azimuth, guide projection)
// - measured upper-arm jumps of 105-172deg per frame (21.10 01:56 k1 172, k2 154, k4 135; R19 k2 134; N37 k3 125) while
// the baseline arm moved < 12deg and the native arm < 3deg. Each local rotation of the clavicle / upper arm / forearm
// may change by at most kJointSlewRadPerSec * dt from the previous output; the hand reaches the native contact a few
// frames later. Frames below the limit are untouched (bit-identical).
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==11
inline constexpr float kJointSlewRadPerSec=1.0e9f; // mutation 11: no limit
#else
inline constexpr float kJointSlewRadPerSec=math::kPi*2400.0f/180.0f; // 40deg per frame at 60fps
#endif
// While the correction weight is still ramping (entry / exit of a stationary pistol reload) the solver output travels from
// the game/MM baseline to the contact solve. Recording 2026-10-25 (rec2 8 of 13 corrected entries, rec1 15+): the written
// upper arm / forearm sat 70-179 deg away from the blended baseline for 0.1-0.4 s and moved at the 40 deg/frame limit
// (LArm 40.5 / 38.2 deg per frame at 108.62 / 108.64, 41.5 at 111.47; an 89 deg excursion beyond BOTH endpoints). 1000 deg/s
// (16.7 deg per frame at 60 fps) smears the transient; at full weight the old limit stays (mid-reload hand moves are
// legitimate). Offline replay of the final series: sim_slew_limit_rec2.txt (15 deg/frame: mean lag 8 deg, max 64 deg).
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==23
inline constexpr float kJointSlewRampRadPerSec=1.0e9f; // mutation 23: no ramp limit
#else
inline constexpr float kJointSlewRampRadPerSec=math::kPi*1000.0f/180.0f;
#endif
inline float JointSlewFor(float correctionWeight)
{
    return (std::isfinite(correctionWeight) && correctionWeight<0.999f) ? kJointSlewRampRadPerSec : kJointSlewRadPerSec;
}
// Rifle: the weapon / hand followers already limit the speed of the targets on the ramp; the 1000 deg/s ramp slew of the arm joints then only delayed the
// arm behind its (smooth) target and released it with a 12cm hand step when the weight reached 1 (rec 08:16 ep 3, replay). The rifle ramp uses the
// ordinary slew. Pistols keep JointSlewFor bit for bit. Mutation 21 restores the ramp slew for rifles.
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==21
inline constexpr bool kRifleOrdinaryRampSlew=false;
#else
inline constexpr bool kRifleOrdinaryRampSlew=true;
#endif
inline float JointSlewForOverlay(int32_t overlay,float correctionWeight)
{
    if(kRifleOrdinaryRampSlew && overlay==static_cast<int32_t>(ov::PoseOverlay::Rifle))return kJointSlewRadPerSec;
    return JointSlewFor(correctionWeight);
}
struct JointLimiter
{
    bool valid=false;
    uint64_t rig=0;
    int64_t lastTicks=0;
    int n=0;
    int bone[6]{};
    math::Quat prev[6]{};
    void Reset(){valid=false;n=0;}
    // The limiter reads (and may write) the records of the WHOLE skeleton: every mapped bone must address a record of the
    // buffer. Callers also validate the snapshot (Valid); this keeps the limiter safe on its own.
    static bool RecordsInBounds(const Prepared& p,const uint8_t* records,uint32_t count)
    {
        if(!records || !p.skeleton.Valid() || !p.arms.Valid(p.skeleton))return false;
        for(int i=0;i<p.skeleton.n;++i)
        {
            const int g=p.gameIndex[i];
            if(g>=0 && uint32_t(g)>=count)return false;
        }
        return true;
    }
    static int Tracked(const Prepared& p,int* tracked)
    {
        int chains[8];const int c=RejectHold::Chains(p,chains);
        int t=0;
        for(int k=0;k<c;++k) // clavicle, upper, lower of each arm (the hand keeps the native rotation)
            if(chains[k]!=p.arms.left.hand && chains[k]!=p.arms.right.hand && t<6)tracked[t++]=chains[k];
        return t;
    }
    // Limits the candidate against the previous OUTPUT (accepted, held or baseline) of this reload.
    void Limit(const Prepared& p,ov::Pose& candidate,uint64_t rigId,int64_t ticks,int64_t ticksPerSecond,
               float slewRadPerSec=kJointSlewRadPerSec) const
    {
        int tracked[6];const int t=Tracked(p,tracked);
        if(!valid || rig!=rigId || n!=t || ticks<=0 || ticksPerSecond<=0 || ticks<=lastTicks ||
           (ticks-lastTicks)>ticksPerSecond/4)
            return;
        const float maxRad=slewRadPerSec*std::min(float(double(ticks-lastTicks)/double(ticksPerSecond)),.05f);
        for(int k=0;k<t;++k)
        {
            if(bone[k]!=tracked[k])continue;
            math::Quat& q=candidate.b[tracked[k]].r;
            const float angle=2.0f*std::acos(std::fabs(std::clamp(math::QuatDot(prev[k],q),-1.0f,1.0f)));
            if(angle>maxRad)q=Slerp(prev[k],q,maxRad,angle);
        }
    }
    // Rotation from `a` towards `b` by exactly `rad` (angle = their separation); nlerp would overshoot on large angles.
    static math::Quat Slerp(math::Quat a,math::Quat b,float rad,float angle)
    {
        if(math::QuatDot(a,b)<0.0f){b.x=-b.x;b.y=-b.y;b.z=-b.z;b.w=-b.w;}
        const float s=std::sin(angle*0.5f);
        if(!(s>1.0e-6f))return b;
        const float half=angle*0.5f,keep=std::sin(half-rad*0.5f)/s,take=std::sin(rad*0.5f)/s;
        return math::QuatNormalize(math::Quat{a.x*keep+b.x*take,a.y*keep+b.y*take,a.z*keep+b.z*take,a.w*keep+b.w*take});
    }
    // Final stage on the written records: limit the tracked rotations against the previous OUTPUT, write back what changed,
    // remember the result. Covers every path (accepted, replayed hold, ramp-in, baseline).
    void Apply(const Prepared& p,uint8_t* records,uint32_t count,uint64_t rigId,int64_t ticks,int64_t ticksPerSecond,
               float slewRadPerSec=kJointSlewRadPerSec)
    {
        ov::Pose pose;
        if(!RecordsInBounds(p,records,count) || !Read(p,records,pose)){valid=false;return;}
        const ov::Pose before=pose;
        Limit(p,pose,rigId,ticks,ticksPerSecond,slewRadPerSec);
        int tracked[6];const int t=Tracked(p,tracked);
        for(int k=0;k<t;++k)
        {
            const int b=tracked[k];
            if(p.gameIndex[b]<0 || std::memcmp(&pose.b[b].r,&before.b[b].r,sizeof(math::Quat))==0)continue;
            const float q[4]={pose.b[b].r.x,pose.b[b].r.y,pose.b[b].r.z,pose.b[b].r.w};
            std::memcpy(records+size_t(p.gameIndex[b])*48+16,q,sizeof(q));
        }
        Capture(p,records,count,rigId,ticks);
    }
    // Remembers what was actually written to the records this frame.
    void Capture(const Prepared& p,const uint8_t* records,uint32_t count,uint64_t rigId,int64_t ticks)
    {
        ov::Pose pose;
        int tracked[6];const int t=Tracked(p,tracked);
        if(ticks<=0 || !RecordsInBounds(p,records,count) || !Read(p,records,pose)){valid=false;return;}
        for(int k=0;k<t;++k){bone[k]=tracked[k];prev[k]=pose.b[tracked[k]].r;}
        n=t;valid=true;rig=rigId;lastTicks=ticks;
    }
};
// Everything the FK hook keeps between frames of one reload (single FK thread).
struct Continuity
{
    PlacementGate placement;
    DivergenceState divergence;
    RejectHold reject;
    JointLimiter joints;
    StockShiftState stock;
    LeftArmState leftArm,weaponFollow;
    PublishedStart published;
    void Reset(){placement.Reset();divergence.Reset();reject.Reset();joints.Reset();stock.Reset();leftArm.Reset();weaponFollow.Reset();published.Reset();}
    // Last stage of a frame: the final records (after replay / ramp / limiter) become the reference of the next frame's
    // head clearance.
    void NoteOutput(const Prepared& p,const uint8_t* records,uint32_t count)
    {
        ov::Pose pose;
        reject.headM=(JointLimiter::RecordsInBounds(p,records,count) && Read(p,records,pose)) ? WeaponHeadM(p,pose) : -1.0f;
    }
};
inline Continuity g_continuity;
// FkHook, rifle out and no correction this frame (not eligible): remember the published weapon / left hand for the start of the next episode.
// Read-only (the records are never written), no allocation; the caller holds the ContinuityLock.
inline void ObservePublished(const Prepared& p,const integ::PoseSnapshot& s,const uint8_t* records,uint32_t count,Continuity& c,int64_t ticks)
{
    if(!kStartFromPreviousFrame || ticks<=0 || !records || !Valid(p,s,count))return;
    ov::Pose pose;
    if(!Read(p,records,pose))return;
    StockBody frame;
    if(!StockBodyOf(p,pose,pose,frame))return;
    const auto& sk=p.skeleton;
    const auto wr=native_reload::Model(sk,pose,p.arms.weaponRight);
    const math::Vec3 d=wr.t-frame.neck;
    const math::Vec3 weapon{math::Dot(d,frame.right),math::Dot(d,frame.fwd),math::Dot(d,frame.up)};
    const math::Vec3 hand=math::QuatRotate(math::QuatInv(wr.r),native_reload::Model(sk,pose,p.arms.left.hand).t-wr.t);
    if(!math::IsFinite(weapon) || !math::IsFinite(hand))return;
    c.published.valid=true;c.published.rig=s.rig;c.published.ticks=ticks;c.published.weapon=weapon;c.published.hand=hand;
}
// g_continuity belongs to one animation job at a time (same single-entrant try-flag as aimofs g_busy): the player's pose
// can be evaluated by concurrent jobs, a busy state means this call leaves the records at the baseline.
inline std::atomic_flag g_continuityBusy=ATOMIC_FLAG_INIT;
// A perspective denial ends the reload episode (an old correction must not be replayed after it) but cannot reset a state
// another job owns at that moment. It leaves a request instead; the next owner honours it before it touches the state.
// Mutation 16 drops the request.
inline std::atomic<bool> g_continuityResetRequested{false};
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==16
inline constexpr bool kDeferredReset=false;
#else
inline constexpr bool kDeferredReset=true;
#endif
struct ContinuityLock
{
    bool held;
    ContinuityLock():held(!g_continuityBusy.test_and_set(std::memory_order_acquire))
    {
        if(held && g_continuityResetRequested.exchange(false,std::memory_order_acq_rel))g_continuity.Reset();
    }
    ~ContinuityLock(){if(held)g_continuityBusy.clear(std::memory_order_release);}
    ContinuityLock(const ContinuityLock&)=delete;
    ContinuityLock& operator=(const ContinuityLock&)=delete;
    explicit operator bool() const{return held;}
};
// Perspective denial (FkHook): reset the state now, or leave the request for its current owner's successor.
inline void ResetContinuityOnDenial()
{
    {
        ContinuityLock lock;
        if(lock){g_continuity.Reset();return;}
    }
    if constexpr(kDeferredReset)
    {
        g_continuityResetRequested.store(true,std::memory_order_release);
        ContinuityLock retry; // the owner may have left meanwhile: taking the lock honours the request at once
    }
}
// Invoke after BlendIntoRecords and AO, BEFORE helper evaluation and original FK. On any
// failure the ordinary blended records remain byte-for-byte untouched. A Cycle
// is consumed once even on failure; never carry one across FK, frames or rigs.
inline bool FinishCore(const Prepared& p,const integ::PoseSnapshot& s,
                       uint8_t* records,uint32_t count,bool perspectiveAllowed,
                       Cycle& cycle,native_reload_contacts::Result& result,float correctionWeight,
                       float authoredMinReach,Continuity* continuity,int64_t nowTicks,int64_t ticksPerSecond)
{
    PlacementGate* const placementGate=continuity ? &continuity->placement : nullptr;
    RejectHold* const rejectHold=continuity ? &continuity->reject : nullptr;
    DivergenceState* const divergenceState=continuity ? &continuity->divergence : nullptr;
    const bool captured=cycle.valid;cycle.valid=false;result={};
    if(!captured || !perspectiveAllowed || cycle.records!=records || cycle.count!=count ||
       cycle.sequence!=s.sequence || cycle.rig!=s.rig || !Valid(p,s,count))return false;
    if(!std::isfinite(correctionWeight) || correctionWeight<0 || correctionWeight>1)return false;
    // A new episode starts every follower from the baseline: the correction weight reached 0 (the lag is gone with it).
    const auto resetFollowers=[&]{if(continuity && kResetFollowersAtZero){continuity->leftArm.Reset();continuity->weaponFollow.Reset();}};
    if(correctionWeight<=0){resetFollowers();result.ok=true;result.reject=native_reload_contacts::Reject::None;return true;} // exact baseline; no IK
    ov::Pose authored,candidate,target;
    if(!Read(p,records,authored))return false;
    const ov::Pose baseline=authored; // records as handed in (the masked-bone blend below edits `authored` only)
    // Continuous target: local blend of the actual final baseline towards THIS
    // cycle native pose; Contacts preserves its resulting MODEL hands/weapons.
    // Never blend final solved limbs afterwards: that would break contacts.
    ov::Blend(authored,cycle.native,correctionWeight,target);
    // Native body far from the MM body (jump / slide / crouch leftovers): keep the native hand/weapon placement
    // relative to the torso, not in absolute model space. Healthy recordings never reach lambda > 0.
    float lambda=0.0f;
    {
        const int shoulder=p.skeleton.parent[p.arms.left.upper];
        const int spine3=shoulder>=0 ? p.skeleton.parent[shoulder] : -1;
        const int hips=spine3>=0 ? TopBoneOf(p,spine3) : -1;
        if(hips>=0 && hips!=spine3)
        {
            const float pelvisDistance=gik::Len(
                native_reload::Model(p.skeleton,cycle.native,hips).t-native_reload::Model(p.skeleton,authored,hips).t);
            float divergence;
            if(divergenceState)divergenceState->Step(pelvisDistance,s.rig,nowTicks,ticksPerSecond,lambda,divergence);
            else{lambda=TorsoTransportLambda(pelvisDistance);divergence=TorsoDivergenceGate(pelvisDistance);}
            // Rifle: always the chest frame (see kReloadContactsRifle); pistols keep the divergence-driven choice above.
            if constexpr(kRifleChestTransport)
                if(s.traceOverlay==static_cast<int32_t>(ov::PoseOverlay::Rifle)){lambda=1.0f;divergence=1.0f;}
            if(divergence<1.0f)
            {
                correctionWeight*=divergence;
                if(correctionWeight<=0){resetFollowers();return false;} // gated out: Finish() fades the last correction (RejectHold)
                ov::Blend(authored,cycle.native,correctionWeight,target);
            }
        }
    }
    // Native body in place (lambda == 0): carry the whole native target rigidly onto the MM torso (translation only).
    // Fail open: a skeleton without that chain keeps the absolute native model positions.
    // Ordinary action masking retained native shoulders/arms. Restore ONLY those mask channels from the final authored snapshot
    // (weighted like the correction); lower body and unmasked torso keep the just-blended pose.
    const auto restoreMasked=[&](ov::Pose& a)->bool{
        for(int i=0;i<a.n;++i)if(integ::SnapshotUpperBone(s,uint32_t(i)))
        {
            if(uint32_t(i)<s.firstBone || p.gameIndex[i]<0)return false;
            const float* v=s.pose+size_t(i)*7;
            for(int k=0;k<7;++k)if(!std::isfinite(v[k]))return false;
            ov::Xf endpoint=a.b[i];
            endpoint.t={v[0],v[1],v[2]};endpoint.r={v[3],v[4],v[5],v[6]};
            a.b[i]=ov::BlendXf(a.b[i],endpoint,correctionWeight);
            // Custom clips already own their choreography. Keep the current TPP
            // feminine socket positions through the mask fade, not a second blend
            // towards raw clip offsets which widens the body halfway through.
            if(s.reloadAuthoredClip && (i==p.arms.left.upper || i==p.arms.right.upper))
                a.b[i].t=endpoint.t;
        }
        return true;
    };
    // Rifle: after the chest-frame transport the stock is carried out of the MM body (kRifleStockClearance), rate-limited, then weighted like
    // the correction so the ramp in / out stays continuous; the body is the pose the solver works on (masked shoulder / arm channels restored).
    const bool rifleOverlay=s.traceOverlay==static_cast<int32_t>(ov::PoseOverlay::Rifle);
    float stockShift=0.0f,leftArmShift=0.0f;
    const auto place=[&]()->bool{
        if(lambda>0.0f)
        {
            if(!TransportNativeTorso(p,authored,target,lambda))return false;
            stockShift=0.0f;leftArmShift=0.0f;
            if constexpr(kRifleStockClearance)
            {
                if(rifleOverlay)
                {
                    ov::Pose roots=authored;
                    if(!restoreMasked(roots))return false;
                    // Rate-limited unweighted (place() can run twice in a frame, the second time with a lowered weight), then weighted.
                    math::Vec3 offset=StockClearanceOffset(p,roots,authored,target);
                    if(continuity)
                    {
                        StockBody frame;
                        if(StockBodyOf(p,roots,target,frame))
                        {
                            const math::Vec3 local=continuity->stock.Step(math::Vec3{math::Dot(offset,frame.right),math::Dot(offset,frame.fwd),
                                math::Dot(offset,frame.up)},s.rig,nowTicks,ticksPerSecond);
                            offset=frame.right*local.x+frame.fwd*local.y+frame.up*local.z;
                        }
                    }
                    offset=offset*correctionWeight;
                    const int shoulder=p.skeleton.parent[p.arms.left.upper];
                    const int top=shoulder>=0 && p.skeleton.parent[shoulder]>=0 ? TopBoneOf(p,p.skeleton.parent[shoulder]) : -1;
                    const float len=gik::Len(offset);
                    if(top>=0 && len>0.0f && ShiftNativeTarget(p,target,top,offset))stockShift=len;
                    FollowWeapon(p,roots,kEntryFromPublished ? baseline : roots,target,top,continuity ? &continuity->weaponFollow : nullptr,s.rig,nowTicks,ticksPerSecond,correctionWeight,continuity ? &continuity->published : nullptr);
                    leftArmShift=AdjustLeftArm(p,roots,kEntryFromPublished ? baseline : roots,target,continuity ? &continuity->leftArm : nullptr,s.rig,nowTicks,ticksPerSecond,correctionWeight,
                                                continuity ? &continuity->published : nullptr);
                }
            }
            return true;
        }
        if constexpr(kReloadContactShoulderAnchor>0.0f)
        {
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==21
            AnchorNativeTarget(p,authored,target); // mutation 21: roots before the mask restore (the first version)
#else
            ov::Pose roots=authored; // the pose the solver works on: masked shoulder / arm channels restored
            if(!restoreMasked(roots))return false;
            AnchorNativeTarget(p,roots,target);
#endif
        }
        return true;
    };
    if(!place())return false;
    // Placement guard: shrink the correction (continuously, before the target is built) while the transported native
    // placement would put the weapon inside the head or the left hand under the MM pelvis.
    {
        float placement=PlacementWeight(p,authored,target);
        if(placementGate)placement=placementGate->Step(placement,s.rig,nowTicks,ticksPerSecond);
        if(placement<1.0f)
        {
            correctionWeight*=placement;
            if(correctionWeight<=0){resetFollowers();return false;} // gated out: Finish() fades the last correction (RejectHold)
            ov::Blend(authored,cycle.native,correctionWeight,target);
            if(!place())return false;
        }
    }
    // The solver then restores exact native MODEL contacts, native fingers and both weapon bindings from THIS cycle.
    if(!restoreMasked(authored))return false;
    native_reload_clavicle_reach::Result reachCorrection;
    const float rightGuideCap=s.traceOverlay==static_cast<int32_t>(ov::PoseOverlay::Pistol2H)
        ? kPistolReloadRightGuideTurnCapRad : native_reload::kUnboundedGuideTurnRad;
    if(!native_reload_clavicle_reach::Apply(p.skeleton,p.arms,target,authored,candidate,result,reachCorrection,
                                             s.reloadAuthoredClip ? 20.0f*correctionWeight : 20.0f,
                                             1.0f,native_reload::kUnboundedGuideTurnRad,
                                             rightGuideCap,s.reloadAuthoredClip,
                                             s.reloadAuthoredClip ? authoredMinReach : 0.0f))return false;
    result.stockShift=stockShift;result.leftArmShift=leftArmShift;
    if(rejectHold)
    {
        const float amp=rejectHold->OnAccept(p,baseline,candidate,s.rig,nowTicks,ticksPerSecond,correctionWeight);
        if(amp<1.0f) // coming back from a rejection: ramp the correction in from the baseline
        {
            const ov::Pose solved=candidate;
            float a=amp;
            for(int pass=0;pass<2;++pass)
            {
                for(int k=0;k<rejectHold->n;++k)
                {
                    const int b=rejectHold->bone[k];
                    candidate.b[b].r=math::QuatNlerpShortest(baseline.b[b].r,solved.b[b].r,a);
                    candidate.b[b].t=baseline.b[b].t+(solved.b[b].t-baseline.b[b].t)*a;
                }
                const float clearance=HeadClearance(p,candidate,rejectHold->headM);
                if(pass==1 || clearance>=1.0f)break;
                a*=clearance; // the ramp must not carry the weapon nearer to the head zone than the previous output either
            }
        }
    }
    // All validation precedes the first write. Preserve pos.w and all scale lanes.
    for(int i=0;i<candidate.n;++i)if(p.gameIndex[i]>=0 && uint32_t(i)>=s.firstBone)
    {
        uint8_t* r=records+size_t(p.gameIndex[i])*48;
        const auto& x=candidate.b[i];
        const float t[3]={x.t.x,x.t.y,x.t.z},q[4]={x.r.x,x.r.y,x.r.z,x.r.w};
        std::memcpy(r,t,sizeof(t));std::memcpy(r+16,q,sizeof(q));
    }
    return true;
}
// Public entry (signature unchanged for existing callers; the trailing arguments are optional).
inline bool Finish(const Prepared& p,const integ::PoseSnapshot& s,
                   uint8_t* records,uint32_t count,bool perspectiveAllowed,
                   Cycle& cycle,native_reload_contacts::Result& result,float correctionWeight=1.0f,
                   float authoredMinReach=kAuthoredContactMinReach,
                   Continuity* continuity=nullptr,int64_t nowTicks=0,int64_t ticksPerSecond=0)
{
    // Evaluated before the core consumes the cycle: a frame that never reached the solver (FPP, stale cycle, invalid
    // weight or config) must not be patched from an old correction.
    const bool reachedSolver=cycle.valid && perspectiveAllowed && cycle.records==records && cycle.count==count &&
        cycle.sequence==s.sequence && cycle.rig==s.rig && std::isfinite(correctionWeight) && correctionWeight>0 &&
        correctionWeight<=1 && Valid(p,s,count);
    RejectHold* const rejectHold=continuity ? &continuity->reject : nullptr;
    if(continuity && !perspectiveAllowed)continuity->Reset();
    const bool ok=FinishCore(p,s,records,count,perspectiveAllowed,cycle,result,correctionWeight,authoredMinReach,
                             continuity,nowTicks,ticksPerSecond);
    bool written=ok;
    if(!ok && reachedSolver && rejectHold)written=rejectHold->OnReject(p,s,records,count,nowTicks,ticksPerSecond,correctionWeight);
    if(continuity && reachedSolver)
    {
        continuity->joints.Apply(p,records,count,s.rig,nowTicks,ticksPerSecond,JointSlewForOverlay(s.traceOverlay,correctionWeight));
        continuity->NoteOutput(p,records,count);
    }
    return written;
}
// HoldOnly writes through the same validated snapshot as Finish: a Prepared that does not match THIS snapshot (a remap
// published Prepared{}, a rig change window, a record count that does not cover the mapped bones) must neither be read from
// nor written to. Mutation 14 skips the check.
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==14
inline constexpr bool kHoldOnlyValidates=false;
#else
inline constexpr bool kHoldOnlyValidates=true;
#endif
// Frames where the native cycle could not be captured at all (eligible reload, Capture failed): keep fading the last
// accepted correction instead of dropping it in one frame. Writes nothing when no correction is pending.
inline bool HoldOnly(const Prepared& p,const integ::PoseSnapshot& s,uint8_t* records,uint32_t count,
                     bool perspectiveAllowed,Continuity& state,int64_t nowTicks,int64_t ticksPerSecond)
{
    if(!perspectiveAllowed){state.Reset();return false;}
    if(!std::isfinite(s.reloadContactsWeight) || s.reloadContactsWeight<=0 || s.weight<1-ov::kZeroWeight)return false;
    if constexpr(kHoldOnlyValidates)
        if(!Valid(p,s,count))return false;
    const bool written=state.reject.OnReject(p,s,records,count,nowTicks,ticksPerSecond,s.reloadContactsWeight);
    state.joints.Apply(p,records,count,s.rig,nowTicks,ticksPerSecond,JointSlewForOverlay(s.traceOverlay,s.reloadContactsWeight));
    state.NoteOutput(p,records,count);
    return written;
}
} // namespace utpp::mm::reload_contacts_runtime

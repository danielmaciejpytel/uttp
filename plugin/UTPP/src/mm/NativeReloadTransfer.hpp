#pragma once
// Reload contact helpers: OrientArmPlane / ParallelTransport are the elbow-plane stage of the runtime contact solve
// (native_reload_contacts::Apply, called from reload_contacts_runtime::FinishCore); Apply below is the offline
// weapon-relative transfer candidate (no runtime caller). Native hands follow their measured
// weapon-relative trajectories while the authored thorax/clavicles stay intact.
// No magazine, finger or weapon-prop attachment claim is made here.
#include "GripIK.hpp"

namespace utpp::mm::native_reload
{
inline constexpr float kUnboundedGuideTurnRad=math::kPi;
// Reject non-finite or out-of-domain policy values; a cap of zero means keep
// the transported authored plane, while pi preserves the historical behavior.
inline bool ClampGuideTurn(float turn,float maxGuideTurnRad,float& out)
{
    if(!std::isfinite(turn) || !std::isfinite(maxGuideTurnRad) ||
       maxGuideTurnRad<0.0f || maxGuideTurnRad>kUnboundedGuideTurnRad)return false;
    out=std::clamp(turn,-maxGuideTurnRad,maxGuideTurnRad);
    return true;
}
#ifdef UTPP_RELOAD_DIAG
// Offline diagnostics only (tools/analysis/replay_pose_csv_reload.cpp builds with UTPP_RELOAD_DIAG): one entry per
// OrientArmPlane call. Never compiled into the DLL or the tests.
struct DiagEntry{int upper=-1;float turnDeg=0,projection=0,transportDeg=0,appliedDeg=0;};
struct DiagLog{DiagEntry e[8];int n=0;void Clear(){n=0;}};
inline DiagLog g_diag;
#endif
// A capped guide turn is the angle between the authored and the native elbow plane, wrapped to (-pi, pi]. A hard clamp
// maps +179deg to +cap and -179deg to -cap: the elbow plane flips by 2*cap when the native elbow crosses the far side of
// the arm axis (measured 21.10 / 19.10: right upper-arm local jump 164deg in ONE frame, R19 k2). Up to kSoftGuideTurnStart
// the result is the old clamp (healthy recordings never reach beyond ~135deg); beyond it the turn returns linearly to 0 at
// +-pi, so the output is continuous across the wrap. Unbounded caps (pi) are untouched.
inline constexpr float kSoftGuideTurnStart=math::kPi*0.75f;
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==9
inline constexpr bool kSoftGuideTurn=false; // mutation 9: hard clamp across the wrap
#else
inline constexpr bool kSoftGuideTurn=true;
#endif
inline float SoftClampGuideTurn(float turn,float cap)
{
    const float a=std::fabs(turn);
    float m=std::min(a,cap);
    if constexpr(kSoftGuideTurn)
        if(cap<kUnboundedGuideTurnRad && kSoftGuideTurnStart>cap && a>kSoftGuideTurnStart)
            m=cap*std::max(0.0f,math::kPi-a)/(math::kPi-kSoftGuideTurnStart);
    return std::copysign(m,turn);
}
// ParallelTransport rotates the authored arm by the MINIMAL rotation old-hand-axis -> target-axis and the authored elbow
// direction goes along. Near an antiparallel pair (100-160deg, measured on the 21.10 01:56 and 19.10 reloads) the
// rotation axis cross(old,new) is ill-conditioned: a 10deg baseline motion swings the transported elbow plane by 80deg
// and the capped guide turn cannot follow (right upper arm jumps 134-172deg per frame, R19 k2 / B56 k1 / k2 / k4 / k10).
// From kTransportBlendStartDeg the elbow plane is therefore taken from the NATIVE elbow guide instead of from the
// transported authored plane (fully at kTransportBlendFullDeg): final turn = lerp(capped turn, full turn, w). SolveArm
// derives the upper-arm roll from the final plane only, so nothing else depends on the transport. Below the start
// angle w == 0 and the result is bit-identical to the former one.
inline constexpr float kTransportBlendStartDeg=100.0f;
inline constexpr float kTransportBlendFullDeg=130.0f;
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==10
inline constexpr bool kTransportBlend=false; // mutation 10: plane always from the transported authored elbow
#else
inline constexpr bool kTransportBlend=true;
#endif
inline float TransportPlaneBlend(float transportDeg)
{
    if constexpr(kTransportBlend)
    {
        if(!std::isfinite(transportDeg))return 0.0f;
        const float x=std::clamp((transportDeg-kTransportBlendStartDeg)/(kTransportBlendFullDeg-kTransportBlendStartDeg),0.0f,1.0f);
        return x*x*(3.0f-2.0f*x);
    }
    return 0.0f;
}
// The blend changes the DEGREE of the map turn -> elbow plane: the soft clamp returns to 0 at +-pi (degree 0), the native turn
// is the identity (degree 1). No blend of the two is continuous across the +-pi wrap of `turn` for 0 < w < 1: lerp(Soft(turn),
// turn, w) is +-w*pi on the two sides, i.e. the plane flips by 2*w*pi in one frame when the native elbow crosses the far
// side of the arm axis (measured with a guide sweep: 60deg at w = .17, 175deg at w = .49, 68deg at w = .81). Fading the
// weight with the SAME soft zone the clamp uses makes the result 0 at +-pi for every w, so the map is continuous in turn AND
// in the transport angle. Up to kSoftGuideTurnStart (135deg) the weight is untouched; beyond it a fully weighted native
// plane (transport > 130deg) is pulled back to the clamped one - the price of continuity (a stateless map cannot keep w == 1
// as the identity: that would only move the discontinuity to the transport angle 130deg).
#if defined(UTPP_RELOAD_CONTACT_MUTATION) && UTPP_RELOAD_CONTACT_MUTATION==12
inline constexpr bool kBlendWrapFade=false; // mutation 12: the weight ignores the wrap of the turn
#else
inline constexpr bool kBlendWrapFade=true;
#endif
inline float PlaneBlendWeight(float transportDeg,float turn,float cap)
{
    float w=TransportPlaneBlend(transportDeg);
    if constexpr(kBlendWrapFade)
    {
        const float a=std::fabs(turn);
        if(w>0.0f && cap<kUnboundedGuideTurnRad && kSoftGuideTurnStart>cap && a>kSoftGuideTurnStart)
        {
            const float x=std::clamp((math::kPi-a)/(math::kPi-kSoftGuideTurnStart),0.0f,1.0f);
            w*=x*x*(3.0f-2.0f*x);
        }
    }
    return w;
}
inline ov::Xf Model(const ov::Skeleton& sk, const ov::Pose& p, int bone)
{
    ov::Xf x; gik::ModelOf(sk, p, bone, x.r, x.t); return x;
}
inline ov::Xf Relative(const ov::Xf& parent, const ov::Xf& child)
{
    const auto inv = math::QuatInv(parent.r);
    return {math::QuatNormalize(math::QuatMul(inv, child.r)), math::QuatRotate(inv, child.t-parent.t)};
}
inline ov::Xf Compose(const ov::Xf& a, const ov::Xf& b)
{
    return {math::QuatNormalize(math::QuatMul(a.r,b.r)), a.t+math::QuatRotate(a.r,b.t)};
}
inline ov::Xf Blend(const ov::Xf& a, const ov::Xf& b, float w)
{
    return {math::QuatNlerpShortest(a.r,b.r,w), a.t*(1-w)+b.t*w};
}
inline float Envelope(float elapsed, float duration, float fade=0.20f)
{
    if (!std::isfinite(elapsed) || !std::isfinite(duration) || !(duration>0) || !(fade>0)) return 0;
    auto smooth=[](float t) { t=std::clamp(t,0.0f,1.0f); return t*t*(3-2*t); };
    return smooth(elapsed/fade)*smooth((duration-elapsed)/fade);
}
struct Result
{
    bool ok=false;
    math::Vec3 commonTranslation{};
    float targetContactResidual=0;
    gik::Result left{},right{};
};
// Rotate the whole seeded arm by the minimal old-hand-axis -> target-axis
// rotation before SolveArm. This parallel-transports its perpendicular pole
// without projecting a fixed vector through a singularity. Antiparallel axes
// have no unique transport; reject rather than invent a bend side.
inline bool ParallelTransport(const ov::Skeleton& sk, ov::Pose& pose,
                              const gik::ArmChain& arm, math::Vec3 target)
{
    if (!sk.Valid() || pose.n!=sk.n || !arm.Valid(sk) || !math::IsFinite(target)) return false;
    const auto upper=Model(sk,pose,arm.upper),hand=Model(sk,pose,arm.hand);
    math::Vec3 oldAxis,newAxis;
    if (!gik::Norm(hand.t-upper.t,oldAxis) || !gik::Norm(target-upper.t,newAxis)) return false;
    const float dot=std::clamp(math::Dot(oldAxis,newAxis),-1.0f,1.0f);
    if (!std::isfinite(dot) || dot<=-0.9999f) return false;
    const auto cross=math::Cross(oldAxis,newAxis);
    const auto q=math::QuatNormalize(math::Quat{cross.x,cross.y,cross.z,1+dot});
    math::Quat parent{};
    if (sk.parent[arm.upper]>=0) parent=Model(sk,pose,sk.parent[arm.upper]).r;
    pose.b[arm.upper].r=math::QuatNormalize(math::QuatMul(math::QuatInv(parent),math::QuatMul(q,upper.r)));
    // Lower/hand local rotations remain unchanged: their model rotations are
    // already rotated by Q through the upper bone. SolveArm sets hand target.
    return math::IsFinite(pose.b[arm.upper].r);
}
// Set an explicit model-space elbow guide using only a rigid rotation around
// the target axis. Authored upper/lower local roll is the seed; the native
// elbow guide controls bend SIDE/PLANE, and native hand rotation is imposed
// separately by the one production SolveArm call below. No historical pole,
// previous-frame twist or accumulated transport enters this calculation.
inline bool OrientArmPlane(const ov::Skeleton& sk, ov::Pose& pose,
                           const gik::ArmChain& arm, math::Vec3 target, math::Vec3 guide,
                           float guideAlpha=1.0f,
                           float maxGuideTurnRad=kUnboundedGuideTurnRad)
{
    float diagTransport=0; // angle between the old hand axis and the target axis (deg)
    {
        const auto u0=Model(sk,pose,arm.upper),h0=Model(sk,pose,arm.hand);
        math::Vec3 o,n;
        if (gik::Norm(h0.t-u0.t,o) && gik::Norm(target-u0.t,n))
            diagTransport=std::acos(std::clamp(math::Dot(o,n),-1.0f,1.0f))*57.29578f;
    }
    if (!math::IsFinite(guide) || !std::isfinite(guideAlpha) || guideAlpha<0.0f || guideAlpha>1.0f ||
        !std::isfinite(maxGuideTurnRad) || maxGuideTurnRad<0.0f || maxGuideTurnRad>kUnboundedGuideTurnRad ||
        !ParallelTransport(sk,pose,arm,target)) return false;
    const auto upper=Model(sk,pose,arm.upper),lower=Model(sk,pose,arm.lower);
    math::Vec3 axis,current,desired;
    if (!gik::Norm(target-upper.t,axis)) return false;
    const auto elbow=lower.t-upper.t,guideVector=guide-upper.t;
    const auto projected=guideVector-axis*math::Dot(guideVector,axis);
    // A nearly axial guide cannot identify the bend side. Fixture margins are
    // not universal proof; reject degenerate input instead of guessing a pole.
    const float length=gik::Len(guideVector),projection=gik::Len(projected);
    if (!(length>1e-4f) || !std::isfinite(projection) || projection<0.02f*length ||
        !gik::Norm(elbow-axis*math::Dot(elbow,axis),current) ||
        !gik::Norm(projected,desired)) return false;
    const float turn=std::atan2(math::Dot(axis,math::Cross(current,desired)),
                               std::clamp(math::Dot(current,desired),-1.0f,1.0f));
    float boundedTurn=0.0f;
    if(!ClampGuideTurn(turn*guideAlpha,maxGuideTurnRad,boundedTurn))return false;
    boundedTurn=SoftClampGuideTurn(turn*guideAlpha,maxGuideTurnRad);
    {
        const float w=PlaneBlendWeight(diagTransport,turn*guideAlpha,maxGuideTurnRad);
        if(w>0.0f)boundedTurn+=w*(turn*guideAlpha-boundedTurn); // |turn*alpha - bounded| <= pi: same sign, no wrap
    }
#ifdef UTPP_RELOAD_DIAG
    if (g_diag.n<8)
    {
        auto& d=g_diag.e[g_diag.n++];
        d.upper=arm.upper;d.turnDeg=turn*57.29578f;d.projection=projection/length;d.transportDeg=diagTransport;d.appliedDeg=boundedTurn*57.29578f;
    }
#endif
    const auto q=gik::AxisAngle(axis,boundedTurn);
    math::Quat parent{};
    if (sk.parent[arm.upper]>=0) parent=Model(sk,pose,sk.parent[arm.upper]).r;
    pose.b[arm.upper].r=math::QuatNormalize(math::QuatMul(math::QuatInv(parent),math::QuatMul(q,upper.r)));
    return std::isfinite(turn) && math::IsFinite(pose.b[arm.upper].r);
}
// Caller supplies one stable native pre-action reference and an authored Rifle
// reference in the SAME skeleton; authoring base may move each frame. Atomic
// output: any infeasibility leaves out unchanged. No shoulder/spine edits.
inline bool Apply(const ov::Skeleton& sk, const gik::Rig& rig, int chest,
                  const ov::Pose& nativeRef, const ov::Pose& nativeNow,
                  const ov::Pose& authoredRef, const ov::Pose& authoredNow,
                  float weight, ov::Pose& out, Result& result, float maxShift=0.12f)
{
    result={};
    if (!sk.Valid() || !rig.Valid(sk) || chest<0 || chest>=sk.n ||
        nativeRef.n!=sk.n || nativeNow.n!=sk.n || authoredRef.n!=sk.n || authoredNow.n!=sk.n ||
        !std::isfinite(weight) || weight<0 || weight>1 || !(maxShift>0)) return false;
    for (const ov::Pose* pose : {&nativeRef,&nativeNow,&authoredRef,&authoredNow})
        for (int i=0;i<sk.n;++i)
            if (!math::IsFinite(pose->b[i].t) || !math::IsFinite(pose->b[i].r) ||
                std::fabs(math::QuatDot(pose->b[i].r,pose->b[i].r)-1)>0.001f) return false;
    if (weight==0) { out=authoredNow; result.ok=true; return true; }
    const auto nw=Model(sk,nativeNow,rig.weaponRight);
    const auto nr=Relative(Model(sk,nativeRef,chest),Model(sk,nativeRef,rig.weaponRight));
    const auto nc=Relative(Model(sk,nativeNow,chest),nw);
    const auto ar=Relative(Model(sk,authoredRef,chest),Model(sk,authoredRef,rig.weaponRight));
    auto weapon=Compose(Compose(Model(sk,authoredNow,chest),ar),Relative(nr,nc));
    auto left=Compose(weapon,Relative(nw,Model(sk,nativeNow,rig.left.hand)));
    auto right=Compose(weapon,Relative(nw,Model(sk,nativeNow,rig.right.hand)));
    auto leftGuide=Compose(weapon,Relative(nw,Model(sk,nativeNow,rig.left.lower))).t;
    auto rightGuide=Compose(weapon,Relative(nw,Model(sk,nativeNow,rig.right.lower))).t;
    leftGuide=Model(sk,authoredNow,rig.left.lower).t*(1-weight)+leftGuide*weight;
    rightGuide=Model(sk,authoredNow,rig.right.lower).t*(1-weight)+rightGuide*weight;
    // Transition deliberately blends hand targets as well. Native rigid contact
    // is exact only at weight 1; prop/contact events are not available here.
    weapon=Blend(Model(sk,authoredNow,rig.weaponRight),weapon,weight);
    left=Blend(Model(sk,authoredNow,rig.left.hand),left,weight);
    right=Blend(Model(sk,authoredNow,rig.right.hand),right,weight);
    const auto ls=Model(sk,authoredNow,rig.left.upper).t;
    const auto rs=Model(sk,authoredNow,rig.right.upper).t;
    // Stay below production soft-clamp onset so targets remain actual contacts.
    const float lr=.949f*(gik::Len(authoredNow.b[rig.left.lower].t)+gik::Len(authoredNow.b[rig.left.hand].t));
    const float rr=.949f*(gik::Len(authoredNow.b[rig.right.lower].t)+gik::Len(authoredNow.b[rig.right.hand].t));
    math::Vec3 q{};
    for (int k=0;k<128;++k)
    {
        for (int side=0;side<2;++side)
        {
            const auto c=(side?rs:ls)-(side?right.t:left.t);
            const float radius=side?rr:lr; const auto d=q-c; const float n=gik::Len(d);
            if (!std::isfinite(n) || !(radius>0)) return false;
            if (n>radius) q=c+d*(radius/n);
        }
    }
    if (!math::IsFinite(q) || gik::Len(q)>maxShift || gik::Len(left.t+q-ls)>lr+1e-5f ||
        gik::Len(right.t+q-rs)>rr+1e-5f) return false;
    left.t=left.t+q; right.t=right.t+q; weapon.t=weapon.t+q;
    leftGuide=leftGuide+q;rightGuide=rightGuide+q;
    ov::Pose candidate=authoredNow; gik::Params params;
    if (!OrientArmPlane(sk,candidate,rig.right,right.t,rightGuide) ||
        !OrientArmPlane(sk,candidate,rig.left,left.t,leftGuide)) return false;
    if (!gik::SolveArm(sk,candidate,rig.right,right.t,&right.r,1,params,result.right) ||
        !gik::SolveArm(sk,candidate,rig.left,left.t,&left.r,1,params,result.left)) return false;
    candidate.b[rig.weaponRight]=Relative(Model(sk,candidate,rig.right.hand),weapon);
    result.targetContactResidual=std::max(result.left.handErr,result.right.handErr);
    if (!std::isfinite(result.targetContactResidual) || result.targetContactResidual>.001f) return false;
    result.commonTranslation=q; result.ok=true; out=candidate; return true;
}
} // namespace utpp::mm::native_reload

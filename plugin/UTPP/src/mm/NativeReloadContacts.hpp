#pragma once
// Strict two-hand contact solver. Runtime reload uses the bounded clavicle
// reach preconditioner in NativeReloadClavicleReach before entering this core.
// Preserve the supplied torso/clavicles, native MODEL hands and BOTH weapon
// bindings. No weapon relocation, helper edits, limb stretch or native torso.
#include "NativeReloadTransfer.hpp"
#include <cstring>

namespace utpp::mm::native_reload_contacts
{
enum class Reject { None, Input, Reach, Pole, Solve, Contact, Binding };
struct Result
{
    bool ok=false;
    Reject reject=Reject::Input;
    bool leftPole=false,rightPole=false;
    float contactMax=0,weaponPositionError=0,weaponRotationError=0;
    float stockShift=0; // runtime adapter (rifle): length of the stock clearance shift applied to the native target (m)
    float leftArmShift=0; // runtime adapter (rifle): length of the left arm offset (torso clearance + hand smoothing) applied to the native target (m)
    gik::Result left{},right{};
};
inline bool Descendant(const ov::Skeleton& sk,int bone,int ancestor)
{
    for(int step=0;step<sk.n && bone>=0;++step)
    { if(bone==ancestor)return true;bone=sk.parent[bone]; }
    return false;
}
inline bool Apply(const ov::Skeleton& sk,const gik::Rig& rig,
                  const ov::Pose& nativeNow,const ov::Pose& authoredNow,
                  ov::Pose& out,Result& result,float poleAlpha=1.0f,
                  float maxLeftGuideTurnRad=native_reload::kUnboundedGuideTurnRad,
                  float maxRightGuideTurnRad=native_reload::kUnboundedGuideTurnRad,
                  bool exactAuthoredReach=false,float minReachFraction=0.0f)
{
    result={};
    // Both inputs use THIS single validated named skeleton/rig. No source-rig
    // index inference or numeric count-based reidentification is performed.
    if(!sk.Valid() || rig.weaponLeft>=sk.n || !rig.Valid(sk) || !std::isfinite(poleAlpha) ||
       poleAlpha<0.0f || poleAlpha>1.0f ||
       !std::isfinite(maxLeftGuideTurnRad) || maxLeftGuideTurnRad<0.0f ||
       maxLeftGuideTurnRad>native_reload::kUnboundedGuideTurnRad ||
       !std::isfinite(maxRightGuideTurnRad) || maxRightGuideTurnRad<0.0f ||
       maxRightGuideTurnRad>native_reload::kUnboundedGuideTurnRad ||
       !std::isfinite(minReachFraction) || minReachFraction<0.0f || minReachFraction>0.2f ||
       nativeNow.n!=sk.n || authoredNow.n!=sk.n)return false;
    for(const ov::Pose* p:{&nativeNow,&authoredNow})for(int b=0;b<sk.n;++b)
        if(!math::IsFinite(p->b[b].t) || !math::IsFinite(p->b[b].r) || !math::IsFinite(p->b[b].s) ||
           std::fabs(p->b[b].s.x-1)>0.001f || std::fabs(p->b[b].s.y-1)>0.001f ||
           std::fabs(p->b[b].s.z-1)>0.001f ||
           std::fabs(math::QuatDot(p->b[b].r,p->b[b].r)-1)>0.001f)return false;
    const auto left=native_reload::Model(sk,nativeNow,rig.left.hand);
    const auto right=native_reload::Model(sk,nativeNow,rig.right.hand);
    const auto weaponRight=native_reload::Model(sk,nativeNow,rig.weaponRight);
    ov::Xf weaponLeft{};
    if(rig.weaponLeft>=0)weaponLeft=native_reload::Model(sk,nativeNow,rig.weaponLeft);
    const auto leftGuide=native_reload::Model(sk,nativeNow,rig.left.lower).t;
    const auto rightGuide=native_reload::Model(sk,nativeNow,rig.right.lower).t;
    gik::Params params;
    // Opt-in for offline authored contacts only. Preserve the physical reach
    // guard and a 2mm bend margin; do not introduce soft-clamp contact drift.
    // Existing native runtime callers retain the original parameters exactly.
    if(exactAuthoredReach){params.softStart=1.0f;params.maxReach=1.0f;}
    // 0 keeps the solver default (minReach 0.20). A native FPP reload pose can fold the arm far tighter than that
    // (hand beside the shoulder); the default fold clamp then misses the contact by centimetres and the whole
    // correction is rejected for those frames (recorded 21.10 03:37 k0: 7 frames, forearm pop 69deg).
    if(minReachFraction>0.0f)params.minReach=minReachFraction;
    // Reject geometrically unreachable targets BEFORE changing even scratch.
    // Final production soft/min reach and rotation/contact validation below
    // remain authoritative; no common-shift or clamp-as-success fallback.
    for(const auto* arm:{&rig.left,&rig.right})
    {
        const auto target=arm==&rig.left?left:right;
        const auto shoulder=native_reload::Model(sk,authoredNow,arm->upper).t;
        const float length=gik::Len(authoredNow.b[arm->lower].t)+gik::Len(authoredNow.b[arm->hand].t);
        const float distance=gik::Len(target.t-shoulder);
        if(!(length>1e-4f) || !std::isfinite(distance) || distance>length ||
           (exactAuthoredReach && distance>length-0.002f))
        {result.reject=Reject::Reach;return false;}
    }
    ov::Pose candidate=authoredNow;
    result.rightPole=native_reload::OrientArmPlane(sk,candidate,rig.right,right.t,rightGuide,poleAlpha,maxRightGuideTurnRad);
    result.leftPole=native_reload::OrientArmPlane(sk,candidate,rig.left,left.t,leftGuide,poleAlpha,maxLeftGuideTurnRad);
    if(!result.leftPole || !result.rightPole){result.reject=Reject::Pole;return false;}
    if(!gik::SolveArm(sk,candidate,rig.right,right.t,&right.r,1,params,result.right) ||
       !gik::SolveArm(sk,candidate,rig.left,left.t,&left.r,1,params,result.left))
    {result.reject=Reject::Solve;return false;}
    result.contactMax=std::max(result.left.handErr,result.right.handErr);
    if(!std::isfinite(result.contactMax) || result.contactMax>0.001f)
    {result.reject=Reject::Contact;return false;}
    // Preserve native finger/hand-end local action channels. The explicit rig
    // identifies the two hand roots and weapon bindings; only their descendants
    // are eligible. Runtime caller must supply its CORE pose, not merged helpers.
    for(int b=0;b<sk.n;++b)
        if(b!=rig.left.hand && b!=rig.right.hand && b!=rig.weaponLeft && b!=rig.weaponRight &&
           (Descendant(sk,b,rig.left.hand)||Descendant(sk,b,rig.right.hand)))
            candidate.b[b]=nativeNow.b[b];
    candidate.b[rig.weaponRight]=native_reload::Relative(
        native_reload::Model(sk,candidate,sk.parent[rig.weaponRight]),weaponRight);
    if(rig.weaponLeft>=0)candidate.b[rig.weaponLeft]=native_reload::Relative(
        native_reload::Model(sk,candidate,sk.parent[rig.weaponLeft]),weaponLeft);
    for(int b:{rig.weaponLeft,rig.weaponRight})if(b>=0)
    {
        const auto actual=native_reload::Model(sk,candidate,b);
        const auto expected=b==rig.weaponRight?weaponRight:weaponLeft;
        result.weaponPositionError=std::max(result.weaponPositionError,gik::Len(actual.t-expected.t));
        result.weaponRotationError=std::max(result.weaponRotationError,ov::AngleDeg(actual.r,expected.r));
    }
    if(!std::isfinite(result.weaponPositionError) || !std::isfinite(result.weaponRotationError) ||
       result.weaponPositionError>0.0001f || result.weaponRotationError>0.1f)
    {result.reject=Reject::Binding;return false;}
    result.ok=true;result.reject=Reject::None;out=candidate;return true;
}
} // namespace utpp::mm::native_reload_contacts

#pragma once
// Bounded TPP reload reach preconditioner. It rotates the authored clavicle
// about its fixed proximal origin only as much as needed to keep the current
// native hand target inside the arm's soft-reach sphere, then delegates all
// hand, finger, weapon, and atomic-failure checks to NativeReloadContacts.
#include "NativeReloadContacts.hpp"

namespace utpp::mm::native_reload_clavicle_reach
{
struct Result
{
    float leftDegrees=0,rightDegrees=0;
    float leftShoulderMove=0,rightShoulderMove=0;
};

inline bool Apply(const ov::Skeleton& sk,const gik::Rig& rig,
                  const ov::Pose& nativeNow,const ov::Pose& authoredNow,
                  ov::Pose& out,native_reload_contacts::Result& contacts,
                  Result& result,float limitDegrees=20.0f,float poleAlpha=1.0f,
                  float maxLeftGuideTurnRad=native_reload::kUnboundedGuideTurnRad,
                  float maxRightGuideTurnRad=native_reload::kUnboundedGuideTurnRad,
                  bool exactAuthoredReach=false,float minReachFraction=0.0f)
{
    result={};
    contacts={};
    if(!sk.Valid() || !rig.Valid(sk) || nativeNow.n!=sk.n || authoredNow.n!=sk.n ||
       !std::isfinite(limitDegrees) || limitDegrees<0.0f || limitDegrees>20.0f ||
       !std::isfinite(poleAlpha) || poleAlpha<0.0f || poleAlpha>1.0f ||
       !std::isfinite(maxLeftGuideTurnRad) || maxLeftGuideTurnRad<0.0f ||
       maxLeftGuideTurnRad>native_reload::kUnboundedGuideTurnRad ||
       !std::isfinite(maxRightGuideTurnRad) || maxRightGuideTurnRad<0.0f ||
       maxRightGuideTurnRad>native_reload::kUnboundedGuideTurnRad)return false;

    ov::Pose corrected=authoredNow;
    for(int side=0;side<2;++side)
    {
        const auto& arm=side?rig.right:rig.left;
        const int clavicle=sk.parent[arm.upper];
        if(clavicle<0 || sk.parent[clavicle]<0)return false;
        const auto clavicleModel=native_reload::Model(sk,authoredNow,clavicle);
        const auto shoulderModel=native_reload::Model(sk,authoredNow,arm.upper);
        const auto handModel=native_reload::Model(sk,nativeNow,arm.hand);
        const auto shoulderOffset=shoulderModel.t-clavicleModel.t;
        const auto handOffset=handModel.t-clavicleModel.t;
        const float shoulderRadius=gik::Len(shoulderOffset);
        const float handRadius=gik::Len(handOffset);
        const float armLength=gik::Len(authoredNow.b[arm.lower].t)+gik::Len(authoredNow.b[arm.hand].t);
        const float safeReach=exactAuthoredReach ? armLength-0.002f : 0.949f*armLength;
        if(!(shoulderRadius>1e-4f) || !(handRadius>1e-4f) || !(armLength>1e-4f) ||
           !std::isfinite(shoulderRadius+handRadius+armLength))return false;

        const float cosine=(shoulderRadius*shoulderRadius+handRadius*handRadius-safeReach*safeReach)/
                           (2.0f*shoulderRadius*handRadius);
        if(!std::isfinite(cosine) || cosine>1.0f)return false;
        const float theta=std::acos(std::clamp(math::Dot(shoulderOffset,handOffset)/
                                               (shoulderRadius*handRadius),-1.0f,1.0f));
        const float cap=std::acos(std::clamp(cosine,-1.0f,1.0f));
        const float delta=std::max(0.0f,theta-cap);
        const float degrees=delta*57.2957795131f;
        if(!std::isfinite(degrees) || degrees>limitDegrees)return false;
        if(side)result.rightDegrees=degrees;else result.leftDegrees=degrees;
        if(delta>0.0f)
        {
            math::Vec3 axis;
            if(!gik::Norm(math::Cross(shoulderOffset,handOffset),axis))return false;
            const auto rotation=gik::AxisAngle(axis,delta);
            const auto parentModel=native_reload::Model(sk,authoredNow,sk.parent[clavicle]);
            corrected.b[clavicle].r=math::QuatNormalize(math::QuatMul(
                math::QuatInv(parentModel.r),math::QuatMul(rotation,clavicleModel.r)));
        }
        const auto movedShoulder=native_reload::Model(sk,corrected,arm.upper);
        if(gik::Len(handModel.t-movedShoulder.t)>safeReach+1e-5f)return false;
        const float displacement=gik::Len(movedShoulder.t-shoulderModel.t);
        if(side)result.rightShoulderMove=displacement;else result.leftShoulderMove=displacement;
    }

    ov::Pose candidate;
    if(!native_reload_contacts::Apply(sk,rig,nativeNow,corrected,candidate,contacts,poleAlpha,
                                      maxLeftGuideTurnRad,maxRightGuideTurnRad,exactAuthoredReach,minReachFraction))return false;
    out=candidate;
    return true;
}
} // namespace utpp::mm::native_reload_clavicle_reach

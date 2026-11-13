#pragma once
// Pure target conversion; no runtime caller, pointer discovery or record writes.
// Caller must provide a verified same-frame player<-component mount transform.
#include "NativeReloadContacts.hpp"
#include <string_view>
#include <string>
#include <vector>
namespace utpp::mm::authored_reload
{
inline bool RigidMagazineTransform(const ov::Xf& x)
{
    return math::IsFinite(x.t) && math::IsFinite(x.r) && math::IsFinite(x.s) &&
        std::fabs(math::QuatDot(x.r,x.r)-1)<1e-4f &&
        std::fabs(x.s.x-1)<1e-5f && std::fabs(x.s.y-1)<1e-5f && std::fabs(x.s.z-1)<1e-5f;
}
inline bool PlaceMagazine(const ov::Skeleton& skeleton,const std::vector<std::string>& names,
                         const ov::Pose& current,std::string_view component,
                         const ov::Xf& playerFromComponent,const ov::Xf& targetPlayerModel,
                         bool permitted,ov::Pose& output)
{
    if(!permitted || !skeleton.Valid() || current.n!=skeleton.n || names.size()!=size_t(skeleton.n) ||
       !RigidMagazineTransform(playerFromComponent)||!RigidMagazineTransform(targetPlayerModel))return false;
    const std::string_view leaf=component=="Magazine"?"mag_std":component=="MagazineReload"?"mag_stdr":"";
    if(leaf.empty())return false;
    int target=-1;
    for(int i=0;i<skeleton.n;++i)
    {
        if(!RigidMagazineTransform(current.b[i]))return false;
        if(names[i]==leaf){if(target>=0)return false;target=i;}
    }
    if(target<0 || skeleton.parent[target]<0)return false;
    // The selected skin leaf must not move a sibling or the other magazine.
    for(int i=0;i<skeleton.n;++i)if(skeleton.parent[i]==target)return false;
    const auto targetComponent=native_reload::Relative(playerFromComponent,targetPlayerModel);
    const auto parent=native_reload::Model(skeleton,current,skeleton.parent[target]);
    const auto local=native_reload::Relative(parent,targetComponent);
    if(!RigidMagazineTransform(local))return false;
    auto candidate=current;candidate.b[target]=local;
    // Validate reconstruction before publishing the candidate; output unchanged on failure.
    const auto reconstructed=native_reload::Compose(playerFromComponent,native_reload::Model(skeleton,candidate,target));
    if(gik::Len(reconstructed.t-targetPlayerModel.t)>1e-4f ||
       ov::AngleDeg(reconstructed.r,targetPlayerModel.r)>0.1f)return false;
    output=candidate;return true;
}
}

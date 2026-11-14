#pragma once
// Own pure helper MODEL -> LOCAL/48-byte patch builder. No hook or game writes.
// Caller must separately prove current-cycle LOCAL ownership and perspective.
// Named catalog selects helpers; direct unowned children get compensating LOCAL
// records so their original MODEL transforms (and descendants) stay unchanged.
#include "BodyDeformerOffline.hpp"
#include <cstring>
#include <span>

namespace utpp::mm::helperpatch
{
namespace d=deformoffline;
inline constexpr uint32_t kMaxRecords=1024,kMaxOutputs=128,kRecordFloats=12;
enum class Error { None,Bounds,Catalog,Duplicate,Cycle,ProtectedChild,MissingRecord,InvalidRecord,Target,Roundtrip };
struct Result { Error error=Error::None; int32_t bone=-1; explicit operator bool()const{return error==Error::None;} };
struct Plan
{
    uint32_t count=0,ownedCount=0,outputCount=0;
    std::array<int32_t,kMaxRecords> parents{},order{};
    std::array<uint64_t,kMaxRecords> names{};
    std::array<bool,kMaxRecords> core{},owned{},needed{};
    std::array<uint32_t,kMaxOutputs> outputs{};
    bool valid=false;
};
struct Target { uint64_t name=0;d::Transform model{};int32_t index=-1; }; // index: optional position of `name` in the rig (verified against the name; -1 = look it up)
struct Patch
{
    uint32_t count=0;
    std::array<uint32_t,kMaxOutputs> indices{};
    std::array<std::array<float,kRecordFloats>,kMaxOutputs> before{},after{};
};
struct Scratch
{
    std::array<d::Transform,kMaxRecords> actual{},desired{};
    std::array<bool,kMaxRecords> ready{},targeted{};
    std::array<d::Transform,kMaxOutputs> local{}; // Emit: validated LOCAL of output k, copied to the patch only when all succeeded
};
inline int32_t Find(std::span<const uint64_t> names,uint64_t name)
{
    for(uint32_t i=0;i<names.size();++i)if(names[i]==name)return int32_t(i);
    return -1;
}
// Cold path only. Missing names, cycles and an owned helper with a protected
// direct child invalidate the whole catalog. Numeric bone counts are not IDs.
inline Result Prepare(std::span<const int32_t> parents,std::span<const uint64_t> names,
                      std::span<const uint64_t> core,std::span<const uint64_t> owned,
                      std::span<const uint64_t> compensatable,Plan& out)
{
    if(parents.empty()||parents.size()>kMaxRecords||parents.size()!=names.size()||
       core.empty()||owned.empty()||owned.size()>kMaxOutputs||compensatable.size()>kMaxOutputs)return {Error::Bounds};
    Plan next;next.count=uint32_t(parents.size());
    for(uint32_t i=0;i<next.count;++i)
    {
        if(!names[i]||parents[i]<-1||parents[i]>=int32_t(next.count))return {Error::Catalog,int32_t(i)};
        for(uint32_t j=0;j<i;++j)if(names[j]==names[i])return {Error::Duplicate,int32_t(i)};
        next.parents[i]=parents[i];next.names[i]=names[i];
    }
    for(uint64_t n:core)
    {
        const int32_t i=Find(names,n);
        if(i<0||next.core[i])return {Error::Catalog,i};
        next.core[i]=true;
    }
    for(uint64_t n:owned)
    {
        const int32_t i=Find(names,n);
        if(i<0||next.core[i]||next.owned[i])return {Error::Catalog,i};
        next.owned[i]=true;++next.ownedCount;
    }
    std::array<bool,kMaxRecords> allowedBoundary{};
    for(uint64_t n:compensatable)
    {
        const int32_t i=Find(names,n);
        if(i<0||next.core[i]||next.owned[i]||allowedBoundary[i])return {Error::Catalog,i};
        allowedBoundary[i]=true;
    }
    std::array<uint8_t,kMaxRecords> state{};
    std::array<int32_t,kMaxRecords> path{};
    uint32_t ordered=0;
    for(uint32_t root=0;root<next.count;++root)
    {
        if(state[root])continue;
        uint32_t size=0;int32_t i=int32_t(root);
        while(i>=0&&state[i]==0){state[i]=1;path[size++]=i;i=next.parents[i];}
        if(i>=0&&state[i]==1)return {Error::Cycle,i};
        while(size){i=path[--size];state[i]=2;next.order[ordered++]=i;}
    }
    uint32_t boundaries=0;
    for(uint32_t k=0;k<next.count;++k)
    {
        const uint32_t i=uint32_t(next.order[k]);const int32_t parent=next.parents[i];
        const bool boundary=parent>=0&&next.owned[parent]&&!next.owned[i];
        if(boundary&&next.core[i])return {Error::ProtectedChild,int32_t(i)};
        if(boundary&&!allowedBoundary[i])return {Error::Catalog,int32_t(i)};
        if(boundary)++boundaries;
        if(!next.owned[i]&&!boundary)continue;
        if(next.outputCount>=kMaxOutputs)return {Error::Bounds,int32_t(i)};
        next.outputs[next.outputCount++]=i;
        for(int32_t j=int32_t(i);j>=0;j=next.parents[j])next.needed[j]=true;
    }
    if(boundaries!=compensatable.size())return {Error::Catalog};
    next.valid=true;out=next;return {};
}
// All 12 floats finite (nothing written otherwise), then the transform, Valid and positive scale.
inline bool Decode(const float* raw,d::Transform& out)
{
    namespace s=d::detail::sse;
    const __m128 t=_mm_loadu_ps(raw),q=_mm_loadu_ps(raw+4),sc=_mm_loadu_ps(raw+8);
    if(!s::Finite(t,15)||!s::Finite(q,15)||!s::Finite(sc,15))return false;
    s::Store(out,t,q,sc);
    const float n=s::Dot4(q);
    return d::detail::Finite(n)&&std::fabs(n-1.f)<1e-3f&&out.s.x>1e-6f&&out.s.y>1e-6f&&out.s.z>1e-6f;
}
inline bool Near(const d::Transform& a,const d::Transform& b)
{
    using namespace d::detail;
    const auto delta=Sub(a.t,b.t),scale=Sub(a.s,b.s);
    const auto qdot=[](d::Quat x,d::Quat y){return double(x.x)*y.x+double(x.y)*y.y+double(x.z)*y.z+double(x.w)*y.w;};
    const double denominator=std::sqrt(qdot(a.q,a.q)*qdot(b.q,b.q));
    const double alignment=denominator>0?std::fabs(qdot(a.q,b.q))/denominator:0;
    // cos(0.02 degrees / 2), normalized in double to avoid float-dot rounding.
    return Dot(delta,delta)<1e-8f&&Dot(scale,scale)<1e-8f&&alignment>=0.9999999847691291;
}
// Shared tail of Build / BuildShared: scratch.ready/actual/desired/targeted are filled for every needed record.
// All outputs are validated and their LOCAL values computed into scratch first; `out` is written (and out.count set last) only
// when every output succeeded, so a failure leaves `out` untouched and only `count` entries are ever written.
inline Result Emit(const Plan& plan,std::span<const float> records,Scratch& scratch,Patch& out)
{
    using namespace d::detail;
    const uint32_t count=plan.outputCount;
    for(uint32_t k=0;k<count;++k)
    {
        const uint32_t i=plan.outputs[k];
        if(plan.core[i]||!scratch.ready[i]||(plan.owned[i]&&!scratch.targeted[i]))return {Error::Target,int32_t(i)};
        const int32_t p=plan.parents[i];
        const d::Transform parent=p>=0?scratch.desired[p]:d::Transform{};
        d::Transform roundtrip;
        if(!Relative(parent,scratch.desired[i],scratch.local[k])||!Compose(parent,scratch.local[k],roundtrip)||
           !Near(roundtrip,scratch.desired[i]))return {Error::Roundtrip,int32_t(i)};
    }
    for(uint32_t k=0;k<count;++k)
    {
        const uint32_t i=plan.outputs[k];
        const d::Transform& local=scratch.local[k];
        out.indices[k]=i;
        std::memcpy(out.before[k].data(),records.data()+size_t(i)*kRecordFloats,sizeof(float)*kRecordFloats);
        out.after[k]=out.before[k];
        auto& r=out.after[k];
        r[0]=local.t.x;r[1]=local.t.y;r[2]=local.t.z;
        r[4]=local.q.x;r[5]=local.q.y;r[6]=local.q.z;r[7]=local.q.w;
        r[8]=local.s.x;r[9]=local.s.y;r[10]=local.s.z;
        // r[3] (position.w/tag) and r[11] (scale.w) remain original bytes.
    }
    out.count=count;
    return {};
}
inline Result SetTargets(const Plan& plan,std::span<const Target> targets,Scratch& scratch)
{
    using namespace d::detail;
    const auto names=std::span<const uint64_t>(plan.names.data(),plan.count);
    scratch.ready.fill(false);scratch.targeted.fill(false);
    for(const auto& target:targets)
    {
        const int32_t hint=target.index;
        const int32_t i=(hint>=0&&uint32_t(hint)<plan.count&&names[hint]==target.name)?hint:Find(names,target.name);
        if(i<0||!plan.owned[i]||scratch.targeted[i]||!Valid(target.model)||
           target.model.s.x<=1e-6f||target.model.s.y<=1e-6f||target.model.s.z<=1e-6f)return {Error::Target,i};
        scratch.targeted[i]=true;scratch.desired[i]=target.model;
    }
    return {};
}
// A successful return publishes only a PROPOSED patch, not a write. On failure
// out and input records stay untouched. availability distinguishes uncaptured
// fixture slots; no missing ancestor may be filled from rest/reference values.
inline Result Build(const Plan& plan,std::span<const float> records,
                    std::span<const uint8_t> availability,std::span<const Target> targets,
                    Scratch& scratch,Patch& out)
{
    using namespace d::detail;
    if(!plan.valid||!plan.count||plan.count>kMaxRecords||plan.outputCount>kMaxOutputs||
       records.size()<size_t(plan.count)*kRecordFloats||availability.size()!=plan.count||
       targets.size()!=plan.ownedCount)return {Error::Bounds};
    if(const auto r=SetTargets(plan,targets,scratch);!r)return r;
    for(uint32_t k=0;k<plan.count;++k)
    {
        const int32_t i=plan.order[k];
        if(!plan.needed[i])continue;
        if(!availability[i])return {Error::MissingRecord,i};
        d::Transform local,parent;
        if(!Decode(records.data()+size_t(i)*kRecordFloats,local))return {Error::InvalidRecord,i};
        const int32_t p=plan.parents[i];
        if(p>=0){if(!scratch.ready[p])return {Error::MissingRecord,p};parent=scratch.actual[p];}
        if(!Compose(parent,local,scratch.actual[i]))return {Error::InvalidRecord,i};
        scratch.ready[i]=true;
        if(!plan.owned[i])scratch.desired[i]=scratch.actual[i];
    }
    return Emit(plan,records,scratch,out);
}
// Same result as Build when the caller already composed the actual MODEL transforms of (at least) every needed record
// from these exact records (the candidate pipeline does, for its anchor chain): skips the second decode/compose pass.
inline Result BuildShared(const Plan& plan,std::span<const float> records,std::span<const d::Transform> actualModel,
                          std::span<const bool> actualReady,std::span<const Target> targets,Scratch& scratch,Patch& out)
{
    if(!plan.valid||!plan.count||plan.count>kMaxRecords||plan.outputCount>kMaxOutputs||
       records.size()<size_t(plan.count)*kRecordFloats||actualModel.size()<plan.count||actualReady.size()<plan.count||
       targets.size()!=plan.ownedCount)return {Error::Bounds};
    if(const auto r=SetTargets(plan,targets,scratch);!r)return r;
    for(uint32_t k=0;k<plan.count;++k)
    {
        const int32_t i=plan.order[k];
        if(!plan.needed[i])continue;
        if(!actualReady[i])return {Error::MissingRecord,i};
        scratch.actual[i]=actualModel[i];scratch.ready[i]=true;
        if(!plan.owned[i])scratch.desired[i]=scratch.actual[i];
    }
    return Emit(plan,records,scratch,out);
}
} // namespace utpp::mm::helperpatch

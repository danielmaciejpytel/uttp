// Targeted proposal checks, not a live write/perspective test.
#include "../src/mm/BodyHelperPatch.hpp"
#include <limits>
#include <memory>
#include <vector>
#include <cstdio>
namespace h=utpp::mm::helperpatch;
namespace d=utpp::mm::deformoffline;
namespace
{
// Scalar definitions of the transform algebra (the form the SSE2 versions in BodyDeformerOffline.hpp must reproduce bit for bit).
namespace scalar
{
using namespace d::detail;
bool NormalizeQ(d::Quat a,d::Quat& out){
 float n=Dot(a,a);if(!Finite(n)||n<1e-16f)return false;
 const float e=n-1;float f;
 if(e>-1e-3f&&e<1e-3f)f=1+e*(-.5f+e*.375f);else f=1/std::sqrt(n);
 out={a.x*f,a.y*f,a.z*f,a.w*f};return Finite(out);
}
bool Valid(d::Transform x){float n=Dot(x.q,x.q);return Finite(x.t)&&Finite(x.q)&&Finite(x.s)&&Finite(n)&&std::fabs(n-1.f)<1e-3f;}
bool Compose(d::Transform a,d::Transform b,d::Transform& out){
 if(!NormalizeQ(Product(a.q,b.q),out.q))return false;
 out.t=Add(a.t,Rotate(a.q,Had(a.s,b.t)));out.s=Had(a.s,b.s);return Valid(out);
}
bool Relative(d::Transform a,d::Transform b,d::Transform& out){
 if(std::fabs(a.s.x)<1e-8f||std::fabs(a.s.y)<1e-8f||std::fabs(a.s.z)<1e-8f)return false;
 d::Vec3 r=Rotate(Conj(a.q),Sub(b.t,a.t));out.t={r.x/a.s.x,r.y/a.s.y,r.z/a.s.z};
 if(!NormalizeQ(Product(Conj(a.q),b.q),out.q))return false;
 out.s={b.s.x/a.s.x,b.s.y/a.s.y,b.s.z/a.s.z};return Valid(out);
}
bool Decode(const float* raw,d::Transform& out){
 for(uint32_t k=0;k<h::kRecordFloats;++k)if(!Finite(raw[k]))return false;
 out.t={raw[0],raw[1],raw[2]};out.q={raw[4],raw[5],raw[6],raw[7]};out.s={raw[8],raw[9],raw[10]};
 return Valid(out)&&out.s.x>1e-6f&&out.s.y>1e-6f&&out.s.z>1e-6f;
}
} // namespace scalar
struct Rng
{
    uint64_t s=0x9E3779B97F4A7C15ull;
    uint32_t U(){s=s*6364136223846793005ull+1442695040888963407ull;return uint32_t(s>>33);}
    float F(float lo,float hi){return lo+(hi-lo)*float(U()&0xFFFFFF)/float(0xFFFFFF);}
    // Mostly ordinary values; sometimes the edge cases the guards test (non-finite, zero, tiny, huge, slightly or far off unit).
    float Any(float lo,float hi)
    {
        switch(U()%40)
        {
        case 0:return std::numeric_limits<float>::quiet_NaN();
        case 1:return std::numeric_limits<float>::infinity();
        case 2:return -std::numeric_limits<float>::infinity();
        case 3:return 0.f;
        case 4:return -0.f;
        case 5:return 1e-9f;
        case 6:return 3e38f;
        case 7:return std::numeric_limits<float>::denorm_min();
        default:return F(lo,hi);
        }
    }
    d::Transform T(bool edge)
    {
        d::Transform x;
        x.t={edge?Any(-2,2):F(-2,2),edge?Any(-2,2):F(-2,2),edge?Any(-2,2):F(-2,2)};
        d::Quat q{F(-1,1),F(-1,1),F(-1,1),F(-1,1)};
        const float n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        const float off=(U()%4==0)?F(-2e-3f,2e-3f):0.f;
        if(n>1e-3f)q={q.x/n*(1+off),q.y/n,q.z/n,q.w/n};
        if(edge&&U()%8==0)q.x=Any(-1,1);
        if(edge&&U()%16==0)q={0,0,0,0};
        x.q=q;
        x.s={edge?Any(.2f,2):F(.5f,1.5f),edge?Any(.2f,2):F(.5f,1.5f),edge?Any(.2f,2):F(.5f,1.5f)};
        return x;
    }
};
} // namespace
int main()
{
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    const std::array<int32_t,4> parents{-1,0,1,2};
    const std::array<uint64_t,4> names{11,22,33,44};
    const std::array<uint64_t,2> core{11,44};
    const std::array<uint64_t,1> owned{22};
    const std::array<uint64_t,1> compensatable{33};
    auto plan=std::make_unique<h::Plan>();
    check(bool(h::Prepare(parents,names,core,owned,compensatable,*plan)),"named catalog including protected grandchild");
    check(plan->ownedCount==1&&plan->outputCount==2,"one owned plus one compensating boundary");
    std::vector<float> records(4*12);
    for(int i=0;i<4;++i){records[i*12+3]=31.25f;records[i*12+7]=1;records[i*12+8]=records[i*12+9]=records[i*12+10]=1;records[i*12+11]=17.5f;}
    records[12]=1;records[24]=.5f;records[36]=.25f;
    const auto before=records;
    std::array<uint8_t,4> available{1,1,1,1};
    h::Target target;target.name=22;target.model.t={2,0,0};
    const float q=std::sqrt(.5f);target.model.q={0,0,q,q};
    auto scratch=std::make_unique<h::Scratch>();auto patch=std::make_unique<h::Patch>();
    check(bool(h::Build(*plan,records,available,std::span(&target,1),*scratch,*patch)),"48B proposal built");
    check(std::memcmp(records.data(),before.data(),records.size()*4)==0,"Build leaves input unchanged");
    auto after=records;
    for(uint32_t k=0;k<patch->count;++k)
    {
        check(patch->indices[k]==1||patch->indices[k]==2,"only named helper or boundary");
        check(patch->before[k][3]==patch->after[k][3]&&patch->before[k][11]==patch->after[k][11],"original W sentinel lanes retained");
        std::memcpy(after.data()+patch->indices[k]*12,patch->after[k].data(),48);
    }
    std::array<d::Transform,4> model;
    for(int i=0;i<4;++i){d::Transform local,parent;check(h::Decode(after.data()+i*12,local),"decode patched record");if(parents[i]>=0)parent=model[parents[i]];check(d::detail::Compose(parent,local,model[i]),"patched FK");}
    check(h::Near(model[1],target.model),"owned MODEL target reached");
    d::Transform grandchild;grandchild.t={1.75f,0,0};
    check(h::Near(model[3],grandchild),"protected descendant MODEL retained by boundary compensation");
    check(std::memcmp(after.data()+36,before.data()+36,48)==0,"protected descendant LOCAL bytes unchanged");
    auto patchBefore=std::make_unique<h::Patch>(*patch);
    available[2]=0;
    check(!h::Build(*plan,records,available,std::span(&target,1),*scratch,*patch),"missing boundary record fails");
    check(std::memcmp(patch.get(),patchBefore.get(),sizeof(h::Patch))==0,"failed build preserves prior output atomically");
    available[2]=1;
    records[4]=std::numeric_limits<float>::quiet_NaN();
    check(!h::Build(*plan,records,available,std::span(&target,1),*scratch,*patch),"nonfinite source fails");records=before;
    target.model.q={};target.name=11;
    check(!h::Build(*plan,records,available,std::span(&target,1),*scratch,*patch),"physical core target rejected");
    auto cyclic=parents;cyclic[0]=3;
    check(!h::Prepare(cyclic,names,core,owned,compensatable,*plan),"parent cycle rejected");
    auto duplicate=names;duplicate[1]=duplicate[0];
    check(!h::Prepare(parents,duplicate,core,owned,compensatable,*plan),"duplicate name hash rejected");
    const std::array<uint64_t,2> protectedChild{11,33};
    check(!h::Prepare(parents,names,protectedChild,owned,compensatable,*plan),"direct protected child rejects ownership catalog");
    check(!h::Prepare(parents,names,core,owned,{},*plan),"unexpected unowned child cannot be added to write set");
    check(plan->valid&&plan->outputCount==2,"failed cold preparation preserves prior immutable plan");
    check(!h::Build(*plan,std::span<const float>(records.data(),12),available,std::span(&target,1),*scratch,*patch),"short record buffer rejected");
    // Regrouped helper: owned helper 55 hangs under group 66 (its usual parent is core bone 11). Its LOCAL is relative to 11 whatever
    // the group holds, and is written into its own record; without the regrouping (mutation) the group's rotation enters the LOCAL.
    {
        const std::array<int32_t,3> gp{-1,2,0};const std::array<uint64_t,3> gn{11,55,66};
        const std::array<uint64_t,1> gcore{11},gowned{55};const std::array<std::pair<uint64_t,uint64_t>,1> regroup{{{55,66}}};
        auto gplan=std::make_unique<h::Plan>(),plain=std::make_unique<h::Plan>();
        check(bool(h::Prepare(gp,gn,gcore,gowned,{},*gplan,regroup)),"regrouped helper: plan prepared");
        check(bool(h::Prepare(gp,gn,gcore,gowned,{},*plain)),"mutation control: plan without the regrouping");
        const std::array<std::pair<uint64_t,uint64_t>,1> wrongGroup{{{55,11}}};
        check(!h::Prepare(gp,gn,gcore,gowned,{},*plain,wrongGroup),"a core bone is never a group");
        const std::array<std::pair<uint64_t,uint64_t>,1> notParent{{{55,77}}};
        check(!h::Prepare(gp,gn,gcore,gowned,{},*plain,notParent),"a group that is not the helper's parent is refused");
        check(bool(h::Prepare(gp,gn,gcore,gowned,{},*plain)),"mutation control: plan without the regrouping (again)");
        std::vector<float> grec(3*12,0.f);
        for(int i=0;i<3;++i){grec[i*12+7]=1;grec[i*12+8]=grec[i*12+9]=grec[i*12+10]=1;}
        grec[0]=0.5f;                                                       // core 11 at x = 0.5
        const float s=std::sin(0.3f),c=std::cos(0.3f);grec[24+6]=s;grec[24+7]=c;grec[24+1]=0.2f; // group 66: 34 deg about z, y = 0.2
        const std::array<uint8_t,3> gav{1,1,1};
        h::Target gt;gt.name=55;gt.model.t={0.5f,0.1f,0};gt.model.q={0,0,0,1};
        auto gscratch=std::make_unique<h::Scratch>();auto gpatch=std::make_unique<h::Patch>(),ppatch=std::make_unique<h::Patch>();
        check(bool(h::Build(*gplan,grec,gav,std::span(&gt,1),*gscratch,*gpatch))&&gpatch->count==1&&gpatch->indices[0]==1,"regrouped helper: only its own record is written");
        const auto& r=gpatch->after[0];
        check(std::fabs(r[0])<1e-6f&&std::fabs(r[1]-0.1f)<1e-6f&&std::fabs(r[2])<1e-6f&&std::fabs(std::fabs(r[7])-1.f)<1e-6f,
              "regrouped helper: LOCAL relative to the usual parent (the group record is ignored)");
        check(bool(h::Build(*plain,grec,gav,std::span(&gt,1),*gscratch,*ppatch))&&std::fabs(std::fabs(ppatch->after[0][7])-c)<1e-5f,
              "mutation: without the regrouping the LOCAL is relative to the group");
    }
    // The SSE2 Compose / Relative / Valid / Decode equal the scalar definitions bit for bit: same return value and the same bytes in
    // the output (both outputs start from the same filler, so a write the scalar form does not make also shows), on ordinary and edge inputs.
    // The only allowed difference is the sign / payload of a NaN, which appears only in the output of a failed call on non-finite input.
    {
        Rng rng;uint32_t composeBad=0,relativeBad=0,validBad=0,decodeBad=0,okCompose=0,okRelative=0,okDecode=0;
        const auto fill=[](d::Transform& x){std::memset(&x,0x5A,sizeof x);};
        const auto same=[](const d::Transform& x,const d::Transform& y)
        {
            const float* a=&x.t.x;const float* b=&y.t.x;
            for(int k=0;k<10;++k)if(std::memcmp(a+k,b+k,sizeof(float))!=0&&!(a[k]!=a[k]&&b[k]!=b[k]))return false;
            return true;
        };
        for(int i=0;i<200000;++i)
        {
            const bool edge=i%4==0;
            const d::Transform a=rng.T(edge),b=rng.T(edge);
            d::Transform s1,v1;fill(s1);fill(v1);
            const bool sc=scalar::Compose(a,b,s1),vc=d::detail::Compose(a,b,v1);
            if(sc!=vc||!same(s1,v1)||(sc&&std::memcmp(&s1,&v1,sizeof s1)!=0))++composeBad;
            okCompose+=sc;
            fill(s1);fill(v1);
            const bool sr=scalar::Relative(a,b,s1),vr=d::detail::Relative(a,b,v1);
            if(sr!=vr||!same(s1,v1)||(sr&&std::memcmp(&s1,&v1,sizeof s1)!=0))++relativeBad;
            okRelative+=sr;
            if(scalar::Valid(a)!=d::detail::Valid(a))++validBad;
            std::array<float,h::kRecordFloats> raw{};
            for(auto& f:raw)f=rng.F(-1,1);
            raw[0]=a.t.x;raw[1]=a.t.y;raw[2]=a.t.z;raw[4]=a.q.x;raw[5]=a.q.y;raw[6]=a.q.z;raw[7]=a.q.w;raw[8]=a.s.x;raw[9]=a.s.y;raw[10]=a.s.z;
            if(edge&&rng.U()%8==0)raw[rng.U()%12]=rng.Any(-1,1);
            fill(s1);fill(v1);
            const bool sd=scalar::Decode(raw.data(),s1),vd=h::Decode(raw.data(),v1);
            if(sd!=vd||!same(s1,v1)||(sd&&std::memcmp(&s1,&v1,sizeof s1)!=0))++decodeBad;
            okDecode+=sd;
        }
        std::printf("transform algebra: %u / %u / %u successful Compose / Relative / Decode of 200000; mismatches %u %u %u, Valid %u\n",
                    okCompose,okRelative,okDecode,composeBad,relativeBad,decodeBad,validBad);
        check(composeBad==0&&okCompose>100000,"SSE2 Compose equals the scalar definition bit for bit (result and written bytes)");
        check(relativeBad==0&&okRelative>100000,"SSE2 Relative equals the scalar definition bit for bit (result and written bytes)");
        check(validBad==0,"SSE2 Valid equals the scalar definition");
        check(decodeBad==0&&okDecode>100000,"SSE2 Decode equals the scalar definition bit for bit (result and written bytes)");
    }
    std::printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}

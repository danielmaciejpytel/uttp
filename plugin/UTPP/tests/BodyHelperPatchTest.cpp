// Targeted proposal checks, not a live write/perspective test.
#include "../src/mm/BodyHelperPatch.hpp"
#include <limits>
#include <memory>
#include <vector>
#include <cstdio>
namespace h=utpp::mm::helperpatch;
namespace d=utpp::mm::deformoffline;
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
    std::printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}

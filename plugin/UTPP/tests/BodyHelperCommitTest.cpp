// Targeted atomic-commit checks; does not call game hooks or modify live game memory.
#include "../src/mm/BodyHelperCommit.hpp"
#include <array>
#include <bit>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace h=utpp::mm::helperpatch;
namespace c=utpp::mm::bodyhelpercommit;

struct Gate { bool allow=true; int calls=0; };
static bool Permission(void* p) noexcept
{
    auto& gate=*static_cast<Gate*>(p); ++gate.calls; return gate.allow;
}

struct Fixture
{
    h::Plan plan{};
    h::Patch patch{};
    std::vector<float> live;
    Fixture():live(40*h::kRecordFloats)
    {
        plan.valid=true; plan.count=40; plan.outputCount=c::kRequiredOutputs;
        for(uint32_t i=0;i<c::kRequiredOutputs;++i)
        {
            plan.outputs[i]=i;
            float* r=live.data()+size_t(i)*h::kRecordFloats;
            r[0]=float(i);r[1]=2;r[2]=3;
            r[3]=std::bit_cast<float>(uint32_t{0x7fc01234}); // opaque W may be NaN
            r[4]=r[5]=r[6]=0;r[7]=1;
            r[8]=r[9]=r[10]=1;
            r[11]=std::bit_cast<float>(uint32_t{0x7fc05678});
            patch.indices[i]=i;
            std::memcpy(patch.before[i].data(),r,sizeof(float)*h::kRecordFloats);
            patch.after[i]=patch.before[i];
            patch.after[i][0]+=0.25f;
        }
        patch.count=c::kRequiredOutputs;
    }
};

int main()
{
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    auto unchanged=[&](const Fixture& f,const std::vector<float>& before,const char* label)
        {check(std::memcmp(f.live.data(),before.data(),before.size()*sizeof(float))==0,label);};

    {
        Fixture f; const auto before=f.live; Gate gate;
        const auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(bool(result),"complete authorized 35-record patch commits");
        check(gate.calls==1,"permission checked exactly once on success");
        for(uint32_t i=0;i<f.plan.count;++i)
        {
            const float expected=i<c::kRequiredOutputs?f.patch.after[i][0]:before[size_t(i)*12];
            check(f.live[size_t(i)*12]==expected,"writes only exact approved outputs");
        }
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.live[2*12]+=1;
        before=f.live;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::BeforeChanged,"stale before bytes rejected");
        check(gate.calls==0,"stale patch rejected before permission callback");
        unchanged(f,before,"stale rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.indices[3]=f.patch.indices[2];
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::DuplicateIndex,"duplicate patch index rejected");
        unchanged(f,before,"duplicate rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.indices[0]=40;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::ApprovedIndex,"out-of-range patch index rejected");
        unchanged(f,before,"out-of-range rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.indices[0]=35;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::ApprovedIndex,"in-range but unapproved extra output rejected");
        unchanged(f,before,"extra-output rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.plan.outputs[1]=f.plan.outputs[0];
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::DuplicateIndex,"duplicate approved plan output rejected");
        unchanged(f,before,"duplicate plan output rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        --f.patch.count;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::PatchCount,"missing output rejected");
        unchanged(f,before,"missing output rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.after[4][0]=std::numeric_limits<float>::quiet_NaN();
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::InvalidAfter,"nonfinite TRS rejected");
        unchanged(f,before,"nonfinite rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.after[4][3]=0;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::WChanged,"changed opaque W lane rejected bitwise");
        unchanged(f,before,"W-lane rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        f.plan.core[4]=true;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::ProtectedCore,"protected core output rejected");
        unchanged(f,before,"protected-core rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate; gate.allow=false;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::PermissionDenied,"final permission revocation rejects");
        check(gate.calls==1,"revoked permission checked once");
        unchanged(f,before,"permission rejection mutates nothing");
    }
    {
        Fixture f; auto before=f.live; Gate gate;
        std::swap(f.patch.indices[34],f.patch.indices[33]);
        std::swap(f.patch.before[34],f.patch.before[33]);
        std::swap(f.patch.after[34],f.patch.after[33]);
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(bool(result),"permuted approved output set accepted");
        check(gate.calls==1,"permuted set receives final permission check");
        check(f.live[33*12]==before[33*12]+0.25f,"permuted output written");
    }
    // translation safety net: one helper moving further than the cap rejects the WHOLE patch before the permission callback
    {
        Fixture f; auto before=f.live; Gate gate;
        f.patch.after[7][1]+=c::kMaxTranslationDelta*1.01f;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::TranslationJump&&result.index==7,"one helper beyond the translation cap rejects the whole patch");
        check(gate.calls==0,"translation-cap rejection happens before the permission callback");
        unchanged(f,before,"translation-cap rejection mutates nothing");
    }
    {
        // the cap is on the 3D distance, not per axis: three axis deltas each below the cap but together above it
        Fixture f; auto before=f.live; Gate gate;
        const float each=c::kMaxTranslationDelta*0.7f;
        f.patch.after[9][0]=f.patch.before[9][0]+each;f.patch.after[9][1]=f.patch.before[9][1]+each;f.patch.after[9][2]=f.patch.before[9][2]+each;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(result.error==c::Error::TranslationJump,"translation cap is a distance (3 axes combined)");
        unchanged(f,before,"distance-cap rejection mutates nothing");
    }
    {
        Fixture f; Gate gate;
        f.patch.after[7][0]=f.patch.before[7][0]+c::kMaxTranslationDelta*0.9f;
        auto result=c::Commit(f.plan,f.live,f.patch,Permission,&gate);
        check(bool(result),"a change just inside the translation cap still commits");
    }
    std::printf("checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}

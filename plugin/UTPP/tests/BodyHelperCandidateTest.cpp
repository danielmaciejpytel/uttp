// Offline proposal pipeline only: it does not apply proposed records to a game pose.
#include "../src/mm/BodyHelperCandidate.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace c=utpp::mm::bodyhelpercandidate;
namespace d=utpp::mm::deformoffline;
namespace h=utpp::mm::helperpatch;

int main()
{
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    auto program=std::make_unique<d::Program>();
    program->nodeCount=35;program->opCount=1;
    std::array<uint64_t,35> programNames{};
    programNames[0]=1000;program->nodes[0].kind=d::NodeKind::Core;program->nodes[0].parent=-1;
    for(uint32_t i=0;i<c::kOwnedHelpers;++i)
    {
        const uint32_t n=i+1;programNames[n]=2000+i;
        program->nodes[n].kind=d::NodeKind::Reference;program->nodes[n].parent=0;
        program->nodes[n].refLocal.t={float(i+1),.25f,0};
    }
    programNames[34]=9999;program->nodes[34].kind=d::NodeKind::Logical;program->nodes[34].parent=-1;
    program->ops[0].kind=d::OpKind::Init;program->ops[0].target=34;program->ops[0].a=0;

    std::array<int32_t,37> parents{};parents.fill(0);parents[0]=-1;
    parents[34]=2;parents[35]=3;
    parents[36]=-1; // protected physical-only bone; not a program input or needed ancestor
    std::array<uint64_t,37> names{};names[0]=1000;
    std::array<uint64_t,c::kOwnedHelpers> owned{};
    std::array<c::HelperOutput,c::kOwnedHelpers> outputs{};
    for(uint32_t i=0;i<c::kOwnedHelpers;++i)
    { names[i+1]=2000+i;owned[i]=2000+i;outputs[i]={2000+i,i+1}; }
    names[34]=3000;names[35]=3001;names[36]=4000;
    const std::array<uint64_t,1> core{1000};
    const std::array<uint64_t,2> physicalCore{1000,4000};
    const std::array<c::CoreLocal,1> coreLocals{{{1000,{5,0,0},true}}};
    const std::array<uint64_t,1> protectedCore{4000};
    const std::array<uint64_t,2> children{3000,3001};
    auto plan=std::make_unique<c::Plan>();
    check(bool(c::Prepare(*program,programNames,parents,names,core,physicalCore,coreLocals,
        protectedCore,owned,children,outputs,1000,*plan)),"cold named program and merged catalogs prepare");
    check(plan->helperPlan.outputCount==35&&plan->outputCount==33,"33 helper targets plus two protected child compensations");

    std::vector<float> records(names.size()*h::kRecordFloats);
    for(uint32_t i=0;i<names.size();++i)
    {
        float* r=records.data()+size_t(i)*h::kRecordFloats;
        r[0]=i==0?10.f:.2f;r[1]=0;r[2]=0;r[3]=71.f;
        r[4]=r[5]=r[6]=0;r[7]=1;r[8]=r[9]=r[10]=1;r[11]=19.f;
    }
    const auto original=records;
    std::array<uint8_t,37> available{};available.fill(1);
    auto scratch=std::make_unique<c::Scratch>();auto patch=std::make_unique<h::Patch>();
    check(bool(c::Build(*plan,records,available,*scratch,*patch)),"hot candidate pipeline produces patch");
    check(scratch->inputs.present[0]&&scratch->inputs.model[0].t.x==5.f,
          "evaluator gets program-hierarchy virtual Core model with local translation override");
    check(scratch->actualMergedModel[0].t.x==10.f,
          "actual transfer anchor is reconstructed from merged LOCAL hierarchy");
    check(std::memcmp(records.data(),original.data(),records.size()*sizeof(float))==0,
          "candidate generation leaves actual input records unchanged");
    check(patch->count==35,"proposed patch covers only 33 helpers plus two compensatable boundaries");
    for(uint32_t i=0;i<patch->count;++i)
    {
        check(patch->before[i][3]==patch->after[i][3]&&patch->before[i][11]==patch->after[i][11],
              "proposed helper patch preserves sentinel W lanes");
    }
    available[36]=0;
    check(bool(c::Build(*plan,records,available,*scratch,*patch)),"unavailable protected physical-only core is not an input dependency");
    available[36]=1;
    auto priorPatch=std::make_unique<h::Patch>(*patch);
    available[0]=0;
    check(!c::Build(*plan,records,available,*scratch,*patch),"missing required actual core fails closed");
    check(std::memcmp(patch.get(),priorPatch.get(),sizeof(h::Patch))==0,"failed candidate preserves output atomically");
    available[0]=1;

    const std::array<c::CoreLocal,1> forbidden{{{1000,{0,0,0},true}}};
    auto unchangedPlan=std::make_unique<c::Plan>(*plan);
    const std::array<uint64_t,1> protectedSpine{1000};
    check(!c::Prepare(*program,programNames,parents,names,core,physicalCore,forbidden,
        protectedSpine,owned,children,outputs,1000,*plan),"protected root/hips/trajectory-style core override rejected");
    check(plan->valid&&plan->outputCount==unchangedPlan->outputCount,"failed Prepare preserves previous plan");
    auto badOutputs=outputs;badOutputs[0].programIndex=0;
    check(!c::Prepare(*program,programNames,parents,names,core,physicalCore,coreLocals,
        protectedCore,owned,children,badOutputs,1000,*plan),"core node cannot masquerade as helper output");

    std::printf("checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}

#pragma once
// Pure offline bridge from a private body-deformer program to named merged-rig
// helper targets. It proposes a helperpatch::Patch only; it has no file I/O,
// hooks, engine references, or pose writes.
#include "BodyHelperPatch.hpp"
#include <span>

namespace utpp::mm::bodyhelpercandidate
{
namespace d = deformoffline;
namespace h = helperpatch;
inline constexpr uint32_t kMaxProgramNodes=d::kMaxNodes;
inline constexpr uint32_t kMaxMergedBones=h::kMaxRecords;
// Legacy exact count of the first (woman) bundle schema; a bundle now declares its own owned count (<= kMaxOwned).
inline constexpr uint32_t kOwnedHelpers=33;
inline constexpr uint32_t kMaxOwned=h::kMaxOutputs;

enum class Error : uint32_t
{
    None, Bounds, Program, Names, Duplicate, CoreMapping, ProtectedOverride,
    Anchor, OutputMapping, Catalog, MissingRecord, InvalidRecord,
    Transform, Evaluation, HelperPatch
};
struct Result
{
    Error error=Error::None;
    uint32_t index=0;
    explicit operator bool() const { return error==Error::None; }
};

// One record for every program Core node. When replaceTranslation is true,
// translation is an authored *local translation* in the source program
// hierarchy. Rotation and scale always come from the current merged LOCAL.
struct CoreLocal
{
    uint64_t nameHash=0;
    d::Vec3 translation{};
    bool replaceTranslation=false;
};
struct HelperOutput { uint64_t nameHash=0; uint32_t programIndex=0; };

struct Plan
{
    const d::Program* program=nullptr; // immutable; caller keeps bundle alive
    uint32_t mergedCount=0,coreCount=0,spine3Merged=~uint32_t{0},spine3Program=~uint32_t{0};
    uint32_t outputCount=0;
    h::Plan helperPlan{};
    std::array<int32_t,kMaxMergedBones> mergedParents{},mergedOrder{},programCoreMerged{};
    std::array<uint64_t,kMaxMergedBones> mergedNames{};
    std::array<bool,kMaxMergedBones> mergedNeeded{},mergedCore{};
    std::array<bool,kMaxProgramNodes> virtualNeeded{}; // program nodes whose virtual MODEL Build needs: the mapped core nodes and their ancestors
    std::array<uint64_t,kMaxProgramNodes> programNames{};
    std::array<CoreLocal,kMaxProgramNodes> coreLocals{};
    std::array<uint32_t,kMaxOwned> outputProgram{};
    std::array<uint64_t,kMaxOwned> outputNames{};
    std::array<int32_t,kMaxOwned> outputMerged{}; // merged-rig index of each output (-1 = not found; Build then looks the name up)
    bool valid=false;
};

struct Scratch
{
    std::array<d::Transform,kMaxMergedBones> actualMergedModel{};
    std::array<bool,kMaxMergedBones> actualReady{};
    std::array<d::Transform,kMaxProgramNodes> virtualProgramModel{};
    std::array<bool,kMaxProgramNodes> virtualReady{};
    d::Inputs inputs{};
    d::Scratch evaluator{};
    h::Scratch helper{};
    std::array<h::Target,kMaxOwned> targets{}; // only [0, outputCount) is written and read per Build
};

inline int32_t Find(std::span<const uint64_t> names,uint64_t value)
{
    for(uint32_t i=0;i<names.size();++i)if(names[i]==value)return int32_t(i);
    return -1;
}

// Cold preparation. `programNames` has one stable name hash per Program node;
// programCoreHashes must enumerate exactly the program Core nodes. The larger
// physicalCoreHashes catalog protects all actual core bones from helper writes;
// translationProtectedHashes (Root/Hips/Trajectory) may never be overridden.
// `outputs` must enumerate the 33 owned helper outputs, with the matching
// program node index. Program storage must outlive Plan and remain immutable.
// `regrouped`, `groups`: owned helpers that hang under a group bone in this rig, and those groups (helperpatch::Prepare).
inline Result Prepare(const d::Program& program,
    std::span<const uint64_t> programNames,
    std::span<const int32_t> mergedParents,
    std::span<const uint64_t> mergedNames,
    std::span<const uint64_t> programCoreHashes,
    std::span<const uint64_t> physicalCoreHashes,
    std::span<const CoreLocal> coreLocals,
    std::span<const uint64_t> translationProtectedHashes,
    std::span<const uint64_t> ownedHashes,
    std::span<const uint64_t> compensatableHashes,
    std::span<const HelperOutput> outputs,
    uint64_t spine3Hash, Plan& out,
    std::span<const std::pair<uint64_t,uint64_t>> regrouped={},std::span<const uint64_t> groups={})
{
    if(program.nodeCount==0||program.nodeCount>kMaxProgramNodes||
       programNames.size()!=program.nodeCount||mergedNames.empty()||
       mergedNames.size()>kMaxMergedBones||mergedParents.size()!=mergedNames.size()||
       programCoreHashes.empty()||programCoreHashes.size()>kMaxMergedBones||
       physicalCoreHashes.empty()||physicalCoreHashes.size()>kMaxMergedBones||
       coreLocals.size()!=programCoreHashes.size()||ownedHashes.empty()||ownedHashes.size()>kMaxOwned||
       outputs.size()!=ownedHashes.size()||!spine3Hash)
        return {Error::Bounds};
    if(!d::Validate(program))return {Error::Program};

    Plan next; next.program=&program; next.mergedCount=uint32_t(mergedNames.size());
    next.coreCount=uint32_t(programCoreHashes.size()); next.outputCount=uint32_t(ownedHashes.size());
    std::array<int32_t,kMaxProgramNodes> programCoreMerged{};
    programCoreMerged.fill(-1);
    std::array<CoreLocal,kMaxProgramNodes> localByProgram{};
    std::array<bool,kMaxProgramNodes> hasCoreLocal{};

    for(uint32_t i=0;i<program.nodeCount;++i)
    {
        if(!programNames[i])return {Error::Names,i};
        for(uint32_t j=0;j<i;++j)if(programNames[j]==programNames[i])return {Error::Duplicate,i};
        next.programNames[i]=programNames[i];
    }
    for(uint32_t i=0;i<next.mergedCount;++i)
    {
        if(!mergedNames[i]||mergedParents[i]<-1||mergedParents[i]>=int32_t(next.mergedCount))
            return {Error::Catalog,i};
        for(uint32_t j=0;j<i;++j)if(mergedNames[j]==mergedNames[i])return {Error::Duplicate,i};
        next.mergedNames[i]=mergedNames[i]; next.mergedParents[i]=mergedParents[i];
    }
    for(uint32_t i=0;i<physicalCoreHashes.size();++i)
    {
        const int32_t merged=Find(mergedNames,physicalCoreHashes[i]);
        if(!physicalCoreHashes[i]||merged<0||next.mergedCore[merged])return {Error::CoreMapping,i};
        next.mergedCore[merged]=true;
    }
    for(uint32_t i=0;i<programCoreHashes.size();++i)
    {
        const uint64_t hash=programCoreHashes[i];
        const int32_t merged=Find(mergedNames,hash);
        const int32_t localIndex=Find(programNames,hash);
        if(!hash||merged<0||localIndex<0||program.nodes[localIndex].kind!=d::NodeKind::Core||
           !next.mergedCore[merged]||hasCoreLocal[localIndex]||coreLocals[i].nameHash!=hash)
            return {Error::CoreMapping,i};
        for(uint32_t p=0;p<translationProtectedHashes.size();++p)
            if(translationProtectedHashes[p]&&translationProtectedHashes[p]==hash&&coreLocals[i].replaceTranslation)
                return {Error::ProtectedOverride,i};
        if(!d::detail::Finite(coreLocals[i].translation))return {Error::CoreMapping,i};
        next.mergedCore[merged]=true;
        programCoreMerged[localIndex]=merged;
        localByProgram[localIndex]=coreLocals[i]; hasCoreLocal[localIndex]=true;
    }
    uint32_t programCoreCount=0;
    for(uint32_t i=0;i<program.nodeCount;++i)
        if(program.nodes[i].kind==d::NodeKind::Core)
        { ++programCoreCount; if(!hasCoreLocal[i])return {Error::CoreMapping,i}; }
    if(programCoreCount!=programCoreHashes.size())return {Error::CoreMapping,programCoreCount};

    for(uint32_t i=0;i<next.outputCount;++i)
    {
        if(!outputs[i].nameHash||outputs[i].nameHash!=ownedHashes[i]||
           outputs[i].programIndex>=program.nodeCount||
           program.nodes[outputs[i].programIndex].kind==d::NodeKind::Core||
           programNames[outputs[i].programIndex]!=outputs[i].nameHash)
            return {Error::OutputMapping,i};
        for(uint32_t j=0;j<i;++j)
            if(outputs[j].nameHash==outputs[i].nameHash||outputs[j].programIndex==outputs[i].programIndex)
                return {Error::Duplicate,i};
        next.outputNames[i]=outputs[i].nameHash; next.outputProgram[i]=outputs[i].programIndex;
        next.outputMerged[i]=Find(mergedNames,outputs[i].nameHash);
    }
    const int32_t anchorMerged=Find(mergedNames,spine3Hash);
    const int32_t anchorProgram=Find(programNames,spine3Hash);
    if(anchorMerged<0||anchorProgram<0||program.nodes[anchorProgram].kind!=d::NodeKind::Core||
       !hasCoreLocal[anchorProgram])return {Error::Anchor};
    next.spine3Merged=uint32_t(anchorMerged); next.spine3Program=uint32_t(anchorProgram);

    auto helperResult=h::Prepare(mergedParents,mergedNames,physicalCoreHashes,ownedHashes,
                                 compensatableHashes,next.helperPlan,regrouped,groups);
    if(!helperResult||next.helperPlan.ownedCount!=next.outputCount)return {Error::Catalog,uint32_t(helperResult.bone<0?0:helperResult.bone)};

    // Actual-space transfer needs all core locals and the real ancestry of the
    // Spine3 anchor and helperpatch outputs. The evaluator uses source topology
    // independently; do not substitute merged parents for Program parents.
    // Protection alone does not make an unused physical core bone an input.
    for(uint32_t i=0;i<program.nodeCount;++i)if(hasCoreLocal[i])
        next.mergedNeeded[programCoreMerged[i]]=true;
    next.mergedNeeded[next.spine3Merged]=true;
    for(uint32_t k=0;k<next.helperPlan.outputCount;++k)next.mergedNeeded[next.helperPlan.outputs[k]]=true;
    for(uint32_t i=0;i<next.mergedCount;++i)
        if(next.mergedNeeded[i])for(int32_t p=int32_t(i);p>=0;p=next.mergedParents[p])next.mergedNeeded[p]=true;

    for(uint32_t i=0;i<program.nodeCount;++i)if(hasCoreLocal[i])
        for(int32_t p=int32_t(i);p>=0&&!next.virtualNeeded[p];p=program.nodes[p].parent)next.virtualNeeded[p]=true;
    std::array<uint8_t,kMaxMergedBones> state{};
    std::array<int32_t,kMaxMergedBones> path{}; uint32_t ordered=0;
    for(uint32_t root=0;root<next.mergedCount;++root)
    {
        if(state[root])continue;
        uint32_t size=0;int32_t i=int32_t(root);
        while(i>=0&&state[i]==0){state[i]=1;path[size++]=i;i=next.mergedParents[i];}
        if(i>=0&&state[i]==1)return {Error::Catalog,uint32_t(i)};
        while(size){i=path[--size];state[i]=2;next.mergedOrder[ordered++]=i;}
    }
    for(uint32_t i=0;i<program.nodeCount;++i)if(hasCoreLocal[i])
    { next.programCoreMerged[i]=programCoreMerged[i]; next.coreLocals[i]=localByProgram[i]; }
    next.valid=true;out=next;return {};
}

// Hot, allocation-free candidate construction. Inputs are merged LOCAL 48-byte
// records and explicit availability. Output is an offline proposal only.
inline Result Build(const Plan& plan,std::span<const float> actualRecords,
    std::span<const uint8_t> available,Scratch& scratch,h::Patch& out)
{
    using namespace d::detail;
    if(!plan.valid||!plan.program||!plan.mergedCount||
       actualRecords.size()<size_t(plan.mergedCount)*h::kRecordFloats||
       available.size()!=plan.mergedCount)return {Error::Bounds};
    scratch.actualReady.fill(false);scratch.virtualReady.fill(false);
    scratch.inputs.present.fill(false);

    // Reconstruct ACTUAL merged MODEL using actual merged parents and LOCALs.
    for(uint32_t oi=0;oi<plan.mergedCount;++oi)
    {
        const int32_t i=plan.mergedOrder[oi];
        if(!plan.mergedNeeded[i])continue;
        if(!available[i])return {Error::MissingRecord,uint32_t(i)};
        d::Transform local,parent;
        if(!h::Decode(actualRecords.data()+size_t(i)*h::kRecordFloats,local))return {Error::InvalidRecord,uint32_t(i)};
        const int32_t p=plan.mergedParents[i];
        if(p>=0){if(!scratch.actualReady[p])return {Error::MissingRecord,uint32_t(p)};parent=scratch.actualMergedModel[p];}
        if(!Compose(parent,local,scratch.actualMergedModel[i]))return {Error::Transform,uint32_t(i)};
        scratch.actualReady[i]=true;
    }

    // Recreate SOURCE/program hierarchy for Core evaluator inputs. Core input
    // values are virtual MODEL transforms composed from named actual LOCALs;
    // Reference nodes use program refLocal; raw actual Core transforms are
    // never copied directly into evaluator inputs.
    const d::Program& program=*plan.program;
    for(uint32_t i=0;i<program.nodeCount;++i)
    {
        const d::Node& node=program.nodes[i];
        if(node.kind==d::NodeKind::Logical||!plan.virtualNeeded[i])continue; // Evaluate composes every other reference node itself
        d::Transform local,parent;
        if(node.kind==d::NodeKind::Core)
        {
            const int32_t merged=plan.programCoreMerged[i];
            if(merged<0||!available[merged])return {Error::MissingRecord,uint32_t(i)};
            if(!h::Decode(actualRecords.data()+size_t(merged)*h::kRecordFloats,local))return {Error::InvalidRecord,uint32_t(merged)};
            const CoreLocal& cfg=plan.coreLocals[i];
            if(cfg.replaceTranslation)local.t=cfg.translation;
        }
        else local=node.refLocal;
        if(node.parent>=0)
        { if(!scratch.virtualReady[node.parent])return {Error::Transform,uint32_t(i)};parent=scratch.virtualProgramModel[node.parent]; }
        if(!Compose(parent,local,scratch.virtualProgramModel[i]))return {Error::Transform,uint32_t(i)};
        scratch.virtualReady[i]=true;
        if(node.kind==d::NodeKind::Core)
        { scratch.inputs.model[i]=scratch.virtualProgramModel[i];scratch.inputs.present[i]=true; }
    }
    if(!d::Evaluate(program,scratch.inputs,scratch.evaluator))return {Error::Evaluation};
    const uint32_t anchorProgram=plan.spine3Program,anchorMerged=plan.spine3Merged;
    if(!scratch.evaluator.initialized[anchorProgram]||!scratch.actualReady[anchorMerged])return {Error::Anchor};

    for(uint32_t i=0;i<plan.outputCount;++i)
    {
        const uint32_t programIndex=plan.outputProgram[i];
        if(programIndex>=program.nodeCount||!scratch.evaluator.initialized[programIndex])return {Error::OutputMapping,i};
        d::Transform relative,actualTarget;
        if(!Relative(scratch.evaluator.model[anchorProgram],scratch.evaluator.model[programIndex],relative)||
           !Compose(scratch.actualMergedModel[anchorMerged],relative,actualTarget))return {Error::Transform,i};
        scratch.targets[i]={plan.outputNames[i],actualTarget,plan.outputMerged[i]};
    }
    // The helper builder needs the same actual MODEL chain already composed above (its needed set is a subset of mergedNeeded).
    // It writes `out` directly and only on success (a failure leaves `out` untouched).
    const auto patchResult=h::BuildShared(plan.helperPlan,actualRecords,{scratch.actualMergedModel.data(),plan.mergedCount},
                                          {scratch.actualReady.data(),plan.mergedCount},{scratch.targets.data(),plan.outputCount},scratch.helper,out);
    if(!patchResult)return {Error::HelperPatch,uint32_t(patchResult.bone<0?0:patchResult.bone)};
    return {};
}
} // namespace utpp::mm::bodyhelpercandidate

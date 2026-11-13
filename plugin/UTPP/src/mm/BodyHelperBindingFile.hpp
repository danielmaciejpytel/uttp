#pragma once
// Cold-only, bounded binding parser. No assets or parameters are embedded here.
#include "BodyHelperCandidate.hpp"
#include "BodyDeformerProgramFile.hpp"
#include <string>
#include <vector>

namespace utpp::mm::helperbinding
{
namespace c=bodyhelpercandidate;
struct Data
{
    uint64_t anchor=0;
    uint32_t anchorNode=0;
    std::vector<uint64_t> physicalCore,owned,children,programCore;
    std::vector<c::CoreLocal> locals;
    std::vector<c::HelperOutput> outputs;
};
inline bool NameHash(std::istream& in,uint64_t& out)
{
    std::string token;if(!(in>>token)||token.empty()||token.size()>16)return false;
    out=0;
    for(unsigned char ch:token)
    {
        if(!std::isxdigit(ch))return false;
        const uint32_t digit=ch>='0'&&ch<='9'?ch-'0':(ch>='a'&&ch<='f'?ch-'a'+10:ch-'A'+10);
        out=(out<<4)|digit;
    }
    return out!=0;
}
// programFileHash must be computed from the actual bytes BEFORE parsing IR.
inline bool Read(std::istream& in,const deformoffline::Program& program,
                 std::span<const uint64_t> names,const std::string& identity,
                 const std::string& programFileHash,Data& out)
{
    namespace f=deformerfile;
    uint32_t version=0,nodes=0,cores=0,owned=0,children=0,overrides=0;
    std::string source,checksum;
    if(names.size()!=program.nodeCount||!f::Tag(in,"UTPP_HELPER_BINDING")||!(in>>version)||version!=1||
       !f::Hash(in,source)||source!=identity||!f::Tag(in,"PROGRAM_SHA256")||!(in>>checksum)||
       checksum!=programFileHash||checksum.size()!=64||!f::Tag(in,"COUNTS")||
       !(in>>nodes>>cores>>owned>>children>>overrides)||nodes!=program.nodeCount||
       !cores||cores>1024||!owned||owned>c::kMaxOwned||children>c::kMaxOwned||overrides>nodes)return false;
    Data next;
    if(!f::Tag(in,"ANCHOR")||(!NameHash(in,next.anchor)||!(in>>next.anchorNode))||next.anchorNode>=nodes||
       names[next.anchorNode]!=next.anchor||program.nodes[next.anchorNode].kind!=deformoffline::NodeKind::Core)return false;
    auto hashes=[&](const char* tag,uint32_t count,std::vector<uint64_t>& values)
    {
        for(uint32_t i=0;i<count;++i)
        {
            uint64_t value=0;if(!f::Tag(in,tag)||!NameHash(in,value)||
                std::find(values.begin(),values.end(),value)!=values.end())return false;
            values.push_back(value);
        }
        return true;
    };
    if(!hashes("CORE",cores,next.physicalCore))return false;
    for(uint32_t i=0;i<owned;++i)
    {
        c::HelperOutput value;
        if(!f::Tag(in,"OWNED")||(!NameHash(in,value.nameHash)||!(in>>value.programIndex))||value.programIndex>=nodes||
           names[value.programIndex]!=value.nameHash||
           std::find(next.owned.begin(),next.owned.end(),value.nameHash)!=next.owned.end())return false;
        next.owned.push_back(value.nameHash);next.outputs.push_back(value);
    }
    if(!hashes("CHILD",children,next.children))return false;
    for(uint32_t i=0;i<nodes;++i)if(program.nodes[i].kind==deformoffline::NodeKind::Core)
    {
        next.programCore.push_back(names[i]);
        c::CoreLocal local;local.nameHash=names[i];next.locals.push_back(local);
    }
    for(uint32_t i=0;i<overrides;++i)
    {
        uint64_t hash=0;deformoffline::Vec3 t;
        if(!f::Tag(in,"OVERRIDE")||(!NameHash(in,hash)||!(in>>t.x>>t.y>>t.z))||!deformoffline::detail::Finite(t))return false;
        auto it=std::find_if(next.locals.begin(),next.locals.end(),[&](const auto& x){return x.nameHash==hash;});
        if(it==next.locals.end()||it->replaceTranslation)return false;
        it->replaceTranslation=true;it->translation=t;
    }
    std::string tail;
    if(!f::Tag(in,"END")||(in>>tail))return false;
    out=std::move(next);return true;
}
} // namespace utpp::mm::helperbinding

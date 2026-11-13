#pragma once
// Own cold-path parser. Asset-derived programs stay local; no hot-path I/O.
#include "BodyDeformerOffline.hpp"
#include "GameIntegration.hpp"
#include <cctype>
#include <istream>
#include <string>

namespace utpp::mm::deformerfile
{
inline bool Tag(std::istream& in,const char* expected)
{ std::string token;return bool(in>>token)&&token==expected; }
inline bool Hash(std::istream& in,std::string& out)
{
    if(!Tag(in,"SHA256")||!(in>>out)||out.size()!=64)return false;
    for(unsigned char c:out)if(!std::isxdigit(c))return false;
    return true;
}
inline bool Transform(std::istream& in,deformoffline::Transform& t)
{
    return bool(in>>t.t.x>>t.t.y>>t.t.z>>t.q.x>>t.q.y>>t.q.z>>t.q.w>>t.s.x>>t.s.y>>t.s.z);
}
// Output is scratch owned by the cold caller; never publish it before success.
inline bool Read(std::istream& in,deformoffline::Program& program,
                 std::array<uint64_t,deformoffline::kMaxNodes>& names,std::string& identity)
{
    namespace d=deformoffline;
    uint32_t version=0;
    if(!Tag(in,"UTPP_DEFORMER_IR")||!(in>>version>>program.nodeCount>>program.opCount)||
       version!=1||!program.nodeCount||program.nodeCount>d::kMaxNodes||!program.opCount||
       program.opCount>d::kMaxOps||!Hash(in,identity))return false;
    for(uint32_t i=0;i<program.nodeCount;++i)
    {
        uint32_t id=0,kind=0;std::string name;auto& node=program.nodes[i];
        if(!Tag(in,"NODE")||!(in>>id>>name>>node.parent>>kind)||id!=i||name.size()>128||kind>2||
           !Transform(in,node.refLocal))return false;
        node.kind=static_cast<d::NodeKind>(kind);names[i]=integ::CNameHash(name);
        if(!names[i])return false;
        for(uint32_t j=0;j<i;++j)if(names[j]==names[i])return false;
    }
    for(uint32_t i=0;i<program.opCount;++i)
    {
        auto& op=program.ops[i];uint32_t kind=0;
        if(!Tag(in,"OP")||!(in>>kind>>op.sourceHandle>>op.target>>op.a>>op.b>>op.c)||kind>11)return false;
        op.kind=static_cast<d::OpKind>(kind);
        for(auto& x:op.p)in>>x;
        for(auto& v:op.v)in>>v.x>>v.y>>v.z;
        if(!Transform(in,op.x)||!(in>>op.inputCount)||op.inputCount>d::kMaxInputs)return false;
        for(uint32_t k=0;k<op.inputCount;++k)in>>op.inputs[k].node>>op.inputs[k].weight;
        if(!(in>>op.channelCount)||op.channelCount>d::kMaxChannels)return false;
        for(uint32_t k=0;k<op.channelCount;++k)
            in>>op.channels[k].target>>op.channels[k].parent>>op.channels[k].kind>>op.channels[k].multiplier;
        if(!in)return false;
    }
    std::string extra;
    return !(in>>extra)&&bool(d::Validate(program));
}
} // namespace utpp::mm::deformerfile

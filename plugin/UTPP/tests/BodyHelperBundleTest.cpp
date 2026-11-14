// Real-data check of a player helper bundle (program.ir + binding.dat) through the production offline pipeline
// (bodyhelpercandidate::Prepare/Build + bodyhelpercommit::Commit) on recorded frames of a local run. Not shipped data:
// the fixture directory is derived from a recording (tools/analysis/make_helper_bundle_fixture.py) and lives in Build/test-data.
// One run per body (man, woman): the fixture directory holds that body's bundle and recorded frames.
// usage: BodyHelperBundleTest <fixture dir> <man|woman> [--bench <iterations> <max Build+Commit microseconds>]
#include "../src/mm/BodyDeformerProgramFile.hpp"
#include "../src/mm/BodyHelperAdmission.hpp"
#include "../src/mm/BodyHelperBindingFile.hpp"
#include "../src/mm/BodyHelperCarry.hpp"
#include "../src/mm/BodyHelperCommit.hpp"
#include "../src/mm/GameIntegration.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace c=utpp::mm::bodyhelpercandidate;
namespace d=utpp::mm::deformoffline;
namespace h=utpp::mm::helperpatch;
namespace k=utpp::mm::bodyhelpercommit;
namespace b=utpp::mm::helperbinding;
namespace f=utpp::mm::deformerfile;

namespace
{
using Record=std::array<float,h::kRecordFloats>;
struct Frame{ std::vector<Record> rec; std::vector<uint8_t> avail; };
struct Expected{ std::map<uint64_t,std::array<double,10>> byHash; };
struct Fixture
{
    std::vector<uint64_t> hashes; std::vector<int32_t> parents; std::vector<std::string> names;
    std::vector<std::string> order; std::map<std::string,Frame> frames; std::map<std::string,Expected> expected;
    std::map<uint64_t,std::array<double,3>> proxy;
};

std::string ReadFile(const std::string& path)
{
    std::ifstream in(path,std::ios::binary);std::ostringstream out;out<<in.rdbuf();return out.str();
}
bool LoadFixture(const std::string& dir,Fixture& fx)
{
    std::ifstream in(dir+"/frames.txt");if(!in)return false;
    std::string tag;uint32_t rig=0;int version=0;
    in>>tag>>version;if(tag!="UTPP_HELPER_FIXTURE"||version!=1)return false;
    in>>tag>>rig;if(tag!="RIG"||!rig||rig>1024)return false;
    fx.hashes.resize(rig);fx.parents.resize(rig);fx.names.resize(rig);
    for(uint32_t i=0;i<rig;++i)
    {
        uint32_t idx=0;std::string hash,name;int32_t parent=0;
        in>>tag>>idx>>hash>>parent>>name;if(tag!="BONE"||idx!=i)return false;
        fx.hashes[i]=std::strtoull(hash.c_str(),nullptr,16);fx.parents[i]=parent;fx.names[i]=name=="-"?std::string():name;
    }
    std::string label;uint32_t count=0;Frame* cur=nullptr;
    while(in>>tag)
    {
        if(tag=="FRAME")
        {
            in>>label>>count;fx.order.push_back(label);cur=&fx.frames[label];
            cur->rec.assign(rig,Record{});cur->avail.assign(rig,0);
        }
        else if(tag=="R"&&cur)
        {
            uint32_t idx=0;in>>idx;if(idx>=rig)return false;
            for(auto& x:cur->rec[idx])in>>x;
            cur->avail[idx]=1;
        }
        else return false;
    }
    std::ifstream ex(dir+"/expected.txt");if(!ex)return false;
    while(ex>>tag)
    {
        if(tag!="E")return false;
        std::string hash;std::array<double,10> v{};ex>>label>>hash;for(auto& x:v)ex>>x;
        fx.expected[label].byHash[std::strtoull(hash.c_str(),nullptr,16)]=v;
    }
    std::ifstream px(dir+"/proxy.txt");
    while(px>>tag)
    {
        std::string hash;std::array<double,3> q{};int n=0;px>>hash>>q[0]>>q[1]>>q[2]>>n;
        fx.proxy[std::strtoull(hash.c_str(),nullptr,16)]=q;
    }
    return true;
}

struct Bundle
{
    std::unique_ptr<d::Program> program=std::make_unique<d::Program>();
    std::array<uint64_t,d::kMaxNodes> names{};
    std::string identity,programHash;
    b::Data binding;
};
std::string Sha256(const std::string& data)
{
    BCRYPT_ALG_HANDLE alg=nullptr;BCRYPT_HASH_HANDLE hash=nullptr;std::string hex;
    if(BCryptOpenAlgorithmProvider(&alg,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0)return hex;
    unsigned char digest[32]{};
    if(BCryptCreateHash(alg,&hash,nullptr,0,nullptr,0,0)>=0&&
       BCryptHashData(hash,reinterpret_cast<PUCHAR>(const_cast<char*>(data.data())),ULONG(data.size()),0)>=0&&
       BCryptFinishHash(hash,digest,32,0)>=0)
    { static const char* digits="0123456789abcdef";for(unsigned char x:digest){hex+=digits[x>>4];hex+=digits[x&15];} }
    if(hash)BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(alg,0);return hex;
}

bool LoadBundle(const std::string& dir,Bundle& out,std::string& programBytes,std::string& bindingBytes)
{
    programBytes=ReadFile(dir+"/program.ir");bindingBytes=ReadFile(dir+"/binding.dat");
    if(programBytes.empty()||bindingBytes.empty())return false;
    out.programHash=Sha256(programBytes);
    std::istringstream pi(programBytes);
    if(!f::Read(pi,*out.program,out.names,out.identity))return false;
    std::istringstream bi(bindingBytes);
    return b::Read(bi,*out.program,{out.names.data(),out.program->nodeCount},out.identity,out.programHash,out.binding);
}

// Angle between two rotations in degrees from the relative quaternion (accurate for tiny angles, unlike acos of a dot product).
double AngleDeg(const d::Quat& a,const d::Quat& b)
{
    const d::Quat r=d::detail::Product(d::detail::Conj(a),b);
    const double v=std::sqrt(double(r.x)*r.x+double(r.y)*r.y+double(r.z)*r.z);
    return 2.0*std::atan2(v,std::fabs(double(r.w)))*180.0/3.14159265358979323846;
}

// FK of every available record: model[i] (parents first, as in every rig dump)
void Fk(const Fixture& fx,const std::vector<Record>& rec,const std::vector<uint8_t>& avail,std::vector<d::Transform>& model,std::vector<uint8_t>& ready)
{
    model.assign(rec.size(),d::Transform{});ready.assign(rec.size(),0);
    for(size_t i=0;i<rec.size();++i)
    {
        if(!avail[i])continue;
        d::Transform local;if(!h::Decode(rec[i].data(),local))continue;
        const int32_t p=fx.parents[i];
        if(p>=0&&!ready[p])continue;
        if(!d::detail::Compose(p>=0?model[p]:d::Transform{},local,model[i]))continue;
        ready[i]=1;
    }
}
d::Vec3 Apply(const d::Transform& m,const std::array<double,3>& q)
{
    return d::detail::Add(m.t,d::detail::Rotate(m.q,d::detail::Had(m.s,d::Vec3{float(q[0]),float(q[1]),float(q[2])})));
}
bool Contains(const std::string& s,const char* part){ return s.find(part)!=std::string::npos; }
} // namespace

int main(int argc,char** argv)
{
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    if(argc<3||(std::string(argv[2])!="man"&&std::string(argv[2])!="woman")){std::printf("usage: BodyHelperBundleTest <fixture dir> <man|woman> [--bench <n> <us>]\n");return 2;}
    const std::string dir=argv[1];
    const std::string bodyName=argv[2];
    const int expectedBody=bodyName=="man"?utpp::mm::helperadmission::kBodyMan:utpp::mm::helperadmission::kBodyWoman;
    const bool bench=argc>=6&&std::string(argv[3])=="--bench";
    Fixture fx;check(LoadFixture(dir,fx),"fixture loads");
    Bundle bundle;std::string programBytes,bindingBytes;
    check(LoadBundle(dir,bundle,programBytes,bindingBytes),"bundle parses with byte-hash identity");
    if(failures){std::printf("bundle/fixture unusable\n");return 1;}

    // the files under test are exactly the bundle the DLL has compiled in as approved (hashes of the exact bytes + counts)
    {
        const auto* entry=utpp::mm::helperadmission::Find(utpp::mm::helperadmission::kApproved,bundle.programHash,Sha256(bindingBytes),
            uint32_t(bundle.binding.owned.size()),uint32_t(bundle.binding.children.size()));
        check(entry&&entry->body==expectedBody,("program.ir + binding.dat are the approved "+bodyName+" bundle compiled into the DLL").c_str());
    }

    // negative controls on the bundle identity: any changed program byte or wrong binding hash is rejected
    {
        Bundle tampered;std::string prog=programBytes,bind=bindingBytes;
        const size_t at=prog.find("\nNODE 1 ");prog[at+1]=(prog[at+1]=='N'?'M':'N');
        std::istringstream pi(prog);
        check(!f::Read(pi,*tampered.program,tampered.names,tampered.identity),"tampered program text rejected by parser");
        // one changed digit inside an op parameter still parses, but the binding is bound to the original bytes
        std::string prog2=programBytes;
        const size_t op=prog2.find("\nOP ");const size_t dot=prog2.find('.',op);
        prog2[dot+1]=(prog2[dot+1]=='7'?'3':'7');
        Bundle t2;std::istringstream pi2(prog2);
        check(f::Read(pi2,*t2.program,t2.names,t2.identity),"program with one changed parameter digit still parses");
        std::istringstream bi2(bind);
        check(Sha256(prog2)!=bundle.programHash&&
              !b::Read(bi2,*t2.program,{t2.names.data(),t2.program->nodeCount},t2.identity,Sha256(prog2),t2.binding),
              "binding rejects a program whose bytes changed");
        Bundle t3;std::istringstream pi3(programBytes);f::Read(pi3,*t3.program,t3.names,t3.identity);
        std::string bad=bind;const size_t hp=bad.find("PROGRAM_SHA256 ");bad[hp+15]=(bad[hp+15]=='0'?'1':'0');
        std::istringstream bi3(bad);
        check(!b::Read(bi3,*t3.program,{t3.names.data(),t3.program->nodeCount},t3.identity,bundle.programHash,t3.binding),
              "binding bound to another program hash rejected");
    }

    // plan on the real merged rig
    std::vector<int32_t> parents(fx.parents.begin(),fx.parents.end());
    const std::array<uint64_t,3> protectedTranslations{utpp::mm::integ::CNameHash("Root"),utpp::mm::integ::CNameHash("Hips"),utpp::mm::integ::CNameHash("Trajectory")};
    auto plan=std::make_unique<c::Plan>();
    const auto& bd=bundle.binding;
    const auto prepared=c::Prepare(*bundle.program,{bundle.names.data(),bundle.program->nodeCount},parents,fx.hashes,bd.programCore,bd.physicalCore,
                                  bd.locals,protectedTranslations,bd.owned,bd.children,bd.outputs,bd.anchor,*plan);
    check(bool(prepared),"Prepare accepts the bundle on the merged player rig");
    // the rig signature compiled into the DLL (closure of the bundle's bones in the recorded merged rig) is the one of this fixture
    {
        std::vector<uint64_t> roots(bd.physicalCore);
        for(const auto* group:{&bd.programCore,&bd.owned,&bd.children})roots.insert(roots.end(),group->begin(),group->end());
        roots.push_back(bd.anchor);
        std::vector<int16_t> parents16;for(const int32_t p:fx.parents)parents16.push_back(static_cast<int16_t>(p));
        const auto entry=utpp::mm::helperadmission::Find(utpp::mm::helperadmission::kApproved,bundle.programHash,Sha256(bindingBytes),
            uint32_t(bd.owned.size()),uint32_t(bd.children.size()));
        const auto sig=utpp::mm::helperadmission::ComputeRigSignature<int16_t>(fx.hashes,parents16,roots);
        std::printf("rig of %zu bones, bundle closure %u bones, fingerprint %016llx\n",fx.hashes.size(),sig.bones,static_cast<unsigned long long>(sig.fingerprint));
        check(sig.ok&&entry&&sig.bones==entry->closureBones&&sig.fingerprint==entry->closureFingerprint&&fx.hashes.size()>=entry->baseMinBones,
              "the recorded merged rig has the base and the closure signature approved for this bundle");
    }
    if(!prepared){std::printf("Prepare code=%u index=%u\n",unsigned(prepared.error),prepared.index);return 1;}
    std::printf("bundle: nodes=%u ops=%u owned=%u children=%zu plan outputs=%u\n",bundle.program->nodeCount,bundle.program->opCount,
                plan->outputCount,bd.children.size(),plan->helperPlan.outputCount);

    // no output may touch core, hands, weapon or IK bones (name checks where the recording knows the name)
    {
        bool clean=true;
        for(uint32_t o=0;o<plan->helperPlan.outputCount;++o)
        {
            const uint32_t i=plan->helperPlan.outputs[o];
            if(plan->helperPlan.core[i])clean=false;
            const std::string& n=fx.names[i];
            if(Contains(n,"Hand")||Contains(n,"Weapon")||Contains(n,"_ik")||Contains(n,"_IK")||Contains(n,"arm_"))clean=false;
        }
        check(clean,"patch outputs never include core, hand, weapon or IK bones");
    }

    thread_local c::Scratch scratch;thread_local h::Patch patch;
    double worstLocalT=0,worstLocalQ=0,worstLocalS=0,worstProxyVsRef=0;
    double worstReproPos=0,worstReproRot=0,maxJumpS0=0,maxJumpS1=0;uint32_t reproFrames=0;
    std::map<std::string,double> armpitStale;bool sawExit=false;
    for(const std::string& label:fx.order)
    {
        const Frame& fr=fx.frames[label];
        const auto built=c::Build(*plan,{reinterpret_cast<const float*>(fr.rec.data()),fr.rec.size()*h::kRecordFloats},fr.avail,scratch,patch);
        check(bool(built),("Build ok "+label).c_str());
        if(!built){std::printf("  Build code=%u index=%u\n",unsigned(built.error),built.index);continue;}
        check(patch.count==plan->helperPlan.outputCount,("patch covers exactly the plan outputs "+label).c_str());
        // 1) equals the offline reference (independent Python implementation of the same transfer)
        const auto& ex=fx.expected[label];
        double mt=0,mq=0,ms=0;bool all=true;
        std::vector<Record> after=fr.rec;
        for(uint32_t o=0;o<patch.count;++o)
        {
            const uint32_t i=patch.indices[o];after[i]=patch.after[o];
            // largest local translation change versus the engine's own record (the Commit translation safety net, in metres)
            {
                const auto& pa=patch.after[o];const auto& pb=patch.before[o];
                const double jump=std::sqrt(std::pow(pa[0]-pb[0],2)+std::pow(pa[1]-pb[1],2)+std::pow(pa[2]-pb[2],2));
                double& slot=Contains(label,".s0")?maxJumpS0:maxJumpS1;slot=std::fmax(slot,jump);
            }
            auto it=ex.byHash.find(fx.hashes[i]);
            if(it==ex.byHash.end()){all=false;continue;}
            const auto& e=it->second;const auto& r=patch.after[o];
            mt=std::fmax(mt,std::sqrt(std::pow(r[0]-e[0],2)+std::pow(r[1]-e[1],2)+std::pow(r[2]-e[2],2)));
            ms=std::fmax(ms,std::sqrt(std::pow(r[8]-e[7],2)+std::pow(r[9]-e[8],2)+std::pow(r[10]-e[9],2)));
            mq=std::fmax(mq,AngleDeg(d::Quat{r[4],r[5],r[6],r[7]},d::Quat{float(e[3]),float(e[4]),float(e[5]),float(e[6])}));
        }
        check(all,("every patched record has a reference "+label).c_str());
        worstLocalT=std::fmax(worstLocalT,mt);worstLocalQ=std::fmax(worstLocalQ,mq);worstLocalS=std::fmax(worstLocalS,ms);
        // FK before (engine/stale helpers) and after the patch
        std::vector<d::Transform> mStale,mNew;std::vector<uint8_t> rStale,rNew;
        Fk(fx,fr.rec,fr.avail,mStale,rStale);Fk(fx,after,fr.avail,mNew,rNew);
        // 2) stage 0 = the engine's own pose: the recomputed helpers must reproduce the helpers the engine recorded.
        // Known exception (car exit): for about 1.2 s after leaving a car (MM weight 0, the runtime does not write) and again for about
        // 0.4 s when MM resumes, the engine's l/r_scapula_A_top and l/r_butterfly_top deviate from the graph (and from this program) by up to
        // 2.7 cm / 19.6 deg in the seven recorded woman sessions (14 episodes). The eight recorded man sessions contain no car ride at all (the mod
        // writes the pose in every frame), so nothing is known about the man body in that state. The cause is not established (not a frame lag,
        // not a degenerate aim, not a clamped track, not a skipped graph segment, not a single core input).
        // Those real frames are the fixture frames labelled exit_*: for them every other helper must still reproduce within the normal tolerance and the four
        // named helpers within the documented bound below. No normal frame is excluded and the normal tolerance is not loosened.
        if(Contains(label,".s0"))
        {
            const bool isExit=label.rfind("exit_",0)==0;
            double maxPos=0,maxRot=0,exitPos=0,exitRot=0;uint32_t worst=0,compared=0;
            for(uint32_t o=0;o<patch.count;++o)
            {
                const uint32_t i=patch.indices[o];if(!rStale[i]||!rNew[i])continue;++compared;
                const double pos=100.0*std::sqrt(d::detail::Dot(d::detail::Sub(mStale[i].t,mNew[i].t),d::detail::Sub(mStale[i].t,mNew[i].t)));
                const double rot=AngleDeg(mStale[i].q,mNew[i].q);
                const bool documented=isExit&&(fx.names[i]=="l_scapula_A_top_out_JNT"||fx.names[i]=="r_scapula_A_top_out_JNT"||
                                              fx.names[i]=="l_butterfly_top_CRV_bot_out_JNT"||fx.names[i]=="r_butterfly_top_CRV_top_out_JNT");
                if(documented){exitPos=std::fmax(exitPos,pos);exitRot=std::fmax(exitRot,rot);continue;}
                if(pos>maxPos){maxPos=pos;worst=i;}
                maxRot=std::fmax(maxRot,rot);
                if(pos>1.0||rot>25.0)std::printf("  note %s %s pos %.2f cm rot %.1f deg\n",label.c_str(),fx.names[i].c_str(),pos,rot);
            }
            std::printf("engine reproduction %s: max helper model error %.4f cm (%s) max rot %.4f deg (%u of %u helpers compared)%s\n",label.c_str(),maxPos,fx.names[worst].c_str(),maxRot,compared,patch.count,
                        isExit?" [car-exit frame: 4 documented helpers excluded here]":"");
            // LIMITS_PLACEHOLDER
            check(compared==patch.count,("every recomputed helper has an engine record to compare "+label).c_str());
            check(maxPos<=0.02,("recomputed helpers reproduce the engine helpers within 0.02 cm on the raw engine pose "+label).c_str());
            check(maxRot<=0.5,("recomputed helpers reproduce the engine helper rotations within 0.5 deg on the raw engine pose "+label).c_str());
            if(isExit)
            {
                std::printf("car-exit deviation %s: the 4 documented helpers differ from the engine by up to %.2f cm / %.1f deg\n",label.c_str(),exitPos,exitRot);
                check(exitPos>0.2,("the exit frame really shows the documented deviation (> 0.2 cm; if the graph is ever corrected, revisit this section) "+label).c_str());
                check(exitPos<=2.8&&exitRot<=20.5,("documented car-exit bound: deviation of the 4 helpers <= 2.8 cm / 20.5 deg (largest of all recorded episodes 2.69 cm / 19.6 deg) "+label).c_str());
                sawExit=true;
            }
            else{worstReproPos=std::fmax(worstReproPos,maxPos);worstReproRot=std::fmax(worstReproRot,maxRot);++reproFrames;}
        }
        // 3) skin proxy points (mean bone-local reference position of the vertices each helper dominates)
        {
            double sumStale=0,sumRef=0,maxStale=0;int n=0;
            for(uint32_t o=0;o<patch.count;++o)
            {
                const uint32_t i=patch.indices[o];const std::string& nm=fx.names[i];
                auto pq=fx.proxy.find(fx.hashes[i]);if(pq==fx.proxy.end()||!rStale[i]||!rNew[i])continue;
                const d::Vec3 a=Apply(mStale[i],pq->second),bb=Apply(mNew[i],pq->second);
                const double ds=100.0*std::sqrt(d::detail::Dot(d::detail::Sub(a,bb),d::detail::Sub(a,bb)));
                // reference: the expected local applied through the same parents (using the patched parent models)
                const auto& e=ex.byHash.at(fx.hashes[i]);
                d::Transform loc;loc.t={float(e[0]),float(e[1]),float(e[2])};loc.q={float(e[3]),float(e[4]),float(e[5]),float(e[6])};loc.s={float(e[7]),float(e[8]),float(e[9])};
                d::Transform refModel;const int32_t p=fx.parents[i];
                d::detail::Compose(p>=0?mNew[p]:d::Transform{},loc,refModel);
                const d::Vec3 rv=Apply(refModel,pq->second);
                const double dr=100.0*std::sqrt(d::detail::Dot(d::detail::Sub(rv,bb),d::detail::Sub(rv,bb)));
                worstProxyVsRef=std::fmax(worstProxyVsRef,dr);
                if(Contains(nm,"l_chest_front")||Contains(nm,"l_scapula")||Contains(nm,"l_deltoid")||Contains(nm,"l_latissimus")||Contains(nm,"l_trapezius"))
                { sumStale+=ds;maxStale=std::fmax(maxStale,ds);sumRef+=dr;++n; }
            }
            if(n){armpitStale[label]=sumStale/n;std::printf("left shoulder/armpit proxy %s: engine-vs-corrected mean %.2f cm max %.2f cm (n=%d), corrected-vs-reference mean %.4f cm\n",label.c_str(),sumStale/n,maxStale,n,sumRef/n);}
        }
    }
    std::printf("engine reproduction over %u stage-0 frames: max position error %.4f cm, max rotation error %.4f deg\n",reproFrames,worstReproPos,worstReproRot);
    std::printf("local translation change versus the engine record (Commit safety net): stage 0 max %.4f m, stage 1 max %.4f m (cap %.2f m)\n",maxJumpS0,maxJumpS1,double(k::kMaxTranslationDelta));
    check(reproFrames>0,"the fixture has stage-0 (raw engine pose) frames");
    if(expectedBody==utpp::mm::helperadmission::kBodyWoman)check(sawExit,"the woman fixture contains the recorded car-exit frames (documented deviation)");
    check(maxJumpS0<=k::kMaxTranslationDelta&&maxJumpS1<=k::kMaxTranslationDelta,"every legitimate helper change is inside the Commit translation cap");
    std::printf("reference match: max local translation %.2e m, rotation %.4f deg, scale %.2e; proxy vs reference max %.4f cm\n",worstLocalT,worstLocalQ,worstLocalS,worstProxyVsRef);
    check(worstLocalT<2e-5,"committed helper local translations equal the offline reference (< 0.002 cm)");
    check(worstLocalQ<0.005,"committed helper local rotations equal the offline reference (< 0.005 deg)");
    check(worstLocalS<2e-5,"committed helper local scales equal the offline reference");
    check(worstProxyVsRef<0.01,"skin proxy of every corrected helper equals the reference body (< 0.01 cm)");
    if(armpitStale.count("stand_armed.s1"))
        check(armpitStale["stand_armed.s1"]>3.0,"armed stand: engine helpers are far from the corrected body at the left shoulder/armpit (documents the defect)");

    // commit: exactly the planned records are written, nothing else; permission denied or a changed live record writes nothing
    {
        const Frame& fr=fx.frames["stand_armed.s1"];
        std::vector<Record> live=fr.rec;
        auto built=c::Build(*plan,{reinterpret_cast<const float*>(live.data()),live.size()*h::kRecordFloats},fr.avail,scratch,patch);
        check(bool(built),"Build for commit test");
        const auto deny=[](void*)noexcept{return false;};
        const auto allow=[](void*)noexcept{return true;};
        auto r0=k::Commit(plan->helperPlan,{reinterpret_cast<float*>(live.data()),live.size()*h::kRecordFloats},patch,deny,nullptr);
        check(!r0&&std::memcmp(live.data(),fr.rec.data(),live.size()*sizeof(Record))==0,"permission denied: no record written");
        auto stale=patch;stale.before[0][0]+=1.f;
        auto r1=k::Commit(plan->helperPlan,{reinterpret_cast<float*>(live.data()),live.size()*h::kRecordFloats},stale,allow,nullptr);
        check(!r1&&std::memcmp(live.data(),fr.rec.data(),live.size()*sizeof(Record))==0,"changed live record: no record written");
        auto r2=k::Commit(plan->helperPlan,{reinterpret_cast<float*>(live.data()),live.size()*h::kRecordFloats},patch,allow,nullptr);
        check(bool(r2),"commit of the complete patch");
        std::vector<uint8_t> touched(live.size(),0);for(uint32_t o=0;o<patch.count;++o)touched[patch.indices[o]]=1;
        bool only=true;
        for(size_t i=0;i<live.size();++i)
            if(!touched[i]&&std::memcmp(&live[i],&fr.rec[i],sizeof(Record))!=0)only=false;
        check(only,"commit writes only the planned records");
        // a record the patch builder cannot decode must fail the whole Build with no output
        auto broken=fr.rec;broken[plan->helperPlan.outputs[0]][4]=NAN;
        h::Patch untouched;untouched.count=777;
        auto r3=c::Build(*plan,{reinterpret_cast<const float*>(broken.data()),broken.size()*h::kRecordFloats},fr.avail,scratch,untouched);
        check(!r3&&untouched.count==777,"undecodable helper record: Build fails and publishes no patch");
        auto unavailable=fr.avail;unavailable[plan->helperPlan.outputs[0]]=0;
        auto r4=c::Build(*plan,{reinterpret_cast<const float*>(fr.rec.data()),fr.rec.size()*h::kRecordFloats},unavailable,scratch,untouched);
        check(!r4,"unavailable helper record: Build fails");
    }

    if(bench)
    {
        const int iterations=std::atoi(argv[4]);const double limit=std::atof(argv[5]);
        std::vector<std::string> labels;for(auto& l:fx.order)if(Contains(l,".s1"))labels.push_back(l);
        std::vector<std::vector<Record>> lives;for(auto& l:labels)lives.push_back(fx.frames[l].rec);
        LARGE_INTEGER hz,t0,t1;QueryPerformanceFrequency(&hz);
        const auto allow=[](void*)noexcept{return true;};
        double buildTicks=0,totalTicks=0;std::vector<double> totals;totals.reserve(size_t(iterations));
        for(int it=0;it<iterations+200;++it)
        {
            const size_t fi=size_t(it)%labels.size();auto& live=lives[fi];const Frame& fr=fx.frames[labels[fi]];
            QueryPerformanceCounter(&t0);
            auto r=c::Build(*plan,{reinterpret_cast<const float*>(live.data()),live.size()*h::kRecordFloats},fr.avail,scratch,patch);
            QueryPerformanceCounter(&t1);
            const double build=double(t1.QuadPart-t0.QuadPart);
            auto cr=k::Commit(plan->helperPlan,{reinterpret_cast<float*>(live.data()),live.size()*h::kRecordFloats},patch,allow,nullptr);
            QueryPerformanceCounter(&t1);
            const double total=double(t1.QuadPart-t0.QuadPart);
            if(!r||!cr){check(false,"bench Build/Commit ok");break;}
            for(uint32_t o=0;o<patch.count;++o)std::memcpy(&live[patch.indices[o]],patch.before[o].data(),sizeof(Record)); // restore, not timed
            if(it>=200){buildTicks+=build;totalTicks+=total;totals.push_back(total/double(hz.QuadPart)*1e6);}
        }
        const double buildUs=buildTicks/double(hz.QuadPart)*1e6/iterations,totalUs=totalTicks/double(hz.QuadPart)*1e6/iterations;
        std::printf("bench: Build %.2f us/call, Build+Commit %.2f us/call over %d calls (%u outputs, %u ops)\n",buildUs,totalUs,iterations,patch.count,bundle.program->opCount);
        std::sort(totals.begin(),totals.end());
        const double medianUs=totals[totals.size()/2],p95Us=totals[totals.size()*95/100];
        std::printf("bench: Build+Commit median %.2f us, p95 %.2f us (ceiling on the median: %.0f us)\n",medianUs,p95Us,limit);
        // The ceiling (argument) is about 2x the median measured when it was set, so a real slowdown of the hot path fails; the median
        // (not the mean) keeps a loaded machine from tripping it on a few slow calls.
        check(medianUs<limit,"Build+Commit median stays under the timing ceiling");
        // The most expensive call of the weight-0 writer recompute (a frame of the fade after a written run): Build, measure the engine's
        // deviation, blend it with the carried one, apply it to the patch, Commit. Same ceiling.
        {
            namespace cy=utpp::mm::helpercarry;
            cy::Offsets carried{},live{},blended{};
            std::vector<double> fades;fades.reserve(size_t(iterations));
            for(int it=0;it<iterations+200;++it)
            {
                const size_t fi=size_t(it)%labels.size();auto& lv=lives[fi];const Frame& fr=fx.frames[labels[fi]];
                QueryPerformanceCounter(&t0);
                auto r=c::Build(*plan,{reinterpret_cast<const float*>(lv.data()),lv.size()*h::kRecordFloats},fr.avail,scratch,patch);
                const bool m=r&&cy::Measure(patch,live)&&cy::Blend(carried,true,live,patch.count,0.5f,blended)&&cy::Apply(patch,blended,false);
                auto cr=m?k::Commit(plan->helperPlan,{reinterpret_cast<float*>(lv.data()),lv.size()*h::kRecordFloats},patch,allow,nullptr):k::Result{};
                QueryPerformanceCounter(&t1);
                if(!r||!m||!cr){check(false,"bench fade path Build/Measure/Blend/Apply/Commit ok");break;}
                for(uint32_t o=0;o<patch.count;++o)std::memcpy(&lv[patch.indices[o]],patch.before[o].data(),sizeof(Record)); // restore, not timed
                if(it>=200)fades.push_back(double(t1.QuadPart-t0.QuadPart)/double(hz.QuadPart)*1e6);
            }
            if(!fades.empty())
            {
                std::sort(fades.begin(),fades.end());
                const double fadeMedian=fades[fades.size()/2],fadeP95=fades[fades.size()*95/100];
                std::printf("bench: weight-0 writer fade path (Build+Measure+Blend+Apply+Commit) median %.2f us, p95 %.2f us (ceiling on the median: %.0f us)\n",
                            fadeMedian,fadeP95,limit);
                check(fadeMedian<limit,"weight-0 writer fade path median stays under the timing ceiling");
            }
        }
    }
    std::printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}

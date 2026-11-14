// End-to-end check of the real BodyHelperRuntime (Load admission, PrepareArm, hot gate, Evaluate + Commit) on the recorded
// man and woman fixtures (Build/test-data/body_helper_man and body_helper_woman, derived from local recordings; not shipped).
// Layout under test: skin_helpers/mode.txt + skin_helpers/man/{program.ir,binding.dat} + skin_helpers/woman/{...}.
// Globals the runtime reads (g_hot, g_mmPose) are defined here, like in EarlyPoseRuntimeTest.
// usage: BodyHelperRuntimeTest <man fixture dir> <woman fixture dir>
#include "../src/BodyHelperRuntime.hpp"
#include "../src/EarlyPoseRuntime.hpp"
#include "../src/State.hpp"
#include "../src/mm/CoreWriteRecord.hpp"
#include "../src/ragdoll/RagdollEpisode.hpp"

#include <Windows.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
}
using namespace utpp;
namespace fs=std::filesystem;
namespace br=utpp::bodyhelperruntime;

namespace
{
constexpr int kBodyWoman=0,kBodyMan=1;
constexpr uint32_t kOwned=68; // helper records of an approved bundle (the same list for both bodies)
int g_checks=0,g_failures=0;
void check(bool ok,const std::string& label){++g_checks;if(!ok){++g_failures;std::printf("FAIL %s\n",label.c_str());}}

struct Fixture
{
    std::vector<uint64_t> hashes;std::vector<int16_t> parents;std::vector<float> bones; // stand_armed.s1, 12 floats per rig bone
};
bool LoadFixture(const std::string& dir,Fixture& fx,const std::string& wanted)
{
    std::ifstream in(dir+"/frames.txt");if(!in)return false;
    std::string tag,label;uint32_t rig=0,count=0;int version=0;
    in>>tag>>version>>tag>>rig;
    fx.hashes.resize(rig);fx.parents.resize(rig);fx.bones.assign(size_t(rig)*12,0.f);
    for(uint32_t i=0;i<rig;++i)
    {
        uint32_t idx=0;std::string hash,name;int parent=0;in>>tag>>idx>>hash>>parent>>name;
        fx.hashes[i]=std::strtoull(hash.c_str(),nullptr,16);fx.parents[i]=int16_t(parent);
    }
    bool take=false;
    while(in>>tag)
    {
        if(tag=="FRAME"){in>>label>>count;take=label==wanted;}
        else if(tag=="R"){uint32_t idx=0;in>>idx;float v[12];for(auto& x:v)in>>x;if(take&&idx<rig)std::memcpy(&fx.bones[size_t(idx)*12],v,sizeof v);}
    }
    return true;
}
// Offline reference (expected.txt, independent Python implementation) for one frame label: hash -> t(3) q(4) s(3)
std::map<uint64_t,std::array<double,10>> LoadExpected(const std::string& dir,const std::string& wanted)
{
    std::map<uint64_t,std::array<double,10>> out;std::ifstream in(dir+"/expected.txt");std::string tag,label,hash;
    while(in>>tag>>label>>hash){std::array<double,10> v{};for(auto& x:v)in>>x;if(label==wanted)out[std::strtoull(hash.c_str(),nullptr,16)]=v;}
    return out;
}
double AngleDeg(const float* a,const double* b)
{
    // relative rotation angle of two unit quaternions (x,y,z,w), accurate for tiny angles
    const double rx=-double(a[0])*b[3]+double(a[3])*b[0]-double(a[1])*b[2]+double(a[2])*b[1];
    const double ry=-double(a[1])*b[3]+double(a[3])*b[1]-double(a[2])*b[0]+double(a[0])*b[2];
    const double rz=-double(a[2])*b[3]+double(a[3])*b[2]-double(a[0])*b[1]+double(a[1])*b[0];
    const double rw=double(a[3])*b[3]+double(a[0])*b[0]+double(a[1])*b[1]+double(a[2])*b[2];
    return 2.0*std::atan2(std::sqrt(rx*rx+ry*ry+rz*rz),std::fabs(rw))*180.0/3.14159265358979323846;
}
bool WriteText(const fs::path& p,const std::string& text){std::ofstream o(p,std::ios::binary);o<<text;return bool(o);}
std::string ReadAll(const fs::path& p){std::ifstream i(p,std::ios::binary);std::ostringstream o;o<<i.rdbuf();return o.str();}
bool Has(const std::string& text,const char* part){return text.find(part)!=std::string::npos;}
bool Has(const std::string& text,const std::string& part){return text.find(part)!=std::string::npos;}

int64_t g_hz=0,g_maxAge=0;

// One body: its fixture (recorded merged rig + one recorded frame), its bundle files and the live bone buffer the hook would hand over.
struct Body
{
    const char* label="";const char* folder="";int bodyType=0;
    std::string dir,program,binding;
    Fixture fx;std::vector<float> original;
    uint32_t count=0;uint64_t rig=0x1234567890ull;
    uint8_t desc[64]{};uint64_t tag=0xABCDEF01ull;
    std::vector<uint64_t> names;
};
bool Init(Body& b,const char* label,const char* folder,int bodyType,const std::string& dir)
{
    b.label=label;b.folder=folder;b.bodyType=bodyType;b.dir=dir;
    if(!LoadFixture(dir,b.fx,"stand_armed.s1"))return false;
    b.program=ReadAll(fs::path(dir)/"program.ir");b.binding=ReadAll(fs::path(dir)/"binding.dat");
    b.count=uint32_t(b.fx.hashes.size());b.original=b.fx.bones;b.names=b.fx.hashes;
    return !b.program.empty()&&!b.binding.empty();
}
// Point the descriptor, the hot state and the buffer identity at this body's live buffer (call while disarmed, then arm).
void Bind(Body& b,const float* bones,uint32_t count)
{
    *reinterpret_cast<const float**>(b.desc+kDescBonesOff)=bones;
    *reinterpret_cast<uint32_t*>(b.desc+kDescCountOff)=count;
    *reinterpret_cast<uint64_t*>(b.desc+kDescTagOff)=b.tag;
    g_hot.playerDesc.store(reinterpret_cast<uint64_t>(b.desc));g_hot.playerRig.store(b.rig);g_hot.playerBones.store(count);
    g_hot.trackA.store(b.tag);g_hot.trackB.store(0);g_hot.mmActive.store(true);g_hot.fault.store(false);
    g_hot.perspectiveWriteMaxAge.store(g_maxAge);
}
int64_t Publish(const Body& b,bool tpp,int bodyType,float weight,int64_t offset)
{
    LARGE_INTEGER now{};QueryPerformanceCounter(&now);
    g_hot.perspectiveWrite.Publish(tpp,now.QuadPart);
    mm::integ::PoseSnapshot snap;snap.rig=b.rig;snap.weight=weight;snap.bodyType=bodyType;snap.publishTicks=now.QuadPart+offset;
    g_mmPose.Publish(snap);
    return now.QuadPart;
}
mm::mappingcycle::Receipt ReceiptAt(int64_t now,bool complete)
{
    mm::mappingcycle::Receipt r;r.status=mm::mappingcycle::ConsumeStatus::Complete;r.complete=complete;
    r.required=r.covered=r.equal=complete?kOwned:kOwned-1;r.startedTicks=now-20;r.lastTicks=now-10;return r;
}
uint32_t Changed(const Body& b,const std::vector<float>& bones)
{
    uint32_t n=0;for(uint32_t i=0;i<b.count;++i)if(std::memcmp(&bones[size_t(i)*12],&b.original[size_t(i)*12],48)!=0)++n;return n;
}
bool Call(Body& b,std::vector<float>& bones,uint32_t count,const mm::mappingcycle::Receipt& r,int64_t now)
{ return br::Evaluate(b.desc,reinterpret_cast<uint8_t*>(bones.data()),count,b.rig,r,now); }

// Install skin_helpers/mode.txt + the given bundle files for each body folder (null = folder absent).
struct Files{const std::string* program;const std::string* binding;};
void Install(const fs::path& work,const char* mode,Files man,Files woman)
{
    const fs::path root=work/"skin_helpers";
    std::error_code ec;fs::remove_all(root,ec);fs::create_directories(root);
    WriteText(root/"mode.txt",mode);
    const std::pair<const char*,Files> folders[2]={{"man",man},{"woman",woman}};
    for(const auto& f:folders)
    {
        if(!f.second.program)continue;
        fs::create_directories(root/f.first);
        WriteText(root/f.first/"program.ir",*f.second.program);WriteText(root/f.first/"binding.dat",*f.second.binding);
    }
}
Files Of(const Body& b){return {&b.program,&b.binding};}
constexpr Files kNone{nullptr,nullptr};
std::string Tampered(const std::string& program)
{
    std::string t=program;const size_t op=t.find("\nOP ");const size_t dot=t.find('.',op);
    t[dot+1]=(t[dot+1]=='7'?'3':'7');return t;
}
std::string TamperedBinding(const std::string& binding)
{
    std::string bad=binding;const size_t hp=bad.find("OWNED ");bad[hp+6]=(bad[hp+6]=='0'?'1':'0');return bad;
}

// What Evaluate wrote equals the offline reference of this body (same tolerances as BodyHelperBundleTest).
void CheckReference(const Body& b,const std::vector<float>& written,const std::string& what)
{
    const auto expected=LoadExpected(b.dir,"stand_armed.s1");
    double mt=0,mq=0,ms=0;uint32_t matched=0;
    for(uint32_t i=0;i<b.count;++i)
    {
        if(std::memcmp(&written[size_t(i)*12],&b.original[size_t(i)*12],48)==0)continue;
        const auto it=expected.find(b.fx.hashes[i]);if(it==expected.end())continue;
        const float* r=&written[size_t(i)*12];const auto& e=it->second;++matched;
        mt=std::fmax(mt,std::sqrt(std::pow(r[0]-e[0],2)+std::pow(r[1]-e[1],2)+std::pow(r[2]-e[2],2)));
        ms=std::fmax(ms,std::sqrt(std::pow(r[8]-e[7],2)+std::pow(r[9]-e[8],2)+std::pow(r[10]-e[9],2)));
        mq=std::fmax(mq,AngleDeg(r+4,e.data()+3));
    }
    std::printf("%s: %u records, max translation %.2e m, rotation %.4f deg, scale %.2e\n",what.c_str(),matched,mt,mq,ms);
    check(matched==kOwned,what+": every written record has a reference value");
    check(mt<2e-5&&mq<0.005&&ms<2e-5,what+": records written by Evaluate equal the offline reference (< 0.002 cm, < 0.005 deg)");
}

// Everything one body's bundle must do on its own: admission through Load, the rig guard, the hot gate with its negative controls.
void Suite(Body& S,const Body& O,const fs::path& work)
{
    const std::string L=std::string(S.label)+": ";
    const fs::path bundle=work/"skin_helpers";
    g_hot.armed.store(false);
    std::vector<float>& bones=S.fx.bones;
    auto restore=[&]{bones=S.original;};
    auto call=[&](const mm::mappingcycle::Receipt& r,int64_t now){return Call(S,bones,S.count,r,now);};
    auto changed=[&]{return Changed(S,bones);};
    auto publish=[&](bool tpp,int body,float weight,int64_t offset){return Publish(S,tpp,body,weight,offset);};

    // --- admission through the real Load(): only this body's folder exists
    Install(work,"APPLY\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);
    check(br::Load(work.wstring())&&br::ApplyEnabled(),L+"approved bundle: APPLY loads");
    check(Has(br::Status(),S.folder)&&Has(br::Status(),"(body "+std::to_string(S.bodyType)+")"),L+"load status names the folder and the body");
    {
        const std::string tampered=Tampered(S.program),bad=TamperedBinding(S.binding);
        Install(work,"APPLY\n",S.folder==std::string("man")?Files{&tampered,&S.binding}:kNone,S.folder==std::string("woman")?Files{&tampered,&S.binding}:kNone);
        check(!br::Load(work.wstring())&&!br::ApplyEnabled()&&!br::Enabled(),L+"modified program.ir: APPLY refused, nothing enabled");
        Install(work,"APPLY\n",S.folder==std::string("man")?Files{&S.program,&bad}:kNone,S.folder==std::string("woman")?Files{&S.program,&bad}:kNone);
        check(!br::Load(work.wstring())&&!br::ApplyEnabled(),L+"modified binding.dat: APPLY refused");
        // the same approved files in the other body's folder are refused (an approved bundle must sit in the folder of its body)
        Install(work,"APPLY\n",S.folder==std::string("woman")?Of(S):kNone,S.folder==std::string("man")?Of(S):kNone);
        check(!br::Load(work.wstring())&&!br::ApplyEnabled()&&Has(br::Status(),"wrong folder"),L+"approved bundle in the other body's folder: refused");
        Install(work,"OFF\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);
        check(br::Load(work.wstring())&&!br::Enabled(),L+"mode OFF: loads, nothing enabled");
        Install(work,"PROBE\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);
        check(br::Load(work.wstring())&&br::Enabled()&&!br::ApplyEnabled(),L+"mode PROBE: enabled, APPLY off");
    }
    // --- arm (cold path) with the approved bundle
    Install(work,"APPLY\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);
    check(br::Load(work.wstring()),L+"reload APPLY");
    const auto catalog=br::RequiredCatalog();
    check(catalog.size()==kOwned,L+"required-helper catalog = the 68 owned helpers of the approved bundle");
    const uint64_t rig=S.rig;
    std::vector<uint64_t> names(S.names);
    const uint32_t rigCount=S.count;
    // --- rig guard (cold path): the rig must have at least the 619-bone base and the bones the bundle reads and writes must have the
    // approved layout (names + parent names). Other bones (hair, accessories, garment dangles) and bone order do not matter. Each
    // refused arm publishes no plan (status says so).
    {
        auto refused=[&](std::span<const int16_t> parents,std::span<const uint64_t> rigNames,const char* label,const char* reason)
        {
            const bool ok=br::PrepareArm(rig,parents,rigNames,catalog);
            check(!ok&&Has(br::Status(),"arm refused")&&Has(br::Status(),reason),L+label);
        };
        auto accepted=[&](std::span<const int16_t> parents,std::span<const uint64_t> rigNames,const char* label)
        {
            const bool ok=br::PrepareArm(rig,parents,rigNames,catalog);
            check(ok&&Has(br::Status(),"arm plan prepared"),L+label);
        };
        refused({S.fx.parents.data(),294},{names.data(),294},"294-bone loading rig: arm refused (smaller than the merged player rig)","smaller");
        check(Has(br::Status(),"619"),L+"refusal status names the required bone count");
        refused({S.fx.parents.data(),618},{names.data(),618},"618 bones, one fewer than the base: arm refused","smaller");
        // equipment / customisation: hair-like chains under Head and bones of any other kind are extra bones, never a reason to refuse
        {
            std::vector<uint64_t> hair(names);std::vector<int16_t> hairParents(S.fx.parents);
            for(uint32_t i=0;i<26;++i){hair.push_back(0xABC000ull+i);hairParents.push_back(int16_t(i<7?1:int(hair.size())-8));}
            accepted(hairParents,hair,"619 base bones + 26 extra hair-like bones: arm accepted");
            std::vector<uint64_t> front(hair);std::vector<int16_t> frontParents(hairParents);
            std::reverse(front.begin(),front.end()); // every bone at a different index: the plan is built by name
            std::vector<int16_t> remapped(frontParents.size());
            const int32_t last=int32_t(frontParents.size())-1;
            for(int32_t i=0;i<=last;++i){const int32_t old=last-i;const int32_t p=frontParents[old];remapped[i]=int16_t(p<0?-1:last-p);}
            accepted(remapped,front,"the same rig in reverse bone order: arm accepted (index independent)");
        }
        {
            std::vector<uint64_t> swapped(names);std::swap(swapped[rigCount-1],swapped[rigCount-2]);
            accepted(S.fx.parents,swapped,"two leaf bones outside the bundle swapped: arm accepted");
            std::vector<int16_t> reparented(S.fx.parents);reparented[rigCount-1]=int16_t(reparented[rigCount-1]==1?2:1);
            accepted(reparented,names,"a bone outside the bundle with another parent: arm accepted");
        }
        // a bone the bundle reads and writes: another parent, or absent from the rig
        {
            size_t owned=0;for(;owned<names.size();++owned)if(names[owned]==catalog[0])break;
            check(owned<names.size(),L+"test setup: an owned helper bone is in the rig");
            std::vector<int16_t> reparented(S.fx.parents);reparented[owned]=int16_t(reparented[owned]==0?1:0);
            refused(reparented,names,"an owned helper bone with another parent: arm refused","differ from the approved layout");
            std::vector<uint64_t> renamed(names);renamed[owned]=0xDEADBEEFull;
            refused(S.fx.parents,renamed,"an owned helper bone missing from the rig: arm refused","differ from the approved layout");
            // the bone above the owned bone (its parent chain is part of the signature)
            const size_t parent=size_t(S.fx.parents[owned]);
            std::vector<int16_t> grand(S.fx.parents);grand[parent]=int16_t(grand[parent]==0?1:0);
            refused(grand,names,"the parent of an owned helper bone has another parent: arm refused","differ from the approved layout");
        }
        std::vector<int16_t> parentsOutOfRange(S.fx.parents);parentsOutOfRange[5]=int16_t(rigCount);
        check(!br::PrepareArm(rig,parentsOutOfRange,names,catalog),L+"a parent index outside the rig: arm refused");
    }
    check(br::PrepareArm(rig,S.fx.parents,names,catalog),L+"PrepareArm accepts the merged player rig");

    // --- hot path state
    std::vector<float>& original=S.original;
    Bind(S,bones.data(),rigCount);g_hot.armed.store(true);
    check(earlyruntime::CurrentOwner()==earlyruntime::Owner::FK,L+"session owner is FK (shipped default, no early-pose experiment)");

    // 1) the shipped situation: FK owner, this body, third person, complete receipt -> exactly the 68 helper records are written
    {
        int64_t now=publish(true,S.bodyType,1.f,-1);
        const bool ok=call(ReceiptAt(now,true),now);
        check(ok&&changed()==kOwned,L+"FK owner, TPP, complete receipt: 68 helper records written");
        CheckReference(S,bones,L+"Evaluate vs reference");
        check(br::ReadCounters().applied>=1&&br::ReadCounters().appliedRecords>=kOwned,L+"applied counters report the write");
        // a second Evaluate of the already corrected buffer: the patch is built from it, so it must not change anything further
        std::vector<float> once=bones;now=publish(true,S.bodyType,1.f,-1);
        check(call(ReceiptAt(now,true),now)&&std::memcmp(once.data(),bones.data(),once.size()*4)==0,L+"idempotent: re-running on the corrected pose changes nothing");
        restore();
    }
    // 2) negative controls: every guard leaves the buffer untouched and is counted under its own reason
    {
        const auto before=br::ReadCounters();
        int64_t now=publish(true,O.bodyType,1.f,-1);  // the other body (its bundle is not loaded)
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"other body: no write");
        now=publish(true,-1,1.f,-1);                  // unknown body
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"unknown body: no write");
        now=publish(false,S.bodyType,1.f,-1);         // first person
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"first-person view (perspective gate closed): no write");
        now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,false),now)&&changed()==0,L+"incomplete mapping-cycle receipt: no write");
        now=publish(true,S.bodyType,0.f,-1);          // MM weight 0
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"pose weight 0: no write");
        now=publish(true,S.bodyType,1.f,-g_hz*60);    // stale pose snapshot
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"stale pose snapshot: no write");
        now=publish(true,S.bodyType,1.f,-1);
        g_hot.mmActive.store(false);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"motion matching inactive: no write");
        g_hot.mmActive.store(true);
        g_hot.fault.store(true);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"fault latched: no write");
        g_hot.fault.store(false);
        ragdoll::g_episode.Begin(0.0);ragdoll::g_episode.Tick(true,0.1);
        check(ragdoll::GameOwned(),L+"test setup: ragdoll episode owns the pose");
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"ragdoll episode (engine owns the pose): no write");
        ragdoll::g_episode.Abort(5.0);
        check(!ragdoll::GameOwned(),L+"test setup: ragdoll episode released");
        float* misaligned=bones.data();
        check(!br::Evaluate(S.desc,reinterpret_cast<uint8_t*>(misaligned)+2,rigCount,rig,ReceiptAt(now,true),now)&&changed()==0,L+"bone buffer identity mismatch: no write");
        check(!br::Evaluate(S.desc,reinterpret_cast<uint8_t*>(bones.data()),rigCount,rig+1,ReceiptAt(now,true),now)&&changed()==0,L+"rig identity mismatch: no write");
        const auto after=br::ReadCounters();
        check(after.rejectReason[6]>=before.rejectReason[6]+4,L+"body / weight / stale snapshot rejections counted under 'body'");
        check(after.rejectReason[4]>=before.rejectReason[4]+1,L+"first-person rejection counted under 'perspective'");
        check(after.rejectReason[5]>=before.rejectReason[5]+1,L+"receipt rejection counted under 'receipt'");
        check(after.rejectReason[1]>=before.rejectReason[1]+1,L+"ragdoll rejection counted under 'ragdoll'");
        check(after.applied==before.applied,L+"no negative control produced a write");
    }
    // 2b) identity and permission-callback negative controls: the hot gate and the final recheck inside Commit each leave the buffer untouched
    {
        auto before=br::ReadCounters();
        int64_t now=publish(true,S.bodyType,1.f,-1);
        // another descriptor with the same rig, bone count and tag
        uint8_t otherDesc[64]{};std::memcpy(otherDesc,S.desc,sizeof S.desc);
        check(!br::Evaluate(otherDesc,reinterpret_cast<uint8_t*>(bones.data()),rigCount,rig,ReceiptAt(now,true),now)&&changed()==0,L+"different descriptor (same rig, count, tag): no write");
        check(!br::Evaluate(S.desc,reinterpret_cast<uint8_t*>(bones.data()),rigCount-1,rig,ReceiptAt(now,true),now)&&changed()==0,L+"wrong bone count: no write");
        check(!br::Evaluate(nullptr,reinterpret_cast<uint8_t*>(bones.data()),rigCount,rig,ReceiptAt(now,true),now)&&changed()==0,L+"null descriptor: no write");
        auto mid=br::ReadCounters();
        check(mid.rejectReason[3]>=before.rejectReason[3]+3,L+"descriptor / count identity rejections counted under 'identity'");
        // the hot gate does not look at the tag or at the descriptor's own buffer fields: those are rechecked inside Commit right before the write
        const uint64_t goodTag=*reinterpret_cast<uint64_t*>(S.desc+kDescTagOff);
        *reinterpret_cast<uint64_t*>(S.desc+kDescTagOff)=goodTag^0x5555ull;
        now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"descriptor tag is neither track A nor B: permission recheck refuses, no write");
        *reinterpret_cast<uint64_t*>(S.desc+kDescTagOff)=goodTag;
        std::vector<float> otherBones(bones);
        *reinterpret_cast<float**>(S.desc+kDescBonesOff)=otherBones.data();
        now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"descriptor points at another bone buffer: permission recheck refuses, no write");
        *reinterpret_cast<float**>(S.desc+kDescBonesOff)=bones.data();
        *reinterpret_cast<uint32_t*>(S.desc+kDescCountOff)=rigCount-1;
        now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"descriptor count changed: permission recheck refuses, no write");
        *reinterpret_cast<uint32_t*>(S.desc+kDescCountOff)=rigCount;
        auto afterCtl=br::ReadCounters();
        check(afterCtl.commitRejected>=mid.commitRejected+3,L+"the three descriptor rechecks are counted as commit rejections");
        check(afterCtl.applied==before.applied,L+"no identity / permission control produced a write");
        // and with everything restored the same call writes
        now=publish(true,S.bodyType,1.f,-1);
        check(call(ReceiptAt(now,true),now)&&changed()==kOwned,L+"descriptor restored: writes again");
        restore();
    }
    // 3) the same state writes again once the guards are satisfied (the controls did not latch anything)
    {
        int64_t now=publish(true,S.bodyType,1.f,-1);
        check(call(ReceiptAt(now,true),now)&&changed()==kOwned,L+"guards satisfied again: writes again");
        restore();
    }
    // 4) PROBE computes a proposal but never writes
    {
        Install(work,"PROBE\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);g_hot.armed.store(false);
        check(br::Load(work.wstring())&&br::PrepareArm(rig,S.fx.parents,names,br::RequiredCatalog()),L+"PROBE reload and arm");
        g_hot.armed.store(true);
        const auto before=br::ReadCounters();
        int64_t now=publish(true,S.bodyType,1.f,-1);
        check(call(ReceiptAt(now,true),now)&&changed()==0&&br::ReadCounters().applied==before.applied,L+"PROBE: proposal only, no record written");
    }
    // 5) Disable (fault handler path) stops everything
    {
        Install(work,"APPLY\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);g_hot.armed.store(false);
        check(br::Load(work.wstring())&&br::PrepareArm(rig,S.fx.parents,names,br::RequiredCatalog()),L+"APPLY reload and arm");
        g_hot.armed.store(true);br::Disable();
        int64_t now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"after Disable: no write");
    }
    // 6) a refused arm leaves no plan: every call is counted as identity (not gate) and writes nothing
    {
        Install(work,"APPLY\n",S.folder==std::string("man")?Of(S):kNone,S.folder==std::string("woman")?Of(S):kNone);g_hot.armed.store(false);
        check(br::Load(work.wstring()),L+"APPLY reload for the no-plan control");
        check(!br::PrepareArm(rig,{S.fx.parents.data(),294},{names.data(),294},br::RequiredCatalog()),L+"arm refused on the 294-bone rig");
        g_hot.armed.store(true);
        const auto before=br::ReadCounters();
        int64_t now=publish(true,S.bodyType,1.f,-1);
        check(!call(ReceiptAt(now,true),now)&&changed()==0,L+"no plan (refused arm): no write");
        const auto after=br::ReadCounters();
        check(after.rejectReason[3]==before.rejectReason[3]+1&&after.rejectReason[2]==before.rejectReason[2],L+"a missing plan is counted as identity, not gate");
        g_hot.armed.store(false);
        check(br::PrepareArm(rig,S.fx.parents,names,br::RequiredCatalog()),L+"re-arm with the real rig after a refusal");
        g_hot.armed.store(true);
        now=publish(true,S.bodyType,1.f,-1);
        check(call(ReceiptAt(now,true),now)&&changed()==kOwned,L+"a refusal does not latch: the approved rig writes again");
        restore();
    }
    // 7) the same rig with 26 extra bones (hair-like chains): it arms and writes the same 68 records (equipment / customisation independence)
    {
        const uint32_t extra=26,extCount=rigCount+extra;
        std::vector<uint64_t> extNames(names);std::vector<int16_t> extParents(S.fx.parents);
        std::vector<float> ext(original);ext.resize(size_t(extCount)*12,0.f);
        for(uint32_t i=0;i<extra;++i)
        {
            extNames.push_back(0xABC000ull+i);extParents.push_back(int16_t(i<7?1:int(extNames.size())-8));
            float* r=&ext[size_t(rigCount+i)*12];r[7]=1.f;r[8]=r[9]=r[10]=1.f;
        }
        g_hot.armed.store(false);
        Bind(S,ext.data(),extCount);
        check(br::PrepareArm(rig,extParents,extNames,br::RequiredCatalog()),L+"rig with 26 extra bones: arm accepted");
        g_hot.armed.store(true);
        int64_t now=publish(true,S.bodyType,1.f,-1);
        const bool ok=br::Evaluate(S.desc,reinterpret_cast<uint8_t*>(ext.data()),extCount,rig,ReceiptAt(now,true),now);
        ext.resize(size_t(rigCount)*12);
        check(ok&&Changed(S,ext)==kOwned,L+"rig with 26 extra bones: the same 68 helper records are written");
        CheckReference(S,ext,L+"extra-bones rig vs reference");
        g_hot.armed.store(false);
        Bind(S,bones.data(),rigCount);
    }
    restore();
}

// Both bundles together: each body gets its own bundle, a missing / unapproved / refused bundle of one body leaves the other working,
// and the rig alone can not select the body (the man and woman rigs have the same bundle layout; the pose snapshot decides).
void Cross(Body& man,Body& woman,const fs::path& work)
{
    const std::string manBad=Tampered(man.program),womanBad=Tampered(woman.program);
    for(Body* rigBody:{&man,&woman})
    {
        Body& R=*rigBody;
        const std::string L=std::string("both bundles, live rig of the ")+R.label+": ";
        std::vector<float>& bones=R.fx.bones;
        auto reset=[&]{bones=R.original;};
        // loads the given bundle files, arms on R's rig, publishes the given body and evaluates once; returns whether it wrote
        std::string loadStatus; // status right after Load (PrepareArm replaces it)
        auto run=[&](Files m,Files w,int bodyType,std::vector<float>& out)
        {
            out=R.original;g_hot.armed.store(false);
            Install(work,"APPLY\n",m,w);
            const bool loaded=br::Load(work.wstring());loadStatus=br::Status();
            if(!loaded||!br::ApplyEnabled())return false;
            Bind(R,bones.data(),R.count);
            if(!br::PrepareArm(R.rig,R.fx.parents,R.names,br::RequiredCatalog()))return false;
            g_hot.armed.store(true);
            reset();
            const int64_t now=Publish(R,true,bodyType,1.f,-1);
            const bool ok=Call(R,bones,R.count,ReceiptAt(now,true),now);
            out=bones;reset();
            return ok&&Changed(R,out)==kOwned;
        };
        std::vector<float> onlyMan,onlyWoman,both,none;
        check(run(Of(man),kNone,kBodyMan,onlyMan),L+"man bundle only, man body: writes");
        check(run(kNone,Of(woman),kBodyWoman,onlyWoman),L+"woman bundle only, woman body: writes (the man bundle folder is absent)");
        check(!run(Of(man),kNone,kBodyWoman,none)&&Changed(R,none)==0,L+"man bundle only, woman body: no write");
        check(!run(kNone,Of(woman),kBodyMan,none)&&Changed(R,none)==0,L+"woman bundle only, man body: no write");
        check(onlyMan!=onlyWoman,L+"the two bundles produce different records on the same pose (the selection below is meaningful)");
        check(run(Of(man),Of(woman),kBodyMan,both)&&both==onlyMan,L+"both bundles, man body: exactly the man bundle's records");
        check(run(Of(man),Of(woman),kBodyWoman,both)&&both==onlyWoman,L+"both bundles, woman body: exactly the woman bundle's records");
        check(!run(Of(man),Of(woman),-1,none)&&Changed(R,none)==0,L+"both bundles, unknown body: no write");
        // one bundle unusable: the other body keeps working, the broken body writes nothing
        {
            const Files badMan{&manBad,&man.binding},badWoman{&womanBad,&woman.binding};
            check(run(badMan,Of(woman),kBodyWoman,both)&&both==onlyWoman,L+"man bundle modified: the woman bundle still loads and writes");
            check(Has(loadStatus,"skipped")&&Has(loadStatus,"man:"),L+"load status reports the skipped man bundle");
            check(!run(badMan,Of(woman),kBodyMan,none)&&Changed(R,none)==0,L+"man bundle modified: man body writes nothing");
            check(run(Of(man),badWoman,kBodyMan,both)&&both==onlyMan,L+"woman bundle modified: the man bundle still loads and writes");
            check(!run(Of(man),badWoman,kBodyWoman,none)&&Changed(R,none)==0,L+"woman bundle modified: woman body writes nothing");
            g_hot.armed.store(false);
            Install(work,"APPLY\n",badMan,badWoman);
            check(!br::Load(work.wstring())&&!br::Enabled(),L+"both bundles modified: nothing loads");
            // man files in the woman folder (and vice versa): refused, the correctly placed bundle is unaffected
            check(run(Of(man),Of(man),kBodyMan,both)&&both==onlyMan,L+"man bundle also in the woman folder: that copy is refused, the man bundle works");
            check(Has(loadStatus,"wrong folder"),L+"load status says wrong folder");
        }
        // the required-helper catalog is the same list for either bundle, so one list serves both
        {
            g_hot.armed.store(false);
            Install(work,"APPLY\n",Of(man),kNone);br::Load(work.wstring());const auto cm=br::RequiredCatalog();
            Install(work,"APPLY\n",kNone,Of(woman));br::Load(work.wstring());const auto cw=br::RequiredCatalog();
            Install(work,"APPLY\n",Of(man),Of(woman));br::Load(work.wstring());const auto cb=br::RequiredCatalog();
            check(cm.size()==kOwned&&cm==cw&&cm==cb,L+"required-helper catalog: same 68 helpers for the man bundle, the woman bundle and both");
        }
        reset();
    }
}

// Host (v1) publishes carry the body the script reported (mm::integ::SetKnownBody, sent by motiondriver::SetRigBody) through the same
// stamp the host uses (mm::integ::StampHostIdentity). The recompute is admitted on them exactly like on a driver pose; an unknown body
// stays rejected. The coverage counters attribute every call to the writers of its core write record (mm/CoreWriteRecord.hpp).
int64_t PublishHost(const Body& b,float weight)
{
    LARGE_INTEGER now{};QueryPerformanceCounter(&now);
    g_hot.perspectiveWrite.Publish(true,now.QuadPart);
    mm::integ::PoseSnapshot snap;snap.rig=b.rig;snap.weight=weight;snap.publishTicks=now.QuadPart-1;
    snap.bodyType=7;snap.source=mm::integ::kPoseSourceDriver; // a reused snapshot: the stamp must replace both
    mm::integ::StampHostIdentity(snap);
    g_mmPose.Publish(snap);
    return now.QuadPart;
}
void HostPublished(Body& man,Body& woman,const fs::path& work)
{
    namespace cw=mm::corewrite;
    for(Body* live:{&man,&woman})
    {
        Body& R=*live;Body& O=live==&man?woman:man;
        const std::string L=std::string("host publish, live body ")+R.label+": ";
        std::vector<float>& bones=R.fx.bones;
        // arm with the given bundles on R's rig
        auto arm=[&](Files m,Files w)
        {
            g_hot.armed.store(false);Install(work,"APPLY\n",m,w);
            const bool ok=br::Load(work.wstring())&&br::ApplyEnabled();
            bones=R.original;Bind(R,bones.data(),R.count);
            const bool armed=ok&&br::PrepareArm(R.rig,R.fx.parents,R.names,br::RequiredCatalog());
            g_hot.armed.store(true);g_hot.mmActive.store(true);
            return armed;
        };
        auto evaluateHost=[&](float weight,uint32_t writers)
        {
            bones=R.original;
            cw::Begin();cw::MarkPlayerCall();
            for(uint32_t w=0;w<cw::kWriterCount;++w)if(writers&(1u<<w))cw::Mark(cw::Writer(w));
            const int64_t now=PublishHost(R,weight);
            const bool ok=Call(R,bones,R.count,ReceiptAt(now,true),now);
            check((cw::Current()&cw::kEvaluated)!=0,L+"Evaluate marks the call as evaluated in the core write record");
            return ok;
        };
        const uint32_t host=cw::Bit(cw::kHost);
        // the stamp itself
        {
            mm::integ::SetKnownBody(R.bodyType);
            mm::integ::PoseSnapshot s;s.bodyType=-1;s.source=mm::integ::kPoseSourceDriver;mm::integ::StampHostIdentity(s);
            check(s.bodyType==R.bodyType&&s.source==mm::integ::kPoseSourceHost,L+"host stamp: reported body, host source");
            mm::integ::SetKnownBody(5);mm::integ::StampHostIdentity(s);
            check(mm::integ::KnownBody()==-1&&s.bodyType==-1,L+"an invalid reported body is stored as unknown (-1)");
        }
        // 1) only R's bundle loaded, R reported: the host pose is admitted and commits the reference records
        check(arm(R.folder==std::string("man")?Of(R):kNone,R.folder==std::string("woman")?Of(R):kNone),L+"arm with this body's bundle");
        {
            mm::integ::SetKnownBody(R.bodyType);
            const auto before=br::ReadCoverage();
            check(evaluateHost(1.f,host)&&Changed(R,bones)==kOwned,L+"host pose, body reported: 68 helper records written");
            CheckReference(R,bones,L+"host pose vs reference");
            const auto after=br::ReadCoverage();
            check(after.byWriter[cw::kHost][size_t(br::Outcome::Committed)]==before.byWriter[cw::kHost][size_t(br::Outcome::Committed)]+1,
                  L+"coverage: the commit is counted under the host writer");
            check(after.calls==before.calls+1,L+"coverage: one call counted");
        }
        // 2) negative controls on host poses: unknown body, the other body (its bundle is not loaded), weight 0
        {
            const auto before=br::ReadCoverage();
            mm::integ::SetKnownBody(-1);
            check(!evaluateHost(1.f,host)&&Changed(R,bones)==0,L+"host pose, body not reported (-1): no write");
            mm::integ::SetKnownBody(O.bodyType);
            check(!evaluateHost(1.f,host)&&Changed(R,bones)==0,L+"host pose, the other body reported (no bundle for it): no write");
            mm::integ::SetKnownBody(R.bodyType);
            check(!evaluateHost(0.f,host)&&Changed(R,bones)==0,L+"host pose at weight 0: no write");
            const auto after=br::ReadCoverage();
            check(after.byWriter[cw::kHost][size_t(br::Outcome::Body)]==before.byWriter[cw::kHost][size_t(br::Outcome::Body)]+3,
                  L+"coverage: the three refusals are counted as 'body' under the host writer");
            check(after.byWriter[cw::kHost][size_t(br::Outcome::Committed)]==before.byWriter[cw::kHost][size_t(br::Outcome::Committed)],
                  L+"coverage: no commit counted for the refusals");
        }
        // 3) both bundles loaded, the reported body changes during the session: each host pose gets exactly the bundle of the reported body
        check(arm(Of(man),Of(woman)),L+"arm with both bundles");
        {
            std::vector<float> asR,asO,again;
            mm::integ::SetKnownBody(R.bodyType);
            check(evaluateHost(1.f,host)&&Changed(R,bones)==kOwned,L+"both bundles, body reported as this body: writes");
            asR=bones;
            mm::integ::SetKnownBody(O.bodyType);
            check(evaluateHost(1.f,host)&&Changed(R,bones)==kOwned,L+"both bundles, body changed to the other body: writes");
            asO=bones;
            check(asR!=asO,L+"the body change switches the bundle (different records on the same pose)");
            mm::integ::SetKnownBody(R.bodyType);
            check(evaluateHost(1.f,host)&&bones==asR,L+"body changed back: exactly the first bundle's records again");
            // the host records equal those of a driver pose of the same body (the existing driver path)
            const int64_t now=Publish(R,true,R.bodyType,1.f,-1);bones=R.original;
            check(Call(R,bones,R.count,ReceiptAt(now,true),now)&&bones==asR,L+"host and driver pose of the same body write the same records");
        }
        // 4) coverage per writer: several writers in one call, no writer, weight-0 writers with MM inactive, calls that never reach Evaluate
        {
            mm::integ::SetKnownBody(R.bodyType);
            auto at=[](const br::Coverage& c,size_t w,br::Outcome o){return c.byWriter[w][size_t(o)];};
            auto before=br::ReadCoverage();
            check(evaluateHost(1.f,host|cw::Bit(cw::kNeckGuard)),L+"host + neck guard in one call: commits");
            auto after=br::ReadCoverage();
            check(at(after,cw::kHost,br::Outcome::Committed)==at(before,cw::kHost,br::Outcome::Committed)+1&&
                  at(after,cw::kNeckGuard,br::Outcome::Committed)==at(before,cw::kNeckGuard,br::Outcome::Committed)+1&&
                  after.calls==before.calls+1,L+"coverage: one call with two writers is counted once per writer and once in calls");
            before=after;
            check(evaluateHost(1.f,0),L+"no writer marked: still commits (the recompute does not depend on the record)");
            after=br::ReadCoverage();
            check(at(after,br::kNoWriter,br::Outcome::Committed)==at(before,br::kNoWriter,br::Outcome::Committed)+1,
                  L+"coverage: a call without a core write is counted under 'none'");
            before=after;
            g_hot.mmActive.store(false);
            check(!evaluateHost(0.f,cw::Bit(cw::kVaultTorso)),L+"vault torso at weight 0 (MM inactive): no write");
            g_hot.mmActive.store(true);
            after=br::ReadCoverage();
            check(at(after,cw::kVaultTorso,br::Outcome::Gate)==at(before,cw::kVaultTorso,br::Outcome::Gate)+1,
                  L+"coverage: the weight-0 writer is counted under 'gate'");
            before=after;
            br::NoteNotEvaluated(cw::Bit(cw::kVehicleGuards)|cw::kPlayerCall);
            after=br::ReadCoverage();
            check(at(after,cw::kVehicleGuards,br::Outcome::NotEvaluated)==at(before,cw::kVehicleGuards,br::Outcome::NotEvaluated)+1&&
                  after.calls==before.calls+1,L+"coverage: a call that never reached Evaluate is counted under 'noeval'");
        }
        mm::integ::SetKnownBody(-1);
        bones=R.original;g_hot.armed.store(false);
    }
}
} // namespace

int main(int argc,char** argv)
{
    if(argc<3){std::printf("usage: BodyHelperRuntimeTest <man fixture dir> <woman fixture dir>\n");return 2;}
    LARGE_INTEGER hz{};QueryPerformanceFrequency(&hz);g_hz=hz.QuadPart;g_maxAge=hz.QuadPart*30;
    Body man,woman;
    check(Init(man,"man","man",kBodyMan,argv[1]),"man fixture loads");
    check(Init(woman,"woman","woman",kBodyWoman,argv[2]),"woman fixture loads");
    if(g_failures)return 1;
    check(man.count==619&&woman.count==645,"fixtures are the recorded merged rigs (man 619 bones, woman 645 bones: the 619-bone base plus 26 hair bones)");
    const fs::path root=fs::temp_directory_path()/("utpp_helper_runtime_test_"+std::to_string(GetCurrentProcessId()));
    fs::create_directories(root/"man");fs::create_directories(root/"woman");fs::create_directories(root/"both");
    Suite(man,woman,root/"man");
    Suite(woman,man,root/"woman");
    Cross(man,woman,root/"both");
    HostPublished(man,woman,root/"both");
    std::error_code ec;fs::remove_all(root,ec);
    std::printf("%d checks, %d failures\n",g_checks,g_failures);
    return g_failures?1:0;
}

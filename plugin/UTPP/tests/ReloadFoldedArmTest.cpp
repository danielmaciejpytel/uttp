// Offline test: the authored reload contact solve must reach a native FPP hand folded close to the shoulder.
// Recorded 21.10 03:37 k0 (empty magazine, standing): the native left hand sat 0.093m from the shoulder of a
// 0.566m arm (0.164 of its length) for 7 frames; the default GripIK fold clamp (minReach .20) stopped at 0.113m,
// the contact missed by up to 20mm, Finish rejected the frame and the whole overlay switched off (forearm pop 69deg).
// Usage: ReloadFoldedArmTest <woman.utppov>
// Mutation 3 (UTPP_RELOAD_CONTACT_MUTATION=3) restores the default clamp for authored clips: the test must fail.
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/OverlayFile.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
using namespace utpp::mm;
static int g_pass=0,g_fail=0;
static void Check(bool ok,const char* what){(ok?g_pass:g_fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);}
static void Records(const ov::Pose& p,std::vector<unsigned char>& out)
{
    out.resize(size_t(p.n)*48);
    for(int b=0;b<p.n;++b){const auto& x=p.b[b];float r[12]={x.t.x,x.t.y,x.t.z,1,x.r.x,x.r.y,x.r.z,x.r.w,1,1,1,1};std::memcpy(out.data()+size_t(b)*48,r,sizeof(r));}
}
int main(int argc,char** argv)
{
    if(argc!=2)return 2;
    ov::OverlayFile file;std::string error;
    if(!file.LoadFromFile(argv[1],ov::kRigIdentityWomanOv,error)){std::printf("load %s\n",error.c_str());return 2;}
    const auto& sk=file.Skel();const auto rig=gik::FindRig(sk.n,[&](int i){return file.BoneName(i);});
    if(!rig.Valid(sk))return 2;
    for(const auto overlay:{ov::PoseOverlay::Pistol2H,ov::PoseOverlay::Pistol1H})
    {
        const int index=file.FindOverlayPose(overlay,ov::AimState::Ready,false,false);if(index<0)return 2;
        auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
        // Native pose: the same body with the LEFT hand folded to 0.15 of the arm length from the shoulder.
        auto native=std::make_unique<ov::Pose>(*base);
        const float armLength=gik::Len(base->b[rig.left.lower].t)+gik::Len(base->b[rig.left.hand].t);
        const auto shoulder=native_reload::Model(sk,*base,rig.left.upper).t;
        auto hand=native_reload::Model(sk,*base,rig.left.hand).t;
        math::Vec3 dir;if(!gik::Norm(hand-shoulder,dir))return 2;
        // tilt the direction towards the chest so the fold is not along the authored line
        const math::Vec3 target=shoulder+dir*(armLength*0.15f);
        gik::Params foldParams;foldParams.minReach=0.05f;gik::Result foldResult;
        if(!gik::SolveArm(sk,*native,rig.left,target,nullptr,1.0f,foldParams,foldResult))return 2;
        const float nativeDistance=gik::Len(native_reload::Model(sk,*native,rig.left.hand).t-shoulder)/armLength;
        std::printf("overlay=%d native left hand at %.3f of the arm length from the shoulder (clamp default .20)\n",int(overlay),nativeDistance);
        Check(nativeDistance<0.18f,"fixture: native left hand is folded inside the default GripIK clamp");
        reload_contacts_runtime::Prepared prepared;prepared.rig=123;prepared.skeleton=sk;prepared.arms=rig;
        integ::PoseSnapshot snap;snap.rig=123;snap.sequence=7;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(overlay);
        snap.reloadAuthoredClip=true;snap.upperScale=0;snap.traceAction=2;
        const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
        std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
        for(int b=0;b<sk.n;++b){prepared.gameIndex[b]=snap.gameIndex[b]=b;const auto& x=base->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));}
        auto run=[&](float minReach,bool useDefaultArg,std::vector<unsigned char>& records,native_reload_contacts::Result& result)
        {
            Records(*native,records);reload_contacts_runtime::Cycle cycle;
            const bool captured=reload_contacts_runtime::Capture(prepared,snap,records.data(),sk.n,true,cycle);
            Records(*base,records);
            if(!captured)return false;
            return useDefaultArg ? reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f)
                                 : reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f,minReach);
        };
        std::vector<unsigned char> fixedRecords,legacyRecords;native_reload_contacts::Result fixedResult,legacyResult;
        const bool fixedOk=run(0,true,fixedRecords,fixedResult);
        const bool legacyOk=run(0.0f,false,legacyRecords,legacyResult);
        std::printf("  default constant: accepted=%d reject=%d contact=%.2fmm | solver default clamp: accepted=%d reject=%d contact=%.2fmm\n",
                    fixedOk,int(fixedResult.reject),fixedResult.contactMax*1000,legacyOk,int(legacyResult.reject),legacyResult.contactMax*1000);
        Check(!legacyOk&&legacyResult.reject==native_reload_contacts::Reject::Contact,"solver default fold clamp reproduces the recorded Contact rejection");
        Check(fixedOk&&fixedResult.contactMax<1.0e-3f,"authored reload accepts the folded native hand within the 1mm contact check");
        if(fixedOk)
        {
            ov::Pose final;const bool read=reload_contacts_runtime::Read(prepared,fixedRecords.data(),final);
            const float miss=read?gik::Len(native_reload::Model(sk,final,rig.left.hand).t-native_reload::Model(sk,*native,rig.left.hand).t):1.0f;
            Check(read&&miss<1.0e-3f,"left hand lands on the native contact point");
            const float weaponMiss=read?gik::Len(native_reload::Model(sk,final,rig.weaponRight).t-native_reload::Model(sk,*native,rig.weaponRight).t):1.0f;
            Check(weaponMiss<1.0e-4f,"weapon binding stays on the native weapon pose");
        }
    }
    // Poses that never fold below the default clamp: the new minimum changes nothing (byte identical records).
    {
        const int index=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Aiming,false,false);if(index<0)return 2;
        auto raw=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*raw);
        reload_contacts_runtime::Prepared prepared;prepared.rig=123;prepared.skeleton=sk;prepared.arms=rig;
        integ::PoseSnapshot snap;snap.rig=123;snap.sequence=9;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(ov::PoseOverlay::Pistol2H);
        snap.reloadAuthoredClip=true;snap.upperScale=0;snap.traceAction=2;
        const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
        std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
        for(int b=0;b<sk.n;++b){prepared.gameIndex[b]=snap.gameIndex[b]=b;const auto& x=raw->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));}
        std::vector<unsigned char> a,b2;native_reload_contacts::Result ra,rb;bool ok[2];
        for(int variant=0;variant<2;++variant)
        {
            auto& records=variant?b2:a;auto& result=variant?rb:ra;
            Records(*raw,records);reload_contacts_runtime::Cycle cycle;
            const bool captured=reload_contacts_runtime::Capture(prepared,snap,records.data(),sk.n,true,cycle);
            Records(*raw,records);
            ok[variant]=captured&&(variant?reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f,0.0f)
                                         :reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f));
        }
        Check(ok[0]==ok[1]&&a==b2,"unfolded native pose: records identical with and without the lower fold clamp");
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

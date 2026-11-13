// Replays the same-cycle reload contact correction (Capture -> blend -> Finish) on recorded pose CSV frames.
// utpp-pose-<T0>.csv rows: ticks,desc,thread,rig,seq,publish,action,overlay,aiming,clip,weight,upper,bone,gameIndex,masked,
// s0_0..11 (game), s1_0..11 (after Blend), s2_0..11 (after AO/Finish), mm_0..6. Stage 1 is the baseline that Finish receives,
// stage 0 the native pose that Capture reads, the mm columns the snapshot pose.
// args: pose.csv woman.utppov T0 tStart tEnd weight [authored=1] [out.csv|-] [minReach (0 = solver default)]
// Finish gets the production Continuity state (placement gate, divergence latch, reject hold) with the CSV QPC ticks;
// REPLAY_NO_CONTINUITY=1 runs the stateless path.
//   tStart/tEnd: seconds after T0 (same time base as the extracted npz meta). weight: correction weight handed to Finish.
// Output per frame: t,seq,action,overlay,accepted,reject,contactMax,dLeftHand_vs_recorded_cm,dRightHand_cm,dWeapon_cm
// REPLAY_DUMP=<file> appends one wide row per frame (header on a new file): t,accepted,reject,action,upper,weight, then MODEL positions
// (x,y,z, metres, Z up) of kDumpBones for four poses: native (stage 0 = what Capture read), authored (stage 1 = records handed to
// Finish), out (records after Finish) and rec (stage 2 as recorded by the game build). Frames that were not captured are skipped.
#include "../../plugin/UTPP/src/mm/NativeReloadContactsRuntime.hpp"
#include "../../plugin/UTPP/src/mm/OverlayFile.hpp"
#include "../../plugin/UTPP/src/mm/ReloadContactFade.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace utpp::mm;
namespace rt=reload_contacts_runtime;

struct Row{long long ticks=0;long long seq=0;int action=0,overlay=0,aiming=0,clip=0;float weight=0,upper=0;int bone=0,gameIndex=-1,masked=0;
           float s[3][12];float mm[7];};
static bool ParseRow(const std::string& line,Row& r)
{
    const char* p=line.c_str();char* end=nullptr;
    std::vector<double> f;f.reserve(58);
    while(*p)
    {
        f.push_back(std::strtod(p,&end));
        if(end==p){f.push_back(std::nan(""));while(*p&&*p!=',')++p;}
        else p=end;
        if(*p==',')++p;else break;
    }
    if(f.size()<58)return false;
    r.ticks=(long long)f[0];r.seq=(long long)f[4];r.action=(int)f[6];r.overlay=(int)f[7];r.aiming=(int)f[8];r.clip=(int)f[9];
    r.weight=(float)f[10];r.upper=(float)f[11];r.bone=(int)f[12];r.gameIndex=(int)f[13];r.masked=(int)f[14];
    for(int s=0;s<3;++s)for(int k=0;k<12;++k)r.s[s][k]=(float)f[15+s*12+k];
    for(int k=0;k<7;++k)r.mm[k]=(float)f[51+k];
    return true;
}
static void Put(const float* r12,uint8_t* dst){std::memcpy(dst,r12,48);}
static const char* const kDumpBones[]={"Hips","Spine3","Neck","Head","LeftShoulder","RightShoulder","LeftArm","RightArm",
                                       "LeftForeArm","RightForeArm","LeftHand","RightHand","WeaponRight"};
static const char* const kDumpPoses[]={"nat","aut","out","rec"};
int main(int argc,char** argv)
{
    if(argc<7){std::fprintf(stderr,"usage: pose.csv woman.utppov T0 tStart tEnd weight [authored=1] [out.csv]\n");return 2;}
    const long long t0=std::atoll(argv[3]);const double ts=std::atof(argv[4]),te=std::atof(argv[5]);
    const float weight=(float)std::atof(argv[6]);const bool authored=argc>7?std::atoi(argv[7])!=0:true;
    const float minReach=argc>9?(float)std::atof(argv[9]):rt::kAuthoredContactMinReach;
    FILE* out=(argc>8&&std::string(argv[8])!="-")?std::fopen(argv[8],"w"):stdout;
    ov::OverlayFile file;std::string error;
    if(!file.LoadFromFile(argv[2],ov::kRigIdentityWomanOv,error)){std::cerr<<error;return 2;}
    const auto& sk=file.Skel();
    // Production topology (MotionDriver::RemapLocked): the cold readMap maps every bone, Root and Trajectory included.
    std::vector<int32_t> coldMap(size_t(sk.n));for(int i=0;i<sk.n;++i)coldMap[size_t(i)]=i;
    rt::Prepared prepared;
    if(!rt::FillPrepared(prepared,0x1234,uint32_t(sk.n),coldMap.data(),[&](uint32_t i){return int(sk.parent[i]);},
                         [&](uint32_t i){return file.BoneName(int(i));})){std::cerr<<"rig invalid\n";return 2;}
    std::ifstream in(argv[1]);std::string line;std::getline(in,line);std::getline(in,line);
    std::vector<Row> frame;long long curTicks=-1;
    std::fprintf(out,"t,seq,action,overlay,upper,weight,accepted,reject,contactMax,dLH_cm,dRH_cm,dWR_cm,lErr,rErr,lReq,lUse,rReq,rUse,lenL,lenR,jumpFA,jumpUA,lhHipZ_cm,wpnHead_cm\n");
    std::vector<uint8_t> rec;bool prepReady=false;int frames=0,accepted=0;
    ov::Pose prevPose,prevBase,prevNative;bool havePrev=false,havePrevBase=false;
    rt::Continuity continuity;const bool useContinuity=std::getenv("REPLAY_NO_CONTINUITY")==nullptr; // production state (FkHook keeps it between frames)
    int hipsBone=-1,headBone=-1;for(int i=0;i<sk.n;++i){if(file.BoneName(i)=="Hips")hipsBone=i;if(file.BoneName(i)=="Head")headBone=i;}
    FILE* dump=nullptr;int dumpBone[sizeof(kDumpBones)/sizeof(kDumpBones[0])];
    if(const char* dumpPath=std::getenv("REPLAY_DUMP"))
    {
        for(size_t k=0;k<sizeof(dumpBone)/sizeof(dumpBone[0]);++k)
        {
            dumpBone[k]=-1;for(int i=0;i<sk.n;++i)if(file.BoneName(i)==kDumpBones[k])dumpBone[k]=i;
            if(dumpBone[k]<0){std::cerr<<"dump bone missing: "<<kDumpBones[k]<<"\n";return 2;}
        }
        dump=std::fopen(dumpPath,"a");
        if(!dump){std::cerr<<"cannot open REPLAY_DUMP\n";return 2;}
        std::fseek(dump,0,SEEK_END);
        if(std::ftell(dump)==0)
        {
            std::fprintf(dump,"t,accepted,reject,action,upper,weight");
            for(const char* pose:kDumpPoses)for(const char* bone:kDumpBones)std::fprintf(dump,",%s_%s_x,%s_%s_y,%s_%s_z",pose,bone,pose,bone,pose,bone);
            std::fprintf(dump,"\n");
        }
    }
    const auto flush=[&](){
        if(frame.empty())return;
        const Row& h=frame[0];const double t=double(h.ticks-t0)/1e7;
        if(t<ts||t>te){frame.clear();return;}
        // The game calls Finish only while the reload correction is active (native action 2, or its action-0 tail while the
        // upper-body handback is still fading). Rows before the episode (action 0, upper == 1) never reach it.
        if(h.action==0&&h.upper>=0.999f){frame.clear();return;}
        int maxGame=-1;for(const Row& r:frame)maxGame=std::max(maxGame,r.gameIndex);
        const uint32_t count=uint32_t(maxGame+1);
        (void)prepReady;
        rec.assign(size_t(count)*48,0);
        static const float kIdentity[12]={0,0,0,1,0,0,0,1,1,1,1,1};
        std::memcpy(rec.data(),kIdentity,48);if(count>1)std::memcpy(rec.data()+48,kIdentity,48); // Root, Trajectory
        integ::PoseSnapshot snap{};snap.rig=prepared.rig;snap.numBones=sk.n;snap.firstBone=2;snap.weight=h.weight;snap.upperScale=h.upper;
        snap.traceAction=h.action;snap.traceOverlay=h.overlay;snap.sequence=uint64_t(h.seq);snap.reloadAuthoredClip=authored;snap.reloadContactsEligible=true;
        snap.reloadContactsWeight=weight;
        for(int i=0;i<sk.n;++i)snap.gameIndex[i]=i>=2 ? prepared.gameIndex[i] : -1; // g_map omits Root / Trajectory
        for(const Row& r:frame)
        {
            if(r.bone>=sk.n)continue;
            float* d=snap.pose+size_t(r.bone)*7;for(int k=0;k<7;++k)d[k]=r.mm[k];
            if(r.masked)snap.upperBits[r.bone>>5]|=1u<<(r.bone&31);
        }
        // stage 0 -> Capture
        for(const Row& r:frame)if(r.gameIndex>=0)Put(r.s[0],rec.data()+size_t(r.gameIndex)*48);
        rt::Cycle cycle;const bool captured=rt::Capture(prepared,snap,rec.data(),count,true,cycle);
        native_reload::g_diag.Clear();
        // stage 1 = records handed to Finish
        for(const Row& r:frame)if(r.gameIndex>=0)Put(r.s[1],rec.data()+size_t(r.gameIndex)*48);
        native_reload_contacts::Result result;
        const bool done=captured&&rt::Finish(prepared,snap,rec.data(),count,true,cycle,result,weight,minReach,
                                             useContinuity?&continuity:nullptr,h.ticks,10000000);
        if(std::getenv("REPLAY_DEBUG"))std::fprintf(stderr,"t=%.3f placement=%.3f divergence gate=%.3f engaged=%d reject amp=%.3f\n",t,continuity.placement.value,continuity.divergence.gate,int(continuity.divergence.engaged),continuity.reject.amp);
        // compare with recorded stage 2 (hand/weapon model positions)
        ov::Pose mine,recorded;float dl=-1,dr=-1,dw=-1;
        std::vector<uint8_t> rec2(size_t(count)*48,0);
        std::memcpy(rec2.data(),kIdentity,48);if(count>1)std::memcpy(rec2.data()+48,kIdentity,48); // Root, Trajectory (Read wants unit scale)
        for(const Row& r:frame)if(r.gameIndex>=0)Put(r.s[2],rec2.data()+size_t(r.gameIndex)*48);
        if(rt::Read(prepared,rec.data(),mine)&&rt::Read(prepared,rec2.data(),recorded))
        {
            dl=gik::Len(native_reload::Model(sk,mine,prepared.arms.left.hand).t-native_reload::Model(sk,recorded,prepared.arms.left.hand).t)*100;
            dr=gik::Len(native_reload::Model(sk,mine,prepared.arms.right.hand).t-native_reload::Model(sk,recorded,prepared.arms.right.hand).t)*100;
            dw=gik::Len(native_reload::Model(sk,mine,prepared.arms.weaponRight).t-native_reload::Model(sk,recorded,prepared.arms.weaponRight).t)*100;
        }
        // diagnostics: per-arm upper-arm local jumps of the candidate, the baseline and the native pose
        ov::Pose baseNow;float jc[2]={0,0},jb[2]={0,0},jn[2]={0,0};
        {
            std::vector<uint8_t> baseRec(size_t(count)*48,0);
            std::memcpy(baseRec.data(),kIdentity,48);if(count>1)std::memcpy(baseRec.data()+48,kIdentity,48);
            for(const Row& r:frame)if(r.gameIndex>=0)Put(r.s[1],baseRec.data()+size_t(r.gameIndex)*48);
            if(rt::Read(prepared,baseRec.data(),baseNow)&&captured)
            {
                if(havePrevBase)
                {
                    int side=0;
                    for(const auto* arm:{&prepared.arms.left,&prepared.arms.right})
                    {
                        jb[side]=ov::AngleDeg(baseNow.b[arm->upper].r,prevBase.b[arm->upper].r);
                        jn[side]=ov::AngleDeg(cycle.native.b[arm->upper].r,prevNative.b[arm->upper].r);
                        ++side;
                    }
                }
                prevBase=baseNow;prevNative=cycle.native;havePrevBase=true;
            }
        }
        float jumpFA=0,jumpUA=0;
        if(havePrev&&rt::Read(prepared,rec.data(),mine))
        {
            for(const auto* arm:{&prepared.arms.left,&prepared.arms.right})
            {
                jumpFA=std::max(jumpFA,ov::AngleDeg(mine.b[arm->lower].r,prevPose.b[arm->lower].r));
                jumpUA=std::max(jumpUA,ov::AngleDeg(mine.b[arm->upper].r,prevPose.b[arm->upper].r));
                jc[arm==&prepared.arms.left?0:1]=ov::AngleDeg(mine.b[arm->upper].r,prevPose.b[arm->upper].r);
            }
        }
        if(std::getenv("REPLAY_JOINTS")&&havePrev&&rt::Read(prepared,rec.data(),mine))
        {
            int tr[8];const int nc=rt::RejectHold::Chains(prepared,tr);
            std::fprintf(stderr,"JOINTS t=%.3f",t);
            for(int k=0;k<nc;++k)std::fprintf(stderr," b%d:%.1f",tr[k],ov::AngleDeg(mine.b[tr[k]].r,prevPose.b[tr[k]].r));
            std::fprintf(stderr,"\n");
        }
        if(rt::Read(prepared,rec.data(),prevPose))havePrev=true;
        if(std::getenv("REPLAY_DIAG"))
        {
            // right arm is oriented first, then the left arm (Apply order); entries carry the upper bone index
            float turn[2]={0,0},proj[2]={0,0},xport[2]={0,0},applied[2]={0,0};
            for(int i=0;i<native_reload::g_diag.n;++i)
            {
                const auto& d=native_reload::g_diag.e[i];const int side=d.upper==prepared.arms.left.upper ? 0 : 1;
                turn[side]=d.turnDeg;proj[side]=d.projection;xport[side]=d.transportDeg;applied[side]=d.appliedDeg;
            }
            std::fprintf(stderr,"DIAG t=%.3f acc=%d | candJump L%.1f R%.1f | baseJump L%.1f R%.1f | nativeJump L%.1f R%.1f | turn L%.1f R%.1f applied L%.1f R%.1f | proj L%.3f R%.3f | transport L%.1f R%.1f\n",
                         t,int(done),jc[0],jc[1],jb[0],jb[1],jn[0],jn[1],turn[0],turn[1],applied[0],applied[1],proj[0],proj[1],xport[0],xport[1]);
        }
        float lhHip=0,wHead=0;
        if(hipsBone>=0&&headBone>=0&&rt::Read(prepared,rec.data(),mine))
        {
            lhHip=(native_reload::Model(sk,mine,prepared.arms.left.hand).t.z-native_reload::Model(sk,mine,hipsBone).t.z)*100;
            wHead=gik::Len(native_reload::Model(sk,mine,prepared.arms.weaponRight).t-native_reload::Model(sk,mine,headBone).t)*100;
        }
        std::fprintf(out,"%.4f,%lld,%d,%d,%.3f,%.3f,%d,%d,%.6f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f\n",t,h.seq,h.action,h.overlay,h.upper,h.weight,
                     done?1:0,captured?int(result.reject):-1,result.contactMax,dl,dr,dw,result.left.handErr,result.right.handErr,
                     result.left.reqDist,result.left.useDist,result.right.reqDist,result.right.useDist,
                     captured?gik::Len(cycle.native.b[prepared.arms.left.lower].t)+gik::Len(cycle.native.b[prepared.arms.left.hand].t):0.0f,
                     captured?gik::Len(cycle.native.b[prepared.arms.right.lower].t)+gik::Len(cycle.native.b[prepared.arms.right.hand].t):0.0f,jumpFA,jumpUA,lhHip,wHead);
        if(dump&&captured)
        {
            ov::Pose outPose,recPose,autPose;
            std::vector<uint8_t> autRec(size_t(count)*48,0);
            std::memcpy(autRec.data(),kIdentity,48);if(count>1)std::memcpy(autRec.data()+48,kIdentity,48);
            for(const Row& r:frame)if(r.gameIndex>=0)Put(r.s[1],autRec.data()+size_t(r.gameIndex)*48);
            if(rt::Read(prepared,rec.data(),outPose)&&rt::Read(prepared,rec2.data(),recPose)&&rt::Read(prepared,autRec.data(),autPose))
            {
                std::fprintf(dump,"%.4f,%d,%d,%d,%.3f,%.3f",t,done?1:0,int(result.reject),h.action,h.upper,h.weight);
                const ov::Pose* poses[4]={&cycle.native,&autPose,&outPose,&recPose};
                for(const ov::Pose* pose:poses)for(int b:dumpBone)
                {
                    const auto m=native_reload::Model(sk,*pose,b);
                    std::fprintf(dump,",%.5f,%.5f,%.5f",m.t.x,m.t.y,m.t.z);
                }
                std::fprintf(dump,"\n");
            }
        }
        ++frames;accepted+=done?1:0;frame.clear();
    };
    while(std::getline(in,line))
    {
        // cheap time filter on the first field
        const long long ticks=std::atoll(line.c_str());
        const double t=double(ticks-t0)/1e7;
        if(t>te+1)break;
        if(t<ts-1)continue;
        Row r;if(!ParseRow(line,r))continue;
        if(r.ticks!=curTicks){flush();curTicks=r.ticks;}
        frame.push_back(r);
    }
    flush();
    std::fprintf(stderr,"frames %d accepted %d\n",frames,accepted);
    if(dump)std::fclose(dump);
    if(out!=stdout)std::fclose(out);
    return 0;
}

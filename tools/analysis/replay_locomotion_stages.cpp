#include <filesystem>
// Recorded per-feed playback through the real driver and leg tail; no game/input access.
#define main FullIntegrationSuiteNotRun
#include "../../plugin/UTPP/tests/MotionDriverIntegrationTest.cpp"
#undef main
static motionhost::MotionSetFrame g_tail;
static mm::integ::PoseSnapshot g_preLeg;
static bool g_preLegValid=false;
bool CaptureTail(const int32_t* map, uint32_t n, motionhost::MotionSetFrame& frame)
{
    const bool ok = motiondriver::Frame(map,n,frame);
    if (ok) {
        g_preLegValid=true;g_preLeg.numBones=frame.numBones;g_preLeg.firstBone=2;
        for(uint32_t k=0;k<frame.numBones;++k){const auto& v=frame.pose[k];float* p=&g_preLeg.pose[k*7];p[0]=v.p.x;p[1]=v.p.y;p[2]=v.p.z;p[3]=v.q.x;p[4]=v.q.y;p[5]=v.q.z;p[6]=v.q.w;g_preLeg.gameIndex[k]=map[k];}
        g_tail.contact[0]=frame.contact[0];g_tail.contact[1]=frame.contact[1];g_tail.rootVel[0]=frame.rootVel[0];g_tail.rootVel[1]=frame.rootVel[1];g_tail.rootYawRate=frame.rootYawRate; }
    return ok;
}
#ifndef UTPP_REPLAY_MAIN
#define UTPP_REPLAY_MAIN main
#endif
int UTPP_REPLAY_MAIN(int argc,char**argv)
{
    if(argc!=5 && argc!=6 && argc!=8)return 2;
    const std::string data=argv[1],feedPath=argv[3];
    FeedFile f;Rig rig;std::unique_ptr<mm::motion::MotionDatabase> db;
    auto womanPaths=DataPaths(data,"woman");if(argc>=6)womanPaths.files.db=argv[5];
    if(!RigFromDb(womanPaths.files.db,rig,&db))return 3;
    std::vector<motionhost::FeedInput> inputs;
    std::ifstream fin(feedPath);std::string line;
    while(std::getline(fin,line)) {
        if(line.empty()||line[0]=='#')continue;
        std::istringstream stream(line);std::vector<std::string> v;std::string token;while(stream>>token)v.push_back(token);
                if(v.size()<17)return 6;
        Frame fr;fr.t=std::stof(v[0]);fr.dt=std::stof(v[1]);fr.vx=std::stof(v[2]);fr.vy=std::stof(v[3]);fr.vz=std::stof(v[4]);
        fr.mvx=std::stof(v[5]);fr.mvy=std::stof(v[6]);fr.yawDeg=std::stof(v[7]);fr.camDeg=std::stof(v[8]);fr.loco=std::stoi(v[9]);fr.psm=int(std::stoul(v[10]));
        f.fr.push_back(fr);
        auto in=ToInput(fr);
        if(v.size()>=31) {
            in.haveTarget=std::stoi(v[17])!=0;in.targetYawDeg=std::stof(v[18]);
            in.haveExtra=true;for(int j=0;j<3;++j)in.capsulePos[j]=std::stof(v[19+j]);
            in.haveAir=true;in.onGround=std::stoi(v[27])!=0;in.distToGround=std::stof(v[28]);in.psmFall=std::stoi(v[29]);in.walkToggled=std::stoi(v[30])!=0;
        }
        inputs.push_back(in);
    }
    const motionhost::MotionHooks hooks{&WStep,&WApply,&motiondriver::OnArmed,&motiondriver::OnDisarmed,&CaptureTail,&motiondriver::OwnsTraversal};
    g_baseDb=Wide(argv[2]);motiondriver::SetPathsOverride(0,womanPaths);motiondriver::SetPathsOverride(1,DataPaths(data,"man"));
    motiondriver::SetRigBody(0);motiondriver::SetOverlayInputs(0,0,false,false);motionhost::SetMotionHooks(&hooks);
    if(!ResetBase(rig))return 4;
    char* noOffset=nullptr;size_t noOffsetSize=0;
    const bool offsetDisabled=_dupenv_s(&noOffset,&noOffsetSize,"UTPP_STAGE_NO_OFFSET")==0&&noOffset!=nullptr;
    std::free(noOffset);
    if(offsetDisabled)motionhost::SetOffsetRoot(false);
    // Controlled stat sample, not telemetry recovered from the feed file.
    // REDscript utpp.reds calls UTPPUpdateAccel AFTER UTPPFeedLocomotion;
    // therefore the sample belongs to the next feed's prediction, including
    // ordinary preroll. Default leaves the missing-stat fallback untouched.
    char* accelProfile=nullptr;size_t accelProfileSize=0;
    const bool controlledAccel = _dupenv_s(&accelProfile,&accelProfileSize,"UTPP_REPLAY_ACCEL_391")==0 &&
        accelProfile!=nullptr && std::strcmp(accelProfile,"1")==0;
    std::free(accelProfile);
    if(controlledAccel)
        std::puts("controlledStatInjection AccelTarget(base=24,applied=3.91/24) AFTER Feed; not recorded native telemetry");
    motiondriver::SetEnabled(true);motiondriver::Step(inputs.front(),1,inputs.front().dt);motiondriver::WaitLoader();
    std::vector<int> bones;
    for(auto name:{"Hips","Spine3","LeftUpLeg","LeftLeg","LeftFoot","LeftToeBase","RightUpLeg","RightLeg","RightFoot","RightToeBase"}) {
        int found=-1;for(uint32_t b=0;b<db->NumBones();++b)if(db->BoneName(b)==name)found=int(b);if(found<0)return 5;bones.push_back(found);
    }
    std::ofstream csv(argv[4]);csv.precision(9);csv<<"t,dt,owned,clip,clipT,contactL,contactR,rootX,rootY";
    for(auto stage:{"pre_","final_"})for(int b:bones)for(auto axis:{"x","y","z"})csv<<','<<stage<<db->BoneName(b)<<axis;csv<<",playRate,actorYaw,rootYawRate,offX,offY,offYaw,offYawGoal,offClamps,offYawClamps,offResets,offTeleports,offNonFinite,capsuleAccel,cancelSupport,lockL,wL,corrL,lockR,wR,corrR\n";
    std::vector<float> poseTimes;
    if(argc==8){std::ifstream points(argv[6]);float t;while(points>>t)poseTimes.push_back(t);std::filesystem::create_directories(argv[7]);}
    int missing=0,bad=0,owned=0,controlledTargetMatches=0;std::string why;
    for(size_t i=0;i<inputs.size();++i) {
        g_log={};g_preLegValid=false;motionhost::Feed(inputs[i]);
        if(controlledAccel) {
            const float requestedTarget=motionhost::AccelTarget(24.f,3.91f/24.f);
            if(std::fabs(requestedTarget-3.91f)<0.01f) ++controlledTargetMatches;
        }
        const bool published=ReadPublished();owned+=g_log.stepResult;
        if(!published){++missing;continue;}
        if(g_log.stepResult&&!MotionPoseOk(g_read,rig,why))++bad;
        const std::string status=motiondriver::Status();
        const std::string pick=StatusField(status,"pick=clip:");const auto at=pick.find('@'),slash=pick.find("/db");
        const int clip=std::atoi(pick.c_str());const float ct=at==std::string::npos?0:float(std::atof(pick.substr(at+1,slash-at-1).c_str()));
        csv<<f.fr[i].t<<','<<f.fr[i].dt<<','<<g_log.stepResult<<','<<clip<<','<<ct<<','<<g_tail.contact[0]<<','<<g_tail.contact[1]<<','<<g_tail.rootVel[0]<<','<<g_tail.rootVel[1];
        for(const auto* snap:{&g_preLeg,&g_read})for(int b:bones){float p[3]={};if(!g_preLegValid||!ModelPos(*snap,rig,b,p))++bad;for(float v:p)csv<<','<<v;}
        csv<<','<<StatusField(status,"rate=")<<','<<inputs[i].playerYawDeg<<','<<g_tail.rootYawRate;
        const auto v1status=motionhost::Status();for(auto field:{"offX=","offY=","offYaw=","offYawGoal=","offClamps=","offYawClamps=","offResets=","offTeleports=","offNonFinite="})csv<<','<<std::atof(StatusField(v1status,field).c_str());
        csv<<','<<StatusField(status,"capsuleAccel=")<<','<<StatusField(status,"cancelSupport=");
        for(auto field:{"lockL=","wL=","corrL=","lockR=","wR=","corrR="})csv<<','<<std::atof(StatusField(v1status,field).c_str());
        csv<<'\n';
        if(argc==8) for(size_t j=0;j<poseTimes.size();++j) if(std::fabs(f.fr[i].t-poseTimes[j])<0.0001f) {
            std::ofstream p(std::filesystem::path(argv[7])/("checkpoint-"+std::to_string(j)+".pose.txt"));
            p.precision(9);p<<"UTPP_POSE_CHECKPOINT\nPOSE "<<db->NumBones()<<'\n';
            for(uint32_t k=0;k<db->NumBones();++k){p<<db->BoneName(k);for(int c=0;c<7;++c)p<<' '<<g_read.pose[size_t(k)*7+c];p<<" 1 1 1\n";}
        }
    }
    motionhost::SetMotionHooks(nullptr);motiondriver::Shutdown();motionhost::Shutdown();
    std::printf("recordedFeed frames=%zu owned=%d missing=%d badPose=%d\n",inputs.size(),owned,missing,bad);
    if(controlledAccel)
        std::printf("controlledStatInjection calls=%zu requestedTargetNear3.91=%d; mismatching targets do not prove accepted cache samples\n",
                    inputs.size(),controlledTargetMatches);
    return missing||bad?1:0;
}

// Contact-level regression for the recorded crouch radius hops and stop tidy pop.
// A real database supplies the skeleton; deterministic bent legs isolate the
// procedural correction from clip selection and rendering. No game access.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#ifdef UTPP_FOOTLOCK_HEADER
#include UTPP_FOOTLOCK_HEADER
#else
#include "../src/mm/FootLocker.hpp"
#endif
using namespace utpp::mm;
static int passed=0, failed=0;
static void Check(bool ok,const char* what) { std::printf("%s %s\n",ok?"PASS":"FAIL",what);ok?++passed:++failed; }
template<class T> static void Policy(T& f,bool on)
{
    // Lets the identical regression run against the saved pre-fix header.
    if constexpr(requires { f.SetCrouchContactPolicy(on); }) f.SetCrouchContactPolicy(on);
}
static float Length(math::Vec3 v) { return std::sqrt(math::Dot(v,v)); }
static math::Quat X(float deg) { return legik::AxisAngle({1,0,0},deg*math::kPi/180.f); }
struct Fixture
{
    FootLocker f;
    std::vector<BoneTransform> pose;
    int hips=-1;
    explicit Fixture(const Database& db,int policy=1,FootLockConfig cfg={}):pose(db.NumBones())
    {
        f.Init(db,cfg);f.SetReplant(true,true);f.SetPoleCarry(true);if(policy>=0)Policy(f,policy!=0);
        for(uint32_t i=0;i<db.NumBones();++i)if(db.BoneName(i)=="Hips")hips=int(i);
        pose[size_t(hips)].p={0,0,1};
        for(int s=0;s<2;++s)
        {
            pose[size_t(f.ChainBone(s,0))]={{s==0?0.1f:-0.1f,0,0},X(35)};
            pose[size_t(f.ChainBone(s,1))]={{0,0,-0.45f},X(-70)};
            pose[size_t(f.ChainBone(s,2))]={{0,0,-0.45f},X(35)};
            if(f.LockBone(s)!=f.ChainBone(s,2))pose[size_t(f.LockBone(s))].p={0,0.14f,-0.04f};
        }
        for(int i=0;i<30;++i)Step(1.f/60,0,1,1);
    }
    void Step(float dt,float vy=0,float cl=1,float cr=1) { f.Update(dt,0,vy,0,cl,cr,pose); }
    void SeedIdleResidual()
    {
        // Establish the measured 5.6cm offset at a quiet source speed, rather
        // than introducing a one-frame teleport that is itself an active stop.
        for(int i=0;i<28;++i)Step(1.f/60,0.12f);
    }
    math::Vec3 World(int side) const
    {
        auto p=f.ModelSpace(f.Output(),f.LockBone(side)).p;
        f.CharacterToWorld(p,p.x,p.y);return p;
    }
    bool Protected() const
    {
        for(size_t i=0;i<pose.size();++i)
        {
            bool leg=false;for(int s=0;s<2;++s)for(int k=0;k<3;++k)leg|=int(i)==f.ChainBone(s,k);
            if(!leg&&std::memcmp(&pose[i],&f.Output()[i],sizeof(BoneTransform))!=0)return false;
            if(!math::IsFinite(f.Output()[i].p)||!math::IsFinite(f.Output()[i].q))return false;
            if(std::memcmp(&pose[i].p,&f.Output()[i].p,sizeof(math::Vec3))!=0)return false;
        }
        return true;
    }
};
static void Moving(const Database& db)
{
    Fixture x(db);x.f.SetReplant(true,false);
    auto prev=x.World(0);float maxStep=0,maxLift=0,maxSafeSlide=0;bool guards=true;
    for(int i=0;i<48;++i)
    {
        x.Step(1.f/60,0.5f);
        const auto p=x.World(0);maxStep=std::max(maxStep,Length(p-prev));prev=p;
        maxLift=std::max(maxLift,std::fabs(x.f.ModelSpace(x.f.Output(),x.f.ChainBone(0,2)).p.z-
            x.f.ModelSpace(x.pose,x.f.ChainBone(0,2)).p.z));
        if(i<6)maxSafeSlide=std::max(maxSafeSlide,std::fabs(p.y-0.14f));
        guards&=x.Protected()&&x.f.Diagnostics().foot[0].locked&&x.f.Diagnostics().foot[1].locked;
    }
    std::printf("moving maxStep=%.4fcm maxLift=%.4fcm correction=%.3fcm\n",maxStep*100,maxLift*100,x.f.Diagnostics().maxCorrection*100);
    Check(maxLift<0.0002f,"moving crouch adds no vertical hop during authored support");
    Check(maxStep<0.012f,"moving crouch necessary planar give has no replant snap");
    Check(x.f.Diagnostics().maxCorrection<0.15f&&guards,"moving support remains locked/reachable; protected bones and lengths unchanged");
    Check(maxSafeSlide<0.0002f,"safe moving support stays planted before radius excess");
    for(int i=0;i<16;++i){x.pose[size_t(x.hips)].p.z=1.f+0.03f*std::sin(i*0.1f);x.Step(1.f/60,0,0,0);}
    Check(!x.f.Diagnostics().foot[0].locked&&x.f.Diagnostics().foot[0].weight==0,
          "authored toe-off releases without retaining a planted foot");
    Check(std::memcmp(x.pose.data(),x.f.Output().data(),x.pose.size()*sizeof(BoneTransform))==0,
          "after release the authored swing passes through exactly");
}
static void Tidy(const Database& db,int hz)
{
    Fixture x(db);const float dt=1.f/hz;
    x.SeedIdleResidual(); // measured left residual at the 13.3s event
    math::Vec3 prev[2]={x.World(0),x.World(1)};
    float maxStep=0,maxLift=0,maxFirst=0;bool protectedBones=true;
    uint64_t updates=0;
    for(int i=0;i<hz*3;++i)
    {
        x.Step(dt);
        const auto& d=x.f.Diagnostics();
        for(int s=0;s<2;++s)
        {
            const auto p=x.World(s);const float step=Length(p-prev[s]);prev[s]=p;
            maxStep=std::max(maxStep,step);
            maxLift=std::max(maxLift,std::fabs(x.f.ModelSpace(x.f.Output(),x.f.ChainBone(s,2)).p.z-
                x.f.ModelSpace(x.pose,x.f.ChainBone(s,2)).p.z));
            if(d.stepUpdates>updates&&updates==0)maxFirst=std::max(maxFirst,step);
            protectedBones&=d.foot[s].locked&&x.Protected();
        }
        updates=d.stepUpdates;
    }
    std::printf("tidy hz=%d maxStep=%.4fcm first=%.4fcm lift=%.4fcm remain=%.3f/%.3fcm\n",hz,
        maxStep*100,maxFirst*100,maxLift*100,x.f.Diagnostics().foot[0].offset*100,x.f.Diagnostics().foot[1].offset*100);
    Check(maxStep<0.12f*dt+0.0004f,"stationary tidy limits displacement rather than a 2.5m/s snap");
    Check(maxFirst<0.002f&&maxLift<0.0002f,"stationary tidy starts gently and adds no sine hop");
    Check(x.f.Diagnostics().foot[0].offset<0.008f&&x.f.Diagnostics().foot[1].offset<0.008f,
          "both feet finish tidy without retaining a large idle error");
    Check(protectedBones&&x.f.Diagnostics().simultaneousSteps==0,"tidy preserves contacts, lengths, upper body and one foot at a time");
    const auto before=std::vector<BoneTransform>(x.f.Output().begin(),x.f.Output().end());x.Step(0);
    Check(std::memcmp(before.data(),x.f.Output().data(),before.size()*sizeof(BoneTransform))==0,"dt zero does not advance a tidy correction");
}
static void FastSupport(const Database& db)
{
    Fixture x(db);x.f.SetReplant(true,false);x.f.SetSlip(0.06f,0.02f);
    bool kept=true;
    for(int i=0;i<30;++i)
    {
        // Fast source/capsule-relative drift as in the recorded rotating start.
        // The 1m/s cap cannot keep up and used to hit maxDrift under contact=1.
        x.Step(1.f/60,3.f);
        kept&=x.f.Diagnostics().foot[0].locked&&x.f.Diagnostics().foot[1].locked&&x.Protected();
    }
    std::printf("fastSupport correction=%.3fcm driftUnlocks=%llu\n",x.f.Diagnostics().maxCorrection*100,
                static_cast<unsigned long long>(x.f.Diagnostics().driftUnlocks));
    Check(kept&&x.f.Diagnostics().driftUnlocks==0,"fast moving support gives necessary excess without emergency contact loss");
    Check(x.f.Diagnostics().maxCorrection<0.15f&&x.f.Diagnostics().maxLift==0,
          "fast moving support stays inside correction guard without adding lift");
    auto prev=x.World(0);float idleStep=0,restartStep=0;bool transitionSafe=true;
    for(int i=0;i<60;++i)
    {
        x.Step(1.f/60,i<30?0.f:0.5f);
        const auto p=x.World(0);const float step=Length(p-prev);prev=p;
        (i<30?idleStep:restartStep)=std::max(i<30?idleStep:restartStep,step);
        transitionSafe&=x.f.Diagnostics().foot[0].locked&&x.f.Diagnostics().foot[1].locked&&x.Protected();
    }
    std::printf("fastSupport transitions idleStep=%.4fcm restartStep=%.4fcm\n",idleStep*100,restartStep*100);
    Check(idleStep<0.12f/60+0.0004f&&restartStep<0.012f&&transitionSafe,
          "moving to idle and back keep contacts with bounded correction and no restart snap");
    for(int i=0;i<16;++i)x.Step(1.f/60,0,0,0);
    Check(x.f.Diagnostics().maxLift==0&&std::memcmp(x.pose.data(),x.f.Output().data(),x.pose.size()*sizeof(BoneTransform))==0,
          "fast-support handover preserves authored swing and never invents vertical lift");
}
static void AnimatedStop(const Database& db)
{
    Fixture x(db);x.f.SetSlip(0.06f,0.02f);
    bool kept=true;
    for(int i=0;i<24;++i)
    {
        // The capsule has stopped, but the authored stop continues to move
        // its toe. Recorded clip173 accelerates its toe to >2m/s here.
        x.pose[size_t(x.hips)].p.y+=std::min(2.2f,0.3f+0.2f*i)/60.f;
        x.Step(1.f/60);
        kept&=x.f.Diagnostics().foot[0].locked&&x.f.Diagnostics().foot[1].locked&&x.Protected();
    }
    std::printf("animatedStop correction=%.3fcm driftUnlocks=%llu\n",x.f.Diagnostics().maxCorrection*100,
                static_cast<unsigned long long>(x.f.Diagnostics().driftUnlocks));
    Check(kept&&x.f.Diagnostics().driftUnlocks==0&&x.f.Diagnostics().maxCorrection<0.15f,
          "stopped capsule does not impose idle rate on a still-moving authored toe");
    const auto before=std::vector<BoneTransform>(x.f.Output().begin(),x.f.Output().end());x.Step(0);
    Check(std::memcmp(before.data(),x.f.Output().data(),before.size()*sizeof(BoneTransform))==0,
          "dt zero does not advance source-motion history or its correction");
    for(int i=0;i<16;++i)x.Step(1.f/60,0,0,0);
    Check(x.f.Diagnostics().maxLift==0&&std::memcmp(x.pose.data(),x.f.Output().data(),x.pose.size()*sizeof(BoneTransform))==0,
          "animated stop still releases at authored toe-off without generated lift");
}
static void Guards(const Database& db)
{
    Fixture a(db,-1),b(db,false);bool same=true;
    for(int i=0;i<90;++i)
    {
        Policy(b.f,false);a.Step(1.f/60,0.25f);b.Step(1.f/60,0.25f);
        same&=std::memcmp(a.f.Output().data(),b.f.Output().data(),a.pose.size()*sizeof(BoneTransform))==0;
    }
    Check(same,"policy disabled preserves ordinary standing/legacy replant exactly");
    Fixture x(db);x.SeedIdleResidual();for(int i=0;i<23;++i)x.Step(1.f/60);
    Policy(x.f,false);x.f.SetReplant(false,false);float maxStep=0;auto prev=x.World(0);
    for(int i=0;i<70;++i){x.Step(1.f/60);auto p=x.World(0);maxStep=std::max(maxStep,Length(p-prev));prev=p;}
    Check(maxStep<0.004f,"crouch policy exit hands over an existing contact without a hop");
    x.pose[size_t(x.hips)].p.y+=0.5f;x.Step(1.f/60);
    Check(x.f.Diagnostics().driftUnlocks>0&&!x.f.Diagnostics().foot[0].locked,"max-drift safety still releases an invalid target");
    x.f.Reset();Policy(x.f,true);x.Step(1.f/60);
    Check(x.Protected()&&x.f.Diagnostics().driftUnlocks==0,"reset removes previous correction state");
    FootLockConfig cfg;cfg.settleSpeed=0;
    Fixture disabled(db,1,cfg);disabled.SeedIdleResidual();
    for(int i=0;i<120;++i)disabled.Step(1.f/60);
    Check(disabled.f.Diagnostics().stepReasons[kFootEventTidy]==0 &&
          !disabled.f.Diagnostics().foot[0].stepping&&!disabled.f.Diagnostics().foot[1].stepping,
          "disabled idle settling does not acquire a permanent tidy owner");
}
int main(int argc,char** argv)
{
    if(argc!=2)return 2;Database db;std::string why;
    if(!db.LoadFromFile(argv[1],0x72CB314EB01F562Dull,why)){std::puts(why.c_str());return 2;}
    Moving(db);for(int hz:{30,60,144})Tidy(db,hz);Guards(db);FastSupport(db);AnimatedStop(db);
    std::printf("SUMMARY %d passed %d failed\n",passed,failed);return failed?1:0;
}

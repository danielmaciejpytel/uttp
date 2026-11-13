#include "../src/mm/NativeReloadContacts.hpp"
#include "../src/mm/OverlayFile.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

using namespace utpp::mm;

static float SignedTurn(math::Vec3 axis,math::Vec3 from,math::Vec3 to)
{
    math::Vec3 a,b;
    if(!gik::Norm(from-axis*math::Dot(from,axis),a) ||
       !gik::Norm(to-axis*math::Dot(to,axis),b))return NAN;
    return std::atan2(math::Dot(axis,math::Cross(a,b)),
                      std::clamp(math::Dot(a,b),-1.0f,1.0f));
}

int main(int argc,char** argv)
{
    if(argc!=2){std::printf("usage: NativeReloadGuideTurnTest <woman.utppov>\n");return 2;}
    int failed=0;
    auto check=[&](bool ok,const char* name)
    {
        std::printf("%s: %s\n",ok?"PASS":"FAIL",name);
        failed+=!ok;
    };
    ov::OverlayFile file;std::string error;
    if(!file.LoadFromFile(argv[1],ov::kRigIdentityWomanOv,error))
    {std::printf("overlay load failed: %s\n",error.c_str());return 2;}
    const auto& sk=file.Skel();
    const auto rig=gik::FindRig(sk.n,[&](int i){return file.BoneName(i);});
    const int poseIndex=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Aiming,false,false);
    if(!rig.Valid(sk)||poseIndex<0){std::printf("Pistol2H Aim rig/pose unavailable\n");return 2;}
    auto source=std::make_unique<ov::Pose>();
    file.FramePose(file.PoseAt(poseIndex).firstFrame,*source);
    const ov::Pose authoredOriginal=*source;
    const auto upper=native_reload::Model(sk,*source,rig.right.upper);
    const auto lower=native_reload::Model(sk,*source,rig.right.lower);
    const auto hand=native_reload::Model(sk,*source,rig.right.hand);
    math::Vec3 axis;
    math::Vec3 oldAxis;
    if(!gik::Norm(hand.t-upper.t,oldAxis)){std::printf("invalid target axis\n");return 2;}
    const float armLength=gik::Len(source->b[rig.right.lower].t)+gik::Len(source->b[rig.right.hand].t);
    const auto target=upper.t+oldAxis*(armLength*0.80f);
    if(!gik::Norm(target-upper.t,axis)){std::printf("invalid synthetic target axis\n");return 2;}
    ov::Pose transported=*source;
    if(!native_reload::ParallelTransport(sk,transported,rig.right,target)){std::printf("parallel transport failed\n");return 2;}
    const auto transportedUpper=native_reload::Model(sk,transported,rig.right.upper);
    const auto transportedLower=native_reload::Model(sk,transported,rig.right.lower);
    const auto q90=gik::AxisAngle(axis,math::kPi*0.5f);
    const auto guide=transportedUpper.t+math::QuatRotate(q90,transportedLower.t-transportedUpper.t);
    const auto initialElbow=transportedLower.t-transportedUpper.t;
    const float cap=math::kPi*0.25f;
    for(const auto [testedCap,name]:{std::pair{cap-0.0001f,"cap just below 45 degrees"},
                                     std::pair{cap,"cap at 45 degrees"},
                                     std::pair{cap+0.0001f,"cap just above 45 degrees"},
                                     std::pair{math::kPi,"default pi cap"}})
    {
        ov::Pose candidate=*source;
        const bool oriented=native_reload::OrientArmPlane(sk,candidate,rig.right,target,guide,1.0f,testedCap);
        const auto movedLower=native_reload::Model(sk,candidate,rig.right.lower);
        const float applied=SignedTurn(axis,initialElbow,movedLower.t-transportedUpper.t);
        const float expected=std::min(math::kPi*0.5f,testedCap);
        check(oriented&&std::isfinite(applied)&&std::fabs(applied-expected)<0.002f,name);
        gik::Params params;gik::Result solve;
        const bool solved=oriented&&gik::SolveArm(sk,candidate,rig.right,target,nullptr,1.0f,params,solve);
        const auto solvedHand=native_reload::Model(sk,candidate,rig.right.hand);
        check(solved&&solve.ok&&gik::Len(solvedHand.t-target)<=0.001f,
              "guide cap keeps target contact after SolveArm");
    }
    check(std::memcmp(source.get(),&authoredOriginal,sizeof(authoredOriginal))==0,
          "source Neutral authored pose remains unchanged");

    ov::Pose nativeGuide=*source;
    const auto sourceLower=native_reload::Model(sk,*source,rig.right.lower);
    const auto sourceUpper=native_reload::Model(sk,*source,rig.right.upper);
    const auto qFade=gik::AxisAngle(oldAxis,math::kPi*0.5f);
    const auto guide90=sourceUpper.t+math::QuatRotate(qFade,sourceLower.t-sourceUpper.t);
    if(!native_reload::OrientArmPlane(sk,nativeGuide,rig.right,hand.t,guide90,1.0f,math::kPi))return 2;
    float previousFadeTurn=-1.0f;
    for(const float weight:{0.0f,0.001f,0.01f,1.0f})
    {
        ov::Pose mixed;
        mixed.n=sk.n;
        for(int b=0;b<sk.n;++b)mixed.b[b]=ov::BlendXf(source->b[b],nativeGuide.b[b],weight);
        const auto mixedHand=native_reload::Model(sk,mixed,rig.right.hand);
        const auto mixedGuide=native_reload::Model(sk,mixed,rig.right.lower);
        ov::Pose candidate=*source;
        if(!native_reload::OrientArmPlane(sk,candidate,rig.right,mixedHand.t,mixedGuide.t,1.0f,cap))return 2;
        const auto candidateUpper=native_reload::Model(sk,candidate,rig.right.upper);
        const auto candidateLower=native_reload::Model(sk,candidate,rig.right.lower);
        const float fadeTurn=std::fabs(SignedTurn(oldAxis,sourceLower.t-sourceUpper.t,
                                                  candidateLower.t-candidateUpper.t));
        std::printf("fade weight %.3f: right guide turn %.5f rad\n",weight,fadeTurn);
        check(std::isfinite(fadeTurn)&&fadeTurn+1e-5f>=previousFadeTurn,
              "guide turn is continuous and monotone while fading in");
        if(weight<=0.001f)check(fadeTurn<0.002f,"zero and epsilon fade remain near authored elbow plane");
        previousFadeTurn=fadeTurn;
    }

    ov::Pose sentinel=*source;sentinel.b[rig.right.upper].t.x+=0.123f;
    const ov::Pose before=sentinel;
    native_reload_contacts::Result result;
    const bool badCapAccepted=native_reload_contacts::Apply(sk,rig,*source,*source,sentinel,result,1.0f,
                                                             native_reload::kUnboundedGuideTurnRad,
                                                             math::kPi+0.01f);
    check(!badCapAccepted&&std::memcmp(&sentinel,&before,sizeof(sentinel))==0,
          "invalid right cap rejects full contact solve without changing output");
    return failed?1:0;
}

// Offline test: upper-arm continuity of the reload contact solve.
// Measured with tools/analysis/replay_pose_csv_reload.cpp (REPLAY_DIAG / REPLAY_JOINTS) on the 21.10 01:56 and 19.10 recordings:
// the upper arm jumped 105-172deg between two frames (01:56 k1 172, k2 154, k4 135; R19 k2 134; N37 k3 125) while the baseline
// arm moved < 12deg and the native arm < 3deg. Causes: (1) ParallelTransport at 100-160deg between the MM and the native hand
// axis makes the transported elbow plane ill-conditioned (mutation 10 = plane always from the transported authored elbow),
// (2) the hand target / guide swinging through the shoulder in one frame (mutation 11 = no joint speed limit).
// (3) The blend of the capped and the native elbow plane must stay continuous across the +-pi wrap of the guide turn
// (mutation 12 = the blend weight ignores the wrap).
// Usage: ReloadArmSlewTest <woman.utppov>
// Every mutation build must report a FAIL line (ctest PASS_REGULAR_EXPRESSION "FAIL ").
#define UTPP_RELOAD_DIAG 1 // OrientArmPlane records raw turn / transport / applied turn (section 4)
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
    const auto& sk=file.Skel();
    std::vector<int32_t> coldMap(size_t(sk.n));for(int b=0;b<sk.n;++b)coldMap[size_t(b)]=b;
    reload_contacts_runtime::Prepared prepared;
    if(!reload_contacts_runtime::FillPrepared(prepared,123,uint32_t(sk.n),coldMap.data(),
        [&](uint32_t i){return int(sk.parent[i]);},[&](uint32_t i){return file.BoneName(int(i));}))return 2;
    const auto rig=prepared.arms;
    const int index=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Aiming,false,false);if(index<0)return 2;
    auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
    const float deg=math::kPi/180.0f;
    const float perFrame=reload_contacts_runtime::kJointSlewRadPerSec*(166667.0f/10000000.0f)/deg; // limit for one 1/60 s frame (deg)

    // 1. JointLimiter on its own: a 100deg jump of the left upper arm is cut to the per-frame limit, a small one is untouched.
    {
        std::vector<unsigned char> records;Records(*base,records);
        reload_contacts_runtime::JointLimiter limiter;
        limiter.Capture(prepared,records.data(),uint32_t(sk.n),123,1000000);
        auto jumped=std::make_unique<ov::Pose>(*base);
        jumped->b[rig.left.upper].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0,0,1},100.0f*deg),base->b[rig.left.upper].r));
        auto moved=std::make_unique<ov::Pose>(*jumped);
        limiter.Limit(prepared,*moved,123,1000000+166667,10000000);
        const float angle=ov::AngleDeg(moved->b[rig.left.upper].r,base->b[rig.left.upper].r);
        const float toTarget=ov::AngleDeg(moved->b[rig.left.upper].r,jumped->b[rig.left.upper].r);
        std::printf("  100deg jump: limited to %.1fdeg from the previous output (%.1fdeg short of the target)\n",angle,toTarget);
        Check(std::fabs(angle-perFrame)<0.6f&&perFrame<=45.0f&&toTarget>50.0f,"joint limiter: a 100deg upper-arm jump is cut to the per-frame limit (<= 45deg per 1/60 s) towards the target");
        auto small=std::make_unique<ov::Pose>(*base);
        // Ramp weight < 1 uses the 1000 deg/s limit (16.7 deg per 1/60 s); weight 1 keeps the 2400 deg/s limit.
        {
            auto ramp=std::make_unique<ov::Pose>(*jumped);
            limiter.Limit(prepared,*ramp,123,1000000+166667,10000000,reload_contacts_runtime::JointSlewFor(0.5f));
            const float a=ov::AngleDeg(ramp->b[rig.left.upper].r,base->b[rig.left.upper].r);
            Check(std::fabs(a-16.7f)<0.6f&&a<perFrame,"joint limiter: correction weight < 1 cuts a 100deg jump to 16.7deg per 1/60 s");
            auto full=std::make_unique<ov::Pose>(*jumped);
            limiter.Limit(prepared,*full,123,1000000+166667,10000000,reload_contacts_runtime::JointSlewFor(1.0f));
            Check(std::fabs(ov::AngleDeg(full->b[rig.left.upper].r,base->b[rig.left.upper].r)-perFrame)<0.6f,
                  "joint limiter: weight 1 keeps the 2400 deg/s limit");
        }
        small->b[rig.left.upper].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0,0,1},20.0f*deg),base->b[rig.left.upper].r));
        const auto before=small->b[rig.left.upper].r;
        limiter.Limit(prepared,*small,123,1000000+166667,10000000);
        Check(small->b[rig.left.upper].r.x==before.x&&small->b[rig.left.upper].r.w==before.w,"joint limiter: a 20deg change is bit-identical");
        auto stale=std::make_unique<ov::Pose>(*jumped);
        limiter.Limit(prepared,*stale,123,1000000+9000000,10000000); // 0.9s later: new episode
        Check(stale->b[rig.left.upper].r.x==jumped->b[rig.left.upper].r.x,"joint limiter: a gap above .25s starts a new episode (no limit)");
    }

    // 2. Whole Finish: the native left hand passes the shoulder - the target direction swings 120deg between two frames.
    {
        integ::PoseSnapshot snap;snap.rig=123;snap.sequence=5;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(ov::PoseOverlay::Pistol2H);
        snap.reloadAuthoredClip=true;snap.upperScale=0;snap.traceAction=2;
        const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
        std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
        for(int b=0;b<sk.n;++b)
        {
            snap.gameIndex[b]=b>=2 ? b : -1;
            const auto& x=base->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));
        }
        const auto shoulder=native_reload::Model(sk,*base,rig.left.upper).t;
        math::Vec3 dirA;if(!gik::Norm(native_reload::Model(sk,*base,rig.left.hand).t-shoulder,dirA))return 2;
        const math::Vec3 dirB=math::QuatRotate(gik::AxisAngle(math::Vec3{0,0,1},120.0f*deg),dirA);
        auto nativeFor=[&](math::Vec3 dir){
            auto native=std::make_unique<ov::Pose>(*base);
            gik::Params wide;wide.minReach=0.05f;gik::Result solved;
            if(!gik::SolveArm(sk,*native,rig.left,shoulder+dir*0.30f,nullptr,1.0f,wide,solved))native.reset();
            return native;
        };
        auto nativeA=nativeFor(dirA),nativeB=nativeFor(dirB);
        if(!nativeA||!nativeB)return 2;
        auto run=[&](bool withState,ov::Pose& outA,ov::Pose& outB,native_reload_contacts::Result& ra,native_reload_contacts::Result& rb)
        {
            reload_contacts_runtime::Continuity state;
            for(int frame=0;frame<2;++frame)
            {
                const ov::Pose& native=frame==0 ? *nativeA : *nativeB;
                std::vector<unsigned char> records;Records(native,records);reload_contacts_runtime::Cycle cycle;
                snap.sequence=5+frame;
                const bool captured=reload_contacts_runtime::Capture(prepared,snap,records.data(),sk.n,true,cycle);
                Records(*base,records);
                if(!captured)return false;
                auto& result=frame==0 ? ra : rb;
                reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f,
                    reload_contacts_runtime::kAuthoredContactMinReach,withState ? &state : nullptr,3000000+int64_t(frame)*166667,10000000);
                if(!reload_contacts_runtime::Read(prepared,records.data(),frame==0 ? outA : outB))return false;
            }
            return true;
        };
        ov::Pose a0,b0,a1,b1;native_reload_contacts::Result ra0,rb0,ra1,rb1;
        const bool ok0=run(false,a0,b0,ra0,rb0),ok1=run(true,a1,b1,ra1,rb1);
        const float unlimited=ov::AngleDeg(a0.b[rig.left.upper].r,b0.b[rig.left.upper].r);
        const float limited=ov::AngleDeg(a1.b[rig.left.upper].r,b1.b[rig.left.upper].r);
        std::printf("  target 120deg round the shoulder in one frame: upper-arm change %.1fdeg without state, %.1fdeg with the production state (solver ok %d/%d)\n",
                    unlimited,limited,ra1.ok,rb1.ok);
        Check(ok0&&ok1&&ra0.ok&&rb0.ok&&unlimited>60.0f,"fixture: without the state the upper arm follows the target swing in one frame (>60deg)");
        Check(ok1&&limited<=perFrame+2.0f&&limited<=45.0f,"with the production state the upper arm changes by at most the per-frame limit (<= 45deg)");
    }

    // 3. Elbow plane near an antiparallel transport (hand axis ~160deg from the target axis): the plane must follow the
    //    native guide, not the ill-conditioned transported authored elbow.
    {
        const auto arm=rig.right;
        const auto upperModel=native_reload::Model(sk,*base,arm.upper);
        math::Vec3 axis0;if(!gik::Norm(native_reload::Model(sk,*base,arm.hand).t-upperModel.t,axis0))return 2;
        const math::Vec3 targetAxis=math::QuatRotate(gik::AxisAngle(math::Vec3{0,0,1},-(180.0f-12.0f)*deg),axis0); // ~168deg from the hand axis
        const math::Vec3 target=upperModel.t+targetAxis*0.40f;
        math::Vec3 side;if(!gik::Norm(math::Cross(targetAxis,math::Vec3{0,0,1}),side))return 2;
        const math::Vec3 guide=upperModel.t+targetAxis*0.18f+side*0.16f-math::Vec3{0,0,0.10f};
        math::Vec3 reference;gik::Norm(math::Vec3{0,0,-1}-targetAxis*math::Dot(math::Vec3{0,0,-1},targetAxis),reference);
        float previous=0,worst=0,minTransport=999,maxTransport=0;bool first=true;
        for(int step=0;step<=14;++step)
        {
            auto pose=std::make_unique<ov::Pose>(*base);
            pose->b[arm.upper].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0,0,1},(3.0f*float(step))*deg),pose->b[arm.upper].r));
            const auto u=native_reload::Model(sk,*pose,arm.upper),h=native_reload::Model(sk,*pose,arm.hand);
            math::Vec3 oldAxis,newAxis;gik::Norm(h.t-u.t,oldAxis);gik::Norm(target-u.t,newAxis);
            const float transport=std::acos(std::clamp(math::Dot(oldAxis,newAxis),-1.0f,1.0f))/deg;
            minTransport=std::fmin(minTransport,transport);maxTransport=std::fmax(maxTransport,transport);
            if(!native_reload::OrientArmPlane(sk,*pose,arm,target,guide,1.0f,math::kPi*0.25f))continue;
            const auto up=native_reload::Model(sk,*pose,arm.upper),lo=native_reload::Model(sk,*pose,arm.lower);
            math::Vec3 elbow=lo.t-up.t;math::Vec3 axis;gik::Norm(target-up.t,axis);
            math::Vec3 e;if(!gik::Norm(elbow-axis*math::Dot(elbow,axis),e))continue;
            math::Vec3 r;gik::Norm(reference-axis*math::Dot(reference,axis),r);
            const float azimuth=std::atan2(math::Dot(axis,math::Cross(r,e)),math::Dot(r,e))/deg;
            if(!first){float d=std::fabs(azimuth-previous);if(d>180.0f)d=360.0f-d;worst=std::fmax(worst,d);}
            previous=azimuth;first=false;
        }
        std::printf("  transport angle %.0f-%.0fdeg: largest elbow-plane step %.1fdeg per 3deg of baseline hand-axis motion\n",minTransport,maxTransport,worst);
        Check(minTransport>130.0f,"fixture: the transport angle stays above 130deg (blend fully on)");
        Check(worst<6.0f,"near-antiparallel transport: the elbow plane follows the native guide (< 6deg per 3deg of baseline motion)");
    }
    // 4. Elbow plane across the +-pi wrap of the guide turn with a transport angle INSIDE the blend band (100-130deg, 0 < w < 1).
    //    The native elbow guide is swept once round the arm axis in 0.5deg steps (right arm, Pistol2H cap pi/4). The soft clamp
    //    returns to 0 at +-pi, the native turn does not: an unfaded lerp of the two flips the plane by 2*w*pi when the guide
    //    crosses the far side (measured 60deg at w = .17, 175deg at w = .49, 68deg at w = .81 per 0.5deg step). Mutation 12.
    {
        const auto arm=rig.right;
        const auto upperModel=native_reload::Model(sk,*base,arm.upper);
        math::Vec3 axis0;if(!gik::Norm(native_reload::Model(sk,*base,arm.hand).t-upperModel.t,axis0))return 2;
        for(const float transportTarget:{90.0f,105.0f,115.0f,125.0f,140.0f})
        {
            const math::Vec3 targetAxis=math::QuatRotate(gik::AxisAngle(math::Vec3{0,0,1},-transportTarget*deg),axis0);
            const math::Vec3 target=upperModel.t+targetAxis*0.40f;
            math::Vec3 s1,s2;if(!gik::Norm(math::Cross(targetAxis,math::Vec3{0,0,1}),s1))return 2;
            s2=math::Cross(targetAxis,s1);
            float previous=0,worst=0,worstAt=0,transport=0,worstIdentity=0,widest=0;int ok=0,wraps=0,inBand=0;float previousTurn=0;bool first=true;
            for(int step=0;step<=720;++step)
            {
                const float a=float(step)*0.5f*deg;
                const math::Vec3 guide=upperModel.t+targetAxis*0.18f+(s1*std::cos(a)+s2*std::sin(a))*0.16f;
                auto pose=std::make_unique<ov::Pose>(*base);
                native_reload::g_diag.Clear();
                if(!native_reload::OrientArmPlane(sk,*pose,arm,target,guide,1.0f,math::kPi*0.25f)||native_reload::g_diag.n<1){first=true;continue;}
                ++ok;
                const auto& d=native_reload::g_diag.e[0];
                transport=d.transportDeg;
                const auto up=native_reload::Model(sk,*pose,arm.upper),lo=native_reload::Model(sk,*pose,arm.lower);
                math::Vec3 ax;gik::Norm(target-up.t,ax);
                math::Vec3 e;if(!gik::Norm((lo.t-up.t)-ax*math::Dot(lo.t-up.t,ax),e)){first=true;continue;}
                const float azimuth=std::atan2(math::Dot(ax,math::Cross(s1,e)),math::Dot(s1,e))/deg;
                if(!first)
                {
                    float step2=std::fabs(azimuth-previous);if(step2>180.0f)step2=360.0f-step2;
                    if(step2>worst){worst=step2;worstAt=float(step)*0.5f;}
                    if(std::fabs(d.turnDeg)>100.0f&&std::fabs(previousTurn)>100.0f&&d.turnDeg*previousTurn<0.0f)++wraps;
                }
                widest=std::fmax(widest,std::fabs(d.turnDeg));
                // Up to the start of the soft zone the plane is exactly the former lerp(Soft(turn), turn, w).
                const float turn=d.turnDeg*deg;
                if(std::fabs(d.turnDeg)<=135.0f)
                {
                    const float soft=native_reload::SoftClampGuideTurn(turn,math::kPi*0.25f),w=native_reload::TransportPlaneBlend(d.transportDeg);
                    worstIdentity=std::fmax(worstIdentity,std::fabs(d.appliedDeg-(soft+w*(turn-soft))/deg));
                    ++inBand;
                }
                previous=azimuth;previousTurn=d.turnDeg;first=false;
            }
            const float w=native_reload::TransportPlaneBlend(transport);
            std::printf("  transport %.1fdeg (w=%.2f): ok %d/721, guide turn crosses +-180deg %d x, largest elbow-plane step %.1fdeg per 0.5deg guide step (at %.1fdeg), |turn| <= 135deg: max deviation from the former blend %.4fdeg\n",
                        transport,w,ok,wraps,worst,worstAt,worstIdentity);
            Check(ok==721&&widest>179.0f,"fixture: the guide sweep covers the whole circle, including the far side of the transported elbow");
            Check(worst<6.0f,"elbow plane: no jump across the +-180deg wrap of the guide turn (< 6deg per 0.5deg guide step)");
            Check(inBand>300&&worstIdentity<0.02f,"elbow plane: unchanged (the former blend) while |turn| <= 135deg");
            if(transportTarget>100.0f&&transportTarget<130.0f)Check(wraps>=1,"fixture: the sweep crosses the wrap with 0 < w < 1");
        }
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

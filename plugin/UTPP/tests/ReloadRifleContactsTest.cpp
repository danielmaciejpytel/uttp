// Offline test: native reload contact correction for RIFLE reloads (overlay Rifle, no authored clip).
// Go/no-go on recording 2026-10-25 (the recorded log): the rifle native targets must be carried in
// the Spine3 frame (anchored in model space they lag a turning MM torso by 19.5cm p50 / 56.8cm max); the pistol path stays as it was.
// Usage: ReloadRifleContactsTest <woman.utppov>
// Mutations (UTPP_RELOAD_RIFLE_MUTATION): 1 = rifle path off, 2 = rifle keeps the pistol placement (no forced chest-frame transport),
// 3 = rifle keeps the stationary-only pose rule of 88910e9 (no crouch / moving rifle reloads), 4 = rifle slide frames excluded (e00bb09),
// 5 = no stock clearance (the stock may sink into the MM body), 6 = stock clearance shift applied raw (no rate limit),
// 7 = no left arm torso clearance, 8 = no left hand smoothing (weapon-frame follower), 9 = the stock shift ignores the left arm,
// 10 = left arm clearance offset applied raw (no rate limit), 11 = left hand follower starts at the blended hand (no glide on the ramp-in),
// 17 = the episode starts from the MM snapshot instead of the published pose, 18 = residual clearance rise at the old 1.5 m/s,
// 19 = contact catch-up at the old 9 m/s, 20 = the episode starts from its first frame, not from the observed published pose, 21 = rifle ramp with the 1000 deg/s joint slew, 12 = no weapon follower (the whole target jumps with the weight on the ramp-in), 14 = the old exit fade (lag at full scale until the weight is
// below .15), 15 = no follower reset at weight 0, 16 = no contact catch-up of the left hand.
// Every mutation build must report an assertion FAIL (ctest PASS_REGULAR_EXPRESSION "FAIL ").
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/OverlayFile.hpp"
#include "../src/mm/ReloadContactFade.hpp"
#include "TestFailFast.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
using namespace utpp::mm;
namespace rt=reload_contacts_runtime;
static int g_pass=0,g_fail=0;
static void Check(bool ok,const char* what){(ok?g_pass:g_fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);if(!ok)UtppFailFastNote(std::string("FAIL ")+what);}
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
    rt::Prepared prepared;
    if(!rt::FillPrepared(prepared,123,uint32_t(sk.n),coldMap.data(),
        [&](uint32_t i){return int(sk.parent[i]);},[&](uint32_t i){return file.BoneName(int(i));}))return 2;
    const auto rig=prepared.arms;const int hips=prepared.hips;
    if(hips<=1)return 2;
    const int spine3=sk.parent[sk.parent[rig.left.upper]];

    // 1) Eligible overlays (FkHook and the driver's reload domain read the same predicate).
    Check(rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Rifle)),"Rifle reloads are eligible for the contact correction");
    Check(rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Pistol1H))&&rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Pistol2H)),
          "Pistol1H / Pistol2H stay eligible");
    Check(!rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Default))&&!rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Injured))&&
          !rt::ReloadContactsOverlay(int32_t(ov::PoseOverlay::Game))&&!rt::ReloadContactsOverlay(9),
          "Default / Injured / Game / raw 9 are not eligible");

    const int index=file.FindOverlayPose(ov::PoseOverlay::Rifle,ov::AimState::Aiming,false,false);if(index<0)return 2;
    auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
    integ::PoseSnapshot snap;snap.rig=123;snap.sequence=11;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;
    snap.reloadAuthoredClip=false;snap.upperScale=0;snap.traceAction=2;snap.reloadContactsEligible=true;
    const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
    std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
    for(int b=0;b<sk.n;++b)
    {
        snap.gameIndex[b]=b>=2 ? b : -1;
        const auto& x=base->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));
    }
    const auto finish=[&](int overlay,const ov::Pose& native,float weight,ov::Pose& out,native_reload_contacts::Result& result,
                          rt::Continuity* state,int64_t ticks,uint64_t sequence,std::vector<unsigned char>* written=nullptr,const ov::Pose* published=nullptr)
    {
        integ::PoseSnapshot s=snap;s.traceOverlay=overlay;s.sequence=sequence;s.reloadContactsWeight=weight;
        std::vector<unsigned char> records;Records(native,records);rt::Cycle cycle;
        const bool captured=rt::Capture(prepared,s,records.data(),sk.n,true,cycle);
        Records(published ? *published : *base,records);
        if(!captured)return false;
        const bool ok=rt::Finish(prepared,s,records.data(),sk.n,true,cycle,result,weight,rt::kAuthoredContactMinReach,state,ticks,10000000);
        if(written)*written=records;
        return ok&&rt::Read(prepared,records.data(),out);
    };
    const auto inSpine3=[&](const ov::Pose& pose,int bone){
        const auto frame=native_reload::Model(sk,pose,spine3);
        return math::QuatRotate(math::QuatInv(frame.r),native_reload::Model(sk,pose,bone).t-frame.t);
    };

    // 2) Chest-frame transport. Native body turned 25deg about the vertical at the hips (pelvis in place: the divergence handling of the
    //    pistols never engages) with its left hand 8cm forward / 6cm up relative to the chest. Rifle: the output left hand keeps the native
    //    chest-relative position. Pistol (shoulder anchor, model-space directions): it does not - the turn of the native torso shows.
    {
        auto native=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*native,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        {
            auto desired=native_reload::Model(sk,*native,hips);
            desired.r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0.0f,0.0f,1.0f},25.0f*math::kPi/180.0f),desired.r));
            const auto parentModel=sk.parent[hips]>=0 ? native_reload::Model(sk,*native,sk.parent[hips]) : ov::Xf{};
            native->b[hips].r=native_reload::Relative(parentModel,desired).r;
        }
        const float pelvis=gik::Len(native_reload::Model(sk,*native,hips).t-native_reload::Model(sk,*base,hips).t);
        ov::Pose rifle,pistol;native_reload_contacts::Result rr,rp;
        const bool okR=finish(int(ov::PoseOverlay::Rifle),*native,1.0f,rifle,rr,nullptr,0,21);
        const bool okP=finish(int(ov::PoseOverlay::Pistol2H),*native,1.0f,pistol,rp,nullptr,0,22);
        const float rifleErr=okR?gik::Len(inSpine3(rifle,rig.left.hand)-inSpine3(*native,rig.left.hand)):9.0f;
        const float rifleMoved=okR?gik::Len(inSpine3(rifle,rig.left.hand)-inSpine3(*base,rig.left.hand)):0.0f;
        const float pistolErr=okP?gik::Len(inSpine3(pistol,rig.left.hand)-inSpine3(*native,rig.left.hand)):0.0f;
        std::printf("  turned native (pelvis %.3fm): rifle ok=%d LH-in-Spine3 err %.4fm (moved %.3fm) | pistol ok=%d err %.3fm\n",
                    pelvis,okR,rifleErr,rifleMoved,okP,pistolErr);
        Check(pelvis<rt::kTorsoFadeStartM,"fixture: pelvis distance below the pistol divergence handling");
        Check(okR&&rr.ok&&rifleErr<0.01f&&rifleMoved>0.05f,"rifle: the native left hand is carried in the Spine3 frame (chest-relative error < 1cm)");
        Check(okP&&rp.ok&&pistolErr>0.03f,"pistol on the same native: shoulder-anchor placement (not the chest frame), unchanged");
    }

    // 3) Pose rule of the correction fade target (ReloadContactFade.hpp ReloadCorrectionPose, called by MotionDriver FrameInputsLocked).
    //    Pistols / every other overlay: exactly the stationary rule of before for every input. Rifle: every ground stance and gait and
    //    every slide frame (MovementMode Sliding 2 with the slide bundle, or the PSM slide detail on the ground, mode 0), never in the air
    //    (MovementMode 1, also a slide frame in the air) and never mode 2 without a slide frame (fail closed). Mutation 4 restores the
    //    e00bb09 rule (slide frames excluded).
    {
        bool pistolSame=true,rifleGround=true,rifleSlide=true,rifleAir=true;
        for(int stationary=0;stationary<2;++stationary)
            for(int mode=0;mode<3;++mode)
                for(int slide=0;slide<2;++slide)
                {
                    pistolSame=pistolSame && rt::ReloadCorrectionPose(false,stationary!=0,mode,slide!=0)==(stationary!=0);
                    const bool rifle=rt::ReloadCorrectionPose(true,stationary!=0,mode,slide!=0);
                    if(mode==0 && slide==0)rifleGround=rifleGround && rifle;
                    else if(slide==1 && mode!=1)rifleSlide=rifleSlide && rifle;
                    else rifleAir=rifleAir && !rifle;
                }
        Check(pistolSame,"pistol pose rule == the stationary rule for every input (bit-identical)");
        Check(rifleGround,"rifle pose rule: every ground frame (stand / crouch, idle / moving) is eligible");
        Check(rifleSlide,"rifle pose rule: slide frames (MovementMode Sliding, or the PSM slide on the ground) are eligible");
        Check(rifleAir,"rifle pose rule: air (mode 1, with or without a slide frame) and mode 2 without a slide frame are not eligible");
    }

    // 4) Production CorrectionFade driven by the pose rule over one rifle reload at 60 fps (native envelope .12 s in, like
    //    g_reloadUpperWeight): stand -> walk -> crouch -> crouch walk -> stand inside the reload never lowers the weight; a jump in the
    //    middle withdraws it as a fade (at most dt / .2 s per frame, no reset) and the landing brings it back at most dt / .12 s per frame.
    //    The same stance sequence on a pistol drops it (stationary rule, unchanged).
    {
        struct F{bool crouch,moving;int mode;bool slide;};
        const auto run=[&](bool rifle,auto&& at,float& minAfterFull,float& maxDrop,float& maxRise,float& endW){
            rt::CorrectionFade fade;rt::UpperEnvelope env;const float dt=1.0f/60.0f;
            minAfterFull=1.0f;maxDrop=0.0f;maxRise=0.0f;endW=0.0f;bool full=false;float last=0.0f;
            for(int k=0;k<110;++k)
            {
                const F f=at(k);
                const float e=env.Step(dt,true,false,!rifle);
                const bool stationary=!f.crouch && !f.moving && f.mode==0;
                const float w=fade.Step(dt,e,rt::ReloadCorrectionPose(rifle,stationary,f.mode,f.slide),true,true);
                if(full)minAfterFull=std::min(minAfterFull,w);
                full=full || w>=0.999f;
                maxDrop=std::max(maxDrop,last-w);maxRise=std::max(maxRise,w-last);last=endW=w;
            }
        };
        const auto stances=[](int k){
            return k<20 ? F{false,false,0,false} : k<40 ? F{false,true,0,false} : k<60 ? F{true,false,0,false} : k<80 ? F{true,true,0,false}
                        : F{false,false,0,false};};
        float minR,dropR,riseR,endR,minP,dropP,riseP,endP;
        run(true,stances,minR,dropR,riseR,endR);
        run(false,stances,minP,dropP,riseP,endP);
        std::printf("  stance changes: rifle min after full %.3f max drop %.4f end %.3f | pistol min %.3f end %.3f\n",minR,dropR,endR,minP,endP);
        Check(minR>=0.999f && dropR<=1.0e-6f && endR>=0.999f,"rifle: stand / walk / crouch / crouch walk changes inside the reload keep the full weight");
        Check(minP<=1.0e-6f,"pistol: the same stance changes withdraw the correction (stationary rule unchanged)");
        const auto jump=[](int k){ return k>=40 && k<70 ? F{false,true,1,false} : F{false,true,0,false};};
        float minJ,dropJ,riseJ,endJ;
        run(true,jump,minJ,dropJ,riseJ,endJ);
        std::printf("  jump inside the reload: min %.3f max drop/frame %.4f max rise/frame %.4f end %.3f\n",minJ,dropJ,riseJ,endJ);
        Check(minJ<=1.0e-6f && dropJ<=(1.0f/60.0f)/0.2f+1.0e-5f && riseJ<=(1.0f/60.0f)/0.12f+1.0e-5f && endJ>=0.999f,
              "rifle: the air withdraws the correction as a .2 s fade (no reset) and the landing ramps it back in over .12 s");
        const auto slide=[](int k){ return k>=40 && k<70 ? F{true,true,2,true} : F{false,true,0,false};};
        float minS,dropS,riseS,endS;
        run(true,slide,minS,dropS,riseS,endS);
        std::printf("  slide inside the reload: min %.3f max drop/frame %.4f end %.3f\n",minS,dropS,endS);
        Check(minS>=0.999f && dropS<=1.0e-6f && endS>=0.999f,"rifle: a slide inside the reload keeps the full correction (de-rolled chest)");
        const auto slideStart=[](int k){ return k<50 ? F{true,true,2,true} : F{true,false,0,false};}; // reload edge inside the slide
        float minE,dropE,riseE,endE;
        run(true,slideStart,minE,dropE,riseE,endE);
        Check(minE>=0.999f && dropE<=1.0e-6f && riseE<=(1.0f/60.0f)/0.12f+1.0e-5f && endE>=0.999f,
              "rifle: a reload that starts inside a slide ramps the correction in over .12 s and keeps it through the slide exit");
        const auto slideAir=[](int k){ return k>=40 && k<70 ? F{true,true,1,true} : F{true,true,2,true};}; // off a ledge mid-slide
        float minA,dropA,riseA,endA;
        run(true,slideAir,minA,dropA,riseA,endA);
        Check(minA<=1.0e-6f && dropA<=(1.0f/60.0f)/0.2f+1.0e-5f && endA>=0.999f,"rifle: a slide that leaves the ground withdraws the correction as a fade");
        float minPS,dropPS,risePS,endPS;
        run(false,slide,minPS,dropPS,risePS,endPS);
        Check(endPS<=1.0e-6f && risePS<=1.0e-6f,"pistol: a slide inside a moving reload keeps the stationary rule (no correction)");
    }

    // 5) Stock clearance (kRifleStockClearance). Native hands (and the weapon on the right hand) carried backward and towards the midline:
    //    the stock sinks into the torso of the aim pose. Rifle: the output stock is clear of the body model, the whole target moved
    //    rigidly (both hands keep their place in the weapon frame, the contact solve is accepted), the shift is bounded. Pistol on the
    //    same native: no clearance. Over frames (Continuity) the applied shift moves at most kStockShiftRateMps. Mutation 5 (clearance
    //    off) fails the depth checks, mutation 6 (raw shift) fails the rate check.
    {
        const int neck=rt::NeckBaseOf(prepared);
        math::Vec3 right,fwd,up;
        if(neck<0 || !rt::ChestFrame(sk,*base,spine3,neck,right,fwd,up))return 2;
        const auto carried=[&](float back,ov::Pose& q)->bool{ // back: along the chest backward axis and half of it towards the midline
            q=*base;
            const auto lh=native_reload::Model(sk,q,rig.left.hand),rh=native_reload::Model(sk,q,rig.right.hand);
            gik::Params wide;gik::Result r1,r2;
            const math::Vec3 move=(fwd+right)*(-back);
            return gik::SolveArm(sk,q,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) &&
                   gik::SolveArm(sk,q,rig.left,lh.t+move,&lh.r,1.0f,wide,r2);
        };
        const auto depth=[&](const ov::Pose& q){float d[rt::kStockParts+1];rt::StockDepths(prepared,q,d);return d[rt::kStockParts];};
        const auto inWeapon=[&](const ov::Pose& q,int bone){
            const auto w=native_reload::Model(sk,q,rig.weaponRight);
            return math::QuatRotate(math::QuatInv(w.r),native_reload::Model(sk,q,bone).t-w.t);
        };
        auto native=std::make_unique<ov::Pose>();
        if(!carried(0.08f,*native))return 2;
        ov::Pose rifle,pistol;native_reload_contacts::Result rr,rp;
        const bool okR=finish(int(ov::PoseOverlay::Rifle),*native,1.0f,rifle,rr,nullptr,0,31);
        const bool okP=finish(int(ov::PoseOverlay::Pistol2H),*native,1.0f,pistol,rp,nullptr,0,32);
        const float dBase=depth(*base),dNative=depth(*native),dOut=okR?depth(rifle):9.0f;
        {float d[rt::kStockParts+1];rt::StockDepths(prepared,*base,d);std::printf("  stock: aim pose parts trapezius %.3f upper arm %.3f neck %.3f torso %.3f\n",d[0],d[1],d[2],d[3]);}
        const float lh=okR?gik::Len(inWeapon(rifle,rig.left.hand)-inWeapon(*native,rig.left.hand)):9.0f;
        const float rh=okR?gik::Len(inWeapon(rifle,rig.right.hand)-inWeapon(*native,rig.right.hand)):9.0f;
        std::printf("  stock: depth base %.3fm native %.3fm rifle out %.3fm (shift %.3fm) | hands in weapon frame LH %.5f RH %.5fm | pistol ok=%d shift %.3f\n",
                    dBase,dNative,dOut,rr.stockShift,lh,rh,okP,rp.stockShift);
        Check(dBase<0.005f && dNative>0.02f,"fixture: the aim pose stock is clear, the carried native stock sinks into the body");
        Check(okR && rr.ok && dOut<0.002f,"rifle: the stock of the output is clear of the body model");
        Check(okR && rr.stockShift>0.0f && rr.stockShift<=rt::kStockShiftMaxM+1.0e-5f,"rifle: the clearance shift is bounded (kStockShiftMaxM)");
        Check(lh<0.001f && rh<0.001f,"rifle: both hands keep their place in the weapon frame (rigid shift, the magazine stays in the left hand)");
        Check(okP && rp.stockShift==0.0f,"pistol on the same native: no stock clearance");
        // Frames: the stock sinks over 20 frames, then the native jumps clear at once.
        rt::Continuity state;float prev=-1.0f,maxStep=0.0f,afterJump=0.0f;bool allOk=true;
        for(int k=0;k<30;++k)
        {
            const float back=k<20 ? 0.004f*float(k) : 0.0f;
            auto q=std::make_unique<ov::Pose>();
            if(!carried(back,*q))return 2;
            ov::Pose out;native_reload_contacts::Result r;
            allOk=finish(int(ov::PoseOverlay::Rifle),*q,1.0f,out,r,&state,int64_t(1000000)+int64_t(k)*166667,uint64_t(100+k)) && r.ok && allOk;
            if(prev>=0.0f)maxStep=std::max(maxStep,std::fabs(r.stockShift-prev));
            if(k==20)afterJump=r.stockShift;
            prev=r.stockShift;
        }
        const float rateStep=0.30f*0.0166667f+1.0e-4f; // the production kStockShiftRateMps (mutation 6 sets it to 1e9)
        std::printf("  stock frames: max shift change per frame %.4fm (limit %.4fm), shift on the first clear frame %.4fm, all accepted %d\n",
                    maxStep,rateStep,afterJump,int(allOk));
        Check(allOk,"rifle frames with a sinking stock: every contact solve accepted");
        Check(maxStep<=rateStep && afterJump>0.0f,"rifle frames: the applied shift moves at most kStockShiftRateMps (no pop when the stock clears)");
    }


    // 6) Left arm of the carried rifle (rec 2026-10-28 05:03): the native left hand / forearm sits inside the chest / belly of the MM body and the
    //    native FPP choreography snaps it between pouch and weapon. (a) torso clearance (AdjustLeftArm, one translation of the left clavicle:
    //    weapon, right hand and stock untouched), (b) the weapon-frame follower (speed / acceleration cap, exact on a static hand), (c) the stock
    //    shift does not drive the left arm deeper into the torso. Pistols: no left arm code (leftArmShift == 0, hand where the native put it).
    //    Mutation 7 fails the clearance checks, 8 the follower check, 9 the stock-shift check, 10 the rate checks.
    {
        const int neck=rt::NeckBaseOf(prepared);
        rt::StockBody frame;
        if(neck<0 || !rt::StockBodyOf(prepared,*base,*base,frame))return 2;
        const auto inWeaponFrame=[&](const ov::Pose& q,int bone){
            const auto w=native_reload::Model(sk,q,rig.weaponRight);
            return math::QuatRotate(math::QuatInv(w.r),native_reload::Model(sk,q,bone).t-w.t);
        };
        const auto leftDepth=[&](const ov::Pose& q){math::Vec3 pts[rt::kLeftArmPoints];rt::LeftArmPoints(prepared,q,pts);
            return rt::LeftArmTorsoDepth(frame,pts,math::Vec3{});};
        // chest-frame target (x right, y forward, z up from the neck base) for the left hand
        const auto handAt=[&](float x,float y,float z,ov::Pose& q)->bool{
            q=*base;
            const math::Vec3 target=frame.neck+frame.right*x+frame.fwd*y+frame.up*z;
            gik::Params wide;gik::Result r;
            return gik::SolveArm(sk,q,rig.left,target,nullptr,1.0f,wide,r);
        };
        auto belly=std::make_unique<ov::Pose>();
        if(!handAt(-0.01f,0.09f,-0.30f,*belly))return 2;
        const float dBelly=leftDepth(*belly),dBase=leftDepth(*base);
        ov::Pose rifle,pistol;native_reload_contacts::Result rr,rp;
        const bool okR=finish(int(ov::PoseOverlay::Rifle),*belly,1.0f,rifle,rr,nullptr,0,41);
        const bool okP=finish(int(ov::PoseOverlay::Pistol2H),*belly,1.0f,pistol,rp,nullptr,0,42);
        const float dOut=okR?leftDepth(rifle):9.0f;
        const float weaponMoved=okR?gik::Len(native_reload::Model(sk,rifle,rig.weaponRight).t-native_reload::Model(sk,*belly,rig.weaponRight).t):9.0f;
        const float rhMoved=okR?gik::Len(inWeaponFrame(rifle,rig.right.hand)-inWeaponFrame(*belly,rig.right.hand)):9.0f;
        std::printf("  left arm: torso depth base %.3f native %.3f out %.3f | leftArmShift %.3f weapon moved %.4f right hand in weapon %.5f | pistol ok=%d shift %.3f\n",
                    dBase,dBelly,dOut,rr.leftArmShift,weaponMoved,rhMoved,okP,rp.leftArmShift);
        Check(dBase<=0.0f && dBelly>0.02f,"fixture: the aim pose left arm is clear of the torso, the native left hand rests inside the belly");
        Check(okR && rr.ok && dOut<=0.0f,"rifle: the left hand / forearm of the output are clear of the torso model");
        Check(okR && rr.leftArmShift>0.0f && rr.leftArmShift<=rt::kLeftArmClearMaxM+1.0e-4f,"rifle: the left arm offset is bounded (kLeftArmClearMaxM)");
        Check(weaponMoved<1.0e-4f && rhMoved<1.0e-4f,"rifle: the weapon and the right hand stay where the native put them (only the left arm moves)");
        Check(okP && rp.ok && rp.leftArmShift==0.0f && gik::Len(native_reload::Model(sk,pistol,rig.left.hand).t-native_reload::Model(sk,*belly,rig.left.hand).t)<2.0e-3f,
              "pistol on the same native: no left arm adjustment (hand where the native put it)");
        // Follower: static for 8 frames, then the native left hand snaps sideways in the chest frame (a pouch grab) and stays there.
        auto nearPose=std::make_unique<ov::Pose>(),farPose=std::make_unique<ov::Pose>();
        if(!handAt(-0.05f,0.38f,-0.10f,*nearPose) || !handAt(-0.30f,0.20f,-0.15f,*farPose))return 2;
        rt::Continuity state;bool allOk=true;float staticErr=0.0f,firstStep=0.0f,maxStep=0.0f,maxAccel=0.0f,endErr=9.0f,prevSpeed=0.0f;
        math::Vec3 prev{};
        const float dt=0.0166667f;
        const float snapDist=gik::Len(inWeaponFrame(*farPose,rig.left.hand)-inWeaponFrame(*nearPose,rig.left.hand));
        for(int k=0;k<90;++k)
        {
            const ov::Pose& q=k<8 ? *nearPose : *farPose;
            ov::Pose out;native_reload_contacts::Result r;
            allOk=finish(int(ov::PoseOverlay::Rifle),q,1.0f,out,r,&state,int64_t(5000000)+int64_t(k)*166667,uint64_t(300+k)) && r.ok && allOk;
            const math::Vec3 h=inWeaponFrame(out,rig.left.hand);
            const float err=gik::Len(h-inWeaponFrame(q,rig.left.hand));
            if(k<8)staticErr=std::max(staticErr,err);
            if(k>0)
            {
                const float step=gik::Len(h-prev),speed=step/dt;
                if(k==8)firstStep=step;
                if(k>=8)maxStep=std::max(maxStep,step);
                if(k>=9)maxAccel=std::max(maxAccel,std::fabs(speed-prevSpeed)/dt);
                prevSpeed=speed;
            }
            prev=h;
            if(k==89)endErr=err;
        }
        std::printf("  follower: snap %.3fm static err %.5f first step after the snap %.3fm max step %.3f max accel %.0f m/s2 end err %.4f all ok %d\n",
                    snapDist,staticErr,firstStep,maxStep,maxAccel,endErr,int(allOk));
        Check(snapDist>0.15f,"fixture: the native left hand snaps by more than 15cm in the weapon frame");
        Check(allOk,"rifle frames with a snapping left hand: every contact solve accepted");
        Check(staticErr<1.0e-3f,"follower: a static left hand (resting on the weapon) is reproduced exactly");
        Check(firstStep<0.5f*snapDist && maxStep<=0.12f,"follower: a snap of the native left hand is spread over frames (first step < half of it, never above the speed cap)");
        Check(maxAccel<=rt::kLeftHandMaxAccelMps2*3.0f,"follower: the left hand speed changes by at most ~3x the acceleration cap per frame");
        Check(endErr<0.005f,"follower: the left hand reaches the native position again");
        // The clearance offset is rate limited: rises with kLeftArmRiseMps, falls with kLeftArmFallMps (chest frame, per frame).
        rt::LeftArmState st;math::Vec3 o=st.StepClear(math::Vec3{0.0f,0.08f,0.0f},7,1000000,10000000);
        bool rateOk=gik::Len(o)>0.079f; // an episode starts at the wanted value
        float maxRise=0.0f,maxFall=0.0f;
        for(int k=1;k<80;++k)
        {
            const math::Vec3 w=k<10 ? math::Vec3{0.0f,0.08f,0.0f} : (k<20 ? math::Vec3{0.0f,0.15f,0.0f} : math::Vec3{});
            const math::Vec3 n=st.StepClear(w,7,1000000+int64_t(k)*166667,10000000);
            const float d=gik::Len(n)-gik::Len(o);
            maxRise=std::max(maxRise,d);maxFall=std::max(maxFall,-d);o=n;
        }
        std::printf("  clearance state: max rise/frame %.4f (limit %.4f) max fall/frame %.4f (limit %.4f)\n",maxRise,1.0f*dt,maxFall,0.25f*dt); // production kLeftArmRiseMps / kLeftArmFallMps (mutation 10 sets them to 1e9)
        Check(rateOk && maxRise<=1.0f*dt+1.0e-4f && maxFall<=0.25f*dt+1.0e-4f && gik::Len(o)<1.0e-4f,
              "left arm clearance offset: rise <= kLeftArmRiseMps, fall <= kLeftArmFallMps (no pop when the native hand leaves the belly)");
        // Entry / exit ramp of the correction: the episode starts at the BASELINE hand (weapon frame of the MM pose), not at the blended one, so the
        // ramp-in glides (mutation 11 starts at the blended hand: the first frame jumps by weight x distance); the last frames of a falling weight
        // do not pop by the lag.
        {
            const float hd=gik::Len(inWeaponFrame(*farPose,rig.left.hand)-inWeaponFrame(*base,rig.left.hand));
            rt::Continuity entry;
            ov::Pose out;native_reload_contacts::Result r;
            const bool ok0=finish(int(ov::PoseOverlay::Rifle),*farPose,0.20f,out,r,&entry,int64_t(9000000),uint64_t(500));
            const float fromBase=ok0?gik::Len(inWeaponFrame(out,rig.left.hand)-inWeaponFrame(*base,rig.left.hand)):9.0f;
            float maxStepRamp=0.0f;math::Vec3 pv=inWeaponFrame(out,rig.left.hand);bool rampOk=ok0;
            for(int k=1;k<12;++k) // weight 0.2 -> 1 in .12 s like the production ramp
            {
                const float wk=std::min(1.0f,0.2f+0.135f*float(k));
                rampOk=finish(int(ov::PoseOverlay::Rifle),*farPose,wk,out,r,&entry,int64_t(9000000)+int64_t(k)*166667,uint64_t(500+k)) && r.ok && rampOk;
                const math::Vec3 h=inWeaponFrame(out,rig.left.hand);
                maxStepRamp=std::max(maxStepRamp,gik::Len(h-pv));pv=h;
            }
            std::printf("  ramp-in: baseline-to-native hand distance %.3fm, first frame %.3fm from the baseline hand, max step %.3fm/frame, all ok %d\n",hd,fromBase,maxStepRamp,int(rampOk));
            Check(hd>0.15f,"fixture: the native hand is more than 15cm from the baseline hand in the weapon frame");
            Check(rampOk && fromBase<0.02f,"ramp-in: the first frame of an episode keeps the left hand at the baseline hand (no jump by weight x distance)");
            Check(maxStepRamp<=rt::kLeftHandMaxSpeedMps*dt*1.3f+0.003f,"ramp-in: the left hand glides at most at the speed cap per frame");
        }
        // Weapon follower: the whole carried target (weapon + hands) glides from the baseline weapon to the native one on the ramp-in (rigid shift,
        // mutation 12 turns it off: the first frame then sits weight x distance away from the baseline).
        {
            auto wpn=std::make_unique<ov::Pose>(*base);
            {
                gik::Params wide;gik::Result r1,r2;
                const auto rh=native_reload::Model(sk,*wpn,rig.right.hand),lh=native_reload::Model(sk,*wpn,rig.left.hand);
                const math::Vec3 move=frame.right*(-0.12f)+frame.up*0.10f+frame.fwd*0.08f;
                if(!gik::SolveArm(sk,*wpn,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) || !gik::SolveArm(sk,*wpn,rig.left,lh.t+move,&lh.r,1.0f,wide,r2))return 2;
            }
            const math::Vec3 weaponBase=inSpine3(*base,rig.weaponRight);
            const float wd=gik::Len(inSpine3(*wpn,rig.weaponRight)-weaponBase);
            rt::Continuity wc;ov::Pose out;native_reload_contacts::Result r;bool okAll=true;float first=9.0f,maxStep=0.0f,endErr=9.0f;math::Vec3 pw{};
            for(int k=0;k<60;++k)
            {
                const float wk=std::min(1.0f,0.2f+0.135f*float(k));
                okAll=finish(int(ov::PoseOverlay::Rifle),*wpn,wk,out,r,&wc,int64_t(30000000)+int64_t(k)*166667,uint64_t(700+k)) && r.ok && okAll;
                const math::Vec3 w=inSpine3(out,rig.weaponRight);
                if(k==0)first=gik::Len(w-weaponBase);else maxStep=std::max(maxStep,gik::Len(w-pw));
                pw=w;
                if(k==59)endErr=gik::Len(w-inSpine3(*wpn,rig.weaponRight));
            }
            std::printf("  weapon ramp-in: native weapon %.3fm from the baseline, first frame %.3fm from the baseline, max step %.4fm/frame (cap %.4f), end err %.4fm, all ok %d\n",
                        wd,first,maxStep,rt::kWeaponMaxSpeedMps*dt,endErr,int(okAll));
            Check(wd>0.12f,"fixture: the native weapon is more than 12cm from the baseline weapon");
            Check(okAll && first<0.02f,"ramp-in: the first frame keeps the weapon at the baseline weapon (the whole target glides, no jump by weight x distance)");
            Check(maxStep<=2.5f*dt*1.3f+0.004f,"ramp-in: the weapon glides at most at the weapon speed cap per frame");
            Check(endErr<0.005f,"ramp-in: the weapon reaches the native placement");
        }
        // Exit ramp (CorrectionFade .2 s linear to 0) and a new episode, at 60 and 30 fps. The natives snap (weapon + hands 0.2m one way, then 0.35m the
        // other way) at the start of the exit, so both followers carry a lag into it. The weight reaches 0 (the records then show the baseline): the
        // lag contribution must fall with the weight - the largest step of the left hand / the weapon (Spine3 frame, including the frame at weight 0)
        // stays within 1.5x the plain blend (no follower) + 1cm at either frame rate (the last step) and within the follower speed caps otherwise (mutation 14: the old .15 threshold pops on the last frames).
        // A new episode 0.1 s after the weight reached 0 starts at the baseline again (mutation 15: it resumes the old follower state).
        {
            const auto displaced=[&](float k,ov::Pose& q)->bool{
                q=*base;gik::Params wide;gik::Result r1,r2;
                const auto rh=native_reload::Model(sk,q,rig.right.hand),lh=native_reload::Model(sk,q,rig.left.hand);
                const math::Vec3 move=(frame.right*(-0.12f)+frame.up*0.10f+frame.fwd*0.08f)*k;
                return gik::SolveArm(sk,q,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) && gik::SolveArm(sk,q,rig.left,lh.t+move,&lh.r,1.0f,wide,r2);
            };
            auto d1=std::make_unique<ov::Pose>(),d2=std::make_unique<ov::Pose>();
            if(!displaced(1.0f,*d1) || !displaced(-3.0f,*d2))return 2;
            for(int fps : {60,30})
            {
                const int64_t tk=10000000/fps;const float dtf=1.0f/float(fps);
                rt::Continuity ex;ov::Pose out,ref;native_reload_contacts::Result r,rf;
                bool okAll=true;float maxHand=0.0f,maxWpn=0.0f,maxHandRef=0.0f,maxWpnRef=0.0f,lastHand=0.0f,lastWpn=0.0f,lastHandRef=0.0f,lastWpnRef=0.0f;
                math::Vec3 ph{},pw{},rh{},rw{};bool have=false;
                const int hold=40,fall=int(0.2f*float(fps)+0.5f);
                for(int k=0;k<hold+fall+2;++k)
                {
                    const float wk=k<hold ? 1.0f : std::max(0.0f,1.0f-float(k-hold+1)/float(fall));
                    const ov::Pose& nat=k<hold-1 ? *d1 : *d2;
                    const int64_t t=int64_t(40000000)+int64_t(k)*tk;
                    okAll=finish(int(ov::PoseOverlay::Rifle),nat,wk,out,r,&ex,t,uint64_t(1000+k)) && r.ok && okAll;
                    const bool refOk=finish(int(ov::PoseOverlay::Rifle),nat,wk,ref,rf,nullptr,0,uint64_t(1500+k));
                    const math::Vec3 h=inSpine3(out,rig.left.hand),w=inSpine3(out,rig.weaponRight);
                    const math::Vec3 hr=inSpine3(ref,rig.left.hand),wr=inSpine3(ref,rig.weaponRight);
                    if(have && k>=hold+1 && refOk)
                    {
                        lastHand=gik::Len(h-ph);lastWpn=gik::Len(w-pw);lastHandRef=gik::Len(hr-rh);lastWpnRef=gik::Len(wr-rw); // the last transition (into weight 0)
                        maxHand=std::max(maxHand,gik::Len(h-ph));maxWpn=std::max(maxWpn,gik::Len(w-pw));
                        maxHandRef=std::max(maxHandRef,gik::Len(hr-rh));maxWpnRef=std::max(maxWpnRef,gik::Len(wr-rw));
                    }
                    ph=h;pw=w;rh=hr;rw=wr;have=true;
                }
                std::printf("  exit at %d fps: max step hand %.4f (plain %.4f) weapon %.4f (plain %.4f) m/frame, all ok %d\n",fps,maxHand,maxHandRef,maxWpn,maxWpnRef,int(okAll));
                Check(okAll && lastHand<=lastHandRef*1.5f+0.005f && lastWpn<=lastWpnRef*1.5f+0.005f && maxHand<=maxHandRef+1.3f*rt::kLeftHandMaxSpeedMps*dtf && maxWpn<=maxWpnRef+1.3f*rt::kWeaponMaxSpeedMps*dtf,
                      fps==60 ? "ramp-out 60 fps: no step of the hand / weapon above the plain blend (lag falls with the weight, no pop to the baseline)"
                              : "ramp-out 30 fps: no step of the hand / weapon above the plain blend (lag falls with the weight, no pop to the baseline)");
                // new episode 0.1 s later (the weight was 0 in between): the first frame sits at the baseline again
                const int64_t t0=int64_t(40000000)+int64_t(hold+fall+2)*tk+10000000/10;
                ov::Pose first;native_reload_contacts::Result r2;
                const bool ok2=finish(int(ov::PoseOverlay::Rifle),*d1,0.2f,first,r2,&ex,t0,uint64_t(2000));
                const float fromBase=ok2?gik::Len(inSpine3(first,rig.weaponRight)-inSpine3(*base,rig.weaponRight)):9.0f;
                std::printf("  new episode after %d fps exit: first frame weapon %.4fm from the baseline\n",fps,fromBase);
                Check(ok2 && fromBase<0.02f,"new episode within .25s of the last: the followers start from the baseline again");
                (void)dtf;
            }
        }
        // Left hand at the weapon: after a snap to a hand that rests at the grip the lag is gone in a few frames (contact catch-up, mutation 16).
        {
            rt::Continuity ct;ov::Pose out;native_reload_contacts::Result r;bool okAll=true;int converge=-1;float ctMaxStep=0.0f;math::Vec3 ctPrev{};
            const float distNear=gik::Len(inWeaponFrame(*nearPose,rig.left.hand));
            for(int k=0;k<40;++k)
            {
                const ov::Pose& q=k<10 ? *farPose : *nearPose;
                okAll=finish(int(ov::PoseOverlay::Rifle),q,1.0f,out,r,&ct,int64_t(60000000)+int64_t(k)*166667,uint64_t(3000+k)) && r.ok && okAll;
                if(k>=10 && converge<0 && gik::Len(inWeaponFrame(out,rig.left.hand)-inWeaponFrame(*nearPose,rig.left.hand))<0.015f)converge=k-10;
                {const math::Vec3 hh=inWeaponFrame(out,rig.left.hand);if(k>10)ctMaxStep=std::max(ctMaxStep,gik::Len(hh-ctPrev));ctPrev=hh;}
            }
            std::printf("  contact catch-up: native hand %.3fm from the grip, within 1.5cm after %d frames after the snap, max step %.4fm/frame, all ok %d\n",distNear,converge,ctMaxStep,int(okAll));
            Check(okAll && distNear<rt::kLeftHandContactM+0.05f && converge>=0 && converge<=8 && ctMaxStep<=0.065f,"left hand at the grip: the lag of a snap is gone within 8 frames, no step above ~5-6cm per frame (contact catch-up)");
        }
        // Entry in motion (run / crouch walk): the game PUBLISHES a baseline that differs from the MM snapshot (native masked arms blended with the MM
        // pose) and moves with the gait. The episode starts from that published pose: no step above 5cm per frame of the weapon or the left hand
        // (Spine3 frame) over the ramp, the first frame sits where the published pose was (mutation 17 starts from the MM snapshot: 6cm jump).
        {
            const auto shifted=[&](float k,ov::Pose& q)->bool{
                q=*base;gik::Params wide;gik::Result r1,r2;
                const auto rh=native_reload::Model(sk,q,rig.right.hand),lh=native_reload::Model(sk,q,rig.left.hand);
                const math::Vec3 move=(frame.right*(-0.12f)+frame.up*0.10f+frame.fwd*0.08f)*k;
                return gik::SolveArm(sk,q,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) && gik::SolveArm(sk,q,rig.left,lh.t+move,&lh.r,1.0f,wide,r2);
            };
            auto nat=std::make_unique<ov::Pose>();
            if(!shifted(1.6f,*nat))return 2;
            float firstW=9.0f,firstH=9.0f,maxW=0.0f,maxH=0.0f;bool okAll=true;math::Vec3 pw{},ph{};rt::Continuity mv;
            for(int k=-1;k<14;++k)
            {
                auto pub=std::make_unique<ov::Pose>();
                if(!shifted(0.45f+0.04f*float(k+1),*pub))return 2; // 0.45 x 0.175m = 8cm away from the MM snapshot, moving 0.7cm per frame
                const float wk=k<0 ? 0.0f : std::min(1.0f,0.6f+0.145f*float(k)); // first weight .6: a rise after a dip (the MM-snapshot start would be 60% restored)
                ov::Pose out;native_reload_contacts::Result r;
                okAll=finish(int(ov::PoseOverlay::Rifle),*nat,wk,out,r,&mv,int64_t(70000000)+int64_t(k+1)*166667,uint64_t(4000+k+1),nullptr,pub.get()) && r.ok && okAll;
                const math::Vec3 w=inSpine3(out,rig.weaponRight),h=inSpine3(out,rig.left.hand);
                if(k==0){firstW=gik::Len(w-pw);firstH=gik::Len(h-ph);}
                if(k>=0)maxW=std::max(maxW,gik::Len(w-pw)),maxH=std::max(maxH,gik::Len(h-ph));
                pw=w;ph=h;
            }
            std::printf("  entry in motion: first frame step weapon %.4f hand %.4f, max step weapon %.4f hand %.4f m/frame, all ok %d\n",firstW,firstH,maxW,maxH,int(okAll));
            Check(okAll && firstW<=0.03f && firstH<=0.03f,"entry in motion: the first frame of the ramp starts from the published pose (step <= 3cm)");
            Check(maxW<=0.05f && maxH<=0.055f,"entry in motion: no step of the weapon / left hand above 5cm per frame over the ramp");
        }
        // The game's envelope jump on the first frame of the action (run / crouch walk): the pose PUBLISHED before the correction (ObservePublished, the
        // FkHook calls it on rifle frames without a correction) is remembered, the episode starts from it and the jump (7cm of the weapon / hand
        // between the last observed frame and the first correction frame) is spread over the ramp: no step above 5cm, first frame <= 5cm from the
        // observed pose. Observe is read-only. Mutation 20: the episode starts from the pose of its first frame (the full jump).
        {
            const auto shifted2=[&](float k,ov::Pose& q)->bool{
                q=*base;gik::Params wide;gik::Result r1,r2;
                const auto rh=native_reload::Model(sk,q,rig.right.hand),lh=native_reload::Model(sk,q,rig.left.hand);
                const math::Vec3 move=(frame.right*(-0.12f)+frame.up*0.10f+frame.fwd*0.08f)*k;
                return gik::SolveArm(sk,q,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) && gik::SolveArm(sk,q,rig.left,lh.t+move,&lh.r,1.0f,wide,r2);
            };
            auto pubA=std::make_unique<ov::Pose>(),pubB=std::make_unique<ov::Pose>(),nat2=std::make_unique<ov::Pose>();
            if(!shifted2(0.45f,*pubA) || !shifted2(0.85f,*pubB) || !shifted2(1.6f,*nat2))return 2;
            const float jump=gik::Len(inSpine3(*pubB,rig.weaponRight)-inSpine3(*pubA,rig.weaponRight));
            rt::Continuity pc;bool readOnly=true;
            for(int k=-3;k<=-1;++k)
            {
                integ::PoseSnapshot os=snap;os.traceOverlay=int(ov::PoseOverlay::Rifle);os.reloadContactsEligible=false;os.reloadContactsWeight=0.0f;
                std::vector<unsigned char> rec;Records(*pubA,rec);const std::vector<unsigned char> before=rec;
                rt::ObservePublished(prepared,os,rec.data(),sk.n,pc,int64_t(80000000)+int64_t(k+4)*166667);
                readOnly=readOnly && rec==before;
            }
            const math::Vec3 observedW=inSpine3(*pubA,rig.weaponRight),observedH=inSpine3(*pubA,rig.left.hand);
            float firstW=9.0f,firstH=9.0f,maxW=0.0f,maxH=0.0f;bool okAll=true;math::Vec3 pw=observedW,ph=observedH;
            for(int k=0;k<14;++k)
            {
                const float wk=std::min(1.0f,0.13f+0.145f*float(k));
                ov::Pose out;native_reload_contacts::Result r;
                okAll=finish(int(ov::PoseOverlay::Rifle),*nat2,wk,out,r,&pc,int64_t(80000000)+int64_t(k+4)*166667,uint64_t(5000+k),nullptr,pubB.get()) && r.ok && okAll;
                const math::Vec3 w=inSpine3(out,rig.weaponRight),h=inSpine3(out,rig.left.hand);
                if(k==0){firstW=gik::Len(w-pw);firstH=gik::Len(h-ph);}
                maxW=std::max(maxW,gik::Len(w-pw));maxH=std::max(maxH,gik::Len(h-ph));
                pw=w;ph=h;
            }
            std::printf("  published-pose start: envelope jump %.3fm, first frame step weapon %.4f hand %.4f, max step weapon %.4f hand %.4f m/frame, observe read-only %d, all ok %d\n",
                        jump,firstW,firstH,maxW,maxH,int(readOnly),int(okAll));
            Check(jump>0.06f && readOnly,"fixture: the published pose jumps by > 6cm on the first action frame; ObservePublished never writes the records");
            Check(okAll && firstW<=0.05f && firstH<=0.05f && maxW<=0.05f && maxH<=0.055f,
                  "published-pose start: the envelope jump is inertialised (no weapon / hand step above 5cm per frame from the observed pose)");
        }
        // Joint slew on the ramp: rifles use the ordinary slew (the followers own the ramp speed), pistols keep the 1000 deg/s ramp slew bit for bit
        // (mutation 21 restores the ramp slew for rifles).
        Check(rt::JointSlewForOverlay(int32_t(ov::PoseOverlay::Rifle),0.5f)==rt::kJointSlewRadPerSec &&
              rt::JointSlewForOverlay(int32_t(ov::PoseOverlay::Pistol2H),0.5f)==rt::JointSlewFor(0.5f) &&
              rt::JointSlewForOverlay(int32_t(ov::PoseOverlay::Pistol1H),0.5f)==rt::kJointSlewRampRadPerSec &&
              rt::JointSlewForOverlay(int32_t(ov::PoseOverlay::Pistol2H),1.0f)==rt::kJointSlewRadPerSec,
              "joint slew: rifle ramp = ordinary slew; pistols unchanged (ramp slew below weight .999)");
        // Residual clearance of the followed hand: a new entry into the torso is pushed out fast (mutation 18: the old 1.5 m/s rise), the release is slow.
        {
            rt::LeftArmState ps;ps.StepPost(math::Vec3{},7,3000000,10000000);
            math::Vec3 o{};for(int k=1;k<=2;++k)o=ps.StepPost(math::Vec3{0.0f,0.066f,0.0f},7,3000000+int64_t(k)*166667,10000000);
            float down=0.0f;{const math::Vec3 a=o;const math::Vec3 b=ps.StepPost(math::Vec3{},7,3000000+int64_t(3)*166667,10000000);down=gik::Len(a)-gik::Len(b);}
            std::printf("  post clearance: %.4fm after 2 frames of a 6.6cm entry, release %.4f m/frame\n",gik::Len(o),down);
            Check(gik::Len(o)>=0.064f && down<=0.75f*0.0166667f+1.0e-4f,"left arm residual clearance: a new entry is pushed out within 2 frames, the release is rate limited");
        }
        // Same-tick repeat (the placement guard re-runs place() with a lowered weight): the follower keeps the lag, not the absolute position.
        {
            rt::LeftArmState hs;hs.StepHand(math::Vec3{0.0f,0.0f,0.0f},math::Vec3{0.0f,0.0f,0.0f},9,2000000,10000000);
            const math::Vec3 first=hs.StepHand(math::Vec3{0.30f,0.0f,0.0f},math::Vec3{0.30f,0.0f,0.0f},9,2000000+166667,10000000);
            const math::Vec3 again=hs.StepHand(math::Vec3{0.10f,0.0f,0.0f},math::Vec3{0.10f,0.0f,0.0f},9,2000000+166667,10000000);
            const float lagFirst=0.30f-first.x,lagAgain=0.10f-again.x;
            std::printf("  follower same tick: lag first %.4f again %.4f\n",lagFirst,lagAgain);
            Check(lagFirst>0.01f && std::fabs(lagFirst-lagAgain)<1.0e-5f,"follower: a repeated call in the same tick keeps the lag (the hand follows a lowered correction weight)");
        }
        // The stock shift never takes the left arm deeper into the torso than the native target already has it: scan stock fixtures
        // (stock sunk by `back`, left hand placed near the chest).
        float worst=-1.0f;int scanned=0;
        for(float back : {0.03f,0.06f,0.09f,0.12f})
          for(float lift : {-0.04f,0.0f,0.04f,0.08f})
            for(float lx : {-0.04f,-0.10f})
              for(float lz : {-0.04f,-0.10f,-0.16f,-0.22f})
                for(float ly : {0.05f,0.09f,0.13f})
                {
                    auto q=std::make_unique<ov::Pose>();
                    {
                        *q=*base;
                        const auto rh=native_reload::Model(sk,*q,rig.right.hand);
                        gik::Params wide;gik::Result r1,r2;
                        const math::Vec3 move=(frame.fwd+frame.right)*(-back)+frame.up*lift;
                        const math::Vec3 lh=frame.neck+frame.right*lx+frame.fwd*ly+frame.up*lz;
                        if(!gik::SolveArm(sk,*q,rig.right,rh.t+move,&rh.r,1.0f,wide,r1) || !gik::SolveArm(sk,*q,rig.left,lh,nullptr,1.0f,wide,r2))continue;
                    }
                    const math::Vec3 off=rt::StockClearanceOffset(prepared,*base,*base,*q);
                    if(!(gik::Len(off)>0.0f))continue;
                    math::Vec3 pts[rt::kLeftArmPoints];rt::LeftArmPoints(prepared,*q,pts);
                    const float grow=rt::LeftArmTorsoDepth(frame,pts,off)-rt::LeftArmTorsoDepth(frame,pts,math::Vec3{});
                    worst=std::max(worst,grow);++scanned;
                }
        std::printf("  stock shift vs left arm: %d fixtures, largest growth of the left arm torso depth %.4fm\n",scanned,worst);
        Check(scanned>=4 && worst<=0.0015f,"the stock clearance shift does not drive the left arm deeper into the torso");
    }

    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

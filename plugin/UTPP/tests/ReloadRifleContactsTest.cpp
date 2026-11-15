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
// UTPP_RELOAD_CONTACT_MUTATION=25: the reject replay / ramp-in fades towards the records as handed in (the game's girdle), section 10.
// UTPP_RELOAD_CONTACT_MUTATION=26: the girdle is restored with the weight shrunk by the placement guard (the game's girdle), section 12.
// UTPP_RELOAD_CONTACT_MUTATION=27: no final anatomy stage of the left arm (LimitArmFinal), sections 12 (13e) and 13.
// UTPP_RELOAD_GROUND_KEEP_MUTATION=1: no ground keep, 2: no edge latch (a pistol reload begun moving stays uncorrected), 3: no girdle
// hold after an early episode end (sections 9 and 11).
// Every mutation build must report an assertion FAIL (ctest PASS_REGULAR_EXPRESSION "FAIL ").
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/OverlayFile.hpp"
#include "../src/mm/ReloadContactFade.hpp"
#include "TestFailFast.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
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

    // 9) Pistol reload that began standing, then V starts to walk / run (GroundMoveKeep, ReloadContactFade.hpp). Policy at 60 fps with
    //    the production CorrectionFade / UpperEnvelope: the keep never changes a frame the former rule allows (standing reloads are
    //    bit-identical), keeps a full correction through the walk start and the exit, ends on crouch / air / slide without re-acquiring,
    //    and never latches during the entry ramp. Reloads that begin moving latch on their edge (section 11).
    {
        struct G{bool stationary,groundStand;};
        const auto play=[&](auto&& at,int len,bool keepOn,std::vector<float>& w,bool& latched){
            rt::CorrectionFade fade;rt::UpperEnvelope env;rt::CorrectionEpisode ep;ep.Begin();
            const float dt=1.0f/60.0f;w.clear();latched=false;
            for(int k=0;k<len+20;++k)
            {
                const G g=at(k);
                const float e=env.Step(dt,k<len,false,true);
                const bool before=rt::ReloadCorrectionPose(false,g.stationary,0,false); // non-authored pistol: the pose rule alone
                const bool keep=keepOn && ep.groundKeep.Step(true,before,g.groundStand,fade.weight,k==0); // k 0: the episode edge
                latched=latched || ep.groundKeep.latched;
                w.push_back(fade.Step(dt,e,before || keep,true,true));
            }
        };
        const float dt=1.0f/60.0f,fall=dt/0.2f+1.0e-5f;
        const auto same=[](const std::vector<float>& a,const std::vector<float>& b){
            if(a.size()!=b.size())return false;
            for(size_t i=0;i<a.size();++i)if(std::memcmp(&a[i],&b[i],sizeof(float))!=0)return false;
            return true;};
        std::vector<float> before,kept;bool latchedBefore=false,latched=false;
        const auto standing=[](int){return G{true,true};};
        play(standing,100,false,before,latchedBefore);play(standing,100,true,kept,latched);
        Check(same(before,kept) && !latched,"ground keep: a standing pistol reload is bit-identical (never latched)");
        // A reload that begins walking latches on its edge (section 11 covers it in full); here only that it is corrected at all.
        const auto walking=[](int){return G{false,true};};
        play(walking,100,true,kept,latched);
        Check(latched && kept[50]>=0.999f,"ground keep: a pistol reload that began walking latches on its edge (corrected, section 11)");
        const auto entryMove=[](int k){return G{k<5,true};};
        play(entryMove,100,false,before,latchedBefore);play(entryMove,100,true,kept,latched);
        Check(same(before,kept) && !latched,"ground keep: V moving during the entry ramp (correction not full) keeps the former withdrawal");
        const auto runStart=[](int k){return G{k<60,true};};
        play(runStart,100,false,before,latchedBefore);play(runStart,100,true,kept,latched);
        float minKept=1.0f,maxDrop=0.0f,minBefore=1.0f;
        for(int k=60;k<100;++k){minKept=std::min(minKept,kept[size_t(k)]);minBefore=std::min(minBefore,before[size_t(k)]);}
        for(size_t k=1;k<kept.size();++k)maxDrop=std::max(maxDrop,kept[k-1]-kept[k]);
        std::printf("  walk start inside a standing pistol reload: former min %.3f | kept min %.3f max drop/frame %.4f end %.4f\n",
                    minBefore,minKept,maxDrop,kept.back());
        Check(minBefore<=1.0e-6f,"fixture: the former pistol rule withdraws the correction on the walk start");
        Check(latched && minKept>=0.999f && maxDrop<=fall && kept.back()<=1.0e-6f,
              "ground keep: the walk start keeps the full correction to the end of the reload, the exit falls with the envelope (.2 s)");
        const auto crouch=[](int k){return k<60 ? G{true,true} : k<75 ? G{false,true} : k<85 ? G{false,false} : G{false,true};};
        play(crouch,110,true,kept,latched);
        float minWalk=1.0f,riseAfter=0.0f,dropCrouch=0.0f;
        for(int k=60;k<75;++k)minWalk=std::min(minWalk,kept[size_t(k)]);
        for(int k=76;k<110;++k){riseAfter=std::max(riseAfter,kept[size_t(k)]-kept[size_t(k-1)]);dropCrouch=std::max(dropCrouch,kept[size_t(k-1)]-kept[size_t(k)]);}
        Check(minWalk>=0.999f && riseAfter<=0.0f && dropCrouch<=fall && kept[109]<=1.0e-6f,
              "ground keep: a crouch / air / slide frame ends it as a .2 s withdrawal, walking upright again does not re-acquire it");
        rt::GroundMoveKeep rifleKeep;
        Check(!rifleKeep.Step(false,false,true,1.0f) && !rifleKeep.latched,"ground keep: never for a rifle / under an owner without the pistol solve");
        {
            integ::PoseSnapshot a=snap;a.traceOverlay=int32_t(ov::PoseOverlay::Pistol1H);
            rt::Admit(a,rt::HookOwner::FK);const bool fk=rt::OwnerCorrectsPistols();
            rt::Admit(a,rt::HookOwner::Mapping);const bool mapping=rt::OwnerCorrectsPistols();
            Check(fk==rt::FkOwnerOverlay(int32_t(ov::PoseOverlay::Pistol1H)) && !fk && mapping,
                  "owner flag: the FK owner (no pistol solve) disables the ground keep, the mapping owner enables it");
        }
        rt::CorrectionEpisode ep;ep.Begin();ep.groundKeep.Step(true,false,true,1.0f);const bool wasLatched=ep.groundKeep.latched;ep.Begin();
        Check(wasLatched && !ep.groundKeep.latched && !ep.groundKeep.closed,"ground keep: a new reload episode starts without the keep");

        // Shoulder girdle bits: only the clavicle channels leave the weapon-action mask.
        const auto action=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});
        uint32_t bits[sizeof(action.bits)/sizeof(action.bits[0])];std::memcpy(bits,action.bits,sizeof(bits));
        const int cleared=rt::ClearShoulderGirdleBits(bits,uint32_t(sk.n),2,[&](uint32_t b){return file.BoneName(b);});
        const int lc=sk.parent[rig.left.upper],rc=sk.parent[rig.right.upper];
        bool othersSame=true;
        for(int b=0;b<sk.n;++b)
        {
            const bool was=(action.bits[b>>5]>>(b&31))&1u,now=(bits[b>>5]>>(b&31))&1u;
            const std::string_view nm=file.BoneName(b);
            const bool girdle=nm.substr(0,12)=="LeftShoulder" || nm.substr(0,13)=="RightShoulder";
            othersSame=othersSame && (girdle ? !now : now==was);
        }
        Check(cleared>=2 && othersSame && !((bits[lc>>5]>>(lc&31))&1u) && !((bits[rc>>5]>>(rc&31))&1u) &&
              ((bits[rig.left.upper>>5]>>(rig.left.upper&31))&1u),
              "girdle bits: both clavicles leave the weapon-action mask, the arms and every other channel stay");

        // Whole frame: the game's clavicles collapse 8 cm each towards the midline (the recorded run start). Records are composed the
        // way the hook does (BlendIntoRecords: masked channels at upperScale = 1 - envelope, the rest at lowerScale 1), then Finish.
        auto native=std::make_unique<ov::Pose>(*base);
        {
            const auto lm=native_reload::Model(sk,*base,lc).t,rm=native_reload::Model(sk,*base,rc).t;
            const math::Vec3 in=(rm-lm)*(1.0f/gik::Len(rm-lm));
            for(int side=0;side<2;++side)
            {
                const int c=side ? rc : lc;
                auto m=native_reload::Model(sk,*native,c);
                m.t=m.t+in*(side ? -0.08f : 0.08f);
                native->b[c]=native_reload::Relative(native_reload::Model(sk,*native,sk.parent[c]),m);
            }
        }
        const auto width=[&](const ov::Pose& q){
            return gik::Len(native_reload::Model(sk,q,rig.left.upper).t-native_reload::Model(sk,q,rig.right.upper).t);};
        const auto frameAt=[&](float envelope,float weight,bool girdleOff,ov::Pose& out){
            integ::PoseSnapshot s=snap;s.traceOverlay=int32_t(ov::PoseOverlay::Pistol2H);s.sequence=900;s.reloadContactsWeight=weight;
            s.upperScale=1.0f-envelope;s.lowerScale=1.0f;
            if(girdleOff)rt::ClearShoulderGirdleBits(s.upperBits,uint32_t(sk.n),2,[&](uint32_t b){return file.BoneName(b);});
            std::vector<unsigned char> records;Records(*native,records);rt::Cycle cycle;
            if(!rt::Capture(prepared,s,records.data(),sk.n,true,cycle))return false;
            integ::BlendIntoRecords(records.data(),uint32_t(sk.n),s,1.0f);
            native_reload_contacts::Result r;
            if(weight>0.0f && !rt::Finish(prepared,s,records.data(),sk.n,true,cycle,r,weight,rt::kAuthoredContactMinReach,nullptr,0,10000000))return false;
            return rt::Read(prepared,records.data(),out);
        };
        const float wOwn=width(*base),wGame=width(*native);
        ov::Pose formerMoving,keptFull,keptCleared;
        const bool okF=frameAt(1.0f,0.0f,false,formerMoving),okK=frameAt(1.0f,1.0f,false,keptFull),okC=frameAt(1.0f,1.0f,true,keptCleared);
        float clavStep=9.0f,handStep=9.0f;
        if(okK && okC)
        {
            clavStep=std::max(gik::Len(native_reload::Model(sk,keptFull,lc).t-native_reload::Model(sk,keptCleared,lc).t),
                              gik::Len(native_reload::Model(sk,keptFull,rc).t-native_reload::Model(sk,keptCleared,rc).t));
            handStep=std::max(gik::Len(native_reload::Model(sk,keptFull,rig.left.hand).t-native_reload::Model(sk,keptCleared,rig.left.hand).t),
                              gik::Len(native_reload::Model(sk,keptFull,rig.right.hand).t-native_reload::Model(sk,keptCleared,rig.right.hand).t));
        }
        float minExit=9.0f,minExitFormer=9.0f;bool exitOk=true;
        for(int k=0;k<=12;++k)
        {
            const float e=1.0f-float(k)/12.0f;ov::Pose a,f2;
            exitOk=exitOk && frameAt(e,e,true,a) && frameAt(e,e,false,f2);
            if(exitOk){minExit=std::min(minExit,width(a));minExitFormer=std::min(minExitFormer,width(f2));}
        }
        std::printf("  girdle: own width %.1fcm, game %.1fcm | former walk start %.1fcm | kept %.1fcm, kept+girdle %.1fcm (clavicle step %.2fmm, hand step %.2fmm) | exit min %.1fcm (former exit %.1fcm)\n",
                    wOwn*100.0f,wGame*100.0f,okF?width(formerMoving)*100.0f:-1.0f,okK?width(keptFull)*100.0f:-1.0f,okC?width(keptCleared)*100.0f:-1.0f,
                    clavStep*1000.0f,handStep*1000.0f,minExit*100.0f,minExitFormer*100.0f);
        Check(wGame<wOwn-0.12f,"fixture: the game's clavicles are more than 12 cm narrower than V's own");
        Check(okF && width(formerMoving)<wOwn-0.10f,"fixture: the former walk start (correction withdrawn, mask on the clavicles) shows the narrow game girdle");
        Check(okK && okC && std::fabs(width(keptCleared)-wOwn)<0.002f && std::fabs(width(keptFull)-wOwn)<0.002f,
              "girdle: the kept correction holds V's own shoulder width, with and without the clavicle bits");
        Check(clavStep<0.001f && handStep<0.001f,"girdle: clearing the clavicle bits under a full correction moves no clavicle or hand (no pop at the latch)");
        Check(exitOk && minExit>=wOwn-0.002f,"girdle: through the .2 s exit the shoulder width stays V's own at every frame");
    }

    // 10) Rejected frames inside a reload (RejectHold, GirdleReference). The game's reload choreography moves its clavicles (both 8 cm
    //     towards the midline, the left one 6 cm up / forward as well) and the left upper-arm bone translation; the records carry them on
    //     the masked channels (upper weight 1). Accepted frames, then .13 s without an accepted solve (the replay of RejectHold, driven
    //     through HoldOnly: the hand followers and the soft reach of this fixture accept even a native hand 88 cm away), then accepted
    //     frames again (ramp-in), 60 fps with the production Continuity. Every frame (accepted, replayed, ramp-in) keeps
    //     V's own clavicle translation and arm bone translations, the clavicle turns at most by the reach step's 20 deg away from V's own,
    //     and the shoulder socket never steps by more than the faded reach rotation. Mutation 25 (replay towards the records) collapses it.
    {
        const int lc=sk.parent[rig.left.upper],rc=sk.parent[rig.right.upper];
        auto game=std::make_unique<ov::Pose>(*base);
        {
            const auto lm=native_reload::Model(sk,*base,lc).t,rm=native_reload::Model(sk,*base,rc).t;
            const math::Vec3 in=(rm-lm)*(1.0f/gik::Len(rm-lm));
            for(int side=0;side<2;++side)
            {
                const int c=side ? rc : lc;
                auto m=native_reload::Model(sk,*game,c);
                m.t=m.t+in*(side ? -0.08f : 0.08f)+(side ? math::Vec3{} : math::Vec3{0.0f,0.04f,0.045f});
                game->b[c]=native_reload::Relative(native_reload::Model(sk,*game,sk.parent[c]),m);
            }
            game->b[rig.left.upper].t=game->b[rig.left.upper].t*1.08f;
        }
        const int64_t tps=10000000,step=tps/60;
        struct Sample{bool ok;float amp,clavT,armT,clavDeg,socketStep,width;};
        const auto angle=[](math::Quat a,math::Quat b){
            return 2.0f*std::acos(std::clamp(std::fabs(math::QuatDot(math::QuatNormalize(a),math::QuatNormalize(b))),0.0f,1.0f))*180.0f/math::kPi;};
        const auto run=[&](int overlay,bool girdleOff,std::vector<Sample>& out){
            rt::Continuity state;out.clear();
            math::Vec3 prevSocket{};bool havePrev=false;
            for(int k=0;k<40;++k)
            {
                // frames 12-19 without an accepted solve: the replay path (HoldOnly runs the reject branch of Finish, RejectHold::OnReject)
                const bool held=k>=12 && k<20;
                integ::PoseSnapshot s=snap;s.traceOverlay=overlay;s.sequence=uint64_t(1000+k);s.reloadContactsWeight=1.0f;
                s.upperScale=0.0f;s.lowerScale=1.0f;
                if(girdleOff)rt::ClearShoulderGirdleBits(s.upperBits,uint32_t(sk.n),2,[&](uint32_t b){return file.BoneName(b);});
                std::vector<unsigned char> records;Records(*game,records);rt::Cycle cycle;
                Sample m{};
                const bool captured=!held && rt::Capture(prepared,s,records.data(),sk.n,true,cycle);
                integ::BlendIntoRecords(records.data(),uint32_t(sk.n),s,1.0f);
                native_reload_contacts::Result r;
                ov::Pose o;
                if(held)rt::HoldOnly(prepared,s,records.data(),sk.n,true,state,int64_t(k+1)*step,tps);
                m.ok=captured && rt::Finish(prepared,s,records.data(),sk.n,true,cycle,r,1.0f,rt::kAuthoredContactMinReach,&state,
                                            int64_t(k+1)*step,tps) && r.ok;
                if(!rt::Read(prepared,records.data(),o)){out.push_back(Sample{false,-1.0f,9.0f,9.0f,180.0f,9.0f,0.0f});continue;}
                m.amp=state.reject.amp;
                m.clavT=std::max(gik::Len(o.b[lc].t-base->b[lc].t),gik::Len(o.b[rc].t-base->b[rc].t));
                m.armT=std::max(gik::Len(o.b[rig.left.upper].t-base->b[rig.left.upper].t),gik::Len(o.b[rig.right.upper].t-base->b[rig.right.upper].t));
                m.clavDeg=std::max(angle(o.b[lc].r,base->b[lc].r),angle(o.b[rc].r,base->b[rc].r));
                const math::Vec3 socket=inSpine3(o,rig.left.upper);
                m.socketStep=havePrev ? gik::Len(socket-prevSocket) : 0.0f;prevSocket=socket;havePrev=true;
                m.width=gik::Len(native_reload::Model(sk,o,rig.left.upper).t-native_reload::Model(sk,o,rig.right.upper).t);
                out.push_back(m);
            }
        };
        const float wOwn=gik::Len(native_reload::Model(sk,*base,rig.left.upper).t-native_reload::Model(sk,*base,rig.right.upper).t);
        const float wGame=gik::Len(native_reload::Model(sk,*game,rig.left.upper).t-native_reload::Model(sk,*game,rig.right.upper).t);
        for(int pass=0;pass<2;++pass)
        {
            const bool pistol=pass==1;
            std::vector<Sample> v;run(pistol ? int(ov::PoseOverlay::Pistol2H) : int(ov::PoseOverlay::Rifle),pistol,v);
            bool acceptedBefore=true,acceptedAfter=true,rejected=true;
            float minAmp=1.0f,clavT=0.0f,armT=0.0f,clavDeg=0.0f,socketStep=0.0f,minWidth=9.0f,minWidthAccepted=9.0f;
            for(int k=0;k<40;++k)
            {
                const auto& m=v[size_t(k)];
                if(k<12){acceptedBefore=acceptedBefore && m.ok;minWidthAccepted=std::min(minWidthAccepted,m.width);}
                else if(k<20)rejected=rejected && !m.ok;
                else if(k>=28)acceptedAfter=acceptedAfter && m.ok;
                if(k>=12)minAmp=std::min(minAmp,m.amp);
                clavT=std::max(clavT,m.clavT);armT=std::max(armT,m.armT);clavDeg=std::max(clavDeg,m.clavDeg);
                socketStep=std::max(socketStep,m.socketStep);minWidth=std::min(minWidth,m.width);
            }
            std::printf("  reject replay (%s): accepted %d/%d, rejected %d, min amp %.2f | own width %.1fcm, game %.1fcm, min width %.1fcm | "
                        "clavicle translation %.2fmm, arm bone translation %.2fmm, clavicle turn %.1fdeg, socket step %.2fcm/frame\n",
                        pistol ? "Pistol2H, girdle bits cleared" : "Rifle",int(acceptedBefore),int(acceptedAfter),int(rejected),minAmp,
                        wOwn*100.0f,wGame*100.0f,minWidth*100.0f,clavT*1000.0f,armT*1000.0f,clavDeg,socketStep*100.0f);
            const char* tag=pistol ? "reject replay (pistol, girdle bits cleared)" : "reject replay (rifle)";
            Check(acceptedBefore && acceptedAfter && rejected && minAmp<0.05f,
                  (std::string("fixture: ")+tag+": accepted, rejected for .13 s (the replay fades out), accepted again").c_str());
            Check(pistol || wGame<wOwn-0.10f,"fixture: the game's girdle is more than 10 cm narrower than V's own");
            Check(clavT<0.001f && armT<0.001f,(std::string(tag)+": V's own clavicle and arm bone translations on every frame").c_str());
            Check(clavDeg<=20.5f && socketStep<0.015f && minWidth>=std::min(wOwn,minWidthAccepted)-0.005f,
                  (std::string(tag)+": the clavicle turns at most by the reach step, no shoulder pop at the rejection or the ramp-in").c_str());
        }
    }

    // 11) Pistol reload that BEGINS while V walks / runs upright on the ground (GroundMoveKeep edge latch, GirdleHold). Policy through the
    //     production CorrectionEpisode::Observe / CorrectionFade / UpperEnvelope at 60 fps in the driver's order, then whole frames on the
    //     Female rig with the recorded walking handgun girdle (left clavicle 14 cm, right 4.5 cm towards the midline; the left one 26 cm
    //     in the last .3 s of the reload, the clavicles crossed). Mutation 2 (no edge latch) and 3 (no hold after an early episode end)
    //     must fail here.
    {
        struct G{bool stationary,groundStand;float speed;bool domain;};
        struct F{float e,w;bool girdleOff,latched,fromEdge;};
        const float dt=1.0f/60.0f,fall=dt/0.2f+1.0e-5f;
        const auto play=[&](auto&& at,int len,bool authored,bool keepOn,std::vector<F>& out){
            rt::CorrectionFade fade;rt::UpperEnvelope env;rt::CorrectionEpisode ep;rt::GirdleHold hold;ep.Begin();out.clear();
            for(int k=0;k<len+24;++k)
            {
                const G g=at(k);
                const float e=env.Step(dt,k<len && g.domain,false,true);
                const bool stEp=ep.Observe(k<len,g.stationary,g.speed,g.domain,e,authored);
                const bool before=(rt::kReloadCorrectionMoving || rt::ReloadCorrectionPose(false,g.stationary,0,false)) && (!authored || stEp);
                const bool keep=keepOn && ep.groundKeep.Step(g.domain,before,g.groundStand,fade.weight,ep.edgeFrame);
                const float w=fade.Step(dt,e,before || keep,g.domain,true);
                const bool off=keepOn ? hold.Step(ep.groundKeep.latched,ep.groundKeep.fromEdge,ep.Live(),e>0.0f || fade.Active())
                                      : ep.groundKeep.latched;
                out.push_back(F{e,w,off,ep.groundKeep.latched,ep.groundKeep.fromEdge});
            }
        };
        const auto maxDiff=[](const std::vector<F>& a,const std::vector<F>& b){
            float d=a.size()==b.size() ? 0.0f : 9.0f;
            for(size_t i=0;i<a.size() && i<b.size();++i)d=std::max(d,std::fabs(a[i].w-b[i].w));
            return d;};
        const auto anyLatch=[](const std::vector<F>& a){for(const auto& f:a)if(f.latched)return true;return false;};
        constexpr int len=85; // 1.41 s, the recorded handgun reload
        std::vector<F> stand,walk,former,aStand,aWalk,shot,standShot;
        play([](int){return G{true,true,0.0f,true};},len,false,true,stand);
        play([](int){return G{false,true,1.37f,true};},len,false,true,walk);
        play([](int){return G{false,true,1.37f,true};},len,false,false,former);
        float peakWalk=0.0f,peakFormer=0.0f;for(const auto& f:walk)peakWalk=std::max(peakWalk,f.w);for(const auto& f:former)peakFormer=std::max(peakFormer,f.w);
        std::printf("  begun walking (game clip): former peak %.3f | edge keep peak %.3f, max diff to the standing reload %.2e\n",
                    peakFormer,peakWalk,maxDiff(walk,stand));
        Check(peakFormer<=1.0e-6f,"fixture: the former rule never corrects a pistol reload that began walking");
        Check(walk[0].latched && walk[0].fromEdge && peakWalk>=0.999f && maxDiff(walk,stand)<=1.0e-5f,
              "edge keep: a game-clip pistol reload begun walking gets the standing reload's correction weights (ramp-in, full, exit)");
        play([](int){return G{true,true,0.0f,true};},len,true,true,aStand);
        play([](int){return G{false,true,1.37f,true};},len,true,true,aWalk);
        Check(aWalk[0].latched && maxDiff(aWalk,aStand)<=1.0e-5f,
              "edge keep: an authored pistol reload begun walking gets the standing authored reload's correction weights");
        {
            std::vector<F> brake,brakeFormer;
            play([](int){return G{true,true,0.5f,true};},len,false,true,brake);play([](int){return G{true,true,0.5f,true};},len,false,false,brakeFormer);
            Check(!anyLatch(brake) && maxDiff(brake,brakeFormer)==0.0f,"edge keep: a game-clip reload begun braking on a stationary pose keeps the former rule");
        }
        // Begun crouched / in the air / on a slide (not upright on the ground on the edge frame): the former rule, never latched.
        for(int variant=0;variant<2;++variant)
        {
            std::vector<F> a,b;
            const auto at=[variant](int k){return G{false,variant==0 ? false : k>0,1.37f,true};}; // 0: whole reload, 1: only the edge frame
            play(at,len,false,true,a);play(at,len,false,false,b);
            Check(!anyLatch(a) && maxDiff(a,b)==0.0f,variant==0 ? "edge keep: a reload begun crouched / in the air / sliding keeps the former rule"
                                                               : "edge keep: an edge frame that is not upright on the ground never latches later");
        }
        {
            std::vector<F> c;
            play([](int k){return G{false,k<40 || k>=55,1.37f,true};},len,false,true,c); // walk, crouch 40-54, upright again
            float dropMax=0.0f,riseAfter=0.0f;
            for(int k=41;k<len+24;++k){dropMax=std::max(dropMax,c[size_t(k-1)].w-c[size_t(k)].w);riseAfter=std::max(riseAfter,c[size_t(k)].w-c[size_t(k-1)].w);}
            Check(c[39].w>=0.999f && dropMax<=fall && c[53].w<=1.0e-6f && riseAfter<=0.0f && !c[60].girdleOff,
                  "edge keep: a crouch in a reload begun walking withdraws the correction over .2 s, no re-acquire, the clavicle bits return");
        }
        {
            rt::GroundMoveKeep k;
            Check(!k.Step(false,false,true,0.0f,true) && !k.latched,"edge keep: never without the pistol solve (rifle, FK owner)");
        }
        // Clavicle bits: cleared from the edge frame to the end of the handback; after a shot (episode reset in one frame) the hold keeps
        // them for a reload begun moving, a keep that latched in a reload begun standing returns them as before.
        {
            bool edgeOff=true;for(const auto& f:walk)if(f.e>0.0f || f.w>0.0f)edgeOff=edgeOff && f.girdleOff;
            play([](int k){return G{false,true,1.37f,k<len+3};},len,false,true,shot);
            play([](int k){return G{k<50,true,k<50 ? 0.0f : 1.37f,k<len+3};},len,false,true,standShot);
            bool shotOff=true;int heldFrames=0;
            for(int k=len+3;k<len+24;++k){if(shot[size_t(k)].e>0.0f){shotOff=shotOff && shot[size_t(k)].girdleOff;++heldFrames;}else shotOff=shotOff && !shot[size_t(k)].girdleOff;}
            bool standSame=true;for(const auto& f:standShot)standSame=standSame && f.girdleOff==f.latched;
            std::printf("  clavicle bits: cleared on every reload frame %d | after the shot held %d frames (upper weight at the shot %.3f)\n",
                        int(edgeOff),heldFrames,shot[size_t(len+3)].e);
            Check(edgeOff,"girdle: a reload begun walking clears the clavicle bits from its first frame to the end of the handback");
            Check(heldFrames>0 && shotOff,"girdle hold: after a shot the clavicles stay on V's pose until the reload upper weight is 0");
            Check(anyLatch(standShot) && standSame,"girdle hold: a keep latched in a reload begun standing returns the bits with the episode (unchanged)");
        }

        // Whole frames: composition (BlendIntoRecords) + Capture + Finish with the production Continuity.
        const int lc=sk.parent[rig.left.upper],rc=sk.parent[rig.right.upper];
        const auto lm0=native_reload::Model(sk,*base,lc).t,rm0=native_reload::Model(sk,*base,rc).t;
        const math::Vec3 inDir=(rm0-lm0)*(1.0f/gik::Len(rm0-lm0));
        const auto girdle=[&](float left,float right,float forearmTwistDeg){
            auto q=std::make_unique<ov::Pose>(*base);
            for(int side=0;side<2;++side)
            {
                const int c=side ? rc : lc;
                auto m=native_reload::Model(sk,*q,c);m.t=m.t+inDir*(side ? -right : left);
                q->b[c]=native_reload::Relative(native_reload::Model(sk,*q,sk.parent[c]),m);
            }
            if(forearmTwistDeg!=0.0f)
                q->b[rig.left.lower].r=math::QuatNormalize(math::QuatMul(q->b[rig.left.lower].r,
                    gik::AxisAngle(math::Vec3{0.0f,1.0f,0.0f},forearmTwistDeg*math::kPi/180.0f)));
            return q;};
        const int64_t tps=10000000,tick=tps/60;
        const auto width=[&](const ov::Pose& q){
            return gik::Len(native_reload::Model(sk,q,rig.left.upper).t-native_reload::Model(sk,q,rig.right.upper).t);};
        const auto gap=[&](const ov::Pose& q){return math::Dot(native_reload::Model(sk,q,rc).t-native_reload::Model(sk,q,lc).t,inDir);};
        const auto inWeapon=[&](const ov::Pose& q,int bone){
            const auto fr=native_reload::Model(sk,q,rig.weaponRight);
            return math::QuatRotate(math::QuatInv(fr.r),native_reload::Model(sk,q,bone).t-fr.t);};
        const auto elbowRy=[&](const ov::Pose& q){
            const auto r=math::QuatNormalize(math::QuatMul(math::QuatInv(base->b[rig.left.lower].r),q.b[rig.left.lower].r));
            return std::fabs(std::asin(std::clamp(2.0f*(r.w*r.y-r.z*r.x),-1.0f,1.0f)))*180.0f/math::kPi;};
        struct W{int frames=0,accepted=0;float minWidth=9,minGap=9,clavStep=0,socketStep=0,startStep=0,endStep=0,handErr=0,maxRy=0;bool ok=true;};
        const auto wholeRun=[&](const std::vector<F>& seq,bool authored,float twistDeg,bool bitsOn){
            auto walkGame=girdle(0.14f,0.045f,twistDeg),lateGame=girdle(0.26f,0.045f,twistDeg);
            rt::Continuity state;W r;
            ov::Pose prev=*base;bool havePrev=true; // the frame before the reload: V's own pose (upper scale 1)
            for(size_t k=0;k<seq.size();++k)
            {
                const F& f=seq[k];
                const ov::Pose& game=int(k)>=len-18 ? *lateGame : *walkGame;
                const float cw=authored ? rt::AuthoredFadeWeight(f.w) : f.w;
                integ::PoseSnapshot s=snap;s.traceOverlay=int32_t(ov::PoseOverlay::Pistol1H);s.sequence=uint64_t(3000+k);
                s.reloadContactsWeight=cw;s.reloadContactsEligible=cw>0.0f;s.reloadAuthoredClip=authored && cw>0.0f;
                s.upperScale=1.0f-(authored ? rt::AuthoredFadeWeight(f.e) : f.e);s.lowerScale=1.0f;
                if(f.girdleOff && !bitsOn)rt::ClearShoulderGirdleBits(s.upperBits,uint32_t(sk.n),2,[&](uint32_t b){return file.BoneName(b);});
                std::vector<unsigned char> records;Records(game,records);rt::Cycle cycle;
                const bool captured=cw>0.0f && rt::Capture(prepared,s,records.data(),sk.n,true,cycle);
                integ::BlendIntoRecords(records.data(),uint32_t(sk.n),s,1.0f);
                bool accepted=false;
                if(cw>0.0f)
                {
                    native_reload_contacts::Result res;
                    accepted=captured && rt::Finish(prepared,s,records.data(),sk.n,true,cycle,res,cw,rt::kAuthoredContactMinReach,&state,
                                                   int64_t(k+1)*tick,tps) && res.ok;
                }
                ov::Pose o;
                if(!rt::Read(prepared,records.data(),o)){r.ok=false;continue;}
                ++r.frames;r.accepted+=accepted ? 1 : 0;
                r.minWidth=std::min(r.minWidth,width(o));r.minGap=std::min(r.minGap,gap(o));
                if(havePrev)
                {
                    const float cs=std::max(gik::Len(native_reload::Model(sk,o,lc).t-native_reload::Model(sk,prev,lc).t),
                                            gik::Len(native_reload::Model(sk,o,rc).t-native_reload::Model(sk,prev,rc).t));
                    const float ss=std::max(gik::Len(inSpine3(o,rig.left.upper)-inSpine3(prev,rig.left.upper)),
                                            gik::Len(inSpine3(o,rig.right.upper)-inSpine3(prev,rig.right.upper)));
                    r.clavStep=std::max(r.clavStep,cs);r.socketStep=std::max(r.socketStep,ss);
                    if(k==0)r.startStep=ss;
                    if(f.e>0.0f || f.w>0.0f)r.endStep=ss; // last frame with a reload weight
                }
                if(int(k)>=40 && int(k)<len-18 && f.w>=0.999f)
                {
                    r.handErr=std::max(r.handErr,gik::Len(inWeapon(o,rig.left.hand)-inWeapon(*walkGame,rig.left.hand)));
                    r.maxRy=std::max(r.maxRy,elbowRy(o));
                }
                prev=o;havePrev=true;
            }
            return r;};
        const float wOwn=width(*base);
        const auto report=[&](const char* name,const W& r){
            std::printf("  whole frame %s: frames %d accepted %d | width own %.1fcm min %.1fcm, clavicle gap min %.1fcm | clavicle step %.2fmm, "
                        "socket step max %.2fcm (start %.2f, end %.2f) | LH-in-weapon err %.2fcm | elbow ry max %.1fdeg\n",
                        name,r.frames,r.accepted,wOwn*100.0f,r.minWidth*100.0f,r.minGap*100.0f,r.clavStep*1000.0f,r.socketStep*100.0f,
                        r.startStep*100.0f,r.endStep*100.0f,r.handErr*100.0f,r.maxRy);};
        const W wf=wholeRun(former,false,0.0f,true);report("former (begun walking, uncorrected)",wf);
        Check(wf.ok && wf.minWidth<wOwn-0.15f && wf.minGap<0.0f,"fixture: the former reload begun walking shows the game's narrow, crossed girdle");
        const W wk=wholeRun(walk,false,0.0f,false);report("edge keep (game clip)",wk);
        Check(wk.ok && wk.accepted>=60,"edge keep: the contact solve accepts the reload begun walking");
        Check(wk.minWidth>=0.31f && wk.minWidth>=wOwn-0.01f && wk.minGap>0.0f,
              "edge keep: V's own shoulder width on every frame (Female >= 31 cm), the clavicles never cross");
        Check(wk.clavStep<0.001f && wk.startStep<0.015f && wk.endStep<0.015f && wk.socketStep<0.015f,
              "edge keep: no clavicle step, no shoulder pop at the start, inside or at the end of the reload");
        Check(wk.handErr<0.015f,"edge keep: the left hand keeps the game's hand-to-weapon relation (hands at the weapon)");
        const W ws=wholeRun(shot,false,0.0f,false);report("edge keep, shot in the exit",ws);
        Check(ws.ok && ws.minWidth>=wOwn-0.01f && ws.clavStep<0.001f && ws.socketStep<0.015f,
              "girdle hold: a shot in the exit of a reload begun walking does not pop the girdle");
        const W wa=wholeRun(aWalk,true,50.0f,false);
        report("edge keep (authored clip, forearm 50 deg off its hinge)",wa);
        Check(wa.ok && wa.minWidth>=wOwn-0.01f && wa.minGap>0.0f && wa.clavStep<0.001f && wa.socketStep<0.015f,
              "edge keep (authored): V's own shoulder width, no clavicle step or shoulder pop");
        Check(elbowRy(*girdle(0.14f,0.045f,50.0f))>45.0f,"fixture: the authored left forearm is 50 deg off its hinge (former output)");
        Check(wa.maxRy<5.0f,"edge keep (authored): the solve keeps the left forearm on its hinge plane (elbow deviation < 5 deg)");
    }

    // 12) Placement guard inside a rifle reload (PlacementGate, kGirdlePlacementUngated). A run start lowers and leans the MM torso while
    //     the game's box-magazine hand stays low: carried in the chest frame, the native left hand ends up 14 - 23 cm under the MM hips and
    //     the guard shrinks the correction (partly, then to 0) for ~.3 s, then recovers over .2 s. The records carry the game's girdle on
    //     the masked channels (section 10's girdle). Every frame keeps V's own clavicle translation and arm bone translations, the clavicle
    //     turns at most by the reach step, the shoulder never pops, and the wrists stay under the bend limit of the arm limits.
    //     Mutation 26 (girdle restored with the shrunk weight) shows the game's girdle on the guarded frames.
    {
        const int lc=sk.parent[rig.left.upper],rc=sk.parent[rig.right.upper];
        auto game=std::make_unique<ov::Pose>(*base);
        {
            const auto lm=native_reload::Model(sk,*base,lc).t,rm=native_reload::Model(sk,*base,rc).t;
            const math::Vec3 in=(rm-lm)*(1.0f/gik::Len(rm-lm));
            for(int side=0;side<2;++side)
            {
                const int c=side ? rc : lc;
                auto m=native_reload::Model(sk,*game,c);
                m.t=m.t+in*(side ? -0.08f : 0.08f)+(side ? math::Vec3{} : math::Vec3{0.0f,0.04f,0.045f});
                game->b[c]=native_reload::Relative(native_reload::Model(sk,*game,sk.parent[c]),m);
            }
            game->b[rig.left.upper].t=game->b[rig.left.upper].t*1.08f;
            // the game's left hand low at the box magazine: 25 cm in front of the hips, 5 cm above them
            const auto hm=native_reload::Model(sk,*base,hips).t;
            gik::Params wide;gik::Result solved;
            if(!gik::SolveArm(sk,*game,rig.left,hm+math::Vec3{0.05f,0.25f,0.05f},nullptr,1.0f,wide,solved))return 2;
        }
        // V's MM pose leaning forward by `deg` at the hips (the pose the snapshot hands in; the game's body stays upright)
        const auto leaned=[&](float deg){
            auto out=std::make_unique<ov::Pose>(*base);
            for(const float sign:{1.0f,-1.0f})
            {
                auto desired=native_reload::Model(sk,*base,hips);
                desired.r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{1.0f,0.0f,0.0f},sign*deg*math::kPi/180.0f),desired.r));
                const auto parentModel=sk.parent[hips]>=0 ? native_reload::Model(sk,*base,sk.parent[hips]) : ov::Xf{};
                out->b[hips].r=native_reload::Relative(parentModel,desired).r;
                if(deg<=0.0f || native_reload::Model(sk,*out,spine3).t.y>native_reload::Model(sk,*base,spine3).t.y)break; // forward = +y
            }
            return out;
        };
        const int64_t tps=10000000,step=tps/60;
        const int frames=60;
        const auto angle=[](math::Quat a,math::Quat b){
            return 2.0f*std::acos(std::clamp(std::fabs(math::QuatDot(math::QuatNormalize(a),math::QuatNormalize(b))),0.0f,1.0f))*180.0f/math::kPi;};
        struct Sample{bool ok;float gate,clavT,armT,clavDeg,socketStep,width,bendL,bendR;bool written=false;float pronL=0.0f,twistL=0.0f;};
        // section 14: optional diagnostic record per frame, an entry ramp of the weight and one HoldOnly frame; the written records are kept
        struct TraceRun{bool trace=false,ramp=false;int holdFrame=-1;std::vector<std::vector<unsigned char>> written;std::vector<reload_trace::Record> recs;};
        const auto run=[&](float leanDeg,std::vector<Sample>& out,TraceRun* opt=nullptr){
            rt::Continuity state;out.clear();
            math::Vec3 prevSocket{};bool havePrev=false;
            for(int k=0;k<frames;++k)
            {
                // lean: 0 for 10 frames, ramp over 4, held 16, back over 4, 0 for the rest
                const float e=k<10 ? 0.0f : k<14 ? float(k-9)/4.0f : k<30 ? 1.0f : k<34 ? float(33-k)/4.0f : 0.0f;
                const auto mm=leaned(leanDeg*e);
                const float weight=(opt && opt->ramp) ? std::min(1.0f,float(k)*0.25f) : 1.0f;
                integ::PoseSnapshot s=snap;s.traceOverlay=int(ov::PoseOverlay::Rifle);s.sequence=uint64_t(3000+k);s.reloadContactsWeight=weight;
                s.upperScale=0.0f;s.lowerScale=1.0f;
                for(int b=0;b<sk.n;++b)
                {
                    const auto& x=mm->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(s.pose+size_t(b)*7,row,sizeof(row));
                }
                std::vector<unsigned char> records;Records(*game,records);rt::Cycle cycle;
                Sample m{};
                const bool captured=rt::Capture(prepared,s,records.data(),sk.n,true,cycle);
                integ::BlendIntoRecords(records.data(),uint32_t(sk.n),s,1.0f);
                native_reload_contacts::Result r;
                ov::Pose o;
                reload_trace::Record rec;reload_trace::Clear(rec);
                state.trace=(opt && opt->trace) ? &rec : nullptr;
                if(opt && k==opt->holdFrame)
                    m.written=rt::HoldOnly(prepared,s,records.data(),sk.n,true,state,int64_t(k+1)*step,tps);
                else
                    m.written=captured && rt::Finish(prepared,s,records.data(),sk.n,true,cycle,r,weight,rt::kAuthoredContactMinReach,&state,
                                                     int64_t(k+1)*step,tps);
                state.trace=nullptr;
                if(opt){opt->written.push_back(records);opt->recs.push_back(rec);}
                m.ok=m.written && r.ok;
                m.gate=state.placement.value;
                if(!rt::Read(prepared,records.data(),o)){out.push_back(Sample{false,-1.0f,9.0f,9.0f,180.0f,9.0f,0.0f,180.0f,180.0f});continue;}
                m.clavT=std::max(gik::Len(o.b[lc].t-base->b[lc].t),gik::Len(o.b[rc].t-base->b[rc].t));
                m.armT=0.0f;
                for(const int b:{rig.left.upper,rig.left.lower,rig.left.hand,rig.right.upper,rig.right.lower,rig.right.hand})
                    m.armT=std::max(m.armT,gik::Len(o.b[b].t-base->b[b].t));
                m.clavDeg=std::max(angle(o.b[lc].r,base->b[lc].r),angle(o.b[rc].r,base->b[rc].r));
                const math::Vec3 socket=inSpine3(o,rig.left.upper);
                m.socketStep=havePrev ? gik::Len(socket-prevSocket) : 0.0f;prevSocket=socket;havePrev=true;
                m.width=gik::Len(native_reload::Model(sk,o,rig.left.upper).t-native_reload::Model(sk,o,rig.right.upper).t);
                m.bendL=rt::WristBendDeg(o.b[rig.left.hand].r);m.bendR=rt::WristBendDeg(o.b[rig.right.hand].r);
                {const auto an=rt::MeasureArm(o,rig.left);m.pronL=std::fabs(an.pronationDeg);m.twistL=std::fabs(an.upperTwistDeg);}
                out.push_back(m);
            }
        };
        const float wOwn=gik::Len(native_reload::Model(sk,*base,rig.left.upper).t-native_reload::Model(sk,*base,rig.right.upper).t);
        // the smallest lean (5 deg steps) at which the guard cuts the correction to 0 on the held frames
        float lean=-1.0f;std::vector<Sample> v;
        for(float d=10.0f;d<=80.0f;d+=5.0f)
        {
            run(d,v);
            float minGate=1.0f;for(int k=14;k<30;++k)minGate=std::min(minGate,v[size_t(k)].gate);
            if(minGate<=0.0f){lean=d;break;}
        }
        Check(lean>0.0f,"fixture: a forward lean of the MM torso brings the carried game hand under the MM hips (placement guard at 0)");
        if(lean>0.0f)
        {
            int partial=0,cut=0,accepted=0;
            float clavT=0,armT=0,clavDeg=0,socketStep=0,minWidth=9,bendL=0,bendR=0,bendL0=0,bendR0=0,maxRise=0;
            for(int k=0;k<frames;++k)
            {
                const auto& m=v[size_t(k)];
                if(m.gate>0.0f && m.gate<1.0f)++partial;
                if(m.gate<=0.0f)++cut;
                accepted+=m.ok ? 1 : 0;
                if(k>0)maxRise=std::max(maxRise,m.gate-v[size_t(k-1)].gate);
                clavT=std::max(clavT,m.clavT);armT=std::max(armT,m.armT);clavDeg=std::max(clavDeg,m.clavDeg);
                socketStep=std::max(socketStep,m.socketStep);minWidth=std::min(minWidth,m.width);
                if(k<10){bendL0=std::max(bendL0,m.bendL);bendR0=std::max(bendR0,m.bendR);}
                else{bendL=std::max(bendL,m.bendL);bendR=std::max(bendR,m.bendR);}
            }
            std::printf("  placement guard (rifle, lean %.0f deg): gate partial %d, zero %d frames, accepted %d/%d, max gate rise %.3f/frame | "
                        "own width %.1fcm, min width %.1fcm | clavicle translation %.2fmm, arm bone translation %.2fmm, clavicle turn %.1fdeg, "
                        "socket step %.2fcm/frame | wrist bend L %.1f R %.1f deg (before the lean %.1f / %.1f)\n",
                        lean,partial,cut,accepted,frames,maxRise,wOwn*100.0f,minWidth*100.0f,clavT*1000.0f,armT*1000.0f,clavDeg,socketStep*100.0f,
                        bendL,bendR,bendL0,bendR0);
            Check(partial>=1 && cut>=4 && maxRise<=(1.0f/60.0f)/0.2f+1.0e-4f,
                  "fixture: the placement guard shrinks the correction partly, then to 0, and recovers at its .2 s rate");
            Check(clavT<0.001f && armT<0.001f,"placement guard (rifle): V's own clavicle and arm bone translations on every frame");
            Check(clavDeg<=20.5f && minWidth>=wOwn-0.005f && socketStep<0.015f,
                  "placement guard (rifle): V's shoulder width, the clavicle turns at most by the reach step, no shoulder pop");
            // On the frames the guard cuts to 0 the arms are the game's arm rotations (RejectHold replay faded out): the game's own wrist.
            const float gameL=rt::WristBendDeg(game->b[rig.left.hand].r),gameR=rt::WristBendDeg(game->b[rig.right.hand].r);
            std::printf("  placement guard: the game's own wrist bend L %.1f R %.1f deg\n",gameL,gameR);
            Check(bendL<=std::max({rt::kReloadWristBendLimitDeg,bendL0,gameL})+0.5f && bendR<=std::max({rt::kReloadWristBendLimitDeg,bendR0,gameR})+0.5f,
                  "placement guard (rifle): no wrist bends past the limit, the unguarded frames or the game's own wrist");
            // 13e. the final anatomy stage on every written frame of this run (full weight): the left arm within its limits
            {
                const auto own=rt::MeasureArm(*game,rig.left);
                int written=0;float wB=0,wP=0,wT=0;
                for(const auto& m:v)if(m.written){++written;wB=std::max(wB,m.bendL);wP=std::max(wP,m.pronL);wT=std::max(wT,m.twistL);}
                std::printf("  final left arm limits: written %d/%d frames, max bend %.1f pronation %.1f upper twist %.1f deg (game's own %.1f / %.1f / %.1f)\n",
                            written,frames,wB,wP,wT,own.bendDeg,std::fabs(own.pronationDeg),std::fabs(own.upperTwistDeg));
                Check(written>=40 && wB<=rt::kReloadWristBendLimitDeg+0.05f && wP<=rt::kReloadPronationLimitDeg+0.5f && wT<=rt::kReloadUpperArmTwistCapDeg+0.05f,
                      "final left arm limits (rifle): every written frame keeps the left wrist bend, pronation and upper-arm twist within the limits");
            }
            // 14. Diagnostic record (reload_trace): the same production run (entry ramp of the weight from 0, the placement guard partly and
            //     fully engaged, rejects with replays, one HoldOnly frame) with the record on and off: the written records are identical byte
            //     for byte on every frame, and the record shows the stages the run went through.
            {
                TraceRun off,on;off.ramp=on.ramp=true;off.holdFrame=on.holdFrame=45;on.trace=true;
                std::vector<Sample> vOff,vOn;
                const auto t0=std::chrono::steady_clock::now();
                for(int rep=0;rep<5;++rep){off.written.clear();off.recs.clear();run(lean,vOff,&off);}
                const auto t1=std::chrono::steady_clock::now();
                for(int rep=0;rep<5;++rep){on.written.clear();on.recs.clear();run(lean,vOn,&on);}
                const auto t2=std::chrono::steady_clock::now();
                bool same=off.written.size()==size_t(frames) && on.written.size()==size_t(frames);
                int differing=0;
                for(size_t k=0;same && k<off.written.size();++k)
                    if(off.written[k].size()!=on.written[k].size() ||
                       std::memcmp(off.written[k].data(),on.written[k].data(),off.written[k].size())!=0)++differing;
                int exits[16]{},replays=0,holds=0,finalLimited=0,gateMatch=0,gateFrames=0,bendMatch=0,bendFrames=0,elbowFinite=0,acceptedRecs=0;
                for(int k=0;k<frames && k<int(on.recs.size());++k)
                {
                    const auto& rec=on.recs[size_t(k)];
                    ++exits[rec.exit&15];
                    if(rec.flags&reload_trace::kFlagReplayWritten)++replays;
                    if(rec.path==reload_trace::kPathHold)++holds;
                    if(rec.flags&reload_trace::kFlagFinalLimit)++finalLimited;
                    if(std::isfinite(rec.placementGate)){++gateFrames;if(rec.placementGate==vOn[size_t(k)].gate)++gateMatch;}
                    if(vOn[size_t(k)].written && std::isfinite(rec.anat[reload_trace::kAnatFinal][2]))
                    {++bendFrames;if(std::fabs(rec.anat[reload_trace::kAnatFinal][2]-vOn[size_t(k)].bendL)<1.0e-3f)++bendMatch;}
                    if(rec.exit==reload_trace::kExitAccepted)
                    {
                        ++acceptedRecs;
                        bool fin=true;
                        for(int i=0;i<3;++i)fin=fin && std::isfinite(rec.elbowSolve[i]) && std::isfinite(rec.elbowSwivel[i]) && std::isfinite(rec.elbowOut[i]) &&
                                                  std::isfinite(rec.shoulder[i]) && std::isfinite(rec.guide[i]) && std::isfinite(rec.handTarget[i]);
                        if(fin)++elbowFinite;
                    }
                }
                const double msOff=std::chrono::duration<double,std::milli>(t1-t0).count()/(5.0*frames);
                const double msOn=std::chrono::duration<double,std::milli>(t2-t1).count()/(5.0*frames);
                std::printf("  record: %d/%d frames differ (record on vs off) | exits: weight0 %d, placement out %d, solve reject %d, acceptedRecs %d | "
                            "replays %d, hold frames %d, final-limit frames %d | gate %d/%d, final bend %d/%d match | per frame %.3f ms off, %.3f ms on\n",
                            differing,frames,exits[reload_trace::kExitWeightZero],exits[reload_trace::kExitPlacementOut],exits[reload_trace::kExitSolveReject],
                            acceptedRecs,replays,holds,finalLimited,gateMatch,gateFrames,bendMatch,bendFrames,msOff,msOn);
                Check(same && differing==0,"record: the written records are identical with the record on and off on every frame");
                Check(exits[reload_trace::kExitWeightZero]>=1 && exits[reload_trace::kExitPlacementOut]>=1 && acceptedRecs>=20 && holds==1 &&
                      on.recs[45].path==reload_trace::kPathHold && on.recs[0].exit==reload_trace::kExitWeightZero,
                      "record: the weight-0 frame, the placement guard cut, the acceptedRecs frames and the HoldOnly frame are told apart");
                Check(replays>=1 && gateFrames>=20 && gateMatch==gateFrames && bendFrames>=20 && bendMatch==bendFrames && elbowFinite==acceptedRecs,
                      "record: replays flagged, placement gate and final left wrist bend equal the production values, accepted frames carry the left-arm solve");
            }
        }
    }

    // 13. Final anatomy stage of the left arm (LimitArmFinal). (a) under the limits: untouched bit for bit; (b) an arm past all three limits at
    //     full weight: every value at its limit; (c) half weight: bound = limit + .5 * (game's own - limit); (d) a bend sweep 60 -> 150 deg
    //     (the one-frame flips of the folded support arm): output at most the limit, the output never steps more than the input.
    //     13e (inside section 12): every written frame of the production run. Mutation 27 drops the stage.
    {
        const auto& L=rig.left;
        ov::Pose a=*base;
        a.b[L.upper].r=math::QuatNormalize(rt::kUpperArmRestLocal);
        a.b[L.lower].r=ov::AxisAngleQuat(math::Vec3{0.0f,0.0f,1.0f},60.0f);
        a.b[L.hand].r=ov::AxisAngleQuat(math::Vec3{0.0f,1.0f,0.0f},30.0f);
        const rt::ArmAnatomy m0=rt::MeasureArm(a,L);
        ov::Pose a0=a;
        const auto r0=rt::LimitArmFinal(a0,L,m0,1.0f);
        Check(m0.ok && m0.bendDeg<rt::kReloadWristBendLimitDeg && !r0.applied &&
              std::memcmp(&a0.b[L.upper].r,&a.b[L.upper].r,sizeof(math::Quat))==0 && std::memcmp(&a0.b[L.hand].r,&a.b[L.hand].r,sizeof(math::Quat))==0,
              "final left arm limits: an arm under the limits is untouched bit for bit");
        ov::Pose bad=a;
        bad.b[L.upper].r=math::QuatNormalize(math::QuatMul(rt::kUpperArmRestLocal,ov::AxisAngleQuat(math::Vec3{1.0f,0.0f,0.0f},170.0f)));
        bad.b[L.hand].r=math::QuatNormalize(math::QuatMul(ov::AxisAngleQuat(math::Vec3{0.0f,1.0f,0.0f},147.0f),ov::AxisAngleQuat(math::Vec3{1.0f,0.0f,0.0f},150.0f)));
        const rt::ArmAnatomy mb=rt::MeasureArm(bad,L);
        ov::Pose b1=bad;
        const auto r1=rt::LimitArmFinal(b1,L,m0,1.0f);
        const rt::ArmAnatomy m1=rt::MeasureArm(b1,L);
        std::printf("  final left arm limits: before bend %.1f pronation %.1f twist %.1f -> after %.1f / %.1f / %.1f deg\n",
                    mb.bendDeg,mb.pronationDeg,mb.upperTwistDeg,m1.bendDeg,m1.pronationDeg,m1.upperTwistDeg);
        Check(mb.bendDeg>140.0f && std::fabs(mb.upperTwistDeg)>160.0f,"fixture: the bad arm is past the wrist and upper-arm limits");
        Check(r1.applied && m1.bendDeg<=rt::kReloadWristBendLimitDeg+0.05f && std::fabs(m1.pronationDeg)<=rt::kReloadPronationLimitDeg+0.5f &&
              std::fabs(m1.upperTwistDeg)<=rt::kReloadUpperArmTwistCapDeg+0.05f,
              "final left arm limits: bend, pronation and upper-arm twist brought to their limits at full weight");
        // half weight against a game arm bent to 120 deg: bound 75 + .5 * 45 = 97.5
        ov::Pose g120=a;
        g120.b[L.hand].r=ov::AxisAngleQuat(math::Vec3{0.0f,1.0f,0.0f},120.0f);
        const rt::ArmAnatomy mg=rt::MeasureArm(g120,L);
        ov::Pose b2=a;b2.b[L.hand].r=ov::AxisAngleQuat(math::Vec3{0.0f,1.0f,0.0f},147.0f);
        rt::LimitArmFinal(b2,L,mg,0.5f);
        const float half=rt::WristBendDeg(b2.b[L.hand].r),expect=rt::kReloadWristBendLimitDeg+0.5f*(mg.bendDeg-rt::kReloadWristBendLimitDeg);
        std::printf("  final left arm limits: half weight, game's own bend %.1f -> bound %.1f, output %.1f deg\n",mg.bendDeg,expect,half);
        Check(std::fabs(half-expect)<0.1f,"final left arm limits: on the weight ramp the bound lies between the limit and the game's own value");
        float prevIn=0,prevOut=0,maxOut=0,maxStepRatio=0;bool first=true;
        for(float d=60.0f;d<=150.0f;d+=5.0f)
        {
            ov::Pose x=a;x.b[L.hand].r=ov::AxisAngleQuat(math::Vec3{0.0f,1.0f,0.0f},d);
            rt::LimitArmFinal(x,L,m0,1.0f);
            const float o=rt::WristBendDeg(x.b[L.hand].r);
            maxOut=std::max(maxOut,o);
            if(!first)maxStepRatio=std::max(maxStepRatio,std::fabs(o-prevOut)/std::max(1e-3f,std::fabs(d-prevIn)));
            prevIn=d;prevOut=o;first=false;
        }
        Check(maxOut<=rt::kReloadWristBendLimitDeg+0.05f && maxStepRatio<=1.0f+1e-3f,
              "final left arm limits: a bend sweep to 150 deg stays at the limit, the output never steps more than the input");
        // the upper arm twist passes 180 deg (160 -> 200 = -160): with the side memory the output holds +100, no 200 deg roll
        {
            rt::FinalLimitSide side;
            float maxJump=0.0f,prevTw=0.0f,minTw=999.0f,maxTw=-999.0f;bool have=false;
            for(float d=150.0f;d<=210.0f;d+=5.0f)
            {
                ov::Pose x=a;
                x.b[L.upper].r=math::QuatNormalize(math::QuatMul(rt::kUpperArmRestLocal,ov::AxisAngleQuat(math::Vec3{1.0f,0.0f,0.0f},d)));
                rt::LimitArmFinal(x,L,m0,1.0f,&side);
                const float tw=rt::MeasureArm(x,L).upperTwistDeg;
                if(have)maxJump=std::max(maxJump,std::fabs(tw-prevTw));
                minTw=std::min(minTw,tw);maxTw=std::max(maxTw,tw);prevTw=tw;have=true;
            }
            std::printf("  final left arm limits: twist 150 -> 210 deg: output %.1f .. %.1f deg, largest step %.1f deg\n",minTw,maxTw,maxJump);
            Check(maxJump<=0.1f && minTw>=rt::kReloadUpperArmTwistCapDeg-0.1f && maxTw<=rt::kReloadUpperArmTwistCapDeg+0.1f,
                  "final left arm limits: a twist that wraps through 180 deg keeps the output on its side (no flip to -100)");
        }
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

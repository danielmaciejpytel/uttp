// Offline test: the reload contact targets when the NATIVE body has diverged from the MM body.
// Recorded 21.10 01:56 (after a jump / slide the native graph left the player squatting): native hips .34m against MM .95m,
// the contact targets are native MODEL positions, so the left hand ended under the MM hips in 27-49 of 89 frames of every
// standing reload and the weapon inside the head in others (weapon-head 1.4-9cm).
// The Prepared config is built by the production builder reload_contacts_runtime::FillPrepared from the production map:
// Root and Trajectory are MAPPED (cold readMap, firstBone 0), no identity roots, the snapshot map starts at bone 2.
// Usage: ReloadDivergedNativeTest <woman.utppov>
// Mutation 4 (UTPP_RELOAD_CONTACT_MUTATION=4): no divergence handling (no torso transport, no correction fade; the shoulder anchor stays
// on, so only fixture B3 - a pelvis collapse WITH a forward lean - kills it). 5: no placement guard. 7: placement factor not
// time-limited. 13: head clearance with a fixed band. 14: HoldOnly without the snapshot validation. 16: a denial that finds
// the state busy leaves no reset request. 19: no correction fade around the anchor <-> transport switch (kills sweep C2). Every mutation build must
// report an assertion FAIL (ctest PASS_REGULAR_EXPRESSION "FAIL ").
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
    // Production topology: gameIndex = cold readMap (every bone, Root and Trajectory included).
    std::vector<int32_t> coldMap(size_t(sk.n));for(int b=0;b<sk.n;++b)coldMap[size_t(b)]=b;
    reload_contacts_runtime::Prepared prepared;
    if(!reload_contacts_runtime::FillPrepared(prepared,123,uint32_t(sk.n),coldMap.data(),
        [&](uint32_t i){return int(sk.parent[i]);},[&](uint32_t i){return file.BoneName(int(i));}))return 2;
    const auto rig=prepared.arms;const int head=prepared.head,hips=prepared.hips;
    if(head<0||hips<=1)return 2;
    Check(prepared.gameIndex[0]==0&&prepared.gameIndex[1]==1,"production map: Root and Trajectory are mapped (no identity roots)");
    {
        const int shoulder=sk.parent[rig.left.upper],spine3=sk.parent[shoulder];
        auto noHips=prepared;noHips.hips=-1;
        Check(reload_contacts_runtime::TopBoneOf(prepared,spine3)==hips&&reload_contacts_runtime::TopBoneOf(noHips,spine3)==hips,
              "top bone of the pelvis chain is Hips (by name and by the parent-chain fallback), not Root");
    }
    const int index=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Aiming,false,false);if(index<0)return 2;
    auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
    integ::PoseSnapshot snap;snap.rig=123;snap.sequence=11;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(ov::PoseOverlay::Pistol2H);
    snap.reloadAuthoredClip=true;snap.upperScale=0;snap.traceAction=2;
    const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
    std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
    for(int b=0;b<sk.n;++b)
    {
        snap.gameIndex[b]=b>=2 ? b : -1; // g_map: the snapshot omits Root / Trajectory
        const auto& x=base->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));
    }
    auto finishWith=[&](const reload_contacts_runtime::Prepared& config,const ov::Pose& native,float weight,ov::Pose& out,
                        native_reload_contacts::Result& result,reload_contacts_runtime::Continuity* state=nullptr,int64_t ticks=0)
    {
        std::vector<unsigned char> records;Records(native,records);reload_contacts_runtime::Cycle cycle;
        const bool captured=reload_contacts_runtime::Capture(config,snap,records.data(),sk.n,true,cycle);
        Records(*base,records);
        if(!captured)return false;
        const bool ok=reload_contacts_runtime::Finish(config,snap,records.data(),sk.n,true,cycle,result,weight,
                                                      reload_contacts_runtime::kAuthoredContactMinReach,state,ticks,10000000);
        const bool read=reload_contacts_runtime::Read(config,records.data(),out); // baseline when nothing was written
        return ok&&read;
    };
    auto finish=[&](const ov::Pose& native,float weight,ov::Pose& out,native_reload_contacts::Result& result)
    { return finishWith(prepared,native,weight,out,result); };
    const auto handOverHips=[&](const ov::Pose& p){return native_reload::Model(sk,p,rig.left.hand).t.z-native_reload::Model(sk,*base,hips).t.z;};
    const auto weaponHead=[&](const ov::Pose& p){return gik::Len(native_reload::Model(sk,p,rig.weaponRight).t-native_reload::Model(sk,*base,head).t);};
    auto collapsedBy=[&](const ov::Pose& source,float offset){
        auto native=std::make_unique<ov::Pose>(source);
        auto desired=native_reload::Model(sk,*base,hips);desired.t.z-=offset;
        const auto parentModel=sk.parent[hips]>=0 ? native_reload::Model(sk,*base,sk.parent[hips]) : ov::Xf{};
        native->b[hips].t=native_reload::Relative(parentModel,desired).t;
        return native;
    };
    // A native pose whose LEFT hand lies beyond the reach of the arm for ANY placement of the body: the hand bone is pushed along the
    // forearm until it is ~1m from the shoulder. These fixtures used a pelvis collapsed by .33m ("absolute targets out of reach"); the
    // shoulder anchor carries such a collapse rigidly onto the MM body, so that native is reachable now and no longer rejected.
    auto unreachableOf=[&](const ov::Pose& source){
        auto native=std::make_unique<ov::Pose>(source);
        native->b[rig.left.hand].t=native->b[rig.left.hand].t*4.0f;
        return native;
    };
    // A collapsed native pelvis that also LEANS forward about the model X axis (jump / slide leftovers: the native Spine3 is pitched, the MM
    // one is not). A pure translation of the pelvis is carried onto the MM body by the shoulder anchor alone; only the torso transport (the
    // native Spine3 FRAME onto the MM one) reproduces the hands and the weapon of a leaning native body.
    auto leaningCollapse=[&](const ov::Pose& source,float offset,float leanDeg){
        auto native=std::make_unique<ov::Pose>(source);
        auto desired=native_reload::Model(sk,*base,hips);desired.t.z-=offset;
        desired.r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{1.0f,0.0f,0.0f},leanDeg*math::kPi/180.0f),desired.r));
        const auto parentModel=sk.parent[hips]>=0 ? native_reload::Model(sk,*base,sk.parent[hips]) : ov::Xf{};
        const auto local=native_reload::Relative(parentModel,desired);
        native->b[hips].t=local.t;native->b[hips].r=local.r;
        return native;
    };
    const int spine3Bone=sk.parent[sk.parent[rig.left.upper]];
    // Position of `bone` in the frame of the Spine3 of `pose`.
    const auto inSpine3=[&](const ov::Pose& pose,int bone){
        const auto frame=native_reload::Model(sk,pose,spine3Bone);
        return math::QuatRotate(math::QuatInv(frame.r),native_reload::Model(sk,pose,bone).t-frame.t);
    };
    const float baseLh=handOverHips(*base);
    std::printf("fixture: baseline left hand %.3fm over hips, weapon-head %.3fm\n",baseLh,weaponHead(*base));

    // A) Native body identical to MM: nothing to transport, contacts reproduce the baseline hands.
    {
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finish(*base,1.0f,out,r);
        Check(ok&&gik::Len(native_reload::Model(sk,out,rig.left.hand).t-native_reload::Model(sk,*base,rig.left.hand).t)<1e-4f,"native == MM: hands stay on the baseline");
    }
    // B) Native pelvis collapsed .65m (squatting native body), MM standing.
    {
        auto native=collapsedBy(*base,0.65f);
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finish(*native,1.0f,out,r);
        const float lh=ok?handOverHips(out):-9;
        std::printf("  collapsed native: accepted=%d reject=%d left hand %.3fm over the MM hips\n",ok,int(r.reject),lh);
        Check(ok&&lh>baseLh-0.03f,"collapsed native body: left hand stays at the baseline height over the MM hips (production map; a pure translation: the shoulder anchor and the transport agree, B3 separates them)");
        Check(ok&&weaponHead(out)>weaponHead(*base)-0.03f,"collapsed native body: weapon keeps its distance from the head");
        auto noHips=prepared;noHips.hips=-1;ov::Pose out2;native_reload_contacts::Result r2;
        const bool ok2=finishWith(noHips,*native,1.0f,out2,r2);
        Check(ok2&&gik::Len(native_reload::Model(sk,out2,rig.left.hand).t-native_reload::Model(sk,out,rig.left.hand).t)<1e-4f,
              "Hips found by the parent-chain fallback gives the same result as the named Hips");
    }
    // B2) Same collapse, native left hand 8cm forward / 6cm up relative to the torso: the transported correction follows it.
    {
        auto native=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*native,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        const math::Vec3 expectedHand=native_reload::Model(sk,*native,rig.left.hand).t;
        native=collapsedBy(*native,0.65f);
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finish(*native,1.0f,out,r);
        const float miss=ok?gik::Len(native_reload::Model(sk,out,rig.left.hand).t-expectedHand):9;
        std::printf("  collapsed native with a displaced left hand: accepted=%d reject=%d miss %.3fm\n",ok,int(r.reject),miss);
        Check(ok&&miss<0.02f,"collapsed native body: the left hand follows the native placement relative to the torso");
    }
    // B3) Collapsed .65m AND leaning 25deg forward, the left hand displaced as in B2: the hands and the weapon must keep their native
    //     place in the frame of the Spine3 (torso transport). The translation of the shoulder anchor alone is ~17cm off. Mutation 4.
    {
        auto native=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*native,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        native=leaningCollapse(*native,0.65f,25.0f);
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finish(*native,1.0f,out,r);
        const float leftMiss=ok ? gik::Len(inSpine3(out,rig.left.hand)-inSpine3(*native,rig.left.hand)) : 9;
        const float weaponMiss=ok ? gik::Len(inSpine3(out,rig.weaponRight)-inSpine3(*native,rig.weaponRight)) : 9;
        std::printf("  collapsed and leaning native: accepted=%d reject=%d, in the Spine3 frame: left hand off by %.3fm, weapon off by %.3fm\n",ok,int(r.reject),leftMiss,weaponMiss);
        Check(ok&&leftMiss<0.02f&&weaponMiss<0.02f,"collapsed and forward-leaning native body: hands and weapon keep their native place in the Spine3 frame (torso transport)");
    }
    // C) Whole divergence range .10-.75m, one 1/60 s frame per 5mm step (a native body standing up / squatting at 0.3 m/s),
    //    through the production state (placement gate + reject hold, as FkHook passes them). A rejected frame replays the
    //    last accepted correction fading out, an accepted frame after it ramps in: the left-hand height may not move by
    //    more than 6cm between neighbouring frames anywhere in the range (the raw accept/reject switch moved it 20-30cm).
    //    The native pelvis also leans forward (25deg at .75m, proportional to the collapse): below .42m the shoulder anchor (a translation)
    //    and above it the torso transport (the Spine3 frame) put the hand ~10cm apart, so the fade of the correction weight and the latch
    //    around .36-.42m are what keeps the sweep continuous (mutation 19: no fade). The step is measured in model height over the hips and
    //    in 3D in the frame of the MM Spine3.
    {
        auto native=leaningCollapse(*base,0.41f,25.0f*0.41f/0.75f);
        ov::Pose target,anchored,transported;
        ov::Blend(*base,*native,1.0f,target);anchored=target;transported=target;
        reload_contacts_runtime::AnchorNativeTarget(prepared,*base,anchored);
        reload_contacts_runtime::TransportNativeTorso(prepared,*base,transported,1.0f);
        const float apart=gik::Len(native_reload::Model(sk,anchored,rig.left.hand).t-native_reload::Model(sk,transported,rig.left.hand).t);
        std::printf("  sweep fixture: anchor and transport put the left hand %.3fm apart at .41m\n",apart);
        Check(apart>0.06f,"fixture: with the forward lean the shoulder anchor and the torso transport differ by > 6cm (the sweep measures the switch)");
    }
    for(int direction=0;direction<2;++direction)
    {
        reload_contacts_runtime::Continuity state;
        float previous=-99,worst=0,worst3=0;math::Vec3 previous3{};int rejected=0,accepted=0;float worstAt=0;bool backToBaseline=true;
        for(int step=0;step<=130;++step)
        {
            const float offset=direction==0 ? 0.10f+0.005f*step : 0.75f-0.005f*step;
            auto native=leaningCollapse(*base,offset,25.0f*offset/0.75f);
            ov::Pose out;native_reload_contacts::Result r;
            const bool ok=finishWith(prepared,*native,1.0f,out,r,&state,3000000+int64_t(step)*166667);
            const float lh=ok ? handOverHips(out) : baseLh;
            const math::Vec3 lh3=inSpine3(ok ? out : *base,rig.left.hand);
            (r.ok?accepted:rejected)++;
            if(previous>-90&&std::fabs(lh-previous)>worst){worst=std::fabs(lh-previous);worstAt=offset;}
            if(previous>-90)worst3=std::max(worst3,gik::Len(lh3-previous3));
            previous=lh;previous3=lh3;
            if(direction==0&&offset>=0.62f&&(!ok||std::fabs(lh-baseLh)>0.03f))backToBaseline=false;
            if(direction==1&&offset>=0.62f&&step<20&&(!ok||std::fabs(lh-baseLh)>0.03f))backToBaseline=false;
        }
        std::printf("  band sweep %s: solver accepted %d rejected %d, largest left-hand step %.3fm (height) / %.3fm (3D in the Spine3 frame) at offset %.3f\n",
                    direction==0?"squatting":"standing up",accepted,rejected,worst,worst3,worstAt);
        Check(worst<0.06f&&worst3<0.06f,direction==0?"pelvis offset .10->.75m with a forward lean: left-hand steps stay below 6cm per frame"
                                                    :"pelvis offset .75->.10m with a forward lean: left-hand steps stay below 6cm per frame");
        Check(backToBaseline,"pelvis offset >= .62m: accepted and within 3cm of the baseline height");
    }
    // C2) Reject hold in isolation: a displaced left hand is accepted, the next frame is unreachable.
    {
        auto native=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*native,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        auto far=unreachableOf(*native); // hand out of reach: rejected
        reload_contacts_runtime::Continuity hold;
        ov::Pose first,second,third,later;native_reload_contacts::Result r1,r2,r3,r4;
        const int64_t t0=5000000;
        const bool ok1=finishWith(prepared,*native,1.0f,first,r1,&hold,t0);
        const bool ok2=finishWith(prepared,*far,1.0f,second,r2,&hold,t0+166667);
        const bool ok3=finishWith(prepared,*far,1.0f,third,r3,&hold,t0+166667*4);
        const bool ok4=finishWith(prepared,*far,1.0f,later,r4,&hold,t0+166667*10);
        const auto handShift=[&](const ov::Pose& p){return gik::Len(native_reload::Model(sk,p,rig.left.hand).t-baseHand);};
        const float s1=ok1?handShift(first):0,s2=ok2?handShift(second):0,s3=ok3?handShift(third):0,s4=ok4?handShift(later):9;
        std::printf("  reject hold: accepted frame shift %.3fm (solver ok=%d), rejected frames at +1/4/10 frames: %.3f / %.3f / %.3f (solver ok=%d %d %d)\n",
                    s1,r1.ok,s2,s3,s4,r2.ok,r3.ok,r4.ok);
        Check(r1.ok&&!r2.ok&&!r3.ok&&s1>0.05f,"fixture: one accepted frame with a displaced hand, then unreachable frames");
        Check(ok2&&s2>0.8f*s1&&s2<s1,"rejected frame: the last correction is still ~90% applied one frame later (no switch-off)");
        Check(ok3&&s3<s2&&s3>0.2f*s1,"rejected frames fade monotonically (about half after 4 frames)");
        Check(s4<0.005f,"after .12s of rejections the baseline is back");
        // A perspective denial resets the state (FkHook): the next rejected frame must not replay the old correction.
        {
            reload_contacts_runtime::Continuity denied;ov::Pose d1,d2;native_reload_contacts::Result e1,e2;
            finishWith(prepared,*native,1.0f,d1,e1,&denied,t0);
            denied.Reset();
            finishWith(prepared,*far,1.0f,d2,e2,&denied,t0+166667);
            Check(e1.ok&&!e2.ok&&handShift(d2)<0.005f,"state reset (perspective denial): a rejected frame afterwards shows the baseline, not the old delta");
            reload_contacts_runtime::Continuity kept;ov::Pose k1,k2;native_reload_contacts::Result f1,f2;
            finishWith(prepared,*native,1.0f,k1,f1,&kept,t0);
            finishWith(prepared,*far,1.0f,k2,f2,&kept,t0+166667);
            Check(handShift(k2)>0.02f,"without the reset the old correction is still replayed one frame later (what the denial reset prevents)");
        }
        {
            reload_contacts_runtime::ContinuityLock holder;
            reload_contacts_runtime::ContinuityLock rival;
            Check(holder&&!rival,"continuity lock: a second concurrent job does not get the state");
        }
        {
            reload_contacts_runtime::ContinuityLock again;
            Check(bool(again),"continuity lock: released when the job ends");
        }
        // ramp in again after a rejection
        reload_contacts_runtime::Continuity again;ov::Pose a1,a2,a3,a4;native_reload_contacts::Result q1,q2,q3,q4;
        finishWith(prepared,*native,1.0f,a1,q1,&again,t0);
        for(int i=1;i<=8;++i)finishWith(prepared,*far,1.0f,a2,q2,&again,t0+166667*i); // 8 rejected frames: faded to 0 ...
        finishWith(prepared,*native,1.0f,a3,q3,&again,t0+166667*9);  // ... accepted again: ramps in
        for(int i=10;i<=16;++i)finishWith(prepared,*native,1.0f,a4,q4,&again,t0+166667*i);
        const float r3s=q3.ok?handShift(a3):9,r4s=q4.ok?handShift(a4):0;
        std::printf("  ramp in: first accepted frame after the rejection %.3fm, 7 frames later %.3fm\n",r3s,r4s);
        Check(q3.ok&&q4.ok&&r3s<0.5f*r4s&&r4s>0.9f*s1,"accepted frame after a rejection ramps in instead of snapping to the full correction");
    }
    // D) Native weapon driven into the head while the native body is otherwise in place: the guard shrinks the correction.
    auto headNative=std::make_unique<ov::Pose>(*base);
    {
        const auto headPos=native_reload::Model(sk,*base,head).t;
        const math::Vec3 wanted=headPos+math::Vec3{0.0f,0.03f,0.0f};
        math::Vec3 target=native_reload::Model(sk,*headNative,rig.right.hand).t;
        gik::Params wide;wide.minReach=0.05f;gik::Result solved;
        bool ok0=true;
        for(int iteration=0;iteration<6&&ok0;++iteration) // move the HAND until the weapon bone sits at the wanted spot
        {
            ok0=gik::SolveArm(sk,*headNative,rig.right,target,nullptr,1.0f,wide,solved);
            target=target+(wanted-native_reload::Model(sk,*headNative,rig.weaponRight).t);
        }
        const float fixtureDistance=weaponHead(*headNative);
        Check(ok0&&fixtureDistance<0.06f,"fixture: native weapon is inside the head zone");
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finish(*headNative,1.0f,out,r);
        const float d=weaponHead(out);
        std::printf("  guarded result: written=%d weapon-head %.3fm\n",ok,d);
        Check(d>=0.06f,"weapon is kept at least 6cm from the head by the placement guard");
    }
    // E) The placement factor falls AT ONCE (the weapon must never be let nearer to the head than the raw factor allows) and
    //    recovers at most 1/.20s (a weapon leaving the head zone cannot snap the whole correction back on).
    {
        reload_contacts_runtime::PlacementGate unit;
        float v=unit.Step(1.0f,123,1000000,10000000);
        v=unit.Step(0.0f,123,1000000+166667,10000000);
        const bool fallsAtOnce=v==0.0f;
        bool riseOk=true;float prev=v;
        for(int i=2;i<=30;++i){v=unit.Step(1.0f,123,1000000+i*166667,10000000);riseOk=riseOk&&(v-prev)<=0.1667f/0.20f*0.1f+1e-4f;prev=v;}
        Check(fallsAtOnce&&riseOk&&v==1.0f,"placement gate: falls at once, recovers at most 1/.20s");
        reload_contacts_runtime::PlacementGate stale;stale.Step(1.0f,123,1000000,10000000);
        const float after=stale.Step(0.2f,123,1000000+5000000,10000000); // 0.5 s gap: new episode starts at the raw value
        reload_contacts_runtime::PlacementGate other;other.Step(1.0f,123,1000000,10000000);
        const float rigChange=other.Step(0.2f,456,1000000+100000,10000000);
        Check(std::fabs(after-0.2f)<1e-6f&&std::fabs(rigChange-0.2f)<1e-6f,"placement gate: stale gap and rig change start a new episode at the raw value");

        // Finish level: frame 1 has the weapon in the head zone (raw factor 0), frame 2 a harmless native pose with a
        // displaced left hand (raw factor 1). The gated correction of frame 2 is ramping in, the raw one is complete.
        auto displaced=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*displaced,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        reload_contacts_runtime::Continuity state;
        ov::Pose first,gated,raw;native_reload_contacts::Result r1,r2,r3;
        finishWith(prepared,*headNative,1.0f,first,r1,&state,2000000);
        finishWith(prepared,*displaced,1.0f,gated,r2,&state,2000000+166667);
        finishWith(prepared,*displaced,1.0f,raw,r3);
        const auto handShift=[&](const ov::Pose& q){return gik::Len(native_reload::Model(sk,q,rig.left.hand).t-baseHand);};
        const float gatedShift=handShift(gated),rawShift=handShift(raw);
        std::printf("  correction one frame after the weapon left the head zone: gated hand shift %.3fm, raw %.3fm\n",gatedShift,rawShift);
        Check(rawShift>0.05f&&gatedShift<0.3f*rawShift,"limited recovery: the correction ramps in after the weapon left the head zone");
    }
    // F) The faded replay of an OLD correction must not carry the weapon into the head zone (01:56 k4: 12.9 -> 3.5cm).
    {
        reload_contacts_runtime::Continuity held;
        int bones[8];const int c=reload_contacts_runtime::RejectHold::Chains(prepared,bones);
        auto unreachable=unreachableOf(*base); // hand out of reach: the solve is rejected
        auto& h=held.reject;
        h.valid=true;h.haveDelta=true;h.rig=123;h.lastTicks=7000000;h.weight=1.0f;h.amp=1.0f;h.n=c;
        for(int k=0;k<c;++k) // a recorded correction that puts the arms where the head-zone native pose has them
        {
            const int b=bones[k];h.bone[k]=b;
            h.dq[k]=math::QuatNormalize(math::QuatMul(headNative->b[b].r,math::QuatInv(base->b[b].r)));
            h.dt[k]=headNative->b[b].t-base->b[b].t;
        }
        ov::Pose out;native_reload_contacts::Result r;
        finishWith(prepared,*unreachable,1.0f,out,r,&held,7000000+166667);
        const float d=weaponHead(out);
        std::printf("  replay of an old correction on a rejected frame: weapon-head %.3fm (the recorded delta alone puts it at %.3fm)\n",d,weaponHead(*headNative));
        Check(!r.ok&&d>=0.06f,"fade-on-reject never replays the weapon into the head zone (output clearance)");
    }
    // Native right arm solved so that the WEAPON bone sits at `wanted` (a native weapon near the head).
    const auto weaponAt=[&](math::Vec3 wanted,std::unique_ptr<ov::Pose>& out)->bool{
        out=std::make_unique<ov::Pose>(*base);
        math::Vec3 target=native_reload::Model(sk,*out,rig.right.hand).t;
        gik::Params wide;wide.minReach=0.05f;gik::Result solved;bool ok=true;
        for(int iteration=0;iteration<8&&ok;++iteration)
        {
            ok=gik::SolveArm(sk,*out,rig.right,target,nullptr,1.0f,wide,solved);
            target=target+(wanted-native_reload::Model(sk,*out,rig.weaponRight).t);
        }
        return ok&&gik::Len(native_reload::Model(sk,*out,rig.weaponRight).t-wanted)<0.01f;
    };
    // H) A healthy reload whose weapon passes 12-18cm from the head (the baseline is there already, the native correction moves
    //    the weapon by 2cm) and ONE rejected frame (reach / contact): the clearance compares the replay with the PREVIOUS output,
    //    not with a fixed 18cm. With the fixed band a replay that is as far from the head as the previous output was cut to
    //    ~.5 and the next accepted frame (no clearance on that path) jumped back to 1: 1.0 -> .5 -> 1.0 (.2 at 13cm). Mutation 13
    //    = the fixed band.
    {
        const auto headPos=native_reload::Model(sk,*base,head).t;
        std::unique_ptr<ov::Pose> nearBase,nearNative;
        const bool built=weaponAt(headPos+math::Vec3{0.0f,0.13f,0.0f},nearBase)&&weaponAt(headPos+math::Vec3{0.0f,0.15f,0.0f},nearNative);
        if(!built)return 2;
        integ::PoseSnapshot nearSnap=snap; // the baseline (snapshot) pose already has the weapon near the head
        for(int b=0;b<sk.n;++b)
        {
            const auto& x=nearBase->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(nearSnap.pose+size_t(b)*7,row,sizeof(row));
        }
        const auto weaponPos=[&](const ov::Pose& q){return native_reload::Model(sk,q,rig.weaponRight).t;};
        reload_contacts_runtime::Continuity state;
        auto run=[&](const ov::Pose& native,ov::Pose& out,native_reload_contacts::Result& result,int64_t ticks,uint64_t sequence)
        {
            nearSnap.sequence=sequence;
            std::vector<unsigned char> records;Records(native,records);reload_contacts_runtime::Cycle cycle;
            const bool captured=reload_contacts_runtime::Capture(prepared,nearSnap,records.data(),sk.n,true,cycle);
            Records(*nearBase,records);
            if(!captured)return false;
            const bool ok=reload_contacts_runtime::Finish(prepared,nearSnap,records.data(),sk.n,true,cycle,result,1.0f,
                                                          reload_contacts_runtime::kAuthoredContactMinReach,&state,ticks,10000000);
            return ok&&reload_contacts_runtime::Read(prepared,records.data(),out);
        };
        auto unreachable=unreachableOf(*nearBase); // hand out of reach: the solve is rejected
        ov::Pose o1,o2,o3;native_reload_contacts::Result q1,q2,q3;
        const int64_t t0=11000000;
        const bool ok1=run(*nearNative,o1,q1,t0,50),ok2=run(*unreachable,o2,q2,t0+166667,51),ok3=run(*nearNative,o3,q3,t0+166667*2,52);
        const auto shift=[&](const ov::Pose& q){return gik::Len(weaponPos(q)-weaponPos(*nearBase));};
        const float dBase=weaponHead(*nearBase),d1=ok1?weaponHead(o1):0,s1=ok1?shift(o1):0,s2=ok2?shift(o2):0,s3=ok3?shift(o3):0;
        std::printf("  baseline weapon %.3fm from the head, output %.3fm: correction (weapon shift) accepted %.3fm / rejected next frame %.3fm / accepted again %.3fm (solver ok %d %d %d)\n",
                    dBase,d1,s1,s2,s3,q1.ok,q2.ok,q3.ok);
        Check(q1.ok&&!q2.ok&&q3.ok&&dBase>0.12f&&d1>0.12f&&d1<0.18f&&s1>0.01f&&s1<0.05f,"fixture: an accepted frame with the weapon 12-18cm from the head, a rejected frame, an accepted frame");
        Check(ok2&&s2>=0.75f*s1&&s2<=s1+0.002f,"one rejected frame at 12-18cm from the head replays the correction at ~86% (no dip)");
        Check(ok3&&s3>=0.9f*s1&&std::fabs(state.reject.headM-weaponHead(o3))<0.01f,"the next accepted frame is back at the full correction; the head distance of the output is remembered");
    }
    // G) HoldOnly (an eligible reload frame without a native cycle) writes through the same validated snapshot as Finish. A
    //    Prepared that does not match THIS snapshot - a remap published Prepared{}, the rig-change window (numBones differs),
    //    a record count that does not cover the mapped bones - must neither be read from nor written to. Without the check the
    //    joint limiter limited and wrote the records of whatever buffer it was handed (out-of-range indices, parent[-1]).
    {
        auto displaced=std::make_unique<ov::Pose>(*base);
        const auto baseHand=native_reload::Model(sk,*base,rig.left.hand).t;
        gik::Params wide;gik::Result solved;
        if(!gik::SolveArm(sk,*displaced,rig.left,baseHand+math::Vec3{0.0f,0.08f,0.06f},nullptr,1.0f,wide,solved))return 2;
        reload_contacts_runtime::Continuity state;ov::Pose first;native_reload_contacts::Result r1;
        const int64_t t0=9000000,next=t0+166667,tps=10000000;
        finishWith(prepared,*displaced,1.0f,first,r1,&state,t0);
        // The next frame hands in a record buffer whose right upper arm swings by 100deg: the limiter has something to do.
        auto jumped=std::make_unique<ov::Pose>(*base);
        jumped->b[rig.right.upper].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0,0,1},100.0f*math::kPi/180.0f),base->b[rig.right.upper].r));
        std::vector<unsigned char> original;Records(*jumped,original);
        integ::PoseSnapshot good=snap;good.sequence=40;good.reloadContactsWeight=1.0f;
        integ::PoseSnapshot mismatched=good;mismatched.numBones=uint32_t(sk.n-1); // rig-change window
        Check(r1.ok&&state.reject.haveDelta&&state.joints.valid&&reload_contacts_runtime::Valid(prepared,good,uint32_t(sk.n))&&
              !reload_contacts_runtime::Valid(prepared,mismatched,uint32_t(sk.n)),"fixture: a live reject hold + joint limiter, a matching and a mismatching snapshot");
        auto run=[&](const reload_contacts_runtime::Prepared& config,const integ::PoseSnapshot& s,uint32_t count,bool& wrote)
        {
            auto st=state;auto records=original;
            const bool ok=reload_contacts_runtime::HoldOnly(config,s,records.data(),count,true,st,next,tps);
            wrote=std::memcmp(records.data(),original.data(),original.size())!=0;
            return ok;
        };
        bool wroteGood=false,wroteMismatch=true,wroteShort=true,wroteEmpty=true;
        const bool okGood=run(prepared,good,uint32_t(sk.n),wroteGood);
        const bool okMismatch=run(prepared,mismatched,uint32_t(sk.n),wroteMismatch);
        const bool okShort=run(prepared,good,uint32_t(sk.n-10),wroteShort);
        const reload_contacts_runtime::Prepared empty;
        const bool okEmpty=run(empty,good,uint32_t(sk.n),wroteEmpty);
        std::printf("  HoldOnly: matching snapshot wrote=%d (ok=%d); mismatching numBones wrote=%d (ok=%d); short record count wrote=%d (ok=%d); empty Prepared wrote=%d (ok=%d)\n",
                    wroteGood,okGood,wroteMismatch,okMismatch,wroteShort,okShort,wroteEmpty,okEmpty);
        Check(wroteGood,"fixture: a matching snapshot makes HoldOnly replay / limit the records");
        Check(!okMismatch&&!wroteMismatch,"HoldOnly: a snapshot with another bone count (rig-change window) leaves the records untouched");
        Check(!okShort&&!wroteShort,"HoldOnly: a record count that does not cover the mapped bones leaves the records untouched");
        Check(!okEmpty&&!wroteEmpty,"HoldOnly: an empty Prepared (published by a remap) leaves the records untouched");
    }
    // J) A perspective denial that finds the state owned by another animation job cannot reset it. It leaves a request that
    //    the NEXT owner honours before touching the state (otherwise the old correction is replayed after the denial).
    //    Mutation 16 drops the request.
    {
        auto& shared=reload_contacts_runtime::g_continuity;
        shared.Reset();
        shared.reject.valid=true;shared.reject.haveDelta=true;shared.reject.amp=0.5f;shared.reject.headM=0.2f;
        {
            reload_contacts_runtime::ContinuityLock owner;
            reload_contacts_runtime::ResetContinuityOnDenial();
            Check(owner&&shared.reject.haveDelta&&shared.reject.amp==0.5f,"denial while another job owns the state: the state of the owner is not touched");
        }
        {
            reload_contacts_runtime::ContinuityLock next;
            Check(next&&!shared.reject.haveDelta&&shared.reject.amp==1.0f&&shared.reject.headM<0.0f,"the next owner finds the state reset (request honoured before use)");
        }
        shared.reject.valid=true;shared.reject.haveDelta=true;shared.reject.amp=0.5f;
        reload_contacts_runtime::ResetContinuityOnDenial();
        Check(!shared.reject.haveDelta&&shared.reject.amp==1.0f,"denial with a free state: reset at once");
        shared.reject.valid=true;shared.reject.haveDelta=true;shared.reject.amp=0.5f;
        {
            reload_contacts_runtime::ContinuityLock again;
            Check(again&&shared.reject.haveDelta&&shared.reject.amp==0.5f,"a denial that was served leaves no request behind");
        }
        shared.Reset();
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

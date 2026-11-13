// Offline test: the native reload contact targets are carried onto the MM shoulders (fix batch 3, P3 RC2 / N6).
// Recorded 21.10 12:15, healthy standing reloads: the MM body stands 6.3cm lower and 8.7cm behind the native body (shoulders 7.6cm lower), the
// contact solve reproduced the ABSOLUTE native hand / weapon model positions on it, so the weapon sat 8-27cm higher above the MM head than in
// the native pose ("pistol over the head") and the shoulder-to-hand distances changed.
// The Prepared config is built by the production builder reload_contacts_runtime::FillPrepared from the production map (Root and Trajectory mapped).
// The run mirrors the production call: the records handed to Finish carry the GAME's (native) channels in the masked shoulder / arm bones
// (upper weight 0) while the snapshot carries the MM channels (rotated clavicles, the shorter feminine arm sockets) that Finish restores before it
// solves; the state (Continuity) is the production one and runs over several frames.
// Usage: ReloadShoulderAnchorTest <woman.utppov>
// Mutation 17 (UTPP_RELOAD_CONTACT_MUTATION=17): absolute native placement (no anchor). 18: the old +-2cm band of the left-hand-under-the-hips
// guard (it cuts the correction when the belt hand lands under the MM hips). 21: the anchor is computed on the records before the masked channels
// are restored.
// Every mutation build must report an assertion FAIL (ctest PASS_REGULAR_EXPRESSION "FAIL ").
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/OverlayFile.hpp"
#include <algorithm>
#include <cmath>
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
    const auto rig=prepared.arms;const int head=prepared.head,hips=prepared.hips;
    if(head<0||hips<=1)return 2;
    const int index=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Ready,false,false);if(index<0)return 2;
    auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
    integ::PoseSnapshot snap;snap.rig=123;snap.sequence=11;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(ov::PoseOverlay::Pistol2H);
    snap.reloadAuthoredClip=true;snap.upperScale=0;snap.traceAction=2;
    const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
    std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
    // MM channels of the masked shoulder / arm bones: asymmetric clavicle rotations (aimwalk-like, 10 and 2deg) and the shorter feminine arm
    // sockets (-11%, 17.4 -> 15.45cm as measured). Finish restores them from the snapshot before it solves.
    const int leftClavicle=sk.parent[rig.left.upper],rightClavicle=sk.parent[rig.right.upper];
    auto mm=std::make_unique<ov::Pose>(*base);
    // Rotate a clavicle about the local axis that moves its arm joint the most (the axes of the rig are not known here).
    const auto swing=[&](int clavicle,int arm,float deg){
        float best=-1.0f;math::Vec3 bestAxis{1.0f,0.0f,0.0f};
        for(const math::Vec3 axis:{math::Vec3{1.0f,0.0f,0.0f},math::Vec3{0.0f,1.0f,0.0f},math::Vec3{0.0f,0.0f,1.0f}})
        {
            ov::Pose trial=*mm;
            trial.b[clavicle].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(axis,deg*math::kPi/180.0f),trial.b[clavicle].r));
            const float moved=gik::Len(native_reload::Model(sk,trial,arm).t-native_reload::Model(sk,*mm,arm).t);
            if(moved>best){best=moved;bestAxis=axis;}
        }
        mm->b[clavicle].r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(bestAxis,deg*math::kPi/180.0f),mm->b[clavicle].r));
    };
    swing(leftClavicle,rig.left.upper,10.0f);
    swing(rightClavicle,rig.right.upper,2.0f);
    mm->b[rig.left.upper].t=mm->b[rig.left.upper].t*0.888f;
    mm->b[rig.right.upper].t=mm->b[rig.right.upper].t*0.888f;
    for(int b:{leftClavicle,rightClavicle,rig.left.upper,rig.right.upper})
        if(!integ::SnapshotUpperBone(snap,uint32_t(b))){std::printf("bone %d is not masked\n",b);return 2;}
    for(int b=0;b<sk.n;++b)
    {
        snap.gameIndex[b]=b>=2 ? b : -1;
        const auto& x=mm->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));
    }
    const auto M=[&](const ov::Pose& p,int bone){return native_reload::Model(sk,p,bone).t;};
    {
        // The roots the anchor must use (restored channels) differ from the records (game channels) by this much at the shoulder mid point.
        const math::Vec3 restoredMid=(M(*mm,rig.left.upper)+M(*mm,rig.right.upper))*0.5f,recordMid=(M(*base,rig.left.upper)+M(*base,rig.right.upper))*0.5f;
        std::printf("  fixture: shoulder mid point of the restored MM channels vs the records: %.4fm apart\n",gik::Len(restoredMid-recordMid));
        if(gik::Len(restoredMid-recordMid)<0.01f){std::printf("fixture: the masked channels barely differ\n");return 2;}
    }
    // Records handed to Finish: the MM pose in the unmasked bones, the game's (native) channels in the masked ones (upper weight 0).
    auto recordsOf=[&](const ov::Pose& native,std::vector<unsigned char>& out){
        ov::Pose rec=*base;
        for(int b=0;b<sk.n;++b)if(integ::SnapshotUpperBone(snap,uint32_t(b)))rec.b[b]=native.b[b];
        Records(rec,out);
    };
    // Finish on the production path: Capture reads `native`; `state` is the production Continuity (null = stateless).
    auto finishWith=[&](const ov::Pose& native,ov::Pose& out,native_reload_contacts::Result& result,reload_contacts_runtime::Continuity* state=nullptr,int64_t ticks=0)
    {
        std::vector<unsigned char> records;Records(native,records);reload_contacts_runtime::Cycle cycle;
        const bool captured=reload_contacts_runtime::Capture(prepared,snap,records.data(),sk.n,true,cycle);
        recordsOf(native,records);
        if(!captured)return false;
        const bool ok=reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,1.0f,
                                                      reload_contacts_runtime::kAuthoredContactMinReach,state,ticks,10000000);
        return ok&&reload_contacts_runtime::Read(prepared,records.data(),out);
    };
    // Several frames of one reload through one production state; the last frame is returned.
    auto finishFrames=[&](const ov::Pose& native,ov::Pose& out,native_reload_contacts::Result& result,int frames)
    {
        reload_contacts_runtime::Continuity state;bool ok=true;
        for(int f=0;f<frames;++f)ok=finishWith(native,out,result,&state,3000000+int64_t(f)*166667)&&ok;
        return ok;
    };
    // A native body that stands `bodyOffset` away from the MM body (a rigid move of the whole pose through the pelvis, optionally turned
    // by yawDeg about the vertical), with a reload choreography of its own: both hands are placed relative to the NATIVE body.
    gik::Params wide;wide.minReach=0.05f;
    auto nativeWith=[&](const math::Vec3& bodyOffset,const math::Vec3& leftHandOnBody,const math::Vec3& rightHandOnBody,std::unique_ptr<ov::Pose>& out,float yawDeg=0.0f)->bool{
        out=std::make_unique<ov::Pose>(*base);
        auto desired=native_reload::Model(sk,*base,hips);desired.t=desired.t+bodyOffset;
        if(yawDeg!=0.0f)desired.r=math::QuatNormalize(math::QuatMul(gik::AxisAngle(math::Vec3{0.0f,0.0f,1.0f},yawDeg*math::kPi/180.0f),desired.r));
        const auto parentModel=sk.parent[hips]>=0 ? native_reload::Model(sk,*base,sk.parent[hips]) : ov::Xf{};
        const auto local=native_reload::Relative(parentModel,desired);
        out->b[hips].t=local.t;out->b[hips].r=local.r;
        gik::Result solved;
        const math::Vec3 leftTarget=M(*base,rig.left.hand)+bodyOffset+leftHandOnBody;
        const math::Vec3 rightTarget=M(*base,rig.right.hand)+bodyOffset+rightHandOnBody;
        if(!gik::SolveArm(sk,*out,rig.left,leftTarget,nullptr,1.0f,wide,solved)||gik::Len(M(*out,rig.left.hand)-leftTarget)>2.0e-3f)return false;
        if(!gik::SolveArm(sk,*out,rig.right,rightTarget,nullptr,1.0f,wide,solved)||gik::Len(M(*out,rig.right.hand)-rightTarget)>2.0e-3f)return false;
        return true;
    };
    const auto shoulderToHand=[&](const ov::Pose& p,const gik::ArmChain& arm){return M(p,arm.hand)-M(p,arm.upper);};
    // What the anchor guarantees: the position of a bone relative to the mid point of the two shoulder joints of the SAME pose.
    const auto mid=[&](const ov::Pose& p){return (M(p,rig.left.upper)+M(p,rig.right.upper))*0.5f;};
    const auto rel=[&](const ov::Pose& p,int bone){return M(p,bone)-mid(p);};
    const auto relError=[&](const ov::Pose& out,const ov::Pose& native,int bone){return gik::Len(rel(out,bone)-rel(native,bone));};
    const auto handOverHips=[&](const ov::Pose& p,const ov::Pose& body){return M(p,rig.left.hand).z-M(body,hips).z;};
    // Elbow flexion (deg, 0 = straight): angle between the upper-arm and the forearm direction.
    const auto flexion=[&](const ov::Pose& p,const gik::ArmChain& arm){
        const math::Vec3 u=M(p,arm.lower)-M(p,arm.upper),l=M(p,arm.hand)-M(p,arm.lower);
        const float c=math::Dot(u,l)/(gik::Len(u)*gik::Len(l));
        return std::acos(std::clamp(c,-1.0f,1.0f))*57.29578f;
    };
    // The measured offset of a healthy standing reload: the MM body 6.3cm lower and 8.7cm behind the native one.
    const math::Vec3 healthy{0.006f,0.087f,0.063f};
    // Reload choreography on the body: left hand down and back towards the belt, right hand (weapon) up and back towards the head.
    const math::Vec3 leftChoreo{-0.02f,-0.10f,-0.22f},rightChoreo{0.0f,-0.08f,0.12f};

    // A) Native body identical to the MM body (but for the masked channels): the hands keep their place relative to the shoulders.
    {
        std::unique_ptr<ov::Pose> native;
        if(!nativeWith({0,0,0},leftChoreo,rightChoreo,native)){std::printf("fixture A failed\n");return 2;}
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finishFrames(*native,out,r,6);
        Check(ok&&relError(out,*native,rig.left.hand)<0.005f&&relError(out,*native,rig.right.hand)<0.005f,
              "native body == MM body: both hands keep their native place relative to the MM shoulders (< 5mm)");
    }
    // B) Healthy offset: the native body stands 6.3cm higher / 8.7cm forward. The position of the hands and of the weapon relative to the
    //    shoulder mid point, the weapon height over the head and the hand-to-weapon binding of the native choreography survive on the MM body.
    std::unique_ptr<ov::Pose> nativeHealthy;
    if(!nativeWith(healthy,leftChoreo,rightChoreo,nativeHealthy)){std::printf("fixture B failed\n");return 2;}
    ov::Pose outHealthy;native_reload_contacts::Result rHealthy;
    const bool okHealthy=finishFrames(*nativeHealthy,outHealthy,rHealthy,6);
    {
        float worst=9;float weaponMiss=9,headMiss=9;
        if(okHealthy)
        {
            worst=std::max({relError(outHealthy,*nativeHealthy,rig.left.hand),relError(outHealthy,*nativeHealthy,rig.right.hand),relError(outHealthy,*nativeHealthy,rig.weaponRight)});
            headMiss=std::fabs((M(outHealthy,rig.weaponRight).z-M(*base,head).z)-(M(*nativeHealthy,rig.weaponRight).z-M(*nativeHealthy,head).z));
            weaponMiss=gik::Len((M(outHealthy,rig.weaponRight)-M(outHealthy,rig.right.hand))-(M(*nativeHealthy,rig.weaponRight)-M(*nativeHealthy,rig.right.hand)));
        }
        std::printf("  healthy offset: accepted=%d reject=%d, hand / weapon error relative to the shoulder mid point %.4fm, weapon height over the head differs from native by %.4fm, weapon-hand binding error %.5fm\n",
                    okHealthy,int(rHealthy.reject),worst,headMiss,weaponMiss);
        Check(okHealthy&&rHealthy.contactMax<1.0e-3f,"healthy offset: the correction is accepted over several frames of the production state (contact within 1mm)");
        Check(okHealthy&&worst<0.005f,"healthy offset: hands and weapon keep their native place relative to the MM shoulder mid point (< 5mm, masked channels restored)");
        Check(okHealthy&&headMiss<0.02f,"healthy offset: the weapon keeps its native height over the MM head (< 2cm; absolute placement left it 6.3cm higher)");
        Check(okHealthy&&weaponMiss<1.0e-3f,"healthy offset: the weapon binding to the hand is untouched");
    }
    // B2) N6: the left elbow folded ~20deg tighter than the native pose (ELBOW_EXTREME 159-163deg against 139deg, recorded 21.10 12:15 k1, k8, k12,
    //     k13). The cause is the one of B: the MM shoulder is lower than the native one while the hand stays at its absolute native
    //     height, so the shoulder-to-hand distance shrinks and the elbow closes (the right arm opens instead). With the target carried onto
    //     the shoulders the flexion follows the native choreography. What stays is the difference of the individual shoulders from their mid
    //     point (the MM clavicles are rotated / the sockets narrower than the native ones, +-2cm) and, below, a turn of the torso: bounded, not zero.
    {
        float dLeft=99,dRight=99,nativeLeft=0,nativeRight=0;
        if(okHealthy)
        {
            nativeLeft=flexion(*nativeHealthy,rig.left);nativeRight=flexion(*nativeHealthy,rig.right);
            dLeft=flexion(outHealthy,rig.left)-nativeLeft;dRight=flexion(outHealthy,rig.right)-nativeRight;
        }
        std::printf("  elbow flexion native L %.1f R %.1f deg, output minus native: L %+.2f R %+.2f deg\n",nativeLeft,nativeRight,dLeft,dRight);
        Check(okHealthy&&std::fabs(dLeft)<8.0f&&std::fabs(dRight)<8.0f,"healthy offset: elbow flexion of both arms stays within 8deg of the native pose (the absolute placement changes them by 6 and 22deg in this fixture)");
        std::unique_ptr<ov::Pose> turned;
        if(!nativeWith(healthy,leftChoreo,rightChoreo,turned,4.0f)){std::printf("fixture B2 failed\n");return 2;}
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finishFrames(*turned,out,r,6);
        float residual=9,dl=99,dr=99;
        if(ok)
        {
            residual=std::max(gik::Len(shoulderToHand(out,rig.left)-shoulderToHand(*turned,rig.left)),gik::Len(shoulderToHand(out,rig.right)-shoulderToHand(*turned,rig.right)));
            dl=flexion(out,rig.left)-flexion(*turned,rig.left);dr=flexion(out,rig.right)-flexion(*turned,rig.right);
        }
        std::printf("  native torso turned 4deg: accepted=%d, shoulder-to-hand vector error %.3fm, elbow flexion output minus native L %+.2f R %+.2f deg\n",ok,residual,dl,dr);
        Check(ok&&residual<0.04f&&std::fabs(dl)<10.0f&&std::fabs(dr)<10.0f,"torso turned 4deg: shoulder-to-hand vectors within 4cm, elbow flexion within 10deg of the native pose (the residual of a translation-only anchor)");
    }
    // C) Belt reach: the native left hand takes the magazine at its own belt and the MM torso hangs lower / leans further forward over its pelvis, so the
    //    carried hand lands UNDER the MM hips (R19 k3/k4: down to 10.6cm for ~.2s). The new magazine rides in the frame of the weapon and the runtime cannot
    //    write it, so the left hand must keep its native place relative to the weapon (and the weapon relative to the shoulders): no lifted arm (the hand
    //    would hover over the magazine), no lifted target (the pistol would jump 14cm), and the correction must not be cut by the placement guard.
    const auto beltNative=[&](const math::Vec3& body,float overHips,std::unique_ptr<ov::Pose>& out)->bool{
        auto desired=native_reload::Model(sk,*base,hips);desired.t=desired.t+body;
        const math::Vec3 want=math::Vec3{M(*base,rig.left.upper).x,desired.t.y+0.03f,desired.t.z+overHips};
        return nativeWith(body,want-(M(*base,rig.left.hand)+body),rightChoreo,out);
    };
    const auto handWeapon=[&](const ov::Pose& p){return M(p,rig.left.hand)-M(p,rig.weaponRight);};
    for(const float nativeOverHips:{-0.01f,-0.035f})
    {
        std::unique_ptr<ov::Pose> native;
        if(!beltNative(healthy,nativeOverHips,native)){std::printf("fixture C failed\n");return 2;}
        std::printf("  fixture: native left hand %.3fm over the native hips, %.3fm from its shoulder (reach .537), %.3fm from the weapon\n",
                    M(*native,rig.left.hand).z-M(*native,hips).z,gik::Len(shoulderToHand(*native,rig.left)),gik::Len(handWeapon(*native)));
        ov::Pose out;native_reload_contacts::Result r;
        const bool ok=finishFrames(*native,out,r,6);
        const float over=ok?handOverHips(out,*base):-9;
        const float magazine=ok?gik::Len(handWeapon(out)-handWeapon(*native)):9;
        const float weaponError=ok?relError(out,*native,rig.weaponRight):9;
        const float weaponAbove=ok?(M(out,rig.weaponRight).z-M(*base,head).z)-(M(*native,rig.weaponRight).z-M(*native,head).z):9;
        std::printf("  belt hand %.3fm over the native hips: accepted=%d reject=%d, left hand %.3fm over the MM hips, hand-to-weapon vector off by %.4fm, weapon off its place relative to the shoulders by %.4fm, weapon over the head minus native %+.4fm\n",
                    nativeOverHips,ok,int(r.reject),over,magazine,weaponError,weaponAbove);
        Check(ok&&magazine<0.01f,"belt hand: the left hand keeps its native place relative to the weapon (the magazine rides with it, < 1cm)");
        Check(ok&&weaponError<0.01f&&relError(out,*native,rig.left.hand)<0.01f,"belt hand: the correction keeps its full weight, hand and weapon keep their place relative to the shoulders (< 1cm, not cut by the guard)");
        Check(ok&&weaponAbove<0.02f,"belt hand: the weapon is not higher over the MM head than the native weapon over the native head (+2cm)");
    }
    // D) Continuity: the native left hand sinks from 8cm over to 3.5cm under the native belt (a 1/60 s frame per 5mm) through the production state.
    //    The left hand over the MM hips and the weapon over the MM head may step by 1.5cm per frame at most (the guard band is wide enough not to cut the
    //    correction at any point of the reach), the left hand never leaves the weapon (< 1cm) and the weapon never leaves its place relative to the
    //    shoulders (< 1cm) nor gets higher over the head than the native one (+2cm).
    {
        reload_contacts_runtime::Continuity state;
        float previousHand=-99,previousWeapon=-99,worstHand=0,worstWeapon=0,worstRel=0,worstAbove=-9,worstMag=0,lowest=9;int rejected=0;
        for(int step=0;step<=23;++step)
        {
            const float over=0.08f-0.005f*float(step);
            std::unique_ptr<ov::Pose> native;
            if(!beltNative(healthy,over,native)){std::printf("fixture D failed at %d\n",step);return 2;}
            ov::Pose out;native_reload_contacts::Result r;
            const bool ok=finishWith(*native,out,r,&state,3000000+int64_t(step)*166667);
            if(!ok){++rejected;continue;}
            const float hand=handOverHips(out,*base),weapon=M(out,rig.weaponRight).z-M(*base,head).z;
            if(previousHand>-90){worstHand=std::max(worstHand,std::fabs(hand-previousHand));worstWeapon=std::max(worstWeapon,std::fabs(weapon-previousWeapon));}
            previousHand=hand;previousWeapon=weapon;lowest=std::min(lowest,hand);
            worstRel=std::max(worstRel,relError(out,*native,rig.weaponRight));
            worstMag=std::max(worstMag,gik::Len(handWeapon(out)-handWeapon(*native)));
            worstAbove=std::max(worstAbove,weapon-(M(*native,rig.weaponRight).z-M(*native,head).z));
        }
        std::printf("  belt sweep +8cm -> -3.5cm: rejected %d, lowest left hand %.3fm over the MM hips, largest per-frame step: left hand over hips %.4fm, weapon over head %.4fm; hand-to-weapon off by %.4fm, weapon off its shoulder-relative place by %.4fm, over the head minus native %+.4fm\n",
                    rejected,lowest,worstHand,worstWeapon,worstMag,worstRel,worstAbove);
        Check(rejected==0&&worstHand<0.015f&&worstWeapon<0.015f,"belt sweep: hand and weapon heights are continuous (< 1.5cm per frame, no rejected frame)");
        Check(rejected==0&&worstMag<0.01f&&worstRel<0.01f&&worstAbove<0.02f,"belt sweep: the left hand never leaves the weapon and the weapon never leaves its place (< 1cm, not higher over the head than native +2cm)");
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

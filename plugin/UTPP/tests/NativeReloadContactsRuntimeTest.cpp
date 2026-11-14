// Focused same-call reload endpoint handover audit. No game process or DLL.
// Usage: NativeReloadContactsRuntimeTest <woman.utppov>
// Raw custom-clip endpoints are motion bases; composed fixture applies the SAME
// feminine socket pass as MotionPose. This is not coverage of intermediate clips/AO.
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/FeminineShoulders.hpp"
#include "../src/mm/OverlayFile.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <vector>
using namespace utpp::mm;
static float Width(const ov::Skeleton& sk,const gik::Rig& rig,const ov::Pose& p){
 return gik::Len(native_reload::Model(sk,p,rig.right.upper).t-native_reload::Model(sk,p,rig.left.upper).t);
}
static void Records(const ov::Pose& p,std::vector<unsigned char>& out){
 out.resize(size_t(p.n)*48);
 for(int b=0;b<p.n;++b){const auto& x=p.b[b];float r[12]={x.t.x,x.t.y,x.t.z,1,x.r.x,x.r.y,x.r.z,x.r.w,1,1,1,1};std::memcpy(out.data()+size_t(b)*48,r,sizeof(r));}
}
int main(int argc,char** argv){
 if(argc!=2)return 2;
 ov::OverlayFile file;std::string error;
 if(!file.LoadFromFile(argv[1],ov::kRigIdentityWomanOv,error)){std::printf("load %s\n",error.c_str());return 2;}
 const auto& sk=file.Skel();const auto rig=gik::FindRig(sk.n,[&](int i){return file.BoneName(i);});
 if(!rig.Valid(sk))return 2;
 int passed=0,failed=0,postures=0;
 const auto check=[&](bool ok,const char* what){(ok?passed:failed)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);};
 for(const auto overlay:{ov::PoseOverlay::Pistol1H,ov::PoseOverlay::Pistol2H})
 for(const auto aim:{ov::AimState::Ready,ov::AimState::Aiming}){
  const int index=file.FindOverlayPose(overlay,aim,false,false);if(index<0)return 2;
  auto raw=std::make_unique<ov::Pose>(),baseline=std::make_unique<ov::Pose>(),scratch=std::make_unique<ov::Pose>();
  file.FramePose(file.PoseAt(index).firstFrame,*raw);*baseline=*raw;
  const auto find=[&](const char* n){for(int i=0;i<sk.n;++i)if(file.BoneName(i)==n)return i;return -1;};
  gik::Params postureParams;postureParams.softStart=.995f;postureParams.maxReach=.999f;femshoulder::Result posture;
  const bool feminine=femshoulder::Apply(sk,*baseline,*scratch,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,postureParams,posture,overlay==ov::PoseOverlay::Pistol1H);
  std::printf("endpoint overlay=%d aim=%d feminine=%d applied=%d rawWidthMM=%.6f composedWidthMM=%.6f\n",int(overlay),int(aim),feminine,posture.applied,Width(sk,rig,*raw)*1000,Width(sk,rig,*baseline)*1000);
  if(!feminine||!posture.applied){std::printf("NOT RUN narrow fixture: actual feminine pass rejected this base\n");continue;}++postures;
  reload_contacts_runtime::Prepared prepared;prepared.rig=123;prepared.skeleton=sk;prepared.arms=rig;
  integ::PoseSnapshot snap;snap.rig=123;snap.sequence=99;snap.weight=1;snap.numBones=sk.n;snap.firstBone=2;snap.traceOverlay=int(overlay);snap.reloadAuthoredClip=true;
  const auto mask=integ::BuildWeaponActionMask(sk.n,2,[&](uint32_t b){return file.BoneName(b);});if(!mask.ok)return 2;
  std::memcpy(snap.upperBits,mask.bits,sizeof(mask.bits));
  for(int b=0;b<sk.n;++b){prepared.gameIndex[b]=snap.gameIndex[b]=b;const auto& x=baseline->b[b];float row[7]={x.t.x,x.t.y,x.t.z,x.r.x,x.r.y,x.r.z,x.r.w};std::memcpy(snap.pose+size_t(b)*7,row,sizeof(row));}
  // Recorded native upper-arm norm^2 pairs straddled the old .001 cutoff.
  // Keep direction/contacts fixed so any discontinuity is adapter admission,
  // not a deliberately different animation or a loosened IK reach condition.
  {
   auto readSnap=snap;readSnap.traceAction=2;readSnap.upperScale=0;readSnap.reloadAuthoredClip=true;
   std::vector<unsigned char> referenceRecords;Records(*raw,referenceRecords);
   reload_contacts_runtime::Cycle referenceCycle;
   const bool referenceCaptured=reload_contacts_runtime::Capture(prepared,readSnap,referenceRecords.data(),sk.n,true,referenceCycle);
   integ::BlendIntoRecords(referenceRecords.data(),sk.n,readSnap,1);
   native_reload_contacts::Result referenceResult;
   const bool referenceFinished=reload_contacts_runtime::Finish(prepared,readSnap,referenceRecords.data(),sk.n,true,referenceCycle,referenceResult,1);
   ov::Pose referencePose;const bool referenceRead=reload_contacts_runtime::Read(prepared,referenceRecords.data(),referencePose);
   check(referenceCaptured&&referenceFinished&&referenceRead,"unit reference contact solve accepted");
   for(float normSquared:{.9997187103f,.9987433754f,.9990186368f,.9982967504f,.9994979151f,.9982827961f}){
    std::vector<unsigned char> records;Records(*raw,records);
    for(int b:{rig.left.upper,rig.right.upper}){
     float q[4];auto* dst=records.data()+size_t(b)*48+16;std::memcpy(q,dst,16);
     const float current=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
     const float scale=std::sqrt(normSquared/current);for(float& v:q)v*=scale;std::memcpy(dst,q,16);
    }
    const auto unchanged=records;reload_contacts_runtime::Cycle cycle;
    const bool captured=reload_contacts_runtime::Capture(prepared,readSnap,records.data(),sk.n,true,cycle);
    check(captured&&records==unchanged,"recorded near-unit sample captured without native writes");
    float normError=0;for(int b=0;b<cycle.native.n;++b)normError=std::max(normError,std::fabs(math::QuatDot(cycle.native.b[b].r,cycle.native.b[b].r)-1));
    check(captured&&normError<2e-6f,"accepted native copy is consistently unit quaternion");
    integ::BlendIntoRecords(records.data(),sk.n,readSnap,1);native_reload_contacts::Result result;
    const bool finished=reload_contacts_runtime::Finish(prepared,readSnap,records.data(),sk.n,true,cycle,result,1);
    ov::Pose final;const bool read=reload_contacts_runtime::Read(prepared,records.data(),final);
    float error=0;
    if(referenceRead&&read)for(int b:{rig.left.hand,rig.right.hand,rig.weaponRight})
     error=std::max(error,gik::Len(native_reload::Model(sk,final,b).t-native_reload::Model(sk,referencePose,b).t));
    check(finished&&read&&referenceFinished&&error<1e-5f&&result.contactMax<1e-4f,
          "contact outcome stays continuous across old norm cutoff");
   }
   for(int bad=0;bad<6;++bad){
    std::vector<unsigned char> records;Records(*raw,records);auto* dst=records.data()+size_t(rig.left.upper)*48;
    float row[12];std::memcpy(row,dst,48);
    if(bad<=2){
     const float target=bad==0?0.f:(bad==1?1.0101f:.9899f);
     const float current=row[4]*row[4]+row[5]*row[5]+row[6]*row[6]+row[7]*row[7];
     const float scale=std::sqrt(target/current);for(int j=4;j<8;++j)row[j]*=scale;
    }else if(bad==3)row[4]=std::numeric_limits<float>::quiet_NaN();
    else if(bad==4)row[8]=1.01f;
    else row[0]=std::numeric_limits<float>::infinity();
    std::memcpy(dst,row,48);const auto unchanged=records;reload_contacts_runtime::Cycle cycle;
    const bool captured=reload_contacts_runtime::Capture(prepared,readSnap,records.data(),sk.n,true,cycle);
    native_reload_contacts::Result result;
    const bool finished=reload_contacts_runtime::Finish(prepared,readSnap,records.data(),sk.n,true,cycle,result,1);
    check(!captured&&!finished&&!cycle.valid&&records==unchanged,
          "gross quaternion/degenerate/NaN/scale invalid sample rejects atomically");
   }
  }
  bool exit=false;
  for(const float weight:{0.f,.001f,.5f,1.f,.5f,.001f,0.f}){
   if(weight==1)exit=true;snap.traceAction=exit?0:2;snap.upperScale=1-weight;
   std::vector<unsigned char> records;Records(*raw,records);reload_contacts_runtime::Cycle cycle;
   const bool captured=reload_contacts_runtime::Capture(prepared,snap,records.data(),sk.n,true,cycle);
   integ::BlendIntoRecords(records.data(),sk.n,snap,1);
   const auto before=records;native_reload_contacts::Result result;
   const bool finished=reload_contacts_runtime::Finish(prepared,snap,records.data(),sk.n,true,cycle,result,weight);
   ov::Pose final;const bool read=reload_contacts_runtime::Read(prepared,records.data(),final);
   float localError=0;for(int b:{rig.left.upper,rig.right.upper})localError=std::max(localError,gik::Len(final.b[b].t-baseline->b[b].t));
   std::printf("weight=%.3f exit=%d captured=%d finished=%d reject=%d widthMM=%.6f upperLocalTerrorMM=%.6f contactMM=%.6f\n",weight,exit,captured,finished,int(result.reject),read?Width(sk,rig,final)*1000:-1,localError*1000,result.contactMax*1000);
   check(captured&&read,"same-cycle capture/read valid");
   if(weight==0)check(finished&&records==before,"zero correction exactly preserves composed baseline");
   else check(finished,"production reach policy accepts raw endpoint/composed feminine handover");
   if(weight>0&&weight<=.001f&&finished)
    check(std::fabs(Width(sk,rig,final)-Width(sk,rig,*baseline))<=.00025f,
          "epsilon correction changes feminine socket width by at most0.25mm");
   // Preserve the old native route as an independent control. Its historical
   // epsilon width shift is diagnostic only, not a failure of the new route.
   {
    auto legacySnap=snap;legacySnap.reloadAuthoredClip=false;
    std::vector<unsigned char> legacyRecords;Records(*raw,legacyRecords);
    reload_contacts_runtime::Cycle legacyCycle;
    const bool legacyCaptured=reload_contacts_runtime::Capture(prepared,legacySnap,legacyRecords.data(),sk.n,true,legacyCycle);
    integ::BlendIntoRecords(legacyRecords.data(),sk.n,legacySnap,1);const auto legacyBefore=legacyRecords;
    native_reload_contacts::Result legacyResult;
    const bool legacyFinished=reload_contacts_runtime::Finish(prepared,legacySnap,legacyRecords.data(),sk.n,true,legacyCycle,legacyResult,weight);
    ov::Pose legacyFinal;const bool legacyRead=reload_contacts_runtime::Read(prepared,legacyRecords.data(),legacyFinal);
    std::printf("legacy weight=%.3f exit=%d captured=%d finished=%d reject=%d widthMM=%.6f deltaFromComposedMM=%.6f\n",
      weight,exit,legacyCaptured,legacyFinished,int(legacyResult.reject),legacyRead?Width(sk,rig,legacyFinal)*1000:-1,
      legacyRead?(Width(sk,rig,legacyFinal)-Width(sk,rig,*baseline))*1000:-1);
    check(legacyCaptured&&legacyRead,"legacy same-cycle control capture/read valid");
    if(weight==0)check(legacyFinished&&legacyRecords==legacyBefore,"legacy zero correction remains byte exact");
    if(!legacyFinished)check(legacyRecords==legacyBefore,"legacy rejected correction remains atomic");
   }
   if(weight>0&&finished)check(localError<1e-5f,"verified authored correction retains composed narrow upper-arm LOCAL translations at every weight");
   if(!finished)check(records==before,"rejected correction is atomic (baseline remains)");
  }
  for(bool exact:{false,true}){
   ov::Pose out;native_reload_contacts::Result result;const bool ok=native_reload_contacts::Apply(sk,rig,*raw,*baseline,out,result,1,native_reload::kUnboundedGuideTurnRad,reload_contacts_runtime::kPistolReloadRightGuideTurnCapRad,exact);
   std::printf("directContact exactReach=%d accepted=%d reject=%d contactMM=%.6f\n",exact,ok,int(result.reject),result.contactMax*1000);
  }
 }
 check(postures>0,"at least one actual feminine narrow endpoint exercised");
 // Arm limits after the reach step (LimitArmBySwivel): hand MODEL transform exact, forearm local unchanged, under both limits
 // bit for bit, weight 0 bit for bit, an over-limit wrist bend / pronation lowered (reached = under the limit) and never raised.
 {
  using reload_contacts_runtime::LimitArmBySwivel;using reload_contacts_runtime::WristBendDeg;
  const int index=file.FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Ready,false,false);if(index<0)return 2;
  auto base=std::make_unique<ov::Pose>();file.FramePose(file.PoseAt(index).firstFrame,*base);
  const auto sameModel=[&](const ov::Pose& a,const ov::Pose& b,int bone){
   const auto x=native_reload::Model(sk,a,bone),y=native_reload::Model(sk,b,bone);
   return gik::Len(x.t-y.t)<1e-5f && std::fabs(std::fabs(math::QuatDot(x.r,y.r))-1.0f)<1e-5f;};
  int limitedCases=0;
  for(const gik::ArmChain* arm:{&rig.left,&rig.right}){
   const auto run=[&](const ov::Pose& in,float weight,ov::Pose& out){out=in;return LimitArmBySwivel(out,sk,arm->upper,arm->lower,arm->hand,
    reload_contacts_runtime::kReloadPronationLimitDeg,reload_contacts_runtime::kReloadWristBendLimitDeg,reload_contacts_runtime::kReloadMaxSwivelDeg,weight);};
   // a pose inside both limits: untouched bit for bit
   // (forearm roll of the overlay pose moved into the hand first, then the hand set to identity: pronation 0, bend 0)
   auto inside=std::make_unique<ov::Pose>(*base);
   {ov::ForearmTwistRig tw;tw.Build(sk,rig.left.lower,rig.left.hand,rig.right.lower,rig.right.hand);float moved[2]={};ov::MoveForearmTwistToHands(tw,*inside,moved);}
   inside->b[arm->hand].r=math::Quat{0,0,0,1};inside->b[arm->lower].r=math::QuatNormalize(math::QuatMul(inside->b[arm->lower].r,ov::AxisAngleQuat(math::Vec3{0,0,1},-40.0f)));
   auto out=std::make_unique<ov::Pose>();
   const auto rIn=run(*inside,1.0f,*out);
   std::printf("arm limit inside side=%s bend %.1f pronation %.1f\n",arm==&rig.left?"L":"R",rIn.bendBeforeDeg,rIn.pronationBeforeDeg);
   check(!rIn.applied && std::memcmp(out.get(),inside.get(),sizeof(ov::Pose))==0,"arm limit: pose inside both limits untouched bit for bit");
   // pure bend (twist 0), pure twist (bend 0) and both; a combined case may stay where lowering one would raise the other
   for(const float bendDeg:{0.0f,95.0f,110.0f})for(const float twistDeg:{0.0f,120.0f,150.0f}){
    if(bendDeg==0.0f && twistDeg==0.0f)continue;
    // elbow flexed 40 deg; hand bent about local z by bendDeg, then twisted about the forearm axis by twistDeg
    auto bad=std::make_unique<ov::Pose>(*inside);
    bad->b[arm->hand].r=math::QuatNormalize(math::QuatMul(ov::AxisAngleQuat(math::Vec3{1,0,0},twistDeg),ov::AxisAngleQuat(math::Vec3{0,0,1},bendDeg)));
    const float b0=WristBendDeg(bad->b[arm->hand].r);
    const auto r=run(*bad,1.0f,*out);
    std::printf("arm limit side=%s bend %.1f->%.1f pronation %.1f->%.1f swivel %.2f applied=%d\n",arm==&rig.left?"L":"R",r.bendBeforeDeg,r.bendAfterDeg,
                r.pronationBeforeDeg,r.pronationAfterDeg,r.swivelDeg,int(r.applied));
    check(std::fabs(b0-r.bendBeforeDeg)<1e-3f,"arm limit: bend measure = axis angle of the hand local");
    check(sameModel(*out,*bad,arm->hand),"arm limit: hand MODEL transform exact (contacts and weapon bindings unchanged)");
    check(std::memcmp(&out->b[arm->lower],&bad->b[arm->lower],sizeof(ov::Xf))==0,"arm limit: forearm local unchanged (pure hinge kept)");
    const bool raised=r.bendAfterDeg>std::fmax(reload_contacts_runtime::kReloadWristBendLimitDeg,r.bendBeforeDeg)+0.01f ||
                      r.pronationAfterDeg>std::fmax(reload_contacts_runtime::kReloadPronationLimitDeg,r.pronationBeforeDeg)+0.01f;
    check(!raised,"arm limit: neither the bend nor the pronation rises past max(limit, before)");
    {
     const math::Quat restInv=math::QuatInv(math::QuatNormalize(reload_contacts_runtime::kUpperArmRestLocal));
     const auto upTw=[&](const math::Quat& q){return std::fabs(ov::TwistAngleDeg(math::QuatNormalize(math::QuatMul(restInv,q)),math::Vec3{1,0,0}));};
     const float u0=upTw(bad->b[arm->upper].r),u1=upTw(out->b[arm->upper].r);
     check(u1<=std::fmax(reload_contacts_runtime::kReloadUpperArmTwistCapDeg,u0)+0.01f,"arm limit: upper-arm twist never raised past max(100 deg, before)");
    }
    const float e0=std::fmax(r.bendBeforeDeg-reload_contacts_runtime::kReloadWristBendLimitDeg,r.pronationBeforeDeg-reload_contacts_runtime::kReloadPronationLimitDeg);
    const float e1=std::fmax(r.bendAfterDeg-reload_contacts_runtime::kReloadWristBendLimitDeg,r.pronationAfterDeg-reload_contacts_runtime::kReloadPronationLimitDeg);
    if(bendDeg==0.0f || twistDeg==0.0f)check(r.applied && e1<e0,"arm limit: an over-limit wrist (bend only / twist only) lowered by the swivel");
    else check(!r.applied || e1<e0,"arm limit: combined over-limit wrist never made worse");
    if(r.reached)
     check(e1<=0.05f,"arm limit: a swivel inside the cap reaches both limits");
    limitedCases+=r.applied;
    const auto r0=run(*bad,0.0f,*out);
    check(!r0.applied && std::memcmp(out.get(),bad.get(),sizeof(ov::Pose))==0,"arm limit: weight 0 untouched bit for bit");
    // half weight: half the swivel, continuous
    const auto rh=run(*bad,0.5f,*out);
    check(std::fabs(rh.swivelDeg-0.5f*r.swivelDeg)<1e-3f,"arm limit: the weight scales the swivel");
   }
  }
  check(limitedCases>0,"arm limit: at least one over-limit case limited");
  // Continuity over time: the hand twist sweeps continuously through 180 deg (130 -> 230 deg about the forearm axis, 2 deg per frame
  // at 60 fps) inside a weight ramp in / out. The stateless rule changes the swivel side when the twist passes 180 deg (an elbow
  // pop); the stateful rule (SwivelState) moves the unweighted swivel by at most kReloadSwivelRateDegPerSec * dt per frame, never
  // changes sign between two frames, never raises the pronation / bend past max(limit, before) and ends at 0 deg with the weight.
  for(const gik::ArmChain* arm:{&rig.left,&rig.right}){
   auto pose=std::make_unique<ov::Pose>(*base);
   {ov::ForearmTwistRig tw;tw.Build(sk,rig.left.lower,rig.left.hand,rig.right.lower,rig.right.hand);float moved[2]={};ov::MoveForearmTwistToHands(tw,*pose,moved);}
   pose->b[arm->lower].r=math::QuatNormalize(math::QuatMul(pose->b[arm->lower].r,ov::AxisAngleQuat(math::Vec3{0,0,1},-40.0f)));
   const int frames=60;
   const int64_t tps=60000;
   reload_contacts_runtime::SwivelState state;
   auto out=std::make_unique<ov::Pose>();
   float prevStateless=0.0f,prevU=0.0f,prevApplied=0.0f,prevW=0.0f,maxStep=0.0f,maxApplied=0.0f;
   int flipsStateless=0,flips=0,raised=0,raisedUnreported=0,overCap=0,stepOver=0,appliedOver=0;
   float maxRaise=0.0f;
   bool endUntouched=false;
   for(int i=0;i<frames;++i){
    const float w=std::fmin(1.0f,std::fmin(float(i)/10.0f,float(frames-1-i)/10.0f));
    const float twistDeg=130.0f+2.0f*float(i);
    pose->b[arm->hand].r=ov::AxisAngleQuat(math::Vec3{1,0,0},twistDeg);
    *out=*pose;
    const auto rs=LimitArmBySwivel(*out,sk,arm->upper,arm->lower,arm->hand,reload_contacts_runtime::kReloadPronationLimitDeg,
                                   reload_contacts_runtime::kReloadWristBendLimitDeg,reload_contacts_runtime::kReloadMaxSwivelDeg,1.0f);
    if(rs.swivelDeg*prevStateless<0.0f)++flipsStateless;
    prevStateless=rs.swivelDeg;
    const float step=state.Begin(1,int64_t(i+1)*(tps/60),tps);
    *out=*pose;
    const auto r=LimitArmBySwivel(*out,sk,arm->upper,arm->lower,arm->hand,reload_contacts_runtime::kReloadPronationLimitDeg,
                                  reload_contacts_runtime::kReloadWristBendLimitDeg,reload_contacts_runtime::kReloadMaxSwivelDeg,w,
                                  &state.deg[arm==&rig.left ? 0 : 1],step);
    const float u=state.deg[arm==&rig.left ? 0 : 1];
    const float limitStep=reload_contacts_runtime::kReloadSwivelRateDegPerSec/60.0f;
    if(w>0.0f && prevW>0.0f){ // weight 0 resets the unweighted state (the applied swivel is 0 there)
     maxStep=std::fmax(maxStep,std::fabs(u-prevU));
     stepOver+=std::fabs(u-prevU)>limitStep+1e-3f;
    }
    if(u*prevU<0.0f)++flips;
    // applied (weighted) step: the unweighted step plus the swivel carried by the weight change
    const float applied=r.swivelDeg;
    maxApplied=std::fmax(maxApplied,std::fabs(applied-prevApplied));
    appliedOver+=std::fabs(applied-prevApplied)>limitStep+std::fabs(prevU)*std::fabs(w-prevW)+1e-3f;
    {
     const float raise=std::fmax(r.bendAfterDeg-std::fmax(reload_contacts_runtime::kReloadWristBendLimitDeg,r.bendBeforeDeg),
                                 r.pronationAfterDeg-std::fmax(reload_contacts_runtime::kReloadPronationLimitDeg,r.pronationBeforeDeg));
     const bool up=raise>0.01f;
     raised+=up;raisedUnreported+=up && !r.overCap;overCap+=r.overCap;
     if(up)maxRaise=std::fmax(maxRaise,raise);
    }
    if(i==frames-1)endUntouched=!r.applied && std::memcmp(out.get(),pose.get(),sizeof(ov::Pose))==0;
    prevU=u;prevApplied=applied;prevW=w;
   }
   std::printf("swivel continuity side=%s stateless flips %d; stateful flips %d max step %.2f deg (applied %.2f) over-rate %d raised %d (max %.1f deg, overCap %d)\n",
               arm==&rig.left?"L":"R",flipsStateless,flips,maxStep,maxApplied,stepOver,raised,maxRaise,overCap);
   check(flipsStateless>0,"swivel continuity: the stateless rule flips the swivel side on this sequence (the case under test)");
   check(flips==0,"swivel continuity: the stateful swivel never changes sign between two frames");
   check(stepOver==0,"swivel continuity: unweighted swivel step <= kReloadSwivelRateDegPerSec * dt");
   check(appliedOver==0,"swivel continuity: applied step <= rate step + swivel x weight change");
   check(raisedUnreported==0,"swivel continuity: pronation / bend raised past max(limit, before) only on frames reported as overCap");
   check(endUntouched && prevU==0.0f,"swivel continuity: weight 0 at the end = untouched pose and swivel 0");
  }
 }
 std::printf("Native reload runtime endpoint %d passed %d failed; no intermediate clip/AO/helper proof\n",passed,failed);return failed?1:0;
}

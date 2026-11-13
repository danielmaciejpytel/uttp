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
 std::printf("Native reload runtime endpoint %d passed %d failed; no intermediate clip/AO/helper proof\n",passed,failed);return failed?1:0;
}

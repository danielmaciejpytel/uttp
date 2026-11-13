// Targeted regression against the pre-adaptive accepted algorithm and actual composition.
#include "../src/mm/MotionPose.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
using namespace utpp;using namespace utpp::mm;
namespace utpp::mm::femshoulder_legacy
{
struct Result
{
    bool applied = false;
    gik::Result left{}, right{};
    float handError = 0.0f, weaponError = 0.0f;
};
inline bool SameContact(const ov::Skeleton& sk, const ov::Pose& pose, int bone,
                        const ov::Quat& rotation, ov::Vec3 position, float& error)
{
    if (bone < 0) return true;
    ov::Quat q; ov::Vec3 p;
    gik::ModelOf(sk, pose, bone, q, p);
    error = std::fmax(error, gik::Len(p - position));
    const float dot = std::fabs(q.x * rotation.x + q.y * rotation.y + q.z * rotation.z + q.w * rotation.w);
    return error <= 1e-4f && std::isfinite(dot) && dot >= 1.0f - 1e-6f;
}
inline bool Apply(const ov::Skeleton& sk, ov::Pose& pose, ov::Pose& scratch, const gik::Rig& rig,
                  int spine, int shoulderL, int shoulderR, float weight, const gik::Params& params, Result& result)
{
    result = Result{};
    const float w = ov::Clamp01(weight);
    if (!std::isfinite(weight) || &pose == &scratch) return false;
    if (w <= ov::kZeroWeight) return true;
    if (!sk.Valid() || pose.n != sk.n || rig.weaponLeft >= sk.n || !rig.Valid(sk) ||
        shoulderL == shoulderR || rig.left.hand == rig.right.hand ||
        spine < 0 || spine >= sk.n || shoulderL < 0 || shoulderR < 0 || shoulderL >= sk.n || shoulderR >= sk.n ||
        sk.parent[shoulderL] != spine || sk.parent[shoulderR] != spine ||
        sk.parent[rig.left.upper] != shoulderL || sk.parent[rig.right.upper] != shoulderR) return false;
    ov::Vec3 lateral;
    if (!gik::Norm(pose.b[shoulderR].t - pose.b[shoulderL].t, lateral)) return false;
    // Some native action poses cross the clavicles. Leave their sockets unchanged.
    // The clavicle roots themselves are never translated by this pass.
    ov::Quat spineRot, upperLRot, upperRRot;
    ov::Vec3 spinePos, upperLPos, upperRPos;
    gik::ModelOf(sk, pose, spine, spineRot, spinePos);
    gik::ModelOf(sk, pose, rig.left.upper, upperLRot, upperLPos);
    gik::ModelOf(sk, pose, rig.right.upper, upperRRot, upperRPos);
    const float signedWidth = math::Dot(upperRPos - upperLPos, math::QuatRotate(spineRot, lateral));
    if (!std::isfinite(signedWidth) || signedWidth <= 0.040f * w) return false;
    // ModelOf/SolveArm use rigid transforms. Fail closed instead of ignoring actual nonunit scale.
    for (int i = 0; i < sk.n; ++i)
        if (!math::IsFinite(pose.b[i].r) || !math::IsFinite(pose.b[i].t) ||
            !math::IsFinite(pose.b[i].s) || std::fabs(pose.b[i].s.x - 1.0f) > 1e-3f ||
            std::fabs(pose.b[i].s.y - 1.0f) > 1e-3f || std::fabs(pose.b[i].s.z - 1.0f) > 1e-3f) return false;
    ov::Quat handLRot, handRRot, weaponLRot, weaponRRot;
    ov::Vec3 handLPos, handRPos, weaponLPos, weaponRPos;
    gik::ModelOf(sk, pose, rig.left.hand, handLRot, handLPos);
    gik::ModelOf(sk, pose, rig.right.hand, handRRot, handRPos);
    if (rig.weaponLeft >= 0) gik::ModelOf(sk, pose, rig.weaponLeft, weaponLRot, weaponLPos);
    gik::ModelOf(sk, pose, rig.weaponRight, weaponRRot, weaponRPos);
    scratch = pose;
    // Move the distal clavicle socket, not its proximal thorax attachment.
    // Convert the same Spine3-lateral MODEL displacement through each actual shoulder basis.
    const ov::Vec3 shiftModel = math::QuatRotate(spineRot, lateral) * (0.020f * w);
    ov::Quat shoulderLRot, shoulderRRot;
    ov::Vec3 shoulderLPos, shoulderRPos;
    gik::ModelOf(sk, pose, shoulderL, shoulderLRot, shoulderLPos);
    gik::ModelOf(sk, pose, shoulderR, shoulderRRot, shoulderRPos);
    scratch.b[rig.left.upper].t = scratch.b[rig.left.upper].t +
        math::QuatRotate(math::QuatInv(shoulderLRot), shiftModel);
    scratch.b[rig.right.upper].t = scratch.b[rig.right.upper].t -
        math::QuatRotate(math::QuatInv(shoulderRRot), shiftModel);
    if (!gik::SolveArm(sk, scratch, rig.left, handLPos, &handLRot, 1.0f, params, result.left) ||
        !result.left.ok || result.left.clamped ||
        !gik::SolveArm(sk, scratch, rig.right, handRPos, &handRRot, 1.0f, params, result.right) ||
        !result.right.ok || result.right.clamped) return false;
    if (!SameContact(sk, scratch, rig.left.hand, handLRot, handLPos, result.handError) ||
        !SameContact(sk, scratch, rig.right.hand, handRRot, handRPos, result.handError) ||
        !SameContact(sk, scratch, rig.weaponLeft, weaponLRot, weaponLPos, result.weaponError) ||
        !SameContact(sk, scratch, rig.weaponRight, weaponRRot, weaponRPos, result.weaponError)) return false;
    pose = scratch;
    result.applied = true;
    return true;
}
} // namespace utpp::mm::femshoulder_legacy

int main(int argc,char**argv){
 if(argc!=4)return 2;std::string error;auto db=std::make_unique<motion::MotionDatabase>();auto ovf=std::make_unique<ov::OverlayFile>();
 if(!db->LoadFromFile(argv[1],motion::kRigIdentityWoman,error)||!ovf->LoadFromFile(argv[2],ov::kRigIdentityWomanOv,error))return 2;
 std::ifstream stream(argv[3]);std::stringstream text;text<<stream.rdbuf();ov::StyleFamilyData style;if(!style.Parse(text.str(),&error))return 2;
 auto ev=std::make_unique<motionpose::PoseEvaluator>();if(!ev->Init(db.get(),ovf.get(),&style,2,error))return 2;
 const auto& sk=ovf->Skel();const auto rig=ev->GripRig();auto find=[&](const char* n){for(int i=0;i<sk.n;++i)if(ovf->BoneName(i)==n)return i;return -1;};
 int pass=0,fail=0;auto check=[&](bool ok,const char*n){(ok?pass:fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",n);};
 gik::Params params;params.softStart=.995f;params.maxReach=.999f;
 for(auto grip:{ov::PoseOverlay::Pistol1H,ov::PoseOverlay::Pistol2H})for(auto aim:{ov::AimState::Ready,ov::AimState::Aiming}){
  const int p=ovf->FindOverlayPose(grip,aim,false,false);if(p<0)return 2;ov::Pose raw;ovf->FramePose(ovf->PoseAt(p).firstFrame,raw);
  ov::Pose original=raw,legacyScratch,adaptive=raw,scratch;femshoulder_legacy::Result before;femshoulder::Result after;
  bool old=femshoulder_legacy::Apply(sk,original,legacyScratch,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,params,before);
  bool ok=femshoulder::Apply(sk,adaptive,scratch,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,params,after,true);
  std::printf("RAW grip=%d aim=%d old=%d new=%d requested=%.6f applied=%.6f handMM=%.6f weaponMM=%.6f\n",int(grip),int(aim),old,ok,after.requestedWeight,after.appliedWeight,after.handError*1000,after.weaponError*1000);
  check(ok&&after.applied&&after.appliedWeight>0&&after.appliedWeight<=1&&after.handError<=.0001f&&after.weaponError<=.0001f,"nonzero bounded socket shift preserves contacts");
  {ov::Pose optOut=raw,tmp;femshoulder::Result r;const bool defaultOK=femshoulder::Apply(sk,optOut,tmp,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,params,r);
   check(defaultOK==old&&std::memcmp(&optOut,&original,sizeof(raw))==0,"default adaptive opt-out preserves historical acceptance and pose");}
  if(old)check(std::memcmp(&original,&adaptive,sizeof(raw))==0&&after.appliedWeight==1,"previous fully accepted pose remains byte-identical");
  else check(after.appliedWeight<1,"reach-limited base uses smaller common bilateral shift");
  bool continuous=true;math::Quat qq;math::Vec3 previousLeft,previousRight;
  gik::ModelOf(sk,raw,rig.left.upper,qq,previousLeft);gik::ModelOf(sk,raw,rig.right.upper,qq,previousRight);
  for(int tick=1;tick<=200;++tick){
   const float requested=float(tick<=100?tick:200-tick)/100;
   ov::Pose frame=raw,tmp;femshoulder::Result r;const bool accepted=femshoulder::Apply(sk,frame,tmp,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),requested,params,r,true);
   math::Vec3 left,right;gik::ModelOf(sk,frame,rig.left.upper,qq,left);gik::ModelOf(sk,frame,rig.right.upper,qq,right);
   continuous=continuous&&accepted&&gik::Len(left-previousLeft)<.00025f&&gik::Len(right-previousRight)<.00025f&&r.handError<=.0001f&&r.weaponError<=.0001f;
   previousLeft=left;previousRight=right;
  }
  check(continuous,"0to1to0 request sweep has bounded continuous sockets and exact contacts");
  ov::Pose sentinel=raw;const auto snapshot=sentinel;femshoulder::Result bad;params.softStart=std::numeric_limits<float>::quiet_NaN();
  check(!femshoulder::Apply(sk,sentinel,scratch,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,params,bad,true)&&std::memcmp(&sentinel,&snapshot,sizeof(raw))==0,"invalid params fail closed without pose writes");params.softStart=.995f;
 }
 int idle=-1;for(uint32_t i=0;i<db->NumClips();++i)if(db->ClipName(i)=="M_Neutral_Stand_Idle_Loop")idle=int(i);if(idle<0)return 2;
 for(bool aim:{false,true}){
  ev->Reset();motionpose::OverlayFrameInputs in;in.bodyWoman=true;in.machine.style=ov::Style::Feminine;in.machine.overlay=ov::PoseOverlay::Pistol1H;in.machine.wantsReady=true;in.machine.wantsToAim=aim;in.capturePoseDiagnostics=true;in.gripSkip=true;
  motion::MatchResult match;match.clip=idle;match.clipTime=.5f;std::vector<BoneTransform> output(db->NumBones());motionpose::FrameInfo info;bool valid=true;
  for(int frame=0;frame<240;++frame)valid=ev->Evaluate(1.f/60,match,in,output.data(),uint32_t(output.size()),&info)&&valid;
  std::printf("COMPOSED aim=%d femUpperLMM=%.6f femUpperRMM=%.6f\n",aim,info.feminineDelta.posM[4]*1000,info.feminineDelta.posM[5]*1000);
  if(info.feminineDelta.posM[4]==0){ov::Pose base,scratch;ev->ToOv(output.data(),base);femshoulder::Result r;bool ok=femshoulder::Apply(sk,base,scratch,rig,find("Spine3"),find("LeftShoulder"),find("RightShoulder"),1,params,r,true);std::printf("DIAG retry=%d Lclamp%d Rclamp%d reqUsedL%.9f/%.9f reqUsedR%.9f/%.9f hand%.9f weapon%.9f\n",ok,r.left.clamped,r.right.clamped,r.left.reqDist,r.left.useDist,r.right.reqDist,r.right.useDist,r.handError,r.weaponError);}
  // 2026-11-02: weights are per side (each arm takes what its own reach window admits), so the two sockets may differ; each stays within the
  // configured amount (motionpose::kFeminineShoulderAmount, default 25 mm) and both still move.
  check(valid&&info.feminineDelta.posM[4]>.0001f&&info.feminineDelta.posM[5]>.0001f&&info.feminineDelta.posM[4]<=ev->FeminineShoulderAmount()+.00001f&&info.feminineDelta.posM[5]<=ev->FeminineShoulderAmount()+.00001f&&info.fem.applied,"actual composed1H Ready/Aim applies bilateral bounded feminine posture");
 }
 // ---- 2026-11-09: Mod Settings "Shoulder width (female V)" -5 .. +5 cm = 5 .. 0 cm per side -------------------------
 // (1) cm -> m conversion of the setting: exact on the 0.25 cm per side grid; 2.5 cm == the shipped default bit for bit
 {
  bool exact=true;
  for(int k=0;k<=20;++k)exact=exact&&femshoulder::AmountFromCm(0.25f*float(k))==static_cast<float>(k*0.0025);
  std::printf("K5 CONV AmountFromCm(2.5)=%.9g kFeminineShoulderAmount=%.9g naive 2.5f*0.01f=%.9g\n",double(femshoulder::AmountFromCm(2.5f)),double(motionpose::kFeminineShoulderAmount),double(2.5f*0.01f));
  check(exact&&femshoulder::AmountFromCm(2.5f)==motionpose::kFeminineShoulderAmount&&femshoulder::AmountFromCm(femshoulder::kMaxAmountCm)==0.05f&&femshoulder::AmountFromCm(0)==0,
        "K5 cm->m conversion exact on the 0.25 cm grid (2.5 cm == shipped default, 5 cm == 0.05 m)");
 }
 struct K5Cfg{const char* name;ov::PoseOverlay overlay;bool ready,aim;};
 const K5Cfg k5cfg[]={{"Default",ov::PoseOverlay::Default,false,false},{"P1H-Ready",ov::PoseOverlay::Pistol1H,true,false},{"P1H-Aim",ov::PoseOverlay::Pistol1H,true,true},
  {"P2H-Ready",ov::PoseOverlay::Pistol2H,true,false},{"P2H-Aim",ov::PoseOverlay::Pistol2H,true,true},{"Rifle-Ready",ov::PoseOverlay::Rifle,true,false},{"Rifle-Aim",ov::PoseOverlay::Rifle,true,true}};
 const char* k5clips[]={"M_Neutral_Stand_Idle_Loop","M_Neutral_Walk_Loop_F","M_Neutral_Run_Loop_F","M_Neutral_Crouch_Idle_Loop"};
 auto k5inputs=[&](const K5Cfg& c,std::string_view clip){motionpose::OverlayFrameInputs in;in.bodyWoman=true;in.machine.style=ov::Style::Feminine;in.machine.overlay=c.overlay;
  in.machine.wantsReady=c.ready;in.machine.wantsToAim=c.aim;in.capturePoseDiagnostics=false;in.gripSkip=true;
  in.authoredTwoHandGrip=c.overlay==ov::PoseOverlay::Pistol2H||c.overlay==ov::PoseOverlay::Rifle;in.machine.crouch=clip.find("Crouch")!=std::string_view::npos;
  in.machine.moving=clip.find("Idle")==std::string_view::npos;in.machine.hasVelocity=in.machine.moving;return in;};
 auto clipIndex=[&](const char* n){for(uint32_t i=0;i<db->NumClips();++i)if(db->ClipName(i)==n)return int(i);return -1;};
 // (2) default identity: an evaluator set to AmountFromCm(2.5) (the setting at 0 after any round trip) == one never set, byte for byte
 {
  auto evSet=std::make_unique<motionpose::PoseEvaluator>();if(!evSet->Init(db.get(),ovf.get(),&style,2,error))return 2;
  evSet->SetFeminineShoulders(femshoulder::AmountFromCm(2.5f));
  bool same=true;int frames=0,femRan=0;
  for(const auto& c:k5cfg)for(const char* clip:k5clips){
   const int ci=clipIndex(clip);if(ci<0){same=false;continue;}
   ev->Reset();evSet->Reset();auto in=k5inputs(c,clip);motion::MatchResult match;match.clip=ci;
   std::vector<BoneTransform> a(db->NumBones()),b(db->NumBones());motionpose::FrameInfo ia,ib;
   for(int f=0;f<120;++f){match.clipTime=float(f)/60.0f;
    const bool okA=ev->Evaluate(1.f/60,match,in,a.data(),uint32_t(a.size()),&ia),okB=evSet->Evaluate(1.f/60,match,in,b.data(),uint32_t(b.size()),&ib);
    same=same&&okA==okB&&std::memcmp(a.data(),b.data(),sizeof(BoneTransform)*a.size())==0;++frames;femRan+=ia.fem.applied;}
  }
  std::printf("K5 DEFAULT frames=%d femApplied=%d identical=%d\n",frames,femRan,same);
  check(same&&femRan>0,"K5 width 0 (2.5 cm per side via the setting) gives byte-identical output to the untouched default");
 }
 // (3) 5 cm per side (width -5) on composed poses: the crossed margin and the reach window bound the pass; contacts stay exact,
 //     no clamped arm solve, a pass that cannot run leaves the pose untouched. Base = the evaluator output without narrowing.
 {
  auto evOff=std::make_unique<motionpose::PoseEvaluator>();if(!evOff->Init(db.get(),ovf.get(),&style,2,error))return 2;
  evOff->SetFeminineShoulders(0.0f);
  const int spine=find("Spine3"),shL=find("LeftShoulder"),shR=find("RightShoulder");
  gik::Params cp;cp.softStart=motionpose::kFeminineSoftStart;cp.maxReach=motionpose::kFeminineMaxReach;
  int n=0,applied=0,partial=0,reasons[8]={},bad=0;float maxErr=0,minAfter=1e9f,maxNarrow=0,sumNarrow=0,maxFlex=0,maxMove=0;
  for(const auto& c:k5cfg)for(const char* clip:k5clips){
   const int ci=clipIndex(clip);if(ci<0){++bad;continue;}
   evOff->Reset();auto in=k5inputs(c,clip);motion::MatchResult match;match.clip=ci;std::vector<BoneTransform> out(db->NumBones());motionpose::FrameInfo info;
   for(int f=0;f<180;++f){match.clipTime=float(f)/60.0f;
    if(!evOff->Evaluate(1.f/60,match,in,out.data(),uint32_t(out.size()),&info)){++bad;continue;}
    if(f%6)continue;
    ov::Pose base,pose,tmp;evOff->ToOv(out.data(),base);pose=base;femshoulder::Result r;femshoulder::Options opt;opt.amount=femshoulder::AmountFromCm(femshoulder::kMaxAmountCm);
    const bool ok=femshoulder::ApplyContinuous(sk,pose,tmp,rig,spine,shL,shR,1.0f,cp,r,opt);++n;++reasons[std::min(int(r.reason),7)];
    if(!r.applied){if(std::memcmp(&pose,&base,sizeof(base))!=0)++bad;continue;}
    ++applied;partial+=r.weight[0]<.999f||r.weight[1]<.999f;
    const float narrow=r.widthBefore-r.widthAfter;
    maxErr=std::max(maxErr,std::max(r.handError,r.weaponError));minAfter=std::min(minAfter,r.widthAfter);maxNarrow=std::max(maxNarrow,narrow);sumNarrow+=narrow;
    maxFlex=std::max(maxFlex,std::max(std::fabs(r.elbowFlexDeg[0]),std::fabs(r.elbowFlexDeg[1])));maxMove=std::max(maxMove,std::max(r.elbowMove[0],r.elbowMove[1]));
    if(!ok||r.handError>1e-4f||r.weaponError>1e-4f||r.left.clamped||r.right.clamped||!(r.weight[0]>=0&&r.weight[0]<=1&&r.weight[1]>=0&&r.weight[1]<=1)||
       narrow>2*opt.amount+1e-5f||(r.widthBefore>=opt.crossedMargin&&r.widthAfter<opt.crossedMargin-1e-5f))++bad;
    for(int i=0;i<sk.n;++i)if(!math::IsFinite(pose.b[i].r)||!math::IsFinite(pose.b[i].t))++bad;
   }
  }
  std::printf("K5 5CM n=%d applied=%d partial=%d bad=%d maxContactMM=%.4f minWidthAfterCM=%.2f narrowCM[mean=%.2f max=%.2f] elbow[maxMoveMM=%.1f maxFlexDeg=%.1f] reasons",
              n,applied,partial,bad,maxErr*1000,minAfter*100,applied?sumNarrow*100/applied:0.0,maxNarrow*100,maxMove*1000,maxFlex);
  for(int i=0;i<8;++i)std::printf(" %d",reasons[i]);std::puts("");
  check(n>0&&applied*2>n&&bad==0,"K5 5 cm per side on composed poses: contacts exact, no clamped arm, crossed margin kept, failures leave the pose untouched");
 }
 // (4) narrow and crossed sockets at 5 cm per side: the weight is capped so the sockets keep crossedMargin; crossed sockets are left alone
 {
  const int spine=find("Spine3"),shL=find("LeftShoulder"),shR=find("RightShoulder");
  const int p=ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H,ov::AimState::Ready,false,false);if(p<0)return 2;
  ov::Pose raw;ovf->FramePose(ovf->PoseAt(p).firstFrame,raw);
  gik::Params cp;cp.softStart=motionpose::kFeminineSoftStart;cp.maxReach=motionpose::kFeminineMaxReach;
  femshoulder::Options opt;opt.amount=femshoulder::AmountFromCm(femshoulder::kMaxAmountCm);
  // sockets moved inward (the pass's own displacement) until the width is `target`
  auto squeeze=[&](float target){ov::Pose q=raw;ov::Vec3 lateral;gik::Norm(q.b[shR].t-q.b[shL].t,lateral);math::Quat sr,lr,rr;math::Vec3 sp,lp,rp;
   gik::ModelOf(sk,q,spine,sr,sp);const ov::Vec3 lm=math::QuatRotate(sr,lateral);const float w0=femshoulder::SocketWidth(sk,q,rig,spine,shL,shR);
   const ov::Vec3 shift=lm*((w0-target)*0.5f);gik::ModelOf(sk,q,shL,lr,lp);gik::ModelOf(sk,q,shR,rr,rp);
   q.b[rig.left.upper].t=q.b[rig.left.upper].t+math::QuatRotate(math::QuatInv(lr),shift);q.b[rig.right.upper].t=q.b[rig.right.upper].t-math::QuatRotate(math::QuatInv(rr),shift);return q;};
  ov::Pose narrowPose=squeeze(opt.crossedMargin+0.01f),tmp;femshoulder::Result r;
  femshoulder::ApplyContinuous(sk,narrowPose,tmp,rig,spine,shL,shR,1.0f,cp,r,opt);
  std::printf("K5 NARROW before=%.2fcm after=%.2fcm applied=%d wL=%.3f wR=%.3f reason=%d\n",r.widthBefore*100,r.widthAfter*100,r.applied,r.weight[0],r.weight[1],int(r.reason));
  check(r.widthBefore<opt.crossedMargin+0.0101f&&r.weight[0]<=0.1001f&&r.weight[1]<=0.1001f&&r.widthAfter>=opt.crossedMargin-1e-5f&&r.handError<=1e-4f&&r.weaponError<=1e-4f,
        "K5 narrow sockets at 5 cm per side: weight capped (<= 0.1), crossed margin kept, contacts exact");
  ov::Pose crossed=squeeze(-0.01f);const ov::Pose snap=crossed;femshoulder::Result rc;
  const bool okC=femshoulder::ApplyContinuous(sk,crossed,tmp,rig,spine,shL,shR,1.0f,cp,rc,opt);
  check(!okC&&rc.reason==femshoulder::Reason::Crossed&&std::memcmp(&crossed,&snap,sizeof(snap))==0,"K5 crossed sockets at 5 cm per side: pass refused, pose untouched");
 }
 std::printf("Feminine adaptive %dPASS %dFAIL\n",pass,fail);return fail?1:0;
}

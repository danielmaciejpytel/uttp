#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>
#include "../src/mm/HipFirePose.hpp"
namespace {
bool rootContextSeen=false,providedRootSeen=false;uint64_t rootSeqSeen=0;float capturedRootYaw=0,capturedAnimationRate=0;
int turnLegBones[4]={-1,-1,-1,-1};
std::array<mm::math::Quat,4> turnLegPose{};
bool CameraStep(const motionhost::FeedInput&in,uint32_t gate,float dt){rootContextSeen|=in.haveRootContext&&in.nativeRootEnabled;rootSeqSeen=in.rootFeedSequence;return WStep(in,gate,dt);}
bool CameraFrame(const int32_t*map,uint32_t n,motionhost::MotionSetFrame&f){bool ok=motiondriver::Frame(map,n,f);providedRootSeen|=ok&&f.providedRoot&&f.rootFeedSequence==rootSeqSeen;if(ok&&f.providedRoot){capturedRootYaw=f.rootOffsetYaw;capturedAnimationRate=f.rootYawRate;for(int i=0;i<4;++i)if(turnLegBones[i]>=0&&uint32_t(turnLegBones[i])<f.numBones)turnLegPose[i]=f.pose[turnLegBones[i]].q;}return ok;}
float Ao(){return std::strtof(StatusField(motiondriver::Status(),"aoYaw=").c_str(),nullptr);}
struct OwnerFrame{float capsuleDeg;int ads,shot;};
#include "data/recording_1215_owner_replay.inc"
}
int main(int argc,char**argv){if(argc!=3){std::puts("usage CameraTargetRootTest <driver-data> <basedb>");return 2;}
 Rig rig;std::unique_ptr<mm::motion::MotionDatabase>db;if(!RigFromDb(DataPaths(argv[1],"woman").files.db,rig,&db))return 3;g_baseDb=std::wstring(argv[2],argv[2]+std::strlen(argv[2]));
 const char* turnLegNames[]={"LeftUpLeg","LeftLeg","RightUpLeg","RightLeg"};
 for(uint32_t b=0;b<db->NumBones();++b)for(int i=0;i<4;++i)if(db->BoneName(b)==turnLegNames[i])turnLegBones[i]=int(b);
 Check(std::all_of(std::begin(turnLegBones),std::end(turnLegBones),[](int b){return b>=0;}),"turn footwork fixture resolves both thigh/shin chains");
 motiondriver::SetPathsOverride(0,DataPaths(argv[1],"woman"));motiondriver::SetRigBody(0);motiondriver::SetOverlayInputs(3,1,false,false);motiondriver::SetAimGameUpper(false);motiondriver::SetWeaponUpper(false);motiondriver::SetGripEnabled(true);
 const motionhost::MotionHooks hooks{&CameraStep,&WApply,&motiondriver::OnArmed,&motiondriver::OnDisarmed,&CameraFrame,&motiondriver::OwnsTraversal};motionhost::SetMotionHooks(&hooks);
 Check(ResetBase(rig),"real base loaded");motionhost::SetOffsetRoot(true);motionhost::SetWeaponUpper(false);motionhost::SetHoldPose(true);motiondriver::SetEnabled(true);motiondriver::Step({},0,1.f/60);motiondriver::WaitLoader();
 motionhost::FeedInput f;f.dt=1.f/60;f.tppActive=true;f.locoState=integ::kLocoCrouch;f.velWorld[1]=2.7f;f.move[1]=1;f.haveAir=true;f.onGround=true;f.distToGround=0;f.psmBlock=integ::kPsmWeaponKnown|integ::kPsmWeaponDrawn;
 auto feed=[&](int n,const float*camera,bool ads=false){for(int j=0;j<n;++j){f.psmBlock=integ::kPsmWeaponKnown|integ::kPsmWeaponDrawn|(ads?(uint32_t(6+1)<<13):0);if(camera)motiondriver::SetCameraTarget(true,true,camera);motiondriver::FeedAirFromInput(f);g_log={};motionhost::Feed(f);}return g_log.stepResult;};
 const float right[3]={-30,60,1.4f},left[3]={30,60,1.4f};motiondriver::SetAimTarget(false,false,nullptr);motiondriver::SetWeaponAction(8);
 Check(feed(180,right),"hipfire2H crouchmoving owned");const float hipRight=Ao();Check(rootContextSeen&&providedRootSeen,"real host samefeed root context and provided frame");Check(hipRight>10,"hipfire camera ray reaches visual-root AO");
 Check(feed(90,left),"opposite hipfire ray owned");const float hipLeft=Ao();Check(hipLeft< -10&&hipRight-hipLeft>25,"hipfire AO follows camera direction both sides");
 motiondriver::SetWeaponAction(0);Check(feed(90,right,true),"ADS crouchmoving owned");Check(Ao()>10,"ADS keeps camera target routing");
 motiondriver::SetAimTarget(true,true,left);feed(30,right,true);Check(Ao()< -10,"legacy ADS target takes priority over camera ray");motiondriver::SetAimTarget(false,false,nullptr);
 // A lost target no longer ends the AO in one frame (fix2/ao 7d231a8): the last angles are latched and faded out within
 // the latch window (<= 0.6 s), so a stale or expired target is never tracked and the AO is gone after the window.
 auto noTrackThenGone=[&](float before,int n){feed(3,nullptr,true);const bool held=std::fabs(Ao())<=std::fabs(before)+.1f;feed(n,nullptr,true);return held&&std::fabs(Ao())<.1f;};
 feed(30,right,true);Check(Ao()>10,"camera ray resumes when legacy target disabled");Sleep(300);Check(noTrackThenGone(Ao(),45),"camera TTL expires without refresh");
 feed(3,right,true);Check(Ao()>10,"fresh heartbeat restores AO after TTL");motiondriver::SetCameraTarget(false,false,nullptr);Check(noTrackThenGone(Ao(),45),"explicit target disable invalidates AO");
 feed(3,right,true);f.tppActive=false;feed(1,nullptr,true);f.tppActive=true;feed(30,nullptr,true);Check(std::fabs(Ao())<.1f,"FPP invalidates camera target until new heartbeat");
 const float invalid[3]={std::numeric_limits<float>::quiet_NaN(),60,1.4f};motiondriver::SetCameraTarget(true,true,invalid);feed(2,nullptr,true);Check(std::fabs(Ao())<.1f,"invalid camera point fails closed");
 // Raw aim PSM without an actual ranged overlay must not release root rotation.
 motiondriver::SetAimGameUpper(true);motiondriver::SetWeaponAction(0);
 f.move[0]=f.move[1]=0;f.velWorld[0]=f.velWorld[1]=f.velWorld[2]=0;
 f.locoState=integ::kLocoStand;f.playerYawDeg=0;
 auto rootReleaseCase=[&](int overlay,bool ads=true,bool nativeIdleUpper=false){
  motiondriver::SetOverlayInputs(overlay,1,false,false);f.playerYawDeg=0;
  Check(ResetBase(rig),"root release regression resets real host");
  motionhost::SetOffsetRoot(true);motionhost::SetHoldPose(true);motionhost::SetWeaponUpper(nativeIdleUpper);motiondriver::SetWeaponUpper(nativeIdleUpper);
  feed(45,nullptr,ads);capturedRootYaw=0;
  for(int j=0;j<12;++j){f.playerYawDeg+=10;feed(1,nullptr,ads);}
  return capturedRootYaw;
 };
 const float defaultAimRoot=rootReleaseCase(0),pistolAimRoot=rootReleaseCase(3);
 const float nativeIdleUpperRoot=rootReleaseCase(3,false,true);
 Check(std::fabs(defaultAimRoot)>.5f,"Default overlay raw Aim retains nonzero accumulated visual yaw");
 Check(std::fabs(pistolAimRoot)<.02f,"Pistol ADS native-upper ownership releases visual yaw");
 Check(std::fabs(nativeIdleUpperRoot)>.5f,"ordinary drawn weapon native idle upper keeps MM root rotation active");
 // Stationary aim must turn using the authored leg animation. A visual root
 // rigidly following a rotated capsule is not a successful turn-in-place.
 // Keep camera point and camera yaw consistent in every real-host scenario.
 const float forward[3]={0,60,1.4f};
 constexpr uint32_t upperAim=uint32_t(6+1)<<integ::kPsmUpperShift;
 auto rawFeed=[&](bool heartbeat){
  if(heartbeat){const float yaw=f.cameraYawDeg*integ::kDegToRad;const float point[3]={-std::sin(yaw)*60,std::cos(yaw)*60,1.4f};motiondriver::SetCameraTarget(true,true,point);}
  motiondriver::FeedAirFromInput(f);g_log={};providedRootSeen=false;capturedAnimationRate=0;motionhost::Feed(f);
  return g_log.stepResult&&providedRootSeen;
 };
 auto seedVisualOffset=[&](int stance,bool seedOffset=true){
  motiondriver::SetAimTarget(false,false,nullptr);motiondriver::SetCameraTarget(false,false,nullptr);
  motiondriver::SetAimGameUpper(false);motiondriver::SetWeaponAction(0);
  motiondriver::SetOverlayInputs(0,0,false,false);
  Check(ResetBase(rig),"aim-facing real host reset");
  motionhost::SetOffsetRoot(true);motionhost::SetHoldPose(true);
  motionhost::SetWeaponUpper(false);motiondriver::SetWeaponUpper(false);
  f={};f.dt=1.f/60;f.tppActive=true;f.locoState=stance;
  f.haveAir=true;f.onGround=true;f.distToGround=0;
  f.psmBlock=integ::kPsmWeaponKnown;
  // Finish Stand_to_Crouch before testing a crouching idle turn.
  for(int j=0;j<180;++j)rawFeed(false);
  if(seedOffset){
   for(int j=0;j<30;++j){f.playerYawDeg+=157.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   Check(providedRootSeen&&std::fabs(capturedRootYaw)>2.4f,"fixture reproduces large visual-root offset");
  }else Check(providedRootSeen&&std::fabs(capturedRootYaw)<.001f,"stationary turn fixture starts aligned after stance settles");
  return capturedRootYaw;
 };
 auto setIntent=[&](int overlay,bool ads,bool hipfire,bool drawn,bool known=true){
  motiondriver::SetOverlayInputs(overlay,ads?2:1,false,false);
  motiondriver::SetWeaponAction(hipfire?8:0);
  f.psmBlock=(known?integ::kPsmWeaponKnown:0)|(drawn?integ::kPsmWeaponDrawn:0)|(ads?upperAim:0);
 };
 struct FacingCase{const char*name;int overlay;bool ads,hipfire,drawn;};
 const FacingCase facingCases[]={{"unarmed RMB",0,true,false,false},
  {"owned pistol2H ADS",3,true,false,true},{"pistol2H hipfire",3,false,true,true},
  {"owned pistol1H ADS",2,true,false,true},{"rifle hipfire",1,false,true,true},
  {"pistol1H hipfire",2,false,true,true},{"owned rifle ADS",1,true,false,true}};
 for(int stance:{integ::kLocoStand,integ::kLocoCrouch})for(const auto&c:facingCases)for(float target:{90.f,-90.f,170.f,-170.f}){
  if(!c.ads&&std::fabs(target)>90.f)continue; // separate single/burst hipfire matrix below
  const std::string name=std::string(stance==integ::kLocoStand?"standing ":"crouching ")+c.name+(target>0?" left":" right");
  seedVisualOffset(stance,false);setIntent(c.overlay,c.ads,c.hipfire,c.drawn);f.cameraYawDeg=target;
  bool framesOwned=true,finite=true,correctFamily=true;int tipFrames=0,directedTipFrames=0,firstTip=-1;
  float authored=0,visual=0,maxUnexplainedYaw=0,progress1=0,progress2=0,maxLegMotion=0;
  float time15=-1,time10=-1,maxVisualStep=0,previousVisual=0;
  const auto initialLegs=turnLegPose;
  int shortTipStarts=0,shortTipRestarts=0,previousClip=-1;float previousTime=-1;
  for(int j=0;j<240;++j){
   // BodyTurn physically aligns the capsule over .5s; the camera already
   // targets that direction. Visual yaw must come from the selected animation.
   f.playerYawDeg=target*std::min(1.f,(j+1)/30.f);
   framesOwned=rawFeed(true)&&framesOwned;
   authored+=capturedAnimationRate*f.dt/integ::kDegToRad;
   visual=std::remainder(f.playerYawDeg+capturedRootYaw/integ::kDegToRad,360.f);
   const float remaining=std::fabs(std::remainder(target-visual,360.f));
   if(time15<0&&remaining<=15)time15=(j+1)*f.dt;
   if(time10<0&&remaining<=10)time10=(j+1)*f.dt;
   maxVisualStep=std::max(maxVisualStep,std::fabs(std::remainder(visual-previousVisual,360.f)));previousVisual=visual;
   if(j==59)progress1=visual*(target>0?1.f:-1.f);
   if(j==119)progress2=visual*(target>0?1.f:-1.f);
   for(int i=0;i<4;++i){const float dot=std::clamp(std::fabs(mm::math::QuatDot(initialLegs[i],turnLegPose[i])),0.f,1.f);maxLegMotion=std::max(maxLegMotion,2*std::acos(dot)/integ::kDegToRad);}
   finite=finite&&std::isfinite(authored)&&std::isfinite(visual);
   maxUnexplainedYaw=std::max(maxUnexplainedYaw,std::fabs(std::remainder(visual-authored,360.f)));
   const auto pick=StatusField(motiondriver::Status(),"pick=clip:");
   const int clip=std::atoi(pick.c_str());
   const auto at=pick.find('@');const float clipTime=at==std::string::npos?-1.f:std::strtof(pick.c_str()+at+1,nullptr);
   if(clip>=0&&static_cast<uint32_t>(clip)<db->NumClips()){
    const auto clipName=db->ClipName(static_cast<uint32_t>(clip));
    if(steer::ClassifyClip(clipName)==steer::kClassTurnInPlace){
     ++tipFrames;if(firstTip<0)firstTip=clip;
     directedTipFrames+=clipName.ends_with(target>0?"_L":"_R")?1:0;
     correctFamily=correctFamily&&clipName.starts_with(stance==integ::kLocoStand?"M_Neutral_Stand_Turn_":"M_Neutral_Crouch_Idle_Turn_");
     if(clipName.find("_045_")!=std::string_view::npos){
      const bool restarted=clip==previousClip&&clipTime+.04f<previousTime;
      if(clip!=previousClip||restarted)++shortTipStarts;
      if(restarted)++shortTipRestarts;
     }
    }
   }
   previousClip=clip;previousTime=clipTime;
  }
  const float error=std::fabs(std::remainder(target-visual,360.f));
  std::printf("aim-turn %s target=%.0f visual=%.2f authored=%.2f error=%.2f TIP=%d directed=%d first=%d progress1=%.2f progress2=%.2f t15=%.3f t10=%.3f maxStep=%.3f shortStarts=%d shortRestarts=%d legMotion=%.2f unexplained=%.5f deg\n",name.c_str(),target,visual,authored,error,tipFrames,directedTipFrames,firstTip,progress1,progress2,time15,time10,maxVisualStep,shortTipStarts,shortTipRestarts,maxLegMotion,maxUnexplainedYaw);
  Check(framesOwned&&finite,name+" supplies finite committed visual roots");
  Check(tipFrames>0&&directedTipFrames>0&&correctFamily,name+" selects authored turn for this stance and requested side");
  Check(authored*target>0&&visual*target>0,name+" authored and visual rotation progress toward target");
  Check(error<=mm::motiontraj::kTurnInPlaceDeg+1.f,name+" finishes within the existing Neutral turn-in-place threshold");
  Check(maxUnexplainedYaw<.05f,name+" capsule rotation cannot rotate idle body without authored yaw");
  if(c.ads){
   const bool large=std::fabs(target)>90.f;
   Check(progress1>=(large?60.f:30.f)&&progress2>=(large?120.f:60.f),name+" authored turn makes substantial progress within one and two seconds");
   Check(shortTipRestarts==0&&shortTipStarts<=1,name+" avoids repeated 045 restart loop");
   Check(maxLegMotion>5.f,name+" renders authored leg articulation instead of spinning a rigid idle");
   // Report peak speed, but do not cap authored 180-degree footwork at the
   // unrelated 540deg/s capsule-release limit. The visual==authored check above
   // is the relevant guard against rigid/synthetic body rotation.
  }
  if(c.ads&&!c.drawn){
   setIntent(0,false,false,false);f.playerYawDeg+=20;f.cameraYawDeg=f.playerYawDeg;rawFeed(true);
   Check(std::fabs(capturedRootYaw)>5*integ::kDegToRad,name+" leaving aim restores ordinary root accumulation");
  }
 }
 // Live goal changes must interrupt an admitted turn rather than completing
 // the obsolete target. This is still the real host/overlay/root route.
 for(int stance:{integ::kLocoStand,integ::kLocoCrouch}){
  seedVisualOffset(stance,false);setIntent(3,true,false,true);f.cameraYawDeg=170;
  for(int j=0;j<24;++j){f.playerYawDeg=170*std::min(1.f,(j+1)/30.f);rawFeed(true);}
  const float startVisual=std::remainder(f.playerYawDeg+capturedRootYaw/integ::kDegToRad,360.f);
  // A fixed absolute -90 goal is NOT necessarily a reversal: a fast turn may
  // already be at +169, whose shortest path to -90 is another +101 degrees.
  const float newGoal=std::remainder(startVisual-120.f,360.f);
  const float oldActor=f.playerYawDeg;f.cameraYawDeg=newGoal;
  float reverseAuthored=0,visual=startVisual;int oppositeTipFrames=0;bool owned=true;
  for(int j=0;j<180;++j){
   f.playerYawDeg=oldActor+std::remainder(newGoal-oldActor,360.f)*std::min(1.f,(j+1)/30.f);
   owned=rawFeed(true)&&owned;reverseAuthored+=capturedAnimationRate*f.dt/integ::kDegToRad;
   visual=std::remainder(f.playerYawDeg+capturedRootYaw/integ::kDegToRad,360.f);
   const int clip=std::atoi(StatusField(motiondriver::Status(),"pick=clip:").c_str());
   if(clip>=0&&uint32_t(clip)<db->NumClips()&&steer::ClassifyClip(db->ClipName(uint32_t(clip)))==steer::kClassTurnInPlace&&db->ClipName(uint32_t(clip)).ends_with("_R"))++oppositeTipFrames;
  }
  const std::string label=stance==integ::kLocoStand?"standing aim goal reversal":"crouching aim goal reversal";
  std::printf("%s from=%.2f goal=%.2f end=%.2f authored=%.2f oppositeTIP=%d\n",label.c_str(),startVisual,newGoal,visual,reverseAuthored,oppositeTipFrames);
  Check(owned&&oppositeTipFrames>0&&reverseAuthored<0&&std::fabs(std::remainder(newGoal-visual,360.f))<=51,label+" abandons old side and follows the new camera goal");
 }
 // Aim query admission uses the same camera goal as its predicted facing, not
 // an already rotated capsule. This pure adapter guard supplements real-host
 // motion/air/FPP exits exercised below without exposing new implementation fields.
 {
  adapter::MapConfig config;adapter::MapContext context;context.aimFacing=true;context.rootYawValid=true;context.rootYawDeg=0;
  adapter::MotionFeed input;input.dt=1.f/60;input.psmDetailed=adapter::kPsmStand;input.aiming=true;input.playerYawDeg=170;input.cameraYawDeg=0;
  adapter::MapState state;
  Check(!adapter::MapToChooser(input,config,state,context).shouldTurnInPlace,"camera on visual goal does not request another aim TIP while actor is elsewhere");
  input.playerYawDeg=0;input.cameraYawDeg=170;state={};
  Check(adapter::MapToChooser(input,config,state,context).shouldTurnInPlace,"stationary aim facing an off-axis goal admits TIP");
  input.move[1]=1;input.velWorld[1]=3.85f;state={};
  Check(!adapter::MapToChooser(input,config,state,context).shouldTurnInPlace,"movement leaves stationary aim TIP admission");
  input.move[1]=input.velWorld[1]=0;input.psmDetailed=adapter::kPsmFall;input.air.valid=true;input.air.onGround=false;state={};
  Check(!adapter::MapToChooser(input,config,state,context).shouldTurnInPlace,"air leaves stationary aim TIP admission");
  input.psmDetailed=adapter::kPsmStand;input.air.onGround=true;input.aiming=false;context.aimFacing=false;state={};
  Check(!adapter::MapToChooser(input,config,state,context).shouldTurnInPlace,"aim release restores ordinary actor-root TIP admission");
  // The rest turn-in-place shares the aim turn's goal fields but is not an aim START: it must not release the hold of a slide-exit clip
  // (ChooserMatcher::HoldFlags bit 0 = stationaryAimTurn && shouldTurnInPlace). Capsule 170deg away from the visible root, no aim.
  {
   adapter::MapContext rest;rest.rootYawValid=true;rest.rootYawDeg=0;rest.restTurn=true;
   adapter::MotionFeed idle;idle.dt=1.f/60;idle.psmDetailed=adapter::kPsmStand;idle.playerYawDeg=170;idle.cameraYawDeg=10;
   adapter::MapState st;
   const auto rc=adapter::MapToChooser(idle,config,st,rest);
   Check(rc.restTurn&&rc.shouldTurnInPlace&&!rc.stationaryAimTurn&&std::fabs(std::fabs(rc.stationaryAimYawDeg)-170.f)<.01f,"rest TIP is admitted as its own chooser flag with the capsule as the goal");
   Check(motion::ChooserMatcher::HoldFlags(rc)==0,"rest TIP does not release the hold of a slide-exit clip (HoldFlags)");
   adapter::MapContext aim=rest;aim.restTurn=false;aim.aimFacing=true;adapter::MotionFeed aiming=idle;aiming.aiming=true;aiming.playerYawDeg=0;aiming.cameraYawDeg=170;st={};
   const auto ac=adapter::MapToChooser(aiming,config,st,aim);
   Check(ac.stationaryAimTurn&&!ac.restTurn&&(motion::ChooserMatcher::HoldFlags(ac)&1)==1,"an aim turn still counts as a start for the slide-exit hold (HoldFlags bit 0)");
   adapter::MotionFeed shortTurn=idle;shortTurn.playerYawDeg=44;st={};
   Check(!adapter::MapToChooser(shortTurn,config,st,rest).restTurn,"rest TIP needs the 50deg admission angle");
  }
 }
 // A short shot can finish its firing pose before a large authored turn has
 // finished. Unlike the broad aim tests above, this reproduces the executor's
 // fast capsule turn and requires the body to finish facing the shot itself.
 auto shotTurnCase=[&](int stance,float target,bool burst){
  const std::string name=std::string(stance==integ::kLocoStand?"standing ":"crouching ")+
   (burst?"burst ":"single shot ")+std::to_string(static_cast<int>(target))+"deg";
  seedVisualOffset(stance,false);setIntent(3,false,true,true);f.cameraYawDeg=target;
  bool owned=true,finite=true,correctFamily=true;int tipFrames=0,directedTipFrames=0,postHoldTipFrames=0,firstTip=-1;
  float authored=0,visual=0,maxUnexplainedYaw=0,postHoldAuthored=0,errorAtHold=std::fabs(target);
  const float poseHold=ov::OverlayTimes{}.aimingMinTime;
  bool passedHold=false;
  for(int j=0;j<180;++j){
   const float elapsed=(j+1)*f.dt;
   motiondriver::SetWeaponAction(burst||j==0?8:0);
   f.playerYawDeg=target*std::min(1.f,elapsed/.17f);
   owned=rawFeed(true)&&owned;
   const float authoredStep=capturedAnimationRate*f.dt/integ::kDegToRad;
   authored+=authoredStep;
   visual=std::remainder(f.playerYawDeg+capturedRootYaw/integ::kDegToRad,360.f);
   finite=finite&&std::isfinite(authored)&&std::isfinite(visual);
   maxUnexplainedYaw=std::max(maxUnexplainedYaw,std::fabs(std::remainder(visual-authored,360.f)));
   // The single Shoot feed starts the hold; measure continuation only after
   // that hold has expired, allowing a turn that already reached the target.
   const bool afterHold=j*f.dt>poseHold+f.dt;
   if(afterHold){postHoldAuthored+=authoredStep;passedHold=true;}
   else errorAtHold=std::fabs(std::remainder(target-visual,360.f));
   const int clip=std::atoi(StatusField(motiondriver::Status(),"pick=clip:").c_str());
   if(clip>=0&&static_cast<uint32_t>(clip)<db->NumClips()){
    const auto clipName=db->ClipName(static_cast<uint32_t>(clip));
    if(steer::ClassifyClip(clipName)==steer::kClassTurnInPlace){
     ++tipFrames;if(firstTip<0)firstTip=clip;
     directedTipFrames+=clipName.ends_with(target>0?"_L":"_R")?1:0;
     if(afterHold)++postHoldTipFrames;
     correctFamily=correctFamily&&clipName.starts_with(stance==integ::kLocoStand?"M_Neutral_Stand_Turn_":"M_Neutral_Crouch_Idle_Turn_");
    }
   }
  }
  const float error=std::fabs(std::remainder(target-visual,360.f));
  std::printf("shot-turn %s visual=%.2f authored=%.2f error=%.2f TIP=%d directed=%d first=%d holdError=%.2f postHoldYaw=%.2f postHoldTIP=%d unexplained=%.5f deg\n",name.c_str(),visual,authored,error,tipFrames,directedTipFrames,firstTip,errorAtHold,postHoldAuthored,postHoldTipFrames,maxUnexplainedYaw);
  Check(owned&&finite,name+" supplies finite committed visual roots");
  Check(tipFrames>0&&directedTipFrames>0&&correctFamily,name+" selects an authored turn for the stance and shot direction");
  Check(authored*target>0&&visual*target>0,name+" authored and visual rotation progress toward the shot");
  Check(error<=15.f,name+" faces within 15deg of the shot after 3s");
  Check(maxUnexplainedYaw<.05f,name+" fast capsule turn cannot silently spin the body");
  if(!burst)Check(passedHold&&(errorAtHold<=15.f||(postHoldTipFrames>0&&postHoldAuthored*target>0)),name+" unfinished turn continues beyond HipFirePoseHold");
 };
 for(int stance:{integ::kLocoStand,integ::kLocoCrouch})for(float target:{170.f,-170.f})for(bool burst:{false,true})shotTurnCase(stance,target,burst);
 for(float target:{90.f,-90.f})shotTurnCase(integ::kLocoStand,target,false);
 // Reset guards observe the actual runtime lease, not the delayed result of
 // a clip blend. The existing camera TTL test above owns the wall-clock wait.
 auto shotActive=[](){return StatusField(motiondriver::Status(),"shotTurn=")=="1";};
 auto shotGoal=[](){return std::strtof(StatusField(motiondriver::Status(),"shotGoal=").c_str(),nullptr);};
 auto beginGuardShot=[&](){
  seedVisualOffset(integ::kLocoStand,false);setIntent(3,false,true,true);
  f.cameraYawDeg=170;f.playerYawDeg=18;rawFeed(true);
  Check(shotActive()&&std::fabs(shotGoal()-170.f)<.01f,"shot guard fixture captures stationary shot goal");
  motiondriver::SetWeaponAction(0);
 };
 beginGuardShot();
 setIntent(3,true,false,true);f.cameraYawDeg=-90;rawFeed(true);
 Check(!shotActive()&&std::fabs(shotGoal())<.01f,"single shot to ADS with new direction clears the old shot goal");
 beginGuardShot();
 motiondriver::SetWeaponAction(2);rawFeed(true);
 Check(!shotActive(),"reload cancels stationary shot turn");
 beginGuardShot();
 f.move[1]=1;f.velWorld[1]=3.85f;rawFeed(true);
 Check(!shotActive(),"movement cancels stationary shot turn");
 f.move[1]=f.velWorld[1]=0;rawFeed(true);
 Check(!shotActive(),"stopping movement cannot revive a cancelled shot goal");
 beginGuardShot();
 motiondriver::SetCameraTarget(false,false,nullptr);rawFeed(false);
 Check(!shotActive(),"invalid camera heartbeat cancels stationary shot turn");
 rawFeed(true);
 Check(!shotActive(),"fresh camera heartbeat cannot revive a cancelled shot goal");
 beginGuardShot();
 f.tppActive=false;rawFeed(false);
 Check(!g_log.stepResult,"FPP yields the stationary shot-turn frame");
 f.tppActive=true;rawFeed(true);
 Check(!shotActive(),"FPP re-entry cannot resume the old stationary shot goal");
 // A stale heartbeat reaches Hold through eligible=false, the same runtime
 // admission gate as the invalid-camera case above. Keep expiry deterministic
 // here; the earlier real-host TTL assertion already checks camera ageing.
 shotturn::Hold expiredCameraHold;
 expiredCameraHold.Step(8,1.f/60,true,170,0);
 expiredCameraHold.Step(0,1.f/60,false,170,0);
 Check(!expiredCameraHold.Active()&&expiredCameraHold.targetYawDeg==0,"expired-camera admission clears a pending shot goal");
 expiredCameraHold.Step(0,1.f/60,true,-90,0);
 Check(!expiredCameraHold.Active(),"camera recovery without a new shot cannot re-arm the goal");
 // Moving aim retains the explicit root-release policy: TIP applies only to
 // stationary aim. Both ADS and hipfire must still align a displaced root.
 for(int stance:{integ::kLocoStand,integ::kLocoCrouch})for(bool ads:{true,false}){
  seedVisualOffset(stance);setIntent(3,ads,!ads,true);
  const float speed=stance==integ::kLocoStand?3.85f:2.7f,yaw=f.cameraYawDeg*integ::kDegToRad;
  f.move[1]=1;f.velWorld[0]=-std::sin(yaw)*speed;f.velWorld[1]=std::cos(yaw)*speed;
  bool owned=true;float previous=capturedRootYaw,maxStep=0;
  for(int j=0;j<60;++j){owned=rawFeed(true)&&owned;maxStep=std::max(maxStep,std::fabs(std::remainder(capturedRootYaw-previous,2*integ::kPiF)));previous=capturedRootYaw;}
  Check(owned&&std::fabs(capturedRootYaw)<2*integ::kDegToRad,"moving pistol2H ADS/hipfire retains root convergence");
  Check(maxStep<=9.01f*integ::kDegToRad,"moving aim root-release remains bounded at 540deg/s");
 }
 auto retainedCase=[&](const char*name,int overlay,bool ads,bool drawn,bool known,bool nativeIdle,bool stale,bool fpp,int resolved=-1){
  const float before=seedVisualOffset(integ::kLocoStand);
  motionhost::SetWeaponUpper(nativeIdle);motiondriver::SetWeaponUpper(nativeIdle);
  setIntent(overlay,ads,false,drawn,known);
  // resolved 0 = the glue's grenade throw / projectile launcher: Pistol1H pose, resolvedAimPose Default (no gun in WeaponRight)
  if(resolved>=0)motiondriver::SetOverlayInputs(overlay,ads?2:1,false,false,resolved);
  if(stale){motiondriver::SetCameraTarget(true,true,forward);Sleep(300);}
  if(fpp){
   motiondriver::SetCameraTarget(true,true,forward);f.tppActive=false;rawFeed(false);
   Check(!g_log.stepResult,"FPP yields motion instead of rotating the third-person body");f.tppActive=true;
   // Re-entry deliberately resets the root. Recreate a visible offset AFTER
   // that valid reset, without publishing a replacement camera heartbeat.
   setIntent(0,false,false,false);
   for(int j=0;j<45;++j)rawFeed(false);
   for(int j=0;j<30;++j){f.playerYawDeg+=157.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   Check(std::fabs(capturedRootYaw)>2.4f,"FPP return fixture recreates visible offset after root reseed");
   setIntent(overlay,ads,false,drawn,known);
  }
  // None of these offsets was built by one of UTPP's facing owners (the fixture turns the capsule with no lease / aim intent), so
  // neither the rest turn-in-place nor its fallback may touch them, with or without a weapon pose.
  for(int j=0;j<60;++j)rawFeed(!stale&&!fpp);
  Check(std::fabs(capturedRootYaw)>std::fabs(before)*.65f,std::string(name)+" does not release unrelated visual root");
 };
 retainedCase("fresh camera without aim",0,false,false,true,false,false,false);
 retainedCase("drawn Default with raw aim",0,true,true,true,false,false,false);
 retainedCase("unknown weapon state with raw aim",0,true,false,false,false,false,false);
 retainedCase("ordinary native idle upper",3,false,true,true,true,false,false);
 retainedCase("expired camera heartbeat with owned ADS",3,true,true,true,false,true,false);
 retainedCase("FPP invalidates aim heartbeat on return",3,true,true,true,false,false,true);
 retainedCase("grenade / projectile launcher pose without a gun in WeaponRight",2,false,true,true,false,false,false,0);
 // Rest turn-in-place (review of 16d03c3). A hidden yaw offset between the capsule and the visible root, left by the hipfire lease /
 // an ADS exit, used to be folded back by turning the root at 120deg/s with no footwork (feet slid 0.3-0.4m for 157deg). With a gun in
 // WeaponRight, standing / crouching, no aim intent, the matcher now gets the offset as a turn-in-place goal (actor = the visible root,
 // goal = the capsule) and plays the authored Turn_xxx footwork; the root absorbs the clip's yaw. The offsets are the recorded
 // hidden offsets of the 12:15 recording's stationary reloads (tests/data/recording_1215_rest_offset.inc). -DUTPP_REST_TURN_MUTATION=1
 // (never admitted) MUST fail the TIP checks.
 {
  const float offsets[]={176,157,134,129,117,99,90,61,-176,-157,-134,-129,-117,-99,-90,-61};
  auto restCase=[&](int stance,float degrees,int hz){
   const float dt=1.f/float(hz);
   seedVisualOffset(stance,false);f.dt=dt;
   motiondriver::SetOverlayInputs(0,2,false,false); // a facing owner (aim intent) turns the capsule behind the camera, as the aim align / lease does
   for(int j=0;j<30;++j){f.playerYawDeg+=degrees/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   setIntent(3,false,false,true);
   const float root0=capturedRootYaw/integ::kDegToRad;
   bool owned=true,finite=true,correctFamily=true;int tipFrames=0,directedFrames=0,firstTip=-1;
   float authored=0,maxUnexplained=0,legTravel=0;std::array<mm::math::Quat,4> previousLeg=turnLegPose;
   const int frames=int(std::lround(3.5f*float(hz)));
   for(int j=0;j<frames;++j){
    owned=rawFeed(false)&&owned;
    authored+=capturedAnimationRate*dt/integ::kDegToRad;
    const float rootNow=capturedRootYaw/integ::kDegToRad;
    finite=finite&&std::isfinite(rootNow)&&std::isfinite(authored);
    maxUnexplained=std::max(maxUnexplained,std::fabs(std::remainder((rootNow-root0)-authored,360.f)));
    for(int i=0;i<4;++i){const auto&a=previousLeg[i],&b=turnLegPose[i];const float dot=std::min(1.f,std::fabs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w));legTravel+=2*std::acos(dot)/integ::kDegToRad;}
    previousLeg=turnLegPose;
    const int clip=std::atoi(StatusField(motiondriver::Status(),"pick=clip:").c_str());
    if(clip>=0&&static_cast<uint32_t>(clip)<db->NumClips()){
     const auto clipName=db->ClipName(static_cast<uint32_t>(clip));
     if(steer::ClassifyClip(clipName)==steer::kClassTurnInPlace){
      ++tipFrames;if(firstTip<0)firstTip=j;
      // the capsule turned by +degrees: the visible root follows it (+ = left); near 180 either side is the shorter way
      directedFrames+=(clipName.ends_with(degrees>0?"_L":"_R")||std::fabs(degrees)>=170.f)?1:0;
      correctFamily=correctFamily&&clipName.starts_with(stance==integ::kLocoStand?"M_Neutral_Stand_Turn_":"M_Neutral_Crouch_Idle_Turn_");
     }
    }
   }
   const float finalOffset=std::fabs(capturedRootYaw/integ::kDegToRad);
   char label[160];std::snprintf(label,sizeof label,"rest TIP %s capsule %+.0fdeg %dHz",stance==integ::kLocoStand?"standing":"crouching",degrees,hz);
   std::printf("%s: first TIP %.2fs, TIP frames %d (directed %d), final offset %.1fdeg, unexplained yaw %.3fdeg, leg travel %.0fdeg\n",label,
               firstTip<0?-1.f:float(firstTip)*dt,tipFrames,directedFrames,finalOffset,maxUnexplained,legTravel);
   Check(owned&&finite,std::string(label)+" supplies finite committed visual roots");
   Check(firstTip>=0&&float(firstTip)*dt<=.25f,std::string(label)+" the matcher admits a turn-in-place within 0.25s");
   Check(directedFrames>0&&correctFamily,std::string(label)+" plays the authored turn of the right side and stance");
   Check(finalOffset<=50.f,std::string(label)+" the offset is turned below the 50deg admission angle");
   Check(maxUnexplained<3.f,std::string(label)+" the visible V turned by authored clip yaw only (<3deg of footwork-less fallback left over at the knee)");
   Check(legTravel>=60.f,std::string(label)+" the legs step (thigh / shin rotation travel)");
  };
  for(int stance:{integ::kLocoStand,integ::kLocoCrouch})for(float degrees:offsets)restCase(stance,degrees,60);
  restCase(integ::kLocoStand,157.f,144);restCase(integ::kLocoStand,-117.f,144);
  // Exclusions. Every offset here is built under a facing owner, so only the named condition keeps the rest TIP away.
  auto seedOwned=[&](float degrees){
   seedVisualOffset(integ::kLocoStand,false);
   motiondriver::SetOverlayInputs(0,2,false,false);
   for(int j=0;j<30;++j){f.playerYawDeg+=degrees/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
  };
  auto tipFrame=[&](){const int clip=std::atoi(StatusField(motiondriver::Status(),"pick=clip:").c_str());
   return clip>=0&&static_cast<uint32_t>(clip)<db->NumClips()&&steer::ClassifyClip(db->ClipName(static_cast<uint32_t>(clip)))==steer::kClassTurnInPlace;};
  auto noRestTip=[&](const char*name,int overlay,float degrees,int resolved){
   seedOwned(degrees);
   setIntent(overlay,false,false,true);motiondriver::SetOverlayInputs(overlay,1,false,false,resolved);
   int tip=0;
   for(int j=0;j<120;++j){rawFeed(false);tip+=tipFrame()?1:0;}
   Check(tip==0,std::string("rest TIP is not played: ")+name);
  };
  noRestTip("offset below the 50deg admission angle",3,44.f,-1);
  noRestTip("unarmed pose",0,134.f,-1);
  noRestTip("grenade / launcher pose (resolvedAimPose Default)",2,134.f,0);
  // An aim intent of any kind excludes it: the ADS turn (camera goal) is the only owner then. Without a fresh camera the aim state
  // asks for no turn, so the offset stays (a rest TIP would turn it out: > 100deg).
  auto aimExcluded=[&](const char*name,bool psmAim,bool overlayAim){
   seedOwned(134.f);
   const float before=capturedRootYaw;
   setIntent(3,psmAim,false,true);motiondriver::SetOverlayInputs(3,overlayAim?2:1,false,false);
   for(int j=0;j<180;++j)rawFeed(false);
   Check(std::fabs(capturedRootYaw)>std::fabs(before)*.65f,std::string("rest TIP is not played: aim intent ")+name);
  };
  aimExcluded("(PSM ADS and RMB intent, stale camera)",true,true);
  aimExcluded("(PSM ADS only)",true,false);
  aimExcluded("(RMB intent only)",false,true);
  // The lease's last capsule step can land just after the owner dropped (recording 92.152 s: the offset of the 94.985 s reload): the
  // owner counts for 0.25 s longer, and the offset still gets its turn.
  {
   seedVisualOffset(integ::kLocoStand,false);f.dt=1.f/60;
   motiondriver::SetOverlayInputs(0,2,false,false);
   for(int j=0;j<29;++j){f.playerYawDeg+=157.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   setIntent(3,false,false,true); // the owner is gone ...
   f.playerYawDeg+=157.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false); // ... but the last 5deg step (300deg/s) still comes
   int tip=0;
   for(int j=0;j<150;++j){rawFeed(false);tip+=tipFrame()?1:0;}
   Check(tip>0&&std::fabs(capturedRootYaw)<10.f*integ::kDegToRad,"rest TIP is played: the last lease step after the owner dropped (0.25s owner grace)");
  }
  // The owner rule must cover the late step that carries an offset OVER the admission angle (review: the lease's last step arrives a
  // feed after the pose hold ran out; the offset was 45deg, below 50, when the owner dropped). The 10deg step (600deg/s) comes one
  // feed after the owner is gone.
  {
   seedVisualOffset(integ::kLocoStand,false);f.dt=1.f/60;
   motiondriver::SetOverlayInputs(0,2,false,false);
   for(int j=0;j<30;++j){f.playerYawDeg+=45.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   setIntent(3,false,false,true);rawFeed(false); // the owner is gone, no step yet: 45deg
   f.playerYawDeg+=10.f;f.cameraYawDeg=f.playerYawDeg;rawFeed(false); // the late step: 55deg
   int tip=0;
   for(int j=0;j<180;++j){rawFeed(false);tip+=tipFrame()?1:0;}
   // a 55deg offset gets the small authored turn (Turn_045 turns ~18deg), which leaves it below the admission angle
   Check(tip>0&&std::fabs(capturedRootYaw)<50.f*integ::kDegToRad,"rest TIP is played: the last lease step carries a 45deg offset over 50deg after the owner dropped");
  }
  // Below the admission angle the fallback is all there is, and it must finish: an offset of 49deg built under an owner is folded to
  // the 40deg knee (the owned flag lives until the offset is below the knee, not below 50deg).
  {
   seedVisualOffset(integ::kLocoStand,false);f.dt=1.f/60;
   motiondriver::SetOverlayInputs(0,2,false,false);
   for(int j=0;j<30;++j){f.playerYawDeg+=49.f/30;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);}
   setIntent(3,false,false,true);
   int tip=0;
   for(int j=0;j<300;++j){rawFeed(false);tip+=tipFrame()?1:0;}
   const float finalDeg=std::fabs(capturedRootYaw)/integ::kDegToRad;
   Check(tip==0&&finalDeg>=39.9f&&finalDeg<=44.f,"rest fallback reaches the 40deg knee: a 49deg offset built under an owner is folded below 44deg");
  }
  // The recorded episode (video 90.6-93.0 s: ADS shot, RMB released, the hipfire lease turns the capsule by ~250deg, reload at 94.985 s):
  // capsule yaw and the ADS / Shoot owner signals replayed at 60Hz with a fresh camera (the camera follows the capsule, so the aim /
  // hipfire facing owners are live as in the game). The owner rule, the turn-in-place and the fallback together: the episode ends with
  // the root turned (by authored clips) below the admission angle. It is an end-to-end check on real signals; the discriminating
  // owner-rule cases are the synthetic ones around it.
  {
   seedVisualOffset(integ::kLocoStand,false);f.dt=1.f/60;
   const int n=int(sizeof kOwnerFrames/sizeof kOwnerFrames[0]);
   int tip=0;bool finite=true;
   auto step=[&](){
    finite=finite&&std::isfinite(capturedRootYaw);
    tip+=tipFrame()?1:0;
   };
   float capsuleTurn=0,previousYaw=0;
   for(int j=0;j<n;++j){
    const OwnerFrame&o=kOwnerFrames[j];
    setIntent(3,o.ads!=0,o.shot!=0,true);
    f.playerYawDeg=o.capsuleDeg;f.cameraYawDeg=f.playerYawDeg;rawFeed(true);
    capsuleTurn+=o.capsuleDeg-previousYaw;previousYaw=o.capsuleDeg;
   }
   setIntent(3,false,false,true);
   for(int j=0;j<300;++j){rawFeed(true);step();}
   Check(finite&&std::fabs(capsuleTurn)>150.f&&tip>0&&std::fabs(capturedRootYaw)<50.f*integ::kDegToRad,
         "rest TIP is played: the recorded lease episode (90.6-93.0 s) ends with the root below the admission angle");
  }
  // Not every capsule turn is UTPP's: the offsets below were NOT built under a facing owner and must stay hidden, as on main.
  auto foreignCase=[&](const char*name,int stance,bool jump){
   seedVisualOffset(stance,false);setIntent(3,false,false,true);
   for(int j=0;j<30;++j)rawFeed(false);
   if(jump){f.playerYawDeg+=180.f;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);} // the game turns V onto the look when a heading lock ends
   else for(int j=0;j<60;++j){f.playerYawDeg+=3.f;f.cameraYawDeg=f.playerYawDeg;rawFeed(false);} // follow mode: the game turns V with the mouse
   const float before=capturedRootYaw;int tip=0;
   for(int j=0;j<180;++j){rawFeed(false);tip+=tipFrame()?1:0;}
   Check(std::fabs(before)>2.4f,std::string("rest TIP is not played: foreign ")+name+" leaves a large hidden offset");
   Check(tip==0&&std::fabs(std::remainder(capturedRootYaw-before,2*integ::kPiF))<3.f*integ::kDegToRad,
         std::string("rest TIP is not played: foreign ")+name+" keeps the hidden offset (no turn, no fallback)");
  };
  foreignCase("heading-lock end jump, standing",integ::kLocoStand,true);
  foreignCase("heading-lock end jump, crouching",integ::kLocoCrouch,true);
  foreignCase("follow-mode look-around, standing",integ::kLocoStand,false);
  foreignCase("follow-mode look-around, crouching",integ::kLocoCrouch,false);
 }
 std::printf("root release Default=%.5f rad PistolADS=%.5f rad NativeIdleUpper=%.5f rad\n",defaultAimRoot,pistolAimRoot,nativeIdleUpperRoot);
 std::printf("camera AO observed hipR=%.1f hipL=%.1f; %d PASS %d FAIL\n",hipRight,hipLeft,g_pass,g_fail);
 motionhost::SetMotionHooks(nullptr);motiondriver::Shutdown();motionhost::Shutdown();return g_fail?1:0;
}

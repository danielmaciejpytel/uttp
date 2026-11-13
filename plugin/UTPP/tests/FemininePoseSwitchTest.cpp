// Actual evaluator regression: changing target with dt0 must not change the
// feminine socket while the composed pose weights remain unchanged.
#include "../src/mm/MotionPose.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>
using namespace utpp; using namespace utpp::mm;
int main(int argc,char**argv)
{
 if(argc!=4)return 2;
 std::string error;auto db=std::make_unique<motion::MotionDatabase>();auto ovf=std::make_unique<ov::OverlayFile>();
 if(!db->LoadFromFile(argv[1],motion::kRigIdentityWoman,error)||!ovf->LoadFromFile(argv[2],ov::kRigIdentityWomanOv,error)){std::puts(error.c_str());return 2;}
 std::ifstream stream(argv[3]);std::stringstream text;text<<stream.rdbuf();ov::StyleFamilyData style;if(!style.Parse(text.str(),&error))return 2;
 int idle=-1;for(uint32_t i=0;i<db->NumClips();++i)if(db->ClipName(i)=="M_Neutral_Stand_Idle_Loop")idle=int(i);if(idle<0)return 2;
 int pass=0,fail=0;auto check=[&](bool ok,const char* label){(ok?pass:fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",label);};
 for(int hz:{30,60,144}){
  auto ev=std::make_unique<motionpose::PoseEvaluator>();if(!ev->Init(db.get(),ovf.get(),&style,2,error))return 2;
  motionpose::OverlayFrameInputs in;in.bodyWoman=true;in.machine.style=ov::Style::Feminine;in.machine.overlay=ov::PoseOverlay::Pistol1H;in.machine.wantsReady=true;in.capturePoseDiagnostics=true;in.gripSkip=true;
  motion::MatchResult match;match.clip=idle;match.clipTime=.5f;std::vector<BoneTransform> output(db->NumBones());motionpose::FrameInfo info;
  bool finite=true;for(int k=0;k<4*hz;++k)finite=ev->Evaluate(1.f/hz,match,in,output.data(),uint32_t(output.size()),&info)&&finite;
  check(finite&&info.feminineDelta.posM[4]>.0001f&&info.feminineDelta.posM[4]<.01999f,"fixture is actual partial feminine1HReady");
  auto zeroSwitch=[&](ov::PoseOverlay target){
   const auto before=output;const auto old=in.machine.overlay;in.machine.overlay=target;
   const bool ok=ev->Evaluate(0,match,in,output.data(),uint32_t(output.size()),&info);float pos=0,rot=0;
   for(uint32_t i=0;i<db->NumBones();++i){pos=std::max(pos,gik::Len(output[i].p-before[i].p));rot=std::max(rot,ov::AngleDeg(output[i].q,before[i].q));}
   std::printf("DT0 hz%d %d->%d posMM%.9f rotDeg%.9f feminineSocketMM%.9f\n",hz,int(old),int(target),pos*1000,rot,info.feminineDelta.posM[4]*1000);
   check(ok&&info.finite&&pos<1e-6f&&rot<.001f,"dt0 target-only switch preserves actual composed pose");
  };
  zeroSwitch(ov::PoseOverlay::Pistol2H);zeroSwitch(ov::PoseOverlay::Pistol1H);
  // Check target changes throughout the actual blend, including its compacted
  // endpoint; all probes consume zero time and leave composition unchanged.
  for(auto target:{ov::PoseOverlay::Pistol2H,ov::PoseOverlay::Pistol1H}){
   in.machine.overlay=target;
   for(int k=0;k<hz;++k){
    finite=ev->Evaluate(1.f/hz,match,in,output.data(),uint32_t(output.size()),&info)&&finite;
    if(k==0||k==hz/10||k==hz/4||k==hz/2||k==hz-1){zeroSwitch(target==ov::PoseOverlay::Pistol1H?ov::PoseOverlay::Pistol2H:ov::PoseOverlay::Pistol1H);zeroSwitch(target);}
   }
  }
  check(finite&&info.finite,"finite through blend and pose-slot compaction at30/60/144Hz");
 }
 std::printf("FemininePoseSwitch %dPASS %dFAIL\n",pass,fail);return fail?1:0;
}

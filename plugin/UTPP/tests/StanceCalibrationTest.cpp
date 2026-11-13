// Focused calibration continuity, not smoothing observed velocity or stance itself.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
using namespace utpp::mm;
static int pass=0,fail=0;
static void Check(const char* n,bool ok){std::printf("%s %s\n",ok?"PASS":"FAIL",n);ok?++pass:++fail;}
static bool Near(float a,float b,float eps=1e-3f){return std::isfinite(a)&&std::isfinite(b)&&std::fabs(a-b)<=eps;}
int main(){
 using stancecal::Blend;
 Blend x;Check("initial stand weight exact",x.Update(0,true,.5f,0)==0);
 Check("stance edge C0 at dt0",x.Update(1,true,.5f,0)==0);
 const float mid=x.Update(1,true,.5f,.25f);Check("midpoint bounded",mid>0&&mid<1);
 Check("retrigger rebases current weight",x.Update(0,true,.5f,0)==mid);
 Check("retrigger endpoint exact",x.Update(0,true,.5f,.5f)==0);
 x.Update(1,true,.5f,0);Check("air bypass exact",x.Update(1,false,.5f,.1f)==1);
 Check("air reset removes old fade",x.Update(0,true,.5f,0)==0);
 Check("duration0 legacy target",x.Update(1,true,0,0)==1);
 Check("invalid duration finite target",x.Update(0,true,std::numeric_limits<float>::quiet_NaN(),.1f)==0);
 Check("invalid dt finite target",std::isfinite(x.Update(1,true,.5f,std::numeric_limits<float>::quiet_NaN())));
 for(int edge=0;edge<3;++edge){
  const float speed=edge==0?3.8447f:(edge==1?2.709f:2.702f);const int before=edge==0?adapter::kPsmStand:adapter::kPsmCrouch;const int after=edge==2?adapter::kPsmCrouchSprint:(before==adapter::kPsmStand?adapter::kPsmCrouch:adapter::kPsmStand);
  adapter::MapConfig c;c.stanceSpeedBlendTime=.5f;adapter::MapState state;adapter::MapContext context;adapter::MotionFeed f;f.dt=1.f/60;f.velWorld[0]=speed;f.move[0]=1;f.psmDetailed=before;
  motion::ChooserState last;for(int i=0;i<60;++i)last=adapter::MapToChooser(f,c,state,context);
  const float beforeSpeed=last.speed2dCmS;
  f.psmDetailed=after;f.dt=0;const auto first=adapter::MapToChooser(f,c,state,context);char name[120];std::snprintf(name,sizeof name,"actual edge%d raw%.4f C0 chooser speed",edge,speed);Check(name,Near(first.speed2dCmS,beforeSpeed));
  adapter::MapConfig legacy=c;legacy.stanceSpeedBlendTime=0;adapter::MapState legacyState;f.psmDetailed=before;adapter::MapToChooser(f,legacy,legacyState,context);f.psmDetailed=after;const auto jump=adapter::MapToChooser(f,legacy,legacyState,context);Check("legacy exposes meaningful edge speed jump",std::fabs(jump.speed2dCmS-beforeSpeed)>50);
  // The same-frame raw vector changes immediately, including a reversal. Only scale is held.
  f.velWorld[0]=0;f.velWorld[1]=-speed*.5f;adapter::MapToChooser(f,c,state,context);Check("raw vector current, no velocity lowpass",Near(state.st.velocity.x,0)&&Near(state.st.velocity.y,-speed*.5f*100*adapter::SpeedScaleFor(c,before==adapter::kPsmCrouch,speed*.5f)));
  f.velWorld[0]=speed;f.velWorld[1]=0;f.dt=.5f;const auto endpoint=adapter::MapToChooser(f,c,state,context);const float target=100*speed*adapter::SpeedScaleFor(c,after==adapter::kPsmCrouch,speed);Check("actual edge endpoint reaches new calibration",Near(endpoint.speed2dCmS,target,.01f));
  f.velWorld[0]=0;f.move[0]=0;f.dt=0;const auto zero=adapter::MapToChooser(f,c,state,context);Check("zero raw velocity remains finite",std::isfinite(zero.speed2dCmS)&&Near(state.st.velocity.x,0)&&Near(state.st.velocity.y,0));
  // Air and traversal leave the ground calibration domain and must not carry a prior stance fade.
  for(int bypass:{adapter::kPsmFall,adapter::kPsmClimb}){f.velWorld[0]=speed;f.move[0]=1;f.psmDetailed=before;f.dt=.5f;adapter::MapToChooser(f,c,state,context);f.psmDetailed=after;f.dt=0;adapter::MapToChooser(f,c,state,context);f.psmDetailed=bypass;adapter::MapToChooser(f,c,state,context);f.psmDetailed=after;const auto reset=adapter::MapToChooser(f,c,state,context);Check("air/traversal bypass resets stale fade",Near(reset.speed2dCmS,target,.01f));}
 }
 std::printf("StanceCalibration:%dPASS %dFAIL\n",pass,fail);return fail?1:0;
}

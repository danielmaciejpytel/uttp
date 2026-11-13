#include "../src/mm/AuthoredRecoil.hpp"
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <cstring>
using namespace utpp::mm;
using namespace utpp::mm::authored_recoil;
static int checks=0;
static bool Check(bool value,const char* name){++checks;if(!value)std::cerr<<"FAIL "<<name<<'\n';return value;}
static bool Same(ov::Xf a,ov::Xf b,float tolerance=1e-5f){return Translation({{},a.t-b.t})<=tolerance&&ov::AngleDeg(a.r,b.r)<.002f;}
static std::string Fixture(){std::ostringstream s;s<<std::setprecision(10)<<"UTPP_RECOIL 1\n3 .8\n0 0 0 0 1 0 0 0\n.4 "<<std::sin(.1f)<<" 0 0 "<<std::cos(.1f)<<" .03 0 0\n.8 0 0 0 1 0 0 0\n";return s.str();}
int main(int argc,char** argv){
 if(argc<2)return 2;const auto directory=std::filesystem::path(argv[1]);std::filesystem::create_directories(directory);const auto file=directory/"fixture.utpprecoil";
 const auto save=[&](const std::string& text){std::ofstream output(file);output<<text;};
 Clip clip;std::string error;save(Fixture());if(!Check(clip.Load(file.string().c_str(),error)&&clip.Valid()&&clip.Count()==3,"valid curve"))return 1;
 ov::Xf out,identity;bool ok=clip.Sample(0,out)&&std::memcmp(&out,&identity,sizeof(out))==0;
 if(!Check(ok&&clip.Sample(clip.Duration(),out)&&std::memcmp(&out,&identity,sizeof(out))==0,"exact endpoints"))return 1;
 if(!Check(clip.Sample(.2,out)&&std::fabs(out.t.x-.015f)<1e-6f&&std::fabs(Rotation(out)-.1f)<1e-5f,"subframe interpolation"))return 1;
 if(!Check(!clip.Sample(-.01,out)&&Identity(out)&&!clip.Sample(std::numeric_limits<double>::quiet_NaN(),out),"invalid sample time"))return 1;
 Player player;Binding binding{7,8,9};if(!Check(player.Trigger(clip,binding,1,10)&&player.Sample(clip,binding,10,out)&&Identity(out),"start identity"))return 1;
 ov::Xf before,after;player.Sample(clip,binding,10.2,before);
 if(!Check(player.Trigger(clip,binding,2,10.2)&&player.Sample(clip,binding,10.2,after)&&std::memcmp(&before,&after,sizeof(before))==0,"retrigger exact continuity"))return 1;
 if(!Check(!player.Trigger(clip,binding,2,10.2)&&player.Sample(clip,binding,10.2,after)&&Same(before,after),"duplicate shot ignored"))return 1;
 if(!Check(!player.Trigger(clip,binding,1,10.2)&&player.Active(),"out-of-order serial ignored"))return 1;
 bool bounded=true,continuous=true;double now=10.2;
 for(uint64_t shot=3;shot<1003;++shot){now+=.015;player.Sample(clip,binding,now,before);player.Trigger(clip,binding,shot,now);player.Sample(clip,binding,now,after);continuous=continuous&&Same(before,after);player.Sample(clip,binding,now+.007,after);bounded=bounded&&Translation(after)<=kMaxTranslation+1e-6f&&Rotation(after)<=kMaxRotation+1e-6f&&Unit(after);}
 if(!Check(continuous&&bounded,"1000 rapid-fire retriggers continuous and bounded"))return 1;
 if(!Check(!player.Sample(clip,binding,now+2,out)&&!player.Active()&&std::memcmp(&out,&identity,sizeof(out))==0,"burst settles exactly"))return 1;
 if(!Check(player.Trigger(clip,{10,8,9},1,now+3)&&player.Sample(clip,{10,8,9},now+3,out)&&Identity(out),"new weapon clears old carry and serial"))return 1;
 if(!Check(!player.Sample(clip,binding,now+3.1,out)&&!player.Active()&&Identity(out),"binding mismatch reset"))return 1;
 player.Trigger(clip,binding,1,30);player.Sample(clip,binding,30.1,out);
 if(!Check(!player.Sample(clip,binding,30.05,out)&&!player.Active()&&Identity(out),"clock rollback reset"))return 1;
 if(!Check(!player.Trigger(clip,binding,2,std::numeric_limits<double>::infinity())&&!player.Active(),"nonfinite trigger reset"))return 1;
 for(const std::string bad:{"WRONG 1\n3 .8\n", "UTPP_RECOIL 2\n3 .8\n", "UTPP_RECOIL 1\n999 .8\n", "UTPP_RECOIL 1\n3 -1\n", "UTPP_RECOIL 1\n3 .8\n0 0 0 0 1 0 0 0\n"}){
  save(bad);if(!Check(!clip.Load(file.string().c_str(),error)&&!clip.Valid(),"malformed curve rejected"))return 1;
 }
 for(const auto& pair:{std::pair<std::string,std::string>{".4 ","0 "},{".03 0 0",".3 0 0"},{"0 0 0 1 0 0 0","0 0 0 0 0 0 0"},{".8 0 0 0 1 0 0 0",".8 0 0 0 1 .001 0 0"}}){
  std::string text=Fixture();const auto at=text.find(pair.first);if(at==std::string::npos)return 2;text.replace(at,pair.first.size(),pair.second);save(text);if(!Check(!clip.Load(file.string().c_str(),error)&&!clip.Valid(),"bad key/endpoint rejected"))return 1;
 }
 save(Fixture()+"trailing\n");if(!Check(!clip.Load(file.string().c_str(),error),"trailing data rejected"))return 1;
 if(!Check(!clip.Load(nullptr,error)&&!clip.Load("",error)&&!clip.Load("missing-recoil-input",error),"invalid file path rejected"))return 1;
 // Time sampling is analytic: tick history cannot change the motion.
 save(Fixture());if(!clip.Load(file.string().c_str(),error))return 2;
 Player direct,stepped;direct.Trigger(clip,binding,1,40);stepped.Trigger(clip,binding,1,40);
 for(int i=0;i<=100;++i)stepped.Sample(clip,binding,40+.2*i/100,after);
 direct.Sample(clip,binding,40.2,before);
 if(!Check(Same(before,after),"sample independent of tick history"))return 1;
 // Solved-arm soft-reach ownership fades separately from the physical curve.
 {
  Player envelope;envelope.Trigger(clip,binding,1,45);
  if(!Check(envelope.TailWeight(45)==1&&envelope.TailWeight(45.7)==1,"new shot tail envelope starts full; admission is caller-owned"))return 1;
  const double end=45+double(clip.Duration());
  const float half=envelope.TailWeight(end-.025);
  if(!Check(std::fabs(half-.5f)<1e-5f&&envelope.TailWeight(end)==0,"last50ms solved arms fade to exact zero"))return 1;
  const double retrigger=end-.025;
  if(!Check(envelope.Trigger(clip,binding,2,retrigger)&&envelope.TailWeight(retrigger)==half,"retrigger during tail preserves exact solved weight"))return 1;
  if(!Check(envelope.TailWeight(retrigger+.000001)-half<.00001f&&envelope.TailWeight(retrigger+.05)==1,"retrigger weight recovers smoothly without a first-frame jump"))return 1;
  bool continuousWeight=true;double clock=retrigger;
  for(uint64_t serial=3;serial<103;++serial){clock+=.775;const float w=envelope.TailWeight(clock);envelope.Trigger(clip,binding,serial,clock);continuousWeight=continuousWeight&&std::fabs(envelope.TailWeight(clock)-w)<1e-6f;}
  if(!Check(continuousWeight,"100 fadeout retriggers preserve solved weight"))return 1;
  if(!Check(envelope.TailWeight(clock+double(clip.Duration()))==0&&envelope.TailWeight(std::numeric_limits<double>::quiet_NaN())==0,"tail end and invalid time fail to zero weight"))return 1;
  envelope.Sample(clip,{99,8,9},clock+.01,out);
  if(!Check(envelope.TailWeight(clock+.01)==0&&!envelope.Active(),"binding reset clears solved weight"))return 1;
  envelope.Trigger(clip,binding,1,200);envelope.Reset();
  if(!Check(envelope.TailWeight(200)==0,"explicit reset clears solved weight"))return 1;
 }
 // Force actual saturation, rather than merely checking an already-small curve.
 std::ostringstream strong;strong<<std::setprecision(10)<<"UTPP_RECOIL 1\n3 .8\n0 0 0 0 1 0 0 0\n.4 "<<std::sin(.345f)<<" 0 0 "<<std::cos(.345f)<<" .24 0 0\n.8 0 0 0 1 0 0 0\n";
 save(strong.str());if(!clip.Load(file.string().c_str(),error))return 2;
 Player saturated;now=50;bool hitTranslation=false,hitRotation=false;
 bounded=continuous=true;saturated.Trigger(clip,binding,1,now);
 for(uint64_t shot=2;shot<1002;++shot){now+=.005;saturated.Sample(clip,binding,now,before);saturated.Trigger(clip,binding,shot,now);saturated.Sample(clip,binding,now,after);continuous=continuous&&Same(before,after);saturated.Sample(clip,binding,now+.002,after);bounded=bounded&&Translation(after)<=kMaxTranslation+1e-6f&&Rotation(after)<=kMaxRotation+1e-6f;hitTranslation=hitTranslation||Translation(after)>.2499f;hitRotation=hitRotation||Rotation(after)>.6999f;}
 if(!Check(bounded&&continuous&&hitTranslation&&hitRotation,"continuous translation and rotation saturation reached"))return 1;
 if(!Check(!saturated.Sample(clip,binding,now+2,out)&&Identity(out),"saturated burst fully returns"))return 1;
 save("UTPP_RECOIL 1\n3 .8\n0 0 0 0 1 0 0 0\n.4 .5 0 0 .8660254 0 0 0\n.8 0 0 0 1 0 0 0\n");
 if(!Check(!clip.Load(file.string().c_str(),error),"excess angular motion rejected"))return 1;
 save("UTPP_RECOIL 1\n3 .8\n0 0 0 0 1 0 0 0\n.4 nan 0 0 1 0 0 0\n.8 0 0 0 1 0 0 0\n");
 if(!Check(!clip.Load(file.string().c_str(),error),"nonfinite file rejected"))return 1;

 // A different clip of the same binding (hip <-> ADS inside a burst) carries the running offset instead of deleting it in one frame.
 {
  save(Fixture());if(!clip.Load(file.string().c_str(),error))return 2; // the malformed-file checks above left `clip` invalid
  Player burst;burst.Trigger(clip,binding,1,60);ov::Xf running,afterSwitch;burst.Sample(clip,binding,60.2,running);
  authored_recoil::Clip other;if(!other.Load(file.string().c_str(),error))return 2;
  const bool switched=burst.Trigger(other,binding,2,60.2)&&burst.Sample(other,binding,60.2,afterSwitch);
  if(!Check(switched&&Translation(running)>.01f&&Same(running,afterSwitch),"clip switch inside a burst keeps the running offset"))return 1;
  burst.Sample(other,binding,60.2+.12,afterSwitch);
  if(!Check(Translation(afterSwitch)>0&&burst.Active(),"the carried offset fades out through the new clip"))return 1;
  Player moved;moved.Trigger(clip,binding,1,60);moved.Sample(clip,binding,60.2,running);
  if(!Check(moved.Trigger(other,{99,8,9},2,60.2)&&moved.Sample(other,{99,8,9},60.2,afterSwitch)&&Identity(afterSwitch),"a new weapon (binding) still clears the carry"))return 1;
 }
 // Series-safe mode (profiles other than the Nue): a burst stacks up to the series cap instead of the saturation clamp, and the
 // output never moves faster than the speed limit, at any frame rate. The default Limits{} is the legacy behaviour (all checks above).
 {
#if defined(UTPP_RECOIL_LIMITS_MUTATION) && UTPP_RECOIL_LIMITS_MUTATION == 1
  const authored_recoil::Limits limits{}; // mutation 1: the limits are not passed
#else
  const authored_recoil::Limits limits{.12f,.60f,.5f,1.5f};
#endif
  save(strong.str());authored_recoil::Clip big;if(!big.Load(file.string().c_str(),error))return 2; // 24 cm / 0.69 rad, far above the cap
  for(const double fps:{30.0,60.0,144.0}){
   Player series;uint64_t serial=0;double clock=70;const double period=1.0/12;double nextShot=clock;ov::Xf last,current;bool havePrev=false;
   float peakT=0,peakR=0,stepT=0,stepR=0;bool clamped=false;
   for(int frame=0;frame<int(fps*(10*period+1));++frame){
    const double t=clock+frame/fps;
    if(serial<10&&t>=nextShot-1e-9){series.Trigger(big,binding,++serial,t,limits);nextShot+=period;}
    if(!series.Sample(big,binding,t,current))current={};
    peakT=std::max(peakT,Translation(current));peakR=std::max(peakR,Rotation(current));
    clamped=clamped||Translation(current)>=kMaxTranslation-1e-4f||Rotation(current)>=kMaxRotation-1e-3f;
    if(havePrev){stepT=std::max(stepT,Translation({{},current.t-last.t}));stepR=std::max(stepR,ov::AngleDeg(last.r,current.r)*3.14159265f/180);}
    last=current;havePrev=true;
   }
   const float dtF=float(1.0/fps);
   if(!Check(!clamped&&peakT<=.12f*1.05f&&peakR<=.60f*1.05f,"series: the burst stays under the series cap, no saturation clamp"))return 1;
   if(!Check(stepT<=.5f*dtF*1.02f+1e-5f&&stepR<=1.5f*dtF*1.02f+1e-4f,"series: step per frame <= speed * dt"))return 1;
  }
  // a single shot of a legacy-mode player is untouched by the new code paths: same curve as a fresh legacy sample
  Player legacy,limited;ov::Xf a,b;legacy.Trigger(clip,binding,1,80);limited.Trigger(clip,binding,1,80,authored_recoil::Limits{});
  bool identical=true;for(int i=0;i<=40;++i){legacy.Sample(clip,binding,80+.02*i,a);limited.Sample(clip,binding,80+.02*i,b);identical=identical&&std::memcmp(&a,&b,sizeof(a))==0;}
  if(!Check(identical,"default Limits{} is bit-identical to the legacy player"))return 1;
 }
 if(argc==3){size_t files=0,samples=0;for(const auto& entry:std::filesystem::directory_iterator(argv[2]))if(entry.path().extension()==".utpprecoil"){
   if(!Check(clip.Load(entry.path().string().c_str(),error),"exported native curve valid"))return 1;
   Player real;real.Trigger(clip,binding,1,100);ov::Xf previous;bool same=true;
   for(int i=0;i<=200;++i){const double t=double(clip.Duration())*i/200;real.Sample(clip,binding,100+t,out);same=same&&Unit(out)&&Translation(out)<=kMaxTranslation+1e-6f&&Rotation(out)<=kMaxRotation+1e-6f;++samples;previous=out;}
   if(!Check(same&&Identity(out),"exported curve subframes/end bounded"))return 1;++files;
  }if(!Check(files==10,"all ten native curves replayed"))return 1;std::cout<<files<<" native curves, "<<samples<<" samples\n";
 }
 // Male V parity (W5 2026-11-06): argv[2] = the shipped weapon-local profiles, argv[3] = the profiles exported from the male pma_* animsets
 // with the same settings (export_nue_native_recoil_curves.py --source-dir <male glb> --source-prefix pma). The profiles are shared by both
 // sexes only while the male curves stay within a few mm / hundredths of a radian of the shipped ones and have the same timing.
 if(argc==4){
  size_t pairs=0;double worstT=0,worstR=0,worstPeakT=0,worstPeakR=0,worstDur=0;std::string worstName;
  for(const auto& entry:std::filesystem::directory_iterator(argv[2])){
   if(entry.path().extension()!=".utpprecoil")continue;
   const auto male=std::filesystem::path(argv[3])/entry.path().filename();
   if(!Check(std::filesystem::exists(male),"male counterpart of every shipped profile exists"))return 1;
   Clip f,m;if(!Check(f.Load(entry.path().string().c_str(),error)&&m.Load(male.string().c_str(),error),"shipped and male profile valid"))return 1;
   worstDur=std::max(worstDur,std::fabs(double(f.Duration())-double(m.Duration())));
   if(!Check(std::fabs(f.Duration()-m.Duration())<1e-4f,"male and shipped profile have the same duration"))return 1;
   float peakFT=0,peakMT=0,peakFR=0,peakMR=0,diffT=0,diffR=0;ov::Xf a,b;bool ends=true;
   for(int i=0;i<=400;++i){const double t=double(f.Duration())*i/400;f.Sample(t,a);m.Sample(t,b);
    peakFT=std::max(peakFT,Translation(a));peakMT=std::max(peakMT,Translation(b));peakFR=std::max(peakFR,Rotation(a));peakMR=std::max(peakMR,Rotation(b));
    diffT=std::max(diffT,Translation({{},a.t-b.t}));diffR=std::max(diffR,ov::AngleDeg(a.r,b.r)*3.14159265f/180);
    if(i==400)ends=Identity(a)&&Identity(b);}
   if(!Check(ends,"male and shipped curves both end at identity"))return 1;
   const float dPeakT=std::fabs(peakMT-peakFT),dPeakR=std::fabs(peakMR-peakFR);
   if(diffT>worstT){worstT=diffT;worstName=entry.path().filename().string();}
   worstR=std::max<double>(worstR,diffR);worstPeakT=std::max<double>(worstPeakT,dPeakT);worstPeakR=std::max<double>(worstPeakR,dPeakR);
   if(!Check(diffT<=.015f&&diffR<=.02f&&dPeakT<=.008f&&dPeakR<=.02f,("male curve within the parity bounds of the shipped one: "+entry.path().filename().string()).c_str()))return 1;
   ++pairs;
  }
  if(!Check(pairs>=32,"all 32 profile files compared (15 families x2 + Nue x2)"))return 1;
  std::cout<<"male parity: "<<pairs<<" profile files, worst curve diff "<<worstT*1000<<" mm ("<<worstName<<"), "<<worstR<<" rad, worst peak diff "<<worstPeakT*1000<<" mm / "<<worstPeakR<<" rad, worst duration diff "<<worstDur<<" s\n";
 }
 std::cout<<"Authored recoil "<<checks<<" checks PASS\n";
}

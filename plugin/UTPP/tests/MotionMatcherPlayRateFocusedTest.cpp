#define main UnusedMatcherSuiteMain
#include "ChooserMatcherTest.cpp"
#undef main
#include "../src/mm/MotionAdapter.hpp"
int main(int argc,char**argv){
 if(argc!=3){std::puts("usage focused <db> <chooser>");return 2;}
 MotionDatabase db;chooser::Table table;std::string err;
 CHECK(db.LoadFromFile(argv[1],kRigIdentityWoman,err),"db %s",err.c_str());CHECK(table.LoadFile(argv[2],err),"chooser %s",err.c_str());if(g_fail)return 2;
 struct Coverage{int jump=0,nonjump=0,loop=0,wrap=0,nonloop=0,handover=0,zero=0;}total;
 for(float rate:{1.f,.75f}){
  MatcherConfig cfg;cfg.minPlayRate=cfg.maxPlayRate=rate;ChooserMatcher mm;CHECK(mm.Init(&db,&table,cfg,err),"init");
  Straight motion;motion.speed=[](double t){return t<2?0.f:t<7?2.f:t<14?5.f:t<20?7.f:0.f;};Sim sim;sim.m=&motion;sim.Build(28);
  std::vector<TrajSample>traj;std::vector<BoneTransform>pose(db.NumBones());MatchResult prev;bool have=false;
  bool times=true,jumptimes=true,bounds=true,finite=true,blend=true,zero=true;
  for(int i=0;i<1440;++i){double t=i/60.;float speed=motion.speed(t);auto s=Ground(speed>0, speed>6?2:speed>3?1:0,speed);if(t>9&&t<11)s.stance=1;
   sim.Traj(t,traj);const auto r=mm.Update(1.f/60,traj.data(),(uint32_t)traj.size(),s);
   if(r.clip<0){finite=false;continue;}const auto&cl=db.Clip(r.clip);float len=(cl.numFrames-1)/db.Header().fps;
   bounds &= r.clipTime>=0&&r.clipTime<=len+1e-4f;finite &= adapter::SamplePose(db,r.clip,r.clipTime,pose.data(),db.NumBones());
   for(const auto&p:pose)finite &= std::isfinite(p.p.x)&&std::isfinite(p.p.y)&&std::isfinite(p.p.z)&&std::isfinite(p.q.x)&&std::isfinite(p.q.y)&&std::isfinite(p.q.z)&&std::isfinite(p.q.w);
   finite &= r.playRate==rate;
   if(cl.flags&utpp::mm::motion::kClipLoop)++total.loop;else ++total.nonloop;
   if(r.jumped){++total.jump;float rowtime=(db.Row(db.Db(r.database).firstRow+r.localRow).frame-cl.firstFrame)/db.Header().fps;jumptimes &= std::fabs(r.clipTime-rowtime)<1e-5f;if(have&&!(db.Clip(prev.clip).flags&utpp::mm::motion::kClipLoop)&&prev.clipTime+rate/60> (db.Clip(prev.clip).numFrames-1)/db.Header().fps)++total.handover;
    if(r.blend.valid)blend &= std::fabs(r.blend.blendTime-mm.BlendTimeFor(s,s))<1e-5f;
   }else if(have&&r.clip==prev.clip){++total.nonjump;float expected=prev.clipTime+prev.playRate/60;if(cl.flags&utpp::mm::motion::kClipLoop){if(expected>len)++total.wrap;expected=std::fmod(expected,len);}times &= std::fabs(expected-r.clipTime)<2e-5f;}
   if(i==300){const auto z=mm.Update(0,traj.data(),(uint32_t)traj.size(),s);if(!z.jumped){zero &= z.clip==r.clip&&z.clipTime==r.clipTime;++total.zero;}prev=z;}else prev=r;have=true;
  }
  // A bounded explicit boundary step proves nonloop end handover, rather than
  // assuming ordinary searches happen to let a transition reach its last frame.
  // G1 (MatcherConfig::slowWalkLoops): this boundary step drives the non-loop expiry through the Run_to_Walk transition clip, which a steady walk
  // no longer picks; the motion selection (flag off) keeps it as the vehicle, the G1 rule has its own test (SlowWalkLoopTest.cpp).
  MatcherConfig referenceLoop=cfg;referenceLoop.slowWalkLoops=false;ChooserMatcher gm;CHECK(gm.Init(&db,&table,referenceLoop,err),"init motion");
  bool nonloopFound=false;
  for(int j=0;j<120;++j){double t=3.+j/60.;sim.Traj(t,traj);const auto s=Ground(1,0,2.f);const auto r=gm.Update(1.f/60,traj.data(),(uint32_t)traj.size(),s);
   if(r.clip>=0&&!(db.Clip(r.clip).flags&utpp::mm::motion::kClipLoop)){
    nonloopFound=true;
    const float len=(db.Clip(r.clip).numFrames-1)/db.Header().fps;
    const float endDt=(len-r.clipTime)/r.playRate+.001f;
    const auto end=gm.Update(endDt,traj.data(),(uint32_t)traj.size(),s);
    CHECK(end.clip>=0&&end.jumped,"rate%.2f actual nonloop expiry produces new selection",rate);
    if(end.clip>=0&&end.jumped)++total.handover;
    break;
   }
  }  CHECK(nonloopFound,"rate%.2f boundary step found a non-loop clip",rate);CHECK(times,"rate%.2f nonjump uses previous rate once",rate);CHECK(jumptimes,"rate%.2f jump exact indexed row time",rate);CHECK(bounds,"rate%.2f clip bounds",rate);CHECK(finite,"rate%.2f all poses/rates finite",rate);CHECK(blend,"rate%.2f source blend remains real policy",rate);CHECK(zero,"rate%.2f dt0 unchanged",rate);
 }
 CHECK(total.jump>0&&total.nonjump>0,"jump/nonjump coverage");CHECK(total.loop>0&&total.nonloop>0,"loop/nonloop coverage");CHECK(total.handover>0,"nonloop end handover coverage");CHECK(total.zero==2,"dt0 nonjump coverage");
 std::printf("coverage jump%d nonjump%d loop%d nonloop%d wrap%d handover%d dt0%d; %dPASS %dFAIL\n",total.jump,total.nonjump,total.loop,total.nonloop,total.wrap,total.handover,total.zero,g_pass,g_fail);return g_fail?1:0;
}

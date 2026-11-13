#define main FullMatcherSuiteMainNotRun
#include "ChooserMatcherTest.cpp"
#undef main
int main(){MotionDatabase db;chooser::Table table;std::string err;
 if(!db.LoadFromFile("Data/motion/utpp_motion_woman.utppmm",kRigIdentityWoman,err)||!table.LoadFile("Data/motion/utpp_motion_woman.chooser.json",err))return 2;
 const auto& cl489=db.Clip(489);std::printf("META489 name=%s frames=%u flags=%u period=%u duration=%f\n",std::string(db.ClipName(489)).c_str(),cl489.numFrames,cl489.flags,cl489.loopPeriod,float(cl489.numFrames-1)/db.Header().fps);
 for(uint32_t e=0;e<db.Header().numEntries;++e){const auto& en=db.Entry(e);if(en.clip==489)std::printf("ENTRY489 db=%u flags=%u range=%f,%f rows=%u\n",en.database,en.flags,en.rangeMin,en.rangeMax,en.numRows);}
 ChooserMatcher mm;MatcherConfig cfg;if(!mm.Init(&db,&table,cfg,err))return 2;
 Straight motion;motion.speed=+[](double){return 6.5f;};Sim sim;sim.m=&motion;sim.Build(20);double now=0;std::vector<TrajSample> traj;int noPose=0,badBlend=0,badClock=0,transitionPicks=0,jumps=0;
 for(int f=0;f<600;++f){now+=kDt;sim.Traj(now,traj);int phase=f/100;auto state=Ground(phase==0||phase==4?0:1,phase==2?2:1,phase==0||phase==4?0:phase==2?6.f:3.5f);state=Ground(1,2,6.5f);
 const auto r=mm.Update(kDt,traj.data(),uint32_t(traj.size()),state);if(r.clip<0){++noPose;continue;}if(r.jumped){++jumps;if(f>0&&(!r.blend.valid||r.blend.blendTime<=0))++badBlend;if(steer::ClassifyClip(db.ClipName(r.clip))==steer::kClassStanceTransition)++transitionPicks;}
 float end=float(db.Clip(r.clip).numFrames-1)/db.Header().fps;if(!std::isfinite(r.clipTime)||r.clipTime<0||r.clipTime>end+1e-4)++badClock;
 if(f%100==0 || (r.clip==489 && r.jumped))std::printf("phase%d clip=%s time=%f jump=%d blend=%d/%f\n",phase,std::string(db.ClipName(r.clip)).c_str(),r.clipTime,r.jumped,r.blend.valid,r.blend.blendTime);
 }
 std::printf("focused600frames noPose=%d badBlend=%d badClock=%d jumps=%d stancePicks=%d\n",noPose,badBlend,badClock,jumps,transitionPicks);return noPose||badBlend||badClock?1:0;}

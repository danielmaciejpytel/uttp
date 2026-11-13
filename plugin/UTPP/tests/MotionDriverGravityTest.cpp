// Focused real v1 bridge + real v2 trajectory checks; no game/RED4ext.
#define main UTPPUnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>
namespace {
motionhost::FeedInput gravityCaptured;
int gravityCalls=0;
bool CaptureGravity(const motionhost::FeedInput& in,uint32_t,float){gravityCaptured=in;++gravityCalls;return false;}
float ReadTtl(){const auto s=motiondriver::Status();const auto p=s.find(" ttl=");return p==std::string::npos?-1:std::strtof(s.c_str()+p+5,nullptr);}
motionhost::FeedInput GravityFeed(bool air,float vz){motionhost::FeedInput f;f.tppActive=true;f.dt=1.f/60;f.locoState=air?0:integ::kLocoStand;f.haveAir=true;f.onGround=!air;f.distToGround=air?1.f:0.f;f.velWorld[2]=vz;f.psmBlock=air?(0x40u|((uint32_t(vz>0?18:14)+1)<<16)):0;return f;}
}
int main(int argc,char**argv){
 if(argc!=3){std::printf("usage: gravitytest <motion data dir> <base db>\n");return 2;}
 const std::string data=argv[1];g_baseDb=std::wstring(argv[2],argv[2]+std::strlen(argv[2]));
 Rig rig;if(!RigFromDb(DataPaths(data,"woman").files.db,rig)){std::puts("FAIL rig");return 3;}
 const motionhost::MotionHooks capture{&CaptureGravity,nullptr,nullptr,nullptr,nullptr,nullptr};
 motionhost::SetMotionHooks(&capture);Check(ResetBase(rig),"load base bridge fixture");
 for(int i=0;i<60;++i)motionhost::Feed(GravityFeed(false,0));
 const int before=gravityCalls;
 float vz=4;for(int i=0;i<44;++i){motionhost::Feed(GravityFeed(true,vz));vz-=16.f/60;}
 Check(gravityCalls>before,"base bridge hook observed airborne feeds");
 Check(std::isfinite(gravityCaptured.predictedGravityDown)&&std::fabs(gravityCaptured.predictedGravityDown-16)<.25f,"base forwards existing descending estimate");
 Check(std::isfinite(gravityCaptured.predictedGravityUp)&&gravityCaptured.predictedGravityUp<18.f&&gravityCaptured.predictedGravityUp>15.5f,"base forwards learned ascent estimate (not default19.2)");
 motionhost::SetMotionHooks(nullptr);motionhost::OnDisarmed();
 motiondriver::SetPathsOverride(0,DataPaths(data,"woman"));motiondriver::SetRigBody(0);motiondriver::SetOverlayInputs(0,0,false,false);motiondriver::SetEnabled(true);
 (void)motiondriver::Step({},0,1.f/60);motiondriver::WaitLoader();
 auto run=[&](float up,float down,float vertical,bool air,std::vector<float>*pose=nullptr){
  motiondriver::OnDisarmed();motiondriver::OnArmed(RigPtr(),rig.hashes,rig.parents);
  auto f=GravityFeed(air,vertical);f.predictedGravityUp=up;f.predictedGravityDown=down;
  bool owned=false;for(int i=0;i< (air?3:180);++i){motiondriver::FeedAirFromInput(f);owned=motiondriver::Step(f,0,f.dt);}
  Check(owned,"real motion Step owns fixture");
  if(pose){std::vector<int32_t> dbParents(rig.parents.begin(),rig.parents.end());const auto boneMap=integ::BuildBoneMap(rig.hashes,dbParents,rig.hashes,rig.parents,motiondriver::kFirstBone);Check(boneMap.ok,"actual hash/parent bone map valid");std::vector<BoneTransform>buf(rig.bones);motionhost::MotionSetFrame frame;frame.buf=buf.data();frame.bufCap=rig.bones;const bool ok=motiondriver::Frame(boneMap.gameIndex,rig.bones,frame);Check(ok,"ground frame available");if(ok){pose->resize(rig.bones*7);std::memcpy(pose->data(),frame.pose,pose->size()*sizeof(float));}}
  return ReadTtl();
 };
 const float riseDefault=run(0,0,3,true),riseMeasured=run(19.2f,16,3,true),riseWrongDown=run(19.2f,29,3,true);
 Check(riseDefault>riseMeasured+.08f,"ascending trajectory uses measured Up instead9.81");
 Check(riseMeasured==riseWrongDown,"ascending uses Up, not Down");
 const float fallDefault=run(0,0,-3,true),fallMeasured=run(19.2f,16,-3,true),fallWrongUp=run(29,16,-3,true);
 Check(fallDefault>fallMeasured+.01f,"descending trajectory uses measured Down instead9.81");
 Check(fallMeasured==fallWrongUp,"descending uses Down, not Up");
 Check(run(std::numeric_limits<float>::quiet_NaN(),16,3,true)==riseDefault,"invalid ascent gravity falls back without poison");
 Check(run(19.2f,31,-3,true)==fallDefault,"out-of-range descent falls back");
 std::vector<float>a,b;run(9.81f,9.81f,0,false,&a);run(19.2f,16,0,false,&b);
 Check(!a.empty()&&a==b,"grounded published local pose bytes unchanged by gravity");
 motiondriver::Shutdown();motionhost::Shutdown();std::printf("gravity focused: %d PASS %d FAIL\n",g_pass,g_fail);return g_fail?1:0;
}

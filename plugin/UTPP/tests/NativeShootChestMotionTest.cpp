#include "../src/mm/NativeShootTransfer.hpp"
#include <cstdio>
#include <fstream>
#include <cstring>
#include <limits>
using namespace utpp::mm;
static ov::Xf Compose(const ov::Xf& a,const ov::Xf& b){return {math::QuatMul(a.r,b.r),a.t+math::QuatRotate(a.r,b.t)};}
static bool Same(const ov::Xf& a,const ov::Xf& b){return gik::Len(a.t-b.t)<.00002f && ov::AngleDeg(a.r,b.r)<.06f;}
int main(int argc,char** argv){
 int checks=0;
 const auto require=[&](bool ok){++checks;if(!ok)std::printf("FAIL %d\n",checks);return ok;};
 const ov::Xf chest0{gik::AxisAngle({0,0,1},.3f),{.1f,.2f,1.2f}};
 const ov::Xf chest1{gik::AxisAngle({0,0,1},1.8f),{2,-1,1.3f}};
 const ov::Xf held{gik::AxisAngle({1,0,0},.2f),{.2f,.4f,.1f}};
 const ov::Xf recoil{gik::AxisAngle({1,0,0},.12f),{.01f,-.025f,.005f}};
 ov::Xf result;
 if(!require(native_shoot::ChestRelativeMotion(chest0,Compose(chest0,held),chest1,Compose(chest1,held),result)&&Same(result,{})))return 1;
 if(!require(native_shoot::ChestRelativeMotion(chest0,Compose(chest0,held),chest1,Compose(chest1,Compose(held,recoil)),result)&&Same(result,recoil)))return 1;
 // Independent global coordinate changes at each sample must not affect extraction.
 const ov::Xf world0{gik::AxisAngle({0,1,0},.5f),{-4,2,3}},world1{gik::AxisAngle({1,0,0},-.7f),{3,4,-2}};
 if(!require(native_shoot::ChestRelativeMotion(Compose(world0,chest0),Compose(world0,Compose(chest0,held)),Compose(world1,chest1),Compose(world1,Compose(chest1,Compose(held,recoil))),result)&&Same(result,recoil)))return 1;
 auto bad=chest1;bad.r={0,0,0,0};const auto sentinel=result;
 if(!require(!native_shoot::ChestRelativeMotion(chest0,held,bad,held,result)&&Same(result,sentinel)))return 1;
 bad=held;bad.t.x=std::numeric_limits<float>::quiet_NaN();
 if(!require(!native_shoot::ChestRelativeMotion(chest0,held,chest1,bad,result)))return 1;
 bad=held;bad.s.x=2;
 if(!require(!native_shoot::ChestRelativeMotion(chest0,held,chest1,bad,result)))return 1;
 bad=held;bad.t.x+=.3f;
 if(!require(!native_shoot::ChestRelativeMotion(chest0,held,chest0,bad,result)))return 1;
 bad=held;bad.r=math::QuatMul(held.r,gik::AxisAngle({0,0,1},.8f));
 if(!require(!native_shoot::ChestRelativeMotion(chest0,held,chest0,bad,result)))return 1;

 // Exact endpoints and post-solver activation fade (including aliased output).
 ov::Skeleton sk;sk.n=8;const int parent[]={-1,0,1,2,0,4,5,6};
 for(int i=0;i<sk.n;++i)sk.parent[i]=static_cast<int16_t>(parent[i]);
 gik::Rig rig;rig.left={1,2,3};rig.right={4,5,6};rig.weaponRight=7;
 ov::Pose base,solved,blended;base.n=solved.n=sk.n;
 for(int i=0;i<sk.n;++i)base.b[i].t={.01f*i,.03f,.1f};
 solved=base;for(int b:{1,2,3,4,5,6})solved.b[b].r=gik::AxisAngle({1,0,0},.35f);
 if(!require(native_shoot::BlendSolvedArms(sk,rig,base,solved,0,blended)&&std::memcmp(&blended,&base,sizeof(base))==0))return 1;
 if(!require(native_shoot::BlendSolvedArms(sk,rig,base,solved,1,blended)&&std::memcmp(&blended,&solved,sizeof(solved))==0))return 1;
 if(!require(native_shoot::BlendSolvedArms(sk,rig,base,solved,.5f,blended)&&ov::AngleDeg(base.b[4].r,blended.b[4].r)>9&&ov::AngleDeg(base.b[4].r,blended.b[4].r)<11))return 1;
 for(int b=0;b<sk.n;++b)if(!require(gik::Len(blended.b[b].t-base.b[b].t)==0&&gik::Len(blended.b[b].s-base.b[b].s)==0))return 1;
 if(!require(std::memcmp(&blended.b[7],&base.b[7],sizeof(ov::Xf))==0))return 1;
 auto alias=base;if(!require(native_shoot::BlendSolvedArms(sk,rig,alias,solved,.5f,alias)&&std::memcmp(&alias,&blended,sizeof(alias))==0))return 1;
 alias=solved;if(!require(native_shoot::BlendSolvedArms(sk,rig,base,alias,.5f,alias)&&std::memcmp(&alias,&blended,sizeof(alias))==0))return 1;
 if(!require(native_shoot::BlendSolvedArms(sk,rig,base,solved,.001f,blended)&&ov::AngleDeg(base.b[4].r,blended.b[4].r)<.1f))return 1;
 const auto unchanged=blended;
 if(!require(!native_shoot::BlendSolvedArms(sk,rig,base,solved,std::numeric_limits<float>::quiet_NaN(),blended)&&std::memcmp(&unchanged,&blended,sizeof(blended))==0))return 1;
 solved.b[4].r={0,0,0,0};
 if(!require(!native_shoot::BlendSolvedArms(sk,rig,base,solved,.5f,blended)&&std::memcmp(&unchanged,&blended,sizeof(blended))==0))return 1;

 // Replay an optional local capture; each line chest-before, weapon-before,
 // chest-after, weapon-after (quaternion xyzw then translation xyz).
 int replay=0;if(argc==2){std::ifstream in(argv[1]);if(!in)return 2;
  for(;;){ov::Xf c0,w0,c1,w1;ov::Xf* values[]={&c0,&w0,&c1,&w1};
   if(!(in>>c0.r.x)){if(in.eof())break;return 2;}
   if(!(in>>c0.r.y>>c0.r.z>>c0.r.w>>c0.t.x>>c0.t.y>>c0.t.z))return 2;
   for(int i=1;i<4;++i){auto& x=*values[i];if(!(in>>x.r.x>>x.r.y>>x.r.z>>x.r.w>>x.t.x>>x.t.y>>x.t.z))return 2;}
   ov::Xf legacy,relative;
   if(!require(native_shoot::RelativeMotion(w0,w1,legacy)&&native_shoot::ChestRelativeMotion(c0,w0,c1,w1,relative)&&Same(legacy,relative)))return 1;
   ++replay;
  }if(!replay)return 2;
 }
 std::printf("Chest motion and solved-arm activation: %d checks PASS, %d captured standing frames (not moving validation)\n",checks,replay);
}

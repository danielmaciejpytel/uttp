#include "../src/mm/MagazinePoseTarget.hpp"
#include <cstdio>
using namespace utpp::mm;
int main(){
 ov::Skeleton sk;sk.n=5;int16_t p[]={-1,0,0,1,2};for(int i=0;i<5;++i)sk.parent[i]=p[i];
 std::vector<std::string> names={"mag_plug","magazine","magazine_reload","mag_std","mag_stdr"};
 ov::Pose current;current.n=5;
 for(int i=0;i<5;++i){current.b[i]=ov::Xf{};current.b[i].t={.01f*i,.02f*i,.03f*i};current.b[i].r=gik::AxisAngle({0,0,1},.12f*i);}
 int cases=0;
 for(bool generic:{false,true})for(bool reload:{false,true})for(int k=0;k<3;++k){
  const std::vector<std::string> order=generic?std::vector<std::string>{"mag_plug","reload_magazine","magazine","mag_stdr","mag_std"}:names;
  ov::Xf mount;mount.t={1.3f+k,.4f,-.2f};mount.r=gik::AxisAngle({0,0,1},.3f*k);
  ov::Xf target;target.t={.2f,.6f+k,.8f};target.r=gik::AxisAngle({1,0,0},.4f);
  ov::Pose output;const int leaf=generic?(reload?3:4):(reload?4:3);
  if(!authored_reload::PlaceMagazine(sk,order,current,reload?"MagazineReload":"Magazine",mount,target,true,output))return 1;
  auto got=native_reload::Compose(mount,native_reload::Model(sk,output,leaf));
  if(gik::Len(got.t-target.t)>1e-5f || ov::AngleDeg(got.r,target.r)>.1f)return 2;
  for(int i=0;i<5;++i)if(i!=leaf && std::memcmp(&current.b[i],&output.b[i],sizeof(ov::Xf)))return 3;
  ++cases;
 }
 ov::Pose unchanged=current;ov::Xf identity;
 if(authored_reload::PlaceMagazine(sk,names,current,"Magazine",identity,identity,false,unchanged) || std::memcmp(&current,&unchanged,sizeof(current)))return 4;
 if(authored_reload::PlaceMagazine(sk,names,current,"Barrel",identity,identity,true,unchanged))return 5;
 auto duplicate=names;duplicate[2]="mag_std";
 if(authored_reload::PlaceMagazine(sk,duplicate,current,"Magazine",identity,identity,true,unchanged))return 6;
 auto scaled=identity;scaled.s.x=2;
 if(authored_reload::PlaceMagazine(sk,names,current,"Magazine",scaled,identity,true,unchanged))return 7;
 std::printf("Magazine target: %d rotated/translated mount cases and rejection invariants PASS\n",cases);
}

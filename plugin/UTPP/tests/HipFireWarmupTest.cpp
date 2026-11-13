#include "../src/mm/HipFirePose.hpp"
#include "../src/mm/NativeShootTransfer.hpp"
#include <cstdio>
int main(){
 using namespace utpp::mm;
 native_shoot::EpisodeGate gate; gate.Begin(1000);
 float weight=0; int checks=0;
 for(int tick=1000;tick<=1080;tick+=10){
   const bool prepare=true;
   const bool capture=ov::CaptureNativeReadyReference(8,prepare,false);
   if(capture) return 1;
   bool accepted=gate.Accept(tick>=1020,tick<1020,tick,100,50,weight);
   if(tick>=1020 && !accepted)return 2;
   bool transferred=accepted && weight>0;
   const bool native=ov::HipFireNativeActionOwner(false,true,true,true,8,transferred,0.5f,gate.pending || gate.admitted);
   if(native)return 3;
   ++checks;
 }
 if(!gate.admitted || weight!=1)return 4;
 gate.Begin(2000);
 if(gate.Accept(false,true,2101,100,50,weight) || gate.pending || gate.admitted)return 7;
 if(!ov::HipFireNativeActionOwner(false,true,true,true,8,false,0.5f,gate.pending || gate.admitted))return 8;
 if(!ov::HipFireNativeActionOwner(true,false,true,true,8,false,0.5f,true))return 9;
 if(!ov::HipFireNeedsNativeAction(true,8,false,1))return 5;
 if(ov::HipFireNeedsNativeAction(true,0,false,1))return 6;
 // P9: no transfer route (rifle, SMG, shotgun, other pistols) -> nothing to wait for, the authored arms stay; weaponUpper stays native
 if(ov::HipFireNativeActionOwner(false,true,false,true,8,false,0.5f,false))return 10;
 if(!ov::HipFireNativeActionOwner(true,true,false,true,8,false,0.5f,false))return 11;
 std::printf("Hipfire warmup: %d frames + admission/fallback checks PASS\n",checks);
}

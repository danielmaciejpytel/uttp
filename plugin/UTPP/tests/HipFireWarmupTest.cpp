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
 // Shoot state before its first shot event (a wind-up weapon fires ~70-90 ms after the trigger): with a route the authored pose waits
 // a bounded time instead of handing the arms to the native shot; the wait never changes a weapon without a route, aimed fire or
 // a frame without action 8.
 if(ov::HipFireNativeActionOwner(false,true,true,true,8,false,0.5f,false,true))return 12;
 if(!ov::HipFireNativeActionOwner(true,true,false,true,8,false,0.5f,false,true))return 13;
 if(ov::HipFireNativeActionOwner(false,true,false,true,8,false,0.5f,false,true))return 14;
 if(ov::HipFireNativeActionOwner(false,true,true,false,8,false,0.5f,false,true) ||
    ov::HipFireNativeActionOwner(false,true,true,true,0,false,0.5f,false,true))return 15;
 {
   const double bound=ov::kHipFireShotEventWaitS;
   static_assert(ov::kHipFireShotEventWaitS>=0.15 && ov::kHipFireShotEventWaitS<=0.3,
                 "measured delays 0.066-0.088 s: the bound keeps a margin and stays short");
   ov::ShotEventWait w;
   // the event comes 70 ms after the Shoot edge: waiting until it, then the in-flight rule owns the shot (no restart in the same state)
   if(!w.Step(true,true,true,false,10.0))return 20;
   if(!w.Step(true,false,true,false,10.05))return 21;
   if(w.Step(true,false,true,true,10.07) || w.expired)return 22;
   if(w.Step(true,false,true,false,10.10))return 23;
   // no event: waiting up to the bound (inclusive), then expired for the rest of the state, the fallback applies
   w.Step(false,false,true,false,11.0);
   if(!w.Step(true,true,true,false,12.0) || !w.Step(true,false,true,false,12.0+bound))return 24;
   if(w.Step(true,false,true,false,12.0+bound+0.01) || !w.expired)return 25;
   if(w.Step(true,false,true,false,12.0+bound+0.02))return 26;
   // a new Shoot state after the action left 8 waits again
   if(w.Step(false,false,true,false,13.0) || w.expired)return 27;
   if(!w.Step(true,true,true,false,14.0))return 28;
   // the route lost inside the state ends the wait for good (the held route can only fall)
   if(w.Step(true,false,false,false,14.02) || w.Step(true,false,true,false,14.03))return 29;
   // an event already in flight on the edge: no wait at all
   w.Step(false,false,true,false,15.0);
   if(w.Step(true,true,true,true,16.0) || w.Step(true,false,true,false,16.01))return 30;
   // no route on the edge: no wait
   w.Step(false,false,true,false,17.0);
   if(w.Step(true,true,false,false,18.0))return 31;
   // a clock that jumps backwards or is not finite expires the wait (fail towards the old fallback)
   w.Step(false,false,true,false,19.0);
   if(!w.Step(true,true,true,false,20.0) || w.Step(true,false,true,false,19.5) || !w.expired)return 32;
 }
 std::printf("Hipfire warmup: %d frames + admission/fallback checks + shot event wait PASS\n",checks);
}

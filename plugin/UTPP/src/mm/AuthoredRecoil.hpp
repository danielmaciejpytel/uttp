#pragma once
// Explicitly bound, immutable weapon-local recoil curves. Loading is startup-only;
// Player is game-thread state, allocation-free, with no native FK or locomotion input.
#include "Additive.hpp"
#include <array>
#include <fstream>
#include <string>
#include <limits>
#include <algorithm>
#include <cmath>
namespace utpp::mm::authored_recoil {
inline constexpr size_t kMaxSamples=256;
inline constexpr float kMaxTranslation=.25f, kMaxRotation=.70f;
inline float Translation(ov::Xf x){return std::sqrt(math::Dot(x.t,x.t));}
inline float Rotation(ov::Xf x){return 2*std::atan2(std::sqrt(x.r.x*x.r.x+x.r.y*x.r.y+x.r.z*x.r.z),std::fabs(x.r.w));}
inline bool Unit(ov::Xf x){return math::IsFinite(x.r)&&math::IsFinite(x.t)&&std::fabs(math::QuatDot(x.r,x.r)-1)<.001f;}
inline bool Identity(ov::Xf x){return Translation(x)<.000001f&&Rotation(x)<.0001f;}
struct Key {float seconds=0;ov::Xf delta;};
// Series-safe mode of a profile (RecoilProfiles.hpp); all zero = the legacy behaviour, bit for bit (the Nue).
//  seriesTranslation / seriesRotation: cap of (offset still carried) + (new kick). A retrigger inside a burst scales the new kick to the
//    headroom the cap leaves, so full-auto stacks up to the cap instead of the saturation clamp below.
//  speedTranslation / speedRotation: speed limit of the output (m/s, rad/s): the step per frame stays <= speed * dt whatever the rate.
struct Limits {
 float seriesTranslation=0,seriesRotation=0,speedTranslation=0,speedRotation=0;
 bool Headroom()const{return seriesTranslation>0&&seriesRotation>0;}
 bool Slew()const{return speedTranslation>0&&speedRotation>0;}
};
class Clip {
public:
 // UTPP_RECOIL 1, followed by count duration and count rows t qx qy qz qw tx ty tz.
 // Failed load invalidates this object. Never mutate a Clip while a Player uses it.
 bool Load(const char* file,std::string& error){
  count_=0;duration_=0;peakTranslation_=peakRotation_=0;error.clear();if(!file||!*file){error="missing recoil path";return false;}std::ifstream in(file);std::string magic;int version=0;size_t count=0;float duration=0;
  if(!(in>>magic>>version)||magic!="UTPP_RECOIL"||version!=1){error="invalid recoil header";return false;}
  if(!(in>>count>>duration)||count<3||count>kMaxSamples||!std::isfinite(duration)||duration<.02f||duration>5){error="invalid recoil size/duration";return false;}
  for(size_t i=0;i<count;++i){auto& k=keys_[i];auto& x=k.delta;
   if(!(in>>k.seconds>>x.r.x>>x.r.y>>x.r.z>>x.r.w>>x.t.x>>x.t.y>>x.t.z)||!std::isfinite(k.seconds)||k.seconds<0||k.seconds>duration||
      (i&&k.seconds<=keys_[i-1].seconds)||!Unit(x)||Translation(x)>kMaxTranslation||Rotation(x)>kMaxRotation){error="invalid recoil key";return false;}
   x.r=math::QuatNormalize(x.r);x.s={1,1,1};
  }
  peakTranslation_=peakRotation_=0;for(size_t i=0;i<count;++i){peakTranslation_=std::max(peakTranslation_,Translation(keys_[i].delta));peakRotation_=std::max(peakRotation_,Rotation(keys_[i].delta));}
  in>>std::ws;if(in.peek()!=std::char_traits<char>::eof()){error="trailing recoil data";return false;}
  if(keys_[0].seconds!=0||std::fabs(keys_[count-1].seconds-duration)>.000001f||!Identity(keys_[0].delta)||!Identity(keys_[count-1].delta)){error="recoil endpoints must be identity";return false;}
  // Exact endpoints prevent roundoff and authored residuals from accumulating.
  keys_[0].delta=keys_[count-1].delta=ov::Xf{};keys_[count-1].seconds=duration;count_=count;duration_=duration;return true;
 }
 bool Valid()const{return count_>=3;}
 float Duration()const{return duration_;}
 float PeakTranslation()const{return peakTranslation_;}
 float PeakRotation()const{return peakRotation_;}
 size_t Count()const{return count_;}
 bool Sample(double seconds,ov::Xf& out)const{
  out={};if(!Valid()||!std::isfinite(seconds)||seconds<0)return false;
  if(seconds==0||seconds>=duration_)return true;
  size_t low=0,high=count_-1;
  while(high-low>1){const size_t middle=(low+high)/2;if(keys_[middle].seconds<=seconds)low=middle;else high=middle;}
  const float weight=static_cast<float>((seconds-keys_[low].seconds)/(keys_[high].seconds-keys_[low].seconds));
  out=ov::BlendXf(keys_[low].delta,keys_[high].delta,weight);return Unit(out);
 }
private:
 std::array<Key,kMaxSamples> keys_{};size_t count_=0;float duration_=0,peakTranslation_=0,peakRotation_=0;
};
struct Binding {
 uint64_t weapon=0,rig=0,generation=0;
 bool Valid()const{return weapon&&rig&&generation;}
 bool operator==(const Binding&)const=default;
};
class Player {
public:
 void Reset(){active_=false;clip_=nullptr;binding_={};start_=last_=0;serial_=0;carry_={};restartWeight_=1;limits_={};gain_=1;prev_={};carrySeconds_=0;}
 bool Active()const{return active_;}
 // Multiply the caller's independent admission fade by this envelope. The final
 // 50ms return solved arms to their ordinary AO/grip baseline even when a zero
 // recoil IK solve would apply soft reach. A retrigger retains the exact current
 // weight and recovers smoothly, rather than jumping back to full IK ownership.
 float TailWeight(double now)const{
  if(!active_||!clip_||!clip_->Valid()||!std::isfinite(now)||now<last_)return 0;
  const double elapsed=now-start_,duration=clip_->Duration();
  if(elapsed<0||elapsed>=duration)return 0;
  const double edge=std::min(.05,duration*.5); // a 0.07-0.1 s automatic-weapon clip fades over half its length (old: a quarter = 17-25 ms, under one frame at 30 fps); >= 0.1 s clips (the Nue 0.8 s) keep 50 ms
  const auto smooth=[](double x){const float u=std::clamp(static_cast<float>(x),0.f,1.f);return u*u*(3-2*u);};
  const float recovery=restartWeight_+(1-restartWeight_)*smooth(elapsed/edge);
  return recovery*smooth((duration-elapsed)/edge);
 }
 // Caller selects the exact verified weapon curve. A duplicate/out-of-order shot
 // serial is ignored without disturbing the current recoil. Binding changes clear
 // old-weapon carry. No cyclic buffer or unbounded additive stack is retained.
 // A different clip of the SAME binding (hip <-> ADS inside a burst) keeps the offset the running clip has reached: it fades out
 // through the carry instead of vanishing in one frame.
 // limits: series-safe mode (see Limits); the default keeps the legacy accumulation.
 bool Trigger(const Clip& clip,Binding binding,uint64_t shotSerial,double now,const Limits& limits={}){
  if(!clip.Valid()||!binding.Valid()||!shotSerial||!std::isfinite(now)||now<0){Reset();return false;}
#if defined(UTPP_RECOIL_SWITCH_MUTATION) && UTPP_RECOIL_SWITCH_MUTATION == 1
  if(clip_ && (!(binding==binding_)||clip_!=&clip))Reset(); // mutation 1: a clip switch deletes the running offset (pre-carry build)
#else
  if(clip_ && !(binding==binding_))Reset();
#endif
  if(clip_&&now<last_){Reset();return false;}
  if(shotSerial<=serial_)return false;
  ov::Xf current;
  const Clip* running=clip_;
  const float previousWeight=active_&&running&&now-start_<running->Duration()?TailWeight(now):1.f;
  if(active_&&running)Sample(*running,binding,now,current);
  restartWeight_=previousWeight;
  carry_=current;clip_=&clip;binding_=binding;serial_=shotSerial;start_=last_=now;active_=true;
  // the carry of the same clip fades within a quarter of it (>= 0.1 s clips: at most 0.12 s); the carry of ANOTHER clip (hip <-> ADS)
  // always takes the full 0.12 s, whatever the length of the clip it moves to
  carrySeconds_=running&&running!=&clip?.12:std::min(.12,static_cast<double>(clip.Duration())*.25);
  limits_=limits;prev_=current;gain_=Headroom(clip,current);
  return true;
 }
 // false means inactive/invalid, with exact identity output. The caller then owns
 // its ordinary authored pose; there is no fallback to an FPP action in this API.
 bool Sample(const Clip& clip,Binding binding,double now,ov::Xf& out){
  out={};
  if(!active_)return false;
  if(!clip.Valid()||clip_!=&clip||!(binding==binding_)||!binding.Valid()||!std::isfinite(now)||now<last_){Reset();return false;}
  const double dt=now-last_;
  last_=now;const double elapsed=now-start_;
  if(elapsed==0){out=carry_;return true;}
  if(elapsed>=clip.Duration()){active_=false;carry_={};return false;}
  ov::Xf recoil;if(!clip.Sample(elapsed,recoil)){Reset();return false;}
  if(gain_<1)recoil=Scaled(recoil,gain_);
  const double carryDuration=carrySeconds_>0?carrySeconds_:std::min(.12,static_cast<double>(clip.Duration())*.25);
  const float u=std::clamp(static_cast<float>(elapsed/carryDuration),0.f,1.f);
  const float fade=1-u*u*u*(u*(u*6-15)+10);
  const ov::Xf carry=ov::BlendXf(ov::Xf{},carry_,fade);
  out.r=math::QuatNormalize(math::QuatMul(carry.r,recoil.r));
  out.t=carry.t+math::QuatRotate(carry.r,recoil.t);
  // Continuous saturation bounds rapid-fire accumulation without resetting at
  // a threshold. At a retrigger t0 the already-bounded current offset is retained.
  const float distance=Translation(out);if(distance>kMaxTranslation)out.t=out.t*(kMaxTranslation/distance);
  const float angle=Rotation(out);if(angle>kMaxRotation){
   const float axisLength=std::sqrt(out.r.x*out.r.x+out.r.y*out.r.y+out.r.z*out.r.z);
   const float factor=std::sin(kMaxRotation*.5f)/axisLength;
   out.r={out.r.x*factor,out.r.y*factor,out.r.z*factor,std::copysign(std::cos(kMaxRotation*.5f),out.r.w)};
  }
  if(!Unit(out)){Reset();out={};return false;}
  if(limits_.Slew()){out=Limited(prev_,out,dt>0?dt:0,limits_);prev_=out;}
  return true;
 }
private:
 // The new kick gets the headroom the series cap leaves above the offset that is still carried (1 = untouched).
 float Headroom(const Clip& clip,const ov::Xf& current)const{
  if(!limits_.Headroom())return 1;
  float gain=1;
  if(clip.PeakTranslation()>0)gain=std::min(gain,(limits_.seriesTranslation-Translation(current))/clip.PeakTranslation());
  if(clip.PeakRotation()>0)gain=std::min(gain,(limits_.seriesRotation-Rotation(current))/clip.PeakRotation());
  return std::clamp(gain,0.f,1.f);
 }
 static ov::Xf Scaled(const ov::Xf& x,float factor){
  ov::Xf r=x;r.t=x.t*factor;
  const float axis=std::sqrt(x.r.x*x.r.x+x.r.y*x.r.y+x.r.z*x.r.z);
  if(axis>1e-12f){const float half=std::atan2(axis,x.r.w)*factor,k=std::sin(half)/axis;r.r={x.r.x*k,x.r.y*k,x.r.z*k,std::cos(half)};}
  return r;
 }
 // Move from `from` toward `to` by at most speed * dt (translation length, rotation angle).
 static ov::Xf Limited(const ov::Xf& from,const ov::Xf& to,double dt,const Limits& limits){
  ov::Xf r=to;
  const float maxT=limits.speedTranslation*static_cast<float>(dt),maxR=limits.speedRotation*static_cast<float>(dt);
  const math::Vec3 d=to.t-from.t;const float length=std::sqrt(math::Dot(d,d));
  if(length>maxT)r.t=from.t+d*(length>0?maxT/length:0.f);
  math::Quat rel=math::QuatNormalize(math::QuatMul(math::QuatInv(from.r),to.r));
  if(rel.w<0)rel=math::QuatNeg(rel);
  const float axis=std::sqrt(rel.x*rel.x+rel.y*rel.y+rel.z*rel.z),angle=2*std::atan2(axis,rel.w);
  if(angle>maxR&&axis>1e-12f){const float k=std::sin(maxR*.5f)/axis;r.r=math::QuatNormalize(math::QuatMul(from.r,math::Quat{rel.x*k,rel.y*k,rel.z*k,std::cos(maxR*.5f)}));}
  return r;
 }
 const Clip* clip_=nullptr;Binding binding_;ov::Xf carry_,prev_;Limits limits_;double start_=0,last_=0,carrySeconds_=0;uint64_t serial_=0;bool active_=false;float restartWeight_=1,gain_=1;
};
} // namespace utpp::mm::authored_recoil

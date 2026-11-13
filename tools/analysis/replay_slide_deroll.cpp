// Offline replay of the slide-reload torso counter-roll (src/mm/SlideDeRoll.hpp + SlideDeRoll / SlideRollHold / UpperEnvelope in
// src/mm/ReloadContactFade.hpp) on a recorded pose CSV (utpp-pose-<T0>.csv, columns as in replay_pose_csv_reload.cpp: s0 = game
// pose, mm = published MM snapshot pose; local records, ov bone order). The composition mirrors MotionDriver: the MM pose of the
// frame is de-rolled (SlideReloadTorsoStepLocked), then the weapon-action bones (integ::IsWeaponActionBone: clavicles, arms,
// hands, weapons) take the game (s0) locals with the native share 1 - upperScale (ApplyLocked reloadScale), the rest stays MM.
// Cases per frame (model space, metres -> cm, Z up):
//   base : native arms (share 1) on the recorded MM torso (no uprighting)  - the failure this fix addresses
//   roll : native arms (share 1) on the de-rolled (roll only) MM torso
//   full : native arms (share 1) on the fully uprighted torso (slide_deroll::Apply weight 1: de-roll + de-pitch)
//   prod : production timeline (uprighting weight, envelope, hold on the roll left, authored fade)
//   old  : production timeline without the uprighting (the hold on the raw roll, the former policy)
// The attack of the weight is rt::kDeRollAttackSeconds as compiled (rebuild with another value to compare).
// Slide flag: not in the CSV; every reload frame of the chosen episode counts as a slide frame (argument slide=1), the raw roll
// is printed so the choice can be checked.
// Build (x64 Native Tools prompt): cl /std:c++20 /O2 /EHsc /D_CRT_SECURE_NO_WARNINGS tools/analysis/replay_slide_deroll.cpp
// Args: pose.csv woman.utppov [episode=-1 (last) | index | -2 (all) | -9 (list only)] [overlay=3] [slide=1] [frames.csv|-]
#include "../../plugin/UTPP/src/mm/GameIntegration.hpp"
#include "../../plugin/UTPP/src/mm/OverlayFile.hpp"
#include "../../plugin/UTPP/src/mm/ReloadContactFade.hpp"
#include "../../plugin/UTPP/src/mm/SlideDeRoll.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace utpp::mm;
namespace rt=reload_contacts_runtime;
namespace sd=slide_deroll;

struct Row{long long ticks=0,seq=0;int action=0,overlay=0,aiming=0,bone=0;float s0[12];float mm[7];};
static bool ParseRow(const std::string& line,Row& r)
{
    const char* p=line.c_str();char* end=nullptr;double f[58];int n=0;
    while(*p&&n<58)
    {
        f[n]=std::strtod(p,&end);
        if(end==p){f[n]=std::nan("");while(*p&&*p!=',')++p;}
        else p=end;
        ++n;
        if(*p==',')++p;else break;
    }
    if(n<58)return false;
    r.ticks=(long long)f[0];r.seq=(long long)f[4];r.action=(int)f[6];r.overlay=(int)f[7];r.aiming=(int)f[8];r.bone=(int)f[12];
    for(int k=0;k<12;++k)r.s0[k]=(float)f[15+k];
    for(int k=0;k<7;++k)r.mm[k]=(float)f[51+k];
    return true;
}
struct Frame{long long ticks=0;int action=0,overlay=0,aiming=0;ov::Pose game,mm;};
template<class Fn>static bool ForEachFrame(const char* path,const ov::Skeleton& sk,bool wantPoses,Fn fn)
{
    std::ifstream in(path);if(!in)return false;std::string line;
    Frame f;bool have=false;long long key=-1,keySeq=-1;
    auto start=[&](const Row& r){f=Frame{};f.ticks=r.ticks;f.action=r.action;f.overlay=r.overlay;f.aiming=r.aiming;f.game.n=f.mm.n=sk.n;
                                 for(int i=0;i<sk.n;++i){f.game.b[i]=ov::Xf{};f.mm.b[i]=ov::Xf{};}have=true;key=r.ticks;keySeq=r.seq;};
    while(std::getline(in,line))
    {
        if(line.empty()||line[0]=='#'||line[0]=='t')continue;
        Row r;
        if(!wantPoses)
        {
            // header fields only (fast path): ticks, seq, action, overlay, aiming
            const char* p=line.c_str();char* e=nullptr;long long v[9];int n=0;
            while(n<9){v[n]=std::strtoll(p,&e,10);if(e==p)break;p=e;++n;if(*p==',')++p;else break;}
            if(n<9)continue;
            r.ticks=v[0];r.seq=v[4];r.action=int(v[6]);r.overlay=int(v[7]);r.aiming=int(v[8]);
        }
        else if(!ParseRow(line,r))continue;
        if(!have||r.ticks!=key||r.seq!=keySeq){if(have&&!fn(f))return true;start(r);}
        if(wantPoses&&r.bone>=0&&r.bone<sk.n)
        {
            ov::Xf& g=f.game.b[r.bone];g.t={r.s0[0],r.s0[1],r.s0[2]};g.r=math::QuatNormalize({r.s0[4],r.s0[5],r.s0[6],r.s0[7]});
            ov::Xf& m=f.mm.b[r.bone];m.t={r.mm[0],r.mm[1],r.mm[2]};m.r=math::QuatNormalize({r.mm[3],r.mm[4],r.mm[5],r.mm[6]});
        }
    }
    if(have)fn(f);
    return true;
}
static math::Vec3 Pos(const ov::Skeleton& sk,const ov::Pose& p,int b){math::Quat q;math::Vec3 t;sd::ModelOf(sk,p,b,q,t);return t;}
static math::Vec3 InFrame(const ov::Skeleton& sk,const ov::Pose& p,int frameBone,math::Vec3 x)
{
    math::Quat q;math::Vec3 t;sd::ModelOf(sk,p,frameBone,q,t);return math::QuatRotate(math::QuatInv(q),x-t);
}
static float Len(math::Vec3 v){return std::sqrt(math::Dot(v,v));}
// roll-only levelling (step 1 of slide_deroll::Apply alone, for comparison)
static void RollOnly(const ov::Skeleton& sk,ov::Pose& p,const sd::Bones& b)
{
    math::Vec3 l,r;if(!sd::ShoulderLine(sk,p,b,l,r))return;
    const math::Vec3 d=r-l,dh{d.x,d.y,0.0f},ax=math::Cross(d,dh);const float al=Len(ax),dl=Len(d);
    if(al>1e-9f&&dl>1e-6f)sd::RotateChain(sk,p,b,ax*(1.0f/al),std::asin(std::clamp(std::fabs(d.z)/dl,0.0f,1.0f)));
}
static float Lean(const ov::Skeleton& sk,const ov::Pose& p,const sd::Bones& b){float l=0.0f;sd::LeanDeg(sk,p,b,l);return l;}static math::Quat Nlerp(math::Quat a,math::Quat b,float w)
{
    if(math::QuatDot(a,b)<0)b=math::QuatNeg(b);
    return math::QuatNormalize({a.x+(b.x-a.x)*w,a.y+(b.y-a.y)*w,a.z+(b.z-a.z)*w,a.w+(b.w-a.w)*w});
}
// mask bones: game locals with the native share (1 - mmShare), the rest = torso pose
static ov::Pose Compose(const ov::Pose& torso,const ov::Pose& game,const std::vector<char>& mask,float mmShare)
{
    ov::Pose o=torso;
    for(int i=0;i<o.n;++i)if(mask[size_t(i)])
    {
        o.b[i].r=Nlerp(game.b[i].r,torso.b[i].r,mmShare);
        o.b[i].t=math::Lerp(game.b[i].t,torso.b[i].t,mmShare);
    }
    return o;
}
struct Track{std::vector<double> t;std::vector<math::Vec3> wModel,wSpine;std::vector<float> whd;};
struct Summary{float whdMax=-1e9f,d05Model=0,d05Spine=0;};
static Summary Summarize(const Track& k,double tFrom,double tTo)
{
    Summary s;
    for(size_t a=0;a<k.t.size();++a)
    {
        if(k.t[a]<tFrom||k.t[a]>tTo)continue;
        s.whdMax=std::fmax(s.whdMax,k.whd[a]);
        for(size_t b=a+1;b<k.t.size()&&k.t[b]-k.t[a]<=.05+1e-6;++b)
        {
            s.d05Model=std::fmax(s.d05Model,Len(k.wModel[b]-k.wModel[a])*100);
            s.d05Spine=std::fmax(s.d05Spine,Len(k.wSpine[b]-k.wSpine[a])*100);
        }
    }
    return s;
}
int main(int argc,char** argv)
{
    if(argc<3){std::fprintf(stderr,"usage: pose.csv woman.utppov [episode=-1] [overlay=3] [slide=1] [frames.csv|-]\n");return 2;}
    const int episodeArg=argc>3?std::atoi(argv[3]):-1,overlayWant=argc>4?std::atoi(argv[4]):3;const bool slideAll=argc>5?std::atoi(argv[5])!=0:true;

    FILE* out=(argc>6&&std::string(argv[6])!="-")?std::fopen(argv[6],"w"):nullptr;
    ov::OverlayFile file;std::string error;
    if(!file.LoadFromFile(argv[2],ov::kRigIdentityWomanOv,error)){std::cerr<<error<<"\n";return 2;}
    const ov::Skeleton& sk=file.Skel();
    auto find=[&](const char* n){for(int i=0;i<sk.n;++i)if(file.BoneName(i)==n)return i;return -1;};
    sd::Bones bones;bones.spine[0]=find("Spine1");bones.spine[1]=find("Spine2");bones.spine[2]=find("Spine3");
    bones.lsh=sk.parent[find("LeftArm")];bones.rsh=sk.parent[find("RightArm")];bones.neck=find("Neck");
    const int head=find("Head"),weapon=find("WeaponRight"),spine3=bones.spine[2];
    if(!bones.Valid(sk)||!bones.NeckValid(sk)||head<0||weapon<0){std::cerr<<"rig: Spine1..3 / clavicles / Neck / Head / WeaponRight not found\n";return 2;}
    std::vector<char> mask(size_t(sk.n),0);int maskN=0;
    for(int i=2;i<sk.n;++i)if(integ::IsWeaponActionBone(file.BoneName(i))){mask[size_t(i)]=1;++maskN;}
    // pass 1: reload episodes (consecutive action-2 frames of the wanted overlay)
    struct Ep{long long first=0,last=0;int frames=0,aimBefore=0;};std::vector<Ep> eps;bool in=false;int prevAim=0;
    if(!ForEachFrame(argv[1],sk,false,[&](const Frame& f){
        const bool r=f.action==2&&f.overlay==overlayWant;
        if(r&&!in){eps.push_back({f.ticks,f.ticks,0,prevAim});in=true;}
        if(r){eps.back().last=f.ticks;++eps.back().frames;}else in=false;
        prevAim=f.aiming;return true;})){std::cerr<<"cannot read "<<argv[1]<<"\n";return 2;}
    std::printf("episodes (action 2, overlay %d): %zu\n",overlayWant,eps.size());
    for(size_t i=0;i<eps.size();++i)std::printf("  #%zu first=%lld frames=%d len=%.3fs aimBefore=%d\n",i,eps[i].first,eps[i].frames,
                                                double(eps[i].last-eps[i].first)/1e7,eps[i].aimBefore);
    if(eps.empty()||episodeArg==-9)return eps.empty()?2:0; // -9: list only
    // -2: every episode in one pass (compact lines); otherwise one episode (default the last) in detail
    std::vector<size_t> picks;
    if(episodeArg==-2)for(size_t i=0;i<eps.size();++i)picks.push_back(i);
    else{const size_t pick=episodeArg<0?eps.size()-1:size_t(episodeArg);if(pick>=eps.size())return 2;picks.push_back(pick);}
    std::printf("mask bones %d, slide flag on every reload frame: %d\n",maskN,slideAll?1:0);
    struct Run
    {
        Ep ep;long long from=0,to=0,prevTicks=-1;
        rt::UpperEnvelope env;rt::SlideRollHold hold,holdOld;rt::SlideDeRoll der;
        Track base,roll,full,prod,old;float rollMax=0,rollMin=1e9f,leftMaxAfter=0,holdMax=0,wMax=0;double tHoldZero=-1,tNativeFull=-1;
        double pitchSum=0,prodLeanSum=0;int pitchN=0,prodLeanN=0;float pitchMin=1e9f,pitchMax=-1e9f,prodLeanErrMax=0;
    };
    std::vector<Run> runs;
    for(const size_t i:picks){Run r;r.ep=eps[i];r.from=r.ep.first-static_cast<long long>(.10*1e7);r.to=r.ep.last+static_cast<long long>(.40*1e7);runs.push_back(r);}
    if(out)std::fprintf(out,"t,action,aiming,rollRaw,deroll,rollLeft,hold,holdOld,env,u,uOld,whdBase,whdFull,whdProd,whdOld\n");
    auto record=[&](Track& k,double t,const ov::Pose& p){
        const math::Vec3 w=Pos(sk,p,weapon),h=Pos(sk,p,head);
        k.t.push_back(t);k.wModel.push_back(w);k.wSpine.push_back(InFrame(sk,p,spine3,w));k.whd.push_back((w.z-h.z)*100);};
    const long long lastTo=runs.back().to;
    // pass 2: replay
    ForEachFrame(argv[1],sk,true,[&](const Frame& f){
        if(f.ticks>lastTo)return false;
        for(Run& R:runs)
        {
            if(f.ticks<R.from||f.ticks>R.to)continue;
            const Ep& ep=R.ep;
            const float dt=R.prevTicks<0?1.0f/60:float(double(f.ticks-R.prevTicks)/1e7);R.prevTicks=f.ticks;
            const double t=double(f.ticks-ep.first)/1e7;
            const bool reload=f.action==2&&f.overlay==overlayWant&&f.ticks>=ep.first&&f.ticks<=ep.last;
            const bool slide=reload&&slideAll;
            const float roll=sd::RollDeg(sk,f.mm,bones);
            // production: de-roll (gate = pistol reload on a slide frame), hold on the roll left, envelope (pistol, aim latched at the edge)
            const float w=R.der.Step(dt,reload&&slide,roll);
            ov::Pose torso=f.mm;if(w>0)sd::Apply(sk,torso,bones,w);
            const float left=sd::RollDeg(sk,torso,bones);
            const float h=R.hold.Step(dt,reload,slide,slide?left:0.0f,rt::SlideDeRoll::Engaged(reload&&slide));
            const float hOld=R.holdOld.Step(dt,reload,slide,slide?roll:0.0f);
            R.env.Step(dt,reload,ep.aimBefore!=0,true);
            const float native=rt::AuthoredFadeWeight(R.env.weight);
            const float u=std::fmax(1.0f-native,h),uOld=std::fmax(1.0f-native,hOld);
            ov::Pose levelled=f.mm;RollOnly(sk,levelled,bones);
            ov::Pose upright=f.mm;sd::Apply(sk,upright,bones,1.0f);
            if(reload)
            {
                const float p=Lean(sk,levelled,bones);
                R.pitchSum+=p;++R.pitchN;R.pitchMin=std::fmin(R.pitchMin,p);R.pitchMax=std::fmax(R.pitchMax,p);
                if(t>.35){const float lp=Lean(sk,torso,bones);R.prodLeanSum+=lp;++R.prodLeanN;
                          R.prodLeanErrMax=std::fmax(R.prodLeanErrMax,std::fabs(lp-sd::kDePitchTargetDeg));}
            }
            record(R.base,t,Compose(f.mm,f.game,mask,0.0f));
            record(R.roll,t,Compose(levelled,f.game,mask,0.0f));
            record(R.full,t,Compose(upright,f.game,mask,0.0f));
            record(R.prod,t,Compose(torso,f.game,mask,u));
            record(R.old,t,Compose(f.mm,f.game,mask,uOld));
            if(reload){R.rollMax=std::fmax(R.rollMax,roll);R.rollMin=std::fmin(R.rollMin,roll);R.holdMax=std::fmax(R.holdMax,h);R.wMax=std::fmax(R.wMax,w);
                       if(t>rt::kDeRollAttackSeconds+.02)R.leftMaxAfter=std::fmax(R.leftMaxAfter,left);
                       if(h==0.0f&&R.tHoldZero<0)R.tHoldZero=t;if(u==0.0f&&R.tNativeFull<0)R.tNativeFull=t;}
            if(out&&runs.size()==1)std::fprintf(out,"%.4f,%d,%d,%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n",t,f.action,f.aiming,roll,w,left,h,
                                                hOld,R.env.weight,u,uOld,R.base.whd.back(),R.full.whd.back(),R.prod.whd.back(),R.old.whd.back());
        }
        return true;});
    for(size_t k=0;k<runs.size();++k)
    {
        const Run& R=runs[k];const double end=double(R.ep.last-R.ep.first)/1e7;
        std::printf("episode #%zu (%.3fs, aimBefore %d): raw roll %.1f..%.1f deg, de-roll w max %.3f, roll left after attack %.2f deg, hold max %.3f "
                    "(0 at +%.3fs), native full at +%.3fs; levelled torso lean (Spine3->Neck, + forward) mean %.1f [%.1f..%.1f] deg, prod lean mid-reload mean %.2f (max err %.2f) deg, attack %.2fs\n",picks[k],end,R.ep.aimBefore,R.rollMin,R.rollMax,R.wMax,R.leftMaxAfter,R.holdMax,R.tHoldZero,R.tNativeFull,R.pitchN?R.pitchSum/R.pitchN:0.0,double(R.pitchMin),double(R.pitchMax),
                    R.prodLeanN?R.prodLeanSum/R.prodLeanN:0.0,double(R.prodLeanErrMax),double(rt::kDeRollAttackSeconds));
        const struct{const char* name;const Track* k;}cases[]={{"base (native arms, rolled MM torso)",&R.base},{"roll (native arms, de-rolled only)",&R.roll},
                                                             {"full (native arms, de-roll + de-pitch)",&R.full},{"prod (upright + envelope + hold)",&R.prod},
                                                             {"old  (hold on raw roll, no uprighting)",&R.old}};
        for(const auto& c:cases)
        {
            if(c.k->t.empty())continue;
            const Summary all=Summarize(*c.k,-.05,end+.05),edge=Summarize(*c.k,-.05,.35),body=Summarize(*c.k,.35,end),ex=Summarize(*c.k,end-.05,end+.40);
            std::printf("  %-37s weapon-head max %+6.1f cm (edge %+6.1f, body %+6.1f) | move/.05s model: edge %5.1f body %5.1f exit %5.1f cm;"
                        " Spine3 frame: edge %5.1f body %5.1f exit %5.1f cm\n",c.name,all.whdMax,edge.whdMax,body.whdMax,edge.d05Model,body.d05Model,
                        ex.d05Model,edge.d05Spine,body.d05Spine,ex.d05Spine);
        }
    }
    if(out)std::fclose(out);
    return 0;
}

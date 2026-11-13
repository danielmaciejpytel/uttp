// Offline test of the native-reload upper-body handback envelope (ReloadContactFade.hpp UpperEnvelope).
// No game process. Fixtures are real recorded weapon positions in the Spine3 frame (cm), 34 frames from the
// reload edge - 2 frames: native = game pose (stage 0), mm = published MM pose (stage 3, aim flag dropped by the
// game after the edge), hold = MM pose frozen at the pre-edge frame (what AimHold keeps).
//   kRun1: 21.10 03:37 k5 (03:39:12.36, aim reload while running), kRun2: 20.10 10:31 k5 (10:31:58.96).
// Mutation 1 (UTPP_RELOAD_ENVELOPE_MUTATION=1) restores the old .12s aim-interrupt ramp: the test must fail.
#include "../src/mm/ReloadContactFade.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
using namespace utpp::mm::reload_contacts_runtime;
static int g_fail=0,g_pass=0;
static void Check(bool ok,const char* what){(ok?g_pass:g_fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);}

// N37 03:39:12.36 k5 edge at index 2
static const float kRun1_dt[34]={0.01541f,0.01845f,0.01541f,0.01843f,0.01518f,0.01608f,0.01706f,0.01673f,0.01609f,0.01714f,0.01921f,0.01391f,0.01650f,0.01659f,0.01651f,0.01745f,0.01636f,0.01801f,0.01591f,0.01628f,0.01668f,0.01670f,0.01689f,0.01809f,0.01495f,0.01674f,0.01761f,0.01582f,0.01662f,0.01858f,0.01469f,0.01701f,0.01565f,0.01687f};
static const float kRun1_native[34][3]={{22.57f,-39.55f,-0.04f},{21.78f,-39.22f,0.99f},{28.76f,-48.42f,0.01f},{35.07f,-50.90f,-0.32f},{38.32f,-52.37f,-0.27f},{39.62f,-53.99f,-0.20f},{39.97f,-55.58f,-0.29f},{39.98f,-56.79f,-0.47f},{40.05f,-57.75f,-0.95f},{40.27f,-58.58f,-1.39f},{40.22f,-58.09f,-1.14f},{40.54f,-57.77f,-0.11f},{40.42f,-57.45f,0.57f},{40.44f,-57.39f,0.90f},{41.20f,-57.17f,0.29f},{41.49f,-57.31f,1.08f},{41.57f,-57.50f,2.38f},{41.56f,-57.07f,3.45f},{41.28f,-56.80f,3.83f},{40.77f,-56.88f,3.78f},{40.56f,-56.76f,4.12f},{40.35f,-56.57f,4.37f},{40.11f,-56.46f,4.66f},{39.93f,-56.39f,5.07f},{39.76f,-56.33f,5.43f},{39.52f,-56.22f,5.75f},{39.25f,-56.09f,6.09f},{39.00f,-55.92f,6.42f},{38.76f,-55.68f,6.73f},{38.42f,-55.34f,7.07f},{38.10f,-54.97f,7.30f},{37.73f,-54.53f,7.69f},{37.33f,-54.09f,8.28f},{36.87f,-53.55f,8.88f}};
static const float kRun1_mm[34][3]={{56.81f,-57.08f,-10.83f},{57.06f,-56.94f,-10.56f},{57.35f,-56.30f,-10.07f},{53.04f,-59.74f,-10.38f},{52.33f,-59.72f,-10.04f},{51.55f,-59.71f,-9.53f},{50.79f,-59.62f,-8.93f},{49.98f,-59.51f,-8.22f},{49.17f,-59.38f,-7.46f},{48.26f,-59.22f,-6.66f},{47.03f,-59.10f,-5.76f},{45.90f,-59.07f,-5.00f},{44.49f,-59.00f,-4.32f},{42.71f,-59.11f,-3.24f},{40.89f,-59.13f,-2.12f},{38.83f,-59.13f,-0.94f},{36.74f,-59.04f,0.15f},{34.60f,-58.75f,0.82f},{32.80f,-58.37f,1.14f},{31.11f,-57.82f,1.09f},{29.62f,-57.13f,0.66f},{28.21f,-56.36f,0.13f},{26.95f,-55.49f,-0.58f},{25.61f,-54.50f,-1.29f},{24.56f,-53.61f,-1.93f},{23.41f,-52.64f,-2.50f},{22.24f,-51.62f,-3.07f},{21.09f,-50.60f,-3.47f},{19.94f,-49.60f,-3.68f},{18.70f,-48.50f,-3.68f},{17.71f,-47.64f,-3.49f},{16.66f,-46.72f,-3.10f},{15.64f,-45.80f,-2.55f},{15.63f,-45.79f,-2.55f}};
static const float kRun1_hold[34][3]={{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f},{57.06f,-56.94f,-10.56f}};
// R20 10:31:58.96 k5 edge at index 2
static const float kRun2_dt[34]={0.01546f,0.01831f,0.01546f,0.01837f,0.01473f,0.01703f,0.01625f,0.01696f,0.01665f,0.01838f,0.01486f,0.01661f,0.01692f,0.01631f,0.01749f,0.01595f,0.01788f,0.01555f,0.01667f,0.01673f,0.01659f,0.01682f,0.01783f,0.01523f,0.01657f,0.01661f,0.01674f,0.01717f,0.01669f,0.01800f,0.01481f,0.01672f,0.01698f,0.01714f};
static const float kRun2_native[34][3]={{22.81f,-39.11f,0.18f},{21.91f,-38.83f,1.21f},{29.06f,-47.97f,0.50f},{35.32f,-50.86f,0.07f},{38.38f,-52.33f,-0.11f},{39.64f,-54.00f,-0.17f},{39.97f,-55.58f,-0.29f},{39.98f,-56.79f,-0.47f},{40.06f,-57.78f,-0.96f},{40.32f,-58.68f,-1.39f},{40.32f,-58.26f,-1.13f},{40.54f,-57.76f,-0.10f},{40.43f,-57.45f,0.58f},{40.44f,-57.38f,0.92f},{41.24f,-57.16f,0.30f},{41.49f,-57.30f,1.09f},{41.56f,-57.46f,2.42f},{41.61f,-57.11f,3.44f},{41.29f,-56.80f,3.84f},{40.77f,-56.87f,3.79f},{40.57f,-56.75f,4.13f},{40.36f,-56.56f,4.38f},{40.11f,-56.43f,4.68f},{39.97f,-56.38f,5.04f},{39.77f,-56.32f,5.43f},{39.53f,-56.21f,5.76f},{39.26f,-56.08f,6.09f},{39.01f,-55.91f,6.42f},{38.76f,-55.67f,6.73f},{38.44f,-55.34f,7.06f},{38.11f,-54.97f,7.29f},{37.73f,-54.53f,7.69f},{37.32f,-54.07f,8.29f},{36.86f,-53.54f,8.88f}};
static const float kRun2_mm[34][3]={{56.02f,-58.07f,12.34f},{56.21f,-57.93f,12.28f},{56.69f,-58.05f,12.00f},{54.72f,-59.12f,11.08f},{53.92f,-59.33f,9.89f},{52.99f,-59.52f,8.55f},{51.95f,-59.73f,6.75f},{50.82f,-59.88f,4.69f},{49.60f,-59.94f,2.64f},{48.17f,-59.90f,0.41f},{46.93f,-59.78f,-1.34f},{45.32f,-59.64f,-3.25f},{43.64f,-59.39f,-5.08f},{41.73f,-59.11f,-7.04f},{39.67f,-58.70f,-8.92f},{37.66f,-58.29f,-10.24f},{35.47f,-57.78f,-11.32f},{33.70f,-57.28f,-11.84f},{31.93f,-56.70f,-12.04f},{30.34f,-56.08f,-11.99f},{28.95f,-55.43f,-11.61f},{27.61f,-54.73f,-11.11f},{26.34f,-54.00f,-10.32f},{25.26f,-53.34f,-9.61f},{24.17f,-52.60f,-8.74f},{23.08f,-51.82f,-7.88f},{22.06f,-51.02f,-6.97f},{21.02f,-50.19f,-6.08f},{19.99f,-49.35f,-5.22f},{18.84f,-48.42f,-4.35f},{17.84f,-47.62f,-3.70f},{16.72f,-46.72f,-3.07f},{15.63f,-45.79f,-2.55f},{15.63f,-45.79f,-2.55f}};
static const float kRun2_hold[34][3]={{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f},{56.21f,-57.93f,12.28f}};


struct Fixture{const char* name;const float* dt;const float (*native)[3];const float (*mm)[3];const float (*hold)[3];};
static const int kN=34;

// Same arithmetic as the pre-change inline ramp in MotionDriver::Step (reference for the unchanged path).
static float LegacyStep(float weight,float dt,bool reload)
{
    const float speed=reload ? (1.0f/0.12f) : (1.0f/0.20f);
    const float step=std::fmin(dt,0.25f)*speed;
    return reload ? std::fmin(1.0f,weight+step) : std::fmax(0.0f,weight-step);
}

// Weapon trajectory (Spine3 frame, cm) of the blended pose: lerp(MM source, native, AuthoredFadeWeight(envelope)).
// Returns the largest weapon displacement within any 0.1s (or 0.05s) window, frames edge-1 .. edge+0.45s.
static void Trajectory(const Fixture& f,bool aimBeforeEdge,bool useHold,float& d10,float& d05)
{
    UpperEnvelope env;float pos[kN][3];float t[kN];float clock=0;
    for(int i=0;i<kN;++i)
    {
        clock+=f.dt[i];t[i]=clock;
        const bool reload=i>=2;
        env.Step(f.dt[i],reload,aimBeforeEdge);
        const bool hold=useHold && env.AimHold();
        const float w=AuthoredFadeWeight(env.weight);
        const float (*src)[3]=hold ? f.hold : f.mm;
        for(int k=0;k<3;++k)pos[i][k]=src[i][k]+(f.native[i][k]-src[i][k])*w;
    }
    d10=d05=0;
    const float edge=t[2];
    for(int a=1;a<kN;++a)
    {
        if(t[a]-edge<-0.05f||t[a]-edge>.45f)continue;
        for(int b=a+1;b<kN;++b)
        {
            if(t[b]-edge>.45f)break;
            const float dx=pos[b][0]-pos[a][0],dy=pos[b][1]-pos[a][1],dz=pos[b][2]-pos[a][2];
            const float d=std::sqrt(dx*dx+dy*dy+dz*dz);
            if(t[b]-t[a]<=.1f+1e-6f)d10=std::fmax(d10,d);
            if(t[b]-t[a]<=.05f+1e-6f)d05=std::fmax(d05,d);
        }
    }
}

int main()
{
    // 1. A reload that does not interrupt ADS keeps the former ramp bit for bit (entry, hold, exit, odd dt).
    {
        UpperEnvelope env;float legacy=0;bool same=true;
        const float dts[]={1.f/60,1.f/120,.0333f,.4f,1.f/30,.001f,1.f/75};
        for(int cycle=0;cycle<6;++cycle)
            for(int i=0;i<90;++i)
            {
                const bool reload=(i/30)%2==0;const float dt=dts[(i+cycle)%7];
                legacy=LegacyStep(legacy,dt,reload);
                const float w=env.Step(dt,reload,false);
                same=same&&(w==legacy)&&(env.weight==legacy);
            }
        Check(same,"non-aim reload: envelope is bit-identical to the former .12s/.20s ramp");
        Check(!env.AimHold(),"non-aim reload never holds the aim overlay");
    }
    // 1b. Pistol reload that does not interrupt ADS: .24 s entry (smoothstep peak step <= 0.11 per 1/60 s instead of 0.21),
    // ADS keeps .30 s, a rifle (pistol=false) and the exit stay bit-identical.
    {
        UpperEnvelope env;env.Step(1.f/60,false,false,true);
        float prev=0,maxStep=0;int frames=0;
        while(env.weight<1.0f && frames<200)
        {
            const float w=env.Step(1.f/60,true,false,true);
            maxStep=std::fmax(maxStep,AuthoredFadeWeight(w)-AuthoredFadeWeight(prev));prev=w;++frames;
        }
        Check(frames>=14&&frames<=16,"pistol non-ADS reload reaches full native ownership in ~.24s (15 frames at 60fps)");
        Check(maxStep<0.11f,"pistol entry: authored fade peak step <= 0.11 per 1/60 s (was 0.21 with .12 s)");
        Check(!env.fromAim&&env.entryPistol,"pistol entry latched without ADS");
        env.Step(1.f/60,true,false,false); // latch must not change mid-reload
        Check(env.entryPistol,"pistol latch survives a pistol=false step inside the same reload");
        UpperEnvelope aim;for(int i=0;i<4;++i)aim.Step(1.f/60,false,true,true);
        int af=0;while(aim.weight<1.0f&&af<200){aim.Step(1.f/60,true,true,true);++af;}
        Check(aim.fromAim&&af>=17&&af<=19,"ADS-interrupting pistol reload keeps the .30 s ramp");
        UpperEnvelope rifle;int rf=0;rifle.Step(1.f/60,false,false,false);
        while(rifle.weight<1.0f&&rf<200){rifle.Step(1.f/60,true,false,false);++rf;}
        Check(rf>=7&&rf<=8,"non-pistol reload keeps the .12 s ramp (7-8 frames)");
    }
    // 2. Aim-interrupting reload: slower entry, same exit, hold only while blending in.
    {
        UpperEnvelope env;
        for(int i=0;i<10;++i)env.Step(1.f/60,false,true); // aiming, no reload
        Check(env.weight==0 && !env.AimHold(),"aiming without reload: nothing latched");
        float w=0;int framesTo1=0;bool holdThroughout=true,monotone=true;float prev=0;
        env.Step(1.f/60,true,false); // edge with the aim flag already false: not an aim reload
        Check(!env.fromAim&&!env.AimHold(),"edge with aim=false is a plain reload");
        env.Reset();
        for(int i=0;i<4;++i)env.Step(1.f/60,false,true);
        env.Step(1.f/60,true,true); // edge with aim flag true
        Check(env.fromAim,"edge with aim=true latches an aim-interrupting reload");
        prev=env.weight;framesTo1=1;
        while(env.weight<1.0f && framesTo1<200)
        {
            holdThroughout=holdThroughout&&env.AimHold();
            w=env.Step(1.f/60,true,false); // aim flag is now false (game dropped it): latch must not change
            monotone=monotone&&w>=prev;prev=w;++framesTo1;
        }
        Check(env.fromAim&&holdThroughout&&monotone,"latched aim reload: hold lasts while blending in; flag change mid-reload ignored");
        Check(framesTo1>=17&&framesTo1<=19,"aim reload reaches full native ownership in ~.30s (18 frames at 60fps)");
        Check(!env.AimHold(),"hold released when the arms are fully native");
        env.Step(1.f/60,true,false);
        Check(env.weight==1.0f&&!env.AimHold(),"weight holds at 1 without re-arming the hold");
        const float out=env.Step(1.f/60,false,true);
        Check(std::fabs((1.0f-out)-1.f/60/.20f)<1e-6f && !env.AimHold(),"exit ramp is still .20s and carries no hold");
        // A new reload from hip fire after an aim reload must not inherit the latch.
        for(int i=0;i<30;++i)env.Step(1.f/60,false,false);
        env.Step(1.f/60,true,false);
        Check(!env.fromAim&&!env.AimHold(),"next reload re-samples the aim flag at its own edge");
        // Invalid dt leaves everything untouched.
        UpperEnvelope bad;bad.Step(1.f/60,true,true);const float before=bad.weight;
        bad.Step(std::numeric_limits<float>::quiet_NaN(),true,true);bad.Step(-1.f,false,false);bad.Step(0.f,false,false);
        Check(bad.weight==before&&bad.active&&bad.fromAim,"non-finite / non-positive dt is ignored");
    }
    // 3. Recorded aim-interrupt reloads: weapon motion at the reload start stays below 12cm per .1s (was 20+).
    const Fixture fixtures[]={
        {"03:37 k5",kRun1_dt,kRun1_native,kRun1_mm,kRun1_hold},
        {"20.10 10:31 k5",kRun2_dt,kRun2_native,kRun2_mm,kRun2_hold}};
    for(const Fixture& f:fixtures)
    {
        float legacy10,legacy05,fixed10,fixed05;
        Trajectory(f,false,false,legacy10,legacy05); // flag already false at the edge in the old code path: ramp .12, MM pops
        Trajectory(f,true,true,fixed10,fixed05);
        std::printf("  %s: legacy d10=%.1fcm d05=%.1fcm  new d10=%.1fcm d05=%.1fcm\n",f.name,legacy10,legacy05,fixed10,fixed05);
        char text[160];
        std::snprintf(text,sizeof text,"%s: legacy envelope reproduces the recorded jump (>15cm/.1s)",f.name);
        Check(legacy10>15.0f,text);
        std::snprintf(text,sizeof text,"%s: new envelope keeps the weapon below 12cm per .1s",f.name);
        Check(fixed10<12.0f,text);
        std::snprintf(text,sizeof text,"%s: new envelope keeps the weapon below 8cm per .05s",f.name);
        Check(fixed05<8.0f,text);
    }
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail==0?0:1;
}

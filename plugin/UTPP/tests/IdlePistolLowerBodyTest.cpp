// Targeted stance regression only; no full MM suite or game dependency.
#include <cstdio>
#include <memory>
#include "../src/mm/Overlay.hpp"
#include "../src/mm/OverlayFile.hpp"
using namespace utpp::mm::ov;
int main(int argc,char** argv)
{
    int checks=0, failures=0;
    auto check=[&](bool ok,const char* what){++checks;if(!ok){++failures;std::printf("FAIL %s\n",what);}};
    OverlayMachine m;m.style=Style::Feminine;
    const int idle=PoseKey(PoseOverlay::Pistol1H,AimState::Relaxed,false,false);
    auto curves=[](){Curves c;for(int i=0;i<kCurveCount;++i)c.Set(i,1.f);return c;};
    m.poses.Reset(idle);auto c=curves();PreserveIdlePistolLowerBody(m,true,c);
    check(c[kLegs]==0&&c[kPelvis]==0,"female relaxed standing removes lower overlay");
    check(c[kArmL]==1&&c[kArmR]==1&&c[kSpine]==1&&c[kHandR]==1,"weapon upper curves retained");
    auto no=curves();PreserveIdlePistolLowerBody(m,false,no);check(no[kLegs]==1&&no[kPelvis]==1,"ineligible unchanged");
    m.style=Style::Masculine;no=curves();PreserveIdlePistolLowerBody(m,true,no);check(no[kLegs]==1,"male unchanged");m.style=Style::Feminine;
    for(int key:{PoseKey(PoseOverlay::Pistol1H,AimState::Ready,false,false),PoseKey(PoseOverlay::Pistol1H,AimState::Aiming,false,false),PoseKey(PoseOverlay::Pistol1H,AimState::Relaxed,true,false),PoseKey(PoseOverlay::Pistol1H,AimState::Relaxed,false,true),PoseKey(PoseOverlay::Pistol2H,AimState::Relaxed,false,false),PoseKey(PoseOverlay::Rifle,AimState::Relaxed,false,false)})
    {m.poses.Reset(key);no=curves();PreserveIdlePistolLowerBody(m,true,no);check(no[kLegs]==1&&no[kPelvis]==1,"other pose context unchanged");}
    m.poses.Reset(idle);m.poses.SetTarget(PoseKey(PoseOverlay::Pistol1H,AimState::Relaxed,false,true),.2f);m.poses.Update(.1f);
    no=curves();PreserveIdlePistolLowerBody(m,true,no);check(no[kLegs]>.0f&&no[kLegs]<1.f,"crossfade preserves continuous lower weight");
    Skeleton sk;sk.n=12;int16_t parents[]={-1,0,1,1,1,4,5,6,7,7,7,9};for(int i=0;i<12;++i)sk.parent[i]=parents[i];
    const char* names[]={"Root","Hips","LeftUpLeg","RightUpLeg","Spine","Spine1","Spine2","Spine3","Neck","LeftShoulder","RightShoulder","LeftInHandThumb"};
    Regions regions;BuildRegions(sk,[&](int i){return std::string_view(names[i]);},regions);
    Pose loco,base,overlay,result;loco.n=base.n=overlay.n=result.n=12;
    for(int i=0;i<12;++i){loco.b[i]=base.b[i]=overlay.b[i]=Xf{};}
    overlay.b[1].t.y=.12f;overlay.b[2].r=Quat{0,0,.38268343f,.92387953f};overlay.b[3].r=overlay.b[2].r;overlay.b[9].r=overlay.b[2].r;
    Additive style;style.SetIdentity(12,true);style.b[2].r=overlay.b[2].r;
    LayerInputs in;in.L=&loco;in.B=&base;in.OP=&overlay;in.dP=&style;in.curves=c;
    auto scratch=std::make_unique<LayerScratch>();ComposeLayers(sk,regions,in,*scratch,result);
    for(int i:{1,2,3}){check(AngleDeg(loco.b[i].r,result.b[i].r)<.01f,"lower rotation matches locomotion");check(std::fabs(loco.b[i].t.y-result.b[i].t.y)<1e-6f,"pelvis/leg translation matches locomotion");}
    check(AngleDeg(loco.b[9].r,result.b[9].r)>10.f,"authored weapon shoulder remains active");
    // male V policy (Style::Masculine, explicit policy style): lower body kept in the Relaxed 1H idle only, never mixed between styles
    {
        OverlayMachine mm;mm.style=Style::Masculine;mm.poses.Reset(idle);auto mc=curves();
        PreserveIdlePistolLowerBody(mm,true,mc,mm.poses.WeightOf(idle),Style::Masculine);
        check(mc[kLegs]==0&&mc[kPelvis]==0&&mc[kArmL]==1&&mc[kArmR]==1&&mc[kSpine]==1,"male relaxed standing removes lower overlay only");
        auto none=curves();PreserveIdlePistolLowerBody(mm,false,none,1.f,Style::Masculine);check(none[kLegs]==1&&none[kPelvis]==1,"male ineligible unchanged");
        mm.style=Style::Feminine;none=curves();PreserveIdlePistolLowerBody(mm,true,none,1.f,Style::Masculine);check(none[kLegs]==1,"male policy leaves a feminine machine unchanged");
        mm.style=Style::Masculine;
        for(int key:{PoseKey(PoseOverlay::Pistol1H,AimState::Ready,false,false),PoseKey(PoseOverlay::Pistol1H,AimState::Aiming,false,false),PoseKey(PoseOverlay::Pistol1H,AimState::Relaxed,false,true),PoseKey(PoseOverlay::Pistol2H,AimState::Relaxed,false,false),PoseKey(PoseOverlay::Rifle,AimState::Relaxed,false,false)})
        {mm.poses.Reset(key);none=curves();PreserveIdlePistolLowerBody(mm,true,none,mm.poses.WeightOf(idle),Style::Masculine);check(none[kLegs]==1&&none[kPelvis]==1,"male other pose context unchanged");}
    }
    if(argc>2)
    {
        // patched male overlay (patch_male_pistol_idle_stance.py output): CRC/schema/rig gates and the four curve values
        OverlayFile file;std::string error;
        const bool loaded=file.LoadFromFile(argv[2],kRigIdentityManOv,error);
        check(loaded,"patched male overlay passes loader CRC/schema/rig gates");
        if(!loaded)std::printf("Loader: %s\n",error.c_str());
        if(loaded)
        {
            const int pose=file.FindPose("Pose_Pistol1H_Stand_Relax_Idle");check(pose>=0,"male relaxed 1H pose exists");
            if(pose>=0)for(uint32_t i=0;i<file.PoseAt(pose).numFrames;++i)
            {Curves actual;file.FrameCurves(pose,file.PoseAt(pose).firstFrame+i,actual);check(actual[kLegs]==0&&actual[kPelvis]==0,"male real data preserves locomotion lower body");check(actual[kArmL]==1&&actual[kArmR]==1,"male real data retains weapon arms");}
        }
    }
    if(argc>1)
    {
        OverlayFile file;std::string error;
        const bool loaded=file.LoadFromFile(argv[1],kRigIdentityWomanOv,error);
        check(loaded,"patched real overlay passes loader CRC/schema/rig gates");
        if(!loaded)std::printf("Loader: %s\n",error.c_str());
        if(loaded)
        {
            const int pose=file.FindPose("Pose_Pistol1H_Stand_Relax_Idle");check(pose>=0,"real relaxed 1H pose exists");
            if(pose>=0)for(uint32_t i=0;i<file.PoseAt(pose).numFrames;++i)
            {Curves actual;file.FrameCurves(pose,file.PoseAt(pose).firstFrame+i,actual);check(actual[kLegs]==0&&actual[kPelvis]==0,"real data preserves locomotion lower body");check(actual[kArmL]==1&&actual[kArmR]==1,"real data retains weapon arms");}
        }
    }
    std::printf("IdlePistolLowerBody: %d/%d PASS\n",checks-failures,checks);return failures?1:0;
}

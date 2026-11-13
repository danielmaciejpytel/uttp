// Rotation channels (RotX / RotY / RotZ, degrees) of a bounce output in the offline deformer interpreter, against an independent
// axis-angle quaternion construction R = Rz(yaw) * Rx(pitch) * Ry(roll). The convention is the one of the game's deformer graph
// (tools/analysis/deformer_rotation_channels.py); the real elbow helpers of both bodies are checked against recorded engine values in
// BodyHelperBundleTest.
#include "../src/mm/BodyDeformerOffline.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
namespace d=utpp::mm::deformoffline;
namespace
{
constexpr double kPi=3.14159265358979323846;
struct Q{double x,y,z,w;};
Q Mul(Q a,Q b){return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};}
Q Axis(int axis,double deg){const double h=deg*kPi/360;const double s=std::sin(h),c=std::cos(h);return {axis==0?s:0,axis==1?s:0,axis==2?s:0,c};}
// pitch about X, roll about Y, yaw about Z; R = Rz(yaw) Rx(pitch) Ry(roll)
Q Euler(double pitch,double roll,double yaw){return Mul(Mul(Axis(2,yaw),Axis(0,pitch)),Axis(1,roll));}
d::Quat F(Q q){return {float(q.x),float(q.y),float(q.z),float(q.w)};}
// Angle between two rotations from the relative quaternion (accurate for tiny angles, unlike the acos of a dot product).
double Angle(d::Quat a,Q b)
{
    const Q r=Mul(Q{-double(a.x),-double(a.y),-double(a.z),double(a.w)},b);
    return 2*std::atan2(std::sqrt(r.x*r.x+r.y*r.y+r.z*r.z),std::fabs(r.w))*180/kPi;
}
}
int main()
{
    int failures=0,checks=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    auto p=std::make_unique<d::Program>();
    p->nodeCount=3;p->opCount=1;
    p->nodes[0].kind=d::NodeKind::Core;
    p->nodes[1].kind=d::NodeKind::Core;p->nodes[1].parent=0;
    p->nodes[2].parent=0;p->nodes[2].refLocal.t={.1f,.2f,.3f};
    auto& op=p->ops[0];op.kind=d::OpKind::Bounce;op.a=0;op.b=1;op.p[0]=1;op.p[1]=1;
    op.channelCount=1;op.channels[0]={2,0,8,1.f};
    auto input=std::make_unique<d::Inputs>();input->present[0]=input->present[1]=true;
    auto scratch=std::make_unique<d::Scratch>();
    // the driver value v (metres along X between the two core nodes) becomes the angle in degrees: scalar = |v| * 1
    auto run=[&](double pitch,double roll,double yaw,float v,uint32_t kind,float multiplier,bool& ok){
        p->nodes[2].refLocal.q=F(Euler(pitch,roll,yaw));
        op.channels[0].kind=kind;op.channels[0].multiplier=multiplier;
        input->model[1].t.x=v;
        ok=bool(d::Validate(*p))&&bool(d::Evaluate(*p,*input,*scratch));
        return scratch->model[2];
    };
    bool ok=false;
    {   // RotZ: only the yaw changes
        const auto m=run(20,-35,50,30.f,8,1.f,ok);
        check(ok,"RotZ evaluates");
        check(Angle(m.q,Euler(20,-35,30))<1e-3,"RotZ replaces the yaw (pitch and roll kept)");
        check(std::fabs(m.t.x-.1f)<1e-6f&&std::fabs(m.t.y-.2f)<1e-6f&&std::fabs(m.t.z-.3f)<1e-6f,"RotZ keeps the translation");
    }
    {   // RotX: only the pitch changes, with a multiplier and a negative value
        const auto m=run(20,-35,50,10.f,6,-2.f,ok);
        check(ok&&Angle(m.q,Euler(-20,-35,50))<1e-3,"RotX replaces the pitch (value * multiplier = -20 deg)");
    }
    {   // RotY: only the roll changes
        const auto m=run(20,-35,50,45.f,7,1.f,ok);
        check(ok&&Angle(m.q,Euler(20,45,50))<1e-3,"RotY replaces the roll");
    }
    {   // the angle equal to the current one: the rotation is unchanged
        const auto m=run(15,25,35,35.f,8,1.f,ok);
        check(ok&&Angle(m.q,Euler(15,25,35))<1e-3,"replacing the yaw by itself keeps the rotation");
    }
    {   // an output with RotZ and PosX (the real elbow outputs): both apply, one local/compose round trip
        p->nodes[2].refLocal.q=F(Euler(5,-10,60));
        op.channelCount=2;op.channels[0]={2,0,8,1.f};op.channels[1]={2,0,0,.001f};
        input->model[1].t.x=40.f;
        check(bool(d::Validate(*p))&&bool(d::Evaluate(*p,*input,*scratch)),"RotZ + PosX output evaluates");
        check(Angle(scratch->model[2].q,Euler(5,-10,40))<1e-3&&std::fabs(scratch->model[2].t.x-.04f)<1e-6f,"RotZ + PosX: yaw replaced and X translation set (40 * 0.001)");
        op.channelCount=1;
    }
    {   // a world rotation of the parent does not change the local result
        input->model[0].q={0,0,.70710678f,.70710678f};input->model[1].q={0,0,.70710678f,.70710678f};
        input->model[1].t={0,40.f,0}; // 40 along the parent's local X (the parent is turned 90 deg about Z)
        p->nodes[2].refLocal.q=F(Euler(5,-10,60));op.channels[0]={2,0,8,1.f};
        const bool good=bool(d::Validate(*p))&&bool(d::Evaluate(*p,*input,*scratch));
        const Q parent{0,0,.70710678,.70710678};
        check(good&&Angle(scratch->model[2].q,Mul(parent,Euler(5,-10,40)))<1e-3,"rotated parent: the channel acts in the parent's frame");
        input->model[0].q=d::Quat{};input->model[1].q=d::Quat{};input->model[1].t={};
    }
    {   // the gimbal boundary (pitch +-90 deg) has no unique split: fail closed
        run(90,0,20,10.f,8,1.f,ok);
        check(!ok,"pitch of 90 degrees: refused (no unique yaw / roll)");
        run(-90,0,20,10.f,8,1.f,ok);
        check(!ok,"pitch of -90 degrees: refused");
        const auto m=run(80,0,20,10.f,8,1.f,ok);
        check(ok&&Angle(m.q,Euler(80,0,10))<5e-3,"pitch of 80 degrees: still split exactly");
    }
    {   // unknown channel kinds stay refused by the cold validation and by Evaluate
        op.channels[0].kind=9;
        check(!d::Validate(*p),"channel kind 9 is rejected by Validate");
        check(!d::Evaluate(*p,*input,*scratch),"channel kind 9 is rejected by Evaluate");
        op.channels[0].kind=8;
        op.channels[0].multiplier=std::nanf("");
        check(!d::Validate(*p),"nonfinite multiplier is rejected by Validate");
        op.channels[0].multiplier=1.f;
    }
    std::printf("checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}

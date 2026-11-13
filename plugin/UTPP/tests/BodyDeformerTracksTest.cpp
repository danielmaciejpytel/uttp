// Targeted offline track behavior: producer ordering, clamping and freshness.
#include "../src/mm/BodyDeformerOffline.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
namespace d=utpp::mm::deformoffline;
int main()
{
    int failures=0,checks=0;
    auto check=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::printf("FAIL %s\n",label);}};
    auto p=std::make_unique<d::Program>();
    p->nodeCount=5;p->opCount=2;
    p->nodes[0].kind=d::NodeKind::Core;
    p->nodes[1].kind=d::NodeKind::Core;p->nodes[1].parent=0;
    for(int i=2;i<5;++i)p->nodes[i].parent=0;
    p->nodes[3].refLocal.t={1,1,0};p->nodes[4].refLocal.t={2,0,0};
    auto& producer=p->ops[0];producer.kind=d::OpKind::TrackBounce;
    producer.target=0;producer.a=0;producer.b=1;
    producer.p[0]=1;producer.p[1]=2;producer.p[5]=1;
    auto& spline=p->ops[1];spline.kind=d::OpKind::SplineTrack;
    spline.target=2;spline.a=0;spline.b=3;spline.c=4;spline.p[1]=0;
    auto input=std::make_unique<d::Inputs>();
    input->present[0]=input->present[1]=true;
    auto scratch=std::make_unique<d::Scratch>();
    check(bool(d::Validate(*p)),"same-program producer validates");
    auto sample=[&](float driver,float x,float y){
        input->model[1].t.x=driver;
        auto before=std::make_unique<d::Inputs>(*input);
        const auto result=d::Evaluate(*p,*input,*scratch);
        check(bool(result),"track evaluate");
        check(std::fabs(scratch->model[2].t.x-x)<1e-5f&&std::fabs(scratch->model[2].t.y-y)<1e-5f,"dynamic spline position");
        check(std::memcmp(input.get(),before.get(),sizeof(d::Inputs))==0,"actual input bytes unchanged");
        check(std::memcmp(&scratch->model[0],&input->model[0],sizeof(d::Transform))==0,"track register 0 never writes core node 0");
    };
    sample(.25f,.5f,.375f);
    sample(-.25f,1.f,.5f);
    sample(2.f,2.f,0.f);
    producer.p[5]=-1;
    sample(.25f,0.f,0.f);
    producer.p[5]=1;
    producer.target=d::kMaxTracks;
    check(!d::Validate(*p),"oversized scalar register rejected");
    producer.target=0;
    const auto saved=producer;
    p->ops[0]=spline;p->opCount=1;
    check(!d::Validate(*p),"unproduced track rejected in cold validation");
    check(!d::Evaluate(*p,*input,*scratch),"previous Evaluate track cannot leak into next call");
    p->ops[0]=saved;p->opCount=2;
    sample(.1f,.2f,.18f);
    std::printf("checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}

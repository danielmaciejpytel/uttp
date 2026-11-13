#include "../src/mm/LocomotionSignalBlend.hpp"
#include <cstdio>
#include <limits>
using namespace utpp::mm::locosignal;
#define CHECK(x) do { if (!(x)) { std::printf("FAIL line %d: %s\n", __LINE__, #x); return 1; } ++checks; } while (false)
int main(){int checks=0;Blend b;Sample a{{0,7.650717f},0,{.232027f,0}},c{{0,7.029675f},0,{0,.978448f}};
 auto same=[](const Sample&a,const Sample&b){return a.rootVel[0]==b.rootVel[0]&&a.rootVel[1]==b.rootVel[1]&&a.rootYawRate==b.rootYawRate&&a.contact[0]==b.contact[0]&&a.contact[1]==b.contact[1];};
 CHECK(same(b.Update(a,true,.5f,.016f),a));
 CHECK(same(b.Update(c,false,.5f,.016f),c));
 b.Update(a,false,.5f,0);CHECK(same(b.Update(c,true,.5f,.016f),a));
 auto mid=b.Update(c,false,.5f,.25f);CHECK(mid.rootVel[1]<a.rootVel[1]&&mid.rootVel[1]>c.rootVel[1]);CHECK(mid.contact[1]>0&&mid.contact[1]<c.contact[1]);
 CHECK(same(b.Update(a,true,.5f,.016f),mid));CHECK(same(b.Update(a,false,.5f,.5f),a));
 CHECK(same(b.Update(c,true,0,.016f),c));
 b.Update(a,true,.5f,0);b.ClearContacts();CHECK(b.Output().contact[0]==0&&b.Output().contact[1]==0);
 b.Reset();CHECK(same(b.Update(c,false,.5f,.016f),c)); // rig/skip/traversal/FPP reset contract
 Sample bad=c;bad.rootVel[0]=std::numeric_limits<float>::quiet_NaN();CHECK(same(b.Update(bad,false,.5f,.016f),Sample{}));
 CHECK(same(b.Update(a,false,.5f,.016f),a));CHECK(same(b.Update(c,false,.5f,-1),Sample{}));
 std::printf("Locomotion signal continuity: %d PASS\n",checks);
}

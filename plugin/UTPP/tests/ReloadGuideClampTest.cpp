// Offline test: the capped guide turn (right arm in Pistol2H reloads, cap pi/4) is continuous across the +-pi wrap.
// Measured on 19.10 11:49 R19 k2 (replay with tools/analysis/replay_pose_csv_reload.cpp, REPLAY_DIAG): the raw turn of the
// native elbow plane went +114.5 -> -165.2 deg in one frame; the hard clamp turned that into +45 -> -45 deg and the
// right upper arm jumped 164 deg between two frames (baseline arm 10 deg, native arm 2 deg).
// Mutation 9 (UTPP_RELOAD_CONTACT_MUTATION=9) restores the hard clamp: the test must report a FAIL.
#include "../src/mm/NativeReloadTransfer.hpp"
#include <cmath>
#include <cstdio>
using namespace utpp::mm;
static int g_pass=0,g_fail=0;
static void Check(bool ok,const char* what){(ok?g_pass:g_fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);}
int main()
{
    const float cap=math::kPi*0.25f,deg=math::kPi/180.0f;
    // 1. Unchanged inside the working range (healthy recordings: raw right turn below ~135deg).
    bool same=true;
    for(float d=-135.0f;d<=135.0f;d+=0.25f)
        same=same&&native_reload::SoftClampGuideTurn(d*deg,cap)==std::clamp(d*deg,-cap,cap);
    Check(same,"|turn| <= 135deg: identical to the hard clamp");
    // 2. Unbounded cap: identity.
    bool identity=true;
    for(float d=-180.0f;d<=180.0f;d+=0.5f)
        identity=identity&&native_reload::SoftClampGuideTurn(d*deg,native_reload::kUnboundedGuideTurnRad)==d*deg;
    Check(identity,"unbounded cap (left arm, other consumers): untouched");
    // 3. Continuity on the circle: walk the raw turn through the wrap in 0.5deg steps.
    float previous=native_reload::SoftClampGuideTurn(100.0f*deg,cap),worst=0;
    for(float d=100.0f;d<=260.0f;d+=0.5f) // 100 .. 260 deg == -100 deg after the wrap
    {
        float wrapped=d;if(wrapped>180.0f)wrapped-=360.0f;
        const float now=native_reload::SoftClampGuideTurn(wrapped*deg,cap);
        worst=std::fmax(worst,std::fabs(now-previous));
        previous=now;
    }
    std::printf("  largest applied-turn step over a 0.5deg raw step: %.2fdeg\n",worst/deg);
    Check(worst<1.0f*deg,"applied turn changes by less than 1deg per 0.5deg of raw turn through the +-180 wrap");
    // 4. The recorded R19 k2 sequence of raw turns (deg): no 90deg swing of the applied turn.
    const float recorded[]={114.5f,-165.2f,-107.0f,-78.8f,-65.0f,-56.8f,-50.0f};
    float maxStep=0,prev=native_reload::SoftClampGuideTurn(recorded[0]*deg,cap);
    for(int i=1;i<7;++i)
    {
        const float v=native_reload::SoftClampGuideTurn(recorded[i]*deg,cap);
        maxStep=std::fmax(maxStep,std::fabs(v-prev));prev=v;
    }
    std::printf("  recorded sequence: largest applied-turn step %.1fdeg (hard clamp: 90deg)\n",maxStep/deg);
    Check(maxStep<65.0f*deg,"recorded R19 k2 wrap: applied-turn step stays below 65deg (hard clamp 90deg)");
    // 5. +-pi meet at 0.
    Check(std::fabs(native_reload::SoftClampGuideTurn(math::kPi,cap))<1e-6f&&std::fabs(native_reload::SoftClampGuideTurn(-math::kPi,cap))<1e-6f,
          "turn of +-180deg applies no turn (the side is undefined)");
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail?1:0;
}

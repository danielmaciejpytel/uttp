// Focused cancellation support contract on the real retargeted leg chain.
// This does not measure sole penetration or enable standing replant.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "../src/mm/FootLocker.hpp"

using namespace utpp::mm;
static int failed = 0, passed = 0;
static void Check(const char* name, bool value)
{
    std::printf("%s %s\n", value ? "PASS" : "FAIL", name);
    value ? ++passed : ++failed;
}
static void Hold(FootLocker& f, int side)
{
#ifndef UTPP_SUPPORT_BASELINE
    f.SetSupportHold(side);
#else
    (void)f; (void)side;
#endif
}
static math::Vec3 WorldToe(const FootLocker& f, int side)
{
    auto p = f.ModelSpace(f.Output(), f.LockBone(side)).p;
    float x = 0, y = 0;
    f.CharacterToWorld(p, x, y);
    return {x, y, p.z};
}
static float Planar(math::Vec3 a, math::Vec3 b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}
int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    Database db; std::string error;
    if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, error))
    { std::printf("database: %s\n", error.c_str()); return 2; }
    int clip = -1;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        if (db.ClipName(c) == "M_Neutral_Walk_Loop_F") clip = static_cast<int>(c);
    if (clip < 0) return 2;
    std::vector<BoneTransform> pose(db.NumBones());
    const auto raw = db.Pose(db.Clips()[clip].firstFrame + 6);
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        const float* r = raw.data() + b * 7;
        pose[b] = {{r[0],r[1],r[2]}, {r[3],r[4],r[5],r[6]}};
    }
    for (int hz : {30, 60, 144})
    {
        const float dt = 1.0f / hz;
        auto setup = [&](FootLocker& f, float displacement)
        {
            Check("real leg chain init", f.Init(db));
            f.SetSlip(0.02f, 0.04f);
            for (int n = 0; n < hz / 3; ++n) f.Update(dt,0,0,0,1,1,pose);
            f.Update(dt,0,-displacement/dt,0,1,1,pose);
        };
        FootLocker f; setup(f, 0.05f);
        const auto left = WorldToe(f,0), right = WorldToe(f,1);
        Hold(f,0);
        for (int n = 0; n < hz / 5; ++n) f.Update(dt,0,0,0,1,1,pose);
        const float leftDrift = Planar(left,WorldToe(f,0));
        std::printf("hz=%d support drift=%.3fmm other=%.3fmm\n",hz,leftDrift*1000,
                    Planar(right,WorldToe(f,1))*1000);
        Check("cancel support remains planted below1mm",leftDrift < .001f);
        Check("other foot retains normal settling",Planar(right,WorldToe(f,1)) > .005f);
        Check("support does not force replant or reach clamp",!f.Diagnostics().foot[0].replant && !f.Diagnostics().foot[0].clamped);
        Hold(f,-1);
        const auto released = WorldToe(f,0);
        for (int n = 0; n < hz / 5; ++n) f.Update(dt,0,0,0,1,1,pose);
        Check("window end resumes normal settling",Planar(released,WorldToe(f,0)) > .005f);

        setup(f,0.05f); Hold(f,0);
        for (int n=0; n<hz/4; ++n) f.Update(dt,0,0,0,0,1,pose);
        Check("contact loss releases support and rotation tail",!f.Diagnostics().foot[0].locked && f.Diagnostics().foot[0].weight==0);
        f.Update(dt,0,0,0,1,1,pose);
        for (int n=0; n<hz/5; ++n) f.Update(dt,0,0,0,1,1,pose);
        f.Update(dt,0,-.05f/dt,0,1,1,pose);
        const auto recontact = WorldToe(f,0);
        for (int n=0; n<hz/5; ++n) f.Update(dt,0,0,0,1,1,pose);
        Check("new contact is not captured by old support episode",Planar(recontact,WorldToe(f,0)) > .005f);

        setup(f,0.05f); Hold(f,0);
        f.Update(dt,0,-.5f/dt,0,1,1,pose);
        Check("teleport drift guard is preserved",!f.Diagnostics().foot[0].locked && f.Diagnostics().driftUnlocks > 0);
        f.Reset(); Hold(f,0);
        for (int n=0; n<hz/5; ++n) f.Update(dt,0,0,0,0,0,pose);
        Check("hold never manufactures missing contact",!f.Diagnostics().foot[0].locked);

        FootLocker guarded, normal;
        setup(guarded,.18f); setup(normal,.18f); Hold(guarded,0);
        guarded.Update(dt,0,0,0,1,1,pose); normal.Update(dt,0,0,0,1,1,pose);
        Check("large incompatible offset keeps normal safety policy",
              Planar(WorldToe(guarded,0),WorldToe(normal,0)) < .00001f);
        guarded.Reset(); normal.Reset();
        for (int n=0;n<hz/3;++n)
        { guarded.Update(dt,0,0,0,1,1,pose); normal.Update(dt,0,0,0,1,1,pose); }
        guarded.Update(dt,0,-.05f/dt,0,1,1,pose); normal.Update(dt,0,-.05f/dt,0,1,1,pose);
        guarded.Update(dt,0,0,0,1,1,pose); normal.Update(dt,0,0,0,1,1,pose);
        Check("reset drops episode support state",Planar(WorldToe(guarded,0),WorldToe(normal,0)) < .00001f);
    }
    std::printf("SUMMARY %d passed %d failed\n",passed,failed);
    return failed ? 1 : 0;
}

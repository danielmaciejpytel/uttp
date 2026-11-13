#include "../src/mm/OffsetRoot.hpp"
#include <cstdio>
#include <cstring>
#include <limits>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 2e-6f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "Data/utpp_base.utppmm";
    Database db;
    std::string error;
    C("actual base DB loads", db.LoadFromFile(path, 0x72CB314EB01F562Dull, error));
    if (fail)
    {
        std::puts(error.c_str());
        return 2;
    }
    OffsetRoot root, legacy;
    C("provided and legacy instances initialize", root.Init(db, 2) && legacy.Init(db, 2));
    if (fail)
        return 2;
    const uint32_t n = db.NumBones();
    std::vector<BoneTransform> pose(n);
    const auto raw = db.Pose(200);
    for (uint32_t b = 0; b < n; ++b)
    {
        const float *v = raw.data() + 7 * b;
        pose[b] = {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }
    const float cap[2] = {1, .5f}, anim[2] = {.4f, .2f};
    root.Update(.02f, cap, .05f, anim, .2f, true);
    C("provided adopt allows yaw beyond legacy max",
      root.AdoptFrame(.12f, -.09f, 1.8f) && Near(root.Yaw(), 1.8f) && root.Yaw() > root.Config().maxYaw);
    root.Apply(pose);
    bool allOtherExact = true, rootRigid = true;
    uint32_t rootChildren = 0;
    const double c = std::cos(1.8), s = std::sin(1.8), qz = std::sin(.9), qw = std::cos(.9);
    for (uint32_t b = 0; b < n; ++b)
    {
        const auto &a = pose[b];
        const auto &x = root.Output()[b];
        if (b >= 2 && db.BoneParent(b) < 2)
        {
            ++rootChildren;
            rootRigid &= Near(x.p.x, float(.12 + c * a.p.x - s * a.p.y)) &&
                         Near(x.p.y, float(-.09 + s * a.p.x + c * a.p.y)) && x.p.z == a.p.z;
            const math::Quat expected{float(qw * a.q.x - qz * a.q.y), float(qw * a.q.y + qz * a.q.x),
                                      float(qw * a.q.z + qz * a.q.w), float(qw * a.q.w - qz * a.q.z)};
            rootRigid &= Near(x.q.x, expected.x) && Near(x.q.y, expected.y) && Near(x.q.z, expected.z) &&
                         Near(x.q.w, expected.w);
        }
        else
            allOtherExact &= std::memcmp(&a, &x, sizeof a) == 0;
    }
    C("actual root children receive independent rigid transform", rootChildren > 0 && rootRigid);
    C("all non-root-child local transforms byte unchanged", allOtherExact);
    std::vector<BoneTransform> once(root.Output().begin(), root.Output().end());
    const auto updates = root.Diagnostics().updates;
    for (int i = 0; i < 5; ++i)
        root.Apply(pose);
    C("repeated Apply does not integrate or double transform",
      std::memcmp(once.data(), root.Output().data(), n * sizeof(BoneTransform)) == 0 &&
          root.Diagnostics().updates == updates && Near(root.X(), .12f) && Near(root.Yaw(), 1.8f));
    const float x = root.X(), y = root.Y(), yaw = root.Yaw();
    C("NaN adopt rejected without state change", !root.AdoptFrame(std::numeric_limits<float>::quiet_NaN(), 0, 0) &&
                                                     root.X() == x && root.Y() == y && root.Yaw() == yaw);
    C("out-of-radius adopt rejected without state change",
      !root.AdoptFrame(1.001f, 0, 0) && root.X() == x && root.Y() == y && root.Yaw() == yaw);
    C("invalid yaw adopt rejected without state change",
      !root.AdoptFrame(0, 0, std::numeric_limits<float>::infinity()) && root.Yaw() == yaw);
    root.Update(0, cap, .3f, anim, .4f, true);
    legacy.Update(0, cap, .3f, anim, .4f, true);
    C("first legacy zero-dt update relinquishes provided owner and resets",
      root.X() == 0 && root.Y() == 0 && root.Yaw() == 0);
    root.Apply(pose);
    C("legacy zero-dt resumed pose equals original",
      std::memcmp(pose.data(), root.Output().data(), n * sizeof(BoneTransform)) == 0);
    for (int i = 0; i < 35; ++i)
    {
        root.Update(.016f, cap, .12f, anim, .7f, true);
        legacy.Update(.016f, cap, .12f, anim, .7f, true);
        root.Apply(pose);
        legacy.Apply(pose);
        C("legacy resumes identically to fresh legacy instance",
          root.X() == legacy.X() && root.Y() == legacy.Y() && root.Yaw() == legacy.Yaw() &&
              std::memcmp(root.Output().data(), legacy.Output().data(), n * sizeof(BoneTransform)) == 0);
        C("legacy yaw limit restored after provided owner", std::fabs(root.Yaw()) <= root.Config().maxYaw + 1e-6f);
    }
    root.AdoptFrame(.12f, -.09f, -2.1f);
    root.Update(.016f, cap, .12f, anim, .7f, true);
    OffsetRoot immediate;
    immediate.Init(db, 2);
    immediate.Update(.016f, cap, .12f, anim, .7f, true);
    C("positive-dt immediate legacy handback also resets owner",
      root.X() == immediate.X() && root.Y() == immediate.Y() && root.Yaw() == immediate.Yaw());
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

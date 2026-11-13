// Focused actual-data livebase evaluator contract; independent two-player cubic oracle.
#include "../src/mm/MotionPose.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool x)
{
    std::printf("%s %s\n", x ? "PASS" : "FAIL", n);
    x ? ++pass : ++fail;
}
static bool Same(const std::vector<BoneTransform> &a, const std::vector<BoneTransform> &b)
{
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(BoneTransform)) == 0;
}
static bool Close(const std::vector<BoneTransform> &a, const std::vector<BoneTransform> &b, float e = 2e-5f)
{
    for (size_t i = 2; i < a.size(); ++i)
    {
        const auto &x = a[i];
        const auto &y = b[i];
        if (std::fabs(x.p.x - y.p.x) > e || std::fabs(x.p.y - y.p.y) > e || std::fabs(x.p.z - y.p.z) > e ||
            std::fabs(std::fabs(math::QuatDot(x.q, y.q)) - 1) > e)
            return false;
    }
    return true;
}
static motion::MatchResult M(int c, float t, bool jump = false, float rate = 1)
{
    motion::MatchResult r;
    r.clip = c;
    r.clipTime = t;
    r.playRate = rate;
    r.jumped = jump;
    r.blend.valid = jump;
    r.blend.blendTime = .5f;
    return r;
}
static int Find(const motion::MotionDatabase &db, const char *n)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == n)
            return int(i);
    return -1;
}
static BoneTransform MixIndependent(const BoneTransform &a, const BoneTransform &b, double w)
{
    BoneTransform o;
    for (int k = 0; k < 3; ++k)
        (&o.p.x)[k] = float((1 - w) * (&a.p.x)[k] + w * (&b.p.x)[k]);
    double d = 0;
    for (int k = 0; k < 4; ++k)
        d += double((&a.q.x)[k]) * (&b.q.x)[k];
    double q[4], n = 0;
    for (int k = 0; k < 4; ++k)
    {
        q[k] = (1 - w) * (&a.q.x)[k] + w * (d < 0 ? -1 : 1) * (&b.q.x)[k];
        n += q[k] * q[k];
    }
    for (int k = 0; k < 4; ++k)
        (&o.q.x)[k] = float(q[k] / std::sqrt(n));
    return o;
}
static int Bone(const motion::MotionDatabase &db, const char *n)
{
    for (uint32_t i = 0; i < db.NumBones(); ++i)
        if (db.BoneName(i) == n)
            return int(i);
    return -1;
}
// Model-space transform of one bone of a local pose (FK along its parent chain from the skeleton root).
static BoneTransform Model(const motion::MotionDatabase &db, const std::vector<BoneTransform> &pose, int bone)
{
    std::vector<int> chain;
    for (int b = bone; b >= 0; b = db.BoneParent(uint32_t(b)))
        chain.insert(chain.begin(), b);
    BoneTransform m{{0, 0, 0}, {0, 0, 0, 1}};
    for (const int b : chain)
    {
        m.p = m.p + math::QuatRotate(m.q, pose[size_t(b)].p);
        m.q = math::QuatNormalize(math::QuatMul(m.q, pose[size_t(b)].q));
    }
    return m;
}
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::puts("usage PoseLiveCrossfadeTest <db> <ov>");
        return 2;
    }
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    std::string err;
    C("actual DB", db->LoadFromFile(argv[1], motion::kRigIdentityWoman, err));
    C("actual OV", ovf->LoadFromFile(argv[2], ov::kRigIdentityWomanOv, err));
    if (fail)
    {
        std::puts(err.c_str());
        return 2;
    }
    const int walk = Find(*db, "M_Neutral_Walk_Loop_F"), run = Find(*db, "M_Neutral_Run_Loop_F");
    C("actual two live clips", walk >= 0 && run >= 0);
    if (fail)
        return 2;
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    C("explicit live policy init",
      ev->Init(db.get(), ovf.get(), nullptr, 2, err, motionpose::BaseBlendPolicy::LiveCrossfade));
    if (fail)
        return 2;
    const uint32_t n = db->NumBones();
    std::vector<BoneTransform> out(n), before(n), raw(n), pa(n), pb(n), expected(n);
    motionpose::OverlayFrameInputs in;
    in.overlayWeight = 0;
    motionpose::FrameInfo info;
    auto eval = [&](float dt, motion::MatchResult m) { return ev->Evaluate(dt, m, in, out.data(), n, &info); };
    C("first actual pose", eval(0, M(walk, .2f, false, .8f)));
    adapter::SamplePose(*db, walk, .2f, raw.data(), n);
    C("single player bitexact", Same(out, raw));
    C("sameclip authoritative sample", eval(.02f, M(walk, .216f, false, .8f)));
    adapter::SamplePose(*db, walk, .216f, raw.data(), n);
    C("sameclip no double advance bitexact", Same(out, raw));
    before = out;
    C("jump dt0 accepted", eval(0, M(run, .4f, true, 1.2f)));
    C("jump dt0 C0", Close(out, before));
    C("live signal ownership", ev->UsesLiveBase() && info.liveCrossfade);
    C("live moving pair", eval(.1f, M(run, .52f, false, 1.2f)));
    adapter::SamplePose(*db, walk, .296f, pa.data(), n);
    adapter::SamplePose(*db, run, .52f, pb.data(), n);
    // Cubic Hermite blend clock .1/.5; outgoing animation .216+.1*.8.
    for (uint32_t i = 0; i < n; ++i)
    {
        const auto name = db->BoneName(i);
        const bool leg = name == "LeftUpLeg" || name == "LeftLeg" || name == "LeftFoot" || name == "LeftToeBase" ||
                         name == "RightUpLeg" || name == "RightLeg" || name == "RightFoot" || name == "RightToeBase";
        const double factor = leg ? 2. / 3. : 1.;
        const double u = std::min(1., .1 / (.5 * factor)), w = u * u * (3 - 2 * u);
        expected[i] = MixIndependent(pa[i], pb[i], w);
    }
    // Ankle floor (MotionBlendStack::KeepAnklesAboveLayers, loco-fix 2026-11-09): the blend may not take an ankle more than 1 cm under the
    // LOWEST ankle of its live layers; the part below is lifted through the leg by rotations of UpLeg / Leg / Foot only (the two-bone
    // lift: every translation and the foot's model orientation kept). In this walk -> run frame the plain cubic blend puts the right ankle
    // 1.8 cm under both clips, so the oracle carries the same contract, computed here from the two clip samples alone:
    //  - every bone except the UpLeg / Leg / Foot rotations of a floored leg == the cubic oracle (2e-5, as before the floor);
    //  - a floored leg: ankle (Foot origin) == the cubic oracle ankle raised to 1 cm under the lowest clip ankle, foot model orientation ==
    //    the cubic oracle's (2e-5). Without the floor (UTPP_FOOTSINK_MUTATION 11) the floored-leg checks fail.
#ifndef UTPP_XFADE_FLOOR_TOL
#define UTPP_XFADE_FLOOR_TOL 0.01 // m, MotionBlendStack kAnkleFloorTolerance (a different value here only to show the checks bite)
#endif
    constexpr double kAnkleFloorTol = UTPP_XFADE_FLOOR_TOL;
    const char *legs[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"}, {"RightUpLeg", "RightLeg", "RightFoot"}};
    std::vector<char> liftedRotation(n, 0);
    bool floored[2] = {false, false}, legsFound = true;
    double lift[2] = {0.0, 0.0};
    for (int side = 0; side < 2; ++side)
    {
        int b[3];
        for (int k = 0; k < 3; ++k)
            legsFound &= (b[k] = Bone(*db, legs[side][k])) >= 2;
        if (!legsFound)
            break;
        const double lowest = std::min(Model(*db, pa, b[2]).p.z, Model(*db, pb, b[2]).p.z);
        lift[side] = lowest - kAnkleFloorTol - Model(*db, expected, b[2]).p.z;
        floored[side] = lift[side] > 0.0;
        if (floored[side])
            for (int k = 0; k < 3; ++k)
                liftedRotation[size_t(b[k])] = 1;
    }
    C("actual leg bones", legsFound);
    if (!legsFound)
        return 2;
    std::printf("plain blend ankle vs 1 cm under the lowest clip ankle: left %+.4f m, right %+.4f m (> 0 = floored)\n", lift[0], lift[1]);
    C("oracle frame exercises the ankle floor (a leg of the plain blend > 1 cm under both clips)", floored[0] || floored[1]);
    bool cubic = true;
    for (uint32_t i = 2; i < n; ++i)
    {
        const auto &x = out[i];
        const auto &y = expected[i];
        const float e = 2e-5f;
        cubic &= std::fabs(x.p.x - y.p.x) <= e && std::fabs(x.p.y - y.p.y) <= e && std::fabs(x.p.z - y.p.z) <= e;
        if (!liftedRotation[i])
            cubic &= std::fabs(std::fabs(math::QuatDot(x.q, y.q)) - 1) <= e;
    }
    C("manual two-clip cubic pose oracle", cubic);
    for (int side = 0; side < 2; ++side)
    {
        if (!floored[side])
            continue;
        const int foot = Bone(*db, legs[side][2]);
        const BoneTransform got = Model(*db, out, foot), want = Model(*db, expected, foot);
        const float e = 2e-5f;
        C(side ? "ankle floor right: ankle = cubic oracle ankle raised to 1 cm under the lowest clip ankle"
               : "ankle floor left: ankle = cubic oracle ankle raised to 1 cm under the lowest clip ankle",
          std::fabs(got.p.x - want.p.x) <= e && std::fabs(got.p.y - want.p.y) <= e &&
              std::fabs(double(got.p.z) - (double(want.p.z) + lift[side])) <= e);
        C(side ? "ankle floor right: foot model orientation = cubic oracle" : "ankle floor left: foot model orientation = cubic oracle",
          std::fabs(std::fabs(math::QuatDot(got.q, want.q)) - 1) <= e);
    }
    bool finite = true, lengths = true;
    for (uint32_t i = 2; i < n; ++i)
    {
        const auto &p = out[i];
        finite &= math::IsFinite(p.p) && math::IsFinite(p.q) && std::fabs(math::QuatDot(p.q, p.q) - 1.f) < 1e-4f;
        double la = std::sqrt(math::Dot(pa[i].p, pa[i].p)), lb = std::sqrt(math::Dot(pb[i].p, pb[i].p)),
               lo = std::sqrt(math::Dot(p.p, p.p));
        lengths &= lo <= std::max(la, lb) + 1e-5;
    }
    C("finite normalized transforms", finite);
    C("local segment lengths bounded by actual source pair", lengths);
    C("endpoint advance", eval(.4f, M(run, 1.f, false, 1.2f)));
    adapter::SamplePose(*db, run, 1.f, raw.data(), n);
    C("incoming endpoint exact", Same(out, raw));
    // Traversal/defaultslot raw branch must inherit the displayed base, then hand back C0.
    before = out;
    in.montageBase = true;
    auto montage = M(run, 1.f, true);
    adapter::SamplePose(*db, walk, .8f, raw.data(), n);
    C("external raw entry", ev->EvaluateWithPose(0, montage, raw.data(), in, out.data(), n, &info));
    C("external entry dt0 no snap", Close(out, before));
    C("external route releases live signals", !ev->UsesLiveBase());
    C("external active finite", ev->EvaluateWithPose(.1f, M(run, 1.f), raw.data(), in, out.data(), n, &info));
    before = out;
    in.montageBase = false;
    C("external exit", eval(0, M(run, 1.2f, true)));
    C("external exit dt0 no snap", Close(out, before));
    C("live resumes", ev->UsesLiveBase());
    ev->Reset();
    C("reset fresh pose", eval(0, M(walk, .5f, true)));
    adapter::SamplePose(*db, walk, .5f, raw.data(), n);
    C("reset clears stale blend", Same(out, raw));
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

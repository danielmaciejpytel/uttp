// Blend profile table (mm/BlendProfile.hpp) and the dead-blended hand-back tail of the blend stack (BlendStack::SetHandBackBlend).
// 1. The locomotion profile resolved to the rig equals the factor vector the pose pipeline passes to the stack, bit for bit; the
//    hand-back profile is root 0, legs 0.75, rest 1 and never exceeds 1 (the stack's relevance pruning needs factors <= 1).
// 2. Extrapolator: starts at the source pose with the source velocity, the velocity decays, the displacement stays bounded.
// 3. With the hand-back blend off, SeedPose(pose, previous, dt) is SeedPose(pose) bit for bit.
// 4. With it on, a hand-back from a moving external pose (a real clip sampled as the external source) keeps the bone velocities
//    continuous at the switch: the velocity change across the hand-back frame is lower than with the frozen tail, at 30 / 60 / 144 Hz.
#include "../src/mm/MotionBlendStack.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    (v ? pass : fail)++;
}
static motion::MatchResult M(int clip, float t, bool jump, float blend = .3f)
{
    motion::MatchResult m;
    m.clip = clip;
    m.clipTime = t;
    m.playRate = 1;
    m.jumped = jump;
    m.blend.valid = jump;
    m.blend.blendTime = blend;
    return m;
}
static int ClipIndex(const motion::MotionDatabase &db, const char *name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name)
            return int(i);
    return -1;
}
// The factor vector the pose pipeline builds for the live cross-fade (kept literally here as the identity reference).
static std::vector<float> PipelineFactors(const motion::MotionDatabase &db)
{
    std::vector<float> factors(db.NumBones(), 1.0f);
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        const auto name = db.BoneName(b);
        if (name == "Root" || name == "Trajectory")
            factors[b] = 0.1f;
        for (const auto leg : {"LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"})
            if (name == leg)
                factors[b] = 1.0f / 1.5f;
    }
    return factors;
}
static deadblend::V3 RotVel(const math::Quat &a, const math::Quat &b, float dt) // b relative to a, per second
{
    const deadblend::Q4 qa{a.x, a.y, a.z, a.w}, qb{b.x, b.y, b.z, b.w};
    return deadblend::ToRotVec(deadblend::Mul(qb, deadblend::Inv(qa))) * (1.f / dt);
}
struct Jump
{
    double rot = 0, pos = 0, rootPos = 0;
};
// Velocity change across the hand-back frame, summed over the bones: (v[k+1] - v[k-1]) where frame k is the hand-back.
static Jump HandBack(const motion::MotionDatabase &db, int extClip, int mmClip, float hz, bool dead, bool seedWithVelocity)
{
    const uint32_t n = db.NumBones();
    const float dt = 1.f / hz;
    motionstack::BlendStack s;
    if (!s.Init(db, PipelineFactors(db)))
        return {1e9, 1e9};
    s.SetHandBackBlend(dead);
    std::vector<std::vector<BoneTransform>> out;
    std::vector<BoneTransform> a(n), b(n);
    const float t0 = .3f;
    // external frames: the clip sampled at its own time (what a montage emits)
    adapter::SamplePose(db, extClip, t0 - dt, b.data(), n);
    adapter::SamplePose(db, extClip, t0, a.data(), n);
    out.push_back(b);
    out.push_back(a);
    if (seedWithVelocity ? !s.SeedPose(a, b, dt) : !s.SeedPose(a))
        return {1e9, 1e9};
    float t = .2f;
    if (!s.Step(M(mmClip, t, true), dt))
        return {1e9, 1e9};
    out.emplace_back(s.Output(), s.Output() + n);
    for (int k = 0; k < 3; ++k)
    {
        t += dt;
        if (!s.Step(M(mmClip, t, false), dt))
            return {1e9, 1e9};
        out.emplace_back(s.Output(), s.Output() + n);
    }
    Jump j;
    for (uint32_t bone = 0; bone < n; ++bone)
    {
        // before: the external velocity (frames 0->1); after: the first hand-back frame (1->2)
        const auto w0 = RotVel(out[0][bone].q, out[1][bone].q, dt), w1 = RotVel(out[1][bone].q, out[2][bone].q, dt);
        j.rot += deadblend::Len(w1 - w0);
        const math::Vec3 v0 = (out[1][bone].p - out[0][bone].p) * hz, v1 = (out[2][bone].p - out[1][bone].p) * hz;
        const double dp = std::sqrt(math::Dot(v1 - v0, v1 - v0));
        if (std::getenv("UTPP_TEST_VERBOSE") && dp > .5)
            std::printf("INFO   bone %s dpos %.3f\n", std::string(db.BoneName(bone)).c_str(), dp);
        // Root / Trajectory carry the root track, which both profiles switch within a frame or two (factor 0.1 / 0): reported apart
        if (db.BoneName(bone) == "Root" || db.BoneName(bone) == "Trajectory")
            j.rootPos += dp;
        else
            j.pos += dp;
    }
    return j;
}
int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    motion::MotionDatabase db;
    std::string error;
    if (!db.LoadFromFile(argv[1], motion::kRigIdentityWoman, error))
    {
        std::puts(error.c_str());
        return 2;
    }
    // 1. profile table
    const auto names = [&](uint32_t b) { return db.BoneName(b); };
    const auto loco = blendprofile::Resolve(blendprofile::kLocomotion, db.NumBones(), names);
    const auto pipe = PipelineFactors(db);
    bool same = loco.size() == pipe.size();
    for (size_t i = 0; same && i < loco.size(); ++i)
        same = loco[i] == pipe[i];
    C("locomotion profile resolves to the pipeline factors bit for bit", same);
    const auto hb = blendprofile::Resolve(blendprofile::kHandBack, db.NumBones(), names);
    bool hbOk = true, hbMax = true;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        const float want = (db.BoneName(b) == "Root" || db.BoneName(b) == "Trajectory") ? 0.f : pipe[b] < 1.f ? .75f : 1.f;
        hbOk &= hb[b] == want;
        hbMax &= hb[b] >= 0 && hb[b] <= 1;
    }
    C("hand-back profile: root 0, legs 0.75, rest 1", hbOk);
    C("hand-back factors within [0, 1]", hbMax);
    C("weight factor 10 -> 0.1, 1.5 -> 1/1.5; time factor kept", blendprofile::DurationFactor(blendprofile::Mode::WeightFactor, 10) == .1f &&
                                                                       blendprofile::DurationFactor(blendprofile::Mode::WeightFactor, 1.5f) == 1.f / 1.5f &&
                                                                       blendprofile::DurationFactor(blendprofile::Mode::TimeFactor, .75f) == .75f);
    // 2. extrapolator
    {
        std::vector<deadblend::BoneTR> cur(1), prev(1), dst(1);
        cur[0].t = {0.1f, 0, 0};
        prev[0].t = {0.09f, 0, 0}; // 1 m/s at 100 Hz
        dst[0].t = {0.5f, 0, 0};
        deadblend::Extrapolator e;
        e.Start(cur, prev, .01f, deadblend::Settings{.maxTranslationVelocity = 5.f});
        e.SetDestination(dst);
        const float x0 = e.Sample(0, 0).t.x, x1 = e.Sample(0, .001f).t.x, xa = e.Sample(0, .1f).t.x, xb = e.Sample(0, .101f).t.x;
        const float xFar = e.Sample(0, 10.f).t.x;
        std::printf("INFO extrapolator x0=%g v0=%g v(0.1)=%g x(10)=%g\n", x0, (x1 - x0) / .001f, (xb - xa) / .001f, xFar);
        C("extrapolator starts at the source", std::fabs(x0 - .1f) < 1e-6f);
        C("extrapolator starts with the source velocity", std::fabs((x1 - x0) / .001f - 1.f) < .02f);
        C("extrapolator velocity decays", (xb - xa) / .001f < (x1 - x0) / .001f);
        C("extrapolator displacement bounded", xFar < .1f + 1.f * 1.0f / deadblend::kLn2 * 1.01f && std::isfinite(xFar));
        e.Start(cur, prev, .01f, deadblend::Settings{.maxTranslationVelocity = .5f});
        C("extrapolator clamps the velocity", std::fabs((e.Sample(0, .001f).t.x - .1f) / .001f - .5f) < .02f);
    }
    const int ext = ClipIndex(db, "M_Neutral_Run_Box_F_LL_Rfoot");
    int mm = -1;
    for (const char *name : {"M_Neutral_Walk_Box_F_LL_Rfoot", "M_Neutral_Run_Box_B_LL_Rfoot", "M_Neutral_Run_Box_F_RR_Lfoot"})
        if ((mm = ClipIndex(db, name)) >= 0)
            break;
    C("fixture clips found", ext >= 0 && mm >= 0 && mm != ext);
    if (ext < 0 || mm < 0)
        return 2;
    // 3. identity with the blend off
    {
        const uint32_t n = db.NumBones();
        std::vector<BoneTransform> a(n), b(n);
        adapter::SamplePose(db, ext, .3f, a.data(), n);
        adapter::SamplePose(db, ext, .29f, b.data(), n);
        motionstack::BlendStack s0, s1;
        s0.Init(db, PipelineFactors(db));
        s1.Init(db, PipelineFactors(db));
        s0.SetHandBackBlend(false);
        s1.SetHandBackBlend(false);
        bool ok = s0.SeedPose(a) && s1.SeedPose(a, b, .01f);
        bool bit = true;
        float t = .2f;
        for (int k = 0; ok && k < 40; ++k, t += 1.f / 60)
        {
            ok = s0.Step(M(mm, t, k == 0), 1.f / 60) && s1.Step(M(mm, t, k == 0), 1.f / 60);
            for (uint32_t i = 0; ok && i < n; ++i)
                bit &= std::memcmp(&s0.Output()[i], &s1.Output()[i], sizeof(BoneTransform)) == 0;
        }
        C("blend off: seed with velocity == static seed, bit for bit (40 frames)", ok && bit);
    }
    // 4. velocity continuity at the hand-back
    for (float hz : {30.f, 60.f, 144.f})
    {
        const Jump off = HandBack(db, ext, mm, hz, false, true), on = HandBack(db, ext, mm, hz, true, true);
        std::printf("INFO %g Hz velocity change at the hand-back, summed over bones: rot %.2f -> %.2f rad/s, pos %.3f -> %.3f m/s "
                    "(root track bones %.3f -> %.3f m/s)\n",
                    hz, off.rot, on.rot, off.pos, on.pos, off.rootPos, on.rootPos);
        char label[128];
        std::snprintf(label, sizeof label, "%g Hz: dead-blended hand-back lowers the rotation velocity change", hz);
        C(label, on.rot < off.rot);
        std::snprintf(label, sizeof label, "%g Hz: dead-blended hand-back lowers the body translation velocity change", hz);
        C(label, std::isfinite(on.pos) && on.pos < off.pos);
    }
    std::printf("%d passed, %d failed\n", pass, fail);
    return fail ? 1 : 0;
}

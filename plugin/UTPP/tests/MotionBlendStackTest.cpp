#include "../src/mm/MotionBlendStack.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 1e-5f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
static motion::MatchResult M(int clip, float t, bool jump, float blend = .5f, float rate = 1)
{
    motion::MatchResult m;
    m.clip = clip;
    m.clipTime = t;
    m.playRate = rate;
    m.jumped = jump;
    m.blend.valid = jump;
    m.blend.blendTime = blend;
    return m;
}
static float AnkleDifference(const motion::MotionDatabase &db, const BoneTransform *a, const BoneTransform *b)
{
    std::vector<BoneTransform> ma(db.NumBones()), mb(db.NumBones());
    float difference = 0;
    for (uint32_t i = 0; i < db.NumBones(); ++i)
    {
        const int parent = db.BoneParent(i);
        auto fk = [&](const BoneTransform *local, std::vector<BoneTransform> &model) {
            model[i] = parent < 0 ? local[i]
                                 : BoneTransform{model[size_t(parent)].p + math::QuatRotate(model[size_t(parent)].q, local[i].p),
                                                 math::QuatNormalize(math::QuatMul(model[size_t(parent)].q, local[i].q))};
        };
        fk(a, ma);
        fk(b, mb);
        if (db.BoneName(i) == "LeftFoot" || db.BoneName(i) == "RightFoot")
        {
            const auto delta = ma[i].p - mb[i].p;
            difference = std::fmax(difference, std::sqrt(math::Dot(delta, delta)));
        }
    }
    return difference;
}

// Clips are found by name: the clip indices differ between bakes.
static int ClipIndex(const motion::MotionDatabase &db, const char *name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name)
            return int(i);
    return -1;
}

// A small warp on a dissimilar outgoing pose must not activate an absolute
// ankle-trajectory blend. Use only the public stack API so this reproduces the
// old bug unchanged, including live layers and collapse to a stored tail.
static void TinyOutgoingWarp(const motion::MotionDatabase &db)
{
    motionstack::BlendStack neutral, tiny;
    neutral.Init(db);
    tiny.Init(db);
    motionstack::GraphInputs graph;
    graph.enabled = true;
    neutral.SetGraphInputs(graph);
    const int outgoing = ClipIndex(db, "M_Neutral_Run_Loop_F_R_20");
    const int incoming = ClipIndex(db, "M_Neutral_Run_Diamond_FL_B_Rfoot");
    const int incomingLfoot = ClipIndex(db, "M_Neutral_Run_Diamond_FL_B_Lfoot");
    C("fixture clips found by name (the incoming pair is adjacent)", outgoing >= 0 && incoming >= 0 && incomingLfoot + 1 == incoming);
    if (outgoing < 0 || incoming < 0 || incomingLfoot + 1 != incoming)
        return;
    constexpr float time = 1.8f, angle = .0024f;
    const auto &clip = db.Clip(outgoing);
    const uint32_t frame = clip.firstFrame + uint32_t(time * db.Header().fps);
    const auto &root = db.Root(frame);
    auto straight = graph;
    straight.velocityWorldX = root.dx;
    straight.velocityWorldY = root.dy;
    neutral.SetGraphInputs(straight);
    auto small = graph;
    small.velocityWorldX = std::cos(2 * angle) * root.dx - std::sin(2 * angle) * root.dy;
    small.velocityWorldY = std::sin(2 * angle) * root.dx + std::cos(2 * angle) * root.dy;
    tiny.SetGraphInputs(small);
    C("tiny outgoing initialized against actual bent run loop", neutral.Step(M(outgoing, time, false), .01f) &&
                                                               tiny.Step(M(outgoing, time, false), .01f));
    C("outgoing fixture really has tiny nonzero orientation warp", std::fabs(tiny.At(0).lastOrientationHalfAngle) > .0001f &&
                                                               std::fabs(tiny.At(0).lastOrientationHalfAngle) < .003f);
    const auto &incomingRoot = db.Root(db.Clip(incoming).firstFrame + uint32_t(.5f * db.Header().fps));
    graph.velocityWorldX = incomingRoot.dx;
    graph.velocityWorldY = incomingRoot.dy;
    tiny.SetGraphInputs(graph);
    neutral.SetGraphInputs(graph);
    // Freeze the sampled phase and OW history while reaching an exact half
    // blend. This isolates the zero-to-tiny relevance boundary from subsequent
    // direction changes, rather than comparing two already-active IK solves.
    neutral.Step(M(incoming, .5f, true, .00000002f), .00000001f);
    tiny.Step(M(incoming, .5f, true, .00000002f), .00000001f);
    float difference = AnkleDifference(db, neutral.Output(), tiny.Output());
    std::printf("INFO half-blend OW neutral=%g,%g tiny=%g,%g\n", neutral.At(0).lastOrientationHalfAngle,
                neutral.At(1).lastOrientationHalfAngle, tiny.At(0).lastOrientationHalfAngle,
                tiny.At(1).lastOrientationHalfAngle);
    std::printf("INFO tiny outgoing ankle difference=%g m\n", difference);
    C("tiny outgoing warp remains millimetric across dissimilar live poses", difference < .005f);
    float storedDifference = 0;
    for (int i = 0; i < 7; ++i)
    {
        neutral.Step(M(incomingLfoot + (i % 2), .3f + .05f * i, true), .035f);
        tiny.Step(M(incomingLfoot + (i % 2), .3f + .05f * i, true), .035f);
        storedDifference = std::fmax(storedDifference, AnkleDifference(db, neutral.Output(), tiny.Output()));
    }
    std::printf("INFO tiny stored-tail max ankle difference=%g m\n", storedDifference);
    C("tiny warp stays millimetric through stored tail collapse", tiny.Collapses() > 0 && storedDifference < .005f);
    const std::vector<BoneTransform> before(tiny.Output(), tiny.Output() + db.NumBones());
    tiny.Step(M(incoming, .9f, true), 0);
    C("relative correction collapse preserves dt0 ankle continuity", AnkleDifference(db, before.data(), tiny.Output()) < .0001f);
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
    const int clip = ClipIndex(db, "M_Neutral_Run_Box_F_LL_Rfoot");
    C("fixture clip found by name", clip >= 0);
    if (clip < 0)
        return 2;
    C("actual clip long enough", db.Clip(clip).numFrames > 20);
    std::vector<float> factors(db.NumBones(), 1);
    factors[2] = .1f;
    motionstack::BlendStack s;
    C("init", s.Init(db, factors));
    C("first pose", s.Step(M(clip, .1f, false, 0, 2), 0));
    C("jump into same asset creates live outgoing", s.Step(M(clip, .4f, true, .5f, .5f), 0) && s.Count() == 2);
    C("head selected time authoritative", Near(s.At(0).time, .4f) && Near(s.At(1).time, .1f));
    C("dt0 new head has no weight", s.Weight(0) == 0 && s.Weight(1) == 1);
    s.Step(M(clip, .405f, false, .5f, .5f), .01f);
    C("outgoing keeps own rate", Near(s.At(1).time, .12f) && Near(s.At(0).time, .405f));
    C("depth accelerates only blend clock", Near(s.At(0).clock, .01f) && Near(s.At(1).clock, .0121f));
    s.Step(M(clip, .7f, true, .5f), 0);
    C("young head replaced under .03", s.Count() == 2 && Near(s.At(0).time, .7f) && Near(s.At(1).time, .12f));
    s.Step(M(clip, .74f, false, .5f), .04f);
    s.Step(M(clip, .2f, true, .5f), 0);
    C("older head preserved after .03", s.Count() == 3 && Near(s.At(1).time, .74f));
    for (int n = 0; n < 9; ++n)
    {
        s.Step(M(clip, .25f + .02f * n, false, .5f), .04f);
        std::vector<BoneTransform> before(s.Output(), s.Output() + db.NumBones());
        const auto beforeSignals = s.OutputSignals();
        s.Step(M(clip, .1f + .01f * n, true, .5f), 0);
        C("bounded stack including static tail", s.Count() <= 5);
        bool same = true;
        for (uint32_t b = 0; b < db.NumBones(); ++b)
        {
            const auto &x = before[b];
            const auto &y = s.Output()[b];
            same &= Near(x.p.x, y.p.x) && Near(x.p.y, y.p.y) && Near(x.p.z, y.p.z) &&
                    Near(std::fabs(math::QuatDot(x.q, y.q)), 1, 2e-5f);
        }
        C("dt0 insertion and tail collapse preserve entire prior output", same);
        C("dt0 collapse preserves co-timeline signals",
          Near(beforeSignals.rootX, s.OutputSignals().rootX) && Near(beforeSignals.left, s.OutputSignals().left));
    }
    C("more than four blends consolidated", s.Collapses() > 0 && s.At(s.Count() - 1).stored);
    C("newest never cut by consolidation", Near(s.At(0).time, .18f) && s.At(0).clip == clip);
    for (int b : {-1, 2, 12})
    {
        float total = 0;
        for (uint32_t i = 0; i < s.Count(); ++i)
        {
            float w = s.Weight(i, b);
            C("finite nonnegative weight", std::isfinite(w) && w >= 0 && w <= 1);
            total += w;
        }
        C("weights partition unity", Near(total, 1));
    }
    bool finite = true;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        const auto &p = s.Output()[b];
        finite &= math::IsFinite(p.q) && std::isfinite(p.p.x) && std::isfinite(p.p.y) && std::isfinite(p.p.z) &&
                  Near(math::QuatDot(p.q, p.q), 1, 1e-4f);
    }
    C("finite normalized mixed pose", finite);
    motionstack::BlendStack fast;
    fast.Init(db, factors);
    fast.Step(M(clip, .1f, false), 0);
    fast.Step(M(clip, .4f, true, .5f), .05f);
    C("fastest profile bone finishes before scalar", Near(fast.Weight(0, 2), 1) && fast.Weight(0, -1) < .1f);
    // Independent actual row sampler oracle for scalar signals at both live times.
    const auto expected = [&](const motionstack::Player &p) {
        const auto &cl = db.Clip(uint32_t(p.clip));
        float ft = std::clamp(p.time * db.Header().fps, 0.f, float(cl.numFrames - 1));
        uint32_t f = uint32_t(ft), g = std::min(f + 1, cl.numFrames - 1);
        float u = ft - f;
        const auto &a = db.Root(cl.firstFrame + f);
        const auto &z = db.Root(cl.firstFrame + g);
        const auto &x = db.Contact(cl.firstFrame + f);
        const auto &y = db.Contact(cl.firstFrame + g);
        return motionstack::Signals{std::lerp(a.dx, z.dx, u) * db.Header().fps * p.rootRate,
                               std::lerp(a.dy, z.dy, u) * db.Header().fps * p.rootRate,
                               std::lerp(a.dyaw, z.dyaw, u) * db.Header().fps * p.rootRate, std::lerp(x.l, y.l, u),
                               std::lerp(x.r, y.r, u)};
    };
    auto incomingSignal = expected(fast.At(0)), outgoingSignal = expected(fast.At(1));
    float incomingWeight = fast.Weight(0);
    const auto &o = fast.OutputSignals();
    C("root and contacts use same players times and scalar weights",
      Near(o.rootX, std::lerp(outgoingSignal.rootX, incomingSignal.rootX, incomingWeight)) && Near(o.rootY, std::lerp(outgoingSignal.rootY, incomingSignal.rootY, incomingWeight)) &&
          Near(o.rootYaw, std::lerp(outgoingSignal.rootYaw, incomingSignal.rootYaw, incomingWeight)) && Near(o.left, std::lerp(outgoingSignal.left, incomingSignal.left, incomingWeight)) &&
          Near(o.right, std::lerp(outgoingSignal.right, incomingSignal.right, incomingWeight)));
    // Verify actual pose mix, not only bookkeeping. Translation is linear and uses bone profile.
    std::vector<BoneTransform> pa(db.NumBones()), pb(db.NumBones());
    adapter::SamplePose(db, clip, fast.At(0).time, pa.data(), db.NumBones());
    adapter::SamplePose(db, clip, fast.At(1).time, pb.data(), db.NumBones());
    C("live outgoing sampled pose contributes",
      Near(fast.Output()[12].p.x, std::lerp(pb[12].p.x, pa[12].p.x, fast.Weight(0, 12))));
    C("fast profile completion equals incoming pose", Near(fast.Output()[2].p.x, pa[2].p.x) &&
                                                          Near(fast.Output()[2].p.y, pa[2].p.y) &&
                                                          Near(fast.Output()[2].p.z, pa[2].p.z));
    auto invalid = M(clip, .2f, false);
    invalid.playRate = std::numeric_limits<float>::quiet_NaN();
    C("nonfinite rate rejected", !fast.Step(invalid, .01f));
    C("negative dt rejected", !fast.Step(M(clip, .2f, false), -.1f));
    // Boundary exactly .03 is retained, unlike a younger head.
    motionstack::BlendStack boundary;
    boundary.Init(db);
    boundary.Step(M(clip, .1f, false), 0);
    boundary.Step(M(clip, .2f, true), .03f);
    boundary.Step(M(clip, .3f, true), 0);
    C("replacement threshold strictly less than .03", boundary.Count() == 3);

    C("out-of-range weight bone is safe zero", s.Weight(0, int32_t(db.NumBones())) == 0 && s.Weight(0, -2) == 0);
    auto tooSlow = factors;
    tooSlow[2] = 1.01f;
    motionstack::BlendStack invalidProfile;
    C("duration factor above one rejected", !invalidProfile.Init(db, tooSlow));
    C("negative selected time rejected", !s.Step(M(clip, -.01f, false), 0));
    int loop = -1, nonloop = -1;
    for (uint32_t i = 0; i < db.NumClips(); ++i)
    {
        const auto &c = db.Clip(i);
        if (c.numFrames < 5)
            continue;
        if ((c.flags & motion::kClipLoop) && c.loopPeriod && loop < 0)
            loop = int(i);
        if (!(c.flags & motion::kClipLoop) && nonloop < 0)
            nonloop = int(i);
    }
    C("actual loop and nonloop fixtures found", loop >= 0 && nonloop >= 0);
    if (loop >= 0)
    {
        motionstack::BlendStack wrap;
        wrap.Init(db);
        const float period = float(db.Clip(uint32_t(loop)).loopPeriod) / db.Header().fps;
        wrap.Step(M(loop, period - .01f, false, 0, 2), 0);
        wrap.Step(M(clip, .1f, true, .5f), .03f);
        C("outgoing loop advances own rate then wraps", Near(wrap.At(1).time, .05f, 2e-5f));
        const auto a = expected(wrap.At(0)), b = expected(wrap.At(1));
        const float w = wrap.Weight(0);
        C("wrapped outgoing contacts sampled at wrapped time",
          Near(wrap.OutputSignals().left, std::lerp(b.left, a.left, w)) &&
              Near(wrap.OutputSignals().rootX, std::lerp(b.rootX, a.rootX, w)));
    }
    if (nonloop >= 0)
    {
        motionstack::BlendStack end;
        end.Init(db);
        const auto &c = db.Clip(uint32_t(nonloop));
        const float length = float(c.numFrames - 1) / db.Header().fps;
        end.Step(M(nonloop, length - .01f, false, 0, 2), 0);
        end.Step(M(clip, .1f, true, .5f), .03f);
        C("outgoing nonloop clamps at last sample", Near(end.At(1).time, length));
        const auto a = expected(end.At(0)), b = expected(end.At(1));
        const float w = end.Weight(0);
        C("nonloop end root and contacts use endpoint sample and stored rate",
          Near(end.OutputSignals().left, std::lerp(b.left, a.left, w)) &&
              Near(end.OutputSignals().right, std::lerp(b.right, a.right, w)) &&
              Near(end.OutputSignals().rootY, std::lerp(b.rootY, a.rootY, w)));
        std::vector<BoneTransform> endpoint(db.NumBones());
        adapter::SamplePose(db, nonloop, length, endpoint.data(), db.NumBones());
        std::vector<BoneTransform> incoming(db.NumBones());
        adapter::SamplePose(db, clip, end.At(0).time, incoming.data(), db.NumBones());
        C("nonloop endpoint pose shares signal timeline",
          Near(end.Output()[12].p.y, std::lerp(endpoint[12].p.y, incoming[12].p.y, w)));
    }
    // Native .03 replacement is allowed to remove a young head's small contribution.
    // It is deliberately NOT a guarantee of C0 output on that replacement frame.
    motionstack::BlendStack young;
    young.Init(db);
    young.Step(M(clip, .05f, false), 0);
    young.Step(M(clip, .6f, true, .5f), .02f);
    std::vector<BoneTransform> prior(young.Output(), young.Output() + db.NumBones());
    const float olderTime = young.At(1).time;
    young.Step(M(clip, .3f, true, .5f), 0);
    std::vector<BoneTransform> older(db.NumBones());
    adapter::SamplePose(db, clip, olderTime, older.data(), db.NumBones());
    bool exactOlder = true;
    float removed = 0;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        exactOlder &= Near(young.Output()[b].p.x, older[b].p.x) &&
                      Near(std::fabs(math::QuatDot(young.Output()[b].q, older[b].q)), 1, 2e-5f);
        removed = std::fmax(removed, 1 - std::fabs(math::QuatDot(prior[b].q, young.Output()[b].q)));
    }
    C("young replacement output equals surviving older player at alpha0", exactOlder && young.Count() == 2);
    C("young replacement explicitly permits removal discontinuity", removed > 1e-8f);
    C("static tail contribution explicitly marked approximate", s.StaticTailWeight() > 0 && !s.RootReliable());
    motionstack::BlendStack endpoint;
    endpoint.Init(db);
    const auto &ec = db.Clip(uint32_t(nonloop));
    const float endTime = float(ec.numFrames - 1) / db.Header().fps;
    endpoint.Step(M(nonloop, endTime - .01f, false, 0, 2), 0);
    C("new post-search jump rate is explicit approximation",
      !endpoint.RootReliable() && endpoint.At(0).rootApproximate);
    endpoint.Step(M(clip, .2f, true, .5f), .04f);
    C("endpoint crossing preserves only actual partial interval", Near(endpoint.At(1).rootRate, .25f, 2e-4f));
    endpoint.Step(M(clip, .21f, false, .5f), .02f);
    C("already-ended outgoing emits zero ROOT rate", endpoint.At(1).rootRate == 0);
    const auto live = expected(endpoint.At(0));
    C("endpoint has no phantom ROOT while contact freezes",
      Near(endpoint.OutputSignals().rootX, live.rootX * endpoint.Weight(0)) &&
          Near(endpoint.OutputSignals().rootY, live.rootY * endpoint.Weight(0)));
    C("endpoint contact remains actual final key",
      Near(endpoint.OutputSignals().left,
           std::lerp(db.Contact(ec.firstFrame + ec.numFrames - 1).l, live.left, endpoint.Weight(0))));
    motionstack::BlendStack headEnd;
    headEnd.Init(db);
    headEnd.Step(M(nonloop, endTime - .01f, false), 0);
    headEnd.Step(M(nonloop, endTime, false), .04f);
    C("authoritative head endpoint partial interval", Near(headEnd.At(0).rootRate, .25f, 2e-4f));
    headEnd.Step(M(nonloop, endTime, false), .03f);
    C("authoritative head endpoint freezes ROOT",
      headEnd.OutputSignals().rootX == 0 && headEnd.OutputSignals().rootY == 0 && headEnd.RootReliable());
    const auto storedBefore = s.At(s.Count() - 1);
    for (float step : {.007f, .019f, .003f})
    {
        const float next = s.At(0).time + step * s.At(0).rate;
        C("variable-dt crossfade step", s.Step(M(clip, next, false), step));
        C("variable-dt tail remains frozen and flagged", s.At(s.Count() - 1).stored &&
                                                             Near(s.At(s.Count() - 1).time, storedBefore.time) &&
                                                             s.StaticTailWeight() > 0 && !s.RootReliable());
        C("variable-dt live player root interval follows own time", Near(s.At(0).rootRate, s.At(0).rate, 2e-4f));
    }
    motionstack::BlendStack bridge;
    bridge.Init(db);
    std::vector<BoneTransform> seed(db.NumBones());
    adapter::SamplePose(db, clip, .7f, seed.data(), db.NumBones());
    C("bridge seed one stored pose",
      bridge.SeedPose(seed) && bridge.Count() == 1 && bridge.At(0).stored && bridge.At(0).clip == -1);
    C("bridge seed rejects wrong size", !bridge.SeedPose(std::span<const BoneTransform>(seed.data(), seed.size() - 1)));
    auto corrupt = seed;
    corrupt[2].p.x = std::numeric_limits<float>::quiet_NaN();
    C("bridge seed rejects nonfinite pose", !bridge.SeedPose(corrupt));
    C("bridge seed rejects nonfinite signals", !bridge.SeedPose(seed, {std::numeric_limits<float>::infinity()}));
    bridge.Step(M(clip, .1f, true, .5f), 0);
    bool bridgeExact = true;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
        bridgeExact &= std::memcmp(&seed[b], &bridge.Output()[b], sizeof(BoneTransform)) == 0;
    C("bridge handback dt0 preserves seeded actual base", bridgeExact && bridge.Count() == 2 && bridge.Weight(0) == 0);
    bridge.Step(M(clip, .6f, false, .5f), .5f);
    std::vector<BoneTransform> complete(db.NumBones());
    adapter::SamplePose(db, clip, .6f, complete.data(), db.NumBones());
    bool endExact = true;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
        endExact &= std::memcmp(&complete[b], &bridge.Output()[b], sizeof(BoneTransform)) == 0;
    C("bridge completion reaches incoming endpoint and prunes seed",
      endExact && bridge.Count() == 1 && bridge.StaticTailWeight() == 0 && bridge.RootReliable());
    TinyOutgoingWarp(db);
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

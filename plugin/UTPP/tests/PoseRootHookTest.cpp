#include "../src/mm/MotionRootFrame.hpp"
#include "../src/mm/MotionPose.hpp"
#include <cstdio>
#include <memory>
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
struct Context
{
    rootframe::Processor root;
    rootframe::Inputs input;
    rootframe::Proposal proposal;
    visualroot::Frame seen{};
    math::Vec3 target{25, 60, 4};
    const BoneTransform *expected = nullptr;
    uint32_t n = 0, calls = 0;
    bool failHook = false, baseMatches = true, prepared = false;
    float expectedDt = 0;
};
static bool Hook(void *pointer, const motionpose::BaseFrameView &base, motionpose::OverlayFrameInputs &inputs)
{
    auto &c = *static_cast<Context *>(pointer);
    ++c.calls;
    c.baseMatches &= base.pose.size() == c.n && Near(base.dt, c.expectedDt) &&
                     std::memcmp(base.pose.data(), c.expected, c.n * sizeof(BoneTransform)) == 0;
    c.input.rootReliable = base.rootReliable;
    c.input.staticTailWeight = base.staticTailWeight;
    c.prepared = c.root.Prepare(c.input, c.proposal);
    if (!c.prepared)
        return false;
    c.seen = c.proposal.Frame();
    visualroot::Aim aim;
    if (!visualroot::ComputeAim(c.seen, c.target, aim))
        return false;
    inputs.haveAim = true;
    inputs.aimYawDeg = aim.yawDeg;
    inputs.aimPitchDeg = aim.pitchDeg;
    return !c.failHook;
}
int main(int argc, char **argv)
{
    const char *dbPath = argc > 1 ? argv[1] : "Build/test-data/LocomotionTwistFix-20261017/db/utpp_motion_woman.utppmm";
    const char *ovPath = argc > 2 ? argv[2]
                                  : "Build/test-data/ReloadPhaseProbe/priority-deployed-20261017/"
                                    "utpp_overlays_woman.utppov";
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    std::string error;
    C("actual392 database loads", db->LoadFromFile(dbPath, motion::kRigIdentityWoman, error));
    C("protected E7E5 overlay loads", ovf->LoadFromFile(ovPath, ov::kRigIdentityWomanOv, error));
    if (fail)
    {
        std::puts(error.c_str());
        return 2;
    }
    // the clip is found by name (clip indices differ between bakes)
    int hookClip = -1;
    for (uint32_t i = 0; i < db->NumClips(); ++i)
        if (db->ClipName(i) == "M_Neutral_Run_Box_F_LL_Rfoot")
            hookClip = int(i);
    C("fixture clip found by name", hookClip >= 0);
    if (hookClip < 0)
        return 2;
    auto hooked = std::make_unique<motionpose::PoseEvaluator>(), reference = std::make_unique<motionpose::PoseEvaluator>();
    C("hook evaluator init live",
      hooked->Init(db.get(), ovf.get(), nullptr, 2, error, motionpose::BaseBlendPolicy::LiveCrossfade));
    C("reference evaluator init live",
      reference->Init(db.get(), ovf.get(), nullptr, 2, error, motionpose::BaseBlendPolicy::LiveCrossfade));
    if (fail)
        return 2;
    hooked->SetGraphInputs({});
    reference->SetGraphInputs({});
    const uint32_t n = db->NumBones();
    std::vector<BoneTransform> expected(n), out(n), ref(n);
    Context context;
    context.root.Reset(.4f);
    context.n = n;
    context.expected = expected.data();
    context.input.dt = 1.f / 60;
    context.input.movingGround = true;
    context.input.actorYaw = .4f;
    context.input.capsuleYawDelta = .005f;
    context.input.capsuleDeltaCurrent = {.015f, .01f, 0};
    context.input.animationDeltaPreviousVisual = {.01f, .012f, 0};
    context.input.animationYawDelta = .002f;
    context.expectedDt = 1.f / 60;
    rootframe::Processor independent;
    independent.Reset(.4f);
    bool sawAO = false;
    for (int frame = 0; frame < 150; ++frame)
    {
        motion::MatchResult match;
        match.clip = hookClip;
        match.clipTime = .2f + .001f * frame;
        match.playRate = 1;
        adapter::SamplePose(*db, match.clip, match.clipTime, expected.data(), n);
        motionpose::OverlayFrameInputs input;
        input.machine.overlay = ov::PoseOverlay::Pistol2H;
        input.machine.style = ov::Style::Feminine;
        input.machine.wantsToAim = true;
        input.bodyWoman = true;
        input.haveAim = true;
        input.aimYawDeg = -7;
        input.aimPitchDeg = 3;
        input.preOverlay = Hook;
        input.preOverlayContext = &context;
        auto refInput = input;
        refInput.preOverlay = nullptr;
        refInput.preOverlayContext = nullptr;
        // Independent target conversion: inverse rigid root in double, float local
        // coordinates at the same boundary, then yaw/pitch from visual chest.
        rootframe::Proposal proposal;
        auto rootIn = context.input;
        C("reference root prepares current frame", independent.Prepare(rootIn, proposal));
        const auto f = proposal.Frame();
        const double c = std::cos(double(f.yaw)), s = std::sin(double(f.yaw)), dx = double(context.target.x) - f.x,
                     dy = double(context.target.y) - f.y;
        const float lx = float(c * dx + s * dy), ly = float(-s * dx + c * dy);
        const double dz = double(context.target.z) - 1.4f;
        refInput.aimYawDeg = float(std::atan2(-double(lx), double(ly)) * 180 / 3.14159265358979323846);
        refInput.aimPitchDeg = float(std::atan2(dz, std::hypot(double(lx), double(ly))) * 180 / 3.14159265358979323846);
        const uint32_t before = context.calls;
        motionpose::FrameInfo actualInfo, referenceInfo;
        bool ok = hooked->Evaluate(context.expectedDt, match, input, out.data(), n, &actualInfo),
             rok = reference->Evaluate(context.expectedDt, match, refInput, ref.data(), n, &referenceInfo);
        C("callback once after current base sampling",
          ok && rok && context.calls == before + 1 && context.baseMatches && context.prepared);
        C("caller inputs remain unchanged", input.aimYawDeg == -7 && input.aimPitchDeg == 3 &&
                                                input.preOverlay == Hook && input.preOverlayContext == &context);
        bool same = true;
        for (uint32_t b = 0; b < n; ++b)
            same &= Near(out[b].p.x, ref[b].p.x) && Near(out[b].p.y, ref[b].p.y) && Near(out[b].p.z, ref[b].p.z) &&
                    Near(std::fabs(math::QuatDot(out[b].q, ref[b].q)), 1, 2e-5f);
        C("full actual pipeline matches same-frame independent aim reference", same);
        C("AO diagnostics match independently converted reference",
          actualInfo.aoApplied == referenceInfo.aoApplied && Near(actualInfo.aoYawDeg, referenceInfo.aoYawDeg) &&
              Near(actualInfo.aoSweepU, referenceInfo.aoSweepU));
        sawAO |= actualInfo.aoApplied;
        if (ok)
            C("root commits only after evaluator success",
              context.root.Commit(context.proposal) && independent.Commit(proposal));
        context.input.actorYaw += context.input.capsuleYawDelta;
    }
    C("actual test exercised active AO", sawAO);
    motion::MatchResult match;
    match.clip = hookClip;
    match.clipTime = .6f;
    adapter::SamplePose(*db, match.clip, match.clipTime, expected.data(), n);
    motionpose::OverlayFrameInputs noHook;
    noHook.overlayWeight = 0;
    const auto previousCalls = context.calls;
    C("next call without hook succeeds", hooked->Evaluate(context.expectedDt, match, noHook, out.data(), n));
    C("previous local context not retained or recalled", context.calls == previousCalls);
    context.failHook = true;
    motionpose::OverlayFrameInputs failure;
    failure.preOverlay = Hook;
    failure.preOverlayContext = &context;
    failure.overlayWeight = 0;
    const auto seq = context.root.Sequence();
    const auto state = context.root.Frame();
    C("hook failure aborts Evaluate", !hooked->Evaluate(context.expectedDt, match, failure, out.data(), n));
    C("hook prepared candidate but no root commit on failure", context.prepared && context.root.Sequence() == seq &&
                                                                   context.root.Frame().x == state.x &&
                                                                   context.root.Frame().yaw == state.yaw);
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

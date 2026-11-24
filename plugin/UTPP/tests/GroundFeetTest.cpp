// Feet on uneven ground (mm/GroundFeet.hpp, mm/GroundFeetRuntime.hpp): flat ground is the body height stage bit for bit (named
// GroundFeetFlatIdentity), slopes 10 / 20 / 30 deg, a 17 cm step up and down, stairs 15 / 20 cm with one foot per step, a 12 cm curb
// snap of the capsule (sudden motion taken by the pelvis), a plane jump under a swinging foot, 30 / 60 / 144 Hz, missing / NaN traces,
// the reach and knee limits, the planted ball kept, a swinging foot over a slope (no tilt, the knee's fold kept), ground two steps
// ahead of a climbing capsule, the moving platform and the gate fade, and the animation-thread stepper; then the
// same terrain checks on the poses of the motion database when its path is given.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/GroundFeet.hpp"
#include "../src/mm/GroundFeetRuntime.hpp"

using namespace utpp::mm;
using namespace utpp::mm::groundfeet;
static int passed = 0, failed = 0;
static void Check(bool ok, const std::string& name)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name.c_str());
    ok ? ++passed : ++failed;
}

struct MockDb
{
    std::vector<std::string> names{"Root", "Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg",
                                   "RightFoot", "RightToeBase", "Spine"};
    std::vector<int32_t> parents{-1, 0, 1, 2, 3, 4, 1, 6, 7, 8, 1};
    bool Loaded() const { return true; }
    uint32_t NumBones() const { return uint32_t(names.size()); }
    int32_t BoneParent(uint32_t b) const { return parents[b]; }
    std::string_view BoneName(uint32_t b) const { return names[b]; }
};

static math::Quat X(float degrees) { return legik::AxisAngle({1, 0, 0}, degrees * math::kPi / 180.f); }

// standing / striding pose: bend (deg) of both knees, swing (deg) of the left leg forward (the right one back); the ball near z 0
static std::vector<float> MockPose(float bend, float swing)
{
    MockDb db;
    std::vector<float> p(db.NumBones() * 7, 0.0f);
    auto set = [&](int b, math::Vec3 t, math::Quat q) {
        float* v = &p[size_t(b) * 7];
        v[0] = t.x, v[1] = t.y, v[2] = t.z, v[3] = q.x, v[4] = q.y, v[5] = q.z, v[6] = q.w;
    };
    set(0, {0, 0, 0}, X(0));
    set(1, {0.0f, 0.0f, 0.95f}, X(0));
    for (int s = 0; s < 2; ++s)
    {
        const int u = s == 0 ? 2 : 6;
        const float sw = s == 0 ? swing : -swing;
        set(u, {s == 0 ? .1f : -.1f, 0, -.05f}, X(bend + sw));
        set(u + 1, {0, 0, -.44f}, X(-2 * bend));
        set(u + 2, {0, 0, -.43f}, X(bend - sw));
        set(u + 3, {0, .13f, -.05f}, X(0));
    }
    set(10, {0, 0, .1f}, X(0));
    return p;
}

using Ground = std::function<float(float, float)>;

// surface normal of h(x, y) by central differences
static math::Vec3 Normal(const Ground& h, float x, float y)
{
    const float e = 0.01f;
    const float gx = (h(x + e, y) - h(x - e, y)) / (2 * e), gy = (h(x, y + e) - h(x, y - e)) / (2 * e);
    return legik::Norm(math::Vec3{-gx, -gy, 1.0f});
}

// samples under the balls of aPose (model = world xy, capsule at the origin, height aCapZ)
static Samples Sample(const Ground& h, const bodyheight::LegRig& rig, const std::vector<float>& pose, float aCapZ, int64_t ticks = 1)
{
    PoseFrame f;
    Gather(rig, pose.data(), rig.numBones, f);
    Samples s;
    s.capZ = aCapZ;
    s.ticks = ticks;
    for (int k = 0; k < 2; ++k)
    {
        const float x = f.leg[k].ball.x, y = f.leg[k].ball.y;
        const math::Vec3 n = Normal(h, x, y);
        s.foot[k] = FootSample{1u, x, y, h(x, y), n.x, n.y, n.z};
    }
    return s;
}

struct Outcome
{
    ApplyResult r;
    std::vector<float> pose;
    PoseFrame anim, after;
    Plan plan;
};

// runs the stage for aSeconds at aHz on a static pose over the ground h with the capsule at aCapZ(t); returns the last frame
static Outcome Run(const Ground& h, const std::vector<float>& pose, int hz, float aSeconds, const std::function<float(float)>& aCapZ,
                   float aBody = 0.0f, Stage* aStage = nullptr, std::vector<float>* aPelvisWorld = nullptr,
                   std::vector<float>* aBallWorld = nullptr)
{
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    Stage local;
    Stage& st = aStage ? *aStage : local;
    Outcome o;
    const float dt = 1.0f / float(hz);
    const int n = int(aSeconds * float(hz) + 0.5f);
    for (int i = 0; i < n; ++i)
    {
        const float t = float(i) * dt, cz = aCapZ(t);
        const Samples s = Sample(h, rig, pose, cz);
        const float drawn[3] = {0.0f, 0.0f, cz};
        o.plan = st.Step(dt, &s, true, false, drawn, true, aBody, rig, pose.data(), rig.numBones); // a fresh stage takes dt 0 itself
        o.pose = pose;
        o.r = Apply(Config{}, rig, o.pose.data(), rig.numBones, o.plan, aBody);
        Gather(rig, pose.data(), rig.numBones, o.anim);
        Gather(rig, o.pose.data(), rig.numBones, o.after);
        if (aPelvisWorld)
            aPelvisWorld->push_back(o.after.pel.p.z + cz);
        if (aBallWorld)
            aBallWorld->push_back(o.after.leg[0].ball.z + cz);
    }
    return o;
}

static float KneeInterior(const LegFrame& l)
{
    const math::Vec3 a = l.ga.p - l.gb.p, b = l.gc.p - l.gb.p;
    return std::acos(std::clamp(math::Dot(a, b) / (legik::Len(a) * legik::Len(b)), -1.0f, 1.0f)) * 57.29578f;
}

static float FootPitchDeg(const PoseFrame& f0, const PoseFrame& f1, int s)
{
    // angle between the animated and the placed foot direction (ankle -> ball)
    const math::Vec3 a = legik::Norm(f0.leg[s].ball - f0.leg[s].gc.p), b = legik::Norm(f1.leg[s].ball - f1.leg[s].gc.p);
    return std::acos(std::clamp(math::Dot(a, b), -1.0f, 1.0f)) * 57.29578f;
}

static void FlatIdentity()
{
    // GroundFeetFlatIdentity: traces exactly at the capsule height (and within the dead zone: +-4 mm, 1.5 deg) never change the pose;
    // the runtime returns false and the body height stage alone gives the same bytes
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    bool inactive = true, untouched = true;
    for (float noise : {0.0f, 0.004f, -0.004f})
        for (float bend : {8.0f, 25.0f})
        {
            const auto pose = MockPose(bend, 15.0f);
            const float tilt = noise != 0.0f ? 1.5f * math::kPi / 180.0f : 0.0f;
            Ground h = [&](float, float y) { return noise + std::tan(tilt) * y * 0.0f; };
            Stage st;
            for (int i = 0; i < 90; ++i)
            {
                Samples s = Sample(h, rig, pose, 0.0f);
                for (auto& f : s.foot)
                {
                    f.ny = -std::sin(tilt);
                    f.nz = std::cos(tilt);
                }
                const float drawn[3] = {0, 0, 0};
                const Plan& p = st.Step(i ? 1.0f / 60 : 0.0f, &s, true, false, drawn, true, -0.02f, rig, pose.data(), rig.numBones);
                std::vector<float> a = pose;
                const ApplyResult r = Apply(Config{}, rig, a.data(), rig.numBones, p, -0.02f);
                inactive = inactive && !r.applied && p.pelvis == 0.0f;
                untouched = untouched && std::memcmp(a.data(), pose.data(), a.size() * sizeof(float)) == 0;
            }
        }
    Check(inactive && untouched, "GroundFeetFlatIdentity: flat ground (exact, +-4 mm, 1.5 deg) leaves the pose bit for bit, pelvis 0");
    // runtime: option on, flat traces -> StepAndApply false; the caller's body height stage gives the bytes of the stage alone
    rt::SetEnabled(true);
    bodyheight::rt::PublishRig(rig);
    rt::PublishGate(true, true);
    integ::PoseSnapshot snap{};
    snap.numBones = db.NumBones();
    snap.weight = 1.0f;
    const auto pose = MockPose(15.0f, 10.0f);
    std::memcpy(snap.pose, pose.data(), pose.size() * sizeof(float));
    const int64_t tps = 1000000;
    bool same = true, noApply = true;
    for (int i = 0; i < 60; ++i)
    {
        const int64_t now = 1000 + i * tps / 60;
        Samples s = Sample([](float, float) { return 0.0f; }, rig, pose, 0.0f, now);
        s.serial = rt::g_gate.load() >> 8;
        rt::g_samples.Publish(s);
        integ::PoseSnapshot a = snap, b = snap;
        const float drawn[3] = {0, 0, 0};
        const bool applied = rt::StepAndApply(a, now, tps, true, true, drawn, -0.02f);
        noApply = noApply && !applied;
        if (!applied)
            bodyheight::rt::ApplyToSnapshot(a, -0.02f);
        bodyheight::rt::ApplyToSnapshot(b, -0.02f);
        same = same && std::memcmp(a.pose, b.pose, sizeof(a.pose)) == 0;
    }
    Check(noApply && same, "GroundFeetFlatIdentity runtime: option on over flat traces = the body height stage alone, bit for bit");
    rt::SetEnabled(false);
    integ::PoseSnapshot c = snap;
    const float drawn[3] = {0, 0, 0.3f};
    Check(!rt::StepAndApply(c, 5 * tps, tps, true, true, drawn, 0.0f) && std::memcmp(c.pose, snap.pose, sizeof(c.pose)) == 0,
          "option off: no step, pose untouched");
}

static void Slopes()
{
    for (float deg : {10.0f, 20.0f, 30.0f})
    {
        const float k = std::tan(deg * math::kPi / 180.0f);
        Ground h = [k](float, float y) { return k * y; };
        const auto pose = MockPose(12.0f, 18.0f);
        const Outcome o = Run(h, pose, 60, 1.5f, [](float) { return 0.0f; });
        float sole = 0.0f, xy = 0.0f, pitchErr = 0.0f;
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 b0 = o.anim.leg[s].ball, b1 = o.after.leg[s].ball;
            const float g = h(b0.x, b0.y);
            const float want = b0.z + g - (g > 0.0f ? 0.005f : -0.005f); // the dead zone is subtracted
            // knee-limited / clamped legs may stop short of the ground (never below it)
            if (o.r.kneeLimited == 0 && o.r.clamped == 0)
            {
                sole = std::fmax(sole, std::fabs(b1.z - want));
                xy = std::fmax(xy, std::hypot(b1.x - b0.x, b1.y - b0.y)); // a limited leg pulls its foot back towards the animation
            }
            const float expect = std::fmin(deg, 25.0f) - 2.0f;
            pitchErr = std::fmax(pitchErr, std::fabs(FootPitchDeg(o.anim, o.after, s) - expect));
        }
        char name[200];
        std::snprintf(name, sizeof name,
                      "slope %.0f deg: balls on the ground (err %.1f mm), ball xy kept (%.2f mm), pitch err %.2f deg, pelvis %.1f cm, limited %d",
                      double(deg), double(sole) * 1000.0, double(xy) * 1000.0, double(pitchErr), double(o.plan.pelvis) * 100.0,
                      o.r.kneeLimited + o.r.clamped);
        Check(sole < 0.01f && xy < 0.001f && pitchErr < 1.0f && o.plan.pelvis <= 0.0f, name);
    }
}

static void Steps()
{
    // the left ball on a step 17 cm up (edge at y = 0.06), the capsule on the lower level: the left ball lifted, the right one stays
    const auto pose = MockPose(12.0f, 18.0f);
    {
        Ground h = [](float, float y) { return y > 0.06f ? 0.17f : 0.0f; };
        const Outcome o = Run(h, pose, 60, 1.0f, [](float) { return 0.0f; });
        const float l = o.after.leg[0].ball.z - o.anim.leg[0].ball.z, r = o.after.leg[1].ball.z - o.anim.leg[1].ball.z;
        const float flex = KneeInterior(o.anim.leg[0]) - KneeInterior(o.after.leg[0]);
        char name[200];
        std::snprintf(name, sizeof name, "step up 17 cm: left ball +%.1f cm, right %.2f cm, extra knee fold %.1f deg, knee limited %d",
                      double(l) * 100, double(r) * 100, double(flex), o.r.kneeLimited);
        Check(std::fabs(l - 0.165f) < 0.01f && std::fabs(r) < 0.005f && flex <= 60.5f, name);
    }
    {
        // step down: the right ball over a step 17 cm down; the pelvis drops so the leg reaches
        Ground h = [](float, float y) { return y < -0.06f ? -0.17f : 0.0f; };
        const Outcome o = Run(h, pose, 60, 1.0f, [](float) { return 0.0f; });
        const float r = o.after.leg[1].ball.z - o.anim.leg[1].ball.z;
        char name[200];
        std::snprintf(name, sizeof name, "step down 17 cm: right ball %.1f cm, pelvis %.1f cm, clamped %d", double(r) * 100,
                      double(o.plan.pelvis) * 100, o.r.clamped);
        Check(r < -0.15f && o.plan.pelvis < -0.05f && o.plan.pelvis >= -0.25f, name);
    }
    for (float rise : {0.15f, 0.20f})
    {
        // stairs, standing with one foot per step: the capsule in the middle (the game smooths it between the levels)
        Ground h = [rise](float, float y) { return y > 0.0f ? 0.5f * rise : -0.5f * rise; };
        const Outcome o = Run(h, pose, 60, 1.5f, [](float) { return 0.0f; });
        float err = 0.0f, flex = 0.0f;
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 b0 = o.anim.leg[s].ball;
            err = std::fmax(err, std::fabs(o.after.leg[s].ball.z - (b0.z + h(b0.x, b0.y)) + (h(b0.x, b0.y) > 0 ? 0.005f : -0.005f)));
            flex = std::fmax(flex, KneeInterior(o.anim.leg[s]) - KneeInterior(o.after.leg[s]));
        }
        char name[200];
        std::snprintf(name, sizeof name, "stairs %.0f cm, one foot per step: balls on their steps (err %.1f mm), pelvis %.1f cm, extra knee fold %.1f deg",
                      double(rise) * 100, double(err) * 1000, double(o.plan.pelvis) * 100, double(flex));
        Check(err < 0.012f && o.plan.pelvis < 0.0f && flex <= 60.5f, name);
    }
}

static void CurbSnap()
{
    // a 12 cm curb: the capsule snaps up in one frame while the left foot stays on the lower level (the right one stepped up). The left
    // ball keeps its world height and the pelvis does not jump with the capsule (sudden motion into the pelvis spring)
    const auto pose = MockPose(12.0f, 18.0f);
    Ground h = [](float, float y) { return y < 0.0f ? 0.12f : 0.0f; }; // right (behind) on the curb
    std::vector<float> pel, ball;
    Stage st;
    Run(h, pose, 60, 1.0f, [](float t) { return t < 0.5f ? 0.0f : 0.12f; }, 0.0f, &st, &pel, &ball);
    float pelJump = 0.0f, ballJump = 0.0f;
    for (size_t i = 1; i < pel.size(); ++i)
    {
        pelJump = std::fmax(pelJump, std::fabs(pel[i] - pel[i - 1]));
        ballJump = std::fmax(ballJump, std::fabs(ball[i] - ball[i - 1]));
    }
    char name[200];
    std::snprintf(name, sizeof name, "curb 12 cm capsule snap: pelvis world step max %.1f cm per frame, lower ball %.2f cm, sudden %llu",
                  double(pelJump) * 100, double(ballJump) * 100, static_cast<unsigned long long>(st.Diagnostics().sudden));
    // the lower ball may move by the dead zone (5 mm) when its ground height leaves zero
    Check(pelJump < 0.03f && ballJump < 0.006f && st.Diagnostics().sudden >= 1, name);
}

static void SteadyDescent()
{
    // a steady descent (stairs taken fast, flat planes under both feet that sink with the capsule) at 30 and 60 Hz: 1.3 m/s is above
    // suddenStep per update at 30 Hz, yet it is expected motion; the pelvis must not climb (it did when every update counted as a snap)
    const auto pose = MockPose(12.0f, 18.0f);
    for (int hz : {30, 60})
    {
        float tNow = 0.0f;
        const float v = 1.3f;
        Ground h = [&tNow, v](float, float) { return -v * tNow; };
        std::vector<float> pel;
        Stage st;
        const Outcome o = Run(h, pose, hz, 2.0f, [&tNow, v](float t) { tNow = t; return -v * t; }, 0.0f, &st, &pel);
        float maxOff = 0.0f;
        const float dt = 1.0f / float(hz);
        for (size_t i = pel.size() / 2; i < pel.size(); ++i)
            maxOff = std::fmax(maxOff, std::fabs(pel[i] - (o.anim.pel.p.z - v * float(i) * dt)));
        char name[200];
        std::snprintf(name, sizeof name, "steady descent 1.3 m/s at %d Hz: pelvis offset max %.1f cm over the last second, snaps %llu", hz,
                      double(maxOff) * 100, static_cast<unsigned long long>(st.Diagnostics().sudden));
        Check(maxOff < 0.03f, name);
    }
}

static void PlaneJump()
{
    // the trace under the left foot jumps 17 cm (a swinging foot over a step edge): the ball rises over several frames, no pop; the
    // same height 0.1 s later at 30 / 60 / 144 Hz
    const auto pose = MockPose(12.0f, 18.0f);
    float at[3] = {};
    float worst60 = 0.0f;
    const int rates[3] = {30, 60, 144};
    for (int k = 0; k < 3; ++k)
    {
        Stage st;
        std::vector<float> ball;
        Run([](float, float) { return 0.0f; }, pose, rates[k], 0.5f, [](float) { return 0.0f; }, 0.0f, &st);
        Outcome o = Run([](float, float y) { return y > 0.06f ? 0.17f : 0.0f; }, pose, rates[k], 0.1f, [](float) { return 0.0f; }, 0.0f, &st,
                        nullptr, &ball);
        at[k] = o.after.leg[0].ball.z - o.anim.leg[0].ball.z;
        if (rates[k] == 60)
            for (size_t i = 1; i < ball.size(); ++i)
                worst60 = std::fmax(worst60, ball[i] - ball[i - 1]);
    }
    char name[200];
    std::snprintf(name, sizeof name, "plane jump 17 cm: ball step max %.1f cm per frame at 60 Hz; after 0.1 s %.1f / %.1f / %.1f cm at 30 / 60 / 144 Hz",
                  double(worst60) * 100, double(at[0]) * 100, double(at[1]) * 100, double(at[2]) * 100);
    Check(worst60 < 0.06f && std::fabs(at[0] - at[2]) < 0.01f && std::fabs(at[1] - at[2]) < 0.005f, name);
}

static void RisingPlaneLimit()
{
    // a swinging foot over a rising step edge: on the first update after the trace jumps 17 cm the planting plane is at most
    // maxPenetration below the new ground (the foot is not dragged through the step while the height spring catches up)
    const auto pose = MockPose(12.0f, 18.0f);
    Stage st;
    Run([](float, float) { return 0.0f; }, pose, 60, 0.5f, [](float) { return 0.0f; }, 0.0f, &st);
    const Outcome o = Run([](float, float y) { return y > 0.06f ? 0.17f : 0.0f; }, pose, 60, 1.0f / 60.0f, [](float) { return 0.0f; }, 0.0f,
                          &st);
    const float lag = 0.17f - o.plan.foot[0].h;
    char name[200];
    std::snprintf(name, sizeof name, "rising step 17 cm: plane %.1f cm below the new ground on the first update (limit 5 cm)",
                  double(lag) * 100);
    Check(o.plan.foot[0].w > 0.99f && lag <= 0.051f, name);
}

static void LedgeAndStairDown()
{
    // ground 45 cm below the capsule under the right ball: on level ground (a ledge ahead) it is not followed; while the capsule
    // descends steadily (stairs down, the trace ahead of the capsule) it is (the plane trails the sinking ground by the height spring lag)
    const auto pose = MockPose(12.0f, 18.0f);
    {
        Stage st;
        const Outcome o = Run([](float, float y) { return y < -0.06f ? -0.45f : 0.0f; }, pose, 60, 1.0f, [](float) { return 0.0f; }, 0.0f,
                              &st);
        const float r = o.after.leg[1].ball.z - o.anim.leg[1].ball.z;
        char name[200];
        std::snprintf(name, sizeof name, "ledge 45 cm on level ground: right foot weight %.2f, right ball %.1f cm, left weight %.2f",
                      double(o.plan.foot[1].w), double(r) * 100, double(o.plan.foot[0].w));
        Check(o.plan.foot[1].w == 0.0f && std::fabs(r) < 0.01f && o.plan.foot[0].w > 0.99f, name);
    }
    {
        float tNow = 0.0f;
        const float v = 1.0f;
        Ground h = [&tNow, v](float, float y) { return (y < -0.06f ? -0.45f : 0.0f) - v * tNow; };
        Stage st;
        const Outcome o = Run(h, pose, 60, 1.0f, [&tNow, v](float t) { tNow = t; return -v * t; }, 0.0f, &st);
        char name[200];
        std::snprintf(name, sizeof name, "ground 45 cm below while descending 1 m/s: right foot weight %.2f, plane %.1f cm", double(o.plan.foot[1].w),
                      double(o.plan.foot[1].h) * 100);
        Check(o.plan.foot[1].w > 0.99f && o.plan.foot[1].h < -0.37f && o.plan.foot[1].h > -0.47f, name);
    }
}

static void BadTraces()
{
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    const auto pose = MockPose(12.0f, 18.0f);
    Stage st;
    Ground h = [](float, float y) { return y > 0.06f ? 0.17f : 0.0f; };
    const float drawn[3] = {0, 0, 0};
    for (int i = 0; i < 60; ++i)
    {
        const Samples s = Sample(h, rig, pose, 0.0f);
        st.Step(i ? 1.0f / 60 : 0.0f, &s, true, false, drawn, true, 0.0f, rig, pose.data(), rig.numBones);
    }
    const bool wasActive = st.Current().foot[0].w > 0.99f;
    // no hit / NaN / a hit 0.5 m above the capsule (a car roof): the weights fade, the plan ends, the pose is untouched again
    Plan p;
    for (int i = 0; i < 60; ++i)
    {
        Samples s = Sample(h, rig, pose, 0.0f);
        s.foot[0].hit = i % 3 == 0 ? 0u : 1u;
        s.foot[0].z = i % 3 == 1 ? std::nanf("") : 0.5f;
        s.foot[1].z = std::nanf("");
        p = st.Step(1.0f / 60, &s, true, false, drawn, true, 0.0f, rig, pose.data(), rig.numBones);
    }
    std::vector<float> a = pose;
    const ApplyResult r = Apply(Config{}, rig, a.data(), rig.numBones, p, 0.0f);
    Check(wasActive && p.foot[0].w == 0.0f && p.foot[1].w == 0.0f && !p.active && !r.applied &&
              std::memcmp(a.data(), pose.data(), a.size() * sizeof(float)) == 0,
          "missing / NaN / inconsistent traces: the stage fades out and the pose is untouched");
    // NaN drawn height: restart
    const float bad[3] = {0, 0, std::nanf("")};
    const Samples s = Sample(h, rig, pose, 0.0f);
    st.Step(1.0f / 60, &s, true, false, bad, true, 0.0f, rig, pose.data(), rig.numBones);
    Check(!st.Current().active && st.Diagnostics().resets >= 1, "non-finite drawn height: restart, no plan");
}

static void Limits()
{
    const auto pose = MockPose(10.0f, 18.0f);
    {
        // the right foot over a 30 cm drop: the pelvis drops at most 25 cm, the leg reaches what it can, no NaN
        Ground h = [](float, float y) { return y < -0.06f ? -0.30f : 0.0f; };
        const Outcome o = Run(h, pose, 60, 1.0f, [](float) { return 0.0f; });
        bool finite = true;
        for (float v : o.pose)
            finite = finite && std::isfinite(v);
        const float r = o.after.leg[1].ball.z - o.anim.leg[1].ball.z;
        char name[200];
        std::snprintf(name, sizeof name, "reach limit (30 cm drop): finite, pelvis %.1f cm >= -25, right ball %.1f cm (not below the ground)",
                      double(o.plan.pelvis) * 100, double(r) * 100);
        Check(finite && o.plan.pelvis >= -0.25f - 1e-4f && r >= -0.30f - 0.002f, name);
    }
    {
        // the left foot under a 30 cm block: the knee folds at most 60 deg more than in the animation
        Ground h = [](float, float y) { return y > 0.06f ? 0.30f : 0.0f; };
        const Outcome o = Run(h, pose, 60, 1.0f, [](float) { return 0.0f; });
        const float flex = KneeInterior(o.anim.leg[0]) - KneeInterior(o.after.leg[0]);
        char name[200];
        std::snprintf(name, sizeof name, "knee limit (30 cm block): extra fold %.1f deg <= 60, limited %d", double(flex), o.r.kneeLimited);
        Check(flex <= 60.5f, name);
    }
}

static void PlatformAndGate()
{
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    const auto pose = MockPose(12.0f, 18.0f);
    Ground h = [](float, float y) { return y > 0.06f ? 0.17f : 0.0f; };
    for (int mode = 0; mode < 2; ++mode)
    {
        Stage st;
        const float drawn[3] = {0, 0, 0};
        for (int i = 0; i < 60; ++i)
        {
            const Samples s = Sample(h, rig, pose, 0.0f);
            st.Step(i ? 1.0f / 60 : 0.0f, &s, true, false, drawn, true, 0.0f, rig, pose.data(), rig.numBones);
        }
        float t0 = -1.0f;
        for (int i = 0; i < 120; ++i)
        {
            const Samples s = Sample(h, rig, pose, 0.0f);
            const Plan& p = st.Step(1.0f / 60, &s, mode == 1 ? false : true, mode == 0, drawn, true, 0.0f, rig, pose.data(), rig.numBones);
            if (t0 < 0.0f && !p.active)
                t0 = float(i + 1) / 60.0f;
        }
        char name[160];
        std::snprintf(name, sizeof name, "%s: the stage fades out in %.2f s (< 1 s) and ends", mode == 0 ? "moving platform" : "gate closed",
                      double(t0));
        Check(t0 > 0.0f && t0 < 1.0f, name);
    }
}

static void Stepper()
{
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    bodyheight::rt::PublishRig(rig);
    rt::SetEnabled(true);
    rt::PublishGate(true, true);
    const auto pose = MockPose(12.0f, 18.0f);
    integ::PoseSnapshot snap{};
    snap.numBones = db.NumBones();
    snap.weight = 1.0f;
    std::memcpy(snap.pose, pose.data(), pose.size() * sizeof(float));
    const int64_t tps = 1000000;
    Ground h = [](float, float y) { return y > 0.06f ? 0.17f : 0.0f; };
    int64_t now = 10 * tps;
    bool applied = false;
    for (int i = 0; i < 60; ++i, now += tps / 60)
    {
        Samples s = Sample(h, rig, pose, 0.0f, now);
        s.serial = rt::g_gate.load() >> 8;
        rt::g_samples.Publish(s);
        integ::PoseSnapshot a = snap;
        const float drawn[3] = {0, 0, 0};
        applied = rt::StepAndApply(a, now, tps, true, true, drawn, 0.0f);
    }
    const uint64_t steps = rt::g_steps.load();
    integ::PoseSnapshot a = snap, b = snap;
    const float drawn[3] = {0, 0, 0};
    const int64_t last = now - tps / 60; // the frame of the last update
    rt::StepAndApply(a, last + 100, tps, true, true, drawn, 0.0f); // the same frame (0.1 ms later)
    rt::StepAndApply(b, last + 200, tps, true, true, drawn, 0.0f);
    Check(applied && rt::g_steps.load() == steps && std::memcmp(a.pose, b.pose, sizeof(a.pose)) == 0,
          "stepper: one update per frame, repeated calls apply the same plan");
    // stale traces (older than 0.25 s) are not used: the stage fades out
    for (int i = 0; i < 60; ++i)
    {
        now += tps / 60;
        integ::PoseSnapshot c = snap;
        rt::StepAndApply(c, now, tps, true, true, drawn, 0.0f);
    }
    integ::PoseSnapshot d = snap;
    Check(!rt::StepAndApply(d, now + tps / 60, tps, true, true, drawn, 0.0f) && rt::g_noSamples.load() > 0,
          "stepper: stale traces end the plan");
    integ::PoseSnapshot melee = snap;
    melee.lowerScale = 0.5f;
    Check(!rt::StepAndApply(melee, now + tps, tps, true, true, drawn, 0.0f) && std::memcmp(melee.pose, snap.pose, sizeof(snap.pose)) == 0,
          "snapshot: melee lower body on the game pose -> untouched");
    rt::SetEnabled(false);
}

static void ProbeAndSamples()
{
    // model -> world and back: a probe point taken with a turned capsule is stored at the same model point
    MockDb db;
    bodyheight::LegRig rig;
    rig.Init(db);
    bodyheight::rt::PublishRig(rig);
    rt::SetEnabled(true);
    const auto pose = MockPose(12.0f, 18.0f);
    rt::g_probe = rt::Probe{};
    rt::ProbeFromPose(pose.data(), rig.numBones);
    float wx = 0, wy = 0;
    const float yaw = 0.7f;
    const bool ok = rt::ProbePoint(0, 10.0f, -4.0f, yaw, wx, wy);
    const bool hit[2] = {true, true};
    const float pos[2][3] = {{wx, wy, 1.5f}, {wx, wy, 1.5f}};
    const float nrm[2][3] = {{0, 0, 1}, {0, 0, 1}};
    rt::PublishSamples(hit, pos, nrm, 10.0f, -4.0f, 1.5f, yaw, 7);
    Samples s;
    rt::g_samples.Read(s);
    PoseFrame f;
    Gather(rig, pose.data(), rig.numBones, f);
    Check(ok && std::hypot(s.foot[0].x - f.leg[0].ball.x, s.foot[0].y - f.leg[0].ball.y) < 1e-4f && s.foot[0].z == 1.5f,
          "probe point -> world -> sample: the same model point");
    rt::SetEnabled(false);
}

// database rig: a 17 cm step under the left ball and a 20 deg slope for sampled poses: finite, knee fold bound, balls on the ground
// unless a limit cut the correction
static void DatabasePoses(const char* aPath)
{
    Database db;
    std::string error;
    if (!db.LoadFromFile(aPath, 0x72CB314EB01F562Dull, error))
    {
        std::printf("database: %s\n", error.c_str());
        Check(false, "database loaded");
        return;
    }
    bodyheight::LegRig rig;
    Check(rig.Init(db), "database rig: leg rig found");
    const uint32_t step = std::max<uint32_t>(1, db.NumFrames() / 300);
    int poses = 0, limited = 0, nonFinite = 0, swinging = 0;
    float worstFold = 0.0f, worstSole = 0.0f, worstSwingDepth = 0.0f;
    const Ground grounds[2] = {[](float, float y) { return y > 0.0f ? 0.17f : 0.0f; },
                               [](float, float y) { return std::tan(20.0f * math::kPi / 180.0f) * y; }};
    for (uint32_t fr = 0; fr < db.NumFrames(); fr += step)
    {
        const auto p = db.Pose(fr);
        std::vector<float> pose(p.begin(), p.end());
        for (const Ground& h : grounds)
        {
            Stage st;
            Plan plan;
            for (int i = 0; i < 30; ++i)
            {
                const Samples s = Sample(h, rig, pose, 0.0f);
                const float drawn[3] = {0, 0, 0};
                plan = st.Step(i ? 1.0f / 30 : 0.0f, &s, true, false, drawn, true, 0.0f, rig, pose.data(), rig.numBones);
            }
            std::vector<float> a = pose;
            const ApplyResult r = Apply(Config{}, rig, a.data(), rig.numBones, plan, 0.0f);
            for (float v : a)
                nonFinite += std::isfinite(v) ? 0 : 1;
            PoseFrame f0, f1;
            if (!Gather(rig, pose.data(), rig.numBones, f0) || !Gather(rig, a.data(), rig.numBones, f1))
                continue;
            ++poses;
            limited += r.kneeLimited + r.clamped;
            for (int s = 0; s < 2; ++s)
            {
                worstFold = std::fmax(worstFold, KneeInterior(f0.leg[s]) - KneeInterior(f1.leg[s]));
                if (plan.foot[s].carry < 1.0f)
                {
                    // a foot in the swing keeps clearance instead of standing on the ground: its ball must not end in the ground
                    ++swinging;
                    const math::Vec3 b1 = f1.leg[s].ball;
                    worstSwingDepth = std::fmax(worstSwingDepth, h(b1.x, b1.y) - b1.z);
                }
                else if (r.kneeLimited == 0 && r.clamped == 0)
                {
                    const math::Vec3 b0 = f0.leg[s].ball;
                    const float g = h(b0.x, b0.y);
                    worstSole = std::fmax(worstSole, std::fabs(f1.leg[s].ball.z - b0.z - g) - (std::fabs(g) > 0.006f ? 0.0051f : std::fabs(g)));
                }
            }
        }
    }
    char name[300];
    std::snprintf(name, sizeof name, "database poses (%d, step 17 cm / slope 20 deg): finite, extra knee fold max %.1f deg <= 60, planted ball err %.1f mm, "
                  "swinging legs %d with the ball at most %.1f mm in the ground, limited legs %d",
                  poses, double(worstFold), double(worstSole) * 1000, swinging, double(worstSwingDepth) * 1000, limited);
    Check(nonFinite == 0 && worstFold <= 60.5f && worstSole < 0.012f && worstSwingDepth < 0.005f, name);
}

static void SwingFoot()
{
    // the left foot high in the swing (ball well above the clip's floor) over a 20 deg slope with the pelvis lowered by the body
    // offset: it is not tilted to the slope and keeps the animation's knee fold (it moves with the pelvis); the planted right foot is
    // tilted to the slope
    auto pose = MockPose(12.0f, 18.0f);
    {
        float* knee = &pose[size_t(3) * 7 + 3]; // LeftLeg: fold the knee so the foot swings up behind
        const math::Quat q = X(-24.0f - 100.0f);
        knee[0] = q.x, knee[1] = q.y, knee[2] = q.z, knee[3] = q.w;
    }
    const float t20 = std::tan(20.0f * math::kPi / 180.0f);
    const Outcome o = Run([t20](float, float y) { return t20 * y; }, pose, 60, 1.0f, [](float) { return 0.0f; }, -0.05f);
    const float above = o.anim.leg[0].ball.z - o.anim.leg[1].ball.z;
    const float knee = KneeInterior(o.after.leg[0]) - KneeInterior(o.anim.leg[0]);
    const float pitchL = FootPitchDeg(o.anim, o.after, 0), pitchR = FootPitchDeg(o.anim, o.after, 1);
    char name[240];
    std::snprintf(name, sizeof name, "swing foot %.0f cm up over a 20 deg slope, pelvis -5 cm: left carry %.2f, knee %+.2f deg, pitch %.1f deg; "
                  "right (planted) pitch %.1f deg", double(above) * 100, double(o.plan.foot[0].carry), double(knee), double(pitchL),
                  double(pitchR));
    Check(above > 0.1f && o.plan.foot[0].carry == 0.0f && std::fabs(knee) < 1.0f && pitchL < 0.5f && pitchR > 10.0f, name);
}

static void StairUpAhead()
{
    // ground 45 cm above the capsule under the right ball: on level ground (a crate, a car roof) it is not used; while the capsule
    // climbs steadily (running up stairs: the foot lands two steps ahead of the capsule) it is
    const auto pose = MockPose(12.0f, 18.0f);
    {
        Stage st;
        const Outcome o = Run([](float, float y) { return y < -0.06f ? 0.45f : 0.0f; }, pose, 60, 1.0f, [](float) { return 0.0f; }, 0.0f,
                              &st);
        char name[200];
        std::snprintf(name, sizeof name, "ground 45 cm above on level ground: right foot weight %.2f, left weight %.2f",
                      double(o.plan.foot[1].w), double(o.plan.foot[0].w));
        Check(o.plan.foot[1].w == 0.0f && o.plan.foot[0].w > 0.99f, name);
    }
    {
        float tNow = 0.0f;
        const float v = 1.0f;
        Ground h = [&tNow, v](float, float y) { return (y < -0.06f ? 0.45f : 0.0f) + v * tNow; };
        Stage st;
        const Outcome o = Run(h, pose, 60, 1.0f, [&tNow, v](float t) { tNow = t; return v * t; }, 0.0f, &st);
        char name[200];
        std::snprintf(name, sizeof name, "ground 45 cm above while climbing 1 m/s: right foot weight %.2f, plane %.1f cm, knee free %.0f deg",
                      double(o.plan.foot[1].w), double(o.plan.foot[1].h) * 100, double(o.plan.kneeFreeDeg));
        Check(o.plan.foot[1].w > 0.99f && o.plan.foot[1].h > 0.37f && o.plan.foot[1].h < 0.47f && o.plan.kneeFreeDeg == Config{}.kneeFreeClimbDeg,
              name);
    }
}

int main(int argc, char** argv)
{
    Check(rt::Enabled(), "default: the stage is on at startup (the menu default mm.groundFeet)");
    rt::SetEnabled(false); // the checks below switch it on where they need it
    FlatIdentity();
    Slopes();
    Steps();
    CurbSnap();
    SteadyDescent();
    PlaneJump();
    RisingPlaneLimit();
    LedgeAndStairDown();
    SwingFoot();
    StairUpAhead();
    BadTraces();
    Limits();
    PlatformAndGate();
    Stepper();
    ProbeAndSamples();
    if (argc >= 2)
        DatabasePoses(argv[1]);
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}

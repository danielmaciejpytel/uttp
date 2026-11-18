// Shrink-only stride warping (mm/StrideWarp.hpp) and its use in the blend stack (BlendStack::SetStrideWarp), 30 / 60 / 144 Hz.
// Solver: the scale never exceeds 1 (no lengthening) and never drops below the floor, follows the capsule / root ratio, is off below the
// minimum root speed and outside ground locomotion, alpha 0 is the identity, the interpolation is continuous and frame-rate independent,
// and a planted foot moving backwards at the clip's root speed moves at capsule speed after the warp.
// Blend stack (actual data, a run loop): a capsule slower than the clip shortens the ankle-to-thigh offset along the root direction by the
// scale and never lengthens it, the ankle height and the frame-to-frame ankle motion stay continuous, and a capsule at or above clip
// speed, a non-ground pick, a clip that is not a loop or the warp switched off leave the pose bit for bit unchanged.
// Clip factor: the scale divides by the clip's planted-foot speed over its root speed (measured per loop clip at Init, clamped, 1 for a
// clip that is not a loop or has too few planted samples), so a crouch loop whose planted feet travel slower than its root shortens less.
#include "../src/mm/MotionBlendStack.hpp"
#include "../src/mm/StrideWarp.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace utpp::mm;
static int passed = 0, failed = 0;
static void Check(bool ok, const char *name, int hz = 0)
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, hz ? " @Hz=" : "", hz ? std::to_string(hz).c_str() : "");
    ok ? ++passed : ++failed;
}

static stridewarp::Inputs In(float dt, float capsule, float root, float alpha = 1, bool allowed = true)
{
    stridewarp::Inputs in;
    in.dt = dt;
    in.capsuleSpeed = capsule;
    in.rootSpeed = root;
    in.alpha = alpha;
    in.allowed = allowed;
    return in;
}

static void Solver(int hz)
{
    const float dt = 1.0f / float(hz);
    // No lengthening, floor, ratio: a sweep of capsule and root speeds.
    bool bounded = true, ratio = true;
    for (float root = 0.2f; root < 8.0f; root += 0.37f)
        for (float capsule = 0.0f; capsule < 10.0f; capsule += 0.29f)
        {
            stridewarp::State st;
            float s = 1;
            for (int k = 0; k < hz; ++k)
                s = stridewarp::Step(st, In(dt, capsule, root));
            bounded = bounded && s <= 1.0f && s >= stridewarp::kScaleFloor;
            const float want = std::fmin(1.0f, std::fmax(stridewarp::kScaleFloor, capsule / root));
            ratio = ratio && std::fabs(s - want) < 1e-4f;
        }
    Check(bounded, "scale stays within [floor, 1] for every capsule / root speed (never lengthens)", hz);
    Check(ratio, "settled scale = capsule / root speed, clamped to [floor, 1]", hz);

    // Gates: not allowed, too slow a root, capsule at or above clip speed -> exactly 1.
    {
        stridewarp::State a, b, c;
        Check(stridewarp::Step(a, In(dt, 1.0f, 3.0f, 1.0f, false)) == 1.0f, "a non-ground pick is not warped", hz);
        Check(stridewarp::Step(b, In(dt, 0.0f, 0.09f)) == 1.0f, "a root slower than the minimum root speed is not warped", hz);
        Check(stridewarp::Step(c, In(dt, 4.0f, 3.0f)) == 1.0f, "a capsule faster than the clip is not warped (no lengthening)", hz);
    }
    // Alpha 0 = identity, bit for bit, also with a fully warped state underneath.
    {
        stridewarp::State st;
        for (int k = 0; k < hz; ++k)
            stridewarp::Step(st, In(dt, 1.0f, 3.0f));
        const float s = stridewarp::Step(st, In(dt, 1.0f, 3.0f, 0.0f));
        const math::Vec3 ankle{0.1f, 0.35f, 0.08f}, thigh{0.1f, 0.02f, 0.9f};
        const math::Vec3 w = stridewarp::WarpPoint(ankle, thigh, {0, 1, 0}, s);
        Check(s == 1.0f && stridewarp::IsIdentity(s) && w.x == ankle.x && w.y == ankle.y && w.z == ankle.z,
              "alpha 0 is the identity (scale 1, ankle unchanged)", hz);
        const float half = stridewarp::Step(st, In(dt, 1.0f, 3.0f, 0.5f));
        Check(std::fabs(half - (1.0f - 0.5f * (1.0f - st.scale))) < 1e-6f, "alpha scales the warp linearly", hz);
    }
    // Continuity and frame-rate independence: a capsule step from 0.8 to 1.2 of the clip speed.
    {
        stridewarp::State st;
        float s = stridewarp::Step(st, In(dt, 2.4f, 3.0f)), maxStep = 0, at05 = 0;
        const float bound = 1.0f - std::exp(-stridewarp::kInterpSpeed * dt);
        for (int k = 1; k <= hz; ++k)
        {
            const float next = stridewarp::Step(st, In(dt, 3.6f, 3.0f));
            maxStep = std::fmax(maxStep, std::fabs(next - s));
            s = next;
            if (k == hz / 2)
                at05 = s;
        }
        Check(maxStep <= 0.2f * bound + 1e-6f, "the scale moves continuously (no frame jumps beyond the 10 / s interpolation)", hz);
        // 0.5 s after the step at 10 / s: 1 - 0.2 * exp(-5) (half a second taken at a whole number of frames).
        const float want = 1.0f - 0.2f * std::exp(-stridewarp::kInterpSpeed * float(hz / 2) * dt);
        Check(std::fabs(at05 - want) < 1e-4f, "interpolation is frame-rate independent (same curve at 30 / 60 / 144 Hz)", hz);
    }
    // Planted foot: the ankle moves backwards in the character frame at the clip's root speed; after the warp it moves at capsule speed.
    {
        const float root = 3.0f, capsule = 2.4f;
        stridewarp::State st;
        float s = 1;
        for (int k = 0; k < hz; ++k)
            s = stridewarp::Step(st, In(dt, capsule, root));
        const math::Vec3 thigh{0.1f, 0.0f, 0.9f}, dir{0, 1, 0};
        const math::Vec3 a0{0.1f, 0.30f, 0.08f}, a1{0.1f, 0.30f - root * dt, 0.08f};
        const math::Vec3 w0 = stridewarp::WarpPoint(a0, thigh, dir, s), w1 = stridewarp::WarpPoint(a1, thigh, dir, s);
        const float speed = std::fabs(w1.y - w0.y) / dt;
        Check(std::fabs(speed - capsule) < 1e-3f, "a planted foot moves at capsule speed after the warp (no slide)", hz);
        Check(w0.z == a0.z && w0.x == a0.x, "only the component along the stride direction changes", hz);
    }
}

static int ClipIndex(const motion::MotionDatabase &db, const char *name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name)
            return int(i);
    return -1;
}

static int Bone(const motion::MotionDatabase &db, const char *name)
{
    for (uint32_t i = 0; i < db.NumBones(); ++i)
        if (db.BoneName(i) == name)
            return int(i);
    return -1;
}

static void Model(const motion::MotionDatabase &db, const BoneTransform *local, std::vector<BoneTransform> &model)
{
    model.resize(db.NumBones());
    for (uint32_t i = 0; i < db.NumBones(); ++i)
    {
        const int parent = db.BoneParent(i);
        model[i] = parent < 0 ? local[i]
                              : BoneTransform{model[size_t(parent)].p + math::QuatRotate(model[size_t(parent)].q, local[i].p),
                                              math::QuatNormalize(math::QuatMul(model[size_t(parent)].q, local[i].q))};
    }
}

static motion::MatchResult Pick(int clip, float t, bool jump, bool ground)
{
    motion::MatchResult m;
    m.clip = clip;
    m.clipTime = t;
    m.playRate = 1;
    m.jumped = jump;
    m.blend.valid = jump;
    m.blend.blendTime = 0.2f;
    m.blend.toGround = ground;
    return m;
}

// Runs the clip for 0.6 s on two stacks (warp off / on) with a capsule at `ratio` of the clip root speed.
struct RunStats
{
    bool ok = true, identical = true;
    float scale = 1, minScale = 1, alongError = 0, lengthened = 0, heightError = 0, jumpOff = 0, jumpOn = 0;
};
static RunStats Run(const motion::MotionDatabase &db, int clip, int hz, float ratio, bool ground, bool enabled)
{
    RunStats r;
    const float dt = 1.0f / float(hz), fps = db.Header().fps;
    motionstack::BlendStack off, on;
    r.ok = off.Init(db) && on.Init(db);
    off.SetStrideWarp(false);
    on.SetStrideWarp(enabled);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")}, thigh[2]{Bone(db, "LeftUpLeg"), Bone(db, "RightUpLeg")};
    const auto &c = db.Clip(uint32_t(clip));
    std::vector<BoneTransform> mOff, mOn;
    math::Vec3 lastOff[2]{}, lastOn[2]{};
    float t = 0.5f;
    for (int k = 0; k < int(0.6f * float(hz)) && r.ok; ++k, t += dt)
    {
        const auto &root = db.Root(c.firstFrame + uint32_t(t * fps));
        motionstack::GraphInputs g;
        g.enabled = true;
        g.velocityWorldX = root.dx * fps * ratio;
        g.velocityWorldY = root.dy * fps * ratio;
        off.SetGraphInputs(g);
        on.SetGraphInputs(g);
        r.ok = off.Step(Pick(clip, t, k == 0, ground), dt) && on.Step(Pick(clip, t, k == 0, ground), dt);
        if (!r.ok)
            break;
        Model(db, off.Output(), mOff);
        Model(db, on.Output(), mOn);
        r.identical = r.identical && std::memcmp(off.Output(), on.Output(), sizeof(BoneTransform) * db.NumBones()) == 0;
        r.scale = on.StrideScale();
        r.minScale = std::fmin(r.minScale, r.scale);
        const float len = std::hypot(root.dx, root.dy);
        const math::Vec3 dir = len > 0 ? math::Vec3{root.dx / len, root.dy / len, 0} : math::Vec3{0, 1, 0};
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 aOff = mOff[size_t(foot[s])].p, aOn = mOn[size_t(foot[s])].p;
            const float alongOff = math::Dot(aOff - mOff[size_t(thigh[s])].p, dir);
            const float alongOn = math::Dot(aOn - mOn[size_t(thigh[s])].p, dir);
            r.alongError = std::fmax(r.alongError, std::fabs(alongOn - r.scale * alongOff));
            r.lengthened = std::fmax(r.lengthened, std::fabs(alongOn) - std::fabs(alongOff));
            r.heightError = std::fmax(r.heightError, std::fabs(aOn.z - aOff.z));
            if (k > 0)
            {
                const math::Vec3 dOff = aOff - lastOff[s], dOn = aOn - lastOn[s];
                r.jumpOff = std::fmax(r.jumpOff, std::sqrt(math::Dot(dOff, dOff)));
                r.jumpOn = std::fmax(r.jumpOn, std::sqrt(math::Dot(dOn, dOn)));
            }
            lastOff[s] = aOff;
            lastOn[s] = aOn;
        }
    }
    return r;
}

static void Stack(const motion::MotionDatabase &db, int clip, int hz)
{
    const RunStats warped = Run(db, clip, hz, 0.8f, true, true);
    std::printf("INFO run loop capsule 0.8: scale %.4f min %.4f along error %.2f mm lengthened %.2f mm height %.2f mm "
                "frame jump off %.2f on %.2f mm @Hz=%d\n",
                warped.scale, warped.minScale, warped.alongError * 1e3f, warped.lengthened * 1e3f, warped.heightError * 1e3f,
                warped.jumpOff * 1e3f, warped.jumpOn * 1e3f, hz);
    Check(warped.ok, "stacks step on actual data", hz);
    Check(std::fabs(warped.scale - 0.8f) < 0.02f && warped.minScale >= stridewarp::kScaleFloor,
          "capsule at 0.8 of the clip speed: stride scale 0.8", hz);
    Check(warped.alongError < 0.005f, "ankle-to-thigh offset along the root direction shrinks by the scale (within 5 mm)", hz);
    Check(warped.lengthened < 0.001f, "no ankle moves farther from its thigh along the root direction (no lengthening)", hz);
    Check(warped.heightError < 0.005f, "ankle height kept (within 5 mm)", hz);
    Check(warped.jumpOn <= warped.jumpOff + 0.002f, "ankle motion stays continuous (largest frame step not above the unwarped one)", hz);
    Check(Run(db, clip, hz, 1.2f, true, true).identical, "capsule faster than the clip: pose bit for bit unchanged", hz);
    Check(Run(db, clip, hz, 0.8f, false, true).identical, "non-ground pick: pose bit for bit unchanged", hz);
    const int start = ClipIndex(db, "M_Neutral_Run_Start_F_Lfoot");
    Check(start >= 0 && Run(db, start, hz, 0.8f, true, true).identical, "a start clip (not a loop): pose bit for bit unchanged", hz);
    Check(Run(db, clip, hz, 0.8f, true, false).identical, "warp switched off: pose bit for bit unchanged", hz);
}

static void ClipFactorSolver(int hz)
{
    const float dt = 1.0f / float(hz);
    Check(stridewarp::ClipFactor(9.0, 10.0, 3) == 1.0f, "too few planted samples: clip factor 1", hz);
    Check(stridewarp::ClipFactor(9.0, 10.0, 40) == 0.9f, "clip factor = planted-foot speed over root speed", hz);
    Check(stridewarp::ClipFactor(5.0, 10.0, 40) == stridewarp::kClipFactorMin &&
              stridewarp::ClipFactor(20.0, 10.0, 40) == stridewarp::kClipFactorMax,
          "clip factor clamped to its range", hz);
    stridewarp::State a, b;
    stridewarp::Inputs in = In(dt, 2.4f, 3.0f);
    in.clipFactor = 0.9f;
    float s = 1;
    for (int k = 0; k < hz; ++k)
        s = stridewarp::Step(a, in);
    Check(std::fabs(s - 2.4f / (3.0f * 0.9f)) < 1e-4f, "settled scale = capsule / (root speed x clip factor)", hz);
    in.capsuleSpeed = 2.9f;
    Check(stridewarp::Step(b, in) == 1.0f, "a clip factor never lengthens the stride (scale stays at most 1)", hz);
}

static void StackClipFactor(const motion::MotionDatabase &db, int hz)
{
    motionstack::BlendStack probe;
    const int crouch = ClipIndex(db, "M_Neutral_Crouch_Loop_F"), start = ClipIndex(db, "M_Neutral_Run_Start_F_Lfoot");
    Check(crouch >= 0 && start >= 0 && probe.Init(db), "clip factor fixture clips found, stack initialised", hz);
    if (crouch < 0 || start < 0)
        return;
    const float k = probe.StrideClipFactor(crouch);
    std::printf("INFO crouch loop clip factor %.4f @Hz=%d\n", k, hz);
    Check(k >= stridewarp::kClipFactorMin && k < 0.995f, "crouch loop: planted feet slower than the root (clip factor below 1)", hz);
    Check(probe.StrideClipFactor(start) == 1.0f && probe.StrideClipFactor(-1) == 1.0f, "a clip that is not a loop keeps clip factor 1", hz);
    const RunStats r = Run(db, crouch, hz, 0.8f, true, true);
    Check(r.ok && std::fabs(r.scale - std::fmin(1.0f, 0.8f / k)) < 0.02f, "crouch loop, capsule at 0.8 of the root speed: scale 0.8 / clip factor", hz);
    Check(r.alongError < 0.005f && r.lengthened < 0.001f, "crouch loop: offset along the root direction shrinks by the scale, never lengthens", hz);
}

int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    for (int hz : {30, 60, 144})
        Solver(hz);
    for (int hz : {30, 60, 144})
        ClipFactorSolver(hz);
    motion::MotionDatabase db;
    std::string error;
    if (!db.LoadFromFile(argv[1], motion::kRigIdentityWoman, error))
    {
        std::printf("database: %s\n", error.c_str());
        return 2;
    }
    const int clip = ClipIndex(db, "M_Neutral_Run_Loop_F_R_20");
    Check(clip >= 0, "run loop fixture clip found by name");
    if (clip >= 0)
        for (int hz : {30, 60, 144})
            Stack(db, clip, hz);
    for (int hz : {30, 60, 144})
        StackClipFactor(db, hz);
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}

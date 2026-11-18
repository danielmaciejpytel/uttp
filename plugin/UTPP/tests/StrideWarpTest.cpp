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

static motion::MatchResult Pick(int clip, float t, bool jump, bool ground, float blend = 0.2f)
{
    motion::MatchResult m;
    m.clip = clip;
    m.clipTime = t;
    m.playRate = 1;
    m.jumped = jump;
    m.blend.valid = jump;
    m.blend.blendTime = blend;
    m.blend.toGround = ground;
    return m;
}

// Runs the clip for 0.6 s on two stacks (warp off / on) with a capsule at `ratio` of the clip root speed.
struct RunStats
{
    bool ok = true, identical = true;
    float scale = 1, minScale = 1, alongError = 0, lengthened = 0, heightError = 0, jumpOff = 0, jumpOn = 0;
};
// crouchMode >= 0 sets the crouched loop mode of the warped stack (SetStrideOptions, no yaw fade, whole-layer fade); -1 keeps the default.
static RunStats Run(const motion::MotionDatabase &db, int clip, int hz, float ratio, bool ground, bool enabled, int crouchMode = -1)
{
    RunStats r;
    const float dt = 1.0f / float(hz), fps = db.Header().fps;
    motionstack::BlendStack off, on;
    r.ok = off.Init(db) && on.Init(db);
    off.SetStrideWarp(false);
    on.SetStrideWarp(enabled);
    if (crouchMode >= 0)
        on.SetStrideOptions(false, crouchMode, true);
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

// A warped loop layer replaced by an unwarped pick of the same pose: its shortening fades out no faster than the whole-layer weight of
// the new pick (smoothstep over the blend time) and never drops by more than the scale interpolation rate allows in one update, also
// under a short blend. It used to be cut on the faster leg weight (or by pruning the warped layer) within a few updates and snap the knee.
// Three stacks run the same picks: `off` never warps, `cont` keeps warping through the new pick, `on` switches to an unwarped pick.
static void StackFadeOut(const motion::MotionDatabase &db, int clip, int hz, float blend)
{
    const float dt = 1.0f / float(hz), fps = db.Header().fps, release = 1.0f - std::exp(-stridewarp::kInterpSpeed * dt);
    motionstack::BlendStack off, cont, on;
    bool ok = off.Init(db) && cont.Init(db) && on.Init(db);
    off.SetStrideWarp(false);
    cont.SetStrideWarp(true);
    on.SetStrideWarp(true);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")};
    const auto &c = db.Clip(uint32_t(clip));
    std::vector<BoneTransform> mOff, mCont, mOn;
    const int switchAt = int(0.4f * float(hz));
    float t = 0.5f, fadeDeficit = 0, overshoot = 0, dropExcess = 0, shortening = 0, lastFade[2]{}, lastWarp[2]{};
    int compared = 0;
    for (int k = 0; k < switchAt + int(0.3f * float(hz)) && ok; ++k, t += dt)
    {
        const auto &root = db.Root(c.firstFrame + uint32_t(t * fps));
        motionstack::GraphInputs g;
        g.enabled = true;
        g.velocityWorldX = root.dx * fps * 0.8f;
        g.velocityWorldY = root.dy * fps * 0.8f;
        off.SetGraphInputs(g);
        cont.SetGraphInputs(g);
        on.SetGraphInputs(g);
        const bool jump = k == 0 || k == switchAt;
        ok = off.Step(Pick(clip, t, jump, true, blend), dt) && cont.Step(Pick(clip, t, jump, true, blend), dt) &&
             on.Step(Pick(clip, t, jump, k < switchAt, blend), dt);
        if (!ok || k < switchAt - 1)
            continue;
        Model(db, off.Output(), mOff);
        Model(db, cont.Output(), mCont);
        Model(db, on.Output(), mOn);
        const float x = std::clamp(float(k - switchAt + 1) * dt / blend, 0.0f, 1.0f);
        const float keep = k < switchAt ? 1.0f : 1.0f - x * x * (3 - 2 * x); // remaining whole-layer weight of the warped layer
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 warp = mCont[size_t(foot[s])].p - mOff[size_t(foot[s])].p;
            const math::Vec3 fade = mOn[size_t(foot[s])].p - mOff[size_t(foot[s])].p;
            const float w = std::sqrt(math::Dot(warp, warp)), f = std::sqrt(math::Dot(fade, fade));
            shortening = std::fmax(shortening, w);
            if (w > 0.01f)
            {
                fadeDeficit = std::fmax(fadeDeficit, keep - f / w);
                overshoot = std::fmax(overshoot, f / w - 1.0f);
                ++compared;
            }
            // one update releases at most the interpolation share of the held shortening (2 mm for the pose difference of the fading blend)
            if (k >= switchAt)
                dropExcess = std::fmax(dropExcess, (lastFade[s] - f) - (release * lastFade[s] + std::fabs(w - lastWarp[s]) + 0.002f));
            lastFade[s] = f;
            lastWarp[s] = w;
        }
    }
    std::printf("INFO fade-out after an unwarped pick, blend %.2f s: shortening up to %.2f cm, fade deficit %.3f, overshoot %.3f over %d "
                "samples, drop excess %.2f mm @Hz=%d\n",
                blend, shortening * 100.0f, fadeDeficit, overshoot, compared, dropExcess * 1e3f, hz);
    Check(ok && shortening > 0.02f && compared > 0, "fade-out fixture: warped loop with a shortening above 2 cm", hz);
    Check(fadeDeficit < 0.05f, "an unwarped pick fades the shortening out no faster than its whole-layer weight", hz);
    Check(overshoot < 0.05f, "the fading shortening never exceeds the continuing one", hz);
    Check(dropExcess <= 0.0f, "the shortening never drops faster than the interpolation rate in one update (no knee snap)", hz);
}

// Yaw fade: the warp weight of a smoothed visual yaw rate, the smoothing itself and its effect on the scale.
static void YawFadeSolver(int hz)
{
    const float dt = 1.0f / float(hz);
    const float mid = 0.5f * (stridewarp::kYawFadeStart + stridewarp::kYawFadeEnd);
    Check(stridewarp::YawFade(0.0f) == 1.0f && stridewarp::YawFade(stridewarp::kYawFadeStart) == 1.0f &&
              stridewarp::YawFade(stridewarp::kYawFadeEnd) == 0.0f && stridewarp::YawFade(4.0f) == 0.0f,
          "yaw fade: full warp up to the start rate, none from the end rate", hz);
    Check(std::fabs(stridewarp::YawFade(mid) - 0.5f) < 1e-5f && stridewarp::YawFade(-mid) == stridewarp::YawFade(mid),
          "yaw fade: linear between the rates, symmetric in the turn direction", hz);
    float r = 0, w = 0;
    for (int k = 0; k < 3 * hz; ++k)
    {
        r = stridewarp::SmoothYawRate(r, 1.0f * dt, dt);
        w = stridewarp::SmoothYawRate(w, -1.0f * dt + 6.28318531f, dt); // the same turn the other way, across the winding
    }
    Check(std::fabs(r - 1.0f) < 1e-3f && std::fabs(w - 1.0f) < 1e-3f, "smoothed yaw rate settles on |rate|, across the yaw winding", hz);
    Check(stridewarp::SmoothYawRate(0.7f, 1.0f, 0.0f) == 0.7f, "smoothed yaw rate holds at dt 0", hz);
    stridewarp::State a, b, c;
    stridewarp::Inputs in = In(dt, 2.4f, 3.0f);
    const float full = stridewarp::Step(a, in);
    in.yawFade = 0.0f;
    const float none = stridewarp::Step(b, in);
    in.yawFade = 0.5f;
    const float half = stridewarp::Step(c, in);
    Check(full < 1.0f && none == 1.0f && std::fabs(half - (1.0f - 0.5f * (1.0f - full))) < 1e-6f,
          "yaw fade 0 is the identity, 0.5 halves the shortening", hz);
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
    const RunStats r = Run(db, crouch, hz, 0.8f, true, true, 0); // crouched loops warped like the others
    Check(r.ok && std::fabs(r.scale - std::fmin(1.0f, 0.8f / k)) < 0.02f, "crouch loop, capsule at 0.8 of the root speed: scale 0.8 / clip factor", hz);
    Check(r.alongError < 0.005f && r.lengthened < 0.001f, "crouch loop: offset along the root direction shrinks by the scale, never lengthens", hz);
}

// Crouched loop modes (SetStrideOptions): 1 leaves a crouched loop bit for bit unwarped and still warps a standing loop; 2 under a steady
// visual yaw (no swing) warps it exactly like 0.
static void StackCrouchModes(const motion::MotionDatabase &db, int run, int hz)
{
    motionstack::BlendStack probe;
    const int crouch = ClipIndex(db, "M_Neutral_Crouch_Loop_F");
    Check(crouch >= 0 && probe.Init(db), "crouched loop fixture clip found, stack initialised", hz);
    if (crouch < 0)
        return;
    Check(probe.StrideCrouchClip(crouch) && !probe.StrideCrouchClip(run) && !probe.StrideCrouchClip(-1),
          "crouched loops recognised by name, a standing loop is not one", hz);
    Check(Run(db, crouch, hz, 0.8f, true, true, 1).identical, "crouched loop mode 1: crouched loop pose bit for bit unwarped", hz);
    const RunStats standing = Run(db, run, hz, 0.8f, true, true, 1);
    Check(standing.ok && std::fabs(standing.scale - 0.8f) < 0.02f, "crouched loop mode 1: a standing loop is still warped", hz);
    const RunStats a = Run(db, crouch, hz, 0.8f, true, true, 0), b = Run(db, crouch, hz, 0.8f, true, true, 2);
    Check(a.ok && b.ok && a.scale < 0.99f && a.scale == b.scale && a.alongError == b.alongError,
          "crouched loop mode 2 without a yaw swing warps like mode 0", hz);
}

// Crouched loop mode 2 under a sustained visual yaw swing (the visual root and the capsule velocity turning at 180 deg/s, well above the
// fade end): over the last 0.3 s of 1.5 s the crouched loop is no longer warped (ankles within 1 mm of the unwarped stack), while mode 0
// still warps it and a standing loop stays warped in mode 2 (the yaw fade applies to crouched loops only).
static float YawSwingWarp(const motion::MotionDatabase &db, int clip, int hz, int crouchMode)
{
    const float dt = 1.0f / float(hz), fps = db.Header().fps, rate = 3.14159265f;
    motionstack::BlendStack off, on;
    if (!off.Init(db) || !on.Init(db))
        return -1.0f;
    off.SetStrideWarp(false);
    on.SetStrideWarp(true);
    on.SetStrideOptions(false, crouchMode, true);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")};
    const auto &c = db.Clip(uint32_t(clip));
    std::vector<BoneTransform> mOff, mOn;
    const int steps = int(1.5f * float(hz)), from = steps - int(0.3f * float(hz));
    float t = 0.5f, warp = 0;
    for (int k = 0; k < steps; ++k, t += dt)
    {
        const auto &root = db.Root(c.firstFrame + uint32_t(std::fmod(t, 1.0f) * fps));
        const float yaw = rate * float(k) * dt, vx = root.dx * fps * 0.8f, vy = root.dy * fps * 0.8f;
        motionstack::GraphInputs g;
        g.enabled = true;
        g.previousVisualWorldYaw = g.targetWorldYaw = g.actorWorldYaw = yaw;
        g.velocityWorldX = std::cos(yaw) * vx - std::sin(yaw) * vy;
        g.velocityWorldY = std::sin(yaw) * vx + std::cos(yaw) * vy;
        off.SetGraphInputs(g);
        on.SetGraphInputs(g);
        if (!off.Step(Pick(clip, t, k == 0, true), dt) || !on.Step(Pick(clip, t, k == 0, true), dt))
            return -1.0f;
        if (k < from)
            continue;
        Model(db, off.Output(), mOff);
        Model(db, on.Output(), mOn);
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 d = mOn[size_t(foot[s])].p - mOff[size_t(foot[s])].p;
            warp = std::fmax(warp, std::sqrt(math::Dot(d, d)));
        }
    }
    return warp;
}
static void StackCrouchYawSwing(const motion::MotionDatabase &db, int run, int hz)
{
    const int crouch = ClipIndex(db, "M_Neutral_Crouch_Loop_F");
    if (crouch < 0)
        return;
    const float faded = YawSwingWarp(db, crouch, hz, 2), crouchWarped = YawSwingWarp(db, crouch, hz, 0),
                standing = YawSwingWarp(db, run, hz, 2);
    std::printf("INFO yaw swing 180 deg/s: largest warp offset of the ankles, crouched mode 2 %.2f mm, mode 0 %.2f mm, standing loop "
                "mode 2 %.2f mm @Hz=%d\n",
                faded * 1e3f, crouchWarped * 1e3f, standing * 1e3f, hz);
    Check(faded >= 0 && faded < 0.001f, "crouched loop mode 2 under a sustained yaw swing: not warped (within 1 mm)", hz);
    Check(crouchWarped > 0.01f, "crouched loop mode 0 under the same yaw swing: still warped (above 1 cm)", hz);
    Check(standing > 0.01f, "standing loop in mode 2 under the same yaw swing: still warped (above 1 cm)", hz);
}

// Stride direction swing: the capsule velocity turns by 90 degrees in one update under a warped loop (a turn pick changing the visual
// root does the same to the orientation warping target). The stride direction follows the angle orientation warping actually turned the
// legs, so the warped ankles may not step farther in one update than the unwarped ones beyond a small absolute margin. It used to follow
// the warped root signal, which turns by the full target angle at once and swung the shortening sideways (a visible knee snap).
static void StackDirectionSwing(const motion::MotionDatabase &db, int clip, int hz, float turn)
{
    const float dt = 1.0f / float(hz), fps = db.Header().fps;
    motionstack::BlendStack off, on;
    bool ok = off.Init(db) && on.Init(db);
    off.SetStrideWarp(false);
    on.SetStrideWarp(true);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")};
    const auto &c = db.Clip(uint32_t(clip));
    std::vector<BoneTransform> mOff, mOn;
    math::Vec3 lastOff[2]{}, lastOn[2]{};
    const int switchAt = int(0.4f * float(hz));
    float t = 0.5f, stepOff = 0, stepOn = 0, minScale = 1, warpStepBefore = 0, warpStepAfter = 0;
    for (int k = 0; k < switchAt + int(0.3f * float(hz)) && ok; ++k, t += dt)
    {
        const auto &root = db.Root(c.firstFrame + uint32_t(t * fps));
        const float a = k >= switchAt ? turn : 0.0f;
        const float vx = root.dx * fps * 0.8f, vy = root.dy * fps * 0.8f;
        motionstack::GraphInputs g;
        g.enabled = true;
        g.velocityWorldX = std::cos(a) * vx - std::sin(a) * vy;
        g.velocityWorldY = std::sin(a) * vx + std::cos(a) * vy;
        off.SetGraphInputs(g);
        on.SetGraphInputs(g);
        ok = off.Step(Pick(clip, t, k == 0, true), dt) && on.Step(Pick(clip, t, k == 0, true), dt);
        if (!ok)
            break;
        Model(db, off.Output(), mOff);
        Model(db, on.Output(), mOn);
        if (k < switchAt)
            minScale = std::fmin(minScale, on.StrideScale());
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 aOff = mOff[size_t(foot[s])].p, aOn = mOn[size_t(foot[s])].p;
            // the warp's own part of the ankle step: change of (warped - unwarped) ankle in one update
            const math::Vec3 dWarp = (aOn - aOff) - (lastOn[s] - lastOff[s]);
            if (k >= switchAt)
            {
                const math::Vec3 dOff = aOff - lastOff[s], dOn = aOn - lastOn[s];
                stepOff = std::fmax(stepOff, std::sqrt(math::Dot(dOff, dOff)));
                stepOn = std::fmax(stepOn, std::sqrt(math::Dot(dOn, dOn)));
                warpStepAfter = std::fmax(warpStepAfter, std::sqrt(math::Dot(dWarp, dWarp)));
            }
            else if (k > 0)
                warpStepBefore = std::fmax(warpStepBefore, std::sqrt(math::Dot(dWarp, dWarp)));
            lastOff[s] = aOff;
            lastOn[s] = aOn;
        }
    }
    std::printf("INFO direction swing %.0f deg: largest ankle step after the swing off %.2f mm, warped %.2f mm, scale before %.3f, warp part of the step "
                "before %.2f after %.2f mm @Hz=%d\n",
                turn * 57.2957795f, stepOff * 1e3f, stepOn * 1e3f, minScale, warpStepBefore * 1e3f, warpStepAfter * 1e3f, hz);
    Check(ok && minScale < 0.9f, "direction swing fixture: warped loop before the swing", hz);
    Check(stepOn <= stepOff + 0.01f, "a 90 degree capsule swing under a warped loop: ankle step within 1 cm of the unwarped one", hz);
    // the warp's own part of the step may grow by at most 3 cm over its largest gait value before the swing (the warped root direction
    // grew it by 9-14 cm at every rate)
    Check(warpStepAfter <= warpStepBefore + 0.03f, "a 90 degree capsule swing: the warp part of the ankle step stays within 3 cm of the gait", hz);
}

int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    for (int hz : {30, 60, 144})
        Solver(hz);
    for (int hz : {30, 60, 144})
        ClipFactorSolver(hz);
    for (int hz : {30, 60, 144})
        YawFadeSolver(hz);
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
        {
            Stack(db, clip, hz);
            StackFadeOut(db, clip, hz, 0.2f);
            StackFadeOut(db, clip, hz, 0.05f);
            StackCrouchModes(db, clip, hz);
            StackCrouchYawSwing(db, clip, hz);
            StackDirectionSwing(db, clip, hz, 1.5707963f);
            StackDirectionSwing(db, clip, hz, -1.5707963f);
        }
    for (int hz : {30, 60, 144})
        StackClipFactor(db, hz);
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}

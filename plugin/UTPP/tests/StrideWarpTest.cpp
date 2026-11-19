// Shrink-only stride warping (mm/StrideWarp.hpp) and its use in the blend stack (BlendStack::SetStrideWarp), 30 / 60 / 144 Hz.
// Solver: the scale never exceeds 1 (no lengthening) and never drops below the floor, follows the capsule / root ratio, is off below the
// minimum root speed and outside ground locomotion, alpha 0 is the identity, the interpolation is continuous and frame-rate independent,
// and a planted foot moving backwards at the clip's root speed moves at capsule speed after the warp.
// Blend stack (actual data, a run loop): a capsule slower than the clip shortens the ankle-to-thigh offset along the root direction by the
// scale and never lengthens it, the ankle height and the frame-to-frame ankle motion stay continuous, and a capsule at or above clip
// speed, a non-ground pick, a clip that is not a loop or the warp switched off leave the pose bit for bit unchanged.
// Clip factor: the scale divides by the clip's planted-foot speed over its root speed (measured per loop clip at Init, clamped, 1 for a
// clip that is not a loop or has too few planted samples), so a crouch loop whose planted feet travel slower than its root shortens less.
// Knee limit (BlendStack::SetStrideKneeLimit, stridewarp::KneeLimitedScale): a leg is shortened by at least its scale's limit but never
// so far that its knee folds beyond max(flexion before + margin, free flexion); with the limit off every leg takes the layer's scale.
// Release of the knee limit (BlendStack::SetStrideLegRelease, stridewarp::LimitLegRelease): a leg's scale falls at most at the release
// rate, so the shortening the knee limit lets go of is spread over updates; the limit itself (a rising scale) is never delayed.
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
    float overShrink = 0;   // m, largest shortening along the root direction beyond the layer's scale
    float kneeExcess = -1e9f; // rad, largest knee flexion above max(flexion unwarped + margin, free flexion)
    float kneeGain = 0;     // rad, largest knee flexion gain over the unwarped pose
    float shortStep = 0;    // m, largest frame-to-frame change of a leg's shortening along the root direction
};
static float KneeFlex(const std::vector<BoneTransform> &m, int thigh, int knee, int ankle)
{
    const math::Vec3 u = m[size_t(knee)].p - m[size_t(thigh)].p, v = m[size_t(ankle)].p - m[size_t(knee)].p;
    const float c = math::Dot(u, v) / std::fmax(std::sqrt(math::Dot(u, u) * math::Dot(v, v)), 1e-12f);
    return std::acos(std::clamp(c, -1.0f, 1.0f));
}
// crouchMode >= 0 sets the crouched loop mode of the warped stack (SetStrideOptions, no yaw fade, whole-layer fade); -1 keeps the default.
// kneeLimit: 1 / 0 switches the knee limit of the warped stack on / off (default margin and free flexion); -1 keeps the default.
static RunStats Run(const motion::MotionDatabase &db, int clip, int hz, float ratio, bool ground, bool enabled, int crouchMode = -1,
                    int kneeLimit = -1, int legRelease = -1)
{
    RunStats r;
    const float dt = 1.0f / float(hz), fps = db.Header().fps;
    motionstack::BlendStack off, on;
    r.ok = off.Init(db) && on.Init(db);
    off.SetStrideWarp(false);
    on.SetStrideWarp(enabled);
    if (crouchMode >= 0)
        on.SetStrideOptions(false, crouchMode, true);
    if (kneeLimit >= 0)
        on.SetStrideKneeLimit(kneeLimit != 0);
    if (legRelease >= 0)
        on.SetStrideLegRelease(legRelease != 0);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")}, thigh[2]{Bone(db, "LeftUpLeg"), Bone(db, "RightUpLeg")};
    const int knee[2]{Bone(db, "LeftLeg"), Bone(db, "RightLeg")};
    const auto &c = db.Clip(uint32_t(clip));
    std::vector<BoneTransform> mOff, mOn;
    math::Vec3 lastOff[2]{}, lastOn[2]{};
    float lastShort[2]{};
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
            r.overShrink = std::fmax(r.overShrink, r.scale * std::fabs(alongOff) - std::fabs(alongOn));
            if (knee[s] >= 0)
            {
                const float fOff = KneeFlex(mOff, thigh[s], knee[s], foot[s]), fOn = KneeFlex(mOn, thigh[s], knee[s], foot[s]);
                r.kneeGain = std::fmax(r.kneeGain, fOn - fOff);
                r.kneeExcess = std::fmax(r.kneeExcess, fOn - std::fmax(fOff + stridewarp::kKneeMargin, stridewarp::kKneeFreeFlexion));
            }
            r.lengthened = std::fmax(r.lengthened, std::fabs(alongOn) - std::fabs(alongOff));
            if (k > 0)
                r.shortStep = std::fmax(r.shortStep, std::fabs((alongOff - alongOn) - lastShort[s]));
            lastShort[s] = alongOff - alongOn;
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
    // The exact scaling of every leg, with the knee limit off; the default (limit on) is checked below.
    const RunStats warped = Run(db, clip, hz, 0.8f, true, true, -1, 0);
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
    const RunStats limited = Run(db, clip, hz, 0.8f, true, true, -1, 1);
    std::printf("INFO knee limit: largest knee flexion gain off %.2f deg, on %.2f deg, above the limit %.3f deg, shrink beyond the scale "
                "%.2f mm, lengthened %.2f mm, height %.2f mm, frame jump off %.2f on %.2f mm @Hz=%d\n",
                warped.kneeGain * 57.2957795f, limited.kneeGain * 57.2957795f, limited.kneeExcess * 57.2957795f, limited.overShrink * 1e3f,
                limited.lengthened * 1e3f, limited.heightError * 1e3f, limited.jumpOff * 1e3f, limited.jumpOn * 1e3f, hz);
    Check(warped.kneeGain > 0.0873f, "knee limit fixture: the unlimited warp folds a run loop knee by more than 5 degrees", hz);
    Check(limited.ok && limited.kneeExcess < 0.0087f,
          "knee limit on: no knee folds above max(unwarped + margin, free flexion) (within 0.5 deg)", hz);
    Check(limited.overShrink < 0.001f, "knee limit on: no leg is shortened beyond the layer's scale", hz);
    // The limit releases the trailing leg's shortening while that knee folds: at its fastest (mid swing) the ankle step grows by a few
    // percent over the unwarped one (2.7 mm of 44.5 mm at 144 Hz), the planted leg is not involved.
    Check(limited.lengthened < 0.001f && limited.heightError < 0.005f && limited.jumpOn <= 1.08f * limited.jumpOff,
          "knee limit on: no lengthening, ankle height kept, largest ankle frame step within 8 % of the unwarped one", hz);
    // The knee limit lets go of a swinging leg's shortening within a few degrees of its unfolding; the release limit spreads that over
    // updates (stridewarp::LimitLegRelease): the shortening never steps more than with the release taken at once.
    const RunStats sudden = Run(db, clip, hz, 0.8f, true, true, -1, 1, 0);
    std::printf("INFO knee limit release: largest frame step of a leg's shortening, released at once %.2f mm, rate-limited %.2f mm @Hz=%d\n",
                sudden.shortStep * 1e3f, limited.shortStep * 1e3f, hz);
    Check(sudden.ok && limited.shortStep <= sudden.shortStep + 1e-4f,
          "knee limit release: a leg's shortening steps no more per update than with the release taken at once", hz);
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

// Runtime switch (BlendStack::SetStrideWarp under live layers, the menu option): `tog` runs a warped loop like `cont` (on from the
// start), is switched off at 0.4 s and on again at 1.6 s; `off` never warps. Switched off, the warp part of the ankles (tog - off) is
// released no faster than the interpolation rate and then the pose is bit for bit the unwarped one; switched on, it grows from 0 no
// faster than the interpolation rate and reaches the continuously warped pose. Before the first switch `tog` is bit for bit `cont`.
static void StackToggle(const motion::MotionDatabase &db, int clip, int hz)
{
    const float dt = 1.0f / float(hz), fps = db.Header().fps, rate = 1.0f - std::exp(-stridewarp::kInterpSpeed * dt);
    motionstack::BlendStack off, cont, tog;
    bool ok = off.Init(db) && cont.Init(db) && tog.Init(db);
    off.SetStrideWarp(false);
    cont.SetStrideWarp(true);
    tog.SetStrideWarp(true);
    const int foot[2]{Bone(db, "LeftFoot"), Bone(db, "RightFoot")};
    const auto &c = db.Clip(uint32_t(clip));
    const float duration = float(c.numFrames - 1) / fps;
    std::vector<BoneTransform> mOff, mCont, mTog;
    const int offAt = int(0.4f * float(hz)), onAt = int(1.6f * float(hz)), end = int(2.8f * float(hz));
    float time = 0.5f, shortening = 0, dropExcess = -1, growExcess = -1, endGap = 0, lastF[2]{}, lastW[2]{};
    bool sameBefore = true, unwarpedAfter = true, stateOk = true;
    for (int k = 0; k < end && ok; ++k, time += dt)
    {
        const float t = std::fmod(time, duration);
        const auto &root = db.Root(c.firstFrame + std::min(uint32_t(t * fps), c.numFrames - 1));
        motionstack::GraphInputs g;
        g.enabled = true;
        g.velocityWorldX = root.dx * fps * 0.8f;
        g.velocityWorldY = root.dy * fps * 0.8f;
        off.SetGraphInputs(g);
        cont.SetGraphInputs(g);
        tog.SetGraphInputs(g);
        if (k == offAt)
            tog.SetStrideWarp(false);
        if (k == onAt)
            tog.SetStrideWarp(true);
        stateOk = stateOk && tog.StrideWarp() == (k < offAt || k >= onAt) && tog.StrideKneeLimit();
        ok = off.Step(Pick(clip, t, k == 0, true), dt) && cont.Step(Pick(clip, t, k == 0, true), dt) &&
             tog.Step(Pick(clip, t, k == 0, true), dt);
        if (!ok)
            break;
        const size_t bytes = sizeof(BoneTransform) * db.NumBones();
        if (k < offAt)
            sameBefore = sameBefore && std::memcmp(tog.Output(), cont.Output(), bytes) == 0;
        if (k >= onAt - int(0.2f * float(hz)) && k < onAt) // 1 s after the switch: the held shortening is long released
            unwarpedAfter = unwarpedAfter && std::memcmp(tog.Output(), off.Output(), bytes) == 0;
        Model(db, off.Output(), mOff);
        Model(db, cont.Output(), mCont);
        Model(db, tog.Output(), mTog);
        for (int s = 0; s < 2; ++s)
        {
            const math::Vec3 warp = mCont[size_t(foot[s])].p - mOff[size_t(foot[s])].p;
            const math::Vec3 part = mTog[size_t(foot[s])].p - mOff[size_t(foot[s])].p;
            const float w = std::sqrt(math::Dot(warp, warp)), f = std::sqrt(math::Dot(part, part));
            shortening = std::fmax(shortening, w);
            // one update moves the warp part by at most the interpolation share (of the held part switched off, of the continuing
            // warp switched on), plus the change of the continuing warp and 2 mm for the pose difference
            if (k >= offAt && k < onAt)
                dropExcess = std::fmax(dropExcess, (lastF[s] - f) - (rate * lastF[s] + std::fabs(w - lastW[s]) + 0.002f));
            if (k >= onAt)
                growExcess = std::fmax(growExcess, (f - lastF[s]) - (rate * w + std::fabs(w - lastW[s]) + 0.002f));
            if (k == end - 1)
            {
                const math::Vec3 gap = mTog[size_t(foot[s])].p - mCont[size_t(foot[s])].p;
                endGap = std::fmax(endGap, std::sqrt(math::Dot(gap, gap)));
            }
            lastF[s] = f;
            lastW[s] = w;
        }
    }
    std::printf("INFO runtime switch: shortening up to %.2f cm, drop excess %.2f mm, growth excess %.2f mm, gap to the continuous warp "
                "%.3f mm @Hz=%d\n",
                shortening * 100.0f, dropExcess * 1e3f, growExcess * 1e3f, endGap * 1e3f, hz);
    Check(ok && shortening > 0.02f, "runtime switch fixture: warped loop with a shortening above 2 cm", hz);
    Check(sameBefore, "runtime switch: before the first switch the pose is bit for bit the continuously warped one", hz);
    Check(stateOk, "runtime switch: StrideWarp follows the switch, the knee limit stays on", hz);
    Check(dropExcess <= 0.0f, "switched off under a warped loop: the shortening is released no faster than the interpolation rate", hz);
    Check(unwarpedAfter, "switched off: once released, the pose is bit for bit the unwarped one", hz);
    Check(growExcess <= 0.0f, "switched on under a running loop: the shortening grows no faster than the interpolation rate", hz);
    Check(endGap < 0.0005f, "switched on: the pose reaches the continuously warped one (within 0.5 mm after 1.2 s)", hz);
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
    const RunStats r = Run(db, crouch, hz, 0.8f, true, true, 0, 0); // crouched loops warped like the others (exact scale: knee limit off)
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

// KneeLimitedScale on a 45 / 45 cm leg: the warped hip-to-ankle distance (d^2 - along^2 (1 - s^2)) keeps the flexion within the limit,
// a leg below the free flexion takes the layer's scale, the result never lowers the scale and never exceeds 1.
static void KneeSolver(int hz)
{
    const float a = 0.45f, b = 0.45f;
    bool bounded = true, within = true, free = true;
    for (float flexDeg = 0; flexDeg <= 160; flexDeg += 5)
        for (float along = -0.4f; along <= 0.4f; along += 0.05f)
            for (float scale : {0.75f, 0.85f, 0.95f})
            {
                const float flex = flexDeg * 0.017453293f;
                const float d = std::sqrt(a * a + b * b + 2 * a * b * std::cos(flex));
                if (std::fabs(along) > d)
                    continue;
                const float s = stridewarp::KneeLimitedScale(scale, along, d, a, b);
                bounded = bounded && s >= scale && s <= 1.0f;
                const float d1 = std::sqrt(std::fmax(d * d - along * along * (1 - s * s), 0.0f));
                const float f1 = stridewarp::KneeFlexion(d1, a, b);
                within = within && f1 <= std::fmax(flex + stridewarp::kKneeMargin, stridewarp::kKneeFreeFlexion) + 1e-3f;
                const float fFull = stridewarp::KneeFlexion(std::sqrt(std::fmax(d * d - along * along * (1 - scale * scale), 0.0f)), a, b);
                if (fFull <= stridewarp::kKneeFreeFlexion - 1e-3f)
                    free = free && s == scale;
            }
    Check(bounded, "knee limit: scale stays within [layer scale, 1]", hz);
    Check(within, "knee limit: warped flexion within max(flexion + margin, free flexion)", hz);
    Check(free, "knee limit: a leg that stays below the free flexion takes the layer's scale", hz);
    Check(stridewarp::KneeLimitedScale(0.8f, 0.0f, 0.5f, a, b) == 0.8f && stridewarp::KneeLimitedScale(1.0f, 0.3f, 0.5f, a, b) == 1.0f,
          "knee limit: no stride offset or no warp returns the scale unchanged", hz);
}

// Release of the knee limit (stridewarp::LimitLegRelease): a leg scale falls at most kLegReleaseRate per second, rises at once, holds at
// dt <= 0, and the full 1 -> floor release takes the same time at every frame rate.
static void LegReleaseSolver(int hz)
{
    const float dt = 1.0f / float(hz);
    bool fall = true, rise = true;
    for (float prev = 0.75f; prev <= 1.0f; prev += 0.05f)
        for (float target = 0.75f; target <= 1.0f; target += 0.05f)
        {
            const float s = stridewarp::LimitLegRelease(prev, target, dt);
            fall = fall && s >= target && s >= prev - stridewarp::kLegReleaseRate * dt - 1e-6f && s <= std::fmax(prev, target) + 1e-6f;
            if (target >= prev)
                rise = rise && s == target;
        }
    Check(fall, "leg release: the scale falls at most the release rate per second and never below the knee-limited target", hz);
    Check(rise, "leg release: a rising scale (the knee limit taking over) is followed at once", hz);
    Check(stridewarp::LimitLegRelease(0.9f, 0.75f, 0.0f) == 0.9f && stridewarp::LimitLegRelease(0.9f, std::nanf(""), dt) == 1.0f,
          "leg release: dt <= 0 holds the scale, a non-finite target is no warp", hz);
    float s = 1.0f, time = 0.0f;
    while (s > stridewarp::kScaleFloor && time < 1.0f)
    {
        s = stridewarp::LimitLegRelease(s, stridewarp::kScaleFloor, dt);
        time += dt;
    }
    Check(time <= (1.0f - stridewarp::kScaleFloor) / stridewarp::kLegReleaseRate + dt + 1e-4f,
          "leg release: the full release reaches the floor within 1 / rate x the range (plus one update)", hz);
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
    KneeSolver(0);
    for (int hz : {30, 60, 144})
        LegReleaseSolver(hz);
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
            StackToggle(db, clip, hz);
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

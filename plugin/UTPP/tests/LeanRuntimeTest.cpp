// Offline test of the motion lean layer runtime (src/mm/LeanRuntime.hpp on the pack of src/mm/LeanPack.hpp).
//
//   utpp_leanruntime_test <utpp_lean_woman.bin> <utpp_lean_man.bin> [<utpp_overlays_woman.utppov> <utpp_overlays_man.utppov>]
//
// Data driven (the packs and overlays are motion derived, outside git; a missing pack makes the test FAIL):
//   * estimator: the stepped velocity of the real feed (updates every 2-3 frames) gives the exact mean lateral acceleration after the
//     low pass (circle: lat = a / 800 cm/s^2, left turn negative, right positive), straight running stays near 0, no NaN, no jumps,
//     gap / invalid velocity reset, the raw ABP formula is bimodal (documents why the low pass exists);
//   * polarity: sample x = +1 of the run / walk blend space leans the Hips -> Neck line to the RIGHT (+x of the model space), x = -1 left;
//   * gates: every blocker fades the weight out in 0.3 s (linear) and ends Idle (Apply never changes the pose), speed ramp, scale;
//   * Disable_AdditiveLeans curve mutes the lean (interp 5/s) and a non-mapped clip does not;
//   * Apply: circle at the real speed leans into the turn by the order of 10 deg, scale 0.5 about half, walk/run crossfade, head layer
//     only when asked, Evaluate failure = no lean, non-finite input harmless, no allocation (stack/member only: checked by the type);
//   * pack vs overlay identity check (names + parents), clip map by name + frame count, file size cap, rig identity constants.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../src/mm/LeanRuntime.hpp"

using namespace utpp::mm;
namespace ln = utpp::mm::lean;
namespace lr = utpp::mm::leanrt;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}
static bool Near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }
static constexpr float kPi = 3.14159265358979f;

static std::vector<uint8_t> ReadAll(const std::string& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

// ---- feed simulation: velocity that updates every 2-3 frames (the real 'velWorld' of the feed) ----------------------------------
struct Sim
{
    float theta = 0.0f; // heading rad (CCW from +x), the true motion
    float speed = 3.85f;
    float omega = 0.0f; // rad/s, > 0 = left turn
    float vx = 0.0f, vy = 0.0f;
    int hold = 0, pattern = 0;
    float headNoise = 0.0f, speedNoise = 0.0f;
    unsigned rng = 12345u;
    float Rand() // -1 .. 1
    {
        rng = rng * 1664525u + 1013904223u;
        return static_cast<float>((rng >> 8) & 0xFFFF) / 32768.0f - 1.0f;
    }
    // advances one frame of dt and returns the (held) feed velocity
    void Tick(float aDt)
    {
        theta += omega * aDt;
        if (hold <= 0)
        {
            const float h = theta + headNoise * Rand();
            const float s = speed + speedNoise * Rand();
            vx = s * std::cos(h);
            vy = s * std::sin(h);
            hold = (pattern++ % 2) ? 3 : 2;
        }
        --hold;
    }
};

static lr::Signals Moving(float vx, float vy)
{
    lr::Signals s;
    s.vx = vx;
    s.vy = vy;
    s.velValid = true;
    s.moving = true;
    return s;
}

// FK lean of the Hips -> Neck line in the model space (+x right, +y forward, +z up): atan2(dx, dz) in degrees (+ = to the right)
static float NeckLeanDeg(const ln::LeanPack& aPack, const ov::Pose& aPose, int aHips, int aNeck, float* aYawDeg = nullptr)
{
    ov::Vec3 pos[ov::kMaxBones];
    ov::Quat rot[ov::kMaxBones];
    ov::MeshPositions(aPack.Skel(), aPose, pos, rot);
    const ov::Vec3 d = pos[aNeck] - pos[aHips];
    (void)aYawDeg;
    return std::atan2(d.x, d.z) * 180.0f / kPi;
}

static int BoneIndex(const ln::LeanPack& aPack, std::string_view aName)
{
    for (int i = 0; i < ln::kNumBones; ++i)
        if (aPack.BoneName(i) == aName)
            return i;
    return -1;
}

static void BasePose(const ov::OverlayFile& aOv, ov::Pose& aOut) { aOv.FramePose(aOv.PoseAt(0).firstFrame, aOut); }

static bool SamePose(const ov::Pose& a, const ov::Pose& b, int aN)
{
    return std::memcmp(a.b, b.b, sizeof(ov::Xf) * static_cast<size_t>(aN)) == 0;
}

// ---- estimator ----------------------------------------------------------------------------------------------------------------------
static void TestEstimator(const ln::LeanPack& aPack)
{
    lr::Params prm;
    // circle at 3.85 m/s, omega 1.5 rad/s: a = 5.775 m/s^2 -> lat = 577.5 / 800 = 0.7219; left turn = negative
    for (const float sgn : {1.0f, -1.0f})
    {
        lr::LeanRuntime rt;
        Check(rt.Bind(aPack), "runtime binds the pack");
        Sim sim;
        sim.omega = 1.5f * sgn;
        float minLat = 9, maxLat = -9, prev = 0.0f, maxJump = 0.0f;
        bool nan = false;
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 360; ++i)
        {
            sim.Tick(dt);
            rt.Step(aPack, prm, Moving(sim.vx, sim.vy), dt);
            const float l = rt.Tel().lat;
            nan = nan || !std::isfinite(l) || !std::isfinite(rt.Tel().aLat);
            if (i >= 120)
            {
                minLat = std::min(minLat, l);
                maxLat = std::max(maxLat, l);
                maxJump = std::max(maxJump, std::fabs(l - prev));
            }
            prev = l;
        }
        const float want = -sgn * 0.7219f;
        Check(!nan, "circle: no NaN");
        Check(Near(0.5f * (minLat + maxLat), want, 0.05f), "circle sgn " + std::to_string(sgn) + ": mean lat " + std::to_string(0.5f * (minLat + maxLat)) + " ~ " + std::to_string(want));
        Check(maxLat - minLat < 0.12f, "circle: low passed lat is smooth, ripple " + std::to_string(maxLat - minLat));
        Check(maxJump < 0.02f, "circle: no frame jump " + std::to_string(maxJump));
        Check((sgn > 0) == (rt.Tel().lat < 0.0f), "left turn (omega > 0) gives negative lat (acceleration to the left)");
    }
    // the raw ABP formula on the stepped feed is bimodal (why the low pass exists): count spikes
    {
        Sim sim;
        sim.omega = 1.5f;
        float px = 0, py = 0;
        int zeros = 0, spikes = 0, n = 0;
        bool first = true;
        for (int i = 0; i < 300; ++i)
        {
            sim.Tick(1.0f / 60.0f);
            if (!first)
            {
                const float ax = (sim.vx - px) * 60.0f, ay = (sim.vy - py) * 60.0f;
                const float sp = std::hypot(sim.vx, sim.vy);
                const float l = ln::LateralAccelAmount(aPack.P(), sim.vx, sim.vy, ax, ay);
                (void)sp;
                zeros += std::fabs(l) < 1e-4f;
                spikes += std::fabs(l) >= 0.99f;
                ++n;
            }
            first = false;
            px = sim.vx;
            py = sim.vy;
        }
        Check(zeros > n / 3 && spikes > n / 4, "raw per-frame lat of the stepped feed is bimodal (zeros " + std::to_string(zeros) + ", spikes " + std::to_string(spikes) + ")");
    }
    // full circle (170 deg/s as in the 23:51 capture) saturates; straight running stays below 0.03
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 2.967f;
        float mean = 0;
        int n = 0;
        for (int i = 0; i < 360; ++i)
        {
            sim.Tick(1.0f / 60.0f);
            rt.Step(aPack, prm, Moving(sim.vx, sim.vy), 1.0f / 60.0f);
            if (i >= 120)
            {
                mean += rt.Tel().lat;
                ++n;
            }
        }
        mean /= static_cast<float>(n);
        Check(mean < -0.9f && mean >= -1.0f, "170 deg/s circle: lat saturates ~ -1 (" + std::to_string(mean) + ")");
    }
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 0.0f;
        sim.headNoise = 0.006f; // 0.35 deg
        sim.speedNoise = 0.03f;
        float amax = 0, amean = 0;
        int n = 0;
        for (int i = 0; i < 600; ++i)
        {
            sim.Tick(1.0f / 60.0f);
            rt.Step(aPack, prm, Moving(sim.vx, sim.vy), 1.0f / 60.0f);
            if (i >= 60)
            {
                amax = std::max(amax, std::fabs(rt.Tel().lat));
                amean += std::fabs(rt.Tel().lat);
                ++n;
            }
        }
        Check(amax < 0.08f && amean / static_cast<float>(n) < 0.03f,
              "straight run with noise: |lat| max " + std::to_string(amax) + " mean " + std::to_string(amean / static_cast<float>(n)));
    }
    // invalid velocity / gap / NaN reset the estimator, never produce NaN
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 1.5f;
        for (int i = 0; i < 120; ++i)
        {
            sim.Tick(1.0f / 60.0f);
            rt.Step(aPack, prm, Moving(sim.vx, sim.vy), 1.0f / 60.0f);
        }
        lr::Signals bad = Moving(NAN, 1.0f);
        rt.Step(aPack, prm, bad, 1.0f / 60.0f);
        Check(rt.Tel().aLat == 0.0f && std::isfinite(rt.Tel().lat), "NaN velocity resets the estimator");
        lr::Signals inf = Moving(INFINITY, 0.0f);
        rt.Step(aPack, prm, inf, NAN);
        Check(std::isfinite(rt.Tel().lat) && std::isfinite(rt.Tel().weight), "inf velocity / NaN dt are harmless");
        sim.Tick(1.0f / 60.0f);
        rt.Step(aPack, prm, Moving(sim.vx, sim.vy), 0.5f); // gap > dtMax
        Check(rt.Tel().aLat == 0.0f, "a feed gap resets the estimator");
        // first feed after Reset: no acceleration from a stale velocity
        rt.Reset();
        rt.Step(aPack, prm, Moving(3.0f, 0.0f), 1.0f / 60.0f);
        Check(rt.Tel().aLat == 0.0f, "first feed has no previous velocity");
    }
}

// ---- polarity of the pack samples ---------------------------------------------------------------------------------------------------
static void TestPolarity(const ln::LeanPack& aPack, const ov::OverlayFile* aOv)
{
    if (!aOv)
        return;
    ov::Pose base;
    BasePose(*aOv, base);
    const int hips = BoneIndex(aPack, "Hips"), neck = BoneIndex(aPack, "Neck");
    Check(hips >= 0 && neck >= 0, "Hips / Neck exist in the pack");
    if (hips < 0 || neck < 0)
        return;
    const float lean0 = NeckLeanDeg(aPack, base, hips, neck);
    for (const char* id : {"run", "walk"})
    {
        const int L = aPack.FindLayer(id);
        Check(L >= 0, std::string("layer ") + id);
        if (L < 0)
            continue;
        for (int k = 0; k < aPack.LayerAt(L).numSamples; ++k)
        {
            const ln::SampleRec& s = aPack.Sample(L, k);
            if (s.y != 0.0f || (s.x != 1.0f && s.x != -1.0f))
                continue;
            ov::Additive add;
            aPack.SampleAdditive(L, k, add);
            ov::Pose p = base;
            ov::ApplyAdditive(aPack.Skel(), p, add, 1.0f);
            const float d = NeckLeanDeg(aPack, p, hips, neck) - lean0;
            Check(s.x * d > 4.0f, std::string(id) + " sample x=" + std::to_string(s.x) + " leans the Hips->Neck line to the " + (s.x > 0 ? "right" : "left") + " (" + std::to_string(d) + " deg)");
        }
    }
}

// ---- gates, fades, idle -------------------------------------------------------------------------------------------------------------
static void RunCircle(lr::LeanRuntime& rt, const ln::LeanPack& aPack, const lr::Params& prm, Sim& sim, int aFrames, lr::Signals aProto)
{
    for (int i = 0; i < aFrames; ++i)
    {
        sim.Tick(1.0f / 60.0f);
        lr::Signals s = aProto;
        s.vx = sim.vx;
        s.vy = sim.vy;
        rt.Step(aPack, prm, s, 1.0f / 60.0f);
    }
}

static void TestGates(const ln::LeanPack& aPack, const ov::OverlayFile* aOv)
{
    lr::Params prm;
    // each blocker: fades out linearly in 0.3 s, then Idle
    struct Case
    {
        const char* name;
        void (*set)(lr::Signals&);
    };
    const Case cases[] = {{"blocked", [](lr::Signals& s) { s.blocked = true; }},   {"not moving", [](lr::Signals& s) { s.moving = false; }},
                          {"crouch", [](lr::Signals& s) { s.crouch = true; }},     {"air", [](lr::Signals& s) { s.air = true; }},
                          {"aiming", [](lr::Signals& s) { s.aiming = true; }},     {"weapon busy", [](lr::Signals& s) { s.weaponBusy = true; }},
                          {"slide", [](lr::Signals& s) { s.slide = true; }},       {"invalid velocity", [](lr::Signals& s) { s.velValid = false; }}};
    for (const Case& c : cases)
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 1.5f;
        RunCircle(rt, aPack, prm, sim, 240, Moving(0, 0));
        const float w0 = rt.Weight();
        Check(w0 > 0.99f && !rt.Idle(), std::string(c.name) + ": lean fully on before the blocker (" + std::to_string(w0) + ")");
        lr::Signals blk = Moving(0, 0);
        c.set(blk);
        RunCircle(rt, aPack, prm, sim, 9, blk); // 9 frames = 0.15 s
        Check(Near(rt.Weight(), 0.5f, 0.06f), std::string(c.name) + ": half faded after 0.15 s (" + std::to_string(rt.Weight()) + ")");
        RunCircle(rt, aPack, prm, sim, 12, blk);
        Check(rt.Weight() == 0.0f && rt.Idle(), std::string(c.name) + ": zero + Idle after 0.35 s");
        if (aOv)
        {
            ov::Pose a, b;
            BasePose(*aOv, a);
            b = a;
            const bool applied = rt.Apply(aPack, prm, b);
            Check(!applied && SamePose(a, b, ln::kNumBones), std::string(c.name) + ": Apply with weight 0 leaves the pose bit-identical");
        }
    }
    // idle from the start: never active
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Check(rt.Idle() && rt.Weight() == 0.0f, "fresh runtime is idle");
        lr::Signals still;
        for (int i = 0; i < 100; ++i)
            rt.Step(aPack, prm, still, 1.0f / 60.0f);
        Check(rt.Idle(), "standing still stays idle");
    }
    // speed ramp: 0.8 .. 1.6 m/s -> lat scale; below 0.8 no lean input at all
    {
        for (const auto& c : {std::pair<float, float>{0.7f, 0.0f}, std::pair<float, float>{1.2f, 0.5f}, std::pair<float, float>{2.0f, 1.0f}})
        {
            lr::LeanRuntime rt;
            rt.Bind(aPack);
            Sim sim;
            sim.speed = c.first;
            sim.omega = 8.0f / c.first; // a = v * omega = 8 m/s^2: lat saturates at every speed (denominator <= 800)
            RunCircle(rt, aPack, prm, sim, 240, Moving(0, 0));
            Check(Near(std::fabs(rt.Tel().lat), c.second, 0.08f), "speed ramp at " + std::to_string(c.first) + " m/s: |lat| " + std::to_string(std::fabs(rt.Tel().lat)) + " ~ " + std::to_string(c.second));
        }
    }
    // head gate only with the head request, never without a head layer in use
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 1.5f;
        RunCircle(rt, aPack, prm, sim, 240, Moving(0, 0));
        Check(!rt.HeadActive() && rt.HeadWeight() == 0.0f, "head lean off unless requested");
        lr::Signals h = Moving(0, 0);
        h.head = true;
        RunCircle(rt, aPack, prm, sim, 30, h);
        Check(rt.HeadActive() && rt.HeadWeight() > 0.99f, "head lean on when requested");
        RunCircle(rt, aPack, prm, sim, 30, Moving(0, 0));
        Check(!rt.HeadActive(), "head lean fades out when no longer requested");
    }
}

// ---- Disable_AdditiveLeans -----------------------------------------------------------------------------------------------------------
static void TestCurve(const ln::LeanPack& aPack)
{
    lr::Params prm;
    // find a curve clip and a time where the value is ~1 (muted) and one where it is ~0
    int clip = -1;
    float tHi = -1, tLo = -1;
    for (int c = 0; c < aPack.NumClips() && clip < 0; ++c)
    {
        const ln::ClipRec& r = aPack.ClipAt(c);
        float hi = -1, lo = -1;
        for (int f = 0; f < r.count; ++f)
        {
            const float t = static_cast<float>(f) / r.fps, v = aPack.DisableCurveAt(c, t);
            if (v > 0.99f && hi < 0)
                hi = t;
            if (v < 0.01f && lo < 0)
                lo = t;
        }
        if (hi >= 0 && lo >= 0)
        {
            clip = c;
            tHi = hi;
            tLo = lo;
        }
    }
    Check(clip >= 0, "a curve table with muted and open frames exists");
    if (clip < 0)
        return;
    lr::LeanRuntime rt;
    rt.Bind(aPack);
    Sim sim;
    sim.omega = 1.5f;
    lr::Signals open = Moving(0, 0);
    open.curveClip = clip;
    open.clipSeconds = tLo;
    RunCircle(rt, aPack, prm, sim, 240, open);
    Check(rt.Weight() > 0.99f, "curve open: full weight");
    lr::Signals mute = open;
    mute.clipSeconds = tHi;
    RunCircle(rt, aPack, prm, sim, 6, mute); // 0.1 s: FInterpTo 5/s -> 1 - exp(-0.5) ~ 0.39 muted
    Check(rt.Weight() < 0.8f && rt.Weight() > 0.4f, "curve mutes with interp 5/s (" + std::to_string(rt.Weight()) + ")");
    RunCircle(rt, aPack, prm, sim, 120, mute);
    Check(rt.Weight() < 0.02f, "curve muted: weight ~0");
    lr::Signals none = Moving(0, 0);
    RunCircle(rt, aPack, prm, sim, 240, none);
    Check(rt.Weight() > 0.98f, "unmapped clip (curve -1) re-opens the lean");
    lr::Signals oob = open;
    oob.curveClip = 9999;
    oob.clipSeconds = tHi;
    RunCircle(rt, aPack, prm, sim, 30, oob);
    Check(rt.Weight() > 0.98f, "out of range curve index is ignored");
}

// ---- Apply ---------------------------------------------------------------------------------------------------------------------------
static void TestApply(const ln::LeanPack& aPack, const ov::OverlayFile& aOv)
{
    lr::Params prm;
    ov::Pose base;
    BasePose(aOv, base);
    const int hips = BoneIndex(aPack, "Hips"), neck = BoneIndex(aPack, "Neck");
    const float lean0 = NeckLeanDeg(aPack, base, hips, neck);
    float leanFull[2] = {0, 0};
    for (int side = 0; side < 2; ++side)
    {
        const float sgn = side == 0 ? 1.0f : -1.0f; // omega > 0 = left turn
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 2.967f * sgn;
        RunCircle(rt, aPack, prm, sim, 480, Moving(0, 0));
        ov::Pose p = base;
        const bool applied = rt.Apply(aPack, prm, p);
        const float d = NeckLeanDeg(aPack, p, hips, neck) - lean0;
        leanFull[side] = d;
        Check(applied, "circle: Apply applied");
        Check(d * sgn < -6.0f && d * sgn > -22.0f, std::string(side == 0 ? "left" : "right") + " circle: Hips->Neck leans into the turn by " + std::to_string(d) + " deg (6..22)");
        // scale 0.5
        lr::Params half = prm;
        half.scale = 0.5f;
        ov::Pose ph = base;
        rt.Apply(aPack, half, ph);
        const float dh = NeckLeanDeg(aPack, ph, hips, neck) - lean0;
        Check(std::fabs(dh) > 0.3f * std::fabs(d) && std::fabs(dh) < 0.7f * std::fabs(d), "scale 0.5 leans about half (" + std::to_string(dh) + " vs " + std::to_string(d) + ")");
        lr::Params zero = prm;
        zero.scale = 0.0f;
        ov::Pose pz = base;
        Check(!rt.Apply(aPack, zero, pz) && SamePose(pz, base, ln::kNumBones), "scale 0 applies nothing");
        // no head request: Neck/Head rotations differ from a head request only by the head layer
        lr::Signals h = Moving(0, 0);
        h.head = true;
        RunCircle(rt, aPack, prm, sim, 240, h);
        ov::Pose ph2 = base;
        rt.Apply(aPack, prm, ph2);
        Check(!SamePose(ph2, p, ln::kNumBones), "head layer changes the pose when asked for");
        RunCircle(rt, aPack, prm, sim, 60, Moving(0, 0));
    }
    Check(std::fabs(leanFull[0] + leanFull[1]) < 0.35f * std::fabs(leanFull[0] - leanFull[1]), "left / right lean are mirror images in size (" + std::to_string(leanFull[0]) + " / " + std::to_string(leanFull[1]) + ")");
    // walk -> run crossfade: slow circle uses the walk set only (alpha 0), fast the run set
    {
        auto lean = [&](float aSpeed, float aOmega) {
            lr::LeanRuntime rt;
            rt.Bind(aPack);
            Sim sim;
            sim.speed = aSpeed;
            sim.omega = aOmega;
            RunCircle(rt, aPack, prm, sim, 480, Moving(0, 0));
            return rt.Tel().walkRun;
        };
        Check(lean(1.5f, 4.0f) < 0.01f, "walk speed: crossfade alpha 0");
        Check(lean(3.8f, 2.0f) > 0.99f, "run speed: crossfade alpha 1");
    }
    // foot anchor: the lean moves the ankles by 20+ cm; with the anchor they stay on the unleaned model-space positions
    {
        lr::LeanRuntime rt;
        rt.Bind(aPack);
        Sim sim;
        sim.omega = 2.967f;
        RunCircle(rt, aPack, prm, sim, 480, Moving(0, 0));
        const int feet[2] = {BoneIndex(aPack, "LeftFoot"), BoneIndex(aPack, "RightFoot")};
        const int shins[2] = {BoneIndex(aPack, "LeftLeg"), BoneIndex(aPack, "RightLeg")};
        const int ups[2] = {BoneIndex(aPack, "LeftUpLeg"), BoneIndex(aPack, "RightUpLeg")};
        ov::Vec3 p0[ov::kMaxBones], p1[ov::kMaxBones], p2[ov::kMaxBones];
        ov::Quat r0[ov::kMaxBones], r1[ov::kMaxBones], r2[ov::kMaxBones];
        ov::MeshPositions(aPack.Skel(), base, p0, r0);
        ov::Pose on = base, off = base;
        lr::Params noAnchor = prm;
        noAnchor.anchorFeet = false;
        Check(rt.Apply(aPack, prm, on) && rt.Apply(aPack, noAnchor, off), "anchor / no anchor both apply");
        ov::MeshPositions(aPack.Skel(), on, p1, r1);
        ov::MeshPositions(aPack.Skel(), off, p2, r2);
        const auto len = [](ov::Vec3 a, ov::Vec3 b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); };
        for (int s = 0; s < 2; ++s)
        {
            Check(len(p1[feet[s]], p0[feet[s]]) < 0.01f, "anchor: ankle " + std::to_string(s) + " stays within 1 cm of the unleaned one (" + std::to_string(len(p1[feet[s]], p0[feet[s]])) + " m)");
            Check(len(p2[feet[s]], p0[feet[s]]) > 0.10f, "no anchor: the lean moves the ankle by more than 10 cm (" + std::to_string(len(p2[feet[s]], p0[feet[s]])) + " m)");
            Check(Near(len(p1[ups[s]], p1[shins[s]]), len(p0[ups[s]], p0[shins[s]]), 1e-3f) && Near(len(p1[shins[s]], p1[feet[s]]), len(p0[shins[s]], p0[feet[s]]), 1e-3f),
                  "anchor: thigh and shin lengths unchanged");
            Check(ov::AngleDeg(r1[feet[s]], r0[feet[s]]) < 0.5f, "anchor: foot keeps its unleaned model-space rotation");
        }
        // the upper body and the pelvis are still leaned with the anchor
        Check(std::fabs(NeckLeanDeg(aPack, on, hips, neck) - lean0) > 6.0f, "anchor: the lean itself is kept (" + std::to_string(NeckLeanDeg(aPack, on, hips, neck) - lean0) + " deg)");
    }
    // Evaluate failure of one layer = lean 0 for that layer, never a crash: out of range bind
    {
        lr::LeanRuntime rt;
        ln::LeanPack empty;
        Check(!rt.Bind(empty), "an empty pack cannot be bound");
        ov::Pose p = base, q = base;
        Check(!rt.Apply(empty, prm, p) && SamePose(p, q, ln::kNumBones), "Apply on an unloaded pack is a no-op");
    }
}

// ---- identity ------------------------------------------------------------------------------------------------------------------------------
static void TestIdentity(const ln::LeanPack& aPack, const ov::OverlayFile& aOv, const std::vector<uint8_t>& aBytes, uint64_t aRig)
{
    std::string why;
    Check(lr::PackMatchesOverlay(aPack, aOv, why), "pack bones == overlay bones: " + why);
    Check(aPack.RigHash() == aRig && aOv.RigHash() == aRig, "pack and overlay carry the expected rig identity");
    // mismatch detection: a pack loaded from a mutated copy (bone name) fails the check
    std::vector<uint8_t> m = aBytes;
    // BONE section: header 64 + 8 table entries * 16, first record is the first tag
    size_t boneOff = 0;
    for (int i = 0; i < 8; ++i)
        if (std::memcmp(m.data() + 64 + 16 * i, "BONE", 4) == 0)
            std::memcpy(&boneOff, m.data() + 64 + 16 * i + 4, 4);
    if (boneOff > 0)
    {
        m[boneOff + 36 * 5] = 'Z'; // bone 5 name
        const uint32_t pc = ln::Crc32(m.data() + 64, m.size() - 64);
        std::memcpy(m.data() + 36, &pc, 4);
        uint8_t h[64];
        std::memcpy(h, m.data(), 64);
        std::memset(h + 40, 0, 4);
        const uint32_t hc = ln::Crc32(h, 64);
        std::memcpy(m.data() + 40, &hc, 4);
        ln::LeanPack bad;
        std::string err;
        const bool ok = bad.LoadFromMemory(m, aRig, err);
        Check(ok, "renamed bone still parses (names are not part of the format checks): " + err);
        Check(ok && !lr::PackMatchesOverlay(bad, aOv, why), "renamed bone fails the overlay identity check");
    }
    // clip map by name with frame count verification
    {
        const int nClips = aPack.NumClips();
        std::vector<std::string> names;
        std::vector<uint32_t> frames;
        for (int i = 0; i < nClips; ++i)
        {
            names.push_back(aPack.ClipAt(i).name);
            frames.push_back(static_cast<uint32_t>(aPack.ClipAt(i).count));
        }
        names.push_back("some_other_clip");
        frames.push_back(10);
        frames[1] += 1; // numFrames mismatch -> that clip stays unmuted
        std::vector<int32_t> map;
        int mismatch = 0;
        const int mapped = lr::BuildClipMap(
            aPack, static_cast<uint32_t>(names.size()), [&](uint32_t c) { return std::string_view(names[c]); },
            [&](uint32_t c) { return frames[c]; }, map, mismatch);
        Check(mapped == nClips - 1 && mismatch == 1 && map[1] == -1 && map[0] == 0 && map[static_cast<size_t>(nClips)] == -1,
              "clip map: names map, frame count mismatch -> unmapped (" + std::to_string(mapped) + "/" + std::to_string(nClips) + ")");
    }
    // file size cap
    {
        std::vector<uint8_t> big(static_cast<size_t>(ln::kMaxFileBytes) + 1, 0);
        ln::LeanPack p;
        std::string err;
        Check(!p.LoadFromMemory(big, 0, err) && err == "file too large", "oversize pack rejected before parsing: " + err);
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_leanruntime_test <utpp_lean_woman.bin> <utpp_lean_man.bin> [<overlays_woman.utppov> <overlays_man.utppov>]\n");
        return 2;
    }
    const uint64_t rigs[2] = {ln::kRigIdentityWoman, ln::kRigIdentityMan};
    for (int body = 0; body < 2; ++body)
    {
        const std::vector<uint8_t> bytes = ReadAll(argv[1 + body]);
        Check(!bytes.empty(), std::string("pack readable: ") + argv[1 + body]);
        ln::LeanPack pack;
        std::string err;
        Check(pack.LoadFromMemory(bytes, rigs[body], err), "pack loads: " + err);
        if (!pack.Ok())
            continue;
        ov::OverlayFile ovf;
        const bool haveOv = argc >= 5 && ovf.LoadFromFile(argv[3 + body], rigs[body], err);
        Check(argc < 5 || haveOv, std::string("overlay loads: ") + err);
        if (body == 0)
        {
            TestEstimator(pack);
            TestGates(pack, haveOv ? &ovf : nullptr);
            TestCurve(pack);
        }
        if (haveOv)
        {
            TestPolarity(pack, &ovf);
            if (body == 0)
                TestApply(pack, ovf);
            TestIdentity(pack, ovf, bytes, rigs[body]);
        }
    }
    std::printf("pass %d fail %d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

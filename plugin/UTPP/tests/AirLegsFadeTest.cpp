// fix2/hip P6 K1: in the air the leg layer of the weapon pose must follow the locomotion (jump) clip.
// Recording 2026-10-21 12:15, jumps in place with a weapon: the Stand_*_Idle weapon poses carry layering_legs = 1, so the leg
// layer is OP + D_MS(B -> L) and the knee flexion of the jump clip (86-123 deg) is added to a thigh that the weapon pose has turned
// by 58-89 deg in mesh space (knee off its hinge: 31 deg median / 47 deg max; jumps in motion, legs curve 0: 1.3 / 11 deg).
//   A) ov::FadeLowerBodyInAir: legs curve * (1 - airW), pelvis and every other curve untouched, multiplicative with
//      PreserveIdlePistolLowerBody;
//   B) synthetic ComposeLayers (Hips -> UpLeg -> Leg -> Foot, thigh turned 70 deg, knee flexed 100 deg): without the fade the knee is
//      off its hinge by > 25 deg; with airW 1 the leg is the locomotion leg to 0.5 deg, the pelvis keeps the weapon pose;
//   C) ov::AirLegsFade ramp at 30 / 60 / 144 Hz: >= 0.9 within 0.07 s of the take-off, 1 for the whole flight and 0.15 s after the
//      landing, 0 within 0.55 s of the landing, bounded step, a jump series with a 0.2 s gap does not return to 0;
//   D) (optional arguments) the same on the real woman data through the PoseEvaluator: jump clips under the Pistol2H / Rifle
//      weapon pose, knee deviation from the unarmed pose without and with airW 1.
// Usage: utpp_air_legs_fade_test [<woman v2 db> <woman overlay> <woman style weights>]
//   E) (optional arguments) thigh pop of a jump in place on the real data at 30 / 60 Hz: <= 13 deg median / 24 max per frame in the
//      0.5 s after the take-off (P6.md criterion), the landing window no worse than without the fade.
//   F) (optional arguments) the stance return after the landing moves a planted foot at most 0.70 m/s.
// Mutations UTPP_AIRLEGS_MUTATION 1 (FadeLowerBodyInAir does nothing), 2 (no hold after the landing), 3 (MotionPose ignores airW), 4 (no
// per-frame cap on the rise: 0.06 s at any frame rate), 5 (fall 0.25 s) must each print a FAIL line.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/MotionPose.hpp"
#include "TestFailFast.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
    if (!aOk)
        UtppFailFastNote(std::string("FAIL ") + aWhat);
}
template <class... A> std::string F(const char* aFmt, A... a)
{
    char b[640];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

Curves AllOnes()
{
    Curves c;
    for (int i = 0; i < kCurveCount; ++i)
        c.Set(i, 1.0f);
    return c;
}

// ---- B) knee off the hinge ----------------------------------------------------------------------------------------
constexpr float kDeg = math::kPi / 180.0f;
math::Quat AxisAngle(math::Vec3 aAxis, float aDeg) { return math::QuatFromScaledAngleAxis(aAxis * (aDeg * kDeg)); }
// Swing angle (deg) of the rotation aQ away from rotation about aHinge (swing-twist decomposition).
float OffHingeDeg(const math::Quat& aQ, math::Vec3 aHinge)
{
    const math::Quat q = math::QuatNormalize(aQ);
    const float d = math::Dot(math::Vec3{q.x, q.y, q.z}, aHinge);
    const math::Quat twist = math::QuatNormalize(math::Quat{aHinge.x * d, aHinge.y * d, aHinge.z * d, q.w});
    return AngleDeg(math::Quat{}, math::QuatMul(q, math::QuatInv(twist)));
}

struct Leg
{
    Skeleton sk;
    Regions regions;
    Pose B, L, OP, out;
    LayerScratch* scratch = nullptr;
};

void SyntheticKnee()
{
    // 0 Root, 1 Hips, 2 LeftUpLeg, 3 LeftLeg, 4 LeftFoot, 5 Spine
    static const char* names[] = {"Root", "Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "Spine"};
    Leg g;
    g.sk.n = 6;
    const int16_t parents[] = {-1, 0, 1, 2, 3, 1};
    for (int i = 0; i < 6; ++i)
        g.sk.parent[i] = parents[i];
    BuildRegions(g.sk, [&](int i) { return std::string_view(names[i]); }, g.regions);
    g.B.n = g.L.n = g.OP.n = g.out.n = 6;
    for (int i = 0; i < 6; ++i)
        g.B.b[i] = g.L.b[i] = g.OP.b[i] = Xf{};
    const math::Vec3 hinge{1.0f, 0.0f, 0.0f}; // knee hinge in the Leg frame
    // locomotion (jump clip): hip flexed 60 deg, knee flexed 100 deg about the hinge, both about X
    g.L.b[2].r = AxisAngle(hinge, 60.0f);
    g.L.b[3].r = AxisAngle(hinge, 100.0f);
    // weapon pose: pelvis turned 20 deg, thigh 50 deg more = 70 deg in mesh space (measured 58-89 deg), knee 18 deg
    g.OP.b[1].r = AxisAngle({0.0f, 0.0f, 1.0f}, 20.0f);
    g.OP.b[2].r = AxisAngle({0.0f, 0.0f, 1.0f}, 50.0f);
    g.OP.b[3].r = AxisAngle(hinge, 18.0f);
    auto scratch = std::make_unique<LayerScratch>();
    LayerInputs in;
    in.L = &g.L;
    in.B = &g.B;
    in.OP = &g.OP;
    in.dP = nullptr;
    const float kneeLoco = OffHingeDeg(g.L.b[3].r, hinge);
    in.curves = AllOnes();
    ComposeLayers(g.sk, g.regions, in, *scratch, g.out);
    const float offBefore = OffHingeDeg(g.out.b[3].r, hinge);
    const float devBefore = AngleDeg(g.L.b[3].r, g.out.b[3].r);
    Check(kneeLoco < 0.01f && offBefore > 25.0f,
          F("B: legs curve 1 in the air bends the knee off its hinge (locomotion %.2f deg, composed %.1f deg off, %.1f deg from the clip)",
            static_cast<double>(kneeLoco), static_cast<double>(offBefore), static_cast<double>(devBefore)));
    in.curves = AllOnes();
    FadeLowerBodyInAir(1.0f, in.curves);
    ComposeLayers(g.sk, g.regions, in, *scratch, g.out);
    const float offAfter = OffHingeDeg(g.out.b[3].r, hinge);
    const float devKnee = AngleDeg(g.L.b[3].r, g.out.b[3].r);
    // the leg keeps the locomotion orientation in mesh space even though the pelvis follows the weapon pose
    math::Quat meshL[ov::kMaxBones], meshOut[ov::kMaxBones];
    MeshRotations(g.sk, g.L, meshL);
    MeshRotations(g.sk, g.out, meshOut);
    const float devThigh = AngleDeg(meshL[2], meshOut[2]);
    const float devFoot = AngleDeg(meshL[4], meshOut[4]);
    const float pelvisDev = AngleDeg(g.L.b[1].r, g.out.b[1].r);
    Check(devKnee < 0.5f && offAfter < 0.5f && devThigh < 0.5f && devFoot < 0.5f,
          F("B: airW 1 gives the locomotion leg (knee %.3f deg from the clip, %.3f off hinge, thigh %.3f, foot %.3f deg in mesh space)",
            static_cast<double>(devKnee), static_cast<double>(offAfter), static_cast<double>(devThigh), static_cast<double>(devFoot)));
    Check(pelvisDev > 10.0f, F("B: the pelvis keeps the weapon pose (%.1f deg from the clip)", static_cast<double>(pelvisDev)));
    // half way: between the two, no overshoot
    in.curves = AllOnes();
    FadeLowerBodyInAir(0.5f, in.curves);
    ComposeLayers(g.sk, g.regions, in, *scratch, g.out);
    const float devHalf = AngleDeg(g.L.b[3].r, g.out.b[3].r);
    Check(devHalf > 0.5f && devHalf < devBefore, F("B: airW 0.5 is in between (%.1f deg from the clip, %.1f at airW 0)",
                                                     static_cast<double>(devHalf), static_cast<double>(devBefore)));
}

// ---- A) curves ------------------------------------------------------------------------------------------------------
void CurveContract()
{
    Curves c = AllOnes();
    FadeLowerBodyInAir(1.0f, c);
    bool others = true;
    for (int i = 0; i < kCurveCount; ++i)
        if (i != kLegs && c[i] != 1.0f)
            others = false;
    Check(c[kLegs] == 0.0f && c[kPelvis] == 1.0f && others, "A: airW 1 removes the leg curve only (pelvis and the upper body keep 1)");
    c = AllOnes();
    FadeLowerBodyInAir(0.0f, c);
    Check(c[kLegs] == 1.0f && c[kPelvis] == 1.0f, "A: airW 0 leaves the curves unchanged");
    c = AllOnes();
    FadeLowerBodyInAir(0.5f, c);
    Check(std::fabs(c[kLegs] - 0.5f) < 1e-6f, "A: airW 0.5 halves the leg curve");
    c = AllOnes();
    FadeLowerBodyInAir(2.0f, c);
    const float over = c[kLegs];
    c = AllOnes();
    FadeLowerBodyInAir(-1.0f, c);
    const float under = c[kLegs];
    c = AllOnes();
    FadeLowerBodyInAir(std::nanf(""), c);
    Check(over == 0.0f && under == 1.0f && c[kLegs] == 1.0f, "A: out of range / NaN airW is clamped (NaN = 0)");
    // composes multiplicatively with the female relaxed-pistol idle policy
    OverlayMachine m;
    m.style = Style::Feminine;
    const int idle = PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false);
    m.poses.Reset(idle);
    m.poses.SetTarget(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, true), 0.2f);
    m.poses.Update(0.1f);
    c = AllOnes();
    PreserveIdlePistolLowerBody(m, true, c);
    const float keep = c[kLegs];
    FadeLowerBodyInAir(0.5f, c);
    Check(keep > 0.0f && keep < 1.0f && std::fabs(c[kLegs] - keep * 0.5f) < 1e-6f && std::fabs(c[kPelvis] - keep) < 1e-6f,
          F("A: multiplicative with PreserveIdlePistolLowerBody (keep %.3f -> legs %.3f)", static_cast<double>(keep),
            static_cast<double>(c[kLegs])));
}

// ---- C) ramp -------------------------------------------------------------------------------------------------------
struct Seq
{
    float maxStep = 0.0f;
    std::vector<float> w;
};
// frames: (air, count) at the given rate
Seq Run(AirLegsFade& aF, int aHz, const std::vector<std::pair<bool, double>>& aSegments)
{
    Seq s;
    const float dt = 1.0f / static_cast<float>(aHz);
    float prev = aF.w;
    for (const auto& seg : aSegments)
    {
        const int frames = static_cast<int>(std::lround(seg.second * aHz));
        for (int i = 0; i < frames; ++i)
        {
            const float w = aF.Step(seg.first, dt);
            s.maxStep = std::fmax(s.maxStep, std::fabs(w - prev));
            prev = w;
            s.w.push_back(w);
        }
    }
    return s;
}

void Ramp(int aHz)
{
    const float dt = 1.0f / static_cast<float>(aHz);
    const auto at = [&](const Seq& s, double aSeconds) {
        const size_t i = std::min(s.w.size() - 1, static_cast<size_t>(std::lround(aSeconds / dt)));
        return s.w[i];
    };
    // 10 frames on the ground, a 0.7 s flight, 0.6 s on the ground
    AirLegsFade f;
    const double ground0 = 10.0 / aHz;
    const Seq s = Run(f, aHz, {{false, ground0}, {true, 0.7}, {false, 0.8}});
    const double takeoff = ground0, landing = ground0 + 0.7;
    const float groundBefore = at(s, takeoff * 0.5);
    // s.w[k] is the value after frame k. Full weight within max(0.07 s, 5 frames) of the first air frame (the rise is capped per frame
    // so that a low frame rate does not move the thigh by 17 deg per frame: 0.22 per frame = 5 frames)
    const double riseBy = std::max(0.07, 5.0 * dt);
    const float rise = at(s, takeoff + riseBy);
    float flightMin = 1.0f;
    for (double t = takeoff + riseBy; t < landing - dt; t += dt)
        flightMin = std::fmin(flightMin, at(s, t));
    // after the landing the value after k ground frames; hold = 0.15 s
    float holdMin = 1.0f;
    for (double t = landing; t < landing + 0.15 - 1.5 * dt; t += dt)
        holdMin = std::fmin(holdMin, at(s, t));
    const float gone = at(s, landing + 0.55 + dt);
    Check(groundBefore == 0.0f && rise >= 0.9f && flightMin >= 0.999f,
          F("C@%dHz: 0 on the ground, %.2f %.3f s after take-off, min %.3f during the flight", aHz, static_cast<double>(rise), riseBy,
            static_cast<double>(flightMin)));
    Check(holdMin >= 0.999f, F("C@%dHz: 1 for 0.15 s after the landing (min %.3f)", aHz, static_cast<double>(holdMin)));
    Check(gone <= 0.001f, F("C@%dHz: 0 within 0.55 s of the landing (hold 0.15 s + fall 0.35 s) (%.3f)", aHz, static_cast<double>(gone)));
    // step per frame <= 0.23 at every frame rate (the thigh moves ~70 deg between the two stances: 0.55 per frame at 30 Hz was a 17 deg
    // median thigh step per frame at the take-off, see TakeoffPop)
    Check(s.maxStep <= 0.23f, F("C@%dHz: step per frame <= 0.23 (%.3f)", aHz, static_cast<double>(s.maxStep)));
    // series of jumps with a 0.2 s gap never returns to 0, the next take-off rises from the current value
    AirLegsFade g;
    float minDuring = 1.0f;
    for (int k = 0; k < 3; ++k)
    {
        const Seq part = Run(g, aHz, {{true, 0.7}, {false, 0.2}});
        if (k > 0)
            for (float w : part.w)
                minDuring = std::fmin(minDuring, w);
    }
    Check(minDuring > 0.5f, F("C@%dHz: a jump series with a 0.2 s gap does not return to 0 (min %.2f)", aHz, static_cast<double>(minDuring)));
    // non-finite / negative dt never moves the value
    AirLegsFade h;
    h.w = 0.4f;
    const float a = h.Step(true, std::nanf(""));
    const float b = h.Step(true, -1.0f);
    Check(a == 0.4f && b == 0.4f, "C: non-finite / negative dt leaves the weight unchanged");
}

// ---- D) real data ---------------------------------------------------------------------------------------------------
int FindClip(const motion::MotionDatabase& aDb, std::string_view aName)
{
    for (uint32_t i = 0; i < aDb.NumClips(); ++i)
        if (aDb.ClipName(i) == aName)
            return static_cast<int>(i);
    return -1;
}
int FindBone(const motion::MotionDatabase& aDb, std::string_view aName)
{
    for (uint32_t i = 0; i < aDb.NumBones(); ++i)
        if (aDb.BoneName(i) == aName)
            return static_cast<int>(i);
    return -1;
}

// E) thigh pop of a jump in place with a weapon pose on the real data: stand idle, take-off into a Jump_B_Start clip (blend 0.5 s like the
// recording), 0.7 s flight, landing clip (blend 0.2 s), 1 s on the ground. The airW comes from the production AirLegsFade stepped at
// the test frame rate. Max mesh-space thigh rotation step per frame (deg) in the 0.5 s after the take-off and in the 0.6 s after the
// landing, median / max over the take-off x landing clip pairs. Criterion (P6.md): thigh pop <= 13 deg median / 24 deg max; the
// landing window may not be worse than the same sequence without the fade (the pre-fix pose) by more than 1 deg.
struct Pop
{
    float take = 0.0f, land = 0.0f;
};
Pop JumpPop(const motion::MotionDatabase& aDb, const OverlayFile& aOv, const StyleFamilyData& aStyle, PoseOverlay aOverlay, int aHz, int aTakeClip,
            float aTakeT, int aLandClip, float aLandT, int aIdle, bool aFade)
{
    const uint32_t n = aDb.NumBones();
    const float dt = 1.0f / static_cast<float>(aHz);
    const int thigh[2] = {FindBone(aDb, "LeftUpLeg"), FindBone(aDb, "RightUpLeg")};
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    std::string err;
    if (!ev->Init(&aDb, &aOv, &aStyle, 2, err))
        return {};
    motionpose::OverlayFrameInputs in;
    in.machine.overlay = aOverlay;
    in.machine.style = Style::Feminine;
    in.bodyWoman = true;
    in.machine.wantsReady = true;
    AirLegsFade fade;
    std::vector<BoneTransform> out(n);
    std::vector<math::Quat> rot(n);
    math::Quat prev[2];
    bool havePrev = false;
    Pop pop;
    const float fps = aDb.Header().fps > 0.0f ? aDb.Header().fps : 30.0f;
    auto duration = [&](int c) { return static_cast<float>(aDb.Clip(static_cast<uint32_t>(c)).numFrames - 1) / fps; };
    auto frame = [&](const motion::MatchResult& m, bool aAir, int aPhase, float aSince) {
        in.machine.airW = aFade ? fade.Step(aAir, dt) : 0.0f;
        ev->Evaluate(dt, m, in, out.data(), n);
        for (uint32_t i = 0; i < n; ++i)
        {
            const int p = aDb.BoneParent(i);
            rot[i] = p < 0 ? out[i].q : math::QuatNormalize(math::QuatMul(rot[static_cast<size_t>(p)], out[i].q));
        }
        const math::Quat cur[2] = {rot[static_cast<size_t>(thigh[0])], rot[static_cast<size_t>(thigh[1])]};
        if (havePrev && aPhase > 0)
        {
            const float a = std::fmax(AngleDeg(prev[0], cur[0]), AngleDeg(prev[1], cur[1]));
            if (aPhase == 1 && aSince <= 0.5f) pop.take = std::fmax(pop.take, a);
            if (aPhase == 2 && aSince <= 0.6f) pop.land = std::fmax(pop.land, a);
        }
        prev[0] = cur[0];
        prev[1] = cur[1];
        havePrev = true;
    };
    motion::MatchResult m;
    m.clip = aIdle;
    m.clipTime = 0.5f;
    for (int i = 0; i < 150; ++i)
        frame(m, false, 0, 0.0f);
    float t = aTakeT;
    for (int i = 0; i < static_cast<int>(std::lround(0.7f * static_cast<float>(aHz))); ++i, t += dt)
    {
        motion::MatchResult r;
        r.clip = aTakeClip;
        r.clipTime = std::fmin(t, duration(aTakeClip));
        if (i == 0)
        {
            r.jumped = true;
            r.blend.valid = true;
            r.blend.blendTime = 0.5f;
            r.blend.toClip = aTakeClip;
            r.blend.fromClip = aIdle;
        }
        frame(r, true, 1, static_cast<float>(i) * dt);
    }
    t = aLandT;
    for (int i = 0; i < aHz; ++i, t += dt)
    {
        motion::MatchResult r;
        r.clip = aLandClip;
        r.clipTime = std::fmin(t, duration(aLandClip));
        if (i == 0)
        {
            r.jumped = true;
            r.blend.valid = true;
            r.blend.blendTime = 0.2f;
            r.blend.toClip = aLandClip;
            r.blend.fromClip = aTakeClip;
        }
        frame(r, false, 2, static_cast<float>(i) * dt);
    }
    return pop;
}

void TakeoffPop(const motion::MotionDatabase& aDb, const OverlayFile& aOv, const StyleFamilyData& aStyle)
{
    const int idle = FindClip(aDb, "M_Neutral_Stand_Idle_Loop");
    struct Clip
    {
        const char* name;
        float t;
    };
    const Clip takes[] = {{"M_Neutral_Jump_B_Start_Rfoot", 0.63f}, {"M_Neutral_Jump_B_Start_Lfoot", 0.63f}, {"M_Neutral_Jump_B_Start_Rfoot", 0.49f}};
    const Clip lands[] = {{"M_Neutral_Jump_F_Land_Stand_Light_Lfoot", 0.47f}, {"M_Neutral_Jump_F_Land_Stand_Light_Rfoot", 0.47f}};
    bool clipsOk = idle >= 0;
    for (const Clip& c : takes) clipsOk = clipsOk && FindClip(aDb, c.name) >= 0;
    for (const Clip& c : lands) clipsOk = clipsOk && FindClip(aDb, c.name) >= 0;
    Check(clipsOk, "E: the take-off / landing / idle clips of the recording exist in the woman database");
    if (!clipsOk)
        return;
    struct Setup
    {
        PoseOverlay overlay;
        const char* name;
    };
    for (const Setup& setup : {Setup{PoseOverlay::Pistol2H, "Pistol2H"}, Setup{PoseOverlay::Rifle, "Rifle"}})
        for (int hz : {30, 60})
        {
            std::vector<float> take, land, landPre;
            for (const Clip& tk : takes)
                for (const Clip& ld : lands)
                {
                    const int tc = FindClip(aDb, tk.name), lc = FindClip(aDb, ld.name);
                    const Pop fade = JumpPop(aDb, aOv, aStyle, setup.overlay, hz, tc, tk.t, lc, ld.t, idle, true);
                    const Pop pre = JumpPop(aDb, aOv, aStyle, setup.overlay, hz, tc, tk.t, lc, ld.t, idle, false);
                    take.push_back(fade.take);
                    land.push_back(fade.land);
                    landPre.push_back(pre.land);
                }
            auto median = [](std::vector<float> v) {
                std::sort(v.begin(), v.end());
                return v[v.size() / 2];
            };
            const float takeMed = median(take), takeMax = *std::max_element(take.begin(), take.end());
            const float landMax = *std::max_element(land.begin(), land.end()), landPreMax = *std::max_element(landPre.begin(), landPre.end());
            Check(takeMed <= 13.0f && takeMax <= 24.0f,
                  F("E %s@%dHz: thigh step per frame in the 0.5 s after the take-off %.1f deg median / %.1f max (criterion 13 / 24)", setup.name, hz,
                    static_cast<double>(takeMed), static_cast<double>(takeMax)));
            Check(landMax <= landPreMax + 1.0f,
                  F("E %s@%dHz: thigh step per frame in the 0.6 s after the landing %.1f deg max, pre-fix %.1f (no worse by > 1 deg)", setup.name, hz,
                    static_cast<double>(landMax), static_cast<double>(landPreMax)));
        }
}

// F) the fall moves the feet on the ground: after the landing the legs return from the clip stance (legs 0) to the standing weapon stance
// (legs 1). Largest foot displacement between the two on the stand idle clip (mesh space), relaxed and ready, Pistol2H and Rifle, divided
// by the fall time, may not exceed the foot speed of the unarmed landing clip's own motion on the ground (0.63 m/s measured 0.3-0.9 s
// after the landing at 30 and 60 Hz) plus 0.07 m/s: the 0.25 s fall was 0.92 m/s on a planted foot (FootLocker settle 0.12 m/s).
void StanceReturn(const motion::MotionDatabase& aDb, const OverlayFile& aOv, const StyleFamilyData& aStyle)
{
    const int idle = FindClip(aDb, "M_Neutral_Stand_Idle_Loop");
    const int feet[2] = {FindBone(aDb, "LeftFoot"), FindBone(aDb, "RightFoot")};
    Check(idle >= 0 && feet[0] >= 0 && feet[1] >= 0, "F: stand idle clip and foot bones found");
    if (idle < 0 || feet[0] < 0 || feet[1] < 0)
        return;
    const uint32_t n = aDb.NumBones();
    float worstCm = 0.0f;
    std::string worst;
    for (PoseOverlay overlay : {PoseOverlay::Pistol2H, PoseOverlay::Rifle})
        for (int ready = 0; ready < 2; ++ready)
        {
            std::vector<math::Vec3> foot[2];
            for (int air = 0; air < 2; ++air)
            {
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                std::string err;
                if (!ev->Init(&aDb, &aOv, &aStyle, 2, err))
                    return;
                motionpose::OverlayFrameInputs in;
                in.machine.overlay = overlay;
                in.machine.style = Style::Feminine;
                in.bodyWoman = true;
                in.machine.wantsReady = ready != 0;
                in.machine.airW = static_cast<float>(air);
                motion::MatchResult m;
                m.clip = idle;
                m.clipTime = 0.5f;
                std::vector<BoneTransform> out(n);
                for (int f = 0; f < 200; ++f)
                    ev->Evaluate(1.0f / 60.0f, m, in, out.data(), n);
                std::vector<math::Quat> rot(n);
                std::vector<math::Vec3> pos(n);
                for (uint32_t i = 0; i < n; ++i)
                {
                    const int p = aDb.BoneParent(i);
                    if (p < 0) { rot[i] = out[i].q; pos[i] = out[i].p; }
                    else
                    {
                        rot[i] = math::QuatNormalize(math::QuatMul(rot[static_cast<size_t>(p)], out[i].q));
                        pos[i] = pos[static_cast<size_t>(p)] + math::QuatRotate(rot[static_cast<size_t>(p)], out[i].p);
                    }
                }
                foot[air] = {pos[static_cast<size_t>(feet[0])], pos[static_cast<size_t>(feet[1])]};
            }
            for (int s = 0; s < 2; ++s)
            {
                const math::Vec3 d = foot[1][static_cast<size_t>(s)] - foot[0][static_cast<size_t>(s)];
                const float cm = std::sqrt(math::Dot(d, d)) * 100.0f;
                if (cm > worstCm)
                {
                    worstCm = cm;
                    worst = std::string(overlay == PoseOverlay::Pistol2H ? "Pistol2H " : "Rifle ") + (ready ? "ready " : "relaxed ") + (s ? "R" : "L");
                }
            }
        }
    const float speed = worstCm / 100.0f / kAirLegsFallSeconds;
    Check(speed <= 0.70f, F("F: the stance return after the landing moves a planted foot %.1f cm (%s) in %.2f s = %.2f m/s (bound 0.70 m/s)",
                            static_cast<double>(worstCm), worst.c_str(), static_cast<double>(kAirLegsFallSeconds), static_cast<double>(speed)));
}

void RealData(const char* aDbPath, const char* aOvPath, const char* aStylePath)
{
    std::string err;
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<OverlayFile>();
    StyleFamilyData style;
    const bool dbOk = db->LoadFromFile(aDbPath, motion::kRigIdentityWoman, err);
    const bool ovOk = ovf->LoadFromFile(aOvPath, kRigIdentityWomanOv, err);
    std::ifstream sf(aStylePath, std::ios::binary);
    std::stringstream ss;
    ss << sf.rdbuf();
    const bool stOk = sf.good() && style.Parse(ss.str(), &err);
    Check(dbOk && ovOk && stOk, F("D: load woman db / overlay / style (%s)", err.c_str()));
    if (!dbOk || !ovOk || !stOk)
        return;
    // premise: the weapon poses carry layering_legs = 1 (the reason the jump knee is touched at all)
    for (const char* pose : {"Pose_Pistol2H_Stand_Ready_Idle", "Pose_Rifle_Stand_Ready_Idle", "Pose_Pistol2H_Stand_Aim_Idle"})
    {
        const int p = ovf->FindPose(pose);
        float minLegs = 1.0f;
        if (p >= 0)
            for (uint32_t i = 0; i < ovf->PoseAt(p).numFrames; ++i)
            {
                Curves actual;
                ovf->FrameCurves(p, ovf->PoseAt(p).firstFrame + i, actual);
                minLegs = std::fmin(minLegs, actual[kLegs]);
            }
        Check(p >= 0 && minLegs > 0.99f, F("D: %s carries layering_legs 1 (min %.2f)", pose, static_cast<double>(minLegs)));
    }
    const uint32_t n = db->NumBones();
    const int kneeL = FindBone(*db, "LeftLeg"), kneeR = FindBone(*db, "RightLeg");
    const int thighL = FindBone(*db, "LeftUpLeg"), thighR = FindBone(*db, "RightUpLeg");
    Check(kneeL >= 0 && kneeR >= 0 && thighL >= 0 && thighR >= 0, "D: leg bones found");
    if (kneeL < 0 || kneeR < 0 || thighL < 0 || thighR < 0)
        return;
    TakeoffPop(*db, *ovf, style);
    StanceReturn(*db, *ovf, style);
    const float dt = 1.0f / 60.0f;
    struct Setup
    {
        PoseOverlay overlay;
        const char* name;
    };
    for (const Setup& setup : {Setup{PoseOverlay::Pistol2H, "Pistol2H"}, Setup{PoseOverlay::Rifle, "Rifle"}})
    {
        std::vector<float> devNoFade, devFade, flexAll;
        int samples = 0;
        std::string worst;
        float worstDev = 0.0f;
        // the clips of the recording: standing take-off clips Jump_B_Start_*, Jump_F_Start_Stand_*, and the landing Jump_F_Land_Stand_*
        for (uint32_t c = 0; c < db->NumClips(); ++c)
        {
            const std::string name(db->ClipName(c));
            if (name.find("Jump_B_Start") == std::string::npos && name.find("Jump_F_Start_Stand") == std::string::npos &&
                name.find("Jump_F_Land_Stand") == std::string::npos)
                continue;
            const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
            const float dur = static_cast<float>(db->Clip(c).numFrames - 1) / fps;
            for (float t = 0.0f; t <= dur; t += 0.1f)
            {
                motion::MatchResult m;
                m.clip = static_cast<int32_t>(c);
                m.clipTime = t;
                std::vector<BoneTransform> pure(n), armed(n), armedAir(n);
                auto settle = [&](PoseOverlay aOverlay, float aAirW, std::vector<BoneTransform>& aOut) {
                    auto ev = std::make_unique<motionpose::PoseEvaluator>();
                    std::string e;
                    if (!ev->Init(db.get(), ovf.get(), &style, 2, e))
                        return false;
                    motionpose::OverlayFrameInputs in;
                    in.machine.overlay = aOverlay;
                    in.machine.style = Style::Feminine;
                    in.bodyWoman = true;
                    in.machine.wantsReady = aOverlay != PoseOverlay::Default;
                    in.machine.airW = aAirW;
                    bool ok = true;
                    for (int f = 0; f < 150; ++f)
                        ok = ev->Evaluate(dt, m, in, aOut.data(), n) && ok;
                    return ok;
                };
                if (!settle(PoseOverlay::Default, 0.0f, pure) || !settle(setup.overlay, 0.0f, armed) ||
                    !settle(setup.overlay, 1.0f, armedAir))
                    continue;
                const float flex = std::fmax(AngleDeg(math::Quat{}, pure[kneeL].q), AngleDeg(math::Quat{}, pure[kneeR].q));
                if (flex < 50.0f) // only the frames with a real knee bend (the jump / landing flexion)
                    continue;
                for (int k : {kneeL, kneeR})
                {
                    const float d0 = AngleDeg(pure[k].q, armed[k].q), d1 = AngleDeg(pure[k].q, armedAir[k].q);
                    devNoFade.push_back(d0);
                    devFade.push_back(d1);
                    if (d0 > worstDev)
                    {
                        worstDev = d0;
                        worst = name + "@" + F("%.1f", static_cast<double>(t));
                    }
                }
                flexAll.push_back(flex);
                ++samples;
            }
        }
        auto med = [](std::vector<float> v) {
            if (v.empty())
                return 0.0f;
            std::sort(v.begin(), v.end());
            return v[v.size() / 2];
        };
        const float mx0 = devNoFade.empty() ? 0.0f : *std::max_element(devNoFade.begin(), devNoFade.end());
        const float mx1 = devFade.empty() ? 0.0f : *std::max_element(devFade.begin(), devFade.end());
        std::printf("INFO D %s: %d clip samples with knee flexion >= 50 deg (median %.0f deg); knee deviation from the unarmed jump: "
                    "legs 1 median %.1f max %.1f (%s), airW 1 median %.3f max %.3f\n",
                    setup.name, samples, static_cast<double>(med(flexAll)), static_cast<double>(med(devNoFade)),
                    static_cast<double>(mx0), worst.c_str(), static_cast<double>(med(devFade)), static_cast<double>(mx1));
        Check(samples >= 10, F("D %s: enough bent-knee jump samples (%d)", setup.name, samples));
        Check(med(devNoFade) > 10.0f, F("D %s: the weapon pose bends the jump knee away from the clip (median %.1f deg)", setup.name,
                                         static_cast<double>(med(devNoFade))));
        Check(mx1 < 0.5f, F("D %s: airW 1 returns the clip knee through the PoseEvaluator (max %.3f deg)", setup.name,
                             static_cast<double>(mx1)));
    }
}
} // namespace

int main(int argc, char** argv)
{
    CurveContract();
    SyntheticKnee();
    for (int hz : {30, 60, 144})
        Ramp(hz);
    if (argc >= 4)
        RealData(argv[1], argv[2], argv[3]);
    else
        std::printf("SKIP D: pass <woman v2 db> <woman overlay> <woman style weights> for the real-data check\n");
    std::printf("air legs fade: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

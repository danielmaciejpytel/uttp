// Clavicle limit of the mesh-space arm layer (src/mm/MeshArmClavicleLimit.hpp): recordings 2026-10-25 01:24 / later show the shoulder
// skin balloon while the LeftShoulder local rotation is 59-84 deg away from the arm-region clavicle (calm aim 6, run aim 10-20 deg).
//   A) below the limit (also in the opposite quaternion hemisphere) the pose is bitwise untouched;
//   B) hard clamp (soft 0): the clavicle ends exactly at the limit angle, the axis of the deviation is kept;
//   C) soft knee: identity up to the limit, C1 at the limit, monotonic, never above limit + soft, per-step change <= the input step;
//   D) children keep their MESH rotation (arm / forearm / hand), every other bone is bitwise untouched, left and right are independent;
//   E) hemisphere: the same rotation stored as -q is limited like q (and a small deviation stored as -q is not touched);
//   F) invalid input (bone index, NaN, negative / NaN limit) leaves the pose bitwise untouched;
//   G) synthetic ComposeLayers (Spine3 turned 70 deg in the locomotion pose, aim arm layer Arm_X 1 / Add 0 / LS 0): the clavicle
//      deviation is the Spine3 turn (the hypothesis), the limit bounds it, a calm pose (Spine3 turn 8 deg) is unchanged bitwise;
//   H) (optional arguments) PoseEvaluator on the real woman data, Pistol1H / Pistol2H / Rifle aiming on the stand idle / walk / run loop:
//      the RUNTIME limit (kMeshArmClavicleRuntimeLimitDeg, measurement only) never engages = pose bitwise as without the limit; prints the
//      largest clavicle deviation per setup. /DUTPP_CLAVLIMIT_SURVEY adds a survey of every loop clip (calm poses reach 40-70 deg).
// Usage: utpp_clavicle_limit_test [<woman v2 db> <woman overlay> <woman style weights>]
// Mutations UTPP_CLAVLIMIT_MUTATION 1 (no limit), 2 (children not compensated), 3 (no shortest-arc fix) must each print a FAIL line.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/MotionPose.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
template <class... A> std::string F(const char* aFmt, A... a)
{
    char b[640];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

constexpr float kDeg = math::kPi / 180.0f;
math::Quat AxisAngle(math::Vec3 aAxis, float aDeg) { return math::QuatFromScaledAngleAxis(aAxis * (aDeg * kDeg)); }
float AngleDeg(const math::Quat& a, const math::Quat& b)
{
    const math::Quat r = math::QuatAbs(math::QuatNormalize(math::QuatMul(math::QuatInv(math::QuatNormalize(a)), math::QuatNormalize(b))));
    return 2.0f * std::atan2(std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z), r.w) / kDeg;
}
bool Same(const Xf& a, const Xf& b) { return std::memcmp(&a, &b, sizeof(Xf)) == 0; }

// 0 Root, 1 Hips, 2 Spine, 3 Spine1, 4 Spine2, 5 Spine3, 6 LeftShoulder, 7 LeftArm, 8 LeftForeArm, 9 LeftHand, 10 LeftWing (second child of
// the clavicle), 11 RightShoulder, 12 RightArm, 13 Neck
constexpr int kN = 14;
constexpr int kSpine3 = 5, kClavL = 6, kBArmL = 7, kForeL = 8, kBHandL = 9, kWingL = 10, kClavR = 11, kBArmR = 12;
const char* const kNames[kN] = {"Root",  "Hips",         "Spine",         "Spine1",      "Spine2",       "Spine3",   "LeftShoulder",
                                "LeftArm", "LeftForeArm", "LeftHand",      "LeftWing",    "RightShoulder", "RightArm", "Neck"};
const int16_t kParents[kN] = {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 6, 5, 11, 5};

void MakeSkeleton(Skeleton& aSk)
{
    aSk.n = kN;
    for (int i = 0; i < kN; ++i)
        aSk.parent[i] = kParents[i];
}
void FillPose(Pose& aP)
{
    aP.n = kN;
    for (int i = 0; i < kN; ++i)
    {
        aP.b[i] = Xf{};
        aP.b[i].t = {0.0f, 0.1f * static_cast<float>(i), 0.0f};
        aP.b[i].r = AxisAngle({0.3f, 0.5f, 0.8f}, 7.0f * static_cast<float>(i + 1));
    }
}
void Mesh(const Skeleton& aSk, const Pose& aP, math::Quat* aOut) { MeshRotations(aSk, aP, aOut); }

// pose whose left clavicle sits aDeg away from aRef about aAxis
Pose WithClavDeviation(const math::Quat& aRef, math::Vec3 aAxis, float aDeg, int aClav = kClavL)
{
    Pose p;
    FillPose(p);
    p.b[aClav].r = math::QuatNormalize(math::QuatMul(aRef, AxisAngle(aAxis, aDeg)));
    return p;
}
const math::Quat kRef = math::QuatNormalize(AxisAngle({0.2f, -0.6f, 0.4f}, 33.0f));
const math::Vec3 kAxis{0.0f, 0.6f, 0.8f};

// ---- A) + E) below the limit ---------------------------------------------------------------------------------------------------
void BelowLimit(const Skeleton& aSk)
{
    for (float soft : {0.0f, kMeshArmClavicleSoftDeg})
        for (float dev : {0.0f, 6.0f, 20.0f, 29.5f})
        {
            Pose p = WithClavDeviation(kRef, kAxis, dev);
            const Pose before = p;
            const float r = LimitClavicleDeviation(aSk, kRef, kClavL, kMeshArmClavicleLimitDeg, soft, p);
            Check(std::memcmp(&before, &p, sizeof(Pose)) == 0 && std::fabs(r - dev) < 0.01f,
                  F("A: deviation %.1f deg (soft %.0f) is untouched bitwise (returned %.3f)", static_cast<double>(dev), static_cast<double>(soft),
                    static_cast<double>(r)));
        }
    // E) the same small deviation stored in the opposite hemisphere (q and -q are one rotation)
    Pose p = WithClavDeviation(kRef, kAxis, 10.0f);
    p.b[kClavL].r = math::QuatNeg(p.b[kClavL].r);
    const Pose before = p;
    const float r = LimitClavicleDeviation(aSk, kRef, kClavL, kMeshArmClavicleLimitDeg, 0.0f, p);
    Check(std::memcmp(&before, &p, sizeof(Pose)) == 0 && std::fabs(r - 10.0f) < 0.05f,
          F("E: 10 deg stored as -q is untouched (returned %.3f, a long-arc measure would read 350)", static_cast<double>(r)));
}

// ---- B) hard clamp -------------------------------------------------------------------------------------------------------------
void HardClamp(const Skeleton& aSk)
{
    for (float dev : {30.5f, 45.0f, 59.0f, 84.0f, 120.0f, 179.0f})
    {
        Pose p = WithClavDeviation(kRef, kAxis, dev);
        const float r = LimitClavicleDeviation(aSk, kRef, kClavL, 30.0f, 0.0f, p);
        const float after = AngleDeg(kRef, p.b[kClavL].r);
        // axis kept: the limited rotation relative to the reference is about kAxis
        const math::Quat rel = math::QuatAbs(math::QuatMul(math::QuatInv(kRef), p.b[kClavL].r));
        const float s = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
        const float axisDot = s > 1e-6f ? (rel.x * kAxis.x + rel.y * kAxis.y + rel.z * kAxis.z) / s : 0.0f;
        Check(std::fabs(r - dev) < 0.05f && std::fabs(after - 30.0f) < 0.02f && axisDot > 0.9999f,
              F("B: %.1f deg -> hard clamp %.3f deg (returned %.2f, axis dot %.5f)", static_cast<double>(dev), static_cast<double>(after),
                static_cast<double>(r), static_cast<double>(axisDot)));
    }
}

// ---- C) soft knee --------------------------------------------------------------------------------------------------------------
void SoftKnee(const Skeleton& aSk)
{
    const float L = kMeshArmClavicleLimitDeg, R = kMeshArmClavicleSoftDeg;
    float prevOut = 0.0f, worstStep = 0.0f, maxOut = 0.0f, worstDelta = 0.0f;
    bool monotonic = true;
    const float step = 0.25f;
    for (float dev = 0.25f; dev <= 179.0f; dev += step)
    {
        Pose p = WithClavDeviation(kRef, kAxis, dev);
        LimitClavicleDeviation(aSk, kRef, kClavL, L, R, p);
        const float out = AngleDeg(kRef, p.b[kClavL].r);
        monotonic = monotonic && out >= prevOut - 0.01f;
        worstStep = std::max(worstStep, out - prevOut);
        maxOut = std::max(maxOut, out);
        worstDelta = std::max(worstDelta, std::fabs(out - SoftLimitAngleDeg(dev, L, R)));
        prevOut = out;
    }
    Check(monotonic, "C: limited angle is monotonic in the deviation");
    Check(worstStep <= step + 0.02f, F("C: output step per %.2f deg input step <= input step (worst %.3f): no kink / pop at the limit", static_cast<double>(step),
                                      static_cast<double>(worstStep)));
    Check(maxOut < L + R + 0.01f, F("C: never above limit + soft = %.0f deg (max %.3f)", static_cast<double>(L + R), static_cast<double>(maxOut)));
    Check(worstDelta < 0.03f, F("C: quaternion result matches SoftLimitAngleDeg (worst %.4f deg)", static_cast<double>(worstDelta)));
    Pose p84 = WithClavDeviation(kRef, kAxis, 84.0f);
    LimitClavicleDeviation(aSk, kRef, kClavL, L, R, p84);
    const float a84 = AngleDeg(kRef, p84.b[kClavL].r);
    Check(a84 > 43.0f && a84 < 45.0f, F("C: heavy-landing deviation 84 deg -> %.2f deg", static_cast<double>(a84)));
}

// ---- D) children keep their mesh rotation --------------------------------------------------------------------------------------
void Children(const Skeleton& aSk)
{
    for (float soft : {0.0f, kMeshArmClavicleSoftDeg})
    {
        Pose p = WithClavDeviation(kRef, kAxis, 75.0f);
        const Pose before = p;
        math::Quat m0[kN], m1[kN];
        Mesh(aSk, before, m0);
        LimitClavicleDeviation(aSk, kRef, kClavL, 30.0f, soft, p);
        Mesh(aSk, p, m1);
        float worstSub = 0.0f;
        for (int i : {kBArmL, kForeL, kBHandL, kWingL})
            worstSub = std::max(worstSub, AngleDeg(m0[i], m1[i]));
        Check(worstSub < 0.01f, F("D: soft %.0f: mesh rotation of LeftArm / ForeArm / Hand / second child kept (worst %.4f deg)",
                                  static_cast<double>(soft), static_cast<double>(worstSub)));
        Check(AngleDeg(m0[kClavL], m1[kClavL]) > 20.0f, "D: the clavicle mesh rotation itself did change");
        bool others = true;
        for (int i = 0; i < kN; ++i)
        {
            const bool touched = i == kClavL || i == kBArmL || i == kWingL; // clavicle + its direct children
            if (!touched && !Same(before.b[i], p.b[i]))
                others = false;
            if (std::memcmp(&before.b[i].t, &p.b[i].t, sizeof(math::Vec3)) != 0 || std::memcmp(&before.b[i].s, &p.b[i].s, sizeof(math::Vec3)) != 0)
                others = false;
        }
        Check(others, "D: every bone except the clavicle and its direct children is bitwise untouched; translations never change");
    }
    // left and right through LimitMeshArmClavicles with a scratch: right side below the limit is untouched
    auto scratch = std::make_unique<LayerScratch>();
    scratch->armL.n = scratch->armR.n = kN;
    for (int i = 0; i < kN; ++i)
        scratch->armL.b[i].r = scratch->armR.b[i].r = kRef;
    Pose p = WithClavDeviation(kRef, kAxis, 70.0f, kClavL);
    p.b[kClavR].r = math::QuatNormalize(math::QuatMul(kRef, AxisAngle(kAxis, 12.0f)));
    const Pose before = p;
    const ClavicleLimitResult r = LimitMeshArmClavicles(aSk, *scratch, kClavL, kClavR, 30.0f, 0.0f, p);
    Check(r.applied && std::fabs(r.leftDeg - 70.0f) < 0.1f && std::fabs(r.rightDeg - 12.0f) < 0.05f &&
              Same(before.b[kClavR], p.b[kClavR]) && Same(before.b[kBArmR], p.b[kBArmR]) && std::fabs(AngleDeg(kRef, p.b[kClavL].r) - 30.0f) < 0.02f,
          F("D: left limited (%.1f -> 30), right 12 deg and its children untouched", static_cast<double>(r.leftDeg)));
    Pose both = before;
    both.b[kClavR].r = math::QuatNormalize(math::QuatMul(kRef, AxisAngle({1.0f, 0.0f, 0.0f}, 80.0f)));
    const ClavicleLimitResult r2 = LimitMeshArmClavicles(aSk, *scratch, kClavL, kClavR, 30.0f, 0.0f, both);
    Check(r2.applied && std::fabs(AngleDeg(kRef, both.b[kClavR].r) - 30.0f) < 0.02f, "D: right side limited independently");
}

// ---- E) hemisphere -------------------------------------------------------------------------------------------------------------
void Hemisphere(const Skeleton& aSk)
{
    Pose a = WithClavDeviation(kRef, kAxis, 70.0f);
    Pose b = a;
    b.b[kClavL].r = math::QuatNeg(b.b[kClavL].r);
    LimitClavicleDeviation(aSk, kRef, kClavL, 30.0f, 0.0f, a);
    LimitClavicleDeviation(aSk, kRef, kClavL, 30.0f, 0.0f, b);
    Check(std::fabs(AngleDeg(kRef, a.b[kClavL].r) - 30.0f) < 0.02f && std::fabs(AngleDeg(kRef, b.b[kClavL].r) - 30.0f) < 0.02f &&
              AngleDeg(a.b[kClavL].r, b.b[kClavL].r) < 0.02f,
          "E: 70 deg stored as q and -q both end at the same rotation 30 deg from the reference");
    // reference stored in the opposite hemisphere
    Pose c = WithClavDeviation(kRef, kAxis, 8.0f);
    const Pose before = c;
    LimitClavicleDeviation(aSk, math::QuatNeg(kRef), kClavL, 30.0f, 0.0f, c);
    Check(std::memcmp(&before, &c, sizeof(Pose)) == 0, "E: 8 deg against a -ref reference is untouched");
}

// ---- F) invalid input ----------------------------------------------------------------------------------------------------------
void Invalid(const Skeleton& aSk)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto untouched = [&](const char* aWhat, int aClav, const math::Quat& aRefQ, float aLimit, float aSoft, bool aPoisonPose) {
        Pose p = WithClavDeviation(kRef, kAxis, 70.0f);
        if (aPoisonPose)
            p.b[kClavL].r.x = nan;
        const Pose before = p;
        const float r = LimitClavicleDeviation(aSk, aRefQ, aClav, aLimit, aSoft, p);
        Check(r == 0.0f && std::memcmp(&before, &p, sizeof(Pose)) == 0, F("F: %s leaves the pose untouched (returned %.1f)", aWhat, static_cast<double>(r)));
    };
    untouched("bone index -1", -1, kRef, 30.0f, 0.0f, false);
    untouched("bone index >= n", kN, kRef, 30.0f, 0.0f, false);
    untouched("NaN limit", kClavL, kRef, nan, 0.0f, false);
    untouched("negative limit", kClavL, kRef, -5.0f, 0.0f, false);
    untouched("NaN soft", kClavL, kRef, 30.0f, nan, false);
    untouched("NaN reference", kClavL, math::Quat{nan, 0.0f, 0.0f, 1.0f}, 30.0f, 0.0f, false);
    untouched("NaN clavicle rotation", kClavL, kRef, 30.0f, 0.0f, true);
    // limit above 180 / infinite never changes anything
    Pose p = WithClavDeviation(kRef, kAxis, 170.0f);
    const Pose before = p;
    LimitClavicleDeviation(aSk, kRef, kClavL, 200.0f, 0.0f, p);
    Check(std::memcmp(&before, &p, sizeof(Pose)) == 0, "F: limit 200 deg never engages");
    auto scratch = std::make_unique<LayerScratch>();
    scratch->armL.n = scratch->armR.n = kN;
    Pose q = before;
    const ClavicleLimitResult r = LimitMeshArmClavicles(aSk, *scratch, -1, 99, 30.0f, 0.0f, q);
    Check(!r.applied && std::memcmp(&before, &q, sizeof(Pose)) == 0, "F: LimitMeshArmClavicles with invalid bone indices is a no-op");
}

// ---- G) synthetic ComposeLayers ------------------------------------------------------------------------------------------------
struct Comp
{
    Skeleton sk;
    Regions regions;
    Pose B, L, OP, out;
    std::unique_ptr<LayerScratch> scratch = std::make_unique<LayerScratch>();
};
Curves AimCurves()
{
    Curves c;
    for (int i = 0; i < kCurveCount; ++i)
        c.Set(i, 0.0f);
    c.Set(kSpine, 1.0f);
    c.Set(kSpineAdd, 1.0f); // Spine_Add 1: the spine follows the locomotion
    c.Set(ov::kArmL, 1.0f);   // Arm_X 1, Arm_X_Add 0, Arm_X_LS 0: mesh-space arm layer
    c.Set(ov::kArmR, 1.0f);
    c.Set(kLegs, 1.0f);
    c.Set(kPelvis, 1.0f);
    return c;
}
void Compose(Comp& g, float aSpine3TurnDeg)
{
    MakeSkeleton(g.sk);
    BuildRegions(g.sk, [&](int i) { return std::string_view(kNames[i]); }, g.regions);
    g.B.n = g.L.n = g.OP.n = g.out.n = kN;
    for (int i = 0; i < kN; ++i)
        g.B.b[i] = g.L.b[i] = g.OP.b[i] = Xf{};
    // overlay pose: arms raised
    g.OP.b[kClavL].r = AxisAngle({0.0f, 1.0f, 0.0f}, 10.0f);
    g.OP.b[kBArmL].r = AxisAngle({0.0f, 0.0f, 1.0f}, 50.0f);
    g.OP.b[kForeL].r = AxisAngle({1.0f, 0.0f, 0.0f}, 40.0f);
    g.OP.b[kClavR].r = AxisAngle({0.0f, 1.0f, 0.0f}, -10.0f);
    // locomotion: Spine3 turned relative to the base pose B
    g.L.b[kSpine3].r = AxisAngle({0.0f, 0.0f, 1.0f}, aSpine3TurnDeg);
    LayerInputs in;
    in.L = &g.L;
    in.B = &g.B;
    in.OP = &g.OP;
    in.curves = AimCurves();
    ComposeLayers(g.sk, g.regions, in, *g.scratch, g.out);
}
void Synthetic()
{
    {
        Comp g;
        Compose(g, 70.0f);
        const float dev = AngleDeg(g.scratch->armL.b[kClavL].r, g.out.b[kClavL].r);
        Check(std::fabs(dev - 70.0f) < 0.5f, F("G: mesh-space arm layer puts the 70 deg Spine3 turn into the clavicle (deviation %.2f deg)", static_cast<double>(dev)));
        math::Quat m0[kN], m1[kN];
        Mesh(g.sk, g.out, m0);
        Pose limited = g.out;
        const ClavicleLimitResult r = LimitMeshArmClavicles(g.sk, *g.scratch, kClavL, kClavR, kMeshArmClavicleLimitDeg, kMeshArmClavicleSoftDeg, limited);
        Mesh(g.sk, limited, m1);
        const float after = AngleDeg(g.scratch->armL.b[kClavL].r, limited.b[kClavL].r);
        float worstSub = 0.0f;
        for (int i : {kBArmL, kForeL, kBHandL, kWingL})
            worstSub = std::max(worstSub, AngleDeg(m0[i], m1[i]));
        Check(r.applied && std::fabs(r.leftDeg - 70.0f) < 0.5f && after < 45.0f && after > 30.0f,
              F("G: limited clavicle deviation %.2f deg (was %.2f, soft knee 30..45)", static_cast<double>(after), static_cast<double>(r.leftDeg)));
        Check(worstSub < 0.01f, F("G: arm chain keeps its composed mesh rotation (worst %.4f deg)", static_cast<double>(worstSub)));
        // the right clavicle sees the same Spine3 turn (mirrored reference): limited as well
        Check(AngleDeg(g.scratch->armR.b[kClavR].r, limited.b[kClavR].r) < 45.0f, "G: right clavicle bounded too");
        // below ComposeLayers' no-overlay early-out the caller does not limit (MotionPose: li.OP && li.B): nothing to test here
    }
    {
        Comp g;
        Compose(g, 8.0f);
        Pose limited = g.out;
        const ClavicleLimitResult r = LimitMeshArmClavicles(g.sk, *g.scratch, kClavL, kClavR, kMeshArmClavicleLimitDeg, kMeshArmClavicleSoftDeg, limited);
        Check(!r.applied && std::memcmp(&limited, &g.out, sizeof(Pose)) == 0 && std::fabs(r.leftDeg - 8.0f) < 0.3f,
              F("G: calm pose (Spine3 turn 8 deg, deviation %.2f) is bitwise unchanged", static_cast<double>(r.leftDeg)));
    }
}

// ---- H) real data --------------------------------------------------------------------------------------------------------------
int FindClip(const motion::MotionDatabase& aDb, std::string_view aName)
{
    for (uint32_t i = 0; i < aDb.NumClips(); ++i)
        if (aDb.ClipName(i) == aName)
            return static_cast<int>(i);
    return -1;
}
#ifdef UTPP_CLAVLIMIT_SURVEY
void Survey(const motion::MotionDatabase& aDb, const OverlayFile& aOv, const StyleFamilyData& aStyle);
#endif
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
    Check(dbOk && ovOk && stOk, F("H: load woman db / overlay / style (%s)", err.c_str()));
    if (!dbOk || !ovOk || !stOk)
        return;
#ifdef UTPP_CLAVLIMIT_SURVEY
    Survey(*db, *ovf, style);
#endif
    const uint32_t n = db->NumBones();
    struct Setup
    {
        const char* clip;
        bool moving;
    };
    const Setup setups[] = {{"M_Neutral_Stand_Idle_Loop", false}, {"M_Neutral_Run_Loop_F", true}, {"M_Neutral_Walk_Loop_F", true}};
    for (const Setup& s : setups)
        if (FindClip(*db, s.clip) < 0)
            Check(false, F("H: clip %s found", s.clip));
    for (PoseOverlay overlay : {PoseOverlay::Pistol1H, PoseOverlay::Pistol2H, PoseOverlay::Rifle})
        for (const Setup& s : setups)
        {
            const int clip = FindClip(*db, s.clip);
            if (clip < 0)
                continue;
            auto ev = std::make_unique<motionpose::PoseEvaluator>();
            if (!ev->Init(db.get(), ovf.get(), &style, 2, err))
            {
                Check(false, F("H: evaluator init (%s)", err.c_str()));
                return;
            }
            motionpose::OverlayFrameInputs in;
            in.machine.overlay = overlay;
            in.machine.style = Style::Feminine;
            in.machine.wantsToAim = true;
            in.machine.wantsReady = true;
            in.machine.moving = s.moving;
            in.bodyWoman = true;
            motion::MatchResult m;
            m.clip = clip;
            m.clipTime = 0.5f;
            std::vector<BoneTransform> out(n);
            float maxL = 0.0f, maxR = 0.0f;
            int applied = 0;
            const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
            const float len = static_cast<float>(db->Clip(static_cast<uint32_t>(clip)).numFrames - 1) / fps;
            for (int f = 0; f < 240; ++f)
            {
                m.clipTime = std::fmod(0.5f + static_cast<float>(f) / 60.0f, std::max(len, 0.1f));
                motionpose::FrameInfo info;
                ev->Evaluate(1.0f / 60.0f, m, in, out.data(), n, &info);
                if (f >= 90) // settled aim
                {
                    maxL = std::max(maxL, info.clavicleLimit.leftDeg);
                    maxR = std::max(maxR, info.clavicleLimit.rightDeg);
                    applied += info.clavicleLimit.applied ? 1 : 0;
                }
            }
            Check(applied == 0, F("H: overlay %d on %s: runtime limit does not engage (clavicle deviation max L %.1f / R %.1f deg, runtime limit %.0f)",
                                  static_cast<int>(overlay), s.clip, static_cast<double>(maxL), static_cast<double>(maxR),
                                  static_cast<double>(kMeshArmClavicleRuntimeLimitDeg)));
        }
}

#ifdef UTPP_CLAVLIMIT_SURVEY
// Survey (information only, build with /DUTPP_CLAVLIMIT_SURVEY): clavicle deviation of the settled aim pose on every loop clip of the database.
void Survey(const motion::MotionDatabase& aDb, const OverlayFile& aOv, const StyleFamilyData& aStyle)
{
    const uint32_t n = aDb.NumBones();
    struct Row { std::string clip; int overlay; float maxL, maxR, p90; };
    std::vector<Row> rows;
    int notAiming = 0;
    std::string err;
    for (PoseOverlay overlay : {PoseOverlay::Pistol1H, PoseOverlay::Pistol2H, PoseOverlay::Rifle})
        for (uint32_t c = 0; c < aDb.NumClips(); ++c)
        {
            const std::string name(aDb.ClipName(c));
            if (name.find("_Loop") == std::string::npos)
                continue;
            for (int crouch = 0; crouch < 2; ++crouch)
            {
                if ((name.find("Crouch") != std::string::npos) != (crouch != 0))
                    continue;
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                if (!ev->Init(&aDb, &aOv, &aStyle, 2, err))
                    return;
                motionpose::OverlayFrameInputs in;
                in.machine.overlay = overlay;
                in.machine.style = Style::Feminine;
                in.machine.wantsToAim = true;
                in.machine.wantsReady = true;
                in.machine.moving = name.find("Idle") == std::string::npos;
                in.machine.crouch = crouch != 0;
                in.machine.wantsToSprint = in.machine.sprinting = name.find("Sprint") != std::string::npos; // the machine blocks Aim while sprinting
                in.machine.wantsToSprint = in.machine.sprinting = name.find("Sprint") != std::string::npos; // the machine blocks Aim while sprinting
                in.bodyWoman = true;
                motion::MatchResult m;
                m.clip = static_cast<int>(c);
                std::vector<BoneTransform> out(n);
                const float fps = aDb.Header().fps > 0.0f ? aDb.Header().fps : 30.0f;
                const float len = static_cast<float>(aDb.Clip(c).numFrames - 1) / fps;
                std::vector<float> dev;
                float mx = 0.0f, mr = 0.0f;
                for (int f = 0; f < 240; ++f)
                {
                    m.clipTime = std::fmod(0.5f + static_cast<float>(f) / 60.0f, std::max(len, 0.1f));
                    motionpose::FrameInfo info;
                    ev->Evaluate(1.0f / 60.0f, m, in, out.data(), n, &info);
                    if (f >= 90)
                    {
                        mx = std::max(mx, info.clavicleLimit.leftDeg);
                        mr = std::max(mr, info.clavicleLimit.rightDeg);
                        dev.push_back(std::max(info.clavicleLimit.leftDeg, info.clavicleLimit.rightDeg));
                    }
                }
                if (ev->Machine().aim != ov::AimState::Aiming)
                    ++notAiming;
                std::sort(dev.begin(), dev.end());
                rows.push_back({name, static_cast<int>(overlay), mx, mr, dev.empty() ? 0.0f : dev[dev.size() * 9 / 10]});
            }
        }
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return std::max(a.maxL, a.maxR) > std::max(b.maxL, b.maxR); });
    for (size_t i = 0; i < rows.size() && i < 25; ++i)
        std::printf("SURVEY overlay %d %-48s maxL %.1f maxR %.1f p90 %.1f\n", rows[i].overlay, rows[i].clip.c_str(), static_cast<double>(rows[i].maxL),
                    static_cast<double>(rows[i].maxR), static_cast<double>(rows[i].p90));
    std::printf("SURVEY rows %zu, not Aiming: %d\n", rows.size(), notAiming);
}
#endif
} // namespace

static_assert(kMeshArmClavicleRuntimeLimitDeg <= 60.0f && kMeshArmClavicleRuntimeLimitDeg >= kMeshArmClavicleLimitDeg, "runtime limit: engages only the balloon range (slides, heavy landings)");

int main(int argc, char** argv)
{
    Skeleton sk;
    MakeSkeleton(sk);
    BelowLimit(sk);
    HardClamp(sk);
    SoftKnee(sk);
    Children(sk);
    Hemisphere(sk);
    Invalid(sk);
    Synthetic();
    if (argc >= 4)
        RealData(argv[1], argv[2], argv[3]);
    else
        std::printf("SKIP H: pass <woman v2 db> <woman overlay> <woman style weights> for the real-data check\n");
    std::printf("clavicle limit: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

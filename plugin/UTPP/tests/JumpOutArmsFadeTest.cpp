// r9 jump-out arms: in the flight of a vehicle jump-out the arm / hand layers of the weapon pose must follow the jump clip.
// Game test r5a (move case): the pistol arm straight and horizontal in the flight; the Pistol2H arm layers ramp 0.14 -> 1.00 with aimW 0.
//   A) ov::FadeArmsInJumpOutFlight: arm / hand curves and the hand IK weights (Enable_HandIK_L / R) * (1 - w); weight 0 leaves every curve
//      bit-identical; spine / head / legs / pelvis curves are never touched;
//   B) ov::JumpOutArmsFade: >= 0.99 within 0.11 s of the first airborne frame at 30 / 60 / 144 Hz, bounded step (a 0.1 s hitch frame does not snap
//      the arms), never raised without a jump-out flight (ground, aiming, a normal weapon jump drive it with false), falls to 0 within 0.45 s of
//      the touchdown without a snap (>= 0.5 left 0.1 s after the touchdown); the fall is capped per frame too (a 0.2 s hitch frame at the touchdown
//      drops the weight by at most kJumpOutArmsMaxFallStep); a weapon request (aim / shot / reload) in the flight returns the weapon arms (the weight is
//      0 within 0.15 s, never raised while the request lasts) at 30 / 60 / 144 Hz.
//   C) (optional arguments) the PoseEvaluator on the real woman data: jump clips under the Pistol2H pose with machine.jumpOutArmW 1 report arm /
//      hand layers 0 and compose the clip arms (arm deviation from the unarmed pose median < 0.5 deg, max < 6 deg), spine / head / legs / pelvis curves identical to
//      the run with jumpOutArmW 0, whose weapon arms are far from the clip; with non-null grip targets (left grip, right hand target) the grip IK
//      solves at jumpOutArmW 0 and does not run at 1 (the hands are not pulled over the clip arms).
// Usage: utpp_jumpout_arms_fade_test [<woman v2 db> <woman overlay> <woman style weights>]
// Mutations UTPP_JOARMS_MUTATION 1 (the fade does nothing), 2 (the arms snap back at the touchdown), 3 (no per-frame cap on the rise), 4 (MotionPose
// does not apply jumpOutArmW), 5 (no per-frame cap on the fall), 6 (the hand IK weights are not faded) must each print a FAIL line.
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>
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
    char b[512];
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
bool IsArmCurve(int i) { return i == kArmL || i == kArmR || i == kHandL || i == kHandR || i == kHandIKL || i == kHandIKR; }
} // namespace

// ---- C) the PoseEvaluator on real data
namespace
{
int FindBoneByName(const motion::MotionDatabase& aDb, std::string_view aName)
{
    for (uint32_t i = 0; i < aDb.NumBones(); ++i)
        if (aDb.BoneName(i) == aName)
            return static_cast<int>(i);
    return -1;
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
    Check(dbOk && ovOk && stOk, F("C: load woman db / overlay / style (%s)", err.c_str()));
    if (!dbOk || !ovOk || !stOk)
        return;
    const uint32_t n = db->NumBones();
    const char* armNames[] = {"LeftArm", "RightArm", "LeftForeArm", "RightForeArm"};
    int arm[4];
    bool bones = true;
    for (int i = 0; i < 4; ++i)
    {
        arm[i] = FindBoneByName(*db, armNames[i]);
        bones = bones && arm[i] >= 0;
    }
    Check(bones, "C: arm bones found");
    if (!bones)
        return;
    const float dt = 1.0f / 60.0f;
    std::vector<float> devArmed, devFaded, devGrip;
    float worstCurve = 0.0f, otherCurveDiff = 0.0f, armCurveW0 = 1.0f, worstIk = 0.0f;
    int samples = 0, gripSolvedW0 = 0, gripSolvedW1 = 0;
    gik::Grip leftGrip; // WeaponRight-space target of the left hand and a model-space target of the right hand: non-null in the grip runs
    leftGrip.valid = true;
    leftGrip.t = {0.04f, 0.02f, 0.0f};
    ov::Xf rightTarget;
    rightTarget.t = {0.15f, 0.35f, 1.20f};
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string name(db->ClipName(c));
        if (name.find("Jump_Loop_Fall") == std::string::npos && name.find("Jump_F_Start_Sprint") == std::string::npos &&
            name.find("Jump_F_Start_Stand") == std::string::npos)
            continue;
        const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
        const float dur = static_cast<float>(db->Clip(c).numFrames - 1) / fps;
        for (float t = 0.0f; t <= dur; t += 0.15f)
        {
            motion::MatchResult m;
            m.clip = static_cast<int32_t>(c);
            m.clipTime = t;
            std::vector<BoneTransform> pure(n), w0(n), w1(n);
            motionpose::FrameInfo i0, i1, ip;
            auto settle = [&](PoseOverlay aOverlay, float aW, std::vector<BoneTransform>& aOut, motionpose::FrameInfo& aInfo, bool aGrip = false) {
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                std::string e;
                if (!ev->Init(db.get(), ovf.get(), &style, 2, e))
                    return false;
                motionpose::OverlayFrameInputs in;
                in.machine.overlay = aOverlay;
                in.machine.style = Style::Feminine;
                in.bodyWoman = true;
                in.machine.wantsReady = aOverlay != PoseOverlay::Default;
                in.machine.jumpOutArmW = aW;
                if (aGrip)
                {
                    in.leftGrip = &leftGrip;
                    in.rightTarget = &rightTarget;
                }
                bool ok = true;
                for (int f = 0; f < 150; ++f)
                    ok = ev->Evaluate(dt, m, in, aOut.data(), n, &aInfo) && ok;
                return ok;
            };
            if (!settle(PoseOverlay::Default, 0.0f, pure, ip) || !settle(PoseOverlay::Pistol2H, 0.0f, w0, i0) || !settle(PoseOverlay::Pistol2H, 1.0f, w1, i1))
                continue;
            ++samples;
            {
                // grip targets present: solved by the grip IK at jumpOutArmW 0, not run at 1
                std::vector<BoneTransform> g0(n), g1(n);
                motionpose::FrameInfo gi0, gi1;
                if (settle(PoseOverlay::Pistol2H, 0.0f, g0, gi0, true) && settle(PoseOverlay::Pistol2H, 1.0f, g1, gi1, true))
                {
                    gripSolvedW0 += (gi0.grip.left.ok || gi0.grip.right.ok) ? 1 : 0;
                    gripSolvedW1 += (gi1.grip.left.ok || gi1.grip.right.ok) ? 1 : 0;
                    worstIk = std::fmax(worstIk, std::fmax(std::fabs(gi1.curves[kHandIKL]), std::fabs(gi1.curves[kHandIKR])));
                    for (const int a : arm)
                        devGrip.push_back(AngleDeg(pure[a].q, g1[a].q));
                }
            }
            armCurveW0 = std::fmin(armCurveW0, i0.curves[kArmR]);
            for (const int a : {kArmL, kArmR, kHandL, kHandR})
                worstCurve = std::fmax(worstCurve, std::fabs(i1.curves[a]));
            for (int k = 0; k < kCurveCount; ++k)
                if (!IsArmCurve(k))
                    otherCurveDiff = std::fmax(otherCurveDiff, std::fabs(i1.curves[k] - i0.curves[k]));
            for (const int a : arm)
            {
                devArmed.push_back(AngleDeg(pure[a].q, w0[a].q));
                devFaded.push_back(AngleDeg(pure[a].q, w1[a].q));
            }
        }
    }
    auto med = [](std::vector<float> v) {
        if (v.empty())
            return 0.0f;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    const float mx1 = devFaded.empty() ? 0.0f : *std::max_element(devFaded.begin(), devFaded.end());
    std::printf("INFO C: %d jump clip samples; Pistol2H arm layer min %.2f; arm deviation from the unarmed pose: jumpOutArmW 0 median %.1f deg, jumpOutArmW 1 median %.3f max %.3f deg; "
                "arm / hand curves at 1 max %.3f, other curves max diff %.6f\n",
                samples, static_cast<double>(armCurveW0), static_cast<double>(med(devArmed)), static_cast<double>(med(devFaded)), static_cast<double>(mx1),
                static_cast<double>(worstCurve), static_cast<double>(otherCurveDiff));
    Check(samples >= 10, F("C: enough jump clip samples (%d)", samples));
    Check(med(devArmed) > 10.0f, F("C: premise: the Pistol2H weapon arms are far from the jump clip arms (median %.1f deg)", static_cast<double>(med(devArmed))));
    Check(worstCurve == 0.0f, F("C: jumpOutArmW 1 reports the arm / hand layers 0 through the PoseEvaluator (max %.3f)", static_cast<double>(worstCurve)));
    // measured 2026-11-04 on the woman data: median 0.000, max 4.217 deg (the later arm stages on a few clip samples); weapon arms: median 57.8 deg
    Check(med(devFaded) < 0.5f && mx1 < 6.0f, F("C: jumpOutArmW 1 composes the clip arms (median %.3f, max %.3f deg from the unarmed pose)", static_cast<double>(med(devFaded)), static_cast<double>(mx1)));
    Check(otherCurveDiff == 0.0f, F("C: spine / head / legs / pelvis curves are identical with jumpOutArmW 0 and 1 (max diff %.6f)", static_cast<double>(otherCurveDiff)));
    std::printf("INFO C: grip targets: IK solved in %d / %d samples at jumpOutArmW 0, %d at 1; hand IK curves at 1 max %.3f; arm deviation with grip targets at 1 median %.3f max %.3f deg\n",
                gripSolvedW0, samples, gripSolvedW1, static_cast<double>(worstIk), static_cast<double>(med(devGrip)),
                static_cast<double>(devGrip.empty() ? 0.0f : *std::max_element(devGrip.begin(), devGrip.end())));
    Check(gripSolvedW0 >= samples / 2 && gripSolvedW0 > 0, F("C: premise: the grip IK solves toward the targets at jumpOutArmW 0 (%d of %d samples)", gripSolvedW0, samples));
    Check(gripSolvedW1 == 0 && worstIk == 0.0f, F("C: jumpOutArmW 1 turns the grip IK off with non-null grip targets (solved in %d samples, hand IK curves max %.3f)", gripSolvedW1, static_cast<double>(worstIk)));
    Check(!devGrip.empty() && *std::max_element(devGrip.begin(), devGrip.end()) < 6.0f, "C: jumpOutArmW 1 with grip targets keeps the clip arms (max < 6 deg from the unarmed pose)");
}
} // namespace

int main(int argc, char** argv)
{
    // ---- A) the curves
    {
        Curves c = AllOnes(), ref = AllOnes();
        FadeArmsInJumpOutFlight(0.0f, c);
        Check(std::memcmp(c.v, ref.v, sizeof(c.v)) == 0, "A: weight 0 leaves every curve bit-identical");
        c = AllOnes();
        FadeArmsInJumpOutFlight(1.0f, c);
        bool armsZero = true, restKept = true;
        for (int i = 0; i < kCurveCount; ++i)
        {
            if (IsArmCurve(i))
                armsZero = armsZero && c.v[i] == 0.0f;
            else
                restKept = restKept && c.v[i] == 1.0f;
        }
        Check(armsZero, "A: weight 1 takes the arm and hand layers (ArmL / ArmR / HandL / HandR) off the weapon pose");
        Check(restKept, "A: weight 1 leaves spine / head / legs / pelvis / add / local-space curves untouched");
        Check(c.v[kHandIKL] == 0.0f && c.v[kHandIKR] == 0.0f, "A: weight 1 takes the hand IK weights off (the grip IK does not pull the hands over the clip arms)");
        c = AllOnes();
        FadeArmsInJumpOutFlight(0.5f, c);
        Check(std::fabs(c.v[kArmR] - 0.5f) < 1e-6f && std::fabs(c.v[kHandL] - 0.5f) < 1e-6f, "A: weight 0.5 halves the arm layers");
        Check(std::fabs(c.v[kHandIKL] - 0.5f) < 1e-6f && std::fabs(c.v[kHandIKR] - 0.5f) < 1e-6f, "A: weight 0.5 halves the hand IK weights");
        c = AllOnes();
        FadeArmsInJumpOutFlight(7.0f, c);
        Check(c.v[kArmR] == 0.0f && c.v[kLegs] == 1.0f, "A: a weight above 1 clamps");
    }
    // ---- B) the weight
    for (const float hz : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / hz;
        JumpOutArmsFade fade;
        float t = 0.0f, prev = 0.0f, maxStep = 0.0f;
        bool reached = false;
        for (int i = 0; i < int(hz * 0.11f) + 1; ++i)
        {
            const float w = fade.Step(true, dt);
            maxStep = std::fmax(maxStep, w - prev);
            prev = w;
            t += dt;
            if (t >= 0.11f - 0.5f * dt && w >= 0.99f)
                reached = true;
        }
        Check(reached, F("B %g Hz: >= 0.99 within 0.11 s of the first airborne frame (w %.3f)", double(hz), double(prev)));
        Check(maxStep <= kJumpOutArmsMaxRiseStep + 1e-6f, F("B %g Hz: rise step <= %.2f per frame (max %.3f)", double(hz), double(kJumpOutArmsMaxRiseStep), double(maxStep)));
        // landing: no snap, gone within 0.45 s
        float wAfter01 = -1.0f, wAfter045 = -1.0f;
        float tl = 0.0f, prevL = fade.w, maxDrop = 0.0f;
        for (int i = 0; i < int(hz * 0.5f); ++i)
        {
            const float w = fade.Step(false, dt);
            maxDrop = std::fmax(maxDrop, prevL - w);
            prevL = w;
            tl += dt;
            if (wAfter01 < 0.0f && tl >= 0.1f - 0.5f * dt)
                wAfter01 = w;
            if (wAfter045 < 0.0f && tl >= 0.45f - 0.5f * dt)
                wAfter045 = w;
        }
        Check(wAfter01 >= 0.5f, F("B %g Hz: >= 0.5 of the fade is left 0.1 s after the touchdown (%.3f)", double(hz), double(wAfter01)));
        Check(wAfter045 <= 1e-6f, F("B %g Hz: the arms are back on the weapon pose 0.45 s after the touchdown (%.3f)", double(hz), double(wAfter045)));
    }
    for (const float hz : {30.0f, 60.0f, 144.0f})
    {
        // a weapon request (aim / shot / reload) in the flight: the weight is 0 within 0.15 s and the rise is blocked while it lasts
        const float dt = 1.0f / hz;
        JumpOutArmsFade fade;
        for (int i = 0; i < 60; ++i)
            fade.Step(true, dt);
        float t = 0.0f, w = fade.w, maxDrop = 0.0f, wEarly = fade.w;
        const bool upBefore = w >= 0.999f;
        for (int i = 0; i < int(hz * 0.5f); ++i)
        {
            const float prev = w;
            w = fade.Step(true, dt, true);
            maxDrop = std::fmax(maxDrop, prev - w);
            t += dt;
            if (i == 0)
                wEarly = w;
        }
        Check(upBefore && w <= 1e-6f, F("B %g Hz: a weapon request in the flight takes the weight 1 -> 0 and holds it there (%.3f)", double(hz), double(w)));
        Check(wEarly < 1.0f && maxDrop <= kJumpOutArmsMaxFallStep + 1e-6f, F("B %g Hz: the weight starts falling at once (%.3f after one frame), step <= %.2f (max %.3f)", double(hz), double(wEarly), double(kJumpOutArmsMaxFallStep), double(maxDrop)));
        JumpOutArmsFade ctl;
        for (int i = 0; i < int(hz * 0.6f); ++i)
            ctl.Step(true, dt);
        Check(ctl.w >= 0.999f, F("B %g Hz: plain flight (no request) still fades the arms (%.3f)", double(hz), double(ctl.w)));
        JumpOutArmsFade quick;
        for (int i = 0; i < 60; ++i)
            quick.Step(true, dt);
        float tq = 0.0f, wq = 1.0f;
        for (int i = 0; i < int(hz * 0.25f); ++i)
        {
            wq = quick.Step(true, dt, true);
            tq += dt;
        }
        Check(wq <= 1e-6f, F("B %g Hz: the weapon arms are back within 0.25 s of the request (%.3f after %.3f s)", double(hz), double(wq), double(tq)));
    }
    {
        // a weapon request that ends in the flight lets the fade rise again
        JumpOutArmsFade fade;
        for (int i = 0; i < 30; ++i)
            fade.Step(true, 1.0f / 60.0f);
        for (int i = 0; i < 20; ++i)
            fade.Step(true, 1.0f / 60.0f, true);
        const float low = fade.w;
        for (int i = 0; i < 12; ++i)
            fade.Step(true, 1.0f / 60.0f);
        Check(low <= 1e-6f && fade.w >= 0.999f, F("B: after the request ends the flight fades the arms again (%.3f -> %.3f)", double(low), double(fade.w)));
    }
    {
        // a 0.2 s hitch frame at the touchdown does not snap the arms back
        JumpOutArmsFade fade;
        for (int i = 0; i < 30; ++i)
            fade.Step(true, 1.0f / 60.0f);
        const float w = fade.Step(false, 0.2f);
        Check(w >= 1.0f - kJumpOutArmsMaxFallStep - 1e-6f, F("B: one 0.2 s frame at the touchdown lowers the weight by at most %.2f (%.3f)", double(kJumpOutArmsMaxFallStep), double(w)));
        const float wReq = fade.Step(true, 0.2f, true);
        Check(wReq >= w - kJumpOutArmsMaxFallStep - 1e-6f, F("B: one 0.2 s frame with a weapon request lowers the weight by at most %.2f (%.3f -> %.3f)", double(kJumpOutArmsMaxFallStep), double(w), double(wReq)));
    }
    {
        // a 0.1 s hitch frame at the launch does not snap the arms
        JumpOutArmsFade fade;
        const float w = fade.Step(true, 0.1f);
        Check(w <= kJumpOutArmsMaxRiseStep + 1e-6f, F("B: one 0.1 s frame raises the weight by at most %.2f (%.3f)", double(kJumpOutArmsMaxRiseStep), double(w)));
    }
    {
        // ground / aiming / a normal weapon jump: the driver passes false, the weight stays exactly 0
        JumpOutArmsFade fade;
        float worst = 0.0f;
        for (int i = 0; i < 240; ++i)
            worst = std::fmax(worst, fade.Step(false, 1.0f / 60.0f));
        Check(worst == 0.0f, "B: never raised without a jump-out flight (240 frames on the ground, weight stays exactly 0)");
        Curves c = AllOnes(), ref = AllOnes();
        FadeArmsInJumpOutFlight(fade.w, c);
        Check(std::memcmp(c.v, ref.v, sizeof(c.v)) == 0, "B: the ground curves are bit-identical after the fade step");
        // a non-finite or zero dt moves nothing
        const float w0 = fade.Step(true, 0.0f), w1 = fade.Step(true, std::nanf(""));
        Check(w0 == 0.0f && w1 == 0.0f, "B: dt 0 / NaN moves nothing");
    }
    if (argc >= 4)
        RealData(argv[1], argv[2], argv[3]);
    else
        std::printf("SKIP C: pass <woman v2 db> <woman overlay> <woman style weights> for the real-data check\n");
    std::printf("jump-out arms fade: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

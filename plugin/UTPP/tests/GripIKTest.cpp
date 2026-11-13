// UTPP motion stage 10.6 - offline test of src/mm/GripIK.hpp (two-bone weapon grip hand IK after the overlay layering).
// Real CP rig and poses from the baked overlay files (motion-derived, outside git).
// Usage: utpp_mm_gripik_test <utpp_overlays_woman.utppov> <utpp_overlays_man.utppov>
// Per rig, on the Rifle Stand Aim overlay pose (left and right arm):
//  1) reachable targets (400 random, 0.30..0.90 of the arm length, random hand rotation): hand within 1 mm, hand
//     rotation within 0.05 deg, bone lengths kept, bones outside the arm chain subtree untouched;
//  2) unreachable targets (200 random, 1.0..3.0 L) and too-close targets (50, 0..0.1 L): clamped (hand inside
//     maxReach * L, outside minReach * L), hand on the shoulder-target ray (0.5 deg), elbow interior angle >= 5 deg,
//     elbow on the input pole side (no flip), all finite;
//  3) elbow plane continuity: 100 random targets: the new bend plane contains the input pole (angle <= 0.5 deg), elbow
//     on the pole side; random walk of 100 steps (1 cm each): elbow step <= 6 x target step;
//  4) weight 0 = input pose bit for bit (and skip flag, absent curve);
//  5) weight blend 0..1 in 100 steps: hand path continuous (max step <= 3 x mean step), distance to target
//     non-increasing, weight 1 = full solve;
//  6) real overlay pose: left-hand target = its own left hand + 3 cm (3 directions): reached within 1 mm, shoulder /
//     elbow rotation change small (<= 8 deg), elbow moves <= 4 cm, only LeftArm / LeftForeArm / LeftHand locals change,
//     no bone outside the left arm subtree moves; grip cache from the same pose -> ApplyGripIK is a no-op (<= 0.1 mm);
//     Enable_HandIK_L of the pose is reported;
//  7) grip table parser: valid table parsed, malformed lines rejected (fail-closed).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>

#include "../src/mm/GripIK.hpp"
#include "../src/mm/OverlayFile.hpp"

using namespace utpp::mm;
using namespace utpp::mm::gik;
using math::Quat;
using math::Vec3;

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
static std::string F(const char* aFmt, double a, double b = 0.0, double c = 0.0)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c);
    return buf;
}

struct Fk
{
    Vec3 p[ov::kMaxBones];
    Quat r[ov::kMaxBones];
};
static void DoFk(const Skeleton& aSk, const Pose& aPose, Fk& aOut) { ov::MeshPositions(aSk, aPose, aOut.p, aOut.r); }

static Vec3 RandUnit(std::mt19937& aRng)
{
    std::normal_distribution<float> n(0.0f, 1.0f);
    for (;;)
    {
        Vec3 v{n(aRng), n(aRng), n(aRng)}, u;
        if (Norm(v, u))
            return u;
    }
}
static Quat RandQuat(std::mt19937& aRng)
{
    std::normal_distribution<float> n(0.0f, 1.0f);
    return math::QuatNormalize(Quat{n(aRng), n(aRng), n(aRng), n(aRng)});
}
static float AngDeg(Vec3 a, Vec3 b)
{
    Vec3 u, v;
    if (!Norm(a, u) || !Norm(b, v))
        return 0.0f;
    return std::atan2(Len(math::Cross(u, v)), math::Dot(u, v)) * 180.0f / math::kPi;
}
static bool InSubtree(const Skeleton& aSk, int aRoot, int aBone) { return aSk.IsAncestorOrSelf(aRoot, aBone); }

static void TestArm(const Skeleton& sk, const Pose& in, const ArmChain& c, const char* tag)
{
    const Params prm;
    Fk f0;
    DoFk(sk, in, f0);
    const Vec3 a = f0.p[c.upper], b = f0.p[c.lower], h = f0.p[c.hand];
    const float lab = Len(b - a), lbc = Len(h - b), L = lab + lbc;
    Vec3 ac, pole;
    Norm(h - a, ac);
    Norm((b - a) - ac * math::Dot(b - a, ac), pole);
    std::printf("[%s] arm L=%.4f m (upper %.4f, lower %.4f), input reach %.3f L\n", tag, L, lab, lbc, Len(h - a) / L);
    std::mt19937 rng(1234u);
    std::uniform_real_distribution<float> u01(0.0f, 1.0f);
    auto pose = std::make_unique<Pose>();
    auto fk = std::make_unique<Fk>();

    // 1) reachable
    {
        float maxErr = 0.0f, maxRot = 0.0f, maxLen = 0.0f, maxOther = 0.0f;
        int okCount = 0;
        for (int i = 0; i < 400; ++i)
        {
            const Vec3 t = a + RandUnit(rng) * (L * (0.30f + 0.60f * u01(rng)));
            const Quat q = RandQuat(rng);
            *pose = in;
            Result r;
            okCount += SolveArm(sk, *pose, c, t, &q, 1.0f, prm, r) && r.ok && !r.clamped ? 1 : 0;
            DoFk(sk, *pose, *fk);
            maxErr = std::max(maxErr, Len(fk->p[c.hand] - t));
            maxRot = std::max(maxRot, ov::AngleDeg(fk->r[c.hand], q));
            maxLen = std::max({maxLen, std::fabs(Len(fk->p[c.lower] - fk->p[c.upper]) - lab),
                               std::fabs(Len(fk->p[c.hand] - fk->p[c.lower]) - lbc)});
            for (int k = 0; k < sk.n; ++k)
                if (!InSubtree(sk, c.upper, k))
                    maxOther = std::max(maxOther, Len(fk->p[k] - f0.p[k]));
        }
        std::printf("[%s] reachable 400: ok %d, max hand err %.6f m, max rot err %.4f deg, max len err %.2e m, "
                    "max move outside chain %.2e m\n",
                    tag, okCount, maxErr, maxRot, maxLen, maxOther);
        Check(okCount == 400, std::string(tag) + ": reachable all ok, unclamped");
        Check(maxErr <= 0.001f, std::string(tag) + F(": reachable hand err %.6f <= 0.001 m", maxErr));
        Check(maxRot <= 0.05f, std::string(tag) + F(": reachable hand rot err %.4f <= 0.05 deg", maxRot));
        Check(maxLen <= 1e-5f, std::string(tag) + F(": bone lengths kept (%.2e m)", maxLen));
        Check(maxOther == 0.0f, std::string(tag) + F(": bones outside the arm untouched (%.2e m)", maxOther));
    }
    // 2) unreachable / too close
    {
        float maxOut = 0.0f, minIn = 1e9f, maxRay = 0.0f, minElbow = 180.0f, minSide = 1e9f;
        int okCount = 0, finite = 0, clamped = 0;
        for (int i = 0; i < 250; ++i)
        {
            const bool far = i < 200;
            const Vec3 dir = RandUnit(rng);
            const Vec3 t = a + dir * (far ? L * (1.0f + 2.0f * u01(rng)) : L * 0.1f * u01(rng) + 1e-3f);
            *pose = in;
            Result r;
            okCount += SolveArm(sk, *pose, c, t, nullptr, 1.0f, prm, r) && r.ok ? 1 : 0;
            clamped += r.clamped ? 1 : 0;
            DoFk(sk, *pose, *fk);
            bool fin = true;
            for (int k = 0; k < sk.n; ++k)
                fin = fin && math::IsFinite(fk->p[k]) && math::IsFinite(fk->r[k]);
            finite += fin ? 1 : 0;
            const Vec3 hn = fk->p[c.hand], en = fk->p[c.lower];
            const float reach = Len(hn - a) / L;
            if (far)
                maxOut = std::max(maxOut, reach);
            else
                minIn = std::min(minIn, reach);
            maxRay = std::max(maxRay, AngDeg(hn - a, t - a));
            minElbow = std::min(minElbow, AngDeg(a - en, hn - en));
            // elbow side vs input pole projected perpendicular to the new shoulder-hand line
            Vec3 d, pp, ep;
            if (Norm(hn - a, d) && Norm(pole - d * math::Dot(pole, d), pp, 1e-3f) &&
                Norm((en - a) - d * math::Dot(en - a, d), ep))
                minSide = std::min(minSide, math::Dot(ep, pp));
        }
        std::printf("[%s] clamp 200 far + 50 near: ok %d, clamped %d, finite %d, max reach %.4f L (limit %.3f), "
                    "min reach %.4f L (limit %.2f), max ray err %.4f deg, min elbow angle %.2f deg, "
                    "min pole-side cos %.4f\n",
                    tag, okCount, clamped, finite, maxOut, prm.maxReach, minIn, prm.minReach, maxRay, minElbow, minSide);
        Check(okCount == 250 && finite == 250 && clamped == 250, std::string(tag) + ": clamp cases ok, finite, clamped");
        Check(maxOut <= prm.maxReach + 1e-4f, std::string(tag) + F(": far targets reach %.4f <= maxReach", maxOut));
        Check(minIn >= prm.minReach - 1e-4f, std::string(tag) + F(": near targets reach %.4f >= minReach", minIn));
        Check(maxRay <= 0.5f, std::string(tag) + F(": clamped hand on the target ray (%.4f deg)", maxRay));
        Check(minElbow >= 5.0f, std::string(tag) + F(": elbow never straight (min %.2f deg)", minElbow));
        Check(minSide > 0.0f, std::string(tag) + F(": no elbow flip (min pole-side cos %.4f)", minSide));
    }
    // 3) elbow plane continuity
    {
        float maxPlane = 0.0f, minSide = 1e9f, maxRatio = 0.0f;
        for (int i = 0; i < 100; ++i)
        {
            const Vec3 t = a + RandUnit(rng) * (L * (0.35f + 0.55f * u01(rng)));
            *pose = in;
            Result r;
            SolveArm(sk, *pose, c, t, nullptr, 1.0f, prm, r);
            DoFk(sk, *pose, *fk);
            const Vec3 hn = fk->p[c.hand], en = fk->p[c.lower];
            Vec3 d, pp, ep;
            if (!Norm(hn - a, d) || !Norm(pole - d * math::Dot(pole, d), pp, 1e-3f))
                continue; // target along the pole: plane undefined by construction (fallback side)
            Norm((en - a) - d * math::Dot(en - a, d), ep);
            maxPlane = std::max(maxPlane, AngDeg(ep, pp));
            minSide = std::min(minSide, math::Dot(ep, pp));
        }
        // random walk
        Vec3 t = h;
        Fk prev;
        DoFk(sk, in, prev);
        float prevE = 0.0f;
        (void)prevE;
        for (int i = 0; i < 100; ++i)
        {
            Vec3 nt = t + RandUnit(rng) * 0.01f;
            const float dn = Len(nt - a) / L;
            if (dn > 0.9f || dn < 0.35f)
                nt = t - (nt - t); // stay in the reachable band
            *pose = in;
            Result r;
            SolveArm(sk, *pose, c, nt, nullptr, 1.0f, prm, r);
            DoFk(sk, *pose, *fk);
            if (i > 0)
                maxRatio = std::max(maxRatio, Len(fk->p[c.lower] - prev.p[c.lower]) / std::max(Len(nt - t), 1e-6f));
            prev = *fk;
            t = nt;
        }
        std::printf("[%s] elbow plane 100 random: max plane dev %.4f deg, min pole-side cos %.4f; random walk 100 x "
                    "1 cm: max elbow/target step ratio %.3f\n",
                    tag, maxPlane, minSide, maxRatio);
        Check(maxPlane <= 0.5f, std::string(tag) + F(": bend plane keeps the input pole (%.4f deg)", maxPlane));
        Check(minSide > 0.99f, std::string(tag) + F(": elbow on the pole side (%.4f)", minSide));
        Check(maxRatio <= 6.0f, std::string(tag) + F(": elbow continuity ratio %.3f <= 6", maxRatio));
    }
    // 4) weight 0
    {
        const Vec3 t = a + RandUnit(rng) * (0.6f * L);
        *pose = in;
        Result r;
        SolveArm(sk, *pose, c, t, nullptr, 0.0f, prm, r);
        Check(std::memcmp(pose.get(), &in, sizeof(Pose)) == 0, std::string(tag) + ": weight 0 = identity (bitwise)");
        *pose = in;
        SolveArm(sk, *pose, c, t, nullptr, -0.5f, prm, r);
        Check(std::memcmp(pose.get(), &in, sizeof(Pose)) == 0, std::string(tag) + ": weight < 0 = identity");
        *pose = in;
        SolveArm(sk, *pose, c, t, nullptr, std::nanf(""), prm, r);
        Check(std::memcmp(pose.get(), &in, sizeof(Pose)) == 0, std::string(tag) + ": weight NaN = identity");
    }
    // 5) weight blend
    {
        const Vec3 t = h + RandUnit(rng) * 0.10f;
        auto full = std::make_unique<Pose>(in);
        Result r;
        SolveArm(sk, *full, c, t, nullptr, 1.0f, prm, r);
        Fk ff;
        DoFk(sk, *full, ff);
        Vec3 prevH = h;
        float prevD = Len(h - t), maxStep = 0.0f, sumStep = 0.0f, maxInc = 0.0f;
        for (int i = 1; i <= 100; ++i)
        {
            *pose = in;
            SolveArm(sk, *pose, c, t, nullptr, i / 100.0f, prm, r);
            DoFk(sk, *pose, *fk);
            const float st = Len(fk->p[c.hand] - prevH);
            maxStep = std::max(maxStep, st);
            sumStep += st;
            const float dd = Len(fk->p[c.hand] - t);
            maxInc = std::max(maxInc, dd - prevD);
            prevD = dd;
            prevH = fk->p[c.hand];
        }
        const float mean = sumStep / 100.0f;
        const float endErr = Len(prevH - ff.p[c.hand]);
        std::printf("[%s] weight blend 100 steps (target 10 cm): mean step %.5f m, max step %.5f m, max distance "
                    "increase %.2e m, w=1 vs full %.2e m\n",
                    tag, mean, maxStep, maxInc, endErr);
        Check(maxStep <= 3.0f * mean + 1e-6f, std::string(tag) + F(": blend smooth (max %.5f <= 3 x mean %.5f)", maxStep, mean));
        Check(maxInc <= 1e-5f, std::string(tag) + F(": distance to target non-increasing (%.2e)", maxInc));
        Check(endErr <= 1e-6f, std::string(tag) + F(": weight 1 = full solve (%.2e m)", endErr));
    }
}

static void TestRealPose(const ov::OverlayFile& file, const Skeleton& sk, const Rig& rig, const Pose& in, int poseIdx,
                         const char* tag)
{
    const Params prm;
    Fk f0;
    DoFk(sk, in, f0);
    ov::Curves cv;
    file.FrameCurves(poseIdx, file.PoseAt(poseIdx).firstFrame, cv);
    const bool hasIk = (cv.present >> ov::kHandIKL) & 1u;
    std::printf("[%s] pose %s: enable_handik_l %s %.3f, enable_handik_r %s %.3f\n", tag,
                std::string(file.PoseName(poseIdx)).c_str(), hasIk ? "present" : "absent", cv[ov::kHandIKL],
                (cv.present >> ov::kHandIKR) & 1u ? "present" : "absent", cv[ov::kHandIKR]);
    const ArmChain& c = rig.left;
    const Vec3 offs[3] = {{0.03f, 0.0f, 0.0f}, {0.0f, 0.03f, 0.0f}, {0.0f, 0.0f, 0.03f}};
    auto pose = std::make_unique<Pose>();
    Fk fk;
    for (const Vec3& o : offs)
    {
        *pose = in;
        const Vec3 t = f0.p[c.hand] + o;
        const Quat q = f0.r[c.hand];
        Result r;
        const bool ok = SolveArm(sk, *pose, c, t, &q, 1.0f, prm, r);
        DoFk(sk, *pose, fk);
        const float err = Len(fk.p[c.hand] - t);
        const float dUp = ov::AngleDeg(in.b[c.upper].r, pose->b[c.upper].r);
        const float dLo = ov::AngleDeg(in.b[c.lower].r, pose->b[c.lower].r);
        const float dEl = Len(fk.p[c.lower] - f0.p[c.lower]);
        int changedLocals = 0;
        bool onlyChain = true;
        for (int k = 0; k < sk.n; ++k)
            if (std::memcmp(&pose->b[k], &in.b[k], sizeof(Xf)) != 0)
            {
                ++changedLocals;
                onlyChain = onlyChain && (k == c.upper || k == c.lower || k == c.hand);
            }
        float maxOther = 0.0f;
        for (int k = 0; k < sk.n; ++k)
            if (!InSubtree(sk, c.upper, k))
                maxOther = std::max(maxOther, Len(fk.p[k] - f0.p[k]));
        std::printf("[%s] left hand +3 cm (%.2f %.2f %.2f): err %.6f m, upper %.3f deg, fore %.3f deg, elbow move "
                    "%.4f m, changed locals %d, max move outside arm %.2e m\n",
                    tag, o.x, o.y, o.z, err, dUp, dLo, dEl, changedLocals, maxOther);
        Check(ok && r.ok && !r.clamped, std::string(tag) + ": real pose +3 cm solved, unclamped");
        Check(err <= 0.001f, std::string(tag) + F(": real pose hand err %.6f <= 0.001 m", err));
        Check(dUp <= 8.0f && dLo <= 8.0f, std::string(tag) + F(": real pose rotation change small (%.3f, %.3f deg)", dUp, dLo));
        Check(dEl <= 0.04f, std::string(tag) + F(": real pose elbow move %.4f <= 0.04 m", dEl));
        Check(onlyChain && changedLocals <= 3, std::string(tag) + ": only LeftArm / LeftForeArm / LeftHand locals changed");
        Check(maxOther == 0.0f, std::string(tag) + ": nothing outside the left arm subtree moved");
    }
    // grip cache round trip: cache from the same pose, apply -> no-op
    {
        const Grip g = CacheGripFromGamePose(sk, in, rig);
        Check(g.valid, std::string(tag) + ": grip cache valid");
        *pose = in;
        GripInputs gi;
        gi.leftGrip = &g;
        gi.handIkL = 1.0f;
        GripResult gr;
        const bool ok = ApplyGripIK(sk, *pose, rig, gi, prm, gr);
        DoFk(sk, *pose, fk);
        float maxMove = 0.0f;
        for (int k = 0; k < sk.n; ++k)
            maxMove = std::max(maxMove, Len(fk.p[k] - f0.p[k]));
        std::printf("[%s] grip cache round trip: ok %d, left err %.2e m, max bone move %.2e m\n", tag, ok ? 1 : 0,
                    gr.left.handErr, maxMove);
        Check(ok && gr.left.ok && gr.left.handErr <= 1e-4f && maxMove <= 1e-4f,
              std::string(tag) + F(": grip round trip no-op (%.2e m)", maxMove));
        // skip flag and absent curve keep the pose
        *pose = in;
        gi.skip = true;
        ApplyGripIK(sk, *pose, rig, gi, prm, gr);
        Check(std::memcmp(pose.get(), &in, sizeof(Pose)) == 0, std::string(tag) + ": skip flag = identity");
        *pose = in;
        ov::Curves none;
        const GripInputs g0 = InputsFromCurves(none, &g, nullptr);
        ApplyGripIK(sk, *pose, rig, g0, prm, gr);
        Check(std::memcmp(pose.get(), &in, sizeof(Pose)) == 0, std::string(tag) + ": absent enable_handik_l = identity");
        // weapon moved by the right arm (right-hand IK 5 cm forward) -> left hand follows the grip
        *pose = in;
        Xf rt;
        rt.r = f0.r[rig.right.hand];
        rt.t = f0.p[rig.right.hand] + Vec3{0.05f, 0.0f, 0.0f};
        GripInputs g2;
        g2.leftGrip = &g;
        g2.rightTarget = &rt;
        g2.handIkL = 1.0f;
        g2.handIkR = 1.0f;
        ApplyGripIK(sk, *pose, rig, g2, prm, gr);
        DoFk(sk, *pose, fk);
        const Grip g3 = CacheGripFromGamePose(sk, *pose, rig);
        const float gripErr = Len(g3.t - g.t);
        const float gripRot = ov::AngleDeg(g3.r, g.r);
        std::printf("[%s] right hand +5 cm then left grip: right err %.2e m, left err %.2e m, grip drift %.2e m / "
                    "%.4f deg\n",
                    tag, gr.right.handErr, gr.left.handErr, gripErr, gripRot);
        Check(gr.right.ok && gr.left.ok && gr.right.handErr <= 0.001f && gripErr <= 0.001f && gripRot <= 0.05f,
              std::string(tag) + F(": left hand stays on the grip after the weapon moves (%.2e m, %.4f deg)", gripErr, gripRot));
    }
}

static void TestTable()
{
    GripEntry tab[8];
    const char* ok = "# class tx ty tz qx qy qz qw\r\nRifle 0.25 0.02 -0.01 0 0 0 1\r\n\r\nSMG  0.18 0.0 0.0  0 0 0.7071068 0.7071068 # c\n";
    const int n = ParseGripTable(ok, tab, 8);
    Check(n == 2, "grip table: 2 entries parsed");
    const Grip* g = FindGrip(tab, n < 0 ? 0 : n, "SMG");
    Check(g && g->valid && std::fabs(g->t.x - 0.18f) < 1e-6f && std::fabs(g->r.z - 0.7071068f) < 1e-5f,
          "grip table: SMG entry values");
    Check(FindGrip(tab, n < 0 ? 0 : n, "Sniper") == nullptr, "grip table: unknown class = nullptr");
    const char* bad[] = {"Rifle 0.1 0.2 0.3 0 0 0\n", "Rifle 0.1 0.2 0.3 0 0 0 1 9\n", "Rifle 0.1 x 0.3 0 0 0 1\n",
                         "Rifle 0.1 0.2 0.3 0 0 0 2\n", "Rifle nan 0.2 0.3 0 0 0 1\n",
                         "AVeryLongWeaponClassNameThatOverflows32 0 0 0 0 0 0 1\n"};
    int rejected = 0;
    for (const char* b : bad)
        rejected += ParseGripTable(b, tab, 8) < 0 ? 1 : 0;
    Check(rejected == 6, "grip table: 6 malformed tables rejected");
    Check(ParseGripTable("A 0 0 0 0 0 0 1\nB 0 0 0 0 0 0 1\n", tab, 1) < 0, "grip table: capacity overflow rejected");
}

static void TestData(const char* aPath, uint64_t aHash, const char* aTag)
{
    auto file = std::make_unique<ov::OverlayFile>();
    std::string err;
    const bool ok = file->LoadFromFile(aPath, aHash, err);
    Check(ok, std::string(aTag) + ": load " + aPath + (ok ? "" : " (" + err + ")"));
    if (!ok)
        return;
    const Skeleton& sk = file->Skel();
    const Rig rig = FindRig(file->NumBones(), [&](int i) { return file->BoneName(i); });
    std::printf("[%s] bones %d: LeftArm %d LeftForeArm %d LeftHand %d RightArm %d RightForeArm %d RightHand %d "
                "WeaponLeft %d WeaponRight %d\n",
                aTag, sk.n, rig.left.upper, rig.left.lower, rig.left.hand, rig.right.upper, rig.right.lower,
                rig.right.hand, rig.weaponLeft, rig.weaponRight);
    Check(rig.Valid(sk), std::string(aTag) + ": CP arm chains + weapon bones found");
    if (!rig.Valid(sk))
        return;
    for (auto aim : {ov::AimState::Ready, ov::AimState::Aiming})
    {
        const int one = file->FindOverlayPose(ov::PoseOverlay::Pistol1H, aim, false, false);
        const int two = file->FindOverlayPose(ov::PoseOverlay::Pistol2H, aim, false, false);
        Check(one >= 0 && two >= 0, std::string(aTag) + ": pistol source-contract poses found");
        if (one < 0 || two < 0) continue;
        auto p1 = std::make_unique<Pose>();
        auto p2 = std::make_unique<Pose>();
        auto solved = std::make_unique<Pose>();
        file->FramePose(file->PoseAt(one).firstFrame, *p1);
        file->FramePose(file->PoseAt(two).firstFrame, *p2);
        const Grip g1 = CacheGripFromGamePose(sk, *p1, rig);
        const Grip g2 = CacheGripFromGamePose(sk, *p2, rig);
        const float delta = Len(g1.t - g2.t);
        GripInputs inputs; inputs.handIkL = 1.0f; inputs.leftGrip = &g1;
        *solved = *p2;
        GripResult wrong;
        ApplyGripIK(sk, *solved, rig, inputs, Params{}, wrong);
        inputs.leftGrip = &g2; *solved = *p2;
        GripResult own;
        ApplyGripIK(sk, *solved, rig, inputs, Params{}, own);
        std::printf("[%s] pistol source contract aim=%d: 1H->2H target delta %.6f m, req/use %.6f/%.6f, clamp %d error %.6f m; authored 2H clamp %d error %.6f m\n",
                    aTag, static_cast<int>(aim), delta, wrong.left.reqDist, wrong.left.useDist, wrong.left.clamped ? 1 : 0,
                    wrong.left.handErr, own.left.clamped ? 1 : 0, own.left.handErr);
        Check(g1.valid && g2.valid && delta > 0.005f,
              std::string(aTag) + ": one-handed left-hand location is not the authored two-handed grip");
        bool lengthsUnchanged = true;
        for (int i = 0; i < sk.n; ++i)
            lengthsUnchanged = lengthsUnchanged && std::memcmp(&p2->b[i].t, &solved->b[i].t, sizeof(Vec3)) == 0;
        Check(own.left.ok && own.left.handErr <= 0.001f && lengthsUnchanged,
              std::string(aTag) + ": authored two-handed grip stays within 1mm with unchanged local bone translations");
        Grip far; far.valid = true; far.t = {5.0f, 0.0f, 0.0f};
        inputs.leftGrip = &far; inputs.coupleWeaponHands = true; *solved = *p2;
        GripResult bounded;
        ApplyGripIK(sk, *solved, rig, inputs, Params{}, bounded);
        const bool rightUnchanged = std::memcmp(&p2->b[rig.right.upper], &solved->b[rig.right.upper], sizeof(Xf)) == 0 &&
                                    std::memcmp(&p2->b[rig.right.lower], &solved->b[rig.right.lower], sizeof(Xf)) == 0 &&
                                    std::memcmp(&p2->b[rig.right.hand], &solved->b[rig.right.hand], sizeof(Xf)) == 0;
        Check(rightUnchanged && !bounded.right.ok && bounded.weaponShift == 0.0f && bounded.left.clamped,
              std::string(aTag) + ": coupled grip refuses a large unrelated correction without moving the right arm");
    }
    const int pi = file->FindOverlayPose(ov::PoseOverlay::Rifle, ov::AimState::Aiming, false, false);
    Check(pi >= 0, std::string(aTag) + ": Rifle Stand Aim pose found");
    if (pi < 0)
        return;
    auto in = std::make_unique<Pose>();
    file->FramePose(file->PoseAt(pi).firstFrame, *in);
    TestArm(sk, *in, rig.left, (std::string(aTag) + " left").c_str());
    TestArm(sk, *in, rig.right, (std::string(aTag) + " right").c_str());
    TestRealPose(*file, sk, rig, *in, pi, aTag);
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_gripik_test <utpp_overlays_woman.utppov> <utpp_overlays_man.utppov>\n");
        return 2;
    }
    TestTable();
    TestData(argv[1], ov::kRigIdentityWomanOv, "woman");
    TestData(argv[2], ov::kRigIdentityManOv, "man");
    std::printf("gripik test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

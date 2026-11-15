// Girdle width bound of the mesh-space arm poses (src/mm/MeshArmClavicleLimit.hpp BoundGirdleWidth) on the real data of both bodies.
//
//   utpp_clavicle_girdle_test woman.utppmm woman.utppov man.utppmm man.utppov [--dump body] [--max-shift cm]
//
// The bound acts on standing (idle) two-handed weapon poses only (share 1 for the rifle / two-handed pistol poses under the idle loops,
// 0 under the moving loops and for the one-handed pistol set, as in the runtime).
// The locomotion pose L plays every standing and crouch loop of the motion database (2.5 s at 60 Hz) under the weapon poses of the
// rifle and both pistol sets in every aim state (standing poses under standing loops, crouch poses under crouch loops, idle or moving
// pose by the loop), and the aim crossfade of the crouch idle: Ready 0.3 s, Ready -> Aim over 0.2 s, Aim 1.2 s, Aim -> Ready over
// 0.2 s, Ready 0.5 s. Each frame is composed (ComposeLayers + the absolute clavicle limit of the runtime) = "before", then bounded =
// "after". Width = lateral distance of the upper arm joints in the Spine3 frame; own width = the same with the clavicles at their
// rotation in L; deficit = own - width.
//   CG-0  before: the two-handed pistol crouch aim is more than 1.5 cm narrower than the own width (the case reaches the fault);
//   CG-1  after: the deficit is not above 1.5 cm on any frame, unless the shared hand shift reached kGirdleMaxHandShiftM (counted and
//         printed); the two-handed pistol crouch aim (stopped by the shift bound: its arms are at full reach) ends at most 3.0 cm below
//         the own width and at least 1.0 cm closer to it than before (measured 2.9 with the 2 cm bound, from 4.1);
//   CG-2  frames with a deficit not above 1.5 cm before are untouched bit for bit;
//   CG-3  the hands and the weapon: the left hand keeps its place on the weapon (WeaponRight frame, two-handed sets) within 1 mm, both hand rotations
//         and the weapon rotation within 0.1 deg; the weapon shift is not above kGirdleMaxHandShiftM (printed);
//   CG-4  the width never falls (after >= before on every frame, both bodies);
//   CG-5  no pop through the aim crossfade: the largest per-frame step of the upper arm joints after is not above the largest step
//         before + 0.5 cm (sequence of the crouch idle aim in / out).
// Mutation builds (UTPP_CLAVLIMIT_MUTATION 6 = never acts, 7 = the arms are not solved back) must print a FAIL CG- line (6: CG-1,
// 7: CG-3).

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../src/mm/GripIK.hpp"
#include "../src/mm/MeshArmClavicleLimit.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/OverlayFile.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

static int g_pass = 0, g_fail = 0;
static std::string g_dumpBody;
static float g_maxShift = kGirdleMaxHandShiftM; // --max-shift cm (study of the bound; the test checks the runtime value)
static void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
static std::string F(const char* aFmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, aFmt);
    std::vsnprintf(buf, sizeof(buf), aFmt, ap);
    va_end(ap);
    return buf;
}
static bool Has(std::string_view n, const char* s) { return n.find(s) != std::string_view::npos; }
static bool SamePose(const Pose& a, const Pose& b, int n) { return std::memcmp(a.b, b.b, sizeof(Xf) * static_cast<size_t>(n)) == 0; }
static void CurvesMix(const Curves& aA, const Curves& aB, float aW, Curves& aOut)
{
    aOut = Curves{};
    aOut.Accumulate(aA, 1.0f - aW);
    aOut.Accumulate(aB, aW);
}

struct Stat
{
    int frames = 0, engaged = 0, capped = 0, over = 0, untouchedBad = 0, narrower = 0;
    float defBefore = 0.0f, defAfter = 0.0f, minBefore = 1e9f, minAfter = 1e9f, handPos = 0.0f, handRot = 0.0f, wpnPos = 0.0f,
          wpnRot = 0.0f;
    void Add(const Stat& o)
    {
        frames += o.frames, engaged += o.engaged, capped += o.capped, over += o.over, untouchedBad += o.untouchedBad,
            narrower += o.narrower;
        defBefore = std::max(defBefore, o.defBefore), defAfter = std::max(defAfter, o.defAfter);
        minBefore = std::min(minBefore, o.minBefore), minAfter = std::min(minAfter, o.minAfter);
        handPos = std::max(handPos, o.handPos), handRot = std::max(handRot, o.handRot), wpnPos = std::max(wpnPos, o.wpnPos),
        wpnRot = std::max(wpnRot, o.wpnRot);
    }
};

static void TestBody(const char* aDb, const char* aOv, uint64_t aHash, const char* aTag)
{
    auto db = std::make_unique<motion::MotionDatabase>();
    auto f = std::make_unique<OverlayFile>();
    std::string err;
    const bool loaded = db->LoadFromFile(aDb, aHash, err) && f->LoadFromFile(aOv, aHash, err);
    Check(loaded, std::string(aTag) + ": load database and overlay file" + (loaded ? "" : " (" + err + ")"));
    if (!loaded)
        return;
    const Skeleton& sk = f->Skel();
    bool sameRig = static_cast<int>(db->NumBones()) == sk.n;
    for (int i = 0; sameRig && i < sk.n; ++i)
        sameRig = db->BoneName(static_cast<uint32_t>(i)) == f->BoneName(i) && db->BoneParent(static_cast<uint32_t>(i)) == sk.parent[i];
    auto bone = [&](const char* aName) {
        for (int i = 0; i < sk.n; ++i)
            if (f->BoneName(i) == aName)
                return i;
        return -1;
    };
    const int clav[2] = {bone("LeftShoulder"), bone("RightShoulder")}, s3 = bone("Spine3");
    const gik::Rig rig = gik::FindRig(sk.n, [&](int i) { return f->BoneName(i); });
    const bool bonesOk = sameRig && clav[0] > 0 && clav[1] > 0 && s3 > 0 && rig.Valid(sk);
    Check(bonesOk, std::string(aTag) + ": shared bone order, shoulder / arm / weapon bones found");
    if (!bonesOk)
        return;
    const int hand[2] = {rig.left.hand, rig.right.hand}, arm[2] = {rig.left.upper, rig.right.upper};
    auto frameOf = [&](uint32_t aF, Pose& aOut) {
        const motion::PoseRecord* p = db->Pose(aF);
        aOut.n = sk.n;
        for (int i = 0; i < sk.n; ++i)
            aOut.b[i] = Xf{math::QuatNormalize(Quat{p[i].qx, p[i].qy, p[i].qz, p[i].qw}), Vec3{p[i].px, p[i].py, p[i].pz}, Vec3{1, 1, 1}};
    };
    const int pBi = f->FindPose("Pose_Neutral_Stand_Idle"), pBm = f->FindPose("Pose_Neutral_Stand_Move");
    const int pCi = f->FindPose("Pose_Neutral_Crouch_Idle"), pCm = f->FindPose("Pose_Neutral_Crouch_Move");
    Check(pBi >= 0 && pBm >= 0 && pCi >= 0 && pCm >= 0, std::string(aTag) + ": base poses found");
    if (pBi < 0 || pBm < 0 || pCi < 0 || pCm < 0)
        return;
    std::unique_ptr<Pose> base[4];
    for (int k = 0; k < 4; ++k)
    {
        base[k] = std::make_unique<Pose>();
        f->FramePose(f->PoseAt((k == 0 ? pBi : k == 1 ? pBm : k == 2 ? pCi : pCm)).firstFrame, *base[k]);
    }
    std::vector<uint32_t> standLoops, crouchLoops;
    uint32_t crouchIdle = ~0u;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string_view n = db->ClipName(c);
        if (!Has(n, "Loop") || Has(n, "Slide") || Has(n, "Jump"))
            continue;
        (Has(n, "Crouch") ? crouchLoops : standLoops).push_back(c);
        if (Has(n, "Crouch") && Has(n, "Idle") && crouchIdle == ~0u)
            crouchIdle = c;
    }
    Check(!standLoops.empty() && !crouchLoops.empty() && crouchIdle != ~0u, std::string(aTag) + ": loops found");
    if (crouchIdle == ~0u)
        return;
    const float fps = db->Header().fps, dt = 1.0f / 60.0f;
    const gik::Params params;

    auto L = std::make_unique<Pose>(), OP = std::make_unique<Pose>(), OPb = std::make_unique<Pose>(), before = std::make_unique<Pose>(),
         after = std::make_unique<Pose>();
    auto S = std::make_unique<LayerScratch>();
    static Vec3 pB[ov::kMaxBones], pA[ov::kMaxBones];
    static Quat rB[ov::kMaxBones], rA[ov::kMaxBones];
    // one frame: compose, bound, measure into aSt; returns the upper arm joints after / before (mesh, m) for the step check
    auto frame = [&](const LayerInputs& aIn, Stat& aSt, bool aDump, float aT, Vec3* aJointsB, Vec3* aJointsA, bool aTwoHanded, float aShare) {
        ComposeLayers(sk, f->Masks(), aIn, *S, *before);
        LimitMeshArmClavicles(sk, *S, clav[0], clav[1], kMeshArmClavicleRuntimeLimitDeg, kMeshArmClavicleSoftDeg, *before);
        *after = *before;
        const GirdleResult g = BoundGirdleWidth(sk, *aIn.L, rig, s3, clav[0], clav[1], kGirdleDeficitLimitM, g_maxShift, aTwoHanded, aShare, params, *after);
        MeshPositions(sk, *before, pB, rB);
        MeshPositions(sk, *after, pA, rA);
        auto lat = [&](const Vec3* aP, const Quat* aR) {
            const Quat inv = math::QuatInv(aR[s3]);
            return (math::QuatRotate(inv, aP[arm[1]] - aP[s3]).z - math::QuatRotate(inv, aP[arm[0]] - aP[s3]).z) * 100.0f;
        };
        const float wB = lat(pB, rB), wA = lat(pA, rA), own = g.ownWidth * 100.0f;
        const float dB = own - wB, dA = own - wA;
        ++aSt.frames;
        const bool touched = !SamePose(*before, *after, sk.n);
        aSt.engaged += touched;
        const bool cap = g.capped[0] || g.capped[1];
        aSt.capped += cap;
        aSt.over += dA > 1.5f + 1e-3f && !cap && aShare > 0.0f; // moving poses (share 0) are not bounded
        aSt.untouchedBad += !(dB > 1.5f) && touched;
        aSt.narrower += wA < wB - 1e-4f;
        if (aShare > 0.0f)
            aSt.defBefore = std::max(aSt.defBefore, dB);
        if (aShare > 0.0f)
            aSt.defAfter = std::max(aSt.defAfter, dA);
        aSt.minBefore = std::min(aSt.minBefore, wB);
        aSt.minAfter = std::min(aSt.minAfter, wA);
        // the left hand in the weapon frame (grip kept), both hand rotations, the weapon shift and rotation
        const Quat wInvB = math::QuatInv(rB[rig.weaponRight]), wInvA = math::QuatInv(rA[rig.weaponRight]);
        const Vec3 gB = math::QuatRotate(wInvB, pB[hand[0]] - pB[rig.weaponRight]), gA = math::QuatRotate(wInvA, pA[hand[0]] - pA[rig.weaponRight]);
        const Vec3 dg = gA - gB;
        if (aTwoHanded) // the one-handed sets have no grip of the left hand
            aSt.handPos = std::max(aSt.handPos, std::sqrt(math::Dot(dg, dg)) * 100.0f);
        for (int s = 0; s < 2; ++s)
            aSt.handRot = std::max(aSt.handRot, AngleDeg(rA[hand[s]], rB[hand[s]]));
        const Vec3 dw = pA[rig.weaponRight] - pB[rig.weaponRight];
        aSt.wpnPos = std::max(aSt.wpnPos, std::sqrt(math::Dot(dw, dw)) * 100.0f);
        aSt.wpnRot = std::max(aSt.wpnRot, AngleDeg(rA[rig.weaponRight], rB[rig.weaponRight]));
        if (aJointsB)
            for (int s = 0; s < 2; ++s)
                aJointsB[s] = pB[arm[s]], aJointsA[s] = pA[arm[s]];
        if (aDump)
            std::printf("DUMP %s t %.3f own %.1f before %.1f after %.1f cap %d/%d hand err %.2f/%.2f mm\n", aTag, double(aT), double(own),
                        double(wB), double(wA), int(g.capped[0]), int(g.capped[1]), double(g.handErr[0] * 1000.0f),
                        double(g.handErr[1] * 1000.0f));
    };

    const PoseOverlay kSets[] = {PoseOverlay::Rifle, PoseOverlay::Pistol2H, PoseOverlay::Pistol1H};
    const char* kSetNames[] = {"Rifle", "Pistol2H", "Pistol1H"};
    const char* kAimNames[] = {"Relax", "Ready", "Aim"};
    Stat all, p2hCrouchAim;
    for (int si = 0; si < 3; ++si)
        for (int ai = 0; ai < 3; ++ai)
            for (int crouch = 0; crouch < 2; ++crouch)
            {
                Stat st;
                for (uint32_t c : crouch ? crouchLoops : standLoops)
                {
                    const bool idle = Has(db->ClipName(c), "Idle");
                    const int pi = f->FindOverlayPose(kSets[si], static_cast<AimState>(ai), crouch != 0, !idle);
                    if (pi < 0)
                        continue;
                    f->FramePose(f->PoseAt(pi).firstFrame, *OP);
                    LayerInputs in;
                    in.L = L.get();
                    in.B = base[crouch * 2 + (idle ? 0 : 1)].get();
                    in.OP = OP.get();
                    in.OPLower = OP.get();
                    f->FrameCurves(pi, f->PoseAt(pi).firstFrame, in.curves);
                    const auto& rec = db->Clip(c);
                    const float dur = static_cast<float>(rec.numFrames - 1) / fps;
                    for (int k = 0; k <= 150 && dur > 0.0f; ++k)
                    {
                        const float t = std::fmod(static_cast<float>(k) * dt, dur);
                        frameOf(rec.firstFrame + std::min(rec.numFrames - 1, static_cast<uint32_t>(t * fps + 0.5f)), *L);
                        frame(in, st, false, t, nullptr, nullptr, true, idle && si < 2 ? 1.0f : 0.0f);
                    }
                }
                if (!st.frames)
                    continue;
                std::printf("  [%s] %-8s %-5s %-6s frames %5d engaged %5d capped %4d | width min before %5.1f after %5.1f | deficit max (bounded frames) before %5.2f after %5.2f |"
                            " hand %.3f cm %.3f deg, weapon %.3f cm %.3f deg\n",
                            aTag, kSetNames[si], kAimNames[ai], crouch ? "crouch" : "stand", st.frames, st.engaged, st.capped,
                            double(st.minBefore), double(st.minAfter), double(st.defBefore), double(st.defAfter), double(st.handPos),
                            double(st.handRot), double(st.wpnPos), double(st.wpnRot));
                all.Add(st);
                if (si == 1 && ai == 2 && crouch)
                    p2hCrouchAim.Add(st);
            }

    // aim crossfade in / out under the crouch idle loop
    Stat seq;
    float stepB = 0.0f, stepA = 0.0f;
    {
        const int pr = f->FindOverlayPose(PoseOverlay::Pistol2H, AimState::Ready, true, false);
        const int pa = f->FindOverlayPose(PoseOverlay::Pistol2H, AimState::Aiming, true, false);
        Curves cr, ca;
        auto OPr = std::make_unique<Pose>(), OPa = std::make_unique<Pose>();
        f->FramePose(f->PoseAt(pr).firstFrame, *OPr);
        f->FramePose(f->PoseAt(pa).firstFrame, *OPa);
        f->FrameCurves(pr, f->PoseAt(pr).firstFrame, cr);
        f->FrameCurves(pa, f->PoseAt(pa).firstFrame, ca);
        const auto& rec = db->Clip(crouchIdle);
        const float dur = static_cast<float>(rec.numFrames - 1) / fps;
        Vec3 jb[2], ja[2], jbPrev[2], jaPrev[2];
        const int frames = static_cast<int>(2.4f / dt);
        for (int k = 0; k <= frames; ++k)
        {
            const float t = static_cast<float>(k) * dt;
            const float w = t < 0.3f ? 0.0f : t < 0.5f ? (t - 0.3f) / 0.2f : t < 1.7f ? 1.0f : t < 1.9f ? 1.0f - (t - 1.7f) / 0.2f : 0.0f;
            frameOf(rec.firstFrame + std::min(rec.numFrames - 1, static_cast<uint32_t>(std::fmod(t, dur) * fps + 0.5f)), *L);
            Blend(*OPr, *OPa, w, *OPb);
            LayerInputs in;
            in.L = L.get();
            in.B = base[2].get();
            in.OP = OPb.get();
            in.OPLower = OPb.get();
            CurvesMix(cr, ca, w, in.curves);
            frame(in, seq, g_dumpBody == aTag, t, jb, ja, true, 1.0f);
            if (k > 0)
                for (int s = 0; s < 2; ++s)
                {
                    const Vec3 db_ = jb[s] - jbPrev[s], da = ja[s] - jaPrev[s];
                    stepB = std::max(stepB, std::sqrt(math::Dot(db_, db_)) * 100.0f);
                    stepA = std::max(stepA, std::sqrt(math::Dot(da, da)) * 100.0f);
                }
            for (int s = 0; s < 2; ++s)
                jbPrev[s] = jb[s], jaPrev[s] = ja[s];
        }
        std::printf("  [%s] Pistol2H crouch Ready -> Aim -> Ready: frames %d engaged %d capped %d | width min before %.1f after %.1f | deficit max before %.2f after %.2f |"
                    " joint step max before %.2f after %.2f cm | hand %.3f cm weapon %.3f cm %.3f deg\n",
                    aTag, seq.frames, seq.engaged, seq.capped, double(seq.minBefore), double(seq.minAfter), double(seq.defBefore),
                    double(seq.defAfter), double(stepB), double(stepA), double(seq.handPos), double(seq.wpnPos), double(seq.wpnRot));
        all.Add(seq);
        p2hCrouchAim.Add(seq);
    }

    Check(p2hCrouchAim.defBefore > 1.5f, F("CG-0 %s: before, the two-handed pistol crouch aim is %.2f cm narrower than the own width (> 1.5)", aTag,
                                           double(p2hCrouchAim.defBefore)));
    Check(all.over == 0 && p2hCrouchAim.defAfter <= 3.0f && p2hCrouchAim.defAfter <= p2hCrouchAim.defBefore - 1.0f,
          F("CG-1 %s: deficit after not above 1.5 cm unless the hand shift bound stops it (%d frames over, %d of %d frames at the shift bound);"
            " two-handed pistol crouch aim: max deficit before %.2f after %.2f cm (<= 3.0 and >= 1.0 cm less; %d frames at the shift bound)",
            aTag, all.over, all.capped, all.frames, double(p2hCrouchAim.defBefore), double(p2hCrouchAim.defAfter), p2hCrouchAim.capped));
    Check(all.untouchedBad == 0, F("CG-2 %s: frames with a deficit not above 1.5 cm untouched bit for bit (%d changed; %d of %d frames engaged)", aTag,
                                   all.untouchedBad, all.engaged, all.frames));
    Check(all.handPos <= 0.1f && all.handRot <= 0.1f && all.wpnRot <= 0.1f && all.wpnPos <= g_maxShift * 100.0f + 0.05f,
          F("CG-3 %s: left hand on the weapon kept %.4f cm, hand rotations %.4f deg, weapon rotation %.4f deg, weapon shift max %.3f cm (bound %.1f)"
            " (two-handed pistol crouch aim: weapon shift max %.3f cm)", aTag, double(all.handPos), double(all.handRot), double(all.wpnRot),
            double(all.wpnPos), double(g_maxShift * 100.0f), double(p2hCrouchAim.wpnPos)));
    Check(all.narrower == 0, F("CG-4 %s: the width never falls (%d frames narrower)", aTag, all.narrower));
    Check(stepA <= stepB + 0.5f, F("CG-5 %s: aim in / out, largest upper arm joint step after %.2f cm, before %.2f cm", aTag, double(stepA), double(stepB)));
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("FAIL usage: utpp_clavicle_girdle_test woman.utppmm woman.utppov man.utppmm man.utppov [--dump body]\n");
        return 1;
    }
    for (int i = 5; i + 1 < argc; ++i)
        if (std::string_view(argv[i]) == "--dump")
            g_dumpBody = argv[++i];
        else if (std::string_view(argv[i]) == "--max-shift")
            g_maxShift = static_cast<float>(std::atof(argv[++i])) / 100.0f;
    TestBody(argv[3], argv[4], kRigIdentityManOv, "man");
    TestBody(argv[1], argv[2], kRigIdentityWomanOv, "woman");
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

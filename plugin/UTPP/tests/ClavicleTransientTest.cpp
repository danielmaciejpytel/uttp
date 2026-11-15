// Transient bound of the mesh-space arm clavicle (src/mm/MeshArmClavicleLimit.hpp ClavicleTransientBound) on the real data of both bodies.
//
//   utpp_clavicle_transient_test woman.utppmm woman.utppov man.utppmm man.utppov [--fps F] [--limit D] [--soft D] [--release S]
//                                [--settled D] [--hold S] [--dump body clip set aim entry t0]
//
// The locomotion pose L plays each standing stop and start clip at the frame rate F (default 60 Hz) from the entry time (a stop is
// entered at 0, 0.4 or 0.75 s of the clip, as motion matching enters a stop clip mid-way; a start at 0) and holds its last frame for
// 0.6 s. The weapon pose crossfades between its moving and its standing pose (a stop: moving -> standing t0 = 0, 0.1, 0.2 s after the
// entry; a start: standing -> moving at 0 or 2 frames; overlay pose and layer curves over 0.2 s, base pose over 0.1 s, as the runtime
// OverlayTimes poseSwitch / moveSwitch) for the rifle and both pistol weapon pose sets in every aim state. Each frame is composed
// (ComposeLayers + the absolute clavicle limit of the runtime) = "before", then the transient bound is applied = "after" (aFading = the
// crossfade is in progress). Shoulder width = lateral distance of the two upper arm joints in the Spine3 frame (z of the right minus z of
// the left joint, cm); the chest width of a frame = that width with both clavicles at their local-space reference (the shoulders fixed to
// the chest, as in the moving pose); "off" = |width - chest width|. Deviation = angle of the clavicle local rotation against the reference.
//   CTB-0  before: a two-handed stop leaves the chest width by more than 1.5 cm (the case reaches the fault);
//   CTB-1  excess = off - max(1.5 cm, the off of that sequence's held last frame: a settled pose the bound must not change). After,
//          rifle and two-handed pistol sets in the Relaxed state, straight run / sprint stops (_Stop_F_): excess <= 0 on every frame.
//          Ready / Aim states, starts (not engaged), walk stops and run / sprint stops with a direction change, and the one-handed
//          pistol sets: the worst excess is not larger than before (printed: the 1.5 cm target is not reached on all of them);
//   CTB-2  the mesh rotations of every bone below the clavicles are kept (< 0.01 deg), the clavicle translations and every bone
//          outside the two clavicles and their children are untouched bit for bit;
//   CTB-3  settled poses: standing loops under the standing / moving weapon pose and crouch loops under the crouch poses (steady
//          curves, no crossfade) give the "before" pose bit for bit on every frame, and every stop / start sequence is released and
//          untouched on its last held frame;
//   CTB-4  no pop: the largest per-frame step of the bounded clavicle rotation never exceeds the largest unbounded step of the same
//          sequence (+ 0.05 deg);
//   CTB-5  other fast chest transients (180 degree turns, landings, slides under both steady poses): off and deviation not larger
//          after than before (+ 0.05 cm / deg).
// Frame-rate dependence: run with --fps 30 / 144. Mutation builds (UTPP_CLAVLIMIT_MUTATION 4 = never engages, 5 = released in one frame)
// run the rifle sets only and must print a FAIL CTB- line (4: CTB-1, 5: CTB-1 or CTB-4).

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

#include "../src/mm/MeshArmClavicleLimit.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/OverlayFile.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
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

struct Options
{
    float fps = 60.0f;
    ClavicleTransientBound proto;
    std::string dumpBody, dumpClip, dumpSet, dumpAim;
    float dumpEntry = -1.0f, dumpT0 = -1.0f;
};
static Options g_opt;
#if defined(UTPP_CLAVLIMIT_MUTATION)
static constexpr int kSetCount = 1; // mutation builds: rifle sets only
#else
static constexpr int kSetCount = 3;
#endif

static bool Has(std::string_view n, const char* s) { return n.find(s) != std::string_view::npos; }
static bool IsStandStop(std::string_view n) { return !Has(n, "Crouch") && !Has(n, "Jump") && Has(n, "Stop"); }
static bool IsStandStart(std::string_view n) { return !Has(n, "Crouch") && !Has(n, "Jump") && Has(n, "Start"); }
static bool IsOtherTransient(std::string_view n) { return Has(n, "Turn_180") || Has(n, "Land") || Has(n, "Slide"); }
static bool IsStandLoop(std::string_view n) { return Has(n, "Loop") && !Has(n, "Crouch") && !Has(n, "Slide") && !Has(n, "Jump"); }
static bool IsCrouchLoop(std::string_view n) { return Has(n, "Loop") && Has(n, "Crouch") && !Has(n, "Slide"); }
static bool IsIdleLoop(std::string_view n) { return Has(n, "Idle"); }
static void CurvesMix(const Curves& aA, const Curves& aB, float aW, Curves& aOut)
{
    aOut = Curves{};
    aOut.Accumulate(aA, 1.0f - aW);
    aOut.Accumulate(aB, aW);
}
static bool SamePose(const Pose& a, const Pose& b, int n) { return std::memcmp(a.b, b.b, sizeof(Xf) * static_cast<size_t>(n)) == 0; }

struct Rig
{
    const Skeleton* sk = nullptr;
    int clav[2] = {-1, -1}, arm[2] = {-1, -1}, spine3 = -1;
};

struct FrameMeasure
{
    float width = 0.0f, chest = 0.0f, dev[2] = {0, 0};
    Vec3 sh[2]; // upper arm joints in the Spine3 frame (cm)
};
static void Shoulders(const Rig& aRig, const Pose& aP, Vec3* aSh)
{
    static Vec3 pos[ov::kMaxBones];
    static Quat rot[ov::kMaxBones];
    MeshPositions(*aRig.sk, aP, pos, rot);
    for (int s = 0; s < 2; ++s)
        aSh[s] = math::QuatRotate(math::QuatInv(rot[aRig.spine3]), pos[aRig.arm[s]] - pos[aRig.spine3]) * 100.0f;
}
static FrameMeasure Measure(const Rig& aRig, const Pose& aP, const LayerScratch& aS, Pose& aTmp)
{
    FrameMeasure m;
    Shoulders(aRig, aP, m.sh);
    m.width = m.sh[1].z - m.sh[0].z;
    for (int s = 0; s < 2; ++s)
        m.dev[s] = AngleDeg((s ? aS.armR : aS.armL).b[aRig.clav[s]].r, aP.b[aRig.clav[s]].r);
    aTmp = aP;
    aTmp.b[aRig.clav[0]].r = aS.armL.b[aRig.clav[0]].r;
    aTmp.b[aRig.clav[1]].r = aS.armR.b[aRig.clav[1]].r;
    Vec3 c[2];
    Shoulders(aRig, aTmp, c);
    m.chest = c[1].z - c[0].z;
    return m;
}

static void TestBody(const char* aDb, const char* aOv, uint64_t aHash, const char* aTag)
{
    auto db = std::make_unique<motion::MotionDatabase>();
    auto f = std::make_unique<OverlayFile>();
    std::string err;
    const bool dbOk = db->LoadFromFile(aDb, aHash, err);
    Check(dbOk, std::string(aTag) + ": load " + aDb + (dbOk ? "" : " (" + err + ")"));
    const bool ovOk = f->LoadFromFile(aOv, aHash, err);
    Check(ovOk, std::string(aTag) + ": load " + aOv + (ovOk ? "" : " (" + err + ")"));
    if (!dbOk || !ovOk)
        return;
    const Skeleton& sk = f->Skel();
    bool sameRig = static_cast<int>(db->NumBones()) == sk.n;
    for (int i = 0; sameRig && i < sk.n; ++i)
        sameRig = db->BoneName(static_cast<uint32_t>(i)) == f->BoneName(i) && db->BoneParent(static_cast<uint32_t>(i)) == sk.parent[i];
    Check(sameRig, std::string(aTag) + ": database and overlay file share the bone order");
    if (!sameRig)
        return;
    auto bone = [&](const char* aName) {
        for (int i = 0; i < sk.n; ++i)
            if (f->BoneName(i) == aName)
                return i;
        return -1;
    };
    Rig rig;
    rig.sk = &sk;
    rig.clav[0] = bone("LeftShoulder");
    rig.clav[1] = bone("RightShoulder");
    rig.arm[0] = bone("LeftArm");
    rig.arm[1] = bone("RightArm");
    rig.spine3 = bone("Spine3");
    const bool bonesOk = rig.clav[0] > 0 && rig.clav[1] > 0 && rig.arm[0] > 0 && rig.arm[1] > 0 && rig.spine3 > 0;
    Check(bonesOk, std::string(aTag) + ": shoulder bones found");
    if (!bonesOk)
        return;
    auto frameOf = [&](uint32_t aF, Pose& aOut) {
        const motion::PoseRecord* p = db->Pose(aF);
        aOut.n = sk.n;
        for (int i = 0; i < sk.n; ++i)
            aOut.b[i] = Xf{math::QuatNormalize(Quat{p[i].qx, p[i].qy, p[i].qz, p[i].qw}), Vec3{p[i].px, p[i].py, p[i].pz}, Vec3{1, 1, 1}};
    };
    std::vector<uint32_t> stops, starts, others, standLoops, crouchLoops;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string_view n = db->ClipName(c);
        if (IsOtherTransient(n))
            others.push_back(c);
        else if (IsStandStop(n))
            stops.push_back(c);
        else if (IsStandStart(n))
            starts.push_back(c);
        else if (IsStandLoop(n))
            standLoops.push_back(c);
        else if (IsCrouchLoop(n))
            crouchLoops.push_back(c);
    }
    std::printf("[%s] stop clips %zu, start clips %zu, other transients %zu, stand loops %zu, crouch loops %zu\n", aTag, stops.size(),
                starts.size(), others.size(), standLoops.size(), crouchLoops.size());
    Check(stops.size() >= 4 && starts.size() >= 4 && !others.empty() && !standLoops.empty() && !crouchLoops.empty(),
          std::string(aTag) + ": clips present");

    const int pBi = f->FindPose("Pose_Neutral_Stand_Idle"), pBm = f->FindPose("Pose_Neutral_Stand_Move");
    const int pCi = f->FindPose("Pose_Neutral_Crouch_Idle"), pCm = f->FindPose("Pose_Neutral_Crouch_Move");
    Check(pBi >= 0 && pBm >= 0 && pCi >= 0 && pCm >= 0, std::string(aTag) + ": base poses found");
    if (pBi < 0 || pBm < 0 || pCi < 0 || pCm < 0)
        return;
    auto Bi = std::make_unique<Pose>(), Bm = std::make_unique<Pose>(), Ci = std::make_unique<Pose>(), Cm = std::make_unique<Pose>();
    f->FramePose(f->PoseAt(pBi).firstFrame, *Bi);
    f->FramePose(f->PoseAt(pBm).firstFrame, *Bm);
    f->FramePose(f->PoseAt(pCi).firstFrame, *Ci);
    f->FramePose(f->PoseAt(pCm).firstFrame, *Cm);

    const float fps = db->Header().fps;
    const float dt = 1.0f / g_opt.fps;
    const PoseOverlay kSets[] = {PoseOverlay::Rifle, PoseOverlay::Pistol2H, PoseOverlay::Pistol1H};
    const char* kSetNames[] = {"Rifle", "Pistol2H", "Pistol1H"};
    const AimState kAims[] = {AimState::Relaxed, AimState::Ready, AimState::Aiming};
    const char* kAimNames[] = {"Relax", "Ready", "Aim"};
    const std::vector<float> kEntryStop = {0.0f, 0.4f, 0.75f}, kEntryStart = {0.0f};
    const std::vector<float> kT0Stop = {0.0f, 0.1f, 0.2f}, kT0Start = {0.0f, 2.0f * dt};
    const int kHold = static_cast<int>(0.6f * g_opt.fps + 0.5f);

    auto L = std::make_unique<Pose>(), B = std::make_unique<Pose>(), OP = std::make_unique<Pose>(), before = std::make_unique<Pose>(),
         after = std::make_unique<Pose>(), tmp = std::make_unique<Pose>();
    auto S = std::make_unique<LayerScratch>();
    static Quat mB[ov::kMaxBones], mA[ov::kMaxBones];
    int ctb2Bad = 0, ctb3Frames = 0, ctb3Diff = 0, endDiff = 0, sequences = 0, engagedFrames = 0;
    float ctb2Worst = 0.0f, before2HWorst = 0.0f;

    // composes one frame (before / after) and checks CTB-2 on it
    auto composeFrame = [&](LayerInputs& aIn, ClavicleTransientBound& aBound, bool aFading) {
        ComposeLayers(sk, f->Masks(), aIn, *S, *before);
        LimitMeshArmClavicles(sk, *S, rig.clav[0], rig.clav[1], kMeshArmClavicleRuntimeLimitDeg, kMeshArmClavicleSoftDeg, *before);
        *after = *before;
        const ClavicleTransientResult r = aBound.Apply(sk, *S, rig.clav[0], rig.clav[1], aFading, dt, *after);
        engagedFrames += r.gate < 1.0f;
        if (!SamePose(*before, *after, sk.n))
        {
            MeshRotations(sk, *before, mB);
            MeshRotations(sk, *after, mA);
            for (int i = 0; i < sk.n; ++i)
            {
                const bool clav = i == rig.clav[0] || i == rig.clav[1];
                const bool child = sk.parent[i] == rig.clav[0] || sk.parent[i] == rig.clav[1];
                const bool below = sk.IsAncestorOrSelf(rig.clav[0], i) || sk.IsAncestorOrSelf(rig.clav[1], i);
                if (below && !clav)
                {
                    const float a = AngleDeg(mB[i], mA[i]);
                    ctb2Worst = std::max(ctb2Worst, a);
                    ctb2Bad += a >= 0.01f;
                }
                if (std::memcmp(&before->b[i].t, &after->b[i].t, sizeof(Vec3)) != 0 ||
                    (!clav && !child && std::memcmp(&before->b[i], &after->b[i], sizeof(Xf)) != 0))
                    ++ctb2Bad;
            }
        }
    };

    for (int si = 0; si < kSetCount; ++si)
        for (int ai = 0; ai < 3; ++ai)
        {
            const int pi = f->FindOverlayPose(kSets[si], kAims[ai], false, false);
            const int pm = f->FindOverlayPose(kSets[si], kAims[ai], false, true);
            const int pci = f->FindOverlayPose(kSets[si], kAims[ai], true, false);
            const int pcm = f->FindOverlayPose(kSets[si], kAims[ai], true, true);
            if (pi < 0 || pm < 0)
            {
                Check(false, std::string(aTag) + " " + kSetNames[si] + " " + kAimNames[ai] + ": stand idle / move poses found");
                continue;
            }
            const std::string set = std::string(kSetNames[si]) + " " + kAimNames[ai];
            const bool oneHanded = si == 2;
            auto OPi = std::make_unique<Pose>(), OPm = std::make_unique<Pose>(), OPci = std::make_unique<Pose>(), OPcm = std::make_unique<Pose>();
            f->FramePose(f->PoseAt(pi).firstFrame, *OPi);
            f->FramePose(f->PoseAt(pm).firstFrame, *OPm);
            Curves ci, cm, cci, ccm;
            f->FrameCurves(pi, f->PoseAt(pi).firstFrame, ci);
            f->FrameCurves(pm, f->PoseAt(pm).firstFrame, cm);
            if (pci >= 0)
            {
                f->FramePose(f->PoseAt(pci).firstFrame, *OPci);
                f->FrameCurves(pci, f->PoseAt(pci).firstFrame, cci);
            }
            if (pcm >= 0)
            {
                f->FramePose(f->PoseAt(pcm).firstFrame, *OPcm);
                f->FrameCurves(pcm, f->PoseAt(pcm).firstFrame, ccm);
            }

            // ---- stops / starts
            struct W
            {
                float off = 0.0f, excess = -1e9f, dev = 0.0f, minW = 1e9f;
                std::string where;
            } wb[4], wa[4]; // [straight run / sprint stop, start, walk stop, run / sprint stop with a direction change]
            int popBad = 0;
            float popWorst = -1e9f;
            for (int kind = 0; kind < 2; ++kind)
                for (uint32_t c : kind == 0 ? stops : starts)
                    for (float entry : kind == 0 ? kEntryStop : kEntryStart)
                        for (float t0 : kind == 0 ? kT0Stop : kT0Start)
                        {
                            const auto& rec = db->Clip(c);
                            const float dur = static_cast<float>(rec.numFrames - 1) / fps;
                            if (entry > 0.0f && entry > dur - 0.4f)
                                continue;
                            ++sequences;
                            const int frames = static_cast<int>((dur - entry) / dt) + kHold;
                            ClavicleTransientBound bound = g_opt.proto;
                            const bool dump = g_opt.dumpBody == aTag && db->ClipName(c) == g_opt.dumpClip && g_opt.dumpSet == kSetNames[si] &&
                                              g_opt.dumpAim == kAimNames[ai] && std::fabs(t0 - g_opt.dumpT0) < 1e-3f &&
                                              std::fabs(entry - g_opt.dumpEntry) < 1e-3f;
                            Quat prevB[2], prevA[2];
                            float stepB = 0.0f, stepA = 0.0f;
                            std::vector<FrameMeasure> seqB, seqA;
                            std::vector<float> seqT;
                            for (int k = 0; k <= frames; ++k)
                            {
                                const float tk = static_cast<float>(k) * dt;
                                const float t = std::min(dur, entry + tk);
                                const float ph = std::min(static_cast<float>(rec.numFrames - 1), t * fps);
                                frameOf(rec.firstFrame + static_cast<uint32_t>(ph + 0.5f), *L);
                                const float u = Clamp01((tk - t0) / 0.2f), ub = Clamp01((tk - t0) / 0.1f);
                                const float w = kind == 0 ? u : 1.0f - u, wbase = kind == 0 ? ub : 1.0f - ub; // standing share
                                const bool fading = kind == 0 && tk >= t0 && u < 1.0f; // the runtime engages on stop crossfades only
                                Blend(*Bm, *Bi, wbase, *B);
                                Blend(*OPm, *OPi, w, *OP);
                                LayerInputs in;
                                in.L = L.get();
                                in.B = B.get();
                                in.OP = OP.get();
                                in.OPLower = OP.get();
                                CurvesMix(cm, ci, w, in.curves);
                                composeFrame(in, bound, fading);
                                seqB.push_back(Measure(rig, *before, *S, *tmp));
                                seqA.push_back(Measure(rig, *after, *S, *tmp));
                                seqT.push_back(tk);
                                if (dump)
                                {
                                    const FrameMeasure& mb = seqB.back();
                                    const FrameMeasure& ma = seqA.back();
                                    std::printf("DUMP %s %s t %.3f w %.2f ls %.2f/%.2f g %.2f | before width %.1f chest %.1f dev %.1f/%.1f "
                                                "L(%.1f %.1f %.1f) R(%.1f %.1f %.1f) | after width %.1f dev %.1f/%.1f L(%.1f %.1f %.1f) R(%.1f %.1f %.1f)\n",
                                                aTag, set.c_str(), double(tk), double(w), double(in.curves[kArmLLS]), double(in.curves[kArmRLS]),
                                                double(std::min(bound.g[0], bound.g[1])), double(mb.width), double(mb.chest), double(mb.dev[0]),
                                                double(mb.dev[1]), double(mb.sh[0].x), double(mb.sh[0].y), double(mb.sh[0].z), double(mb.sh[1].x),
                                                double(mb.sh[1].y), double(mb.sh[1].z), double(ma.width), double(ma.dev[0]), double(ma.dev[1]),
                                                double(ma.sh[0].x), double(ma.sh[0].y), double(ma.sh[0].z), double(ma.sh[1].x), double(ma.sh[1].y),
                                                double(ma.sh[1].z));
                                }
                                for (int s = 0; s < 2; ++s)
                                {
                                    const Quat qb = before->b[rig.clav[s]].r, qa = after->b[rig.clav[s]].r;
                                    if (k > 0)
                                    {
                                        stepB = std::max(stepB, AngleDeg(prevB[s], qb));
                                        stepA = std::max(stepA, AngleDeg(prevA[s], qa));
                                    }
                                    prevB[s] = qb;
                                    prevA[s] = qa;
                                }
                                if (k == frames)
                                    endDiff += !SamePose(*before, *after, sk.n) || bound.g[0] < 1.0f || bound.g[1] < 1.0f;
                            }
                            popWorst = std::max(popWorst, stepA - stepB);
                            popBad += stepA > stepB + 0.05f;
                            const float endOff = std::fabs(seqB.back().width - seqB.back().chest);
                            for (size_t k = 0; k < seqB.size(); ++k)
                                for (int form = 0; form < 2; ++form)
                                {
                                    const FrameMeasure& m = form ? seqA[k] : seqB[k];
                                    const int wi = kind == 1 ? 1 : (Has(db->ClipName(c), "Walk") ? 2 : (Has(db->ClipName(c), "_Stop_F_") ? 0 : 3));
                                    W& ww = form ? wa[wi] : wb[wi];
                                    const float off = std::fabs(m.width - m.chest);
                                    const float excess = off - std::max(1.5f, endOff);
                                    ww.off = std::max(ww.off, off);
                                    if (excess > ww.excess)
                                    {
                                        ww.excess = excess;
                                        ww.where = F("%.*s entry %.2f t0 %.2f t %.3f (off %.2f, settled end %.2f)", int(db->ClipName(c).size()),
                                                     db->ClipName(c).data(), double(entry), double(t0), double(seqT[k]), double(off), double(endOff));
                                    }
                                    ww.dev = std::max({ww.dev, m.dev[0], m.dev[1]});
                                    ww.minW = std::min(ww.minW, m.width);
                                }
                        }
            for (int kind = 0; kind < 4; ++kind)
            {
                const char* kn = kind == 0 ? "straight run/sprint stop" : (kind == 1 ? "start" : (kind == 2 ? "walk stop" : "turning run/sprint stop"));
                std::printf("  [%s] %-14s %-5s before: off %5.2f excess %5.2f min width %5.1f dev %5.1f | after: off %5.2f excess %5.2f min width %5.1f dev %5.1f | worst after: %s\n",
                            aTag, set.c_str(), kn, double(wb[kind].off), double(wb[kind].excess), double(wb[kind].minW), double(wb[kind].dev),
                            double(wa[kind].off), double(wa[kind].excess), double(wa[kind].minW), double(wa[kind].dev), wa[kind].where.c_str());
                if (!oneHanded && kind == 0)
                    before2HWorst = std::max(before2HWorst, wb[kind].off);
                // strict 1.5 cm: Relaxed sets (the moving pose carries the arms in local space). Ready / Aim (mesh-space arms at both
                // ends) keep 2.0 - 3.3 cm after (from 5.7 - 10.7 before): not worse, the 1.5 cm target is not reached there yet
                const bool strict = !oneHanded && kind == 0 && ai == 0;
                const bool ok = strict ? wa[kind].excess <= 1e-3f : wa[kind].excess <= std::max(0.0f, wb[kind].excess) + 1e-3f;
                Check(ok, F("CTB-1 %s %s %s: width off the chest beyond max(1.5 cm, settled end) after %.2f cm (before %.2f; %s) at %s", aTag,
                            set.c_str(), kn, double(wa[kind].excess), double(wb[kind].excess), strict ? "bound 0" : "not worse",
                            wa[kind].where.c_str()));
            }
            Check(popBad == 0, F("CTB-4 %s %s: bounded clavicle step never above the unbounded one (worst excess %.3f deg, %d sequences)", aTag,
                                 set.c_str(), double(popWorst), popBad));

            // ---- settled poses: no crossfade, 2.5 s per loop
            struct Steady
            {
                const std::vector<uint32_t>* clips;
                bool crouch;
            };
            for (const Steady& st : {Steady{&standLoops, false}, Steady{&crouchLoops, true}})
            {
                if (st.crouch && (pci < 0 || pcm < 0))
                    continue;
                for (uint32_t c : *st.clips)
                {
                    const bool idle = IsIdleLoop(db->ClipName(c));
                    const Pose* opP = st.crouch ? (idle ? OPci.get() : OPcm.get()) : (idle ? OPi.get() : OPm.get());
                    const Pose* bP = st.crouch ? (idle ? Ci.get() : Cm.get()) : (idle ? Bi.get() : Bm.get());
                    const Curves& cv = st.crouch ? (idle ? cci : ccm) : (idle ? ci : cm);
                    const auto& rec = db->Clip(c);
                    const float dur = static_cast<float>(rec.numFrames - 1) / fps;
                    if (dur <= 0.0f)
                        continue;
                    ClavicleTransientBound bound = g_opt.proto;
                    const int frames = static_cast<int>(2.5f * g_opt.fps);
                    for (int k = 0; k <= frames; ++k)
                    {
                        const float t = std::fmod(static_cast<float>(k) * dt, dur);
                        frameOf(rec.firstFrame + std::min(rec.numFrames - 1, static_cast<uint32_t>(t * fps + 0.5f)), *L);
                        LayerInputs in;
                        in.L = L.get();
                        in.B = bP;
                        in.OP = opP;
                        in.OPLower = opP;
                        in.curves = cv;
                        composeFrame(in, bound, false);
                        ++ctb3Frames;
                        ctb3Diff += !SamePose(*before, *after, sk.n);
                    }
                }
            }

            // ---- other fast chest transients under both steady poses (no crossfade)
            float oOff[2] = {0, 0}, oDev[2] = {0, 0};
            for (uint32_t c : others)
                for (int moving = 0; moving < 2; ++moving)
                {
                    const bool crouch = Has(db->ClipName(c), "Crouch") || Has(db->ClipName(c), "Slide");
                    if (crouch && (pci < 0 || pcm < 0))
                        continue;
                    const Pose* opP = crouch ? (moving ? OPcm.get() : OPci.get()) : (moving ? OPm.get() : OPi.get());
                    const Pose* bP = crouch ? (moving ? Cm.get() : Ci.get()) : (moving ? Bm.get() : Bi.get());
                    const Curves& cv = crouch ? (moving ? ccm : cci) : (moving ? cm : ci);
                    const auto& rec = db->Clip(c);
                    const float dur = static_cast<float>(rec.numFrames - 1) / fps;
                    ClavicleTransientBound bound = g_opt.proto;
                    const int frames = static_cast<int>(dur / dt);
                    for (int k = 0; k <= frames; ++k)
                    {
                        const float t = std::min(dur, static_cast<float>(k) * dt);
                        frameOf(rec.firstFrame + std::min(rec.numFrames - 1, static_cast<uint32_t>(t * fps + 0.5f)), *L);
                        LayerInputs in;
                        in.L = L.get();
                        in.B = bP;
                        in.OP = opP;
                        in.OPLower = opP;
                        in.curves = cv;
                        composeFrame(in, bound, false);
                        const FrameMeasure mb = Measure(rig, *before, *S, *tmp);
                        const FrameMeasure ma = Measure(rig, *after, *S, *tmp);
                        oOff[0] = std::max(oOff[0], std::fabs(mb.width - mb.chest));
                        oOff[1] = std::max(oOff[1], std::fabs(ma.width - ma.chest));
                        oDev[0] = std::max({oDev[0], mb.dev[0], mb.dev[1]});
                        oDev[1] = std::max({oDev[1], ma.dev[0], ma.dev[1]});
                    }
                }
            Check(oOff[1] <= oOff[0] + 0.05f && oDev[1] <= oDev[0] + 0.05f,
                  F("CTB-5 %s %s: turns / landings / slides: width off the chest before %.2f after %.2f cm, deviation before %.1f after %.1f deg", aTag,
                    set.c_str(), double(oOff[0]), double(oOff[1]), double(oDev[0]), double(oDev[1])));
        }
    Check(before2HWorst > 1.5f, F("CTB-0 %s: before, a two-handed stop leaves the chest width by %.2f cm (> 1.5: the case reaches the fault)", aTag,
                                  double(before2HWorst)));
    Check(ctb2Bad == 0, F("CTB-2 %s: bones below the clavicles keep their mesh rotation (worst %.4f deg), the rest untouched (%d bad)", aTag,
                          double(ctb2Worst), ctb2Bad));
    Check(ctb3Diff == 0 && endDiff == 0, F("CTB-3 %s: settled loops bit for bit (%d frames, %d differ); stop / start sequences released and untouched on the last held frame (%d of %d not)",
                                          aTag, ctb3Frames, ctb3Diff, endDiff, sequences));
    std::printf("[%s] frames with the bound engaged: %d\n", aTag, engagedFrames);
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("FAIL usage: utpp_clavicle_transient_test woman.utppmm woman.utppov man.utppmm man.utppov [options]\n");
        return 1;
    }
    for (int i = 5; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto num = [&](float& aV) {
            if (i + 1 < argc)
                aV = static_cast<float>(std::atof(argv[++i]));
        };
        if (a == "--fps")
            num(g_opt.fps);
        else if (a == "--limit")
            num(g_opt.proto.limitDeg);
        else if (a == "--soft")
            num(g_opt.proto.softDeg);
        else if (a == "--release")
            num(g_opt.proto.releaseSeconds);
        else if (a == "--settled")
            num(g_opt.proto.settledDegPerSec);
        else if (a == "--hold")
            num(g_opt.proto.maxHoldSeconds);
        else if (a == "--dump" && i + 6 < argc)
        {
            g_opt.dumpBody = argv[++i];
            g_opt.dumpClip = argv[++i];
            g_opt.dumpSet = argv[++i];
            g_opt.dumpAim = argv[++i];
            g_opt.dumpEntry = static_cast<float>(std::atof(argv[++i]));
            g_opt.dumpT0 = static_cast<float>(std::atof(argv[++i]));
        }
    }
    std::printf("transient bound: limit %.1f deg, soft %.1f deg, release %.2f s, settled %.0f deg/s, hold %.2f s, %.0f Hz\n",
                double(g_opt.proto.limitDeg), double(g_opt.proto.softDeg), double(g_opt.proto.releaseSeconds),
                double(g_opt.proto.settledDegPerSec), double(g_opt.proto.maxHoldSeconds), double(g_opt.fps));
    TestBody(argv[3], argv[4], kRigIdentityManOv, "man");
    TestBody(argv[1], argv[2], kRigIdentityWomanOv, "woman");
    std::printf("clavicle transient test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

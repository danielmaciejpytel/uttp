// Swing gate of the weapon pose's legs layer (src/mm/Overlay.hpp LegSwingGate + ComposeLayers legsShare).
//
//   utpp_mm_overlay_legs_swing_test woman.utppmm woman.utppov man.utppmm man.utppov
//
// Real data of both bodies. The locomotion pose L plays each standing stop and start clip (jump starts excluded: in the air the leg
// layer is faded by its own factor) at 60 Hz and then holds its last frame for 0.6 s; the foot contacts are the clip's contact
// curves. The overlay crossfades between its moving and idle stance pose (a stop: moving -> idle from the time t0, a start: idle ->
// moving; overlay pose and layer curves over 0.2 s, base pose over 0.1 s, as OverlayTimes poseSwitch / moveSwitch) for the rifle and
// both pistol weapon pose sets in every aim state, t0 = 0, 0.1, 0.2, 0.3 s on a stop, 0 and 2 frames on a start. Each frame is composed
// ungated (shares 1, the composition before the gate) and gated (the same LegSwingGate object type as the runtime, updated with the
// crossfade's legs weight, the contacts and the knee flex difference of L against B). Knee metric: the shin's local rotation against the rest pose, intrinsic ZYX (roll = |x|, side bend = |y|), each leg.
//   OVLEGS-SWING-0  the ungated composition leaves the knee band on these sequences (the case reaches the fault);
//   OVLEGS-SWING-1  gated: per frame and leg, the excess of the shin roll and of the knee side bend over the largest of the body's
//                   band, the source frame's own value and the settled stance's own value (the ungated composition of the idle clip
//                   frames under the idle weapon pose, max per leg). Never more frames outside and never a larger excess than the
//                   ungated composition. Rifle and two-handed pistol sets: worst excess <= 1.5 deg (a few walk stop frames).
//                   One-handed pistol sets (their Ready / Aim base pose knees are far from a hinge, the gate cannot remove all of
//                   it): stops <= 12.5 deg with at most 15 % of the ungated frames left outside, starts <= 4.5 deg;
//   OVLEGS-SWING-2  every frame on which both legs keep share 1 is the ungated composition bit for bit, and every sequence ends with
//                   both shares at 1 (the gate releases; the held frames are settled frames);
//   OVLEGS-SWING-3  the settled stance is untouched: the standing idle clips found (the idle loop) under the idle weapon pose with a
//                   steady legs weight give the ungated composition bit for bit on every frame;
//   OVLEGS-SWING-4  the largest per-frame step of a leg's effective legs weight (share x w) is <= 1 / 0.2 s x dt + 1e-4 on a rise and
//                   <= the faster fall (kFallRate x dt) or the crossfade's own step on a fall.
// Printed for review: per set worst knee ungated / gated, planted ankle (contact > 0.5) change gated vs ungated (cm, height), the knee
// flex difference of the idle loop against the base pose (the clip poses carry root motion, so a planted ankle speed is not measured).
// Mutation UTPP_OVLEGS_SWING_MUTATION=1 (no gate) must print FAIL OVLEGS-SWING-1.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/Overlay.hpp"
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

struct KneeBand
{
    float roll[2]; // left, right (deg)
    float dev[2];
};
// Upper ends of the knee bands of the body shape survey of each body (the game's own animations of that body).
static const KneeBand kBandMan{{9.37f, 9.32f}, {21.89f, 23.17f}};
static const KneeBand kBandWoman{{9.39f, 9.38f}, {24.67f, 24.63f}};

struct Knee
{
    float roll = 0.0f, dev = 0.0f, flex = 0.0f;
};
static Knee KneeOf(const Quat& aRest, const Quat& aLocal)
{
    const Quat q = math::QuatNormalize(math::QuatMul(math::QuatInv(aRest), aLocal));
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    const float m00 = 1 - 2 * (y * y + z * z), m10 = 2 * (x * y + z * w), m20 = 2 * (x * z - y * w);
    const float m21 = 2 * (y * z + x * w), m22 = 1 - 2 * (x * x + y * y);
    const float r2d = 180.0f / math::kPi;
    Knee k;
    k.roll = std::fabs(std::atan2(m21, m22)) * r2d;
    k.dev = std::fabs(std::asin(std::clamp(-m20, -1.0f, 1.0f))) * r2d;
    k.flex = -std::atan2(m10, m00) * r2d;
    return k;
}

static bool Has(std::string_view n, const char* s) { return n.find(s) != std::string_view::npos; }
static bool IsStandStop(std::string_view n) { return !Has(n, "Crouch") && !Has(n, "Jump") && Has(n, "Stop"); }
static bool IsStandStart(std::string_view n) { return !Has(n, "Crouch") && !Has(n, "Jump") && Has(n, "Start"); }
static bool IsStandIdleFamily(std::string_view n)
{
    return Has(n, "Stand_Idle") && !Has(n, "Crouch") && !Has(n, "Slide") && !Has(n, "Start") && !Has(n, "Stop") && !Has(n, "Jump");
}
static void CurvesMix(const Curves& aA, const Curves& aB, float aW, Curves& aOut)
{
    aOut = Curves{};
    aOut.Accumulate(aA, 1.0f - aW);
    aOut.Accumulate(aB, aW);
}
static bool SamePose(const Pose& a, const Pose& b, int n) { return std::memcmp(a.b, b.b, sizeof(Xf) * static_cast<size_t>(n)) == 0; }

static void TestBody(const char* aDb, const char* aOv, uint64_t aHash, const KneeBand& aBand, const char* aTag)
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
    const int leg[2] = {bone("LeftLeg"), bone("RightLeg")}, foot[2] = {bone("LeftFoot"), bone("RightFoot")};
    const int thigh[2] = {bone("LeftUpLeg"), bone("RightUpLeg")};
    const bool bonesOk = leg[0] > 0 && leg[1] > 0 && foot[0] > 0 && foot[1] > 0 && thigh[0] > 0 && thigh[1] > 0;
    Check(bonesOk, std::string(aTag) + ": leg bones found");
    if (!bonesOk)
        return;
    Quat rest[2];
    for (int s = 0; s < 2; ++s)
    {
        const auto& r = db->RigRest(static_cast<uint32_t>(leg[s]));
        rest[s] = math::QuatNormalize(Quat{r.qx, r.qy, r.qz, r.qw});
    }
    auto frameOf = [&](uint32_t aF, Pose& aOut) {
        const motion::PoseRecord* p = db->Pose(aF);
        aOut.n = sk.n;
        for (int i = 0; i < sk.n; ++i)
            aOut.b[i] = Xf{math::QuatNormalize(Quat{p[i].qx, p[i].qy, p[i].qz, p[i].qw}), Vec3{p[i].px, p[i].py, p[i].pz}, Vec3{1, 1, 1}};
    };
    auto knees = [&](const Pose& aP, Knee* aK) {
        for (int s = 0; s < 2; ++s)
            aK[s] = KneeOf(rest[s], aP.b[leg[s]].r);
    };

    std::vector<uint32_t> stops, starts, idles;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string_view n = db->ClipName(c);
        if (IsStandStop(n))
            stops.push_back(c);
        else if (IsStandStart(n))
            starts.push_back(c);
        else if (IsStandIdleFamily(n))
            idles.push_back(c);
    }
    std::printf("[%s] standing stop clips %zu, start clips %zu, standing idle family clips %zu\n", aTag, stops.size(), starts.size(),
                idles.size());
    Check(stops.size() >= 4 && starts.size() >= 4 && !idles.empty(), std::string(aTag) + ": stop, start and idle clips present");

    const int pBi = f->FindPose("Pose_Neutral_Stand_Idle"), pBm = f->FindPose("Pose_Neutral_Stand_Move");
    Check(pBi >= 0 && pBm >= 0, std::string(aTag) + ": base poses found");
    if (pBi < 0 || pBm < 0)
        return;
    auto Bi = std::make_unique<Pose>(), Bm = std::make_unique<Pose>();
    f->FramePose(f->PoseAt(pBi).firstFrame, *Bi);
    f->FramePose(f->PoseAt(pBm).firstFrame, *Bm);

    const float fps = db->Header().fps;
    const float dt = 1.0f / 60.0f;
    const PoseOverlay kSets[] = {PoseOverlay::Rifle, PoseOverlay::Pistol2H, PoseOverlay::Pistol1H};
    const char* kSetNames[] = {"Rifle", "Pistol2H", "Pistol1H"};
    const AimState kAims[] = {AimState::Relaxed, AimState::Ready, AimState::Aiming};
    const char* kAimNames[] = {"Relax", "Ready", "Aim"};
    // crossfade start after the clip start: a stop 0 - 0.3 s; a start 0 or 2 frames (the moving flag and the start clip begin
    // together; while the crossfade has not begun the legs weight is steady and the gate does not act)
    const float kT0[] = {0.0f, 0.1f, 0.2f, 0.3f};
    const std::vector<float> kT0Start = {0.0f, 2.0f / 60.0f};
    const float kneeFree = LegSwingGate{}.kneeFreeDeg; // the runtime value
    std::printf("[%s] knee free flex difference %.1f deg\n", aTag, double(kneeFree));
    float idleFlexDiff = 0.0f;
    auto kneeDelta = [&](const Pose& aL, const Pose& aB, float* aOut) {
        for (int s = 0; s < 2; ++s)
            aOut[s] = std::fabs(KneeFlexDeg(rest[s], aL.b[leg[s]].r) - KneeFlexDeg(rest[s], aB.b[leg[s]].r));
    };

    auto L = std::make_unique<Pose>(), B = std::make_unique<Pose>(), OPi = std::make_unique<Pose>(), OPm = std::make_unique<Pose>(),
         OP = std::make_unique<Pose>(), outOld = std::make_unique<Pose>(), outNew = std::make_unique<Pose>();
    auto S = std::make_unique<LayerScratch>();
    Vec3 posO[ov::kMaxBones], posN[ov::kMaxBones];
    Quat rotO[ov::kMaxBones], rotN[ov::kMaxBones];
    int ungatedOutside = 0, totalOutside = 0;
    float plantCm = 0.0f, plantZcm = 0.0f, riseStep = 0.0f, fallStep = 0.0f;
    float toLMax[2] = {0, 0}, toLSum[2] = {0, 0}, sinkMax[2] = {0, 0}; // planted ankle against L: [ungated, gated]
    int toLN = 0;
    Vec3 posL[ov::kMaxBones];
    Quat rotL[ov::kMaxBones];
    std::string plantWhere;
    int changedFrames = 0, changedPlanted = 0, sharedFrames = 0, sharedDiff = 0, notReleased = 0, steadyFrames = 0, steadyDiff = 0;
    const int kHold = 36; // 0.6 s at 60 Hz

    for (int si = 0; si < 3; ++si)
        for (int ai = 0; ai < 3; ++ai)
        {
            const int pi = f->FindOverlayPose(kSets[si], kAims[ai], false, false);
            const int pm = f->FindOverlayPose(kSets[si], kAims[ai], false, true);
            if (pi < 0 || pm < 0)
            {
                Check(false, std::string(aTag) + " " + kSetNames[si] + " " + kAimNames[ai] + ": stand idle / move poses found");
                continue;
            }
            const std::string set = std::string(kSetNames[si]) + " " + kAimNames[ai];
            f->FramePose(f->PoseAt(pi).firstFrame, *OPi);
            f->FramePose(f->PoseAt(pm).firstFrame, *OPm);
            Curves ci, cm;
            f->FrameCurves(pi, f->PoseAt(pi).firstFrame, ci);
            f->FrameCurves(pm, f->PoseAt(pm).firstFrame, cm);
            {
                const int pci = f->FindOverlayPose(kSets[si], kAims[ai], true, false), pcm = f->FindOverlayPose(kSets[si], kAims[ai], true, true);
                Curves cci, ccm;
                if (pci >= 0)
                    f->FrameCurves(pci, f->PoseAt(pci).firstFrame, cci);
                if (pcm >= 0)
                    f->FrameCurves(pcm, f->PoseAt(pcm).firstFrame, ccm);
                std::printf("  [%s] %s %s crouch legs layering idle %.2f move %.2f\n", aTag, kSetNames[si], kAimNames[ai], double(cci[kLegs]),
                            double(ccm[kLegs]));
            }

            // settled stance: idle family clips under the idle weapon pose, steady legs weight, gate updated with the clip contacts
            Knee settled[2] = {};
            for (uint32_t c : idles)
            {
                LegSwingGate gate;
                gate.kneeFreeDeg = kneeFree;
                LayerInputs in;
                in.B = Bi.get();
                in.OP = OPi.get();
                in.OPLower = OPi.get();
                in.curves = ci;
                const auto& rec = db->Clip(c);
                for (uint32_t i = 0; i < rec.numFrames; ++i)
                {
                    frameOf(rec.firstFrame + i, *L);
                    in.L = L.get();
                    const motion::ContactRecord& ct = db->Contact(rec.firstFrame + i);
                    const float cts[2] = {ct.l, ct.r};
                    float kd[2];
                    kneeDelta(*L, *Bi, kd);
                    gate.Update(in.curves[kLegs], cts, kd, 1.0f / fps);
                    ComposeLayers(sk, f->Masks(), in, *S, *outOld);
                    for (int s = 0; s < 2; ++s)
                    {
                        in.legsThigh[s] = thigh[s];
                        in.legsShare[s] = gate.Share(s, in.curves[kLegs]);
                    }
                    ComposeLayers(sk, f->Masks(), in, *S, *outNew);
                    in.legsShare[0] = in.legsShare[1] = 1.0f;
                    ++steadyFrames;
                    steadyDiff += !SamePose(*outOld, *outNew, sk.n);
                    if (Has(db->ClipName(c), "Loop"))
                    {
                        Knee k[2];
                        knees(*outOld, k);
                        for (int s = 0; s < 2; ++s)
                        {
                            settled[s].roll = std::max(settled[s].roll, k[s].roll);
                            settled[s].dev = std::max(settled[s].dev, k[s].dev);
                            idleFlexDiff = std::max(idleFlexDiff, kd[s]);
                        }
                    }
                }
            }

            struct W
            {
                float roll[2] = {0, 0}, dev[2] = {0, 0}, over = 0.0f;
                int outside = 0, frames = 0;
                std::string where;
            } wOld[2], wNew[2]; // [stop, start]
            for (int kind = 0; kind < 2; ++kind)
                for (uint32_t c : kind == 0 ? stops : starts)
                    for (float t0 : kind == 0 ? std::vector<float>(std::begin(kT0), std::end(kT0)) : kT0Start)
                    {
                        const auto& rec = db->Clip(c);
                        const float dur = static_cast<float>(rec.numFrames - 1) / fps;
                        const int frames = static_cast<int>(dur / dt) + kHold;
                        LegSwingGate gate;
                        gate.kneeFreeDeg = kneeFree;
                        float prevEff[2] = {-1.0f, -1.0f};
                        float prevW = -1.0f;
                        for (int k = 0; k <= frames; ++k)
                        {
                            const float t = std::min(dur, static_cast<float>(k) * dt);
                            const uint32_t fi = std::min(rec.numFrames - 1, static_cast<uint32_t>(t * fps + 0.5f));
                            frameOf(rec.firstFrame + fi, *L);
                            const motion::ContactRecord& ct = db->Contact(rec.firstFrame + fi);
                            const float cts[2] = {ct.l, ct.r};
                            const float tk = static_cast<float>(k) * dt;
                            const float u = Clamp01((tk - t0) / 0.2f), ub = Clamp01((tk - t0) / 0.1f);
                            const float w = kind == 0 ? u : 1.0f - u, wb = kind == 0 ? ub : 1.0f - ub; // idle share
                            Blend(*Bm, *Bi, wb, *B);
                            Blend(*OPm, *OPi, w, *OP);
                            LayerInputs in;
                            in.L = L.get();
                            in.B = B.get();
                            in.OP = OP.get();
                            in.OPLower = OP.get();
                            CurvesMix(cm, ci, w, in.curves);
                            const float legsW = in.curves[kLegs];
                            float kd[2];
                            kneeDelta(*L, *B, kd);
                            gate.Update(legsW, cts, kd, dt);
                            ComposeLayers(sk, f->Masks(), in, *S, *outOld);
                            float share[2];
                            for (int s = 0; s < 2; ++s)
                            {
                                in.legsThigh[s] = thigh[s];
                                share[s] = in.legsShare[s] = gate.Share(s, legsW);
                            }
                            ComposeLayers(sk, f->Masks(), in, *S, *outNew);
                            const bool shared = !(share[0] < 1.0f) && !(share[1] < 1.0f);
                            if (shared)
                            {
                                ++sharedFrames;
                                sharedDiff += !SamePose(*outOld, *outNew, sk.n);
                            }
                            else
                            {
                                ++changedFrames;
                                for (int s = 0; s < 2; ++s)
                                    changedPlanted += share[s] < 1.0f && cts[s] >= LegSwingGate::kContactOn;
                            }
                            if (k == frames && !shared)
                                ++notReleased;
                            // weight steps
                            for (int s = 0; s < 2; ++s)
                            {
                                const float eff = share[s] * Clamp01(legsW);
                                if (prevEff[s] >= 0.0f)
                                {
                                    const float d = eff - prevEff[s];
                                    const float own = std::fabs(Clamp01(legsW) - prevW);
                                    if (d > 0.0f)
                                        riseStep = std::max(riseStep, d - std::max(0.0f, Clamp01(legsW) - prevW) * (share[s] >= 1.0f));
                                    else
                                        fallStep = std::max(fallStep, -d - own);
                                }
                                prevEff[s] = eff;
                            }
                            prevW = Clamp01(legsW);
                            // knees
                            Knee ks[2], ko[2], kn[2];
                            knees(*L, ks);
                            knees(*outOld, ko);
                            knees(*outNew, kn);
                            char where[200];
                            std::snprintf(where, sizeof(where), "%.*s t0 %.1f t %.3f", int(db->ClipName(c).size()), db->ClipName(c).data(),
                                          double(t0), double(tk));
                            for (int form = 0; form < 2; ++form)
                            {
                                W& ww = form ? wNew[kind] : wOld[kind];
                                const Knee* kk = form ? kn : ko;
                                ++ww.frames;
                                bool out = false;
                                for (int s = 0; s < 2; ++s)
                                {
                                    ww.roll[s] = std::max(ww.roll[s], kk[s].roll);
                                    ww.dev[s] = std::max(ww.dev[s], kk[s].dev);
                                    const float eR = kk[s].roll - std::max({aBand.roll[s], ks[s].roll, settled[s].roll});
                                    const float eD = kk[s].dev - std::max({aBand.dev[s], ks[s].dev, settled[s].dev});
                                    const float e = std::max(eR, eD);
                                    out = out || e > 0.0f;
                                    if (e > ww.over)
                                    {
                                        ww.over = e;
                                        ww.where = std::string(where) + (s ? " right" : " left");
                                    }
                                }
                                ww.outside += out;
                            }
                            // planted ankles
                            MeshPositions(sk, *outOld, posO, rotO);
                            MeshPositions(sk, *outNew, posN, rotN);
                            const bool layerOn = legsW > 0.0f && (legsW < 1.0f || share[0] < 1.0f || share[1] < 1.0f);
                            if (layerOn)
                                MeshPositions(sk, *L, posL, rotL);
                            for (int s = 0; s < 2; ++s)
                            {
                                const bool planted = cts[s] >= LegSwingGate::kContactOn;
                                if (planted)
                                {
                                    const Vec3 d = posN[foot[s]] - posO[foot[s]];
                                    const float cm1 = 100.0f * std::sqrt(math::Dot(d, d));
                                    if (cm1 > plantCm)
                                    {
                                        plantCm = cm1;
                                        plantWhere = set + " " + where + (s ? " right" : " left");
                                    }
                                    plantZcm = std::max(plantZcm, 100.0f * std::fabs(d.z));
                                    if (layerOn)
                                    {
                                        const Vec3 dl[2] = {posO[foot[s]] - posL[foot[s]], posN[foot[s]] - posL[foot[s]]};
                                        for (int q = 0; q < 2; ++q)
                                        {
                                            const float cmq = 100.0f * std::sqrt(math::Dot(dl[q], dl[q]));
                                            toLMax[q] = std::max(toLMax[q], cmq);
                                            toLSum[q] += cmq;
                                            sinkMax[q] = std::max(sinkMax[q], 100.0f * std::max(0.0f, -dl[q].z));
                                        }
                                        ++toLN;
                                    }
                                }
                            }
                        }
                    }
            for (int kind = 0; kind < 2; ++kind)
            {
                const char* kn = kind ? "start" : "stop";
                std::printf("  [%s] %-14s %-5s legs idle %.2f move %.2f | ungated roll L %5.1f R %5.1f dev L %5.1f R %5.1f outside %5d/%d | "
                            "gated roll L %5.1f R %5.1f dev L %5.1f R %5.1f outside %d | settled roll %.1f/%.1f dev %.1f/%.1f%s%s\n",
                            aTag, set.c_str(), kn, double(ci[kLegs]), double(cm[kLegs]), double(wOld[kind].roll[0]), double(wOld[kind].roll[1]),
                            double(wOld[kind].dev[0]), double(wOld[kind].dev[1]), wOld[kind].outside, wOld[kind].frames, double(wNew[kind].roll[0]),
                            double(wNew[kind].roll[1]), double(wNew[kind].dev[0]), double(wNew[kind].dev[1]), wNew[kind].outside,
                            double(settled[0].roll), double(settled[1].roll), double(settled[0].dev), double(settled[1].dev),
                            wNew[kind].outside ? " | worst " : "", wNew[kind].outside ? wNew[kind].where.c_str() : "");
                ungatedOutside += wOld[kind].outside;
                totalOutside += wNew[kind].outside;
                const bool oneHanded = set.rfind("Pistol1H", 0) == 0;
                const float bound = oneHanded ? (kind == 0 ? 12.5f : 4.5f) : 1.5f;
                const bool notWorse = wNew[kind].outside <= wOld[kind].outside && wNew[kind].over <= wOld[kind].over + 1e-3f;
                const bool mostGone = !oneHanded || kind != 0 || wNew[kind].outside * 100 <= wOld[kind].outside * 15;
                Check(wNew[kind].over <= bound && notWorse && mostGone,
                      "OVLEGS-SWING-1 " + std::string(aTag) + " " + set + " " + kn + ": gated knee against the band / source / settled stance (frames outside " +
                          std::to_string(wNew[kind].outside) + " of " + std::to_string(wNew[kind].frames) + ", ungated " +
                          std::to_string(wOld[kind].outside) + "; worst excess " + std::to_string(wNew[kind].over) + " deg, bound " +
                          std::to_string(bound) + ", ungated " + std::to_string(wOld[kind].over) + " deg; at " + wNew[kind].where + ")");
            }
        }
    Check(ungatedOutside > 0, "OVLEGS-SWING-0 " + std::string(aTag) + ": the ungated composition leaves the band on these sequences (" +
                                  std::to_string(ungatedOutside) + " frames)");
    Check(sharedDiff == 0 && notReleased == 0, "OVLEGS-SWING-2 " + std::string(aTag) + ": frames with both shares 1 are the ungated composition bit for bit (" +
                                                   std::to_string(sharedFrames) + " frames, " + std::to_string(sharedDiff) +
                                                   " differ) and every sequence ends released (" + std::to_string(notReleased) + " not)");
    Check(steadyDiff == 0, "OVLEGS-SWING-3 " + std::string(aTag) + ": settled stance (idle clips, steady legs weight) bit for bit (" +
                               std::to_string(steadyFrames) + " frames, " + std::to_string(steadyDiff) + " differ)");
    const float riseMax = LegSwingGate::kRiseRate * dt + 1e-4f, fallMax = LegSwingGate::kFallRate * dt + 1e-4f;
    Check(riseStep <= riseMax && fallStep <= fallMax,
          "OVLEGS-SWING-4 " + std::string(aTag) + ": effective legs weight step beyond the crossfade's own: rise " + std::to_string(riseStep) +
              " (<= " + std::to_string(riseMax) + "), fall " + std::to_string(fallStep) + " (<= " + std::to_string(fallMax) + ") per 60 Hz frame");
    std::printf("[%s] idle loop knee flex against the stand idle base pose: max difference %.2f deg (gate free below %.1f)\n", aTag,
                double(idleFlexDiff), double(kneeFree));
    std::printf("[%s] planted ankle against L (contact >= 0.5, legs layer on and fading or gated, n %d): ungated max %.2f cm mean %.2f cm sink %.2f cm | "
                "gated max %.2f cm mean %.2f cm sink %.2f cm\n", aTag, toLN, double(toLMax[0]), double(toLN ? toLSum[0] / float(toLN) : 0.0f),
                double(sinkMax[0]), double(toLMax[1]), double(toLN ? toLSum[1] / float(toLN) : 0.0f), double(sinkMax[1]));
    std::printf("[%s] gated frames %d (leg planted on %d of their leg-frames), shared frames %d; planted ankle gated vs ungated max %.2f cm "
                "(height %.2f cm) at %s\n",
                aTag, changedFrames, changedPlanted, sharedFrames, double(plantCm), double(plantZcm), plantWhere.c_str());
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("FAIL usage: utpp_mm_overlay_legs_swing_test woman.utppmm woman.utppov man.utppmm man.utppov\n");
        return 1;
    }
    TestBody(argv[3], argv[4], kRigIdentityManOv, kBandMan, "man");
    TestBody(argv[1], argv[2], kRigIdentityWomanOv, kBandWoman, "woman");
    std::printf("overlay legs swing test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

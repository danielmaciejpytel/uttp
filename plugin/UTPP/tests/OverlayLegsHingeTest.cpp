// Knee hinge of the weapon pose's legs region (src/mm/Overlay.hpp ComposeLayers, legs region = Blend(L, OP + D(B -> L), c[Layering_Legs])).
//
//   utpp_mm_overlay_legs_hinge_test woman.utppmm woman.utppov man.utppmm man.utppov [--all]
//
// Real data of both bodies: rest pose and locomotion frames from the motion database, base / overlay poses and layer curves from the
// overlay file. The locomotion pose L runs through the standing stop and start clips (every second frame; --all: every frame), a leg
// in swing on many of them; the overlay crossfades between its moving and idle stance pose (base pose, overlay pose of the legs and the
// layer curves on the same weight w), for the rifle and both pistol weapon pose sets in every aim state; the stance fade runs the per
// stance side path at crouch share 0.5. Measured on the composed legs: the shin's local rotation against the rest pose, intrinsic ZYX
// (roll = |x|, knee side bend = |y|, knee flex = -z), each leg; the thigh twist the same way on the thigh.
// Three forms of the legs region are composed on the same frame: Hinge (the runtime form), Mesh (the earlier form), Local.
//   OVLEGS-HINGE-0  the earlier form leaves the knee band on these frames (the case reaches the fault);
//   OVLEGS-HINGE-1  stops / starts: the composed shin roll and knee side bend stay inside the band of the body, or within 0.5 deg of
//                   the larger of the source frame's own value and the knee offset of the weapon pose against the base pose
//                   (B_knee^-1 * OP_knee, which a hinge composition carries unchanged at any flex), on every frame, both legs, every
//                   weapon pose set; the offsets above the band are printed;
//   OVLEGS-HINGE-2  the settled stance (idle clips under the idle weapon pose) changes by <= 1 deg at the knees and <= 1 cm at the ankles;
//   OVLEGS-HINGE-3  the same as 1 in the stance fade;
//   OVLEGS-HINGE-4  the shins found on the baked masks are LeftLeg and RightLeg.
// Printed for review: layer curves and knee offsets of the poses, thigh twist, the planted ankle / foot against L (contact > 0.5,
// 0 < legs layering < 1) per form, knee flex ranges.
// Mutation UTPP_OVLEGS_HINGE_MUTATION=1 (the mesh-space legs region) must print FAIL OVLEGS-HINGE-1 man Rifle Relax.
// Built on request only (tests/overlay.cmake): the runtime keeps the Mesh form, and the Hinge form fails OVLEGS-HINGE-2.

#include <algorithm>
#include <cmath>
#include <cstdio>
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
using Form = LayerInputs::LegsForm;

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

struct Worst
{
    float roll[2] = {0, 0}, dev[2] = {0, 0}, flexMax[2] = {-1e9f, -1e9f}, twist[2] = {0, 0};
    float over = 0.0f; // largest excess over the allowed value (deg)
    int frames = 0, outside = 0;
    std::string where;
    void Add(const Knee* aK, const Knee* aThigh, const Knee* aAllow, const KneeBand& aB, const std::string& aWhere)
    {
        ++frames;
        bool out = false;
        for (int s = 0; s < 2; ++s)
        {
            roll[s] = std::max(roll[s], aK[s].roll);
            dev[s] = std::max(dev[s], aK[s].dev);
            flexMax[s] = std::max(flexMax[s], aK[s].flex);
            twist[s] = std::max(twist[s], aThigh[s].roll);
            const float eR = aK[s].roll - std::max(aB.roll[s], aAllow[s].roll + 0.5f);
            const float eD = aK[s].dev - std::max(aB.dev[s], aAllow[s].dev + 0.5f);
            const float e = std::max(eR, eD);
            if (e > 0.0f)
                out = true;
            if (e > over)
            {
                over = e;
                where = aWhere + (s ? " right" : " left");
            }
        }
        outside += out;
    }
    void Print(const char* aTag) const
    {
        std::printf("    %-30s frames %6d outside %5d | roll L %5.1f R %5.1f | dev L %5.1f R %5.1f | flex max L %6.1f R %6.1f | thigh twist L %5.1f R %5.1f%s%s\n",
                    aTag, frames, outside, double(roll[0]), double(roll[1]), double(dev[0]), double(dev[1]), double(flexMax[0]),
                    double(flexMax[1]), double(twist[0]), double(twist[1]), outside ? " | worst " : "", outside ? where.c_str() : "");
    }
};

struct Plant // planted foot against L
{
    float maxCm = 0.0f, sumCm = 0.0f, maxDeg = 0.0f, maxSinkCm = 0.0f;
    int n = 0;
    void Add(float aCm, float aDeg, float aSinkCm)
    {
        maxCm = std::max(maxCm, aCm);
        sumCm += aCm;
        maxDeg = std::max(maxDeg, aDeg);
        maxSinkCm = std::max(maxSinkCm, aSinkCm);
        ++n;
    }
};

static bool Has(std::string_view n, const char* s) { return n.find(s) != std::string_view::npos; }
static bool IsStandStopStart(std::string_view n) { return !Has(n, "Crouch") && (Has(n, "Stop") || Has(n, "Start")); }
static bool IsStandIdle(std::string_view n)
{
    return Has(n, "Stand_Idle") && !Has(n, "Crouch") && !Has(n, "Break") && !Has(n, "Turn") && !Has(n, "Slide") && !Has(n, "_To_") &&
           !Has(n, "_to_") && !Has(n, "Transition") && !Has(n, "Start") && !Has(n, "Stop");
}

static void CurvesMix(const Curves& aA, const Curves& aB, float aW, Curves& aOut)
{
    aOut = Curves{};
    aOut.Accumulate(aA, 1.0f - aW);
    aOut.Accumulate(aB, aW);
}

static void TestBody(const char* aDb, const char* aOv, uint64_t aHash, const KneeBand& aBand, const char* aTag, int aStep)
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
    Check(leg[0] > 0 && leg[1] > 0 && foot[0] > 0 && foot[1] > 0 && thigh[0] > 0 && thigh[1] > 0, std::string(aTag) + ": leg bones found");
    if (leg[0] <= 0 || leg[1] <= 0 || foot[0] <= 0 || foot[1] <= 0 || thigh[0] <= 0 || thigh[1] <= 0)
        return;
    {
        int sh[4] = {};
        const int n = FindShins(sk, f->Masks(), sh);
        Check(n == 2 && ((sh[0] == leg[0] && sh[1] == leg[1]) || (sh[0] == leg[1] && sh[1] == leg[0])),
              "OVLEGS-HINGE-4 " + std::string(aTag) + ": the shins found on the baked masks are LeftLeg and RightLeg (found " + std::to_string(n) + ")");
    }
    Quat rest[2], restThigh[2];
    for (int s = 0; s < 2; ++s)
    {
        const auto& r = db->RigRest(static_cast<uint32_t>(leg[s]));
        rest[s] = math::QuatNormalize(Quat{r.qx, r.qy, r.qz, r.qw});
        const auto& t = db->RigRest(static_cast<uint32_t>(thigh[s]));
        restThigh[s] = math::QuatNormalize(Quat{t.qx, t.qy, t.qz, t.qw});
    }
    auto frameOf = [&](uint32_t aF, Pose& aOut) {
        const motion::PoseRecord* p = db->Pose(aF);
        aOut.n = sk.n;
        for (int i = 0; i < sk.n; ++i)
            aOut.b[i] = Xf{math::QuatNormalize(Quat{p[i].qx, p[i].qy, p[i].qz, p[i].qw}), Vec3{p[i].px, p[i].py, p[i].pz}, Vec3{1, 1, 1}};
    };
    auto offsetOf = [&](const Pose& aB, const Pose& aOp, Knee* aK) { // knee offset of a weapon pose against a base pose
        for (int s = 0; s < 2; ++s)
            aK[s] = KneeOf(math::QuatMul(math::QuatInv(rest[s]), aB.b[leg[s]].r), math::QuatMul(math::QuatInv(rest[s]), aOp.b[leg[s]].r));
    };
    float offsetOver = -1e9f;
    std::string offsetWhere;
    auto knees = [&](const Pose& aP, Knee* aK, Knee* aT = nullptr) {
        for (int s = 0; s < 2; ++s)
        {
            aK[s] = KneeOf(rest[s], aP.b[leg[s]].r);
            if (aT)
                aT[s] = KneeOf(restThigh[s], aP.b[thigh[s]].r);
        }
    };

    std::vector<uint32_t> clips, idleClips;
    uint32_t nFrames = 0;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string_view n = db->ClipName(c);
        if (IsStandStopStart(n))
        {
            clips.push_back(c);
            nFrames += db->Clip(c).numFrames;
        }
        else if (IsStandIdle(n))
            idleClips.push_back(c);
    }
    std::printf("[%s] standing stop / start clips %zu (%u frames, step %d), standing idle clips %zu:", aTag, clips.size(), nFrames, aStep,
                idleClips.size());
    for (uint32_t c : idleClips)
        std::printf(" %.*s", int(db->ClipName(c).size()), db->ClipName(c).data());
    std::printf("\n");
    Check(clips.size() >= 4 && !idleClips.empty(), std::string(aTag) + ": stop / start and idle clips present");

    const int pBi = f->FindPose("Pose_Neutral_Stand_Idle"), pBm = f->FindPose("Pose_Neutral_Stand_Move");
    const int pCi = f->FindPose("Pose_Neutral_Crouch_Idle");
    Check(pBi >= 0 && pBm >= 0 && pCi >= 0, std::string(aTag) + ": base poses found");
    if (pBi < 0 || pBm < 0 || pCi < 0)
        return;
    auto Bi = std::make_unique<Pose>(), Bm = std::make_unique<Pose>(), Ci = std::make_unique<Pose>();
    f->FramePose(f->PoseAt(pBi).firstFrame, *Bi);
    f->FramePose(f->PoseAt(pBm).firstFrame, *Bm);
    f->FramePose(f->PoseAt(pCi).firstFrame, *Ci);
    {
        Knee a[2], b[2], c[2];
        knees(*Bi, a);
        knees(*Bm, b);
        knees(*Ci, c);
        std::printf("[%s] knee of the base poses (roll / side bend, left | right): stand idle %.1f / %.1f | %.1f / %.1f, stand move %.1f / %.1f | %.1f / %.1f, "
                    "crouch idle %.1f / %.1f | %.1f / %.1f\n",
                    aTag, double(a[0].roll), double(a[0].dev), double(a[1].roll), double(a[1].dev), double(b[0].roll), double(b[0].dev),
                    double(b[1].roll), double(b[1].dev), double(c[0].roll), double(c[0].dev), double(c[1].roll), double(c[1].dev));
    }

    const PoseOverlay kSets[] = {PoseOverlay::Rifle, PoseOverlay::Pistol2H, PoseOverlay::Pistol1H};
    const char* kSetNames[] = {"Rifle", "Pistol2H", "Pistol1H"};
    const AimState kAims[] = {AimState::Relaxed, AimState::Ready, AimState::Aiming};
    const char* kAimNames[] = {"Relax", "Ready", "Aim"};
    const float kW[] = {0.33f, 0.67f, 1.0f};
    const Form kForms[3] = {Form::Hinge, Form::Mesh, Form::Local};
    const char* kFormNames[3] = {"hinge", "earlier (mesh)", "local"};

    auto L = std::make_unique<Pose>(), B = std::make_unique<Pose>(), OPi = std::make_unique<Pose>(), OPm = std::make_unique<Pose>(),
         OPc = std::make_unique<Pose>(), OPl = std::make_unique<Pose>(), Bst = std::make_unique<Pose>();
    std::unique_ptr<Pose> out[3] = {std::make_unique<Pose>(), std::make_unique<Pose>(), std::make_unique<Pose>()};
    auto S = std::make_unique<LayerScratch>();
    int earlierOutside = 0;
    Plant plant[3];
    float settledDeg = 0.0f, settledCm = 0.0f, plantHingeVsEarlier = 0.0f, plantHingeVsEarlierZ = 0.0f;
    std::string settledWhere;
    Vec3 pos[3][ov::kMaxBones], posL[ov::kMaxBones];
    Quat rot[3][ov::kMaxBones], rotL[ov::kMaxBones];
    auto compose = [&](LayerInputs aIn, int aForm, Pose& aOut) {
        aIn.legsForm = kForms[aForm];
        ComposeLayers(sk, f->Masks(), aIn, *S, aOut);
    };
    for (int si = 0; si < 3; ++si)
        for (int ai = 0; ai < 3; ++ai)
        {
            const int pi = f->FindOverlayPose(kSets[si], kAims[ai], false, false);
            const int pm = f->FindOverlayPose(kSets[si], kAims[ai], false, true);
            const int pc = f->FindOverlayPose(kSets[si], kAims[ai], true, false);
            if (pi < 0 || pm < 0 || pc < 0)
            {
                Check(false, std::string(aTag) + " " + kSetNames[si] + " " + kAimNames[ai] + ": stand idle / move and crouch poses found");
                continue;
            }
            const std::string set = std::string(kSetNames[si]) + " " + kAimNames[ai];
            f->FramePose(f->PoseAt(pi).firstFrame, *OPi);
            f->FramePose(f->PoseAt(pm).firstFrame, *OPm);
            f->FramePose(f->PoseAt(pc).firstFrame, *OPc);
            Curves ci, cm, cc;
            f->FrameCurves(pi, f->PoseAt(pi).firstFrame, ci);
            f->FrameCurves(pm, f->PoseAt(pm).firstFrame, cm);
            f->FrameCurves(pc, f->PoseAt(pc).firstFrame, cc);

            // settled stance: idle clip frames under the idle weapon pose; the earlier form's knee there is the stance's own offset
            Knee settled[2] = {};
            {
                LayerInputs in;
                in.B = Bi.get();
                in.OP = OPi.get();
                in.OPLower = OPi.get();
                in.curves = ci;
                for (uint32_t c : idleClips)
                    for (uint32_t i = 0; i < db->Clip(c).numFrames; i += 4)
                    {
                        frameOf(db->Clip(c).firstFrame + i, *L);
                        in.L = L.get();
                        compose(in, 0, *out[0]);
                        compose(in, 1, *out[1]);
                        Knee kn[2], ko[2];
                        knees(*out[0], kn);
                        knees(*out[1], ko);
                        MeshPositions(sk, *out[0], pos[0], rot[0]);
                        MeshPositions(sk, *out[1], pos[1], rot[1]);
                        for (int s = 0; s < 2; ++s)
                        {
                            settled[s].roll = std::max(settled[s].roll, ko[s].roll);
                            settled[s].dev = std::max(settled[s].dev, ko[s].dev);
                            const float dDeg = std::max(std::fabs(kn[s].roll - ko[s].roll), std::fabs(kn[s].dev - ko[s].dev));
                            const Vec3 d = pos[0][foot[s]] - pos[1][foot[s]];
                            const float dCm = 100.0f * std::sqrt(math::Dot(d, d));
                            if (dDeg > settledDeg || dCm > settledCm)
                                settledWhere = set + " " + std::string(db->ClipName(c)) + " frame " + std::to_string(i);
                            settledDeg = std::max(settledDeg, dDeg);
                            settledCm = std::max(settledCm, dCm);
                        }
                    }
            }
            Knee kOi[2];
            knees(*OPi, kOi);
            std::printf("  [%s] %-14s legs / pelvis idle %.2f / %.2f, move %.2f / %.2f, crouch %.2f / %.2f | knee of the idle pose %.1f / %.1f | %.1f / %.1f, "
                        "settled stance (earlier form) %.1f / %.1f | %.1f / %.1f\n",
                        aTag, set.c_str(), double(ci[kLegs]), double(ci[kPelvis]), double(cm[kLegs]), double(cm[kPelvis]), double(cc[kLegs]),
                        double(cc[kPelvis]), double(kOi[0].roll), double(kOi[0].dev), double(kOi[1].roll), double(kOi[1].dev), double(settled[0].roll),
                        double(settled[0].dev), double(settled[1].roll), double(settled[1].dev));

            Worst ws[3], wf[3];
            for (uint32_t c : clips)
                for (uint32_t i = 0; i < db->Clip(c).numFrames; i += static_cast<uint32_t>(aStep))
                {
                    const uint32_t fr = db->Clip(c).firstFrame + i;
                    frameOf(fr, *L);
                    Knee ks[2];
                    knees(*L, ks);
                    Knee allow[2], offset[2];
                    const motion::ContactRecord& ct = db->Contact(fr);
                    const float cts[2] = {ct.l, ct.r};
                    MeshPositions(sk, *L, posL, rotL);
                    char where[160];
                    for (float w : kW)
                    {
                        // move -> idle crossfade at weight w (a start runs the same mix the other way)
                        Blend(*Bm, *Bi, w, *B);
                        Blend(*OPm, *OPi, w, *OPl);
                        LayerInputs in;
                        in.L = L.get();
                        in.B = B.get();
                        in.OP = OPl.get();
                        in.OPLower = OPl.get();
                        CurvesMix(cm, ci, w, in.curves);
                        std::snprintf(where, sizeof(where), "%.*s frame %u w %.2f", int(db->ClipName(c).size()), db->ClipName(c).data(), i, double(w));
                        offsetOf(*B, *OPl, offset);
                        for (int s = 0; s < 2; ++s)
                        {
                            allow[s] = Knee{std::max(ks[s].roll, offset[s].roll), std::max(ks[s].dev, offset[s].dev), 0.0f};
                            const float o = std::max(offset[s].roll - aBand.roll[s], offset[s].dev - aBand.dev[s]);
                            if (o > offsetOver)
                            {
                                offsetOver = o;
                                offsetWhere = set + " w " + std::to_string(w) + (s ? " right" : " left");
                            }
                        }
                        for (int fm = 0; fm < 3; ++fm)
                        {
                            compose(in, fm, *out[fm]);
                            Knee kn[2], kt[2];
                            knees(*out[fm], kn, kt);
                            ws[fm].Add(kn, kt, allow, aBand, where);
                            if (in.curves[kLegs] > 0.0f && w < 1.0f)
                            {
                                MeshPositions(sk, *out[fm], pos[fm], rot[fm]);
                                for (int s = 0; s < 2; ++s)
                                    if (cts[s] > 0.5f)
                                    {
                                        const Vec3 d = pos[fm][foot[s]] - posL[foot[s]];
                                        plant[fm].Add(100.0f * std::sqrt(math::Dot(d, d)), AngleDeg(rot[fm][foot[s]], rotL[foot[s]]),
                                                      100.0f * std::max(0.0f, -d.z));
                                    }
                            }
                        }
                        if (in.curves[kLegs] > 0.0f && w < 1.0f)
                            for (int s = 0; s < 2; ++s)
                                if (cts[s] > 0.5f)
                                {
                                    const Vec3 d = pos[0][foot[s]] - pos[1][foot[s]];
                                    plantHingeVsEarlier = std::max(plantHingeVsEarlier, 100.0f * std::sqrt(math::Dot(d, d)));
                                    plantHingeVsEarlierZ = std::max(plantHingeVsEarlierZ, 100.0f * std::fabs(d.z));
                                }
                    }
                    // stance fade at crouch share 0.5: legs per stance side
                    {
                        const float s = 0.5f;
                        Blend(*Bi, *Ci, s, *Bst);
                        Blend(*OPi, *OPc, s, *OPl);
                        LayerInputs in;
                        in.L = L.get();
                        in.B = Bst.get();
                        in.OP = OPl.get();
                        in.OPLower = OPl.get();
                        CurvesMix(ci, cc, s, in.curves);
                        in.torso[0] = {OPi.get(), Bi.get(), ci[kSpine], ci[kSpineAdd], ci[kHead], ci[kHeadAdd], ci[kLegs], ci[kPelvis]};
                        in.torso[1] = {OPc.get(), Ci.get(), cc[kSpine], cc[kSpineAdd], cc[kHead], cc[kHeadAdd], cc[kLegs], cc[kPelvis]};
                        in.torsoCrouch = s;
                        std::snprintf(where, sizeof(where), "%.*s frame %u crouch %.2f", int(db->ClipName(c).size()), db->ClipName(c).data(), i,
                                      double(s));
                        Knee o0[2], o1[2];
                        offsetOf(*Bi, *OPi, o0);
                        offsetOf(*Ci, *OPc, o1);
                        for (int q = 0; q < 2; ++q)
                            allow[q] = Knee{std::max({ks[q].roll, o0[q].roll, o1[q].roll}), std::max({ks[q].dev, o0[q].dev, o1[q].dev}), 0.0f};
                        for (int fm = 0; fm < 3; ++fm)
                        {
                            compose(in, fm, *out[fm]);
                            Knee kn[2], kt[2];
                            knees(*out[fm], kn, kt);
                            wf[fm].Add(kn, kt, allow, aBand, where);
                        }
                    }
                }
            for (int fm = 0; fm < 3; ++fm)
                ws[fm].Print((set + " stop/start " + kFormNames[fm]).c_str());
            for (int fm = 0; fm < 3; ++fm)
                wf[fm].Print((set + " stance fade " + kFormNames[fm]).c_str());
            earlierOutside += ws[1].outside;
            Check(ws[0].outside == 0, "OVLEGS-HINGE-1 " + std::string(aTag) + " " + set + ": the composed knee stays a hinge on stops / starts (frames outside " +
                                          std::to_string(ws[0].outside) + " of " + std::to_string(ws[0].frames) + ", worst excess " + std::to_string(ws[0].over) +
                                          " deg at " + ws[0].where + ")");
            Check(wf[0].outside == 0, "OVLEGS-HINGE-3 " + std::string(aTag) + " " + set + ": the composed knee stays a hinge in the stance fade (frames outside " +
                                          std::to_string(wf[0].outside) + " of " + std::to_string(wf[0].frames) + ", worst excess " + std::to_string(wf[0].over) +
                                          " deg at " + wf[0].where + ")");
        }
    for (int fm = 0; fm < 3; ++fm)
        std::printf("[%s] planted foot (contact > 0.5, 0 < legs layering < 1), %-14s: ankle to L max %.2f cm mean %.2f cm (n %d), below L max %.2f cm, "
                    "foot rotation to L max %.1f deg\n",
                    aTag, kFormNames[fm], double(plant[fm].maxCm), double(plant[fm].n ? plant[fm].sumCm / float(plant[fm].n) : 0.0f), plant[fm].n,
                    double(plant[fm].maxSinkCm), double(plant[fm].maxDeg));
    std::printf("[%s] planted foot (contact > 0.5, 0 < legs layering < 1): largest ankle change hinge vs earlier form %.2f cm (height %.2f cm)\n",
                aTag, double(plantHingeVsEarlier), double(plantHingeVsEarlierZ));
    std::printf("[%s] knee offset of the weapon poses against the base pose: largest excess over the band %.2f deg (%s)\n", aTag,
                double(offsetOver), offsetWhere.c_str());
    Check(earlierOutside > 0, "OVLEGS-HINGE-0 " + std::string(aTag) + ": the earlier form leaves the band on these frames (" + std::to_string(earlierOutside) + ")");
    Check(settledDeg <= 1.0f && settledCm <= 1.0f, "OVLEGS-HINGE-2 " + std::string(aTag) + ": the settled stance keeps its knees (" + std::to_string(settledDeg) +
                                                       " deg) and ankles (" + std::to_string(settledCm) + " cm), largest at " + settledWhere);
}

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("FAIL usage: utpp_mm_overlay_legs_hinge_test woman.utppmm woman.utppov man.utppmm man.utppov [--all]\n");
        return 1;
    }
    const int step = argc > 5 && std::strcmp(argv[5], "--all") == 0 ? 1 : 2;
    TestBody(argv[3], argv[4], kRigIdentityManOv, kBandMan, "man", step);
    TestBody(argv[1], argv[2], kRigIdentityWomanOv, kBandWoman, "woman", step);
    std::printf("overlay legs hinge test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

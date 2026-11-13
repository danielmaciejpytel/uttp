// Male V upper-body correction (src/mm/MaleBodyCorrection.hpp): world-preserving rotations of Spine3 / clavicles / upper arms by weapon-class family.
// Checks on the real male skeleton + authored poses (male overlay file):
//  (1) key -> family map (weapon class x stance), invalid keys / weights ignored;
//  (2) strength 0, zero weights, non-finite strength / weights, invalid rig: pose bit for bit unchanged;
//  (3) every family at strength 1 on every authored pose of its family: the model POSITION of every bone and the model ROTATION of every bone
//      except Spine3 / clavicles / upper arms are unchanged (< 1e-4): hands, weapons, fingers, head, feet and the shoulder sockets stay;
//      the corrected bones really rotate (tables are not all zero);
//  (4) continuity: sweeping the family weight 0 -> 1 in 200 steps moves every model rotation by a bounded amount per step; total weight above 1 is clamped;
//  (5) a blend of two families = the weighted blend (weights 0.5 / 0.5 give the mean rotation vector).
// Argument: male .utppov. Mutations (UTPP_MALECORR_MUTATION) must print "FAIL ": 1 = children not compensated, 2 = crouch bit ignored, 3 = weights not clamped.
#include "../src/mm/MaleBodyCorrection.hpp"
#include "../src/mm/OverlayFile.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <cstring>
#include <memory>
#include <string>

using namespace utpp;
using namespace utpp::mm;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const char* aLabel)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aLabel);
}
float QuatErr(const math::Quat& a, const math::Quat& b)
{
    const float s = math::QuatDot(a, b) < 0.0f ? -1.0f : 1.0f;
    return std::max(std::max(std::fabs(a.x - s * b.x), std::fabs(a.y - s * b.y)), std::max(std::fabs(a.z - s * b.z), std::fabs(a.w - s * b.w)));
}
bool SamePose(const ov::Pose& a, const ov::Pose& b, int n)
{
    return std::memcmp(a.b, b.b, sizeof(ov::Xf) * static_cast<size_t>(n)) == 0;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::puts("usage: utpp_male_body_correction_test <man.utppov>");
        return 2;
    }
    std::string error;
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!ovf->LoadFromFile(argv[1], ov::kRigIdentityManOv, error))
    {
        std::printf("load error: %s\n", error.c_str());
        return 2;
    }
    const ov::Skeleton& sk = ovf->Skel();
    int spine3 = -1, clav[2] = {-1, -1}, arm[2] = {-1, -1}, hand[2] = {-1, -1}, head = -1, foot[2] = {-1, -1};
    for (int i = 0; i < sk.n; ++i)
    {
        const auto n = ovf->BoneName(i);
        if (n == "Spine3") spine3 = i;
        if (n == "LeftShoulder") clav[0] = i;
        if (n == "RightShoulder") clav[1] = i;
        if (n == "LeftArm") arm[0] = i;
        if (n == "RightArm") arm[1] = i;
        if (n == "LeftHand") hand[0] = i;
        if (n == "RightHand") hand[1] = i;
        if (n == "Head") head = i;
        if (n == "LeftFoot") foot[0] = i;
        if (n == "RightFoot") foot[1] = i;
    }
    ov::MaleBodyCorrectionRig rig;
    rig.Build(sk, spine3, clav[0], arm[0], clav[1], arm[1]);
    Check(rig.valid && rig.childCount[0] >= 3 && rig.childCount[1] >= 1 && rig.childCount[3] >= 1, "rig resolves Spine3 / clavicles / upper arms and their children");
    {
        // frame check against the offline fit: local quaternions (x y z w) of the corrected bones in Pose_Pistol2H_Stand_Aim_Idle frame 0
        const int key = ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H, ov::AimState::Aiming, false, false);
        auto fp = std::make_unique<ov::Pose>();
        if (key >= 0)
        {
            ovf->FramePose(ovf->PoseAt(key).firstFrame, *fp);
            const int bones[5] = {spine3, clav[0], clav[1], arm[0], arm[1]};
            const char* names[5] = {"Spine3", "LeftShoulder", "RightShoulder", "LeftArm", "RightArm"};
            for (int i = 0; i < 5; ++i)
                std::printf("LOCAL %s %.5f %.5f %.5f %.5f\n", names[i], fp->b[bones[i]].r.x, fp->b[bones[i]].r.y, fp->b[bones[i]].r.z, fp->b[bones[i]].r.w);
            // corrected locals with the StandP2H table at strength 1 (golden values come from male_body_correction_eval.py, see GOLDEN below)
            ov::MaleCorrFamilyWeights gw;
            gw.w[static_cast<int>(ov::MaleCorrFamily::StandP2H)] = 1.0f;
            ov::MaleBodyCorrectionRig grig;
            grig.Build(sk, spine3, clav[0], arm[0], clav[1], arm[1]);
            ov::ApplyMaleBodyCorrection(sk, grig, gw, 1.0f, *fp);
            for (int i = 0; i < 5; ++i)
                std::printf("CORRECTED %s %.5f %.5f %.5f %.5f\n", names[i], fp->b[bones[i]].r.x, fp->b[bones[i]].r.y, fp->b[bones[i]].r.z, fp->b[bones[i]].r.w);
            // GOLDEN: python male_body_correction_eval.apply_family(corr_c stand_P2H) on the utppclip frame 0 (same local frames: the LOCAL lines equal the clip)
            const float gl[5][4] = {{0.00251f, -0.00486f, -0.03717f, 0.99929f}, {-0.02557f, 0.64229f, -0.16351f, 0.74838f}, {-0.64111f, -0.02156f, 0.74994f, 0.16153f},
                                    {-0.47238f, 0.17538f, -0.59431f, 0.62681f}, {-0.44369f, 0.12075f, -0.67474f, 0.57731f}};
            const float gc[5][4] = {{0.08654f, -0.09469f, -0.12341f, 0.98403f}, {-0.15219f, 0.74081f, -0.03719f, 0.65319f}, {-0.50340f, 0.06539f, 0.85898f, 0.06686f},
                                    {-0.41994f, 0.28991f, -0.63004f, 0.58536f}, {-0.49032f, 0.07380f, -0.65920f, 0.56533f}};
            float errLocal = 0.0f, errCorr = 0.0f;
            {
                auto q0 = std::make_unique<ov::Pose>();
                ovf->FramePose(ovf->PoseAt(key).firstFrame, *q0);
                for (int i = 0; i < 5; ++i)
                {
                    errLocal = std::max(errLocal, QuatErr(q0->b[bones[i]].r, math::Quat{gl[i][0], gl[i][1], gl[i][2], gl[i][3]}));
                    errCorr = std::max(errCorr, QuatErr(fp->b[bones[i]].r, math::Quat{gc[i][0], gc[i][1], gc[i][2], gc[i][3]}));
                }
            }
            std::printf("frame check vs offline fit: local err %.2e, corrected err %.2e\n", errLocal, errCorr);
            Check(errLocal < 2e-5f, "utppov local frames equal the utppclip frames the table was fitted on (Pistol2H stand aim, 5 bones)");
            Check(errCorr < 2e-5f, "corrected locals equal the offline (python) correction of the same pose");
        }
    }
    ov::MaleBodyCorrectionRig bad;
    bad.Build(sk, -1, clav[0], arm[0], clav[1], arm[1]);
    Check(!bad.valid, "rig with a missing bone is invalid");

    // (1) key -> family
    using F = ov::MaleCorrFamily;
    Check(ov::MaleCorrFamilyOfKey(0) == F::Loco && ov::MaleCorrFamilyOfKey(11) == F::Loco, "Default overlay keys -> Loco");
    Check(ov::MaleCorrFamilyOfKey(12 + 8) == F::StandRifle && ov::MaleCorrFamilyOfKey(12 + 8 + 2) == F::CrouchRifle, "Rifle stand / crouch");
    Check(ov::MaleCorrFamilyOfKey(24 + 8) == F::StandP1H && ov::MaleCorrFamilyOfKey(24 + 4 + 2 + 1) == F::CrouchP1H, "Pistol1H stand / crouch");
    Check(ov::MaleCorrFamilyOfKey(36) == F::StandP2H && ov::MaleCorrFamilyOfKey(36 + 2) == F::CrouchP2H, "Pistol2H stand / crouch (key 36 = relaxed stand idle)");
    Check(ov::MaleCorrFamilyOfKey(-1) == F::None && ov::MaleCorrFamilyOfKey(4 * 12) == F::None && ov::MaleCorrFamilyOfKey(9 * 12 + 5) == F::None,
          "invalid keys and unmeasured overlays (Injured, Game, ...) -> no family");
    {
        ov::MaleCorrFamilyWeights none;
        ov::AddPoseKeyWeight(none, 4 * 12, 1.0f);
        ov::AddPoseKeyWeight(none, -1, 1.0f);
        Check(none.Sum() == 0.0f, "weights of unmeasured overlays are dropped (no correction)");
    }
    {
        ov::MaleCorrFamilyWeights w;
        ov::AddPoseKeyWeight(w, 36, 0.25f);
        ov::AddPoseKeyWeight(w, 36 + 4, 0.25f);
        ov::AddPoseKeyWeight(w, 36, -1.0f);
        ov::AddPoseKeyWeight(w, 36, std::numeric_limits<float>::quiet_NaN());
        Check(std::fabs(w.w[static_cast<int>(F::StandP2H)] - 0.5f) < 1e-6f && std::fabs(w.Sum() - 0.5f) < 1e-6f, "pose key weights accumulate, negative / NaN ignored");
    }

    {
        // aim gate: Pistol2H crouch ready (key 36 + 4 + 2) is gated off, relaxed / aiming crouch P2H and every other family are not
        ov::MaleCorrFamilyWeights w;
        ov::AddPoseKeyWeight(w, 36 + 4 + 2, 1.0f);
        Check(w.Sum() == 0.0f, "aim gate: Pistol2H crouch ready contributes nothing");
        ov::AddPoseKeyWeight(w, 36 + 8 + 2, 1.0f);
        ov::AddPoseKeyWeight(w, 12 + 4, 1.0f);
        Check(std::fabs(w.w[static_cast<int>(F::CrouchP2H)] - 1.0f) < 1e-6f && std::fabs(w.w[static_cast<int>(F::StandRifle)] - 1.0f) < 1e-6f,
              "aim gate: Pistol2H crouch aim and Rifle stand ready pass in full");
    }

    // authored poses per family
    struct Fam
    {
        F f;
        ov::PoseOverlay ov;
        bool crouch;
    };
    const Fam fams[] = {{F::StandRifle, ov::PoseOverlay::Rifle, false},   {F::StandP1H, ov::PoseOverlay::Pistol1H, false},
                        {F::StandP2H, ov::PoseOverlay::Pistol2H, false},  {F::CrouchRifle, ov::PoseOverlay::Rifle, true},
                        {F::CrouchP1H, ov::PoseOverlay::Pistol1H, true},  {F::CrouchP2H, ov::PoseOverlay::Pistol2H, true},
                        {F::Loco, ov::PoseOverlay::Pistol2H, false}};
    auto base = std::make_unique<ov::Pose>();
    auto p = std::make_unique<ov::Pose>();
    math::Vec3 pos0[ov::kMaxBones], pos1[ov::kMaxBones];
    math::Quat rot0[ov::kMaxBones], rot1[ov::kMaxBones];
    const bool corrected[ov::kMaxBones] = {};
    (void)corrected;
    auto isCorrected = [&](int b) { return b == spine3 || b == clav[0] || b == clav[1] || b == arm[0] || b == arm[1]; };

    // (2) off switches
    {
        const int key = ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H, ov::AimState::Aiming, false, false);
        if (key < 0)
            return 2;
        ovf->FramePose(ovf->PoseAt(key).firstFrame, *base);
        ov::MaleCorrFamilyWeights w;
        w.w[static_cast<int>(F::StandP2H)] = 1.0f;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        bool all = true;
        const float strengths[] = {0.0f, -1.0f, nan};
        for (float s : strengths)
        {
            *p = *base;
            const auto r = ov::ApplyMaleBodyCorrection(sk, rig, w, s, *p);
            all = all && !r.applied && SamePose(*p, *base, sk.n);
        }
        Check(all, "strength 0 / negative / NaN: pose bit for bit unchanged");
        *p = *base;
        ov::MaleCorrFamilyWeights zero;
        Check(!ov::ApplyMaleBodyCorrection(sk, rig, zero, 1.0f, *p).applied && SamePose(*p, *base, sk.n), "zero weights: pose bit for bit unchanged");
        ov::MaleCorrFamilyWeights wn;
        wn.w[0] = nan;
        *p = *base;
        Check(!ov::ApplyMaleBodyCorrection(sk, rig, wn, 1.0f, *p).applied && SamePose(*p, *base, sk.n), "NaN weights: pose bit for bit unchanged");
        *p = *base;
        Check(!ov::ApplyMaleBodyCorrection(sk, bad, w, 1.0f, *p).applied && SamePose(*p, *base, sk.n), "invalid rig: pose bit for bit unchanged");
    }

    // (3) world preservation on every authored pose of every family
    float maxPos = 0, maxRotOther = 0, maxHand = 0, maxHeadFoot = 0, maxCorrRot = 0;
    int posesChecked = 0;
    for (const Fam& fam : fams)
        for (int aim = 0; aim < 3; ++aim)
        {
            const int key = ovf->FindOverlayPose(fam.ov, static_cast<ov::AimState>(aim), fam.crouch, false);
            if (key < 0)
                continue;
            ovf->FramePose(ovf->PoseAt(key).firstFrame, *base);
            *p = *base;
            ov::MaleCorrFamilyWeights w;
            w.w[static_cast<int>(fam.f)] = 1.0f;
            ov::MeshPositions(sk, *p, pos0, rot0);
            const auto r = ov::ApplyMaleBodyCorrection(sk, rig, w, 1.0f, *p);
            ov::MeshPositions(sk, *p, pos1, rot1);
            ++posesChecked;
            (void)r;
            for (int b = 0; b < sk.n; ++b)
            {
                maxPos = std::max(maxPos, std::sqrt(math::Dot(pos1[b] - pos0[b], pos1[b] - pos0[b])));
                const float re = QuatErr(rot1[b], rot0[b]);
                if (isCorrected(b))
                    maxCorrRot = std::max(maxCorrRot, re);
                else
                    maxRotOther = std::max(maxRotOther, re);
            }
            for (int s = 0; s < 2; ++s)
            {
                maxHand = std::max(maxHand, std::max(std::sqrt(math::Dot(pos1[hand[s]] - pos0[hand[s]], pos1[hand[s]] - pos0[hand[s]])), QuatErr(rot1[hand[s]], rot0[hand[s]])));
                maxHeadFoot = std::max(maxHeadFoot, std::max(QuatErr(rot1[foot[s]], rot0[foot[s]]), QuatErr(rot1[head], rot0[head])));
            }
        }
    std::printf("world preservation over %d poses: max model pos err %.2e m, other-bone rot err %.2e, hand err %.2e, head/foot rot err %.2e, corrected-bone rot change %.3f\n",
                posesChecked, maxPos, maxRotOther, maxHand, maxHeadFoot, maxCorrRot);
    Check(posesChecked >= 12, "authored poses of all weapon classes / stances found");
    Check(maxPos < 1e-4f, "model position of every bone unchanged (sockets / hands / weapons / head / feet stay)");
    Check(maxRotOther < 1e-4f && maxHand < 1e-4f && maxHeadFoot < 1e-4f, "model rotation of every bone except Spine3 / clavicles / upper arms unchanged");
    Check(maxCorrRot > 0.02f, "the corrected bones really rotate (table is not empty)");

    // (4) continuity and clamping
    {
        const int key = ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H, ov::AimState::Aiming, false, false);
        ovf->FramePose(ovf->PoseAt(key).firstFrame, *base);
        float maxStep = 0;
        math::Quat prev[ov::kMaxBones];
        math::Vec3 pp[ov::kMaxBones];
        {
            *p = *base;
            ov::MeshPositions(sk, *p, pp, prev);
        }
        const math::Quat first[2] = {prev[spine3], prev[arm[0]]};
        (void)first;
        for (int i = 1; i <= 200; ++i)
        {
            *p = *base;
            ov::MaleCorrFamilyWeights w;
            w.w[static_cast<int>(F::StandP2H)] = static_cast<float>(i) / 200.0f;
            ov::ApplyMaleBodyCorrection(sk, rig, w, 1.0f, *p);
            math::Quat cur[ov::kMaxBones];
            ov::MeshPositions(sk, *p, pp, cur);
            for (int b = 0; b < sk.n; ++b)
                maxStep = std::max(maxStep, math::QuatAngleBetween(cur[b], prev[b]));
            std::memcpy(prev, cur, sizeof(math::Quat) * static_cast<size_t>(sk.n));
        }
        std::printf("continuity: max per-step model rotation change %.4f rad over 200 steps\n", maxStep);
        Check(maxStep < 0.01f, "weight sweep 0 -> 1: per-step change small (no pop)");
        // clamp: weights 2.0 on one family must equal weight 1.0
        auto a = std::make_unique<ov::Pose>(*base), b2 = std::make_unique<ov::Pose>(*base);
        ov::MaleCorrFamilyWeights w1, w2;
        w1.w[static_cast<int>(F::StandP2H)] = 1.0f;
        w2.w[static_cast<int>(F::StandP2H)] = 2.0f;
        ov::ApplyMaleBodyCorrection(sk, rig, w1, 1.0f, *a);
        ov::ApplyMaleBodyCorrection(sk, rig, w2, 1.0f, *b2);
        Check(SamePose(*a, *b2, sk.n), "total family weight above 1 is clamped to 1");
        // partial total (0.5) scales the correction down
        ov::MaleCorrFamilyWeights wh;
        wh.w[static_cast<int>(F::StandP2H)] = 0.5f;
        auto h = std::make_unique<ov::Pose>(*base);
        ov::ApplyMaleBodyCorrection(sk, rig, wh, 1.0f, *h);
        ov::ApplyMaleBodyCorrection(sk, rig, w1, 0.5f, *b2 = *base);
        float diff = 0;
        for (int i = 0; i < sk.n; ++i)
            diff = std::max(diff, QuatErr(h->b[i].r, b2->b[i].r));
        Check(diff < 1e-5f, "weight 0.5 equals strength 0.5 (linear scaling)");
    }

    // (5) blend of two families = rotation-vector mean
    {
        const int key = ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H, ov::AimState::Aiming, false, false);
        ovf->FramePose(ovf->PoseAt(key).firstFrame, *base);
        ov::MaleCorrFamilyWeights wa;
        wa.w[static_cast<int>(F::StandP2H)] = 0.5f;
        wa.w[static_cast<int>(F::StandRifle)] = 0.5f;
        float sp[3];
        for (int i = 0; i < 3; ++i)
            sp[i] = 0.5f * ov::kMaleCorrectionTable[static_cast<int>(F::StandP2H)].spine3[i] + 0.5f * ov::kMaleCorrectionTable[static_cast<int>(F::StandRifle)].spine3[i];
        *p = *base;
        const auto r = ov::ApplyMaleBodyCorrection(sk, rig, wa, 1.0f, *p);
        const float mag = std::sqrt(sp[0] * sp[0] + sp[1] * sp[1] + sp[2] * sp[2]);
        std::printf("blend: Spine3 %.3f deg applied, mean of the tables %.3f deg\n", r.spine3Deg, mag);
        Check(std::fabs(r.spine3Deg - mag) < 1e-3f, "two-family blend applies the weighted mean rotation vector");
    }
    std::printf("MaleBodyCorrection %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

// Offline ablation of the Pistol1H Relaxed standing idle stance (female policy vs male policy), real PoseEvaluator on real assets.
// No game, no install, no runtime claim. Usage:
//   measure_pistol_idle_stance <w|m> <db.utppmm> <ov_raw.utppov> <ov_patched.utppov> <style_weights.txt>
// Per overlay file x policy flag (bodyWoman for w, maleIdlePistolStance for m) the settled idle pose is composed and compared with
// (a) the unarmed Default overlay (lower body), (b) the authored raw Pose_Pistol1H_Stand_Relax_Idle (right arm chain).
#include "../../plugin/UTPP/src/mm/MotionPose.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
using namespace utpp::mm;

namespace
{
struct Frame { ov::Pose pose; };
int Find(const ov::OverlayFile& f, const char* name)
{
    for (int i = 0; i < f.Skel().n; ++i) if (f.BoneName(i) == name) return i;
    return -1;
}
math::Vec3 Model(const ov::Skeleton& sk, const ov::Pose& p, int b)
{
    math::Quat q; math::Vec3 pos; gik::ModelOf(sk, p, b, q, pos); return pos;
}
}

int main(int argc, char** argv)
{
    if (argc != 6) { std::puts("usage: <w|m> db ov_raw ov_patched style"); return 2; }
    const bool man = argv[1][0] == 'm';
    std::string error;
    auto db = std::make_unique<v2::MotionDatabase>();
    if (!db->LoadFromFile(argv[2], man ? v2::kRigIdentityMan : v2::kRigIdentityWoman, error)) { std::puts(error.c_str()); return 2; }
    std::ifstream stream(argv[5]); std::stringstream text; text << stream.rdbuf();
    ov::StyleFamilyData style;
    if (!style.Parse(text.str(), &error)) { std::puts(error.c_str()); return 2; }
    int idleClip = -1;
    for (uint32_t c = 0; c < db->NumClips(); ++c) if (db->ClipName(c) == "M_Neutral_Stand_Idle_Loop") idleClip = int(c);
    if (idleClip < 0) { std::puts("no idle clip"); return 2; }

    struct Result { const char* label; ov::Pose pose; uint64_t hash; float curvesLegs, curvesPelvis, styleArmR; };
    std::vector<std::pair<std::string, Result>> rows;
    ov::Pose unarmed, authoredRaw;
    const char* files[2] = {argv[3], argv[4]};
    const char* fileLabel[2] = {"raw", "patched"};
    int lf = -1, rf = -1, hips = -1, ra = -1, rfa = -1, rh = -1, lu = -1, ru = -1, ll = -1, rl = -1;
    std::unique_ptr<ov::OverlayFile> ovs[2];
    for (int fi = 0; fi < 2; ++fi)
    {
        ovs[fi] = std::make_unique<ov::OverlayFile>();
        if (!ovs[fi]->LoadFromFile(files[fi], man ? ov::kRigIdentityManOv : ov::kRigIdentityWomanOv, error)) { std::printf("%s: %s\n", files[fi], error.c_str()); return 2; }
    }
    const ov::OverlayFile& ref = *ovs[0];
    lf = Find(ref, "LeftFoot"); rf = Find(ref, "RightFoot"); hips = Find(ref, "Hips"); ra = Find(ref, "RightArm");
    rfa = Find(ref, "RightForeArm"); rh = Find(ref, "RightHand"); lu = Find(ref, "LeftUpLeg"); ru = Find(ref, "RightUpLeg");
    ll = Find(ref, "LeftLeg"); rl = Find(ref, "RightLeg");
    if (lf < 0 || rf < 0 || hips < 0 || ra < 0) { std::puts("bones missing"); return 2; }
    const int poseIdx = ref.FindPose("Pose_Pistol1H_Stand_Relax_Idle");
    if (poseIdx < 0) { std::puts("pose missing"); return 2; }
    ref.FramePose(ref.PoseAt(poseIdx).firstFrame, authoredRaw);

    constexpr float dt = 1.f / 60;
    std::vector<BoneTransform> output(db->NumBones());
    v2::MatchResult match; match.clip = idleClip; match.clipTime = .5f;
    auto run = [&](int fi, ov::PoseOverlay overlay, bool policy, ov::Pose& out, float& styleArmR, float& legs, float& pelvis, uint64_t* hash) {
        auto ev = std::make_unique<motionpose::PoseEvaluator>();
        if (!ev->Init(db.get(), ovs[fi].get(), &style, 2, error, motionpose::BaseBlendPolicy::LiveCrossfade)) { std::puts(error.c_str()); std::exit(2); }
        motionpose::OverlayFrameInputs in;
        in.machine.style = man ? ov::Style::Masculine : ov::Style::Feminine;
        in.machine.overlay = overlay;
        in.machine.wantsReady = false;
        in.capturePoseDiagnostics = true; in.gripSkip = true;
        if (man) in.maleIdlePistolStance = policy; else in.bodyWoman = policy;
        motionpose::FrameInfo info;
        uint64_t h = 1469598103934665603ull;
        for (int k = 0; k < 300; ++k)
        {
            if (!ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) || !info.finite) { std::puts("evaluate failed"); std::exit(2); }
            if (k >= 270)
            {
                const auto* bytes = reinterpret_cast<const uint8_t*>(output.data());
                for (size_t b = 0; b < output.size() * sizeof(BoneTransform); ++b) h = (h ^ bytes[b]) * 1099511628211ull;
            }
        }
        if (hash) *hash = h;
        ev->ToOv(output.data(), out);
        legs = info.curves.v[ov::kLegs]; pelvis = info.curves.v[ov::kPelvis];
        styleArmR = policy ? ov::IdlePistolRightArmStyleWeight(ev->Machine(), true, in.machine.style) : 1.0f; // weight the evaluator applies
    };
    float sa, lg, pv; uint64_t h;
    run(0, ov::PoseOverlay::Default, false, unarmed, sa, lg, pv, &h);
    const auto& sk = ref.Skel();
    auto feet = [&](const ov::Pose& p, float& span, float& lat, float& fore, float& hipZ) {
        const auto l = Model(sk, p, lf), r = Model(sk, p, rf), hp = Model(sk, p, hips);
        span = std::sqrt((l.x - r.x) * (l.x - r.x) + (l.y - r.y) * (l.y - r.y)) * 100;
        lat = std::fabs(l.x - r.x) * 100; fore = std::fabs(l.y - r.y) * 100; hipZ = hp.z * 100;
    };
    float uspan, ulat, ufore, uhip;
    feet(unarmed, uspan, ulat, ufore, uhip);
    std::printf("BODY=%s db=%s\n", man ? "man" : "woman", argv[2]);
    std::printf("%-34s legs pelvis | feetXY  latX   foreY  hipZ | dFootPos(cm) maxLowerRot(deg) | RArm  RFore RHand (deg vs authored) styleArmR  hash\n", "config");
    auto printRow = [&](const char* label, const ov::Pose& p, float legs, float pelvis, float armW, uint64_t hash) {
        float span, lat, fore, hipZ; feet(p, span, lat, fore, hipZ);
        const auto lp = Model(sk, p, lf) - Model(sk, unarmed, lf), rp = Model(sk, p, rf) - Model(sk, unarmed, rf);
        const float dFoot = std::max(gik::Len(lp), gik::Len(rp)) * 100;
        float maxLower = 0;
        for (int b : {hips, lu, ru, ll, rl, lf, rf}) if (b >= 0) maxLower = std::max(maxLower, ov::AngleDeg(p.b[b].r, unarmed.b[b].r));
        auto ang = [&](int b) { return b >= 0 ? ov::AngleDeg(p.b[b].r, authoredRaw.b[b].r) : -1.f; };
        std::printf("%-34s %.2f  %.2f  | %6.2f %6.2f %6.2f %6.2f | %6.2f  %8.2f | %6.2f %6.2f %6.2f  %.3f  %016llx\n", label, legs, pelvis, span, lat, fore, hipZ,
                    dFoot, maxLower, ang(ra), ang(rfa), ang(rh), armW, static_cast<unsigned long long>(hash));
    };
    printRow("unarmed Default (reference)", unarmed, lg, pv, 1.f, h);
    for (int fi = 0; fi < 2; ++fi)
        for (int policy = 0; policy < 2; ++policy)
        {
            ov::Pose p; float armW, legs, pelvis; uint64_t hash;
            run(fi, ov::PoseOverlay::Pistol1H, policy != 0, p, armW, legs, pelvis, &hash);
            char label[96]; std::snprintf(label, sizeof label, "P1H relaxed %s data, policy %s", fileLabel[fi], policy ? "ON " : "off");
            printRow(label, p, legs, pelvis, armW, hash);
        }
    // authored raw pose (what source pack Relax shows) for the lower body / arm reference
    {
        float span, lat, fore, hipZ; feet(authoredRaw, span, lat, fore, hipZ);
        std::printf("authored raw Pose_Pistol1H_Stand_Relax_Idle (frame 0, local, no loco): feetXY %.2f latX %.2f foreY %.2f hipZ %.2f\n", span, lat, fore, hipZ);
    }
    return 0;
}

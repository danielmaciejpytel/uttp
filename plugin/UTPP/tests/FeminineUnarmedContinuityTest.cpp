// Actual evaluator regression for the observed Walk_Stop_F_Lfoot socket pop.
// Uses local assets only; no game, native-pose substitution, or modified IK limits.
#include "../src/mm/MotionPose.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
using namespace utpp::mm;

int main(int argc, char** argv)
{
    if (argc != 4) return 2;
    std::string error;
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!db->LoadFromFile(argv[1], motion::kRigIdentityWoman, error) ||
        !ovf->LoadFromFile(argv[2], ov::kRigIdentityWomanOv, error)) { std::puts(error.c_str()); return 2; }
    std::ifstream stream(argv[3]); std::stringstream text; text << stream.rdbuf();
    ov::StyleFamilyData style;
    if (!style.Parse(text.str(), &error)) return 2;
    int stop = -1;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
        if (db->ClipName(c) == "M_Neutral_Walk_Stop_F_Lfoot") stop = int(c);
    if (stop < 0) return 2;
    int pass = 0, fail = 0;
    auto check = [&](bool ok, const char* label) { (ok ? pass : fail)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", label); };
    auto makeEvaluator = [&] {
        auto ev = std::make_unique<motionpose::PoseEvaluator>();
        if (!ev->Init(db.get(), ovf.get(), &style, 2, error, motionpose::BaseBlendPolicy::LiveCrossfade)) ev.reset();
        return ev;
    };
    auto input = [] {
        motionpose::OverlayFrameInputs in;
        in.bodyWoman = true; in.machine.style = ov::Style::Feminine;
        in.machine.overlay = ov::PoseOverlay::Default; in.machine.wantsReady = true;
        in.capturePoseDiagnostics = true; in.gripSkip = true;
        return in;
    };
    constexpr float dt = 1.f / 60;
    motion::MatchResult match; match.clip = stop; match.clipTime = 1.9f;
    std::vector<BoneTransform> output(db->NumBones());
    motionpose::FrameInfo info;
    auto ev = makeEvaluator(); if (!ev) return 2;
    auto in = input();
    for (int k = 0; k < 180; ++k) ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info);
    auto before = std::make_unique<ov::Pose>(), after = std::make_unique<ov::Pose>();
    float previousWidth = 0, maxWidthStep = 0, minSocket = 1, maxContactError = 0;
    bool finite = true, unchangedOtherBones = true;
    const auto& rig = ev->GripRig();
    const int armBones[] = {rig.left.upper, rig.right.upper, rig.left.lower, rig.right.lower, rig.left.hand, rig.right.hand};
    for (int k = 0; k <= 36; ++k)
    {
        match.clipTime = 1.9f + k * dt;
        finite = ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) && info.finite && finite;
        ev->ToOv(ev->BasePose().data(), *before); ev->ToOv(output.data(), *after);
        math::Quat q; math::Vec3 left, right;
        gik::ModelOf(ev->Skel(), *after, rig.left.upper, q, left);
        gik::ModelOf(ev->Skel(), *after, rig.right.upper, q, right);
        const float width = gik::Len(right - left);
        if (k) maxWidthStep = std::max(maxWidthStep, std::fabs(width - previousWidth));
        previousWidth = width;
        minSocket = std::min(minSocket, std::min(info.feminineDelta.posM[4], info.feminineDelta.posM[5]));
        for (int bone : {rig.left.hand, rig.right.hand, rig.weaponLeft, rig.weaponRight}) if (bone >= 0)
        {
            math::Vec3 p0, p1; math::Quat q0, q1;
            gik::ModelOf(ev->Skel(), *before, bone, q0, p0); gik::ModelOf(ev->Skel(), *after, bone, q1, p1);
            maxContactError = std::max(maxContactError, gik::Len(p1 - p0));
            finite = finite && std::fabs(math::QuatDot(q0, q1)) > 1 - 1e-6f;
        }
        for (int b = 0; b < ev->Skel().n; ++b)
        {
            bool arm = false; for (int a : armBones) arm = arm || a == b;
            if (!arm) unchangedOtherBones = unchangedOtherBones && std::memcmp(&before->b[b], &after->b[b], sizeof(ov::Xf)) == 0;
        }
    }
    std::printf("unarmed stop692 maxWidthStepCm=%.9f minSocketMm=%.9f contactErrorMm=%.9f\n", maxWidthStep * 100, minSocket * 1000, maxContactError * 1000);
    check(finite, "observed stop phases are finite and preserve contact rotations");
    check(maxWidthStep < .005f, "observed stop phases have no four-centimetre socket toggle");
    check(minSocket > .015f, "reachable partial posture survives the near-extension stop phase");
    check(maxContactError < .0001f, "both hands and weapon attachments preserve same-frame contacts");
    check(unchangedOtherBones, "root spine feet and all non-arm local transforms are bitwise unchanged");

    // Actual pose weights, including their compacted endpoint, govern the policy.
    // A dt0 target change must never change the currently composed geometry.
    for (auto weapon : {ov::PoseOverlay::Pistol1H, ov::PoseOverlay::Pistol2H, ov::PoseOverlay::Rifle})
    {
        ev = makeEvaluator(); if (!ev) return 2;
        in = input(); match.clipTime = 2.18f;
        for (int k = 0; k < 180; ++k) ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info);
        bool zeroStable = true, boundaryFinite = true;
        float maxZeroPos = 0, maxZeroRot = 0, maxSocketStep = 0;
        float previous[2] = {info.feminineDelta.posM[4], info.feminineDelta.posM[5]};
        for (auto target : {weapon, ov::PoseOverlay::Default})
        {
            in.machine.overlay = target;
            for (int k = 0; k < 120; ++k)
            {
                boundaryFinite = ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) && info.finite && boundaryFinite;
                for (int side = 0; side < 2; ++side)
                {
                    maxSocketStep = std::max(maxSocketStep, std::fabs(info.feminineDelta.posM[4 + side] - previous[side]));
                    previous[side] = info.feminineDelta.posM[4 + side];
                }
                if (k == 0 || k == 6 || k == 15 || k == 30 || k == 119)
                {
                    const auto saved = output;
                    in.machine.overlay = target == weapon ? ov::PoseOverlay::Default : weapon;
                    zeroStable = ev->Evaluate(0, match, in, output.data(), uint32_t(output.size()), &info) && zeroStable;
                    for (size_t b = 0; b < output.size(); ++b)
                    {
                        maxZeroPos = std::max(maxZeroPos, gik::Len(output[b].p - saved[b].p));
                        maxZeroRot = std::max(maxZeroRot, ov::AngleDeg(output[b].q, saved[b].q));
                    }
                    in.machine.overlay = target;
                    zeroStable = ev->Evaluate(0, match, in, output.data(), uint32_t(output.size()), &info) && zeroStable;
                }
            }
        }
        std::printf("boundary Default<->%d maxSocketStepMm=%.9f dt0PosMm=%.9f dt0RotDeg=%.9f\n", int(weapon), maxSocketStep * 1000, maxZeroPos * 1000, maxZeroRot);
        check(boundaryFinite, "Default weapon transition and endpoint stay finite");
        check(zeroStable && maxZeroPos < 1e-6f && maxZeroRot < .001f, "dt0 target switches preserve actual composition through Default weapon fades");
        check(maxSocketStep < .005f, "Default weapon fade does not drop the entire socket correction at its boundary");
        // Pure weapon outputs have no remaining Default contributor. Hashes are
        // compared against the isolated baseline to protect 1H/2H/Rifle paths.
        in.machine.overlay = weapon;
        for (int k = 0; k < 180; ++k) ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info);
        uint64_t hash = 1469598103934665603ull;
        for (int k = 0; k < 30; ++k)
        {
            match.clipTime = 1.9f + k * dt;
            ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info);
            const auto* bytes = reinterpret_cast<const uint8_t*>(output.data());
            for (size_t b = 0; b < output.size() * sizeof(BoneTransform); ++b) hash = (hash ^ bytes[b]) * 1099511628211ull;
        }
        std::printf("pureWeapon overlay=%d hash=%016llx\n", int(weapon), static_cast<unsigned long long>(hash));
    }
    std::printf("FeminineUnarmedContinuity %d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

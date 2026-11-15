// Feminine figure pass (FeminineFigure.hpp) through the real pose evaluator on the woman data: on the unarmed idle the shoulder
// sockets drop and set back, the head sits lower and the hands come closer to the body; hips, spine and legs are untouched; a
// weapon pose, a male body and a traversal clip get no figure pass; the weight never moves faster than its fade time.
#include "../src/mm/MotionPose.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace utpp;
using namespace utpp::mm;

int main(int argc, char** argv)
{
    if (argc != 4) return 2;
    std::string error;
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!db->LoadFromFile(argv[1], motion::kRigIdentityWoman, error) || !ovf->LoadFromFile(argv[2], ov::kRigIdentityWomanOv, error))
    {
        std::puts(error.c_str());
        return 2;
    }
    std::ifstream stream(argv[3]);
    std::stringstream text;
    text << stream.rdbuf();
    ov::StyleFamilyData style;
    if (!style.Parse(text.str(), &error)) return 2;
    int idle = -1;
    for (uint32_t i = 0; i < db->NumClips(); ++i)
        if (db->ClipName(i) == "M_Neutral_Stand_Idle_Loop") idle = int(i);
    if (idle < 0) return 2;
    const uint32_t n = db->NumBones();
    auto bone = [&](const char* name) {
        for (uint32_t i = 0; i < n; ++i)
            if (db->BoneName(i) == name) return int(i);
        return -1;
    };
    const int bHips = bone("Hips"), bSpine3 = bone("Spine3"), bHead = bone("Head"), bArmL = bone("LeftArm"), bArmR = bone("RightArm"),
              bHandL = bone("LeftHand"), bHandR = bone("RightHand"), bFootL = bone("LeftFoot"), bFootR = bone("RightFoot");
    if (bHips < 0 || bSpine3 < 0 || bHead < 0 || bArmL < 0 || bArmR < 0 || bHandL < 0 || bHandR < 0 || bFootL < 0 || bFootR < 0) return 2;
    int pass = 0, fail = 0;
    auto check = [&](bool ok, const std::string& label) { (ok ? pass : fail)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", label.c_str()); };
    auto model = [&](const std::vector<BoneTransform>& pose, int b) {
        math::Quat q{};
        math::Vec3 p{};
        std::vector<int> chain;
        for (int k = b; k >= 0; k = db->BoneParent(uint32_t(k))) chain.push_back(k);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            p = p + math::QuatRotate(q, pose[size_t(*it)].p);
            q = math::QuatNormalize(math::QuatMul(q, pose[size_t(*it)].q));
        }
        return p;
    };
    struct Run
    {
        std::vector<BoneTransform> out;
        motionpose::FrameInfo info;
        float maxWeightStep = 0.0f;
        bool ok = true;
    };
    // settles a pose for 2 s at 60 Hz, then optionally switches the inputs and runs on
    auto settle = [&](bool woman, bool figure, ov::PoseOverlay overlay, bool montage, ov::PoseOverlay thenOverlay, bool thenMontage, int thenFrames) {
        Run r;
        r.out.resize(n);
        auto ev = std::make_unique<motionpose::PoseEvaluator>();
        if (!ev->Init(db.get(), ovf.get(), &style, 2, error)) { r.ok = false; return r; }
        ev->SetFeminineFigure(figure);
        motionpose::OverlayFrameInputs in;
        in.bodyWoman = woman;
        in.machine.style = woman ? ov::Style::Feminine : ov::Style::Masculine;
        in.machine.overlay = overlay;
        in.machine.wantsReady = true;
        in.montageBase = montage;
        in.gripSkip = true;
        motion::MatchResult match;
        match.clip = idle;
        match.clipTime = 0.5f;
        float prev = 0.0f;
        for (int k = 0; k < 120 + thenFrames; ++k)
        {
            if (k == 120) { in.machine.overlay = thenOverlay; in.montageBase = thenMontage; }
            r.ok = ev->Evaluate(1.0f / 60.0f, match, in, r.out.data(), n, &r.info) && r.info.finite && r.ok;
            if (k > 0) r.maxWeightStep = std::fmax(r.maxWeightStep, std::fabs(r.info.figure.weight - prev));
            prev = r.info.figure.weight;
        }
        return r;
    };
    const auto D = ov::PoseOverlay::Default;
    const Run off = settle(true, false, D, false, D, false, 0);
    const Run on = settle(true, true, D, false, D, false, 0);
    check(off.ok && on.ok, "the evaluator runs with the figure pass off and on");
    check(on.info.figure.ran && on.info.figure.applied && on.info.figure.weight > 0.99f && off.info.figure.weight == 0.0f,
          "unarmed idle: the pass runs at full weight when on, weight 0 when off");
    const float drop = 0.5f * ((model(off.out, bArmL).z - model(on.out, bArmL).z) + (model(off.out, bArmR).z - model(on.out, bArmR).z));
    const float head = model(off.out, bHead).z - model(on.out, bHead).z;
    const float handsOff = gik::Len(model(off.out, bHandL) - model(off.out, bHandR));
    const float handsOn = gik::Len(model(on.out, bHandL) - model(on.out, bHandR));
    const float widthOff = gik::Len(model(off.out, bArmL) - model(off.out, bArmR));
    const float widthOn = gik::Len(model(on.out, bArmL) - model(on.out, bArmR));
    const float back = 0.5f * ((model(off.out, bArmL).y - model(on.out, bArmL).y) + (model(off.out, bArmR).y - model(on.out, bArmR).y));
    std::printf("INFO socket drop %.4f m, set back %.4f m, head lower %.4f m, socket width %.4f -> %.4f m, hand distance %.4f -> %.4f m\n",
                double(drop), double(back), double(head), double(widthOff), double(widthOn), double(handsOff), double(handsOn));
    check(drop > 0.020f && drop < 0.040f, "unarmed idle: the shoulder sockets sit 2 - 4 cm lower");
    check(back > 0.010f && back < 0.035f, "unarmed idle: the shoulder sockets sit 1 - 3.5 cm further back");
    check(head > 0.017f && head < 0.023f, "unarmed idle: the head sits 2 cm lower");
    check(std::fabs(widthOn - widthOff) < 0.008f, "unarmed idle: the socket width stays within 0.8 cm (the shoulder width option owns it)");
    check(handsOff - handsOn > 0.08f && handsOff - handsOn < 0.20f, "unarmed idle: the hands hang 8 - 20 cm closer together");
    bool same = true;
    for (const int b : {bHips, bSpine3, bFootL, bFootR})
        same = same && gik::Len(model(off.out, b) - model(on.out, b)) == 0.0f;
    check(same, "unarmed idle: hips, chest and feet are where they were, bit for bit");
    // weapon pose, male body, traversal clip: no figure pass
    const auto P2 = ov::PoseOverlay::Pistol2H;
    const Run wOff = settle(true, false, P2, false, P2, false, 0), wOn = settle(true, true, P2, false, P2, false, 0);
    bool equal = wOff.ok && wOn.ok;
    for (uint32_t i = 0; i < n && equal; ++i)
        equal = std::memcmp(&wOff.out[i], &wOn.out[i], sizeof(BoneTransform)) == 0;
    check(equal && wOn.info.figure.weight == 0.0f, "settled two-handed pistol pose: bit for bit the pose without the pass");
    const Run mOff = settle(false, false, D, false, D, false, 0), mOn = settle(false, true, D, false, D, false, 0);
    equal = mOff.ok && mOn.ok;
    for (uint32_t i = 0; i < n && equal; ++i)
        equal = std::memcmp(&mOff.out[i], &mOn.out[i], sizeof(BoneTransform)) == 0;
    check(equal && !mOn.info.figure.ran, "male body: the pass never runs");
    const Run tr = settle(true, true, D, true, D, true, 0);
    check(tr.ok && tr.info.figure.weight == 0.0f, "traversal clip from the start: weight 0");
    // fades: never faster than the fade time, and they arrive
    const float stepLimit = (1.0f / 60.0f) / motionpose::kFigureSeconds + 1e-4f;
    const Run toTrav = settle(true, true, D, false, D, true, 60);
    check(toTrav.ok && toTrav.info.figure.weight == 0.0f && toTrav.maxWeightStep <= stepLimit && toTrav.maxWeightStep > 0.0f,
          "traversal start: the weight fades to 0 at the fade rate, never in one frame");
    const Run toGun = settle(true, true, D, false, P2, false, 120);
    check(toGun.ok && toGun.info.figure.weight < 0.01f && toGun.maxWeightStep <= stepLimit, "weapon draw: the weight fades out, never in one frame");
    const Run fromGun = settle(true, true, P2, false, D, false, 120);
    check(fromGun.ok && fromGun.info.figure.weight > 0.99f && fromGun.maxWeightStep <= stepLimit, "holster: the weight fades in, never in one frame");
    std::printf("FeminineFigure %d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}

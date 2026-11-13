// Forearm roll convention (src/mm/ForearmTwist.hpp, recording 2026-10-25 video 01:32): the forearm twist about the
// forearm -> hand axis moves into the hand. Checks on the real woman skeleton / data:
//  (1) diag fixture: LeftForeArm q=(-0.7122,0.5454,-0.2452,0.3678) reads ~-125.4 deg of twist, 0 after the pass;
//  (2) 400 random arm poses, both sides: every model transform except the forearms unchanged (< 1e-4), forearm model
//      position unchanged, forearm twist ~0, clavicle / upper arm / every non-forearm-child local bit for bit,
//      second pass is a no-op (idempotent);
//  (3) PoseEvaluator, Pistol2H relaxed stand idle (key 36), Feminine, M_Neutral_Stand_Idle_Loop at 30/60/144 Hz:
//      the output forearm twist is ~0 on both sides while the pass moved > 90 deg on the left (the reported pose);
//  (4) rifle-hand-fix 2026-11-09, LimitHandTwistBySwivel on 600 random left-arm poses: twist above the limit brought to it,
//      only the upper-arm and hand locals change, hand / weapon / finger model transforms exact, bit for bit below the limit
//      and at weight 0, continuous in the weight;
//  (5) PoseEvaluator Pistol2H relaxed / ready and Rifle relaxed / ready / aiming (30 / 144 Hz): output support-hand twist
//      within the limit (no swivel for Rifle); no swivel for the male body (bodyWoman false).
// Arguments: woman .utppmm, woman .utppov, style weights. Mutations (UTPP_FOREARMTWIST_MUTATION) must print "FAIL ":
// 1 = pass not called by PoseEvaluator, 2 = forearm children not compensated.
#include "../src/mm/MotionPose.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
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
uint32_t g_seed = 12345u;
float Rnd() // [-1, 1)
{
    g_seed = g_seed * 1664525u + 1013904223u;
    return static_cast<float>((g_seed >> 8) & 0xFFFFFF) / static_cast<float>(0x800000) - 1.0f;
}
math::Quat RandomQuat()
{
    for (;;)
    {
        const math::Quat q{Rnd(), Rnd(), Rnd(), Rnd()};
        const float n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (n > 0.05f && n <= 1.0f)
            return math::QuatNormalize(q);
    }
}
float QuatErr(const math::Quat& a, const math::Quat& b) // max component error up to the sign
{
    const float s = math::QuatDot(a, b) < 0.0f ? -1.0f : 1.0f;
    return std::max(std::max(std::fabs(a.x - s * b.x), std::fabs(a.y - s * b.y)),
                    std::max(std::fabs(a.z - s * b.z), std::fabs(a.w - s * b.w)));
}
float ForearmTwistDeg(const ov::Pose& aPose, int aFa, int aHand)
{
    const math::Vec3 t = aPose.b[aHand].t;
    const float l = std::sqrt(math::Dot(t, t));
    return ov::TwistAngleDeg(math::QuatNormalize(aPose.b[aFa].r), t * (1.0f / l));
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::puts("usage: utpp_forearm_twist_test <woman.utppmm> <woman.utppov> <style_weights_woman.txt>");
        return 2;
    }
    std::string error;
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!db->LoadFromFile(argv[1], motion::kRigIdentityWoman, error) || !ovf->LoadFromFile(argv[2], ov::kRigIdentityWomanOv, error))
    {
        std::printf("load error: %s\n", error.c_str());
        return 2;
    }
    std::ifstream stream(argv[3]);
    std::stringstream text;
    text << stream.rdbuf();
    ov::StyleFamilyData style;
    if (!style.Parse(text.str(), &error))
        return 2;
    const ov::Skeleton& sk = ovf->Skel();
    const gik::Rig rig = gik::FindRig(sk.n, [&](int i) { return ovf->BoneName(i); });
    if (!rig.Valid(sk))
        return 2;
    ov::ForearmTwistRig ftr;
    ftr.Build(sk, rig.left.lower, rig.left.hand, rig.right.lower, rig.right.hand);
    Check(ftr.Valid(0) && ftr.Valid(1), "forearm twist rig resolves both arms (forearm children include the hand)");
    const int fa[2] = {rig.left.lower, rig.right.lower}, hd[2] = {rig.left.hand, rig.right.hand};
    const int up[2] = {rig.left.upper, rig.right.upper};
    auto isForearm = [&](int b) { return b == fa[0] || b == fa[1]; };
    auto isForearmChild = [&](int b) { return sk.parent[b] >= 0 && isForearm(sk.parent[b]); };

    auto base = std::make_unique<ov::Pose>();
    const int key36 = ovf->FindOverlayPose(ov::PoseOverlay::Pistol2H, ov::AimState::Relaxed, false, false);
    if (key36 < 0)
        return 2;
    ovf->FramePose(ovf->PoseAt(key36).firstFrame, *base);

    // (1) diag fixture (diag_all.txt arm-stretch-0132): -125.4 deg in LeftForeArm
    {
        auto p = std::make_unique<ov::Pose>(*base);
        p->b[fa[0]].r = math::QuatNormalize(math::Quat{-0.7122f, 0.5454f, -0.2452f, 0.3678f});
        const float before = ForearmTwistDeg(*p, fa[0], hd[0]);
        float moved[2];
        ov::MoveForearmTwistToHands(ftr, *p, moved);
        const float after = ForearmTwistDeg(*p, fa[0], hd[0]);
        std::printf("fixture LeftForeArm twist before %.2f moved %.2f after %.5f deg (hand axis %.4f %.4f %.4f)\n", before,
                    moved[0], after, p->b[hd[0]].t.x, p->b[hd[0]].t.y, p->b[hd[0]].t.z);
        Check(std::fabs(before + 125.4f) < 0.5f, "fixture reads the diagnosed -125.4 deg forearm twist");
        Check(std::fabs(moved[0] - before) < 0.01f && std::fabs(after) < 0.01f, "fixture twist moved out of the forearm");
    }

    // (2) random arm poses
    {
        auto p = std::make_unique<ov::Pose>();
        auto q = std::make_unique<ov::Pose>();
        math::Vec3 pos0[ov::kMaxBones], pos1[ov::kMaxBones];
        math::Quat rot0[ov::kMaxBones], rot1[ov::kMaxBones];
        float maxPos = 0, maxRot = 0, maxFaPos = 0, maxTwist = 0, maxIdem = 0, maxBefore[2] = {0, 0};
        bool otherLocalsBitwise = true, handTBitwise = true;
        for (int trial = 0; trial < 400; ++trial)
        {
            *p = *base;
            for (int s = 0; s < 2; ++s)
            {
                p->b[fa[s]].r = RandomQuat();
                p->b[hd[s]].r = RandomQuat();
                if (trial % 3 == 0)
                    p->b[up[s]].r = RandomQuat();
            }
            for (int s = 0; s < 2; ++s)
                maxBefore[s] = std::max(maxBefore[s], std::fabs(ForearmTwistDeg(*p, fa[s], hd[s])));
            *q = *p;
            ov::MeshPositions(sk, *p, pos0, rot0);
            float moved[2];
            ov::MoveForearmTwistToHands(ftr, *p, moved);
            ov::MeshPositions(sk, *p, pos1, rot1);
            for (int b = 0; b < sk.n; ++b)
            {
                const float dp = gik::Len(pos1[b] - pos0[b]);
                if (isForearm(b))
                    maxFaPos = std::max(maxFaPos, dp);
                else
                {
                    maxPos = std::max(maxPos, dp);
                    maxRot = std::max(maxRot, QuatErr(rot1[b], rot0[b]));
                }
                if (!isForearm(b) && !isForearmChild(b) && std::memcmp(&p->b[b], &q->b[b], sizeof(ov::Xf)) != 0)
                    otherLocalsBitwise = false;
                if ((b == hd[0] || b == hd[1]) && std::memcmp(&p->b[b].t, &q->b[b].t, sizeof(math::Vec3)) != 0)
                    handTBitwise = false;
            }
            for (int s = 0; s < 2; ++s)
                maxTwist = std::max(maxTwist, std::fabs(ForearmTwistDeg(*p, fa[s], hd[s])));
            *q = *p;
            ov::MoveForearmTwistToHands(ftr, *p, moved);
            for (int b = 0; b < sk.n; ++b)
                maxIdem = std::max(maxIdem, QuatErr(p->b[b].r, q->b[b].r));
        }
        std::printf("random 400: twist before max L %.1f R %.1f deg; after max %.5f deg; model err pos %.2e m rot %.2e; "
                    "forearm pos %.2e m; second pass %.2e\n",
                    maxBefore[0], maxBefore[1], maxTwist, maxPos, maxRot, maxFaPos, maxIdem);
        Check(maxBefore[0] > 150.0f && maxBefore[1] > 150.0f, "random poses cover large forearm twist on both sides");
        Check(maxPos < 1e-4f && maxRot < 1e-4f, "hand / weapon / finger / all other model transforms unchanged (<1e-4)");
        Check(maxFaPos < 1e-6f, "forearm model position unchanged");
        Check(maxTwist < 0.01f, "forearm twist ~0 after the pass on both sides");
        Check(maxIdem < 1e-5f, "second pass changes nothing (idempotent)");
        Check(otherLocalsBitwise, "clavicles, upper arms and every bone outside forearm + children keep their locals bit for bit");
        Check(handTBitwise, "hand local translations bit for bit (on the twist axis)");
    }

    // (3) PoseEvaluator on the reported state: Pistol2H relaxed stand idle (key 36)
    int idle = -1;
    for (uint32_t i = 0; i < db->NumClips(); ++i)
        if (db->ClipName(i) == "M_Neutral_Stand_Idle_Loop")
            idle = static_cast<int>(i);
    int dbFa[2] = {-1, -1}, dbHand[2] = {-1, -1};
    for (uint32_t i = 0; i < db->NumBones(); ++i)
    {
        const auto n = db->BoneName(i);
        if (n == "LeftForeArm") dbFa[0] = static_cast<int>(i);
        if (n == "RightForeArm") dbFa[1] = static_cast<int>(i);
        if (n == "LeftHand") dbHand[0] = static_cast<int>(i);
        if (n == "RightHand") dbHand[1] = static_cast<int>(i);
    }
    if (idle < 0 || dbFa[0] < 0 || dbFa[1] < 0 || dbHand[0] < 0 || dbHand[1] < 0)
        return 2;
    for (int hz : {30, 60, 144})
    {
        auto ev = std::make_unique<motionpose::PoseEvaluator>();
        if (!ev->Init(db.get(), ovf.get(), &style, 2, error))
            return 2;
        motionpose::OverlayFrameInputs in;
        in.bodyWoman = true;
        in.machine.style = ov::Style::Feminine;
        in.machine.overlay = ov::PoseOverlay::Pistol2H;
        in.authoredTwoHandGrip = true;
        motion::MatchResult match;
        match.clip = idle;
        match.clipTime = 0.5f;
        std::vector<BoneTransform> out(db->NumBones());
        motionpose::FrameInfo info;
        bool ok = true;
        float maxOut[2] = {0, 0};
        for (int k = 0; k < 4 * hz; ++k)
        {
            match.clipTime = 0.5f + static_cast<float>(k) / static_cast<float>(hz);
            ok = ev->Evaluate(1.0f / static_cast<float>(hz), match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && ok;
            if (k >= 2 * hz)
                for (int s = 0; s < 2; ++s)
                {
                    const math::Vec3 t = out[dbHand[s]].p;
                    const float l = std::sqrt(math::Dot(t, t));
                    maxOut[s] = std::max(maxOut[s], std::fabs(ov::TwistAngleDeg(math::QuatNormalize(out[dbFa[s]].q), t * (1.0f / l))));
                }
        }
        const bool settled = ev->Machine().poses.count == 1 && ev->Machine().poses.key[0] == 36;
        std::printf("evaluator %d Hz: key36 %d, moved L %.1f R %.1f deg, output |forearm twist| max L %.4f R %.4f deg (last 2 s)\n",
                    hz, settled ? 1 : 0, info.forearmTwistMovedDeg[0], info.forearmTwistMovedDeg[1], maxOut[0], maxOut[1]);
        Check(ok && info.finite && info.layered && settled, "evaluator settles in Pistol2H relaxed stand idle (key 36)");
        Check(std::fabs(info.forearmTwistMovedDeg[0]) > 90.0f, "reported left forearm roll (> 90 deg) moved into the hand");
        Check(maxOut[0] < 0.05f && maxOut[1] < 0.05f, "output forearms are pure hinges (twist ~0) on both sides");
    }

    // (4) rifle-hand-fix 2026-11-09: LimitHandTwistBySwivel on 600 random left-arm poses (forearm pass first, the evaluator order)
    const float lim = ov::kSupportHandTwistLimitDeg, maxSw = ov::kSupportHandMaxSwivelDeg;
    auto handTwist = [](const math::Quat& aR, math::Vec3 aT) {
        const float l = std::sqrt(math::Dot(aT, aT));
        return ov::TwistAngleDeg(math::QuatNormalize(aR), aT * (1.0f / l));
    };
    auto samePose = [&](const ov::Pose& aA, const ov::Pose& aB) {
        for (int b = 0; b < sk.n; ++b)
            if (std::memcmp(&aA.b[b], &aB.b[b], sizeof(ov::Xf)) != 0)
                return false;
        return true;
    };
    {
        auto p = std::make_unique<ov::Pose>();
        auto q = std::make_unique<ov::Pose>();
        auto h = std::make_unique<ov::Pose>();
        math::Vec3 pos0[ov::kMaxBones], pos1[ov::kMaxBones];
        math::Quat rot0[ov::kMaxBones], rot1[ov::kMaxBones];
        int over = 0, reached = 0, capped = 0, notApplied = 0;
        float maxPos = 0, maxRot = 0, maxAfter = 0, maxSwivel = 0, maxHalf = 0, maxReport = 0, maxBefore = 0;
        bool belowBitwise = true, zeroBitwise = true, othersBitwise = true, cappedBetter = true;
        for (int trial = 0; trial < 600; ++trial)
        {
            *p = *base;
            p->b[up[0]].r = RandomQuat();
            p->b[fa[0]].r = RandomQuat();
            p->b[hd[0]].r = RandomQuat();
            float moved[2];
            ov::MoveForearmTwistToHands(ftr, *p, moved);
            *q = *p;
            *h = *p;
            ov::MeshPositions(sk, *p, pos0, rot0);
            const ov::HandTwistLimitResult r = ov::LimitHandTwistBySwivel(*p, up[0], fa[0], hd[0], ftr.childCount[0], lim, maxSw, 1.0f);
            ov::MeshPositions(sk, *p, pos1, rot1);
            maxBefore = std::max(maxBefore, std::fabs(r.twistBeforeDeg));
            if (std::fabs(r.twistBeforeDeg) > lim)
            {
                ++over;
                const float after = std::fabs(handTwist(p->b[hd[0]].r, p->b[hd[0]].t));
                maxReport = std::max(maxReport, std::fabs(after - std::fabs(r.twistAfterDeg)));
                maxSwivel = std::max(maxSwivel, std::fabs(r.swivelDeg));
                if (!r.applied)
                {
                    ++notApplied; // out of reach and no grid swivel lowers the twist: must be untouched
                    cappedBetter = cappedBetter && samePose(*p, *q);
                }
                else if (after <= lim + 0.01f)
                {
                    ++reached;
                    maxAfter = std::max(maxAfter, after);
                }
                else
                {
                    ++capped;
                    cappedBetter = cappedBetter && after < std::fabs(r.twistBeforeDeg);
                }
                const ov::HandTwistLimitResult r5 = ov::LimitHandTwistBySwivel(*h, up[0], fa[0], hd[0], ftr.childCount[0], lim, maxSw, 0.5f);
                maxHalf = std::max(maxHalf, std::fabs(r5.swivelDeg - 0.5f * r.swivelDeg));
                *h = *q;
                ov::LimitHandTwistBySwivel(*h, up[0], fa[0], hd[0], ftr.childCount[0], lim, maxSw, 0.0f);
                zeroBitwise = zeroBitwise && samePose(*h, *q);
            }
            else
                belowBitwise = belowBitwise && !r.applied && samePose(*p, *q);
            for (int b = 0; b < sk.n; ++b)
            {
                if (b != fa[0] && b != up[0])
                {
                    maxPos = std::max(maxPos, gik::Len(pos1[b] - pos0[b]));
                    maxRot = std::max(maxRot, QuatErr(rot1[b], rot0[b]));
                }
                if (b != up[0] && b != hd[0] && std::memcmp(&p->b[b], &q->b[b], sizeof(ov::Xf)) != 0)
                    othersBitwise = false;
            }
            maxPos = std::max(maxPos, gik::Len(pos1[up[0]] - pos0[up[0]]));
        }
        std::printf("swivel random 600: over %d (before max %.1f deg), reached %d (after max %.3f), capped %d, not applied %d; "
                    "|swivel| max %.2f deg; model err pos %.2e m rot %.2e; half-weight err %.2e deg; reported-twist err %.2e deg\n",
                    over, maxBefore, reached, maxAfter, capped, notApplied, maxSwivel, maxPos, maxRot, maxHalf, maxReport);
        Check(over > 150 && reached > over / 2, "random poses cover hand twist above the limit, most within reach of the swivel");
        Check(maxAfter <= lim + 0.01f, "hand twist above the limit is brought to the limit when within reach");
        Check(maxSwivel <= maxSw + 1e-3f && cappedBetter, "swivel bounded; out of reach: twist only lowered, never raised");
        Check(maxPos < 1e-4f && maxRot < 1e-4f, "shoulder, hand, weapon, fingers and every bone except upper arm / forearm keep model transforms");
        Check(othersBitwise, "every local except the upper arm and hand (forearm included) bit for bit");
        Check(belowBitwise, "hand twist within the limit: pose untouched bit for bit");
        Check(zeroBitwise && maxHalf < 1e-3f, "weight 0: bit for bit; weight 0.5: half the swivel (continuous)");
        Check(maxReport < 0.01f, "reported twist after matches the output hand");
    }

    // (5) PoseEvaluator, Feminine, authored 2H grip: Pistol2H relaxed / ready (the reported 129-133 deg wrist twist) and Rifle relaxed /
    //     ready / aiming (within the limit: no swivel), Female V; the male body (bodyWoman false) is never swivelled.
    {
        struct Case
        {
            ov::PoseOverlay overlay;
            bool ready, aim, woman;
            int key;
            const char* label;
        };
        const Case cases[] = {{ov::PoseOverlay::Pistol2H, false, false, true, 36, "Pistol2H relaxed"},
                              {ov::PoseOverlay::Pistol2H, true, false, true, 40, "Pistol2H ready"},
                              {ov::PoseOverlay::Rifle, false, false, true, 12, "Rifle relaxed"},
                              {ov::PoseOverlay::Rifle, true, false, true, 16, "Rifle ready"},
                              {ov::PoseOverlay::Rifle, false, true, true, 20, "Rifle aiming"},
                              {ov::PoseOverlay::Pistol2H, false, false, false, 36, "Pistol2H relaxed, bodyWoman false"}};
        for (const Case& c : cases)
            for (int hz : {30, 144})
            {
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                if (!ev->Init(db.get(), ovf.get(), &style, 2, error))
                    return 2;
                motionpose::OverlayFrameInputs in;
                in.bodyWoman = c.woman;
                in.machine.style = ov::Style::Feminine;
                in.machine.overlay = c.overlay;
                in.machine.wantsReady = c.ready;
                in.machine.wantsToAim = c.aim;
                in.authoredTwoHandGrip = true;
                motion::MatchResult match;
                match.clip = idle;
                std::vector<BoneTransform> out(db->NumBones());
                motionpose::FrameInfo info;
                bool ok = true;
                float maxTwistOut = 0, maxBefore = 0, maxSwivel = 0;
                for (int k = 0; k < 4 * hz; ++k)
                {
                    match.clipTime = 0.5f + static_cast<float>(k) / static_cast<float>(hz);
                    ok = ev->Evaluate(1.0f / static_cast<float>(hz), match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && ok;
                    if (k < 2 * hz)
                        continue;
                    maxTwistOut = std::max(maxTwistOut, std::fabs(handTwist(out[dbHand[0]].q, out[dbHand[0]].p)));
                    maxBefore = std::max(maxBefore, std::fabs(info.supportTwistBeforeDeg));
                    maxSwivel = std::max(maxSwivel, std::fabs(info.supportSwivelDeg));
                }
                const bool settled = ev->Machine().poses.count == 1 && ev->Machine().poses.key[0] == c.key;
                std::printf("evaluator %s %d Hz: key %d settled %d, support twist before max %.1f, output max %.2f deg, |swivel| max %.1f deg\n",
                            c.label, hz, c.key, settled ? 1 : 0, maxBefore, maxTwistOut, maxSwivel);
                Check(ok && info.finite && info.layered && settled, "evaluator settles in the 2H stand idle state");
                if (!c.woman)
                {
                    Check(maxSwivel == 0.0f && maxTwistOut > 100.0f, "bodyWoman false: no swivel (the male output keeps its twist)");
                    continue;
                }
                Check(maxTwistOut <= lim + 0.05f && maxSwivel <= maxSw + 1e-3f, "output support-hand twist within the limit, swivel bounded");
                if (c.overlay == ov::PoseOverlay::Pistol2H && !c.ready)
                    Check(maxBefore > 100.0f, "Pistol2H relaxed: the reported > 100 deg support twist is seen before the swivel");
                if (c.overlay == ov::PoseOverlay::Rifle)
                    Check(maxSwivel == 0.0f, "Rifle stand idle states: twist within the limit, no swivel");
            }
    }
    std::printf("ForearmTwist %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

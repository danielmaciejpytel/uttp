// rifle-grip-stock 2026-11-10 (src/mm/SupportGrip.hpp + MotionPose.hpp): support hand on the handguard and stock in the shoulder of the long
// guns (Rifle overlay), on the real data of both bodies.
//  (1) table: one entry per long-gun recoil family (12-19, 23-27), none for pistols / retired ids; unit quaternions; finger bones resolve
//      on both skeletons; stock contact behind the weapon origin;
//  (2) PoseEvaluator Rifle Ready / Aiming steady (Female V shoulder widths 0 / 2.5 / 5 cm per side, Male V; 30 and 144 Hz; Ajax, Saratoga,
//      Copperhead, Hercules; without an aim point and with one at pitch -40 / 0 / +40, yaw at its clamp): LeftHand on the game grip of the
//      weapon (position <= 0.5 cm, rotation <= 3 deg) with the arm inside the soft reach band and bent; stock contact on the shoulder pocket
//      (|gap| <= 0.5 cm); support-hand twist within the 85 deg limit; (2b) every family; (2c) the stock gap of every stock-pass weapon;
//  (3) unchanged: the Pistol2H overlay with an entry, and the Rifle overlay after its entry went away and the route faded out, are bit for
//      bit the evaluator that never had an input;
//  (4) continuity at 30 Hz over relaxed -> ready -> aim -> walk -> run -> sprint -> stop, hand-back, a weapon / overlay switch while the
//      gate falls, a holster and a one-frame data drop-out: the left hand moves at most 2.5 cm per frame relative to the weapon's grip
//      point (the route never jumps).
// Arguments: woman .utppmm, woman .utppov, woman style, man .utppmm, man .utppov, man style.
// Mutation (UTPP_SUPPORTGRIP_MUTATION=1, pass compiled out = the build before) must print "FAIL ".
#include "../src/mm/MotionPose.hpp"
#include "../src/mm/RecoilProfiles.hpp"
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
struct Data
{
    std::unique_ptr<motion::MotionDatabase> db = std::make_unique<motion::MotionDatabase>();
    std::unique_ptr<ov::OverlayFile> ov = std::make_unique<ov::OverlayFile>();
    ov::StyleFamilyData style;
    bool woman = true;
    int idle = -1, walk = -1, run = -1, sprint = -1;
};
bool Load(Data& aD, const char* aDb, const char* aOv, const char* aStyle, bool aWoman)
{
    std::string error;
    aD.woman = aWoman;
    if (!aD.db->LoadFromFile(aDb, aWoman ? motion::kRigIdentityWoman : motion::kRigIdentityMan, error) ||
        !aD.ov->LoadFromFile(aOv, aWoman ? ov::kRigIdentityWomanOv : ov::kRigIdentityManOv, error))
    {
        std::printf("load error: %s\n", error.c_str());
        return false;
    }
    std::ifstream stream(aStyle);
    std::stringstream text;
    text << stream.rdbuf();
    if (!aD.style.Parse(text.str(), &error))
        return false;
    for (uint32_t i = 0; i < aD.db->NumClips(); ++i)
    {
        const std::string_view n = aD.db->ClipName(i);
        if (n == "M_Neutral_Stand_Idle_Loop") aD.idle = static_cast<int>(i);
        if (n == "M_Neutral_Walk_Loop_F") aD.walk = static_cast<int>(i);
        if (n == "M_Neutral_Run_Loop_F") aD.run = static_cast<int>(i);
        if (n == "M_Neutral_Sprint_Loop_F") aD.sprint = static_cast<int>(i);
    }
    return aD.idle >= 0 && aD.walk >= 0 && aD.run >= 0 && aD.sprint >= 0;
}
float AngleDeg(const math::Quat& a, const math::Quat& b)
{
    const float d = std::fabs(math::QuatDot(math::QuatNormalize(a), math::QuatNormalize(b)));
    return 2.0f * std::acos(std::min(1.0f, d)) * 57.29578f;
}
// model-space measures of one output pose
struct Meas
{
    float gripErrCm = 0, gripRotDeg = 0, reach = 0, elbowDeg = 0, gapCm = 0;
    math::Vec3 handRel{}; // LeftHand - grip point of the weapon (m, model)
};
Meas Measure(const motionpose::PoseEvaluator& aEv, const std::vector<BoneTransform>& aOut, const sgrip::Entry& aE)
{
    static ov::Pose p;
    aEv.ToOv(aOut.data(), p);
    const ov::Skeleton& sk = aEv.Skel();
    const gik::Rig& rig = aEv.GripRig();
    math::Quat rw, rh, rs, ru, rl, rp;
    math::Vec3 pw, ph, ps, pu, pl, pp;
    gik::ModelOf(sk, p, rig.weaponRight, rw, pw);
    gik::ModelOf(sk, p, rig.left.hand, rh, ph);
    gik::ModelOf(sk, p, rig.left.upper, ru, pu);
    gik::ModelOf(sk, p, rig.left.lower, rl, pl);
    Meas m;
    const math::Vec3 tgt = pw + math::QuatRotate(rw, aE.t);
    m.handRel = ph - tgt;
    m.gripErrCm = gik::Len(m.handRel) * 100.0f;
    m.gripRotDeg = AngleDeg(rh, math::QuatMul(rw, aE.r));
    const float full = gik::Len(p.b[rig.left.lower].t) + gik::Len(p.b[rig.left.hand].t);
    m.reach = gik::Len(ph - pu) / full;
    math::Vec3 u, v;
    gik::Norm(pl - pu, u);
    gik::Norm(ph - pl, v);
    m.elbowDeg = std::acos(std::clamp(math::Dot(u, v), -1.0f, 1.0f)) * 57.29578f;
    return m;
}
// right elbow flexion (deg, 0 = straight): the reach shift bends the right arm (SlideWeaponBack)
float RightElbowDeg(const motionpose::PoseEvaluator& aEv, const std::vector<BoneTransform>& aOut)
{
    static ov::Pose p;
    aEv.ToOv(aOut.data(), p);
    const gik::Rig& rig = aEv.GripRig();
    math::Quat r;
    math::Vec3 pu, pl, ph, u, v;
    gik::ModelOf(aEv.Skel(), p, rig.right.upper, r, pu);
    gik::ModelOf(aEv.Skel(), p, rig.right.lower, r, pl);
    gik::ModelOf(aEv.Skel(), p, rig.right.hand, r, ph);
    gik::Norm(pl - pu, u);
    gik::Norm(ph - pl, v);
    return std::acos(std::clamp(math::Dot(u, v), -1.0f, 1.0f)) * 57.29578f;
}
float GapCm(const ov::OverlayFile& aOv, const motionpose::PoseEvaluator& aEv, const std::vector<BoneTransform>& aOut, const sgrip::Entry& aE,
            bool aWoman)
{
    static ov::Pose p;
    aEv.ToOv(aOut.data(), p);
    const ov::Skeleton& sk = aEv.Skel();
    int bone = -1;
    for (int i = 0; i < sk.n; ++i)
        if (aOv.BoneName(i) == sgrip::kPocketBone)
            bone = i;
    math::Quat rw, rp;
    math::Vec3 pw, pp;
    gik::ModelOf(sk, p, aEv.GripRig().weaponRight, rw, pw);
    gik::ModelOf(sk, p, bone, rp, pp);
    const sgrip::Pocket& pk = aWoman ? sgrip::kPocketWoman : sgrip::kPocketMan;
    const math::Vec3 pocket = pp + math::QuatRotate(rp, pk.local);
    const math::Vec3 axis = math::QuatRotate(rw, math::Vec3{0.0f, 1.0f, 0.0f});
    return (math::Dot(pw - pocket, axis) + aE.stockY) * 100.0f;
}
// Distance (cm) of the shoulder pocket from the stock's centre line (the barrel axis through WeaponRight; the Ajax / Saratoga butts are
// centred on it within 1 cm): the along-barrel gap alone would also be 0 with the stock beside or above the shoulder.
float PocketOffAxisCm(const ov::OverlayFile& aOv, const motionpose::PoseEvaluator& aEv, const std::vector<BoneTransform>& aOut, bool aWoman)
{
    static ov::Pose p;
    aEv.ToOv(aOut.data(), p);
    const ov::Skeleton& sk = aEv.Skel();
    int bone = -1;
    for (int i = 0; i < sk.n; ++i)
        if (aOv.BoneName(i) == sgrip::kPocketBone)
            bone = i;
    math::Quat rw, rp;
    math::Vec3 pw, pp;
    gik::ModelOf(sk, p, aEv.GripRig().weaponRight, rw, pw);
    gik::ModelOf(sk, p, bone, rp, pp);
    const math::Vec3 pocket = pp + math::QuatRotate(rp, (aWoman ? sgrip::kPocketWoman : sgrip::kPocketMan).local);
    const math::Vec3 axis = math::QuatRotate(rw, math::Vec3{0.0f, 1.0f, 0.0f});
    const math::Vec3 d = pocket - pw;
    return gik::Len(d - axis * math::Dot(d, axis)) * 100.0f;
}
motionpose::OverlayFrameInputs Inputs(const Data& aD, ov::PoseOverlay aOverlay, bool aReady, bool aAim, const sgrip::Entry* aE)
{
    motionpose::OverlayFrameInputs in;
    in.bodyWoman = aD.woman;
    in.maleIdlePistolStance = !aD.woman;
    in.machine.style = aD.woman ? ov::Style::Feminine : ov::Style::Masculine;
    in.machine.overlay = aOverlay;
    in.machine.wantsReady = aReady;
    in.machine.wantsToAim = aAim;
    in.authoredTwoHandGrip = aOverlay != ov::PoseOverlay::Default;
    in.supportGrip = aOverlay == ov::PoseOverlay::Rifle ? aE : nullptr;
    return in;
}
// aim inputs of the steady loops (review round 5): none; an aim point at pitch -40 / +40 (yaw 0); pitch -40 / 0 / +40 with the yaw at its
// clamp (both sides at pitch 0)
struct AimVar
{
    bool have;
    float pitchDeg, yawDeg;
};
constexpr AimVar kAimVars[] = {{false, 0.0f, 0.0f},
                               {true, -40.0f, 0.0f},
                               {true, 40.0f, 0.0f},
                               {true, -40.0f, motionpose::kAimSpineYawClampDeg},
                               {true, 0.0f, motionpose::kAimSpineYawClampDeg},
                               {true, 40.0f, motionpose::kAimSpineYawClampDeg},
                               {true, 0.0f, -motionpose::kAimSpineYawClampDeg}};
// check label + the aim point (none: the label as before)
const char* Lbl(const char* aBase, const AimVar& aA)
{
    static char buf[320];
    if (!aA.have)
        return aBase;
    std::snprintf(buf, sizeof buf, "%s [aim point p %+.0f y %+.0f]", aBase, aA.pitchDeg, aA.yawDeg);
    return buf;
}
void SetAim(motionpose::OverlayFrameInputs& aIn, const AimVar& aA)
{
    aIn.haveAim = aA.have;
    aIn.aimPitchDeg = aA.pitchDeg;
    aIn.aimYawDeg = aA.yawDeg;
}
// aim pitch away from neutral (round 6: own expectations there, "never worse than without the pass, and smooth")
bool Pitched(const AimVar& aA) { return aA.have && aA.pitchDeg != 0.0f; }
// Round 7: mesh-derived bounds at aim pitch +-40 for the two groups where "without the pass" is no
// baseline (Copperhead option A puts the grip before the stock; the Ajax butt hangs 4-5 cm off the body without the pass).
// Source: Source/tools/films/gripstock_pitch.py -> Build/work/rifle-grip-stock-20261110/r7_pitch_copperhead_{woman,man}.json and
// r7_pitch_ajax_{woman,man}.json (log r7_pitch.log / r7_pitch2.log): evaluator pose at aim pitch +-40 (an aim point with the barrel
// loop - 0.0-0.55 cm deeper and 0.7 cm further off axis than this test's pose), the game weapon mesh against the skinned body.
// Measured there: torso / chest / shoulder skin 0; right upper arm skin in the weapon
//   Copperhead p-40 / p+40: F (width 0 / 2.5 / 5 cm) 0.91 / 0.66, 0.44 / 0.73, 0.44 / 0.74 cm; M 0.71 / 0.60 cm
//     (accepted at pitch 0: F 1.03 / M 0.51 cm; rule: <= accepted + 0.3 cm);
//   Ajax p-40: 0 cm (F all widths, M).
// Bound = the measured depth / off-axis distance of that pose + 0.5 cm. Pitch 0 keeps its own bounds; other weapons keep round 6.
float MeshWidthKey(float aFem) { return aFem < 0.0f ? 0.025f : aFem; } // RunSteady -1 = the evaluator default (kFeminineShoulderAmount 0.025)
// Copperhead: minimum stock gap (cm, < 0 = butt behind the shoulder point) allowed at pitch +-40
float CopperheadPitchedGapBound(bool aWoman, float aFem, float aPitch)
{
    const bool down = aPitch < 0.0f;
    if (!aWoman)
        return (down ? -7.60f : -6.70f) - 0.5f;
    const float w = MeshWidthKey(aFem);
    const float g = w < 0.0125f ? (down ? -6.37f : -4.24f) : w < 0.0375f ? (down ? -6.65f : -4.45f) : (down ? -6.31f : -4.45f);
    return g - 0.5f;
}
// Ajax at pitch -40: maximum shoulder-pocket distance from the stock centre line (cm)
float AjaxPitchDownOffBound(bool aWoman, float aFem)
{
    if (!aWoman)
        return 2.99f + 0.5f;
    const float w = MeshWidthKey(aFem);
    return (w < 0.0125f ? 7.25f : w < 0.0375f ? 6.79f : 6.59f) + 0.5f;
}
// Steady state of one input set: 3 s at aHz, measured over the last second, against the grip / stock of aE (also for a run whose
// input has no entry = the evaluator without the SupportGrip pass: the round 6 reference).
struct Steady
{
    bool ok = true;
    float maxErr = 0, maxRot = 0, maxReach = 0, minElbow = 180, maxGap = 0, minGap = 1e9f, maxOff = 0, minW = 1, maxTwist = 0;
    float maxShift = 0, maxNeed = 0, minElbowR = 180;
    motionpose::FrameInfo info;
};
bool RunSteady(const Data& aD, float aFem, const sgrip::Entry& aE, const motionpose::OverlayFrameInputs& aIn, int aHz, Steady& aS)
{
    std::string error;
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    if (!ev->Init(aD.db.get(), aD.ov.get(), &aD.style, 2, error))
        return false;
    if (aFem >= 0.0f)
        ev->SetFeminineShoulders(aFem);
    motion::MatchResult match;
    match.clip = aD.idle;
    std::vector<BoneTransform> out(aD.db->NumBones());
    for (int k = 0; k < 3 * aHz; ++k)
    {
        match.clipTime = 0.5f + static_cast<float>(k) / static_cast<float>(aHz);
        aS.ok = ev->Evaluate(1.0f / static_cast<float>(aHz), match, aIn, out.data(), static_cast<uint32_t>(out.size()), &aS.info) && aS.ok;
        if (k < 2 * aHz)
            continue;
        const Meas m = Measure(*ev, out, aE);
        aS.maxErr = std::max(aS.maxErr, m.gripErrCm);
        aS.maxRot = std::max(aS.maxRot, m.gripRotDeg);
        aS.maxReach = std::max(aS.maxReach, m.reach);
        aS.minElbow = std::min(aS.minElbow, m.elbowDeg);
        const float gapNow = GapCm(*aD.ov, *ev, out, aE, aD.woman);
        aS.maxGap = std::max(aS.maxGap, std::fabs(gapNow));
        aS.minGap = std::min(aS.minGap, gapNow);
        aS.maxOff = std::max(aS.maxOff, PocketOffAxisCm(*aD.ov, *ev, out, aD.woman));
        aS.minW = std::min(aS.minW, aS.info.supportGripWeight);
        aS.maxTwist = std::max(aS.maxTwist, std::fabs(aS.info.supportTwistAfterDeg));
        aS.maxShift = std::max(aS.maxShift, aS.info.reachShiftM);
        aS.maxNeed = std::max(aS.maxNeed, aS.info.reachShiftNeedM);
        aS.minElbowR = std::min(aS.minElbowR, RightElbowDeg(*ev, out));
    }
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 7)
    {
        std::puts("usage: utpp_supportgrip_test <woman.utppmm> <woman.utppov> <woman style> <man.utppmm> <man.utppov> <man style>");
        return 2;
    }
    Data body[2];
    if (!Load(body[0], argv[1], argv[2], argv[3], true) || !Load(body[1], argv[4], argv[5], argv[6], false))
        return 2;
    std::string error;

    // (1) table
    {
        bool ids = true;
        for (int32_t id = 0; id < recoil_profile::kSlots; ++id)
        {
            const bool want = id >= 12 && id <= 27 && !(id >= 20 && id <= 22) && recoil_profile::Valid(id) &&
                              recoil_profile::kTable[static_cast<size_t>(id)].longGun;
            const sgrip::Entry* e = sgrip::Of(id);
            ids = ids && (e != nullptr) == want;
            if (e)
                ids = ids && std::string_view(e->family) == recoil_profile::kTable[static_cast<size_t>(id)].family;
        }
        Check(ids, "an entry for every long-gun recoil family (12-19, 23-27), none for pistols / retired / unknown ids, families match");
        bool unit = true, stock = true;
        for (const sgrip::Entry& e : sgrip::kTable)
        {
            unit = unit && std::fabs(std::sqrt(math::QuatDot(e.r, e.r)) - 1.0f) < 1e-3f;
            for (const math::Quat& q : e.fingers)
                unit = unit && std::fabs(std::sqrt(math::QuatDot(q, q)) - 1.0f) < 1e-3f;
            stock = stock && (e.stockY == 0.0f || (e.stockY < -0.10f && e.stockY > -0.45f)); // 0 = no mesh measured: stock pass off
        }
        Check(unit, "grip and finger rotations are unit quaternions");
        Check(stock, "every stock contact (mesh-measured) lies 10-45 cm behind the weapon origin; 0 = pass off");
        Check(sgrip::Of(12)->stockY < 0.0f && sgrip::Of(18)->stockY < 0.0f, "Ajax and Saratoga carry a mesh-measured stock contact");
        bool caps = true;
        for (const sgrip::Entry& e : sgrip::kTable)
            caps = caps && e.maxShift == (e.profile == 27 ? 0.10f : e.profile == 13 ? 0.04f : sgrip::kMaxReachShift) &&
                   e.handFirst == (e.profile == 13);
        Check(caps, "reach-shift cap per weapon: Hercules 10 cm, Copperhead 4 cm, others 3 cm; hand priority Copperhead only (2026-11-10)");
        bool fingers = true;
        for (const Data& d : body)
        {
            const sgrip::FingerBones fb = sgrip::FindFingers(d.ov->Skel().n, [&](int i) { return d.ov->BoneName(i); });
            for (int b : fb)
                fingers = fingers && b >= 0;
        }
        Check(fingers, "the 19 grip finger bones resolve on both overlay skeletons");
        const sgrip::Pocket pk[2] = {sgrip::kPocketWoman, sgrip::kPocketMan};
        Check(gik::Len(pk[0].local) > 0.01f && gik::Len(pk[1].local) > 0.01f, "shoulder pocket calibrated for both bodies");
    }

    // Ajax (assault rifle), Saratoga (SMG), Copperhead (assault rifle, other grip), Hercules (reach shift up to its 10 cm cap)
    const int families[4] = {12, 18, 13, 27};
    // (2) steady Ready / Aiming
    for (int b = 0; b < 2; ++b)
    {
        Data& d = body[b];
        const float fems[3] = {0.0f, 0.025f, 0.05f};
        for (int fi = 0; fi < (d.woman ? 3 : 1); ++fi)
            for (int fam : families)
                for (int aimState = 0; aimState < 2; ++aimState)
                    for (int hz : {30, 144})
                        for (const AimVar& av : kAimVars)
                    {
                        if (hz == 144 && (fi != 1 || fam != 12 || av.have))
                            continue; // 144 Hz: default width, Ajax, no aim point
                        const sgrip::Entry* e = sgrip::Of(fam);
                        motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, aimState == 0, aimState == 1, e);
                        SetAim(in, av);
                        Steady st;
                        if (!RunSteady(d, fems[fi], *e, in, hz, st))
                            return 2;
                        const bool ok = st.ok;
                        const motionpose::FrameInfo& info = st.info;
                        const float maxErr = st.maxErr, maxRot = st.maxRot, maxReach = st.maxReach, minElbow = st.minElbow, maxGap = st.maxGap;
                        const float maxOff = st.maxOff, minW = st.minW, maxTwist = st.maxTwist, maxShift = st.maxShift, maxNeed = st.maxNeed;
                        const float minElbowR = st.minElbowR, minGap = st.minGap;
                        std::printf("%s fem %.3f %-10s %-6s %3d Hz aim point %s p %+.0f y %+.0f: grip %.2f cm %.1f deg, weight min %.3f, reach max %.3f, elbow min %.1f deg, "
                                    "|stock gap| max %.2f cm, pocket off axis max %.2f cm, twist max %.1f deg, reach shift %.2f cm (needed %.2f), "
                                    "right elbow min %.1f deg, stock gap min %.2f cm\n",
                                    d.woman ? "woman" : "man", fems[fi], e->family, aimState ? "aim" : "ready", hz, av.have ? "on " : "off",
                                    av.pitchDeg, av.yawDeg, maxErr, maxRot, minW, maxReach,
                                    minElbow, maxGap, maxOff, maxTwist, maxShift * 100.0f, maxNeed * 100.0f, minElbowR, minGap);
                        // round 6: away from neutral pitch the reach / stock needs can exceed the weapon's
                        // cap (Hercules 10 cm, Copperhead 4 cm) and the stock only slides along the barrel. There the
                        // neutral bounds give way to "never worse than the evaluator without the SupportGrip pass at the same pitch" (the
                        // grip bound stays wherever the reach fits the cap); anatomy and continuity stay mandatory.
                        const bool pitched = Pitched(av);
                        Steady st0;
                        if (pitched)
                        {
                            motionpose::OverlayFrameInputs in0 = in;
                            in0.supportGrip = nullptr;
                            if (!RunSteady(d, fems[fi], *e, in0, hz, st0))
                                return 2;
                            std::printf("   without the pass: grip %.2f cm %.1f deg, stock gap min %.2f cm, pocket off axis max %.2f cm\n", st0.maxErr,
                                        st0.maxRot, st0.minGap, st0.maxOff);
                        }
                        const bool capLimited = pitched && maxNeed > e->maxShift + 1e-5f;
                        Check(ok && info.finite && info.layered, Lbl("evaluator runs the Rifle state", av));
                        if (!capLimited)
                            Check(maxErr <= 0.5f && maxRot <= 3.0f && minW >= 0.9f, Lbl("left hand on the game grip of the weapon (<= 0.5 cm, 3 deg)", av));
                        else
                            Check(maxErr <= st0.maxErr && maxRot <= st0.maxRot,
                                  Lbl("reach beyond the cap (pitched aim): grip error and rotation <= without the pass", av));
                        Check(maxReach <= 0.951f && minElbow >= 20.0f, Lbl("left arm inside the soft reach band and bent (>= 20 deg)", av));
                        // the stock pass is an Aim pass (Ready: butt above the shoulder, a pose change - not claimed), for weapons with a
                        // mesh-measured stock contact (stockY 0 = pass off, pose unchanged)
                        if (aimState == 1 && e->stockY < 0.0f && e->handFirst)
                        {
                            // hand priority (Copperhead, option A): no stock pass; the butt may sit behind the
                            // shoulder point, but not further than measured for option A on the skinned mesh (F -4.1 cm / M -6.0 cm with
                            // <= 1.03 cm of right upper arm skin in the butt; gripstock_copperhead_alt.py) + 0.5 cm
                            if (!pitched)
                                Check(minGap >= (d.woman ? -4.6f : -6.5f), Lbl("hand priority aim: butt behind the shoulder point within the option A bound", av));
                            else if (fam == 13) // round 7: mesh-derived (CopperheadPitchedGapBound)
                                Check(minGap >= CopperheadPitchedGapBound(d.woman, fems[fi], av.pitchDeg),
                                      Lbl("hand priority, pitched aim: butt depth within the mesh-checked +-40 depth + 0.5 cm", av));
                            else
                                Check(minGap >= st0.minGap - 0.5f, Lbl("hand priority, pitched aim: butt depth <= without the pass + 0.5 cm", av));
                        }
                        if (aimState == 1 && e->stockY < 0.0f && !e->handFirst)
                        {
                            Check(maxGap <= 0.5f, Lbl("aim: stock on the shoulder pocket along the barrel (|gap| <= 0.5 cm)", av));
                            if (!pitched)
                                Check(maxOff <= 5.0f, Lbl("aim: shoulder pocket within the butt (<= 5 cm off the stock centre line)", av));
                            else if (fam == 12 && av.pitchDeg < 0.0f) // round 7: mesh-derived (AjaxPitchDownOffBound)
                                Check(maxOff <= AjaxPitchDownOffBound(d.woman, fems[fi]),
                                      Lbl("Ajax, aim pitched down: pocket off the stock centre line within the mesh-checked -40 value + 0.5 cm", av));
                            else
                                Check(maxOff <= st0.maxOff + 0.5f, Lbl("pitched aim: pocket off the stock centre line <= without the pass + 0.5 cm", av));
                        }
                        Check(maxTwist <= ov::kSupportHandTwistLimitDeg + 0.05f, Lbl("support-hand twist within the 85 deg limit (both bodies)", av));
                        if (fam == 27) // Hercules: the full grip by the reach shift alone, inside its 10 cm cap, both elbows bent >= 30 deg
                        {
                            if (!pitched)
                                Check(maxShift <= e->maxShift + 1e-5f && maxNeed <= e->maxShift && minElbow >= 30.0f && minElbowR >= 30.0f,
                                      Lbl("Hercules: full grip within the 10 cm reach shift, both elbows bent >= 30 deg", av));
                            else
                                Check(maxShift <= e->maxShift + 1e-5f && minElbow >= 30.0f && minElbowR >= 30.0f,
                                      Lbl("Hercules, pitched aim: reach shift inside the 10 cm cap, both elbows bent >= 30 deg", av));
                        }
                    }
    }

    // (2b) every long-gun family, both bodies, default width, Relaxed / Ready / Aiming: support-hand twist after the route within the 85 deg
    //      limit (the twist-limited weight; Defender's game grip asks 104-119 deg at the overlay's elbow bend), weight > 0 (the route acts)
    for (Data& d : body)
        for (const sgrip::Entry& e : sgrip::kTable)
            for (const AimVar& av : kAimVars)
        {
            float maxTwist = 0, minW = 1, maxNeed = 0, maxShift = 0;
            bool ok = true;
            for (int st = 0; st < 3; ++st)
            {
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                if (!ev->Init(d.db.get(), d.ov.get(), &d.style, 2, error))
                    return 2;
                motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, st == 1, st == 2, &e);
                SetAim(in, av);
                motion::MatchResult match;
                match.clip = d.idle;
                std::vector<BoneTransform> out(d.db->NumBones());
                motionpose::FrameInfo info;
                for (int k = 0; k < 60; ++k)
                {
                    match.clipTime = 0.5f + static_cast<float>(k) / 30.0f;
                    ok = ev->Evaluate(1.0f / 30.0f, match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && ok;
                    if (k < 30)
                        continue;
                    maxTwist = std::max(maxTwist, std::fabs(info.supportTwistAfterDeg));
                    minW = std::min(minW, info.supportGripWeight);
                    maxNeed = std::max(maxNeed, info.reachShiftNeedM);
                    maxShift = std::max(maxShift, info.reachShiftM);
                }
            }
            std::printf("%s %-10s relaxed/ready/aim, aim point %s p %+.0f y %+.0f: twist max %.1f deg, route weight min %.3f, reach shift max %.2f cm (needed %.2f cm)\n",
                        d.woman ? "woman" : "man", e.family, av.have ? "on " : "off", av.pitchDeg, av.yawDeg, maxTwist, minW, maxShift * 100.0f,
                        maxNeed * 100.0f);
            Check(ok && maxTwist <= ov::kSupportHandTwistLimitDeg + 0.05f && minW > 0.25f,
                  Lbl("every family: support-hand twist <= 85 deg on the route, route weight > 0.25", av));
        }

    // (2c) every weapon with a stock contact and the stock pass (stockY < 0, no hand priority: Ajax, Masamune, Saratoga), both bodies, default
    //      width, steady Aim with and without an aim point: the stock stays on the shoulder pocket along the barrel (|gap| <= 0.5 cm, the
    //      stock bound of (2)) - the reach shift may not slide the butt behind the shoulder; the rest of the reach goes to the grip weight
    for (Data& d : body)
        for (const sgrip::Entry& e : sgrip::kTable)
        {
            if (!(e.stockY < 0.0f) || e.handFirst)
                continue;
            for (const AimVar& av : kAimVars)
            {
                auto ev = std::make_unique<motionpose::PoseEvaluator>();
                if (!ev->Init(d.db.get(), d.ov.get(), &d.style, 2, error))
                    return 2;
                motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, false, true, &e);
                SetAim(in, av);
                motion::MatchResult match;
                match.clip = d.idle;
                std::vector<BoneTransform> out(d.db->NumBones());
                motionpose::FrameInfo info;
                bool ok = true;
                float maxGap = 0, minGap = 1e9f, maxGapS = -1e9f, minW = 1, maxErr = 0, maxShift = 0, maxNeed = 0;
                for (int k = 0; k < 90; ++k)
                {
                    match.clipTime = 0.5f + static_cast<float>(k) / 30.0f;
                    ok = ev->Evaluate(1.0f / 30.0f, match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && ok;
                    if (k < 60)
                        continue;
                    const float gapNow = GapCm(*d.ov, *ev, out, e, d.woman);
                    maxGap = std::max(maxGap, std::fabs(gapNow));
                    minGap = std::min(minGap, gapNow);
                    maxGapS = std::max(maxGapS, gapNow);
                    minW = std::min(minW, info.supportGripWeight);
                    maxErr = std::max(maxErr, Measure(*ev, out, e).gripErrCm);
                    maxShift = std::max(maxShift, info.reachShiftM);
                    maxNeed = std::max(maxNeed, info.reachShiftNeedM);
                }
                std::printf("%s %-10s aim, aim point %s p %+.0f y %+.0f: stock gap %.2f..%.2f cm, grip weight min %.3f, grip error max %.2f cm, "
                            "reach shift max %.2f cm (needed %.2f cm)\n",
                            d.woman ? "woman" : "man", e.family, av.have ? "on " : "off", av.pitchDeg, av.yawDeg, minGap, maxGapS, minW, maxErr,
                            maxShift * 100.0f, maxNeed * 100.0f);
                Check(ok && info.finite && maxGap <= 0.5f, Lbl("stock pass weapons, aim: stock on the shoulder pocket (|gap| <= 0.5 cm), reach shift included", av));
            }
        }

    // (2d) round 6: the grip weight degrades monotonically with the aim pitch. Every family, both bodies, default width, steady Aim at
    //      pitch 0, +-10, +-20, +-30, +-40: the route weight never rises as |pitch| grows on either side (resolution of the reach / twist
    //      limited weight: its 10-step bisection, 1/1024).
    for (const Data& d : body)
        for (const sgrip::Entry& e : sgrip::kTable)
        {
            bool mono = true, ok = true;
            char line[256];
            int at = std::snprintf(line, sizeof line, "%s %-10s aim weight by pitch:", d.woman ? "woman" : "man", e.family);
            for (int side : {1, -1})
            {
                float prevW = 2.0f;
                for (int step = 0; step <= 4; ++step)
                {
                    const AimVar av{true, static_cast<float>(side * step * 10), 0.0f};
                    motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, false, true, &e);
                    SetAim(in, av);
                    Steady st;
                    if (!RunSteady(d, -1.0f, e, in, 30, st))
                        return 2;
                    ok = ok && st.ok;
                    mono = mono && st.minW <= prevW + 1.0f / 1024.0f;
                    prevW = st.minW;
                    if (!(side < 0 && step == 0) && at > 0 && at < static_cast<int>(sizeof line))
                        at += std::snprintf(line + at, sizeof line - static_cast<size_t>(at), " %+.0f:%.3f", av.pitchDeg, st.minW);
                }
            }
            std::printf("%s\n", line);
            Check(ok && mono, "pitched aim: the grip weight degrades monotonically from pitch 0 to +-40 (both sides)");
        }

    // (2e) round 6: every family, both bodies, default width, steady Aim at pitch +-40 (yaw 0 and at its clamp): never worse than the
    //      evaluator without the SupportGrip pass at the same pitch - grip error (position, rotation); the butt depth of the hand-priority
    //      weapon (Copperhead) and the pocket's distance from the stock centre line (stock pass weapons) within + 0.5 cm.
    for (const Data& d : body)
        for (const sgrip::Entry& e : sgrip::kTable)
            for (const AimVar& av : kAimVars)
            {
                if (!Pitched(av))
                    continue;
                motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, false, true, &e);
                SetAim(in, av);
                motionpose::OverlayFrameInputs in0 = in;
                in0.supportGrip = nullptr;
                Steady st, st0;
                if (!RunSteady(d, -1.0f, e, in, 30, st) || !RunSteady(d, -1.0f, e, in0, 30, st0))
                    return 2;
                std::printf("PITCH40 %s %-10s p %+.0f y %+.0f | without the pass: grip %.2f cm %.1f deg, stock gap min %.2f cm, pocket off axis %.2f cm "
                            "| with: grip %.2f cm %.1f deg, weight %.3f, stock gap min %.2f cm, pocket off axis %.2f cm, reach shift %.2f (needed %.2f) cm\n",
                            d.woman ? "woman" : "man", e.family, av.pitchDeg, av.yawDeg, st0.maxErr, st0.maxRot, st0.minGap, st0.maxOff, st.maxErr,
                            st.maxRot, st.minW, st.minGap, st.maxOff, st.maxShift * 100.0f, st.maxNeed * 100.0f);
                Check(st.ok && st0.ok && st.maxErr <= st0.maxErr && st.maxRot <= st0.maxRot,
                      Lbl("every family, pitched aim: grip error and rotation <= without the pass", av));
                if (e.stockY < 0.0f && e.handFirst && e.profile == 13) // round 7: mesh-derived (CopperheadPitchedGapBound)
                    Check(st.minGap >= CopperheadPitchedGapBound(d.woman, -1.0f, av.pitchDeg),
                          Lbl("every family, pitched aim: hand-priority butt depth within the mesh-checked +-40 depth + 0.5 cm", av));
                else if (e.stockY < 0.0f && e.handFirst)
                    Check(st.minGap >= st0.minGap - 0.5f, Lbl("every family, pitched aim: hand-priority butt depth <= without the pass + 0.5 cm", av));
                if (e.stockY < 0.0f && !e.handFirst && e.profile == 12 && av.pitchDeg < 0.0f) // round 7: mesh-derived (AjaxPitchDownOffBound)
                    Check(st.maxOff <= AjaxPitchDownOffBound(d.woman, -1.0f),
                          Lbl("every family, Ajax aim pitched down: pocket off the stock centre line within the mesh-checked -40 value + 0.5 cm", av));
                else if (e.stockY < 0.0f && !e.handFirst)
                    Check(st.maxOff <= st0.maxOff + 0.5f, Lbl("every family, pitched aim: pocket off the stock centre line <= without the pass + 0.5 cm", av));
            }

    // (2e) information only (no check): the hand-priority weapon at neutral pitch, with vs without the pass - its butt depth comes from
    //      the reach shift of option A, at every pitch
    for (const Data& d : body)
        for (const sgrip::Entry& e : sgrip::kTable)
        {
            if (!e.handFirst)
                continue;
            motionpose::OverlayFrameInputs in = Inputs(d, ov::PoseOverlay::Rifle, false, true, &e);
            SetAim(in, AimVar{true, 0.0f, 0.0f});
            motionpose::OverlayFrameInputs in0 = in;
            in0.supportGrip = nullptr;
            Steady st, st0;
            if (!RunSteady(d, -1.0f, e, in, 30, st) || !RunSteady(d, -1.0f, e, in0, 30, st0))
                return 2;
            std::printf("PITCH0 %s %-10s p +0 | without the pass: grip %.2f cm, stock gap min %.2f cm | with: grip %.2f cm, stock gap min %.2f cm, "
                        "reach shift %.2f cm\n", d.woman ? "woman" : "man", e.family, st0.maxErr, st0.minGap, st.maxErr, st.minGap, st.maxShift * 100.0f);
        }

    // (3) unchanged outside the long-gun route, A/B against an evaluator that never had an entry, bit for bit:
    //  (a) the Pistol2H overlay with an entry (one the driver never passes);
    //  (b) review round 5: the Rifle route WITH an entry (Ajax: stock pass, Hercules: reach shift up to 10 cm), Ready then Aim, then the
    //      entry goes away (nullptr, e.g. a weapon epoch change): once the route has faded out (weight, reach shift, stock, hand release 0, within
    //      1.5 s), every later frame equals the evaluator without the input bit for bit (no state left over from the route).
    for (const Data& d : body)
    {
        bool same = true;
        {
            auto evA = std::make_unique<motionpose::PoseEvaluator>();
            auto evB = std::make_unique<motionpose::PoseEvaluator>();
            if (!evA->Init(d.db.get(), d.ov.get(), &d.style, 2, error) || !evB->Init(d.db.get(), d.ov.get(), &d.style, 2, error))
                return 2;
            motionpose::OverlayFrameInputs a = Inputs(d, ov::PoseOverlay::Pistol2H, true, false, nullptr);
            motionpose::OverlayFrameInputs bIn = a;
            bIn.supportGrip = sgrip::Of(12); // Pistol2H: an entry the driver would never pass
            motion::MatchResult match;
            match.clip = d.idle;
            std::vector<BoneTransform> oa(d.db->NumBones()), ob(d.db->NumBones());
            for (int k = 0; k < 90; ++k)
            {
                match.clipTime = 0.5f + k / 30.0f;
                if (k == 45)
                    a.machine.wantsToAim = bIn.machine.wantsToAim = true;
                evA->Evaluate(1.0f / 30.0f, match, a, oa.data(), static_cast<uint32_t>(oa.size()));
                evB->Evaluate(1.0f / 30.0f, match, bIn, ob.data(), static_cast<uint32_t>(ob.size()));
                same = same && std::memcmp(oa.data(), ob.data(), sizeof(BoneTransform) * oa.size()) == 0;
            }
        }
        Check(same, d.woman ? "woman: Pistol2H with an entry is bit for bit unchanged" : "man: Pistol2H with an entry is bit for bit unchanged");
        for (int fam : {12, 27})
        {
            const sgrip::Entry* e = sgrip::Of(fam);
            auto evA = std::make_unique<motionpose::PoseEvaluator>(); // with the entry, then without
            auto evB = std::make_unique<motionpose::PoseEvaluator>(); // never an entry
            if (!evA->Init(d.db.get(), d.ov.get(), &d.style, 2, error) || !evB->Init(d.db.get(), d.ov.get(), &d.style, 2, error))
                return 2;
            motionpose::OverlayFrameInputs a = Inputs(d, ov::PoseOverlay::Rifle, true, false, e);
            motionpose::OverlayFrameInputs bIn = Inputs(d, ov::PoseOverlay::Rifle, true, false, nullptr);
            motion::MatchResult match;
            match.clip = d.idle;
            std::vector<BoneTransform> oa(d.db->NumBones()), ob(d.db->NumBones());
            motionpose::FrameInfo infoA;
            constexpr int kDrop = 75, kFrames = 165;
            int faded = -1, compared = 0;
            bool routeRan = false, eq = true;
            for (int k = 0; k < kFrames; ++k)
            {
                match.clipTime = 0.5f + k / 30.0f;
                if (k == 30)
                    a.machine.wantsReady = bIn.machine.wantsReady = false, a.machine.wantsToAim = bIn.machine.wantsToAim = true;
                if (k == kDrop)
                    a.supportGrip = nullptr;
                evA->Evaluate(1.0f / 30.0f, match, a, oa.data(), static_cast<uint32_t>(oa.size()), &infoA);
                evB->Evaluate(1.0f / 30.0f, match, bIn, ob.data(), static_cast<uint32_t>(ob.size()));
                if (k < kDrop)
                    routeRan = routeRan || infoA.supportGripWeight > 0.5f;
                else if (faded < 0 && infoA.supportGripWeight == 0.0f && infoA.reachShiftM == 0.0f && infoA.stockWeight == 0.0f &&
                         infoA.supportReleaseM == 0.0f)
                    faded = k;
                if (faded >= 0)
                {
                    eq = eq && std::memcmp(oa.data(), ob.data(), sizeof(BoneTransform) * oa.size()) == 0;
                    ++compared;
                }
            }
            std::printf("%s %s: route on until frame %d, faded out at frame %d (%.2f s), %d frames compared, bit for bit %s\n",
                        d.woman ? "woman" : "man", e->family, kDrop, faded, faded >= 0 ? (faded - kDrop) / 30.0f : -1.0f, compared,
                        eq && faded >= 0 ? "yes" : "no");
            Check(routeRan && faded >= 0 && faded - kDrop <= 45 && compared >= 30 && eq,
                  "Rifle: after the entry goes away and the route fades out, the pose equals the evaluator that never had an entry");
        }
    }

    // (4) continuity, 30 Hz: state sequence + a one-frame drop-out of the weapon data + holster. A/B against the same evaluator without
    //     the input (= the build before, bit for bit by (3)): per frame, the left hand's motion in the Spine3 frame exceeds the larger of
    //     the hand motion without the input and the motion of the weapon's game grip point without the input (a hand on the handguard
    //     moves with that point) by <= 2.5 cm / frame: the gate runs >= 0.2 s and the hand <= 0.75 m/s (sgrip::GateSeconds), also on the
    //     release / regrip of the hand-back (reload) segments in Ready and Aim.
    for (Data& d : body)
        // + Defender (twist-limited weight), Copperhead (hand priority), Masamune, Hercules (reach shift up to its 10 cm cap)
        for (int fam : {12, 18, 23, 13, 15, 27})
        {
            const sgrip::Entry* e = sgrip::Of(fam);
            std::unique_ptr<motionpose::PoseEvaluator> ev[2] = {std::make_unique<motionpose::PoseEvaluator>(), std::make_unique<motionpose::PoseEvaluator>()};
            for (auto& x : ev)
                if (!x->Init(d.db.get(), d.ov.get(), &d.style, 2, error))
                    return 2;
            struct Seg
            {
                int frames, clip;
                bool ready, aim, moving, sprint, holster, dropout, handback = false;
                int swap = 0; // review round 5: 1 = another weapon's entry (Hercules -> Ajax, Ajax -> Hercules), 2 = the Pistol1H overlay
                bool aimPoint = false; // round 6: an aim point with the pitch ramped p0 -> p1 over the segment (sweep 0 -> +-40 -> 0)
                float p0 = 0.0f, p1 = 0.0f;
            };
            const Seg segs[] = {{30, d.idle, false, false, false, false, false, false}, {30, d.idle, true, false, false, false, false, false},
                                {45, d.idle, false, true, false, false, false, false},  {20, d.idle, true, false, false, false, false, true},
                                {45, d.walk, true, false, true, false, false, false},   {30, d.walk, false, true, true, false, false, false},
                                {30, d.run, false, false, true, false, false, false},   {45, d.sprint, false, false, true, true, false, false},
                                {30, d.idle, true, false, false, false, false, false},
                                // release / regrip: the game's hand-back (reload) = no authored two-hand grip for 1.6 s, then the grip again
                                {48, d.idle, true, false, false, false, false, false, true}, {30, d.idle, true, false, false, false, false, false},
                                {30, d.idle, false, true, false, false, false, false},  {48, d.idle, false, true, false, false, false, false, true},
                                {30, d.idle, false, true, false, false, false, false},
                                // review round 5: a weapon switch / an overlay switch while the gate is falling (6 frames into a hand-back)
                                {6, d.idle, false, true, false, false, false, false, true}, {40, d.idle, false, true, false, false, false, false, false, 1},
                                {45, d.idle, false, true, false, false, false, false},
                                {6, d.idle, false, true, false, false, false, false, true}, {45, d.idle, false, true, false, false, false, false, false, 2},
                                {45, d.idle, false, true, false, false, false, false},
                                // round 6: aim pitch sweep 0 -> +40 -> 0 -> -40 -> 0 at 30 Hz (40 deg per second, 0.5 s hold at the ends)
                                {30, d.idle, false, true, false, false, false, false, false, 0, true, 0.0f, 40.0f},
                                {15, d.idle, false, true, false, false, false, false, false, 0, true, 40.0f, 40.0f},
                                {30, d.idle, false, true, false, false, false, false, false, 0, true, 40.0f, 0.0f},
                                {30, d.idle, false, true, false, false, false, false, false, 0, true, 0.0f, -40.0f},
                                {15, d.idle, false, true, false, false, false, false, false, 0, true, -40.0f, -40.0f},
                                {30, d.idle, false, true, false, false, false, false, false, 0, true, -40.0f, 0.0f},
                                {30, d.idle, true, false, false, false, true, false}};
            const sgrip::Entry* swapEntry = sgrip::Of(fam == 12 ? 27 : 12);
            motion::MatchResult match;
            std::vector<BoneTransform> out(d.db->NumBones());
            motionpose::FrameInfo info;
            static ov::Pose p;
            int prevClip = -1, maxFrame = -1, frame = 0;
            float t = 0, maxExtra = 0, maxStep[2] = {}, maxExtraSwap[3] = {}, maxExtraSweep = 0;
            float sweepW[2] = {1.0f, 1.0f}, sweepStep = 0.0f; // round 6: the sweep really pitches the pose (weight dip at +-40, raw hand step)
            math::Vec3 prev[2]{}, prevGrip{};
            float gripStep = 0.0f;
            bool have = false, ok = true;
            for (const Seg& s : segs)
                for (int k = 0; k < s.frames; ++k, ++frame)
                {
                    match.clip = s.clip;
                    match.jumped = s.clip != prevClip;
                    if (match.jumped && prevClip >= 0)
                    {
                        match.blend.valid = true;
                        match.blend.blendTime = 0.3f;
                        match.blend.fromClip = prevClip;
                        match.blend.toClip = s.clip;
                        match.blend.toTime = 0.0f;
                        t = 0.0f;
                    }
                    else
                        match.blend.valid = false;
                    prevClip = s.clip;
                    t += 1.0f / 30.0f;
                    match.clipTime = std::fmod(t, 1.9f);
                    float step[2] = {};
                    for (int v = 0; v < 2; ++v)
                    {
                        const ov::PoseOverlay o = s.holster ? ov::PoseOverlay::Default : s.swap == 2 ? ov::PoseOverlay::Pistol1H : ov::PoseOverlay::Rifle;
                        motionpose::OverlayFrameInputs in = Inputs(d, o, s.ready, s.aim, v == 0 ? (s.swap == 1 ? swapEntry : e) : nullptr);
                        in.machine.moving = in.machine.hasVelocity = s.moving;
                        if (s.aimPoint)
                            SetAim(in, AimVar{true, s.p0 + (s.p1 - s.p0) * static_cast<float>(k + 1) / static_cast<float>(s.frames), 0.0f});
                        in.machine.wantsToSprint = in.machine.sprinting = s.sprint;
                        if (s.handback)
                            in.authoredTwoHandGrip = false; // MotionDriver: hand-back to the game (reload) = no authored two-hand grip
                        if (s.dropout && k == 5)
                            in.supportGrip = nullptr; // one frame without the weapon's data (e.g. a record re-send)
                        ok = ev[v]->Evaluate(1.0f / 30.0f, match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && ok;
                        ev[v]->ToOv(out.data(), p);
                        math::Quat rs, rh;
                        math::Vec3 ps, ph;
                        int spine3 = -1;
                        for (int i = 0; i < ev[v]->Skel().n; ++i)
                            if (d.ov->BoneName(i) == "Spine3")
                                spine3 = i;
                        gik::ModelOf(ev[v]->Skel(), p, spine3, rs, ps);
                        gik::ModelOf(ev[v]->Skel(), p, ev[v]->GripRig().left.hand, rh, ph);
                        const math::Vec3 rel = math::QuatRotate(math::QuatInv(rs), ph - ps);
                        step[v] = have ? gik::Len(rel - prev[v]) * 100.0f : 0.0f;
                        maxStep[v] = std::max(maxStep[v], step[v]);
                        prev[v] = rel;
                        if (v == 0 && s.aimPoint)
                        {
                            sweepW[s.p0 + s.p1 > 0.0f ? 0 : 1] = std::min(sweepW[s.p0 + s.p1 > 0.0f ? 0 : 1], info.supportGripWeight);
                            sweepStep = std::max(sweepStep, step[0]);
                        }
                        if (v == 1) // the game grip point on the weapon of the run without the input: where a hand on the handguard would go
                        {
                            math::Quat rw;
                            math::Vec3 pw;
                            gik::ModelOf(ev[v]->Skel(), p, ev[v]->GripRig().weaponRight, rw, pw);
                            const math::Vec3 g = math::QuatRotate(math::QuatInv(rs), pw + math::QuatRotate(rw, e->t) - ps);
                            gripStep = have ? gik::Len(g - prevGrip) * 100.0f : 0.0f;
                            prevGrip = g;
                        }
                    }
                    have = true;
                    const float extra = step[0] - std::max(step[1], gripStep);
                    maxExtraSwap[s.swap] = std::max(maxExtraSwap[s.swap], extra);
                    if (s.aimPoint)
                        maxExtraSweep = std::max(maxExtraSweep, extra);
                    if (extra > maxExtra)
                    {
                        maxExtra = extra;
                        maxFrame = frame;
                    }
                }
            std::printf("%s %s continuity: %d frames, left hand step max %.2f cm / frame (without the input %.2f), extra over max(hand, grip point) without it %.2f cm (frame %d; "
                        "in the weapon swap %.2f, in the Pistol1H swap %.2f, in the pitch sweep %.2f)\n",
                        d.woman ? "woman" : "man", e->family, frame, maxStep[0], maxStep[1], maxExtra, maxFrame, maxExtraSwap[1], maxExtraSwap[2], maxExtraSweep);
            std::printf("   pitch sweep: grip weight min %.3f (0 -> +40 -> 0), %.3f (0 -> -40 -> 0), left hand step max %.2f cm / frame\n", sweepW[0],
                        sweepW[1], sweepStep);
            Check(ok && maxExtra <= 2.5f, "continuity: the route adds <= 2.5 cm / frame of left-hand motion (30 Hz)");
        }
    std::printf("SupportGrip %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

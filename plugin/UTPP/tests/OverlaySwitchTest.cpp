// Inertialized overlay switch (mm.overlayInertialSwitch: PoseEvaluator::SetOverlayInertialSwitch, OverlayMachine::instantSwitch).
//   1. machine: off = the crossfades (two pose keys after a weapon class change, style over styleSwitch); on = the keys swap at once,
//      switchedThisFrame is set on that frame only; the one-handed -> two-handed aim fade and a hip raise keep their crossfade.
//   2. real evaluator on both bodies (30 / 60 / 144 Hz, still base pose, grip IK skipped): draw, holster and weapon class changes.
//      Per switch the hands and the weapon bone (chest frame, Spine3) are tracked for 1 s against their settled positions:
//        continuity  - the first frame and the largest frame step stay within 1.5 x those of a pure quintic inertialization of the
//                      switch distance (x(u) = 1 - 10u^3 + 15u^4 - 6u^5; 1.5 = path arc vs chord of the swinging arm) + 2 mm; the
//                      crossfade's linear start can be a one-frame jump (holster at 30 Hz: ~0.2 m), the inertialized switch may not;
//        overshoot   - no point passes its settled position by more than 2 mm along its path, and the distance to it never grows
//                      by more than 1 mm after it has fallen;
//        settle      - every point is within 2 mm of its settled position 0.2 s (+1 frame) after the switch; on the woman body the
//                      feminine passes (shoulders, figure) follow a reach change with their own 0.25 s rise (off and on alike), so
//                      there the 0.2 s bound is checked with both passes off, and the default woman run may not settle later than off;
//        legs        - the toes (model space) blend over the longer leg duration: no frame step larger than the crossfade's, no
//                      overshoot, within 2 mm after kOverlaySwitchSeconds x kOverlaySwitchLegFactor;
//        off         - an option-off run repeats bit for bit (determinism of the off path).
//      A switch back 0.08 s into a blend stays finite and settles within 0.2 s of the second switch.
//   3. grip IK on (authored two-hand grip, rifle support grip and stock), aiming: the gates after the inertialization keep the
//      crossfade weights (authored grip held no fewer frames, stock weight falls no faster than off, first hand step no larger than off,
//      later steps within the quintic peak).
//   4. toggling off: before the first frame or after a settled window = never on, bit for bit; inside a blend the remaining offset
//      snaps (at most the switch distance) and the pose settles on the option-off pose. Both thighs are found for the leg blend.
// Arguments: woman db, woman ov, woman style, man db, man ov, man style.
#include "../src/mm/MotionPose.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp;
using namespace utpp::mm;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    std::printf("%s ovswitch %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
    (aOk ? g_pass : g_fail)++;
}

void MachineChecks()
{
    for (int on = 0; on < 2; ++on)
    {
        ov::OverlayMachine m;
        m.instantSwitch = on != 0;
        ov::OverlayInputs in;
        in.style = ov::Style::Feminine;
        for (int k = 0; k < 30; ++k)
            m.Update(in, 1.0f / 60.0f);
        in.overlay = ov::PoseOverlay::Rifle;
        m.Update(in, 1.0f / 60.0f);
        const std::string tag = on ? "on" : "off";
        Check(on ? (m.poses.count == 1 && m.switchedThisFrame) : (m.poses.count == 2 && !m.switchedThisFrame),
              tag + ": draw swaps the pose keys at once only when on (count " + std::to_string(m.poses.count) + ")");
        m.Update(in, 1.0f / 60.0f);
        Check(!m.switchedThisFrame, tag + ": switchedThisFrame lasts one frame");
        in.style = ov::Style::Masculine;
        m.Update(in, 1.0f / 60.0f);
        Check(on ? (m.styles.count == 1 && m.switchedThisFrame) : (m.styles.count == 2 && !m.switchedThisFrame),
              tag + ": style change swaps at once only when on");
        // one-handed Ready -> two-handed Aim keeps its authored arc (pistol1HTo2HAim) also when on
        ov::OverlayMachine p;
        p.instantSwitch = on != 0;
        ov::OverlayInputs pi;
        pi.overlay = ov::PoseOverlay::Pistol1H;
        pi.wantsReady = true;
        for (int k = 0; k < 60; ++k)
            p.Update(pi, 1.0f / 60.0f);
        pi.overlay = ov::PoseOverlay::Pistol2H;
        pi.wantsToAim = true;
        p.Update(pi, 1.0f / 60.0f);
        Check(p.poses.count >= 2 && !p.switchedThisFrame && p.poses.duration >= p.times.pistol1HTo2HAim - 1e-6f,
              tag + ": 1H Ready -> 2H Aim keeps the crossfade");
        // a hip raise on the draw frame (Relaxed -> Aiming, 0.25 s) keeps the crossfade
        ov::OverlayMachine h;
        h.instantSwitch = on != 0;
        ov::OverlayInputs hi;
        for (int k = 0; k < 30; ++k)
            h.Update(hi, 1.0f / 60.0f);
        hi.overlay = ov::PoseOverlay::Rifle;
        hi.wantsToAim = hi.wantsReady = hi.hipFireRaise = true;
        h.Update(hi, 1.0f / 60.0f);
        Check(h.poses.count >= 2 && !h.switchedThisFrame, tag + ": hip raise on the draw frame keeps the crossfade");
    }
}

struct Body
{
    const char* name;
    bool woman;
    std::unique_ptr<motion::MotionDatabase> db;
    std::unique_ptr<ov::OverlayFile> ovf;
    ov::StyleFamilyData style;
    int idle = -1;
};

bool LoadBody(Body& aB, const char* aDb, const char* aOv, const char* aStyle)
{
    std::string err;
    aB.db = std::make_unique<motion::MotionDatabase>();
    aB.ovf = std::make_unique<ov::OverlayFile>();
    if (!aB.db->LoadFromFile(aDb, aB.woman ? motion::kRigIdentityWoman : motion::kRigIdentityMan, err) ||
        !aB.ovf->LoadFromFile(aOv, aB.woman ? ov::kRigIdentityWomanOv : ov::kRigIdentityManOv, err))
    {
        std::printf("load %s: %s\n", aB.name, err.c_str());
        return false;
    }
    std::ifstream f(aStyle);
    std::stringstream t;
    t << f.rdbuf();
    if (!aB.style.Parse(t.str(), &err))
        return false;
    for (uint32_t i = 0; i < aB.db->NumClips(); ++i)
        if (aB.db->ClipName(i) == "M_Neutral_Stand_Idle_Loop")
            aB.idle = static_cast<int>(i);
    return aB.idle >= 0;
}

// arm points (left hand, right hand, weapon bone) in the chest frame (Spine3), then the toes in model space
constexpr int kArms = 3, kPoints = 5;

struct Sampler
{
    std::unique_ptr<motionpose::PoseEvaluator> ev;
    motionpose::OverlayFrameInputs in;
    motion::MatchResult match;
    std::vector<BoneTransform> out;
    std::unique_ptr<ov::Pose> pose = std::make_unique<ov::Pose>();
    std::vector<math::Vec3> pos;
    std::vector<math::Quat> rot;
    int pt[kPoints]{-1, -1, -1, -1, -1};
    int chest = -1;
    bool finite = true;
    bool grip = false;   // grip IK on: authored two-hand grip like the driver (MayUseAuthoredTwoHandGrip), support grip of a rifle
    int32_t gripProfile = 14; // support grip entry of the Rifle overlay (sgrip::Of; 14 = a stock-contact entry)
    motionpose::FrameInfo info;

    bool Init(Body& aB, bool aOn, bool aNoFeminineShoulders)
    {
        std::string err;
        ev = std::make_unique<motionpose::PoseEvaluator>();
        if (!ev->Init(aB.db.get(), aB.ovf.get(), &aB.style, 2, err))
            return false;
        ev->SetOverlayInertialSwitch(aOn);
        if (aNoFeminineShoulders)
        {
            ev->SetFeminineShoulders(0.0f);
            ev->SetFeminineFigure(false);
        }
        in.bodyWoman = aB.woman;
        in.machine.style = aB.woman ? ov::Style::Feminine : ov::Style::Masculine;
        in.gripSkip = true;
        match.clip = aB.idle;
        match.clipTime = 0.5f;
        out.assign(aB.db->NumBones(), BoneTransform{});
        const ov::Skeleton& sk = ev->Skel();
        pos.assign(static_cast<size_t>(sk.n), math::Vec3{});
        rot.assign(static_cast<size_t>(sk.n), math::Quat{});
        const gik::Rig& rig = ev->GripRig();
        pt[0] = rig.left.hand;
        pt[1] = rig.right.hand;
        pt[2] = rig.weaponRight;
        for (int b = 0; b < sk.n; ++b)
        {
            const auto name = aB.ovf->BoneName(b);
            if (name == "Spine3")
                chest = b;
            else if (name == "LeftToeBase")
                pt[3] = b;
            else if (name == "RightToeBase")
                pt[4] = b;
        }
        // every bone on the chains of the tracked points must come from the output (database bones)
        std::vector<bool> fromDb(static_cast<size_t>(sk.n), false);
        for (uint32_t i = 0; i < aB.db->NumBones(); ++i)
            fromDb[static_cast<size_t>(ev->OvBone(i))] = true;
        for (int p : pt)
            for (int b = p; b >= 0; b = sk.parent[b])
                if (b < 0 || !fromDb[static_cast<size_t>(b)])
                    return false;
        for (int b = chest; b >= 0; b = sk.parent[b])
            if (!fromDb[static_cast<size_t>(b)])
                return false;
        return chest >= 0 && pt[0] >= 0 && pt[1] >= 0 && pt[2] >= 0 && pt[3] >= 0 && pt[4] >= 0;
    }
    void Step(float aDt, math::Vec3 aP[kPoints])
    {
        if (grip)
        {
            const ov::PoseOverlay o = in.machine.overlay;
            in.gripSkip = false;
            in.authoredTwoHandGrip = motionpose::MayUseAuthoredTwoHandGrip(true, false, true, o != ov::PoseOverlay::Default, o);
            in.supportGrip = o == ov::PoseOverlay::Rifle ? sgrip::Of(gripProfile) : nullptr;
        }
        info = motionpose::FrameInfo{};
        finite = ev->Evaluate(aDt, match, in, out.data(), static_cast<uint32_t>(out.size()), &info) && finite;
        const ov::Skeleton& sk = ev->Skel();
        pose->n = sk.n;
        for (int b = 0; b < sk.n; ++b)
            pose->b[b] = ov::Xf{};
        for (uint32_t i = 0; i < out.size(); ++i)
        {
            pose->b[ev->OvBone(i)].r = out[i].q;
            pose->b[ev->OvBone(i)].t = out[i].p;
        }
        ov::MeshPositions(sk, *pose, pos.data(), rot.data());
        const math::Quat cInv = math::QuatInv(rot[static_cast<size_t>(chest)]);
        const math::Vec3 cPos = pos[static_cast<size_t>(chest)];
        for (int k = 0; k < kPoints; ++k)
        {
            const math::Vec3 p = pos[static_cast<size_t>(pt[k])];
            aP[k] = k < kArms ? math::QuatRotate(cInv, p - cPos) : p;
        }
    }
};

// first and largest per-frame step of a quintic inertialization from rest (fraction of the switch distance)
void QuinticSteps(int aHz, float aSeconds, float& aFirst, float& aPeak)
{
    const auto x = [](float u) { u = std::min(1.0f, u); return 1.0f - u * u * u * (10.0f - 15.0f * u + 6.0f * u * u); };
    const float du = 1.0f / (static_cast<float>(aHz) * aSeconds);
    aFirst = 1.0f - x(du);
    aPeak = 0.0f;
    for (float u = 0.0f; u < 1.0f; u += du)
        aPeak = std::max(aPeak, x(u) - x(u + du));
}

float Dist(const math::Vec3& a, const math::Vec3& b)
{
    const math::Vec3 d = a - b;
    return std::sqrt(math::Dot(d, d));
}

struct Track
{
    std::vector<std::array<math::Vec3, kPoints>> p; // p[0] = the frame before the switch
    std::vector<std::vector<float>> raw;             // whole output per frame (identity check)
    std::vector<float> stockW;                       // FrameInfo::stockWeight per recorded frame (p[0] included)
    std::vector<char> authored;                      // FrameInfo::gripFromOverlay per recorded frame (p[0] included)
};

struct RunOpts
{
    bool grip = false;       // grip IK on (Sampler::grip)
    bool aim = false;        // aiming with a live target (aim offset on) until a switch to Default
    bool offAtInit = false;  // built with the option on, turned off before the first frame
    int toggleOffAt = -1;    // recorded frame at which the option is turned off (before that frame's switch), -1 = never
};

// aSwitches: (frame index after settling, overlay) pairs; aFrames frames are recorded from the frame before the first switch
Track Run(Body& aB, bool aOn, int aHz, ov::PoseOverlay aFrom, const std::vector<std::pair<int, ov::PoseOverlay>>& aSwitches, int aFrames,
          bool aReady, bool& aOk, bool aNoFemShoulders = false, const RunOpts& aOpt = {})
{
    Track t;
    Sampler s;
    if (!s.Init(aB, aOn, aNoFemShoulders))
    {
        aOk = false;
        return t;
    }
    if (aOpt.offAtInit)
        s.ev->SetOverlayInertialSwitch(false);
    s.grip = aOpt.grip;
    const float dt = 1.0f / static_cast<float>(aHz);
    s.in.machine.overlay = aFrom;
    s.in.machine.wantsReady = aReady;
    if (aOpt.aim)
    {
        s.in.machine.wantsToAim = s.in.machine.wantsReady = true;
        s.in.haveAim = true;
        s.in.aimYawDeg = 25.0f;
        s.in.aimPitchDeg = 15.0f;
    }
    math::Vec3 p[kPoints];
    for (int k = 0; k < 2 * aHz; ++k)
        s.Step(dt, p);
    t.p.push_back({p[0], p[1], p[2], p[3], p[4]});
    t.stockW.push_back(s.info.stockWeight);
    t.authored.push_back(s.info.gripFromOverlay ? 1 : 0);
    for (int f = 0; f < aFrames; ++f)
    {
        if (f == aOpt.toggleOffAt)
            s.ev->SetOverlayInertialSwitch(false);
        for (const auto& sw : aSwitches)
            if (sw.first == f)
            {
                s.in.machine.overlay = sw.second;
                if (aOpt.aim && sw.second == ov::PoseOverlay::Default)
                    s.in.machine.wantsToAim = s.in.machine.wantsReady = s.in.haveAim = false; // holster ends the aim
            }
        s.Step(dt, p);
        t.p.push_back({p[0], p[1], p[2], p[3], p[4]});
        t.stockW.push_back(s.info.stockWeight);
        t.authored.push_back(s.info.gripFromOverlay ? 1 : 0);
        std::vector<float> r(s.out.size() * 7);
        std::memcpy(r.data(), s.out.data(), sizeof(float) * r.size());
        t.raw.push_back(std::move(r));
    }
    aOk = aOk && s.finite;
    return t;
}

struct Metrics
{
    float d0 = 0.0f, firstStep = 0.0f, maxStep = 0.0f, overshoot = 0.0f, rebound = 0.0f, settleS = 1e9f, atTarget = 0.0f;
};

// aFrom: index in the track of the frame before the (last) switch; points [aK0, aK1)
Metrics Measure(const Track& aT, size_t aFrom, int aHz, float aTargetS, int aK0 = 0, int aK1 = kArms)
{
    Metrics m;
    const auto& fin = aT.p.back();
    const size_t target = aFrom + static_cast<size_t>(std::lround(aTargetS * aHz)) + 1;
    for (int k = aK0; k < aK1; ++k)
    {
        const math::Vec3 start = aT.p[aFrom][k];
        const float d0 = Dist(start, fin[k]);
        m.d0 = std::max(m.d0, d0);
        m.firstStep = std::max(m.firstStep, Dist(aT.p[aFrom + 1][k], start));
        math::Vec3 dir = start - fin[k];
        const float len = std::sqrt(math::Dot(dir, dir));
        dir = len > 1e-6f ? dir * (1.0f / len) : math::Vec3{};
        float minD = d0;
        for (size_t f = aFrom + 1; f < aT.p.size(); ++f)
        {
            const math::Vec3 off = aT.p[f][k] - fin[k];
            if (len > 1e-6f)
                m.overshoot = std::max(m.overshoot, -math::Dot(off, dir));
            m.maxStep = std::max(m.maxStep, Dist(aT.p[f][k], aT.p[f - 1][k]));
            const float d = std::sqrt(math::Dot(off, off));
            m.rebound = std::max(m.rebound, d - minD);
            minD = std::min(minD, d);
            if (f == target)
                m.atTarget = std::max(m.atTarget, d);
        }
    }
    // settle: first time after which every point stays within 2 mm
    for (size_t f = aT.p.size() - 1; f > aFrom; --f)
    {
        bool in = true;
        for (int k = aK0; k < aK1; ++k)
            in = in && Dist(aT.p[f][k], fin[k]) <= 0.002f;
        if (!in)
        {
            m.settleS = static_cast<float>(f + 1 - aFrom) / static_cast<float>(aHz);
            break;
        }
        if (f == aFrom + 1)
            m.settleS = 1.0f / static_cast<float>(aHz);
    }
    return m;
}

void EvaluatorChecks(Body& aB)
{
    struct Case
    {
        const char* name;
        ov::PoseOverlay from, to;
        bool ready;
    };
    const Case cases[] = {{"draw rifle", ov::PoseOverlay::Default, ov::PoseOverlay::Rifle, false},
                          {"holster rifle", ov::PoseOverlay::Rifle, ov::PoseOverlay::Default, false},
                          {"draw pistol", ov::PoseOverlay::Default, ov::PoseOverlay::Pistol1H, false},
                          {"pistol2h -> rifle ready", ov::PoseOverlay::Pistol2H, ov::PoseOverlay::Rifle, true},
                          {"rifle ready -> pistol1h", ov::PoseOverlay::Rifle, ov::PoseOverlay::Pistol1H, true}};
    for (int fem = 0; fem < (aB.woman ? 2 : 1); ++fem)
    for (int hz : {30, 60, 144})
        for (const Case& c : cases)
        {
            const bool noFem = fem == 1;
            bool ok = true;
            const std::vector<std::pair<int, ov::PoseOverlay>> sw = {{0, c.to}};
            const Track off = Run(aB, false, hz, c.from, sw, hz, c.ready, ok, noFem);
            const Track on = Run(aB, true, hz, c.from, sw, hz, c.ready, ok, noFem);
            const std::string name = std::string(aB.name) + (noFem ? "-nofemshoulder" : "");
            const std::string tag = name + " " + std::to_string(hz) + "Hz " + c.name;
            Check(ok, tag + ": evaluator init / finite");
            if (!ok)
                continue;
            const Metrics mo = Measure(off, 0, hz, motionpose::kOverlaySwitchSeconds), mn = Measure(on, 0, hz, motionpose::kOverlaySwitchSeconds);
            std::printf("OVSWITCH %s %dHz %-24s d0 %.1f mm | off: step %.2f max %.2f over %.2f rebound %.2f settle %.3f s | on: step %.2f "
                        "max %.2f over %.2f rebound %.2f settle %.3f s at0.2 %.2f mm\n",
                        name.c_str(), hz, c.name, mo.d0 * 1000, mo.firstStep * 1000, mo.maxStep * 1000, mo.overshoot * 1000,
                        mo.rebound * 1000, mo.settleS, mn.firstStep * 1000, mn.maxStep * 1000, mn.overshoot * 1000, mn.rebound * 1000,
                        mn.settleS, mn.atTarget * 1000);
            float qFirst = 0.0f, qPeak = 0.0f;
            QuinticSteps(hz, motionpose::kOverlaySwitchSeconds, qFirst, qPeak);
            Check(mn.firstStep <= 1.5f * qFirst * mo.d0 + 0.002f && mn.maxStep <= 1.5f * qPeak * mo.d0 + 0.002f,
                  tag + ": frame steps of a quintic blend (first " + std::to_string(mn.firstStep * 1000) + " mm, peak " +
                      std::to_string(mn.maxStep * 1000) + " mm)");
            Check(mn.overshoot <= 0.002f && mn.rebound <= 0.001f, tag + ": no overshoot");
            if (aB.woman && !noFem)
                Check(mn.settleS <= mo.settleS + 1.0f / hz + 1e-4f, tag + ": settles no later than the crossfade");
            else
                Check(mn.atTarget <= 0.002f, tag + ": settled 0.2 s after the switch");
            // legs (toes in model space): the longer leg blend moves no faster per frame than the crossfade, does not overshoot and
            // settles within the leg duration
            const float legS = motionpose::kOverlaySwitchSeconds * motionpose::kOverlaySwitchLegFactor;
            const Metrics lo = Measure(off, 0, hz, legS, kArms, kPoints), ln = Measure(on, 0, hz, legS, kArms, kPoints);
            std::printf("OVSWITCH %s %dHz %-24s toes d0 %.1f mm | off: max %.2f settle %.3f s | on: max %.2f over %.2f rebound %.2f "
                        "settle %.3f s at%.1f %.2f mm\n",
                        name.c_str(), hz, c.name, lo.d0 * 1000, lo.maxStep * 1000, lo.settleS, ln.maxStep * 1000, ln.overshoot * 1000,
                        ln.rebound * 1000, ln.settleS, legS, ln.atTarget * 1000);
            Check(ln.maxStep <= lo.maxStep + 0.001f, tag + ": toe frame steps no larger than the crossfade (" +
                                                         std::to_string(ln.maxStep * 1000) + " vs " + std::to_string(lo.maxStep * 1000) + " mm)");
            Check(ln.overshoot <= 0.002f && ln.rebound <= 0.001f, tag + ": toes do not overshoot");
            Check(ln.atTarget <= 0.002f, tag + ": toes settled after the leg blend");
            // off matches an evaluator that never had the option (both are built off here; the on run must differ when d0 > 1 cm)
            bool same = true;
            const Track ref = Run(aB, false, hz, c.from, sw, hz, c.ready, ok, noFem);
            for (size_t f = 0; f < ref.raw.size() && same; ++f)
                same = std::memcmp(ref.raw[f].data(), off.raw[f].data(), sizeof(float) * ref.raw[f].size()) == 0;
            Check(same, tag + ": off run repeats bit for bit");
        }
    // switch back 0.08 s into a running blend
    for (int hz : {30, 60, 144})
    {
        bool ok = true;
        const int back = static_cast<int>(std::lround(0.08f * hz));
        const std::vector<std::pair<int, ov::PoseOverlay>> sw = {{0, ov::PoseOverlay::Rifle}, {back, ov::PoseOverlay::Default}};
        const Track on = Run(aB, true, hz, ov::PoseOverlay::Default, sw, hz, false, ok, aB.woman);
        const std::string tag = std::string(aB.name) + " " + std::to_string(hz) + "Hz draw + holster at 0.08 s";
        Check(ok, tag + ": finite");
        if (!ok)
            continue;
        const Metrics m = Measure(on, static_cast<size_t>(back), hz, motionpose::kOverlaySwitchSeconds);
        std::printf("OVSWITCH %s %dHz %-24s on: step %.2f over %.2f rebound %.2f settle %.3f s at0.2 %.2f mm\n", aB.name, hz, "re-switch",
                    m.firstStep * 1000, m.overshoot * 1000, m.rebound * 1000, m.settleS, m.atTarget * 1000);
        Check(m.atTarget <= 0.002f, tag + ": settled 0.2 s after the second switch");
    }
}
// Grip IK on (authored two-hand grip, support grip and stock of a rifle entry), aiming with a live target: the gates after the
// inertialization read the crossfade weights, so with the option on the authored grip is held no fewer frames, the stock weight
// falls no faster per frame than with it off, the first hand step is no larger than off and later steps stay within the quintic peak.
void GripChecks(Body& aB)
{
    struct Case
    {
        const char* name;
        ov::PoseOverlay from, to;
        bool aim;
    };
    const Case cases[] = {{"aim rifle -> pistol1h", ov::PoseOverlay::Rifle, ov::PoseOverlay::Pistol1H, true},
                          {"aim rifle -> holster", ov::PoseOverlay::Rifle, ov::PoseOverlay::Default, true},
                          {"aim pistol2h -> pistol1h", ov::PoseOverlay::Pistol2H, ov::PoseOverlay::Pistol1H, true},
                          {"ready rifle -> holster", ov::PoseOverlay::Rifle, ov::PoseOverlay::Default, false}};
    for (int hz : {30, 60, 144})
        for (const Case& c : cases)
        {
            bool ok = true;
            RunOpts o;
            o.grip = true;
            o.aim = c.aim;
            const std::vector<std::pair<int, ov::PoseOverlay>> sw = {{0, c.to}};
            const Track off = Run(aB, false, hz, c.from, sw, hz, !c.aim, ok, false, o);
            const Track on = Run(aB, true, hz, c.from, sw, hz, !c.aim, ok, false, o);
            const std::string tag = std::string(aB.name) + " grip " + std::to_string(hz) + "Hz " + c.name;
            Check(ok, tag + ": evaluator init / finite");
            if (!ok)
                continue;
            int heldOff = 0, heldOn = 0;
            float dropOff = 0.0f, dropOn = 0.0f, stockMax = 0.0f;
            for (size_t f = 1; f < off.authored.size(); ++f)
            {
                heldOff += off.authored[f];
                heldOn += on.authored[f];
                dropOff = std::max(dropOff, off.stockW[f - 1] - off.stockW[f]);
                dropOn = std::max(dropOn, on.stockW[f - 1] - on.stockW[f]);
                stockMax = std::max(stockMax, off.stockW[f - 1]);
            }
            // hands only (the weapon bone of the holstered pose is not a hand path)
            const Metrics mo = Measure(off, 0, hz, motionpose::kOverlaySwitchSeconds, 0, 2),
                          mn = Measure(on, 0, hz, motionpose::kOverlaySwitchSeconds, 0, 2);
            float qFirst = 0.0f, qPeak = 0.0f;
            QuinticSteps(hz, motionpose::kOverlaySwitchSeconds, qFirst, qPeak);
            std::printf("OVSWITCH %s grip %dHz %-26s d0 %.1f mm | authored frames off %d on %d | stock max %.3f, drop/frame off %.3f on %.3f | "
                        "hands off: step %.2f max %.2f | on: step %.2f max %.2f (bound %.2f / %.2f) mm\n",
                        aB.name, hz, c.name, mo.d0 * 1000, heldOff, heldOn, stockMax, dropOff, dropOn, mo.firstStep * 1000,
                        mo.maxStep * 1000, mn.firstStep * 1000, mn.maxStep * 1000, (1.5f * qFirst * mo.d0 + 0.002f) * 1000,
                        (1.5f * qPeak * mo.d0 + 0.002f) * 1000);
            Check(heldOn >= heldOff, tag + ": authored grip held no fewer frames than off (" + std::to_string(heldOn) + " vs " +
                                         std::to_string(heldOff) + ")");
            Check(dropOn <= dropOff + 1e-3f, tag + ": stock weight falls no faster than off");
            // the switch frame: the grip routes release / regrip at their hand speed limit in both modes, so the first step is held
            // to the crossfade's first step (not to the pure quintic start); later steps to the quintic peak
            Check(mn.firstStep <= mo.firstStep + 0.001f, tag + ": first hand step with grip IK on no larger than off");
            Check(mn.maxStep <= 1.5f * qPeak * mo.d0 + 0.002f, tag + ": hand frame steps within the quintic peak with grip IK on");
        }
}

// Turning the option off: before the first frame or after a settled window without a switch the evaluator matches one that never
// had it, bit for bit, through a later switch; inside a running blend the remaining offset snaps (bounded by the switch distance)
// and the pose settles onto the option-off pose.
void ToggleChecks(Body& aB)
{
    for (int hz : {30, 60, 144})
    {
        const std::string tag = std::string(aB.name) + " " + std::to_string(hz) + "Hz toggle";
        bool ok = true;
        const int later = hz / 4;
        const std::vector<std::pair<int, ov::PoseOverlay>> sw0 = {{0, ov::PoseOverlay::Rifle}}, swLate = {{later, ov::PoseOverlay::Rifle}};
        const Track never0 = Run(aB, false, hz, ov::PoseOverlay::Default, sw0, hz, false, ok);
        RunOpts atInit;
        atInit.offAtInit = true;
        const Track init0 = Run(aB, true, hz, ov::PoseOverlay::Default, sw0, hz, false, ok, false, atInit);
        const Track neverLate = Run(aB, false, hz, ov::PoseOverlay::Default, swLate, hz, false, ok);
        RunOpts settled;
        settled.toggleOffAt = 0;
        const Track settledOff = Run(aB, true, hz, ov::PoseOverlay::Default, swLate, hz, false, ok, false, settled);
        const int mid = static_cast<int>(std::lround(0.08f * hz));
        RunOpts midBlend;
        midBlend.toggleOffAt = mid;
        const Track midOff = Run(aB, true, hz, ov::PoseOverlay::Default, sw0, hz, false, ok, false, midBlend);
        Check(ok, tag + ": evaluator init / finite");
        if (!ok)
            continue;
        const auto same = [](const Track& a, const Track& b) {
            if (a.raw.size() != b.raw.size())
                return false;
            for (size_t f = 0; f < a.raw.size(); ++f)
                if (std::memcmp(a.raw[f].data(), b.raw[f].data(), sizeof(float) * a.raw[f].size()) != 0)
                    return false;
            return true;
        };
        Check(same(never0, init0), tag + ": on then off before the first frame = never on, bit for bit");
        Check(same(neverLate, settledOff), tag + ": on for a settled window, then off = never on, bit for bit through the switch");
        // mid-blend: the step on the toggle frame (raw track index mid + 1) and the end pose against the option-off run
        float snap = 0.0f, endGap = 0.0f, d0 = 0.0f;
        const size_t fm = static_cast<size_t>(mid) + 1;
        for (int k = 0; k < kPoints; ++k)
        {
            snap = std::max(snap, Dist(midOff.p[fm][k], midOff.p[fm - 1][k]));
            endGap = std::max(endGap, Dist(midOff.p.back()[k], never0.p.back()[k]));
            d0 = std::max(d0, Dist(never0.p[0][k], never0.p.back()[k]));
        }
        std::printf("OVSWITCH %s %dHz toggle off at 0.08 s: snap %.2f mm (switch distance %.1f mm), end gap to off %.3f mm\n", aB.name, hz,
                    snap * 1000, d0 * 1000, endGap * 1000);
        Check(snap <= d0 + 0.002f, tag + ": off inside a blend snaps by at most the switch distance");
        Check(endGap <= 0.002f, tag + ": off inside a blend settles on the option-off pose");
    }
}
} // namespace

int main(int argc, char** argv)
{
    MachineChecks();
    if (argc != 7)
    {
        std::printf("usage: woman db ov style, man db ov style\n");
        return 2;
    }
    Body bodies[2] = {{"woman", true, nullptr, nullptr, {}, -1}, {"man", false, nullptr, nullptr, {}, -1}};
    for (int b = 0; b < 2; ++b)
    {
        const bool loaded = LoadBody(bodies[b], argv[1 + 3 * b], argv[2 + 3 * b], argv[3 + 3 * b]);
        Check(loaded, std::string(bodies[b].name) + ": data loads");
        if (loaded)
        {
            Sampler legs;
            Check(legs.Init(bodies[b], true, false) && legs.ev->OverlaySwitchLegRoots() == 2,
                  std::string(bodies[b].name) + ": both thighs found for the leg blend");
            EvaluatorChecks(bodies[b]);
            GripChecks(bodies[b]);
            ToggleChecks(bodies[b]);
        }
    }
    std::printf("OverlaySwitch %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

// P1 B2 (fix batch 2, 2026-10-21 12:15 recording): the armed hips dip / pop on a stance change. ComposeLayers builds the legs / pelvis regions as
// inner = OP + (L - B) weighted by c[Layering_Legs / Pelvis] (0.92 -> 0). B moves between the stand and crouch base poses over stanceSwitch (1.0 s);
// the overlay poses (and the layer curves) used to fade over poseSwitch 0.2 s, and EVERY pose key change (stance, move, aim) restarts that fade from
// frozen weights - so for 0.2 s (OP - B) was the whole stand / crouch pose difference: with a Rifle / Pistol2H the hips dipped 12 cm at +0.08 s and
// rose 9.5 cm (10 of 10 armed crouches), leaving the crouch 36 - 41 cm, and a move / aim change inside the 1 s of the base fade brought the pop back
// (4 - 23 cm). B1 (the whole pose crossfade on stanceSwitch) fixed the isolated stance change but slowed the hands / weapon to the Aiming pose to
// 0.8 - 0.97 s when the stance changed during a hipfire / ADS start. B2: the legs / pelvis regions take their overlay pose and layer weights from
// OverlayMachine::LowerBodyKeys (the stance alpha of the base poses, whatever else changes); hands, arms, spine, head keep the pose crossfade.
//
// Usage: utpp_p1_overlay_stance_sync_test <utpp_overlays_woman.utppov> [--report]
// The locomotion pose L is synthetic: Pose_Neutral_Stand_Idle -> Pose_Neutral_Crouch_Idle blended with the hips curve of the motion clip
// M_Neutral_Transition_Stand_to_Crouch (hips 0.990 / 0.974 / 0.938 / 0.875 / 0.759 / 0.577 / 0.474 / 0.450 m at ct 0 .. 0.7 s) from the frame the
// stance input flips. (out - L) of the hips does not depend on L (inner - L = OP - B), so any L gives the same excursion.
// The composed hips may leave the corridor of the steady offsets of the visited configurations (an Aim / Ready pose carries its own hips offset
// from L, up to 6 cm, that fades with the layer curves) by at most 0.5 cm, also when a move / aim event follows the stance change inside the 1 s of
// the base fade. Guard (P1.md T2): the Aiming pose of the hands reaches 0.9 not later than the aim fade (0.2 s) + the stance change offset + 0.1 s.
// Mutations (-DUTPP_P1_MUTATION=k) MUST fail: 2 = no separate stance alpha (the pre-P1 pop), 5 = the retired B1 (slow hands, broken sync).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/OverlayFile.hpp"
#include "TestFailFast.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

namespace
{
int g_fail = 0, g_checks = 0;
bool g_report = false;
void Check(bool aOk, const std::string& aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
        ++g_fail;
    if (!aOk || g_report)
        std::printf("%s %s (%.4f, %.4f)\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aA, aB);
    if (!aOk)
    {
        char b[384];
        std::snprintf(b, sizeof b, " (%.4f, %.4f)", aA, aB);
        UtppFailFastNote(std::string("FAIL ") + aWhat + b);
    }
}

constexpr float kDt = 1.0f / 60.0f;

// hips (m) of M_Neutral_Transition_Stand_to_Crouch (ct 0 .. 0.7 s step 0.1), the Crouch_Idle value after
constexpr float kClipHips[8] = {0.990f, 0.974f, 0.938f, 0.875f, 0.759f, 0.577f, 0.474f, 0.450f};
float EntryAlpha(float aT)
{
    const float u = std::max(0.0f, aT) * 10.0f;
    const int i = std::min(6, static_cast<int>(u));
    const float h = u >= 7.0f ? kClipHips[7] : kClipHips[i] + (kClipHips[i + 1] - kClipHips[i]) * (u - float(i));
    return Clamp01((kClipHips[0] - h) / (kClipHips[0] - kClipHips[7]));
}
float ExitAlpha(float aT)
{
    const float u = Clamp01(aT / 0.45f);
    return 1.0f - u * u * (3.0f - 2.0f * u);
}

struct Rig
{
    OverlayFile f;
    std::unique_ptr<Pose> base[4];
    std::unique_ptr<LayerScratch> scratch = std::make_unique<LayerScratch>();
    int hips = -1;

    bool Load(const char* aPath, std::string& aErr)
    {
        if (!f.LoadFromFile(aPath, kRigIdentityWomanOv, aErr))
            return false;
        static const char* kBase[4] = {"Pose_Neutral_Stand_Idle", "Pose_Neutral_Stand_Move", "Pose_Neutral_Crouch_Idle", "Pose_Neutral_Crouch_Move"};
        for (int k = 0; k < 4; ++k)
        {
            int p = f.FindPose(kBase[k]);
            if (p < 0 && k >= 2)
                p = f.FindPose("Pose_Neutral_Crouch");
            if (p < 0)
            {
                aErr = kBase[k];
                return false;
            }
            base[k] = std::make_unique<Pose>();
            f.FramePose(f.PoseAt(p).firstFrame, *base[k]);
        }
        for (int i = 0; i < f.NumBones(); ++i)
            if (f.BoneName(i) == "Hips")
                hips = i;
        return hips >= 0;
    }
    float HipsZ(const Pose& aPose) const
    {
        Vec3 pos[kMaxBones];
        Quat rot[kMaxBones];
        MeshPositions(f.Skel(), aPose, pos, rot);
        return pos[hips].z;
    }
};

struct Frame
{
    float hipsFinal = 0, hipsL = 0;
    float lowerCrouchW = 0, lowerTotalW = 0; // weights of the stance-split keys (crouch bit / all)
    float baseCrouchW = 0;                   // crouch share of the base poses B
    float aimingPoseW = 0;                   // weight of the Aiming-state pose keys of the machine (hands, arms: the pose crossfade)
    float legsW = 0, pelvisW = 0;            // Layering_Legs / Layering_Pelvis after the MotionPose chain (B2 keys, idle pistol exception, air fade)
    bool identity = true;                    // output == L bit for bit (Default overlay)
    float upperMeshDiff = 0;                 // arms / hands / head: mesh rotation difference to the compose without the split (deg)
};

// One frame of PoseEvaluator (MotionPose.hpp 404-421, BuildLayerInputs 808-900): the machine, B from bases, OP and the curves from poses, the
// legs / pelvis pose and curves from OverlayMachine::LowerBodyKeys. aCheckUpper: also compose without the split and compare the upper body.
Frame Step(Rig& aRig, OverlayMachine& aM, const OverlayInputs& aIn, const Pose& aL, bool aCheckUpper = false, float aAirW = 0.0f)
{
    aM.Update(aIn, kDt);
    const OverlayFile& f = aRig.f;
    const Pose* bp[4];
    float bw[4];
    int nb = 0;
    Frame out;
    for (int i = 0; i < aM.bases.count; ++i)
    {
        bp[nb] = aRig.base[aM.bases.key[i] & 3].get();
        bw[nb++] = aM.bases.w[i];
        if (aM.bases.key[i] & 2)
            out.baseCrouchW += aM.bases.w[i];
    }
    auto B = std::make_unique<Pose>();
    if (nb == 0)
        *B = *aRig.base[0];
    else
        BlendPosesWeighted(bp, bw, nb, *B);
    Curves cur;
    static Pose slots[7], lowSlots[13];
    const Pose* op[7];
    float ow[7];
    int no = 0;
    float wNone = 0.0f;
    for (int i = 0; i < aM.poses.count; ++i)
    {
        const int key = aM.poses.key[i];
        PoseOverlay ko;
        AimState ka;
        bool kc, km;
        DecodePoseKey(key, ko, ka, kc, km);
        if (ka == AimState::Aiming)
            out.aimingPoseW += aM.poses.w[i];
        const int p = f.FindOverlayPose(ko, ka, kc, km);
        if (p < 0)
        {
            wNone += aM.poses.w[i];
            continue;
        }
        f.FramePose(f.PoseAt(p).firstFrame, slots[no]);
        Curves c;
        f.FrameCurves(p, f.PoseAt(p).firstFrame, c);
        cur.Accumulate(c, aM.poses.w[i]);
        op[no] = &slots[no];
        ow[no++] = aM.poses.w[i];
    }
    auto OP = std::make_unique<Pose>(), OPLower = std::make_unique<Pose>();
    const bool haveOP = no > 0;
    if (haveOP)
    {
        if (wNone > 0.0f)
        {
            slots[no] = aL;
            op[no] = &slots[no];
            ow[no++] = wNone;
        }
        BlendPosesWeighted(op, ow, no, *OP);
        OverlayMachine::LowerKeys lk;
        aM.LowerBodyKeys(lk);
        const Pose* lop[13];
        float low[13];
        int nl = 0;
        float lowNone = 0.0f;
        Curves curLow;
        for (int i = 0; i < lk.count; ++i)
        {
            PoseOverlay ko;
            AimState ka;
            bool kc, km;
            DecodePoseKey(lk.key[i], ko, ka, kc, km);
            out.lowerTotalW += lk.w[i];
            if (kc)
                out.lowerCrouchW += lk.w[i];
            const int p = f.FindOverlayPose(ko, ka, kc, km);
            if (p < 0)
            {
                lowNone += lk.w[i];
                continue;
            }
            f.FramePose(f.PoseAt(p).firstFrame, lowSlots[nl]);
            Curves c;
            f.FrameCurves(p, f.PoseAt(p).firstFrame, c);
            curLow.Accumulate(c, lk.w[i]);
            lop[nl] = &lowSlots[nl];
            low[nl++] = lk.w[i];
        }
        if (lowNone > 0.0f)
        {
            lowSlots[nl] = aL;
            lop[nl] = &lowSlots[nl];
            low[nl++] = lowNone;
        }
        if (nl > 0)
            BlendPosesWeighted(lop, low, nl, *OPLower);
        else
            *OPLower = *OP;
        cur.v[kLegs] = curLow.v[kLegs];
        cur.v[kPelvis] = curLow.v[kPelvis];
        cur.present |= curLow.present;
    }
    // MotionPose.hpp 443-445: the idle pistol exception (female V, Pistol1H Relaxed standing) reads the weight of the idle pose in the B2 keys; the air fade
    PreserveIdlePistolLowerBody(aM, aIn.style == Style::Feminine, cur, aM.LowerIdlePistolWeight());
    FadeLowerBodyInAir(aAirW, cur);
    out.legsW = cur.v[kLegs];
    out.pelvisW = cur.v[kPelvis];
    LayerInputs li;
    li.L = &aL;
    li.B = B.get();
    li.OP = haveOP ? OP.get() : nullptr;
    li.OPLower = haveOP ? OPLower.get() : nullptr;
    li.curves = cur;
    auto lay = std::make_unique<Pose>();
    ComposeLayers(f.Skel(), f.Masks(), li, *aRig.scratch, *lay);
    out.hipsFinal = aRig.HipsZ(*lay);
    out.hipsL = aRig.HipsZ(aL);
    out.identity = std::memcmp(lay->b, aL.b, sizeof(Xf) * size_t(f.NumBones())) == 0;
    if (aCheckUpper && haveOP)
    {
        li.OPLower = nullptr; // the pose crossfade for the legs / pelvis too, with the same curves
        auto plain = std::make_unique<Pose>();
        ComposeLayers(f.Skel(), f.Masks(), li, *aRig.scratch, *plain);
        Quat qa[kMaxBones], qb[kMaxBones];
        MeshRotations(f.Skel(), *lay, qa);
        MeshRotations(f.Skel(), *plain, qb);
        const Regions& rg = f.Masks();
        for (int i = 0; i < f.NumBones(); ++i)
            if (rg.armL.w[i] >= 0.999f || rg.armR.w[i] >= 0.999f || rg.handL.w[i] >= 0.999f || rg.handR.w[i] >= 0.999f || rg.head.w[i] >= 0.999f)
                out.upperMeshDiff = std::max(out.upperMeshDiff, AngleDeg(qa[i], qb[i]));
    }
    return out;
}

// ---- scenarios -------------------------------------------------------------------------------------------------------------
enum EventKind { kMoveOn = 1, kMoveOff, kReadyOn, kReadyOff, kAimOn, kAimOff };
struct Ev
{
    float t;
    int kind;
};
struct Cfg
{
    PoseOverlay ov = PoseOverlay::Rifle;
    AimState aim = AimState::Relaxed;
    bool moving = false;
    bool crouch = false;
};
OverlayInputs InputsOf(const Cfg& c)
{
    OverlayInputs in;
    in.overlay = c.ov;
    in.crouch = c.crouch;
    in.moving = c.moving;
    in.hasVelocity = c.moving;
    in.wantsToAim = c.aim == AimState::Aiming;
    in.wantsReady = c.aim != AimState::Relaxed;
    return in;
}
void Apply(OverlayInputs& in, int kind)
{
    switch (kind)
    {
    case kMoveOn: in.moving = in.hasVelocity = true; break;
    case kMoveOff: in.moving = in.hasVelocity = false; break;
    case kReadyOn: in.wantsReady = true; break;
    case kReadyOff: in.wantsReady = false; break;
    case kAimOn: in.wantsToAim = true; break;
    case kAimOff: in.wantsToAim = false; break;
    }
}
// aLastEvent: the configuration after the events (for the corridor)
Cfg After(Cfg c, const std::vector<Ev>& aEvs)
{
    OverlayInputs in = InputsOf(c);
    for (const Ev& e : aEvs)
        Apply(in, e.kind);
    c.moving = in.moving;
    c.aim = in.wantsToAim ? AimState::Aiming : (in.wantsReady ? AimState::Ready : AimState::Relaxed);
    return c;
}

// steady offset of the composed hips from L: 4 s in a configuration
float SteadyDev(Rig& aRig, const Cfg& aCfg)
{
    OverlayMachine m;
    const OverlayInputs in = InputsOf(aCfg);
    auto L = std::make_unique<Pose>();
    *L = *aRig.base[aCfg.crouch ? 2 : 0];
    Frame last;
    for (int i = 0; i < 300; ++i)
        last = Step(aRig, m, in, *L);
    return last.hipsFinal - last.hipsL;
}

struct Run
{
    std::vector<Frame> frames;
    float excursion = 0, excursionT = 0, maxLowerGap = 0, maxLowerSum = 0, upperDiff = 0;
    float lo = 0, hi = 0, d0 = 0, d1 = 0;
    float aimW90 = -1; // time from the first event of kind aim-on to Aiming pose weight >= 0.9
};

// The stance input flips at aStanceT (s from the start of the observed window, >= 0); aEvs are the other input events (times from the window
// start). The observed window is 2 s. aStart: start configuration (crouch = the state BEFORE the flip).
Run Scenario(Rig& aRig, const Cfg& aStart, float aStanceT, const std::vector<Ev>& aEvs, bool aUpper = false)
{
    OverlayMachine m;
    OverlayInputs in = InputsOf(aStart);
    auto L = std::make_unique<Pose>();
    const bool enter = !aStart.crouch;
    Blend(*aRig.base[0], *aRig.base[2], enter ? 0.0f : 1.0f, *L);
    for (int i = 0; i < 300; ++i) // settle: 5 s in the start configuration
        Step(aRig, m, in, *L);
    Run run;
    Cfg endCfg = After(aStart, aEvs);
    endCfg.crouch = !aStart.crouch;
    Cfg startCfg = aStart;
    run.d0 = SteadyDev(aRig, startCfg);
    run.d1 = SteadyDev(aRig, endCfg);
    Cfg mixStance = After(aStart, {}); // the stance flip only
    mixStance.crouch = !aStart.crouch;
    Cfg mixEvents = After(aStart, aEvs); // the events only
    mixEvents.crouch = aStart.crouch;
    const float dm1 = SteadyDev(aRig, mixStance), dm2 = SteadyDev(aRig, mixEvents);
    run.lo = std::min(std::min(run.d0, run.d1), std::min(dm1, dm2));
    run.hi = std::max(std::max(run.d0, run.d1), std::max(dm1, dm2));
    float stanceAt = aStanceT;
    float firstAim = -1.0f;
    for (const Ev& e : aEvs)
        if (e.kind == kAimOn && firstAim < 0.0f)
            firstAim = e.t;
    const int frames = 120;
    size_t nextEv = 0;
    std::vector<Ev> evs = aEvs;
    std::sort(evs.begin(), evs.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
    bool flipped = false;
    for (int k = 0; k <= frames; ++k)
    {
        const float t = float(k) * kDt;
        while (nextEv < evs.size() && evs[nextEv].t <= t + 1e-4f)
            Apply(in, evs[nextEv++].kind);
        if (!flipped && t + 1e-4f >= stanceAt)
        {
            flipped = true;
            in.crouch = enter;
        }
        const float tf = std::max(0.0f, t - stanceAt);
        Blend(*aRig.base[0], *aRig.base[2], flipped ? (enter ? EntryAlpha(tf) : ExitAlpha(tf)) : (enter ? 0.0f : 1.0f), *L);
        const Frame fr = Step(aRig, m, in, *L, aUpper);
        run.frames.push_back(fr);
        if (t <= 1.6f + 1e-4f)
        {
            const float dev = fr.hipsFinal - fr.hipsL;
            const float out = std::max(run.lo - dev, dev - run.hi);
            if (out > run.excursion)
            {
                run.excursion = out;
                run.excursionT = t;
            }
            run.maxLowerGap = std::max(run.maxLowerGap, std::fabs(fr.lowerCrouchW - fr.lowerTotalW * fr.baseCrouchW));
            run.maxLowerSum = std::max(run.maxLowerSum, std::fabs(fr.lowerTotalW - 1.0f));
            run.upperDiff = std::max(run.upperDiff, fr.upperMeshDiff);
        }
        if (firstAim >= 0.0f && run.aimW90 < 0.0f && t >= firstAim && fr.aimingPoseW >= 0.9f)
            run.aimW90 = t - firstAim;
    }
    return run;
}

const char* AimName(AimState a) { return a == AimState::Relaxed ? "Relaxed" : (a == AimState::Ready ? "Ready" : "Aiming"); }
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_p1_overlay_stance_sync_test <utpp_overlays_woman.utppov> [<unpatched woman overlay>] [--report]\n");
        return 2;
    }
    for (int i = 2; i < argc; ++i)
        if (!std::strcmp(argv[i], "--report"))
            g_report = true;
    auto rig = std::make_unique<Rig>();
    std::string err;
    if (!rig->Load(argv[1], err))
    {
        std::printf("FAIL: overlay file %s: %s\n", argv[1], err.c_str());
        return 1;
    }
    {
        const float hs = rig->HipsZ(*rig->base[0]), hc = rig->HipsZ(*rig->base[2]);
        std::printf("overlay rig: %d bones, hips z stand idle %.3f m, crouch idle %.3f m\n", rig->f.NumBones(), double(hs), double(hc));
        Check(hs - hc > 0.3f, "rig: the Crouch_Idle hips are > 0.3 m below the Stand_Idle hips (z is up)", hs, hc);
    }
    const struct
    {
        PoseOverlay ov;
        const char* name;
    } kOv[] = {{PoseOverlay::Rifle, "Rifle"}, {PoseOverlay::Pistol2H, "Pistol2H"}};
    constexpr float kTol = 0.005f; // 0.5 cm outside the corridor of the steady offsets

    // ---- 1. an isolated stance change, 12 configurations (the pop of the 12:15 recording)
    for (const auto& o : kOv)
        for (const AimState aim : {AimState::Relaxed, AimState::Ready, AimState::Aiming})
            for (const bool enter : {true, false})
            {
                Cfg c;
                c.ov = o.ov;
                c.aim = aim;
                c.crouch = !enter;
                const Run r = Scenario(*rig, c, 0.0f, {});
                char what[220];
                std::snprintf(what, sizeof(what), "%s %s %s: hips stay in the corridor of their steady offsets from L +- 0.5 cm (excursion m, at s)", o.name, AimName(aim),
                              enter ? "stand -> crouch" : "crouch -> stand");
                Check(r.excursion <= kTol, what, r.excursion, r.excursionT);
                if (g_report)
                    std::printf("    steady offset of the hips from L before %.1f cm, after %.1f cm\n", double(r.d0) * 100.0, double(r.d1) * 100.0);
                std::snprintf(what, sizeof(what), "%s %s %s: the legs / pelvis overlay pose has the crouch share of the base poses (max weight gap, weight sum error)", o.name,
                              AimName(aim), enter ? "stand -> crouch" : "crouch -> stand");
                Check(r.maxLowerGap <= 1e-4f && r.maxLowerSum <= 1e-4f, what, r.maxLowerGap, r.maxLowerSum);
            }

    // ---- 2. a move / aim event inside the 1 s of the base fade (the second key change used to bring the pop back: 4 - 23 cm)
    struct Seq
    {
        const char* name;
        AimState aim;
        std::vector<Ev> evs;
    };
    const Seq seqs[] = {
        {"move after 0.3 s", AimState::Relaxed, {{0.3f, kMoveOn}}},
        {"move after 0.3 s, stop after 0.6 s", AimState::Ready, {{0.3f, kMoveOn}, {0.6f, kMoveOff}}},
        {"move after 0.6 s", AimState::Aiming, {{0.6f, kMoveOn}}},
        {"Relaxed -> Ready after 0.3 s", AimState::Relaxed, {{0.3f, kReadyOn}}},
        {"Ready -> Aiming (ADS) after 0.3 s", AimState::Ready, {{0.3f, kAimOn}}},
        {"hipfire after 0.6 s", AimState::Relaxed, {{0.6f, kAimOn}}},
        {"Aiming -> Ready after 0.3 s", AimState::Aiming, {{0.3f, kAimOff}}},
    };
    for (const auto& o : kOv)
        for (const Seq& sq : seqs)
            for (const bool enter : {true, false})
            {
                Cfg c;
                c.ov = o.ov;
                c.aim = sq.aim;
                c.crouch = !enter;
                const Run r = Scenario(*rig, c, 0.0f, sq.evs, true);
                char what[240];
                std::snprintf(what, sizeof(what), "%s %s, %s: hips stay in the corridor +- 0.5 cm (excursion m, at s)", o.name, enter ? "stand -> crouch" : "crouch -> stand",
                              sq.name);
                Check(r.excursion <= kTol, what, r.excursion, r.excursionT);
                std::snprintf(what, sizeof(what), "%s %s, %s: the legs / pelvis pose keeps the base stance share (max weight gap)", o.name, enter ? "stand -> crouch" : "crouch -> stand",
                              sq.name);
                Check(r.maxLowerGap <= 1e-4f && r.maxLowerSum <= 1e-4f, what, r.maxLowerGap, r.maxLowerSum);
                std::snprintf(what, sizeof(what), "%s %s, %s: arms / hands / head are the pose crossfade, untouched by the split (max mesh rotation diff deg)", o.name,
                              enter ? "stand -> crouch" : "crouch -> stand", sq.name);
                Check(r.upperDiff <= 0.01f, what, r.upperDiff);
            }

    // ---- 3. guard (P1.md T2): the hands reach the Aiming pose as fast as without the stance change (+ 0.1 s) when hipfire / ADS and the stance
    //         change coincide; the stance flip comes aTau after the hipfire starts. Reference: the same hipfire with no stance change.
    for (const auto& o : kOv)
        for (const AimState from : {AimState::Ready, AimState::Relaxed})
            for (const bool crouched : {false, true})
            {
                Cfg c;
                c.ov = o.ov;
                c.aim = from;
                c.crouch = crouched;
                const Run ref = Scenario(*rig, c, 99.0f, {{0.0f, kAimOn}}); // the flip never comes inside the window
                for (const float tau : {0.0f, 0.033f, 0.067f, 0.133f, 0.2f, 0.3f})
                {
                    const Run r = Scenario(*rig, c, tau, {{0.0f, kAimOn}});
                    const float limit = ref.aimW90 + 0.1f;
                    char what[260];
                    std::snprintf(what, sizeof(what), "%s %s, hipfire from %s with the stance change %.0f ms later: the Aiming pose reaches 0.9 within the stance-free %.3f s + 0.1 s (s, limit)",
                                  o.name, crouched ? "crouch -> stand" : "stand -> crouch", AimName(from), double(tau) * 1000.0, double(ref.aimW90));
                    Check(ref.aimW90 >= 0.0f && r.aimW90 >= 0.0f && r.aimW90 <= limit + 1e-3f, what, r.aimW90, limit);
                }
            }
    // crouch hipfire without any stance change is the reference
    for (const auto& o : kOv)
    {
        Cfg c;
        c.ov = o.ov;
        c.aim = AimState::Ready;
        c.crouch = true;
        OverlayMachine m;
        OverlayInputs in = InputsOf(c);
        auto L = std::make_unique<Pose>();
        *L = *rig->base[2];
        for (int i = 0; i < 300; ++i)
            Step(*rig, m, in, *L);
        in.wantsToAim = true;
        float t90 = -1.0f;
        for (int k = 0; k <= 60 && t90 < 0.0f; ++k)
        {
            const Frame fr = Step(*rig, m, in, *L);
            if (fr.aimingPoseW >= 0.9f)
                t90 = float(k) * kDt;
        }
        char what[160];
        std::snprintf(what, sizeof(what), "%s crouch hipfire without a stance change: the Aiming pose reaches 0.9 within 0.28 s (s)", o.name);
        Check(t90 >= 0.0f && t90 <= 0.28f, what, t90);
    }

    // ---- B2 x the idle pistol exception (female V, Pistol1H Relaxed, standing still; PreserveIdlePistolLowerBody) and the air fade (FadeLowerBodyInAir)
    //      The exception removes the weapon-idle stand pose from the legs / pelvis weights. Those weights now come from the B2 keys (the idle stand
    //      pose keeps (1 - s) of its weight for the 1.0 s of the base fade), so the exception must read the weight from the same keys: with the pose
    //      crossfade (0.2 s) the weapon-idle legs came back for 0.2 - 0.8 s of every crouch / stand-up (female V, overlay file without the E7E5 patch).
    //      The group weight is 1 (one pose group), the idle pose weight is 1 - s, so legs = ((1 - s) * c_idle + s * c_crouch) * s; air fade scales the legs only.
    //      The exception only has an effect on data whose Pistol1H Relaxed idle pose carries legs / pelvis layering = 1. The shipped woman overlay (E7E5)
    //      has them at 0 (curve patch), so there the exception is redundant and mutation 6 cannot be seen; the block therefore also runs on the
    //      unpatched copy of the same overlay (argv[2], Build/test-data/overlay: the B307 file E7E5 was made from) - the situation of the man overlay.
    auto exceptionBlock = [&](Rig& R, const char* aTag) {
        std::printf("-- B2 x idle pistol exception on: %s\n", aTag);
        const OverlayFile& f = R.f;
        const int pIdle = f.FindOverlayPose(PoseOverlay::Pistol1H, AimState::Relaxed, false, false);
        const int pCrouch = f.FindOverlayPose(PoseOverlay::Pistol1H, AimState::Relaxed, true, false);
        Check(pIdle >= 0 && pCrouch >= 0, "Pistol1H Relaxed idle / crouch poses exist in the overlay file");
        Curves ci, cc;
        if (pIdle >= 0 && pCrouch >= 0)
        {
            f.FrameCurves(pIdle, f.PoseAt(pIdle).firstFrame, ci);
            f.FrameCurves(pCrouch, f.PoseAt(pCrouch).firstFrame, cc);
            if (g_report)
                std::printf("    Pistol1H Relaxed curves: idle legs %.2f pelvis %.2f, crouch legs %.2f pelvis %.2f\n", double(ci[kLegs]), double(ci[kPelvis]), double(cc[kLegs]),
                            double(cc[kPelvis]));
        }
        for (const bool enter : {true, false})
            for (const float air : {0.0f, 0.5f})
            {
                OverlayMachine m;
                OverlayInputs in;
                in.overlay = PoseOverlay::Pistol1H;
                in.style = Style::Feminine;
                in.crouch = !enter;
                auto L = std::make_unique<Pose>();
                *L = *R.base[enter ? 0 : 2];
                for (int i = 0; i < 300; ++i)
                    Step(R, m, in, *L, false, air);
                in.crouch = enter;
                float worst = 0.0f, peakLegs = 0.0f, peakPelvis = 0.0f;
                for (int k = 0; k <= 90; ++k)
                {
                    const Frame fr = Step(R, m, in, *L, false, air);
                    const float sc = fr.baseCrouchW;
                    const float expLegs = ((1.0f - sc) * ci[kLegs] + sc * cc[kLegs]) * sc * (1.0f - air);
                    const float expPelvis = ((1.0f - sc) * ci[kPelvis] + sc * cc[kPelvis]) * sc;
                    worst = std::max(worst, std::max(std::fabs(fr.legsW - expLegs), std::fabs(fr.pelvisW - expPelvis)));
                    peakLegs = std::max(peakLegs, fr.legsW);
                    peakPelvis = std::max(peakPelvis, fr.pelvisW);
                }
                char what[260];
                std::snprintf(what, sizeof(what), "Feminine Pistol1H Relaxed %s, air fade %.1f: legs / pelvis weights = ((1 - s) idle + s crouch) * s, the idle exception follows the B2 keys (max error; peak legs %.2f)",
                              enter ? "stand -> crouch" : "crouch -> stand", double(air), double(peakLegs));
                Check(worst <= 1e-3f, what, worst, peakPelvis);
                if (air == 0.0f)
                {
                    std::snprintf(what, sizeof(what), "Feminine Pistol1H Relaxed %s: the weapon-idle stand pose never gets more than 0.26 of the legs / pelvis weight (peak legs, pelvis)",
                                  enter ? "stand -> crouch" : "crouch -> stand");
                    Check(peakLegs <= 0.26f && peakPelvis <= 0.26f, what, peakLegs, peakPelvis);
                }
            }
    };
    exceptionBlock(*rig, "the overlay under test");
    if (argc > 2 && std::strcmp(argv[2], "--report") != 0)
    {
        auto unpatched = std::make_unique<Rig>();
        if (!unpatched->Load(argv[2], err))
        {
            std::printf("FAIL: unpatched overlay file %s: %s\n", argv[2], err.c_str());
            return 1;
        }
        exceptionBlock(*unpatched, "the unpatched overlay (curve patch absent)");
    }

    // ---- Default overlay: nothing to layer, the result is L bit for bit
    {
        bool identity = true;
        for (const bool enter : {true, false})
        {
            Cfg c;
            c.ov = PoseOverlay::Default;
            c.crouch = !enter;
            const Run r = Scenario(*rig, c, 0.0f, {{0.3f, kMoveOn}});
            for (const Frame& f : r.frames)
                identity = identity && f.identity;
        }
        Check(identity, "Default overlay: the composed pose is L bit for bit through the stance change and a move");
    }
    // ---- the pose crossfade of the machine keeps its own times (hands, arms): a stance change does not stretch it
    {
        OverlayMachine m;
        Cfg c;
        c.aim = AimState::Ready;
        OverlayInputs in = InputsOf(c);
        auto L = std::make_unique<Pose>();
        *L = *rig->base[0];
        for (int i = 0; i < 300; ++i)
            Step(*rig, m, in, *L);
        in.crouch = true;
        Step(*rig, m, in, *L);
        Check(std::fabs(m.poses.duration - m.times.poseSwitch) < 1e-6f, "stance change: the pose crossfade keeps poseSwitch (s)", m.poses.duration, m.times.poseSwitch);
        Check(std::fabs(m.bases.duration - m.times.stanceSwitch) < 1e-6f, "stance change: the base poses take stanceSwitch (s)", m.bases.duration, m.times.stanceSwitch);
    }
    std::printf("p1 overlay stance sync: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

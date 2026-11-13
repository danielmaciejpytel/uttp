// Fix batch 2 (AO) x reload contacts: the aim offset (live during the ADS reload aim hold, latched after a lost target) must not keep
// tilting the MM torso while the stationary pistol reload puts the hands and the weapon on the absolute NATIVE model positions
// (NativeReloadContactsRuntime FinishCore; Spine..Spine3 stay MM). Real motion data (woman + man .utppmm / .utppov / style
// weights), the real PoseEvaluator and the real UpperEnvelope / CorrectionFade, driven by the reload episodes of the user
// recording 2026-10-21 12:15 (tests/data/recording_1215_aim_offset_reload.csv: aim angles and ADS / shoot / reload flags of 53 reloads, 25 of
// them Pistol2H from ADS or within 1 s of a shot; made by tools/analysis/make_ao_reload_fixture.py). The driver logic that feeds the
// AO is replayed here (MotionDriver.cpp Step / FrameInputsLocked): hipfire hold 0.75 s (reset by a reload), ADS reload aim hold
// (UpperEnvelope::AimHold), wantsToAim, aoScale = motionpose::AoReloadScale (the driver calls the same function).
// Usage: utpp_mm_aoreload_test woman.utppmm woman.utppov woman_style.txt man.utppmm man.utppov man_style.txt reload.csv
//
// Three runs on identical machine inputs: NONE (never an AO = the pose the contacts meet once the AO is gone), LATCH (the AO as of
// the latch commit: live while the target exists, latched after, aoScale 1) and NEW (aoScale from the reload envelope).
// "Weapon above the head" AH = max(0, (1 - c) (W_mm - H) + c (W_native - H)) with the MM weapon / head heights of the run, the contact
// weight c of the real CorrectionFade (envelope = the game upper weight) and the native weapon height fixed 0.10 m above the NONE
// head (P3 RC2 steady state: mean +0.03, p90 +0.11 m). OLD = the AO before the latch (live while a target, none after) = LATCH on the
// frames with a target, NONE on the others. Only the part above the head counts.
// Checks: (a) once the game owns the upper body (envelope weight 1) NEW == NONE (<= 0.5 deg), all pistol episodes; (b) reloads at
// pitch <= -20: the peak of the weapon above the head is not higher than before the latch, and no single frame is more than 5 cm
// above it for an ADS reload (the weapon rises with the fading AO while OLD keeps the aim-down pose until its pop) / 0.5 cm for a
// hipfire reload; (c) the peak is not higher than with the latched AO alone; (d) synthetic fixed camera: the part of the AO that
// the hand-over takes away changes the Spine3 orientation by <= 3.5 deg/frame (P3 acceptance 3) in an ADS reload and <= 8.5 deg/frame
// after a hipfire (0.12 s ramp, 56 deg over 7.2 frames; the AO before the latch ended in one step of 45-62 deg).
// (e) a reload that is cancelled or ends before the latch is gone: the hidden part of the latched AO is used up, it does not come back
// in one frame (<= 8.5 deg/frame of Spine3). (f) the driver gate covers the exit tail of the envelope (the arms return over 0.2 s):
// a live AO right after a reload enters with them (<= 8.5 deg/frame, not higher above the head than with the first gate).
// (g) the AO weight is continuous where the target is lost while the hand-over runs (one formula min(a, release, s) for the live AO and the latch).
// Mutation 17 (the product a * s for the live AO) MUST fail (g).
// Mutation 8 (UTPP_POSEV2_AO_MUTATION): the AO ignores the reload hand-over; it MUST fail (a), (b), (c), (d). Mutation 14 (aoScale only
// hides the latched AO) MUST fail (e).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/MotionPose.hpp"
#include "../src/mm/ReloadContactFade.hpp"

using namespace utpp::mm;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
template <class... A> static std::string F(const char* aFmt, A... a)
{
    char b[640];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

struct Row
{
    int seg = 0;
    float t = 0, dt = 0, yaw = 0, pitch = 0;
    bool ads = false, shoot = false, reload = false;
    int overlay = 0;
};
static std::vector<Row> LoadCsv(const char* aPath)
{
    std::vector<Row> rows;
    std::ifstream in(aPath);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#' || line[0] == 's')
            continue;
        std::vector<std::string> c;
        std::istringstream ls(line);
        for (std::string field; std::getline(ls, field, ',');)
            c.push_back(field);
        if (c.size() != 9)
            continue;
        Row r;
        r.seg = std::atoi(c[0].c_str());
        r.t = std::strtof(c[1].c_str(), nullptr);
        r.dt = std::strtof(c[2].c_str(), nullptr);
        r.yaw = std::strtof(c[3].c_str(), nullptr);
        r.pitch = std::strtof(c[4].c_str(), nullptr);
        r.ads = std::atoi(c[5].c_str()) != 0;
        r.shoot = std::atoi(c[6].c_str()) != 0;
        r.reload = std::atoi(c[7].c_str()) != 0;
        r.overlay = std::atoi(c[8].c_str());
        rows.push_back(r);
    }
    return rows;
}

static int FindClip(const motion::MotionDatabase& db, std::string_view name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name)
            return static_cast<int>(i);
    return -1;
}
static float MaxStepDegPose(const BoneTransform* a, const BoneTransform* b, uint32_t n)
{
    float m = 0.0f;
    for (uint32_t i = 0; i < n; ++i)
        m = std::max(m, ov::AngleDeg(a[i].q, b[i].q));
    return m;
}

// One frame of the driver-side inputs of an episode row.
struct DriverFrame
{
    bool wantsToAim = false, haveAim = false;
    float aoScale = 1.0f;
    float upperWeight = 0.0f, contact = 0.0f;
    bool reload = false, aimHold = false;
};

// MotionDriver.cpp Step (envelope, contact fade) + FrameInputsLocked (wantsToAim, AimHold, hipfire hold, aoScale) for one episode.
static std::vector<DriverFrame> EmulateDriver(const std::vector<Row>& aRows, size_t aFrom, size_t aTo, bool aFirstGate = false)
{
    std::vector<DriverFrame> out;
    reload_contacts_runtime::UpperEnvelope env;
    reload_contacts_runtime::CorrectionFade fade;
    bool prevAds = false; // g_aimingNow of the previous published frame = the native ADS flag
    float lastShot = -1e9f;
    for (size_t k = aFrom; k < aTo; ++k)
    {
        const Row& r = aRows[k];
        const bool reload = r.reload;
        const float dt = k == aFrom ? 1.0f / 60.0f : std::min(r.dt, 0.25f);
        const float w = env.Step(dt, reload, prevAds); // pistol
        if (r.shoot)
            lastShot = r.t;
        if (reload)
            lastShot = -1e9f; // HipFirePoseHold::Step resets on action 2
        DriverFrame f;
        f.reload = reload;
        f.aimHold = reload && env.AimHold();
        const bool hip = r.t - lastShot <= 0.75f && !reload;
        f.wantsToAim = r.ads || hip || f.aimHold;
        f.haveAim = f.wantsToAim; // target feed valid, no handback
        f.upperWeight = w;
        f.contact = fade.Step(std::min(dt, 0.05f), w, true, reload || w > 0.0f, true);
        // the driver gate: the game holds the arms (entry, reload, exit tail); aFirstGate = the first version (the reload only)
        const bool gate = aFirstGate ? (reload && env.active) : w > 0.0f;
        f.aoScale = gate ? motionpose::AoReloadScale(w, 0.0f) : 1.0f;
        prevAds = r.ads;
        out.push_back(f);
    }
    return out;
}

struct Frame
{
    float headZ = 0.0f, weaponZ = 0.0f;
    math::Quat spine3{};
    std::vector<BoneTransform> pose;
};

struct Rig
{
    std::unique_ptr<motion::MotionDatabase> db;
    std::unique_ptr<ov::OverlayFile> ovf;
    ov::StyleFamilyData style;
    std::unique_ptr<motionpose::PoseEvaluator> ev;
    std::unique_ptr<ov::Pose> tmp = std::make_unique<ov::Pose>();
    std::vector<math::Vec3> mpos = std::vector<math::Vec3>(ov::kMaxBones);
    std::vector<math::Quat> mrot = std::vector<math::Quat>(ov::kMaxBones);
    std::vector<BoneTransform> out;
    motion::MatchResult match;
    uint32_t n = 0;
    int sp3 = -1, hd = -1, wr = -1;
    bool man = false;
};

// Runs rows [aFrom, aTo) with the driver frames aDrv: aMode 0 NONE, 1 LATCH, 2 NEW.
static std::vector<Frame> Run(Rig& aR, const std::vector<Row>& aRows, size_t aFrom, size_t aTo, const std::vector<DriverFrame>& aDrv, int aMode)
{
    std::vector<Frame> frames(aTo - aFrom);
    aR.ev->Reset();
    for (size_t k = aFrom; k < aTo; ++k)
    {
        const DriverFrame& d = aDrv[k - aFrom];
        motionpose::OverlayFrameInputs in;
        in.machine.overlay = ov::PoseOverlay::Pistol2H;
        in.machine.style = aR.man ? ov::Style::Masculine : ov::Style::Feminine;
        in.machine.wantsToAim = d.wantsToAim;
        in.machine.wantsReady = d.wantsToAim;
        in.haveAim = aMode != 0 && d.haveAim;
        in.aimYawDeg = aRows[k].yaw;
        in.aimPitchDeg = aRows[k].pitch;
        in.aoScale = aMode == 2 ? d.aoScale : 1.0f;
        const float dt = k == aFrom ? 1.0f / 60.0f : std::min(aRows[k].dt, 0.25f);
        aR.ev->Evaluate(dt, aR.match, in, aR.out.data(), aR.n);
        Frame& fr = frames[k - aFrom];
        aR.ev->ToOv(aR.out.data(), *aR.tmp);
        ov::MeshPositions(aR.ev->Skel(), *aR.tmp, aR.mpos.data(), aR.mrot.data());
        fr.headZ = aR.mpos[static_cast<size_t>(aR.hd)].z;
        fr.weaponZ = aR.mpos[static_cast<size_t>(aR.wr)].z;
        fr.spine3 = aR.mrot[static_cast<size_t>(aR.sp3)];
        fr.pose = aR.out;
    }
    return frames;
}

static bool Load(Rig& aR, const char* aTag, const char* aDb, const char* aOv, const char* aStyle, bool aMan)
{
    std::string err;
    aR.man = aMan;
    aR.db = std::make_unique<motion::MotionDatabase>();
    const bool dbOk = aR.db->LoadFromFile(aDb, aMan ? motion::kRigIdentityMan : motion::kRigIdentityWoman, err);
    aR.ovf = std::make_unique<ov::OverlayFile>();
    const bool ovOk = aR.ovf->LoadFromFile(aOv, aMan ? ov::kRigIdentityManOv : ov::kRigIdentityWomanOv, err);
    std::ifstream sf(aStyle, std::ios::binary);
    std::stringstream ss;
    ss << sf.rdbuf();
    const bool stOk = sf.good() && aR.style.Parse(ss.str(), &err);
    Check(dbOk && ovOk && stOk, F("%s: data loaded (%s)", aTag, err.c_str()));
    if (!dbOk || !ovOk || !stOk)
        return false;
    aR.n = aR.db->NumBones();
    aR.ev = std::make_unique<motionpose::PoseEvaluator>();
    const bool init = aR.ev->Init(aR.db.get(), aR.ovf.get(), &aR.style, 2, err);
    const int idle = FindClip(*aR.db, "M_Neutral_Stand_Idle_Loop");
    const ov::Skeleton& sk = aR.ev->Skel();
    for (int k = 0; init && k < sk.n; ++k)
    {
        if (aR.ovf->BoneName(k) == "Spine3") aR.sp3 = k;
        if (aR.ovf->BoneName(k) == "Head") aR.hd = k;
        if (aR.ovf->BoneName(k) == "WeaponRight") aR.wr = k;
    }
    Check(init && idle >= 0 && aR.sp3 >= 0 && aR.hd >= 0 && aR.wr >= 0,
          F("%s: PoseEvaluator Init (%s), idle clip %d, bones Spine3 %d Head %d WeaponRight %d", aTag, err.c_str(), idle, aR.sp3, aR.hd, aR.wr));
    if (!init || idle < 0 || aR.sp3 < 0 || aR.hd < 0 || aR.wr < 0)
        return false;
    aR.out.resize(aR.n);
    aR.match.clip = idle;
    aR.match.clipTime = 0.5f;
    return true;
}

// (d) fixed camera: the part of the AO that the hand-over takes away. ADS: 90 frames of ADS then a reload; hipfire: shots at frames 60-64 (the Aiming weight is 1 at the reload),
// the reload at frame 90 (hold 0.75 s reset by the reload). The AO (yaw 40, pitch aPitch) is the same in LATCH and NEW up to the
// hand-over, so the orientation difference NEW / LATCH changes only by the aoScale.
static float FadeStep(Rig& aR, bool aAds, float aPitch)
{
    std::vector<Row> rows(150);
    for (size_t k = 0; k < rows.size(); ++k)
    {
        Row& r = rows[k];
        r.t = static_cast<float>(k) / 60.0f;
        r.dt = 1.0f / 60.0f;
        r.yaw = 40.0f;
        r.pitch = aPitch;
        r.overlay = 3;
        r.ads = aAds && k < 90;
        r.shoot = !aAds && k >= 60 && k < 65;
        r.reload = k >= 90;
    }
    const std::vector<DriverFrame> drv = EmulateDriver(rows, 0, rows.size());
    const std::vector<Frame> latch = Run(aR, rows, 0, rows.size(), drv, 1), nw = Run(aR, rows, 0, rows.size(), drv, 2);
    float step = 0.0f, prev = 0.0f;
    for (size_t k = 90; k < rows.size(); ++k)
    {
        const float d = ov::AngleDeg(nw[k].spine3, latch[k].spine3);
        step = std::max(step, std::fabs(d - prev));
        prev = d;
    }
    return step;
}

// (e) a reload that is cancelled or ends before the latch is gone (sprint, weapon switch, melee, a short reload): the part of the
// latched AO that the hand-over has hidden must not come back. Fixed camera (yaw 40, pitch aPitch); the largest one-frame change of the
// AO share of Spine3 (NEW against the run without an AO) from the last frames before the reload to 1.5 s after it.
static float CancelStep(Rig& aR, bool aAds, float aPitch, int aReloadFrames, float* aRise = nullptr)
{
    std::vector<Row> rows(210);
    for (size_t k = 0; k < rows.size(); ++k)
    {
        Row& r = rows[k];
        r.t = static_cast<float>(k) / 60.0f;
        r.dt = 1.0f / 60.0f;
        r.yaw = 40.0f;
        r.pitch = aPitch;
        r.overlay = 3;
        r.ads = aAds && k < 90;
        r.shoot = !aAds && k >= 60 && k < 65;
        r.reload = k >= 90 && k < 90 + static_cast<size_t>(aReloadFrames);
    }
    const std::vector<DriverFrame> drv = EmulateDriver(rows, 0, rows.size());
    const std::vector<Frame> none = Run(aR, rows, 0, rows.size(), drv, 0), nw = Run(aR, rows, 0, rows.size(), drv, 2);
    float step = 0.0f, prev = ov::AngleDeg(nw[85].spine3, none[85].spine3), atCancel = 0.0f, rise = 0.0f;
    const size_t cancel = 90 + static_cast<size_t>(aReloadFrames) - 1; // the last frame of the reload action
    for (size_t k = 86; k < rows.size(); ++k)
    {
        const float share = ov::AngleDeg(nw[k].spine3, none[k].spine3);
        step = std::max(step, std::fabs(share - prev));
        prev = share;
        if (k == cancel)
            atCancel = share;
        if (k > cancel)
            rise = std::max(rise, share - atCancel); // the AO share grows above its level at the end of the reload: the hidden part came back
    }
    if (aRise)
        *aRise = rise;
    return step;
}

// (f) right after a reload (fixed camera, pitch -60, yaw 40): the player shoots from the hip / enters ADS the frame the reload ends. The
// game still hands the arms back over 0.2 s (contacts fade 0.2 s). Peak weapon above the head and the largest one-frame change of the AO
// share of Spine3 in the 40 frames after the end, for the driver gate (weight > 0) and for the first gate (the reload action only).
static void ExitTail(Rig& aR, bool aAds, bool aFirstGate, float& aPeak, float& aStep)
{
    std::vector<Row> rows(220);
    for (size_t k = 0; k < rows.size(); ++k)
    {
        Row& r = rows[k];
        r.t = static_cast<float>(k) / 60.0f;
        r.dt = 1.0f / 60.0f;
        r.yaw = 40.0f;
        r.pitch = -60.0f;
        r.overlay = 3;
        r.ads = (aAds && k < 90) || (aAds && k >= 150);
        r.shoot = (!aAds && k >= 60 && k < 65) || (!aAds && k >= 150 && k < 160);
        r.reload = k >= 90 && k < 150;
    }
    const std::vector<DriverFrame> drv = EmulateDriver(rows, 0, rows.size(), aFirstGate);
    const std::vector<Frame> none = Run(aR, rows, 0, rows.size(), drv, 0), nw = Run(aR, rows, 0, rows.size(), drv, 2);
    const float nativeZ = none[89].headZ + 0.10f;
    aPeak = 0.0f;
    aStep = 0.0f;
    float prev = ov::AngleDeg(nw[149].spine3, none[149].spine3);
    for (size_t k = 150; k < 190; ++k)
    {
        const float c = drv[k].contact;
        const float ah = std::max(0.0f, (1.0f - c) * (nw[k].weaponZ - nw[k].headZ) + c * (nativeZ - nw[k].headZ));
        aPeak = std::max(aPeak, ah);
        const float share = ov::AngleDeg(nw[k].spine3, none[k].spine3);
        aStep = std::max(aStep, std::fabs(share - prev));
        prev = share;
    }
}

// (g) the weight is continuous where the target is lost while the hand-over is running: a = 0.5 (the Aiming weight fades after wantsToAim ended),
// s = 0.6 (aoScale constant). The target stays live for 15 frames after the machine began to fade, then it is lost. The AO share of Spine3 (against the
// run without an AO) must not jump at the loss: the live weight a * s = 0.30 became min(a, s) = 0.50 of the latch, +0.2 of the AO: 7 deg in one frame.
static float BoundaryStep(Rig& aR, float aScale)
{
    aR.ev->Reset();
    std::vector<Frame> with, none;
    auto run = [&](bool aAo, std::vector<Frame>& aOut) {
        aR.ev->Reset();
        for (int k = 0; k < 150; ++k)
        {
            motionpose::OverlayFrameInputs in;
            in.machine.overlay = ov::PoseOverlay::Pistol2H;
            in.machine.style = aR.man ? ov::Style::Masculine : ov::Style::Feminine;
            in.machine.wantsToAim = k < 90; // aiming, then the machine fades the Aiming weight 1 -> 0 over 0.5 s
            in.haveAim = aAo && k < 105;   // the target is lost 15 frames into the fade: a = 0.5
            in.aimYawDeg = 40.0f;
            in.aimPitchDeg = -60.0f;
            in.aoScale = k >= 60 ? aScale : 1.0f;
            aR.ev->Evaluate(1.0f / 60.0f, aR.match, in, aR.out.data(), aR.n);
            Frame fr;
            aR.ev->ToOv(aR.out.data(), *aR.tmp);
            ov::MeshPositions(aR.ev->Skel(), *aR.tmp, aR.mpos.data(), aR.mrot.data());
            fr.spine3 = aR.mrot[static_cast<size_t>(aR.sp3)];
            aOut.push_back(fr);
        }
    };
    run(true, with);
    run(false, none);
    float step = 0.0f, prev = ov::AngleDeg(with[89].spine3, none[89].spine3);
    for (size_t k = 90; k < 150; ++k)
    {
        const float share = ov::AngleDeg(with[k].spine3, none[k].spine3);
        step = std::max(step, std::fabs(share - prev));
        prev = share;
    }
    return step;
}

static void RunBody(const char* aTag, const char* aDb, const char* aOv, const char* aStyle, bool aMan, const std::vector<Row>& aRows)
{
    Rig rig;
    if (!Load(rig, aTag, aDb, aOv, aStyle, aMan))
        return;
    const uint32_t n = rig.n;
    int episodes = 0, adsLow = 0, hipLow = 0, bad = 0;
    float worstAdsPeak = -9.0f, worstHipPeak = -9.0f, worstAdsFrame = -9.0f, worstHipFrame = -9.0f, worstLatchPeak = -9.0f;
    size_t i = 0;
    while (i < aRows.size())
    {
        size_t j = i;
        while (j < aRows.size() && aRows[j].seg == aRows[i].seg)
            ++j;
        const size_t segFrom = i, segTo = j;
        i = j;
        // the reload edge: the window can start inside the previous reload
        size_t e = segFrom + 1;
        while (e < segTo && !(aRows[e].reload && !aRows[e - 1].reload))
            ++e;
        if (e >= segTo || aRows[e].overlay != 3)
            continue; // the AimHold / contact logic is the Nue pistol path
        const std::vector<DriverFrame> drv = EmulateDriver(aRows, segFrom, segTo);
        const std::vector<Frame> none = Run(rig, aRows, segFrom, segTo, drv, 0), latch = Run(rig, aRows, segFrom, segTo, drv, 1),
                                 nw = Run(rig, aRows, segFrom, segTo, drv, 2);
        ++episodes;
        const size_t ke = e - segFrom;
        const float nativeZ = none[ke - 1].headZ + 0.10f;
        const bool ads = aRows[e - 1].ads, lowPitch = aRows[e].pitch <= -20.0f;
        const size_t last = std::min(none.size(), ke + 60);
        auto above = [&](const Frame& fr, float c) { return std::max(0.0f, (1.0f - c) * (fr.weaponZ - fr.headZ) + c * (nativeZ - fr.headZ)); };
        float peakNew = 0.0f, peakLatch = 0.0f, peakOld = 0.0f, frameOver = -9.0f;
        bool gone = true;
        for (size_t k = ke - 1; k < last; ++k)
        {
            const DriverFrame& d = drv[k];
            const Frame& old = d.haveAim ? latch[k] : none[k];
            // (a) the game owns the upper body: the AO is gone
            if (d.upperWeight >= 0.999f && d.reload)
                gone = gone && MaxStepDegPose(nw[k].pose.data(), none[k].pose.data(), n) <= 0.5f;
            if (k >= ke)
            {
                const float ahNew = above(nw[k], d.contact), ahOld = above(old, d.contact);
                peakNew = std::max(peakNew, ahNew);
                peakLatch = std::max(peakLatch, above(latch[k], d.contact));
                peakOld = std::max(peakOld, ahOld);
                frameOver = std::max(frameOver, ahNew - ahOld);
            }
        }
        bad += gone ? 0 : 1;
        if (lowPitch)
        {
            (ads ? adsLow : hipLow) += 1;
            float& peak = ads ? worstAdsPeak : worstHipPeak;
            float& frame = ads ? worstAdsFrame : worstHipFrame;
            peak = std::max(peak, peakNew - peakOld);
            frame = std::max(frame, frameOver);
            worstLatchPeak = std::max(worstLatchPeak, peakNew - peakLatch);
        }
        if (lowPitch)
            std::printf("INFO %s episode %d t0 %.3f %s pitch %.1f: peak weapon above head (cm) NEW %.1f LATCH %.1f OLD %.1f, worst single frame NEW - OLD %+.1f\n", aTag,
                        aRows[segFrom].seg, aRows[e].t, ads ? "ADS" : "hipfire", aRows[e].pitch, peakNew * 100.0f, peakLatch * 100.0f, peakOld * 100.0f,
                        frameOver * 100.0f);
    }
    Check(episodes >= 20, F("%s: %d pistol reload episodes replayed (%d ADS and %d hipfire at pitch <= -20)", aTag, episodes, adsLow, hipLow));
    Check(bad == 0, F("%s: (a) once the game owns the upper body the AO is gone (NEW == no AO within 0.5 deg): %d of %d episodes differ", aTag, bad, episodes));
    Check(adsLow >= 7 && hipLow >= 1 && worstAdsPeak <= 0.005f && worstHipPeak <= 0.005f,
          F("%s: (b) peak weapon above the head NEW - before the latch at pitch <= -20: ADS %+.2f cm (%d episodes), hipfire %+.2f cm (%d) (<= 0.5)", aTag,
            worstAdsPeak * 100.0f, adsLow, worstHipPeak * 100.0f, hipLow));
    Check(worstAdsFrame <= 0.05f && worstHipFrame <= 0.005f,
          F("%s: (b) worst single frame above the one before the latch: ADS reload %+.2f cm (<= 5), hipfire reload %+.2f cm (<= 0.5)", aTag,
            worstAdsFrame * 100.0f, worstHipFrame * 100.0f));
    Check(worstLatchPeak <= 0.001f, F("%s: (c) the peak is not higher than with the latched AO alone: %+.2f cm (<= 0.1)", aTag, worstLatchPeak * 100.0f));
    // cancelled / short reloads at pitch -60: hipfire after 18 frames (0.30 s), ADS after 27 frames (0.45 s, the aim hold ended at 18),
    // and a complete reload for reference
    float riseHip = 0.0f, riseAds = 0.0f, risePart = 0.0f;
    const float cancelHip = CancelStep(rig, false, -60.0f, 18, &riseHip), cancelAds = CancelStep(rig, true, -60.0f, 27, &riseAds);
    const float cancelPart = CancelStep(rig, false, -60.0f, 4, &risePart); // cancelled while the hand-over is half done
    const float fullHip = CancelStep(rig, false, -60.0f, 90), fullAds = CancelStep(rig, true, -60.0f, 90);
    Check(cancelHip <= 8.5f && cancelAds <= 8.5f && cancelPart <= 8.5f && fullHip <= 8.5f && fullAds <= 8.5f,
          F("%s: (e) a reload cancelled after 0.30 s (hipfire) / 0.45 s (ADS) / 0.07 s (hipfire, half handed over) at pitch -60: the largest one-frame change of the AO "
            "share of Spine3 is %.2f / %.2f / %.2f deg (<= 8.5; the hidden latch used to come back in one frame: 42 / 27 deg); complete reloads %.2f / %.2f",
            aTag, cancelHip, cancelAds, cancelPart, fullHip, fullAds));
    Check(riseHip <= 1.0f && riseAds <= 1.0f && risePart <= 1.0f,
          F("%s: (e) ... and the hidden latch does not come back at all: the AO share never rises above its level at the end of the reload by more than "
            "%.2f / %.2f / %.2f deg (<= 1; used up by the hand-over)", aTag, riseHip, riseAds, risePart));
    const float boundary = BoundaryStep(rig, 0.6f), boundaryFull = BoundaryStep(rig, 1.0f);
    Check(boundary <= 3.0f && boundaryFull <= 3.0f,
          F("%s: (g) the target is lost while the hand-over runs (a = 0.5, aoScale 0.6): the largest one-frame change of the AO share of Spine3 is %.2f deg (<= 3; "
            "the product for the live AO and min(a, s) for the latch jumped by 0.2 of the AO: 7 deg); without a hand-over %.2f",
            aTag, boundary, boundaryFull));
    float peakTail[2][2], stepTail[2][2]; // [ads][first gate]
    for (int ads = 0; ads < 2; ++ads)
        for (int first = 0; first < 2; ++first)
            ExitTail(rig, ads != 0, first != 0, peakTail[ads][first], stepTail[ads][first]);
    Check(stepTail[0][0] <= 8.5f && stepTail[1][0] <= 8.5f && peakTail[0][0] <= peakTail[0][1] + 0.001f && peakTail[1][0] <= peakTail[1][1] + 0.001f,
          F("%s: (f) right after a reload at pitch -60: hipfire shot / ADS in the frame the reload ends: the largest one-frame change of the AO share %.2f / %.2f deg "
            "(first gate: %.2f / %.2f), peak weapon above the head %.1f / %.1f cm (first gate %.1f / %.1f) - the AO enters with the returning arms",
            aTag, stepTail[0][0], stepTail[1][0], stepTail[0][1], stepTail[1][1], peakTail[0][0] * 100.0f, peakTail[1][0] * 100.0f,
            peakTail[0][1] * 100.0f, peakTail[1][1] * 100.0f));
    const float adsStep = std::max(FadeStep(rig, true, -65.0f), FadeStep(rig, true, -30.0f));
    const float hipStep = std::max(FadeStep(rig, false, -65.0f), FadeStep(rig, false, -30.0f));
    Check(adsStep <= 3.5f && hipStep <= 8.5f && adsStep > 0.5f && hipStep > 0.5f,
          F("%s: (d) fixed camera, the hand-over takes the AO away at %.2f deg/frame of Spine3 in an ADS reload (<= 3.5) and %.2f after a hipfire (<= 8.5: ~56 deg over the 7.2 frames of the 0.12 s ramp); "
            "the AO before the latch ended in one step of 45-62 deg",
            aTag, adsStep, hipStep));
}

int main(int argc, char** argv)
{
    if (argc < 8)
    {
        std::printf("usage: %s woman.utppmm woman.utppov woman_style.txt man.utppmm man.utppov man_style.txt reload.csv\n", argv[0]);
        return 2;
    }
    const std::vector<Row> rows = LoadCsv(argv[7]);
    Check(rows.size() > 9000, F("fixture %s: %zu rows", argv[7], rows.size()));
    if (rows.size() > 9000)
    {
        RunBody("woman", argv[1], argv[2], argv[3], false, rows);
        RunBody("man", argv[4], argv[5], argv[6], true, rows);
    }
    std::printf("aoreload: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

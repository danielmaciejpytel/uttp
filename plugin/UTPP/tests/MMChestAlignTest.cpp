// Offline test of strafe S2 chest align (mm/ChestAlign.hpp, b1 strafe design point 8 + critique MUST changes), not shipped.
// Usage: utpp_mm_chestalign_test <database.utppmm>      Exit code 0 = all checks passed.
//
// Plays the motion facing-fixed walk / crouch loops (v6 clip names) at 30 / 60 / 144 Hz (nearest database frame of the
// real time), runs ChestAlign on every pose with weight 1 and measures the chest twist about Z relative to the stance
// reference before / after the correction (mean and swing = (max - min) / 2 over one loop period after 3 warm-up
// periods). swayKeep is verified EXACTLY with smoothHalflife 0 (delta = target): per frame the corrected twist must equal
// swayKeep * (dev - devLow) within 0.05 deg (swayKeep 0.5 and 0), and Spine_model' must equal Rz(delta) * Spine_model
// (formula / multiplication order). With the default 0.05 s smoothing the swing ratio is only a sanity band (0.3..0.9:
// the lag and the 0.3 s low-pass following part of the sway leave 0.5..0.86 on v6), plus swayKeep 0 must leave less
// swing than swayKeep 0.5 and at most 0.7 x the base swing (the lag residual is 0.3..0.65 x base on v6, largest on the
// small-swing crouch loops at 144 Hz). Also: weight 0 and Hips / legs bit-identical, clamp, NaN
// robustness, stale-free diagnostics, determinism, stance-switch devLow re-init, per-stance references.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/ChestAlign.hpp"

using namespace utpp::mm;

static int g_fail = 0;
static int g_pass = 0;
static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}
static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0)
{
    char buf[512];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d, e, f);
    return buf;
}
static constexpr float kRad2Deg = 180.0f / math::kPi;

static void LoadPose(const Database& aDb, uint32_t aFrame, std::vector<BoneTransform>& aOut)
{
    const auto p = aDb.Pose(aFrame);
    aOut.resize(aDb.NumBones());
    for (uint32_t b = 0; b < aDb.NumBones(); ++b)
    {
        aOut[b].p = {p[b * 7 + 0], p[b * 7 + 1], p[b * 7 + 2]};
        aOut[b].q = {p[b * 7 + 3], p[b * 7 + 4], p[b * 7 + 5], p[b * 7 + 6]};
    }
}

struct LoopStats
{
    float baseMean = 0, baseSwing = 0, outMean = 0, outSwing = 0, maxCorr = 0;
    float maxFormulaErr = 0; // max component error of Spine_model' vs Rz(delta) * Spine_model (sign-aligned)
    float maxSwayErr = 0;    // deg, max |out - swayKeep * (base - low)| over unclamped frames (exact only when smoothHalflife 0)
    int swayFrames = 0;
    bool othersIdentical = true, finite = true;
};

// Plays clip aClip (loopPeriod frames) at aHz for 4 periods, measures the last one.
static LoopStats PlayLoop(const Database& aDb, ChestAlign& aCa, int aStance, int32_t aClip, float aHz, float aWeight = 1.0f)
{
    LoopStats s;
    const ClipRecord& c = aDb.Clips()[size_t(aClip)];
    const uint32_t period = c.loopPeriod > 0 ? c.loopPeriod : c.numFrames;
    const float fps = aDb.Fps();
    const float dt = 1.0f / aHz;
    const float periodTime = float(period) / fps;
    const int steps = int(std::ceil(4.0f * periodTime * aHz));
    const int measureFrom = int(std::floor(3.0f * periodTime * aHz));
    std::vector<BoneTransform> pose, orig;
    double bs = 0, os = 0;
    int n = 0;
    float bMin = 1e9f, bMax = -1e9f, oMin = 1e9f, oMax = -1e9f;
    aCa.Reset();
    for (int i = 0; i < steps; ++i)
    {
        const uint32_t f = c.firstFrame + uint32_t(int64_t(std::floor(double(i) * dt * fps + 1e-6)) % int64_t(period));
        LoadPose(aDb, f, pose);
        orig = pose;
        aCa.Apply(pose, aStance, aWeight, dt);
        if (i < measureFrom)
            continue;
        const float b = aCa.Deviation(orig, aStance) * kRad2Deg;
        const float o = aCa.Deviation(pose, aStance) * kRad2Deg;
        if (!std::isfinite(b) || !std::isfinite(o))
            s.finite = false;
        bs += b;
        os += o;
        ++n;
        bMin = std::min(bMin, b), bMax = std::max(bMax, b), oMin = std::min(oMin, o), oMax = std::max(oMax, o);
        {
            const float d = aCa.Diagnostics().delta / kRad2Deg;
            const math::Quat rz{0.0f, 0.0f, std::sin(0.5f * d), std::cos(0.5f * d)};
            const math::Quat expect = math::QuatNormalize(math::QuatMul(rz, math::QuatNormalize(aCa.Model(orig, aCa.SpineBone()).q)));
            math::Quat got = math::QuatNormalize(aCa.Model(pose, aCa.SpineBone()).q);
            if (math::QuatDot(got, expect) < 0.0f)
                got = math::QuatNeg(got);
            s.maxFormulaErr = std::max({s.maxFormulaErr, std::fabs(got.x - expect.x), std::fabs(got.y - expect.y), std::fabs(got.z - expect.z),
                                        std::fabs(got.w - expect.w)});
            const float k = aCa.Config().swayKeep, low = aCa.Diagnostics().low;
            const float tgt = low + (1.0f - k) * (b - low);
            if (std::fabs(tgt) < aCa.Config().chestMaxDeg - 0.01f)
            {
                float e = o - k * (b - low);
                e = std::fmod(e + 540.0f, 360.0f) - 180.0f;
                s.maxSwayErr = std::max(s.maxSwayErr, std::fabs(e));
                ++s.swayFrames;
            }
        }
        s.maxCorr = std::max(s.maxCorr, std::fabs(aCa.Diagnostics().delta));
        for (size_t k = 0; k < pose.size(); ++k)
            if (int32_t(k) != aCa.SpineBone() && std::memcmp(&pose[k], &orig[k], sizeof(BoneTransform)) != 0)
                s.othersIdentical = false;
    }
    s.baseMean = float(bs / std::max(n, 1));
    s.outMean = float(os / std::max(n, 1));
    s.baseSwing = 0.5f * (bMax - bMin);
    s.outSwing = 0.5f * (oMax - oMin);
    return s;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s <database.utppmm>\n", argv[0]);
        return 2;
    }
    Database db;
    std::string err;
    if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, err))
    {
        std::printf("load failed: %s\n", err.c_str());
        return 2;
    }
    std::printf("database %s: %u frames, %u clips, %.0f fps\n", argv[1], db.NumFrames(), db.NumClips(), double(db.Fps()));

    ChestAlignConfig cfg; // defaults: swayKeep 0.5, chestMaxDeg 60, devLow 0.3 s, smooth 0.05 s
    ChestAlign ca;
    Expect("init (Hips / Spine resolved, Spine child of Hips)", ca.Init(db, cfg) && ca.HipsBone() >= 0 && ca.SpineBone() >= 0);
    {
        ChestAlign none;
        none.Init(db, cfg);
        std::vector<BoneTransform> pose, orig;
        LoadPose(db, 100, pose);
        orig = pose;
        none.Apply(pose, ChestAlign::kStand, 1.0f, 1.0f / 60.0f);
        Expect("no reference -> pose bit-identical, counted",
               std::memcmp(pose.data(), orig.data(), pose.size() * sizeof(BoneTransform)) == 0 && none.Diagnostics().noReference == 1);
    }
    const int32_t standF = ChestAlign::FindClip(db, "M_Neutral_Walk_Loop_F");
    const int32_t crouchF = ChestAlign::FindClip(db, "M_Neutral_Crouch_Loop_F");
    Expect("reference clips present", standF >= 0 && crouchF >= 0);
    if (standF < 0 || crouchF < 0)
        return 1;
    const ClipRecord& sc = db.Clips()[size_t(standF)];
    const ClipRecord& cc = db.Clips()[size_t(crouchF)];
    Expect("stand reference (forward walk loop)", ca.BuildReference(db, ChestAlign::kStand, sc.firstFrame, sc.loopPeriod));
    Expect("crouch reference (forward crouch loop)", ca.BuildReference(db, ChestAlign::kCrouch, cc.firstFrame, cc.loopPeriod));
    {
        const math::Quat rs = ca.Reference(ChestAlign::kStand), rc = ca.Reference(ChestAlign::kCrouch);
        const float ang = math::QuatAngleBetween(rs, rc) * kRad2Deg;
        const float yaw = ChestAlign::TwistZ(math::QuatMul(rc, math::QuatInv(rs))) * kRad2Deg;
        // crouch forward loop against the STAND reference: what a single shared reference would leave as offset
        ChestAlign wrong;
        wrong.Init(db, cfg);
        wrong.BuildReference(db, ChestAlign::kStand, sc.firstFrame, sc.loopPeriod);
        const LoopStats w = PlayLoop(db, wrong, ChestAlign::kStand, crouchF, 60.0f, 0.0f);
        Expect("crouch reference differs from stand", ang > 2.0f,
               Fmt("angle %.1f deg, yaw part %.2f deg; crouch F vs stand ref: mean %.2f swing %.2f", ang, yaw, w.baseMean, w.baseSwing));
    }
    {
        const LoopStats f = PlayLoop(db, ca, ChestAlign::kStand, standF, 60.0f);
        Expect("stand forward loop vs own reference: mean ~0", std::fabs(f.baseMean) <= 2.0f,
               Fmt("mean %.2f swing %.2f -> out mean %.2f swing %.2f", f.baseMean, f.baseSwing, f.outMean, f.outSwing));
    }

    struct L
    {
        const char* name;
        int stance;
        bool required;
    };
    const L loops[] = {
        {"M_Neutral_Walk_Loop_LL", 0, true},         {"M_Neutral_Walk_Loop_RL", 0, true},         {"M_Neutral_Walk_Loop_B", 0, true},
        {"M_Neutral_Walk_Loop_BL", 0, true},         {"M_Neutral_Walk_Loop_BR", 0, true},         {"M_Neutral_Walk_Loop_FL", 0, false},
        {"M_Neutral_Walk_Loop_FR", 0, false},        {"M_Neutral_Walk_Loop_LR_offset", 0, false}, {"M_Neutral_Walk_Loop_RR_offset", 0, false},
        {"M_Neutral_Crouch_Loop_LL", 1, true},       {"M_Neutral_Crouch_Loop_RL", 1, true},       {"M_Neutral_Crouch_Loop_B", 1, true},
        {"M_Neutral_Crouch_Loop_BL", 1, true},       {"M_Neutral_Crouch_Loop_BR", 1, true},       {"M_Neutral_Crouch_Loop_LR", 1, false},
        {"M_Neutral_Crouch_Loop_RR", 1, false},
    };
    const float rates[] = {30.0f, 60.0f, 144.0f};
    std::printf("\nloop                              Hz   base mean/swing   keep0 mean/swing   keep0.5 mean/swing (ratio)  max|corr|\n");
    for (const L& l : loops)
    {
        const int32_t clip = ChestAlign::FindClip(db, l.name);
        if (clip < 0)
        {
            Expect(l.name, !l.required, "clip missing");
            continue;
        }
        for (float hz : rates)
        {
            ChestAlignConfig c0 = cfg;
            c0.swayKeep = 0.0f;
            ca.SetConfig(c0);
            const LoopStats a = PlayLoop(db, ca, l.stance, clip, hz);
            ca.SetConfig(cfg);
            const LoopStats b = PlayLoop(db, ca, l.stance, clip, hz);
            const float ratio = a.baseSwing > 1e-3f ? b.outSwing / a.baseSwing : 0.0f;
            std::printf("%-33s %4.0f   %+7.2f / %5.2f   %+6.2f / %5.2f    %+6.2f / %5.2f (%.2f)    %5.2f\n", l.name, double(hz),
                        double(a.baseMean), double(a.baseSwing), double(a.outMean), double(a.outSwing), double(b.outMean),
                        double(b.outSwing), double(ratio), double(std::max(a.maxCorr, b.maxCorr)));
            if (!l.required)
                continue;
            const std::string tag = std::string(l.name) + Fmt(" @%.0f Hz", hz);
            Expect((tag + ": swayKeep 0 residual mean <= 2 deg").c_str(), std::fabs(a.outMean) <= 2.0f && a.finite,
                   Fmt("base %.2f -> %.2f", a.baseMean, a.outMean));
            Expect((tag + ": swayKeep 0.5 mean removed, swing x(0.3..0.9)").c_str(),
                   std::fabs(b.outMean) <= 2.0f && (a.baseSwing < 2.0f || (ratio >= 0.3f && ratio <= 0.9f)),
                   Fmt("mean %.2f, swing %.2f -> %.2f (ratio %.2f)", b.outMean, a.baseSwing, b.outSwing, ratio));
            Expect((tag + ": |correction| <= 60, non-spine bones bit-identical").c_str(),
                   a.maxCorr <= 60.0001f && b.maxCorr <= 60.0001f && a.othersIdentical && b.othersIdentical);
            Expect((tag + ": swayKeep 0 swing < swayKeep 0.5 swing, <= 0.7 x base").c_str(),
                   a.baseSwing < 2.0f || (a.outSwing < b.outSwing && a.outSwing <= 0.7f * a.baseSwing),
                   Fmt("keep0 %.2f, keep0.5 %.2f, base %.2f", a.outSwing, b.outSwing, a.baseSwing));
            Expect((tag + ": Spine_model' == Rz(delta) * Spine_model").c_str(), a.maxFormulaErr < 1e-5f && b.maxFormulaErr < 1e-5f,
                   Fmt("max component err %.2e / %.2e", a.maxFormulaErr, b.maxFormulaErr));
            // exact swayKeep semantics: no smoothing -> out twist == swayKeep * (dev - devLow)
            for (float keep : {0.5f, 0.0f})
            {
                ChestAlignConfig cx = cfg;
                cx.swayKeep = keep;
                cx.smoothHalflife = 0.0f;
                ca.SetConfig(cx);
                const LoopStats x = PlayLoop(db, ca, l.stance, clip, hz);
                ca.SetConfig(cfg);
                Expect((tag + Fmt(": smooth 0, swayKeep %.1f -> out == swayKeep*(dev-low)", keep)).c_str(),
                       x.swayFrames > 10 && x.maxSwayErr < 0.05f && x.maxFormulaErr < 1e-5f && (keep > 0.0f || x.outSwing < 0.1f),
                       Fmt("max err %.4f deg over %.0f frames, out swing %.2f (base %.2f)", x.maxSwayErr, x.swayFrames, x.outSwing, x.baseSwing));
            }
        }
    }
    ca.SetConfig(cfg);

    // weight 0 -> bit-identical (filters still run)
    {
        const int32_t clip = ChestAlign::FindClip(db, "M_Neutral_Walk_Loop_LL");
        std::vector<BoneTransform> pose, orig;
        bool same = true;
        ca.Reset();
        for (uint32_t i = 0; i < 120; ++i)
        {
            LoadPose(db, db.Clips()[size_t(clip)].firstFrame + i % db.Clips()[size_t(clip)].loopPeriod, pose);
            orig = pose;
            ca.Apply(pose, ChestAlign::kStand, 0.0f, 1.0f / 60.0f);
            same = same && std::memcmp(pose.data(), orig.data(), pose.size() * sizeof(BoneTransform)) == 0;
        }
        Expect("weight 0 -> whole pose bit-identical", same && ca.Diagnostics().delta == 0.0f && std::fabs(ca.Diagnostics().low) > 1.0f,
               Fmt("filter low %.2f deg", ca.Diagnostics().low));
    }
    // clamp: Spine yawed by +80 deg on top of the forward pose
    {
        std::vector<BoneTransform> pose;
        bool ok = true;
        float maxAbs = 0.0f, resid = 0.0f;
        ChestAlignConfig c0 = cfg;
        c0.swayKeep = 0.0f;
        ca.SetConfig(c0);
        ca.Reset();
        for (int i = 0; i < 120; ++i)
        {
            LoadPose(db, sc.firstFrame, pose);
            const BoneTransform h = ca.Model(pose, ca.HipsBone());
            const float a = 80.0f / kRad2Deg;
            const math::Quat rz{0.0f, 0.0f, std::sin(0.5f * a), std::cos(0.5f * a)};
            const math::Quat pre = math::QuatMul(math::QuatMul(math::QuatInv(h.q), rz), h.q);
            pose[size_t(ca.SpineBone())].q = math::QuatNormalize(math::QuatMul(pre, pose[size_t(ca.SpineBone())].q));
            ca.Apply(pose, ChestAlign::kStand, 1.0f, 1.0f / 60.0f);
            maxAbs = std::max(maxAbs, std::fabs(ca.Diagnostics().delta));
            resid = ca.Deviation(pose, ChestAlign::kStand) * kRad2Deg;
            ok = ok && std::isfinite(resid);
        }
        ca.SetConfig(cfg);
        LoadPose(db, sc.firstFrame, pose);
        const float dev0 = ca.Deviation(pose, ChestAlign::kStand) * kRad2Deg;
        Expect("80 deg twist -> correction clamped at 60", ok && maxAbs <= 60.0001f && maxAbs >= 59.9f && std::fabs(resid - (dev0 + 20.0f)) < 0.5f,
               Fmt("max |delta| %.3f, residual %.2f (expected %.2f)", maxAbs, resid, dev0 + 20.0f));
    }
    // NaN robustness
    {
        std::vector<BoneTransform> pose, orig;
        ca.Reset();
        LoadPose(db, sc.firstFrame, pose);
        ca.Apply(pose, ChestAlign::kStand, 1.0f, 1.0f / 60.0f);
        const uint64_t nf0 = ca.Diagnostics().nonFinite;
        LoadPose(db, sc.firstFrame + 5, pose);
        pose[size_t(ca.HipsBone())].q.x = std::numeric_limits<float>::quiet_NaN();
        orig = pose;
        ca.Apply(pose, ChestAlign::kStand, 1.0f, 1.0f / 60.0f);
        const bool nanPose = std::memcmp(pose.data(), orig.data(), pose.size() * sizeof(BoneTransform)) == 0 && ca.Diagnostics().nonFinite == nf0 + 1;
        const bool nanDiag = !ca.Diagnostics().valid && ca.Diagnostics().dev == 0.0f && ca.Diagnostics().low == 0.0f;
        LoadPose(db, sc.firstFrame + 6, pose);
        orig = pose;
        ca.Apply(pose, ChestAlign::kStand, std::numeric_limits<float>::quiet_NaN(), 1.0f / 60.0f);
        const bool nanW = std::memcmp(pose.data(), orig.data(), pose.size() * sizeof(BoneTransform)) == 0;
        bool fin = true;
        const float bad[] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -1.0f, 0.0f, 1e9f};
        for (float d : bad)
        {
            LoadPose(db, sc.firstFrame + 7, pose);
            ca.Apply(pose, ChestAlign::kStand, 1.0f, d);
            for (const BoneTransform& b : pose)
                fin = fin && math::IsFinite(b.q) && math::IsFinite(b.p);
            fin = fin && std::isfinite(ca.Diagnostics().delta) && std::isfinite(ca.Diagnostics().low);
        }
        LoadPose(db, sc.firstFrame + 8, pose);
        ca.Apply(pose, 7, 1.0f, 1.0f / 60.0f); // bad stance
        const bool badDiag = !ca.Diagnostics().valid && ca.Diagnostics().dev == 0.0f && ca.Diagnostics().low == 0.0f;
        Expect("NaN pose -> unchanged + counted; NaN weight -> unchanged; NaN/inf/neg/huge dt finite; bad stance no-op",
               nanPose && nanW && fin && ca.Diagnostics().noReference >= 1);
        Expect("diagnostics not stale on NaN pose / missing reference (valid false, dev = low = 0)", nanDiag && badDiag);
    }
    // determinism + stance switch continuity (stanceBlendTime 0: the snap switch, devLow re-initialised; the pose here switches
    // loops in one frame, so the blended switch is covered by the crossfade block below)
    {
        const int32_t ll = ChestAlign::FindClip(db, "M_Neutral_Walk_Loop_LL");
        const int32_t cll = ChestAlign::FindClip(db, "M_Neutral_Crouch_Loop_LL");
        std::vector<float> r[2];
        float maxStep = 0.0f, switchLowErr = 1e9f, preSwitchGap = 0.0f;
        ChestAlignConfig cSnap = cfg;
        cSnap.stanceBlendTime = 0.0f;
        ca.SetConfig(cSnap);
        for (int run = 0; run < 2; ++run)
        {
            ca.Reset();
            std::vector<BoneTransform> pose;
            float prev = 0.0f;
            for (int i = 0; i < 240; ++i)
            {
                const bool crouch = i >= 120;
                const ClipRecord& c = db.Clips()[size_t(crouch ? cll : ll)];
                LoadPose(db, c.firstFrame + uint32_t(i) % c.loopPeriod, pose);
                ca.Apply(pose, crouch ? ChestAlign::kCrouch : ChestAlign::kStand, 1.0f, 1.0f / 60.0f);
                r[run].push_back(ca.Diagnostics().delta);
                if (run == 0 && i == 119)
                    preSwitchGap = std::fabs(ca.Diagnostics().low - ca.Diagnostics().dev);
                if (run == 0 && i == 120)
                    switchLowErr = std::fabs(ca.Diagnostics().low - ca.Diagnostics().dev);
                if (i > 0 && run == 0)
                    maxStep = std::max(maxStep, std::fabs(ca.Diagnostics().delta - prev));
                prev = ca.Diagnostics().delta;
            }
        }
        Expect("deterministic after Reset; stance switch stand->crouch delta step bounded", r[0] == r[1] && maxStep < 10.0f,
               Fmt("max per-frame delta step %.2f deg @60 Hz", maxStep));
        Expect("stanceBlendTime 0: stance switch re-initialises devLow to the new stance's dev", switchLowErr < 1e-3f,
               Fmt("|low - dev| at switch frame %.5f deg (frame before, stand: %.2f)", switchLowErr, preSwitchGap));
        ca.SetConfig(cfg);
    }
    // chest_repair: stance toggle while the pose crossfades (the matcher reaches the new stance's loops over several frames
    // after the input stance flips). Pose = per-bone blend of a stand and a crouch forward frame, alpha 0 -> 1 over 0.4 s
    // starting 2 frames after the stance flip, held, then back. Output chest twist vs the STAND reference must move between
    // its settled stand / crouch values without overshooting (the snap switch, stanceBlendTime 0, yawed the chest by the
    // reference offset ahead of the pose and overshot while devLow caught up).
    {
        auto blendPose = [&](float aAlpha, std::vector<BoneTransform>& aOut)
        {
            std::vector<BoneTransform> s, c;
            LoadPose(db, sc.firstFrame, s);
            LoadPose(db, cc.firstFrame, c);
            aOut.resize(s.size());
            for (size_t b = 0; b < s.size(); ++b)
            {
                math::Quat qc = c[b].q;
                if (math::QuatDot(s[b].q, qc) < 0.0f)
                    qc = math::QuatNeg(qc);
                aOut[b].p = {s[b].p.x + (c[b].p.x - s[b].p.x) * aAlpha, s[b].p.y + (c[b].p.y - s[b].p.y) * aAlpha,
                             s[b].p.z + (c[b].p.z - s[b].p.z) * aAlpha};
                aOut[b].q = math::QuatNormalize(math::Quat{s[b].q.x + (qc.x - s[b].q.x) * aAlpha, s[b].q.y + (qc.y - s[b].q.y) * aAlpha,
                                                           s[b].q.z + (qc.z - s[b].q.z) * aAlpha, s[b].q.w + (qc.w - s[b].q.w) * aAlpha});
            }
        };
        const math::Quat rs = ca.Reference(ChestAlign::kStand);
        struct Tog
        {
            float overshoot = 0.0f, maxStep = 0.0f, before = 0.0f, mid = 0.0f, after = 0.0f, blendAtEnd = -1.0f;
            int stuckFrames = -1;
            bool det = true, othersSame = true;
        };
        auto runToggle = [&](float aBlendTime, float aHz, bool aStuck)
        {
            Tog t;
            ChestAlignConfig cx = cfg;
            cx.stanceBlendTime = aBlendTime;
            ca.SetConfig(cx);
            const float dt = 1.0f / aHz;
            const int hold = int(1.0f * aHz + 0.5f), fade = int(0.4f * aHz + 0.5f), lag = 2;
            std::vector<float> outs[2];
            for (int run = 0; run < 2; ++run)
            {
                ca.Reset();
                std::vector<BoneTransform> pose, orig;
                for (int i = 0; i < 3 * hold; ++i)
                {
                    // stance: stand [0, hold), crouch [hold, 2 hold), stand [2 hold, 3 hold); pose alpha follows after `lag`
                    const int seg = i / hold, k = i - seg * hold - lag;
                    float alpha = seg == 1 ? 1.0f : 0.0f;
                    if (seg > 0)
                    {
                        const float f = std::fmin(std::fmax(float(k) / float(fade), 0.0f), 1.0f);
                        alpha = seg == 1 ? f : 1.0f - f;
                    }
                    if (aStuck)
                        alpha = 0.0f;
                    blendPose(alpha, pose);
                    orig = pose;
                    ca.Apply(pose, seg == 1 ? ChestAlign::kCrouch : ChestAlign::kStand, 1.0f, dt);
                    for (size_t b = 0; b < pose.size(); ++b)
                        if (int32_t(b) != ca.SpineBone() && std::memcmp(&pose[b], &orig[b], sizeof(BoneTransform)) != 0)
                            t.othersSame = false;
                    outs[run].push_back(ChestAlign::TwistZ(math::QuatMul(math::QuatNormalize(ca.Model(pose, ca.SpineBone()).q), math::QuatInv(rs))) *
                                        kRad2Deg);
                    if (aStuck && seg == 1 && t.stuckFrames < 0 && ca.Diagnostics().blend >= 1.0f)
                        t.stuckFrames = i - hold + 1;
                    if (i == 2 * hold - 1)
                        t.blendAtEnd = ca.Diagnostics().blend;
                }
            }
            t.det = outs[0] == outs[1];
            const std::vector<float>& o = outs[0];
            t.before = o[size_t(hold - 1)];
            t.mid = o[size_t(2 * hold - 1)];
            t.after = o.back();
            for (size_t i = 1; i < o.size(); ++i)
                t.maxStep = std::max(t.maxStep, std::fabs(o[i] - o[i - 1]));
            // overshoot beyond the settled values, both transitions
            for (int seg = 1; seg <= 2; ++seg)
            {
                const float a = seg == 1 ? t.before : t.mid, b = seg == 1 ? t.mid : t.after;
                const float lo = std::min(a, b), hi = std::max(a, b);
                for (int i = seg * hold; i < (seg + 1) * hold; ++i)
                    t.overshoot = std::max({t.overshoot, o[size_t(i)] - hi, lo - o[size_t(i)]});
            }
            ca.SetConfig(cfg);
            return t;
        };
        const float zs = ca.ReferenceHipsZ(ChestAlign::kStand), zc = ca.ReferenceHipsZ(ChestAlign::kCrouch);
        Expect("reference Hips heights: crouch lower than stand by >= 5 cm (pose-driven stance blend usable)", zs - zc >= 0.05f,
               Fmt("stand %.3f m, crouch %.3f m", zs, zc));
        for (float hz : rates)
        {
            const Tog nw = runToggle(cfg.stanceBlendTime, hz, false);
            const Tog old = runToggle(0.0f, hz, false);
            std::printf("crouch toggle @%.0f Hz: out twist vs stand ref stand %.2f -> crouch %.2f -> stand %.2f; overshoot %.2f deg (snap %.2f), "
                        "max step %.2f (snap %.2f)\n",
                        double(hz), double(nw.before), double(nw.mid), double(nw.after), double(nw.overshoot), double(old.overshoot),
                        double(nw.maxStep), double(old.maxStep));
            Expect(Fmt("crouch toggle while aiming @%.0f Hz: no chest yaw overshoot (<= 1.5 deg), snap switch shows the bug (>= 3 deg)", hz).c_str(),
                   nw.overshoot <= 1.5f && old.overshoot >= 3.0f && nw.det && nw.othersSame && nw.blendAtEnd == 1.0f,
                   Fmt("overshoot %.2f deg (snap %.2f), blend at the end of crouch %.3f", nw.overshoot, old.overshoot, nw.blendAtEnd));
            const Tog st = runToggle(cfg.stanceBlendTime, hz, true);
            const int maxFrames = int(std::ceil(cfg.stanceBlendTime * hz)) + 1;
            Expect(Fmt("pose that never crouches @%.0f Hz: the reference blend still completes within stanceBlendTime", hz).c_str(),
                   st.stuckFrames > 0 && st.stuckFrames <= maxFrames && st.det,
                   Fmt("blend 1 after %.0f frames (limit %.0f)", st.stuckFrames, maxFrames));
        }
        // no switch: the blend stays exactly at the stance (single-reference path), counter untouched
        {
            ca.Reset();
            const uint64_t b0 = ca.Diagnostics().blendUpdates;
            std::vector<BoneTransform> pose;
            bool atStance = true;
            for (int i = 0; i < 60; ++i)
            {
                LoadPose(db, cc.firstFrame + uint32_t(i) % cc.loopPeriod, pose);
                ca.Apply(pose, ChestAlign::kCrouch, 1.0f, 1.0f / 60.0f);
                atStance = atStance && ca.Diagnostics().blend == 1.0f;
            }
            Expect("no stance switch -> blend == stance on every update, no blend updates counted",
                   atStance && ca.Diagnostics().blendUpdates == b0);
        }
        {
            ChestAlignConfig bad = cfg;
            bad.stanceBlendTime = -0.1f;
            const bool r1 = !ca.SetConfig(bad);
            bad.stanceBlendTime = std::numeric_limits<float>::quiet_NaN();
            const bool r2 = !ca.SetConfig(bad);
            bad.stanceBlendTime = 5.5f;
            const bool r3 = !ca.SetConfig(bad);
            Expect("stanceBlendTime NaN / negative / > 5 s rejected", r1 && r2 && r3 && ca.Config().stanceBlendTime == cfg.stanceBlendTime);
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

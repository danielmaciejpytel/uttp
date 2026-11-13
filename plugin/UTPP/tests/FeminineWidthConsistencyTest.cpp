// Feminine shoulder width consistency (2026-11-02): replays the real woman database + overlay file through the actual
// PoseEvaluator (LiveCrossfade, the production base policy) for every clip x weapon pose and measures the upper-arm socket
// width along the Spine3 lateral before / after the feminine pass.
//   argv: woman.utppmm woman.utppov style_weights.txt [report|legacy]
//   (no mode)  = assertions on the continuous pass, report = the same plus the full table, legacy = baseline table of the
//                all-or-nothing pass (no assertions; the "before" numbers of the analysis).
// Jump metric = frame-to-frame change of the NARROWING (width before - width after), not of the raw width: the raw width moves with
// the animation, the narrowing is the part this pass owns. Pops are what the player sees as the shoulders snapping back wide.
#include "../src/mm/MotionPose.hpp"
#include "TestFailFast.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
using namespace utpp;
using namespace utpp::mm;

namespace
{
struct Cfg { const char* name; ov::PoseOverlay overlay; bool ready, aim; };
const Cfg kCfgs[] = {
    {"Default", ov::PoseOverlay::Default, false, false},   {"P1H-Ready", ov::PoseOverlay::Pistol1H, true, false},
    {"P1H-Aim", ov::PoseOverlay::Pistol1H, true, true},    {"P2H-Ready", ov::PoseOverlay::Pistol2H, true, false},
    {"P2H-Aim", ov::PoseOverlay::Pistol2H, true, true},    {"Rifle-Ready", ov::PoseOverlay::Rifle, true, false},
    {"Rifle-Aim", ov::PoseOverlay::Rifle, true, true}};
float gAmount = 0.02f; // m per side of the pass under test (main)
const char* kReason[] = {"none", "zero", "invalid", "crossed", "scale", "reach", "solve", "contact"};

struct Agg
{
    int frames = 0, applied = 0, partial = 0, pops3 = 0, pops10 = 0;
    int reason[8] = {};
    double sumBefore = 0, sumAfter = 0, sumNarrow = 0;
    float minAfter = 1e9f, maxAfter = -1e9f, minNarrow = 1e9f, maxNarrow = -1e9f, maxStep = 0, maxStepAfter = 0;
    float maxRise = 0, maxDrop = 0; // signed steps of the narrowing: rise = posture returning, drop = posture lost
    int drops5 = 0;
    float maxContact = 0; // largest hand / weapon displacement of an applied pass (m)
    double sumAsym = 0;   // sum of |wL - wR| / 2 (shoulder midline shift = amount x this)
    float maxElbowMove = 0, maxElbowJump = 0, maxElbowFlex = 0; // elbow evidence: displacement (m), frame-to-frame jump (m), bend change (deg)
    float maxDrift = 0; // largest socket drift toward the hand (m, femshoulder REACH DRIFT)
    void Add(const motionpose::FrameInfo::FemShoulder& f, bool haveStep, float stepNarrow, float stepAfter)
    {
        ++frames;
        applied += f.applied;
        maxDrift = std::max(maxDrift, std::max(f.drift[0], f.drift[1]));
        if (f.applied) maxContact = std::max(maxContact, std::max(f.handError, f.weaponError));
        const float narrow = f.widthBefore - f.widthAfter;
        partial += f.applied && (std::fmax(f.weight[0], f.weight[1]) < f.requested - 1e-3f || std::fmin(f.weight[0], f.weight[1]) < f.requested - 1e-3f);
        ++reason[std::min(int(f.reason), 7)];
        sumBefore += f.widthBefore; sumAfter += f.widthAfter; sumNarrow += narrow;
        minAfter = std::min(minAfter, f.widthAfter); maxAfter = std::max(maxAfter, f.widthAfter);
        minNarrow = std::min(minNarrow, narrow); maxNarrow = std::max(maxNarrow, narrow);
        if (haveStep)
        {
            maxStep = std::max(maxStep, stepNarrow); maxStepAfter = std::max(maxStepAfter, stepAfter);
            pops3 += stepNarrow > 0.003f; pops10 += stepNarrow > 0.010f;
        }
    }
    void Merge(const Agg& o)
    {
        frames += o.frames; applied += o.applied; partial += o.partial; pops3 += o.pops3; pops10 += o.pops10;
        for (int i = 0; i < 8; ++i) reason[i] += o.reason[i];
        sumBefore += o.sumBefore; sumAfter += o.sumAfter; sumNarrow += o.sumNarrow;
        minAfter = std::min(minAfter, o.minAfter); maxAfter = std::max(maxAfter, o.maxAfter);
        minNarrow = std::min(minNarrow, o.minNarrow); maxNarrow = std::max(maxNarrow, o.maxNarrow);
        maxStep = std::max(maxStep, o.maxStep); maxStepAfter = std::max(maxStepAfter, o.maxStepAfter);
        maxRise = std::max(maxRise, o.maxRise); maxDrop = std::max(maxDrop, o.maxDrop); drops5 += o.drops5;
        maxContact = std::max(maxContact, o.maxContact);
        sumAsym += o.sumAsym; maxElbowMove = std::max(maxElbowMove, o.maxElbowMove); maxElbowJump = std::max(maxElbowJump, o.maxElbowJump);
        maxElbowFlex = std::max(maxElbowFlex, o.maxElbowFlex);
        maxDrift = std::max(maxDrift, o.maxDrift);
    }
    void Elbow(const motionpose::FrameInfo::FemShoulder& f, const float prevMove[2], bool have)
    {
        sumAsym += 0.5 * std::fabs(f.weight[0] - f.weight[1]);
        for (int side = 0; side < 2; ++side)
        {
            maxElbowMove = std::max(maxElbowMove, f.elbowMove[side]);
            maxElbowFlex = std::max(maxElbowFlex, std::fabs(f.elbowFlexDeg[side]));
            if (have) maxElbowJump = std::max(maxElbowJump, std::fabs(f.elbowMove[side] - prevMove[side]));
        }
    }
    void Signed(float step) // narrowing(t) - narrowing(t-1)
    {
        maxRise = std::max(maxRise, step); maxDrop = std::max(maxDrop, -step); drops5 += step < -0.005f;
    }
};

std::string Group(std::string_view n)
{
    const auto has = [&](const char* s) { return n.find(s) != std::string_view::npos; };
    if (has("Sprint")) return "sprint";
    if (has("Jump") || has("Fall") || has("Land")) return "air";
    if (has("Crouch")) return has("Transition") ? "crouch-trans" : "crouch";
    if (has("Transition")) return "transition";
    if (has("Turn") || has("Spin") || has("Reface") || has("Pivot")) return "turn";
    if (has("Stop")) return "stop";
    if (has("Start")) return "start";
    if (has("Idle")) return "idle";
    if (has("Walk")) return "walk";
    if (has("Run")) return "run";
    return "other";
}
void Print(const char* group, const char* cfg, const Agg& a)
{
    if (!a.frames) return;
    std::printf("%-13s %-11s n=%6d app=%5.1f%% part=%5.1f%% cm[b=%5.2f a=%5.2f min=%5.2f max=%5.2f] narrow[mean=%4.2f min=%4.2f max=%4.2f] step[max=%5.2fmm "
                "p3=%d p10=%d] reasons[",
                group, cfg, a.frames, 100.0 * a.applied / a.frames, 100.0 * a.partial / a.frames, 100 * a.sumBefore / a.frames,
                100 * a.sumAfter / a.frames, 100 * a.minAfter, 100 * a.maxAfter, 100 * a.sumNarrow / a.frames, 100 * a.minNarrow,
                100 * a.maxNarrow, 1000 * a.maxStep, a.pops3, a.pops10);
    for (int i = 1; i < 8; ++i)
        if (a.reason[i]) std::printf("%s=%d ", kReason[i], a.reason[i]);
    std::printf("] asym=%.1fmm elbow[mv=%.1f jump=%.1f flex=%.1fdeg]", 1000 * gAmount * a.sumAsym / a.frames, 1000 * a.maxElbowMove, 1000 * a.maxElbowJump, a.maxElbowFlex);
    std::puts("");
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 4) return 2;
    const std::string mode = argc > 4 ? argv[4] : "";
    const bool legacy = mode == "legacy", report = mode == "report" || legacy;
    float amount = motionpose::kFeminineShoulderAmount;
    if (argc > 5) amount = std::strtof(argv[5], nullptr);
    std::string error;
    auto db = std::make_unique<motion::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!db->LoadFromFile(argv[1], motion::kRigIdentityWoman, error) ||
        !ovf->LoadFromFile(argv[2], ov::kRigIdentityWomanOv, error)) { std::puts(error.c_str()); return 2; }
    std::ifstream stream(argv[3]);
    std::stringstream text; text << stream.rdbuf();
    ov::StyleFamilyData style;
    if (!style.Parse(text.str(), &error)) return 2;
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    if (!ev->Init(db.get(), ovf.get(), &style, 2, error, motionpose::BaseBlendPolicy::LiveCrossfade)) { std::puts(error.c_str()); return 2; }
    gAmount = legacy ? 0.020f : amount;
    ev->SetFeminineShoulders(gAmount, !legacy);
    const float fps = db->Header().fps > 0 ? db->Header().fps : 30.0f;
    constexpr float dt = 1.0f / 60;
    std::vector<BoneTransform> output(db->NumBones());
    motionpose::FrameInfo info;
    int pass = 0, fail = 0;
    const auto check = [&](bool ok, const char* label) { (ok ? pass : fail)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", label); if (!ok) UtppFailFastNote(std::string("FAIL ") + label); };

    const auto inputsFor = [&](const Cfg& c, std::string_view clip) {
        motionpose::OverlayFrameInputs in;
        in.bodyWoman = true; in.machine.style = ov::Style::Feminine; in.machine.overlay = c.overlay;
        in.machine.wantsReady = c.ready; in.machine.wantsToAim = c.aim;
        in.capturePoseDiagnostics = false; in.gripSkip = true;
        in.authoredTwoHandGrip = c.overlay == ov::PoseOverlay::Pistol2H || c.overlay == ov::PoseOverlay::Rifle;
        in.machine.crouch = clip.find("Crouch") != std::string_view::npos;
        in.machine.sprinting = in.machine.wantsToSprint = clip.find("Sprint") != std::string_view::npos;
        in.machine.moving = clip.find("Idle") == std::string_view::npos && clip.find("Stop") == std::string_view::npos;
        in.machine.hasVelocity = in.machine.moving;
        in.machine.airW = (clip.find("Jump") != std::string_view::npos || clip.find("Fall") != std::string_view::npos) ? 1.0f : 0.0f;
        return in;
    };

    // ---- 1. every clip x every weapon pose: frozen warm-up (overlay settles), then the clip plays (<= 3 s) ----------
    std::map<std::pair<std::string, int>, Agg> table;
    std::map<std::string, int> failFrames;
    int excluded = 0;
    Agg all[sizeof(kCfgs) / sizeof(kCfgs[0])];
    int clips = 0;
    bool finite = true;
    for (uint32_t c = 0; c < db->NumClips(); ++c)
    {
        const std::string_view name = db->ClipName(c);
        if (name.rfind("M_", 0) != 0 && name.rfind("F_", 0) != 0) continue; // locomotion clips only
        if (name.find("Dance") != std::string_view::npos) continue;
        if (db->Clip(c).flags & motion::kClipAdded) { ++excluded; continue; } // motion / motion clips only (project rule)
        const float duration = db->Clip(c).numFrames / fps;
        if (!(duration > 0.2f)) continue;
        ++clips;
        const std::string group = Group(name);
        for (size_t k = 0; k < sizeof(kCfgs) / sizeof(kCfgs[0]); ++k)
        {
            ev->Reset();
            auto in = inputsFor(kCfgs[k], name);
            motion::MatchResult match; match.clip = int(c); match.clipTime = 0.0f;
            for (int i = 0; i < 45; ++i) finite = ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) && finite;
            Agg& a = table[{group, int(k)}];
            float prevNarrow = 0, prevAfter = 0, prevMove[2] = {}; bool have = false;
            const float end = std::min(duration - 0.05f, 3.0f);
            for (float t = 0; t < end; t += dt)
            {
                match.clipTime = t;
                finite = ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) && finite;
                if (!info.fem.ran) continue;
                if (mode == "fails" && !(info.fem.applied && info.fem.reason == femshoulder::Reason::None) && info.fem.requested > 0.5f)
                    ++failFrames[std::string(name) + " " + kCfgs[k].name + " " + kReason[int(info.fem.reason)]];
                if (mode == "fails" && (info.fem.elbowMove[0] > 0.04f || info.fem.elbowMove[1] > 0.04f))
                    ++failFrames[std::string("ELBOW>40mm ") + std::string(name) + " " + kCfgs[k].name];
                const float narrow = info.fem.widthBefore - info.fem.widthAfter;
                a.Add(info.fem, have, std::fabs(narrow - prevNarrow), std::fabs(info.fem.widthAfter - prevAfter));
                all[k].Add(info.fem, false, 0, 0);
                a.Elbow(info.fem, prevMove, have); all[k].Elbow(info.fem, prevMove, have);
                prevMove[0] = info.fem.elbowMove[0]; prevMove[1] = info.fem.elbowMove[1];
                if (have) all[k].Signed(narrow - prevNarrow);
                if (have) all[k].maxStep = std::max(all[k].maxStep, std::fabs(narrow - prevNarrow));
                if (have) { all[k].pops3 += std::fabs(narrow - prevNarrow) > 0.003f; all[k].pops10 += std::fabs(narrow - prevNarrow) > 0.010f; }
                if (mode == "fails" && have && std::fabs(narrow - prevNarrow) > 0.006f)
                    std::printf("STEP %s %s t=%.3f narrow %.2f->%.2f cm w[%.2f %.2f] feas[%.2f %.2f] req %.2f reason=%s\n", std::string(name).c_str(), kCfgs[k].name, t,
                                prevNarrow * 100, narrow * 100, info.fem.weight[0], info.fem.weight[1], info.fem.feasible[0], info.fem.feasible[1],
                                info.fem.requested, kReason[int(info.fem.reason)]);
                prevNarrow = narrow; prevAfter = info.fem.widthAfter; have = true;
            }
        }
    }
    for (const auto& [k, n] : failFrames) if (n >= 8) std::printf("DIAG %-70s %d\n", k.c_str(), n);
    std::printf("excluded non-Neutral clips=%d\n", excluded);
    std::printf("clips=%d mode=%s amount=%.4f\n", clips, legacy ? "legacy" : "continuous", legacy ? 0.020f : amount);
    if (report)
        for (const auto& [key, agg] : table) Print(key.first.c_str(), kCfgs[key.second].name, agg);
    Agg total;
    for (size_t k = 0; k < sizeof(kCfgs) / sizeof(kCfgs[0]); ++k) { Print("ALL", kCfgs[k].name, all[k]); total.Merge(all[k]); }
    Print("ALL", "ALL", total);
    check(finite, "all replayed frames finite");

    // ---- 2. scripted transitions: aim toggles, weapon swaps, locomotion chain with MM-style blends ------------------
    struct Ev { float t; const char* clip; int cfg; }; // clip nullptr = keep; cfg index into kCfgs
    const auto clipIndex = [&](const char* n) {
        for (uint32_t c = 0; c < db->NumClips(); ++c) if (db->ClipName(c) == n) return int(c);
        return -1;
    };
    struct Script { const char* label; std::vector<Ev> ev; float end; };
    std::vector<Script> scripts;
    for (int base : {1, 3, 5}) // P1H / P2H / Rifle: Ready <-> Aim every 2 s on a run loop
        scripts.push_back({"aim-toggle", {{0, "M_Neutral_Run_Loop_F", base}, {2, nullptr, base + 1}, {4, nullptr, base}, {6, nullptr, base + 1}, {8, nullptr, base}}, 10});
    scripts.push_back({"weapon-swap", {{0, "M_Neutral_Walk_Loop_F", 0}, {1.5f, nullptr, 1}, {3, nullptr, 2}, {4.5f, nullptr, 4}, {6, nullptr, 6}, {7.5f, nullptr, 3}, {9, nullptr, 0}}, 10.5f});
    for (int cfg : {0, 2, 4, 6, 1, 3})
        scripts.push_back({"loco-chain", {{0, "M_Neutral_Stand_Idle_Loop", cfg}, {1.5f, "M_Neutral_Run_Start_F_Lfoot", cfg}, {2.5f, "M_Neutral_Run_Loop_F", cfg},
            {4.5f, "M_Neutral_Transition_Run_to_Sprint_Rfoot", cfg}, {5.5f, "M_Neutral_Sprint_Loop_F", cfg}, {7, "M_Neutral_Transition_Sprint_to_Run_Rfoot", cfg},
            {8, "M_Neutral_Run_Loop_F", cfg}, {9.5f, "M_Neutral_Transition_Stand_to_Crouch", cfg}, {10.5f, "M_Neutral_Crouch_Idle_Loop", cfg},
            {12, "M_Neutral_Crouch_Loop_F", cfg}, {14, "M_Neutral_Transition_Crouch_to_Stand", cfg}, {15, "M_Neutral_Jump_F_Start_Run_Lfoot", cfg},
            {15.8f, "M_Neutral_Jump_Loop_Fall", cfg}, {17, "M_Neutral_Jump_F_Land_Run_Light_Lfoot", cfg}, {18.5f, "M_Neutral_Stand_Idle_Loop", cfg}}, 20});
    Agg scenario;
    float scenarioMinNarrow = 1e9f;
    for (const auto& sc : scripts)
    {
        ev->Reset();
        int clip = -1, cfg = 0; float clipStart = 0, prevNarrow = 0, prevMove[2] = {}; bool have = false;
        motion::MatchResult match;
        Agg a;
        size_t next = 0;
        std::string clipName;
        for (float t = 0; t < sc.end; t += dt)
        {
            bool switched = false; const int oldClip = clip; const float oldTime = match.clipTime;
            while (next < sc.ev.size() && sc.ev[next].t <= t + 1e-6f)
            {
                cfg = sc.ev[next].cfg;
                if (sc.ev[next].clip) { const int ci = clipIndex(sc.ev[next].clip); if (ci < 0) return 2; if (ci != clip) { clip = ci; clipStart = t; clipName = sc.ev[next].clip; switched = true; } }
                ++next;
            }
            const float duration = db->Clip(clip).numFrames / fps;
            float ct = t - clipStart;
            const bool loops = (db->Clip(clip).flags & (motion::kClipLoop | motion::kClipNameLoop)) != 0;
            ct = loops ? std::fmod(ct, std::fmax(duration - 0.05f, 0.1f)) : std::fmin(ct, duration - 0.05f);
            match.clip = clip; match.clipTime = ct; match.jumped = switched;
            match.blend = motion::BlendRequest{};
            if (switched && oldClip >= 0)
            {
                match.blend.valid = true; match.blend.blendTime = 0.25f; match.blend.fromClip = oldClip; match.blend.fromTime = oldTime;
                match.blend.toClip = clip; match.blend.toTime = ct;
            }
            auto in = inputsFor(kCfgs[cfg], clipName);
            finite = ev->Evaluate(dt, match, in, output.data(), uint32_t(output.size()), &info) && finite;
            if (!info.fem.ran || t < 0.5f) continue; // the first half second lets the overlay machine settle
            const float narrow = info.fem.widthBefore - info.fem.widthAfter;
            a.Add(info.fem, have, std::fabs(narrow - prevNarrow), 0);
            if (have) a.Signed(narrow - prevNarrow);
            a.Elbow(info.fem, prevMove, have);
            prevMove[0] = info.fem.elbowMove[0]; prevMove[1] = info.fem.elbowMove[1];
            if (mode == "fails" && have && std::fabs(narrow - prevNarrow) > 0.006f)
                std::printf("SCSTEP %s cfg=%s t=%.3f %s narrow %.2f->%.2f cm w[%.2f %.2f] feas[%.2f %.2f] reason=%s\n", sc.label, kCfgs[cfg].name, t, clipName.c_str(),
                            prevNarrow * 100, narrow * 100, info.fem.weight[0], info.fem.weight[1], info.fem.feasible[0], info.fem.feasible[1], kReason[int(info.fem.reason)]);
            prevNarrow = narrow; have = true;
        }
        if (report) Print(sc.label, kCfgs[cfg].name, a);
        scenario.Merge(a);
        scenarioMinNarrow = std::min(scenarioMinNarrow, a.minNarrow);
    }
    std::printf("SCENARIOS frames=%d applied=%d narrowMin=%.2fcm maxRise=%.2fmm maxDrop=%.2fmm drops>5mm=%d\n", scenario.frames, scenario.applied,
                scenarioMinNarrow * 100, scenario.maxRise * 1000, scenario.maxDrop * 1000, scenario.drops5);
    check(finite, "all scripted frames finite");
    if (legacy) { std::printf("Feminine width %dPASS %dFAIL\n", pass, fail); return 0; }

    // ---- 2b. steady aim while running / walking (dll-shoulder-brake 2026-11-09): the gun hand swings out once per stride; at wide
    // settings the reach window then takes the gun side down every ~0.6 s. Pulse = the narrowing loses > 5 mm within 1/30 s (any frame
    // rate) AFTER the aim has settled (from 1.5 s on); the first loss at the start of the aim is the forced, one-off drop. RETURN HOLD keeps
    // one steady level instead of giving the posture back between strides. Frame rates 30 / 60 / 144.
    int pulseMax = 0;
    float pulseRiseMax = 0.0f;
    for (const char* loopClip : {"M_Neutral_Run_Loop_F", "M_Neutral_Walk_Loop_F"})
        for (const float pdt : {1.0f / 30, 1.0f / 60, 1.0f / 144})
        {
            const int ci = clipIndex(loopClip);
            if (ci < 0) return 2;
            const float duration = db->Clip(ci).numFrames / fps;
            ev->Reset();
            const auto in = inputsFor(kCfgs[2], loopClip); // P1H-Aim
            motion::MatchResult match; match.clip = ci; match.clipTime = 0.0f;
            const int lag = std::max(1, int(std::lround((1.0f / 30) / pdt)));
            std::vector<float> narrowHist;
            int pulses = 0;
            float rise = 0.0f; // summed return (upward steps of the narrowing) in the steady part, m
            for (float t = 0; t < 6.0f; t += pdt)
            {
                match.clipTime = std::fmod(t, std::fmax(duration - 0.05f, 0.1f));
                finite = ev->Evaluate(pdt, match, in, output.data(), uint32_t(output.size()), &info) && finite;
                if (!info.fem.ran) continue;
                const float narrow = info.fem.widthBefore - info.fem.widthAfter;
                if (t >= 1.5f && narrowHist.size() >= size_t(lag))
                {
                    pulses += narrow - narrowHist[narrowHist.size() - lag] < -0.005f;
                    rise += std::fmax(0.0f, narrow - narrowHist.back());
                }
                narrowHist.push_back(narrow);
            }
            std::printf("PULSE %s P1H-Aim dt=1/%.0f: drops>5mm/33ms=%d returned=%.1fmm (1.5-6 s)\n", loopClip, 1.0f / pdt, pulses, rise * 1000);
            pulseMax = std::max(pulseMax, pulses);
            pulseRiseMax = std::max(pulseRiseMax, rise);
        }

    // ---- 3. assertions (continuous pass) -----------------------------------------------------------------------------
    const float total2 = 2.0f * amount; // full posture: both sides at weight 1
    const float riseBound = total2 * (dt / motionpose::kFeminineRiseSeconds) * 1.15f + 0.0002f; // rise limiter + float noise
    check(total.applied == total.frames && scenario.applied == scenario.frames, "the pass applies on every frame (no all-or-nothing fail-closed frame)");
    check(total.minNarrow > 0.15f * total2 && scenarioMinNarrow > 0.15f * total2, "no frame loses more than 85 percent of the posture");
    check(total.maxRise <= riseBound && scenario.maxRise <= riseBound, "the posture returns at the limited rate only (no upward pop)");
    check(total.maxDrop < 0.6f * total2 && scenario.maxDrop < 0.6f * total2, "no single-frame loss of more than 60 percent of the posture");
    check(scenario.drops5 * 200 <= std::max(scenario.frames, 1) && total.drops5 * 1000 <= total.frames, "drops above 5 mm stay rare (reach cliffs only)");
    std::printf("max hand/weapon displacement of an applied pass: %.4f mm (sweep) %.4f mm (scripts)\n", total.maxContact * 1000, scenario.maxContact * 1000);
    check(total.maxContact <= 1e-4f && scenario.maxContact <= 1e-4f, "hand and weapon contacts stay within 0.1 mm of the unnarrowed pose");
    for (size_t k = 3; k < 7; ++k) // P2H / Rifle: steady two-hand poses keep (nearly) the full posture on all motion locomotion
        check(all[k].partial * 100 <= all[k].frames, "two-hand poses are reach-limited on at most 1 percent of the frames");
    // REACH DRIFT (2026-11-09): above the shipped default a socket may follow its hand by at most the extra amount; none up to it.
    const float driftAllowance = femshoulder::DriftAllowance(amount);
    std::printf("max socket drift toward the hand: %.2f mm (sweep) %.2f mm (scripts), allowance %.2f mm\n", total.maxDrift * 1000,
                scenario.maxDrift * 1000, driftAllowance * 1000);
    check(total.maxDrift <= driftAllowance + 1e-6f && scenario.maxDrift <= driftAllowance + 1e-6f,
          "the socket drift toward the hand stays within its allowance (none up to the shipped default)");
    std::printf("steady aim pulses: max drops>5mm/33ms=%d, max returned %.1f mm\n", pulseMax, pulseRiseMax * 1000);
    check(pulseMax == 0, "steady aim while running / walking: the shoulder does not pulse in step (no loss > 5 mm per 1/30 s after 1.5 s)");
    std::printf("Feminine width %dPASS %dFAIL\n", pass, fail);
    return fail ? 1 : 0;
}

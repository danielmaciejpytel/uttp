// Rifle grip + stock 2026-11-10 (offline, local-only output): copy of weapons_eval.cpp (weapon films 2026-11-09) with three additions:
//   trailing options  --man (Male V data: rig identity Man, bodyWoman false, style Masculine, maleIdlePistolStance; no feminine pass),
//                     --profile N (recoil profile id of the equipped weapon = the driver's g_recoilProfile; with GRIPSTOCK_NEW the
//                     evaluator gets the support grip / stock entry of that weapon exactly like MotionDriver.cpp: Rifle overlay only),
//   extra diag columns (GRIPSTOCK_NEW: support grip weight, stock weight / shift, twist, reach shift / need, stock gap);
//   --nostock (GRIPSTOCK_NEW): the entry without its stock pass (Copperhead decision study 2026-11-10);
//   --maxshift m (GRIPSTOCK_NEW): the entry's reach-shift cap overridden (Hercules cap comparison 2026-11-10).  Built twice by gripstock_build_eval.cmd (before / after).
//   gripstock_eval list  <db> <ov> <style> [--man]                          -> clip names / durations / bone names on stdout
//   gripstock_eval run   <db> <ov> <style> <recoil_dir> <steps.txt> <out_prefix> [--man] [--profile N]
// Original header of weapons_eval.cpp:
// Weapon films 2026-11-09 (offline, local-only output): drives the PRODUCTION pose evaluator (src/mm/MotionPose.hpp, header-only, unchanged)
// with a scripted per-step timeline and writes the composed overlay-skeleton locals for skinning / rendering.
// steps.txt: one row per evaluator step (the caller decides clips, clip times, transitions, aim requests, shots):
//   dt clip clipTime jumped blendTime overlay wantsReady wantsToAim moving hasVelocity shot recoilFile femMode femAmount haveAim tx ty tz emit
//   overlay: 0 Default, 1 Rifle, 2 Pistol1H, 3 Pistol2H.  shot: 1 = new authored recoil shot this step (Player::Trigger, as MotionDriver.cpp
//   authored route).  recoilFile: file name inside recoil_dir ('-' = none).  femMode: 0 = SetFeminineShoulders(femAmount) (continuous
//   production pass); 1 = femAmount * (weight of the Pistol2H Relaxed poses of the machine in THIS step) - offline emulation of a
//   "low 2H pose only" variant, NOT DLL logic.  t = aim target point in the actor/pose frame (+Y forward, Z up, m); the aim angles use the
//   driver's AimAnglesFromModel (origin z 1.4), the barrel loop gets the same point (mm.aimAtCrosshair default on).  emit: 1 = write this step.
// Recoil admission / weight = MotionDriver.cpp authored route: admitted at the first step the evaluator reports shootApplied, 50 ms smoothstep
// admission, x Player::TailWeight, exit weight 1 (the weapon stays in the domain).  Binding {1,1,1}; limits recoil_profile::LimitsOf(id).
// Outputs: <out_prefix>.bones.txt (n, name parent per bone, overlay skeleton order), <out_prefix>.pose.bin (float32 per emitted step:
// n x [tx ty tz qx qy qz qw] locals), <out_prefix>.diag.csv (per emitted step), <out_prefix>.root.bin (float32 per emitted step: raw clip
// sample of the first two db bones, [tx ty tz qx qy qz qw] x 2, db order - root motion source for walking).
#include "../../plugin/UTPP/src/mm/MotionPose.hpp"
#include "../../plugin/UTPP/src/mm/RecoilProfiles.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp;
using namespace utpp::mm;

static bool ReadAll(const char* path, std::string& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

struct Step
{
    float dt = 0, clipTime = 0, blendTime = 0, femAmount = 0, tx = 0, ty = 0, tz = 0;
    std::string clip, recoilFile;
    int jumped = 0, overlay = 0, wantsReady = 0, wantsToAim = 0, moving = 0, hasVelocity = 0, shot = 0, femMode = 0, haveAim = 0, emit = 0, handback = 0; // handback: optional 20th column (gripstock reload emulation: hand-back = no authored grip)
};

static int FindClip(const v2::MotionDatabase& db, const std::string& name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name) return static_cast<int>(i);
    return -1;
}

static int ProfileOfFile(const std::string& file)
{
    for (int32_t id = 1; id < recoil_profile::kSlots; ++id)
    {
        const auto& e = recoil_profile::kTable[static_cast<size_t>(id)];
        if (!e.family) continue;
        if (file == e.shoulder || file == e.ironsight) return id;
    }
    return recoil_profile::kNone;
}

int main(int argc, char** argv)
{
    if (argc < 5) { std::fprintf(stderr, "usage: list|run ...\n"); return 2; }
    const std::string mode = argv[1];
    std::string err;
    bool man = false, noStock = false;
    float maxShift = -1.0f; // --maxshift m: the entry's reach-shift cap overridden (decision study; < 0 = the table's value)
    int32_t profile = 0;
    int nargs = argc;
    for (int i = 2; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--man") { man = true; nargs = std::min(nargs, i); }
        else if (a == "--nostock") { noStock = true; nargs = std::min(nargs, i); } // decision study: the weapon's entry without its stock pass
        else if (a == "--maxshift" && i + 1 < argc) { maxShift = static_cast<float>(std::atof(argv[i + 1])); nargs = std::min(nargs, i); ++i; }
        else if (a == "--profile" && i + 1 < argc) { profile = std::atoi(argv[i + 1]); nargs = std::min(nargs, i); ++i; }
    }
    auto db = std::make_unique<v2::MotionDatabase>();
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!db->LoadFromFile(argv[2], man ? v2::kRigIdentityMan : v2::kRigIdentityWoman, err)) { std::fprintf(stderr, "db load failed: %s\n", err.c_str()); return 3; }
    if (!ovf->LoadFromFile(argv[3], man ? ov::kRigIdentityManOv : ov::kRigIdentityWomanOv, err)) { std::fprintf(stderr, "ov load failed: %s\n", err.c_str()); return 3; }
    std::string styleText;
    ov::StyleFamilyData style;
    const bool styleOk = ReadAll(argv[4], styleText) && style.Parse(styleText, &err);
    if (!styleOk) { std::fprintf(stderr, "style parse failed: %s\n", err.c_str()); return 3; }
    const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
    if (mode == "list")
    {
        std::printf("DB bones %u clips %u fps %.3f\n", db->NumBones(), db->NumClips(), fps);
        for (uint32_t i = 0; i < db->NumClips(); ++i)
            std::printf("CLIP %u %s %.4f\n", i, std::string(db->ClipName(i)).c_str(), static_cast<float>(db->Clip(i).numFrames - 1) / fps);
        const auto& sk = ovf->Skel();
        for (int i = 0; i < sk.n; ++i) std::printf("OVBONE %d %s %d\n", i, std::string(ovf->BoneName(i)).c_str(), sk.parent[i]);
        return 0;
    }
    if (mode != "run" || nargs != 8) { std::fprintf(stderr, "bad args\n"); return 2; }
    const std::string recoilDir = argv[5];
    std::vector<Step> steps;
    {
        std::ifstream in(argv[6]);
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream r(line);
            Step s;
            if (!(r >> s.dt >> s.clip >> s.clipTime >> s.jumped >> s.blendTime >> s.overlay >> s.wantsReady >> s.wantsToAim >> s.moving >>
                  s.hasVelocity >> s.shot >> s.recoilFile >> s.femMode >> s.femAmount >> s.haveAim >> s.tx >> s.ty >> s.tz >> s.emit))
            { std::fprintf(stderr, "bad step row: %s\n", line.c_str()); return 4; }
            if (!(r >> s.handback)) s.handback = 0;
            steps.push_back(s);
        }
    }
    if (steps.empty()) return 4;
    auto ev = std::make_unique<motionpose::PoseEvaluator>();
    if (!ev->Init(db.get(), ovf.get(), &style, 2, err, motionpose::BaseBlendPolicy::LiveCrossfade))
    { std::fprintf(stderr, "evaluator init failed: %s\n", err.c_str()); return 3; }
    // production default (MotionDriver.cpp g_femShoulderM = kFeminineShoulderAmount, continuous); the steps override the amount per step
    ev->SetFeminineShoulders(motionpose::kFeminineShoulderAmount);
#if defined(GRIPSTOCK_NEW)
    const sgrip::Entry* support = sgrip::Of(profile); // MotionDriver.cpp: the equipped weapon's entry, nullptr = no game data
    static sgrip::Entry noStockEntry{};
    if (support && (noStock || maxShift >= 0.0f)) // --nostock: hand priority (stockY 0 = stock pass off); --maxshift: other cap
    {
        noStockEntry = *support;
        if (noStock)
            noStockEntry.stockY = 0.0f;
        if (maxShift >= 0.0f)
            noStockEntry.maxShift = maxShift;
        support = &noStockEntry;
    }
    std::printf("PROFILE %d support %s%s, reach-shift cap %.3f m\n", profile, support ? "yes" : "no", noStock ? " (no stock pass)" : "",
                support ? support->maxShift : 0.0f);
#else
    std::printf("PROFILE %d (before build: no support grip input)\n", profile);
#endif
    const auto& sk = ev->Skel();
    const std::string prefix = argv[7];
    {
        std::ofstream b(prefix + ".bones.txt");
        b << sk.n << "\n";
        for (int i = 0; i < sk.n; ++i) b << ovf->BoneName(i) << ' ' << sk.parent[i] << "\n";
    }
    std::ofstream pose(prefix + ".pose.bin", std::ios::binary), root(prefix + ".root.bin", std::ios::binary);
    std::ofstream diag(prefix + ".diag.csv");
    diag << "step,time,clip,clipTime,femRan,femApplied,femReason,femRequested,femAmount,femWL,femWR,femFeasL,femFeasR,widthBefore,widthAfter,"
            "femHandErr,femWeaponErr,elbowMoveL,elbowMoveR,elbowFlexL,elbowFlexR,shotTrig,shootValid,shootWeight,shootApplied,shootScale,"
            "recoilT,recoilR,gripRightErr,gripLeftErr,wRelaxedP2H,wReadyP2H,wAimP2H,wRelaxedRifle,wReadyRifle,wAimRifle,wDefault,aoApplied,"
            "supportW,stockW,stockShift,twistAfter,reachShift,reachNeed,stockGap\n";
    std::vector<BoneTransform> out(db->NumBones()), raw(db->NumBones());
    ov::Pose op;
    authored_recoil::Player player;
    std::vector<std::unique_ptr<authored_recoil::Clip>> clips;
    std::vector<std::string> clipFiles;
    auto recoilClip = [&](const std::string& file) -> const authored_recoil::Clip* {
        if (file == "-") return nullptr;
        for (size_t i = 0; i < clipFiles.size(); ++i) if (clipFiles[i] == file) return clips[i].get();
        auto c = std::make_unique<authored_recoil::Clip>();
        std::string e;
        if (!c->Load((recoilDir + "\\" + file).c_str(), e)) { std::fprintf(stderr, "recoil load failed %s: %s\n", file.c_str(), e.c_str()); return nullptr; }
        std::printf("RECOIL %s duration %.4f peakT %.4f peakR %.4f keys %zu\n", file.c_str(), c->Duration(), c->PeakTranslation(), c->PeakRotation(), c->Count());
        clipFiles.push_back(file);
        clips.push_back(std::move(c));
        return clips.back().get();
    };
    const authored_recoil::Binding binding{1, 1, 1};
    uint64_t serial = 0;
    bool admitted = false;
    double admitTime = 0, now = 0;
    const authored_recoil::Clip* active = nullptr;
    int prevClip = -1;
    size_t emitted = 0;
    for (size_t k = 0; k < steps.size(); ++k)
    {
        const Step& s = steps[k];
        now += s.dt;
        const int clip = FindClip(*db, s.clip);
        if (clip < 0) { std::fprintf(stderr, "unknown clip %s\n", s.clip.c_str()); return 5; }
        v2::MatchResult m;
        m.clip = clip;
        m.clipTime = s.clipTime;
        m.playRate = 1.0f;
        m.jumped = s.jumped != 0 || prevClip < 0;
        if (s.jumped && prevClip >= 0)
        {
            m.blend.valid = true;
            m.blend.blendTime = s.blendTime;
            m.blend.fromClip = prevClip;
            m.blend.toClip = clip;
            m.blend.toTime = s.clipTime;
        }
        prevClip = clip;
        motionpose::OverlayFrameInputs in;
        in.bodyWoman = !man;
        in.maleIdlePistolStance = man;
        in.machine.style = man ? ov::Style::Masculine : ov::Style::Feminine;
        in.machine.overlay = static_cast<ov::PoseOverlay>(s.overlay);
#if defined(GRIPSTOCK_NEW)
        // MotionDriver.cpp: the entry of the equipped weapon goes in on the Rifle overlay only (long guns)
        in.supportGrip = in.machine.overlay == ov::PoseOverlay::Rifle ? support : nullptr;
#endif
        in.machine.wantsReady = s.wantsReady != 0;
        in.machine.wantsToAim = s.wantsToAim != 0;
        in.machine.moving = s.moving != 0;
        in.machine.wantsToSprint = in.machine.sprinting = s.moving == 2; // gripstock: moving 2 = sprint
        in.machine.hasVelocity = s.hasVelocity != 0;
        const bool twoHand = in.machine.overlay == ov::PoseOverlay::Rifle || in.machine.overlay == ov::PoseOverlay::Pistol2H;
        in.authoredTwoHandGrip = twoHand && s.handback == 0; // MayUseAuthoredTwoHandGrip: grip enabled, weapon drawn, 2H overlay, no handback
        if (s.haveAim)
        {
            const float x = s.tx, y = s.ty, z = s.tz - 1.4f, h = std::sqrt(x * x + y * y);
            in.haveAim = true;
            in.aimYawDeg = std::atan2(-x, y) * (180.0f / 3.14159265358979f);
            in.aimPitchDeg = std::atan2(z, h) * (180.0f / 3.14159265358979f);
            in.haveActorAimPoint = true;
            in.actorAimPoint = {s.tx, s.ty, s.tz};
            in.barrelLoop = true;
            in.barrelAxis = aim::kDefaultBarrelAxis;
            in.barrelMuzzle = aim::Params{}.muzzleOffset;
            in.haveAimPointVisual = true;
            in.aimPointVisual = in.actorAimPoint;
        }
        // authored recoil route (MotionDriver.cpp ~2884-2913)
        const authored_recoil::Clip* rc = recoilClip(s.recoilFile);
        if (s.shot && rc)
        {
            if (!player.Active()) admitted = false;
            const int id = ProfileOfFile(s.recoilFile);
            player.Trigger(*rc, binding, ++serial, now, recoil_profile::LimitsOf(id));
            active = rc;
        }
        float shootWeight = 0;
        bool shootValid = false;
        ov::Xf delta{};
        if (active)
        {
            shootValid = player.Sample(*active, binding, now, delta);
            const float u = admitted ? std::clamp(float((now - admitTime) / .05), 0.0f, 1.0f) : 0.0f;
            const float admission = admitted ? std::max(.001f, u * u * (3.0f - 2.0f * u)) : .001f;
            shootWeight = admission * player.TailWeight(now) * 1.0f;
            in.shootDeltaValid = shootValid;
            in.shootDelta = delta;
            in.shootWeight = shootWeight;
            in.shootPrepare = true;
            in.shootAction = true;
            in.shootOneHand = in.machine.overlay == ov::PoseOverlay::Pistol1H;
            if (!shootValid && !player.Active()) active = nullptr;
        }
        // feminine amount of this step
        float amount = s.femAmount;
        if (s.femMode == 1)
        {
            ov::OverlayMachine probe = ev->Machine(); // the same update the evaluator runs first in this step
            probe.Update(in.machine, s.dt);
            float wl = 0;
            for (int i = 0; i < probe.poses.count; ++i)
            {
                ov::PoseOverlay o; ov::AimState a; bool c, mv;
                ov::DecodePoseKey(probe.poses.key[i], o, a, c, mv);
                if (o == ov::PoseOverlay::Pistol2H && a == ov::AimState::Relaxed) wl += probe.poses.w[i];
            }
            amount = s.femAmount * ov::Clamp01(wl);
        }
        ev->SetFeminineShoulders(amount);
        motionpose::FrameInfo info;
        if (!ev->Evaluate(s.dt, m, in, out.data(), static_cast<uint32_t>(out.size()), &info))
        { std::fprintf(stderr, "Evaluate failed at step %zu\n", k); return 6; }
        if (active && info.shootApplied && !admitted) { admitted = true; admitTime = now; }
        if (!s.emit) continue;
        ev->ToOv(out.data(), op);
        for (int i = 0; i < sk.n; ++i)
        {
            const auto& x = op.b[i];
            const float row[7] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y, x.r.z, x.r.w};
            pose.write(reinterpret_cast<const char*>(row), sizeof(row));
        }
        adapter::SamplePose(*db, clip, s.clipTime, raw.data(), static_cast<uint32_t>(raw.size()));
        for (int i = 0; i < 2; ++i)
        {
            const auto& x = raw[i];
            const float row[7] = {x.p.x, x.p.y, x.p.z, x.q.x, x.q.y, x.q.z, x.q.w};
            root.write(reinterpret_cast<const char*>(row), sizeof(row));
        }
        float w[7] = {};
        for (int i = 0; i < ev->Machine().poses.count; ++i)
        {
            ov::PoseOverlay o; ov::AimState a; bool c, mv;
            ov::DecodePoseKey(ev->Machine().poses.key[i], o, a, c, mv);
            const float pw = ev->Machine().poses.w[i];
            if (o == ov::PoseOverlay::Pistol2H) w[static_cast<int>(a)] += pw;
            else if (o == ov::PoseOverlay::Rifle) w[3 + static_cast<int>(a)] += pw;
            else if (o == ov::PoseOverlay::Default) w[6] += pw;
        }
        const ov::Xf eff = shootValid ? ov::BlendXf(ov::Xf{}, delta, shootWeight) : ov::Xf{};
        char buf[1024];
        std::snprintf(buf, sizeof(buf),
                      "%zu,%.5f,%s,%.5f,%d,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.7f,%.7f,%.5f,%.5f,%.3f,%.3f,%d,%d,%.5f,%d,%.5f,%.5f,%.5f,"
                      "%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d",
                      k, now, s.clip.c_str(), s.clipTime, info.fem.ran, info.fem.applied, static_cast<int>(info.fem.reason), info.fem.requested,
                      amount, info.fem.weight[0], info.fem.weight[1], info.fem.feasible[0], info.fem.feasible[1], info.fem.widthBefore,
                      info.fem.widthAfter, info.fem.handError, info.fem.weaponError, info.fem.elbowMove[0], info.fem.elbowMove[1],
                      info.fem.elbowFlexDeg[0], info.fem.elbowFlexDeg[1], s.shot, shootValid, shootWeight, info.shootApplied, info.shootScale,
                      authored_recoil::Translation(eff), authored_recoil::Rotation(eff), info.grip.right.handErr, info.grip.left.handErr,
                      w[0], w[1], w[2], w[3], w[4], w[5], w[6], info.aoApplied);
        diag << buf;
#if defined(GRIPSTOCK_NEW)
        std::snprintf(buf, sizeof(buf), ",%.5f,%.5f,%.5f,%.3f,%.5f,%.5f,%.5f\n", info.supportGripWeight, info.stockWeight, info.stockShiftM,
                      info.supportTwistAfterDeg, info.reachShiftM, info.reachShiftNeedM, info.stockGapM);
#else
        std::snprintf(buf, sizeof(buf), ",0,0,0,%.3f,0,0,0\n", info.supportTwistAfterDeg);
#endif
        diag << buf;
        ++emitted;
    }
    std::printf("RAN steps %zu emitted %zu bones %d serial %llu\n", steps.size(), emitted, sk.n, static_cast<unsigned long long>(serial));
    return 0;
}

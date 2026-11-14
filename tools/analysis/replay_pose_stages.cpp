// Replays a recorded game feed dump through the real motion driver and leg tail and writes the local pose of every database bone at two
// stages per feed: the driver output handed to the leg tail ("driver": sampled clips, blends, root frame not yet applied) and the published
// snapshot ("final": root offset applied, stance guard, foot locker, head lead, hand-in). No game or input access.
//
//   utpp_replay_pose_stages <data dir> <base database> <feed dump> <out prefix> [--from T] [--to T] [--settings-log LOG]
//
// <out prefix>.txt  header: bone names and parents, the record layout, the clip names seen
// <out prefix>.bin  per feed one record of float32: kMetaFloats meta values (see kMeta below), then driver pose, then final pose
//                   (numBones x [px py pz qx qy qz qw] each, database local space)
// --from / --to limit the written records to feed times in [T0, T1]; every feed is still stepped.
// --settings-log applies the setting calls of a game log of the session (the ones this tool knows, in their order) before the first feed.
// <feed dump> = --clips [name part] writes the stored frames of the woman database clips whose name contains the part (all without it)
// instead (same layout, see the clip mode below).
#include <filesystem>
#define main FullIntegrationSuiteNotRun
#include "../../plugin/UTPP/tests/MotionDriverIntegrationTest.cpp"
#undef main

namespace
{
constexpr const char* kMeta[] = {"t", "dt", "owned", "published", "tailed", "clip", "clipTime", "db", "contactL", "contactR",
                                 "rootVelX", "rootVelY", "rootYawRate", "providedRoot", "rootOffX", "rootOffY", "rootOffYaw",
                                 "offX_cm", "offY_cm", "offYaw_deg", "offClamps", "offResets", "offTeleports",
                                 "lockL", "wL", "corrL_cm", "lockR", "wR", "corrR_cm", "playRate", "capX", "capY", "yawDeg",
                                 "mvx", "mvy", "loco", "rpPrepared", "rpCapDX", "rpCapDY", "rpCapDYaw", "rpAnimDX", "rpAnimDY",
                                 "rpAnimDYaw", "rpMovingGround", "rpMontage", "rpBeforeX", "rpBeforeY", "rpBeforeYaw", "rpAfterX", "rpAfterY",
                                 "rpAfterYaw"};
constexpr size_t kMetaFloats = sizeof(kMeta) / sizeof(kMeta[0]);

motionhost::MotionSetFrame g_tail;
std::vector<float> g_driverPose;
bool g_tailValid = false;

bool CaptureTail(const int32_t* aMap, uint32_t aN, motionhost::MotionSetFrame& aFrame)
{
    const bool ok = motiondriver::Frame(aMap, aN, aFrame);
    if (ok)
    {
        g_tailValid = true;
        g_tail = aFrame;
        g_driverPose.assign(size_t(aFrame.numBones) * 7, 0.0f);
        for (uint32_t k = 0; k < aFrame.numBones; ++k)
        {
            const auto& v = aFrame.pose[k];
            float* p = &g_driverPose[size_t(k) * 7];
            p[0] = v.p.x, p[1] = v.p.y, p[2] = v.p.z, p[3] = v.q.x, p[4] = v.q.y, p[5] = v.q.z, p[6] = v.q.w;
        }
    }
    return ok;
}

float Num(const std::string& aStatus, const char* aKey)
{
    const std::string v = StatusField(aStatus, aKey);
    return v.empty() ? std::nanf("") : std::strtof(v.c_str(), nullptr);
}

// The setting calls of a game log ("UTPPSet<Name>(<args>) -> 1"), applied in their order through the C++ API; unknown names are counted.
void ApplyLoggedSettings(const std::string& aLog, int& aApplied, int& aUnknown)
{
    std::ifstream in(aLog);
    std::string line;
    while (std::getline(in, line))
    {
        const size_t at = line.find("UTPPSet");
        const size_t open = at == std::string::npos ? at : line.find('(', at);
        const size_t close = open == std::string::npos ? open : line.find(')', open);
        if (close == std::string::npos || line.find("-> 1", close) == std::string::npos)
            continue;
        const std::string name = line.substr(at + 7, open - at - 7);
        std::vector<float> a;
        std::istringstream args(line.substr(open + 1, close - open - 1));
        std::string tok;
        while (std::getline(args, tok, ','))
            a.push_back(std::strtof(tok.c_str(), nullptr));
        const auto arg = [&](size_t i) { return i < a.size() ? a[i] : 0.0f; };
        bool known = true;
        if (name == "MotionOrientWarp") motiondriver::SetOrientWarp(arg(0) != 0);
        else if (name == "MotionPelvisYawTrim") motiondriver::SetPelvisYawTrim(arg(0));
        else if (name == "MotionBodyYawOffset") motiondriver::SetBodyYawOffset(arg(0));
        else if (name == "MotionLean") motiondriver::SetLean(arg(0) != 0);
        else if (name == "MotionLeanHead") motiondriver::SetLeanHead(arg(0) != 0);
        else if (name == "MotionLeanParams") motiondriver::SetLeanParams(arg(0), arg(1));
        else if (name == "MMHeadLead") motiondriver::SetHeadLead(arg(0) != 0);
        else if (name == "MotionWalkSpeed") motiondriver::SetWalkSpeed(arg(0));
        else if (name == "MotionMinAnalogWalk") motiondriver::SetMinAnalogWalk(arg(0));
        else if (name == "MotionSlide") motiondriver::SetSlide(arg(0) != 0);
        else if (name == "MotionBarrelLoop") motiondriver::SetBarrelLoop(arg(0) != 0);
        else if (name == "MotionShooterMode") motiondriver::SetShooterMode(arg(0) != 0);
        else if (name == "MMTurnRateParams") motionhost::SetTurnRateParams(arg(0), arg(1));
        else if (name == "MMStrafeParams") motionhost::SetStrafeParams(arg(0) != 0, arg(1), arg(2), arg(3), arg(4));
        else if (name == "MMAir") motionhost::SetAir(static_cast<int32_t>(arg(0)));
        else if (name == "MMFootReplant") motionhost::SetFootReplant(static_cast<int32_t>(arg(0)));
        else if (name == "MMKneeFix") motionhost::SetKneeFix(static_cast<int32_t>(arg(0)));
        else if (name == "MMClimbRelease") motionhost::SetClimbRelease(arg(0) != 0);
        else known = false;
        (known ? aApplied : aUnknown) += 1;
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::fprintf(stderr, "usage: utpp_replay_pose_stages <data dir> <base database> <feed dump> <out prefix> [--from T] [--to T]\n");
        return 2;
    }
    const std::string data = argv[1], feedPath = argv[3], out = argv[4];
    float from = -1e30f, to = 1e30f;
    std::string settingsLog;
    for (int i = 5; i + 1 < argc; i += 2)
    {
        if (std::strcmp(argv[i], "--from") == 0)
            from = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--to") == 0)
            to = std::stof(argv[i + 1]);
        else if (std::strcmp(argv[i], "--settings-log") == 0)
            settingsLog = argv[i + 1];
    }
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    const auto womanPaths = DataPaths(data, "woman");
    if (!RigFromDb(womanPaths.files.db, rig, &db))
        return 3;
    if (feedPath == "--clips")
    {
        // clip mode: <out prefix>.bin = the stored frames of every clip (one record per frame, same layout; both pose blocks = the frame,
        // meta: t = frame time in the clip, clip = clip index, dt = 1 / fps), for the content check of a recorded step
        const uint32_t nbc = db->NumBones();
        const float fps = db->Header().fps > 0.0f ? db->Header().fps : 30.0f;
        std::ofstream cbin(out + ".bin", std::ios::binary);
        std::vector<float> rec(kMetaFloats + size_t(nbc) * 14, 0.0f);
        size_t frames = 0;
        const std::string filter = argc > 5 ? argv[5] : "";
        for (uint32_t c = 0; c < db->NumClips(); ++c)
        {
            if (!filter.empty() && db->ClipName(c).find(filter) == std::string_view::npos)
                continue;
            const auto& clip = db->Clip(c);
            for (uint32_t k = 0; k < clip.numFrames; ++k)
            {
                std::fill(rec.begin(), rec.end(), 0.0f);
                rec[0] = float(k) / fps, rec[1] = 1.0f / fps, rec[5] = float(c);
                const auto* pose = db->Pose(clip.firstFrame + k);
                for (uint32_t b = 0; b < nbc; ++b)
                {
                    const float v[7] = {pose[b].px, pose[b].py, pose[b].pz, pose[b].qx, pose[b].qy, pose[b].qz, pose[b].qw};
                    std::copy(v, v + 7, rec.begin() + kMetaFloats + size_t(b) * 7);
                    std::copy(v, v + 7, rec.begin() + kMetaFloats + size_t(nbc + b) * 7);
                }
                cbin.write(reinterpret_cast<const char*>(rec.data()), std::streamsize(rec.size() * sizeof(float)));
                ++frames;
            }
        }
        std::ofstream ctxt(out + ".txt");
        ctxt << "UTPP_POSE_STAGES 1\nbones " << nbc << "\nmeta " << kMetaFloats;
        for (const char* m : kMeta)
            ctxt << ' ' << m;
        ctxt << "\nrecords " << frames << "\nfps " << fps << "\n";
        for (uint32_t b = 0; b < nbc; ++b)
            ctxt << "bone " << b << ' ' << db->BoneName(b) << ' ' << db->BoneParent(b) << '\n';
        for (uint32_t c = 0; c < db->NumClips(); ++c)
            ctxt << "clip " << c << ' ' << db->ClipName(c) << '\n';
        std::printf("clips=%u frames=%zu fps=%g\n", db->NumClips(), frames, double(fps));
        return 0;
    }
    std::vector<motionhost::FeedInput> inputs;
    std::vector<Frame> rows;
    {
        std::ifstream fin(feedPath);
        std::string line;
        while (std::getline(fin, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream stream(line);
            std::vector<std::string> v;
            std::string token;
            while (stream >> token)
                v.push_back(token);
            if (v.size() < 17)
                return 6;
            Frame fr;
            fr.t = std::stof(v[0]);
            fr.dt = std::stof(v[1]);
            fr.vx = std::stof(v[2]);
            fr.vy = std::stof(v[3]);
            fr.vz = std::stof(v[4]);
            fr.mvx = std::stof(v[5]);
            fr.mvy = std::stof(v[6]);
            fr.yawDeg = std::stof(v[7]);
            fr.camDeg = std::stof(v[8]);
            fr.loco = std::stoi(v[9]);
            fr.psm = int(std::stoul(v[10]));
            auto in = ToInput(fr);
            if (v.size() >= 31)
            {
                in.haveTarget = std::stoi(v[17]) != 0;
                in.targetYawDeg = std::stof(v[18]);
                in.haveExtra = true;
                for (int j = 0; j < 3; ++j)
                    in.capsulePos[j] = std::stof(v[19 + j]);
                fr.cap[0] = in.capsulePos[0];
                fr.cap[1] = in.capsulePos[1];
                in.haveAir = true;
                in.onGround = std::stoi(v[27]) != 0;
                in.distToGround = std::stof(v[28]);
                in.psmFall = std::stoi(v[29]);
                in.walkToggled = std::stoi(v[30]) != 0;
            }
            rows.push_back(fr);
            inputs.push_back(in);
        }
    }
    if (inputs.empty())
        return 6;
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &CaptureTail,
                                        &motiondriver::OwnsTraversal};
    g_baseDb = Wide(argv[2]);
    motiondriver::SetPathsOverride(0, womanPaths);
    motiondriver::SetPathsOverride(1, DataPaths(data, "man"));
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motionhost::SetMotionHooks(&hooks);
    if (!ResetBase(rig))
        return 4;
    int applied = 0, unknown = 0;
    if (!settingsLog.empty())
        ApplyLoggedSettings(settingsLog, applied, unknown);
    motiondriver::SetEnabled(true);
    motiondriver::Step(inputs.front(), 1, inputs.front().dt);
    motiondriver::WaitLoader();
    const uint32_t nb = db->NumBones();
    std::ofstream bin(out + ".bin", std::ios::binary);
    std::vector<float> rec(kMetaFloats + size_t(nb) * 14);
    std::map<int, std::string> clipsSeen;
    size_t written = 0, missing = 0;
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        g_log = {};
        g_tailValid = false;
        motionhost::Feed(inputs[i]);
        const bool published = ReadPublished();
        if (rows[i].t < from || rows[i].t > to)
            continue;
        if (!published)
            ++missing;
        const std::string st = motiondriver::Status();
        const std::string pick = StatusField(st, "pick=clip:");
        const auto at = pick.find('@'), slash = pick.find("/db");
        const int clip = pick.empty() ? -1 : std::atoi(pick.c_str());
        const float ct = at == std::string::npos ? 0.f : float(std::atof(pick.substr(at + 1, slash - at - 1).c_str()));
        const int dbi = slash == std::string::npos ? -1 : std::atoi(pick.c_str() + slash + 3);
        if (clip >= 0 && uint32_t(clip) < db->NumClips())
            clipsSeen.emplace(clip, std::string(db->ClipName(uint32_t(clip))));
        const std::string hs = motionhost::Status();
        const motiondriver::RootProbe rp = motiondriver::TestRootProbe();
        const float meta[kMetaFloats] = {rows[i].t, rows[i].dt, g_log.stepResult ? 1.f : 0.f, published ? 1.f : 0.f, g_tailValid ? 1.f : 0.f,
                                         float(clip), ct, float(dbi), g_tail.contact[0], g_tail.contact[1], g_tail.rootVel[0],
                                         g_tail.rootVel[1], g_tail.rootYawRate, g_tail.providedRoot ? 1.f : 0.f, g_tail.rootOffsetX,
                                         g_tail.rootOffsetY, g_tail.rootOffsetYaw, Num(hs, "offX="), Num(hs, "offY="), Num(hs, "offYaw="),
                                         Num(hs, "offClamps="), Num(hs, "offResets="), Num(hs, "offTeleports="), Num(hs, "lockL="),
                                         Num(hs, "wL="), Num(hs, "corrL="), Num(hs, "lockR="), Num(hs, "wR="), Num(hs, "corrR="),
                                         Num(st, "rate="), rows[i].cap[0], rows[i].cap[1], rows[i].yawDeg, rows[i].mvx, rows[i].mvy,
                                         float(rows[i].loco), rp.prepared ? 1.f : 0.f, rp.capsuleDelta[0], rp.capsuleDelta[1],
                                         rp.capsuleYawDelta, rp.animDelta[0], rp.animDelta[1], rp.animYawDelta, rp.movingGround ? 1.f : 0.f,
                                         rp.montage ? 1.f : 0.f, rp.before[0], rp.before[1], rp.before[2], rp.after[0], rp.after[1], rp.after[2]};
        std::copy(meta, meta + kMetaFloats, rec.begin());
        std::fill(rec.begin() + kMetaFloats, rec.end(), std::nanf(""));
        if (g_tailValid && g_driverPose.size() == size_t(nb) * 7)
            std::copy(g_driverPose.begin(), g_driverPose.end(), rec.begin() + kMetaFloats);
        if (published && g_read.numBones == nb)
            std::copy(g_read.pose, g_read.pose + size_t(nb) * 7, rec.begin() + kMetaFloats + size_t(nb) * 7);
        bin.write(reinterpret_cast<const char*>(rec.data()), std::streamsize(rec.size() * sizeof(float)));
        ++written;
    }
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    std::ofstream txt(out + ".txt");
    txt << "UTPP_POSE_STAGES 1\nbones " << nb << "\nmeta " << kMetaFloats;
    for (const char* m : kMeta)
        txt << ' ' << m;
    txt << "\nrecords " << written << "\n";
    for (uint32_t b = 0; b < nb; ++b)
        txt << "bone " << b << ' ' << db->BoneName(b) << ' ' << db->BoneParent(b) << '\n';
    for (const auto& [id, name] : clipsSeen)
        txt << "clip " << id << ' ' << name << '\n';
    std::printf("feeds=%zu written=%zu unpublished=%zu settingsApplied=%d settingsUnknown=%d\n", inputs.size(), written, missing, applied,
                unknown);
    return 0;
}

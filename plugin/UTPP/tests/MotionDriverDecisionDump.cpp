// Shooter mode, evidence A8: cross-build decision dump of the REAL v1 MMDriver + MotionDriver (no RED4ext / game).
// Usage: utpp_motion_driver_decisiondump <out.txt> <v2 data dir> <v1 database.utppmm> [--closed13 <flick_180_run.feed> <run_arc_left.feed>] [feed ...]
//   One line per frame and feed, in the order given: "<feed> <i> <stepped> <clip> <db> <clipTime> <visualYawDeg> <hash>"
//   ("%s %zu %d %d %d %.9g %.9g %016llx": feed name, frame index, v2 stepped flag, picked clip, database, clip time (status resolution,
//   0.01 s), committed visual root yaw (deg), FrameOut::hash = hash of the published pose snapshot).
//   --closed13 appends the 13 ClosedLoop variants of the two recordings (the actor follows the velocity: the normal mode of the game).
// The tool uses ONLY the API present at 723aee0 (and the Shooter branch with the setting off): it is built twice - on the baseline commit and on
// the feature branch - and the two output files must be byte-identical (Shooter OFF = bit-identical to main, spec off_invariants P2).
// The harness (Wrap, ClosedLoop, RunDriver, DumpMain) is ported from fix4/spin-guard a77e03a tests/MotionDriverSpinGuardTest.cpp; the line format
// is extended by the exact clip time, the visual root yaw and the published pose hash.
// --full-feed (opt-in, churn analysis 2026-11-11; the default output above is unchanged): the 31-column game feed dump is read completely
// (Locomotion world target, capsule position, walk toggle, air feed - the inputs the driver had in the game, so the replay reproduces the clip changes
// of the recorded session) and the two names of the picked database / clip are appended to every line.
#define main UTPPUnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace
{
bool g_fullFeed = false; // --full-feed
constexpr float kRadToDeg = 57.29577951f;

// The 31-column game feed dumps (the recorded log) hold '-' in the v1 columns (baseClip .. step) on every v2-owned frame, which
// the integration LoadFeed (>> int / >> float) rejects. Same first 11 numeric columns (t .. psm); trans and the five v1 columns are read as text and ignored (ToInput
// never uses them). Used for every feed, so both builds load the files identically.
bool LoadFeedAny(const std::string& aPath, FeedFile& f)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    f.name = aPath.substr(aPath.find_last_of("/\\") + 1);
    std::string line, transTxt, baseClip, baseFlags, baseCost, baseMicros, step;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream s(line);
        Frame r;
        s >> r.t >> r.dt >> r.vx >> r.vy >> r.vz >> r.mvx >> r.mvy >> r.yawDeg >> r.camDeg >> r.loco >> r.psm >> transTxt >> baseClip >> baseFlags >> baseCost >>
            baseMicros >> step;
        if (!s)
            return false;
        // columns 17..30 (31-column game dump): Locomotion world target, capsule, gates, PSM details, air feed, walk toggle. Read into the
        // optional Frame fields so a game dump replays with the inputs the driver had (a 17-column feed leaves them unset).
        std::string haveTarget, targetYaw, cap[3], effGate, holdEligible, gateRaw, upperBody, detailed, onGround, distGround, psmFall, walk;
        s >> haveTarget >> targetYaw >> cap[0] >> cap[1] >> cap[2] >> effGate >> holdEligible >> gateRaw >> upperBody >> detailed >> onGround >> distGround >>
            psmFall >> walk;
        if (g_fullFeed && s)
        {
            const auto num = [](const std::string& a, float& out) {
                if (a == "-")
                    return false;
                char* e = nullptr;
                out = std::strtof(a.c_str(), &e);
                return e != a.c_str();
            };
            r.haveTarget = haveTarget == "1" && num(targetYaw, r.targetYawDeg);
            r.haveExtra = walk == "0" || walk == "1";
            r.walk = walk == "1";
            for (int i = 0; i < 3; ++i)
                num(cap[i], r.cap[i]);
            r.haveAir = onGround == "0" || onGround == "1";
            r.onGround = onGround != "0";
            num(distGround, r.distGround);
            float pf = 0.0f;
            r.psmFall = num(psmFall, pf) ? static_cast<int>(pf) : 0;
        }
        f.fr.push_back(r);
    }
    return !f.fr.empty();
}

float Wrap(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}

// a feed whose player yaw follows the velocity heading at aRateDegS (the normal mode of the game: Locomotion + native body turn);
// aRepeat > 1 plays the recorded run that many times in a row (the actor stays where the previous copy left it)
FeedFile ClosedLoop(const FeedFile& aOpen, float aRateDegS, int aRepeat)
{
    FeedFile out;
    out.name = aOpen.name + "_follow" + std::to_string(static_cast<int>(aRateDegS)) + (aRepeat > 1 ? "x" + std::to_string(aRepeat) : std::string());
    float yaw = aOpen.fr.front().yawDeg;
    float t = 0.0f;
    for (int rep = 0; rep < aRepeat; ++rep)
        for (const Frame& src : aOpen.fr)
        {
            Frame f = src;
            const float dt = (std::isfinite(f.dt) && f.dt > 0.0f) ? f.dt : 1.0f / 60.0f;
            if (std::hypot(f.vx, f.vy) > 0.5f)
            {
                const float target = std::atan2(-f.vx, f.vy) * kRadToDeg;
                const float d = Wrap(target - yaw);
                yaw = Wrap(yaw + std::fmax(-aRateDegS * dt, std::fmin(aRateDegS * dt, d)));
            }
            f.yawDeg = std::round(yaw * 1000.0f) / 1000.0f;
            t += dt;
            f.t = t;
            out.fr.push_back(f);
        }
    return out;
}

// one feed through the real driver (fresh v1 state, v2 bundle loaded before the measured run, as check 2 of the integration test)
int g_dumpBody = 0; // UTPP_DUMP_BODY=man: the Male V bundle (body slot 1)
bool g_dumpCorr = false; // UTPP_DUMP_SLIDE=3: the foot lock correction of each foot after every frame
std::vector<float> g_corr; // 2 per frame (cm): corrL, corrR of motionhost::Status()

void ReadFootLockCorr(float& aL, float& aR)
{
    const std::string st = motionhost::Status();
    const size_t p = st.find("flReady=");
    aL = aR = 0.0f;
    if (p == std::string::npos)
        return;
    const size_t l = st.find("corrL=", p), r = st.find("corrR=", p);
    if (l != std::string::npos)
        aL = std::strtof(st.c_str() + l + 6, nullptr);
    if (r != std::string::npos)
        aR = std::strtof(st.c_str() + r + 6, nullptr);
}

std::vector<FrameOut> RunDriver(const Rig& aRig, const FeedFile& aF)
{
    motiondriver::SetRigBody(g_dumpBody);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    ResetBase(aRig);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    motionhost::Feed(ToInput(aF.fr.front()));
    motiondriver::WaitLoader();
    ResetBase(aRig);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    g_corr.clear();
    if (!g_dumpCorr)
        return RunFeed(aF, kNoop);
    // the callback runs before each frame: it reads the state the previous frame left
    std::vector<FrameOut> run = RunFeed(aF, [](size_t aI) {
        if (aI == 0)
            return;
        float l = 0.0f, r = 0.0f;
        ReadFootLockCorr(l, r);
        g_corr.push_back(l);
        g_corr.push_back(r);
    });
    float l = 0.0f, r = 0.0f;
    ReadFootLockCorr(l, r);
    g_corr.push_back(l);
    g_corr.push_back(r);
    return run;
}

int DumpMain(const std::string& aOut, const std::string& aData, const std::string& aFlick, const std::string& aArc,
             const std::vector<std::string>& aFeeds)
{
    // opt-in: UTPP_DUMP_BODY=man replays a Male V feed (default woman: output of the A8 tool unchanged)
    char* bodyEnv = nullptr;
    size_t bodyEnvLen = 0;
    _dupenv_s(&bodyEnv, &bodyEnvLen, "UTPP_DUMP_BODY");
    g_dumpBody = bodyEnv && std::strcmp(bodyEnv, "man") == 0 ? 1 : 0;
    std::free(bodyEnv);
    // opt-in: UTPP_DUMP_GUARDS=0 replays with every selection churn guard off (motiondriver::SetSelectionGuards(false)); unset = on
    char* guardsEnv = nullptr;
    size_t guardsEnvLen = 0;
    _dupenv_s(&guardsEnv, &guardsEnvLen, "UTPP_DUMP_GUARDS");
    motiondriver::SetSelectionGuards(!(guardsEnv && std::strcmp(guardsEnv, "0") == 0));
    std::free(guardsEnv);
    const motiondriver::Paths pw = DataPaths(aData, g_dumpBody ? "man" : "woman");
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(pw.files.db, rig, &db))
    {
        std::printf("FAIL v2 rig of %s\n", pw.files.db.c_str());
        return 3;
    }
    motiondriver::SetPathsOverride(g_dumpBody, pw);
    motionhost::SetMotionHooks(&kHooks);
    // UTPP_DUMP_SLIDE=1 or 2 (full-feed dump): two more columns, the toe displacement per frame (m) of each foot while the pick's contact is >= 0.5
    // (MetricCtx of the integration test): a proxy of the planted-foot slide the foot lock has to correct
    MetricCtx mc;
    mc.rig = &rig;
    mc.db = db.get();
    for (uint32_t bn = 0; bn < db->NumBones(); ++bn)
    {
        if (db->BoneName(bn) == "LeftToeBase")
            mc.toe[0] = static_cast<int32_t>(bn);
        if (db->BoneName(bn) == "RightToeBase")
            mc.toe[1] = static_cast<int32_t>(bn);
    }
    char* slideEnv = nullptr;
    size_t slideEnvLen = 0;
    _dupenv_s(&slideEnv, &slideEnvLen, "UTPP_DUMP_SLIDE");
    const bool slideCols = slideEnv != nullptr;
    // UTPP_DUMP_SLIDE=2: four more columns, the world position (m, x / y) of the left and the right toe (the planted-foot drift of a contact phase)
    // UTPP_DUMP_SLIDE=3: as 2, plus two columns, the foot lock correction (cm) of the left and the right foot after the frame
    g_dumpCorr = slideEnv && std::strcmp(slideEnv, "3") == 0;
    const bool slideToe = slideEnv && (std::strcmp(slideEnv, "2") == 0 || g_dumpCorr);
    std::free(slideEnv);
    if (slideCols)
        g_metrics = &mc;
    std::FILE* o = nullptr;
    if (fopen_s(&o, aOut.c_str(), "wb") != 0 || !o)
    {
        std::printf("FAIL cannot write %s\n", aOut.c_str());
        return 3;
    }
    std::vector<FeedFile> feeds;
    for (const std::string& path : aFeeds)
    {
        FeedFile f;
        if (!LoadFeedAny(path, f))
        {
            std::printf("FAIL feed %s\n", path.c_str());
            return 3;
        }
        feeds.push_back(std::move(f));
    }
    if (!aFlick.empty())
    {
        FeedFile flick, arc;
        if (!LoadFeedAny(aFlick, flick) || !LoadFeedAny(aArc, arc))
        {
            std::printf("FAIL closed-loop feeds %s / %s\n", aFlick.c_str(), aArc.c_str());
            return 3;
        }
        struct Case
        {
            const FeedFile* open;
            float rate;
            int repeat;
        };
        // the 13 cases of fix4/spin-guard check C (360 deg/s: the root keeps up ... 5000 deg/s: a snap)
        const Case cases[] = {{&flick, 360.0f, 1}, {&flick, 720.0f, 1}, {&flick, 720.0f, 2}, {&arc, 360.0f, 1},  {&arc, 720.0f, 1},
                              {&flick, 540.0f, 1}, {&flick, 1080.0f, 1}, {&flick, 1440.0f, 1}, {&flick, 5000.0f, 1}, {&arc, 540.0f, 1},
                              {&arc, 1080.0f, 1},  {&arc, 1440.0f, 1},   {&arc, 5000.0f, 1}};
        for (const Case& c : cases)
            feeds.push_back(ClosedLoop(*c.open, c.rate, c.repeat));
    }
    size_t total = 0;
    for (const FeedFile& f : feeds)
    {
        const std::vector<FrameOut> run = RunDriver(rig, f);
        int32_t lb = -1;
        const bool loaded = motiondriver::LoadedBody(lb) && lb == g_dumpBody && motiondriver::Reason() == Fallback::None;
        size_t stepped = 0;
        for (size_t i = 0; i < run.size(); ++i)
        {
            stepped += run[i].stepped;
            // columns after the hash (names of the picked database / clip, '-' when none) are informational for the analysis scripts
            if (!g_fullFeed)
            {
                std::fprintf(o, "%s %zu %d %d %d %.9g %.9g %016llx\n", f.name.c_str(), i, run[i].stepped ? 1 : 0, run[i].clip, run[i].db,
                             static_cast<double>(run[i].clipTime), static_cast<double>(run[i].visualYawDeg),
                             static_cast<unsigned long long>(run[i].hash));
                continue;
            }
            const bool dbOk = run[i].db >= 0 && static_cast<uint32_t>(run[i].db) < db->NumDatabases();
            const bool clipOk = run[i].clip >= 0 && static_cast<uint32_t>(run[i].clip) < db->Header().numClips;
            const std::string_view dbn = dbOk ? db->DbName(static_cast<uint32_t>(run[i].db)) : std::string_view("-");
            const std::string_view cln = clipOk ? db->ClipName(static_cast<uint32_t>(run[i].clip)) : std::string_view("-");
            std::fprintf(o, "%s %zu %d %d %d %.9g %.9g %016llx %.*s %.*s", f.name.c_str(), i, run[i].stepped ? 1 : 0, run[i].clip, run[i].db,
                         static_cast<double>(run[i].clipTime), static_cast<double>(run[i].visualYawDeg),
                         static_cast<unsigned long long>(run[i].hash), static_cast<int>(dbn.size()), dbn.data(), static_cast<int>(cln.size()),
                         cln.data());
            if (slideCols && slideToe)
            {
                std::fprintf(o, " %.5f %.5f %.4f %.4f %.4f %.4f", static_cast<double>(run[i].slide[0]), static_cast<double>(run[i].slide[1]),
                             static_cast<double>(run[i].toeW[0][0]), static_cast<double>(run[i].toeW[0][1]), static_cast<double>(run[i].toeW[1][0]),
                             static_cast<double>(run[i].toeW[1][1]));
                if (g_dumpCorr && 2 * i + 1 < g_corr.size())
                    std::fprintf(o, " %.1f %.1f", static_cast<double>(g_corr[2 * i]), static_cast<double>(g_corr[2 * i + 1]));
                std::fputc('\n', o);
            }
            else if (slideCols)
                std::fprintf(o, " %.5f %.5f\n", static_cast<double>(run[i].slide[0]), static_cast<double>(run[i].slide[1]));
            else
                std::fputc('\n', o);
        }
        total += run.size();
        // diagnostics go to stdout only (the dump file holds nothing but decisions)
        std::printf("INFO %s frames %zu stepped %zu bundle %s\n", f.name.c_str(), run.size(), stepped, loaded ? "loaded" : "NOT LOADED");
        if (!loaded || stepped == 0)
        {
            std::printf("FAIL v2 did not run on %s\n", f.name.c_str());
            std::fclose(o);
            return 4;
        }
    }
    std::fclose(o);
    std::printf("INFO dump %s: %zu feeds, %zu frames\n", aOut.c_str(), feeds.size(), total);
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return 0;
}
} // namespace

#ifndef UTPP_DECISIONDUMP_NO_MAIN // tests/MMChurnTest.cpp includes this file for its helpers and has its own main
int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("usage: decisiondump <out.txt> <motion data dir> <base db> [--full-feed] [--closed13 <flick_180_run.feed> <run_arc_left.feed>] [feed ...]\n");
        return 2;
    }
    g_baseDb.assign(argv[3], argv[3] + std::strlen(argv[3]));
    std::string flick, arc;
    int next = 4;
    if (next < argc && std::string(argv[next]) == "--full-feed")
    {
        g_fullFeed = true;
        ++next;
    }
    if (next + 2 < argc && std::string(argv[next]) == "--closed13")
    {
        flick = argv[next + 1];
        arc = argv[next + 2];
        next += 3;
    }
    return DumpMain(argv[1], argv[2], flick, arc, std::vector<std::string>(argv + next, argv + argc));
}
#endif

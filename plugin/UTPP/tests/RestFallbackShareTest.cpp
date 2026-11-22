// Rest turn-in-place vs its footwork-less fallback, real host, one body per run.
//
// While V stands with a ranged weapon and a hidden yaw offset built under one of UTPP's facing owners (hipfire lease, ADS exit), the
// matcher turns the offset out with authored turn-in-place clips (MotionAdapter restTurn) and MotionOffsetRoot folds back what is left
// beyond its 40 deg knee without footwork (the fallback, feet slide about r * angle). This test measures how much of the resolved rest
// yaw the fallback carries: the offsets of the 20 stationary reloads of the 12:15 recording (tests/data/recording_1215_rest_offset.inc)
// for standing and crouching at 60 and 144 Hz, plus the band between the fallback knee and the turn admission angle (41/45/49 deg),
// where no turn-in-place is admitted.
//
// Per episode: fallback = sum |rest step| of the offset root (test hooks), resolved = |offset at the start| - |offset at the end|.
// Share = sum fallback / sum resolved over the recorded offsets (degree-weighted). Required: share <= 20 % on each body / stance / rate.
// The band episodes are reported (all fallback by design) and must still be folded to the knee.
// Compiled with -DUTPP_REST_TURN_MUTATION=1 (the rest turn-in-place is never admitted) this test MUST report FAIL lines.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include "../src/mm/MotionOffsetRoot.hpp"

namespace
{
bool providedRootSeen = false;
float capturedRootYaw = 0;
bool CaptureRestFrame(const int32_t *map, uint32_t n, motionhost::MotionSetFrame &f)
{
    const bool ok = motiondriver::Frame(map, n, f);
    providedRootSeen |= ok && f.providedRoot;
    if (ok && f.providedRoot)
        capturedRootYaw = f.rootOffsetYaw;
    return ok;
}
struct RestFrame
{
    float dt, capDeg;
    int flags;
};
struct RestEpisode
{
    const char *name;
    int first, count;
    float offsetDeg, capEventDeg, camEventDeg;
    int event;
    float preVisToCamDeg, heldVisToCamDeg;
};
#include "data/recording_1215_rest_offset.inc"
struct Sum
{
    double fallback = 0, resolved = 0, fallbackWindow = 0, resolvedWindow = 0;
    int episodes = 0, tipEpisodes = 0;
    float maxEnd = 0;
};
} // namespace

#include "LegacyDllDefaults.hpp" // earlier DLL startup values for the scenarios below
int main(int argc, char **argv)
{
    utpp::testdefaults::LegacyDriverDefaults(); // scenarios below were tuned with the earlier DLL startup values
    utpp::testdefaults::LegacyHostDefaults();
    if (argc != 4)
    {
        std::puts("usage RestFallbackShareTest <driver-data> <basedb> <woman|man>");
        return 2;
    }
    const std::string body = argv[3];
    const bool man = body == "man";
    if (!man && body != "woman")
        return 2;
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(DataPaths(argv[1], body.c_str()).files.db, rig, &db))
        return 3;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(argv[1], "man"));
    motiondriver::SetRigBody(man ? 1 : 0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motiondriver::SetAimGameUpper(false);
    motiondriver::SetWeaponUpper(false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &CaptureRestFrame, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    Check(ResetBase(rig), "real base loaded");
    motionhost::SetOffsetRoot(true);
    motiondriver::SetEnabled(true);
    motiondriver::Step({}, 0, 1.f / 60);
    motiondriver::WaitLoader();
    const std::string bundle = StatusField(motiondriver::Status(), "bundle=");
    std::printf("bundle %s\n", bundle.c_str());
    Check(bundle.starts_with(body + "/ready"), "the " + body + " bundle is loaded", bundle);
    auto &stats = mm::motionRoot::OffsetRoot::Hooks();
    stats.stats = true;

    motionhost::FeedInput f;
    auto feed = [&]() {
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        providedRootSeen = false;
        motionhost::Feed(f);
        return g_log.stepResult && providedRootSeen;
    };
    auto tipFrame = [&]() {
        const int clip = std::atoi(StatusField(motiondriver::Status(), "pick=clip:").c_str());
        return clip >= 0 && static_cast<uint32_t>(clip) < db->NumClips() &&
               steer::ClassifyClip(db->ClipName(static_cast<uint32_t>(clip))) == steer::kClassTurnInPlace;
    };
    struct Out
    {
        float start = 0, end = 0, endWindow = 0, fallback = 0, fallbackWindow = 0;
        int tipFrames = 0;
        bool owned = true;
    };
    // One episode: settle the stance, build the offset under an aim intent (the capsule turns, the root keeps its heading), then stand
    // with a drawn Pistol2H and no aim for max(window, 3.5 s). The window is the recorded time to the next ADS / shot / move.
    auto episode = [&](int stance, float degrees, int hz, float window) {
        Out o;
        motiondriver::SetAimTarget(false, false, nullptr);
        motiondriver::SetCameraTarget(false, false, nullptr);
        motiondriver::SetWeaponAction(0);
        motiondriver::SetOverlayInputs(0, 0, false, false);
        o.owned = ResetBase(rig);
        motionhost::SetOffsetRoot(true);
        motionhost::SetHoldPose(true);
        motionhost::SetWeaponUpper(false);
        f = {};
        f.dt = 1.f / 60;
        f.tppActive = true;
        f.locoState = stance;
        f.haveAir = true;
        f.onGround = true;
        f.distToGround = 0;
        f.psmBlock = integ::kPsmWeaponKnown;
        for (int j = 0; j < 180; ++j)
            feed();
        const float dt = 1.f / float(hz);
        f.dt = dt;
        motiondriver::SetOverlayInputs(0, 2, false, false); // a facing owner (aim intent) turns the capsule, as the lease / ADS align does
        for (int j = 0; j < 30; ++j)
        {
            f.playerYawDeg += degrees / 30;
            f.cameraYawDeg = f.playerYawDeg;
            feed();
        }
        motiondriver::SetOverlayInputs(3, 1, false, false);
        f.psmBlock = integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn;
        o.start = std::fabs(capturedRootYaw) / integ::kDegToRad;
        const double fb0 = stats.restFallback;
        const int windowFrames = int(std::lround(window * float(hz)));
        const int frames = std::max(windowFrames, int(std::lround(3.5f * float(hz))));
        o.endWindow = o.start;
        for (int j = 0; j < frames; ++j)
        {
            o.owned = feed() && o.owned;
            o.tipFrames += tipFrame() ? 1 : 0;
            if (j + 1 == windowFrames)
            {
                o.endWindow = std::fabs(capturedRootYaw) / integ::kDegToRad;
                o.fallbackWindow = float((stats.restFallback - fb0) / mm::motionRoot::kDegToRad);
            }
        }
        o.end = std::fabs(capturedRootYaw) / integ::kDegToRad;
        o.fallback = float((stats.restFallback - fb0) / mm::motionRoot::kDegToRad);
        if (windowFrames <= 0)
            o.fallbackWindow = 0;
        return o;
    };
    for (int stance : {integ::kLocoStand, integ::kLocoCrouch})
        for (int hz : {60, 144})
        {
            const char *stanceName = stance == integ::kLocoStand ? "standing" : "crouching";
            Sum s;
            for (const RestEpisode &e : kRestEpisodes)
            {
                float window = 0;
                for (int i = 0; i < e.count; ++i)
                    window += kRestFrames[e.first + i].dt;
                const Out o = episode(stance, e.offsetDeg, hz, window);
                std::printf("EP %s %s %dHz %s offset %+.1f start %.1f end %.1f fallback %.1f | window %.2fs end %.1f fallback %.1f | TIP frames %d\n",
                            body.c_str(), stanceName, hz, e.name, e.offsetDeg, o.start, o.end, o.fallback, window, o.endWindow, o.fallbackWindow,
                            o.tipFrames);
                Check(o.owned && std::isfinite(o.end), body + " " + stanceName + " " + e.name + " supplies finite committed roots");
                s.fallback += o.fallback;
                s.resolved += std::max(0.f, o.start - o.end);
                s.fallbackWindow += o.fallbackWindow;
                s.resolvedWindow += std::max(0.f, o.start - o.endWindow);
                s.tipEpisodes += o.tipFrames > 0 ? 1 : 0;
                s.maxEnd = std::max(s.maxEnd, o.end);
                ++s.episodes;
            }
            const double share = s.resolved > 0 ? s.fallback / s.resolved : 1.0;
            const double shareWindow = s.resolvedWindow > 0 ? s.fallbackWindow / s.resolvedWindow : 1.0;
            std::printf("SHARE %s %s %dHz recorded offsets: fallback %.1f of %.1f deg resolved = %.1f %% (to the recorded event: %.1f of %.1f = %.1f %%)"
                        ", TIP in %d of %d episodes, max end offset %.1f deg\n",
                        body.c_str(), stanceName, hz, s.fallback, s.resolved, 100 * share, s.fallbackWindow, s.resolvedWindow,
                        100 * shareWindow, s.tipEpisodes, s.episodes, s.maxEnd);
            char label[160];
            std::snprintf(label, sizeof label, "%s %s %dHz: the fallback carries <= 20 %% of the resolved rest yaw", body.c_str(), stanceName, hz);
            Check(share <= .20, label, std::to_string(100 * share));
            std::snprintf(label, sizeof label, "%s %s %dHz: every recorded offset ends below the turn admission angle", body.c_str(), stanceName, hz);
            Check(s.maxEnd < motiontraj::kTurnInPlaceDeg, label, std::to_string(s.maxEnd));
            // the band between the knee and the admission angle: no turn is admitted, the fallback folds it to the knee
            for (float degrees : {41.f, 45.f, 49.f, -45.f})
            {
                const Out o = episode(stance, degrees, hz, 0);
                std::printf("BAND %s %s %dHz offset %+.0f start %.1f end %.1f fallback %.1f TIP frames %d\n", body.c_str(), stanceName, hz,
                            degrees, o.start, o.end, o.fallback, o.tipFrames);
                std::snprintf(label, sizeof label, "%s %s %dHz band %+.0fdeg is folded to the knee", body.c_str(), stanceName, hz, degrees);
                Check(o.owned && o.end <= mm::motionRoot::kRestKneeLow / mm::motionRoot::kDegToRad + 4.f, label, std::to_string(o.end));
            }
        }
    std::printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return g_fail ? 1 : 0;
}

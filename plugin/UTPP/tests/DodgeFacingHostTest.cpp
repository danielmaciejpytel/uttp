// Facing while a ground dodge clip plays (real host + motion driver, mm/Dodge.hpp rule): V runs forward, the stick turns 90 deg left and
// the dodge is pressed on the same feed; the game pushes the capsule along the stick (the corrected push, velocity model of the recorded
// dodges). The body turn issues commands for the game order (command for the current yaw, applied after the feed).
// Checks: the left clip is shown; while it is shown every body yaw command holds the yaw (step 0) and the drawn heading stays within
// 3 deg of the heading at the clip start and within 5 deg of the capsule yaw (no hidden turn to undo after the clip); after the
// hand-back the body turns to the stick (within 15 deg 1.2 s after the press). 30 and 60 Hz, both bodies. Before the rule the root frame already
// kept the drawn heading near its start (5 / 10 deg at 60 / 30 Hz) while the capsule turned ~85 deg under the clip (the mutant).
// Mutant UTPP_DODGE_FACING_MUTATION=1 (the body keeps turning under the clip) MUST print "FAIL ... drawn heading holds".
// Usage: utpp_loco_dodge_facing_host_test <driver-data> <basedb> <utpp_dodge_woman.utpptrav> [<utpp_dodge_man.utpptrav>]
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>

#include "../src/mm/Dodge.hpp"
namespace
{
constexpr float kDegF = mm::integ::kDegToRad;
float Wrap180(float a) { return std::remainder(a, 360.0f); }

struct DodgeRun
{
    bool ok = false;
    int clipFeeds = 0, clipCmds = 0, nonZeroClipCmds = 0, handbacks = 0;
    int32_t sector = -1;
    float drawnAtStart = 0, drawnDev = 0, yawAt12 = 0, maxClipStep = 0, offsetMax = 0; // offsetMax: |drawn - capsule yaw| under the clip
    bool clipAfter = false;
};

DodgeRun RunDodge(const Rig& aRig, float aHz)
{
    DodgeRun out;
    if (!ResetBase(aRig))
        return out;
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    const float dt = 1.0f / aHz;
    const int stand = int(aHz * 1.0f), press = int(aHz * 2.5f), total = int(aHz * 4.0f);
    motionhost::FeedInput f;
    f.tppActive = true;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    f.psmBlock = mm::integ::kPsmWeaponKnown;
    float yaw = 0, vx = 0, vy = 0, entry[2] = {0, 0};
    bool clipStarted = false, prevActive = false;
    for (int i = 0; i < total; ++i)
    {
        const bool moving = i >= stand;
        const float target = i >= press ? 90.0f : 0.0f;
        const float tDodge = (i - press) * dt;
        const bool dodge = i >= press && tDodge < 0.2f - 1e-4f;
        if (i == press)
        {
            entry[0] = vx;
            entry[1] = vy;
        }
        if (dodge && tDodge > 1e-4f)
        {
            // the recorded dodge (tests/DodgeTest.cpp SimulateGame): impulse P along the push + k of the entry velocity
            const float n = tDodge * 60.0f;
            const float P = n < 1.5f ? 6.2f * std::fmin(1.0f, n) : std::fmax(0.0f, 11.4f - (n - 2.0f));
            const float k = n < 1.5f ? 1.0f - 0.27f * std::fmin(1.0f, n) : std::fmax(0.3f, 0.6f - 0.05f * (n - 2.0f));
            vx = P * -std::sin(target * kDegF) + k * entry[0];
            vy = P * std::cos(target * kDegF) + k * entry[1];
        }
        else if (moving && !dodge)
        {
            const float sp = std::fmin(3.85f, std::sqrt(vx * vx + vy * vy) + 8.0f * dt);
            vx = -std::sin(target * kDegF) * sp;
            vy = std::cos(target * kDegF) * sp;
        }
        f.dt = dt;
        f.velWorld[0] = vx;
        f.velWorld[1] = vy;
        f.move[0] = 0;
        f.move[1] = moving ? 1.0f : 0.0f;
        f.cameraYawDeg = 0;
        f.haveTarget = moving;
        f.targetYawDeg = target;
        f.locoState = dodge ? 0 : mm::integ::kLocoStand; // the recorded feeds: locomotion state 0 during the dodge detail
        f.psmBlock = (mm::integ::kPsmWeaponKnown & ~(0xFFu << 16)) | (static_cast<uint32_t>((dodge ? 7 : -1) + 1) << 16);
        f.yawAheadDeg = std::numeric_limits<float>::quiet_NaN();
        float cmdStep = 0;
        if (motionhost::BodyTurnActive())
        {
            const float cmd = motionhost::BodyYawCommand(yaw, yaw);
            cmdStep = Wrap180(cmd - yaw);
        }
        f.playerYawDeg = yaw;
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        motionhost::Feed(f);
        const motiondriver::DodgeProbe p = motiondriver::TestDodgeProbe();
        if (p.active)
        {
            ++out.clipFeeds;
            if (!clipStarted)
            {
                clipStarted = true;
                out.sector = p.sector;
                out.drawnAtStart = g_log.visualYawDeg;
            }
            out.drawnDev = std::fmax(out.drawnDev, std::fabs(Wrap180(g_log.visualYawDeg - out.drawnAtStart)));
            out.offsetMax = std::fmax(out.offsetMax, std::fabs(Wrap180(g_log.visualYawDeg - g_log.actorYawDeg)));
            if (prevActive)
            {
                // the command of this feed was issued while the clip of the previous feed was shown
                ++out.clipCmds;
                out.nonZeroClipCmds += cmdStep != 0.0f ? 1 : 0;
                out.maxClipStep = std::fmax(out.maxClipStep, std::fabs(cmdStep));
            }
        }
        else if (clipStarted && !prevActive && i > press + int(aHz * 0.6f))
            out.clipAfter = out.clipAfter || p.active;
        out.handbacks += p.handback ? 1 : 0;
        prevActive = p.active;
        yaw = Wrap180(yaw + cmdStep); // game order: the command is applied after the feed
        if (i == press + int(aHz * 1.2f))
            out.yawAt12 = yaw;
    }
    out.ok = true;
    return out;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::puts("usage DodgeFacingHostTest <driver-data> <basedb> <utpp_dodge_woman.utpptrav> [<utpp_dodge_man.utpptrav>]");
        return 2;
    }
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrame, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    const struct
    {
        const char* body;
        int32_t index;
        const char* dodge;
    } bodies[] = {{"woman", 0, argv[3]}, {"man", 1, argc >= 5 ? argv[4] : nullptr}};
    for (const auto& bd : bodies)
    {
        if (!bd.dodge)
            continue;
        Rig rig;
        Check(RigFromDb(DataPaths(argv[1], bd.body).files.db, rig), std::string(bd.body) + ": rig from the motion database");
        motiondriver::SetEnabled(false);
        motiondriver::Paths paths = DataPaths(argv[1], bd.body);
        paths.dodge = bd.dodge;
        motiondriver::SetPathsOverride(bd.index, paths);
        motiondriver::SetRigBody(bd.index);
        Check(ResetBase(rig), std::string(bd.body) + ": real host initialized");
        motiondriver::SetEnabled(true);
        motiondriver::Step({}, 0, 1.f / 60);
        motiondriver::WaitLoader();
        for (float hz : {60.0f, 30.0f})
        {
            const DodgeRun r = RunDodge(rig, hz);
            char d[320];
            std::snprintf(d, sizeof d,
                          "%.0f Hz: clip %s for %d feeds, %d commands under it (%d non-zero, max step %.2f deg), drawn heading moved %.2f deg, drawn - "
                          "capsule up to %.1f deg, yaw 1.2 s after the press %.1f deg, hand-backs %d",
                          double(hz), mm::dodge::SectorName(static_cast<mm::dodge::Sector>(r.sector)), r.clipFeeds, r.clipCmds, r.nonZeroClipCmds,
                          double(r.maxClipStep), double(r.drawnDev), double(r.offsetMax), double(r.yawAt12), r.handbacks);
            std::printf("INFO %s %s\n", bd.body, d);
            const std::string tag = std::string(bd.body) + (hz == 60.0f ? " 60 Hz" : " 30 Hz");
            Check(r.ok && r.clipFeeds >= int(hz * 0.2f) && r.sector == static_cast<int32_t>(mm::dodge::Sector::Left) && r.clipCmds > 0,
                  tag + ": fixture - the left clip is shown and the body turn issues commands under it", d);
            Check(r.nonZeroClipCmds == 0 && r.drawnDev <= 3.0f && r.offsetMax <= 5.0f,
                  tag + ": drawn heading holds while the dodge clip is shown (yaw commands hold)", d);
            Check(r.handbacks >= 1 && std::fabs(Wrap180(r.yawAt12 - 90.0f)) <= 15.0f, tag + ": after the hand-back the body turns to the stick", d);
        }
    }
    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAILED" : "PASSED", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

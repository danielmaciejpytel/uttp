#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>
namespace {
float capturedDeceleration = -1;
bool BrakingStep(const motionhost::FeedInput& f, uint32_t gate, float dt)
{
    capturedDeceleration = f.capsuleBrakingDeceleration;
    return WStep(f, gate, dt);
}
}
int main(int argc, char** argv)
{
    if (argc != 3) { std::puts("usage CapsuleBrakingHostTest <driver-data> <basedb>"); return 2; }
    Rig rig; if (!RigFromDb(DataPaths(argv[1], "woman").files.db, rig)) return 3;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetRigBody(0); motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&BrakingStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &motiondriver::Frame, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    Check(ResetBase(rig), "real host initialized");
    motionhost::SetCapsuleSync(true); motionhost::SetHoldPose(true);
    motionhost::FeedInput f; f.dt = 1.f / 60; f.tppActive = true;
    f.locoState = integ::kLocoStand; f.velWorld[1] = 4.86f;
    f.haveAir = true; f.onGround = true; f.distToGround = 0;
    f.psmBlock = integ::kPsmWeaponKnown;
    auto feed = [&](int n) { for (int i=0; i<n; ++i) { motiondriver::FeedAirFromInput(f); g_log={}; motionhost::Feed(f); } return g_log.stepResult; };
    motiondriver::SetEnabled(false); feed(20);
    const float baseTarget = motionhost::DecelTarget(24, 1);
    const bool fixtureOk = baseTarget > 0 && baseTarget < 24;
    Check(fixtureOk, "fixture reproduces legacy slowdown under base ownership");
    if (!fixtureOk)
    {
        // The legacy slowdown comes from the capsule stop profile of the v1 database (*_Run_Stop_F clips, floor 10 m/s2). A database
        // without them (e.g. the 2026-10-06 dense subset: Walk_Stop_F only) gives target 0, and every later check would either fail
        // for that reason (the multiplier baseTarget/24 = 0) or pass vacuously (handback 0 == 0). Stop here with the actual cause.
        std::puts("FAIL v1 database (argv[2]) has no Run/Crouch/Sprint _Stop_F capsule stop profile: pass the game's v1 bake, e.g. Data/utpp_base.utppmm");
        std::printf("CapsuleBrakingHost: %d PASS %d FAIL\n", g_pass, g_fail);
        return 4;
    }
    motiondriver::SetEnabled(true); motiondriver::Step({}, 0, f.dt); motiondriver::WaitLoader();
    Check(feed(20), "motion owns real host frame");
    Check(motionhost::DecelTarget(24, baseTarget / 24) == 0, "motion removes legacy slowdown including previous applied multiplier");
    feed(1); Check(capturedDeceleration == 24, "fresh native base reaches motion context in physical units");
    f.onGround = false; feed(1); motionhost::DecelTarget(1.3f, 1);
    f.onGround = true; feed(1);
    Check(capturedDeceleration == 0, "first landing feed rejects fresh airborne 1.3 stat before grounded query");
    motionhost::DecelTarget(24, 1); feed(1);
    Check(capturedDeceleration == 24, "grounded query restores 24 after landing without script reordering");
    f.locoState = integ::kLocoCrouch; feed(1);
    Check(capturedDeceleration == 0, "stance change rejects previous ground-domain stat");
    feed(19);
    Check(motionhost::DecelTarget(24, 1) == 0, "crouched motion preserves native braking");
    feed(1); Check(capturedDeceleration == 24, "crouch receives same native stat, mapping remains adapter owned");
    Sleep(250); feed(1); Check(capturedDeceleration == 0, "stale native stat clears from internal context");
    motionhost::DecelTarget(std::numeric_limits<float>::quiet_NaN(), 1); feed(1);
    Check(capturedDeceleration == 0, "invalid stat cannot reuse old sample");
    motionhost::DecelTarget(24, 1); feed(1);
    Check(capturedDeceleration == 24, "fresh native query restores prediction after expiration");
    f.tppActive = false; Check(!feed(1), "FPP releases motion ownership");
    Check(motionhost::DecelTarget(24, 1) == 0, "FPP does not request capsule override");
    f.tppActive = true; f.locoState = integ::kLocoStand; motiondriver::SetEnabled(false); feed(30);
    Check(motionhost::DecelTarget(24, 1) == baseTarget, "motion to base handback restores unchanged legacy target");
    f.move[1] = 1; feed(1); Check(motionhost::DecelTarget(24, 1) == 0, "held input keeps vanilla braking under base");
    motionhost::ResetSessionState(); feed(1);
    Check(capturedDeceleration == 0, "session reset invalidates cached native stat");
    std::printf("CapsuleBrakingHost: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

// Layer order of the weapon upper body on the real driver path (UTPP_POSE_STAGE_TRACE, src/mm/PoseStageTrace.hpp).
// Regression pin of the order the pose layers touch the published pose in the current code (not a reference order):
//   overlay layering (pose overlay + style additives) -> feminine posture passes -> inertialized overlay switch -> aim offset ->
//   barrel pitch loop -> support grip / stock -> shot (recoil) delta -> grip IK -> hand-back of the upper body to the game
//   (mm.aimGameUpper; the game's own graph and aim offset then own the upper body).
// Checks, woman, Rifle overlay, aiming at a target 30 deg to the right:
//   1. mm.aimGameUpper off: layering, aim offset and grip IK all run, in that order, no hand-back; the aim offset turns the spine
//      towards the target and the barrel loop measures the barrel after it;
//   2. mm.aimGameUpper on: the hand-back is the last stage of every frame and, once its fade is over, the motion aim offset no longer
//      runs (the game owns the aimed upper body); grip IK still runs before the hand-back;
//   3. back off: the aim offset returns before the grip IK.
//   Not exercised here (no shot, barrel loop off, no overlay switch, no support grip): those positions are unpinned.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include "../src/mm/PoseStageTrace.hpp"

namespace utpp::motiondriver
{
float TestAoAppliedWeight();
} // namespace utpp::motiondriver

namespace
{
namespace st = mm::stagetrace;

const char* StageName(st::Stage aStage)
{
    switch (aStage)
    {
    case st::Stage::Layering: return "Layering";
    case st::Stage::Feminine: return "Feminine";
    case st::Stage::OverlaySwitch: return "OverlaySwitch";
    case st::Stage::AimOffset: return "AimOffset";
    case st::Stage::BarrelLoop: return "BarrelLoop";
    case st::Stage::SupportGrip: return "SupportGrip";
    case st::Stage::ShootAdditive: return "ShootAdditive";
    case st::Stage::GripIK: return "GripIK";
    case st::Stage::GameUpper: return "GameUpper";
    }
    return "?";
}

std::string TraceText(const st::Trace& aTrace)
{
    std::string s;
    for (int i = 0; i < aTrace.n; ++i)
        s += (i ? " > " : "") + std::string(StageName(aTrace.s[i]));
    return s;
}

// the enum values are the current order: every frame's trace must be strictly increasing (each stage at most once, none early)
bool InOrder(const st::Trace& aTrace)
{
    for (int i = 1; i < aTrace.n; ++i)
        if (static_cast<int>(aTrace.s[i]) <= static_cast<int>(aTrace.s[i - 1]))
            return false;
    return true;
}

int IndexOf(const st::Trace& aTrace, st::Stage aStage)
{
    for (int i = 0; i < aTrace.n; ++i)
        if (aTrace.s[i] == aStage)
            return i;
    return -1;
}

float AoYaw() { return std::strtof(StatusField(motiondriver::Status(), "aoYaw=").c_str(), nullptr); }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::puts("usage LayerOrderDriverTest <driver-data> <basedb>");
        return 2;
    }
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(DataPaths(argv[1], "woman").files.db, rig, &db))
        return 3;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(1, 2, false, false); // Rifle, Aiming
    motiondriver::SetAimGameUpper(false);
    motiondriver::SetWeaponUpper(false);
    motiondriver::SetGripEnabled(true);
    motionhost::SetMotionHooks(&kHooks);
    Check(ResetBase(rig), "real base loaded");
    motionhost::SetOffsetRoot(true);
    motionhost::SetWeaponUpper(false);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    motiondriver::Step({}, 0, 1.f / 60);
    motiondriver::WaitLoader();

    motionhost::FeedInput f;
    f.dt = 1.f / 60;
    f.tppActive = true;
    f.locoState = integ::kLocoStand;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    const float right[3] = {-30, 60, 1.4f};
    // aN ADS feeds; every frame's trace is checked for the order, the stages seen in any frame are collected
    struct Seen
    {
        int frames = 0, ordered = 0, withStage[16] = {};
        st::Trace last;
    };
    const auto feed = [&](int aN, Seen& aSeen) {
        bool ok = true;
        for (int j = 0; j < aN; ++j)
        {
            f.psmBlock = integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn | (uint32_t(6 + 1) << 13);
            motiondriver::SetCameraTarget(true, true, right);
            motiondriver::SetAimTarget(true, true, right);
            motiondriver::FeedAirFromInput(f);
            g_log = {};
            motionhost::Feed(f);
            ok = ok && g_log.stepResult;
            const st::Trace& t = st::Get();
            ++aSeen.frames;
            aSeen.ordered += InOrder(t) ? 1 : 0;
            for (int i = 0; i < t.n; ++i)
                ++aSeen.withStage[static_cast<int>(t.s[i])];
            aSeen.last = t;
        }
        return ok;
    };
    const auto count = [](const Seen& aSeen, st::Stage aStage) { return aSeen.withStage[static_cast<int>(aStage)]; };

    // 1. mm.aimGameUpper off
    Seen off;
    Check(feed(180, off), "ADS standing, Rifle, aim target right, owned");
    std::printf("  off: last frame %s (frames %d, in order %d, aim offset %d, grip IK %d, hand-back %d)\n", TraceText(off.last).c_str(),
                off.frames, off.ordered, count(off, st::Stage::AimOffset), count(off, st::Stage::GripIK), count(off, st::Stage::GameUpper));
    Check(off.ordered == off.frames, "off: every frame keeps the layer order");
    Check(IndexOf(off.last, st::Stage::Layering) == 0, "off: the overlay layering is the first stage");
    Check(IndexOf(off.last, st::Stage::AimOffset) > IndexOf(off.last, st::Stage::Layering), "off: the aim offset runs after the layering");
    Check(IndexOf(off.last, st::Stage::GripIK) > IndexOf(off.last, st::Stage::AimOffset), "off: the grip IK runs after the aim offset");
    Check(count(off, st::Stage::GameUpper) == 0, "off: no hand-back to the game");
    const float yawOff = AoYaw();
    const motiondriver::BarrelProbe bp = motiondriver::TestBarrelProbe();
    std::printf("  off: aoYaw %.2f applied weight %.3f barrel loop %d measured %d error before %.2f after %.2f deg\n", yawOff,
                motiondriver::TestAoAppliedWeight(), bp.loop ? 1 : 0, bp.measured ? 1 : 0, bp.errBeforeDeg, bp.errAfterDeg);
    Check(yawOff > 10.0f, "off: the aim offset turns the spine towards the target");

    // 2. mm.aimGameUpper on: the hand-back after every layer of the frame; the motion aim offset fades out and stops
    motiondriver::SetAimGameUpper(true);
    Seen fade, on;
    Check(feed(30, fade), "ADS with the game upper body, fade");
    Check(feed(120, on), "ADS with the game upper body, settled");
    std::printf("  on: last frame %s (frames %d, in order %d, aim offset %d, grip IK %d, hand-back %d; fade frames with the aim offset %d)\n",
                TraceText(on.last).c_str(), on.frames, on.ordered, count(on, st::Stage::AimOffset), count(on, st::Stage::GripIK),
                count(on, st::Stage::GameUpper), count(fade, st::Stage::AimOffset));
    Check(fade.ordered == fade.frames && on.ordered == on.frames, "on: every frame keeps the layer order");
    Check(count(on, st::Stage::GameUpper) == on.frames, "on: every aimed frame hands the upper body back");
    Check(on.last.n > 0 && on.last.s[on.last.n - 1] == st::Stage::GameUpper, "on: the hand-back is the last stage");
    Check(count(on, st::Stage::AimOffset) == 0, "on: the motion aim offset stops once the hand-back owns the upper body");
    Check(count(on, st::Stage::GripIK) == on.frames, "on: the grip IK still runs, before the hand-back");

    // 3. back off: the motion aim offset returns, before the grip IK, no hand-back
    motiondriver::SetAimGameUpper(false);
    Seen back;
    Check(feed(120, back), "ADS without the game upper body again");
    std::printf("  back off: last frame %s\n", TraceText(back.last).c_str());
    Check(back.ordered == back.frames, "back off: every frame keeps the layer order");
    Check(IndexOf(back.last, st::Stage::AimOffset) >= 0 && IndexOf(back.last, st::Stage::GripIK) > IndexOf(back.last, st::Stage::AimOffset),
          "back off: the aim offset is back, before the grip IK");
    Check(IndexOf(back.last, st::Stage::GameUpper) < 0, "back off: no hand-back");

    motiondriver::SetAimTarget(false, false, nullptr);
    motionhost::SetMotionHooks(nullptr);
    std::printf("%s (%d failures)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}

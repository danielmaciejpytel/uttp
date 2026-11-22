// Optional aim options on the real driver path (world aim smoothing default on at 12 Hz, switched off here for the unsmoothed
// reference; neutral aim offset default on):
//   1. world aim smoothing (SetAimWorldSmoothing): with a fixed aim target and a still body the smoothed direction converges, so the
//      root-path aim offset yaw with the smoothing on equals the unsmoothed one within a small epsilon, and the smoothing delta is
//      really carried (feeds with a delta > 0);
//   2. neutral aim offset (SetNeutralAimOffset): in the shooter facing without an aim target the spine follows the camera through
//      the aim offset (applied weight, yaw towards the camera side); turning the option off fades the applied weight out instead of
//      cutting it in one frame. The option is on by default and, with the neutral grid in the overlay pack, a frame without an aim
//      target runs on the grid alone (grid share 1);
//   3. a one-frame camera jerk without aiming: the world aim smoothing spreads the step of the neutral aim offset yaw (its own world
//      filter) and converges to the unsmoothed yaw.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace utpp::motiondriver
{
void TestAimOptionFeeds(uint64_t& aNeutralAo, uint64_t& aAimSmooth);
float TestAoAppliedWeight();
float TestAoGridShare();
bool SetNeutralAimOffset(bool aOn);
bool SetAimWorldSmoothing(bool aOn, float aHz);
} // namespace utpp::motiondriver

namespace
{
float AoYaw() { return std::strtof(StatusField(motiondriver::Status(), "aoYaw=").c_str(), nullptr); }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::puts("usage AimOptionsDriverTest <driver-data> <basedb>");
        return 2;
    }
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(DataPaths(argv[1], "woman").files.db, rig, &db))
        return 3;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(3, 1, false, false);
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
    const auto feed = [&](int aN, const float* aCamera, bool aAds) {
        bool ok = true;
        for (int j = 0; j < aN; ++j)
        {
            f.psmBlock = integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn | (aAds ? (uint32_t(6 + 1) << 13) : 0);
            if (aCamera)
                motiondriver::SetCameraTarget(true, true, aCamera);
            motiondriver::FeedAirFromInput(f);
            g_log = {};
            motionhost::Feed(f);
            ok = ok && g_log.stepResult;
        }
        return ok;
    };
    const float right[3] = {-30, 60, 1.4f};
    uint64_t neutralFeeds = 0, smoothFeeds = 0;

    // 1. world aim smoothing, converged = unsmoothed (still body, fixed target); the DLL default is on, so the unsmoothed reference
    // and the smoothing feed count start with it off
    Check(motiondriver::SetAimWorldSmoothing(false, 0.0f), "world aim smoothing off for the unsmoothed reference");
    motiondriver::SetAimTarget(true, true, right);
    Check(feed(180, right, true), "ADS standing with a fixed target owned");
    const float yawRaw = AoYaw();
    Check(yawRaw > 10.0f, "aim offset yaw follows the target to the right");
    Check(motiondriver::SetAimWorldSmoothing(true, 12.0f), "world aim smoothing on");
    Check(feed(180, right, true), "ADS standing with smoothing owned");
    const float yawSmooth = AoYaw();
    motiondriver::TestAimOptionFeeds(neutralFeeds, smoothFeeds);
    Check(smoothFeeds > 0, "the smoothing delta is carried on the aim path");
    std::printf("  aoYaw raw %.3f smoothed %.3f (smoothing feeds %llu)\n", yawRaw, yawSmooth, static_cast<unsigned long long>(smoothFeeds));
    Check(std::fabs(yawSmooth - yawRaw) < 0.25f, "root-path aim offset yaw: smoothing on and converged equals the unsmoothed yaw");
    // The body turns +30 deg in 0.25 s with the target point held in model space: the raw aim offset yaw follows the target at once,
    // the smoothed world direction lags behind it (behind the raw yaw at the end of the turn, never past it) and reaches it once the
    // turn holds. A delta carried with the wrong sign or frame would overshoot the raw yaw.
    const auto turn = [&](float& aLast, float& aMax) {
        aLast = 0.0f;
        aMax = -1e9f;
        const float start = f.playerYawDeg;
        for (int j = 1; j <= 15; ++j)
        {
            f.playerYawDeg = start + 2.0f * static_cast<float>(j);
            feed(1, right, true);
            aLast = AoYaw();
            aMax = std::max(aMax, aLast);
        }
        feed(180, right, true);
    };
    float smLast = 0.0f, smMax = 0.0f, rawLast = 0.0f, rawMax = 0.0f;
    turn(smLast, smMax);
    const float yawSmoothAfterTurn = AoYaw();
    Check(motiondriver::SetAimWorldSmoothing(false, 0.0f), "world aim smoothing off");
    f.playerYawDeg = 0.0f;
    feed(120, right, true);
    turn(rawLast, rawMax);
    const float yawRawAfterTurn = AoYaw();
    std::printf("  turn +30 deg: end of turn smoothed %.2f (max %.2f), raw %.2f (max %.2f); after the hold %.3f / %.3f\n", smLast, smMax,
                rawLast, rawMax, yawSmoothAfterTurn, yawRawAfterTurn);
    Check(rawLast - smLast > 3.0f, "world aim smoothing: the root-relative yaw lags behind a body turn (behind the raw yaw)");
    Check(smMax < rawMax + 0.5f, "world aim smoothing: no overshoot past the raw yaw during the turn");
    Check(std::fabs(yawSmoothAfterTurn - yawRawAfterTurn) < 0.25f, "world aim smoothing: converged after the turn equals the unsmoothed yaw");
    f.playerYawDeg = 0.0f;
    feed(60, right, true);
    motiondriver::SetAimTarget(false, false, nullptr);

    // 2. neutral aim offset in the shooter facing, no aim target, camera to the right
    motiondriver::SetShooterMode(true);
    uint64_t neutralBefore = 0;
    motiondriver::TestAimOptionFeeds(neutralBefore, smoothFeeds);
    Check(feed(180, right, false), "shooter facing without aiming owned (default options)");
    motiondriver::TestAimOptionFeeds(neutralFeeds, smoothFeeds);
    const float gridShare = motiondriver::TestAoGridShare();
    std::printf("  neutral by default: feeds %llu, grid share %.3f\n", static_cast<unsigned long long>(neutralFeeds - neutralBefore), gridShare);
    Check(neutralFeeds > neutralBefore, "the neutral aim offset is on by default");
    Check(gridShare > 0.999f, "a frame without an aim target runs on the neutral grid");
    Check(motiondriver::SetNeutralAimOffset(true), "neutral aim offset on");
    Check(feed(180, right, false), "shooter facing without aiming owned");
    motiondriver::TestAimOptionFeeds(neutralFeeds, smoothFeeds);
    Check(neutralFeeds > 0, "the neutral aim offset weight reaches the pose");
    const float wOn = motiondriver::TestAoAppliedWeight();
    const float yawNeutral = AoYaw();
    std::printf("  neutral: applied weight %.3f aoYaw %.3f (neutral feeds %llu)\n", wOn, yawNeutral, static_cast<unsigned long long>(neutralFeeds));
    Check(wOn > 0.5f, "neutral aim offset applied without an aim target");
    Check(yawNeutral > 5.0f, "neutral aim offset turns the spine towards the camera side");
    Check(motiondriver::SetNeutralAimOffset(false), "neutral aim offset off");
    feed(1, right, false);
    const float wFirstOff = motiondriver::TestAoAppliedWeight();
    feed(30, right, false);
    const float wLaterOff = motiondriver::TestAoAppliedWeight();
    std::printf("  neutral off: applied weight one frame later %.3f, 0.5 s later %.3f\n", wFirstOff, wLaterOff);
    Check(wFirstOff > 0.5f * wOn, "turning the option off fades the weight (no cut in one frame)");
    Check(wLaterOff == 0.0f, "the weight is gone after the fade");

    // 3. a one-frame camera jerk of 12 deg in the shooter facing without aiming: unsmoothed the neutral aim offset yaw takes the whole
    // step in that frame; with the world aim smoothing on (12 Hz) the step is spread over the next frames (< half in the first) and
    // the yaw converges to the unsmoothed one.
    const float jerkRad = -12.0f * 3.14159265f / 180.0f;
    const float jerked[3] = {right[0] * std::cos(jerkRad) - right[1] * std::sin(jerkRad), right[0] * std::sin(jerkRad) + right[1] * std::cos(jerkRad),
                             right[2]};
    Check(motiondriver::SetNeutralAimOffset(true), "neutral aim offset on again");
    const auto jerk = [&](float& aStep, float& aSettled) {
        feed(180, right, false);
        const float before = AoYaw();
        feed(1, jerked, false);
        aStep = AoYaw() - before;
        feed(120, jerked, false);
        aSettled = AoYaw();
        feed(120, right, false);
    };
    float stepRaw = 0.0f, settledRaw = 0.0f, stepSmooth = 0.0f, settledSmooth = 0.0f;
    jerk(stepRaw, settledRaw);
    Check(motiondriver::SetAimWorldSmoothing(true, 12.0f), "world aim smoothing on (neutral)");
    jerk(stepSmooth, settledSmooth);
    Check(motiondriver::SetAimWorldSmoothing(false, 0.0f), "world aim smoothing off (neutral)");
    std::printf("  neutral camera jerk 12 deg: first-frame aoYaw step raw %.2f, smoothed %.2f; settled %.3f / %.3f\n", stepRaw, stepSmooth,
                settledRaw, settledSmooth);
    Check(std::fabs(stepRaw) > 8.0f, "unsmoothed, the neutral aim offset yaw takes the camera jerk in one frame");
    Check(std::fabs(stepSmooth) < 0.5f * std::fabs(stepRaw) && stepSmooth * stepRaw > 0.0f,
          "world aim smoothing spreads the camera jerk of the neutral aim offset (less than half in the first frame)");
    Check(std::fabs(settledSmooth - settledRaw) < 0.25f, "world aim smoothing: the neutral aim offset yaw converges to the unsmoothed one");
    motiondriver::SetShooterMode(false);

    motionhost::SetMotionHooks(nullptr);
    std::printf("%s (%d failures)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}

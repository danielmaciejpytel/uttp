// Directional locomotion inputs (motiontraj::UpdateMoveDirection, IsStartingDirectional, IsPivotingDirectional, IsCircling) and the
// capsule play-rate scale modes (CapsuleRateScaleForMode, WriteRateSpeedScale). No data files.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

using namespace utpp::mm;
using namespace utpp::mm::adapter;
using motiontraj::MoveDirection;
using motiontraj::V3;

static int passed = 0, failed = 0;
static void Check(const char* label, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    ok ? ++passed : ++failed;
}

// One update from a given previous direction (state aged beyond the recent window).
static MoveDirection From(MoveDirection aPrev, float aAngle, const motiontraj::MoveDirectionConfig& c, bool aPivot = false,
                          bool aWide = false, bool aRightFoot = false)
{
    motiontraj::MoveDirectionState s;
    s.current = s.last = s.recent = s.previousDifferent = aPrev;
    motiontraj::MoveDirectionInput in;
    in.moving = in.directional = in.hasDirection = true;
    in.angleDeg = aAngle;
    in.pivoting = aPivot;
    in.lateralWide = aWide;
    in.rightFootForward = aRightFoot;
    motiontraj::UpdateMoveDirection(s, c, in, 1.0f / 60.0f);
    return s.current;
}

struct WithScale
{
    float rateSpeedScale = 1.0f;
    int rateScaleScope = 0;
};
struct WithoutScale
{
    float other = 0.0f;
};
struct WithDirection
{
    int moveDirection = -1, moveDirectionLast = -1, moveDirectionRecent = -1;
    bool isStartingDirectional = false, isPivotingDirectional = false;
};

int main()
{
    const motiontraj::MoveDirectionConfig c;
    // forward set: F within +-60, B beyond +-120
    Check("from F: 59 deg stays F", From(MoveDirection::F, 59.0f, c) == MoveDirection::F);
    Check("from F: -59 deg stays F", From(MoveDirection::F, -59.0f, c) == MoveDirection::F);
    Check("from F: 61 deg is RL", From(MoveDirection::F, 61.0f, c) == MoveDirection::RL);
    Check("from F: -61 deg is LL", From(MoveDirection::F, -61.0f, c) == MoveDirection::LL);
    Check("from F: 119 deg is RL", From(MoveDirection::F, 119.0f, c) == MoveDirection::RL);
    Check("from F: 121 deg is B", From(MoveDirection::F, 121.0f, c) == MoveDirection::B);
    Check("from F: -121 deg is B", From(MoveDirection::F, -121.0f, c) == MoveDirection::B);
    Check("from F: 180 and -180 are B", From(MoveDirection::F, 180.0f, c) == MoveDirection::B && From(MoveDirection::F, -180.0f, c) == MoveDirection::B);
    Check("from F: 540 normalises to B", From(MoveDirection::F, 540.0f, c) == MoveDirection::B);
    // backward set
    Check("from B: 121 deg stays B", From(MoveDirection::B, 121.0f, c) == MoveDirection::B);
    Check("from B: 119 deg is RL", From(MoveDirection::B, 119.0f, c) == MoveDirection::RL);
    Check("from B: 59 deg is F", From(MoveDirection::B, 59.0f, c) == MoveDirection::F);
    // lateral hysteresis: the loop set keeps the lateral quadrant up to 140 behind
    Check("from RL loop: 130 deg stays RL", From(MoveDirection::RL, 130.0f, c) == MoveDirection::RL);
    Check("from RL loop: 141 deg is B", From(MoveDirection::RL, 141.0f, c) == MoveDirection::B);
    Check("from RL loop: 59 deg is F", From(MoveDirection::RL, 59.0f, c) == MoveDirection::F);
    Check("from RL transition: 45 deg stays RL", From(MoveDirection::RL, 45.0f, c, false, true) == MoveDirection::RL);
    Check("from RL transition: 39 deg is F", From(MoveDirection::RL, 39.0f, c, false, true) == MoveDirection::F);
    Check("from LL pivoting: the backward set (130 deg is B)", From(MoveDirection::LL, -130.0f, c, true) == MoveDirection::B);
    // F -> lateral -> F needs the lateral thresholds: no flip-flop at 60 from F, back to F only inside 60 (loop set)
    Check("from LL loop: -61 deg stays LL", From(MoveDirection::LL, -61.0f, c) == MoveDirection::LL);
    // foot bias: off reports LL / RL only; on reports LR / RR with the right foot forward
    {
        bool onlyFour = true;
        for (float a = -180.0f; a <= 180.0f; a += 1.0f)
            for (int p = 0; p < motiontraj::kMoveDirectionCount; ++p)
            {
                const MoveDirection d = From(static_cast<MoveDirection>(p), a, c, false, false, true);
                onlyFour = onlyFour && d != MoveDirection::LR && d != MoveDirection::RR;
            }
        Check("no foot bias: never LR / RR", onlyFour);
        motiontraj::MoveDirectionConfig b = c;
        b.footBias = true;
        Check("foot bias, right foot forward: LR / RR", From(MoveDirection::F, -90.0f, b, false, false, true) == MoveDirection::LR &&
                                                            From(MoveDirection::F, 90.0f, b, false, false, true) == MoveDirection::RR);
        Check("foot bias, left foot forward: LL / RL", From(MoveDirection::F, -90.0f, b) == MoveDirection::LL &&
                                                           From(MoveDirection::F, 90.0f, b) == MoveDirection::RL);
    }
    // rotation mode, sprint, idle
    {
        motiontraj::MoveDirectionState s;
        motiontraj::MoveDirectionInput in;
        in.moving = in.hasDirection = true;
        in.angleDeg = 170.0f;
        in.directional = false;
        motiontraj::UpdateMoveDirection(s, c, in, 0.016f);
        Check("OrientToMovement reports F", s.current == MoveDirection::F);
        in.directional = true;
        in.sprint = true;
        motiontraj::UpdateMoveDirection(s, c, in, 0.016f);
        Check("sprint reports F", s.current == MoveDirection::F);
        in.sprint = false;
        motiontraj::UpdateMoveDirection(s, c, in, 0.016f);
        Check("strafe at 170 reports B", s.current == MoveDirection::B && s.last == MoveDirection::F);
        in.moving = false;
        in.angleDeg = 0.0f;
        motiontraj::UpdateMoveDirection(s, c, in, 0.016f);
        Check("idle holds the direction", s.current == MoveDirection::B && s.last == MoveDirection::B);
    }
    // recent: the previous direction for recentLimitS after a change
    {
        motiontraj::MoveDirectionState s;
        motiontraj::MoveDirectionInput in;
        in.moving = in.hasDirection = in.directional = true;
        in.angleDeg = 0.0f;
        motiontraj::UpdateMoveDirection(s, c, in, 0.05f);
        in.angleDeg = 170.0f;
        motiontraj::UpdateMoveDirection(s, c, in, 0.05f);
        const bool first = s.current == MoveDirection::B && s.recent == MoveDirection::F;
        motiontraj::UpdateMoveDirection(s, c, in, 0.05f);
        const bool mid = s.recent == MoveDirection::F;
        motiontraj::UpdateMoveDirection(s, c, in, 0.06f);
        Check("recent = previous direction inside the window", first && mid);
        Check("recent = current after the window", s.recent == MoveDirection::B);
    }
    // angle from game world axes: facing yaw 0 looks along +Y, a positive game yaw turns left (towards -X)
    {
        Check("game axes: +Y at yaw 0 is forward", std::fabs(motiontraj::MoveDirectionAngle(V3{0, 1, 0}, 0.0f)) < 1e-3f);
        Check("game axes: -X at yaw 0 is left (-90)", std::fabs(motiontraj::MoveDirectionAngle(V3{-1, 0, 0}, 0.0f) + 90.0f) < 1e-3f);
        Check("game axes: +X at yaw 0 is right (+90)", std::fabs(motiontraj::MoveDirectionAngle(V3{1, 0, 0}, 0.0f) - 90.0f) < 1e-3f);
        Check("game axes: -Y at yaw 0 is backward", std::fabs(std::fabs(motiontraj::MoveDirectionAngle(V3{0, -1, 0}, 0.0f)) - 180.0f) < 1e-3f);
        // facing yaw 90 looks along -X; +Y is then on the right
        Check("game axes: facing yaw 90, +Y is right", std::fabs(motiontraj::MoveDirectionAngle(V3{0, 1, 0}, 90.0f) - 90.0f) < 1e-3f);
        Check("game axes: left strafe classifies LL", From(MoveDirection::F, motiontraj::MoveDirectionAngle(V3{-1, 0.2f, 0}, 0.0f), c) == MoveDirection::LL);
        Check("game axes: zero heading is 0", motiontraj::MoveDirectionAngle(V3{}, 30.0f) == 0.0f);
    }
    // IsStartingDirectional
    {
        const V3 fut{300.0f, 0.0f, 0.0f}, vel{50.0f, 0.0f, 0.0f};
        Check("directional start at Speed2D 100", motiontraj::IsStartingDirectional(true, fut, vel, false, 100.0f, true));
        Check("no directional start at Speed2D 101", !motiontraj::IsStartingDirectional(true, fut, vel, false, 101.0f, true));
        Check("no directional start in a pivot database", !motiontraj::IsStartingDirectional(true, fut, vel, true, 50.0f, true));
        Check("no directional start while not moving", !motiontraj::IsStartingDirectional(false, fut, vel, false, 50.0f, true));
        Check("future below velocity + 100 is not a start", !motiontraj::IsStartingDirectional(true, V3{140.0f, 0, 0}, vel, false, 50.0f, true));
        const V3 slowFut{80.0f, 0.0f, 0.0f}, slowVel{30.0f, 0.0f, 0.0f};
        Check("slow target: half-future margin starts", motiontraj::IsStartingDirectional(true, slowFut, slowVel, false, 30.0f, true));
        Check("slow target: fixed margin does not start", !motiontraj::IsStartingDirectional(true, slowFut, slowVel, false, 30.0f, false));
        Check("matches IsStarting below the speed bound",
              motiontraj::IsStartingDirectional(true, fut, vel, false, 90.0f, true) == motiontraj::IsStarting(true, fut, vel, false));
    }
    // IsPivotingDirectional (75 deg) and circling
    {
        const V3 vel{200.0f, 0.0f, 0.0f};
        auto at = [](float deg) { const float r = deg / motiontraj::kRad2Deg; return V3{200.0f * std::cos(r), 200.0f * std::sin(r), 0.0f}; };
        Check("74 deg is no directional pivot", !motiontraj::IsPivotingDirectional(true, at(74.0f), vel, false));
        Check("76 deg is a directional pivot", motiontraj::IsPivotingDirectional(true, at(76.0f), vel, false));
        Check("-76 deg is a directional pivot", motiontraj::IsPivotingDirectional(true, at(-76.0f), vel, false));
        Check("50 deg pivots at 45 but not at 75", motiontraj::IsPivoting(true, at(50.0f), vel, motiontraj::RotationMode::OrientToMovement) &&
                                                        !motiontraj::IsPivotingDirectional(true, at(50.0f), vel, false));
        Check("circling suppresses the directional pivot", !motiontraj::IsPivotingDirectional(true, at(120.0f), vel, true));
        Check("no directional pivot without velocity", !motiontraj::IsPivotingDirectional(true, at(120.0f), V3{}, false));
        Check("no directional pivot while not moving", !motiontraj::IsPivotingDirectional(false, at(120.0f), vel, false));
        Check("circling: both rates above 200 the same way and turn > 50", motiontraj::IsCircling(250.0f, 210.0f, 60.0f) &&
                                                                             motiontraj::IsCircling(-250.0f, -210.0f, -60.0f));
        Check("no circling: opposite rates, slow rate or small turn", !motiontraj::IsCircling(250.0f, -250.0f, 60.0f) &&
                                                                          !motiontraj::IsCircling(250.0f, 150.0f, 60.0f) &&
                                                                          !motiontraj::IsCircling(250.0f, 250.0f, 40.0f));
    }
    // capsule play-rate scale modes
    {
        Check("off: scale 1", CapsuleRateScaleForMode(CapsuleRateMode::Off, 1.3f) == 1.0f);
        Check("both: scale as computed", CapsuleRateScaleForMode(CapsuleRateMode::Both, 0.8f) == 0.8f &&
                                             CapsuleRateScaleForMode(CapsuleRateMode::Both, 1.3f) == 1.3f);
        Check("raise only", CapsuleRateScaleForMode(CapsuleRateMode::RaiseOnly, 0.8f) == 1.0f &&
                                CapsuleRateScaleForMode(CapsuleRateMode::RaiseOnly, 1.3f) == 1.3f);
        Check("lower only", CapsuleRateScaleForMode(CapsuleRateMode::LowerOnly, 0.8f) == 0.8f &&
                                CapsuleRateScaleForMode(CapsuleRateMode::LowerOnly, 1.3f) == 1.0f);
        Check("non-finite or non-positive scale is 1", CapsuleRateScaleForMode(CapsuleRateMode::Both, std::numeric_limits<float>::quiet_NaN()) == 1.0f &&
                                                          CapsuleRateScaleForMode(CapsuleRateMode::Both, 0.0f) == 1.0f);
        Check("default mode is off", kCapsuleSpeedRateDefault == CapsuleRateMode::Off || UTPP_CAPSULE_SPEED_RATE_DEFAULT != 0);
        WithScale a;
        WithoutScale b;
        WriteRateSpeedScale(a, 1.2f);
        WriteRateSpeedScale(b, 1.2f);
        Check("rate scale written into a state that declares it", a.rateSpeedScale == 1.2f && b.other == 0.0f);
        WriteRateScaleScope(a, 2);
        WriteRateScaleScope(b, 2);
        Check("rate scale scope written into a state that declares it", a.rateScaleScope == 2 && b.other == 0.0f);
        motiontraj::States st;
        st.moveDirection.current = MoveDirection::RL;
        st.moveDirection.last = MoveDirection::F;
        st.moveDirection.recent = MoveDirection::F;
        st.isStartingDirectional = true;
        WithDirection w;
        WriteDirectionalInputs(w, st);
        Check("directional inputs written into a state that declares them", w.moveDirection == 4 && w.moveDirectionLast == 0 &&
                                                                                w.moveDirectionRecent == 0 && w.isStartingDirectional &&
                                                                                !w.isPivotingDirectional);
    }
    // defaults: every new option is off
    {
        const MapConfig d;
        Check("MapConfig defaults: directional inputs and capsule rate off",
              !d.moveDirection && !d.directionalPredicates && (d.capsuleSpeedRate == CapsuleRateMode::Off || UTPP_CAPSULE_SPEED_RATE_DEFAULT != 0));
        const motion::ChooserState cs;
        Check("capsule rate scope defaults: every pick in the map, no scale in the chooser state",
              d.capsuleRateScope == 0 && cs.rateSpeedScale == 1.0f && cs.rateScaleScope == 0);
    }
    std::printf("%d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

// Directional locomotion inputs (motiontraj::UpdateMoveDirection, IsStartingDirectional, IsPivotingDirectional, IsCircling), the
// capsule play-rate scale modes (CapsuleRateScaleForMode, WriteRateSpeedScale) and the speed converter (speedconv::Step,
// MapConfig::speedConverter). No data files.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <vector>

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

static float HeadingDeg(V3 v) { return std::atan2(v.y, v.x) * 57.2957795f; }

// A run along world +y (game m/s), then the stick and the capsule velocity turn by aTurnDeg at once (the game turns the capsule within a
// feed). Returns the converted query velocity past (-0.3..-0.2 s) of the trajectory and the turn angle 0.1 s after the turn.
struct TurnRun
{
    V3 past, current;
    float turnAngle = 0.0f;
    std::vector<float> q; // every chooser output and query sample (bit identity)
};
static TurnRun RunTurn(int aConverter, float aTurnDeg)
{
    const float dt = 1.0f / 60.0f;
    MapConfig mc;
    mc.speedConverter = aConverter;
    MapState ms;
    TurnRun out;
    const int before = 90, after = 8;
    for (int i = 0; i < before + after; ++i)
    {
        const float yaw = i >= before ? aTurnDeg : 0.0f; // integ yaw convention: forward(psi) = (-sin psi, cos psi)
        const float r = yaw / 57.2957795f;
        const float sp = std::fmin(1.0f, float(i) / 30.0f) * 3.85f;
        const float vel[3] = {-std::sin(r) * sp, std::cos(r) * sp, 0.0f}, mv[2] = {0.0f, 1.0f};
        MotionFeed f = MakeMotionFeed(vel, mv, dt, 0u, 0, AirFeed{}, yaw, yaw);
        f.psmDetailed = kPsmStand;
        MapContext ctx;
        ctx.rootYawValid = true;
        const motion::ChooserState cs = MapToChooser(f, mc, ms, ctx);
        out.q.insert(out.q.end(), {float(cs.gait), float(cs.movementState), cs.isStarting ? 1.f : 0.f, cs.speed2dCmS});
        for (int k = -2; k <= 15; ++k)
        {
            const motion::TrajSample s = QuerySample(ms.traj, 0.1f * float(k));
            out.q.insert(out.q.end(), {s.x, s.y, s.z, s.yaw});
        }
    }
    out.past = ms.traj.VelocityBetween(-0.1f, 0.0f);
    out.current = ms.traj.VelocityBetween(0.0f, 0.1f);
    out.turnAngle = ms.st.turnAngle;
    return out;
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
    // speed converter (speedconv::Step)
    {
        const float dt = 1.0f / 60.0f;
        const speedconv::Limits lim;
        speedconv::State s;
        V3 v{};
        const V3 run{500.0f, 0.0f, 0.0f};
        for (int i = 0; i < 120; ++i)
            v = speedconv::Step(s, run, true, true, dt, lim);
        Check("converter: a steady game velocity passes unchanged", v.x == 500.0f && v.y == 0.0f && v.z == 0.0f);
        // the game reverses the capsule in one feed; the converted velocity turns along the friction arc
        const V3 back{-500.0f, 0.0f, 0.0f};
        speedconv::State r = s;
        V3 a = speedconv::Step(r, back, true, true, dt, lim);
        Check("converter: one feed after a reversal the velocity still points forward", a.x > 300.0f);
        int feeds = 1;
        while (feeds < 120 && a.x > -400.0f)
        {
            a = speedconv::Step(r, back, true, true, dt, lim);
            ++feeds;
        }
        Check("converter: a reversal reaches 80 % of the new velocity in 0.2-1.0 s (game: one feed)", feeds * dt > 0.2f && feeds * dt < 1.0f);
        // 90 deg turn: the heading after 0.1 s is part way
        speedconv::State t = s;
        V3 b{};
        for (int i = 0; i < 6; ++i)
            b = speedconv::Step(t, V3{0.0f, 500.0f, 0.0f}, true, true, dt, lim);
        const float h = HeadingDeg(b);
        Check("converter: 0.1 s into a 90 deg turn the heading is part way (10-80 deg)", h > 10.0f && h < 80.0f);
        // no input: braking at most 2000 cm/s^2, never below the game speed
        speedconv::State k = s;
        const V3 c = speedconv::Step(k, V3{}, false, true, 0.1f, lim);
        Check("converter: no-input braking 2000 cm/s^2", std::fabs(c.x - 300.0f) < 0.01f);
        // speed not shaped: a straight speed-up follows the game at once, the shaped one is limited to the acceleration
        speedconv::State p0 = s, p1 = s;
        const V3 fast{700.0f, 0.0f, 0.0f};
        const V3 g0 = speedconv::Step(p0, fast, true, true, dt, lim, false), g1 = speedconv::Step(p1, fast, true, true, dt, lim, true);
        Check("converter: aligned speed-up follows the game speed unless the speed is shaped",
              g0.x == 700.0f && std::fabs(g1.x - (500.0f + 800.0f * dt)) < 0.01f);
        // direction only: a 90 deg turn keeps the game speed with the heading part way; an exact reversal does not stall
        speedconv::State q0 = s;
        V3 h0{};
        for (int i = 0; i < 6; ++i)
            h0 = speedconv::Step(q0, V3{0.0f, 450.0f, 0.0f}, true, true, dt, lim, false, true);
        Check("converter direction only: 0.1 s into a 90 deg turn the speed is the game speed, the heading part way",
              std::fabs(motiontraj::SizeXY(h0) - 450.0f) < 0.01f && HeadingDeg(h0) > 10.0f && HeadingDeg(h0) < 80.0f);
        speedconv::State q1 = s;
        V3 h1{};
        for (int i = 0; i < 60; ++i)
            h1 = speedconv::Step(q1, back, true, true, dt, lim, false, true);
        Check("converter direction only: a reversal turns through within 1 s", h1.x < -440.0f);
        // steady-speed filter: a held speed is steady, a start ramp is not
        speedconv::State f0;
        bool steady = false, ramp = false;
        for (int i = 0; i < 60; ++i)
            steady = speedconv::UpdateSteady(f0, 500.0f + ((i & 1) ? 2.5f : -2.5f), dt, true);
        speedconv::State f1;
        for (int i = 0; i < 30; ++i)
            ramp = speedconv::UpdateSteady(f1, 700.0f * float(i) * dt, dt, true) || ramp;
        Check("converter steady filter: a held speed with feed jitter is steady, a start ramp is not", steady && !ramp);
        Check("converter steady filter: an ineligible feed is not steady", !speedconv::UpdateSteady(f0, 500.0f, dt, false));
        // inactive / divergence: the game velocity
        speedconv::State u = s;
        const V3 d = speedconv::Step(u, back, true, false, dt, lim);
        Check("converter: an inactive feed is the game velocity and resets", d.x == -500.0f && u.vel.x == -500.0f);
        speedconv::State w = s;
        const V3 e = speedconv::Step(w, V3{-1500.0f, 0.0f, 0.0f}, true, true, dt, lim);
        Check("converter: a divergence above the limit resets to the game velocity", e.x == -1500.0f);
    }
    // speed converter in MapToChooser: off = identical, on = curved history and a larger turn angle right after a game turn
    {
        const TurnRun off = RunTurn(0, 90.0f), off2 = RunTurn(0, 90.0f), on = RunTurn(speedconv::kQuery, 90.0f);
        Check("converter off: identical runs", off.q == off2.q);
        // the game capsule already moves along the new direction (world -x), the converted one is still part way
        const float hOff = HeadingDeg(off.past), hOn = HeadingDeg(on.past);
        Check("converter off: the last 0.1 s of history follows the turned capsule", std::fabs(std::fabs(hOff) - 180.0f) < 5.0f);
        Check("converter on: the last 0.1 s of history is part way through the turn", std::fabs(hOn) > 95.0f && std::fabs(hOn) < 175.0f);
        Check("converter on: the turn angle stays larger than with the game velocity", std::fabs(on.turnAngle) > std::fabs(off.turnAngle) + 10.0f);
        // straight run: the converter changes nothing once the speed holds
        const TurnRun s0 = RunTurn(0, 0.0f), s1 = RunTurn(speedconv::kQuery, 0.0f);
        const size_t tail = 8 * 76;
        Check("converter on: a steady straight run queries the game velocity", s0.q.size() == s1.q.size() &&
              std::equal(s0.q.end() - tail, s0.q.end(), s1.q.end() - tail, [](float x, float y) { return std::fabs(x - y) < 0.01f; }));
        MapConfig loop;
        loop.speedConverter = speedconv::kQuery | speedconv::kLoopRate;
        Check("converter loop rate: capsule rate Both on loop clips while capsuleSpeedRate is off",
              EffectiveCapsuleRate(loop) == CapsuleRateMode::Both && EffectiveCapsuleRateScope(loop) == 1);
        loop.capsuleSpeedRate = CapsuleRateMode::RaiseOnly;
        loop.capsuleRateScope = 2;
        Check("converter loop rate: an explicit capsuleSpeedRate wins",
              EffectiveCapsuleRate(loop) == CapsuleRateMode::RaiseOnly && EffectiveCapsuleRateScope(loop) == 2);
        const MapConfig d;
        Check("MapConfig defaults: speed converter off", d.speedConverter == 0 && EffectiveCapsuleRate(d) == d.capsuleSpeedRate);
    }
    std::printf("%d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

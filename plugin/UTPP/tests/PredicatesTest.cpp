// port: table-driven boundary tests of src/mm/MotionTrajectory.hpp (motion trajectory + ABP predicates, report 3.3 / 3.4).
// No data files.
#include <cmath>
#include <cstdio>

#include "../src/mm/MotionTrajectory.hpp"

using namespace utpp::mm::motiontraj;

static int g_pass = 0, g_fail = 0;
static void Check(bool c, const char* what)
{
    if (c)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL %s\n", what);
    }
}

int main()
{
    // units: 4 m/s -> 400 cm/s horizontal (scale 1), 0.5 scale -> 200; vertical never scaled
    {
        const float v[3] = {4.0f, 0.0f, -7.0f};
        const V3 a = GameVelocityToCm(v, 1.0f), b = GameVelocityToCm(v, 0.5f);
        Check(a.x == 400.0f && a.z == -700.0f, "m/s -> cm/s");
        Check(b.x == 200.0f && b.z == -700.0f, "speedScale horizontal only");
    }
    // IsMoving: tolerance 10 per component (inclusive equal = zero), acceleration must be non-zero
    struct MoveRow { V3 fut, acc; bool exp; const char* n; };
    const MoveRow mr[] = {
        {{10.0f, 0, 0}, {800, 0, 0}, false, "fut 10 = zero"},
        {{10.01f, 0, 0}, {800, 0, 0}, true, "fut 10.01 moving"},
        {{0, -10.01f, 0}, {800, 0, 0}, true, "fut y -10.01 moving"},
        {{400, 0, 0}, {0, 0, 0}, false, "no acceleration -> idle"},
        {{9, 9, 9}, {1, 0, 0}, false, "all comps <= 10"},
    };
    for (const auto& r : mr)
        Check(IsMoving(r.fut, r.acc) == r.exp, r.n);
    // IsStarting: |fut|xy >= |vel|xy + 100, not in Pivots
    Check(IsStarting(true, {300, 0, 0}, {200, 0, 0}, false), "start at +100");
    Check(!IsStarting(true, {299.9f, 0, 0}, {200, 0, 0}, false), "start below +100");
    Check(!IsStarting(true, {300, 0, 0}, {200, 0, 0}, true), "start blocked by Pivots tag");
    Check(!IsStarting(false, {300, 0, 0}, {200, 0, 0}, false), "start needs moving");
    // IsPivoting: 45 orient / 30 strafe (inclusive)
    auto dir = [](float deg) { return V3{400.0f * std::cos(deg / kRad2Deg), 400.0f * std::sin(deg / kRad2Deg), 0}; };
    Check(IsPivoting(true, dir(45.01f), dir(0), RotationMode::OrientToMovement), "pivot orient 45");
    Check(!IsPivoting(true, dir(44.9f), dir(0), RotationMode::OrientToMovement), "pivot orient 44.9");
    Check(IsPivoting(true, dir(30.01f), dir(0), RotationMode::Strafe), "pivot strafe 30");
    Check(!IsPivoting(true, dir(29.9f), dir(0), RotationMode::Strafe), "pivot strafe 29.9");
    Check(std::fabs(TurnAngle(dir(170), dir(-170)) + 20.0f) < 1e-3f, "turn angle wraps");
    // TIP / Spin
    Check(ShouldTurnInPlace(RootDeltaYaw(50, 0), true, 1, 1, false), "tip 50 aiming");
    Check(!ShouldTurnInPlace(RootDeltaYaw(49.9f, 0), true, 1, 1, false), "tip 49.9");
    Check(ShouldTurnInPlace(60, false, 0, 1, false) && !ShouldTurnInPlace(60, false, 0, 0, false), "tip just stopped");
    Check(ShouldSpinTransition(130, 150, false) && !ShouldSpinTransition(129.9f, 150, false) &&
              !ShouldSpinTransition(130, 149.9f, false) && !ShouldSpinTransition(130, 150, true),
          "spin 130 / 150 / pivots");
    // lands
    Check(JustLandedLight(true, -699.9f) && !JustLandedHeavy(true, -699.9f), "land light < 700");
    Check(JustLandedHeavy(true, -700.0f) && !JustLandedLight(true, -700.0f), "land heavy >= 700");
    // J1: the heavy threshold scales with the game gravity (same 2.5 m drop); motion gravity / invalid -> exactly 700
    {
        Check(HeavyLandThresholdCmS(kMotionGravityMs2) == kHeavyLand && std::fabs(HeavyLandThresholdCmS(9.81f) - 700.0f) <= 0.5f,
              "heavy threshold at 9.81 = 700");
        const float t9 = HeavyLandThresholdCmS(9.81f);
        Check(JustLandedLight(true, -699.9f, t9) && !JustLandedHeavy(true, -699.9f, t9), "land light < 700 at 9.81 (3-arg)");
        Check(JustLandedHeavy(true, -700.0f, t9) && !JustLandedLight(true, -700.0f, t9), "land heavy >= 700 at 9.81 (3-arg)");
        const float t16 = HeavyLandThresholdCmS(16.0f);
        std::printf("heavy threshold g 16: %.2f cm/s\n", static_cast<double>(t16));
        Check(JustLandedLight(true, -800.0f, t16) && !JustLandedHeavy(true, -800.0f, t16), "g 16: -800 (hood jump-off) light");
        Check(JustLandedLight(true, -876.0f, t16) && !JustLandedHeavy(true, -876.0f, t16), "g 16: -876 (roof jump-off) light");
        Check(JustLandedHeavy(true, -990.0f, t16) && !JustLandedLight(true, -990.0f, t16), "g 16: -990 (3 m drop) heavy");
        Check(!JustLandedLight(false, -800.0f, t16) && !JustLandedHeavy(false, -990.0f, t16), "g 16: no flags outside JustLanded");
        Check(HeavyLandThresholdCmS(std::nanf("")) == kHeavyLand && HeavyLandThresholdCmS(50.0f) == kHeavyLand &&
                  HeavyLandThresholdCmS(7.9f) == kHeavyLand && HeavyLandThresholdCmS(-16.0f) == kHeavyLand,
              "invalid gravity (NaN / 50 / 7.9 / -16) -> 700");
    }
    Check(PlayLand(0, 1) && !PlayLand(0, 0) && !PlayLand(1, 1), "PlayLand");
    Check(PlayMovingLand(0, 1, 120) && !PlayMovingLand(0, 1, 120.1f), "PlayMovingLand 120");
    // prediction: run 400 cm/s, stick released (acc 0) -> future velocity brakes to zero, IsMoving false while capsule moves
    {
        MotionCmc c;
        Trajectory t;
        const V3 vel{400, 0, 0};
        const Derived d = MakeDerived(c, 1, 0, 0.0f, 400.0f, V3{}, true, false);
        for (int i = 0; i < 5; ++i)
            t.Update(1.0f / 60.0f, vel, V3{}, d);
        const V3 fut = t.FutureVelocity();
        Check(SizeXY(fut) < SizeXY(vel), "stop: future slower than capsule");
        Check(!IsMoving(fut, V3{}), "stop: Idle while capsule at 400");
        std::printf("stop future %.1f cm/s (capsule 400)\n", SizeXY(fut));
        // held stick from rest -> start (future >= velocity + 100)
        Trajectory s;
        const V3 acc{800, 0, 0};
        const Derived ds = MakeDerived(c, 1, 0, 1.0f, 0.0f, acc, true, false);
        s.Update(1.0f / 60.0f, V3{}, acc, ds);
        const V3 f2 = s.FutureVelocity();
        Check(IsMoving(f2, acc) && IsStarting(true, f2, V3{}, false), "start from rest");
        std::printf("start future %.1f cm/s\n", SizeXY(f2));
    }
    // ---- expected values below are hand-derived from the UE / motion formulas cited in MotionTrajectory.hpp
    const float kDt = 1.0f / 60.0f;
    const double g = 980.0;
    auto near = [](double a, double b, double tol) { return std::fabs(a - b) <= tol; };
    // (1) braking: CDO bUseSeparateBrakingFriction True, BrakingFriction unset (UE ctor 0), factor 1 -> friction 0; decel 2000
    // without input. Zero friction = one sub-step per 0.1 s (PSTL l. 166): 500 -> 300 -> 100 -> -100 = reversal -> 0.
    // Continuous stop time 500 / 2000 = 0.25 s lies in the third step (0.2, 0.3].
    {
        MotionCmc c;
        const Derived d = MakeDerived(c, 1, 0, 0.0f, 500.0f, V3{}, true, false);
        Check(d.friction == 0.0f && d.brakingDeceleration == 2000.0f, "brake: friction 0 / decel 2000");
        const V3 v1 = StepGround(0.1f, {500, 0, 0}, {}, d), v2 = StepGround(0.1f, v1, {}, d), v3 = StepGround(0.1f, v2, {}, d);
        Check(near(v1.x, 300, 1e-3) && near(v2.x, 100, 1e-3) && v3.x == 0.0f, "brake: 500 -> 300 -> 100 -> 0 per 0.1 s");
        // (the continuous stop time 500 / 2000 = 0.25 s in (0.2, 0.3] is a derivation note, not a check: v3 == 0 above
        // is the code's zeroing step; a literal-only Check was removed, verifier finding 2026-10-13)
        Trajectory t;
        t.Update(kDt, {500, 0, 0}, {}, d);
        t.Update(kDt, {500, 0, 0}, {}, d);
        const float x0 = t.Prediction(0).pos.x;
        Check(near(t.Prediction(1).pos.x - x0, 50, 1e-3) && near(t.Prediction(2).pos.x - x0, 80, 1e-3) &&
                  near(t.Prediction(3).pos.x - x0, 90, 1e-3) && near(t.Prediction(15).pos.x - x0, 90, 1e-3),
              "brake: predicted stop distance 50 / 80 / 90 cm");
    }
    // (2) history: last frame's current sample at t = 0, current at +DeltaTime (PSTL l. 297-317, 489) -> constant velocity
    // gives Trj_CurrentVelocity == v (was 0.958 v with the record at -dt).
    {
        MotionCmc c;
        const V3 vel{500, 0, 0}, acc{800, 0, 0};
        const Derived d = MakeDerived(c, 1, 0, 1.0f, 500.0f, acc, true, false); // maxSpeed 500 -> prediction stays 500
        Trajectory t, tg;
        for (int i = 0; i < 40; ++i)
        {
            t.Update(kDt, vel, acc, d);
            tg.Update(kDt, vel, acc, d, 980.0f, false, 0.0f);
        }
        const V3 cv = t.CurrentVelocity(), pv = t.PastVelocity(), fv = t.FutureVelocity();
        Check(near(cv.x, 500, 1e-3) && near(cv.y, 0, 1e-3) && near(cv.z, 0, 1e-3), "history: CurrentVelocity == v");
        Check(near(pv.x, 500, 1e-3) && near(fv.x, 500, 1e-3), "history: Past / Future == v");
        Check(t.History(kHistoryCount - 1).t == 0.0f && near(t.History(kHistoryCount - 2).t, -kDt, 1e-6) &&
                  near(t.Prediction(0).t, kDt, 1e-7),
              "history: record at 0, previous at -dt, current at +dt");
        std::printf("current velocity %.4f (v 500)\n", cv.x);
        // gravity pass on the ground: flat floor at the current position [P], samples at floor + 0.01 (l. 939), steady
        // FallHeight 0 -> TimeToLand = 0
        const V3 cg = tg.CurrentVelocity();
        Check(near(cg.x, 500, 1e-3) && near(cg.z, 0, 1e-4), "ground gravity: CurrentVelocity == v, z 0");
        Check(near(tg.Prediction(7).pos.z - tg.Prediction(0).pos.z, 0, 1e-5) && tg.LandPredicted() && near(tg.TimeToLand(), 0, 1e-6),
              "ground gravity: flat samples, TimeToLand 0");
    }
    // (3) fall: apex (vz 0) in the air, floor 100 cm below. Frame 2: P(0) = frame-1 current sample after gravity
    // (-1/2 g dt^2), first hit at k = 5 (t = dt + 0.5: drop 130.8 >= 100), FallHeight = 99.99 - 1/2 g dt^2,
    // TimeToLand = 0 / g + sqrt(0 + 2 g FallHeight) / g (l. 941-949).
    {
        MotionCmc c;
        const Derived d = MakeDerived(c, 1, 0, 0.0f, 0.0f, V3{}, true, true);
        Trajectory t;
        t.Update(kDt, {}, {}, d, 980.0f, true, -100.0f);
        t.Update(kDt, {}, {}, d, 980.0f, true, -100.0f);
        const double fh = 99.99 - 0.5 * g * double(kDt) * double(kDt);
        const double ttl = std::sqrt(2.0 * g * fh) / g;
        Check(near(t.TimeToLand(), ttl, 2e-5) && t.LandPredicted(), "fall: TimeToLand from FallHeight after history fix");
        Check(near(t.Prediction(4).pos.z, -0.5 * g * std::pow(double(kDt) + 0.4, 2.0), 1e-3) &&
                  near(t.Prediction(5).pos.z, -99.99, 1e-3) && near(t.Prediction(6).pos.z, -99.99, 1e-3) &&
                  near(t.Prediction(15).pos.z, -99.99, 1e-3),
              "fall: free fall until the hit, then on the floor + 0.01");
        Check(near(t.LandSpeed(), -g * ttl, 1e-2), "fall: LandSpeed = v0 - g ttl");
        std::printf("fall ttl %.6f (expected %.6f)\n", t.TimeToLand(), ttl);
        // jump vz +300: frame 2 position z = 300 dt = 5, floor at 5 - 100, P(0) z = -1/2 g dt^2
        Trajectory j;
        j.Update(kDt, {0, 0, 300}, {}, d, 980.0f, true, -100.0f);
        j.Update(kDt, {0, 0, 300}, {}, d, 980.0f, true, -100.0f);
        const double fhj = std::fabs((300.0 * kDt - 100.0 + 0.01) - (-0.5 * g * double(kDt) * double(kDt)));
        const double ttlj = 300.0 / g + std::sqrt(300.0 * 300.0 + 2.0 * g * fhj) / g;
        Check(near(j.TimeToLand(), ttlj, 2e-5), "jump: TimeToLand with v0 300");
        // no floor in the air -> last sample time (l. 900)
        Trajectory n;
        n.Update(kDt, {}, {}, d, 980.0f, false, 0.0f);
        Check(!n.LandPredicted() && near(n.TimeToLand(), kDt + 1.5, 1e-5), "fall: no floor -> last sample time");
    }
    // (4) MaxControllerYawRate: control yaw +5 deg per 1/60 s = 300 deg/s, Idle clamps to 100 -> 10 deg per 0.1 s step;
    // the acceleration rotates before each step (l. 451-458). Moving (0) -> no rotation.
    {
        MotionCmc c;
        const V3 acc{800, 0, 0};
        auto run = [&](bool aOrient, GenerationData aGen, float aYawStep, float aActor) {
            const Derived d = MakeDerived(c, 1, 0, 1.0f, 0.0f, acc, aOrient, false);
            Trajectory t;
            t.Update(kDt, {}, acc, d, 0.0f, false, 0.0f, FacingInput{true, aActor, 0.0f, aGen});
            t.Update(kDt, {}, acc, d, 0.0f, false, 0.0f, FacingInput{true, aActor, aYawStep, aGen});
            return t;
        };
        const Trajectory si = run(false, kGenIdle, 5.0f, 50.0f);
        Check(near(si.ControllerYawRate(), 100, 1e-2), "yaw rate: idle clamp 100");
        Check(near(YawOf(si.Prediction(2).pos - si.Prediction(1).pos), 10, 1e-3), "yaw rate: acceleration rotated 10 deg");
        Check(near(si.Prediction(0).facing, 50, 1e-4) && near(si.Prediction(1).facing, 60, 1e-3) &&
                  near(si.Prediction(3).facing, 80, 1e-3) && near(si.Prediction(15).facing, NormalizeAxis(200), 1e-3),
              "strafe facing: actor yaw + 10 deg per step");
        const Trajectory sn = run(false, kGenIdle, -5.0f, 50.0f);
        Check(near(sn.ControllerYawRate(), -100, 1e-2) && near(sn.Prediction(2).facing, 30, 1e-3), "yaw rate: -100");
        const Trajectory sm = run(false, kGenMoving, 5.0f, 50.0f);
        Check(sm.ControllerYawRate() == 0.0f && near(sm.Prediction(15).facing, 50, 1e-4) &&
                  near(YawOf(sm.Prediction(3).pos - sm.Prediction(2).pos), 0, 1e-4),
              "yaw rate: moving 0 -> no rotation");
        // (6) OrientToMovement: RotateTowardsMovementSpeed 0 -> QInterpConstantTo jumps to the acceleration yaw
        // (UnrealMath.cpp l. 2727-2731), not a RotationRate convergence
        const Trajectory oi = run(true, kGenIdle, 5.0f, 50.0f);
        Check(near(oi.Prediction(0).facing, 50, 1e-4) && near(oi.Prediction(1).facing, 10, 1e-3) &&
                  near(oi.Prediction(4).facing, 40, 1e-3),
              "orient facing: snaps to the rotated acceleration");
        const Trajectory om = run(true, kGenMoving, 5.0f, 50.0f);
        Check(near(om.Prediction(1).facing, 0, 1e-4) && near(om.Prediction(15).facing, 0, 1e-4), "orient facing: moving -> acc yaw");
        // FacingYawAt: FastLerp + normalise of yaw quats at the midpoint = bisector 25
        Check(near(om.FacingYawAt(kDt + 0.05f), 25, 1e-3), "FacingYawAt midpoint");
        // history facing = last frame's current sample facing (actor yaw)
        Check(near(om.History(kHistoryCount - 1).facing, 50, 1e-4), "history facing from actor yaw");
        // VelocityAt = (P(t) - P(t - 1/60)) * 60: constant velocity -> v
        Trajectory cvt;
        const Derived dc = MakeDerived(c, 1, 0, 1.0f, 500.0f, acc, true, false);
        for (int i = 0; i < 5; ++i)
            cvt.Update(kDt, {500, 0, 0}, acc, dc);
        Check(near(cvt.VelocityAt(0.7f).x, 500, 1e-2) && near(cvt.VelocityAt(-0.03f).x, 500, 1e-2), "VelocityAt constant v");
    }
    // (5) CalculateMaxSpeed with Curve_StrafeSpeedMap (t3d keys) in strafe; forward column in OrientToMovement
    {
        MotionCmc c;
        Check(near(CalculateDirection({0, 100, 0}, 0.0f), 90, 1e-3) && near(CalculateDirection({0, -100, 0}, 0.0f), -90, 1e-3) &&
                  CalculateDirection({}, 0.0f) == 0.0f,
              "CalculateDirection");
        Check(near(StrafeSpeedMap(45), 0, 1e-6) && near(StrafeSpeedMap(62.5f), 0.5, 1e-5) && near(StrafeSpeedMap(90), 1, 1e-6) &&
                  near(StrafeSpeedMap(117.5f), 1.5, 1e-5) && near(StrafeSpeedMap(180), 2, 1e-6),
              "StrafeSpeedMap keys");
        Check(near(MotionMaxSpeed(c, 1, 0, true, 0), 500, 1e-3) && near(MotionMaxSpeed(c, 1, 0, true, 90), 350, 1e-3) &&
                  near(MotionMaxSpeed(c, 1, 0, true, -180), 300, 1e-3) && near(MotionMaxSpeed(c, 1, 0, true, 62.5f), 425, 1e-2) &&
                  near(MotionMaxSpeed(c, 1, 0, false, 90), 500, 1e-3),
              "max speed run columns 500 / 350 / 300");
        Check(near(MotionMaxSpeed(c, 0, 0, true, 135), 150, 1e-3) && near(MotionMaxSpeed(c, 2, 0, true, 90), 700, 1e-3) &&
                  near(MotionMaxSpeed(c, 1, 1, true, 90), 200, 1e-3),
              "max speed walk back 150 / sprint 700 / crouch strafe 200");
        Check(near(MakeDerived(c, 1, 0, 1.0f, 300.0f, {0, 800, 0}, false, false, 90.0f).maxSpeed, 350, 1e-3),
              "MakeDerived strafe column");
    }
    std::printf("predicates test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

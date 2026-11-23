// Aim rotation sub-mode (MapConfig::aimRotation, mm.aimRotation): while aiming in the strafe rotation mode IsPivoting uses the Aim
// pivot threshold (MapConfig::aimPivotDeg, default 10; the reference uses 0) and the pivot / start elapsed-time gate
// (MapConfig::aimPivotGateS 0.5 s).
// Checks: the predicate thresholds 0 / 10 / 15 against the strafe 30; the default threshold ignores the small turn angle of straight
// movement (within 2 deg) that threshold 0 reports as a pivot; the velocity floor; the gate holds a new pivot while a pivot / start
// pick is young and releases it after, a gate of 0 holds nothing; MapToChooser off (default) and on without aiming give the strafe IsPivoting and the same
// chooser state, bit for bit; on while aiming follows the reported turn angle at the configured threshold; the gate reaches
// MapToChooser through MapContext (currentDbPivots / currentDbStarts / currentPickAgeS).
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
using namespace utpp::mm;
namespace
{
int pass = 0, fail = 0;
void Check(bool ok, const char* name)
{
    (ok ? pass : fail)++;
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
}
motiontraj::V3 Dir(float aDeg, float aLen = 300.0f)
{
    const float r = aDeg * 3.14159265f / 180.0f;
    return motiontraj::V3{std::cos(r) * aLen, std::sin(r) * aLen, 0.0f};
}
adapter::MotionFeed Feed(bool aAim, float aStickDeg)
{
    adapter::MotionFeed f;
    f.dt = 1.0f / 60.0f;
    f.psmDetailed = adapter::kPsmStand;
    f.aiming = aAim;
    f.velWorld[1] = 2.0f; // forward at 2 m/s (integ yaw 0 = +Y)
    const float r = aStickDeg * 3.14159265f / 180.0f;
    f.move[0] = std::sin(r);
    f.move[1] = std::cos(r);
    return f;
}
struct Frame
{
    adapter::MapState state;
    motion::ChooserState cs;
};
// 40 forward frames, then one frame with the stick turned by aStickDeg
Frame Run(const adapter::MapConfig& aC, bool aAim, float aStickDeg, const adapter::MapContext& aCtx = {})
{
    Frame r;
    for (int i = 0; i < 40; ++i)
        r.cs = adapter::MapToChooser(Feed(aAim, 0.0f), aC, r.state, {});
    r.cs = adapter::MapToChooser(Feed(aAim, aStickDeg), aC, r.state, aCtx);
    return r;
}
bool SameChooser(const motion::ChooserState& a, const motion::ChooserState& b)
{
    return a.isPivoting == b.isPivoting && a.isStarting == b.isStarting && a.gait == b.gait && a.stance == b.stance &&
           a.movementState == b.movementState && a.movementMode == b.movementMode && a.speed2dCmS == b.speed2dCmS &&
           a.shouldTurnInPlace == b.shouldTurnInPlace && a.shouldSpinTransition == b.shouldSpinTransition;
}
} // namespace

int main()
{
    using motiontraj::IsPivoting;
    using motiontraj::IsPivotingAim;
    using motiontraj::RotationMode;
    const motiontraj::V3 vel = Dir(0.0f);
    // predicate thresholds
    Check(IsPivotingAim(true, Dir(5.0f), vel, 0.0f, false, 1.0e9f, 0.5f), "aim 0: a 5 deg turn pivots");
    Check(IsPivotingAim(true, Dir(0.0f), vel, 0.0f, false, 1.0e9f, 0.5f), "aim 0: a straight run reports pivoting (any angle)");
    Check(!IsPivotingAim(true, Dir(10.0f), vel, 15.0f, false, 1.0e9f, 0.5f) && IsPivotingAim(true, Dir(15.5f), vel, 15.0f, false, 1.0e9f, 0.5f),
          "aim 15: 10 deg no, 15.5 deg yes");
    Check(!IsPivoting(true, Dir(20.0f), vel, RotationMode::Strafe) && IsPivotingAim(true, Dir(20.0f), vel, 15.0f, false, 1.0e9f, 0.5f),
          "a 20 deg turn: strafe 30 no, aim 15 yes");
    Check(!IsPivotingAim(false, Dir(90.0f), vel, 0.0f, false, 1.0e9f, 0.5f), "aim: not moving, no pivot");
    Check(!IsPivotingAim(true, Dir(90.0f), Dir(0.0f, 4.0f), 0.0f, false, 1.0e9f, 0.5f), "aim: velocity floor (no heading) kept");
    // elapsed-time gate
    Check(!IsPivotingAim(true, Dir(90.0f), vel, 0.0f, true, 0.3f, 0.5f), "gate: a pivot / start pick 0.3 s old holds a new pivot");
    Check(!IsPivotingAim(true, Dir(90.0f), vel, 0.0f, true, 0.5f, 0.5f), "gate: held at exactly 0.5 s (elapsed > 0.5 releases)");
    Check(IsPivotingAim(true, Dir(90.0f), vel, 0.0f, true, 0.51f, 0.5f), "gate: released after 0.5 s");
    Check(IsPivotingAim(true, Dir(90.0f), vel, 0.0f, false, 0.0f, 0.5f), "gate: a young loop pick does not hold");
    Check(IsPivotingAim(true, Dir(90.0f), vel, 0.0f, true, 0.01f, -1.0f), "gate: a negative gate time disables the gate");
    Check(IsPivotingAim(true, Dir(90.0f), vel, 0.0f, true, 0.0f, 0.0f), "gate: a gate of 0 holds nothing (not even the pick frame)");
    // default threshold: straight movement with a small turn angle (the game feed stays within about 2 deg) is no pivot
    {
        const adapter::MapConfig def;
        bool straightNo = true, zeroYes = true;
        for (float a : {-2.0f, -1.0f, -0.5f, 0.3f, 1.5f, 2.0f})
        {
            straightNo = straightNo && !IsPivotingAim(true, Dir(a), vel, def.aimPivotDeg, false, 1.0e9f, def.aimPivotGateS);
            zeroYes = zeroYes && IsPivotingAim(true, Dir(a), vel, 0.0f, false, 1.0e9f, def.aimPivotGateS);
        }
        Check(def.aimPivotDeg == motiontraj::kPivotAim && motiontraj::kPivotAim == 10.0f, "default aim threshold 10 deg");
        Check(straightNo, "default threshold: straight movement (|turn| <= 2 deg) is no pivot");
        Check(zeroYes, "threshold 0: the same straight movement reports a pivot");
        Check(IsPivotingAim(true, Dir(12.0f), vel, def.aimPivotDeg, false, 1.0e9f, def.aimPivotGateS) &&
                  !IsPivoting(true, Dir(12.0f), vel, RotationMode::Strafe),
              "default threshold: a 12 deg turn pivots (strafe 30 does not)");
    }

    // MapToChooser: off identity
    adapter::MapConfig off;
    adapter::MapConfig on = off;
    on.aimRotation = true;
    bool offSame = true, noAimSame = true, aimFlagOff = true;
    for (float a : {0.0f, 10.0f, 20.0f, 35.0f, 60.0f, 120.0f})
    {
        const Frame base = Run(off, true, a), explicitOff = Run(adapter::MapConfig{}, true, a);
        offSame = offSame && SameChooser(base.cs, explicitOff.cs) && !base.state.st.aimRotation &&
                  base.cs.isPivoting == IsPivoting(base.state.st.isMoving, base.state.st.trjFuture, base.state.st.velocity, RotationMode::Strafe);
        const Frame noAimOff = Run(off, false, a), noAimOn = Run(on, false, a);
        noAimSame = noAimSame && SameChooser(noAimOff.cs, noAimOn.cs) && !noAimOn.state.st.aimRotation &&
                    std::memcmp(&noAimOff.state.st.trjFuture, &noAimOn.state.st.trjFuture, sizeof(motiontraj::V3)) == 0;
        aimFlagOff = aimFlagOff && base.state.st.rotationMode == RotationMode::Strafe;
    }
    Check(offSame, "off (default): the strafe IsPivoting while aiming, aimRotation flag false");
    Check(noAimSame, "on without aiming: the same chooser state and trajectory as off");
    Check(aimFlagOff, "aiming keeps the strafe rotation mode");

    // MapToChooser on while aiming: IsPivoting follows the reported turn angle at the configured threshold
    for (float thr : {0.0f, 10.0f, 15.0f})
    {
        adapter::MapConfig c = on;
        c.aimPivotDeg = thr;
        bool consistent = true, flag = true, discriminating = false;
        for (float a : {0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 40.0f, 60.0f, 90.0f})
        {
            const Frame fr = Run(c, true, a), ref = Run(off, true, a);
            const auto& st = fr.state.st;
            const bool want = st.isMoving && st.hasVelocity && std::fabs(st.turnAngle) >= thr;
            consistent = consistent && fr.cs.isPivoting == want;
            flag = flag && st.aimRotation && st.rotationMode == RotationMode::Strafe;
            discriminating = discriminating || (fr.cs.isPivoting && !ref.cs.isPivoting);
        }
        std::printf("aim pivot threshold %.0f\n", static_cast<double>(thr));
        Check(consistent, "on, aiming: IsPivoting = moving && |turn angle| >= aim threshold");
        Check(flag, "on, aiming: aimRotation flag set, rotation mode still strafe");
        Check(discriminating, "on, aiming: some turn below the strafe 30 deg pivots at the aim threshold");
    }
    // the gate through MapContext
    {
        adapter::MapContext young;
        young.currentDbPivots = true;
        young.currentPickAgeS = 0.2f;
        adapter::MapContext youngStart;
        youngStart.currentDbStarts = true;
        youngStart.currentPickAgeS = 0.2f;
        adapter::MapContext old = young;
        old.currentPickAgeS = 0.8f;
        Check(!Run(on, true, 90.0f, young).cs.isPivoting && !Run(on, true, 90.0f, youngStart).cs.isPivoting,
              "on, aiming: a 90 deg turn during a young pivot / start pick does not pivot");
        Check(Run(on, true, 90.0f, old).cs.isPivoting, "on, aiming: the same turn after 0.5 s pivots");
        Check(Run(off, true, 90.0f, young).cs.isPivoting, "off: the context gate has no effect (strafe IsPivoting)");
    }
    std::printf("%d passed, %d failed\n", pass, fail);
    return fail == 0 ? 0 : 1;
}

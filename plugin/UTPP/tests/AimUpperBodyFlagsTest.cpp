// Optional aim / upper body behaviours (all default off):
//   1. WorldAimTargetFilter (mm/AimSpineYawFilter.hpp): smoothing the aim direction in world space keeps the root-relative AO exact
//      while the root turns, the same smoothing on the root-relative angles lags; AimSpineYawFilter takes world-tracked angles
//      without the root-relative jump band (mutation UTPP_POSEV2_AO_MUTATION 19 keeps the band: it MUST print a FAIL line);
//   2. LeanGateWant (mm/LeanRuntime.hpp): the default coverage is the former gate for every signal combination; the wide coverage
//      opens with a drawn weapon (body + head), while aiming (body only) and crouch moving, and stays closed for weapon actions;
//   3. NeutralAoBlend (mm/Strafe.hpp): the enable rule of the neutral aim offset and its fade.
#include <cmath>
#include <cstdio>

#include "mm/AimSpineYawFilter.hpp"
#include "mm/LeanRuntime.hpp"
#include "mm/Strafe.hpp"

namespace
{
int g_fail = 0;

void Check(bool aOk, const char* aWhat)
{
    std::printf("%s %s\n", aOk ? "ok  " : "FAIL", aWhat);
    if (!aOk)
        ++g_fail;
}

namespace mp = utpp::mm::motionpose;
namespace lrt = utpp::mm::leanrt;
namespace sf = utpp::mm::strafe;

void WorldTargetTest()
{
    constexpr float dt = 1.0f / 60.0f;
    mp::WorldAimTargetParams p;
    p.enabled = true;
    p.smoothHz = 12.0f;

    // Fixed world target at yaw 30 deg, pitch 5 deg; the root turns in place 0 -> 90 deg in 0.5 s, then holds for 0.5 s.
    mp::WorldAimTargetFilter world;
    float relSmoothed = 0.0f;
    bool relInit = false;
    float maxErrWorld = 0.0f, maxErrRel = 0.0f, steadyErrWorld = 0.0f;
    for (int i = 0; i <= 60; ++i)
    {
        const float t = static_cast<float>(i) * dt;
        const float root = 90.0f * std::min(1.0f, t / 0.5f);
        const float trueRel = mp::RootRelativeYawDeg(30.0f, root);
        float wy = 0.0f, wp = 0.0f;
        world.Step(true, 30.0f, 5.0f, dt, p, wy, wp);
        const float relW = mp::RootRelativeYawDeg(wy, root);
        // the same exponential on the root-relative angle (what smoothing the AO itself would do)
        if (!relInit)
        {
            relSmoothed = trueRel;
            relInit = true;
        }
        else
            relSmoothed += (trueRel - relSmoothed) * (1.0f - std::exp(-p.smoothHz * dt));
        maxErrWorld = std::max(maxErrWorld, std::fabs(relW - trueRel));
        maxErrRel = std::max(maxErrRel, std::fabs(relSmoothed - trueRel));
        if (t >= 0.5f)
            steadyErrWorld = std::max(steadyErrWorld, std::fabs(relW - trueRel));
        if (std::fabs(wp - 5.0f) > 1e-4f)
            Check(false, "world filter: a fixed target keeps its pitch");
    }
    std::printf("  90 deg turn in place: AO yaw error world-smoothed max %.4f deg (steady %.4f), root-relative-smoothed max %.2f deg\n",
                maxErrWorld, steadyErrWorld, maxErrRel);
    Check(maxErrWorld <= steadyErrWorld + 1e-3f && maxErrWorld < 1e-3f, "world filter: error during the turn <= steady value");
    Check(maxErrRel > 5.0f, "root-relative smoothing lags during the same turn (control)");

    // smoothHz 0: exact tracking of camera motion below the jump band
    mp::WorldAimTargetParams exact;
    exact.enabled = true;
    mp::WorldAimTargetFilter f0;
    bool exactOk = true;
    for (int i = 0; i < 120; ++i)
    {
        const float cam = mp::WrapDeg180(-170.0f + 3.0f * static_cast<float>(i)); // crosses +-180
        float wy = 0.0f, wp = 0.0f;
        f0.Step(true, cam, 0.0f, dt, exact, wy, wp);
        exactOk = exactOk && std::fabs(mp::WrapDeg180(wy - cam)) < 1e-3f;
    }
    Check(exactOk, "world filter smoothHz 0: exact tracking across the +-180 wrap");

    // a camera jump of 100 deg in one frame is caught up at the AO rate (no one-frame swing)
    mp::WorldAimTargetFilter fj;
    float wy = 0.0f, wp = 0.0f;
    fj.Step(true, 0.0f, 0.0f, dt, p, wy, wp);
    fj.Step(true, 0.0f, 0.0f, dt, p, wy, wp);
    float prev = wy, maxStep = 0.0f;
    int frames = 0;
    for (int i = 0; i < 60; ++i)
    {
        fj.Step(true, 100.0f, 0.0f, dt, p, wy, wp);
        maxStep = std::max(maxStep, std::fabs(wy - prev));
        prev = wy;
        ++frames;
        if (std::fabs(wy - 100.0f) < 0.5f)
            break;
    }
    std::printf("  100 deg jump: max step %.2f deg/frame, settled in %d frames\n", maxStep, frames);
    Check(maxStep <= mp::kAoYawRateDegPerSec * dt + 1e-3f, "world filter: a jump is caught up at the AO rate");
    Check(std::fabs(wy - 100.0f) < 0.5f, "world filter: a jump settles");

    // a lost target holds the last direction; dt <= 0 does not move the state
    float hy = 0.0f, hp = 0.0f;
    Check(fj.Step(false, 0.0f, 0.0f, dt, p, hy, hp) && hy == wy, "world filter: a lost target holds the last direction");
    Check(fj.Step(true, -50.0f, 0.0f, 0.0f, p, hy, hp) && hy == wy, "world filter: dt 0 keeps the state");
}

// AimSpineYawFilter with world-tracked angles (the world aim target ran the jump band already): a step of the root-relative angles
// caused by the root (a recentred root under a still crosshair) is followed in the same frame; the side change of the clamp behind
// the back and the re-acquire after a latch still catch up at the AO rate. Mutation 19 runs the root-relative jump band anyway.
void WorldTrackedSpineTest()
{
    constexpr float dt = 1.0f / 60.0f;
    constexpr float clampDeg = 60.0f;
    const float maxStep = mp::kAoYawRateDegPerSec * dt + 1e-3f;
    mp::AimSpineYawFilter::Output o;
    // root recentred by 55 deg in one frame: 50 -> -5 (yaw) and pitch 10 -> -45 (a near-vertical target re-expressed)
    const auto rootStep = [&](bool aWorld, float& aYawAfter, float& aPitchAfter) {
        mp::AimSpineYawFilter f;
        for (int i = 0; i < 10; ++i)
            f.Step(true, 1.0f, 50.0f, 10.0f, dt, clampDeg, 0, false, o, aWorld);
        f.Step(true, 1.0f, -5.0f, -45.0f, dt, clampDeg, 0, false, o, aWorld);
        aYawAfter = o.yawDeg;
        aPitchAfter = o.pitchDeg;
    };
    float yW = 0.0f, pW = 0.0f, yR = 0.0f, pR = 0.0f;
    rootStep(true, yW, pW);
    rootStep(false, yR, pR);
    std::printf("  root step 55 deg: world-tracked yaw %.2f pitch %.2f, root-relative band yaw %.2f pitch %.2f\n", yW, pW, yR, pR);
    Check(std::fabs(yW + 5.0f) < 1e-3f && std::fabs(pW + 45.0f) < 1e-3f, "world-tracked angles follow a root step in the same frame");
    Check(std::fabs(yR - 50.0f) <= maxStep && std::fabs(yR - 50.0f) > 1.0f, "control: without it the jump band catches the step up");

    // side change behind the back (+100 -> -100 over one frame) is still caught up at the AO rate
    mp::AimSpineYawFilter fs;
    for (int i = 0; i < 10; ++i)
        fs.Step(true, 1.0f, 100.0f, 0.0f, dt, clampDeg, 0, false, o, true);
    const float before = o.yawDeg;
    fs.Step(true, 1.0f, -100.0f, 0.0f, dt, clampDeg, 0, false, o, true);
    Check(std::fabs(o.yawDeg - before) <= maxStep && o.yawDeg > 0.0f, "world-tracked: the clamp side change still catches up");

    // re-acquire after a lost target (latched) still catches up
    mp::AimSpineYawFilter fl;
    for (int i = 0; i < 10; ++i)
        fl.Step(true, 1.0f, 40.0f, 0.0f, dt, clampDeg, 0, false, o, true);
    fl.Step(false, 1.0f, 0.0f, 0.0f, dt, clampDeg, 0, false, o, true);
    const float latched = o.yawDeg;
    fl.Step(true, 1.0f, -40.0f, 0.0f, dt, clampDeg, 0, false, o, true);
    Check(std::fabs(o.yawDeg - latched) <= maxStep, "world-tracked: a re-acquire after the latch still catches up");
}

void LeanGateTest()
{
    // Default coverage = the former gate for every combination of the boolean signals
    bool same = true;
    for (unsigned m = 0; m < (1u << 11); ++m)
    {
        lrt::Signals s;
        s.moving = (m & 1u) != 0;
        s.aiming = (m & 2u) != 0;
        s.weaponBusy = (m & 4u) != 0;
        s.air = (m & 8u) != 0;
        s.crouch = (m & 16u) != 0;
        s.slide = (m & 32u) != 0;
        s.blocked = (m & 64u) != 0;
        s.head = (m & 128u) != 0;
        s.weaponAction = (m & 256u) != 0;
        s.weaponDrawn = (m & 512u) != 0;
        const bool vOk = (m & 1024u) != 0;
        const bool oldWant = !s.blocked && s.moving && !s.crouch && !s.air && !s.aiming && !s.weaponBusy && !s.slide && vOk;
        const bool oldHead = oldWant && s.head;
        const lrt::GateWant w = lrt::LeanGateWant(lrt::Coverage::Default, s, vOk);
        same = same && w.body == oldWant && w.head == oldHead;
    }
    Check(same, "lean default coverage: identical to the former gate (2048 combinations)");

    lrt::Signals base;
    base.moving = true;
    base.head = true;
    auto want = [&](lrt::Signals s) { return lrt::LeanGateWant(lrt::Coverage::Wide, s, true); };
    lrt::Signals drawn = base;
    drawn.weaponDrawn = drawn.weaponBusy = true;
    Check(want(drawn).body && want(drawn).head, "lean wide: weapon drawn = body and head");
    Check(!lrt::LeanGateWant(lrt::Coverage::Default, drawn, true).body, "lean default: weapon drawn closes the gate");
    lrt::Signals aim = drawn;
    aim.aiming = true;
    Check(want(aim).body && !want(aim).head, "lean wide: aiming = body only");
    lrt::Signals crouch = base;
    crouch.crouch = true;
    Check(want(crouch).body, "lean wide: crouch moving");
    lrt::Signals act = drawn;
    act.weaponAction = true;
    Check(!want(act).body && !want(act).head, "lean wide: weapon action closes the gate");
    for (int k = 0; k < 4; ++k)
    {
        lrt::Signals s = base;
        if (k == 0) s.air = true;
        if (k == 1) s.slide = true;
        if (k == 2) s.blocked = true;
        if (k == 3) s.moving = false;
        Check(!want(s).body, k == 0 ? "lean wide: air closed" : k == 1 ? "lean wide: slide closed" : k == 2 ? "lean wide: blocked closed" : "lean wide: idle closed");
    }
    Check(!lrt::LeanGateWant(lrt::Coverage::Wide, base, false).body, "lean wide: invalid velocity closed");
    lrt::Params p;
    Check(p.coverage == 0, "lean coverage default = Default");
}

void NeutralAoTest()
{
    sf::NeutralAoConfig cfg;
    sf::NeutralAoInput in;
    in.strafeFacing = true;
    in.aoYawDeg = in.prevAoYawDeg = 40.0f;
    Check(sf::NeutralAoTargetAlpha(in, cfg) == 0.0f, "neutral AO: disabled by default");
    cfg.enabled = true;
    Check(sf::NeutralAoTargetAlpha(in, cfg) == 1.0f, "neutral AO: strafe facing within 110 deg");
    sf::NeutralAoInput s = in;
    s.aiming = true;
    Check(sf::NeutralAoTargetAlpha(s, cfg) == 0.0f, "neutral AO: the Aiming AO has priority");
    s = in;
    s.strafeFacing = false;
    Check(sf::NeutralAoTargetAlpha(s, cfg) == 0.0f, "neutral AO: no strafe facing");
    s = in;
    s.aoYawDeg = s.prevAoYawDeg = 120.0f;
    Check(sf::NeutralAoTargetAlpha(s, cfg) == 0.0f, "neutral AO: |yaw| above 110 deg");
    s = in;
    s.slotWeight = 0.6f;
    Check(sf::NeutralAoTargetAlpha(s, cfg) == 0.0f, "neutral AO: upper body slot >= 0.5");
    s = in;
    s.aoYawDeg = 100.0f;
    s.prevAoYawDeg = -40.0f;
    Check(sf::NeutralAoTargetAlpha(s, cfg) == 0.0f, "neutral AO: a step of 140 deg in one frame");
    s = in;
    s.disableAo = 0.25f;
    Check(std::fabs(sf::NeutralAoTargetAlpha(s, cfg) - 0.75f) < 1e-6f, "neutral AO: alpha = 1 - Disable_AO");

    sf::NeutralAoBlend b;
    constexpr float dt = 1.0f / 60.0f;
    for (int i = 0; i < 6; ++i)
        b.Step(in, cfg, dt);
    Check(std::fabs(b.Alpha() - 0.5f) < 1e-4f, "neutral AO: fade in 0.2 s (0.5 after 0.1 s)");
    for (int i = 0; i < 30; ++i)
        b.Step(in, cfg, dt);
    Check(b.Alpha() == 1.0f, "neutral AO: full weight after the fade");
    sf::NeutralAoInput off = in;
    off.aiming = true;
    for (int i = 0; i < 12; ++i)
        b.Step(off, cfg, dt);
    Check(b.Alpha() == 0.0f, "neutral AO: fades out in 0.2 s");

    // Turning the option off (cfg.enabled false) fades the weight out like any other end of the enable rule, no cut.
    for (int i = 0; i < 30; ++i)
        b.Step(in, cfg, dt);
    sf::NeutralAoConfig disabled = cfg;
    disabled.enabled = false;
    b.Step(in, disabled, dt);
    const float firstOff = b.Alpha();
    Check(firstOff > 0.9f && firstOff < 1.0f, "neutral AO: option off fades (one frame later the weight is still above 0.9)");
    for (int i = 0; i < 12; ++i)
        b.Step(in, disabled, dt);
    Check(b.Alpha() == 0.0f, "neutral AO: option off reaches 0 within the fade time");

    // A full turn (the camera leaves the 110 deg sector, the turn sits at the 60 deg clamp) while the body turns at 340 deg/s (a running
    // turn) fades no faster than kNeutralAoTurnRateDegPerSec of turn: weight step x 60 deg <= 200 deg/s x dt (a 0.2 s fade unwound 5 deg
    // per frame at 60 fps on top of the body's own turn), done within 0.3 s plus a frame; fading in towards a camera at 100 deg the same;
    // a body that stands keeps the 0.2 s fade.
    sf::NeutralAoInput wide = in;
    wide.aoYawDeg = wide.prevAoYawDeg = 100.0f;
    wide.bodyYawRateDegPerSec = -340.0f;
    sf::NeutralAoBlend f;
    for (int i = 0; i < 60; ++i)
        f.Step(wide, cfg, dt);
    Check(f.Alpha() == 1.0f, "neutral AO: full weight at 100 deg after 1 s");
    sf::NeutralAoInput gone = wide;
    gone.aoYawDeg = gone.prevAoYawDeg = 120.0f;
    float prev = f.Alpha(), maxTurnStep = 0.0f;
    int frames = 0;
    for (int i = 0; i < 30 && f.Alpha() > 0.0f; ++i)
    {
        f.Step(gone, cfg, dt);
        maxTurnStep = std::max(maxTurnStep, std::fabs(f.Alpha() - prev) * 60.0f);
        prev = f.Alpha();
        ++frames;
    }
    Check(maxTurnStep <= 200.0f * dt + 1e-3f && f.Alpha() == 0.0f && frames <= 19,
          "neutral AO: a 60 deg turn fades out at <= 200 deg/s of turn (<= 3.33 deg per frame), within 0.3 s");
    std::printf("INFO neutral AO fade of a 60 deg turn: %.3f deg per frame, %d frames\n", maxTurnStep, frames);
    prev = 0.0f;
    maxTurnStep = 0.0f;
    for (int i = 0; i < 30; ++i)
    {
        f.Step(wide, cfg, dt);
        maxTurnStep = std::max(maxTurnStep, std::fabs(f.Alpha() - prev) * 60.0f);
        prev = f.Alpha();
    }
    Check(maxTurnStep <= 200.0f * dt + 1e-3f && f.Alpha() == 1.0f, "neutral AO: fading in towards a camera at 100 deg at <= 200 deg/s of turn");
    sf::NeutralAoInput still = gone;
    still.bodyYawRateDegPerSec = 0.0f;
    for (int i = 0; i < 12; ++i)
        f.Step(still, cfg, dt);
    Check(f.Alpha() == 0.0f, "neutral AO: a standing body fades a 60 deg turn out in 0.2 s");
}

// Rest framing dead band of the neutral aim offset pitch: no pitch from the rest framing pitch (-15 deg) up to level, the camera
// pitch above level, the excess below the rest pitch, continuous at both edges.
void NeutralAoPitchDeadBandTest()
{
    Check(sf::NeutralAoPitchDeadBand(-15.0f) == 0.0f, "neutral AO pitch: the rest framing pitch gives no pitch");
    Check(sf::NeutralAoPitchDeadBand(-7.0f) == 0.0f, "neutral AO pitch: inside the band no pitch");
    Check(sf::NeutralAoPitchDeadBand(0.0f) == 0.0f, "neutral AO pitch: level gives no pitch");
    Check(std::fabs(sf::NeutralAoPitchDeadBand(20.0f) - 20.0f) < 1e-6f, "neutral AO pitch: looking up keeps the camera pitch");
    Check(std::fabs(sf::NeutralAoPitchDeadBand(-40.0f) + 25.0f) < 1e-6f, "neutral AO pitch: looking down keeps the excess only");
    Check(std::fabs(sf::NeutralAoPitchDeadBand(-15.01f) + 0.01f) < 1e-4f && std::fabs(sf::NeutralAoPitchDeadBand(0.01f) - 0.01f) < 1e-6f,
          "neutral AO pitch: continuous at both edges");
    Check(sf::NeutralAoPitchDeadBand(std::nanf("")) == 0.0f, "neutral AO pitch: NaN gives no pitch");
}
} // namespace

int main()
{
    WorldTargetTest();
    WorldTrackedSpineTest();
    LeanGateTest();
    NeutralAoTest();
    NeutralAoPitchDeadBandTest();
    std::printf("%s (%d failures)\n", g_fail == 0 ? "PASS" : "FAIL ", g_fail);
    return g_fail == 0 ? 0 : 1;
}

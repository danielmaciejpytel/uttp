// Shooter mode + H2 safeguard, adapter level (A1): MapToChooser only (src/mm/MotionAdapter.hpp), hand-built MotionFeed / MapConfig / MapContext, no data.
//   a) the default configuration: the rotation mode and the spin predicate are the pre-Shooter ones for every aiming / hip-fire pose / air combination
//   b) MapConfig::shooterMode: Strafe on the ground and in the air; Orient only in the slide itself (PSM slide, the feed after it) and in traversal;
//      the spin transition only where the root lock runs (ground, moving: one pick, the lock resolves the offset), never in the air, in the slide,
//      its exit window and traversal (the suppression follows the setting); CanSprint cone for the sprint gait
//   c) MapConfig::h2Strafe + MapContext::bodyTurnSwitchedOff (native body turn off for another reason) + running BACKWARD: the latch, its holds,
//      its exclusions (aim, hip fire pose and RAW hip fire, slide, traversal, AimWalk), its sector (105 / 65 deg), its release
//   d) ShooterModeSpeedRatio: motion direction speeds (stand run 1 / 0.70 / 0.60, walk 1 / 0.90 / 0.75, crouch 1 / 0.889 / 0.80), 1.0 outside Stand / Crouch
// Mutation builds (-DUTPP_SHOOTER_MODE_MUTATION=k, src/mm/MotionAdapter.hpp) MUST print a line starting "FAIL ": the test is registered with
// PASS_REGULAR_EXPRESSION "FAIL " there. Usage: utpp_ShooterModeStrafeMap_test (no arguments).
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "../src/mm/MotionAdapter.hpp"

namespace
{
using namespace utpp::mm;
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
}
constexpr float kDt = 1.0f / 60.0f;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

struct In
{
    float inputDeg = kNaN; // world input yaw against the actor (actor = camera = 0): 0 forward, 180 back; NaN = stick released
    float stick = 1.0f;
    float speed = 0.0f;    // game m/s along the actor forward
    int32_t psm = adapter::kPsmStand;
    bool aiming = false, air = false, switchedOff = false, hipAny = false, hipLease = false, visualStrafe = false;
    float rootYawDeg = 0.0f;
};
struct Out
{
    bool strafe = false, spin = false, h2 = false, fn = false;
    int gait = -1;
};
struct Sim
{
    adapter::MapConfig cfg;
    adapter::MapState ms;
    adapter::MapContext ctx;
    Out Frame(const In& in)
    {
        adapter::MotionFeed f;
        f.dt = kDt;
        f.velWorld[1] = in.speed;
        if (std::isfinite(in.inputDeg))
        {
            const float r = in.inputDeg * 3.14159265358979f / 180.0f;
            f.move[0] = -std::sin(r) * in.stick;
            f.move[1] = std::cos(r) * in.stick;
        }
        f.psmDetailed = in.psm;
        f.aiming = in.aiming;
        if (in.air)
        {
            f.air.valid = true;
            f.air.onGround = false;
            f.air.distanceToGround = 2.0f;
            f.air.psmFall = 1;
        }
        else
        {
            f.air.valid = true;
            f.air.onGround = true;
        }
        adapter::MapContext x = ctx;
        x.rootYawValid = true;
        x.rootYawDeg = in.rootYawDeg;
        x.bodyTurnSwitchedOff = in.switchedOff;
        x.hipFireAny = in.hipAny;
        x.hipLease = in.hipLease;
        cfg.visualStrafe = in.visualStrafe;
        const motion::ChooserState cs = adapter::MapToChooser(f, cfg, ms, x);
        Out o;
        o.strafe = ms.st.rotationMode == motiontraj::RotationMode::Strafe;
        o.spin = cs.shouldSpinTransition;
        o.h2 = ms.h2Active;
        o.fn = ms.shooterModeActive;
        o.gait = cs.gait;
        return o;
    }
    Out Run(int n, const In& in)
    {
        Out o;
        for (int i = 0; i < n; ++i)
            o = Frame(in);
        return o;
    }
};
In Running(float inputDeg = 0.0f, float speed = 3.85f)
{
    In in;
    in.inputDeg = inputDeg;
    in.speed = speed;
    return in;
}
std::string F(float v) { char b[32]; std::snprintf(b, sizeof b, "%.3f", static_cast<double>(v)); return b; }
std::string B(bool v) { return v ? "1" : "0"; }
} // namespace

int main()
{
    // ---------------------------------------------------------------- a) default configuration: the pre-Shooter behaviour
    {
        bool allRot = true, allSpin = true;
        std::string bad;
        for (int aiming = 0; aiming < 2; ++aiming)
            for (int vis = 0; vis < 2; ++vis)
                for (int air = 0; air < 2; ++air)
                {
                    Sim s; // MapConfig defaults: shooterMode false, h2Strafe false
                    In in = Running(0.0f);
                    in.aiming = aiming != 0;
                    in.visualStrafe = vis != 0;
                    in.air = air != 0;
                    in.psm = air ? adapter::kPsmFall : adapter::kPsmStand;
                    in.rootYawDeg = 180.0f;
                    in.switchedOff = true; // the H2 inputs alone change nothing while the config is default
                    in.hipAny = true;
                    const Out o = s.Run(3, in);
                    const bool expectStrafe = aiming != 0 || (vis != 0 && air == 0); // strafeWhileAiming && aiming || visualStrafe on the ground
                    allRot = allRot && o.strafe == expectStrafe && !o.h2 && !o.fn;
                    allSpin = allSpin && o.spin; // root 180 deg off at 385 cm/s: the spin predicate is untouched by the rotation mode
                    if (o.strafe != expectStrafe || !o.spin)
                        bad += " [aim" + std::to_string(aiming) + " vis" + std::to_string(vis) + " air" + std::to_string(air) + "]";
                }
        Check(allRot, "A1a default config: rot = strafe iff (aiming) or (hip-fire pose on the ground), no h2 / shooterMode", bad);
        Check(allSpin, "A1a default config: ShouldSpinTransition untouched (root 180 deg off, 3.85 m/s) in every row", bad);
    }

    // ---------------------------------------------------------------- b) Shooter mode
    {
        Sim s;
        s.cfg.shooterMode = true;
        In run = Running(0.0f);
        run.rootYawDeg = 180.0f;
        Out o = s.Run(3, run);
        Check(o.strafe && o.fn, "A1b shooterMode: Strafe on the ground", "strafe " + B(o.strafe) + " fn " + B(o.fn));
        Check(o.spin, "A1b shooterMode: the spin predicate runs on the ground while moving (root lock engaged: one pick, then the lock resolves it)");
        In air = run;
        air.air = true;
        air.psm = adapter::kPsmFall;
        o = s.Run(3, air);
        Check(o.strafe && o.fn, "A1b shooterMode: Strafe in the air", "strafe " + B(o.strafe));
        Check(!o.spin, "A1b shooterMode: no spin transition in the air");

        // slide itself (PSM 5) and the feed after it: Orient. Then Strafe at once (the exit window is not an Orient window any more)
        Sim sl;
        sl.cfg.shooterMode = true;
        sl.cfg.slide = true;
        sl.cfg.slideSpec.declared = true;
        In slide = Running(0.0f, 6.0f);
        slide.rootYawDeg = 180.0f;
        sl.Run(10, run); // running first
        slide.psm = adapter::kPsmSlide;
        o = sl.Run(4, slide);
        Check(!o.strafe && !o.fn, "A1b shooterMode: Orient while the PSM reports the slide", "strafe " + B(o.strafe));
        Check(!o.spin, "A1b shooterMode: no spin transition during the slide (the suppression follows the setting)");
        In exitFrame = slide;
        exitFrame.psm = adapter::kPsmCrouch;
        o = sl.Frame(exitFrame);
        Check(!o.strafe, "A1b shooterMode: the first feed after the slide is still Orient (exit rows fire once)");
        Check(!o.spin, "A1b shooterMode: no spin transition on the first feed after the slide");
        bool strafeFromNext = true, noSpinInExit = true;
        for (int i = 0; i < 30; ++i) // 0.5 s: inside the 0.6 s exit window
        {
            o = sl.Frame(exitFrame);
            strafeFromNext = strafeFromNext && o.strafe;
            noSpinInExit = noSpinInExit && !o.spin;
        }
        Check(strafeFromNext, "A1b shooterMode: Strafe again from the second feed after the slide (no 0.6 s Orient window)");
        Check(noSpinInExit, "A1b shooterMode: no spin transition in the slide-exit window with the root 180 deg off");
        o = sl.Run(12, exitFrame); // past the 0.6 s window
        Check(o.spin && o.strafe, "A1b shooterMode: after the exit window the spin predicate runs again where the lock does");
        // the same slide with Shooter OFF does spin (the suppression is not a side effect of the slide handling)
        Sim off;
        off.cfg.slide = true;
        off.cfg.slideSpec.declared = true;
        off.Run(10, run);
        o = off.Run(3, slide);
        Check(o.spin, "A1b shooterMode OFF: the same slide frames keep the spin predicate (discriminating reference)");

        // traversal (Climb): Orient, no spin
        Sim tr;
        tr.cfg.shooterMode = true;
        In climb = run;
        climb.psm = adapter::kPsmClimb;
        o = tr.Run(3, climb);
        Check(!o.strafe && !o.fn, "A1b shooterMode: Orient during traversal (Climb)");
        Check(!o.spin, "A1b shooterMode: no spin transition during traversal");

        // sprint gait: CanSprint cone (motion: < 50 deg from the actor), PSM Sprint with the stick at 0 / 40 / 60 / 90 / 180 deg
        const float angles[5] = {0, 40, 60, 90, 180};
        const int expectShooterMode[5] = {2, 2, 1, 1, 1};
        std::string got, gotOff;
        bool okFn = true, okOff = true;
        for (int i = 0; i < 5; ++i)
        {
            Sim a;
            a.cfg.shooterMode = true;
            In sp = Running(angles[i], 8.2f);
            sp.psm = adapter::kPsmSprint;
            const Out oa = a.Run(3, sp);
            got += std::to_string(oa.gait) + " ";
            okFn = okFn && oa.gait == expectShooterMode[i];
            Sim b;
            const Out ob = b.Run(3, sp);
            gotOff += std::to_string(ob.gait) + " ";
            okOff = okOff && ob.gait == 2;
        }
        Check(okFn, "A1b shooterMode: sprint gait for stick 0/40/60/90/180 deg = 2/2/1/1/1", got);
        Check(okOff, "A1b shooterMode OFF: sprint gait is 2 at every angle (Orient: the vanilla behaviour)", gotOff);
    }

    // ---------------------------------------------------------------- c) H2 safeguard (Shooter OFF)
    {
        auto h2Sim = [] {
            Sim s;
            s.cfg.h2Strafe = true;
            return s;
        };
        // 0.3 s forward input with the switch off: input 0 deg against the actor, never latched
        {
            Sim s = h2Sim();
            In in = Running(0.0f);
            in.switchedOff = true;
            bool any = false;
            for (int i = 0; i < 40; ++i)
                any = any || s.Frame(in).h2;
            Check(!any, "A1c h2: forward running with the switch off never latches");
        }
        // flick to 180 after the holds: latched on the first flick frame; spin suppressed (root 180 off); not latched without the safeguard
        {
            Sim s = h2Sim();
            In fwd = Running(0.0f);
            fwd.switchedOff = true;
            s.Run(30, fwd);
            In back = Running(180.0f);
            back.switchedOff = true;
            back.rootYawDeg = 180.0f;
            const Out o = s.Frame(back);
            Check(o.h2 && o.strafe, "A1c h2: a flick to 180 deg latches on the first flick frame (Strafe)", "h2 " + B(o.h2) + " strafe " + B(o.strafe));
            Check(!o.spin, "A1c h2: no spin transition while latched");
            Check(s.Run(60, back).h2, "A1c h2: stays latched while running backward");
            Sim noGuard;
            noGuard.Run(30, fwd);
            const Out n = noGuard.Frame(back);
            Check(!n.h2 && n.spin && !n.strafe, "A1c h2 off by config: Orient and the spin predicate as before (discriminating reference)");
        }
        // < 0.2 s after the switch went off: not latched even with input at 180
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            bool early = false;
            for (int i = 1; i <= 10; ++i)
                early = early || s.Frame(back).h2;
            Check(!early, "A1c h2: frames 1-10 after the switch went off (< 0.2 s) are not latched");
            const Out late = s.Run(6, back);
            Check(late.h2, "A1c h2: latched once the switch has been off for 0.2 s", "frame 16");
        }
        // aiming: unlatched; 0.10 s after the aim end not latched; 0.16 s after latched
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            s.Run(30, back);
            In aim = back;
            aim.aiming = true;
            const Out a = s.Frame(aim);
            Check(!a.h2, "A1c h2: an aiming frame unlatches");
            const Out e6 = s.Run(6, back);
            Check(!e6.h2, "A1c h2: 0.10 s after the aim end: not latched");
            const Out e10 = s.Run(4, back);
            Check(e10.h2, "A1c h2: 0.16 s after the aim end: latched");
        }
        // raw hip fire (weapon action 8 / pose hold) excludes even when the hip-fire POSE gate is closed (visualStrafe false)
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            back.hipAny = true;
            bool any = false;
            for (int i = 0; i < 90; ++i)
                any = any || s.Frame(back).h2;
            Check(!any, "A1c h2: the raw hip-fire state excludes the latch although visualStrafe is false");
            In rel = back;
            rel.hipAny = false;
            s.Run(6, rel);
            Check(s.Run(6, rel).h2, "A1c h2: latches 0.15 s after the raw hip-fire state ended");
            Sim v = h2Sim();
            In vis = back;
            vis.hipAny = false;
            vis.visualStrafe = true;
            any = false;
            for (int i = 0; i < 90; ++i)
                any = any || v.Frame(vis).h2;
            Check(!any, "A1c h2: the hip-fire pose (visualStrafe) excludes the latch");
        }
        // the CET hip-fire lease (UTPPSetMotionHipLease) is an exclusion although neither the raw hip-fire state nor the hip-fire pose is on (a weapon
        // whose resolved aim pose is Default: grenade / projectile launcher, unlisted types): running backward under the lease never latches
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            back.hipLease = true; // hipAny false, visualStrafe false: only the lease is on
            bool any = false;
            for (int i = 0; i < 90; ++i)
                any = any || s.Frame(back).h2;
            Check(!any, "A1c h2: the CET hip-fire lease excludes the latch (Default-pose weapon: no raw hip fire, no pose)");
            In rel = back;
            rel.hipLease = false;
            const bool at6 = s.Run(6, rel).h2;
            const bool at10 = s.Run(4, rel).h2;
            Check(!at6 && at10, "A1c h2: 0.10 s after the lease ended: not latched, 0.16 s: latched", "0.10s:" + B(at6) + " 0.16s:" + B(at10));
        }
        // sector: only the backward sector latches (105 in, 65 out)
        {
            std::string got;
            bool ok = true;
            const float angles[6] = {10, 60, 90, 100, 104, 106};
            for (int i = 0; i < 6; ++i)
            {
                Sim s = h2Sim();
                In fwd = Running(0.0f);
                fwd.switchedOff = true;
                s.Run(30, fwd);
                In in = Running(angles[i]);
                in.switchedOff = true;
                const bool l = s.Run(3, in).h2;
                got += std::to_string(static_cast<int>(angles[i])) + ":" + B(l) + " ";
                ok = ok && (l == (angles[i] >= 105.0f));
            }
            Check(ok, "A1c h2: input at 10/60/90/100/104 deg does not latch, 106 deg does (backward sector only)", got);
            Sim s = h2Sim();
            In fwd = Running(0.0f);
            fwd.switchedOff = true;
            s.Run(30, fwd);
            In back = Running(180.0f);
            back.switchedOff = true;
            s.Run(3, back);
            In diag = Running(70.0f);
            diag.switchedOff = true;
            const bool hold70 = s.Run(3, diag).h2;
            In out = Running(60.0f);
            out.switchedOff = true;
            const bool at60 = s.Run(3, out).h2;
            Check(hold70 && !at60, "A1c h2: hysteresis - still latched at 70 deg, released below 65 deg", "70:" + B(hold70) + " 60:" + B(at60));
        }
        // standing start: entry needs a run (a spin needs >= 1.5 m/s)
        {
            Sim s = h2Sim();
            In back = Running(180.0f, 0.0f);
            back.switchedOff = true;
            s.Run(30, back);
            const bool standing = s.Frame(back).h2;
            back.speed = 3.0f;
            const bool running = s.Frame(back).h2;
            Check(!standing && running, "A1c h2: no entry while standing, entry at speed", "standing " + B(standing) + " running " + B(running));
        }
        // release: no input 0.2 s -> still latched, 0.3 s -> unlatched
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            s.Run(30, back);
            In none = back;
            none.inputDeg = kNaN;
            const bool at200 = s.Run(12, none).h2;
            const bool at300 = s.Run(6, none).h2;
            Check(at200 && !at300, "A1c h2: released stick - latched at 0.2 s, unlatched at 0.3 s", "0.2s:" + B(at200) + " 0.3s:" + B(at300));
        }
        // air keeps the latch, a slide frame / traversal / AimWalk drops it
        {
            Sim s = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = true;
            s.Run(30, back);
            In air = back;
            air.air = true;
            air.psm = adapter::kPsmFall;
            const bool inAir = s.Run(10, air).h2;
            const bool landed = s.Run(2, back).h2;
            In slide = back;
            slide.psm = adapter::kPsmSlide;
            const bool sl = s.Frame(slide).h2;
            Check(inAir && landed && !sl, "A1c h2: air frames keep the latch, a slide frame drops it", "air " + B(inAir) + " ground " + B(landed) + " slide " + B(sl));
            Sim t = h2Sim();
            t.Run(30, back);
            In climb = back;
            climb.psm = adapter::kPsmClimb;
            Check(!t.Frame(climb).h2, "A1c h2: traversal drops the latch");
            Sim w = h2Sim();
            In walk = back;
            walk.psm = adapter::kPsmAimWalk;
            bool any = false;
            for (int i = 0; i < 60; ++i)
                any = any || w.Frame(walk).h2;
            Check(!any, "A1c h2: AimWalk never latches");
        }
        // never latched without the switch or with the safeguard off; never with Shooter on
        {
            Sim a = h2Sim();
            In back = Running(180.0f);
            back.switchedOff = false;
            bool any = false;
            for (int i = 0; i < 120; ++i)
                any = any || a.Frame(back).h2;
            Check(!any, "A1c h2: bodyTurnSwitchedOff false (the body turn drives) never latches");
            Sim b;
            In off = Running(180.0f);
            off.switchedOff = true;
            any = false;
            for (int i = 0; i < 120; ++i)
                any = any || b.Frame(off).h2;
            Check(!any, "A1c h2: h2Strafe false never latches");
            Sim c = h2Sim();
            c.cfg.shooterMode = true;
            any = false;
            bool fn = true;
            for (int i = 0; i < 120; ++i)
            {
                const Out o = c.Frame(off);
                any = any || o.h2;
                fn = fn && o.fn;
            }
            Check(!any && fn, "A1c h2: Shooter on takes precedence (no h2 latch, shooterMode active)");
        }
    }

    // ---------------------------------------------------------------- d) speed ratio by direction
    {
        auto nearTo = [](float a, float b) { return std::fabs(a - b) < 0.005f; };
        struct Row { float x, y; int32_t d; float expect; const char* what; };
        // body-relative stick: x right, y forward; side stick x = +-1
        const Row rows[] = {
            {0.0f, 1.0f, adapter::kPsmStand, 1.0f, "stand forward"},   {1.0f, 0.0f, adapter::kPsmStand, 0.70f, "stand right"},
            {-1.0f, 0.0f, adapter::kPsmStand, 0.70f, "stand left"},    {0.0f, -1.0f, adapter::kPsmStand, 0.60f, "stand back"},
            {0.5f, 0.0f, adapter::kPsmStand, 0.90f, "walk right (half stick)"}, {0.0f, -0.5f, adapter::kPsmStand, 0.75f, "walk back"},
            {0.0f, 0.5f, adapter::kPsmStand, 1.0f, "walk forward"},    {0.0f, 1.0f, adapter::kPsmCrouch, 1.0f, "crouch forward"},
            {1.0f, 0.0f, adapter::kPsmCrouch, 0.889f, "crouch right"}, {0.0f, -1.0f, adapter::kPsmCrouch, 0.80f, "crouch back"},
            {0.0f, -1.0f, adapter::kPsmSprint, 1.0f, "sprint detail: untouched"}, {1.0f, 0.0f, adapter::kPsmAimWalk, 1.0f, "aim walk: untouched"},
            {0.0f, -1.0f, adapter::kPsmSlide, 1.0f, "slide: untouched"}, {0.0f, -0.1f, adapter::kPsmStand, 1.0f, "stick below 0.15"},
            {kNaN, -1.0f, adapter::kPsmStand, 1.0f, "non-finite"},
        };
        bool all = true;
        std::string bad;
        for (const Row& r : rows)
        {
            const float v = adapter::ShooterModeSpeedRatio(r.x, r.y, r.d);
            if (!nearTo(v, r.expect))
            {
                all = false;
                bad += std::string(" [") + r.what + " " + F(v) + " != " + F(r.expect) + "]";
            }
        }
        Check(all, "A1d ShooterModeSpeedRatio: stand 1/0.70/0.60, walk 1/0.90/0.75, crouch 1/0.889/0.80, 1.0 elsewhere", bad);
        Check(nearTo(adapter::ShooterModeSpeedRatio(0.0f, -1.0f, adapter::kPsmStand), 0.6f), "A1d ShooterModeSpeedRatio: backward run is 300 / 500");
        // the gait the chooser plays decides (the walk toggle walks at full stick: motion walk strafe 180/200 side, 150/200 back)
        const float walkSide = adapter::ShooterModeSpeedRatio(1.0f, 0.0f, adapter::kPsmStand, 0), walkBack = adapter::ShooterModeSpeedRatio(0.0f, -1.0f, adapter::kPsmStand, 0);
        const float runHalf = adapter::ShooterModeSpeedRatio(0.5f, 0.0f, adapter::kPsmStand, 1);
        Check(nearTo(walkSide, 0.90f) && nearTo(walkBack, 0.75f) && nearTo(runHalf, 0.70f),
              "A1d ShooterModeSpeedRatio: an explicit chooser gait overrides the stick size (walk at full stick 0.90 / 0.75, run at half stick 0.70)",
              "walk side " + F(walkSide) + " back " + F(walkBack) + ", run half stick side " + F(runHalf));
        Check(nearTo(adapter::ShooterModeSpeedRatio(1.0f, 0.0f, adapter::kPsmStand, 2), 0.70f), "A1d ShooterModeSpeedRatio: the sprint gait uses the run ratios");
    }

    std::printf("shooterMode strafe map: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

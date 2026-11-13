// P5 FIX C (recording 2026-10-21 12:15): the stationary shot-turn goal is dropped once the camera has left the shot.
//
// Recorded (rifle hipfire, video 372.8-374.2 s): the goal froze on the camera heading of the last Shoot sample (+17 deg) for
// up to 3 s; the camera then swung to -124 deg within 0.3 s, the capsule followed it, and the footwork turned V back towards the
// old shot: she stood 117 deg off the camera for 0.8 s. The hold must end when the camera is more than kAbandonDegrees from the
// goal while the hipfire pose hold (visualStrafe, 0.75 s after the last Shoot) takes the facing over; after it expired the hold
// finishes the started turn as before. Built with -DUTPP_SHOTTURN_MUTATION=1 (never abandon), =2 (abandon without a successor
// owner) or =3 (R3, recording 2026-10-23 21:08: any error above 5 deg starts a turn and a passed goal is kept) this test MUST report
// FAIL lines.
#include "../src/mm/StationaryShotTurn.hpp"
#include <cmath>
#include <cstdio>
#include <string>

using namespace utpp::mm;

namespace
{
struct ShotFrame
{
    float dt;
    int action;
    float cam, vis;
};
#include "data/recording_1215_shot_turn.inc"

int passed = 0, failed = 0;
void Check(const char *name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passed : ++failed;
}
float Diff(float a, float b)
{
    return std::fabs(std::remainder(a - b, 360.f));
}
std::string Name(const char *what, int hz)
{
    return std::string(what) + " " + std::to_string(hz) + "Hz";
}
} // namespace

int main()
{
    // ---- the recording: Hold fed with the recorded Shoot / camera / visible yaw (stationary hipfire, eligible)
    {
        shotturn::Hold hold;
        float t = 0, goal = 0, lastShot = -1, awayAt = -1, endAt = -1, activeBeforeAway = 0;
        bool wasActive = false;
        for (const ShotFrame &f : kShotFrames)
        {
            t += f.dt;
            if (f.action == 8)
                lastShot = t;
            const bool visualStrafe = f.action == 8 || (lastShot >= 0 && t - lastShot < 0.75f); // HipFirePoseHold
            hold.Step(f.action, f.dt, true, f.cam, f.vis, visualStrafe);
            if (f.action == 8)
                goal = hold.targetYawDeg;
            else if (lastShot >= 0 && awayAt < 0 && Diff(f.cam, goal) > shotturn::kAbandonDegrees)
                awayAt = t;
            if (lastShot >= 0 && awayAt < 0 && hold.Active() && f.action != 8)
                activeBeforeAway = t - lastShot;
            if (wasActive && !hold.Active() && lastShot >= 0 && f.action != 8 && endAt < 0)
                endAt = t;
            wasActive = hold.Active();
        }
        std::printf("recording: goal %.1f deg latched at the last Shoot (%.3f s), camera > %.0f deg away at %.3f s, hold ended at %.3f s "
                    "(lag %.3f s), hold active %.2f s after the shot before the camera left\n",
                    goal, lastShot, shotturn::kAbandonDegrees, awayAt, endAt, endAt - awayAt, activeBeforeAway);
        Check("recording: the last Shoot latches the camera heading (+17 deg)", std::fabs(goal - 17.f) < 2.f);
        Check("recording: the camera leaves the goal by more than 45 deg during the window", awayAt > lastShot);
        Check("recording: the hold is kept while the camera stays near the goal (>= 0.15 s after the shot)", activeBeforeAway >= 0.15f);
        Check("recording: the hold ends within 0.1 s after the camera left the goal (was: kept for the rest of the 3 s)",
              endAt > 0 && endAt - awayAt <= 0.1f);
    }

    for (int hz : {60, 144})
    {
        const float dt = 1.f / float(hz);
        const int n3 = int(std::lround(3.f * hz));
        // ---- camera stays on the shot: the hold keeps its goal for 3 s (existing behaviour)
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 170.f, 0.f);
            bool keeps = h.Active() && Diff(h.targetYawDeg, 170.f) < 1e-3f;
            int i = 1;
            for (; i < n3 - 2; ++i)
            {
                h.Step(0, dt, true, 170.f, 0.f);
                keeps = keeps && h.Active();
            }
            const bool activeAt29 = h.Active();
            for (; i < n3 + 3; ++i)
                h.Step(0, dt, true, 170.f, 0.f);
            Check(Name("camera on the shot: the hold lasts 3 s and keeps the goal", hz).c_str(), keeps && activeAt29 && !h.Active());
        }
        // ---- camera drifts less than the threshold: the hold is kept
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 0.f, 120.f);
            bool keeps = true;
            for (int i = 1; i < hz; ++i)
            {
                h.Step(0, dt, true, 44.f * float(i) / float(hz), 120.f);
                keeps = keeps && h.Active();
            }
            Check(Name("camera drift of 44 deg keeps the hold", hz).c_str(), keeps && Diff(h.targetYawDeg, 0.f) < 1e-3f);
        }
        // ---- boundary: the first frame beyond 45 deg ends the hold, in that frame
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 0.f, 120.f);
            h.Step(0, dt, true, 44.9f, 120.f);
            const bool at449 = h.Active();
            h.Step(0, dt, true, 45.1f, 120.f);
            Check(Name("goal abandoned in the frame the camera is > 45 deg away (44.9 kept, 45.1 dropped)", hz).c_str(),
                  at449 && !h.Active() && h.targetYawDeg == 0.f);
        }
        // ---- a flick right after the shot (the recorded case): dropped in the next frame, and it cannot come back
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 17.f, -80.f, true);
            h.Step(0, dt, true, 160.f, -80.f, true);
            bool dropped = !h.Active();
            bool revived = false;
            for (int i = 0; i < n3; ++i)
            {
                h.Step(0, dt, true, 17.f, -80.f); // the camera returns to the old shot direction
                revived = revived || h.Active();
            }
            Check(Name("206 deg camera flick after a shot: hold dropped at once and not revived by the camera returning", hz).c_str(),
                  dropped && !revived);
        }
        // ---- the camera leaves the shot AFTER the hipfire pose hold expired (0.75 s): nothing else owns the facing, the hold
        //      keeps finishing the started turn (it used to be cut half way and V stood neither on the shot nor on the camera)
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 17.f, -100.f, true);
            float t = 0;
            bool keeps = true;
            for (int i = 1; i < int(std::lround(0.8f * hz)); ++i) // camera stays on the shot for 0.8 s
            {
                t += dt;
                h.Step(0, dt, true, 17.f, -100.f, t < 0.75f);
                keeps = keeps && h.Active();
            }
            h.Step(0, dt, true, 190.f, -100.f, false); // 173 deg flick, pose hold expired
            const bool afterFlick = h.Active() && Diff(h.targetYawDeg, 17.f) < 1e-3f;
            bool still = true;
            for (int i = 0; i < hz; ++i)
            {
                h.Step(0, dt, true, 190.f, -100.f, false);
                still = still && h.Active();
            }
            Check(Name("camera flick after the pose hold expired: the hold keeps finishing the started turn", hz).c_str(),
                  keeps && afterFlick && still);
        }
        // ---- intermittent fire: a gap with the camera far off the old goal drops the hold, the next Shoot latches the new heading
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 17.f, -100.f, true);
            h.Step(0, dt, true, 120.f, -100.f, true);
            const bool dropped = !h.Active();
            h.Step(0, dt, true, 125.f, -100.f, true);
            const bool stays = !h.Active();
            h.Step(8, dt, true, 125.f, -100.f, true); // the next shot, 2 frames later
            Check(Name("intermittent fire: dropped in the gap, the next shot latches the new camera heading", hz).c_str(),
                  dropped && stays && h.Active() && Diff(h.targetYawDeg, 125.f) < 1e-3f);
        }
        // ---- continuous fire follows the camera: the goal is re-latched every Shoot sample, never abandoned
        {
            shotturn::Hold h;
            bool always = true, tracks = true;
            const int burst = hz; // 1 s; V faces -100 deg, the sweep 20 -> 200 deg never meets it (no finish)
            for (int i = 0; i < burst; ++i)
            {
                const float cam = 20.f + 180.f * float(i) / float(burst - 1);
                h.Step(8, dt, true, cam, -100.f);
                always = always && h.Active();
                tracks = tracks && Diff(h.targetYawDeg, cam) < 1e-3f;
            }
            const float last = h.targetYawDeg;
            h.Step(0, dt, true, last, -100.f);
            Check(Name("burst with a 200 deg camera sweep: the goal tracks the camera and the hold never drops", hz).c_str(),
                  always && tracks && h.Active());
        }
        // ---- R3 (recording 2026-10-23 21:08): an error the aim offset covers starts no footwork; from 50 deg it does
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 30.f, 0.f);
            const bool small = !h.Active();
            h.Step(8, dt, true, 49.f, 0.f);
            const bool below = !h.Active();
            h.Step(8, dt, true, 51.f, 0.f);
            const bool admitted = h.Active() && Diff(h.targetYawDeg, 51.f) < 1e-3f;
            h.Step(8, dt, true, 30.f, 0.f); // a running turn keeps re-latching the camera on the same side
            const bool relatched = h.Active() && Diff(h.targetYawDeg, 30.f) < 1e-3f;
            Check(Name("admission: 30 / 49 deg start no turn, 51 deg does, a running turn follows the camera to 30 deg", hz).c_str(),
                  small && below && admitted && relatched);
        }
        // ---- R3: a turn that jumps over the goal is finished, it does not ask for a turn back (21:12:52.3 Turn_180_L ->
        //      21:12:53.0 Turn_090_R)
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 140.f, 0.f);
            const bool armed = h.Active();
            h.Step(0, dt, true, 140.f, 120.f);
            const bool turning = h.Active();
            h.Step(0, dt, true, 140.f, 148.f); // one frame of a fast 180 clip: 8 deg past the goal
            Check(Name("overshoot: 8 deg past the goal ends the hold (was: kept, the chooser turned back)", hz).c_str(),
                  armed && turning && !h.Active());
        }
        // ---- R3: the camera crosses to the other side of the body during a burst: a small error there is the aim offset's
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 100.f, 0.f);
            h.Step(8, dt, true, -20.f, 0.f);
            const bool smallOther = !h.Active();
            h.Step(8, dt, true, 100.f, 0.f);
            h.Step(8, dt, true, -90.f, 0.f);
            const bool bigOther = h.Active() && h.side == -1 && Diff(h.targetYawDeg, -90.f) < 1e-3f;
            Check(Name("side change in a burst: -20 deg ends the turn, -90 deg re-admits it to the right", hz).c_str(),
                  smallOther && bigOther);
        }
        // ---- review L1: continuous fire with the camera hanging near 180 deg behind V: the noise in the sign of the error does not
        //      flip the side or end the turn; a real pass of the goal still ends it
        {
            shotturn::Hold h;
            bool stable = true;
            for (int i = 0; i < hz; ++i)
            {
                const float cam = (i % 2) ? -179.f : 179.f; // +-1 deg of camera noise around the back of V (visual 0)
                h.Step(8, dt, true, cam, 0.f);
                stable = stable && h.Active() && h.side == 1; // admitted left on the first frame (+179)
            }
            h.Step(0, dt, true, -179.f, 0.f); // gap frame on the other side of 180
            const bool keptInGap = h.Active() && h.side == 1;
            h.Step(0, dt, true, -179.f, -173.f); // the left turn (visual 187 = -173) passed the goal (-179 = 181) by 6 deg
            Check(Name("camera noise at +-179 deg: the side holds for a 1 s burst, a real pass still finishes", hz).c_str(),
                  stable && keptInGap && !h.Active());
        }
        // ---- unchanged: reaching the goal finishes the hold; reload and ineligible frames reset it
        {
            shotturn::Hold h;
            h.Step(8, dt, true, 90.f, 88.f);
            const bool finished = !h.Active();
            h.Step(8, dt, true, 90.f, 0.f);
            const bool armed = h.Active();
            h.Step(2, dt, true, 90.f, 0.f);
            const bool reloadReset = !h.Active();
            h.Step(8, dt, true, 90.f, 0.f);
            h.Step(0, dt, false, 90.f, 0.f);
            Check(Name("finish within 5 deg, reload and ineligible frames still reset the hold", hz).c_str(),
                  finished && armed && reloadReset && !h.Active());
        }
    }
    std::printf("stationary shot turn: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

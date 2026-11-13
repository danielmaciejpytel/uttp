// Sustained hipfire pose (recording 2026-10-23 21:08): the raised firing pose survives the short gaps of intermittent fire.
//
// Recorded (Nue, standing hipfire 21:10:18-21:10:24, utpp-2026-10-23-20-47-10.log): semi-automatic shots 1.1-1.8 s apart. The
// 0.75 s HipFirePoseHold expired before every next shot: aimW 1.0 -> 0.16 and back (ov=3/2 <-> 3/1), the arms flipped between the
// raised and the Ready pose about every 0.5-1.1 s, and the strafe facing ended with the pose. The replay below feeds the recorded
// weapon action edges (tests/data/recording_2108_hipfire_actions.inc) at 60 and 144 Hz.
// Built with -DUTPP_HIPFIRE_SUSTAIN_MUTATION=1 (every shot isolated, the pre-fix hold), =2 (a sprint request keeps the 2 s hold) or
// =3 (the vehicle exit window counts as a shot), =4 (the holds survive the air / landing) or =5 (the jump-out window holds the pose
// through the landing) or =6 (the release is also evaluated on the launch feed) this test MUST report FAIL lines.
#include "../src/mm/HipFirePose.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace utpp::mm::ov;

namespace
{
struct ActionEdge
{
    float t;
    int action;
};
#include "data/recording_2108_hipfire_actions.inc"

int passed = 0, failed = 0;
void Check(const std::string &name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name.c_str());
    ok ? ++passed : ++failed;
}

struct Replay
{
    int flips = 0;              // the pose dropped while no reload ran and a Shoot started within kSustainLinkS after the drop
    int activeInReload = 0;     // frames with the pose active during a reload (must be 0)
    bool raisedThrough = true;  // the pose stayed active over [from, to]
};

template <size_t N>
Replay Run(const ActionEdge (&edges)[N], int hz, float from = 0, float to = -1)
{
    Replay r;
    HipFirePoseHold hold;
    const float dt = 1.f / float(hz);
    const float t0 = edges[0].t, t1 = edges[N - 1].t + 3.f;
    size_t k = 0;
    int action = 0;
    std::vector<float> drops, shotStarts;
    bool was = false;
    for (float t = t0; t <= t1; t += dt)
    {
        int prevAction = action;
        while (k < N && edges[k].t <= t)
            action = edges[k++].action;
        if (action == 8 && prevAction != 8)
            shotStarts.push_back(t);
        hold.Step(action, dt, true);
        const bool now = hold.Active();
        if (was && !now && action == 0)
            drops.push_back(t);
        if (now && action == 2)
            ++r.activeInReload;
        if (to > from && t >= from && t <= to && action != 2 && !now)
            r.raisedThrough = false;
        was = now;
    }
    for (float d : drops)
        for (float s : shotStarts)
            if (s > d && s - d < kSustainLinkS)
            {
                ++r.flips;
                break;
            }
    return r;
}
} // namespace

int main()
{
    for (int hz : {60, 144})
    {
        const std::string at = " " + std::to_string(hz) + "Hz";
        const float dt = 1.f / float(hz);
        // ---- the recording
        {
            // 21:10:19.871 is the second shot of the engagement (1.25 s after the first): from it to the reload at 21:10:26.771 the
            // shots are 0.03-1.77 s apart, the pose must not drop once (pre-fix: dropped at 20.99 and 23.10).
            const Replay nue = Run(kNueEdges, hz, 19.9f, 26.7f);
            std::printf("nue replay%s: flips %d, active in reload %d, raised 19.9-26.7 %d\n", at.c_str(), nue.flips,
                        nue.activeInReload, nue.raisedThrough ? 1 : 0);
            Check("recording Nue: the pose stays raised between the shots 21:10:19.9-26.7" + at, nue.raisedThrough);
            // pre-fix replay: 4 drops followed by a shot within 2.5 s; now 1 (the first shot of an engagement keeps its 0.75 s hold)
            Check("recording Nue: at most 2 raise/lower flips in 21:10:15-21:11:14 (pre-fix 4)" + at, nue.flips <= 2);
            Check("recording Nue: a reload always owns the arms (pose off in every reload frame)" + at, nue.activeInReload == 0);
            const Replay rifle = Run(kRifleEdges, hz);
            std::printf("rifle replay%s: flips %d, active in reload %d\n", at.c_str(), rifle.flips, rifle.activeInReload);
            Check("recording rifle: a reload always owns the arms" + at, rifle.activeInReload == 0);
            Check("recording rifle: no raise/lower flip between bursts" + at, rifle.flips == 0);
        }
        // ---- an isolated shot keeps its hold (kIsolatedHoldS: 0.75 s until HIP-2 of the 2026-10-31 probe, now 1.2 s)
        {
            HipFirePoseHold h;
            for (int i = 0; i < int(0.3f * hz); ++i)
                h.Step(8, dt, true);
            float held = 0;
            while (h.Active() && held < 5.f)
            {
                h.Step(0, dt, true);
                held += dt;
            }
            Check("isolated shot: the pose is held kIsolatedHoldS after the Shoot state" + at,
                  std::fabs(held - kIsolatedHoldS) <= dt + 1e-4f);
        }
        // ---- a second shot inside the link window: sustained, held kSustainedHoldS after it
        {
            HipFirePoseHold h;
            h.Step(8, dt, true);
            float gap = 0;
            while (gap < 1.6f)
            {
                h.Step(0, dt, true);
                gap += dt;
            }
            const bool droppedInGap = !h.Active(); // the first shot is isolated: kIsolatedHoldS < 1.6 s
            h.Step(8, dt, true);
            float held = 0;
            while (h.Active() && held < 5.f)
            {
                h.Step(0, dt, true);
                held += dt;
            }
            Check("second shot 1.6 s after the first: sustained, held 2.0 s" + at,
                  droppedInGap && std::fabs(held - kSustainedHoldS) <= dt + 1e-4f);
        }
        // ---- shots further apart than the link window stay isolated
        {
            HipFirePoseHold h;
            h.Step(8, dt, true);
            for (float t = 0; t < kSustainLinkS + 0.1f; t += dt)
                h.Step(0, dt, true);
            h.Step(8, dt, true);
            Check("shots 2.6 s apart: the second is isolated (kIsolatedHoldS)" + at,
                  !h.sustained && std::fabs(h.remaining - kIsolatedHoldS) < 1e-4f);
        }
        // ---- review M1: a sprint request ends a sustained hold (no 2 s strafe run with a raised weapon); without it the burst sustains
        {
            HipFirePoseHold h;
            h.Step(8, dt, true);
            for (float t = 0; t < 1.0f; t += dt)
                h.Step(0, dt, true);
            h.Step(8, dt, true); // second shot: sustained
            for (float t = 0; t < 0.9f; t += dt)
                h.Step(0, dt, true); // 0.9 s after it: still raised (2.0 s hold)
            const bool sustainedBefore = h.Active() && h.sustained;
            h.Step(0, dt, true, true); // sprint requested, 0.9 s > 0.75 s after the shot: the pose ends now
            const bool endedBySprint = !h.Active() && !h.sustained;
            HipFirePoseHold s;
            s.Step(8, dt, true);
            s.Step(0, dt, true, true); // isolated shot + sprint: the 0.75 s hold is unchanged
            const bool isolatedKept = s.Active() && std::fabs(s.remaining - (OverlayTimes{}.aimingMinTime - dt)) < 1e-4f;
            Check("sprint request ends a sustained hold at once; the isolated 0.75 s hold ignores it" + at,
                  sustainedBefore && endedBySprint && isolatedKept);
        }
        // ---- review L5: the vehicle exit window (synthetic action 8) raises the pose but is no shot of an engagement
        {
            HipFirePoseHold h;
            for (float t = 0; t < 1.0f; t += dt)
                h.Step(8, dt, true, false, true); // exit window
            const bool raised = h.Active();
            for (float t = 0; t < 0.5f; t += dt)
                h.Step(0, dt, true);
            h.Step(8, dt, true); // first real shot 0.5 s after the window
            Check("exit window raises the pose, the first real shot after it is isolated (kIsolatedHoldS)" + at,
                  raised && !h.sustained && std::fabs(h.remaining - kIsolatedHoldS) < 1e-4f);
        }
        // ---- combat probe 2026-10-24 13:38 (shoot_jump): 3 shots ~0.5 s apart, jump 0.02 s / 0.9 s after the last one. Pre-fix the
        //      sustained hold kept aimW 1.0 through the whole flight (13:38:59.13-59.88, 13:39:09.03-09.78). Now the take-off ends it.
        for (float jumpAfter : {0.02f, 0.9f})
        {
            HipFirePoseHold h;
            for (int s = 0; s < 3; ++s)
            {
                for (float t = 0; t < 0.35f; t += dt)
                    h.Step(8, dt, true);
                for (float t = 0; t < 0.15f; t += dt)
                    h.Step(0, dt, true);
            }
            for (float t = 0; t < jumpAfter; t += dt)
                h.Step(0, dt, true);
            const bool raisedBefore = h.Active() && h.sustained;
            h.Step(0, dt, true, false, false, false); // take-off
            bool heldInAir = !h.Active();
            for (float t = 0; t < 0.75f; t += dt) // flight
            {
                h.Step(0, dt, true, false, false, false);
                heldInAir = heldInAir && !h.Active();
            }
            for (float t = 0; t < 0.3f; t += dt) // landing window
            {
                h.Step(0, dt, true, false, false, false);
                heldInAir = heldInAir && !h.Active();
            }
            h.Step(0, dt, true); // grounded again: nothing re-arms
            const bool noRearm = !h.Active() && !h.sustained;
            h.Step(8, dt, true); // first shot after the landing (1.1-2.0 s after the burst): isolated
            Check("jump " + std::to_string(jumpAfter) + " s after a burst: the take-off ends the hold, nothing in the flight / landing, "
                  "the next shot is isolated" + at,
                  raisedBefore && heldInAir && noRearm && !h.sustained &&
                      std::fabs(h.remaining - kIsolatedHoldS) < 1e-4f);
        }
        // ---- the 0.75 s single-shot hold ends in the air too; a Shoot frame in the air holds nothing after it
        {
            HipFirePoseHold h;
            h.Step(8, dt, true);
            h.Step(0, dt, true);
            const bool single = h.Active() && !h.sustained;
            h.Step(0, dt, true, false, false, false);
            const bool endedSingle = !h.Active();
            h.Step(8, dt, true, false, false, false); // shot in the air
            h.Step(0, dt, true, false, false, false);
            Check("airborne: the 0.75 s hold ends, an air shot holds nothing after it" + at,
                  single && endedSingle && !h.Active() && !h.sustained);
        }
        // ---- a reload drops the pose at once but keeps the engagement; unequip clears it
        {
            HipFirePoseHold h;
            h.Step(8, dt, true);
            h.Step(0, dt, true);
            h.Step(2, dt, true);
            const bool offInReload = !h.Active();
            for (float t = 0; t < 1.5f; t += dt)
                h.Step(2, dt, true);
            h.Step(0, dt, true);
            h.Step(8, dt, true);
            const bool linked = h.sustained;
            h.Step(0, dt, true);
            h.Step(0, dt, false); // unequip
            h.Step(8, dt, true);
            Check("reload: pose off at once, the next shot 1.5 s later is sustained; unequip forgets the engagement" + at,
                  offInReload && linked && !h.sustained);
        }
    }
    // ---- 2026-10-28 bike jump-out: the exit window holds the pose through the flight and releases it at the landing (only there)
    {
        // the driver wiring (MotionDriver.cpp) as a table: exitWindowPose = window && !released; Step(action 8 synthetic, grounded = window pose)
        const float dt = 1.0f / 60.0f;
        const auto run = [&](bool aJumpOut, const std::vector<float> &aTtl, int aGroundFrom, int &aReleasedAt, int &aActiveAfter,
                             bool &aActiveBefore) {
            JumpOutExitHold j;
            HipFirePoseHold h;
            aReleasedAt = -1;
            aActiveAfter = 0;
            aActiveBefore = true;
            const int n = int(aTtl.size());
            for (int i = 0; i < n; ++i)
            {
                const bool ground = aGroundFrom >= 0 && i >= aGroundFrom;
                const bool airborne = !ground;
                j.Step(aJumpOut, airborne, aTtl[size_t(i)], ground);
                if (j.ReleaseEdge())
                    h.Reset();
                const bool pose = !j.ReleasesExitHold(); // vehicleExitWindow stays true over the whole run
                if (j.ReleasesExitHold() && aReleasedAt < 0)
                    aReleasedAt = i;
                h.Step(pose ? 8 : 0, dt, true, false, pose, pose || ground);
                const bool active = h.Active() || pose;
                if (aReleasedAt < 0)
                    aActiveBefore = aActiveBefore && active;
                else
                    aActiveAfter += active ? 1 : 0;
            }
        };
        std::vector<float> flight; // 1.2 s of flight with the ground unknown (ttl 1.5), then the approach 0.6 s -> 0, then 0.5 s on the ground
        for (int i = 0; i < 72; ++i)
            flight.push_back(1.5f);
        for (int i = 0; i < 36; ++i)
            flight.push_back(0.6f - 0.6f * float(i) / 36.0f);
        for (int i = 0; i < 30; ++i)
            flight.push_back(0.0f);
        int rel = -1, after = 0;
        bool before = false;
        run(true, flight, 108, rel, after, before);
        // r5 air: the pose is no longer held in the air (the right arm locked forward with the pistol over the jump clip): released on the first
        // airborne feed after the launch feed, never shown again in that episode
        Check("jump-out: the window pose is released in the flight (first airborne feed after the launch feed), not held to the landing lead",
              before && rel == 1 && after == 0);
        // ground unknown to the end (ttl stays 1.5): the same release in the flight
        std::vector<float> blind(120, 1.5f);
        run(true, blind, 100, rel, after, before);
        Check("jump-out: unknown floor (ttl 1.5) - the pose is released in the flight, not at the first ground signal", before && rel == 1 && after == 0);
        // a ground signal before any airborne feed (the launch feed lags) must not release
        {
            JumpOutExitHold j;
            j.Step(true, false, 1.5f, true);
            j.Step(true, false, 1.5f, true);
            const bool notYet = !j.ReleasesExitHold();
            j.Step(true, true, 1.5f, false); // the first airborne feed (the launch feed): no release yet
            const bool onLaunchFeed = !j.ReleasesExitHold();
            j.Step(true, true, 1.5f, false); // the next airborne feed: released (r5 air: no pose in the air)
            const bool inFlight = j.ReleasesExitHold();
            Check("jump-out: no release before the flight was seen or on the launch feed, release on the first airborne feed after it",
                  notYet && onLaunchFeed && inFlight);
        }
        // the launch feed carries the previous step's chooser state (a stale ttl 0 / justLanded on the ground): no release on it
        {
            JumpOutExitHold j;
            j.Step(true, true, 0.0f, true); // first airborne feed, stale landing signals
            const bool launchFeed = !j.ReleasesExitHold() && !j.ReleaseEdge();
            j.Step(true, true, 1.5f, false); // the second airborne feed releases (the flight pose belongs to the jump clips)
            Check("jump-out: stale ttl 0 / ground flag on the launch feed does not release the pose on that feed (the release is the next airborne feed)",
                  launchFeed && j.ReleasesExitHold());
        }
        // a normal exit window (no jump-out) is untouched: never released, even with a landing signal
        run(false, flight, 108, rel, after, before);
        Check("normal exit window (not a jump-out): the pose is held through the lead and the ground signal, never released", rel == -1 && before);
        // released episode stays released until the episode ends; a new jump-out starts held again
        {
            JumpOutExitHold j;
            j.Step(true, true, 1.5f, false);
            j.Step(true, true, 0.1f, false);
            const bool released = j.ReleasesExitHold();
            j.Step(true, true, 1.5f, false); // a stale feed with a long ttl does not re-hold
            const bool stays = j.ReleasesExitHold();
            j.Step(false, false, 1.5f, true); // the window / episode ended
            const bool reset = !j.ReleasesExitHold();
            j.Step(true, true, 1.5f, false);
            Check("jump-out: the release is kept for the episode, reset at its end, the next jump-out holds the pose again",
                  released && stays && reset && !j.ReleasesExitHold());
        }
    }
    std::printf("hipfire sustain: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}

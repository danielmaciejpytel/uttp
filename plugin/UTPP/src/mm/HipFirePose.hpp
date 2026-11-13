#pragma once
#include "Overlay.hpp"
namespace utpp::mm::ov
{
// Sustained fire (recording 2026-10-23 21:08, Nue hipfire 21:10:18-21:10:24): semi-automatic shots 1.1-1.8 s apart. With the
// single-shot hold (aimingMinTime, 0.75 s after the Shoot state) the arms dropped to Ready between every two shots and were raised
// again on the next one (aimW 1.0 -> 0.16 -> 1.0, ~0.5 s down, ~1.1 s up), and with the pose the strafe facing ended, so the body
// stood still while the camera orbited and the next shot had to turn it 135-180 degrees. A Shoot that STARTS within kSustainLinkS
// of the previous Shoot (the hold may already have expired) belongs to the same engagement: after it the pose is held
// kSustainedHoldS. An isolated shot keeps the 0.75 s hold. A reload still ends the pose at once (the reload owns the arms), but
// it does not end the engagement: the first shot after it links to the shots before (the reload of 1.5-2.3 s sits inside the
// window only when the gap allows it). tests/HipFireSustainTest.cpp
inline constexpr float kSustainLinkS = 2.5f;
inline constexpr float kSustainedHoldS = 2.0f;
// HIP-2 (recording 2026-10-31 22:57:36.75 / 37.76, standing walk hipfire): shots that START 1.0 s apart. The Shoot state lasts ~0.2 s, so
// the 0.75 s hold of the first (isolated) shot ended ~50 ms before the second one: Aiming -> Ready (0.5 s) began, the strafe facing
// ended (rot=orient), and the next shot re-raised the arms (aimW 0.97 -> 0.78 -> 1.00, a dip-and-pop of arms and rifle at the shot). The
// isolated shot now holds kIsolatedHoldS after its last Shoot frame: a second shot up to ~1.4 s after the first start (0.2 s Shoot + 1.2 s)
// finds the pose still up and then sustains. Longer gaps of an engagement (1.4 - 2.5 s, the 2026-10-23 cadence of 1.1 - 1.8 s) still
// drop once, the OverlayMachine re-raises such a shot at once from the partial Ready (Overlay.hpp, HIP-2). A sprint request keeps the
// former 0.75 s (OverlayTimes::aimingMinTime) for the isolated shot too, so "shoot once, then sprint" is unchanged. The vehicle exit
// window keeps aimingMinTime. Mirrored by the CET lease (cet/utpp_bodyturn/init.lua HipFireStep). Mutation 4 (UTPP_LOCOHIP_MUTATION) =
// the 0.75 s isolated hold.
#if defined(UTPP_LOCOHIP_MUTATION) && UTPP_LOCOHIP_MUTATION == 4
inline constexpr float kIsolatedHoldS = 0.75f;
#else
inline constexpr float kIsolatedHoldS = 1.2f;
#endif
// Visual firing pose only. Never writes native ADS/PSM, camera FOV or movement chooser input.
struct HipFirePoseHold
{
    static constexpr float kNever = 1.0e9f;
    float remaining = 0.0f;
    float sinceShot = kNever; // time since the last Shoot frame (engagement memory)
    bool inShot = false, sustained = false;
    bool fromShot = false; // `remaining` was set by a real Shoot frame (not by the exit window): the sprint cut of the isolated hold applies
    void Reset() { remaining = 0.0f; sinceShot = kNever; inShot = sustained = fromShot = false; }
    // sprintRequested: the player asks for a sprint (PSM sprint). The pose zeroes the overlay sprint and keeps the strafe facing, so a
    //   sustained hold must not outlive the request: it is demoted to what is left of the 0.75 s (aimingMinTime) after the last Shoot
    //   frame, usually 0. The isolated-shot hold is cut to the same 0.75 s (HIP-2: its kIsolatedHoldS beyond that is for the next shot of
    //   a standing / walking engagement, not for a sprint); within the 0.75 s it ignores the sprint as before.
    // synthetic: action 8 is not a real Shoot (the vehicle exit window holds the pose): it raises the 0.75 s hold but is no shot of
    //   an engagement (sinceShot keeps running, nothing is sustained).
    // grounded (combat probe 2026-10-24 13:38, shoot_jump): false while V is airborne (jump / fall), in a traversal, in the v2 slide or
    //   in the landing window. The sustained 2 s hold kept the raised arms (aimW 1.0, ov 3/2) through the whole jump 0.4 s and 0.9 s
    //   after a burst, the jump clip only moved the legs. There every hold ends at once (also the 0.75 s one) and the engagement is
    //   forgotten, so the landing clip owns the arms and the first shot after it is isolated. A Shoot frame in the air still raises
    //   the arms (UseHipFirePose takes action 8 directly); nothing is held after it.
    void Step(int action, float dt, bool validWeapon, bool sprintRequested = false, bool synthetic = false, bool grounded = true)
    {
        if (!validWeapon) { Reset(); return; }
#if !(defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 4) // mutation 4: the holds survive the air
        if (!grounded)
        {
            remaining = 0.0f;
            sinceShot = kNever;
            sustained = false;
            inShot = action == 8;
            return;
        }
#else
        (void)grounded;
#endif
        const bool advance = std::isfinite(dt) && dt > 0.0f;
        if (action == 2)
        {
            remaining = 0.0f; // the reload owns the arms; the engagement memory keeps running
            inShot = sustained = false;
            if (advance) sinceShot = std::fmin(kNever, sinceShot + dt);
            return;
        }
#if !(defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 3) // mutation 3: the exit window counts as a shot
        if (action == 8 && synthetic)
        {
            inShot = false;
            remaining = std::fmax(remaining, OverlayTimes{}.aimingMinTime);
            fromShot = false; // the window tail keeps its 0.75 s with a sprint request (as before HIP-2)
            if (advance) sinceShot = std::fmin(kNever, sinceShot + dt);
            return;
        }
#else
        (void)synthetic;
#endif
#if !(defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 2) // mutation 2: a sprint request keeps the 2 s hold
#if defined(UTPP_LOCOHIP_MUTATION) && UTPP_LOCOHIP_MUTATION == 5 // mutation 5: the sprint cut also hits the exit-window tail
        const bool cutIsolated = true;
#else
        const bool cutIsolated = fromShot;
#endif
        if (sprintRequested && action != 8 &&
            (sustained || (cutIsolated && remaining > std::fmax(0.0f, OverlayTimes{}.aimingMinTime - sinceShot) + 1.0e-4f)))
        {
            sustained = false;
            remaining = std::fmin(remaining, std::fmax(0.0f, OverlayTimes{}.aimingMinTime - sinceShot));
        }
#else
        (void)sprintRequested;
#endif
        if (action == 8)
        {
            if (!inShot)
            {
#if defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 1
                sustained = false; // mutation 1: every shot is isolated (the pre-fix 0.75 s hold only)
#else
                sustained = sinceShot <= kSustainLinkS;
#endif
                inShot = true;
            }
            sinceShot = 0.0f;
            fromShot = true;
            remaining = sustained ? kSustainedHoldS : kIsolatedHoldS;
        }
        else
        {
            inShot = false;
            if (advance)
            {
                remaining = std::fmax(0.0f, remaining - dt);
                sinceShot = std::fmin(kNever, sinceShot + dt);
            }
        }
    }
    bool Active() const { return remaining > 0.0f; }
};
// Bike jump-out (recording 2026-10-28 05-03-09): the vehicle exit window showed the hip-fire pose through the flight (quick fix
// 2026-10-23), so V came down from ~3 m with the weapon at the face over the heavy landing clip (a deep crouch). The pose was
// released at the landing; r5 air releases it in the flight itself (see Step): from the first airborne feed after the launch feed (or the
// LandApproach lead / the first ground signal when no airborne feed was seen), and stays released for the rest of that jump-out episode
// (aJumpOut false = a normal exit window or no window: nothing changes there). Only the jump-out exit is affected.
inline constexpr float kJumpOutLandLeadS = 0.25f; // s of predicted time to land (AirConfig landLead 0.20 + margin)
struct JumpOutExitHold
{
    bool seenAir = false, released = false, edge = false; // edge: this Step released the pose (the 0.75 s window hold must not outlive it)
    void Reset() { seenAir = released = edge = false; }
    // aAirborne: off the ground (feed air / chooser movement mode). aTtl: predicted s to the touchdown while airborne.
    // aGround: a ground / landing signal (feed onGround, PSM land, chooser justLanded).
    void Step(bool aJumpOut, bool aAirborne, float aTtl, bool aGround)
    {
#if defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 5 // mutation 5: the jump-out window holds the pose through the landing
        (void)aJumpOut; (void)aAirborne; (void)aTtl; (void)aGround;
        edge = false;
        return;
#else
        edge = false;
        if (!aJumpOut) { Reset(); return; }
        if (released) return;
        // the launch feed carries the previous step's chooser state (ground / ride: a stale ttl or justLanded): the release is only
        // evaluated on feeds AFTER the first airborne one
#if defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 6 // mutation 6: the release is evaluated on the launch feed too
        const bool hadAir = true;
#else
        const bool hadAir = seenAir;
#endif
        if (aAirborne) seenAir = true;
        // r5 air (game test r4-jump-cycle 2026-11-04): the window pose at full weight in the air was UTPP's own aim layer (the right arm locked straight
        // forward with the pistol, "superman" arms over the jump clip). The flight belongs to the jump clips: the pose is released on the first airborne
        // feed AFTER the launch feed (the launch feed carries the previous step's state); the landing lead / ground signals below stay as the
        // fallbacks for a flight whose airborne feeds were missed.
#if defined(UTPP_HIPFIRE_SUSTAIN_MUTATION) && UTPP_HIPFIRE_SUSTAIN_MUTATION == 7 // mutation 7: the window holds the pose through the flight (pre-r5)
        const bool flightRelease = false;
#else
        const bool flightRelease = aAirborne;
#endif
        if (hadAir && (aGround || flightRelease || (aAirborne && std::isfinite(aTtl) && aTtl <= kJumpOutLandLeadS))) { released = true; edge = true; }
#endif
    }
    // true: the exit window must not hold the hip-fire pose on this feed (the landing clip owns the arms).
    bool ReleasesExitHold() const { return released; }
    bool ReleaseEdge() const { return edge; }
};
inline bool CaptureNativeReadyReference(int action, bool authoredPrepare, bool idlePreShotCapture)
{
    // Authoring an Aim reference during a shot is NOT a native pre-shot reference.
    return action == 0 && (authoredPrepare || idlePreShotCapture);
}
inline bool RangedPose(PoseOverlay pose)
{
    return pose == PoseOverlay::Rifle || pose == PoseOverlay::Pistol1H || pose == PoseOverlay::Pistol2H;
}
inline bool UseHipFirePose(bool nativeADS, int action, bool held, bool release, bool haveWeapon,
                          bool drawn, PoseOverlay current, PoseOverlay resolvedAim)
{
    return !nativeADS && action != 2 && (action == 8 || held) && !release && haveWeapon && drawn &&
           RangedPose(current) && RangedPose(resolvedAim);
}
// Preserve the previously working native shot while an authored recoil episode is unproven.
inline bool HipFireNeedsNativeAction(bool hipFire, int action, bool transferred, float aimWeight)
{
    // A successful transfer already solved contacts on this frame's authored pose.
    // Partial Aim weight is not a reason to replace those arms with the FPP shot.
    return hipFire && action == 8 && (!transferred || !std::isfinite(aimWeight));
}
// transferRoute: this frame's weapon HAS a route that can replace the native shot (the authored recoil domain or the native FK
// transfer). Without it there is nothing to "preserve until proven": the authored Aim pose (MM + AO + grip) already holds the
// arms and swapping them for the FPP shot only deforms the shoulders for one frame (rifles, SMGs, shotguns, other pistols).
// The weaponUpper option (user choice) keeps the native shot regardless of the route.
inline bool HipFireNativeActionOwner(bool weaponUpper, bool recoilTransfer, bool transferRoute, bool hipFire, int action,
                                    bool transferred, float aimWeight, bool transferInFlight = false)
{
#if defined(UTPP_HIPFIRE_ROUTE_MUTATION) && UTPP_HIPFIRE_ROUTE_MUTATION == 1
    transferRoute = true; // mutation 1: pre-P9, every weapon waits for a transfer that never comes
#endif
    // The bounded capture/admission gate owns the authored pose while waiting for FK.
    // Yielding to FPP for those few frames produces a native->authored arm pop.
    if (recoilTransfer && transferRoute && hipFire && action == 8 && transferInFlight) return false;
    return (weaponUpper || (recoilTransfer && transferRoute)) && HipFireNeedsNativeAction(hipFire, action, transferred, aimWeight);
}
} // namespace utpp::mm::ov

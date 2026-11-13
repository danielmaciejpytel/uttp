#pragma once
#include <algorithm>
#include <cmath>

namespace utpp::mm::shotturn
{
inline constexpr float kFinishDegrees = 5.f;
// The goal is the camera heading of the LAST Shoot sample. Once the camera has left it by more than this the goal is a stale
// direction: footwork towards it turns V away from where she is looking (recording 2026-10-21 12:15, 373.1-373.9 s: camera and
// capsule at -124 deg, V turned back towards the shot at +17 deg and stood 117 deg off the camera for 0.8 s). The hold is dropped
// then, but only while the hipfire pose hold (visualStrafe, 0.75 s after the last Shoot) takes the facing over: after it expired
// nothing else owns the facing, so dropping the goal would cut an admitted 135/180 footwork clip half way and leave V neither on
// the shot nor on the camera. There the hold keeps its original meaning (finish the turn that was started; 3 s / 5 deg).
// Intermittent fire: a gap with the camera > 45 deg off the goal drops the hold and the next Shoot latches the new camera
// heading; a TIP towards the old goal had to be replaced then anyway (same rule as the ADS goal change in ChooserMatcher).
// tests/StationaryShotTurnTest.cpp
inline constexpr float kAbandonDegrees = 45.f;
// Admission (recording 2026-10-23 21:08, Nue / rifle hipfire): every Shoot with the body more than kFinishDegrees off the camera
// started a footwork clip, also for errors the aim offset absorbs (MotionPose spine yaw clamp 60 deg), so short bursts turned V
// 45 deg back and forth. A NEW shot turn is admitted only from the ADS turn-in-place angle (motiontraj::kTurnInPlaceDeg, 50 deg, checked
// in MotionDriver.cpp): below it the arms aim with the aim offset and the feet stay. A running turn keeps re-latching the camera.
inline constexpr float kAdmitDegrees = 50.f;
// A single Shoot state can end well before a 135/180-degree footwork clip.
// Keep only its facing goal; this never extends recoil, ADS, or the arm pose.
struct Hold
{
    float remaining = 0, targetYawDeg = 0;
    int side = 0; // +1 the admitted turn goes left (positive error), -1 right
    bool Active() const { return remaining > 0; }
    void Reset() { remaining = targetYawDeg = 0; side = 0; }
    static int SideOf(float err) { return err > 0 ? 1 : -1; }
    // R3 review L1: near +-180 deg the sign of the error is camera noise. While a turn runs, an error beyond kSideHoldDegrees on the
    // other side is read on the admitted side (unwrapped by 360 deg), so the side, the passed rule and the clip direction the chooser
    // gets (MapContext::shotTurnSide) cannot flip from frame to frame.
    static constexpr float kSideHoldDegrees = 165.f;
    float Unwrapped(float err) const
    {
#if defined(UTPP_SHOTTURN_MUTATION) && UTPP_SHOTTURN_MUTATION == 4
        return err; // mutation 4: no hysteresis at 180 deg
#else
        if (side > 0 && err < -kSideHoldDegrees) return err + 360.f;
        if (side < 0 && err > kSideHoldDegrees) return err - 360.f;
        return err;
#endif
    }
    // handover: the hipfire pose hold (or any other owner of the facing) is active, so a dropped goal is taken over
    void Step(int action, float dt, bool eligible, float viewYaw, float visualYaw, bool handover = true)
    {
        if (!eligible || action == 2 || !std::isfinite(dt) || dt < 0 || dt > .25f ||
            !std::isfinite(viewYaw) || !std::isfinite(visualYaw))
        { Reset(); return; }
        if (dt == 0) return;
        if (action == 8)
        {
            const float goal = std::remainder(viewYaw, 360.f);
            const float err = std::remainder(goal - visualYaw, 360.f);
#if defined(UTPP_SHOTTURN_MUTATION) && UTPP_SHOTTURN_MUTATION == 3
            (void)err; // mutation 3: any error above the finish angle starts a turn and the goal may be passed (pre-fix)
            if (!Active()) side = 0;
#else
            // a new turn, or the camera crossed to the other side of the body: admitted only beyond the aim offset's reach
            if (!Active() || SideOf(Unwrapped(err)) != side)
            {
                if (std::fabs(err) < kAdmitDegrees) { Reset(); return; }
                side = SideOf(err);
            }
#endif
            targetYawDeg = goal;
            remaining = 3.f;
        }
        else
        {
#if !(defined(UTPP_SHOTTURN_MUTATION) && UTPP_SHOTTURN_MUTATION == 1) // mutation 1: the stale goal is never abandoned
#if defined(UTPP_SHOTTURN_MUTATION) && UTPP_SHOTTURN_MUTATION == 2 // mutation 2: abandoned without a successor owner
            (void)handover;
            if (Active() && std::fabs(std::remainder(viewYaw - targetYawDeg, 360.f)) > kAbandonDegrees)
#else
            if (handover && Active() && std::fabs(std::remainder(viewYaw - targetYawDeg, 360.f)) > kAbandonDegrees)
#endif
            { Reset(); return; }
#else
            (void)handover;
#endif
            remaining = std::max(0.f, remaining - dt);
        }
        const float left = std::remainder(targetYawDeg - visualYaw, 360.f);
        // Finished: within kFinishDegrees, or passed (the remaining error changed side). A 180 clip turns ~6 deg per 60 Hz frame and
        // can jump over the 5 deg window; the hold then asked for a turn back (21:12:52.3 Turn_180_L -> 21:12:53.0 Turn_090_R).
#if defined(UTPP_SHOTTURN_MUTATION) && UTPP_SHOTTURN_MUTATION == 3
        const bool passed = false;
#else
        const bool passed = side != 0 && SideOf(Unwrapped(left)) != side;
#endif
        if (!Active() || std::fabs(left) <= kFinishDegrees || passed)
            Reset();
    }
};
}

// UTPP motion matching runtime - magazine top-up gesture of the left hand (refill without a reload state).
// The game's level-50 Shinobi bonus refills part of the magazine of the equipped weapon on every slide / air dash with no reload
// state and no animation (log 'UTPPMotion RELOAD refill-no-state', recording 2026-10-26 11:28: 24 refills, Kyubi +2..3, Nue +1..2,
// mostly on slide frames). In TPP the glue (scripts/utpp_motion.reds OnMagazineAmmo) reports such a refill once
// (UTPPFeedMotionMagTap); the driver (MotionDriver.cpp MagTapStepLocked) plays one short left-hand gesture on the final MM pose of the
// frame: out to the magazine of the weapon, a short hold, back to the grip.
//
// Envelope (one-shot, feed time): kOutSeconds rise, kHoldSeconds hold, kBackSeconds fall, smoothstep ramps. A second trigger
// during the gesture never pops: in the rise it is ignored (the rise goes on), in the hold the hold restarts, in the fall the clock is
// mapped back onto the rise at the SAME weight (inverse smoothstep) and the hand turns around. An ownership loss (reload, game-owned
// upper body, traversal, ...) fades the gesture out over kCancelSeconds from its current weight; ownership back before that recovers
// it at the same rate. Weight exactly 0 when idle: the caller then does not touch the pose (bit-identical frame).
//
// Pose (Apply): two-bone IK of the left arm (gik::SolveArm: elbow plane of the input pose, soft reach clamp, local rotations of
// LeftArm / LeftForeArm / LeftHand blended with the input by the weight; the input model rotation of the hand is kept) to a point
// in the WeaponRight frame (kPistolMagWell below the pistol grip, kRifleMagazine under the receiver in front of the right hand),
// then the forearm roll of the left arm only is moved into the hand (ForearmTwist.hpp convention of the game rig). No other bone is
// written: the clavicles, the right arm, WeaponRight and the weapon keep their local transforms bit for bit.
// Header-only, allocation-free, no game types. Quaternions x, y, z, w, Hamilton product (MMMath.hpp).
// Mutations (UTPP_MAGTAP_MUTATION, tests/magtap.cmake): 1 = trigger ignored (the driver never plays the gesture),
// 2 = a retrigger restarts the clock at 0 (weight pop in the fall), 4 = no wait for the slide de-roll (SlideDefer).
#pragma once

#include <cmath>

#include "Additive.hpp"
#include "ForearmTwist.hpp"
#include "GripIK.hpp"

#ifndef UTPP_MAGTAP_MUTATION
#define UTPP_MAGTAP_MUTATION 0
#endif

namespace utpp::mm::magtap
{
using math::Quat;
using math::Vec3;

inline constexpr float kOutSeconds = 0.15f;   // grip -> magazine
inline constexpr float kHoldSeconds = 0.10f;  // on the magazine
inline constexpr float kBackSeconds = 0.20f;  // magazine -> grip
inline constexpr float kCancelSeconds = 0.10f; // fade out / in of an ownership loss
inline constexpr float kMaxDt = 0.10f;         // one feed step never advances the gesture by more (hitch)

// Targets of the LeftHand bone (the wrist) in the WeaponRight frame, metres. Frame of the woman overlays (tests/MagTapDriverTest.cpp
// section A prints it): +X = right side of the weapon (model left = -X in Rifle / Pistol2H Ready), +Y = barrel (model forward in
// Pistol2H Ready), +Z = weapon up; the RightHand bone sits at (0.016, -0.090, -0.039). Authored left grips: Pistol2H Ready
// (-0.039, -0.086, -0.089) (support hand cupping the grip), Rifle Ready (-0.063, 0.092, -0.084) (hand on the magazine well).
// Pistol: the support palm slides 7 cm down the grip onto the magazine base. Rifle: the palm drops ~11 cm onto the magazine in
// front of the trigger grip.
inline constexpr Vec3 kPistolMagWell{-0.03f, -0.10f, -0.16f};
inline constexpr Vec3 kRifleMagazine{-0.05f, 0.06f, -0.19f};
// A gesture starts only when the left hand is at most this far from its target (a one-handed pistol hold, a lowered rifle with the
// left hand off the weapon: Pistol1H Ready ~0.95 m, which would be an arm swing, not a top-up).
inline constexpr float kMaxTravel = 0.30f;

enum class Kind : int
{
    Pistol = 0, // overlays Pistol1H / Pistol2H
    Rifle = 1,  // overlay Rifle
};

inline Vec3 TargetOffset(Kind aKind) { return aKind == Kind::Rifle ? kRifleMagazine : kPistolMagWell; }

// Slide roll-in (recording 2026-10-27 15:14: the refill comes with the first slide frames, when the rifle carry torso rolls ~35 deg and
// the whole weapon moves; the gesture was lost in that motion): a NEW rifle gesture requested on a slide frame waits until the slide
// torso de-roll (ReloadContactFade.hpp SlideDeRoll, carry path) has settled (weight >= kSlideSettledWeight) or kSlideDelaySeconds of
// feed time have passed since the first deferred Step, whichever comes first; it starts at once off slide frames. Pistols (no carry
// de-roll) never wait. The driver widens the stale window of a deferred request by kSlideDelaySeconds (MotionDriver.cpp).
// Mutation 4 (UTPP_MAGTAP_MUTATION) disables the wait.
inline constexpr float kSlideDelaySeconds = 0.35f;
inline constexpr float kSlideSettledWeight = 0.9f;
inline bool SlideDefer([[maybe_unused]] Kind aKind, [[maybe_unused]] bool aSlideFrame, [[maybe_unused]] float aDeRollWeight,
                       [[maybe_unused]] float aWaitedSeconds)
{
#if UTPP_MAGTAP_MUTATION == 4
    return false; // mutation 4: no wait
#else
    return aKind == Kind::Rifle && aSlideFrame && !(aDeRollWeight >= kSlideSettledWeight) &&
           std::isfinite(aWaitedSeconds) && aWaitedSeconds < kSlideDelaySeconds;
#endif
}

inline float Smooth(float x)
{
    x = x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
    return x * x * (3.0f - 2.0f * x);
}
// Inverse of Smooth on [0, 1].
inline float SmoothInv(float y)
{
    y = y < 0.0f ? 0.0f : (y > 1.0f ? 1.0f : y);
    return 0.5f - std::sin(std::asin(1.0f - 2.0f * y) / 3.0f);
}

struct Envelope
{
    bool active = false;
    float t = 0.0f;    // time on the timeline [0, kOut + kHold + kBack]
    float own = 1.0f;  // ownership fade (1 = owned)
    float weight = 0.0f;

    static constexpr float Total() { return kOutSeconds + kHoldSeconds + kBackSeconds; }
    static float Shape(float aT)
    {
        if (aT < kOutSeconds)
            return Smooth(aT / kOutSeconds);
        if (aT < kOutSeconds + kHoldSeconds)
            return 1.0f;
        return 1.0f - Smooth((aT - kOutSeconds - kHoldSeconds) / kBackSeconds);
    }
    bool Idle() const { return !active; }
    void Reset() { *this = Envelope{}; }
    // A refill reported for this frame (admitted by the caller). Never changes the current weight.
    void Trigger()
    {
#if UTPP_MAGTAP_MUTATION == 1
        return; // mutation 1: the trigger is ignored
#else
        if (!active)
        {
            active = true;
            t = 0.0f;
            own = 1.0f;
            weight = 0.0f;
            return;
        }
#if UTPP_MAGTAP_MUTATION == 2
        t = 0.0f; // mutation 2: naive restart (pop in the hold / fall)
        return;
#else
        if (t < kOutSeconds)
            return; // still rising
        if (t < kOutSeconds + kHoldSeconds)
        {
            t = kOutSeconds; // the hold restarts
            return;
        }
        // falling: back onto the rise at the same envelope weight (the hand turns around, no jump)
        t = kOutSeconds * SmoothInv(Shape(t));
#endif
#endif
    }
    // Advances by aDt (feed time, clamped to kMaxDt; invalid dt = no advance); aOwned: MM owns the arms this frame.
    // Returns the effective weight (envelope x ownership fade), exactly 0 when idle.
    float Step(float aDt, bool aOwned)
    {
        if (!active)
            return weight = 0.0f;
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? (aDt > kMaxDt ? kMaxDt : aDt) : 0.0f;
        t += dt;
        own += (aOwned ? dt : -dt) / kCancelSeconds;
        own = own < 0.0f ? 0.0f : (own > 1.0f ? 1.0f : own);
        if (t >= Total() || own <= 0.0f)
        {
            Reset();
            return 0.0f;
        }
        weight = Shape(t) * Smooth(own);
        return weight;
    }
};

struct ApplyResult
{
    bool applied = false;     // the pose was edited
    bool clamped = false;     // the target lay outside the soft reach band of the arm
    float handErr = 0.0f;     // |hand - target| of the full-weight solve (m)
    float handMoved = 0.0f;   // |hand after - hand before| (model space, m)
    float targetDist = 0.0f;  // |target - hand before| (m)
};

// Distance (m) of the LeftHand bone from the magazine target of aKind in this pose; < 0 = invalid rig / pose.
inline float TargetDistance(const ov::Skeleton& aSk, const ov::Pose& aPose, const gik::Rig& aRig, Kind aKind)
{
    if (!aRig.Valid(aSk) || aPose.n != aSk.n)
        return -1.0f;
    Quat rw, rh;
    Vec3 pw, ph;
    gik::ModelOf(aSk, aPose, aRig.weaponRight, rw, pw);
    gik::ModelOf(aSk, aPose, aRig.left.hand, rh, ph);
    const float d = gik::Len(pw + math::QuatRotate(rw, TargetOffset(aKind)) - ph);
    return std::isfinite(d) ? d : -1.0f;
}

// Moves the left hand toward the magazine target of aKind by aWeight. aWeight <= kZeroWeight (or an invalid rig) = pose untouched.
inline bool Apply(const ov::Skeleton& aSk, ov::Pose& aIo, const gik::Rig& aRig, Kind aKind, float aWeight, ApplyResult& aOut)
{
    aOut = ApplyResult{};
    const float w = ov::Clamp01(aWeight);
    if (!(w > ov::kZeroWeight) || !aRig.Valid(aSk) || aIo.n != aSk.n)
        return false;
    Quat rw, rh;
    Vec3 pw, ph;
    gik::ModelOf(aSk, aIo, aRig.weaponRight, rw, pw);
    gik::ModelOf(aSk, aIo, aRig.left.hand, rh, ph);
    const Vec3 target = pw + math::QuatRotate(rw, TargetOffset(aKind));
    if (!math::IsFinite(target))
        return false;
    aOut.targetDist = gik::Len(target - ph);
    // candidate first: the caller's pose changes only on a finite, successful solve
    ov::Pose cand;
    cand.n = aIo.n;
    for (int i = 0; i < aIo.n; ++i)
        cand.b[i] = aIo.b[i];
    gik::Result r;
    if (!gik::SolveArm(aSk, cand, aRig.left, target, nullptr, w, gik::Params{}, r) || !r.ok)
        return false;
    // forearm roll of the LEFT arm into the hand (game rig convention; the right arm is not touched)
    int children[ov::ForearmTwistRig::kMaxChildren];
    int count = 0;
    for (int i = 0; i < aSk.n && count < ov::ForearmTwistRig::kMaxChildren; ++i)
        if (aSk.parent[i] == aRig.left.lower)
            children[count++] = i;
    (void)ov::MoveForearmTwistToHand(cand, aRig.left.lower, aRig.left.hand, children, count);
    for (const int b : {aRig.left.upper, aRig.left.lower, aRig.left.hand})
        if (!math::IsFinite(cand.b[b].r))
            return false;
    for (int k = 0; k < count; ++k)
        if (!math::IsFinite(cand.b[children[k]].r) || !math::IsFinite(cand.b[children[k]].t))
            return false;
    aIo.b[aRig.left.upper] = cand.b[aRig.left.upper];
    aIo.b[aRig.left.lower] = cand.b[aRig.left.lower];
    for (int k = 0; k < count; ++k)
        aIo.b[children[k]] = cand.b[children[k]];
    Quat rh2;
    Vec3 ph2;
    gik::ModelOf(aSk, aIo, aRig.left.hand, rh2, ph2);
    aOut.applied = true;
    aOut.clamped = r.clamped;
    aOut.handErr = r.handErr;
    aOut.handMoved = gik::Len(ph2 - ph);
    return true;
}
} // namespace utpp::mm::magtap

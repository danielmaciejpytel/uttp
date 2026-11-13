// Which frames the native motion offset root treats as "montage" (OffsetRoot Release mode). Header-only, no game state.
//
// MotionOffsetRoot releases the rotation offset (half-life 0.1 s, bounded by kReleaseMaxRate) when montage && !aimFacing. A reload
// on the spot has no explicit aim facing (reload clears the hipfire / shot-turn facing), so counting it as montage made the root
// shed whatever yaw offset the body had accumulated while the capsule was turned by the camera: the visible V spun 100-180 deg in
// ~0.3 s without footwork (recording 2026-10-21 12:15: 83.1, 88.4, 95.0 s; 20 stationary reloads with |offset| > 45).
// The reload upper body is native either way, so only the ROTATION policy changes here: a reload counts as montage only when it
// began in motion and V still moves. A reload that began on the spot stays out of montage when V starts moving mid-reload
// (the first review of 4bdffd6: ~770 deg/s, 102 deg in 0.3 s at the first moving frame); the remaining offset is then folded
// back by ordinary locomotion (start clip yaw, steering), as for any start from rest. Translation is unaffected (released on the
// spot anyway). ADS montage keeps its policy.
#pragma once

#ifndef UTPP_ROOT_MONTAGE_MUTATION
#define UTPP_ROOT_MONTAGE_MUTATION 0 // 1 = every reload is montage (pre-4bdffd6), 2 = no began-moving latch (4bdffd6); tests must FAIL
#endif

namespace utpp::mm::rootframe
{
// Latches, on the first frame of a reload (reload upper weight > 0), whether V was moving. Resets when the weight returns to 0.
struct ReloadRootLatch
{
    bool active = false, beganMoving = false;
    void Reset()
    {
        active = beganMoving = false;
    }
    bool Step(float reloadUpperWeight, bool movingGround)
    {
        if (!(reloadUpperWeight > 0))
        {
            Reset();
            return false;
        }
        if (!active)
        {
            active = true;
            beganMoving = movingGround;
        }
        return beganMoving;
    }
};

inline bool RootMontage(bool weaponOverlay, bool aimGameUpper, bool aimingNow, float reloadUpperWeight, bool movingGround,
                        bool reloadBeganMoving = true)
{
    const bool aimMontage = aimGameUpper && aimingNow;
#if UTPP_ROOT_MONTAGE_MUTATION == 1
    (void)movingGround;
    (void)reloadBeganMoving;
    const bool reloadMontage = reloadUpperWeight > 0;
#elif UTPP_ROOT_MONTAGE_MUTATION == 2
    (void)reloadBeganMoving;
    const bool reloadMontage = reloadUpperWeight > 0 && movingGround;
#else
    const bool reloadMontage = reloadUpperWeight > 0 && movingGround && reloadBeganMoving;
#endif
    return weaponOverlay && (aimMontage || reloadMontage);
}
} // namespace utpp::mm::rootframe

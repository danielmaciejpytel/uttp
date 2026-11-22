// Earlier DLL startup values of the settings whose DLL defaults now equal the menu defaults (tests only).
// Scenario tests written against the earlier DLL defaults call these at the start of main, so their bounds keep covering the
// configuration they were tuned for; a test that checks a new default reads it before the call. Only the helpers a test calls are
// compiled into it (inline), so a test that links only the driver or only the host calls only that helper.
// Declarations only (as in src/MotionDriver.hpp, src/MotionHost.hpp, src/mm/MeleeAimRuntime.hpp), so host-only tests with their own
// helpers can include it.
#pragma once

namespace utpp::motiondriver
{
void SetAimGameUpper(bool aOn);
bool SetAimWorldSmoothing(bool aOn, float aHz);
bool SetHeadLead(bool aOn);
bool SetPelvisYawTrim(float aDeg);
bool SetBodyYawOffsets(float aFemaleDeg, float aMaleDeg);
bool SetBarrelLoop(bool aOn);
bool SetLeanParams(float aScale, float aHeadScale);
bool SetWalkSpeed(float aMs);
bool SetMinAnalogWalk(float aMs);
void SetWeaponUpper(bool aOn);
void SetShootTransferEnabled(bool aOn);
void SetMagTapEnabled(bool aOn);
bool SetOverlayInertialSwitch(bool aOn);
} // namespace utpp::motiondriver

namespace utpp::motionhost
{
void SetWeaponUpper(bool aOn);
void SetAirSteer(bool aOn);
bool SetClimbRelease(bool aOn);
bool SetStrafeParams(bool aUnarmed, float aRootTurnMaxDeg, float aRootWarpMaxDeg, float aExitHold, float aJogF);
bool SetTurnRateParams(float aMin, float aMax);
void SetMeleeGuardSetting(bool aOn);
bool SetFirmFeet(bool aOn);
} // namespace utpp::motionhost

namespace utpp::meleeaim
{
bool SetParams(float aMaxUpDeg, float aMaxDownDeg, float aSmoothHz);
} // namespace utpp::meleeaim

namespace utpp::testdefaults
{
// Driver: aim game upper body on, world aim smoothing off, head lead off, pelvis trim 0, body yaw offsets 0 / 0, barrel loop off,
// lean amounts 1 / 1, walk speeds off, weapon upper body on, Pistol2H shot transfer off, magazine top-up off, inertialized overlay
// switch off.
inline void LegacyDriverDefaults()
{
    motiondriver::SetAimGameUpper(true);
    motiondriver::SetAimWorldSmoothing(false, 0.0f);
    motiondriver::SetHeadLead(false);
    motiondriver::SetPelvisYawTrim(0.0f);
    motiondriver::SetBodyYawOffsets(0.0f, 0.0f);
    motiondriver::SetBarrelLoop(false);
    motiondriver::SetLeanParams(1.0f, 1.0f);
    motiondriver::SetWalkSpeed(0.0f);
    motiondriver::SetMinAnalogWalk(0.0f);
    motiondriver::SetWeaponUpper(true);
    motiondriver::SetShootTransferEnabled(false);
    motiondriver::SetMagTapEnabled(false);
    motiondriver::SetOverlayInertialSwitch(false);
}

// Host: weapon upper body on, air steering off, climb release on, unarmed strafe off with the walk gait, turn clip max 1.1, melee guard off,
// firm feet in turns and stops off.
inline void LegacyHostDefaults()
{
    motionhost::SetWeaponUpper(true);
    motionhost::SetAirSteer(false);
    motionhost::SetClimbRelease(true);
    motionhost::SetStrafeParams(false, 10.0f, 30.0f, 0.15f, 0.0f);
    motionhost::SetTurnRateParams(0.9f, 1.1f);
    motionhost::SetMeleeGuardSetting(false);
    motionhost::SetFirmFeet(false);
}

// Melee aim tilt off (0 / 0, smoothing 10 Hz as before); only for tests that link mm/MeleeAim.cpp.
inline void LegacyMeleeAimDefaults() { meleeaim::SetParams(0.0f, 0.0f, 10.0f); }
} // namespace utpp::testdefaults

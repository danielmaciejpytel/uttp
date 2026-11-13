#pragma once
// UTPP 0.6.3 aim offset runtime (mm/AimOffset.cpp): lock-free state between the game thread (natives, arming, feed tap) and
// the FK detour (animation threads). Core math: mm/AimOffset.hpp.
//
// Data flow:
//   CET (each frame while the utpp_camera aim state is on) -> UTPPSetAimTarget(camPos, camFwd, true) -> Main.cpp reads the
//   player entity world transform (GetWorldPosition / GetWorldOrientation), P = camPos + norm(camFwd) * targetDistance,
//   converts P to model space -> SetTarget() (seqlock);
//   UTPPFeedLocomotion -> OnFeed(tpp, psmBlockMask) (PSM.UpperBody, melee and vehicle bits, feed time);
//   FK detour -> ApplyMotionMatching -> BlendIntoRecords -> Apply() -> guards -> Controller::Apply on the local records.

#include <cstdint>
#include <string>
#include <vector>

namespace utpp::aimofs
{
// Game thread, arm time: resolves RightArm / RightHand / WeaponRight / LeftArm / LeftForeArm / LeftHand by name.
void OnArmed(void* aRig, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents);
void OnDisarmed();
// Game thread: tap of every UTPPFeedLocomotion (raw UTPPPsmBlockMask incl. diagnostic fields).
void OnFeed(bool aTpp, uint32_t aPsmBlockMask);
// Game thread: mirrors UTPPSetMMUpperMaskStates (bit 2 = aim) after the driver accepted it.
void SetMaskStates(int32_t aBits);
// Game thread: aim target in model space. aValid = false when the player transform could not be read (fail closed).
void SetTarget(bool aEnabled, bool aValid, const float aTargetModel[3]);
float TargetDistance();
// maxYaw / maxPitch 0..60 deg, smoothHz 0..120 (exponential rate 1/s, 0 = none). maxYaw = maxPitch = 0 disables the offset.
bool SetParams(float aMaxYawDeg, float aMaxPitchDeg, float aSmoothHz);
// Barrel axis of WeaponRight: 1 = X, 2 = Y (default), 3 = Z, negative = opposite direction.
bool SetAxis(int32_t aAxis);
std::string Status();

// FK detour (animation thread), after the MM blend. aMMWeight / aUpperScale from the published snapshot. No allocation, no
// locks (a try-flag skips concurrent calls), no logging.
void Apply(uint8_t* aBones, uint32_t aCount, void* aRig, float aMMWeight, float aUpperScale, int64_t aNowTicks);
} // namespace utpp::aimofs

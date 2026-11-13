// UTPP Locomotion - native 360-degree movement input, exported by red4ext/plugins/UTPP/UTPP.dll.
// While active, the player state machine sees MoveX = 0, MoveY = |stick| and GetInputHeading() = 0 (every direction is
// "forward" relative to the body, so sprint and the other locomotion gates work in all directions). The real
// camera-relative stick angle is UTPPLocomotionRawAngle(). Default OFF; off as well when the
// hook could not be proven, or after UTPPLocomotionKill().
// Hard kill switch without the game: create an empty file utpp_locomotion.disable next to UTPP.dll.
//
// Requires utpp.reds (UTPPIsTppActive, UTPPPsmBlockMask, UTPPYawDegFromForward) in the same script folder.

// Status line for the console: print(Game.UTPPLocomotionStatus())
public native func UTPPLocomotionStatus() -> String
// User setting (default false). Ignored (stays inactive) when not installed / killed.
public native func UTPPLocomotionSetEnabled(on: Bool) -> Void
// Session kill switch: pass-through until the game restarts.
public native func UTPPLocomotionKill() -> Void
// External hold (free look, camera transitions, other UTPP modules): true blocks Locomotion.
public native func UTPPLocomotionSetHold(on: Bool) -> Void
// Which heartbeat bits block (default 0x7FBF = all below). E.g. 0x7FBF & ~8192 lets Locomotion run with a weapon in hand.
public native func UTPPLocomotionSetBlockMask(mask: Int32) -> Void
// Heartbeat timeout s (0.25), stick deadzone (0.15), horizontal camera distance treated as TPP m (0.6). False = invalid.
public native func UTPPLocomotionSetParams(staleSeconds: Float, deadzone: Float, tppCameraDistance: Float) -> Bool
// Internal: called every PSM locomotion update below.
public native func UTPPLocomotionHeartbeat(ownerSaysTpp: Bool, cameraDistance2D: Float, blockBits: Int32) -> Void
public native func UTPPLocomotionActive() -> Bool
// Bitmask of reasons Locomotion is not active (0 = active); bits documented in src/input/LocomotionCore.hpp.
public native func UTPPLocomotionReasons() -> Int32
// Raw stick (before the rewrite) above the deadzone and fresh.
public native func UTPPLocomotionHasInput() -> Bool
// Raw stick angle relative to the camera, degrees CCW, 0 = forward, +90 = left, -90 = right, 180 = back.
public native func UTPPLocomotionRawAngle() -> Float

// Heartbeat bits (mirror of utpp::freemove::Reason 0..14). Keep the existing low-13 PSM guards except bit 64.
// PSM bits 13+ are packed diagnostics, NOT Locomotion reasons: UpperBody=ForceEmptyHands encodes 0xC000,
// which would otherwise become a false kHold (0x4000). Weapon and aim reasons are derived below.
public func UTPPLocomotionBlockBits(scriptInterface: ref<StateGameScriptInterface>, player: ref<PlayerPuppet>) -> Int32 {
  let bits: Int32 = UTPPPsmBlockMask(scriptInterface) & 8127; // 0x1FBF; retain legacy melee guards too
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  if IsDefined(bb) {
    let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    // gamePSMDetailedLocomotionStates: AimWalk 2, Climb 8, Vault 9, Ladder 10..13, Knockdown 29, Felled 31
    let detailed: Int32 = bb.GetInt(psm.LocomotionDetailed);
    if detailed == 2 {
      bits = bits | 4096;
    }
    if (detailed >= 8 && detailed <= 13) || detailed == 29 || detailed == 31 {
      bits = bits | 2048;
    }
    // gamePSMUpperBodyStates.Aim = 6
    if bb.GetInt(psm.UpperBody) == 6 {
      bits = bits | 4096;
    }
  }
  let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(player.GetGame());
  if IsDefined(ts) && IsDefined(ts.GetItemInSlot(player, t"AttachmentSlots.WeaponRight")) {
    bits = bits | 8192;
  }
  return bits;
}

public func UTPPLocomotionTick(scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if !IsDefined(player) {
    return;
  }
  // Keep the raw stick cache fresh (both reads go through the native hook, which records the raw values).
  scriptInterface.GetActionValue(n"MoveX");
  scriptInterface.GetActionValue(n"MoveY");
  let cam: Vector4 = Transform.GetPosition(scriptInterface.GetCameraWorldTransform());
  let pos: Vector4 = player.GetWorldPosition();
  let dx: Float = cam.X - pos.X;
  let dy: Float = cam.Y - pos.Y;
  UTPPLocomotionHeartbeat(UTPPIsTppActive(player.GetGame()), SqrtF(dx * dx + dy * dy), UTPPLocomotionBlockBits(scriptInterface, player));
}

// World target facing for a body-turn controller: camera yaw + raw stick angle while Locomotion is active and the stick is
// moved (UTPP convention, see utpp.reds).
public func UTPPLocomotionTarget(cameraYawDeg: Float, out targetYawDeg: Float) -> Bool {
  targetYawDeg = 0.0;
  if !UTPPLocomotionActive() || !UTPPLocomotionHasInput() {
    return false;
  }
  targetYawDeg = cameraYawDeg + UTPPLocomotionRawAngle();
  return true;
}

// Slide camera exit. Vanilla SlideDecisions.ShouldExit / SlideFallDecisions.ToFall / SlideDecisions.HasValidDodgeAngle
// (the only callers of GetCameraYaw in 2.31) leave the slide as soon as |angle(camera forward, BODY forward)| >= 95 deg. Locomotion
// turns the body onto the movement, so a sprint toward the camera (and diagonal back) gives ~180 deg and the slide ended after one
// frame. While Locomotion drives the body AND the motion slide feed owns the slide (UTPPSlideOwned: mm.slide on + a slide bundle) the
// camera never ends the slide; it still ends on the crouch release / minSpeedToExit as in the game. Anything else: vanilla value.
@wrapMethod(DefaultTransition)
protected final const func GetCameraYaw(const stateContext: ref<StateContext>, const scriptInterface: ref<StateGameScriptInterface>) -> Float {
  if UTPPLocomotionActive() && UTPPSlideOwned() {
    return 0.0;
  }
  return wrappedMethod(stateContext, scriptInterface);
}

// Chained with the utpp.reds wraps of the same method (redscript chains all wraps). Air states reach it via super.
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  UTPPLocomotionTick(scriptInterface);
  wrappedMethod(timeDelta, stateContext, scriptInterface);
}

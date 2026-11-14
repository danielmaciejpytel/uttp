// UTPP - native globals exported by red4ext/plugins/UTPP/UTPP.dll + the motion matching feed.
// Callable from redscript and from CET (e.g. print(Game.UTPPGetStatus())).
//
// UTPPArm() codes: >= 0 bitmask (1 = trackA, 2 = trackB, 4 = bone resolved);
// -1 busy, -2 no player, -3 no animation system, -4 no "root" animated object,
// -5 read fault, -6 no tracks.
// UTPPSetTestAxis: 0 = local X, 1 = local Y (default), 2 = local Z.
// UTPPSetTestBone returns true only if the name resolved against the armed rig.
//
// Uses the UTPP Locomotion natives declared in utpp_locomotion.reds (same folder; both files are always shipped together).
// TPP signal: UTPPSetTppOverride / UTPPGetTppOverride below, set by the CET module utpp_camera.

public native func UTPPGetStatus() -> String
// Immediate pose-write permission heartbeat. Native stores one QPC timestamp:
// true = current QPC, false = 0; stale/unknown permission forbids MM and AO.
// Positive permission is sent by the rendering camera after a successful TPP frame.
public native func UTPPSetPoseWriteTpp(active: Bool) -> Bool
// Explicit helper trace, also with MM off. Arm while reproducing the pose, then Flush after ~4 s.
// Captures at most 200 MAIN-rig FK calls; does not modify transforms or start automatically.
public native func UTPPBodyPoseTraceArm(frames: Int32) -> Bool
public native func UTPPBodyPoseTraceFlush() -> Bool
public native func UTPPBodyPoseTraceStatus() -> String
// Continuous capture of ALL helper bones (both stages) by a background writer thread into red4ext\logs\utpp-bodytrace-<time>.csv (+ posthelper file);
// idempotent; refused (false) while an explicit UTPPBodyPoseTraceArm session runs; the explicit Arm refuses while this is on.
public native func UTPPBodyPoseTraceContinuous(on: Bool) -> Bool
// Skin helper recompute on the frames of the core writers that run at motion weight 0 (vehicle transition, vehicle guards, seated neck
// cut, vault torso). On by default; off = only the frames of the published pose are recomputed. Always returns true.
public native func UTPPSetHelperZeroWeightWriters(on: Bool) -> Bool
// Compact pose capture of ALL helper bones (both stages, every frame; 16-bit values, differences between frames, block compression) into
// red4ext\logs\utpp-pose-<time>.utpppose. keepMinutes 0 = the whole session (one file per rig); > 0 = one-minute segments named utpp-pose-ring-<time>.utpppose, only the last keepMinutes
// minutes stay on disk. Refused (false) while the text capture above or an explicit Arm session runs, or while a capture with ANOTHER window runs
// (call Stop first); the same window again = true, no change. Stop returns the session's files (paths
// relative to the game folder, joined with '|'; empty when nothing was written); Status = state, frames, lost frames, bytes, file.
public native func UTPPPoseCaptureStart(keepMinutes: Int32) -> Bool
// Asks the captures started afterwards for the helper bones after the mod's helper commit as a second stream (this game session only).
public native func UTPPSetPostHelperStream(on: Bool) -> Bool
public native func UTPPPoseCaptureStop() -> String
public native func UTPPPoseCaptureStatus() -> String
// Writes red4ext\logs\utpp-report-<time>.txt for a bug report: mod and game version, `body`, then the mod's files of the last 30 minutes with the
// instruction to zip and send them. Returns the report's path relative to the game folder ("" when it could not be written).
public native func UTPPDiagWriteReport(body: String) -> String
// Read-only mapping/FK order census; default off. Dump disables capture and returns a CSV path.
public native func UTPPPoseCensus(enabled: Bool) -> Bool
public native func UTPPPoseCensusDump() -> String
public native func UTPPSetEnabled(enabled: Bool) -> Void
public native func UTPPSetTestRotation(degrees: Float) -> Void
public native func UTPPSetTestAxis(axis: Int32) -> Void
public native func UTPPSetTestBone(boneName: String) -> Bool
public native func UTPPArm() -> Int32
// 0 = tag (D+0x18 == trackA/B, default), 1 = FK descriptor == player desc, 2 = FK rig == player metaRig, 3 = any
public native func UTPPSetMatchMode(mode: Int32) -> Void

// ---- Motion matching ----
// Yaw convention (degrees): counter-clockwise about +Z from world +Y to the facing direction,
// i.e. forward = (-sin(yaw), cos(yaw)). velWorld: capsule linear velocity in world space (m/s).
// moveInput = (MoveX, MoveY) raw action values. locoState: see UTPPLocoStateId. dt: PSM timeDelta (s).
// haveTargetYaw/targetYawDeg: world target facing (valid only while the stick is moved) - drives the
// body-turn matcher input: the target comes from UTPPOwnMoveTarget (UTPP Locomotion raw angle, or player yaw +
// stick angle) while the UTPP override is on.
// psmBlockMask: UTPPPsmBlockMask below; non-zero closes the MM gate (fast fade-out, reopen hold).
public native func UTPPFeedLocomotion(velWorld: Vector4, moveInput: Vector2, cameraYawDeg: Float, playerYawDeg: Float, locoState: Int32, tppActive: Bool, dt: Float, haveTargetYaw: Bool, targetYawDeg: Float, psmBlockMask: Int32) -> Void
// extra fields of the NEXT UTPPFeedLocomotion call (sent right before it): the PSM permanent bool WalkToggled (vanilla
// walk toggle: LocomotionDetailed Crouch while Locomotion stays stand; deliberately not a psmBlockMask bit) and the capsule
// (player entity) world position. Diagnostics only (feed dump columns walk, capX capY capZ).
public native func UTPPFeedLocomotionExtra(walkToggled: Bool, capsulePos: Vector4) -> Void
// AIR HOLD stage: air fields of the NEXT UTPPFeedLocomotion call (sent right before it, UTPPFeedAirFromLocomotion):
// onGround = IsOnGround(), distToGround = downward raycast (m below the feet; -1 no hit, -2 not probed), fallState = PSM.Fall
// (read before the game writes it this frame: the previous frame's value).
public native func UTPPFeedAir(onGround: Bool, distToGround: Float, fallState: Int32) -> Void
// True when the next air feed should carry a raycast: airborne (last air feed), descending, mm.airProbe on, every 2nd feed.
public native func UTPPMMAirWantsProbe() -> Bool
// fade-out time (s) for hard closures (PSM block, TPP off, stale feed; default 0.12) and the time the gate must
// stay open after a state closure before MM fades back in (default 0.10). Both 0..5; false = invalid, previous kept.
public native func UTPPSetMMGateFade(hardOutSeconds: Float, reopenHoldSeconds: Float) -> Bool
// Default false. MM weight target is 1 only if enabled, db loaded, armed + bone map ok, tppActive and locoState allowed.
public native func UTPPSetMMEnabled(enabled: Bool) -> Void
// Restrict matching to clips whose tag contains the substring (e.g. "Stand_"); "" = all clips. False = no tag matched.
public native func UTPPSetMMTagFilter(filter: String) -> Bool
// Fade time (s, smoothstep) of the MM weight, default 0.25, range 0..5.
public native func UTPPSetMMBlendTime(seconds: Float) -> Void
// Bitmask of allowed locoState ids (bit n = id n). Default 2 (= Stand only).
public native func UTPPSetMMAllowedStates(mask: Int32) -> Void
// True when MM is enabled and the database is loaded (the feed below is skipped otherwise); also while MM is disabled
// but its last pose is held (UTPPSetMMHoldPose), so the PSM state can release the body.
public native func UTPPMMWantsFeed() -> Bool
// Count of vehicle jump-out launches (the flight after an unmount: the native edge that releases the PSM vehicle bit). Monotonic, never
// reset; a caller compares it with a stored value to see each new launch.
public native func UTPPVehicleJumpOutEpoch() -> Int32

// One log line with the numeric values of the vanilla enum members UTPP compares as hard-coded integers (reds and CET Lua),
// so a game patch that renumbers them shows up in the UTPP log.
public func UTPPLogEnumContract() -> Void {
  UTPPLogLine("[utpp_enums] game enum contract"
    + " UpperBody:TemporaryUnequip=" + IntToString(EnumInt(gamePSMUpperBodyStates.TemporaryUnequip))
    + ",ForceEmptyHands=" + IntToString(EnumInt(gamePSMUpperBodyStates.ForceEmptyHands))
    + ",Aim=" + IntToString(EnumInt(gamePSMUpperBodyStates.Aim))
    + ",DualWield=" + IntToString(EnumInt(gamePSMUpperBodyStates.DualWield))
    + " Ranged:Charging=" + IntToString(EnumInt(gamePSMRangedWeaponStates.Charging))
    + ",Reload=" + IntToString(EnumInt(gamePSMRangedWeaponStates.Reload))
    + ",QuickMelee=" + IntToString(EnumInt(gamePSMRangedWeaponStates.QuickMelee))
    + ",Overheat=" + IntToString(EnumInt(gamePSMRangedWeaponStates.Overheat))
    + ",Shoot=" + IntToString(EnumInt(gamePSMRangedWeaponStates.Shoot))
    + " Detailed:Stand=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Stand))
    + ",AimWalk=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.AimWalk))
    + ",Crouch=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Crouch))
    + ",Sprint=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Sprint))
    + ",Slide=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Slide))
    + ",Climb=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Climb))
    + ",LadderJump=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.LadderJump))
    + ",HoverJump=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.HoverJump))
    + ",Knockdown=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Knockdown))
    + ",CrouchSprint=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.CrouchSprint))
    + ",Felled=" + IntToString(EnumInt(gamePSMDetailedLocomotionStates.Felled))
    + " Loco:Crouch=" + IntToString(EnumInt(gamePSMLocomotionStates.Crouch))
    + ",Slide=" + IntToString(EnumInt(gamePSMLocomotionStates.Slide))
    + ",SlideFall=" + IntToString(EnumInt(gamePSMLocomotionStates.SlideFall))
    + ",CrouchSprint=" + IntToString(EnumInt(gamePSMLocomotionStates.CrouchSprint))
    + ",CrouchDodge=" + IntToString(EnumInt(gamePSMLocomotionStates.CrouchDodge))
    + " Melee:Parried=" + IntToString(EnumInt(gamePSMMeleeWeapon.Parried))
    + ",Targeting=" + IntToString(EnumInt(gamePSMMeleeWeapon.Targeting))
    + ",Deflect=" + IntToString(EnumInt(gamePSMMeleeWeapon.Deflect))
    + ",ComboAttack=" + IntToString(EnumInt(gamePSMMeleeWeapon.ComboAttack))
    + ",EquipAttack=" + IntToString(EnumInt(gamePSMMeleeWeapon.EquipAttack))
    + " Gadget:Charging=" + IntToString(EnumInt(gamePSMCombatGadget.Charging))
    + ",Throwing=" + IntToString(EnumInt(gamePSMCombatGadget.Throwing))
    + ",QuickThrow=" + IntToString(EnumInt(gamePSMCombatGadget.QuickThrow))
    + " LeftCw:Charge=" + IntToString(EnumInt(gamePSMLeftHandCyberware.Charge))
    + ",Loop=" + IntToString(EnumInt(gamePSMLeftHandCyberware.Loop))
    + ",QuickAction=" + IntToString(EnumInt(gamePSMLeftHandCyberware.QuickAction))
    + ",ChargeAction=" + IntToString(EnumInt(gamePSMLeftHandCyberware.ChargeAction))
    + " Combat:InCombat=" + IntToString(EnumInt(gamePSMCombat.InCombat))
    + " HighLevel:Swimming=" + IntToString(EnumInt(gamePSMHighLevel.Swimming))
    + " Vehicle:Driving=" + IntToString(EnumInt(gamePSMVehicle.Driving))
    + ",Passenger=" + IntToString(EnumInt(gamePSMVehicle.Passenger))
    + ",Transition=" + IntToString(EnumInt(gamePSMVehicle.Transition))
    + ",DriverCombat=" + IntToString(EnumInt(gamePSMVehicle.DriverCombat))
    + ",Scene=" + IntToString(EnumInt(gamePSMVehicle.Scene))
    + " Reaction:Stagger=" + IntToString(EnumInt(gamePSMReaction.Stagger))
    // dual-wield D0: gamePSMShootStates (PSM.ShootState arbiter, game 2.5; member order from the 2.5 bytecode name table)
    + " ShootState:RightWeapon=" + IntToString(EnumInt(gamePSMShootStates.RightWeapon))
    + ",LeftWeapon=" + IntToString(EnumInt(gamePSMShootStates.LeftWeapon))
    + ",BothWeapons=" + IntToString(EnumInt(gamePSMShootStates.BothWeapons))
    + ",ValentinosFrenzyInit=" + IntToString(EnumInt(gamePSMShootStates.ValentinosFrenzyInit))
    + ",ValentinosFrenzyRightOnly=" + IntToString(EnumInt(gamePSMShootStates.ValentinosFrenzyRightOnly))
    + ",ValentinosFrenzy=" + IntToString(EnumInt(gamePSMShootStates.ValentinosFrenzy)));
}

// PSM locomotion state name -> UTPP id: stand 1, crouch 2, sprint 3, anything else 0. The forced walk state
// (ForceWalkEvents: detailed state Stand, WalkToggled cleared) counts as stand, so the MM gate (kGateLocoState) no longer closes there.
// Note: the base gate opens in that state too; the feed carries no walk flag (WalkToggled is false in it), so the chooser sees a plain
// stand with the game's own slow forced-walk speed.
public func UTPPLocoStateId(stateName: CName) -> Int32 {
  switch stateName {
    case n"stand":
      return 1;
    case n"forceWalk":
      return 1;
    case n"ForceWalk":
      return 1;
    case n"crouch":
      return 2;
    case n"sprint":
      return 3;
    default:
      return 0;
  }
}

// Yaw in degrees (convention above) of a world-space forward vector. AtanF's argument order is not documented, so the
// result is verified against forward = (-sin, cos) and the swapped order is used if the check fails.
public func UTPPYawDegFromForward(fwd: Vector4) -> Float {
  let len: Float = SqrtF(fwd.X * fwd.X + fwd.Y * fwd.Y);
  if len < 0.0001 {
    return 0.0;
  }
  let fx: Float = fwd.X / len;
  let fy: Float = fwd.Y / len;
  let yaw: Float = AtanF(-fx, fy);
  if AbsF(-SinF(yaw) - fx) + AbsF(CosF(yaw) - fy) > 0.01 {
    yaw = AtanF(fy, -fx);
  }
  return Rad2Deg(yaw);
}

// ---- UTPP-own TPP signal ----
// "Is the third-person body active" is answered by UTPP's own state only; without this signal the MM gate stays closed
// (bit 0x8, tpp=0).
// The CET module utpp_camera reports its own TPP state here on every change (enter / exit, first frame, session reset,
// shutdown): CET Game.UTPPSetTppOverride(true|false), fallback
// Game.GetScriptableSystemsContainer():Get("UTPPPerspectiveState"):SetTppOverride(true|false).
// Console check: print(Game.UTPPGetTppOverride())   print(Game.UTPPTppOverrideStatus())
public class UTPPPerspectiveState extends ScriptableSystem {
  private let m_tppOverride: Bool;
  private let m_changes: Int32;
  private let m_calls: Int32;

  // Returns true when the value changed.
  public func SetTppOverride(active: Bool) -> Bool {
    // Revoke before changing perspective state, even when the override was
    // already false. A true override alone cannot establish rendering-camera ownership.
    if !active {
      UTPPSetPoseWriteTpp(false);
    }
    this.m_calls += 1;
    if Equals(this.m_tppOverride, active) {
      return false;
    }
    this.m_tppOverride = active;
    this.m_changes += 1;
    // render-plane policy (utpp_itemplane.reds, always shipped with this file): every edge of the TPP signal
    // re-syncs it, here and not in the global wrapper (CET's fallback route calls this method directly).
    UTPPItemPlaneOnTppSignal(this.GetGameInstance(), active);
    return true;
  }

  public func GetTppOverride() -> Bool {
    return this.m_tppOverride;
  }

  public func Status() -> String {
    let on: String = "false";
    if this.m_tppOverride {
      on = "true";
    }
    return "tppOverride=" + on + " changes=" + IntToString(this.m_changes) + " calls=" + IntToString(this.m_calls);
  }
}

public func UTPPPerspectiveStateOf(game: GameInstance) -> ref<UTPPPerspectiveState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPPerspectiveState") as UTPPPerspectiveState;
}

// CET: Game.UTPPSetTppOverride(true|false). False = the system is not available (no game session yet); nothing stored.
public func UTPPSetTppOverride(active: Bool) -> Bool {
  let state: ref<UTPPPerspectiveState> = UTPPPerspectiveStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetTppOverride(active);
  return true;
}

public func UTPPGetTppOverride() -> Bool {
  return UTPPTppOverrideFor(GetGameInstance());
}

public func UTPPTppOverrideFor(game: GameInstance) -> Bool {
  let state: ref<UTPPPerspectiveState> = UTPPPerspectiveStateOf(game);
  return IsDefined(state) && state.GetTppOverride();
}

public func UTPPTppOverrideStatus() -> String {
  let game: GameInstance = GetGameInstance();
  let state: ref<UTPPPerspectiveState> = UTPPPerspectiveStateOf(game);
  if !IsDefined(state) {
    return "UTPPPerspectiveState missing";
  }
  let tpp: String = "false";
  if UTPPIsTppActive(game) {
    tpp = "true";
  }
  return state.Status() + " tppActive=" + tpp;
}

// Third-person body active: the UTPP override (utpp_camera).
// Used by the MM feed (gate bit 0x8) and by Locomotion's heartbeat (utpp_locomotion.reds).
public func UTPPIsTppActive(game: GameInstance) -> Bool {
  return UTPPTppOverrideFor(game);
}

// target facing (used only while the TPP signal is on):
//   * UTPP Locomotion active (utpp_locomotion.reds): the scripts see the rewritten stick (0, |s|) and the capsule moves along the
//     camera heading + raw stick -> target = camera yaw + UTPPLocomotionRawAngle();
//   * otherwise the game moves the capsule relative to the body (vanilla): target = player yaw + atan2(-x, y) of the move
//     input. In utpp_camera's default mode the game turns V with the mouse (player yaw == camera yaw), so this equals
//     camera yaw + stick angle; in free look / idle orbit it stays the real run direction.
public func UTPPOwnMoveTarget(cameraYawDeg: Float, playerYawDeg: Float, move: Vector2, out targetYawDeg: Float) -> Bool {
  targetYawDeg = 0.0;
  if UTPPLocomotionActive() {
    if !UTPPLocomotionHasInput() {
      return false;
    }
    targetYawDeg = cameraYawDeg + UTPPLocomotionRawAngle();
    return true;
  }
  if move.X * move.X + move.Y * move.Y <= 0.0225 {
    return false;
  }
  targetYawDeg = playerYawDeg + UTPPYawDegFromForward(Vector4(move.X, move.Y, 0.0, 0.0));
  return true;
}

// gate signals from the player state machine blackboard (fail closed: missing blackboard -> blocked).
// Bit layout of the returned mask (mirror of PsmBlock / kPsm*Field in src/mm/GameIntegration.hpp):
//   bit  0 (1)      vehicle (PSM.Vehicle != Default or MountedToVehicle; also cuts the MM weight at once, UTPPSetMMMountCut)
//   bit  1 (2)      scene tier 3..5 / unknown high level
//   bit  2 (4)      swimming
//   bit  3 (8)      workspot
//   bit  4 (16)     takedown / grapple
//   bit  5 (32)     body carry
//   bit  6 (64)     detailed locomotion not stand / crouch / sprint
//   bit  7 (128)    vitals (dead / resurrecting)
//   bit  8 (256)    photo mode or menu
//   bit  9 (512)    no blackboard (returned alone)
//   bit 10 (1024)   reaction (stagger)
//   bit 11 (2048)   melee attack: PSM.MeleeWeapon ComboAttack..EquipAttack (11..21) or PSM.Weapon QuickMelee (3)
//   bit 12 (4096)   melee pose: PSM.MeleeWeapon Parried 5, Hold 6, ChargedHold 7, Block 8, Targeting 9, Deflect 10
//   bits 13..15     diagnostic: PSM.UpperBody + 1 (0 = not reported) - never a gate reason
//   bits 16..23     diagnostic: PSM.LocomotionDetailed + 1, only together with bit 6
//   bits 24..28     diagnostic: PSM.MeleeWeapon + 1 (0 = not reported) - never a gate reason
//   bits 29 / 30    WeaponRight known / drawn - diagnostic, stripped natively
//   bit  31         dual-wield D0 diagnostic: UpperBody DualWield 7 || PSM.IsDualWielding (Int32 sign bit) - never a gate reason
// Bits 0..10 close the gate (kGatePsmBlock). Bits 11 / 12 are handled natively (UTPPSetMMMeleeGate: soft close like a crouch
// blip, normal fade, reopen 0.3 s after the melee state ends; off = ignored). Not blocking: MeleeWeapon NotReady 0, Equipping 1,
// Idle 2, Safe 3 (aiming at a friendly / safe area: weapon lowered while moving), PublicSafe 4, Default 22; every UpperBody
// value (SwitchItems, Reload, Aim, ...) and ranged firing (PSM.Weapon Shoot 8): upper-body layers - locomotion stays MM.
// (UTPPSetMMUpperMask, default on): the melee gate, UpperBody Aim (with its AimWalk bits) and the vehicle exit window no
// longer fade the whole body out - MM keeps Hips / Spine / legs and the game drives Spine1 and above (bone mask, design 29).
public func UTPPPsmBlockMask(scriptInterface: ref<StateGameScriptInterface>) -> Int32 {
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  if !IsDefined(bb) {
    return 512;
  }
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  let mask: Int32 = 0;
  // gamePSMVehicle: Default 0
  if bb.GetInt(psm.Vehicle) != 0 || bb.GetBool(psm.MountedToVehicle) {
    mask = mask | 1;
  }
  // gamePSMHighLevel: Default 0, SceneTier1..5 = 1..5, Swimming 6
  let high: Int32 = bb.GetInt(psm.HighLevel);
  if high == 6 {
    mask = mask | 4;
  } else {
    if high < 0 || high > 2 {
      mask = mask | 2;
    }
  }
  // gamePSMSwimming: Default 0
  if bb.GetInt(psm.Swimming) != 0 {
    mask = mask | 4;
  }
  if bb.GetInt(psm.IsInWorkspot) != 0 {
    mask = mask | 8;
  }
  // gamePSMTakedown: Default 0
  if bb.GetInt(psm.Takedown) != 0 {
    mask = mask | 16;
  }
  // gamePSMBodyCarrying: Default 0
  if bb.GetInt(psm.BodyCarrying) != 0 || bb.GetBool(psm.Carrying) {
    mask = mask | 32;
  }
  // gamePSMDetailedLocomotionStates: Stand 1, Crouch 3, Sprint 4 (everything else, incl. NotInBaseLocomotion 0, blocks)
  // the value + 1 goes into bits 16..23 (diagnostic for the native trace, stripped before the gate); only together
  // with bit 64, so an older UTPP.dll sees a non-zero mask exactly when it did before.
  let detailed: Int32 = bb.GetInt(psm.LocomotionDetailed);
  // ragdoll: the vanilla car / bike knockdown state keeps detail Stand (1); while its effect exists the feed reports
  // Knockdown 29 so the game's knockdown / get-up clips (and the ragdoll) are not covered by the MM pose (utpp_ragdoll.reds)
  if (detailed == 1 || detailed == 3 || detailed == 4) && UTPPRgKnockdownFeed(scriptInterface.executionOwner) {
    detailed = EnumInt(gamePSMDetailedLocomotionStates.Knockdown);
  }
  if detailed != 1 && detailed != 3 && detailed != 4 {
    mask = mask | 64;
    if detailed >= 0 && detailed < 255 {
      mask = mask | ((detailed + 1) * 65536);
    }
  }
  // gamePSMVitals: Alive 0
  if bb.GetInt(psm.Vitals) != 0 {
    mask = mask | 128;
  }
  // gamePSMReaction: Default 0, Stagger 1
  if bb.GetInt(psm.Reaction) != 0 {
    mask = mask | 1024;
  }
  // gamePSMMeleeWeapon (vanilla 2.31 orphans.script enum; values written by meleeTransitions.script *Events.OnEnter)
  let melee: Int32 = bb.GetInt(psm.MeleeWeapon);
  if melee >= 11 && melee <= 21 {
    mask = mask | 2048;
  } else {
    if melee >= 5 && melee <= 10 {
      mask = mask | 4096;
    }
  }
  // gamePSMRangedWeaponStates QuickMelee 3 (weaponTransitions.script QuickMeleeEvents): a melee strike with a gun.
  // Game 2.5 split PSM.Weapon into RightWeapon / LeftWeapon (dual wield); the left hand counts only while dual wielding
  // (UpperBody DualWield 7 or PSM.IsDualWielding, same gate as utpp_motion.reds), so a single weapon behaves exactly as before.
  let dualWield: Bool = bb.GetInt(psm.UpperBody) == 7 || bb.GetBool(psm.IsDualWielding);
  if bb.GetInt(psm.RightWeapon) == 3 || (dualWield && bb.GetInt(psm.LeftWeapon) == 3) {
    mask = mask | 2048;
  }
  if melee >= 0 && melee < 31 {
    mask = mask | ((melee + 1) * 16777216);
  }
  // gamePSMUpperBodyStates Default 0 .. Aim 6 (diagnostic only). Game 2.5 DualWield 7 (DualWieldEvents: two weapons
  // equipped, not an aim state) does not fit the 3-bit field and stays unreported there (-1 for every legacy consumer on the
  // native side, exactly as before).
  // Dual-wield D0: bit 31 (the Int32 sign bit, the only free bit; mirror kPsmDualFlag in src/mm/GameIntegration.hpp) is set
  // while dualWield (above) is true. The native side strips it from the gate bits and uses it for diagnostics only: the status
  // string (dualWield= upperBodyRaw=). Field 0 + flag = UpperBody 7. Behaviour is unchanged.
  let upper: Int32 = bb.GetInt(psm.UpperBody);
  if upper >= 0 && upper < 7 {
    mask = mask | ((upper + 1) * 8192);
  }
  if dualWield {
    mask = mask | (-2147483647 - 1);
  }
  let game: GameInstance = scriptInterface.GetGame();
  // WeaponRight diagnostic only, bits29/30. Native strips both before evaluating the locomotion gate.
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(game);
  if IsDefined(player) && IsDefined(ts) {
    mask = mask | 536870912;
    if IsDefined(ts.GetItemInSlot(player, t"AttachmentSlots.WeaponRight")) {
      mask = mask | 1073741824;
    }
  }
  let photo: ref<PhotoModeSystem> = GameInstance.GetPhotoModeSystem(game);
  if IsDefined(photo) && photo.IsPhotoModeActive() {
    mask = mask | 256;
  }
  let ui: ref<IBlackboard> = scriptInterface.GetBlackboardSystem().Get(GetAllBlackboardDefs().UI_System);
  if IsDefined(ui) && ui.GetBool(GetAllBlackboardDefs().UI_System.IsInMenu) {
    mask = mask | 256;
  }
  return mask;
}

// ---- capsule acceleration matched to the MM start clips ----
// The UTPP modifier is a single Multiplier (<= 1) on the player's Acceleration stat, owned through the fields below. It is
// re-evaluated every locomotion update and removed (exact vanilla value) whenever the native returns 0: MM disabled or
// not ready, gate closed (FPP, unsupported PSM state, vehicle, scene...), stale feed, kill switch off. The stop
// deceleration gets the same treatment (UTPPUpdateDecel, separate modifier, only while the stick is released; the
// dash/landing modifiers and vanilla's back-input modifier are kept). MaxSpeed (top speed) is never touched.
@addField(PlayerPuppet)
public let utppAccelMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppAccelMul: Float;

public func UTPPRemoveAccelMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppAccelMod) {
    stats.RemoveModifier(id, player.utppAccelMod);
    player.utppAccelMod = null;
  }
  player.utppAccelMul = 1.0;
}

public func UTPPUpdateAccel(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  let applied: Float = 1.0;
  if IsDefined(player.utppAccelMod) && player.utppAccelMul > 0.0001 {
    applied = player.utppAccelMul;
  }
  // Vanilla value = current / our multiplier (all other modifiers, incl. vanilla multipliers, are kept as they are).
  let current: Float = stats.GetStatValue(id, gamedataStatType.Acceleration);
  let base: Float = current / applied;
  let target: Float = UTPPMMAccelTarget(base, applied);
  let want: Float = 1.0;
  if target > 0.0 && base > target {
    want = ClampF(target / base, 0.05, 1.0);
  }
  if want >= 0.999 {
    UTPPRemoveAccelMod(player, stats, id);
    return;
  }
  if IsDefined(player.utppAccelMod) && AbsF(want - applied) < 0.02 {
    return;
  }
  UTPPRemoveAccelMod(player, stats, id);
  player.utppAccelMod = RPGManager.CreateStatModifier(gamedataStatType.Acceleration, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppAccelMod);
  player.utppAccelMul = want;
}

// ---- stop deceleration matched to the MM stop clips (UTPPSetMMCapsuleSync) ----
// Vanilla 2.31 reads gamedataStatType.Deceleration in locomotionTransitions.script (SetMovementDecelerationStatModifier,
// GetSpeedBasedOnDistance, back-input AddDecelerationStatModifier); one Multiplier (<= 1) owned by the fields below, removed
// (exact vanilla value) whenever the native returns 0 (capsule sync off, MM not driving, stick held).
@addField(PlayerPuppet)
public let utppDecelMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppDecelMul: Float;

public func UTPPRemoveDecelMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppDecelMod) {
    stats.RemoveModifier(id, player.utppDecelMod);
    player.utppDecelMod = null;
  }
  player.utppDecelMul = 1.0;
}

public func UTPPUpdateDecel(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  let applied: Float = 1.0;
  if IsDefined(player.utppDecelMod) && player.utppDecelMul > 0.0001 {
    applied = player.utppDecelMul;
  }
  let current: Float = stats.GetStatValue(id, gamedataStatType.Deceleration);
  let base: Float = current / applied;
  let target: Float = UTPPMMDecelTarget(base, applied);
  let want: Float = 1.0;
  if target > 0.0 && base > target {
    want = ClampF(target / base, 0.1, 1.0);
  }
  if want >= 0.999 {
    UTPPRemoveDecelMod(player, stats, id);
    return;
  }
  if IsDefined(player.utppDecelMod) && AbsF(want - applied) < 0.02 {
    return;
  }
  UTPPRemoveDecelMod(player, stats, id);
  player.utppDecelMod = RPGManager.CreateStatModifier(gamedataStatType.Deceleration, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppDecelMod);
  player.utppDecelMul = want;
}

// ---- jog while aiming (CET bodyturn.aimJog / bodyturn.aimJogSpeed -> UTPPSetMMStrafeParams jogF) ----
// Vanilla 2.31 sets the aim walk top speed through the AimWalk stat modifier group (player_locomotion.tweak
// player_locomotion_data_AimWalk: MaxSpeed 1.5 + Additive Reflexes curve max_speed_in_aim_walk; measured capsule 1.63-1.67 m/s)
// and adds Additive MaxSpeed modifiers (sprint, reload: locomotionTransitions.script AddMaxSpeedModifier / EnableReloadStatModifier).
// UTPP adds one Multiplier on MaxSpeed, owned by the fields below, only while LocomotionDetailed == AimWalk (2) and UpperBody ==
// Aim (6) and the native answers a jog target (strafe mode Active in the jog gait): multiplier = target / 1.65 (the nominal aim
// walk speed), so weapon / reload / Reflexes differences keep their relative effect; target = jogF x profile of the body-relative
// stick angle (F : side : back = 1 : 0.70 : 0.60). Re-applied only when the multiplier changes by > 0.02 (stick released: kept);
// removed (exact vanilla value) as soon as any condition fails, e.g. on the first locomotion update after the aim ends.
@addField(PlayerPuppet)
public let utppAimSpeedMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppAimSpeedMul: Float;

public func UTPPRemoveAimSpeedMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppAimSpeedMod) {
    stats.RemoveModifier(id, player.utppAimSpeedMod);
    player.utppAimSpeedMod = null;
  }
  player.utppAimSpeedMul = 1.0;
}

public func UTPPUpdateAimSpeed(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>, move: Vector2, moving: Bool) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  let aimWalk: Bool = false;
  if IsDefined(bb) {
    let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    // gamePSMDetailedLocomotionStates.AimWalk 2, gamePSMUpperBodyStates.Aim 6
    aimWalk = bb.GetInt(psm.LocomotionDetailed) == 2 && bb.GetInt(psm.UpperBody) == 6;
  }
  // the native query also runs with aimWalk false (status aimJogT / counters); 0 = vanilla
  let target: Float = UTPPMMAimJogTarget(move.X, move.Y);
  if !aimWalk || target <= 0.0 {
    UTPPRemoveAimSpeedMod(player, stats, id);
    return;
  }
  if !moving && IsDefined(player.utppAimSpeedMod) {
    return;
  }
  let want: Float = ClampF(target / 1.65, 0.5, 2.5);
  if AbsF(want - 1.0) < 0.001 {
    UTPPRemoveAimSpeedMod(player, stats, id);
    return;
  }
  let applied: Float = 1.0;
  if IsDefined(player.utppAimSpeedMod) {
    applied = player.utppAimSpeedMul;
    if AbsF(want - applied) <= 0.02 {
      return;
    }
  }
  UTPPRemoveAimSpeedMod(player, stats, id);
  player.utppAimSpeedMod = RPGManager.CreateStatModifier(gamedataStatType.MaxSpeed, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppAimSpeedMod);
  player.utppAimSpeedMul = want;
}

// ---- Shooter mode (CET bodyturn.shooterMode -> UTPPSetMotionShooterMode): motion set speeds by direction ----
// With the mode live the chooser strafes, so the capsule must run at the speed the side / back clips were authored for. UTPP adds one
// Multiplier on MaxSpeed (owned by the fields below, same approach as the aim jog above) while LocomotionDetailed is Stand (1) or
// Crouch (3): multiplier = the native answer (UTPPMotionShooterModeSpeedMul: motion set StrafeSpeedMap ratio of the body-relative stick angle -
// run 1 : 0.70 : 0.60 forward : side : back, walk 1 : 0.90 : 0.75, crouch 1 : 0.89 : 0.80). The native returns exactly 1.0 while the
// mode is off / not live, the stick is below the dead zone or the state is not Stand / Crouch, so with the mode off no modifier is
// ever added. Re-applied only when the multiplier changes by > 0.02 (stick released: kept); removed (exact vanilla value) as soon as
// the answer is 1.0, the state changes (sprint, aim walk, slide, air, ...) or the feed is off. AimWalk (2) keeps the aim jog above.
@addField(PlayerPuppet)
public let utppShooterModeSpeedMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppShooterModeSpeedMul: Float;

public func UTPPRemoveShooterModeSpeedMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppShooterModeSpeedMod) {
    stats.RemoveModifier(id, player.utppShooterModeSpeedMod);
    player.utppShooterModeSpeedMod = null;
  }
  player.utppShooterModeSpeedMul = 1.0;
}

public func UTPPUpdateShooterModeSpeed(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>, move: Vector2, moving: Bool, tpp: Bool, stateName: CName, walkToggled: Bool) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  // first person: no motion set strafe clip plays, vanilla speed. Checked before the "stick released: keep" rule below, so a modifier left
  // from third person (TPP -> FPP while moving / standing) never survives the view change
  if !tpp {
    UTPPRemoveShooterModeSpeedMod(player, stats, id);
    return;
  }
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  let detailed: Int32 = 0;
  if IsDefined(bb) {
    let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    detailed = bb.GetInt(psm.LocomotionDetailed);
  }
  // the walk toggle (G) is suspected to put LocomotionDetailed on Crouch (3) while the locomotion state stays 'stand' (the toggle
  // selects the crouch stat group; unconfirmed in 2.5 - the [utpp_walk] log line of UTPPUpdateWalkSpeed records both values). V then walks
  // standing and the chooser plays the stand WALK gait (the feed carries WalkToggled as wantsToWalk), so the native must answer the walk
  // proportions (1 / 0.90 / 0.75 forward / side / back), not the crouch ones (1 / 0.89 / 0.80): report it as Stand (1), the native takes
  // the gait the chooser plays. A real crouch has the state 'crouch'. Unchanged when the toggle is off or the state is not 'stand'.
  // The walk speed multiplier of UTPPUpdateWalkSpeed (target / 2.72) and this one are separate MaxSpeed Multipliers and multiply: the
  // forward walk speed is the walk target, the side / back speeds are that target x 0.90 / 0.75 (intended: motion set's walk strafe speeds).
  if detailed == 3 && walkToggled && Equals(stateName, n"stand") {
    detailed = 1;
  }
  // gamePSMDetailedLocomotionStates.Stand 1, Crouch 3
  if detailed != 1 && detailed != 3 {
    UTPPRemoveShooterModeSpeedMod(player, stats, id);
    return;
  }
  // stick released or below the 0.15 gate: keep the last multiplier, except under a small tilt the minimum analog walk speed lifts
  // (UTPPAnalogFloorWouldApply): there it is recomputed (the native answers 1 below 0.15 -> removed), so a stale side / back value
  // (~0.7 / 0.6) never stacks under the floor. Floor setting 0 or a released stick (|mv|^2 <= 0.0004): unchanged.
  if !moving && IsDefined(player.utppShooterModeSpeedMod) && !UTPPAnalogFloorWouldApply(scriptInterface, stateName, move, walkToggled) {
    return;
  }
  let want: Float = ClampF(UTPPMotionShooterModeSpeedMul(move.X, move.Y, detailed), 0.5, 1.0);
  if AbsF(want - 1.0) < 0.001 {
    UTPPRemoveShooterModeSpeedMod(player, stats, id);
    return;
  }
  let applied: Float = 1.0;
  if IsDefined(player.utppShooterModeSpeedMod) {
    applied = player.utppShooterModeSpeedMul;
    if AbsF(want - applied) <= 0.02 {
      return;
    }
  }
  UTPPRemoveShooterModeSpeedMod(player, stats, id);
  player.utppShooterModeSpeedMod = RPGManager.CreateStatModifier(gamedataStatType.MaxSpeed, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppShooterModeSpeedMod);
  player.utppShooterModeSpeedMul = want;
}

// ---- walk toggle speed (CET mm.walkSpeed -> UTPPSetMotionWalkSpeed) ----
// The vanilla walk toggle (G, PSM bool WalkToggled) holds the capsule at 2.72 m/s at the full stick (measured game 2.5, capsule |v| stable;
// without G the full stick gives 3.84 m/s), while the motion speed map puts game 1.6 m/s on the motion set walk cap (200 cm/s): at 2.72 the walk loops slid (root offset -10 .. -18 cm).
// UTPP adds one Multiplier on MaxSpeed, owned by the fields below, only while WalkToggled is on, the locomotion state is 'stand'
// (LocomotionDetailed Stand 1, or Crouch 3 when the toggle selects the crouch stat group), the third-person body is active, V is on the
// ground and no Tier2 Slow applies; multiplier = target / 2.72 (the toggle's measured full-stick speed; a constant like 1.65 in the aim
// jog, the live MaxSpeed statistic is NOT read), target = UTPPMotionWalkSpeedTarget() (CET mm.walkSpeed, default 1.6 = the game walk point
// of the motion speed map; 0 = vanilla). Removed (exact vanilla value) as soon as one condition fails: toggle off, sprint, aim walk (AimWalk 2), a real
// crouch (state 'crouch'), air, first person, Tier2 Slow, motion not driving or target 0. First person is checked before the "stick released:
// keep" rule, so a modifier from third person never survives the view change. Re-applied only when the multiplier changes by > 0.02.
// The Shooter mode side / back multiplier multiplies with this one on purpose (UTPPUpdateShooterModeSpeed).
@addField(PlayerPuppet)
public let utppWalkSpeedMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppWalkSpeedMul: Float;

@addField(PlayerPuppet)
public let utppWalkLogKey: Int32;

@addField(PlayerPuppet)
public let utppWalkLogToggled: Bool;

public func UTPPRemoveWalkSpeedMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppWalkSpeedMod) {
    stats.RemoveModifier(id, player.utppWalkSpeedMod);
    player.utppWalkSpeedMod = null;
  }
  player.utppWalkSpeedMul = 1.0;
}

// One log line per change of (LocomotionDetailed, WalkToggled) around a walk toggle session (the line before the toggle and after it, with the
// locomotion state name), to confirm in game whether the toggle makes LocomotionDetailed 3 (Crouch) at the state 'stand' or leaves it 1.
public func UTPPLogWalkState(player: ref<PlayerPuppet>, stateName: CName, walkToggled: Bool, detailed: Int32) -> Void {
  // only Stand 1 / AimWalk 2 / Crouch 3: jumps, slides etc. during a toggle session would only add noise (UTPPLogLine is always written)
  if detailed < 1 || detailed > 3 {
    return;
  }
  let key: Int32 = detailed + 1;
  if walkToggled {
    key += 100;
  }
  if key == player.utppWalkLogKey {
    return;
  }
  let was: Bool = player.utppWalkLogToggled;
  player.utppWalkLogKey = key;
  player.utppWalkLogToggled = walkToggled;
  if !walkToggled && !was {
    return;
  }
  let toggled: String = "0";
  if walkToggled {
    toggled = "1";
  }
  UTPPLogLine("[utpp_walk] walkToggled=" + toggled + " detailed=" + IntToString(detailed) + " state=" + NameToString(stateName));
}

public func UTPPUpdateWalkSpeed(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>, stateName: CName, walkToggled: Bool, moving: Bool, tpp: Bool, tier2Slow: Bool) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  let detailed: Int32 = 0;
  if IsDefined(bb) {
    let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    detailed = bb.GetInt(psm.LocomotionDetailed);
  }
  UTPPLogWalkState(player, stateName, walkToggled, detailed);
  // first person: vanilla speed, before the "stick released: keep" rule
  if !tpp {
    UTPPRemoveWalkSpeedMod(player, stats, id);
    return;
  }
  let target: Float = UTPPMotionWalkSpeedTarget();
  // gamePSMDetailedLocomotionStates.Stand 1, Crouch 3 (the toggle's stat group); AimWalk 2 / Sprint 4 and every other state: vanilla
  let active: Bool = walkToggled && Equals(stateName, n"stand") && (detailed == 1 || detailed == 3) && scriptInterface.IsOnGround() && !tier2Slow && target > 0.0;
  if !active {
    UTPPRemoveWalkSpeedMod(player, stats, id);
    return;
  }
  if !moving && IsDefined(player.utppWalkSpeedMod) {
    return;
  }
  // 2.72 m/s = the capsule speed of the unmodified walk toggle at the full stick (measured game 2.5)
  let want: Float = ClampF(target / 2.72, 0.03, 1.3);
  if AbsF(want - 1.0) < 0.001 {
    UTPPRemoveWalkSpeedMod(player, stats, id);
    return;
  }
  let applied: Float = 1.0;
  if IsDefined(player.utppWalkSpeedMod) {
    applied = player.utppWalkSpeedMul;
    if AbsF(want - applied) <= 0.02 {
      return;
    }
  }
  UTPPRemoveWalkSpeedMod(player, stats, id);
  player.utppWalkSpeedMod = RPGManager.CreateStatModifier(gamedataStatType.MaxSpeed, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppWalkSpeedMod);
  player.utppWalkSpeedMul = want;
}

// ---- Minimum analog walk speed (CET mm.minWalkSpeed -> UTPPSetMotionMinAnalogWalk) ----
// A small stick tilt moves the capsule slower than any walk clip (|mv| 0.09 -> 0.24 m/s, 0.27 -> 0.42 m/s), so V glided. UTPP adds one
// Multiplier on MaxSpeed, owned by the fields below: multiplier = UTPPMotionAnalogFloorMul (1 .. 4, up to 10 under the walk toggle; the fraction of the full-stick speed at
// the stick lifted to the floor's fraction, 0.82 m/s = the |mv| 0.455 point; exactly 1 at and above it). Only in third person, on the ground,
// locomotion state 'stand' / 'crouch' (forceWalk counts as stand, like the motion feed) with LocomotionDetailed Stand 1 / Crouch 3 (AimWalk 2,
// Sprint 4 and every other state: vanilla), and while motion drives (the native answers 1 otherwise). Unlike the multipliers above it is removed
// at once when the stick is released (|mv|^2 <= 0.0004): the statistic lags one frame, a kept x3.4 would reach the first frame of a
// full-stick start. Its own gate does not use the 'moving' test (|mv| > 0.15) of the other updates: the band below 0.15 is what it is for.
// Re-applied only when the multiplier changes by > 0.02. The Shooter mode and walk toggle multipliers multiply with this one on purpose.
@addField(PlayerPuppet)
public let utppAnalogFloorMod: ref<gameStatModifierData>;

@addField(PlayerPuppet)
public let utppAnalogFloorMul: Float;

public func UTPPRemoveAnalogFloorMod(player: ref<PlayerPuppet>, stats: ref<StatsSystem>, id: StatsObjectID) -> Void {
  if IsDefined(player.utppAnalogFloorMod) {
    stats.RemoveModifier(id, player.utppAnalogFloorMod);
    player.utppAnalogFloorMod = null;
  }
  player.utppAnalogFloorMul = 1.0;
}

// The floor multiplier of this frame without the third-person test (1 .. 4, up to 10 under the walk toggle; exactly 1.0 = no floor): released stick (|mv|^2 <= 0.0004),
// a state other than stand / crouch, LocomotionDetailed other than Stand 1 / Crouch 3, air, floor off or motion not driving -> 1.0.
public func UTPPAnalogFloorWant(scriptInterface: ref<StateGameScriptInterface>, stateName: CName, move: Vector2, walkToggled: Bool) -> Float {
  if move.X * move.X + move.Y * move.Y <= 0.0004 {
    return 1.0;
  }
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  let detailed: Int32 = 0;
  if IsDefined(bb) {
    let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    detailed = bb.GetInt(psm.LocomotionDetailed);
  }
  let state: Int32 = UTPPLocoStateId(stateName);
  // UTPPLocoStateId: stand / forceWalk 1, crouch 2 (sprint 3 and the rest: vanilla); gamePSMDetailedLocomotionStates.Stand 1, Crouch 3
  if !((state == 1 || state == 2) && (detailed == 1 || detailed == 3) && scriptInterface.IsOnGround()) {
    return 1.0;
  }
  // walk toggle (G): the same m/s against the walk speed, not the stand run (0.34 m/s instead of 0.82 at walk speed 1.6)
  return ClampF(UTPPMotionAnalogFloorMul(move.X, move.Y, detailed, walkToggled && Equals(stateName, n"stand")), 1.0, 10.0);
}

// UTPPUpdateShooterModeSpeed: the floor lifts this small tilt (third person is checked by the caller).
public func UTPPAnalogFloorWouldApply(scriptInterface: ref<StateGameScriptInterface>, stateName: CName, move: Vector2, walkToggled: Bool) -> Bool {
  return UTPPAnalogFloorWant(scriptInterface, stateName, move, walkToggled) >= 1.001;
}

public func UTPPUpdateAnalogFloor(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>, stateName: CName, move: Vector2, tpp: Bool, walkToggled: Bool) -> Void {
  let stats: ref<StatsSystem> = scriptInterface.GetStatsSystem();
  if !IsDefined(stats) {
    return;
  }
  let id: StatsObjectID = Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID);
  // first person: vanilla speed at once (the released stick and the other gates: UTPPAnalogFloorWant answers 1.0)
  if !tpp {
    UTPPRemoveAnalogFloorMod(player, stats, id);
    return;
  }
  let want: Float = UTPPAnalogFloorWant(scriptInterface, stateName, move, walkToggled);
  if AbsF(want - 1.0) < 0.001 {
    UTPPRemoveAnalogFloorMod(player, stats, id);
    return;
  }
  if IsDefined(player.utppAnalogFloorMod) && AbsF(want - player.utppAnalogFloorMul) <= 0.02 {
    return;
  }
  UTPPRemoveAnalogFloorMod(player, stats, id);
  player.utppAnalogFloorMod = RPGManager.CreateStatModifier(gamedataStatType.MaxSpeed, gameStatModifierType.Multiplier, want);
  stats.AddModifier(id, player.utppAnalogFloorMod);
  player.utppAnalogFloorMul = want;
}

// GameplayRestriction.Tier2LocomotionSlow (PSM HighLevel SceneTier2, highLevelTransitions.script SetTier2LocomotionSlow; the status
// effect adds -1.3 m/s to MaxSpeed). The status effect is only looked up in SceneTier2 (HighLevel 2), the cheap blackboard value first.
public func UTPPIsTier2Slow(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Bool {
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  if !IsDefined(bb) {
    return false;
  }
  // gamePSMHighLevel.SceneTier2 = 2
  if bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.HighLevel) != 2 {
    return false;
  }
  return StatusEffectSystem.ObjectHasStatusEffect(player, t"GameplayRestriction.Tier2LocomotionSlow");
}

public func UTPPFeedFromLocomotion(stateName: CName, timeDelta: Float, scriptInterface: ref<StateGameScriptInterface>, walkToggled: Bool) -> Void {
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if !IsDefined(player) {
    return;
  }
  if !UTPPMMWantsFeed() {
    // MM off / no database: make sure the acceleration is vanilla.
    if IsDefined(player.utppAccelMod) {
      UTPPRemoveAccelMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    if IsDefined(player.utppDecelMod) {
      UTPPRemoveDecelMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    if IsDefined(player.utppAimSpeedMod) {
      UTPPRemoveAimSpeedMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    if IsDefined(player.utppShooterModeSpeedMod) {
      UTPPRemoveShooterModeSpeedMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    if IsDefined(player.utppWalkSpeedMod) {
      UTPPRemoveWalkSpeedMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    if IsDefined(player.utppAnalogFloorMod) {
      UTPPRemoveAnalogFloorMod(player, scriptInterface.GetStatsSystem(), Cast<StatsObjectID>(scriptInterface.executionOwnerEntityID));
    }
    return;
  }
  let velocity: Vector4 = scriptInterface.GetOwnerStateVectorParameterVector(physicsStateValue.LinearVelocity);
  let move: Vector2 = Vector2(scriptInterface.GetActionValue(n"MoveX"), scriptInterface.GetActionValue(n"MoveY"));
  let cameraYaw: Float = UTPPYawDegFromForward(Transform.GetForward(scriptInterface.GetCameraWorldTransform()));
  let playerYaw: Float = UTPPYawDegFromForward(player.GetWorldForward());
  let targetYaw: Float = 0.0;
  let moving: Bool = move.X * move.X + move.Y * move.Y > 0.0225;
  let game: GameInstance = player.GetGame();
  let tpp: Bool = UTPPIsTppActive(game);
  let haveTarget: Bool = false;
  // target facing (the stick test is inside: UTPP Locomotion rewrites what the scripts see)
  haveTarget = tpp && UTPPOwnMoveTarget(cameraYaw, playerYaw, move, targetYaw);
  UTPPFeedLocomotionExtra(walkToggled, player.GetWorldPosition());
  UTPPFeedAirFromLocomotion(player, scriptInterface); // AIR HOLD stage: right before the feed
  let tier2Slow: Bool = UTPPIsTier2Slow(player, scriptInterface);
  UTPPFeedMotionTier2Slow(tier2Slow); // right before the feed, like the air fields
  UTPPFeedLocomotion(velocity, move, cameraYaw, playerYaw, UTPPLocoStateId(stateName), tpp, timeDelta, haveTarget, targetYaw, UTPPPsmBlockMask(scriptInterface));
  UTPPUpdateAccel(player, scriptInterface);
  UTPPUpdateDecel(player, scriptInterface);
  UTPPUpdateAimSpeed(player, scriptInterface, move, moving); // jog while aiming
  UTPPUpdateShooterModeSpeed(player, scriptInterface, move, moving, tpp, stateName, walkToggled); // Shooter mode: motion set side / back speeds (1.0 = nothing while off)
  UTPPUpdateWalkSpeed(player, scriptInterface, stateName, walkToggled, moving, tpp, tier2Slow); // walk toggle speed (0 = vanilla)
  UTPPUpdateAnalogFloor(player, scriptInterface, stateName, move, tpp, walkToggled); // minimum analog walk speed (no 'moving' gate; 1.0 = nothing while off)
}

// AIR HOLD stage: the air fields of this frame's feed. PSM.Fall is read here, before LocomotionAirEvents writes it
// (after super.OnUpdate): the previous frame's value. The raycast runs only while airborne and UTPPMMAirWantsProbe().
public func UTPPFeedAirFromLocomotion(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let onGround: Bool = scriptInterface.IsOnGround();
  let fall: Int32 = 0;
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  if IsDefined(bb) {
    fall = bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.Fall);
  }
  let dist: Float = -2.0;
  if !onGround && UTPPMMAirWantsProbe() {
    dist = UTPPAirGroundDistance(player);
  }
  UTPPFeedAir(onGround, dist, fall);
}

// Downward raycasts (20 m) from 0.1 m above the feet: Static + Terrain, and the 'Vehicle Chassis' collision preset over the same
// segment (car hoods and roofs are not in the static groups). Returns the nearer distance from the feet to a hit (>= 0),
// -1 = no hit, -2 = no spatial query system. A vehicle hit at the ray start (initial overlap) is ignored.
public func UTPPAirGroundDistance(player: ref<PlayerPuppet>) -> Float {
  let sqs: ref<SpatialQueriesSystem> = GameInstance.GetSpatialQueriesSystem(player.GetGame());
  if !IsDefined(sqs) {
    return -2.0;
  }
  let pos: Vector4 = player.GetWorldPosition();
  let start: Vector4 = Vector4(pos.X, pos.Y, pos.Z + 0.1, 1.0);
  let end: Vector4 = Vector4(pos.X, pos.Y, pos.Z - 20.0, 1.0);
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  let dist: Float = -1.0;
  let hit: TraceResult;
  if sqs.SyncRaycastByQueryFilter(start, end, filter, hit) {
    dist = MaxF(pos.Z - hit.position.Z, 0.0);
  }
  let vehHit: TraceResult;
  if sqs.SyncRaycastByCollisionPreset(start, end, n"Vehicle Chassis", vehHit) && TraceResult.IsValid(vehHit) {
    if vehHit.position.Z < start.Z - 0.01 {
      let vehDist: Float = MaxF(pos.Z - vehHit.position.Z, 0.0);
      if dist < 0.0 || vehDist < dist {
        dist = vehDist;
      }
    }
  }
  return dist;
}

// Vanilla 2.31: public func OnUpdate(...) in LocomotionEventsTransition; redscript chains all wraps of the same method.
// Subclasses (StandEvents, CrouchEvents, SprintEvents...)
// reach it through super.OnUpdate.
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(timeDelta, stateContext, scriptInterface);
  // PSM permanent bool WalkToggled (vanilla locomotionTransitions.script: the walk toggle; cleared by sprint, combat,
  // swim, AI control, death and scenes) -> UTPPFeedLocomotionExtra
  UTPPFeedFromLocomotion(this.GetStateName(), timeDelta, scriptInterface, stateContext.GetBoolParameter(n"WalkToggled", true));
}
// ClimbEvents overrides OnUpdate without calling super.OnUpdate (vanilla locomotionTransitions.script l.2813), so
// the wrap above never ran during a climb: the MM feed went stale and Tick held the pre-climb pose for the whole climb. Feed
// the climb state too, so LocomotionDetailed 8 reaches the gate (mm.climbRelease -> the game's climb animation, or the motion
// traversal montage when it owns the climb). VaultEvents has no OnUpdate override and is fed by the wrap above.
@wrapMethod(ClimbEvents)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(timeDelta, stateContext, scriptInterface);
  UTPPFeedFromLocomotion(this.GetStateName(), timeDelta, scriptInterface, stateContext.GetBoolParameter(n"WalkToggled", true));
}
// Diagnostic: log every MM clip transition and a 1 Hz summary to the UTPP RED4ext log.
public native func UTPPSetMMTrace(on: Bool) -> Void
// Speed map game -> database (defaults 1.6, 3.85, 2.19, 5.46, 0.6, 1.4); matcher time scaled by |v|/|v_db|.
public native func UTPPSetMMSpeedMap(gameWalk: Float, gameRun: Float, dbWalk: Float, dbRun: Float, rateMin: Float, rateMax: Float) -> Void
// Per-state speed point (loco 1 stand, 2 crouch, 3 sprint; index 0..2): game m/s -> database m/s.
public native func UTPPSetMMSpeedPoint(loco: Int32, index: Int32, gameSpeed: Float, dbSpeed: Float) -> Void
// Input-driven trajectory (stick direction relative to the camera), default true.
public native func UTPPSetMMInputDriven(on: Bool) -> Void
// Bias towards continuing the playing clip (normalized cost units), default 0.15.
public native func UTPPSetMMTransitionCost(cost: Float) -> Void
// Input frame: false (default, body-relative) or true (camera-relative stick).
public native func UTPPSetMMInputCameraRelative(on: Bool) -> Void
// contact-based foot lock with two-bone leg IK (default true); false blends locked feet out.
public native func UTPPSetMMFootLock(on: Bool) -> Void
// ---- body yaw follows the motion matching animation ----
// Kill switch, default true. false = fixed-time body turn, no animation-driven yaw.
public native func UTPPSetMMBodyTurn(on: Bool) -> Void
// Fallback convergence when the clip does not turn enough: time constant (s, default 0.4), rate floor (deg/s, 20),
// fallback rate cap (deg/s, 180) and total rate cap (deg/s, 900). False = invalid, previous kept.
public native func UTPPSetMMBodyTurnParams(correctionTime: Float, minRateDeg: Float, maxCorrectionRateDeg: Float, maxRateDeg: Float) -> Bool
// True while the body-yaw command replaces the base command (kill switch on, MM weight > 0.5, gate open, Locomotion target, fresh feed).
public native func UTPPMMBodyTurnActive() -> Bool
// Next body yaw (deg) from the current yaw: animation root yaw rate + bounded convergence to the target; returns
// baseYawDeg unchanged when not active. Called by the CET mod utpp_bodyturn with the yaw the base turn would command.
public native func UTPPMMBodyYawCommand(currentYawDeg: Float, baseYawDeg: Float) -> Float
// ---- capsule / animation sync ----
// Offset root (motion set style): the visual body follows the animation root motion and springs back to the capsule. Default on.
public native func UTPPSetMMOffsetRoot(on: Bool) -> Void
// Translation halflife (s, default 0.25), rotation halflife (s, 0.20), max distance (m, 0.25), max yaw (deg, 15).
public native func UTPPSetMMOffsetRootParams(translationHalflife: Float, rotationHalflife: Float, maxDistance: Float, maxYawDeg: Float) -> Bool
// Capsule acceleration matching (lowers the Acceleration stat while MM drives the body). Default on.
public native func UTPPSetMMAccelMatch(on: Bool) -> Void
// Target accelerations (m/s^2) for stand, crouch, sprint (defaults 3.75, 12.2, 6.96; 0 = no change for that state).
public native func UTPPSetMMAccelParams(standAccel: Float, crouchAccel: Float, sprintAccel: Float) -> Bool
// Internal (UTPPUpdateAccel): reports the vanilla stat and the applied multiplier, returns the target (0 = remove).
public native func UTPPMMAccelTarget(baseAccel: Float, appliedMul: Float) -> Float
// ---- stops, slow walk, start reface (all default on; false switches them off) ----
// Playback rate from the played clip's root speed vs the capsule, only during the stop phase (stick released, 0.6 s).
public native func UTPPSetMMRootRate(on: Bool) -> Void
// Root rate: min (0.5), max (1.5), epsilon m/s (0.5), smoothing halflife s (0.08).
public native func UTPPSetMMRootRateParams(rateMin: Float, rateMax: Float, epsilon: Float, halflife: Float) -> Bool
// Stop sync: zero desired velocity when the stick is released + trajectory stop halflife (s, default 0.05).
public native func UTPPSetMMStopSync(on: Bool, stopHalflife: Float) -> Bool
// Slow walk: below 1.6 m/s (crouch 2.7) the query asks for the walk/crouch loop speed, rate floor walkRateMin (0.25).
public native func UTPPSetMMSlowWalk(on: Bool, walkRateMin: Float) -> Bool
// Offset root saturation guard: after `seconds` at the 25 cm clamp the halflife becomes `halflife` (0.3 s / 0.1 s; 0 = off).
public native func UTPPSetMMOffsetSaturation(seconds: Float, halflife: Float) -> Bool
// Start reface: starting towards a target behind the body asks for a turn / reface start (Locomotion path).
public native func UTPPSetMMStartReface(on: Bool) -> Void
// Exclude landing / from-traversal clip sets from ground locomotion (stand/crouch/sprint).
public native func UTPPSetMMExcludeLands(on: Bool) -> Void
// ---- steering stability (all default on; false switches them off) ----
// Body-turn hysteresis: animation-led turns only for sharp direction changes (> enter deg), otherwise the body follows the
// stick with a rate-limited spring and the matcher predicts a curved path.
public native func UTPPSetMMSteerFollow(on: Bool) -> Void
// enter deg (100), exit deg (30), follow time s (0.15), max follow rate deg/s (360).
public native func UTPPSetMMSteerParams(enterDeg: Float, exitDeg: Float, followTime: Float, maxFollowRateDeg: Float) -> Bool
// Minimum clip-segment dwell (matcher s, 0.3) unless the cost drops below costRatio (0.5) x the continuing cost.
public native func UTPPSetMMDwell(on: Bool, minDwell: Float, costRatio: Float) -> Bool
// Clip curation: no Box/Diamond/Hourglass/Prism patterns; pivots / moving turns / reface / turn in place by situation.
public native func UTPPSetMMCuration(on: Bool) -> Void
// Slow walk decided from the real capsule speed (pad analog walk), requires UTPPSetMMSlowWalk on.
public native func UTPPSetMMSlowWalkCapsule(on: Bool) -> Void
// ---- medium direction changes (default on; false / 0 switches them off) ----
// No stop clips while moving with the stick held, no start / speed-transition clips mid-run, no turn clips when stopping.
public native func UTPPSetMMCurationMoving(on: Bool) -> Void
// Largest facing gap (deg) the follow-mode query asks for (default 8; 0 = whole target gap).
public native func UTPPSetMMSteerGap(gapDeg: Float) -> Bool
// Stance-width guard: each ankle at least minHalfWidthCm (default 5) from the pelvis midline (no leg crossing).
public native func UTPPSetMMStanceGuard(on: Bool, minHalfWidthCm: Float) -> Bool
// ---- travel alignment (default on; false switches it off) ----
// Animation root velocity turned towards the capsule velocity for the offset root (no lateral "floating"); max angle deg (60).
public native func UTPPSetMMRootWarp(on: Bool, maxDeg: Float) -> Bool
// Visual body leans (offset yaw, up to 15 deg) towards the travel heading while the body turns.
public native func UTPPSetMMHeadingLean(on: Bool) -> Void
// Stance guard on the swing foot only (push held while the foot is planted).
public native func UTPPSetMMStanceSwing(on: Bool) -> Void
// One diagnostic line into the UTPP red4ext log (used by the UTPP CET mods; CET prints do not reach a file).
public native func UTPPLogLine(line: String) -> Void
// Large turns (default on; false = off): speed-class pivots, pivot capsule acceleration, stop prediction fixes.
public native func UTPPSetMMLargeTurn(on: Bool) -> Void
// Capsule acceleration during an animation-led large turn (m/s^2; defaults 8 / 8 / 12; 0 = no limit).
public native func UTPPSetMMPivotAccel(standAccel: Float, crouchAccel: Float, sprintAccel: Float) -> Bool
// Stance guard geometry: ankle half width cm (default 3), swing-ankle separation cm (default 6), knee-keeping IK (default true).
public native func UTPPSetMMStanceParams(minHalfWidthCm: Float, minSeparationCm: Float, kneeIk: Bool) -> Bool
// ---- anti-jitter, slow-walk loop (default on; false / 0 switches them off) ----
// No start clips (Walk_Start) kept in a steady slow walk after the start window.
public native func UTPPSetMMSlowWalkLoop(on: Bool) -> Void
// Stance guard anti-jitter filter time (s, default 0.08; 0 = off: knee IK, rate-limited push, unfiltered swivel).
public native func UTPPSetMMStanceSmooth(smoothTime: Float) -> Bool
// ---- runtime motion matching database switching ----
// <dll dir>/data/utpp_base.utppmm = "default", <dll dir>/data/<name>.utppmm = <name> (the per-body utpp_motion_* files are not
// listed). List: "default;name1;...".
public native func UTPPListDatabases() -> String
// Loads on a worker thread (full validation, same rig / bone set), then MM fades out, swaps and fades back in.
// False = invalid / missing name or a load already running; failures keep the active database.
public native func UTPPLoadDatabase(name: String) -> Bool
public native func UTPPGetDatabaseName() -> String
// "idle" | "loading <name>" | "pending <name>" | "ok <name>" | "FAILED <name>: reason"
public native func UTPPGetDatabaseSwitchStatus() -> String
// Per-database speed calibration is automatic (forward loop clips, optional <database>.json sidecar; status speedCal=).
// Stand <-> crouch transition clips only within this window after a real stance change (s, default 0.6; < 0 = no window).
public native func UTPPSetMMStanceTransitionWindow(seconds: Float) -> Bool
// ---- locomotion fixes (default on; false switches them off) ----
// Stick filter: half-stick magnitudes count as held, releases < 0.12 s bridged (no stop clip in a 180 deg flick).
public native func UTPPSetMMStickFilter(on: Bool) -> Void
// Per-clip playback-rate factor by source family / own loop speed + slow-walk query = slowest forward walk loop.
public native func UTPPSetMMFamilyRate(on: Bool) -> Void
// No pivots / moving turns from standstill, no stop clips while the stick is held, no sprint starts unless sprinting.
public native func UTPPSetMMStartRules(on: Bool) -> Void
// Speed bands: pivots / moving turns only of the motion's speed class (walk / run / sprint, per-database edges with
// hysteresis), no idle clips while the stick is held and the capsule moves. Status: speedBands= band= bandEdges= sprintData=.
public native func UTPPSetMMSpeedBands(on: Bool) -> Void
// ---- gate debounce (default on; off = no debounce) ----
// Gate debounce: a closure only from the locomotion state / PSM detailed locomotion (no vertical motion) must last this
// long before MM fades out (s, default 0.06; 0 = off; false = invalid > 0.5). Vehicle, scene, workspot, swim,
// takedown, carry, death, menu, stagger, jump / fall (|vz| >= 1 m/s), TPP off close at once. Status: gateDebounce=
// rawGate= debounced= debounceClosed= psmDetail= softDetail=; MMGATE trace: detail= (PSM.LocomotionDetailed).
public native func UTPPSetMMGateDebounce(seconds: Float) -> Bool
// Crouch rules: stand <-> crouch transitions only in the direction of the current stance and not above 6 m/s (slide
// entry); in crouch, sideways loops (Left / Right / SideStep / Strafe in the name) not for forward motion. Status: crouchRules=
// lateralOk= lateralExcl=.
public native func UTPPSetMMCrouchRules(on: Bool) -> Void
// Crouch direction tag: crouch loops / moves tagged L / R / B / BL / BR / LL / LR / RL / RR count as sideways (excluded for
// forward crouch motion where a forward loop exists; with the crouch rules). Status: crouchDirTag= lateralExcl=.
public native func UTPPSetMMCrouchDirTag(on: Bool) -> Void
// Stance transitions once per stance change and only from their first 6 frames (the frame window follows the switch at the
// next UTPPSetMMEnabled(true) / database load). Status: stanceOnce= stanceUsed=.
public native func UTPPSetMMStanceOnce(on: Bool) -> Void
// Turn commit: no other moving turn / pivot / reface clip while the playing one still turns towards the target. Status:
// turnCommit= committed= commitHolds=.
public native func UTPPSetMMTurnCommit(on: Bool) -> Void
// Standstill idle: no stick (or MM gated out) and capsule < 0.2 m/s -> idle / stop clips only. Status: standIdle=.
public native func UTPPSetMMStandIdle(on: Bool) -> Void
// Start turn: turn direction consistent with the body, no slow turning starts, on-the-spot turn segments at rate >= 1.
// Status: startTurn= classSet= startRate=.
public native func UTPPSetMMStartTurn(on: Bool) -> Void
// Speed cap: capsule faster than factor x game sprint speed (default 1.25 x 8.2 = 10.25 m/s; dash / dodge bursts) fades MM
// out (gate bit 0x200, no debounce) and the matcher query never exceeds the sprint speed. 0 = off; false = invalid.
// Status: speedCap= speedCapMs= overSpeed= overSpeedFeeds= queryClamped=.
public native func UTPPSetMMSpeedCap(factor: Float) -> Bool
// Cost cap: best candidate above the cap (default 100; normal picks < 40) -> the playing clip continues (<= 0.3 s), else a
// retry over the ordinary candidate set. 0 = off. Status: costCap= costCapHits= costCapHolds= costCapRetries= costCapForced=;
// trace: MMTRACE costcap.
public native func UTPPSetMMCostCap(cost: Float) -> Bool
// Stance cap: turn commit only for a playing clip still in the clip set; no stance transition above the cost cap.
// Status: stanceCap= commitNotAllowed= costCapNever=.
public native func UTPPSetMMStanceCap(on: Bool) -> Void
// Turn settle: no same-direction turn restart for 0.35 s after a committed turn reached 90 % of its root turn.
// Status: turnSettle= settling= settleHolds=.
public native func UTPPSetMMTurnSettle(on: Bool) -> Void
// Loop bands: loops / speed transitions only of the speed band of the query speed. Status: loopBands= loopBand= loopAvail=.
public native func UTPPSetMMLoopBands(on: Bool) -> Void
// Start bind: body turn bound to the playing start / reface clip's root-yaw sign, same-direction candidates for 0.5 s unless
// the stick moves > 90 deg. Status: startBind= bindSign= binds= bindReversals= stSign=; trace: stSign=clip|fallback|off.
public native func UTPPSetMMStartBind(on: Bool) -> Void

// Capsule sync: acceleration targets from the database's start clips, stop deceleration from its stop clips (measured at load).
// Status: capsuleSync= capAccel= capDecel= decelTarget= decelBase= decelMul= decelActive= capProfile=; trace: decel=t/base x mul.
public native func UTPPSetMMCapsuleSync(on: Bool) -> Void
// database-switch repair (default true; false = off): a database switch (UTPPLoadDatabase) clears the playback state of the old
// database's clips (turn commit, RootRate window, StanceGuard knee filter, chest align, cost-cap hold). Status: swapReset=
// swapResets= swapCarry= swapPost=.
public native func UTPPSetMMSwapReset(on: Bool) -> Void
// Offset catch-up: soft limit + time-bounded convergence of the visual body to the capsule, foot soft lock (gradual release).
// Status: offCatchup= catchCfg= offSoftS= offSatS= offFar= offFarMax= footSlip= footSlips=; trace: far=.
public native func UTPPSetMMOffsetCatchup(on: Bool) -> Void
// maxDistance m (0.30), catchupTime s (0.6), softDistance m (0.15), catchupHalflife s (0.06).
public native func UTPPSetMMOffsetCatchupParams(maxDistance: Float, catchupTime: Float, softDistance: Float, catchupHalflife: Float) -> Bool
// Foot soft lock: slip start m (0.05), slip half-life s (0.02).
public native func UTPPSetMMFootSlipParams(slipStart: Float, slipHalflife: Float) -> Bool
// Trajectory calibration: velocity spring half-life per state from the capsule profile. Status: trajCal= trajHl=; trace: trajHl=.
public native func UTPPSetMMTrajCal(on: Bool) -> Void
// Internal (UTPPUpdateDecel): reports the vanilla Deceleration stat and the applied multiplier, returns the target (0 = remove).
public native func UTPPMMDecelTarget(baseDecel: Float, appliedMul: Float) -> Float

// ---- melee, vehicle and mount gate signals (default on; false / 0 switches them off) ----
// Melee gate: whole-body melee attacks / poses (UTPPPsmBlockMask bits 11 / 12) close the MM gate (debounced like a crouch blip,
// normal fade) and keep it closed 0.3 s after the melee state ends. Status: meleeGate= meleeBlocks= meleeState= meleeActive=
// meleeBits= upperBody=; MMGATE trace: melee= meleeState= meleeHold= upperBody=.
public native func UTPPSetMMMeleeGate(on: Bool) -> Void
// Mount cut: the vehicle bit (and PlayerPuppet.OnMountingEvent below) cuts the MM weight to 0 at once (no 0.12 s fade with the
// standing pose over the bike) and keeps it 0 while mounted. Status: mountCut= mountCuts= mountEventCuts= mountMuted=.
public native func UTPPSetMMMountCut(on: Bool) -> Void
// Internal: called from the mounting event wrapper (no-op with the mount cut off).
public native func UTPPMMMountEvent() -> Void
// Same with the vehicle class: isCar = CarObject; the mount transition blends a large root yaw of a car entry only.
public native func UTPPMMMountEventKind(isCar: Bool) -> Void
// Stance clip minimum speed: a moving into-crouch transition (RunToCrouch / WalkToCrouch) only when the capsule speed at the
// stand -> crouch change is >= mps and the stick >= 0.7 forward; otherwise straight into the crouch loop / start (default 4.0,
// 0 = off). Status: stanceClipMin= stanceClipSkips= stanceClipAllows= stanceClipSpeed= stanceClipStick=.
public native func UTPPSetMMStanceClipMinSpeed(mps: Float) -> Bool
// upper-body bone mask (default on; off = the whole body is driven): for the masked states the gate keeps MM on Hips, Spine, legs and feet
// (normal stand / walk query) and the game drives Spine1..3, neck, head, clavicles, arms, hands, fingers, WeaponLeft /
// WeaponRight; the upper-body bones fade over 0.15 s. Status: upperMask= maskBones= maskActive= maskSwitches=; trace: mask=.
public native func UTPPSetMMUpperMask(on: Bool) -> Void
// Masked states (bits): 1 melee gate (attack / block / targeting + 0.3 s release), 2 PSM.UpperBody Aim, 4 vehicle exit window
// (end of the PSM vehicle state until MM is fully back, max 2 s, no reopen hold). Default 7; false = invalid (< 0 or > 7).
public native func UTPPSetMMUpperMaskStates(bits: Int32) -> Bool
// preserve the game's aim mask through a 0.25 s exit, including motion weapon overlays. Default on; off = legacy.
public native func UTPPSetMMAimUpperFade(on: Bool) -> Void
// Live WeaponRight held: retain the game's upper body over owned MM legs, including AIR. Default on; off = legacy.
public native func UTPPSetMMWeaponUpper(on: Bool) -> Void
public native func UTPPSetMotionGrip(on: Bool) -> Void
public native func UTPPSetMotionShootTransfer(on: Bool) -> Void
// Magazine top-up gesture during the Shinobi bonus (CET mm.shinobiBonusReload, default off; the DLL default is off too): while off
// UTPPFeedMotionMagTap (utpp_motion.reds) is refused at once. Keep this declaration and the DLL registration (Main.cpp) in sync.
public native func UTPPSetMotionMagTap(on: Bool) -> Void
public native func UTPPSetMotionAimGameUpper(on: Bool) -> Void
// hold last pose (default on; off = no hold): in TPP the game's full body is a T-pose outside the states it
// animates, so MM keeps the body for every closure except vehicle, scene, workspot, swim, ladder, takedown, body carry, death,
// knockdown and TPP off (MM disabled, database switch, blips, unsupported locomotion, menu / stale feed, over-speed... are held).
// Status: holdPose= holdPoseActive= holdPoseFrames=; trace: hold=.
public native func UTPPSetMMHoldPose(on: Bool) -> Void
// aim offset (native post-process on the final pose, before FK): while the upper-body mask holds the game's FPP aim pose
// in TPP, the right upper arm (clamped yaw / pitch, smoothed) and the right wrist (residual <= 5 deg) are rotated so the
// WeaponRight barrel (local +Y by default) points at P = camPos + norm(camFwd) * 60 m; a two-handed left hand follows the weapon
// (two-bone IK). Guards (all required): target set within 0.25 s, fresh feed with TPP, PSM.UpperBody Aim, mask state bit 2,
// upper-body mask active, no melee bits (+ 0.3 s), no vehicle / other PSM block, rig bones resolved by name (fail closed).
// CET (utpp_camera, wiring pending): call every frame while the aim state is on with the ACTIVE camera world position and
// forward, and once with enabled = false when it ends; the target is converted to the player's model space natively.
public native func UTPPSetAimTarget(camPos: Vector4, camFwd: Vector4, enabled: Bool) -> Void
// Active TPP camera ray for owned weapon overlays, including hipfire. Does not enable the native FPP aim-mask solver.
public native func UTPPSetCameraTarget(camPos: Vector4, camFwd: Vector4, enabled: Bool) -> Void
// maxYawDeg / maxPitchDeg: shoulder clamp, 0..60 (default 30 / 25; both 0 = aim offset off). smoothHz: exponential rate 1/s,
// 0..120 (default 12 = time constant 0.083 s; 0 = no smoothing). False = invalid, previous kept.
public native func UTPPSetAimOffsetParams(maxYawDeg: Float, maxPitchDeg: Float, smoothHz: Float) -> Bool
// Barrel axis of WeaponRight in its local frame: 1 = X, 2 = Y (default, rig bind-pose assumption), 3 = Z, negative = opposite.
public native func UTPPSetAimOffsetAxis(axis: Int32) -> Bool
// "aimofs ... guard=0x.. applied= err=yaw/pitch cmd=yaw/pitch res= errBefore= errAfter= w= twoHand= ik= lhErr=" (degrees, m).
// guard bits: 1 no target, 2 target stale, 4 target invalid, 8 feed stale, 16 TPP off, 32 not UpperBody Aim, 64 mask state off,
// 128 melee, 256 vehicle, 512 other PSM block, 1024 mask inactive, 2048 rig, 4096 busy, 8192 solve failed, 16384 disabled.
public native func UTPPAimOffsetStatus() -> String
// turn rate (default on): raise the plant of a moving turn / pivot / reface towards the clip's own timing.
// Default min / max 0.9 / 1.4; false = invalid (NaN, min < 0.5, max > 2, min > max), previous kept. When the raise of a turn
// ends, its summary goes to the status (UTPPMMTurnRateStatus, last=) and, only while the motion matching trace is on
// (mm.trace; part of the diagnostic logs), as one log line: "turnRate: clip= avg= peak= dur= capsule= root=".
public native func UTPPSetMMTurnRate(on: Bool) -> Void
public native func UTPPSetMMTurnRateParams(min: Float, max: Float) -> Bool
// "turnRate on= min= max= lastTarget= active= w= feeds= turns= last=\"<last turn summary>\"".
public native func UTPPMMTurnRateStatus() -> String
// item render plane (src/render/ItemPlane.cpp, utpp_itemplane.reds). The player's AttachmentSlots bind WeaponRight /
// WeaponLeft / PersonalLink / SilverhandArm to the animation parameter "renderPlane"; every change of it re-applies RPl_Weapon.
// Constructs gameItemEventsPropagateRenderingPlane, writes the plane (0 scene, 1 background, 2 weapon) and queues it on the item
// and its child items / parts. False = not an ItemObject / RTTI mismatch (logged).
public native func UTPPSetItemRenderPlane(item: ref<ItemObject>, plane: Int32) -> Bool
// TPP override: while on, RPl_Weapon written by the animation parameter broadcast becomes RPl_Scene. Returns whether the native
// hook is installed (false = fix inert, prologue mismatch / attach failure, see UTPPItemPlaneNativeStatus).
// a change of the value also re-applies the native registry of plane-bound components (game thread): force ->
// RPl_Scene, vanilla -> exactly the plane vanilla requested. Decided only by UTPPItemPlaneSystem.SyncPolicy.
public native func UTPPSetItemPlaneOverride(on: Bool) -> Bool
// re-applies the current policy to every live registered component; returns how many were written.
public native func UTPPItemPlaneReapply() -> Int32
// header line with the counters, then one line per registered component (up to max): component class, name,
// requested plane, current plane, owner is the player (0/1). Also written to the UTPP log.
public native func UTPPItemPlaneDump(max: Int32) -> String
public native func UTPPItemPlaneDumpPlayer(max: Int32) -> String
// "itemplane hook= override= broadcastsOn= weaponInBroadcast= forcedScene= eventsScene= eventsWeapon= eventsOther= eventFails= install="
// (+ registry= reapplied= bgInBc= weaponOutBc=).
public native func UTPPItemPlaneNativeStatus() -> String
// game look write (src/input/GameLook.cpp). The game's FPP look (aim animation, shots, crosshair, Locomotion heading,
// AimOffset) is the animation graph's YawInput / PitchInput, fed by deltas from the FPP camera component each tick; while a
// target is set (expires 0.25 s after the last call) and the heading is locked, the hook replaces that delta so that the world
// look = worldYawDeg (deg, CCW positive, atan2(-x, y)) / pitchDeg (deg, up positive). True = the write is live; false =
// hook not installed, RTTI / component / entity transform check failed or the loop tripped (reason in UTPPGameLookStatus).
// entityYawAheadDeg = V's yaw turn requested for the coming tick (utpp_bodyturn), used once by the next write
// (optional: omitted = 0, the 2-argument form keeps working). 0 never clears a pending turn, turns add up until the
// next write; after a divergence trip the call fails for 3 s, then re-arms the write (at most 3 trips per game session).
public native func UTPPSetGameLook(worldYawDeg: Float, pitchDeg: Float, opt entityYawAheadDeg: Float) -> Bool
// Ends the write at once (the game's own look input applies again). True = it was active.
public native func UTPPStopGameLook() -> Bool
// "gamelook hook= active= tripped= tripCount= tripPermanent= tripOff= rearms= comp= target= sets= setFails= stops= calls= ...
// k= err= rawErr= maxAbsErr= ff= ovrTicks= releases= lastRelease= maxRelease= aheadSets= aheadStale= aheadUsed= ahead=
// aheadEff= aheadN= aheadMean= aheadNoFeed= aheadDiscN= aheadAfter= aheadBefore= aheadLate= aheadOrder= lagHint= entYaw= yawIn=
// pitchIn= yawLimits= desired= delta= feat= featRepl= featReplOvr= yawMult= pitchMult= resetTicks= reset= overrideTicks=
// override= maxAbsDelta= entYawCheck= last=\"<last SetTarget result>\" install=\"...\"".
public native func UTPPGameLookStatus() -> String
// derived class data (default on; false = built before the masks): data derived from the per-state tag masks (loop bands, crouch lateral rule)
// rebuilt right after the masks on a database load / switch (so that the walk loops of the startup database are banded). True = the active database at once; false = from the next load / switch.
// One info line per load /
// switch "UTPP MM class data [<db>]: ..."; status: derivedRecompute= derivedRecomputes= derivedStale=.
public native func UTPPSetMMDerivedRecompute(on: Bool) -> Void
// (default on; false = the plain trace / status text): feet diagnostics - with the MM trace on, one "UTPP MMTRACE feet"
// line after every MMTRACE TRANS / tick line (per foot: phase P/R/F, contact, lock offset, correction, slips, drift unlocks,
// steps) and one "UTPP MMTRACE footstep" line per foot event; status segment feet="...".
public native func UTPPSetMMFeetDiag(on: Bool) -> Void
// crouch idle curation (default on; false = the full crouch clip set): the crouch clip set leaves out the exact tags of the exclusion list
// (default: empty list, nothing excluded); kept in full when no crouch idle loop would remain. Info line "UTPP MM crouch idle exclusion [<db>] (...): ...".
public native func UTPPSetMMCrouchIdleExclude(on: Bool) -> Void
// Exact tag names, comma-separated (letters, digits, '_'; at most 16 names / 512 characters); "" = the default (empty) list.
// False = invalid, previous list kept.
public native func UTPPSetMMCrouchExcludeTags(tags: String) -> Bool
// Standstill idle (default on; false = off): once V has stood still for 0.25 s with no stick, a playing
// idle or a clip about to end is followed by the stance's idle family only - no re-entry into a stop clip's tail (a small pop
// about every second). CET setting mm.idleHold. Status: idleHold= (only while on).
public native func UTPPSetMMIdleHold(on: Bool) -> Void
// (default 2 = crouch; 0 = off): plant / replant per stance (mask bit 1 stand / sprint, bit 2 crouch;
// 0 off, 2 crouch, 3 all). A planted foot stays planted; beyond the unplant radius (7 cm, idle / stop clips 10 cm), after 0.1 s of
// clamped IK reach or (standing) a lasting offset > 4 cm it makes one short lifted step (<= 3 cm) to the animated toe, one foot at a
// time. CET setting mm.footReplant (off / crouch). Bit 1 (standing replant, masks 1 / 3) is experimental, console only: run starts
// and sharp turns stretch the legs beyond the 15 cm foot-correction limit. False = invalid mask.
public native func UTPPSetMMFootReplant(mask: Int32) -> Bool
// Radii (m, 0.02..0.15), relock ratios (0..0.95), step speeds (m/s, 0.2..10), lift (m, 0..0.1); defaults 0.07, 0.10, 0.2, 0.5, 1.0,
// 2.5, 0.03. False = invalid, previous kept.
public native func UTPPSetMMFootReplantParams(radius: Float, stopsRadius: Float, ratio: Float, stopsRatio: Float, stepSpeed: Float,
  stopsStepSpeed: Float, lift: Float) -> Bool
// knee fix (default 7; 0 = off): bit 1 = no stance-guard knee swivel while
// crouching (faded out), bit 2 = foot-lock IK knee pole carried from the animated hip-ankle axis (the right knee no longer swings
// sideways on standing turn starts), bit 4 = the stance-guard knee swivel follows its target at 0.3 of its rate while the foot is
// planted (standing turn starts). CET settings mm.kneeFix (off = 0) and mm.kneeHold (bit 4; off = 3). Status: kneeFix= (only while not 0). False = invalid mask (< 0, > 7).
public native func UTPPSetMMKneeFix(mask: Int32) -> Bool
// Opt-in head lead (default off; CET setting mm.headLead): while V runs a turn (heading rate above ~25 deg/s, speed above ~0.8 m/s, not
// aiming / reloading / airborne / crouching) Neck / Neck1 / Head turn INTO the turn by up to ~22 deg relative to Spine3 and the
// clip's own head-vs-chest bias is removed (motion set run clips point the head forward / against the turn). motion poses only. Status /
// MMTRACE: headLead= lead= corr=. Params: leadDeg = lead at 120 deg/s heading rate (0..45, default 14), maxDeg = clamp (0..45,
// default 22); false = NaN / out of range (nothing changed).
public native func UTPPSetMMHeadLead(on: Bool) -> Bool
public native func UTPPSetMMHeadLeadParams(leadDeg: Float, maxDeg: Float) -> Bool
// Feminine shoulder narrowing (motion poses, female body; CET setting mm.shoulderWidth -> 2.5 - width / 2, default 2.5): how far (cm, 0..5) each upper-arm
// socket moves toward the spine at full weight; hands and weapon contacts are kept exactly. 0 = off. False = NaN / out of range.
public native func UTPPSetMotionShoulderNarrow(cm: Float) -> Bool
// Run pelvis yaw trim (motion poses; CET setting mm.pelvisYawTrim, default 0): turns the pelvis by deg (+ = left, -10..10) while V
// walks / runs / sprints forward on the ground; chest, head and feet keep their direction. False = NaN / out of range (nothing changed).
public native func UTPPSetMotionPelvisYawTrim(deg: Float) -> Bool
// Body yaw offset (motion poses; CET setting mm.bodyYawOffset, default 0): turns the whole character by deg (+ = left, -20..20) about
// its visual root; the movement direction is unchanged; off while aiming / during weapon actions. False = NaN / out of range (nothing changed).
public native func UTPPSetMotionBodyYawOffset(deg: Float) -> Bool
// Barrel pitch loop of the motion aim offset (CET setting mm.aimAtCrosshair, DLL default off): the weapon
// barrel is steered onto the crosshair point (UTPPSetAimTarget / UTPPSetCameraTarget). Off: a running loop fades out. Always true.
// The barrel axis comes from UTPPSetAimOffsetAxis (shared with the aim offset corrector).
public native func UTPPSetMotionBarrelLoop(on: Bool) -> Bool
// Muzzle distance of the barrel loop (m from WeaponRight along the barrel axis), 0.05 .. 1.0, default 0.25. False = NaN / out of range.
public native func UTPPSetMotionBarrelMuzzle(len: Float) -> Bool
// Aim strafe (default on; CET setting bodyturn.aimStrafe; off = no aim strafe): while V aims with a weapon in hand the legs
// strafe (side / back / diagonal steps, facing-fixed motion set clips) and hips / feet keep facing the aim. Params (console): unarmed
// (CET bodyturn.aimStrafeUnarmed, default false), rootTurnMaxDeg 10 (1..90), rootWarpMaxDeg 30 (0..90), exitHold 0.15 s (0..1),
// jogF 0 (0 = walk gait; 3.3..6 = jog gait). False = invalid / NaN, previous kept. Status: strafe= ... in UTPPGetStatus().
public native func UTPPSetMMStrafe(on: Bool) -> Void
public native func UTPPSetMMStrafeParams(unarmed: Bool, rootTurnMaxDeg: Float, rootWarpMaxDeg: Float, exitHold: Float, jogF: Float) -> Bool
// (internal, UTPPUpdateAimSpeed): jog target capsule speed (m/s) for the body-relative stick = jogF x profile (F : side :
// back = 1 : 0.70 : 0.60); 0 = vanilla aim speed (jog off, strafe not Active, crouch, MM not driving). Status: aimJogT= while jogF > 0.
public native func UTPPMMAimJogTarget(moveX: Float, moveY: Float) -> Float
// Shooter mode (internal, UTPPUpdateShooterModeSpeed): MaxSpeed multiplier for the body-relative stick = MotionMaxSpeed(direction) /
// MotionMaxSpeed(forward) of the current gait (stand run 1 / 0.70 / 0.60, walk 1 / 0.90 / 0.75, crouch 1 / 0.89 / 0.80). Exactly 1.0 when
// Shooter mode is not live, the input is not finite, |move| < 0.15 or detailed is not Stand (1) / Crouch (3). Lock-free (one atomic
// load). Keep this declaration and the DLL native (Main.cpp) in sync.
public native func UTPPMotionShooterModeSpeedMul(moveX: Float, moveY: Float, detailed: Int32) -> Float
// Walk toggle speed (CET setting mm.walkSpeed, DLL default 0 = off): target capsule speed (m/s, 0 .. 3.5) at the full stick while
// the game walk toggle is on. UTPPSetMotionWalkSpeed stores it (false = NaN / out of range, previous kept); UTPPMotionWalkSpeedTarget (internal,
// UTPPUpdateWalkSpeed) answers it only while motion drives third-person locomotion, else exactly 0.0 (= no modifier). Lock-free (atomic loads).
// UTPPFeedMotionTier2Slow: GameplayRestriction.Tier2LocomotionSlow is active (sent right before every UTPPFeedLocomotion); the motion driver
// lowers the stand run cap by 1.3 m/s. Keep these declarations and the DLL natives (Main.cpp) in sync.
public native func UTPPSetMotionWalkSpeed(metersPerSecond: Float) -> Bool
public native func UTPPMotionWalkSpeedTarget() -> Float
public native func UTPPFeedMotionTier2Slow(slow: Bool) -> Void
// Minimum analog walk speed (CET setting mm.minWalkSpeed, DLL default 0 = off): capsule speed (m/s at the stand run curve, 0 .. 1.6) a
// small stick tilt is lifted to. UTPPSetMotionMinAnalogWalk stores it (false = NaN / out of range, previous kept); UTPPMotionAnalogFloorMul
// (internal, UTPPUpdateAnalogFloor) answers the MaxSpeed multiplier (1 .. 4, up to 10 under the walk toggle) only while the floor is on and motion drives third-person
// locomotion, else exactly 1.0 (also for detailed other than Stand 1 / Crouch 3, a released stick or a stick at / above the floor).
// walk: the game walk toggle is on in the stand state - the floor's m/s is then measured against the walk toggle speed (mm.walkSpeed).
// Lock-free (atomic loads). Keep these declarations and the DLL natives (Main.cpp) in sync.
public native func UTPPSetMotionMinAnalogWalk(metersPerSecond: Float) -> Bool
public native func UTPPMotionAnalogFloorMul(moveX: Float, moveY: Float, detailed: Int32, walk: Bool) -> Float
// ---- AIR HOLD (CET mm.air / mm.airProbe / mm.airSteer) ----
// Air area mode: 0 auto (default), 1 clips, 2 hold, 3 off. auto / clips run the data-free hold mode
// (no mid-air search / turn / pivot picks, no foot locks or offset-root drag in the air, slowed stride, clean hand-back). False = invalid.
public native func UTPPSetMMAir(mode: Int32) -> Bool
// Debug tuning (no UI): take-off lead frames (2), land lead s (0.20), rate min / max (0.7 / 1.8), recover light / heavy s (0.30 /
// 0.50), hold-mode stride rate (0.5), heavy impact m/s (7). False = NaN / out of range, previous kept.
public native func UTPPSetMMAirParams(takeoffLeadFrames: Int32, landLeadSeconds: Float, rateMin: Float, rateMax: Float, recoverLightSeconds: Float, recoverHeavySeconds: Float, holdRate: Float, heavyVz: Float) -> Bool
// Ground raycast while descending (default on; off = dead-reckoned flat-jump height only).
public native func UTPPSetMMAirProbe(on: Bool) -> Void
// In-air body steering (default off): while airborne the body yaw follows the stick target at <= 200 deg/s (UTPPMMBodyYawCommand).
public native func UTPPSetMMAirSteer(on: Bool) -> Void
// AIR stage (CET mm.airRobust, default on): physics falls (walk / run off a ledge, vehicle exit launch) get air handling;
// one-frame or rising touchdowns are ignored. Off = the previous air phase machine.
public native func UTPPSetMMAirRobust(on: Bool) -> Void
// CLIMB RELEASE stage (CET mm.climbRelease, default off): with mm.motionDriver on, every PSM Climb 8 / Vault 9 that the motion path
// does not play is released to the game climb / vault animation (MM fades out with the normal gate fade and back in after it),
// whatever this setting says. The setting only matters with mm.motionDriver off (base only): on = released the same way, off = held on the
// MM pose. Always true.
public native func UTPPSetMMClimbRelease(on: Bool) -> Bool
// Aim chest align (default on; CET setting bodyturn.aimChestAlign; off = the plain aim strafe pose path): while strafing, the chest
// twist of the side / back loops is turned back towards the forward-walk chest (Spine only; hips and legs untouched). Params
// (console): swayKeep 0.5 (0..1, part of the sway kept), chestMaxDeg 60 (0..90). False = invalid / NaN, previous kept.
// Status: chestAlign= ... chestDev= chestCorr= in UTPPGetStatus().
public native func UTPPSetMMChestAlign(on: Bool) -> Void
public native func UTPPSetMMChestAlignParams(swayKeep: Float, chestMaxDeg: Float) -> Bool
// (default off; CET setting mm.feedDump): feed dump - one line per locomotion feed (raw feed, base clip choice, gates) into
// red4ext\logs\utpp-feed-<yyyy-mm-dd-hh-mm-ss>.txt (first line "# cols ..."), written by a background thread about every second
// and when switched off. False = could not start. Status: UTPPMMFeedDumpStatus() and feedDump="..." in UTPPGetStatus().
public native func UTPPSetMMFeedDump(on: Bool) -> Bool
public native func UTPPMMFeedDumpStatus() -> String

// vanilla 2.31 PlayerPuppet.OnMountingEvent (player.script) sets PSM.MountedToVehicle; the MM weight is cut here already,
// before the next locomotion feed, for any vehicle (2- and 4-wheelers). Other mounts (no VehicleObject parent) are ignored.
@wrapMethod(PlayerPuppet)
protected cb func OnMountingEvent(evt: ref<MountingEvent>) -> Bool {
  if IsDefined(evt) && IsDefined(evt.request) {
    let vehicle: ref<VehicleObject> = GameInstance.FindEntityByID(this.GetGame(), evt.request.lowLevelMountingInfo.parentId) as VehicleObject;
    if IsDefined(vehicle) {
      UTPPMMMountEventKind(IsDefined(vehicle as CarObject));
    }
  }
  return wrappedMethod(evt);
}

// ---- head-look driver for the CET module utpp_core ----
// Passive until utpp_core calls it.

// Head look in the DLL (mm/HeadLook.hpp): utpp_core sends its decision every tick while it looks (the tick is the heartbeat: a target
// older than 0.25 s counts as none) and once with weight 0 to release. target = world point; mode 0 camera / 1 face / 2 loot; chest
// 0..1; maxYawDeg 0..160. The layer turns Spine3 (a little, never with a weapon in hand), Neck, Neck1, Head and the eyes on the final
// local pose, with limits and a critically damped ease in / out. Returns true when it can act now (rig resolved, motion matching on, the
// motion pose owns the head: no reload / shot / aim mask closed its gate within 0.5 s):
// the caller then does not need the game's own look-at request (UTPPHeadLook below, which only moves the eye bones that the motion
// pose overwrites). False: older DLL behaviour, use UTPPHeadLook.
public native func UTPPSetHeadLook(target: Vector4, weight: Float, mode: Int32, chest: Float, maxYawDeg: Float) -> Bool
// "headlook rig= fk= on= valid= mode= w= chest= maxYaw= age= why= gate= gaze=yaw/pitch head=yaw/pitch applied chest= yaw= pitch= eyes=
//  flags= calls= appliedCalls= busy= reapplied= fails= gapResets= chestHeld=" (degrees)
public native func UTPPHeadLookStatus() -> String

// Head-look driver: one static look-at target on the player (eyes, with head and chest parts). Apply() replaces the previous
// request (new request queued first, old one removed), Clear() removes it. Decisions (when / where) are made in utpp_core.
// The DLL layer above is the primary path; this request is the fallback when the layer cannot act.
public class UTPPHeadLook extends ScriptableSystem {
  private let m_event: ref<LookAtAddEvent>;
  private let m_applied: Int32;

  private final func Player() -> ref<GameObject> {
    return GameInstance.GetPlayerSystem(this.GetGameInstance()).GetLocalPlayerMainGameObject();
  }

  public func Apply(target: Vector4, headWeight: Float, chestWeight: Float, transitionSpeed: Float) -> Bool {
    let player: ref<GameObject> = this.Player();
    if !IsDefined(player) {
      return false;
    }
    // a ragdoll episode (utpp_ragdoll.reds) owns the body: no look-at until it ends. CET reads Game.UTPPRagdollBusy() as a HeadLook
    // block (HL.Blocked "ragdoll": clears its state, engages again after the get-up) and resets its state when this returns false.
    if UTPPRgBusy(player as PlayerPuppet) {
      return false;
    }
    let ev: ref<LookAtAddEvent> = new LookAtAddEvent();
    ev.SetStaticTarget(target);
    ev.bodyPart = n"Eyes";
    ev.SetStyle(animLookAtStyle.Normal);
    ev.SetLimits(animLookAtLimitDegreesType.Wide, animLookAtLimitDegreesType.Wide, animLookAtLimitDistanceType.None,
      animLookAtLimitDegreesType.Normal);
    let req: LookAtRequest = ev.request;
    req.calculatePositionInParentSpace = true;
    req.hasOutTransition = true;
    if transitionSpeed > 0.0 {
      req.transitionSpeed = transitionSpeed;
      req.outTransitionSpeed = transitionSpeed;
    }
    ev.request = req;
    let parts: array<LookAtPartRequest>;
    let part: LookAtPartRequest;
    part.partName = n"Head";
    part.weight = headWeight;
    part.suppress = 0.5;
    part.mode = 0;
    ArrayPush(parts, part);
    if chestWeight > 0.0 {
      part.partName = n"Chest";
      part.weight = chestWeight;
      part.suppress = 0.0;
      part.mode = 0;
      ArrayPush(parts, part);
    }
    ev.SetAdditionalPartsArray(parts);
    player.QueueEvent(ev);
    if IsDefined(this.m_event) {
      LookAtRemoveEvent.QueueRemoveLookatEvent(player, this.m_event);
    }
    this.m_event = ev;
    this.m_applied += 1;
    return true;
  }

  public func Clear() -> Bool {
    if !IsDefined(this.m_event) {
      return false;
    }
    let player: ref<GameObject> = this.Player();
    if IsDefined(player) {
      LookAtRemoveEvent.QueueRemoveLookatEvent(player, this.m_event);
    }
    this.m_event = null;
    return true;
  }

  public func IsActive() -> Bool {
    return IsDefined(this.m_event);
  }

  public func AppliedCount() -> Int32 {
    return this.m_applied;
  }
}

// UTPP car door hold (CAR enter): keeps the car door open until the UTPP third-person entry clip has cleared the doorway.
//
// Why: the UTPP third-person car 'enter' clips (utpp_vehicle_car.archive, see utpp_vehicle_anim.reds) are longer than the vanilla
// first-person enter. The vanilla door close is scheduled from script and knows nothing about the clip:
//   * VehicleComponent.OnMountingEvent queues a VehicleDoorOpen for the mount slot (autoCloseTime = vehicleDataPackage.Normal_open(),
//     shouldAutoClose unless the car is destroyed / a combat-doors driver holds a weapon); skipped for isInstant mounts;
//   * VehicleComponent.OnVehicleDoorOpen then DelayPSEvent's a VehicleDoorClose after autoCloseTime (0 -> 1.5 s).
// Measured in game: open event +0.047 s after the mount, close starts +1.555 s and takes 0.458 s (Turbo-R lifting door: +0.08 / close
// +1.88 / 1.03 s). V is still in the doorway then, so the door passes through her.
//
// What (all behind one switch, default on):
//   * VehicleComponent.OnMountingEvent wrap (before vanilla): for the player's own, non-instant, non-steal mount into a car (not a
//     bike) it stores a one-shot entry token on the component: door bone name, seat variable and sim time. Nothing else changes.
//   * VehicleComponent.OnVehicleDoorOpen wrap: when the open event matches the token (same bone, fresh < 1 s, shouldAutoClose true =
//     the game's own verdict on destroyed / combat doors) the token is consumed and closeAfter = vanilla delay +
//     UTPPCarDoorCloseDeltaS(seat variable) (generated utpp_vehicle_door_timing.reds: seconds the close START is moved past the vanilla
//     close, margin included; -1 = seat without UTPP timing). Anchored to the vanilla close, not to the detected door motion the
//     offline model measures from: the open-motion latency after the event differs per class (Columbus ~0.05 s later) and cancels
//     out this way. Only when the delta is positive: shouldAutoClose is cleared for the vanilla call (so vanilla schedules no close),
//     restored afterwards, and the same VehicleDoorClose is scheduled by our own DelayCallback at closeAfter. The close is never
//     earlier than vanilla; every other case calls vanilla unchanged (also when another door of the car still has our close pending).
//   * Our close is a plain VehicleDoorClose through the persistency system, the same way vanilla CloseDoor / CloseSelectedDoors queue
//     it. It is cancelled (DelayCallback cancelled + serial dropped) when ANY later door-open on the same bone arrives while it is
//     pending: that is the exit (VehicleExternalDoorRequestEvent -> PS -> OnVehicleDoorOpen; Doors-type driver-combat cars send
//     autoClose=false there, so the condition must not depend on shouldAutoClose) or the driver-combat OpenDoor. The opener then
//     decides what happens to the door and the exit stays 100% vanilla.
//   * Hold (VehicleComponentPS.OnVehicleDoorClose wrap): while our close is pending, a FOREIGN close of the same bone is dropped
//     (DoNotNotifyEntity, door state stays Open; forceScene quest closes pass). It is released the moment our close fires or is
//     cancelled (before our close is queued, so it needs no tag) and expires on its own 1 s after our close was due.
//
// Seat variable (design choice): computed here the way the game builds it in VehicleComponent.SetAnimsetOverrideForPassenger,
//   bone = vehicle.GetBoneNameFromSlot(slot); key = vehicle.GetAnimsetOverrideForPassenger(bone)
// (log: 'workspot animVariables [standard2_vehicle_seat_front_left,seat_front_left,utpp_vehicle_tpp]' = key, bone, utpp_vehicle_tpp).
// Reason: the entry variables are produced by EnteringEvents.OnEnter (player state machine, after the mount is registered in the
// mounting facility) and ScriptedPuppet.OnWorkspotStartedEvent; their order relative to VehicleComponent.OnMountingEvent is not
// fixed by the scripts (logged at the same timestamp). Deriving the key from the vehicle data has no ordering dependency. Whether
// the UTPP entry applies (utpp_vehicle_tpp in the list) is answered by the existing public UTPPVehicleAnimState.EntryWantsTpp(false,
// true) - the same predicate the SetAnimsetOverrideForPassenger wrap in utpp_vehicle_anim.reds uses (switch + UTPP third-person
// signal); no edit of that file is needed. The combat entry 'enter_fast' (player in combat, driver slot) plays a vanilla clip and is
// excluded, like bikes, steals (mountEventOptions.entityID) and instant mounts (seat swaps).
//
// Other vanilla close paths during the extended window (vehicleComponent.script, vehicleComponent PS, player.script):
//   * speed loop (VehicleComponent.OnVehicleSpeedChange, close all doors at SpeedToClose, returns below 0.5 m/s): gated by
//     m_ignoreAutoDoorClose, true from the player's mount until OnVehicleFinishedMountingEvent. When it does close the door it goes
//     through CloseSelectedDoors -> VehicleDoorClose -> held by the PS hook above while our close is pending;
//   * OnSetIgnoreAutoDoorCloseEvent only flips that flag (PlayVehicleExitDoorAnimation sets it for the exit; HandleDoorsForCombat);
//   * ToDrive seat swap (isInstant mount, CloseDoor): no token, vanilla;
//   * quest ForceClose / CloseAll (forceScene or not), trunk dump, delamain taxi: not part of a normal enter; forceScene closes pass
//     the hold, the rest is held only while our close is pending;
//   * Player.OnVehicleStateChange -> HandleDoorsForCombat (cars whose DriverCombat type is Doors only; player.script): CloseDoor
//     driver + front passenger on the first vehicle-state change that is not Transition / Passenger, i.e. when the entering state
//     ends. EnteringDecisions.ExitCondition (vehicleTransition.script) is TIME driven (in-state time > vehicleDataPackage.Entering()
//     / CombatEntering(), 3.7 s for a steal), not clip driven, so that close can land inside a longer UTPP clip. It is a VehicleDoorClose
//     through the persistency system, so the PS hold above stops it for the held bone while our close is pending. Which of the six
//     contract classes are Doors-type was not available offline (no tweak dump of DriverCombat); runtime-unverified.
//
// Jump-out (V quick dismount): no cancel on unmount. A pending close of ours still fires (later than the vanilla one), the exit
// door logic is untouched; cancelling would leave the entry door open for good because nothing else closes it.
//
// Hold cap (UTPPSetVehicleEntryHoldCap, native; UTPP.dll vehtrans HOLD CAP): the native standing-entry root hold keeps the RENDERED body at
// V's pre-mount spot for up to 0.45 s (released over 0.6 s) while this door opens; when that spot lies in the door's swept sector the door
// swings through the body (hip to head height depending on the car; the entity root itself is clear). At the mount
// event V is still at the pre-mount spot: its position in the vehicle frame (x right, y forward, origin = vehicle entity origin = the 'vtpp'
// log frame) is tested against the door arc of the seat (generated UTPPCarDoorArcOf, utpp_vehicle_door_timing.reds). The door footprint of
// the hold is r <= length + 0.20 m, an angular half width asin(0.20 / r) (body radius 0.15 m + margin 0.05 m); t_edge = the time the door
// angle (openDeg * smoothstep((t - openDelayS) / openDurS)) reaches the near edge of that footprint, and when the uncapped hold would
// still be holding then (t_edge <= 0.75 s: the 0.45 s hold plus half of the 0.6 s release) the native gets cap = t_edge - 0.05 s (seconds
// after the mount cut by which the visible root has to be back on the entity path). The native applies it only to a large, fast hold
// (gate in VehicleTransition.hpp) and only to cars; the math here is mirrored in tests/MMVehicleTransitionTest.cpp (RedsCap) - change both.
//
// Log: "[utpp_vehicle_door] ride-entry seat=<key> slot=<bone> vanilla=<s> delta=<s> closeAfter=<s> applied=<bool> holdcap=[...]" once per
// consumed entry token (holdcap = the test inputs: why, anchor x/y in the vehicle frame, r, theta, t_edge, cap seconds), and one line when
// our close fires / is cancelled / is dropped.
// Switch (default on): Game.UTPPSetVehicleDoorEnabled(false) = vanilla door timing; status: Game.UTPPVehicleDoorStatus().
// Runtime unverified: that the derived key equals entryVars[0] of the same ride (compare with Game.UTPPVehicleTppAnimStatus());
// that the open event always follows the mount event (token fresh window 1.0 s vs measured 0.05-0.08 s); that DelayCallback with time
// dilation matches the vanilla DelayPSEvent timing; that CancelCallback cancels the callback; that VehicleComponentPS.OnVehicleDoorClose
// (a final PS method) takes the wrap and DoNotNotifyEntity leaves the door open; the HandleDoorsForCombat timing above; the Turbo-R
// lifting door; no in-game test of any kind yet.
// Requires utpp.reds (UTPPLogLine), utpp_vehicle_anim.reds (UTPPVehicleAnimStateOf, EntryWantsTpp), utpp_vehicle_door_timing.reds
// (UTPPCarDoorCloseDeltaS, UTPPCarDoorArcOf / UTPPCarDoorArc) and UTPP.dll (UTPPSetVehicleEntryHoldCap).

// Native (UTPP.dll, MotionDriverNatives.cpp): per-mount cap of the standing-entry root hold, seconds after the mount cut; < 0 = none.
public native func UTPPSetVehicleEntryHoldCap(seconds: Float) -> Void

// hold cap constants (see the header)
public func UTPPDoorCapBodyM() -> Float { return 0.20; }        // body radius 0.15 m + margin 0.05 m
public func UTPPDoorCapMarginS() -> Float { return 0.05; }      // cap = t_edge - this
public func UTPPDoorCapHoldKeptS() -> Float { return 1.5; }     // the uncapped hold is assumed to still hold the root up to here
// 0.75 -> 1.5 s with the native glide limit (VehicleTransition.hpp GLIDE LIMIT: an uncapped car hold may last up to 1.5 s)

public struct UTPPDoorCapInfo {
  public let why: Int32;      // 0 = cap, 1 = no arc for the seat, 2 = anchor beyond the door radius, 3 = the door never reaches the footprint, 4 = the uncapped hold is released before the door arrives
  public let r: Float;
  public let thetaDeg: Float; // anchor angle from the closed door, positive toward the open door
  public let tEdgeS: Float;
  public let capS: Float;     // < 0 = none
}

// Inverse of s = 3x^2 - 2x^3 on [0, 1].
public func UTPPDoorInvSmoothstep(s: Float) -> Float {
  let c: Float = ClampF(s, 0.0, 1.0);
  return 0.5 - SinF(AsinF(1.0 - 2.0 * c) / 3.0);
}

// Cap for an anchor (vehicle frame, x right / y forward, metres) against a door arc. Pure function (mirrored in MMVehicleTransitionTest.cpp).
public func UTPPDoorCapOf(arc: UTPPCarDoorArc, px: Float, py: Float) -> UTPPDoorCapInfo {
  let info: UTPPDoorCapInfo;
  info.capS = -1.0;
  info.tEdgeS = -1.0;
  if !arc.valid {
    info.why = 1;
    return info;
  }
  let dx: Float = px - arc.hingeX;
  let dy: Float = py - arc.hingeY;
  let r: Float = SqrtF(dx * dx + dy * dy);
  info.r = r;
  let body: Float = UTPPDoorCapBodyM();
  if r > arc.length + body {
    info.why = 2;
    return info;
  }
  // signed angle (deg, counter-clockwise seen from above) from the closed direction to the anchor direction
  let cd: Float = Deg2Rad(arc.closedDirDeg);
  let ax: Float = CosF(cd);
  let ay: Float = SinF(cd);
  let dot: Float = ax * dx + ay * dy;
  let cross: Float = ax * dy - ay * dx;
  let ang: Float = Rad2Deg(AcosF(ClampF(dot / MaxF(r, 0.0001), -1.0, 1.0)));
  if cross < 0.0 {
    ang = -ang;
  }
  let sgn: Float = 1.0;
  if arc.openDeg < 0.0 {
    sgn = -1.0;
  }
  let th: Float = ang * sgn;
  info.thetaDeg = th;
  let delta: Float = Rad2Deg(AsinF(MinF(1.0, body / MaxF(r, body))));
  let op: Float = AbsF(arc.openDeg);
  if th + delta < 0.0 || th - delta >= op {
    info.why = 3;
    return info;
  }
  let lo: Float = MaxF(th - delta, 0.0);
  info.tEdgeS = arc.openDelayS + arc.openDurS * UTPPDoorInvSmoothstep(lo / op);
  if info.tEdgeS > UTPPDoorCapHoldKeptS() {
    info.why = 4;
    return info;
  }
  info.why = 0;
  info.capS = MaxF(info.tEdgeS - UTPPDoorCapMarginS(), 0.0);
  return info;
}

public func UTPPVehDoorTokenFreshS() -> Float { return 1.00; }

// vanilla OnVehicleDoorOpen fallback when the open event carries autoCloseTime 0
public func UTPPVehDoorVanillaFallbackS() -> Float { return 1.50; }

// the hold on foreign closes ends at the latest this long after our own close was due (safety net if it were never released)
public func UTPPVehDoorHoldSlackS() -> Float { return 1.00; }

public class UTPPVehicleDoorState extends ScriptableSystem {
  private let m_enabled: Bool = true;
  private let m_tokens: Int32;
  private let m_entries: Int32;
  private let m_applied: Int32;
  private let m_fired: Int32;
  private let m_cancelled: Int32;
  private let m_held: Int32;
  private let m_last: String;

  public func SetEnabled(on: Bool) -> Void {
    this.m_enabled = on;
  }

  public func IsEnabled() -> Bool {
    return this.m_enabled;
  }

  public func CountToken() -> Void {
    this.m_tokens += 1;
  }

  public func Entry(applied: Bool, line: String) -> Void {
    this.m_entries += 1;
    if applied {
      this.m_applied += 1;
    }
    this.m_last = line;
    UTPPLogLine(line);
  }

  public func Fired(line: String) -> Void {
    this.m_fired += 1;
    this.m_last = line;
    UTPPLogLine(line);
  }

  public func Cancelled(line: String) -> Void {
    this.m_cancelled += 1;
    this.m_last = line;
    UTPPLogLine(line);
  }

  public func Held(line: String) -> Void {
    this.m_held += 1;
    this.m_last = line;
    UTPPLogLine(line);
  }

  public func Status() -> String {
    let en: String = "false";
    if this.m_enabled {
      en = "true";
    }
    return "enabled=" + en + " tokens=" + IntToString(this.m_tokens) + " entries=" + IntToString(this.m_entries) + " applied="
      + IntToString(this.m_applied) + " fired=" + IntToString(this.m_fired) + " cancelled=" + IntToString(this.m_cancelled)
      + " held=" + IntToString(this.m_held) + " last=[" + this.m_last + "]";
  }
}

public func UTPPVehicleDoorStateOf(game: GameInstance) -> ref<UTPPVehicleDoorState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPVehicleDoorState") as UTPPVehicleDoorState;
}

// Game-callable switch (default on). False = the system is not available yet (no game session).
public func UTPPSetVehicleDoorEnabled(on: Bool) -> Bool {
  let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetEnabled(on);
  UTPPLogLine("UTPPSetVehicleDoorEnabled(" + (on ? "1" : "0") + ")");
  return true;
}

public func UTPPVehicleDoorStatus() -> String {
  let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPVehicleDoorState missing";
  }
  return state.Status();
}

// Delayed close of one door; the component checks the serial so a cancelled / replaced close never fires.
public class UTPPVehicleDoorCloseCallback extends DelayCallback {
  public let m_component: wref<VehicleComponent>;
  public let m_serial: Int32;

  public func Call() -> Void {
    if IsDefined(this.m_component) {
      this.m_component.UTPPDoorCloseFire(this.m_serial);
    }
  }
}

// One-shot entry token (set at the player's mount, consumed by the matching door-open event) and the pending own close.
@addField(VehicleComponent)
public let utppDoorTokSlot: CName;

@addField(VehicleComponent)
public let utppDoorTokKey: CName;

@addField(VehicleComponent)
public let utppDoorTokTime: Float;

@addField(VehicleComponent)
public let utppDoorCapNote: String;

@addField(VehicleComponent)
public let utppDoorPendSlot: CName;

@addField(VehicleComponent)
public let utppDoorPendSerial: Int32;

@addField(VehicleComponent)
public let utppDoorPendId: DelayID;

@addField(VehicleComponent)
public let utppDoorPendOn: Bool;

@addField(VehicleComponent)
public let utppDoorSerial: Int32;

// Hold on the persistent side (see the header: foreign closes of the held door during the window are dropped).
@addField(VehicleComponentPS)
public let utppDoorHoldSlot: CName;

@addField(VehicleComponentPS)
public let utppDoorHoldUntil: Float;

@addMethod(VehicleComponent)
public func UTPPDoorClearToken() -> Void {
  this.utppDoorTokSlot = n"None";
  this.utppDoorTokKey = n"None";
  this.utppDoorCapNote = "";
}

// Hold cap (see the header): V's position in the vehicle frame against the seat's door arc, then the native cap. Called at the mount event,
// before vanilla runs (V is still at the pre-mount spot). The result is kept as a note for the ride-entry log line.
@addMethod(VehicleComponent)
public func UTPPDoorSendHoldCap(vehicle: ref<VehicleObject>, player: ref<PlayerPuppet>, key: CName) -> Void {
  let rel: Vector4 = player.GetWorldPosition() - vehicle.GetWorldPosition();
  let px: Float = Vector4.Dot(rel, vehicle.GetWorldRight());
  let py: Float = Vector4.Dot(rel, vehicle.GetWorldForward());
  let info: UTPPDoorCapInfo = UTPPDoorCapOf(UTPPCarDoorArcOf(key), px, py);
  this.utppDoorCapNote = " holdcap=[why=" + IntToString(info.why) + " anchor=(" + FloatToStringPrec(px, 2) + "," + FloatToStringPrec(py, 2)
    + ") r=" + FloatToStringPrec(info.r, 2) + " theta=" + FloatToStringPrec(info.thetaDeg, 1) + " tEdge=" + FloatToStringPrec(info.tEdgeS, 3)
    + " cap=" + FloatToStringPrec(info.capS, 3) + "]";
  if info.capS >= 0.0 {
    UTPPSetVehicleEntryHoldCap(info.capS);
  }
}

// Mount-time token. Only the player's own mount into a car as a normal (non-instant, non-steal, non-combat-driver) entry.
@addMethod(VehicleComponent)
public func UTPPDoorNoteMount(evt: ref<MountingEvent>) -> Void {
  let vehicle: ref<VehicleObject> = this.GetVehicle();
  if !IsDefined(evt) || !IsDefined(evt.request) || !IsDefined(vehicle) {
    return;
  }
  let game: GameInstance = vehicle.GetGame();
  let player: ref<PlayerPuppet> = GameInstance.FindEntityByID(game, evt.request.lowLevelMountingInfo.childId) as PlayerPuppet;
  if !IsDefined(player) {
    return; // an NPC mount (passenger, companion) must not wipe the player's token
  }
  this.UTPPDoorClearToken();
  if evt.request.mountData.isInstant || EntityID.IsDefined(evt.request.mountData.mountEventOptions.entityID)
      || IsDefined(vehicle as BikeObject) {
    return;
  }
  let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(game);
  let anim: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(game);
  if !IsDefined(state) || !state.IsEnabled() || !IsDefined(anim) || !anim.EntryWantsTpp(false, true) {
    return;
  }
  let slot: CName = evt.request.lowLevelMountingInfo.slotId.id;
  if player.IsInCombat() && VehicleComponent.IsDriverSlot(slot) {
    return; // 'enter_fast' (vanilla clip), no UTPP timing
  }
  let bone: CName = vehicle.GetBoneNameFromSlot(slot);
  this.utppDoorTokSlot = bone;
  this.utppDoorTokKey = vehicle.GetAnimsetOverrideForPassenger(bone);
  this.utppDoorTokTime = EngineTime.ToFloat(GameInstance.GetSimTime(game));
  state.CountToken();
  this.UTPPDoorSendHoldCap(vehicle, player, this.utppDoorTokKey);
}

@addMethod(VehicleComponent)
public func UTPPDoorReleaseHold() -> Void {
  let ps: ref<VehicleComponentPS> = this.GetPS();
  if IsDefined(ps) {
    ps.utppDoorHoldSlot = n"None";
  }
}

// Drops our pending close (if any). Returns true when one was pending.
@addMethod(VehicleComponent)
public func UTPPDoorCancelPending(why: String) -> Bool {
  if !this.utppDoorPendOn {
    return false;
  }
  this.utppDoorPendOn = false;
  this.UTPPDoorReleaseHold();
  this.utppDoorSerial += 1; // invalidates the callback even if the cancel below does not reach it
  GameInstance.GetDelaySystem(this.GetVehicle().GetGame()).CancelCallback(this.utppDoorPendId);
  let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(this.GetVehicle().GetGame());
  if IsDefined(state) {
    state.Cancelled("[utpp_vehicle_door] own close cancelled slot=" + NameToString(this.utppDoorPendSlot) + " (" + why + ")");
  }
  return true;
}

@addMethod(VehicleComponent)
public func UTPPDoorCloseFire(serial: Int32) -> Void {
  let vehicle: ref<VehicleObject> = this.GetVehicle();
  if !this.utppDoorPendOn || serial != this.utppDoorSerial || !IsDefined(vehicle) {
    return; // cancelled or replaced
  }
  this.utppDoorPendOn = false;
  this.UTPPDoorReleaseHold(); // before the queue: our own close must pass the PS hook
  let close: ref<VehicleDoorClose> = new VehicleDoorClose();
  close.slotID = this.utppDoorPendSlot;
  GameInstance.GetPersistencySystem(vehicle.GetGame()).QueuePSEvent(this.GetPS().GetID(), this.GetPS().GetClassName(), close);
  let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(vehicle.GetGame());
  if IsDefined(state) {
    state.Fired("[utpp_vehicle_door] own close fired slot=" + NameToString(this.utppDoorPendSlot));
  }
}

// Chains with the other VehicleComponent wraps (none touch the door events). The token is set before vanilla runs, so an open
// event delivered inside the vanilla call could not miss it.
@wrapMethod(VehicleComponent)
protected cb func OnMountingEvent(evt: ref<MountingEvent>) -> Bool {
  this.UTPPDoorNoteMount(evt);
  return wrappedMethod(evt);
}

@wrapMethod(VehicleComponent)
protected cb func OnVehicleDoorOpen(evt: ref<VehicleDoorOpen>) -> Bool {
  if !IsDefined(evt) {
    return wrappedMethod(evt);
  }
  let vehicle: ref<VehicleObject> = this.GetVehicle();
  let state: ref<UTPPVehicleDoorState> = null;
  if IsDefined(vehicle) {
    state = UTPPVehicleDoorStateOf(vehicle.GetGame());
  }
  if !IsDefined(state) {
    return wrappedMethod(evt);
  }
  let hasToken: Bool = IsNameValid(this.utppDoorTokSlot) && Equals(this.utppDoorTokSlot, evt.slotID);
  if !hasToken {
    // any later open of the pending door (the exit, driver combat doors; with or without an auto close of its own): drop ours,
    // the opener decides what happens to the door
    if this.utppDoorPendOn && Equals(this.utppDoorPendSlot, evt.slotID) {
      this.UTPPDoorCancelPending("door re-opened");
    }
    return wrappedMethod(evt);
  }
  // consume the token whatever happens next (one-shot)
  let key: CName = this.utppDoorTokKey;
  let capNote: String = this.utppDoorCapNote;
  let age: Float = EngineTime.ToFloat(GameInstance.GetSimTime(vehicle.GetGame())) - this.utppDoorTokTime;
  this.UTPPDoorClearToken();
  if age < 0.0 || age > UTPPVehDoorTokenFreshS() || !evt.shouldAutoClose || !state.IsEnabled() {
    return wrappedMethod(evt);
  }
  let vanilla: Float = evt.autoCloseTime;
  if vanilla == 0.0 {
    vanilla = UTPPVehDoorVanillaFallbackS();
  }
  let delta: Float = UTPPCarDoorCloseDeltaS(key);
  let closeAfter: Float = vanilla + MaxF(delta, 0.0);
  // one pending close per component: while another door still waits for ours, this door keeps the vanilla close (cancelling the
  // other one would leave that door open, its vanilla close was suppressed)
  let otherPending: Bool = this.utppDoorPendOn && !Equals(this.utppDoorPendSlot, evt.slotID);
  let applied: Bool = delta > 0.0 && !otherPending;
  state.Entry(applied, "[utpp_vehicle_door] ride-entry seat=" + NameToString(key) + " slot=" + NameToString(evt.slotID) + " vanilla="
    + FloatToStringPrec(vanilla, 2) + " delta=" + FloatToStringPrec(delta, 3) + " closeAfter=" + FloatToStringPrec(closeAfter, 2)
    + " applied=" + UTPPBoolStr(applied) + (otherPending ? " (other door pending)" : "") + capNote);
  if !applied {
    return wrappedMethod(evt);
  }
  // a still-pending close of ours for an earlier entry through the same door is replaced by this one
  this.UTPPDoorCancelPending("replaced by a new entry");
  evt.shouldAutoClose = false;
  let result: Bool = wrappedMethod(evt);
  evt.shouldAutoClose = true;
  this.utppDoorSerial += 1;
  this.utppDoorPendSerial = this.utppDoorSerial;
  this.utppDoorPendSlot = evt.slotID;
  let cb: ref<UTPPVehicleDoorCloseCallback> = new UTPPVehicleDoorCloseCallback();
  cb.m_component = this;
  cb.m_serial = this.utppDoorSerial;
  this.utppDoorPendId = GameInstance.GetDelaySystem(vehicle.GetGame()).DelayCallback(cb, closeAfter, true);
  this.utppDoorPendOn = true;
  let ps: ref<VehicleComponentPS> = this.GetPS();
  if IsDefined(ps) {
    ps.utppDoorHoldSlot = evt.slotID;
    ps.utppDoorHoldUntil = EngineTime.ToFloat(GameInstance.GetSimTime(vehicle.GetGame())) + closeAfter + UTPPVehDoorHoldSlackS();
  }
  return result;
}

// Persistent side: the PS applies Closed before the component sees the event, so this is the only place a foreign close of the held
// door can still be stopped. Only while our own close is pending for that slot (hold set at the consumed entry, released when our
// close fires or is cancelled; safety expiry above). Dropped: the speed loop and HandleDoorsForCombat closes. Not dropped: forceScene
// (quest) closes and everything else outside the window - vanilla.
@wrapMethod(VehicleComponentPS)
public final func OnVehicleDoorClose(evt: ref<VehicleDoorClose>) -> EntityNotificationType {
  if IsDefined(evt) && !evt.forceScene && IsNameValid(this.utppDoorHoldSlot) && Equals(this.utppDoorHoldSlot, evt.slotID) {
    let game: GameInstance = this.GetGameInstance();
    if EngineTime.ToFloat(GameInstance.GetSimTime(game)) < this.utppDoorHoldUntil {
      let state: ref<UTPPVehicleDoorState> = UTPPVehicleDoorStateOf(game);
      if IsDefined(state) {
        state.Held("[utpp_vehicle_door] foreign close held slot=" + NameToString(evt.slotID) + " (own close pending)");
      }
      return EntityNotificationType.DoNotNotifyEntity;
    }
    this.utppDoorHoldSlot = n"None";
  }
  return wrappedMethod(evt);
}

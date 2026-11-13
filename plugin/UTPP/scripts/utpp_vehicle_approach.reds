// UTPP car entry approach walk: V walks to the entry clip's start spot, then the mount is issued.
//
// Why: when V chooses "enter" on a car, the engine slides the entity to the start spot of the entry clip in a FIXED time (about 0.30 s, Turbo-R 0.82 s)
// whatever the distance. The UTPP third-person clips start 0.85-2.8 m from where V usually stands, so the slide skates her toward the rear of the car at
// 5-16 m/s and she enters diagonally through the bodywork. Native hold / glide limits cannot fix it (the door hold has to release before the door opens),
// and changing the clip paths fails the door gates. So the distance is removed before the mount: V walks to the start spot first and the vanilla mount
// runs when she is there; the engine slide is then a few centimetres.
//
// What (one switch, default on; setting vehicle.approachWalk, Game.UTPPSetVehicleApproachEnabled):
//   * gamevehicleVehicleMountableComponent.OnInteractionChoice wrap: for the player's own regular mount into a CAR seat that has a row in the generated table
//     (utpp_vehicle_approach_table.reds: class / seat / body -> start spot in the vehicle frame) the mount is deferred. The conditions are exactly those under
//     which the table applies: on foot and alive, not in combat (that mount plays the vanilla 'enter_fast' clip), the UTPP third-person entry applies
//     (UTPPVehicleAnimState.EntryWantsTpp, otherwise the vanilla first-person clip with another start spot plays), the seat is free (no steal), the vehicle stands
//     still, PSM Locomotion is Default, no scene / menu / traversal / swimming / vehicle state. Anything else: the vanilla path, untouched.
//   * Tick (wrap of LocomotionEventsTransition.OnUpdate, before the PSM reads its input): the native planner (src/input/ApproachCore.hpp) steers a stick
//     override (UTPPLocomotionSetMoveOverride channel, src/input/LocomotionCore.hpp) from V's pose in the vehicle frame to the spot; see the native header for the
//     path (footprint box, closed-door sweep sector), the speed profile and the arrival rule.
//   * Walk quality (native planner): the stick stays below the matcher's Walk/Run split (0.66 cap; a larger stick selects run-start clips) and rises at a
//     bounded rate; while the body is more than 80 deg off the direction of the current leg (start of the walk, after a sharp corner) the stick is held
//     low until she has turned (turn first, bounded to 0.7 s per leg), and the speed is capped before a sharp waypoint corner. A low-stick stall near the
//     spot is lifted by a stick boost and, if V still does not move, ends the position phase.
//   * Path: chosen by an estimated cost (native header): direct to the spot, direct to a settle point behind the spot (the facing turn carries V along the
//     clip heading onto the spot; the end line shows settle=<m>), or through the run-up point. The stick never points more than 60 deg away from the
//     body yaw (V arcs round a turn instead of walking against her facing) and keeps its direction within 0.35 m of a waypoint.
//   * Arrival: position phase done (<= 0.03 m of the walk's end, the spot was passed or a stall within 0.3 m), then, when
//     the body yaw is controllable, a facing phase turns V (small stick toward the clip's start yaw) until her yaw is within 8 deg of it (bounded to
//     1.2 s; the end line shows yawErr= and faceGaveUp=1 when the bound was hit) AND she stands within 0.03 m of the spot (was 0.10 m: aligned but short of it: she
//     creeps along the clip heading, at most 0.3 s inside the 0.10 m gate, then the mount runs from there; past the spot by up to 0.10 m: arrived, farther: end reason near_spot, the
//     mount slide is short). The end line's residual= is the jump from the last approach sample to the first clip sample (entry_feel E6 limit 0.03 m). A start within 0.03 m of
//     the spot and 25 deg of its yaw mounts at once ('skip code=0' line; was 0.35 m). A timeout within 0.6 m also mounts. The stored choice event is
//     dispatched to OnInteractionChoice again with a one-shot bypass flag, so the game's own (2.5) acceptance checks and mount code run unchanged.
//   * Abort without a mount (V can press F again): timeout farther than 0.6 m, the player pushes the stick against / beside the path (> 0.5), damage, combat,
//     status-effect knockdown, the vehicle moves (> 0.5 m/s), a scene / menu / traversal state, a second F press (that one runs the vanilla mount at once),
//     a stale tick (no locomotion update for 0.5 s).
//   * No-motion guard: when V has not moved 0.08 m after 0.45 s although the stick override was set (the engine's native movement may read the hardware
//     input and ignore what the PSM scripts see), the mount is issued at once (vanilla behaviour). The second such result in a row switches the approach
//     off for the session (one can be V blocked by a wall, a car or an NPC); an arrival resets the count.
//
// Log: "[utpp_vehicle_approach] start ..." (seat, body, from / to pose in the vehicle frame, distance, path), "end reason=<arrived|timeout_near|timeout|input|
// no_motion|...> mount=<bool> t=<s> residual=<m>" and "mount-check" 1.5 s after a dispatched mount (mounted or not).
// Console: Game.UTPPVehicleApproachStatus(), Game.UTPPSetVehicleApproachEnabled(false).
// Requires utpp.reds (UTPPLogLine, UTPPYawDegFromForward), utpp_locomotion.reds (UTPPLocomotionActive, UTPPLocomotionReasons),
// utpp_vehicle_anim.reds (UTPPVehicleAnimStateOf / EntryWantsTpp, UTPPBoolStr), utpp_vehicle_approach_table.reds and UTPP.dll (natives below).
// Independent of utpp_vehicle_door*.reds (the door sweep sector is part of the generated approach table).
// Runtime unverified: everything (no in-game test yet). The first test showed that the native player movement does NOT follow the PSM-visible stick (V stayed put,
// end reason=no_motion); the native side therefore also hooks the engine's locomotion input reader and overwrites its vector while an approach runs (src/input/NativeInputCore.hpp).
// The approach refuses to start (UTPPApproachBegin = -5) when that hook is not attached. Open question: does the capsule follow it? The tick log line (every 0.1 s) answers it:
// "[utpp_vehicle_approach] tick t= cmd=(x,y) pos=(px,py) dpos= ref= yaw= velYaw= spotErr= ph= stick= spd= nativeApplied= ..." (dpos = metres moved in the vehicle
// frame since the previous line; ref = reference yaw of the stick, yaw = body yaw, velYaw = measured velocity yaw, spotErr = body yaw error vs the clip start
// yaw, ph = 0 walk / 1 turn-first hold / 2 facing, stick = commanded magnitude, spd = measured speed in m/s; all in the vehicle frame).

// Native (UTPP.dll, src/input/Locomotion.cpp): stick override channel and planner.
public native func UTPPLocomotionSetMoveOverride(x: Float, y: Float, headingDeg: Float, ttlSeconds: Float) -> Void
public native func UTPPLocomotionClearMoveOverride() -> Void
public native func UTPPLocomotionOverrideReady() -> Bool
public native func UTPPApproachSetScene(boxCx: Float, boxCy: Float, boxHx: Float, boxHy: Float, arcValid: Bool, hingeX: Float, hingeY: Float, arcLength: Float, arcClosedDeg: Float, arcOpenDeg: Float) -> Void
public native func UTPPApproachBegin(px: Float, py: Float, yawDeg: Float, spotX: Float, spotY: Float, spotYawDeg: Float) -> Int32
public native func UTPPApproachStep(px: Float, py: Float, yawDeg: Float, refYawDeg: Float, dt: Float, yawControl: Bool) -> Int32
public native func UTPPApproachCancel() -> Void
public native func UTPPApproachActive() -> Bool
public native func UTPPApproachInfo() -> String
public native func UTPPApproachTickInfo(t: Float, px: Float, py: Float, dpos: Float) -> String

// Locomotion heartbeat reasons (utpp::freemove::Reason) that end / forbid an approach: vehicle 1, scene 2, swimming 4, workspot 8, takedown 16, carrying 32,
// vitals 128, menu 256, reaction 1024, traversal 2048.
public func UTPPApproachBlockedReasons() -> Int32 { return 3519; }
public func UTPPApproachHealthDropPct() -> Float { return 0.2; }   // health lost since the start that counts as damage, in percent points (GetStatPoolValue(.., true) is 0..100)
public func UTPPApproachStaleS() -> Float { return 0.5; }          // no locomotion update for this long ends the approach
public func UTPPApproachVehicleSpeedMps() -> Float { return 0.5; } // the vehicle moves faster than this: no approach / abort
public func UTPPApproachMountCheckS() -> Float { return 1.5; }

public struct UTPPApproachPose {
  public let ok: Bool;
  public let px: Float;
  public let py: Float;
  public let yawDeg: Float;
  public let carYawDeg: Float;
}

// V's position and facing in the vehicle frame (x right, y forward; yaw UTPP convention: 0 = vehicle forward, counter-clockwise).
public func UTPPApproachPoseOf(vehicle: ref<VehicleObject>, player: ref<PlayerPuppet>) -> UTPPApproachPose {
  let pose: UTPPApproachPose;
  if !IsDefined(vehicle) || !IsDefined(player) {
    return pose;
  }
  let right: Vector4 = vehicle.GetWorldRight();
  let fwd: Vector4 = vehicle.GetWorldForward();
  let rel: Vector4 = player.GetWorldPosition() - vehicle.GetWorldPosition();
  pose.px = Vector4.Dot(rel, right);
  pose.py = Vector4.Dot(rel, fwd);
  let pf: Vector4 = player.GetWorldForward();
  pose.yawDeg = UTPPYawDegFromForward(Vector4(Vector4.Dot(pf, right), Vector4.Dot(pf, fwd), 0.0, 0.0));
  pose.carYawDeg = UTPPYawDegFromForward(fwd);
  pose.ok = true;
  return pose;
}

public class UTPPVehicleApproachState extends ScriptableSystem {
  private let m_enabled: Bool = true;
  private let m_latchedOff: Bool;
  private let m_noMotionStreak: Int32;
  private let m_active: Bool;
  private let m_comp: wref<gamevehicleVehicleMountableComponent>;
  // The deferred choice event and the one-shot bypass of the OnInteractionChoice wrap. Kept here, not as @addField on the mountable
  // component: that class is native-only (no script definition) and the game failed "to initialize scripts data" at boot with the
  // two fields added (scripts boot fine without this file).
  private let m_evt: ref<InteractionChoiceEvent>;
  private let m_bypass: Bool;
  private let m_vehicle: wref<VehicleObject>;
  private let m_seat: CName;
  private let m_t0: Float;
  private let m_lastTick: Float;
  private let m_health0: Float;
  private let m_starts: Int32;
  private let m_mounts: Int32;
  private let m_aborts: Int32;
  private let m_last: String;
  private let m_logT: Float;
  private let m_logPx: Float;
  private let m_logPy: Float;
  private let m_logInit: Bool;

  public func SetEnabled(on: Bool) -> Void {
    this.m_enabled = on;
    if on {
      this.m_latchedOff = false; // a new switch-on gives the approach a fresh start (the no-motion latch is per switch-on, not per game load)
    }
    if !on && this.m_active {
      this.Finish("setting off", false);
    }
  }

  public func IsEnabled() -> Bool {
    return this.m_enabled && !this.m_latchedOff;
  }

  public func IsActive() -> Bool {
    return this.m_active;
  }

  public func IsStale(game: GameInstance) -> Bool {
    return EngineTime.ToFloat(GameInstance.GetSimTime(game)) - this.m_lastTick > UTPPApproachStaleS();
  }

  public func Note(line: String) -> Void {
    this.m_last = line;
    UTPPLogLine(line);
  }

  public func SetBypass(on: Bool) -> Void {
    this.m_bypass = on;
  }

  // True once after SetBypass(true): the wrap hands this one call to the vanilla handler.
  public func TakeBypass() -> Bool {
    let b: Bool = this.m_bypass;
    this.m_bypass = false;
    return b;
  }

  public func Begin(comp: ref<gamevehicleVehicleMountableComponent>, evt: ref<InteractionChoiceEvent>, vehicle: ref<VehicleObject>, player: ref<PlayerPuppet>, seat: CName, line: String) -> Void {
    let game: GameInstance = vehicle.GetGame();
    this.m_comp = comp;
    this.m_evt = evt;
    this.m_vehicle = vehicle;
    this.m_seat = seat;
    this.m_t0 = EngineTime.ToFloat(GameInstance.GetSimTime(game));
    this.m_lastTick = this.m_t0;
    this.m_logInit = false;
    this.m_health0 = GameInstance.GetStatPoolsSystem(game).GetStatPoolValue(Cast<StatsObjectID>(player.GetEntityID()), gamedataStatPoolType.Health, true);
    this.m_active = true;
    this.m_starts += 1;
    this.Note(line);
  }

  // Ends the approach. mount = dispatch the stored choice event to the vanilla handler (bypass flag).
  public func Finish(reason: String, mount: Bool) -> Void {
    if !this.m_active {
      return;
    }
    this.m_active = false;
    let info: String = UTPPApproachInfo();
    UTPPApproachCancel();
    let game: GameInstance = GetGameInstance();
    let t: Float = 0.0;
    if IsDefined(this.m_vehicle) {
      game = this.m_vehicle.GetGame();
      t = EngineTime.ToFloat(GameInstance.GetSimTime(game)) - this.m_t0;
    }
    this.Note("[utpp_vehicle_approach] end reason=" + reason + " mount=" + UTPPBoolStr(mount) + " seconds=" + FloatToStringPrec(t, 2) + " " + info);
    let comp: ref<gamevehicleVehicleMountableComponent> = this.m_comp;
    let evt: ref<InteractionChoiceEvent> = this.m_evt;
    this.m_evt = null;
    if mount && IsDefined(comp) {
      this.m_mounts += 1;
      comp.UTPPApproachMount(this, evt);
    } else {
      this.m_aborts += 1;
    }
  }

  public func Tick(dt: Float, scriptInterface: ref<StateGameScriptInterface>) -> Void {
    let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
    let vehicle: ref<VehicleObject> = this.m_vehicle;
    if !IsDefined(player) || !IsDefined(vehicle) {
      this.Finish("vehicle or player gone", false);
      return;
    }
    let game: GameInstance = player.GetGame();
    let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(game));
    if now - this.m_lastTick > UTPPApproachStaleS() {
      this.Finish("stale", false);
      return;
    }
    this.m_lastTick = now;
    if dt <= 0.0001 {
      return; // paused
    }
    if NotEquals(player.GetResolvedGenderName(), n"Female") && NotEquals(player.GetResolvedGenderName(), n"Male") {
      this.Finish("unknown body", false);
      return;
    }
    if !this.IsEnabled() {
      this.Finish("setting off", false);
      return;
    }
    if (UTPPLocomotionReasons() & UTPPApproachBlockedReasons()) != 0 || !ScriptedPuppet.IsActive(player) {
      this.Finish("player state", false);
      return;
    }
    if player.IsInCombat() {
      this.Finish("combat", false);
      return;
    }
    if StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown") || StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.BikeKnockdown") {
      this.Finish("knockdown", false);
      return;
    }
    let health: Float = GameInstance.GetStatPoolsSystem(game).GetStatPoolValue(Cast<StatsObjectID>(player.GetEntityID()), gamedataStatPoolType.Health, true);
    if health < this.m_health0 - UTPPApproachHealthDropPct() {
      this.Finish("damage", false);
      return;
    }
    if IsDefined(scriptInterface.localBlackboard) && scriptInterface.localBlackboard.GetInt(GetAllBlackboardDefs().PlayerStateMachine.Locomotion) != 0 {
      this.Finish("locomotion state", false);
      return;
    }
    if AbsF(vehicle.GetCurrentSpeed()) > UTPPApproachVehicleSpeedMps() {
      this.Finish("vehicle moving", false);
      return;
    }
    let pose: UTPPApproachPose = UTPPApproachPoseOf(vehicle, player);
    if !pose.ok {
      this.Finish("no pose", false);
      return;
    }
    // reference yaw of the stick: camera yaw while Locomotion drives the movement (camera + stick), the body yaw otherwise
    let locoActive: Bool = UTPPLocomotionActive();
    let refWorld: Float = UTPPYawDegFromForward(player.GetWorldForward());
    if locoActive {
      refWorld = UTPPYawDegFromForward(Transform.GetForward(scriptInterface.GetCameraWorldTransform()));
    }
    let r: Int32 = UTPPApproachStep(pose.px, pose.py, pose.yawDeg, refWorld - pose.carYawDeg, MinF(dt, 0.25), locoActive);
    if r == 0 {
      this.LogTick(now, pose.px, pose.py);
      return;
    }
    // 1 arrived, 2 timeout near, 3 timeout, 4 input, 5 no motion, 6 near spot (facing done, more than the arrival gate from the spot: the slide is short)
    if r == 1 {
      this.m_noMotionStreak = 0;
      this.Finish("arrived", true);
    } else {
      if r == 2 || r == 6 {
        this.m_noMotionStreak = 0;
        this.Finish(r == 6 ? "near_spot" : "timeout_near", true);
      } else {
        if r == 5 {
          this.m_noMotionStreak += 1;
          if this.m_noMotionStreak >= 2 {
            this.m_latchedOff = true;
            this.Finish("no_motion x" + IntToString(this.m_noMotionStreak) + " (approach off for this session)", true);
          } else {
            this.Finish("no_motion (1st; vanilla mount now)", true);
          }
        } else {
          if r == 4 {
            this.Finish("input", false);
          } else {
            this.Finish("timeout", false);
          }
        }
      }
    }
  }

  // Tick log line, at most every 0.1 s of simulation time while the approach runs.
  private func LogTick(now: Float, px: Float, py: Float) -> Void {
    if this.m_logInit && now - this.m_logT < 0.1 {
      return;
    }
    let dpos: Float = 0.0;
    if this.m_logInit {
      dpos = SqrtF((px - this.m_logPx) * (px - this.m_logPx) + (py - this.m_logPy) * (py - this.m_logPy));
    }
    this.m_logInit = true;
    this.m_logT = now;
    this.m_logPx = px;
    this.m_logPy = py;
    UTPPLogLine(UTPPApproachTickInfo(now - this.m_t0, px, py, dpos));
  }

  public func Status() -> String {
    return "enabled=" + UTPPBoolStr(this.m_enabled) + " latchedOff=" + UTPPBoolStr(this.m_latchedOff) + " active=" + UTPPBoolStr(this.m_active)
      + " starts=" + IntToString(this.m_starts) + " mounts=" + IntToString(this.m_mounts) + " aborts=" + IntToString(this.m_aborts)
      + " overrideReady=" + UTPPBoolStr(UTPPLocomotionOverrideReady()) + " last=[" + this.m_last + "]";
  }
}

public func UTPPVehicleApproachStateOf(game: GameInstance) -> ref<UTPPVehicleApproachState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPVehicleApproachState") as UTPPVehicleApproachState;
}

// Game-callable switch (setting vehicle.approachWalk, default on). False = the system is not available yet (no game session): the CET module retries.
public func UTPPSetVehicleApproachEnabled(on: Bool) -> Bool {
  let state: ref<UTPPVehicleApproachState> = UTPPVehicleApproachStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetEnabled(on);
  return true;
}

public func UTPPVehicleApproachStatus() -> String {
  let state: ref<UTPPVehicleApproachState> = UTPPVehicleApproachStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPVehicleApproachState missing";
  }
  return state.Status();
}

// 1.5 s after a dispatched mount: did the player end up mounted?
public class UTPPVehicleApproachMountCheck extends DelayCallback {
  public let m_player: wref<PlayerPuppet>;

  public func Call() -> Void {
    if IsDefined(this.m_player) {
      let mounted: Bool = VehicleComponent.IsMountedToVehicle(this.m_player.GetGame(), this.m_player);
      UTPPLogLine("[utpp_vehicle_approach] mount-check mounted=" + UTPPBoolStr(mounted));
    }
  }
}

// Dispatches the deferred choice (kept by the state, UTPPVehicleApproachState.m_evt) to the vanilla handler. The game's own
// acceptance checks and mount code run (2.5 may differ from 2.31).
@addMethod(gamevehicleVehicleMountableComponent)
public func UTPPApproachMount(state: ref<UTPPVehicleApproachState>, evt: ref<InteractionChoiceEvent>) -> Void {
  if !IsDefined(evt) || !IsDefined(state) {
    return;
  }
  state.SetBypass(true);
  this.OnInteractionChoice(evt);
  state.SetBypass(false);
  let player: ref<PlayerPuppet> = evt.activator as PlayerPuppet;
  let vehicle: ref<VehicleObject> = this.GetEntity() as VehicleObject;
  if IsDefined(player) && IsDefined(vehicle) {
    let cb: ref<UTPPVehicleApproachMountCheck> = new UTPPVehicleApproachMountCheck();
    cb.m_player = player;
    GameInstance.GetDelaySystem(vehicle.GetGame()).DelayCallback(cb, UTPPApproachMountCheckS(), true);
  }
}

// Starts an approach when every condition holds (see the header). False = nothing happened, run the vanilla mount.
@addMethod(gamevehicleVehicleMountableComponent)
public func UTPPApproachTryStart(choiceEvent: ref<InteractionChoiceEvent>) -> Bool {
  let vehicle: ref<VehicleObject> = this.GetEntity() as VehicleObject;
  let player: ref<PlayerPuppet> = choiceEvent.activator as PlayerPuppet;
  if !IsDefined(vehicle) || !IsDefined(player) {
    return false;
  }
  let game: GameInstance = vehicle.GetGame();
  let state: ref<UTPPVehicleApproachState> = UTPPVehicleApproachStateOf(game);
  if !IsDefined(state) {
    return false;
  }
  if state.IsActive() {
    if state.IsStale(game) {
      state.Finish("stale", false); // the locomotion tick stopped (state change): start fresh below
    } else {
      state.Finish("second press (vanilla mount at once)", false); // the new press then runs the vanilla handler
      return false;
    }
  }
  if !state.IsEnabled() || IsDefined(vehicle as BikeObject) || !MountableComponent.IsInteractionAcceptable(choiceEvent) {
    return false;
  }
  if StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown") || StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.BikeKnockdown") {
    return false;
  }
  let record: wref<InteractionMountBase_Record> = InteractionChoiceMetaData.GetTweakData(choiceEvent.choice.choiceMetaData) as InteractionMountBase_Record;
  if !IsDefined(record) {
    return false;
  }
  let slot: CName = record.VehicleMountSlot();
  let anim: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(game);
  if !IsDefined(anim) || !anim.EntryWantsTpp(false, true) {
    return false;
  }
  if player.IsInCombat() && VehicleComponent.IsDriverSlot(slot) {
    return false; // 'enter_fast': vanilla clip
  }
  if !ScriptedPuppet.IsActive(player) || (UTPPLocomotionReasons() & UTPPApproachBlockedReasons()) != 0 {
    return false;
  }
  if VehicleComponent.IsSlotOccupied(game, vehicle.GetEntityID(), slot) || AbsF(vehicle.GetCurrentSpeed()) > UTPPApproachVehicleSpeedMps() {
    return false;
  }
  let bb: ref<IBlackboard> = GameInstance.GetBlackboardSystem(game).GetLocalInstanced(player.GetEntityID(), GetAllBlackboardDefs().PlayerStateMachine);
  if !IsDefined(bb) || bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.Locomotion) != 0 {
    return false;
  }
  let bone: CName = vehicle.GetBoneNameFromSlot(slot);
  let key: CName = vehicle.GetAnimsetOverrideForPassenger(bone);
  let female: Bool = Equals(player.GetResolvedGenderName(), n"Female");
  if !female && NotEquals(player.GetResolvedGenderName(), n"Male") {
    return false;
  }
  let spot: UTPPApproachSpot = UTPPApproachSpotOf(key, female);
  let scene: UTPPApproachScene = UTPPApproachSceneOf(key);
  if !spot.valid || !scene.valid || !UTPPLocomotionOverrideReady() {
    return false;
  }
  let pose: UTPPApproachPose = UTPPApproachPoseOf(vehicle, player);
  if !pose.ok {
    return false;
  }
  UTPPApproachSetScene(scene.boxCx, scene.boxCy, scene.boxHx, scene.boxHy, scene.arcValid, scene.hingeX, scene.hingeY, scene.arcLength, scene.arcClosedDeg, scene.arcOpenDeg);
  let r: Int32 = UTPPApproachBegin(pose.px, pose.py, pose.yawDeg, spot.x, spot.y, spot.yawDeg);
  let from: String = "(" + FloatToStringPrec(pose.px, 2) + "," + FloatToStringPrec(pose.py, 2) + ",yaw " + FloatToStringPrec(pose.yawDeg, 0) + ")";
  let to: String = "(" + FloatToStringPrec(spot.x, 2) + "," + FloatToStringPrec(spot.y, 2) + ",yaw " + FloatToStringPrec(spot.yawDeg, 0) + ")";
  if r != 1 {
    // 0 = already at the spot, -2 no path, -3 too far (> 3 m of walking): the vanilla mount (slide) runs
    state.Note("[utpp_vehicle_approach] skip code=" + IntToString(r) + " seat=" + NameToString(key) + " from=" + from + " to=" + to);
    return false;
  }
  state.Begin(this, choiceEvent, vehicle, player, key, "[utpp_vehicle_approach] start seat=" + NameToString(key) + " body=" + (female ? "female" : "male") + " from=" + from
    + " to=" + to + " dist=" + FloatToStringPrec(SqrtF((pose.px - spot.x) * (pose.px - spot.x) + (pose.py - spot.y) * (pose.py - spot.y)), 2)
    + " " + UTPPApproachInfo() + " locomotion=" + UTPPBoolStr(UTPPLocomotionActive()));
  return true;
}

@wrapMethod(gamevehicleVehicleMountableComponent)
protected cb func OnInteractionChoice(choiceEvent: ref<InteractionChoiceEvent>) -> Bool {
  let state: ref<UTPPVehicleApproachState> = UTPPVehicleApproachStateOf(GetGameInstance());
  if IsDefined(state) && state.TakeBypass() {
    return wrappedMethod(choiceEvent);
  }
  if this.UTPPApproachTryStart(choiceEvent) {
    return true;
  }
  return wrappedMethod(choiceEvent);
}

// One tick per PSM locomotion update, before the PSM reads its input (chained with the utpp_locomotion.reds wrap of the same method).
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if IsDefined(player) {
    let state: ref<UTPPVehicleApproachState> = UTPPVehicleApproachStateOf(player.GetGame());
    if IsDefined(state) && state.IsActive() {
      state.Tick(timeDelta, scriptInterface);
    }
  }
  wrappedMethod(timeDelta, stateContext, scriptInterface);
}

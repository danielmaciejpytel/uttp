// UTPP utpp_bodyturn executor - applies the motion matching body yaw.
// The CET module utpp_bodyturn arms this executor and,
// every frame while Game.UTPPMMBodyTurnActive(), requests the yaw from Game.UTPPMMBodyYawCommand(...) here. The request is
// applied in the next player locomotion update with the game's own transform adjustment (vanilla 2.31):
//   AdjustTransformWithDurations, rotation only (position 0, slide -1) set as the temporary state-context parameter
//   "adjustTransform" - the same request the game uses to turn V to a knockdown source
//   (locomotionTransitions.script RotateToKnockdownSource) and to orient melee lunges (defaultTransition.script
//   RequestPlayerPositionAdjustment).
// While armed and UTPP Locomotion is active (utpp_locomotion.reds) the movement uses the camera heading
// (LocomotionParameters.SetUseCameraHeadingForMovement(true), vanilla state default false), so the capsule keeps running
// along camera + stick while the body turns; the state defaults are restored once when that ends.
// Limits: one rotation request per CET frame (<= 1 frame latency); the capsule/feet yaw is what turns (the MM pose follows
// the measured yaw); a request is dropped when the game already queued its own adjustTransform this frame (melee lunge,
// knockdown) or when it is older than 0.2 s sim time; nothing is applied without UTPP Locomotion (its target is the camera
// heading + raw stick; the vanilla-input target would turn with the body).
// CET: Game.UTPPBodyTurnExecStatus()   (fallback: Game.GetScriptableSystemsContainer():Get("UTPPBodyTurnExec"):Status())
// Game.UTPPLookWorldSet(on, setName) / Game.UTPPLookWorldStatus() - the game look sync (see m_lookOn).
// Requires utpp.reds (UTPPYawDegFromForward) and utpp_locomotion.reds (UTPPLocomotionActive).

public class UTPPBodyTurnExec extends ScriptableSystem {
  private let m_armed: Bool;
  private let m_seq: Int32;
  private let m_appliedSeq: Int32;
  private let m_yaw: Float;
  private let m_reqTime: Float;
  private let m_noCameraHeading: Bool;
  private let m_camHeadingOn: Bool;
  private let m_rotationSeconds: Float;
  private let m_haveYaws: Bool;
  private let m_cameraYaw: Float;
  private let m_bodyYaw: Float;
  private let m_requests: Int32;
  private let m_applied: Int32;
  private let m_stale: Int32;
  private let m_gameAdjust: Int32;
  private let m_skipLocomotion: Int32;
  // aim align request (utpp_bodyturn CET turns V onto the shown camera yaw while aiming; Locomotion is blocked
  // by the aim then, so the Locomotion gate below does not apply to these requests)
  private let m_aimReq: Bool;
  private let m_aimApplied: Int32;
  private let m_camHeadingSets: Int32;
  private let m_camHeadingRestores: Int32;
  // game look sync (utpp_camera camera.lookSync): while on, the player's FPP camera runs the camera
  // parameter set m_lookSet (fppCameraParamSets.tweak VehicleCombat: headingLocked + maintainWorldSpaceHeading, yaw +-180,
  // no rubber band) so the game's look (aim pose, shot, crosshair, Locomotion camera heading = GetCameraWorldTransform)
  // keeps its world heading while this executor turns V. The PSM re-queues its locomotion set on every locomotion state
  // enter (DefaultTransition.UpdateCameraParams); the wrap at the end swaps the plain locomotion sets for ours while on.
  // Off: the last set the game asked for is queued again (Default before the first request).
  private let m_lookOn: Bool;
  private let m_lookSet: CName;
  private let m_gameSet: CName;
  private let m_haveGameSet: Bool;
  private let m_lookSends: Int32;
  private let m_lookSwaps: Int32;
  private let m_lookRestores: Int32;

  // Idempotent (the CET module re-asserts it every frame while Locomotion is active; a new game session starts disarmed).
  // Returns true: the CET module uses the result as the "executor answers" probe.
  public func Arm(on: Bool) -> Bool {
    if NotEquals(this.m_armed, on) {
      this.m_armed = on;
      this.m_appliedSeq = this.m_seq;
    }
    return true;
  }

  public func IsArmed() -> Bool {
    return this.m_armed;
  }

  // World yaw (deg, UTPP convention: forward = (-sin, cos)) for the next locomotion update. False = not armed / invalid.
  public func Request(yawDeg: Float) -> Bool {
    if !this.m_armed || !(yawDeg > -100000.0 && yawDeg < 100000.0) {
      return false;
    }
    this.m_yaw = yawDeg;
    this.m_reqTime = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
    this.m_seq += 1;
    this.m_requests += 1;
    this.m_aimReq = false;
    return true;
  }

  // like Request, applied also while UTPP Locomotion is inactive (aim align); same staleness / game-adjust rules
  public func RequestAim(yawDeg: Float) -> Bool {
    if !this.Request(yawDeg) {
      return false;
    }
    this.m_aimReq = true;
    return true;
  }

  // Camera-heading movement while Locomotion drives the body (default on). Rotation time of each request (s; <= 0 = 1e-6).
  public func Configure(cameraHeading: Bool, rotationSeconds: Float) -> Bool {
    this.m_noCameraHeading = !cameraHeading;
    this.m_rotationSeconds = rotationSeconds;
    return true;
  }

  // Camera yaw seen by the last locomotion update (same source as the MM feed). NaN-free; 0 before the first update.
  public func HeldCameraYaw() -> Float {
    return this.m_cameraYaw;
  }

  public func BodyYaw() -> Float {
    return this.m_bodyYaw;
  }

  public func HaveYaws() -> Bool {
    return this.m_haveYaws;
  }

  public func WantsCameraHeading(game: GameInstance) -> Bool {
    return this.m_armed && !this.m_noCameraHeading && UTPPLocomotionActive();
  }

  public func CameraHeadingApplied() -> Bool {
    return this.m_camHeadingOn;
  }

  public func NoteCameraHeading(on: Bool) -> Void {
    if on {
      if !this.m_camHeadingOn {
        this.m_camHeadingSets += 1;
      }
    } else {
      if this.m_camHeadingOn {
        this.m_camHeadingRestores += 1;
      }
    }
    this.m_camHeadingOn = on;
  }

  public func Tick(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
    let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
    if !IsDefined(player) {
      return;
    }
    this.m_cameraYaw = UTPPYawDegFromForward(Transform.GetForward(scriptInterface.GetCameraWorldTransform()));
    this.m_bodyYaw = UTPPYawDegFromForward(player.GetWorldForward());
    this.m_haveYaws = true;
    if this.m_seq == this.m_appliedSeq {
      return;
    }
    this.m_appliedSeq = this.m_seq;
    let game: GameInstance = player.GetGame();
    if !UTPPLocomotionActive() && !this.m_aimReq {
      this.m_skipLocomotion += 1;
      return;
    }
    let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(game));
    if now - this.m_reqTime > 0.2 || now < this.m_reqTime - 0.01 {
      this.m_stale += 1;
      return;
    }
    if IsDefined(stateContext.GetTemporaryScriptableParameter(n"adjustTransform")) {
      this.m_gameAdjust += 1;
      return;
    }
    let yawRad: Float = Deg2Rad(this.m_yaw);
    let fwd: Vector4 = Vector4(-SinF(yawRad), CosF(yawRad), 0.0, 0.0);
    let seconds: Float = this.m_rotationSeconds;
    if seconds <= 0.0 {
      seconds = 0.000001;
    }
    let req: ref<AdjustTransformWithDurations> = new AdjustTransformWithDurations();
    req.SetPosition(Vector4(0.0, 0.0, 0.0, 0.0));
    req.SetSlideDuration(-1.0);
    req.SetRotation(Quaternion.BuildFromDirectionVector(fwd, Vector4(0.0, 0.0, 1.0, 0.0)));
    req.SetRotationDuration(seconds);
    stateContext.SetTemporaryScriptableParameter(n"adjustTransform", req, true);
    this.m_applied += 1;
    if this.m_aimReq {
      this.m_aimApplied += 1;
    }
  }

  // game look sync on / off (see m_lookOn). True = the event was queued to the player.
  public func LookWorld(on: Bool, setName: CName) -> Bool {
    let player: ref<PlayerPuppet> = GetPlayer(this.GetGameInstance());
    if !IsDefined(player) {
      return false;
    }
    if on && IsNameValid(setName) {
      this.m_lookSet = setName;
    }
    if on && !IsNameValid(this.m_lookSet) {
      return false;
    }
    this.m_lookOn = on;
    let evt: ref<SetCameraParamsEvent> = new SetCameraParamsEvent();
    if on {
      evt.paramsName = this.m_lookSet;
      this.m_lookSends += 1;
    } else {
      evt.paramsName = n"Default";
      if this.m_haveGameSet {
        evt.paramsName = this.m_gameSet;
      }
      this.m_lookRestores += 1;
    }
    player.QueueEvent(evt);
    return true;
  }

  public func LookWorldOn() -> Bool {
    return this.m_lookOn;
  }

  // Called for every camera parameter set the PSM queues: remembered (restored at LookWorld(false)); the plain locomotion
  // sets (heading not locked, or locked without the world heading) become ours while the sync is on.
  public func FilterCameraParams(name: CName) -> CName {
    this.m_gameSet = name;
    this.m_haveGameSet = true;
    if this.m_lookOn && IsNameValid(this.m_lookSet) && UTPPLookSyncSwappable(name) {
      this.m_lookSwaps += 1;
      return this.m_lookSet;
    }
    return name;
  }

  public func LookStatus() -> String {
    let on: String = "false";
    if this.m_lookOn {
      on = "true";
    }
    return "lookOn=" + on + " lookSet=" + NameToString(this.m_lookSet) + " gameSet=" + NameToString(this.m_gameSet)
      + " lookSends=" + IntToString(this.m_lookSends) + " lookSwaps=" + IntToString(this.m_lookSwaps)
      + " lookRestores=" + IntToString(this.m_lookRestores);
  }

  public func Status() -> String {
    let armed: String = "false";
    if this.m_armed {
      armed = "true";
    }
    let heading: String = "false";
    if this.m_camHeadingOn {
      heading = "true";
    }
    return "armed=" + armed + " requests=" + IntToString(this.m_requests) + " applied=" + IntToString(this.m_applied)
      + " stale=" + IntToString(this.m_stale) + " gameAdjust=" + IntToString(this.m_gameAdjust)
      + " skipLocomotion=" + IntToString(this.m_skipLocomotion)
      + " aimApplied=" + IntToString(this.m_aimApplied)
      + " camHeading=" + heading + " camHeadingSets=" + IntToString(this.m_camHeadingSets)
      + " camHeadingRestores=" + IntToString(this.m_camHeadingRestores)
      + " lastYaw=" + FloatToStringPrec(this.m_yaw, 1) + " bodyYaw=" + FloatToStringPrec(this.m_bodyYaw, 1)
      + " cameraYaw=" + FloatToStringPrec(this.m_cameraYaw, 1) + " " + this.LookStatus();
  }
}

// locomotion camera parameter sets replaced by the look sync set while it is on (vanilla
// fppCameraParamSets.tweak: Default / Vault / HoverJump = heading not locked, Slide = locked without the world heading).
// Vehicles, ladders, climbs, workspots, scenes, knockdowns, carrying and consumables keep the game's set.
public func UTPPLookSyncSwappable(name: CName) -> Bool {
  return Equals(name, n"Default") || Equals(name, n"Slide") || Equals(name, n"Vault") || Equals(name, n"HoverJump");
}

public func UTPPBodyTurnExecOf(game: GameInstance) -> ref<UTPPBodyTurnExec> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPBodyTurnExec") as UTPPBodyTurnExec;
}

// ---- CET entry points (Game.<name>(...)); false / 0 when no game session yet ----
public func UTPPBodyTurnExecArm(on: Bool) -> Bool {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  return IsDefined(exec) && exec.Arm(on);
}

public func UTPPBodyTurnExecRequest(yawDeg: Float) -> Bool {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  return IsDefined(exec) && exec.Request(yawDeg);
}

public func UTPPBodyTurnExecRequestAim(yawDeg: Float) -> Bool {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  return IsDefined(exec) && exec.RequestAim(yawDeg);
}

public func UTPPBodyTurnExecConfigure(cameraHeading: Bool, rotationSeconds: Float) -> Bool {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  return IsDefined(exec) && exec.Configure(cameraHeading, rotationSeconds);
}

public func UTPPBodyTurnHeldCameraYaw() -> Float {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  if !IsDefined(exec) || !exec.HaveYaws() {
    return 1000.0;
  }
  return exec.HeldCameraYaw();
}

// game look sync (utpp_camera): Game.UTPPLookWorldSet(true, "VehicleCombat") / (false, "")
public func UTPPLookWorldSet(on: Bool, setName: CName) -> Bool {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  return IsDefined(exec) && exec.LookWorld(on, setName);
}

public func UTPPLookWorldStatus() -> String {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  if !IsDefined(exec) {
    return "UTPPBodyTurnExec missing";
  }
  return exec.LookStatus();
}

public func UTPPBodyTurnExecStatus() -> String {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(GetGameInstance());
  if !IsDefined(exec) {
    return "UTPPBodyTurnExec missing";
  }
  return exec.Status();
}

// Body-turn wrap of LocomotionEventsTransition.OnUpdate; the other UTPP wraps of the same method (utpp.reds feed, utpp_locomotion.reds heartbeat, ...) are chained by redscript.
// Stand / crouch / sprint and the air states reach it through super.OnUpdate.
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(timeDelta, stateContext, scriptInterface);
  // no body turn while a ragdoll episode (utpp_ragdoll.reds) owns the body, including its get-up
  if UTPPRgBusy(scriptInterface.executionOwner as PlayerPuppet) {
    return;
  }
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(scriptInterface.GetGame());
  if !IsDefined(exec) {
    return;
  }
  // Camera-heading movement only on the ground. Air states keep their own parameters (OnEnter already replaced ours with
  // the state defaults, plus e.g. the charged-jump gravity), so leaving into the air only clears the flag.
  let onGround: Bool = scriptInterface.IsOnGround();
  let want: Bool = exec.WantsCameraHeading(scriptInterface.GetGame()) && onGround;
  if !want && !onGround && exec.CameraHeadingApplied() {
    exec.NoteCameraHeading(false);
  }
  if want || exec.CameraHeadingApplied() {
    let params: ref<LocomotionParameters> = new LocomotionParameters();
    this.GetStateDefaultLocomotionParameters(params);
    if want {
      params.SetUseCameraHeadingForMovement(true);
    }
    stateContext.SetTemporaryScriptableParameter(n"locomotionParameters", params, true);
    exec.NoteCameraHeading(want);
  }
  if exec.IsArmed() {
    exec.Tick(stateContext, scriptInterface);
  }
}

// game look sync: every PSM camera parameter request passes UTPPBodyTurnExec.FilterCameraParams
// (remembered for the restore; the plain locomotion sets swapped for the look sync set while it is on).
@wrapMethod(DefaultTransition)
protected final func QueueSetCameraParamsEvent(cameraParams: CName, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let exec: ref<UTPPBodyTurnExec> = UTPPBodyTurnExecOf(scriptInterface.GetGame());
  if IsDefined(exec) {
    wrappedMethod(exec.FilterCameraParams(cameraParams), scriptInterface);
    return;
  }
  wrappedMethod(cameraParams, scriptInterface);
}

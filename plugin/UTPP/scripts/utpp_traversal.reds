// ---- Obstacle reach, drop vault input and jump press diagnostics (climb / vault of the game) ----
//
// The game checks a vault once per jump press and only within minDetectionRange + (detectionRange - minDetectionRange) *
// speed / maxSpeedNormalizer of the front edge (tweaks 0.6 / 2.0 m / 8.5 m/s: 1.26 m at 3.7 m/s); its climb probe reaches about
// climb.detectionRange (1.0 m). A press farther out wins a plain jump and nothing else is tried again.
//  - Obstacle reach (menu option, default off): on a jump press with an obstacle ahead that the probe below finds, the jump is
//    held for at most UTPPObstacleMaxHoldS while V keeps running at it, so the game's OWN climb or vault checks (their native
//    obstacle data, their collision tests, their capsule movement) get evaluated closer to the edge. Nothing is ever forced:
//    when neither check passes, the held jump is released as a normal jump (time out, at the edge, input released, turned
//    away, stopped, obstacle lost). The probe only decides whether holding is worth it.
//  - Drop vault (menu option, default off): the game hands over its vault with the landing clamped to the feet height, so a
//    drop behind the obstacle cannot be read from it. A downward ray behind the obstacle measures the real floor and sends it
//    to the DLL (UTPPFeedTraversalBackFloor), which then selects the vault-and-fall clip when the option is on.
//  - Jump probe log: one "[utpp_jump_probe]" pair of lines per plain jump (what the game's probe reported, what this probe
//    found, why nothing fired) plus one line per hold start / release and per game climb / vault. On with
//    Game.UTPPSetJumpProbeLog(true) or while the ground recorder (diagnostic logs) runs.
// Console: Game.UTPPSetObstacleReach(bool), Game.UTPPSetDropVault(bool), Game.UTPPSetJumpProbeLog(bool),
// Game.UTPPObstacleStatus(). Menu: CET settings traversal.obstacleReach / traversal.dropVault (utpp_motion sends them to every
// new game session); the jump probe log is part of "Diagnostic logs".

// DLL: measured floor behind the vaulted obstacle of the NEXT UTPPFeedTraversal call, metres below the obstacle top at two
// points past the back edge (99 = nothing within the ray, < 0 = not measured; the shallower one counts), and the drop vault
// option.
public native func UTPPFeedTraversalBackFloor(nearM: Float, farM: Float, dropVault: Bool) -> Void

// longest time a jump press is held for the game's climb / vault check
public func UTPPObstacleMaxHoldS() -> Float { return 0.40; }
// front face this close (capsule centre, capsule radius 0.4 m): no more waiting; farther when fast, so the jump still has room
public func UTPPObstacleReleaseDistM(speed: Float) -> Float { return MaxF(0.55, 0.12 * speed); }
// obstacle top above the feet the hold is worth for (higher walls: the jump itself reaches them, the game climbs from the air)
public func UTPPObstacleMinTopM() -> Float { return 0.45; }
public func UTPPObstacleMaxTopM() -> Float { return 1.45; }
// slowest run / walk that is held
public func UTPPObstacleMinSpeed() -> Float { return 1.00; }

// Reach by horizontal speed: 0.75 m standing to 3.0 m at 3.75 m/s, never more than the game's reach plus the held run.
public func UTPPObstacleReachM(speed: Float) -> Float {
  let byRef: Float = LerpF(ClampF(speed / 3.75, 0.0, 1.0), 0.75, 3.0);
  return MinF(byRef, 1.0 + speed * UTPPObstacleMaxHoldS());
}

public class UTPPTraversalState extends ScriptableSystem {
  private let m_reach: Bool;
  private let m_dropVault: Bool;
  private let m_log: Bool;
  private let m_holding: Bool;
  private let m_holdStart: Float;
  private let m_releasedAt: Float;
  private let m_released: Bool;
  private let m_releaseWhy: String;
  private let m_holds: Int32;
  private let m_climbAfterHold: Int32;
  private let m_vaultAfterHold: Int32;
  private let m_jumpAfterHold: Int32;

  public func SetReach(on: Bool) -> Void {
    this.m_reach = on;
    if !on {
      this.m_holding = false;
      this.m_released = false;
    }
  }
  public func SetDropVault(on: Bool) -> Void { this.m_dropVault = on; }
  public func SetLog(on: Bool) -> Void { this.m_log = on; }
  public func ReachOn() -> Bool { return this.m_reach; }
  public func DropVaultOn() -> Bool { return this.m_dropVault; }
  public func LogOn() -> Bool { return this.m_log || UTPPGroundRecorderOn(); }
  public func Holding() -> Bool { return this.m_holding; }
  public func HoldElapsed(now: Float) -> Float { return now - this.m_holdStart; }
  public func StartHold(now: Float) -> Void {
    this.m_holding = true;
    this.m_released = false;
    this.m_holdStart = now;
    this.m_holds += 1;
  }
  public func Release(now: Float, why: String) -> Void {
    this.m_holding = false;
    this.m_released = true;
    this.m_releasedAt = now;
    this.m_releaseWhy = why;
  }
  // Ends a hold or a release that the starting state belongs to: "held 0.18 s" / "released (...)", "" = no hold before it.
  // outcome 0 plain jump, 1 climb, 2 vault.
  public func Take(now: Float, outcome: Int32) -> String {
    let text: String = "";
    if this.m_holding {
      text = "after a hold of " + FloatToStringPrec(now - this.m_holdStart, 2) + " s";
    } else {
      if this.m_released && now - this.m_releasedAt < 0.25 {
        text = "after a released hold (" + this.m_releaseWhy + ", held "
          + FloatToStringPrec(this.m_releasedAt - this.m_holdStart, 2) + " s)";
      }
    }
    if NotEquals(text, "") {
      if outcome == 1 {
        this.m_climbAfterHold += 1;
      } else {
        if outcome == 2 {
          this.m_vaultAfterHold += 1;
        } else {
          this.m_jumpAfterHold += 1;
        }
      }
    }
    this.m_holding = false;
    this.m_released = false;
    return text;
  }
  public func Cancel() -> Void {
    this.m_holding = false;
    this.m_released = false;
  }
  public func Status() -> String {
    return "obstacleReach=" + (this.m_reach ? "1" : "0") + " dropVault=" + (this.m_dropVault ? "1" : "0") + " log="
      + (this.LogOn() ? "1" : "0") + " holds=" + IntToString(this.m_holds) + " climbAfterHold=" + IntToString(this.m_climbAfterHold)
      + " vaultAfterHold=" + IntToString(this.m_vaultAfterHold) + " jumpAfterHold=" + IntToString(this.m_jumpAfterHold);
  }
}

public func UTPPTraversalStateOf(game: GameInstance) -> ref<UTPPTraversalState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPTraversalState") as UTPPTraversalState;
}

// Game-callable. False = the system is not available yet (no game session).
public func UTPPSetObstacleReach(on: Bool) -> Bool {
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetReach(on);
  UTPPLogLine("UTPPSetObstacleReach(" + (on ? "1" : "0") + ")");
  return true;
}

public func UTPPSetDropVault(on: Bool) -> Bool {
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetDropVault(on);
  UTPPLogLine("UTPPSetDropVault(" + (on ? "1" : "0") + ")");
  return true;
}

public func UTPPSetJumpProbeLog(on: Bool) -> Bool {
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetLog(on);
  UTPPLogLine("UTPPSetJumpProbeLog(" + (on ? "1" : "0") + ")");
  return true;
}

public func UTPPObstacleStatus() -> String {
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPTraversalState missing";
  }
  return state.Status();
}

// ---- the probe (static, terrain and dynamic collision only; vehicles stay with the game) ----

public func UTPPObstacleRay(sqs: ref<SpatialQueriesSystem>, from: Vector4, to: Vector4, out hit: TraceResult) -> Bool {
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  QueryFilter.AddGroup(filter, n"Dynamic");
  return sqs.SyncRaycastByQueryFilter(from, to, filter, hit) && TraceResult.IsValid(hit);
}

public func UTPPObstacleFlat(v: Vector4) -> Vector4 {
  let flat: Vector4 = Vector4(v.X, v.Y, 0.0, 0.0);
  if Vector4.Length(flat) < 0.001 {
    return flat;
  }
  return Vector4.Normalize(flat);
}

// Floor under one point past the back edge of an obstacle: metres below the top of the first floor, 99 = nothing within 6 m.
public func UTPPBackFloorSample(sqs: ref<SpatialQueriesSystem>, topZ: Float, backEdge: Vector4, facing: Vector4,
                                ahead: Float) -> Float {
  let hit: TraceResult;
  let x: Float = backEdge.X + facing.X * ahead;
  let y: Float = backEdge.Y + facing.Y * ahead;
  if UTPPObstacleRay(sqs, Vector4(x, y, topZ + 0.1, 1.0), Vector4(x, y, topZ - 6.0, 1.0), hit) {
    return MaxF(topZ - hit.position.Z, 0.0);
  }
  return 99.0;
}

// distances of the two floor samples past the back edge, m (the first one as the reference's back floor trace: radius + 2 cm)
public func UTPPBackFloorNearM() -> Float { return 0.32; }
public func UTPPBackFloorFarM() -> Float { return 0.60; }

// Called by the vault feed (utpp_motion.reds) right before UTPPFeedTraversal.
public func UTPPTraversalSendBackFloor(scriptInterface: ref<StateGameScriptInterface>, info: ref<PlayerClimbInfo>) -> Void {
  let nearDepth: Float = -1.0;
  let farDepth: Float = -1.0;
  let facing: Vector4 = UTPPObstacleFlat(Vector4(-info.descResult.collisionNormal.X, -info.descResult.collisionNormal.Y, 0.0, 0.0));
  if Vector4.Length(facing) > 0.5 {
    let sqs: ref<SpatialQueriesSystem> = scriptInterface.GetSpatialQueriesSystem();
    nearDepth = UTPPBackFloorSample(sqs, info.descResult.topPoint.Z, info.obstacleEnd, facing, UTPPBackFloorNearM());
    farDepth = UTPPBackFloorSample(sqs, info.descResult.topPoint.Z, info.obstacleEnd, facing, UTPPBackFloorFarM());
  }
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(scriptInterface.owner.GetGame());
  UTPPFeedTraversalBackFloor(nearDepth, farDepth, IsDefined(state) && state.DropVaultOn());
}

public class UTPPObstacleHit {
  public let found: Bool;      // an obstacle of a height worth holding for, with room on top of its front edge
  public let dist: Float;      // capsule centre -> front face, m
  public let height: Float;    // top above the feet, m
  public let depth: Float;     // along the probe direction, m (2.2 = at least)
  public let floorDepth: Float; // floor behind, below the top, m (99 none, -1 not measured: deep obstacle)
  public let why: String;      // why found is false ("" when found)
}

public func UTPPProbeObstacle(const scriptInterface: ref<StateGameScriptInterface>, feet: Vector4, dir: Vector4,
                              reach: Float) -> ref<UTPPObstacleHit> {
  let result: ref<UTPPObstacleHit> = new UTPPObstacleHit();
  result.floorDepth = -1.0;
  let sqs: ref<SpatialQueriesSystem> = scriptInterface.GetSpatialQueriesSystem();
  let hit: TraceResult;
  let from: Vector4 = Vector4(feet.X, feet.Y, feet.Z + UTPPObstacleMinTopM(), 1.0);
  let to: Vector4 = Vector4(feet.X + dir.X * reach, feet.Y + dir.Y * reach, from.Z, 1.0);
  if !UTPPObstacleRay(sqs, from, to, hit) {
    result.why = "nothing ahead";
    return result;
  }
  let dx: Float = hit.position.X - feet.X;
  let dy: Float = hit.position.Y - feet.Y;
  result.dist = SqrtF(dx * dx + dy * dy);
  from = Vector4(feet.X, feet.Y, feet.Z + UTPPObstacleMaxTopM() + 0.05, 1.0);
  let reachHigh: Float = result.dist + 0.15;
  to = Vector4(feet.X + dir.X * reachHigh, feet.Y + dir.Y * reachHigh, from.Z, 1.0);
  if UTPPObstacleRay(sqs, from, to, hit) {
    result.why = "too high";
    return result;
  }
  let s: Float = result.dist + 0.08;
  let x: Float = feet.X + dir.X * s;
  let y: Float = feet.Y + dir.Y * s;
  if !UTPPObstacleRay(sqs, Vector4(x, y, feet.Z + UTPPObstacleMaxTopM() + 0.1, 1.0), Vector4(x, y, feet.Z + 0.3, 1.0), hit) {
    result.why = "no top surface";
    return result;
  }
  let topZ: Float = hit.position.Z;
  result.height = topZ - feet.Z;
  if result.height < UTPPObstacleMinTopM() || result.height > UTPPObstacleMaxTopM() {
    result.why = "top height out of range";
    return result;
  }
  // top surface: step along it until it ends (a drop or a rise of more than 0.2 m)
  result.depth = 2.2;
  let k: Int32 = 1;
  while k <= 14 {
    let at: Float = result.dist + 0.08 + 0.15 * Cast<Float>(k);
    x = feet.X + dir.X * at;
    y = feet.Y + dir.Y * at;
    if !UTPPObstacleRay(sqs, Vector4(x, y, topZ + 0.2, 1.0), Vector4(x, y, topZ - 0.2, 1.0), hit) {
      result.depth = at - 0.075 - result.dist;
      k = 15;
    }
    k += 1;
  }
  if result.depth < 2.2 {
    let edge: Float = result.dist + result.depth;
    let backEdge: Vector4 = Vector4(feet.X + dir.X * edge, feet.Y + dir.Y * edge, topZ, 1.0);
    result.floorDepth = MinF(UTPPBackFloorSample(sqs, topZ, backEdge, dir, UTPPBackFloorNearM()),
      UTPPBackFloorSample(sqs, topZ, backEdge, dir, UTPPBackFloorFarM()));
  }
  // the game's own room test above the front edge (ClimbDecisions.OverlapFitTest: capsule radius at the top + radius + 0.15)
  let capsule: Vector4;
  capsule.X = 0.40;
  capsule.Y = -1.0;
  capsule.Z = -1.0;
  let rotation: EulerAngles;
  let overlap: TraceResult;
  let above: Vector4 = Vector4(feet.X + dir.X * (result.dist + 0.05), feet.Y + dir.Y * (result.dist + 0.05), topZ + 0.55, 1.0);
  if scriptInterface.LocomotionOverlapTest(capsule, above, rotation, overlap) {
    result.why = "no room on top";
    return result;
  }
  result.found = true;
  return result;
}

// What the reference rules make of the obstacle (log only).
public func UTPPObstacleKindName(o: ref<UTPPObstacleHit>) -> String {
  if o.depth >= 0.59 {
    return "mantle";
  }
  return o.floorDepth > 1.40 ? "dropVault" : "hurdle";
}

public func UTPPObstacleText(o: ref<UTPPObstacleHit>) -> String {
  return "found=" + (o.found ? "1" : "0") + " dist=" + FloatToStringPrec(o.dist, 2) + " h=" + FloatToStringPrec(o.height, 2)
    + " depth=" + FloatToStringPrec(o.depth, 2) + " floor=" + FloatToStringPrec(o.floorDepth, 2)
    + (o.found ? " kind=" + UTPPObstacleKindName(o) : " why=" + o.why);
}

// Probe direction: the horizontal velocity while moving, else the facing.
public func UTPPObstacleDirection(const scriptInterface: ref<StateGameScriptInterface>) -> Vector4 {
  let velocity: Vector4 = DefaultTransition.GetLinearVelocity(scriptInterface);
  if SqrtF(velocity.X * velocity.X + velocity.Y * velocity.Y) > 0.7 {
    return UTPPObstacleFlat(velocity);
  }
  return UTPPObstacleFlat(scriptInterface.GetOwnerForward());
}

public func UTPPObstacleSpeed(const scriptInterface: ref<StateGameScriptInterface>) -> Float {
  let velocity: Vector4 = DefaultTransition.GetLinearVelocity(scriptInterface);
  return SqrtF(velocity.X * velocity.X + velocity.Y * velocity.Y);
}

public func UTPPObstacleNow(const scriptInterface: ref<StateGameScriptInterface>) -> Float {
  return EngineTime.ToFloat(GameInstance.GetSimTime(scriptInterface.owner.GetGame()));
}

// While a jump press is held, letting the button go (a tap) must not switch the game's vault check off.
@wrapMethod(VaultDecisions)
protected cb func OnAction(action: ListenerAction, consumer: ListenerActionConsumer) -> Bool {
  let result: Bool = wrappedMethod(action, consumer);
  if ListenerAction.IsButtonJustReleased(action) {
    let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(GetGameInstance());
    if IsDefined(state) && state.ReachOn() && state.Holding() {
      this.m_shouldDisableEnterCondition = false;
    }
  }
  return result;
}

// True = keep the game's vault decision evaluating next frame: a hold is running, or this is the press frame of one that may
// start (the order of the two decisions within a frame is not known).
public func UTPPObstacleKeepVaultCheck(const scriptInterface: ref<StateGameScriptInterface>) -> Bool {
  if !IsDefined(scriptInterface) || !IsDefined(scriptInterface.owner) {
    return false;
  }
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(scriptInterface.owner.GetGame());
  return IsDefined(state) && state.ReachOn() && (state.Holding() || scriptInterface.IsActionJustPressed(n"Jump"));
}

// Why a running hold ends in a normal jump this frame ("" = keep holding).
public func UTPPObstacleReleaseReason(const scriptInterface: ref<StateGameScriptInterface>, grounded: Bool, elapsed: Float,
                                      pressedAgain: Bool) -> String {
  if pressedAgain && elapsed > 0.05 { // not the press frame itself
    return "pressed again";
  }
  if elapsed >= UTPPObstacleMaxHoldS() {
    return "time out";
  }
  if !grounded {
    return "left the ground";
  }
  if !scriptInterface.IsMoveInputConsiderable() {
    return "move input released";
  }
  if AbsF(scriptInterface.GetInputHeading()) > 45.0 {
    return "turned away";
  }
  let speed: Float = UTPPObstacleSpeed(scriptInterface);
  if speed < 0.7 {
    return "stopped";
  }
  let ahead: ref<UTPPObstacleHit> = UTPPProbeObstacle(scriptInterface, DefaultTransition.GetPlayerPosition(scriptInterface),
    UTPPObstacleDirection(scriptInterface), UTPPObstacleReachM(speed));
  if !ahead.found {
    return "obstacle lost: " + ahead.why;
  }
  return ahead.dist <= UTPPObstacleReleaseDistM(speed) ? "at the obstacle" : "";
}

@wrapMethod(JumpDecisions)
protected const func EnterCondition(const stateContext: ref<StateContext>, const scriptInterface: ref<StateGameScriptInterface>) -> Bool {
  let jump: Bool = wrappedMethod(stateContext, scriptInterface);
  if NotEquals(this.GetClassName(), n"JumpDecisions") || !IsDefined(scriptInterface) || !IsDefined(scriptInterface.owner) {
    return jump;
  }
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(scriptInterface.owner.GetGame());
  if !IsDefined(state) || !state.ReachOn() {
    return jump;
  }
  let now: Float = UTPPObstacleNow(scriptInterface);
  let pressed: Bool = scriptInterface.IsActionJustPressed(n"Jump");
  if state.Holding() {
    let elapsed: Float = state.HoldElapsed(now);
    if elapsed > UTPPObstacleMaxHoldS() + 0.2 || elapsed < 0.0 {
      // not evaluated for a while (another state took over): that press is gone; a new one is handled below
      state.Cancel();
      if !pressed {
        stateContext.SetConditionBoolParameter(n"JumpPressed", false, true);
        return false;
      }
    } else {
      if !jump {
        // the game does not allow a jump this frame (or a state change cleared the buffered press): ask again until time out
        if elapsed >= UTPPObstacleMaxHoldS() {
          state.Cancel();
          stateContext.SetConditionBoolParameter(n"JumpPressed", false, true);
          if state.LogOn() {
            UTPPLogLine("[utpp_jump_probe] hold dropped after " + FloatToStringPrec(elapsed, 2) + " s: the game allows no jump");
          }
        } else {
          stateContext.SetConditionBoolParameter(n"JumpPressed", true, true);
          this.EnableOnEnterCondition(true);
        }
        return false;
      }
      let why: String = UTPPObstacleReleaseReason(scriptInterface, this.IsTouchingGround(scriptInterface), elapsed, pressed);
      if NotEquals(why, "") {
        state.Release(now, why);
        if state.LogOn() {
          UTPPLogLine("[utpp_jump_probe] hold released after " + FloatToStringPrec(elapsed, 2) + " s: " + why);
        }
        return true;
      }
      // keeps this decision enabled and the game's climb check asking (it reads JumpPressed)
      stateContext.SetConditionBoolParameter(n"JumpPressed", true, true);
      this.EnableOnEnterCondition(true);
      return false;
    }
  }
  if !jump || !pressed {
    return jump;
  }
  if !this.IsTouchingGround(scriptInterface) || scriptInterface.HasStatFlag(gamedataStatType.HasChargeJump)
      || scriptInterface.HasStatFlag(gamedataStatType.HasAirHover) || this.IsVaultingClimbingRestricted(scriptInterface)
      || !MeleeTransition.MeleeUseExplorationCondition(stateContext, scriptInterface)
      || !scriptInterface.IsMoveInputConsiderable() || AbsF(scriptInterface.GetInputHeading()) > 30.0 {
    return jump;
  }
  let v: Float = UTPPObstacleSpeed(scriptInterface);
  if v < UTPPObstacleMinSpeed() {
    return jump;
  }
  let found: ref<UTPPObstacleHit> = UTPPProbeObstacle(scriptInterface, DefaultTransition.GetPlayerPosition(scriptInterface),
    UTPPObstacleDirection(scriptInterface), UTPPObstacleReachM(v));
  if !found.found || found.dist <= UTPPObstacleReleaseDistM(v) {
    return jump;
  }
  state.StartHold(now);
  stateContext.SetConditionBoolParameter(n"JumpPressed", true, true);
  if state.LogOn() {
    UTPPLogLine("[utpp_jump_probe] hold start v=" + FloatToStringPrec(v, 1) + " reach=" + FloatToStringPrec(UTPPObstacleReachM(v), 2)
      + " " + UTPPObstacleText(found));
  }
  return false;
}

// Why the plain jump won: the first of the game's checks that the logged values fail (script order; the native probe's own
// decision and the collision tests are not visible to script).
public func UTPPJumpProbeReason(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>,
                                info: ref<PlayerClimbInfo>, top: Float, reachNow: Float, behind: Float, extent: Float,
                                speed: Float, pitch: Float) -> String {
  if !IsDefined(info) || (!info.vaultValid && !info.climbValid) {
    return "the game reports no climb and no vault here";
  }
  let heading: Float = AbsF(scriptInterface.GetInputHeading());
  if info.climbValid {
    if heading > 30.0 {
      return "climb data: input more than 30 deg off";
    }
    if pitch < -70.0 {
      return "climb data: camera pitch below -70";
    }
    return "climb data: a game collision / angle test or the check order";
  }
  if behind > 0.40 {
    return "vault data: ground behind more than 0.40 m above the feet";
  }
  if extent >= 2.10 {
    return "vault data: deeper than 2.1 m";
  }
  let vanillaReach: Float = 0.6 + 1.4 * MinF(1.0, speed / 8.5) + 0.05;
  if reachNow >= vanillaReach {
    return "vault data: pressed " + FloatToStringPrec(reachNow, 2) + " m out, reach " + FloatToStringPrec(vanillaReach, 2);
  }
  return "vault data: sweep over the top / blocker / input / restriction";
}

@wrapMethod(JumpEvents)
protected func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(scriptInterface.owner.GetGame());
  if !IsDefined(state) {
    return;
  }
  let hold: String = state.Take(UTPPObstacleNow(scriptInterface), 0);
  if NotEquals(hold, "") {
    stateContext.SetConditionBoolParameter(n"JumpPressed", false, true); // a held press must not become a double jump
  }
  if !state.LogOn() {
    return;
  }
  let info: ref<PlayerClimbInfo> = scriptInterface.GetSpatialQueriesSystem().GetPlayerObstacleSystem().GetCurrentClimbInfo(scriptInterface.owner);
  let feet: Vector4 = DefaultTransition.GetPlayerPosition(scriptInterface);
  let speed: Float = UTPPObstacleSpeed(scriptInterface);
  let camera: Transform = scriptInterface.GetCameraWorldTransform();
  let angles: EulerAngles = Transform.ToEulerAngles(camera);
  let pitch: Float = angles.Pitch;
  let top: Float = 0.0;
  let reachNow: Float = 0.0;
  let behind: Float = 0.0;
  let extent: Float = 0.0;
  let flags: String = "info=0";
  if IsDefined(info) {
    flags = "vault=" + (info.vaultValid ? "1" : "0") + " climb=" + (info.climbValid ? "1" : "0");
    if IsDefined(info.descResult) {
      let offset: Vector4 = info.descResult.topPoint - feet;
      top = offset.Z;
      reachNow = AbsF(Vector4.Dot(UTPPObstacleFlat(info.inputDirection), offset));
      behind = info.descResult.behindPoint.Z - feet.Z;
      extent = info.descResult.topExtent;
    }
  }
  UTPPLogLine("[utpp_jump_probe] plain jump " + hold + ": game " + flags + " top=" + FloatToStringPrec(top, 2) + " reach="
    + FloatToStringPrec(reachNow, 2) + " ext=" + FloatToStringPrec(extent, 2) + " behind=" + FloatToStringPrec(behind, 2)
    + " v=" + FloatToStringPrec(speed, 1) + " head=" + FloatToStringPrec(scriptInterface.GetInputHeading(), 0) + " pitch="
    + FloatToStringPrec(pitch, 0) + " feetZ=" + FloatToStringPrec(feet.Z, 2));
  let ahead: ref<UTPPObstacleHit> = UTPPProbeObstacle(scriptInterface, feet, UTPPObstacleDirection(scriptInterface),
    MaxF(UTPPObstacleReachM(speed), 2.0));
  UTPPLogLine("[utpp_jump_probe]   probe " + UTPPObstacleText(ahead) + " | reason: "
    + UTPPJumpProbeReason(stateContext, scriptInterface, info, top, reachNow, behind, extent, speed, pitch));
}

// The game's climb / vault took the press: end the hold and clear the buffered press.
public func UTPPObstacleGameTraversal(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>,
                                      outcome: Int32) -> Void {
  let state: ref<UTPPTraversalState> = UTPPTraversalStateOf(scriptInterface.owner.GetGame());
  if !IsDefined(state) {
    return;
  }
  let hold: String = state.Take(UTPPObstacleNow(scriptInterface), outcome);
  if NotEquals(hold, "") {
    stateContext.SetConditionBoolParameter(n"JumpPressed", false, true);
  }
  if state.LogOn() {
    UTPPLogLine("[utpp_jump_probe] game " + (outcome == 1 ? "climb " : "vault ") + hold);
  }
}

@wrapMethod(ClimbEvents)
public func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  UTPPObstacleGameTraversal(stateContext, scriptInterface, 1);
}

@wrapMethod(VaultEvents)
public func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  UTPPObstacleGameTraversal(stateContext, scriptInterface, 2);
}

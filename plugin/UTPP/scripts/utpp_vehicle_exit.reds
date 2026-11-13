// UTPP vehicle exit: instant "teleport exit" diagnostics and a safe landing position for motorcycles in third person.
//
// Vanilla VehicleEventsTransition.ExitWithTeleport (vehicleTransition.script) unmounts the player instantly and teleports her to
// validUnmountDirection.position. Quick motorcycle exits (unmount about 1.0 s after the exit start, i.e. the direction
// re-check of ExitingEvents.OnUpdate fired the teleport) launched the player into the air in recordings.
//   Step 1 (always on): one log line per teleport exit through UTPPLogLine - direction, target, vehicle and player positions,
//     bike yes/no.
//   Step 2 (default on, motorcycles in third person only): the vanilla target (checked by vehicle.CanUnmount) is kept unless it is
//     clearly off the ground. Ground = downward ray against the Static and Terrain groups at the vanilla XY, accepted only within
//     1.0 m of the bike origin height. Clearly off = more than 0.25 m above or more than 0.10 m below that ground.
//     a) Off: keep the vanilla XY and snap Z to ground + 0.02 m, if a standing box there is free (overlap: Static, Terrain, Dynamic,
//        Vehicle, AI - the groups of the vanilla fit checks).
//     b) Only if a) is not free: the spot beside / behind the bike (side +-1.05 m or back -1.6 m), used only when its ground is
//        found the same way, the standing box is free and a horizontal ray from the bike to it is clear.
//     c) Otherwise, or with no reliable ground, the vanilla position is kept. One log line per decision.
//     NoDirection exits (vanilla teleports 2 m up from the player, not to the target), cars, first person and on foot are unchanged.
// Runtime unverified: the second teleport in the same frame may be overridden by the vanilla one; the step 1/2 log lines show it.
// Not verified in game: that the player capsule is outside the Dynamic / AI groups (otherwise a) is reported as occupied).
// Console / CET: Game.UTPPSetSafeTeleportExit(false) turns step 2 off (step 1 stays), Game.UTPPSafeTeleportExitStatus().
// Requires utpp.reds (UTPPIsTppActive, UTPPLogLine).

public class UTPPVehicleExitState extends ScriptableSystem {
  private let m_safe: Bool = true;
  private let m_moves: Int32;

  public func SetSafe(on: Bool) -> Void {
    this.m_safe = on;
  }

  public func IsSafe() -> Bool {
    return this.m_safe;
  }

  public func CountMove() -> Void {
    this.m_moves += 1;
  }

  public func Moves() -> Int32 {
    return this.m_moves;
  }
}

public func UTPPVehicleExitStateOf(game: GameInstance) -> ref<UTPPVehicleExitState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPVehicleExitState") as UTPPVehicleExitState;
}

// Game-callable. False = the system is not available yet (no game session); nothing stored.
public func UTPPSetSafeTeleportExit(on: Bool) -> Bool {
  let state: ref<UTPPVehicleExitState> = UTPPVehicleExitStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetSafe(on);
  return true;
}

public func UTPPSafeTeleportExitStatus() -> String {
  let state: ref<UTPPVehicleExitState> = UTPPVehicleExitStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPVehicleExitState missing";
  }
  let on: String = "false";
  if state.IsSafe() {
    on = "true";
  }
  return "safeTeleportExit=" + on + " moves=" + IntToString(state.Moves());
}

// jump-out side push. Queues the vanilla locomotion impulse on the player: a PSMImpulse with id 'impulse', the same
// event and fields the game itself queues in the air and on the ground (cyberpunk/devices/jumpPod.script ApplyImpulse,
// cyberpunk/player/player.script OnCarHitPlayer, cyberpunk/player/psm/locomotionTransitions.script KnockdownEvents friction
// impulse). The vector is a velocity change in m/s, world space. Game-callable (utpp_vehiclecam, CET). False = no player or
// the vector is refused (not finite, or longer than 2.0 m/s: a guard against a bad caller, the push is only ever a small nudge).
public func UTPPQueuePlayerImpulse(x: Float, y: Float, z: Float) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  let v: Vector4 = Vector4(x, y, z, 0.0);
  let len: Float = Vector4.Length(v);
  if len != len || len > 2.0 {
    return false;
  }
  let ev: ref<PSMImpulse> = new PSMImpulse();
  ev.id = n"impulse";
  ev.impulse = v;
  player.QueueEvent(ev);
  return true;
}

public func UTPPVehicleExitVec(v: Vector4) -> String {
  return "(" + FloatToStringPrec(v.X, 2) + "," + FloatToStringPrec(v.Y, 2) + "," + FloatToStringPrec(v.Z, 2) + ")";
}

// Safe horizontal position beside / behind the bike; false = direction not handled (Front, Top, NoDirection).
public func UTPPSafeExitXY(vehPos: Vector4, right: Vector4, fwd: Vector4, dir: vehicleExitDirection, out target: Vector4) -> Bool {
  target = vehPos;
  if Equals(dir, vehicleExitDirection.Left) {
    target.X = vehPos.X - right.X * 1.05;
    target.Y = vehPos.Y - right.Y * 1.05;
    return true;
  }
  if Equals(dir, vehicleExitDirection.Right) {
    target.X = vehPos.X + right.X * 1.05;
    target.Y = vehPos.Y + right.Y * 1.05;
    return true;
  }
  if Equals(dir, vehicleExitDirection.Back) {
    target.X = vehPos.X - fwd.X * 1.6;
    target.Y = vehPos.Y - fwd.Y * 1.6;
    return true;
  }
  return false;
}

// Ground height under xy: ray from (bike origin + 1.2 m) or the probe height, whichever is higher, down to 2.5 m below the lower
// of the two, against Static and Terrain (terrain collision is its own group). Accepted only within 1.0 m of the bike origin
// height, so a buried mesh under the terrain or a deep drop is not used. False = no reliable ground.
public func UTPPExitGroundZ(sqs: ref<SpatialQueriesSystem>, xy: Vector4, probeZ: Float, vehZ: Float, out groundZ: Float) -> Bool {
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  let from: Vector4 = Vector4(xy.X, xy.Y, MaxF(probeZ, vehZ) + 1.2, 1.0);
  let to: Vector4 = Vector4(xy.X, xy.Y, MinF(probeZ, vehZ) - 2.5, 1.0);
  let hit: TraceResult;
  if !sqs.SyncRaycastByQueryFilter(from, to, filter, hit) || !TraceResult.IsValid(hit) {
    return false;
  }
  if AbsF(hit.position.Z - vehZ) > 1.0 {
    return false;
  }
  groundZ = hit.position.Z;
  return true;
}

// Free space for V standing at feet: a 0.5 x 0.5 x 1.6 m box from 0.1 m above the feet, against the groups of the vanilla fit
// checks (aiStatusEffectTask, aiFindTeleportPositionForTakedown): Static, Terrain, Dynamic, Vehicle, AI. True = free.
public func UTPPExitSpotFree(sqs: ref<SpatialQueriesSystem>, feet: Vector4, orientation: Quaternion) -> Bool {
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  QueryFilter.AddGroup(filter, n"Dynamic");
  QueryFilter.AddGroup(filter, n"Vehicle");
  QueryFilter.AddGroup(filter, n"AI");
  let extents: Vector4 = Vector4(0.25, 0.25, 0.8, 0.0);
  let center: Vector4 = Vector4(feet.X, feet.Y, feet.Z + 0.1 + 0.8, 1.0);
  let res: TraceResult;
  return !sqs.OverlapByQueryFilter(extents, center, orientation, filter, res);
}

// Clear horizontal path at 0.9 m from the bike origin to a side / back spot (Static, Terrain, Dynamic; the bike's own Vehicle
// collision is skipped because the ray starts inside it). True = clear.
public func UTPPExitPathClear(sqs: ref<SpatialQueriesSystem>, vehPos: Vector4, feet: Vector4) -> Bool {
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  QueryFilter.AddGroup(filter, n"Dynamic");
  let from: Vector4 = Vector4(vehPos.X, vehPos.Y, vehPos.Z + 0.9, 1.0);
  let to: Vector4 = Vector4(feet.X, feet.Y, feet.Z + 0.9, 1.0);
  let hit: TraceResult;
  if sqs.SyncRaycastByQueryFilter(from, to, filter, hit) && TraceResult.IsValid(hit) {
    return false;
  }
  return true;
}

// Step 2 decision. True = teleport to target; reason is the log text in both cases.
public func UTPPSafeExitPick(sqs: ref<SpatialQueriesSystem>, vanilla: Vector4, vehPos: Vector4, right: Vector4, fwd: Vector4,
    orientation: Quaternion, dir: vehicleExitDirection, out target: Vector4, out reason: String) -> Bool {
  let groundZ: Float;
  if !UTPPExitGroundZ(sqs, vanilla, vanilla.Z, vehPos.Z, groundZ) {
    reason = "vanilla position kept (no ground within 1.0 m of the bike under the vanilla target)";
    return false;
  }
  let gap: Float = vanilla.Z - groundZ;
  if gap <= 0.25 && gap >= -0.10 {
    reason = "vanilla position kept (on ground, gap " + FloatToStringPrec(gap, 2) + " m)";
    return false;
  }
  target = vanilla;
  target.Z = groundZ + 0.02;
  if UTPPExitSpotFree(sqs, target, orientation) {
    reason = "z snap " + FloatToStringPrec(-gap, 2) + " m at the vanilla XY";
    return true;
  }
  let side: Vector4;
  let sideZ: Float;
  if UTPPSafeExitXY(vehPos, right, fwd, dir, side) && UTPPExitGroundZ(sqs, side, vehPos.Z, vehPos.Z, sideZ) {
    side.Z = sideZ + 0.02;
    if UTPPExitSpotFree(sqs, side, orientation) && UTPPExitPathClear(sqs, vehPos, side) {
      target = side;
      reason = "vanilla target off ground (gap " + FloatToStringPrec(gap, 2) + " m) and occupied at ground height, " + ToString(dir) + " spot used";
      return true;
    }
  }
  reason = "vanilla position kept (gap " + FloatToStringPrec(gap, 2) + " m, no free spot at ground height)";
  return false;
}

@wrapMethod(VehicleEventsTransition)
protected final func ExitWithTeleport(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>, validUnmountDirection: vehicleUnmountPosition, opt moveVehicle: Bool, opt skipUnmount: Bool) -> Void {
  let vehicle: ref<GameObject> = scriptInterface.owner;
  let player: ref<GameObject> = scriptInterface.executionOwner;
  let game: GameInstance = scriptInterface.GetGame();
  let isBike: Bool = IsDefined(vehicle as BikeObject);
  let dir: vehicleExitDirection = validUnmountDirection.direction;
  let vanilla: Vector4 = WorldPosition.ToVector4(validUnmountDirection.position);
  let vehPos: Vector4;
  let playerPos: Vector4;
  let tpp: Bool = UTPPIsTppActive(game);
  if IsDefined(vehicle) {
    vehPos = vehicle.GetWorldPosition();
  }
  if IsDefined(player) {
    playerPos = player.GetWorldPosition();
  }
  UTPPLogLine("[utpp_vehicle_exit] ExitWithTeleport dir=" + ToString(dir) + " target=" + UTPPVehicleExitVec(vanilla)
    + " vehicle=" + UTPPVehicleExitVec(vehPos) + " player=" + UTPPVehicleExitVec(playerPos)
    + " bike=" + ToString(isBike) + " tpp=" + ToString(tpp) + " moveVehicle=" + ToString(moveVehicle) + " skipUnmount=" + ToString(skipUnmount));
  wrappedMethod(stateContext, scriptInterface, validUnmountDirection, moveVehicle, skipUnmount);

  // Step 2: motorcycles in third person only; a moved vehicle (seat-switch exit) is left to the vanilla teleport.
  // NoDirection: vanilla teleports 2 m above the player, not to the target; left to vanilla.
  if !isBike || !tpp || moveVehicle || !IsDefined(vehicle) || !IsDefined(player) || Equals(dir, vehicleExitDirection.NoDirection) {
    return;
  }
  let state: ref<UTPPVehicleExitState> = UTPPVehicleExitStateOf(game);
  if !IsDefined(state) || !state.IsSafe() {
    return;
  }
  let sqs: ref<SpatialQueriesSystem> = GameInstance.GetSpatialQueriesSystem(game);
  if !IsDefined(sqs) {
    return;
  }
  let orientation: Quaternion = vehicle.GetWorldOrientation();
  let safe: Vector4;
  let reason: String;
  if UTPPSafeExitPick(sqs, vanilla, vehPos, vehicle.GetWorldRight(), vehicle.GetWorldForward(), orientation, dir, safe, reason) {
    GameInstance.GetTeleportationFacility(game).Teleport(player, safe, Quaternion.ToEulerAngles(orientation));
    state.CountMove();
    UTPPLogLine("[utpp_vehicle_exit] safe teleport exit: " + reason + ", moved " + FloatToStringPrec(Vector4.Distance(vanilla, safe), 2)
      + " m to " + UTPPVehicleExitVec(safe));
  } else {
    UTPPLogLine("[utpp_vehicle_exit] safe teleport exit: " + reason);
  }
}

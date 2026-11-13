// UTPP vehicle animation set (motorcycle riding, car enter): third-person motorcycle riding pose from the first seated frame and
// third-person car entry clips.
//
// utpp_vehicle_tpp.archive.xl registers the game's own TPP bike sets (tpp_pwa_sportbikeN / tpp_pma_sportbikeN, by path)
// on the player's root component with the vars [utpp_vehicle_tpp, sportbikeN_vehicle_seat_front_left] at priority 95;
// utpp_vehicle_car.archive overrides the shared car workspot (vehicle_workspot.workspot) with TPP-only 'enter' clips for the standard2,
// standard25, standard3, sport1, sport2 and utility4 player sets (female and male V), vars [utpp_vehicle_tpp, <class>_vehicle_seat_front_left / _right] at priority 127 / 230. (The
// earlier entity-level utpp_vehicle_entry.archive.xl registration at priority 95 is not part of the workspot's clip lookup and never
// played; retire it.)
// This file drives the wrapper variable utpp_vehicle_tpp (AnimWrapperWeightSetter on the player, the same event the game
// uses for its seat variables in VehicleComponent.SetAnimsetOverrideForPassenger):
//   * bike mount (PlayerPuppet.OnMountingEvent, BikeObject parent) while the UTPP third-person signal is on -> 1;
//   * car / other vehicle mount (same event, any other VehicleObject) while the UTPP third-person signal is on -> 1, queued
//     at the mount event itself: the game's own seat variable is set later, at the entry workspot start
//     (ScriptedPuppet.OnWorkspotStartedEvent -> SetAnimsetOverrideForPassenger), so both vars are in place when 'enter'
//     picks its animset. The TPP signal is the on-foot one at that moment (utpp_camera keeps it on through the mount: the
//     CET vehicle hold starts after the event, and the vehicle FPP/TPP decision comes only when seated, then ignored);
//   * Game.UTPPSetVehicleTppAnim(on) from the CET module utpp_perspective while on a bike: follows the ride view
//     (false on a first-person choice, true when third person returns); cars keep the mount-time value for the ride;
//   * unmount (PlayerPuppet.OnUnmountingEvent) or the switch turned off -> 0.
// The value stays 1 through the exit until the unmount, so the exit clips also come from the TPP set (cars: the UTPP car sets
// have no exit clips, so car exits stay vanilla).
// Logging: one line per value per ride ("[utpp_vehicle_anim] ride N: utpp_vehicle_tpp=1 (...)").
// Switch (default on): Game.UTPPSetVehicleTppAnimEnabled(false) turns it off (value 0, vanilla behaviour); status:
// Game.UTPPVehicleTppAnimStatus().
// Runtime unverified: that the priority 95 entry wins over the vanilla 100 / 119 entries (lower number = more specific is
// inferred from the vanilla data) and that several vars on one entry are all required. Bike enter / riding routes (seat pop fix):
// see the SetAnimsetOverrideForPassenger wrap; Game.UTPPSetVehicleBikeRoute(1 | 2 | 3 | 0) selects them.
// On foot nothing changes (the variable is only ever 1 while mounted on a vehicle in third person).
// Requires utpp.reds (UTPPIsTppActive, UTPPLogLine).

public class UTPPVehicleAnimState extends ScriptableSystem {
  private let m_enabled: Bool = true;
  // false = the Game Animations pack is in force: the mod's entry clips are not selected (EntryWantsTpp), the game's own
  // entry clips play; the exit clips come from the game's TPP riding set, which stays; the wrapper weight utpp_vehicle_tpp is still sent (Wanted). Set by CET (utpp_core).
  private let m_entryClips: Bool = true;
  private let m_onBike: Bool;
  private let m_inCar: Bool;
  private let m_value: Float;
  // last CET view request in this ride: 0 none, 1 first person, 2 third person
  private let m_cetView: Int32;
  private let m_rides: Int32;
  private let m_sends: Int32;
  private let m_cetCalls: Int32;
  private let m_loggedOn: Bool;
  private let m_loggedOff: Bool;
  // workspot entry variables (SetAnimsetOverrideForPassenger wrap below): adds of utpp_vehicle_tpp, last list, one log per ride
  private let m_entryAdds: Int32;
  private let m_entryVars: String;
  private let m_loggedEntry: Bool;
  private let m_entryKind: String;
  // bike route variables (see the wrap below): sum of 1 = 'ext' (utpp_bike_route_ext: UTPP enter sets on
  // the own extension component UTPPMotorcycleAnimsets, resource section of utpp_vehicle_bike.archive.xl), 2 = 'root'
  // (utpp_bike_route_root: UTPP enter sets merged into the root component at priority 130, same .xl) and 4 = 'ride'
  // (utpp_bike_route_ride: the game's TPP riding sets, incl. their exit clips, on the same extension component). Default 0 (
  // off): none of these entity routes; the third-person enter comes from the motorcycle workspot override in
  // utpp_vehicle_bike.archive (entries gated by utpp_vehicle_tpp alone). The routes stay selectable for A/B.
  private let m_bikeRoute: Int32 = 0;
  private let m_routeExt: Float;
  private let m_routeRoot: Float;
  private let m_routeRide: Float;

  private func Player() -> ref<PlayerPuppet> {
    return GameInstance.GetPlayerSystem(this.GetGameInstance()).GetLocalPlayerMainGameObject() as PlayerPuppet;
  }

  private func Send(player: ref<GameObject>, on: Bool, why: String) -> Void {
    if !IsDefined(player) {
      return;
    }
    let value: Float = 0.0;
    if on {
      value = 1.0;
    }
    let evt: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
    evt.key = n"utpp_vehicle_tpp";
    evt.value = value;
    player.QueueEvent(evt);
    let changed: Bool = value != this.m_value;
    this.m_value = value;
    this.m_sends += 1;
    if !changed {
      return;
    }
    if on && !this.m_loggedOn {
      this.m_loggedOn = true;
      UTPPLogLine("[utpp_vehicle_anim] ride " + IntToString(this.m_rides) + ": utpp_vehicle_tpp=1 (" + why + ")");
    } else {
      if !on && !this.m_loggedOff {
        this.m_loggedOff = true;
        UTPPLogLine("[utpp_vehicle_anim] ride " + IntToString(this.m_rides) + ": utpp_vehicle_tpp=0 (" + why + ")");
      }
    }
  }

  public func RouteExt() -> Bool {
    return this.m_bikeRoute % 2 == 1;
  }

  public func RouteRoot() -> Bool {
    return (this.m_bikeRoute / 2) % 2 == 1;
  }

  public func RouteRide() -> Bool {
    return (this.m_bikeRoute / 4) % 2 == 1;
  }

  // Route variables as wrapper weights (same event as utpp_vehicle_tpp). on = bike mount in UTPP third person: the selected routes
  // 1, the others 0; off = both 0. Both are always queued (called only at the entry, mount, unmount and a route change; a reloaded
  // player starts with every wrapper weight at 0).
  private func SendRoute(player: ref<GameObject>, on: Bool) -> Void {
    if !IsDefined(player) {
      return;
    }
    let ext: Float = 0.0;
    let root: Float = 0.0;
    if on && this.RouteExt() {
      ext = 1.0;
    }
    if on && this.RouteRoot() {
      root = 1.0;
    }
    let ride: Float = 0.0;
    if on && this.RouteRide() {
      ride = 1.0;
    }
    let e: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
    e.key = n"utpp_bike_route_ext";
    e.value = ext;
    player.QueueEvent(e);
    this.m_routeExt = ext;
    let r: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
    r.key = n"utpp_bike_route_root";
    r.value = root;
    player.QueueEvent(r);
    this.m_routeRoot = root;
    let d: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
    d.key = n"utpp_bike_route_ride";
    d.value = ride;
    player.QueueEvent(d);
    this.m_routeRide = ride;
  }

  // Wanted value while mounted (bike or car): the switch, then the last CET view request (bikes only), else the UTPP
  // third-person signal.
  private func Wanted() -> Bool {
    if !this.m_enabled || !(this.m_onBike || this.m_inCar) {
      return false;
    }
    if this.m_cetView != 0 {
      return this.m_cetView == 2;
    }
    return UTPPIsTppActive(this.GetGameInstance());
  }

  // Workspot entry variables (bike and car): true = utpp_vehicle_tpp joins the animVariables the entering state hands to
  // WorkspotGameSystem.MountToVehicle. Same rule as Wanted() for this ride; also valid before OnMountingEvent of the same mount.
  // isVehicle = the mounted entity is a VehicleObject (bike or car). The CET ride view only exists for bikes; a car entry follows the
  // on-foot UTPP third-person signal (every entry starts in third person; the ride view is decided later and not stored).
  public func EntryWantsTpp(isBike: Bool, isVehicle: Bool) -> Bool {
    if !this.m_enabled || !this.m_entryClips || !isVehicle {
      return false;
    }
    if isBike && this.m_onBike && this.m_cetView != 0 {
      return this.m_cetView == 2;
    }
    return UTPPIsTppActive(this.GetGameInstance());
  }

  // Called by the wrap before OnEntryVars: the vehicle kind for the per-ride log line.
  public func NoteEntryKind(isBike: Bool) -> Void {
    if isBike {
      this.m_entryKind = "bike";
    } else {
      this.m_entryKind = "car";
    }
  }

  // Called by the wrap with the final list: queues the wrapper weight in the same event batch as the vanilla seat variables.
  public func OnEntryVars(player: ref<GameObject>, vars: array<CName>) -> Void {
    this.m_entryAdds += 1;
    let s: String = "";
    let i: Int32 = 0;
    while i < ArraySize(vars) {
      if i > 0 {
        s += ",";
      }
      s += NameToString(vars[i]);
      i += 1;
    }
    this.m_entryVars = s;
    this.Send(player, true, "workspot entry variables");
    this.SendRoute(player, true);
    if !this.m_loggedEntry {
      this.m_loggedEntry = true;
      UTPPLogLine("[utpp_vehicle_anim] ride " + IntToString(this.m_rides) + ": workspot animVariables [" + s + "] (utpp_vehicle_tpp added) kind="
        + this.m_entryKind + " route=" + IntToString(this.m_bikeRoute) + " extComponent=" + UTPPBoolStr(UTPPHasBikeAnimsetComponent(player)));
    }
  }

  public func OnBikeMount(player: ref<GameObject>) -> Void {
    this.m_onBike = true;
    this.m_inCar = false;
    this.m_rides += 1;
    this.m_cetView = 0;
    this.m_loggedOn = false;
    this.m_loggedOff = false;
    if this.Wanted() {
      this.Send(player, true, "bike mount, UTPP third person");
      this.SendRoute(player, true);
    } else {
      if this.m_value != 0.0 {
        this.Send(player, false, "bike mount, not third person");
      }
    }
  }

  // Car or other vehicle: queued immediately in the mount event (no delay), before the entry clip picks its animset.
  public func OnOtherMount(player: ref<GameObject>) -> Void {
    this.m_onBike = false;
    this.m_inCar = true;
    this.m_rides += 1;
    this.m_cetView = 0;
    this.m_loggedOn = false;
    this.m_loggedOff = false;
    if this.Wanted() {
      this.Send(player, true, "car mount, UTPP third person");
    } else {
      if this.m_value != 0.0 {
        this.Send(player, false, "car mount, not third person");
      }
    }
  }

  public func OnUnmount(player: ref<GameObject>) -> Void {
    let wasMounted: Bool = this.m_onBike || this.m_inCar;
    this.m_onBike = false;
    this.m_inCar = false;
    this.m_cetView = 0;
    this.m_loggedEntry = false; // reset here: the entry variables of the next mount may come before its OnMountingEvent
    if wasMounted || this.m_value != 0.0 {
      this.Send(player, false, "unmount");
    }
    if this.m_routeExt != 0.0 || this.m_routeRoot != 0.0 || this.m_routeRide != 0.0 {
      this.SendRoute(player, false);
    }
  }

  // CET (utpp_perspective): the ride view. Ignored when not on a bike. True = handled.
  public func CetView(on: Bool) -> Bool {
    this.m_cetCalls += 1;
    if !this.m_onBike {
      return true;
    }
    if on {
      this.m_cetView = 2;
    } else {
      this.m_cetView = 1;
    }
    let want: Bool = this.Wanted();
    if !Equals(want, this.m_value > 0.5) {
      if want {
        this.Send(this.Player(), true, "third person in the ride");
      } else {
        if this.m_enabled {
          this.Send(this.Player(), false, "first person in the ride");
        } else {
          this.Send(this.Player(), false, "switched off");
        }
      }
    }
    return true;
  }

  public func SetEntryClips(on: Bool) -> Void {
    this.m_entryClips = on;
    UTPPLogLine("[utpp_vehicle_anim] entry clips of the mod: " + UTPPBoolStr(on));
  }

  public func SetEnabled(on: Bool) -> Void {
    this.m_enabled = on;
    let want: Bool = this.Wanted();
    if !Equals(want, this.m_value > 0.5) {
      if want {
        this.Send(this.Player(), true, "switched on while riding");
      } else {
        this.Send(this.Player(), false, "switched off");
      }
    }
  }

  // Route selection (sum of 1 'ext', 2 'root' and 4 'ride'; 0 = the vanilla sets in TPP as before). The enter routes apply to the
  // next bike mount; while riding in third person the weights follow at once.
  public func SetBikeRoute(route: Int32) -> Void {
    let v: Int32 = route;
    if v < 0 || v > 7 {
      v = 0;
    }
    this.m_bikeRoute = v;
    UTPPLogLine("[utpp_vehicle_anim] bike route=" + IntToString(v) + " (ext=" + UTPPBoolStr(this.RouteExt()) + " root="
      + UTPPBoolStr(this.RouteRoot()) + " ride=" + UTPPBoolStr(this.RouteRide()) + ")");
    if this.m_onBike && this.Wanted() {
      this.SendRoute(this.Player(), true);
    }
  }

  public func Status() -> String {
    let en: String = "false";
    if this.m_enabled {
      en = "true";
    }
    let bike: String = "false";
    if this.m_onBike {
      bike = "true";
    }
    let car: String = "false";
    if this.m_inCar {
      car = "true";
    }
    return "enabled=" + en + " onBike=" + bike + " inCar=" + car + " value=" + FloatToString(this.m_value) + " cetView="
      + IntToString(this.m_cetView) + " rides=" + IntToString(this.m_rides) + " sends=" + IntToString(this.m_sends)
      + " cetCalls=" + IntToString(this.m_cetCalls) + " entryVarAdds=" + IntToString(this.m_entryAdds) + " entryVars=["
      + this.m_entryVars + "] bikeRoute=" + IntToString(this.m_bikeRoute) + " routeWeights=" + FloatToString(this.m_routeExt) + "/"
      + FloatToString(this.m_routeRoot) + "/" + FloatToString(this.m_routeRide) + " extComponent=" + UTPPBoolStr(UTPPHasBikeAnimsetComponent(this.Player()));
  }
}

public func UTPPVehicleAnimStateOf(game: GameInstance) -> ref<UTPPVehicleAnimState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPVehicleAnimState") as UTPPVehicleAnimState;
}

// Game-callable (CET utpp_perspective, pcall + nil-safe there). False = the system is not available yet (no game session).
public func UTPPSetVehicleTppAnim(on: Bool) -> Bool {
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  return state.CetView(on);
}

// Switch (default on). False = the system is not available yet.
public func UTPPSetVehicleTppAnimEnabled(on: Bool) -> Bool {
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetEnabled(on);
  return true;
}

// Animation pack (CET utpp_core): true = the mod's vehicle entry / exit clips, false = the game's own (Game Animations). The
// state is per session: CET sends it again after a load. False = the system is not available yet.
public func UTPPSetVehicleEntryClips(on: Bool) -> Bool {
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetEntryClips(on);
  return true;
}

public func UTPPVehicleTppAnimStatus() -> String {
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPVehicleAnimState missing";
  }
  return state.Status();
}

// Bike route (seat pop A/B), sum of: 1 = UTPP enter on the extension component, 2 = UTPP enter on root at priority 130, 4 = TPP
// riding / exit sets on the extension component. Default 0 = none (the workspot override alone). False = the system is not
// available yet.
public func UTPPSetVehicleBikeRoute(route: Int32) -> Bool {
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetBikeRoute(route);
  return true;
}

public func UTPPBoolStr(b: Bool) -> String {
  if b {
    return "1";
  }
  return "0";
}

// True = the ArchiveXL resource patch (utpp_vehicle_bike.archive.xl) added the extension component to this player entity.
public func UTPPHasBikeAnimsetComponent(obj: ref<GameObject>) -> Bool {
  if !IsDefined(obj) {
    return false;
  }
  return obj.UTPPHasComponentNamed(n"UTPPMotorcycleAnimsets");
}

@addMethod(Entity)
public func UTPPHasComponentNamed(name: CName) -> Bool {
  return IsDefined(this.FindComponentByName(name));
}

// Root cause of the FPP bike entry in TPP (the vanilla FPP 'enter_l' played, Hips track identical,
// although utpp_vehicle_tpp=1 had been queued at the MountingEvent 12 ms earlier). The entering state (EnteringEvents.OnEnter) passes
// the list returned here as animVariables to WorkspotGameSystem.MountToVehicle; the workspot picks the entry / exit clips from the
// animation entries whose variableNames are all in that list (vanilla: [<vehicle seat override>, <seat bone>] -> the FPP set with
// [sportbike1_vehicle_seat_front_left]; the vanilla TPP set needs player_tpp_head as well and is never picked for the entry). The
// archive entries [utpp_vehicle_tpp, sportbikeN_vehicle_seat_front_left] (priority 94 enter, 95 riding set) can only match when
// utpp_vehicle_tpp is in the list, so it is added here for a bike mount in UTPP third person, and its wrapper weight is queued in
// the same batch as the vanilla seat variables. Value 0 calls (exit / unmount) and every non-player passenger are unchanged. Cars:
// the same push for every non-bike VehicleObject mount in UTPP third person - the car OccupantSlots workspot
// (vehicle_workspot.workspot, overridden by utpp_vehicle_car.archive) has UTPP
// third-person 'enter' sets [utpp_vehicle_tpp, <class>_vehicle_seat_front_left | _right] at priority 127 / 230 for the classes
// standard2, standard25, standard3, sport1, sport2 and utility4; every vanilla car entry keeps matching because its single seat var is still in the list (an entry needs
// ALL its vars in the list: b72796e), so a class without UTPP clips plays its vanilla FPP enter as before, 'enter_fast'
// (combat entry of a driver) and 'steal' / exits stay vanilla. Log: "[utpp_vehicle_anim] ride N: workspot animVariables [...]";
// status: Game.UTPPVehicleTppAnimStatus() -> "... entryVarAdds=<n> entryVars=[<override>,<bone>,utpp_vehicle_tpp,...]".
// Seat pop fix: with utpp_vehicle_tpp in the list the FPP enter still played in every TPP ride (its end -> TPP
// drive_slow_bike f0 = the logged 'vehtrans seat' hipsStep 9.1 cm), so the root-merged entries are not the ones the entry uses.
// Two enter routes and a riding route, each with its own variable so one game run attributes the result (default 0 = off, route
// bits of Game.UTPPSetVehicleBikeRoute):
//   1 'ext'  utpp_bike_route_ext:  UTPP enter sets (94) on the own extension component UTPPMotorcycleAnimsets
//                                  (resource section of utpp_vehicle_bike.archive.xl, next to the game's MotorcycleAnimsets);
//   2 'root' utpp_bike_route_root: UTPP enter sets merged into the root component at priority 130 (utpp_vehicle_bike.archive.xl);
//   4 'ride' utpp_bike_route_ride: the game's TPP riding sets (95, with their exit clips) on the same extension component.
// The ride's line "workspot animVariables [...] route=N extComponent=1" names the route; "UTPP vehtrans seat: ride=N fires=0" (no
// seat swap step >= 4 cm, the UTPP enter ends on the TPP seat pose) is the pass, "fires=1 hipsStep=9.1cm" the old FPP enter.
// Diagnosis: the seat entry clips come from the vehicle's OccupantSlots workspot (vehicle_motorcycle_workspot.workspot, own per-rig
// animset list), not from the entity sets above. utpp_vehicle_bike.archive overrides that workspot with the UTPP enter sets on the
// player rigs, vars [utpp_vehicle_tpp, sportbikeN_vehicle_seat_front_left], so
// the utpp_vehicle_tpp push below is what selects them; the route variables are off by default.
@wrapMethod(VehicleComponent)
public final static func SetAnimsetOverrideForPassenger(passenger: wref<GameObject>, vehicleID: EntityID, slotName: CName, value: Float) -> array<CName> {
  let vars: array<CName> = wrappedMethod(passenger, vehicleID, slotName, value);
  if value <= 0.0 || ArraySize(vars) == 0 || ArrayContains(vars, n"utpp_vehicle_tpp") {
    return vars;
  }
  let player: ref<PlayerPuppet> = passenger as PlayerPuppet;
  if !IsDefined(player) {
    return vars;
  }
  let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(player.GetGame());
  let vehicle: ref<VehicleObject> = GameInstance.FindEntityByID(player.GetGame(), vehicleID) as VehicleObject;
  let bike: ref<BikeObject> = vehicle as BikeObject;
  if !IsDefined(state) || !state.EntryWantsTpp(IsDefined(bike), IsDefined(vehicle)) {
    return vars;
  }
  state.NoteEntryKind(IsDefined(bike));
  ArrayPush(vars, n"utpp_vehicle_tpp");
  if state.RouteExt() {
    ArrayPush(vars, n"utpp_bike_route_ext");
  }
  if state.RouteRoot() {
    ArrayPush(vars, n"utpp_bike_route_root");
  }
  if state.RouteRide() {
    ArrayPush(vars, n"utpp_bike_route_ride");
  }
  state.OnEntryVars(player, vars);
  return vars;
}

// Chains with the other OnMountingEvent wraps (utpp.reds). Only the player's own mounts (child = this player).
@wrapMethod(PlayerPuppet)
protected cb func OnMountingEvent(evt: ref<MountingEvent>) -> Bool {
  let result: Bool = wrappedMethod(evt);
  if IsDefined(evt) && IsDefined(evt.request) && evt.request.lowLevelMountingInfo.childId == this.GetEntityID() {
    let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(this.GetGame());
    let vehicle: ref<VehicleObject> = GameInstance.FindEntityByID(this.GetGame(), evt.request.lowLevelMountingInfo.parentId) as VehicleObject;
    if IsDefined(state) && IsDefined(vehicle) {
      if IsDefined(vehicle as BikeObject) {
        state.OnBikeMount(this);
      } else {
        state.OnOtherMount(this);
      }
    }
  }
  return result;
}

// ---- Vehicle hood slide (game vault over a parked car): vehicle-only tolerance + optional diagnostic line per attempt ----
//
// The slide over a car is the game's own vault (VaultDecisions / VaultEvents, locomotionTransitions.script; identical in the
// 2.50 scripts); UTPP hands it to the game animation (MotionTraversal "vehicle").
// Observed: 12 traversal starts on vehicles, 11 Climb (action 8) and
// 1 Vault (action 9). The vault decision is evaluated once per jump press (non-leap: EnableOnEnterCondition(false) after the
// first evaluation) and rejects when the jump is pressed more than minDetectionRange (tweak 0.6 m) + speed term from the
// front edge, or when the ground behind is > 0.40 m above the feet; the jump then falls through to Climb (onto the car).
// From the log, 7 of the 11 climbs had the probe's behind point 0.67-0.91 m above the feet (windshield / roof: no slide is
// possible along that line, unchanged here); for the rest the failing check was not logged (the optional line logs it).
// Wrap: vanilla (and other wraps) decide first; only when they reject AND the obstacle is a parked, driverless car AND the
// press is a real jump (no dodge / dash vault, no leap attack) do the same vanilla checks run again with a longer reach at
// low speed and a slightly higher landing tolerance, restricted to a slide ACROSS the car at hood / trunk height. Sweep over
// the top and the VehicleBlocker overlap behind stay exactly as vanilla, so no new path through the body is allowed.
// Successful vanilla cases are unchanged (wrappedMethod true is returned as is, nothing else is computed).
// Not verified in game: the PSM evaluates Vault before Climb on the same frame (the one logged vault happened where a climb
// was also geometrically valid, so vault-first is the working assumption); the diagnostic line shows the result per attempt.
// Switches: Game.UTPPSetVehicleVaultAssist(false) = vanilla only (default on); Game.UTPPSetVehicleVaultLog(true) = one
// "[utpp_vehicle_vault]" line per attempt near a car (default OFF, release-quiet; test harness / console turn it on);
// status: Game.UTPPVehicleVaultAssistStatus().

// reach at standstill (vanilla tweak playerStateMachineLocomotion.vault.minDetectionRange = 0.6 m); the speed term above it
// is the vanilla one (detectionRange 2.0 m at maxSpeedNormalizer 8.5 m/s), so at a sprint the reach is almost unchanged
public func UTPPVehVaultMinReachM() -> Float { return 1.00; }
// landing tolerance: ground behind the car at most this much above the feet. Vanilla FitTest allows 0.40 m; the landing
// itself is native (VaultEvents only passes Destination Z = max(behindPoint.Z, feet Z), no script-side foot handling), so
// how much more the animation absorbs is unproven: small margin for slopes beside cars only
public func UTPPVehVaultMaxBehindRiseM() -> Float { return 0.48; }
// slide only over hood / trunk height (front edge above the feet); higher = roof / van / truck -> vanilla only
public func UTPPVehVaultMinTopM() -> Float { return 0.55; }
public func UTPPVehVaultMaxTopM() -> Float { return 1.20; }
// slide across the car: |cos| between the input direction and the vehicle's right axis (0.64 = within 50 deg)
public func UTPPVehVaultMinCrossCos() -> Float { return 0.64; }
// parked or rolling slowly only
public func UTPPVehVaultMaxVehicleSpeed() -> Float { return 1.00; }
// at most one diagnostic line per this many seconds (leap-attack vaults evaluate every frame)
public func UTPPVehVaultLogGapS() -> Float { return 0.25; }

public class UTPPVehicleVaultState extends ScriptableSystem {
  private let m_assist: Bool = true;
  private let m_log: Bool;
  private let m_vehicleEvals: Int32;
  private let m_vanillaVaults: Int32;
  private let m_assistVaults: Int32;
  private let m_lastLog: Float;
  private let m_logged: Bool;

  public func SetAssist(on: Bool) -> Void {
    this.m_assist = on;
  }

  public func IsAssist() -> Bool {
    return this.m_assist;
  }

  public func SetLog(on: Bool) -> Void {
    this.m_log = on;
  }

  public func IsLog() -> Bool {
    return this.m_log;
  }

  public func Count(vanilla: Bool, assist: Bool) -> Void {
    this.m_vehicleEvals += 1;
    if vanilla {
      this.m_vanillaVaults += 1;
    }
    if assist {
      this.m_assistVaults += 1;
    }
  }

  // True = a line may be written now (logging on and not throttled); force skips the throttle (vault results).
  public func MayLog(now: Float, force: Bool) -> Bool {
    if !this.m_log {
      return false;
    }
    if !force && this.m_logged && now - this.m_lastLog < UTPPVehVaultLogGapS() {
      return false;
    }
    this.m_logged = true;
    this.m_lastLog = now;
    return true;
  }

  public func Status() -> String {
    let on: String = "false";
    if this.m_assist {
      on = "true";
    }
    let log: String = "false";
    if this.m_log {
      log = "true";
    }
    return "vehicleVaultAssist=" + on + " log=" + log + " vehicleEvals=" + IntToString(this.m_vehicleEvals) + " vanillaVaults="
      + IntToString(this.m_vanillaVaults) + " assistVaults=" + IntToString(this.m_assistVaults);
  }
}

public func UTPPVehicleVaultStateOf(game: GameInstance) -> ref<UTPPVehicleVaultState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPVehicleVaultState") as UTPPVehicleVaultState;
}

// Game-callable. False = the system is not available yet (no game session).
public func UTPPSetVehicleVaultAssist(on: Bool) -> Bool {
  let state: ref<UTPPVehicleVaultState> = UTPPVehicleVaultStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetAssist(on);
  return true;
}

// Game-callable diagnostics switch (default off). False = the system is not available yet.
public func UTPPSetVehicleVaultLog(on: Bool) -> Bool {
  let state: ref<UTPPVehicleVaultState> = UTPPVehicleVaultStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetLog(on);
  // one echo line per call: the test harness confirms each typed command by its '<Name>(' echo in the UTPP log
  UTPPLogLine("UTPPSetVehicleVaultLog(" + (on ? "1" : "0") + ")");
  return true;
}

public func UTPPVehicleVaultAssistStatus() -> String {
  let state: ref<UTPPVehicleVaultState> = UTPPVehicleVaultStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "UTPPVehicleVaultState missing";
  }
  return state.Status();
}

// The car under the obstacle probe, or null (static geometry, NPC, motorcycle).
public func UTPPVehVaultCar(info: ref<PlayerClimbInfo>) -> ref<VehicleObject> {
  if !IsDefined(info) || !IsDefined(info.descResult) {
    return null;
  }
  let climbed: ref<Entity> = info.descResult.climbedEntity;
  if !IsDefined(climbed) {
    return null;
  }
  let veh: ref<VehicleObject> = climbed as VehicleObject;
  if !IsDefined(veh) || IsDefined(veh as BikeObject) {
    return null;
  }
  return veh;
}

public func UTPPVehVaultFlat(v: Vector4) -> Vector4 {
  let r: Vector4 = new Vector4(v.X, v.Y, 0.0, 0.0);
  let l: Float = SqrtF(r.X * r.X + r.Y * r.Y);
  if l < 0.0001 {
    return new Vector4(0.0, 0.0, 0.0, 0.0);
  }
  return new Vector4(r.X / l, r.Y / l, 0.0, 0.0);
}

// Diagnostic tail of a log line; built only when a line is actually written.
public func UTPPVehVaultFacts(info: ref<PlayerClimbInfo>, top: Float, behind: Float, reach: Float, vanillaReach: Float,
    assistReach: Float, speed: Float, cross: Float, carSpeed: Float, driver: Bool) -> String {
  return " valid=" + ToString(info.vaultValid) + "/" + ToString(info.climbValid) + " top=" + FloatToStringPrec(top, 2)
    + " ext=" + FloatToStringPrec(info.descResult.topExtent, 2) + " behind=" + FloatToStringPrec(behind, 2)
    + " reach=" + FloatToStringPrec(reach, 2) + " max=" + FloatToStringPrec(vanillaReach, 2) + "/" + FloatToStringPrec(assistReach, 2)
    + " v=" + FloatToStringPrec(speed, 1) + " cross=" + FloatToStringPrec(cross, 2) + " car=" + FloatToStringPrec(carSpeed, 1)
    + " driver=" + ToString(driver);
}

// A dodge / dash input is held: the dash vault (Reflexes perk) is vanilla's own path, never assisted.
public func UTPPVehVaultDodgeHeld(si: ref<StateGameScriptInterface>) -> Bool {
  return si.GetActionValue(n"Dodge") > 0.00 || si.GetActionValue(n"DodgeDirection") > 0.00 || si.GetActionValue(n"DodgeForward") > 0.00
    || si.GetActionValue(n"DodgeRight") > 0.00 || si.GetActionValue(n"DodgeLeft") > 0.00 || si.GetActionValue(n"DodgeBack") > 0.00;
}

@wrapMethod(VaultDecisions)
protected const func EnterCondition(const stateContext: ref<StateContext>, const scriptInterface: ref<StateGameScriptInterface>) -> Bool {
  // read before vanilla: the latch (OnAction) is what allowed this evaluation; it is also cleared by dodge presses
  let latched: Bool = !this.m_shouldDisableEnterCondition;
  let result: Bool = wrappedMethod(stateContext, scriptInterface);
  if !IsDefined(scriptInterface) || !IsDefined(scriptInterface.owner) {
    return result;
  }
  let game: GameInstance = scriptInterface.owner.GetGame();
  let state: ref<UTPPVehicleVaultState> = UTPPVehicleVaultStateOf(game);
  if !IsDefined(state) || (result && !state.IsLog()) || (!result && !state.IsAssist() && !state.IsLog()) {
    return result; // nothing to assist and nothing to log: no extra work
  }
  let info: ref<PlayerClimbInfo> = scriptInterface.GetSpatialQueriesSystem().GetPlayerObstacleSystem().GetCurrentClimbInfo(scriptInterface.owner);
  let car: ref<VehicleObject> = UTPPVehVaultCar(info);
  if !IsDefined(car) {
    return result; // not a car: vanilla only, no log
  }
  let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(game));
  let feet: Vector4 = DefaultTransition.GetPlayerPosition(scriptInterface);
  let d: ref<GeometryDescriptionResult> = info.descResult;
  let top: Float = d.topPoint.Z - feet.Z;
  let behind: Float = d.behindPoint.Z - feet.Z;
  let dir: Vector4 = UTPPVehVaultFlat(info.inputDirection);
  let reach: Float = AbsF(Vector4.Dot(dir, d.topPoint - feet));
  let speed: Float = Vector4.Length(DefaultTransition.GetLinearVelocity(scriptInterface));
  let norm: Float = MinF(1.0, speed / this.GetStaticFloatParameterDefault("maxSpeedNormalizer", 8.50));
  let minRange: Float = this.GetStaticFloatParameterDefault("minDetectionRange", 0.40);
  let range: Float = this.GetStaticFloatParameterDefault("detectionRange", 2.00);
  let vanillaReach: Float = minRange + (range - minRange) * norm + 0.05;
  let assistMin: Float = MaxF(minRange, UTPPVehVaultMinReachM());
  let assistReach: Float = assistMin + (range - assistMin) * norm + 0.05;
  let cross: Float = AbsF(Vector4.Dot(dir, UTPPVehVaultFlat(car.GetWorldRight())));
  let carSpeed: Float = AbsF(car.GetCurrentSpeed());
  let driver: Bool = IsDefined(VehicleComponent.GetDriverMounted(game, car.GetEntityID()));
  if result {
    state.Count(true, false);
    if state.MayLog(now, true) {
      UTPPLogLine("[utpp_vehicle_vault] vanilla vault over a car"
        + UTPPVehVaultFacts(info, top, behind, reach, vanillaReach, assistReach, speed, cross, carSpeed, driver));
    }
    return true;
  }
  let assist: Bool = false;
  let veto: String = "";
  if !state.IsAssist() {
    veto = "assist off";
  } else {
    if stateContext.GetBoolParameter(n"enableVaultFromleapAttack", true) {
      veto = "leap attack (vanilla only)";
    } else {
      // a real jump only: the latch alone is also cleared by dodge presses (dash vault keeps the vanilla 0.6 m reach)
      if !latched || !(scriptInterface.GetActionValue(n"Jump") > 0.00 || scriptInterface.IsActionJustPressed(n"Jump"))
          || UTPPVehVaultDodgeHeld(scriptInterface) || !this.IsTouchingGround(scriptInterface)
          || this.IsVaultingClimbingRestricted(scriptInterface)
          || !MeleeTransition.MeleeUseExplorationCondition(stateContext, scriptInterface)
          || AbsF(scriptInterface.GetInputHeading()) > this.GetStaticFloatParameterDefault("enterAngleThreshold", -180.00) {
        veto = "no jump / dodge / not on ground / restricted / input";
      } else {
        if !info.vaultValid {
          veto = "no game vault data";
        } else {
          if carSpeed > UTPPVehVaultMaxVehicleSpeed() || driver {
            veto = "car moving or with a driver";
          } else {
            if top < UTPPVehVaultMinTopM() || top > UTPPVehVaultMaxTopM() {
              veto = "not hood / trunk height";
            } else {
              if cross < UTPPVehVaultMinCrossCos() {
                veto = "not across the car";
              } else {
                if behind > UTPPVehVaultMaxBehindRiseM() {
                  veto = "ground behind too high";
                } else {
                  if d.topExtent >= this.GetStaticFloatParameterDefault("maxExtent", 2.10) || reach >= assistReach {
                    veto = "too deep / too far";
                  } else {
                    // vanilla FitTest without the 0.40 m landing limit (checked above with the vehicle tolerance)
                    let capsule: Vector4;
                    capsule.X = this.GetStaticFloatParameterDefault("capsuleRadius", 0.40);
                    capsule.Y = -1.00;
                    capsule.Z = -1.00;
                    let rotation: EulerAngles;
                    let sweep: TraceResult;
                    let query: Vector4 = d.topPoint + DefaultTransition.GetUpVector() * (capsule.X + 0.01);
                    if scriptInterface.LocomotionSweepTest(capsule, query, rotation, info.inputDirection, d.topExtent, false, sweep) {
                      veto = "body would pass through the car (sweep)";
                    } else {
                      capsule.X = 0.10;
                      let over: Vector4 = d.behindPoint;
                      over.Z += 0.40;
                      let blocker: TraceResult;
                      if scriptInterface.OverlapWithASingleGroup(capsule, over, rotation, n"VehicleBlocker", blocker) {
                        veto = "vehicle blocker behind";
                      } else {
                        assist = true;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  state.Count(false, assist);
  if state.MayLog(now, assist) {
    // which vanilla check rejected (same order as VaultDecisions.EnterCondition)
    let why: String = "sweep over the top / blocker behind / input / restriction";
    if !info.vaultValid {
      why = "vaultValid=false (game obstacle probe)";
    } else {
      if behind > 0.40 {
        why = "ground behind too high (> 0.40 m)";
      } else {
        if d.topExtent >= this.GetStaticFloatParameterDefault("maxExtent", 2.10) {
          why = "too deep";
        } else {
          if reach >= vanillaReach {
            why = "jump pressed too far from the edge";
          }
        }
      }
    }
    let facts: String = UTPPVehVaultFacts(info, top, behind, reach, vanillaReach, assistReach, speed, cross, carSpeed, driver);
    if assist {
      UTPPLogLine("[utpp_vehicle_vault] assisted vault over a car (vanilla: " + why + ")" + facts);
    } else {
      UTPPLogLine("[utpp_vehicle_vault] no vault over a car (vanilla: " + why + "; assist: " + veto + ")" + facts);
    }
  }
  if assist {
    this.EnableOnEnterCondition(false); // one evaluation per press, as vanilla after its own success
    return true;
  }
  return false;
}

@wrapMethod(PlayerPuppet)
protected cb func OnUnmountingEvent(evt: ref<UnmountingEvent>) -> Bool {
  let result: Bool = wrappedMethod(evt);
  if IsDefined(evt) && IsDefined(evt.request) && evt.request.lowLevelMountingInfo.childId == this.GetEntityID() {
    let state: ref<UTPPVehicleAnimState> = UTPPVehicleAnimStateOf(this.GetGame());
    if IsDefined(state) {
      state.OnUnmount(this);
    }
  }
  return result;
}

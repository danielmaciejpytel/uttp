// UTPP utpp_body bridge - TPP representation events and the TppHead slot state for CET utpp_body.
// Why: CET cannot create the representation events under any name ("cannot create
// gameActivateTPPRepresentationEvent (tried gameActivateTPPRepresentationEvent.new(), NewObject(...), ...,
// ActivateTPPRepresentationEvent.new(), ...)"), while redscript constructs them like vanilla does
// (takeOverControlSystem.script EnablePlayerTPPRepresenation: player.QueueEvent(new ActivateTPPRepresentationEvent())).
// Called from CET: Game.UTPPQueueTppRepresentation(true|false), Game.UTPPTppHeadSlotState(); the same through the
// scriptable system for a CET build that does not resolve script globals:
// Game.GetScriptableSystemsContainer():Get("UTPPBodyBridge"):QueueTppRepresentation(true).
// The calls above keep no state. The exit keep below keeps a short latch and the counters on the player.
// Compile-time dependency: Codeware's import gameAnimFeature_TPPRepresentation (UTPPApplyTppRepFeature only);
// utpp_itemplane.reds (UTPPItemPlaneOnRepresentation; always shipped together).

// Queues ActivateTPPRepresentationEvent (on = true) or DeactivateTPPRepresentationEvent (on = false) on the player.
// false = no player. Deactivation follows vanilla: never while the mounted vehicle's TPP camera owns the representation.
public static func UTPPQueueTppRepresentation(on: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  if on {
    player.QueueEvent(new ActivateTPPRepresentationEvent());
    return true;
  }
  let vehicle: wref<VehicleObject>;
  if VehicleComponent.GetVehicle(player.GetGame(), player, vehicle) && IsDefined(vehicle) {
    if vehicle.GetCameraManager().IsTPPActive() {
      return false;
    }
  }
  player.utppRepOwnDeact += 1;
  player.utppRepOwnDeactAt = UTPPRepNow(player);
  player.QueueEvent(new DeactivateTPPRepresentationEvent());
  return true;
}

// Anim wrapper variable utpp_foot_tpp on the player. 1 = UTPP's third-person body is wanted (set by CET utpp_body,
// which repeats the value; a reloaded player starts at 0), 0 = it is not, or the game has taken over (security camera).
// archive\utpp_foot_tpp.archive.xl and player_locomotion.animgraph read it: with it set, the locomotion set outranks the
// security-camera set that would otherwise replace the standing and crouch idles with the hacking crouch. Same event the game
// uses for its own seat variables (AnimWrapperWeightSetter), as utpp_vehicle_anim.reds does for utpp_vehicle_tpp.
// false = no player.
// the last value is kept on the player (utppFootTppWanted) for the melee full body (utpp_melee.reds): the UTPP
// melee clips are registered behind the same variable, so the legs may follow the game's melee pose only while it is set.
@addField(PlayerPuppet) public let utppFootTppWanted: Bool;

public static func UTPPSetFootTpp(on: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  player.utppFootTppWanted = on;
  let evt: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
  evt.key = n"utpp_foot_tpp";
  if on {
    evt.value = 1.0;
  } else {
    evt.value = 0.0;
  }
  player.QueueEvent(evt);
  return true;
}

// Anim wrapper variable utpp_melee_tpp on the player: the gate of the UTPP melee clips (archive\utpp_melee_tpp.archive.xl). CET
// utpp_body sends 1 while UTPP's third-person body is wanted on foot (the value of utpp_foot_tpp) and the mod's animations are
// the animation pack in force, 0 with the Game Animations pack, so the game plays its own melee animations then. A reloaded
// player starts at 0; CET repeats the value. false = no player.
public static func UTPPSetMeleeTpp(on: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  let evt: ref<AnimWrapperWeightSetter> = new AnimWrapperWeightSetter();
  evt.key = n"utpp_melee_tpp";
  if on {
    evt.value = 1.0;
  } else {
    evt.value = 0.0;
  }
  player.QueueEvent(evt);
  return true;
}

// A short message in the middle of the screen (the game's warning message: the blackboard variable UI_Notifications.WarningMessage,
// the same one the game's own warning effector writes). Called from CET utpp_core for the diagnostic logs. false = no game session.
public static func UTPPShowMessage(text: String, seconds: Float) -> Bool {
  let game: GameInstance = GetGameInstance();
  let system: ref<BlackboardSystem> = GameInstance.GetBlackboardSystem(game);
  if !IsDefined(system) {
    return false;
  }
  let defs: ref<AllBlackboardDefinitions> = GetAllBlackboardDefs();
  let board: ref<IBlackboard> = system.Get(defs.UI_Notifications);
  if !IsDefined(board) {
    return false;
  }
  let msg: SimpleScreenMessage;
  msg.isShown = true;
  msg.duration = seconds;
  msg.message = text;
  board.SetVariant(defs.UI_Notifications.WarningMessage, ToVariant(msg), true);
  return true;
}

// ---- exit keep of the TPP representation (bike exit flash) ----
// Every bike exit with the UTPP third-person body: the TppHead slot goes tpp -> empty 0.47 s after the exit start, the
// head is gone for ~0.1 s and the vehicle graph shows its first-person branch for 6 frames (Switch on
// TPPRepresentation.IsActive). No script sends that deactivation (the only script sender of
// DeactivateTPPRepresentationEvent is TakeOverControlSystem); the drop comes 0.45-0.47 s after the vehicle camera's
// active perspective turns FPP (2-wheel camera blend 0.45 s), also when that turn is 0.2 s late (car exit delay).
// While CET utpp_body holds the latch (UTPPSetTppRepKeep, exit window, only while UTPP wants the TPP body):
//   - a deactivation that reaches the player is answered in the same dispatch with ActivateTPPRepresentationEvent and the
//     anim feature TPPRepresentation.IsActive = true;
//   - FinalizeDeactivationTPPRepresentationEvent re-applies the feature.
// Without the latch the handlers only count (diagnostics). A script handler cannot cancel the native handling; whether
// the native drop reaches the player as an event at all is unknown - UTPPTppRepKeepDiag reports what arrived.
@addField(PlayerPuppet)
public let utppRepKeepUntil: Float;

@addField(PlayerPuppet)
public let utppRepOwnDeact: Int32;

@addField(PlayerPuppet)
public let utppRepOwnDeactAt: Float;

@addField(PlayerPuppet)
public let utppRepGameDeact: Int32;

@addField(PlayerPuppet)
public let utppRepKept: Int32;

@addField(PlayerPuppet)
public let utppRepLastDeactAt: Float;

@addField(PlayerPuppet)
public let utppRepFinalDeact: Int32;

@addField(PlayerPuppet)
public let utppRepFinalAct: Int32;

public static func UTPPRepNow(player: ref<PlayerPuppet>) -> Float {
  return EngineTime.ToFloat(GameInstance.GetSimTime(player.GetGame()));
}

public static func UTPPApplyTppRepFeatureOn(player: ref<PlayerPuppet>, on: Bool) -> Void {
  let f: ref<gameAnimFeature_TPPRepresentation> = new gameAnimFeature_TPPRepresentation();
  f.IsActive = on;
  AnimationControllerComponent.ApplyFeature(player, n"TPPRepresentation", f);
}

// Anim feature TPPRepresentation.IsActive on the player. false = no player.
public static func UTPPApplyTppRepFeature(on: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  UTPPApplyTppRepFeatureOn(player, on);
  return true;
}

// Latch for seconds s (on = false or seconds <= 0 clears it). The caller refreshes it while the window is open, so a
// stopped caller cannot leave it set. false = no player.
public static func UTPPSetTppRepKeep(on: Bool, seconds: Float) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  if on && seconds > 0.0 {
    player.utppRepKeepUntil = UTPPRepNow(player) + MinF(seconds, 5.0);
  } else {
    player.utppRepKeepUntil = 0.0;
  }
  return true;
}

// "game=<deactivations not sent by UTPP> kept=<answered under the latch> own=<UTPP's own> finalDeact=<n> finalAct=<n>
// last=<s since the last game deactivation, -1 = none> keep=<s left on the latch, 0 = off>" | "noplayer"
public static func UTPPTppRepKeepDiag() -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return "noplayer";
  }
  let now: Float = UTPPRepNow(player);
  let last: Float = -1.0;
  if player.utppRepGameDeact > 0 {
    last = now - player.utppRepLastDeactAt;
  }
  return "game=" + IntToString(player.utppRepGameDeact) + " kept=" + IntToString(player.utppRepKept)
    + " own=" + IntToString(player.utppRepOwnDeact) + " finalDeact=" + IntToString(player.utppRepFinalDeact)
    + " finalAct=" + IntToString(player.utppRepFinalAct) + " last=" + FloatToStringPrec(last, 3)
    + " keep=" + FloatToStringPrec(MaxF(player.utppRepKeepUntil - now, 0.0), 2);
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPDeactivateTppRepresentation(evt: ref<DeactivateTPPRepresentationEvent>) -> Bool {
  let now: Float = UTPPRepNow(this);
  if this.utppRepOwnDeact > 0 && now - this.utppRepOwnDeactAt < 1.0 {
    this.utppRepOwnDeact -= 1;
    return false;
  }
  this.utppRepOwnDeact = 0;
  this.utppRepGameDeact += 1;
  this.utppRepLastDeactAt = now;
  if now < this.utppRepKeepUntil {
    this.utppRepKept += 1;
    this.QueueEvent(new ActivateTPPRepresentationEvent());
    UTPPApplyTppRepFeatureOn(this, true);
  }
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPFinalizeDeactivationTppRepresentation(evt: ref<FinalizeDeactivationTPPRepresentationEvent>) -> Bool {
  this.utppRepFinalDeact += 1;
  if UTPPRepNow(this) < this.utppRepKeepUntil {
    UTPPApplyTppRepFeatureOn(this, true);
  }
  // render-plane policy (utpp_itemplane.reds): sync, then re-apply to the registered plane-bound parts.
  UTPPItemPlaneOnRepresentation(this.GetGame(), "rep-deactivated");
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPFinalizeActivationTppRepresentation(evt: ref<FinalizeActivationTPPRepresentationEvent>) -> Bool {
  this.utppRepFinalAct += 1;
  // render-plane policy (utpp_itemplane.reds): the TPP arms / nails / garments of the new representation may
  // have been set to RPl_Weapon before the policy turned on (or before they were registered) - sync, then re-apply.
  UTPPItemPlaneOnRepresentation(this.GetGame(), "rep-activated");
  return false;
}

// AttachmentSlots.TppHead: "tpp" | "fpp" | "empty" | "spawning" (FPP head or empty slot while a TPP head of either
// gender is being spawned) | "other" | "noplayer".
public static func UTPPTppHeadSlotState() -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return "noplayer";
  }
  let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(player.GetGame());
  let slot: TweakDBID = t"AttachmentSlots.TppHead";
  let state: String = "other";
  let item: ref<ItemObject> = ts.GetItemInSlot(player, slot);
  if !IsDefined(item) {
    state = "empty";
  } else {
    let rec: TweakDBID = ItemID.GetTDBID(item.GetItemID());
    if rec == t"Items.PlayerFppHead" {
      state = "fpp";
    } else {
      if rec == t"Items.PlayerWaTppHead" || rec == t"Items.PlayerMaTppHead" {
        state = "tpp";
      };
    };
  };
  if Equals(state, "fpp") || Equals(state, "empty") {
    if ts.IsSlotSpawningItem(player, slot, ItemID.FromTDBID(t"Items.PlayerWaTppHead"))
      || ts.IsSlotSpawningItem(player, slot, ItemID.FromTDBID(t"Items.PlayerMaTppHead")) {
      return "spawning";
    };
  };
  return state;
}

public class UTPPBodyBridge extends ScriptableSystem {

  public func QueueTppRepresentation(on: Bool) -> Bool {
    return UTPPQueueTppRepresentation(on);
  }

  public func TppHeadSlotState() -> String {
    return UTPPTppHeadSlotState();
  }

  public func ApplyTppRepFeature(on: Bool) -> Bool {
    return UTPPApplyTppRepFeature(on);
  }

  public func SetFootTpp(on: Bool) -> Bool {
    return UTPPSetFootTpp(on);
  }

  public func SetMeleeTpp(on: Bool) -> Bool {
    return UTPPSetMeleeTpp(on);
  }

  public func SetTppRepKeep(on: Bool, seconds: Float) -> Bool {
    return UTPPSetTppRepKeep(on, seconds);
  }

  public func TppRepKeepDiag() -> String {
    return UTPPTppRepKeepDiag();
  }
}

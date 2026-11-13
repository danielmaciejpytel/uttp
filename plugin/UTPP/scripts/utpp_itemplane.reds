// UTPP item plane fix (body.itemPlane).
// Standalone: API from the vanilla scripts + Codeware imports.
//
// Bug: with a weapon drawn in TPP the weapon is visible THROUGH V's body. Vanilla
// EquipCycleInitEvents.OnEnter queues EquipStart with SetStartingRenderingPlane(RPl_Weapon) whenever the player is not
// in a vehicle (cyberpunk\player\psm\equipment.script:737-741), and the plane is only chosen at equip time
// (EquipStart.SetStartingRenderingPlane, orphans.script:47787; ERenderingPlane RPl_Scene=0/RPl_Background=1/RPl_Weapon=2,
// orphans.script:475-479). RPl_Weapon is composited over the scene, so in TPP the gun shows through the body.
//
// Fix without @replaceMethod:
//   1. @wrapMethod(EquipCycleInitEvents) OnEnter calls the original, then remembers the item and re-checks it 0.3 s later
//      (item spawn is asynchronous; animation tracks may re-apply the plane) and once more ~1.0 s after equip.
//   2. A watcher (DelaySystem, 0.25 s, while an item is held or the TPP signal is on, see below)
//      re-applies the plane when UTPPIsTppActive(game) changes. CET can force it immediately on a switch: Game.UTPPItemPlaneApply() or
//      Game.UTPPItemPlaneApplyPerspective(true|false).
//   3. One log line per correction: "[utpp_itemplane] plane -> scene|weapon (reason) ...", status UTPPItemPlaneStatus().
//   4. Without a further fix the weapon stays visible through the body for the first ~0.3 s
//      after the draw (the first correction would be "equip+0.3s"). The vanilla RPl_Weapon is only the STARTING plane of
//      the queued EquipStart; that event is a local of the vanilla OnEnter, queued inside it, so a wrap cannot reach it
//      ("SetStartingRenderingPlane again on the same event" is impossible without @replaceMethod). Two additions:
//      a) default: @wrapMethod(ItemObject) OnVisualSpawnAttached (WeaponObject's override reaches it through super) -
//         the first script-visible moment after the item is attached; a held player item (the pending equip or the
//         item in WeaponRight/WeaponLeft) in TPP outside a vehicle gets the scene event at once: "plane -> scene
//         (attach)". The 0.3 s / 1.0 s re-checks stay as the safety net (log lines unchanged).
//      b) opt-in A/B UTPPSetItemPlaneEquipRequeue(1): right after the vanilla OnEnter, queue a second EquipStart for the
//         same slot / item with SetStartingRenderingPlane(RPl_Scene) (vanilla fields, firstEquip false) - the closest
//         equivalent of re-setting the starting plane; what the engine does with two EquipStart events for one item is
//         UNVERIFIED (possible second spawn / animation restart), so default 0. Log "equip start re-queued (RPl_Scene)".
//
// API used for TPP -> RPl_Scene: queue gameItemEventsPropagateRenderingPlane on the ItemObject (+ its child items).
//   - Codeware import: public native class gameItemEventsPropagateRenderingPlane extends Event {} (no script fields);
//   - RED4ext.SDK generated header ItemEventsPropagateRenderingPlane.hpp: size 0x48, one unreflected 8-byte block at
//     +0x40 (the plane), i.e. the plane can NOT be set from redscript/CET; queued as created it carries the default 0 =
//     RPl_Scene;
//   - expected: queuing the default event on a held item moves it to the world plane.
//   Writing MeshComponent.renderingPlane after attach does NOT change the drawn plane - not used.
// FPP -> RPl_Weapon: NO verified script API (the event field is unreflected). Default = fail-closed: the item stays in
//   RPl_Scene until the next equip (vanilla equip in FPP sets RPl_Weapon again), one "UNAVAILABLE" line per item.
//   Experimental opt-in UTPPSetItemPlaneFppRestore(1): re-queue EquipStart(slot, item, RPl_Weapon, firstEquip=false) on
//   the player, as vanilla does at equip - effect on an item that is already attached is UNVERIFIED.
//   The verifiable alternative is a small native (UTPPSetItemRenderPlane: construct the event, write the byte at +0x40,
//   queue) - not implemented here.
//
//
// Problem: "plane -> scene" is logged at attach / +0.3 s / +1.0 s, but the weapon is still drawn through the
// body. Cause (details in src/render/ItemPlane.cpp): the scene event works (ctor zeroes +0x40 = RPl_Scene, the
// ItemObject handler applies it to the item and its child items), but the player entity's AttachmentSlots component binds
// WeaponRight / WeaponLeft / PersonalLink / SilverhandArm to the animation parameter "renderPlane" (function RenderingPlane);
// the FPP animation graph keeps it at weapon (>= 1.5) and every change of the parameter re-applies RPl_Weapon to the held
// item's components. Fix (native):
//   - UTPPSetItemPlaneOverride(true) while TPP outside a vehicle: inside the animation parameter broadcast RPl_Weapon becomes
//     RPl_Scene (all bound items: every weapon, its parts, cyberware arms); false in FPP / vehicle / fix off;
//   - UTPPSetItemRenderPlane(item, 0|2): event with the plane written natively, queued on the item and its child items -
//     used for every correction below; RPl_Weapon is now restored on a switch to FPP ("plane -> weapon (reason)").
//   If the native is unavailable (old DLL / hook inert) the default events apply (FPP fail-closed).
//   Log lines keep the format "[utpp_itemplane] plane -> scene|weapon (reason) item=<name>" + " via=native|event".
//
// one render-plane policy for V in third person (not only for the held item). The FPP animation parameters
// renderPlane / renderPlaneLeftArm / renderPlaneInspect drive not only the held item but also V's TPP arms and nails, every
// cyberware arm and the garments bound to them (tops, sleeves, gloves + their _shadow meshes); at 2 (RPl_Weapon) all of
// them are drawn over the torso, unbound clothes and the world. Before, the native override was armed only by the
// held-item lifecycle (equip / attach / watcher while an item was held), so unarmed TPP was not covered.
//   - One decision point, SyncPolicy(reason): override (force RPl_Scene) exactly when the fix is on AND the UTPP TPP signal
//     is on (UTPPPerspectiveState) AND V is not mounted AND photo mode is not active. Everything else is vanilla: FPP,
//     every path that ends the TPP signal (scenes / braindance / takeovers forcing first person through utpp_camera /
//     utpp_perspective), vehicles (the game's own TPP vehicle camera already avoids RPl_Weapon,
//     equipment.script:739) and photo mode (the visible V there is the photo-mode puppet; vanilla planes for it).
//   - Triggers: every SetTppOverride edge (utpp.reds), the TPP representation finalize events (utpp_body.reds: sync, then
//     UTPPItemPlaneReapply() for components that appeared with the representation), SetFix, the player attach (session
//     start; the native flag is process-wide and survives a load), the PhotoMode.IsActive blackboard and the watcher.
//   - The watcher (0.25 s) now also runs while the TPP signal is on with nothing held, so mount / unmount are seen.
//   - The native side (UTPPSetItemPlaneOverride) re-applies its registry of plane-bound components when the value changes
//     (force -> 0, vanilla -> exactly what vanilla requested), so a switch no longer latches.
//   - Equip / attach / ApplyFor only request a re-sync; their perspective argument no longer arms the override.
//     Correct() still sets the held item's starting plane (vanilla RPl_Weapon from EquipStart, outside the broadcast).
//   One log line per policy change: "[utpp_itemplane] policy -> scene|vanilla (<reason>) why=<term> hook=<bool>".
//
// OnTppSignal / OnRepresentation also correct the held item right at the edge (scope / silencer
// children are reached through Correct(); before, they waited for the next watcher tick); a PlayerStateMachine
// Vehicle / MountedToVehicle blackboard listener re-syncs the policy at the mount / unmount edge ("policy -> ... (mount)").
//
// Requires Codeware (event import + Reflection probe in the status) and utpp.reds (UTPPIsTppActive, UTPPLogLine,
// the natives UTPPSetItemPlaneOverride / UTPPItemPlaneReapply / UTPPItemPlaneDump). utpp.reds and utpp_body.reds call
// UTPPItemPlaneOnTppSignal / UTPPItemPlaneOnRepresentation below (the three files are always shipped together).
//
// CET console:
//   print(Game.UTPPItemPlaneStatus())
//   Game.UTPPSetItemPlaneFix(false)        -- default true
//   print(Game.UTPPItemPlaneApply())       -- re-apply for the current perspective now
//   print(Game.UTPPItemPlaneApplyPerspective(true))   -- utpp_camera/utpp_perspective: call with the NEW perspective
//   Game.UTPPSetItemPlaneFppRestore(1)     -- experimental FPP restore (default 0 = fail-closed)
//   Game.UTPPSetItemPlaneEquipRequeue(1)   -- experimental second EquipStart(RPl_Scene) at the draw in TPP (default 0)
//   print(Game.UTPPItemPlaneDump(64))      -- native registry: counters + one line per plane-bound component (also logged)
//   print(Game.UTPPItemPlaneReapply())     -- re-apply the current policy to every registered component now
// Fallback for a CET build that does not resolve script globals:
//   Game.GetScriptableSystemsContainer():Get("UTPPItemPlaneSystem"):Status()

public class UTPPItemPlaneTick extends DelayCallback {
  // 0 = perspective watcher, 1 = equip re-check, 2 = equip confirm re-send.
  public let kind: Int32;
  public let gen: Int32;
  public let attempt: Int32;

  public func Call() -> Void {
    let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
    if IsDefined(sys) {
      sys.OnTick(this.kind, this.gen, this.attempt);
    }
  }
}

public class UTPPItemPlaneSystem extends ScriptableSystem {
  private let m_off: Bool;               // false = fix on (default)
  private let m_fppMode: Int32;          // 0 = fail-closed (default), 1 = experimental EquipStart re-queue
  private let m_equipRequeue: Int32;     // 0 = off (default), 1 = experimental EquipStart(RPl_Scene) at the draw in TPP
  // Tracked item: the one whose plane UTPP knows about.
  private let m_item: ItemID;
  private let m_slot: TweakDBID;
  private let m_itemValid: Bool;
  private let m_plane: Int32;            // -1 unknown (assume vanilla), 0 RPl_Scene (sent by UTPP), 2 RPl_Weapon
  private let m_unavailLogged: Bool;
  private let m_lastTpp: Bool;
  // Equip hook: "drawn in perspective X" + pending re-check.
  private let m_pendingItem: ItemID;
  private let m_pendingSlot: TweakDBID;
  private let m_pending: Bool;
  private let m_drawnTpp: Bool;
  private let m_recheckGen: Int32;
  // Watcher.
  private let m_watchGen: Int32;
  private let m_watching: Bool;
  // Counters.
  private let m_hooks: Int32;
  private let m_toScene: Int32;
  private let m_toWeapon: Int32;
  private let m_unavailable: Int32;
  private let m_vehicleSkips: Int32;
  private let m_attachTimeouts: Int32;
  private let m_attachHooks: Int32;      // OnVisualSpawnAttached calls seen (any item)
  private let m_attachFixes: Int32;      // scene corrections at the attach
  private let m_requeues: Int32;         // experimental EquipStart(RPl_Scene) sent
  private let m_last: String;
  // native override state (what was last sent) + native event counters.
  private let m_overrideKnown: Bool;
  private let m_overrideOn: Bool;
  private let m_overrideHook: Bool;
  private let m_nativeEvents: Int32;
  private let m_nativeFails: Int32;
  // policy (SyncPolicy): the term that decided the last evaluation, the trigger of the last change, counters.
  private let m_policyWhy: String;
  private let m_policyReason: String;
  private let m_policyChanges: Int32;
  private let m_repReapplies: Int32;     // UTPPItemPlaneReapply() calls after a representation finalize event
  private let m_repReapplied: Int32;     // components written by those calls (sum of the native results)
  private let m_photoListener: ref<CallbackHandle>;
  private let m_photoEdges: Int32;
  private let m_photoBB: Bool;           // last PhotoMode.IsActive value from the blackboard listener
  //: PSM Vehicle / MountedToVehicle listeners (mount / unmount edges), edge-time correction.
  private let m_vehBB: ref<IBlackboard>;
  private let m_vehListener: ref<CallbackHandle>;
  private let m_mountListener: ref<CallbackHandle>;
  private let m_vehEdges: Int32;
  private let m_edgeCorrections: Int32;  // held-item corrections sent right at a TPP signal / representation edge

  // Session start: the engine calls this after a load / new game (same entry point as utpp_motion.reds OnPlayerAttach). The
  // native override flag is process-wide and survives a load, so the cached value is dropped and the policy is sent again.
  private final func OnPlayerAttach(request: ref<PlayerAttachRequest>) -> Void {
    this.m_overrideKnown = false;
    this.m_watching = false;
    this.m_watchGen += 1;
    this.m_pending = false;
    this.m_recheckGen += 1;
    this.m_itemValid = false;
    this.m_photoBB = false;
    this.ListenPhotoMode(true);
    this.ListenVehicle(true);
    this.SyncPolicy("session");
    this.EnsureWatch();
  }

  private func OnDetach() -> Void {
    this.ListenPhotoMode(false);
    this.ListenVehicle(false);
  }

  // PhotoMode.IsActive (global blackboard, the listener of vehicleTransition.script OnAttach): photo mode can freeze the
  // DelaySystem watcher, so the policy is re-synced from the blackboard change itself.
  private final func ListenPhotoMode(on: Bool) -> Void {
    let bb: ref<IBlackboard> = GameInstance.GetBlackboardSystem(this.GetGameInstance()).Get(GetAllBlackboardDefs().PhotoMode);
    if IsDefined(this.m_photoListener) {
      if IsDefined(bb) {
        bb.UnregisterListenerBool(GetAllBlackboardDefs().PhotoMode.IsActive, this.m_photoListener);
      }
      this.m_photoListener = null;
    }
    if on && IsDefined(bb) {
      this.m_photoListener = bb.RegisterListenerBool(GetAllBlackboardDefs().PhotoMode.IsActive, this, n"OnUTPPPhotoMode");
    }
  }

  protected cb func OnUTPPPhotoMode(active: Bool) -> Bool {
    this.m_photoEdges += 1;
    this.m_photoBB = active;
    if active {
      this.SyncPolicy("photo-on");
    } else {
      this.SyncPolicy("photo-off");
      this.EnsureWatch();
    }
    return false;
  }

  //: the player's state-machine blackboard (per player, unlike PhotoMode) PlayerStateMachine.Vehicle
  // (Int32, gamePSMVehicle: Default 0, Driving 1, Combat 2, Passenger 3, Transition 4, Turret 5, DriverCombat 6, Scene 7;
  // orphans.script:6791; same listener as player.script OnVehicleStateChange) and PlayerStateMachine.MountedToVehicle (Bool;
  // the other term utpp.reds UTPPPsmBlockMask reads). Either change re-evaluates the policy at the edge instead of at the next
  // 0.25 s watcher tick. The predicate (PolicyWanted: IsMountedToVehicle) is unchanged; if the mounting facility lags behind
  // the blackboard the sync is a no-op and the watcher remains the safety net. Registered on the blackboard that is current
  // at the player attach and unregistered from the one it was registered on (a load creates a new player and blackboard).
  private final func ListenVehicle(on: Bool) -> Void {
    let defs: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let old: ref<IBlackboard> = this.m_vehBB;
    if IsDefined(old) {
      if IsDefined(this.m_vehListener) {
        old.UnregisterListenerInt(defs.Vehicle, this.m_vehListener);
      }
      if IsDefined(this.m_mountListener) {
        old.UnregisterListenerBool(defs.MountedToVehicle, this.m_mountListener);
      }
    }
    this.m_vehListener = null;
    this.m_mountListener = null;
    this.m_vehBB = null;
    if !on {
      return;
    }
    let player: ref<PlayerPuppet> = GetPlayer(this.GetGameInstance());
    if !IsDefined(player) {
      return;
    }
    let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
    if !IsDefined(bb) {
      return;
    }
    this.m_vehBB = bb;
    this.m_vehListener = bb.RegisterListenerInt(defs.Vehicle, this, n"OnUTPPVehicleState");
    this.m_mountListener = bb.RegisterListenerBool(defs.MountedToVehicle, this, n"OnUTPPMountedState");
  }

  protected cb func OnUTPPVehicleState(value: Int32) -> Bool {
    this.m_vehEdges += 1;
    this.SyncPolicy("mount");
    this.EnsureWatch();
    return false;
  }

  protected cb func OnUTPPMountedState(value: Bool) -> Bool {
    this.m_vehEdges += 1;
    this.SyncPolicy("mount");
    this.EnsureWatch();
    return false;
  }

  public func SetFix(on: Bool) -> Void {
    if Equals(!this.m_off, on) {
      return;
    }
    this.m_off = !on;
    UTPPItemPlaneLog("fix " + UTPPItemPlaneBool(on));
    if on {
      this.SyncPolicy("fix");
      this.EnsureWatch();
    } else {
      this.m_watchGen += 1;
      this.m_watching = false;
      this.m_recheckGen += 1;
      this.m_pending = false;
      this.SyncPolicy("fix");
    }
  }

  // The policy as a predicate; why = the deciding term ("tpp" when the override is wanted).
  private func PolicyWanted(out why: String) -> Bool {
    if this.m_off {
      why = "fix-off";
      return false;
    }
    let game: GameInstance = this.GetGameInstance();
    if !UTPPTppOverrideFor(game) {
      why = "tpp-off";
      return false;
    }
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) {
      why = "noplayer";
      return false;
    }
    if VehicleComponent.IsMountedToVehicle(game, player) {
      why = "mounted";
      return false;
    }
    // The blackboard value and the system are both read: either one may change first.
    if this.m_photoBB || GameInstance.GetPhotoModeSystem(game).IsPhotoModeActive() {
      why = "photo";
      return false;
    }
    why = "tpp";
    return true;
  }

  // the only place that decides the native TPP override (RPl_Weapon from the animation parameter broadcast ->
  // RPl_Scene). Sends the value only on a change (the native re-applies its component registry then and logs the change).
  // Returns the policy (true = force RPl_Scene); callers use it as "the held item belongs on the scene plane".
  public func SyncPolicy(reason: String) -> Bool {
    let why: String;
    let on: Bool = this.PolicyWanted(why);
    this.m_policyWhy = why;
    if this.m_overrideKnown && Equals(on, this.m_overrideOn) {
      return on;
    }
    this.m_overrideKnown = true;
    this.m_overrideOn = on;
    this.m_policyChanges += 1;
    this.m_policyReason = reason;
    this.m_overrideHook = UTPPSetItemPlaneOverride(on);
    let plane: String = "vanilla";
    if on {
      plane = "scene";
    }
    UTPPItemPlaneLog("policy -> " + plane + " (" + reason + ") why=" + why + " hook=" + UTPPItemPlaneBool(this.m_overrideHook));
    return on;
  }

  // utpp.reds UTPPPerspectiveState.SetTppOverride, on every change of the TPP signal (after the new value is stored).
  public func OnTppSignal(active: Bool) -> Void {
    if active {
      let tpp: Bool = this.SyncPolicy("tpp-on");
      this.EnsureWatch();
      this.CorrectHeldAtEdge(tpp, "edge-tpp-on");
    } else {
      let tpp: Bool = this.SyncPolicy("tpp-off");
      this.CorrectHeldAtEdge(tpp, "edge-tpp-off");
    }
  }

  //: corrects the held item (and, through Correct(), its child items: scope, silencer) right at
  // an edge, after SyncPolicy (whose native override change re-applies the registry; OnRepresentation adds an explicit
  // UTPPItemPlaneReapply). Before, attachments - never registered natively, they have no binding - waited for the next
  // WatchTick (<= 0.25 s) or equip re-check. tpp = the policy just evaluated (same convention as attach / equip-recheck);
  // no correction while mounted (the game's vehicle camera owns the plane, equipment.script:739, same as WatchTick) or with
  // the fix off. The watcher / equip re-checks stay as the safety net.
  private func CorrectHeldAtEdge(tpp: Bool, reason: String) -> Void {
    if this.m_off {
      return;
    }
    let game: GameInstance = this.GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) || VehicleComponent.IsMountedToVehicle(game, player) {
      return;
    }
    let slot: TweakDBID;
    let obj: ref<ItemObject> = UTPPItemPlaneHeldItem(player, slot);
    if !IsDefined(obj) {
      return;
    }
    if !this.m_itemValid || !Equals(this.m_item, obj.GetItemID()) {
      // Plane unknown (vanilla assumed), as in ApplyFor / WatchTick.
      this.Adopt(obj, slot, -1);
    }
    this.m_lastTpp = tpp;
    this.m_edgeCorrections += 1;
    this.Correct(player, obj, tpp, reason);
  }

  // utpp_body.reds: the TPP representation finished (de)activating - new parts may have been set by a broadcast before the
  // policy changed (or before they were registered), so sync, then re-apply the current policy to every registered part.
  public func OnRepresentation(reason: String) -> Void {
    let tpp: Bool = this.SyncPolicy(reason);
    if this.m_off {
      return;
    }
    let written: Int32 = UTPPItemPlaneReapply();
    this.m_repReapplies += 1;
    this.m_repReapplied += written;
    UTPPItemPlaneLog("representation " + reason + ": reapplied " + IntToString(written) + " (policy "
      + UTPPItemPlaneBool(this.m_overrideOn) + ")");
    this.CorrectHeldAtEdge(tpp, "edge-" + reason);
  }

  public func GetFix() -> Bool {
    return !this.m_off;
  }

  public func SetEquipRequeue(mode: Int32) -> Bool {
    if mode < 0 || mode > 1 {
      return false;
    }
    this.m_equipRequeue = mode;
    UTPPItemPlaneLog("equipRequeue " + IntToString(mode));
    return true;
  }

  public func SetFppMode(mode: Int32) -> Bool {
    if mode < 0 || mode > 1 {
      return false;
    }
    this.m_fppMode = mode;
    UTPPItemPlaneLog("fppRestore " + IntToString(mode));
    return true;
  }

  // Called by the EquipCycleInitEvents.OnEnter wrap after vanilla queued EquipStart.
  public func OnEquipEnter(player: ref<PlayerPuppet>, item: ItemID, slot: TweakDBID, inVehicle: Bool) -> Void {
    this.m_hooks += 1;
    if this.m_off {
      return;
    }
    let game: GameInstance = player.GetGame();
    this.m_drawnTpp = UTPPIsTppActive(game);
    // set before the item spawns, so the first broadcast after the attach already gives RPl_Scene in TPP.
    // only a re-sync request; the policy decides (mount = IsMountedToVehicle, not the PSM vehicle state).
    this.SyncPolicy("equip");
    if inVehicle {
      // Vanilla decides the plane by the vehicle camera there (equipment.script:739); not ours.
      this.m_vehicleSkips += 1;
      this.m_pending = false;
      this.m_last = "skip vehicle " + UTPPItemPlaneItemName(item);
      return;
    }
    this.m_pendingItem = item;
    this.m_pendingSlot = slot;
    this.m_pending = true;
    this.m_recheckGen += 1;
    // opt-in: the closest equivalent of SetStartingRenderingPlane(RPl_Scene) on the vanilla event (unreachable).
    if this.m_equipRequeue == 1 && this.m_drawnTpp {
      let evt: ref<EquipStart> = new EquipStart();
      evt.SetSlotID(slot);
      evt.SetItemID(item);
      evt.SetHighPriority(NotEquals(WeaponObject.GetWeaponType(item), gamedataItemType.Invalid));
      evt.SetStartingRenderingPlane(ERenderingPlane.RPl_Scene);
      evt.SetFirstEquip(false);
      player.QueueEvent(evt);
      this.m_requeues += 1;
      UTPPItemPlaneLog("equip start re-queued (RPl_Scene, experimental) item=" + UTPPItemPlaneItemName(item));
    }
    // Immediate attempt only if the item object is already there (usually it is spawned later).
    let obj: ref<ItemObject> = GameInstance.GetTransactionSystem(game).GetItemInSlot(player, slot);
    if IsDefined(obj) && Equals(obj.GetItemID(), item) && this.m_drawnTpp {
      this.Adopt(obj, slot, 2);
      this.Correct(player, obj, true, "equip");
    }
    this.Schedule(game, 1, this.m_recheckGen, 1, 0.3);
    this.EnsureWatch();
  }

  // ItemObject.OnVisualSpawnAttached (any item; WeaponObject reaches it through super). A held player item
  // (pending equip or WeaponRight/WeaponLeft) in TPP outside a vehicle gets the scene plane now instead of 0.3 s later.
  public func OnItemAttached(item: ref<ItemObject>) -> Void {
    this.m_attachHooks += 1;
    if this.m_off || !IsDefined(item) {
      return;
    }
    let player: ref<PlayerPuppet> = item.GetOwner() as PlayerPuppet;
    if !IsDefined(player) {
      return;
    }
    let id: ItemID = item.GetItemID();
    let slot: TweakDBID;
    let held: ref<ItemObject> = UTPPItemPlaneHeldItem(player, slot);
    let isPending: Bool = this.m_pending && Equals(id, this.m_pendingItem);
    if isPending {
      slot = this.m_pendingSlot;
    } else {
      if !IsDefined(held) || !Equals(held.GetItemID(), id) {
        return;
      }
    }
    // not mounted, TPP signal on, no photo mode, fix on = the policy (same terms as before plus photo mode).
    if !this.SyncPolicy("attach") {
      return;
    }
    if !this.m_itemValid || !Equals(this.m_item, id) {
      this.Adopt(item, slot, 2);
    }
    this.m_lastTpp = true;
    this.m_attachFixes += 1;
    this.Correct(player, item, true, "attach");
    this.EnsureWatch();
  }

  public func OnTick(kind: Int32, gen: Int32, attempt: Int32) -> Void {
    if this.m_off {
      return;
    }
    if kind == 0 {
      if gen == this.m_watchGen {
        this.WatchTick();
      }
      return;
    }
    if gen != this.m_recheckGen || !this.m_pending {
      return;
    }
    let game: GameInstance = this.GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) {
      return;
    }
    let obj: ref<ItemObject> = GameInstance.GetTransactionSystem(game).GetItemInSlot(player, this.m_pendingSlot);
    if !IsDefined(obj) || !Equals(obj.GetItemID(), this.m_pendingItem) {
      if kind == 1 && attempt < 6 {
        this.Schedule(game, 1, gen, attempt + 1, 0.3);
      } else {
        this.m_pending = false;
        this.m_attachTimeouts += 1;
        UTPPItemPlaneLog("no correction: " + UTPPItemPlaneItemName(this.m_pendingItem) + " not in "
          + TDBID.ToStringDEBUG(this.m_pendingSlot) + " after " + IntToString(attempt) + " checks");
      }
      return;
    }
    if VehicleComponent.IsMountedToVehicle(game, player) {
      this.m_pending = false;
      this.m_vehicleSkips += 1;
      this.SyncPolicy("equip-recheck");
      return;
    }
    let tpp: Bool = this.SyncPolicy("equip-recheck");
    if !this.m_itemValid || !Equals(this.m_item, this.m_pendingItem) {
      // Not in a vehicle at equip -> vanilla gave RPl_Weapon (the scene event below corrects it in TPP).
      this.Adopt(obj, this.m_pendingSlot, 2);
    }
    this.m_lastTpp = tpp;
    if kind == 1 {
      if tpp {
        this.Correct(player, obj, true, "equip+0.3s");
        // One confirm re-send ~1.0 s after equip (the draw animation may re-apply the plane).
        this.Schedule(game, 2, gen, attempt, 0.7);
      } else {
        this.m_pending = false;
        // Switched to FPP after an immediate scene correction: restore / report (no-op when still RPl_Weapon).
        this.Correct(player, obj, false, "equip+0.3s");
      }
      return;
    }
    this.m_pending = false;
    this.Correct(player, obj, tpp, "equip+1.0s");
  }

  // Re-applies the plane for the given perspective now (CET / perspective switch); always re-sends the scene event.
  public func ApplyFor(tpp: Bool, reason: String) -> String {
    if this.m_off {
      return "off";
    }
    let game: GameInstance = this.GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) {
      return "noplayer";
    }
    if VehicleComponent.IsMountedToVehicle(game, player) {
      this.m_vehicleSkips += 1;
      this.SyncPolicy(reason);
      return "vehicle";
    }
    // the override follows the policy (TPP signal), not the caller's argument; `tpp` (the perspective the caller
    // switches to, possibly ahead of the signal) still decides the held item's correction below.
    this.SyncPolicy(reason);
    let slot: TweakDBID;
    let obj: ref<ItemObject> = UTPPItemPlaneHeldItem(player, slot);
    if !IsDefined(obj) {
      return "noitem";
    }
    if !this.m_itemValid || !Equals(this.m_item, obj.GetItemID()) {
      this.Adopt(obj, slot, -1);
    }
    this.m_lastTpp = tpp;
    let result: String = this.Correct(player, obj, tpp, reason);
    this.EnsureWatch();
    return result;
  }

  public func Status() -> String {
    let game: GameInstance = this.GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    let tpp: String = "?";
    let vehicle: String = "?";
    let held: String = "none";
    if IsDefined(player) {
      tpp = UTPPItemPlaneBool(UTPPIsTppActive(game));
      vehicle = UTPPItemPlaneBool(VehicleComponent.IsMountedToVehicle(game, player));
      let slot: TweakDBID;
      let obj: ref<ItemObject> = UTPPItemPlaneHeldItem(player, slot);
      if IsDefined(obj) {
        held = UTPPItemPlaneItemName(obj.GetItemID()) + "@" + TDBID.ToStringDEBUG(slot);
      }
    }
    let tracked: String = "none";
    if this.m_itemValid {
      tracked = UTPPItemPlaneItemName(this.m_item);
    }
    // RTTI probe (par. 29.5 item 2): number of reflected properties of the plane event (expected 0 = plane unreflected).
    let props: Int32 = -1;
    let cls: ref<ReflectionClass> = Reflection.GetClass(n"gameItemEventsPropagateRenderingPlane");
    if IsDefined(cls) {
      let propList: array<ref<ReflectionProp>> = cls.GetProperties();
      props = ArraySize(propList);
    }
    return "fix=" + UTPPItemPlaneBool(!this.m_off) + " fppRestore=" + IntToString(this.m_fppMode)
      + " tppActive=" + tpp + " inVehicle=" + vehicle + " held=" + held + " tracked=" + tracked
      + " plane=" + UTPPItemPlaneName(this.m_plane) + " drawnTpp=" + UTPPItemPlaneBool(this.m_drawnTpp)
      + " pending=" + UTPPItemPlaneBool(this.m_pending) + " watching=" + UTPPItemPlaneBool(this.m_watching)
      + " hooks=" + IntToString(this.m_hooks) + " toScene=" + IntToString(this.m_toScene)
      + " toWeapon=" + IntToString(this.m_toWeapon) + " unavailable=" + IntToString(this.m_unavailable)
      + " vehicleSkips=" + IntToString(this.m_vehicleSkips) + " attachTimeouts=" + IntToString(this.m_attachTimeouts)
      + " attachHooks=" + IntToString(this.m_attachHooks) + " attachFixes=" + IntToString(this.m_attachFixes)
      + " equipRequeue=" + IntToString(this.m_equipRequeue) + " requeues=" + IntToString(this.m_requeues)
      + " eventProps=" + IntToString(props) + " override=" + UTPPItemPlaneBool(this.m_overrideOn)
      + " overrideHook=" + UTPPItemPlaneBool(this.m_overrideHook) + " nativeEvents=" + IntToString(this.m_nativeEvents)
      + " nativeFails=" + IntToString(this.m_nativeFails) + " policyWhy=" + this.m_policyWhy
      + " policyReason=" + this.m_policyReason + " policyChanges=" + IntToString(this.m_policyChanges)
      + " photoEdges=" + IntToString(this.m_photoEdges) + " repReapplies=" + IntToString(this.m_repReapplies)
      + " repReapplied=" + IntToString(this.m_repReapplied) + " vehEdges=" + IntToString(this.m_vehEdges)
      + " edgeCorrections=" + IntToString(this.m_edgeCorrections) + " last=" + this.m_last + " | " + UTPPItemPlaneNativeStatus();
  }

  private func Adopt(obj: ref<ItemObject>, slot: TweakDBID, plane: Int32) -> Void {
    this.m_item = obj.GetItemID();
    this.m_slot = slot;
    this.m_itemValid = true;
    this.m_plane = plane;
    this.m_unavailLogged = false;
  }

  private func Correct(player: ref<PlayerPuppet>, obj: ref<ItemObject>, tpp: Bool, reason: String) -> String {
    let name: String = UTPPItemPlaneItemName(obj.GetItemID());
    if tpp {
      // native event (plane written explicitly, item + child items); fallback = default-constructed event (RPl_Scene).
      let via: String = "native";
      if UTPPSetItemRenderPlane(obj, 0) {
        this.m_nativeEvents += 1;
      } else {
        this.m_nativeFails += 1;
        via = "event";
        obj.QueueEvent(new gameItemEventsPropagateRenderingPlane());
        obj.QueueEventToChildItems(new gameItemEventsPropagateRenderingPlane());
      }
      this.m_plane = 0;
      this.m_unavailLogged = false;
      this.m_toScene += 1;
      this.m_last = "scene " + reason + " " + name;
      UTPPItemPlaneLog("plane -> scene (" + reason + ") item=" + name + " via=" + via);
      return "scene";
    }
    if this.m_plane != 0 {
      // Drawn in FPP (vanilla RPl_Weapon) or unknown: nothing to undo.
      return "weapon-kept";
    }
    // RPl_Weapon back through the native event (the policy is already vanilla, see SyncPolicy).
    if UTPPSetItemRenderPlane(obj, 2) {
      this.m_nativeEvents += 1;
      this.m_plane = 2;
      this.m_toWeapon += 1;
      this.m_last = "weapon " + reason + " " + name;
      UTPPItemPlaneLog("plane -> weapon (" + reason + ") item=" + name + " via=native");
      return "weapon";
    }
    this.m_nativeFails += 1;
    if this.m_fppMode == 1 {
      let evt: ref<EquipStart> = new EquipStart();
      evt.SetSlotID(this.m_slot);
      evt.SetItemID(obj.GetItemID());
      evt.SetStartingRenderingPlane(ERenderingPlane.RPl_Weapon);
      evt.SetFirstEquip(false);
      player.QueueEvent(evt);
      this.m_plane = 2;
      this.m_toWeapon += 1;
      this.m_last = "weapon " + reason + " " + name;
      UTPPItemPlaneLog("plane -> weapon (" + reason + ", experimental EquipStart re-queue) item=" + name);
      return "weapon";
    }
    if !this.m_unavailLogged {
      this.m_unavailLogged = true;
      this.m_unavailable += 1;
      this.m_last = "weapon-unavailable " + reason + " " + name;
      UTPPItemPlaneLog("plane -> weapon UNAVAILABLE (" + reason + ") item=" + name
        + ": no script API for RPl_Weapon after equip; stays RPl_Scene until next equip");
    }
    return "weapon-unavailable";
  }

  private func WatchTick() -> Void {
    let game: GameInstance = this.GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) {
      this.SyncPolicy("watch");
      this.m_watching = false;
      return;
    }
    let slot: TweakDBID;
    let obj: ref<ItemObject> = UTPPItemPlaneHeldItem(player, slot);
    let mounted: Bool = VehicleComponent.IsMountedToVehicle(game, player);
    if !IsDefined(obj) {
      //       // Nothing held: the policy still follows the TPP signal / mount / photo mode. The watcher keeps running
      // while the TPP signal is on (it is the only trigger for mount / unmount); stops in FPP until the next signal edge /
      // equip / CET apply.
      this.SyncPolicy("watch");
      this.m_itemValid = false;
      if !UTPPIsTppActive(game) {
        this.m_watching = false;
        return;
      }
      this.Schedule(game, 0, this.m_watchGen, 0, 0.25);
      return;
    }
    if mounted {
      this.SyncPolicy("watch");
    } else {
      let tpp: Bool = this.SyncPolicy("watch");
      let id: ItemID = obj.GetItemID();
      if this.m_pending && Equals(id, this.m_pendingItem) {
        // The equip re-check owns this item.
      } else {
        if !this.m_itemValid || !Equals(this.m_item, id) {
          // Item equipped without a seen OnEnter (or after a reload): plane unknown, vanilla assumed.
          this.Adopt(obj, slot, -1);
          if tpp {
            this.Correct(player, obj, true, "watch-new-item");
          }
          this.m_lastTpp = tpp;
        } else {
          if !Equals(tpp, this.m_lastTpp) {
            this.m_lastTpp = tpp;
            if tpp {
              this.Correct(player, obj, true, "perspective-tpp");
            } else {
              this.Correct(player, obj, false, "perspective-fpp");
            }
          }
        }
      }
    }
    this.Schedule(game, 0, this.m_watchGen, 0, 0.25);
  }

  private func EnsureWatch() -> Void {
    if this.m_watching || this.m_off {
      return;
    }
    this.m_watching = true;
    this.m_watchGen += 1;
    this.Schedule(this.GetGameInstance(), 0, this.m_watchGen, 0, 0.25);
  }

  private func Schedule(game: GameInstance, kind: Int32, gen: Int32, attempt: Int32, delay: Float) -> Void {
    let cb: ref<UTPPItemPlaneTick> = new UTPPItemPlaneTick();
    cb.kind = kind;
    cb.gen = gen;
    cb.attempt = attempt;
    GameInstance.GetDelaySystem(game).DelayCallback(cb, delay, false);
  }
}

public func UTPPItemPlaneOf(game: GameInstance) -> ref<UTPPItemPlaneSystem> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPItemPlaneSystem") as UTPPItemPlaneSystem;
}

// utpp.reds UTPPPerspectiveState.SetTppOverride: the TPP signal changed (the new value is already stored).
public func UTPPItemPlaneOnTppSignal(game: GameInstance, active: Bool) -> Void {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(game);
  if IsDefined(sys) {
    sys.OnTppSignal(active);
  }
}

// utpp_body.reds: Finalize(De)ActivationTPPRepresentationEvent reached the player.
public func UTPPItemPlaneOnRepresentation(game: GameInstance, reason: String) -> Void {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(game);
  if IsDefined(sys) {
    sys.OnRepresentation(reason);
  }
}

// Item in AttachmentSlots.WeaponRight, else WeaponLeft; slot returned through `slot`.
public func UTPPItemPlaneHeldItem(player: ref<PlayerPuppet>, out slot: TweakDBID) -> ref<ItemObject> {
  let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(player.GetGame());
  slot = t"AttachmentSlots.WeaponRight";
  let obj: ref<ItemObject> = ts.GetItemInSlot(player, slot);
  if IsDefined(obj) {
    return obj;
  }
  slot = t"AttachmentSlots.WeaponLeft";
  return ts.GetItemInSlot(player, slot);
}

public func UTPPItemPlaneLog(line: String) -> Void {
  UTPPLogLine("[utpp_itemplane] " + line);
}

public func UTPPItemPlaneBool(b: Bool) -> String {
  if b {
    return "true";
  }
  return "false";
}

public func UTPPItemPlaneName(plane: Int32) -> String {
  if plane == 0 {
    return "scene";
  }
  if plane == 2 {
    return "weapon";
  }
  return "unknown";
}

public func UTPPItemPlaneItemName(id: ItemID) -> String {
  return TDBID.ToStringDEBUG(ItemID.GetTDBID(id));
}

// ---- CET entry points (Game.<name>(...)) ----

// Default on. False = no game session yet (nothing stored).
public func UTPPSetItemPlaneFix(on: Bool) -> Bool {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  if !IsDefined(sys) {
    return false;
  }
  sys.SetFix(on);
  return true;
}

public func UTPPGetItemPlaneFix() -> Bool {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  return IsDefined(sys) && sys.GetFix();
}

// 0 = fail-closed (default), 1 = experimental EquipStart re-queue with RPl_Weapon on a switch to FPP.
public func UTPPSetItemPlaneFppRestore(mode: Int32) -> Bool {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  return IsDefined(sys) && sys.SetFppMode(mode);
}

// 0 = off (default), 1 = experimental second EquipStart(RPl_Scene) right after the vanilla OnEnter when drawn in TPP.
public func UTPPSetItemPlaneEquipRequeue(mode: Int32) -> Bool {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  return IsDefined(sys) && sys.SetEquipRequeue(mode);
}

// Re-apply for the current perspective (UTPPIsTppActive). Returns scene | weapon | weapon-kept | weapon-unavailable |
// noitem | vehicle | off | noplayer | nosystem.
public func UTPPItemPlaneApply() -> String {
  let game: GameInstance = GetGameInstance();
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(game);
  if !IsDefined(sys) {
    return "nosystem";
  }
  let tpp: Bool = UTPPIsTppActive(game);
  if tpp {
    return sys.ApplyFor(true, "cet-tpp");
  }
  return sys.ApplyFor(false, "cet-fpp");
}

// Same with the perspective given by the caller (use on a switch, before UTPPIsTppActive may have caught up).
public func UTPPItemPlaneApplyPerspective(tpp: Bool) -> String {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  if !IsDefined(sys) {
    return "nosystem";
  }
  if tpp {
    return sys.ApplyFor(true, "switch-tpp");
  }
  return sys.ApplyFor(false, "switch-fpp");
}

public func UTPPItemPlaneStatus() -> String {
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(GetGameInstance());
  if !IsDefined(sys) {
    return "UTPPItemPlaneSystem missing";
  }
  return sys.Status();
}

// ---- Hook ----

@wrapMethod(EquipCycleInitEvents)
protected func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if !IsDefined(player) {
    return;
  }
  // Vanilla queues no EquipStart for fluff consumables (equipment.script:735).
  if this.IsUsingFluffConsumable(scriptInterface, this.stateMachineInstanceData) {
    return;
  }
  let item: ItemID = this.GetSlotActiveItem(scriptInterface, this.stateMachineInstanceData);
  if !ItemID.IsValid(item) {
    return;
  }
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(player.GetGame());
  if IsDefined(sys) {
    // Game 2.5: the placement slot of a left-hand (dual wield) weapon is its second placement slot; same flag as the vanilla
    // 2.5 EquipCycleInitEvents.OnEnter (false for the right hand and for non-weapons, i.e. the pre-2.5 result).
    let isLeftWeapon: Bool = Equals(this.GetItemCategoryFromItemID(item), gamedataItemCategory.Weapon)
      && this.IsLeftHandLogic(this.stateMachineInstanceData);
    sys.OnEquipEnter(player, item, EquipmentSystem.GetPlacementSlot(item, isLeftWeapon), stateContext.IsStateMachineActive(n"Vehicle"));
  }
}

// earliest script-visible moment after an item is attached (see the header, item 4a).
@wrapMethod(ItemObject)
protected cb func OnVisualSpawnAttached() -> Bool {
  let result: Bool = wrappedMethod();
  let sys: ref<UTPPItemPlaneSystem> = UTPPItemPlaneOf(this.GetGame());
  if IsDefined(sys) {
    sys.OnItemAttached(this);
  }
  return result;
}

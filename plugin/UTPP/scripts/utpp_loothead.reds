// UTPP LootRange + HeadGuard, natives exported by red4ext/plugins/UTPP/UTPP.dll.
//
// LootRange: while third person is active, the look-at test of the "Loot" layer of container and NPC body
//   interactions is replaced by the game's own screen-centre window (the predicate CDPR uses for loot drops), so the
//   loot prompt follows the orbit camera. The 2 m reach around the player is unchanged. First person = vanilla.
// HeadGuard: while third person is active, the game's near-camera distance threshold (CameraSystem
//   PlayerAimUpdateTick, squared distance 0.2025 = 0.45 m) is set to the TPP value (default 0 = never "near") and
//   restored in first person. The site is found at load by UTPP's own signature + validation; any mismatch = status
//   FAILED and nothing is written. UTPPHeadGuardSetSite can still override it for experiments (console).
// Both: default OFF (enabled by the CET module utpp_loothead / UTPP tab), off by session kill natives and hard kill
//   files next to UTPP.dll: utpp_lootrange.disable, utpp_headguard.disable.
// Requires utpp.reds (UTPPIsTppActive) in the same script folder.

// Third-person gate (shared). Called every locomotion update below; stale after 0.5 s -> vanilla.
public native func UTPPLootHeadHeartbeat(ownerSaysTpp: Bool, cameraDistance: Float) -> Void
// Heartbeat timeout s (0.5) and camera distance from the first-person camera treated as TPP m (0.6).
public native func UTPPLootHeadSetGate(staleSeconds: Float, tppCameraDistance: Float) -> Bool
public native func UTPPLootHeadIsTpp() -> Bool

// print(Game.UTPPLootRangeStatus())
public native func UTPPLootRangeStatus() -> String
public native func UTPPLootRangeSetEnabled(on: Bool) -> Void
// Session kill switch: restores vanilla look-at immediately, off until the game restarts.
public native func UTPPLootRangeKill() -> Void
// Window size relative to the vanilla loot drop window (0.65 x 0.65): 0.25 .. 2.5, default 1.0.
public native func UTPPLootRangeSetScale(scale: Float) -> Bool
// Raw window (width, height 0.05..2, curvature 0.1..4, maxPriorityBoundsFactor 0..1).
public native func UTPPLootRangeSetWindow(width: Float, height: Float, curvature: Float, maxPriorityBoundsFactor: Float) -> Bool
// Extra interaction descriptor (depot path of a *.interaction) whose "Loot" layer look-at should follow the camera.
public native func UTPPLootRangeAddDescriptor(path: String) -> Bool
public native func UTPPLootRangeActive() -> Bool
// Bitmask (0 = active): 1 not resolved, 2 disabled, 4 killed, (8 unused), 16 not TPP, 32 kill file, 64 game not
// validated, 128 no predicate captured yet.
public native func UTPPLootRangeReasons() -> Int32

// print(Game.UTPPHeadGuardStatus())
public native func UTPPHeadGuardStatus() -> String
public native func UTPPHeadGuardSetEnabled(on: Bool) -> Void
public native func UTPPHeadGuardKill() -> Void
// Candidate site: hex pattern ("??" wildcards) unique in .text, instruction offset from the match, the constant's
// current value and the value to use in TPP. Only scalar-float RIP-relative loads/compares are accepted.
public native func UTPPHeadGuardSetSite(pattern: String, instructionOffset: Int32, expectedValue: Float, tppValue: Float) -> Bool
// TPP value of the built-in site = squared metres (0 = never near; vanilla 0.2025).
public native func UTPPHeadGuardSetValue(tppValue: Float) -> Bool
public native func UTPPHeadGuardActive() -> Bool
// Bitmask (0 = active): 1 no site, 2 disabled, 4 killed, (8 unused), 16 not TPP, 32 kill file, 64 game not validated,
// 128 patch failed.
public native func UTPPHeadGuardReasons() -> Int32

// InteractRange: while third person is active, the look-at test of the seat layers of vehicles and of the activation
// layers of devices / doors / NPC talk is replaced (UTPP.dll, src/world/InteractRangeCore.hpp):
//   vehicles  -> the vanilla seat sphere around V alone decides (engine AlwaysSame predicate, priority 1.0); the seat
//                group is also listed as "not look-at gated" in the descriptor's load-time classification while morphed,
//   devices, doors, NPC talk -> the game's own screen-centre window (as for dropped items) x user scale.
// Reach around V is unchanged; first person = vanilla. Default OFF in the natives; the CET module utpp_loothead turns it on
// (setting interactRange.impl). Kill: UTPPInteractRangeKill(), file utpp_interactrange.disable next to UTPP.dll.
// print(Game.UTPPInteractRangeStatus())
public native func UTPPInteractRangeStatus() -> String
public native func UTPPInteractRangeSetEnabled(on: Bool) -> Void
public native func UTPPInteractRangeKill() -> Void
// Window size relative to the vanilla dropped-item window (0.65 x 0.65): 0.25 .. 2.5, default 1.0.
public native func UTPPInteractRangeSetScale(scale: Float) -> Bool
// Vehicle seats in third person: 0 vanilla look-at, 1 seat sphere around V (default), 2 screen-centre window,
// 3 seat sphere without the classification list change (A/B check). Status lists the classification per descriptor.
public native func UTPPInteractRangeSetVehicleMode(mode: Int32) -> Bool
// Priority of a vehicle seat in modes 1 / 3 (0.05 .. 1, default 1.0); applies to already morphed seats on the next tick.
public native func UTPPInteractRangeSetVehiclePriority(priority: Float) -> Bool
public native func UTPPInteractRangeActive() -> Bool
// Same bitmask as UTPPLootRangeReasons.
public native func UTPPInteractRangeReasons() -> Int32
// Names the interaction descriptor of an InteractionComponent and whether UTPP manages it.
public native func UTPPInteractDescriptorOf(component: ref<IComponent>) -> String

@addMethod(Entity)
public func UTPPInteractionComponent() -> ref<IComponent> {
  let c: ref<IComponent> = this.FindComponentByName(n"interaction");
  if !IsDefined(c) {
    c = this.FindComponentByName(n"Interaction");
  }
  return c;
}

// Console probe: print(Game.UTPPInteractProbe(4.0)) - the look-at object and every entity within range m of V that has
// an interaction component, with the descriptor it uses (managed by UTPP or not). Read-only.
public func UTPPInteractProbe(range: Float) -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return "UTPPInteractProbe: no player";
  }
  let out: String = "UTPPInteractProbe tpp=" + ToString(UTPPLootHeadIsTpp());
  let look: ref<GameObject> = GameInstance.GetTargetingSystem(player.GetGame()).GetLookAtObject(player, false, false);
  if IsDefined(look) {
    out += "\n look-at " + NameToString(look.GetClassName()) + ": ";
    let lc: ref<IComponent> = look.UTPPInteractionComponent();
    out += IsDefined(lc) ? UTPPInteractDescriptorOf(lc) : "no interaction component";
  } else {
    out += "\n look-at: none";
  }
  let ents: array<ref<Entity>> = player.GetEntitiesAroundObject(range);
  let pos: Vector4 = player.GetWorldPosition();
  let i: Int32 = 0;
  while i < ArraySize(ents) {
    let e: ref<Entity> = ents[i];
    if IsDefined(e) {
      let c: ref<IComponent> = e.UTPPInteractionComponent();
      if IsDefined(c) {
        out += "\n " + NameToString(e.GetClassName()) + " d=" + FloatToStringPrec(Vector4.Distance(pos, e.GetWorldPosition()), 2)
          + ": " + UTPPInteractDescriptorOf(c);
      }
    }
    i += 1;
  }
  return out;
}

// Distance between the active camera and the player's first-person camera (0 in first person).
public func UTPPLootHeadCameraDistance(scriptInterface: ref<StateGameScriptInterface>, player: ref<PlayerPuppet>) -> Float {
  let fpp: ref<FPPCameraComponent> = player.GetFPPCameraComponent();
  if !IsDefined(fpp) {
    return 0.0;
  }
  let cam: Vector4 = Transform.GetPosition(scriptInterface.GetCameraWorldTransform());
  let eye: Vector4 = Matrix.GetTranslation(fpp.GetLocalToWorld());
  let dx: Float = cam.X - eye.X;
  let dy: Float = cam.Y - eye.Y;
  let dz: Float = cam.Z - eye.Z;
  return SqrtF(dx * dx + dy * dy + dz * dz);
}

public func UTPPLootHeadTick(scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if !IsDefined(player) {
    return;
  }
  UTPPLootHeadHeartbeat(UTPPIsTppActive(player.GetGame()), UTPPLootHeadCameraDistance(scriptInterface, player));
}

// Chained with the other wraps of the same method (redscript chains all wraps).
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  UTPPLootHeadTick(scriptInterface);
  wrappedMethod(timeDelta, stateContext, scriptInterface);
}

// ClimbEvents overrides OnUpdate without calling super.OnUpdate (vanilla locomotionTransitions.script), so the wrap above
// never ran during a climb: the heartbeat went stale after 0.5 s and HeadGuard / LootRange reported "not TPP" (restore +
// revert) for the rest of every longer climb. Keep the heartbeat alive in the climb state too (same wrap as utpp.reds uses for
// the MM feed; redscript chains all wraps). VaultEvents has no OnUpdate override and is covered by the wrap above.
@wrapMethod(ClimbEvents)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  UTPPLootHeadTick(scriptInterface);
  wrappedMethod(timeDelta, stateContext, scriptInterface);
}

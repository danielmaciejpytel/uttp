// Read-only car-hit probe (ragdoll gate R0). Diagnostic only: no gameplay change, no events queued, no state written
// except the probe's own bookkeeping fields below.
//
// What it logs (UTPPLogLine, prefix "[utpp_ragdoll]"):
//   carhit      one line per episode, written after the vanilla OnCarHitPlayer body ran (wrappedMethod first):
//               hit direction, separation impulse (the event field is spelled "seperationImpulse" by the game), car id /
//               record / speed, player position + yaw, PSM high level / locomotion / detailed locomotion / reaction, time.
//   carhit+t    10 Hz samples for 3 s after the hit: PSM, status effect ids of the player, player position.
//   knockdown   once per episode when PSM LocomotionDetailed becomes Knockdown (29) inside the window.
//   carhit-end  episode end (window over / unsafe read) with the number of suppressed hit events.
// Bounds: one episode at a time, window 3 s, cooldown 5 s measured from the start of the previous episode, at most 32 samples.
// Sources (2.31, the 2.5 scripts cannot be decompiled; names and signatures are checked by the offline 2.5 compile):
//   player.script:5776 OnCarHitPlayer, orphans.script:43532 OnCarHitPlayer event (hitDirection, carId, seperationImpulse),
//   orphans.script:17877 StatusEffectSystem.GetAppliedEffects, :20608 StatusEffect.GetRecord, vehicles.script:116 GetRecordID /
//   :240 GetLinearVelocity, blackboardDefinitions.script:4-24 PlayerStateMachine fields, orphans.script:6647 / :6685 / :6766 enums.
// Requires utpp.reds (UTPPLogLine, UTPPYawDegFromForward).

// Single gate. Default OFF since the production script utpp_ragdoll.reds owns the OnCarHitPlayer behaviour: with the gate off
// this file's OnCarHitPlayer wrap is a pass-through (the "[utpp_ragdoll] eng ev" event log still runs during a probe episode). The CET-callable debug
// functions (UTPPRagdollEngineStatus / Force / Stop) work regardless; Force refuses while a production episode runs and the
// production start refuses while a probe episode runs (utppRgProbeBusy). Set true only for a research run without the production
// ragdoll (the probe observes the vanilla car hit and logs, it changes nothing).
public func UTPPRagdollProbeEnabled() -> Bool {
  return false;
}

@addField(PlayerPuppet)
public let utppRdSeq: Int32;

@addField(PlayerPuppet)
public let utppRdEnd: Float;

@addField(PlayerPuppet)
public let utppRdCooldown: Float;

@addField(PlayerPuppet)
public let utppRdSuppressed: Int32;

public func UTPPRdNow(game: GameInstance) -> Float {
  return EngineTime.ToFloat(GameInstance.GetSimTime(game));
}

public func UTPPRdVec(v: Vector4) -> String {
  return "(" + FloatToStringPrec(v.X, 2) + "," + FloatToStringPrec(v.Y, 2) + "," + FloatToStringPrec(v.Z, 2) + ")";
}

// "high=.. loco=.. det=.. react=.. fall=.. veh=.. mount=.."; "psm=none" when the blackboard is not there.
public func UTPPRdPsm(player: ref<PlayerPuppet>) -> String {
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  if !IsDefined(bb) {
    return "psm=none";
  }
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  return "high=" + IntToString(bb.GetInt(psm.HighLevel))
    + " loco=" + IntToString(bb.GetInt(psm.Locomotion))
    + " det=" + IntToString(bb.GetInt(psm.LocomotionDetailed))
    + " react=" + IntToString(bb.GetInt(psm.Reaction))
    + " fall=" + IntToString(bb.GetInt(psm.Fall))
    + " veh=" + IntToString(bb.GetInt(psm.Vehicle))
    + " mount=" + (bb.GetBool(psm.MountedToVehicle) ? "1" : "0");
}

public func UTPPRdDetailed(player: ref<PlayerPuppet>) -> Int32 {
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  if !IsDefined(bb) {
    return -1;
  }
  return bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.LocomotionDetailed);
}

// Reads are unsafe (and uninteresting) once the player is in / entering a vehicle.
public func UTPPRdUnsafe(player: ref<PlayerPuppet>) -> Bool {
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  if !IsDefined(bb) {
    return true;
  }
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  if bb.GetInt(psm.Vehicle) != 0 || bb.GetBool(psm.MountedToVehicle) {
    return true;
  }
  // Braindance: Braindance.IsActive (blackboardDefinitions.script:1076, read as playerVisionModeController.script:117 / :148).
  let bd: ref<IBlackboard> = GameInstance.GetBlackboardSystem(player.GetGame()).Get(GetAllBlackboardDefs().Braindance);
  return IsDefined(bd) && bd.GetBool(GetAllBlackboardDefs().Braindance.IsActive);
}

// Applied status effect ids of the player ("n=3 [a,b,c]", at most 12 listed).
public func UTPPRdEffects(player: ref<PlayerPuppet>) -> String {
  let sys: ref<StatusEffectSystem> = GameInstance.GetStatusEffectSystem(player.GetGame());
  if !IsDefined(sys) {
    return "n=? [no system]";
  }
  let effects: array<ref<StatusEffect>>;
  sys.GetAppliedEffects(player.GetEntityID(), effects);
  let count: Int32 = ArraySize(effects);
  let out: String = "";
  let i: Int32 = 0;
  while i < count && i < 12 {
    if i > 0 {
      out += ",";
    }
    if IsDefined(effects[i]) && IsDefined(effects[i].GetRecord()) {
      out += TDBID.ToStringDEBUG(effects[i].GetRecord().GetID());
    } else {
      out += "?";
    }
    i += 1;
  }
  return "n=" + IntToString(count) + " [" + out + "]";
}

public func UTPPRdPos(player: ref<PlayerPuppet>) -> String {
  return UTPPRdVec(player.GetWorldPosition());
}

public class UTPPRdTick extends DelayCallback {
  public let seq: Int32;
  public let n: Int32;
  public let t0: Float;
  public let prevDetailed: Int32;
  public let knockdownLogged: Bool;

  public func Call() -> Void {
    let game: GameInstance = GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) || player.utppRdSeq != this.seq {
      return;
    }
    let now: Float = UTPPRdNow(game);
    let dt: Float = now - this.t0;
    if UTPPRdUnsafe(player) {
      UTPPRdEnd(player, "unsafe(vehicle/psm) n=" + IntToString(this.n), dt);
      return;
    }
    let detailed: Int32 = UTPPRdDetailed(player);
    this.n += 1;
    UTPPLogLine("[utpp_ragdoll] carhit+t seq=" + IntToString(this.seq) + " dt=" + FloatToStringPrec(dt, 2) + " " + UTPPRdPsm(player)
      + " fx=" + UTPPRdEffects(player) + " pos=" + UTPPRdPos(player));
    if detailed == EnumInt(gamePSMDetailedLocomotionStates.Knockdown) && !this.knockdownLogged {
      this.knockdownLogged = true;
      UTPPLogLine("[utpp_ragdoll] knockdown seq=" + IntToString(this.seq) + " dt=" + FloatToStringPrec(dt, 2) + " prevDet="
        + IntToString(this.prevDetailed) + " " + UTPPRdPsm(player));
    }
    if dt >= 3.0 || this.n >= 32 || now >= player.utppRdEnd {
      UTPPRdEnd(player, "window n=" + IntToString(this.n), dt);
      return;
    }
    let next: ref<UTPPRdTick> = new UTPPRdTick();
    next.seq = this.seq;
    next.n = this.n;
    next.t0 = this.t0;
    next.prevDetailed = detailed;
    next.knockdownLogged = this.knockdownLogged;
    GameInstance.GetDelaySystem(game).DelayCallback(next, 0.10, false);
  }
}

public func UTPPRdEnd(player: ref<PlayerPuppet>, why: String, dt: Float) -> Void {
  let seq: Int32 = player.utppRdSeq;
  player.utppRdEnd = 0.0;
  UTPPLogLine("[utpp_ragdoll] carhit-end seq=" + IntToString(seq) + " " + why + " dt=" + FloatToStringPrec(dt, 2)
    + " suppressedHits=" + IntToString(player.utppRdSuppressed));
}

// Called after the vanilla body. kdBefore = VehicleKnockdown / BikeKnockdown already active when the event arrived (vanilla returns early then).
public func UTPPRdOnCarHit(player: ref<PlayerPuppet>, evt: ref<OnCarHitPlayer>, kdBefore: Bool) -> Void {
  let game: GameInstance = player.GetGame();
  let now: Float = UTPPRdNow(game);
  if now < player.utppRdEnd || now < player.utppRdCooldown {
    player.utppRdSuppressed += 1;
    return;
  }
  player.utppRdSeq += 1;
  player.utppRdEnd = now + 3.0;
  player.utppRdCooldown = now + 5.0;
  let suppressed: Int32 = player.utppRdSuppressed;
  player.utppRdSuppressed = 0;

  let car: String = "none";
  let vehicle: ref<VehicleObject> = GameInstance.FindEntityByID(game, evt.carId) as VehicleObject;
  if IsDefined(vehicle) {
    car = "rec=" + TDBID.ToStringDEBUG(vehicle.GetRecordID()) + " speed=" + FloatToStringPrec(Vector4.Length(vehicle.GetLinearVelocity()), 2)
      + " vel=" + UTPPRdVec(vehicle.GetLinearVelocity());
  }
  UTPPLogLine("[utpp_ragdoll] carhit seq=" + IntToString(player.utppRdSeq) + " t=" + FloatToStringPrec(now, 2)
    + " hitDir=" + UTPPRdVec(evt.hitDirection)
    + " seperationImpulse=" + UTPPRdVec(evt.seperationImpulse) + " impLen=" + FloatToStringPrec(Vector4.Length(evt.seperationImpulse), 2)
    + " carId=" + EntityID.ToDebugString(evt.carId) + " car=" + car
    + " pos=" + UTPPRdPos(player) + " yaw=" + FloatToStringPrec(UTPPYawDegFromForward(player.GetWorldForward()), 1)
    + " " + UTPPRdPsm(player) + " fx=" + UTPPRdEffects(player)
    + " kdBefore=" + (kdBefore ? "1" : "0") + " suppressedBefore=" + IntToString(suppressed));

  let first: ref<UTPPRdTick> = new UTPPRdTick();
  first.seq = player.utppRdSeq;
  first.n = 0;
  first.t0 = now;
  first.prevDetailed = UTPPRdDetailed(player);
  first.knockdownLogged = false;
  GameInstance.GetDelaySystem(game).DelayCallback(first, 0.10, false);
}

// player.script:5776. Observation only: the vanilla body runs first, its result is returned unchanged.
@wrapMethod(PlayerPuppet)
protected cb func OnCarHitPlayer(evt: ref<OnCarHitPlayer>) -> Bool {
  let kdBefore: Bool = false;
  let enabled: Bool = UTPPRagdollProbeEnabled() && IsDefined(evt);
  if enabled {
    kdBefore = StatusEffectSystem.ObjectHasStatusEffect(this, t"BaseStatusEffect.VehicleKnockdown")
      || StatusEffectSystem.ObjectHasStatusEffect(this, t"BaseStatusEffect.BikeKnockdown");
  }
  let r: Bool = wrappedMethod(evt);
  if enabled {
    UTPPRdOnCarHit(this, evt, kdBefore);
  }
  return r;
}

// ======================================================================================================================
// R1: engine-ragdoll PoC on the player. Everything below is passive until a CET / harness call:
//   Game.UTPPRagdollEngineStatus()                -> one status line (component, episode, UTPP / MM flags, PSM)
//   Game.UTPPRagdollEngineForce(mode, impulse)    -> mode 0 CreateForceRagdollEvent, 1 ...NoPowerPoseEvent, 2 Forced without momentum;
//                                                    impulse > 0 (0..500) also queues a RagdollApplyImpulseEvent 0.10 s later at the
//                                                    Hips slot along the player's forward (+30 % up), radius 10000 as the vanilla throw
//   Game.UTPPRagdollEngineStop()                  -> RagdollDisableEvent + restore of the UTPP flag, idempotent
// Episode: at most one, 8 s watchdog, the engine event is queued 0.30 s after the call (baseline sampling window before the force),
// 10 Hz samples "[utpp_ragdoll] eng t=.." of the Hips / Head slot world positions, entity position, hipsPolePitch (NPCPuppet
// UpdateAnimgraphRagdollState formula) and the hips -> head yaw. The player only receives the ragdoll events when an asset package adds
// an entRagdollComponent named "RagdollComponent" (+ ragdoll bodies on the rig); without it Force refuses.
// IMPORTANT: UTPPSetEnabled(false) (native Main.cpp Native_SetEnabled -> g_hot.enabled) only switches the legacy test-rotation write
// (FkHook.cpp: ApplyMotionMatching runs BEFORE that flag is read). MM v1 ("MM enabled=") and mm.motionDriver ("v2 enabled=") keep writing the
// pose; their state is logged at the start so a result can be judged. For an engine-only pose BOTH must be off (v2 off only hands the pose
// over to v1): GetMod("utpp_motion").SetEnabled(false) plus Game.UTPPSetMMEnabled(false); restore with SetEnabled(true) and
// GetMod("utpp_core").ReapplyMM("test"). Not done from here (the CET Sync loop re-asserts the glue state).
// Sources (2.31, the 2.5 symbols are checked by the offline 2.5 compile): animCommunication.script:1-36 (event factories),
// orphans.script:30043-30068 / :34641-34675 / :46432-46441 (event classes), NPCPuppet.script:1988 / :2022 / :2057, carriedObject.script:1132
// (vanilla impulse = direction * 30..100, radius 10000, queued 0.10 s after the force).
// ======================================================================================================================

@addField(PlayerPuppet)
public let utppRdEngSeq: Int32;

@addField(PlayerPuppet)
public let utppRdEngActive: Bool;

@addField(PlayerPuppet)
public let utppRdEngStart: Float;

@addField(PlayerPuppet)
public let utppRdEngForceT: Float;

@addField(PlayerPuppet)
public let utppRdEngEnd: Float;

@addField(PlayerPuppet)
public let utppRdEngMode: Int32;

@addField(PlayerPuppet)
public let utppRdEngImpulse: Float;

@addField(PlayerPuppet)
public let utppRdEngPrevUtpp: Bool;

@addField(PlayerPuppet)
public let utppRdEngEvents: Int32;

// R1g comparison: an NPC near V forced into ragdoll at the same moment (its component and pose slot are logged next to V's).
@addField(PlayerPuppet)
public let utppRdEngNpc: wref<ScriptedPuppet>;

// R1i: comparison NPC spawned next to V for the episode (Codeware DynamicEntitySystem), deleted when the episode ends.
@addField(PlayerPuppet)
public let utppRdEngNpcId: EntityID;

// R1k: RagdollDisableEvent is queued on V and the comparison NPC this long after the force (once), so the rest of the episode shows
// whether the pose blends back to animation.
@addField(PlayerPuppet)
public let utppRdEngReleased: Bool;

public func UTPPRdEngReleaseS() -> Float {
  return 3.0;
}

public func UTPPRdEngRelease(player: ref<PlayerPuppet>, now: Float) -> Void {
  player.utppRdEngReleased = true;
  UTPPLogLine("[utpp_ragdoll] eng release pre " + UTPPRdEngSince(player, now) + " " + UTPPRdEngComp(player, true));
  UTPPLogLine("[utpp_ragdoll] eng rslot release " + UTPPRdEngSince(player, now) + " " + UTPPRagdollSlotDump(player.UTPPRdEngComponent()));
  if IsDefined(player.UTPPRdEngComponent()) {
    player.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdollEngRelease"));
  }
  let npc: ref<ScriptedPuppet> = player.utppRdEngNpc;
  if IsDefined(npc) {
    UTPPLogLine("[utpp_ragdoll] eng npc release pre " + UTPPRdEngSince(player, now) + " " + UTPPRdEngNpcLine(player, true));
    npc.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdollEngNpcRelease"));
  }
  UTPPLogLine("[utpp_ragdoll] eng release queued " + UTPPRdEngSince(player, now));
}

public func UTPPRdEngNpcRecord() -> TweakDBID {
  return t"Character.Judy";
}

public func UTPPRdEngNpcWaitS() -> Float {
  return 4.0;
}

public func UTPPRdEngSpawnNpc(player: ref<PlayerPuppet>) -> Void {
  let des: ref<DynamicEntitySystem> = GameInstance.GetDynamicEntitySystem();
  if !IsDefined(des) || !des.IsReady() {
    UTPPLogLine("[utpp_ragdoll] eng npc spawn skipped: DynamicEntitySystem not ready");
    return;
  }
  let fwd: Vector4 = player.GetWorldForward();
  let spec: ref<DynamicEntitySpec> = new DynamicEntitySpec();
  spec.recordID = UTPPRdEngNpcRecord();
  spec.position = player.GetWorldPosition() + Vector4(fwd.X * 2.5, fwd.Y * 2.5, 0.0, 0.0);
  spec.orientation = player.GetWorldOrientation();
  spec.persistState = false;
  spec.persistSpawn = false;
  spec.alwaysSpawned = true;
  spec.spawnInView = true;
  spec.active = true;
  ArrayPush(spec.tags, n"UTPPRagdollEngNpc");
  player.utppRdEngNpcId = des.CreateEntity(spec);
  UTPPLogLine("[utpp_ragdoll] eng npc spawn requested id=" + (EntityID.IsDefined(player.utppRdEngNpcId) ? "ok" : "none")
    + " record=" + TDBID.ToStringDEBUG(UTPPRdEngNpcRecord()) + " at=" + UTPPRdVec(spec.position));
}

// The spawned comparison NPC once it exists (None while pending / without a spawn).
public func UTPPRdEngSpawnedNpc(player: ref<PlayerPuppet>) -> ref<ScriptedPuppet> {
  if !EntityID.IsDefined(player.utppRdEngNpcId) {
    return null;
  }
  let des: ref<DynamicEntitySystem> = GameInstance.GetDynamicEntitySystem();
  if !IsDefined(des) || !des.IsSpawned(player.utppRdEngNpcId) {
    return null;
  }
  return des.GetEntity(player.utppRdEngNpcId) as ScriptedPuppet;
}

// The force waits (at most UTPPRdEngNpcWaitS after the start) until a requested comparison NPC exists.
public func UTPPRdEngNpcReady(player: ref<PlayerPuppet>, now: Float) -> Bool {
  if !EntityID.IsDefined(player.utppRdEngNpcId) || now - player.utppRdEngStart >= UTPPRdEngNpcWaitS() {
    return true;
  }
  return IsDefined(UTPPRdEngSpawnedNpc(player));
}

public func UTPPRdEngDeleteNpc(player: ref<PlayerPuppet>) -> Void {
  if !EntityID.IsDefined(player.utppRdEngNpcId) {
    return;
  }
  let des: ref<DynamicEntitySystem> = GameInstance.GetDynamicEntitySystem();
  let ok: Bool = IsDefined(des) && des.DeleteEntity(player.utppRdEngNpcId);
  UTPPLogLine("[utpp_ragdoll] eng npc deleted=" + (ok ? "1" : "0"));
  player.utppRdEngNpcId = EntityID();
  player.utppRdEngNpc = null;
}

public func UTPPRdEngLeadS() -> Float {
  return 0.30;
}

public func UTPPRdEngMaxS() -> Float {
  return 8.0;
}

public func UTPPRdEngImpulseMax() -> Float {
  return 500.0;
}

// Native (MotionDriverNatives.cpp, read only): decoded activation fields of the entRagdollComponent (exe 2.5 offsets) + optional hex dump.
public native func UTPPRagdollCompDump(comp: ref<IComponent>, hex: Bool) -> String

// Native (ragdoll/RagdollSlotProbe.cpp, read only): the animation system's pose slot of this component (the source the
// "update ragdolls" job copies into the pose buffers) + a summary of every registered ragdoll (NPCs).
public native func UTPPRagdollSlotDump(comp: ref<IComponent>) -> String

// Native (same file): owner EntityID hash of comp (other < 0) or of the other-th registered ragdoll that is not comp; 0 = none.
public native func UTPPRagdollEntityHash(comp: ref<IComponent>, other: Int32) -> Uint64

public func UTPPRdEngComp(player: ref<PlayerPuppet>, hex: Bool) -> String {
  return UTPPRagdollCompDump(player.UTPPRdEngComponent(), hex);
}

// R1d showed the request reaching the activator (req=1 mode=2 job=1) and failing on empty pose buffers (cntA=cntB=0). The vanilla
// body-drop path (carriedObject.script, Drop state OnEnter) queues RagdollRequestCollectAnimPoseEvent on the player before it enables
// the ragdoll; the episode now does the same from its start until the first ragdoll event.
// Nearest NPC that has a "RagdollComponent": first among the ragdolls the animation system has registered (native owner ids,
// trusted only when V's own id matches), then the targeting search (<= 30 m); None when there is none.
public func UTPPRdEngFindNpc(player: ref<PlayerPuppet>) -> ref<ScriptedPuppet> {
  let game: GameInstance = player.GetGame();
  let spawned: ref<ScriptedPuppet> = UTPPRdEngSpawnedNpc(player);
  if IsDefined(spawned) {
    UTPPLogLine("[utpp_ragdoll] eng npc using the spawned comparison NPC");
    return spawned;
  }
  let mine: ref<IComponent> = player.UTPPRdEngComponent();
  let selfHash: Uint64 = UTPPRagdollEntityHash(mine, -1);
  let selfOk: Bool = Cast<EntityID>(selfHash) == player.GetEntityID();
  let found: ref<ScriptedPuppet>;
  let foundD: Float = 1000000.0;
  let names: String = "";
  if selfOk {
    let k: Int32 = 0;
    while k < 64 {
      let h: Uint64 = UTPPRagdollEntityHash(mine, k);
      if !EntityID.IsDefined(Cast<EntityID>(h)) {
        break;
      }
      let ent: ref<ScriptedPuppet> = GameInstance.FindEntityByID(game, Cast<EntityID>(h)) as ScriptedPuppet;
      if IsDefined(ent) {
        let d: Float = Vector4.Distance(ent.GetWorldPosition(), player.GetWorldPosition());
        names += " " + NameToString(ent.GetClassName()) + "@" + FloatToStringPrec(d, 1) + (ent.IsDead() ? "(dead)" : "");
        if !ent.IsDead() && d < foundD {
          foundD = d;
          found = ent;
        }
      } else {
        names += " ?";
      }
      k += 1;
    }
  }
  UTPPLogLine("[utpp_ragdoll] eng npc registry selfIdOk=" + (selfOk ? "1" : "0") + " others:" + names);
  if IsDefined(found) {
    return found;
  }
  let query: TargetSearchQuery = TSQ_NPC();
  query.maxDistance = 30.0;
  let parts: array<TS_TargetPartInfo>;
  GameInstance.GetTargetingSystem(game).GetTargetParts(player, query, parts);
  let best: ref<ScriptedPuppet>;
  let bestD: Float = 1000000.0;
  let i: Int32 = 0;
  while i < ArraySize(parts) {
    let comp: ref<TargetingComponent> = TS_TargetPartInfo.GetComponent(parts[i]);
    if IsDefined(comp) {
      let npc: ref<ScriptedPuppet> = comp.GetEntity() as ScriptedPuppet;
      if IsDefined(npc) && npc != player && IsDefined(npc.FindComponentByName(n"RagdollComponent")) {
        let d: Float = Vector4.Distance(npc.GetWorldPosition(), player.GetWorldPosition());
        if d < bestD {
          bestD = d;
          best = npc;
        }
      }
    }
    i += 1;
  }
  return best;
}

public func UTPPRdEngNpcLine(player: ref<PlayerPuppet>, slot: Bool) -> String {
  let npc: ref<ScriptedPuppet> = player.utppRdEngNpc;
  if !IsDefined(npc) {
    return "npc=none";
  }
  let comp: ref<IComponent> = npc.FindComponentByName(n"RagdollComponent");
  let s: String = "npc " + UTPPRagdollCompDump(comp, false);
  if slot {
    s += " " + UTPPRagdollSlotDump(comp);
  }
  return s;
}

public func UTPPRdEngCollectPose(player: ref<PlayerPuppet>, why: String) -> Void {
  player.QueueEvent(new RagdollRequestCollectAnimPoseEvent());
  UTPPLogLine("[utpp_ragdoll] eng collect-pose " + why + " seq=" + IntToString(player.utppRdEngSeq) + " " + UTPPRdEngComp(player, false));
}

@addMethod(PlayerPuppet)
public func UTPPRdEngComponent() -> ref<IComponent> {
  return this.FindComponentByName(n"RagdollComponent");
}

// 1 / 0 for the flag "<key>0|1" found after <anchor> in <s> ("" = from the start); -1 = not found.
public func UTPPRdEngFlag(s: String, anchor: String, key: String) -> Int32 {
  let a: Int32 = 0;
  if NotEquals(anchor, "") {
    a = StrFindFirst(s, anchor);
    if a < 0 {
      return -1;
    }
  }
  let rest: String = StrMid(s, a, StrLen(s));
  let b: Int32 = StrFindFirst(rest, key);
  if b < 0 {
    return -1;
  }
  let c: String = StrMid(rest, b + StrLen(key), 1);
  if Equals(c, "1") {
    return 1;
  }
  if Equals(c, "0") {
    return 0;
  }
  return -1;
}

// Legacy test-rotation flag (UTPPSetEnabled). The anchor skips the free-text reason="..." field that comes earlier in the status.
public func UTPPRdEngUtppFlag() -> Int32 {
  return UTPPRdEngFlag(UTPPGetStatus(), "lastDescCount=", " enabled=");
}

public func UTPPRdEngMM1Flag() -> Int32 {
  return UTPPRdEngFlag(UTPPGetStatus(), "", "MM enabled=");
}

public func UTPPRdEngMM2Flag() -> Int32 {
  return UTPPRdEngFlag(UTPPGetMotionStatus(), "", "v2 enabled=");
}

public func UTPPRdEngFlags() -> String {
  return "utpp=" + IntToString(UTPPRdEngUtppFlag()) + " mm1=" + IntToString(UTPPRdEngMM1Flag()) + " mm2=" + IntToString(UTPPRdEngMM2Flag());
}

// World transform of the first slot (any SlotComponent of the player) whose slotName or boneName equals <name>.
public func UTPPRdEngSlotTransform(player: ref<PlayerPuppet>, name: CName, out world: WorldTransform) -> Bool {
  let components: array<ref<IComponent>> = player.GetComponents();
  let slots: ref<SlotComponent>;
  let c: Int32 = 0;
  let i: Int32;
  while c < ArraySize(components) && c < 128 {
    slots = components[c] as SlotComponent;
    if IsDefined(slots) {
      i = 0;
      while i < ArraySize(slots.slots) {
        if Equals(slots.slots[i].slotName, name) || Equals(slots.slots[i].boneName, name) {
          if slots.GetSlotTransform(slots.slots[i].slotName, world) {
            return true;
          }
        }
        i += 1;
      }
    }
    c += 1;
  }
  return false;
}

// Reasons not to force (empty = fine): everything UTPPRdUnsafe covers, scene tiers 2..5 / swimming (HighLevel > 1; 0 Default and 1 SceneTier1 = free gameplay, game test 2026-11-03), menu, photo mode.
public func UTPPRdEngUnsafe(player: ref<PlayerPuppet>) -> String {
  if UTPPRdUnsafe(player) {
    return "vehicle / braindance / no PSM";
  }
  let game: GameInstance = player.GetGame();
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  let high: Int32 = bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.HighLevel);
  if high > 1 {
    return "PSM HighLevel=" + IntToString(high) + " (scene / swimming)";
  }
  let photo: ref<PhotoModeSystem> = GameInstance.GetPhotoModeSystem(game);
  if IsDefined(photo) && photo.IsPhotoModeActive() {
    return "photo mode";
  }
  let ui: ref<IBlackboard> = GameInstance.GetBlackboardSystem(game).Get(GetAllBlackboardDefs().UI_System);
  if IsDefined(ui) && ui.GetBool(GetAllBlackboardDefs().UI_System.IsInMenu) {
    return "menu";
  }
  return "";
}

// "hips=(..) head=(..) ent=(..) pitch=.. yaw=.."; n/a where the slot / the geometry is missing.
public func UTPPRdEngPose(player: ref<PlayerPuppet>) -> String {
  let hips: WorldTransform;
  let head: WorldTransform;
  let okH: Bool = UTPPRdEngSlotTransform(player, n"Hips", hips);
  let okD: Bool = UTPPRdEngSlotTransform(player, n"Head", head);
  let hp: Vector4;
  let hd: Vector4;
  let out: String = "";
  if okH {
    hp = WorldPosition.ToVector4(WorldTransform.GetWorldPosition(hips));
    out += "hips=" + UTPPRdVec(hp);
  } else {
    out += "hips=n/a";
  }
  if okD {
    hd = WorldPosition.ToVector4(WorldTransform.GetWorldPosition(head));
    out += " head=" + UTPPRdVec(hd);
  } else {
    out += " head=n/a";
  }
  out += " ent=" + UTPPRdPos(player);
  if !okH || !okD {
    return out + " pitch=n/a yaw=n/a";
  }
  let toHead: Vector4 = hd - hp;
  if Vector4.Length(toHead) < 0.05 {
    return out + " pitch=n/a yaw=n/a";
  }
  let left: Vector4 = Vector4.Cross(Quaternion.GetForward(WorldTransform.GetOrientation(hips)), toHead);
  let fwd: Vector4 = Vector4.Cross(left, toHead);
  if Vector4.Length(left) < 0.0001 || Vector4.Length(fwd) < 0.0001 {
    out += " pitch=n/a";
  } else {
    out += " pitch=" + FloatToStringPrec(Vector4.GetAngleDegAroundAxis(Vector4.Normalize(fwd), Vector4(0.0, 0.0, 1.0, 0.0), Vector4.Normalize(left)) + 90.0, 1);
  }
  if SqrtF(toHead.X * toHead.X + toHead.Y * toHead.Y) < 0.05 {
    out += " yaw=n/a";
  } else {
    out += " yaw=" + FloatToStringPrec(UTPPYawDegFromForward(toHead), 1);
  }
  return out;
}

public func UTPPRdEngSince(player: ref<PlayerPuppet>, now: Float) -> String {
  if !player.utppRdEngActive {
    return "t=none";
  }
  if player.utppRdEngForceT < 0.0 {
    return "t=pre(" + FloatToStringPrec(now - player.utppRdEngStart, 2) + ")";
  }
  return "t=" + FloatToStringPrec(now - player.utppRdEngForceT, 2);
}

// Queues the engine event (and the optional impulse). Called once per episode from the tick, UTPPRdEngLeadS() after the Force call.
public func UTPPRdEngDoForce(player: ref<PlayerPuppet>, now: Float) -> Void {
  let game: GameInstance = player.GetGame();
  let mode: Int32 = player.utppRdEngMode;
  let evt: ref<RagdollActivationRequestEvent>;
  if mode == 1 {
    evt = CreateForceRagdollNoPowerPoseEvent(n"UTPPRagdollEng");
  } else {
    if mode == 2 {
      evt = CreateRagdollActivationRequestEvent(entragdollActivationRequestType.Forced, n"None", true, false, n"UTPPRagdollEng");
    } else {
      evt = CreateForceRagdollEvent(n"UTPPRagdollEng");
    }
  }
  player.utppRdEngForceT = now;
  player.utppRdEngEvents = 0;
  UTPPLogLine("[utpp_ragdoll] eng rcomp pre seq=" + IntToString(player.utppRdEngSeq) + " " + UTPPRdEngComp(player, true));
  UTPPLogLine("[utpp_ragdoll] eng rslot pre seq=" + IntToString(player.utppRdEngSeq) + " " + UTPPRagdollSlotDump(player.UTPPRdEngComponent()));
  UTPPRdEngCollectPose(player, "force");
  player.QueueEvent(evt);
  let npc: ref<ScriptedPuppet> = UTPPRdEngFindNpc(player);
  player.utppRdEngNpc = npc;
  if IsDefined(npc) {
    UTPPLogLine("[utpp_ragdoll] eng npc pre seq=" + IntToString(player.utppRdEngSeq) + " " + NameToString(npc.GetClassName()) + " d="
      + FloatToStringPrec(Vector4.Distance(npc.GetWorldPosition(), player.GetWorldPosition()), 1) + " canRagdoll="
      + (npc.CanRagdoll() ? "1" : "0") + " " + UTPPRdEngNpcLine(player, true));
    let npcp: ref<NPCPuppet> = npc as NPCPuppet;
    if IsDefined(npcp) {
      npcp.SetDisableRagdoll(false, true, true);
    }
    npc.QueueEvent(new RagdollRequestCollectAnimPoseEvent());
    npc.QueueEvent(CreateForceRagdollEvent(n"UTPPRagdollEngNpc"));
  } else {
    UTPPLogLine("[utpp_ragdoll] eng npc pre seq=" + IntToString(player.utppRdEngSeq) + " none within 30 m");
  }
  UTPPLogLine("[utpp_ragdoll] eng rcomp queued seq=" + IntToString(player.utppRdEngSeq) + " " + UTPPRdEngComp(player, false));
  UTPPLogLine("[utpp_ragdoll] eng force seq=" + IntToString(player.utppRdEngSeq) + " mode=" + IntToString(mode)
    + " applyPowerPose=" + (evt.data.applyPowerPose ? "1" : "0") + " applyMomentum=" + (evt.data.applyMomentum ? "1" : "0")
    + " comp=" + (IsDefined(player.UTPPRdEngComponent()) ? "1" : "0") + " canRagdoll=" + (player.CanRagdoll() ? "1" : "0") + " "
    + UTPPRdPsm(player));
  let impulse: Float = player.utppRdEngImpulse;
  if impulse > 0.0 {
    let hips: WorldTransform;
    let pos: Vector4 = player.GetWorldPosition();
    let src: String = "entity";
    if UTPPRdEngSlotTransform(player, n"Hips", hips) {
      pos = WorldPosition.ToVector4(WorldTransform.GetWorldPosition(hips));
      src = "hips";
    }
    let fwd: Vector4 = player.GetWorldForward();
    let dir: Vector4 = Vector4(fwd.X * impulse, fwd.Y * impulse, 0.3 * impulse, 0.0);
    GameInstance.GetDelaySystem(game).DelayEvent(player, CreateRagdollApplyImpulseEvent(pos, dir, 10000.0), 0.10, false);
    UTPPLogLine("[utpp_ragdoll] eng impulse queued +0.10s at=" + src + " pos=" + UTPPRdVec(pos) + " value=" + UTPPRdVec(dir));
  }
}

// Ends the episode (idempotent): RagdollDisableEvent first, then the UTPP flag back to its remembered value.
public func UTPPRdEngFinish(player: ref<PlayerPuppet>, why: String) -> Bool {
  if !player.utppRdEngActive {
    return false;
  }
  let now: Float = UTPPRdNow(player.GetGame());
  let t: String = UTPPRdEngSince(player, now);
  player.utppRdEngActive = false;
  player.utppRgProbeBusy = false;
  if IsDefined(player.UTPPRdEngComponent()) {
    player.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdollEng"));
  }
  UTPPSetEnabled(player.utppRdEngPrevUtpp);
  UTPPRdEngDeleteNpc(player);
  UTPPLogLine("[utpp_ragdoll] eng end seq=" + IntToString(player.utppRdEngSeq) + " why=" + why + " " + t + " events="
    + IntToString(player.utppRdEngEvents) + " utppRestoredTo=" + (player.utppRdEngPrevUtpp ? "1" : "0") + " " + UTPPRdEngFlags());
  return true;
}

public class UTPPRdEngTick extends DelayCallback {
  public let seq: Int32;
  public let n: Int32;

  public func Call() -> Void {
    let game: GameInstance = GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) || !player.utppRdEngActive || player.utppRdEngSeq != this.seq {
      return;
    }
    let now: Float = UTPPRdNow(game);
    if UTPPRdUnsafe(player) {
      UTPPRdEngFinish(player, "unsafe(vehicle/psm/braindance)");
      return;
    }
    if player.utppRdEngForceT < 0.0 && now - player.utppRdEngStart >= UTPPRdEngLeadS() && UTPPRdEngNpcReady(player, now) {
      UTPPRdEngDoForce(player, now);
    } else {
      if player.utppRdEngEvents == 0 {
        UTPPRdEngCollectPose(player, "tick");
      }
    }
    UTPPLogLine("[utpp_ragdoll] eng " + UTPPRdEngSince(player, now) + " " + UTPPRdEngPose(player));
    if player.utppRdEngForceT >= 0.0 && !player.utppRdEngReleased && now - player.utppRdEngForceT >= UTPPRdEngReleaseS() {
      UTPPRdEngRelease(player, now);
    }
    if player.utppRdEngForceT >= 0.0 {
      UTPPLogLine("[utpp_ragdoll] eng rcomp " + UTPPRdEngSince(player, now) + " " + UTPPRdEngComp(player, this.n % 10 == 0));
      if this.n % 5 == 0 {
        UTPPLogLine("[utpp_ragdoll] eng rslot " + UTPPRdEngSince(player, now) + " " + UTPPRagdollSlotDump(player.UTPPRdEngComponent()));
      }
      if IsDefined(player.utppRdEngNpc) {
        UTPPLogLine("[utpp_ragdoll] eng npc " + UTPPRdEngSince(player, now) + " " + UTPPRdEngNpcLine(player, this.n % 2 == 0));
      }
    }
    if now >= player.utppRdEngEnd || this.n >= 120 {
      UTPPRdEngFinish(player, "watchdog");
      return;
    }
    let next: ref<UTPPRdEngTick> = new UTPPRdEngTick();
    next.seq = this.seq;
    next.n = this.n + 1;
    GameInstance.GetDelaySystem(game).DelayCallback(next, 0.10, false);
  }
}

public func UTPPRagdollEngineStatus() -> String {
  let game: GameInstance = GetGameInstance();
  let player: ref<PlayerPuppet> = GetPlayer(game);
  if !IsDefined(player) {
    return "eng no player";
  }
  let comp: ref<IComponent> = player.UTPPRdEngComponent();
  let c: String = "comp=none";
  if IsDefined(comp) {
    c = "comp=" + NameToString(comp.GetClassName()) + " on=" + (comp.IsEnabled() ? "1" : "0");
  }
  // Native gate (exe 2.5 FUN_141072638): cached ragdoll component enabled + anim-system entry + rig with ragdoll bodies.
  c += " canRagdoll=" + (player.CanRagdoll() ? "1" : "0");
  c += " " + UTPPRdEngComp(player, false) + " " + UTPPRagdollSlotDump(player.UTPPRdEngComponent());
  let now: Float = UTPPRdNow(game);
  let ep: String = "idle";
  if player.utppRdEngActive {
    ep = "active seq=" + IntToString(player.utppRdEngSeq) + " mode=" + IntToString(player.utppRdEngMode) + " age="
      + FloatToStringPrec(now - player.utppRdEngStart, 2) + " " + UTPPRdEngSince(player, now);
  }
  let unsafe: String = UTPPRdEngUnsafe(player);
  return "eng " + c + " | " + ep + " events=" + IntToString(player.utppRdEngEvents) + " | " + UTPPRdEngFlags()
    + " | pos=" + UTPPRdPos(player) + " " + UTPPRdPsm(player) + " | unsafe=" + (Equals(unsafe, "") ? "no" : unsafe);
}

public func UTPPRagdollEngineForce(mode: Int32, impulse: Float) -> String {
  let game: GameInstance = GetGameInstance();
  let player: ref<PlayerPuppet> = GetPlayer(game);
  if !IsDefined(player) {
    return "refused: no player";
  }
  if player.utppRdEngActive {
    return "refused: episode seq=" + IntToString(player.utppRdEngSeq) + " is active (call UTPPRagdollEngineStop first)";
  }
  if player.utppRgPhase != 0 || UTPPRagdollState() != 0 {
    return "refused: a production ragdoll episode is active (utpp_ragdoll.reds)";
  }
  if mode < 0 || mode > 2 {
    return "refused: mode must be 0, 1 or 2";
  }
  if !IsDefined(player.UTPPRdEngComponent()) {
    return "refused: no RagdollComponent on the player entity (asset package not loaded)";
  }
  let unsafe: String = UTPPRdEngUnsafe(player);
  if NotEquals(unsafe, "") {
    return "refused: unsafe state: " + unsafe;
  }
  let imp: Float = impulse;
  if imp < 0.0 || imp != imp {
    imp = 0.0;
  }
  if imp > UTPPRdEngImpulseMax() {
    imp = UTPPRdEngImpulseMax();
  }
  let flags: String = UTPPRdEngFlags();
  let now: Float = UTPPRdNow(game);
  player.utppRdEngSeq += 1;
  player.utppRdEngActive = true;
  player.utppRgProbeBusy = true;
  player.utppRdEngStart = now;
  player.utppRdEngForceT = -1.0;
  player.utppRdEngReleased = false;
  player.utppRdEngEnd = now + UTPPRdEngMaxS();
  player.utppRdEngMode = mode;
  player.utppRdEngImpulse = imp;
  player.utppRdEngEvents = 0;
  player.utppRdEngPrevUtpp = UTPPRdEngUtppFlag() == 1;
  UTPPSetEnabled(false);
  UTPPLogLine("[utpp_ragdoll] eng start seq=" + IntToString(player.utppRdEngSeq) + " mode=" + IntToString(mode) + " impulse="
    + FloatToStringPrec(imp, 1) + " leadS=" + FloatToStringPrec(UTPPRdEngLeadS(), 2) + " before: " + flags);
  UTPPLogLine("[utpp_ragdoll] eng note UTPPSetEnabled(false) stops only the test rotation; mm1/mm2=1 above can still write the pose");
  UTPPLogLine("[utpp_ragdoll] eng " + UTPPRdEngPose(player) + " (baseline) " + UTPPRdPsm(player));
  UTPPRdEngCollectPose(player, "start");
  UTPPRdEngSpawnNpc(player);
  let first: ref<UTPPRdEngTick> = new UTPPRdEngTick();
  first.seq = player.utppRdEngSeq;
  first.n = 0;
  GameInstance.GetDelaySystem(game).DelayCallback(first, 0.10, false);
  return "started seq=" + IntToString(player.utppRdEngSeq) + " mode=" + IntToString(mode) + " impulse=" + FloatToStringPrec(imp, 1)
    + " force in " + FloatToStringPrec(UTPPRdEngLeadS(), 2) + "s, watchdog " + FloatToStringPrec(UTPPRdEngMaxS(), 0) + "s | " + flags;
}

public func UTPPRagdollEngineStop() -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return "no player";
  }
  if UTPPRdEngFinish(player, "stop") {
    return "stopped seq=" + IntToString(player.utppRdEngSeq) + " | " + UTPPRdEngFlags();
  }
  if IsDefined(player.UTPPRdEngComponent()) {
    player.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdollEng"));
    return "idle (disable queued) | " + UTPPRdEngFlags();
  }
  return "idle (no RagdollComponent)";
}

// Event log. Observation only (returns false: no consumer claim; the player has no vanilla handlers for these). Capped per episode.
public func UTPPRdEngEvent(player: ref<PlayerPuppet>, text: String) -> Void {
  // silent during production episodes; logs during probe episodes (Force) and when the probe gate is on
  if !UTPPRagdollProbeEnabled() && !player.utppRdEngActive {
    return;
  }
  player.utppRdEngEvents += 1;
  if player.utppRdEngEvents > 60 {
    return;
  }
  UTPPLogLine("[utpp_ragdoll] eng ev " + UTPPRdEngSince(player, UTPPRdNow(player.GetGame())) + " #" + IntToString(player.utppRdEngEvents) + " " + text);
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPRagdollNotifyEnabled(evt: ref<RagdollNotifyEnabledEvent>) -> Bool {
  UTPPRdEngEvent(this, "RagdollNotifyEnabled instigator=" + EntityID.ToDebugString(evt.instigator));
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPRagdollNotifyDisabled(evt: ref<RagdollNotifyDisabledEvent>) -> Bool {
  UTPPRdEngEvent(this, "RagdollNotifyDisabled");
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPRagdollImpact(evt: ref<RagdollImpactEvent>) -> Bool {
  let n: Int32 = ArraySize(evt.impactPoints);
  let s: String = "RagdollImpact points=" + IntToString(n) + " triggeredSim=" + (evt.triggeredSimulation ? "1" : "0");
  if IsDefined(evt.otherEntity) {
    s += " other=" + NameToString(evt.otherEntity.GetClassName());
  }
  if n > 0 {
    s += " p0=" + UTPPRdVec(WorldPosition.ToVector4(evt.impactPoints[0].worldPosition)) + " force=" + FloatToStringPrec(evt.impactPoints[0].forceMagnitude, 1)
      + " impulse=" + FloatToStringPrec(evt.impactPoints[0].impulseMagnitude, 1);
  }
  UTPPRdEngEvent(this, s);
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPAnimatedRagdollNotifyEnabled(evt: ref<AnimatedRagdollNotifyEnabledEvent>) -> Bool {
  UTPPRdEngEvent(this, "AnimatedRagdollNotifyEnabled instigator=" + EntityID.ToDebugString(evt.instigator));
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPAnimatedRagdollNotifyDisabled(evt: ref<AnimatedRagdollNotifyDisabledEvent>) -> Bool {
  UTPPRdEngEvent(this, "AnimatedRagdollNotifyDisabled");
  return false;
}

@addMethod(PlayerPuppet)
protected cb func OnUTPPRagdollVelocityTreshold(evt: ref<RagdollNotifyVelocityTresholdEvent>) -> Bool {
  UTPPRdEngEvent(this, "RagdollNotifyVelocityTreshold vel=" + UTPPRdVec(evt.velocity) + " speed=" + FloatToStringPrec(Vector4.Length(evt.velocity), 2));
  return false;
}

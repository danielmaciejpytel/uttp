// UTPP production ragdoll: V is knocked down by a car and falls as an engine ragdoll, then gets up with the game's knockdown
// recovery animation. Script layer of the native episode in src/ragdoll/RagdollEpisode.hpp (natives declared below).
//
// Flow of one episode (log prefix "[utpp_ragdoll]", one line per transition, no per-tick lines):
//   hit      @wrapMethod(PlayerPuppet) OnCarHitPlayer runs the vanilla body first (damage, BaseStatusEffect.VehicleKnockdown, 5 s
//            cooldown, sound, horn stay vanilla), then starts an episode when the vanilla body took its success path.
//   start    UTPPRagdollBegin(), HeadLook cleared, RagdollRequestCollectAnimPoseEvent now, the force request 0.05 s later and the
//            impulse 0.10 s after that (the sequence proven in game by a diagnostic probe). Force and impulse
//            are sent only while the knockdown effect exists (the first tick waits up to 0.35 s for it, then the episode aborts).
//   tick     DelayCallback chain at 10 Hz (UTPPRgTick): abort conditions, UTPPRagdollSettled, the release cap.
//   release  native get-up first (UTPPRgGetUpStart: plan from the last ragdoll pose, ground ray, free-spot check, begin, one teleport to
//            the anchor), then CreateDisableRagdollEvent + UTPPRagdollRelease(). Any refusal logs 'getup=vanilla why=<code>' on the release
//            line and the game's knockdown recovery clip is requested instead (see below); 'getup=utpp clip=<n> rms=<m>' otherwise.
//   getup    (native path) the tick follows UTPPRagdollGetUpState(): the knockdown effect is removed at the hand-off (state 3), the episode
//            ends when the state is back to 0 (done) or -1 (the native get-up aborted itself). A missing effect does not end it.
//   end      after a pushed recovery clip: VehicleKnockdown is removed so the locomotion state machine leaves the state; without a
//            push the vanilla effect ends by itself.
//   abort    mount / vehicle, death, scene tier / swimming, workspot, braindance, teleport (> max(3 m, 40 m/s * tick time)), episode timeout,
//            native episode dropped, knockdown effect missing, session change: Disable event + UTPPRagdollAbort(), idempotent.
//            Menu and photo mode do not abort (the game is frozen meanwhile); they only prevent a start.
//
// Evidence and decisions (vanilla 2.31 sources; the 2.5 bodies cannot be decompiled, names and signatures are checked by the
// offline 2.5 script compile):
//  * Trigger. player.script:5776-5829 OnCarHitPlayer returns false on both early exits (:5786 knockdown already applied, :5790
//    5 s cooldown) and has no return on the success path, so the return value cannot tell them apart. The success path stamps
//    m_vehicleKnockdownTimestamp (:5793): the wrap compares it before / after wrappedMethod and also requires that no knockdown
//    effect was applied before the call. The status effect itself arrives later through QueueHitEvent (:5806), not inside the call.
//  * Direction and impulse. RotateToKnockdownSource faces -direction and ApplyCounterForce pushes along +direction
//    (locomotionTransitions.script:5116-5164), so hitDirection / seperationImpulse (game spelling) point from the car to V.
//    The push uses the car velocity relative to V (VehicleObject.GetLinearVelocity minus the puppet velocity) when it is >= 1 m/s,
//    since the callback may run after the collision resolved the car's speed; else seperationImpulse, else hitDirection, else
//    -forward. Magnitude = clamp(3 * speed, 10, 60) * ragdoll.impulseScale, +25 % upward, applied at the Hips, radius 10000.
//  * PSMImpulse. OnCarHitPlayer queues PSMImpulse(seperationImpulse) on V (:5809-5814) and the knockdown state queues a counter
//    force on entry (locomotionTransitions.script:5138, tiny: impulseDistance 0.005). Both move V's capsule, the engine ragdoll
//    moves the entity root with the body horizontally. They point the same way as the impulse used here and cannot be
//    cancelled without replacing the vanilla method, so they are left alone (not suppressed).
//  * State during the episode. The vanilla car hit state is the generic status effect state "vehicleKnockdown"
//    (stateMachine_locomotion.tweak:502, StatusEffectDecisions / StatusEffectEvents, locomotionTransitions.script:5017-5288). It
//    blocks movement input, unequips the weapon and exits when the VehicleKnockdown effect is gone (ToStand :5055). Unlike the
//    Knockdown state it never sets LocomotionDetailed 29 (det=1 for the whole effect), so the MM gate
//    would keep writing a standing pose (utpp.reds UTPPPsmBlockMask: detail 1 / 3 / 4 open the gate). The game's blackboard is
//    not written: UTPPPsmBlockMask itself reports detail 29 (Knockdown, a native release detail) to the DLL while V has the
//    VehicleKnockdown or BikeKnockdown effect (UTPPRgKnockdownFeed), with or without a ragdoll episode, so the game's knockdown
//    and get-up clips are shown instead of the MM pose. CET reads the whole episode + get-up as Game.UTPPRagdollBusy().
//  * Lifetime. VehicleKnockdown lasts 4.1 s in the 2.31 tweak (knockdowns.tweak:77-93: one 4.1 s clip, startupAnimDuration 4.1, no
//    land / recovery durations) and the state exits when it ends. The 2.5 value is not assumed: the release budget is read at
//    runtime from the applied effect (UTPPRgEffectTime, remaining seconds). Re-applying the effect could re-enter the state and
//    restart the clip, and a TweakXL duration edit would change every vanilla car hit, so neither is used (no tweak file is needed).
//    The ragdoll is released at the latest when the effect's remaining time falls to the recovery length + 0.35 s (one 10 Hz hop,
//    its lateness and the 0.1 s end margin), so the recovery clip has its whole length before the effect ends; ragdoll.maxSeconds
//    (default 2.5 s, max 2.6 s) stays the upper bound on the time, a settle before either releases earlier. The recovery clip is
//    pushed only while the effect still has at least the recovery length left.
//  * Get-up. After the disable event the engine blends the ragdoll out over 0.5 s into whatever the graph plays. The graph plays
//    the vanilla clip started at the hit (the whole fall + get-up). The clip variant is chosen once, at state entry, from the hit
//    direction (SendStatusEffectAnimDataToGraph, :5144; the generic state never resends it), so it cannot follow the body's final
//    orientation. What can be steered: at release the script pushes the AnimFeature "StatusEffect" itself (state Recovery 4, the
//    direction of a hit from the front for a body face up / from behind for face down, the Knockdown record's recovery data,
//    StatusEffectHelper.PopulateStatusEffectAnimData, statusEffectSystem.script:366), the same data the Knockdown state sends.
//    The Knockdown record is used because VehicleKnockdown has no recovery durations. Face unknown (on the side) or ragdoll.getUpPush
//    off: nothing is pushed and the vanilla effect ends on its own (it is never removed by the script then). After a push the effect
//    is removed once the recovery clip had its whole length (the state exits and clears the feature); never earlier. UNVERIFIED in game: whether the player graph accepts the feature switch from the Vehicle variation,
//    how the variant names map to face up / down, and the blend from the lying ragdoll pose to the clip.
//  * Native get-up (ragdoll.getUp, default on). The pose after the release comes from get-up clips chosen by how V lies and
//    anchored in the world by the DLL (src/ragdoll/GetUpPlayer.hpp); the script only plans before the disable event (the plan needs the
//    last ragdoll pose), checks the ground and the free spot, teleports the entity once to the anchor and follows the native state. The
//    effect runs out mid get-up in most runs; the native hand-off on movement input and, when the game has one, a no-movement status
//    effect (UTPPRgGuRestrictOn) keep the vanilla state from moving the capsule away from the anchored pose.
//  * Not done: aligning the entity yaw to the body's pelvis -> head direction at release. The engine's world-space ragdoll
//    pose vs a changed entity frame cannot be judged offline and a wrong guess shows as a snap; the clip starts in the entity's
//    own frame.
// Requires utpp.reds (UTPPLogLine, UTPPHeadLook) and the natives of src/ragdoll/RagdollSlotProbe.cpp. Without an entRagdollComponent
// named "RagdollComponent" on V (asset package) nothing here acts: the vanilla knockdown plays unchanged.

public native func UTPPRagdollSetPlayer(entityIdHash: Uint64) -> Void
public native func UTPPRagdollBegin() -> Bool
public native func UTPPRagdollRelease() -> Void
public native func UTPPRagdollAbort() -> Void
public native func UTPPRagdollState() -> Int32
public native func UTPPRagdollPelvis() -> Vector4
public native func UTPPRagdollPelvisSpeed() -> Float
public native func UTPPRagdollFaceUp() -> Int32
public native func UTPPRagdollWorldSpeed(entityPos: Vector4, entityRot: Quaternion) -> Float
public native func UTPPRagdollSettled(entityPos: Vector4, entityRot: Quaternion) -> Bool
// Native get-up (src/ragdoll/GetUpPlayer.hpp, natives in src/MotionDriverNatives.cpp). Plan codes < 0: -1 unavailable, -2 no or old
// snapshot, -3 bones missing, -4 pose mismatch, -5 anchor too far, -6 busy. State: 0 idle, 1 blend, 2 play, 3 hand-off, -1 aborted.
public native func UTPPRagdollGetUpAvailable() -> Bool
public native func UTPPRagdollGetUpPlan(entityPos: Vector4, entityRot: Quaternion) -> Int32
public native func UTPPRagdollGetUpTarget() -> Vector4
public native func UTPPRagdollGetUpForward() -> Vector4
public native func UTPPRagdollGetUpBegin(groundZ: Float) -> Bool
public native func UTPPRagdollGetUpCancel() -> Void
public native func UTPPRagdollGetUpState() -> Int32
public native func UTPPRagdollGetUpStatus() -> String
public native func UTPPRagdollSetGetUp(enabled: Bool) -> Void

// ---------------------------------------------------------------------------------------------------- settings (per session)
// CET registry keys ragdoll.enabled / ragdoll.impulseScale / ragdoll.maxSeconds / ragdoll.getUpPush -> Game.UTPPRagdollSetConfig.
// Without a push the defaults below apply (disabled: the ragdoll asset package is not part of the release; the CET key
// ragdoll.enabled, developer level, turns it on for builds that carry the package).
public class UTPPRagdollConfig extends ScriptableSystem {
  public let m_enabled: Bool = false;
  public let m_impulseScale: Float = 1.0;
  public let m_maxSeconds: Float = 2.5;
  public let m_getUpPush: Bool = true;
  //  The native get-up (clips chosen by how V lies). Default true: when the native layer
  // refuses (data files missing, no snapshot, no room, ...) the vanilla recovery path below runs unchanged and the release line
  // says why. The CET key ragdoll.getUp (default on) overwrites it per session.
  public let m_getUp: Bool = true;
  public let m_pushes: Int32;
  // A status effect that blocks movement input during the get-up. Looked up once at runtime; applied only when found.
  public let m_restrictChecked: Bool;
  public let m_restrictFound: Bool;
  public let m_restrictId: TweakDBID;

  // A new session / save load: a native episode of the previous session must not survive (idempotent).
  private final func OnPlayerAttach(request: ref<PlayerAttachRequest>) -> Void {
    UTPPRagdollAbort();
    // The movement restriction is a status effect and persists in a save made during a get-up: take it off in every
    // new session / load (the record is looked up here when this session has not done it yet).
    UTPPRgGuRestrictProbe(this);
    let owner: ref<GameObject> = request.owner;
    if this.m_restrictFound && IsDefined(owner) {
      StatusEffectHelper.RemoveStatusEffect(owner, this.m_restrictId);
    }
  }
}

public func UTPPRagdollConfigOf(game: GameInstance) -> ref<UTPPRagdollConfig> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPRagdollConfig") as UTPPRagdollConfig;
}

// CET: Game.UTPPRagdollSetConfig(enabled, impulseScale, maxSeconds, getUpPush). False = no game session yet (nothing stored).
public func UTPPRagdollSetConfig(enabled: Bool, impulseScale: Float, maxSeconds: Float, getUpPush: Bool) -> Bool {
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(GetGameInstance());
  if !IsDefined(cfg) {
    return false;
  }
  cfg.m_enabled = enabled;
  cfg.m_impulseScale = ClampF(impulseScale, 0.25, 2.0);
  cfg.m_maxSeconds = ClampF(maxSeconds, 1.2, 2.6);
  cfg.m_getUpPush = getUpPush;
  cfg.m_pushes += 1;
  return true;
}

// CET: Game.UTPPRagdollSetGetUpEnabled(enabled) (key ragdoll.getUp). False = no game session yet (nothing stored). Its own function so
// UTPPRagdollSetConfig keeps its four arguments; the native UTPPRagdollSetGetUp has the contract name, so this one cannot share it.
public func UTPPRagdollSetGetUpEnabled(enabled: Bool) -> Bool {
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(GetGameInstance());
  if !IsDefined(cfg) {
    return false;
  }
  cfg.m_getUp = enabled;
  UTPPRagdollSetGetUp(enabled);
  return true;
}

// ---------------------------------------------------------------------------------------------------- episode state
// phase: 0 none, 1 ragdoll (force / impulse / settle), 2 get-up (released, waiting for the recovery clip)
@addField(PlayerPuppet)
public let utppRgPhase: Int32;

@addField(PlayerPuppet)
public let utppRgSeq: Int32;

@addField(PlayerPuppet)
public let utppRgT0: Float;

@addField(PlayerPuppet)
public let utppRgGetUpEnd: Float;

@addField(PlayerPuppet)
public let utppRgImpulse: Vector4;

@addField(PlayerPuppet)
public let utppRgHips: Vector4;

@addField(PlayerPuppet)
public let utppRgLastPos: Vector4;

@addField(PlayerPuppet)
public let utppRgLastTime: Float;

@addField(PlayerPuppet)
public let utppRgPushed: Bool;

@addField(PlayerPuppet)
public let utppRgFxLogged: Bool;

// The vanilla knockdown effect was seen in this episode (set by the first tick that finds it). The ragdoll camera starts only then:
// a hit whose knockdown the game refuses (landing after a jump-out) aborts without a camera dip.
@addField(PlayerPuppet)
public let utppRgConfirmed: Bool;

@addField(PlayerPuppet)
public let utppRgNoCompLogged: Bool;

@addField(PlayerPuppet)
public let utppRgRefusedLogged: Bool;

// A diagnostic probe has an engine episode running: production refuses while it does.
@addField(PlayerPuppet)
public let utppRgProbeBusy: Bool;

// The native get-up owns this episode's pose after the release (set when UTPPRagdollGetUpBegin succeeded). Reset by every start.
@addField(PlayerPuppet)
public let utppRgUtppGetUp: Bool;

// Native get-up state was non-zero at least once; the end is then state 0 (done) or -1 (aborted).
@addField(PlayerPuppet)
public let utppRgGuSeen: Bool;

// The hand-off step ran: knockdown effect removed, movement restriction taken off.
@addField(PlayerPuppet)
public let utppRgGuHandoff: Bool;

// The no-movement status effect is applied on V.
@addField(PlayerPuppet)
public let utppRgGuRestrict: Bool;

@addField(PlayerPuppet)
public let utppRgGuT0: Float;

// bit 0: status line logged shortly after the begin, bit 1: status line logged at the hand-off
@addField(PlayerPuppet)
public let utppRgGuLogged: Int32;

public func UTPPRgNow(game: GameInstance) -> Float {
  return EngineTime.ToFloat(GameInstance.GetSimTime(game));
}

public func UTPPRgVec(v: Vector4) -> String {
  return "(" + FloatToStringPrec(v.X, 2) + "," + FloatToStringPrec(v.Y, 2) + "," + FloatToStringPrec(v.Z, 2) + ")";
}

// Four decimals: the get-up metric tool rebuilds the entity transform before and after the teleport from these.
public func UTPPRgVecP(v: Vector4) -> String {
  return "(" + FloatToStringPrec(v.X, 4) + "," + FloatToStringPrec(v.Y, 4) + "," + FloatToStringPrec(v.Z, 4) + ")";
}

// True while a UTPP ragdoll episode owns V's body: the script phase, or the native state (the native blend-out is still running
// after the script finished). HeadLook (utpp.reds) and the body turn (utpp_bodyturn.reds) stay off while it is true.
public func UTPPRgBusy(player: ref<PlayerPuppet>) -> Bool {
  return IsDefined(player) && (player.utppRgPhase != 0 || UTPPRagdollState() != 0);
}

@addMethod(PlayerPuppet)
public func UTPPRgComponent() -> ref<IComponent> {
  return this.FindComponentByName(n"RagdollComponent");
}

public func UTPPRgHasKnockdown(player: ref<PlayerPuppet>) -> Bool {
  return StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown");
}

// Total / remaining seconds of the applied VehicleKnockdown effect (the game's real value, not the 2.31 tweak file); -1 = not applied.
public func UTPPRgEffectTime(player: ref<PlayerPuppet>, remaining: Bool) -> Float {
  let sys: ref<StatusEffectSystem> = GameInstance.GetStatusEffectSystem(player.GetGame());
  if !IsDefined(sys) {
    return -1.0;
  }
  let effects: array<ref<StatusEffect>>;
  sys.GetAppliedEffects(player.GetEntityID(), effects);
  let i: Int32 = 0;
  while i < ArraySize(effects) {
    if IsDefined(effects[i]) && IsDefined(effects[i].GetRecord()) && Equals(effects[i].GetRecord().GetID(), t"BaseStatusEffect.VehicleKnockdown") {
      return remaining ? effects[i].GetRemainingDuration() : effects[i].GetTotalDuration();
    }
    i += 1;
  }
  return -1.0;
}

// World transform of the first slot whose slotName or boneName equals name (the Hips slot follows the ragdoll body).
public func UTPPRgSlotTransform(player: ref<PlayerPuppet>, name: CName, out world: WorldTransform) -> Bool {
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

// Hips world position; the entity position + 1 m up when the slot is missing.
public func UTPPRgHipsWorld(player: ref<PlayerPuppet>) -> Vector4 {
  let hips: WorldTransform;
  if UTPPRgSlotTransform(player, n"Hips", hips) {
    return WorldPosition.ToVector4(WorldTransform.GetWorldPosition(hips));
  }
  let p: Vector4 = player.GetWorldPosition();
  return Vector4(p.X, p.Y, p.Z + 1.0, 0.0);
}

// Reasons that END a running episode (empty = fine); checked every tick. Menu and photo mode are deliberately not among them: the
// game (and the ragdoll) is frozen meanwhile and the episode, including its get-up, continues afterwards.
public func UTPPRgAbortReason(player: ref<PlayerPuppet>) -> String {
  let game: GameInstance = player.GetGame();
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  if !IsDefined(bb) {
    return "no PSM";
  }
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  if bb.GetInt(psm.Vehicle) != 0 || bb.GetBool(psm.MountedToVehicle) {
    return "vehicle / mount";
  }
  if bb.GetInt(psm.Vitals) != 0 || player.IsDeadNoStatPool() {
    return "dead / defeated";
  }
  // HighLevel 0 Default and 1 SceneTier1 are free gameplay; 2..5 scene tiers, 6 swimming
  if bb.GetInt(psm.HighLevel) > 1 {
    return "scene / swimming";
  }
  if bb.GetInt(psm.Swimming) != 0 {
    return "swimming";
  }
  if bb.GetInt(psm.IsInWorkspot) != 0 {
    return "workspot";
  }
  let bd: ref<IBlackboard> = GameInstance.GetBlackboardSystem(game).Get(GetAllBlackboardDefs().Braindance);
  if IsDefined(bd) && bd.GetBool(GetAllBlackboardDefs().Braindance.IsActive) {
    return "braindance";
  }
  return "";
}

// Reasons not to START an episode (empty = fine): the abort reasons plus takedown, body carrying, photo mode and menu.
public func UTPPRgUnsafe(player: ref<PlayerPuppet>) -> String {
  let game: GameInstance = player.GetGame();
  let why: String = UTPPRgAbortReason(player);
  if NotEquals(why, "") {
    return why;
  }
  let bb: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  if bb.GetInt(psm.Takedown) != 0 {
    return "takedown";
  }
  if bb.GetInt(psm.BodyCarrying) != 0 || bb.GetBool(psm.Carrying) {
    return "carrying a body";
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

// Reasons not to start (empty = start). "disabled" is the setting, not worth a log line.
public func UTPPRgEligible(player: ref<PlayerPuppet>) -> String {
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(player.GetGame());
  if !IsDefined(cfg) || !cfg.m_enabled { // fail-closed: no config object = not eligible (the ragdoll stays off)
    return "disabled";
  }
  if !IsDefined(player.UTPPRgComponent()) {
    return "no RagdollComponent";
  }
  if player.utppRgProbeBusy {
    return "research probe episode active";
  }
  if player.utppRgPhase != 0 || UTPPRagdollState() != 0 {
    return "episode active";
  }
  return UTPPRgUnsafe(player);
}

// ---------------------------------------------------------------------------------------------------- MM feed / CET
// True while V lies or gets up after a car / bike hit: the vanilla knockdown effects (the state "vehicleKnockdown" leaves
// LocomotionDetailed on Stand). utpp.reds UTPPPsmBlockMask reports detail 29 to the DLL then, so MM releases the body.
public func UTPPRgKnockdownFeed(owner: ref<GameObject>) -> Bool {
  return IsDefined(owner) && (StatusEffectSystem.ObjectHasStatusEffect(owner, t"BaseStatusEffect.VehicleKnockdown")
    || StatusEffectSystem.ObjectHasStatusEffect(owner, t"BaseStatusEffect.BikeKnockdown"));
}

// CET (utpp_camera): a UTPP episode is running (or blending out) and its knockdown effect was confirmed.
public func UTPPRagdollConfirmed() -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance()) as PlayerPuppet;
  return IsDefined(player) && player.utppRgConfirmed && UTPPRgBusy(player);
}

// CET (utpp_camera): the ragdoll episode, the native blend-out or a vanilla car knockdown / get-up is running.
public func UTPPRagdollBusy() -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance()) as PlayerPuppet;
  return IsDefined(player) && (UTPPRgBusy(player) || UTPPRgKnockdownFeed(player));
}

// ---------------------------------------------------------------------------------------------------- get-up request
// The Knockdown record's recovery length (1.5 s in knockdowns.tweak:35); 1.5 when the record cannot be read.
public func UTPPRgRecoverySeconds() -> Float {
  let rec: ref<StatusEffect_Record> = TweakDBInterface.GetStatusEffectRecord(t"BaseStatusEffect.Knockdown");
  if IsDefined(rec) && IsDefined(rec.PlayerData()) && rec.PlayerData().RecoveryAnimDuration() > 0.1 {
    return rec.PlayerData().RecoveryAnimDuration();
  }
  return 1.5;
}

// Pushes AnimFeature "StatusEffect" (state Recovery) with the direction of a hit from the front (body face up: knocked onto the
// back) or from behind (face down), relative to V's current facing; the direction int is computed by the game's own helper.
public func UTPPRgPushRecovery(player: ref<PlayerPuppet>, faceUp: Int32) -> Bool {
  let rec: ref<StatusEffect_Record> = TweakDBInterface.GetStatusEffectRecord(t"BaseStatusEffect.Knockdown");
  if !IsDefined(rec) {
    return false;
  }
  let fwd: Vector4 = player.GetWorldForward();
  let dir: Vector4 = Vector4(fwd.X, fwd.Y, 0.0, 0.0);
  if faceUp == 1 {
    dir = Vector4(-fwd.X, -fwd.Y, 0.0, 0.0);
  }
  let feature: ref<AnimFeature_StatusEffect> = new AnimFeature_StatusEffect();
  StatusEffectHelper.PopulateStatusEffectAnimData(player, rec, EKnockdownStates.Recovery, dir, feature);
  AnimationControllerComponent.ApplyFeature(player, n"StatusEffect", feature);
  return true;
}

// The feature pushed by UTPPRgPushRecovery goes through the animation controller (AnimationControllerComponent.ApplyFeature); the
// state exit clears only its own copy through the PSM (StatusEffectEvents.DefaultOnExit: SetAnimationParameterFeature). Game run
// (MM off): after the effect was removed V got up, then dropped into a crouch with a hand at the head and stayed
// there while the PSM was back on Stand - the pushed Recovery feature was still set. A cleared feature on the same channel ends it.
public func UTPPRgClearPushedFeature(player: ref<PlayerPuppet>) -> Void {
  let feature: ref<AnimFeature_StatusEffect> = new AnimFeature_StatusEffect();
  feature.Clear();
  AnimationControllerComponent.ApplyFeature(player, n"StatusEffect", feature);
}

// ---------------------------------------------------------------------------------------------------- native get-up
// Plan code (< 0) as a short reason for the log.
public func UTPPRgGetUpPlanWhy(code: Int32) -> String {
  if code == -1 {
    return "plan-unavailable";
  }
  if code == -2 {
    return "no-snapshot";
  }
  if code == -3 {
    return "bones-missing";
  }
  if code == -4 {
    return "pose-mismatch";
  }
  if code == -5 {
    return "anchor-far";
  }
  if code == -6 {
    return "busy";
  }
  return "plan" + IntToString(code);
}

// The first of these records that exists is applied to V while the get-up runs, so the vanilla state cannot walk the capsule
// away from the world-anchored pose. None found = the native early hand-off on movement input is the only protection. The lookup result
// is logged once per session (log: "restriction probe").
public func UTPPRgGuRestrictProbe(cfg: ref<UTPPRagdollConfig>) -> Void {
  if IsDefined(cfg) && !cfg.m_restrictChecked {
    cfg.m_restrictChecked = true;
    let found: String = "";
    let candidates: array<TweakDBID>;
    ArrayPush(candidates, t"GameplayRestriction.NoMovement");
    ArrayPush(candidates, t"GameplayRestriction.MovementLocked");
    ArrayPush(candidates, t"GameplayRestriction.NoLocomotion");
    ArrayPush(candidates, t"BaseStatusEffect.NoMovement");
    let names: array<String>;
    ArrayPush(names, "GameplayRestriction.NoMovement");
    ArrayPush(names, "GameplayRestriction.MovementLocked");
    ArrayPush(names, "GameplayRestriction.NoLocomotion");
    ArrayPush(names, "BaseStatusEffect.NoMovement");
    let i: Int32 = 0;
    while i < ArraySize(candidates) {
      if IsDefined(TweakDBInterface.GetStatusEffectRecord(candidates[i])) {
        if !cfg.m_restrictFound {
          cfg.m_restrictFound = true;
          cfg.m_restrictId = candidates[i];
        }
        found = found + names[i] + " ";
      }
      i += 1;
    }
    UTPPLogLine("[utpp_ragdoll] restriction probe: defined=[" + found + "] used=" + (cfg.m_restrictFound ? "1" : "0"));
  }
}

public func UTPPRgGuRestrictOn(player: ref<PlayerPuppet>, cfg: ref<UTPPRagdollConfig>) -> Void {
  if !IsDefined(cfg) {
    return;
  }
  UTPPRgGuRestrictProbe(cfg);
  if cfg.m_restrictFound && !player.utppRgGuRestrict {
    StatusEffectHelper.ApplyStatusEffect(player, cfg.m_restrictId);
    player.utppRgGuRestrict = true;
  }
}

public func UTPPRgGuRestrictOff(player: ref<PlayerPuppet>) -> Void {
  if !player.utppRgGuRestrict {
    return;
  }
  player.utppRgGuRestrict = false;
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(player.GetGame());
  if IsDefined(cfg) && cfg.m_restrictFound {
    StatusEffectHelper.RemoveStatusEffect(player, cfg.m_restrictId);
  }
}

// Hand-off of the native get-up (state 3, or a missed state 3): the knockdown effect goes now so the locomotion state machine leaves the
// state while motion matching takes over, and the movement restriction ends. Idempotent.
public func UTPPRgGuHandoff(player: ref<PlayerPuppet>) -> Void {
  if player.utppRgGuHandoff {
    return;
  }
  player.utppRgGuHandoff = true;
  UTPPRgGuRestrictOff(player);
  let had: Bool = UTPPRgHasKnockdown(player);
  if had {
    StatusEffectHelper.RemoveStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown");
  }
  // fixed line, always logged: the metric starts its false-air window here (motion matching resumes at the hand-off)
  UTPPLogLine("[utpp_ragdoll] getup handoff seq=" + IntToString(player.utppRgSeq) + " effectRemoved=" + (had ? "1" : "0") + " state="
    + IntToString(UTPPRagdollGetUpState()));
}

// Ground height under the anchor (Static + Terrain, as the vehicle exit ray): from 0.35 m above the anchor z (near the body) down to
// 1.0 m below it; a hit above anchor z + 0.25 m is geometry over the body, not ground. False = none accepted (the native then uses its
// own ground estimate).
public func UTPPRgGuGroundZ(sqs: ref<SpatialQueriesSystem>, tgt: Vector4, out groundZ: Float) -> Bool {
  let filter: QueryFilter;
  QueryFilter.AddGroup(filter, n"Static");
  QueryFilter.AddGroup(filter, n"Terrain");
  let from: Vector4 = Vector4(tgt.X, tgt.Y, tgt.Z + 0.35, 1.0);
  let to: Vector4 = Vector4(tgt.X, tgt.Y, tgt.Z - 1.0, 1.0);
  let hit: TraceResult;
  if !sqs.SyncRaycastByQueryFilter(from, to, filter, hit) || !TraceResult.IsValid(hit) {
    return false;
  }
  if hit.position.Z > tgt.Z + 0.25 {
    return false;
  }
  groundZ = hit.position.Z;
  return true;
}

// Plan, check the spot, begin and teleport (before the disable event, see UTPPRgRelease). "" = the native get-up runs; otherwise the
// reason it does not (the vanilla recovery path then runs unchanged). clip / rms are filled once a plan exists.
public func UTPPRgGetUpStart(player: ref<PlayerPuppet>, cfg: ref<UTPPRagdollConfig>, out clip: Int32, out rms: Float) -> String {
  let game: GameInstance = player.GetGame();
  let want: Bool = !IsDefined(cfg) || cfg.m_getUp;
  UTPPRagdollSetGetUp(want);
  if !want {
    return "off";
  }
  if !UTPPRagdollGetUpAvailable() {
    return "unavailable";
  }
  let plan: Int32 = UTPPRagdollGetUpPlan(player.GetWorldPosition(), player.GetWorldOrientation());
  if plan < 0 {
    return UTPPRgGetUpPlanWhy(plan);
  }
  clip = plan;
  let tgt: Vector4 = UTPPRagdollGetUpTarget();
  let fwd: Vector4 = UTPPRagdollGetUpForward();
  rms = tgt.W;
  let sqs: ref<SpatialQueriesSystem> = GameInstance.GetSpatialQueriesSystem(game);
  if !IsDefined(sqs) {
    UTPPRagdollGetUpCancel();
    return "no-spatial-queries";
  }
  // ground under the anchor: a short ray that starts just above the lying body, so the top of a bench / kerb / planter above V is
  // not taken for the floor; without an accepted hit the native z (capsule / joints) is used
  let ground: Float = tgt.Z;
  let hasGround: Bool = UTPPRgGuGroundZ(sqs, tgt, ground);
  let feet: Vector4 = Vector4(tgt.X, tgt.Y, ground, 1.0);
  let rot: EulerAngles = Vector4.ToRotation(fwd);
  if !UTPPExitSpotFree(sqs, feet, EulerAngles.ToQuat(rot)) {
    UTPPRagdollGetUpCancel();
    return "blocked";
  }
  if !UTPPRagdollGetUpBegin(hasGround ? ground : -1000000.0) {
    UTPPRagdollGetUpCancel();
    return "begin-refused";
  }
  // entity transform before the teleport: the body trace is in the entity frame, the metric needs both transforms
  let prePos: Vector4 = player.GetWorldPosition();
  let preFwd: Vector4 = player.GetWorldForward();
  GameInstance.GetTeleportationFacility(game).Teleport(player, feet, rot);
  player.utppRgUtppGetUp = true;
  player.utppRgGuSeen = false;
  player.utppRgGuHandoff = false;
  player.utppRgGuLogged = 0;
  player.utppRgGuT0 = UTPPRgNow(game);
  UTPPRgGuRestrictOn(player, cfg);
  UTPPLogLine("[utpp_ragdoll] getup begin seq=" + IntToString(player.utppRgSeq) + " clip=" + IntToString(clip) + " pre="
    + UTPPRgVecP(prePos) + " preFwd=" + UTPPRgVecP(preFwd) + " feet=" + UTPPRgVecP(feet) + " fwd=" + UTPPRgVecP(fwd) + " ground="
    + (hasGround ? "ray" : "native") + " restriction=" + (player.utppRgGuRestrict ? "1" : "0") + " status="
    + UTPPRagdollGetUpStatus());
  return "";
}

// ---------------------------------------------------------------------------------------------------- end paths
// Idempotent. Everything the episode changed is restored: engine ragdoll off, native episode ended, PSM detail, HeadLook (CET
// re-applies it once the native state is idle).
public func UTPPRgAbort(player: ref<PlayerPuppet>, why: String) -> Void {
  if player.utppRgPhase == 0 {
    return;
  }
  let phase: Int32 = player.utppRgPhase;
  let dt: Float = UTPPRgNow(player.GetGame()) - player.utppRgT0;
  player.utppRgPhase = 0;
  if player.utppRgPushed {
    UTPPRgClearPushedFeature(player);
  }
  player.utppRgPushed = false;
  if IsDefined(player.UTPPRgComponent()) {
    player.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdollAbort"));
  }
  let guStatus: String = player.utppRgUtppGetUp ? " getupStatus=" + UTPPRagdollGetUpStatus() : "";
  UTPPRgGuRestrictOff(player);
  UTPPRagdollAbort();
  UTPPLogLine("[utpp_ragdoll] abort seq=" + IntToString(player.utppRgSeq) + " phase=" + IntToString(phase) + " why=" + why + " dt="
    + FloatToStringPrec(dt, 2) + guStatus);
}

// The native get-up aborted itself after the teleport: the knockdown effect is still on and nothing was pushed, so V would drop into
// the vanilla lying pose and never get up. Hand the episode to the vanilla recovery tail: push the Recovery feature and stay in phase 2
// until it had its length. False = nothing to recover from (no effect left, or the push failed): the caller just finishes.
public func UTPPRgGuAbortToVanilla(player: ref<PlayerPuppet>, why: String) -> Bool {
  UTPPRgGuRestrictOff(player);
  let hadEffect: Bool = UTPPRgHasKnockdown(player);
  let status: String = UTPPRagdollGetUpStatus();
  player.utppRgUtppGetUp = false;
  if !hadEffect {
    UTPPLogLine("[utpp_ragdoll] getup aborted (" + why + "): no knockdown effect left, finishing seq=" + IntToString(player.utppRgSeq)
      + " status=" + status);
    return false;
  }
  let face: Int32 = UTPPRagdollFaceUp();
  if face != 0 && face != 1 {
    face = 0;
  }
  let recovery: Float = UTPPRgRecoverySeconds();
  player.utppRgPushed = UTPPRgPushRecovery(player, face);
  player.utppRgGetUpEnd = UTPPRgNow(player.GetGame()) + recovery + 0.1;
  UTPPLogLine("[utpp_ragdoll] getup aborted (" + why + "): vanilla recovery pushed=" + (player.utppRgPushed ? "1" : "0") + " faceUp="
    + IntToString(face) + " seq=" + IntToString(player.utppRgSeq) + " status=" + status);
  return player.utppRgPushed;
}

public func UTPPRgFinish(player: ref<PlayerPuppet>, why: String) -> Void {
  if player.utppRgPhase == 0 {
    return;
  }
  let dt: Float = UTPPRgNow(player.GetGame()) - player.utppRgT0;
  player.utppRgPhase = 0;
  // only after a pushed recovery clip (UTPPRgRelease gave it its whole length): without a push the vanilla effect ends on its own.
  // The native get-up removes it at its hand-off (UTPPRgGuHandoff); a get-up that was aborted leaves it to the vanilla clip.
  let utpp: Bool = player.utppRgUtppGetUp;
  if utpp && NotEquals(why, "getup aborted") {
    UTPPRgGuHandoff(player);
  }
  UTPPRgGuRestrictOff(player);
  let hadEffect: Bool = UTPPRgHasKnockdown(player);
  let doRemove: Bool = hadEffect && player.utppRgPushed;
  if doRemove {
    StatusEffectHelper.RemoveStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown");
  }
  if player.utppRgPushed {
    UTPPRgClearPushedFeature(player);
  }
  UTPPLogLine("[utpp_ragdoll] end seq=" + IntToString(player.utppRgSeq) + " why=" + why + " dt=" + FloatToStringPrec(dt, 2)
    + " effectRemoved=" + (doRemove ? "1" : "0") + " effectLeft=" + (hadEffect && !doRemove ? "1" : "0") + " pushed="
    + (player.utppRgPushed ? "1" : "0") + " getup=" + (utpp ? "utpp" : "vanilla")
    + (utpp ? " getupStatus=" + UTPPRagdollGetUpStatus() : ""));
}

// Phase 1 -> 2.
public func UTPPRgRelease(player: ref<PlayerPuppet>, why: String, now: Float) -> Void {
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(player.GetGame());
  let face: Int32 = UTPPRagdollFaceUp();
  let hasEffect: Bool = UTPPRgHasKnockdown(player);
  let recovery: Float = UTPPRgRecoverySeconds();
  let remaining: Float = hasEffect ? UTPPRgEffectTime(player, true) : -1.0;
  // the recovery clip only while the real effect still lasts for its whole length (else the effect would cut it)
  let push: Bool = hasEffect && remaining >= recovery && (face == 0 || face == 1) && (!IsDefined(cfg) || cfg.m_getUpPush);
  // The native get-up first: its plan reads the last ragdoll pose, which the disable event below replaces. A refusal of any kind
  // (reason in the log line) leaves the vanilla recovery path below exactly as it was.
  let guClip: Int32 = -1;
  let guRms: Float = 0.0;
  let guWhy: String = UTPPRgGetUpStart(player, cfg, guClip, guRms);
  let utpp: Bool = Equals(guWhy, "");
  player.QueueEvent(CreateDisableRagdollEvent(n"UTPPRagdoll"));
  UTPPRagdollRelease();
  player.utppRgPushed = false;
  if push && !utpp {
    player.utppRgPushed = UTPPRgPushRecovery(player, face);
  }
  player.utppRgPhase = 2;
  if utpp {
    // the native get-up decides the end (UTPPRagdollGetUpState), the 15 s episode timeout of the tick stays as the guard
    player.utppRgGetUpEnd = now + 15.0;
  } else {
    if player.utppRgPushed {
      player.utppRgGetUpEnd = now + recovery + 0.1;
    } else {
      // the original clip plays on; the effect ends it on its own (real remaining time), a guard stops the phase shortly after
      player.utppRgGetUpEnd = now + MaxF(remaining, 0.0) + 0.1;
    }
  }
  UTPPLogLine("[utpp_ragdoll] release seq=" + IntToString(player.utppRgSeq) + " why=" + why + " t=" + FloatToStringPrec(now - player.utppRgT0, 2)
    + " faceUp=" + IntToString(face) + " effect=" + (hasEffect ? "1" : "0") + " effectRemaining="
    + FloatToStringPrec(remaining, 2) + " pushed=" + (player.utppRgPushed ? "1" : "0")
    + " pelvisSpeed=" + FloatToStringPrec(UTPPRagdollPelvisSpeed(), 2)
    + (utpp ? " getup=utpp clip=" + IntToString(guClip) + " rms=" + FloatToStringPrec(guRms, 3)
      : " getup=vanilla" + (guClip >= 0 ? " clip=" + IntToString(guClip) : "") + " why=" + guWhy));
}

// ---------------------------------------------------------------------------------------------------- tick
public class UTPPRgTick extends DelayCallback {
  public let seq: Int32;
  public let n: Int32;

  public func Call() -> Void {
    let game: GameInstance = GetGameInstance();
    let player: ref<PlayerPuppet> = GetPlayer(game);
    if !IsDefined(player) {
      UTPPRagdollAbort();
      return;
    }
    if player.utppRgSeq != this.seq || player.utppRgPhase == 0 {
      return;
    }
    let now: Float = UTPPRgNow(game);
    let dt: Float = now - player.utppRgT0;
    let why: String = UTPPRgAbortReason(player);
    if NotEquals(why, "") {
      UTPPRgAbort(player, why);
      return;
    }
    let pos: Vector4 = player.GetWorldPosition();
    // a speed limit, not a fixed 3 m: a hop is nominally 0.1 s but longer on a frame hitch (ragdoll activation, low FPS), and a fast
    // throw moves the root with the body (15 - 20 m/s)
    let hop: Float = MaxF(now - player.utppRgLastTime, 0.0);
    if Vector4.Distance(pos, player.utppRgLastPos) > MaxF(3.0, 40.0 * hop) {
      UTPPRgAbort(player, "teleport");
      return;
    }
    player.utppRgLastPos = pos;
    player.utppRgLastTime = now;
    if dt > 15.0 {
      UTPPRgAbort(player, "timeout");
      return;
    }
    let hasEffect: Bool = UTPPRgHasKnockdown(player);
    // the MM feed reports detail 29 while the effect exists (UTPPRgKnockdownFeed); it arrives a frame or two after the hit
    if hasEffect {
      player.utppRgConfirmed = true;
      if !player.utppRgFxLogged {
        player.utppRgFxLogged = true;
        UTPPLogLine("[utpp_ragdoll] effect seq=" + IntToString(player.utppRgSeq) + " t=" + FloatToStringPrec(dt, 2) + " total="
          + FloatToStringPrec(UTPPRgEffectTime(player, false), 2) + " remaining=" + FloatToStringPrec(UTPPRgEffectTime(player, true), 2));
      }
    }
    let nextN: Int32 = this.n + 1;
    let delay: Float = 0.10;
    if player.utppRgPhase == 1 {
      if this.n == 0 && !hasEffect {
        // force and impulse only for a confirmed knockdown: wait for the effect (it arrives a frame or two after the hit), then give up
        if dt >= 0.35 {
          UTPPRgAbort(player, "knockdown effect missing");
          return;
        }
        nextN = 0;
        delay = 0.05;
      } else {
        if this.n == 0 {
          UTPPRgCollectPose(player);
          player.QueueEvent(CreateForceRagdollNoPowerPoseEvent(n"UTPPRagdoll"));
        } else {
          if this.n == 1 {
            if !hasEffect {
              UTPPRgAbort(player, "knockdown effect lost");
              return;
            }
            player.QueueEvent(CreateRagdollApplyImpulseEvent(player.utppRgHips, player.utppRgImpulse, 10000.0));
          }
        }
        if this.n >= 2 {
          if dt >= 0.5 && !hasEffect {
            UTPPRgAbort(player, "knockdown effect missing");
            return;
          }
          if dt >= 1.3 && UTPPRagdollState() == 0 {
            UTPPRgAbort(player, "native episode dropped (gate not confirmed)");
            return;
          }
          let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(game);
          let cap: Float = 2.5;
          if IsDefined(cfg) {
            cap = cfg.m_maxSeconds;
          }
          // the effect's real remaining time: the recovery clip needs its whole length before the effect ends (+ one hop + margin).
          // GetRemainingDuration semantics are unverified in 2.5 (> 0 guards a reading of 0): a bad reading leaves the time cap + the vanilla end
          let remaining: Float = hasEffect ? UTPPRgEffectTime(player, true) : -1.0;
          let byEffect: Bool = hasEffect && remaining > 0.0 && remaining <= UTPPRgRecoverySeconds() + 0.35;
          if UTPPRagdollSettled(pos, player.GetWorldOrientation()) {
            UTPPRgRelease(player, "settled", now);
          } else {
            if dt >= cap {
              UTPPRgRelease(player, "cap", now);
            } else {
              if byEffect {
                UTPPRgRelease(player, "effect time", now);
              }
            }
          }
        }
      }
    } else {
      if player.utppRgUtppGetUp {
        // The native get-up decides: state 0 after it ran = done, -1 = it aborted itself (vehicle / death / lost feed ...). The
        // knockdown effect usually runs out mid get-up, so a missing effect does not end the phase here. The 15 s timeout above
        // is the guard.
        let gu: Int32 = UTPPRagdollGetUpState();
        if gu > 0 {
          player.utppRgGuSeen = true;
          if player.utppRgGuLogged == 0 && now - player.utppRgGuT0 >= 0.25 {
            player.utppRgGuLogged += 1;
            UTPPLogLine("[utpp_ragdoll] getup status seq=" + IntToString(player.utppRgSeq) + " " + UTPPRagdollGetUpStatus());
          }
        }
        if gu == 3 {
          UTPPRgGuHandoff(player);
          if player.utppRgGuLogged < 2 {
            player.utppRgGuLogged += 2;
            UTPPLogLine("[utpp_ragdoll] getup status seq=" + IntToString(player.utppRgSeq) + " " + UTPPRagdollGetUpStatus());
          }
        }
        if gu < 0 {
          if !UTPPRgGuAbortToVanilla(player, "native state -1") {
            UTPPRgFinish(player, "getup aborted");
            return;
          }
          // the vanilla branch of this tick chain takes over from the next tick
          let nextV: ref<UTPPRgTick> = new UTPPRgTick();
          nextV.seq = this.seq;
          nextV.n = nextN;
          GameInstance.GetDelaySystem(game).DelayCallback(nextV, delay, false);
          return;
        }
        if gu == 0 && player.utppRgGuSeen {
          UTPPRgFinish(player, "getup done");
          return;
        }
        // Begin succeeded, so the state must be non-zero within a tick or two; if it never is, give the vanilla path the body back
        if gu == 0 && !player.utppRgGuSeen && now - player.utppRgGuT0 > 1.0 {
          UTPPRagdollAbort();
          if !UTPPRgGuAbortToVanilla(player, "state never started") {
            UTPPRgFinish(player, "getup aborted");
            return;
          }
          let nextV: ref<UTPPRgTick> = new UTPPRgTick();
          nextV.seq = this.seq;
          nextV.n = nextN;
          GameInstance.GetDelaySystem(game).DelayCallback(nextV, delay, false);
          return;
        }
      } else {
        if now >= player.utppRgGetUpEnd || !hasEffect {
          UTPPRgFinish(player, now >= player.utppRgGetUpEnd ? "getup done" : "effect gone");
          return;
        }
      }
    }
    let next: ref<UTPPRgTick> = new UTPPRgTick();
    next.seq = this.seq;
    next.n = nextN;
    GameInstance.GetDelaySystem(game).DelayCallback(next, delay, false);
  }
}

public func UTPPRgCollectPose(player: ref<PlayerPuppet>) -> Void {
  player.QueueEvent(new RagdollRequestCollectAnimPoseEvent());
}

// ---------------------------------------------------------------------------------------------------- start
// Horizontal unit direction of the push + its speed from the relative car velocity / the event fields; see the header.
public func UTPPRgPush(player: ref<PlayerPuppet>, relVel: Vector4, sep: Vector4, hitDir: Vector4, out dir: Vector4, out speed: Float, out src: String) -> Void {
  let len: Float = SqrtF(relVel.X * relVel.X + relVel.Y * relVel.Y);
  if len >= 1.0 {
    dir = Vector4(relVel.X / len, relVel.Y / len, 0.0, 0.0);
    speed = len;
    src = "relVel";
    return;
  }
  len = SqrtF(sep.X * sep.X + sep.Y * sep.Y);
  if len >= 0.1 {
    dir = Vector4(sep.X / len, sep.Y / len, 0.0, 0.0);
    speed = MaxF(len, 6.0);
    src = "separation";
    return;
  }
  len = SqrtF(hitDir.X * hitDir.X + hitDir.Y * hitDir.Y);
  if len >= 0.01 {
    dir = Vector4(hitDir.X / len, hitDir.Y / len, 0.0, 0.0);
    speed = 6.0;
    src = "hitDirection";
    return;
  }
  let fwd: Vector4 = player.GetWorldForward();
  let flen: Float = MaxF(SqrtF(fwd.X * fwd.X + fwd.Y * fwd.Y), 0.0001);
  dir = Vector4(-fwd.X / flen, -fwd.Y / flen, 0.0, 0.0);
  speed = 6.0;
  src = "-forward";
}

// Starts an episode (caller checked UTPPRgEligible). false = refused by the native layer; the vanilla knockdown then plays alone.
public func UTPPRgStart(player: ref<PlayerPuppet>, relVel: Vector4, sep: Vector4, hitDir: Vector4, carSpeed: Float, source: String) -> Bool {
  let game: GameInstance = player.GetGame();
  if !UTPPRagdollBegin() {
    if !player.utppRgRefusedLogged {
      player.utppRgRefusedLogged = true;
      UTPPLogLine("[utpp_ragdoll] start refused by the native layer (no hook / no player id / not armed): vanilla knockdown only");
    }
    return false;
  }
  let dir: Vector4;
  let speed: Float;
  let src: String;
  UTPPRgPush(player, relVel, sep, hitDir, dir, speed, src);
  let scale: Float = 1.0;
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(game);
  if IsDefined(cfg) {
    scale = cfg.m_impulseScale;
  }
  let mag: Float = ClampF(3.0 * speed, 10.0, 60.0) * scale;
  player.utppRgImpulse = Vector4(dir.X * mag, dir.Y * mag, 0.25 * mag, 0.0);
  player.utppRgHips = UTPPRgHipsWorld(player);
  player.utppRgSeq += 1;
  player.utppRgPhase = 1;
  player.utppRgPushed = false;
  UTPPRgGuRestrictOff(player);
  player.utppRgUtppGetUp = false;
  player.utppRgGuSeen = false;
  player.utppRgGuHandoff = false;
  player.utppRgGuLogged = 0;
  player.utppRgFxLogged = false;
  player.utppRgConfirmed = false;
  player.utppRgT0 = UTPPRgNow(game);
  player.utppRgLastPos = player.GetWorldPosition();
  player.utppRgLastTime = player.utppRgT0;
  // HeadLook off: the look-at event is removed now and UTPPHeadLook.Apply refuses while UTPPRgBusy
  let head: ref<UTPPHeadLook> = GameInstance.GetScriptableSystemsContainer(game).Get(n"UTPPHeadLook") as UTPPHeadLook;
  if IsDefined(head) {
    head.Clear();
  }
  UTPPRgCollectPose(player);
  let first: ref<UTPPRgTick> = new UTPPRgTick();
  first.seq = player.utppRgSeq;
  first.n = 0;
  GameInstance.GetDelaySystem(game).DelayCallback(first, 0.05, false);
  UTPPLogLine("[utpp_ragdoll] start seq=" + IntToString(player.utppRgSeq) + " src=" + source + " push=" + src + " dir=" + UTPPRgVec(dir)
    + " speed=" + FloatToStringPrec(speed, 1) + " carSpeed=" + FloatToStringPrec(carSpeed, 1) + " impulse=" + FloatToStringPrec(mag, 1)
    + " hips=" + UTPPRgVec(player.utppRgHips) + " pos=" + UTPPRgVec(player.GetWorldPosition()));
  return true;
}

// ---------------------------------------------------------------------------------------------------- hit
// Called after the vanilla body of OnCarHitPlayer ran its success path.
public func UTPPRgOnCarHit(player: ref<PlayerPuppet>, evt: ref<OnCarHitPlayer>) -> Void {
  let why: String = UTPPRgEligible(player);
  if NotEquals(why, "") {
    if NotEquals(why, "disabled") {
      if Equals(why, "no RagdollComponent") {
        if !player.utppRgNoCompLogged {
          player.utppRgNoCompLogged = true;
          UTPPLogLine("[utpp_ragdoll] hit: no RagdollComponent on V (asset package not loaded): vanilla knockdown");
        }
      } else {
        UTPPLogLine("[utpp_ragdoll] hit ignored: " + why);
      }
    }
    return;
  }
  let relVel: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0);
  let carSpeed: Float = 0.0;
  let vehicle: ref<VehicleObject> = GameInstance.FindEntityByID(player.GetGame(), evt.carId) as VehicleObject;
  if IsDefined(vehicle) {
    let carVel: Vector4 = vehicle.GetLinearVelocity();
    carSpeed = Vector4.Length(carVel);
    relVel = carVel - player.GetVelocity();
  }
  UTPPRgStart(player, relVel, evt.seperationImpulse, evt.hitDirection, carSpeed, "carhit");
}

// player.script:5776. The vanilla body runs first and its result is returned unchanged.
@wrapMethod(PlayerPuppet)
protected cb func OnCarHitPlayer(evt: ref<OnCarHitPlayer>) -> Bool {
  if !IsDefined(evt) {
    return wrappedMethod(evt);
  }
  let kdBefore: Bool = StatusEffectSystem.ObjectHasStatusEffect(this, t"BaseStatusEffect.VehicleKnockdown")
    || StatusEffectSystem.ObjectHasStatusEffect(this, t"BaseStatusEffect.BikeKnockdown");
  let stampBefore: Float = this.m_vehicleKnockdownTimestamp;
  let r: Bool = wrappedMethod(evt);
  if !kdBefore && this.m_vehicleKnockdownTimestamp != stampBefore {
    UTPPRgOnCarHit(this, evt);
  }
  return r;
}

// ---------------------------------------------------------------------------------------------------- CET entry points
public func UTPPRagdollStatus() -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return "ragdoll: no player";
  }
  let cfg: ref<UTPPRagdollConfig> = UTPPRagdollConfigOf(GetGameInstance());
  let c: String = "cfg=none";
  if IsDefined(cfg) {
    c = "enabled=" + (cfg.m_enabled ? "1" : "0") + " impulseScale=" + FloatToStringPrec(cfg.m_impulseScale, 2) + " maxSeconds="
      + FloatToStringPrec(cfg.m_maxSeconds, 2) + " getUpPush=" + (cfg.m_getUpPush ? "1" : "0") + " getUp="
      + (cfg.m_getUp ? "1" : "0");
  }
  let unsafe: String = UTPPRgEligible(player);
  return "ragdoll: phase=" + IntToString(player.utppRgPhase) + " native=" + IntToString(UTPPRagdollState()) + " getupState="
    + IntToString(UTPPRagdollGetUpState()) + " seq="
    + IntToString(player.utppRgSeq) + " comp=" + (IsDefined(player.UTPPRgComponent()) ? "1" : "0") + " " + c + " eligible="
    + (Equals(unsafe, "") ? "yes" : unsafe);
}

// Test entry (CET: Game.UTPPRagdollDebugHit(dirDeg, speed)): the same episode path as a real car hit without a car. dirDeg = where
// the car comes from, in degrees counter-clockwise from V's facing (0 = in front, V is pushed backwards; 90 = from V's left; 180 = from
// behind); speed = car speed in m/s relative to V (clamped 1..40). The vanilla VehicleKnockdown effect is applied with the hit
// direction like the vanilla attack does, so the get-up path is identical. No damage, no cooldown stamp, no event is constructed
// (a synthetic OnCarHitPlayer with an empty carId would dereference a null vehicle in the vanilla body).
public func UTPPRagdollDebugHit(dirDeg: Float, speed: Float) -> String {
  let game: GameInstance = GetGameInstance();
  let player: ref<PlayerPuppet> = GetPlayer(game);
  if !IsDefined(player) {
    return "refused: no player";
  }
  let why: String = UTPPRgEligible(player);
  if NotEquals(why, "") {
    return "refused: " + why;
  }
  if StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.VehicleKnockdown")
      || StatusEffectSystem.ObjectHasStatusEffect(player, t"BaseStatusEffect.BikeKnockdown") {
    return "refused: knockdown effect already applied";
  }
  let v: Float = ClampF(speed, 1.0, 40.0);
  let fwd: Vector4 = player.GetWorldForward();
  let a: Float = Deg2Rad(dirDeg);
  // unit vector from V towards the car, then the push direction = the opposite
  let toCarX: Float = fwd.X * CosF(a) - fwd.Y * SinF(a);
  let toCarY: Float = fwd.X * SinF(a) + fwd.Y * CosF(a);
  let hit: Vector4 = Vector4(-toCarX, -toCarY, 0.0, 0.0);
  let relVel: Vector4 = Vector4(hit.X * v, hit.Y * v, 0.0, 0.0);
  GameInstance.GetStatusEffectSystem(game).ApplyStatusEffect(player.GetEntityID(), t"BaseStatusEffect.VehicleKnockdown",
    GameObject.GetTDBID(player), player.GetEntityID(), 1u, hit);
  if !UTPPRgStart(player, relVel, Vector4(0.0, 0.0, 0.0, 0.0), hit, v, "debug") {
    return "refused: native layer (vanilla knockdown applied)";
  }
  return "started seq=" + IntToString(player.utppRgSeq) + " dir=" + UTPPRgVec(hit) + " speed=" + FloatToStringPrec(v, 1);
}

// UTPP melee full body + melee aim tilt (melee goes full body, the strike leans toward the aim within
// natural human limits). DLL: mm/MeleeFullBody.hpp (lower-body handover in the motion matching feed, the melee guard) and
// mm/MeleeAim.hpp (spine tilt in the FK detour). Settings (CET utpp_core): "Melee full body" (UTPPSetMeleeFullBodySetting,
// default on), "Melee aim tilt" (UTPPSetMeleeAimParams 20 / 35 / 10, off = 0 / 0; default on) and the guard settings
// melee.guard / melee.guardFullBody / melee.guardAimTilt (UTPPSetMeleeGuard*Setting, all default off).
//
// This script sends, on a change and repeated every 2 s (wrap of LocomotionEventsTransition.OnUpdate):
//   * the full-body FLAG (UTPPSetMeleeFullBody): true only while
//       - UTPP wants its third-person body on foot (utpp_foot_tpp, utpp_body.reds UTPPSetFootTpp): the UTPP melee clips are
//         registered behind the same anim wrapper variable; without it the game plays its first-person melee set (static,
//         crouched legs) and the legs must stay on motion matching;
//       - the UTPP third-person body is active (UTPPIsTppActive) and V is not mounted;
//       - the weapon in AttachmentSlots.WeaponRight is no arm cyberware (Gorilla Arms inherit the fists anim variable
//         w_melee_fists: StrongArms : Base_Fists has no friendlyName of its own) and its anim variable (item record
//         friendlyName) is covered by the UTPP clips (UTPPMeleeFullBodyCoveredName);
//   * the full-body STATE MASK (UTPPSetMeleeFullBodyStates): PSM.MeleeWeapon states whose third-person clips exist, from the
//     per-weapon table UTPPMeleeFullBodyStates (key = anim variable of the drawn weapon; default = the attacks without
//     ThrowAttack), also sent when the drawn weapon changes;
//   * the GUARD flag (UTPPSetMeleeGuard): the same conditions with the guard list (UTPPMeleeGuardCoveredName: the weapon's own
//     third-person melee_idlepose must be installed; fists only for now);
// and, at most 20 Hz while a melee weapon is drawn in third person, the game's melee target (UTPPSetMeleeAimTarget: the
// vanilla melee target query DefaultTransition.GetTargetObject, hostile / neutral active NPC within 5 m, its feet).
// The DLL adds its own conditions (melee mask reason, standing, stick released and no sustained capsule speed, on the ground,
// base owns the pose) and fades both ways.

public native func UTPPSetMeleeFullBody(enabled: Bool) -> Void
public native func UTPPSetMeleeFullBodySetting(on: Bool) -> Void
public native func UTPPSetMeleeAimParams(maxUpDeg: Float, maxDownDeg: Float, smoothHz: Float) -> Bool
// "meleefull flag= setting= want= why= stick= ground= fastT= fastOff= airT= full= progress= aimW= switches= states= mwState=
//  guardFlag= guardSet= guardFull= guardTilt= guardWant= guardWhy= guardAirT= mask= guardSwitches= | meleeaim rig= fk= ..."
public native func UTPPMeleeStatus() -> String
// (review)
public native func UTPPSetMeleeFullBodyStates(mask: Int32) -> Bool
public native func UTPPSetMeleeGuard(enabled: Bool) -> Void
public native func UTPPSetMeleeGuardSetting(on: Bool) -> Void
public native func UTPPSetMeleeGuardFullSetting(on: Bool) -> Void
public native func UTPPSetMeleeGuardAimTiltSetting(on: Bool) -> Void
public native func UTPPSetMeleeAimTarget(feet: Vector4, valid: Bool) -> Void

// ---- coverage table: weapon anim variables with UTPP melee clips ----
// The game selects a weapon's animation set by its anim wrapper variable = the friendlyName of the weapon's TweakDB item record
// (vanilla SetAnimWrapperBasedOnItemFriendlyName / GetMeleeWeaponFriendlyName). Only the variables whose UTPP sets are packed and
// registered (standing lower body, planted feet, behind utpp_foot_tpp) get the full body; the item type is not enough (batons
// share Wea_OneHandedClub with w_melee_one_hand_blunt but use w_melee_baton; long blades use w_melee_one_hand_blunt).
// The list follows the installed clip export: only the variables packed for both rigs. The pack has fists, knife, katana,
// one-handed blunt and baton sets for both bodies. Other melee variables (two-handed blunt, hammer, ...) take the upper body
// from the game and keep the legs on motion matching.
public func UTPPMeleeFullBodyCoveredName(n: CName) -> Bool {
  return Equals(n, n"w_melee_fists") || Equals(n, n"w_melee_knife") || Equals(n, n"w_melee_katana")
    || Equals(n, n"w_melee_one_hand_blunt") || Equals(n, n"w_melee_baton");
}

// PSM.MeleeWeapon states (gamePSMMeleeWeapon, bit = 1 << state) that may take the full body, per weapon: a table keyed by the
// weapon's anim variable (the friendlyName key the game uses to pick the animation set, as UTPPMeleeFullBodyCoveredName; the item
// type is too coarse: batons share Wea_OneHandedClub with w_melee_one_hand_blunt but use their own set). Arm cyberware never reaches
// the table (UTPPMeleeFullBodyTick passes n"" for it: Gorilla Arms inherit w_melee_fists) and n"" / any other variable gets the
// default. Default = the attack states ComboAttack 11 .. EquipAttack 21 without ThrowAttack 19 (the throw clips were rejected by
// the export and stay vanilla) = 0x37F800 = 3667968 (UTPPMeleeStatesDefault).
// Add a state's bit to a variable's line only when the INSTALLED pack has third-person clips for it (coverage of that pack): Hold 6
// (64: hold_start_*, hold_loop_*), ChargedHold 7 (128), Block 8 (256: block_start, block, block_end), Parried 5 (32: parried_*),
// Deflect 10 (1024: block_deflect_*). The melee mask (upper body from the game) covers Parried 5 .. Deflect 10, the attacks and
// the 0.3 s release hold after them (spent in Idle 2), so Idle 2 would only add that release hold; Equipping 1 / Safe 3 /
// PublicSafe 4 have no effect. A new bit changes the behaviour with the guard settings off as well (e.g. Hold 6: the legs follow
// the hold clip from the button press instead of from the strike).
// Stage 2 of the guard (melee.guardFullBody) adds Idle 2 in the DLL; strikes start in Hold 6, so stage 2 needs Hold 6 /
// ChargedHold 7 on the fists line (otherwise the legs dip to motion matching at every strike start).
// Fists (NPC-based clip pack): the
// default + Parried 5 (parried_left / _right) + Hold 6 (hold_start_01, hold_start_offscreen_01 / _02, hold_loop_01 / _02) +
// ChargedHold 7 (no clip of its own: the same hold clips, mapping by name) + Block 8 (block_start, block, block_end) +
// Deflect 10 (block_deflect_left / _right, mapping by name) = 0x37FDE0 = 3669472; all of these clips are third-person in that
// pack. With a pack that keeps any of them vanilla, set the fists line back to 3667968 (or drop that bit). The two-body pack
// (pwa_fists + pma_fists) covers the same states for Male V; the female-only pack (pwa_fists) leaves
// Male V fists on the vanilla first-person clips.
// Knife: the default + Parried 5 + Hold 6 + ChargedHold 7 (no block for the knife) = 3668192. Katana, one-handed blunt and
// baton: the same states as the fists = 3669472. The per-strike parried clips of these weapons (parried_left_<strike> /
// parried_right_<strike>) are picked by the melee parried graph archive; without that archive drop Parried 5 (knife 3668160,
// the other three 3669440).
// Format: one line per variable between the UTPP_STATES_TABLE markers (read by the melee stage installer).
public func UTPPMeleeStatesDefault() -> Int32 {
  return 3667968;
}

public func UTPPMeleeFullBodyStates(n: CName) -> Int32 {
  // UTPP_STATES_TABLE begin
  if Equals(n, n"w_melee_fists") { return 3669472; }
  if Equals(n, n"w_melee_knife") { return 3668192; }
  if Equals(n, n"w_melee_katana") { return 3669472; }
  if Equals(n, n"w_melee_one_hand_blunt") { return 3669472; }
  if Equals(n, n"w_melee_baton") { return 3669472; }
  // UTPP_STATES_TABLE end
  return UTPPMeleeStatesDefault();
}

// Guard: the anim variables whose own third-person melee_idlepose (the graph's Ready / Idle pose) is installed. Fists only for
// now (the pose is being built; the user setting melee.guard stays off until it is accepted). Separate from the full-body list:
// every weapon set needs its own guard pose.
public func UTPPMeleeGuardCoveredName(n: CName) -> Bool {
  return Equals(n, n"w_melee_fists");
}

// The anim variable of a weapon: StringToName(friendlyName) of its item record; no record / empty friendlyName -> n"" (not
// covered).
public func UTPPMeleeWeaponAnimVariable(weapon: ref<WeaponObject>) -> CName {
  if !IsDefined(weapon) {
    return n"";
  }
  let rec: ref<Item_Record> = TweakDBInterface.GetItemRecord(ItemID.GetTDBID(weapon.GetItemID()));
  if !IsDefined(rec) {
    return n"";
  }
  let friendly: String = rec.FriendlyName();
  if StrLen(friendly) == 0 {
    return n"";
  }
  return StringToName(friendly);
}

// Arm cyberware weapons (Gorilla Arms, Mantis Blades, Monowire, Projectile Launch System): never the UTPP fists / blade clips.
// Gorilla Arms inherit the fists friendlyName (StrongArms : Base_Fists) - so the item type and the weapon tag decide, robust to
// the record variants (vanilla checks the type the same way: aiHitReactionTasks.script:393, meleeTransitions.script:1982).
public func UTPPMeleeIsArmCyberware(weapon: ref<WeaponObject>) -> Bool {
  if !IsDefined(weapon) {
    return false;
  }
  if weapon.WeaponHasTag(n"StrongArms") || weapon.WeaponHasTag(n"MantisBlades") {
    return true;
  }
  let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(ItemID.GetTDBID(weapon.GetItemID()));
  if !IsDefined(rec) || !IsDefined(rec.ItemType()) {
    return false;
  }
  let t: gamedataItemType = rec.ItemType().Type();
  return Equals(t, gamedataItemType.Cyb_StrongArms) || Equals(t, gamedataItemType.Cyb_MantisBlades)
    || Equals(t, gamedataItemType.Cyb_NanoWires) || Equals(t, gamedataItemType.Cyb_Launcher);
}

// The weapon in WeaponRight while V is on foot in UTPP third person (null otherwise).
public func UTPPMeleeTppWeapon(player: ref<PlayerPuppet>) -> ref<WeaponObject> {
  if !IsDefined(player) {
    return null;
  }
  let game: GameInstance = player.GetGame();
  if !UTPPIsTppActive(game) || VehicleComponent.IsMountedToVehicle(game, player) {
    return null;
  }
  return GameInstance.GetTransactionSystem(game).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
}

@addField(PlayerPuppet) public let utppMeleeFullSent: Int32;   // 0 = never sent, 1 = false, 2 = true
@addField(PlayerPuppet) public let utppMeleeFullSentAt: Float; // sim time of the last send
@addField(PlayerPuppet) public let utppMeleeGuardSent: Int32;  // 0 = never sent, 1 = false, 2 = true
@addField(PlayerPuppet) public let utppMeleeStatesSent: Int32; // last state mask sent (-1 = never)
@addField(PlayerPuppet) public let utppMeleeStatesInit: Bool;
@addField(PlayerPuppet) public let utppMeleeStatesVar: CName;  // table key of the last state mask sent (n"" = default)
@addField(PlayerPuppet) public let utppMeleeAimAt: Float;      // sim time of the last melee target send
@addField(PlayerPuppet) public let utppMeleeAimSent: Int32;    // 0 = nothing sent, 1 = invalid sent, 2 = a target sent

public func UTPPMeleeFullBodyWanted(player: ref<PlayerPuppet>) -> Bool {
  if !IsDefined(player) || !player.utppFootTppWanted {
    return false;
  }
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  if !IsDefined(weapon) || UTPPMeleeIsArmCyberware(weapon) {
    return false;
  }
  let variable: CName = UTPPMeleeWeaponAnimVariable(weapon);
  if Equals(variable, n"") {
    return false;
  }
  return UTPPMeleeFullBodyCoveredName(variable);
}

public func UTPPMeleeGuardWanted(player: ref<PlayerPuppet>) -> Bool {
  if !IsDefined(player) || !player.utppFootTppWanted {
    return false;
  }
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  if !IsDefined(weapon) || UTPPMeleeIsArmCyberware(weapon) {
    return false;
  }
  let variable: CName = UTPPMeleeWeaponAnimVariable(weapon);
  if Equals(variable, n"") {
    return false;
  }
  return UTPPMeleeGuardCoveredName(variable);
}

// The game's melee target for the melee aim tilt: at most 20 Hz while a melee weapon is drawn in third person (the DLL drops it
// after 0.3 s without a send); one 'invalid' when the target is lost or the weapon / view changes.
public func UTPPMeleeAimTargetTick(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>, now: Float) -> Void {
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  let target: ref<GameObject>;
  if IsDefined(weapon) && weapon.IsMelee() && IsDefined(scriptInterface) {
    if now - player.utppMeleeAimAt < 0.05 && now >= player.utppMeleeAimAt {
      return;
    }
    player.utppMeleeAimAt = now; // every query counts for the 20 Hz limit, also one that finds nothing
    let q: TargetSearchQuery = TSQ_NPC();
    q.maxDistance = 5.0;
    q.filterObjectByDistance = true;
    target = DefaultTransition.GetTargetObject(scriptInterface, q, 5.0);
  }
  if IsDefined(target) {
    player.utppMeleeAimSent = 2;
    UTPPSetMeleeAimTarget(target.GetWorldPosition(), true);
  } else {
    if player.utppMeleeAimSent != 1 {
      player.utppMeleeAimSent = 1;
      UTPPSetMeleeAimTarget(Vector4(0.0, 0.0, 0.0, 1.0), false);
    }
  }
}

public func UTPPMeleeFullBodyTick(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  if !IsDefined(player) {
    return;
  }
  let want: Bool = UTPPMeleeFullBodyWanted(player);
  let guard: Bool = UTPPMeleeGuardWanted(player);
  let code: Int32 = 1;
  if want {
    code = 2;
  }
  let guardCode: Int32 = 1;
  if guard {
    guardCode = 2;
  }
  // per-weapon state mask: table key = anim variable of the drawn weapon (n"" without a weapon / for arm cyberware -> default);
  // sent when the weapon (key) or the mask changes, and with the periodic send
  let variable: CName = n"";
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  if IsDefined(weapon) && !UTPPMeleeIsArmCyberware(weapon) {
    variable = UTPPMeleeWeaponAnimVariable(weapon);
  }
  let states: Int32 = UTPPMeleeFullBodyStates(variable);
  let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(player.GetGame()));
  let periodic: Bool = now - player.utppMeleeFullSentAt >= 2.0 || now < player.utppMeleeFullSentAt;
  if code != player.utppMeleeFullSent || guardCode != player.utppMeleeGuardSent || !player.utppMeleeStatesInit
      || states != player.utppMeleeStatesSent || !Equals(variable, player.utppMeleeStatesVar) || periodic {
    player.utppMeleeFullSent = code;
    player.utppMeleeGuardSent = guardCode;
    player.utppMeleeStatesSent = states;
    player.utppMeleeStatesVar = variable;
    player.utppMeleeStatesInit = true;
    player.utppMeleeFullSentAt = now;
    UTPPSetMeleeFullBodyStates(states);
    UTPPSetMeleeFullBody(want);
    UTPPSetMeleeGuard(guard);
  }
  UTPPMeleeAimTargetTick(player, scriptInterface, now);
}

// Strike turn signal (CET utpp_bodyturn, setting bodyturn.meleeStrikeTurn): where the game's fist strike will go.
// The game's melee hit starts at the FPP camera component and points at the targeting system's tracked target component when
// there is one (else along the game's camera). Capsule yaw is not part of it, so in third person (the capsule does not follow the
// camera while a strike runs) the fist can point elsewhere.
// Returns the world yaw in degrees (the UTPPYawDegFromForward convention, as the body yaw) from the FPP camera component to the
// tracked target while a UTPP third-person fists strike state runs (PSM.MeleeWeapon attack states 11..21 without ThrowAttack 19);
// 1000.0 = no signal: other weapons and arm cyberware, no UTPP third-person fists pack, first person, mounted, no tracked target,
// target farther than 5 m (vanilla melee target range) or closer than 0.3 m. The weapon check uses the guard list
// (UTPPMeleeGuardWanted, fists only), not the full-body list, which also covers weapons with their own strike directions.
public func UTPPMeleeStrikeYawOf(player: ref<PlayerPuppet>) -> Float {
  if !IsDefined(player) || !UTPPMeleeGuardWanted(player) {
    return 1000.0;
  }
  let game: GameInstance = player.GetGame();
  let bb: ref<IBlackboard> = GameInstance.GetBlackboardSystem(game).GetLocalInstanced(player.GetEntityID(), GetAllBlackboardDefs().PlayerStateMachine);
  if !IsDefined(bb) {
    return 1000.0;
  }
  let state: Int32 = bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.MeleeWeapon);
  if state < 11 || state > 21 || state == 19 {
    return 1000.0;
  }
  let tracked: wref<TargetingComponent> = GameInstance.GetTargetingSystem(game).GetTrackedTargetComponent(player);
  let fpp: ref<FPPCameraComponent> = player.GetFPPCameraComponent();
  if !IsDefined(tracked) || !IsDefined(fpp) {
    return 1000.0;
  }
  let toTarget: Vector4 = Matrix.GetTranslation(tracked.GetLocalToWorld()) - Matrix.GetTranslation(fpp.GetLocalToWorld());
  toTarget.Z = 0.0;
  toTarget.W = 0.0;
  let dist: Float = Vector4.Length2D(toTarget);
  if dist < 0.3 || dist > 5.0 {
    return 1000.0;
  }
  return UTPPYawDegFromForward(toTarget);
}

// CET: Game.UTPPMeleeStrikeYaw()
public func UTPPMeleeStrikeYaw() -> Float {
  return UTPPMeleeStrikeYawOf(GetPlayer(GetGameInstance()));
}

// Chained with the other wraps of the same method (redscript chains all wraps); every locomotion state reaches it (melee runs
// in the upper-body / melee layers while Locomotion stays in Stand / AimWalk / Crouch ...).
@wrapMethod(LocomotionEventsTransition)
public func OnUpdate(timeDelta: Float, stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(timeDelta, stateContext, scriptInterface);
  UTPPMeleeFullBodyTick(scriptInterface.executionOwner as PlayerPuppet, scriptInterface);
}

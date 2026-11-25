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
//       - the weapon in AttachmentSlots.WeaponRight is no excluded arm cyberware (monowire, projectile launcher)
//         and its clip variable (UTPPMeleeClipVariable of the item record friendlyName) is covered by the UTPP clips
//         (UTPPMeleeFullBodyCoveredName); the strong arms (in-hand variable w_strong_arms) play the UTPP fist clips; the arm
//         blades (mantis_blade) play their own UTPP clips for the states of their line in UTPPMeleeFullBodyStates;
//   * the full-body STATE MASK (UTPPSetMeleeFullBodyStates): PSM.MeleeWeapon states whose third-person clips exist, from the
//     per-weapon table UTPPMeleeFullBodyStates (key = clip variable of the drawn weapon; default = the attacks without
//     ThrowAttack), also sent when the drawn weapon changes;
//   * the GUARD flag (UTPPSetMeleeGuard): the same conditions with the guard list (UTPPMeleeGuardCoveredName: the weapon's own
//     third-person melee_idlepose must be installed; fists only for now, the strong arms through their clip variable, and for
//     them only after the first melee action since the draw: UTPPMeleeGuardNeedsAction / UTPPMeleeGuardActionNext);
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
// Melee full pose (standing strikes play the clip's hips, spine and legs): the drawn weapon's anim variable picks the clip
// section of the pose tables (n"" = none); the setting melee.fullPose switches the layer.
public native func UTPPSetMeleePoseWeapon(variable: CName) -> Void
public native func UTPPSetMeleeFullPoseSetting(on: Bool) -> Void

// ---- coverage table: weapon anim variables with UTPP melee clips ----
// The game selects a weapon's animation set by its anim wrapper variable = the friendlyName of the weapon's TweakDB item record
// (vanilla SetAnimWrapperBasedOnItemFriendlyName / GetMeleeWeaponFriendlyName). Only the variables whose UTPP sets are packed and
// registered (standing lower body, planted feet, behind utpp_foot_tpp) get the full body; the item type is not enough (batons
// share Wea_OneHandedClub with w_melee_one_hand_blunt but use w_melee_baton; long blades use w_melee_one_hand_blunt).
// The list follows the installed clip export: only the variables packed for both rigs. The pack has fists, knife, katana,
// one-handed blunt, baton, two-handed blunt and arm blade (mantis_blade) sets for both bodies. Other melee variables (hammer,
// ...) take the upper body from the game and keep the legs on motion matching.
public func UTPPMeleeFullBodyCoveredName(n: CName) -> Bool {
  return Equals(n, n"w_melee_fists") || Equals(n, n"w_melee_knife") || Equals(n, n"w_melee_katana")
    || Equals(n, n"w_melee_one_hand_blunt") || Equals(n, n"w_melee_baton") || Equals(n, n"w_two_hand_blunt")
    || Equals(n, n"mantis_blade");
}

// PSM.MeleeWeapon states (gamePSMMeleeWeapon, bit = 1 << state) that may take the full body, per weapon: a table keyed by the
// weapon's anim variable (the friendlyName key the game uses to pick the animation set, as UTPPMeleeFullBodyCoveredName; the item
// type is too coarse: batons share Wea_OneHandedClub with w_melee_one_hand_blunt but use their own set). The key is the clip
// variable (UTPPMeleeClipVariable: the strong arms use the fists line); excluded arm cyberware never reaches the table
// (UTPPMeleeFullBodyTick passes n"" for it) and n"" / any other variable gets the default. Default = the attack states
// ComboAttack 11 .. EquipAttack 21 without ThrowAttack 19 (the throw clips were rejected by the export and stay vanilla) =
// 0x37F800 = 3667968 (UTPPMeleeStatesDefault).
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
// Knife: the default + Parried 5 + Hold 6 + ChargedHold 7 (no block for the knife) = 3668192. Katana, one-handed blunt,
// baton and two-handed blunt: the same states as the fists = 3669472. The per-strike parried clips of these weapons
// (parried_left_<strike> / parried_right_<strike>) are picked by the melee parried graph archive; without that archive drop
// Parried 5 (knife 3668160, the other four 3669440).
// Arm blades (mantis_blade): only the states whose clips are all third-person on both bodies: Parried 5 (parried_left / _right) +
// Hold 6 + ChargedHold 7 (hold_start_01, hold_start_offscreen_01 / _02, hold_loop_01 / _02) + Block 8 (block_start, block, block_end)
// + Deflect 10 (block_deflect_left / _right) + ComboAttack 11 (combo_attack_01 .. 09) + StrongAttack 13 (heavy_attack_01 / _02)
// + SafeAttack 14 (safe_attack) + BlockAttack 15 (block_attack, block_attack_hit) + SprintAttack 16 + CrouchAttack 17 + JumpAttack 18
// + DeflectAttack 20 (combo_attack_01 .. 09_deflect and the safe, sprint, crouch, jump and block attack deflects) = 0x17EDE0 = 1568224.
// Draw (melee_equip) and holster (melee_unequip) are third-person on both bodies as well (Equipping 1 has no effect on the mask).
// The set has no final attack and no equip attack (12 and 21 stay out).
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
  if Equals(n, n"w_two_hand_blunt") { return 3669472; }
  if Equals(n, n"mantis_blade") { return 1568224; }
  // UTPP_STATES_TABLE end
  return UTPPMeleeStatesDefault();
}

// Guard: the clip variables whose own third-person melee_idlepose (the graph's Ready / Idle pose) is installed. Fists only for
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

// Clip variable: the anim variable whose UTPP clips play for a weapon's in-hand variable. The strong arms (w_strong_arms) play
// the UTPP fist sets (archive\utpp_melee_tpp.archive.xl registers them behind w_strong_arms), so they use the fists entries of
// every table here and the fists section of the melee full pose tables. Every other variable is its own clip variable.
public func UTPPMeleeClipVariable(n: CName) -> CName {
  if Equals(n, n"w_strong_arms") { return n"w_melee_fists"; }
  return n;
}

// Clip variable of a weapon (n"" = no record / no friendlyName).
public func UTPPMeleeWeaponClipVariable(weapon: ref<WeaponObject>) -> CName {
  return UTPPMeleeClipVariable(UTPPMeleeWeaponAnimVariable(weapon));
}

// Arm cyberware weapons without UTPP clips (Monowire, Projectile Launch System): never the UTPP fists / blade clips. The item
// type decides, robust to the record variants (vanilla checks the type the same way: aiHitReactionTasks.script:393,
// meleeTransitions.script:1982). The strong arms and the arm blades are not in this group: the strong arms play the UTPP fist
// clips (UTPPMeleeClipVariable), the arm blades their own clips (anim variable mantis_blade).
public func UTPPMeleeIsArmCyberware(weapon: ref<WeaponObject>) -> Bool {
  if !IsDefined(weapon) {
    return false;
  }
  let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(ItemID.GetTDBID(weapon.GetItemID()));
  if !IsDefined(rec) || !IsDefined(rec.ItemType()) {
    return false;
  }
  let t: gamedataItemType = rec.ItemType().Type();
  return Equals(t, gamedataItemType.Cyb_NanoWires) || Equals(t, gamedataItemType.Cyb_Launcher);
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
@addField(PlayerPuppet) public let utppMeleeGuardAction: Bool; // a melee action ran since the draw (UTPPMeleeGuardActionNext)

public func UTPPMeleeFullBodyWanted(player: ref<PlayerPuppet>) -> Bool {
  if !IsDefined(player) || !player.utppFootTppWanted {
    return false;
  }
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  if !IsDefined(weapon) || UTPPMeleeIsArmCyberware(weapon) {
    return false;
  }
  let variable: CName = UTPPMeleeWeaponClipVariable(weapon);
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
  let variable: CName = UTPPMeleeWeaponClipVariable(weapon);
  if Equals(variable, n"") {
    return false;
  }
  return UTPPMeleeGuardCoveredName(variable);
}

// Guard after the draw. The strong arms (in-hand variable w_strong_arms) share the fists guard, but after the draw the game holds the
// cyberware's own ready pose in Idle (clavicles pulled together on the third-person body) until the first melee action; after any
// melee action Idle shows the fist guard. So for them the guard flag waits for the first melee action since the draw (PSM.MeleeWeapon
// Parried 5 .. EquipAttack 21); until then the upper body stays on UTPP's own standing pose. The first strike then fades in like a
// strike without the guard (the upper mask fade), and the guard holds from its end on. Fists and every other variable: no wait.
public func UTPPMeleeGuardNeedsAction(animVariable: CName) -> Bool {
  return Equals(animVariable, n"w_strong_arms");
}

// The strong arms by item type (robust to the record variants, as UTPPMeleeIsArmCyberware).
public func UTPPMeleeIsStrongArms(weapon: ref<WeaponObject>) -> Bool {
  if !IsDefined(weapon) {
    return false;
  }
  let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(ItemID.GetTDBID(weapon.GetItemID()));
  if !IsDefined(rec) || !IsDefined(rec.ItemType()) {
    return false;
  }
  return Equals(rec.ItemType().Type(), gamedataItemType.Cyb_StrongArms);
}

// Next value of the "melee action since the draw" latch: cleared without a weapon that needs it, and when the weapon is not ready or
// being drawn (NotReady 0, Equipping 1, not reported -1: a new draw); set by a melee action state 5..21; otherwise kept (Idle 2,
// Safe 3, PublicSafe 4).
public func UTPPMeleeGuardActionNext(latched: Bool, needsAction: Bool, meleeState: Int32) -> Bool {
  if !needsAction || meleeState <= 1 {
    return false;
  }
  if meleeState >= 5 && meleeState <= 21 {
    return true;
  }
  return latched;
}

// The guard flag sent to the DLL: the guard list (UTPPMeleeGuardWanted) and, for a weapon that needs it, a melee action since the draw.
public func UTPPMeleeGuardFlag(wanted: Bool, needsAction: Bool, latched: Bool) -> Bool {
  return wanted && (!needsAction || latched);
}

// PSM.MeleeWeapon of the player (-1 = not reported).
public func UTPPMeleeWeaponStateOf(player: ref<PlayerPuppet>) -> Int32 {
  if !IsDefined(player) {
    return -1;
  }
  let bb: ref<IBlackboard> = GameInstance.GetBlackboardSystem(player.GetGame()).GetLocalInstanced(player.GetEntityID(), GetAllBlackboardDefs().PlayerStateMachine);
  if !IsDefined(bb) {
    return -1;
  }
  return bb.GetInt(GetAllBlackboardDefs().PlayerStateMachine.MeleeWeapon);
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
    UTPPMeleeEnemyTick(player, target, now);
  } else {
    player.utppMeleeEnemyNear = false;
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

// Arm IK of the base graph. With no IK request the base graph re-solves each arm toward its cached effector with the elbow hinge on
// the forearm's local Z. A clip whose forearm carries the hand's pronation as a roll of the forearm bone gets its elbow moved by
// that solve (a held block of the one-handed blunt weapon: tens of degrees). While a covered melee weapon is drawn in UTPP third
// person an IK request with weight 0 keeps that solve off for both arms; its priority is below the game's own requests.
@addField(PlayerPuppet) public let utppMeleeArmIkRight: ref<IKTargetAddEvent>;
@addField(PlayerPuppet) public let utppMeleeArmIkLeft: ref<IKTargetAddEvent>;

public func UTPPMeleeArmIkRequest(player: ref<PlayerPuppet>, chain: CName) -> ref<IKTargetAddEvent> {
  let ev: ref<IKTargetAddEvent> = new IKTargetAddEvent();
  ev.SetStaticTarget(player.GetWorldPosition());
  ev.request.weightPosition = 0.0;
  ev.request.weightOrientation = 0.0;
  ev.request.transitionIn = 0.0;
  ev.request.transitionOut = 0.0;
  ev.request.priority = -1000;
  ev.bodyPart = chain;
  player.QueueEvent(ev);
  return ev;
}

// wanted = a covered melee weapon is drawn in UTPP third person (UTPPMeleeFullBodyWanted). The requests are added once when that
// begins and removed when it ends (weapon put away, first person, mounted, the other animation pack). They are not renewed while it
// lasts: replacing a request moves the arm for one frame.
public func UTPPMeleeArmIkTick(player: ref<PlayerPuppet>, wanted: Bool) -> Void {
  if wanted {
    if !IsDefined(player.utppMeleeArmIkRight) {
      player.utppMeleeArmIkRight = UTPPMeleeArmIkRequest(player, n"ikRightArm");
    }
    if !IsDefined(player.utppMeleeArmIkLeft) {
      player.utppMeleeArmIkLeft = UTPPMeleeArmIkRequest(player, n"ikLeftArm");
    }
    return;
  }
  if IsDefined(player.utppMeleeArmIkRight) {
    IKTargetRemoveEvent.QueueRemoveIkTargetRemoveEvent(player, player.utppMeleeArmIkRight);
    player.utppMeleeArmIkRight = null;
  }
  if IsDefined(player.utppMeleeArmIkLeft) {
    IKTargetRemoveEvent.QueueRemoveIkTargetRemoveEvent(player, player.utppMeleeArmIkLeft);
    player.utppMeleeArmIkLeft = null;
  }
}

public func UTPPMeleeFullBodyTick(player: ref<PlayerPuppet>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  if !IsDefined(player) {
    return;
  }
  let want: Bool = UTPPMeleeFullBodyWanted(player);
  let guardWeapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  let guardNeedsAction: Bool = UTPPMeleeGuardNeedsAction(UTPPMeleeWeaponAnimVariable(guardWeapon)) || UTPPMeleeIsStrongArms(guardWeapon);
  player.utppMeleeGuardAction = UTPPMeleeGuardActionNext(player.utppMeleeGuardAction, guardNeedsAction, UTPPMeleeWeaponStateOf(player));
  let guard: Bool = UTPPMeleeGuardFlag(UTPPMeleeGuardWanted(player), guardNeedsAction, player.utppMeleeGuardAction);
  let code: Int32 = 1;
  if want {
    code = 2;
  }
  let guardCode: Int32 = 1;
  if guard {
    guardCode = 2;
  }
  // per-weapon state mask: table key = clip variable of the drawn weapon (n"" without a weapon / for excluded arm cyberware ->
  // default); the same key picks the section of the melee full pose tables;
  // sent when the weapon (key) or the mask changes, and with the periodic send
  let variable: CName = n"";
  let weapon: ref<WeaponObject> = UTPPMeleeTppWeapon(player);
  if IsDefined(weapon) && !UTPPMeleeIsArmCyberware(weapon) {
    variable = UTPPMeleeWeaponClipVariable(weapon);
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
    UTPPSetMeleePoseWeapon(variable);
    UTPPSetMeleeFullBody(want);
    UTPPSetMeleeGuard(guard);
  }
  UTPPMeleeArmIkTick(player, want);
  UTPPMeleeAimTargetTick(player, scriptInterface, now);
  // read by the Locomotion heartbeat together with the melee state of its own frame (utpp_locomotion.reds)
  player.utppFreeStrikeEligible = !player.utppMeleeFreeStrikeOff && UTPPMeleeGuardWanted(player) && !player.utppMeleeEnemyNear;
}

// ---- strikes in V's own direction (CET setting bodyturn.meleeFreeStrike, default on) ----
// Fists and strong arms (the guard list). With no enemy near, a strike does not turn V onto the camera: she keeps facing where
// she faces or runs. With an enemy near, the strike keeps the previous behaviour: V turns toward the camera and the tracked
// target. The game's melee hit follows the camera and its tracked target, never the capsule yaw, so "near" is judged on what the
// game can hit: its crosshair query or its tracked target, a live NPC that is not friendly, within a short ground distance of V.
@addField(PlayerPuppet) public let utppMeleeFreeStrikeOff: Bool; // the setting is off (a new session starts with it on)
@addField(PlayerPuppet) public let utppMeleeEnemyNear: Bool;
@addField(PlayerPuppet) public let utppMeleeEnemyAt: Float;      // sim time the near enemy was last seen

// m: 3.0 to count as near (the game's fist lock reaches about 2 m, plus a step), 3.5 to stay near
public func UTPPMeleeEnemyRange(near: Bool) -> Float {
  if near {
    return 3.5;
  }
  return 3.0;
}

// An enemy stays near for 0.5 s after the last sighting (the crosshair query drops a target for single frames in a swing).
public func UTPPMeleeEnemyNearNext(near: Bool, seen: Bool, sinceSeen: Float) -> Bool {
  if seen {
    return true;
  }
  return near && sinceSeen >= 0.0 && sinceSeen < 0.5;
}

public func UTPPMeleeIsEnemyWithin(player: ref<PlayerPuppet>, object: ref<GameObject>, range: Float) -> Bool {
  let puppet: ref<ScriptedPuppet> = object as ScriptedPuppet;
  if !IsDefined(puppet) || !ScriptedPuppet.IsActive(puppet) {
    return false;
  }
  if Equals(GameObject.GetAttitudeTowards(puppet, player), EAIAttitude.AIA_Friendly) {
    return false;
  }
  return Vector4.Distance2D(puppet.GetWorldPosition(), player.GetWorldPosition()) <= range;
}

public func UTPPMeleeEnemyTick(player: ref<PlayerPuppet>, queried: ref<GameObject>, now: Float) -> Void {
  let range: Float = UTPPMeleeEnemyRange(player.utppMeleeEnemyNear);
  let seen: Bool = UTPPMeleeIsEnemyWithin(player, queried, range);
  if !seen {
    let tracked: wref<TargetingComponent> = GameInstance.GetTargetingSystem(player.GetGame()).GetTrackedTargetComponent(player);
    if IsDefined(tracked) {
      seen = UTPPMeleeIsEnemyWithin(player, tracked.GetEntity() as GameObject, range);
    }
  }
  player.utppMeleeEnemyNear = UTPPMeleeEnemyNearNext(player.utppMeleeEnemyNear, seen, now - player.utppMeleeEnemyAt);
  if seen {
    player.utppMeleeEnemyAt = now;
  }
}

// CET: Game.UTPPSetMeleeFreeStrike(on); false without a player (CET sends it again)
public func UTPPSetMeleeFreeStrike(on: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  if !IsDefined(player) {
    return false;
  }
  player.utppMeleeFreeStrikeOff = !on;
  return true;
}

// Strike turn signal (CET utpp_bodyturn, setting bodyturn.meleeStrikeTurn): where the game's fist strike will go.
// The game's melee hit starts at the FPP camera component and points at the targeting system's tracked target component when
// there is one (else along the game's camera). Capsule yaw is not part of it, so in third person (the capsule does not follow the
// camera while a strike runs) the fist can point elsewhere.
// Returns the world yaw in degrees (the UTPPYawDegFromForward convention, as the body yaw) from the FPP camera component to the
// tracked target while a UTPP third-person fists strike state runs (PSM.MeleeWeapon attack states 11..21 without ThrowAttack 19);
// 1000.0 = no signal: other weapons and excluded arm cyberware, no UTPP third-person fists pack, first person, mounted, no tracked
// target, target farther than 5 m (vanilla melee target range) or closer than 0.3 m. The weapon check uses the guard list
// (UTPPMeleeGuardWanted: fists and strong arms), not the full-body list, which also covers weapons with their own strike
// directions.
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

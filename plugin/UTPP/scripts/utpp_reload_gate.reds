// Uses Codeware existing native import gameAnimFeature_TPPRepresentation.
public native func UTPPSetMMAuthoredReload(on: Bool) -> Void
// Last weapon is weak: do not retain unequipped objects just for gate cleanup.
@addField(PlayerPuppet)
public let utppNueReloadGateWeapon: wref<WeaponObject>;

// Called only by the authored reload episode policy. No automatic activation.
// Success means features were submitted, not that a component selected a clip.
public func UTPPNueReloadGrip(aimAtEntry: Bool) -> Int32 {
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(GetGameInstance());
  if IsDefined(glue) {
    return glue.ReloadPistolPose(aimAtEntry);
  }
  return 0;
}

// Refresh only native permission for an already-published, still-owned episode.
// Never republish graph features or restart the animation from onUpdate.
public func UTPPRefreshNueReloadPermission(aimAtEntry: Bool, twoHand: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  let weapon: ref<WeaponObject>;
  if !IsDefined(player) { return false; };
  weapon = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
  if !IsDefined(weapon) || !IsDefined(player.utppNueReloadGateWeapon) { return false; };
  if NotEquals(player.utppNueReloadGateWeapon.GetEntityID(), weapon.GetEntityID())
    || NotEquals(player.GetResolvedGenderName(), n"Female")
    || NotEquals(ItemID.GetTDBID(weapon.GetItemID()), t"Items.Preset_Nue_Default")
    || UTPPNueReloadGrip(aimAtEntry) != (twoHand ? UTPPMotionPosePistol2H() : UTPPMotionPosePistol1H()) {
    return false;
  };
  UTPPSetMMAuthoredReload(true);
  return true;
}

public func UTPPPublishNueReloadGate(on: Bool, aimAtEntry: Bool, twoHand: Bool) -> Bool {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  let weapon: ref<WeaponObject>;
  let previous: wref<WeaponObject>;
  let accepted: Bool = !on;
  let active: ref<gameAnimFeature_TPPRepresentation> = new gameAnimFeature_TPPRepresentation();
  let aim: ref<gameAnimFeature_TPPRepresentation> = new gameAnimFeature_TPPRepresentation();
  let grip: ref<gameAnimFeature_TPPRepresentation> = new gameAnimFeature_TPPRepresentation();
  let off: ref<gameAnimFeature_TPPRepresentation> = new gameAnimFeature_TPPRepresentation();
  // A feature submission can fail midway. Only publish the verified exact-reach
  // permission after every feature has been submitted successfully.
  UTPPSetMMAuthoredReload(false);
  if !IsDefined(player) {
    return false;
  };
  weapon = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
  previous = player.utppNueReloadGateWeapon;
  if on && Equals(player.GetResolvedGenderName(), n"Female") && IsDefined(weapon) {
    accepted = Equals(ItemID.GetTDBID(weapon.GetItemID()), t"Items.Preset_Nue_Default")
      && UTPPNueReloadGrip(aimAtEntry) == (twoHand ? UTPPMotionPosePistol2H() : UTPPMotionPosePistol1H());
  };
  active.IsActive = on && accepted;
  aim.IsActive = active.IsActive && aimAtEntry;
  grip.IsActive = active.IsActive && twoHand;
  off.IsActive = false;
  if IsDefined(previous) {
    if !active.IsActive || !IsDefined(weapon) || NotEquals(previous.GetEntityID(), weapon.GetEntityID()) {
      AnimationControllerComponent.ApplyFeature(previous, n"UTPPNueReload", off);
      AnimationControllerComponent.ApplyFeature(previous, n"UTPPNueReloadAim", off);
      AnimationControllerComponent.ApplyFeature(previous, n"UTPPNueReload2H", off);
      player.utppNueReloadGateWeapon = null;
    };
  };
  if active.IsActive {
    // Select both bases before enabling either graph. Weapon graphs are not
    // assumed to inherit private player features automatically.
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReloadAim", aim);
    AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReloadAim", aim);
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReload2H", grip);
    AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReload2H", grip);
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReload", active);
    AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReload", active);
    player.utppNueReloadGateWeapon = weapon;
  } else {
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReload", off);
    if IsDefined(weapon) {
      AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReload", off);
    };
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReloadAim", off);
    AnimationControllerComponent.ApplyFeature(player, n"UTPPNueReload2H", off);
    if IsDefined(weapon) {
      AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReloadAim", off);
      AnimationControllerComponent.ApplyFeature(weapon, n"UTPPNueReload2H", off);
    };
  };
  UTPPSetMMAuthoredReload(active.IsActive);
  return accepted;
}

// Snapshot after native OnEnter/OnUpdate; no writes to native reload data.
public func UTPPNueReloadNativeInputs(state: ref<ReloadEvents>) -> String {
  let speed: Float;
  if !IsDefined(state) || !IsDefined(state.m_animReloadData) || !IsDefined(state.m_animReloadSpeed) {
    return "unavailable";
  };
  if state.m_animReloadData.emptyReload {
    speed = state.m_animReloadSpeed.emptyReloadSpeed;
  } else {
    speed = state.m_animReloadSpeed.reloadSpeed;
  };
  return "empty=" + ToString(state.m_animReloadData.emptyReload)
    + ";loop=" + ToString(state.m_animReloadData.continueLoop)
    + ";speed=" + ToString(speed);
}

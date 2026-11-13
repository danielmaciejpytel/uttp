// Read-only route/calibration evidence. No hooks, pose writes or feature calls.
public func UTPPMountProbeTransform(t: WorldTransform) -> String {
  let p: Vector4 = WorldPosition.ToVector4(WorldTransform.GetWorldPosition(t));
  let q: Quaternion = WorldTransform.GetOrientation(t);
  return ToString(p.X) + "," + ToString(p.Y) + "," + ToString(p.Z) + "," + ToString(q.i) + "," + ToString(q.j) + "," + ToString(q.k) + "," + ToString(q.r);
}

public func UTPPNueMountRoute() -> String {
  let player: ref<PlayerPuppet> = GetPlayer(GetGameInstance());
  let weapon: ref<WeaponObject>;
  let slots: ref<SlotComponent>;
  let world: WorldTransform;
  let components: array<ref<IComponent>>;
  let animated: ref<AnimatedComponent>;
  let row: String;
  let rigPath: String;
  let i: Int32;
  let componentIndex: Int32;
  let found: Int32 = 0;
  if !IsDefined(player) {
    return "unavailable:player";
  };
  weapon = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
  if !IsDefined(weapon) || NotEquals(ItemID.GetTDBID(weapon.GetItemID()), t"Items.Preset_Nue_Default") {
    return "unavailable:drawn-nue-default";
  };
  row = "playerTemplate=" + ResRef.ToString(player.GetTemplatePath())
    + "|weaponTemplate=" + ResRef.ToString(weapon.GetTemplatePath())
    + "|playerWorld=" + UTPPMountProbeTransform(player.GetWorldTransform());
  components = player.GetComponents();
  componentIndex = 0;
  while componentIndex < ArraySize(components) && componentIndex < 128 {
    slots = components[componentIndex] as SlotComponent;
    if IsDefined(slots) {
    i = 0;
    while i < ArraySize(slots.slots) && found < 8 {
      if Equals(slots.slots[i].boneName, n"WeaponRight") {
        if slots.GetSlotTransform(slots.slots[i].slotName, world) {
          row += "|slot=" + NameToString(slots.slots[i].slotName)
            + ";bone=" + NameToString(slots.slots[i].boneName)
            + ";world=" + UTPPMountProbeTransform(world)
            + ";relative=" + ToString(slots.slots[i].relativePosition.X) + "," + ToString(slots.slots[i].relativePosition.Y) + "," + ToString(slots.slots[i].relativePosition.Z)
            + "," + ToString(slots.slots[i].relativeRotation.i) + "," + ToString(slots.slots[i].relativeRotation.j) + "," + ToString(slots.slots[i].relativeRotation.k) + "," + ToString(slots.slots[i].relativeRotation.r);
          found += 1;
        };
      };
      i += 1;
    };
    };
    componentIndex += 1;
  };
  row += "|weaponRightSlots=" + ToString(found);
  components = weapon.GetComponents();
  i = 0;
  while i < ArraySize(components) && i < 128 {
    animated = components[i] as AnimatedComponent;
    if IsDefined(animated) && (Equals(animated.name, n"Barrel") || Equals(animated.name, n"Magazine") || Equals(animated.name, n"MagazineReload")) {
      row += "|component=" + NameToString(animated.name)
        + ";world=" + UTPPMountProbeTransform(animated.worldTransform)
        + ";local=" + UTPPMountProbeTransform(animated.localTransform);
      if IsDefined(animated.parentTransform) {
        row += ";bind=" + NameToString(animated.parentTransform.bindName);
      };
    };
    i += 1;
  };
  components = player.GetComponents();
  i = 0;
  while i < ArraySize(components) && i < 128 {
    animated = components[i] as AnimatedComponent;
    if IsDefined(animated) {
      rigPath = ResRef.ToString(ResourceRef.GetPath(animated.rig));
      if StrContains(rigPath, "player_woman_skeleton.rig") {
        row += "|playerComponent=" + NameToString(animated.name) + ";rig=" + rigPath
          + ";graph=" + ResRef.ToString(ResourceRef.GetPath(animated.graph));
      };
    };
    i += 1;
  };
  return row;
}

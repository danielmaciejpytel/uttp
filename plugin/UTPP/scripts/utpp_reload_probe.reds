// Passive resource identity bridge. CET cannot pass component.rig directly to
// Codeware methods requiring script_ref<ResourceRef>. Keep that call here.
// kind: 0 rig, 1 graph, 2 configured gameplay set (not selected animation).
public func UTPPReloadResourceIdentity(component: ref<AnimatedComponent>, kind: Int32, index: Int32) -> String {
  if !IsDefined(component) {
    return "unavailable:null-component";
  };
  if kind == 0 {
    return "hash=" + ToString(ResourceRef.GetHash(component.rig)) + ";path=" + ResRef.ToString(ResourceRef.GetPath(component.rig));
  };
  if kind == 1 {
    return "hash=" + ToString(ResourceRef.GetHash(component.graph)) + ";path=" + ResRef.ToString(ResourceRef.GetPath(component.graph));
  };
  if kind == 2 {
    if index < 0 || index >= ArraySize(component.animations.gameplay) || index >= 16 {
      return "unavailable:set-index";
    };
    return "hash=" + ToString(ResourceAsyncRef.GetHash(component.animations.gameplay[index].animSet)) + ";path=" + ResRef.ToString(ResourceAsyncRef.GetPath(component.animations.gameplay[index].animSet));
  };
  return "unavailable:resource-kind";
}

// Read the already-owned native feature; never synthesize or ApplyFeature it.
// This describes inputs to the player graph, not its selected resource/clip.
public func UTPPReloadBranchMetadata(state: ref<ReloadEvents>) -> String {
  if !IsDefined(state) {
    return "unavailable:null-state";
  };
  if !IsDefined(state.m_animReloadData) {
    return "unavailable:reload-feature";
  };
  return "amountToReload=" + ToString(state.m_animReloadData.amountToReload)
    + ";empty=" + ToString(state.m_animReloadData.emptyReload)
    + ";continueLoop=" + ToString(state.m_animReloadData.continueLoop);
}

// Read-only world positions of the player's hand/weapon attachment slots (fallback when CET cannot call
// SlotComponent.GetSlotTransform with an out parameter). Output: "key;x,y,z|key;x,y,z|..." where key is the
// slot name or bone name equal to one of the wanted names. Never changes state.
private func UTPPReloadWantedSlot(n: CName) -> Bool {
  return Equals(n, n"WeaponRight") || Equals(n, n"WeaponLeft") || Equals(n, n"LeftHand") || Equals(n, n"RightHand") || Equals(n, n"Head") || Equals(n, n"TppHead");
}

public func UTPPReloadRefSlots(player: ref<GameObject>) -> String {
  let components: array<ref<IComponent>>;
  let slots: ref<SlotComponent>;
  let world: WorldTransform;
  let p: Vector4;
  let key: CName;
  let row: String = "";
  let found: Int32 = 0;
  let i: Int32;
  let c: Int32 = 0;
  if !IsDefined(player) {
    return "unavailable:null-player";
  };
  components = player.GetComponents();
  while c < ArraySize(components) && c < 128 {
    slots = components[c] as SlotComponent;
    if IsDefined(slots) {
      i = 0;
      while i < ArraySize(slots.slots) && found < 12 {
        key = n"";
        if UTPPReloadWantedSlot(slots.slots[i].slotName) {
          key = slots.slots[i].slotName;
        } else {
          if UTPPReloadWantedSlot(slots.slots[i].boneName) {
            key = slots.slots[i].boneName;
          };
        };
        if NotEquals(key, n"") && slots.GetSlotTransform(slots.slots[i].slotName, world) {
          p = WorldPosition.ToVector4(WorldTransform.GetWorldPosition(world));
          if found > 0 {
            row += "|";
          };
          row += NameToString(key) + ";" + ToString(p.X) + "," + ToString(p.Y) + "," + ToString(p.Z);
          found += 1;
        };
        i += 1;
      };
    };
    c += 1;
  };
  if found == 0 {
    return "unavailable:no-slots";
  };
  return row;
}

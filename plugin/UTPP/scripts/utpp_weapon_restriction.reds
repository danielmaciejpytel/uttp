// Passive diagnosis of native forced holstering: one line per state entry.
// Vanilla 2.31 upperBodyTransitions.script:326-333 defines this signature and
// queues UnequipAll. Preserve it; never change equipment, facts or restrictions.
// Predicate sources: defaultTransition.script:1225-1227,1334-1339.
// Requires utpp.reds (UTPPLogLine); no retained StateContext/interface handles.
@wrapMethod(ForceEmptyHandsEvents)
protected func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  if !IsDefined(stateContext) || !IsDefined(scriptInterface)
    || !IsDefined(scriptInterface.executionOwner) || !scriptInterface.executionOwner.IsPlayer() {
    return;
  }
  let bb: ref<IBlackboard> = scriptInterface.localBlackboard;
  if !IsDefined(bb) {
    return;
  }
  let psm: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
  // GetBoolParameter(..., true) is the exact permanent-parameter read used by
  // vanilla IsEmptyHandsForced, including its missing-parameter semantics.
  let forceHands: Bool = stateContext.GetBoolParameter(n"ForceEmptyHands", true);
  let forceZone: Bool = stateContext.GetBoolParameter(n"ForceEmptyHandsByZone", true);
  let forceFact: Int32 = scriptInterface.GetQuestsSystem().GetFact(n"ForceEmptyHands");
  let noCombat: Bool = StatusEffectSystem.ObjectHasStatusEffectWithTag(scriptInterface.executionOwner, n"NoCombat");
  let fastForward: Bool = StatusEffectSystem.ObjectHasStatusEffectWithTag(scriptInterface.executionOwner, n"FastForward");
  let vehicleScene: Bool = StatusEffectSystem.ObjectHasStatusEffectWithTag(scriptInterface.executionOwner, n"VehicleScene");
  UTPPLogLine("[utpp_weapon_restriction] ForceEmptyHands enter"
    + ";forceParam=" + ToString(forceHands)
    + ";zoneParam=" + ToString(forceZone)
    + ";forceFact=" + ToString(forceFact)
    + ";noCombat=" + ToString(noCombat)
    + ";fastForward=" + ToString(fastForward)
    + ";vehicleScene=" + ToString(vehicleScene)
    + ";zones=" + ToString(bb.GetInt(psm.Zones))
    + ";sceneAim=" + ToString(bb.GetBool(psm.SceneAimForced))
    + ";sceneSafe=" + ToString(bb.GetBool(psm.SceneSafeForced)));
}

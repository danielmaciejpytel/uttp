// Retail ShootEvents.OnEnter emits Shoot and LastShotTime for the individual shot.
// PSM.Weapon8 is also used by cycling/wait states, so it is not the shot clock.
// Preserve vanilla execution and never retain the borrowed state interface.
@wrapMethod(ShootEvents)
protected func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  if !IsDefined(scriptInterface) || !IsDefined(scriptInterface.executionOwner) || !scriptInterface.executionOwner.IsPlayer() {
    return;
  }
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(scriptInterface.executionOwner.GetGame());
  if IsDefined(glue) {
    glue.FeedRecoilShot(this.GetWeaponObject(scriptInterface));
  }
}

// UTPP crosshair in third person.
// Behaviour and API follow the vanilla scripts.
//
// Bug: vanilla hides the crosshair while aiming (PSM crosshair state Aim -> gameuiCrosshairBaseGameController.OnState_Aim ->
// m_rootWidget.SetVisible(false), cyberpunk\UI\weapons\crosshairs\crosshairBaseControllers.script:225-227, dispatched by
// UpdateCrosshairState :190-215), because in first person the ironsight/scope replaces it. In UTPP third person there is
// no ironsight on screen, so the player aims blind.
//
// Fix without @replaceMethod: @wrapMethod(gameuiCrosshairBaseGameController) UpdateCrosshairState calls the original, then,
// only when the third-person body is active (UTPPIsTppActive, utpp.reds), the crosshair state is Aim, the PSM UpperBody
// state is Aim and the drawn weapon is ranged, applies the hip-fire visual (OnState_HipFire) - the same entry vanilla uses
// for the camo (SetCamoActiveOnPlayer :398-408) and the TPP driver-combat case (UpdateTPPDriverCombatCrosshair :261-269).
// Controllers with their own OnState_HipFire (Basic, Smart Rifle, Tech, ...) show their normal hip-fire crosshair.
// Not touched: FPP (UTPPIsTppActive false -> the original result stays), vehicles in TPP (IsVehicleInTPP keeps vanilla's
// own handling), melee / no-weapon controllers (weapon not ranged).
//
// Limits: UpdateCrosshairState runs on crosshair-state changes and OnPreIntro only, so a CET TPP switch while the aim is
// held takes effect at the next state change (release / re-aim). Log: one line per change of the applied state,
// "[utpp_crosshair] tppAim=1 (hipfire)" / "tppAim=0" (utpp.reds UTPPLogLine).
// Requires utpp.reds (UTPPIsTppActive, UTPPLogLine).

@addField(gameuiCrosshairBaseGameController)
public let utppCrosshairTppAim: Bool;

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPWantsTppAimCrosshair() -> Bool {
  if !UTPPIsTppActive(this.GetGame()) {
    return false;
  }
  if !IsDefined(this.m_psmBlackboard) {
    return false;
  }
  let defs: ref<AllBlackboardDefinitions> = GetAllBlackboardDefs();
  if this.m_psmBlackboard.GetBool(defs.PlayerStateMachine.IsVehicleInTPP) {
    return false;
  }
  if NotEquals(this.GetCrosshairState(), gamePSMCrosshairStates.Aim) {
    return false;
  }
  if this.m_psmBlackboard.GetInt(defs.PlayerStateMachine.UpperBody) != EnumInt(gamePSMUpperBodyStates.Aim) {
    return false;
  }
  return WeaponObject.IsRanged(this.GetWeaponRecordID());
}

@wrapMethod(gameuiCrosshairBaseGameController)
protected final func UpdateCrosshairState() -> Void {
  wrappedMethod();
  if !this.IsActive() {
    return;
  }
  let want: Bool = this.UTPPWantsTppAimCrosshair();
  if want {
    this.OnState_HipFire();
  }
  if NotEquals(want, this.utppCrosshairTppAim) {
    this.utppCrosshairTppAim = want;
    // the UTPP ring (utpp_reticle.reds) keeps its resting radius in hip fire and kicks only while aiming
    this.UTPPRetAimChanged();
    if want {
      UTPPLogLine("[utpp_crosshair] tppAim=1 (hipfire)");
    } else {
      UTPPLogLine("[utpp_crosshair] tppAim=0");
    }
  }
}

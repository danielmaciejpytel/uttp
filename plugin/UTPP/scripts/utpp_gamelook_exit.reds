// UTPP vehicle exit look zeroing: tells the native game look hook (input/GameLook.cpp VehicleExitStart) that the
// player's PSM vehicle exit started, so the game's FPP look yaw is kept along V's heading through the whole exit. At the
// unmount the game turns V onto its look (RotateEntityToCamera reads the look offset ~0.13 s before the vehicle state ends);
// with the offset at 0 V keeps the exit yaw instead of swivelling onto the held camera yaw.
// Third person only: UTPP third person (UTPPIsTppActive) and the ride's vehicle camera not first person (read before the
// vanilla exit code runs: the vehicle camera may switch to first person during the exit).
// Requires utpp.reds (UTPPIsTppActive) and UTPP.dll with UTPPGameLookVehicleExit.

public native func UTPPGameLookVehicleExit(utppTpp: Bool, rideTpp: Int32) -> Bool

// 1 = the vehicle camera is (or stays persistently) third person, 0 = first person, -1 = no vehicle camera manager.
public func UTPPGameLookRideTpp(scriptInterface: ref<StateGameScriptInterface>) -> Int32 {
  let vehicle: wref<VehicleObject> = scriptInterface.owner as VehicleObject;
  if !IsDefined(vehicle) {
    VehicleComponent.GetVehicle(scriptInterface.executionOwner.GetGame(), scriptInterface.executionOwner, vehicle);
  }
  if !IsDefined(vehicle) {
    return -1;
  }
  let mgr: ref<VehicleCameraManager> = vehicle.GetCameraManager();
  if !IsDefined(mgr) {
    return -1;
  }
  if mgr.IsTPPActive() || NotEquals(mgr.GetPersistentPerspective(), vehicleCameraPerspective.FPP) {
    return 1;
  }
  return 0;
}

@wrapMethod(ExitingEventsBase)
protected func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  let player: ref<PlayerPuppet> = scriptInterface.executionOwner as PlayerPuppet;
  if IsDefined(player) {
    UTPPGameLookVehicleExit(UTPPIsTppActive(player.GetGame()), UTPPGameLookRideTpp(scriptInterface));
  }
  wrappedMethod(stateContext, scriptInterface);
}

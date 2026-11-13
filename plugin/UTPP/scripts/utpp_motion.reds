// Game-side glue for the motion motion matching driver (weapon class -> overlay table). Uses ONLY the fixed motion native interface
// below (documented on the DLL side in MotionDriver.hpp). Nothing here touches the base feed: the base natives, the
// LocomotionEventsTransition.OnUpdate feed (utpp.reds) and the psmBlock mask stay as they are; while motion is off
// (CET setting mm.motionDriver) no listener is registered and no motion native is called (the status getter only on
// request); switching motion off calls UTPPSetMotionEnabled(false) once and removes the listeners.
// Every vanilla API below is cited as file:line of the vanilla scripts.
//
// Inputs sent to UTPPSetMMOverlayInputs(pose, aim, release, injured, resolvedAimPose):
//   pose    = Overlay.hpp PoseOverlay value, only 0 Default, 1 Rifle, 2 Pistol1H, 3 Pistol2H (Injured and the
//             game-owned case travel as the separate Bools, so the C++ enum numbering of Injured / Game is not used);
//   aim     = Overlay.hpp AimState: 0 Relaxed, 1 Ready, 2 Aiming;
//   release = true while the upper body stays on game animations (melee incl. Mantis / Gorilla / Monowire, body carry,
//             the grenade throw itself) - motion hands control back to the game like base's release;
//   injured = health <= the game's own low-health threshold, or Bleeding.
// Cadence: change events only (PSM blackboard listeners, active weapon record, health / status effect events); no
// per-frame hook and no second wrap of a method utpp.reds wraps.
//
// Console (CET): Game.UTPPMotionGlueStatus()  or  Game.GetScriptableSystemsContainer():Get("UTPPMotionGlue"):Status()

// ---- fixed motion native interface (UTPP.dll) ----
// motion on / off (mm.motionDriver in CET). motion runs inside the hold path and never opens the hard gate.
public native func UTPPSetMotionEnabled(on: Bool) -> Void
// Rig body of V: 0 = woman (female V, woman_base), 1 = man (male V, man_base).
public native func UTPPSetMMRigBody(body: Int32) -> Void
// Overlay inputs (see the header).
// Protocol 2: DLL + this script must be installed together (atomic resolved Ready/Aim grip).
public native func UTPPSetMMOverlayInputs(pose: Int32, aim: Int32, release: Bool, injured: Bool, resolvedAimPose: Int32) -> Void
// Independent PSM.Weapon action (game 2.5: PSM.RightWeapon, plus LeftWeapon while dual wielding): 0 none, 2 Reload, 8 Shoot. Reload temporarily preserves the game's upper-body animation.
public native func UTPPSetMMWeaponAction(action: Int32) -> Void
// Exact equipped-weapon token and per-shot pulse; install together with matching DLL.
// recoilProfile: authored recoil profile id of the equipped weapon (plugin\UTPP\src\mm\RecoilProfiles.hpp, ids below), 0 = none.
public native func UTPPSetMMRecoilWeapon(weaponEpoch: Uint32, recoilProfile: Int32) -> Void
public native func UTPPFeedMMRecoilShot(weaponEpoch: Uint32, aiming: Bool) -> Void
// One line: enabled, body, db loaded, fallback reason, last pick, feed p95 us.
public native func UTPPGetMotionStatus() -> String
// the motion set-motionfix item 2: while aiming with a weapon overlay, hand the upper body back to the game + aim offset
// (true, default in the DLL) or keep the motion overlay + motion aim offset (false). CET setting mm.aimGameUpper.
public native func UTPPSetMotionAimGameUpper(on: Bool) -> Void
// Slide feed (default off; CET setting mm.slide): PSM LocomotionDetailed Slide (5, sprint -> crouch) reaches the the motion set
// chooser as MovementMode Sliding + Stance Crouch + MovementDirection (Forward / Other) + MovementMode_LastFrame (exit rows), motion
// owns the body for it (no foot plants, no body-yaw command). Takes effect ONLY with a bundle whose chooser.json has the "slide"
// section and whose database bakes PSD_Slide_FeetOut / PSD_Slide_KneesOut; with any other bundle the setting is a no-op. Always
// true (no validation). Status: slide= slideBundle= slideFrames=. Keep this declaration and the DLL native (Main.cpp) in sync.
public native func UTPPSetMotionSlide(on: Bool) -> Bool
// Shooter mode (CET setting bodyturn.shooterMode, utpp_bodyturn): on the ground and in the air the chooser strafes like motion set
// (camera-relative directional clips, no spin transition, a standing V keeps her visible direction while the camera orbits and turns in
// place with the motion set turn-in-place clips when she moves / aims / shoots, sprint only roughly forward). A process-lifetime DLL flag:
// idempotent, never resets matcher / root state, always true (no validation), takes effect on the next motion Step. Status: shooterMode=
// shooterModeLive= shooterFrames=. Keep this declaration and the DLL native (Main.cpp) in sync: a declared native without registration breaks
// the scripts.
public native func UTPPSetMotionShooterMode(on: Bool) -> Bool
// Backward-run safeguard kill switch (console only, default on; CET never calls it): while the CET native body turn is switched off for another
// reason (weapon turn off, kill switch, guard pause) and V runs backward, motion strafes instead of repeating the 180 deg spin transition.
// Always true. Keep this declaration and the DLL native (Main.cpp) in sync.
public native func UTPPSetMotionH2Guard(on: Bool) -> Bool
// CET hip-fire lease (utpp_bodyturn HipFireStep: a Shoot=8 with a drawn weapon snaps the capsule onto the camera and keeps the native body
// turn off for 0.75 s): tells the DLL that the native body turn is off for THAT reason, so the backward-run safeguard stays closed (the raw PSM
// Weapon state is pose-gated for weapons without a pose row). Sent by CET on every change of the lease (also with Shooter mode off),
// false when the executor is selected / disarmed and after a new game session. Always true, idempotent, resets nothing. Keep this
// declaration and the DLL native (Main.cpp) in sync.
public native func UTPPSetMotionHipLease(on: Bool) -> Bool
// True while Shooter mode is live in the DLL (setting on, motion enabled, no fallback, bundle loaded): one atomic load. CET (utpp_bodyturn)
// polls it once per second; false = Shooter mode is inert (Locomotion and the native body turn stay as without the mode). Keep this
// declaration and the DLL native (Main.cpp) in sync.
public native func UTPPMotionShooterModeLive() -> Bool
// True while the slide feed is live (mm.slide on, motion able to run, the active bundle declares slide): one atomic load, false for
// vanilla behaviour (motion off, setting off, plain bundle, MM disarmed). Used by the utpp_locomotion.reds GetCameraYaw wrap. Keep this
// declaration and the DLL native (Main.cpp) in sync.
public native func UTPPSlideOwned() -> Bool
// Magazine top-up gesture: the magazine of the right-hand weapon was refilled with no ReloadEvents state (game Shinobi slide / air dash
// bonus, no game animation). Called once per such refill from OnMagazineAmmo (weapon overlay, no reload). The DLL plays one short
// left-hand gesture to the magazine on the next motion frame only while motion owns the arms in TPP (no vehicle / FPP / reload / ADS hand-back),
// else the request is dropped. false = refused at once (also always while the CET option mm.shinobiBonusReload "Reload animation
// during Shinobi bonus" is off, the default; native UTPPSetMotionMagTap in utpp.reds). Keep this declaration and the DLL native (Main.cpp) in sync: a declared native
// without registration breaks the scripts.
public native func UTPPFeedMotionMagTap() -> Bool
// Orientation warp (default off; CET setting mm.orientWarp): while V runs a circle the pelvis and the torso are turned onto the
// path of motion instead of the outside of the turn (loop clips on the ground, no aim / weapon / air / crouch / slide / traversal;
// straight running, low speed and everything else stay bit-identical). Always true (no validation). Status: orientWarp= owW= owRoot=
// owTorso=. Keep this declaration and the DLL native (Main.cpp) in sync: a declared native without registration breaks the scripts.
public native func UTPPSetMotionOrientWarp(on: Bool) -> Bool
// 1.2 lateral-acceleration lean (default off; CET settings mm.lean, mm.leanHead, mm.leanScale): while V runs / walks on the
// ground the body leans into the turn (mesh space additive from the pack motion\utpp_lean_<body>.bin; no pack = nothing happens). Aiming,
// weapons, air, crouch, slide, traversal and straight running stay bit-identical. UTPPSetMotionLeanHead: the head layer replaces the
// head lead while on. UTPPSetMotionLeanParams(scale, headScale): amount multipliers 0 .. 2 (1 = full strength), false = NaN / out of range.
// Status: lean=<on>/<pack>/<alpha>/<lat>/<x>/<curve> leanHead= leanScale=. Keep these declarations and the DLL natives (Main.cpp) in sync.
public native func UTPPSetMotionLean(on: Bool) -> Bool
public native func UTPPSetMotionLeanHead(on: Bool) -> Bool
public native func UTPPSetMotionLeanParams(scale: Float, headScale: Float) -> Bool
// the motion set-trav (MotionTraversal): obstacle of a game climb (action 8) / vault (action 9) start, sent once from the
// ClimbEvents / VaultEvents OnEnter wraps below (only while motion is on). The DLL plans one motion set traversal montage from it
// (MotionDriver.hpp FeedTraversal); false = rejected input (invalid action / non-finite value).
public native func UTPPFeedTraversal(action: Int32, topPoint: Vector4, collisionNormal: Vector4, topExtent: Float,
  behindPoint: Vector4, obstacleEnd: Vector4, inputDirection: Vector4, hasHands: Bool, leftHand: Vector4, rightHand: Vector4,
  playerPos: Vector4, ownerForward: Vector4, speed: Float, airborne: Bool, climbSpeedModifier: Float,
  highThreshold: Float, midThreshold: Float, verticalDurationHigh: Float, verticalDurationMid: Float,
  verticalDurationLow: Float, horizontalDurationHigh: Float, horizontalDurationMid: Float, horizontalDurationLow: Float,
  forwardStep: Float) -> Bool
// Kind of the climbed / vaulted obstacle for the NEXT UTPPFeedTraversal call (0 static geometry, 1 vehicle, 2 other entity;
// descResult.climbedEntity). The DLL copies it into that traversal event and resets it to 0; a vault over a vehicle is handed
// to the game's vault animation (MotionTraversal.hpp Config::vehicleVaultToGame). Other values are stored as 0.
public native func UTPPFeedTraversalObstacleKind(kind: Int32) -> Void

// ---- constants (values of the 2.31 enums, cited) ----
// Overlay.hpp PoseOverlay (l. 329-337) / AimState (l. 346-351)
public func UTPPMotionPoseDefault() -> Int32 { return 0; }
public func UTPPMotionPoseRifle() -> Int32 { return 1; }
public func UTPPMotionPosePistol1H() -> Int32 { return 2; }
public func UTPPMotionPosePistol2H() -> Int32 { return 3; }
// table value: weapon stays on game animations (release = true); never sent as a pose
public func UTPPMotionPoseGame() -> Int32 { return -1; }
// table value: handgun row, resolved by the Pistol1H option (Pistol2H default)
public func UTPPMotionPoseHandgun() -> Int32 { return -2; }
public func UTPPMotionAimRelaxed() -> Int32 { return 0; }
public func UTPPMotionAimReady() -> Int32 { return 1; }
public func UTPPMotionAimAiming() -> Int32 { return 2; }

// Authored recoil profile of a weapon FAMILY tag (ids = index of RecoilProfiles.hpp kTable, checked against it by RecoilProfilesTest).
// The family is a tag of the weapon record (retail tweakdb.bin: "...,Rifle Assault,...,Ajax"). 0 = no profile: that weapon keeps the
// plain authored aim pose. Nue is not here: it stays bound to the exact preset id (Items.Preset_Nue_Default, profile 1) in
// ResolveRecoilProfile. Kyubi is a Militech Ajax variant: its animation set is the Ajax folder (w_rifle_assault__militech_kyubi__base1).
public func UTPPMotionRecoilProfileOfFamily(family: CName) -> Int32 {
  if Equals(family, n"Liberty") { return 2; }
  if Equals(family, n"Unity") { return 3; }
  if Equals(family, n"Yukimura") { return 4; }
  if Equals(family, n"Chao") { return 5; }
  if Equals(family, n"Lexington") { return 6; }
  if Equals(family, n"Slaughtomatic") { return 7; }
  // ids 8 Nova, 9 Overture, 10 Burya are retired (their clips are a hammer-cock the hand IK cannot follow, see RecoilProfiles.hpp): 0
  if Equals(family, n"Quasar") { return 11; }
  if Equals(family, n"Ajax") { return 12; }
  if Equals(family, n"Kyubi") { return 12; }
  if Equals(family, n"Copperhead") { return 13; }
  if Equals(family, n"Sidewinder") { return 14; }
  if Equals(family, n"Masamune") { return 15; }
  if Equals(family, n"Shingen") { return 16; }
  if Equals(family, n"Pulsar") { return 17; }
  if Equals(family, n"Saratoga") { return 18; }
  if Equals(family, n"Senkoh") { return 19; }
  // 2.5 additions; Borg4a and Hercules records carry no tag: UTPPMotionRecoilFamilyOfAnimset
  if Equals(family, n"Kenshin") { return 20; }
  if Equals(family, n"Omaha") { return 21; }
  if Equals(family, n"Ticon") { return 22; }
  if Equals(family, n"Defender") { return 23; }
  if Equals(family, n"Dian") { return 24; }
  if Equals(family, n"Warden") { return 25; }
  if Equals(family, n"Borg4a") { return 26; }
  if Equals(family, n"Hercules") { return 27; }
  return 0;
}

// Family of a weapon record that carries no usable family tag, from the animation wrapper of the record (Item_Record.FriendlyName,
// the variable that gates its player animset in player_wa_fpp(_ep1).ent). Resolved after the tags and before the class
// fallback; a Nue variant is never listed (the Nue stays bound to Items.Preset_Nue_Default). n"" = no family.
public func UTPPMotionRecoilFamilyOfAnimset(name: String) -> CName {
  if Equals(name, "w_handgun_arasaka_yukimura") { return n"Yukimura"; } // Kappa
  if Equals(name, "w_lmg_constitutional_defender") { return n"Defender"; } // MA70
  if Equals(name, "w_rifle_assault_nokota_copperhead") { return n"Copperhead"; } // Umbra
  if Equals(name, "w_rifle_assault_arasaka_masamune") { return n"Masamune"; } // Masamune presets without the Masamune tag
  if Equals(name, "w_submachinegun_militech_saratoga") { return n"Saratoga"; } // Guillotine (same profile as its fallback today)
  if Equals(name, "w_smg_midnight_borg") { return n"Borg4a"; } // Borg4a
  if Equals(name, "w_special__militech_hercules") { return n"Hercules"; } // Hercules
  return n"";
}

// Authoritative reload state for the motion glue. Game 2.5 ReloadEvents.OnEnter writes PSM.RightWeapon 2 (GetBlackboardID) and
// pushes RightReload; OnExit / OnForcedExit run OnExitCleanup (EndRightReload). The glue is told after the vanilla body, for
// the player's right-hand weapon logic only (IsRightWeaponLogic), so the vanilla behaviour and the left hand are untouched.
@wrapMethod(ReloadEvents)
protected final func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  UTPPNotifyReloadState(this, scriptInterface, true);
}

@wrapMethod(ReloadEvents)
protected func OnExit(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  UTPPNotifyReloadState(this, scriptInterface, false);
}

@wrapMethod(ReloadEvents)
protected func OnForcedExit(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  UTPPNotifyReloadState(this, scriptInterface, false);
}

@addMethod(ReloadEvents)
public final func UTPPIsRightHandReload() -> Bool {
  return this.IsRightWeaponLogic();
}

public func UTPPNotifyReloadState(state: ref<ReloadEvents>, scriptInterface: ref<StateGameScriptInterface>, active: Bool) -> Void {
  if !IsDefined(state) || !IsDefined(scriptInterface) || !IsDefined(scriptInterface.executionOwner) || !scriptInterface.executionOwner.IsPlayer() {
    return;
  };
  if !state.UTPPIsRightHandReload() {
    return;
  };
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(scriptInterface.executionOwner.GetGame());
  if IsDefined(glue) {
    glue.OnReloadState(active, scriptInterface.owner as WeaponObject);
  };
}

public class UTPPMotionReloadStateExpiry extends DelayCallback {
  public let glue: wref<UTPPMotionGlue>;
  public let generation: Int32;
  public func Call() -> Void {
    if IsDefined(this.glue) { this.glue.OnReloadStateExpiry(this.generation); }
  }
}

// Switch for the reload diagnostic lines (default off). CET / harness: Game.UTPPMotionSetReloadDiag(true).
public func UTPPMotionSetReloadDiag(on: Bool) -> Void {
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(GetGameInstance());
  if IsDefined(glue) {
    glue.SetReloadDiag(on);
  }
  UTPPLogLine("UTPPMotionSetReloadDiag(" + (on ? "1" : "0") + ")");
}

public class UTPPMotionReloadGripTail extends DelayCallback {
  public let glue: wref<UTPPMotionGlue>;
  public let generation: Int32;
  public func Call() -> Void {
    if IsDefined(this.glue) { this.glue.RefreshReloadGripTail(this.generation); }
  }
}

// the motion set-motionfix item 6: one-shot follow-up read after a weapon slot event (see UTPPMotionGlue.OnWeaponSlot).
public class UTPPMotionRecheckCallback extends DelayCallback {
  public let m_glue: wref<UTPPMotionGlue>;

  public func Call() -> Void {
    if IsDefined(this.m_glue) {
      this.m_glue.OnRecheck();
    }
  }
}

// dual-wield D0: one-shot flush of the [utpp_dual] rate limiter (UTPPMotionGlue.LogDual). It calls LogDual only, never Refresh.
public class UTPPMotionDualFlushCallback extends DelayCallback {
  public let m_glue: wref<UTPPMotionGlue>;

  public func Call() -> Void {
    if IsDefined(this.m_glue) {
      this.m_glue.OnDualFlush();
    }
  }
}

public class UTPPMotionGlue extends ScriptableSystem {
  private let m_player: wref<PlayerPuppet>;
  private let m_psm: wref<IBlackboard>;
  private let m_weaponBB: wref<IBlackboard>;
  private let m_recoilWeapon: wref<WeaponObject>;
  private let m_recoilEpoch: Uint32;
  private let m_recoilSupported: Bool;
  private let m_recoilProfile: Int32;
  private let m_cbUpper: ref<CallbackHandle>;
  private let m_cbWeapon: ref<CallbackHandle>;
  private let m_cbWeaponLeft: ref<CallbackHandle>;
  private let m_cbMelee: ref<CallbackHandle>;
  private let m_cbGadget: ref<CallbackHandle>;
  private let m_cbLeftCw: ref<CallbackHandle>;
  private let m_cbCombat: ref<CallbackHandle>;
  private let m_cbCarry: ref<CallbackHandle>;
  private let m_cbCarrying: ref<CallbackHandle>;
  private let m_cbWeaponRec: ref<CallbackHandle>;
  // Magazine of the right-hand weapon (WeaponObject shared blackboard, Weapon.MagazineAmmoCount): it detects a magazine refill that
  // happens without the vanilla ReloadEvents state (no PSM.RightWeapon 2, so no game reload animation): magazine top-up gesture
  // request (UTPPFeedMotionMagTap) + the optional diagnostic line.
  private let m_ammoBB: wref<IBlackboard>;
  private let m_ammoWeapon: wref<WeaponObject>;
  private let m_cbAmmo: ref<CallbackHandle>;
  private let m_lastMag: Int32;
  // Authoritative reload state: set by the ReloadEvents OnEnter / OnExit wraps (top of this file) of the player's
  // right-hand weapon state machine; independent of the PSM blackboard change event.
  private let m_reloadStateActive: Bool;
  private let m_reloadStateSince: Float;
  private let m_reloadStateEnters: Int32;
  private let m_reloadStateWeapon: wref<WeaponObject>;
  private let m_reloadStateGeneration: Int32;
  private let m_reloadDiag: Bool;
  // One diagnostic line per published reload episode (action 2 interval).
  private let m_epOpen: Bool;
  private let m_epStart: Float;
  private let m_epEnters: Int32;
  private let m_epLoco: Int32;
  private let m_epDetail: Int32;
  private let m_epUpper: Int32;
  private let m_epRight: Int32;
  private let m_epMag: Int32;
  private let m_epRefill: Bool;
  private let m_epRecord: TweakDBID;

  // weapon class -> overlay table (data; filled by BuildWeaponTable)
  private let m_tableTypes: array<gamedataItemType>;
  private let m_tablePoses: array<Int32>;
  private let m_pistolOneHand: Bool;
  private let m_pistolManual: Bool; // false = automatic family + Ready/Aim policy
  private let m_pistolSource: String;
  // Reload owns its ENTRY grip, independently of the native ADS/Ready state.
  private let m_reloadGrip: Int32;
  private let m_reloadEntity: EntityID;
  private let m_reloadRecord: TweakDBID;
  private let m_reloadTailUntil: Float;
  private let m_reloadGeneration: Int32;
  private let m_previousPistol: Bool;
  private let m_previousEntity: EntityID;
  private let m_previousRecord: TweakDBID;
  private let m_pistolOverrideIDs: array<TweakDBID>;
  private let m_pistolOverrideReady: array<Int32>;
  private let m_pistolOverrideAim: array<Int32>;

  private let m_enabled: Bool;
  private let m_body: Int32;
  private let m_healthPerc: Float;
  private let m_bleeding: Bool;
  // last values sent (-100 = never)
  private let m_sentBody: Int32;
  private let m_sentPose: Int32;
  private let m_sentResolvedAimPose: Int32;
  private let m_sentAim: Int32;
  private let m_sentAction: Int32;
  private let m_sentRelease: Bool;
  private let m_sentInjured: Bool;
  private let m_recheckPending: Bool;
  // diagnostics
  private let m_lastWhy: String;
  private let m_updates: Int32;
  private let m_pushes: Int32;
  // dual-wield D0 log ([utpp_dual], LogDual): state of the rate limiter; read-only, never feeds back into a decision
  private let m_dualOn: Bool;
  private let m_dualSig: String;
  private let m_dualVol: String;
  private let m_dualFlushPending: Bool;
  private let m_dualLogAt: Float;
  private let m_dualPrevRight: Int32;
  private let m_dualPrevLeft: Int32;
  private let m_dualShotsRight: Int32;
  private let m_dualShotsLeft: Int32;

  public func OnAttach() -> Void {
    this.m_body = -1;
    this.m_healthPerc = 100.0;
    this.m_sentBody = -100;
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.m_sentAim = -100;
    this.m_sentAction = -100;
    this.m_lastMag = -1;
    this.m_lastWhy = "init";
    this.BuildWeaponTable();
    UTPPLogEnumContract();
  }

  public func OnDetach() -> Void {
    this.ResetRecoilWeapon();
    this.ResetReloadGrip();
    this.UnregisterAll();
    if this.m_enabled {
      UTPPSetMMWeaponAction(0);
    }
  }

  // Engine callback on player attach (same entry point as craftingSystem.script:34-36, which reads the player's gender
  // here). Also runs after a load / new game. Listeners exist only while motion is enabled (off = no motion work at all).
  private final func OnPlayerAttach(request: ref<PlayerAttachRequest>) -> Void {
    this.ResetRecoilWeapon();
    this.ResetReloadGrip();
    this.UnregisterAll();
    if this.m_enabled {
      UTPPSetMMWeaponAction(0);
    }
    this.m_player = GetPlayer(this.GetGameInstance());
    this.m_recheckPending = false; // a callback of the previous session is gone
    this.m_dualFlushPending = false;
    this.m_dualOn = false; // sim time restarts with the session: a stale log time would mute the rate limiter
    this.m_dualLogAt = -10.0;
    this.m_sentBody = -100;
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.m_sentAction = -100;
    if this.m_enabled {
      this.Start("attach");
    }
  }

  // Registers the listeners on the current player, reads health / bleeding / body and pushes everything once.
  private final func Start(why: String) -> Void {
    let player: ref<PlayerPuppet> = this.m_player;
    if !IsDefined(player) {
      player = GetPlayer(this.GetGameInstance());
      this.m_player = player;
    }
    if !IsDefined(player) {
      return;
    }
    if !IsDefined(this.m_psm) {
      this.RegisterAll(player);
    }
    this.ReadHealth(player);
    this.m_bleeding = StatusEffectSystem.ObjectHasStatusEffectOfType(player, gamedataStatusEffectType.Bleeding);
    this.Refresh(why);
  }

  // ---- weapon class -> overlay (gamedataItemType values orphans.script:1069ff) ----
  // Rows not listed: Default overlay (no aim pose) [P] - e.g. Wea_GrenadeLauncher (orphans.script:1149) and the vehicle
  // weapons have no row in the table. Throwing knives / grenades / Projectile Launcher are handled by their PSM aim states
  // in ResolveInputs (they are not a right-hand weapon class of their own).
  private final func AddRow(t: gamedataItemType, pose: Int32) -> Void {
    ArrayPush(this.m_tableTypes, t);
    ArrayPush(this.m_tablePoses, pose);
  }

  private final func BuildWeaponTable() -> Void {
    ArrayClear(this.m_tableTypes);
    ArrayClear(this.m_tablePoses);
    // handguns, revolvers -> Pistol2H (Pistol1H as option); orphans.script:1151, :1161
    this.AddRow(gamedataItemType.Wea_Handgun, UTPPMotionPoseHandgun());
    this.AddRow(gamedataItemType.Wea_Revolver, UTPPMotionPoseHandgun());
    // long guns -> Rifle; orphans.script:1145, :1167, :1162, :1164, :1165, :1160, :1166, :1155, :1152
    this.AddRow(gamedataItemType.Wea_AssaultRifle, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_SubmachineGun, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_Rifle, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_Shotgun, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_ShotgunDual, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_PrecisionRifle, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_SniperRifle, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_LightMachineGun, UTPPMotionPoseRifle());
    this.AddRow(gamedataItemType.Wea_HeavyMachineGun, UTPPMotionPoseRifle());
    // melee -> game animations; orphans.script:1146-1148, :1150, :1153, :1154, :1156-1159, :1163, :1168, :1169
    this.AddRow(gamedataItemType.Wea_Axe, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Chainsword, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Fists, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Hammer, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Katana, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Knife, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_LongBlade, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Machete, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Melee, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_OneHandedClub, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_ShortBlade, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_Sword, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Wea_TwoHandedClub, UTPPMotionPoseGame());
    // arm cyberware weapons -> game animations; orphans.script:1086 Mantis, :1088 Gorilla (StrongArms), :1087 Monowire
    this.AddRow(gamedataItemType.Cyb_MantisBlades, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Cyb_StrongArms, UTPPMotionPoseGame());
    this.AddRow(gamedataItemType.Cyb_NanoWires, UTPPMotionPoseGame());
  }

  private final func TablePose(t: gamedataItemType) -> Int32 {
    let i: Int32 = 0;
    let n: Int32 = ArraySize(this.m_tableTypes);
    while i < n {
      if Equals(this.m_tableTypes[i], t) {
        let pose: Int32 = this.m_tablePoses[i];
        if pose == UTPPMotionPoseHandgun() {
          return UTPPMotionPoseHandgun(); // resolved after the real weapon and Aim state are known
        }
        return pose;
      }
      i += 1;
    }
    return UTPPMotionPoseDefault();
  }

  // ---- listeners (change events) ----
  // PSM blackboard of the player: PlayerPuppet.GetPlayerStateMachineBlackboard (player.script:1561); fields
  // blackboardDefinitions.script:10 UpperBody, :14 Weapon, :36 CombatGadget, :88 LeftHandCyberware, :92 MeleeWeapon,
  // :94 Carrying, :104 Combat, :138 BodyCarrying. RegisterListenerInt / Bool (orphans.script:17427 / :17415), callback
  // form as arcadeMachine.script:158 / :164. Weapon swaps: UI_ActiveWeaponData.RightWeaponRecordID (blackboardDefinitions
  // .script:1597; same listener as crosshairContainerController.script:45).
  // Game 2.5 (dual wield): PSM.Weapon became RightWeapon + LeftWeapon; both are listened to (LeftWeapon is expected to stay 0
  // with one weapon; a refresh it triggers only re-reads and re-sends the same values, and its value is read only while dual
  // wielding, see IsDualWielding). WeaponRecordID became Right/LeftWeaponRecordID; only the right record is
  // listened to (the recoil / reload-grip state below belongs to the WeaponRight item).
  private final func RegisterAll(player: ref<PlayerPuppet>) -> Void {
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let psm: ref<IBlackboard> = player.GetPlayerStateMachineBlackboard();
    this.m_psm = psm;
    if IsDefined(psm) {
      this.m_cbUpper = psm.RegisterListenerInt(psmDef.UpperBody, this, n"OnPsmInt");
      this.m_cbWeapon = psm.RegisterListenerInt(psmDef.RightWeapon, this, n"OnPsmInt");
      this.m_cbWeaponLeft = psm.RegisterListenerInt(psmDef.LeftWeapon, this, n"OnPsmInt");
      this.m_cbMelee = psm.RegisterListenerInt(psmDef.MeleeWeapon, this, n"OnPsmInt");
      this.m_cbGadget = psm.RegisterListenerInt(psmDef.CombatGadget, this, n"OnPsmInt");
      this.m_cbLeftCw = psm.RegisterListenerInt(psmDef.LeftHandCyberware, this, n"OnPsmInt");
      this.m_cbCombat = psm.RegisterListenerInt(psmDef.Combat, this, n"OnPsmInt");
      this.m_cbCarry = psm.RegisterListenerInt(psmDef.BodyCarrying, this, n"OnPsmInt");
      this.m_cbCarrying = psm.RegisterListenerBool(psmDef.Carrying, this, n"OnPsmBool");
    }
    let wbb: ref<IBlackboard> = GameInstance.GetBlackboardSystem(player.GetGame()).Get(GetAllBlackboardDefs().UI_ActiveWeaponData);
    this.m_weaponBB = wbb;
    if IsDefined(wbb) {
      this.m_cbWeaponRec = wbb.RegisterListenerVariant(GetAllBlackboardDefs().UI_ActiveWeaponData.RightWeaponRecordID, this, n"OnWeaponRecord");
    }
  }

  private final func UnregisterAll() -> Void {
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let psm: ref<IBlackboard> = this.m_psm;
    if IsDefined(psm) {
      if IsDefined(this.m_cbUpper) { psm.UnregisterListenerInt(psmDef.UpperBody, this.m_cbUpper); }
      if IsDefined(this.m_cbWeapon) { psm.UnregisterListenerInt(psmDef.RightWeapon, this.m_cbWeapon); }
      if IsDefined(this.m_cbWeaponLeft) { psm.UnregisterListenerInt(psmDef.LeftWeapon, this.m_cbWeaponLeft); }
      if IsDefined(this.m_cbMelee) { psm.UnregisterListenerInt(psmDef.MeleeWeapon, this.m_cbMelee); }
      if IsDefined(this.m_cbGadget) { psm.UnregisterListenerInt(psmDef.CombatGadget, this.m_cbGadget); }
      if IsDefined(this.m_cbLeftCw) { psm.UnregisterListenerInt(psmDef.LeftHandCyberware, this.m_cbLeftCw); }
      if IsDefined(this.m_cbCombat) { psm.UnregisterListenerInt(psmDef.Combat, this.m_cbCombat); }
      if IsDefined(this.m_cbCarry) { psm.UnregisterListenerInt(psmDef.BodyCarrying, this.m_cbCarry); }
      if IsDefined(this.m_cbCarrying) { psm.UnregisterListenerBool(psmDef.Carrying, this.m_cbCarrying); }
    }
    let wbb: ref<IBlackboard> = this.m_weaponBB;
    if IsDefined(wbb) && IsDefined(this.m_cbWeaponRec) {
      wbb.UnregisterListenerVariant(GetAllBlackboardDefs().UI_ActiveWeaponData.RightWeaponRecordID, this.m_cbWeaponRec);
    }
    this.m_cbUpper = null;
    this.m_cbWeapon = null;
    this.m_cbWeaponLeft = null;
    this.m_cbMelee = null;
    this.m_cbGadget = null;
    this.m_cbLeftCw = null;
    this.m_cbCombat = null;
    this.m_cbCarry = null;
    this.m_cbCarrying = null;
    this.m_cbWeaponRec = null;
    this.UnregisterAmmo();
    this.CloseReloadEpisode("detach");
    this.ClearReloadState();
    this.m_psm = null;
    this.m_weaponBB = null;
  }

  protected cb func OnPsmInt(value: Int32) -> Bool {
    this.Refresh("psm");
    return true;
  }

  protected cb func OnPsmBool(value: Bool) -> Bool {
    this.Refresh("carry");
    return true;
  }

  protected cb func OnWeaponRecord(value: Variant) -> Bool {
    this.ResetRecoilWeapon();
    this.ResetReloadGrip();
    // Two pistols may share the same overlay and PSM action. Do not reuse the
    // previous weapon's pre-Shoot reference across a record replacement.
    this.CloseReloadEpisode("weapon");
    this.ClearReloadState();
    if this.m_enabled {
      UTPPSetMMWeaponAction(0);
      this.m_sentAction = 0;
    }
    this.Refresh("weapon");
    return true;
  }

  // ---- body (UTPPSetMMRigBody) ----
  // gamePuppet.GetResolvedGenderName (orphans.script:16554 class gamePuppet, :16578): n"Male" / n"Female" - the
  // game picks the Ma / Wa head item by this name (inGameCharacterCustomizationGameController.script:7-13) and the
  // item appearance (equipmentSystem.script:2528-2529). Unknown name: nothing is sent (the DLL keeps its default).
  // Re-read on attach, on enable and on every refresh (one native CName read; the push only happens on a change),
  // which covers a body change after the in-game character customization without hooking the menu.
  public final func ReadBody() -> Int32 {
    let player: ref<PlayerPuppet> = this.m_player;
    if !IsDefined(player) {
      return -1;
    }
    let g: CName = player.GetResolvedGenderName();
    if Equals(g, n"Female") {
      return 0;
    }
    if Equals(g, n"Male") {
      return 1;
    }
    return -1;
  }

  private final func RefreshBody(why: String) -> Void {
    this.m_body = this.ReadBody();
    if this.m_enabled && this.m_body >= 0 && this.m_body != this.m_sentBody {
      UTPPSetMMRigBody(this.m_body);
      this.m_sentBody = this.m_body;
      this.m_lastWhy = "body:" + why;
    }
  }

  // ---- injured ----
  // Health % (StatPoolsSystem.GetStatPoolValue with perc, orphans.script:20547) compared against the game's own
  // low-health threshold PlayerPuppet.GetLowHealthThreshold (player.script:1519-1520, TweakDB
  // player.hitVFX.lowHealthThreshold) - the same test that switches the InLowHealth game tone (player.script:3722-3728),
  // so no threshold of ours. Bleeding: gamedataStatusEffectType.Bleeding (orphans.script:4339).
  private final func ReadHealth(player: ref<PlayerPuppet>) -> Void {
    let pools: ref<StatPoolsSystem> = GameInstance.GetStatPoolsSystem(player.GetGame());
    if IsDefined(pools) {
      this.m_healthPerc = pools.GetStatPoolValue(Cast<StatsObjectID>(player.GetEntityID()), gamedataStatPoolType.Health, true);
    }
  }

  public final func OnHealthPerc(value: Float) -> Void {
    this.m_healthPerc = value;
    this.Refresh("health");
  }

  public final func OnStatusEffectsChanged() -> Void {
    let player: ref<PlayerPuppet> = this.m_player;
    if !this.m_enabled || !IsDefined(player) {
      return;
    }
    let bleeding: Bool = StatusEffectSystem.ObjectHasStatusEffectOfType(player, gamedataStatusEffectType.Bleeding);
    if NotEquals(bleeding, this.m_bleeding) {
      this.m_bleeding = bleeding;
      this.Refresh("bleeding");
    }
  }

  public final func IsInjured() -> Bool {
    return this.m_bleeding || this.m_healthPerc <= PlayerPuppet.GetLowHealthThreshold();
  }

  // ---- overlay inputs ----
  // Right hand: AttachmentSlots.WeaponRight (read as combatGadgetTransitions.script:78), class by
  // WeaponObject.GetWeaponType (weapon.script:487), IsThrowable (weapon.script:279). Precedence (first match wins):
  //  1 body carry: PSM.BodyCarrying != 0 (gamePSMBodyCarrying orphans.script:6731) or PSM.Carrying (as utpp.reds l. 293-296)
  //    -> release (game animations);
  //  2 grenade: PSM.CombatGadget (gamePSMCombatGadget orphans.script:6781-6788) Charging 3 = aiming the throw
  //    (CombatGadgetChargeEvents, combatGadgetTransitions.script:450) -> Pistol1H Aiming; Throwing 4 /
  //    QuickThrow 6 (combatGadgetTransitions.script:371 / :325) -> release [P: the throw itself is a game animation];
  //  3 Projectile Launcher: PSM.LeftHandCyberware (gamePSMLeftHandCyberware orphans.script:6814ff) Charge 5 / Loop 6 /
  //    QuickAction 8 / ChargeAction 9 (leftHandCyberwareTransitions.script:595, :673, :769, :807) -> Pistol1H Aiming
  //    (the launcher sits in WeaponLeft, leftHandCyberwareTransitions.script:9-10) [P: the fire states included];
  //  4 right-hand weapon by table; a throwable melee weapon (throwing knife) while PSM.MeleeWeapon Targeting 9
  //    (MeleeTargetingEvents meleeTransitions.script:3025; orphans.script:6830ff) or UpperBody Aim (the game shows the
  //    throw hint in that state, inputContextTransitions.script:564) -> Pistol1H Aiming, otherwise release;
  //  5 no weapon in hand, or UpperBody TemporaryUnequip 4 / ForceEmptyHands 5 (orphans.script:6663-6671) -> Default.
  // Aim state of a gun overlay [P - mapping choice]: Aiming = PSM.UpperBody Aim 6 (orphans.script:6670); Ready =
  // PSM.Combat InCombat 1 (gamePSMCombat orphans.script:6891ff) or PSM.Weapon (gamePSMRangedWeaponStates
  // orphans.script:6918-6928, stored in PSM.Weapon per rangedWeaponPSMPrereq.script:20) Charging 1 / Reload 2 /
  // Overheat 7 / Shoot 8; Relaxed otherwise (Default 0, NoAmmo 4, Ready 5 out of combat, Safe 6). The DLL's
  // OverlayMachine (Overlay.hpp) still applies its own transition times on top of this.
  public final func ResolveInputs(out pose: Int32, out aim: Int32, out release: Bool, out resolvedAimPose: Int32) -> Void {
    resolvedAimPose = UTPPMotionPoseDefault();
    pose = UTPPMotionPoseDefault();
    aim = UTPPMotionAimRelaxed();
    release = false;
    let player: ref<PlayerPuppet> = this.m_player;
    let psm: ref<IBlackboard> = this.m_psm;
    if !IsDefined(player) || !IsDefined(psm) {
      return;
    }
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    // 1 body carry
    if psm.GetInt(psmDef.BodyCarrying) != 0 || psm.GetBool(psmDef.Carrying) {
      release = true;
      return;
    }
    let upper: Int32 = psm.GetInt(psmDef.UpperBody);
    // 2 grenade
    let gadget: Int32 = psm.GetInt(psmDef.CombatGadget);
    if gadget == 3 {
      pose = UTPPMotionPosePistol1H();
      aim = UTPPMotionAimAiming();
      return;
    }
    if gadget == 4 || gadget == 6 {
      release = true;
      return;
    }
    // 3 Projectile Launcher
    let leftCw: Int32 = psm.GetInt(psmDef.LeftHandCyberware);
    if leftCw == 5 || leftCw == 6 || leftCw == 8 || leftCw == 9 {
      pose = UTPPMotionPosePistol1H();
      aim = UTPPMotionAimAiming();
      return;
    }
    // 5 empty hands forced by the upper-body PSM
    if upper == 4 || upper == 5 {
      return;
    }
    // 4 right-hand weapon
    let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(player.GetGame());
    let weapon: ref<WeaponObject> = ts.GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
    if !IsDefined(weapon) {
      return;
    }
    let rowPose: Int32 = this.TablePose(WeaponObject.GetWeaponType(weapon.GetItemID()));
    if rowPose == UTPPMotionPoseGame() {
      if weapon.IsThrowable() && (psm.GetInt(psmDef.MeleeWeapon) == 9 || upper == 6) {
        pose = UTPPMotionPosePistol1H();
        aim = UTPPMotionAimAiming();
        return;
      }
      release = true;
      return;
    }
    resolvedAimPose = rowPose;
    if rowPose == UTPPMotionPoseHandgun() {
      rowPose = this.ResolvePistolPose(weapon, upper == 6);
      let selectedSource: String = this.m_pistolSource;
      resolvedAimPose = this.ResolvePistolPose(weapon, true);
      this.m_pistolSource = selectedSource;
    }
    pose = rowPose;
    if pose == UTPPMotionPoseDefault() {
      return;
    }
    if upper == 6 {
      aim = UTPPMotionAimAiming();
      return;
    }
    // Game 2.5 dual wield: UpperBody DualWield 7 is the two-weapons-drawn state (DualWieldEvents), not ADS -> Ready, never
    // Aiming. The left hand's ranged state counts like the right one only while dual wielding (single weapon unchanged).
    let dual: Bool = this.IsDualWielding(psm, upper);
    let ranged: Int32 = psm.GetInt(psmDef.RightWeapon);
    let rangedLeft: Int32 = dual ? psm.GetInt(psmDef.LeftWeapon) : 0;
    if psm.GetInt(psmDef.Combat) == 1 || ranged == 1 || ranged == 2 || ranged == 7 || ranged == 8
        || rangedLeft == 1 || rangedLeft == 2 || rangedLeft == 7 || rangedLeft == 8 || upper == 7 {
      aim = UTPPMotionAimReady();
    }
  }

  // Dual wield per the vanilla PSM: UpperBody DualWield (7) or PSM.IsDualWielding (EquipCycleInitEvents, left-hand ranged item).
  // Both are false with a single weapon, so every left-hand read below is skipped there.
  private final func IsDualWielding(psm: ref<IBlackboard>, upper: Int32) -> Bool {
    return upper == 7 || psm.GetBool(GetAllBlackboardDefs().PlayerStateMachine.IsDualWielding);
  }

  // Weapon action of the ranged PSM: 0 none, 2 Reload, 8 Shoot. While dual wielding either hand counts (Shoot first, then Reload).
  private final func RangedAction(psm: ref<IBlackboard>) -> Int32 {
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let ranged: Int32 = psm.GetInt(psmDef.RightWeapon);
    if ranged == 8 {
      return 8;
    }
    if this.IsDualWielding(psm, psm.GetInt(psmDef.UpperBody)) {
      let left: Int32 = psm.GetInt(psmDef.LeftWeapon);
      if left == 8 {
        return 8;
      }
      if ranged == 2 || left == 2 {
        return 2;
      }
      return 0;
    }
    if ranged == 2 || this.ReloadStateHeld() {
      return 2;
    }
    return 0;
  }

  // The vanilla right-hand ReloadEvents state is active (OnEnter seen, OnExit / OnForcedExit not yet). It complements PSM.RightWeapon
  // 2 (ReloadEvents.OnEnter writes both in the same call), so a change event that is missed or coalesced cannot drop a reload.
  // Fail-safes for an exit that never arrives: the state is dropped as soon as the reloading weapon is no longer the drawn
  // right-hand weapon (holster / unequip / swap, or UpperBody TemporaryUnequip 4 / ForceEmptyHands 5), and after 8 s (the longest
  // vanilla empty reload is shorter; while RightWeapon still reads 2 the blackboard path keeps the action anyway). The 8 s expiry
  // has its own delayed Refresh, so action 2 does not outlive it until an unrelated PSM event.
  private final func ReloadStateHeld() -> Bool {
    if !this.m_reloadStateActive {
      return false;
    }
    let drop: Bool = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance())) - this.m_reloadStateSince >= 8.00;
    let player: ref<PlayerPuppet> = this.m_player;
    let held: wref<WeaponObject> = this.m_reloadStateWeapon;
    if !drop {
      let weapon: ref<WeaponObject>;
      if IsDefined(player) {
        weapon = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
      }
      drop = !IsDefined(weapon) || !IsDefined(held) || NotEquals(held.GetEntityID(), weapon.GetEntityID());
    }
    if !drop && IsDefined(this.m_psm) {
      let upper: Int32 = this.m_psm.GetInt(GetAllBlackboardDefs().PlayerStateMachine.UpperBody);
      drop = upper == 4 || upper == 5;
    }
    if drop {
      this.m_reloadStateActive = false;
      this.m_reloadStateWeapon = null;
      this.m_reloadStateGeneration += 1;
      return false;
    }
    return true;
  }

  // Called by the ReloadEvents wraps (top of this file), after the vanilla body, for the player's right-hand weapon only.
  public final func OnReloadState(active: Bool, weapon: ref<WeaponObject>) -> Void {
    if !this.m_enabled {
      return;
    }
    this.m_reloadStateGeneration += 1;
    if active {
      this.m_reloadStateActive = true;
      this.m_reloadStateSince = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
      this.m_reloadStateWeapon = weapon;
      if !IsDefined(weapon) && IsDefined(this.m_player) {
        this.m_reloadStateWeapon = GameInstance.GetTransactionSystem(this.m_player.GetGame()).GetItemInSlot(this.m_player, t"AttachmentSlots.WeaponRight") as WeaponObject;
      }
      this.m_reloadStateEnters += 1;
      let expiry: ref<UTPPMotionReloadStateExpiry> = new UTPPMotionReloadStateExpiry();
      expiry.glue = this;
      expiry.generation = this.m_reloadStateGeneration;
      GameInstance.GetDelaySystem(this.GetGameInstance()).DelayCallback(expiry, 8.05, false);
    } else {
      this.m_reloadStateActive = false;
      this.m_reloadStateWeapon = null;
    }
    this.Refresh(active ? "reload-enter" : "reload-exit");
  }

  private final func ClearReloadState() -> Void {
    this.m_reloadStateActive = false;
    this.m_reloadStateWeapon = null;
    this.m_reloadStateGeneration += 1;
  }

  // Delayed check of the 8 s fail-safe; a newer enter / exit (other generation) makes it a no-op.
  public final func OnReloadStateExpiry(generation: Int32) -> Void {
    if this.m_enabled && this.m_reloadStateActive && generation == this.m_reloadStateGeneration {
      this.Refresh("reload-expire");
    }
  }

  // Reload diagnostics (episode / refill-no-state lines) are off by default. UTPPSetMMTrace has no script-visible state, so
  // the switch is the script global UTPPMotionSetReloadDiag (CET console / harness: Game.UTPPMotionSetReloadDiag(true)).
  public final func SetReloadDiag(on: Bool) -> Void {
    this.m_reloadDiag = on;
  }

  // ---- magazine listener (top-up gesture + diagnostic) ----
  private final func UnregisterAmmo() -> Void {
    let bb: ref<IBlackboard> = this.m_ammoBB;
    if IsDefined(bb) && IsDefined(this.m_cbAmmo) {
      bb.UnregisterListenerUint(GetAllBlackboardDefs().Weapon.MagazineAmmoCount, this.m_cbAmmo);
    }
    this.m_cbAmmo = null;
    this.m_ammoBB = null;
    this.m_ammoWeapon = null;
    this.m_lastMag = -1;
  }

  // Follows the object in AttachmentSlots.WeaponRight; re-registers only when the object changes.
  private final func RefreshAmmoListener() -> Void {
    let player: ref<PlayerPuppet> = this.m_player;
    if !IsDefined(player) {
      this.UnregisterAmmo();
      return;
    }
    let weapon: ref<WeaponObject> = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
    let previous: wref<WeaponObject> = this.m_ammoWeapon;
    if IsDefined(weapon) && IsDefined(previous) && Equals(previous.GetEntityID(), weapon.GetEntityID()) && IsDefined(this.m_cbAmmo) {
      return;
    }
    this.UnregisterAmmo();
    if !IsDefined(weapon) {
      return;
    }
    let bb: ref<IBlackboard> = weapon.GetSharedData();
    if !IsDefined(bb) {
      return;
    }
    this.m_ammoBB = bb;
    this.m_ammoWeapon = weapon;
    this.m_lastMag = Cast<Int32>(bb.GetUint(GetAllBlackboardDefs().Weapon.MagazineAmmoCount));
    this.m_cbAmmo = bb.RegisterListenerUint(GetAllBlackboardDefs().Weapon.MagazineAmmoCount, this, n"OnMagazineAmmo");
  }

  protected cb func OnMagazineAmmo(value: Uint32) -> Bool {
    let mag: Int32 = Cast<Int32>(value);
    let previous: Int32 = this.m_lastMag;
    this.m_lastMag = mag;
    if !this.m_enabled || previous < 0 || mag <= previous {
      return true;
    }
    if this.m_epOpen {
      this.m_epRefill = true;
      return true;
    }
    let psm: ref<IBlackboard> = this.m_psm;
    let right: Int32 = IsDefined(psm) ? psm.GetInt(GetAllBlackboardDefs().PlayerStateMachine.RightWeapon) : -1;
    // Self-heal: the game is in Reload but the action was not published (missed change event).
    if right == 2 && this.m_sentAction != 2 {
      this.Refresh("ammo");
      if this.m_epOpen {
        this.m_epRefill = true;
        return true;
      }
    }
    // A refill outside any reload episode: no ReloadEvents state, so the game pushes no reload animation event either.
    // Always active (a counter compare per magazine change): a weapon overlay (Rifle / Pistol1H / Pistol2H) published without release
    // and no reload -> one magazine top-up gesture request to the DLL (it plays only in TPP while motion owns the arms, and only with the
    // CET option mm.shinobiBonusReload on - default off: the DLL then refuses the request at once).
    if !this.m_reloadStateActive && this.m_sentAction != 2 && !this.m_sentRelease
      && (this.m_sentPose == UTPPMotionPoseRifle() || this.m_sentPose == UTPPMotionPosePistol1H() || this.m_sentPose == UTPPMotionPosePistol2H()) {
      UTPPFeedMotionMagTap();
    }
    if !this.m_reloadDiag {
      return true;
    }
    UTPPLogLine("UTPPMotion RELOAD refill-no-state t=" + FloatToStringPrec(EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance())), 2)
      + " rec=" + this.RightRecordName() + " mag=" + ToString(previous) + "->" + ToString(mag) + " " + this.ReloadContext()
      + " state=" + (this.m_reloadStateActive ? "1" : "0") + " act=" + ToString(this.m_sentAction));
    return true;
  }

  // ---- reload episode diagnostic (one line per episode) ----
  private final func RightRecordName() -> String {
    let weapon: wref<WeaponObject> = this.m_ammoWeapon;
    if !IsDefined(weapon) {
      return "-";
    }
    return TDBID.ToStringDEBUG(ItemID.GetTDBID(weapon.GetItemID()));
  }

  // loco=PSM.Locomotion (2 = sprint) / LocomotionDetailed, upper=PSM.UpperBody, rw=PSM.RightWeapon, sprint=Locomotion 2.
  private final func ReloadContext() -> String {
    let psm: ref<IBlackboard> = this.m_psm;
    if !IsDefined(psm) {
      return "loco=-";
    }
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let loco: Int32 = psm.GetInt(psmDef.Locomotion);
    return "loco=" + ToString(loco) + "/" + ToString(psm.GetInt(psmDef.LocomotionDetailed)) + " upper=" + ToString(psm.GetInt(psmDef.UpperBody))
      + " rw=" + ToString(psm.GetInt(psmDef.RightWeapon)) + " sprint=" + (loco == 2 ? "1" : "0");
  }

  private final func OpenReloadEpisode() -> Void {
    this.m_epOpen = true;
    this.m_epStart = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
    this.m_epEnters = this.m_reloadStateEnters;
    if this.m_reloadStateActive {
      this.m_epEnters -= 1; // the enter that opened this episode counts for it
    }
    this.m_epRefill = false;
    this.m_epMag = this.m_lastMag;
    this.m_epLoco = -1;
    this.m_epDetail = -1;
    this.m_epUpper = -1;
    this.m_epRight = -1;
    let psm: ref<IBlackboard> = this.m_psm;
    if IsDefined(psm) {
      let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
      this.m_epLoco = psm.GetInt(psmDef.Locomotion);
      this.m_epDetail = psm.GetInt(psmDef.LocomotionDetailed);
      this.m_epUpper = psm.GetInt(psmDef.UpperBody);
      this.m_epRight = psm.GetInt(psmDef.RightWeapon);
    }
    let weapon: wref<WeaponObject> = this.m_ammoWeapon;
    this.m_epRecord = IsDefined(weapon) ? ItemID.GetTDBID(weapon.GetItemID()) : TDBID.None();
  }

  private final func CloseReloadEpisode(why: String) -> Void {
    if !this.m_epOpen {
      return;
    }
    this.m_epOpen = false;
    if !this.m_reloadDiag {
      return;
    }
    let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
    UTPPLogLine("UTPPMotion RELOAD episode t0=" + FloatToStringPrec(this.m_epStart, 2) + " t1=" + FloatToStringPrec(now, 2)
      + " dur=" + FloatToStringPrec(now - this.m_epStart, 2) + " rec=" + TDBID.ToStringDEBUG(this.m_epRecord)
      + " state=" + ToString(this.m_reloadStateEnters - this.m_epEnters) + " refill=" + (this.m_epRefill ? "1" : "0")
      + " mag=" + ToString(this.m_epMag) + "->" + ToString(this.m_lastMag)
      + " start:loco=" + ToString(this.m_epLoco) + "/" + ToString(this.m_epDetail) + " upper=" + ToString(this.m_epUpper)
      + " rw=" + ToString(this.m_epRight) + " sprint=" + (this.m_epLoco == 2 ? "1" : "0")
      + " end:" + this.ReloadContext() + " why=" + why);
  }

  private final func ResetReloadGrip() -> Void {
    this.m_reloadGrip = 0;
    this.m_reloadTailUntil = 0.00;
    this.m_previousPistol = false;
    this.m_reloadGeneration += 1;
  }

  public final func RefreshReloadGripTail(generation: Int32) -> Void {
    if this.m_enabled && generation == this.m_reloadGeneration {
      this.Refresh("reload-grip-tail");
    }
  }

  private final func ReloadGrip(pose: Int32, release: Bool, action: Int32) -> Int32 {
    if release || (pose != 2 && pose != 3) || (action != 0 && action != 2) || !IsDefined(this.m_player) {
      this.ResetReloadGrip();
      return pose;
    }
    let weapon: ref<WeaponObject> = GameInstance.GetTransactionSystem(this.m_player.GetGame()).GetItemInSlot(this.m_player, t"AttachmentSlots.WeaponRight") as WeaponObject;
    if !IsDefined(weapon) {
      this.ResetReloadGrip();
      return pose;
    }
    let entity: EntityID = weapon.GetEntityID();
    let record: TweakDBID = ItemID.GetTDBID(weapon.GetItemID());
    let samePrevious: Bool = this.m_previousPistol && entity == this.m_previousEntity && Equals(record, this.m_previousRecord);
    if this.m_reloadGrip != 0 && (entity != this.m_reloadEntity || !Equals(record, this.m_reloadRecord)) {
      this.ResetReloadGrip();
      samePrevious = false;
    }
    let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
    if action == 2 {
      if this.m_reloadGrip == 0 {
        // If attach/enable began inside Reload, entry history is unavailable:
        // use the current explicit classification, never another weapon's pose.
        this.m_reloadGrip = samePrevious && (this.m_sentPose == 2 || this.m_sentPose == 3) ? this.m_sentPose : pose;
        this.m_reloadEntity = entity;
        this.m_reloadRecord = record;
      }
      this.m_reloadTailUntil = 0.00;
    } else {
      if this.m_reloadGrip != 0 && this.m_reloadTailUntil == 0.00 {
        this.m_reloadTailUntil = now + 0.20;
        this.m_reloadGeneration += 1;
        let callback: ref<UTPPMotionReloadGripTail> = new UTPPMotionReloadGripTail();
        callback.glue = this;
        callback.generation = this.m_reloadGeneration;
        GameInstance.GetDelaySystem(this.GetGameInstance()).DelayCallback(callback, 0.21);
      }
      if this.m_reloadGrip != 0 && now >= this.m_reloadTailUntil {
        this.m_reloadGrip = 0;
        this.m_reloadTailUntil = 0.00;
        this.m_reloadGeneration += 1;
      }
    }
    this.m_previousPistol = true;
    this.m_previousEntity = entity;
    this.m_previousRecord = record;
    return this.m_reloadGrip != 0 ? this.m_reloadGrip : pose;
  }

  // the motion set-motionfix item 6 (weapon type latency 0.5-1 s): the weapon class is read from
  // AttachmentSlots.WeaponRight, which only holds the new weapon once the engine has attached it - the PSM / blackboard events
  // of an equip or swap fire earlier (read: old weapon) and the UI weapon record later, so the overlay followed the weapon
  // by the time between them. PlayerPuppet.OnItemEquipped / OnItemUnequipped (player.script:4026 / :4033, called with the
  // slot id when WeaponRight changes) and OnItemAddedToSlot (player.script:1892) are the events AT the slot change: read the
  // class there (Refresh pushes only on a change) and once more 0.2 s later (item records / PSM Weapon state settle).
  // No polling, no per-frame hook, nothing while motion is off.
  public final func OnWeaponSlot(why: String) -> Void {
    if !this.m_enabled {
      return;
    }
    this.Refresh(why);
    if this.m_recheckPending {
      return;
    }
    let cb: ref<UTPPMotionRecheckCallback> = new UTPPMotionRecheckCallback();
    cb.m_glue = this;
    this.m_recheckPending = true;
    GameInstance.GetDelaySystem(this.GetGameInstance()).DelayCallback(cb, 0.20, false);
  }

  public final func OnRecheck() -> Void {
    this.m_recheckPending = false;
    this.Refresh("recheck");
  }

  // Sends only on a change (or after enable / attach); does nothing while motion is off (no native call, no game read).
  public final func Refresh(why: String) -> Void {
    if !this.m_enabled {
      return;
    }
    this.m_updates += 1;
    // Context precedes overlay/action publication, including the first Shoot edge.
    this.RefreshRecoilWeapon();
    this.RefreshAmmoListener();
    this.RefreshBody(why);
    let pose: Int32;
    let aim: Int32;
    let release: Bool;
    let resolvedAimPose: Int32;
    this.ResolveInputs(pose, aim, release, resolvedAimPose);
    if this.m_reloadStateActive {
      this.ReloadStateHeld(); // evaluated on every refresh: a holster / unequip drops a reload state whose exit never came
    }
    let action: Int32 = 0;
    if !release && (pose == UTPPMotionPoseRifle() || pose == UTPPMotionPosePistol1H() || pose == UTPPMotionPosePistol2H()) && IsDefined(this.m_psm) {
      action = this.RangedAction(this.m_psm);
    }
    pose = this.ReloadGrip(pose, release, action);
    let injured: Bool = this.IsInjured();
    if pose != this.m_sentPose || resolvedAimPose != this.m_sentResolvedAimPose || aim != this.m_sentAim || NotEquals(release, this.m_sentRelease) || NotEquals(injured, this.m_sentInjured) {
      // Publish the atomic grip choice before an action event can consume it.
      UTPPSetMMOverlayInputs(pose, aim, release, injured, resolvedAimPose);
      this.m_sentPose = pose;
      this.m_sentResolvedAimPose = resolvedAimPose;
      this.m_sentAim = aim;
      this.m_sentRelease = release;
      this.m_sentInjured = injured;
      this.m_pushes += 1;
      this.m_lastWhy = why;
    }
    if action != this.m_sentAction {
      if action != 2 && this.m_sentAction == 2 {
        this.CloseReloadEpisode(why);
      }
      UTPPSetMMWeaponAction(action);
      if action == 2 && this.m_sentAction != 2 {
        this.OpenReloadEpisode();
      }
      this.m_sentAction = action;
    }
    this.LogDual(why);
  }

  // ---- dual-wield D0 observability (log only; changes no decision) ----
  // "[utpp_dual]" lines, written only while dual wielding (UpperBody DualWield 7 or PSM.IsDualWielding, IsDualWielding below) and
  // once when it ends; a single weapon produces none. Called at the end of every Refresh (PSM / weapon change events), so a
  // line follows each change of the stable tuple (upper, flag, records, selection), and at most one line per second otherwise
  // (the per-shot states rw / lw / shoot, or an unchanged heartbeat) while dual wielding:
  //   why=  the Refresh cause (psm, weapon, ...; hb = 1 s heartbeat of an unchanged tuple; exit = dual wield ended)
  //   upper=PSM.UpperBody raw (7 = DualWield)  flag=PSM.IsDualWielding  rw / lw=PSM.RightWeapon / LeftWeapon
  //   shoot=PSM.ShootState (gamePSMShootStates, the right / left / both arbiter)  shots=Shoot entries seen R/L since dual started
  //   R / L=<record>|<gamedataItemType>|<entity id> of AttachmentSlots.WeaponRight / WeaponLeft ("-" = empty slot)
  //   sel=WeaponRight (ResolveInputs reads only the right slot: overlay table, pistol family, grip and recoil follow it)
  //   pose / resolved / aim / src = what was last sent to the DLL (m_sent*, m_pistolSource)
  private final func WeaponDesc(player: ref<PlayerPuppet>, slot: TweakDBID) -> String {
    let ts: ref<TransactionSystem> = GameInstance.GetTransactionSystem(player.GetGame());
    let weapon: ref<WeaponObject> = ts.GetItemInSlot(player, slot) as WeaponObject;
    if !IsDefined(weapon) {
      return "-";
    }
    let kind: gamedataItemType = WeaponObject.GetWeaponType(weapon.GetItemID());
    return TDBID.ToStringDEBUG(ItemID.GetTDBID(weapon.GetItemID())) + "|" + EnumValueToString("gamedataItemType", Cast<Int64>(EnumInt(kind)))
      + "|" + EntityID.ToDebugStringDecimal(weapon.GetEntityID());
  }

  public final func OnDualFlush() -> Void {
    this.m_dualFlushPending = false;
    this.LogDual("flush");
  }

  private final func LogDual(why: String) -> Void {
    let psm: ref<IBlackboard> = this.m_psm;
    let player: ref<PlayerPuppet> = this.m_player;
    if !IsDefined(psm) || !IsDefined(player) {
      return;
    }
    let psmDef: ref<PlayerStateMachineDef> = GetAllBlackboardDefs().PlayerStateMachine;
    let upper: Int32 = psm.GetInt(psmDef.UpperBody);
    let dual: Bool = this.IsDualWielding(psm, upper);
    if !dual && !this.m_dualOn {
      return;
    }
    let now: Float = EngineTime.ToFloat(GameInstance.GetSimTime(this.GetGameInstance()));
    let cause: String = why;
    if dual && !this.m_dualOn {
      this.m_dualOn = true;
      this.m_dualSig = "";
      this.m_dualShotsRight = 0;
      this.m_dualShotsLeft = 0;
      this.m_dualPrevRight = 0;
      this.m_dualPrevLeft = 0;
      cause = "enter:" + why;
    }
    let rw: Int32 = psm.GetInt(psmDef.RightWeapon);
    let lw: Int32 = psm.GetInt(psmDef.LeftWeapon);
    if rw == 8 && this.m_dualPrevRight != 8 {
      this.m_dualShotsRight += 1;
    }
    if lw == 8 && this.m_dualPrevLeft != 8 {
      this.m_dualShotsLeft += 1;
    }
    this.m_dualPrevRight = rw;
    this.m_dualPrevLeft = lw;
    let vol: String = " rw=" + IntToString(rw) + " lw=" + IntToString(lw) + " shoot=" + IntToString(psm.GetInt(psmDef.ShootState));
    let sig: String = "upper=" + IntToString(upper) + " flag=" + ToString(psm.GetBool(psmDef.IsDualWielding))
      + " R=" + this.WeaponDesc(player, t"AttachmentSlots.WeaponRight") + " L=" + this.WeaponDesc(player, t"AttachmentSlots.WeaponLeft")
      + " sel=WeaponRight pose=" + IntToString(this.m_sentPose) + " resolved=" + IntToString(this.m_sentResolvedAimPose)
      + " aim=" + IntToString(this.m_sentAim) + " src=" + this.m_pistolSource;
    if !dual {
      this.m_dualOn = false;
      cause = "exit:" + why;
    } else {
      if Equals(sig, this.m_dualSig) {
        // only the per-shot states (rw / lw / shoot) moved, or nothing: at most one line per second
        if now - this.m_dualLogAt < 1.00 {
          // the resting state after a burst must not stay unlogged: one trailing flush (LogDual only) when it differs from the last line
          if !Equals(vol, this.m_dualVol) && !this.m_dualFlushPending {
            let cb: ref<UTPPMotionDualFlushCallback> = new UTPPMotionDualFlushCallback();
            cb.m_glue = this;
            this.m_dualFlushPending = true;
            GameInstance.GetDelaySystem(this.GetGameInstance()).DelayCallback(cb, 1.00 - (now - this.m_dualLogAt) + 0.02, false);
          }
          return;
        }
        cause = Equals(vol, this.m_dualVol) ? "hb" : "state:" + why;
      }
    }
    this.m_dualVol = vol;
    this.m_dualSig = sig;
    this.m_dualLogAt = now;
    UTPPLogLine("[utpp_dual] why=" + cause + " t=" + FloatToStringPrec(now, 2) + " " + sig + vol
      + " shots=" + IntToString(this.m_dualShotsRight) + "/" + IntToString(this.m_dualShotsLeft));
  }

  // A local monotonic epoch identifies the verified equipped object. Never cast
  // EntityID to a guessed integer layout; stale callbacks cannot select a curve.
  private final func ResetRecoilWeapon() -> Void {
    this.m_recoilWeapon = null;
    this.m_recoilSupported = false;
    this.m_recoilProfile = 0;
    UTPPSetMMRecoilWeapon(Cast<Uint32>(0), 0);
  }

  // Authored recoil profile of the equipped weapon (RecoilProfiles.hpp ids). The Nue stays bound to its exact preset id; every
  // other weapon is resolved from the family tag of its record. A record that carries two different family tags, an unknown
  // family or a missing record yields 0 (fail closed: the plain authored aim pose, no recoil kick).
  private final func ResolveRecoilProfile(weapon: ref<WeaponObject>) -> Int32 {
    let recordID: TweakDBID = ItemID.GetTDBID(weapon.GetItemID());
    if Equals(recordID, t"Items.Preset_Nue_Default") {
      return 1;
    }
    let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(recordID);
    if !IsDefined(rec) {
      return 0;
    }
    let tags: array<CName> = rec.Tags();
    let profile: Int32 = 0;
    let index: Int32 = 0;
    while index < ArraySize(tags) {
      let candidate: Int32 = UTPPMotionRecoilProfileOfFamily(tags[index]);
      if candidate != 0 {
        if profile != 0 && candidate != profile {
          return 0;
        }
        profile = candidate;
      }
      index += 1;
    }
    if profile == 0 {
      // no usable family tag: the animation wrapper of the record (Kappa, MA70, Umbra, untagged presets)
      profile = UTPPMotionRecoilProfileOfFamily(UTPPMotionRecoilFamilyOfAnimset(rec.FriendlyName()));
    }
    if profile == 0 {
      return this.FallbackRecoilProfile(weapon);
    }
    return profile;
  }

  // Class fallback for a record whose family tag is not in the table (Kenshin/Kappa/Omaha pistols,
  // Guillotine SMG, shotguns, precision/sniper rifles, LMG/HMG had no profile = no recoil kick at all). Ids are RecoilProfiles.hpp
  // table indexes: 2 Liberty (pistols), 18 Saratoga (SMG), 12 Ajax (assault/other rifles, LMG). Revolvers stay 0: their profiles were
  // retired deliberately. A record with two conflicting family tags still returns 0 above (fail closed).
  private final func FallbackRecoilProfile(weapon: ref<WeaponObject>) -> Int32 {
    let t: gamedataItemType = WeaponObject.GetWeaponType(weapon.GetItemID());
    if Equals(t, gamedataItemType.Wea_Handgun) {
      return 2;
    }
    if Equals(t, gamedataItemType.Wea_SubmachineGun) {
      return 18;
    }
    if Equals(t, gamedataItemType.Wea_AssaultRifle) || Equals(t, gamedataItemType.Wea_Rifle)
       || Equals(t, gamedataItemType.Wea_LightMachineGun) {
      return 12;
    }
    // Shotguns, precision/sniper rifles and HMGs stay 0 on purpose: their native shot animation carries a bolt/pump cycle that an
    // Ajax kick would replace; add them only with a profile that keeps that cycle.
    return 0;
  }

  private final func RefreshRecoilWeapon() -> Void {
    let player: ref<PlayerPuppet> = this.m_player;
    let currentPlayer: ref<PlayerPuppet> = GetPlayer(this.GetGameInstance());
    let weapon: ref<WeaponObject>;
    let previous: wref<WeaponObject> = this.m_recoilWeapon;
    if !this.m_enabled || !IsDefined(player) || !IsDefined(currentPlayer) || !IsDefined(this.m_psm) {
      this.ResetRecoilWeapon();
      return;
    }
    if NotEquals(player.GetEntityID(), currentPlayer.GetEntityID()) {
      this.ResetRecoilWeapon();
      return;
    }
    weapon = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
    if !IsDefined(weapon) {
      this.ResetRecoilWeapon();
      return;
    }
    if !IsDefined(previous) || NotEquals(previous.GetEntityID(), weapon.GetEntityID()) {
      // Fail closed instead of recycling a token after a pathological session.
      if this.m_recoilEpoch >= Cast<Uint32>(2147483647) {
        this.ResetRecoilWeapon();
        return;
      }
      this.m_recoilEpoch += Cast<Uint32>(1);
      this.m_recoilWeapon = weapon;
      // The record of an equipped object never changes: resolve its profile once per object, not on every refresh or shot.
      this.m_recoilProfile = this.ResolveRecoilProfile(weapon);
      this.m_recoilSupported = this.m_recoilProfile != 0;
    }
    UTPPSetMMRecoilWeapon(this.m_recoilEpoch, this.m_recoilProfile);
  }

  // Called after vanilla ShootEvents.OnEnter, once for each native Shoot event.
  // Re-read the slot here: UI record notifications alone cannot identify a shot.
  public final func FeedRecoilShot(weapon: ref<WeaponObject>) -> Void {
    this.RefreshRecoilWeapon();
    let current: wref<WeaponObject> = this.m_recoilWeapon;
    if !this.m_enabled || !this.m_recoilSupported || !IsDefined(current) || !IsDefined(weapon) || !IsDefined(this.m_psm) {
      return;
    }
    if NotEquals(current.GetEntityID(), weapon.GetEntityID()) {
      return;
    }
    UTPPFeedMMRecoilShot(this.m_recoilEpoch, this.m_psm.GetInt(GetAllBlackboardDefs().PlayerStateMachine.UpperBody) == 6);
  }

  // ---- CET API (cet\utpp_motion) ----
  public final func SetEnabled(on: Bool) -> Void {
    if !on { this.ResetRecoilWeapon(); this.ResetReloadGrip(); }
    if !on && this.m_enabled {
      UTPPSetMMWeaponAction(0);
    }
    this.m_enabled = on;
    UTPPSetMotionEnabled(on);
    this.m_sentBody = -100;
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.m_sentAim = -100;
    this.m_sentAction = -100;
    if on {
      this.Start("enable");
    } else {
      this.UnregisterAll();
    }
  }

  public final func SetPistolOneHand(on: Bool) -> Void {
    this.m_pistolOneHand = on;
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.Refresh("pistol1h");
  }

  // Model tags confirmed in retail for 28 records; capability tags are not pose tags.
  private final func ResolvePistolPose(weapon: ref<WeaponObject>, aiming: Bool) -> Int32 {
    return this.ResolvePistolRecord(ItemID.GetTDBID(weapon.GetItemID()), aiming);
  }

  // Reload asset family is chosen with the episode's latched Ready/Aim state.
  // Use the same resolver as locomotion, including per-record overrides.
  public final func ReloadPistolPose(aiming: Bool) -> Int32 {
    // Publish the verified equipped-object epoch before granting authored
    // contacts, including a reload immediately after equip/attach.
    this.RefreshRecoilWeapon();
    let player: ref<PlayerPuppet> = this.m_player;
    let current: ref<PlayerPuppet> = GetPlayer(this.GetGameInstance());
    if !this.m_enabled || !IsDefined(player) || !IsDefined(current) {
      return UTPPMotionPoseDefault();
    }
    if NotEquals(player.GetEntityID(), current.GetEntityID()) {
      return UTPPMotionPoseDefault();
    }
    let weapon: ref<WeaponObject> = GameInstance.GetTransactionSystem(player.GetGame()).GetItemInSlot(player, t"AttachmentSlots.WeaponRight") as WeaponObject;
    if !IsDefined(weapon) || this.TablePose(WeaponObject.GetWeaponType(weapon.GetItemID())) != UTPPMotionPoseHandgun() {
      return UTPPMotionPoseDefault();
    }
    let previousSource: String = this.m_pistolSource;
    let pose: Int32 = this.ResolvePistolPose(weapon, aiming);
    this.m_pistolSource = previousSource;
    return pose;
  }

  // Diagnostic preview runs the same resolver without equipping an item or publishing a pose.
  public final func PreviewPistolPose(recordID: TweakDBID, aiming: Bool) -> String {
    let kind: gamedataItemType = WeaponObject.GetWeaponType(ItemID.CreateQuery(recordID));
    if NotEquals(kind, gamedataItemType.Wea_Handgun) && NotEquals(kind, gamedataItemType.Wea_Revolver) {
      return "not-pistol";
    }
    let previousSource: String = this.m_pistolSource;
    let pose: Int32 = this.ResolvePistolRecord(recordID, aiming);
    let result: String = "pose=" + IntToString(pose) + " source=" + this.m_pistolSource;
    this.m_pistolSource = previousSource;
    return result;
  }

  private final func ResolvePistolRecord(recordID: TweakDBID, aiming: Bool) -> Int32 {
    let index: Int32 = 0;
    while index < ArraySize(this.m_pistolOverrideIDs) {
      if Equals(recordID, this.m_pistolOverrideIDs[index]) {
        let overridePose: Int32 = aiming ? this.m_pistolOverrideAim[index] : this.m_pistolOverrideReady[index];
        if overridePose != 0 {
          this.m_pistolSource = "record-override";
          return overridePose;
        }
        break;
      }
      index += 1;
    }
    this.m_pistolSource = "manual";
    if !this.m_pistolManual {
      let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(recordID);
      if IsDefined(rec) {
        let tags: array<CName> = rec.Tags();
        let families: array<CName>;
        ArrayPush(families, n"Kenshin"); ArrayPush(families, n"Yukimura");
        ArrayPush(families, n"Lexington"); ArrayPush(families, n"Liberty");
        ArrayPush(families, n"Nue"); ArrayPush(families, n"Omaha"); ArrayPush(families, n"Unity");
        ArrayPush(families, n"Burya"); ArrayPush(families, n"Nova"); ArrayPush(families, n"Overture");
        ArrayPush(families, n"Chao"); ArrayPush(families, n"Quasar"); ArrayPush(families, n"Slaughtomatic");
        let i: Int32 = 0;
        let found: Int32 = 0;
        let family: CName;
        while i < ArraySize(families) {
          if ArrayContains(tags, families[i]) { found += 1; family = families[i]; }
          i += 1;
        }
        if found == 1 {
          this.m_pistolSource = "family:" + NameToString(family);
          if Equals(family, n"Slaughtomatic") { return UTPPMotionPosePistol1H(); }
          if !aiming && (Equals(family, n"Burya") || Equals(family, n"Nova") || Equals(family, n"Overture") || Equals(family, n"Quasar")) {
            return UTPPMotionPosePistol1H();
          }
          return UTPPMotionPosePistol2H();
        }
        // This family has no model tag in retail. Exact observed IDs, never a substring guess.
        if found == 0 && (Equals(recordID, t"Items.Base_Silverhand") || Equals(recordID, t"Items.Silverhand_Malorian") || Equals(recordID, t"Items.Preset_Silverhand_3516")) {
          this.m_pistolSource = "record-family:Silverhand";
          return aiming ? UTPPMotionPosePistol2H() : UTPPMotionPosePistol1H();
        }
        this.m_pistolSource = found > 1 ? "fallback:conflicting-family" : "fallback:unknown-family";
      } else {
        this.m_pistolSource = "fallback:missing-record";
      }
    }
    return this.m_pistolOneHand ? UTPPMotionPosePistol1H() : UTPPMotionPosePistol2H();
  }

  public final func SetPistolAuto(on: Bool) -> Void {
    this.m_pistolManual = !on;
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.Refresh("pistolAuto");
  }

  // 0 = automatic for this state, 2 = Pistol1H, 3 = Pistol2H. Explicit record override wins.
  public final func SetWeaponPoseOverride(id: TweakDBID, ready: Int32, aim: Int32) -> Bool {
    if (ready != 0 && ready != 2 && ready != 3) || (aim != 0 && aim != 2 && aim != 3) { return false; }
    // Removal must remain possible if another mod changes or removes the weapon record.
    if ready != 0 || aim != 0 {
      let rec: ref<WeaponItem_Record> = TweakDBInterface.GetWeaponItemRecord(id);
      if !IsDefined(rec) { return false; }
      let kind: gamedataItemType = WeaponObject.GetWeaponType(ItemID.CreateQuery(id));
      if NotEquals(kind, gamedataItemType.Wea_Handgun) && NotEquals(kind, gamedataItemType.Wea_Revolver) { return false; }
    }
    let i: Int32 = 0;
    while i < ArraySize(this.m_pistolOverrideIDs) {
      if Equals(id, this.m_pistolOverrideIDs[i]) { break; }
      i += 1;
    }
    if i == ArraySize(this.m_pistolOverrideIDs) {
      if ready == 0 && aim == 0 { return true; }
      if i >= 64 { return false; }
      ArrayPush(this.m_pistolOverrideIDs, id);
      ArrayPush(this.m_pistolOverrideReady, ready);
      ArrayPush(this.m_pistolOverrideAim, aim);
    } else {
      if ready == 0 && aim == 0 {
        ArrayErase(this.m_pistolOverrideIDs, i);
        ArrayErase(this.m_pistolOverrideReady, i);
        ArrayErase(this.m_pistolOverrideAim, i);
      } else {
        this.m_pistolOverrideReady[i] = ready;
        this.m_pistolOverrideAim[i] = aim;
      }
    }
    this.m_sentPose = -100;
    this.m_sentResolvedAimPose = -100;
    this.Refresh("record-override");
    return true;
  }

  public final func IsEnabled() -> Bool {
    return this.m_enabled;
  }

  public final func Status() -> String {
    return "glue enabled=" + ToString(this.m_enabled) + " body=" + IntToString(this.m_body) + " pose=" + IntToString(this.m_sentPose)
      + " aim=" + IntToString(this.m_sentAim) + " release=" + ToString(this.m_sentRelease) + " injured=" + ToString(this.m_sentInjured)
      + " hp%=" + FloatToStringPrec(this.m_healthPerc, 1) + " bleed=" + ToString(this.m_bleeding) + " pistolPreference1h=" + ToString(this.m_pistolOneHand)
      + " pistolAuto=" + ToString(!this.m_pistolManual) + " lastPistolSource=" + this.m_pistolSource
      + " updates=" + IntToString(this.m_updates) + " pushes=" + IntToString(this.m_pushes) + " why=" + this.m_lastWhy
      + " | " + UTPPGetMotionStatus();
  }
}

public func UTPPMotionGetGlue(game: GameInstance) -> ref<UTPPMotionGlue> {
  return GameInstance.GetScriptableSystemsContainer(game).Get(n"UTPPMotionGlue") as UTPPMotionGlue;
}

public func UTPPMotionGlueStatus() -> String {
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(GetGameInstance());
  if !IsDefined(glue) {
    return "glue missing | " + UTPPGetMotionStatus();
  }
  return glue.Status();
}

public func UTPPMotionGlueSetEnabled(on: Bool) -> Bool {
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(GetGameInstance());
  if !IsDefined(glue) {
    UTPPSetMotionEnabled(on);
    return false;
  }
  glue.SetEnabled(on);
  return true;
}

public func UTPPMotionGlueSetPistolOneHand(on: Bool) -> Bool {
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(GetGameInstance());
  if !IsDefined(glue) {
    return false;
  }
  glue.SetPistolOneHand(on);
  return true;
}

// ---- change events on the player (not wrapped by any other UTPP script; redscript chains wraps of other mods) ----
// player.script:3714 OnHealthUpdateEvent: evt.value = health pool value from HealthStatListener.OnStatPoolValueChanged
// (playerListeners.script:600-608, newValue of the percentage listener; the game compares it with
// GetLowHealthThreshold in UpdateHealthStateSFX, player.script:3722-3725).
@wrapMethod(PlayerPuppet)
protected cb func OnHealthUpdateEvent(evt: ref<HealthUpdateEvent>) -> Bool {
  let r: Bool = wrappedMethod(evt);
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
  if IsDefined(glue) && IsDefined(evt) {
    glue.OnHealthPerc(evt.value);
  }
  return r;
}

// the motion set-motionfix item 6: the weapon slot events (player.script:4026 OnItemEquipped / :4033 OnItemUnequipped,
// :1892 OnItemAddedToSlot) - read-only wraps, wrappedMethod first.
@wrapMethod(PlayerPuppet)
public final func OnItemEquipped(slot: TweakDBID, item: ItemID) -> Void {
  wrappedMethod(slot, item);
  if slot == t"AttachmentSlots.WeaponRight" {
    let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
    if IsDefined(glue) {
      glue.OnWeaponSlot("equipped");
    }
  }
}

@wrapMethod(PlayerPuppet)
public final func OnItemUnequipped(slot: TweakDBID, item: ItemID) -> Void {
  wrappedMethod(slot, item);
  if slot == t"AttachmentSlots.WeaponRight" {
    let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
    if IsDefined(glue) {
      glue.OnWeaponSlot("unequipped");
    }
  }
}

@wrapMethod(PlayerPuppet)
protected cb func OnItemAddedToSlot(evt: ref<ItemAddedToSlot>) -> Bool {
  let r: Bool = wrappedMethod(evt);
  if IsDefined(evt) && evt.GetSlotID() == t"AttachmentSlots.WeaponRight" {
    let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
    if IsDefined(glue) {
      glue.OnWeaponSlot("slot");
    }
  }
  return r;
}

// player.script:4213 / :4688 - Bleeding applied / removed.
@wrapMethod(PlayerPuppet)
protected cb func OnStatusEffectApplied(evt: ref<ApplyStatusEffectEvent>) -> Bool {
  let r: Bool = wrappedMethod(evt);
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
  if IsDefined(glue) {
    glue.OnStatusEffectsChanged();
  }
  return r;
}

@wrapMethod(PlayerPuppet)
protected cb func OnStatusEffectRemoved(evt: ref<RemoveStatusEffect>) -> Bool {
  let r: Bool = wrappedMethod(evt);
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(this.GetGame());
  if IsDefined(glue) {
    glue.OnStatusEffectsChanged();
  }
  return r;
}

// ---- climb / vault start -> UTPPFeedTraversal (MotionTraversal, MotionDriver.hpp FeedTraversal) ----
// Read exactly as the game reads it at the state start (vanilla cyberpunk\player\psm\
// locomotionTransitions.script = LT): PlayerObstacleSystem.GetCurrentClimbInfo(owner) (LT:2721 climb, LT:3052 vault) ->
// PlayerClimbInfo (orphans.script:58976: descResult, obstacleEnd, inputDirection) / GeometryDescriptionResult
// (orphans.script:27659: topPoint, collisionNormal, behindPoint, topExtent, left/rightHandData). Player position =
// DefaultTransition.GetPlayerPosition (LT:2722 / LT:3049), forward = GetOwnerForward (LT:2719). Hand IK target per hand =
// grabPointEnd + 0.5 * (grabPointStart - grabPointEnd) (LT:2767 / LT:2770, HandIKDescriptionResult orphans.script:51946);
// climb sends hands only where the game itself IKs them (m_shouldIkHands, High / Mid, LT:2731-2743, set by
// GetClimbParameter inside OnEnter LT:2803 before the wrap reads it). ClimbSpeedModifier read like LT:2746-2749 (<= 0 -> 1).
// The wraps call wrappedMethod first (vanilla + other mods' wraps run unchanged, redscript chains them) and only read.
public func UTPPMotionTraversalHand(const h: HandIKDescriptionResult) -> Vector4 {
  return h.grabPointEnd + (h.grabPointStart - h.grabPointEnd) * 0.50;
}

// the motion set-next: t = the state's live static parameters read by the wrap like the game
// (GetStaticFloatParameterDefault, same names and defaults as LT:2727-2751 / LT:3063): [highThreshold, midThreshold,
// verticalDurationHigh/Mid/Low, horizontalDurationHigh/Mid/Low, forwardStep]; -1 = not used by that action (DLL default).
public func UTPPMotionFeedTraversalFrom(action: Int32, scriptInterface: ref<StateGameScriptInterface>, ikHands: Bool,
                                           airborne: Bool, climbSpeed: Float, const t: array<Float>) -> Void {
  if ArraySize(t) != 9 {
    return;
  }
  if !IsDefined(scriptInterface) || !IsDefined(scriptInterface.owner) {
    return;
  }
  let glue: ref<UTPPMotionGlue> = UTPPMotionGetGlue(scriptInterface.owner.GetGame());
  if !IsDefined(glue) || !glue.IsEnabled() {
    return; // motion off: no native call (the base path never sees traversal events)
  }
  let climbInfo: ref<PlayerClimbInfo> = scriptInterface.GetSpatialQueriesSystem().GetPlayerObstacleSystem().GetCurrentClimbInfo(scriptInterface.owner);
  if !IsDefined(climbInfo) || !IsDefined(climbInfo.descResult) {
    return;
  }
  let d: ref<GeometryDescriptionResult> = climbInfo.descResult;
  let vel: Vector4 = DefaultTransition.GetLinearVelocity(scriptInterface);
  vel.Z = 0.00;
  // obstacle kind (orphans.script:27701 GeometryDescriptionResult.climbedEntity, read by vanilla ClimbEvents LT:2728)
  let kind: Int32 = 0;
  let climbed: ref<Entity> = d.climbedEntity;
  if IsDefined(climbed) {
    let veh: ref<VehicleObject> = climbed as VehicleObject;
    kind = IsDefined(veh) ? 1 : 2;
  }
  UTPPFeedTraversalObstacleKind(kind);
  UTPPFeedTraversal(action, d.topPoint, d.collisionNormal, d.topExtent, d.behindPoint, climbInfo.obstacleEnd,
    climbInfo.inputDirection, ikHands, UTPPMotionTraversalHand(d.leftHandData), UTPPMotionTraversalHand(d.rightHandData),
    DefaultTransition.GetPlayerPosition(scriptInterface), scriptInterface.GetOwnerForward(), Vector4.Length(vel), airborne,
    climbSpeed, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8]);
}

// LT:2800 ClimbEvents.OnEnter (sets LocomotionDetailed Climb 8, LT:2804).
@wrapMethod(ClimbEvents)
public func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  let climbSpeed: Float = this.GetStatFloatValue(scriptInterface, gamedataStatType.ClimbSpeedModifier, scriptInterface.GetStatsSystem());
  if climbSpeed <= 0.00 {
    climbSpeed = 1.00;
  }
  let t: array<Float>;
  ArrayPush(t, this.GetStaticFloatParameterDefault("highThreshold", 1.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("midThreshold", 1.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("verticalDurationHigh", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("verticalDurationMid", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("verticalDurationLow", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("horizontalDurationHigh", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("horizontalDurationMid", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("horizontalDurationLow", 10.00));
  ArrayPush(t, this.GetStaticFloatParameterDefault("forwardStep", 0.50));
  UTPPMotionFeedTraversalFrom(8, scriptInterface, this.m_shouldIkHands, !this.IsTouchingGround(scriptInterface), climbSpeed, t);
}

// LT:3069 VaultEvents.OnEnter (LocomotionDetailed Vault 9). Vault has no hand IK and no duration in script (LT:3046-3066).
@wrapMethod(VaultEvents)
public func OnEnter(stateContext: ref<StateContext>, scriptInterface: ref<StateGameScriptInterface>) -> Void {
  wrappedMethod(stateContext, scriptInterface);
  let t: array<Float>;
  let i: Int32 = 0;
  while i < 8 {
    ArrayPush(t, -1.00); // no climb class / duration for a vault (LT:3046-3066)
    i += 1;
  }
  ArrayPush(t, this.GetStaticFloatParameterDefault("forwardStep", 0.50));
  UTPPMotionFeedTraversalFrom(9, scriptInterface, false, !this.IsTouchingGround(scriptInterface), 1.00, t);
}

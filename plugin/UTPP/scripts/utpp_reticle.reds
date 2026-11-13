// UTPP crosshair for ordinary guns: a small dot and a thin ring of 4 arcs (gaps on the diagonals) in the game's crosshair
// colour, the ring's radius following the game's bullet spread and kicked outwards by every shot, a soft dark shadow under
// both, a black x hit marker and a thicker black x kill marker.
//
// Scope: the crosshair controller classes of ordinary guns, CrosshairGameController_Basic and CrosshairGameController_Simple
// (allow-list by class name, UTPPRetKindOf). Smart, tech / charge, power weapons with their own crosshair class, launchers,
// melee and no-weapon controllers stay as they are; scopes and ironsights are not crosshair controllers.
//
// How it works:
// - The hooks sit on the base class gameuiCrosshairBaseGameController. Every crosshair controller's OnPreIntro / OnPreOutro /
//   OnBulletSpreadChanged ends in the base version (the overrides call super), so the hooks run for whichever controller the
//   weapon's crosshair widget spawns, and the equip line names its class even when it is not handled.
// - The widgets are built once per handled controller, lazily, in the OnPreIntro wrap before the original base body runs (it
//   calls OnBulletSpreadChanged and UpdateCrosshairState right away). They hang under the controller's root compound widget;
//   without one the controller keeps the game's crosshair and logs root=none.
// - Visibility mirrors the game's own end state.
//   Basic: each wrap of ExpandCrosshair / ColapseCrosshair (ring) and of ShowCenterPart / HideCenterPart (dot) first calls the
//   original, then fades our group to the same target with the same duration.
//   Simple: its states only show / hide the root (hip fire shown, everything else hidden), so both groups stay at full opacity
//   and follow the root.
//   Root-level hiding (vision mode, camo, base states that hide the root) is inherited by both. The forced hip-fire visual of
//   utpp_crosshair.reds in third-person aim therefore shows the reticle without any further hook.
// - Ownership: when on, the game's parts of the handled class are set invisible and our canvas visible; when off, the
//   reverse. The game keeps updating its own parts in both cases, so switching back is immediate and correct.
// - Ring radius: R = clamp(minSpread + pad, 16, 360) in ink units at rest (Basic: its own min spread; Simple: 20), plus a
//   transient kick that is a pure function of time (see "Shot kick" below); relaid out when it moves by 0.3 or more.
//   Hip fire (not the third-person aim state of utpp_crosshair.reds): every shot kicks the ring once (8 units, at most 24
//   in total) and it comes back in 0.25 s; the game's accumulated spread bloom is not used. Aiming: the game's spread bloom (excess = max(spread.X, spread.Y) above the weapon's resting spread floor,
//   smallest non-zero spread since the equip) and the shot kick enter as a kick, 2.5 * multiplier * excess, which falls back
//   in 0.25 s. The game's own bloom fade takes about 2.5 s, so the ring no longer follows it down.
//   The spread arrives through the base class's delayed blackboard listener.
// - Hit / kill: non-delayed listeners on UI_DamageInfo DamageList / KillList, the same filter as the game's hit indicator.
//   A hit only counts within 1.0 s after a ShootEvent of the active weapon (no x for damage over time, quickhacks or
//   grenades); if no ShootEvent arrived since the equip, hits are accepted ungated and that is logged once per equip.
//   A kill (also a non-lethal defeat) stops the hit x, plays the thick x and suppresses hits for 0.2 s.
// - Ring: 4 arcs of 60 degrees (gaps on the diagonals) made of 3 degree chords of the game's Rectangle shape (antialiased),
//   with a thicker black copy behind each as the shadow; console fallback Game.UTPPSetCrosshairArcs(2) = plain rectangle
//   chords, (6) or (-1) = the default. Line thickness stays constant at any radius (see "ring" below). The x strokes use the
//   rotated containers too.
// - Shot kick: every shot (UI ShootEvent or a drop of the weapon's magazine count, de-duplicated within 40 ms) adds a step to
//   the radius (capped), shrinking back in about 0.25 s as an ink animation.
// - Colour: the dot and the arcs use the game's crosshair colour. They carry the game's crosshair style
//   (crosshair_colors.inkstyle) with tintColor bound to Crosshair.mainColor (arcs) / Crosshair.dotColor (dot) and follow the
//   controller's GUI state (Civilian / Hostile / Friendly / Dead, from ApplyCrosshairGUIState). Before the binding they get
//   the tint read from the game's own part, or the HUD cyan when that part has no colour. Shadow and the x stay black.
//   Setting crosshair.color (CET -> Game.UTPPSetCrosshairColor(0..4)): 0 Standard = the above; 1 White, 2 Red, 3 Green,
//   4 Black = a fixed tint (the style binding is removed, so the HUD style cannot repaint it). With Black the shadows turn
//   light (white, 0.4) and the hit / kill x white, so they stay readable.
//
// Setting: CET utpp_core "crosshair.impl" (default on) -> Game.UTPPSetCrosshair(on) -> UTPPCrosshairState (per game session;
// without a push the reticle is on). The setter applies the change to the active controller immediately.
// Log ([utpp_reticle], UTPPLogLine): one "equip" line per crosshair controller OnPreIntro (every class, handled=0|1), one "on="
// line per change, at most one "shot gate unavailable" line per equip, one "centre dot suppressed=" line per change of the
// accessibility centre dot (hidden while our reticle owns the centre). The equip line lists the root children hidden for a
// Simple controller (hidden=<names>). Console: print(Game.UTPPCrosshairStatus())
// Requires utpp.reds (UTPPLogLine).

// ---------------------------------------------------------------- state (CET bridge)

public class UTPPCrosshairState extends ScriptableSystem {
  private let m_off: Bool;
  private let m_rev: Int32;
  private let m_calls: Int32;
  public let m_active: wref<gameuiCrosshairBaseGameController>;
  public let m_hits: Int32;
  public let m_kills: Int32;
  public let m_equips: Int32;
  public let m_lastClass: CName;
  public let m_lastHandled: Bool;
  public let m_centerDot: wref<CrosshairGameControllerPersistentDot>;
  public let m_arcModeSet: Int32; // 0 = not set (default mode), else mode + 1
  public let m_diagLines: Int32;  // radius diagnostic lines written this session (max 40)
  public let m_color: Int32;      // 0 Standard (game colour), 1 White, 2 Red, 3 Green, 4 Black

  // Returns true when the value changed.
  public func SetColor(color: Int32) -> Bool {
    let c: Int32 = Max(0, Min(4, color));
    if c == this.m_color {
      return false;
    }
    this.m_color = c;
    UTPPLogLine("[utpp_reticle] color=" + UTPPRetColourModeName(c));
    if IsDefined(this.m_active) {
      this.m_active.UTPPRetApplyColour();
    }
    return true;
  }

  // Returns true when the value changed.
  public func SetOn(on: Bool) -> Bool {
    this.m_calls += 1;
    if Equals(!this.m_off, on) {
      return false;
    }
    this.m_off = !on;
    this.m_rev += 1;
    UTPPLogLine("[utpp_reticle] on=" + UTPPRetFlag(on) + " rev=" + IntToString(this.m_rev));
    if IsDefined(this.m_active) {
      this.m_active.UTPPReticleApply();
    }
    this.RefreshCentreDot();
    return true;
  }

  public func IsOn() -> Bool {
    return !this.m_off;
  }

  // True while our reticle owns the screen centre: on, and a handled controller with built widgets is active.
  public func OwnsCentre() -> Bool {
    return !this.m_off && IsDefined(this.m_active) && IsDefined(this.m_active.utppReticle);
  }

  // Re-evaluates the accessibility centre dot (hidden while our reticle owns the centre).
  public func RefreshCentreDot() -> Void {
    if IsDefined(this.m_centerDot) {
      this.m_centerDot.UTPPRetRefresh();
    }
  }

  public func Status() -> String {
    let s: String = "on=" + UTPPRetFlag(!this.m_off) + " rev=" + IntToString(this.m_rev) + " calls="
      + IntToString(this.m_calls) + " hits=" + IntToString(this.m_hits) + " kills=" + IntToString(this.m_kills)
      + " equips=" + IntToString(this.m_equips) + " last=" + NameToString(this.m_lastClass) + " lastHandled="
      + UTPPRetFlag(this.m_lastHandled) + " arcs=" + UTPPRetArcModeName(UTPPRetArcModeOf(GetGameInstance()));
    if IsDefined(this.m_active) {
      return s + " " + this.m_active.UTPPReticleStatus();
    }
    return s + " active=none";
  }
}

public func UTPPCrosshairStateOf(game: GameInstance) -> ref<UTPPCrosshairState> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(game);
  if !IsDefined(container) {
    return null;
  }
  return container.Get(n"UTPPCrosshairState") as UTPPCrosshairState;
}

// CET: Game.UTPPSetCrosshair(true|false). False = no game session yet; nothing stored.
public func UTPPSetCrosshair(on: Bool) -> Bool {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetOn(on);
  return true;
}

// Console: Game.UTPPSetCrosshairArcs(n): 6 = Rectangle-shape chords (default), 2 = plain rectangle chords (fallback),
// -1 = back to the default. Rebuilds the ring of the active controller at once and logs the build; per game session.
public func UTPPSetCrosshairArcs(mode: Int32) -> Bool {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  if mode != -1 && mode != 2 && mode != 6 {
    UTPPLogLine("[utpp_reticle] arcs: unsupported mode " + IntToString(mode) + " (use 6, 2 or -1)");
    return false;
  }
  if mode < 0 {
    state.m_arcModeSet = 0;
  } else {
    state.m_arcModeSet = mode + 1;
  }
  UTPPLogLine("[utpp_reticle] arcs=" + UTPPRetArcModeName(UTPPRetArcModeOf(GetGameInstance())) + " (console)");
  if IsDefined(state.m_active) {
    state.m_active.UTPPRetBuildRing();
  }
  return true;
}

// CET: Game.UTPPSetCrosshairColor(0..4) (crosshair.color). False = no game session yet; nothing stored.
public func UTPPSetCrosshairColor(color: Int32) -> Bool {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) {
    return false;
  }
  state.SetColor(color);
  return true;
}

// Missing system (no push yet) -> 0, Standard.
public func UTPPRetColourModeOf(game: GameInstance) -> Int32 {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(game);
  if !IsDefined(state) {
    return 0;
  }
  return state.m_color;
}

public func UTPPRetColourModeName(mode: Int32) -> String {
  switch mode {
    case 1:
      return "white";
    case 2:
      return "red";
    case 3:
      return "green";
    case 4:
      return "black";
  }
  return "standard";
}

// The fixed tint of modes 1..4 (mode 0 has none).
public func UTPPRetFixedColour(mode: Int32) -> HDRColor {
  switch mode {
    case 1:
      return new HDRColor(1.0, 1.0, 1.0, 1.0);
    case 2:
      return new HDRColor(1.0, 0.18, 0.15, 1.0);
    case 3:
      return new HDRColor(0.2, 1.0, 0.3, 1.0);
    case 4:
      return new HDRColor(0.0, 0.0, 0.0, 1.0);
  }
  return new HDRColor(1.0, 1.0, 1.0, 1.0);
}

public func UTPPRetArcModeDefault() -> Int32 {
  return 6;
}

public func UTPPRetArcModeOf(game: GameInstance) -> Int32 {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(game);
  if !IsDefined(state) || state.m_arcModeSet <= 0 {
    return UTPPRetArcModeDefault();
  }
  return state.m_arcModeSet - 1;
}

public func UTPPRetArcModeName(mode: Int32) -> String {
  if mode == 2 {
    return "2:rect3";
  }
  if mode == 6 {
    return "6:shapeChord3";
  }
  return "?";
}

// Degrees per chord (both modes).
public func UTPPRetChordStep(mode: Int32) -> Float {
  return 3.0;
}

// Missing system (no push yet) -> on, the default.
public func UTPPCrosshairOn(game: GameInstance) -> Bool {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(game);
  if !IsDefined(state) {
    return true;
  }
  return state.IsOn();
}

// Console: print(Game.UTPPCrosshairStatus())
public func UTPPCrosshairStatus() -> String {
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) {
    return "nosystem";
  }
  return state.Status();
}

public func UTPPRetFlag(b: Bool) -> String {
  if b {
    return "1";
  }
  return "0";
}

// Allow-list: 0 = not handled (the game's crosshair stays), 1 = Basic, 2 = Simple.
public func UTPPRetKindOf(cls: CName) -> Int32 {
  if Equals(cls, n"CrosshairGameController_Basic") {
    return 1;
  }
  if Equals(cls, n"CrosshairGameController_Simple") {
    return 2;
  }
  return 0;
}

// ---------------------------------------------------------------- style constants (ink reference units, 4K authoring)

public func UTPPRetRPad() -> Float {
  return 0.0;
}

public func UTPPRetBox() -> Float {
  return 1024.0;
}

public func UTPPRetArcThickness() -> Float {
  return 4.0;
}

// Share of the bullet spread added to the radius (the resting size stays: min spread + pad).
public func UTPPRetSpreadScale() -> Float {
  return 2.5;
}

// The HUD cyan, used when the game's part carries no colour (white or transparent tint).
public func UTPPRetFallbackColour() -> HDRColor {
  return new HDRColor(0.3686, 0.9647, 1.0, 1.0);
}

public func UTPPRetIsUncoloured(c: HDRColor) -> Bool {
  return c.Alpha <= 0.01 || c.Red > 0.95 && c.Green > 0.95 && c.Blue > 0.95;
}

public func UTPPRetColourText(c: HDRColor) -> String {
  return FloatToStringPrec(c.Red, 2) + "," + FloatToStringPrec(c.Green, 2) + "," + FloatToStringPrec(c.Blue, 2);
}

public func UTPPRetNow() -> Float {
  return EngineTime.ToFloat(GameInstance.GetSimTime(GetGameInstance()));
}

// The game's hit indicator filter: a real, damaging, non-missed hit by the player.
public func UTPPRetCountsAsHit(d: DamageInfo, player: wref<GameObject>) -> Bool {
  if d.entityHit == null || d.instigator != player {
    return false;
  }
  if d.damageValue == 0.0 || Equals(d.hitType, gameuiHitType.Miss) {
    return false;
  }
  if IsDefined(d.userData) {
    let i: Int32 = 0;
    while i < ArraySize(d.userData.flags) {
      let f: hitFlag = d.userData.flags[i].flag;
      if Equals(f, hitFlag.ImmortalTarget) || Equals(f, hitFlag.DealNoDamage) || Equals(f, hitFlag.DontShowDamageFloater) {
        return false;
      }
      i += 1;
    }
  }
  return true;
}

// ---------------------------------------------------------------- controller fields

@addField(gameuiCrosshairBaseGameController)
public let utppRetKind: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppReticle: ref<inkCanvas>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetRing: ref<inkCanvas>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDot: ref<inkCanvas>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetHitX: ref<inkCanvas>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetKillX: ref<inkCanvas>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetArcs: array<ref<inkRectangle>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetArcShadows: array<ref<inkRectangle>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetR: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetBuildTried: Bool;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDotKind: String;

@addField(gameuiCrosshairBaseGameController)
public let utppRetOn: Bool;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDamageBB: wref<IBlackboard>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDamageCb: ref<CallbackHandle>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetKillCb: ref<CallbackHandle>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetWeaponBB: wref<IBlackboard>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShootCb: ref<CallbackHandle>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShots: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetLastShot: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetKillUntil: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetGateLogged: Bool;

@addField(gameuiCrosshairBaseGameController)
public let utppRetIntroSeen: Bool;

// the game widgets hidden by us (restored when the setting goes off) and their names for the log
@addField(gameuiCrosshairBaseGameController)
public let utppRetHidden: array<wref<inkWidget>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetHiddenNames: String;

// kept in the tree (1x1, transparent) so the atlas can load; the dot switches to it once its "dot" part resolves
@addField(gameuiCrosshairBaseGameController)
public let utppRetAtlasCheck: ref<inkImage>;

// widgets coloured by the crosshair style (arcs, dot core); the ring's are replaced on every ring rebuild
@addField(gameuiCrosshairBaseGameController)
public let utppRetStyled: array<wref<inkWidget>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDotCore: wref<inkWidget>;

// shadows (ring chords + dot) with their normal opacity, and the x strokes: recoloured by UTPPRetApplyColour
@addField(gameuiCrosshairBaseGameController)
public let utppRetRingShadows: array<wref<inkWidget>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetRingShadowOpacity: array<Float>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDotShadow: wref<inkWidget>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetXStrokes: array<wref<inkWidget>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetGuiState: CName;

@addField(gameuiCrosshairBaseGameController)
public let utppRetColour: HDRColor;

@addField(gameuiCrosshairBaseGameController)
public let utppRetColourSrc: String;

@addField(gameuiCrosshairBaseGameController)
public let utppRetBindFails: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetArcModeBuilt: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetRingWidgets: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetRingBuilds: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetChordStepBuilt: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetStateSeen: Bool;

// ---------------------------------------------------------------- widget build

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetAttach(w: ref<inkWidget>, parent: ref<inkCompoundWidget>) -> Void {
  w.SetAnchor(inkEAnchor.Centered);
  w.SetAnchorPoint(0.5, 0.5);
  w.SetRenderTransformPivot(0.5, 0.5);
  w.Reparent(parent);
  // keep the creation order as the draw order (later = in front)
  parent.ReorderChild(w, parent.GetNumChildren() - 1);
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetCanvas(parent: ref<inkCompoundWidget>, name: CName) -> ref<inkCanvas> {
  let c: ref<inkCanvas> = new inkCanvas();
  c.SetName(name);
  c.SetSize(UTPPRetBox(), UTPPRetBox());
  this.UTPPRetAttach(c, parent);
  return c;
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetRect(parent: ref<inkCompoundWidget>, w: Float, h: Float, white: Bool, opacity: Float)
    -> ref<inkRectangle> {
  let r: ref<inkRectangle> = new inkRectangle();
  r.SetSize(w, h);
  if white {
    r.SetTintColor(new HDRColor(1.0, 1.0, 1.0, 1.0));
  } else {
    r.SetTintColor(new HDRColor(0.0, 0.0, 0.0, 1.0));
  }
  r.SetOpacity(opacity);
  this.UTPPRetAttach(r, parent);
  return r;
}

// A container rotated by angle (degrees) around the centre; its child sits on the +X axis.
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetSpoke(parent: ref<inkCompoundWidget>, angle: Float) -> ref<inkCanvas> {
  let c: ref<inkCanvas> = this.UTPPRetCanvas(parent, n"spoke");
  c.SetRotation(angle);
  return c;
}

// The game's own coloured part of this controller (its tint is the crosshair colour of the current state).
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetColourSource() -> wref<inkWidget> {
  let basic: ref<CrosshairGameController_Basic> = this as CrosshairGameController_Basic;
  if IsDefined(basic) {
    return basic.UTPPRetBasicColourPart();
  }
  let simple: ref<CrosshairGameController_Simple> = this as CrosshairGameController_Simple;
  if IsDefined(simple) {
    return simple.UTPPRetSimpleColourPart();
  }
  return null;
}

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetSampleColour() -> Void {
  let src: wref<inkWidget> = this.UTPPRetColourSource();
  if IsDefined(src) {
    let c: HDRColor = src.GetTintColor();
    if !UTPPRetIsUncoloured(c) {
      this.utppRetColour = new HDRColor(c.Red, c.Green, c.Blue, 1.0);
      this.utppRetColourSrc = "part";
      return;
    }
  }
  this.utppRetColour = UTPPRetFallbackColour();
  this.utppRetColourSrc = "fallback";
}

// Colours one of our widgets like the game's crosshair: the sampled tint now, then the style binding (wins once it resolves)
// and the current GUI state.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetStyle(w: ref<inkWidget>, prop: CName) -> Void {
  let mode: Int32 = UTPPRetColourModeOf(GetGameInstance());
  if mode != 0 {
    w.SetTintColor(UTPPRetFixedColour(mode));
    ArrayPush(this.utppRetStyled, w);
    return;
  }
  w.SetTintColor(this.utppRetColour);
  w.SetStyle(r"base\\gameplay\\gui\\widgets\\crosshair\\crosshair_colors.inkstyle");
  if !w.BindProperty(n"tintColor", prop) {
    this.utppRetBindFails += 1;
  }
  if NotEquals(this.utppRetGuiState, n"") {
    w.SetState(this.utppRetGuiState);
  }
  ArrayPush(this.utppRetStyled, w);
}

// GUI state of the game's crosshair (Civilian / Hostile / Friendly / Dead): passed on to our styled widgets.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetGuiState(state: CName) -> Void {
  this.utppRetGuiState = state;
  if !IsDefined(this.utppReticle) || UTPPRetColourModeOf(GetGameInstance()) != 0 {
    return;
  }
  let sampleOnly: Bool = this.utppRetBindFails > 0;
  if sampleOnly {
    this.UTPPRetSampleColour();
  }
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetStyled) {
    if IsDefined(this.utppRetStyled[i]) {
      this.utppRetStyled[i].SetState(state);
      if sampleOnly {
        this.utppRetStyled[i].SetTintColor(this.utppRetColour);
      }
    }
    i += 1;
  }
}

// Shadow tint / opacity for the current colour: black at its normal opacity, white at 0.4 under a black reticle.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetShadowLook(w: ref<inkWidget>, opacity: Float) -> Void {
  if UTPPRetColourModeOf(GetGameInstance()) == 4 {
    w.SetTintColor(new HDRColor(1.0, 1.0, 1.0, 1.0));
    w.SetOpacity(0.40);
  } else {
    w.SetTintColor(new HDRColor(0.0, 0.0, 0.0, 1.0));
    w.SetOpacity(opacity);
  }
}

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetRingShadow(w: ref<inkWidget>, opacity: Float) -> Void {
  this.UTPPRetShadowLook(w, opacity);
  ArrayPush(this.utppRetRingShadows, w);
  ArrayPush(this.utppRetRingShadowOpacity, opacity);
}

// x strokes: black, white under a black reticle.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetXLook(w: ref<inkWidget>) -> Void {
  if UTPPRetColourModeOf(GetGameInstance()) == 4 {
    w.SetTintColor(new HDRColor(1.0, 1.0, 1.0, 1.0));
  } else {
    w.SetTintColor(new HDRColor(0.0, 0.0, 0.0, 1.0));
  }
}

// Applies the crosshair.color setting to the existing widgets (live switch): fixed modes unbind the style and set the tint;
// Standard re-binds the style and the current GUI state. Shadows and x follow.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetApplyColour() -> Void {
  if !IsDefined(this.utppReticle) {
    return;
  }
  let mode: Int32 = UTPPRetColourModeOf(GetGameInstance());
  if mode == 0 {
    this.UTPPRetSampleColour();
  }
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetStyled) {
    let w: wref<inkWidget> = this.utppRetStyled[i];
    if IsDefined(w) {
      if mode != 0 {
        w.UnbindProperty(n"tintColor");
        w.SetTintColor(UTPPRetFixedColour(mode));
      } else {
        let prop: CName = n"Crosshair.mainColor";
        if w == this.utppRetDotCore {
          prop = n"Crosshair.dotColor";
        }
        w.SetTintColor(this.utppRetColour);
        w.SetStyle(r"base\\gameplay\\gui\\widgets\\crosshair\\crosshair_colors.inkstyle");
        if !w.BindProperty(n"tintColor", prop) {
          this.utppRetBindFails += 1;
        }
        if NotEquals(this.utppRetGuiState, n"") {
          w.SetState(this.utppRetGuiState);
        }
      }
    }
    i += 1;
  }
  i = 0;
  while i < ArraySize(this.utppRetRingShadows) {
    if IsDefined(this.utppRetRingShadows[i]) {
      this.UTPPRetShadowLook(this.utppRetRingShadows[i], this.utppRetRingShadowOpacity[i]);
    }
    i += 1;
  }
  if IsDefined(this.utppRetDotShadow) {
    this.UTPPRetShadowLook(this.utppRetDotShadow, 0.45);
  }
  i = 0;
  while i < ArraySize(this.utppRetXStrokes) {
    if IsDefined(this.utppRetXStrokes[i]) {
      this.UTPPRetXLook(this.utppRetXStrokes[i]);
    }
    i += 1;
  }
}

// ---------------------------------------------------------------- ring
// 4 arcs of 60 degrees (gaps on the diagonals), each made of 20 chords of 3 degrees in containers rotated around the centre,
// with a thicker black chord behind each as the shadow. The chords are resized and moved, never the ring, so the line
// thickness stays the same at any radius.
// Default (mode 6): each chord is the game's "Rectangle" shape of main_shapes.inkshapecollection (Fill variant: antialiased
// edges, tinted by the style), scaled from the shape's bounds (read back with GetVertexList right after ChangeShape; 100 x 100
// in game, also the assumed value when they cannot be read). Fallback (mode 2, console): plain inkRectangle chords.
// In game, an inkShape without a shape resource draws nothing, and a resource-backed inkShape draws its preset
// geometry at the preset's own size centred on the widget (a vertex list set from script is not used), so arcs are built
// from scaled preset chords rather than polylines.
// Shot kick (hip fire and aiming): every shot adds 8 units to the radius (up to 24 in total from shots), and a rise of the game's
// spread bloom sets the kick to at least 2.5 * multiplier * excess (UTPPRetBloomKickTarget, max 120). The kick is a function
// of time (UTPPRetKickAt: k0 * exp(-16 * dt), zero after 0.25 s), so no update loop can leave it stuck; the shrink back to the
// resting radius is an ink animation of the chords' translation (and of the shape chords' scale), played by the engine every
// frame. A spread change during a kick restarts that animation from the current kick to the resting radius.
// Hip fire has the shot kick only (capped at 24); UTPPRetBloomKickTarget returns 0 without the aim flag.
// Diagnostics: "[utpp_reticle] r via=<shot|spread|late> R= base= kick= spread= shots=" at most every 0.25 s while a kick is
// active or R differs from the resting radius, max 40 lines per game session; "late" is a delay callback 0.5 s after a shot.

public class UTPPRetLateDiagCallback extends DelayCallback {
  public let ctrl: wref<gameuiCrosshairBaseGameController>;

  public func Call() -> Void {
    if IsDefined(this.ctrl) {
      this.ctrl.UTPPRetDiag("late", true);
    }
  }
}

public func UTPPRetMainShapes() -> ResRef {
  return r"base\\gameplay\\gui\\common\\main_shapes.inkshapecollection";
}

public func UTPPRetVecText(v: Vector2) -> String {
  return FloatToStringPrec(v.X, 1) + "," + FloatToStringPrec(v.Y, 1);
}

public func UTPPRetKickStep() -> Float {
  return 8.0;
}

public func UTPPRetKickMax() -> Float {
  return 24.0;
}

// per second; exp(-16 * 0.2) = 4 % of the kick left after 0.2 s. A rate of 12 (5 % after 0.25 s, over at 0.3 s) with
// the game's bloom (about 2.5 s to fade) driving the radius, which made the return slow.
public func UTPPRetKickDecay() -> Float {
  return 16.0;
}

// seconds after which a kick is over (exp(-16 * 0.25) = 1.8 %); the ink animation of the return lasts this long
public func UTPPRetKickDuration() -> Float {
  return 0.25;
}

// largest kick a spread bloom may set (ink units)
public func UTPPRetBloomMax() -> Float {
  return 120.0;
}

// The kick left at time now from k0 set at t0. Pure function of its arguments.
public func UTPPRetKickAt(k0: Float, t0: Float, now: Float) -> Float {
  let dt: Float = now - t0;
  if dt < 0.0 {
    dt = 0.0;
  }
  if dt >= UTPPRetKickDuration() || k0 <= 0.0 {
    return 0.0;
  }
  return k0 * PowF(2.7182818, -UTPPRetKickDecay() * dt);
}

// The game's bullet spread does not return to 0 after the first shot (in game it stays near 30 at rest), so the ring uses
// only the spread above the weapon's own floor: the smallest non-zero spread seen since the equip. Spread 0 (not set yet)
// counts as no excess. Pure functions of their arguments.
public func UTPPRetSpreadFloorNext(spreadMax: Float, floor: Float) -> Float {
  if spreadMax <= 0.001 {
    return floor;
  }
  if floor < 0.0 {
    return spreadMax;
  }
  return MinF(floor, spreadMax);
}

public func UTPPRetSpreadExcess(spreadMax: Float, floor: Float) -> Float {
  if spreadMax <= 0.001 {
    return 0.0;
  }
  return MaxF(0.0, spreadMax - UTPPRetSpreadFloorNext(spreadMax, floor));
}

// The kick after one more shot at time now: the remaining kick plus one step, capped. Pure function of its arguments.
// A kick above the cap (set by a spread bloom) is not cut down by a shot.
public func UTPPRetKickAfterShot(k0: Float, t0: Float, now: Float) -> Float {
  let left: Float = UTPPRetKickAt(k0, t0, now);
  return MinF(left + UTPPRetKickStep(), MaxF(UTPPRetKickMax(), left));
}

// The kick after a shot, the same in hip fire and while aiming: one step per shot on top of what is left, capped at
// UTPPRetKickMax (24), gone 0.25 s after the last shot. Hip fire takes no spread bloom (UTPPRetBloomKickTarget), so the ring
// grows once per shot and comes back instead of growing with the game's accumulated spread.
public func UTPPRetShotKick(ads: Bool, k0: Float, t0: Float, now: Float) -> Float {
  if !ads {
    return MinF(UTPPRetKickAt(k0, t0, now) + UTPPRetKickStep(), UTPPRetKickMax());
  }
  return UTPPRetKickAfterShot(k0, t0, now);
}

// The kick wanted after a spread update: 0 in hip fire; while aiming the kick left (kickNow), raised to the bloom
// 2.5 * mult * excess (at most UTPPRetBloomMax) only when the game's excess rose since the previous update (lastExcess), so
// the game's slow fade (excess falling) never restarts it.
public func UTPPRetBloomKickTarget(ads: Bool, excess: Float, lastExcess: Float, mult: Float, kickNow: Float) -> Float {
  if !ads {
    return 0.0;
  }
  if excess <= lastExcess + 0.05 {
    return kickNow;
  }
  return MaxF(kickNow, MinF(UTPPRetSpreadScale() * mult * excess, UTPPRetBloomMax()));
}

// Offline-style self check of the kick arithmetic (console: print(Game.UTPPCrosshairKickSelfTest())): "ok" or the failures.
public func UTPPCrosshairKickSelfTest() -> String {
  let fails: String = "";
  if AbsF(UTPPRetKickAt(4.0, 10.0, 10.0) - 4.0) > 0.001 {
    fails += " at0";
  }
  let k: Float = UTPPRetKickAt(16.0, 10.0, 10.2);
  if k > 16.0 * 0.05 || k < 16.0 * 0.03 {
    fails += " at0.2=" + FloatToStringPrec(k, 3);
  }
  if UTPPRetKickAt(16.0, 10.0, 10.3) != 0.0 || UTPPRetKickAt(16.0, 10.0, 9.0) != 16.0 {
    fails += " edge";
  }
  // aiming: any kick up to the bloom maximum is over 0.25 s after it was set (recovery target 0.25 s)
  if UTPPRetKickAt(UTPPRetBloomMax(), 10.0, 10.25) != 0.0 || UTPPRetKickAt(24.0, 10.0, 10.25) != 0.0
      || UTPPRetKickDuration() > 0.25 {
    fails += " adsRecover";
  }
  // hip fire: one shot kicks once (one step); 20 shots at 0.1 s with the worst bloom never exceed the cap, and the ring is
  // back at rest 0.25 s after the last shot
  if AbsF(UTPPRetShotKick(false, 0.0, 0.0, 0.0) - UTPPRetKickStep()) > 0.01 {
    fails += " hipOne";
  }
  let hk: Float = 0.0;
  let ht: Float = 0.0;
  let hmax: Float = 0.0;
  let hi: Int32 = 0;
  while hi < 20 {
    hk = UTPPRetShotKick(false, hk, ht, 0.1 * Cast<Float>(hi));
    ht = 0.1 * Cast<Float>(hi);
    hk = MaxF(hk, UTPPRetBloomKickTarget(false, 40.0, 0.0, 1.0, hk));
    hmax = MaxF(hmax, hk);
    hi += 1;
  }
  if hmax > UTPPRetKickMax() + 0.01 || UTPPRetKickAt(hk, ht, ht + 0.25) != 0.0 {
    fails += " hipGrow=" + FloatToStringPrec(hmax, 2);
  }
  // aiming: a bloom rise sets the kick (2.5 * 10 = 25), a falling or equal excess leaves it, a shot does not cut it down
  if AbsF(UTPPRetBloomKickTarget(true, 10.0, 0.0, 1.0, 3.0) - 25.0) > 0.01
      || UTPPRetBloomKickTarget(true, 9.0, 10.0, 1.0, 3.0) != 3.0
      || UTPPRetBloomKickTarget(true, 100.0, 0.0, 1.0, 0.0) != UTPPRetBloomMax()
      || AbsF(UTPPRetKickAfterShot(40.0, 5.0, 5.0) - 40.0) > 0.01 {
    fails += " bloom";
  }
  let s: Float = 0.0;
  let t: Float = 0.0;
  let i: Int32 = 0;
  while i < 10 {
    s = UTPPRetKickAfterShot(s, t, 0.01 * Cast<Float>(i));
    t = 0.01 * Cast<Float>(i);
    i += 1;
  }
  if s != UTPPRetKickMax() {
    fails += " cap=" + FloatToStringPrec(s, 2);
  }
  if UTPPRetKickAt(s, t, t + 1.0) != 0.0 {
    fails += " settle";
  }
  // spread floor: 0 -> none, 30.4 -> floor 30.4 (excess 0), 34.2 -> excess 3.8, 29.8 -> new floor (excess 0)
  let f: Float = -1.0;
  if UTPPRetSpreadExcess(0.0, f) != 0.0 {
    fails += " ex0";
  }
  f = UTPPRetSpreadFloorNext(0.0, f);
  if UTPPRetSpreadExcess(30.4, f) != 0.0 {
    fails += " exFirst";
  }
  f = UTPPRetSpreadFloorNext(30.4, f);
  if AbsF(UTPPRetSpreadExcess(34.2, f) - 3.8) > 0.01 {
    fails += " exUp";
  }
  f = UTPPRetSpreadFloorNext(34.2, f);
  if UTPPRetSpreadExcess(29.8, f) != 0.0 || AbsF(UTPPRetSpreadFloorNext(29.8, f) - 29.8) > 0.001 {
    fails += " exDown";
  }
  if Equals(fails, "") {
    return "ok";
  }
  return "fail:" + fails;
}

@addField(gameuiCrosshairBaseGameController)
public let utppRetShapes: array<ref<inkShape>>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShapeShadow: array<Bool>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShapeThick: array<Float>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetPresetSize: Vector2;

@addField(gameuiCrosshairBaseGameController)
public let utppRetPresetRead: Bool;

@addField(gameuiCrosshairBaseGameController)
public let utppRetBaseR: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetKick0: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetKickT0: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetSpread: Vector2;

@addField(gameuiCrosshairBaseGameController)
public let utppRetRestR: Float;

@addField(gameuiCrosshairBaseGameController)
public let utppRetDiagT: Float;

// smallest non-zero spread seen since the equip (-1 = none yet): the weapon's own resting spread
@addField(gameuiCrosshairBaseGameController)
public let utppRetSpreadFloor: Float;

// the game's spread excess at the previous spread update (a rise of it while aiming sets the bloom kick)
@addField(gameuiCrosshairBaseGameController)
public let utppRetExcessLast: Float;

// per-shot signal from the weapon's magazine count (the UI ShootEvent fired only once per equip in game)
@addField(gameuiCrosshairBaseGameController)
public let utppRetAmmoBB: wref<IBlackboard>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetAmmoCb: ref<CallbackHandle>;

@addField(gameuiCrosshairBaseGameController)
public let utppRetAmmoLast: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShotsEvent: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetShotsAmmo: Int32;

@addField(gameuiCrosshairBaseGameController)
public let utppRetLastKickT: Float;

// Mode 6: one chord of the game's Rectangle shape (Fill) in a rotated container; size / scale set by UTPPRetLayoutChords.
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetShapeChord(parent: ref<inkCompoundWidget>, theta: Float, thick: Float, shadow: Bool)
    -> ref<inkShape> {
  let spoke: ref<inkCanvas> = this.UTPPRetSpoke(parent, theta);
  let shape: ref<inkShape> = new inkShape();
  shape.SetResource(UTPPRetMainShapes());
  shape.ChangeShape(n"Rectangle");
  shape.SetVariant(inkEShapeVariant.Fill);
  shape.SetFillOpacity(1.0);
  shape.SetBorderOpacity(0.0);
  shape.SetKeepInBounds(false);
  this.UTPPRetAttach(shape, spoke);
  if shadow {
    this.UTPPRetRingShadow(shape, 0.40);
  } else {
    this.UTPPRetStyle(shape, n"Crosshair.mainColor");
  }
  if !this.utppRetPresetRead {
    let pts: array<Vector2> = shape.GetVertexList();
    if ArraySize(pts) > 0 {
      let lo: Vector2 = UTPPRetMinOf(pts);
      let hi: Vector2 = UTPPRetMaxOf(pts);
      if hi.X - lo.X > 0.01 && hi.Y - lo.Y > 0.01 {
        this.utppRetPresetSize = new Vector2(hi.X - lo.X, hi.Y - lo.Y);
        this.utppRetPresetRead = true;
      }
    }
  }
  ArrayPush(this.utppRetShapes, shape);
  ArrayPush(this.utppRetShapeShadow, shadow);
  ArrayPush(this.utppRetShapeThick, thick);
  return shape;
}

public func UTPPRetMinOf(pts: array<Vector2>) -> Vector2 {
  let m: Vector2 = new Vector2(100000.0, 100000.0);
  let k: Int32 = 0;
  while k < ArraySize(pts) {
    m.X = MinF(m.X, pts[k].X);
    m.Y = MinF(m.Y, pts[k].Y);
    k += 1;
  }
  return m;
}

public func UTPPRetMaxOf(pts: array<Vector2>) -> Vector2 {
  let m: Vector2 = new Vector2(-100000.0, -100000.0);
  let k: Int32 = 0;
  while k < ArraySize(pts) {
    m.X = MaxF(m.X, pts[k].X);
    m.Y = MaxF(m.Y, pts[k].Y);
    k += 1;
  }
  return m;
}

// (Re)creates the ring for the current radius and mode: shadows first, arcs in front. Logs the build when the mode changes.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetBuildRing() -> Void {
  if !IsDefined(this.utppRetRing) {
    return;
  }
  let mode: Int32 = UTPPRetArcModeOf(GetGameInstance());
  let modeChanged: Bool = mode != this.utppRetArcModeBuilt;
  this.utppRetArcModeBuilt = mode;
  this.utppRetRingBuilds += 1;
  this.utppRetRing.RemoveAllChildren();
  ArrayClear(this.utppRetArcs);
  ArrayClear(this.utppRetArcShadows);
  ArrayClear(this.utppRetStyled);
  ArrayClear(this.utppRetRingShadows);
  ArrayClear(this.utppRetRingShadowOpacity);
  ArrayClear(this.utppRetShapes);
  ArrayClear(this.utppRetShapeShadow);
  ArrayClear(this.utppRetShapeThick);
  if IsDefined(this.utppRetDotCore) {
    ArrayPush(this.utppRetStyled, this.utppRetDotCore);
  }
  let r: Float = MaxF(this.utppRetR, 16.0);
  let t: Float = UTPPRetArcThickness();
  let step: Float = UTPPRetChordStep(mode);
  this.utppRetChordStepBuilt = step;
  let n: Int32 = RoundF(60.0 / step);
  let widgets: Int32 = 0;
  let pass: Int32 = 0;
  while pass < 2 {
    let thick: Float = t;
    if pass == 0 {
      thick = t + 3.0;
    }
    let a: Int32 = 0;
    while a < 4 {
      let s: Int32 = 0;
      while s < n {
        let theta: Float = 90.0 * Cast<Float>(a) - 30.0 + (Cast<Float>(s) + 0.5) * step;
        if mode == 6 {
          this.UTPPRetShapeChord(this.utppRetRing, theta, thick, pass == 0);
        } else {
          let spoke: ref<inkCanvas> = this.UTPPRetSpoke(this.utppRetRing, theta);
          if pass == 0 {
            let sh: ref<inkRectangle> = this.UTPPRetRect(spoke, 1.0, 1.0, false, 0.40);
            this.UTPPRetRingShadow(sh, 0.40);
            ArrayPush(this.utppRetArcShadows, sh);
          } else {
            let chord: ref<inkRectangle> = this.UTPPRetRect(spoke, 1.0, 1.0, true, 1.0);
            this.UTPPRetStyle(chord, n"Crosshair.mainColor");
            ArrayPush(this.utppRetArcs, chord);
          }
        }
        widgets += 2;
        s += 1;
      }
      a += 1;
    }
    pass += 1;
  }
  this.utppRetRingWidgets = widgets;
  this.UTPPRetLayoutChords(r);
  if modeChanged {
    let preset: String = "-";
    if mode == 6 {
      preset = "assumed";
      if this.utppRetPresetRead {
        preset = "read";
      }
      preset += " size=" + UTPPRetVecText(this.UTPPRetPresetSizeOrDefault());
    }
    UTPPLogLine("[utpp_reticle] ring arcs=" + UTPPRetArcModeName(mode) + " widgets=" + IntToString(widgets) + " R="
      + FloatToStringPrec(r, 1) + " preset=" + preset + " class=" + NameToString(this.GetClassName()));
  }
}

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetPresetSizeOrDefault() -> Vector2 {
  if this.utppRetPresetRead {
    return this.utppRetPresetSize;
  }
  return new Vector2(100.0, 100.0);
}

// Resizes / moves the chords of either mode to radius r (the long side runs along the tangent, local Y of a spoke).
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetLayoutChords(r: Float) -> Void {
  let t: Float = UTPPRetArcThickness();
  let len: Float = 2.0 * r * SinF(Deg2Rad(0.5 * this.utppRetChordStepBuilt)) + 1.0;
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetArcs) {
    this.utppRetArcs[i].SetSize(t, len);
    this.utppRetArcs[i].SetTranslation(r, 0.0);
    i += 1;
  }
  i = 0;
  while i < ArraySize(this.utppRetArcShadows) {
    this.utppRetArcShadows[i].SetSize(t + 3.0, len + 3.0);
    this.utppRetArcShadows[i].SetTranslation(r, 0.0);
    i += 1;
  }
  let size: Vector2 = this.UTPPRetPresetSizeOrDefault();
  i = 0;
  while i < ArraySize(this.utppRetShapes) {
    let l: Float = len;
    if this.utppRetShapeShadow[i] {
      l += 3.0;
    }
    this.utppRetShapes[i].SetSize(size.X, size.Y);
    this.utppRetShapes[i].SetScale(new Vector2(this.utppRetShapeThick[i] / size.X, l / size.Y));
    this.utppRetShapes[i].SetTranslation(r, 0.0);
    i += 1;
  }
}

// Chord length for radius r (3 degree chords plus 1 unit of overlap).
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetChordLen(r: Float) -> Float {
  return 2.0 * r * SinF(Deg2Rad(0.5 * this.utppRetChordStepBuilt)) + 1.0;
}

// Plays the shrink from radius rFrom to rTo over duration on every chord (static layout set to rTo first, so the end state
// is right even without the animation). One shared animation definition per chord kind.
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetAnimateRadius(rFrom: Float, rTo: Float, duration: Float) -> Void {
  this.UTPPRetLayoutChords(rTo);
  if duration <= 0.01 || AbsF(rFrom - rTo) < 0.05 {
    return;
  }
  let t: Float = UTPPRetArcThickness();
  let lenFrom: Float = this.UTPPRetChordLen(rFrom);
  let lenTo: Float = this.UTPPRetChordLen(rTo);
  let move: ref<inkAnimDef> = new inkAnimDef();
  move.AddInterpolator(this.UTPPRetMoveInterp(rFrom, rTo, duration));
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetArcs) {
    this.utppRetArcs[i].StopAllAnimations();
    this.utppRetArcs[i].PlayAnimation(move);
    i += 1;
  }
  i = 0;
  while i < ArraySize(this.utppRetArcShadows) {
    this.utppRetArcShadows[i].StopAllAnimations();
    this.utppRetArcShadows[i].PlayAnimation(move);
    i += 1;
  }
  if ArraySize(this.utppRetShapes) == 0 {
    return;
  }
  let size: Vector2 = this.UTPPRetPresetSizeOrDefault();
  let arcDef: ref<inkAnimDef> = new inkAnimDef();
  arcDef.AddInterpolator(this.UTPPRetMoveInterp(rFrom, rTo, duration));
  arcDef.AddInterpolator(this.UTPPRetScaleInterp(new Vector2(t / size.X, lenFrom / size.Y),
    new Vector2(t / size.X, lenTo / size.Y), duration));
  let shadowDef: ref<inkAnimDef> = new inkAnimDef();
  shadowDef.AddInterpolator(this.UTPPRetMoveInterp(rFrom, rTo, duration));
  shadowDef.AddInterpolator(this.UTPPRetScaleInterp(new Vector2((t + 3.0) / size.X, (lenFrom + 3.0) / size.Y),
    new Vector2((t + 3.0) / size.X, (lenTo + 3.0) / size.Y), duration));
  i = 0;
  while i < ArraySize(this.utppRetShapes) {
    this.utppRetShapes[i].StopAllAnimations();
    if this.utppRetShapeShadow[i] {
      this.utppRetShapes[i].PlayAnimation(shadowDef);
    } else {
      this.utppRetShapes[i].PlayAnimation(arcDef);
    }
    i += 1;
  }
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetMoveInterp(rFrom: Float, rTo: Float, duration: Float) -> ref<inkAnimTranslation> {
  let m: ref<inkAnimTranslation> = new inkAnimTranslation();
  m.SetStartTranslation(new Vector2(rFrom, 0.0));
  m.SetEndTranslation(new Vector2(rTo, 0.0));
  m.SetDuration(duration);
  m.SetType(inkanimInterpolationType.Exponential);
  m.SetMode(inkanimInterpolationMode.EasyOut);
  return m;
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetScaleInterp(from: Vector2, to: Vector2, duration: Float) -> ref<inkAnimScale> {
  let sc: ref<inkAnimScale> = new inkAnimScale();
  sc.SetStartScale(from);
  sc.SetEndScale(to);
  sc.SetDuration(duration);
  sc.SetType(inkanimInterpolationType.Exponential);
  sc.SetMode(inkanimInterpolationMode.EasyOut);
  return sc;
}

// Stops any running radius animation (the static layout that follows is then the visible state).
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetStopRadiusAnims() -> Void {
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetArcs) {
    this.utppRetArcs[i].StopAllAnimations();
    i += 1;
  }
  i = 0;
  while i < ArraySize(this.utppRetArcShadows) {
    this.utppRetArcShadows[i].StopAllAnimations();
    i += 1;
  }
  i = 0;
  while i < ArraySize(this.utppRetShapes) {
    this.utppRetShapes[i].StopAllAnimations();
    i += 1;
  }
}

// Applies the spread radius (utppRetBaseR) plus the current kick. During a kick the chords animate from base + kick to base
// over the kick's remaining time; otherwise they are set directly when R moves by 0.3 or more (or always when forced).
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetApplyRadius(force: Bool) -> Void {
  if !IsDefined(this.utppRetRing) {
    return;
  }
  let base: Float = ClampF(this.utppRetBaseR, 16.0, 360.0);
  let mode: Int32 = UTPPRetArcModeOf(GetGameInstance());
  if mode != this.utppRetArcModeBuilt {
    this.utppRetR = base;
    this.UTPPRetBuildRing();
    return;
  }
  let now: Float = UTPPRetNow();
  let kick: Float = UTPPRetKickAt(this.utppRetKick0, this.utppRetKickT0, now);
  if kick > 0.05 {
    this.utppRetR = base;
    this.UTPPRetAnimateRadius(base + kick, base, UTPPRetKickDuration() - (now - this.utppRetKickT0));
    this.UTPPRetDiag("spread", false);
    return;
  }
  if !force && this.utppRetR >= 0.0 && AbsF(base - this.utppRetR) < 0.3 {
    return;
  }
  this.utppRetR = base;
  this.UTPPRetStopRadiusAnims();
  this.UTPPRetLayoutChords(base);
  this.UTPPRetDiag("spread", false);
}

// True in the third-person aim state (the forced hip-fire visual of utpp_crosshair.reds), read fresh from the game's state;
// false in real hip fire and for every other state.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetIsAds() -> Bool {
  return this.UTPPWantsTppAimCrosshair();
}

// Called by utpp_crosshair.reds when the aim state flips: lays the ring out again at once, so an aiming-sized ring does not
// carry over into hip fire (a kick above the hip cap is dropped there) until the next spread update.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetAimChanged() -> Void {
  this.UTPPReticleLayout(this.utppRetSpread);
}

// One shot (hip fire or aiming): kick the ring outwards by one step (capped) and let it shrink back.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetKickShot() -> Void {
  if !IsDefined(this.utppRetRing) {
    return;
  }
  let now: Float = UTPPRetNow();
  let ads: Bool = this.UTPPRetIsAds();
  this.utppRetKick0 = UTPPRetShotKick(ads, this.utppRetKick0, this.utppRetKickT0, now);
  this.utppRetKickT0 = now;
  let base: Float = ClampF(this.utppRetBaseR, 16.0, 360.0);
  this.utppRetR = base;
  this.UTPPRetAnimateRadius(base + this.utppRetKick0, base, UTPPRetKickDuration());
  this.UTPPRetDiag("shot", true);
  let cb: ref<UTPPRetLateDiagCallback> = new UTPPRetLateDiagCallback();
  cb.ctrl = this;
  GameInstance.GetDelaySystem(GetGameInstance()).DelayCallback(cb, 0.5, false);
}

// Compact radius log: at most every 0.25 s (always for shots / late checks), only while a kick is active or R is away from
// the resting radius, max 40 lines per game session.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetDiag(via: String, always: Bool) -> Void {
  let now: Float = UTPPRetNow();
  let kick: Float = UTPPRetKickAt(this.utppRetKick0, this.utppRetKickT0, now);
  let away: Bool = AbsF(this.utppRetR - this.utppRetRestR) >= 0.5;
  if !always && (now - this.utppRetDiagT < 0.25 || (kick <= 0.05 && !away)) {
    return;
  }
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) || state.m_diagLines >= 40 {
    return;
  }
  state.m_diagLines += 1;
  this.utppRetDiagT = now;
  UTPPLogLine("[utpp_reticle] r via=" + via + " ads=" + UTPPRetFlag(this.UTPPRetIsAds()) + " R=" + FloatToStringPrec(this.utppRetR + kick, 1) + " base="
    + FloatToStringPrec(this.utppRetBaseR, 1) + " rest=" + FloatToStringPrec(this.utppRetRestR, 1) + " kick="
    + FloatToStringPrec(kick, 1) + " spread=" + UTPPRetVecText(this.utppRetSpread) + " shots="
    + IntToString(this.utppRetShots) + " (event " + IntToString(this.utppRetShotsEvent) + " ammo " + IntToString(this.utppRetShotsAmmo)
    + ") floor=" + FloatToStringPrec(this.utppRetSpreadFloor, 1) + " t=" + FloatToStringPrec(now, 2));
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetDotPiece(parent: ref<inkCompoundWidget>, atlas: Bool, size: Float, white: Bool, opacity: Float)
    -> ref<inkWidget> {
  if atlas {
    let img: ref<inkImage> = new inkImage();
    img.SetAtlasResource(r"base\\gameplay\\gui\\widgets\\crosshair\\master_crosshair.inkatlas");
    img.SetTexturePart(n"dot");
    img.SetSize(size, size);
    if white {
      img.SetTintColor(new HDRColor(1.0, 1.0, 1.0, 1.0));
    } else {
      img.SetTintColor(new HDRColor(0.0, 0.0, 0.0, 1.0));
    }
    img.SetOpacity(opacity);
    this.UTPPRetAttach(img, parent);
    return img;
  }
  return this.UTPPRetRect(parent, size, size, white, opacity);
}

// x marker: 4 strokes at 45 / 135 / 225 / 315 degrees, stroke centre at radius r.
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetCross(parent: ref<inkCompoundWidget>, name: CName, r: Float, len: Float, thick: Float)
    -> ref<inkCanvas> {
  let x: ref<inkCanvas> = this.UTPPRetCanvas(parent, name);
  let i: Int32 = 0;
  while i < 4 {
    let spoke: ref<inkCanvas> = this.UTPPRetSpoke(x, 45.0 + 90.0 * Cast<Float>(i));
    let stroke: ref<inkRectangle> = this.UTPPRetRect(spoke, len, thick, false, 1.0);
    stroke.SetTranslation(r, 0.0);
    this.UTPPRetXLook(stroke);
    ArrayPush(this.utppRetXStrokes, stroke);
    i += 1;
  }
  x.SetOpacity(0.0);
  return x;
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPReticleBuild() -> Void {
  if this.utppRetBuildTried {
    return;
  }
  this.utppRetBuildTried = true;
  this.utppRetDotKind = "none";
  let root: ref<inkCompoundWidget> = this.GetRootCompoundWidget();
  if !IsDefined(root) {
    return;
  }
  this.UTPPRetSampleColour();
  let top: ref<inkCanvas> = this.UTPPRetCanvas(root, n"utpp_reticle");
  // ring: filled by UTPPRetBuildRing (UTPPReticleLayout below)
  let ring: ref<inkCanvas> = this.UTPPRetCanvas(top, n"ring");
  // dot: the game's filtered "dot" atlas part when it resolves, else a small rectangle
  let dot: ref<inkCanvas> = this.UTPPRetCanvas(top, n"dot");
  this.utppRetDot = dot;
  let atlasCheck: ref<inkImage> = new inkImage();
  atlasCheck.SetAtlasResource(r"base\\gameplay\\gui\\widgets\\crosshair\\master_crosshair.inkatlas");
  atlasCheck.SetSize(1.0, 1.0);
  atlasCheck.SetOpacity(0.0);
  this.UTPPRetAttach(atlasCheck, top);
  this.utppRetAtlasCheck = atlasCheck;
  this.utppRetDotKind = "";
  this.UTPPRetDotUpgrade();
  // hit x (thin) and kill x (thick); both stay inside the smallest ring
  this.utppRetHitX = this.UTPPRetCross(top, n"hitX", 10.5, 9.0, 3.0);
  this.utppRetKillX = this.UTPPRetCross(top, n"killX", 12.0, 12.0, 5.0);
  this.utppReticle = top;
  this.utppRetRing = ring;
  this.utppRetArcModeBuilt = -1;
  // Basic: start from its parts' current opacity (its state calls drive both groups from here on); others: full opacity,
  // the root's visibility drives them
  let basic: ref<CrosshairGameController_Basic> = this as CrosshairGameController_Basic;
  if IsDefined(basic) {
    basic.UTPPRetBasicSyncOpacity();
  }
  this.utppRetR = -1.0;
  this.UTPPReticleLayout(new Vector2(0.0, 0.0));
  top.SetVisible(false);
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPReticleLayout(spread: Vector2) -> Void {
  if !IsDefined(this.utppRetRing) {
    return;
  }
  let minSpread: Float = 20.0;
  let mult: Float = 1.0;
  let basic: ref<CrosshairGameController_Basic> = this as CrosshairGameController_Basic;
  if IsDefined(basic) {
    minSpread = basic.m_horizontalMinSpread;
    mult = basic.m_gameplaySpreadMultiplier;
  }
  this.utppRetSpread = spread;
  this.utppRetRestR = ClampF(minSpread + UTPPRetRPad(), 16.0, 360.0);
  let excess: Float = UTPPRetSpreadExcess(MaxF(spread.X, spread.Y), this.utppRetSpreadFloor);
  this.utppRetSpreadFloor = UTPPRetSpreadFloorNext(MaxF(spread.X, spread.Y), this.utppRetSpreadFloor);
  // the base radius is always the resting one; the game's spread bloom reaches the ring only as a short kick while aiming
  this.utppRetBaseR = this.utppRetRestR;
  let now: Float = UTPPRetNow();
  let ads: Bool = this.UTPPRetIsAds();
  let kickNow: Float = UTPPRetKickAt(this.utppRetKick0, this.utppRetKickT0, now);
  // a bloom rise counts only right after a shot of this weapon (the spread listener is delayed and smoothed, hence 0.3 s):
  // movement, uneven spread values or the jump to the hip spread when aim is released must not kick the ring
  let afterShot: Bool = this.utppRetShots > 0 && now - this.utppRetLastShot <= 0.3;
  let target: Float = kickNow;
  if afterShot {
    target = UTPPRetBloomKickTarget(ads, excess, this.utppRetExcessLast, mult, kickNow);
  }
  this.utppRetExcessLast = excess;
  let force: Bool = false;
  if !ads {
    // hip fire keeps its own shot kicks (at most UTPPRetKickMax); a bigger aiming bloom kick does not carry over
    if kickNow > UTPPRetKickMax() + 0.05 {
      this.utppRetKick0 = 0.0;
      force = true;
    }
  } else if target > kickNow + 0.05 {
    this.utppRetKick0 = target;
    this.utppRetKickT0 = now;
  }
  this.UTPPRetApplyRadius(force);
}

// ---------------------------------------------------------------- ownership and fades

// Sets one of the game's parts visible / invisible; never the root or our own canvas.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetPartVisible(part: inkWidgetRef, visible: Bool) -> Void {
  if !inkWidgetRef.IsValid(part) {
    return;
  }
  let w: wref<inkWidget> = inkWidgetRef.Get(part);
  if w == this.m_rootWidget || w == this.utppReticle {
    return;
  }
  w.SetVisible(visible);
}

@addMethod(CrosshairGameController_Basic)
public final func UTPPRetBasicParts(visible: Bool) -> Void {
  this.UTPPRetPartVisible(this.m_leftPart, visible);
  this.UTPPRetPartVisible(this.m_rightPart, visible);
  this.UTPPRetPartVisible(this.m_upPart, visible);
  this.UTPPRetPartVisible(this.m_downPart, visible);
  this.UTPPRetPartVisible(this.m_centerPart, visible);
}

@addMethod(CrosshairGameController_Basic)
public final func UTPPRetBasicColourPart() -> wref<inkWidget> {
  return inkWidgetRef.Get(this.m_leftPart);
}

@addMethod(CrosshairGameController_Basic)
public final func UTPPRetBasicSyncOpacity() -> Void {
  if IsDefined(this.utppRetRing) {
    this.utppRetRing.SetOpacity(inkWidgetRef.GetOpacity(this.m_leftPart));
  }
  if IsDefined(this.utppRetDot) {
    this.utppRetDot.SetOpacity(inkWidgetRef.GetOpacity(this.m_centerPart));
  }
}

// Dot pieces: the atlas "dot" part (soft edge) once it resolves, else rectangles; switches rect -> atlas at most once.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetDotUpgrade() -> Void {
  if !IsDefined(this.utppRetDot) || Equals(this.utppRetDotKind, "atlas") {
    return;
  }
  let atlas: Bool = IsDefined(this.utppRetAtlasCheck) && this.utppRetAtlasCheck.IsTexturePartExist(n"dot");
  if Equals(this.utppRetDotKind, "rect") && !atlas {
    return;
  }
  this.utppRetDot.RemoveAllChildren();
  let core: ref<inkWidget>;
  let shadow: ref<inkWidget>;
  if atlas {
    this.utppRetDotKind = "atlas";
    shadow = this.UTPPRetDotPiece(this.utppRetDot, true, 11.0, false, 0.45);
    core = this.UTPPRetDotPiece(this.utppRetDot, true, 6.0, true, 1.0);
  } else {
    this.utppRetDotKind = "rect";
    shadow = this.UTPPRetDotPiece(this.utppRetDot, false, 9.0, false, 0.45);
    core = this.UTPPRetDotPiece(this.utppRetDot, false, 5.0, true, 1.0);
  }
  this.utppRetDotShadow = shadow;
  this.UTPPRetShadowLook(shadow, 0.45);
  if IsDefined(this.utppRetDotCore) {
    ArrayRemove(this.utppRetStyled, this.utppRetDotCore);
  }
  this.utppRetDotCore = core;
  this.UTPPRetStyle(core, n"Crosshair.dotColor");
}

// Every direct child of the root except our canvas and the dead-eye widget: hidden while on (only those that were visible,
// remembered with their names), restored when off. Catches game parts that the class has no reference to.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetRootChildren(visible: Bool) -> Void {
  if visible {
    let k: Int32 = 0;
    while k < ArraySize(this.utppRetHidden) {
      if IsDefined(this.utppRetHidden[k]) {
        this.utppRetHidden[k].SetVisible(true);
      }
      k += 1;
    }
    ArrayClear(this.utppRetHidden);
    return;
  }
  let root: ref<inkCompoundWidget> = this.GetRootCompoundWidget();
  if !IsDefined(root) {
    return;
  }
  let deadEye: wref<inkWidget> = inkWidgetRef.Get(this.m_deadEyeWidget);
  let names: String = "";
  let i: Int32 = 0;
  while i < root.GetNumChildren() {
    let w: wref<inkWidget> = root.GetWidgetByIndex(i);
    if IsDefined(w) && w != this.utppReticle && w != deadEye && w.IsVisible() {
      w.SetVisible(false);
      ArrayPush(this.utppRetHidden, w);
      names += NameToString(w.GetName()) + ",";
    }
    i += 1;
  }
  if NotEquals(names, "") {
    this.utppRetHiddenNames = names;
  }
}

@addMethod(CrosshairGameController_Simple)
public final func UTPPRetSimpleColourPart() -> wref<inkWidget> {
  return inkWidgetRef.Get(this.m_leftPart);
}

@addMethod(CrosshairGameController_Simple)
public final func UTPPRetSimpleParts(visible: Bool) -> Void {
  this.UTPPRetPartVisible(this.m_topPart, visible);
  this.UTPPRetPartVisible(this.m_bottomPart, visible);
  this.UTPPRetPartVisible(this.m_horiPart, visible);
  this.UTPPRetPartVisible(this.m_vertPart, visible);
  this.UTPPRetPartVisible(this.m_leftPart, visible);
  this.UTPPRetPartVisible(this.m_rightPart, visible);
  this.UTPPRetPartVisible(this.m_targetColorChange, visible);
}

// Applies the current setting to this controller: our canvas or the game's parts.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPReticleApply() -> Void {
  if !IsDefined(this.utppReticle) {
    return;
  }
  let on: Bool = UTPPCrosshairOn(GetGameInstance());
  this.utppRetOn = on;
  let basic: ref<CrosshairGameController_Basic> = this as CrosshairGameController_Basic;
  if IsDefined(basic) {
    basic.UTPPRetBasicParts(!on);
  }
  let simple: ref<CrosshairGameController_Simple> = this as CrosshairGameController_Simple;
  if IsDefined(simple) {
    simple.UTPPRetSimpleParts(!on);
    this.UTPPRetRootChildren(!on);
  }
  this.utppReticle.SetVisible(on);
  if this.IsActive() {
    // re-run the current state (and the third-person aim wrap with it) so both groups match it at once
    this.UpdateCrosshairState();
  }
}

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetFade(w: ref<inkWidget>, to: Float, duration: Float) -> Void {
  if !IsDefined(w) {
    return;
  }
  w.StopAllAnimations();
  if duration <= 0.0 {
    w.SetOpacity(to);
    return;
  }
  let anim: ref<inkAnimDef> = new inkAnimDef();
  let alpha: ref<inkAnimTransparency> = new inkAnimTransparency();
  alpha.SetStartTransparency(w.GetOpacity());
  alpha.SetEndTransparency(to);
  alpha.SetDuration(duration);
  alpha.SetType(inkanimInterpolationType.Linear);
  alpha.SetMode(inkanimInterpolationMode.EasyIn);
  anim.AddInterpolator(alpha);
  w.PlayAnimation(anim);
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPRetFlash(w: ref<inkWidget>, fade: Float, scaleFrom: Float, scaleTime: Float) -> Void {
  if !IsDefined(w) {
    return;
  }
  w.StopAllAnimations();
  w.SetOpacity(1.0);
  let anim: ref<inkAnimDef> = new inkAnimDef();
  let alpha: ref<inkAnimTransparency> = new inkAnimTransparency();
  alpha.SetStartTransparency(1.0);
  alpha.SetEndTransparency(0.0);
  alpha.SetDuration(fade);
  alpha.SetType(inkanimInterpolationType.Quadratic);
  alpha.SetMode(inkanimInterpolationMode.EasyOut);
  anim.AddInterpolator(alpha);
  let scale: ref<inkAnimScale> = new inkAnimScale();
  scale.SetStartScale(new Vector2(scaleFrom, scaleFrom));
  scale.SetEndScale(new Vector2(1.0, 1.0));
  scale.SetDuration(scaleTime);
  scale.SetType(inkanimInterpolationType.Quadratic);
  scale.SetMode(inkanimInterpolationMode.EasyOut);
  anim.AddInterpolator(scale);
  w.PlayAnimation(anim);
}

// ---------------------------------------------------------------- listeners

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPReticleUnlisten() -> Void {
  let defs: ref<AllBlackboardDefinitions> = GetAllBlackboardDefs();
  if IsDefined(this.utppRetDamageBB) {
    if IsDefined(this.utppRetDamageCb) {
      this.utppRetDamageBB.UnregisterListenerVariant(defs.UI_DamageInfo.DamageList, this.utppRetDamageCb);
    }
    if IsDefined(this.utppRetKillCb) {
      this.utppRetDamageBB.UnregisterListenerVariant(defs.UI_DamageInfo.KillList, this.utppRetKillCb);
    }
  }
  if IsDefined(this.utppRetWeaponBB) && IsDefined(this.utppRetShootCb) {
    this.utppRetWeaponBB.UnregisterListenerVariant(defs.UI_ActiveWeaponData.ShootEvent, this.utppRetShootCb);
  }
  if IsDefined(this.utppRetAmmoBB) && IsDefined(this.utppRetAmmoCb) {
    this.utppRetAmmoBB.UnregisterListenerUint(defs.Weapon.MagazineAmmoCount, this.utppRetAmmoCb);
  }
  this.utppRetAmmoCb = null;
  this.utppRetAmmoBB = null;
  this.utppRetDamageCb = null;
  this.utppRetKillCb = null;
  this.utppRetShootCb = null;
  this.utppRetDamageBB = null;
  this.utppRetWeaponBB = null;
}

@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPReticleListen() -> Void {
  this.UTPPReticleUnlisten();
  let defs: ref<AllBlackboardDefinitions> = GetAllBlackboardDefs();
  let dmg: ref<IBlackboard> = this.GetBlackboardSystem().Get(defs.UI_DamageInfo);
  if IsDefined(dmg) {
    this.utppRetDamageBB = dmg;
    this.utppRetDamageCb = dmg.RegisterListenerVariant(defs.UI_DamageInfo.DamageList, this, n"UTPPReticleOnDamage");
    this.utppRetKillCb = dmg.RegisterListenerVariant(defs.UI_DamageInfo.KillList, this, n"UTPPReticleOnKill");
  }
  let wbb: ref<IBlackboard> = this.GetBlackboardSystem().Get(defs.UI_ActiveWeaponData);
  if IsDefined(wbb) {
    this.utppRetWeaponBB = wbb;
    this.utppRetShootCb = wbb.RegisterListenerVariant(defs.UI_ActiveWeaponData.ShootEvent, this, n"UTPPReticleOnShoot");
  }
  this.utppRetAmmoLast = -1;
  let abb: ref<IBlackboard> = this.GetWeaponLocalBlackboard();
  if IsDefined(abb) {
    this.utppRetAmmoBB = abb;
    this.utppRetAmmoCb = abb.RegisterListenerUint(defs.Weapon.MagazineAmmoCount, this, n"UTPPReticleOnAmmo");
    this.utppRetAmmoLast = Cast<Int32>(abb.GetUint(defs.Weapon.MagazineAmmoCount));
  }
  this.utppRetShotsEvent = 0;
  this.utppRetShotsAmmo = 0;
  this.utppRetShots = 0;
  this.utppRetGateLogged = false;
  this.utppRetKillUntil = 0.0;
}

@addMethod(gameuiCrosshairBaseGameController)
protected cb func UTPPReticleOnShoot(value: Variant) -> Bool {
  this.utppRetShotsEvent += 1;
  this.UTPPRetShotSignal();
  return true;
}

// A drop of the magazine count is a shot (a reload raises it).
@addMethod(gameuiCrosshairBaseGameController)
protected cb func UTPPReticleOnAmmo(value: Uint32) -> Bool {
  let v: Int32 = Cast<Int32>(value);
  let last: Int32 = this.utppRetAmmoLast;
  this.utppRetAmmoLast = v;
  if last >= 0 && v < last {
    this.utppRetShotsAmmo += 1;
    this.UTPPRetShotSignal();
  }
  return true;
}

// One shot from either signal; two signals of the same shot (within 40 ms) kick once.
@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPRetShotSignal() -> Void {
  let now: Float = UTPPRetNow();
  this.utppRetLastShot = now;
  if this.utppRetShots > 0 && now - this.utppRetLastKickT < 0.04 {
    return;
  }
  this.utppRetLastKickT = now;
  this.utppRetShots += 1;
  this.UTPPRetKickShot();
}

@addMethod(gameuiCrosshairBaseGameController)
protected cb func UTPPReticleOnDamage(value: Variant) -> Bool {
  if !IsDefined(this.utppRetHitX) || !IsDefined(this.m_playerPuppet) {
    return false;
  }
  let now: Float = UTPPRetNow();
  if now < this.utppRetKillUntil {
    return false;
  }
  let list: array<DamageInfo> = FromVariant<array<DamageInfo>>(value);
  let found: Bool = false;
  let i: Int32 = 0;
  while i < ArraySize(list) && !found {
    found = UTPPRetCountsAsHit(list[i], this.m_playerPuppet);
    i += 1;
  }
  if !found {
    return false;
  }
  if this.utppRetShots == 0 {
    if !this.utppRetGateLogged {
      this.utppRetGateLogged = true;
      UTPPLogLine("[utpp_reticle] shot gate unavailable (no ShootEvent)");
    }
  } else {
    if now - this.utppRetLastShot > 1.0 {
      return false;
    }
  }
  this.UTPPRetFlash(this.utppRetHitX, 0.20, 1.15, 0.06);
  if this.utppRetOn {
    let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
    if IsDefined(state) {
      state.m_hits += 1;
    }
  }
  return true;
}

@addMethod(gameuiCrosshairBaseGameController)
protected cb func UTPPReticleOnKill(value: Variant) -> Bool {
  if !IsDefined(this.utppRetKillX) || !IsDefined(this.m_playerPuppet) {
    return false;
  }
  let list: array<KillInfo> = FromVariant<array<KillInfo>>(value);
  let found: Bool = false;
  let i: Int32 = 0;
  while i < ArraySize(list) && !found {
    found = list[i].victimEntity != null && list[i].killerEntity == this.m_playerPuppet;
    i += 1;
  }
  if !found {
    return false;
  }
  this.utppRetKillUntil = UTPPRetNow() + 0.2;
  this.utppRetHitX.StopAllAnimations();
  this.utppRetHitX.SetOpacity(0.0);
  this.UTPPRetFlash(this.utppRetKillX, 0.35, 1.25, 0.08);
  if this.utppRetOn {
    let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
    if IsDefined(state) {
      state.m_kills += 1;
    }
  }
  return true;
}

@addMethod(gameuiCrosshairBaseGameController)
public final func UTPPReticleStatus() -> String {
  let gate: String = "ok";
  if this.utppRetShots == 0 {
    gate = "ungated";
  }
  if !IsDefined(this.utppReticle) {
    return "active=" + NameToString(this.GetClassName()) + " root=none";
  }
  // tint read back from the widgets: shows whether the style binding resolved and follows the GUI state
  let dotTint: String = "-";
  if IsDefined(this.utppRetDotCore) {
    dotTint = UTPPRetColourText(this.utppRetDotCore.GetTintColor());
  }
  let arcTint: String = "-";
  let i: Int32 = 0;
  while i < ArraySize(this.utppRetStyled) && Equals(arcTint, "-") {
    if IsDefined(this.utppRetStyled[i]) && this.utppRetStyled[i] != this.utppRetDotCore {
      arcTint = UTPPRetColourText(this.utppRetStyled[i].GetTintColor());
    }
    i += 1;
  }
  return "active=" + NameToString(this.GetClassName()) + " shown=" + UTPPRetFlag(this.utppRetOn) + " R="
    + FloatToStringPrec(this.utppRetR, 1) + " shots=" + IntToString(this.utppRetShots) + " gate=" + gate + " dot="
    + this.utppRetDotKind + " guiState=" + NameToString(this.utppRetGuiState) + " sampled="
    + UTPPRetColourText(this.utppRetColour) + " color=" + UTPPRetColourModeName(UTPPRetColourModeOf(GetGameInstance()))
    + " dotTint=" + dotTint + " arcTint=" + arcTint + " bindFails="
    + IntToString(this.utppRetBindFails) + " ring=" + UTPPRetArcModeName(this.utppRetArcModeBuilt) + " ringWidgets="
    + IntToString(this.utppRetRingWidgets) + " ringBuilds=" + IntToString(this.utppRetRingBuilds) + " presetRead="
    + UTPPRetFlag(this.utppRetPresetRead) + " presetSize=" + UTPPRetVecText(this.UTPPRetPresetSizeOrDefault()) + " baseR="
    + FloatToStringPrec(this.utppRetBaseR, 1) + " kick=" + FloatToStringPrec(UTPPRetKickAt(this.utppRetKick0, this.utppRetKickT0,
    UTPPRetNow()), 1) + " spread=" + UTPPRetVecText(this.utppRetSpread) + " kickSelfTest=" + UTPPCrosshairKickSelfTest();
}

// ---------------------------------------------------------------- hooks on the base class (every crosshair controller)

@wrapMethod(gameuiCrosshairBaseGameController)
protected cb func OnPreIntro() -> Bool {
  let cls: CName = this.GetClassName();
  this.utppRetKind = UTPPRetKindOf(cls);
  this.utppRetIntroSeen = true;
  this.utppRetSpreadFloor = -1.0;
  this.utppRetExcessLast = 0.0;
  // the base OnPreIntro calls OnBulletSpreadChanged and UpdateCrosshairState right away, so the widgets must exist first
  if this.utppRetKind > 0 {
    this.UTPPReticleBuild();
  }
  let result: Bool = wrappedMethod();
  this.UTPPReticleActivate("equip");
  return result;
}

// After the widgets exist (or the build failed): listeners, active controller, ownership, one log line naming the class.
@addMethod(gameuiCrosshairBaseGameController)
private final func UTPPReticleActivate(why: String) -> Void {
  let cls: CName = this.GetClassName();
  let rootState: String = "none";
  let weapon: String = TDBID.ToStringDEBUG(ItemID.GetTDBID(this.GetWeaponRecordID()));
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if IsDefined(state) {
    state.m_equips += 1;
    state.m_lastClass = cls;
    state.m_lastHandled = IsDefined(this.utppReticle);
  }
  if IsDefined(this.utppReticle) {
    rootState = "ok";
    this.UTPPRetDotUpgrade();
    this.UTPPReticleListen();
    if IsDefined(state) {
      state.m_active = this;
    }
    this.UTPPReticleApply();
  }
  let line: String = "[utpp_reticle] " + why + " class=" + NameToString(cls) + " handled=" + UTPPRetFlag(this.utppRetKind > 0)
    + " weapon=" + weapon + " on=" + UTPPRetFlag(UTPPCrosshairOn(GetGameInstance()));
  if this.utppRetKind > 0 {
    line += " root=" + rootState + " dot=" + this.utppRetDotKind + " R=" + FloatToStringPrec(this.utppRetR, 1) + " hidden="
      + this.utppRetHiddenNames + " arcs=" + UTPPRetArcModeName(this.utppRetArcModeBuilt) + " ringWidgets="
      + IntToString(this.utppRetRingWidgets) + " colour="
      + UTPPRetColourText(this.utppRetColour) + " src=" + this.utppRetColourSrc + " color="
      + UTPPRetColourModeName(UTPPRetColourModeOf(GetGameInstance())) + " bindFails="
      + IntToString(this.utppRetBindFails);
  }
  UTPPLogLine(line);
  if IsDefined(state) {
    state.RefreshCentreDot();
  }
}

// Second, independent entry: the first crosshair-state change of an active controller. It names the class once per controller
// even if the OnPreIntro wrap above never ran for it ("state ... intro=0"), and in that case builds and activates a handled
// controller late ("late" line), so the reticle does not depend on a single hook point.
@wrapMethod(gameuiCrosshairBaseGameController)
protected func OnCrosshairStateChange(oldState: gamePSMCrosshairStates, newState: gamePSMCrosshairStates) -> Void {
  wrappedMethod(oldState, newState);
  if this.utppRetStateSeen || !this.IsActive() {
    return;
  }
  this.utppRetStateSeen = true;
  let cls: CName = this.GetClassName();
  UTPPLogLine("[utpp_reticle] state class=" + NameToString(cls) + " handled=" + UTPPRetFlag(UTPPRetKindOf(cls) > 0) + " intro="
    + UTPPRetFlag(this.utppRetIntroSeen) + " built=" + UTPPRetFlag(IsDefined(this.utppReticle)));
  if !this.utppRetIntroSeen && UTPPRetKindOf(cls) > 0 {
    this.utppRetKind = UTPPRetKindOf(cls);
    this.UTPPReticleBuild();
    this.UTPPReticleActivate("late");
  }
}

@wrapMethod(gameuiCrosshairBaseGameController)
protected cb func OnPreOutro() -> Bool {
  this.UTPPReticleUnlisten();
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if IsDefined(state) && state.m_active == this {
    state.m_active = null;
    state.RefreshCentreDot();
  }
  return wrappedMethod();
}

@wrapMethod(gameuiCrosshairBaseGameController)
protected cb func OnBulletSpreadChanged(spread: Vector2) -> Bool {
  let result: Bool = wrappedMethod(spread);
  this.UTPPReticleLayout(spread);
  return result;
}

// ---------------------------------------------------------------- Basic: mirror its part fades

@wrapMethod(CrosshairGameController_Basic)
protected final func ColapseCrosshair(full: Bool, duration: Float) -> Void {
  wrappedMethod(full, duration);
  this.UTPPRetFade(this.utppRetRing, 0.0, duration);
}

@wrapMethod(CrosshairGameController_Basic)
protected final func ExpandCrosshair(full: Bool, duration: Float) -> Void {
  wrappedMethod(full, duration);
  this.UTPPRetFade(this.utppRetRing, 1.0, duration);
}

// The dot follows the centre part: Expand / Colapse with full = true reach these through the original.
@wrapMethod(CrosshairGameController_Basic)
private final func ShowCenterPart(duration: Float) -> Void {
  wrappedMethod(duration);
  this.UTPPRetFade(this.utppRetDot, 1.0, duration);
}

@wrapMethod(CrosshairGameController_Basic)
private final func HideCenterPart(duration: Float) -> Void {
  wrappedMethod(duration);
  this.UTPPRetFade(this.utppRetDot, 0.0, duration);
}

// ---------------------------------------------------------------- crosshair GUI state -> our colour

@wrapMethod(CrosshairGameController_Basic)
protected func ApplyCrosshairGUIState(state: CName, aimedAtEntity: ref<Entity>) -> Void {
  wrappedMethod(state, aimedAtEntity);
  this.UTPPRetGuiState(state);
}

@wrapMethod(CrosshairGameController_Simple)
protected func ApplyCrosshairGUIState(state: CName, aimedAtEntity: ref<Entity>) -> Void {
  wrappedMethod(state, aimedAtEntity);
  this.UTPPRetGuiState(state);
}

// ---------------------------------------------------------------- accessibility centre dot

// The accessibility "persistent centre dot" (its own HUD controller) sits on top of our dot. While our reticle owns the
// centre it stays hidden; it comes back with the setting off, without a handled crosshair or after the weapon is put away.
@addField(CrosshairGameControllerPersistentDot)
public let utppRetSuppressed: Bool;

@addMethod(CrosshairGameControllerPersistentDot)
public final func UTPPRetRefresh() -> Void {
  this.UpdateRootVisibility();
}

@wrapMethod(CrosshairGameControllerPersistentDot)
private final func UpdateRootVisibility() -> Void {
  wrappedMethod();
  let state: ref<UTPPCrosshairState> = UTPPCrosshairStateOf(GetGameInstance());
  if !IsDefined(state) {
    return;
  }
  state.m_centerDot = this;
  let suppress: Bool = state.OwnsCentre();
  if suppress {
    this.GetRootWidget().SetVisible(false);
  }
  if NotEquals(suppress, this.utppRetSuppressed) {
    this.utppRetSuppressed = suppress;
    UTPPLogLine("[utpp_reticle] centre dot suppressed=" + UTPPRetFlag(suppress));
  }
}

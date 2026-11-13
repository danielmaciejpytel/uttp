// UTPP - manual shoulder swap input.
// Installed to r6\scripts\UTPP\utpp_shoulder_input.reds together with r6\input\utpp_shoulder.xml. Kept out of the
// generated utpp_modsettings.reds on purpose. Script functions only, no natives.
// Input Loader merges r6\input\utpp_shoulder.xml: UTPP_ShoulderSwap (keyboard / mouse, default L) and three controller
// gestures (UTPP_ShoulderSwapPadR3 = right stick click held 0.5 s, UTPP_ShoulderSwapPadDown / Left = d-pad taps). The
// actions live in the on-foot context Exploration (also active while aiming and scanning). This system only turns the
// chosen gesture into "swap requests"; utpp_camera (CET) drains them every frame with Game.UTPPShoulderSwapTake() and
// applies them through its own gates (UTPP camera on, third person, on foot, no scanner / takedown / carry, no menu);
// in first person the request is ignored (not remembered).
// Mod Settings: utppShoulderSwapKey is an EInputKey field whose NAME equals the overridableUI id of the XML button, so
// the framework rebinds the input mapping itself (user.ini [UTPPShoulderInputSettings]; no module statement). The controller
// gesture is a fixed list (enum), because every controller button is already used by the game.
// Without Mod Settings: the defaults apply (L, right stick hold). Without Input Loader: no action exists, nothing fires.
// The R3 hold is measured here as well as by the engine: while crouched the engine's BUTTON_HOLD_COMPLETE for the R3 hold
// gesture is not delivered. BUTTON_PRESSED starts a 0.5 s DelaySystem timer (sim time, not time dilated); when it fires and
// the button is still down, that is the same swap as BUTTON_HOLD_COMPLETE. At most one swap per press (HOLD_COMPLETE and the
// timer are deduplicated per press). A release before 0.5 s never swaps (a 0.35 s hold stays a crouch tap). The action is
// never consumed (OnAction returns false), so the game's tap = crouch toggle is untouched.

public enum UTPPShoulderPad {
  Off = 0,
  RightStickHold = 1,
  DpadDownTap = 2,
  DpadLeftTap = 3
}

public class UTPPShoulderInputSettings {
  @runtimeProperty("ModSettings.mod", "Ultimate Third Person")
  @runtimeProperty("ModSettings.category", "Camera")
  @runtimeProperty("ModSettings.category.order", "2")
  @runtimeProperty("ModSettings.order", "900")
  @runtimeProperty("ModSettings.displayName", "Swap shoulder (keyboard / mouse)")
  @runtimeProperty("ModSettings.description", "Moves the third-person camera to the other shoulder while on foot (also while aiming; ignored in first person, vehicles, photo mode and menus). Needs Input Loader. Default L. Pick a key the game does not use (for example L, O, F2, F3, Mouse 4 or Mouse 5); a key that also has a game action triggers both.")
  public let utppShoulderSwapKey: EInputKey = EInputKey.IK_L;

  @runtimeProperty("ModSettings.mod", "Ultimate Third Person")
  @runtimeProperty("ModSettings.category", "Camera")
  @runtimeProperty("ModSettings.category.order", "2")
  @runtimeProperty("ModSettings.order", "901")
  @runtimeProperty("ModSettings.displayName", "Swap shoulder (controller)")
  @runtimeProperty("ModSettings.description", "Every controller button already has a game action. Hold right stick click (default): hold R3 for half a second; a short R3 click keeps quick melee or crouch. The d-pad options also trigger the game's own d-pad action.")
  @runtimeProperty("ModSettings.displayValues.Off", "Off")
  @runtimeProperty("ModSettings.displayValues.RightStickHold", "Hold right stick click")
  @runtimeProperty("ModSettings.displayValues.DpadDownTap", "Tap d-pad down (also cycles the objective)")
  @runtimeProperty("ModSettings.displayValues.DpadLeftTap", "Tap d-pad left (also toggles the radio)")
  public let utppShoulderSwapPad: UTPPShoulderPad = UTPPShoulderPad.RightStickHold;
}

// 0.5 s timer of one R3 press (DelaySystem, not time dilated); a stale timer is ignored by sequence number
public class UTPPShoulderHoldTick extends DelayCallback {
  public let seq: Int32;

  public func Call() -> Void {
    let s: ref<UTPPShoulderInput> = UTPPShoulderInputOf();
    if IsDefined(s) {
      s.OnHoldTick(this.seq);
    };
  }
}

public class UTPPShoulderInput extends ScriptableSystem {
  public let m_settings: ref<UTPPShoulderInputSettings>;
  private let m_player: wref<PlayerPuppet>;
  private let m_requests: Int32;      // swap requests since the last Take()
  private let m_source: String;       // "key" | "pad" of the last request
  private let m_downHeld: Bool;       // d-pad down reached its hold -> its release is not a tap
  private let m_leftHeld: Bool;
  private let m_events: Int32;        // diagnostics: UTPP actions seen
  private let m_registered: Bool;
  private let m_r3Down: Bool;         // R3 action pressed and not yet released
  private let m_r3Seq: Int32;         // press counter (a late timer of an older press is ignored)
  private let m_r3Swapped: Bool;      // this press already produced its swap

  public func OnAttach() -> Void {
    this.m_settings = new UTPPShoulderInputSettings();
    this.Listen(true);
  }

  public func OnDetach() -> Void {
    this.Listen(false);
    this.Unregister();
  }

  private final func OnPlayerAttach(request: ref<PlayerAttachRequest>) -> Void {
    let player: ref<PlayerPuppet> = request.owner as PlayerPuppet;
    if !IsDefined(player) {
      return;
    };
    this.Unregister();
    this.m_r3Down = false;
    this.m_player = player;
    player.RegisterInputListener(this, n"UTPP_ShoulderSwap");
    player.RegisterInputListener(this, n"UTPP_ShoulderSwapPadR3");
    player.RegisterInputListener(this, n"UTPP_ShoulderSwapPadDown");
    player.RegisterInputListener(this, n"UTPP_ShoulderSwapPadLeft");
    this.m_registered = true;
    this.m_requests = 0;
    this.Listen(false);
    this.Listen(true);
  }

  private final func OnPlayerDetach(request: ref<PlayerDetachRequest>) -> Void {
    this.Unregister();
  }

  private final func Unregister() -> Void {
    let player: ref<PlayerPuppet> = this.m_player;
    if IsDefined(player) && this.m_registered {
      player.UnregisterInputListener(this);
    };
    this.m_registered = false;
    this.m_r3Down = false;
    this.m_downHeld = false;
    this.m_leftHeld = false;
    this.m_player = null;
  }

  @if(ModuleExists("ModSettingsModule"))
  private final func Listen(on: Bool) -> Void {
    if on {
      ModSettings.RegisterListenerToClass(this.m_settings);
    } else {
      ModSettings.UnregisterListenerToClass(this.m_settings);
    };
  }

  @if(!ModuleExists("ModSettingsModule"))
  private final func Listen(on: Bool) -> Void {}

  private final func Request(source: String) -> Void {
    this.m_requests += 1;
    this.m_source = source;
  }

  protected cb func OnAction(action: ListenerAction, consumer: ListenerActionConsumer) -> Bool {
    let name: CName = ListenerAction.GetName(action);
    let t: gameinputActionType = ListenerAction.GetType(action);
    let pad: UTPPShoulderPad = this.m_settings.utppShoulderSwapPad;
    if Equals(name, n"UTPP_ShoulderSwap") {
      this.m_events += 1;
      if Equals(t, gameinputActionType.BUTTON_PRESSED) {
        this.Request("key");
      };
      return false;
    };
    if Equals(name, n"UTPP_ShoulderSwapPadR3") {
      this.m_events += 1;
      this.OnR3(t, pad);
      return false;
    };
    if Equals(name, n"UTPP_ShoulderSwapPadDown") {
      this.m_events += 1;
      if Equals(t, gameinputActionType.BUTTON_PRESSED) {
        this.m_downHeld = false;
      } else {
        if Equals(t, gameinputActionType.BUTTON_HOLD_COMPLETE) {
          this.m_downHeld = true;
        } else {
          if Equals(t, gameinputActionType.BUTTON_RELEASED) && !this.m_downHeld && Equals(pad, UTPPShoulderPad.DpadDownTap) {
            this.Request("pad");
          };
        };
      };
      return false;
    };
    if Equals(name, n"UTPP_ShoulderSwapPadLeft") {
      this.m_events += 1;
      if Equals(t, gameinputActionType.BUTTON_PRESSED) {
        this.m_leftHeld = false;
      } else {
        if Equals(t, gameinputActionType.BUTTON_HOLD_COMPLETE) {
          this.m_leftHeld = true;
        } else {
          if Equals(t, gameinputActionType.BUTTON_RELEASED) && !this.m_leftHeld && Equals(pad, UTPPShoulderPad.DpadLeftTap) {
            this.Request("pad");
          };
        };
      };
      return false;
    };
    return false;
  }

  private final func StartR3Press() -> Void {
    this.m_r3Seq += 1;
    this.m_r3Down = true;
    this.m_r3Swapped = false;
  }

  private final func SwapR3() -> Void {
    if this.m_r3Swapped {
      return;
    };
    this.m_r3Swapped = true;
    this.Request("pad");
  }

  private final func OnR3(t: gameinputActionType, pad: UTPPShoulderPad) -> Void {
    if Equals(t, gameinputActionType.BUTTON_PRESSED) {
      this.StartR3Press();
      if Equals(pad, UTPPShoulderPad.RightStickHold) {
        let cb: ref<UTPPShoulderHoldTick> = new UTPPShoulderHoldTick();
        cb.seq = this.m_r3Seq;
        GameInstance.GetDelaySystem(this.GetGameInstance()).DelayCallback(cb, 0.5, false);
      };
      return;
    };
    if Equals(t, gameinputActionType.BUTTON_HOLD_COMPLETE) {
      if !this.m_r3Down {
        // the press itself was not seen: treat the completed hold as its own press
        this.StartR3Press();
      };
      if Equals(pad, UTPPShoulderPad.RightStickHold) {
        this.SwapR3();
      };
      return;
    };
    if Equals(t, gameinputActionType.BUTTON_RELEASED) {
      this.m_r3Down = false;
      this.m_r3Swapped = false;
    };
  }

  // the 0.5 s timer of press seq ran: swap when that press is still held and has not swapped yet (the engine's own
  // BUTTON_HOLD_COMPLETE is the first choice; this covers a stance in which it is not delivered)
  public final func OnHoldTick(seq: Int32) -> Void {
    if seq != this.m_r3Seq || !this.m_r3Down {
      return;
    };
    if Equals(this.m_settings.utppShoulderSwapPad, UTPPShoulderPad.RightStickHold) {
      this.SwapR3();
    };
  }

  public final func Take() -> Int32 {
    let n: Int32 = this.m_requests;
    this.m_requests = 0;
    return n;
  }

  public final func Status() -> String {
    return "registered=" + ToString(this.m_registered) + " events=" + ToString(this.m_events) + " pending="
      + ToString(this.m_requests) + " last=" + this.m_source + " pad=" + ToString(EnumInt(this.m_settings.utppShoulderSwapPad))
      + " key=" + NameToString(EnumValueToName(n"EInputKey", Cast<Int64>(EnumInt(this.m_settings.utppShoulderSwapKey))));
  }

  public final func LastTrigger() -> String {
    return this.m_source;
  }
}

public func UTPPShoulderInputOf() -> ref<UTPPShoulderInput> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(GetGameInstance());
  if !IsDefined(container) {
    return null;
  };
  return container.Get(n"UTPPShoulderInput") as UTPPShoulderInput;
}

// CET: Game.UTPPShoulderSwapTake() -> number of swap requests since the last call (-1 = system missing)
public func UTPPShoulderSwapTake() -> Int32 {
  let s: ref<UTPPShoulderInput> = UTPPShoulderInputOf();
  if !IsDefined(s) {
    return -1;
  };
  return s.Take();
}

// CET: Game.UTPPShoulderSwapTrigger() -> "key" | "pad" | "" of the last request
public func UTPPShoulderSwapTrigger() -> String {
  let s: ref<UTPPShoulderInput> = UTPPShoulderInputOf();
  if !IsDefined(s) {
    return "";
  };
  return s.LastTrigger();
}

// CET / console: Game.UTPPShoulderInputStatus()
public func UTPPShoulderInputStatus() -> String {
  let s: ref<UTPPShoulderInput> = UTPPShoulderInputOf();
  if !IsDefined(s) {
    return "UTPPShoulderInput missing";
  };
  return s.Status();
}

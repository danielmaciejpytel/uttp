// UTPP - Mod Settings bridge (CET <-> Mod Settings).
//
// The UTPP options shown in the game's Mod Settings screen are declared in utpp_modsettings.reds (class UTPPModSettings,
// generated from the CET settings registry by the settings generator). The CET registry (utpp_core) stays the
// only place that applies settings; these global functions let it read and write the Mod Settings values:
//   Game.UTPPModSettingsBridgeVariant() -> String  "modsettings" (compiled with the framework) | "stub" (without it)
//   Game.UTPPModSettingsBridgeInfo() -> String     diagnostics: variant, mods, UTPP listed, listener, revision, ConfigVars
//   Game.UTPPModSettingsAvailable() -> Bool    the framework is compiled in and lists the UTPP mod (static data, works
//                                              in the main menu)
//   Game.UTPPModSettingsMenuOpen()  -> Bool    the Mod Settings screen is open (its edits are not accepted yet)
//   Game.UTPPModSettingsRevision()  -> Int32   counts Mod Settings change notifications (-1 = no game session yet)
//   Game.UTPPModSettingsSnapshot()  -> String  "field=value\n" for every UTPP option (bool true|false, float, int,
//                                              enum = its integer value);
//                                              "" until a game session exists
//   Game.UTPPModSettingsPush(data)  -> Int32   writes "field=value\n" lines into the Mod Settings values and accepts them
//                                              (saved to Mod Settings' user.ini); returns the number of changed values,
//                                              -1 = Mod Settings missing, -2 = its screen is open or holds pending edits,
//                                              -3 = the framework lists no UTPP ConfigVars (values cannot be written)
//   Game.UTPPModSettingsSelfTest() -> String   reads camera_impl, writes camera_aim_distance +-0.05 and restores it
//
// Reading: like other Mod Settings users, the values come from a registered listener instance of UTPPModSettings (the
// framework fills its fields when it is registered and after every accepted change). The instance lives in the script
// system UTPPModSettingsBridge, so it exists per game session. Writing goes through the framework's ConfigVars of the
// UTPP mod (ModSettings.GetCategories / GetVars). Without Mod Settings the stub variants below compile instead.

// name of the UTPP entry in the Mod Settings list = ModSettings.mod of utpp_modsettings.reds (the generator checks it)
public func UTPPModSettingsModName() -> CName {
  return n"Ultimate Third Person";
}

public func UTPPMSBool(b: Bool) -> String {
  return b ? "true" : "false";
}

public func UTPPMSFloat(f: Float) -> String {
  return FloatToStringPrec(f, 6);
}

// ---------------------------------------------------------------- per-session listener instance

public class UTPPModSettingsBridge extends ScriptableSystem {
  public let m_settings: ref<UTPPModSettings>;
  public let m_revision: Int32;
  public let m_listening: Bool;

  private func OnAttach() -> Void {
    this.m_settings = new UTPPModSettings();
    this.Listen(true);
  }

  private func OnDetach() -> Void {
    this.Listen(false);
  }

  // registered again when the player attaches (the pattern of the framework author's own mods)
  private final func OnPlayerAttach(request: ref<PlayerAttachRequest>) -> Void {
    this.Listen(false);
    this.Listen(true);
  }

  @if(ModuleExists("ModSettingsModule"))
  private final func Listen(on: Bool) -> Void {
    if on {
      ModSettings.RegisterListenerToClass(this.m_settings);
      ModSettings.RegisterListenerToModifications(this);
    } else {
      ModSettings.UnregisterListenerToClass(this.m_settings);
      ModSettings.UnregisterListenerToModifications(this);
    };
    this.m_listening = on;
  }

  @if(!ModuleExists("ModSettingsModule"))
  private final func Listen(on: Bool) -> Void {}

  public cb func OnModSettingsChange() {
    this.m_revision += 1;
  }
}

public func UTPPModSettingsBridgeOf() -> ref<UTPPModSettingsBridge> {
  let container: ref<ScriptableSystemsContainer> = GameInstance.GetScriptableSystemsContainer(GetGameInstance());
  if !IsDefined(container) {
    return null;
  };
  return container.Get(n"UTPPModSettingsBridge") as UTPPModSettingsBridge;
}

public func UTPPModSettingsRevision() -> Int32 {
  let b: ref<UTPPModSettingsBridge> = UTPPModSettingsBridgeOf();
  if !IsDefined(b) {
    return -1;
  };
  return b.m_revision;
}

public func UTPPModSettingsSnapshot() -> String {
  let b: ref<UTPPModSettingsBridge> = UTPPModSettingsBridgeOf();
  if !IsDefined(b) || !IsDefined(b.m_settings) || !b.m_listening {
    return "";
  };
  return b.m_settings.UTPPSnapshot();
}

// ---------------------------------------------------------------- framework calls (ConfigVars)

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsVars() -> array<ref<ConfigVar>> {
  let out: array<ref<ConfigVar>>;
  let vars: array<ref<ConfigVar>> = ModSettings.GetVars(UTPPModSettingsModName(), n"None");
  let i: Int32 = 0;
  while i < ArraySize(vars) {
    ArrayPush(out, vars[i]);
    i += 1;
  };
  let cats: array<CName> = ModSettings.GetCategories(UTPPModSettingsModName());
  let c: Int32 = 0;
  while c < ArraySize(cats) {
    if NotEquals(cats[c], n"None") {
      vars = ModSettings.GetVars(UTPPModSettingsModName(), cats[c]);
      i = 0;
      while i < ArraySize(vars) {
        ArrayPush(out, vars[i]);
        i += 1;
      };
    };
    c += 1;
  };
  return out;
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsListed() -> Bool {
  let mods: array<CName> = ModSettings.GetMods();
  let i: Int32 = 0;
  while i < ArraySize(mods) {
    if Equals(mods[i], UTPPModSettingsModName()) {
      return true;
    };
    i += 1;
  };
  return false;
}

// sets one value when it differs from the current one; true = changed
@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsAssign(v: ref<ConfigVar>, value: String) -> Bool {
  let b: ref<ModConfigVarBool> = v as ModConfigVarBool;
  if IsDefined(b) {
    let bv: Bool = Equals(value, "true");
    if !bv && !Equals(value, "false") {
      return false;
    };
    if Equals(b.GetValue(), bv) {
      return false;
    };
    b.SetValue(bv);
    return true;
  };
  let f: ref<ModConfigVarFloat> = v as ModConfigVarFloat;
  if IsDefined(f) {
    let fv: Float = StringToFloat(value, f.GetValue());
    if AbsF(f.GetValue() - fv) <= 0.00001 {
      return false;
    };
    f.SetValue(fv);
    return true;
  };
  let n: ref<ModConfigVarInt32> = v as ModConfigVarInt32;
  if IsDefined(n) {
    let nv: Int32 = StringToInt(value, n.GetValue());
    if n.GetValue() == nv {
      return false;
    };
    n.SetValue(nv);
    return true;
  };
  let e: ref<ModConfigVarEnum> = v as ModConfigVarEnum;
  if IsDefined(e) {
    let ev: Int32 = StringToInt(value, e.GetValue());
    let idx: Int32 = e.GetIndexFor(ev);
    if e.GetValue() == ev || idx < 0 {
      return false;
    };
    e.SetIndex(idx);
    return true;
  };
  return false;
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsBridgeVariant() -> String {
  return "modsettings";
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsBridgeVariant() -> String {
  return "stub";
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsBridgeInfo() -> String {
  let b: ref<UTPPModSettingsBridge> = UTPPModSettingsBridgeOf();
  let session: String = "no session";
  if IsDefined(b) {
    session = "listener=" + UTPPMSBool(b.m_listening) + " revision=" + IntToString(b.m_revision);
  };
  return "variant=modsettings mods=" + IntToString(ArraySize(ModSettings.GetMods())) + " utppListed="
    + UTPPMSBool(UTPPModSettingsListed()) + " " + session + " categories="
    + IntToString(ArraySize(ModSettings.GetCategories(UTPPModSettingsModName()))) + " configVars="
    + IntToString(ArraySize(UTPPModSettingsVars()));
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsBridgeInfo() -> String {
  return "variant=stub (compiled without the Mod Settings framework)";
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsAvailable() -> Bool {
  return UTPPModSettingsListed();
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsAvailable() -> Bool {
  return false;
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsMenuOpen() -> Bool {
  let ms: ref<ModSettings> = ModSettings.GetInstance();
  return IsDefined(ms) && ms.isActive;
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsMenuOpen() -> Bool {
  return false;
}

@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsPush(data: String) -> Int32 {
  let ms: ref<ModSettings> = ModSettings.GetInstance();
  if !IsDefined(ms) {
    return -1;
  };
  // AcceptChanges is global: it must not accept edits that the player has not confirmed in the open screen
  if ms.isActive || ms.changesRequested {
    return -2;
  };
  let vars: array<ref<ConfigVar>> = UTPPModSettingsVars();
  if ArraySize(vars) == 0 {
    return -3;
  };
  let names: array<String>;
  let values: array<String>;
  let lines: array<String> = StrSplit(data, "\n", false);
  let i: Int32 = 0;
  while i < ArraySize(lines) {
    let left: String;
    let right: String;
    if StrSplitFirst(lines[i], "=", left, right) {
      ArrayPush(names, left);
      ArrayPush(values, right);
    };
    i += 1;
  };
  let changed: Int32 = 0;
  let v: Int32 = 0;
  while v < ArraySize(vars) {
    let name: String = NameToString(vars[v].GetName());
    let j: Int32 = 0;
    while j < ArraySize(names) {
      if Equals(names[j], name) {
        if UTPPModSettingsAssign(vars[v], values[j]) {
          changed += 1;
        };
        j = ArraySize(names);
      } else {
        j += 1;
      };
    };
    v += 1;
  };
  if changed > 0 {
    ModSettings.AcceptChanges();
  };
  return changed;
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsPush(data: String) -> Int32 {
  return -1;
}

// CET: print(Game.UTPPModSettingsSelfTest()). Reads camera_impl from the listener instance, then writes camera_aim_distance
// (aim camera distance, +-0.05) through the ConfigVars, reads it back from the ConfigVar and the instance, and
// restores the old value. Needs a game session and a closed Mod Settings screen.
@if(ModuleExists("ModSettingsModule"))
public func UTPPModSettingsSelfTest() -> String {
  let out: String = UTPPModSettingsBridgeInfo();
  let b: ref<UTPPModSettingsBridge> = UTPPModSettingsBridgeOf();
  if !IsDefined(b) || !IsDefined(b.m_settings) {
    return out + " | read: no game session";
  };
  out += " | read camera_impl=" + UTPPMSBool(b.m_settings.camera_impl) + " camera_aim_distance="
    + UTPPMSFloat(b.m_settings.camera_aim_distance);
  let ms: ref<ModSettings> = ModSettings.GetInstance();
  if !IsDefined(ms) {
    return out + " | write: skipped (Mod Settings instance unavailable)";
  };
  if ms.isActive || ms.changesRequested {
    return out + " | write: skipped (Mod Settings screen open or pending edits)";
  };
  let vars: array<ref<ConfigVar>> = UTPPModSettingsVars();
  let target: ref<ModConfigVarFloat>;
  let i: Int32 = 0;
  while i < ArraySize(vars) {
    if Equals(vars[i].GetName(), n"camera_aim_distance") {
      target = vars[i] as ModConfigVarFloat;
    };
    i += 1;
  };
  if !IsDefined(target) {
    return out + " | write: ConfigVar camera_aim_distance not found (" + IntToString(ArraySize(vars)) + " ConfigVars)";
  };
  let old: Float = target.GetValue();
  let probe: Float = old > 1.9 ? old - 0.05 : old + 0.05;
  target.SetValue(probe);
  ModSettings.AcceptChanges();
  out += " | write " + UTPPMSFloat(probe) + ": configVar=" + UTPPMSFloat(target.GetValue()) + " instance="
    + UTPPMSFloat(b.m_settings.camera_aim_distance) + " revision=" + IntToString(b.m_revision);
  target.SetValue(old);
  ModSettings.AcceptChanges();
  out += " | restored " + UTPPMSFloat(old) + ": configVar=" + UTPPMSFloat(target.GetValue()) + " instance="
    + UTPPMSFloat(b.m_settings.camera_aim_distance);
  return out;
}

@if(!ModuleExists("ModSettingsModule"))
public func UTPPModSettingsSelfTest() -> String {
  return UTPPModSettingsBridgeInfo();
}

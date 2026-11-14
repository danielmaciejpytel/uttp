-- Isolated CET v2 state/acknowledgement regression tests (Lua 5.1).
local passed, failed = 0, 0
local function check(name, value)
  if value then passed = passed + 1; print("[PASS] utpp_motion: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_motion: " .. name) end
end
local function fresh()
  local v = dofile(UTPP_MOTION_PATH)
  v.sink = function() end
  return v
end
local function mock()
  local t = { enabled = false, calls = {}, mode = "ok", aim = {}, slide = {}, orient = {}, guards = {}, lean = {}, leanHead = {}, leanParams = {}, trim = {}, shoulder = {}, bodyOffset = {},
    walk = {}, minWalk = {}, barrel = {}, camCalls = {}, camOn = false }
  -- utpp_camera stand-in: SetAimPoint drives CM.AIM_POINT.enabled (t.camOn)
  local cam = { SetAimPoint = function(on)
    t.camCalls[#t.camCalls + 1] = on
    if t.camError then error("camera failure") end
    t.camOn = on == true
    return t.camOn
  end }
  local glue = {}
  function glue:IsEnabled() return t.enabled end
  function glue:SetPistolOneHand(on) t.pistol = on end
  function glue:SetPistolAuto(on)
    t.autoCalls = (t.autoCalls or 0) + 1
    if t.autoError then error("auto failure") end
    if t.autoReject then return false end
    t.auto = on
  end
  function glue:SetEnabled(on)
    t.calls[#t.calls + 1] = on
    if t.mode == "error" then error("glue failure") end
    if t.mode == "false" then return false end
    t.enabled = on
  end
  t.api = {
    glue = function() if t.glueMissing then return nil end; return glue end,
    havePlayer = function() return true end,
    setEnabledGlobal = function(on) t.global = (t.global or 0) + 1; return t.globalResult end,
    setAimGameUpper = function(on)
      t.aim[#t.aim + 1] = on
      if t.aimError then error("aim native failure") end
      return not t.aimReject
    end,
    setSlide = function(on)
      t.slide[#t.slide + 1] = on
      if t.slideError then error("slide native failure") end
      return not t.slideReject
    end,
    setLean = function(on)
      t.lean[#t.lean + 1] = on
      if t.leanError then error("lean native failure") end
      return not t.leanReject
    end,
    setLeanHead = function(on)
      t.leanHead[#t.leanHead + 1] = on
      return not t.leanHeadReject
    end,
    setLeanParams = function(scale, headScale)
      t.leanParams[#t.leanParams + 1] = { scale, headScale }
      return not t.leanParamsReject
    end,
    setSelectionGuards = function(on)
      t.guards[#t.guards + 1] = on
      if t.guardsError then error("selectionGuards native failure") end
      return not t.guardsReject
    end,
    setOrientWarp = function(on)
      t.orient[#t.orient + 1] = on
      if t.orientError then error("orientWarp native failure") end
      return not t.orientReject
    end,
    setShoulderNarrow = function(cm)
      t.shoulder[#t.shoulder + 1] = cm
      if t.shoulderError then error("shoulder narrow native failure") end
      return not t.shoulderReject
    end,
    setPelvisYawTrim = function(deg)
      t.trim[#t.trim + 1] = deg
      if t.trimError then error("pelvis trim native failure") end
      return not t.trimReject
    end,
    setBodyYawOffset = function(deg)
      t.bodyOffset[#t.bodyOffset + 1] = deg
      if t.bodyOffsetError then error("body yaw offset native failure") end
      return not t.bodyOffsetReject
    end,
    setWalkSpeed = function(ms)
      t.walk[#t.walk + 1] = ms
      if t.walkError then error("walk speed native failure") end
      return not t.walkReject
    end,
    setMinAnalogWalk = function(ms)
      t.minWalk[#t.minWalk + 1] = ms
      if t.minWalkError then error("minimum walk speed native failure") end
      return not t.minWalkReject
    end,
    setBarrelLoop = function(on)
      t.barrel[#t.barrel + 1] = on
      if t.barrelError then error("barrel loop native missing") end
      return not t.barrelReject
    end,
    cameraModule = function() if t.camMissing then return nil end; return cam end,
    setWeaponPoseOverride = function(record, ready, aim)
      t.overrideCalls = (t.overrideCalls or 0) + 1
      if t.overrideError then error("override failure") end
      if t.overrideNil then return nil end
      if t.overrideReject then return false end
      t.lastOverride = {record, ready, aim}
      return true
    end,
  }
  return t
end

do
  local v, t = fresh(), mock()
  check("default OFF does not touch native or glue", v.Sync(t.api) == "off" and #t.calls == 0 and t.global == nil)
  v.SetEnabled(true)
  check("enable through Glue", v.Sync(t.api) == "enabled" and v.rt.applied == true and t.enabled)
  check("already enabled never calls setter again", v.Sync(t.api) == "same" and #t.calls == 1)
  v.SetEnabled(false)
  check("disable through Glue", v.Sync(t.api) == "disabled" and v.rt.applied == false and not t.enabled)
end
do
  local v, t = fresh(), mock()
  t.mode = "false"
  v.SetEnabled(true)
  check("explicit false SetEnabled keeps enabled unacknowledged", v.Sync(t.api) == "enable-failed" and v.rt.applied ~= true)
  t.mode = "error"
  check("thrown SetEnabled keeps enabled unacknowledged", v.Sync(t.api) == "enable-failed" and v.rt.applied ~= true)
  t.mode = "ok"
  check("failed enable retries and recovers", v.Sync(t.api) == "enabled" and v.rt.applied == true)
  t.mode = "false"
  v.SetEnabled(false)
  t.globalResult = false
  check("failed disable retains last enabled acknowledgement", v.Sync(t.api) == "disable-failed" and v.rt.applied == true)
  t.mode = "ok"
  check("failed disable retries and recovers", v.Sync(t.api) == "disabled" and v.rt.applied == false)
end
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  t.glueMissing = true
  t.globalResult = false
  check("native-only REDscript false does not claim full Glue ownership", v.Sync(t.api) == "noglue" and v.rt.applied ~= true)
  t.globalResult = true
  check("confirmed global fallback is native-only, not Glue acknowledged",
    v.Sync(t.api) == "enabled-global" and v.rt.applied ~= true and v.rt.nativeOnly == true)
  t.glueMissing = false
  check("fresh Glue after native-only fallback reenables with inputs", v.Sync(t.api) == "enabled" and t.enabled)
  t.enabled = false
  check("fresh game session re-enables Glue", v.Sync(t.api) == "enabled" and t.enabled)
end
do
  local v, t = fresh(), mock()
  local aimLog = {}
  v.sink = function(line) aimLog[#aimLog + 1] = line end
  check("aim default true and no native call while motion never enabled", v.want.aimGameUpper and
    v.SetAimGameUpper(false, t.api) == false and #t.aim == 0)
  v.SetEnabled(true)
  check("saved aim false reaches native when motion starts", v.Sync(t.api) == "enabled" and
    #t.aim == 1 and t.aim[1] == false and v.rt.aimApplied == false)
  check("accepted aim choice is logged", table.concat(aimLog, "|"):find("mm.aimGameUpper %-> false %(native call accepted%)") ~= nil)
  t.aimReject = true
  v.SetAimGameUpper(true, t.api)
  check("aim native false rejection preserves unacknowledged true", #t.aim == 2 and v.rt.aimApplied == false)
  check("rejected aim choice is not logged as accepted", not table.concat(aimLog, "|"):find("mm.aimGameUpper %-> true %(native call accepted%)"))
  t.aimReject = false
  v.Sync(t.api)
  check("aim native false rejection retried", #t.aim == 3 and t.aim[3] == true and v.rt.aimApplied == true)
  t.aimError = true
  v.SetAimGameUpper(false, t.api)
  check("aim thrown error not acknowledged", #t.aim == 4 and v.rt.aimApplied == true)
  t.aimError = false
  v.Sync(t.api)
  check("aim thrown error retried", #t.aim == 5 and v.rt.aimApplied == false)
  v.SetAimGameUpper(true, t.api)
  check("aim reset true restores native", v.rt.aimApplied == true and t.aim[#t.aim] == true)
end
do
  local v = fresh()
  local defs = {}
  v.RegisterSettings({ RegisterSection = function() end,
    Register = function(d) defs[d.key] = d end })
  check("registered mm.motionDriver on, aimGameUpper off defaults (user defaults 2026-10-20)", defs["mm.motionDriver"].default == true
    and defs["mm.aimGameUpper"].default == false)
  check("registered mm.slide on by default", defs["mm.slide"] ~= nil and defs["mm.slide"].default == true)
  local pt = defs["mm.pelvisYawTrim"]
  check("registered mm.pelvisYawTrim: float -10..10 step 0.5, default 5, animation section, label <= 38 chars",
    pt ~= nil and pt.type == "float" and pt.default == 5 and pt.min == -10 and pt.max == 10 and pt.step == 0.5
    and pt.section == "animation" and #pt.label <= 38, pt and pt.label)
  local ws = defs["mm.walkSpeed"]
  check("registered mm.walkSpeed: float 0..3.5 step 0.1, default 1.6, animation section, label <= 38 chars, desc names the walk toggle",
    ws ~= nil and ws.type == "float" and ws.default == 1.6 and ws.min == 0 and ws.max == 3.5 and ws.step == 0.1
    and ws.section == "animation" and #ws.label <= 38 and type(ws.desc) == "string" and ws.desc:find("(G)", 1, true) ~= nil, ws and ws.label)
  local mw = defs["mm.minWalkSpeed"]
  check("registered mm.minWalkSpeed: float 0..1.6 step 0.01, default 0.82, animation section, label <= 38 chars",
    mw ~= nil and mw.type == "float" and mw.default == 0.82 and mw.min == 0 and mw.max == 1.6 and mw.step == 0.01
    and mw.section == "animation" and #mw.label <= 38 and type(mw.desc) == "string", mw and mw.label)
  local bo = defs["mm.bodyYawOffset"]
  check("registered mm.bodyYawOffset: float -20..20 step 0.5, default 8, animation section, label 'Body yaw offset (deg)'",
    bo ~= nil and bo.type == "float" and bo.default == 8 and bo.min == -20 and bo.max == 20 and bo.step == 0.5
    and bo.section == "animation" and bo.label == "Body yaw offset (deg)" and type(bo.desc) == "string"
    and bo.desc:find("movement direction is unchanged", 1, true) ~= nil and bo.desc:find("Not while aiming", 1, true) ~= nil)
  check("registered mm.orientWarp on by default, bool, animation section",
    defs["mm.orientWarp"] ~= nil and defs["mm.orientWarp"].default == true and defs["mm.orientWarp"].type == "bool"
    and defs["mm.orientWarp"].section == "animation")
  check("registered mm.selectionGuards on by default, bool, motion section",
    defs["mm.selectionGuards"] ~= nil and defs["mm.selectionGuards"].default == true and defs["mm.selectionGuards"].type == "bool"
    and defs["mm.selectionGuards"].section == "motion")
  check("registered mm.lean on by default, bool, motion section",
    defs["mm.lean"] ~= nil and defs["mm.lean"].default == true and defs["mm.lean"].type == "bool"
    and defs["mm.lean"].section == "motion")
  check("registered mm.leanHead on by default, bool",
    defs["mm.leanHead"] ~= nil and defs["mm.leanHead"].default == true and defs["mm.leanHead"].type == "bool")
  check("registered mm.leanScale float 0.85 in 0..2",
    defs["mm.leanScale"] ~= nil and defs["mm.leanScale"].type == "float" and defs["mm.leanScale"].default == 0.85
    and defs["mm.leanScale"].min == 0 and defs["mm.leanScale"].max == 2)
  check("registered mm.leanHeadScale float 0.9 in 0..2 step 0.05, motion section",
    defs["mm.leanHeadScale"] ~= nil and defs["mm.leanHeadScale"].type == "float" and defs["mm.leanHeadScale"].default == 0.9
    and defs["mm.leanHeadScale"].min == 0 and defs["mm.leanHeadScale"].max == 2 and defs["mm.leanHeadScale"].step == 0.05
    and defs["mm.leanHeadScale"].section == "motion")
end
do
  local v, t = fresh(), mock()
  v.SetAimGameUpper(false, t.api)
  v.SetEnabled(true)
  check("session-A starts with aim false", v.Sync(t.api) == "enabled" and
    v.rt.aimApplied == false and #t.aim == 1 and t.aim[1] == false)
  -- Session B has a fresh disabled Glue and a native reset to its true default.
  t.enabled = false
  t.nativeAim = true
  t.api.setAimGameUpper = function(on)
    t.aim[#t.aim + 1] = on
    t.nativeAim = on
    return true
  end
  check("fresh session reapplies stored false after Glue activation",
    v.Sync(t.api) == "enabled" and #t.aim == 2 and t.aim[2] == false
    and t.nativeAim == false and v.rt.aimApplied == false)
  check("steady fresh session does not resend aim false",
    v.Sync(t.api) == "same" and #t.aim == 2)
end

-- mm.pelvisYawTrim (2026-10-30): DLL default 0 -> no native call until the value differs; clamped to -10..10, NaN -> 0
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("pelvisYawTrim default 0: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.trim == 0)
  check("pelvisYawTrim default 0: steady sync makes no call", v.Sync(t.api) == "same" and #t.trim == 0)
  v.SetPelvisYawTrim(5, t.api)
  check("pelvisYawTrim 5 is sent once", #t.trim == 1 and t.trim[1] == 5 and v.rt.pelvisYawTrimApplied == 5)
  check("steady pelvisYawTrim is not resent", v.Sync(t.api) == "same" and #t.trim == 1)
  v.SetPelvisYawTrim(25, t.api)
  check("pelvisYawTrim clamped to 10", #t.trim == 2 and t.trim[2] == 10)
  v.SetPelvisYawTrim(0 / 0, t.api)
  check("pelvisYawTrim NaN -> 0 (back to the DLL default, sent)", #t.trim == 3 and t.trim[3] == 0 and v.want.pelvisYawTrim == 0)
  t.trimReject = true
  v.SetPelvisYawTrim(-3, t.api)
  check("rejected pelvisYawTrim is never acknowledged", v.rt.pelvisYawTrimApplied == 0 and #t.trim == 4)
  t.trimReject = false
  v.Sync(t.api)
  check("rejected pelvisYawTrim retries", v.rt.pelvisYawTrimApplied == -3 and #t.trim == 5 and t.trim[5] == -3)
end
do
  local v, t = fresh(), mock()
  v.SetPelvisYawTrim(4.5, t.api)
  check("pelvisYawTrim while motion is off makes no native call", #t.trim == 0 and v.want.pelvisYawTrim == 4.5)
  v.SetEnabled(true)
  check("pelvisYawTrim is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.trim == 1 and t.trim[1] == 4.5)
end

-- mm.shoulderNarrow (2026-11-02, console per side since 2026-11-09): DLL default 2.5 cm -> no native call until the value
-- differs; clamped to 0..5 (2026-11-09: was 0..4), NaN -> 2.5
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("shoulderNarrow default 2.5: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.shoulder == 0)
  check("shoulderNarrow default 2.5: steady sync makes no call", v.Sync(t.api) == "same" and #t.shoulder == 0)
  v.SetShoulderNarrow(3, t.api)
  check("shoulderNarrow 3 is sent once", #t.shoulder == 1 and t.shoulder[1] == 3 and v.rt.shoulderNarrowApplied == 3)
  check("steady shoulderNarrow is not resent", v.Sync(t.api) == "same" and #t.shoulder == 1)
  v.SetShoulderNarrow(9, t.api)
  check("shoulderNarrow clamped to 5", #t.shoulder == 2 and t.shoulder[2] == 5)
  v.SetShoulderNarrow(-1, t.api)
  check("shoulderNarrow clamped to 0 (off)", #t.shoulder == 3 and t.shoulder[3] == 0)
  v.SetShoulderNarrow(0 / 0, t.api)
  check("shoulderNarrow NaN -> 2.5 (back to the DLL default, sent)", #t.shoulder == 4 and t.shoulder[4] == 2.5 and v.want.shoulderNarrow == 2.5)
  t.shoulderReject = true
  v.SetShoulderNarrow(2, t.api)
  check("rejected shoulderNarrow is never acknowledged", v.rt.shoulderNarrowApplied == 2.5 and #t.shoulder == 5)
  t.shoulderReject = false
  v.Sync(t.api)
  check("rejected shoulderNarrow retries", v.rt.shoulderNarrowApplied == 2 and #t.shoulder == 6 and t.shoulder[6] == 2)
end
do
  local v, t = fresh(), mock()
  v.SetShoulderNarrow(1.5, t.api)
  check("shoulderNarrow while motion is off makes no native call", #t.shoulder == 0 and v.want.shoulderNarrow == 1.5)
  v.SetEnabled(true)
  check("shoulderNarrow is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.shoulder == 1 and t.shoulder[1] == 1.5)
end

-- mm.shoulderWidth (2026-11-09, user K5): cm against the default look, both shoulders, -5..5 step 0.5, default 0;
-- narrowing per side = 2.5 - width / 2 (0 -> 2.5 = the DLL default: no native call, the pose stays byte-identical)
do
  local v = fresh()
  local f = v.ShoulderNarrowFromWidth
  check("shoulderWidth map: 0 -> 2.5, -5 -> 5, +5 -> 0, 1 -> 2, -1.5 -> 3.25, 0.5 -> 2.25",
    f(0) == 2.5 and f(-5) == 5 and f(5) == 0 and f(1) == 2 and f(-1.5) == 3.25 and f(0.5) == 2.25)
  check("shoulderWidth map: every 0.5 step of -5..5 lands on the 0.25 cm per side grid, inside 0..5", (function()
    for i = -10, 10 do
      local n = f(i * 0.5)
      if n ~= 2.5 - i * 0.25 or n < 0 or n > 5 then return false end
    end
    return true
  end)())
  check("shoulderWidth map: out of range clamped (99 -> 0, -99 -> 5)", f(99) == 0 and f(-99) == 5)
  check("shoulderWidth map: NaN / nil / text -> 2.5 (the default look)", f(0 / 0) == 2.5 and f(nil) == 2.5 and f("abc") == 2.5)
  check("shoulderWidth map: a float step away from 0 (+-1e-7, -0) -> exactly 2.5", f(1e-7) == 2.5 and f(-1e-7) == 2.5 and f(-0.0) == 2.5)
end
do
  local v, t = fresh(), mock()
  local defs = {}
  v.RegisterSettings({ RegisterSection = function() end, Register = function(d) defs[d.key] = d end })
  local sw = defs["mm.shoulderWidth"]
  check("registered mm.shoulderWidth: float -5..5 step 0.5, default 0, animation section, label 'Shoulder width (female V, cm)'",
    sw ~= nil and sw.type == "float" and sw.default == 0 and sw.min == -5 and sw.max == 5 and sw.step == 0.5
    and sw.section == "animation" and sw.label == "Shoulder width (female V, cm)" and #sw.label <= 38 and type(sw.apply) == "function")
  check("shoulderWidth description: says 0 = default look, male V unaffected, ASCII only",
    type(sw.desc) == "string" and sw.desc:find("0 (default)", 1, true) ~= nil and sw.desc:find("No effect on male V", 1, true) ~= nil
    and not sw.desc:find("[\128-\255]") and not sw.label:find("[\128-\255]"))
  check("the replaced key mm.shoulderNarrow is no longer registered", defs["mm.shoulderNarrow"] == nil)
  v.api = t.api
  sw.apply(sw.default) -- the core applies the shipped default
  v.SetEnabled(true)
  check("shoulderWidth default 0: motion on makes no shoulder call (DLL default kept, pose byte-identical)",
    v.Sync(t.api) == "enabled" and #t.shoulder == 0 and v.want.shoulderNarrow == 2.5)
  sw.apply(-5)
  check("shoulderWidth -5 sends 5 cm per side", #t.shoulder == 1 and t.shoulder[1] == 5)
  sw.apply(5)
  check("shoulderWidth +5 sends 0 (no narrowing)", #t.shoulder == 2 and t.shoulder[2] == 0)
  sw.apply(0)
  check("shoulderWidth back to 0 sends exactly 2.5 (the DLL default)", #t.shoulder == 3 and t.shoulder[3] == 2.5)
  check("steady shoulderWidth is not resent", v.Sync(t.api) == "same" and #t.shoulder == 3)
end

-- migration (utpp_core MIGRATED_KEYS): a stored mm.shoulderNarrow becomes mm.shoulderWidth = 5 - 2 * narrow, the old key is
-- dropped from the file; a stored new key wins
do
  local tmp = UTPP_MOTION_PATH .. ".shoulder-migration-test.tmp"
  local function run(lines)
    local fh = assert(io.open(tmp, "w"))
    fh:write("# test\n")
    for _, l in ipairs(lines) do fh:write(l .. "\n") end
    fh:close()
    local v, t, core = fresh(), mock(), dofile(UTPP_CORE_PATH)
    core.sink = function() end
    core.SETTINGS_FILE = tmp
    core.Load(tmp)
    v.api = t.api
    v.RegisterSettings(core)
    v.SetEnabled(true); v.Sync(t.api)
    core.Save(tmp)
    local fh2 = assert(io.open(tmp, "r"))
    local text = fh2:read("*a")
    fh2:close()
    os.remove(tmp)
    return core.Get("mm.shoulderWidth"), t, text
  end
  local w, t, text = run({ "mm.shoulderNarrow=2" })
  check("migration 2.0 cm per side -> width 1, 2.0 sent, file has width=1 and no old key",
    w == 1 and #t.shoulder == 1 and t.shoulder[1] == 2 and text:find("\nmm.shoulderWidth=1\n", 1, true) ~= nil
    and text:find("shoulderNarrow", 1, true) == nil)
  w, t = run({ "mm.shoulderNarrow=4" })
  check("migration 4 -> width -3 (4 cm per side sent)", w == -3 and #t.shoulder == 1 and t.shoulder[1] == 4)
  w, t = run({ "mm.shoulderNarrow=0" })
  check("migration 0 (off) -> width 5 (0 sent)", w == 5 and #t.shoulder == 1 and t.shoulder[1] == 0)
  w, t = run({ "mm.shoulderNarrow=2.5" })
  check("migration 2.5 (old default) -> width 0, no shoulder call", w == 0 and #t.shoulder == 0)
  w, t = run({ "mm.shoulderNarrow=2.25" })
  check("migration 2.25 -> width 0.5 (2.25 sent)", w == 0.5 and #t.shoulder == 1 and t.shoulder[1] == 2.25)
  w, t = run({ "mm.shoulderNarrow=3", "mm.shoulderWidth=2" })
  check("both keys stored: the new key wins (2 -> 1.5 cm per side)", w == 2 and #t.shoulder == 1 and t.shoulder[1] == 1.5)
  w, t = run({ "mm.shoulderWidth=-1", "mm.shoulderNarrow=0" })
  check("both keys stored, old key last: the new key still wins", w == -1 and #t.shoulder == 1 and t.shoulder[1] == 3)
  w, t, text = run({ "mm.shoulderNarrow=abc" })
  check("migration of an invalid value: default 0, no call, old key dropped", w == 0 and #t.shoulder == 0
    and text:find("shoulderNarrow", 1, true) == nil)
  w, t = run({ "mm.shoulderNarrow=10" })
  check("migration out of range (10 -> -15) clamped to -5 (5 cm per side)", w == -5 and #t.shoulder == 1 and t.shoulder[1] == 5)
  w, t = run({})
  check("no stored value: width 0, no call", w == 0 and #t.shoulder == 0)
end

-- mm.walkSpeed (2026-10-31): DLL default 0 (= off) -> no native call until the value differs; clamped to 0..3.5, NaN -> 0
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("walkSpeed default (DLL 0): enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.walk == 0)
  check("walkSpeed default: steady sync makes no call", v.Sync(t.api) == "same" and #t.walk == 0)
  v.SetWalkSpeed(2.0, t.api)
  check("walkSpeed 2.0 is sent once", #t.walk == 1 and t.walk[1] == 2.0 and v.rt.walkSpeedApplied == 2.0)
  check("steady walkSpeed is not resent", v.Sync(t.api) == "same" and #t.walk == 1)
  v.SetWalkSpeed(9, t.api)
  check("walkSpeed clamped to 3.5", #t.walk == 2 and t.walk[2] == 3.5)
  v.SetWalkSpeed(-1, t.api)
  check("walkSpeed below 0 -> 0 (off, sent)", #t.walk == 3 and t.walk[3] == 0 and v.want.walkSpeed == 0)
  v.SetWalkSpeed(0 / 0, t.api)
  check("walkSpeed NaN -> 0, already applied, not resent", #t.walk == 3 and v.want.walkSpeed == 0)
  t.walkReject = true
  v.SetWalkSpeed(1.5, t.api)
  check("rejected walkSpeed is never acknowledged", v.rt.walkSpeedApplied == 0 and #t.walk == 4)
  t.walkReject = false
  v.Sync(t.api)
  check("rejected walkSpeed retries", v.rt.walkSpeedApplied == 1.5 and #t.walk == 5 and t.walk[5] == 1.5)
  t.walkError = true
  v.SetWalkSpeed(2.5, t.api)
  check("thrown walkSpeed native is never acknowledged", v.rt.walkSpeedApplied == 1.5 and v.rt.errors >= 1)
end
do
  local v, t = fresh(), mock()
  v.SetWalkSpeed(2.0, t.api)
  check("walkSpeed while motion is off makes no native call", #t.walk == 0 and v.want.walkSpeed == 2.0)
  v.SetEnabled(true)
  check("walkSpeed is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.walk == 1 and t.walk[1] == 2.0)
  -- a fresh game session does not reset the DLL value (process lifetime), so a new glue session resends nothing
  t.enabled = false
  check("walkSpeed is not resent for a fresh glue session", v.Sync(t.api) == "enabled" and #t.walk == 1)
end

-- mm.minWalkSpeed (2026-10-31): DLL default 0 (= off) -> no native call until the value differs; clamped to 0..1.6, NaN -> 0
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("minAnalogWalk default (DLL 0): enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.minWalk == 0)
  v.SetMinAnalogWalk(0.82, t.api)
  check("minAnalogWalk 0.82 is sent once", #t.minWalk == 1 and t.minWalk[1] == 0.82 and v.rt.minAnalogWalkApplied == 0.82)
  check("steady minAnalogWalk is not resent", v.Sync(t.api) == "same" and #t.minWalk == 1)
  v.SetMinAnalogWalk(9, t.api)
  check("minAnalogWalk clamped to 1.6", #t.minWalk == 2 and t.minWalk[2] == 1.6)
  v.SetMinAnalogWalk(-1, t.api)
  check("minAnalogWalk below 0 -> 0 (off, sent)", #t.minWalk == 3 and t.minWalk[3] == 0 and v.want.minAnalogWalk == 0)
  v.SetMinAnalogWalk(0 / 0, t.api)
  check("minAnalogWalk NaN -> 0, already applied, not resent", #t.minWalk == 3 and v.want.minAnalogWalk == 0)
  t.minWalkReject = true
  v.SetMinAnalogWalk(1.0, t.api)
  check("rejected minAnalogWalk is never acknowledged", v.rt.minAnalogWalkApplied == 0 and #t.minWalk == 4)
  t.minWalkReject = false
  v.Sync(t.api)
  check("rejected minAnalogWalk retries", v.rt.minAnalogWalkApplied == 1.0 and #t.minWalk == 5 and t.minWalk[5] == 1.0)
  t.minWalkError = true
  v.SetMinAnalogWalk(0.5, t.api)
  check("thrown minAnalogWalk native is never acknowledged", v.rt.minAnalogWalkApplied == 1.0 and v.rt.errors >= 1)
end
do
  local v, t = fresh(), mock()
  v.SetMinAnalogWalk(0.82, t.api)
  check("minAnalogWalk while motion is off makes no native call", #t.minWalk == 0 and v.want.minAnalogWalk == 0.82)
  v.SetEnabled(true)
  check("minAnalogWalk is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.minWalk == 1 and t.minWalk[1] == 0.82)
end

-- mm.bodyYawOffset (2026-10-30): DLL default 0 -> no native call until the value differs; clamped to -20..20, NaN -> 0
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("bodyYawOffset default 0: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.bodyOffset == 0)
  check("bodyYawOffset default 0: steady sync makes no call", v.Sync(t.api) == "same" and #t.bodyOffset == 0)
  v.SetBodyYawOffset(8, t.api)
  check("bodyYawOffset 8 is sent once", #t.bodyOffset == 1 and t.bodyOffset[1] == 8 and v.rt.bodyYawOffsetApplied == 8)
  check("steady bodyYawOffset is not resent", v.Sync(t.api) == "same" and #t.bodyOffset == 1)
  v.SetBodyYawOffset(35, t.api)
  check("bodyYawOffset clamped to 20", #t.bodyOffset == 2 and t.bodyOffset[2] == 20)
  v.SetBodyYawOffset(-35, t.api)
  check("bodyYawOffset clamped to -20", #t.bodyOffset == 3 and t.bodyOffset[3] == -20)
  v.SetBodyYawOffset(0 / 0, t.api)
  check("bodyYawOffset NaN -> 0 (back to the DLL default, sent)", #t.bodyOffset == 4 and t.bodyOffset[4] == 0 and v.want.bodyYawOffset == 0)
  t.bodyOffsetReject = true
  v.SetBodyYawOffset(-3, t.api)
  check("rejected bodyYawOffset is never acknowledged", v.rt.bodyYawOffsetApplied == 0 and #t.bodyOffset == 5)
  t.bodyOffsetReject = false
  v.Sync(t.api)
  check("rejected bodyYawOffset retries", v.rt.bodyYawOffsetApplied == -3 and #t.bodyOffset == 6 and t.bodyOffset[6] == -3)
  check("bodyYawOffset never touches the pelvis trim native", #t.trim == 0)
end
do
  local v, t = fresh(), mock()
  v.SetBodyYawOffset(6.5, t.api)
  check("bodyYawOffset while motion is off makes no native call", #t.bodyOffset == 0 and v.want.bodyYawOffset == 6.5)
  v.SetEnabled(true)
  check("bodyYawOffset is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.bodyOffset == 1 and t.bodyOffset[1] == 6.5)
end

-- mm.aimAtCrosshair (aim pitch plan 2026-10-30): DLL loop (default off) + utpp_camera crosshair point, P only with an ACKNOWLEDGED loop
do
  local v, t = fresh(), mock()
  local defs = {}
  v.RegisterSettings({ RegisterSection = function() end, Register = function(d) defs[d.key] = d end })
  local bl = defs["mm.aimAtCrosshair"]
  check("registered mm.aimAtCrosshair: bool, default true (in-game step 0 2026-10-31), label 'Weapon aims at crosshair point'",
    bl ~= nil and bl.type == "bool" and bl.default == true and bl.label == "Weapon aims at crosshair point" and type(bl.desc) == "string"
    and type(bl.apply) == "function")
  v.api = t.api
  bl.apply(true) -- the user switches it on (default off); v2 is still off
  check("barrelLoop while motion is off: no native call, crosshair point off", #t.barrel == 0 and t.camOn == false
    and v.want.barrelLoop == true)
  v.SetEnabled(true)
  check("motion on: the loop is sent once, then the crosshair point goes on", v.Sync(t.api) == "enabled" and #t.barrel == 1
    and t.barrel[1] == true and v.rt.barrelLoopApplied == true and t.camOn == true and t.camCalls[#t.camCalls] == true)
  local nCam = #t.camCalls
  check("steady sync: no resend, the point stays on, no camera call", v.Sync(t.api) == "same" and #t.barrel == 1
    and t.camOn == true and #t.camCalls == nCam)
  v.SetBarrelLoop(false, t.api)
  check("setting off: the loop off is sent and the point goes off together", #t.barrel == 2 and t.barrel[2] == false
    and v.rt.barrelLoopApplied == false and t.camOn == false)
  t.barrelReject = true
  v.SetBarrelLoop(true, t.api)
  check("rejected loop: never acknowledged, the point stays off", #t.barrel == 3 and v.rt.barrelLoopApplied == false
    and t.camOn == false)
  t.barrelReject = false
  v.Sync(t.api)
  check("rejected loop retries; acknowledged -> the point goes on", #t.barrel == 4 and v.rt.barrelLoopApplied == true
    and t.camOn == true)
  v.SetEnabled(false)
  check("motion off: the point goes off (the DLL loop is idle without motion)", v.Sync(t.api) == "disabled" and t.camOn == false)
  v.SetEnabled(true)
  check("motion back on: the acknowledged loop is not resent, the point goes on again", v.Sync(t.api) == "enabled"
    and #t.barrel == 4 and t.camOn == true)
end
do
  local v, t = fresh(), mock()
  local defs = {}
  v.RegisterSettings({ RegisterSection = function() end, Register = function(d) defs[d.key] = d end })
  v.api = t.api
  defs["mm.aimAtCrosshair"].apply(defs["mm.aimAtCrosshair"].default) -- the core applies the shipped default
  v.SetEnabled(true)
  check("shipped default (on): motion on sends the loop once, the crosshair point goes on", v.Sync(t.api) == "enabled"
    and #t.barrel == 1 and t.barrel[1] == true and v.rt.barrelLoopApplied == true and t.camOn == true)
end
do
  local v, t = fresh(), mock()
  t.barrelError = true -- an older DLL without UTPPSetMotionBarrelLoop
  v.SetBarrelLoop(true, t.api)
  v.SetEnabled(true)
  local okS, res = pcall(v.Sync, t.api)
  check("older DLL (native missing): Sync never raises, the point stays off", okS and res == "enabled" and t.camOn == false
    and v.rt.barrelLoopApplied == false)
end
do
  local v, t = fresh(), mock()
  t.camMissing = true -- utpp_camera not loaded yet (CET load order) / not installed
  v.SetBarrelLoop(true, t.api)
  v.SetEnabled(true)
  check("camera module missing: the loop is sent, no point, no error", v.Sync(t.api) == "enabled" and #t.barrel == 1
    and v.rt.aimPointApplied == nil and t.camOn == false)
  t.camMissing = false
  check("camera module appears: the next Sync switches the point on", v.Sync(t.api) == "same" and t.camOn == true
    and #t.barrel == 1)
  t.camError = true
  v.SetBarrelLoop(false, t.api)
  check("camera error: counted, the point is not acknowledged off and is retried", v.rt.aimPointApplied == true and #t.barrel == 2)
  t.camError = false
  v.Sync(t.api)
  check("camera retry: the point goes off", t.camOn == false and v.rt.aimPointApplied == false)
end

-- mm.orientWarp: same contract as the slide - default off makes no native call (the DLL default is off); true is sent once after v2 is on
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("orientWarp default off: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.orient == 0)
  check("orientWarp default off: steady sync makes no call", v.Sync(t.api) == "same" and #t.orient == 0)
  v.SetOrientWarp(true, t.api)
  check("orientWarp on is sent once", #t.orient == 1 and t.orient[1] == true and v.rt.orientWarpApplied == true)
  check("steady orientWarp on is not resent", v.Sync(t.api) == "same" and #t.orient == 1)
  v.SetOrientWarp(false, t.api)
  check("orientWarp off is sent when it differs from the DLL", #t.orient == 2 and t.orient[2] == false and v.rt.orientWarpApplied == false)
end
do
  local v, t = fresh(), mock()
  v.SetOrientWarp(true, t.api)
  check("orientWarp on while motion is off makes no native call", #t.orient == 0 and v.want.orientWarp == true)
  v.SetEnabled(true)
  check("orientWarp on is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.orient == 1 and t.orient[1] == true)
  t.orientReject = true
  v.SetOrientWarp(false, t.api)
  check("rejected orientWarp call is never acknowledged", v.rt.orientWarpApplied == true and #t.orient == 2)
  t.orientReject = false
  v.Sync(t.api)
  check("rejected orientWarp call retries", v.rt.orientWarpApplied == false and #t.orient == 3 and t.orient[3] == false)
  t.orientError = true
  v.SetOrientWarp(true, t.api)
  check("thrown orientWarp call is never acknowledged", v.rt.orientWarpApplied == false)
  t.orientError = false
  v.Sync(t.api)
  check("thrown orientWarp call retries", v.rt.orientWarpApplied == true)
  local before = #t.orient
  v.SetEnabled(false)
  v.Sync(t.api)
  check("orientWarp setting alone never calls the native while motion is off", #t.orient == before)
end

-- mm.selectionGuards: the DLL default is ON - the default makes no native call; off is sent once after v2 is on, on again when it differs
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("selectionGuards default on: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.guards == 0)
  check("selectionGuards default on: steady sync makes no call", v.Sync(t.api) == "same" and #t.guards == 0)
  v.SetSelectionGuards(false, t.api)
  check("selectionGuards off is sent once", #t.guards == 1 and t.guards[1] == false and v.rt.selectionGuardsApplied == false)
  check("steady selectionGuards off is not resent", v.Sync(t.api) == "same" and #t.guards == 1)
  v.SetSelectionGuards(true, t.api)
  check("selectionGuards on is sent when it differs from the DLL", #t.guards == 2 and t.guards[2] == true and v.rt.selectionGuardsApplied == true)
end
do
  local v, t = fresh(), mock()
  v.SetSelectionGuards(false, t.api)
  check("selectionGuards off while motion is off makes no native call", #t.guards == 0 and v.want.selectionGuards == false)
  v.SetEnabled(true)
  check("selectionGuards off is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.guards == 1 and t.guards[1] == false)
  t.guardsReject = true
  v.SetSelectionGuards(true, t.api)
  check("rejected selectionGuards call is never acknowledged", v.rt.selectionGuardsApplied == false and #t.guards == 2)
  t.guardsReject = false
  t.guardsError = true
  v.Sync(t.api)
  check("thrown selectionGuards call is never acknowledged", v.rt.selectionGuardsApplied == false and #t.guards == 3)
  t.guardsError = false
  v.Sync(t.api)
  check("selectionGuards call retries until accepted", v.rt.selectionGuardsApplied == true and #t.guards == 4 and t.guards[4] == true)
end

-- mm.lean / leanHead / leanScale: same contract as the slide - the defaults (off / off / 1.0) make no native call at all
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("lean defaults: enabling motion makes no call", v.Sync(t.api) == "enabled" and #t.lean == 0 and #t.leanHead == 0 and #t.leanParams == 0)
  check("lean defaults: steady sync makes no call", v.Sync(t.api) == "same" and #t.lean == 0 and #t.leanHead == 0 and #t.leanParams == 0)
  v.SetLean(true, t.api)
  check("lean on is sent once", #t.lean == 1 and t.lean[1] == true and v.rt.leanApplied == true)
  check("steady lean on is not resent", v.Sync(t.api) == "same" and #t.lean == 1)
  v.SetLeanHead(true, t.api)
  check("leanHead on is sent once", #t.leanHead == 1 and t.leanHead[1] == true and v.rt.leanHeadApplied == true)
  v.SetLeanScale(0.6, t.api)
  check("leanScale 0.6 sends (0.6, 1.0) once (head amount is independent)", #t.leanParams == 1 and t.leanParams[1][1] == 0.6 and t.leanParams[1][2] == 1.0)
  check("steady leanScale is not resent", v.Sync(t.api) == "same" and #t.leanParams == 1)
  v.SetLeanScale(0 / 0, t.api)
  check("NaN leanScale falls back to 1.0 (resent as the default)", v.want.leanScale == 1.0 and #t.leanParams == 2 and t.leanParams[2][1] == 1.0 and t.leanParams[2][2] == 1.0)
  v.SetLeanScale(nil, t.api)
  check("nil leanScale stays 1.0 without a resend", v.want.leanScale == 1.0 and #t.leanParams == 2)
  v.SetLeanScale(7, t.api)
  check("leanScale 7 clamps to 2.0", v.want.leanScale == 2.0 and #t.leanParams == 3 and t.leanParams[3][1] == 2.0 and t.leanParams[3][2] == 1.0)
  v.SetLeanScale(-1, t.api)
  check("leanScale -1 clamps to 0", v.want.leanScale == 0 and #t.leanParams == 4 and t.leanParams[4][1] == 0 and t.leanParams[4][2] == 1.0)
  v.SetLean(false, t.api)
  check("lean off is sent when it differs from the DLL", #t.lean == 2 and t.lean[2] == false and v.rt.leanApplied == false)
end
-- mm.leanHeadScale: second argument of the single UTPPSetMotionLeanParams(scale, headScale) call; default 1.0 never inherits leanScale
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  v.Sync(t.api)
  check("leanHeadScale defaults: no params call", #t.leanParams == 0 and v.want.leanHeadScale == 1.0 and v.rt.leanHeadScaleApplied == 1.0)
  v.SetLeanHeadScale(0.5, t.api)
  check("leanHeadScale 0.5 sends (1.0, 0.5) once", #t.leanParams == 1 and t.leanParams[1][1] == 1.0 and t.leanParams[1][2] == 0.5 and v.rt.leanHeadScaleApplied == 0.5)
  check("steady leanHeadScale is not resent", v.Sync(t.api) == "same" and #t.leanParams == 1)
  v.SetLeanScale(0.6, t.api)
  check("leanScale 0.6 after head 0.5 sends (0.6, 0.5) in one call", #t.leanParams == 2 and t.leanParams[2][1] == 0.6 and t.leanParams[2][2] == 0.5)
  v.SetLeanHeadScale(3, t.api)
  check("leanHeadScale 3 clamps to 2.0 and keeps the torso scale", v.want.leanHeadScale == 2.0 and #t.leanParams == 3 and t.leanParams[3][1] == 0.6 and t.leanParams[3][2] == 2.0)
  v.SetLeanHeadScale(-1, t.api)
  check("leanHeadScale -1 clamps to 0", v.want.leanHeadScale == 0 and #t.leanParams == 4 and t.leanParams[4][2] == 0)
  v.SetLeanHeadScale(0 / 0, t.api)
  check("NaN leanHeadScale falls back to 1.0", v.want.leanHeadScale == 1.0 and #t.leanParams == 5 and t.leanParams[5][1] == 0.6 and t.leanParams[5][2] == 1.0)
  v.SetLeanHeadScale(nil, t.api)
  check("nil leanHeadScale stays 1.0 without a resend", v.want.leanHeadScale == 1.0 and #t.leanParams == 5)
  -- native range survives a re-arm: values wanted while v2 is off are applied when it comes on; applied values are not resent
  v.SetEnabled(false)
  v.Sync(t.api)
  v.SetLeanHeadScale(0.4, t.api)
  check("leanHeadScale while motion is off makes no native call", #t.leanParams == 5 and v.want.leanHeadScale == 0.4)
  v.SetEnabled(true)
  check("re-arm applies the pending leanHeadScale (0.6, 0.4)", v.Sync(t.api) ~= nil and #t.leanParams == 6 and t.leanParams[6][1] == 0.6 and t.leanParams[6][2] == 0.4)
  v.SetEnabled(false)
  v.SetEnabled(true)
  v.Sync(t.api)
  check("re-arm without a change does not resend the params", #t.leanParams == 6)
  t.leanParamsReject = true
  v.SetLeanHeadScale(0.9, t.api)
  check("rejected leanHeadScale is never acknowledged", v.rt.leanHeadScaleApplied == 0.4 and v.rt.leanScaleApplied == 0.6)
  t.leanParamsReject = false
  v.Sync(t.api)
  check("rejected leanHeadScale retries with both values", v.rt.leanHeadScaleApplied == 0.9 and v.rt.leanScaleApplied == 0.6 and t.leanParams[#t.leanParams][1] == 0.6 and t.leanParams[#t.leanParams][2] == 0.9)
end
do
  local v, t = fresh(), mock()
  v.SetLean(true, t.api)
  check("lean on while motion is off makes no native call", #t.lean == 0 and v.want.lean == true)
  v.SetEnabled(true)
  check("lean on is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.lean == 1 and t.lean[1] == true)
  t.leanReject = true
  v.SetLean(false, t.api)
  check("rejected lean call is never acknowledged", v.rt.leanApplied == true and #t.lean == 2)
  t.leanReject = false
  v.Sync(t.api)
  check("rejected lean call retries", v.rt.leanApplied == false and #t.lean == 3 and t.lean[3] == false)
  t.leanError = true
  v.SetLean(true, t.api)
  check("thrown lean call is never acknowledged", v.rt.leanApplied == false)
  t.leanError = false
  v.Sync(t.api)
  check("thrown lean call retries", v.rt.leanApplied == true)
  t.leanParamsReject = true
  v.SetLeanScale(0.5, t.api)
  check("rejected leanScale is never acknowledged", v.rt.leanScaleApplied == 1.0)
  t.leanParamsReject = false
  v.Sync(t.api)
  check("rejected leanScale retries", v.rt.leanScaleApplied == 0.5)
  local before = #t.lean
  v.SetEnabled(false)
  v.Sync(t.api)
  check("lean setting alone never calls the native while motion is off", #t.lean == before)
end

-- mm.slide: default off makes no native call at all (the DLL default is off); true is sent once after v2 is on
do
  local v, t = fresh(), mock()
  v.SetEnabled(true)
  check("slide default off: enabling motion makes no slide call", v.Sync(t.api) == "enabled" and #t.slide == 0)
  check("slide default off: steady sync makes no slide call", v.Sync(t.api) == "same" and #t.slide == 0)
  v.SetSlide(true, t.api)
  check("slide on is sent once", #t.slide == 1 and t.slide[1] == true and v.rt.slideApplied == true)
  check("steady slide on is not resent", v.Sync(t.api) == "same" and #t.slide == 1)
  v.SetSlide(false, t.api)
  check("slide off is sent when it differs from the DLL", #t.slide == 2 and t.slide[2] == false and v.rt.slideApplied == false)
end
do
  local v, t = fresh(), mock()
  v.SetSlide(true, t.api)
  check("slide on while motion is off makes no native call", #t.slide == 0 and v.want.slide == true)
  v.SetEnabled(true)
  check("slide on is applied when motion comes on", v.Sync(t.api) == "enabled" and #t.slide == 1 and t.slide[1] == true)
  t.slideReject = true
  v.SetSlide(false, t.api)
  check("rejected slide call is never acknowledged", v.rt.slideApplied == true and #t.slide == 2)
  t.slideReject = false
  v.Sync(t.api)
  check("rejected slide call retries", v.rt.slideApplied == false and #t.slide == 3 and t.slide[3] == false)
  t.slideError = true
  v.SetSlide(true, t.api)
  check("thrown slide call is never acknowledged", v.rt.slideApplied == false)
  t.slideError = false
  v.Sync(t.api)
  check("thrown slide call retries", v.rt.slideApplied == true)
end

do
  local v, t = fresh(), mock()
  v.SetPistolAuto(false, t.api)
  check("auto preference while off makes no Glue call", t.autoCalls == nil)
  v.SetEnabled(true)
  check("manual preference applied on enable", v.Sync(t.api) == "enabled" and t.auto == false and v.rt.pistolAutoApplied == false)
  local calls = t.autoCalls
  v.Sync(t.api)
  check("steady auto policy is not resent", t.autoCalls == calls)
  t.enabled = false
  v.Sync(t.api)
  check("fresh session reapplies manual preference", t.autoCalls == calls + 1 and t.auto == false)
  t.autoReject = true
  v.SetPistolAuto(true, t.api)
  check("rejected policy never acknowledged", v.rt.pistolAutoApplied == false)
  t.autoReject = false; t.autoError = true
  v.Sync(t.api)
  check("policy exception never acknowledged", v.rt.pistolAutoApplied == false)
  t.autoError = false
  v.Sync(t.api)
  check("failed policy retries successfully", v.rt.pistolAutoApplied == true and t.auto == true)
end

do
  local v, t = fresh(), mock()
  local core = dofile(UTPP_CORE_PATH)
  core.sink = function() end
  local tmp = UTPP_MOTION_PATH .. ".override-test.tmp"
  core.SETTINGS_FILE = tmp
  v.api = t.api
  v.RegisterSettings(core)
  core.Set("mm.motionDriver", false) -- mm.motionDriver is ON by default (user defaults 2026-10-20); this block tests the disabled path
  local record = "Items.Preset_Overture_Kerry"
  check("override invalid pose rejected before persistence", not v.SetWeaponPoseOverride(record, "wrong", "2h"))
  check("override unsafe record rejected", not v.SetWeaponPoseOverride("Items.bad\nrecord", "1h", "2h"))
  check("override setting saves while disabled without game call", v.SetWeaponPoseOverride(record, "1h", "2h") and not t.overrideCalls)
  v.SetEnabled(true); v.Sync(t.api)
  check("override transports distinct Ready/Aim codes", t.lastOverride[1] == record and t.lastOverride[2] == 2 and t.lastOverride[3] == 3)
  local calls = t.overrideCalls
  v.Sync(t.api)
  check("override acknowledgement prevents repeated calls", t.overrideCalls == calls)
  t.enabled = false; v.Sync(t.api)
  check("override restored to fresh Glue", t.overrideCalls == calls + 1)
  t.overrideReject = true
  v.SetWeaponPoseOverride(record, "2h", "1h")
  check("override rejected result does not update acknowledgement", v.overrideApplied[record] == "1h/2h")
  t.overrideReject = false; t.overrideNil = true; v.Sync(t.api)
  check("override nil result is not acknowledgement", v.overrideApplied[record] == "1h/2h")
  t.overrideNil = false; t.overrideError = true; v.Sync(t.api)
  check("override exception preserves previous acknowledgement", v.overrideApplied[record] == "1h/2h")
  t.overrideError = false; v.Sync(t.api)
  check("override retry succeeds", v.overrideApplied[record] == "2h/1h")
  local restored, core2 = fresh(), dofile(UTPP_CORE_PATH)
  core2.sink = function() end; core2.SETTINGS_FILE = tmp
  core2.Load(tmp); restored.RegisterSettings(core2)
  check("override real Core save/load restores both choices", restored.weaponOverrides[record].ready == "2h" and restored.weaponOverrides[record].aim == "1h")
  v.SetWeaponPoseOverride(record, "auto", "auto")
  check("override clear transports zero/zero", t.lastOverride[2] == 0 and t.lastOverride[3] == 0)
  os.remove(tmp)
end

do
  local v, core = fresh(), dofile(UTPP_CORE_PATH)
  core.sink = function() end; core.Save = function() return true end
  v.RegisterSettings(core)
  local filled = true
  for i = 1, 64 do filled = v.SetWeaponPoseOverride("Items.Test_" .. i, "1h", "2h") and filled end
  check("64 active record overrides allowed", filled)
  check("65th active record refused", not v.SetWeaponPoseOverride("Items.Test_65", "1h", "2h"))
  check("existing record can clear at capacity", v.SetWeaponPoseOverride("Items.Test_1", "auto", "auto"))
  check("cleared record releases capacity", v.SetWeaponPoseOverride("Items.Test_65", "1h", "2h"))
  check("reactivating existing cleared record cannot exceed capacity", not v.SetWeaponPoseOverride("Items.Test_1", "1h", "2h"))
  check("refused reactivation preserves cleared choices", v.weaponOverrides["Items.Test_1"].ready == "auto" and v.weaponOverrides["Items.Test_1"].aim == "auto")
end

do
  local v, t = fresh(), mock()
  local sent = {}
  t.api.setWeaponPoseOverride = function(record, ready, aim)
    sent[#sent + 1] = record
    return true
  end
  v.weaponOverrides = { ["Items.ZClear"] = {ready="auto",aim="auto"}, ["Items.ANew"] = {ready="1h",aim="2h"} }
  v.SetEnabled(true); v.Sync(t.api)
  check("pending clears precede additions regardless of record name", sent[1] == "Items.ZClear" and sent[2] == "Items.ANew")
end

-- "Diagnostic logs" (utpp_core core.diagnosticLogs): the session diagnostics are re-armed for every fresh session
do
  local v, t = fresh(), mock()
  local core = dofile(UTPP_CORE_PATH)
  core.sink = function() end; core.Save = function() return true end
  local nat = {}
  core.diag.api = {}
  for _, f in ipairs({ "setReloadDiag", "setVaultLog", "setFeetDiag" }) do
    core.diag.api[f] = function(on) nat[#nat + 1] = f .. "=" .. tostring(on) end
  end
  local reload, vault, feet = {}, {}, {}
  local g = t.api.glue()
  function g:SetReloadDiag(on) reload[#reload + 1] = on end
  t.api.setVehicleVaultLog = function(on) vault[#vault + 1] = on; return true end
  t.api.setFeetDiag = function(on) feet[#feet + 1] = on end
  v.api = t.api
  v.RegisterSettings(core)
  v.SetEnabled(true)
  check("diagnostic logs off: fresh session enabled", v.Sync(t.api) == "enabled")
  check("diagnostic logs off: fresh session arms no session diagnostic", #reload == 0 and #vault == 0 and #feet == 0 and #nat == 0)
  core.Set("core.diagnosticLogs", "until")
  check("diagnostic logs on: core switches the natives on at once", table.concat(nat, ",") == "setReloadDiag=true,setVaultLog=true,setFeetDiag=true")
  check("diagnostic logs on: mm.debug NOT held (no UTPP motion window)", core.Get("mm.debug") == false and v.want.debug ~= true)
  check("diagnostic logs on: same session does not re-arm", v.Sync(t.api) == "same" and #reload == 0 and #vault == 0 and #feet == 0)
  t.enabled = false -- new player entity: a fresh glue starts disabled
  local errors0 = v.rt.errors
  check("diagnostic logs on: fresh session enabled", v.Sync(t.api) == "enabled")
  check("diagnostic logs on: fresh session re-arms reload diagnostic, vehicle vault log and feet diagnostic",
    reload[1] == true and vault[1] == true and feet[1] == true and #reload == 1 and #vault == 1 and #feet == 1)
  t.api.setVehicleVaultLog, t.api.setFeetDiag = nil, nil
  t.enabled = false
  check("diagnostic logs on: missing session natives are skipped", v.Sync(t.api) == "enabled" and #reload == 2 and #vault == 1)
  t.api.setVehicleVaultLog = function() error("vault native missing") end
  t.api.setFeetDiag = function() error("feet native missing") end
  t.enabled = false
  local okS, res = pcall(v.Sync, t.api)
  check("diagnostic logs on: failing session natives never raise", okS and res == "enabled" and #reload == 3 and v.rt.errors == errors0)
  for i = #nat, 1, -1 do nat[i] = nil end
  core.Set("core.diagnosticLogs", "off")
  check("diagnostic logs off: natives off, reload diagnostic follows mm.trace (off)",
    table.concat(nat, ",") == "setReloadDiag=false,setVaultLog=false,setFeetDiag=false")
  check("diagnostic logs off: mm.debug still at its default", core.Get("mm.debug") == false and v.want.debug ~= true)
  t.api.setVehicleVaultLog = function(on) vault[#vault + 1] = on; return true end
  t.api.setFeetDiag = function(on) feet[#feet + 1] = on end
  t.enabled = false
  check("diagnostic logs off: fresh session re-arms nothing", v.Sync(t.api) == "enabled" and #reload == 3 and #vault == 1 and #feet == 1)
end


-- "Diagnostic logs" do not start the continuous helper-bone capture (UTPPBodyPoseTraceContinuous, about 250 MB per minute): neither core
-- nor a fresh v2 session calls an injected setBodyTrace, on or off
do
  local v, t = fresh(), mock()
  local core = dofile(UTPP_CORE_PATH)
  core.sink = function() end; core.Save = function() return true end
  local nat, body = {}, {}
  core.diag.api = {}
  for _, f in ipairs({ "setReloadDiag", "setVaultLog", "setFeetDiag" }) do
    core.diag.api[f] = function(on) nat[#nat + 1] = f .. "=" .. tostring(on) end
  end
  core.diag.api.setBodyTrace = function(on) body[#body + 1] = on end
  t.api.setBodyTrace = function(on) body[#body + 1] = on; return true end
  v.api = t.api
  v.RegisterSettings(core)
  v.SetEnabled(true)
  check("body trace: diagnostic logs off: fresh session does not touch the native", v.Sync(t.api) == "enabled" and #body == 0 and #nat == 0)
  core.Set("core.diagnosticLogs", "until")
  check("body trace: diagnostic logs on: core switches on the three session diagnostics, not the body trace",
    table.concat(nat, ",") == "setReloadDiag=true,setVaultLog=true,setFeetDiag=true" and #body == 0)
  t.enabled = false -- new player entity
  check("body trace: diagnostic logs on: fresh session does not arm the body trace", v.Sync(t.api) == "enabled" and #body == 0)
  core.Set("core.diagnosticLogs", "off")
  check("body trace: diagnostic logs off: the body trace native is not called", #body == 0, table.concat(nat, ","))
end
print(string.format("utpp_motion CET test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_motion CET regression")

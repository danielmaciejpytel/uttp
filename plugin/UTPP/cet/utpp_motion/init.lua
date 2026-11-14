-- (new CET module, the motion set-port): switch and debug line for the motion motion matching driver.
--
-- Settings (registered through GetMod("utpp_core").Register, the same public API as utpp_bodyturn;
-- motion owns mm.aimGameUpper so core must not register that key a second time):
--   mm.motionDriver          bool, default true - motion on / off. Off and never switched on in this process: this module makes no
--                  game call at all, so the base path is untouched (the DLL starts with motion off); on:
--                  UTPPMotionGlue:SetEnabled(true) (scripts\utpp_motion.reds), which calls the native UTPPSetMotionEnabled and
--                  pushes body + overlay inputs; switched off again: SetEnabled(false) once.
--   mm.pistolAuto bool, default true - model-tag and Ready/Aim policy in Glue.
--   mm.pistol1h bool, default true - fallback for unknown/conflicting models, or manual pose when auto is off.
--   mm.debug    bool, default true - always draw the status window (it is also drawn while the CET overlay is open).
--   mm.aimGameUpper bool, default false (the DLL default is true: the first Sync sends false) - the motion set-motionfix item 2: while aiming with a weapon overlay the upper body comes
--                  from the game + the aim offset (native UTPPSetMotionAimGameUpper; the DLL default is the same, so no
--                  call is made until the value differs from it).
--   mm.slide    bool, default true - slide feed (sprint -> crouch slide, PSM LocomotionDetailed 5): the slide
--                  databases play instead of the held pose. Native UTPPSetMotionSlide; the DLL default is off, so no call is made
--                  until the value is true. It only takes effect with a bundle that carries the chooser.json "slide" section
--                  (the status line shows slideBundle=1); with any other bundle it is a harmless no-op.
--   mm.orientWarp bool, default true - orientation warp of the run circles: pelvis and torso are turned onto the path of motion
--                  instead of the outside of the turn (native UTPPSetMotionOrientWarp; the DLL default is off, so no call is made until
--                  the value is true). Loop clips on the ground only; aim / weapon / air / crouch / slide / traversal and straight
--                  running are untouched. Shown in Mods > UTPP > Animation.
--   mm.lean     bool, default true - Lean: V leans into the turn from the lateral acceleration while running / walking on the
--                  ground (native UTPPSetMotionLean; the DLL default is off, so no call is made until the value is true). It needs the
--                  pack motion\utpp_lean_<body>.bin next to the database (status: lean=<on>/<pack 1 = loaded>/...); without it nothing happens.
--   mm.leanHead bool, default true - the head layer replaces the head lead while the lean is on (native UTPPSetMotionLeanHead).
--   mm.shoulderWidth float -5 .. 5, step 0.5, default 0 ("Shoulder width (female V)" in Mod Settings > Camera) - female V
--                  shoulder width in cm against the current look (both shoulders together; - = narrower, + = wider). It drives the female
--                  shoulder narrowing of the DLL, in cm per side: narrow = 2.5 - width / 2 (0 -> 2.5 = the DLL default, so no call is made
--                  until it changes; -5 -> 5 cm per side, +5 -> 0 = no narrowing). Each upper-arm socket moves toward the spine; hands
--                  and weapon stay exactly where the animation put them, and the amount is lowered smoothly where an arm is nearly
--                  straight (native UTPPSetMotionShoulderNarrow). Male V: no effect (the narrowing is female only).
--                  Replaces mm.shoulderNarrow (cm per side 0 .. 4): utpp_core converts a stored value on load
--                  (width = 5 - 2 * narrow) and drops the old key. Console (per side, 0 .. 5): GetMod("utpp_motion").SetShoulderNarrow(cm).
--   mm.pelvisYawTrim float -10 .. 10, default 5 (cancel the ~5 deg right hip bias of the run clips; DLL default 0) - run pelvis yaw trim in degrees (+ = left): turns the
--                  pelvis while V walks / runs / sprints forward (the forward loops carry it ~5 deg to the right; +5 squares it),
--                  chest, head and feet keep their direction; not while aiming / weapon drawn (native UTPPSetMotionPelvisYawTrim).
--   mm.bodyYawOffset float -20 .. 20, default 8 (in-game look; DLL default 0) - body yaw offset in degrees (+ = left): turns the whole character
--                  (hips, torso, head, legs, feet) about its visual root against the over-the-shoulder camera parallax; the movement
--                  direction is unchanged; off while aiming / during weapon actions (native UTPPSetMotionBodyYawOffset).
--   mm.aimAtCrosshair bool, default true (DLL default off; in game: muzzle slot +Y = weapon entity +Y, barrel vs shot
--                  pitch error median 4.8 deg / p90 10.9 off -> 0.2-0.6 / p90 1.3 on, Kyubi + pistol) - "Weapon aims at crosshair point":
--                  the DLL barrel
--                  pitch loop of the motion aim offset (native UTPPSetMotionBarrelLoop) together with utpp_camera's crosshair point feed
--                  (SetAimPoint: the native aim / camera targets = the hit point P of the camera ray). P is switched on only while motion is on,
--                  the setting is on and the DLL acknowledged the loop (never P alone: open loop from the chest at a near P); re-checked
--                  every Sync (camera module loaded later, motion off -> P off, an older DLL without the native -> P off).
--   mm.walkSpeed float 0 .. 3.5, default 1.6 (m/s; the game walk point the motion speed map puts on the motion set Walk cap, DLL default 0 = off) - capsule speed at the full stick while
--                  the game walk toggle (G) is on (native UTPPSetMotionWalkSpeed; reds UTPPUpdateWalkSpeed holds the capsule there with one
--                  MaxSpeed multiplier = walkSpeed / 2.72, the speed the unmodified toggle gives in game 2.5; the motion driver maps it onto the
--                  motion set Walk cap). 0 = vanilla toggle speed. Third person, on the ground, motion driving only; sprint, aim walk, crouch, air: untouched.
--   mm.minWalkSpeed float 0 .. 1.6, default 0.82 (m/s at the stand run curve, the |mv| 0.455 point; DLL default 0 = off) - minimum capsule speed of
--                  a small stick tilt (native UTPPSetMotionMinAnalogWalk; reds UTPPUpdateAnalogFloor lifts the capsule with one MaxSpeed multiplier
--                  1 .. 4 = the fraction of the full-stick speed raised to this one, the motion driver gives the prediction the lifted cap). 0 = the
--                  game's own slow speeds. Third person, on the ground, stand / crouch, motion driving only; sprint, aim walk, air: untouched.
--   mm.leanScale float 0 .. 2, default 0.85 (DLL default 1) - amount of the lean on the torso (1 = full strength; first argument of native UTPPSetMotionLeanParams(scale, headScale)).
--   mm.leanHeadScale float 0 .. 2, default 0.9 (DLL default 1) - amount of the head lean (head layer only; second argument of UTPPSetMotionLeanParams; independent of leanScale).
--   mm.selectionGuards bool, default true (developer option; DLL default on, so no call is made at the default) - clip selection guards of the
--                  motion matcher as a group (native UTPPSetMotionSelectionGuards); off = every guard off, for A/B tests.
-- Debug line: UTPPMotionGlue:Status() = glue state + " | " + Game.UTPPGetMotionStatus() (enabled, body, db loaded, fallback
-- reason, last pick, feed p95 us), refreshed every STATUS_PERIOD s while shown.
-- Console: print(GetMod("utpp_motion").Status())   GetMod("utpp_motion").SetEnabled(true|false)
--
-- Game calls go through the scriptable system (Game.GetScriptableSystemsContainer():Get("UTPPMotionGlue"), the pattern of
-- utpp_body / utpp_bodyturn) with the script global Game.UTPPMotionGlueSetEnabled as fallback; every call is pcall-guarded.
-- The wanted state is re-applied when a new game session starts (the glue system of a fresh session starts disabled).

local M = { VERSION = "0.1.0" }

M.SYNC_PERIOD = 1.0    -- s between checks that the glue matches the settings (cheap; no per-frame game call) [P]
M.STATUS_PERIOD = 0.5  -- s between status reads while the debug window is shown [P]

-- wanted state (settings) and what the glue reported last
M.want = { enabled = false, pistol1h = false, pistolAuto = true, debug = false, aimGameUpper = true, slide = false, orientWarp = false,
  lean = false, leanHead = false, leanScale = 1.0, leanHeadScale = 1.0, pelvisYawTrim = 0, shoulderNarrow = 2.5,
  bodyYawOffset = 0, barrelLoop = false, walkSpeed = 0, minAnalogWalk = 0, selectionGuards = true }
M.rt = { sinceSync = 0, sinceStatus = 0, overlayOpen = false, status = "not read yet", applied = nil,
  nativeOnly = false, aimApplied = true, slideApplied = false, orientWarpApplied = false,
  leanApplied = false, leanHeadApplied = false, leanScaleApplied = 1.0, leanHeadScaleApplied = 1.0, pelvisYawTrimApplied = 0, shoulderNarrowApplied = 2.5,
  bodyYawOffsetApplied = 0, barrelLoopApplied = false, walkSpeedApplied = 0, minAnalogWalkApplied = 0, aimPointApplied = nil,
  pistolAutoApplied = nil, selectionGuardsApplied = true, errors = 0 }

-- no file system paths in the log: a Lua error text names the script file with its full path (user name included); a path in
-- the CET mods folder is cut to "mods\...", the game folder prefix is dropped, a user profile path becomes "~" (same rules as
-- utpp_core M.ScrubPaths)
local function scrubPaths(s)
  s = tostring(s)
  if not string.find(s, "%a:[\\/]") then return s end
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-([Mm]ods[\\/])", "%1")
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-Cyberpunk 2077[\\/]", "")
  s = string.gsub(s, "%a:[\\/][Uu]sers[\\/][^\\/%s\"']+", "~")
  return (string.gsub(s, "%a:[\\/][^%s\"']*", "<path>"))
end

local function log(msg)
  local line = "[utpp_motion] " .. tostring(msg)
  if M.sink then M.sink(line) return end
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(scrubPaths(line)) end) end
end
M.log = log

-- api = { glue() -> object|nil, setEnabledGlobal(on) -> bool, nativeStatus() -> string|nil, havePlayer() -> bool }
-- (offline tests pass a mock)
local function defaultApi()
  local api = {}
  api.glue = function()
    local ok, s = pcall(function() return Game.GetScriptableSystemsContainer():Get("UTPPMotionGlue") end)
    if ok then return s end
    return nil
  end
  api.setEnabledGlobal = function(on)
    -- true = the call went through (the glue or, without it, the native was switched)
    local ok, accepted = pcall(function() return Game.UTPPMotionGlueSetEnabled(on) end)
    -- REDscript returns false when Glue is missing, even though it called the native.
    -- A false return therefore does not acknowledge an enabled Glue session.
    return ok and accepted == true
  end
  api.nativeStatus = function()
    local ok, s = pcall(function() return Game.UTPPGetMotionStatus() end)
    if ok and s ~= nil then return tostring(s) end
    return nil
  end
  api.havePlayer = function()
    local ok, p = pcall(Game.GetPlayer)
    return ok and p ~= nil
  end
  -- session diagnostics re-armed while the diagnostic logs are on (utpp_core core.diagnosticLogs); errors raise into the caller's pcall
  api.setVehicleVaultLog = function(on) return Game.UTPPSetVehicleVaultLog(on) end
  api.setFeetDiag = function(on) return Game.UTPPSetMMFeetDiag(on) end
  return api
end
M.api = nil
M.weaponOverrides = {}
M.overrideApplied = {}

local function callGlue(api, method, ...)
  local g = api.glue and api.glue() or nil
  if g == nil then return false, nil end
  local args = { ... }
  local unpackf = table.unpack or unpack -- CET = LuaJIT (5.1 unpack); 5.2+ table.unpack
  local ok, r = pcall(function() return g[method](g, unpackf(args)) end)
  if not ok or (method ~= "IsEnabled" and r == false) then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log(method .. " failed: " .. tostring(r)) end
    return false, nil
  end
  return true, r
end

-- Brings the glue in line with M.want. Returns a short result string (for the log / tests).
-- M.rt.applied = what this module last told the DLL (process lifetime: the DLL flag survives a session change).
-- Off and never switched on in this process: no game call at all (base untouched).
function M.Sync(api)
  api = api or M.api or defaultApi()
  M.api = api
  if not M.want.enabled then
    if M.rt.aimPointApplied == true then M.ApplyAimPoint(api, false) end -- the crosshair point never without the motion barrel loop
    if M.rt.applied ~= true then return "off" end
    -- switch off: through the glue when it exists, else the script global (which falls back to the native)
    local ok = callGlue(api, "SetEnabled", false)
    if not ok and api.setEnabledGlobal then
      local okG, accepted = pcall(api.setEnabledGlobal, false)
      ok = okG and accepted == true
    end
    if not ok then return "disable-failed" end
    M.rt.applied = false
    M.rt.nativeOnly = false
    log("mm.motionDriver -> off")
    return "disabled"
  end
  if not (api.havePlayer and api.havePlayer()) then return "noplayer" end
  local okE, isOn = callGlue(api, "IsEnabled")
  if not okE then
    -- The global can switch a native without Glue, but it cannot supply overlay inputs.
    -- Never record this as an acknowledged, fully enabled motion session.
    if api.setEnabledGlobal then
      local okG, accepted = pcall(api.setEnabledGlobal, true)
      if okG and accepted == true then
        -- The global confirmed a native switch, but a Glue session is still unavailable.
        M.rt.nativeOnly = true
        return "enabled-global"
      end
    end
    return "noglue"
  end
  if isOn ~= true then -- off, or a fresh session's glue (starts disabled)
    callGlue(api, "SetPistolOneHand", M.want.pistol1h)
    M.rt.pistolAutoApplied = nil -- fresh Glue has its own policy state
    M.overrideApplied = {}
    M.ApplyPistolAuto(api)
    M.ApplyWeaponOverrides(api)
    -- The new session can reset the native aim flag to its default (true): forget the old acknowledgement before the settings
    -- are pushed. The settings go to the DLL BEFORE the glue enables it (these natives need only M.want.enabled, not an
    -- enabled glue), so its first enabled frame already runs with the wanted values and not with its own defaults. A failed
    -- enable returns below and the next Sync takes this whole branch again.
    M.rt.aimApplied = true
    M.ApplyAimGameUpper(api)
    M.ApplySlide(api)
    M.ApplyOrientWarp(api)
    M.ApplySelectionGuards(api)
    M.ApplyPelvisYawTrim(api)
    M.ApplyShoulderNarrow(api)
    M.ApplyBodyYawOffset(api)
    M.ApplyWalkSpeed(api)
    M.ApplyMinAnalogWalk(api)
    M.ApplyBarrelLoop(api)
    M.ApplyLean(api)
    local ok = callGlue(api, "SetEnabled", true)
    if not ok then return "enable-failed" end
    M.rt.applied = true
    M.rt.nativeOnly = false
    -- a new player entity (load / respawn) creates a fresh glue whose reload diagnostic starts off; with the motion
    -- matching trace on, the RELOAD episode lines must keep coming (a session change silently dropped them)
    local okT, traceOn = pcall(function() return M.core and M.core.Effective and M.core.Effective("mm.trace") end)
    -- the diagnostic logs ("On for 5 minutes" / "On until switched off", core.diagnosticLogs) also re-arm the other session
    -- diagnostics (vehicle vault log, feet diagnostic) of the new session; each call in pcall, an api without the field
    -- (offline harness) skips it
    local okA, mode = pcall(function() return M.core and M.core.Effective and M.core.Effective("core.diagnosticLogs") end)
    local allOn = okA and (mode == "timed" or mode == "until")
    if ((okT and traceOn == true) or allOn) and callGlue(api, "SetReloadDiag", true) then
      log("reload diagnostic re-armed (" .. (allOn and "diagnostic logs on" or "mm.trace is on") .. ")")
    end
    if allOn then
      local function arm(fn)
        if type(fn) ~= "function" then return false end
        local ok, r = pcall(fn, true)
        return ok and r ~= false -- UTPPSetVehicleVaultLog returns false while its system is missing; FeetDiag returns nothing
      end
      local okV, okF = arm(api.setVehicleVaultLog), arm(api.setFeetDiag)
      log(string.format("diagnostic logs: vehicle vault log %s, feet diagnostic %s", okV and "re-armed" or "unavailable",
        okF and "re-armed" or "unavailable"))
    end
    log("mm.motionDriver -> on")
    return "enabled"
  end
  M.rt.applied = true
  M.rt.nativeOnly = false
  M.ApplyPistolAuto(api)
  M.ApplyWeaponOverrides(api)
  M.ApplyAimGameUpper(api)
  M.ApplySlide(api)
  M.ApplyOrientWarp(api)
  M.ApplySelectionGuards(api)
  M.ApplyPelvisYawTrim(api)
  M.ApplyShoulderNarrow(api)
  M.ApplyBodyYawOffset(api)
  M.ApplyWalkSpeed(api)
  M.ApplyMinAnalogWalk(api)
  M.ApplyBarrelLoop(api)
  M.ApplyLean(api)
  return "same"
end

function M.SetEnabled(on)
  M.want.enabled = on == true
  M.rt.sinceSync = M.SYNC_PERIOD -- check on the next frame
  return M.want.enabled
end

function M.SetPistolOneHand(on)
  M.want.pistol1h = on == true
  if M.want.enabled then
    local api = M.api or defaultApi()
    if api.havePlayer and api.havePlayer() then callGlue(api, "SetPistolOneHand", M.want.pistol1h) end
  end
  return M.want.pistol1h
end

function M.ApplyPistolAuto(api)
  if not M.want.enabled then return false end
  if M.rt.pistolAutoApplied == M.want.pistolAuto then return true end
  api = api or M.api or defaultApi()
  local ok = callGlue(api, "SetPistolAuto", M.want.pistolAuto)
  if not ok then return false end
  M.rt.pistolAutoApplied = M.want.pistolAuto
  return true
end
function M.SetPistolAuto(on, api)
  M.want.pistolAuto = on == true
  M.ApplyPistolAuto(api)
  return M.want.pistolAuto
end

local poseCodes = { auto = 0, ["1h"] = 2, ["2h"] = 3 }
function M.ApplyWeaponOverrides(api)
  if not M.want.enabled then return false end
  api = api or M.api or defaultApi()
  local all = true
  local records = {}
  for record in pairs(M.weaponOverrides) do records[#records + 1] = record end
  table.sort(records, function(a, b)
    local pa, pb = M.weaponOverrides[a], M.weaponOverrides[b]
    local clearA, clearB = pa.ready == "auto" and pa.aim == "auto", pb.ready == "auto" and pb.aim == "auto"
    if clearA ~= clearB then return clearA end -- release native slots before pending additions
    return a < b
  end)
  for _, record in ipairs(records) do
    local poses = M.weaponOverrides[record]
    local signature = poses.ready .. "/" .. poses.aim
    if M.overrideApplied[record] ~= signature then
      local ok, accepted
      if api.setWeaponPoseOverride then
        ok, accepted = pcall(api.setWeaponPoseOverride, record, poseCodes[poses.ready], poseCodes[poses.aim])
      else
        local valid, id = pcall(function() return TweakDBID.new(record) end)
        if valid then ok, accepted = callGlue(api, "SetWeaponPoseOverride", id, poseCodes[poses.ready], poseCodes[poses.aim]) end
      end
      if ok and accepted == true then M.overrideApplied[record] = signature else all = false end
    end
  end
  return all
end

function M.RegisterWeaponOverride(core, record)
  if type(record) ~= "string" or #record > 128 or not record:match("^[%w_.]+$") then return false end
  if M.weaponOverrides[record] then return true end
  local count = 0
  for _, poses in pairs(M.weaponOverrides) do
    if poses.ready ~= "auto" or poses.aim ~= "auto" then count = count + 1 end
  end
  if count >= 64 then return false end
  M.weaponOverrides[record] = { ready = "auto", aim = "auto" }
  for _, state in ipairs({ "ready", "aim" }) do
    local stateKey = state
    core.Register({ key = "mm.weapon." .. record .. "." .. stateKey, section = "motion", type = "choice", default = "auto",
      label = record .. " / " .. stateKey, choices = {{"auto", "Automatic"}, {"1h", "Pistol 1H"}, {"2h", "Pistol 2H"}},
      apply = function(v) M.weaponOverrides[record][stateKey] = v; M.ApplyWeaponOverrides() end })
  end
  return true
end
function M.SetWeaponPoseOverride(record, ready, aim)
  if poseCodes[ready] == nil or poseCodes[aim] == nil or not M.core then return false end
  if ready ~= "auto" or aim ~= "auto" then
    local count = 0
    for id, poses in pairs(M.weaponOverrides) do
      if id ~= record and (poses.ready ~= "auto" or poses.aim ~= "auto") then count = count + 1 end
    end
    if count >= 64 then return false end
  end
  if not M.RegisterWeaponOverride(M.core, record) then return false end
  M.core.Set("mm.weapon." .. record .. ".ready", ready)
  M.core.Set("mm.weapon." .. record .. ".aim", aim)
  return true
end

-- The native defaults to true. The acknowledgement cache is independent of the stored preference.
function M.ApplyAimGameUpper(api)
  if not M.want.enabled then return false end
  local v = M.want.aimGameUpper
  if M.rt.aimApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setAimGameUpper then
    ok, accepted = pcall(api.setAimGameUpper, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionAimGameUpper(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.aimGameUpper: native call failed") end
    return false
  end
  M.rt.aimApplied = v
  log("mm.aimGameUpper -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetAimGameUpper(on, api)
  local v = on == true
  M.want.aimGameUpper = v
  M.ApplyAimGameUpper(api)
  return M.want.aimGameUpper
end

-- The native is a process-lifetime DLL flag that defaults to off: nothing is called while the wanted value equals what the DLL has.
function M.ApplySlide(api)
  if not M.want.enabled then return false end
  local v = M.want.slide
  if M.rt.slideApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setSlide then
    ok, accepted = pcall(api.setSlide, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionSlide(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.slide: native call failed") end
    return false
  end
  M.rt.slideApplied = v
  log("mm.slide -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetSlide(on, api)
  local v = on == true
  M.want.slide = v
  M.ApplySlide(api)
  return M.want.slide
end

-- Same contract as the slide: a process-lifetime DLL flag that defaults to off, nothing is called while the wanted value equals it.
function M.ApplyOrientWarp(api)
  if not M.want.enabled then return false end
  local v = M.want.orientWarp
  if M.rt.orientWarpApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setOrientWarp then
    ok, accepted = pcall(api.setOrientWarp, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionOrientWarp(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.orientWarp: native call failed") end
    return false
  end
  M.rt.orientWarpApplied = v
  log("mm.orientWarp -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetOrientWarp(on, api)
  local v = on == true
  M.want.orientWarp = v
  M.ApplyOrientWarp(api)
  return M.want.orientWarp
end

-- Selection churn guards as a group (developer option, A/B tests): a process-lifetime DLL flag that defaults to ON, so nothing is called
-- while the wanted value is on (the default).
function M.ApplySelectionGuards(api)
  if not M.want.enabled then return false end
  local v = M.want.selectionGuards
  if M.rt.selectionGuardsApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setSelectionGuards then
    ok, accepted = pcall(api.setSelectionGuards, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionSelectionGuards(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.selectionGuards: native call failed") end
    return false
  end
  M.rt.selectionGuardsApplied = v
  log("mm.selectionGuards -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetSelectionGuards(on, api)
  M.want.selectionGuards = on ~= false
  M.ApplySelectionGuards(api)
  return M.want.selectionGuards
end

-- Run pelvis yaw trim: a process-lifetime DLL value (default 0), same contract as the orientation warp: nothing is called while the
-- wanted value equals what the DLL holds. nil / non-numeric / NaN -> 0; otherwise clamped to -10 .. 10 (the native rejects more).
local function trimDeg(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 0 end
  if x < -10 then return -10 end
  if x > 10 then return 10 end
  return x
end
function M.ApplyPelvisYawTrim(api)
  if not M.want.enabled then return false end
  local v = M.want.pelvisYawTrim
  if M.rt.pelvisYawTrimApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setPelvisYawTrim then
    ok, accepted = pcall(api.setPelvisYawTrim, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionPelvisYawTrim(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.pelvisYawTrim: native call failed") end
    return false
  end
  M.rt.pelvisYawTrimApplied = v
  log("mm.pelvisYawTrim -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetPelvisYawTrim(v, api)
  M.want.pelvisYawTrim = trimDeg(v)
  M.ApplyPelvisYawTrim(api)
  return M.want.pelvisYawTrim
end

-- Female shoulder narrowing (cm per side): a process-lifetime DLL value (default 2.5 = MotionPose.hpp kFeminineShoulderAmount), same contract
-- as the pelvis trim: nothing is called while the wanted value equals what the DLL holds. nil / non-numeric / NaN -> 2.5; otherwise
-- clamped to 0 .. 5 (the setting mm.shoulderWidth -5 .. 5 maps onto it; the native rejects more).
M.SHOULDER_DEFAULT_CM = 2.5 -- cm per side at shoulderWidth 0 (= MotionPose.hpp kFeminineShoulderAmount, the DLL default)
M.SHOULDER_MAX_CM = 5       -- cm per side at shoulderWidth -5
local function shoulderCm(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return M.SHOULDER_DEFAULT_CM end
  if x < 0 then return 0 end
  if x > M.SHOULDER_MAX_CM then return M.SHOULDER_MAX_CM end
  return x
end
-- Shoulder width setting (cm, both shoulders together, - = narrower) -> narrowing in cm per side. nil / non-numeric / NaN -> 0
-- (the current look); clamped to -5 .. 5 and snapped to 0.001 cm, so a slider value a float step away from 0 still sends nothing
-- (exactly 2.5 = the DLL default: the pose stays byte-identical).
local function shoulderWidthCm(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 0 end
  if x < -5 then x = -5 elseif x > 5 then x = 5 end
  x = math.floor(x * 1000 + 0.5) / 1000
  if x == 0 then x = 0 end -- -0 -> 0
  return x
end
function M.ShoulderNarrowFromWidth(width)
  return M.SHOULDER_DEFAULT_CM - shoulderWidthCm(width) / 2
end
function M.ApplyShoulderNarrow(api)
  if not M.want.enabled then return false end
  local v = M.want.shoulderNarrow
  if M.rt.shoulderNarrowApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setShoulderNarrow then
    ok, accepted = pcall(api.setShoulderNarrow, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionShoulderNarrow(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.shoulderWidth: shoulder narrowing native call failed") end
    return false
  end
  M.rt.shoulderNarrowApplied = v
  log("mm.shoulderWidth: shoulder narrowing -> " .. tostring(v) .. " cm per side (native call accepted)")
  return true
end
function M.SetShoulderNarrow(v, api)
  M.want.shoulderNarrow = shoulderCm(v)
  M.ApplyShoulderNarrow(api)
  return M.want.shoulderNarrow
end
-- the Mod Settings option (mm.shoulderWidth); returns the narrowing in cm per side that is now wanted
function M.SetShoulderWidth(v, api)
  return M.SetShoulderNarrow(M.ShoulderNarrowFromWidth(v), api)
end

-- Body yaw offset: the same contract as the pelvis trim (DLL default 0, nothing called while the wanted value equals what the DLL
-- holds). nil / non-numeric / NaN -> 0; otherwise clamped to -20 .. 20 (the native rejects more).
local function bodyOffsetDeg(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 0 end
  if x < -20 then return -20 end
  if x > 20 then return 20 end
  return x
end
function M.ApplyBodyYawOffset(api)
  if not M.want.enabled then return false end
  local v = M.want.bodyYawOffset
  if M.rt.bodyYawOffsetApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setBodyYawOffset then
    ok, accepted = pcall(api.setBodyYawOffset, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionBodyYawOffset(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.bodyYawOffset: native call failed") end
    return false
  end
  M.rt.bodyYawOffsetApplied = v
  log("mm.bodyYawOffset -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetBodyYawOffset(v, api)
  M.want.bodyYawOffset = bodyOffsetDeg(v)
  M.ApplyBodyYawOffset(api)
  return M.want.bodyYawOffset
end

-- Walk toggle speed (mm.walkSpeed): the same contract as the trims (process-lifetime DLL value, default 0 = off, nothing called while
-- the wanted value equals what the DLL holds, only while motion is on). nil / non-numeric / NaN -> 0; otherwise clamped to 0 .. 3.5 (the native
-- rejects more).
local function walkSpeedMs(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 0 end
  if x < 0 then return 0 end
  if x > 3.5 then return 3.5 end
  return x
end
function M.ApplyWalkSpeed(api)
  if not M.want.enabled then return false end
  local v = M.want.walkSpeed
  if M.rt.walkSpeedApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setWalkSpeed then
    ok, accepted = pcall(api.setWalkSpeed, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionWalkSpeed(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.walkSpeed: native call failed") end
    return false
  end
  M.rt.walkSpeedApplied = v
  log("mm.walkSpeed -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetWalkSpeed(v, api)
  M.want.walkSpeed = walkSpeedMs(v)
  M.ApplyWalkSpeed(api)
  return M.want.walkSpeed
end

-- Minimum analog walk speed (mm.minWalkSpeed): the same contract as the walk speed (process-lifetime DLL value, default 0 = off,
-- nothing called while the wanted value equals what the DLL holds, only while motion is on). nil / non-numeric / NaN -> 0; otherwise clamped
-- to 0 .. 1.6 (the native rejects more).
local function minAnalogWalkMs(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 0 end
  if x < 0 then return 0 end
  if x > 1.6 then return 1.6 end
  return x
end
function M.ApplyMinAnalogWalk(api)
  if not M.want.enabled then return false end
  local v = M.want.minAnalogWalk
  if M.rt.minAnalogWalkApplied == v then return true end
  api = api or M.api
  local ok, accepted
  if api and api.setMinAnalogWalk then
    ok, accepted = pcall(api.setMinAnalogWalk, v)
  else
    ok, accepted = pcall(function() return Game.UTPPSetMotionMinAnalogWalk(v) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.minWalkSpeed: native call failed") end
    return false
  end
  M.rt.minAnalogWalkApplied = v
  log("mm.minWalkSpeed -> " .. tostring(v) .. " (native call accepted)")
  return true
end
function M.SetMinAnalogWalk(v, api)
  M.want.minAnalogWalk = minAnalogWalkMs(v)
  M.ApplyMinAnalogWalk(api)
  return M.want.minAnalogWalk
end

-- Barrel pitch loop + crosshair point (mm.aimAtCrosshair). The DLL loop: the same contract as the trims (DLL default off, called only
-- while motion is on and the wanted value differs from what the DLL holds). The crosshair point of utpp_camera (CM.AIM_POINT, shipped
-- off) follows the ACKNOWLEDGED loop: on only with motion on + setting on + native accepted, off otherwise; recomputed on every call.
-- api.cameraModule() (offline tests) or GetMod("utpp_camera"); a missing / older camera module keeps P off and is retried next Sync.
local function cameraModule(api)
  if api and api.cameraModule then return api.cameraModule() end
  if type(GetMod) ~= "function" then return nil end
  local ok, m = pcall(GetMod, "utpp_camera")
  if ok and type(m) == "table" then return m end
  return nil
end
function M.ApplyAimPoint(api, on)
  on = on == true
  if M.rt.aimPointApplied == on then return true end
  local cam = cameraModule(api or M.api)
  if cam == nil or type(cam.SetAimPoint) ~= "function" then return false end
  local ok, r = pcall(cam.SetAimPoint, on)
  if not ok or r ~= on then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.aimAtCrosshair: utpp_camera SetAimPoint failed: " .. tostring(r)) end
    return false
  end
  M.rt.aimPointApplied = on
  log("mm.aimAtCrosshair: crosshair aim point (utpp_camera) -> " .. (on and "on" or "off"))
  return true
end
function M.ApplyBarrelLoop(api)
  api = api or M.api
  local nativeOk = true
  if M.want.enabled and M.rt.barrelLoopApplied ~= M.want.barrelLoop then
    local v = M.want.barrelLoop
    local ok, accepted
    if api and api.setBarrelLoop then
      ok, accepted = pcall(api.setBarrelLoop, v)
    else
      ok, accepted = pcall(function() return Game.UTPPSetMotionBarrelLoop(v) end)
    end
    if not ok or accepted ~= true then
      nativeOk = false
      M.rt.errors = M.rt.errors + 1
      if M.rt.errors <= 3 then log("mm.aimAtCrosshair: native call failed") end
    else
      M.rt.barrelLoopApplied = v
      log("mm.aimAtCrosshair -> " .. tostring(v) .. " (native call accepted)")
    end
  end
  M.ApplyAimPoint(api, M.want.enabled and M.want.barrelLoop and nativeOk and M.rt.barrelLoopApplied == true)
  return nativeOk
end
function M.SetBarrelLoop(v, api)
  M.want.barrelLoop = v == true
  M.ApplyBarrelLoop(api)
  return M.want.barrelLoop
end

-- Lean: three process-lifetime DLL values with defaults (off / off / scale 1); nothing is called while the wanted value equals the DLL default.
local unpackArgs = table.unpack or unpack
local function leanCall(api, field, native, ...)
  local ok, accepted
  local args = { ... }
  if api and api[field] then
    ok, accepted = pcall(api[field], unpackArgs(args))
  else
    ok, accepted = pcall(function() return Game[native](unpackArgs(args)) end)
  end
  if not ok or accepted == false then
    M.rt.errors = M.rt.errors + 1
    if M.rt.errors <= 3 then log("mm.lean: native call " .. native .. " failed") end
    return false
  end
  return true
end
function M.ApplyLean(api)
  if not M.want.enabled then return false end
  api = api or M.api
  local all = true
  if M.rt.leanApplied ~= M.want.lean then
    if leanCall(api, "setLean", "UTPPSetMotionLean", M.want.lean) then
      M.rt.leanApplied = M.want.lean
      log("mm.lean -> " .. tostring(M.want.lean) .. " (native call accepted)")
    else all = false end
  end
  if M.rt.leanHeadApplied ~= M.want.leanHead then
    if leanCall(api, "setLeanHead", "UTPPSetMotionLeanHead", M.want.leanHead) then
      M.rt.leanHeadApplied = M.want.leanHead
      log("mm.leanHead -> " .. tostring(M.want.leanHead) .. " (native call accepted)")
    else all = false end
  end
  if M.rt.leanScaleApplied ~= M.want.leanScale or M.rt.leanHeadScaleApplied ~= M.want.leanHeadScale then
    -- one native call carries both amounts: (torso scale, head scale)
    if leanCall(api, "setLeanParams", "UTPPSetMotionLeanParams", M.want.leanScale, M.want.leanHeadScale) then
      M.rt.leanScaleApplied = M.want.leanScale
      M.rt.leanHeadScaleApplied = M.want.leanHeadScale
      log("mm.leanScale -> " .. tostring(M.want.leanScale) .. ", leanHeadScale -> " .. tostring(M.want.leanHeadScale) .. " (native call accepted)")
    else all = false end
  end
  return all
end
function M.SetLean(on, api)
  M.want.lean = on == true
  M.ApplyLean(api)
  return M.want.lean
end
function M.SetLeanHead(on, api)
  M.want.leanHead = on == true
  M.ApplyLean(api)
  return M.want.leanHead
end
-- nil / non-numeric / NaN -> 1.0 (the default); otherwise clamped to 0 .. 2 (the native rejects values outside that range)
local function leanAmount(v)
  local x = tonumber(v)
  if x == nil or x ~= x then return 1.0 end
  if x < 0 then return 0.0 end
  if x > 2 then return 2.0 end
  return x
end
function M.SetLeanScale(v, api)
  M.want.leanScale = leanAmount(v)
  M.ApplyLean(api)
  return M.want.leanScale
end
function M.SetLeanHeadScale(v, api)
  M.want.leanHeadScale = leanAmount(v)
  M.ApplyLean(api)
  return M.want.leanHeadScale
end

function M.Status(api)
  api = api or M.api or defaultApi()
  local ok, s = callGlue(api, "Status")
  if ok and s ~= nil then return tostring(s) end
  local n = api.nativeStatus and api.nativeStatus() or nil
  return "glue unavailable | " .. tostring(n or "native status unavailable")
end

-- Registers the UTPP-tab settings (utpp_core public API, as utpp_bodyturn.RegisterSettings). Offline tests pass a mock.
function M.RegisterSettings(core)
  M.core = core
  core.RegisterSection("motion", "Motion set (advanced)")
  core.Register({ key = "mm.motionDriver", section = "motion", type = "bool", default = true,
    label = "Player motion driver",
    desc = "Experimental motion driver with overlays (Rifle / Pistol / Injured) and the female / male V database. Off = the current motion matching (base) unchanged.",
    apply = function(v) M.SetEnabled(v == true) end })
  core.Register({ key = "mm.pistol1h", section = "motion", type = "bool", default = true,
    label = "motion: fallback / manual one-handed pistol pose",
    desc = "Used for unknown or conflicting model tags, or when automatic pistol poses are off.",
    apply = function(v) M.SetPistolOneHand(v == true) end })
  core.Register({ key = "mm.pistolAuto", section = "motion", type = "bool", default = true,
    label = "motion: automatic pistol poses",
    desc = "Recognizes weapon model tags and selects separate Ready/Aim poses; unknown models use the fallback.",
    apply = function(v) M.SetPistolAuto(v == true) end })
  core.Register({ key = "mm.aimGameUpper", section = "motion", type = "bool", default = false,
    label = "motion: game upper body while aiming",
    desc = "While aiming a gun, the upper body comes from the game with the aim correction (on target). Off = motion aim pose and motion aim offset (about 9 deg off target in the offline measurement).",
    apply = function(v) M.SetAimGameUpper(v == true) end })
  core.Register({ key = "mm.slide", section = "motion", type = "bool", default = true,
    label = "motion: sprint slide (experimental)",
    desc = "Plays the slide animations when V slides from a sprint. Needs a motion database with slide data (status: slideBundle=1); otherwise it does nothing.",
    apply = function(v) M.SetSlide(v == true) end })
  core.Register({ key = "mm.orientWarp", section = "animation", type = "bool", default = true,
    label = "motion: orientation warp (run circles, experimental)",
    desc = "While V runs a circle, turns the pelvis and torso onto the path of motion instead of the outside of the turn (orientation warping). Only running loops on the ground; aiming, weapons, air, crouch, slide and straight running are untouched.",
    apply = function(v) M.SetOrientWarp(v == true) end })
  -- replaces mm.shoulderNarrow (utpp_core converts a stored value on load); placed in the Camera
  -- category after the other body options (utpp_core LAYOUT)
  core.Register({ key = "mm.shoulderWidth", section = "animation", type = "float", default = 0, min = -5, max = 5, step = 0.5,
    label = "Shoulder width (female V, cm)",
    desc = "Width of female V's shoulders (both together) against the default look. 0 (default) = the default look, below 0 = "
      .. "narrower (down to -5), above 0 = wider (+5 = the body's own width). Her hands and weapon stay where the animation puts "
      .. "them; narrower shoulders bend the elbows a little more. No effect on male V.",
    apply = function(v) M.SetShoulderWidth(v) end })
  core.Register({ key = "mm.pelvisYawTrim", section = "animation", type = "float", default = 5, min = -10, max = 10, step = 0.5,
    label = "Run pelvis yaw trim (deg)",
    desc = "Turns V's hips around the vertical axis while she walks, runs or sprints forward (+ = to the left, - = to the right); her "
      .. "chest, head and feet keep their direction. The running animations carry the hips about 5 degrees to the right: about +5 "
      .. "squares them with the direction of travel. 0 (default) = the animations as authored. Not while aiming or with a weapon drawn.",
    apply = function(v) M.SetPelvisYawTrim(v) end })
  core.Register({ key = "mm.bodyYawOffset", section = "animation", type = "float", default = 8, min = -20, max = 20, step = 0.5,
    label = "Body yaw offset (deg)",
    desc = "Turns V's whole character around the vertical axis (+ = to the left, - = to the right): hips, chest, head, legs and "
      .. "feet together. With the camera over the right shoulder V can look turned to the right; a few degrees to the left "
      .. "squares her with the view (default 8). The movement direction is unchanged. 0 = off. Not while aiming or during weapon actions.",
    apply = function(v) M.SetBodyYawOffset(v) end })
  core.Register({ key = "mm.walkSpeed", section = "animation", type = "float", default = 1.6, min = 0, max = 3.5, step = 0.1,
    label = "Walk speed (m/s)",
    desc = "Speed of V's walk when the walk toggle (G) is on and the stick is fully pushed. The game's own walk toggle is faster than the "
      .. "walking animations (about 2.7 m/s); 1.6 matches them, so the feet do not slide. 0 = the game's own speed. Only on foot in "
      .. "third person; sprinting, aiming, crouching and jumping are unchanged.",
    apply = function(v) M.SetWalkSpeed(v) end })
  core.Register({ key = "mm.minWalkSpeed", section = "animation", type = "float", default = 0.82, min = 0, max = 1.6, step = 0.01,
    label = "Minimum walk speed (m/s)",
    desc = "Lowest speed of V's walk when the stick is only slightly tilted. The game moves her slower than any walking animation "
      .. "there, so her feet glided over the ground; 0.82 (default) is the speed of a half-tilted stick. 0 = the game's own speed. "
      .. "Only on foot in third person, standing or crouching; sprinting, aiming and jumping are unchanged.",
    apply = function(v) M.SetMinAnalogWalk(v) end })
  core.Register({ key = "mm.aimAtCrosshair", section = "animation", type = "bool", default = true,
    label = "Weapon aims at crosshair point",
    desc = "While aiming or shooting from the hip, V's weapon barrel is steered onto the point under the crosshair (also at close "
      .. "range). Off = the earlier aim pose, which can point a few degrees above or below the crosshair.",
    apply = function(v) M.SetBarrelLoop(v == true) end })
  core.Register({ key = "mm.lean", section = "motion", type = "bool", default = true,
    label = "motion: lean layer (experimental)",
    desc = "Lean: V leans into the turn from the lateral acceleration while running or walking on the ground. Needs the lean pack next to the motion database (status: lean=on/1/...). Aiming, weapons, air, crouch, slide and straight running are untouched.",
    apply = function(v) M.SetLean(v == true) end })
  core.Register({ key = "mm.leanHead", section = "motion", type = "bool", default = true,
    label = "motion: lean layer head turn",
    desc = "The head lean replaces the head lead while the lean layer is on (they are never combined).",
    apply = function(v) M.SetLeanHead(v == true) end })
  core.Register({ key = "mm.leanScale", section = "motion", type = "float", default = 0.85, min = 0, max = 2, step = 0.05,
    label = "motion: lean amount",
    desc = "Amount of the lean layer (1 = full strength, 0.6 = softer). Scales the torso lean; the head has its own amount.",
    apply = function(v) M.SetLeanScale(v) end })
  core.Register({ key = "mm.leanHeadScale", section = "motion", type = "float", default = 0.9, min = 0, max = 2, step = 0.05,
    label = "motion: lean head amount",
    desc = "Amount of the head lean (1 = full strength, applies to the head layer only; lean amount scales the torso).",
    apply = function(v) M.SetLeanHeadScale(v) end })
  core.Register({ key = "mm.selectionGuards", section = "motion", type = "bool", default = true,
    label = "motion: clip selection guards",
    desc = "Keeps the motion matcher from switching animation clips back and forth on tiny differences (loops, young starts, dodges, short stick dips). Off = every such guard off, for comparison tests.",
    apply = function(v) M.SetSelectionGuards(v ~= false) end })
  core.Register({ key = "mm.debug", section = "motion", type = "bool", default = false,
    label = "motion: debug status line",
    desc = "Shows the motion status line (body, database, fallback, last pick, feed time) on screen.",
    apply = function(v) M.want.debug = v == true end })
  -- Restore only well-formed persisted record choices; Core owns validation and storage.
  for key in pairs(core.settings and core.settings.stored or {}) do
    local record, state = key:match("^mm%.weapon%.([%w_.]+)%.(%a+)$")
    if record and (state == "ready" or state == "aim") then M.RegisterWeaponOverride(core, record) end
  end
end

-- Debug window / status polling: mm.debug, or the CET overlay once motion was switched on in this process.
-- Off and never switched on: no window and no game call even with the overlay open.
function M.ShowDebug()
  return M.want.debug or (M.rt.overlayOpen and (M.want.enabled or M.rt.applied ~= nil))
end

function M.OnUpdate(dt, api)
  local rt = M.rt
  rt.sinceSync = rt.sinceSync + (dt or 0)
  if rt.sinceSync >= M.SYNC_PERIOD then
    rt.sinceSync = 0
    M.Sync(api)
  end
  if M.ShowDebug() then
    rt.sinceStatus = rt.sinceStatus + (dt or 0)
    if rt.sinceStatus >= M.STATUS_PERIOD then
      rt.sinceStatus = 0
      local ok, s = pcall(M.Status, api)
      rt.status = ok and s or ("status error: " .. tostring(s))
    end
  end
end

function M.Draw()
  if not M.ShowDebug() or ImGui == nil then return end
  if ImGui.Begin("UTPP motion", ImGuiWindowFlags and ImGuiWindowFlags.AlwaysAutoResize or 0) then
    ImGui.Text("mm.motionDriver " .. (M.want.enabled and "on" or "off") .. (M.want.pistol1h and "  pistol1h" or ""))
    ImGui.Text(tostring(M.rt.status))
  end
  ImGui.End()
end

if type(registerForEvent) == "function" then
  registerForEvent("onInit", function()
    local okC, core = false, nil
    if type(GetMod) == "function" then okC, core = pcall(GetMod, "utpp_core") end
    if okC and type(core) == "table" and type(core.Register) == "function" then
      local okR, err = pcall(M.RegisterSettings, core)
      if not okR then log("settings registration FAILED: " .. tostring(err)) end
    else
      log("utpp_core not found - mm.motionDriver stays off (console: GetMod(\"utpp_motion\").SetEnabled(true))")
    end
  end)
  registerForEvent("onUpdate", function(dt)
    local ok, err = pcall(M.OnUpdate, dt)
    if not ok then
      M.rt.errors = M.rt.errors + 1
      if M.rt.errors <= 3 then log("onUpdate error: " .. tostring(err)) end
    end
  end)
  registerForEvent("onOverlayOpen", function() M.rt.overlayOpen = true; M.rt.sinceStatus = M.STATUS_PERIOD end)
  registerForEvent("onOverlayClose", function() M.rt.overlayOpen = false end)
  registerForEvent("onDraw", function() pcall(M.Draw) end)
end

return M

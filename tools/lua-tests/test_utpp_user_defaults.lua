-- User-chosen defaults (menu screenshots, 2026-10-20): every setting shown in the Mods > UTPP menu has the value the user chose as
-- its registration default, "Defaults" (utpp_core.ResetAll, the Native Settings restore callback) restores exactly those, a fresh
-- start (no stored settings) sends them to the natives (incl. mm.leanScale / leanHeadScale 0.85 / 0.90) and stored values
-- of existing users still win over the defaults. Real utpp_core + utpp_bodyturn + utpp_body + utpp_motion registry, mocked natives.
-- Section 6 covers all 167 statically registered keys of the whole reference settings file (also camera, perspective, loothead).
-- Section 7 registers utpp_vehiclecam into the core (vehicle.* keys, incl. vehicle.forceTppOnEnter and
-- vehicle.safeTeleportExit added 2026-10-22).
local passed, failed = 0, 0
local function check(name, ok, detail)
  if ok then passed = passed + 1; print("[PASS] utpp_user_defaults: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_user_defaults: " .. name .. (detail and (" -- " .. tostring(detail)) or "")) end
end
local function near(a, b) return type(a) == "number" and type(b) == "number" and math.abs(a - b) < 1e-9 end

local savedRegister, savedGame, savedGetMod = registerForEvent, Game, GetMod
local tmp = os.tmpname()

-- key -> expected default (implementation switches: ON)
local EXPECTED = {
  -- Body turn
  ["bodyturn.guardFix"] = true, ["bodyturn.aimSmooth"] = true, ["bodyturn.weaponTurn"] = true, ["bodyturn.turnRate"] = true,
  ["bodyturn.turnRateMax"] = 1.5, ["bodyturn.aimStrafe"] = true, ["bodyturn.aimStrafeUnarmed"] = true,
  ["bodyturn.aimChestAlign"] = true, ["bodyturn.aimJog"] = true, ["bodyturn.aimJogSpeed"] = 3.3,
  ["bodyturn.shooterMode"] = false, -- Shooter mode (2026-10-22): opt-in, OFF by default
  -- Body visibility
  ["body.impl"] = true, ["body.hideFpp"] = true, ["body.showTpp"] = true, ["body.shadowProxy"] = false, ["body.garments"] = true,
  ["body.headGuard"] = true, ["body.garmentOffsets"] = false, ["body.retry"] = true,
  -- Animation
  ["mm.shootTransfer"] = true, ["mm.aimUpperFade"] = true, ["mm.idleHold"] = true, ["mm.kneeFix"] = true,
  ["mm.kneeHold"] = true, ["mm.headLead"] = true, ["mm.air"] = "auto", ["mm.airProbe"] = true, ["mm.airSteer"] = true,
  ["mm.airRobust"] = true, ["mm.climbRelease"] = false, ["mm.database"] = "utpp_base",
  ["mm.orientWarp"] = true, ["mm.enabled"] = true, ["mm.trace"] = false,
  ["mm.footReplant"] = "crouch", ["mm.feedDump"] = false, ["mm.weaponUpper"] = false, ["mm.grip"] = true,
  ["mm.shinobiBonusReload"] = false, -- "Reload animation during Shinobi bonus" (2026-10-27): opt-in, OFF by default
  -- Animation v2
  ["mm.motionDriver"] = true, ["mm.pistol1h"] = true, ["mm.pistolAuto"] = true, ["mm.aimGameUpper"] = false,
  ["mm.slide"] = true, ["mm.lean"] = true, ["mm.leanHead"] = true, ["mm.leanScale"] = 0.85,
  ["mm.leanHeadScale"] = 0.9, ["mm.selectionGuards"] = true, ["mm.debug"] = false,
  -- Crosshair (2026-10-30): the UTPP crosshair is on by default
  ["crosshair.impl"] = true, ["crosshair.color"] = "standard",
}

local function newCore()
  registerForEvent = nil
  local core = dofile(UTPP_CORE_PATH)
  registerForEvent = savedRegister
  core.sink = function() end
  core.SETTINGS_FILE = tmp
  return core
end

-- Boots the whole registry the way the game does (core onInit natives + database list, then the other CET modules).
local function boot(stored, dbList)
  os.remove(tmp)
  local core = newCore()
  if stored then local f = io.open(tmp, "w"); f:write(stored); f:close(); core.Load(tmp) end
  local rec = { dbRequests = {}, leanParams = {}, strafeParams = {}, turnParams = {} }
  local function rec1(name, ret)
    rec[name] = {}
    return function(...) rec[name][#rec[name] + 1] = { ... }; if ret == nil then return true end return ret end
  end
  core.SetupMMSettings(rec1("enabled"), rec1("trace"), {
    setFootReplant = rec1("replant"), setFeedDump = rec1("feedDump"),
    setIdleHold = rec1("idleHold"), setAimUpperFade = rec1("aimUpperFade"), setWeaponUpper = rec1("weaponUpper"),
    setMotionGrip = rec1("grip"), setMotionShootTransfer = rec1("shootTransfer"), setMotionMagTap = rec1("magTap"), setKneeFix = rec1("kneeFix"),
    setHeadLead = rec1("headLead"), setAir = rec1("air"), setAirProbe = rec1("airProbe"), setAirSteer = rec1("airSteer"),
    setAirRobust = rec1("airRobust"), setClimbRelease = rec1("climbRelease") })
  core.SetupDatabaseSetting(dbList or "default;alt_db;utpp_base",
    function(n) rec.dbRequests[#rec.dbRequests + 1] = n; return true end)
  -- utpp_bodyturn natives
  Game = {
    UTPPSetMMTurnRate = function() end,
    UTPPSetMMTurnRateParams = function(min, max) rec.turnParams[#rec.turnParams + 1] = { min, max }; return true end,
    UTPPSetMMStrafe = function() end,
    UTPPSetMMStrafeParams = function(unarmed, rt, rw, hold, jog) rec.strafeParams[#rec.strafeParams + 1] = { unarmed, jog }; return true end,
    UTPPSetMMChestAlign = function() end,
  }
  local bt = dofile(UTPP_BODYTURN_PATH)
  bt.log = function() end
  bt.RegisterSettings(core)
  local body = dofile(UTPP_BODY_PATH)
  body.RegisterSettings(core)
  -- utpp_motion with a mocked Glue / natives
  local t = { enabled = false, aim = {}, slide = {}, orient = {}, lean = {}, leanHead = {}, leanParams = {} }
  local glue = {}
  function glue:IsEnabled() return t.enabled end
  function glue:SetPistolOneHand(on) t.pistol = on end
  function glue:SetPistolAuto(on) t.auto = on end
  function glue:SetEnabled(on) t.enabled = on end
  t.api = {
    glue = function() return glue end, havePlayer = function() return true end,
    setEnabledGlobal = function() return true end,
    setAimGameUpper = function(on) t.aim[#t.aim + 1] = on; return true end,
    setSlide = function(on) t.slide[#t.slide + 1] = on; return true end,
    setOrientWarp = function(on) t.orient[#t.orient + 1] = on; return true end,
    setSelectionGuards = function() return true end,
    setLean = function(on) t.lean[#t.lean + 1] = on; return true end,
    setLeanHead = function(on) t.leanHead[#t.leanHead + 1] = on; return true end,
    setLeanParams = function(s, h) t.leanParams[#t.leanParams + 1] = { s, h }; return true end,
    setWeaponPoseOverride = function() return true end,
  }
  local v2 = dofile(UTPP_MOTION_PATH)
  v2.sink = function() end
  v2.api = t.api
  v2.RegisterSettings(core)
  return core, rec, t, v2, bt
end

local function matches(def, got, want)
  if type(want) == "number" then return near(got, want) end
  return got == want
end

-- ---------------------------------------------------------------- 1. registration defaults and fresh values
do
  local core, rec, t, v2 = boot(nil)
  local bad = {}
  local count = 0
  for key, want in pairs(EXPECTED) do
    count = count + 1
    local def = core.settings.defs[key]
    if def == nil then bad[#bad + 1] = key .. " (not registered)"
    elseif not matches(def, def.default, want) then bad[#bad + 1] = key .. " default " .. tostring(def.default)
    elseif not matches(def, core.Get(key), want) then bad[#bad + 1] = key .. " fresh value " .. tostring(core.Get(key)) end
  end
  check("all " .. count .. " listed keys are registered with the reference default and start on it", #bad == 0, table.concat(bad, "; "))
  check("implementation switches are ON by default (body.impl, mm.enabled)",
    core.Effective("body.impl") == true and core.Effective("mm.enabled") == true)
  check("mm.leanScale / leanHeadScale defaults 0.85 / 0.90 stay inside 0..2",
    core.settings.defs["mm.leanScale"].min == 0 and core.settings.defs["mm.leanScale"].max == 2
    and core.settings.defs["mm.leanHeadScale"].max == 2)

  -- ---------------------------------------------------------------- 2. fresh start reaches the natives
  check("fresh start: core natives get the new defaults (headLead/airSteer on, trace/feedDump/weaponUpper/climbRelease off, shoot transfer on)",
    rec.trace[1][1] == false and rec.feedDump[1][1] == false and rec.headLead[1][1] == true and rec.airSteer[1][1] == true
    and rec.weaponUpper[1][1] == false and rec.climbRelease[1][1] == false and rec.shootTransfer[1][1] == true
    and rec.enabled[1][1] == true and rec.grip[1][1] == true)
  check("fresh start: Shinobi bonus reload animation stays off without a native call (DLL default off)", #rec.magTap == 0
    and core.Get("mm.shinobiBonusReload") == false)
  check("fresh start: database default utpp_base requested", rec.dbRequests[1] == "utpp_base" and core.Get("mm.database") == "utpp_base")
  local sp, tp = rec.strafeParams[#rec.strafeParams], rec.turnParams[#rec.turnParams]
  check("fresh start: bodyturn natives get unarmed strafe ON, jog 3.3 and turn clip max 1.5",
    sp ~= nil and sp[1] == true and near(sp[2], 3.3) and tp ~= nil and near(tp[2], 1.5))
  check("motion wanted state follows the defaults before the first sync", v2.want.enabled == true and v2.want.pistol1h == true
    and v2.want.aimGameUpper == false and v2.want.slide == true and v2.want.orientWarp == true and v2.want.lean == true
    and v2.want.leanHead == true and near(v2.want.leanScale, 0.85) and near(v2.want.leanHeadScale, 0.9) and v2.want.debug == false)
  local fresh = dofile(UTPP_MOTION_PATH)
  check("v2 module starts with the DLL's own values acknowledged (aim true, slide/orient/lean off, scales 1.0), so 0.85 / 0.90 differ and are sent",
    fresh.rt.aimApplied == true and fresh.rt.slideApplied == false and fresh.rt.orientWarpApplied == false
    and fresh.rt.leanApplied == false and fresh.rt.leanHeadApplied == false and fresh.rt.leanScaleApplied == 1.0
    and fresh.rt.leanHeadScaleApplied == 1.0)
  -- with mm.motionDriver ON by default the switches are pushed while the settings register (as for a stored mm.motionDriver=true), the sync adds the glue
  check("motion registration already sent the defaults (motion is on at registration)", t.aim[#t.aim] == false and t.slide[#t.slide] == true
    and t.orient[#t.orient] == true and t.lean[#t.lean] == true and t.leanHead[#t.leanHead] == true and v2.rt.aimApplied == false
    and v2.rt.slideApplied == true and v2.rt.orientWarpApplied == true and v2.rt.leanApplied == true and v2.rt.leanHeadApplied == true)
  check("motion first sync enables the glue with the one-handed fallback pose and automatic poses", v2.Sync(t.api) == "enabled"
    and t.enabled == true and t.pistol == true and t.auto == true)
  check("v2 fresh start: every native holds the default (aim false, slide/orient/lean/leanHead true)", t.aim[#t.aim] == false
    and t.slide[#t.slide] == true and t.orient[#t.orient] == true and t.lean[#t.lean] == true and t.leanHead[#t.leanHead] == true)
  -- registration order leanScale -> leanHeadScale: the first call carries (0.85, 1.0), the last one the final pair
  local lastLean = t.leanParams[#t.leanParams]
  check("motion fresh start: lean amounts reach the native as (0.85, 0.90) (final params call)", #t.leanParams >= 1 and #t.leanParams <= 2
    and near(lastLean[1], 0.85) and near(lastLean[2], 0.9)
    and near(v2.rt.leanScaleApplied, 0.85) and near(v2.rt.leanHeadScaleApplied, 0.9))
  local nAim, nSlide, nLean, nParams = #t.aim, #t.slide, #t.lean, #t.leanParams
  check("motion second sync is quiet (acknowledged values are not resent)", v2.Sync(t.api) == "same" and #t.leanParams == nParams
    and #t.aim == nAim and #t.slide == nSlide and #t.lean == nLean)

  -- ---------------------------------------------------------------- 3. "Defaults" (ResetAll) restores them, also for late-registered keys
  local changed = {}
  for key, want in pairs(EXPECTED) do
    local def = core.settings.defs[key]
    local other
    if def.type == "bool" then other = not want
    elseif def.type == "float" then other = near(want, def.min) and def.max or def.min
    elseif def.type == "impl" then other = "off"
    elseif def.type == "choice" then
      for _, c in ipairs(def.choices) do if c[1] ~= want then other = c[1]; break end end
    end
    if core.Set(key, other, { noSave = true }) ~= nil then changed[#changed + 1] = key end
  end
  check("every listed key could be moved off its default (guards the reset check)", #changed == count, #changed)
  check("moved values differ from the defaults", core.Get("mm.leanScale") ~= 0.85 and core.Get("mm.trace") == true
    and core.Get("bodyturn.turnRateMax") ~= 1.5 and core.Get("mm.database") ~= "utpp_base")
  -- the Native Settings restore callback is core.ResetAll (BuildNativeSettings: registerRestoreDefaultsCallback)
  check("ResetAll returns true", core.ResetAll() == true)
  local still = {}
  for key, want in pairs(EXPECTED) do
    if not matches(core.settings.defs[key], core.Get(key), want) then still[#still + 1] = key .. "=" .. tostring(core.Get(key)) end
  end
  check("Defaults restores the reference defaults for all listed keys (core, bodyturn, body and motion keys)", #still == 0, table.concat(still, "; "))
  check("Defaults: implementation switches effective ON again", core.Effective("body.impl") == true and core.Effective("mm.enabled") == true)
  local text = io.open(tmp):read("*a")
  check("Defaults persists the lean amounts and the database", text:find("mm.leanScale=0.85", 1, true) ~= nil
    and text:find("mm.leanHeadScale=0.9", 1, true) ~= nil and text:find("mm.database=utpp_base", 1, true) ~= nil)
  check("Defaults: database request is utpp_base", rec.dbRequests[#rec.dbRequests] == "utpp_base")
  v2.Sync(t.api)
  local lp = t.leanParams[#t.leanParams]
  check("Defaults: motion lean amounts end at (0.85, 0.90), switches at the reference defaults",
    near(lp[1], 0.85) and near(lp[2], 0.9) and t.aim[#t.aim] == false and t.slide[#t.slide] == true and t.lean[#t.lean] == true
    and t.leanHead[#t.leanHead] == true and t.orient[#t.orient] == true and v2.want.debug == false)
  check("Defaults: core natives end at the reference defaults", rec.trace[#rec.trace][1] == false and rec.feedDump[#rec.feedDump][1] == false
    and rec.weaponUpper[#rec.weaponUpper][1] == false and rec.climbRelease[#rec.climbRelease][1] == false
    and rec.headLead[#rec.headLead][1] == true and rec.airSteer[#rec.airSteer][1] == true)
end

-- ---------------------------------------------------------------- 4. stored values of existing users win over the new defaults
do
  local stored = "mm.motionDriver=false\nmm.leanScale=1\nmm.leanHeadScale=1.2\nmm.trace=true\nmm.feedDump=true\nmm.weaponUpper=true\n"
    .. "mm.climbRelease=true\nmm.headLead=false\nmm.airSteer=false\nbodyturn.turnRateMax=1.4\nbodyturn.aimStrafeUnarmed=false\n"
    .. "mm.database=alt_db\nmm.aimGameUpper=true\n"
  local core, rec, t, v2 = boot(stored)
  check("stored values keep winning over the new defaults", core.Get("mm.motionDriver") == false and near(core.Get("mm.leanScale"), 1)
    and near(core.Get("mm.leanHeadScale"), 1.2) and core.Get("mm.trace") == true and core.Get("mm.feedDump") == true
    and core.Get("mm.weaponUpper") == true and core.Get("mm.climbRelease") == true and core.Get("mm.headLead") == false
    and core.Get("mm.airSteer") == false and near(core.Get("bodyturn.turnRateMax"), 1.4)
    and core.Get("bodyturn.aimStrafeUnarmed") == false and core.Get("mm.database") == "alt_db"
    and core.Get("mm.aimGameUpper") == true)
  check("stored values are what the natives get at start", rec.trace[1][1] == true and rec.feedDump[1][1] == true
    and rec.weaponUpper[1][1] == true and rec.climbRelease[1][1] == true and rec.dbRequests[1] == "alt_db"
    and rec.strafeParams[#rec.strafeParams][1] == false and near(rec.turnParams[#rec.turnParams][2], 1.4))
  check("stored mm.motionDriver=false keeps motion untouched (no glue calls)", v2.Sync(t.api) == "off" and t.enabled == false and #t.leanParams == 0)
  core.Set("mm.motionDriver", true)
  v2.Sync(t.api)
  check("stored lean amounts (1.0, 1.2) are what motion sends", #t.leanParams == 1 and near(t.leanParams[1][1], 1.0) and near(t.leanParams[1][2], 1.2))
end

-- ---------------------------------------------------------------- 5. database default falls back when utpp_base is not installed
do
  local core, rec = boot(nil, "default;alt_db")
  check("database default is 'default' when utpp_base is not installed", core.Get("mm.database") == "default" and rec.dbRequests[1] == "default")
  local core2, rec2 = boot(nil, "alt_db;motion_plus")
  check("database default is the first entry when neither is installed", core2.Get("mm.database") == "alt_db" and rec2.dbRequests[1] == "alt_db")
end

-- ---------------------------------------------------------------- 6. every registered key of the reference settings file (2026-10-20)
-- The values below are the whole reference settings file (utpp_settings.txt after the camera restore) for the keys that utpp_core and
-- the CET modules register statically (the key bindings were removed 2026-10-30, their bind.* keys are retired; per-weapon
-- mm.weapon.* overrides are registered dynamically and have no default of their own). Each is the registration default, so "Defaults" restores it.
local USER_SETTINGS = {
  ["core.log"] = true, ["headLook.impl"] = true, ["headLook.whileWalking"] = true, ["headLook.faceCamera"] = true,
  ["headLook.lookAtLoot"] = true, ["headLook.maxYaw"] = 100, ["headLook.chest"] = 0.5, ["lootView.impl"] = true, ["crosshair.impl"] = true,
  ["crosshair.color"] = "standard",
  ["lootView.zoom"] = 0.9, ["lootView.radius"] = 1.8, ["lootRange.impl"] = true,
  ["lootRange.scale"] = 1, ["headGuard.impl"] = true, ["camera.impl"] = true, ["camera.startTpp"] = true,
  ["camera.body"] = true, ["camera.audio"] = true, ["camera.sens"] = 1, ["camera.mouseScale"] = 0, ["camera.invertX"] = false,
  ["camera.invertY"] = false, ["camera.idleOrbit"] = true, ["camera.pitchSource"] = "fpp", ["camera.pivotSource"] = "capsule",
  ["camera.followMaxRate"] = 240, ["camera.followHoldOnFlip"] = true, ["camera.aimReturn"] = true,
  ["camera.aimReturnBlend"] = 0.35, ["camera.aim.enabled"] = true, ["camera.decoupleLocomotion"] = true,
  ["camera.predictBodyTurn"] = false, ["camera.worldCamera"] = true, ["camera.worldCameraRetry"] = true,
  ["camera.trace"] = false, ["camera.aim.distance"] = 1.5, ["camera.aim.boomPitchScale"] = 0.35,
  ["camera.aim.boomFrame"] = 0.45, ["camera.zoomUnarmed.boomPitchScale"] = 0.25, ["camera.zoomUnarmed.boomFrame"] = 0.5,
  ["camera.aim.side"] = 0.5, ["camera.aim.zoomFov"] = false, ["camera.aim.ownYaw"] = true, ["camera.lookSync"] = false,
  ["camera.lookWrite"] = true, ["camera.aim.fov"] = 30, ["camera.aim.autoSwap"] = true, ["camera.aim.occlusion"] = true,
  ["camera.aim.range"] = 60, ["camera.zoomUnarmed.enabled"] = true, ["camera.zoomUnarmed.steps"] = 3,
  ["camera.zoomUnarmed.distance"] = 1.75, ["camera.zoomUnarmed.lastDistance"] = 0.6, ["camera.zoomUnarmed.fov"] = 45,
  ["camera.zoomUnarmed.lastFov"] = 25, ["camera.zoomUnarmed.fppHandoff"] = false, ["camera.collision"] = true,
  ["camera.enterTime"] = 0.45, ["camera.exitTime"] = 0.35, ["camera.poseNudge"] = true, ["camera.fppWithWeapon"] = true,
  ["camera.distance"] = 2.75, ["camera.height"] = -0.35, ["camera.side"] = 0.5,
  ["camera.fov"] = 55, ["camera.framingSens"] = 1, ["vehicle.keepTpp"] = true, ["vehicle.pullOut"] = true, ["vehicle.rideWatchdog"] = true,
  ["perspective.impl"] = true, ["perspective.startInLastView"] = true, ["perspective.defaultTpp"] = true,
  ["perspective.scene"] = true, ["perspective.sceneTier"] = 3, ["perspective.combat"] = "tpp",
  ["perspective.weaponDrawn"] = "tpp", ["perspective.aiming"] = "tpp", ["perspective.aimingScoped"] = "fpp",
  ["perspective.shortScopeFpp"] = false, ["perspective.meleeBlockTpp"] = true, ["perspective.vehicleKeepTpp"] = true,
  ["perspective.combatCut"] = false, ["perspective.combatExit"] = 1.25, ["perspective.takedown"] = false,
  ["perspective.workspot"] = false, ["perspective.interaction"] = false, ["perspective.interactionExit"] = 1,
  ["perspective.traversal"] = false, ["perspective.scanner"] = false, ["perspective.toggleOverrides"] = false,
  ["perspective.sceneExit"] = 1.5, ["perspective.animate"] = true, ["perspective.blend"] = 0.8, ["perspective.minGap"] = 0.5,
  ["perspective.log"] = true, ["perspective.userWantsTpp"] = true, ["mm.motionDriver"] = true, ["mm.pistol1h"] = true,
  ["mm.pistolAuto"] = true, ["mm.aimGameUpper"] = false, ["mm.slide"] = true, ["mm.orientWarp"] = true,
  ["mm.lean"] = true, ["mm.leanHead"] = true, ["mm.leanScale"] = 0.85, ["mm.leanHeadScale"] = 0.9,
  ["mm.selectionGuards"] = true, ["mm.debug"] = false, ["mm.enabled"] = true, ["mm.trace"] = false,
  ["mm.footReplant"] = "crouch", ["mm.feedDump"] = false, ["mm.weaponUpper"] = false, ["mm.grip"] = true,
  ["mm.shootTransfer"] = true, ["mm.aimUpperFade"] = true, ["mm.idleHold"] = true, ["mm.kneeFix"] = true,
  ["mm.kneeHold"] = true, ["mm.headLead"] = true, ["mm.air"] = "auto", ["mm.airProbe"] = true, ["mm.airSteer"] = true,
  ["mm.airRobust"] = true, ["mm.climbRelease"] = false, ["mm.database"] = "utpp_base", ["bodyturn.weaponTurn"] = true,
  ["bodyturn.turnRate"] = true, ["bodyturn.turnRateMax"] = 1.5, ["bodyturn.aimStrafe"] = true,
  ["bodyturn.aimStrafeUnarmed"] = true, ["bodyturn.aimChestAlign"] = true, ["bodyturn.aimJog"] = true,
  ["bodyturn.aimJogSpeed"] = 3.3, ["bodyturn.guardFix"] = true, ["bodyturn.aimSmooth"] = true, ["bodyturn.shooterMode"] = false,
  ["body.impl"] = true,
  ["body.hideFpp"] = true, ["body.showTpp"] = true, ["body.shadowProxy"] = false, ["body.garments"] = true,
  ["body.headGuard"] = true, ["body.garmentOffsets"] = false, ["body.retry"] = true,
}
-- registered by utpp_vehiclecam from its onInit with a live controller (not part of this harness; the three defaults are on)
local NOT_IN_HARNESS = { ["vehicle.keepTpp"] = true, ["vehicle.pullOut"] = true, ["vehicle.rideWatchdog"] = true }

do
  local core = boot(nil)
  local savedInput, savedGetMod, savedPath = registerInput, GetMod, package.path
  -- utpp_camera (profiles come from camera_math), utpp_perspective and utpp_loothead register into the same core
  package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. package.path
  registerForEvent, registerInput, GetMod = function() end, function() end, nil
  local okCam, cam = pcall(dofile, UTPP_CAMERA_PATH)
  if okCam then okCam = pcall(cam.RegisterSettings, core) end
  registerForEvent, registerInput = nil, nil
  local okPersp, persp = pcall(dofile, UTPP_PERSPECTIVE_PATH)
  if okPersp then okPersp = pcall(persp.AttachCore, core) end
  local handlers = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  GetMod = function(name) if name == "utpp_core" then return core end return nil end
  local okLoot = pcall(dofile, UTPP_LOOTHEAD_PATH)
  if okLoot and handlers.onInit then okLoot = pcall(handlers.onInit) end
  registerForEvent, registerInput, GetMod, package.path = savedRegister, savedInput, savedGetMod, savedPath
  check("camera, perspective and loothead settings registered into the core", okCam == true and okPersp == true and okLoot == true,
    tostring(okCam) .. "/" .. tostring(okPersp) .. "/" .. tostring(okLoot))

  local bad, count, keys = {}, 0, {}
  for key, want in pairs(USER_SETTINGS) do
    keys[#keys + 1] = key
    if not NOT_IN_HARNESS[key] then
      count = count + 1
      local def = core.settings.defs[key]
      if def == nil then bad[#bad + 1] = key .. " (not registered)"
      elseif not matches(def, def.default, want) then bad[#bad + 1] = key .. " default " .. tostring(def.default) .. " ~= " .. tostring(want)
      elseif not matches(def, core.Get(key), want) then bad[#bad + 1] = key .. " fresh value " .. tostring(core.Get(key)) end
    end
  end
  check("all " .. count .. " keys of the reference settings file are registered with that value as the default and start on it",
    #bad == 0 and count >= 130, table.concat(bad, "; "))

  -- 2026-10-30: no key binding options any more (own key-binding system removed)
  local binds = {}
  for _, key in ipairs(core.settings.order) do
    if core.settings.defs[key].type == "key" or string.sub(key, 1, 5) == "bind." then binds[#binds + 1] = key end
  end
  check("no key binding option is registered", #binds == 0, table.concat(binds, "; "))

  -- move every listed key off its default, then "Defaults" (ResetAll) must bring each one back to the file's value
  local moved, notMoved = 0, {}
  for _, key in ipairs(keys) do
    local def = core.settings.defs[key]
    if def and not NOT_IN_HARNESS[key] then
      local want, other = USER_SETTINGS[key], nil
      if def.type == "bool" then other = not want
      elseif def.type == "float" or def.type == "int" then other = near(want, def.min) and def.max or def.min
      elseif def.type == "choice" then
        for _, c in ipairs(def.choices) do if c[1] ~= want then other = c[1]; break end end
      end
      if other ~= nil and core.Set(key, other, { noSave = true }) ~= nil and not matches(def, core.Get(key), want) then moved = moved + 1
      else notMoved[#notMoved + 1] = key end
    end
  end
  check("every listed key could be moved off its default (guards the reset check)", #notMoved == 0 and moved == count, table.concat(notMoved, "; "))
  check("ResetAll returns true", core.ResetAll() == true)
  local still = {}
  for key, want in pairs(USER_SETTINGS) do
    if not NOT_IN_HARNESS[key] and not matches(core.settings.defs[key], core.Get(key), want) then
      still[#still + 1] = key .. "=" .. tostring(core.Get(key)) .. " (want " .. tostring(want) .. ")"
    end
  end
  check("Defaults restores every key of the reference settings file to its value", #still == 0, table.concat(still, "; "))
  local text = io.open(tmp):read("*a")
  check("Defaults persists the camera profile 2 and the aiming perspective",
    text:find("camera.distance=2.75", 1, true) ~= nil and text:find("camera.height=-0.35", 1, true) ~= nil
    and text:find("perspective.aiming=tpp", 1, true) ~= nil and text:find("camera.aim.fov=30", 1, true) ~= nil)
end

-- ---------------------------------------------------------------- 7. utpp_vehiclecam keys (vehicle batch 2026-10-22)
-- utpp_vehiclecam registers from its onInit (live controller): loaded here with utpp_core as GetMod("utpp_core"). New key
-- vehicle.forceTppOnEnter (default on: every vehicle entry starts in third person).
local VEHICLE_KEYS = { ["vehicle.keepTpp"] = true, ["vehicle.pullOut"] = true, ["vehicle.rideWatchdog"] = true,
                       ["vehicle.forceTppOnEnter"] = true, ["vehicle.safeTeleportExit"] = true, ["vehicle.jumpOutSidePush"] = true }
do
  os.remove(tmp)
  local core = newCore()
  local handlers = {}
  local sv = { reg = registerForEvent, getmod = GetMod, game = Game, print = print }
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  Game = setmetatable({ UTPPLogLine = function() end }, { __index = function() return function() return nil end end })
  print = function() end
  local okV, V = pcall(dofile, UTPP_VEHICLECAM_PATH)
  local ownFile = os.tmpname()
  if okV then V.SETTINGS_FILE = ownFile end
  GetMod = function(n) if n == "utpp_core" then return core end return nil end
  local okI = okV and handlers.onInit ~= nil and pcall(handlers.onInit)
  registerForEvent, GetMod, Game, print = sv.reg, sv.getmod, sv.game, sv.print
  check("utpp_vehiclecam registers its settings into the core", okV and okI, tostring(V))
  local bad = {}
  for key, want in pairs(VEHICLE_KEYS) do
    local def = core.settings.defs[key]
    if def == nil then bad[#bad + 1] = key .. " (not registered)"
    elseif def.type ~= "bool" or def.default ~= want or core.Get(key) ~= want then
      bad[#bad + 1] = key .. " default " .. tostring(def.default) .. " fresh " .. tostring(core.Get(key))
    end
  end
  check("vehicle keys registered with default on (incl. vehicle.forceTppOnEnter, vehicle.safeTeleportExit)", #bad == 0,
    table.concat(bad, "; "))
  if okV then
    core.Set("vehicle.safeTeleportExit", false, { noSave = true })
    local offOk = V.ctrl.cfg.safeTeleportExit == false and V.safeExit.pending == true
    check("vehicle.safeTeleportExit: core.Set reaches the controller (push pending), Defaults restores on", offOk
      and core.ResetAll() == true and V.ctrl.cfg.safeTeleportExit == true)
  end
  if okV then
    core.Set("vehicle.jumpOutSidePush", false, { noSave = true })
    local offOk = V.ctrl.cfg.jumpOutSidePush == false
    check("vehicle.jumpOutSidePush: core.Set reaches the controller, Defaults restores on", offOk and core.ResetAll() == true
      and V.ctrl.cfg.jumpOutSidePush == true and core.Get("vehicle.jumpOutSidePush") == true)
  end
  local def = core.settings.defs["vehicle.forceTppOnEnter"]
  check("vehicle.forceTppOnEnter label and controller default", def ~= nil
    and def.label == "Start vehicle rides in third person" and okV and V.ctrl.cfg.forceTppOnEnter == true)
  if okV and def then
    core.Set("vehicle.forceTppOnEnter", false, { noSave = true })
    local offOk = V.ctrl.cfg.forceTppOnEnter == false
    check("vehicle.forceTppOnEnter: core.Set reaches the controller, Defaults restores on", offOk and core.ResetAll() == true
      and V.ctrl.cfg.forceTppOnEnter == true and core.Get("vehicle.forceTppOnEnter") == true)
  end
  os.remove(ownFile)
end

os.remove(tmp)
registerForEvent, Game = savedRegister, savedGame
print(string.format("utpp_user_defaults test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_user_defaults test failed")

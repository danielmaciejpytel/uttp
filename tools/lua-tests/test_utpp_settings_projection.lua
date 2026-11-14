-- utpp_core settings projection (menu layout) offline tests. The simplified UTPP menu is a projection of the existing
-- settings: the registry (keys, types, defaults, ranges, choices, sections, registration order), the stored values, the
-- storage format, the order of apply callbacks and of the native calls must stay exactly as before; only the widgets differ.
--
-- T01 golden identity: the whole registry of the real CET modules (core, bodyturn, body, v2, camera, perspective, loothead,
--     vehiclecam) with mocked natives is compared with a golden captured after the 2026-10-30 settings change (7 categories,
--     own key bindings removed: no bind.* keys, the stored bind lines of the input are dropped as retired keys)
--     (fixtures/<GOLDEN_NAME>). Write mode (UTPP_PROJECTION_WRITE_GOLDEN = true or the environment variable
--     UTPP_PROJECTION_WRITE_GOLDEN=1) writes the golden from the current code instead of comparing; run it on the base only.
-- T02 layout / views / late registration / Native Settings contract (standard and "all" view, RegisterView / RefreshView).
-- Globals: UTPP_CORE_PATH, UTPP_BODYTURN_PATH, UTPP_BODY_PATH, UTPP_MOTION_PATH, UTPP_CAMERA_PATH, UTPP_CAMERA_DIR,
-- UTPP_PERSPECTIVE_PATH, UTPP_LOOTHEAD_PATH, UTPP_VEHICLECAM_PATH (run_lua_tests.py sets all of them); optional
-- UTPP_PROJECTION_GOLDEN (full path of the golden file).
local GOLDEN_NAME = "settings_registry_7cat_nobind.lua"

local passed, failed = 0, 0
local outPrint = print
local function check(name, ok, detail)
  if ok then passed = passed + 1 else failed = failed + 1 end
  outPrint(string.format("[%s] utpp_settings_projection: %s%s", ok and "PASS" or "FAIL", name,
    (not ok and detail ~= nil) and (" -- " .. tostring(detail)) or ""))
end

-- the runner shares one Lua state between suites: every global touched here is restored at the end
local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod,
                print = print, path = package.path }
local SAVED_OPEN = io.open
local function restoreGlobals()
  registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
  print, package.path, io.open = SAVED.print, SAVED.path, SAVED_OPEN
end

local WRITE = UTPP_PROJECTION_WRITE_GOLDEN == true or os.getenv("UTPP_PROJECTION_WRITE_GOLDEN") == "1"
local goldenPath = UTPP_PROJECTION_GOLDEN
if goldenPath == nil then
  local root = string.gsub(UTPP_CORE_PATH, "[/\\]plugin[/\\]UTPP[/\\]cet[/\\]utpp_core[/\\]init%.lua$", "")
  goldenPath = root .. "/tools/lua-tests/fixtures/" .. GOLDEN_NAME
end

local tmpSettings = os.tmpname()
-- CET mods open "utpp_settings.txt" relative to their own folder (core reads it while loading, utpp_vehiclecam reads and
-- writes its own copy); offline that would be the working directory. Those relative opens go to a temp file that is
-- deleted before every boot, so the capture never depends on files left by other runs.
local tmpModFile = os.tmpname()
local realOpen = io.open
local function sandboxIo()
  os.remove(tmpModFile)
  io.open = function(path, mode)
    if path == "utpp_settings.txt" then path = tmpModFile end
    return realOpen(path, mode)
  end
end
local function unsandboxIo() io.open = realOpen end

-- ---------------------------------------------------------------- recording of apply callbacks and native calls
local LOG = {}
local function ser(v)
  local t = type(v)
  if t == "number" then
    if v == math.floor(v) and math.abs(v) < 1e15 then return string.format("%d", v) end
    return string.format("%.17g", v)
  elseif t == "string" then return string.format("%q", v)
  elseif t == "boolean" or t == "nil" then return tostring(v) end
  return "<" .. t .. ">"
end
local function record(kind, name, ...)
  local parts = {}
  for i = 1, select("#", ...) do parts[i] = ser((select(i, ...))) end
  LOG[#LOG + 1] = kind .. " " .. name .. "(" .. table.concat(parts, ",") .. ")"
end
local function recorder(kind, name, ret)
  return function(...) record(kind, name, ...); if ret == nil then return true end return ret end
end
-- Game: every UTPP* native is a recorder returning true (the log line native is silent); anything else is absent
local function nativeGame()
  return setmetatable({}, { __index = function(_, name)
    if type(name) ~= "string" or string.sub(name, 1, 4) ~= "UTPP" then return nil end
    if name == "UTPPLogLine" then return function() end end
    return recorder("native", name)
  end })
end

-- apply callbacks: wrapped when registered (and once more after the boot for keys registered before the wrapper existed)
local function wrapApply(def)
  if type(def.apply) ~= "function" or def.__wrapped then return end
  local key, fn = def.key, def.apply
  def.apply = function(v)
    local ok, err = pcall(fn, v)
    record("apply", key, v, ok and "ok" or "error")
    if not ok then error(err, 0) end
  end
  def.__wrapped = true
end
local function hookRegister(core)
  local orig = core.Register
  core.Register = function(def)
    if type(def) == "table" then wrapApply(def) end
    return orig(def)
  end
end

local MM_API = { "setFootReplant", "setFeedDump", "setIdleHold", "setAimUpperFade", "setWeaponUpper",
  "setMotionGrip", "setMotionShootTransfer", "setMotionMagTap", "setKneeFix", "setHeadLead", "setAir", "setAirProbe", "setAirSteer", "setAirRobust",
  "setClimbRelease" }
local MOTION_API = { "setEnabledGlobal", "setAimGameUpper", "setSlide", "setOrientWarp", "setSelectionGuards", "setLean", "setLeanHead", "setLeanParams",
  "setWeaponPoseOverride" }

local coreLines = {}
local function loadCore()
  registerForEvent, registerInput = nil, nil
  sandboxIo()
  local core = dofile(UTPP_CORE_PATH)
  unsandboxIo()
  core.sink = function(line) coreLines[#coreLines + 1] = line end
  core.SETTINGS_FILE = tmpSettings
  return core
end

-- Boots the whole registry the way the game does: utpp_core (onInit natives + database list), then the other CET modules.
-- opts.skip = { v2 = true, vehiclecam = true, ... } leaves modules out (late registration tests add them afterwards).
local function bootModules(core, opts)
  opts = opts or {}
  local skip = opts.skip or {}
  local quiet = function() end
  print = quiet
  sandboxIo()
  Game = nativeGame()
  local api = {}
  for _, n in ipairs(MM_API) do api[n] = recorder("mm", n) end
  core.SetupMMSettings(recorder("mm", "setEnabled"), recorder("mm", "setTrace"), api)
  core.SetupDatabaseSetting("default;alt_db;utpp_base", recorder("db", "load"))
  local mods = {}
  if not skip.bodyturn then
    local bt = dofile(UTPP_BODYTURN_PATH)
    bt.log = quiet
    bt.RegisterSettings(core)
    mods.bodyturn = bt
  end
  if not skip.body then
    local body = dofile(UTPP_BODY_PATH)
    body.RegisterSettings(core)
    mods.body = body
  end
  local function loadMotion()
    local v2 = dofile(UTPP_MOTION_PATH)
    v2.sink = quiet
    local glueOn = false
    local glue = {}
    function glue:IsEnabled() return glueOn end
    function glue:SetEnabled(on) glueOn = on; record("glue", "SetEnabled", on) end
    function glue:SetPistolOneHand(on) record("glue", "SetPistolOneHand", on) end
    function glue:SetPistolAuto(on) record("glue", "SetPistolAuto", on) end
    local vapi = { glue = function() return glue end, havePlayer = function() return true end }
    for _, n in ipairs(MOTION_API) do vapi[n] = recorder("v2", n) end
    v2.api = vapi
    return v2
  end
  if not skip.v2 then
    mods.v2 = loadMotion()
    mods.v2.RegisterSettings(core)
  end
  mods.loadMotion = loadMotion
  if not skip.camera then
    package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. SAVED.path
    registerForEvent, registerInput, GetMod = function() end, function() end, nil
    local cam = dofile(UTPP_CAMERA_PATH)
    cam.RegisterSettings(core)
    mods.camera = cam
    package.path = SAVED.path
  end
  if not skip.perspective then
    registerForEvent, registerInput = nil, nil
    local persp = dofile(UTPP_PERSPECTIVE_PATH)
    persp.AttachCore(core)
    mods.perspective = persp
  end
  GetMod = function(name) if name == "utpp_core" then return core end return nil end
  if not skip.loothead then
    local handlers = {}
    registerForEvent = function(ev, fn) handlers[ev] = fn end
    dofile(UTPP_LOOTHEAD_PATH)
    if handlers.onInit then handlers.onInit() end
  end
  local function loadVehiclecam()
    local handlers = {}
    registerForEvent, registerInput = function(ev, fn) handlers[ev] = fn end, function() end
    GetMod = function(name) if name == "utpp_core" then return core end return nil end
    print = quiet
    local vc = dofile(UTPP_VEHICLECAM_PATH)
    if handlers.onInit then pcall(handlers.onInit) end
    return vc
  end
  if not skip.vehiclecam then mods.vehiclecam = loadVehiclecam() end
  mods.loadVehiclecam = loadVehiclecam
  restoreGlobals()
  for _, key in ipairs(core.settings.order) do wrapApply(core.settings.defs[key]) end
  return mods
end

-- ---------------------------------------------------------------- Native Settings mock (data model of Native Settings 1.96)
-- tab = { label, keys = { subPath ... }, subcategories = { [subPath] = { label, options } }, closed = cb };
-- optionalIndex inserts like table.insert, removeSubcategory leaves a nil in keys (as the real library does),
-- setOption calls the widget callback synchronously and returns early when the value is unchanged.
local function newNS(opts)
  opts = opts or {}
  local ns = { data = {}, errors = {}, setCalls = 0, restore = nil, liveCalls = 0, currentTab = nil, closing = false }
  local function split(path)
    local tab, sub = string.match(path, "^/([^/]+)/([^/]+)$")
    return tab, sub
  end
  function ns.pathExists(path)
    local tab, sub = split(path)
    if tab then return ns.data[tab] ~= nil and ns.data[tab].subcategories[sub] ~= nil end
    tab = string.match(path, "^/([^/]+)$")
    return tab ~= nil and ns.data[tab] ~= nil
  end
  -- Native Settings 1.96: while nativeSettings.currentTab is the tab, every add* / removeSubcategory takes the "tab is open"
  -- branch and spawns into the live controller; during RequestClose that controller is going away (queued scroll restores fail)
  local function live(path)
    local tab = split(path)
    if tab == nil then tab = string.match(path, "^/([^/]+)$") end
    if tab ~= nil and ns.currentTab == tab then
      ns.liveCalls = ns.liveCalls + 1
      if ns.closing then ns.errors[#ns.errors + 1] = "spawned into the closing controller " .. path end
    end
  end
  function ns.addTab(path, label, closed)
    ns.data[string.sub(path, 2)] = { label = label, keys = {}, subcategories = {}, closed = closed }
  end
  function ns.addSubcategory(path, label, idx)
    live(path)
    local tab, sub = split(path)
    local t = tab and ns.data[tab]
    if not t then ns.errors[#ns.errors + 1] = "addSubcategory without tab " .. path; return end
    if t.subcategories[sub] then ns.errors[#ns.errors + 1] = "duplicate subcategory " .. path end
    t.subcategories[sub] = { label = label, options = {} }
    local n = table.maxn(t.keys)
    if idx ~= nil and (idx < 1 or idx > n + 1) then ns.errors[#ns.errors + 1] = "subcategory index out of range " .. path .. " " .. idx end
    table.insert(t.keys, idx or n + 1, sub)
  end
  function ns.removeSubcategory(path)
    live(path)
    local tab, sub = split(path)
    local t = tab and ns.data[tab]
    if not t or not t.subcategories[sub] then return end
    t.subcategories[sub] = nil
    for i = 1, table.maxn(t.keys) do if t.keys[i] == sub then t.keys[i] = nil end end
  end
  local function add(kind, path, label, desc, fields, cb, idx)
    if opts.failLabel and label == opts.failLabel then error("mock widget failure: " .. label) end
    live(path)
    local tab, sub = split(path)
    local s = tab and ns.data[tab] and ns.data[tab].subcategories[sub]
    if not s then ns.errors[#ns.errors + 1] = "widget without subcategory " .. path .. " " .. tostring(label); return nil end
    local w = { kind = kind, path = path, label = label, desc = desc, cb = cb }
    for k, v in pairs(fields) do w[k] = v end
    if idx ~= nil and (type(idx) ~= "number" or idx < 1 or idx > #s.options + 1) then
      ns.errors[#ns.errors + 1] = "option index out of range " .. path .. " " .. tostring(label) .. " " .. tostring(idx)
      idx = nil
    end
    table.insert(s.options, idx or #s.options + 1, w)
    return w
  end
  function ns.addSwitch(path, label, desc, cur, dflt, cb, idx)
    return add("switch", path, label, desc, { state = cur, default = dflt }, cb, idx)
  end
  function ns.addRangeInt(path, label, desc, min, max, step, cur, dflt, cb, idx)
    return add("int", path, label, desc, { min = min, max = max, step = step, value = cur, default = dflt }, cb, idx)
  end
  function ns.addRangeFloat(path, label, desc, min, max, step, format, cur, dflt, cb, idx)
    return add("float", path, label, desc, { min = min, max = max, step = step, format = format, value = cur, default = dflt }, cb, idx)
  end
  function ns.addSelectorString(path, label, desc, elements, cur, dflt, cb, idx)
    return add("selector", path, label, desc, { elements = elements, value = cur, default = dflt }, cb, idx)
  end
  function ns.addButton(path, label, desc, text, size, cb, idx)
    return add("button", path, label, desc, { text = text }, cb, idx)
  end
  function ns.addKeyBinding(path, label, desc, value, dflt, hold, cb, idx)
    return add("key", path, label, desc, { value = value, default = dflt, hold = hold }, cb, idx)
  end
  function ns.registerRestoreDefaultsCallback(path, override, cb) ns.restore = { path = path, override = override, cb = cb } end
  -- current options in display order: { { sub, w } ... }
  function ns.list(tabName)
    local t, out = ns.data[tabName or "utpp"], {}
    if not t then return out end
    for i = 1, table.maxn(t.keys) do
      local sub = t.keys[i]
      if sub then for _, w in ipairs(t.subcategories[sub].options) do out[#out + 1] = { sub = sub, w = w } end end
    end
    return out
  end
  function ns.subs(tabName)
    local t, out = ns.data[tabName or "utpp"], {}
    if not t then return out end
    for i = 1, table.maxn(t.keys) do if t.keys[i] then out[#out + 1] = t.keys[i] end end
    return out
  end
  function ns.live(w)
    for _, e in ipairs(ns.list()) do if e.w == w then return true end end
    return false
  end
  function ns.setOption(w, v)
    ns.setCalls = ns.setCalls + 1
    if not ns.live(w) then ns.errors[#ns.errors + 1] = "setOption on a widget that is not in the tab: " .. tostring(w and w.label); return end
    if w.kind == "switch" then
      if type(v) ~= "boolean" or w.state == v then return end
      w.state = v
      w.cb(v)
    elseif w.kind == "int" or w.kind == "float" or w.kind == "selector" then
      if type(v) ~= "number" or w.value == v then return end
      w.value = v
      w.cb(v)
    end
  end
  -- the player changes a widget: the controller stores the value, then calls the callback (as Native Settings does)
  function ns.userChange(w, v)
    if w.kind == "switch" then w.state = v elseif w.kind ~= "button" then w.value = v end
    w.cb(v)
  end
  -- RequestClose: the callback runs while currentTab is still the tab; afterwards the controller is cleared (currentTab = nil).
  -- hold = true leaves the tab current (the player is still on the tab).
  function ns.closeTab(tabName, hold)
    local name = tabName or "utpp"
    local t = ns.data[name]
    ns.currentTab, ns.closing = name, true
    if t and t.closed then t.closed() end
    ns.closing = false
    if not hold then ns.currentTab = nil end
  end
  return ns
end

local function widgetKeys(core)
  local inv = {}
  for key, w in pairs(core.settings.widgets) do inv[w] = key end
  return inv
end

-- ---------------------------------------------------------------- capture (identical on the base and on the projection)
local STORED_TEXT = table.concat({
  "# fixed stored settings of the golden (the two bind.* lines: retired keys since 2026-10-30, dropped on load)",
  "core.log=false", "headLook.maxYaw=80", "lootView.zoom=1.4", "bind.shoulder_swap=IK_LShift+IK_F", "bind.enabled=false",
  "camera.profile=3", "camera.profileCount=5", "camera.distance=2.23456789", "camera.aim.side=0.123456", "camera.p3.fov=62", "camera.sens=1.3",
  "camera.invertY=true", "camera.aim.fov=34", "perspective.aiming=fpp", "perspective.combat=fpp", "perspective.userWantsTpp=false",
  "perspective.startInLastView=false", "mm.air=hold", "mm.database=alt_db", "mm.trace=true", "mm.footReplant=off",
  "mm.motionDriver=false", "mm.leanScale=1.1", "bodyturn.turnRateMax=1.7", "bodyturn.shooterMode=true", "body.retry=false",
  "lootRange.scale=1.5", "headGuard.impl=false", "vehicle.pullOut=false", "other.module=42", "lootView.radius=9",
  "headLook.impl=auto", "camera.p9.distance=3", "" }, "\n")

local function readFile(path)
  local f = io.open(path, "rb")
  if not f then return nil end
  local s = f:read("*a")
  f:close()
  return s
end

local function snapshotDefs(core)
  local S = core.settings
  local defs = {}
  for _, key in ipairs(S.order) do
    local d = S.defs[key]
    local choices = nil
    if d.choices then
      choices = {}
      for i, c in ipairs(d.choices) do choices[i] = c[1] end
      choices = table.concat(choices, ",")
    end
    defs[key] = { type = d.type, default = ser(d.default), min = d.min, max = d.max, step = d.step, choices = choices,
                  section = d.section or "", hold = d.hold and true or false, onOff = d.onOff and true or false,
                  initial = d.initial ~= nil and ser(d.initial) or nil, hasApply = type(d.apply) == "function" }
  end
  return defs
end
local function snapshotValues(core)
  local out = {}
  for _, key in ipairs(core.settings.order) do out[key] = ser(core.settings.values[key]) end
  return out
end
local function copyList(t) local r = {} for i, v in ipairs(t) do r[i] = v end return r end
local function cfgSnapshot(cfg)
  local out = {}
  for k, v in pairs(cfg) do out[k] = ser(v) end
  return out
end

local function capture()
  local G = {}
  os.remove(tmpSettings)
  LOG = {}
  local core = loadCore()
  hookRegister(core)
  bootModules(core)
  G.boot = copyList(LOG)
  G.order = copyList(core.settings.order)
  G.defs = snapshotDefs(core)
  G.sectionOrder = copyList(core.settings.sectionOrder)
  G.sectionKeys = {}
  for _, id in ipairs(core.settings.sectionOrder) do G.sectionKeys[id] = copyList(core.settings.sections[id].keys) end
  G.fresh = snapshotValues(core)
  G.hl = cfgSnapshot(core.hl.cfg)
  G.la = cfgSnapshot(core.la.cfg)
  -- legacy build (the projection core builds its "all" view, which must be the same set of setting widgets)
  LOG = {}
  local ns = newNS()
  if core.SetMenuView then core.SetMenuView("all") end
  G.buildOk = core.BuildNativeSettings(ns) == true
  G.build = copyList(LOG)
  local inv = widgetKeys(core)
  G.widgets = {}
  for _, e in ipairs(ns.list()) do
    if e.w.kind ~= "button" and string.sub(e.sub, 1, 2) ~= "g_" then
      G.widgets[#G.widgets + 1] = "/utpp/" .. e.sub .. "|" .. e.w.kind .. "|" .. tostring(inv[e.w])
    end
  end
  G.fileAfterBuild = readFile(tmpSettings) or "<none>"
  LOG = {}
  core.ResetAll()
  G.reset = copyList(LOG)
  G.afterReset = snapshotValues(core)
  G.resetFile = readFile(tmpSettings) or "<none>"
  -- stored settings of an existing user
  os.remove(tmpSettings)
  LOG = {}
  local core2 = loadCore()
  local f = io.open(tmpSettings, "wb"); f:write(STORED_TEXT); f:close()
  core2.Load(tmpSettings)
  hookRegister(core2)
  bootModules(core2)
  G.bootStored = copyList(LOG)
  G.stored = snapshotValues(core2)
  G.orderStored = copyList(core2.settings.order)
  core2.Save(tmpSettings)
  G.storedFile = readFile(tmpSettings)
  os.remove(tmpSettings)
  return G, core, ns
end

-- ---------------------------------------------------------------- deterministic Lua serialisation of the golden
local function dump(v, indent)
  indent = indent or ""
  local t = type(v)
  if t ~= "table" then return ser(v) end
  local n = #v
  local keys, isArray = {}, true
  for k in pairs(v) do
    if type(k) ~= "number" or k < 1 or k > n or k ~= math.floor(k) then isArray = false end
    keys[#keys + 1] = k
  end
  local inner = indent .. "  "
  local parts = {}
  if isArray then
    for i = 1, n do parts[#parts + 1] = inner .. dump(v[i], inner) end
  else
    table.sort(keys, function(a, b) return tostring(a) < tostring(b) end)
    for _, k in ipairs(keys) do parts[#parts + 1] = inner .. "[" .. ser(k) .. "] = " .. dump(v[k], inner) end
  end
  if #parts == 0 then return "{}" end
  return "{\n" .. table.concat(parts, ",\n") .. "\n" .. indent .. "}"
end

local function diff(a, b, path, out, limit)
  if #out >= limit then return end
  if type(a) ~= type(b) then out[#out + 1] = path .. ": " .. ser(a) .. " ~= " .. ser(b); return end
  if type(a) ~= "table" then
    if a ~= b then out[#out + 1] = path .. ": " .. ser(a) .. " ~= " .. ser(b) end
    return
  end
  local seen = {}
  for k, v in pairs(a) do seen[k] = true; diff(v, b[k], path .. "." .. tostring(k), out, limit) end
  for k, v in pairs(b) do if not seen[k] then diff(nil, v, path .. "." .. tostring(k), out, limit) end end
end

-- ================================================================ T01 golden identity
local captured = capture()
if WRITE then
  local f = assert(io.open(goldenPath, "wb"))
  f:write("-- generated by test_utpp_settings_projection.lua (write mode): settings registry golden, do not edit\n")
  f:write("return " .. dump(captured) .. "\n")
  f:close()
  check("golden written: " .. goldenPath .. " (" .. #captured.order .. " keys)", true)
else
  local okG, golden = pcall(dofile, goldenPath)
  check("T01 golden fixture loads (" .. GOLDEN_NAME .. ")", okG and type(golden) == "table", golden)
  if okG and type(golden) == "table" then
    for _, part in ipairs({ "order", "defs", "sectionOrder", "sectionKeys", "fresh", "hl", "la", "boot", "buildOk", "build",
                            "widgets", "fileAfterBuild", "reset", "afterReset", "resetFile", "bootStored", "stored",
                            "orderStored", "storedFile" }) do
      local out = {}
      diff(golden[part], captured[part], part, out, 8)
      check("T01 golden identity: " .. part, #out == 0, table.concat(out, "; "))
    end
    check("T01 golden covers the real registry (" .. #captured.order .. " keys, " .. #captured.boot .. " boot calls)",
      #captured.order >= 150 and #captured.boot > 0)
  end
end

-- ================================================================ T02 layout, views, late registration, Native Settings contract
local function countSave(core)
  local n = { save = 0 }
  local orig = core.Save
  core.Save = function(...) n.save = n.save + 1; return orig(...) end
  return n
end
local function applyCount(key)
  local n = 0
  for _, l in ipairs(LOG) do if string.find(l, "apply " .. key .. "(", 1, true) == 1 then n = n + 1 end end
  return n
end
local function fullBoot(opts)
  os.remove(tmpSettings)
  LOG = {}
  local core = loadCore()
  hookRegister(core)
  local mods = bootModules(core, opts)
  return core, mods
end
-- displayed setting / view widgets in tab order: { "g_<group>|<key or view id>" ... } (buttons: "g_menu|button:<label>")
local function shown(core, ns)
  local inv = widgetKeys(core)
  for id, w in pairs(core.ui.viewWidgets) do inv[w] = "view:" .. id end
  local out = {}
  for _, e in ipairs(ns.list()) do
    out[#out + 1] = e.sub .. "|" .. (e.w.kind == "button" and ("button:" .. e.w.label) or tostring(inv[e.w]))
  end
  return out
end
-- intended standard order, computed from MenuPlacement and the registration order (independent of the build sequence)
local function expectedStandard(core)
  local S = core.settings
  local pos, GI = {}, {}
  for i, k in ipairs(S.order) do pos[k] = i end
  for i, g in ipairs(core.MENU_GROUPS) do GI[g.id] = i end
  local per = {}
  local function put(gid, sk, id) per[gid] = per[gid] or {}; per[gid][#per[gid] + 1] = { sk = sk, id = id } end
  for _, key in ipairs(S.order) do
    local level, group, rank = core.MenuPlacement(key)
    if S.defs[key].section and (level == "player" or level == "advanced") then
      put(group, { GI[group], rank, pos[key] }, key)
    end
  end
  for _, id in ipairs(core.views.order) do
    local v = core.views.byId[id]
    if v.level == "player" or v.level == "advanced" then
      put(v.group, { GI[v.group], v.rank, 100000 + v.index }, "view:" .. id)
    end
  end
  local out = {}
  for _, g in ipairs(core.MENU_GROUPS) do
    local list = per[g.id] or {}
    table.sort(list, function(a, b)
      for i = 1, 3 do if a.sk[i] ~= b.sk[i] then return a.sk[i] < b.sk[i] end end
      return false
    end)
    for _, e in ipairs(list) do out[#out + 1] = "g_" .. g.id .. "|" .. e.id end
  end
  return out
end
local function withoutButtons(list)
  local out = {}
  for _, s in ipairs(list) do if not string.find(s, "|button:", 1, true) then out[#out + 1] = s end end
  return out
end
local function sameList(a, b)
  if #a ~= #b then return false, "#" .. #a .. " ~= #" .. #b end
  for i = 1, #a do if a[i] ~= b[i] then return false, i .. ": " .. tostring(a[i]) .. " ~= " .. tostring(b[i]) end end
  return true
end
local function legacyList(ns)
  local out = {}
  for _, e in ipairs(ns.list()) do out[#out + 1] = e.sub .. "|" .. e.w.label end
  return out
end
local function valuesOf(core)
  local out = {}
  for _, k in ipairs(core.settings.order) do out[#out + 1] = k .. "=" .. ser(core.settings.values[k]) end
  return table.concat(out, "\n")
end
local function shownValueMatches(core, key, w)
  local def, v = core.settings.defs[key], core.settings.values[key]
  if w.kind == "switch" then return w.state == v end
  if w.kind == "selector" then return def.choices[w.value] ~= nil and def.choices[w.value][1] == v end
  return w.value == v
end

if not WRITE then
  -- ---------------------------------------------------------------- layout data
  do
    local core = fullBoot()
    local ids = {}
    for _, g in ipairs(core.MENU_GROUPS) do ids[#ids + 1] = g.id end
    check("layout: groups in order start, frame, combat, world, vehicles, advanced, debug, menu",
      table.concat(ids, ",") == "start,frame,combat,world,vehicles,advanced,debug,menu", table.concat(ids, ","))
    local labels = {}
    for _, g in ipairs(core.MENU_GROUPS) do labels[#labels + 1] = g.label end
    check("layout: group labels General, Camera, Combat, Interactions, Vehicles, Advanced, Debug, Menu",
      table.concat(labels, ",") == "General,Camera,Combat,Interactions,Vehicles,Advanced,Debug,Menu", table.concat(labels, ","))
    local bad, stale = {}, {}
    for key, l in pairs(core.MENU_LAYOUT) do
      local okG = false
      for _, g in ipairs(core.MENU_GROUPS) do if g.id == l.group and g.id ~= "menu" then okG = true end end
      if not core.MENU_LEVELS[l.level] or not okG or type(l.rank) ~= "number" then bad[#bad + 1] = key end
      if core.settings.defs[key] == nil then stale[#stale + 1] = key end
    end
    table.sort(stale)
    check("layout: every entry has a valid level, group and rank", #bad == 0, table.concat(bad, ", "))
    outPrint("[INFO] utpp_settings_projection: layout keys not registered by the loaded modules: "
      .. (#stale > 0 and table.concat(stale, ", ") or "none"))
    local counts, defaults, multi = {}, {}, {}
    for _, key in ipairs(core.settings.order) do
      local level, group, rank, source = core.MenuPlacement(key)
      counts[level] = (counts[level] or 0) + 1
      if source == "default" then defaults[#defaults + 1] = key end
      if level == "internal" and core.settings.defs[key].section ~= nil then multi[#multi + 1] = key end
      if level ~= "internal" and (group == nil or rank == nil) then multi[#multi + 1] = key end
    end
    check("layout: every registered key resolves to one level and group (internal = no section only)", #multi == 0,
      table.concat(multi, ", "))
    outPrint(string.format("[INFO] utpp_settings_projection: %d keys: player %d, advanced %d, developer %d, diagnostic %d, internal %d",
      #core.settings.order, counts.player or 0, counts.advanced or 0, counts.developer or 0, counts.diagnostic or 0,
      counts.internal or 0))
    outPrint("[INFO] utpp_settings_projection: keys placed by the default rule (Advanced): "
      .. (#defaults > 0 and table.concat(defaults, ", ") or "none"))
    -- 2026-10-23 single fixed camera profile: profile 2's raw fields are developer level (view "all" only), the other
    -- slots and camera.profile / profileCount / profileBlend are registered without a section (internal, no widget)
    local pat = true
    for _, key in ipairs({ "camera.distance", "camera.height", "camera.side", "camera.fov", "camera.framingSens" }) do
      if core.MenuPlacement(key) ~= "developer" then pat = false end
    end
    check("layout: camera.distance / height / side / fov / framingSens developer level (raw fields), no other camera slot registered",
      pat and core.settings.defs["camera.p1.distance"] == nil)
    check("layout: camera.profile / profileCount / profileBlend are not registered",
      core.settings.defs["camera.profile"] == nil and core.settings.defs["camera.profileCount"] == nil
      and core.settings.defs["camera.profileBlend"] == nil)
    -- 2026-10-30: own key bindings removed: no bind.* key is registered or listed in the layout
    local binds = {}
    for _, key in ipairs(core.settings.order) do if string.sub(key, 1, 5) == "bind." then binds[#binds + 1] = key end end
    for key in pairs(core.MENU_LAYOUT) do if string.sub(key, 1, 5) == "bind." then binds[#binds + 1] = "layout " .. key end end
    check("layout: no bind.* key registered or listed", #binds == 0, table.concat(binds, ", "))
    check("layout: perspective.toggleOverrides demoted to Debug (developer, no player trigger without the perspective key)",
      core.MenuPlacement("perspective.toggleOverrides") == "developer"
      and select(2, core.MenuPlacement("perspective.toggleOverrides")) == "debug")
    local function placeOf(k) local lv, g, r = core.MenuPlacement(k); return lv .. "/" .. g .. "/" .. tostring(r) end
    -- General: Use Ultimate Third Person, Animation pack, automatic perspective, (motion matching: developer level), Shooter mode,
    -- Start in the last used view, Diagnostic logs
    check("layout: General = Use Ultimate Third Person, Animation pack, Use automatic perspective, Shooter mode, Start in the last used view, Diagnostic logs (no Start in third person; motion matching is developer level)",
      placeOf("camera.impl") == "player/start/10" and placeOf("animation.pack") == "player/start/20"
      and placeOf("perspective.impl") == "advanced/start/30"
      and placeOf("mm.enabled") == "developer/start/40" and placeOf("bodyturn.shooterMode") == "player/start/50"
      and placeOf("perspective.startInLastView") == "player/start/60" and placeOf("core.diagnosticLogs") == "player/start/70"
      and core.MenuPlacement("perspective.defaultTpp") == "developer" and core.MenuPlacement("camera.startTpp") == "developer"
      and core.settings.defs["camera.impl"].label == "Use Ultimate Third Person",
      placeOf("camera.impl") .. " " .. placeOf("bodyturn.shooterMode"))
    check("layout: Camera = Orbit sensitivity, Invert horizontal / vertical orbit, empty-hands zoom (after the framing views)",
      placeOf("camera.sens") == "player/frame/10" and placeOf("camera.invertX") == "player/frame/20"
      and placeOf("camera.invertY") == "player/frame/30" and placeOf("camera.zoomUnarmed.enabled") == "player/frame/40")
    -- 2026-10-30 user: Run pelvis yaw trim, then Body yaw offset right after it (both registered by utpp_motion); 2026-10-31: Walk speed,
    -- then Minimum walk speed; 2026-11-09 (user K5): Shoulder width (female V) last
    -- 2026-11-13: Run pelvis yaw trim, Body yaw offset and Minimum walk speed are hidden in 0.1.0 (level "hidden", layout place kept)
    check("layout: Camera = (hidden: Run pelvis yaw trim, Body yaw offset, Head follows turns), Walk speed, (hidden: Minimum walk speed), Shoulder width, then the advanced camera options",
      placeOf("mm.pelvisYawTrim") == "hidden/frame/50" and placeOf("mm.bodyYawOffset") == "hidden/frame/60"
      and placeOf("mm.headLead") == "hidden/frame/70"
      and placeOf("mm.walkSpeed") == "player/frame/80" and placeOf("mm.minWalkSpeed") == "hidden/frame/90"
      and placeOf("mm.shoulderWidth") == "player/frame/100" and placeOf("camera.idleOrbit") == "advanced/frame/110",
      placeOf("mm.pelvisYawTrim") .. " " .. placeOf("mm.bodyYawOffset") .. " " .. placeOf("mm.walkSpeed") .. " "
        .. placeOf("mm.minWalkSpeed") .. " " .. placeOf("mm.shoulderWidth") .. " " .. placeOf("camera.idleOrbit"))
    -- 2026-10-30 aim pitch plan: "Weapon aims at crosshair point" (mm.aimAtCrosshair, utpp_motion) after the crosshair colour, player level
    -- 2026-11-09: Melee full body / Melee aim tilt (player level) after Weapon aims at crosshair point
    check("layout: Combat = crosshair colour, then Weapon aims at crosshair point, Melee full body, Melee aim tilt, then the advanced combat options",
      placeOf("crosshair.color") == "player/combat/70" and placeOf("mm.aimAtCrosshair") == "hidden/combat/80"
      and placeOf("melee.fullBody") == "player/combat/90" and placeOf("melee.aimTilt") == "player/combat/100"
      and placeOf("perspective.shortScopeFpp") == "advanced/combat/110",
      placeOf("crosshair.color") .. " " .. placeOf("mm.aimAtCrosshair") .. " " .. placeOf("melee.fullBody") .. " "
      .. placeOf("melee.aimTilt") .. " " .. placeOf("perspective.shortScopeFpp"))
    check("layout: Vehicles = vehicle player options then the advanced vehicle options (running rank)",
      placeOf("perspective.vehicleKeepTpp") == "player/vehicles/10" and placeOf("vehicle.forceTppOnEnter") == "player/vehicles/20"
      and placeOf("ragdoll.enabled") == "developer/vehicles/100"
      and placeOf("vehicle.keepTpp") == "hidden/vehicles/30" and placeOf("vehicle.rideWatchdog") == "hidden/vehicles/60",
      placeOf("ragdoll.enabled") .. " " .. placeOf("vehicle.keepTpp") .. " " .. placeOf("vehicle.rideWatchdog"))
    check("layout: Debug = aim occlusion (hidden in 0.1.0: no Debug category in the menu), the single logs diagnostic",
      placeOf("camera.aim.occlusion") == "hidden/debug/10" and placeOf("core.log") == "diagnostic/debug/20")
    local zu = {}
    for _, key in ipairs(core.settings.order) do
      if string.find(key, "camera.zoomUnarmed.", 1, true) == 1 then
        local level, group = core.MenuPlacement(key)
        zu[#zu + 1] = string.sub(key, 20) .. "=" .. level .. "/" .. group
      end
    end
    check("layout: unarmed zoom: enabled player (Camera), boom keys developer, tuning advanced (Advanced)", table.concat(zu, ",")
      == "boomPitchScale=developer/advanced,boomFrame=developer/advanced,enabled=player/frame,steps=advanced/advanced,"
      .. "distance=advanced/advanced,lastDistance=advanced/advanced,fov=hidden/advanced,lastFov=hidden/advanced,"
      .. "fppHandoff=advanced/advanced", table.concat(zu, ","))
    check("layout: unknown key -> advanced level in group advanced", select(1, core.MenuPlacement("vehicle.futureOption")) == "advanced"
      and select(2, core.MenuPlacement("vehicle.futureOption")) == "advanced" and select(4, core.MenuPlacement("vehicle.futureOption")) == "default")
    check("layout: dynamic per-weapon keys are developer level", core.MenuPlacement("mm.weapon.Items.X.ready") == "developer")
    check("layout: perspective.userWantsTpp stays internal (no section, no widget)", core.MenuPlacement("perspective.userWantsTpp") == "internal")
    check("layout: Shooter mode label 'Shooter mode' on the definition, no presentation override", core.settings.defs["bodyturn.shooterMode"].label
      == "Shooter mode" and core.MENU_LAYOUT["bodyturn.shooterMode"].label == nil)
  end

  -- ---------------------------------------------------------------- standard view build
  do
    local core = fullBoot()
    local saves = countSave(core)
    core.Save(tmpSettings)
    saves.save = 0
    local before = readFile(tmpSettings)
    local valsBefore = valuesOf(core)
    LOG = {}
    local ns = newNS()
    check("standard: default view is standard", core.MenuView() == "standard")
    check("standard: built", core.BuildNativeSettings(ns) == true)
    check("standard: Native Settings calls valid (indexes, paths, no stale setOption)", #ns.errors == 0, table.concat(ns.errors, "; "))
    check("standard: subcategories in group order", table.concat(ns.subs(), ",") == "g_start,g_frame,g_combat,g_world,g_vehicles,g_advanced,g_menu",
      table.concat(ns.subs(), ","))
    local list = shown(core, ns)
    local okOrder, why = sameList(withoutButtons(list), expectedStandard(core))
    check("standard: widgets and order follow the layout (own group for player and advanced options, then rank)", okOrder, why)
    local hidden, dup, seen = {}, {}, {}
    for _, s in ipairs(list) do
      local key = string.match(s, "|(.*)$")
      if seen[key] then dup[#dup + 1] = key end
      seen[key] = true
    end
    for _, key in ipairs(core.settings.order) do
      local level = core.MenuPlacement(key)
      local has = core.settings.widgets[key] ~= nil
      if (level == "developer" or level == "diagnostic" or level == "internal") and has then hidden[#hidden + 1] = key end
      if (level == "player" or level == "advanced") and not has then hidden[#hidden + 1] = "missing " .. key end
    end
    check("standard: no widget for developer / diagnostic / internal keys, one for every player / advanced key", #hidden == 0,
      table.concat(hidden, ", "))
    check("standard: every visible key exactly once", #dup == 0, table.concat(dup, ", "))
    local raw = false
    for _, s in ipairs(list) do if string.match(s, "|camera%.p%d%.") or string.find(s, "|mm.weapon.", 1, true) then raw = true end end
    check("standard: no raw camera.pN.* / per-weapon widgets", not raw)
    local menu = {}
    for _, s in ipairs(list) do if string.sub(s, 1, 7) == "g_menu|" then menu[#menu + 1] = s end end
    check("standard: menu group = view switch + reset", table.concat(menu, ",")
      == "g_menu|button:Show all options (this session),g_menu|button:Reset all UTPP settings", table.concat(menu, ","))
    local wrong = {}
    for key, w in pairs(core.settings.widgets) do if not shownValueMatches(core, key, w) then wrong[#wrong + 1] = key end end
    check("standard: every widget shows the current value", #wrong == 0, table.concat(wrong, ", "))
    -- 2026-10-24: camera.startTpp / perspective.defaultTpp are developer-level, so the standard menu has no widget for them;
    -- the label override stays presentation-only (the definition keeps its own label)
    check("standard: no widget for the removed start-perspective options",
      core.settings.widgets["camera.startTpp"] == nil and core.settings.widgets["perspective.defaultTpp"] == nil)
    check("standard: presentation label override stays out of the definition",
      core.MENU_LAYOUT["camera.startTpp"].label ~= core.settings.defs["camera.startTpp"].label
      and core.settings.defs["camera.startTpp"].label == "Start in third person")
    check("standard: building calls no apply / native and no Save", #LOG == 0 and saves.save == 0, table.concat(LOG, "; "))
    check("standard: settings file and values unchanged by the build", readFile(tmpSettings) == before and valuesOf(core) == valsBefore)
    -- registry flag hidden = true: four head look tuning options and 23 more options are registered
    -- and working on their defaults, listed in no view; HIDDEN is the exact set (the head look switch and "Look at the camera"
    -- are visible since 2026-11-13: they keep their defaults, DEFAULTS lists them too, but HIDDEN does not)
    do
      local DEFAULTS = { ["headLook.impl"] = true, ["headLook.whileWalking"] = true, ["headLook.faceCamera"] = true,
                         ["headLook.lookAtLoot"] = true, ["headLook.maxYaw"] = 100, ["headLook.chest"] = 0.5,
                         ["camera.aim.occlusion"] = true, ["mm.pelvisYawTrim"] = 5, ["mm.bodyYawOffset"] = 8, ["mm.minWalkSpeed"] = 0.82,
                         ["camera.body"] = true, ["camera.collision"] = true, ["mm.aimAtCrosshair"] = true, ["perspective.sceneTier"] = 3,
                         ["interactRange.scale"] = 1.0, ["lootRange.scale"] = 1.0, ["lootView.zoom"] = 0.9, ["lootView.radius"] = 1.8,
                         ["vehicle.keepTpp"] = true, ["vehicle.pullOut"] = true, ["vehicle.rideWatchdog"] = true,
                         ["vehicle.safeTeleportExit"] = true, ["camera.mouseScale"] = 0, ["perspective.minGap"] = 0.5,
                         ["camera.aim.range"] = 60, ["camera.zoomUnarmed.fov"] = 45, ["camera.zoomUnarmed.lastFov"] = 25,
                         ["perspective.blend"] = 0.8, ["mm.headLead"] = true }
      local HIDDEN = {}
      for key in pairs(DEFAULTS) do HIDDEN[key] = true end
      HIDDEN["headLook.impl"], HIDDEN["headLook.faceCamera"] = nil, nil
      local flagged, listed, bad = 0, {}, {}
      for _, key in ipairs(core.settings.order) do
        local def = core.settings.defs[key]
        if def.hidden then flagged = flagged + 1 end
        if (def.hidden == true) ~= (HIDDEN[key] == true) then bad[#bad + 1] = key end
      end
      for _, s in ipairs(list) do if HIDDEN[string.match(s, "|(.*)$")] then listed[#listed + 1] = s end end
      check("hidden: exactly the 27 listed options carry the flag (4 head look + 23 more)", flagged == 27 and #bad == 0, table.concat(bad, ", "))
      check("hidden: level 'hidden' (kept layout group), no widget, not in the standard list", #listed == 0
        and core.MenuPlacement("headLook.maxYaw") == "hidden" and select(2, core.MenuPlacement("headLook.maxYaw")) == "world"
        and core.settings.widgets["headLook.whileWalking"] == nil and core.settings.widgets["headLook.lookAtLoot"] == nil
        and core.settings.widgets["headLook.maxYaw"] == nil and core.settings.widgets["headLook.chest"] == nil
        and core.settings.widgets["camera.aim.occlusion"] == nil and core.settings.widgets["lootView.zoom"] == nil
        and core.settings.sections["headlook"] ~= nil and #core.settings.sections["headlook"].keys == 6, table.concat(listed, ", "))
      -- visible since 2026-11-13: player level, Interactions group (g_world), the switch first, "Look at the camera" right after it
      do
        local pos, prev = {}, nil
        for i, s in ipairs(list) do pos[string.match(s, "|(.*)$") or s] = i end
        local lvI, gI, rI = core.MenuPlacement("headLook.impl")
        local lvF, gF, rF = core.MenuPlacement("headLook.faceCamera")
        check("head look: switch and 'Look at the camera' are player-level widgets in g_world, in this order, standard view",
          lvI == "player" and gI == "world" and lvF == "player" and gF == "world" and rF == rI + 10
          and core.settings.widgets["headLook.impl"] ~= nil and core.settings.widgets["headLook.faceCamera"] ~= nil
          and core.settings.widgets["headLook.impl"].path == "/utpp/g_world" and core.settings.widgets["headLook.faceCamera"].path == "/utpp/g_world"
          and pos["headLook.impl"] ~= nil and pos["headLook.faceCamera"] == pos["headLook.impl"] + 1
          and shownValueMatches(core, "headLook.impl", core.settings.widgets["headLook.impl"])
          and shownValueMatches(core, "headLook.faceCamera", core.settings.widgets["headLook.faceCamera"]))
      end
      check("hidden: the Debug group has no widget, so the standard view has no g_debug subcategory",
        core.settings.widgets["camera.aim.occlusion"] == nil and not ns.pathExists("/utpp/g_debug"))
      -- the head lead switch is hidden (the lean head layer takes the head in the default configuration): key, label, default
      -- and description unchanged
      local leadShown = false
      for _, s in ipairs(list) do if s == "g_frame|mm.headLead" then leadShown = true end end
      local lead = core.settings.defs["mm.headLead"]
      check("hidden: 'Head follows turns' (mm.headLead) has no widget; key, label, default and text are unchanged", not leadShown
        and core.MenuPlacement("mm.headLead") == "hidden" and lead.label == "Head follows turns" and lead.default == true
        and lead.type == "bool" and core.settings.widgets["mm.headLead"] == nil
        and lead.desc == "While V runs or sprints through a turn, her head turns into the turn. Off: the head stays as the run animations have it. No effect with the Game Animations pack.")
      local blendShown = false
      for _, s in ipairs(list) do if s == "g_frame|perspective.blend" then blendShown = true end end
      local blend = core.settings.defs["perspective.blend"]
      check("hidden: 'Transition time (s)' (perspective.blend) has no widget; key, label, default and range are unchanged", not blendShown
        and core.MenuPlacement("perspective.blend") == "hidden" and blend.label == "Transition time (s)" and blend.default == 0.8
        and blend.min == 0.2 and blend.max == 2.5 and core.settings.widgets["perspective.blend"] == nil)
      local dflt = {}
      for key, v in pairs(DEFAULTS) do
        if core.settings.defs[key].default ~= v or core.Get(key) ~= v or core.Effective(key) ~= v then dflt[#dflt + 1] = key end
      end
      check("hidden: default, value and effective value unchanged (head look still on)", #dflt == 0, table.concat(dflt, ", "))
      core.Set("headLook.maxYaw", 80)
      check("hidden: console Set works, applies, is saved and is stored with the other settings", core.Get("headLook.maxYaw") == 80
        and core.hl.cfg.maxYaw == 80 and string.find(readFile(tmpSettings) or "", "headLook.maxYaw=80", 1, true) ~= nil)
      core.Set("headLook.maxYaw", 100)
    end
    local nPlayer = 0
    for _, k in ipairs(core.settings.order) do if core.MenuPlacement(k) == "player" then nPlayer = nPlayer + 1 end end
    outPrint(string.format("[INFO] utpp_settings_projection: standard view %d option rows (%d player level), view all %d",
      #withoutButtons(list), nPlayer, #captured.widgets))

    -- one Set -> exactly one apply, at most one Save, no recursion (setOption calls the callback synchronously)
    LOG, saves.save = {}, 0
    core.Set("camera.sens", 1.4)
    check("set: console Set of a shown key -> one apply, one Save, widget synced", applyCount("camera.sens") == 1 and saves.save == 1
      and core.settings.widgets["camera.sens"].value == 1.4, #LOG .. " " .. saves.save)
    LOG, saves.save = {}, 0
    ns.userChange(core.settings.widgets["camera.sens"], 1.6)
    check("set: widget change -> one apply, one Save", applyCount("camera.sens") == 1 and saves.save == 1 and core.Get("camera.sens") == 1.6)
    LOG, saves.save = {}, 0
    local aimChoices, other = core.settings.defs["perspective.aiming"].choices, nil
    for i, c in ipairs(aimChoices) do if other == nil and c[1] ~= core.Get("perspective.aiming") then other = i end end
    ns.userChange(core.settings.widgets["perspective.aiming"], other)
    check("set: selector change -> one apply, one Save", applyCount("perspective.aiming") == 1 and saves.save == 1
      and core.Get("perspective.aiming") == aimChoices[other][1], table.concat(LOG, "; ") .. " saves=" .. saves.save)
    LOG, saves.save = {}, 0
    core.Set("mm.air", "hold")
    check("set: hidden key still works from the console (one apply, one Save)", applyCount("mm.air") == 1 and saves.save == 1
      and core.Get("mm.air") == "hold")
    check("set: no Native Settings errors after the changes", #ns.errors == 0, table.concat(ns.errors, "; "))

    -- restore defaults: ResetAll, visible widgets re-synced
    check("reset: restore-defaults callback overrides the library reset", ns.restore and ns.restore.override == true and ns.restore.path == "/utpp")
    LOG, saves.save = {}, 0
    ns.restore.cb()
    check("reset: restore defaults resets every key (also hidden ones), one Save", core.Get("camera.sens") == 1.0
      and core.Get("mm.air") == "auto" and core.Get("perspective.aiming") == "tpp" and saves.save == 1)
    wrong = {}
    for key, w2 in pairs(core.settings.widgets) do if not shownValueMatches(core, key, w2) then wrong[#wrong + 1] = key end end
    check("reset: visible widgets show the defaults", #wrong == 0, table.concat(wrong, ", "))
    local resetBtn
    for _, e in ipairs(ns.list()) do if e.w.label == "Reset all UTPP settings" then resetBtn = e.w end end
    core.Set("camera.sens", 2.0)
    resetBtn.cb()
    check("reset: the Menu group reset button runs ResetAll", core.Get("camera.sens") == 1.0)

    -- view switch: session only, rebuilt when the tab closes
    core.Save(tmpSettings)
    local file0, defs0 = readFile(tmpSettings), #core.settings.order
    saves.save = 0
    LOG = {}
    local stdList = shown(core, ns)
    check("view: invalid view rejected", core.SetMenuView("dev") == nil and core.MenuView() == "standard")
    check("view: SetMenuView(all) after the build is pending until the tab closes", core.SetMenuView("all") == "all"
      and core.MenuView() == "standard" and select(2, core.MenuView()) == "all" and sameList(shown(core, ns), stdList))
    ns.closeTab()
    core.MenuTick()
    check("view: tab closed -> view all built", core.MenuView() == "all" and select(2, core.MenuView()) == nil)
    local subs = ns.subs()
    -- sections whose keys are all hidden have no subcategory (head look in 0.1.0)
    local visibleSections = {}
    for _, id in ipairs(captured.sectionOrder) do
      for _, key in ipairs(core.settings.sections[id].keys) do
        if not core.settings.defs[key].hidden then visibleSections[#visibleSections + 1] = id; break end
      end
    end
    check("view all: source sections in registration order (sections with a visible key), then the menu group", table.concat(subs, ",")
      == table.concat(visibleSections, ",") .. ",g_menu" and core.settings.sections["headlook"] ~= nil
      and ns.pathExists("/utpp/headlook"), table.concat(subs, ","))
    local allW = {}
    local inv = widgetKeys(core)
    for _, e in ipairs(ns.list()) do
      if e.w.kind ~= "button" and string.sub(e.sub, 1, 2) ~= "g_" then allW[#allW + 1] = "/utpp/" .. e.sub .. "|" .. e.w.kind .. "|" .. tostring(inv[e.w]) end
    end
    check("view all: identical to the legacy build (paths, kinds, keys, order)", sameList(allW, captured.widgets))
    check("view all: menu group = 'Show standard options' + reset", table.concat(withoutButtons(shown(core, ns)), ",") ~= ""
      and ns.list()[#ns.list()].w.label == "Reset all UTPP settings" and ns.list()[#ns.list() - 1].w.label == "Show standard options")
    check("view all: raw camera.p2.* fields shown, other slots and camera.profile not", core.settings.widgets["camera.fov"] ~= nil
      and core.settings.widgets["camera.p3.fov"] == nil and core.settings.widgets["camera.profile"] == nil)
    check("view: rebuild leaves no Native Settings error (old widgets never synced)", #ns.errors == 0, table.concat(ns.errors, "; "))
    check("view: switch to all called no apply / native, no Save, file unchanged", #LOG == 0 and saves.save == 0
      and readFile(tmpSettings) == file0, table.concat(LOG, "; ") .. "/" .. saves.save)
    core.Set("camera.fov", 61)
    check("view all: Set syncs the new widget", core.settings.widgets["camera.fov"].value == 61 and #ns.errors == 0)
    core.Set("camera.fov", core.settings.defs["camera.fov"].default)
    core.Save(tmpSettings)
    file0 = readFile(tmpSettings)
    saves.save = 0
    LOG = {}
    -- the button toggles back; two toggles before closing cancel out
    local btn
    for _, e in ipairs(ns.list()) do if e.w.label == "Show standard options" then btn = e.w end end
    btn.cb()
    check("view: button requests standard", select(2, core.MenuView()) == "standard")
    btn.cb()
    check("view: second press cancels the request", select(2, core.MenuView()) == nil)
    local allList = shown(core, ns)
    ns.closeTab()
    core.MenuTick()
    check("view: close without a request keeps the tab as it is", sameList(shown(core, ns), allList) and core.MenuView() == "all")
    btn.cb()
    ns.closeTab()
    core.MenuTick()
    check("view: back to standard = the first standard build", core.MenuView() == "standard" and sameList(shown(core, ns), stdList),
      select(2, sameList(shown(core, ns), stdList)))
    check("view: switches called no apply / native, no Save, file unchanged", #LOG == 0 and saves.save == 0
      and readFile(tmpSettings) == file0 and #core.settings.order == defs0, table.concat(LOG, "; ") .. "/" .. saves.save)
    local text = readFile(tmpSettings)
    check("view: not stored (no menu / view key in the settings file)", not string.find(text, "menu", 1, true)
      and not string.find(text, "view", 1, true))
    local fresh = loadCore()
    check("view: a new session starts in the standard view", fresh.MenuView() == "standard")
    check("view: no Native Settings error after the switches", #ns.errors == 0, table.concat(ns.errors, "; "))
  end

  -- ---------------------------------------------------------------- view rebuild is never run inside RequestClose (review F4)
  -- Native Settings calls the tab-closed callback while currentTab is still the UTPP tab; a rebuild there spawns every widget into
  -- the closing controller and queues scroll restores that fail one per frame. The callback only marks the request; onUpdate
  -- applies it once the UTPP tab is not the current one.
  do
    local core = fullBoot()
    local ns = newNS()
    core.BuildNativeSettings(ns)
    local stdList = shown(core, ns)
    core.SetMenuView("all")
    ns.liveCalls = 0
    ns.closeTab()
    check("close: the callback alone rebuilds nothing (no widget spawned while currentTab is the UTPP tab)",
      ns.liveCalls == 0 and #ns.errors == 0 and core.MenuView() == "standard" and select(2, core.MenuView()) == "all"
      and sameList(shown(core, ns), stdList), table.concat(ns.errors, "; ") .. " live=" .. ns.liveCalls)
    check("close: no rebuild error in the closing controller", #ns.errors == 0, table.concat(ns.errors, "; "))
    ns.currentTab = "utpp"
    check("close: still on the UTPP tab -> the tick keeps waiting (request kept)", core.MenuTick() == false
      and select(2, core.MenuView()) == "all" and ns.liveCalls == 0 and sameList(shown(core, ns), stdList))
    ns.currentTab = nil
    check("close: the tick after the menu is closed applies the view once", core.MenuTick() == true and core.MenuView() == "all"
      and select(2, core.MenuView()) == nil and #ns.errors == 0 and core.ui.applyWhenClosed == false)
    check("close: a further tick does nothing", core.MenuTick() == false and #ns.errors == 0)

    -- tab change instead of closing: the library calls the callback with the old tab current, the rebuild waits for the next frame
    core.SetMenuView("standard")
    ns.liveCalls = 0
    ns.closeTab("utpp", true)
    check("tab change: callback while the UTPP tab is current rebuilds nothing", ns.liveCalls == 0 and #ns.errors == 0
      and core.MenuView() == "all" and select(2, core.MenuView()) == "standard")
    ns.currentTab = "other"
    check("tab change: tick on another tab applies the view without a live spawn", core.MenuTick() == true
      and core.MenuView() == "standard" and ns.liveCalls == 0 and #ns.errors == 0 and sameList(shown(core, ns), stdList))

    -- a cancelled request (pressed twice) leaves nothing to apply; the mark is dropped by the tick
    core.SetMenuView("all")
    ns.currentTab = "utpp"
    ns.closeTab("utpp", true)
    core.SetMenuView("standard")
    ns.currentTab = nil
    check("cancel: request cancelled before the tick -> nothing rebuilt, mark dropped", core.MenuTick() == false
      and core.MenuView() == "standard" and core.ui.applyWhenClosed == false and ns.liveCalls == 0 and #ns.errors == 0)

    -- onUpdate drives the tick (registered handler of the module)
    local handlers = {}
    registerForEvent, registerInput = function(ev, fn) handlers[ev] = fn end, function() end
    sandboxIo()
    local core2 = dofile(UTPP_CORE_PATH)
    unsandboxIo()
    registerForEvent, registerInput = SAVED.registerForEvent, SAVED.registerInput
    core2.sink = function() end
    core2.SETTINGS_FILE = tmpSettings
    local ns2 = newNS()
    core2.BuildNativeSettings(ns2)
    core2.SetMenuView("all")
    ns2.closeTab()
    check("onUpdate: registered handler exists, view still pending before the frame", type(handlers.onUpdate) == "function"
      and select(2, core2.MenuView()) == "all")
    local okU = pcall(handlers.onUpdate, 1 / 30)
    check("onUpdate: first frame after the close applies the view (no player needed)", okU and core2.MenuView() == "all"
      and select(2, core2.MenuView()) == nil and ns2.liveCalls == 0 and #ns2.errors == 0, table.concat(ns2.errors, "; "))
  end

  -- ---------------------------------------------------------------- tab created before utpp_core / library without removeSubcategory
  do
    local core = fullBoot()
    local ns = newNS()
    ns.addTab("/utpp", "UTPP")
    check("foreign tab: built into the existing tab", core.BuildNativeSettings(ns) == true and ns.subs()[1] == "g_start")
    core.SetMenuView("all")
    ns.closeTab()
    core.MenuTick()
    check("foreign tab: no close callback -> still standard", core.MenuView() == "standard")
    check("foreign tab: next BuildNativeSettings applies the view", core.BuildNativeSettings(ns) == true and core.MenuView() == "all"
      and ns.subs()[1] == "general" and #ns.errors == 0, table.concat(ns.subs(), ","))
    local core2 = fullBoot()
    local ns2 = newNS()
    ns2.removeSubcategory = nil
    core2.BuildNativeSettings(ns2)
    local before = shown(core2, ns2)
    core2.SetMenuView("all")
    coreLines = {}
    ns2.closeTab()
    core2.MenuTick()
    local logged = false
    for _, l in ipairs(coreLines) do if string.find(l, "without removeSubcategory", 1, true) then logged = true end end
    check("old library: view switch refused visibly, tab unchanged", core2.MenuView() == "standard" and logged and sameList(shown(core2, ns2), before))
  end

  -- ---------------------------------------------------------------- one failing widget, missing Native Settings
  do
    local core = fullBoot()
    local vals = valuesOf(core)
    check("no NS: BuildNativeSettings(nil) -> false", core.BuildNativeSettings(nil) == false and core.BuildNativeSettings({}) == false)
    check("no NS: registry and values unchanged", valuesOf(core) == vals and #core.settings.order == #captured.order and not core.ui.built)
    coreLines = {}
    local ns = newNS({ failLabel = "Orbit sensitivity" })
    check("failing widget: build still succeeds", core.BuildNativeSettings(ns) == true)
    local logged = false
    for _, l in ipairs(coreLines) do if string.find(l, "widget camera.sens FAILED", 1, true) then logged = true end end
    check("failing widget: logged, no widget for it, the rest built", logged and core.settings.widgets["camera.sens"] == nil
      and core.settings.widgets["camera.invertY"] ~= nil and core.settings.widgets["lootView.impl"] ~= nil)
    check("failing widget: registry and values intact", valuesOf(core) == vals and core.Set("camera.sens", 1.2) == 1.2)
  end

  -- ---------------------------------------------------------------- late registration after the build
  do
    -- core alone first (onInit order in game: utpp_core builds the tab, other mods register later)
    local core = fullBoot({ skip = { bodyturn = true, body = true, v2 = true, camera = true, perspective = true, loothead = true,
                                     vehiclecam = true } })
    local ns = newNS()
    core.BuildNativeSettings(ns)
    local s0 = table.concat(ns.subs(), ",")
    -- core's own options: Animation pack and Diagnostic logs in General, mm.shinobiBonusReload in Combat, mm.headLead (Head follows turns, hidden: no Camera group from core alone), ragdoll.enabled is developer level since 0.1.0 (no Vehicles group from core alone); the rest of the frame group and advanced are created late
    check("late: core only -> groups that have options", s0 == "g_start,g_combat,g_world,g_menu", s0)
    -- the other modules register after the build
    LOG = {}
    local mods = bootModules(core, { skip = { vehiclecam = true } })
    check("late: groups created at their place", table.concat(ns.subs(), ",") == "g_start,g_frame,g_combat,g_world,g_vehicles,g_advanced,g_menu",
      table.concat(ns.subs(), ","))
    mods.loadVehiclecam()
    restoreGlobals()
    for _, key in ipairs(core.settings.order) do wrapApply(core.settings.defs[key]) end
    local v2 = mods.loadMotion()
    check("late: dynamic per-weapon key registered", v2.RegisterWeaponOverride(core, "Items.Preset_Lexington_Default") == true
      and core.settings.defs["mm.weapon.Items.Preset_Lexington_Default.ready"] ~= nil)
    core.RegisterSection("vehicle", "Vehicles")
    core.Register({ key = "vehicle.futureOption", section = "vehicle", type = "bool", default = true, label = "Future vehicle option" })
    core.Register({ key = "newmod.option", section = "newmod", type = "float", default = 1, min = 0, max = 2, label = "New module option" })
    check("late: Native Settings calls valid (indexes in range, no duplicates)", #ns.errors == 0, table.concat(ns.errors, "; "))
    local okOrder, why = sameList(withoutButtons(shown(core, ns)), expectedStandard(core))
    check("late: every widget at its layout place, none duplicated (vehicle keys, unlisted keys)", okOrder, why)
    check("late: unlisted vehicle / module keys visible in Advanced", core.settings.widgets["vehicle.futureOption"] ~= nil
      and core.settings.widgets["vehicle.futureOption"].path == "/utpp/g_advanced" and core.settings.widgets["newmod.option"] ~= nil)
    check("late: vehicle keys listed in the layout placed in Vehicles (hidden ones have no widget)",
      core.settings.widgets["vehicle.approachWalk"] ~= nil and core.settings.widgets["vehicle.approachWalk"].path == "/utpp/g_vehicles"
      and core.settings.widgets["vehicle.keepTpp"] == nil)
    check("late: developer keys registered late get no widget (mm.orientWarp, per-weapon key)",
      core.settings.widgets["mm.orientWarp"] == nil and core.settings.widgets["mm.weapon.Items.Preset_Lexington_Default.ready"] == nil)
    check("late: no legacy subcategory in the standard view", not ns.pathExists("/utpp/vehicle") and not ns.pathExists("/utpp/newmod"))
    -- same registry in view all after the switch, then a late section there
    core.SetMenuView("all")
    ns.closeTab()
    core.MenuTick()
    core.RegisterSection("later", "Later section")
    core.Register({ key = "later.option", section = "later", type = "bool", default = false, label = "Later option" })
    core.Register({ key = "newmod.second", section = "newmod", type = "bool", default = false, label = "Second option" })
    local subs = ns.subs()
    check("late (all): new section before the menu group", subs[#subs] == "g_menu" and subs[#subs - 1] == "later", table.concat(subs, ","))
    local lst = legacyList(ns)
    local iSecond, iNew
    for i, s in ipairs(lst) do
      if s == "newmod|Second option" then iSecond = i end
      if s == "newmod|New module option" then iNew = i end
    end
    check("late (all): key appended to its existing section", iSecond and iNew and iSecond == iNew + 1, tostring(iNew) .. "/" .. tostring(iSecond))
    check("late (all): per-weapon key shown under its section", core.settings.widgets["mm.weapon.Items.Preset_Lexington_Default.ready"] ~= nil)
    check("late (all): no Native Settings error", #ns.errors == 0, table.concat(ns.errors, "; "))
  end

  -- ---------------------------------------------------------------- views (RegisterView / RefreshView)
  do
    local core = fullBoot()
    local store = { a = 2.5, b = 0.1, flag = true }
    local sets = {}
    local function view(id, field, extra)
      local v = { id = id, type = "float", label = "View " .. id, desc = "test view", min = 0, max = 5, step = 0.05, format = "%.2f",
                  get = function() return store[field] end, set = function(x) sets[#sets + 1] = id .. "=" .. tostring(x); store[field] = x end,
                  default = function() return 1 end }
      for k, x in pairs(extra or {}) do v[k] = x end
      return v
    end
    check("views: RegisterView rejects a view without get / set", core.RegisterView({ id = "x", type = "float", min = 0, max = 1 }) == nil)
    check("views: RegisterView rejects an unknown type", core.RegisterView(view("t.bad", "a", { type = "choice" })) == nil)
    check("views: RegisterView rejects a range without min / max", core.RegisterView(view("t.nomin", "a", { min = false })) == nil)
    -- rank 5: after utpp_camera's framing views (ranks 1-4), before camera.sens (rank 10)
    check("views: player view registered before the build", core.RegisterView(view("test.edit.a", "a", { group = "frame", level = "player", rank = 5 })) == true)
    check("views: advanced view", core.RegisterView(view("test.edit.b", "b", { group = "frame", level = "advanced", rank = 15 })) == true)
    check("views: developer view", core.RegisterView(view("test.dev", "b", { group = "frame", level = "developer", rank = 1 })) == true)
    check("views: never settings", core.settings.defs["test.edit.a"] == nil and core.settings.values["test.edit.a"] == nil
      and #core.settings.order == #captured.order)
    check("views: RefreshView before the build -> 0", core.RefreshView("test.") == 0)
    local saves = countSave(core)
    LOG = {}
    local ns = newNS()
    core.BuildNativeSettings(ns)
    local list = shown(core, ns)
    local iFirst, iHeight, iA, iSens
    for i, s in ipairs(list) do
      if iFirst == nil and string.sub(s, 1, 8) == "g_frame|" then iFirst = i end
      if s == "g_frame|view:camera.edit.height" then iHeight = i end
      if s == "g_frame|view:test.edit.a" then iA = i end
      if s == "g_frame|camera.sens" then iSens = i end
    end
    -- the framing group opens with the camera framing views (distance, shoulder offset, FOV, height: ranks 1-4)
    local framing = iFirst ~= nil and list[iFirst] == "g_frame|view:camera.edit.distance"
      and list[iFirst + 1] == "g_frame|view:camera.edit.side" and list[iFirst + 2] == "g_frame|view:camera.edit.fov"
      and iHeight == iFirst + 3
    check("views: player view (rank 5) right after the framing views (ranks 1-4), before camera.sens (rank 10)", framing
      and iA == iHeight + 1 and iSens == iA + 1, tostring(iFirst) .. "/" .. tostring(iHeight) .. "/" .. tostring(iA) .. "/" .. tostring(iSens))
    check("views: layout order incl. views", sameList(withoutButtons(list), expectedStandard(core)))
    local wa, wb = core.ui.viewWidgets["test.edit.a"], core.ui.viewWidgets["test.edit.b"]
    check("views: advanced view in its own group (Camera), developer view hidden", wb and wb.path == "/utpp/g_frame"
      and core.ui.viewWidgets["test.dev"] == nil)
    check("views: widget shows get(), default(), limits", wa and wa.kind == "float" and wa.value == 2.5 and wa.default == 1 and wa.min == 0
      and wa.max == 5 and wa.step == 0.05 and wa.format == "%.2f")
    ns.userChange(wa, 3)
    check("views: widget change -> set once, no Save, no apply", #sets == 1 and sets[1] == "test.edit.a=3" and saves.save == 0 and #LOG == 0)
    store.a, store.b = 4.25, 0.3
    sets = {}
    check("views: RefreshView(prefix) updates matching widgets", core.RefreshView("test.edit.") == 2 and wa.value == 4.25 and wb.value == 0.3)
    check("views: refresh never calls set, Save or apply (callbacks inside the sync are dropped)", #sets == 0 and saves.save == 0 and #LOG == 0)
    store.a = 1.5
    check("views: RefreshView(exact id) only that view", core.RefreshView("test.edit.a") == 1 and wa.value == 1.5 and wb.value == 0.3)
    check("views: re-registering an id adds no second widget", core.RegisterView(view("test.edit.a", "a", { group = "frame", level = "player", rank = 5 })) == true
      and #shown(core, ns) == #list)
    check("views: late view placed at its rank", core.RegisterView(view("test.edit.c", "b", { group = "frame", level = "player", rank = 12, type = "int", min = 0, max = 9, step = 1 })) == true
      and sameList(withoutButtons(shown(core, ns)), expectedStandard(core)) and #ns.errors == 0)
    core.SetMenuView("all")
    ns.closeTab()
    core.MenuTick()
    check("views: not shown in the view all", core.ui.viewWidgets["test.edit.a"] == nil and core.RefreshView("test.") == 0)
    core.SetMenuView("standard")
    ns.closeTab()
    core.MenuTick()
    check("views: back in the standard view", core.ui.viewWidgets["test.edit.a"] ~= nil and core.ui.viewWidgets["test.edit.a"].value == 1.5
      and #ns.errors == 0)
    check("views: no Save from building, switching or refreshing", saves.save == 0)
    local nsOff = loadCore()
    check("views: RefreshView without Native Settings -> 0", nsOff.RefreshView("test.") == 0)
  end

  -- ---------------------------------------------------------------- "Diagnostic logs" (core.diagnosticLogs; the list replaces "Turn all logs on")
  local SIX = { "core.log", "mm.trace", "mm.feedDump", "camera.trace", "perspective.log" } -- 5 since 2026-10-30 (mm.debug = on-screen window, not held)
  local function natRecorder()
    local calls = {}
    local api = {}
    for _, f in ipairs({ "setReloadDiag", "setVaultLog", "setFeetDiag" }) do
      api[f] = function(on) calls[#calls + 1] = f .. "=" .. tostring(on) end
    end
    return api, calls
  end
  local function sixState(core)
    local out = {}
    for _, k in ipairs(SIX) do out[#out + 1] = k .. "=" .. tostring(core.Get(k)) end
    return table.concat(out, ",")
  end
  local function sixAll(core, want)
    for _, k in ipairs(SIX) do
      local w = want
      if w == nil then w = core.settings.defs[k].default end
      if core.Get(k) ~= w then return false end
    end
    return true
  end
  local function logHas(line)
    for _, l in ipairs(LOG) do if l == line then return true end end
    return false
  end
  local DEFAULTS_SIX = "core.log=true,mm.trace=false,mm.feedDump=false,camera.trace=false,perspective.log=true"

  -- registry and placement
  do
    local core = fullBoot()
    local def = core.settings.defs["core.diagnosticLogs"]
    local ids = {}
    for _, c in ipairs(def and def.choices or {}) do ids[#ids + 1] = c[1] .. "=" .. c[2] end
    check("diagnostic logs: registered choice, default off, label 'Diagnostic logs', section general, three values",
      def ~= nil and def.type == "choice" and def.default == "off" and def.label == "Diagnostic logs" and def.section == "general"
      and core.Get("core.diagnosticLogs") == "off"
      and table.concat(ids, ",") == "off=Off,timed=On for 5 minutes,until=On until switched off", table.concat(ids, ","))
    check("diagnostic logs: the old switch 'Turn all logs on' is gone", core.settings.defs["core.allLogs"] == nil and core.allLogs == nil)
    check("diagnostic logs: desc says what it is for, the 5 minutes, the report file and the restart", def ~= nil
      and string.find(def.desc, "For reporting a problem", 1, true) ~= nil and string.find(def.desc, "5 minutes of gameplay", 1, true) ~= nil
      and string.find(def.desc, "utpp-report-(date).txt", 1, true) ~= nil and string.find(def.desc, "red4ext\\logs", 1, true) ~= nil
      and string.find(def.desc, "also after a game restart", 1, true) ~= nil)
    local level, group, rank, source = core.MenuPlacement("core.diagnosticLogs")
    check("diagnostic logs: placement player / start (General) / rank 70, last in General (layout)", level == "player" and group == "start"
      and rank == 70 and source == "layout", tostring(level) .. "/" .. tostring(group) .. "/" .. tostring(rank) .. "/" .. tostring(source))
    local okD = true
    for _, k in ipairs(SIX) do if core.MenuPlacement(k) ~= "diagnostic" then okD = false end end
    check("diagnostic logs: the single switches stay diagnostic level", okD)
    check("diagnostic logs: fresh boot keeps every diagnostic at its default (release stays quiet)", sixState(core) == DEFAULTS_SIX
      and core.diag.mode == "off" and core.diag.calls == 0, sixState(core))
    local ns = newNS()
    LOG = {}
    core.BuildNativeSettings(ns)
    local w = core.settings.widgets["core.diagnosticLogs"]
    check("diagnostic logs: standard view shows it in General as a selector, off", w ~= nil and w.path == "/utpp/g_start" and w.kind == "selector"
      and w.value == 1 and w.label == "Diagnostic logs", w and (tostring(w.path) .. "/" .. tostring(w.kind) .. "/" .. tostring(w.value)))
    local lastStart
    for _, s in ipairs(withoutButtons(shown(core, ns))) do
      local key = string.match(s, "^g_start|(.*)$")
      if key then lastStart = key end
    end
    check("diagnostic logs: last option of General", lastStart == "core.diagnosticLogs", tostring(lastStart))
    local hidden = true
    for _, k in ipairs(SIX) do if core.settings.widgets[k] ~= nil then hidden = false end end
    check("diagnostic logs: the single switches still have no standard widget", hidden)
    check("diagnostic logs: building calls no apply / native", #LOG == 0 and core.diag.calls == 0, table.concat(LOG, "; "))
  end

  -- on / hold / off through Set
  do
    local core = fullBoot()
    local api, nat = natRecorder()
    core.diag.api = api
    local saves = countSave(core)
    LOG = {}
    core.Set("core.diagnosticLogs", "until")
    check("diagnostic logs on: the five switches on", sixAll(core, true), sixState(core))
    check("diagnostic logs on: their apply callbacks ran (mm.trace, mm.feedDump, camera.trace, perspective.log)",
      applyCount("mm.trace") == 1 and applyCount("mm.feedDump") == 1 and applyCount("camera.trace") == 1
      and applyCount("perspective.log") == 1, table.concat(LOG, "; "))
    check("diagnostic logs on: mm.debug (UTPP motion window) not held on", core.Get("mm.debug") == false and applyCount("mm.debug") == 0)
    check("diagnostic logs on: motion matching trace and feed dump natives switched on", logHas("mm setTrace(true)")
      and logHas("mm setFeedDump(true)"), table.concat(LOG, "; "))
    check("diagnostic logs on: reload diagnostic, vehicle vault log, feet diagnostic natives on",
      table.concat(nat, ",") == "setReloadDiag=true,setVaultLog=true,setFeetDiag=true", table.concat(nat, ","))
    check("diagnostic logs on: one Save, stored on", saves.save == 1 and string.find(readFile(tmpSettings) or "", "core.diagnosticLogs=until", 1, true) ~= nil)
    core.Set("mm.trace", false)
    core.Set("core.log", false)
    core.Set("camera.trace", false)
    check("diagnostic logs on: single switches cannot be turned off while it is on", sixAll(core, true), sixState(core))
    local n0 = #nat
    core.Set("core.diagnosticLogs", "until")
    check("diagnostic logs on: setting on again calls no native", #nat == n0)
    LOG = {}
    for i = #nat, 1, -1 do nat[i] = nil end
    core.Set("core.diagnosticLogs", "off")
    check("diagnostic logs off: each of the five back to its registered default (core.log and perspective.log on)",
      sixState(core) == DEFAULTS_SIX, sixState(core))
    check("diagnostic logs off: trace and feed dump natives off", logHas("mm setTrace(false)") and logHas("mm setFeedDump(false)"),
      table.concat(LOG, "; "))
    check("diagnostic logs off: natives off (reload diagnostic follows mm.trace = off)",
      table.concat(nat, ",") == "setReloadDiag=false,setVaultLog=false,setFeetDiag=false", table.concat(nat, ","))
    for i = #nat, 1, -1 do nat[i] = nil end
    core.Set("core.diagnosticLogs", "off")
    check("diagnostic logs off: off again calls no native", #nat == 0)
    core.Set("mm.trace", true)
    check("diagnostic logs off: single switches work on their own again", core.Get("mm.trace") == true and #nat == 0)
  end

  -- stored "until switched off": the game order (core registered, file read, then the other modules register)
  do
    os.remove(tmpSettings)
    LOG = {}
    local core = loadCore()
    local api, nat = natRecorder()
    core.diag.api = api
    local f = io.open(tmpSettings, "wb")
    f:write("core.diagnosticLogs=until\ncore.log=false\nmm.trace=false\nmm.feedDump=false\ncamera.trace=false\nperspective.log=false\n"
      .. "mm.debug=false\n")
    f:close()
    core.Load(tmpSettings)
    check("diagnostic logs stored on: load applies it (core.log held on; no session native before a game session exists)",
      core.Get("core.diagnosticLogs") == "until" and core.Get("core.log") == true and core.diag.mode == "until" and #nat == 0,
      table.concat(nat, ","))
    hookRegister(core)
    bootModules(core)
    check("diagnostic logs stored on: later modules register their switches on (stored off ignored)", sixAll(core, true), sixState(core))
    check("diagnostic logs stored on: mm trace / feed dump natives on at registration", logHas("mm setTrace(true)")
      and logHas("mm setFeedDump(true)"), table.concat(LOG, "; "))
    check("diagnostic logs stored on: no session native during the boot (utpp_motion arms them per session)", #nat == 0, table.concat(nat, ","))
    core.Load(tmpSettings)
    check("diagnostic logs stored on: a second Load keeps the switches held", sixAll(core, true) and #nat == 0, sixState(core))
    -- ResetAll walks the registration order: the switches before core.diagnosticLogs stay held, its own reset restores them
    for i = #nat, 1, -1 do nat[i] = nil end
    core.ResetAll()
    check("diagnostic logs: ResetAll while on -> off, the five at their defaults", core.Get("core.diagnosticLogs") == "off"
      and sixState(core) == DEFAULTS_SIX, sixState(core))
    check("diagnostic logs: ResetAll natives off", table.concat(nat, ",") == "setReloadDiag=false,setVaultLog=false,setFeetDiag=false",
      table.concat(nat, ","))
    os.remove(tmpSettings)
  end

  -- The continuous helper-bone capture (Game.UTPPBodyPoseTraceContinuous, hundreds of MB per minute) is not part of the diagnostic
  -- logs (release build: the list must not start a multi-GB writer); an injected api.setBodyTrace is never called, on or off.
  do
    local core = fullBoot()
    local api, nat = natRecorder()
    local bodyCalls = 0
    api.setBodyTrace = function(on) bodyCalls = bodyCalls + 1; nat[#nat + 1] = "setBodyTrace=" .. tostring(on) end
    core.diag.api = api
    core.Set("core.diagnosticLogs", "until")
    check("diagnostic logs on: only the three session diagnostics are switched on, the body pose trace is not",
      table.concat(nat, ",") == "setReloadDiag=true,setVaultLog=true,setFeetDiag=true" and bodyCalls == 0, table.concat(nat, ","))
    core.Set("core.diagnosticLogs", "until")
    check("diagnostic logs on: setting on again calls no native", #nat == 3)
    core.Set("core.diagnosticLogs", "off")
    check("diagnostic logs off: the three session diagnostics are switched off, the body pose trace native is never called",
      #nat == 6 and nat[6] == "setFeetDiag=false" and bodyCalls == 0, table.concat(nat, ","))
    check("diagnostic logs: natives are called without errors", core.diag.calls == 6 and core.diag.errors == 0, tostring(core.diag.errors))
  end
  -- stored off: individually stored diagnostic switches are kept, no native
  do
    os.remove(tmpSettings)
    LOG = {}
    local core = loadCore()
    local api, nat = natRecorder()
    core.diag.api = api
    local f = io.open(tmpSettings, "wb")
    f:write("core.diagnosticLogs=off\nmm.trace=true\ncamera.trace=true\ncore.log=false\n")
    f:close()
    core.Load(tmpSettings)
    hookRegister(core)
    bootModules(core)
    check("diagnostic logs stored off: stored single switches kept (mm.trace, camera.trace on, core.log off)", core.Get("mm.trace") == true
      and core.Get("camera.trace") == true and core.Get("core.log") == false and core.Get("mm.feedDump") == false)
    check("diagnostic logs stored off: no diagnostic native called", #nat == 0 and core.diag.calls == 0, table.concat(nat, ","))
    os.remove(tmpSettings)
  end
end

-- ================================================================ T03 stored values, hidden options, followers, defaults revision
do
  local function writeStored(text)
    local f = assert(io.open(tmpSettings, "wb"))
    f:write(text)
    f:close()
  end
  -- the game's order: utpp_core registers its own options and reads the file, the other modules register afterwards; the
  -- apply callbacks are recorded from the start (the ones of the options registered before the file is read as well)
  local function bootWithFile(text, opts)
    os.remove(tmpSettings)
    LOG = {}
    coreLines = {}
    local core = loadCore()
    for _, key in ipairs(core.settings.order) do wrapApply(core.settings.defs[key]) end
    hookRegister(core)
    if text then
      writeStored(text)
      core.Load(tmpSettings)
    end
    local mods = bootModules(core, opts)
    return core, mods
  end
  local function lastApplied()
    local out = {}
    for _, line in ipairs(LOG) do
      local key, arg = string.match(line, '^apply ([^%(]+)%((.*),"%a+"%)$')
      if key then out[key] = arg end
    end
    return out
  end
  local function count(text, needle)
    local n, from = 0, 1
    while true do
      local a, b = string.find(text, needle, from, true)
      if not a then return n end
      n, from = n + 1, b + 1
    end
  end
  local function defaultsLines()
    local n = 0
    for _, l in ipairs(coreLines) do if string.find(l, "settings defaults revision", 1, true) then n = n + 1 end end
    return n
  end

  -- ---------------------------------------------------------------- every registered key gets its stored value
  -- Keys with their own start path and a value that cannot be arbitrary are left out here (they have their own tests):
  -- core.diagnosticLogs ("On for 5 minutes" becomes "off" on a restart, "until" holds the diagnostic switches) and
  -- animation.pack (the pack in force forces eight other options).
  local EXCLUDED = { ["core.diagnosticLogs"] = true, ["animation.pack"] = true }
  do
    local probe = bootWithFile(nil)
    local lines, chosen = {}, {}
    for _, key in ipairs(probe.settings.order) do
      local def = probe.settings.defs[key]
      if not EXCLUDED[key] then
        local text
        if def.type == "bool" then text = tostring(not def.default)
        elseif def.type == "choice" then
          for _, c in ipairs(def.choices) do if c[1] ~= def.default then text = c[1]; break end end
        else
          local other
          if def.max ~= nil and def.default ~= def.max then other = def.max
          elseif def.min ~= nil and def.default ~= def.min then other = def.min
          else other = def.default + 1 end
          text = string.format("%.9g", other)
        end
        if text ~= nil then
          chosen[key] = probe.ValidateSetting(def, text)
          lines[#lines + 1] = key .. "=" .. text
        end
      end
    end
    -- a follower holds the value of its authority (see T03 followers)
    for follower, authority in pairs(probe.FOLLOWS or { ["vehicle.keepTpp"] = "perspective.vehicleKeepTpp" }) do chosen[follower] = chosen[authority] end
    local nKeys = #lines
    -- a file of the current defaults revision (a stored value equal to an old default is a choice, not a leftover)
    table.insert(lines, 1, "core.defaultsRevision=" .. tostring(probe.DEFAULTS_REVISION or 1))
    check("T03 stored values: every key of the whole registry could be given a non-default value (" .. nKeys .. " keys)",
      nKeys >= 150 and nKeys == #probe.settings.order - 2, nKeys .. " of " .. #probe.settings.order)

    local core = bootWithFile(table.concat(lines, "\n") .. "\n")
    local last = lastApplied()
    local wrong, notApplied = {}, {}
    for key, want in pairs(chosen) do
      local def = core.settings.defs[key]
      if def == nil then wrong[#wrong + 1] = key .. " (not registered)"
      else
        if core.Get(key) ~= want then wrong[#wrong + 1] = key .. "=" .. tostring(core.Get(key)) .. " want " .. tostring(want) end
        -- the callback must have seen the stored value, not the default it was registered with
        if def.apply ~= nil and last[key] ~= ser(core.Effective(key)) then
          notApplied[#notApplied + 1] = key .. " (last apply " .. tostring(last[key]) .. ", value " .. ser(core.Effective(key)) .. ")"
        end
      end
    end
    table.sort(wrong)
    table.sort(notApplied)
    check("T03 stored values: after a restart every registered key reads back its stored value", #wrong == 0, table.concat(wrong, "; "))
    check("T03 stored values: the apply callback of every key last ran with the stored value, whatever the registration order",
      #notApplied == 0, table.concat(notApplied, "; "))
    -- the options that utpp_core registers before it reads the file: their callbacks fill the head look / loot view config
    local hl, la = core.hl.cfg, core.la.cfg
    check("T03 stored values: head look options reach the head look config (faceCamera, whileWalking, lookAtLoot, maxYaw)",
      hl.faceCamera == chosen["headLook.faceCamera"] and hl.whileWalking == chosen["headLook.whileWalking"]
      and hl.lookAtLoot == chosen["headLook.lookAtLoot"] and hl.maxYaw == chosen["headLook.maxYaw"] and hl.faceCamera ~= true,
      tostring(hl.faceCamera) .. " " .. tostring(hl.whileWalking) .. " " .. tostring(hl.lookAtLoot) .. " " .. tostring(hl.maxYaw))
    check("T03 stored values: loot view options reach the loot view config (zoom, radius)",
      la.zoom == chosen["lootView.zoom"] and la.radius == chosen["lootView.radius"] and la.zoom ~= 0.9 and la.radius ~= 1.8,
      tostring(la.zoom) .. " " .. tostring(la.radius))
    -- one concrete value that is neither the default nor a limit
    local core2 = bootWithFile("headLook.faceCamera=false\nheadLook.maxYaw=60\nlootView.zoom=0.3\nlootView.radius=1.2\n")
    check("T03 stored values: a stored head look limit / loot view zoom is what the config holds after the start",
      core2.hl.cfg.faceCamera == false and core2.hl.cfg.maxYaw == 60 and math.abs(core2.la.cfg.zoom - 0.3) < 1e-9
      and math.abs(core2.la.cfg.radius - 1.2) < 1e-9 and core2.hl.cfg.whileWalking == true)
    -- Load twice: no callback runs again for a value that did not change
    local before = #LOG
    core2.Load(tmpSettings)
    check("T03 stored values: loading the same file again runs no callback", #LOG == before, #LOG - before)
  end

  -- ---------------------------------------------------------------- options hidden from the player menu keep working
  do
    local core = bootWithFile("perspective.blend=1.6\nmm.headLead=false\nvehicle.keepTpp=false\n")
    local names = { "perspective.blend", "mm.headLead" }
    local ok = true
    for _, key in ipairs(names) do
      local def = core.settings.defs[key]
      ok = ok and def ~= nil and def.hidden == true and core.MenuPlacement(key) == "hidden" and core.settings.widgets[key] == nil
    end
    check("T03 hidden options: perspective.blend and mm.headLead are registered, flagged hidden and have no widget", ok)
    check("T03 hidden options: their stored values are read (no error, no change of the published key or default)",
      core.Get("perspective.blend") == 1.6 and core.Get("mm.headLead") == false
      and core.settings.defs["perspective.blend"].default == 0.8 and core.settings.defs["mm.headLead"].default == true)
    core.Save(tmpSettings)
    local text = readFile(tmpSettings)
    check("T03 hidden options: the next Save keeps both keys in the file", string.find(text, "perspective.blend=1.6", 1, true) ~= nil
      and string.find(text, "mm.headLead=false", 1, true) ~= nil)
  end

  -- ---------------------------------------------------------------- followers: one authority for the vehicle camera switch
  do
    local function followCase(authorityFirst)
      os.remove(tmpSettings)
      local core = loadCore()
      writeStored("perspective.vehicleKeepTpp=false\nvehicle.keepTpp=true\n")
      core.Load(tmpSettings)
      local applied = {}
      local function reg(key)
        core.Register({ key = key, section = "vehicle", type = "bool", default = true, label = key,
          apply = function(v) applied[key] = v end })
      end
      if authorityFirst then reg("perspective.vehicleKeepTpp"); reg("vehicle.keepTpp")
      else reg("vehicle.keepTpp"); reg("perspective.vehicleKeepTpp") end
      local tag = authorityFirst and "authority registered first" or "follower registered first"
      check("T03 follower (" .. tag .. "): the follower holds the authority's value and its callback saw it",
        core.Get("vehicle.keepTpp") == false and core.Effective("vehicle.keepTpp") == false and applied["vehicle.keepTpp"] == false
        and core.Get("perspective.vehicleKeepTpp") == false)
      core.Set("perspective.vehicleKeepTpp", true, { noSave = true })
      check("T03 follower (" .. tag .. "): setting the authority runs the follower's callback",
        core.Get("vehicle.keepTpp") == true and applied["vehicle.keepTpp"] == true)
      local v = core.Set("vehicle.keepTpp", false)
      check("T03 follower (" .. tag .. "): setting the follower sets the authority",
        v == false and core.Get("perspective.vehicleKeepTpp") == false and core.Get("vehicle.keepTpp") == false
        and applied["perspective.vehicleKeepTpp"] == false and applied["vehicle.keepTpp"] == false)
      local text = readFile(tmpSettings)
      check("T03 follower (" .. tag .. "): both keys stay in the file with the same value",
        string.find(text, "perspective.vehicleKeepTpp=false", 1, true) ~= nil and string.find(text, "vehicle.keepTpp=false", 1, true) ~= nil)
      core.ResetAll()
      check("T03 follower (" .. tag .. "): Defaults restores both", core.Get("vehicle.keepTpp") == true
        and core.Get("perspective.vehicleKeepTpp") == true)
    end
    followCase(true)
    followCase(false)

    -- the real modules, both registration orders: the vehicle camera hold follows the visible option
    local core, mods = bootWithFile("perspective.vehicleKeepTpp=false\nvehicle.keepTpp=true\n")
    check("T03 follower (real modules): the vehicle camera hold and the perspective rule both follow the visible option",
      mods.vehiclecam.ctrl.cfg.keepTpp == false and mods.perspective.cfg.vehicleKeepTpp == false
      and core.Get("vehicle.keepTpp") == false)
    sandboxIo() -- the vehicle camera writes its own utpp_settings.txt next to the working directory when the hold changes
    core.Set("perspective.vehicleKeepTpp", true, { noSave = true })
    unsandboxIo()
    check("T03 follower (real modules): switching the visible option on turns the hold on",
      mods.vehiclecam.ctrl.cfg.keepTpp == true and mods.perspective.cfg.vehicleKeepTpp == true)
    local core3, mods3 = bootWithFile("perspective.vehicleKeepTpp=false\nvehicle.keepTpp=true\n", { skip = { perspective = true } })
    check("T03 follower (real modules, vehicle camera first): the hold starts on its own stored value",
      mods3.vehiclecam.ctrl.cfg.keepTpp == true)
    registerForEvent, registerInput = nil, nil
    local persp = dofile(UTPP_PERSPECTIVE_PATH)
    sandboxIo()
    persp.AttachCore(core3)
    unsandboxIo()
    restoreGlobals()
    check("T03 follower (real modules, vehicle camera first): the perspective option registering later turns the hold off",
      mods3.vehiclecam.ctrl.cfg.keepTpp == false and persp.cfg.vehicleKeepTpp == false and core3.Get("vehicle.keepTpp") == false)
  end

  -- ---------------------------------------------------------------- defaults revision
  do
    local REV = "core.defaultsRevision=" .. tostring(2)
    local oldFile = bootWithFile("melee.guard=false\nmm.headLead=false\n")
    local onDisk = readFile(tmpSettings) or ""
    check("T03 defaults revision: the moved value is written back at once (with the revision) and the other keys stay in the file",
      count(onDisk, "core.defaultsRevision=2") == 1 and string.find(onDisk, "melee.guard=true", 1, true) ~= nil
      and string.find(onDisk, "mm.headLead=false", 1, true) ~= nil)
    check("T03 defaults revision: the registered default is on", oldFile.settings.defs["melee.guard"].default == true)
    check("T03 defaults revision: a file without a revision with the old default stored (false) moves to the new default (once)",
      oldFile.Get("melee.guard") == true and defaultsLines() == 1, tostring(oldFile.Get("melee.guard")) .. " lines=" .. defaultsLines())
    local logged = ""
    for _, l in ipairs(coreLines) do if string.find(l, "settings defaults revision", 1, true) then logged = l end end
    check("T03 defaults revision: the log line names the revisions and the key",
      string.find(logged, "0 -> 2", 1, true) ~= nil and string.find(logged, "melee.guard false -> true", 1, true) ~= nil, logged)
    check("T03 defaults revision: another stored value of the same file is untouched", oldFile.Get("mm.headLead") == false)
    oldFile.Save(tmpSettings)
    local saved = readFile(tmpSettings)
    check("T03 defaults revision: Save writes the revision once and the new value", count(saved, "core.defaultsRevision=") == 1
      and string.find(saved, REV, 1, true) ~= nil and string.find(saved, "melee.guard=true", 1, true) ~= nil)
    check("T03 defaults revision: the revision is not a stored setting", oldFile.settings.stored["core.defaultsRevision"] == nil
      and oldFile.Get("core.defaultsRevision") == nil)

    -- loading the saved file again: nothing moves, nothing is logged
    local again = bootWithFile(saved)
    check("T03 defaults revision: loading the file written after the move changes nothing and logs nothing",
      again.Get("melee.guard") == true and defaultsLines() == 0 and again.Get("mm.headLead") == false)

    -- a choice made after the move survives restarts
    again.Set("melee.guard", false)
    local chosen = readFile(tmpSettings)
    local restart = bootWithFile(chosen)
    check("T03 defaults revision: a later choice of the old default is kept after a restart (no second move)",
      restart.Get("melee.guard") == false and defaultsLines() == 0 and count(chosen, "core.defaultsRevision=") == 1)

    -- a file of the current revision with the old default is a choice
    local newFile = bootWithFile(REV .. "\nmelee.guard=false\n")
    check("T03 defaults revision: a file of the current revision keeps a stored false", newFile.Get("melee.guard") == false and defaultsLines() == 0)

    -- other stored values and absent keys
    local trueFile = bootWithFile("melee.guard=true\n")
    check("T03 defaults revision: an old file with the new value stored stays on, nothing logged",
      trueFile.Get("melee.guard") == true and defaultsLines() == 0)
    local absent = bootWithFile("mm.headLead=false\n")
    check("T03 defaults revision: key absent from an old file: the new default applies, nothing logged",
      absent.Get("melee.guard") == true and defaultsLines() == 0)
    local fresh = bootWithFile(nil)
    fresh.Save(tmpSettings)
    check("T03 defaults revision: a fresh install gets the new default and the file carries the revision",
      fresh.Get("melee.guard") == true and defaultsLines() == 0 and string.find(readFile(tmpSettings), REV, 1, true) ~= nil)

    -- a newer revision written by another build is kept
    local newer = bootWithFile("core.defaultsRevision=7\nmelee.guard=false\n")
    newer.Save(tmpSettings)
    check("T03 defaults revision: a higher revision in the file is kept and no move happens", newer.Get("melee.guard") == false
      and string.find(readFile(tmpSettings), "core.defaultsRevision=7", 1, true) ~= nil)
    -- an old default that was a number is compared by value
    check("T03 defaults revision: every listed change names a registered key with a valid new default", (function()
      for _, change in ipairs(oldFile.DEFAULT_CHANGES or {}) do
        local def = oldFile.settings.defs[change.key]
        if def == nil or def.default ~= change.to or change.rev > (oldFile.DEFAULTS_REVISION or 0) then return false end
      end
      return oldFile.DEFAULT_CHANGES ~= nil and #oldFile.DEFAULT_CHANGES > 0
    end)())
  end
end

os.remove(tmpSettings)
os.remove(tmpModFile)
restoreGlobals()
outPrint(string.format("utpp_settings_projection test: %d passed, %d failed", passed, failed))
if failed > 0 then error("utpp_settings_projection: " .. failed .. " check(s) failed") end

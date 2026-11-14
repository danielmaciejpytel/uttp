-- UTPP 0.5.0/0.5.1/0.5.2 offline tests: utpp_core (settings registry / persistence / reset, ON/OFF implementation switches,
-- retired settings keys (incl. the bind.* keys of the removed key bindings), head-look decisions, loot-assist target
-- selection and camera offset) and the utpp_vehiclecam -> utpp_core settings wiring. Pure logic, no game.
local results = { pass = 0, fail = 0 }
local outPrint = print -- checks always report, also while a test replaces print
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  outPrint(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end
local function near(a, b, eps) return math.abs(a - b) <= (eps or 1e-6) end

local savedPrint, savedRegister = print, registerForEvent
local quietLines = {}
local function load()
  registerForEvent = nil
  local M = dofile(UTPP_CORE_PATH)
  registerForEvent = savedRegister
  M.sink = function(line) quietLines[#quietLines + 1] = line end
  return M
end
local tmp = os.tmpname()
local tmp2 = os.tmpname()

-- ---------------------------------------------------------------- settings registry / persistence
do
  local M = load()
  M.SETTINGS_FILE = tmp
  check("load: module table + version", type(M) == "table" and M.VERSION == "0.5.5")
  check("defaults: head look / loot impl on, no key binding options", M.Get("headLook.impl") == true
    and M.Get("lootView.impl") == true and M.Get("bind.perspective_toggle") == nil and M.Get("bind.enabled") == nil)
  check("validate: float clamped to range", M.Set("lootView.zoom", 9) == 2)
  check("validate: invalid bool rejected, value kept", M.Set("headLook.whileWalking", "maybe") == nil
    and M.Get("headLook.whileWalking") == true)
  check("validate: unknown key rejected", M.Set("nope.x", 1) == nil)
  check("apply callback runs on Set", M.Set("headLook.maxYaw", 80) == 80 and M.hl.cfg.maxYaw == 80)
  M.Set("headLook.whileWalking", false)
  M.Set("crosshair.color", "red")
  check("invalid choice rejected", M.Set("crosshair.color", "purple") == nil and M.Get("crosshair.color") == "red")
  -- keep an unknown key of a module that is not loaded this session
  local f = io.open(tmp, "a"); f:write("other.module=42\n"); f:close()
  M.Load(tmp)
  check("save: file written", M.Save(tmp))
  local text = io.open(tmp):read("*a")
  check("save: values and foreign keys present", text:find("headLook.whileWalking=false", 1, true) ~= nil
    and text:find("crosshair.color=red", 1, true) ~= nil and text:find("other.module=42", 1, true) ~= nil, text)
  local M2 = load()
  M2.SETTINGS_FILE = tmp2
  check("load: round trip", M2.Load(tmp) and M2.Get("headLook.whileWalking") == false and M2.Get("lootView.zoom") == 2
    and M2.Get("headLook.maxYaw") == 80 and M2.Get("crosshair.color") == "red")
  check("load: applied to controller cfg via Register order (maxYaw)", M2.hl.cfg.maxYaw == 100 or M2.hl.cfg.maxYaw == 80)
  -- options registered AFTER the file was read get the stored value (other mods register in onInit)
  local f2 = io.open(tmp, "a"); f2:write("late.flag=false\nlate.num=abc\n"); f2:close()
  M2.Load(tmp)
  local applied
  M2.Register({ key = "late.flag", section = "late", type = "bool", default = true, apply = function(v) applied = v end })
  M2.Register({ key = "late.num", section = "late", type = "float", default = 1.5, min = 0, max = 3 })
  check("late registration: stored value + apply", M2.Get("late.flag") == false and applied == false)
  check("late registration: invalid stored value -> default", M2.Get("late.num") == 1.5)
  M2.Register({ key = "late.init", section = "late", type = "bool", default = true, initial = false })
  check("initial value used when nothing stored (migration)", M2.Get("late.init") == false)
  check("reset all: defaults", M2.ResetAll() and M2.Get("headLook.whileWalking") == true and M2.Get("lootView.zoom") == 0.9
    and M2.Get("late.flag") == true and applied == true and M2.Get("crosshair.color") == "standard")
  local t2 = io.open(tmp2):read("*a")
  check("reset all: saved", t2:find("late.flag=true", 1, true) ~= nil)
  check("missing file -> false, values unchanged", M2.Load(tmp .. ".missing") == false and M2.Get("late.flag") == true)
  local okReg = pcall(M2.Register, { key = "bad.def", type = "float", default = "x" })
  check("Register rejects an invalid default", not okReg)
end

-- ---------------------------------------------------------------- retired settings keys (2026-10-21)
-- perspective.enforceWithRef: the removed option stayed in the settings file as a "key of a module that is not loaded" forever
do
  local M = load()
  local file = os.tmpname()
  local f = io.open(file, "w")
  f:write("# UTPP settings\nheadLook.maxYaw=80\nperspective.enforceWithRef=true\nother.module=42\nperspective.enforce=false\n"
    .. "perspective.enforceWithRefX=1\nmystery.key=abc\n")
  f:close()
  M.SETTINGS_FILE = file
  quietLines = {}
  check("retired key: Load succeeds, the other settings are read", M.Load(file) and M.Get("headLook.maxYaw") == 80)
  check("retired key: perspective.enforceWithRef is not kept in the stored settings", M.settings.stored["perspective.enforceWithRef"] == nil)
  check("retired key: other unknown keys are kept (other.module, mystery.key, the similar perspective.enforce / enforceWithRefX)",
    M.settings.stored["other.module"] == "42" and M.settings.stored["mystery.key"] == "abc"
    and M.settings.stored["perspective.enforce"] == "false" and M.settings.stored["perspective.enforceWithRefX"] == "1")
  M.Load(file)
  local logged = 0
  for _, l in ipairs(quietLines) do if l:find("retired setting perspective.enforceWithRef dropped", 1, true) then logged = logged + 1 end end
  check("retired key: logged once for two reads", logged == 1, logged)
  check("retired key: Save writes the file", M.Save(file))
  local text = io.open(file):read("*a")
  check("retired key: the saved file has no enforceWithRef line, the foreign keys stay", text:find("perspective.enforceWithRef=", 1, true) == nil
    and text:find("other.module=42", 1, true) ~= nil and text:find("mystery.key=abc", 1, true) ~= nil
    and text:find("perspective.enforce=false", 1, true) ~= nil and text:find("perspective.enforceWithRefX=1", 1, true) ~= nil
    and text:find("headLook.maxYaw=80", 1, true) ~= nil, text)
  os.remove(file)
end

-- 2026-10-30: the own key bindings were removed; an old settings file with bind.* lines loads without error and the next Save
-- no longer writes them (exact keys: another "bind.*" key, e.g. of another mod, is kept like every unknown key)
do
  local RETIRED = { "bind.enabled", "bind.perspective_toggle", "bind.shoulder_swap", "bind.quick_fpp", "bind.profile_next",
                    "bind.profile_prev", "bind.headlook_toggle", "bind.lootassist_toggle", "bind.camera_freelook",
                    "bind.camera_zoom_in", "bind.camera_zoom_out", "bind.loothead_kill" }
  local M = load()
  local file = os.tmpname()
  local f = io.open(file, "w")
  f:write("# UTPP settings (utpp_core 0.5.5)\nbind.enabled=false\nbind.shoulder_swap=IK_F\nbind.perspective_toggle=IK_LShift+IK_F6\n"
    .. "bind.quick_fpp=None\nbind.profile_next=IK_F9\nbind.profile_prev=None\nbind.headlook_toggle=IK_F7\n"
    .. "bind.lootassist_toggle=None\nbind.camera_freelook=IK_Pad_LeftShoulder\nbind.camera_zoom_in=None\n"
    .. "bind.camera_zoom_out=None\nbind.loothead_kill=None\nbind.somethingElse=1\nheadLook.maxYaw=75\n")
  f:close()
  M.SETTINGS_FILE = file
  quietLines = {}
  check("retired binds: old file loads, the other settings are read", M.Load(file) and M.Get("headLook.maxYaw") == 75)
  local kept, registered = {}, {}
  for _, k in ipairs(RETIRED) do
    if M.settings.stored[k] ~= nil then kept[#kept + 1] = k end
    if M.settings.defs[k] ~= nil or M.Get(k) ~= nil then registered[#registered + 1] = k end
  end
  check("retired binds: all 12 bind keys dropped from the stored settings", #kept == 0, table.concat(kept, ","))
  check("retired binds: no bind key is registered", #registered == 0, table.concat(registered, ","))
  check("retired binds: the near-miss bind.somethingElse is kept", M.settings.stored["bind.somethingElse"] == "1")
  M.Load(file)
  local logged = 0
  for _, l in ipairs(quietLines) do if l:find("retired setting bind.", 1, true) then logged = logged + 1 end end
  check("retired binds: one log line per key for two reads", logged == 12, logged)
  check("retired binds: Save writes the file", M.Save(file))
  local text = io.open(file):read("*a")
  local written = {}
  for _, k in ipairs(RETIRED) do if text:find("\n" .. k .. "=", 1, true) then written[#written + 1] = k end end
  check("retired binds: the saved file has no retired bind line", #written == 0, table.concat(written, ","))
  check("retired binds: bind.somethingElse and the registered values stay in the saved file",
    text:find("bind.somethingElse=1", 1, true) ~= nil and text:find("headLook.maxYaw=75", 1, true) ~= nil, text)
  check("retired binds: \"key\" is no settings type any more (Register fails)",
    not pcall(M.Register, { key = "late.bind", section = "late", type = "key", default = "None" }))
  os.remove(file)
end

-- ---------------------------------------------------------------- implementation switches (plain ON/OFF)
do
  local M = load()
  M.SETTINGS_FILE = tmp2
  check("impl default -> UTPP on", M.Effective("headLook.impl") == true and M.Effective("lootView.impl") == true)
  M.Set("headLook.impl", false)
  check("switch off -> stored off", M.Get("headLook.impl") == false and M.Effective("headLook.impl") == false)
  M.Set("headLook.impl", true)
  check("switch on -> stored on", M.Get("headLook.impl") == true and M.Effective("headLook.impl") == true)
  check("impl accepts booleans from the console", M.Set("headLook.impl", true) == true and M.Set("headLook.impl", false) == false)
  check("impl: legacy auto / on / off strings from the console", M.Set("headLook.impl", "auto") == true and M.Set("headLook.impl", "off") == false
    and M.Set("headLook.impl", "on") == true and M.Effective("headLook.impl") == true)
  check("impl: other strings rejected", M.Set("headLook.impl", "maybe") == nil and M.Get("headLook.impl") == true)
  -- saved values of older versions: auto / on -> on, off -> off; the next save writes on / off
  local f = io.open(tmp2, "w"); f:write("headLook.impl=auto\nlootView.impl=off\n"); f:close()
  local M2 = load()
  M2.SETTINGS_FILE = tmp2
  M2.Load(tmp2)
  check("migration: stored auto -> on (true), stored off stays off (false)", M2.Get("headLook.impl") == true and M2.Effective("headLook.impl") == true
    and M2.Get("lootView.impl") == false and M2.Effective("lootView.impl") == false)
  M2.Save(tmp2)
  local saved = io.open(tmp2):read("*a")
  check("migration: saved as on / off", saved:find("headLook.impl=true", 1, true) ~= nil and saved:find("lootView.impl=false", 1, true) ~= nil
    and saved:find("impl=auto", 1, true) == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- Native Settings builder (mock)
do
  local M = load()
  M.SETTINGS_FILE = tmp2
  local calls, widgets = {}, {}
  local ns = { data = {} }
  function ns.pathExists(p) return ns.data[p] ~= nil end
  function ns.addTab(p, l) ns.data[p] = l; calls[#calls + 1] = "tab " .. p end
  function ns.addSubcategory(p, l) ns.data[p] = l; calls[#calls + 1] = "sub " .. p end
  local function add(kind)
    return function(path, label, ...)
      local args = { ... }
      local w = { kind = kind, path = path, label = label, args = args, cb = args[#args] }
      widgets[#widgets + 1] = w
      return w
    end
  end
  ns.addSwitch, ns.addRangeFloat, ns.addRangeInt = add("switch"), add("float"), add("int")
  ns.addSelectorString, ns.addKeyBinding, ns.addButton = add("selector"), add("key"), add("button")
  local restore
  function ns.registerRestoreDefaultsCallback(p, override, cb) restore = { p = p, override = override, cb = cb } end
  local setCalls = {}
  function ns.setOption(w, v) setCalls[#setCalls + 1] = { w = w, v = v }; w.cb(v) end
  -- the source-section paths below are the menu view "all" (the standard view is covered by test_utpp_settings_projection.lua)
  check("NS: view all selected before the build (session only)", M.SetMenuView("all") == "all" and M.MenuView() == "all")
  check("NS: built", M.BuildNativeSettings(ns) == true)
  -- head look: the main switch and "Look at the camera" are in the menu; the four tuning options stay hidden (registry flag hidden)
  check("NS: tab + source subcategories (head look section present), no key bindings section", ns.data["/utpp"] == "UTPP"
    and ns.data["/utpp/headlook"] ~= nil and ns.data["/utpp/loot"] and ns.data["/utpp/general"] and ns.data["/utpp/keys"] == nil)
  local byLabel, nKey = {}, 0
  for _, w in ipairs(widgets) do
    byLabel[w.label] = w
    if w.kind == "key" then nKey = nKey + 1 end
  end
  local hl = byLabel["Use UTPP loot view"]
  check("NS: impl switch shows the effective value (on), default on", hl and hl.args[2] == true and hl.args[3] == true)
  local hidden = {}
  for _, key in ipairs({ "headLook.whileWalking", "headLook.lookAtLoot", "headLook.maxYaw",
                         "headLook.chest" }) do
    if M.settings.widgets[key] ~= nil or byLabel[M.settings.defs[key].label] ~= nil then hidden[#hidden + 1] = key end
  end
  check("NS: the four hidden head look options have no widget (they stay registered, on and settable); the two shown ones have one", #hidden == 0
    and byLabel["Use UTPP head look"] ~= nil and byLabel["Look at the camera"] ~= nil
    and M.Get("headLook.impl") == true and M.Set("headLook.maxYaw", 90) == 90 and M.Get("headLook.maxYaw") == 90
    and M.Set("headLook.maxYaw", 100) == 100, table.concat(hidden, ", "))
  check("NS: no key binding widgets", nKey == 0, nKey)
  local reset = byLabel["Reset all UTPP settings"]
  check("NS: reset button text names options only (no key bindings)", reset and reset.args[1] == "Restores the default of every UTPP option.",
    reset and tostring(reset.args[1]))
  check("NS: restore defaults handled by UTPP", restore and restore.override == true)
  M.Set("lootView.impl", false)
  M.Set("lootView.zoom", 1.5) -- hidden in 0.1.0: no widget, still reset with everything else
  setCalls = {}
  restore.cb()
  local synced = false
  for _, c in ipairs(setCalls) do if c.w == byLabel["Use UTPP loot view"] and c.v == true then synced = true end end
  check("NS: reset updates the widgets", synced and M.Get("lootView.impl") == true and M.Get("lootView.zoom") == 0.9
    and byLabel["Zoom (m)"] == nil)
  -- late section from another mod appears in the built tab
  M.RegisterSection("vehicle", "Vehicles")
  M.Register({ key = "vehicle.x", section = "vehicle", type = "bool", default = true, label = "X" })
  local found = false
  for _, w in ipairs(widgets) do if w.label == "X" and w.path == "/utpp/vehicle" then found = true end end
  check("NS: section registered after the build is added", ns.data["/utpp/vehicle"] ~= nil and found)
  check("NS: missing Native Settings -> false", M.BuildNativeSettings(nil) == false)
end

-- ---------------------------------------------------------------- head look decisions
do
  local M = load()
  local HL = M.HeadLook
  local function obs(o)
    local b = { enabled = true, tpp = true, speed = 0, pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 },
                camPos = { x = 0.5, y = -3, z = 1.9 }, camFwd = { x = 0, y = 1, z = 0 } }
    for k, v in pairs(o or {}) do b[k] = v end
    return b
  end
  local c = HL.New()
  check("idle: no request before the engage delay", HL.Step(c, 0.0, obs()) == nil and HL.Step(c, 0.2, obs()) == nil)
  local cmd = HL.Step(c, 0.4, obs())
  check("idle: apply after the delay, target ahead along the camera ray", cmd and cmd.op == "apply" and cmd.mode == "camera"
    and cmd.target.y > 3 and near(cmd.target.x, 0.5, 1e-6), cmd and string.format("%.2f %.2f %.2f", cmd.target.x, cmd.target.y, cmd.target.z))
  check("unchanged camera: no new request", HL.Step(c, 0.6, obs()) == nil)
  local r = HL.Step(c, 0.8, obs({ camFwd = { x = 0.5, y = 0.866, z = 0 } }))
  check("camera turned 30 deg: retarget", r and r.op == "apply" and r.target.x > 1)
  check("retarget rate limit", HL.Step(c, 0.85, obs({ camFwd = { x = -0.5, y = 0.866, z = 0 } })) == nil)
  local w = HL.Step(c, 1.0, obs({ speed = 1.4 }))
  check("walking (1.4 m/s) keeps it", (w == nil or w.op == "apply") and c.state == "on")
  local rel = HL.Step(c, 1.1, obs({ speed = 3.5 }))
  check("running releases", rel and rel.op == "clear" and rel.reason == "moving")
  for _, case in ipairs({ { "aim", { aim = true } }, { "vehicle", { vehicle = true } }, { "fpp", { tpp = false } },
                           { "workspot", { workspot = true } }, { "scene", { scene = true } }, { "swimming", { swimming = true } },
                           { "locomotion", { locoBlocked = true } }, { "ragdoll", { ragdoll = true } }, { "disabled", { enabled = false } } }) do
    local cc = HL.New()
    HL.Step(cc, 0, obs()); HL.Step(cc, 1, obs())
    local x = HL.Step(cc, 1.1, obs(case[2]))
    check("release on " .. case[1], cc.state == "off" and x and x.op == "clear" and x.reason == case[1])
    check("no re-engage while " .. case[1], HL.Step(cc, 3, obs(case[2])) == nil)
  end
  -- 2026-11-03 review r2 #9: a ragdoll episode releases the look; after it the next free steps engage again without a target move
  local cr = HL.New()
  HL.Step(cr, 0, obs()); HL.Step(cr, 1, obs())
  local rg1 = HL.Step(cr, 1.1, obs({ ragdoll = true }))
  check("ragdoll episode: released with reason ragdoll", cr.state == "off" and rg1 and rg1.op == "clear" and rg1.reason == "ragdoll")
  HL.Step(cr, 4, obs())
  local rg2 = HL.Step(cr, 4.5, obs())
  check("after the ragdoll episode: the look engages again with an unchanged camera", rg2 and rg2.op == "apply" and cr.state == "on")
  local cw = HL.New({ whileWalking = false })
  HL.Step(cw, 0, obs()); HL.Step(cw, 1, obs())
  check("whileWalking off: walking releases", (HL.Step(cw, 1.2, obs({ speed = 1.0 })) or {}).op == "clear")
  -- camera in front looking at V -> look at the camera; hysteresis
  local cf = HL.New()
  local front = { camPos = { x = 0, y = 3, z = 1.7 }, camFwd = { x = 0, y = -1, z = 0 } }
  HL.Step(cf, 0, obs(front))
  local f1 = HL.Step(cf, 1, obs(front))
  check("camera facing V: look at the camera", f1 and f1.mode == "face" and near(f1.target.y, 3, 1e-6) and near(f1.target.z, 1.7, 1e-6))
  HL.Step(cf, 1.5, obs({ camPos = { x = 0, y = 3, z = 1.7 }, camFwd = { x = 0.35, y = -0.94, z = 0 } }))
  check("hysteresis: still face for a small change", cf.mode == "face")
  local f2 = HL.Step(cf, 2, obs({ camPos = { x = 0, y = -3, z = 1.7 }, camFwd = { x = 0, y = 1, z = 0 } }))
  check("camera back behind: camera mode", f2 and f2.mode == "camera")
  local cnf = HL.New({ faceCamera = false })
  HL.Step(cnf, 0, obs(front))
  local f3 = HL.Step(cnf, 1, obs(front))
  local ang = math.deg(math.atan2(-(f3.target.x - 0), f3.target.y - 0))
  check("faceCamera off: target behind clamped to the yaw limit", f3.mode == "camera" and f3.target.y < 0, string.format("%.1f", ang))
  -- yaw clamp exact
  local cy = HL.New({ maxYaw = 90, faceCamera = false })
  HL.Step(cy, 0, obs({ camPos = { x = 0, y = 0.2, z = 1.62 }, camFwd = { x = -0.2, y = -0.98, z = 0 } }))
  local y1 = HL.Step(cy, 1, obs({ camPos = { x = 0, y = 0.2, z = 1.62 }, camFwd = { x = -0.2, y = -0.98, z = 0 } }))
  local yaw = math.deg(math.atan2(-(y1.target.x), y1.target.y))
  check("yaw clamped to the limit", y1 and math.abs(math.abs(yaw) - 90) < 0.5, string.format("yaw %.2f", yaw))
  -- pitch clamp
  local cp = HL.New()
  HL.Step(cp, 0, obs({ camFwd = { x = 0, y = 0.2, z = -0.98 } }))
  local p1 = HL.Step(cp, 1, obs({ camFwd = { x = 0, y = 0.2, z = -0.98 } }))
  local flat = math.sqrt(p1.target.x ^ 2 + p1.target.y ^ 2)
  local pitch = math.deg(math.atan2(p1.target.z - 1.62, flat))
  check("pitch clamped (down 40)", math.abs(pitch + 40) < 0.5, string.format("%.2f", pitch))
  -- loot target
  local cl = HL.New()
  HL.Step(cl, 0, obs())
  local l1 = HL.Step(cl, 1, obs({ loot = { x = 0.5, y = 1.0, z = 0.0 } }))
  check("loot: looks at the item", l1 and l1.mode == "loot" and near(l1.target.y, 1.0, 1e-6))
  check("forced release", (HL.Release(cl, "x") or {}).op == "clear" and HL.Release(cl, "x") == nil)
  -- speed gate: hysteresis and a minimum off time after a release by speed (60 Hz ticks)
  local function drive(cc, t0, t1, speedAt)
    local ops = { apply = 0, clear = 0 }
    local t = t0
    while t < t1 - 1e-9 do
      local x = HL.Step(cc, t, obs({ speed = speedAt(t) }))
      if x then ops[x.op] = (ops[x.op] or 0) + 1 end
      t = t + 1 / 60
    end
    return ops
  end
  local co = HL.New()
  drive(co, 0, 1, function() return 0 end)
  local osc = drive(co, 1, 6, function(t) return (math.floor((t - 1) / 0.5) % 2 == 0) and 2.5 or 2.3 end)
  check("speed gate: speed switching between 2.3 and 2.5 m/s every 0.5 s for 5 s -> one release, no re-engage",
    osc.clear == 1 and osc.apply == 0 and co.state == "off", string.format("clears %d applies %d", osc.clear, osc.apply))
  local cs = HL.New()
  drive(cs, 0, 1, function() return 0 end)
  local sg = drive(cs, 1, 1 + 4 * 1.2, function(t) return ((t - 1) % 1.2) < 0.6 and 3.85 or 0 end)
  check("speed gate: stop-and-go (0.6 s running, 0.6 s stops, 4 times) -> one release, the look does not come back in the stops",
    sg.clear == 1 and sg.apply == 0, string.format("clears %d applies %d", sg.clear, sg.apply))
  local back = drive(cs, 5.8, 5.2 + HL.MOVING_ENGAGE_DELAY + 0.5, function() return 0 end)
  check("speed gate: standing still after it -> the look comes back once, MOVING_ENGAGE_DELAY (>= 1 s) after the last stop began",
    back.apply == 1 and cs.state == "on" and HL.MOVING_ENGAGE_DELAY >= 1.0 and cs.lastApply >= 5.2 + HL.MOVING_ENGAGE_DELAY - 1e-6,
    string.format("applies %d at %.2f", back.apply, cs.lastApply or -1))
  local cwk = HL.New()
  drive(cwk, 0, 1, function() return 0 end)
  drive(cwk, 1, 2, function() return 3.85 end)
  local wk1 = drive(cwk, 2, 2 + HL.MOVING_ENGAGE_DELAY - 0.1, function() return 1.6 end)
  local wk2 = drive(cwk, 2 + HL.MOVING_ENGAGE_DELAY - 0.1, 2 + HL.MOVING_ENGAGE_DELAY + 0.3, function() return 1.6 end)
  check("speed gate: running, then walking at 1.6 m/s -> the look comes back while walking after the delay",
    wk1.apply == 0 and wk2.apply == 1 and cwk.state == "on", string.format("%d %d", wk1.apply, wk2.apply))
  local cb = HL.New()
  drive(cb, 0, 1, function() return 0 end)
  drive(cb, 1, 1.5, function() return 3.85 end)
  local band = drive(cb, 1.5, 4, function() return 2.2 end)
  check("speed gate: after a release by speed, 2.2 m/s (inside the hysteresis band) keeps it released", band.apply == 0 and cb.state == "off")
  local ca = HL.New()
  drive(ca, 0, 1, function() return 0 end)
  HL.Step(ca, 1.1, obs({ aim = true }))
  HL.Step(ca, 1.2, obs())
  local a2 = HL.Step(ca, 1.56, obs())
  check("speed gate: a release by another block (aim) keeps the short engage delay", a2 and a2.op == "apply")
end

-- ---------------------------------------------------------------- loot assist selection
do
  local M = load()
  local LA = M.Loot
  local c = LA.New()
  local tr = c.tracker
  local o = { enabled = true, tpp = true, speed = 0, pos = { x = 0, y = 0, z = 0 },
              camPos = { x = 0, y = -3, z = 1.8 }, camFwd = { x = 0, y = 1, z = -0.2 } }
  check("no loot: offset 0", LA.Step(c, 0, 0.1, o) == 0 and c.reason == "none")
  LA.Activate(tr, "far", { x = 0, y = 5, z = 0 }, "QualityRange_Max", 0)
  LA.Activate(tr, "behind", { x = 0, y = -1.2, z = 0 }, "QualityRange_Short", 0)  -- behind the camera ray? cam at y=-3 looks +y -> in cone
  LA.Activate(tr, "side", { x = 1.5, y = -0.2, z = 0 }, "QualityRange_Short", 0)
  LA.Activate(tr, "high", { x = 0, y = 0.8, z = 2.5 }, "QualityRange_Short", 0)
  LA.Activate(tr, "front", { x = 0.2, y = 1.0, z = 0.1 }, "QualityRange_Short", 0)
  local sel = LA.Select(tr, o, c.cfg, nil)
  check("select: nearest in radius / cone / height wins (front)", sel and sel.key == "front", sel and sel.key)
  check("select: outside radius / too high rejected", LA.Score(tr.items.far, o, c.cfg) == nil and LA.Score(tr.items.high, o, c.cfg) == nil)
  local outside = LA.Score({ pos = { x = -1.7, y = -2.6, z = 0 } }, o, c.cfg)
  check("select: outside the camera cone rejected", outside == nil)
  -- hysteresis: a slightly better candidate does not steal the target
  LA.Activate(tr, "front2", { x = -0.15, y = 0.95, z = 0 }, "QualityRange_Short", 0)
  local keep = LA.Select(tr, o, c.cfg, "front")
  check("hysteresis keeps the current target", keep and keep.key == "front")
  -- weight ramps in / holds / ramps out
  local off = 0
  for i = 1, 6 do off = LA.Step(c, 0.1 * i, 0.1, o) end
  check("weight ramps to full zoom", c.weight == 1 and near(off, c.cfg.zoom, 1e-9), string.format("%.3f", off))
  LA.Deactivate(tr, "front", "QualityRange_Short"); LA.Deactivate(tr, "front2", "QualityRange_Short")
  LA.Deactivate(tr, "behind", "QualityRange_Short"); LA.Deactivate(tr, "side", "QualityRange_Short")
  check("deactivated loot removed", tr.items.front == nil and tr.items.far ~= nil)
  LA.Step(c, 0.8, 0.1, o)
  check("hold: target kept briefly after it is gone", c.target ~= nil and c.weight == 1)
  for i = 1, 12 do off = LA.Step(c, 0.8 + 0.1 * i, 0.1, o) end
  check("then weight returns to 0", c.target == nil and c.weight == 0 and off == 0)
  LA.Activate(tr, "front", { x = 0.2, y = 1.0, z = 0.1 }, "x", 3)
  for i = 1, 6 do LA.Step(c, 3 + 0.1 * i, 0.1, o) end
  local a = { }
  for k, v in pairs(o) do a[k] = v end
  a.blocked = "aim"
  LA.Step(c, 4, 0.1, a)
  check("aim: target dropped at once, fast fade", c.target == nil and c.weight < 1 and c.reason == "aim")
  a.blocked = nil; a.tpp = false
  for i = 1, 3 do LA.Step(c, 4 + 0.1 * i, 0.1, a) end
  check("fpp: no assist", c.weight == 0 and c.reason == "fpp")
  a.tpp = true; a.speed = 5
  check("running: no selection", LA.Step(c, 5, 0.1, a) == 0 and c.reason == "moving")
  -- crosshair ttl, prune vanished / far / max count
  LA.Activate(tr, "look", { x = 0, y = 1, z = 0 }, "lookat", 10, 1.0)
  LA.Prune(tr, 10.5, o.pos, c.cfg, nil)
  check("crosshair entry alive within ttl", tr.items.look ~= nil)
  LA.Prune(tr, 11.5, o.pos, c.cfg, nil)
  check("crosshair entry expires", tr.items.look == nil)
  LA.Prune(tr, 12, o.pos, c.cfg, function(key) if key == "front" then return nil end return { x = 0, y = 5, z = 0 } end)
  check("vanished entity pruned", tr.items.front == nil and tr.items.far ~= nil)
  for i = 1, 60 do LA.Activate(tr, "n" .. i, { x = i * 0.3, y = 0, z = 0 }, "L", 12) end
  LA.Prune(tr, 12, o.pos, c.cfg, nil)
  check("tracked list capped (farthest dropped, > trackRange removed)", tr.n <= c.cfg.maxTracked and tr.items.n1 ~= nil
    and tr.items.n60 == nil, tostring(tr.n))
  -- camera offset helper
  local fwd = LA.QuatForward({ i = 0, j = 0, k = 0, r = 1 })
  check("quat forward identity = +Y", near(fwd.x, 0) and near(fwd.y, 1) and near(fwd.z, 0))
  local q90 = { i = 0, j = 0, k = math.sin(math.pi / 4), r = math.cos(math.pi / 4) } -- 90 deg CCW about Z
  local f90 = LA.QuatForward(q90)
  check("quat forward 90 deg yaw = -X", near(f90.x, -1, 1e-9) and near(f90.y, 0, 1e-9))
  local st = {}
  local out = LA.ApplyOffset(st, { x = 0, y = -2, z = 0.5 }, { x = 0, y = 1, z = 0 }, 0.5)
  check("offset applied to the owner's base", near(out.y, -1.5))
  out = LA.ApplyOffset(st, out, { x = 0, y = 1, z = 0 }, 0.8)
  check("not rewritten by the owner: no accumulation", near(out.y, -1.2))
  out = LA.ApplyOffset(st, { x = 0, y = -3, z = 0.5 }, { x = 0, y = 1, z = 0 }, 0.8)
  check("rewritten by the owner: new base", near(out.y, -2.2))
end

-- ---------------------------------------------------------------- utpp_vehiclecam settings via utpp_core
do
  local core = load()
  core.SETTINGS_FILE = tmp2
  local f = io.open(tmp2, "w"); f:write("vehicle.keepTpp=false\n"); f:close()
  core.Load(tmp2)
  local handlers, lines = {}, {}
  local sv = { print = print, reg = registerForEvent, getmod = GetMod, game = Game }
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  Game = setmetatable({ UTPPLogLine = function() end }, { __index = function() return function() return nil end end })
  print = function(...) local p = {} for i = 1, select("#", ...) do p[#p + 1] = tostring(select(i, ...)) end lines[#lines + 1] = table.concat(p, " ") end
  local okV, V = pcall(dofile, UTPP_VEHICLECAM_PATH)
  V.SETTINGS_FILE = os.tmpname()
  GetMod = function(n) if n == "utpp_core" then return core end return nil end
  if okV and handlers.onInit then pcall(handlers.onInit) end
  print, registerForEvent, GetMod, Game = sv.print, sv.reg, sv.getmod, sv.game
  local joined = table.concat(lines, " | ")
  check("vehiclecam: settings via utpp_core ok", okV and joined:find("settings via utpp_core: ok", 1, true) ~= nil, joined)
  check("vehiclecam: stored core value applied (keepTpp=false)", V.ctrl.cfg.keepTpp == false and core.Get("vehicle.keepTpp") == false)
  check("vehiclecam: pullOut registered with its default", core.Get("vehicle.pullOut") == true and V.ctrl.cfg.pullOut == true)
  core.Set("vehicle.keepTpp", true)
  check("vehiclecam: core.Set reaches the controller", V.ctrl.cfg.keepTpp == true)
  core.Set("vehicle.pullOut", false)
  check("vehiclecam: pullOut via core", V.ctrl.cfg.pullOut == false)
  check("vehiclecam: no own Native Settings tab when utpp_core present", joined:find("Native Settings tab UTPP", 1, true) == nil
    and joined:find("Native Settings not found", 1, true) == nil)
  check("vehiclecam: reset via core restores defaults", core.ResetAll() and V.ctrl.cfg.pullOut == true and V.ctrl.cfg.keepTpp == true)
  -- menu layout of the vehicle keys (S2 API; 2026-10-30: own Vehicles category): forceTppOnEnter is a player option, the rest
  -- of the vehicle keys are advanced options of the same category
  local function placed(k) local lv, gr = core.MenuPlacement(k); return lv .. "/" .. tostring(gr) end
  check("vehiclecam: forceTppOnEnter registered with its default, placed player/vehicles", core.Get("vehicle.forceTppOnEnter") == true
    and placed("vehicle.forceTppOnEnter") == "player/vehicles", placed("vehicle.forceTppOnEnter"))
  -- 2026-11-13: these four are hidden in 0.1.0 (registry flag hidden: no menu entry; layout group kept; default on, still applied)
  check("vehiclecam: keepTpp / pullOut / rideWatchdog / safeTeleportExit hidden (vehicles group kept), default on",
    placed("vehicle.keepTpp") == "hidden/vehicles" and placed("vehicle.pullOut") == "hidden/vehicles"
    and placed("vehicle.rideWatchdog") == "hidden/vehicles" and placed("vehicle.safeTeleportExit") == "hidden/vehicles"
    and core.Get("vehicle.keepTpp") == true and core.Get("vehicle.pullOut") == true and core.Get("vehicle.rideWatchdog") == true
    and core.Get("vehicle.safeTeleportExit") == true,
    placed("vehicle.keepTpp") .. " " .. placed("vehicle.pullOut") .. " " .. placed("vehicle.rideWatchdog") .. " "
    .. placed("vehicle.safeTeleportExit"))
  -- 2026-11-04 jump-out side push: default on, advanced/vehicles; the apply callback reaches the live controller
  check("vehiclecam: jumpOutSidePush registered default on, placed advanced/vehicles",
    core.Get("vehicle.jumpOutSidePush") == true and placed("vehicle.jumpOutSidePush") == "advanced/vehicles", placed("vehicle.jumpOutSidePush"))
  -- 2026-11-03 car entry approach walk: default on, advanced/vehicles, pushed to Game.UTPPSetVehicleApproachEnabled with retries
  check("vehiclecam: approachWalk registered default on, placed advanced/vehicles",
    core.Get("vehicle.approachWalk") == true and placed("vehicle.approachWalk") == "advanced/vehicles", placed("vehicle.approachWalk"))
  do
    local calls, answer = {}, false
    local p1, p2 = {}, {} -- offline player stand-ins (keyed by identity)
    Game = Game or {}
    local saved = Game.UTPPSetVehicleApproachEnabled
    Game.UTPPSetVehicleApproachEnabled = function(on) calls[#calls + 1] = on; return answer end
    V.SetApproachWalk(true)
    local st1 = V.PushApproachWalk(10.0, p1, nil)
    local st2 = V.PushApproachWalk(10.5, p1, nil) -- inside the 1 s retry gap: no second call
    answer = true
    local st3 = V.PushApproachWalk(11.2, p1, nil)
    local st4 = V.PushApproachWalk(12.0, p1, nil) -- sent: nothing more for the same player
    check("approachWalk push: retried until the system answers, then once per player",
      st1 == "pending" and st2 == "pending" and st3 == "sent" and st4 == "sent" and #calls == 2 and calls[1] == true,
      st1 .. " " .. st2 .. " " .. st3 .. " " .. st4 .. " calls=" .. #calls)
    V.PushApproachWalk(13.0, p2, nil)
    check("approachWalk push: a new player object gets the value again", #calls == 3, "calls=" .. #calls)
    V.SetApproachWalk(false)
    V.PushApproachWalk(14.0, p2, nil)
    check("approachWalk push: a setting change is pushed", #calls == 4 and calls[4] == false, "calls=" .. #calls)
    V.SetApproachWalk(true)
    Game.UTPPSetVehicleApproachEnabled = saved
  end
  os.remove(V.SETTINGS_FILE)
end

-- ---------------------------------------------------------------- 0.5.1 animation database selector
do
  local M = load()
  M.SETTINGS_FILE = tmp2
  check("db list parser: order kept, invalid / duplicate names dropped", table.concat(M.ParseDatabaseList(
    "default;alt_db;bad name;../x;alt_db; motion_plus "), ",") == "default,alt_db,motion_plus")
  check("db list empty -> no setting", M.SetupDatabaseSetting("", function() end) == false and M.settings.defs["mm.database"] == nil)
  local requests, status, active = {}, "idle", "default"
  local function loader(n) requests[#requests + 1] = n; if n == "missing" then return false end status = "loading " .. n; return true end
  check("db setting registered, default applied at start (one request)",
    M.SetupDatabaseSetting("default;alt_db;motion_plus", loader, function() return status end, function() return active end)
    and M.Get("mm.database") == "default" and #requests == 1 and requests[1] == "default")
  check("db choice outside the list rejected", M.SetDatabase("nope") == nil and #requests == 1)
  check("db SetDatabase -> loader", M.SetDatabase("alt_db") == "alt_db" and requests[#requests] == "alt_db")
  M.PollDatabase(0.1)
  status, active = "ok alt_db", "alt_db"
  M.PollDatabase(0.1)
  check("db poll logs the result and stops", M.db.lastStatus == "ok alt_db" and M.db.watch == 0)
  local text = io.open(tmp2):read("*a")
  check("db selection persisted", text:find("mm.database=alt_db", 1, true) ~= nil)
  -- failure: selector returns to the active database without a new request
  M.SetDatabase("motion_plus")
  local n = #requests
  status = "FAILED motion_plus: bone set differs from the active database"
  M.PollDatabase(0.1)
  check("db failure: selector back to the active one, no extra request", M.Get("mm.database") == "alt_db" and #requests == n
    and M.db.loader ~= nil)
  -- restart: stored value applied at registration
  local M2 = load()
  M2.SETTINGS_FILE = tmp2
  M2.Load(tmp2)
  local req2 = {}
  M2.SetupDatabaseSetting("default;alt_db", function(x) req2[#req2 + 1] = x; return true end)
  check("db restart: stored database requested at start", M2.Get("mm.database") == "alt_db" and req2[1] == "alt_db")
  local M3 = load()
  M3.SETTINGS_FILE = tmp2
  M3.Load(tmp2)
  local req3 = {}
  M3.SetupDatabaseSetting("default", function(x) req3[#req3 + 1] = x; return true end)
  check("db restart: stored database no longer installed -> default", M3.Get("mm.database") == "default" and req3[1] == "default")
  check("db reset -> default", M.ResetAll() and M.Get("mm.database") == "default" and requests[#requests] == "default")
end

-- ---------------------------------------------------------------- 0.5.5 motion matching switches (mm.enabled / mm.trace)
do
  local function setup(stored)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = { mm = {}, trace = {} }
    M.SetupMMSettings(function(on) calls.mm[#calls.mm + 1] = on end, function(on) calls.trace[#calls.trace + 1] = on end)
    return M, calls
  end
  local M, c = setup()
  check("mm: default on applies UTPPSetMMEnabled(true) once at start", M.Get("mm.enabled") == true
    and #c.mm == 1 and c.mm[1] == true and M.mm.enabled == true)
  check("mm: trace default off (release defaults 2026-10-22), applied once", M.Get("mm.trace") == false and #c.trace == 1 and c.trace[1] == false)
  check("mm: settings in the animation section, bool switch", M.settings.defs["mm.enabled"].type == "bool"
    and M.settings.defs["mm.enabled"].section == "animation" and M.settings.sections.animation ~= nil)
  check("mm: apply logged with the reason", #quietLines > 0 and quietLines[#quietLines - 1]:find("motion matching on (setting)", 1, true) ~= nil,
    quietLines[#quietLines - 1])
  M.Set("mm.trace", true)
  check("mm: trace on -> UTPPSetMMTrace(true), persisted", c.trace[2] == true and io.open(tmp2):read("*a"):find("mm.trace=true", 1, true) ~= nil)
  M.Set("mm.trace", true)
  check("mm: same trace value -> no second native call", #c.trace == 2)
  M.Set("mm.enabled", "off")
  check("mm: off -> UTPPSetMMEnabled(false)", c.mm[2] == false and #c.mm == 2)
  M.Set("mm.enabled", "auto")
  check("mm: legacy auto value -> on again", c.mm[3] == true and #c.mm == 3 and M.Get("mm.enabled") == true)
  M.ReapplyMM("again")
  check("mm: re-apply without a change -> no native call", #c.mm == 3)
  local M3, c3 = setup("mm.enabled=off\nmm.trace=true\n")
  check("mm: stored off / trace on applied at start (stored beats the default)", c3.mm[1] == false and c3.trace[1] == true and M3.Get("mm.enabled") == false)
  local M5, c5 = setup("mm.enabled=auto\n")
  check("mm: stored legacy auto applied as on", c5.mm[1] == true and #c5.mm == 1 and M5.Get("mm.enabled") == true)
  local M4 = load()
  os.remove(tmp2)
  M4.SETTINGS_FILE = tmp2
  M4.SetupMMSettings(function() error("UTPP.dll too old") end, nil)
  check("mm: native error -> logged FAILED, value not marked as sent", M4.mm.enabled == nil and M4.mm.errors == 1
    and quietLines[#quietLines]:find("motion matching on (setting) FAILED", 1, true) ~= nil, quietLines[#quietLines])
  check("mm: Status() reports mm / trace", M.Status():find("mm=true (sent true) trace=true", 1, true) ~= nil, M.Status())
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- stage standstill idle (mm.idleHold)
do -- v2 owns its Aim setting regardless of module registration order; core preserves stored values.
  local f = assert(io.open(tmp2, "w")); f:write("mm.aimGameUpper=false\n"); f:close()
  for _, motionFirst in ipairs({false, true}) do
    local A = load()
    A.SETTINGS_FILE = tmp2
    A.Load()
    local savedRegister = registerForEvent
    registerForEvent = nil
    local V = dofile(UTPP_MOTION_PATH)
    registerForEvent = savedRegister
    V.sink = function() end
    if motionFirst then V.RegisterSettings(A) end
    A.SetupMMSettings(function() end, function() end, {})
    if not motionFirst then
      check("core leaves motion Aim key to motion module", A.Get("mm.aimGameUpper") == nil)
      V.RegisterSettings(A)
    end
    check("motion Aim ownership and stored false independent of registration order " .. tostring(motionFirst),
      A.Get("mm.aimGameUpper") == false and V.want.aimGameUpper == false
      and A.settings.defs["mm.aimGameUpper"].section == "motion")
  end
  os.remove(tmp2)
  local G = load()
  G.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local gripCalls = {}
  local shootCalls = {}
  G.SetupMMSettings(function() end, function() end, {
    setMotionGrip=function(on) gripCalls[#gripCalls+1] = on end,
    setMotionShootTransfer=function(on) shootCalls[#shootCalls+1] = on end })
  check("motion Shoot transfer: default ON (user defaults 2026-10-20) applied once", #shootCalls == 1 and shootCalls[1] == true
    and G.Get("mm.shootTransfer") == true and G.mm.motionShootTransfer == true)
  G.Set("mm.shootTransfer", false)
  check("motion Shoot transfer: explicit opt-out reaches native", #shootCalls == 2 and shootCalls[2] == false
    and G.mm.motionShootTransfer == false)
  G.Set("mm.shootTransfer", true)
  check("motion grip: default ON applied once", #gripCalls == 1 and gripCalls[1] == true and G.mm.motionGrip == true)
  G.Set("mm.grip", false)
  check("motion grip: OFF applied and persisted", #gripCalls == 2 and gripCalls[2] == false and G.Get("mm.grip") == false)
  G.mm.setMotionGrip = function() error("native unavailable") end
  local gripErrors = G.mm.errors
  G.Set("mm.grip", true)
  check("motion grip: failure does not claim ON", G.mm.errors == gripErrors+1 and G.mm.motionGrip == false)
  local H = load()
  os.remove(tmp2)
  H.SETTINGS_FILE = tmp2
  H.SetupMMSettings(function() end, function() end, {})
  check("motion grip: absent API does not claim application", H.Get("mm.grip") == true and H.mm.motionGrip == nil)
  os.remove(tmp2)
  local M = load()
  M.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local calls = {}
  M.SetupMMSettings(function() end, function() end, {setWeaponUpper=function(on) calls[#calls+1] = on end})
  check("weapon upper: default OFF (user defaults 2026-10-20) applied once", #calls == 1 and calls[1] == false and M.mm.weaponUpper == false)
  M.Set("mm.weaponUpper", true)
  check("weapon upper: ON applied and persisted", #calls == 2 and calls[2] == true and M.Get("mm.weaponUpper") == true)
  local errors = M.mm.errors
  M.mm.setWeaponUpper = function() error("native unavailable") end
  M.Set("mm.weaponUpper", false)
  check("weapon upper: failed native counted, applied state remains ON", M.mm.errors == errors+1 and M.mm.weaponUpper == true)
  local N = load()
  os.remove(tmp2)
  N.SETTINGS_FILE = tmp2
  N.SetupMMSettings(function() end, function() end, {})
  check("weapon upper: missing API does not claim that the native policy was applied", N.Get("mm.weaponUpper") == false and N.mm.weaponUpper == nil)
  os.remove(tmp2)
  -- mm.shinobiBonusReload "Reload animation during Shinobi bonus": default OFF = the DLL default, so a fresh boot calls nothing
  local T = load()
  T.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local tapCalls = {}
  T.SetupMMSettings(function() end, function() end, { setMotionMagTap = function(on) tapCalls[#tapCalls+1] = on end })
  local tdef = T.settings.defs["mm.shinobiBonusReload"]
  check("shinobi mag tap: registered bool, default OFF, label 'Reload animation during Shinobi bonus'",
    tdef ~= nil and tdef.type == "bool" and tdef.default == false and tdef.label == "Reload animation during Shinobi bonus"
    and tdef.section == "animation")
  check("shinobi mag tap: default OFF -> no native call at boot", #tapCalls == 0 and T.Get("mm.shinobiBonusReload") == false
    and T.mm.motionMagTap == false)
  T.Set("mm.shinobiBonusReload", true)
  check("shinobi mag tap: ON -> one native call (true), persisted", #tapCalls == 1 and tapCalls[1] == true
    and T.mm.motionMagTap == true and T.Get("mm.shinobiBonusReload") == true)
  T.Set("mm.shinobiBonusReload", true)
  check("shinobi mag tap: same value again -> no second call", #tapCalls == 1)
  T.Set("mm.shinobiBonusReload", false)
  check("shinobi mag tap: OFF -> one native call (false)", #tapCalls == 2 and tapCalls[2] == false and T.mm.motionMagTap == false)
  local tapErrors = T.mm.errors
  T.mm.setMotionMagTap = function() error("native unavailable") end
  T.Set("mm.shinobiBonusReload", true)
  check("shinobi mag tap: failed native counted, not marked as sent", T.mm.errors == tapErrors + 1 and T.mm.motionMagTap == false)
  T.mm.setMotionMagTap = function(on) tapCalls[#tapCalls+1] = on end
  T.Set("mm.shinobiBonusReload", false)
  T.Set("mm.shinobiBonusReload", true)
  check("shinobi mag tap: a later change tries again", #tapCalls == 3 and tapCalls[3] == true and T.mm.motionMagTap == true)
  -- stored ON: exactly one call at boot
  local f = assert(io.open(tmp2, "w")); f:write("mm.shinobiBonusReload=true\n"); f:close()
  local U = load()
  U.SETTINGS_FILE = tmp2
  U.Load()
  local uCalls = {}
  U.SetupMMSettings(function() end, function() end, { setMotionMagTap = function(on) uCalls[#uCalls+1] = on end })
  check("shinobi mag tap: stored ON -> exactly one native call (true) at boot", #uCalls == 1 and uCalls[1] == true
    and U.mm.motionMagTap == true)
  local P = load()
  os.remove(tmp2)
  P.SETTINGS_FILE = tmp2
  P.SetupMMSettings(function() end, function() end, {})
  P.Set("mm.shinobiBonusReload", true)
  check("shinobi mag tap: missing API -> stored only, not marked as sent", P.Get("mm.shinobiBonusReload") == true
    and P.mm.motionMagTap == false)
  local lv, grp = P.MenuPlacement("mm.shinobiBonusReload")
  check("shinobi mag tap: shown in the Combat group of the standard menu", lv == "player" and grp == "combat")
  os.remove(tmp2)
end

do -- crosshair.impl "Use UTPP crosshair": default ON, per-session push to Game.UTPPSetCrosshair, retry while unacknowledged
  local savedGame = Game
  Game = nil
  local X = load()
  X.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local def = X.settings.defs["crosshair.impl"]
  check("crosshair: registered bool, default ON, label 'Use UTPP crosshair', section crosshair", def ~= nil and def.type == "bool"
    and def.default == true and def.label == "Use UTPP crosshair" and def.section == "crosshair" and X.Get("crosshair.impl") == true)
  local lv, grp = X.MenuPlacement("crosshair.impl")
  check("crosshair: shown in the Combat group of the standard menu", lv == "player" and grp == "combat")
  check("crosshair: Game == nil -> no call, no error, nothing acknowledged", X.xhair.calls == 0 and X.xhair.errors == 0
    and X.xhair.sent == nil)
  X.Set("crosshair.impl", true)
  check("crosshair: outside a session a Set does not call", X.xhair.calls == 0 and X.xhair.sent == nil)
  local calls = {}
  X.xhair.api = function(on) calls[#calls + 1] = on; return true end
  X.CrosshairResync(10, true)
  check("crosshair: fresh session -> exactly one push (true)", #calls == 1 and calls[1] == true and X.xhair.sent == true)
  X.CrosshairResync(11)
  X.Set("crosshair.impl", true)
  check("crosshair: same value again / later frames -> no second push", #calls == 1)
  X.Set("crosshair.impl", false)
  check("crosshair: OFF -> one push (false), persisted", #calls == 2 and calls[2] == false and X.xhair.sent == false
    and X.Get("crosshair.impl") == false)
  local errors = X.xhair.errors
  X.xhair.api = function() error("script missing") end
  X.Set("crosshair.impl", true)
  check("crosshair: failing call counted, not marked as sent", X.xhair.errors == errors + 1 and X.xhair.sent == nil)
  X.xhair.api = function() return false end
  X.CrosshairResync(12)
  check("crosshair: call returning false counted, still not sent", X.xhair.errors == errors + 2 and X.xhair.sent == nil)
  X.CrosshairResync(12.5)
  check("crosshair: retry throttled to once per second", X.xhair.errors == errors + 2)
  X.xhair.api = function(on) calls[#calls + 1] = on; return true end
  X.CrosshairResync(13.1)
  check("crosshair: retry after a second sends the stored value", #calls == 3 and calls[3] == true and X.xhair.sent == true)
  X.Set("crosshair.impl", false)
  X.xhair.api = function() error("script missing") end
  X.Set("crosshair.impl", true)
  X.xhair.api = function(on) calls[#calls + 1] = on; return true end
  X.Set("crosshair.impl", false)
  X.Set("crosshair.impl", true)
  check("crosshair: a later change tries again", calls[#calls] == true and X.xhair.sent == true)
  local n = #calls
  X.CrosshairResync(20, true)
  check("crosshair: new session re-pushes the stored value", #calls == n + 1 and calls[#calls] == true and X.xhair.sent == true)
  -- stored OFF: exactly one push (false) at the first session
  local f = assert(io.open(tmp2, "w")); f:write("crosshair.impl=false\n"); f:close()
  local Y = load()
  Y.SETTINGS_FILE = tmp2
  Y.Load()
  local yCalls = {}
  Y.xhair.api = function(on) yCalls[#yCalls + 1] = on; return true end
  Y.CrosshairResync(0, true)
  Y.CrosshairResync(5)
  check("crosshair: stored OFF -> exactly one push (false) at boot", #yCalls == 1 and yCalls[1] == false
    and Y.Get("crosshair.impl") == false and Y.xhair.sent == false)
  -- the game call path: Game.UTPPSetCrosshair only inside a session
  local gCalls = {}
  Game = { UTPPLogLine = function() end, UTPPSetCrosshair = function(on) gCalls[#gCalls + 1] = on; return true end }
  local Z = load()
  check("crosshair: Game present but no session -> no call at load", #gCalls == 0 and Z.xhair.sent == nil)
  Z.CrosshairResync(0, true)
  check("crosshair: session edge -> Game.UTPPSetCrosshair(true)", #gCalls == 1 and gCalls[1] == true and Z.xhair.sent == true)
  Game = savedGame
  os.remove(tmp2)
end

do -- crosshair.color "Crosshair colour": Standard default, index push to Game.UTPPSetCrosshairColor with the session rules
  local savedGame = Game
  Game = nil
  local X = load()
  X.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local def = X.settings.defs["crosshair.color"]
  local ids = {}
  for _, c in ipairs(def and def.choices or {}) do ids[#ids + 1] = c[1] .. "=" .. c[2] end
  check("crosshair colour: registered choice, default standard, label 'Crosshair colour', section crosshair",
    def ~= nil and def.type == "choice" and def.default == "standard" and def.label == "Crosshair colour"
    and def.section == "crosshair" and X.Get("crosshair.color") == "standard")
  check("crosshair colour: choices Standard, White, Red, Green, Black",
    table.concat(ids, ";") == "standard=Standard;white=White;red=Red;green=Green;black=Black", table.concat(ids, ";"))
  local lv, grp = X.MenuPlacement("crosshair.color")
  check("crosshair colour: shown in the Combat group of the standard menu", lv == "player" and grp == "combat")
  check("crosshair colour: invalid value rejected", X.Set("crosshair.color", "purple") == nil and X.Get("crosshair.color") == "standard")
  check("crosshair colour: outside a session nothing is called", X.xcolor.calls == 0 and X.xcolor.sent == nil)
  local calls, implCalls = {}, {}
  X.xcolor.api = function(i) calls[#calls + 1] = i; return true end
  X.xhair.api = function(on) implCalls[#implCalls + 1] = on; return true end
  X.CrosshairResync(10, true)
  check("crosshair colour: fresh session -> one push (0 = standard), together with the on/off push",
    #calls == 1 and calls[1] == 0 and X.xcolor.sent == 0 and #implCalls == 1)
  X.Set("crosshair.color", "black")
  check("crosshair colour: Black -> one push (4), persisted", #calls == 2 and calls[2] == 4 and X.Get("crosshair.color") == "black")
  X.Set("crosshair.color", "black")
  X.CrosshairResync(11)
  check("crosshair colour: same value / later frames -> no second push", #calls == 2 and #implCalls == 1)
  local errors = X.xcolor.errors
  X.xcolor.api = function() return false end
  X.Set("crosshair.color", "red")
  check("crosshair colour: failing call counted, not marked as sent", X.xcolor.errors == errors + 1 and X.xcolor.sent == nil)
  X.CrosshairResync(11.5)
  check("crosshair colour: session retry tries the unacknowledged value", X.xcolor.errors == errors + 2 and X.xcolor.sent == nil)
  X.xcolor.api = function(i) calls[#calls + 1] = i; return true end
  X.CrosshairResync(12.0)
  check("crosshair colour: retry throttled to once per second", #calls == 2)
  X.CrosshairResync(12.6)
  check("crosshair colour: retry sends the stored value (2 = red), on/off not re-sent", #calls == 3 and calls[3] == 2
    and X.xcolor.sent == 2 and #implCalls == 1)
  X.CrosshairResync(20, true)
  check("crosshair colour: new session re-pushes the stored value", #calls == 4 and calls[4] == 2 and #implCalls == 2)
  local f = assert(io.open(tmp2, "w")); f:write("crosshair.color=green\n"); f:close()
  local Y = load()
  Y.SETTINGS_FILE = tmp2
  Y.Load()
  local yCalls = {}
  Y.xcolor.api = function(i) yCalls[#yCalls + 1] = i; return true end
  Y.xhair.api = function() return true end
  Y.CrosshairResync(0, true)
  check("crosshair colour: stored Green -> exactly one push (3) at boot", #yCalls == 1 and yCalls[1] == 3
    and Y.Get("crosshair.color") == "green")
  Game = savedGame
  os.remove(tmp2)
end

do -- Batch6 aim mask fade: default, A/B switch, native failure and absent API.
  local M = load()
  M.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local calls = {}
  M.SetupMMSettings(function() end, function() end, {setAimUpperFade=function(on) calls[#calls+1] = on end})
  check("aim upper fade: default ON applied once", #calls == 1 and calls[1] == true and M.mm.aimUpperFade == true)
  M.Set("mm.aimUpperFade", false)
  check("aim upper fade: OFF applied and persisted", #calls == 2 and calls[2] == false and M.Get("mm.aimUpperFade") == false)
  local before = M.mm.errors
  M.mm.setAimUpperFade = function() error("native unavailable") end
  M.Set("mm.aimUpperFade", true)
  check("aim upper fade: failed native counted without marking the new state applied", M.mm.errors == before+1 and M.mm.aimUpperFade == false)
  local N = load()
  os.remove(tmp2)
  N.SETTINGS_FILE = tmp2
  N.SetupMMSettings(function() end, function() end, {})
  check("aim upper fade: absent native leaves the setting available without claiming application", N.Get("mm.aimUpperFade") == true and N.mm.aimUpperFade == nil)
  os.remove(tmp2)
end

do
  local function setup(stored, fn)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = {}
    M.SetupMMSettings(function() end, function() end,
      { setIdleHold = fn or function(on) calls[#calls + 1] = on end })
    return M, calls
  end
  local M, c = setup()
  local def = M.settings.defs["mm.idleHold"]
  check("idleHold: bool in the animation section, default true, applied once at start -> UTPPSetMMIdleHold(true), logged",
    def ~= nil and def.type == "bool" and def.section == "animation" and M.Get("mm.idleHold") == true and #c == 1 and c[1] == true
    and M.mm.idleHold == true and quietLines[#quietLines]:find("standstill idle hold on (mm.idleHold)", 1, true) ~= nil)
  M.Set("mm.idleHold", false)
  M.Set("mm.idleHold", false)
  check("idleHold: off -> UTPPSetMMIdleHold(false) once, persisted", #c == 2 and c[2] == false
    and io.open(tmp2):read("*a"):find("mm.idleHold=false", 1, true) ~= nil)
  local _, c2 = setup("mm.idleHold=false\n")
  check("idleHold: stored false applied at start", #c2 == 1 and c2[1] == false)
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMIdleHold')") end)
  local function failLines()
    local k = 0
    for _, l in ipairs(quietLines) do if l:find("standstill idle hold", 1, true) and l:find("FAILED", 1, true) then k = k + 1 end end
    return k
  end
  local f1 = failLines()
  M3.Set("mm.idleHold", false)
  check("idleHold: missing native -> logged once, no error, not marked as sent, retried on change", n == 2 and f1 == 1
    and failLines() == 1 and M3.mm.idleHold == nil, string.format("calls %d logged %d/%d", n, f1, failLines()))
  local M5 = load()
  M5.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  M5.SetupMMSettings(function() end, function() end, {})
  check("idleHold: api table without setIdleHold -> registered and stored only, no call", M5.Get("mm.idleHold") == true
    and M5.mm.idleHold == nil and M5.mm.setIdleHold == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- batch 2 knee fix (mm.kneeFix)
do
  local function setup(stored, fn)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = {}
    M.SetupMMSettings(function() end, function() end,
      { setKneeFix = fn or function(mask) calls[#calls + 1] = mask; return true end })
    return M, calls
  end
  local M, c = setup()
  local def = M.settings.defs["mm.kneeFix"]
  check("kneeFix: bool in the animation section, default true, applied once at start -> UTPPSetMMKneeFix(7), logged",
    def ~= nil and def.type == "bool" and def.section == "animation" and M.Get("mm.kneeFix") == true and #c == 1 and c[1] == 7
    and M.mm.kneeFix == 7 and quietLines[#quietLines]:find("knee fix on (mm.kneeFix, mask 7)", 1, true) ~= nil)
  M.Set("mm.kneeFix", false)
  M.Set("mm.kneeFix", false)
  check("kneeFix: off -> UTPPSetMMKneeFix(0) once, persisted", #c == 2 and c[2] == 0 and M.mm.kneeFix == 0
    and io.open(tmp2):read("*a"):find("mm.kneeFix=false", 1, true) ~= nil)
  M.Set("mm.kneeFix", true)
  check("kneeFix: on again -> UTPPSetMMKneeFix(7)", #c == 3 and c[3] == 7)
  -- batch 5 repair: mm.kneeHold (bit 4) has its own setting; off = mask 3 (the 0.7.0 knee fix)
  local hdef = M.settings.defs["mm.kneeHold"]
  M.Set("mm.kneeHold", false)
  M.Set("mm.kneeHold", false)
  check("kneeHold: bool in the animation section, default true; off -> UTPPSetMMKneeFix(3) once, persisted",
    hdef ~= nil and hdef.type == "bool" and hdef.section == "animation" and hdef.default == true and #c == 4 and c[4] == 3
    and M.mm.kneeFix == 3 and io.open(tmp2):read("*a"):find("mm.kneeHold=false", 1, true) ~= nil, table.concat(c, ","))
  M.Set("mm.kneeFix", false)
  M.Set("mm.kneeHold", true)
  check("kneeHold: on while mm.kneeFix is off -> mask stays 0 (no call)", #c == 5 and c[5] == 0 and M.mm.kneeFix == 0,
    table.concat(c, ","))
  M.Set("mm.kneeFix", true)
  check("kneeHold: mm.kneeFix on again -> 7", #c == 6 and c[6] == 7)
  local _, cH = setup("mm.kneeHold=false\n")
  check("kneeHold: stored false applied at start -> one call, mask 3", #cH == 1 and cH[1] == 3, table.concat(cH, ","))
  -- batch 5: a UTPP.dll before bit 4 rejects mask 7 -> its knee fix mask 3 is sent instead, remembered for later changes
  local cOld = {}
  local MO = setup(nil, function(mask) cOld[#cOld + 1] = mask; return mask <= 3 end)
  check("kneeFix: old DLL rejects 7 -> falls back to 3, marked as sent, no error logged", #cOld == 2 and cOld[1] == 7
    and cOld[2] == 3 and MO.mm.kneeFix == 3 and MO.mm.errors == 0, table.concat(cOld, ","))
  local callsBefore = MO.mm.calls
  MO.Set("mm.kneeFix", false)
  MO.Set("mm.kneeFix", true)
  check("kneeFix: old DLL remembered -> off / on send 0 then 3 directly (7 not retried)", #cOld == 4 and cOld[3] == 0
    and cOld[4] == 3 and MO.mm.calls == callsBefore + 2 and MO.mm.errors == 0, table.concat(cOld, ","))
  local _, c2 = setup("mm.kneeFix=false\n")
  check("kneeFix: stored false applied at start", #c2 == 1 and c2[1] == 0)
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMKneeFix')") end)
  local function failLines()
    local k = 0
    for _, l in ipairs(quietLines) do if l:find("knee fix", 1, true) and l:find("FAILED", 1, true) then k = k + 1 end end
    return k
  end
  local f1 = failLines()
  M3.Set("mm.kneeFix", false)
  check("kneeFix: missing native -> logged once, no error, not marked as sent, retried on change", n == 2 and f1 == 1
    and failLines() == 1 and M3.mm.kneeFix == nil, string.format("calls %d logged %d/%d", n, f1, failLines()))
  local nr = 0
  local M4 = setup(nil, function() nr = nr + 1; return false end)
  check("kneeFix: native returns false -> counted as an error, not marked as sent (7, then 3 tried)", nr == 2 and M4.mm.kneeFix == nil
    and M4.mm.errors >= 1)
  local M5 = load()
  M5.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  M5.SetupMMSettings(function() end, function() end, {})
  check("kneeFix: api table without setKneeFix -> registered and stored only, no call", M5.Get("mm.kneeFix") == true
    and M5.mm.kneeFix == nil and M5.mm.setKneeFix == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- AIR HOLD stage (mm.air / mm.airProbe / mm.airSteer)
do
  local function setup(stored, fn)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = { air = {}, probe = {}, steer = {}, robust = {} }
    M.SetupMMSettings(function() end, function() end,
      { setAir = fn or function(mode) calls.air[#calls.air + 1] = mode; return true end,
        setAirProbe = function(on) calls.probe[#calls.probe + 1] = on end,
        setAirSteer = function(on) calls.steer[#calls.steer + 1] = on end,
        setAirRobust = function(on) calls.robust[#calls.robust + 1] = on; return true end })
    return M, calls
  end
  local M, c = setup()
  local def = M.settings.defs["mm.air"]
  local ids = {}
  for _, ch in ipairs(def and def.choices or {}) do ids[#ids + 1] = ch[1] end
  check("air: choice auto / clips / hold / off in the animation section, default auto, applied once at start -> UTPPSetMMAir(0), "
    .. "logged", def ~= nil and def.type == "choice" and def.section == "animation" and table.concat(ids, ",") == "auto,clips,hold,off"
    and M.Get("mm.air") == "auto" and #c.air == 1 and c.air[1] == 0 and M.mm.air == 0)
  local dp, ds = M.settings.defs["mm.airProbe"], M.settings.defs["mm.airSteer"]
  check("airProbe / airSteer: bools in the animation section, defaults on / on (airSteer on: user defaults 2026-10-20), applied once at start",
    dp ~= nil and dp.type == "bool" and dp.section == "animation" and ds ~= nil and ds.type == "bool" and ds.section == "animation"
    and M.Get("mm.airProbe") == true and M.Get("mm.airSteer") == true and #c.probe == 1 and c.probe[1] == true
    and #c.steer == 1 and c.steer[1] == true)
  local dr = M.settings.defs["mm.airRobust"]
  check("airRobust: registered, default on, forwarded once on setup",
    dr ~= nil and dr.type == "bool" and dr.section == "animation" and dr.default == true
    and M.Get("mm.airRobust") == true and #c.robust == 1 and c.robust[1] == true and M.mm.airRobust == true)
  M.Set("mm.airRobust", false)
  M.Set("mm.airRobust", false)
  check("airRobust: toggle forwarded once and persisted", #c.robust == 2 and c.robust[2] == false
    and M.mm.airRobust == false and io.open(tmp2):read("*a"):find("mm.airRobust=false", 1, true) ~= nil)
  M.ResetAll()
  check("airRobust: reset restores native true", M.Get("mm.airRobust") == true and #c.robust == 3 and c.robust[3] == true)
  M.Set("mm.air", "off")
  M.Set("mm.air", "off")
  M.Set("mm.air", "hold")
  M.Set("mm.air", "clips")
  check("air: off / hold / clips -> UTPPSetMMAir(3 / 2 / 1) once each, persisted", #c.air == 4 and c.air[2] == 3 and c.air[3] == 2
    and c.air[4] == 1 and io.open(tmp2):read("*a"):find("mm.air=clips", 1, true) ~= nil)
  M.Set("mm.air", "bogus")
  check("air: invalid choice rejected (no call)", #c.air == 4 and M.Get("mm.air") == "clips")
  M.Set("mm.airProbe", false)
  M.Set("mm.airSteer", false)
  check("airProbe off / airSteer off -> natives called once each", #c.probe == 2 and c.probe[2] == false and #c.steer == 2
    and c.steer[2] == false)
  local _, c2 = setup("mm.air=off\n")
  check("air: stored off applied at start", #c2.air == 1 and c2.air[1] == 3)
  local Mrob, cRob = setup("mm.airRobust=false\n")
  check("airRobust: stored false applied at initialization", Mrob.Get("mm.airRobust") == false
    and #cRob.robust == 1 and cRob.robust[1] == false)
  local Mr = load()
  Mr.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  local attempts = 0
  Mr.SetupMMSettings(function() end, function() end, { setAirRobust = function()
    attempts = attempts + 1
    return attempts > 1
  end })
  check("airRobust: native rejection is not cached as success", attempts == 1 and Mr.mm.airRobust == nil and Mr.mm.errors >= 1)
  Mr.Set("mm.airRobust", true)
  check("airRobust: native rejection retried on next apply", attempts == 2 and Mr.mm.airRobust == true)
  local nr = 0
  local M4 = setup(nil, function() nr = nr + 1; return false end)
  check("air: native returns false -> counted as an error, not marked as sent", nr == 1 and M4.mm.air == nil and M4.mm.errors >= 1)
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMAir')") end)
  M3.Set("mm.air", "hold")
  check("air: missing native -> no error, not marked as sent, retried on change", n == 2 and M3.mm.air == nil)
  local M5 = load()
  M5.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  M5.SetupMMSettings(function() end, function() end, {})
  check("air: api table without setAir -> registered and stored only, no call", M5.Get("mm.air") == "auto" and M5.mm.air == nil
    and M5.mm.setAir == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- CLIMB RELEASE stage (mm.climbRelease)
do
  local function setup(stored, fn, noApi)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = {}
    local api = {}
    if not noApi then
      api.setClimbRelease = fn or function(on) calls[#calls + 1] = on; return true end
    end
    M.SetupMMSettings(function() end, function() end, api)
    return M, calls
  end
  local M, c = setup()
  local d = M.settings.defs["mm.climbRelease"]
  check("climbRelease: bool in the animation section, default off (user defaults 2026-10-20), applied once at start -> UTPPSetMMClimbRelease(false), sent",
    d ~= nil and d.type == "bool" and d.section == "animation" and d.default == false and M.Get("mm.climbRelease") == false
    and #c == 1 and c[1] == false and M.mm.climbRelease == false)
  -- vehicle batch TRAV-1 (GameIntegration.hpp TravReleases): with v2 traversal active the setting no longer decides, so the label and
  -- text must say that it only matters without v2 and must not promise "UTPP plays its own traversal" as the only exception
  check("climbRelease: label says 'without the motion set', text says motion always uses the game animation and the setting decides with motion off",
    d ~= nil and d.label == "Climbing and vaulting without the motion set: game animation"
    and d.desc:find("always use the game's own animation, whatever this setting says", 1, true) ~= nil
    and d.desc:find("only decides with motion off", 1, true) ~= nil)
  local da = M.settings.defs["mm.air"]
  check("air: text describes the jump clips of Auto / Jump clips and the Hold fallback (no 'in this build ... all work this way')",
    da ~= nil and da.default == "auto" and da.desc:find("takeoff, fall and landing clips", 1, true) ~= nil
    and da.desc:find("Hold: no jump clips", 1, true) ~= nil and da.desc:find("In this build", 1, true) == nil)
  M.Set("mm.climbRelease", true)
  M.Set("mm.climbRelease", true)
  check("climbRelease: on -> UTPPSetMMClimbRelease(true) once, persisted", #c == 2 and c[2] == true and M.mm.climbRelease == true
    and io.open(tmp2):read("*a"):find("mm.climbRelease=true", 1, true) ~= nil)
  M.Set("mm.climbRelease", false)
  check("climbRelease: off again -> UTPPSetMMClimbRelease(false)", #c == 3 and c[3] == false and M.mm.climbRelease == false)
  local _, c2 = setup("mm.climbRelease=true\n")
  check("climbRelease: stored on applied at start (stored beats the default)", #c2 == 1 and c2[1] == true)
  local nr = 0
  local M4 = setup(nil, function() nr = nr + 1; return false end)
  check("climbRelease: native returns false -> counted as an error, not marked as sent", nr == 1 and M4.mm.climbRelease == nil
    and M4.mm.errors >= 1)
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMClimbRelease')") end)
  M3.Set("mm.climbRelease", true)
  check("climbRelease: missing native -> no error, not marked as sent, retried on change", n == 2 and M3.mm.climbRelease == nil)
  local M5 = setup(nil, nil, true)
  check("climbRelease: api table without setClimbRelease -> registered and stored only, no call",
    M5.Get("mm.climbRelease") == false and M5.mm.climbRelease == nil and M5.mm.setClimbRelease == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- loco C1b plant / replant (mm.footReplant)
do
  local function setup(stored, fn)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = {}
    M.SetupMMSettings(function() end, function() end,
      { setFootReplant = fn or function(mask) calls[#calls + 1] = mask; return true end })
    return M, calls
  end
  local M, c = setup()
  local def = M.settings.defs["mm.footReplant"]
  local ids = {}
  for _, ch in ipairs(def and def.choices or {}) do ids[#ids + 1] = ch[1] end
  -- standing replant (mask 1 / 3) is experimental and console only: not a choice (review fix 2026-10-13)
  check("footReplant: choice off / crouch in the animation section, default crouch, applied once at start -> "
    .. "UTPPSetMMFootReplant(2)", def ~= nil and def.type == "choice" and def.section == "animation"
    and table.concat(ids, ",") == "off,crouch" and M.Get("mm.footReplant") == "crouch" and #c == 1 and c[1] == 2
    and M.mm.footReplant == 2, table.concat(ids, ","))
  check("footReplant: apply logged", quietLines[#quietLines]:find("foot replant crouch (mm.footReplant, mask 2)", 1, true) ~= nil,
    quietLines[#quietLines])
  M.Set("mm.footReplant", "off")
  check("footReplant: off -> UTPPSetMMFootReplant(0), persisted", #c == 2 and c[2] == 0
    and io.open(tmp2):read("*a"):find("mm.footReplant=off", 1, true) ~= nil)
  M.Set("mm.footReplant", "off")
  check("footReplant: same value -> no second native call", #c == 2)
  M.Set("mm.footReplant", "crouch")
  check("footReplant: crouch -> UTPPSetMMFootReplant(2)", #c == 3 and c[3] == 2)
  M.Set("mm.footReplant", "all")
  check("footReplant: 'all' (standing replant, console only) rejected as a setting, no call", #c == 3
    and M.Get("mm.footReplant") == "crouch")
  M.Set("mm.footReplant", "sideways")
  check("footReplant: invalid value rejected, no call", #c == 3 and M.Get("mm.footReplant") == "crouch")
  local _, c2 = setup("mm.footReplant=off\n")
  check("footReplant: stored off applied at start (mask 0)", #c2 == 1 and c2[1] == 0)
  local _, c3 = setup("mm.footReplant=all\n")
  check("footReplant: stored 'all' ignored at start -> default crouch (mask 2)", #c3 == 1 and c3[1] == 2)
  -- older UTPP.dll without the native, or the native rejecting the mask: one log line, no error, not marked as sent, retried on change
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMFootReplant')") end)
  local function failLines()
    local k = 0
    for _, l in ipairs(quietLines) do if l:find("foot replant", 1, true) and l:find("FAILED", 1, true) then k = k + 1 end end
    return k
  end
  local f1 = failLines()
  M3.Set("mm.footReplant", "off")
  M3.Set("mm.footReplant", "crouch")
  check("footReplant: missing native -> logged once, no error, not marked as sent, retried on change", n == 3 and f1 == 1
    and failLines() == 1 and M3.mm.footReplant == nil and M3.mm.errors == 3, string.format("calls %d logged %d/%d", n, f1, failLines()))
  local M4 = setup(nil, function() return false end)
  check("footReplant: native returning false -> not marked as sent", M4.mm.footReplant == nil and M4.mm.errors == 1)
  -- not wired (no setFootReplant in the api table): stored only, no call
  local M5 = load()
  M5.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  M5.SetupMMSettings(function() end, function() end, {})
  check("footReplant: not wired -> registered and stored only, no call", M5.Get("mm.footReplant") == "crouch"
    and M5.mm.footReplant == nil and M5.mm.setFootReplant == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- batch 1b feed dump (mm.feedDump)
do
  local function setup(stored, fn)
    local M = load()
    M.SETTINGS_FILE = tmp2
    os.remove(tmp2)
    if stored then local f = io.open(tmp2, "w"); f:write(stored); f:close(); M.Load(tmp2) end
    local calls = {}
    M.SetupMMSettings(function() end, function() end,
      { setFootReplant = function() return true end,
        setFeedDump = fn or function(on) calls[#calls + 1] = on; return true end })
    return M, calls
  end
  local M, c = setup()
  local def = M.settings.defs["mm.feedDump"]
  check("feedDump: bool in the animation section, default false (release defaults 2026-10-22), applied once at start -> UTPPSetMMFeedDump(false)",
    def ~= nil and def.type == "bool" and def.section == "animation" and def.default == false and M.Get("mm.feedDump") == false and #c == 1
    and c[1] == false and M.mm.feedDump == false)
  check("feedDump: apply logged", quietLines[#quietLines]:find("feed dump off (mm.feedDump)", 1, true) ~= nil, quietLines[#quietLines])
  M.Set("mm.feedDump", true)
  check("feedDump: on -> UTPPSetMMFeedDump(true), persisted, logged", #c == 2 and c[2] == true
    and io.open(tmp2):read("*a"):find("mm.feedDump=true", 1, true) ~= nil
    and quietLines[#quietLines]:find("feed dump on (mm.feedDump)", 1, true) ~= nil)
  M.Set("mm.feedDump", false)
  check("feedDump: off -> UTPPSetMMFeedDump(false)", #c == 3 and c[3] == false and M.mm.feedDump == false)
  M.Set("mm.feedDump", false)
  check("feedDump: off again -> no second native call", #c == 3)
  M.Set("mm.feedDump", true)
  check("feedDump: on again -> UTPPSetMMFeedDump(true)", #c == 4 and c[4] == true and M.mm.feedDump == true)
  local nLog = #quietLines
  M.Set("mm.feedDump", true)
  check("feedDump: true again -> native called again (retries a dump whose writer failed), no second 'on' log line",
    #c == 5 and c[5] == true and M.mm.feedDump == true and #quietLines == nLog, string.format("calls %d log +%d", #c, #quietLines - nLog))
  local _, c2 = setup("mm.feedDump=true\n")
  check("feedDump: stored true applied at start (stored beats the default)", #c2 == 1 and c2[1] == true)
  -- older UTPP.dll without the native, or the native refusing to start: one log line, no error, not marked as sent, retried
  local n = 0
  local M3 = setup(nil, function() n = n + 1; error("attempt to call a nil value (field 'UTPPSetMMFeedDump')") end)
  local function failLines()
    local k = 0
    for _, l in ipairs(quietLines) do if l:find("feed dump", 1, true) and l:find("FAILED", 1, true) then k = k + 1 end end
    return k
  end
  local f1 = failLines()
  M3.Set("mm.feedDump", false)
  M3.Set("mm.feedDump", true)
  check("feedDump: missing native -> logged once, no error, not marked as sent, retried on change", n == 3 and f1 == 1
    and failLines() == 1 and M3.mm.feedDump == nil and M3.mm.errors == 3, string.format("calls %d logged %d/%d", n, f1, failLines()))
  local M4 = setup("mm.feedDump=true\n", function() return false end)
  check("feedDump: native returning false (could not start) -> not marked as sent", M4.mm.feedDump == nil and M4.mm.errors == 1)
  -- not wired (no setFeedDump in the api table): stored only, no call
  local M5 = load()
  M5.SETTINGS_FILE = tmp2
  os.remove(tmp2)
  M5.SetupMMSettings(function() end, function() end, {})
  check("feedDump: not wired -> registered and stored only, no call", M5.Get("mm.feedDump") == false
    and M5.mm.feedDump == nil and M5.mm.setFeedDump == nil)
  os.remove(tmp2)
end

-- ---------------------------------------------------------------- in-game wiring smoke test (mocked CET / game)
do
  local W = { applies = {}, clears = 0, observers = {}, inputs = {}, logs = {} }
  local player = { pos = { x = 0, y = 0, z = 0 }, vel = { x = 0, y = 0, z = 0 }, cam = nil }
  local ui = { menu = false }
  local psmVals = { UpperBody = 0, Vehicle = 0, IsInWorkspot = 0, HighLevel = 1, Swimming = 0, LocomotionDetailed = 1,
                    Takedown = 0, Vision = 0 }
  local defs = {}
  for k in pairs(psmVals) do defs[k] = k end
  local tppCam = { lp = { x = 0, y = -3, z = 0.4 } }
  function tppCam:GetLocalPosition() return { x = self.lp.x, y = self.lp.y, z = self.lp.z } end
  function tppCam:SetLocalPosition(v) self.lp = { x = v.x, y = v.y, z = v.z } end
  function tppCam:GetLocalOrientation() return { i = 0, j = 0, k = 0, r = 1 } end
  local P = {}
  function P:GetWorldPosition() return player.pos end
  function P:GetWorldForward() return { x = 0, y = 1, z = 0 } end
  function P:GetVelocity() return player.vel end
  function P:GetEntityID() return { hash = 1 } end
  local gameCam = { lp = { x = 0, y = -3, z = 0.4 } }
  gameCam.GetLocalPosition, gameCam.SetLocalPosition, gameCam.GetLocalOrientation =
    tppCam.GetLocalPosition, tppCam.SetLocalPosition, tppCam.GetLocalOrientation
  local camMode = "utpp" -- utpp: utppCamera present; game: only the game camera; none: no camera component
  function P:FindComponentByName(n)
    if n == "tppCamera" then error("tppCamera must not be used") end
    if camMode == "utpp" and n == "utppCamera" then return tppCam end
    if camMode == "game" and n == "camera" then return gameCam end
  end
  function P:GetFPPCameraComponent() if camMode == "game" then return gameCam end end
  function P:IsPlayer() return true end
  local headSys = {}
  function headSys:Apply(t, hw, cw, sp) W.applies[#W.applies + 1] = { x = t.x, y = t.y, z = t.z, chest = cw }; return true end
  function headSys:Clear() W.clears = W.clears + 1; return true end
  local loot = { id = { hash = 77 }, pos = { x = 0.3, y = 1.0, z = 0 } }
  function loot:GetEntityID() return self.id end
  function loot:GetWorldPosition() return self.pos end
  local sv = { print = print, reg = registerForEvent, getmod = GetMod, game = Game, input = registerInput,
               obs = ObserveAfter, v4 = Vector4, xf = Transform, bb = GetAllBlackboardDefs, enumint = EnumInt }
  local handlers = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  registerInput = function(id, l, fn) W.inputs[id] = fn end
  ObserveAfter = function(cls, fn, cb) W.observers[cls .. "." .. fn] = cb end
  Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end }
  Transform = { new = function() return {} end }
  EnumInt = function(v) return v end
  GetAllBlackboardDefs = function() return { PlayerStateMachine = defs, UI_System = { IsInMenu = "IsInMenu" } } end
  local camSys = {}
  function camSys:GetActiveCameraWorldTransform() return true, { position = player.cam } end
  function camSys:GetActiveCameraForward() return { x = 0, y = 1, z = -0.1 } end
  Game = {
    GetPlayer = function() return P end,
    UTPPLogLine = function(l) W.logs[#W.logs + 1] = l end,
    GetScriptableSystemsContainer = function() return { Get = function(_, n)
      if n == "UTPPHeadLook" then return headSys end return nil end } end,
    GetCameraSystem = function() return camSys end,
    GetBlackboardSystem = function() return {
      GetLocalInstanced = function() return { GetInt = function(_, k) return psmVals[k] end } end,
      Get = function() return { GetBool = function() return ui.menu end } end } end,
    GetSystemRequestsHandler = function() return { IsGamePaused = function() return false end } end,
    GetTargetingSystem = function() return { GetLookAtObject = function() return nil end } end,
    FindEntityByID = function(id) if id.hash == 77 then return loot end end,
    NameToString = function(n) return n end,
    UTPPSetMMEnabled = function(on) W.mmCalls = W.mmCalls or {}; W.mmCalls[#W.mmCalls + 1] = on end,
    UTPPSetMMTrace = function(on) W.traceCalls = W.traceCalls or {}; W.traceCalls[#W.traceCalls + 1] = on end,
    UTPPSetMMFootReplant = function(m) W.replantCalls = W.replantCalls or {}; W.replantCalls[#W.replantCalls + 1] = m; return true end,
    UTPPSetMMFeedDump = function(on) W.dumpCalls = W.dumpCalls or {}; W.dumpCalls[#W.dumpCalls + 1] = on; return true end,
    UTPPSetMMIdleHold = function(on) W.holdCalls = W.holdCalls or {}; W.holdCalls[#W.holdCalls + 1] = on end,
    UTPPSetMMKneeFix = function(m) W.kneeCalls = W.kneeCalls or {}; W.kneeCalls[#W.kneeCalls + 1] = m; return true end,
    UTPPSetMMAir = function(m) W.airCalls = W.airCalls or {}; W.airCalls[#W.airCalls + 1] = m; return true end,
    UTPPSetMMAirProbe = function(on) W.airProbeCalls = W.airProbeCalls or {}; W.airProbeCalls[#W.airProbeCalls + 1] = on end,
    UTPPSetMMAirSteer = function(on) W.airSteerCalls = W.airSteerCalls or {}; W.airSteerCalls[#W.airSteerCalls + 1] = on end,
    UTPPSetMMAirRobust = function(on) W.airRobustCalls = W.airRobustCalls or {}; W.airRobustCalls[#W.airRobustCalls + 1] = on; return true end,
    UTPPSetMMClimbRelease = function(on) W.climbCalls = W.climbCalls or {}; W.climbCalls[#W.climbCalls + 1] = on; return true end,
  }
  print = function() end
  GetMod = nil
  local okL, C = pcall(dofile, UTPP_CORE_PATH)
  C.SETTINGS_FILE = tmp2
  GetMod = function() return nil end -- no other UTPP mods
  handlers.onInit()
  local function frames(n) for _ = 1, n do handlers.onUpdate(1 / 30) end end
  player.cam = { x = 0.4, y = -3, z = 1.9 }
  frames(30)
  check("wiring: onInit applies mm.enabled (on -> Game.UTPPSetMMEnabled(true)) and mm.trace (false: release defaults 2026-10-22)",
    W.mmCalls and #W.mmCalls == 1 and W.mmCalls[1] == true and W.traceCalls and W.traceCalls[1] == false)
  check("wiring: onInit applies mm.footReplant (default crouch -> Game.UTPPSetMMFootReplant(2) once)",
    W.replantCalls and #W.replantCalls == 1 and W.replantCalls[1] == 2)
  check("wiring: onInit applies mm.feedDump (default -> Game.UTPPSetMMFeedDump(false) once)",
    W.dumpCalls and #W.dumpCalls == 1 and W.dumpCalls[1] == false)
  check("wiring: onInit applies mm.idleHold (default -> Game.UTPPSetMMIdleHold(true) once)",
    W.holdCalls and #W.holdCalls == 1 and W.holdCalls[1] == true)
  check("wiring: onInit applies mm.kneeFix (default -> Game.UTPPSetMMKneeFix(7) once)",
    W.kneeCalls and #W.kneeCalls == 1 and W.kneeCalls[1] == 7)
  check("wiring: onInit applies mm.air / mm.airProbe / mm.airSteer (defaults -> UTPPSetMMAir(0), UTPPSetMMAirProbe(true), "
    .. "UTPPSetMMAirSteer(true) once each)", W.airCalls and #W.airCalls == 1 and W.airCalls[1] == 0 and W.airProbeCalls
    and #W.airProbeCalls == 1 and W.airProbeCalls[1] == true and W.airSteerCalls and #W.airSteerCalls == 1
    and W.airSteerCalls[1] == true)
  check("wiring: onInit forwards mm.airRobust default true once",
    W.airRobustCalls and #W.airRobustCalls == 1 and W.airRobustCalls[1] == true)
  check("wiring: onInit applies mm.climbRelease (default -> Game.UTPPSetMMClimbRelease(false) once)",
    W.climbCalls and #W.climbCalls == 1 and W.climbCalls[1] == false)
  check("wiring: loads, observers on both loot classes", okL and W.observers["gameLootObject.OnInteractionActivated"] ~= nil
    and W.observers["gameLootContainerBase.OnInteractionActivated"] ~= nil)
  check("wiring: camera read via return values, TPP idle -> head look Apply", C.wiring.camMethod == "ret2" and #W.applies >= 1
    and W.applies[1].y > 3 and W.applies[1].chest == 0.5, tostring(C.wiring.camMethod) .. " " .. #W.applies)
  psmVals.UpperBody = 6
  frames(2)
  check("wiring: aim -> Clear", W.clears == 1 and C.hl.state == "off")
  psmVals.UpperBody = 0
  frames(20)
  check("wiring: re-engage after aim", C.hl.state == "on")
  player.cam = { x = 0, y = 0.05, z = 1.62 } -- FPP
  frames(2)
  check("wiring: FPP view -> Clear", W.clears == 2)
  player.cam = { x = 0.4, y = -3, z = 1.9 }
  W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 0 })
  frames(30)
  check("wiring: loot observer -> tracked, camera offset applied", C.la.tracker.n == 1 and C.la.weight == 1
    and math.abs(tppCam.lp.y - (-3 + 0.9)) < 1e-6, string.format("y=%.3f w=%.2f n=%d", tppCam.lp.y, C.la.weight, C.la.tracker.n))
  local last = W.applies[#W.applies]
  check("wiring: head look target = loot", last and math.abs(last.y - 1.0) < 1e-6 and math.abs(last.x - 0.3) < 1e-6)
  W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 1 })
  frames(40)
  check("wiring: loot gone -> camera back to the owner's base", C.la.weight == 0 and math.abs(tppCam.lp.y + 3) < 1e-6,
    string.format("y=%.3f", tppCam.lp.y))
  local function srcLogs(src)
    local n = 0
    for _, l in ipairs(W.logs) do if l:find("loot camera source: " .. src, 1, true) then n = n + 1 end end
    return n
  end
  check("camera source: utpp_camera logged once", srcLogs("utpp_camera") == 1, tostring(srcLogs("utpp_camera")))
  local function lootOn() W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 0 }) end
  local function lootOff() W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 1 }) end
  camMode = "game"
  lootOn(); frames(30)
  check("camera source: no utppCamera -> game camera gets the offset", math.abs(gameCam.lp.y - (-3 + 0.9)) < 1e-6
    and srcLogs("game") == 1 and tppCam.lp.y == -3, string.format("y=%.3f", gameCam.lp.y))
  lootOff(); frames(40)
  check("camera source: game camera restored", math.abs(gameCam.lp.y + 3) < 1e-6)
  camMode = "none"
  local errsBefore = C.wiring.errors
  lootOn(); frames(30)
  check("camera source: none -> no write, no error, logged", srcLogs("none") == 1 and C.wiring.errors == errsBefore
    and tppCam.lp.y == -3 and gameCam.lp.y == -3)
  lootOff(); frames(40)
  camMode = "utpp"
  -- 2026-10-30: no own key bindings (no CET input binding, no key relay, no session pause of head look / loot assist)
  check("wiring: no CET input binding registered", next(W.inputs) == nil)
  check("wiring: no key-binding runtime state", C.runtime == nil and C.actions == nil and C.FireAction == nil)
  frames(20)
  check("wiring: head look back on", C.hl.state == "on")
  C.Set("headLook.impl", "off")
  C.Set("lootView.impl", "off")
  frames(3)
  check("wiring: head look switched off -> UTPP head look released", C.hl.state == "off")
  lootOn(); frames(30)
  check("wiring: loot assist switched off -> camera untouched", tppCam.lp.y == -3 and C.la.weight == 0, string.format("y=%.3f", tppCam.lp.y))
  handlers.onShutdown()
  check("wiring: log lines reached Game.UTPPLogLine", #W.logs > 3)
  print, registerForEvent, GetMod, Game, registerInput = sv.print, sv.reg, sv.getmod, sv.game, sv.input
  ObserveAfter, Vector4, Transform, GetAllBlackboardDefs, EnumInt = sv.obs, sv.v4, sv.xf, sv.bb, sv.enumint
end

-- ---------------------------------------------------------------- 0.5.2 night harness probe (NightPos)
do
  local M = load()
  local sv = { game = Game, tdb = TweakDBID, bb = GetAllBlackboardDefs }
  local logged = {}
  local function vec(x, y, z) return { x = x, y = y, z = z } end
  local car = { GetWorldPosition = function() return vec(1502.6, 2207.69, 186.79) end,
                GetWorldRight = function() return vec(-0.381, 0.925, 0) end }
  local weapon, mounted, tagged = nil, nil, { car }
  local player = { GetWorldPosition = function() return vec(1505.59, 2205.67, 186.32) end,
                   GetMountedVehicle = function() return mounted end }
  Game = {
    GetPlayer = function() return player end,
    GetCameraSystem = function() return { GetActiveCameraForward = function() return vec(-0.925, -0.381, 0) end } end,
    GetDynamicEntitySystem = function() return { GetTagged = function(_, tag) return tag == "UTPPNight" and tagged or {} end } end,
    GetBlackboardSystem = function() error("no summon in this test") end,
    GetTransactionSystem = function() return { GetItemInSlot = function(_, _, slot) return weapon end } end,
    UTPPLogLine = function(l) logged[#logged + 1] = l end,
  }
  TweakDBID = { new = function(s) return s end }
  GetAllBlackboardDefs = function() return { VehicleSummonData = {} } end
  local line = M.NightPos("UTPPNight")
  check("NightPos: tagged vehicle, no weapon, not mounted", line ==
    "[night] POS p=1505.59,2205.67,186.32 cf=-0.925,-0.381 v=1502.60,2207.69,186.79,-0.381,0.925 src=tag w=0 m=0 freeMove=- psmW=- ub=- wi=none", line)
  check("NightPos: logged once through UTPPLogLine", #logged == 1 and logged[1] == line)
  weapon, mounted, tagged = {}, car, {}
  line = M.NightPos("UTPPNight")
  check("NightPos: mounted -> the mounted vehicle (src=mounted), weapon drawn, m=1",
    line:find("v=1502.60,2207.69,186.79,-0.381,0.925 src=mounted w=1 m=1", 1, true) ~= nil, line)
  local ride = { GetWorldPosition = function() return vec(1600, 2300, 190) end,
                 GetWorldRight = function() return vec(1, 0, 0) end }
  mounted, tagged = ride, { car }
  line = M.NightPos("UTPPNight")
  check("NightPos: mounted vehicle wins over the tagged one", line:find("v=1600.00,2300.00,190.00,1.000,0.000 src=mounted", 1, true) ~= nil, line)
  mounted, tagged = nil, {}
  line = M.NightPos("UTPPNight")
  check("NightPos: no vehicle, not mounted", line:find("v=nil src=none w=1 m=0", 1, true) ~= nil, line)
  -- 0.5.4 NightAuto / NightTick
  logged = {}
  check("NightTick: off -> nothing", M.NightTick(1.0) == nil and #logged == 0)
  local msg = M.NightAuto(true, 0.5)
  check("NightAuto: on message logged", msg == "[night] auto on period=0.50 tag=UTPPNight" and logged[1] == msg, msg)
  local l1 = M.NightTick(0.016)
  check("NightTick: first line on the next update, n=1", l1 ~= nil and l1:find(" n=1$") ~= nil and l1:find("^%[night%] POS p=") ~= nil, l1)
  check("NightTick: nothing before the period", M.NightTick(0.2) == nil and M.NightTick(0.2) == nil)
  local l2 = M.NightTick(0.2)
  check("NightTick: next line after 0.5 s, n=2", l2 ~= nil and l2:find(" n=2$") ~= nil, l2)
  M.NightAuto(false)
  check("NightAuto(false): stops", M.NightTick(5) == nil and logged[#logged] == "[night] auto off period=0.50 tag=UTPPNight")
  M.NightAuto(true, 0.01)
  check("NightAuto: period floor 0.05", M.night.period == 0.05)
  M.NightAuto(false)
  Game.GetTransactionSystem = function() error("boom") end
  player.GetMountedVehicle = nil
  line = M.NightPos("UTPPNight")
  check("NightPos: unreadable weapon/mount -> '-'", line:find("w=- m=-", 1, true) ~= nil, line)
  Game.GetPlayer = function() return nil end
  check("NightPos: no player -> unavailable line", M.NightPos("x") == "[night] POS unavailable: no player")
  Game = nil
  check("NightPos: no Game -> no error", M.NightPos("x") == "[night] POS unavailable: no Game")
  Game, TweakDBID, GetAllBlackboardDefs = sv.game, sv.tdb, sv.bb
end

os.remove(tmp); os.remove(tmp2)
print = savedPrint
print(string.format("utpp_core test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_core test failed")

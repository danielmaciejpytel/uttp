-- utpp_core Mod Settings bridge (offline): field names, value mapping, start / menu-close sync with the baseline file,
-- push of CET changes (batched, deferred while the Mod Settings screen is open or holds unconfirmed edits), float tolerance,
-- hidden options, restart with a changed Mod Settings value, missing framework, retired keys in the baseline file.
-- The Game calls are replaced by core.msync.api (a mock of utpp_modsettings_bridge.reds). Global: UTPP_CORE_PATH.
-- The mock page is not the generated class: T01-T16 use headLook_* as ordinary stand-in fields of a page (head look is hidden in
-- 0.1.0, so the real page has none); T17 tests the hidden-option rule against the real generated file.
local passed, failed = 0, 0
local function check(name, ok, detail)
  if ok then passed = passed + 1; print("[PASS] utpp_modsettings: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_modsettings: " .. name .. (detail ~= nil and (" -- " .. tostring(detail)) or "")) end
end

local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod,
                print = print, open = io.open }
local settingsFile, baseFile = os.tmpname(), os.tmpname()
os.remove(settingsFile)
os.remove(baseFile)
local logs = {}

local function newCore()
  registerForEvent, registerInput, Game = nil, nil, nil
  local realOpen = SAVED.open
  io.open = function(p, m) if p == "utpp_settings.txt" then p = settingsFile end return realOpen(p, m) end
  local core = dofile(UTPP_CORE_PATH)
  io.open = realOpen
  core.sink = function(line) logs[#logs + 1] = line end
  core.SETTINGS_FILE = settingsFile
  core.MS_BASE_FILE = baseFile
  core.Register({ key = "t.view", section = "test", type = "choice", default = "tpp",
                  choices = { { "fpp", "First person" }, { "tpp", "Third person" } }, label = "View" })
  core.Register({ key = "t.steps", section = "test", type = "int", default = 3, min = 1, max = 6, label = "Steps" })
  return core
end

-- Mod Settings mock: fields = field -> text; push compares texts (the bridge compares values)
local function mockMS(fields)
  local m = { fields = {}, open = false, pending = false, avail = true, snapshots = 0, pushes = {} }
  for k, v in pairs(fields or {}) do m.fields[k] = v end
  m.api = {
    available = function() return m.avail end,
    menuOpen = function() return m.open end,
    snapshot = function()
      m.snapshots = m.snapshots + 1
      local ks, out = {}, {}
      for k in pairs(m.fields) do ks[#ks + 1] = k end
      table.sort(ks)
      for _, k in ipairs(ks) do out[#out + 1] = k .. "=" .. m.fields[k] end
      return table.concat(out, "\n") .. "\n"
    end,
    push = function(data)
      m.pushes[#m.pushes + 1] = data
      if m.open or m.pending then return -2 end
      local n = 0
      for line in string.gmatch(data, "[^\n]+") do
        local f, v = string.match(line, "^([%w_]+)=(.*)$")
        if m.fields[f] ~= nil and m.fields[f] ~= v then m.fields[f] = v; n = n + 1 end
      end
      return n
    end,
  }
  return m
end
-- Mod Settings page with the class defaults of the shown options
local function defaultsPage()
  return { headLook_impl = "true", headLook_maxYaw = "100.000000", crosshair_color = "0", t_view = "1", t_steps = "3" }
end
local function tick(core, n, dt)
  for _ = 1, n or 1 do core.ModSettingsTick(dt or 0.3) end
end
local function readFile(p)
  local f = SAVED.open(p, "r")
  if not f then return nil end
  local s = f:read("*a")
  f:close()
  return s
end
local function pushedFields(data)
  local out = {}
  for line in string.gmatch(data or "", "[^\n]+") do
    local f = string.match(line, "^([%w_]+)=")
    if f then out[#out + 1] = f end
  end
  table.sort(out)
  return table.concat(out, ",")
end

local ok, err = pcall(function()
  -- ---------------------------------------------------------------- T01 field rule and value mapping
  local core = newCore()
  local S = core.settings
  check("T01 field = key with non-alphanumerics as _", core.ModSettingsField("camera.height") == "camera_height"
    and core.ModSettingsField("camera.zoomUnarmed.enabled") == "camera_zoomUnarmed_enabled")
  local dView, dColor, dYaw = S.defs["t.view"], S.defs["crosshair.color"], S.defs["headLook.maxYaw"]
  check("T01 choice -> index in choice order", core.ModSettingsEncode(dView, "fpp") == "0" and core.ModSettingsEncode(dView, "tpp") == "1"
    and core.ModSettingsEncode(dColor, "red") == "2")
  check("T01 index -> choice", core.ModSettingsDecode(dView, "0") == "fpp" and core.ModSettingsDecode(dColor, "4") == "black"
    and core.ModSettingsDecode(dColor, "9") == nil and core.ModSettingsDecode(dColor, "1.5") == nil)
  check("T01 float clamped to the registry range", core.ModSettingsDecode(dYaw, "500.0") == 140
    and core.ModSettingsDecode(dYaw, "x") == nil)
  check("T01 same: float32 tolerance", core.ModSettingsSame(dYaw, 100, 100.0000001)
    and not core.ModSettingsSame(dYaw, 100, 100.5))

  -- ---------------------------------------------------------------- T02 first run: CET values kept and pushed
  core.Set("headLook.maxYaw", 120)
  core.Set("crosshair.color", "green")
  core.Set("t.view", "fpp")
  local ms = mockMS(defaultsPage())
  core.msync.api = ms.api
  tick(core, 1)
  check("T02 state ready, synced at the first tick", core.msync.state == "ready" and core.msync.reconciled == true)
  check("T02 CET values unchanged (no baseline = CET wins)", S.values["headLook.maxYaw"] == 120
    and S.values["crosshair.color"] == "green" and S.values["t.view"] == "fpp")
  check("T02 one batched push of the 3 differing options", #ms.pushes == 1
    and pushedFields(ms.pushes[1]) == "crosshair_color,headLook_maxYaw,t_view", ms.pushes[1])
  check("T02 Mod Settings shows the CET values", ms.fields.headLook_maxYaw == "120.000000" and ms.fields.crosshair_color == "3"
    and ms.fields.t_view == "0")
  check("T02 dirty cleared, baseline written", next(core.msync.dirty) == nil and string.find(readFile(baseFile) or "",
    "headLook.maxYaw=120", 1, true) ~= nil)
  local snaps = ms.snapshots
  tick(core, 4)
  check("T02 no further pushes or snapshots while nothing changes", #ms.pushes == 1 and ms.snapshots == snaps)

  -- ---------------------------------------------------------------- T03 Mod Settings edit -> import on screen close
  ms.open = true
  tick(core, 2)
  ms.fields.headLook_maxYaw = "90.000000"
  ms.fields.crosshair_color = "1"
  check("T03 nothing applied while the screen is open", S.values["headLook.maxYaw"] == 120 and ms.snapshots == snaps)
  ms.open = false
  tick(core, 1)
  check("T03 imported on close", S.values["headLook.maxYaw"] == 90 and S.values["crosshair.color"] == "white")
  check("T03 no echo push", #ms.pushes == 1 and next(core.msync.dirty) == nil)
  check("T03 settings file saved", string.find(readFile(settingsFile) or "", "headLook.maxYaw=90", 1, true) ~= nil)

  -- ---------------------------------------------------------------- T04 CET change -> push, deferred while open
  core.Set("t.steps", 5)
  check("T04 changed key dirty", core.msync.dirty["t.steps"] == true)
  ms.open = true
  tick(core, 3)
  check("T04 no push while the screen is open", #ms.pushes == 1)
  ms.open = false
  tick(core, 1)
  check("T04 close: CET change survives the sync and is pushed", S.values["t.steps"] == 5 and ms.fields.t_steps == "5"
    and #ms.pushes == 2 and pushedFields(ms.pushes[2]) == "t_steps", ms.pushes[2])
  core.Set("headLook.maxYaw", 95)
  core.Set("headLook.maxYaw", 100)
  core.Set("t.steps", 6)
  tick(core, 4)
  check("T04 changes within one interval are batched", #ms.pushes == 3
    and pushedFields(ms.pushes[3]) == "headLook_maxYaw,t_steps" and ms.fields.headLook_maxYaw == "100.000000")

  -- ---------------------------------------------------------------- T05 unconfirmed edits in Mod Settings: retry slowly
  ms.pending = true
  core.Set("t.view", "tpp")
  local n0 = #ms.pushes
  core.msync.pushAt = 0
  tick(core, 1, 0.25)
  local n1 = #ms.pushes
  check("T05 push attempted and deferred, key stays dirty", n1 == n0 + 1 and core.msync.dirty["t.view"] == true
    and core.msync.stats.deferred == 1)
  tick(core, 3, 0.25)
  check("T05 no retry before the interval", #ms.pushes == n1)
  tick(core, 1, 0.25)
  check("T05 retried after the interval", #ms.pushes == n1 + 1)
  ms.pending = false
  tick(core, 4)
  check("T05 pushed once accepted", ms.fields.t_view == "1" and core.msync.dirty["t.view"] == nil)

  -- ---------------------------------------------------------------- T06 both sides changed: Mod Settings wins
  core.msync.api = { available = ms.api.available, menuOpen = function() return true end, snapshot = ms.api.snapshot,
                     push = ms.api.push }
  core.Set("t.steps", 2)        -- CET change, push deferred (screen open)
  tick(core, 1)
  ms.fields.t_steps = "4"        -- edit in the open screen
  core.msync.api = ms.api
  tick(core, 1)
  check("T06 both changed -> Mod Settings value", S.values["t.steps"] == 4 and ms.fields.t_steps == "4"
    and core.msync.dirty["t.steps"] == nil)

  -- ---------------------------------------------------------------- T07 hidden option (not on the page) never pushed
  local before = #ms.pushes
  core.Set("core.log", false)
  tick(core, 4)
  check("T07 hidden option dropped from the dirty set, no push", #ms.pushes == before and core.msync.dirty["core.log"] == nil)

  -- ---------------------------------------------------------------- T08 float tolerance
  ms.open = true; tick(core, 1)
  ms.fields.headLook_maxYaw = "100.000003"
  ms.open = false; tick(core, 1)
  check("T08 float32 noise is not a change", S.values["headLook.maxYaw"] == 100 and #ms.pushes == before)

  -- ---------------------------------------------------------------- T09 invalid Mod Settings value: CET wins
  ms.open = true; tick(core, 1)
  ms.fields.crosshair_color = "9"
  ms.open = false; tick(core, 1)
  check("T09 invalid enum value replaced by the CET value", S.values["crosshair.color"] == "white" and ms.fields.crosshair_color == "1")

  -- ---------------------------------------------------------------- T10 ResetAll pushes the defaults
  core.ResetAll()
  tick(core, 4)
  check("T10 reset reaches Mod Settings", ms.fields.crosshair_color == "0" and ms.fields.t_steps == "3"
    and next(core.msync.dirty) == nil)

  -- ---------------------------------------------------------------- T11 restart: edit made in the main menu of a new start
  core.Set("headLook.maxYaw", 130)
  tick(core, 4)
  check("T11 setup: synced 130", ms.fields.headLook_maxYaw == "130.000000")
  local core2 = newCore()
  check("T11 new start loads the CET value", core2.settings.values["headLook.maxYaw"] == 130)
  ms.fields.headLook_maxYaw = "60.000000"  -- changed in Mod Settings while the game was closed / before this sync
  ms.fields.t_view = "0"
  core2.Set("t.steps", 6)                   -- changed in CET (e.g. settings file) since the last sync
  local np = #ms.pushes
  core2.msync.api = ms.api
  tick(core2, 1)
  check("T11 Mod Settings change imported, CET change pushed", core2.settings.values["headLook.maxYaw"] == 60
    and core2.settings.values["t.view"] == "fpp" and ms.fields.t_steps == "6" and pushedFields(ms.pushes[np + 1]) == "t_steps",
    ms.pushes[np + 1])

  -- ---------------------------------------------------------------- T12 detection without a confirmed page
  os.remove(baseFile) -- first run: the CET value wins once Mod Settings is confirmed
  local core3 = newCore()
  local m3 = mockMS(defaultsPage())
  m3.avail = false
  core3.msync.api = m3.api
  check("T12 available() false -> pending (checked again)", core3.ModSettingsPresent() == false and core3.msync.state == "pending")
  core3.Set("t.steps", 2)
  tick(core3, 5)
  check("T12 pending: no snapshot or push", m3.snapshots == 0 and #m3.pushes == 0)
  m3.avail = true
  tick(core3, 4)
  check("T12 confirmed by a later check, then synced", core3.msync.state == "ready" and core3.msync.reconciled == true
    and m3.fields.t_steps == "2")
  local core4 = newCore()
  local calls4 = 0
  core4.msync.api = { available = function() calls4 = calls4 + 1; error("UTPPModSettingsAvailable missing") end }
  check("T12 failing call = pending", core4.ModSettingsPresent() == false and core4.msync.state == "pending"
    and string.find(core4.msync.lastReason or "", "UTPPModSettingsAvailable missing", 1, true) ~= nil)
  tick(core4, 8, 0.25)
  check("T12 retried once per MS_DETECT_RETRY", calls4 == 3, calls4)
  local core5 = newCore()
  local m5 = mockMS({ someOtherMod_option = "true" })
  core5.msync.api = m5.api
  check("T12 listed mod = confirmed (static data, no snapshot needed)", core5.ModSettingsPresent() == true)
  tick(core5, 4)
  check("T12 snapshot without options of this registry: not synced, no push", core5.msync.reconciled == false
    and #m5.pushes == 0)
  local core6 = newCore()
  local m6 = mockMS(defaultsPage())
  m6.api.variant = function() return "stub" end
  core6.msync.api = m6.api
  check("T12 bridge compiled without Mod Settings = absent (final)", core6.ModSettingsPresent() == false
    and core6.msync.state == "absent" and core6.msync.variant == "stub")
  tick(core6, 8)
  check("T12 absent: no further checks", m6.snapshots == 0)

  -- ---------------------------------------------------------------- T13 onInit menu choice and the Game call path
  -- mode: "ms" = bridge works at onInit, "late" = bridge calls fail at onInit and work from onUpdate, "none" = no bridge,
  -- "stub" = bridge compiled without Mod Settings
  local function bootInGame(mode)
    os.remove(baseFile)
    local handlers, tabs = {}, {}
    local m = mockMS(defaultsPage())
    local phase = "init"
    registerForEvent = function(ev, fn) handlers[ev] = fn end
    registerInput = function() end
    Game = {}
    if mode ~= "none" then
      local function gate(fn) return function(...)
        if mode == "late" and phase == "init" then error("script call unavailable at onInit") end
        return fn(...)
      end end
      Game.UTPPModSettingsBridgeVariant = gate(function() return mode == "stub" and "stub" or "modsettings" end)
      Game.UTPPModSettingsBridgeInfo = gate(function() return "variant=" .. (mode == "stub" and "stub" or "modsettings") end)
      Game.UTPPModSettingsAvailable = gate(function() return mode ~= "stub" and m.api.available() end)
      Game.UTPPModSettingsMenuOpen = gate(m.api.menuOpen)
      Game.UTPPModSettingsSnapshot = gate(m.api.snapshot)
      Game.UTPPModSettingsPush = gate(m.api.push)
    end
    local ns = { data = {}, currentTab = nil, removed = {} }
    function ns.addTab(p) tabs[#tabs + 1] = p; ns.data[string.sub(p, 2)] = { subcategories = {} } end
    function ns.pathExists(p) return ns.data[string.sub(p, 2)] ~= nil end
    function ns.addSubcategory() end
    function ns.removeSubcategory(p) ns.removed[#ns.removed + 1] = p end
    for _, n in ipairs({ "addSwitch", "addRangeFloat", "addRangeInt", "addSelectorString", "addKeyBinding", "addButton" }) do
      ns[n] = function() return {} end
    end
    GetMod = function(name) if name == "nativeSettings" then return ns end return nil end
    local realOpen = SAVED.open
    io.open = function(p, md) if p == "utpp_settings.txt" then p = settingsFile end return realOpen(p, md) end
    local c = dofile(UTPP_CORE_PATH)
    io.open = realOpen
    c.SETTINGS_FILE, c.MS_BASE_FILE = settingsFile, baseFile
    c.sink = function(line) logs[#logs + 1] = line end
    print = function() end
    pcall(handlers.onInit)
    local afterInit = { built = c.ui.built, state = c.msync.state, tabs = #tabs }
    phase = "update"
    c.Set("headLook.maxYaw", 110)
    if mode == "late" then ns.currentTab = "utpp" end -- the UTPP tab is open: removal waits
    for _ = 1, 5 do pcall(handlers.onUpdate, 0.1) end
    afterInit.builtWhileOpen = c.ui.built
    ns.currentTab = nil
    for _ = 1, 10 do pcall(handlers.onUpdate, 0.1) end
    print = SAVED.print
    registerForEvent, registerInput, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.GetMod
    Game = SAVED.Game
    return c, tabs, m, ns, afterInit
  end
  local cA, tabsA, mA = bootInGame("ms")
  check("T13 Mod Settings confirmed at onInit: no Native Settings tab", #tabsA == 0 and cA.msync.state == "ready"
    and cA.ui.built == false)
  check("T13 onUpdate syncs through the Game calls", cA.msync.reconciled == true and mA.fields.headLook_maxYaw == "110.000000")
  local cL, tabsL, mL, nsL, initL = bootInGame("late")
  check("T13 calls failing at onInit: fallback tab built, state pending", initL.tabs == 1 and initL.built == true
    and initL.state == "pending")
  check("T13 tab kept while it is open", initL.builtWhileOpen == true)
  check("T13 confirmed from onUpdate: tab removed, synced", cL.msync.state == "ready" and cL.ui.built == false
    and nsL.data.utpp == nil and #nsL.removed > 0 and cL.msync.reconciled == true and mL.fields.headLook_maxYaw == "110.000000")
  local cB, tabsB = bootInGame("none")
  check("T13 no bridge: Native Settings tab built and kept", #tabsB == 1 and tabsB[1] == "/utpp" and cB.ui.built == true
    and cB.msync.state == "pending")
  local cS, tabsS = bootInGame("stub")
  check("T13 stub bridge: tab built, absent (no more checks)", #tabsS == 1 and cS.ui.built == true and cS.msync.state == "absent")
  check("T13 status line names variant and tab", string.find(cS.ModSettingsStatus(), "variant=stub", 1, true) ~= nil
    and string.find(cS.ModSettingsStatus(), "nativeTab=true", 1, true) ~= nil)

  -- ---------------------------------------------------------------- T14 revision, session, framework without ConfigVars
  os.remove(baseFile)
  local c7 = newCore()
  local m7 = mockMS(defaultsPage())
  local rev7 = -1
  m7.api.revision = function() return rev7 end
  c7.msync.api = m7.api
  tick(c7, 4)
  check("T14 no session (revision -1): confirmed but not read", c7.msync.state == "ready" and c7.msync.reconciled == false
    and m7.snapshots == 0)
  rev7 = 0
  tick(c7, 1)
  check("T14 session: first read and sync", c7.msync.reconciled == true and m7.snapshots == 1)
  local s7 = m7.snapshots
  tick(c7, 4)
  check("T14 unchanged revision: no further reads", m7.snapshots == s7)
  m7.fields.t_steps = "5"
  rev7 = 1
  tick(c7, 1)
  check("T14 change notification (screen not seen open): imported", c7.settings.values["t.steps"] == 5)
  rev7 = -1
  tick(c7, 1)
  rev7 = 0
  m7.fields.t_steps = "4"
  tick(c7, 1)
  check("T14 new session: read again", c7.settings.values["t.steps"] == 4)

  -- framework lists the mod but returns no ConfigVars: CET values stay, Mod Settings edits still apply, slow retry
  os.remove(baseFile)
  local c8 = newCore()
  c8.Set("headLook.maxYaw", 120)
  local m8 = mockMS(defaultsPage())
  local push8 = 0
  local rev8 = 0
  m8.api.revision = function() return rev8 end
  m8.api.push = function() push8 = push8 + 1; return -3 end
  c8.msync.api = m8.api
  tick(c8, 1)
  check("T14 no ConfigVars: CET value kept, push refused", c8.settings.values["headLook.maxYaw"] == 120 and push8 == 1
    and c8.msync.dirty["headLook.maxYaw"] == true)
  tick(c8, 8)
  check("T14 no ConfigVars: retried slowly", push8 == 1)
  m8.fields.headLook_maxYaw = "70.000000"
  rev8 = 1
  tick(c8, 1)
  check("T14 no ConfigVars: Mod Settings edit applies", c8.settings.values["headLook.maxYaw"] == 70
    and c8.msync.dirty["headLook.maxYaw"] == nil)
  rev8 = 2
  tick(c8, 1)
  check("T14 no ConfigVars: unchanged Mod Settings value does not overwrite CET", c8.settings.values["t.steps"] ~= nil
    and c8.settings.values["headLook.maxYaw"] == 70)

  -- ---------------------------------------------------------------- T15 retired keys in an old baseline file (2026-10-30)
  -- the baseline of a version with key bindings holds bind.* lines: they are not read (RETIRED_KEYS), so the next save of the
  -- baseline drops them; other lines (here bind.somethingElse) are kept and the baseline still decides the sync
  os.remove(settingsFile)
  local fb = SAVED.open(baseFile, "w")
  fb:write("# UTPP: values last synced with Mod Settings\nbind.enabled=true\nbind.headlook_toggle=IK_F7\n"
    .. "bind.camera_freelook=IK_Pad_LeftShoulder\nbind.somethingElse=1\nheadLook.maxYaw=100\nmm.shoulderNarrow=2.5\nt.steps=3\n")
  fb:close()
  local c9 = newCore()
  local m9 = mockMS(defaultsPage())
  m9.fields.t_steps = "5"  -- changed in Mod Settings since the baseline (CET still 3)
  c9.msync.api = m9.api
  tick(c9, 1)
  check("T15 old baseline read: Mod Settings change against it imported", c9.msync.reconciled == true
    and c9.settings.values["t.steps"] == 5, c9.settings.values["t.steps"])
  check("T15 retired keys not kept in the baseline table", c9.msync.base["bind.enabled"] == nil
    and c9.msync.base["bind.headlook_toggle"] == nil and c9.msync.base["bind.camera_freelook"] == nil
    and c9.msync.base["bind.somethingElse"] == "1")
  -- 2026-11-09: a migrated key (utpp_core MIGRATED_KEYS: mm.shoulderNarrow -> mm.shoulderWidth) is dropped the same way
  check("T15 migrated key not kept in the baseline table nor in the saved baseline", c9.msync.base["mm.shoulderNarrow"] == nil
    and (readFile(baseFile) or ""):find("shoulderNarrow", 1, true) == nil)
  local b9 = readFile(baseFile) or ""
  check("T15 saved baseline without the retired bind lines, other lines kept", b9:find("bind.enabled=", 1, true) == nil
    and b9:find("bind.headlook_toggle=", 1, true) == nil and b9:find("bind.camera_freelook=", 1, true) == nil
    and b9:find("bind.somethingElse=1", 1, true) ~= nil and b9:find("t.steps=5", 1, true) ~= nil, b9)

  -- ---------------------------------------------------------------- T16 migrated value vs the Mod Settings class default (2026-11-09)
  -- a player of an older version: utpp_settings.txt holds mm.shoulderNarrow=2 (-> mm.shoulderWidth 1), the baseline of the
  -- last sync has no mm.shoulderWidth line, and Mod Settings shows the new field at its class default 0 (no user.ini entry):
  -- the migrated CET value must win and be pushed (the bridge must not undo the migration)
  os.remove(settingsFile)
  local fs = SAVED.open(settingsFile, "w")
  fs:write("# UTPP settings\nmm.shoulderNarrow=2\nt.steps=3\n")
  fs:close()
  local fb2 = SAVED.open(baseFile, "w")
  fb2:write("# UTPP: values last synced with Mod Settings\nmm.shoulderNarrow=2.5\nt.steps=3\n")
  fb2:close()
  local c16 = newCore()
  c16.Register({ key = "mm.shoulderWidth", section = "animation", type = "float", default = 0, min = -5, max = 5, step = 0.5,
                 label = "Shoulder width (female V, cm)" })
  check("T16 migrated on load: mm.shoulderWidth = 1, old key not stored", c16.settings.values["mm.shoulderWidth"] == 1
    and c16.settings.stored["mm.shoulderNarrow"] == nil, c16.settings.values["mm.shoulderWidth"])
  local m16 = mockMS(defaultsPage())
  m16.fields.mm_shoulderWidth = "0.000000"
  c16.msync.api = m16.api
  tick(c16, 1)
  check("T16 Mod Settings class default 0 does not undo the migrated value; CET value pushed",
    c16.settings.values["mm.shoulderWidth"] == 1 and tonumber(m16.fields.mm_shoulderWidth) == 1
    and next(c16.msync.dirty) == nil, m16.fields.mm_shoulderWidth)
  local b16 = readFile(baseFile) or ""
  check("T16 baseline: new key synced, old key dropped", b16:find("mm.shoulderWidth=1", 1, true) ~= nil
    and b16:find("shoulderNarrow", 1, true) == nil, b16)

  -- ---------------------------------------------------------------- T17 hidden options (registry flag hidden = true, 2026-11-12)
  -- the six head look options are hidden in 0.1.0: no class field, never in the snapshot. Their CET value, default and settings
  -- file stay as they are; nothing is pushed, imported or kept in the baseline for them. A shown switch syncs both ways; a baseline of false must not read as "no baseline" (ModSettingsSync).
  local reds = SAVED.open((string.gsub(UTPP_CORE_PATH, "[/\\]cet[/\\]utpp_core[/\\]init%.lua$", "/scripts/utpp_modsettings.reds")), "rb")
  local redsText = reds and reds:read("*a") or ""
  if reds then reds:close() end
  check("T17 generated class: no field, snapshot line or listing for the four hidden head look options; the head look switch and Look at the camera are there",
    #redsText > 1000 and string.find(redsText, "UTPPSnapshot", 1, true) ~= nil
    and string.find(redsText, "headLook_whileWalking", 1, true) == nil and string.find(redsText, "headLook_lookAtLoot", 1, true) == nil
    and string.find(redsText, "headLook_maxYaw", 1, true) == nil and string.find(redsText, "headLook_chest", 1, true) == nil
    and string.find(redsText, "public let headLook_impl: Bool = true;", 1, true) ~= nil
    and string.find(redsText, '"headLook_impl=" + UTPPMSBool(this.headLook_impl)', 1, true) ~= nil
    and string.find(redsText, "public let headLook_faceCamera: Bool = true;", 1, true) ~= nil
    and string.find(redsText, '"headLook_faceCamera=" + UTPPMSBool(this.headLook_faceCamera)', 1, true) ~= nil
    and string.find(redsText, "public let camera_aim_distance", 1, true) ~= nil)
  -- the options flagged through utpp_core's HIDDEN_KEYS (registered by other modules) have no field either
  local leaked = {}
  for _, f in ipairs({ "camera_aim_occlusion", "mm_pelvisYawTrim", "mm_bodyYawOffset", "mm_minWalkSpeed", "camera_body", "camera_collision",
                       "mm_aimAtCrosshair", "perspective_sceneTier", "interactRange_scale", "lootRange_scale", "lootView_zoom",
                       "lootView_radius", "vehicle_keepTpp", "vehicle_pullOut", "vehicle_rideWatchdog", "vehicle_safeTeleportExit",
                       "camera_mouseScale", "perspective_minGap", "camera_aim_range", "camera_zoomUnarmed_fov",
                       "camera_zoomUnarmed_lastFov", "perspective_blend", "mm_headLead" }) do
    if string.find(redsText, "public let " .. f .. ":", 1, true) or string.find(redsText, '"' .. f .. '="', 1, true) then
      leaked[#leaked + 1] = f
    end
  end
  check("T17 generated class: no field, snapshot line or listing for the 23 options flagged hidden by utpp_core", #redsText > 1000 and #leaked == 0,
    table.concat(leaked, ", "))
  os.remove(settingsFile)
  os.remove(baseFile)
  local c17 = newCore()
  local D = c17.settings.defs
  check("T17 four head look options hidden, switch and Look at the camera visible; defaults intact", D["headLook.impl"].hidden == nil
    and D["headLook.whileWalking"].hidden == true and D["headLook.faceCamera"].hidden == nil
    and D["headLook.lookAtLoot"].hidden == true and D["headLook.maxYaw"].hidden == true and D["headLook.chest"].hidden == true
    and c17.Get("headLook.impl") == true and c17.Get("headLook.maxYaw") == 100 and c17.Get("headLook.chest") == 0.5)
  c17.Set("headLook.maxYaw", 80)
  c17.Set("headLook.impl", false)
  c17.Set("t.steps", 5)
  c17.Register({ key = "t.lead", section = "test", type = "bool", default = true, label = "Lead" })  -- a shown switch
  c17.Set("t.lead", false)
  -- the page as generated: the shown switch (class default true), no field for the hidden options
  local m17 = mockMS({ t_lead = "true", crosshair_color = "0", t_view = "1", t_steps = "3" })
  c17.msync.api = m17.api
  tick(c17, 1)
  check("T17 sync: shown options (switch, steps) are pushed, no hidden key in the push", #m17.pushes == 1
    and pushedFields(m17.pushes[1]) == "t_lead,t_steps" and m17.fields.t_lead == "false"
    and m17.fields.headLook_maxYaw == nil and next(c17.msync.dirty) == nil, m17.pushes[1])
  m17.open = true
  tick(c17, 2)
  m17.fields.t_steps = "2"
  m17.fields.t_lead = "true"  -- the player switches it on in Mod Settings (the baseline holds false)
  m17.open = false
  tick(c17, 1)
  local b17 = readFile(baseFile) or ""
  local s17 = readFile(settingsFile) or ""
  check("T17 Mod Settings edit of a switch with a false baseline is imported; the hidden values stay", c17.Get("t.steps") == 2
    and c17.Get("t.lead") == true and c17.Get("headLook.impl") == false and c17.Get("headLook.maxYaw") == 80,
    tostring(c17.Get("t.steps")) .. " " .. tostring(c17.Get("t.lead")))
  check("T17 baseline has no hidden key, settings file keeps them", b17:find("headLook", 1, true) == nil
    and b17:find("t.lead=true", 1, true) ~= nil and s17:find("headLook.maxYaw=80", 1, true) ~= nil
    and s17:find("headLook.impl=false", 1, true) ~= nil, b17)
  c17.Set("t.lead", false)  -- CET side change -> pushed to Mod Settings
  tick(c17, 1)
  check("T17 CET change of the switch is pushed", m17.fields.t_lead == "false" and #m17.pushes >= 2 and next(c17.msync.dirty) == nil)
  check("T17 hidden option stays settable from the console", c17.Set("headLook.maxYaw", 120) == 120 and c17.Get("headLook.maxYaw") == 120
    and c17.Set("headLook.impl", true) == true and string.find(c17.Status(), "headLook=true", 1, true) ~= nil)
end)
if not ok then check("no Lua error", false, err) end

io.open = SAVED.open
registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
print = SAVED.print
os.remove(settingsFile)
os.remove(baseFile)
print(string.format("utpp_modsettings test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_modsettings test failed")

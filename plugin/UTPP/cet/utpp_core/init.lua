-- UTPP utpp_core: shared CET module of the UTPP mods (standalone mod).
--
--   * settings registry (typed options, defaults, validation), persistence (utpp_settings.txt in this mod folder),
--     reset, Native Settings tab "UTPP" builder, two-way sync with the "Ultimate Third Person" page of the Mod Settings
--     screen (used instead of the tab when Mod Settings is installed); other UTPP CET mods register their options through
--     GetMod("utpp_core");
--   * menu layout: the tab is a projection of the registry (standard view = own groups and levels, view "all" = every
--     option under its source section; GetMod("utpp_core").SetMenuView("standard"|"all") for this session only);
--   * logging: print + Game.UTPPLogLine (UTPP red4ext log; CET prints do not reach a file in game);
--   * per-feature ON/OFF switches ("Use UTPP ..." bool settings, keys *.impl and mm.enabled; values saved by older versions:
--     auto / on -> on, off -> off);
--   * no own key bindings (the stored bind.* keys of older versions are retired settings, dropped on load);
--   * idle head look: V looks where the camera looks (or at the camera when it faces V, or at nearby loot) while standing
--     or walking in a third-person view; released on running, aiming, vehicles, workspots, scenes, swimming, ladders ...
--     Game side: the DLL head look layer (Game.UTPPSetHeadLook: target, weight, mode, chest share, turn limit every tick; it turns
--     Spine3 a little, Neck, Neck1, Head and the eyes on the final pose) - the decision stays here; UTPPHeadLook (utpp.reds) = one
--     static LookAtAddEvent target on the player, used only as the fallback while the DLL layer cannot act (older DLL, MM off);
--   * loot view: nearby loot (interaction activation of gameLootObject / gameLootContainerBase + the object
--     under the crosshair) selected by distance and camera angle; the third-person camera moves closer by up to `zoom` m.
--
-- Console: print(GetMod("utpp_core").Status())     GetMod("utpp_core").Set("headLook.impl", true|false)
--          GetMod("utpp_core").ResetAll()
--          GetMod("utpp_core").SetDatabase("default"|"<name>")  (animation database, red4ext/plugins/UTPP/db)
--          GetMod("utpp_core").NightPos("UTPPNight")  (test harness probe: one
--          "[night] POS ..." line in the UTPP log - player position, camera forward, the vehicle tagged <tag> (Codeware
--          DynamicEntitySystem) or the summoned player vehicle, weapon in the right hand, mounted; while
--          mounted the vehicle V sits in, src=mounted)
--          GetMod("utpp_core").NightAuto(true, 0.5[, "UTPPNight"])  (the same POS line every 0.5 s from
--          onUpdate with a counter " n=<k>" - the harness reads it from the log instead of typing NightPos);
--          GetMod("utpp_core").NightAuto(false) stops it
--          GetMod("utpp_core").Set("mm.enabled", true|false)  GetMod("utpp_core").Set("mm.trace", true|false)
--          (motion matching switches applied at game start through Game.UTPPSetMMEnabled / Game.UTPPSetMMTrace)

local M = { VERSION = "0.5.5" }

local function clamp(x, a, b) if x < a then return a elseif x > b then return b end return x end
local function smoothstep(t) t = clamp(t, 0, 1); return t * t * (3 - 2 * t) end
local function copy(t) local r = {} for k, v in pairs(t) do r[k] = v end return r end

-- ================================================================ logging
-- Log lines never carry the player's file system paths: Lua error texts name the script file with its full path (user name
-- included). A path inside the CET mods folder is cut to "mods\..."; the game folder prefix is dropped (the rest is relative to
-- it); a user profile path becomes "~"; any other drive path is replaced by "<path>".
function M.ScrubPaths(s)
  s = tostring(s)
  if not string.find(s, "%a:[\\/]") then return s end
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-([Mm]ods[\\/])", "%1")
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-Cyberpunk 2077[\\/]", "")
  s = string.gsub(s, "%a:[\\/][Uu]sers[\\/][^\\/%s\"']+", "~")
  s = string.gsub(s, "%a:[\\/][^%s\"']*", "<path>")
  return s
end
M.logTail = {}
function M.Log(msg)
  local line = "[utpp_core] " .. M.ScrubPaths(msg)
  M.logTail[#M.logTail + 1] = line
  if #M.logTail > 60 then table.remove(M.logTail, 1) end
  if M.sink then M.sink(line) return end
  print(line)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(M.ScrubPaths(line)) end) end
end
local loggedOnce = {}
function M.LogOnce(key, msg)
  if loggedOnce[key] then return end
  loggedOnce[key] = true
  M.Log(msg)
end

-- ================================================================ settings registry
-- def = { key, section, type = "bool"|"float"|"int"|"choice", label, desc, default, min, max, step, format,
--         choices = { {id, label}, ... }, apply = function(value) end, initial (value used when nothing stored) }
-- "Use UTPP ..." switches (keys *.impl and mm.enabled) are plain bool settings. Values saved by older versions
-- ("auto" / "on" -> true, "off" -> false) migrate when validated / loaded; type = "impl" is accepted as an alias of such a bool.
local S = { defs = {}, order = {}, values = {}, stored = {}, sections = {}, sectionOrder = {}, widgets = {}, uiSync = false }
M.settings = S
M.SETTINGS_FILE = "utpp_settings.txt"
-- Mod Settings bridge state (section "Mod Settings bridge" below): Set marks every changed key dirty unless the change came
-- from Mod Settings itself (importing); the bridge pushes the dirty keys that the Mod Settings screen shows.
M.msync = { dirty = {}, importing = false }
-- "Diagnostic logs" (core.diagnosticLogs, registered with core.log below; "off" | "timed" | "until"): while it is not "off",
-- these diagnostic switches are held on (Register and Set keep them true); switching it off restores each one to its
-- registered default
M.DIAG_KEY = "core.diagnosticLogs"
-- mm.debug is not held (user): it draws the "UTPP motion" window on screen, it is not a log
M.DIAG_KEYS = { "core.log", "mm.trace", "mm.feedDump", "camera.trace", "perspective.log" }
local DIAG_SET = {}
for _, k in ipairs(M.DIAG_KEYS) do DIAG_SET[k] = true end
local function diagHeld(key) return DIAG_SET[key] == true and S.values[M.DIAG_KEY] ~= nil and S.values[M.DIAG_KEY] ~= "off" end

-- Animation pack (animation.pack: "utpp" = the mod's animations, "game" = the game's own animations with the third-person
-- view kept). The setting (requested) is never rewritten; the pack in force (applied) changes at a safe moment only. While
-- "game" is in force, M.Effective returns the PACK_FORCED value for the listed keys, so the options of the player stay stored
-- and come back with "utpp". Everything else is wired in the "animation pack" section below.
M.PACK_KEY = "animation.pack"
M.PACK_FORCED = {
  game = {
    ["mm.enabled"] = false, ["mm.motionDriver"] = false, ["bodyturn.shooterMode"] = false, ["melee.fullBody"] = false,
    ["melee.aimTilt"] = false, ["mm.aimAtCrosshair"] = false, ["camera.idleOrbit"] = false, ["camera.followHoldOnFlip"] = false,
  },
}
-- options that only work with the mod's animations: their menu text ends with PACK_NOTE (added when the option is registered)
M.PACK_NOTE = " No effect with the Game Animations pack."
M.PACK_DEPENDENT = {}
for _, k in ipairs({ "mm.pelvisYawTrim", "mm.bodyYawOffset", "mm.walkSpeed", "mm.minWalkSpeed", "mm.shoulderWidth",
  "mm.shinobiBonusReload", "mm.aimAtCrosshair", "melee.fullBody", "melee.aimTilt", "bodyturn.shooterMode", "camera.idleOrbit",
  "camera.followHoldOnFlip", "vehicle.approachWalk", "mm.headLead" }) do M.PACK_DEPENDENT[k] = true end
-- applied: pack in force; requested: value of the setting; pending: a change waits for a safe moment; sent: what the game
-- scripts of the session were told last (nil = unknown, a fresh session starts with the mod's animations)
M.pack = { requested = "utpp", applied = "utpp", pending = false, sent = nil, sentAt = -1e9, safeFor = 0, inSession = false,
  deferWhy = nil, api = nil, calls = 0, errors = 0, applies = 0 }

local function legacyOnOff(def)
  return def.onOff == true or def.key == "mm.enabled" or string.match(def.key or "", "%.impl$") ~= nil
end

local function validate(def, v)
  local t = def.type
  if t == "bool" then
    if type(v) == "boolean" then return v end
    if v == "true" then return true elseif v == "false" then return false end
    if legacyOnOff(def) then
      if v == "on" or v == "auto" then return true elseif v == "off" then return false end
    end
    return nil
  elseif t == "float" or t == "int" then
    local n = tonumber(v)
    if n == nil or n ~= n then return nil end
    if def.min ~= nil then n = math.max(def.min, n) end
    if def.max ~= nil then n = math.min(def.max, n) end
    if t == "int" then n = math.floor(n + 0.5) end
    return n
  elseif t == "choice" then
    for _, c in ipairs(def.choices or {}) do if c[1] == v then return v end end
    return nil
  end
  return nil
end
M.ValidateSetting = validate

local function encode(def, v)
  if def.type == "float" then return string.format("%.4g", v) end
  return tostring(v)
end

-- position of each key in S.order (presentation tie-break of the menu layout; S.order itself is never re-sorted)
local orderIndex = {}

-- Options left out of every menu in 0.1.0: the same hidden = true flag as on the head look
-- registrations, applied here for the options that other modules register (Register sets def.hidden). They are registered and work
-- on their current values (defaults, fresh-install initial values, Get / Set / Effective, the Game Animations pack's forced
-- values, the settings file and the console are unchanged); only their menu entries are left out. The layout entries stay.
M.HIDDEN_KEYS = {}
for _, k in ipairs({ "camera.aim.occlusion", "mm.pelvisYawTrim", "mm.bodyYawOffset", "mm.minWalkSpeed", "camera.body",
  "camera.collision", "mm.aimAtCrosshair", "perspective.sceneTier", "interactRange.scale", "lootRange.scale", "lootView.zoom",
  "lootView.radius", "vehicle.keepTpp", "vehicle.pullOut", "vehicle.rideWatchdog", "vehicle.safeTeleportExit", "camera.mouseScale",
  "perspective.minGap", "camera.aim.range", "camera.zoomUnarmed.fov", "camera.zoomUnarmed.lastFov" }) do M.HIDDEN_KEYS[k] = true end

function M.RegisterSection(id, label, opts)
  if S.sections[id] then return S.sections[id] end
  local sec = { id = id, label = label, keys = {}, owner = opts and opts.owner or "utpp_core", built = false }
  S.sections[id] = sec
  S.sectionOrder[#S.sectionOrder + 1] = id
  if M.ui and M.ui.built and M.LateSection then M.LateSection(sec) end
  return sec
end

function M.Register(def)
  assert(type(def) == "table" and type(def.key) == "string" and type(def.type) == "string", "utpp_core.Register: bad def")
  if def.type == "impl" then def.type, def.onOff = "bool", true end
  local default = validate(def, def.default)
  if default == nil then error("utpp_core.Register: invalid default for " .. def.key) end
  def.default = default
  if M.HIDDEN_KEYS[def.key] then def.hidden = true end
  if M.PACK_DEPENDENT[def.key] and type(def.desc) == "string" and string.sub(def.desc, -#M.PACK_NOTE) ~= M.PACK_NOTE then
    def.desc = def.desc .. M.PACK_NOTE
  end
  local fresh = S.defs[def.key] == nil
  S.defs[def.key] = def
  if fresh then
    S.order[#S.order + 1] = def.key
    orderIndex[def.key] = #S.order
  end
  local v = nil
  if S.stored[def.key] ~= nil then v = validate(def, S.stored[def.key]) end
  if v == nil and def.initial ~= nil then v = validate(def, def.initial) end
  if v == nil then v = default end
  if diagHeld(def.key) then v = true end -- registered while the diagnostic logs are on (stored value ignored)
  S.values[def.key] = v
  if def.apply then
    local ok, err = pcall(def.apply, M.Effective(def.key))
    if not ok then M.Log("apply " .. def.key .. " failed: " .. tostring(err)) end
  end
  if def.section then
    local sec = S.sections[def.section] or M.RegisterSection(def.section, def.section)
    if fresh then
      sec.keys[#sec.keys + 1] = def.key
      if M.ui and M.ui.built and M.LateWidget then M.LateWidget(def) end
    end
  end
  return v
end

function M.Get(key) return S.values[key] end
-- the effective value of a setting (a bool switch: its boolean); the animation pack in force may override it (PACK_FORCED)
function M.Effective(key)
  local forced = M.PACK_FORCED[M.pack.applied]
  if forced ~= nil and forced[key] ~= nil then return forced[key] end
  return S.values[key]
end

local function syncWidget(key)
  local w, ns = S.widgets[key], M.ui and M.ui.ns
  if not (w and ns and ns.setOption) then return end
  local def = S.defs[key]
  local shown = M.Effective(key)
  if def.type == "choice" then
    for i, c in ipairs(def.choices) do if c[1] == shown then shown = i end end
  end
  local prev = S.uiSync -- restored (not forced off): a sync inside another sync keeps the outer guard
  S.uiSync = true
  pcall(ns.setOption, w, shown)
  S.uiSync = prev
end

-- opts.noSave: do not write the file (bulk operations)
function M.Set(key, value, opts)
  local def = S.defs[key]
  if not def then return nil, "unknown setting " .. tostring(key) end
  local v = validate(def, value)
  if v == nil then return nil, "invalid value for " .. key .. ": " .. tostring(value) end
  if v ~= true and diagHeld(key) then
    v = true
    M.LogOnce("diagnosticLogs.held:" .. key, key .. " stays on while \"Diagnostic logs\" is on (switch that option off first)")
  end
  local changed = S.values[key] ~= v
  S.values[key] = v
  if changed and not M.msync.importing then M.msync.dirty[key] = true end
  if def.apply then
    local ok, err = pcall(def.apply, M.Effective(key))
    if not ok then M.Log("apply " .. key .. " failed: " .. tostring(err)) end
  end
  if changed and not (opts and opts.noSave) then pcall(M.Save) end
  if not S.uiSync then syncWidget(key) end
  return v
end

function M.ResetAll()
  for _, key in ipairs(S.order) do M.Set(key, S.defs[key].default, { noSave = true }) end
  pcall(M.Save)
  M.Log("settings reset to defaults")
  return true
end

-- keys of options that no module registers any more. Save() keeps unknown stored keys (modules not loaded this
-- session), so a removed option would stay in the file forever; these exact keys are dropped when the file is read (the next
-- Save no longer writes them). Every other unknown key is kept as before.
-- perspective.enforceWithRef: the weaponDrawn=tpp "enforce with ref" option of utpp_perspective, removed with the
-- perspective simplification (nothing reads it).
-- the own key-binding system was removed: bind.enabled and the 11 bind.<action> strings (exact keys, so a
-- "bind.*" key of another mod stays). Also skipped when the Mod Settings sync baseline is read (msLoadBase).
local RETIRED_KEYS = { ["perspective.enforceWithRef"] = true,
  ["bind.enabled"] = true, ["bind.perspective_toggle"] = true, ["bind.shoulder_swap"] = true, ["bind.quick_fpp"] = true,
  ["bind.profile_next"] = true, ["bind.profile_prev"] = true, ["bind.headlook_toggle"] = true,
  ["bind.lootassist_toggle"] = true, ["bind.camera_freelook"] = true, ["bind.camera_zoom_in"] = true,
  ["bind.camera_zoom_out"] = true, ["bind.loothead_kill"] = true }

-- keys replaced by a new key with a converted value. Load converts the stored value (when the file has no value of
-- the new key; a stored new key wins) and drops the old key like a retired one (the next Save no longer writes it; the Mod
-- Settings sync baseline skips it as well). convert(text) -> value of the new key or nil (invalid: the new key keeps its default).
-- mm.shoulderNarrow (female V shoulder narrowing, cm per side 0 .. 4, default 2.5) -> mm.shoulderWidth (cm against the
-- default look, both shoulders together, -5 .. 5, default 0): width = 5 - 2 * narrow (2.5 -> 0, 2 -> 1, 0 -> 5, 4 -> -3).
local MIGRATED_KEYS = {
  ["mm.shoulderNarrow"] = { key = "mm.shoulderWidth", convert = function(v)
    local n = tonumber(v)
    if n == nil or n ~= n or n == math.huge or n == -math.huge then return nil end
    return 5 - 2 * n
  end },
}
M.MIGRATED_KEYS = MIGRATED_KEYS

function M.Load(path)
  local f = io.open(path or M.SETTINGS_FILE, "r")
  if not f then return false end
  local migrated = {}
  for line in f:lines() do
    local k, v = string.match(line, "^%s*([%w_%.]+)%s*=%s*(.-)%s*$")
    if k and not string.match(line, "^%s*#") then
      if MIGRATED_KEYS[k] then
        S.stored[k] = nil
        migrated[#migrated + 1] = { from = k, raw = v }
      elseif RETIRED_KEYS[k] then
        S.stored[k] = nil
        M.LogOnce("retired:" .. k, "retired setting " .. k .. " dropped from the stored settings (no module uses it any more)")
      else
        S.stored[k] = v
        local def = S.defs[k]
        if def then
          local val = validate(def, v)
          if val ~= nil then S.values[k] = val end
        end
      end
    end
  end
  f:close()
  -- converted after the whole file is read, so a stored new key wins whatever the line order
  for _, m in ipairs(migrated) do
    local spec = MIGRATED_KEYS[m.from]
    local to = spec.key
    if S.stored[to] == nil then
      local okC, nv = pcall(spec.convert, m.raw)
      local def = S.defs[to]
      if okC and nv ~= nil and (def == nil or validate(def, nv) ~= nil) then
        S.stored[to] = def and encode(def, validate(def, nv)) or string.format("%.4g", nv)
        if def then S.values[to] = validate(def, S.stored[to]) end
        M.LogOnce("migrated:" .. m.from, "setting " .. m.from .. "=" .. tostring(m.raw) .. " converted to " .. to .. "=" .. S.stored[to])
      else
        M.LogOnce("migrated:" .. m.from, "setting " .. m.from .. "=" .. tostring(m.raw) .. " dropped (invalid; " .. to .. " keeps its default)")
      end
    else
      M.LogOnce("migrated:" .. m.from, "setting " .. m.from .. " dropped (" .. to .. " is stored)")
    end
  end
  -- Load writes values without apply callbacks. The pack in force is taken from the stored setting here (nothing runs yet, so
  -- nothing is deferred), and the diagnostic logs are applied, so a stored "until switched off" holds the diagnostic switches
  -- (in the game both keys are registered before this file is read, so this is the normal start path). "On for 5 minutes" does
  -- not survive a restart: it becomes "off" (and is pushed to Mod Settings as such).
  if S.defs[M.PACK_KEY] and M.LoadPack then pcall(M.LoadPack) end
  if S.defs[M.DIAG_KEY] and M.LoadDiagnosticLogs then pcall(M.LoadDiagnosticLogs) end
  return true
end

-- Writes every registered value plus stored keys of modules not loaded this session (kept as read).
function M.Save(path)
  local f = io.open(path or M.SETTINGS_FILE, "w")
  if not f then return false end
  f:write("# UTPP settings (utpp_core " .. M.VERSION .. "): key=value, delete a line to restore its default\n")
  local written = {}
  for _, key in ipairs(S.order) do
    if M.diag and M.diag.mode ~= "off" and DIAG_SET[key] then
      -- held by "On for 5 minutes" or "On until switched off": the held value is never persisted. The file keeps the value
      -- of the player (S.stored: captured when the mode took hold, else the last saved value, else the default), so a restart
      -- after the timed mode, or after a switch-off, leaves nothing on; the permanent mode is persisted by the list itself
      f:write(key .. "=" .. tostring(S.stored[key] or encode(S.defs[key], S.defs[key].default)) .. "\n")
    else
      f:write(key .. "=" .. encode(S.defs[key], S.values[key]) .. "\n")
      S.stored[key] = encode(S.defs[key], S.values[key])
    end
    written[key] = true
  end
  local extra = {}
  for k in pairs(S.stored) do if not written[k] then extra[#extra + 1] = k end end
  table.sort(extra)
  for _, k in ipairs(extra) do f:write(k .. "=" .. tostring(S.stored[k]) .. "\n") end
  f:close()
  return true
end

-- ================================================================ Native Settings tab
-- The tab is a projection of the registry above: it never changes a key, default, value, registration / apply order or
-- the settings file. Two views (session only, never saved):
--   * "standard" (default): own groups (M.MENU_GROUPS: General, Camera, Combat, Interactions, Vehicles, Advanced, Debug) with
--     the player and advanced options of each group in layout order (an advanced option stays in its own group),
--     a "Menu" group (view switch, reset). Developer and diagnostic options get no widget here; they keep their stored
--     values, callbacks and console access (GetMod("utpp_core").Set(key, value)).
--   * "all": every option with a section under its source section (/utpp/<section>, registration order), as before, plus
--     the "Menu" group.
-- Options registered after the tab was built (other mods load later) follow the same rule; an option that the layout does
-- not list goes to "Advanced", so it is never lost from the standard view.
M.ui = { built = false, ns = nil, view = "standard", pendingView = nil, tabOrder = {}, groups = {}, viewWidgets = {} }
local TAB = "/utpp"
local unpack = table.unpack or unpack

-- 7 categories; the group ids stay (field names, tests and utpp_camera's views use them), the labels changed
M.MENU_GROUPS = {
  { id = "start", label = "General" },
  { id = "frame", label = "Camera" },
  { id = "combat", label = "Combat" },
  { id = "world", label = "Interactions" },
  { id = "vehicles", label = "Vehicles" },
  { id = "advanced", label = "Advanced" },
  { id = "debug", label = "Debug" },
  { id = "menu", label = "Menu" },
}
M.MENU_LEVELS = { player = true, advanced = true, developer = true, diagnostic = true }
local GROUP_INDEX = {}
for i, g in ipairs(M.MENU_GROUPS) do GROUP_INDEX[g.id] = i end
-- { level, group, keys }: rank = running position of the key in its group across the blocks below, in steps of 10 (room
-- for presentation entries such as RegisterView: utpp_camera's framing views use ranks 1-4, so they open "Camera"), so a
-- group can mix levels in the listed order. Levels: player / advanced = shown in the standard view and in Mod Settings,
-- developer / diagnostic = "Show all options" only. label = presentation-only override.
local LAYOUT = {
  --   General: "Use Ultimate Third Person" first, "Animation pack" second, then "Shooter mode"; "Diagnostic logs" last
  --   no "Start in third person" option: the mod always starts in third person (perspective.defaultTpp and
  -- camera.startTpp are in the Debug developer list below, kept only so stored values load; Start in the last used view stays)
  --   "Use motion matching" (mm.enabled) is developer level: the animation pack decides which animations play
  { "player", "start", { "camera.impl", "animation.pack" } },
  { "advanced", "start", { "perspective.impl" } },
  { "developer", "start", { "mm.enabled" } },
  { "player", "start", { "bodyturn.shooterMode", "perspective.startInLastView", "core.diagnosticLogs" } },
  --   Camera: one fixed camera profile, no "Active profile" selector; utpp_camera's framing views
  -- (distance, shoulder offset, FOV, height: ranks 1-4) come first, then these
  --   "Run pelvis yaw trim" (mm.pelvisYawTrim, registered by utpp_motion) after the camera controls, then
  -- "Body yaw offset" (mm.bodyYawOffset, utpp_motion: the whole character turned against the over-the-shoulder parallax);
  --   "Head follows turns" (mm.headLead, registered below: the head turns into a run turn; player level since 0.1.0) right
  -- after it;
  --   "Walk speed (m/s)" (mm.walkSpeed, utpp_motion: capsule speed under the walk toggle) after it, then "Minimum walk
  -- speed (m/s)" (mm.minWalkSpeed, utpp_motion: capsule speed floor of a small stick tilt)
  --   "Shoulder width (female V, cm)" (mm.shoulderWidth, utpp_motion) last: the menu has no body category and the
  -- other options that shape how V looks in the third-person view are here
  { "player", "frame", { "camera.sens", "camera.invertX", "camera.invertY", "camera.zoomUnarmed.enabled", "mm.pelvisYawTrim",
                         "mm.bodyYawOffset", "mm.headLead", "mm.walkSpeed", "mm.minWalkSpeed", "mm.shoulderWidth" } },
  { "advanced", "frame", { "camera.idleOrbit", "camera.followHoldOnFlip", "camera.collision", "perspective.animate",
                           "perspective.blend", "camera.body", "camera.audio" } },
  { "developer", "frame", { "body.impl", "headGuard.impl" } },
  --   Combat: "Reload animation during Shinobi bonus" (mm.shinobiBonusReload, default off) after the weapon
  --   options, then crosshair.impl "Use UTPP crosshair" (default on); crosshair.color "Crosshair colour"
  -- (default Standard = the game's HUD colour) right after it; then "Weapon aims at crosshair point" (mm.aimAtCrosshair, utpp_motion:
  --   the DLL barrel pitch loop + utpp_camera's crosshair point feed)
  -- melee full body + melee aim tilt (both default on; full body only for the melee weapon types the UTPP
  -- melee clips cover, scripts\utpp_melee.reds)
  { "player", "combat", { "perspective.aiming", "perspective.aimingScoped", "perspective.combat", "perspective.weaponDrawn",
                          "mm.shinobiBonusReload", "crosshair.impl", "crosshair.color", "mm.aimAtCrosshair",
                          "melee.fullBody", "melee.aimTilt" } },
  { "advanced", "combat", { "perspective.shortScopeFpp", "camera.aim.enabled", "camera.aim.distance", "camera.aim.side",
                            "camera.aim.zoomFov", "camera.aim.fov", "camera.aim.autoSwap", "camera.aimReturn",
                            "camera.aimReturnBlend", "perspective.combatCut", "perspective.combatExit",
                            "perspective.meleeBlockTpp", "perspective.takedown" } },
  --   Release scope: melee guard (default off; the third-person guard pose is not installed - the fists
  -- melee_idlepose stays vanilla) is developer level, so the release menu has no option without content
  { "developer", "combat", { "bodyturn.aimJog", "melee.guard", "melee.guardFullBody", "melee.guardAimTilt",
                              "bodyturn.meleeStrikeTurn" } },
  -- Interactions
  { "player", "world", { "perspective.scene", "lootView.impl", "interactRange.impl", "headLook.impl",
                        "headLook.faceCamera" } },
  { "advanced", "world", { "headLook.whileWalking", "headLook.lookAtLoot", "headLook.maxYaw",
                           "headLook.chest", "interactRange.scale", "lootRange.impl", "lootRange.scale", "lootView.zoom",
                           "lootView.radius", "perspective.sceneTier", "perspective.sceneExit", "perspective.interaction",
                           "perspective.interactionExit", "perspective.workspot", "perspective.traversal",
                           "perspective.scanner" } },
  -- Vehicles (vehicle.safeTeleportExit: registered by utpp_vehiclecam)
  --   Release scope: the ragdoll after a car hit is not part of the release (no ragdoll archives are shipped), so
  -- ragdoll.enabled is developer level (not in Mod Settings) and defaults to off
  { "player", "vehicles", { "perspective.vehicleKeepTpp", "vehicle.forceTppOnEnter" } },
  { "advanced", "vehicles", { "vehicle.keepTpp", "vehicle.pullOut", "vehicle.safeTeleportExit", "vehicle.rideWatchdog",
                              "vehicle.approachWalk", "vehicle.jumpOutSidePush" } },
  { "developer", "vehicles", { "body.headGuard", "ragdoll.enabled", "ragdoll.impulseScale", "ragdoll.maxSeconds",
                                "ragdoll.getUpPush", "ragdoll.getUp" } },
  -- Advanced
  { "advanced", "advanced", { "camera.mouseScale", "perspective.minGap", "camera.aim.range",
                              "camera.zoomUnarmed.steps", "camera.zoomUnarmed.distance", "camera.zoomUnarmed.lastDistance",
                              "camera.zoomUnarmed.fov", "camera.zoomUnarmed.lastFov", "camera.zoomUnarmed.fppHandoff" } },
  { "developer", "advanced", { "camera.followMaxRate", "camera.enterTime", "camera.exitTime", "camera.aim.boomPitchScale",
                               "camera.aim.boomFrame", "camera.zoomUnarmed.boomPitchScale", "camera.zoomUnarmed.boomFrame",
                               "body.garmentOffsets", "mm.motionDriver", "mm.pistolAuto", "mm.pistol1h", "bodyturn.aimJogSpeed",
                               "bodyturn.aimStrafeUnarmed", "bodyturn.turnRateMax", "mm.orientWarp",
                               "mm.slide", "mm.lean", "mm.leanScale", "mm.leanHead", "mm.leanHeadScale" } },
  --   Debug: the diagnostic logs have one visible list in General (the single switches stay diagnostic level,
  -- console only); camera.aim.occlusion stays visible (detection only, the switch saves its rays)
  { "advanced", "debug", { "camera.aim.occlusion" } },
  { "diagnostic", "debug", { "core.log", "perspective.log", "camera.trace", "mm.trace", "mm.feedDump", "mm.debug" } },
  { "developer", "debug", { "mm.database", "camera.pitchSource", "camera.pivotSource", "camera.decoupleLocomotion",
                            "camera.predictBodyTurn", "camera.worldCamera", "camera.worldCameraRetry", "camera.lookSync",
                            "camera.lookWrite", "camera.aim.ownYaw", "camera.poseNudge", "body.hideFpp", "body.showTpp",
                            "body.shadowProxy", "body.garments", "body.retry", "bodyturn.weaponTurn", "bodyturn.turnRate",
                            "bodyturn.aimStrafe", "bodyturn.aimChestAlign", "bodyturn.guardFix", "bodyturn.aimSmooth",
                            "mm.footReplant", "mm.weaponUpper", "mm.aimUpperFade", "mm.idleHold",
                            "mm.kneeFix", "mm.kneeHold", "mm.air", "mm.airProbe", "mm.airSteer", "mm.airRobust",
                            "mm.climbRelease", "mm.grip", "mm.shootTransfer", "mm.aimGameUpper",
                            -- inert without a player trigger (no perspective key), console ToggleUser only
                            "perspective.toggleOverrides",
                            -- start perspective is no longer a user option (always third person)
                            "perspective.defaultTpp", "camera.startTpp", "camera.fppWithWeapon" } },
}
M.MENU_LAYOUT = {}
do
  local rank = {}
  for _, block in ipairs(LAYOUT) do
    local g = block[2]
    for _, key in ipairs(block[3]) do
      rank[g] = (rank[g] or 0) + 10
      M.MENU_LAYOUT[key] = { level = block[1], group = g, rank = rank[g] }
    end
  end
end
-- camera.startTpp is used only when the automatic perspective module is not installed (developer level)
M.MENU_LAYOUT["camera.startTpp"].label = "Start in third person (only without the automatic perspective module)"
-- key patterns checked when the layout does not list the key: { pattern, level, group, rank base } (rank = base + position in
-- the registration order): raw preset fields and per-weapon overrides, and future unarmed zoom tuning keys after the listed
-- Advanced options (the current camera.zoomUnarmed keys are listed above)
M.MENU_PATTERNS = { { "^camera%.distance$", "developer", "debug", 0 }, { "^camera%.height$", "developer", "debug", 0 },
                    { "^camera%.side$", "developer", "debug", 0 }, { "^camera%.fov$", "developer", "debug", 0 },
                    { "^camera%.framingSens$", "developer", "debug", 0 }, { "^mm%.weapon%.", "developer", "advanced", 0 },
                    { "^camera%.zoomUnarmed%.", "advanced", "advanced", 1000 } }

-- level, group, rank, source ("layout" | "pattern" | "default" | "no section") of a key in the menu layout
function M.MenuPlacement(key)
  local def = S.defs[key]
  if def and not def.section then return "internal", nil, nil, "no section" end
  local l = M.MENU_LAYOUT[key]
  -- def.hidden: registered and fully functional, but listed in no menu (Native Settings tab, Mod Settings page); the layout
  -- entry is kept as the place it returns to when the flag is removed
  if def and def.hidden then
    local hg = l and GROUP_INDEX[l.group] and l.group ~= "menu" and l.group or "advanced"
    return "hidden", hg, l and tonumber(l.rank) or 0, "hidden"
  end
  if l then
    local group = GROUP_INDEX[l.group] and l.group ~= "menu" and l.group or "advanced"
    local level = M.MENU_LEVELS[l.level] and l.level or "advanced"
    return level, group, tonumber(l.rank) or 0, "layout"
  end
  for _, p in ipairs(M.MENU_PATTERNS) do
    if string.match(key, p[1]) then
      local group = GROUP_INDEX[p[3]] and p[3] ~= "menu" and p[3] or "advanced"
      return M.MENU_LEVELS[p[2]] and p[2] or "advanced", group, (tonumber(p[4]) or 0) + (orderIndex[key] or 0), "pattern"
    end
  end
  return "advanced", "advanced", orderIndex[key] or 0, "default"
end

-- presentation entries that are not settings (frozen API, used by utpp_camera's active profile editor):
-- view = { id, group, level, rank, type = "float"|"int"|"bool", label, desc, min, max, step, format,
--          get = fn() -> value, set = fn(value), default = fn() -> value }
-- Views are never part of S.defs / S.order / S.values / Save / ResetAll and are shown in the standard view only.
local VIEWS = { order = {}, byId = {} }
M.views = VIEWS

-- standard view target of a key or view: subcategory group id + sort key { nominal group, rank, tie-break } or nil
local function standardTarget(level, group, rank, tie)
  local g
  if level == "player" or level == "advanced" then g = group else return nil end
  return g, { GROUP_INDEX[group] or GROUP_INDEX.advanced, rank or 0, tie or 0 }
end
local function settingTarget(key)
  local def = S.defs[key]
  if not def or not def.section or def.hidden then return nil end -- hidden: in no view (Get / Set / Save unaffected)
  if M.ui.view == "all" then
    local sec = S.sections[def.section]
    if sec and sec.built then return TAB .. "/" .. sec.id, { 0, orderIndex[key] or 0, 0 }, sec end
    return nil
  end
  local level, group, rank = M.MenuPlacement(key)
  local g, sk = standardTarget(level, group, rank, orderIndex[key])
  if not g then return nil end
  return TAB .. "/g_" .. g, sk
end
local function viewTarget(v)
  if M.ui.view ~= "standard" then return nil end
  local g, sk = standardTarget(v.level, v.group, v.rank, 100000 + (v.index or 0))
  if not g then return nil end
  return TAB .. "/g_" .. g, sk
end
local function before(a, b)
  for i = 1, 3 do if a[i] ~= b[i] then return a[i] < b[i] end end
  return false
end
local function groupLabel(id)
  for _, g in ipairs(M.MENU_GROUPS) do if g.id == id then return g.label end end
  return id
end

-- appends the optional index argument of a Native Settings add* call only when it is set
local function withIndex(idx, ...)
  if idx == nil then return ... end
  local a, n = { ... }, select("#", ...)
  a[n + 1] = idx
  return unpack(a, 1, n + 1)
end

-- subcategory at path (created when missing): standard groups by M.MENU_GROUPS order, legacy sections before "Menu"
local function ensureSub(path, label, order)
  local g = M.ui.groups[path]
  if g then return g end
  local ns = M.ui.ns
  local pos = #M.ui.tabOrder + 1
  for i, p in ipairs(M.ui.tabOrder) do
    if (M.ui.groups[p].order or 0) > order then pos = i; break end
  end
  if not (ns.pathExists and ns.pathExists(path)) then
    ns.addSubcategory(withIndex(pos <= #M.ui.tabOrder and pos or nil, path, label))
  end
  g = { path = path, order = order, entries = {} }
  M.ui.groups[path] = g
  table.insert(M.ui.tabOrder, pos, path)
  return g
end
local function ensureGroup(id) return ensureSub(TAB .. "/g_" .. id, groupLabel(id), 1000 + (GROUP_INDEX[id] or 0)) end
-- index of a new entry in its subcategory (by sort key), nil = append
local function slot(g, sk)
  local pos = #g.entries + 1
  for i, e in ipairs(g.entries) do
    if before(sk, e.sk) then pos = i; break end
  end
  return pos, (pos <= #g.entries) and pos or nil
end

local function addSettingWidget(path, def, label, idx)
  local ns = M.ui.ns
  local key = def.key
  label = label or def.label
  local w
  if def.type == "bool" then
    w = ns.addSwitch(withIndex(idx, path, label, def.desc or "", S.values[key], def.default, function(v)
      if not S.uiSync then M.Set(key, v) end
    end))
  elseif def.type == "float" then
    w = ns.addRangeFloat(withIndex(idx, path, label, def.desc or "", def.min, def.max, def.step or 0.1, def.format or "%.2f",
      S.values[key], def.default, function(v) if not S.uiSync then M.Set(key, v) end end))
  elseif def.type == "int" then
    w = ns.addRangeInt(withIndex(idx, path, label, def.desc or "", def.min, def.max, def.step or 1, S.values[key], def.default,
      function(v) if not S.uiSync then M.Set(key, v) end end))
  elseif def.type == "choice" then
    local labels, cur, dflt = {}, 1, 1
    for i, c in ipairs(def.choices) do
      labels[i] = c[2]
      if c[1] == S.values[key] then cur = i end
      if c[1] == def.default then dflt = i end
    end
    w = ns.addSelectorString(withIndex(idx, path, label, def.desc or "", labels, cur, dflt, function(i)
      if not S.uiSync and def.choices[i] then M.Set(key, def.choices[i][1]) end
    end))
  end
  S.widgets[key] = w
  return w
end

local function addViewWidget(path, v, idx)
  local ns = M.ui.ns
  local okG, cur = pcall(v.get)
  if not okG or cur == nil then error("view " .. v.id .. ": no value (" .. tostring(cur) .. ")") end
  local dflt = cur
  if type(v.default) == "function" then
    local okD, d = pcall(v.default)
    if okD and d ~= nil then dflt = d end
  end
  local function cb(x)
    if S.uiSync then return end
    local ok, err = pcall(v.set, x)
    if not ok then M.Log("view " .. v.id .. " set FAILED: " .. tostring(err)) end
  end
  local w
  if v.type == "bool" then
    w = ns.addSwitch(withIndex(idx, path, v.label, v.desc or "", cur == true, dflt == true, cb))
  elseif v.type == "float" then
    w = ns.addRangeFloat(withIndex(idx, path, v.label, v.desc or "", v.min, v.max, v.step or 0.1, v.format or "%.2f", cur, dflt, cb))
  else
    w = ns.addRangeInt(withIndex(idx, path, v.label, v.desc or "", v.min, v.max, v.step or 1, cur, dflt, cb))
  end
  M.ui.viewWidgets[v.id] = w
  return w
end

-- one setting widget at its place in the current view (no-op when hidden there or already built)
local function placeSetting(key)
  if S.widgets[key] then return end
  local def = S.defs[key]
  local path, sk = settingTarget(key)
  if not path then return end
  local g
  if M.ui.view == "all" then
    g = M.ui.groups[path]
    if not g then return end
  else
    local _, group = M.MenuPlacement(key)
    g = ensureGroup(group)
  end
  local l = M.MENU_LAYOUT[key]
  local label = (M.ui.view == "standard" and l and l.label) or nil
  local pos, idx = slot(g, sk)
  local ok, err = pcall(addSettingWidget, path, def, label, idx)
  if ok and S.widgets[key] ~= nil then
    table.insert(g.entries, pos, { sk = sk, key = key })
  else
    S.widgets[key] = nil
    M.Log("widget " .. key .. " FAILED: " .. tostring(ok and "no widget" or err))
  end
end
local function placeView(v)
  if M.ui.viewWidgets[v.id] then return end
  local path, sk = viewTarget(v)
  if not path then return end
  local g = ensureGroup(v.group)
  local pos, idx = slot(g, sk)
  local ok, err = pcall(addViewWidget, path, v, idx)
  if ok and M.ui.viewWidgets[v.id] ~= nil then
    table.insert(g.entries, pos, { sk = sk, view = v.id })
  else
    M.ui.viewWidgets[v.id] = nil
    M.Log("widget " .. v.id .. " FAILED: " .. tostring(ok and "no widget" or err))
  end
end

-- legacy section subcategory (view "all"), kept for callers of the previous builder
function M.BuildSection(sec)
  -- a section whose keys are all hidden has no subcategory (no empty header); a visible key registered later builds it
  local visible = false
  for _, key in ipairs(sec.keys) do
    if not (S.defs[key] and S.defs[key].hidden) then visible = true; break end
  end
  if not visible and #sec.keys > 0 then return end
  ensureSub(TAB .. "/" .. sec.id, sec.label, 0)
  sec.built = true
  for _, key in ipairs(sec.keys) do placeSetting(key) end
end
function M.BuildWidget(sec, def)
  if not sec.built then M.BuildSection(sec) return end
  placeSetting(def.key)
end

local function menuGroup()
  local ns = M.ui.ns
  local g = ensureGroup("menu")
  if not ns.addButton then return end
  local all = M.ui.view == "all"
  local ok, err = pcall(ns.addButton, g.path, all and "Show standard options" or "Show all options (this session)",
    all and "Returns to the grouped list of the main options. Applies when the UTPP tab is opened again."
      or "Shows every UTPP option, including technical and diagnostic ones, under their original sections. Applies when "
      .. "the UTPP tab is opened again; the next game start shows the standard options. Changes no setting.",
    all and "Standard" or "Show all", 45, function() M.SetMenuView(((M.ui.pendingView or M.ui.view) == "all") and "standard" or "all") end)
  if not ok then M.Log("menu view button FAILED: " .. tostring(err)) end
  ok, err = pcall(ns.addButton, g.path, "Reset all UTPP settings", "Restores the default of every UTPP option.",
    "Reset", 45, function() M.ResetAll() end)
  if not ok then M.Log("reset button FAILED: " .. tostring(err)) end
end

local function buildView()
  M.ui.tabOrder, M.ui.groups, M.ui.viewWidgets = {}, {}, {}
  if M.ui.view == "all" then
    for _, id in ipairs(S.sectionOrder) do M.BuildSection(S.sections[id]) end
  else
    -- presentation copy, sorted per group; S.order and sec.keys stay as registered
    local items = {}
    for _, key in ipairs(S.order) do
      local path, sk = settingTarget(key)
      if path then items[#items + 1] = { path = path, sk = sk, key = key } end
    end
    for _, id in ipairs(VIEWS.order) do
      local path, sk = viewTarget(VIEWS.byId[id])
      if path then items[#items + 1] = { path = path, sk = sk, view = id } end
    end
    for _, grp in ipairs(M.MENU_GROUPS) do
      local list = {}
      for _, it in ipairs(items) do if it.path == TAB .. "/g_" .. grp.id then list[#list + 1] = it end end
      table.sort(list, function(a, b) return before(a.sk, b.sk) end)
      for _, it in ipairs(list) do
        if it.key then placeSetting(it.key) else placeView(VIEWS.byId[it.view]) end
      end
    end
  end
  menuGroup()
end

-- removes every UTPP subcategory of the current view (Native Settings removeSubcategory) and forgets its widgets
local function teardown()
  local ns = M.ui.ns
  for i = #M.ui.tabOrder, 1, -1 do
    local ok, err = pcall(ns.removeSubcategory, M.ui.tabOrder[i])
    if not ok then M.Log("remove " .. M.ui.tabOrder[i] .. " FAILED: " .. tostring(err)) end
  end
  for k in pairs(S.widgets) do S.widgets[k] = nil end
  for _, sec in pairs(S.sections) do sec.built = false end
  M.ui.tabOrder, M.ui.groups, M.ui.viewWidgets = {}, {}, {}
end

-- late registration (Register / RegisterSection after the tab was built): same visibility rule as the first build
function M.LateWidget(def)
  if not M.ui.ns or def.hidden then return end
  local sec = S.sections[def.section]
  -- "all" view: the section of a visible key that was skipped because all its keys were hidden is built now
  local ok, err = pcall(function()
    if M.ui.view == "all" and sec and not sec.built then M.BuildSection(sec) else placeSetting(def.key) end
  end)
  if not ok then M.Log("widget " .. def.key .. " FAILED: " .. tostring(err)) end
end
function M.LateSection(sec)
  if not M.ui.ns or M.ui.view ~= "all" then return end
  local ok, err = pcall(M.BuildSection, sec)
  if not ok then M.Log("section " .. sec.id .. " FAILED: " .. tostring(err)) end
end

function M.RegisterView(v)
  if type(v) ~= "table" or type(v.id) ~= "string" or type(v.get) ~= "function" or type(v.set) ~= "function" then
    return nil, "utpp_core.RegisterView: id, get and set are required"
  end
  if v.type ~= "float" and v.type ~= "int" and v.type ~= "bool" then return nil, "utpp_core.RegisterView: bad type " .. tostring(v.type) end
  if (v.type == "float" or v.type == "int") and (type(v.min) ~= "number" or type(v.max) ~= "number") then
    return nil, "utpp_core.RegisterView: min and max are required for " .. v.id
  end
  if not M.MENU_LEVELS[v.level] then v.level = "advanced" end
  if not GROUP_INDEX[v.group] or v.group == "menu" then v.group = "advanced" end
  v.rank = tonumber(v.rank) or 0
  local old = VIEWS.byId[v.id]
  if old then v.index = old.index else
    VIEWS.order[#VIEWS.order + 1] = v.id
    v.index = #VIEWS.order
  end
  VIEWS.byId[v.id] = v
  if M.ui.built and M.ui.ns then
    local ok, err = pcall(placeView, v)
    if not ok then M.Log("widget " .. v.id .. " FAILED: " .. tostring(err)) end
  end
  return true
end
-- re-reads view.get() into the built view widgets whose id equals idOrPrefix or starts with it; never calls Set / Save
function M.RefreshView(idOrPrefix)
  local ns = M.ui.ns
  if type(idOrPrefix) ~= "string" or not (M.ui.built and ns and ns.setOption) then return 0 end
  local n = 0
  for _, id in ipairs(VIEWS.order) do
    local w = M.ui.viewWidgets[id]
    if w and (id == idOrPrefix or string.sub(id, 1, #idOrPrefix) == idOrPrefix) then
      local ok, val = pcall(VIEWS.byId[id].get)
      if ok and val ~= nil then
        local prev = S.uiSync
        S.uiSync = true
        pcall(ns.setOption, w, val)
        S.uiSync = prev
        n = n + 1
      end
    end
  end
  return n
end

-- view "standard" | "all" for this session (not saved). Before the tab exists it applies at once; afterwards the tab is
-- rebuilt on the first onUpdate after the UTPP tab was closed (or at the next BuildNativeSettings when UTPP did not create the tab).
function M.SetMenuView(view)
  if view ~= "standard" and view ~= "all" then return nil, "unknown menu view " .. tostring(view) .. " (standard | all)" end
  if not M.ui.built then
    M.ui.view, M.ui.pendingView = view, nil
    return view
  end
  M.ui.pendingView = (view ~= M.ui.view) and view or nil
  M.Log("menu view " .. view .. (M.ui.pendingView and " requested: applies when the UTPP tab is opened again" or " (current)"))
  return view
end
function M.MenuView() return M.ui.view, M.ui.pendingView end
-- Native Settings calls the tab-closed callback from RequestClose while nativeSettings.currentTab is still the UTPP tab (and
-- again when the player changes the tab). Rebuilding there takes the library's "tab is open" branch for every widget and queues
-- scroll restores that fail once the controller is gone. So the callback only marks the request; onUpdate (MenuTick) applies it
-- as soon as the UTPP tab is no longer the current one.
function M.OnTabClosed()
  if M.ui.pendingView then M.ui.applyWhenClosed = true end
end
function M.MenuTick()
  if not M.ui.applyWhenClosed then return false end
  if not M.ui.pendingView then M.ui.applyWhenClosed = false; return false end
  local ns = M.ui.ns
  if ns and ns.currentTab == string.sub(TAB, 2) then return false end -- still the current tab: try again next frame
  M.ui.applyWhenClosed = false
  return M.ApplyPendingView()
end
function M.ApplyPendingView()
  local want = M.ui.pendingView
  M.ui.pendingView = nil
  if not want or want == M.ui.view then return false end
  if not M.ui.built then M.ui.view = want; return true end
  local ns = M.ui.ns
  if not (ns and type(ns.removeSubcategory) == "function") then
    M.Log("menu view " .. want .. " FAILED: Native Settings without removeSubcategory - view stays " .. M.ui.view)
    return false
  end
  local ok, err = pcall(function()
    teardown()
    M.ui.view = want
    buildView()
  end)
  M.Log("menu view " .. want .. ": " .. (ok and "ok" or ("FAILED " .. tostring(err))))
  return ok
end

function M.BuildNativeSettings(ns)
  if type(ns) ~= "table" or type(ns.addTab) ~= "function" then
    M.Log("Native Settings not found - console: GetMod(\"utpp_core\").Set(key, value), keys in utpp_settings.txt")
    return false
  end
  M.ui.ns = ns
  local ok, err = pcall(function()
    if not (ns.pathExists and ns.pathExists(TAB)) then
      ns.addTab(TAB, "UTPP", function() pcall(M.OnTabClosed) end)
    end
    if M.ui.built then
      if M.ui.pendingView then M.ApplyPendingView() end
    else
      M.ui.built = true
      buildView()
    end
    if ns.registerRestoreDefaultsCallback then
      ns.registerRestoreDefaultsCallback(TAB, true, function() M.ResetAll() end)
    end
  end)
  M.Log("Native Settings tab UTPP: " .. (ok and "ok" or ("FAILED " .. tostring(err))))
  return ok
end

-- ================================================================ Mod Settings bridge
-- With the Mod Settings framework installed the UTPP options appear in the game's Mod Settings screen (entry "Ultimate
-- Third Person") and the Native Settings tab is not created (onInit); without it (or without the UTPP bridge script) the tab is built as before.
-- The page is r6/scripts/UTPP/utpp_modsettings.reds, generated from this registry by the settings generator
-- (the options of the standard view; field = M.ModSettingsField(key)). This registry stays the only place that applies a
-- value; Mod Settings keeps its own copy in its user.ini. Calls: utpp_modsettings_bridge.reds (Game.UTPPModSettings*),
-- they work in the main menu too.
--   * sync (M.ModSettingsSync), at the first onUpdate tick with Mod Settings present and whenever its screen closes: per
--     option, CET value, Mod Settings value and the last synced value (utpp_modsettings_sync.txt in this mod folder, the
--     baseline) decide: no baseline for the option (first run, new option) -> the CET value is pushed (existing CET-saved
--     values are kept and shown in Mod Settings); only Mod Settings changed -> imported with Set; only CET changed -> pushed;
--     both changed -> Mod Settings wins (the player's latest menu edit).
--   * every Set from elsewhere (console, profile editor, ResetAll) marks the key dirty; dirty options shown in
--     Mod Settings are pushed (at most once per second, never while its screen is open or holds unconfirmed edits, because
--     Mod Settings accepts changes globally).
--   * choices are enum values 0..n-1 in choice order.
--   * detection: the bridge is checked at onInit and, until confirmed, again every MS_DETECT_RETRY seconds from onUpdate
--     (game script calls can fail at onInit). Not confirmed at onInit -> the Native Settings tab is built as the fallback
--     and removed as soon as Mod Settings is confirmed; the bridge compiled without the framework ("stub") = final.
-- Console: print(GetMod("utpp_core").ModSettingsStatus())
local MS = M.msync
MS.state = "unknown"   -- "unknown" (not checked) | "pending" (not confirmed, checked again) | "ready" (Mod Settings lists
                       -- the UTPP options) | "absent" (bridge compiled without Mod Settings)
MS.api = nil           -- offline tests: { variant, info, available, menuOpen, snapshot, push(data) } instead of the Game calls
MS.detectAt, MS.lastReason, MS.variant, MS.info, MS.revision, MS.msSeen = 0, nil, nil, nil, nil, {}
MS.base = nil          -- baseline: key -> CET-encoded value of the last sync (nil = file not read yet)
MS.fields = nil        -- last Mod Settings snapshot: field -> text
MS.reconciled, MS.menuWasOpen, MS.acc, MS.clock, MS.pushAt = false, false, 0, 0, 0
MS.stats = { syncs = 0, imported = 0, pushed = 0, pushCalls = 0, deferred = 0, errors = 0 }
M.MS_BASE_FILE = "utpp_modsettings_sync.txt"
M.MS_TICK = 0.25          -- s between checks of the Mod Settings screen
M.MS_PUSH_INTERVAL = 1.0  -- s between pushes (and retries while Mod Settings holds unconfirmed edits)
M.MS_DETECT_RETRY = 1.0   -- s between checks while Mod Settings is not confirmed

-- Mod Settings field (redscript identifier and user.ini key) of a setting key; gen_modsettings.py uses the same rule
function M.ModSettingsField(key) return (string.gsub(key, "[^%w]", "_")) end

local function msApi()
  if MS.api then return MS.api end
  if Game == nil then return nil end
  return { variant = function() return Game.UTPPModSettingsBridgeVariant() end,
           info = function() return Game.UTPPModSettingsBridgeInfo() end,
           available = function() return Game.UTPPModSettingsAvailable() end,
           menuOpen = function() return Game.UTPPModSettingsMenuOpen() end,
           snapshot = function() return Game.UTPPModSettingsSnapshot() end,
           revision = function() return Game.UTPPModSettingsRevision() end,
           push = function(data) return Game.UTPPModSettingsPush(data) end }
end

-- CET value -> Mod Settings text (nil = cannot be shown)
function M.ModSettingsEncode(def, v)
  local t = def.type
  if t == "bool" then return v and "true" or "false"
  elseif t == "float" then return string.format("%.6f", v)
  elseif t == "int" then return string.format("%d", v)
  elseif t == "choice" then
    for i, c in ipairs(def.choices or {}) do if c[1] == v then return tostring(i - 1) end end
    return nil
  end
  return nil
end
-- Mod Settings text -> CET value (validated; nil = not a valid value)
function M.ModSettingsDecode(def, s)
  if type(s) ~= "string" then return nil end
  local t = def.type
  if t == "bool" then
    if s == "true" then return true elseif s == "false" then return false end
    return nil
  elseif t == "float" or t == "int" then
    local n = tonumber(s)
    if n == nil then return nil end
    return validate(def, n)
  elseif t == "choice" then
    local n = tonumber(s)
    local c = n and n == math.floor(n) and (def.choices or {})[n + 1]
    return c and c[1] or nil
  end
  return nil
end
-- two CET values look the same in Mod Settings (floats: float32 and 6 decimals)
function M.ModSettingsSame(def, a, b)
  if a == nil or b == nil then return a == b end
  if def.type == "float" then return math.abs(a - b) <= 1e-4 * math.max(1, math.abs(a)) end
  return a == b
end

function M.ParseModSettingsSnapshot(s)
  local out = {}
  for line in string.gmatch(type(s) == "string" and s or "", "[^\n]+") do
    local f, v = string.match(line, "^([%w_]+)=(.-)%s*$")
    if f then out[f] = v end
  end
  return out
end

local function msLoadBase()
  if MS.base then return MS.base end
  local f = io.open(M.MS_BASE_FILE, "r")
  if not f then return nil end
  local base = {}
  for line in f:lines() do
    local k, v = string.match(line, "^%s*([%w_%.]+)%s*=%s*(.-)%s*$")
    -- retired and migrated settings (RETIRED_KEYS, MIGRATED_KEYS) are not kept, so the next msSaveBase drops them from the file
    if k and not string.match(line, "^%s*#") and not RETIRED_KEYS[k] and not MIGRATED_KEYS[k] then base[k] = v end
  end
  f:close()
  MS.base = base
  return base
end
local function msSaveBase()
  local base = MS.base or {}
  local keys = {}
  for k in pairs(base) do keys[#keys + 1] = k end
  table.sort(keys)
  local f = io.open(M.MS_BASE_FILE, "w")
  if not f then MS.stats.errors = MS.stats.errors + 1; return false end
  f:write("# UTPP: values last synced with Mod Settings (written by utpp_core; delete the file to push the CET values again)\n")
  for _, k in ipairs(keys) do f:write(k .. "=" .. base[k] .. "\n") end
  f:close()
  return true
end

-- baseline text of a CET value (floats with full precision; the settings file keeps 4 significant digits)
local function baseEncode(def, v)
  if def.type == "float" then return string.format("%.9g", v) end
  return encode(def, v)
end

-- options shown in Mod Settings (registered keys whose field is in the snapshot), registration order
local function msPairs(fields)
  local list = {}
  for _, key in ipairs(S.order) do
    local raw = fields[M.ModSettingsField(key)]
    if raw ~= nil then list[#list + 1] = { key = key, def = S.defs[key], raw = raw } end
  end
  return list
end

-- reconciles CET with a Mod Settings snapshot (field -> text); returns imported, toPush
function M.ModSettingsSync(fields, why)
  local base = msLoadBase()
  local firstRun = base == nil
  base = base or {}
  local pairsList = msPairs(fields)
  local imported = 0
  MS.importing = true
  local ok, err = pcall(function()
    for _, p in ipairs(pairsList) do
      local def, key = p.def, p.key
      local ms, cet = M.ModSettingsDecode(def, p.raw), S.values[key]
      p.ms = ms
      if ms ~= nil and not M.ModSettingsSame(def, ms, cet) then
        local b = nil -- (not "x and validate(...) or nil": a false baseline of a switch would read as no baseline)
        if base[p.key] ~= nil then b = validate(def, base[p.key]) end
        -- Mod Settings changed since the last sync (or, without a baseline entry, since the last snapshot of this
        -- session, e.g. while its values cannot be written): it wins
        local msChanged
        if b ~= nil then msChanged = not M.ModSettingsSame(def, ms, b)
        else msChanged = MS.msSeen[key] ~= nil and MS.msSeen[key] ~= p.raw end
        if msChanged then
          M.Set(key, ms, { noSave = true })
          imported = imported + 1
        end
      end
    end
  end)
  MS.importing = false
  for _, p in ipairs(pairsList) do MS.msSeen[p.key] = p.raw end
  if not ok then
    MS.stats.errors = MS.stats.errors + 1
    M.Log("Mod Settings sync (" .. tostring(why) .. ") FAILED: " .. tostring(err))
  end
  if imported > 0 then pcall(M.Save) end
  -- after the imports (and their apply callbacks): every option that still differs is pushed, the others are in sync
  local toPush = 0
  for _, p in ipairs(pairsList) do
    local def, key = p.def, p.key
    if p.ms == nil or not M.ModSettingsSame(def, p.ms, S.values[key]) then
      MS.dirty[key] = true
      toPush = toPush + 1
    else
      MS.dirty[key] = nil
      base[key] = baseEncode(def, S.values[key])
    end
  end
  MS.base = base
  msSaveBase()
  MS.stats.syncs = MS.stats.syncs + 1
  MS.stats.imported = MS.stats.imported + imported
  M.Log(string.format("Mod Settings sync (%s): %d options, %d imported, %d to push%s", tostring(why), #pairsList, imported,
    toPush, firstRun and " (first run: CET values kept)" or ""))
  return imported, toPush
end

-- pushes the dirty options that Mod Settings shows; returns the number written, -2 = deferred, nil = nothing / error
function M.ModSettingsPush()
  local fields, api = MS.fields, msApi()
  if not fields or not api then return nil end
  local lines, sent = {}, {}
  for key in pairs(MS.dirty) do
    local def = S.defs[key]
    local f = def and M.ModSettingsField(key)
    local enc = f and fields[f] ~= nil and M.ModSettingsEncode(def, S.values[key]) or nil
    if enc then
      lines[#lines + 1] = f .. "=" .. enc
      sent[#sent + 1] = { key = key, field = f, enc = enc }
    else
      MS.dirty[key] = nil -- not shown in Mod Settings (developer / diagnostic option, unknown key)
    end
  end
  if #lines == 0 then return nil end
  table.sort(lines)
  MS.stats.pushCalls = MS.stats.pushCalls + 1
  local ok, n = pcall(api.push, table.concat(lines, "\n") .. "\n")
  if not ok or type(n) ~= "number" then
    MS.stats.errors = MS.stats.errors + 1
    M.LogOnce("msync.push", "Mod Settings push FAILED: " .. tostring(n))
    return nil
  end
  if n == -2 then
    MS.stats.deferred = MS.stats.deferred + 1
    M.LogOnce("msync.deferred", "Mod Settings holds unconfirmed changes: UTPP values are pushed once they are applied or discarded")
    return n
  end
  if n == -3 then
    MS.stats.errors = MS.stats.errors + 1
    MS.pushAt = MS.clock + 10 * M.MS_PUSH_INTERVAL
    M.LogOnce("msync.noConfigVars", "Mod Settings lists no UTPP ConfigVars: CET changes cannot be shown there (Mod Settings "
      .. "edits still apply); retrying every " .. (10 * M.MS_PUSH_INTERVAL) .. " s")
    return n
  end
  if n < 0 then return n end
  MS.base = MS.base or {}
  for _, s in ipairs(sent) do
    MS.dirty[s.key] = nil
    fields[s.field] = s.enc
    MS.base[s.key] = baseEncode(S.defs[s.key], S.values[s.key])
  end
  msSaveBase()
  MS.stats.pushed = MS.stats.pushed + n
  M.Log(string.format("Mod Settings push: %d options sent, %d changed", #sent, n))
  return n
end

-- removes the UTPP Native Settings tab built before Mod Settings was confirmed. Native Settings has no removeTab: the
-- subcategories go through removeSubcategory, then the empty tab entry is dropped from its data table and its tab page
-- cache is reset (Native Settings rebuilds it when the menu opens). While the UTPP tab is open it waits (false).
function M.RemoveNativeSettingsTab()
  local ns = M.ui.ns
  if not (M.ui.built and ns) then return true end
  local id = string.sub(TAB, 2)
  if ns.currentTab == id then return false end
  local ok, err = pcall(function()
    teardown()
    if type(ns.data) == "table" then ns.data[id] = nil end
    if ns.tabSizeCache ~= nil then ns.tabSizeCache = nil end
  end)
  M.ui.built, M.ui.ns, M.ui.pendingView, M.ui.applyWhenClosed = false, nil, nil, false
  M.Log("Native Settings tab UTPP removed (Mod Settings confirmed): " .. (ok and "ok" or ("FAILED " .. tostring(err))))
  return true
end

local function msInfo(api)
  if not api.info then return "no info" end
  local ok, info = pcall(api.info)
  return ok and tostring(info) or ("info FAILED: " .. tostring(info))
end

-- true when the framework is compiled into the bridge and lists the UTPP mod (static data: works in the main menu, no
-- session needed). Not confirmed -> state "pending" (checked again by ModSettingsTick); the bridge variant "stub"
-- (compiled without Mod Settings) -> "absent", final.
function M.ModSettingsPresent()
  if MS.state == "ready" then return true elseif MS.state == "absent" then return false end
  local api = msApi()
  if not api then return false end
  local reason = nil
  if api.variant then
    local okV, variant = pcall(api.variant)
    MS.variant = okV and tostring(variant) or ("FAILED: " .. tostring(variant))
    if okV and variant == "stub" then
      MS.state, MS.info = "absent", msInfo(api)
      M.Log("Mod Settings not installed (UTPP bridge compiled without it): UTPP options stay in Native Settings")
      return false
    end
  end
  local ok, avail = pcall(api.available)
  if not ok then
    reason = "available() FAILED: " .. tostring(avail)
  elseif avail ~= true then
    reason = "available() = " .. tostring(avail)
  end
  MS.info = msInfo(api)
  if reason == nil then
    MS.state, MS.lastReason = "ready", nil
    M.Log("Mod Settings found (" .. MS.info .. "): UTPP options are in the Mod Settings screen")
    return true
  end
  MS.state = "pending"
  if reason ~= MS.lastReason then
    MS.lastReason = reason
    MS.stats.errors = MS.stats.errors + 1
    M.Log("Mod Settings not confirmed (" .. reason .. "; variant " .. tostring(MS.variant) .. "; " .. MS.info
      .. "): Native Settings stays the menu, checking again")
  end
  return false
end

-- onUpdate (also in menus): confirm Mod Settings (retried), drop the fallback tab, read the values once a session
-- exists (listener instance), sync at the first read, after the Mod Settings screen closed and on every change
-- notification (revision), push dirty options
function M.ModSettingsTick(dt)
  if MS.state == "absent" then return false end
  MS.acc = MS.acc + math.max(dt or 0, 0)
  if MS.acc < M.MS_TICK then return false end
  MS.clock = MS.clock + MS.acc
  MS.acc = 0
  local api = msApi()
  if not api then return false end
  if MS.state ~= "ready" then
    if MS.clock < MS.detectAt then return false end
    MS.detectAt = MS.clock + M.MS_DETECT_RETRY
    if not M.ModSettingsPresent() then return false end
  end
  if M.ui.built and not M.RemoveNativeSettingsTab() then return false end
  local okO, open = pcall(api.menuOpen)
  if not okO then MS.stats.errors = MS.stats.errors + 1; return false end
  if open == true then MS.menuWasOpen = true; return false end
  local rev = nil
  if api.revision then
    local okR, r = pcall(api.revision)
    if okR and type(r) == "number" then rev = r end
  end
  if rev ~= nil and rev < 0 then -- no game session yet: no listener instance to read
    MS.reconciled, MS.revision = false, nil
    return false
  end
  if not MS.reconciled or MS.menuWasOpen or (rev ~= nil and rev ~= MS.revision) then
    local okS, snap = pcall(api.snapshot)
    if not okS then
      MS.stats.errors = MS.stats.errors + 1
      M.LogOnce("msync.snapshot", "Mod Settings snapshot FAILED: " .. tostring(snap))
      return false
    end
    local fields = M.ParseModSettingsSnapshot(snap)
    if next(fields) == nil then return false end -- listener not registered yet
    local mapped = false
    for _, key in ipairs(S.order) do
      if fields[M.ModSettingsField(key)] ~= nil then mapped = true; break end
    end
    if not mapped then
      MS.stats.errors = MS.stats.errors + 1
      M.LogOnce("msync.unmapped", "Mod Settings snapshot has no option of this registry - not synced")
      return false
    end
    local why = (not MS.reconciled) and "start" or (MS.menuWasOpen and "menu closed" or "change")
    MS.menuWasOpen, MS.revision = false, rev
    MS.fields = fields
    M.ModSettingsSync(fields, why)
    MS.reconciled = true
    MS.pushAt = 0
  end
  if next(MS.dirty) ~= nil and MS.clock >= MS.pushAt then
    MS.pushAt = MS.clock + M.MS_PUSH_INTERVAL
    M.ModSettingsPush()
  end
  return true
end

function M.ModSettingsStatus()
  local nd = 0
  for _ in pairs(MS.dirty) do nd = nd + 1 end
  local st = MS.stats
  return string.format("Mod Settings bridge: state=%s variant=%s synced=%s menuOpen=%s dirty=%d syncs=%d imported=%d "
    .. "pushed=%d pushCalls=%d deferred=%d errors=%d nativeTab=%s info=[%s] last=[%s]", MS.state, tostring(MS.variant),
    tostring(MS.reconciled), tostring(MS.menuWasOpen), nd, st.syncs, st.imported, st.pushed, st.pushCalls, st.deferred,
    st.errors, tostring(M.ui.built), tostring(MS.info), tostring(MS.lastReason))
end

-- ================================================================ head look (pure decision)
local HL = {}
M.HeadLook = HL
HL.DEFAULTS = {
  enabled = true,
  whileWalking = true,
  walkSpeed = 2.4,        -- m/s: above -> released (running / sprinting)
  stillSpeed = 0.3,       -- m/s: limit when whileWalking is off
  engageDelay = 0.35,     -- s without a block before the look starts
  eyeHeight = 1.62,       -- m above the player origin
  lookDistance = 4.0,     -- m beyond the character along the camera ray
  maxYaw = 100,           -- deg from the body forward (target clamped to it)
  maxPitchUp = 35, maxPitchDown = 40,
  faceCamera = true,      -- camera in front looking at V -> look at the camera
  faceEnterDot = -0.3, faceExitDot = -0.1,
  lookAtLoot = true,
  retargetDist = 0.2,     -- m target change before a new request
  retargetInterval = 0.15,-- s between requests
}
-- deg: the resting framing looks this far down at V; camera pitches from -NEUTRAL_PITCH up to level leave the head level (no staring at
-- the floor while she stands), only the excess tilts the head (camera mode only). Not a setting: kept out of cfg (cfg.neutralPitch
-- overrides it for tests).
HL.NEUTRAL_PITCH = 15
function HL.New(cfg)
  local c = { cfg = copy(HL.DEFAULTS), state = "off", reason = "init", face = false, target = nil, lastApply = -1e9,
              freeSince = nil, applies = 0, clears = 0 }
  for k, v in pairs(cfg or {}) do c.cfg[k] = v end
  return c
end
-- obs: enabled, tpp, speed, aim, vehicle, workspot, scene, swimming, takedown, scanner, ragdoll, locoBlocked
function HL.Blocked(obs, cfg)
  if not obs.enabled then return "disabled" end
  if not obs.tpp then return "fpp" end
  if obs.aim then return "aim" end
  if obs.vehicle then return "vehicle" end
  if obs.workspot then return "workspot" end
  if obs.scene then return "scene" end
  if obs.swimming then return "swimming" end
  if obs.takedown then return "takedown" end
  if obs.scanner then return "scanner" end
  -- a ragdoll episode / car knockdown (Game.UTPPRagdollBusy): utpp_ragdoll.reds refuses Apply meanwhile and PSM LocomotionDetailed
  -- stays Stand, so locoBlocked does not see it; blocking here clears the state and re-engages cleanly after the get-up
  if obs.ragdoll then return "ragdoll" end
  if obs.locoBlocked then return "locomotion" end
  local limit = cfg.whileWalking and cfg.walkSpeed or cfg.stillSpeed
  if (obs.speed or 0) > limit then return "moving" end
  return nil
end
local function norm2(x, y) local l = math.sqrt(x * x + y * y); if l < 1e-6 then return 0, 0, 0 end return x / l, y / l, l end
-- obs.pos {x,y,z}, obs.fwd {x,y} body forward, obs.camPos, obs.camFwd {x,y,z}, obs.loot {x,y,z}|nil
function HL.Target(c, obs)
  local cfg = c.cfg
  local hx, hy, hz = obs.pos.x, obs.pos.y, obs.pos.z + cfg.eyeHeight
  local fx, fy = norm2(obs.fwd.x, obs.fwd.y)
  local tx, ty, tz, mode
  if cfg.lookAtLoot and obs.loot then
    tx, ty, tz, mode = obs.loot.x, obs.loot.y, obs.loot.z + 0.1, "loot"
    c.face = false
  else
    local cx, cy, cz = obs.camPos.x, obs.camPos.y, obs.camPos.z
    local vx, vy, vz = obs.camFwd.x, obs.camFwd.y, obs.camFwd.z
    local cfx, cfy = norm2(vx, vy)
    local facing = cfx * fx + cfy * fy                  -- camera looks along (+1) or against (-1) the body
    local ox, oy = norm2(cx - hx, cy - hy)
    local inFront = ox * fx + oy * fy                   -- camera position in front of the body (+1)
    if cfg.faceCamera then
      if not c.face and facing < cfg.faceEnterDot and inFront > 0.3 then c.face = true
      elseif c.face and (facing > cfg.faceExitDot or inFront < 0.1) then c.face = false end
    else
      c.face = false
    end
    if c.face then
      tx, ty, tz, mode = cx, cy, cz, "face"
    else
      -- the camera ray with its pitch reduced by the resting framing (dead band [-neutralPitch, 0]); the heading stays
      local d = math.sqrt((cx - hx) ^ 2 + (cy - hy) ^ 2 + (cz - hz) ^ 2) + cfg.lookDistance
      local hxn, hyn = cfx, cfy
      if hxn == 0 and hyn == 0 then hxn, hyn = fx, fy end
      local pitch = math.atan2(vz, math.sqrt(vx * vx + vy * vy))
      local neutral = math.rad(cfg.neutralPitch or HL.NEUTRAL_PITCH)
      local pe = pitch - clamp(pitch, -neutral, 0)
      tx, ty, tz, mode = cx + hxn * math.cos(pe) * d, cy + hyn * math.cos(pe) * d, cz + math.sin(pe) * d, "camera"
    end
  end
  -- clamp yaw / pitch relative to the body
  local raw = { x = tx, y = ty, z = tz } -- unclamped: the DLL layer clamps and fades it itself (it needs the true side of the target)
  local dx, dy, dz = tx - hx, ty - hy, tz - hz
  local flat = math.sqrt(dx * dx + dy * dy)
  if flat < 0.3 then dx, dy, flat = fx * 0.3, fy * 0.3, 0.3 end
  local yaw = math.atan2(fx * dy - fy * dx, fx * dx + fy * dy) -- signed, + = left (CCW)
  local lim = math.rad(cfg.maxYaw)
  local clamped = false
  if math.abs(yaw) > lim then
    yaw = (yaw > 0) and lim or -lim
    clamped = true
    local cs, sn = math.cos(yaw), math.sin(yaw)
    dx, dy = (fx * cs - fy * sn) * flat, (fx * sn + fy * cs) * flat
  end
  local pitch = math.atan2(dz, flat)
  local pc = clamp(pitch, -math.rad(cfg.maxPitchDown), math.rad(cfg.maxPitchUp))
  if pc ~= pitch then dz = math.tan(pc) * flat; clamped = true end
  return { x = hx + dx, y = hy + dy, z = hz + dz }, mode, clamped, raw
end
-- returns nil | { op = "apply", target } | { op = "clear" }
function HL.Step(c, t, obs)
  local why = HL.Blocked(obs, c.cfg)
  if why then
    c.freeSince = nil
    c.reason = why
    if c.state == "on" then
      c.state, c.target = "off", nil
      c.clears = c.clears + 1
      return { op = "clear", reason = why }
    end
    return nil
  end
  c.freeSince = c.freeSince or t
  if c.state == "off" then
    if t - c.freeSince < c.cfg.engageDelay then c.reason = "settling"; return nil end
    local target, mode = HL.Target(c, obs)
    c.state, c.target, c.mode, c.lastApply, c.reason = "on", target, mode, t, mode
    c.applies = c.applies + 1
    return { op = "apply", target = target, mode = mode }
  end
  local target, mode = HL.Target(c, obs)
  c.reason = mode
  local d = math.sqrt((target.x - c.target.x) ^ 2 + (target.y - c.target.y) ^ 2 + (target.z - c.target.z) ^ 2)
  if (d > c.cfg.retargetDist or mode ~= c.mode) and t - c.lastApply >= c.cfg.retargetInterval then
    c.target, c.mode, c.lastApply = target, mode, t
    c.applies = c.applies + 1
    return { op = "apply", target = target, mode = mode }
  end
  return nil
end
-- forced release (setting off, shutdown)
function HL.Release(c, why)
  c.freeSince = nil
  c.reason = why or "release"
  if c.state == "on" then
    c.state, c.target = "off", nil
    c.clears = c.clears + 1
    return { op = "clear", reason = c.reason }
  end
  return nil
end

-- ================================================================ loot view (pure)
local LA = {}
M.Loot = LA
LA.DEFAULTS = {
  radius = 1.8,          -- m horizontal from the player
  maxHeight = 1.6,       -- m |loot z - player z|
  coneDeg = 70,          -- deg between the camera forward and camera->loot (horizontal)
  angleWeight = 0.6,     -- score = distance + angleWeight * angle(rad)
  switchMargin = 0.25,   -- keep the current target unless another scores this much better
  zoom = 0.9,            -- m the camera moves closer at full weight
  blendIn = 0.35, blendOut = 0.5, hold = 0.5,  -- s
  trackRange = 25, maxTracked = 48, lookTtl = 1.0,
  runSpeed = 3.0,        -- m/s: above -> no assist
}
function LA.NewTracker() return { items = {}, n = 0 } end
-- layer activation (layer = interaction layer tag, "lookat" = crosshair poll with ttl)
function LA.Activate(tr, key, pos, layer, t, ttl)
  local it = tr.items[key]
  if not it then
    it = { key = key, layers = {}, pos = pos }
    tr.items[key] = it
    tr.n = tr.n + 1
  end
  it.pos, it.seen = pos or it.pos, t
  it.layers[layer or "?"] = ttl and (t + ttl) or true
  return it
end
function LA.Deactivate(tr, key, layer)
  local it = tr.items[key]
  if not it then return end
  it.layers[layer or "?"] = nil
  if next(it.layers) == nil then tr.items[key] = nil; tr.n = tr.n - 1 end
end
-- drops expired crosshair entries, far / vanished items (alive(key) -> pos|nil), and the farthest above maxTracked
function LA.Prune(tr, t, ppos, cfg, alive)
  local list = {}
  for key, it in pairs(tr.items) do
    for l, exp in pairs(it.layers) do if exp ~= true and t > exp then it.layers[l] = nil end end
    local keep = next(it.layers) ~= nil
    if keep and alive then
      local p = alive(key)
      if p == nil then keep = false else it.pos = p end
    end
    if keep and ppos and it.pos then
      local d = math.sqrt((it.pos.x - ppos.x) ^ 2 + (it.pos.y - ppos.y) ^ 2)
      it.dist = d
      if d > cfg.trackRange then keep = false end
    end
    if keep then list[#list + 1] = it else tr.items[key] = nil end
  end
  table.sort(list, function(a, b) return (a.dist or 0) < (b.dist or 0) end)
  for i = cfg.maxTracked + 1, #list do tr.items[list[i].key] = nil end
  local n = 0
  for _ in pairs(tr.items) do n = n + 1 end
  tr.n = n
end
function LA.Score(it, obs, cfg)
  local p, pp = it.pos, obs.pos
  if not p then return nil end
  local d = math.sqrt((p.x - pp.x) ^ 2 + (p.y - pp.y) ^ 2)
  if d > cfg.radius or math.abs(p.z - pp.z) > cfg.maxHeight then return nil end
  local fx, fy = norm2(obs.camFwd.x, obs.camFwd.y)
  local lx, ly, ll = norm2(p.x - obs.camPos.x, p.y - obs.camPos.y)
  local ang = 0
  if ll > 1e-3 and (fx ~= 0 or fy ~= 0) then ang = math.acos(clamp(fx * lx + fy * ly, -1, 1)) end
  if ang > math.rad(cfg.coneDeg) then return nil end
  return d + cfg.angleWeight * ang
end
function LA.Select(tr, obs, cfg, currentKey)
  local best, bestScore, curScore = nil, math.huge, nil
  for key, it in pairs(tr.items) do
    local s = LA.Score(it, obs, cfg)
    if s then
      if s < bestScore then best, bestScore = it, s end
      if key == currentKey then curScore = s end
    end
  end
  if curScore and best and best.key ~= currentKey and curScore <= bestScore + cfg.switchMargin then
    return tr.items[currentKey], curScore
  end
  return best, best and bestScore or nil
end
function LA.New(cfg)
  local c = { cfg = copy(LA.DEFAULTS), weight = 0, target = nil, lostAt = nil, reason = "init", tracker = LA.NewTracker() }
  for k, v in pairs(cfg or {}) do c.cfg[k] = v end
  return c
end
-- obs: enabled, tpp, blocked (aim/vehicle/scene ...), speed, pos, camPos, camFwd. Returns camera offset (m, >= 0).
function LA.Step(c, t, dt, obs)
  local cfg = c.cfg
  local sel = nil
  if not obs.enabled then c.reason = "disabled"
  elseif not obs.tpp then c.reason = "fpp"
  elseif obs.blocked then c.reason = tostring(obs.blocked)
  elseif (obs.speed or 0) > cfg.runSpeed then c.reason = "moving"
  else
    sel = LA.Select(c.tracker, obs, cfg, c.target and c.target.key or nil)
    c.reason = sel and "loot" or "none"
  end
  local hard = not obs.enabled or not obs.tpp or obs.blocked
  if sel then
    c.target, c.lostAt = sel, nil
  elseif c.target then
    c.lostAt = c.lostAt or t
    if hard or t - c.lostAt > cfg.hold then c.target = nil end
  end
  local want = c.target and 1 or 0
  local rate = (want > c.weight) and (1 / math.max(cfg.blendIn, 1e-3)) or (1 / math.max(cfg.blendOut, 1e-3))
  if hard then rate = 1 / 0.15 end
  if c.weight < want then c.weight = math.min(want, c.weight + rate * dt)
  else c.weight = math.max(want, c.weight - rate * dt) end
  return cfg.zoom * smoothstep(c.weight)
end
-- Rotates (0,1,0) by quaternion q {i,j,k,r}: local forward of the camera component.
function LA.QuatForward(q)
  local x, y, z, w = q.i, q.j, q.k, q.r
  return { x = 2 * (x * y - w * z), y = 1 - 2 * (x * x + z * z), z = 2 * (y * z + w * x) }
end
-- Additive camera offset that survives an owner rewriting the position every frame: if the component still holds what
-- we wrote last frame, the base stays our previous base; otherwise the owner wrote a fresh base.
function LA.ApplyOffset(st, cur, fwd, offset)
  local base
  if st.written and math.abs(cur.x - st.written.x) < 1e-4 and math.abs(cur.y - st.written.y) < 1e-4
     and math.abs(cur.z - st.written.z) < 1e-4 then
    base = st.base
  else
    base = { x = cur.x, y = cur.y, z = cur.z }
  end
  st.base = base
  local out = { x = base.x + fwd.x * offset, y = base.y + fwd.y * offset, z = base.z + fwd.z * offset }
  st.written = out
  return out, base
end

-- ================================================================ built-in options
M.RegisterSection("general", "General")
M.RegisterSection("headlook", "Head look")
M.RegisterSection("loot", "Loot view")

M.hl = HL.New()
M.la = LA.New()

M.Register({ key = "core.log", section = "general", type = "bool", default = true, label = "Core and camera log",
  desc = "Status lines of the UTPP CET mods in the UTPP red4ext log (red4ext/logs)." })

-- ================================================================ diagnostic logs (core.diagnosticLogs)
-- One list for reporting a problem: "off" (default) | "timed" ("On for 5 minutes": five minutes of GAMEPLAY - the timer does
-- not run in a menu, on the pause screen or without a player - then it switches itself to off, also in Mod Settings; after a
-- game restart it is always off) | "until" ("On until switched off": stays on after a restart).
-- On: core.log, mm.trace, mm.feedDump, camera.trace and perspective.log are set on through M.Set (their apply callbacks run)
-- and held on (Register / Set keep them true, also for modules that register later and for stored values); the session
-- diagnostics UTPPMotionSetReloadDiag, UTPPSetVehicleVaultLog and UTPPSetMMFeetDiag are switched on when the option is switched
-- during play (utpp_motion re-arms them for every new game session / player entity, also after a restart with the option on; that
-- needs mm.motionDriver on); the compact pose recording starts on the first gameplay frame (Game.UTPPPoseCaptureStart(keepMinutes):
-- 0 = the whole recording for "timed", 10 = only the last 10 minutes for "until").
-- Off (a switch during the session, by hand or when the time is up): each held switch goes back to its registered default
-- (not to the value it had before - core.log and perspective.log on, the others off), the session diagnostics are switched
-- off (the reload diagnostic follows mm.trace), the pose recording stops (Game.UTPPPoseCaptureStop() -> files, relative to
-- the game folder, separated by "|") and the instruction file is written (Game.UTPPDiagWriteReport(body) ->
-- red4ext\logs\utpp-report-<date>.txt, relative path or ""); a message on the screen (Game.UTPPShowMessage) tells the file name.
-- Every game call goes through M.diag.api (injected by the tests: setReloadDiag(on), setVaultLog(on), setFeetDiag(on),
-- poseStart(keepMinutes) -> bool, poseStop() -> string, report(body) -> string, notify(text, seconds)); without it the Game
-- natives are called in pcall: a missing native is one log line, the rest keeps working.
-- The continuous helper-bone CSV (Game.UTPPBodyPoseTraceContinuous, hundreds of MB per minute) is NOT part of this list: it
-- stays an explicit diagnostic native, called by hand from the CET console.
M.DIAG_TIMED_SECONDS = 300
M.DIAG_UNTIL_KEEP_MINUTES = 10
M.diag = { mode = "off", api = nil, calls = 0, errors = 0, remaining = 0, played = 0, elapsed = 0, poseDue = false,
  poseState = nil, files = {}, notice = nil, endReason = nil, reports = 0 }
local DIAG_NATIVES = {
  setReloadDiag = { "UTPPMotionSetReloadDiag", "reload diagnostic" },
  setVaultLog = { "UTPPSetVehicleVaultLog", "vehicle vault log" },
  setFeetDiag = { "UTPPSetMMFeetDiag", "feet diagnostic" },
  poseStart = { "UTPPPoseCaptureStart", "pose recording" },
  poseStop = { "UTPPPoseCaptureStop", "pose recording stop" },
  poseStatus = { "UTPPPoseCaptureStatus", "pose recording status" },
  report = { "UTPPDiagWriteReport", "diagnostic report" },
  notify = { "UTPPShowMessage", "screen message" },
}
-- true, result | false, error text | nil when there is no way to call it
local function diagCall(field, ...)
  local n = DIAG_NATIVES[field]
  local api, fn = M.diag.api, nil
  if type(api) == "table" then
    fn = api[field]
  elseif Game ~= nil then
    fn = function(...) return Game[n[1]](...) end
  end
  if type(fn) ~= "function" then return nil end
  local ok, res = pcall(fn, ...)
  M.diag.calls = M.diag.calls + 1
  if not ok then
    M.diag.errors = M.diag.errors + 1
    M.LogOnce("diagnosticLogs:" .. field, string.format("%s FAILED: %s (native missing?) - further failures not logged", n[2], tostring(res)))
    return false, res
  end
  return true, res
end

local function diagModeLabel(mode)
  if mode == "timed" then return "On for 5 minutes" elseif mode == "until" then return "On until switched off" end
  return "Off"
end

-- keys that the report always lists (with the value in force), besides every setting that differs from its default
local REPORT_KEYS = { "animation.pack", "camera.impl", "body.impl", "mm.enabled", "mm.motionDriver", "bodyturn.shooterMode", "perspective.impl",
  "melee.fullBody", "mm.aimAtCrosshair" }
-- body of the instruction file (the DLL adds the versions of the mod and the game and the list of the session files)
function M.DiagReportBody(info)
  local D = M.diag
  local out = {}
  out[#out + 1] = "Diagnostic logs: " .. diagModeLabel(info.mode)
  out[#out + 1] = string.format("Duration: %d s of gameplay (%d s with the logs on)", math.floor(info.played + 0.5), math.floor(info.elapsed + 0.5))
  out[#out + 1] = "Switched off: " .. tostring(info.reason)
  out[#out + 1] = "Animation pack in force: " .. tostring(M.pack.applied) .. " (setting: " .. tostring(S.values[M.PACK_KEY]) .. ")"
  if info.poseFiles and info.poseFiles ~= "" then out[#out + 1] = "Pose recording files: " .. info.poseFiles end
  if info.poseState == "unavailable" then out[#out + 1] = "Pose recording: not available" end
  if info.poseFailed then
    out[#out + 1] = "Pose recording: failed (disk full or folder not writable)"
  elseif info.poseRecorded and (info.poseFiles == nil or info.poseFiles == "") then
    out[#out + 1] = "Pose recording: no file was written"
  end
  out[#out + 1] = ""
  out[#out + 1] = "Settings that differ from the defaults:"
  local skip = {}
  for _, k in ipairs(M.DIAG_KEYS) do skip[k] = true end
  skip[M.DIAG_KEY] = true
  local n = 0
  for _, key in ipairs(S.order) do
    local def = S.defs[key]
    if not skip[key] and encode(def, S.values[key]) ~= encode(def, def.default) then
      out[#out + 1] = "  " .. key .. "=" .. encode(def, S.values[key])
      n = n + 1
    end
  end
  if n == 0 then out[#out + 1] = "  (none)" end
  out[#out + 1] = ""
  out[#out + 1] = "Key settings (value in force):"
  for _, key in ipairs(REPORT_KEYS) do
    if S.defs[key] then out[#out + 1] = "  " .. key .. "=" .. encode(S.defs[key], M.Effective(key)) end
  end
  return table.concat(out, "\n") .. "\n"
end

local function diagNotice(text)
  M.diag.notice = text
end
local function reportName(path)
  return (string.match(tostring(path), "[^\\/]+$")) or tostring(path)
end

local function diagSetSessionNatives(on)
  diagCall("setReloadDiag", on or M.Effective("mm.trace") == true)
  diagCall("setVaultLog", on)
  diagCall("setFeetDiag", on)
end

-- Starts the pose recording (first gameplay frame after the switch-on or a restart with the option stored on).
local function diagStartPose()
  local D = M.diag
  D.poseDue = false
  local keep = D.mode == "until" and M.DIAG_UNTIL_KEEP_MINUTES or 0
  local ok, res = diagCall("poseStart", keep)
  if ok and res ~= false then
    D.poseState = "recording"
    M.Log(string.format("pose recording started (%s)", keep == 0 and "whole session" or ("last " .. keep .. " minutes")))
  else
    D.poseState = "unavailable"
    M.LogOnce("diagnosticLogs:pose", "pose recording not started (" .. (ok == nil and "no native" or "refused") .. ") - the other logs keep working")
  end
end

-- Stops the pose recording; returns the file list text ("" when nothing was recorded).
local function diagStopPose()
  local D = M.diag
  D.poseDue = false
  if D.poseState ~= "recording" then return "" end
  D.poseState = nil
  D.poseRecorded = true
  local okS, status = diagCall("poseStatus")
  if okS and type(status) == "string" and string.find(status, "failed=1", 1, true) then D.poseFailed = true end
  local ok, res = diagCall("poseStop")
  if ok and type(res) == "string" then return res end
  return ""
end

-- Brings the diagnostic switches, the session natives and the pose recording in line with the list (mode = its effective
-- value). Only a change of the applied state acts: an initial off (every start without the option) touches nothing, so
-- individually stored diagnostic switches keep working. why: "setting" | "load" (the settings file was read: no game session
-- yet) | "time" (the five minutes are over).
function M.ApplyDiagnosticLogs(mode, why)
  mode = (mode == "timed" or mode == "until") and mode or "off"
  local D = M.diag
  local prev = D.mode
  if prev == mode then
    -- already on: only re-hold a switch that something wrote without Set (a second Load of the settings file)
    if mode ~= "off" then
      for _, key in ipairs(M.DIAG_KEYS) do
        if S.defs[key] and S.values[key] ~= true then M.Set(key, true, { noSave = true }) end
      end
    end
    return false
  end
  local wasOn = prev ~= "off"
  D.mode = mode
  if mode ~= "off" then
    D.remaining = (mode == "timed") and M.DIAG_TIMED_SECONDS or 0
    if not wasOn then
      D.played, D.elapsed, D.files, D.endReason, D.poseState = 0, 0, {}, nil, nil
      D.poseRecorded, D.poseFailed = false, false
      for _, key in ipairs(M.DIAG_KEYS) do
        if S.defs[key] then
          -- the player's own value is what Save persists while a mode holds the switch (see M.Save)
          S.stored[key] = encode(S.defs[key], S.values[key])
          M.Set(key, true, { noSave = true })
        end
      end
      -- the session natives only on a switch during play: when the settings file is read (CET loads the mods) no game
      -- session exists yet; utpp_motion arms them for every new session (its fresh-session sync)
      if why ~= "load" then diagSetSessionNatives(true) end
      D.poseDue = true
    else
      -- timed <-> until: the recording restarts with the window of the new mode (the files of the first part stay listed)
      local files = diagStopPose()
      if files ~= "" then D.files[#D.files + 1] = files end
      D.poseDue = true
    end
    diagNotice(mode == "timed" and "UTPP diagnostic logs are ON for 5 minutes of gameplay. Repeat the problem now."
      or "UTPP diagnostic logs are ON until you switch them off.")
    M.Log(string.format("diagnostic logs ON: %s (%s)", diagModeLabel(mode), tostring(why or "setting")))
    return true
  end
  -- off
  for _, key in ipairs(M.DIAG_KEYS) do
    local def = S.defs[key]
    if def then
      M.Set(key, def.default, { noSave = true })
    else
      S.stored[key] = nil -- module not loaded this session: it registers with its default
    end
  end
  if why ~= "load" then diagSetSessionNatives(false) end
  local files = diagStopPose()
  local reason = D.endReason or (why == "time" and "the five minutes were over" or "switched off in the menu")
  D.endReason = nil
  local list = {}
  for _, f in ipairs(D.files) do list[#list + 1] = f end
  if files ~= "" then list[#list + 1] = files end
  D.files = {}
  local okR, path = diagCall("report", M.DiagReportBody({ mode = prev, played = D.played, elapsed = D.elapsed, reason = reason,
    poseFiles = table.concat(list, "|"), poseState = D.poseState,
    poseRecorded = D.poseRecorded, poseFailed = D.poseFailed }))
  if okR and type(path) == "string" and path ~= "" then
    D.reports = D.reports + 1
    diagNotice("UTPP diagnostic logs are OFF. The files to send are listed in " .. reportName(path) .. " (folder red4ext\\logs).")
    M.Log("diagnostic logs OFF (" .. tostring(why or "setting") .. "): report " .. path)
  else
    diagNotice("UTPP diagnostic logs are OFF. The report file could not be written; send the files of red4ext\\logs.")
    M.Log("diagnostic logs OFF (" .. tostring(why or "setting") .. "): report file not written")
  end
  return true
end

-- Called when the settings file was read (Load): "On for 5 minutes" never survives a restart (the list shows Off, also in Mod
-- Settings: the key is marked for the next push); "On until switched off" is applied (held switches; the session natives and
-- the pose recording follow in the first gameplay frames).
function M.LoadDiagnosticLogs()
  local v = S.values[M.DIAG_KEY]
  if v == "timed" then
    S.values[M.DIAG_KEY] = "off"
    M.msync.dirty[M.DIAG_KEY] = true
    v = "off"
  end
  if v == "until" then M.ApplyDiagnosticLogs("until", "load") end
end

-- One step of the diagnostic timer and its messages (CET onUpdate). playing = a player exists and neither a menu nor the
-- pause screen is open: only then the five minutes run and a message is shown.
function M.DiagTick(dt, playing)
  local D = M.diag
  dt = (type(dt) == "number" and dt == dt and dt > 0 and dt < 5) and dt or 0
  if D.mode ~= "off" then
    D.elapsed = D.elapsed + dt
    if playing then
      D.played = D.played + dt
      if D.poseDue then diagStartPose() end
      if D.mode == "timed" then
        D.remaining = D.remaining - dt
        if D.remaining <= 0 then
          D.endReason = "the five minutes were over"
          M.Set(M.DIAG_KEY, "off") -- apply: ApplyDiagnosticLogs("off"); the list shows Off in Mod Settings after the next push
          if D.mode ~= "off" then M.ApplyDiagnosticLogs("off", "time") end
        end
      end
    end
  end
  if playing and D.notice then
    local text = D.notice
    D.notice = nil
    diagCall("notify", text, 8.0)
  end
end

M.Register({ key = M.DIAG_KEY, section = "general", type = "choice", default = "off",
  choices = { { "off", "Off" }, { "timed", "On for 5 minutes" }, { "until", "On until switched off" } },
  label = "Diagnostic logs",
  desc = "For reporting a problem. 'On for 5 minutes' switches every UTPP diagnostic log on for 5 minutes of gameplay, then "
    .. "off by itself: switch it on, repeat the problem in the game, then switch it off or wait. A file named "
    .. "utpp-report-(date).txt in red4ext\\logs then lists the files to zip and send to the mod author. 'On until switched off' "
    .. "keeps the logs on, also after a game restart; use it only for long test sessions.",
  apply = function(v) M.ApplyDiagnosticLogs(v, "setting") end })

-- ================================================================ animation pack (animation.pack)
-- "Ultimate Third Person Project" (utpp, default) = the mod's animations. "Game Animations" (game) = the game's own animations
-- on the third-person body: the camera, the full body, the head, the crosshair, the shoulder key and the corrections that the
-- third-person view needs stay; the mod's locomotion and everything that depends on it (weapon poses, aim offset, recoil, hand
-- grip, shoulders, lean, traversal), the authored reload, the melee clips, the vehicle entry and exit clips and Shooter mode
-- are off. How: M.Effective returns the PACK_FORCED values while "game" is in force (the stored options of the player are not
-- touched), the apply callbacks of those keys run again (M.ReapplyPack), the body-turn executor and the
-- reload gate are switched through their modules, the vehicle entry clips go through Game.UTPPSetVehicleEntryClips and the
-- melee clips through the wrapper variable utpp_melee_tpp (utpp_body).
-- The setting is the request. The pack in force changes at a safe moment only: with no player at once (menu, loading), in
-- the game when V has stood on the ground for PACK_SAFE_SECONDS without a vehicle, workspot, scene, takedown, carrying,
-- melee, aiming, weapon action (reload) or air / climb / slide state (M.PackBlockReason). The stored pack is in force from
-- the start of the game (LoadPack), so the dependent options register with their forced values.
-- M.pack.api (injected by the tests): mod(name) -> module table, setVehicleEntryClips(on) -> bool.
M.PACK_SAFE_SECONDS = 0.5

function M.PackIsGame() return M.pack.applied == "game" end

local function packModule(name)
  local P = M.pack
  local getter = P.api and P.api.mod
  local ok, mod
  if getter then ok, mod = pcall(getter, name)
  elseif type(GetMod) == "function" then ok, mod = pcall(GetMod, name) end
  if ok and type(mod) == "table" then return mod end
  return nil
end
M.PackModule = packModule

-- Runs the apply callback of every forced key with the value in force, then switches the modules that have no key. Nil-safe
-- both ways: a module that is not loaded (yet) reads the pack itself when it starts (core.PackIsGame).
function M.ReapplyPack()
  for _, key in ipairs(S.order) do
    local def = S.defs[key]
    if def and def.apply and M.PACK_FORCED.game[key] ~= nil then
      local ok, err = pcall(def.apply, M.Effective(key))
      if not ok then M.Log("apply " .. key .. " failed: " .. tostring(err)) end
      if not S.uiSync then pcall(syncWidget, key) end -- the Native Settings fallback shows the value in force
    end
  end
  local on = not M.PackIsGame()
  for _, c in ipairs({ { "utpp_bodyturn", "SetExecutor" }, { "utpp_reload_gate", "SetEnabled" } }) do
    local mod = packModule(c[1])
    if mod and type(mod[c[2]]) == "function" then
      local ok, err = pcall(mod[c[2]], on)
      if not ok then M.Log(c[1] .. "." .. c[2] .. " failed: " .. tostring(err)) end
    end
  end
end

-- Puts a pack in force (the game scripts are told by PackSend).
function M.ApplyPack(v, why)
  local P = M.pack
  v = (v == "game") and "game" or "utpp"
  P.pending, P.safeFor, P.deferWhy = false, 0, nil
  if P.applied == v then return false end
  P.applied = v
  P.applies = P.applies + 1
  if why ~= "load" then
    M.ReapplyPack()
    M.Log(string.format("animation pack -> %s (%s)", v == "game" and "Game Animations" or "Ultimate Third Person Project", tostring(why)))
  end
  return true
end

-- The setting changed: in force at once without a player, otherwise at the next safe moment (PackTick).
function M.RequestAnimationPack(v, why)
  local P = M.pack
  v = (v == "game") and "game" or "utpp"
  P.requested = v
  if P.applied == v then
    P.pending, P.safeFor, P.deferWhy = false, 0, nil
    return false
  end
  if not P.inSession then return M.ApplyPack(v, why or "setting") end
  P.pending, P.safeFor = true, 0
  M.Log("animation pack change waits for a safe moment")
  return true
end

-- Called when the settings file was read: the stored pack is in force from the start (no game, nothing to defer).
function M.LoadPack()
  local v = (S.values[M.PACK_KEY] == "game") and "game" or "utpp"
  M.pack.requested = v
  M.ApplyPack(v, "load")
end

-- obs (nil = no player): { menu, vehicle, workspot, scene, swimming, takedown, carrying, reload, melee, aim, upper, loco,
-- unreadable }. Returns the reason why the pack must not change now, nil = safe.
function M.PackBlockReason(o)
  if o == nil then return nil end
  if o.unreadable then return "state unreadable" end
  if o.menu then return "menu" end
  if o.vehicle then return "vehicle" end
  if o.workspot then return "workspot" end
  if o.scene then return "scene" end
  if o.swimming then return "swimming" end
  if o.takedown then return "takedown" end
  if o.carrying then return "carrying" end
  if o.reload then return "reload" end
  if o.melee then return "melee" end
  if o.aim then return "aim" end
  if o.upper ~= nil and o.upper ~= 0 and o.upper ~= 7 then return "weapon action" end
  if o.loco ~= nil and o.loco ~= 1 and o.loco ~= 3 and o.loco ~= 4 then return "locomotion" end
  return nil
end

-- One step of a waiting pack change; returns true when it was put in force.
function M.PackTick(dt, obs)
  local P = M.pack
  if not P.pending then
    P.safeFor = 0
    return false
  end
  if obs == nil then return M.ApplyPack(P.requested, "no player") end
  local why = M.PackBlockReason(obs)
  if why then
    P.safeFor = 0
    if P.deferWhy ~= why then
      P.deferWhy = why
      M.Log("animation pack change deferred: " .. why)
    end
    return false
  end
  P.safeFor = P.safeFor + ((type(dt) == "number" and dt > 0) and dt or 0)
  if P.safeFor >= M.PACK_SAFE_SECONDS then return M.ApplyPack(P.requested, "safe moment") end
  return false
end

-- Tells the game scripts of the session which pack is in force (vehicle entry clips). A fresh session starts with the mod's
-- animations, so nothing is sent for "utpp" until the value changes. A refused call is retried after 0.5 s.
function M.PackSend(now, newSession)
  local P = M.pack
  if newSession then P.sent = nil end
  if P.sent == P.applied then return false end
  if P.sent == nil and P.applied == "utpp" then
    P.sent = "utpp"
    return false
  end
  if now < P.sentAt then return false end
  P.sentAt = now + 0.5
  local fn = P.api and P.api.setVehicleEntryClips
  if fn == nil and Game ~= nil then fn = function(on) return Game.UTPPSetVehicleEntryClips(on) end end
  if type(fn) ~= "function" then return false end
  local ok, res = pcall(fn, P.applied ~= "game")
  P.calls = P.calls + 1
  if ok and res ~= false then
    P.sent = P.applied
    return true
  end
  P.errors = P.errors + 1
  M.LogOnce("animationPack:send", "vehicle entry clips switch " .. (ok and "refused" or ("FAILED: " .. tostring(res))) .. " - retrying")
  return false
end

M.Register({ key = M.PACK_KEY, section = "general", type = "choice", default = "utpp",
  choices = { { "utpp", "Ultimate Third Person Project" }, { "game", "Game Animations" } },
  label = "Animation pack",
  desc = "Chooses the animations that V uses in third person. Ultimate Third Person Project: the animations of this mod "
    .. "(movement, aiming, reload, melee, vehicles). Game Animations: the animations of the game itself, with the third-person "
    .. "camera, body and crosshair kept; arm and shoulder deformations of the game animations can come back. The change works "
    .. "at once while V stands idle; melee, vehicle and reload animations change at the next weapon draw, vehicle entry or "
    .. "reload. During an action the change waits until it ends.",
  apply = function(v) M.RequestAnimationPack(v, "setting") end })

-- Head look: the main switch and "Look at the camera" are in the menu. The four tuning
-- options stay hidden = true: same defaults, stored values, Get / Set / Effective and console access, no menu entry.
M.Register({ key = "headLook.impl", section = "headlook", type = "bool", default = true, label = "Use UTPP head look",
  desc = "V turns her head (and a little of the chest) towards what the camera looks at while standing or walking in third person." })
M.Register({ key = "headLook.whileWalking", section = "headlook", type = "bool", default = true, label = "Also while walking", hidden = true,
  desc = "Keep looking while walking; running, sprinting, aiming, vehicles, scenes, ladders and swimming always release it.",
  apply = function(v) M.hl.cfg.whileWalking = v end })
M.Register({ key = "headLook.faceCamera", section = "headlook", type = "bool", default = true, label = "Look at the camera",
  desc = "When the camera is in front of V and looks at her, she looks into the camera.",
  apply = function(v) M.hl.cfg.faceCamera = v end })
M.Register({ key = "headLook.lookAtLoot", section = "headlook", type = "bool", default = true, label = "Look at nearby loot", hidden = true,
  desc = "With the loot view active, V looks at the selected item.",
  apply = function(v) M.hl.cfg.lookAtLoot = v end })
M.Register({ key = "headLook.maxYaw", section = "headlook", type = "float", default = 100, min = 40, max = 140, step = 5, hidden = true,
  format = "%.0f", label = "Head turn limit (deg)", desc = "Largest angle from the body forward; targets beyond are clamped.",
  apply = function(v) M.hl.cfg.maxYaw = v end })
M.Register({ key = "headLook.chest", section = "headlook", type = "float", default = 0.5, min = 0, max = 1, step = 0.05, hidden = true,
  label = "Chest follow", desc = "How much the upper body follows the head (0 = head only)." })

M.Register({ key = "lootView.impl", section = "loot", type = "bool", default = true, label = "Use UTPP loot view",
  desc = "Near lootable items in front of the camera the third-person camera moves closer, which makes them easier to target." })
M.Register({ key = "lootView.zoom", section = "loot", type = "float", default = 0.9, min = 0, max = 2, step = 0.1,
  label = "Zoom (m)", desc = "How much closer the camera moves at full assist.", apply = function(v) M.la.cfg.zoom = v end })
M.Register({ key = "lootView.radius", section = "loot", type = "float", default = 1.8, min = 0.8, max = 3, step = 0.1,
  label = "Radius (m)", desc = "Loot within this horizontal distance of V triggers the assist.",
  apply = function(v) M.la.cfg.radius = v end })

-- crosshair.impl (default on): the UTPP crosshair for ordinary guns (scripts\utpp_reticle.reds). The value lives in a script
-- system that exists per game session (Game.UTPPSetCrosshair); without a push the script uses on. M.xhair.sent = the value the
-- current session acknowledged (nil = not yet, new session or failed call); M.xhair.live = a game session with a player exists
-- (set by M.CrosshairResync, cleared by onUpdate without a player); M.xhair.api replaces the game call (offline tests).
-- Outside a session nothing is called; the session edge pushes the stored value and, while unacknowledged, retries once a second.
M.RegisterSection("crosshair", "Crosshair")
M.xhair = { api = nil, sent = nil, live = false, retryAt = 0, calls = 0, errors = 0 }
local function applyCrosshair(v)
  local on = v ~= false
  if M.xhair.sent == on then return end
  local fn = M.xhair.api
  if fn == nil then
    if Game == nil or not M.xhair.live then return end
    fn = function(value) return Game.UTPPSetCrosshair(value) end
  end
  local ok, res = pcall(fn, on)
  M.xhair.calls = M.xhair.calls + 1
  if not ok or res ~= true then
    M.xhair.errors = M.xhair.errors + 1
    M.xhair.sent = nil -- not acknowledged: the session retry or a later change sends it again
    M.LogOnce("crosshair.push", "UTPP crosshair push not applied: " .. tostring(res))
    return
  end
  M.xhair.sent = on
  M.Log("UTPP crosshair -> " .. tostring(on))
end
M.Register({ key = "crosshair.impl", section = "crosshair", type = "bool", default = true, label = "Use UTPP crosshair",
  desc = "Replaces the game's crosshair for ordinary guns with a dot, a spread ring and a black hit / kill marker. Smart "
    .. "weapons, charge weapons, power weapons with their own crosshair and scopes keep the game's crosshair.",
  apply = applyCrosshair })
-- crosshair.color: Standard (default, the game's HUD crosshair colour) | White | Red | Green | Black, pushed as an index to
-- Game.UTPPSetCrosshairColor(0..4) with the same session rules as crosshair.impl (M.xcolor.sent = acknowledged index;
-- M.xcolor.api replaces the game call in offline tests).
M.CROSSHAIR_COLORS = { standard = 0, white = 1, red = 2, green = 3, black = 4 }
M.xcolor = { api = nil, sent = nil, calls = 0, errors = 0 }
local function applyCrosshairColor(v)
  local idx = M.CROSSHAIR_COLORS[v] or 0
  if M.xcolor.sent == idx then return end
  local fn = M.xcolor.api
  if fn == nil then
    if Game == nil or not M.xhair.live then return end
    fn = function(value) return Game.UTPPSetCrosshairColor(value) end
  end
  local ok, res = pcall(fn, idx)
  M.xcolor.calls = M.xcolor.calls + 1
  if not ok or res ~= true then
    M.xcolor.errors = M.xcolor.errors + 1
    M.xcolor.sent = nil -- not acknowledged: the session retry or a later change sends it again
    M.LogOnce("crosshair.color.push", "UTPP crosshair colour push not applied: " .. tostring(res))
    return
  end
  M.xcolor.sent = idx
  M.Log("UTPP crosshair colour -> " .. tostring(v))
end
M.Register({ key = "crosshair.color", section = "crosshair", type = "choice", default = "standard",
  choices = { { "standard", "Standard" }, { "white", "White" }, { "red", "Red" }, { "green", "Green" }, { "black", "Black" } },
  label = "Crosshair colour",
  desc = "Colour of the UTPP crosshair's dot and ring. Standard (default): the game's crosshair colour, which also changes "
    .. "over enemies and friends like the game's own crosshair. White, Red, Green, Black: a fixed colour. With Black the "
    .. "shadow and the hit / kill marker turn white so they stay visible.",
  apply = applyCrosshairColor })
-- Ragdoll after a car hit (utpp_ragdoll.reds, needs the ragdoll asset package; without it the vanilla knockdown plays):
-- ragdoll.enabled (default OFF: no ragdoll archives in the release) and three developer tunables, pushed together as Game.UTPPRagdollSetConfig(enabled, impulseScale,
-- maxSeconds, getUpPush) with the same session rules as crosshair.impl (M.rg.sent = the acknowledged value signature;
-- M.rg.api replaces the game call in offline tests).
M.RegisterSection("ragdoll", "Ragdoll")
M.rg = { api = nil, sent = nil, retryAt = 0, calls = 0, errors = 0 }
local function applyRagdoll()
  local fn = M.rg.api
  if fn == nil then
    if Game == nil or not M.xhair.live then return end
    fn = function(e, s, m, g) return Game.UTPPRagdollSetConfig(e, s, m, g) end
  end
  local e, s, m, g = M.Effective("ragdoll.enabled"), M.Effective("ragdoll.impulseScale"), M.Effective("ragdoll.maxSeconds"),
    M.Effective("ragdoll.getUpPush")
  if e == nil then e = false end
  if s == nil then s = 1.0 end
  if m == nil then m = 2.5 end
  if g == nil then g = true end
  local sig = string.format("%s|%.3f|%.3f|%s", tostring(e), s, m, tostring(g))
  if M.rg.sent == sig then return end
  local ok, res = pcall(fn, e, s, m, g)
  M.rg.calls = M.rg.calls + 1
  if not ok or res ~= true then
    M.rg.errors = M.rg.errors + 1
    M.rg.sent = nil -- not acknowledged: the session retry or a later change sends it again
    M.LogOnce("ragdoll.push", "UTPP ragdoll push not applied: " .. tostring(res))
    return
  end
  M.rg.sent = sig
  M.Log("UTPP ragdoll config -> " .. sig)
end
M.Register({ key = "ragdoll.enabled", section = "ragdoll", type = "bool", default = false, label = "Ragdoll when hit by a car",
  desc = "Not part of this release (the ragdoll asset package is not shipped): the game's own knockdown animation always plays. "
    .. "Developer option for builds that include the UTPP ragdoll asset package; there it throws V as a physics ragdoll after a car hit.",
  apply = function() applyRagdoll() end })
M.Register({ key = "ragdoll.impulseScale", section = "ragdoll", type = "float", default = 1.0, min = 0.25, max = 2.0, step = 0.05,
  label = "Ragdoll hit strength", desc = "Scales how hard the car throws V (1 = tuned for the car speed).",
  apply = function() applyRagdoll() end })
M.Register({ key = "ragdoll.maxSeconds", section = "ragdoll", type = "float", default = 2.5, min = 1.2, max = 2.6, step = 0.1,
  label = "Ragdoll longest fall (s)", desc = "The ragdoll ends at the latest after this time, so the knockdown recovery animation "
    .. "still has the rest of the game's 4.1 s knockdown to play.",
  apply = function() applyRagdoll() end })
M.Register({ key = "ragdoll.getUpPush", section = "ragdoll", type = "bool", default = true, label = "Pick the get-up by body orientation",
  desc = "At the end of the ragdoll the game's knockdown recovery clip is chosen for a face-up or face-down body.",
  apply = function() applyRagdoll() end })
-- ragdoll.getUp (default on): the get-up chosen by how V lies (native get-up of the DLL). Off, or the DLL refusing for any reason
-- (data files missing, no usable pose, no room), = the game's own knockdown recovery above. Pushed as Game.UTPPRagdollSetGetUpEnabled(bool)
-- (utpp_ragdoll.reds; it returns true once stored, which the DLL native UTPPRagdollSetGetUp cannot) with the session rules of applyRagdoll;
-- M.rgu.sent = the acknowledged value, M.rgu.api replaces the game call in offline tests.
M.rgu = { api = nil, sent = nil, retryAt = 0, calls = 0, errors = 0 }
local function applyRagdollGetUp()
  local fn = M.rgu.api
  if fn == nil then
    if Game == nil or not M.xhair.live then return end
    fn = function(v) return Game.UTPPRagdollSetGetUpEnabled(v) end
  end
  local v = M.Effective("ragdoll.getUp")
  if v == nil then v = true end
  local sig = tostring(v)
  if M.rgu.sent == sig then return end
  local ok, res = pcall(fn, v)
  M.rgu.calls = M.rgu.calls + 1
  if not ok or res ~= true then
    M.rgu.errors = M.rgu.errors + 1
    M.rgu.sent = nil -- not acknowledged: the session retry or a later change sends it again
    M.LogOnce("ragdoll.getUp.push", "UTPP ragdoll get-up push not applied: " .. tostring(res))
    return
  end
  M.rgu.sent = sig
  M.Log("UTPP ragdoll get-up -> " .. sig)
end
M.Register({ key = "ragdoll.getUp", section = "ragdoll", type = "bool", default = true, label = "UTPP get-up animation",
  desc = "get-up chosen by how V lies after a car hit; off = the game's knockdown recovery.",
  apply = function() applyRagdollGetUp() end })
-- newSession: a game session with a player has just started (the script system is new): forget the acknowledged values and
-- push the stored ones. Otherwise only unacknowledged values are retried, at most once per second (now = seconds).
function M.CrosshairResync(now, newSession)
  now = now or 0
  if newSession then
    M.xhair.live, M.xhair.sent, M.xhair.retryAt = true, nil, 0
    M.xcolor.sent = nil
    M.rg.sent, M.rg.retryAt = nil, 0
    M.rgu.sent, M.rgu.retryAt = nil, 0
  end
  if M.xhair.live and M.rg.sent == nil and now >= M.rg.retryAt then -- ragdoll config: own retry throttle (once per second)
    M.rg.retryAt = now + 1
    applyRagdoll()
  end
  if M.xhair.live and M.rgu.sent == nil and now >= M.rgu.retryAt then -- get-up switch: its own retry throttle (once per second)
    M.rgu.retryAt = now + 1
    applyRagdollGetUp()
  end
  if not M.xhair.live or (M.xhair.sent ~= nil and M.xcolor.sent ~= nil) or now < M.xhair.retryAt then return end
  M.xhair.retryAt = now + 1
  if M.xhair.sent == nil then applyCrosshair(M.Effective("crosshair.impl")) end
  if M.xcolor.sent == nil then applyCrosshairColor(M.Effective("crosshair.color")) end
end

-- ================================================================ animation database (motion matching)
-- list = UTPPListDatabases() ("default;name1;..."), loader(name) = UTPPLoadDatabase(name) -> Bool (queued / active).
-- The selection is persisted (mm.database) and applied when registered (game start); a failed load is logged and the
-- selector returns to the active database (reported by statusFn = UTPPGetDatabaseSwitchStatus / activeFn).
M.db = { names = {}, loader = nil, statusFn = nil, activeFn = nil, lastStatus = nil, watch = 0, requests = 0 }
function M.ParseDatabaseList(s)
  local names, seen = {}, {}
  if type(s) ~= "string" then return names end
  for n in string.gmatch(s, "[^;]+") do
    n = string.gsub(n, "^%s*(.-)%s*$", "%1")
    if n ~= "" and not seen[n] and string.match(n, "^[%w_%-][%w_%.%-]*$") then
      seen[n] = true
      names[#names + 1] = n
    end
  end
  return names
end
M.PREFERRED_DATABASE = "utpp_base"
function M.SetupDatabaseSetting(list, loader, statusFn, activeFn)
  local names = M.ParseDatabaseList(list)
  if #names == 0 then return false end
  M.db.names, M.db.loader, M.db.statusFn, M.db.activeFn = names, loader, statusFn, activeFn
  local choices = {}
  for _, n in ipairs(names) do choices[#choices + 1] = { n, n } end
  -- user-chosen default: utpp_base when installed, else "default", else the first listed
  local default = names[1]
  for _, n in ipairs(names) do if n == "default" then default = "default" end end
  for _, n in ipairs(names) do if n == M.PREFERRED_DATABASE then default = n end end
  M.RegisterSection("animation", "Animation (motion matching)")
  M.Register({ key = "mm.database", section = "animation", type = "choice", choices = choices, default = default,
    label = "Animation database", desc = "Motion matching database (red4ext/plugins/UTPP/data/utpp_base.utppmm = default, "
      .. "data/<name>.utppmm). Switching loads in the background and blends over in about half a second.",
    apply = function(v)
      if not M.db.loader then return end
      local ok, res = pcall(M.db.loader, v)
      M.db.requests = M.db.requests + 1
      M.db.watch = 10 -- s of status polling
      M.Log(string.format("animation database %s requested: %s", tostring(v), (ok and res) and "queued" or ("REJECTED " .. tostring(res))))
    end })
  return true
end
function M.SetDatabase(name) return M.Set("mm.database", name) end

-- ================================================================ motion matching switches (mm.enabled, mm.trace)
-- setEnabled(on) = Game.UTPPSetMMEnabled, setTrace(on) = Game.UTPPSetMMTrace (natives of UTPP.dll; injected for the tests).
-- Applied when registered (game start, onInit) and on every change; a native call happens only when the effective value
-- differs from the last one sent (the console / test harness may still switch MM directly).
M.mm = { setEnabled = nil, setTrace = nil, enabled = nil, trace = nil, calls = 0, errors = 0,
  motionMagTap = false } -- motionMagTap: DLL default of UTPPSetMotionMagTap (off), see applyMotionMagTap
local function mmCall(fn, on, what, why)
  if type(fn) ~= "function" then return false end
  local ok, err = pcall(fn, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then M.mm.errors = M.mm.errors + 1 end
  M.Log(string.format("%s %s (%s)%s", what, on and "on" or "off", tostring(why), ok and "" or (" FAILED: " .. tostring(err))))
  return ok
end
function M.ApplyMM(why)
  local def = S.defs["mm.enabled"]
  if not def then return false end
  local on = M.Effective("mm.enabled") and true or false
  if M.mm.enabled == on then return false end
  if mmCall(M.mm.setEnabled, on, "motion matching", tostring(why or "apply")) then M.mm.enabled = on end
  return true
end
function M.ReapplyMM(why) return M.ApplyMM(why) end
-- mm.footReplant = off | crouch (default) - planted feet step instead of sliding (plant / replant in the
-- foot locker) while crouching. api.setFootReplant(mask) = Game.UTPPSetMMFootReplant (mask: 0 off, 2 crouch). Standing replant
-- (mask 1 / 3) is not offered here: experimental, console only - in run starts and sharp turns it stretches the legs beyond the
-- 15 cm foot-correction limit (offline loco test 20-23 cm). A failing call is logged once, the value is not marked as
-- sent, a later change tries again.
local kReplantMask = { off = 0, crouch = 2 }
local function applyFootReplant(v)
  local mask = kReplantMask[v]
  if mask == nil or M.mm.footReplant == mask then return end
  local fn = M.mm.setFootReplant
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, mask)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.footReplant", string.format("foot replant %s FAILED: %s (UTPP.dll without UTPPSetMMFootReplant?) - further failures "
      .. "not logged", tostring(v), ok and "rejected" or tostring(res)))
    return
  end
  M.mm.footReplant = mask
  M.Log(string.format("foot replant %s (mm.footReplant, mask %d)", tostring(v), mask))
end
-- mm.feedDump = true | false (default off) - one line per locomotion feed (raw motion matching input, chosen
-- clip, gates) into red4ext\logs\utpp-feed-<date-time>.txt for the offline replay tools. api.setFeedDump(on) =
-- Game.UTPPSetMMFeedDump (false = the dump could not start). Same failure handling as mm.footReplant (logged once, not marked as
-- sent, a later change tries again). Setting true again while it is on still calls the native (idempotent while the dump
-- runs; restarts a dump whose writer failed - the driver switched it off, Lua cannot see that).
local function applyFeedDump(v)
  local on = v and true or false
  if M.mm.feedDump == on and not on then return end
  local fn = M.mm.setFeedDump
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, on)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.feedDump", string.format("feed dump %s FAILED: %s (UTPP.dll without UTPPSetMMFeedDump?) - further failures "
      .. "not logged", on and "on" or "off", ok and "rejected" or tostring(res)))
    return
  end
  if M.mm.feedDump == on then return end
  M.mm.feedDump = on
  M.Log(string.format("feed dump %s (mm.feedDump)", on and "on" or "off"))
end
-- stage standstill idle: mm.idleHold = true (default) - standing still, V settles into the idle loop instead of
-- re-entering the end of the stop clip about once per second. api.setIdleHold(on) = Game.UTPPSetMMIdleHold. A failing call is
-- logged once, the value is not marked as sent, a later change tries again.
local function applyMotionGrip(v)
  local on = v == true
  if M.mm.motionGrip == on or type(M.mm.setMotionGrip) ~= "function" then return end
  local ok, err = pcall(M.mm.setMotionGrip, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.grip", "grip FAILED: " .. tostring(err))
    return
  end
  M.mm.motionGrip = on
  M.Log("motion grip -> " .. tostring(on))
end
local function applyMotionShootTransfer(v)
  local on = v == true
  if M.mm.motionShootTransfer == on or type(M.mm.setMotionShootTransfer) ~= "function" then return end
  local ok, err = pcall(M.mm.setMotionShootTransfer, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.shootTransfer", "Shoot transfer FAILED: " .. tostring(err))
    return
  end
  M.mm.motionShootTransfer = on
  M.Log("motion Pistol2H Shoot transfer -> " .. tostring(on))
end
-- mm.shinobiBonusReload = false (default) | true - "Reload animation during Shinobi bonus": short left-hand move to the magazine when a
-- slide or air dash refills it without a reload. api.setMotionMagTap(on) = Game.UTPPSetMotionMagTap (process-lifetime DLL flag, default
-- off). M.mm.motionMagTap starts at the DLL default (false), so nothing is called while the option stays off; a failed call is logged
-- once and not marked as sent (a later change tries again).
local function applyMotionMagTap(v)
  local on = v == true
  if M.mm.motionMagTap == on or type(M.mm.setMotionMagTap) ~= "function" then return end
  local ok, err = pcall(M.mm.setMotionMagTap, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.shinobiBonusReload", "Shinobi bonus reload animation FAILED: " .. tostring(err))
    return
  end
  M.mm.motionMagTap = on
  M.Log("motion Shinobi bonus reload animation -> " .. tostring(on))
end

local function applyWeaponUpper(v)
  local on = v == true
  if M.mm.weaponUpper == on then return end
  if type(M.mm.setWeaponUpper) ~= "function" then return end
  local ok, err = pcall(M.mm.setWeaponUpper, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.weaponUpper", "weapon upper FAILED: " .. tostring(err))
    return
  end
  M.mm.weaponUpper = on
  M.Log("weapon upper -> " .. tostring(on))
end

-- melee full body (DLL mm/MeleeFullBody.hpp): standing melee hands the legs to the game's melee clips as well
-- (only for the melee weapon types the UTPP melee clips cover: scripts\utpp_melee.reds). Off: the legs stay on motion matching.
local function applyMeleeFullBody(v)
  local on = v == true
  if M.mm.meleeFullBody == on then return end
  if type(M.mm.setMeleeFullBody) ~= "function" then return end
  local ok, err = pcall(M.mm.setMeleeFullBody, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("melee.fullBody", "melee full body FAILED: " .. tostring(err))
    return
  end
  M.mm.meleeFullBody = on
  M.Log("melee full body -> " .. tostring(on))
end

-- melee aim tilt (DLL mm/MeleeAim.hpp): the spine leans toward the camera aim point during melee, up to 20 deg up and
-- 35 deg down (10 Hz smoothing). Off sends 0 / 0 (the DLL default).
M.MELEE_AIM_TILT = { maxUpDeg = 20.0, maxDownDeg = 35.0, smoothHz = 10.0 }
local function applyMeleeAimTilt(v)
  local on = v == true
  if M.mm.meleeAimTilt == on then return end
  if type(M.mm.setMeleeAimParams) ~= "function" then return end
  local p = M.MELEE_AIM_TILT
  local ok, res = pcall(M.mm.setMeleeAimParams, on and p.maxUpDeg or 0.0, on and p.maxDownDeg or 0.0, p.smoothHz)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("melee.aimTilt", "melee aim tilt FAILED: " .. tostring(res))
    return
  end
  M.mm.meleeAimTilt = on
  M.Log("melee aim tilt -> " .. tostring(on))
end

-- melee guard (DLL mm/MeleeFullBody.hpp GuardGate): with fists drawn (never Gorilla Arms) the upper body keeps
-- the game's guard pose between strikes; guardFullBody = the legs too while standing in the guard; guardAimTilt = the melee aim
-- tilt also through the guard. All default off (the third-person guard pose is not installed yet).
local function applyMeleeGuardBool(key, field, fn, label)
  return function(v)
    local on = v == true
    if M.mm[field] == on then return end
    if type(M.mm[fn]) ~= "function" then return end
    local ok, err = pcall(M.mm[fn], on)
    M.mm.calls = M.mm.calls + 1
    if not ok then
      M.mm.errors = M.mm.errors + 1
      M.LogOnce(key, label .. " FAILED: " .. tostring(err))
      return
    end
    M.mm[field] = on
    M.Log(label .. " -> " .. tostring(on))
  end
end
local applyMeleeGuard = applyMeleeGuardBool("melee.guard", "meleeGuard", "setMeleeGuard", "melee guard")
local applyMeleeGuardFullBody = applyMeleeGuardBool("melee.guardFullBody", "meleeGuardFullBody", "setMeleeGuardFullBody",
  "melee guard full body")
local applyMeleeGuardAimTilt = applyMeleeGuardBool("melee.guardAimTilt", "meleeGuardAimTilt", "setMeleeGuardAimTilt",
  "melee guard aim tilt")

local function applyAimUpperFade(v)
  local on = v == true
  if M.mm.aimUpperFade == on then return end
  if type(M.mm.setAimUpperFade) ~= "function" then return end
  local ok, err = pcall(M.mm.setAimUpperFade, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.aimUpperFade", "aim upper fade FAILED: " .. tostring(err))
    return
  end
  M.mm.aimUpperFade = on
  M.Log("aim upper fade -> " .. tostring(on))
end

local function applyIdleHold(v)
  local on = v and true or false
  if M.mm.idleHold == on then return end
  local fn = M.mm.setIdleHold
  if type(fn) ~= "function" then return end
  local ok, err = pcall(fn, on)
  M.mm.calls = M.mm.calls + 1
  if not ok then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.idleHold", string.format("standstill idle hold %s FAILED: %s (UTPP.dll without UTPPSetMMIdleHold?) "
      .. "- further failures not logged", on and "on" or "off", tostring(err)))
    return
  end
  M.mm.idleHold = on
  M.Log(string.format("standstill idle hold %s (mm.idleHold)", on and "on" or "off"))
end
-- Head lead: mm.headLead = true (default) - while V runs a turn her head turns into the turn (Game.UTPPSetMMHeadLead,
-- motion poses only). Same failure handling as mm.idleHold.
local function applyHeadLead(v)
  local on = v and true or false
  if M.mm.headLead == on then return end
  local fn = M.mm.setHeadLead
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, on)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.headLead", string.format("head lead %s FAILED: %s (UTPP.dll without UTPPSetMMHeadLead?) - further failures "
      .. "not logged", on and "on" or "off", ok and "rejected" or tostring(res)))
    return
  end
  M.mm.headLead = on
  M.Log(string.format("head lead %s (mm.headLead)", on and "on" or "off"))
end
-- knee fix: mm.kneeFix = true (default) - the right knee no longer swings inward / sideways on starts (no
-- stance-guard knee swivel while crouching, foot-lock knee pole carried from the animation). api.setKneeFix(mask) =
-- Game.UTPPSetMMKneeFix (mask 7 on, 0 off = the plain leg pipeline). Same failure handling as mm.footReplant (logged once, not
-- marked as sent, a later change tries again). On = mask 7 (bit 4: the stance-guard knee swivel slowed while the foot is
-- planted, standing turn starts); a UTPP.dll without bit 4 rejects mask 7 -> mask 3 (its knee fix) is sent instead and
-- remembered (M.mm.kneeOldDll: later changes send 3 directly). Bit 4 has its own setting mm.kneeHold = true
-- (default; off = mask 3). Both settings feed one mask: the call waits until both are registered.
local function applyKneeFix()
  if M.settings.defs["mm.kneeFix"] == nil or M.settings.defs["mm.kneeHold"] == nil then return end
  local v = M.Get("mm.kneeFix") == true
  local mask = v and ((M.Get("mm.kneeHold") ~= false and not M.mm.kneeOldDll) and 7 or 3) or 0
  if M.mm.kneeFix == mask then return end
  local fn = M.mm.setKneeFix
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, mask)
  M.mm.calls = M.mm.calls + 1
  if mask == 7 and ok and res == false then
    mask = 3
    if M.mm.kneeFix == mask then return end
    ok, res = pcall(fn, mask)
    M.mm.calls = M.mm.calls + 1
    if ok and res ~= false then M.mm.kneeOldDll = true end
  end
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.kneeFix", string.format("knee fix %s FAILED: %s (UTPP.dll without UTPPSetMMKneeFix?) - further failures "
      .. "not logged", v and "on" or "off", ok and "rejected" or tostring(res)))
    return
  end
  M.mm.kneeFix = mask
  M.Log(string.format("knee fix %s (mm.kneeFix, mask %d)", v and "on" or "off", mask))
end
-- AIR HOLD stage: mm.air = auto (default) | clips | hold | off - jumps, drops and landings. Auto and clips are the
-- same in the driver (MotionHost.cpp g_air.SetMode: Clips): the takeoff / fall / landing clips play when the air table
-- (.airz) of the loaded database is valid (it ships with the mod), otherwise the air phase machine runs in hold mode. Hold: while
-- V is airborne the legs no longer pick turns / pivots mid-air, no foot locks or body drag in the air, slowed stride, clean resume
-- after landing, no jump clips. Off = disabled. api.setAir(mode) = Game.UTPPSetMMAir (0 auto, 1 clips, 2 hold, 3 off). mm.airProbe = true (default) |
-- false: ground raycast while falling (api.setAirProbe = Game.UTPPSetMMAirProbe). mm.airSteer = true (default) | false: in-air
-- body steering (api.setAirSteer = Game.UTPPSetMMAirSteer; experimental, body yaw follows the stick <= 200 deg/s in the air). Same failure handling as mm.footReplant (logged
-- once, not marked as sent, a later change tries again).
local kAirMode = { auto = 0, clips = 1, hold = 2, off = 3 }
local function applyAir(v)
  local mode = kAirMode[v]
  if mode == nil or M.mm.air == mode then return end
  local fn = M.mm.setAir
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, mode)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.air", string.format("air mode %s FAILED: %s (UTPP.dll without UTPPSetMMAir?) - further failures not logged",
      tostring(v), ok and "rejected" or tostring(res)))
    return
  end
  M.mm.air = mode
  M.Log(string.format("air mode %s (mm.air, mode %d)", tostring(v), mode))
end
local function applyAirBool(key, field, fnField, what)
  return function(v)
    local on = v and true or false
    if M.mm[field] == on then return end
    local fn = M.mm[fnField]
    if type(fn) ~= "function" then return end
    local ok, err = pcall(fn, on)
    M.mm.calls = M.mm.calls + 1
    if not ok or err == false then
      M.mm.errors = M.mm.errors + 1
      M.LogOnce(key, string.format("%s %s FAILED: %s - further failures not logged", what, on and "on" or "off",
        ok and "rejected" or tostring(err)))
      return
    end
    M.mm[field] = on
    M.Log(string.format("%s %s (%s)", what, on and "on" or "off", key))
  end
end
local applyAirProbe = applyAirBool("mm.airProbe", "airProbe", "setAirProbe", "air ground probe")
local applyAirSteer = applyAirBool("mm.airSteer", "airSteer", "setAirSteer", "air steering")
-- AIR stage: mm.airRobust = true (default) | false (api.setAirRobust = Game.UTPPSetMMAirRobust): physics falls and the
-- touchdown guard; false = the previous air phase machine. Same failure handling as mm.airSteer.
local applyAirRobust = applyAirBool("mm.airRobust", "airRobust", "setAirRobust", "air physics falls")
-- CLIMB RELEASE: mm.climbRelease = false (default) | true. On: while V climbs or vaults (PSM Climb / Vault)
-- the body plays the game's climb / vault animation (motion matching fades out and back in after it) unless the path
-- plays its own traversal. Off: the motion matching pose is held as before. Vehicle (GameIntegration.hpp
-- TravReleases): with motion traversal active (mm.motionDriver on and traversal data in the bundle) every climb / vault that does not
-- play goes to the game animation whatever this setting says; the setting decides only with motion off. api.setClimbRelease(on) = Game.UTPPSetMMClimbRelease
-- (-> Bool). Same failure handling as mm.air (logged once, not marked as sent, a later change tries again).
local function applyClimbRelease(v)
  local on = v and true or false
  if M.mm.climbRelease == on then return end
  local fn = M.mm.setClimbRelease
  if type(fn) ~= "function" then return end
  local ok, res = pcall(fn, on)
  M.mm.calls = M.mm.calls + 1
  if not ok or res == false then
    M.mm.errors = M.mm.errors + 1
    M.LogOnce("mm.climbRelease", string.format("climb / vault release %s FAILED: %s (UTPP.dll without UTPPSetMMClimbRelease?) - "
      .. "further failures not logged", on and "on" or "off", ok and "rejected" or tostring(res)))
    return
  end
  M.mm.climbRelease = on
  M.Log(string.format("climb / vault release %s (mm.climbRelease)", on and "on" or "off"))
end
-- api (optional table): further natives of the motion matching settings, injected so the Lua tests can mock them
-- (setFootReplant, setFeedDump, setIdleHold, setKneeFix, ...; further fields are added here instead of new positional
-- arguments)
function M.SetupMMSettings(setEnabled, setTrace, api)
  api = type(api) == "table" and api or {}
  M.mm.setEnabled, M.mm.setTrace = setEnabled, setTrace
  M.mm.setFootReplant = api.setFootReplant
  M.mm.setFeedDump = api.setFeedDump
  M.mm.setIdleHold = api.setIdleHold
  M.mm.setAimUpperFade = api.setAimUpperFade
  M.mm.setWeaponUpper = api.setWeaponUpper
  M.mm.setMeleeFullBody = api.setMeleeFullBody   -- melee full body / aim tilt
  M.mm.setMeleeAimParams = api.setMeleeAimParams
  M.mm.setMeleeGuard = api.setMeleeGuard         -- melee guard
  M.mm.setMeleeGuardFullBody = api.setMeleeGuardFullBody
  M.mm.setMeleeGuardAimTilt = api.setMeleeGuardAimTilt
  M.mm.setMotionGrip = api.setMotionGrip
  M.mm.setMotionShootTransfer = api.setMotionShootTransfer
  M.mm.setMotionMagTap = api.setMotionMagTap
  M.mm.setKneeFix = api.setKneeFix
  M.mm.setHeadLead = api.setHeadLead
  M.mm.setAir = api.setAir             -- AIR HOLD stage
  M.mm.setAirProbe = api.setAirProbe
  M.mm.setAirSteer = api.setAirSteer
  M.mm.setAirRobust = api.setAirRobust
  M.mm.setClimbRelease = api.setClimbRelease -- CLIMB RELEASE stage
  M.RegisterSection("animation", "Animation (motion matching)")
  M.Register({ key = "mm.enabled", section = "animation", type = "bool", default = true, label = "Use motion matching",
    desc = "UTPP motion matching drives V's third-person locomotion (stand, walk, run, sprint, crouch).",
    apply = function() M.ApplyMM("setting") end })
  M.Register({ key = "mm.trace", section = "animation", type = "bool", default = false, label = "Motion matching trace (log)",
    desc = "Writes every clip transition and a 1 Hz summary (MMTRACE) to the UTPP red4ext log. Diagnostics only.",
    apply = function(v)
      v = v and true or false
      if M.mm.trace == v then return end
      if mmCall(M.mm.setTrace, v, "motion matching trace", "setting") then M.mm.trace = v end
    end })
  M.Register({ key = "mm.footReplant", section = "animation", type = "choice", default = "crouch",
    choices = { { "off", "Off (feet slide)" }, { "crouch", "Crouch" } },
    label = "Planted feet step instead of sliding",
    desc = "Crouch (default): when V starts, stops or turns while crouching, a planted foot stays where it is; if it has to move it "
      .. "makes one short, slightly lifted step, one foot at a time, instead of sliding back and snapping forward. Off: the previous "
      .. "behaviour (planted feet slide softly).",
    apply = applyFootReplant })
  M.Register({ key = "mm.feedDump", section = "animation", type = "bool", default = false,
    label = "Motion matching feed dump (log)",
    desc = "Writes one line per locomotion update (the raw motion matching input, the chosen clip and the gate) to "
      .. "red4ext\\logs\\utpp-feed-<date-time>.txt for the offline replay tools. Diagnostics only.",
    apply = applyFeedDump })
  M.Register({ key = "mm.weaponUpper", section = "animation", type = "bool", default = false,
    label = "Weapon held: keep the game's upper body",
    desc = "Keeps the game's low-ready arms above MM legs while a live weapon is in hand, including jumps/falls. "
      .. "Requires the current REDscript feed; absent/empty slot stays on MM. Off = previous upper-body policy.",
    apply = applyWeaponUpper })
  M.Register({ key = "melee.fullBody", section = "animation", type = "bool", default = true,
    label = "Melee full body",
    desc = "Fists only: standing punches, holds and blocks use the whole melee animation, legs included (not arm cyberware). "
      .. "Throws, other melee weapons and arm cyberware keep the game's animation and the locomotion legs. Moving, crouching "
      .. "or in the air the legs keep the locomotion. Off: the legs always keep the locomotion during melee.",
    apply = applyMeleeFullBody })
  M.Register({ key = "melee.aimTilt", section = "animation", type = "bool", default = true,
    label = "Melee aim tilt",
    desc = "Melee attacks and blocks (fists and the other melee weapons) lean V's upper body toward where you aim (up to 20 "
      .. "degrees up, 35 down, within natural limits), like the body and rifle while aiming a gun. Off: the melee animations "
      .. "play unchanged.",
    apply = applyMeleeAimTilt })
  M.Register({ key = "melee.guard", section = "animation", type = "bool", default = false,
    label = "Melee guard (experimental)",
    desc = "With fists drawn V keeps the game's guard pose between strikes instead of lowering the arms after every hit "
      .. "(not with Gorilla Arms). Needs the third-person guard animation; until it is installed this shows the first-person "
      .. "pose, so leave it off.",
    apply = applyMeleeGuard })
  M.Register({ key = "melee.guardFullBody", section = "animation", type = "bool", default = false,
    label = "Melee guard: legs (experimental)",
    desc = "With Melee guard and Melee full body on: standing in the guard, the legs play the guard animation too. Off: the legs "
      .. "keep the locomotion while guarding.",
    apply = applyMeleeGuardFullBody })
  M.Register({ key = "melee.guardAimTilt", section = "animation", type = "bool", default = false,
    label = "Melee guard: aim tilt",
    desc = "With Melee guard and Melee aim tilt on: the guard leans toward where you aim as well, so the lean does not fade in "
      .. "and out at every strike.",
    apply = applyMeleeGuardAimTilt })
  M.Register({ key = "mm.grip", section = "animation", type = "bool", default = true,
    label = "motion: left-hand weapon grip",
    desc = "Keeps the left hand on the captured weapon grip for two-handed pistols and rifles while motion owns the upper body. Off disables this correction.",
    apply = applyMotionGrip })
  M.Register({ key = "mm.shootTransfer", section = "animation", type = "bool", default = true,
    label = "Pistol 2H: native shot motion on the aim pose",
    desc = "Stationary Pistol2H: transfer fresh native shot motion onto the authored Aim pose and keep both hands on the weapon. Default on.",
    apply = applyMotionShootTransfer })
  M.Register({ key = "mm.shinobiBonusReload", section = "animation", type = "bool", default = false,
    label = "Reload animation during Shinobi bonus",
    desc = "Short left-hand move to the magazine when a slide or air dash refills the magazine (Shinobi bonus). Off by default.",
    apply = applyMotionMagTap })
  M.Register({ key = "mm.aimUpperFade", section = "animation", type = "bool", default = true,
    label = "Aim exit: ease the upper body back",
    desc = "Returns the arms from aiming over 0.25 s, including motion weapon overlays. Off = the previous aim exit.",
    apply = applyAimUpperFade })
  M.Register({ key = "mm.idleHold", section = "animation", type = "bool", default = true,
    label = "Standing still: settle into the idle",
    desc = "On (default): when V stands still after a stop (standing or crouching), she settles into the idle loop instead of "
      .. "replaying the end of the stop clip about once per second (a small pop). Off: the previous behaviour.",
    apply = applyIdleHold })
  M.Register({ key = "mm.kneeFix", section = "animation", type = "bool", default = true,
    label = "Steady knees on starts",
    desc = "On (default): when V starts moving (crouching, or turning on the spot first) her knees keep the bend of the animation "
      .. "instead of swinging inward or sideways for a few frames. Off: the previous behaviour.",
    apply = applyKneeFix })
  M.Register({ key = "mm.kneeHold", section = "animation", type = "bool", default = true,
    label = "Steady knee over a planted foot",
    desc = "On (default; needs \"Steady knees on starts\"): on standing turn starts the knee over the planted foot follows the "
      .. "turn more slowly instead of swinging sideways (tuned for the motion set database). Off: the previous behaviour (0.7.0).",
    apply = applyKneeFix })
  M.Register({ key = "mm.headLead", section = "animation", type = "bool", default = true,
    label = "Head follows turns",
    desc = "While V runs or sprints through a turn, her head turns into the turn. Off: the head stays as the run animations have it.",
    apply = applyHeadLead })
  M.Register({ key = "mm.air", section = "animation", type = "choice", default = "auto",
    choices = { { "auto", "Auto" }, { "clips", "Jump clips" }, { "hold", "Hold (no jump clips)" }, { "off", "Off" } },
    label = "Jumps, drops and landings",
    desc = "Auto (default) and Jump clips: takeoff, fall and landing clips play for jumps, drops and falls (the data they "
      .. "need ships with the mod; without it they behave like Hold). Hold: no jump clips - while V is in the air her legs no longer "
      .. "pick turns or pivots mid-air, the feet are not locked to the ground and the body is not dragged, the stride slows down, and "
      .. "after the landing the normal movement resumes cleanly. Off: the previous behaviour.",
    apply = applyAir })
  M.Register({ key = "mm.airProbe", section = "animation", type = "bool", default = true,
    label = "Air: measure the ground below",
    desc = "On (default): while V falls, a short ground check below her estimates when she lands. Off: the landing time is guessed "
      .. "from a flat jump only.",
    apply = applyAirProbe })
  M.Register({ key = "mm.airSteer", section = "animation", type = "bool", default = true,
    label = "Air: turn the body in the air",
    desc = "On (default). Experimental: V's body turns towards the stick while airborne (at most 200 deg/s). Meant to "
      .. "stop a mid-air stick change pushing the jump sideways (not yet confirmed in game). Needs the held pose in the "
      .. "air (UTPPSetMMHoldPose on, the default).",
    apply = applyAirSteer })
  M.Register({ key = "mm.airRobust", section = "animation", type = "bool", default = true,
    label = "Air: falls and safe landings",
    desc = "On (default): walking or running off a ledge (or being thrown out of a car) plays the fall and landing like a jump, "
      .. "and a one-frame ground contact while still rising no longer plays a landing in the air. Off: previous behaviour.",
    apply = applyAirRobust })
  M.Register({ key = "mm.climbRelease", section = "animation", type = "bool", default = false,
    label = "Climbing and vaulting without the motion set: game animation",
    desc = "With the player motion driver on, climbs and vaults that the motion set cannot play always use the game's own "
      .. "animation, whatever this setting says; it only decides with motion off. With motion off: On = V's body plays the game's own climb / "
      .. "vault animation and motion matching fades back in afterwards. Off (default) = the last motion matching pose is held while "
      .. "climbing or vaulting.",
    apply = applyClimbRelease })
  return true
end
-- polled from onUpdate while a request is fresh: logs status changes; on failure the selector returns to the active one
function M.PollDatabase(dt)
  if M.db.watch <= 0 or not M.db.statusFn then return end
  M.db.watch = M.db.watch - (dt or 0)
  local ok, st = pcall(M.db.statusFn)
  if not ok or type(st) ~= "string" or st == M.db.lastStatus then return end
  M.db.lastStatus = st
  M.Log("animation database: " .. st)
  if string.sub(st, 1, 6) == "FAILED" or string.sub(st, 1, 2) == "ok" then M.db.watch = 0 end
  if string.sub(st, 1, 6) == "FAILED" and M.db.activeFn then
    local okA, active = pcall(M.db.activeFn)
    if okA and type(active) == "string" and S.defs["mm.database"] and validate(S.defs["mm.database"], active) then
      M.db.loader, M.db.saved = nil, M.db.loader -- revert without a new request
      M.Set("mm.database", active)
      M.db.loader, M.db.saved = M.db.saved, nil
    end
  end
end

-- ================================================================ night harness probe
-- Format (parsed by the test harness):
--   [night] POS p=x,y,z cf=fx,fy v=x,y,z,rx,ry|nil src=mounted|tag|summon|none w=0|1|- m=0|1|-
-- While mounted, v = the vehicle V sits in (src=mounted).
-- p = player world position, cf = active camera forward (x,y), v = vehicle position + its world right (x,y),
-- w = item in AttachmentSlots.WeaponRight, m = player mounted in a vehicle; "-" = could not be read.
-- APIs: Entity.GetWorldPosition/GetWorldRight (entity.script:44/:52), CameraSystem.GetActiveCameraForward
-- (orphans.script:26736), Codeware DynamicEntitySystem.GetTagged, VehicleSummonDataDef.SummonedVehicleEntityID
-- (blackboardDefinitions.script:1005), TransactionSystem.GetItemInSlot (orphans.script:18131),
-- PlayerPuppet.GetMountedVehicle (player.script:1414).
function M.NightPosLine(tag)
  if Game == nil then return nil, "no Game" end
  local okP, p = pcall(function() return Game.GetPlayer() end)
  if not okP or p == nil then return nil, "no player" end
  local okA, a = pcall(function() return p:GetWorldPosition() end)
  if not okA or a == nil then return nil, "no player position" end
  local okF, f = pcall(function() return Game.GetCameraSystem():GetActiveCameraForward() end)
  if not okF or f == nil then f = { x = 0, y = 0 } end
  local v, src = nil, "none"
  local okM, mv = pcall(function() return p:GetMountedVehicle() end)
  if okM and mv ~= nil then v, src = mv, "mounted" end
  if v == nil and tag and tag ~= "" then
    pcall(function()
      local t = Game.GetDynamicEntitySystem():GetTagged(tag)
      if t and t[1] then v, src = t[1], "tag" end
    end)
  end
  if v == nil then
    pcall(function()
      local d = GetAllBlackboardDefs().VehicleSummonData
      local e = Game.FindEntityByID(Game.GetBlackboardSystem():Get(d):GetEntityID(d.SummonedVehicleEntityID))
      if e then v, src = e, "summon" end
    end)
  end
  local vs = "nil"
  if v ~= nil then
    pcall(function()
      local b, r = v:GetWorldPosition(), v:GetWorldRight()
      vs = string.format("%.2f,%.2f,%.2f,%.3f,%.3f", b.x, b.y, b.z, r.x, r.y)
    end)
    if vs == "nil" then src = "none" end
  end
  local w, m = "-", "-"
  pcall(function()
    local item = Game.GetTransactionSystem():GetItemInSlot(p, TweakDBID.new("AttachmentSlots.WeaponRight"))
    w = item ~= nil and "1" or "0"
  end)
  if okM then m = mv ~= nil and "1" or "0" end
  -- evening: Locomotion state per sample (active 0/1 and the block reasons) - the body did not turn on the
  -- run arcs (cam == pl yaw, btCmds=0) and nothing logged the reasons while running
  local freeMove = "-"
  pcall(function()
    local act = Game.UTPPLocomotionActive() and 1 or 0
    freeMove = string.format("%d/0x%X", act, Game.UTPPLocomotionReasons() or 0)
  end)
  -- PSM Weapon / UpperBody and the WeaponRight item record next to the CET slot read, to settle why the
  -- Locomotion heartbeat (redscript GetItemInSlot) reported a weapon while this CET read said none
  local psmW, ub, wi = "-", "-", "none"
  pcall(function()
    local D = GetAllBlackboardDefs().PlayerStateMachine
    local psm = Game.GetBlackboardSystem():GetLocalInstanced(p:GetEntityID(), D)
    if psm ~= nil then
      -- game 2.5: PSM.Weapon is RightWeapon (dual wield split); psmW keeps reporting the right hand
      psmW = tostring(psm:GetInt(D.RightWeapon or D.Weapon)); ub = tostring(psm:GetInt(D.UpperBody))
    end
  end)
  pcall(function()
    local item = Game.GetTransactionSystem():GetItemInSlot(p, TweakDBID.new("AttachmentSlots.WeaponRight"))
    if item ~= nil then wi = TDBID.ToStringDEBUG(ItemID.GetTDBID(item:GetItemID())) end
  end)
  return string.format("[night] POS p=%.2f,%.2f,%.2f cf=%.3f,%.3f v=%s src=%s w=%s m=%s freeMove=%s psmW=%s ub=%s wi=%s",
    a.x, a.y, a.z, f.x or 0, f.y or 0, vs, src, w, m, freeMove, psmW, ub, wi)
end

function M.NightPos(tag)
  local line, why = M.NightPosLine(tag)
  line = line or ("[night] POS unavailable: " .. tostring(why))
  if Game ~= nil then pcall(function() Game.UTPPLogLine(M.ScrubPaths(line)) end) end -- always logged (independent of core.log)
  return line
end

-- periodic POS lines (the test harness never has to open the CET console during scenarios)
M.night = { on = false, period = 0.5, tag = "UTPPNight", acc = 0, n = 0 }
function M.NightAuto(on, periodS, tag)
  local nt = M.night
  nt.on = on and true or false
  nt.period = math.max(tonumber(periodS) or 0.5, 0.05)
  if tag ~= nil then nt.tag = tostring(tag) end
  nt.acc = nt.period -- first line on the next update
  local msg = string.format("[night] auto %s period=%.2f tag=%s", nt.on and "on" or "off", nt.period, nt.tag)
  if Game ~= nil then pcall(function() Game.UTPPLogLine(M.ScrubPaths(msg)) end) end
  return msg
end
-- onUpdate: returns the logged line or nil (pure apart from NightPosLine / UTPPLogLine)
function M.NightTick(dt)
  local nt = M.night
  if not nt.on then return nil end
  nt.acc = nt.acc + math.max(dt or 0, 0)
  if nt.acc < nt.period then return nil end
  nt.acc = 0
  local line = M.NightPosLine(nt.tag)
  if line == nil then return nil end
  nt.n = nt.n + 1
  line = line .. " n=" .. nt.n
  if Game ~= nil then pcall(function() Game.UTPPLogLine(M.ScrubPaths(line)) end) end
  return line
end

function M.Status()
  local hl, la = M.hl, M.la
  return string.format("utpp_core %s headLook=%s state=%s reason=%s applies=%d clears=%d "
    .. "| lootAssist=%s weight=%.2f reason=%s tracked=%d target=%s "
    .. "| db=%s requests=%d last=%s | mm=%s (sent %s) trace=%s",
    M.VERSION,
    tostring(M.Effective("headLook.impl")), hl.state, tostring(hl.reason), hl.applies, hl.clears,
    tostring(M.Effective("lootView.impl")), la.weight, tostring(la.reason),
    la.tracker.n, la.target and tostring(la.target.key) or "-",
    "-", M.db.requests, tostring(M.db.lastStatus), tostring(M.Effective("mm.enabled")),
    tostring(M.mm.enabled), tostring(M.mm.trace))
end

-- settings file read at load time, so that options registered later by other mods find their stored values
pcall(M.Load)

-- ================================================================ in-game wiring (skipped by the offline tests)
M.wiring = { camMethod = nil, errors = 0 }
local W = M.wiring

if type(registerForEvent) == "function" then
  M.sink = function(line)
    if M.Get("core.log") == false then return end
    print(line)
    if Game ~= nil then pcall(function() Game.UTPPLogLine(M.ScrubPaths(line)) end) end
  end
  local clock, announced, inGame = 0, false, false
  local lastLog = {}

  local function getSystem(name)
    local ok, s = pcall(function() return Game.GetScriptableSystemsContainer():Get(name) end)
    if ok then return s end
    return nil
  end

  local function vec(v) return { x = v.x, y = v.y, z = v.z } end

  -- active camera world position: tries the CET out-parameter forms, logs once which one worked
  local function cameraPos(camSys)
    if W.camMethod == nil or W.camMethod == "ret2" then
      local ok, a, b = pcall(function() return camSys:GetActiveCameraWorldTransform() end)
      if ok and type(b) ~= "boolean" and b ~= nil then
        local okP, p = pcall(function() return b.position end)
        if okP and p ~= nil then W.camMethod = "ret2"; return vec(p) end
      end
      if ok and type(a) ~= "boolean" and a ~= nil then
        local okP, p = pcall(function() return a.position end)
        if okP and p ~= nil then W.camMethod = "ret2"; return vec(p) end
      end
    end
    if W.camMethod == nil or W.camMethod == "arg" then
      local ok, p = pcall(function()
        local xf = Transform.new()
        camSys:GetActiveCameraWorldTransform(xf)
        return xf.position
      end)
      if ok and p ~= nil and (p.x ~= 0 or p.y ~= 0 or p.z ~= 0) then W.camMethod = "arg"; return vec(p) end
    end
    if W.camMethod == nil then W.camMethod = "none"; M.Log("camera position read FAILED - head look / loot assist inactive") end
    return nil
  end

  local function psm(player)
    local r = {}
    pcall(function()
      local defs = GetAllBlackboardDefs().PlayerStateMachine
      local bb = Game.GetBlackboardSystem():GetLocalInstanced(player:GetEntityID(), defs)
      if not bb then return end
      r.upper = bb:GetInt(defs.UpperBody)
      r.vehicle = bb:GetInt(defs.Vehicle)
      r.workspot = bb:GetInt(defs.IsInWorkspot)
      r.high = bb:GetInt(defs.HighLevel)
      r.swim = bb:GetInt(defs.Swimming)
      r.loco = bb:GetInt(defs.LocomotionDetailed)
      r.takedown = bb:GetInt(defs.Takedown)
      r.vision = bb:GetInt(defs.Vision)
      r.melee = bb:GetInt(defs.MeleeWeapon)
      r.carrying = bb:GetInt(defs.BodyCarrying)
      r.mounted = bb:GetBool(defs.MountedToVehicle)
    end)
    return r
  end

  -- state for the safe moment of an animation pack change (M.PackBlockReason); only read while a change waits
  local function packObservation(player, menu)
    local p = psm(player)
    local o = { menu = menu, unreadable = p.loco == nil, upper = p.upper, loco = p.loco }
    local high = p.high or 0
    o.vehicle = (p.vehicle or 0) ~= 0 or p.mounted == true
    o.workspot = (p.workspot or 0) ~= 0
    o.scene = high > 2 and high ~= 6
    o.swimming = (p.swim or 0) ~= 0 or high == 6
    o.takedown = (p.takedown or 0) ~= 0
    o.carrying = (p.carrying or 0) ~= 0
    o.melee = (p.melee or 0) >= 5 and (p.melee or 0) <= 21
    o.aim = p.upper == 6
    local okR, active = pcall(function() local g = M.PackModule("utpp_reload_gate"); return g ~= nil and g.state ~= nil and g.state.active end)
    o.reload = okR and active == true
    return o
  end

  local function inMenu()
    local ok, v = pcall(function()
      local d = GetAllBlackboardDefs().UI_System
      return Game.GetBlackboardSystem():Get(d):GetBool(d.IsInMenu)
    end)
    if ok and v then return true end
    local okP, paused = pcall(function() return Game.GetSystemRequestsHandler():IsGamePaused() end)
    return okP and paused == true
  end

  local function entityKey(id)
    local ok, h = pcall(function() return tostring(id.hash) end)
    if ok and h ~= nil and h ~= "nil" then return h end
    return tostring(id)
  end
  local entityIds = {}

  local function lootActive() return M.Effective("lootView.impl") end

  local function onLootActivation(self, evt)
    if not lootActive() then return end
    pcall(function()
      if not evt.activator or not evt.activator:IsPlayer() then return end
      local id = self:GetEntityID()
      local key = entityKey(id)
      entityIds[key] = id
      local layer = Game.NameToString(evt.layerData.tag)
      if EnumInt(evt.eventType) == 0 then
        LA.Activate(M.la.tracker, key, vec(self:GetWorldPosition()), layer, clock)
      else
        LA.Deactivate(M.la.tracker, key, layer)
      end
    end)
  end

  local headSys, lastHeadOk = nil, true
  local camState = {}

  -- DLL head look layer (Game.UTPPSetHeadLook, src/mm/HeadLook.hpp). The decision (when / where) stays in HL; every tick while the look is
  -- on the raw target goes to the DLL (the tick is the heartbeat: the DLL treats a target older than 0.25 s as none), weight 0 releases.
  -- Returns true when the layer can act now (rig resolved, motion matching on): then the game's own look-at request is not needed.
  -- Missing native (older DLL): the call errors once, native = false for the session and the game request stays the only path.
  local HEAD_MODE = { camera = 0, face = 1, loot = 2 }
  local lastStatusLog = -1e9
  local function nativeLook(on, raw, mode)
    local c = M.hl
    if c.native == false then return false end
    if not on and not c.nativeSent then return false end
    raw = raw or { x = 0, y = 0, z = 0 }
    local ok, res = pcall(function()
      return Game.UTPPSetHeadLook(Vector4.new(raw.x, raw.y, raw.z, 1), on and 1.0 or 0.0, HEAD_MODE[mode] or 0,
        M.Get("headLook.chest") or 0.5, c.cfg.maxYaw or 100)
    end)
    if not ok then
      c.native = false
      c.nativeSent = false
      M.Log("head look: UTPPSetHeadLook unavailable (" .. tostring(res) .. ") - game look-at request only")
      return false
    end
    c.native = true
    c.nativeSent = on
    return on and res == true
  end

  local function gameApply(target, mode)
    headSys = headSys or getSystem("UTPPHeadLook")
    if not headSys then M.LogOnce("nohead", "UTPPHeadLook system missing (utpp.reds 0.5.0 not installed?) - head look inactive"); return end
    local ok, err = pcall(function()
      local chest = M.Get("headLook.chest") or 0.5
      local applied = headSys:Apply(Vector4.new(target.x, target.y, target.z, 1), 0.3, chest, 0)
      -- refused by the game side (a ragdoll episode started between the observation and this call): the look is not active,
      -- so the state goes back to off and the next free step engages again (otherwise it would wait for a target move)
      if applied == false and M.hl then M.hl.state, M.hl.target, M.hl.freeSince = "off", nil, nil
      else M.hl.gameApplied, M.hl.gameStamp = true, clock end
    end)
    if not ok and lastHeadOk then M.Log("head look call FAILED: " .. tostring(err)) end
    lastHeadOk = ok
  end

  local function gameClear()
    headSys = headSys or getSystem("UTPPHeadLook")
    if not headSys then M.LogOnce("nohead", "UTPPHeadLook system missing (utpp.reds 0.5.0 not installed?) - head look inactive"); return end
    local ok, err = pcall(function() headSys:Clear() end)
    M.hl.gameApplied = false
    if not ok and lastHeadOk then M.Log("head look call FAILED: " .. tostring(err)) end
    lastHeadOk = ok
  end

  local function headCmd(cmd)
    if not cmd then return end
    if cmd.op == "apply" then
      if not M.hl.nativeTook and M.hl.gameStamp ~= clock then gameApply(cmd.target, cmd.mode) end
    else
      M.hl.nativeTook = false
      nativeLook(false)
      gameClear()
    end
    local line = string.format("head look %s (%s)", cmd.op, tostring(cmd.mode or cmd.reason))
    if lastLog.head ~= line then lastLog.head = line; M.Log(string.format("t=%.2f %s", clock, line)) end
  end

  -- every tick after HL.Step while the look is on: live target to the DLL layer; the game request is the fallback (and is removed once
  -- the layer takes over, so that the two do not both turn the eyes)
  local function headTick(obs)
    local c = M.hl
    if c.state ~= "on" then
      if c.nativeSent then c.nativeTook = false; nativeLook(false) end -- released without a command (the game side refused): release the layer too
      return
    end
    local target, mode, _, raw = HL.Target(c, obs)
    local took = nativeLook(true, raw, mode)
    if took ~= c.nativeTook then
      c.nativeTook = took
      M.Log(string.format("t=%.2f head look layer %s", clock, took and "in the DLL (game look-at request not used)" or "cannot act - game look-at request"))
    end
    if took then
      if c.gameApplied then gameClear() end
      if M.diag and M.diag.mode and M.diag.mode ~= "off" and clock - lastStatusLog >= 5 then -- only while the diagnostic logs are on
        lastStatusLog = clock
        local okS, st = pcall(function() return Game.UTPPHeadLookStatus() end)
        if okS and st then M.Log("head look status: " .. tostring(st)) end
      end
    elseif not c.gameApplied then
      gameApply(target, mode)
    end
  end

  -- Loot camera component: the UTPP camera (utppCamera) when present, otherwise the player's
  -- active game camera component (GetFPPCameraComponent / "camera"). Returns comp, source ("utpp_camera"|"game"|"none").
  local lootCamSource = nil
  local function lootCameraComponent(player)
    local okU, c = pcall(function() return player:FindComponentByName("utppCamera") end)
    if okU and c ~= nil then return c, "utpp_camera" end
    local okG, g = pcall(function() return player:GetFPPCameraComponent() end)
    if okG and g ~= nil then return g, "game" end
    local okN, n = pcall(function() return player:FindComponentByName("camera") end)
    if okN and n ~= nil then return n, "game" end
    return nil, "none"
  end

  local function applyLootCamera(player, offset)
    local ok, err = pcall(function()
      local comp, src = lootCameraComponent(player)
      if src ~= lootCamSource then
        lootCamSource = src
        M.Log("loot camera source: " .. src)
      end
      if comp == nil then camState = {}; return end
      if offset <= 1e-4 and not camState.written then return end
      local cur = vec(comp:GetLocalPosition())
      local q = comp:GetLocalOrientation()
      local fwd = LA.QuatForward({ i = q.i, j = q.j, k = q.k, r = q.r })
      local out, base = LA.ApplyOffset(camState, cur, fwd, offset)
      comp:SetLocalPosition(Vector4.new(out.x, out.y, out.z, 1))
      if offset <= 1e-4 then
        comp:SetLocalPosition(Vector4.new(base.x, base.y, base.z, 1))
        camState = {}
      end
    end)
    if not ok then M.LogOnce("lootcam", "loot camera offset FAILED: " .. tostring(err)) end
  end

  registerForEvent("onInit", function()
    if announced then return end
    announced = true
    M.Log(string.format("onInit %s: settings %s, headLook=%s lootAssist=%s", M.VERSION,
      tostring(next(S.stored) ~= nil and "loaded" or "defaults"),
      tostring(M.Effective("headLook.impl")), tostring(M.Effective("lootView.impl"))))
    -- motion matching switches (applied now: play needs MM without console commands)
    pcall(M.SetupMMSettings, function(on) return Game.UTPPSetMMEnabled(on) end, function(on) return Game.UTPPSetMMTrace(on) end,
      { setFootReplant = function(mask) return Game.UTPPSetMMFootReplant(mask) end,
        setFeedDump = function(on) return Game.UTPPSetMMFeedDump(on) end,
        setIdleHold = function(on) return Game.UTPPSetMMIdleHold(on) end,
        setAimUpperFade = function(on) return Game.UTPPSetMMAimUpperFade(on) end,
        setWeaponUpper = function(on) return Game.UTPPSetMMWeaponUpper(on) end,
        setMeleeFullBody = function(on) return Game.UTPPSetMeleeFullBodySetting(on) end,
        setMeleeAimParams = function(up, down, hz) return Game.UTPPSetMeleeAimParams(up, down, hz) end,
        setMeleeGuard = function(on) return Game.UTPPSetMeleeGuardSetting(on) end,
        setMeleeGuardFullBody = function(on) return Game.UTPPSetMeleeGuardFullSetting(on) end,
        setMeleeGuardAimTilt = function(on) return Game.UTPPSetMeleeGuardAimTiltSetting(on) end,
        setMotionGrip = function(on) return Game.UTPPSetMotionGrip(on) end,
        setMotionShootTransfer = function(on) return Game.UTPPSetMotionShootTransfer(on) end,
        setMotionMagTap = function(on) return Game.UTPPSetMotionMagTap(on) end,
        setKneeFix = function(mask) return Game.UTPPSetMMKneeFix(mask) end,
        setHeadLead = function(on) return Game.UTPPSetMMHeadLead(on) end,
        setAir = function(mode) return Game.UTPPSetMMAir(mode) end,
        setAirProbe = function(on) return Game.UTPPSetMMAirProbe(on) end,
        setAirSteer = function(on) return Game.UTPPSetMMAirSteer(on) end,
        setAirRobust = function(on) return Game.UTPPSetMMAirRobust(on) end,
        setClimbRelease = function(on) return Game.UTPPSetMMClimbRelease(on) end })
    -- settings menu: the Mod Settings screen when the framework lists the UTPP options, otherwise the Native Settings tab
    -- (not confirmed here = the tab is built now and removed when ModSettingsTick confirms Mod Settings later)
    local okMS, hasMS = pcall(M.ModSettingsPresent)
    if not (okMS and hasMS) then
      pcall(function() M.BuildNativeSettings(GetMod and GetMod("nativeSettings") or nil) end)
    end
    for _, cls in ipairs({ "gameLootObject", "gameLootContainerBase" }) do
      local observe = ObserveAfter or Observe
      local ok, err = pcall(observe, cls, "OnInteractionActivated", onLootActivation)
      if not ok then M.Log("loot observer " .. cls .. " FAILED: " .. tostring(err)) end
    end
  end)

  registerForEvent("onShutdown", function()
    -- game exit: player gone -> the cached UTPPHeadLook system must not be called, only the Lua state is dropped
    local okP, p = pcall(Game.GetPlayer)
    local alive = okP and p ~= nil and (pcall(function() return p:GetEntityID() end))
    if not alive then
      M.Log("[utpp_core] shutdown: player gone - game calls skipped")
      headSys, inGame = nil, false
      return
    end
    pcall(function() if M.hl.nativeSent then Game.UTPPSetHeadLook(Vector4.new(0, 0, 0, 1), 0.0, 0, 0.5, 100) end end)
    pcall(function() if headSys then headSys:Clear() end end)
  end)

  registerForEvent("onUpdate", function(delta)
    local dt = math.max(delta or 0, 0)
    clock = clock + dt
    pcall(M.PollDatabase, dt)
    pcall(M.MenuTick)
    pcall(M.ModSettingsTick, dt)
    pcall(M.NightTick, dt)
    local okP, player = pcall(function() return Game.GetPlayer() end)
    if not okP or player == nil then
      inGame = false; M.xhair.live = false
      M.pack.inSession = false
      pcall(M.DiagTick, dt, false)
      pcall(M.PackTick, dt, nil) -- no player: a waiting animation pack change has nothing to wait for
      return
    end
    local newSession = not inGame
    if not inGame then
      inGame = true
    end
    M.pack.inSession = true
    -- the crosshair script system is per session: push the stored value at the session edge, then retry until acknowledged
    pcall(M.CrosshairResync, clock, newSession)
    local menu = inMenu()
    pcall(M.DiagTick, dt, not menu)
    pcall(M.PackSend, clock, newSession)
    if M.pack.pending then
      local okO, obs = pcall(packObservation, player, menu)
      pcall(M.PackTick, dt, okO and obs or { unreadable = true })
    end
    local hlOn = M.Effective("headLook.impl")
    local laOn = lootActive()
    if not hlOn and M.hl.state == "on" then headCmd(HL.Release(M.hl, "disabled")) end
    if not hlOn and not laOn then
      if camState.written then applyLootCamera(player, 0) end
      return
    end
    if menu then return end
    local ok, err = pcall(function()
      local camSys = Game.GetCameraSystem()
      local camPos = cameraPos(camSys)
      local cf = camSys:GetActiveCameraForward()
      local pos = vec(player:GetWorldPosition())
      local f = player:GetWorldForward()
      local v = player:GetVelocity()
      local speed = math.sqrt((v.x or 0) ^ 2 + (v.y or 0) ^ 2)
      local tpp = camPos ~= nil and math.sqrt((camPos.x - pos.x) ^ 2 + (camPos.y - pos.y) ^ 2
        + (camPos.z - pos.z - HL.DEFAULTS.eyeHeight) ^ 2) > 0.6
      local p = psm(player)
      local loco = p.loco or 0
      local obs = {
        tpp = tpp, speed = speed, pos = pos, fwd = { x = f.x, y = f.y }, camPos = camPos or pos,
        camFwd = { x = cf.x, y = cf.y, z = cf.z },
        aim = p.upper == 6, vehicle = (p.vehicle or 0) ~= 0, workspot = (p.workspot or 0) ~= 0,
        scene = (p.high or 0) >= 2 and (p.high or 0) <= 5, swimming = (p.swim or 0) ~= 0 or p.high == 6,
        takedown = (p.takedown or 0) ~= 0, scanner = p.vision == 1,
        ragdoll = (function() local okR, busy = pcall(function() return Game.UTPPRagdollBusy() end); return okR and busy == true end)(),
        locoBlocked = (loco >= 5 and loco <= 13) or loco == 29 or loco == 31,
      }
      -- loot
      local offset = 0
      if laOn then
        if math.floor(clock * 5) ~= math.floor((clock - dt) * 5) then -- 5 Hz crosshair poll
          pcall(function()
            local obj = Game.GetTargetingSystem():GetLookAtObject(player, false, false)
            if obj and (obj:IsA("gameLootObject") or obj:IsA("gameLootContainerBase")) then
              local id = obj:GetEntityID()
              local key = entityKey(id)
              entityIds[key] = id
              LA.Activate(M.la.tracker, key, vec(obj:GetWorldPosition()), "lookat", clock, M.la.cfg.lookTtl)
            end
          end)
          LA.Prune(M.la.tracker, clock, pos, M.la.cfg, function(key)
            local id = entityIds[key]
            if not id then return nil end
            local okE, ent = pcall(Game.FindEntityByID, id)
            if not okE or ent == nil then entityIds[key] = nil; return nil end
            return vec(ent:GetWorldPosition())
          end)
        end
        local blocked = (obs.aim and "aim") or (obs.vehicle and "vehicle") or (obs.workspot and "workspot")
          or (obs.scene and "scene") or (obs.swimming and "swimming") or (obs.takedown and "takedown") or nil
        offset = LA.Step(M.la, clock, dt, { enabled = true, tpp = tpp, blocked = blocked, speed = speed, pos = pos,
          camPos = obs.camPos, camFwd = obs.camFwd })
        applyLootCamera(player, offset)
      elseif camState.written then
        M.la.weight, M.la.target = 0, nil
        applyLootCamera(player, 0)
      end
      -- head look
      if hlOn then
        obs.enabled = camPos ~= nil
        obs.loot = (laOn and M.la.target and M.la.weight > 0.2) and M.la.target.pos or nil
        local cmd = HL.Step(M.hl, clock, obs)
        headTick(obs) -- first: decides DLL layer or game request for this tick (headCmd only forwards retargets / the release)
        headCmd(cmd)
      end
    end)
    if not ok then
      W.errors = W.errors + 1
      if W.errors <= 3 then M.Log("update error: " .. tostring(err)) end
    end
  end)
end

return M

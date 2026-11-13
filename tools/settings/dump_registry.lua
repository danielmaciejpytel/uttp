-- Settings registry dump for tools/settings/gen_modsettings.py.
-- Boots the shipped CET modules (core, bodyturn, body, v2, camera, perspective, loothead, vehiclecam) offline with inert
-- natives, the same way tools/lua-tests/test_utpp_settings_projection.lua does, and returns a JSON document with the
-- standard menu projection of the registry: every setting and presentation view with its menu level, group, sort key,
-- label, description, type, default, range and choices. Nothing is written; settings files are redirected to a temp file.
-- Globals (set by the caller): UTPP_CORE_PATH, UTPP_BODYTURN_PATH, UTPP_BODY_PATH, UTPP_MOTION_PATH, UTPP_CAMERA_PATH,
-- UTPP_CAMERA_DIR, UTPP_PERSPECTIVE_PATH, UTPP_LOOTHEAD_PATH, UTPP_VEHICLECAM_PATH. Result: global UTPP_REGISTRY_JSON.

local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod,
                print = print, path = package.path, open = io.open }
local tmpFile = os.tmpname()
os.remove(tmpFile)
local realOpen = io.open
io.open = function(path, mode)
  if path == "utpp_settings.txt" then path = tmpFile end
  return realOpen(path, mode)
end
local quiet = function() end
local function inert() return true end
print = quiet
Game = setmetatable({}, { __index = function(_, name)
  if type(name) == "string" and string.sub(name, 1, 4) == "UTPP" then return inert end
  return nil
end })
registerForEvent, registerInput, GetMod = nil, nil, nil

local ok, err = pcall(function()
  local core = dofile(UTPP_CORE_PATH)
  core.sink = quiet
  core.SETTINGS_FILE = tmpFile
  local api = setmetatable({}, { __index = function() return inert end })
  core.SetupMMSettings(inert, inert, api)
  local bt = dofile(UTPP_BODYTURN_PATH)
  bt.log = quiet
  bt.RegisterSettings(core)
  dofile(UTPP_BODY_PATH).RegisterSettings(core)
  local v2 = dofile(UTPP_MOTION_PATH)
  v2.sink = quiet
  local glue = { IsEnabled = function() return false end, SetEnabled = quiet, SetPistolOneHand = quiet, SetPistolAuto = quiet }
  v2.api = setmetatable({ glue = function() return glue end, havePlayer = function() return true end },
    { __index = function() return inert end })
  v2.RegisterSettings(core)
  package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. SAVED.path
  registerForEvent, registerInput = quiet, quiet
  local cam = dofile(UTPP_CAMERA_PATH)
  cam.RegisterSettings(core)
  package.path = SAVED.path
  registerForEvent, registerInput = nil, nil
  dofile(UTPP_PERSPECTIVE_PATH).AttachCore(core)
  GetMod = function(name) if name == "utpp_core" then return core end return nil end
  local handlers = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  dofile(UTPP_LOOTHEAD_PATH)
  if handlers.onInit then handlers.onInit() end
  handlers = {}
  registerInput = quiet
  dofile(UTPP_VEHICLECAM_PATH)
  if handlers.onInit then pcall(handlers.onInit) end
  UTPP_REGISTRY_CORE, UTPP_REGISTRY_CAMERA = core, cam
end)
registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
print, package.path, io.open = SAVED.print, SAVED.path, SAVED.open
os.remove(tmpFile)
if not ok then error("registry boot failed: " .. tostring(err), 0) end

-- ---------------------------------------------------------------- JSON
local function jstr(s)
  s = string.gsub(tostring(s), '[%c"\\]', function(c)
    if c == '"' then return '\\"' elseif c == "\\" then return "\\\\" elseif c == "\n" then return "\\n"
    elseif c == "\t" then return "\\t" end
    return string.format("\\u%04x", string.byte(c))
  end)
  return '"' .. s .. '"'
end
local function jval(v)
  local t = type(v)
  if t == "nil" then return "null" elseif t == "boolean" then return tostring(v)
  elseif t == "number" then
    if v == math.floor(v) and math.abs(v) < 1e15 then return string.format("%d", v) end
    return string.format("%.17g", v)
  elseif t == "string" then return jstr(v)
  elseif t == "table" then
    if #v > 0 or next(v) == nil then
      local parts = {}
      for i = 1, #v do parts[i] = jval(v[i]) end
      return "[" .. table.concat(parts, ",") .. "]"
    end
    local keys = {}
    for k in pairs(v) do keys[#keys + 1] = k end
    table.sort(keys)
    local parts = {}
    for _, k in ipairs(keys) do parts[#parts + 1] = jstr(k) .. ":" .. jval(v[k]) end
    return "{" .. table.concat(parts, ",") .. "}"
  end
  return "null"
end

local core, cam = UTPP_REGISTRY_CORE, UTPP_REGISTRY_CAMERA
UTPP_REGISTRY_CORE, UTPP_REGISTRY_CAMERA = nil, nil
local S = core.settings
local orderIndex = {}
for i, k in ipairs(S.order) do orderIndex[k] = i end
local groupIndex = {}
for i, g in ipairs(core.MENU_GROUPS) do groupIndex[g.id] = i end

local diagKeys = {}
for _, k in ipairs(core.DIAG_KEYS or {}) do diagKeys[k] = true end
local settings = {}
for _, key in ipairs(S.order) do
  local def = S.defs[key]
  local level, group, rank, source = core.MenuPlacement(key)
  local l = core.MENU_LAYOUT[key]
  local choices = nil
  if def.choices then
    choices = {}
    for i, c in ipairs(def.choices) do choices[i] = { c[1], c[2] } end
  end
  settings[#settings + 1] = {
    key = key, section = def.section, type = def.type, label = (l and l.label) or def.label, desc = def.desc or "",
    default = def.default, min = def.min, max = def.max, step = def.step, choices = choices,
    level = level, group = group, rank = rank, order = orderIndex[key], source = source, hidden = def.hidden == true,
    -- overrides (informational; the generator ignores them): value of a fresh install other than default (def.initial), value
    -- forced while the Game Animations pack is in force, option text note of that pack, diagnostic switch held on by the
    -- Diagnostic logs list, presentation label of the standard menu, switch kind and format
    initial = def.initial, packForced = core.PACK_FORCED.game and core.PACK_FORCED.game[key], packDependent = core.PACK_DEPENDENT[key] == true,
    diagHeld = diagKeys[key] == true, layoutLabel = l and l.label or nil, onOff = def.onOff == true, hold = def.hold == true,
    format = def.format, hasApply = type(def.apply) == "function", owner = (S.sections[def.section] or {}).owner,
  }
end
local views = {}
for i, id in ipairs(core.views.order) do
  local v = core.views.byId[id]
  views[#views + 1] = { id = id, type = v.type, label = v.label, desc = v.desc or "", min = v.min, max = v.max,
                        step = v.step, level = v.level, group = v.group, rank = v.rank, index = v.index or i }
end
local groups = {}
for i, g in ipairs(core.MENU_GROUPS) do groups[i] = { g.id, g.label } end
local sections = {}
for i, id in ipairs(S.sectionOrder) do sections[i] = { id, S.sections[id].label } end

UTPP_REGISTRY_JSON = jval({
  coreVersion = core.VERSION, groups = groups, sections = sections, settings = settings, views = views,
  cameraProfile = cam.FIXED_PROFILE or (cam.rt and cam.rt.profileIndex) or nil,
})

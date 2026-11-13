-- UTPP S3 camera framing editor offline tests (utpp_camera RegisterSettings views "camera.edit.<field>" on the utpp_core view
-- API, real core + camera + camera_math, mocked Native Settings whose setOption calls the widget callback synchronously and
-- returns early on an unchanged value, like the real library).
-- 2026-10-23 single fixed camera profile (utpp_camera FIXED_PROFILE = 2): the "Active profile" selector and profile
-- switching no longer exist (their old checks are replaced by checks that every switch request keeps slot 2).
-- T03 editing the fixed profile: exactly one camera.p2.* key per edit (diff of all values), one Save; key / console /
--     camera.profile requests keep slot 2 and change no setting; reset re-syncs the editor.
-- T04 compatibility: next / prev always return slot 2; aim / unarmed zoom outputs with ignored switches and an editor edit
--     identical to a run without Native Settings; roundtrip of all 25 fields (storage format kept), a stored
--     camera.profile=4 / an old file with camera.profile=3 is ignored without error; view limits / step / format = the raw
--     fields; no Save when the menu is built; the view "all" and a core without RegisterView show profile 2's raw fields only.
-- Globals from the runner (test_utpp_camera_run.py): UTPP_CORE_PATH, UTPP_CAMERA_PATH, UTPP_CAMERA_DIR, UTPP_CAMERA_MATH_PATH.
local passed, failed = 0, 0
local outPrint = print
local function check(name, ok, detail)
  if ok then passed = passed + 1 else failed = failed + 1 end
  outPrint(string.format("[%s] utpp_profile_editor: %s%s", ok and "PASS" or "FAIL", name,
    (not ok and detail ~= nil) and (" -- " .. tostring(detail)) or ""))
end

-- globals touched here are restored at the end (a runner may share one Lua state)
local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod,
                print = print, path = package.path, open = io.open }
local function restoreGlobals()
  registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
  print, package.path, io.open = SAVED.print, SAVED.path, SAVED.open
end

package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. package.path
local CM = dofile(UTPP_CAMERA_MATH_PATH)

-- CET mods open "utpp_settings.txt" relative to their folder: offline that goes to a temp file (content set per boot)
local tmpMod = os.tmpname()
local tmpSettings = os.tmpname()
local realOpen = io.open
local function readFile(p)
  local f = realOpen(p, "rb")
  if not f then return nil end
  local s = f:read("*a")
  f:close()
  return s
end
local function writeFile(p, s)
  local f = realOpen(p, "wb")
  f:write(s)
  f:close()
end

-- ---------------------------------------------------------------- Native Settings mock
local function newNS()
  local ns = { data = {}, errors = {}, setCalls = 0, restore = nil }
  local function split(path) return string.match(path, "^/([^/]+)/([^/]+)$") end
  function ns.pathExists(path)
    local tab, sub = split(path)
    if tab then return ns.data[tab] ~= nil and ns.data[tab].subcategories[sub] ~= nil end
    tab = string.match(path, "^/([^/]+)$")
    return tab ~= nil and ns.data[tab] ~= nil
  end
  function ns.addTab(path, label, closed) ns.data[string.sub(path, 2)] = { label = label, keys = {}, subcategories = {}, closed = closed } end
  function ns.addSubcategory(path, label, idx)
    local tab, sub = split(path)
    local t = tab and ns.data[tab]
    if not t then ns.errors[#ns.errors + 1] = "addSubcategory without tab " .. path; return end
    t.subcategories[sub] = { label = label, options = {} }
    table.insert(t.keys, idx or table.maxn(t.keys) + 1, sub)
  end
  function ns.removeSubcategory(path)
    local tab, sub = split(path)
    local t = tab and ns.data[tab]
    if not t or not t.subcategories[sub] then return end
    t.subcategories[sub] = nil
    for i = 1, table.maxn(t.keys) do if t.keys[i] == sub then t.keys[i] = nil end end
  end
  local function add(kind, path, label, desc, fields, cb, idx)
    local tab, sub = split(path)
    local s = tab and ns.data[tab] and ns.data[tab].subcategories[sub]
    if not s then ns.errors[#ns.errors + 1] = "widget without subcategory " .. path; return nil end
    local w = { kind = kind, path = path, label = label, desc = desc, cb = cb }
    for k, v in pairs(fields) do w[k] = v end
    if idx ~= nil and (type(idx) ~= "number" or idx < 1 or idx > #s.options + 1) then
      ns.errors[#ns.errors + 1] = "option index out of range " .. path .. " " .. tostring(label)
      idx = nil
    end
    table.insert(s.options, idx or #s.options + 1, w)
    return w
  end
  function ns.addSwitch(path, label, desc, cur, dflt, cb, idx) return add("switch", path, label, desc, { state = cur, default = dflt }, cb, idx) end
  function ns.addRangeInt(path, label, desc, min, max, step, cur, dflt, cb, idx)
    return add("int", path, label, desc, { min = min, max = max, step = step, value = cur, default = dflt }, cb, idx)
  end
  function ns.addRangeFloat(path, label, desc, min, max, step, format, cur, dflt, cb, idx)
    return add("float", path, label, desc, { min = min, max = max, step = step, format = format, value = cur, default = dflt }, cb, idx)
  end
  function ns.addSelectorString(path, label, desc, elements, cur, dflt, cb, idx)
    return add("selector", path, label, desc, { elements = elements, value = cur, default = dflt }, cb, idx)
  end
  function ns.addButton(path, label, desc, text, size, cb, idx) return add("button", path, label, desc, { text = text }, cb, idx) end
  function ns.addKeyBinding(path, label, desc, value, dflt, hold, cb, idx)
    return add("key", path, label, desc, { value = value, default = dflt, hold = hold }, cb, idx)
  end
  function ns.registerRestoreDefaultsCallback(path, override, cb) ns.restore = { path = path, override = override, cb = cb } end
  function ns.list()
    local t, out = ns.data.utpp, {}
    if not t then return out end
    for i = 1, table.maxn(t.keys) do
      local sub = t.keys[i]
      if sub then for _, w in ipairs(t.subcategories[sub].options) do out[#out + 1] = { sub = sub, w = w } end end
    end
    return out
  end
  function ns.find(label)
    for _, e in ipairs(ns.list()) do if e.w.label == label then return e.w, e.sub end end
    return nil
  end
  function ns.count(pattern)
    local n = 0
    for _, e in ipairs(ns.list()) do if string.find(e.w.label, pattern) then n = n + 1 end end
    return n
  end
  function ns.setOption(w, v)
    ns.setCalls = ns.setCalls + 1
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
  -- the player moves a widget: the controller stores the value, then calls the callback
  function ns.userChange(w, v)
    if w.kind == "switch" then w.state = v elseif w.kind ~= "button" then w.value = v end
    w.cb(v)
  end
  return ns
end

-- ---------------------------------------------------------------- boot: real core + camera (no CET adapter)
-- opts.stored = settings file content for the boot; opts.noView = a core without RegisterView / RefreshView;
-- opts.ns = false (no Native Settings) | nil (build); opts.view = "all" | nil
local function boot(opts)
  opts = opts or {}
  local quiet = function() end
  print = quiet
  registerForEvent, registerInput, Game, GetMod = nil, nil, nil, nil
  writeFile(tmpMod, opts.stored or "")
  io.open = function(path, mode)
    if path == "utpp_settings.txt" then path = tmpMod end
    return realOpen(path, mode)
  end
  local core = dofile(UTPP_CORE_PATH)
  core.sink = quiet
  core.SETTINGS_FILE = tmpSettings
  os.remove(tmpSettings)
  if opts.noView then core.RegisterView, core.RefreshView = nil, nil end
  local B = { core = core, saves = 0, lines = {} }
  core.sink = function(l) B.lines[#B.lines + 1] = l end
  local origSave = core.Save
  core.Save = function(...) B.saves = B.saves + 1; return origSave(...) end
  local cam = dofile(UTPP_CAMERA_PATH)
  cam.sink = function(l) B.lines[#B.lines + 1] = l end
  cam.RegisterSettings(core)
  B.cam = cam
  if opts.view then core.SetMenuView(opts.view) end
  if opts.ns ~= false then
    B.ns = newNS()
    B.savesBeforeBuild = B.saves
    core.BuildNativeSettings(B.ns)
  end
  io.open = realOpen
  print = SAVED.print
  return B
end
local function snapshot(core)
  local t = {}
  for _, k in ipairs(core.settings.order) do t[k] = core.settings.values[k] end
  return t
end
local function diff(a, b)
  local out = {}
  for k, v in pairs(b) do if a[k] ~= v then out[#out + 1] = k end end
  for k in pairs(a) do if b[k] == nil then out[#out + 1] = k end end
  table.sort(out)
  return out
end
local FIELDS = CM.PROFILE_FIELDS
-- 2026-10-23 single fixed camera profile: four framing views (no sensitivity view), labels without a selector prefix
local VIEW_FIELDS = { "distance", "side", "fov", "height" }
local LABEL = { distance = "Distance (m)", side = "Shoulder offset (m)", fov = "FOV", height = "Height (m)" }
local function editor(B)
  local w = {}
  for _, f in ipairs(VIEW_FIELDS) do w[f] = B.ns.find(LABEL[f]) end
  return w
end
local function showsSlot(B, i)
  local w = editor(B)
  for _, f in ipairs(VIEW_FIELDS) do
    if not w[f] or w[f].value ~= B.cam.profiles[i][f] then return false, f end
  end
  return true
end
local function key(i, f)
  if i == 2 then return "camera." .. (f == "sens" and "framingSens" or f) end
  return string.format("camera.p%d.%s", i, f)
end

local okAll, errAll = pcall(function()
  -- ================================================================ layout of the framing views
  do
    local B = boot()
    check("fixed profile: FIXED_PROFILE = 2 and the active slot is 2", B.cam.FIXED_PROFILE == 2 and B.cam.rt.profileIndex == 2)
    local w = editor(B)
    local all = true
    for _, f in ipairs(VIEW_FIELDS) do all = all and w[f] ~= nil end
    check("editor: four framing views in the standard view (no sensitivity view)", all and B.ns.find("Sensitivity") == nil
      and B.ns.count("sensitivity multiplier") == 0)
    check("editor: no 'Active profile' selector or prefix anywhere", B.ns.find("Active profile") == nil
      and B.ns.count("^Active profile") == 0)
    local list = B.ns.list()
    local idx = {}
    for i, e in ipairs(list) do idx[e.w.label] = { i = i, sub = e.sub } end
    local first, orbit = nil, idx["Orbit sensitivity"]
    for i, e in ipairs(list) do if e.sub == "g_frame" then first = first or i end end
    check("editor: distance / shoulder / FOV / height open the framing group, then Orbit sensitivity and the inverts",
      orbit ~= nil and first ~= nil and idx[LABEL.distance].i == first and idx[LABEL.side].i == first + 1
      and idx[LABEL.fov].i == first + 2 and idx[LABEL.height].i == first + 3 and orbit.i == first + 4
      and idx["Invert horizontal orbit"].i == first + 5 and idx["Invert vertical orbit"].i == first + 6
      and idx[LABEL.height].sub == "g_frame" and orbit.sub == "g_frame",
      first and string.format("first %d / %s %s %s %s orbit %s", first, tostring(idx[LABEL.distance] and idx[LABEL.distance].i),
        tostring(idx[LABEL.side] and idx[LABEL.side].i), tostring(idx[LABEL.fov] and idx[LABEL.fov].i),
        tostring(idx[LABEL.height] and idx[LABEL.height].i), tostring(orbit and orbit.i)))
    local descOk = true
    for _, f in ipairs(VIEW_FIELDS) do
      if not w[f] or string.find(w[f].desc or "", "profile", 1, true) or (w[f].desc or "") == "" then descOk = false end
    end
    check("editor: descriptions present and without any profile selector reference", descOk)
    local profKeys = {}
    for _, e in ipairs(list) do
      if string.find(e.w.label, "profile", 1, true) or string.find(e.w.label, "Profile", 1, true) then profKeys[#profKeys + 1] = e.w.label end
    end
    check("standard view: no profile selector / rotation / blend / profile key widgets", #profKeys == 0, table.concat(profKeys, ", "))
    check("editor: no raw camera.pN widgets in the standard view", B.ns.count("^%d [%a]+: ") == 0
      and B.ns.count("^Camera placement: ") == 0)
    -- limits / step / format = the raw fields of the fixed profile
    local same = true
    for _, f in ipairs(VIEW_FIELDS) do
      local d = B.core.settings.defs[key(2, f)]
      local fmt = (d.step >= 1) and "%.0f" or "%.2f"
      if not (w[f].kind == "float" and w[f].min == d.min and w[f].max == d.max and w[f].step == d.step and w[f].format == fmt
          and d.min == CM.PROFILE_LIMITS[f][1] and d.max == CM.PROFILE_LIMITS[f][2] and d.step == CM.PROFILE_LIMITS[f][3]) then
        same = false
      end
    end
    check("T04: view limits / step / format = the raw fields = CM.PROFILE_LIMITS (same units)", same)
    check("editor: shows the fixed profile (2) values, defaults = default profile 2 (2.75 m / 0.50 m / FOV 55 / -0.35 m)",
      showsSlot(B, 2) and w.distance.default == CM.DEFAULT_PROFILES[2].distance and w.fov.default == CM.DEFAULT_PROFILES[2].fov
      and w.side.default == CM.DEFAULT_PROFILES[2].side and w.height.default == CM.DEFAULT_PROFILES[2].height
      and w.distance.value == 2.75 and w.side.value == 0.5 and w.fov.value == 55 and w.height.value == -0.35)
    check("T04: building the menu saves nothing", B.saves == B.savesBeforeBuild and B.saves == 0 and readFile(tmpSettings) == nil,
      B.saves)
    check("editor: no new settings key (views are not settings)", B.core.settings.defs["camera.edit.distance"] == nil
      and B.core.settings.values["camera.edit.distance"] == nil)
    check("editor: Native Settings calls valid", #B.ns.errors == 0, table.concat(B.ns.errors, "; "))
    -- storage keys kept, without a widget section (old settings files load and save unchanged)
    local regOk = true
    for _, k in ipairs({ "camera.profile", "camera.profileCount", "camera.profileBlend" }) do
      local d = B.core.settings.defs[k]
      if d ~= nil then regOk = false end
    end
    for i = 1, 5 do
      for _, f in ipairs(FIELDS) do
        local d = B.core.settings.defs[key(i, f)]
        if (i == 2) ~= (d ~= nil and d.section == "camera_placement") then regOk = false end
      end
    end
    check("registry: only the fixed slot's 5 camera fields are registered (with a section); camera.profile / profileCount / profileBlend and the other slots are not",
      regOk)
    -- view all: only the fixed profile's raw fields, no editor views
    local A = boot({ view = "all" })
    check("view all: the 5 raw fields of profile 2 only, no other slot, no editor views, no profile selector",
      A.ns.count("^Camera placement: ") == 5 and A.ns.count("^%d [%a]+: ") == 0 and A.ns.find(LABEL.distance) == nil
      and A.ns.find("Active profile") == nil and A.core.settings.widgets["camera.p3.fov"] == nil
      and A.core.settings.widgets["camera.fov"] ~= nil and A.core.settings.widgets["camera.profile"] == nil)
  end

  -- ================================================================ T03 edits hit exactly the fixed slot
  do
    local B = boot()
    local core, cam, ns = B.core, B.cam, B.ns
    local w = editor(B)
    local s0 = snapshot(core)
    local saves0 = B.saves
    ns.userChange(w.distance, 3.4)
    local d1 = diff(s0, snapshot(core))
    check("T03: editor distance changes exactly camera.distance, one Save", #d1 == 1 and d1[1] == "camera.distance"
      and core.Get("camera.distance") == 3.4 and cam.profiles[2].distance == 3.4 and B.saves == saves0 + 1, table.concat(d1, ","))
    local h2 = core.Get("camera.height")
    local s1 = snapshot(core)
    ns.userChange(w.height, 0.4)
    local d2 = diff(s1, snapshot(core))
    check("T03: height edit -> camera.height only", #d2 == 1 and d2[1] == "camera.height" and core.Get("camera.height") == 0.4
      and h2 ~= 0.4, table.concat(d2, ","))
    -- profile switching no longer exists: keys, console and a stored / console camera.profile keep slot 2
    local s2 = snapshot(core)
    cam.NextProfile(1)
    cam.NextProfile(-1)
    check("T03: key next / prev profile are no-ops (slot 2, editor unchanged, no setting changed)", cam.rt.profileIndex == 2
      and showsSlot(B, 2) and w.distance.value == 3.4 and #diff(s2, snapshot(core)) == 0)
    check("T03: console Profile(3) keeps the fixed slot 2", cam.Profile(3) == 2 and cam.rt.profileIndex == 2 and showsSlot(B, 2))
    core.Set("camera.profile", 4)
    check("T03: console Set camera.profile=4 is ignored (slot 2, editor shows p2)", cam.rt.profileIndex == 2 and showsSlot(B, 2)
      and w.distance.value == 3.4)
    -- console Set of the fixed slot's raw key re-syncs the editor field
    core.Set("camera.fov", 61)
    check("T03: console Set of the fixed slot -> editor FOV shows it", w.fov.value == 61)
    core.Set("camera.p4.fov", 40)
    check("T03: console Set of an unused slot -> editor unchanged", w.fov.value == 61)
    check("T03: Native Settings calls valid", #ns.errors == 0, table.concat(ns.errors, "; "))
    -- reset re-syncs the editor
    local ss = B.saves
    ns.restore.cb()
    check("T04: restore defaults (ResetAll) -> profile 2 defaults in the editor, one Save", cam.rt.profileIndex == 2
      and w.distance.value == CM.DEFAULT_PROFILES[2].distance and w.fov.value == CM.DEFAULT_PROFILES[2].fov
      and w.height.value == CM.DEFAULT_PROFILES[2].height and B.saves == ss + 1, string.format("%s %s", tostring(w.distance.value), B.saves - ss))
  end

  -- ================================================================ T04 next / prev: always the fixed slot
  do
    local function seq(withNS, count)
      local B = boot(withNS and {} or { ns = false })
      B.core.Set("camera.profileCount", count)
      local out = {}
      for _ = 1, 7 do out[#out + 1] = B.cam.NextProfile(1) end
      for _ = 1, 9 do out[#out + 1] = B.cam.NextProfile(-1) end
      return table.concat(out, ",")
    end
    local expect = string.rep("2,", 15) .. "2"
    for _, n in ipairs({ 2, 3, 5 }) do
      local a, b = seq(true, n), seq(false, n)
      check("T04: next / prev with 'profiles in rotation' " .. n .. ": always slot 2, with and without the editor", a == b and a == expect,
        a .. " | " .. b)
    end
  end

  -- ================================================================ T04 blend / aim / unarmed zoom outputs unchanged
  do
    local function run(withNS)
      local B = boot(withNS and {} or { ns = false })
      local cam, core = B.cam, B.core
      local dt = 1 / 60
      local rows = {}
      local ao = { enabled = true, aimingSetting = "tpp", tpp = true, upperBody = 6, weaponRanged = true }
      local aimOff = { enabled = true, aimingSetting = "tpp", tpp = true, upperBody = 0, weaponRanged = true }
      local function rowOf(i)
        local p = CM.aimCompose(CM.blendStep(cam.blend, dt), cam.aim, cam.cfg, nil)
        p = CM.zoomUnarmedCompose(p, cam.zoomU)
        local b = cam.blend
        rows[#rows + 1] = string.format("%d %.9f %.9f %.9f %.9f %.9f t=%.9f aw=%.9f zw=%.9f %s", i, p.distance, p.height, p.side,
          p.fov, p.sens, b.t, cam.aim.w, cam.zoomU.w, tostring(cam.rt.profileIndex))
      end
      for i = 1, 150 do
        if i == 10 then cam.Profile(3) end          -- ignored request (fixed slot)
        if i == 20 then -- edit of the fixed slot: editor (with NS) = one core.Set (without NS)
          if withNS then B.ns.userChange(editor(B).distance, 2.4) else core.Set("camera.distance", 2.4) end
        end
        if i == 40 then cam.NextProfile(1) end
        cam.AimUpdate((i >= 30 and i < 80) and ao or aimOff, dt) -- aim state across the ignored switch
        if i == 50 then cam.Profile(2) end
        if i >= 90 and i < 130 then CM.zoomUnarmedStep(cam.zoomU, cam.cfg, dt, true, "test", 0)
        elseif i >= 130 then CM.zoomUnarmedStep(cam.zoomU, cam.cfg, dt, false, "test", 0) end
        if i == 100 then cam.NextProfile(-1) end   -- ignored switch during the unarmed zoom
        rowOf(i)
      end
      return rows, B
    end
    local a, B = run(true)
    local b = run(false)
    local same, first = #a == #b, nil
    for i = 1, math.max(#a, #b) do if a[i] ~= b[i] then same = false; first = first or i end end
    check("T04: aim / unarmed zoom outputs with ignored switches and an editor edit = the run without Native Settings",
      same, first and (tostring(a[first]) .. " | " .. tostring(b[first])))
    local slot2 = true
    for _, r in ipairs(a) do if string.sub(r, -2) ~= " 2" then slot2 = false end end
    check("T04: the slot stays 2 in every frame, the edit reaches the output (distance 2.4 before the aim)", slot2
      and math.abs(tonumber(string.match(a[25], "^%d+ ([%d%.%-]+)")) - 2.4) < 1e-6, a[25])
    check("T04: that run had an aim and an unarmed zoom", tonumber(string.match(a[60], "aw=([%d%.]+)")) > 0.9
      and tonumber(string.match(a[120], "zw=([%d%.]+)")) > 0.5, a[60])
    check("T04: no errors in the run", #B.ns.errors == 0)
  end

  -- ================================================================ T04 roundtrip of all 25 fields, old camera.profile ignored
  do
    local B = boot()
    local core = B.core
    local want = {}
    for i = 2, 2 do
      for _, f in ipairs(FIELDS) do
        local lim = CM.PROFILE_LIMITS[f]
        local v = lim[1] + lim[3] * (3 + i * 2 + #f)
        if v > lim[2] then v = lim[2] end
        v = tonumber(string.format("%.4g", v)) -- a value the storage format (%.4g) keeps exactly
        want[key(i, f)] = core.Set(key(i, f), v)
      end
    end
    core.Set("camera.profile", 4)
    local saved = readFile(tmpSettings)
    local F = boot({ stored = saved })
    local same, bad = true, nil
    for k, v in pairs(want) do
      if F.core.Get(k) ~= v or F.core.Get(k) ~= B.core.Get(k) then same = false; bad = k end
    end
    check("T04: roundtrip of all 25 fields Save -> Load in a fresh core identical (storage format kept)", same and saved ~= nil
      and F.core.Get("camera.profile") == nil and string.find(saved, "camera.profile=4", 1, true) == nil, bad)
    check("T04: a stored camera.profile=4 is ignored: the fresh core uses slot 2, the editor shows p2",
      F.cam.rt.profileIndex == 2 and showsSlot(F, 2) and editor(F).distance.value == want[key(2, "distance")])
    check("T04: loading + building the menu wrote nothing (no Save, no settings file, the loaded file unchanged)",
      F.saves == 0 and readFile(tmpSettings) == nil and readFile(tmpMod) == saved)
  end

  -- ================================================================ old settings file with camera.profile=3
  do
    local old = "# UTPP settings (old)\ncamera.profile=3\ncamera.profileCount=5\ncamera.profileBlend=1\ncamera.p3.distance=1.5\n"
      .. "camera.distance=3.1\nbind.profile_next=IK_F9\nbind.enabled=true\n"
    local O = boot({ stored = old })
    local failedLine = false
    for _, l in ipairs(O.lines) do if string.find(l, "FAILED", 1, true) or string.find(l, "failed", 1, true) then failedLine = true end end
    check("old settings camera.profile=3: loads without error, slot 2, the editor shows p2 (3.1 m), no FAILED line",
      O.cam.rt.profileIndex == 2 and editor(O).distance.value == 3.1 and O.core.Get("camera.profile") == nil
      and O.core.Get("camera.p3.distance") == nil and #O.ns.errors == 0 and not failedLine)
    O.core.Save(tmpSettings)
    local f = readFile(tmpSettings) or ""
    -- 2026-10-30: the key bindings were removed; their stored bind.* keys are retired (dropped on load, not written again)
    check("old settings: Save keeps the stored keys (camera.profile=3, p3 distance), drops the retired key bindings",
      string.find(f, "camera.profile=3", 1, true) ~= nil and string.find(f, "camera.p3.distance=1.5", 1, true) ~= nil
      and string.find(f, "bind.profile_next=", 1, true) == nil and string.find(f, "bind.enabled=", 1, true) == nil, f)
    os.remove(tmpSettings)
  end

  -- ================================================================ core without RegisterView
  do
    local B = boot({ noView = true, view = "all" })
    check("legacy core (no RegisterView): the 5 raw fields of profile 2, no editor, no error", B.ns.count("^Camera placement: ") == 5
      and B.ns.count("^%d [%a]+: ") == 0 and B.ns.find(LABEL.distance) == nil and #B.ns.errors == 0)
    B.cam.NextProfile(1)
    B.core.Set("camera.distance", 2.0)
    local failedLine = false
    for _, l in ipairs(B.lines) do if string.find(l, "FAILED", 1, true) then failedLine = true end end
    check("legacy core: next profile ignored, raw edit of p2 works, no FAILED line", B.cam.rt.profileIndex == 2
      and B.cam.profiles[2].distance == 2.0 and not failedLine)
    local N = boot({ ns = false })
    N.cam.NextProfile(1)
    check("no Native Settings: next profile is a no-op without errors", N.cam.rt.profileIndex == 2)
  end
end)
restoreGlobals()
os.remove(tmpMod)
os.remove(tmpSettings)
if not okAll then check("suite ran without a Lua error", false, errAll) end
print(string.format("utpp_profile_editor test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_profile_editor test failed")

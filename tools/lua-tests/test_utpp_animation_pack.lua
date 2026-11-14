-- utpp_core animation pack list (animation.pack: "Ultimate Third Person Project" / "Game Animations"): registration and menu
-- placement, the values in force (M.Effective forces the dependent options while the Game Animations pack is in force and leaves
-- the stored options of the player alone), the change at a safe moment only, the restart with a stored pack, the modules that
-- have no key (body-turn executor, reload gate), the vehicle entry clips sent to the game scripts, the real utpp_motion / utpp_bodyturn
-- registrations, and the Mod Settings import. Globals: UTPP_CORE_PATH, UTPP_MOTION_PATH, UTPP_BODYTURN_PATH.
-- Mutations of the shipped source must be caught.
local passed, failed = 0, 0
local function check(name, ok, detail)
  if ok then passed = passed + 1; print("[PASS] utpp_animation_pack: " .. name)
  else failed = failed + 1; print("[FAIL] utpp_animation_pack: " .. name .. (detail ~= nil and (" -- " .. tostring(detail)) or "")) end
end

local SAVED = { registerForEvent = registerForEvent, registerInput = registerInput, Game = Game, GetMod = GetMod, open = io.open,
                print = print }
GetMod = nil -- the pack modules are looked up through core.pack.api.mod in these tests
local settingsFile, baseFile = os.tmpname(), os.tmpname()
os.remove(settingsFile)
os.remove(baseFile)
local logs = {}

local function writeFile(p, text)
  local f = assert(SAVED.open(p, "wb"))
  f:write(text)
  f:close()
end

local function loadModule(path, mutation)
  registerForEvent, registerInput = nil, nil
  local realOpen = SAVED.open
  io.open = function(p, m) if p == "utpp_settings.txt" then p = settingsFile end return realOpen(p, m) end
  local mod
  if mutation then
    local f = assert(realOpen(path, "rb"))
    local src = f:read("*a"); f:close()
    local a, b = src:find(mutation[1], 1, true)
    assert(a and not src:find(mutation[1], b + 1, true), "mutation anchor must exist exactly once: " .. mutation[1])
    mod = assert(loadstring(src:sub(1, a - 1) .. mutation[2] .. src:sub(b + 1), "=utpp_core_mutant"))()
  else
    mod = dofile(path)
  end
  io.open = realOpen
  return mod
end
-- every core starts without a settings file unless the test wrote one (keepFile)
local function loadCore(mutation, keepFile)
  if not keepFile then os.remove(settingsFile); os.remove(baseFile) end
  local core = loadModule(UTPP_CORE_PATH, mutation)
  logs = {}
  core.sink = function(line) logs[#logs + 1] = line end
  core.SETTINGS_FILE = settingsFile
  core.MS_BASE_FILE = baseFile
  return core
end
local function logHas(text)
  for _, l in ipairs(logs) do if string.find(l, text, 1, true) then return true end end
  return false
end

-- the motion matching switches of the core (the real registrations) with recording natives, plus the keys that other CET modules
-- register (stubs with a recording apply)
local function boot(core)
  local rec = { mm = {}, fullBody = {}, aimTilt = {}, applied = {} }
  core.SetupMMSettings(function(on) rec.mm[#rec.mm + 1] = on end, function() end,
    { setMeleeFullBody = function(on) rec.fullBody[#rec.fullBody + 1] = on end,
      setMeleeAimParams = function(up) rec.aimTilt[#rec.aimTilt + 1] = up; return true end })
  for _, spec in ipairs({ { "mm.motionDriver", true }, { "bodyturn.shooterMode", true }, { "mm.aimAtCrosshair", true }, { "camera.idleOrbit", true },
                          { "camera.followHoldOnFlip", true } }) do
    local key = spec[1]
    rec.applied[key] = {}
    core.Register({ key = key, section = "test", type = "bool", default = spec[2], label = key, desc = "stub " .. key,
      apply = function(v) local t = rec.applied[key]; t[#t + 1] = v end })
  end
  return rec
end
local function lastOf(t) return t[#t] end
local FORCED_KEYS = { "mm.enabled", "mm.motionDriver", "bodyturn.shooterMode", "melee.fullBody", "melee.aimTilt", "mm.aimAtCrosshair",
  "camera.idleOrbit", "camera.followHoldOnFlip" }

local function companions()
  local c = { calls = {}, mods = {} }
  c.mods.utpp_bodyturn = { SetExecutor = function(on) c.calls[#c.calls + 1] = "executor=" .. tostring(on); return on end }
  c.mods.utpp_reload_gate = { SetEnabled = function(on) c.calls[#c.calls + 1] = "gate=" .. tostring(on); return on end }
  return c
end

local function shipped()
  -- ---------------------------------------------------------------- registration and placement
  local core = loadCore()
  local rec = boot(core)
  local def = core.settings.defs["animation.pack"]
  local ids = {}
  for _, c in ipairs(def and def.choices or {}) do ids[#ids + 1] = c[1] .. "=" .. c[2] end
  check("registered: choice, default utpp, label 'Animation pack', section general, two values", def ~= nil and def.type == "choice"
    and def.default == "utpp" and def.label == "Animation pack" and def.section == "general"
    and table.concat(ids, ",") == "utpp=Ultimate Third Person Project,game=Game Animations", table.concat(ids, ","))
  check("desc: what it does, when it works, the deformation warning", def ~= nil and string.find(def.desc, "works at once while V stands idle", 1, true) ~= nil
    and string.find(def.desc, "next weapon draw, vehicle entry or reload", 1, true) ~= nil
    and string.find(def.desc, "deformations", 1, true) ~= nil)
  local level, group, rank = core.MenuPlacement("animation.pack")
  check("placement: player level, General, second (rank 20, before Shooter mode)", level == "player" and group == "start" and rank == 20
    and select(3, core.MenuPlacement("bodyturn.shooterMode")) > rank)
  check("placement: 'Use motion matching' is developer level (not in Mod Settings)", core.MenuPlacement("mm.enabled") == "developer"
    and core.settings.defs["mm.enabled"] ~= nil)
  check("Mod Settings field name", core.ModSettingsField("animation.pack") == "animation_pack")
  check("Mod Settings values: index 0 / 1", core.ModSettingsEncode(def, "utpp") == "0" and core.ModSettingsEncode(def, "game") == "1"
    and core.ModSettingsDecode(def, "1") == "game" and core.ModSettingsDecode(def, "2") == nil)
  check("default: the mod's animations, nothing forced", core.PackIsGame() == false and core.Effective("mm.enabled") == true
    and core.Effective("melee.fullBody") == true)
  local notes = 0
  for _, key in ipairs({ "melee.fullBody", "melee.aimTilt", "mm.shinobiBonusReload" }) do
    local d = core.settings.defs[key]
    if d and string.sub(d.desc, -#core.PACK_NOTE) == core.PACK_NOTE and select(2, string.gsub(d.desc, "Game Animations pack", "")) == 1 then notes = notes + 1 end
  end
  check("options that depend on the mod's animations say so (once)", notes == 3, notes)
  check("other options have no such note", string.find(core.settings.defs["headLook.impl"].desc, "Game Animations", 1, true) == nil)

  -- ---------------------------------------------------------------- no session: the pack is in force at once, options are forced, not rewritten
  core.Set("melee.aimTilt", false) -- a stored choice of the player
  core.Set("animation.pack", "game")
  check("game pack: in force at once without a game session", core.PackIsGame() == true and core.pack.pending == false)
  local forcedOk = true
  for _, k in ipairs(FORCED_KEYS) do if core.Effective(k) ~= false then forcedOk = false end end
  check("game pack: every dependent option reads false (effective value)", forcedOk)
  check("game pack: the stored options of the player are not rewritten", core.Get("mm.enabled") == true and core.Get("mm.motionDriver") == true
    and core.Get("melee.fullBody") == true and core.Get("melee.aimTilt") == false and core.Get("bodyturn.shooterMode") == true)
  check("game pack: the apply callbacks ran with the forced values (motion matching, motion, Shooter mode, melee, crosshair point, camera)",
    lastOf(rec.mm) == false and lastOf(rec.applied["mm.motionDriver"]) == false and lastOf(rec.applied["bodyturn.shooterMode"]) == false
    and lastOf(rec.fullBody) == false and lastOf(rec.applied["mm.aimAtCrosshair"]) == false
    and lastOf(rec.applied["camera.idleOrbit"]) == false and lastOf(rec.applied["camera.followHoldOnFlip"]) == false)
  check("game pack: unrelated options keep their values", core.Effective("headLook.impl") == true and core.Effective("lootView.impl") == true)
  local nMM, nFB = #rec.mm, #rec.fullBody
  core.ReapplyPack()
  check("game pack: applying again changes nothing (idempotent natives)", #rec.mm == nMM and #rec.fullBody == nFB)
  check("game pack: the pack change is logged", logHas("animation pack -> Game Animations"))
  core.Set("mm.aimAtCrosshair", false)
  core.Set("melee.fullBody", false)
  core.Set("animation.pack", "utpp")
  check("back to the mod's animations: values in force again, and they are the player's own (a choice made meanwhile stays)",
    core.PackIsGame() == false and core.Effective("mm.enabled") == true and core.Effective("mm.motionDriver") == true
    and core.Effective("melee.fullBody") == false and core.Effective("melee.aimTilt") == false and core.Effective("bodyturn.shooterMode") == true
    and core.Effective("mm.aimAtCrosshair") == false)
  check("back to the mod's animations: the apply callbacks ran again", lastOf(rec.mm) == true and lastOf(rec.applied["mm.motionDriver"]) == true
    and lastOf(rec.applied["bodyturn.shooterMode"]) == true and lastOf(rec.fullBody) == false)

  -- ---------------------------------------------------------------- modules without a key
  core = loadCore()
  boot(core)
  local c = companions()
  core.pack.api = { mod = function(name) return c.mods[name] end }
  core.Set("animation.pack", "game")
  check("game pack: the body-turn executor (Locomotion 360) and the authored reload gate are switched off",
    table.concat(c.calls, ",") == "executor=false,gate=false", table.concat(c.calls, ","))
  c.calls = {}
  core.Set("animation.pack", "utpp")
  check("mod pack: both are switched on again", table.concat(c.calls, ",") == "executor=true,gate=true", table.concat(c.calls, ","))
  core.pack.api = { mod = function() return nil end }
  check("a module that is not loaded is skipped without an error", pcall(core.Set, "animation.pack", "game") and core.PackIsGame())
  core.pack.api = { mod = function() return { SetExecutor = function() error("boom") end, SetEnabled = 5 } end }
  check("a failing or malformed module never raises", pcall(core.Set, "animation.pack", "utpp") and not core.PackIsGame())

  -- ---------------------------------------------------------------- the change waits for a safe moment during the game
  core = loadCore()
  rec = boot(core)
  c = companions()
  core.pack.api = { mod = function(name) return c.mods[name] end }
  core.pack.inSession = true
  core.Set("animation.pack", "game")
  check("in the game: the pack does not change at once, the request waits", core.PackIsGame() == false and core.pack.pending == true
    and core.pack.requested == "game" and core.Get("animation.pack") == "game" and core.Effective("mm.enabled") == true and #c.calls == 0)
  local reasons = {
    { "menu", { menu = true } }, { "vehicle", { vehicle = true } }, { "workspot", { workspot = true } }, { "scene", { scene = true } },
    { "swimming", { swimming = true } }, { "takedown", { takedown = true } }, { "carrying", { carrying = true } },
    { "reload", { reload = true } }, { "melee", { melee = true } }, { "aim", { aim = true } },
    { "weapon action", { upper = 3 } }, { "locomotion", { loco = 9 } }, { "state unreadable", { unreadable = true } },
  }
  local allBlock = true
  for _, r in ipairs(reasons) do
    if core.PackBlockReason(r[2]) ~= r[1] then allBlock = false; print("   reason differs: " .. r[1] .. " -> " .. tostring(core.PackBlockReason(r[2]))) end
  end
  check("safe moment: every action of V blocks the change with its reason", allBlock)
  check("safe moment: standing, crouching, sprinting and holding two weapons are safe", core.PackBlockReason({ loco = 1, upper = 0 }) == nil
    and core.PackBlockReason({ loco = 3 }) == nil and core.PackBlockReason({ loco = 4 }) == nil and core.PackBlockReason({ upper = 7, loco = 1 }) == nil)
  for _, r in ipairs(reasons) do
    local o = {}
    for k, v in pairs(r[2]) do o[k] = v end
    core.PackTick(1.0, o)
    if core.PackIsGame() then allBlock = false end
  end
  check("safe moment: an action of V keeps the change waiting however long it lasts", allBlock and core.pack.pending == true)
  check("safe moment: the reason is logged", logHas("animation pack change deferred: reload"))
  core.PackTick(0.4, { loco = 1, upper = 0 })
  check("safe moment: 0.4 s of calm is not enough", core.PackIsGame() == false)
  core.PackTick(0.1, { loco = 1, upper = 0, reload = true })
  core.PackTick(0.4, { loco = 1, upper = 0 })
  check("safe moment: an action in between starts the calm period again", core.PackIsGame() == false)
  core.PackTick(0.2, { loco = 1, upper = 0 })
  check("safe moment: 0.5 s of calm puts the pack in force", core.PackIsGame() == true and core.pack.pending == false
    and core.Effective("mm.enabled") == false and table.concat(c.calls, ",") == "executor=false,gate=false" and lastOf(rec.mm) == false)
  check("safe moment: the change is logged with its cause", logHas("animation pack -> Game Animations (safe moment)"))
  core.Set("animation.pack", "utpp")
  core.Set("animation.pack", "game")
  check("request and withdrawal: asking for the pack in force again changes nothing", core.pack.pending == false and core.PackIsGame() == true)
  core.Set("animation.pack", "utpp")
  check("a request waits again for the way back", core.pack.pending == true and core.PackIsGame() == true)
  core.Set("animation.pack", "game")
  check("withdrawing the request cancels the waiting change", core.pack.pending == false and core.PackIsGame() == true)
  core.Set("animation.pack", "utpp")
  core.PackTick(0.1, nil)
  check("no player (main menu, loading): a waiting change is put in force at once", core.PackIsGame() == false and core.pack.pending == false)
  core.pack.inSession = false
  core.Set("animation.pack", "game")
  check("main menu: a change is in force at once", core.PackIsGame() == true)

  -- ---------------------------------------------------------------- the game scripts of the session (vehicle entry clips)
  core = loadCore()
  boot(core)
  local sent = {}
  local accept = true
  core.pack.api = { setVehicleEntryClips = function(on) sent[#sent + 1] = on; return accept end, mod = function() return nil end }
  check("scripts: a fresh session with the mod's animations sends nothing", core.PackSend(0, true) == false and #sent == 0)
  core.Set("animation.pack", "game")
  accept = false
  check("scripts: the Game Animations pack is sent (entry clips off)", core.PackSend(1, false) == false and sent[1] == false and #sent == 1)
  check("scripts: a refused call is retried after 0.5 s, not every frame", core.PackSend(1.2, false) == false and #sent == 1)
  accept = true
  check("scripts: the retry goes through", core.PackSend(1.6, false) == true and #sent == 2 and core.pack.sent == "game")
  check("scripts: sent once", core.PackSend(3, false) == false and #sent == 2)
  check("scripts: a new game session (load) sends it again", core.PackSend(4, true) == true and #sent == 3 and sent[3] == false)
  core.Set("animation.pack", "utpp")
  check("scripts: back to the mod's animations sends entry clips on", core.PackSend(5, false) == true and sent[4] == true)
  check("scripts: a new session with the mod's animations sends nothing", core.PackSend(6, true) == false and #sent == 4)
  core.pack.api = nil
  Game = nil
  check("scripts: no Game, no error", pcall(core.PackSend, 7, true))
end

shipped()

-- ---------------------------------------------------------------- restart with the stored pack (the game order: the file is read when core loads)
do
  writeFile(settingsFile, "animation.pack=game\n")
  local core = loadCore(nil, true)
  check("restart: the stored pack is in force before any option registers", core.PackIsGame() == true and core.pack.pending == false
    and core.Get("animation.pack") == "game")
  local rec = boot(core)
  local mmOn = false
  for _, v in ipairs(rec.mm) do if v == true then mmOn = true end end
  check("restart: motion matching is never switched on, not even for a moment", mmOn == false and lastOf(rec.mm) == false)
  check("restart: the other dependent options register with their forced values", lastOf(rec.applied["mm.motionDriver"]) == false
    and lastOf(rec.applied["bodyturn.shooterMode"]) == false and lastOf(rec.fullBody) == false and lastOf(rec.applied["camera.idleOrbit"]) == false)
  check("restart: the players's own values are kept", core.Get("mm.enabled") == true and core.Get("mm.motionDriver") == true)
  writeFile(settingsFile, "animation.pack=utpp\n")
  core = loadCore(nil, true)
  rec = boot(core)
  check("restart: the mod's animations: nothing forced, motion matching on", core.PackIsGame() == false and lastOf(rec.mm) == true
    and lastOf(rec.applied["mm.motionDriver"]) == true)
  writeFile(settingsFile, "animation.pack=nonsense\n")
  core = loadCore(nil, true)
  check("restart: an unknown stored value falls back to the mod's animations", core.PackIsGame() == false and core.Get("animation.pack") == "utpp")
  os.remove(settingsFile)
end

-- ---------------------------------------------------------------- the real utpp_motion / utpp_bodyturn registrations
do
  local core = loadCore()
  core.SetupMMSettings(function() end, function() end, {})
  local bt = loadModule(UTPP_BODYTURN_PATH)
  bt.log = function() end
  bt.RegisterSettings(core)
  local v2 = loadModule(UTPP_MOTION_PATH)
  v2.sink = function() end
  local glue = { IsEnabled = function() return false end, SetEnabled = function() end, SetPistolOneHand = function() end,
                 SetPistolAuto = function() end }
  v2.api = setmetatable({ glue = function() return glue end, havePlayer = function() return true end },
    { __index = function() return function() return true end end })
  v2.RegisterSettings(core)
  check("real modules: Motion set and Shooter mode follow the registry on the mod's animations", v2.want.enabled == true and bt.shooterMode.on == false)
  check("real modules: 'clip selection guards' is developer level (not in Mod Settings)", core.settings.defs["mm.selectionGuards"] ~= nil
    and core.MenuPlacement("mm.selectionGuards") == "developer")
  core.Set("bodyturn.shooterMode", true)
  check("real modules: Shooter mode on (stored choice)", bt.shooterMode.on == true)
  core.Set("animation.pack", "game")
  check("real modules: the Game Animations pack switches Motion set and Shooter mode off", v2.want.enabled == false and bt.shooterMode.on == false)
  check("real modules: their stored choices stay", core.Get("mm.motionDriver") == true and core.Get("bodyturn.shooterMode") == true)
  core.pack.api = { mod = function(name) if name == "utpp_bodyturn" then return bt end end }
  core.Set("animation.pack", "utpp")
  check("real modules: back to the mod's animations: motion and Shooter mode follow their stored choices", v2.want.enabled == true
    and bt.shooterMode.on == true)
  -- body-turn executor switch through the module itself (what ReapplyPack calls)
  core.Set("animation.pack", "game")
  check("real modules: the body-turn executor is disabled by the pack switch", string.find(bt.Status(), "enabled=false", 1, true) ~= nil, bt.Status():sub(1, 160))
  core.Set("animation.pack", "utpp")
  check("real modules: and enabled again", string.find(bt.Status(), "enabled=true", 1, true) ~= nil)
  -- a start with the Game Animations pack: the executor stays off (the CET event wiring selects it after a delay)
  local savedGame, savedGet = Game, GetMod
  local function startBodyturn(packGame, mutation)
    local events, locoCalls = {}, {}
    registerForEvent = function(n, f) events[n] = f end
    GetMod = function(n) if n == "utpp_core" then return { Register = function() end, PackIsGame = function() return packGame end } end end
    Game = setmetatable({ GetPlayer = function() return nil end, UTPPLocomotionSetEnabled = function(on) locoCalls[#locoCalls + 1] = on end },
      { __index = function() return function() return true end end })
    local quietPrint = print
    print = function() end
    local mod
    if mutation then
      local f = assert(SAVED.open(UTPP_BODYTURN_PATH, "rb"))
      local src = f:read("*a"); f:close()
      local a, b = src:find(mutation[1], 1, true)
      assert(a and not src:find(mutation[1], b + 1, true), "mutation anchor must exist exactly once: " .. mutation[1])
      mod = assert(loadstring(src:sub(1, a - 1) .. mutation[2] .. src:sub(b + 1), "=utpp_bodyturn_mutant"))()
    else
      mod = dofile(UTPP_BODYTURN_PATH)
    end
    mod.log = function() end
    events.onInit()
    for _ = 1, 305 do events.onUpdate(1 / 60) end
    print = quietPrint
    return mod.Status(), locoCalls
  end
  local status, locoCalls = startBodyturn(true)
  check("start with the Game Animations pack: the executor is selected disabled, Locomotion 360 never enabled",
    string.find(status, "exec=disabled", 1, true) ~= nil and #locoCalls == 0, status:sub(1, 120) .. " calls=" .. #locoCalls)
  status = startBodyturn(false)
  check("start with the mod's animations: the executor is selected as before", string.find(status, "exec=executor", 1, true) ~= nil, status:sub(1, 120))
  status, locoCalls = startBodyturn(true, { "      if gamePackInForce() then E.enabled = false end\n", "" })
  check("start with the Game Animations pack: mutation [pack not read at the start] is caught", string.find(status, "exec=disabled", 1, true) == nil
    or #locoCalls > 0, status:sub(1, 120))
  registerForEvent, Game, GetMod = SAVED.registerForEvent, savedGame, savedGet
end

-- ---------------------------------------------------------------- Mod Settings: a change on its screen becomes the request
do
  local core = loadCore()
  boot(core)
  writeFile(baseFile, "animation.pack=utpp\n")
  local fields = { animation_pack = "1" }
  core.msync.api = {
    available = function() return true end, menuOpen = function() return false end,
    snapshot = function() return "animation_pack=" .. fields.animation_pack .. "\n" end,
    push = function() return 0 end,
  }
  for _ = 1, 6 do core.ModSettingsTick(0.3) end
  check("Mod Settings: choosing Game Animations there sets the pack (list index 1)", core.Get("animation.pack") == "game" and core.PackIsGame() == true,
    tostring(core.Get("animation.pack")))
  os.remove(baseFile)
end

-- ---------------------------------------------------------------- mutations of the shipped source are caught
local function scenario(core, name)
  local rec = boot(core)
  local c = companions()
  core.pack.api = { mod = function(n) return c.mods[n] end }
  if name == "forced values" then
    core.Set("animation.pack", "game")
    return core.Effective("mm.enabled") == false and core.Get("mm.enabled") == true and lastOf(rec.mm) == false
  elseif name == "change waits in the game" then
    core.pack.inSession = true
    core.Set("animation.pack", "game")
    core.PackTick(1.0, { reload = true })
    return core.PackIsGame() == false and core.pack.pending == true
  elseif name == "calm period" then
    core.pack.inSession = true
    core.Set("animation.pack", "game")
    core.PackTick(0.3, { loco = 1 })
    return core.PackIsGame() == false
  elseif name == "calm period is reached" then
    core.pack.inSession = true
    core.Set("animation.pack", "game")
    core.PackTick(0.6, { loco = 1 })
    return core.PackIsGame() == true
  elseif name == "back restores" then
    core.Set("animation.pack", "game")
    core.Set("animation.pack", "utpp")
    return core.Effective("mm.enabled") == true and lastOf(rec.mm) == true
  elseif name == "modules follow" then
    core.Set("animation.pack", "game")
    return table.concat(c.calls, ",") == "executor=false,gate=false"
  elseif name == "vehicle clips not sent for the default" then
    local sent = {}
    core.pack.api.setVehicleEntryClips = function(on) sent[#sent + 1] = on; return true end
    core.PackSend(0, true)
    return #sent == 0
  elseif name == "vehicle clips sent for game" then
    local sent = {}
    core.pack.api.setVehicleEntryClips = function(on) sent[#sent + 1] = on; return true end
    core.Set("animation.pack", "game")
    core.PackSend(0, true)
    return sent[1] == false
  elseif name == "dependent note" then
    return string.find(core.settings.defs["melee.fullBody"].desc, "No effect with the Game Animations pack.", 1, true) ~= nil
  end
  error("unknown scenario " .. name)
end
local MUTANTS = {
  { "forced values", { "  if forced ~= nil and forced[key] ~= nil then return forced[key] end\n", "" } },
  { "change waits in the game", { "  if not P.inSession then return M.ApplyPack(v, why or \"setting\") end", "  if true then return M.ApplyPack(v, why or \"setting\") end" } },
  { "calm period", { "if P.safeFor >= M.PACK_SAFE_SECONDS then", "if P.safeFor >= 0 then" } },
  { "calm period is reached", { "if P.safeFor >= M.PACK_SAFE_SECONDS then", "if P.safeFor >= 1e9 then" } },
  { "back restores", { "  P.pending, P.safeFor, P.deferWhy = false, 0, nil\n  if P.applied == v then return false end\n  P.applied = v",
    "  P.pending, P.safeFor, P.deferWhy = false, 0, nil\n  if P.applied == v then return false end\n  if v == \"game\" then P.applied = v end" } },
  { "modules follow", { "  local on = not M.PackIsGame()\n", "  local on = true\n" } },
  { "vehicle clips not sent for the default", { "  if P.sent == nil and P.applied == \"utpp\" then\n    P.sent = \"utpp\"\n    return false\n  end\n", "" } },
  { "vehicle clips sent for game", { "  local ok, res = pcall(fn, P.applied ~= \"game\")", "  local ok, res = pcall(fn, true)" } },
  { "dependent note", { "    def.desc = def.desc .. M.PACK_NOTE\n", "" } },
}
for _, mu in ipairs(MUTANTS) do
  check("scenario [" .. mu[1] .. "] holds on the shipped source", scenario(loadCore(), mu[1]) == true)
end
for _, mu in ipairs(MUTANTS) do
  local core = loadCore(mu[2]) -- an anchor that no longer exists is a failure of the test itself
  local okS, res = pcall(scenario, core, mu[1])
  check("mutation [" .. mu[1] .. "] is caught", not (okS and res == true), tostring(res))
end
-- the stored pack at the start
do
  writeFile(settingsFile, "animation.pack=game\n")
  local function startScenario(core)
    local rec = boot(core)
    for _, v in ipairs(rec.mm) do if v == true then return false end end
    return core.PackIsGame() == true
  end
  check("scenario [stored pack in force at the start] holds on the shipped source", startScenario(loadCore(nil, true)) == true)
  writeFile(settingsFile, "animation.pack=game\n")
  local mutant = loadCore({ "  M.pack.requested = v\n  M.ApplyPack(v, \"load\")", "  M.pack.requested = v" }, true)
  check("mutation [stored pack not taken at the start] is caught", startScenario(mutant) ~= true)
  os.remove(settingsFile)
end

registerForEvent, registerInput, Game, GetMod = SAVED.registerForEvent, SAVED.registerInput, SAVED.Game, SAVED.GetMod
io.open = SAVED.open
print = SAVED.print
os.remove(settingsFile)
os.remove(baseFile)
print(string.format("utpp_animation_pack test: %d passed, %d failed", passed, failed))
assert(failed == 0, "utpp_animation_pack: " .. failed .. " check(s) failed")

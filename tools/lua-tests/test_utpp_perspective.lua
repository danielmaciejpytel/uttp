-- UTPP utpp_perspective 0.2.7 offline tests (Phase 5N step 4b): every rule, priorities, latching / hysteresis / delays,
-- minimum gap, freeze, session restore, user toggle / quick FPP / override, preference memory through utpp_core,
-- modes (active / off) and subscriber API. Pure logic, no game.
-- Paths: globals UTPP_PERSPECTIVE_PATH / UTPP_CORE_PATH (runner), else derived from this file's location.
local results = { pass = 0, fail = 0 }
local outPrint = print
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  outPrint(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local here = string.match(debug.getinfo(1, "S").source, "^@(.*)$") or ""
local root = string.match(here, "^(.*)[/\\]Source[/\\]tools[/\\]lua%-tests[/\\]") or "."
local PERSP = UTPP_PERSPECTIVE_PATH or (root .. "\\Source\\plugin\\UTPP\\cet\\utpp_perspective\\init.lua")
local CORE = UTPP_CORE_PATH or (root .. "\\Source\\plugin\\UTPP\\cet\\utpp_core\\init.lua")

local savedRegister, savedInput = registerForEvent, registerInput
local function loadMod()
  registerForEvent, registerInput = nil, nil
  local M = dofile(PERSP)
  registerForEvent, registerInput = savedRegister, savedInput
  M.cfg.combat = "fpp"   -- the rule tests below assume FPP in combat (0.2.1 default is TPP; see the policy matrix)
  M.cfg.aiming = "fpp"   -- and FPP while aiming (the fallback default is tpp, like the registered one)
  M.logLines = {}
  M.sink = function(line) M.logLines[#M.logLines + 1] = line end
  return M
end
local function loadCore(file)
  registerForEvent, registerInput = nil, nil
  local C = dofile(CORE)
  registerForEvent, registerInput = savedRegister, savedInput
  C.sink = function() end
  C.SETTINGS_FILE = file
  C.settings.stored, C.settings.values = {}, C.settings.values
  C.Load(file)
  return C
end
local function count(lines, needle)
  local n = 0
  for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end

local M0 = loadMod()
local P = M0.Engine
check("load: module + version + engine", type(M0) == "table" and M0.VERSION == "0.2.7" and type(P.Step) == "function")
-- Contract with utpp_reload_gate: it calls perspective.<Name>() through GetMod inside a pcall, so a missing or renamed
-- function silently disables the Nue authored reload branch. Every name the real gate calls must exist here.
do
  local GATE = string.gsub(PERSP, "utpp_perspective([/\\])init%.lua$", "utpp_reload_gate%1init.lua")
  local fh = io.open(GATE, "r")
  check("contract: utpp_reload_gate source readable", fh ~= nil, GATE)
  if fh then
    local src = fh:read("*a")
    fh:close()
    local names, n = {}, 0
    for name in string.gmatch(src, "perspective%.([%a_][%w_]*)%s*%(") do
      if not names[name] then names[name] = true; n = n + 1 end
    end
    check("contract: utpp_reload_gate calls perspective functions", n > 0, n)
    for name in pairs(names) do
      check("contract: perspective." .. name .. " exists for utpp_reload_gate", type(M0[name]) == "function")
    end
  end
end
M0.lastObs = { aiming = true }
check("contract: IsAiming falls back to the last observation", M0.IsAiming() == true)
M0.lastObs = { aiming = false }
check("contract: IsAiming false when the last observation is not aiming", M0.IsAiming() == false)
M0.lastObs = nil

local function cfg(over)
  local c = {}
  for k, v in pairs(P.DEFAULTS) do c[k] = v end
  c.combat = "fpp"   -- see loadMod
  c.aiming = "fpp"   -- the rule tests below use the FPP-while-aiming policy as their baseline (the fallback default is tpp)
  if over then for k, v in pairs(over) do c[k] = v end end
  return c
end
local function obs(t)
  local o = { player = true }
  if t then for k, v in pairs(t) do o[k] = v end end
  return o
end
-- ready engine that wants TPP and is already in TPP at t=0 (long ago last switch)
local function readyTpp(over)
  local st = P.New(cfg(over), { ready = true, wantsTpp = true })
  local d = P.Step(st, 0, obs())
  st.lastSwitch = -100
  return st, d
end

-- ---------------------------------------------------------------- each rule
do
  local st, d = readyTpp()
  check("baseline: user wants TPP -> TPP (user)", d.perspective == "TPP" and d.reason == "user")
end

local ruleCases = {
  { "outside: replacer (Johnny)", { replacer = true }, "outside", "replacer", "instant" },
  { "outside: braindance", { braindance = true }, "outside", "braindance", "instant" },
  { "outside: cyberspace", { cyberspace = true }, "outside", "cyberspace", "instant" },
  { "mirror", { mirror = true }, "mirror", "mirror", "instant" },
  { "vehicle: driving", { vehicle = 1 }, "vehicle", "vehicle:1", "instant" },
  { "vehicle: passenger", { vehicle = 3 }, "vehicle", "vehicle:3", "instant" },
  { "vehicle: mounted turret", { vehicle = 5 }, "vehicle", "turret", "instant" },
  { "vehicle: MountedToVehicle only (vehicleKeepTpp off)", { mounted = true }, "vehicle", "mounted", "instant", 0,
    { vehicleKeepTpp = false } },
  { "vehicle: transition 4 (vehicleKeepTpp off)", { vehicle = 4 }, "vehicle", "vehicle:4", "instant", 0,
    { vehicleKeepTpp = false } },
  { "takeover: controlling a device / camera", { controllingDevice = true }, "takeover", "device control", "instant" },
  { "scene: SceneTier3", { high = 3 }, "scene", "SceneTier3", "blend" },
  { "scene: SceneTier5", { high = 5 }, "scene", "SceneTier5", "blend" },
  { "takedown: grapple", { takedown = 2 }, "takedown", "takedown:2", "instant" },
  { "takedown: body carry", { carry = 2 }, "takedown", "carry:2", "instant" },
  { "workspot (0.6 s enter delay)", { workspot = true }, "workspot", "workspot", "blend", 0.6 },
  { "combat: InCombat (cut)", { combat = 1 }, "combat", "InCombat", "instant" },
  { "weapon: drawn (policy drawn)", { weaponDrawn = true }, "weapon", "weapon drawn", "instant", 0, { weapon = "drawn" } },
  { "weapon: aiming with a weapon (default policy aim)", { aiming = true, weaponDrawn = true }, "weapon", "aiming", "instant" },
  { "weapon: combat gadget", { gadget = 2 }, "weapon", "gadget", "instant" },
  { "weapon: left-hand cyberware", { lhcw = 4 }, "weapon", "arm cyberware", "instant" },
  { "interaction: device zoom", { uiZoom = true }, "interaction", "device zoom", "blend", 0.1 },
  { "interaction: terminal", { terminal = true }, "interaction", "terminal", "blend", 0.1 },
  { "interaction: elevator", { elevator = true }, "interaction", "elevator", "blend", 0.1 },
  { "interaction: forcing door", { forceDoor = true }, "interaction", "forcing door", "blend", 0.1 },
  { "interaction: personal link", { personalLink = true }, "interaction", "personal link", "blend", 0.1 },
  { "interaction: breach minigame", { minigame = true }, "interaction", "breach", "blend", 0.1 },
}
for _, c in ipairs(ruleCases) do
  local st = readyTpp(c[7])
  local tEnter = c[6] or 0
  if tEnter > 0 then P.Step(st, 1, obs(c[2])) end -- first sample starts the enter timer
  local d, changed = P.Step(st, 1 + tEnter, obs(c[2]))
  check("rule " .. c[1], d.perspective == "FPP" and d.rule == c[3] and d.detail == c[4] and d.style == c[5] and changed == true,
    string.format("%s %s %s %s", d.perspective, tostring(d.rule), tostring(d.detail), tostring(d.style)))
end

do -- default-off rules
  local st = readyTpp()
  local d = P.Step(st, 1, obs({ swim = 1 }))
  check("traversal off by default: swimming stays TPP", d.perspective == "TPP")
  d = P.Step(st, 1.5, obs({ loco = 10 }))
  check("traversal off by default: ladder stays TPP", d.perspective == "TPP")
  d = P.Step(st, 2, obs({ vision = 1 }))
  check("scanner off by default: stays TPP", d.perspective == "TPP")
  st = readyTpp({ traversal = true })
  P.Step(st, 1, obs({ loco = 11 }))
  d = P.Step(st, 1.25, obs({ loco = 11 }))
  check("traversal on: ladder -> FPP after 0.2 s enter delay", d.perspective == "FPP" and d.detail == "ladder")
  st = readyTpp({ traversal = true })
  P.Step(st, 1, obs({ high = 6 }))
  d = P.Step(st, 1.25, obs({ high = 6 }))
  check("traversal on: HighLevel Swimming -> FPP (swimming), not a scene", d.rule == "traversal" and d.detail == "swimming")
  st = readyTpp({ scanner = true })
  d = P.Step(st, 1, obs({ vision = 1 }))
  check("scanner on: FPP instant", d.perspective == "FPP" and d.rule == "scanner" and d.style == "instant")
  st.lastSwitch = -100
  d = P.Step(st, 5, obs())
  check("scanner on: closing returns instantly (exit 0, instant style)", d.perspective == "TPP" and d.style == "instant")
end

-- ---------------------------------------------------------------- settings per situation
do
  local st = readyTpp({ combat = false })
  check("combat off: InCombat stays TPP", P.Step(st, 1, obs({ combat = 1 })).perspective == "TPP")
  st = readyTpp({ scene = false })
  check("scene off: SceneTier4 stays TPP", P.Step(st, 1, obs({ high = 4 })).perspective == "TPP")
  st = readyTpp({ sceneTier = 4 })
  check("sceneTier 4: tier 3 stays TPP", P.Step(st, 1, obs({ high = 3 })).perspective == "TPP")
  check("sceneTier 4: tier 4 -> FPP", P.Step(st, 2, obs({ high = 4 })).perspective == "FPP")
  st = readyTpp()
  check("default tier 3: tier 2 stays TPP", P.Step(st, 1, obs({ high = 2 })).perspective == "TPP")
  st = readyTpp({ weapon = "aim" })
  check("weapon=aim: drawn weapon stays TPP", P.Step(st, 1, obs({ weaponDrawn = true })).perspective == "TPP")
  check("weapon=aim: aiming -> FPP", P.Step(st, 2, obs({ weaponDrawn = true, aiming = true })).detail == "aiming")
  st = readyTpp({ weapon = "never" })
  check("weapon=never: aiming / gadget stay TPP",
    P.Step(st, 1, obs({ weaponDrawn = true, aiming = true, gadget = 3, lhcw = 5 })).perspective == "TPP")
  st = readyTpp({ takedown = false, workspot = false, interaction = false })
  check("takedown / workspot / interaction off: stay TPP",
    P.Step(st, 1, obs({ takedown = 4, carry = 2, workspot = true, terminal = true })).perspective == "TPP"
    and P.Step(st, 2, obs({ takedown = 4, carry = 2, workspot = true, terminal = true })).perspective == "TPP")
  st = readyTpp({ combatCut = false })
  local d = P.Step(st, 1, obs({ combat = 1 }))
  check("combatCut off: combat uses an animated blend", d.style == "blend" and d.blend == 0.8)
  st = readyTpp()
  d = P.Step(st, 1, obs({ high = 3 }))
  check("scene blend uses sceneBlend 0.6 s", d.style == "blend" and math.abs(d.blend - 0.6) < 1e-9)
  st = readyTpp({ animate = false })
  d = P.Step(st, 1, obs({ high = 3 }))
  check("animate off: scene switch is an instant cut", d.style == "instant" and d.blend == 0)
  st = readyTpp({ auto = false })
  check("auto off: combat ignored", P.Step(st, 1, obs({ combat = 1, weaponDrawn = true, high = 3 })).perspective == "TPP")
  check("auto off: vehicle still FPP (hard rule)", P.Step(st, 2, obs({ vehicle = 1 })).rule == "vehicle")
end

-- ---------------------------------------------------------------- priorities
do
  local function winnerOf(o)
    local st = readyTpp({ traversal = true, scanner = true, weapon = "drawn" })
    P.Step(st, 1, obs(o))
    return P.Step(st, 1.5, obs(o)).rule
  end
  check("priority: outside > vehicle", winnerOf({ replacer = true, vehicle = 1 }) == "outside")
  check("priority: vehicle > scene > combat", winnerOf({ vehicle = 1, high = 4, combat = 1 }) == "vehicle")
  check("priority: scene > combat", winnerOf({ high = 4, combat = 1 }) == "scene")
  check("priority: takeover > scene", winnerOf({ controllingDevice = true, high = 4 }) == "takeover")
  check("priority: takedown > combat", winnerOf({ takedown = 4, combat = 1 }) == "takedown")
  check("priority: combat > weapon", winnerOf({ combat = 1, weaponDrawn = true }) == "combat")
  check("priority: weapon > interaction", winnerOf({ weaponDrawn = true, terminal = true }) == "weapon")
  check("priority: interaction > traversal > scanner", winnerOf({ elevator = true, loco = 10, vision = 1 }) == "interaction")
  check("priority: traversal > scanner", winnerOf({ loco = 10, vision = 1 }) == "traversal")
  -- reason handover while staying in FPP is not a perspective change
  local st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  local d, changed = P.Step(st, 2, obs({ combat = 1, high = 3 }))
  check("reason handover combat -> scene: reason updated, not a change", d.rule == "scene" and changed == false)
end

-- ---------------------------------------------------------------- latching, hysteresis, delays
do
  local st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  P.Step(st, 2, obs())                 -- combat cleared at t=2
  local d = P.Step(st, 3.2, obs())
  check("combat exit delay: still FPP 1.2 s after combat", d.perspective == "FPP" and d.rule == "combat")
  local d2, ch = P.Step(st, 3.26, obs())
  check("combat exit delay: TPP after 1.25 s, animated return", d2.perspective == "TPP" and ch and d2.style == "blend")
  st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  P.Step(st, 2, obs())
  P.Step(st, 2.8, obs({ combat = 1 }))  -- flicker back before the exit delay
  P.Step(st, 3.0, obs())               -- cleared again: exit timer restarts
  d = P.Step(st, 4.0, obs())
  check("hysteresis: brief re-entry restarts the exit timer (no switch at 4.0)", d.perspective == "FPP")
  d = P.Step(st, 4.3, obs())
  check("hysteresis: returns 1.25 s after the last clear", d.perspective == "TPP")
  st = readyTpp()
  local a = P.Step(st, 1, obs({ terminal = true }))
  local b = P.Step(st, 1.05, obs({ terminal = true }))
  local c = P.Step(st, 1.1, obs({ terminal = true }))
  check("interaction enter delay 0.1 s: TPP at 0 and 0.05 s, FPP at 0.1 s",
    a.perspective == "TPP" and b.perspective == "TPP" and c.perspective == "FPP")
  st = readyTpp()
  P.Step(st, 1, obs({ terminal = true })); P.Step(st, 1.05, obs())
  check("interaction: a 0.05 s blip does not latch", P.Step(st, 1.2, obs()).perspective == "TPP")
  st = readyTpp({ interactionExit = 3 })
  P.Step(st, 1, obs({ terminal = true })); P.Step(st, 1.1, obs({ terminal = true })); P.Step(st, 2, obs())
  check("interactionExit setting 3 s: FPP at +2.9 s", P.Step(st, 4.9, obs()).perspective == "FPP")
  check("interactionExit setting 3 s: TPP at +3 s", P.Step(st, 5.0, obs()).perspective == "TPP")
  st = readyTpp()
  P.Step(st, 1, obs({ high = 3 })); P.Step(st, 10, obs())
  check("scene exit 1.5 s: FPP at +1.4", P.Step(st, 11.4, obs()).perspective == "FPP")
  check("scene exit 1.5 s: TPP at +1.5", P.Step(st, 11.5, obs()).perspective == "TPP")
  -- workspot max hold 300 s
  st = readyTpp()
  P.Step(st, 1, obs({ workspot = true }))
  check("workspot: FPP at 299 s", P.Step(st, 300, obs({ workspot = true })).perspective == "FPP")
  check("workspot: released at 300 s (stuck guard)", P.Step(st, 301, obs({ workspot = true })).perspective == "TPP")
  d = P.Step(st, 302.5, obs({ workspot = true }))
  check("workspot: stays TPP while the stuck state persists", d.perspective == "TPP")
  P.Step(st, 303, obs())
  P.Step(st, 304, obs({ workspot = true }))
  check("workspot: not re-armed before the 0.6 s enter delay", P.Step(st, 304.1, obs({ workspot = true })).perspective == "TPP")
  check("workspot: re-armed after the state cleared once", P.Step(st, 304.7, obs({ workspot = true })).perspective == "FPP")
  -- minimum gap between automatic switches
  st = readyTpp({ scanner = true })
  P.Step(st, 1, obs({ vision = 1 }))          -- switch at t=1
  d = P.Step(st, 1.2, obs())                  -- scanner closed, exit 0, but minGap 0.5
  check("minGap: automatic return held 0.2 s after a switch (pending)", d.perspective == "FPP" and d.pending == "user")
  d = P.Step(st, 1.5, obs())
  check("minGap: return allowed after 0.5 s", d.perspective == "TPP")
  st = readyTpp({ combatCut = true })
  P.Step(st, 1, obs({ vision = 1, scanner = true }))
  P.Step(st, 1.1, obs())
  st.lastSwitch = 1.05
  d = P.Step(st, 1.1, obs({ vehicle = 1 }))
  check("minGap: urgent rule (vehicle) bypasses the gap", d.rule == "vehicle")
  st = readyTpp({ minGap = 2 })
  P.Step(st, 1, obs({ high = 3 }))
  P.Step(st, 1.1, obs())
  d = P.Step(st, 2.7, obs())
  check("minGap setting 2 s: scene return held until t=3", d.perspective == "FPP")
  check("minGap setting 2 s: returns at t=3", P.Step(st, 3.0, obs()).perspective == "TPP")
end

-- ---------------------------------------------------------------- user: toggle, quick FPP, override, lock
do
  local st = readyTpp()
  local ok, note = P.Toggle(st)
  local d, ch = P.Step(st, 0.1, obs())
  check("toggle TPP -> FPP: immediate (user event bypasses minGap), blend", ok and note == "FPP" and d.perspective == "FPP"
    and ch and d.style == "blend" and st.wantsTpp == false)
  ok, note = P.Toggle(st); d = P.Step(st, 0.2, obs())
  check("toggle FPP -> TPP", ok and note == "TPP" and d.perspective == "TPP" and st.wantsTpp)
  st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  ok, note = P.Toggle(st); d = P.Step(st, 1.1, obs({ combat = 1 }))
  check("toggle during combat (no override): deferred -> wantsTpp flips, FPP kept",
    ok and note == "deferred-cancelled:combat" and st.wantsTpp == false and d.perspective == "FPP", note)
  ok, note = P.Toggle(st)
  check("second toggle: deferred TPP again", note == "deferred:combat" and st.wantsTpp == true)
  P.Step(st, 2, obs()); d = P.Step(st, 3.3, obs())
  check("after combat ends: user's TPP returns", d.perspective == "TPP" and d.reason == "user")
  st = readyTpp({ toggleOverrides = true, weapon = "drawn" })
  P.Step(st, 1, obs({ combat = 1, weaponDrawn = true }))
  ok, note = P.Toggle(st); d = P.Step(st, 1.1, obs({ combat = 1, weaponDrawn = true }))
  check("toggleOverrides: toggle in combat -> TPP, combat+weapon suppressed", note == "override:combat"
    and d.perspective == "TPP" and st.suppressed.combat and st.suppressed.weapon)
  d = P.Step(st, 2, obs({ combat = 1, weaponDrawn = true, high = 3 }))
  check("toggleOverrides: non-overridable scene still wins", d.rule == "scene")
  P.Step(st, 3, obs()); P.Step(st, 5, obs())
  check("toggleOverrides: suppression cleared after the situation ended", st.suppressed.combat == nil)
  P.Step(st, 6, obs({ combat = 1 }))
  check("toggleOverrides: next combat switches to FPP again", P.Step(st, 6.1, obs({ combat = 1 })).perspective == "FPP")
  st = readyTpp({ toggleOverrides = true })
  P.Step(st, 1, obs({ high = 4 }))
  ok, note = P.Toggle(st)
  check("toggleOverrides: scene is not overridable (deferred)", string.find(note, "deferred", 1, true) ~= nil, note)
  st = readyTpp()
  P.Step(st, 1, obs({ replacer = true }))
  local before = st.wantsTpp
  ok, note = P.Toggle(st)
  check("lock: toggle refused while outside the body", ok == false and note == "locked:outside" and st.wantsTpp == before)
  check("lock: Desired.locked reflects it", st.locked == true)
  st = readyTpp()
  P.SetQuick(st, true)
  d, ch = P.Step(st, 0.1, obs())
  check("quick FPP hold: instant FPP", d.perspective == "FPP" and d.reason == "quick_fpp" and d.style == "instant" and ch)
  ok = P.Toggle(st)
  check("quick FPP: toggle refused while held", ok == false)
  P.SetQuick(st, false)
  d = P.Step(st, 0.2, obs())
  check("quick FPP release: instant TPP (minGap bypassed)", d.perspective == "TPP" and d.style == "instant")
  st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  P.SetQuick(st, true)
  check("quick FPP under combat: reason stays combat", P.Step(st, 1.1, obs({ combat = 1 })).rule == "combat")
end

-- ---------------------------------------------------------------- freeze, session
do
  local st = readyTpp()
  P.Step(st, 1, obs({ combat = 1 }))
  P.Step(st, 2, obs())                     -- exit timer starts
  local d = P.Step(st, 2.5, obs({ menu = true }))
  check("menu freezes the decision", d.perspective == "FPP" and d.frozen == "menu")
  P.Step(st, 30, obs({ menu = true, combat = 0 }))
  d = P.Step(st, 30.1, obs())
  check("freeze: time in the menu does not count towards the exit delay", d.perspective == "FPP" and d.frozen == nil)
  d = P.Step(st, 30.9, obs())
  check("freeze: exit delay completes after the menu (0.5 + 0.75 s)", d.perspective == "TPP")
  st = readyTpp()
  d = P.Step(st, 1, obs({ photo = true, vehicle = 1 }))
  check("photo mode freezes (vehicle input ignored meanwhile)", d.perspective == "TPP" and d.frozen == "photo")
  -- session loss and restore
  st = P.New(cfg(), { wantsTpp = true })
  local ch
  for i = 1, 9 do d, ch = P.Step(st, i * 0.1, obs()) end
  check("session: decision held until 10 stable polls", d.reason == "init" and d.perspective == "FPP" and ch == false)
  d, ch = P.Step(st, 1.0, obs())
  check("session: restore on the 10th poll -> TPP, instant, changed", d.perspective == "TPP" and d.restore == true
    and d.style == "instant" and ch == true)
  P.Step(st, 2, obs({ combat = 1 }))
  P.Step(st, 2.5, { player = false })
  d = P.Step(st, 2.7, { player = false })
  check("session: short player gap (<0.75 s) keeps the state", st.session == "ready")
  d = P.Step(st, 3.3, { player = false })
  check("session: player gone >= 0.75 s -> lost, latches cleared", st.session == "lost" and not st.rules.combat.latched)
  for i = 1, 5 do P.Step(st, 4 + i * 0.1, obs({ menu = true })) end
  check("session: menus do not count as stable polls", st.session ~= "ready")
  d, ch = P.Step(st, 4 + 12.1, obs({ menu = true }))
  check("session: settleMax 12 s ends the wait even in menus", st.session == "ready" and ch == false and d.frozen == "menu")
  d, ch = P.Step(st, 16.3, obs())
  check("session: restore emitted once the menu closes", ch == true and d.restore == true and d.perspective == "TPP")
end

-- ---------------------------------------------------------------- module API: subscribe, desired, modes, logging
do
  local M = loadMod()
  M.forceMode = "active"
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  local got, got2 = {}, {}
  local tok = M.Subscribe(function(d, prev, mode) got[#got + 1] = { d = d, prev = prev, mode = mode } end)
  M.Subscribe(function() error("boom") end)
  M.Update(0, obs())
  check("active: subscriber notified on first decision", #got == 1 and got[1].d.perspective == "TPP" and got[1].mode == "active")
  M.Update(1, obs({ combat = 1 }))
  check("active: subscriber gets the combat FPP with reason / style / previous",
    #got == 2 and got[2].d.perspective == "FPP" and got[2].d.reason == "combat" and got[2].d.style == "instant"
    and got[2].prev.perspective == "TPP")
  check("listener error isolated and logged", M.stats.listenerErrors == 2 and count(M.logLines, "listener error") == 2)
  M.Update(1.5, obs({ combat = 1, high = 3 }))
  check("reason-only change: no notification, logged as (reason)", #got == 2 and count(M.logLines, "(reason) FPP <- scene (SceneTier3)") == 1)
  local d = M.Desired()
  d.perspective = "XXX"
  check("Desired returns a copy with mode / wantsTpp", M.Desired().perspective == "FPP" and M.Desired().mode == "active"
    and M.Desired().wantsTpp == true)
  check("Unsubscribe", M.Unsubscribe(tok) == true and M.Unsubscribe(tok) == false)
  M.Update(5, obs())
  check("unsubscribed listener not called", #got == 2)
  check("decision log line with style and mode", count(M.logLines, "FPP <- combat (InCombat) instant [active]") == 1,
    table.concat(M.logLines, " | "))
  check("Status string", string.find(M.Status(), "utpp_perspective 0.2.7 mode=active desired=FPP reason=scene:SceneTier3", 1, true) ~= nil, M.Status())
  check("history recorded", #M.history >= 3 and M.history[2].reason == "combat")
  -- ToggleUser applies immediately on the last observation
  M.Update(20, obs())
  check("situations over: back to TPP", M.Desired().perspective == "TPP")
  M.Subscribe(function(dd) got2[#got2 + 1] = dd end)
  M.ToggleUser("test")
  check("ToggleUser: applied at once (FPP), logged", M.Desired().perspective == "FPP" and got2[1] and got2[1].reason == "user"
    and count(M.logLines, "toggle (test): FPP") == 1)
  M.SetQuickFpp(false)
  M.SetUserWantsTpp(true, "test")
  check("SetUserWantsTpp(true) -> TPP", M.Desired().perspective == "TPP")
  M.SetQuickFpp(true)
  check("SetQuickFpp(true) -> FPP quick", M.Desired().reason == "quick_fpp" and M.Desired().quick == true)
  M.SetQuickFpp(false)
  check("SetQuickFpp(false) -> TPP", M.Desired().perspective == "TPP")

  -- off mode: subscribers are still notified (hard rules only)
  local S = loadMod()
  S.forceMode = "off"
  S.st = P.New(S.cfg, { ready = true, wantsTpp = true })
  local offGot = {}
  S.Subscribe(function(dd, _, mode) offGot[#offGot + 1] = { d = dd, mode = mode } end)
  S.Update(0, obs())
  S.Update(1, obs({ vehicle = 1 }))
  check("off: subscriber notified by the hard vehicle rule, mode off", #offGot == 2 and offGot[2].d.perspective == "FPP"
    and offGot[2].mode == "off" and offGot[2].d.mode == "off")
  -- log off
  local L = loadMod()
  L.forceMode = "active"
  L.st = P.New(L.cfg, { ready = true, wantsTpp = true })
  L.Update(0, obs())
  L.cfg.log = false
  local nl = #L.logLines
  L.Update(30, obs({ combat = 1 }))
  check("log setting off: nothing logged", #L.logLines == nl)
end

-- ---------------------------------------------------------------- utpp_core: settings, persistence, modes, actions
do
  local file = os.tmpname()
  local C = loadCore(file)
  local M = loadMod()
  local ok = M.AttachCore(C)
  local sec = C.settings.sections.perspective
  check("AttachCore: section 'perspective' with all settings", ok and sec ~= nil and #sec.keys == #M.SETTINGS
    and sec.owner == "utpp_perspective", sec and #sec.keys)
  check("AttachCore: remembered choice registered but hidden", C.settings.defs["perspective.userWantsTpp"] ~= nil
    and not string.find(table.concat(sec.keys, ","), "userWantsTpp", 1, true))
  check("AttachCore: default remembered TPP", M.st.wantsTpp == true)
  check("AttachCore: the module binds no keys (no ACTIONS / ownedActions)", M.ACTIONS == nil and M.ownedActions == nil)
  check("mode: impl default (on) -> active", M.Mode() == "active")
  C.Set("perspective.impl", "off")
  check("mode: impl off -> off", M.Mode() == "off")
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  M.Update(0, obs())
  M.Update(1, obs({ combat = 1 }))
  check("mode off: automatic combat rule disabled", M.Desired().perspective == "TPP")
  M.Update(2, obs({ vehicle = 1 }))
  check("mode off: vehicle still hands over", M.Desired().rule == "vehicle")
  C.Set("perspective.impl", "auto")
  check("mode: a legacy auto value is on -> active", C.Get("perspective.impl") == true and M.Mode() == "active")
  -- settings apply to the engine
  C.Set("perspective.combatExit", 3)
  C.Set("perspective.aiming", "fpp")
  C.Set("perspective.scanner", true)
  check("settings apply to cfg (combatExit, aiming, scanner)", M.cfg.combatExit == 3 and M.cfg.aiming == "fpp"
    and M.cfg.scanner == true and M.st.cfg == M.cfg)
  check("choice validation: invalid aiming policy rejected", C.Set("perspective.aiming", "sometimes") == nil and M.cfg.aiming == "fpp")
  -- console toggle (no key binding any more)
  M.Update(3, obs())
  M.ToggleUser("test")
  check("ToggleUser -> toggles the remembered perspective", M.Desired().perspective == "FPP" and M.st.wantsTpp == false)
  local f = io.open(file, "r"); local txt = f:read("*a"); f:close()
  check("preference persisted to utpp_settings (perspective.userWantsTpp=false)",
    string.find(txt, "perspective.userWantsTpp=false", 1, true) ~= nil)
  M.SetQuickFpp(true)
  check("SetQuickFpp(true) holds quick first person", M.st.quick == true)
  M.SetQuickFpp(false)
  -- new session: preference restored
  local C2 = loadCore(file)
  local M2 = loadMod()
  M2.AttachCore(C2)
  check("memory: next session starts with the remembered FPP", M2.st.wantsTpp == false)
  check("memory: other settings restored (combatExit 3)", M2.cfg.combatExit == 3)
  -- remember off -> always third person (2026-10-24: no 'Start in third person' option; an old stored false is ignored)
  C2.Set("perspective.startInLastView", false)
  C2.Set("perspective.defaultTpp", false)
  local C3 = loadCore(file)
  local M3 = loadMod()
  M3.AttachCore(C3)
  check("remember off: always starts in third person (stored perspective.defaultTpp=false ignored)", M3.st.wantsTpp == true)
  check("perspective.defaultTpp is not a menu option any more", C3.MenuPlacement("perspective.defaultTpp") == "developer")
  M3.st = P.New(M3.cfg, { ready = true, wantsTpp = true })
  M3.Update(0, obs())
  M3.ToggleUser("test")
  f = io.open(file, "r"); txt = f:read("*a"); f:close()
  check("remember off: toggle not persisted (file keeps false)", string.find(txt, "perspective.userWantsTpp=false", 1, true) ~= nil
    and M3.st.wantsTpp == false)
  local M4 = loadMod()
  check("AttachCore without utpp_core fails cleanly", M4.AttachCore(nil) == false)
  os.remove(file)
end

-- ---------------------------------------------------------------- 0.2.0: sequences replayed from in-game logs
-- recorded play sessions of 0.5.2 / 0.5.3 (utpp_vehiclecam obs lines, video frames).
-- Each segment: { from, to, obs } polled at 10 Hz like the in-game wiring.
local function activeMod(wantsTpp)
  local S = loadMod()
  S.forceMode = "active"
  S.st = P.New(S.cfg, { wantsTpp = wantsTpp ~= false })
  return S
end
local function replay(S, segs, trace)
  for _, sg in ipairs(segs) do
    local t = sg[1]
    while t < sg[2] - 1e-9 do
      local o = obs(sg[3])
      local d = S.Update(t, o)
      if trace then trace[#trace + 1] = { t = t, p = d.perspective, reason = d.reason, detail = d.detail, style = d.style } end
      t = t + 0.1
    end
  end
end
local function at(trace, t)
  local best = nil
  for _, e in ipairs(trace) do if e.t <= t + 1e-6 then best = e end end
  return best or {}
end
local function anyIn(trace, t0, t1, pred)
  for _, e in ipairs(trace) do if e.t >= t0 - 1e-6 and e.t < t1 - 1e-6 and pred(e) then return true end end
  return false
end

do -- (1) 0.5.3 02:12:07-02:12:14 (t 246.29-252.50): drive cycle; keepTpp: enter and exit in TPP (frames: TPP while
  -- entering, vehicle camera seated, TPP from the exit). Times relative to this sequence.
  local S = activeMod(true)
  local tr = {}
  replay(S, {
    { 0, 3, {} },                                                    -- on foot, session settles + restore (TPP)
    { 3, 3.1, { mounted = true, workspot = true } },                 -- MountedToVehicle first (state 0)
    { 3.1, 5.0, { mounted = true, vehicle = 4, workspot = true } },  -- enter transition 1.9 s
    { 5.0, 5.6, { mounted = true, vehicle = 1, workspot = true } },  -- Driving
    { 5.6, 7.3, { mounted = true, vehicle = 1, workspot = true } },  -- ride
    { 7.3, 7.8, { mounted = true, vehicle = 4, workspot = true } },  -- exit starts
    { 7.8, 9.3, { mounted = true, vehicle = 4, workspot = true } },
    { 9.3, 9.4, { mounted = true, workspot = true } },               -- unmount
    { 9.4, 9.5, { workspot = true } },                               -- IsInWorkspot clears a frame later
    { 9.5, 12, {} },
  }, tr)
  check("log seq 0.5.3 drive: TPP while entering (transition kept)", not anyIn(tr, 3, 5.0, function(e) return e.p == "FPP" end)
    and at(tr, 4.9).reason == "user", at(tr, 4.9).reason)
  local d = at(tr, 5.05)
  check("log seq 0.5.3 drive: Driving -> FPP (game vehicle camera), instant", d.p == "FPP" and d.reason == "vehicle"
    and d.detail == "vehicle:1" and d.style == "instant", tostring(d.detail))
  check("log seq 0.5.3 drive: exit -> TPP 0.4-0.5 s after the exit start, instant",
    at(tr, 7.65).p == "FPP" and at(tr, 7.85).p == "TPP" and at(tr, 7.85).style == "instant")
  check("log seq 0.5.3 drive: seat workspot never wins (no workspot reason, no FPP after the exit)",
    not anyIn(tr, 0, 12, function(e) return e.reason == "workspot" end)
    and not anyIn(tr, 7.8, 12, function(e) return e.p == "FPP" end))
  check("log seq 0.5.3 drive: the user preference stays TPP", S.st.wantsTpp == true)
end

do -- (2) 0.5.2 01:41:27-01:41:32: restore TPP after the load and stay in TPP.
  local S = activeMod(true)
  local tr = {}
  replay(S, { { 0, 20, {} } }, tr)
  check("log seq 0.5.2 start: TPP from the restore to the end", at(tr, 2).p == "TPP" and at(tr, 19.9).p == "TPP"
    and S.st.wantsTpp == true)
end

do -- (3) 0.5.2 01:43:55 / 01:45:48: UpperBody Aim for ~0.4 s during a sprint dodge with empty hands (frames: TPP, no
  -- weapon). 0.1.0: "FPP <- weapon (aiming)".
  local S = activeMod(true)
  local tr = {}
  replay(S, { { 0, 3, {} }, { 3, 3.4, { aiming = true } }, { 3.4, 6, {} } }, tr)
  check("log seq 0.5.2 dodge: aim state without a weapon stays TPP", not anyIn(tr, 1.5, 6, function(e) return e.p == "FPP" end))
end

do -- (4) 0.5.3 02:14:39 (t 398.83): weapon drawn on foot. Policy aim: TPP kept; aiming the weapon -> FPP.
  local S = activeMod(true)
  local tr = {}
  replay(S, { { 0, 3, {} }, { 3, 6, { weaponDrawn = true } }, { 6, 7, { weaponDrawn = true, aiming = true } },
    { 7, 9.5, {} } }, tr)
  check("log seq 0.5.3 weapon: drawn weapon stays TPP (default policy aim)", not anyIn(tr, 3, 6, function(e) return e.p == "FPP" end))
  check("log seq 0.5.3 weapon: the drawn weapon does not change the user preference", S.st.wantsTpp == true)
  local d = at(tr, 6.05)
  check("log seq 0.5.3 weapon: aiming the weapon -> FPP instant", d.p == "FPP" and d.detail == "aiming" and d.style == "instant")
  check("log seq 0.5.3 weapon: back to TPP 1.25 s after aiming", at(tr, 8.2).p == "FPP" and at(tr, 8.4).p == "TPP")
  local S2 = activeMod(true)
  S2.cfg.weapon = "drawn"
  local tr2 = {}
  replay(S2, { { 0, 3, {} }, { 3, 6, { weaponDrawn = true } } }, tr2)
  check("log seq 0.5.3 weapon: policy drawn -> FPP with a drawn weapon", at(tr2, 3.05).p == "FPP")
end

do -- (5) negative / boundary cases of the vehicle hold
  local function drive(wantsTpp, pre, cfgOver)
    local S = activeMod(wantsTpp)
    if cfgOver then for k, v in pairs(cfgOver) do S.cfg[k] = v end end
    local tr = {}
    local w = pre and pre.weaponDrawn or nil
    replay(S, { { 0, 3, pre or {} }, { 3, 5, { mounted = true, vehicle = 4, weaponDrawn = w } },
      { 5, 7, { mounted = true, vehicle = 1 } }, { 7, 9, { mounted = true, vehicle = 4 } }, { 9, 12, {} } }, tr)
    return tr, S
  end
  local tr = drive(false)
  check("vehicle hold: user FPP -> FPP throughout (no TPP hold)", not anyIn(tr, 1.5, 12, function(e) return e.p == "TPP" end))
  tr = drive(true, nil, { vehicleKeepTpp = false })
  check("vehicle hold off: transition -> FPP at once (old behaviour)", at(tr, 3.05).p == "FPP"
    and at(tr, 3.05).detail == "vehicle:4" and at(tr, 8.5).p == "FPP" and at(tr, 9.65).p == "TPP")
  tr = drive(true, { weaponDrawn = true }, { weapon = "drawn" })
  check("vehicle hold: entering while a rule forces FPP -> not kept (vehicle:4)", at(tr, 4).p == "FPP"
    and at(tr, 4).detail == "vehicle:4", tostring(at(tr, 4).detail))
  local S = activeMod(true)
  tr = {}
  replay(S, { { 0, 3, {} }, { 3, 9.5, { mounted = true, vehicle = 4 } } }, tr)
  check("vehicle hold: transition stuck > 6 s -> game camera (vehicle_hold, reason vehicle)", at(tr, 8.9).p == "TPP"
    and at(tr, 9.1).p == "FPP" and at(tr, 9.1).reason == "vehicle", tostring(at(tr, 9.1).detail))
  S = activeMod(true)
  tr = {}
  replay(S, { { 0, 3, {} }, { 3, 5, { mounted = true, vehicle = 4 } }, { 5, 7, { mounted = true, vehicle = 3 } } }, tr)
  check("vehicle hold: passenger seat -> FPP (vehicle:3)", at(tr, 5.05).p == "FPP" and at(tr, 5.05).detail == "vehicle:3")
  S = activeMod(true)
  tr = {}
  replay(S, { { 0, 3, {} }, { 3, 5, { mounted = true, vehicle = 1 } }, { 5, 6, { mounted = true, vehicle = 4, combat = 1 } },
    { 6, 8, { combat = 1 } } }, tr)
  check("vehicle hold: exit during combat -> combat FPP after the vehicle", at(tr, 5.6).p == "FPP"
    and at(tr, 5.6).reason == "combat" and at(tr, 7.9).reason == "combat")
  S = activeMod(true)
  tr = {}
  replay(S, { { 0, 3, {} }, { 3, 6, { workspot = true } }, { 6, 8, {} } }, tr)
  check("workspot on foot (bench) still -> FPP after 0.6 s", at(tr, 3.45).p == "TPP" and at(tr, 3.65).p == "FPP"
    and at(tr, 3.65).reason == "workspot")
end

-- ---------------------------------------------------------------- 0.2.1: perspective policies (combat / weaponDrawn / aiming)
-- Defaults (2026-10-07): combat TPP (default) | FPP, weapon drawn TPP (default) | FPP, aiming FPP (default) | TPP.
do
  check("policy defaults: combat TPP, weaponDrawn TPP, aiming TPP (same as the registered defaults)", P.DEFAULTS.combat == "tpp"
    and P.DEFAULTS.weaponDrawn == "tpp" and P.DEFAULTS.aiming == "tpp")
  local combos = 0
  for _, cb in ipairs({ "tpp", "fpp" }) do
    for _, wd in ipairs({ "tpp", "fpp" }) do
      for _, am in ipairs({ "fpp", "tpp" }) do
        combos = combos + 1
        local tag = string.format("[combat=%s drawn=%s aiming=%s] ", cb, wd, am)
        local over = { combat = cb, weaponDrawn = wd, aiming = am }
        local C, W, A = cb == "fpp", wd == "fpp", am == "fpp"
        local function run(o, expectFpp, name)
          local st = P.New(cfg(over), { ready = true, wantsTpp = true })
          P.Step(st, 0, obs())
          local d = P.Step(st, 1, obs(o))
          check(tag .. name .. " -> " .. (expectFpp and "FPP" or "TPP"), (d.perspective == "FPP") == expectFpp,
            d.perspective .. " " .. tostring(d.reason) .. " " .. tostring(d.detail))
        end
        run({ combat = 1 }, C, "combat without weapon")
        run({ weaponDrawn = true }, W, "weapon drawn, calm")
        run({ weaponDrawn = true, aiming = true }, W or A, "aiming")
        run({ gadget = 2 }, A, "grenade / gadget")
        run({ lhcw = 4 }, A, "arm cyberware")
        run({ aiming = true }, false, "aim state without a weapon (dodge / slide)")
        run({ combat = 1, weaponDrawn = true }, C or W, "combat + weapon drawn")
        run({ combat = 1, weaponDrawn = true, aiming = true }, C or W or A, "combat + aiming")
        -- end of aiming: back to TPP with an animated blend after the exit delay (when the aim was FPP at all)
        do
          local st = P.New(cfg(over), { ready = true, wantsTpp = true })
          P.Step(st, 0, obs())
          local d = P.Step(st, 1, obs({ weaponDrawn = true, aiming = true }))
          local wasFpp = d.perspective == "FPP"
          check(tag .. "end of aiming: aim FPP iff a policy asks for it", wasFpp == (W or A))
          d = P.Step(st, 1.5, obs())
          check(tag .. "end of aiming: exit delay holds the same view", (d.perspective == "FPP") == wasFpp)
          local d2, changed = P.Step(st, 3.0, obs())
          check(tag .. "end of aiming: TPP after the delay", d2.perspective == "TPP" and d2.reason == "user")
          if wasFpp then
            check(tag .. "end of aiming: smooth blend back to TPP", changed and d2.style == "blend" and d2.blend == 0.8,
              tostring(d2.style) .. " " .. tostring(d2.blend))
          else
            check(tag .. "never left TPP: no switch at all", not changed)
          end
        end
        -- combat + vehicle: the vehicle always wins (game camera); on leaving, combat follows its policy
        do
          local st = P.New(cfg(over), { ready = true, wantsTpp = true })
          P.Step(st, 0, obs())
          local d = P.Step(st, 1, obs({ combat = 1, weaponDrawn = true, vehicle = 1 }))
          check(tag .. "combat + vehicle: FPP (vehicle)", d.perspective == "FPP" and d.rule == "vehicle")
          P.Step(st, 3.5, obs({ combat = 1, weaponDrawn = true }))
          d = P.Step(st, 4, obs({ combat = 1, weaponDrawn = true }))
          check(tag .. "left the vehicle in combat: afterwards by policy", (d.perspective == "FPP") == (C or W))
          d = P.Step(st, 5, obs({ combat = 1, weaponDrawn = true, aiming = true }))
          check(tag .. "aiming after the vehicle: by policy", (d.perspective == "FPP") == (C or W or A))
        end
      end
    end
  end
  check("policy matrix: all 8 combinations ran", combos == 8)
  -- negative control inside the suite: with all TPP policies nothing may switch to FPP
  local stN = P.New(cfg({ combat = "tpp", weaponDrawn = "tpp", aiming = "tpp" }), { ready = true, wantsTpp = true })
  P.Step(stN, 0, obs())
  check("negative control: all TPP policies keep combat + aiming in TPP (a FPP expectation would fail)",
    P.Step(stN, 1, obs({ combat = 1, weaponDrawn = true, aiming = true, gadget = 2, lhcw = 4 })).perspective == "TPP")
  -- legacy cfg forms
  local stL = P.New(cfg({ combat = true, weaponDrawn = "tpp", aiming = "fpp" }), { ready = true, wantsTpp = true })
  P.Step(stL, 0, obs())
  check("legacy boolean combat=true means FPP", P.Step(stL, 1, obs({ combat = 1 })).perspective == "FPP")
  check("legacy weapon=drawn -> FPP with a weapon, never -> aiming TPP, aim -> neutral",
    P.DrawnFpp({ weapon = "drawn" }) and not P.DrawnFpp({ weapon = "aim", weaponDrawn = "tpp" })
    and not P.AimingFpp({ weapon = "never", aiming = "fpp" }) and P.AimingFpp({ weapon = "aim", aiming = "fpp" }))
  check("Policies() text", P.Policies({ combat = "tpp", weaponDrawn = "fpp", aiming = "tpp" }) == "combat=TPP weaponDrawn=FPP aiming=TPP")
end

-- settings: registration, defaults, apply, validation, persistence and the migration of the old perspective.weapon
do
  local file = os.tmpname()
  local C = loadCore(file)
  local M = loadMod()
  M.AttachCore(C)
  local defs = C.settings.defs
  check("settings: combat / weaponDrawn / aiming registered as choices with defaults tpp / tpp / tpp",
    defs["perspective.combat"].type == "choice" and C.Get("perspective.combat") == "tpp"
    and defs["perspective.weaponDrawn"].type == "choice" and C.Get("perspective.weaponDrawn") == "tpp"
    and defs["perspective.aiming"].type == "choice" and C.Get("perspective.aiming") == "tpp")
  -- 2026-10-23 (user): no "(experimental)" in the Combat labels / options; both options still offered
  local function choiceLabels(d)
    local t = {}
    for _, c in ipairs(d.choices or {}) do t[#t + 1] = c[1] .. "=" .. c[2] end
    return table.concat(t, ",")
  end
  local noExp = true
  for _, k in ipairs({ "perspective.aiming", "perspective.aimingScoped", "perspective.combat", "perspective.weaponDrawn" }) do
    local d = defs[k]
    if string.find(string.lower(d.label), "experimental", 1, true) or string.find(string.lower(choiceLabels(d)), "experimental", 1, true) then
      noExp = false
    end
  end
  check("settings: aiming label 'Perspective while aiming', options First person / Third person, no 'experimental' in Combat labels",
    defs["perspective.aiming"].label == "Perspective while aiming" and choiceLabels(defs["perspective.aiming"]) == "fpp=First person,tpp=Third person"
    and choiceLabels(defs["perspective.aimingScoped"]) == "fpp=First person,tpp=Third person" and noExp
    and string.find(defs["perspective.aiming"].desc, "over-the-shoulder", 1, true) ~= nil,
    choiceLabels(defs["perspective.aiming"]))
  check("settings: every new option has a label and a description", defs["perspective.combat"].desc
    and defs["perspective.weaponDrawn"].desc and defs["perspective.weaponDrawn"].label)
  check("settings: legacy perspective.weapon no longer a widget", defs["perspective.weapon"] == nil)
  check("settings: applied to cfg", M.cfg.combat == "tpp" and M.cfg.weaponDrawn == "tpp" and M.cfg.aiming == "tpp")
  C.Set("perspective.combat", "fpp"); C.Set("perspective.weaponDrawn", "fpp"); C.Set("perspective.aiming", "tpp")
  check("settings: changed values reach cfg", M.cfg.combat == "fpp" and M.cfg.weaponDrawn == "fpp" and M.cfg.aiming == "tpp"
    and P.Policies(M.cfg) == "combat=FPP weaponDrawn=FPP aiming=TPP")
  check("settings: invalid values rejected", C.Set("perspective.combat", "maybe") == nil and C.Set("perspective.aiming", true) == nil
    and M.cfg.combat == "fpp")
  check("Status() shows the active policies", string.find(M.Status(), "policies=[combat=FPP weaponDrawn=FPP aiming=TPP]", 1, true) ~= nil,
    M.Status())
  local f = io.open(file, "r"); local txt = f:read("*a"); f:close()
  check("settings persisted", string.find(txt, "perspective.combat=fpp", 1, true) and string.find(txt, "perspective.aiming=tpp", 1, true))
  local C2 = loadCore(file)
  local M2 = loadMod()
  M2.AttachCore(C2)
  check("settings restored in the next session", M2.cfg.combat == "fpp" and M2.cfg.aiming == "tpp" and M2.cfg.weaponDrawn == "fpp")
  -- module level: the policies drive Update()
  M.forceMode = "active"
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  M.Update(0, obs())
  M.Update(1, obs({ weaponDrawn = true, aiming = true }))
  check("module: drawn FPP + aiming TPP -> FPP (drawn)", M.Desired().perspective == "FPP" and M.Desired().detail == "weapon drawn")
  C.Set("perspective.weaponDrawn", "tpp")
  M.Update(1.1, obs({ weaponDrawn = true, aiming = true }))
  M.Update(5, obs({ weaponDrawn = true, aiming = true }))
  check("module: drawn TPP + aiming TPP -> aiming stays TPP (experimental rule)", M.Desired().perspective == "TPP")
  -- legacy migration
  local function legacy(value)
    local fl = os.tmpname()
    local h = io.open(fl, "w"); h:write("perspective.weapon=" .. value .. "\nperspective.combat=true\n"); h:close()
    local Cl = loadCore(fl)
    local Ml = loadMod()
    Ml.AttachCore(Cl)
    return Cl, Ml
  end
  local Cl = legacy("drawn")
  check("migration: weapon=drawn -> weaponDrawn FPP, aiming FPP; old boolean combat=true -> default TPP",
    Cl.Get("perspective.weaponDrawn") == "fpp" and Cl.Get("perspective.aiming") == "fpp" and Cl.Get("perspective.combat") == "tpp")
  Cl = legacy("never")
  check("migration: weapon=never -> aiming TPP, weaponDrawn TPP", Cl.Get("perspective.aiming") == "tpp" and Cl.Get("perspective.weaponDrawn") == "tpp")
  Cl = legacy("aim")
  check("migration: weapon=aim -> defaults", Cl.Get("perspective.aiming") == "fpp" and Cl.Get("perspective.weaponDrawn") == "tpp")
  local fl = os.tmpname()
  local h = io.open(fl, "w"); h:write("perspective.weapon=never\nperspective.aiming=fpp\n"); h:close()
  local Cw = loadCore(fl); local Mw = loadMod(); Mw.AttachCore(Cw)
  check("migration: stored new key wins over legacy", Cw.Get("perspective.aiming") == "fpp")
end

-- decisions with the new policies are logged; Status shows the policies
do
  local S = activeMod(true)
  S.st = P.New(S.cfg, { ready = true, wantsTpp = true })
  S.cfg.combat, S.cfg.weaponDrawn, S.cfg.aiming = "tpp", "tpp", "fpp"
  S.Update(0, obs())
  S.Update(1, obs({ combat = 1 }))
  check("policy log: combat TPP policy -> decision stays TPP, no FPP combat line",
    S.Desired().perspective == "TPP" and count(S.logLines, "FPP <- combat") == 0)
  S.Update(4, obs({ combat = 1, weaponDrawn = true, aiming = true }))
  check("policy log: aiming FPP policy logged [active]", count(S.logLines, "FPP <- weapon (aiming) instant [active]") == 1,
    table.concat(S.logLines, " | "))
  check("policy log: Status shows policies and mode", string.find(S.Status(), "mode=active", 1, true)
    and string.find(S.Status(), "policies=[combat=TPP weaponDrawn=TPP aiming=FPP]", 1, true))
  local S2 = activeMod(true)
  S2.st = P.New(S2.cfg, { ready = true, wantsTpp = true })
  S2.cfg.combat = "fpp"
  S2.Update(0, obs())
  S2.Update(1, obs({ combat = 1 }))
  check("policy log: combat FPP policy logged", count(S2.logLines, "FPP <- combat (InCombat) instant [active]") == 1)
end

-- ---------------------------------------------------------------- 0.2.3: aim with melee / fists = block, never FPP
do
  local function aimDetail(over, o)
    local st = P.New(cfg(over), { ready = true, wantsTpp = true })
    P.Step(st, 0, obs())
    st.lastSwitch = -100
    local d = P.Step(st, 1, obs(o))
    return d.perspective, d.detail, d
  end
  check("melee block: default setting on", P.DEFAULTS.meleeBlockTpp == true)
  local p1 = aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" })
  check("melee block: fists + aim -> stays TPP", p1 == "TPP")
  check("melee block: empty WeaponRight type (\"\") + aim with the item flag -> stays TPP",
    aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = "" }) == "TPP")
  for _, t in ipairs({ "Wea_Katana", "Wea_Knife", "Wea_OneHandedClub", "Wea_TwoHandedClub", "Wea_Hammer", "Wea_Machete",
      "Wea_Chainsword", "Wea_Axe", "Wea_LongBlade", "Wea_ShortBlade", "Wea_Sword", "Cyb_MantisBlades", "Cyb_StrongArms",
      "Cyb_NanoWires" }) do
    check("melee block: " .. t .. " + aim -> stays TPP", aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = t }) == "TPP")
  end
  local pp, pd = aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun" })
  check("melee block: pistol + aim -> FPP (aiming)", pp == "FPP" and pd == "aiming")
  check("melee block: ranged types are not melee (rifle, shotgun, sniper, smg, lmg, revolver, unknown)",
    P.MeleeBlock({ weaponType = "Wea_AssaultRifle" }) == nil and P.MeleeBlock({ weaponType = "Wea_Shotgun" }) == nil
    and P.MeleeBlock({ weaponType = "Wea_SniperRifle" }) == nil and P.MeleeBlock({ weaponType = "Wea_SubmachineGun" }) == nil
    and P.MeleeBlock({ weaponType = "Wea_LightMachineGun" }) == nil and P.MeleeBlock({ weaponType = "Wea_Revolver" }) == nil
    and P.MeleeBlock({}) == nil and P.MeleeBlock({ weaponType = "Weird" }) == nil)
  local po, pod = aimDetail({ meleeBlockTpp = false }, { aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" })
  check("melee block: setting off -> fists + aim = aiming (old behaviour, FPP)", po == "FPP" and pod == "aiming")
  check("melee block: setting off -> katana + aim = aiming (old behaviour, FPP)",
    aimDetail({ meleeBlockTpp = false }, { aiming = true, weaponDrawn = true, weaponType = "Wea_Katana" }) == "FPP")
  check("melee block: PSM MeleeWeapon 5..10 + aim (item type unknown / ranged) -> stays TPP",
    aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 8 }) == "TPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun", meleeState = 5 }) == "TPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 10 }) == "TPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 6 }) == "TPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 9 }) == "TPP")
  check("melee block: PSM MeleeWeapon outside 5..10 (idle 2, safe 3, attack 11, default 22) is not a block -> FPP when aiming",
    aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 2 }) == "FPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 3 }) == "FPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 11 }) == "FPP"
    and aimDetail(nil, { aiming = true, weaponDrawn = true, meleeState = 22 }) == "FPP")
  check("melee block: PSM block state + setting off -> aiming again",
    aimDetail({ meleeBlockTpp = false }, { aiming = true, weaponDrawn = true, meleeState = 8 }) == "FPP")
  check("melee block: item tag flag (meleeWeapon) alone also counts",
    aimDetail(nil, { aiming = true, weaponDrawn = true, meleeWeapon = true }) == "TPP")
  check("melee block: MeleeBlock() detail text", P.MeleeBlock({ weaponType = "Wea_Katana", meleeState = 8 }) == "weapon=Wea_Katana, PSM MeleeWeapon=8"
    and P.MeleeBlock({ weaponType = "" }) == "fists")
  -- gadget / arm cyberware are separate rules and still switch (only aiming is a block)
  check("melee block: a gadget thrown while holding melee still -> FPP (gadget rule unchanged)",
    select(2, aimDetail(nil, { aiming = true, weaponDrawn = true, weaponType = "Wea_Katana", gadget = 3 })) == "gadget")
  -- policy matrix: aiming=tpp is TPP anyway; weaponDrawn=fpp still FPP with a melee weapon (the drawn rule, not aiming)
  check("melee block: weaponDrawn=fpp + fists drawn -> FPP (drawn rule, unchanged)",
    select(2, aimDetail({ weaponDrawn = "fpp" }, { aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" })) == "weapon drawn")
  check("melee block: aiming=tpp + pistol -> TPP (policy, unchanged)",
    aimDetail({ aiming = "tpp" }, { aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun" }) == "TPP")
  -- aiming a pistol (FPP), then a melee block: FPP held for the exit delay, then TPP while the block continues
  do
    local st = P.New(cfg(), { ready = true, wantsTpp = true })
    P.Step(st, 0, obs()); st.lastSwitch = -100
    P.Step(st, 1, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun" }))
    local d = P.Step(st, 1.1, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Katana" }))
    check("melee block: pistol aim -> katana block: FPP held for the exit delay", d.perspective == "FPP")
    d = P.Step(st, 3, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Katana" }))
    check("melee block: ... then TPP while the block continues", d.perspective == "TPP")
  end

  -- module level: log line per decision change, Status, settings through utpp_core
  local M = loadMod()
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  M.Update(0, obs())
  M.st.lastSwitch = -100
  check("melee block: Status shows the setting", string.find(M.Status(), "meleeBlock=on(-)", 1, true) ~= nil, M.Status())
  for i = 1, 4 do M.Update(0.1 * i, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" })) end
  check("melee block: Update keeps TPP, exactly one log line while it lasts", M.Desired().perspective == "TPP"
    and count(M.logLines, "aiming ignored: melee block (weapon=Wea_Fists)") == 1, table.concat(M.logLines, " | "))
  check("melee block: Status shows the active decision", string.find(M.Status(), "meleeBlock=on(weapon=Wea_Fists)", 1, true) ~= nil, M.Status())
  M.Update(0.6, obs({ weaponDrawn = true, weaponType = "Wea_Fists" }))
  M.Update(0.7, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" }))
  check("melee block: aim released and held again -> a new log line", count(M.logLines, "aiming ignored: melee block") == 2)
  M.Update(0.8, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Fists", meleeState = 8 }))
  check("melee block: decision detail changed (PSM state added) -> a new log line",
    count(M.logLines, "aiming ignored: melee block (weapon=Wea_Fists, PSM MeleeWeapon=8)") == 1)
  M.cfg.meleeBlockTpp = false
  M.Update(0.9, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Fists" }))
  check("melee block: setting off -> no melee log line, FPP (aiming)", count(M.logLines, "aiming ignored: melee block") == 3
    and M.Desired().perspective == "FPP" and string.find(M.Status(), "meleeBlock=off(-)", 1, true) ~= nil, M.Status())

  local fl = os.tmpname()
  local C = loadCore(fl)
  local S = loadMod()
  S.AttachCore(C)
  check("melee block setting: registered bool, default true, label + description", C.Get("perspective.meleeBlockTpp") == true
    and S.cfg.meleeBlockTpp == true and C.settings.defs["perspective.meleeBlockTpp"].type == "bool"
    and C.settings.defs["perspective.meleeBlockTpp"].label and C.settings.defs["perspective.meleeBlockTpp"].desc)
  C.Set("perspective.meleeBlockTpp", false)
  check("melee block setting: off reaches cfg", S.cfg.meleeBlockTpp == false)
  local h = io.open(fl, "r"); local text = h and h:read("*a") or ""; if h then h:close() end
  check("melee block setting: saved by utpp_core", string.find(text, "perspective.meleeBlockTpp=false", 1, true) ~= nil, text)
  local C2 = loadCore(fl)
  local S2 = loadMod()
  S2.AttachCore(C2)
  check("melee block setting: restored in the next session", S2.cfg.meleeBlockTpp == false)
  os.remove(fl)

end

-- ---------------------------------------------------------------- 0.2.4: scoped weapons use aimingScoped
do
  -- pure detection: scope slot > sniper type > ScopeFOV; melee / fists / unknown / nothing -> nil
  check("scoped: scope part in the slot -> 'scope slot'", P.Scoped({ weaponType = "Wea_Handgun", scopeSlot = true }) == "scope slot (long)")
  check("scoped: sniper rifle type without a part -> 'sniper type'",
    P.Scoped({ weaponType = "Wea_SniperRifle", scopeSlot = false }) == "sniper type")
  check("scoped: precision rifle (iron sights, Kyubi) without a real scope part -> not scoped",
    P.Scoped({ weaponType = "Wea_PrecisionRifle", scopeSlot = false }) == nil and P.Scoped({ weaponType = "Wea_PrecisionRifle" }) == nil
    and P.Scoped({ weaponType = "Wea_PrecisionRifle", scopeSlot = false, scopeFov = 0 }) == nil)
  check("scoped: precision rifle WITH a real scope part -> 'scope slot'",
    P.Scoped({ weaponType = "Wea_PrecisionRifle", scopeSlot = true, scopeKind = "long" }) == "scope slot (long)")
  check("scoped: ScopeFOV sanity guard - pistol / revolver / SMG / shotgun / LMG never scoped by ScopeFOV",
    P.Scoped({ weaponType = "Wea_Handgun", scopeFov = 35 }) == nil and P.Scoped({ weaponType = "Wea_Revolver", scopeFov = 35 }) == nil
    and P.Scoped({ weaponType = "Wea_SubmachineGun", scopeFov = 35 }) == nil and P.Scoped({ weaponType = "Wea_Shotgun", scopeFov = 35 }) == nil
    and P.Scoped({ weaponType = "Wea_ShotgunDual", scopeFov = 35 }) == nil and P.Scoped({ weaponType = "Wea_LightMachineGun", scopeFov = 35 }) == nil
    and P.Scoped({ weaponType = "Wea_Revolver", scopeSlot = true, scopeKind = "short" }) == "scope slot (short)")
  check("scoped: part types default (shortScopeFpp off): Long / TechSniper / PowerSniper yes; Prt_Scope, ShortScope, ScopeRail, Muzzle, nil no",
    P.IsScopePartType("Prt_LongScope") and P.IsScopePartType("Prt_TechSniperScope") and P.IsScopePartType("Prt_PowerSniperScope")
    and not P.IsScopePartType("Prt_ShortScope") and not P.IsScopePartType("Prt_Scope")
    and not P.IsScopePartType("Prt_ScopeRail") and not P.IsScopePartType("Prt_Muzzle") and not P.IsScopePartType(nil)
    and not P.IsScopePartType("?") and not P.IsScopePartType("Prt_ShortScope", false))
  check("scoped: shortScopeFpp = true: short scopes (Prt_ShortScope, generic Prt_Scope) count too; non-scopes still no",
    P.IsScopePartType("Prt_ShortScope", true) and P.IsScopePartType("Prt_Scope", true) and P.IsScopePartType("Prt_LongScope", true)
    and not P.IsScopePartType("Prt_ScopeRail", true) and not P.IsScopePartType(nil, true) and not P.IsScopePartType(5, true))
  check("scoped: ScopeKind long / short / nil", P.ScopeKind("Prt_LongScope") == "long" and P.ScopeKind("Prt_PowerSniperScope") == "long"
    and P.ScopeKind("Prt_TechSniperScope") == "long" and P.ScopeKind("Prt_ShortScope") == "short" and P.ScopeKind("Prt_Scope") == "short"
    and P.ScopeKind("Prt_ScopeRail") == nil and P.ScopeKind(nil) == nil)
  check("scoped: shortScopeFpp default off", P.DEFAULTS.shortScopeFpp == false)
  check("scoped: a short scope the policy does not count (scopeKind short, no scopeSlot) -> its ScopeFOV is not a reason",
    P.Scoped({ weaponType = "Wea_AssaultRifle", scopeSlot = false, scopeKind = "short", scopeFov = 26 }) == nil
    and P.Scoped({ weaponType = "Wea_AssaultRifle", scopeSlot = false, scopeFov = 26 }) == "ScopeFOV")
  check("scoped: ScopeFOV > 0 on an ordinary rifle -> 'ScopeFOV'", P.Scoped({ weaponType = "Wea_AssaultRifle", scopeFov = 35 }) == "ScopeFOV")
  check("scoped: reason priority slot > type > ScopeFOV",
    P.Scoped({ weaponType = "Wea_SniperRifle", scopeSlot = true, scopeFov = 20 }) == "scope slot (long)"
    and P.Scoped({ weaponType = "Wea_SniperRifle", scopeFov = 20 }) == "sniper type")
  check("scoped: none (plain pistol, ScopeFOV 0 / nil / non-number, slot false, unknown type, nil obs)",
    P.Scoped({ weaponType = "Wea_Handgun", scopeSlot = false, scopeFov = 0 }) == nil and P.Scoped({ weaponType = "Wea_Handgun" }) == nil
    and P.Scoped({ weaponType = "Wea_Handgun", scopeFov = "35" }) == nil and P.Scoped({}) == nil and P.Scoped(nil) == nil
    and P.Scoped({ weaponType = "Weird" }) == nil)
  check("scoped: fists and melee types are never scoped (even with a stray slot / ScopeFOV)",
    P.Scoped({ weaponType = "", scopeSlot = true }) == nil and P.Scoped({ weaponType = "Wea_Katana", scopeFov = 30 }) == nil
    and P.Scoped({ weaponType = "Cyb_MantisBlades", scopeSlot = true }) == nil)
  check("scoped: setting default fpp, helper reads tpp / false as TPP",
    P.DEFAULTS.aimingScoped == "fpp" and P.AimingScopedFpp({}) and P.AimingScopedFpp({ aimingScoped = "fpp" })
    and not P.AimingScopedFpp({ aimingScoped = "tpp" }) and not P.AimingScopedFpp({ aimingScoped = false }))

  -- rule selection with both settings (aiming x aimingScoped, scoped / plain weapon)
  local function aim(over, o)
    local st = P.New(cfg(over), { ready = true, wantsTpp = true })
    P.Step(st, 0, obs())
    st.lastSwitch = -100
    local d = P.Step(st, 1, obs(o))
    return d.perspective, d.detail
  end
  local scopedPistol = { aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun", scopeSlot = true }
  local plainPistol = { aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun", scopeSlot = false }
  local pf, pdet = aim({ aiming = "fpp", aimingScoped = "fpp" }, scopedPistol)
  check("scoped rule: aiming=fpp, aimingScoped=fpp, scoped -> FPP (scoped: scope slot (long))", pf == "FPP" and pdet == "aiming (scoped: scope slot (long))", pdet)
  check("scoped rule: aiming=tpp, aimingScoped=fpp, scoped -> FPP", aim({ aiming = "tpp", aimingScoped = "fpp" }, scopedPistol) == "FPP")
  check("scoped rule: aiming=fpp, aimingScoped=tpp, scoped -> TPP", aim({ aiming = "fpp", aimingScoped = "tpp" }, scopedPistol) == "TPP")
  check("scoped rule: aiming=tpp, aimingScoped=tpp, scoped -> TPP", aim({ aiming = "tpp", aimingScoped = "tpp" }, scopedPistol) == "TPP")
  local pa, pad = aim({ aiming = "fpp", aimingScoped = "tpp" }, plainPistol)
  check("scoped rule: plain weapon ignores aimingScoped (aiming=fpp -> FPP, detail 'aiming')", pa == "FPP" and pad == "aiming")
  check("scoped rule: plain weapon, aiming=tpp, aimingScoped=fpp -> TPP",
    aim({ aiming = "tpp", aimingScoped = "fpp" }, plainPistol) == "TPP")
  check("scoped rule: sniper type follows aimingScoped (false -> TPP; default fpp -> FPP even with aiming=tpp)",
    aim({ aimingScoped = false }, { aiming = true, weaponDrawn = true, weaponType = "Wea_SniperRifle" }) == "TPP"
    and aim({ aiming = "tpp" }, { aiming = true, weaponDrawn = true, weaponType = "Wea_SniperRifle" }) == "FPP")
  check("scoped rule: melee block wins over a scope reading (katana with ScopeFOV, aiming=tpp -> TPP)",
    aim({ aiming = "tpp", aimingScoped = "fpp" }, { aiming = true, weaponDrawn = true, weaponType = "Wea_Katana", scopeFov = 20 }) == "TPP")
  check("scoped rule: gadget / arm cyberware still follow aiming, not aimingScoped",
    aim({ aiming = "tpp", aimingScoped = "fpp" }, { gadget = 2, weaponDrawn = true, weaponType = "Wea_SniperRifle" }) == "TPP"
    and aim({ aiming = "fpp", aimingScoped = "tpp" }, { gadget = 2, weaponDrawn = true, weaponType = "Wea_SniperRifle" }) == "FPP")
  check("scoped rule: not aiming -> nothing from aimingScoped", aim({ aiming = "fpp", aimingScoped = "fpp" },
    { aiming = false, weaponDrawn = true, weaponType = "Wea_SniperRifle" }) == "TPP")

  -- cache per weapon: refreshed on item change, not re-read for the same item, failed read not cached
  local R = loadMod()
  local reads = { slot = 0, fov = 0 }
  local function readers(slotV, fovV)
    return { slot = function() reads.slot = reads.slot + 1 return slotV end, fov = function() reads.fov = reads.fov + 1 return fovV end }
  end
  local s1, f1 = R.ScopeInfo("w1", readers(true, 35))
  local s2, f2 = R.ScopeInfo("w1", readers(false, 0))
  check("scope cache: same weapon -> cached values, no second read", s1 == true and f1 == 35 and s2 == true and f2 == 35
    and reads.slot == 1 and reads.fov == 1)
  local s3, f3 = R.ScopeInfo("w2", readers(false, 0))
  check("scope cache: weapon change -> re-read (a false slot result stays false)", s3 == false and f3 == 0 and reads.slot == 2)
  R.ScopeInfo("w2", readers(true, 1))
  check("scope cache: false result is cached too", reads.slot == 2)
  R.ScopeInfo("w3", readers(nil, nil))
  R.ScopeInfo("w3", readers(nil, nil))
  check("scope cache: failed slot read (nil) is not cached -> retried", reads.slot == 4)
  R.ScopeInfo(nil, readers(true, 2))
  R.ScopeInfo(nil, readers(true, 2))
  check("scope cache: no weapon key -> never cached", reads.slot == 6)
  local sb = R.ScopeInfo("w1", readers(true, 9))
  check("scope cache: returning to the first weapon after another one re-reads (single entry)", sb == true and reads.slot == 7)

  -- 0.2.5: the cache holds only plain values (string / number / boolean); a reader returning a game object is dropped
  local fakeObj = setmetatable({}, { __index = function() return 1 end })
  local objReader = { slot = function() return true, fakeObj, fakeObj end, fov = function() return fakeObj end }
  local so, fo, po = R.ScopeInfo("obj1", objReader)
  local okPlain = true
  for _, v in pairs(R.scopeCache or {}) do
    local t = type(v)
    if not (t == "string" or t == "number" or t == "boolean") then okPlain = false end
  end
  check("scope cache: objects returned by a reader never reach the cache (part / fov dropped)", so == true and fo == nil and po == nil
    and R.scopeCache ~= nil and okPlain and R.scopeCache.part == nil and R.scopeCache.fov == nil)
  R.ScopeInfo("p1", { slot = function() return true, "Prt_LongScope", "Items.w_att_scope_long_01" end, fov = function() return 12 end })
  local c = R.scopeCache
  check("scope cache: entry = key / slot / fov / part / partName, all plain", c.key == "p1" and c.slot == true and c.fov == 12
    and c.part == "Prt_LongScope" and c.partName == "Items.w_att_scope_long_01")
  R.ScopeInfo(fakeObj, { slot = function() return true end })
  check("scope cache: a non-string key is never cached (entity handle as key)", R.scopeCache == nil)
  R.ScopeInfo("p2", { slot = function() return true, "Prt_ShortScope" end })
  R.ReleaseHandles()
  check("ReleaseHandles: cache and diagnostic line dropped", R.scopeCache == nil and R.scopeDiagLine == nil)

  -- log line and Status()
  local L = loadMod()
  L.cfg.aiming, L.cfg.aimingScoped = "tpp", "fpp"
  local function aimObs() return obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Handgun",
    weaponRecord = "Items.Preset_Pistol", scopeSlot = true }) end
  L.Update(0, aimObs())
  L.Update(0.1, aimObs())
  check("scoped log: one line while the decision stays the same", count(L.logLines,
    "[utpp_perspective] aiming scoped weapon=Items.Preset_Pistol -> fpp (scope slot (long))") == 1, table.concat(L.logLines, " | "))
  L.cfg.aimingScoped = "tpp"
  L.Update(0.2, aimObs())
  check("scoped log: a new line when the setting changes the result", count(L.logLines,
    "aiming scoped weapon=Items.Preset_Pistol -> tpp (scope slot (long))") == 1)
  L.Update(0.3, obs({ aiming = false, weaponDrawn = true, weaponType = "Wea_Handgun", weaponRecord = "Items.Preset_Pistol", scopeSlot = true }))
  L.Update(0.4, aimObs())
  check("scoped log: aim released and raised again logs again", count(L.logLines, "-> tpp (scope slot (long))") == 2)
  L.Update(0.5, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_Katana", scopeFov = 5 }))
  check("scoped log: melee weapon never logged", count(L.logLines, "weapon=Wea_Katana") == 0)
  L.Update(0.6, obs({ aiming = true, weaponDrawn = true, weaponType = "Wea_SniperRifle" }))
  check("scoped log: record absent -> item type named", count(L.logLines, "aiming scoped weapon=Wea_SniperRifle -> tpp (sniper type)") == 1)
  check("scoped: Status() shows scoped= and aimingScoped=",
    string.find(L.Status(), "scoped=yes(Wea_SniperRifle: sniper type) aimingScoped=tpp", 1, true) ~= nil, L.Status())
  L.Update(0.7, obs({ weaponDrawn = true, weaponType = "Wea_Handgun" }))
  check("scoped: Status() scoped=no for a plain weapon",
    string.find(L.Status(), "scoped=no aimingScoped=tpp", 1, true) ~= nil)
end

-- settings: aimingScoped registered like aiming (choice, default fpp, label), applied to cfg
do
  local file = os.tmpname()
  local C = loadCore(file)
  local M = loadMod()
  M.AttachCore(C)
  local d = C.settings.defs["perspective.aimingScoped"]
  check("settings: perspective.aimingScoped registered (choice fpp|tpp, default fpp, label)", d ~= nil and d.type == "choice"
    and d.default == "fpp" and d.label == "Perspective while aiming with a scope" and #d.choices == 2
    and d.choices[1][1] == "fpp" and d.choices[2][1] == "tpp" and d.desc ~= nil and d.section == "perspective")
  check("settings: aimingScoped default reaches cfg", C.Get("perspective.aimingScoped") == "fpp" and M.cfg.aimingScoped == "fpp")
  C.Set("perspective.aimingScoped", "tpp")
  check("settings: aimingScoped changed -> cfg", M.cfg.aimingScoped == "tpp")
  local ds = C.settings.defs["perspective.shortScopeFpp"]
  check("settings: perspective.shortScopeFpp registered (bool, default false, label, desc)", ds ~= nil and ds.type == "bool"
    and ds.default == false and ds.label ~= nil and ds.desc ~= nil and ds.section == "perspective")
  check("settings: shortScopeFpp default false reaches cfg", C.Get("perspective.shortScopeFpp") == false and M.cfg.shortScopeFpp == false)
  C.Set("perspective.shortScopeFpp", true)
  check("settings: shortScopeFpp changed -> cfg", M.cfg.shortScopeFpp == true)
  os.remove(file)
end

-- ---------------------------------------------------------------- 0.2.6 rule "zoom": utpp_camera's unarmed zoom handoff
do
  -- engine: st.zoomFpp forces FPP instant (rule zoom, reason "unarmed zoom", detail "camera handoff"), exit 0 / instant
  local st = readyTpp()
  st.zoomFpp = true
  local d, ch = P.Step(st, 1, obs({ aiming = true }))
  check("zoom rule: request -> FPP <- unarmed zoom (camera handoff), instant, changed", d.perspective == "FPP" and d.rule == "zoom"
    and d.reason == "unarmed zoom" and d.detail == "camera handoff" and d.style == "instant" and ch == true,
    string.format("%s %s %s %s", d.perspective, tostring(d.rule), tostring(d.detail), tostring(d.style)))
  check("zoom rule: not a lock (toggle stays possible)", st.locked == false)
  st.zoomFpp = false
  st.lastSwitch = -100
  d, ch = P.Step(st, 1.1, obs({ aiming = true }))
  check("zoom rule: cleared -> TPP (user) at once, instant", d.perspective == "TPP" and d.reason == "user" and d.style == "instant"
    and ch == true)
  -- obs.zoomFpp works as well (same rule)
  st = readyTpp()
  d = P.Step(st, 1, obs({ zoomFpp = true }))
  check("zoom rule: obs.zoomFpp input", d.perspective == "FPP" and d.rule == "zoom")
  -- hard: also with the automatic rules off; urgent: ignores minGap
  st = readyTpp({ auto = false })
  st.zoomFpp = true
  check("zoom rule: auto off (perspective mode off) -> still FPP (hard rule)", P.Step(st, 1, obs()).rule == "zoom")
  st = readyTpp()
  st.lastSwitch = 0.9
  st.zoomFpp = true
  check("zoom rule: urgent - switches within minGap", P.Step(st, 1, obs()).perspective == "FPP")
  -- priorities: vehicle / scene win, zoom wins over interactions
  local function winnerOf(o)
    local s = readyTpp()
    s.zoomFpp = true
    P.Step(s, 1, obs(o))
    return P.Step(s, 1.5, obs(o)).rule
  end
  check("zoom priority: vehicle > zoom", winnerOf({ vehicle = 1 }) == "vehicle")
  check("zoom priority: scene > zoom", winnerOf({ high = 4 }) == "scene")
  check("zoom priority: zoom > interaction", winnerOf({ terminal = true }) == "zoom")
  check("zoom priority: zoom > quick FPP / user", winnerOf({}) == "zoom")
  -- session reset drops the request
  st = readyTpp()
  st.zoomFpp = true
  P.Step(st, 1, obs())
  P.Step(st, 1.2, { player = false })
  P.Step(st, 2.0, { player = false })
  check("zoom rule: session lost (player gone >= 0.75 s) -> request dropped", st.session == "lost" and st.zoomFpp == false)
  -- toggle while the handoff holds: deferred (not overridable), the request still wins
  st = readyTpp({ toggleOverrides = true })
  st.zoomFpp = true
  P.Step(st, 1, obs())
  local okT, note = P.Toggle(st)
  d = P.Step(st, 1.1, obs())
  check("zoom rule: toggle with toggleOverrides is only deferred (not overridable), FPP kept", okT == true
    and string.find(tostring(note), "deferred", 1, true) == 1 and d.rule == "zoom", tostring(note))

  -- module API: RequestFppZoom (log, immediate reapply, user event = no minGap wait both ways), ZoomFppRequested, Status
  local M = loadMod()
  M.forceMode = "active"
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  local got = {}
  M.Subscribe(function(dd) got[#got + 1] = dd end)
  M.Update(0, obs())
  M.Update(0.1, obs({ aiming = true }))
  check("RequestFppZoom: API present, nothing requested yet", type(M.RequestFppZoom) == "function" and M.ZoomFppRequested() == false
    and string.find(M.Status(), "zoomFpp=off(0)", 1, true) ~= nil, M.Status())
  local r = M.RequestFppZoom(true, "utpp_camera: test")
  check("RequestFppZoom(true): applied at once on the last observation, subscriber notified, logged",
    r == true and M.Desired().perspective == "FPP" and M.Desired().rule == "zoom" and #got == 2 and got[2].perspective == "FPP"
    and count(M.logLines, "unarmed zoom FPP request ON (utpp_camera: test)") == 1
    and count(M.logLines, "FPP <- unarmed zoom (camera handoff) instant [active]") == 1, table.concat(M.logLines, " | "))
  check("RequestFppZoom: ZoomFppRequested / Status zoomFpp=on(1)", M.ZoomFppRequested() == true
    and string.find(M.Status(), "zoomFpp=on(1)", 1, true) ~= nil and string.find(M.Status(), "zoom*(camera handoff)", 1, true) ~= nil,
    M.Status())
  check("RequestFppZoom(true) again: no second log line", M.RequestFppZoom(true, "again") == true
    and count(M.logLines, "unarmed zoom FPP request ON") == 1)
  M.Update(0.2, obs({ aiming = true }))
  check("RequestFppZoom: held across polls while RMB is held", M.Desired().perspective == "FPP")
  M.RequestFppZoom(false, "utpp_camera: release")
  check("RequestFppZoom(false) 0.1 s after the switch: back to TPP at once (user event, no minGap wait), instant",
    M.Desired().perspective == "TPP" and M.Desired().reason == "user" and M.Desired().style == "instant" and #got == 3
    and count(M.logLines, "unarmed zoom FPP request OFF (utpp_camera: release)") == 1, table.concat(M.logLines, " | "))
  -- the user's FPP choice is respected after the release
  M.SetUserWantsTpp(false, "test")
  M.RequestFppZoom(true, "x")
  M.RequestFppZoom(false, "y")
  check("RequestFppZoom: release returns to the user's choice (FPP stays FPP)", M.Desired().perspective == "FPP"
    and M.Desired().reason == "user")
  -- without any poll yet: the flag is kept for the first decision
  local M2 = loadMod()
  M2.forceMode = "active"
  M2.st = P.New(M2.cfg, { ready = true, wantsTpp = true })
  M2.RequestFppZoom(true, "early")
  M2.Update(0, obs())
  check("RequestFppZoom before the first poll: applied by the first poll", M2.Desired().rule == "zoom")
end

-- ---------------------------------------------------------------- 0.2.7 vehicles (recording 2026-10-22 14:57)
-- CAR-8 (2) / MOTO-5 (2): driver seat / driver combat with the game's TPP vehicle camera while utpp_camera holds the ride
do
  local function seat(o) local st = readyTpp() return P.Step(st, 1, obs(o)), st end
  local d = seat({ mounted = true, vehicle = 6, camHeld = true, vehActive = 5 })
  check("vehicle 0.2.7 MOTO-5 (d): vehicle:6 + vehActive 5 + ride hold -> no FPP (TPP <- vehicle, game TPP camera)",
    d.perspective == "TPP" and d.reason == "vehicle" and d.rule == nil and d.detail == "vehicle:6 (game TPP camera)"
    and d.vehView == 6, tostring(d.perspective) .. " " .. tostring(d.detail))
  d = seat({ mounted = true, vehicle = 6, camHeld = true, vehActive = 0 })
  check("vehicle 0.2.7 MOTO-5 (d): vehActive 0 (game FPP) -> 'vehicle:6'", d.perspective == "FPP" and d.detail == "vehicle:6"
    and d.rule == "vehicle", tostring(d.detail))
  d = seat({ mounted = true, vehicle = 1, camHeld = true, vehActive = 2 })
  check("vehicle 0.2.7 CAR-8 (e): driver seat (1) + TPPMedium + ride hold -> no FPP", d.perspective == "TPP"
    and d.detail == "vehicle:1 (game TPP camera)")
  d = seat({ mounted = true, vehicle = 1, camHeld = true, vehTpp = true })
  check("vehicle 0.2.7: enum unreadable, IsTPPActive true -> no FPP", d.perspective == "TPP")
  d = seat({ mounted = true, vehicle = 1, camHeld = true, vehTpp = false })
  check("vehicle 0.2.7: IsTPPActive false -> 'vehicle:1'", d.perspective == "FPP" and d.detail == "vehicle:1")
  d = seat({ mounted = true, vehicle = 6, camHeld = true })
  check("vehicle 0.2.7: game camera unknown -> 'vehicle:6' (unchanged)", d.perspective == "FPP" and d.detail == "vehicle:6")
  d = seat({ mounted = true, vehicle = 6, camHeld = false, vehActive = 5 })
  check("vehicle 0.2.7: no camera hold (game owns the view) -> 'vehicle:6' (no TPP the camera would take)",
    d.perspective == "FPP" and d.detail == "vehicle:6")
  d = seat({ mounted = true, vehicle = 6, vehActive = 5 })
  check("vehicle 0.2.7: hold unknown (no utpp_camera) -> 'vehicle:6' (unchanged)", d.perspective == "FPP" and d.detail == "vehicle:6")
  d = seat({ mounted = true, vehicle = 3, camHeld = true, vehActive = 2 })
  check("vehicle 0.2.7: passenger (3) keeps the passenger rule", d.perspective == "FPP" and d.detail == "vehicle:3")
  d = seat({ mounted = true, vehicle = 5, camHeld = true, vehActive = 2 })
  check("vehicle 0.2.7: mounted turret (5) unchanged", d.perspective == "FPP" and d.detail == "turret")
  d = seat({ mounted = true, vehicle = 6, camHeld = true, vehActive = 5, combat = 1, weaponDrawn = true, aiming = true })
  check("vehicle 0.2.7: lower rules (combat / aiming FPP policies) stay masked like under the vehicle rule", d.perspective == "TPP"
    and d.detail == "vehicle:6 (game TPP camera)", tostring(d.reason))
  d = seat({ mounted = true, vehicle = 6, camHeld = true, vehActive = 5, replacer = true })
  check("vehicle 0.2.7: higher rules still win (outside)", d.rule == "outside")
  -- seat -> the vehicle TPP lands 0.3 s later: FPP <- vehicle:1, then TPP at once (vehicle latch, minGap not waited)
  local st = readyTpp()
  P.Step(st, 1, obs({ mounted = true, vehicle = 1, camHeld = true, vehActive = 0 }))
  d = P.Step(st, 1.3, obs({ mounted = true, vehicle = 1, camHeld = true, vehActive = 2 }))
  check("vehicle 0.2.7: game camera turns TPP 0.3 s after seating -> TPP at once (no exit delay / minGap wait)",
    d.perspective == "TPP" and d.detail == "vehicle:1 (game TPP camera)")
  d = P.Step(st, 1.4, obs({ mounted = true, vehicle = 1, camHeld = true, vehActive = 0 }))
  check("vehicle 0.2.7: the player picks the game's FPP camera in the ride -> 'vehicle:1' at once", d.perspective == "FPP"
    and d.detail == "vehicle:1")
  -- on foot: every rule unchanged (the vehicle inputs are ignored there)
  st = readyTpp()
  d = P.Step(st, 1, obs({ camHeld = true, vehActive = 2, combat = 1 }))
  check("vehicle 0.2.7 on foot: vehicle inputs ignored, combat rule as before", d.rule == "combat" and st.vehGameTpp == nil)
end

-- Desired() re-checks the camera hold live (the frame in which the hold ends never reads the vehicle-camera TPP)
do
  local M = loadMod()
  M.forceMode = "active"
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  M.Update(0, obs())
  M.Update(1, obs({ mounted = true, vehicle = 6, camHeld = true, vehActive = 5 }))
  local held = true
  M.liveCamHeld = function() return held end
  check("vehicle 0.2.7 Desired: hold on -> TPP (game TPP camera)", M.Desired().perspective == "TPP" and M.Desired().vehView == 6)
  held = false
  local d = M.Desired()
  check("vehicle 0.2.7 Desired: hold ended before the next poll -> FPP 'vehicle:6' live", d.perspective == "FPP" and d.tpp == false
    and d.rule == "vehicle" and d.detail == "vehicle:6" and d.live ~= nil, tostring(d.detail))
  M.liveCamHeld = function() error("camera gone") end
  check("vehicle 0.2.7 Desired: a failing live read keeps the polled decision", M.Desired().perspective == "TPP")
end

-- CAR-7 (1)(3): replay of session.log L1660-1730 (15:00:44-15:00:53): mount (hold ON), seated FPP <- vehicle (vehicle:1)
-- ignored by the camera hold, exit, 'TPP <- user instant' at 546.57 while the hold is still on, unmount 548.16, hold OFF
-- 549.46. The subscriber stands in for the game perspective / representation switch. Times relative (0 = 538.0).
local function car7(over)
  over = over or {}
  local S = activeMod(true)
  local calls = {}
  S.Subscribe(function(d) calls[#calls + 1] = { t = S.lastNow, p = d.perspective, reason = d.reason } end)
  local va = over.vehActive
  local after = { camHeld = true, camTpp = true, tppActive = over.tppActive ~= false }
  replay(S, {
    { 0, 2.1, {} },                                                                      -- on foot (session settles)
    { 2.1, 4.0, { mounted = true, vehicle = 4, camHeld = true } },                       -- 540.14 mount, hold ON
    { 4.0, 4.3, { mounted = true, vehicle = 1, camHeld = true, vehActive = va and 0 or nil } },   -- 542.04 seated
    { 4.3, 8.1, { mounted = true, vehicle = 1, camHeld = true, vehActive = va and 2 or nil } },   -- 542.32 TPPMedium done
    { 8.1, 10.2, { mounted = true, vehicle = 4, camHeld = true } },                      -- 546.11 exit start
    { 10.2, 11.5, after },                                                               -- 548.16 unmount
    { 11.5, 13, { camHeld = false, camTpp = true, tppActive = true } },                  -- 549.46 hold OFF
  })
  local tppCalls = 0
  for _, c in ipairs(calls) do if c.t > 2 and c.p == "TPP" then tppCalls = tppCalls + 1 end end
  return S, calls, tppCalls
end
do
  local S, calls, tppCalls = car7()
  check("vehicle 0.2.7 CAR-7: replay L1676-1700 -> 0 game switch calls for the TPP return (before: 1)", tppCalls == 0, tppCalls)
  check("vehicle 0.2.7 CAR-7: the FPP <- vehicle (vehicle:1) of the seat is still notified", (function()
    for _, c in ipairs(calls) do if c.p == "FPP" and c.reason == "vehicle" then return true end end
    return false end)())
  check("vehicle 0.2.7 CAR-7: decision is TPP internally, logged 'perspective switch suppressed (vehicle hold, rep owned)'",
    S.Desired().perspective == "TPP" and count(S.logLines, "perspective switch suppressed (vehicle hold, rep owned)") == 1
    and count(S.logLines, "TPP <- user instant [active] - perspective switch suppressed") == 1, table.concat(S.logLines, " | "))
  check("vehicle 0.2.7 CAR-7 watchdog: third person active 0.2 s after the unmount -> no switch issued",
    count(S.logLines, "third person active after the unmount - no switch needed") == 1 and (S.stats.watchdog or 0) == 0)
  -- with the vehicle camera read (vehActive): the return happens when TPPMedium lands, also suppressed, nothing at the exit
  local S2, _, tpp2 = car7({ vehActive = true })
  check("vehicle 0.2.7 CAR-7 + vehActive: TPPMedium return suppressed, no switch at the exit", tpp2 == 0
    and count(S2.logLines, "TPP <- vehicle (vehicle:1 (game TPP camera)) instant [active] - perspective switch suppressed") == 1
    and count(S2.logLines, "TPP <- user instant [active]") == 0, tostring(tpp2) .. " " .. table.concat(S2.logLines, " | "))
  -- watchdog: third person still not active 0.2 s after the unmount -> the switch is issued once
  local S3, calls3, tpp3 = car7({ tppActive = false })
  local issuedAt = nil
  for _, c in ipairs(calls3) do if c.p == "TPP" and c.t > 2 then issuedAt = c.t end end
  check("vehicle 0.2.7 CAR-7 watchdog: TPP not active 0.2 s after the unmount -> switch issued once (subscriber)", tpp3 == 1
    and issuedAt ~= nil and issuedAt >= 10.4 - 1e-6 and issuedAt < 10.55
    and count(S3.logLines, "perspective switch watchdog: third person not active 0.20 s after the unmount - switch issued") == 1,
    tostring(issuedAt))
  -- on foot: a user FPP -> TPP still reaches the switch (also with stray vehicle inputs)
  local F = activeMod(true)
  local got = {}
  F.Subscribe(function(d) got[#got + 1] = d.perspective end)
  replay(F, { { 0, 2, {} } })
  F.SetUserWantsTpp(false, "test")
  replay(F, { { 2, 3, { camHeld = true } } })
  F.SetUserWantsTpp(true, "test")
  check("vehicle 0.2.7 on foot: user FPP -> TPP still calls the switch", got[#got] == "TPP" and got[#got - 1] == "FPP"
    and count(F.logLines, "switch suppressed") == 0, table.concat(got, ","))
  -- hold already ended at the return (game camera owned the view): the switch is not suppressed
  local G = activeMod(true)
  local gt = 0
  G.Subscribe(function(d) if d.perspective == "TPP" then gt = gt + 1 end end)
  replay(G, { { 0, 2, {} }, { 2, 3, { mounted = true, vehicle = 1, camHeld = true } },
    { 3, 4, { mounted = true, vehicle = 1, camHeld = false } }, { 4, 5, { mounted = true, vehicle = 4, camHeld = false } },
    { 5, 6, {} } })
  check("vehicle 0.2.7 CAR-7: hold ended before the return -> TPP switch notified as before", gt == 2, gt)
end

-- requirement 7: perspective choices while mounted are for that ride only (perspective.userWantsTpp never written mounted)
do
  local file = os.tmpname()
  local C = loadCore(file)
  local M = loadMod()
  M.AttachCore(C)
  M.forceMode = "active"
  M.st = P.New(M.cfg, { ready = true, wantsTpp = true })
  local function stored()
    local f = io.open(file, "r")
    local txt = f and f:read("*a") or ""
    if f then f:close() end
    return string.match(txt, "perspective%.userWantsTpp=(%a+)")
  end
  local storedBefore = stored()
  M.Update(0, obs())
  M.Update(1, obs({ mounted = true, vehicle = 4 }))
  M.Update(2, obs({ mounted = true, vehicle = 1, camHeld = true, vehActive = 2 }))
  M.ToggleUser("key")
  check("vehicle 0.2.7 req 7: toggle while mounted -> in memory for the ride, perspective.userWantsTpp not written",
    M.st.wantsTpp == false and stored() == storedBefore
    and count(M.logLines, "perspective choice while mounted: not stored") == 1, tostring(stored()))
  M.SetUserWantsTpp(true, "menu")
  M.SetUserWantsTpp(false, "menu")
  check("vehicle 0.2.7 req 7: SetUserWantsTpp while mounted -> not written either", stored() == storedBefore
    and count(M.logLines, "perspective choice while mounted: not stored") == 3)
  M.Update(3, obs({ mounted = true, vehicle = 4 }))
  M.Update(4, obs())
  check("vehicle 0.2.7 req 7: unmount -> the choice from before the mount returns (TPP), logged", M.st.wantsTpp == true
    and count(M.logLines, "unmount: perspective choice of the ride dropped (FPP -> TPP") == 1, table.concat(M.logLines, " | "))
  M.Update(4.6, obs())
  check("vehicle 0.2.7 req 7: on foot afterwards -> TPP <- user", M.Desired().perspective == "TPP" and M.Desired().reason == "user")
  M.ToggleUser("key")
  check("vehicle 0.2.7 req 7: on foot a toggle is stored as before", stored() == "false" and M.st.wantsTpp == false)
  -- pre-entry FPP: a TPP choice in the ride is dropped at the unmount
  M.Update(6, obs({ mounted = true, vehicle = 1 }))
  M.SetUserWantsTpp(true, "menu")
  M.Update(7, obs())
  check("vehicle 0.2.7 req 7: FPP before the ride, TPP chosen in the ride -> FPP again after the unmount, nothing written",
    M.st.wantsTpp == false and stored() == "false")
  -- a load while mounted (player gone >= 0.75 s) ends the ride the same way
  M.Update(8, obs({ mounted = true, vehicle = 1 }))
  M.ToggleUser("key")
  M.Update(8.5, { player = false })
  M.Update(9.5, { player = false })
  check("vehicle 0.2.7 req 7: session reset while mounted -> the earlier choice returns", M.st.wantsTpp == false
    and M.st.vehMounted == false and stored() == "false")
  check("vehicle 0.2.7 Status shows the vehicle state", string.find(M.Status(), "vehicle=[mounted=false", 1, true) ~= nil, M.Status())
  os.remove(file)
end

-- review fix (double switch): a UTPP toggle in the ride neither decides the exit nor is undone after it
do
  -- TPP on foot: held ride shown by the game's TPP vehicle camera; FPP on foot: the vehicle rule's first person ride
  local function ride(wantsTpp)
    local S = activeMod(wantsTpp)
    local tr = {}
    local seat = wantsTpp and { mounted = true, vehicle = 1, camHeld = true, vehActive = 2 } or { mounted = true, vehicle = 1 }
    local trans = { mounted = true, vehicle = 4, camHeld = wantsTpp or nil }
    replay(S, { { 0, 2, {} }, { 2, 3, trans }, { 3, 5, seat } }, tr)
    S.ToggleUser("key")
    local inRide = S.st.wantsTpp
    replay(S, { { 5, 5.5, seat } }, tr)
    local d = S.Desired()
    local tr2 = {}
    replay(S, { { 5.5, 6.5, trans } }, tr2)
    local hold = S.st.vehHold
    replay(S, { { 6.5, 9, {} } }, tr2)
    local flips, last = 0, nil
    for _, e in ipairs(tr2) do
      if last ~= nil and e.p ~= last then flips = flips + 1 end
      last = e.p
    end
    local seq = {}
    for _, e in ipairs(tr2) do if #seq == 0 or seq[#seq] ~= e.p then seq[#seq + 1] = e.p end end
    return { S = S, inRide = inRide, d = d, hold = hold, flips = flips, seq = table.concat(seq, ">"), last = last }
  end
  local a = ride(true)
  check("review double switch: TPP on foot, UTPP toggle to FPP in the ride -> exit kept third person (vehHold exit), "
    .. "Desired().wantsTpp = the choice from before the ride", a.inRide == false and a.d.wantsTpp == true
    and a.d.rideWantsTpp == false and a.hold == "exit", string.format("%s %s %s", tostring(a.inRide),
    tostring(a.d.wantsTpp), tostring(a.hold)))
  check("review double switch: TPP on foot + in-ride toggle -> no perspective switch from the exit start to on foot",
    a.flips == 0 and a.last == "TPP" and a.S.st.wantsTpp == true, a.seq)
  local b = ride(false)
  check("review double switch: FPP on foot, toggle to TPP in the ride -> exit not held, Desired().wantsTpp false",
    b.inRide == true and b.d.wantsTpp == false and b.hold == nil, string.format("%s %s %s", tostring(b.inRide),
    tostring(b.d.wantsTpp), tostring(b.hold)))
  check("review double switch: FPP on foot + in-ride toggle -> no perspective switch from the exit start to on foot",
    b.flips == 0 and b.last == "FPP" and b.S.st.wantsTpp == false, b.seq)
  -- on foot (never mounted) Desired().wantsTpp is the toggled choice as before
  local F = activeMod(true)
  replay(F, { { 0, 2, {} } })
  F.ToggleUser("key")
  check("review double switch: on foot Desired().wantsTpp follows the toggle as before", F.Desired().wantsTpp == false
    and F.Desired().rideWantsTpp == false)
end

-- MOTO-2 hook: Game.UTPPSetVehicleTppAnim(bool), pcall + nil-safe, follows the view in the driver seat
do
  local savedGame = Game
  local calls = {}
  Game = { UTPPSetVehicleTppAnim = function(on) calls[#calls + 1] = on end }
  local S = activeMod(true)
  replay(S, {
    { 0, 2, {} },
    { 2, 3, { mounted = true, vehicle = 4, camHeld = true } },
    { 3, 3.3, { mounted = true, vehicle = 1, camHeld = true, vehActive = 0 } },  -- entry TPP request still pending
    { 3.3, 5, { mounted = true, vehicle = 1, camHeld = true, vehActive = 2 } },
  })
  check("vehicle 0.2.7 MOTO-2: on foot / mount transition / pending seat FPP -> no call, TPP view -> true",
    #calls == 1 and calls[1] == true, #calls)
  replay(S, { { 5, 6, { mounted = true, vehicle = 1, camHeld = true, vehActive = 0 } } })
  check("vehicle 0.2.7 MOTO-2: FPP choice while mounted calls UTPPSetVehicleTppAnim(false)", #calls == 2 and calls[2] == false
    and S.Desired().perspective == "FPP" and count(S.logLines, "vehicle TPP animation -> off (first person chosen in the ride)") == 1)
  replay(S, { { 6, 7, { mounted = true, vehicle = 6, camHeld = true, vehActive = 5 } } })
  check("vehicle 0.2.7 MOTO-2: return to TPP calls it with true (once)", #calls == 3 and calls[3] == true)
  replay(S, { { 7, 8, { mounted = true, vehicle = 4, camHeld = true } }, { 8, 9, {} }, { 9, 10, { mounted = true, vehicle = 3 } } })
  check("vehicle 0.2.7 MOTO-2: exit / unmount / passenger seat -> no call, state reset", #calls == 3 and S.vehAnim == nil)
  -- a ride that stays first person from the seat: false after VEH_ANIM_FPP_WAIT s
  local S2 = activeMod(true)
  local n0 = #calls
  replay(S2, { { 0, 2, {} }, { 2, 3.2, { mounted = true, vehicle = 1, vehActive = 0 } } })
  check("vehicle 0.2.7 MOTO-2: first person 1.0 s after seating -> false once", #calls == n0 + 1 and calls[#calls] == false)
  -- game camera unreadable: the decision decides (FPP 'vehicle:1' -> after the wait false)
  local S4 = activeMod(true)
  n0 = #calls
  replay(S4, { { 0, 2, {} }, { 2, 3.2, { mounted = true, vehicle = 1, camHeld = true } } })
  check("vehicle 0.2.7 MOTO-2: game camera unreadable -> the FPP decision decides after the wait (false once)",
    #calls == n0 + 1 and calls[#calls] == false)
  -- no native: nil-safe, one log line, no error
  Game = nil
  local S3 = activeMod(true)
  local okR, errR = pcall(replay, S3, { { 0, 2, {} }, { 2, 4, { mounted = true, vehicle = 1, camHeld = true, vehActive = 2 } },
    { 4, 5, { mounted = true, vehicle = 1, camHeld = true, vehActive = 0 } } })
  check("vehicle 0.2.7 MOTO-2: no Game / native -> no error, unavailable logged once", okR
    and count(S3.logLines, "Game.UTPPSetVehicleTppAnim unavailable") == 1, tostring(errR))
  Game = savedGame
end

outPrint(string.format("utpp_perspective tests: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_perspective tests failed")

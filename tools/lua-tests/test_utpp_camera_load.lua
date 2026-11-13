-- UTPP utpp_camera 0.1.2 / utpp_probe: CET load and wiring test with a mocked game (no real engine).
-- Loads the real utpp_core (settings registry, bindings) and utpp_camera init.lua in a CET-like environment
-- (registerForEvent / registerInput / Observe / GetMod after load) and drives onInit / onUpdate frames:
--   A) full wiring: settings + action handlers registered, toggle -> representation event, utppCamera.Activate,
--      boom behind the right shoulder, mouse calibration, free look (heading lock), collision with a wall, shoulder swap,
--      profile next (no-op: one fixed profile), quick FPP, weapon cut, vehicle cut, menu freeze, toggle back (FPP camera, representation off,
--      utppCamera.Deactivate), no update errors;
--   B) "Use Ultimate Third Person" (camera.impl) off: key ignored with a log line; "on" enables it; stored "auto" migrates to on;
--   C) utppCamera missing (no ArchiveXL patch): logged once, stays first person;
--   D) utpp_core missing: defaults, console API works;
--   E) utpp_probe: Cycle() dumps / diff / representation events, CamSpike takeover check;
--   G) 0.1.1 TPP signal (Game.UTPPSetTppOverride on enter / exit / first frame / session reset / shutdown, fallback route
--      UTPPPerspectiveState, failure retry with one log line) and the pose nudge ("StandEnter" after the full body);
--   J) 0.1.4 over-the-shoulder aim: enter / exit through PSM UpperBody + WeaponRight + perspective.aiming, weapon AimFOV
--      stat, forced follow, auto-swap at a wall, melee / vehicle hold / settings / menu, logs and Status().
--   K) 0.1.4 aim occlusion (level A): camera ray -> P, muzzle (GetMuzzleSlotWorldTransform) -> P ray through the mocked
--      raycast, cover hit only by the muzzle ray -> blocked after 0.1 s, clear after 0.1 s, P on a wall (tolerance),
--      exit tween, camera.aim.occlusion off, unreadable muzzle, 30 Hz throttle, 144 fps edge, AimBlocked via GetMod.
--   M) 0.1.6 unarmed zoom with the real utpp_perspective: RMB with empty hands -> zoom state, wheel steps (game ZoomIn /
--      ZoomOut actions, value / type edge), first person handoff (RequestFppZoom, rule zoom), RMB release -> third person,
--      key fallback, weapon / dodge / setting off, session reset during the handoff; N) the same without utpp_perspective.
-- Globals from the runner: UTPP_CAMERA_PATH, UTPP_CAMERA_DIR, UTPP_CAMERA_MATH_PATH, UTPP_CORE_PATH, UTPP_PROBE_PATH.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, (not ok and detail) and (" -- " .. tostring(detail)) or ""))
end
local CM = dofile(UTPP_CAMERA_MATH_PATH)
package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. package.path
local TMP = (os.getenv("TEMP") or os.getenv("TMP") or ".") .. "\\"
local savedPrint = print

-- ================================================================ mocked game
local function makeWorld(opts)
  opts = opts or {}
  local W = { pos = CM.v3(100, 50, 10), yaw = 0, speed = 0, psm = {}, menu = false, photo = false, weapon = false,
              wallY = nil, active = "fpp", events = {}, features = {}, hasComp = opts.hasComp ~= false, fppPitch = -5,
              player = true, observers = {}, inputs = {}, logged = {}, lines = {}, rays = 0,
              core = nil, cameraMod = nil, onRep = nil, tppCalls = {}, tppReds = true, sysCalls = {}, pushed = {},
              system = nil, wallX = nil, weaponItem = nil, records = {}, statValues = {}, statCalls = 0 }
  local function playerQ() return CM.qFromYawPitch(W.yaw, 0) end
  -- W.eyeDrop: the game's FPP animation set lowers the FPP camera (melee / block / vehicle exit), camera round 3
  local function eye() return CM.add(W.pos, CM.qrot(playerQ(), CM.v3(0, 0.05, 1.62 - (W.eyeDrop or 0)))) end
  W.eye = eye
  local utpp = { lp = CM.v3(0, 0, 0), lq = CM.q(0, 0, 0, 1), fov = 60, activations = 0, deactivations = 0, writes = 0 }
  W.utpp = utpp
  local function utppWorld()
    return CM.add(W.pos, CM.qrot(playerQ(), utpp.lp)), CM.qmul(playerQ(), utpp.lq)
  end
  W.utppWorld = utppWorld
  local function rowsMatrix(pos, q)
    local r, f, u = CM.qrot(q, CM.v3(1, 0, 0)), CM.qrot(q, CM.v3(0, 1, 0)), CM.qrot(q, CM.v3(0, 0, 1))
    return { X = { x = r.x, y = r.y, z = r.z, w = 0 }, Y = { x = f.x, y = f.y, z = f.z, w = 0 },
             Z = { x = u.x, y = u.y, z = u.z, w = 0 }, W = { x = pos.x, y = pos.y, z = pos.z, w = 1 } }
  end
  function utpp:Activate(b, audio) self.activations = self.activations + 1; W.active = "utpp" end
  function utpp:Deactivate() self.deactivations = self.deactivations + 1; if W.active == "utpp" then W.active = "none" end end
  function utpp:SetLocalTransform(p, q) self.lp = CM.v3(p.x, p.y, p.z); self.lq = CM.q(q.i, q.j, q.k, q.r); self.writes = self.writes + 1 end
  function utpp:SetLocalPosition(p) self.lp = CM.v3(p.x, p.y, p.z) end
  function utpp:SetFOV(f) self.fov = f end
  function utpp:GetFOV() return self.fov end
  function utpp:GetLocalToWorld() local p, q = utppWorld(); return rowsMatrix(p, q) end
  function utpp:GetLocalPosition() return self.lp end
  function utpp:GetLocalOrientation() return self.lq end
  function utpp:GetClassName() return "gameCameraComponent" end
  function utpp:IsEnabled() return true end
  local fpp = { headingLocked = false, activations = 0, lockedEver = false }
  W.fpp = fpp
  function fpp:Activate() self.activations = self.activations + 1; W.active = "fpp" end
  function fpp:Deactivate() end
  -- 2026-10-12 round 3: W.look = the game's look yaw when it differs from V's yaw (S: game look sync), nil = V's yaw
  function fpp:GetLocalToWorld() return rowsMatrix(eye(), CM.qFromYawPitch(W.look or W.yaw, W.fppPitch)) end
  function fpp:GetFOV() return 51 end
  function fpp:GetLocalPosition() return CM.v3(0, 0, 0) end
  function fpp:GetLocalOrientation() return CM.q(0, 0, 0, 1) end
  function fpp:GetClassName() return "gameFPPCameraComponent" end
  function fpp:IsEnabled() return true end
  -- 2026-10-12 world camera (opts.wcam): Codeware StaticEntitySystem mock and the spawned entity (world origin, identity)
  -- whose camera component's local transform is the world pose. W.wcSpace = offset of the read-back world position
  -- (a wrong entity space), W.wcNoRender = Activate does not make it render, W.wcLost = GetEntity returns nil,
  -- W.wcSpawnId = id returned by SpawnEntity, W.sesReady = IsReady(); every call is counted in W.ses.calls.
  local ses = { calls = 0, spawns = 0, despawns = 0, tagged = 0, id = nil, at = nil, delay = opts.wcDelay or 0.1 }
  W.ses = ses
  local wc = { wp = CM.v3(0, 0, 0), wq = CM.q(0, 0, 0, 1), fov = 60, activations = 0, deactivations = 0, writes = 0 }
  W.wc = wc
  function wc:Activate() ses.calls = ses.calls + 1; self.activations = self.activations + 1; if not W.wcNoRender then W.active = "wc" end end
  function wc:Deactivate() ses.calls = ses.calls + 1; self.deactivations = self.deactivations + 1; if W.active == "wc" then W.active = "none" end end
  function wc:SetLocalTransform(p, q) ses.calls = ses.calls + 1; self.wp = CM.v3(p.x, p.y, p.z); self.wq = CM.q(q.i, q.j, q.k, q.r); self.writes = self.writes + 1 end
  function wc:SetFOV(f) ses.calls = ses.calls + 1; self.fov = f end
  function wc:SetIsHighPriority(on) ses.calls = ses.calls + 1; self.prio = on end
  function wc:GetLocalToWorld() ses.calls = ses.calls + 1; return rowsMatrix(W.wcSpace and CM.add(self.wp, W.wcSpace) or self.wp, self.wq) end
  local wcEnt = {}
  W.wcEnt = wcEnt
  function wcEnt:FindComponentByName(n) ses.calls = ses.calls + 1; if n == "utppCamera" then return wc end return nil end
  function ses:IsReady() self.calls = self.calls + 1; return W.sesReady ~= false end
  function ses:DespawnTagged(tag) self.calls = self.calls + 1; self.tagged = self.tagged + 1; self.tag = tag end
  function ses:SpawnEntity(spec)
    self.calls = self.calls + 1
    self.spawns = self.spawns + 1
    self.spec = spec
    self.id = W.wcSpawnId or { hash = 9000 + self.spawns }
    self.at = W.clock or 0
    return self.id
  end
  function ses:GetEntity(id)
    self.calls = self.calls + 1
    if W.wcLost or self.id == nil or id == nil or id.hash ~= self.id.hash then return nil end
    if (W.clock or 0) < self.at + self.delay then return nil end
    return wcEnt
  end
  function ses:DespawnEntity(id) self.calls = self.calls + 1; self.despawns = self.despawns + 1; self.id = nil; return true end
  -- body components for the probe
  local function comp(name, cls, en)
    local c = { n = name, cls = cls, en = en, mesh = { hash = "H_" .. name }, meshAppearance = "default", chunkMask = 255 }
    function c:GetClassName() return self.cls end
    function c:GetName() return self.n end
    function c:IsEnabled() return self.en end
    function c:GetAppearanceName() return "app_" .. self.n end
    return c
  end
  W.bodyComps = { comp("t0_000_pwa_fpp__torso", "entGarmentSkinnedMeshComponent", true),
                  comp("t0_000_pwa_base__full", "entMorphTargetSkinnedMeshComponent", false),
                  comp("right_arm", "entSkinnedMeshComponent", true) }
  W.onRep = function(on) W.bodyComps[1].en = not on; W.bodyComps[2].en = on end
  local player = {}
  W.playerObj = player
  function player:GetEntityID() return { hash = 4242 } end
  function player:FindComponentByName(n)
    if n == "utppCamera" then return W.hasComp and utpp or nil end
    if n == "camera" then return fpp end
    return nil
  end
  function player:GetWorldPosition() return { x = W.pos.x, y = W.pos.y, z = W.pos.z, w = 1 } end
  function player:GetWorldOrientation() return playerQ() end
  function player:GetWorldForward() local f = CM.forward(W.yaw, 0); return { x = f.x, y = f.y, z = f.z, w = 0 } end
  function player:GetVelocity() local f = CM.forward(W.yaw, 0); return { x = f.x * W.speed, y = f.y * W.speed, z = 0 } end
  function player:QueueEvent(e)
    W.events[#W.events + 1] = e.className
    if e.className == "gameActivateTPPRepresentationEvent" then W.onRep(true) end
    if e.className == "gameDeactivateTPPRepresentationEvent" then W.onRep(false) end
  end
  function player:GetComponents() local r = {} for i, c in ipairs(W.bodyComps) do r[i] = c end r[#r + 1] = fpp; return r end
  function player:GetResolvedGenderName() return "Female" end
  function player:GetTemplatePath() return { hash = 777 } end
  function player:GetCurrentAppearanceName() return "default" end

  local function bb(src)
    return { GetInt = function(_, k) return src[k] or 0 end, GetBool = function(_, k) return src[k] == true end }
  end
  local camSys = {}
  function camSys:GetActiveCameraForward()
    if W.active == "utpp" then local _, q = utppWorld(); return CM.qrot(q, CM.v3(0, 1, 0)) end
    if W.active == "wc" then return CM.qrot(wc.wq, CM.v3(0, 1, 0)) end
    return CM.forward(W.yaw, W.fppPitch)
  end
  function camSys:GetActiveCameraFOV() return W.active == "utpp" and utpp.fov or (W.active == "wc" and wc.fov) or 80 end
  function camSys:GetActiveCameraWorldTransform(xf)
    if xf then xf.position = (W.active == "utpp") and (utppWorld()) or (W.active == "wc" and wc.wp) or eye() end
    return true
  end
  local sqs = {}
  -- 0.1.4 aim occlusion: axis-aligned boxes (cover) in the Static group, nearest box hit (slab test) wins
  local function segBox(from, to, bx)
    local tmin, tmax = 0, 1
    for _, ax in ipairs({ "x", "y", "z" }) do
      local dd = to[ax] - from[ax]
      if math.abs(dd) < 1e-12 then
        if from[ax] < bx.min[ax] or from[ax] > bx.max[ax] then return nil end
      else
        local t1, t2 = (bx.min[ax] - from[ax]) / dd, (bx.max[ax] - from[ax]) / dd
        if t1 > t2 then t1, t2 = t2, t1 end
        if t1 > tmin then tmin = t1 end
        if t2 < tmax then tmax = t2 end
        if tmin > tmax then return nil end
      end
    end
    return tmin
  end
  function sqs:SyncRaycastByCollisionGroup(from, to, group, s, d)
    W.rays = W.rays + 1
    if W.boxes and group == "Static" then
      local best = nil
      for _, bx in ipairs(W.boxes) do
        local t = segBox(from, to, bx)
        if t and (best == nil or t < best) then best = t end
      end
      if best then
        W.boxHits = (W.boxHits or 0) + 1
        return true, { position = { x = from.x + (to.x - from.x) * best, y = from.y + (to.y - from.y) * best,
                                    z = from.z + (to.z - from.z) * best } }
      end
    end
    if W.wallY and group == "Static" then
      local a, b = from.y - W.wallY, to.y - W.wallY
      if a ~= 0 and b == 0 then -- 0.1.4: segment ending on the wall (aim point P on it) counts as a hit at the end
        return true, { position = { x = to.x, y = W.wallY, z = to.z } }
      end
      if (a > 0) ~= (b > 0) then
        local t = a / (a - b)
        return true, { position = { x = from.x + (to.x - from.x) * t, y = W.wallY, z = from.z + (to.z - from.z) * t } }
      end
    end
    if W.wallX and group == "Static" then -- 0.1.4: wall beside V (plane x = wallX) for the aim auto-swap
      local a, b = from.x - W.wallX, to.x - W.wallX
      if (a > 0) ~= (b > 0) then
        local t = a / (a - b)
        return true, { position = { x = W.wallX, y = from.y + (to.y - from.y) * t, z = from.z + (to.z - from.z) * t } }
      end
    end
    return false, { position = { x = 0, y = 0, z = 0 } }
  end
  local ts = {}
  local chestItem = { }
  function chestItem:GetItemID() return { tdb = "Items.TestJacket" } end
  function chestItem:GetCurrentAppearanceName() return "jacket_&FPP" end
  function chestItem:GetTemplatePath() return { hash = 9 } end
  function chestItem:GetComponents() return { comp("jacket_mesh", "entGarmentSkinnedMeshComponent", true) } end
  function ts:GetItemInSlot(_, slot)
    if slot == "AttachmentSlots.WeaponRight" then return W.weapon and (W.weaponItem or {}) or nil end
    if slot == "AttachmentSlots.Chest" then return chestItem end
    return nil
  end
  function ts:GetItemAppearance() return "jacket_&FPP" end

  W.env = {
    registerForEvent = function(ev, fn) W.handlers[ev] = fn end,
    registerInput = function(id, label, fn) W.inputs[#W.inputs + 1] = id end, -- stub: utpp_camera registers no bindings (checked in A)
    Observe = function(cls, fn, cb) W.observers[cls .. "." .. fn] = cb end,
    GetMod = function(n)
      if n == "utpp_core" then return W.core end
      if n == "utpp_camera" then return W.cameraMod end
      return nil
    end,
    Game = {
      GetPlayer = function() W.playerCalls = (W.playerCalls or 0) + 1 return W.player and player or nil end,
      GetCameraSystem = function() return camSys end,
      GetStaticEntitySystem = opts.wcam and function() return ses end or nil,
      GetSpatialQueriesSystem = function() return sqs end,
      GetTransactionSystem = function() return ts end,
      GetBlackboardSystem = function()
        return { GetLocalInstanced = function() return bb(W.psm) end,
                 Get = function(_, d) return bb({ IsInMenu = W.menu }) end }
      end,
      GetPhotoModeSystem = function() return { IsPhotoModeActive = function() return W.photo end } end,
      GetSystemRequestsHandler = function() return { IsGamePaused = function() return false end } end,
      GetMountedVehicle = function() return nil end,
      GetStatsSystem = function()
        return { GetStatValue = function(_, eid, stat)
          W.statCalls = W.statCalls + 1
          if stat ~= "AimFOV" then error("unexpected stat " .. tostring(stat)) end
          return W.statValues[eid.hash] or 0
        end }
      end,
      GetScriptableSystemsContainer = function()
        return { Get = function(_, n) if n == "UTPPPerspectiveState" then return W.system end return nil end }
      end,
      NameToString = function(n) return tostring(n) end,
      UTPPLogLine = function(l) W.logged[#W.logged + 1] = l end,
      UTPPSetTppOverride = function(on)
        if W.tppReds == nil then error("attempt to call field 'UTPPSetTppOverride' (a nil value)") end
        W.tppCalls[#W.tppCalls + 1] = on
        return W.tppReds
      end,
    },
    Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end },
    Quaternion = { new = function(i, j, k, r) return { i = i, j = j, k = k, r = r } end },
    Transform = { new = function() return {} end },
    StaticEntitySpec = { new = function() return {} end },
    TweakDBID = { new = function(s) return s end },
    TDBID = { ToStringDEBUG = function(t) return t end },
    ItemID ={ GetTDBID = function(id) return id.tdb end },
    TweakDBInterface = { GetWeaponItemRecord = function(t)
      local ty = W.records[t]
      if ty == nil then return nil end
      return { ItemType = function() return { Type = function() return ty end } end }
    end },
    gamedataStatType = { AimFOV = "AimFOV" },
    ResourceAsyncRef = { GetHash = function(r) return r.hash end },
    NewObject = function(n) return { className = n } end,
    AnimationControllerComponent = { ApplyFeature = function(p, name, feat) W.features[#W.features + 1] = { name, feat.IsActive } end,
                                     PushEvent = function(p, name) W.pushed[#W.pushed + 1] = name end },
    ListenerAction = { GetName = function(a) return a.name end, GetValue = function(a) return a.value end },
    GetAllBlackboardDefs = function()
      return { PlayerStateMachine = { Vehicle = "Vehicle", IsInWorkspot = "IsInWorkspot", HighLevel = "HighLevel",
                                      IsControllingDevice = "IsControllingDevice", LocomotionDetailed = "LocomotionDetailed",
                                      Locomotion = "Locomotion", UpperBody = "UpperBody", MeleeWeapon = "MeleeWeapon",
                                      BodyCarrying = "BodyCarrying", Carrying = "Carrying",
                                      Takedown = "Takedown", Swimming = "Swimming", MountedToVehicle = "MountedToVehicle" },
               UI_System = { IsInMenu = "IsInMenu" }, UI_PlayerStats = { isReplacer = "isReplacer" },
               Braindance = { IsActive = "IsActive" } }
    end,
    print = function(...)
      local parts = {}
      for i = 1, select("#", ...) do parts[#parts + 1] = tostring(select(i, ...)) end
      W.lines[#W.lines + 1] = table.concat(parts, " ")
    end,
  }
  W.handlers = {}
  return W
end

local GLOBALS = { "registerForEvent", "registerInput", "Observe", "GetMod", "Game", "Vector4", "Quaternion", "Transform",
                  "TweakDBID", "TDBID", "ItemID", "ResourceAsyncRef", "NewObject", "AnimationControllerComponent",
                  "ListenerAction", "GetAllBlackboardDefs", "print", "TweakDBInterface", "gamedataStatType",
                  "StaticEntitySpec", "ResRef", "EntityID", "exEntitySpawner", "WorldTransform" }
local function withEnv(W, fn)
  local saved = {}
  for _, g in ipairs(GLOBALS) do saved[g] = _G[g]; _G[g] = W.env[g] end
  local ok, err = pcall(fn)
  for _, g in ipairs(GLOBALS) do _G[g] = saved[g] end
  if not ok then error(err, 0) end
end

local FIXTURE_STORED = {
  ["camera.trace"] = "true", -- release default is off (2026-10-22); the camTrace assertions below need the log lines
  ["camera.startTpp"] = "false", ["camera.predictBodyTurn"] = "true", ["camera.aim.distance"] = "1.1", ["camera.aim.boomFrame"] = "0",
  ["camera.zoomUnarmed.boomFrame"] = "0", ["camera.aim.side"] = "0.45", ["camera.aim.fov"] = "42", ["camera.zoomUnarmed.steps"] = "4",
  ["camera.zoomUnarmed.distance"] = "0.9", ["camera.zoomUnarmed.lastFov"] = "28",
  ["camera.distance"] = "2.1", ["camera.height"] = "0.12", ["camera.fov"] = "56",
  ["perspective.aiming"] = "fpp", ["perspective.aimingScoped"] = "fpp", ["perspective.combatCut"] = "true",
  ["perspective.takedown"] = "true", ["perspective.workspot"] = "true", ["perspective.interaction"] = "true",
}
-- the original "Medium" profile (registered default of profile 2 before 2026-10-20), matches the stored fixture values above
local MEDIUM = { name = "Medium", distance = 2.10, height = 0.12, side = 0.50, fov = 56, sens = 1.0 }
local settingsFile = TMP .. "utpp_camera_test_settings.txt"
local function loadCore(W)
  local C
  withEnv(W, function()
    registerForEvent = nil -- pure core (its own wiring is covered by test_utpp_core.lua)
    registerInput = nil
    C = dofile(UTPP_CORE_PATH)
  end)
  C.SETTINGS_FILE = settingsFile
  C.sink = function(line) W.lines[#W.lines + 1] = line end
  -- Fixture values: the registration defaults were changed to the current settings (2026-10-20); the scenarios below were written
  -- against the earlier ones, so they are stored here (stored values win over defaults). The new defaults themselves are
  -- asserted in test_utpp_user_defaults.lua and by the "registered" checks of J) / M) via defs[key].default.
  for k, v in pairs(FIXTURE_STORED) do C.settings.stored[k] = v end
  W.core = C
  return C
end
local function loadCamera(W)
  local cam
  withEnv(W, function() cam = dofile(UTPP_CAMERA_PATH) end)
  W.cameraMod = cam
  return cam
end
local function frames(W, secs, fps, perFrame)
  fps = fps or 60
  withEnv(W, function()
    for i = 1, math.floor(secs * fps + 0.5) do
      W.clock = (W.clock or 0) + 1 / fps -- 2026-10-12: mock time (world camera spawn delay)
      if perFrame then perFrame(i) end
      W.handlers.onUpdate(1 / fps)
    end
  end)
end
local function count(lines, needle)
  local n = 0
  for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end
local function countEv(W, name) local n = 0 for _, e in ipairs(W.events) do if e == name then n = n + 1 end end return n end
local function action(W, name, value) withEnv(W, function() W.observers["PlayerPuppet.OnAction"](nil, { name = name, value = value }) end) end

-- 0.1.5 handles: walk tables (keys + values) and the upvalues of functions DEFINED IN the module under test from the roots;
-- returns the paths of objects that are in `mocks` (a set of mocked game objects). Other modules' functions are not entered
-- (their test sinks reference the mock world).
local function findMockRefs(roots, mocks, srcPart)
  local seen, found = {}, {}
  local function visit(v, path)
    local t = type(v)
    if t == "userdata" then found[#found + 1] = path .. "<userdata>" return end
    if t ~= "table" and t ~= "function" then return end
    if seen[v] then return end
    seen[v] = true
    if mocks[v] then found[#found + 1] = path return end
    if t == "table" then
      for k, x in pairs(v) do
        visit(k, path .. "<key>")
        visit(x, path .. "." .. tostring(k))
      end
    else
      local info = debug.getinfo(v, "S")
      if info and info.source and string.find(info.source, srcPart, 1, true) then
        local i = 1
        while true do
          local n, x = debug.getupvalue(v, i)
          if not n then break end
          visit(x, path .. "^" .. n)
          i = i + 1
        end
      end
    end
  end
  for name, r in pairs(roots) do visit(r, name) end
  return found
end

-- ================================================================ A) full wiring
do
  local W = makeWorld()
  local C = loadCore(W)
  local cam = loadCamera(W)
  check("A: utpp_camera loads in a CET-like environment", type(cam) == "table" and cam.VERSION == "0.1.6")
  check("A: onInit / onUpdate / onShutdown registered at load", type(W.handlers.onInit) == "function"
        and type(W.handlers.onUpdate) == "function" and type(W.handlers.onShutdown) == "function")
  check("A: no CET input bindings registered while loading (key-binding system removed)", #W.inputs == 0)
  withEnv(W, function() W.handlers.onInit() end)
  check("A: settings registered through utpp_core", C.settings.defs["camera.impl"] ~= nil and C.settings.defs["camera.fov"] ~= nil
        and C.settings.defs["camera.pitchSource"] ~= nil and C.Effective("camera.impl") == true)
  check("A: no key-binding API left on utpp_camera", cam.FreeLookKey == nil and cam.OWN_ACTIONS == nil and cam.rt.keyHeld == nil)
  check("A: input observer on PlayerPuppet.OnAction", type(W.observers["PlayerPuppet.OnAction"]) == "function")
  check("A: onInit line (camera on) in the UTPP log", count(W.lines, "onInit 0.1.6: camera on") == 1,
        table.concat(W.lines, " | "))
  W.player = false
  frames(W, 0.2)
  check("A: no errors without a player", count(W.lines, "update error") == 0)
  W.player = true
  frames(W, 2.0)
  check("A: TPP signal: first frame sends OFF once (initial sync, clears a stale value)", #W.tppCalls == 1 and W.tppCalls[1] == false
        and count(W.lines, "TPP signal -> OFF (fpp, initial sync) via Game.UTPPSetTppOverride") == 1, table.concat(W.lines, " | "))
  check("A: FPP matrix layout detected (rows)", count(W.lines, "FPP camera matrix layout: rows") == 1, table.concat(W.lines, " | "))
  -- calibration while V follows the mouse (moving so idle orbit stays off)
  W.speed = 2
  frames(W, 0.5, 60, function() action(W, "CameraMouseX", 10); W.yaw = W.yaw - 1.5 end)
  check("A: mouse scale calibrated from the game's body turn", cam.cal.samples >= 10 and math.abs(cam.cal.value - 0.15) < 1e-6,
        tostring(cam.cal.value))
  -- enter
  withEnv(W, function() cam.Toggle("key") end)
  check("A: toggle -> want TPP", cam.T.want == true)
  frames(W, 1.0)
  check("A: representation ON (event + anim feature) before the camera", countEv(W, "gameActivateTPPRepresentationEvent") == 1
        and W.features[1] and W.features[1][1] == "TPPRepresentation" and W.features[1][2] == true)
  check("A: utppCamera activated once and rendering", W.utpp.activations == 1 and W.active == "utpp" and cam.T.state == "tpp",
        cam.T.state .. " " .. W.active)
  check("A: TPP signal ON exactly once with the camera activation (one log line)", #W.tppCalls == 2 and W.tppCalls[2] == true
        and count(W.lines, "TPP signal -> ON (cam_on epoch=") == 1 and count(W.lines, "TPP signal ->") == 2, table.concat(W.lines, " | "))
  check("A: pose nudge skipped outside a standing stand state (2 scheduled, both logged, nothing pushed)", #W.pushed == 0
        and count(W.lines, "pose nudge (after full body) skipped:") == 2, table.concat(W.lines, " | "))
  local lp = W.utpp.lp
  local P2 = MEDIUM
  check("A: boom behind the right shoulder (player space)", lp.y < -(P2.distance - 0.2) and lp.x > 0.3 and lp.z > 1.4,
        string.format("%.2f %.2f %.2f", lp.x, lp.y, lp.z))
  check("A: FOV moves to the profile FOV", math.abs(W.utpp.fov - P2.fov) < 2.5, W.utpp.fov)
  check("A: watchdog sees utppCamera active (no reactivation)", cam.stats.reactivations == 0)
  -- follow: V turned by the mouse keeps the camera rigid behind her (local transform unchanged; 0.1.3: the mouse part
  -- passes the follow limiter at once - calibrated 0.15 deg per unit: -13.33 units = +2 deg)
  local lpBefore = CM.v3(lp.x, lp.y, lp.z)
  frames(W, 0.3, 60, function() action(W, "CameraMouseX", -2 / 0.15); W.yaw = W.yaw + 2 end)
  check("A: follow mode: local boom constant while V turns with the mouse", CM.dist(W.utpp.lp, lpBefore) < 1e-3, CM.dist(W.utpp.lp, lpBefore))
  -- free look: heading lock + own yaw from calibrated mouse
  cam.FreeLook(true)
  local wp0 = W.utppWorld()
  frames(W, 0.5, 60, function() action(W, "CameraMouseX", 10) end)
  check("A: free look locks V's heading", W.fpp.headingLocked == true)
  local wp1, wq1 = W.utppWorld()
  local yaw1 = CM.yawPitchFromForward(CM.qrot(wq1, CM.v3(0, 1, 0)))
  check("A: free look orbits by the calibrated mouse (~ -45 deg)", math.abs(CM.wrapDeg(yaw1 - W.yaw) + 45) < 3,
        CM.wrapDeg(yaw1 - W.yaw))
  cam.FreeLook(false)
  frames(W, 1.5)
  check("A: free look release: heading unlocked, camera back behind V", W.fpp.headingLocked == false
        and math.abs(cam.S.look.yawOff) < 1, cam.S.look.yawOff)
  -- collision: wall 1 m behind V
  W.wallY = W.pos.y - 1.0
  frames(W, 0.3)
  local cw = W.utppWorld()
  check("A: collision: camera stays in front of the wall", cw.y > W.wallY and W.rays > 0, string.format("%.3f vs %.3f", cw.y, W.wallY))
  W.wallY = nil
  frames(W, 1.5)
  check("A: wall removed: boom back to full length", W.utpp.lp.y < -(P2.distance - 0.2), W.utpp.lp.y)
  -- shoulder swap
  withEnv(W, function() cam.Swap() end)
  frames(W, 0.5)
  check("A: shoulder swap -> left shoulder", W.utpp.lp.x < -0.3, W.utpp.lp.x)
  -- profile next: 2026-10-23 one fixed camera profile - NextProfile is a no-op (slot 2, nothing persisted, same boom)
  local yBefore = W.utpp.lp.y
  withEnv(W, function() cam.NextProfile(1) end)
  frames(W, 1.2)
  check("A: profile next -> no-op (fixed slot 2, camera.profile unchanged, boom length unchanged)", cam.rt.profileIndex == 2
        and math.abs(W.utpp.lp.y - yBefore) < 0.05 and W.utpp.lp.y < -(P2.distance - 0.2),
        string.format("%.3f -> %.3f", yBefore, W.utpp.lp.y))
  -- quick FPP
  withEnv(W, function() cam.QuickFpp(true) end)
  frames(W, 1 / 60)
  check("A: quick FPP (hold): first person in the same frame", W.active == "fpp")
  withEnv(W, function() cam.QuickFpp(false) end)
  frames(W, 0.5)
  check("A: quick FPP released: third person again", W.active == "utpp" and cam.T.state == "tpp")
  -- weapon / vehicle cuts
  W.weapon = true
  frames(W, 1 / 60)
  check("A: weapon drawn -> cut to first person", W.active == "fpp" and cam.T.blocked == "weapon")
  W.weapon = false
  frames(W, 1.0)
  check("A: weapon away -> third person again", W.active == "utpp")
  W.psm.Vehicle = 1
  frames(W, 0.1)
  check("A: vehicle -> first person (vehicle camera takes over)", W.active ~= "utpp" and cam.T.state == "fpp")
  W.psm.Vehicle = 0
  frames(W, 1.0)
  check("A: out of the vehicle -> third person", W.active == "utpp")
  -- menu freeze
  W.menu = true
  local wr = W.utpp.writes
  frames(W, 0.3)
  check("A: menu: no camera writes", W.utpp.writes == wr)
  W.menu = false
  -- zoom
  local zy = W.utpp.lp.y
  withEnv(W, function() cam.Zoom(1) end)
  frames(W, 0.8)
  check("A: zoom out lengthens the boom", W.utpp.lp.y < zy - 0.15, string.format("%.2f -> %.2f", zy, W.utpp.lp.y))
  -- exit
  local deact = W.utpp.deactivations
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("A: toggle back: FPP camera active, utppCamera deactivated after it", W.active == "fpp" and W.utpp.deactivations > deact
        and cam.T.state == "fpp")
  check("A: TPP signal OFF on exit (camera back in the eyes)", W.tppCalls[#W.tppCalls] == false and cam.tppSig.sent == false
        and count(W.lines, "TPP signal -> OFF (fpp_on+rep_off epoch=") >= 1)
  check("A: representation OFF on exit (owned by UTPP)", countEv(W, "gameDeactivateTPPRepresentationEvent") >= 1
        and W.features[#W.features][2] == false)
  check("A: heading lock released", W.fpp.headingLocked == false)
  local st
  withEnv(W, function() st = cam.Status() end)
  check("A: Status()", string.find(st, "state=fpp", 1, true) ~= nil and string.find(st, "enabled=true", 1, true) ~= nil, st)
  check("A: no update / write errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  -- 2026-10-21: the game never unlocks in this scenario - the heading lock read-back must not write or skip anything
  check("A: no heading-lock re-assert in a session where the game never unlocks (relocks 0, skips 0)", (cam.stats.lockRelocks or 0) == 0
        and (cam.stats.lockSkips or 0) == 0, string.format("%s %s", tostring(cam.stats.lockRelocks), tostring(cam.stats.lockSkips)))
  -- 144 fps run of a full enter / exit: no errors, same end state
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0, 144)
  local ok144 = W.active == "utpp" and cam.T.state == "tpp"
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0, 144)
  check("A: enter / exit at 144 fps", ok144 and W.active == "fpp" and cam.stats.errors == 0)
  os.remove(settingsFile)
end

-- ================================================================ B) "Use Ultimate Third Person" (camera.impl) off
do
  local W = makeWorld()
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.impl", "off") end)
  check("B: switched off -> UTPP camera off", C.Effective("camera.impl") == false and cam.Enabled() == false)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("B: key ignored with a log line, no activation", cam.T.want == false and W.utpp.activations == 0
        and count(W.lines, "UTPP camera is off (setting 'Use Ultimate Third Person' off)") == 1, table.concat(W.lines, " | "))
  check("B: TPP signal while off: only the initial OFF", #W.tppCalls == 1 and W.tppCalls[1] == false)
  withEnv(W, function() C.Set("camera.impl", "on") end)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("B: 'Use Ultimate Third Person' = on enables it", W.utpp.activations == 1 and W.active == "utpp")
  withEnv(W, function() C.Set("camera.impl", "off") end)
  frames(W, 0.2)
  check("B: switched off while active -> immediate cut to first person", W.active == "fpp" and cam.T.state == "fpp")
  withEnv(W, function() C.Set("camera.impl", "auto") end)
  check("B: a stored legacy 'auto' value migrates to on", C.Get("camera.impl") == true and C.Effective("camera.impl") == true)
  os.remove(settingsFile)
end

-- ================================================================ C) component missing
do
  local W = makeWorld({ hasComp = false })
  loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("C: utppCamera missing: logged once, stays first person, no errors", count(W.lines, "utppCamera MISSING") == 1
        and count(W.lines, "TPP requested but utppCamera is missing") == 1 and cam.T.state == "fpp" and W.active == "fpp"
        and cam.stats.errors == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end

-- ================================================================ D) utpp_core missing
do
  local W = makeWorld()
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("D: utpp_core missing reported", count(W.lines, "utpp_core not found") == 1)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.0)
  check("D: console Toggle() works with defaults", W.active == "utpp" and cam.T.state == "tpp")
end

-- ================================================================ F) parallel modules: utpp_perspective / utpp_body (API 0.1.0 stubs)
do
  local W = makeWorld()
  local C = loadCore(W)
  local st = { tpp = false, style = "blend", mode = "active", toggles = 0, quick = nil }
  local persp = {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = st.style, reason = "test",
                                  mode = st.mode } end,
    Mode = function() return st.mode end,
    ToggleUser = function() st.toggles = st.toggles + 1; st.tpp = not st.tpp; return true end,
    SetQuickFpp = function(d) st.quick = d; return d end,
  }
  local bodyCalls = {}
  local body = { Enabled = function() return true end,
                 SetTppBody = function(on, why) bodyCalls[#bodyCalls + 1] = on; return true end }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n)
    if n == "utpp_perspective" then return persp end
    if n == "utpp_body" then return body end
    return baseGetMod(n)
  end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("F: modules found and logged", count(W.lines, "utpp_perspective found, utpp_body found") == 1, table.concat(W.lines, " | "))
  frames(W, 2.0)
  st.tpp = true
  frames(W, 1.2)
  check("F: follows the perspective decision (TPP) with the full body from utpp_body", W.active == "utpp" and cam.T.state == "tpp"
        and bodyCalls[1] == true and countEv(W, "gameActivateTPPRepresentationEvent") == 0, cam.T.state)
  W.weapon = true
  frames(W, 0.3)
  check("F: own block rules off (utpp_perspective decides about weapons)", W.active == "utpp")
  W.weapon = false
  withEnv(W, function() cam.Toggle() end)
  frames(W, 0.1)
  check("F: Toggle() goes through utpp_perspective; 'blend' decision = animated exit", st.toggles == 1 and cam.T.state == "leaving"
        and W.active == "utpp", cam.T.state)
  frames(W, 1.0)
  check("F: exit done, full body released through utpp_body", W.active == "fpp" and bodyCalls[#bodyCalls] == false
        and countEv(W, "gameDeactivateTPPRepresentationEvent") == 0)
  st.tpp = true
  frames(W, 1.2)
  st.tpp, st.style = false, "instant"
  frames(W, 1 / 60)
  check("F: 'instant' decision = cut in the same frame", W.active == "fpp")
  withEnv(W, function() cam.QuickFpp(true) end)
  check("F: quick FPP forwarded to utpp_perspective", st.quick == true)
  check("F: no errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ G) 0.1.1 TPP signal routes / retries and the pose nudge
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("G: pose nudge setting registered (default on)", C.settings.defs["camera.poseNudge"] ~= nil and C.Get("camera.poseNudge") == true)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle() end)
  frames(W, 0.45)
  check("G: standing still: first StandEnter 0.4 s after the full body", #W.pushed == 1 and W.pushed[1] == "StandEnter",
        #W.pushed)
  frames(W, 0.8)
  check("G: second StandEnter at 0.9 s, then no more", #W.pushed == 2
        and count(W.lines, "pose nudge (after full body): StandEnter pushed") == 2, table.concat(W.lines, " | "))
  frames(W, 1.0)
  check("G: no further nudges while staying in TPP", #W.pushed == 2)
  -- crouching: skipped
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.0)
  W.psm.LocomotionDetailed = 3
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.2)
  check("G: crouch (PSM locomotion 3): nudge skipped with a reason", #W.pushed == 2
        and count(W.lines, "skipped: PSM locomotion 3 is not stand") == 2, table.concat(W.lines, " | "))
  -- setting off: no nudge scheduled
  W.psm.LocomotionDetailed = 1
  withEnv(W, function() cam.Toggle(); C.Set("camera.poseNudge", false) end)
  frames(W, 1.0)
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.2)
  check("G: camera.poseNudge=false: nothing pushed", #W.pushed == 2 and cam.T.state == "tpp")
  local onCalls = 0
  for _, v in ipairs(W.tppCalls) do if v then onCalls = onCalls + 1 end end
  check("G: one ON per enter (3 enters) and one OFF per exit", onCalls == 3 and #W.tppCalls == 6 and W.tppCalls[6] == true,
        #W.tppCalls)
  -- session reset while in TPP (player gone) -> OFF
  W.player = false
  frames(W, 0.1)
  check("G: session reset in TPP sends OFF", W.tppCalls[#W.tppCalls] == false and count(W.lines, "TPP signal -> OFF (session reset: no player)") == 1,
        table.concat(W.lines, " | "))
  W.player = true
  frames(W, 2.5)
  -- fallback route: the script global unresolved in CET -> UTPPPerspectiveState
  W.tppReds = nil
  W.system = { calls = {} }
  function W.system:SetTppOverride(on) self.calls[#self.calls + 1] = on; return true end
  check("G: back in TPP after the reset (want kept)", cam.T.state == "tpp" and cam.tppSig.sent == true)
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.0)
  check("G: fallback route UTPPPerspectiveState:SetTppOverride", W.system.calls[1] == false and cam.tppSig.sent == false
        and cam.tppSig.route == "UTPPPerspectiveState:SetTppOverride", tostring(cam.tppSig.route))
  -- both routes missing (old utpp.reds): one log line, retries every second, success once the route is back
  W.system = nil
  withEnv(W, function() cam.Toggle() end)
  frames(W, 2.5)
  check("G: no route: FAILED logged once, old value kept, retried (~1 s)", cam.tppSig.sent == false and cam.tppSig.failures >= 2
        and cam.tppSig.failures <= 4 and count(W.lines, "TPP signal -> ON FAILED") == 1, cam.tppSig.failures)
  W.tppReds = true
  frames(W, 1.2)
  check("G: route back: ON delivered", cam.tppSig.sent == true and W.tppCalls[#W.tppCalls] == true and cam.T.state == "tpp")
  -- shutdown while in TPP -> OFF
  local n = #W.tppCalls
  withEnv(W, function() W.handlers.onShutdown() end)
  check("G: shutdown in TPP sends OFF", #W.tppCalls == n + 1 and W.tppCalls[n + 1] == false)
  local st
  withEnv(W, function() st = cam.Status() end)
  check("G: Status() reports the signal and nudges", string.find(st, "tppSignal=off via Game.UTPPSetTppOverride", 1, true) ~= nil
        and string.find(st, "nudges=2", 1, true) ~= nil, st)
  check("G: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end

-- ================================================================ H) 0.1.3 camera round 3 (earlier manual test no. 2): capsule pivot,
-- crouch, menu-close pose nudge, body-flip hold log, return from aiming (own rule)
local function yawOfCam(W)
  local _, q = W.utppWorld()
  return CM.yawPitchFromForward(CM.qrot(q, CM.v3(0, 1, 0)))
end
local function enterTiming(W, cam, maxFrames)
  local enterAt, tppAt = nil, nil
  for i = 1, maxFrames or 120 do
    frames(W, 1 / 60)
    if not enterAt and cam.T.state == "entering" then enterAt = i end
    if cam.T.state == "tpp" then tppAt = i; break end
  end
  return enterAt and tppAt and (tppAt - enterAt + 1) / 60 or nil
end
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("H: round-3 settings registered (pivot capsule, follow limit 240 deg/s, hold on flip, aim return 0.35 s)",
        C.Get("camera.pivotSource") == "capsule" and C.Get("camera.followMaxRate") == 240 and C.Get("camera.followHoldOnFlip") == true
        and C.Get("camera.aimReturn") == true and math.abs(C.Get("camera.aimReturnBlend") - 0.35) < 1e-9
        and cam.cfg.pivotSource == "capsule")
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle() end)
  frames(W, 1.5)
  local z0 = W.utppWorld().z
  W.eyeDrop = 0.5
  frames(W, 0.6)
  local z1 = W.utppWorld().z
  check("H: capsule pivot: FPP camera 0.5 m lower (FPP set) -> camera height unchanged", math.abs(z1 - z0) < 1e-3,
        string.format("%.3f -> %.3f", z0, z1))
  withEnv(W, function() C.Set("camera.pivotSource", "fpp") end)
  frames(W, 0.6)
  local z2 = W.utppWorld().z
  check("H: negative control camera.pivotSource=fpp: camera follows the FPP drop (the reported crouch look), logged",
        z0 - z2 > 0.4 and count(W.lines, "pivot source -> fpp") == 1, string.format("%.3f -> %.3f", z0, z2))
  withEnv(W, function() C.Set("camera.pivotSource", "capsule") end)
  W.eyeDrop = 0
  frames(W, 0.6)
  check("H: capsule again: height restored, logged", math.abs(W.utppWorld().z - z0) < 0.01
        and count(W.lines, "pivot source -> capsule (stand 1.55 m, crouch 1.05 m, blend 0.25 s)") == 1, W.utppWorld().z)
  W.psm.Locomotion = 1
  -- 2026-10-21 (P1 C): a clean in-place crouch takes the body's Stand_to_Crouch timing (crouchDelay 0.2 s + crouchBlend 0.4 s = 0.6 s,
  -- was 0.25 s from the PSM edge); the assertion (the full 0.50 m) is unchanged, only the wait grew from 0.5 s to 0.8 s
  frames(W, 0.8)
  local z3 = W.utppWorld().z
  check("H: crouch (PSM Locomotion 1): camera 0.50 m lower, logged once", math.abs((z0 - z3) - 0.5) < 0.02
        and count(W.lines, "pivot stance -> crouch (PSM Locomotion 1, LocomotionDetailed 1)") == 1, string.format("%.3f", z0 - z3))
  W.psm.Locomotion = 0
  frames(W, 0.5)
  check("H: stand up: back to the standing height, logged", math.abs(W.utppWorld().z - z0) < 0.01
        and count(W.lines, "pivot stance -> stand") == 1)
  -- 2026-10-13: the walk toggle (G) writes LocomotionDetailed 3 with Locomotion stand; a workspot writes Locomotion 9
  -- with LocomotionDetailed 1 (stand) - neither is a crouch (the camera must not drop)
  W.psm.LocomotionDetailed = 3
  frames(W, 0.5)
  check("H: walk toggle (Locomotion 0, LocomotionDetailed 3): still standing height, no crouch log",
        math.abs(W.utppWorld().z - z0) < 0.01 and count(W.lines, "pivot stance -> crouch") == 1, W.utppWorld().z)
  W.psm.LocomotionDetailed = 1
  W.psm.Locomotion = 9
  frames(W, 0.5)
  check("H: workspot (Locomotion 9, LocomotionDetailed 1): still standing height", math.abs(W.utppWorld().z - z0) < 0.01
        and count(W.lines, "pivot stance -> crouch") == 1, W.utppWorld().z)
  W.psm.LocomotionDetailed = 5
  frames(W, 0.7)
  -- 2026-10-21: the seated slide is a stance of its own (logged once). The free camera keeps the crouch pivot (the low slide
  -- pivot 0.62 m is scaled in by the aim weight: block H2; the 0.62 m pivot put the free camera 0.35 m over the floor)
  check("H: free slide (Locomotion 9, LocomotionDetailed 5): crouch pivot (0.50 m under the standing height), logged once as the slide stance",
        math.abs((z0 - W.utppWorld().z) - 0.5) < 0.02
        and count(W.lines, "pivot stance -> slide (PSM Locomotion 9, LocomotionDetailed 5): pivot 0.62 m aimed / 1.05 m free over 0.50 s") == 1,
        string.format("%.3f", z0 - W.utppWorld().z))
  W.psm.Locomotion, W.psm.LocomotionDetailed = 0, 1
  frames(W, 0.7)
  check("H: slide ended: back to the standing height, logged as the stand stance", math.abs(W.utppWorld().z - z0) < 0.01
        and count(W.lines, "pivot stance -> stand") == 2, string.format("%.3f", W.utppWorld().z - z0))
  -- menu closed: pose nudge 0.1 s and 0.5 s later (standing still in TPP)
  local p0 = #W.pushed
  W.menu = true
  frames(W, 0.5)
  W.menu = false
  frames(W, 0.15)
  local p1 = #W.pushed
  frames(W, 0.4)
  check("H: menu closed: StandEnter at +0.1 s and +0.5 s (logged)", p1 == p0 + 1 and #W.pushed == p0 + 2
        and count(W.lines, "menu closed after 0.50 s: pose nudge scheduled +0.10 s / +0.50 s") == 1
        and count(W.lines, "pose nudge (after menu close): StandEnter pushed") == 2, string.format("%d %d %d", p0, p1, #W.pushed))
  W.speed = 2
  W.menu = true
  frames(W, 0.2)
  W.menu = false
  frames(W, 0.6)
  check("H: menu closed while moving: nudges skipped with the reason (same guards)", #W.pushed == p0 + 2
        and count(W.lines, "pose nudge (after menu close) skipped: moving") == 2)
  W.speed = 0
  -- body flip without the mouse while running (idle orbit had turned the camera to V's face)
  frames(W, 0.6)
  frames(W, 0.5, 60, function() action(W, "CameraMouseX", -60) end) -- idle orbit: 0.1 deg per unit -> +180 deg
  local off = CM.wrapDeg(yawOfCam(W) - W.yaw)
  W.speed = 3
  local y0, maxStep, prevY = W.yaw, 0, yawOfCam(W)
  frames(W, 1.6, 60, function(i)
    if i <= 12 then W.yaw = y0 + 180 * i / 12 end
  end)
  check("H: idle orbit had the camera in front of V before the run", math.abs(math.abs(off) - 180) < 5, off)
  check("H: body flip (180 deg in 0.2 s, no mouse) -> world yaw hold logged once", count(W.lines, "follow: body turned") == 1
        and cam.stats.flips == 1, table.concat(W.lines, " | "))
  check("H: camera ends behind V's back after the flip", math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw)) < 3,
        CM.wrapDeg(yawOfCam(W) - W.yaw))
  W.speed = 0
  -- return from aiming with the own rules (weapon drawn = first person): fast entry, behind the back
  frames(W, 0.6)
  frames(W, 0.25, 60, function() action(W, "CameraMouseX", -60) end) -- idle orbit offset ~90 deg before the aim
  local offAim = CM.wrapDeg(yawOfCam(W) - W.yaw)
  W.weapon = true
  frames(W, 0.3)
  check("H: weapon drawn -> first person (own rule)", W.active == "fpp" and cam.T.state == "fpp")
  W.weapon = false
  local dur = enterTiming(W, cam)
  check("H: aim return (own rule): entry over 0.35 s, logged", dur and math.abs(dur - 0.35) <= 2 / 60 + 1e-9
        and count(W.lines, "aim return (own rule: weapon put away): third person over 0.35 s") == 1, tostring(dur))
  check("H: aim return lands behind the back (orbit offset before the aim was ~90 deg)", math.abs(offAim) > 60
        and math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw)) < 1, string.format("before %.1f, after %.2f", offAim,
        CM.wrapDeg(yawOfCam(W) - W.yaw)))
  -- switch off: the old entry time and the orbit offset kept
  withEnv(W, function() C.Set("camera.aimReturn", false) end)
  frames(W, 0.25, 60, function() action(W, "CameraMouseX", -60) end)
  local offAim2 = CM.wrapDeg(yawOfCam(W) - W.yaw)
  W.weapon = true
  frames(W, 0.3)
  W.weapon = false
  local dur2 = enterTiming(W, cam)
  check("H: camera.aimReturn=false: entry over enterTime (0.45 s), orbit offset kept (0.1.2 behaviour)", dur2
        and math.abs(dur2 - 0.45) <= 2 / 60 + 1e-9 and math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw) - offAim2) < 2
        and count(W.lines, "aim return (") == 1, string.format("%s %.1f %.1f", tostring(dur2), offAim2, CM.wrapDeg(yawOfCam(W) - W.yaw)))
  local st
  withEnv(W, function() st = cam.Status() end)
  check("H: Status() reports pivot / follow / aim return / menu nudges", string.find(st, "pivot=capsule followMaxRate=240 "
        .. "holdOnFlip=true flips=1 aimReturns=1 menuNudges=2", 1, true) ~= nil, st)
  check("H: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end

-- ================================================================ H2) 2026-10-21 slide pivot scaled by the aim weight
-- (recording 03:39: aimed slides v43-44 / v52 aimW 1.00, free slide v33-35 aimW 0.00): aimed = camera.slideHeight 0.62 m, free =
-- the crouch height 1.05 m, the pivot follows the aim tween; the stance log names both heights
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local persp = {
    Desired = function() return { tpp = true, perspective = "TPP", style = "blend", reason = "user", mode = "active" } end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" },
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  W.records["Items.Preset_Lexington_Default"] = "Wea_Handgun"
  W.statValues["ent_Items.Preset_Lexington_Default"] = 48
  local pistol = {}
  function pistol:GetItemID() return { tdb = "Items.Preset_Lexington_Default" } end
  function pistol:GetEntityID() return { hash = "ent_Items.Preset_Lexington_Default" } end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 3.0)
  local function pivotH() return cam.S.pivot.height end
  check("H2: standing, no aim: pivot 1.55 m", cam.T.state == "tpp" and math.abs(pivotH() - 1.55) < 1e-6, pivotH())
  W.weapon, W.weaponItem, W.psm.UpperBody = true, pistol, 6
  frames(W, 0.6)
  check("H2: aim on while standing: pivot unchanged 1.55 m", cam.aim.on and math.abs(pivotH() - 1.55) < 1e-6, pivotH())
  W.psm.Locomotion, W.psm.LocomotionDetailed = 9, 5
  frames(W, 0.9)
  check("H2: aimed slide (Locomotion 9, LocomotionDetailed 5): slide pivot 0.62 m, one slide stance log with both heights",
        cam.aim.on and math.abs(pivotH() - 0.62) < 1e-3
        and count(W.lines, "pivot stance -> slide (PSM Locomotion 9, LocomotionDetailed 5): pivot 0.62 m aimed / 1.05 m free over 0.50 s") == 1,
        pivotH())
  W.psm.UpperBody = 0
  frames(W, 0.9)
  check("H2: aim released during the slide: pivot back to the crouch height 1.05 m (free camera)",
        not cam.aim.on and math.abs(pivotH() - 1.05) < 1e-3, pivotH())
  W.psm.UpperBody = 6
  frames(W, 0.9)
  check("H2: aim again during the slide: pivot 0.62 m, no second stance log",
        cam.aim.on and math.abs(pivotH() - 0.62) < 1e-3 and count(W.lines, "pivot stance -> slide") == 1, pivotH())
  -- negative control (the fix off: camera.slideHeight at the crouch height = the old behaviour): the aimed slide is a crouch
  cam.cfg.slideHeight = cam.cfg.crouchHeight
  frames(W, 0.9)
  check("H2: negative control slideHeight = crouchHeight: the aimed slide keeps the crouch pivot 1.05 m",
        cam.aim.on and math.abs(pivotH() - 1.05) < 1e-3, pivotH())
  cam.cfg.slideHeight = 0.62
  frames(W, 0.9)
  check("H2: slideHeight restored: the aimed slide pivot 0.62 m again", math.abs(pivotH() - 0.62) < 1e-3, pivotH())
  W.psm.Locomotion, W.psm.LocomotionDetailed = 0, 1
  frames(W, 1.0)
  check("H2: slide ended while aiming: pivot back to the standing height 1.55 m, stance stand",
        cam.aim.on and math.abs(pivotH() - 1.55) < 1e-3 and count(W.lines, "pivot stance -> stand") == 1, pivotH())
  check("H2: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
end

-- ================================================================ P) 2026-10-21 fix batch 2: stance of the pivot (P7 F1, P1 A / C)
-- Recording 12:15: the game writes Locomotion 1 + LocomotionDetailed 30 for a CrouchSprint, the DLL plays it as Stand + Sprint, the camera held
-- the crouch pivot (1.05 m) through whole runs (8.7 of 11 s wrong) and pumped 37 times in the Crouch 3 <-> CrouchSprint 30 flicker. The camera
-- stance goes through the port of the DLL stance filter; a clean in-place crouch / stand-up passes at once and takes the clip's timing.
do
  local function freshWorld()
    local W = makeWorld()
    W.psm.LocomotionDetailed = 1
    local C = loadCore(W)
    local cam = loadCamera(W)
    withEnv(W, function() W.handlers.onInit() end)
    frames(W, 2.0)
    withEnv(W, function() cam.Toggle() end)
    frames(W, 1.5)
    return W, cam, C
  end
  -- seconds until the camera has covered aFrac of aDrop metres (world z), stepping frame by frame; nil = not within aMax
  local function timeTo(W, z0, aDrop, aFrac, aMax)
    local zStart = z0
    for i = 1, math.floor(aMax * 60) do
      frames(W, 1 / 60)
      if math.abs(W.utppWorld().z - zStart) >= aDrop * aFrac then return i / 60 end
    end
    return nil
  end
  -- (a) CrouchSprint is a stand
  do
    local W, cam = freshWorld()
    local z0 = W.utppWorld().z
    W.speed = 8
    W.psm.Locomotion, W.psm.LocomotionDetailed = 1, 30
    frames(W, 0.8)
    check("P: CrouchSprint (Locomotion 1, LocomotionDetailed 30) at 8 m/s is a stand: camera at the standing height, no crouch stance log",
          math.abs(W.utppWorld().z - z0) < 0.02 and count(W.lines, "pivot stance -> crouch") == 0, string.format("%.3f", z0 - W.utppWorld().z))
    -- mutation: the pre-fix rule (Locomotion 1 = crouch whatever the detail) puts the pivot 0.5 m down
    local src = io.open(UTPP_CAMERA_PATH, "rb"):read("*a")
    local from = "(loco == 1 and det ~= 30)"
    local a, b = src:find(from, 1, true)
    check("P (mutation setup): the stance rule of init.lua is found", a ~= nil)
    if a then
      local mutPath = TMP .. "utpp_camera_mut_rule.lua"
      local f = io.open(mutPath, "wb")
      f:write(src:sub(1, a - 1) .. "loco == 1" .. src:sub(b + 1))
      f:close()
      local Wm = makeWorld()
      Wm.psm.LocomotionDetailed = 1
      loadCore(Wm)
      local camM
      withEnv(Wm, function() camM = dofile(mutPath) end)
      Wm.cameraMod = camM
      withEnv(Wm, function() Wm.handlers.onInit() end)
      frames(Wm, 2.0)
      withEnv(Wm, function() camM.Toggle() end)
      frames(Wm, 1.5)
      local zm0 = Wm.utppWorld().z
      Wm.speed = 8
      Wm.psm.Locomotion, Wm.psm.LocomotionDetailed = 1, 30
      frames(Wm, 1.2)
      check("P (mutation): with 'Locomotion 1 = crouch' the CrouchSprint pivot sits 0.5 m low (the test would fail)", zm0 - Wm.utppWorld().z > 0.4,
            string.format("%.3f", zm0 - Wm.utppWorld().z))
      os.remove(mutPath)
    end
  end
  -- (b) the flicker Crouch 3 <-> CrouchSprint 30 does not pump the pivot
  do
    local W, cam = freshWorld()
    local z0 = W.utppWorld().z
    W.speed = 3
    W.psm.Locomotion = 1
    for i = 1, 20 do
      W.psm.LocomotionDetailed = (i % 2 == 1) and 3 or 30
      frames(W, 0.1)
    end
    check("P: Crouch 3 <-> CrouchSprint 30 every 0.1 s for 2 s at 3 m/s: at most one stance change, camera height steady",
          count(W.lines, "pivot stance") <= 1 and math.abs(W.utppWorld().z - z0) < 0.6, count(W.lines, "pivot stance"))
    local Wo, camO = freshWorld()
    camO.cfg.stanceFilter = 0
    Wo.speed = 3
    Wo.psm.Locomotion = 1
    for i = 1, 20 do
      Wo.psm.LocomotionDetailed = (i % 2 == 1) and 3 or 30
      frames(Wo, 0.1)
    end
    check("P (mutation): without the filter every flip is logged (>= 10 stance lines)", count(Wo.lines, "pivot stance") >= 10, count(Wo.lines, "pivot stance"))
  end
  -- (c) Stand -> Crouch 3 for 0.1 s -> CrouchSprint at the run speed (feed t 278.786, 280.435): never a crouch
  do
    local W, cam = freshWorld()
    local z0 = W.utppWorld().z
    W.speed = 3.85
    W.psm.Locomotion, W.psm.LocomotionDetailed = 1, 3
    frames(W, 0.1)
    W.psm.LocomotionDetailed = 30
    local zmin = z0
    for _ = 1, 60 do
      frames(W, 1 / 60)
      zmin = math.min(zmin, W.utppWorld().z)
    end
    check("P: Stand -> Crouch 3 for 0.1 s -> CrouchSprint at 3.85 m/s: no crouch stance, camera never drops more than 3 cm",
          count(W.lines, "pivot stance -> crouch") == 0 and z0 - zmin < 0.03, string.format("%.3f", z0 - zmin))
  end
  -- (d) a clean in-place crouch / stand-up: the stance passes on its own frame, the pivot takes the clip's timing
  do
    local W, cam = freshWorld()
    local z0 = W.utppWorld().z
    W.speed = 0
    W.psm.Locomotion, W.psm.LocomotionDetailed = 1, 3
    frames(W, 1 / 60)
    check("P: a crouch in place is logged on the frame of the PSM change", count(W.lines, "pivot stance -> crouch (PSM Locomotion 1, LocomotionDetailed 3)") == 1)
    local t50 = timeTo(W, z0, 0.5, 0.5, 1.5)
    frames(W, 0.6)
    check("P: the in-place crouch moves the camera 50 % at 0.34 .. 0.50 s after the PSM change (the hips of Stand_to_Crouch: 0.43 s), then 0.50 m in all",
          t50 and t50 >= 0.34 and t50 <= 0.50 and math.abs((z0 - W.utppWorld().z) - 0.5) < 0.02, tostring(t50))
    local zc = W.utppWorld().z
    W.psm.Locomotion, W.psm.LocomotionDetailed = 0, 1
    frames(W, 1 / 60)
    check("P: the stand-up in place is logged on the frame of the PSM change", count(W.lines, "pivot stance -> stand") == 1)
    local u50 = timeTo(W, zc, 0.5, 0.5, 1.5)
    check("P: the in-place stand-up moves the camera 50 % at 0.16 .. 0.32 s (Crouch_to_Stand: 0.24 s)", u50 and u50 >= 0.16 and u50 <= 0.32, tostring(u50))
    -- walking: the stance passes at once as well (below the run speed), the pivot keeps the 0.25 s blend
    local Ww = freshWorld()
    local zw0 = Ww.utppWorld().z
    Ww.speed = 2
    Ww.psm.Locomotion, Ww.psm.LocomotionDetailed = 1, 3
    frames(Ww, 1 / 60)
    check("P: a crouch while walking at 2 m/s is logged on the frame of the PSM change", count(Ww.lines, "pivot stance -> crouch") == 1)
    local w50 = timeTo(Ww, zw0, 0.5, 0.5, 1.5)
    check("P: a crouch while walking keeps the 0.25 s blend (50 % at 0.07 .. 0.2 s)", w50 and w50 >= 0.07 and w50 <= 0.2, tostring(w50))
    -- a crouch at the run speed keeps the filter time (0.15 s) before the stance changes
    local Wr = freshWorld()
    Wr.speed = 3.85
    Wr.psm.Locomotion, Wr.psm.LocomotionDetailed = 1, 3
    frames(Wr, 0.1)
    local early = count(Wr.lines, "pivot stance -> crouch")
    frames(Wr, 0.1)
    check("P: a crouch entered at 3.85 m/s waits for the filter time (not logged at 0.1 s, logged by 0.2 s)", early == 0 and count(Wr.lines, "pivot stance -> crouch") == 1,
          string.format("%d %d", early, count(Wr.lines, "pivot stance -> crouch")))
    -- mutation: no clean-edge bypass -> the in-place crouch is delayed by the filter time
    local Wm = freshWorld()
    Wm.cameraMod.cfg.stanceCleanEdge = false
    Wm.speed = 0
    Wm.psm.Locomotion, Wm.psm.LocomotionDetailed = 1, 3
    frames(Wm, 0.05)
    check("P (mutation): without the clean-edge bypass the in-place crouch is not yet logged at 0.05 s", count(Wm.lines, "pivot stance -> crouch") == 0)
  end
  -- (e) P7 F2: a jump out of a slide - the pivot over the capsule must not fall below the crouch pivot while the body stands up and rises
  do
    local srcFull = io.open(UTPP_CAMERA_PATH, "rb"):read("*a")
    local function slideJump(aPath)
      local W = makeWorld()
      W.psm.LocomotionDetailed = 1
      loadCore(W)
      local cam
      withEnv(W, function() cam = dofile(aPath) end)
      W.cameraMod = cam
      withEnv(W, function() W.handlers.onInit() end)
      frames(W, 2.0)
      withEnv(W, function() cam.Toggle() end)
      frames(W, 1.5)
      local base = W.utppWorld().z - W.pos.z -- boom + pivot over the capsule while standing (pivot 1.55 m)
      W.speed = 8
      W.psm.Locomotion, W.psm.LocomotionDetailed = 10, 5
      frames(W, 1.2)
      W.psm.Locomotion, W.psm.LocomotionDetailed = 0, 18 -- take-off: Jump, 5.5 m/s up under 16 m/s^2
      local z0, vz, minRel = W.pos.z, 5.5, 9
      frames(W, 0.6, 60, function(i)
        vz = (i == 1) and 5.5 or (vz - 16 / 60)
        W.pos.z = math.max(z0, W.pos.z + vz / 60)
        if i / 60 >= 0.02 and i / 60 <= 0.4 then
          minRel = math.min(minRel, (W.utppWorld().z - W.pos.z) - (base - 1.55))
        end
      end)
      return minRel, W
    end
    local minRel = slideJump(UTPP_CAMERA_PATH)
    -- the world z of the camera is pivot + boom offset: the boom offset differs a little from "pivot over the capsule", so the bound is the measured
    -- one of the pre-fix camera (0.75 m) plus the fix (>= 0.95 m at the camera)
    local from = "air = det == 6 or (det >= 14 and det <= 22),"
    local a, b = srcFull:find(from, 1, true)
    check("P (mutation setup): the air flag of init.lua is found", a ~= nil)
    local minOld
    if a then
      local mutPath = TMP .. "utpp_camera_mut_air.lua"
      local f = io.open(mutPath, "wb")
      f:write(srcFull:sub(1, a - 1) .. "air = false," .. srcFull:sub(b + 1))
      f:close()
      minOld = slideJump(mutPath)
      os.remove(mutPath)
    end
    check(string.format("P: a jump out of a slide keeps the pivot over the capsule >= 0.95 m for 0.4 s (%.2f m)", minRel), minRel >= 0.95, string.format("%.3f", minRel))
    check(string.format("P (mutation): without the air flag the same jump dips the pivot below 0.90 m (%.2f m; the pre-fix recording 0.73 .. 0.85)", minOld or 0), minOld ~= nil and minOld < 0.90, tostring(minOld))
  end
  -- (f) the way back to third person while already crouching (V crouched in first person): no clip timing for a crouch that began outside
  do
    local srcFull = io.open(UTPP_CAMERA_PATH, "rb"):read("*a")
    local function backCrouched(aPath)
      local W = makeWorld()
      W.psm.LocomotionDetailed = 1
      loadCore(W)
      local cam
      withEnv(W, function() cam = dofile(aPath) end)
      W.cameraMod = cam
      withEnv(W, function() W.handlers.onInit() end)
      frames(W, 2.0)
      withEnv(W, function() cam.Toggle() end)
      frames(W, 1.5)
      local z0 = W.utppWorld().z
      withEnv(W, function() cam.Toggle() end) -- first person
      frames(W, 1.0)
      W.psm.Locomotion, W.psm.LocomotionDetailed = 1, 3 -- crouches in first person, stands still
      frames(W, 1.0)
      withEnv(W, function() cam.Toggle() end) -- back to third person
      local drop = {}
      for i = 1, 90 do
        frames(W, 1 / 60)
        drop[i] = z0 - W.utppWorld().z
      end
      return W, z0, drop
    end
    local W, z0, drop = backCrouched(UTPP_CAMERA_PATH)
    local line
    for _, l in ipairs(W.lines) do if l:find("pivot stance -> crouch", 1, true) then line = l end end
    check("P: back in third person while already crouching: the stance is logged with the 0.25 s blend, not the clip timing (0.40 s)",
          line ~= nil and line:find("over 0.25 s", 1, true) ~= nil, tostring(line))
    local t90 = nil
    for i = 1, #drop do if drop[i] >= 0.45 then t90 = i / 60 break end end
    check(string.format("P: back in third person while crouching: the camera is at the crouch height within 0.7 s of the first frame (%s s)", tostring(t90 and string.format("%.2f", t90) or nil)),
          t90 ~= nil and t90 <= 0.7, tostring(t90))
    -- mutation: without the gap reset the first frame back reads a fresh Stand -> Crouch edge and takes the clip timing
    local from = "if G.sf == nil or G.sfN ~= (G.updN or 0) - 1 then"
    local a, b = srcFull:find(from, 1, true)
    check("P (mutation setup): the gap reset of init.lua is found", a ~= nil)
    if a then
      local mutPath = TMP .. "utpp_camera_mut_gap.lua"
      local f = io.open(mutPath, "wb")
      f:write(srcFull:sub(1, a - 1) .. "if G.sf == nil then" .. srcFull:sub(b + 1))
      f:close()
      local Wm = backCrouched(mutPath)
      local lineM
      for _, l in ipairs(Wm.lines) do if l:find("pivot stance -> crouch", 1, true) then lineM = l end end
      check("P (mutation): without the gap reset the way back takes the 0.40 s clip timing", lineM ~= nil and lineM:find("over 0.40 s", 1, true) ~= nil, tostring(lineM))
      os.remove(mutPath)
    end
  end
  os.remove(settingsFile)
end

-- ================================================================ I) 0.1.3 return from aiming through utpp_perspective (rule "weapon")
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local st = { tpp = true, reason = "user", rule = nil, detail = nil, style = "blend" }
  local persp = {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = st.style, reason = st.reason,
                                  rule = st.rule, detail = st.detail, mode = "active" } end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 3.0)
  check("I: in third person by the decision", cam.T.state == "tpp" and W.active == "utpp")
  frames(W, 0.3, 60, function() action(W, "CameraMouseX", -60) end) -- orbit offset before aiming
  local before = CM.wrapDeg(yawOfCam(W) - W.yaw)
  st.tpp, st.reason, st.rule, st.detail, st.style = false, "weapon", "weapon", "aiming", "instant"
  frames(W, 0.5)
  check("I: aiming -> first person (cut)", W.active == "fpp" and cam.T.state == "fpp")
  st.tpp, st.reason, st.rule, st.detail, st.style = true, "user", nil, nil, "blend"
  local dur = enterTiming(W, cam)
  check("I: aim return after the aiming rule: entry over 0.35 s, behind the back, logged", dur
        and math.abs(dur - 0.35) <= 2 / 60 + 1e-9 and math.abs(before) > 60 and math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw)) < 1
        and count(W.lines, "aim return (utpp_perspective weapon: aiming): third person over 0.35 s") == 1,
        string.format("%s before %.1f after %.2f", tostring(dur), before, CM.wrapDeg(yawOfCam(W) - W.yaw)))
  -- a scene (not aiming): normal entry, no aim return
  st.tpp, st.reason, st.rule, st.detail, st.style = false, "scene", "scene", "SceneTier3", "instant"
  frames(W, 0.5)
  st.tpp, st.reason, st.rule, st.detail, st.style = true, "user", nil, nil, "blend"
  local dur2 = enterTiming(W, cam)
  check("I: return after another rule (scene): normal entry (0.45 s), no aim return", dur2
        and math.abs(dur2 - 0.45) <= 2 / 60 + 1e-9 and count(W.lines, "aim return (") == 1, tostring(dur2))
  check("I: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ J) 0.1.4 over-the-shoulder aim (perspective.aiming = tpp)
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local st = { tpp = true }
  local persp = {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = "blend", reason = "user",
                                  mode = "active" } end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" }, -- read when utpp_core has no perspective.aiming (utpp_perspective registers it in game)
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  local function item(rec, ty, fov)
    W.records[rec] = ty
    W.statValues["ent_" .. rec] = fov
    local it = {}
    function it:GetItemID() return { tdb = rec } end
    function it:GetEntityID() return { hash = "ent_" .. rec } end
    return it
  end
  local pistol = item("Items.Preset_Lexington_Default", "Wea_Handgun", 48)
  local katana = item("Items.Preset_Katana_Default", "Wea_Katana", 0)
  local rifle = item("Items.Preset_Ajax_Default", "Wea_AssaultRifle", 0) -- AimFOV 0: outside 20-90 -> camera.aim.fov
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("J: camera.aim.boomPitchScale registered (default 0.35 = camera cfg)", C.Get("camera.aim.boomPitchScale") == 0.35
        and cam.cfg.aimBoomPitchScale == 0.35, tostring(C.Get("camera.aim.boomPitchScale")))
  -- 2026-10-13 round 2: the weapon aim framing limit and the unarmed zoom's own orbit are settings (camera cfg defaults);
  -- round 3: the weapon framing limit is opt-in (default 0)
  check("J: camera.aim.boomFrame 0, camera.zoomUnarmed.boomPitchScale 0.25 / boomFrame 0 registered (= camera cfg)",
        C.Get("camera.aim.boomFrame") == 0 and cam.cfg.aimBoomFrame == 0
        and C.Get("camera.zoomUnarmed.boomPitchScale") == 0.25 and cam.cfg.zoomBoomPitchScale == 0.25
        and C.Get("camera.zoomUnarmed.boomFrame") == 0 and cam.cfg.zoomBoomFrame == 0,
        string.format("%s %s %s", tostring(C.Get("camera.aim.boomFrame")), tostring(C.Get("camera.zoomUnarmed.boomPitchScale")),
        tostring(C.Get("camera.zoomUnarmed.boomFrame"))))
  withEnv(W, function() C.Set("camera.aim.boomFrame", 0.5); C.Set("camera.zoomUnarmed.boomPitchScale", 0.4); C.Set("camera.zoomUnarmed.boomFrame", 0.3) end)
  check("J: the three settings apply to the camera cfg", cam.cfg.aimBoomFrame == 0.5 and cam.cfg.zoomBoomPitchScale == 0.4
        and cam.cfg.zoomBoomFrame == 0.3)
  withEnv(W, function() C.Set("camera.aim.boomFrame", 0); C.Set("camera.zoomUnarmed.boomPitchScale", 0.25); C.Set("camera.zoomUnarmed.boomFrame", 0) end)
  withEnv(W, function() C.Set("camera.aim.zoomFov", true); C.Set("camera.aim.ownYaw", false); C.Set("camera.aim.distance", 0.8); C.Set("camera.aim.boomPitchScale", 1) end) -- legacy aim checks (2026-10-12 defaults differ; 23:40 full-pitch boom)
  check("J: aim settings registered (enabled, 0.8 m, 0.45 m, FOV 42, auto-swap)", C.Get("camera.aim.enabled") == true
        and math.abs(C.Get("camera.aim.distance") - 0.8) < 1e-9 and math.abs(C.Get("camera.aim.side") - 0.45) < 1e-9
        and C.Get("camera.aim.fov") == 42 and C.Get("camera.aim.autoSwap") == true and cam.cfg.aimEnabled == true)
  frames(W, 3.0)
  check("J: third person by the decision", cam.T.state == "tpp" and W.active == "utpp")
  local sOff
  withEnv(W, function() sOff = cam.Status() end)
  check("J: Status() shows aim=off aimT=", string.find(sOff, "aim=off aimT=", 1, true) ~= nil, sOff)
  -- weapon drawn, not aiming: no aim state, no weapon stat reads
  W.weapon, W.weaponItem = true, pistol
  frames(W, 0.3)
  check("J: weapon drawn without aiming (UpperBody 0): no aim, no stat read", not cam.aim.on and W.statCalls == 0
        and count(W.lines, "cam aim enter") == 0)
  local P2 = MEDIUM
  local function camFwd() local _, q = W.utppWorld(); return CM.qrot(q, CM.v3(0, 1, 0)) end
  -- aim: UpperBody 6 with the pistol
  W.psm.UpperBody = 6
  frames(W, 0.4)
  check("J: aim enter logged once with the weapon record, its AimFOV stat and the shoulder",
        count(W.lines, "[utpp_camera] cam aim enter weapon=Items.Preset_Lexington_Default fov=48 side=R") == 1 and cam.aim.on,
        table.concat(W.lines, " | "))
  check("J: AimFOV stat form logged once (StatsSystem GetStatValue with gamedataStatType)",
        count(W.lines, "aim FOV: weapon stat read via StatsSystem GetStatValue(EntityID, gamedataStatType.AimFOV)") == 1)
  local lp = W.utpp.lp
  local expectLocal = CM.add(CM.v3(0, 0, 1.55), CM.boomOffset(0, -5, 0.45, 0.8, 0.05))
  check("J: aim pose: 0.8 m behind the capsule pivot, 0.45 m right (player space)", CM.dist(lp, expectLocal) < 1e-3,
        string.format("%.3f %.3f %.3f", lp.x, lp.y, lp.z))
  check("J: aim FOV = weapon AimFOV 48 (no lag)", math.abs(W.utpp.fov - 48) < 1e-6, W.utpp.fov)
  check("J: camera forward = FPP aim direction (body yaw, FPP pitch)", CM.dist(camFwd(), CM.forward(W.yaw, W.fppPitch)) < 1e-4)
  check("J: V follows the camera (heading not locked)", W.fpp.headingLocked == false)
  local sOn
  withEnv(W, function() sOn = cam.Status() end)
  check("J: Status() shows aim=on aimT=", string.find(sOn, "aim=on aimT=", 1, true) ~= nil
        and string.find(sOn, "aimFov=48", 1, true) ~= nil, sOn)
  -- free look and idle orbit disabled while aiming
  cam.FreeLook(true)
  W.speed = 0
  frames(W, 0.6, 60, function() action(W, "CameraMouseX", 10) end)
  check("J: free look held + standing still while aiming: no orbit, heading not locked",
        math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw)) < 1e-3 and W.fpp.headingLocked == false and not cam.S.look.free)
  cam.FreeLook(false)
  W.speed = 2
  frames(W, 0.1)
  -- zoom keys masked while aiming
  withEnv(W, function() cam.Zoom(1) end)
  frames(W, 0.2)
  check("J: zoom out while aiming: aim distance kept", CM.dist(W.utpp.lp, expectLocal) < 1e-3)
  withEnv(W, function() cam.Zoom(-1) end)
  -- auto-swap: wall 0.35 m on the right shoulder side
  W.wallX = W.pos.x + 0.35
  frames(W, 0.2)
  check("J: wall on the shoulder side < 0.3 s: no swap yet", count(W.lines, "cam aim swap") == 0)
  frames(W, 0.6)
  check("J: wall for 0.3 s -> one auto-swap to the left, logged with the wall distance",
        count(W.lines, "[utpp_camera] cam aim swap -> L (wall 0.35 m)") == 1 and count(W.lines, "cam aim swap") == 1
        and cam.shoulder.target == -1 and W.utpp.lp.x < -0.4, table.concat(W.lines, " | "))
  W.wallX = nil
  -- melee weapon (block) while UpperBody stays 6 -> exit
  W.weaponItem = katana
  frames(W, 0.1)
  check("J: melee weapon in the hand -> aim exit (no ranged weapon) logged once",
        count(W.lines, "[utpp_camera] cam aim exit (no ranged weapon) after ") == 1 and not cam.aim.on, table.concat(W.lines, " | "))
  -- exit blend: back to the profile over 0.20 s (FOV passes between)
  local fovs = {}
  frames(W, 0.4, 60, function() fovs[#fovs + 1] = W.utpp.fov end)
  local midFov = fovs[2]
  check("J: exit blend 0.20 s: FOV between during the blend, profile FOV after it", midFov > 48 + 0.5 and midFov < P2.fov - 0.5
        and math.abs(W.utpp.fov - P2.fov) < 1e-6, string.format("mid %.2f end %.2f", midFov, W.utpp.fov))
  check("J: after the exit the camera stays behind V (no aim return needed, none logged)",
        math.abs(CM.wrapDeg(yawOfCam(W) - W.yaw)) < 1 and count(W.lines, "aim return (") == 0)
  -- ranged rifle with AimFOV 0 -> camera.aim.fov 42
  W.weaponItem = rifle
  frames(W, 0.4)
  check("J: AimFOV outside 20-90 -> camera.aim.fov 42 (logged), enter on the current shoulder",
        count(W.lines, "cam aim enter weapon=Items.Preset_Ajax_Default fov=42 side=L") == 1
        and count(W.lines, "AimFOV=0.00 outside 20-90 - camera.aim.fov 42 used") == 1 and math.abs(W.utpp.fov - 42) < 1e-6,
        table.concat(W.lines, " | "))
  -- vehicle hold wins over aim
  local okH
  withEnv(W, function() okH = cam.HoldForVehicle(true, "test") end)
  frames(W, 0.1)
  check("J: vehicle hold -> aim exit (vehicle hold)", okH == true and not cam.aim.on
        and count(W.lines, "cam aim exit (vehicle hold) after ") == 1)
  withEnv(W, function() cam.HoldForVehicle(false, "test") end)
  frames(W, 0.3)
  check("J: hold released while still aiming -> aim again", cam.aim.on
        and count(W.lines, "cam aim enter weapon=Items.Preset_Ajax_Default") == 2)
  -- menu freezes the aim state
  W.psm.UpperBody = 0
  W.menu = true
  frames(W, 0.3)
  check("J: menu: aim state frozen (no exit while paused)", cam.aim.on and count(W.lines, "cam aim exit (UpperBody=0)") == 0)
  W.menu = false
  frames(W, 0.1)
  check("J: aim released (UpperBody 0) -> exit logged with the time in aim", not cam.aim.on
        and count(W.lines, "[utpp_camera] cam aim exit (UpperBody=0) after ") == 1)
  -- perspective.aiming = fpp (utpp_core value wins over utpp_perspective's cfg) / camera.aim.enabled off: no aim
  C.Register({ key = "perspective.aiming", section = "perspective", type = "choice", default = "fpp",
    choices = { { "fpp", "First person" }, { "tpp", "Third person" } } })
  W.psm.UpperBody = 6
  frames(W, 0.3)
  check("J: perspective.aiming = fpp in utpp_core -> no aim", not cam.aim.on and count(W.lines, "cam aim enter") == 3)
  withEnv(W, function() C.Set("perspective.aiming", "tpp") end)
  frames(W, 0.3)
  check("J: perspective.aiming = tpp -> aim", cam.aim.on and count(W.lines, "cam aim enter") == 4)
  withEnv(W, function() C.Set("camera.aim.enabled", false) end)
  frames(W, 0.1)
  check("J: camera.aim.enabled off -> exit (camera.aim.enabled off)", not cam.aim.on
        and count(W.lines, "cam aim exit (camera.aim.enabled off)") == 1)
  withEnv(W, function() C.Set("camera.aim.enabled", true) end)
  -- 144 fps enter / exit, then the perspective leaves third person while aiming
  W.psm.UpperBody = 0
  frames(W, 0.5, 144)
  W.psm.UpperBody = 6
  frames(W, 0.5, 144)
  local on144 = cam.aim.on and math.abs(W.utpp.fov - 42) < 1e-6
  W.psm.UpperBody = 0
  frames(W, 0.5, 144)
  check("J: enter / exit at 144 fps", on144 and not cam.aim.on and math.abs(W.utpp.fov - P2.fov) < 1e-6)
  W.psm.UpperBody = 6
  frames(W, 0.3)
  st.tpp = false
  frames(W, 1.0)
  check("J: perspective to first person while aiming -> aim exit (not third person), camera back in the eyes",
        not cam.aim.on and count(W.lines, "cam aim exit (not third person)") == 1 and W.active == "fpp")
  check("J: no update / write errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  -- 2026-10-21: the game never unlocks in this scenario - the heading lock read-back must not write or skip anything
  check("J: no heading-lock re-assert in a session where the game never unlocks (relocks 0, skips 0)", (cam.stats.lockRelocks or 0) == 0
        and (cam.stats.lockSkips or 0) == 0, string.format("%s %s", tostring(cam.stats.lockRelocks), tostring(cam.stats.lockSkips)))
  os.remove(settingsFile)
end

-- ================================================================ K) 0.1.4 aim occlusion (level A)
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local persp = {
    Desired = function() return { tpp = true, perspective = "TPP", style = "blend", reason = "user", mode = "active" } end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" },
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  local REC = "Items.Preset_Lexington_Default"
  W.records[REC] = "Wea_Handgun"
  W.statValues["ent_" .. REC] = 48
  W.muzzleCalls = 0
  local muzzleOff = CM.v3(0.25, 0.55, 1.40) -- player space: right hand forward, below the eyes
  local function muzzleWorld() return CM.add(W.pos, CM.qrot(CM.qFromYawPitch(W.yaw, 0), muzzleOff)) end
  local pistol = {}
  function pistol:GetItemID() return { tdb = REC } end
  function pistol:GetEntityID() return { hash = "ent_" .. REC } end
  function pistol:GetMuzzleSlotWorldTransform()
    W.muzzleCalls = W.muzzleCalls + 1
    local p = muzzleWorld()
    return { position = { x = p.x, y = p.y, z = p.z, w = 1 } }
  end
  local bare = {} -- same weapon without a muzzle reader (CET form missing)
  function bare:GetItemID() return { tdb = REC } end
  function bare:GetEntityID() return { hash = "ent_" .. REC } end
  -- cover right in front of V: x 100.00-100.35, 1.45-1.75 m ahead; the muzzle ray (x ~100.25) crosses it, the camera
  -- ray (x = 100.45, right shoulder) passes beside it
  local COVER = { { min = CM.v3(100.0, 52.0, 9.0), max = CM.v3(100.35, 52.3, 13.0) } }
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  check("K: occlusion settings registered (camera.aim.occlusion on, camera.aim.range 60 m)",
        C.Get("camera.aim.occlusion") == true and C.Get("camera.aim.range") == 60 and cam.cfg.aimOcclusion == true
        and cam.cfg.aimRange == 60)
  frames(W, 3.0)
  check("K: not aiming: AimBlocked() false, no samples", cam.AimBlocked() == false and cam.occ.samples == 0)
  W.weapon, W.weaponItem, W.psm.UpperBody = true, pistol, 6
  frames(W, 0.5)
  local i0 = cam.AimBlockInfo()
  local cw, cq = W.utppWorld()
  local cf = CM.qrot(cq, CM.v3(0, 1, 0))
  local near = function(a, b, e) return a ~= nil and b ~= nil and math.abs(a - b) <= (e or 1e-6) end
  check("K: aiming in the open: not blocked, P = far end 60 m on the written camera ray, muzzle read",
        cam.aim.on and cam.AimBlocked() == false and i0.reason == "clear" and near(i0.t, 60, 1e-6) and i0.p ~= nil
        and CM.dist(i0.p, CM.add(cw, CM.scale(cf, 60))) < 1e-3 and W.muzzleCalls > 0 and count(W.lines, "aim blocked") == 0,
        string.format("%s t=%s", tostring(i0.reason), tostring(i0.t)))
  check("K: t0 = muzzle plane along the camera ray", near(i0.t0, CM.dot(CM.sub(muzzleWorld(), cw), cf), 1e-3), i0.t0)
  local st0
  withEnv(W, function() st0 = cam.Status() end)
  check("K: Status() aimBlocked=0 aimBlockD=-", string.find(st0, "aimBlocked=0 aimBlockD=-", 1, true) ~= nil, st0)
  local s0 = cam.occ.samples
  frames(W, 1.0)
  check("K: throttle: ~30 samples per second at 60 fps", cam.occ.samples - s0 >= 29 and cam.occ.samples - s0 <= 31,
        cam.occ.samples - s0)
  -- cover in front of the muzzle, camera ray clear
  W.boxes = COVER
  frames(W, 0.05)
  check("K: cover seen < 0.1 s: not blocked yet (hysteresis)", cam.AimBlocked() == false)
  frames(W, 0.3)
  local ib = cam.AimBlockInfo()
  local m = muzzleWorld()
  local expHit = CM.dist(m, ib.p) * (52.0 - m.y) / (ib.p.y - m.y)
  check("K: cover between the muzzle and P -> blocked, one log line with the muzzle->obstacle distance",
        cam.AimBlocked() == true and ib.blocked and near(ib.dMuzzleHit, expHit, 1e-3) and near(ib.dMuzzleP, CM.dist(m, ib.p), 1e-3)
        and count(W.lines, string.format("[utpp_camera] aim blocked d=%.2f (muzzle->P hit at %.2f of %.2f)", ib.dMuzzleHit,
          ib.dMuzzleHit, ib.dMuzzleP)) == 1 and count(W.lines, "aim blocked") == 1,
        string.format("hit %s exp %.3f | %s", tostring(ib.dMuzzleHit), expHit, table.concat(W.lines, " | ")))
  check("K: the camera ray itself is clear (P still the far end)", near(ib.t, 60, 1e-6))
  local viaMod, st1
  withEnv(W, function() viaMod = GetMod("utpp_camera").AimBlocked(); st1 = cam.Status() end)
  check("K: GetMod(\"utpp_camera\").AimBlocked() = true", viaMod == true)
  check("K: Status() aimBlocked=1 aimBlockD=<m>", string.find(st1, string.format("aimBlocked=1 aimBlockD=%.2f", ib.dMuzzleHit),
        1, true) ~= nil, st1)
  -- cover removed: clear after the hysteresis, logged with the blocked time
  W.boxes = nil
  frames(W, 0.05)
  check("K: cover gone < 0.1 s: still blocked (hysteresis)", cam.AimBlocked() == true)
  frames(W, 0.3)
  check("K: cover gone -> clear, one log line \"aim clear after <s> s\"", cam.AimBlocked() == false
        and count(W.lines, "[utpp_camera] aim clear after ") == 1, table.concat(W.lines, " | "))
  -- aim point on a wall: the muzzle ray ends on the same wall (tolerance) -> not blocked
  W.wallY = 58
  frames(W, 0.4)
  local iw = cam.AimBlockInfo()
  check("K: P on a wall ~8.8 m ahead (y = 58): not blocked (hit at P within 0.15 m)", cam.AimBlocked() == false
        and iw.t < 10 and (iw.reason == "hit at P" or iw.reason == "clear") and count(W.lines, "aim blocked") == 1,
        string.format("%s t=%.2f", tostring(iw.reason), iw.t or -1))
  W.wallY = nil
  -- blocked again, then the aim is released: the flag holds while the exit tween weight > 0.5, then drops at once
  W.boxes = COVER
  frames(W, 0.3)
  local blocked2 = cam.AimBlocked() and count(W.lines, "aim blocked") == 2
  W.psm.UpperBody = 0
  frames(W, 0.05)
  local heldInTween = cam.AimBlocked() and not cam.aim.on and cam.aim.w > 0.5
  frames(W, 0.2)
  check("K: aim released while blocked: flag kept while the aim weight > 0.5, then cleared (aim off) at once",
        blocked2 and heldInTween and cam.AimBlocked() == false and count(W.lines, "aim clear after ") == 2
        and count(W.lines, " s (aim off)") == 1, table.concat(W.lines, " | "))
  -- camera.aim.occlusion off: no rays while aiming behind the cover
  withEnv(W, function() C.Set("camera.aim.occlusion", false) end)
  W.psm.UpperBody = 6
  local s1 = cam.occ.samples
  frames(W, 0.5)
  check("K: camera.aim.occlusion off: aiming behind cover, no samples, not blocked", cam.aim.on and cam.AimBlocked() == false
        and cam.occ.samples == s1 and cam.cfg.aimOcclusion == false)
  withEnv(W, function() C.Set("camera.aim.occlusion", true) end)
  frames(W, 0.3)
  check("K: camera.aim.occlusion on again -> blocked", cam.AimBlocked() == true and count(W.lines, "aim blocked") == 3)
  -- weapon object without a readable muzzle: logged once, the flag clears, no errors
  W.weaponItem = bare
  frames(W, 0.5)
  check("K: muzzle unreadable -> logged once, not blocked, no update errors", cam.AimBlocked() == false
        and count(W.lines, "aim occlusion: weapon muzzle position unreadable (GetMuzzleSlotWorldTransform: ") == 1
        and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  -- 144 fps: blocked edge 0.1 s .. 0.1 s + 2 samples + 1 frame after the cover appears
  W.weaponItem, W.boxes = pistol, nil
  frames(W, 0.5, 144)
  local tEdge = nil
  W.boxes = COVER
  frames(W, 0.5, 144, function(i) if tEdge == nil and cam.AimBlocked() then tEdge = (i - 1) / 144 end end)
  check("K: 144 fps: blocked edge 0.10-0.18 s after the cover appears", cam.AimBlocked() == true and tEdge ~= nil
        and tEdge >= 0.1 - 1e-6 and tEdge <= 0.1 + 2 * 5 / 144 + 1 / 144 + 1e-6, tostring(tEdge))
  check("K: no update / write errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end

-- ================================================================ L) 0.1.5 game object handles (ExitGame crash, cycles 28-29)
-- No mocked game object (player, camera components, weapon object) may be reachable from the module state after the aim exit,
-- a session reset (no player), or onShutdown (both paths); the weapon object is never cached across frames.
do
  local REC = "Items.Preset_Lexington_Default"
  local function setup()
    local W = makeWorld()
    W.psm.LocomotionDetailed = 1
    local C = loadCore(W)
    local persp = {
      Desired = function() return { tpp = true, perspective = "TPP", style = "blend", reason = "user", mode = "active" } end,
      Mode = function() return "active" end,
      ToggleUser = function() return true end, SetQuickFpp = function() return true end,
      cfg = { aiming = "tpp" },
    }
    local baseGetMod = W.env.GetMod
    W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
    W.records[REC] = "Wea_Handgun"
    W.statValues["ent_" .. REC] = 48
    W.muzzleCalls = 0
    local pistol = {}
    function pistol:GetItemID() return { tdb = REC } end
    function pistol:GetEntityID() return { hash = "ent_" .. REC } end
    function pistol:GetMuzzleSlotWorldTransform()
      W.muzzleCalls = W.muzzleCalls + 1
      return { position = { x = W.pos.x + 0.25, y = W.pos.y + 0.55, z = W.pos.z + 1.4, w = 1 } }
    end
    local cam = loadCamera(W)
    withEnv(W, function() W.handlers.onInit() end)
    frames(W, 3.0)
    W.weapon, W.weaponItem, W.psm.UpperBody = true, pistol, 6
    frames(W, 1.0)
    local mocks = { [pistol] = true, [W.playerObj] = true, [W.utpp] = true, [W.fpp] = true }
    for _, c in ipairs(W.bodyComps) do mocks[c] = true end
    return W, cam, pistol, mocks
  end
  local function refs(cam, W, mocks)
    return findMockRefs({ cam = cam, onUpdate = W.handlers.onUpdate, onShutdown = W.handlers.onShutdown, onInit = W.handlers.onInit },
      mocks, "@" .. UTPP_CAMERA_PATH)
  end
  local function only(list, pattern) -- the reference paths that do not match the allowed pattern
    local bad = {}
    for _, r in ipairs(list) do if not string.find(r, pattern) then bad[#bad + 1] = r end end
    return bad
  end

  -- aiming: muzzle read per sample from the slot; the weapon object is not cached (no G.aimObj, weapon not reachable)
  do
    local W, cam, pistol, mocks = setup()
    check("L: aiming with a pistol (setup): aim on, muzzle read", cam.aim.on and W.muzzleCalls > 0 and cam.AimBlocked() == false)
    check("L: no G.aimObj field, the weapon object is not reachable from the module state while aiming",
      cam.G.aimObj == nil and not next(findMockRefs({ cam = cam }, { [pistol] = true }, "@" .. UTPP_CAMERA_PATH)),
      table.concat(findMockRefs({ cam = cam }, { [pistol] = true }, "@" .. UTPP_CAMERA_PATH), ", "))
    check("L: while playing only the cached camera components / player are held (G.comp / G.fpp / G.player)",
      #only(refs(cam, W, mocks), "%^G%.%a+$") == 0, table.concat(refs(cam, W, mocks), ", "))
    W.psm.UpperBody = 0
    local mc = W.muzzleCalls
    frames(W, 1.5)
    check("L: aim exit: weapon object unreachable, no muzzle read after the exit tween", not cam.aim.on and cam.aim.w == 0
      and not next(findMockRefs({ cam = cam }, { [pistol] = true }, "@" .. UTPP_CAMERA_PATH)) and W.muzzleCalls - mc <= 12,
      tostring(W.muzzleCalls - mc))
    -- weapon swapped while not aiming: nothing of the old weapon is kept for the next aim
    local mc2 = W.muzzleCalls
    frames(W, 0.5)
    check("L: not aiming: no muzzle reads at all", W.muzzleCalls == mc2)
    -- player gone (load screen / exit): handles dropped at once, session reset
    W.psm.UpperBody = 6
    frames(W, 0.5)
    W.player = false
    frames(W, 0.3)
    check("L: no player: G.comp / G.fpp / G.player dropped, nothing reachable", cam.G.comp == nil and cam.G.fpp == nil
      and cam.G.player == nil and #refs(cam, W, mocks) == 0 and count(W.lines, "update error") == 0, table.concat(refs(cam, W, mocks), ", "))
    -- exit with the player gone: guarded path, no game call, handles dropped, the frame loop stops
    withEnv(W, function() W.handlers.onShutdown() end)
    local calls = W.playerCalls
    check("L: shutdown (player gone): logged, no handle left, G.shuttingDown", count(W.lines, "shutdown: player gone - game calls skipped") == 1
      and cam.G.comp == nil and cam.G.fpp == nil and cam.G.player == nil and #refs(cam, W, mocks) == 0 and cam.G.shuttingDown == true)
    W.player = true
    frames(W, 0.5)
    check("L: after onShutdown the frame loop stops (no game call, nothing cached again)", W.playerCalls == calls
      and cam.G.comp == nil and #refs(cam, W, mocks) == 0, tostring(W.playerCalls - calls))
    check("L: no update errors (player gone / shutdown)", cam.stats.errors == 0 and count(W.lines, "update error") == 0,
      table.concat(W.lines, " | "))
  end

  -- exit with the player still alive (normal shutdown path): state restored, then every handle dropped
  do
    local W, cam, pistol, mocks = setup()
    local before = #refs(cam, W, mocks)
    check("L: alive shutdown (setup): camera components cached while playing", before > 0 and cam.G.comp ~= nil)
    withEnv(W, function() W.handlers.onShutdown() end)
    check("L: shutdown (player alive): G.comp / G.fpp / G.player dropped, no mock game object reachable",
      cam.G.comp == nil and cam.G.fpp == nil and cam.G.player == nil and cam.G.shuttingDown == true and #refs(cam, W, mocks) == 0,
      table.concat(refs(cam, W, mocks), ", "))
    local calls = W.playerCalls
    frames(W, 0.5)
    check("L: after an alive shutdown no further game calls", W.playerCalls == calls and cam.G.comp == nil and #refs(cam, W, mocks) == 0)
    withEnv(W, function() cam.DropHandles() end)
    check("L: DropHandles() is idempotent", cam.G.comp == nil and cam.G.fpp == nil and cam.G.player == nil)
  end
end

-- ================================================================ M) 0.1.6 unarmed zoom (RMB with empty hands) + the real utpp_perspective
-- RMB unarmed (PSM UpperBody 6, empty WeaponRight) -> zoom state over the shoulder; the game's ZoomIn / ZoomOut actions
-- (value edge without ListenerAction.GetType, type edge with it) step 0 -> 1 -> 2; past the last step -> first person
-- handoff through utpp_perspective.RequestFppZoom (rule "zoom", instant); RMB release -> request cleared, third person
-- again (fast return), profile pose back; hotkey fallback; weapon / dodge / setting off; session reset during the handoff.
check("M: runner provides UTPP_PERSPECTIVE_PATH (test_utpp_camera_run.py)", type(UTPP_PERSPECTIVE_PATH) == "string")
if UTPP_PERSPECTIVE_PATH then
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local persp
  withEnv(W, function()
    registerForEvent, registerInput = nil, nil -- pure module: polled below like its 0.1 s onUpdate poll
    persp = dofile(UTPP_PERSPECTIVE_PATH)
  end)
  persp.sink = function(line) W.lines[#W.lines + 1] = line end
  local okA = persp.AttachCore(C)
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.aim.zoomFov", true); C.Set("camera.aim.ownYaw", false); C.Set("camera.aim.distance", 0.8); C.Set("camera.aim.boomPitchScale", 1) end) -- legacy aim checks (2026-10-12 defaults differ; 23:40 full-pitch boom)
  withEnv(W, function() C.Set("camera.zoomUnarmed.boomPitchScale", 1) end) -- 2026-10-13: the zoom's own orbit scale, legacy full-pitch boom too
  check("M: utpp_perspective attached, found by the camera, zoom settings registered (defaults 4 steps, no handoff)",
        okA == true and cam.persp == persp
        and C.Get("camera.zoomUnarmed.enabled") == true and C.Get("camera.zoomUnarmed.steps") == 4
        and math.abs(C.Get("camera.zoomUnarmed.distance") - 0.9) < 1e-9 and C.Get("camera.zoomUnarmed.fov") == 45
        and C.Get("camera.zoomUnarmed.lastFov") == 28 and C.Get("camera.zoomUnarmed.fppHandoff") == false)
  -- the handoff scenario below runs with 3 steps and the first person handoff on (settings, not the defaults)
  C.Set("camera.zoomUnarmed.steps", 3)
  C.Set("camera.zoomUnarmed.fppHandoff", true)
  local pclock = 0
  local function pobs()
    return { player = W.player == true, aiming = (W.psm.UpperBody or 0) == 6, weaponDrawn = W.weapon == true,
             weaponType = W.weapon and "Wea_Handgun" or "", loco = W.psm.LocomotionDetailed }
  end
  local function run(secs, fps)
    fps = fps or 60
    frames(W, secs, fps, function()
      pclock = pclock + 1 / fps
      persp.Update(pclock, pobs())
    end)
  end
  local function act(name, value, ty)
    withEnv(W, function() W.observers["PlayerPuppet.OnAction"](nil, { name = name, value = value, type = ty }) end)
  end
  local function notch(name, ty) -- one wheel notch: press then release, a frame between
    act(name, 1, ty and "BUTTON_PRESSED" or nil)
    run(1 / 60)
    act(name, 0, ty and "BUTTON_RELEASED" or nil)
    run(1 / 60)
  end
  local P2 = MEDIUM
  local function expectAt(d) return CM.add(CM.v3(0, 0, 1.55), CM.boomOffset(0, -5, 0.45, d, 0.05)) end
  local function camFwd() local _, q = W.utppWorld(); return CM.qrot(q, CM.v3(0, 1, 0)) end
  run(3.0)
  check("M: third person by utpp_perspective (user TPP)", cam.T.state == "tpp" and W.active == "utpp"
        and persp.Desired().perspective == "TPP", cam.T.state .. " " .. W.active)
  -- RMB with empty hands: enter after the 0.1 s delay, pose over the shoulder
  W.psm.UpperBody = 6
  run(0.05)
  check("M: UpperBody 6 for 0.05 s: not yet (enter delay 0.1 s)", not cam.zoomU.on)
  run(0.35)
  check("M: zoom enter logged once (k=0/3, 0.90 m, FOV 45, right shoulder)",
        count(W.lines, "[utpp_camera] cam zoomUnarmed enter k=0/3 d=0.90 fov=45 side=R") == 1 and cam.zoomU.on
        and not cam.aim.on, table.concat(W.lines, " | "))
  check("M: step 0 pose: 0.90 m behind the capsule pivot, 0.45 m right, FOV 45", CM.dist(W.utpp.lp, expectAt(0.9)) < 1e-3
        and math.abs(W.utpp.fov - 45) < 1e-6, string.format("%.3f %.3f %.3f fov %.2f", W.utpp.lp.x, W.utpp.lp.y, W.utpp.lp.z, W.utpp.fov))
  check("M: body behind the camera: camera forward = FPP aim direction, V not heading-locked",
        CM.dist(camFwd(), CM.forward(W.yaw, W.fppPitch)) < 1e-4 and W.fpp.headingLocked == false)
  check("M: perspective stays TPP while zooming in third person", persp.Desired().perspective == "TPP")
  -- wheel without ListenerAction.GetType: value edge
  notch("ZoomIn")
  run(0.25)
  check("M: ZoomIn (value edge) -> step 1/3: 0.75 m, FOV 35, first sighting of the action logged",
        count(W.lines, "cam zoomUnarmed step 1/3 d=0.75 fov=35 (in wheel ZoomIn)") == 1 and cam.zoomU.k == 1
        and CM.dist(W.utpp.lp, expectAt(0.75)) < 1e-3 and math.abs(W.utpp.fov - math.sqrt(45 * 28)) < 1e-6
        and count(W.lines, "zoom input: game action ZoomIn seen through PlayerPuppet.OnAction (press from value edge") == 1,
        table.concat(W.lines, " | "))
  -- with the action type (BUTTON_PRESSED / BUTTON_RELEASED, numeric 0 / 1)
  W.env.ListenerAction.GetType = function(a) return a.type end
  notch("ZoomIn", true)
  run(0.25)
  check("M: ZoomIn (BUTTON_PRESSED) -> step 2/3: 0.60 m, FOV 28", cam.zoomU.k == 2 and count(W.lines, "cam zoomUnarmed step 2/3 d=0.60 fov=28") == 1
        and CM.dist(W.utpp.lp, expectAt(0.6)) < 1e-3 and math.abs(W.utpp.fov - 28) < 1e-6, table.concat(W.lines, " | "))
  act("ZoomOut", 1, 0); run(1 / 60); act("ZoomOut", 0, 1); run(0.25)
  check("M: ZoomOut (numeric type 0 / 1) -> step 1/3 again", cam.zoomU.k == 1
        and count(W.lines, "cam zoomUnarmed step 1/3 d=0.75 fov=35 (out wheel ZoomOut)") == 1
        and count(W.lines, "zoom input: game action ZoomOut seen") == 1)
  notch("ZoomIn", true)
  run(0.2)
  check("M: no first person before the last step is passed", cam.zoomU.k == 2 and not cam.zoomU.fpp and W.active == "utpp")
  -- past the last step: first person handoff, the game's zoom shows
  notch("ZoomIn", true)
  check("M: handoff logged (step 3/3 via utpp_perspective.RequestFppZoom), request logged by utpp_perspective",
        count(W.lines, "[utpp_camera] cam zoomUnarmed fpp handoff (step 3/3, wheel ZoomIn): first person, the game's zoom "
          .. "takes over via utpp_perspective.RequestFppZoom") == 1
        and count(W.lines, "[utpp_perspective] unarmed zoom FPP request ON (utpp_camera: unarmed zoom past step 2/3)") == 1,
        table.concat(W.lines, " | "))
  check("M: utpp_perspective -> FPP <- unarmed zoom (camera handoff), instant", persp.Desired().perspective == "FPP"
        and persp.Desired().rule == "zoom" and count(W.lines, "FPP <- unarmed zoom (camera handoff) instant [active]") == 1,
        table.concat(W.lines, " | "))
  check("M: camera cut to first person within two frames (the FPP camera renders)", W.active == "fpp"
        and (cam.T.state == "fpp" or cam.T.state == "settle"), cam.T.state .. " " .. W.active)
  run(0.3)
  local stepLines = count(W.lines, "cam zoomUnarmed step ")
  notch("ZoomIn", true)
  notch("ZoomOut", true)
  run(0.3)
  check("M: wheel during the handoff belongs to the game (no camera step, still first person)",
        count(W.lines, "cam zoomUnarmed step ") == stepLines and cam.zoomU.k == 3 and cam.zoomU.fpp and W.active == "fpp")
  local sZ
  withEnv(W, function() sZ = cam.Status() end)
  check("M: Status() shows zoomUnarmed=on k=3/3 fpp=true", string.find(sZ, "zoomUnarmed=on k=3/3 fpp=true", 1, true) ~= nil, sZ)
  check("M: utpp_perspective Status() shows zoomFpp=on", string.find(persp.Status(), "zoomFpp=on(1)", 1, true) ~= nil, persp.Status())
  -- RMB released: request cleared, third person again with the fast return, profile pose back
  W.psm.UpperBody = 0
  run(1 / 60)
  check("M: RMB release -> exit logged (first person, handoff cleared), utpp_perspective back to TPP at once",
        count(W.lines, "[utpp_camera] cam zoomUnarmed exit (UpperBody=0) after ") == 1
        and count(W.lines, "at step 3/3 (first person) - handoff cleared via utpp_perspective.RequestFppZoom") == 1
        and count(W.lines, "unarmed zoom FPP request OFF") == 1 and persp.Desired().perspective == "TPP"
        and not persp.ZoomFppRequested(), table.concat(W.lines, " | "))
  run(1.0)
  check("M: back in third person with the fast return behind the back (aim return after rule zoom)",
        cam.T.state == "tpp" and W.active == "utpp" and count(W.lines, "aim return (utpp_perspective zoom: camera handoff)") == 1,
        table.concat(W.lines, " | "))
  check("M: previous camera: profile pose and FOV back, profile zoom untouched", W.utpp.lp.y < -(P2.distance - 0.2)
        and math.abs(W.utpp.fov - P2.fov) < 2.5 and cam.rt.zoom == 0 and cam.rt.profileIndex == 2 and not cam.zoomU.on,
        string.format("y %.2f fov %.2f", W.utpp.lp.y, W.utpp.fov))
  -- console Zoom(-1 / 1) steps the unarmed zoom while the state is on, normal zoom otherwise
  W.psm.UpperBody = 6
  run(0.4)
  withEnv(W, function() cam.Zoom(-1) end)
  run(0.2)
  check("M: Zoom(-1) while zooming -> one step in (profile zoom unchanged)", cam.zoomU.k == 1 and cam.rt.zoom == 0
        and count(W.lines, "cam zoomUnarmed step 1/3 d=0.75 fov=35 (in console zoom in)") == 1, table.concat(W.lines, " | "))
  withEnv(W, function() cam.Zoom(1) end)
  run(0.2)
  check("M: Zoom(1) -> back to step 0", cam.zoomU.k == 0 and cam.rt.zoom == 0)
  W.psm.UpperBody = 0
  run(0.5)
  withEnv(W, function() cam.Zoom(-1) end)
  check("M: Zoom(-1) without the zoom state -> normal profile zoom", math.abs(cam.rt.zoom + 0.25) < 1e-9)
  withEnv(W, function() cam.Zoom(1) end)
  -- weapon in hand: no unarmed zoom (the weapon aim / perspective policy decide)
  local enters = count(W.lines, "cam zoomUnarmed enter")
  W.weapon = true
  W.psm.UpperBody = 6
  run(0.5)
  check("M: RMB with a weapon in WeaponRight -> no unarmed zoom", count(W.lines, "cam zoomUnarmed enter") == enters
        and not cam.zoomU.on)
  W.weapon, W.psm.UpperBody = false, 0
  run(3.0)
  check("M: weapon put away -> third person again", cam.T.state == "tpp" and W.active == "utpp", cam.T.state)
  -- dodge: UpperBody Aim during PSM Locomotion Dodge does not enter; after the dodge it does
  W.psm.Locomotion, W.psm.UpperBody = 6, 6
  run(0.3)
  check("M: UpperBody 6 during a dodge (Locomotion 6) -> no entry", not cam.zoomU.on
        and count(W.lines, "cam zoomUnarmed enter") == enters)
  W.psm.Locomotion = 0
  run(0.3)
  check("M: dodge over, RMB still held -> entry", cam.zoomU.on and count(W.lines, "cam zoomUnarmed enter") == enters + 1)
  W.psm.UpperBody = 0
  run(0.3)
  -- setting off
  withEnv(W, function() C.Set("camera.zoomUnarmed.enabled", false) end)
  W.psm.UpperBody = 6
  run(0.4)
  check("M: camera.zoomUnarmed.enabled off -> no entry", not cam.zoomU.on and count(W.lines, "cam zoomUnarmed enter") == enters + 1)
  withEnv(W, function() C.Set("camera.zoomUnarmed.enabled", true) end)
  run(0.3)
  check("M: setting on again while RMB is held -> entry", cam.zoomU.on)
  -- session reset (load / death) during the handoff: request cleared
  notch("ZoomIn", true); notch("ZoomIn", true); notch("ZoomIn", true)
  run(0.1)
  check("M: second handoff (3 notches)", cam.zoomU.fpp and persp.ZoomFppRequested() and W.active == "fpp")
  W.player = false
  run(1.0)
  check("M: player gone during the handoff -> zoom exit (session reset), request cleared",
        count(W.lines, "cam zoomUnarmed exit (session reset: no player)") == 1 and not persp.ZoomFppRequested()
        and not cam.zoomU.on and not cam.zuIn.requested, table.concat(W.lines, " | "))
  W.player, W.psm.UpperBody = true, 0
  run(3.0)
  check("M: after the reload: third person, no request, no update errors", cam.T.state == "tpp" and W.active == "utpp"
        and not persp.ZoomFppRequested() and cam.stats.errors == 0 and count(W.lines, "update error") == 0,
        table.concat(W.lines, " | "))
  os.remove(settingsFile)
end

-- ================================================================ N) 0.1.6 unarmed zoom without utpp_perspective (own rules)
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  C.Set("camera.zoomUnarmed.steps", 3) -- handoff scenario: 3 steps + first person handoff (defaults: 4, off)
  C.Set("camera.zoomUnarmed.fppHandoff", true)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("N: own toggle -> third person", cam.T.state == "tpp" and W.active == "utpp")
  W.psm.UpperBody = 6
  frames(W, 0.4)
  for _ = 1, 3 do withEnv(W, function() cam.Zoom(-1) end); frames(W, 0.1) end
  check("N: handoff through the own rule (no utpp_perspective): cut to first person",
        count(W.lines, "cam zoomUnarmed fpp handoff (step 3/3, console zoom in): first person, the game's zoom takes over via "
          .. "own rule 'unarmed zoom' (no utpp_perspective)") == 1 and W.active == "fpp" and cam.T.blocked == "unarmed zoom",
        table.concat(W.lines, " | "))
  W.psm.UpperBody = 0
  frames(W, 1.0)
  check("N: RMB release -> own rule cleared, fast return to third person", W.active == "utpp" and cam.T.state == "tpp"
        and count(W.lines, "aim return (own rule: unarmed zoom released)") == 1, table.concat(W.lines, " | "))
  check("N: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ T) 2026-10-12 round 2: camera trace, Locomotion flicker hold,
-- prediction of the entity turn of the coming game tick (free camera while utpp_bodyturn turns V)
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local freeMove = true
  W.env.Game.UTPPLocomotionActive = function() return freeMove end
  local bt = { exec = { why = "applied", nativeOn = true } }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_bodyturn" then return bt end return baseGetMod(n) end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  C.Set("camera.idleOrbit", false) -- standing V: the free mode below comes from Locomotion only
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("T: own toggle -> third person, Locomotion active -> free (decoupled) camera", cam.T.state == "tpp" and W.active == "utpp"
        and cam.S.look.free == true)
  local n0 = count(W.lines, "camTrace t=")
  frames(W, 1.0)
  local n1 = count(W.lines, "camTrace t=")
  check("T: camera.trace on (stored by the fixture; registered default is off): ~10 camTrace lines per second in third person",
        C.settings.defs["camera.trace"].default == false and n1 - n0 >= 9 and n1 - n0 <= 11, n1 - n0)
  local last
  for i = #W.lines, 1, -1 do if string.find(W.lines[i], "camTrace t=", 1, true) then last = W.lines[i] break end end
  local function has(s) return last ~= nil and string.find(last, s, 1, true) ~= nil end
  check("T: camTrace fields (mode, Locomotion, body turn, aim weight, distance, pivot, occlusion, mouse, FPP yaw, camErr, pred)",
        has("mode=free") and has("freeMove=1 dec=1") and has("bt=1(applied)") and has("aimW=0.00") and has("dist=")
        and has("pivotZ=") and has("blocked=0") and has("dyaw=") and has("fpp=") and has("camErr=") and has("pred=")
        and has("unit="), last)
  -- Locomotion flicker (< decoupleHold 0.35 s) keeps the free camera; longer -> follow
  freeMove = false
  frames(W, 0.2)
  local freeShort = cam.S.look.free
  freeMove = true
  frames(W, 0.2)
  freeMove = false
  frames(W, 0.6)
  local followLong = cam.S.look.free == false
  freeMove = true
  frames(W, 0.2)
  check("T: Locomotion off 0.2 s keeps the free camera, 0.6 s -> follow, back on -> free", freeShort == true and followLong
        and cam.S.look.free == true and count(W.lines, "follow: UTPP Locomotion inactive") == 2)
  -- the game turns the entity 6 deg per frame in its tick, after the CET frame (perFrame = the tick of the last frame)
  local function turnRun(predict)
    C.Set("camera.predictBodyTurn", predict)
    frames(W, 0.2)
    local mx, n = 0, 0
    frames(W, 0.5, 60, function(i)
      W.yaw = CM.wrapDeg(W.yaw + 6)
      local f = W.env.Game.GetCameraSystem():GetActiveCameraForward()
      local ry = CM.yawPitchFromForward(f)
      if i > 3 and cam.vh.last then
        mx = math.max(mx, math.abs(CM.wrapDeg(ry - cam.vh.last.yaw)))
        n = n + 1
      end
    end)
    return mx, n
  end
  local errOff = turnRun(false)
  local errOn, nOn = turnRun(true)
  check("T: free camera, V turning 6 deg per tick: rendered yaw = intended with the prediction (without: 6 deg off)",
        nOn > 20 and errOn < 0.05 and errOff > 5.5, string.format("with %.3f without %.3f", errOn, errOff))
  bt.exec.why = "deadband"
  local errIdle = turnRun(true)
  check("T: no prediction when utpp_bodyturn did not turn V (other turns are not repeated)", errIdle > 5.5, errIdle)
  -- trace off -> no lines; first person -> no lines
  C.Set("camera.trace", false)
  local n2 = count(W.lines, "camTrace t=")
  frames(W, 0.5)
  check("T: camera.trace off -> no camTrace lines", count(W.lines, "camTrace t=") == n2)
  C.Set("camera.trace", true)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.5)
  local n3 = count(W.lines, "camTrace t=")
  frames(W, 1.0)
  check("T: first person -> no camTrace lines", cam.T.state == "fpp" and count(W.lines, "camTrace t=") == n3)
  check("T: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ T2) 2026-10-22 Shooter mode (utpp_bodyturn.ShooterModeActive): the camera always
-- follows V's capsule - no Locomotion decoupling (and no 0.35 s decouple hold), no idle orbit; the
-- free-look hold of utpp_bodyturn's face-the-camera align (M.FreeLook) still works and its turn is predicted ("shooterMode align").
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local freeMove = true
  W.env.Game.UTPPLocomotionActive = function() return freeMove end
  local fnActive = false
  local bt = { exec = { why = "deadband", nativeOn = false }, ShooterModeActive = function() return fnActive end }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_bodyturn" then return bt end return baseGetMod(n) end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("T2: Shooter off: Locomotion active -> free (decoupled) camera, as today", cam.T.state == "tpp" and cam.S.look.free == true)
  freeMove = false
  frames(W, 0.1, 60)
  check("T2: Shooter off: Locomotion dropped 0.1 s ago -> the 0.35 s decouple hold keeps the free camera (today)",
        cam.S.look.free == true)
  freeMove = true
  frames(W, 0.3, 60)
  fnActive = true
  frames(W, 1 / 60, 60)
  check("T2: Shooter effective: Locomotion active -> decoupled == false the same frame (follow)", cam.S.look.free == false)
  frames(W, 1.0, 60)
  check("T2: Shooter: 1 s later still follow (Locomotion reported active, standing: no idle orbit)", cam.S.look.free == false)
  freeMove = false
  frames(W, 0.1, 60)
  freeMove = true
  frames(W, 0.3, 60)
  freeMove = false
  frames(W, 0.1, 60)
  check("T2: Shooter: Locomotion dropped 0.1 s ago -> no decouple hold (follow)", cam.S.look.free == false)
  -- trace: shooter=1 only while Shooter is effective
  local function lastTrace()
    for i = #W.lines, 1, -1 do if string.find(W.lines[i], "camTrace t=", 1, true) then return W.lines[i] end end
  end
  frames(W, 0.3, 60)
  local tFn = lastTrace()
  fnActive = false
  frames(W, 0.3, 60)
  local tOff = lastTrace()
  check("T2: camTrace carries ' shooter=1' only while Shooter is effective", tFn ~= nil and string.find(tFn, " shooter=1", 1, true) ~= nil
        and tOff ~= nil and string.find(tOff, " shooter=1", 1, true) == nil, tostring(tFn) .. " || " .. tostring(tOff))
  -- idle orbit: off the Shooter mode the standing V orbits (today), Shooter does not
  freeMove = false
  frames(W, 1.0, 60)
  check("T2: Shooter off, standing, Locomotion off: idle orbit free after the delay (today)", cam.S.look.free == true)
  fnActive = true
  frames(W, 1 / 60, 60)
  check("T2: Shooter effective: the idle orbit ends at once (follow)", cam.S.look.free == false)
  frames(W, 1.0, 60)
  check("T2: Shooter: standing 1 s -> still follow", cam.S.look.free == false)
  -- the module hold (M.FreeLook, utpp_bodyturn's align hold) works
  cam.FreeLook(true)
  frames(W, 0.2, 60)
  check("T2: Shooter: utpp_bodyturn's align hold (M.FreeLook(true)) still frees the camera", cam.S.look.free == true)
  frames(W, 0.1, 60)
  check("T2: Shooter: the align hold stays while held", cam.rt.freeHeld == true and cam.S.look.free == true)
  -- prediction of V's turn during the hold: why 'shooterMode align' like 'applied'
  local function turnRun(why)
    bt.exec.why = why
    frames(W, 0.2)
    local mx, n = 0, 0
    frames(W, 0.5, 60, function(i)
      W.yaw = CM.wrapDeg(W.yaw + 6)
      local f = W.env.Game.GetCameraSystem():GetActiveCameraForward()
      local ry = CM.yawPitchFromForward(f)
      if i > 3 and cam.vh.last then
        mx = math.max(mx, math.abs(CM.wrapDeg(ry - cam.vh.last.yaw)))
        n = n + 1
      end
    end)
    return mx, n
  end
  local errAlign, nAlign = turnRun("shooterMode align")
  local errNone = turnRun("shooterMode align waiting")
  check("T2: the body-turn prediction accepts why 'shooterMode align' (error ~0) and not 'shooterMode align waiting' (6 deg off)",
        nAlign > 20 and errAlign < 0.05 and errNone > 5.5, string.format("align %.3f waiting %.3f", errAlign, errNone))
  cam.FreeLook(false)
  fnActive = false
  frames(W, 1 / 60, 60) -- the flag is read once per camera frame
  cam.FreeLook(true)
  frames(W, 0.2, 60)
  check("T2: Shooter off: free look works (today)", cam.rt.freeHeld == true and cam.S.look.free == true)
  cam.FreeLook(false)
  frames(W, 0.1, 60)
  check("T2: Shooter off: release -> not held", cam.rt.freeHeld == false)
  check("T2: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ W) 2026-10-12 world camera (separate camera entity)
-- Codeware StaticEntitySystem mock (opts.wcam): spawn in gameplay, space check, entry on the world camera, world poses
-- written as is, V's turn between frames does not move the shown camera, setting off / on in third person, exit,
-- failures (no Codeware, no id, spawn timeout, space check, entity lost, not rendering) -> component path, session end
-- and shutdown (player gone: no game call; alive: deactivate + despawn), no mocked handle reachable.
do
  local function setupW(opts, before)
    local W = makeWorld(opts or { wcam = true })
    W.psm.LocomotionDetailed = 1
    local C = loadCore(W)
    if before then before(W) end
    local cam = loadCamera(W)
    withEnv(W, function() W.handlers.onInit() end)
    return W, C, cam
  end
  local function setW(W, C, k, v) withEnv(W, function() C.Set(k, v) end) end
  local function lastTrace(W)
    for i = #W.lines, 1, -1 do if string.find(W.lines[i], "camTrace t=", 1, true) then return W.lines[i] end end
    return ""
  end
  local function wcMocks(W) return { [W.wc] = true, [W.wcEnt] = true, [W.ses] = true } end
  local function wcRefs(W, cam)
    return findMockRefs({ cam = cam, onUpdate = W.handlers.onUpdate, onShutdown = W.handlers.onShutdown, onInit = W.handlers.onInit },
      wcMocks(W), "@" .. UTPP_CAMERA_PATH)
  end

  -- spawn, ready, entry, world poses, V turning, setting off / on, exit
  do
    local W, C, cam = setupW()
    check("W: camera.worldCamera registered (default on)", C.settings.defs["camera.worldCamera"] ~= nil
          and C.Get("camera.worldCamera") == true and cam.rt.worldCamera == true)
    W.yaw = 37
    frames(W, 1.4)
    check("W: no spawn while loading (player present < 1.5 s)", W.ses.spawns == 0 and cam.wc.state == "off")
    frames(W, 0.6)
    local spec = W.ses.spec or {}
    local pos, ori = spec.position or {}, spec.orientation or {}
    check("W: spawned once in gameplay: StaticEntitySystem, template utpp\\camera\\utpp_player_camera.ent, world origin, identity, "
          .. "attached, tag UTPPWorldCamera (leftovers despawned by the tag first)", W.ses.spawns == 1
          and spec.templatePath == "utpp\\camera\\utpp_player_camera.ent" and pos.x == 0 and pos.y == 0 and pos.z == 0
          and ori.i == 0 and ori.j == 0 and ori.k == 0 and ori.r == 1 and spec.attached == true and type(spec.tags) == "table"
          and spec.tags[1] == "UTPPWorldCamera" and W.ses.tagged == 1 and W.ses.tag == "UTPPWorldCamera",
          tostring(spec.templatePath))
    check("W: one 'world camera: spawned <how> entity=<id>' line", count(W.lines,
          "world camera: spawned Codeware StaticEntitySystem (templatePath via string) entity=9001") == 1, table.concat(W.lines, " | "))
    check("W: entity resolved after the spawn delay, space check ok -> ready (one line each)", cam.wc.state == "ready"
          and count(W.lines, "world camera: entity resolved after") == 1 and count(W.lines, "world camera: ready (space check ok") == 1
          and count(W.lines, "world camera: FAILED") == 0, cam.WorldCameraStatus())
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    check("W: entry on the world camera: rendering, 'world camera: active' once, utppCamera never activated nor written",
          cam.T.state == "tpp" and W.active == "wc" and W.wc.activations == 1 and W.utpp.activations == 0 and W.utpp.writes == 0
          and cam.G.use == "world" and count(W.lines, ": world camera Activate ok") == 1
          and count(W.lines, "world camera: active (enter epoch=") == 1 and count(W.lines, ", entity=9001)") == 1, W.active .. " " .. cam.WorldCameraStatus()
          .. " | " .. table.concat(W.lines, " | "))
    local r = cam.G.last
    local dq = math.abs(W.wc.wq.i - r.q.i) + math.abs(W.wc.wq.j - r.q.j) + math.abs(W.wc.wq.k - r.q.k) + math.abs(W.wc.wq.r - r.q.r)
    check("W: world pose written as is: position = solver res.world, orientation = res.q, FOV = res.fov (no entity-space "
          .. "conversion: res.localPos differs)", CM.dist(W.wc.wp, r.world) < 1e-9 and dq < 1e-9 and math.abs(W.wc.fov - r.fov) < 1e-9
          and CM.dist(r.localPos, r.world) > 50, string.format("d=%.6f dq=%.6f", CM.dist(W.wc.wp, r.world), dq))
    local camYaw = CM.yawPitchFromForward(CM.qrot(W.wc.wq, CM.v3(0, 1, 0)))
    check("W: the written pose is world space around V at (100, 50, 10) and looks along V (yaw 37)",
          CM.dist(W.wc.wp, W.pos) > 1.0 and CM.dist(W.wc.wp, W.pos) < 6 and math.abs(CM.wrapDeg(camYaw - 37)) < 2,
          string.format("(%.2f,%.2f,%.2f) yaw %.1f", W.wc.wp.x, W.wc.wp.y, W.wc.wp.z, camYaw))
    check("W: watchdog sees the world camera rendering (no re-activation)", cam.stats.reactivations == 0)
    check("W: camTrace wcam=1 while the world camera is written", string.find(lastTrace(W), "wcam=1", 1, true) ~= nil, lastTrace(W))
    check("W: Status() shows the world camera", string.find(cam.Status(), "wcam=ready use=world route=static entity=9001", 1, true) ~= nil,
          cam.Status())
    check("W: no mocked world camera handle (entity / component / system) reachable from the module state while playing",
          #wcRefs(W, cam) == 0, table.concat(wcRefs(W, cam), ", "))
    -- free camera (decoupled), V turned by 6 deg per game tick after the CET frame (utpp_bodyturn): the shown camera does
    -- not move with V - no prediction, no OnBodyStep rewrite
    W.env.Game.UTPPLocomotionActive = function() return true end
    local bt = { exec = { why = "applied", nativeOn = true }, execStats = { lastStep = 6 } }
    local baseGetMod = W.env.GetMod
    W.env.GetMod = function(n) if n == "utpp_bodyturn" then return bt end return baseGetMod(n) end
    setW(W, C, "camera.idleOrbit", false)
    frames(W, 0.5)
    local function turnRun()
      local mxY, mxP, n, steps = 0, 0, 0, 0
      frames(W, 0.5, 60, function(i)
        W.yaw = CM.wrapDeg(W.yaw + 6)
        local cs = W.env.Game.GetCameraSystem()
        local ry = CM.yawPitchFromForward(cs:GetActiveCameraForward())
        local xf = {}
        cs:GetActiveCameraWorldTransform(xf)
        if i > 3 and cam.vh.last then
          mxY = math.max(mxY, math.abs(CM.wrapDeg(ry - cam.vh.last.yaw)))
          mxP = math.max(mxP, CM.dist(CM.v3(xf.position.x, xf.position.y, xf.position.z), cam.vh.last))
          n = n + 1
        end
        withEnv(W, function() if cam.OnBodyStep(6) then steps = steps + 1 end end)
      end)
      return mxY, mxP, n, steps
    end
    local eY, eP, nW, stW = turnRun()
    check("W: free camera, V turning 6 deg per tick: the shown camera = the pose written (yaw and position), no prediction "
          .. "(pred 0), OnBodyStep does nothing", cam.S.look.free == true and nW > 20 and eY < 1e-6 and eP < 1e-6
          and (cam.G.pred or 0) == 0 and stW == 0 and string.find(lastTrace(W), "pred=0.00", 1, true) ~= nil,
          string.format("yaw %.4f pos %.4f n %d steps %d", eY, eP, nW, stW))
    -- setting off in third person: utppCamera takes over at the shown pose, the entity is despawned
    local despawns0 = W.ses.despawns
    setW(W, C, "camera.worldCamera", false)
    frames(W, 0.1)
    check("W: camera.worldCamera off in third person: switch to utppCamera (activated, rendering), world camera deactivated "
          .. "and despawned (one line each)", W.active == "utpp" and W.utpp.activations == 1 and W.wc.deactivations == 1
          and W.ses.despawns == despawns0 + 1 and cam.G.use == "comp" and cam.wc.id == nil
          and count(W.lines, "camera switch world camera -> utppCamera (camera.worldCamera off), Activate(0) ok") == 1
          and count(W.lines, "world camera: deactivated (camera.worldCamera off)") == 1
          and count(W.lines, "world camera: released (camera.worldCamera off): StaticEntitySystem.DespawnEntity true") == 1,
          W.active .. " " .. cam.WorldCameraStatus())
    -- component path: the entity turn is carried for a frame again unless predicted (contrast to the world camera)
    setW(W, C, "camera.predictBodyTurn", false)
    local eYc = turnRun()
    check("W: component camera (setting off, no prediction): V's 6 deg tick turn shows in the camera (the root cause)",
          eYc > 5.5 and string.find(lastTrace(W), "wcam=0", 1, true) ~= nil, string.format("%.2f", eYc))
    setW(W, C, "camera.predictBodyTurn", true)
    -- setting on again in third person: new spawn, ready, the world camera takes over at the shown pose
    setW(W, C, "camera.worldCamera", true)
    frames(W, 0.6)
    check("W: camera.worldCamera on again in third person: spawned again, ready, takes over from utppCamera",
          W.ses.spawns == 2 and W.active == "wc" and cam.G.use == "world" and W.utpp.deactivations == 1
          and count(W.lines, "camera switch utppCamera -> world camera (world camera ready in third person), Activate(0) ok") == 1
          and count(W.lines, "world camera: active (world camera ready in third person)") == 1, W.active .. " " .. cam.WorldCameraStatus())
    -- exit to first person: FPP camera activated, then the world camera deactivated (one line), entity kept for the next entry
    W.env.Game.UTPPLocomotionActive = nil
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    check("W: exit: FPP camera renders, world camera deactivated once ('world camera: deactivated'), entity kept",
          cam.T.state == "fpp" and W.active == "fpp" and W.wc.deactivations == 2 and cam.G.use == nil and cam.wc.state == "ready"
          and count(W.lines, "world camera: deactivated (") == 2, W.active .. " " .. cam.WorldCameraStatus())
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    check("W: re-entry on the same entity (no new spawn)", W.active == "wc" and W.ses.spawns == 2 and cam.G.use == "world")
    -- alive shutdown in third person: FPP camera back, world camera deactivated, entity despawned, nothing reachable
    local d0 = W.ses.despawns
    withEnv(W, function() W.handlers.onShutdown() end)
    check("W: alive shutdown: FPP camera activated, world camera deactivated + despawned (released line), nothing reachable",
          W.active == "fpp" and W.ses.despawns == d0 + 1 and cam.wc.id == nil and #wcRefs(W, cam) == 0
          and count(W.lines, "world camera: released (shutdown): StaticEntitySystem.DespawnEntity true") == 1,
          table.concat(wcRefs(W, cam), ", "))
    check("W: no update errors (main run)", cam.stats.errors == 0 and count(W.lines, "update error") == 0
          and count(W.lines, "world camera: FAILED") == 0, table.concat(W.lines, " | "))
    os.remove(settingsFile)
  end

  -- session end / shutdown with the player gone: the entity id is forgotten without any game call
  do
    local W, C, cam = setupW()
    frames(W, 2.0)
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    local c0 = W.ses.calls
    W.player = false
    frames(W, 0.2)
    check("W: no player (load / exit): world camera forgotten without game calls (one line), nothing reachable",
          W.ses.calls == c0 and cam.wc.id == nil and cam.G.use == nil
          and count(W.lines, "world camera: released (session reset: no player): entity 9001 forgotten - no game calls") == 1
          and #wcRefs(W, cam) == 0, string.format("%d calls", W.ses.calls - c0))
    withEnv(W, function() W.handlers.onShutdown() end)
    check("W: shutdown with the player gone: no world camera game call", W.ses.calls == c0 and cam.wc.id == nil)
    -- the shutdown arrives in third person while the player is already gone (ExitGame): no call on the dead handles
    local W2, C2, cam2 = setupW()
    frames(W2, 2.0)
    withEnv(W2, function() cam2.Toggle("key") end)
    frames(W2, 1.0)
    local c2 = W2.ses.calls
    W2.player = false
    withEnv(W2, function() W2.handlers.onShutdown() end)
    check("W: ExitGame in third person (player gone, onShutdown first): no world camera call, id forgotten, nothing reachable",
          W2.ses.calls == c2 and cam2.wc.id == nil and #wcRefs(W2, cam2) == 0
          and count(W2.lines, "world camera: released (session reset: shutdown): entity 9001 forgotten - no game calls") == 1,
          string.format("%d calls", W2.ses.calls - c2))
    W2.player = true
    frames(W2, 0.5)
    check("W: after onShutdown no further world camera calls", W2.ses.calls == c2)
    os.remove(settingsFile)
  end

  -- failures -> component camera (one FAILED line each, the component path unchanged)
  local function failCase(name, opts, before, setupFn, needle)
    local W, C, cam = setupW(opts, before)
    frames(W, 2.0)
    if setupFn then setupFn(W) end
    frames(W, 6.0)
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    frames(W, 3.0)
    check("W: " .. name .. ": one 'world camera: FAILED ... -> component camera' line, third person on utppCamera",
          count(W.lines, "world camera: FAILED") == 1 and count(W.lines, needle) == 1 and W.active == "utpp"
          and W.utpp.activations == 1 and cam.G.use == "comp" and cam.wc.state == "failed" and cam.stats.errors == 0
          and count(W.lines, "update error") == 0, cam.WorldCameraStatus() .. " | " .. table.concat(W.lines, " | "))
    os.remove(settingsFile)
    return W, cam
  end
  local Wn = failCase("no Codeware / no exEntitySpawner", {}, nil, nil,
    "world camera: FAILED Codeware StaticEntitySystem unavailable")
  check("W: no Codeware: the failure names both routes", count(Wn.lines, "CET exEntitySpawner unavailable -> component camera") == 1)
  local Wi = failCase("SpawnEntity returns no id", { wcam = true }, function(W) W.wcSpawnId = { hash = 0 } end, nil,
    "world camera: FAILED StaticEntitySystem.SpawnEntity failed (ResRef.FromString:")
  check("W: no id: both templatePath forms tried once (ResRef.FromString unavailable offline), no retry", Wi.ses.spawns == 1
        and count(Wi.lines, "string: no entity id") == 1, Wi.ses.spawns)
  local Wt = failCase("spawn timeout", { wcam = true, wcDelay = 100 }, nil, nil,
    "world camera: FAILED spawn timeout: entity 9001 not resolved after 5.0")
  check("W: spawn timeout: the pending entity is despawned", Wt.ses.despawns == 1 and count(Wt.lines, "world camera: released (failed)") == 1)
  failCase("space check", { wcam = true }, function(W) W.wcSpace = CM.v3(0, 0, 1) end, nil,
    "world camera: FAILED space check: component 1.00 m away from the written position")
  -- not ready: retried every 0.5 s, then the failure after 10 s
  local Wr = failCase("spawner never ready", { wcam = true }, function(W) W.sesReady = false end, nil,
    "world camera: FAILED Codeware StaticEntitySystem not ready")
  check("W: not ready: no spawn call", Wr.ses.spawns == 0)

  -- failures while the world camera shows third person: utppCamera takes over at the shown pose
  do
    local W, C, cam = setupW()
    frames(W, 2.0)
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 1.0)
    local shown = cam.vh.last
    W.wcLost = true
    frames(W, 0.05)
    local uw = W.utppWorld()
    check("W: entity lost in third person: FAILED once, utppCamera activated at the shown pose and rendering",
          count(W.lines, "world camera: FAILED entity lost (StaticEntitySystem.GetEntity: nil) -> component camera") == 1
          and W.active == "utpp" and W.utpp.activations == 1 and cam.G.use == "comp"
          and count(W.lines, "camera switch world camera -> utppCamera (world camera failed), Activate(0) ok") == 1
          and CM.dist(uw, CM.v3(shown.x, shown.y, shown.z)) < 0.2, string.format("%.3f m", CM.dist(uw, CM.v3(shown.x, shown.y, shown.z))))
    os.remove(settingsFile)
  end
  do
    local W, C, cam = setupW(nil, function(W) W.wcNoRender = true end)
    frames(W, 2.0)
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 5.0)
    check("W: world camera activated but not rendering: 2 watchdog re-activations, then FAILED -> utppCamera renders",
          count(W.lines, "watchdog: active camera") == 2 and count(W.lines, "Activate again") == 2
          and count(W.lines, "world camera: FAILED not rendering: active camera") == 1 and W.active == "utpp"
          and cam.G.use == "comp" and W.ses.despawns == 1, cam.WorldCameraStatus() .. " " .. W.active)
    -- batch 6 (male V test 23:37:48.9: the failure 3 s after a save load was final, game look write OFF for the session):
    -- a transient failure is retried after 8 s (camera.worldCameraRetry, default on), the world camera takes over again
    check("W: retry: the FAILED line announces the retry, Status() counts down",
          count(W.lines, "-> component camera (retry 1/3 in 8 s)") == 1 and cam.wc.failAt ~= nil
          and string.find(cam.WorldCameraStatus(), "failRetries=0 retryIn=", 1, true) ~= nil, cam.WorldCameraStatus())
    local failAt = cam.wc.failAt
    frames(W, math.max(0, failAt + 7.5 - W.clock))
    check("W: retry: no new spawn before 8 s", W.ses.spawns == 1 and cam.wc.state == "failed" and W.active == "utpp",
          cam.WorldCameraStatus())
    W.wcNoRender = false -- the game renders it now (the loading camera is gone)
    frames(W, 2.5)
    check("W: retry 1/3 after 8 s: logged with the failure, new spawn, ready, takes over at the shown pose, renders, look line",
          count(W.lines, "world camera: retry 1/3 - new spawn 8 s after the failure (not rendering: active camera") == 1
          and W.ses.spawns == 2 and cam.wc.state == "ready" and cam.G.use == "world" and W.active == "wc"
          and count(W.lines, "camera switch utppCamera -> world camera (world camera ready in third person), Activate(0) ok") == 1
          and count(W.lines, "the game look write is off meanwhile (world camera not shown)") == 1
          and string.find(cam.WorldCameraStatus(), "wcam=ready use=world", 1, true) ~= nil
          and string.find(cam.WorldCameraStatus(), "failRetries=1 retryIn=-", 1, true) ~= nil,
          cam.WorldCameraStatus() .. " | " .. table.concat(W.lines, " | "))
    os.remove(settingsFile)
  end
  do -- batch 6: a camera that never renders: 3 retries (4 spawns), then final; setting off: the old single failure
    local W, C, cam = setupW(nil, function(W) W.wcNoRender = true end)
    frames(W, 2.0)
    withEnv(W, function() cam.Toggle("key") end)
    frames(W, 45.0)
    check("W: retry: at most 3 spawn attempts after failures, then final (component camera)", W.ses.spawns == 4
          and cam.wc.state == "failed" and cam.wc.failRetries == 3 and count(W.lines, "world camera: retry 3/3") == 1
          and count(W.lines, "world camera: retry 4/3") == 0 and count(W.lines, "world camera: FAILED not rendering") == 4
          and count(W.lines, "(retry 4/3") == 0 and W.active == "utpp" and cam.G.use == "comp"
          and string.find(cam.WorldCameraStatus(), "failRetries=3 retryIn=-", 1, true) ~= nil, cam.WorldCameraStatus())
    os.remove(settingsFile)
    local W2, C2, cam2 = setupW(nil, function(W) W.wcNoRender = true end)
    check("W: camera.worldCameraRetry registered (default on)", C2.settings.defs["camera.worldCameraRetry"] ~= nil
          and C2.Get("camera.worldCameraRetry") == true and cam2.rt.worldCameraRetry == true)
    setW(W2, C2, "camera.worldCameraRetry", false)
    frames(W2, 2.0)
    withEnv(W2, function() cam2.Toggle("key") end)
    frames(W2, 25.0)
    check("W: camera.worldCameraRetry off: one failure for the session (old behaviour), no retry text", W2.ses.spawns == 1
          and count(W2.lines, "world camera: FAILED") == 1 and count(W2.lines, "(retry") == 0 and count(W2.lines, "world camera: retry") == 0
          and count(W2.lines, "world camera retry setting -> false") == 1 and W2.active == "utpp", table.concat(W2.lines, " | "))
    os.remove(settingsFile)
  end
end

-- ================================================================ S) 2026-10-12 round 3 game look sync
-- Mocked game look: W.look = the FPP camera world yaw. The game's mouse turns V (heading unlocked) or only the look
-- (headingLocked) at 0.05 deg per unit (half the camera's uncalibrated 0.1: the 2x mismatch of the 22:48 log); V's turns
-- by the body turn executor carry the look unless the param set VehicleCombat (maintainWorldSpaceHeading) is active.
-- Game.UTPPLookWorldSet = utpp_bodyturn.reds (W.lwCalls); W.lwIgnore = the set has no effect (failure path).
local function lookWorld(opts)
  opts = opts or {}
  local W = makeWorld({ wcam = true })
  W.look = W.yaw
  W.set = "Default"
  W.lwCalls = {}
  if opts.reds ~= false then
    W.env.Game.UTPPLookWorldSet = function(on, set)
      W.lwCalls[#W.lwCalls + 1] = { on, set }
      W.set = on and set or "Default"
      return true
    end
  end
  local baseQueue = W.playerObj.QueueEvent
  W.playerObj.QueueEvent = function(self, e)
    if e.className == "gameSetCameraParamsEvent" then W.set = e.paramsName; W.cetSets = (W.cetSets or 0) + 1 end
    return baseQueue(self, e)
  end
  -- one game tick before the CET frame: mouse units, V's turn (deg)
  function W.tick(mouseX, turn)
    local d = -(mouseX or 0) * 0.05
    if W.fpp.headingLocked then W.look = CM.wrapDeg(W.look + d) else W.yaw = CM.wrapDeg(W.yaw + d); W.look = W.yaw end
    if turn and turn ~= 0 then
      W.yaw = CM.wrapDeg(W.yaw + turn)
      if not (W.set == "VehicleCombat" and not W.lwIgnore) then W.look = CM.wrapDeg(W.look + turn) end
    end
    if mouseX and mouseX ~= 0 then W.observers["PlayerPuppet.OnAction"](nil, { name = "CameraMouseX", value = mouseX }) end
  end
  return W
end
local function shown(cam) return cam.G.last and cam.G.last.yaw end
local function lastTrace(W)
  for i = #W.lines, 1, -1 do if string.find(W.lines[i], "camTrace t=", 1, true) then return W.lines[i] end end
end
do
  local W = lookWorld()
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.lookSync", true) end) -- 2026-10-12: default OFF, the sync path is tested on
  C.Set("camera.idleOrbit", false)
  frames(W, 1.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 2.0)
  check("S: third person, follow mode: no param set sent (heading not locked)", cam.T.state == "tpp" and #W.lwCalls == 0
        and W.set == "Default", string.format("%d calls, set %s, lines: %s", #W.lwCalls, tostring(W.set), table.concat(W.lines, " | ")))
  cam.FreeLook(true)
  frames(W, 0.1)
  check("S: free look -> param set VehicleCombat via Game.UTPPLookWorldSet, route logged once", #W.lwCalls == 1
        and W.lwCalls[1][1] == true and W.lwCalls[1][2] == "VehicleCombat" and W.fpp.headingLocked == true
        and count(W.lines, "game look sync: FPP camera param set VehicleCombat via Game.UTPPLookWorldSet") == 1,
        table.concat(W.lines, " | "))
  frames(W, 0.3)
  local before = cam.G.lookUsed
  frames(W, 0.2, 60, function() W.tick(10, 0) end)
  check("S: look moved by the mouse while V stood -> confirmed (one log line), camera = game look",
        before == false and cam.G.lookUsed == true and cam.LookSynced() == true
        and count(W.lines, "game look sync confirmed") == 1, cam.LookSyncStatus())
  frames(W, 1.0, 60, function() W.tick(10, 0) end)
  local err = CM.wrapDeg(shown(cam) - W.look)
  check("S: free look + mouse: shown yaw = the game's look (the game's mouse scale, no 2x drift)", math.abs(err) < 0.01, err)
  local maxErr = 0
  frames(W, 0.5, 60, function(i)
    W.tick(0, 6)
    if i > 1 then maxErr = math.max(maxErr, math.abs(CM.wrapDeg(shown(cam) - W.look))) end
  end)
  check("S: V turned 6 deg per tick (body turn executor): the look keeps its world heading, camera = look, not carried",
        maxErr < 0.01 and count(W.lines, "sent again") == 0, maxErr)
  frames(W, 0.3, 60, function() W.tick(-7, 4) end)
  check("S: mouse + V turning together: camera = look", math.abs(CM.wrapDeg(shown(cam) - W.look)) < 0.01,
        CM.wrapDeg(shown(cam) - W.look))
  local tr = lastTrace(W)
  check("S: camTrace sync=on lerr=0.0 lsend=1", tr ~= nil and string.find(tr, "sync=on lerr=0.0 lsend=1", 1, true) ~= nil
        or (tr ~= nil and string.find(tr, "sync=on lerr=-0.0 lsend=1", 1, true) ~= nil), tr)
  check("S: Status() shows the look sync", string.find(cam.Status(), "lookSync=true set=VehicleCombat sent=true route=Game.UTPPLookWorldSet ok=true broken=false used=true", 1, true) ~= nil,
        cam.Status())
  -- release: the game's set is restored, the camera back to follow
  cam.FreeLook(false)
  frames(W, 0.5)
  check("S: free look released -> restore sent (UTPPLookWorldSet false), heading unlocked, not synced",
        #W.lwCalls == 2 and W.lwCalls[2][1] == false and W.set == "Default" and W.fpp.headingLocked == false
        and cam.LookSynced() == false, #W.lwCalls)
  -- confirmed stays for the session: the next free look is synced at once
  cam.FreeLook(true)
  frames(W, 0.4, 60, function() W.tick(5, 0) end)
  check("S: next free look: synced without a new confirmation (one confirm line in total)", cam.LookSynced() == true
        and #W.lwCalls == 3 and count(W.lines, "game look sync confirmed") == 1, cam.LookSyncStatus())
  -- setting off while locked -> own yaw at once, the set kept until the lock ends; on again -> synced (no new send)
  C.Set("camera.lookSync", false)
  frames(W, 0.1, 60, function() W.tick(5, 0) end)
  check("S: camera.lookSync off while locked -> own camera yaw, set kept (heading stays locked)", #W.lwCalls == 3
        and cam.LookSynced() == false and W.fpp.headingLocked == true and W.set == "VehicleCombat", #W.lwCalls)
  C.Set("camera.lookSync", true)
  frames(W, 0.4, 60, function() W.tick(5, 0) end)
  check("S: camera.lookSync on again -> synced, no new send", #W.lwCalls == 3 and cam.LookSynced() == true)
  C.Set("camera.lookSync", false)
  cam.FreeLook(false)
  frames(W, 0.3)
  check("S: lock ends with camera.lookSync off -> the game's set restored", #W.lwCalls == 4 and W.lwCalls[4][1] == false
        and W.set == "Default")
  cam.FreeLook(true)
  frames(W, 0.3)
  check("S: camera.lookSync off -> no set sent for a new lock (old behaviour)", #W.lwCalls == 4 and W.fpp.headingLocked == true)
  cam.FreeLook(false)
  frames(W, 0.3)
  C.Set("camera.lookSync", true)
  -- leaving third person while locked restores the set as well
  cam.FreeLook(true)
  frames(W, 0.2)
  local nOn = #W.lwCalls
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.5)
  check("S: exit to first person while locked -> restore sent", cam.T.state == "fpp" and #W.lwCalls == nOn + 1
        and W.lwCalls[#W.lwCalls][1] == false and W.set == "Default")
  local failed = {}
  for _, l in ipairs(W.lines) do if string.find(l, "FAILED", 1, true) or string.find(l, "update error", 1, true) then failed[#failed + 1] = l end end
  check("S: no update errors, no FAILED line", cam.stats.errors == 0 and #failed == 0, table.concat(failed, " | "))
  os.remove(settingsFile)
end
do -- the set does not keep the look (another set applied / no effect): resend twice, then FAILED once, own yaw
  local W = lookWorld()
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.lookSync", true) end) -- 2026-10-12: default OFF, the sync path is tested on
  C.Set("camera.idleOrbit", false)
  frames(W, 1.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  W.lwIgnore = true
  cam.FreeLook(true)
  frames(W, 0.4, 60, function() W.tick(10, 0) end)
  check("S2: confirmed by the mouse first", cam.LookSynced() == true)
  frames(W, 2.0, 60, function() W.tick(0, 5) end)
  check("S2: look carried by V's turn -> 2 resends (logged), then one FAILED line, own camera yaw",
        count(W.lines, "param set VehicleCombat sent again") == 2 and count(W.lines, "game look sync FAILED") == 1
        and cam.LookSynced() == false and #W.lwCalls == 3, table.concat(W.lines, " | "))
  local tr = lastTrace(W)
  check("S2: camTrace sync=broken", tr ~= nil and string.find(tr, "sync=broken", 1, true) ~= nil, tr)
  local y0 = shown(cam)
  frames(W, 0.3, 60, function() W.tick(0, 5) end)
  check("S2: fallback: the own free camera yaw is not carried by V's turn", math.abs(CM.wrapDeg(shown(cam) - y0)) < 0.01,
        CM.wrapDeg(shown(cam) - y0))
  check("S2: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end
do -- no utpp_bodyturn.reds global: the event is queued from CET (gameSetCameraParamsEvent), the sync works the same
  local W = lookWorld({ reds = false })
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.lookSync", true) end) -- 2026-10-12: default OFF, the sync path is tested on
  C.Set("camera.idleOrbit", false)
  frames(W, 1.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  cam.FreeLook(true)
  frames(W, 0.5, 60, function() W.tick(10, 0) end)
  frames(W, 0.5, 60, function() W.tick(0, 6) end)
  check("S3: CET event route: set VehicleCombat queued, route logged, synced and not carried",
        W.cetSets == 1 and W.set == "VehicleCombat" and cam.LookSynced() == true
        and count(W.lines, "via CET gameSetCameraParamsEvent") == 1
        and math.abs(CM.wrapDeg(shown(cam) - W.look)) < 0.01, cam.LookSyncStatus())
  cam.FreeLook(false)
  frames(W, 0.3)
  check("S3: release queues Default from CET", W.cetSets == 2 and W.set == "Default")
  check("S3: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ LW) 2026-10-12 (UTPP.dll 0.6.5) game look write
-- Mocked native: Game.UTPPSetGameLook(yaw, pitch, ahead) records the call (0.6.5b: { yaw, pitch, headingLocked, ahead })
-- and stores the target (returns W.gl.live); UTPPStopGameLook clears it; W.gl.tripped = the native divergence guard
-- tripped (status " tripped=1 "). The game tick (W.lwTick) turns V and applies the game's mouse (W.tick: without the write
-- the locked look turns with V), then the hooked FPP camera update sets the look to the stored target (heading locked).
local function lwWorld(opts)
  opts = opts or {}
  local W = lookWorld()
  W.gl = { sets = {}, stops = 0, live = true, status = 0 }
  if opts.native ~= false then
    W.env.Game.UTPPSetGameLook = function(yaw, pitch, ahead)
      W.gl.sets[#W.gl.sets + 1] = { yaw, pitch, W.fpp.headingLocked, ahead }
      W.gl.target = W.gl.live and { yaw = yaw, pitch = pitch } or nil
      return W.gl.live
    end
    W.env.Game.UTPPStopGameLook = function()
      W.gl.stops = W.gl.stops + 1
      local was = W.gl.target ~= nil
      W.gl.target = nil
      return was
    end
    W.env.Game.UTPPGameLookStatus = function()
      W.gl.status = W.gl.status + 1
      -- (0.6.5b round 2: tripCount / tripPermanent as the native reports them)
      return "gamelook hook=1 active=" .. (W.gl.target and 1 or 0) .. (W.gl.tripped and (" tripped=1 tripCount="
        .. tostring(W.gl.tripCount or 1) .. (W.gl.permanent and " tripPermanent=1" or "")) or "") .. " last=\""
        .. (W.gl.tripped and "tripped: mock" or (W.gl.live and "ok" or "RTTI layout: mock")) .. "\""
    end
  end
  -- 2026-10-13 round 3 (opts.unlockTurn): the game's unlock as found in 2.31 (GameLook.cpp evidence 1 / 3: a change of
  -- headingLocked to false calls RotateEntityToCamera = entity yaw + YawInput, i.e. V turns onto the look; the pitch
  -- stays). W.unlockTurns = degrees V turned by unlocks (W.unlocks = count).
  W.unlockTurns, W.unlocks = 0, 0
  local wasLocked = false
  function W.lwTick(mouseX, turn)
    if opts.unlockTurn and wasLocked and not W.fpp.headingLocked then
      W.unlocks = W.unlocks + 1
      W.unlockTurns = W.unlockTurns + math.abs(CM.wrapDeg(W.look - W.yaw))
      W.yaw = W.look
    end
    wasLocked = W.fpp.headingLocked == true
    W.tick(mouseX, turn)
    if W.gl.target and W.fpp.headingLocked then W.look = W.gl.target.yaw; W.fppPitch = W.gl.target.pitch end
  end
  return W
end
local function lwTrace(W)
  for i = #W.lines, 1, -1 do if string.find(W.lines[i], "camTrace t=", 1, true) then return W.lines[i] end end
  return ""
end
local function lwSetup(opts)
  local W = lwWorld(opts)
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.idleOrbit", false) end)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  return W, C, cam
end
do
  local W, C, cam = lwSetup()
  check("LW: camera.lookWrite registered (default on)", C.settings.defs["camera.lookWrite"] ~= nil
        and C.Get("camera.lookWrite") == true and cam.cfg.lookWrite == true)
  check("LW: third person on the world camera, follow mode (heading not locked): no game look write",
        cam.T.state == "tpp" and cam.G.use == "world" and #W.gl.sets == 0 and cam.G.lwState == "off"
        and W.fpp.headingLocked == false, string.format("%s %s %d", cam.T.state, tostring(cam.G.use), #W.gl.sets))
  cam.FreeLook(true)
  local n0 = #W.gl.sets
  frames(W, 0.5, 60, function() W.lwTick(10, 0) end)
  local last = W.gl.sets[#W.gl.sets] or {}
  check("LW: free camera: UTPPSetGameLook every frame with the shown camera yaw / pitch (heading locked), ON logged once",
        #W.gl.sets - n0 >= 29 and last[3] == true and math.abs(CM.wrapDeg(last[1] - cam.G.last.yaw)) < 1e-9
        and math.abs(last[2] - cam.G.last.pitch) < 1e-9 and count(W.lines, "game look write ON (free)") == 1,
        string.format("%d sets, last %s / %s", #W.gl.sets - n0, tostring(last[1]), tostring(last[2])))
  -- V turned 6 deg per tick by the body turn executor: the game's look stays on the camera (not carried)
  local maxErr, n = 0, 0
  frames(W, 0.5, 60, function(i)
    W.lwTick(0, 6)
    if i > 1 then maxErr = math.max(maxErr, math.abs(CM.wrapDeg(W.look - shown(cam)))); n = n + 1 end
  end)
  check("LW: V turning 6 deg per tick: the game's look = the shown camera yaw every tick", n > 20 and maxErr < 1e-6, maxErr)
  -- 0.6.5b: the frame's write passes entityYawAheadDeg 0; utpp_bodyturn's step (it runs after the camera in the CET frame)
  -- is sent again with the same target and the exact step (M.OnBodyLookStep), once
  local lastS = W.gl.sets[#W.gl.sets]
  check("LW: the frame's write passes entityYawAheadDeg 0 (3rd argument)", lastS[4] == 0, tostring(lastS[4]))
  local nB = #W.gl.sets
  local okB
  withEnv(W, function() okB = cam.OnBodyLookStep(7.25) end)
  local rs = W.gl.sets[#W.gl.sets]
  check("LW: OnBodyLookStep(7.25) after the camera frame: UTPPSetGameLook again with the same yaw / pitch and ahead 7.25",
        okB == true and #W.gl.sets == nB + 1 and rs[1] == lastS[1] and rs[2] == lastS[2] and rs[3] == true and rs[4] == 7.25,
        string.format("ok %s sets +%d ahead %s", tostring(okB), #W.gl.sets - nB, tostring(rs[4])))
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  check("LW: the next frame's write is back to ahead 0 (one-shot), trace / status count the ahead send",
        #W.gl.sets == nB + 2 and W.gl.sets[#W.gl.sets][4] == 0 and string.find(cam.Status(), "aheadSends=1 ", 1, true) ~= nil,
        cam.Status())
  -- utpp_bodyturn measured BEFORE the camera: the step is kept and goes with the camera's own write of the same frame
  cam.G.btBefore = true
  local nP = #W.gl.sets
  local okP
  withEnv(W, function() okP = cam.OnBodyLookStep(-3.5) end)
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  local pA = W.gl.sets[#W.gl.sets]
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  local pB = W.gl.sets[#W.gl.sets]
  cam.G.btBefore = nil
  check("LW: bodyturn BEFORE the camera: no extra call, the step rides on the camera's write of that frame, then 0",
        okP == true and #W.gl.sets == nP + 2 and pA[4] == -3.5 and pB[4] == 0,
        string.format("ok %s sets +%d ahead %s / %s", tostring(okP), #W.gl.sets - nP, tostring(pA[4]), tostring(pB[4])))
  -- mouse + V turning together
  local y0 = shown(cam)
  maxErr = 0
  frames(W, 0.5, 60, function(i)
    W.lwTick(-7, 4)
    if i > 1 then maxErr = math.max(maxErr, math.abs(CM.wrapDeg(W.look - shown(cam)))) end
  end)
  check("LW: mouse + V turning: the camera follows the mouse, the game's look = the camera (one frame behind the mouse at most)",
        math.abs(CM.wrapDeg(shown(cam) - y0)) > 5 and maxErr < 2.0, string.format("moved %.2f maxErr %.3f",
        CM.wrapDeg(shown(cam) - y0), maxErr))
  frames(W, 0.3, 60, function() W.lwTick(0, 6) end)
  local tr = lwTrace(W)
  local lwerr = tonumber(string.match(tr, "lwerr=(%-?[%d%.]+)"))
  local lwperr = tonumber(string.match(tr, "lwperr=(%-?[%d%.]+)"))
  check("LW: camTrace lw=1, lwerr / lwperr = read-back - written ~0", string.find(tr, " lw=1 ", 1, true) ~= nil
        and lwerr ~= nil and math.abs(lwerr) < 0.01 and lwperr ~= nil and math.abs(lwperr) < 0.01, tr)
  check("LW: game look status logged while writing (trace on, every 2 s)", count(W.lines, "game look status: gamelook hook=1") >= 1)
  check("LW: Status() shows the game look write", string.find(cam.Status(), "lookWrite=true state=on", 1, true) ~= nil, cam.Status())
  -- free look released -> follow mode, heading unlocked: one stop, OFF logged, no more writes
  cam.FreeLook(false)
  frames(W, 0.3, 60, function() W.lwTick(0, 0) end)
  local nOff = #W.gl.sets
  frames(W, 0.3, 60, function() W.lwTick(0, 3) end)
  check("LW: free look released (heading unlocked): one UTPPStopGameLook, 'OFF' logged once, no writes in follow mode",
        W.gl.stops == 1 and cam.G.lwState == "off" and count(W.lines, "game look write OFF (heading unlocked (follow))") == 1 and #W.gl.sets == nOff
        and W.fpp.headingLocked == false, string.format("stops %d state %s", W.gl.stops, tostring(cam.G.lwState)))
  check("LW: camTrace lw=0 when not written", string.find(lwTrace(W), " lw=0 lwerr=- lwperr=-", 1, true) ~= nil, lwTrace(W))
  local nR = #W.gl.sets
  local okR
  withEnv(W, function() okR = cam.OnBodyLookStep(5) end)
  check("LW: nothing written this frame (follow mode): OnBodyLookStep sends nothing", okR == false and #W.gl.sets == nR)
  -- setting off while free: stop, no writes; on again: writes resume
  cam.FreeLook(true)
  frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  withEnv(W, function() C.Set("camera.lookWrite", false) end)
  frames(W, 0.1, 60, function() W.lwTick(0, 0) end)
  local nS = #W.gl.sets
  frames(W, 0.3, 60, function() W.lwTick(0, 6) end)
  check("LW: camera.lookWrite off while free: stop (logged), no writes, the look turns with V again (old behaviour)",
        W.gl.stops == 2 and #W.gl.sets == nS and count(W.lines, "game look write OFF (camera.lookWrite off)") == 1
        and math.abs(CM.wrapDeg(W.look - shown(cam))) > 10, string.format("stops %d sets %d/%d err %.1f", W.gl.stops,
        #W.gl.sets, nS, CM.wrapDeg(W.look - shown(cam))))
  withEnv(W, function() C.Set("camera.lookWrite", true) end)
  frames(W, 0.2, 60, function() W.lwTick(0, 6) end)
  -- (free look pressed again above: ON line 3)
  check("LW: camera.lookWrite on again: writes resume, the look = the camera", #W.gl.sets > nS
        and math.abs(CM.wrapDeg(W.look - shown(cam))) < 1e-6 and count(W.lines, "game look write ON (free)") == 3,
        string.format("sets %d/%d err %.3f ON %d | %s", #W.gl.sets, nS, CM.wrapDeg(W.look - shown(cam)),
        count(W.lines, "game look write ON (free)"), table.concat((function() local o = {} for _, l in ipairs(W.lines) do
        if string.find(l, "game look", 1, true) and not string.find(l, "status", 1, true) then o[#o + 1] = l end end return o end)(), " | ")))
  -- the native refuses (hook not installed / RTTI / component check): one line with its status, lw=0, no spam
  W.gl.live = false
  frames(W, 0.5, 60, function() W.lwTick(0, 0) end)
  check("LW: native returns false: one 'not possible' line with the native status, lw=0, still asked each frame",
        cam.G.lwState == "fail" and count(W.lines, "game look write not possible (no write): gamelook hook=1 active=0 "
        .. "last=\"RTTI layout: mock\"") == 1 and string.find(lwTrace(W), " lw=0 ", 1, true) ~= nil
        and string.find(cam.Status(), "state=fail", 1, true) ~= nil, cam.Status())
  W.gl.live = true
  frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  check("LW: native live again -> ON again", cam.G.lwState == "on" and count(W.lines, "game look write ON (free)") == 4)
  -- exit to first person: stop once more, no writes in first person
  cam.FreeLook(false)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0, 60, function() W.lwTick(0, 0) end)
  local nF = #W.gl.sets
  frames(W, 0.5, 60, function() W.lwTick(5, 0) end)
  check("LW: first person: no writes, stopped", cam.T.state == "fpp" and #W.gl.sets == nF and cam.G.lwState == "off"
        and W.gl.target == nil)
  check("LW: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end
-- older UTPP.dll (no native): one line, no errors, the camera works as before
do
  local W, C, cam = lwSetup({ native = false })
  cam.FreeLook(true)
  frames(W, 1.0, 60, function() W.lwTick(10, 6) end)
  check("LW2: no UTPPSetGameLook (UTPP.dll < 0.6.5): one 'unavailable' line, free camera works, no errors",
        count(W.lines, "game look write: Game.UTPPSetGameLook unavailable") == 1 and cam.S.look.free == true
        and cam.G.lwState ~= "on" and cam.stats.errors == 0 and count(W.lines, "update error") == 0
        and string.find(cam.Status(), "state=unavailable", 1, true) ~= nil, cam.Status())
  os.remove(settingsFile)
  local _ = C
end
-- with camera.lookSync on as well: the camera keeps following the mouse (it does not follow the look it writes)
do
  local W, C, cam = lwSetup()
  withEnv(W, function() C.Set("camera.lookSync", true) end)
  cam.FreeLook(true)
  frames(W, 0.5, 60, function() W.lwTick(0, 0) end)
  local y0 = shown(cam)
  frames(W, 1.0, 60, function() W.lwTick(10, 6) end)
  check("LW3: lookSync + lookWrite: the camera follows the mouse (not frozen on the written look), the look = the camera",
        math.abs(CM.wrapDeg(shown(cam) - y0)) > 10 and cam.G.lwState == "on"
        and math.abs(CM.wrapDeg(W.look - shown(cam))) < 2.0, string.format("moved %.2f", CM.wrapDeg(shown(cam) - y0)))
  check("LW3: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end
-- 2026-11-04 r13: the first game look write after a vehicle hold ramps the SENT pitch from the game's current look pitch to the camera pitch (cfg.lookWriteRamp,
-- smoothstep, frame time aware). Log 13:00:19.44: the pitch went 0.2 -> -6 deg in 34 ms and the torso dropped. cam.vh.endedAt = the hold ended (as utpp_vehiclecam's
-- release end sets it); the free look press is the first write.
do
  local ramps = {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    for _, ramp in ipairs({ 0.25, 0 }) do
      local W, C, cam = lwSetup()
      cam.cfg.lookWriteRamp = ramp
      W.fppPitch = 25 -- the game's look pitch while no write runs (the exit animation's)
      cam.vh.endedAt = cam.G.clock
      local n0 = #W.gl.sets
      cam.FreeLook(true)
      local camP0 = nil
      frames(W, 0.6, fps, function() W.lwTick(0, 0) end)
      local p, prev, maxStep, firstP, endAt, camP = {}, nil, 0, nil, nil, cam.G.last.pitch
      for i = n0 + 1, #W.gl.sets do
        local v = W.gl.sets[i][2]
        p[#p + 1] = v
        if prev then maxStep = math.max(maxStep, math.abs(v - prev)) end
        prev = v
      end
      for i, v in ipairs(p) do if math.abs(v - camP) < 0.05 then endAt = i; break end end
      local r = { n = #p, first = p[1], last = p[#p], camP = camP, maxStep = maxStep, endT = endAt and endAt / fps, lines = count(W.lines, "game look write ramp") }
      ramps[fps .. "/" .. ramp] = r
      print(string.format("  look write ramp %.2f s, %d fps: %d sends, first %.2f, last %.2f (camera %.2f), largest step %.2f deg/frame, reaches the camera pitch after %.2f s",
        ramp, fps, r.n, r.first or -1, r.last or -1, camP, maxStep, r.endT or -1))
      os.remove(settingsFile)
    end
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local on, off = ramps[fps .. "/0.25"], ramps[fps .. "/0"]
    local D = math.abs(on.camP - 25)
    check(string.format("LWR %d fps: the first write after a vehicle hold starts at the game's pitch (25 -> %.2f), not at the camera pitch (first %.2f)", fps, on.camP, on.first or -1),
          on.first ~= nil and math.abs(on.first - 25) < 0.5 and on.lines == 1)
    check(string.format("LWR %d fps: the sent pitch reaches the camera pitch in 0.25 s (%.2f s) and stays", fps, on.endT or -1),
          on.endT ~= nil and on.endT >= 0.22 and on.endT <= 0.32 and math.abs(on.last - on.camP) < 0.05)
    check(string.format("LWR %d fps: the largest step per frame is %.2f deg (smoothstep: <= 1.6 x %.1f deg / 0.25 s x frame time + 0.05)", fps, on.maxStep, D),
          on.maxStep <= 1.6 * D / 0.25 / fps + 0.05)
    check(string.format("LWR %d fps: lookWriteRamp = 0 is the old write - the first send is the camera pitch (%.2f): a %.0f deg jump for the game's look", fps, off.first or -1, math.abs((off.first or 0) - 25)),
          off.first ~= nil and math.abs(off.first - off.camP) < 0.05 and off.lines == 0 and off.maxStep < 1e-6)
  end
end
-- 0.6.5b: the native divergence guard tripped (UTPPSetGameLook false, status " tripped=1 "): one TRIPPED line, state
-- tripped, no calls while waiting (also across a free look release / press), Status() state=tripped. Round 2: tried again
-- after cfg lookWriteRetry (3.5 s; the native re-arms after 3 s): still tripped with the same tripCount = silent, re-armed =
-- ON again ("re-armed after a trip"); a permanent trip (tripPermanent=1) ends the calls for the session
do
  local W, C, cam = lwSetup()
  cam.FreeLook(true)
  frames(W, 0.3, 60, function() W.lwTick(0, 0) end)
  local on0 = cam.G.lwState == "on"
  W.gl.live, W.gl.tripped = false, true
  frames(W, 0.1, 60, function() W.lwTick(0, 0) end)
  local nT = #W.gl.sets
  frames(W, 0.5, 60, function() W.lwTick(3, 2) end)
  cam.FreeLook(false)
  frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  cam.FreeLook(true)
  frames(W, 0.3, 60, function() W.lwTick(0, 0) end)
  local okB
  withEnv(W, function() okB = cam.OnBodyLookStep(4) end)
  check("LW5: native tripped: one TRIPPED line, state tripped, no more UTPPSetGameLook calls, Status state=tripped",
        on0 and count(W.lines, "game look write TRIPPED (native divergence guard)") == 1 and cam.G.lwTripped == true
        and #W.gl.sets == nT and okB == false and string.find(cam.Status(), "state=tripped", 1, true) ~= nil
        and count(W.lines, "game look write not possible") == 0,
        string.format("sets %d/%d state %s | %s", #W.gl.sets, nT, tostring(cam.G.lwState), cam.Status()))
  check("LW5: tripped: trace lw=0 lwown=0 (the camera follows the game's look again)",
        string.find(lwTrace(W), " lw=0 ", 1, true) ~= nil and string.find(lwTrace(W), " lwown=0 ", 1, true) ~= nil, lwTrace(W))
  check("LW5: the TRIPPED line says when it is tried again; Status trips=1 retry=<s>",
        count(W.lines, "-> off, tried again in 3.5 s") == 1 and string.find(cam.Status(), " trips=1 retry=", 1, true) ~= nil
        and string.find(cam.Status(), " retry=- ", 1, true) == nil, cam.Status())
  -- round 2: 3.7 s after the trip one retry went out; the native still in its cooldown (same tripCount): no new line
  frames(W, 2.6, 60, function() W.lwTick(0, 0) end)
  check("LW5 round 2: one retry after 3.5 s (the native still tripped, same trip: silent, waits again)",
        #W.gl.sets == nT + 1 and count(W.lines, "game look write TRIPPED") == 1 and cam.G.lwTripped == true,
        string.format("sets %d/%d TRIPPED %d", #W.gl.sets, nT, count(W.lines, "game look write TRIPPED")))
  W.gl.live, W.gl.tripped = true, false
  frames(W, 4.2, 60, function() W.lwTick(0, 0) end)
  check("LW5 round 2: the native re-armed: the next retry is ON again (logged 're-armed after a trip'), writes every frame, "
        .. "trace lw=1 lwown=1", cam.G.lwState == "on" and cam.G.lwTripped == false and #W.gl.sets > nT + 30
        and count(W.lines, "- re-armed after a trip") == 1 and string.find(lwTrace(W), " lw=1 ", 1, true) ~= nil
        and string.find(lwTrace(W), " lwown=1 ", 1, true) ~= nil, string.format("state %s sets %d | %s",
        tostring(cam.G.lwState), #W.gl.sets, lwTrace(W)))
  -- a permanent trip (the native's third): one line, no calls for the rest of the session
  W.gl.live, W.gl.tripped, W.gl.tripCount, W.gl.permanent = false, true, 3, true
  frames(W, 0.1, 60, function() W.lwTick(0, 0) end)
  local nPm = #W.gl.sets
  frames(W, 8.0, 60, function() W.lwTick(0, 0) end)
  check("LW5 round 2: permanent trip (tripPermanent=1): one 'off for this session' line, no calls for 8 s, retry=-",
        count(W.lines, "-> off for this session") == 1 and #W.gl.sets == nPm and cam.G.lwTripPermanent == true
        and string.find(cam.Status(), "state=tripped", 1, true) ~= nil and string.find(cam.Status(), " retry=- ", 1, true) ~= nil,
        string.format("sets %d/%d | %s", #W.gl.sets, nPm, cam.Status()))
  check("LW5: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
  local _ = C
end
-- 0.6.5b round 2: utpp_bodyturn's request counter (execStats.adjSeq) moved this frame or the last one -> the frame is an
-- adjustTransform override frame: camTrace adj=1, Status adjFrames; none without requests
do
  local W, C, cam = lwSetup()
  local bt = { exec = { why = "aim align", nativeOn = false }, execStats = { adjSeq = 0, lastStep = 0 } }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_bodyturn" then return bt end return baseGetMod(n) end
  cam.G.btMod = nil
  cam.FreeLook(true)
  frames(W, 0.35, 60, function() W.lwTick(0, 0) end)
  local trA = lwTrace(W)
  local f0 = cam.G.adjFrames or 0
  frames(W, 0.35, 60, function() bt.execStats.adjSeq = bt.execStats.adjSeq + 1; W.lwTick(0, 0) end)
  local trB = lwTrace(W)
  local f1 = cam.G.adjFrames or 0
  -- one request, then none: that frame and the next one are marked, then clean again
  frames(W, 1 / 60, 60, function() bt.execStats.adjSeq = bt.execStats.adjSeq + 1; W.lwTick(0, 0) end)
  local p1 = cam.G.adjPend
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  local p2 = cam.G.adjPend
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  local p3 = cam.G.adjPend
  frames(W, 0.35, 60, function() W.lwTick(0, 0) end)
  local trC = lwTrace(W)
  check("adj: no requests -> adj=0; a request every frame -> adj=1 (Status adjFrames counts them); one request -> that "
        .. "frame and the next marked, then clean", f0 == 0 and string.find(trA, " adj=0", 1, true) ~= nil
        and string.find(trB, " adj=1", 1, true) ~= nil and f1 >= 20 and p1 == true and p2 == true and p3 == false
        and string.find(trC, " adj=0", 1, true) ~= nil
        and string.find(cam.Status(), " adjFrames=" .. tostring(cam.G.adjFrames), 1, true) ~= nil,
        string.format("f0 %d f1 %d p %s %s %s | %s | %s", f0, f1, tostring(p1), tostring(p2), tostring(p3), trB, cam.Status()))
  check("adj: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
  local _ = C
end
-- 2026-10-13 parallax-correct game look while aiming / zooming (review finding 3): the camera sits off V's eye line, so
-- the game's look goes from the eye (FPP camera position) to the crosshair point P (first hit of the camera forward ray,
-- here a wall plane ~5 m ahead) instead of parallel to the camera forward. The read-back of the written look is our own
-- echo (the native replaces the game's mouse delta; W.lwTick sets W.fppPitch to the target): the camera pitch must not
-- follow it (no drift) and must still follow the mouse. No ray: the last P distance for aimLookHold s, then the camera
-- yaw / pitch. Also the solve's zoomOnly wiring (unarmed zoom: boom pitch = look * 0.25; weapon aim 0.35).
-- Round 3: the game's unlock turns V onto the look (lwWorld unlockTurn) - the release flush must keep the correction out
-- of that turn and out of the camera pitch; the AimOffset target (UTPPSetAimTarget, recorded in W.aimT) follows the same
-- corrected direction from the eye.
do
  local W = lwWorld({ unlockTurn = true })
  W.aimT = {}
  W.env.Game.UTPPSetAimTarget = function(pos, fwd, on) W.aimT[#W.aimT + 1] = { pos = pos, fwd = fwd, on = on } end
  local C = loadCore(W)
  local persp = { -- weapon aim in third person needs perspective.aiming = tpp (J)
    Desired = function() return { tpp = true, perspective = "TPP", style = "blend", reason = "user", mode = "active" } end,
    Mode = function() return "active" end, ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" },
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  W.records["Items.Preset_Lexington_Default"] = "Wea_Handgun"
  local pistol = {}
  function pistol:GetItemID() return { tdb = "Items.Preset_Lexington_Default" } end
  function pistol:GetEntityID() return { hash = "ent_pistol" } end
  local cam = loadCamera(W)
  -- CM.AIM_POINT ships disabled (review 2026-10-30): on for the crosshair point checks, then back to the shipped default
  local apShipped = cam.CM.AIM_POINT.enabled
  cam.CM.AIM_POINT.enabled = true
  withEnv(W, function() W.handlers.onInit() end)
  withEnv(W, function() C.Set("camera.idleOrbit", false) end)
  frames(W, 3.0, 60, function() W.lwTick(0, 0) end)
  check("LW4: third person on the world camera (utpp_perspective decision)", cam.T.state == "tpp" and cam.G.use == "world",
        tostring(cam.T.state) .. " " .. tostring(cam.G.use))
  local function lastSet() return W.gl.sets[#W.gl.sets] or {} end
  -- the target the look must have: eye -> P (P = camera forward ray on the wall plane y = W.wallY)
  local function expected()
    local r = cam.G.last
    local fwd = CM.forward(r.yaw, r.pitch)
    local t = (W.wallY - r.world.y) / fwd.y
    local y, p = CM.anglesEyeToPoint(W.eye(), CM.add(r.world, CM.scale(fwd, t)))
    return y, p, t, r
  end
  -- weapon aim (pistol, RMB): heading locked, the game's look written every frame
  W.weapon, W.weaponItem = true, pistol
  W.psm.UpperBody = 6
  frames(W, 0.5, 60, function() W.lwTick(0, 0) end)
  local r0 = cam.G.last
  check("LW4: weapon aim on the world camera, game look written (heading locked)", cam.aim.on and cam.G.lwState == "on"
        and W.fpp.headingLocked == true and r0 ~= nil, string.format("aim %s lw %s", tostring(cam.aim.on), tostring(cam.G.lwState)))
  check("LW4: no wall -> P at camera.aim.range (60 m): lwp=60.00", cam.G.lwp ~= nil and math.abs(cam.G.lwp - 60) < 1e-6
        and string.find(lwTrace(W), " lwp=60.00", 1, true) ~= nil, lwTrace(W))
  -- a wall 5 m ahead of the camera along its forward
  local f0 = CM.forward(r0.yaw, r0.pitch)
  W.wallY = r0.world.y + 5 * f0.y
  local pitch0 = r0.pitch
  frames(W, 1.0, 60, function() W.lwTick(0, 0) end)
  local ey, ep, et, r = expected()
  local s = lastSet()
  check("LW4: aiming, ray hit at 5 m: the game's look = eye -> P (not parallel to the camera forward), lwp=5.00",
        s[1] ~= nil and math.abs(CM.wrapDeg(s[1] - ey)) < 1e-6 and math.abs(s[2] - ep) < 1e-6
        and math.abs(et - 5) < 1e-3 and math.abs(cam.G.lwp - et) < 1e-6
        and math.abs(CM.wrapDeg(s[1] - r.yaw)) > 1 and math.abs(s[2] - r.pitch) > 0.3
        and string.find(lwTrace(W), " lwp=5.00", 1, true) ~= nil,
        string.format("written %.3f/%.3f expected %.3f/%.3f camera %.3f/%.3f P %.3f m | %s", s[1] or 0, s[2] or 0, ey, ep,
        r.yaw, r.pitch, et, lwTrace(W)))
  local at = W.aimT[#W.aimT] or {}
  -- 2026-10-30 aim point feed: the native target point Q = pos + fwd * 60 m is the crosshair point P itself (the wall
  -- hit 5 m along the camera ray; filtered in 1/e, settled to < 2 mm after 1 s), sent along the camera forward
  local P5 = CM.add(r.world, CM.scale(CM.forward(r.yaw, r.pitch), et))
  local Q5 = at.pos and CM.add(at.pos, CM.scale(at.fwd, 60)) or nil
  check("LW4: AimOffset target point = the crosshair point P (wall 5 m along the camera ray), ap=5.00 in the trace",
        at.on == true and Q5 ~= nil and CM.dist(Q5, P5) < 2e-3 and CM.dist(at.fwd, CM.forward(r.yaw, r.pitch)) < 1e-9
        and string.find(lwTrace(W), " ap=5.00", 1, true) ~= nil,
        Q5 and string.format("Q %.4f,%.4f,%.4f P %.4f,%.4f,%.4f | %s", Q5.x, Q5.y, Q5.z, P5.x, P5.y, P5.z, lwTrace(W))
        or "no call")
  -- the aim point feed off (CM.AIM_POINT.enabled = false): exactly the target before 2026-10-30 - 60 m along the written
  -- (corrected) look from the eye, not along the camera ray
  cam.CM.AIM_POINT.enabled = false
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  -- the rest of LW4 runs on the shipped default (off); the P glide-out is covered by test_utpp_aim_point.lua
  cam.CM.AIM_POINT.enabled = apShipped
  do
    local s1, r1 = lastSet(), cam.G.last
    local at1 = W.aimT[#W.aimT] or {}
    check("LW4: aim point feed off -> AimOffset target = 60 m along the written (corrected) look from the eye (as before)",
          at1.on == true and at1.pos ~= nil and CM.dist(at1.pos, W.eye()) == 0
          and CM.dist(at1.fwd, CM.forward(s1[1], s1[2])) < 1e-9 and CM.dist(at1.fwd, CM.forward(r1.yaw, r1.pitch)) > 0.01,
          at1.pos and string.format("pos %.3f,%.3f,%.3f eye %.3f,%.3f,%.3f", at1.pos.x, at1.pos.y, at1.pos.z, W.eye().x,
          W.eye().y, W.eye().z) or "no call")
  end
  frames(W, 1.0, 60, function() W.lwTick(0, 0) end)
  check("LW4: the camera pitch does not follow the corrected written look (echo, no drift over 1 s)",
        math.abs(r.pitch - pitch0) < 1e-6 and math.abs(W.fppPitch - r.pitch) > 0.3,
        string.format("camera %.4f -> %.4f, game look %.4f", pitch0, r.pitch, W.fppPitch))
  -- 0.6.5b (user 01:16: the aim jerked with the game's look): the game's look read back jumping every frame (pitch
  -- +-25 deg, yaw +9 / -4 deg after the hooked update) moves neither the camera pitch nor its yaw while it is written
  do
    local pj, yj = cam.G.last.pitch, cam.G.last.yaw
    local maxDP, maxDY = 0, 0
    frames(W, 0.5, 60, function(i)
      W.lwTick(0, 0)
      W.fppPitch = W.fppPitch + ((i % 2 == 0) and 25 or -25)
      W.look = CM.wrapDeg(W.look + ((i % 3 == 0) and 9 or -4))
      if i > 1 then
        maxDP = math.max(maxDP, math.abs(cam.G.last.pitch - pj))
        maxDY = math.max(maxDY, math.abs(CM.wrapDeg(cam.G.last.yaw - yj)))
      end
    end)
    check("LW4: the game's look pitch +-25 / yaw jitter while written: camera pitch / yaw unchanged, trace lwown=1",
          maxDP < 1e-6 and maxDY < 1e-6 and string.find(lwTrace(W), " lwown=1 ", 1, true) ~= nil,
          string.format("maxDP %.6f maxDY %.6f | %s", maxDP, maxDY, lwTrace(W)))
    frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  end
  -- the mouse moves the aim pitch while the look is written (the game's own mouse delta is replaced by the native)
  frames(W, 0.3, 60, function()
    W.lwTick(0, 0)
    W.observers["PlayerPuppet.OnAction"](nil, { name = "CameraMouseY", value = -3 })
  end)
  local pm = cam.G.last.pitch
  frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  ey, ep, et, r = expected()
  s = lastSet()
  check("LW4: mouse Y while aiming with the look write: the camera pitch moves (not frozen on the echo), then holds",
        math.abs(pm - pitch0) > 3 and math.abs(r.pitch - pm) < 1e-6 and math.abs(s[2] - ep) < 1e-6,
        string.format("%.3f -> %.3f -> %.3f (written %.3f expected %.3f)", pitch0, pm, r.pitch, s[2] or 0, ep))
  check("LW4: weapon aim boom = look * 0.35 (weapon scale, no framing limit by default)",
        math.abs(r.boomPitch - r.pitch * 0.35) < 1e-6, string.format("%.4f / %.4f", r.boomPitch, r.pitch))
  -- no ray this frame: the last P distance reused (aimLookHold 0.5 s), then the camera yaw / pitch
  local baseSqs = W.env.Game.GetSpatialQueriesSystem
  W.env.Game.GetSpatialQueriesSystem = function() return nil end
  frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
  s = lastSet()
  r = cam.G.last
  check("LW4: ray unavailable for a frame -> the last P distance reused (eye -> P)", cam.G.lwp ~= nil
        and math.abs(cam.G.lwp - et) < 1e-6 and math.abs(CM.wrapDeg(s[1] - ey)) < 1e-6 and math.abs(s[2] - ep) < 1e-6,
        string.format("lwp %s written %.3f/%.3f", tostring(cam.G.lwp), s[1] or 0, s[2] or 0))
  frames(W, 0.6, 60, function() W.lwTick(0, 0) end)
  s = lastSet()
  r = cam.G.last
  check("LW4: ray unavailable > 0.5 s -> falls back to the camera yaw / pitch, lwp=-",
        cam.G.lwp == nil and math.abs(CM.wrapDeg(s[1] - r.yaw)) < 1e-9 and math.abs(s[2] - r.pitch) < 1e-9
        and string.find(lwTrace(W), " lwp=-", 1, true) ~= nil and math.abs(r.pitch - pm) < 1e-6,
        string.format("written %.3f/%.3f camera %.3f/%.3f | %s", s[1] or 0, s[2] or 0, r.yaw, r.pitch, lwTrace(W)))
  at = W.aimT[#W.aimT] or {}
  check("LW4: no correction written -> AimOffset target along the camera ray from the camera (as before)",
        at.on == true and at.pos ~= nil and CM.dist(at.pos, r.world) < 1e-9 and CM.dist(at.fwd, CM.forward(r.yaw, r.pitch)) < 1e-9)
  W.env.Game.GetSpatialQueriesSystem = baseSqs
  frames(W, 0.2, 60, function() W.lwTick(0, 0) end)
  -- round 3 release probe (UpperBody 0, 12 single frames): largest camera pitch step per frame, frames the lock was held
  -- past the release, game look writes after it (flush = the first), yaw V was turned by the game's unlock; r1 / r2 =
  -- the solve of frame 1 / 2, w2 = the zoom weight after frame 2
  local function release()
    local o = { p0 = cam.G.last.pitch, g0 = W.fppPitch, look0 = W.look, yaw0 = W.yaw, sets0 = #W.gl.sets,
                turns0 = W.unlockTurns, unl0 = W.unlocks, maxDP = 0, held = 0 }
    W.psm.UpperBody = 0
    local prevP = o.p0
    for i = 1, 12 do
      frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
      local rr = cam.G.last
      if i == 1 then o.r1 = rr end
      if i == 2 then o.r2, o.w2 = rr, cam.zoomU.w end
      o.maxDP = math.max(o.maxDP, math.abs(rr.pitch - prevP))
      prevP = rr.pitch
      if W.fpp.headingLocked then o.held = o.held + 1 end
    end
    o.sets, o.flush = #W.gl.sets - o.sets0, W.gl.sets[o.sets0 + 1] or {}
    o.turn, o.unlocks = W.unlockTurns - o.turns0, W.unlocks - o.unl0
    o.txt = string.format("held %d sets %d flush %s/%s (camera %.4f/%.4f) unlocks %d turn %.4f (correction %.3f) pitch %.3f "
      .. "(game %.3f) max step %.4f", o.held, o.sets, tostring(o.flush[1]), tostring(o.flush[2]), o.r1.yaw, o.r1.pitch,
      o.unlocks, o.turn, CM.wrapDeg(o.look0 - o.yaw0), o.p0, o.g0, o.maxDP)
    return o
  end
  -- aim released (review: the game's unlock turned V by the correction, the camera pitch jumped onto it a frame later):
  -- the lock is held one frame writing the plain camera yaw / pitch (read back matched the next frame), then unlocked
  local rel = release()
  check("LW4: aim released: lock held 1 frame writing the plain camera yaw / pitch (release flush), then unlocked, stopped",
        not cam.aim.on and rel.held == 1 and rel.sets == 1 and rel.flush[3] == true and rel.flush[1] == rel.r1.yaw
        and rel.flush[2] == rel.r1.pitch and rel.unlocks == 1 and W.gl.target == nil and W.fpp.headingLocked == false, rel.txt)
  check("LW4: aim released: the game's unlock turns V by < 0.01 deg (the correction written while aiming was > 1 deg)",
        rel.turn < 0.01 and math.abs(CM.wrapDeg(rel.look0 - rel.yaw0)) > 1, rel.txt)
  frames(W, 0.5, 60, function() W.lwTick(0, 0) end)
  check("LW4: aim released: no camera pitch jump (< 0.5 deg per frame over 12 frames; game pitch was > 0.3 deg off), then = game",
        math.abs(rel.g0 - rel.p0) > 0.3 and rel.maxDP < 0.5 and math.abs(cam.G.last.pitch - W.fppPitch) < 0.5, rel.txt)
  -- empty hands, RMB again: the unarmed zoom - the same parallax target, the zoom's own boom (0.25)
  W.weapon, W.weaponItem = false, nil
  W.psm.UpperBody = 6
  frames(W, 0.6, 60, function() W.lwTick(0, 0) end)
  ey, ep, et, r = expected()
  s = lastSet()
  check("LW4: unarmed zoom: zoom boom (zoomShare 1) = look * 0.25, the game's look = eye -> P",
        cam.zoomU.on and not cam.aim.on and math.abs(r.boomPitch - r.pitch * 0.25) < 1e-6 and math.abs(r.pitch) > 1
        and math.abs(CM.wrapDeg(s[1] - ey)) < 1e-6 and math.abs(s[2] - ep) < 1e-6 and math.abs(cam.G.lwp - et) < 1e-6,
        string.format("zoom %s boom %.4f pitch %.4f written %.3f/%.3f expected %.3f/%.3f", tostring(cam.zoomU.on),
        r.boomPitch, r.pitch, s[1] or 0, s[2] or 0, ey, ep))
  -- zoom exit tween: the pose shown (A.w 0: zoomShare 1) keeps the zoom scale, blended by the weight
  local relZ = release()
  r = relZ.r2
  local zw = relZ.w2
  check("LW4: zoom exit tween: boom = look * (1 + (0.25 - 1) * w) (zoomShare = ZU.w / (A.w + ZU.w) = 1)",
        not cam.zoomU.on and zw > 0 and zw < 1 and math.abs(r.boomPitch - r.pitch * (1 - 0.75 * zw)) < 1e-6,
        string.format("w %.3f boom %.4f pitch %.4f", zw, r.boomPitch, r.pitch))
  check("LW4: unarmed zoom released: release flush (1 frame, plain camera), unlock turn < 0.01 deg (correction > 1 deg), "
        .. "no camera pitch jump (< 0.5 deg per frame)", relZ.held == 1 and relZ.sets == 1 and relZ.flush[1] == relZ.r1.yaw
        and relZ.flush[2] == relZ.r1.pitch and relZ.unlocks == 1 and relZ.turn < 0.01
        and math.abs(CM.wrapDeg(relZ.look0 - relZ.yaw0)) > 1 and relZ.maxDP < 0.5, relZ.txt)
  frames(W, 1.0, 60, function() W.lwTick(0, 0) end)
  -- round 3 (review: zoomOnly from the on flags switched the boom rules in one frame): the weapon leaves the hand while
  -- UpperBody stays 6 - the aim exits and the unarmed zoom enters on the same frame (ZU held counts UpperBody 6 with the
  -- weapon too); while both tweens overlap the boom lag is the blend by zoomShare = ZU.w / (A.w + ZU.w)
  W.weapon, W.weaponItem = true, pistol
  W.psm.UpperBody = 6
  frames(W, 0.5, 60, function() W.lwTick(0, 0) end)
  local aimOn0 = cam.aim.on
  W.weapon, W.weaponItem = false, nil
  local nOver, badOv, zoomFrame = 0, {}, nil
  for i = 1, 20 do
    frames(W, 1 / 60, 60, function() W.lwTick(0, 0) end)
    local rr, aW, zW = cam.G.last, cam.aim.w, cam.zoomU.w
    if cam.zoomU.on and not zoomFrame then zoomFrame = i end
    local aw = math.max(aW, zW)
    if aw > 0 then
      local sh = zW / math.max(aW + zW, 1e-6)
      local want = rr.pitch - aw * rr.pitch * ((1 - 0.35) * (1 - sh) + (1 - 0.25) * sh)
      if aW > 0 and zW > 0 then nOver = nOver + 1 end
      if math.abs(rr.boomPitch - want) > 1e-9 then
        badOv[#badOv + 1] = string.format("f%d A.w %.3f ZU.w %.3f boom %.5f want %.5f", i, aW, zW, rr.boomPitch, want)
      end
    end
  end
  check("LW4: weapon dropped with UpperBody 6: aim exit + zoom enter on the same frame, the boom blended by the weights "
        .. "while both tweens overlap", aimOn0 == true and zoomFrame == 1 and nOver >= 5 and #badOv == 0
        and math.abs(cam.G.last.pitch) > 1,
        string.format("aim %s zoom frame %s overlap frames %d pitch %.3f %s", tostring(aimOn0), tostring(zoomFrame), nOver,
        cam.G.last.pitch, table.concat(badOv, "; ")))
  W.psm.UpperBody = 0
  frames(W, 1.0, 60, function() W.lwTick(0, 0) end)
  check("LW4: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

-- ================================================================ R) 2026-10-21 heading lock re-assert
-- (log 03:37: the PSM applies a camera parameter set with headingLocked = false at the change out of a slide / jump / vault; the
-- cached lock was never read back: the native game look found it unlocked in 78 % of its ticks). While the lock is wanted and
-- cached the camera reads the component every frame and sets it again (camera.lockReassert), except where the game's own set
-- stays (vehicle / ladder / climb / workspot / scene / knockdown / carrying / device), in menu / photo mode, and within the rate
-- limit (8 writes per 1 s, then a 2 s pause).
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  -- utpp_perspective decides third person (the PSM states below are not cut to first person by the camera's own block rules)
  local persp = {
    Desired = function() return { tpp = true, perspective = "TPP", style = "blend", reason = "user", mode = "active" } end,
    Mode = function() return "active" end,
    ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" },
  }
  local baseGetMod = W.env.GetMod
  W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  local cam = loadCamera(W)
  -- the native of utpp_bodyturn.reds: LookStatus "lookOn=.. lookSet=.. gameSet=<the last set the PSM queued> lookSends=.."
  W.env.Game.UTPPLookWorldStatus = function()
    return "lookOn=false lookSet=VehicleCombat gameSet=" .. (W.gameSet or "Default") .. " lookSends=0 lookSwaps=0 lookRestores=0"
  end
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 3.0)
  check("R: third person by the decision", cam.T.state == "tpp" and W.active == "utpp")
  -- moving (no idle orbit after 3 s standing: that would want the lock by itself): follow mode, heading not locked
  W.speed = 2
  frames(W, 2.0)
  check("R: moving in follow mode: heading not locked", cam.G.lockSet == false and not cam.S.look.free, tostring(cam.G.lockSet))
  -- the component's headingLocked as a recorded field: writes are listed with the module clock, the game's own set apply is
  -- simulated by storing false from outside (no write recorded)
  local store, writes = W.fpp.headingLocked, {}
  W.fpp.headingLocked = nil
  setmetatable(W.fpp, {
    __index = function(_, k) if k == "headingLocked" then return store end end,
    __newindex = function(t, k, v)
      if k == "headingLocked" then writes[#writes + 1] = { cam.G.clock, v }; store = v else rawset(t, k, v) end
    end })
  local function gameUnlock() store = false end
  local function nTrue() local n = 0 for _, w in ipairs(writes) do if w[2] == true then n = n + 1 end end return n end
  local function status() local s; withEnv(W, function() s = cam.LookWriteStatus() end); return s end
  -- free look wanted (hold): the lock is set by the edge
  cam.FreeLook(true)
  frames(W, 0.5, 60, function() action(W, "CameraMouseX", 10) end)
  check("R: free look: the lock is set once by the edge, nothing to re-assert", store == true and nTrue() == 1 and (cam.stats.lockRelocks or 0) == 0,
        string.format("store %s writes %d relocks %s", tostring(store), nTrue(), tostring(cam.stats.lockRelocks)))
  frames(W, 0.5)
  check("R: locked: no read-back writes while the game leaves it set", nTrue() == 1 and (cam.stats.lockRelocks or 0) == 0, nTrue())
  -- the game unlocks (set apply at a state change): set again in the very next frame, counted, logged, calibration held
  gameUnlock()
  local t0 = cam.G.clock
  frames(W, 1 / 60, 60)
  check("R: the game's unlock (Default set): locked again in the next frame, counted (relocks=1) and logged",
        store == true and nTrue() == 2 and cam.stats.lockRelocks == 1 and math.abs(cam.G.unlockAt - cam.G.clock) < 1e-9
        and count(W.lines, "heading lock lost (the game applied a camera parameter set: PSM Locomotion 0, LocomotionDetailed 1) -> set again (#1, stance stand)") == 1
        and string.find(status(), "relocks=1 skips=0", 1, true) ~= nil,
        string.format("store %s writes %d relocks %s | %s", tostring(store), nTrue(), tostring(cam.stats.lockRelocks), status()))
  frames(W, 0.3)
  check("R: and it stays locked afterwards (one write only)", store == true and nTrue() == 2 and cam.stats.lockRelocks == 1)
  -- states of plain locomotion: every unlock is set again (stand / walk toggle / sprint / slide / jump / crouch sprint; the vault 9
  -- and the hover jump 21 keep the game's set: scope cases below)
  for _, det in ipairs({ 2, 3, 4, 5, 18, 30 }) do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned = nil, nil, nil
    W.psm.LocomotionDetailed = det
    local n0 = cam.stats.lockRelocks
    gameUnlock()
    frames(W, 3 / 60, 60)
    check(string.format("R: plain locomotion (LocomotionDetailed %d): locked again", det), store == true and cam.stats.lockRelocks == n0 + 1,
          string.format("store %s relocks %d -> %d", tostring(store), n0, cam.stats.lockRelocks))
  end
  W.psm.LocomotionDetailed = 1
  -- rate limit: the game unlocks every frame -> at most 8 writes per second, then a 2 s pause with ONE log line, then it resumes
  cam.G.relock = nil
  local nW0, tStart = nTrue(), cam.G.clock
  frames(W, 3.2, 60, function() gameUnlock() end)
  local stamps = {}
  for i = 1, #writes do if writes[i][2] == true and writes[i][1] > tStart then stamps[#stamps + 1] = writes[i][1] end end
  local maxWin = 0
  for i = 1, #stamps do
    local n = 0
    for j = i, #stamps do if stamps[j] - stamps[i] < 1.0 then n = n + 1 end end
    maxWin = math.max(maxWin, n)
  end
  local inPause, resumed = 0, 0
  for _, t in ipairs(stamps) do
    if t > tStart + 0.2 and t < tStart + 2.0 then inPause = inPause + 1 end
    if t > tStart + 2.2 then resumed = resumed + 1 end
  end
  check("R: unlocked every frame: at most 8 writes in any 1 s, none during the pause, writes resume after it",
        maxWin <= 8 and #stamps >= 9 and inPause == 0 and resumed >= 1,
        string.format("writes %d, max in 1 s %d, in pause %d, after %d", #stamps, maxWin, inPause, resumed))
  check("R: the pause is logged once per pause (not per frame)",
        count(W.lines, "heading lock lost 8 times within 1.0 s -> not set again for 2.0 s") == 1
        or count(W.lines, "heading lock lost 8 times within 1.0 s -> not set again for 2.0 s") == 2,
        count(W.lines, "heading lock lost 8 times within 1.0 s"))
  frames(W, 5.0) -- (the second pause is 4 s)
  check("R: after the game stops resetting it, the lock is back", store == true)
  -- backoff (review 2026-10-21): a state that never lets go - pauses 2 / 4 / 8 / 16 / 30 s, 8 writes per pause, three logged pauses
  -- and one "keeps being reset" line (was: 8 writes and a log line every 2 s for as long as the state lasts)
  local function pauseLines(from)
    local t = {}
    for i = from + 1, #W.lines do
      local s = string.match(W.lines[i], "heading lock lost %d+ times within [%d%.]+ s %-> not set again for ([%d%.]+) s")
      if s then t[#t + 1] = tonumber(s) end
    end
    return t
  end
  do
    cam.G.relock = nil
    local l0, w0 = #W.lines, nTrue()
    frames(W, 70, 60, function() gameUnlock() end)
    local pauses, writesN = pauseLines(l0), nTrue() - w0
    check("R: a state that resets the lock all the time (70 s): pauses 2 / 4 / 8 s logged, then one 'keeps being reset' line, 24..56 writes (was ~260)",
          #pauses == 3 and pauses[1] == 2 and pauses[2] == 4 and pauses[3] == 8 and count(W.lines, "heading lock keeps being reset") == 1
          and writesN >= 24 and writesN <= 56, string.format("pauses %s writes %d", table.concat(pauses, "/"), writesN))
    -- the pause is capped (30 s): 6 pauses in a row by now, the running one is at most 30 s long (uncapped it would be 64 s)
    local R = cam.G.relock
    check("R: the pause stops growing at lockReassertPauseMax (30 s)", R.pauses >= 6 and R.paused ~= nil and R.paused - cam.G.clock <= 30 + 1e-6,
          string.format("pauses %d remaining %.1f s", R.pauses, R.paused and (R.paused - cam.G.clock) or -1))
  end
  -- a new PSM state ends the pause and the backoff at once: sprint during the 8 s pause -> written within 2 frames, the next pause is 2 s again
  do
    cam.G.relock = nil
    local l0 = #W.lines
    frames(W, 7, 60, function() gameUnlock() end)
    local nPaused = nTrue()
    W.psm.LocomotionDetailed = 4
    frames(W, 2 / 60, 60, function() gameUnlock() end)
    local resumed = nTrue() > nPaused
    frames(W, 1, 60, function() gameUnlock() end)
    local p = pauseLines(l0)
    W.psm.LocomotionDetailed = 1
    check("R: a changed PSM state ends the pause and the backoff: written within 2 frames, the next pause is the short one (2 s) again",
          resumed and #p == 4 and p[3] == 8 and p[4] == 2, string.format("resumed %s pauses %s", tostring(resumed), table.concat(p, "/")))
  end
  -- a loss again after 6 s (< lockReassertStable 10 s) goes on with the backoff (8 s), it is not a new beginning
  do
    frames(W, 40) -- the game lets go; the pause ends, the lock is back
    cam.G.relock = nil
    local l0 = #W.lines
    frames(W, 3, 60, function() gameUnlock() end)   -- pauses 2, 4
    frames(W, 12, 60)                                -- the pause (till ~6.3 s) ends, the lock is written, then quiet
    frames(W, 1, 60, function() gameUnlock() end)
    local p = pauseLines(l0)
    check("R: a loss again after about 6 s (< 10 s) continues the backoff: pauses 2 / 4, then 8 s", #p == 3 and p[1] == 2 and p[2] == 4 and p[3] == 8,
          table.concat(p, "/"))
  end
  -- 10 s without a lost lock: the backoff starts again (2 s, not 8 s)
  do
    frames(W, 20) -- the game lets go; the pause ends, the lock is back
    cam.G.relock = nil
    local l0 = #W.lines
    frames(W, 3, 60, function() gameUnlock() end)   -- pauses 2, 4
    frames(W, 5, 60)                                  -- the game lets go, the pause ends, the lock is written
    frames(W, 11, 60)                                 -- 11 s without a lost lock
    frames(W, 1, 60, function() gameUnlock() end)
    local p = pauseLines(l0)
    check("R: 11 s without a lost lock: the backoff starts again (pauses 2 / 4, then 2 s again, not 8 s)",
          #p == 3 and p[1] == 2 and p[2] == 4 and p[3] == 2, table.concat(p, "/"))
    frames(W, 5.0)
    check("R: and the lock is back afterwards", store == true)
  end
  -- negative controls: camera.lockReassert off / menu / photo mode -> no write at all (menu / photo: no solve result, the
  -- lock call site is not reached while frozen)
  cam.G.relock = nil
  cam.cfg.lockReassert = false
  local nOff, rOff = nTrue(), cam.stats.lockRelocks
  gameUnlock()
  frames(W, 0.4)
  check("R: negative control camera.lockReassert = false: the game's unlock is left alone (old behaviour)",
        store == false and nTrue() == nOff and cam.stats.lockRelocks == rOff, string.format("store %s", tostring(store)))
  cam.cfg.lockReassert = true
  frames(W, 3 / 60, 60)
  check("R: lockReassert on again: the same unlock is set again", store == true and nTrue() == nOff + 1 and cam.stats.lockRelocks == rOff + 1)
  for _, mode in ipairs({ "menu", "photo" }) do
    cam.G.relock = nil
    local nM, rM = nTrue(), cam.stats.lockRelocks
    W[mode] = true
    gameUnlock()
    frames(W, 0.4)
    local quiet = store == false and nTrue() == nM and cam.stats.lockRelocks == rM
    W[mode] = false
    frames(W, 3 / 60, 60)
    check(string.format("R: negative control %s: no read-back write while it is open, set again right after", mode),
          quiet and store == true and nTrue() == nM + 1, string.format("quiet %s store %s", tostring(quiet), tostring(store)))
  end
  -- a lock that is not wanted is not forced: the released free look leaves the game's unlocked state alone, unreadable = no write
  cam.FreeLook(false)
  frames(W, 2.0)
  local nFree, rFree = nTrue(), cam.stats.lockRelocks
  check("R: free look released: heading unlocked by the camera itself", store == false and cam.G.lockSet == false)
  gameUnlock()
  frames(W, 0.4)
  check("R: lock not wanted: nothing is re-asserted", store == false and nTrue() == nFree and cam.stats.lockRelocks == rFree)
  cam.FreeLook(true)
  frames(W, 0.5, 60, function() action(W, "CameraMouseX", 10) end)
  check("R: free look again: lock set by the edge", store == true and cam.G.lockSet == true)
  local nU, rU = nTrue(), cam.stats.lockRelocks
  store = nil -- an older build without the property: unreadable
  frames(W, 0.3)
  check("R: headingLocked unreadable (nil): no write", store == nil and nTrue() == nU and cam.stats.lockRelocks == rU)
  store = true
  -- the game's own set stays: no read-back write in vehicles, on ladders, climbs, workspots, scenes, knockdowns, carrying
  local cases = {
    { "vehicle", function(on) W.psm.Vehicle = on and 1 or 0 end },
    { "workspot", function(on) W.psm.IsInWorkspot = on and 1 or 0 end },
    { "scene", function(on) W.psm.HighLevel = on and 3 or 0 end },
    { "climb", function(on) W.psm.LocomotionDetailed = on and 8 or 1 end },
    { "ladder", function(on) W.psm.LocomotionDetailed = on and 10 or 1 end },
    { "ladder", function(on) W.psm.LocomotionDetailed = on and 13 or 1 end },
    { "knockdown", function(on) W.psm.LocomotionDetailed = on and 29 or 1 end },
    { "felled", function(on) W.psm.LocomotionDetailed = on and 31 or 1 end },
    { "carrying", function(on) W.psm.BodyCarrying = on and 1 or 0 end },
    { "carrying", function(on) W.psm.Carrying = on or nil end },
    { "device", function(on) W.psm.IsControllingDevice = on or nil end },
    -- (2026-10-21 review) the states utpp.reds hands back to the game, the vault and the hover jump, any other queued set
    { "vault", function(on) W.psm.LocomotionDetailed = on and 9 or 1 end },
    { "hoverJump", function(on) W.psm.LocomotionDetailed = on and 21 or 1 end },
    { "takedown", function(on) W.psm.Takedown = on and 1 or 0 end },
    { "swimming", function(on) W.psm.HighLevel = on and 6 or 0 end }, -- (first: the reason is logged once, from this case)
    { "swimming", function(on) W.psm.Swimming = on and 1 or 0 end },
    { "vehicle", function(on) W.psm.MountedToVehicle = on or nil end },
    { "scene", function(on) W.psm.HighLevel = on and 9 or 0 end }, -- an unknown HighLevel is a scene (utpp.reds)
    { "game set Consumable", function(on) W.gameSet = on and "Consumable" or nil end },
    { "game set Vault", function(on) W.gameSet = on and "Vault" or nil end },
  }
  for _, c in ipairs(cases) do
    local why, set = c[1], c[2]
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned = nil, nil, nil
    local nC, rC, sC = nTrue(), cam.stats.lockRelocks, cam.stats.lockSkips or 0
    set(true)
    gameUnlock()
    frames(W, 0.3)
    local held = store == false and nTrue() == nC and cam.stats.lockRelocks == rC and (cam.stats.lockSkips or 0) > sC
    local named = count(W.lines, "heading lock lost while " .. why .. " (PSM") >= 1 -- logged under its own reason (once per reason)
    set(false)
    frames(W, 8 / 60, 60) -- past the lockReassertGrace (0.05 s)
    check(string.format("R: scope %s: the game's unlock is left alone there, set again once plain locomotion is back", why),
          held and named and store == true and nTrue() == nC + 1, string.format("held %s store %s writes %d -> %d skips %d -> %d", tostring(held),
          tostring(store), nC, nTrue(), sC, cam.stats.lockSkips or 0))
  end
  -- the PSM blackboard unreadable: the camera cannot tell whether the game owns the set -> no write, logged once
  do
    local realBbs = W.env.Game.GetBlackboardSystem
    W.env.Game.GetBlackboardSystem = function()
      local real = realBbs()
      return { GetLocalInstanced = function() error("no PSM blackboard") end, Get = real.Get }
    end
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned = nil, nil, nil
    local nC, sC = nTrue(), cam.stats.lockSkips or 0
    gameUnlock()
    frames(W, 0.3)
    local held = store == false and nTrue() == nC and (cam.stats.lockSkips or 0) > sC
    W.env.Game.GetBlackboardSystem = realBbs
    frames(W, 8 / 60, 60)
    check("R: PSM blackboard unreadable: the game's unlock is left alone, logged once, set again when readable",
          held and store == true and nTrue() == nC + 1
          and count(W.lines, "heading lock lost while PSM unreadable (PSM Locomotion 0, LocomotionDetailed 0)") == 1,
          string.format("held %s store %s", tostring(held), tostring(store)))
  end
  -- grace: the vault ends, the queued Default set is applied a little later - no write in the first frame after it, set again soon
  do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned = nil, nil, nil
    W.psm.LocomotionDetailed = 9
    gameUnlock()
    frames(W, 0.2)
    local nG = nTrue()
    W.psm.LocomotionDetailed = 1
    frames(W, 2 / 60, 60)
    local quiet = store == false and nTrue() == nG
    frames(W, 8 / 60, 60)
    check("R: grace: the first two frames (0.03 s) after the vault ends write nothing, the lock is set again within 0.17 s",
          quiet and store == true and nTrue() == nG + 1, string.format("quiet %s store %s", tostring(quiet), tostring(store)))
  end
  -- (review 2026-10-21) the vanilla PSM queues no camera set when a consumable ends: "UseConsumable" stays in gameSet= - it inherits
  -- Default's unlocked heading, the lock is set again as in plain locomotion
  do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName = nil, nil, nil, nil
    W.gameSet = "UseConsumable"
    local nU, sU = nTrue(), cam.stats.lockSkips or 0
    gameUnlock()
    frames(W, 8 / 60, 60)
    check("R: the stale set UseConsumable (the consumable is over, no new set queued): the lock is set again at once, nothing skipped",
          store == true and nTrue() == nU + 1 and (cam.stats.lockSkips or 0) == sU, string.format("store %s writes %d skips %d", tostring(store), nTrue() - nU, (cam.stats.lockSkips or 0) - sU))
    W.gameSet = nil
  end
  -- (review 2026-10-21) the Tier2 staged area (SpaceShuttleInterior status effect): the PSM queues SpaceShuttleInterior : Default at every
  -- locomotion state enter instead of Default - the name stays for the whole area, the lock must be set again there for as long as it
  -- lasts (not only 3 s), every time the game drops it
  do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName = nil, nil, nil, nil
    W.gameSet = "SpaceShuttleInterior"
    local nSh, sSh, ok = nTrue(), cam.stats.lockSkips or 0, true
    for _ = 1, 3 do
      gameUnlock()
      frames(W, 6 / 60, 60)
      if store ~= true then ok = false end
      frames(W, 4.0, 60) -- 4 s apart: past the 3 s of an unlisted set
    end
    check("R: the whole area with SpaceShuttleInterior (set derived from Default): the lock is set again every time, also after 3 s, nothing skipped",
          ok and nTrue() == nSh + 3 and (cam.stats.lockSkips or 0) == sSh, string.format("ok %s writes %d skips %d", tostring(ok), nTrue() - nSh,
          (cam.stats.lockSkips or 0) - sSh))
    W.gameSet = nil
  end
  -- (review 2026-10-21) the game look sync ON: utpp_bodyturn.reds leaves UseConsumable / SpaceShuttleInterior to the game (they are
  -- not swapped for the sync set), so they are not "own" sets there: skipped like any unlisted set, and when the lock is restored the
  -- sync set is sent again - a bare headingLocked = true would leave the world heading off and the sync would count the look turning
  -- with V (resends, then "broken" for the session)
  do
    local syncCalls = {}
    W.env.Game.UTPPLookWorldSet = function(on, set)
      syncCalls[#syncCalls + 1] = { on, set }
      if on then store = true end -- the queued VehicleCombat set locks the heading (+ the world heading)
      return true
    end
    cam.cfg.lookSync = true
    cam.G.ls = CM.newLookSync()
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName, cam.G.lockSetSeen = nil, nil, nil, nil, nil
    frames(W, 0.3) -- the sync set is sent for the lock that is set (reconcile)
    local nSync0, nRaw = #syncCalls, nTrue()
    check("R: look sync on: the sync set was sent for the lock (VehicleCombat), route known", nSync0 >= 1 and syncCalls[nSync0][1] == true
          and syncCalls[nSync0][2] == "VehicleCombat" and cam.G.ls.sent == true and cam.G.ls.route ~= nil, tostring(nSync0))
    W.gameSet = "UseConsumable"
    gameUnlock()
    frames(W, 2.0, 60, function() gameUnlock() end)
    check("R: look sync on: UseConsumable is the game's (no bare lock, no sync resend) for the 3 s", store == false and nTrue() == nRaw and #syncCalls == nSync0)
    local stamped = false
    for _ = 1, 120 do -- frame by frame up to the send: the evidence of the sync skips 0.2 s after it (the set applies in a later tick)
      frames(W, 1 / 60, 60, function() if #syncCalls == nSync0 then gameUnlock() end end)
      if #syncCalls > nSync0 then stamped = math.abs(cam.G.clock - cam.G.ls.sentAt) < 1e-6; break end
    end
    frames(W, 0.1)
    check("R: look sync on: after 3 s the name is stale: the lock is restored by sending the sync set again (not by a bare lock), stamped for the evidence",
          store == true and #syncCalls == nSync0 + 1 and syncCalls[#syncCalls][1] == true and syncCalls[#syncCalls][2] == "VehicleCombat"
          and nTrue() == nRaw and stamped, string.format("store %s sends %d raw %d stamped %s", tostring(store), #syncCalls - nSync0, nTrue() - nRaw, tostring(stamped)))
    -- a plain set (Default, e.g. a state the swap did not cover): the sync set again as well
    W.gameSet = "Default"
    cam.G.relock = nil
    local nSync1 = #syncCalls
    gameUnlock()
    frames(W, 8 / 60, 60)
    check("R: look sync on: a lost lock in Default: the sync set is sent again, no bare lock", store == true and #syncCalls == nSync1 + 1 and nTrue() == nRaw)
    -- a failed send is not a write: logged, nothing counted as set again
    W.env.Game.UTPPLookWorldSet = function() return false end
    local qEvent = W.playerObj.QueueEvent
    W.playerObj.QueueEvent = function() error("no event route") end -- both routes of lookSyncSend fail
    cam.G.relock = nil
    local rF = cam.stats.lockRelocks
    gameUnlock()
    frames(W, 8 / 60, 60)
    W.playerObj.QueueEvent = qEvent
    check("R: look sync on, the send fails: not counted as set again, logged once", store == false and cam.stats.lockRelocks == rF
          and count(W.lines, "headingLocked write FAILED (re-lock)") == 1, string.format("store %s relocks %d -> %d", tostring(store), rF, cam.stats.lockRelocks))
    -- the sync off again (the default): the same names are plain and the lock is a bare write as before
    cam.cfg.lookSync = false
    cam.G.ls = CM.newLookSync()
    W.env.Game.UTPPLookWorldSet = nil
    W.gameSet = "UseConsumable"
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName, cam.G.lockSetSeen = nil, nil, nil, nil, nil
    local nB, sB = nTrue(), cam.stats.lockSkips or 0
    gameUnlock()
    frames(W, 8 / 60, 60)
    check("R: look sync off: UseConsumable is plain (the bare lock at once, nothing skipped)", store == true and nTrue() == nB + 1 and (cam.stats.lockSkips or 0) == sB)
    W.gameSet = nil
    frames(W, 12 / 60, 60)
  end
  -- the 3 s of an unlisted set start with each NEW loss, not with the first read of the name (review: the same set again after a long
  -- time - a second landing recovery in the aim - was taken for stale at once and the lock set before the state was over)
  do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName, cam.G.lockSetSeen = nil, nil, nil, nil, nil
    W.gameSet = "LandRecoveryLike"
    local n0 = nTrue()
    gameUnlock()
    frames(W, 2.0, 60, function() gameUnlock() end)
    local first = store == false and nTrue() == n0
    frames(W, 1.5, 60, function() if nTrue() == n0 then gameUnlock() end end)
    local relocked = store == true and nTrue() == n0 + 1
    frames(W, 6.0) -- the lock is back and stays; nobody reads the set name (no loss)
    local n1 = nTrue()
    gameUnlock() -- the game applies the SAME set again (a new state of the same kind)
    frames(W, 2.0, 60, function() gameUnlock() end)
    local again = store == false and nTrue() == n1
    frames(W, 1.5, 60, function() if nTrue() == n1 then gameUnlock() end end)
    check("R: the same unlisted set again after a long time is the game's again for 3 s (the old first-read time is not reused), then plain",
          first and relocked and again and store == true and nTrue() == n1 + 1,
          string.format("first %s relocked %s again %s store %s", tostring(first), tostring(relocked), tostring(again), tostring(store)))
    W.gameSet = nil
    frames(W, 12 / 60, 60)
  end
  -- an unlisted set counts as the game's for lockReassertSetHold (3 s) after it was first seen, then as plain (stale names stay)
  do
    cam.G.relock, cam.G.lockHold, cam.G.lockOwned, cam.G.lockSetName = nil, nil, nil, nil
    W.gameSet = "SomeNewSet"
    local nS = nTrue()
    gameUnlock()
    frames(W, 2.0, 60, function() gameUnlock() end)
    local heldFirst = store == false and nTrue() == nS
    frames(W, 1.5, 60, function() if not (nTrue() > nS) then gameUnlock() end end)
    local logged = count(W.lines, "game set SomeNewSet unchanged for 3 s: treated as plain") == 1
    check("R: an unlisted set is the game's for 3 s (no write), then plain (written, logged once); the stale name does not disable the lock for ever",
          heldFirst and store == true and nTrue() == nS + 1 and logged,
          string.format("heldFirst %s store %s writes %d logged %s", tostring(heldFirst), tostring(store), nTrue() - nS, tostring(logged)))
    -- a new name restarts the 3 s
    W.gameSet = "AnotherNewSet"
    local nA = nTrue()
    gameUnlock()
    frames(W, 2.0, 60, function() gameUnlock() end)
    check("R: a different unlisted set restarts the 3 s: skipped again", store == false and nTrue() == nA)
    W.gameSet = nil
    frames(W, 12 / 60, 60)
  end
  check("R: swimming / takedown / vault / unknown scene / queued set are logged with their own reason",
        count(W.lines, "heading lock lost while swimming (PSM Locomotion 0, LocomotionDetailed 1)") == 1
        and count(W.lines, "heading lock lost while takedown") == 1 and count(W.lines, "heading lock lost while vault") == 1
        and count(W.lines, "heading lock lost while hoverJump") == 1 and count(W.lines, "heading lock lost while game set Consumable") == 1
        and count(W.lines, "heading lock lost while game set Vault") == 1 and count(W.lines, "heading lock lost while scene") == 1,
        table.concat(W.lines, " | "))
  check("R: each scope reason is logged once, with the PSM state",
        count(W.lines, "heading lock lost while vehicle (PSM Locomotion 0, LocomotionDetailed 1): the game's own camera set stays") == 1
        and count(W.lines, "heading lock lost while ladder (PSM Locomotion 0, LocomotionDetailed 10)") == 1
        and count(W.lines, "heading lock lost while carrying") == 1 and count(W.lines, "heading lock lost while scene") == 1)
  -- session reset: the re-assert window starts again
  cam.G.relock = { n = 8, winStart = cam.G.clock, total = 99, paused = cam.G.clock + 50 }
  W.player = false
  frames(W, 0.2)
  check("R: session reset (no player) clears the re-assert window", cam.G.relock == nil)
  check("R: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
end

-- ================================================================ V) vehicle batch (P1a): vehicle API version 2
-- CAM-1 (1) no FPP camera activation while the game's vehicle camera renders in third person for the driver; CAM-1 (2)
-- HoldForVehicle(false, why, "game", blendS) before a hand-over; CAM-2 (2) TakeForVehicleExit; CAM-6 (3) override FOV.
local function vehWorld(persp)
  local W = makeWorld()
  local C = loadCore(W)
  W.vehTpp, W.veh = true, nil
  W.env.Game.GetMountedVehicle = function() return W.veh end
  W.deactBlends, W.actBlends = {}, {}
  local u = W.utpp
  local baseA, baseD = u.Activate, u.Deactivate
  function u:Activate(b, a) W.actBlends[#W.actBlends + 1] = b; baseA(self, b, a) end
  function u:Deactivate(b, a) W.deactBlends[#W.deactBlends + 1] = b; baseD(self, b, a) end
  function u:SetIsHighPriority(on) self.prio = on end
  if persp then
    local baseGetMod = W.env.GetMod
    W.env.GetMod = function(n) if n == "utpp_perspective" then return persp end return baseGetMod(n) end
  end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 2.0)
  return W, C, cam
end
local function mountCar(W, tpp)
  W.vehTpp = tpp
  W.veh = { GetCameraManager = function() return { IsTPPActive = function() return W.vehTpp end } end }
end
do -- CAM-1 T3: own rules, vehicle decision while not held, vehicle camera TPP (driver, state 1)
  local W, _, cam = vehWorld(nil)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("V: in third person before the mount", cam.T.state == "tpp" and W.active == "utpp")
  check("V: vehicle API version 2", cam.VehicleApiVersion == 2 and type(cam.TakeForVehicleExit) == "function")
  local fppA, deact = W.fpp.activations, W.utpp.deactivations
  mountCar(W, true)
  W.psm.Vehicle = 1
  frames(W, 0.3)
  check("V CAM-1 T3: vehicle TPP camera renders -> no FPP camera activation", W.fpp.activations == fppA
    and count(W.lines, "vehicle TPP camera renders (vehicle state 1) - no FPP camera activation") == 1
    and count(W.lines, "): camera.Activate") == 0, table.concat(W.lines, " | "))
  check("V CAM-1 T3: first person state, utppCamera deactivated, TPP signal off", cam.T.state == "fpp"
    and W.utpp.deactivations == deact + 1 and W.tppCalls[#W.tppCalls] == false, cam.T.state)
  check("V CAM-1 T3: representation left to the vehicle camera", count(W.lines, "TPP representation left to the vehicle camera") == 1)
  -- out of the vehicle: third person again (own rules)
  W.veh, W.psm.Vehicle = nil, 0
  frames(W, 1.0)
  check("V: after the ride the camera re-enters third person", cam.T.state == "tpp")
  -- negative: the vehicle camera reports first person -> FPP camera as before
  fppA = W.fpp.activations
  mountCar(W, false)
  W.psm.Vehicle = 1
  frames(W, 0.3)
  check("V CAM-1 negative: vehicle camera in first person -> FPP camera activated", W.fpp.activations == fppA + 1
    and cam.T.state == "fpp")
  W.veh, W.psm.Vehicle = nil, 0
  frames(W, 1.0)
  -- negative: a passenger (state 3, no driver slot) keeps the game's own rules
  fppA = W.fpp.activations
  mountCar(W, true)
  W.psm.Vehicle = 3
  frames(W, 0.3)
  check("V CAM-1 negative: passenger (state 3) -> FPP camera activated", W.fpp.activations == fppA + 1)
  W.veh, W.psm.Vehicle = nil, 0
  frames(W, 1.0)
  -- negative: on foot the FPP camera is activated as before (toggle)
  fppA = W.fpp.activations
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("V on foot: toggle to first person activates the FPP camera", W.fpp.activations == fppA + 1 and cam.T.state == "fpp")
  check("V: no update errors (T3)", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end
do -- CAM-1 T4 + CAM-2 (2) + CAM-6 (3) with a utpp_perspective stub
  local st = { tpp = false, reason = "test", style = "blend" }
  local persp = {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = st.style, reason = st.reason,
                                  mode = "active" } end,
    Mode = function() return "active" end, ToggleUser = function() return true end, SetQuickFpp = function(d) return d end,
  }
  local W, _, cam = vehWorld(persp)
  st.tpp, st.reason = true, "user"
  frames(W, 1.2)
  check("V: third person through the perspective stub", cam.T.state == "tpp" and W.active == "utpp")
  mountCar(W, true)
  W.psm.Vehicle = 4
  local okH
  withEnv(W, function() okH = cam.HoldForVehicle(true, "mount start") end)
  st.tpp, st.reason = false, "vehicle"
  frames(W, 0.3)
  check("V: mount hold keeps utppCamera through the vehicle decision", okH == true and cam.vh.held and W.active == "utpp"
    and count(W.lines, "vehicle hold: decision FPP (vehicle) ignored") == 1)
  -- CAM-1 T4: game release before the hand-over
  local fppA, nD, nA = W.fpp.activations, #W.deactBlends, W.utpp.activations
  local okG
  withEnv(W, function() okG = cam.HoldForVehicle(false, "x", "game", 0.4) end)
  check("V CAM-1 T4: HoldForVehicle(false, game) before the hand-over -> first person state, Deactivate(0.40) once",
    okG == true and cam.T.state == "fpp" and not cam.vh.held and #W.deactBlends == nD + 1 and W.deactBlends[#W.deactBlends] == 0.4
    and count(W.lines, "view left to the game's vehicle camera (no FPP activation), body released, TPP signal off, "
      .. "Deactivate(0.40) ok") == 1, table.concat(W.lines, " | "))
  frames(W, 0.5)
  check("V CAM-1 T4: no FPP camera activation, no second Deactivate (blend not cut), TPP signal off", W.fpp.activations == fppA
    and #W.deactBlends == nD + 1 and W.utpp.activations == nA and W.tppCalls[#W.tppCalls] == false and cam.G.use == nil
    and cam.T.state == "fpp", string.format("fppA %d->%d deact %d->%d", fppA, W.fpp.activations, nD, #W.deactBlends))
  -- default blend 0.30
  withEnv(W, function() cam.HoldForVehicle(true, "y") end)
  check("V: hold refused from first person (VehicleReady)", cam.vh.held == false)
  -- CAM-2 (2): refusals
  local okR, whyR
  W.veh = nil
  withEnv(W, function() okR, whyR = cam.TakeForVehicleExit({ x = 100, y = 45, z = 12 }, 0, -10, 0.25, "test") end)
  check("V CAM-2: TakeForVehicleExit refused when not mounted", okR == false and string.find(tostring(whyR), "not mounted", 1, true) ~= nil,
    tostring(whyR))
  mountCar(W, true)
  -- CAM-2 (2): take-over from the game camera
  W.psm.Vehicle = 4
  local nEnter, nRep = count(W.lines, "enter epoch="), countEv(W, "gameActivateTPPRepresentationEvent")
  local epoch = cam.T.epoch
  local okT
  local pose = { x = 100, y = 44.5, z = 12.2 }
  withEnv(W, function() okT = cam.TakeForVehicleExit(pose, 0, -12, 0.25, "exit start (from the game camera)", 48) end)
  check("V CAM-2: TakeForVehicleExit -> held, third person at once (w = 1), Activate(0.25), priority on", okT == true
    and cam.vh.held and cam.T.state == "tpp" and cam.T.w == 1 and cam.T.want == true and cam.T.epoch == epoch + 1
    and W.actBlends[#W.actBlends] == 0.25 and W.utpp.prio == true and cam.vh.prio == true and W.active == "utpp",
    table.concat(W.lines, " | "))
  local wp0 = W.utppWorld()
  check("V CAM-2: the override pose is written before the activation (world pose, FOV 48)", CM.dist(wp0, CM.v3(100, 44.5, 12.2)) < 1e-6
    and math.abs(W.utpp.fov - 48) < 1e-6, string.format("%.3f,%.3f,%.3f fov %.1f", wp0.x, wp0.y, wp0.z, W.utpp.fov))
  check("V CAM-2: pending rep_on queued (body on the next frame)", type(cam.vh.pending) == "table" and cam.vh.pending[1] == "rep_on"
    and #cam.vh.pending == 1)
  frames(W, 1 / 60)
  check("V CAM-2: pending ops run on the next frame: full body requested, TPP signal on, no cam_on enter from the eyes",
    cam.vh.pending == nil and countEv(W, "gameActivateTPPRepresentationEvent") == nRep + 1 and W.tppCalls[#W.tppCalls] == true
    and count(W.lines, "enter epoch=") == nEnter and W.fpp.activations == fppA, table.concat(W.lines, " | "))
  frames(W, 0.2)
  local wp1 = W.utppWorld()
  check("V CAM-2: override pose and FOV kept while held", CM.dist(wp1, CM.v3(100, 44.5, 12.2)) < 1e-6 and math.abs(W.utpp.fov - 48) < 1e-6)
  -- an explicit "TPP" choice during the exit is a no-op (logged)
  local nAct = W.utpp.activations
  st.tpp, st.reason = true, "user"
  frames(W, 0.1)
  check("V CAM-2: 'TPP <- user' while held is ignored (no second enter)", count(W.lines, "vehicle hold: decision TPP (user) ignored") == 1
    and cam.T.state == "tpp" and count(W.lines, "enter epoch=") == nEnter and W.utpp.activations == nAct)
  -- unmount + release to the orbit: the override FOV blends back to the profile FOV
  W.veh, W.psm.Vehicle = nil, 0
  local okO
  withEnv(W, function() okO = cam.ReleaseToOrbit(0.4, "test release") end)
  -- r13e: after a governed exit hold the release may be longer than the caller's 0.4 s (cfg.vehReleaseSpeed, logged): mid / end from the logged time
  local relDur = 0.4
  for _, l in ipairs(W.lines) do local d = l:match("vehicle release: blend ([%d%.]+) s instead of") if d then relDur = tonumber(d) end end
  frames(W, relDur / 2)
  local midFov = W.utpp.fov
  frames(W, relDur / 2 + 0.2)
  check("V CAM-6 (3): release blends the override FOV back to the profile FOV", okO == true and midFov > 48 + 0.5
    and midFov < MEDIUM.fov - 0.5 and math.abs(W.utpp.fov - MEDIUM.fov) < 0.5 and not cam.vh.held,
    string.format("mid %.2f end %.2f (release %.2f s)", midFov, W.utpp.fov, relDur))
  local okR2, whyR2
  mountCar(W, true)
  withEnv(W, function() okR2, whyR2 = cam.TakeForVehicleExit(pose, 0, -12, 0.25, "test") end)
  check("V CAM-2: refused in third person when not held", okR2 == false and string.find(tostring(whyR2), "camera state tpp", 1, true) ~= nil,
    tostring(whyR2))
  W.veh = nil
  local stv
  withEnv(W, function() stv = cam.VehicleStatus() end)
  check("V: VehicleStatus reports exitTakes / api", string.find(stv, "exitTakes=1", 1, true) ~= nil and string.find(stv, "api=2", 1, true) ~= nil, stv)
  check("V: no update errors (T4 / take-over)", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end
do -- review fix: a failed exit take-over (no camera / Activate failed) never leaves the hold on
  local st = { tpp = false, reason = "test", style = "blend" }
  local persp = {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = st.style, reason = st.reason,
                                  mode = "active" } end,
    Mode = function() return "active" end, ToggleUser = function() return true end, SetQuickFpp = function(d) return d end,
  }
  local W, _, cam = vehWorld(persp)
  st.tpp, st.reason = true, "user"
  frames(W, 1.2)
  mountCar(W, false)
  W.psm.Vehicle = 1
  st.tpp, st.reason = false, "vehicle"
  frames(W, 1.5)
  check("V take-over fail: vehicle first person ride, camera in its first person state", cam.T.state == "fpp"
    and not cam.vh.held, cam.T.state)
  W.psm.Vehicle = 4
  local pose = { x = 100, y = 44.5, z = 12.2 }
  local ops = cam.vehOps
  local baseAct, baseUse = ops.activate, ops.useCam
  local function attempt(label)
    local nRep = countEv(W, "gameActivateTPPRepresentationEvent")
    local okT, whyT
    withEnv(W, function() okT, whyT = cam.TakeForVehicleExit(pose, 0, -12, 0, label) end)
    local heldNow, stateNow = cam.vh.held, cam.T.state
    frames(W, 0.5)
    return okT, whyT, heldNow, stateNow, countEv(W, "gameActivateTPPRepresentationEvent") - nRep
  end
  ops.activate = function() return false, "Activate failed (test)" end
  local okA, whyA, heldA, stateA, repA = attempt("activate fails")
  check("V take-over fail: Activate failed -> false, hold released at once, first person state, no full body request",
    okA == false and string.find(tostring(whyA), "Activate", 1, true) ~= nil and heldA == false and stateA == "fpp"
    and repA == 0 and not cam.vh.held and cam.T.state == "fpp" and cam.vh.prio ~= true and cam.vh.override == nil,
    string.format("%s %s held %s state %s rep %d", tostring(okA), tostring(whyA), tostring(heldA), tostring(stateA), repA))
  ops.activate, ops.useCam = baseAct, function() return false, "utppCamera missing" end
  local okU, whyU, heldU, stateU, repU = attempt("no camera")
  check("V take-over fail: no camera (useCam failed) -> false, hold released, first person state", okU == false
    and string.find(tostring(whyU), "no camera", 1, true) ~= nil and heldU == false and stateU == "fpp" and repU == 0
    and not cam.vh.held and cam.T.state == "fpp", string.format("%s %s", tostring(okU), tostring(whyU)))
  check("V take-over fail: logged with the reason", count(W.lines, "vehicle exit take-over failed (activate fails)") == 1
    and count(W.lines, "vehicle exit take-over failed (no camera)") == 1 and (cam.vh.takeFails or 0) == 2)
  ops.useCam = baseUse
  local okS
  withEnv(W, function() okS = cam.TakeForVehicleExit(pose, 0, -12, 0, "after the failures") end)
  check("V take-over fail: a later working take-over holds as before", okS == true and cam.vh.held and cam.T.state == "tpp")
  W.veh, W.psm.Vehicle = nil, 0
  withEnv(W, function() cam.ReleaseToOrbit(0, "test end") end)
  frames(W, 0.3)
  check("V take-over fail: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0 and not cam.vh.held)
  os.remove(settingsFile)
end
do -- CAM-6 (3): SetPoseOverride without fov keeps the profile FOV (unchanged path); invalid fov ignored
  local W, _, cam = vehWorld(nil)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  withEnv(W, function() cam.HoldForVehicle(true, "t") end)
  withEnv(W, function() cam.SetPoseOverride({ x = 100, y = 46, z = 12 }, 0, -5, "t") end)
  frames(W, 0.1)
  check("V CAM-6 (3): override without fov -> profile FOV", math.abs(W.utpp.fov - MEDIUM.fov) < 0.5 and cam.vh.override.fov == nil,
    W.utpp.fov)
  withEnv(W, function() cam.SetPoseOverride({ x = 100, y = 46, z = 12 }, 0, -5, "t", 500) end)
  frames(W, 0.1)
  check("V CAM-6 (3): out-of-range fov ignored", cam.vh.override.fov == nil and math.abs(W.utpp.fov - MEDIUM.fov) < 0.5)
  withEnv(W, function() cam.HoldForVehicle(false, "t") end)
  frames(W, 0.2)
  check("V: no update errors (fov)", cam.stats.errors == 0)
  os.remove(settingsFile)
end

-- ================================================================ V2) vehicle batch (P1b): own pitch / hold yaw / release / framing / aim
-- the rendered camera: the world camera when it is the active one, else utppCamera
local function shownPose(W)
  if W.active == "wc" then return W.wc.wp, W.wc.wq end
  return W.utppWorld()
end
local function shownYP(W)
  local _, q = shownPose(W)
  return CM.yawPitchFromForward(CM.qrot(q, CM.v3(0, 1, 0)))
end
local function aimPersp()
  local st = { tpp = true }
  return st, {
    Desired = function() return { tpp = st.tpp, perspective = st.tpp and "TPP" or "FPP", style = "blend", reason = "user",
                                  mode = "active" } end,
    Mode = function() return "active" end, ToggleUser = function() return true end, SetQuickFpp = function() return true end,
    cfg = { aiming = "tpp" },
  }
end
-- a frame recorder: rendered yaw / pitch / position per frame, worst change per 50 ms window
local function recorder(W)
  local R = { rows = {} }
  function R.add()
    local y, p = shownYP(W)
    R.rows[#R.rows + 1] = { y = y, p = p, pos = (shownPose(W)), t = W.clock }
  end
  function R.worst(from)
    local wa, wd, wp, n = 0, 0, 0, 3 -- 3 frames = 50 ms at 60 fps
    for i = (from or 1) + n, #R.rows do
      local a, b = R.rows[i - n], R.rows[i]
      wa = math.max(wa, math.abs(CM.wrapDeg(b.y - a.y)) + math.abs(b.p - a.p))
      wd = math.max(wd, CM.dist(a.pos, b.pos))
    end
    for i = (from or 1) + 1, #R.rows do wp = math.max(wp, math.abs(R.rows[i].p - R.rows[i - 1].p)) end
    return wa, wd, wp
  end
  return R
end
do -- CAM-3 T3/T4 + CAR-4 + CAM-4 T1/T2 in the harness (car exit with the game's look pitch at -60)
  local _, persp = aimPersp()
  local W, _, cam = vehWorld(persp)
  W.speed = 2 -- moving: no idle orbit (its free look has its own pitch)
  frames(W, 1.2)
  check("motion: third person before the mount", cam.T.state == "tpp" and W.active == "utpp")
  local R = recorder(W)
  mountCar(W, true)
  W.psm.Vehicle = 4
  withEnv(W, function() cam.HoldForVehicle(true, "mount start") end)
  -- CAM-3 T4: the mount animation pitches the game's look 0 -> -30; CAM-4 T1: the capsule turns 120 deg to the door in 0.15 s
  local y0, p0 = shownYP(W)
  local mark = #R.rows + 1
  frames(W, 0.6, 60, function(i)
    if i == 10 then W.fppPitch = -30 end
    if i >= 12 and i <= 21 then W.yaw = W.yaw + 12 end
    R.add()
  end)
  local wa, _, wp = R.worst(mark)
  local y1, p1 = shownYP(W)
  check("motion CAM-3 T4: mount hold - the game's look pitch step does not move the shown pitch (<= 2 deg per frame)",
    wp <= 2 and math.abs(p1 - p0) < 0.5, string.format("max %.2f deg/frame, %.2f -> %.2f", wp, p0, p1))
  check("motion CAM-4 T1: mount hold - V's capsule turning 120 deg does not whip the shown yaw (<= 25 deg per 50 ms)",
    wa <= 25 and math.abs(CM.wrapDeg(y1 - y0)) < 1, string.format("max %.2f per 50 ms, yaw %.1f -> %.1f", wa, y0, y1))
  -- pull-out / exit hold: the vehicle-behind override pose, the game's look at -60 (exit animation)
  local vy = W.yaw
  local behind = CM.add(W.pos, CM.qrot(CM.qFromYawPitch(vy, 0), CM.v3(0, -5.5, 2.4)))
  withEnv(W, function() cam.SetPoseOverride(behind, vy, -14, "vehicle-behind") end)
  W.fppPitch = -60
  frames(W, 0.3)
  -- unmount: V turned +150 deg relative to the vehicle (CAM-4 T2), still at -60
  W.veh, W.psm.Vehicle = nil, 0
  W.yaw = CM.wrapDeg(vy + 150)
  frames(W, 0.3)
  mark = #R.rows + 1
  R.add()
  local okR
  withEnv(W, function() okR = cam.ReleaseToOrbit(0.8, "exit release") end)
  local yr0 = select(1, shownYP(W))
  frames(W, 0.9, 60, function() R.add() end)
  local wa2, wd2 = R.worst(mark)
  local ye, pe = shownYP(W)
  check("motion CAM-3 T3 / CAR-4: release ends inside [-25, 5] near the start pitch (not the game's -60), hold over",
    okR == true and not cam.vh.held and pe >= -25 - 1e-6 and pe <= 5 + 1e-6 and math.abs(pe - (-14)) <= 11,
    string.format("end pitch %.2f held=%s", pe, tostring(cam.vh.held)))
  check("motion CAM-3: the look state continues from the released pitch", math.abs(cam.S.look.pitch - pe) < 0.5,
    string.format("look %.2f shown %.2f", cam.S.look.pitch, pe))
  check("motion CAM-4 T2: the release keeps the shown yaw (V turned 150 deg: <= 30 deg over the release)",
    math.abs(CM.wrapDeg(ye - yr0)) <= 30, string.format("%.1f -> %.1f", yr0, ye))
  check("motion: release continuous (<= 25 deg per 50 ms, <= 0.8 m per 50 ms)", wa2 <= 25 and wd2 <= 0.8,
    string.format("%.2f deg %.3f m", wa2, wd2))
  -- after the hold: the game's look still at -60 (exit animation) -> own pitch kept (no slide back to top-down)
  mark = #R.rows + 1
  frames(W, 1.0, 60, function() R.add() end)
  local _, pk = shownYP(W)
  check("motion CAM-3: while the game's look stays at -60 after the hold the camera keeps its pitch", math.abs(pk - pe) < 1,
    string.format("%.2f -> %.2f", pe, pk))
  -- the animation ended: the game's look pitch back to -8 -> glide (<= 2 deg per frame at 60 fps), then the game's pitch
  W.fppPitch = -8
  mark = #R.rows + 1
  frames(W, 2.5, 60, function() R.add() end)
  local _, _, wp3 = R.worst(mark)
  local _, pg = shownYP(W)
  check("motion CAM-3: the game's look pitch back in range -> glide onto it (<= 2 deg per frame), then the follow pitch",
    wp3 <= 2.01 and math.abs(pg - (-8)) < 0.6, string.format("max %.2f deg/frame end %.2f", wp3, pg))
  check("motion: own pitch lines logged (vehicle hold, OFF)", count(W.lines, "own pitch ON (vehicle hold)") >= 1
    and count(W.lines, "own pitch OFF -> the follow pitch glides onto the game's look pitch") >= 1, table.concat(W.lines, " | "))
  check("motion: no update errors (release)", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end
do -- release-end pitch clamp: a hold pose looking steeply down / up ends at -25 / +5 (camera cfg vehReleasePitchMin / Max)
  for _, c in ipairs({ { from = -40, want = -25 }, { from = 20, want = 5 }, { from = -12, want = -12 } }) do
    local _, persp = aimPersp()
    local W, _, cam = vehWorld(persp)
    W.speed = 2
    frames(W, 1.2)
    mountCar(W, true) -- mounted: no framing band (CAR-3), the clamp alone
    withEnv(W, function() cam.HoldForVehicle(true, "hold") end)
    local behind = CM.add(W.pos, CM.qrot(CM.qFromYawPitch(W.yaw, 0), CM.v3(0, -3.0, 1.6)))
    withEnv(W, function() cam.SetPoseOverride(behind, W.yaw, c.from, "pose") end)
    frames(W, 0.2)
    withEnv(W, function() cam.ReleaseToOrbit(0.5, "release") end)
    frames(W, 0.6)
    local _, pe = shownYP(W)
    check(string.format("motion: release-end pitch clamp: hold pitch %d -> %d", c.from, c.want), not cam.vh.held
      and math.abs(pe - c.want) < 0.5 and math.abs(cam.S.look.pitch - c.want) < 0.5, string.format("%.2f", pe))
    os.remove(settingsFile)
  end
end
do -- CAR-3 (1)(2): V launched up after the unmount stays framed (rate limited pitch), negative control without the band
  local function run(band)
    local _, persp = aimPersp()
    local W, _, cam = vehWorld(persp)
    frames(W, 1.2)
    cam.cfg.vehFrame = band
    mountCar(W, true)
    W.psm.Vehicle = 4
    withEnv(W, function() cam.HoldForVehicle(true, "mount start") end)
    local vy = W.yaw
    local behind = CM.add(W.pos, CM.qrot(CM.qFromYawPitch(vy, 0), CM.v3(0, -5.5, 2.4)))
    withEnv(W, function() cam.SetPoseOverride(behind, vy, -14, "vehicle-behind") end)
    frames(W, 0.2)
    W.veh, W.psm.Vehicle = nil, 0
    local worstHead, worstStep, prevP = 0, 0, nil
    frames(W, 0.8, 60, function(i)
      if i <= 30 then W.pos = CM.add(W.pos, CM.v3(0, 0, 2.5 / 30)) end
    end)
    -- per-frame check over a second run of frames (after the launch, V stays up)
    local inBand = true
    frames(W, 0.4, 60, function()
      local y, p = shownYP(W)
      local cp = W.utppWorld()
      local head = CM.add(W.pos, CM.v3(0, 0, 1.4))
      local d = CM.sub(head, cp)
      local el = math.deg(math.atan2(d.z, math.sqrt(d.x * d.x + d.y * d.y)))
      worstHead = math.max(worstHead, math.abs(el - p))
      if prevP then worstStep = math.max(worstStep, math.abs(p - prevP)) end
      prevP = p
      if el - p > 0.25 * 28 + 0.05 or el - p < -0.35 * 28 - 0.05 then inBand = false end
    end)
    withEnv(W, function() cam.HoldForVehicle(false, "end") end)
    os.remove(settingsFile)
    return inBand, worstHead, worstStep, W, cam
  end
  local inBand, head, step, W, cam = run(nil)
  check("CAR-3 (1): V launched 2.5 m up after the unmount stays inside the framing band (head <= 25 % of the half FOV)",
    inBand and step <= 3, string.format("head off-axis %.1f deg, step %.2f", head, step))
  check("CAR-3: framing logged once", count(W.lines, "vehicle framing: V's pivot kept in the frame after the unmount") == 1
    and cam.stats.errors == 0)
  local inBandN, headN = run(false)
  check("CAR-3 negative control (band off = the old behaviour): V's head leaves the band", not inBandN,
    string.format("head off-axis %.1f deg", headN))
end
do -- CAM-9: aim requested while held is not started; the first aim after the release blends in 0.20 s, no snaps
  local _, persp = aimPersp()
  local W, C, cam = vehWorld(persp)
  W.psm.LocomotionDetailed = 1
  W.records["Items.Preset_Lexington_Default"] = "Wea_Handgun"
  W.statValues["ent_Items.Preset_Lexington_Default"] = 48
  local it = {}
  function it:GetItemID() return { tdb = "Items.Preset_Lexington_Default" } end
  function it:GetEntityID() return { hash = "ent_Items.Preset_Lexington_Default" } end
  frames(W, 1.2)
  -- on-foot aim (reference: normal blend, no suffix)
  W.weapon, W.weaponItem = true, it
  W.psm.UpperBody = 6
  frames(W, 0.3)
  check("motion CAM-9: on-foot aim unchanged (blend in = aimBlendIn, no vehicle suffix)", cam.aim.on
    and math.abs(cam.aim.dur - (cam.cfg.aimBlendIn or 0.12)) < 1e-9 and count(W.lines, "first aim after a vehicle hold") == 0)
  W.psm.UpperBody = 0
  frames(W, 0.5)
  local R = recorder(W)
  mountCar(W, true)
  withEnv(W, function() cam.HoldForVehicle(true, "exit hold") end)
  local vy = W.yaw
  local behind = CM.add(W.pos, CM.qrot(CM.qFromYawPitch(vy + 40, 0), CM.v3(0, -5.5, 2.4)))
  withEnv(W, function() cam.SetPoseOverride(behind, vy + 40, -14, "vehicle-behind") end)
  W.veh = nil
  W.psm.UpperBody = 6 -- aim requested while held
  frames(W, 0.3)
  check("motion CAM-9: no aim while held", not cam.aim.on and count(W.lines, "cam aim exit (vehicle hold)") == 0)
  local mark = #R.rows + 1
  withEnv(W, function() cam.ReleaseToOrbit(0.4, "exit release") end)
  frames(W, 1.2, 60, function() R.add() end)
  local wa, wd = R.worst(mark)
  check("motion CAM-9: the first aim after the release blends in 0.20 s (logged)", cam.aim.on and math.abs(cam.aim.dur - 0.20) < 1e-9
    and count(W.lines, "(first aim after a vehicle hold: blend in 0.20 s)") == 1, table.concat(W.lines, " | "))
  check("motion CAM-9: release + aim without snaps (<= 25 deg / 0.8 m per 50 ms)", wa <= 25 and wd <= 0.8,
    string.format("%.2f deg %.3f m", wa, wd))
  W.psm.UpperBody = 0
  frames(W, 0.5)
  W.psm.UpperBody = 6
  frames(W, 0.3)
  check("motion CAM-9: the next aim is a normal one again", cam.aim.on and math.abs(cam.aim.dur - (cam.cfg.aimBlendIn or 0.12)) < 1e-9
    and count(W.lines, "first aim after a vehicle hold") == 1)
  check("motion: no update errors (aim)", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  local _ = C
  os.remove(settingsFile)
end

-- TRAV-4 (CAM-3): the game look write turns OFF ('heading unlocked (follow)') mid-vault (LocomotionDetailed 9) while the
-- vault animation moves the game's FPP pitch: no one-frame pitch snap (<= 0.5 deg on that frame and while vaulting)
do
  local W, _, cam = lwSetup()
  W.speed = 2
  cam.FreeLook(true)
  frames(W, 0.5, 60, function() W.lwTick(5, 0) end)
  check("TRAV-4: free look with the game look write ON", cam.G.lwState == "on")
  local _, p0 = shownYP(W)
  W.psm.LocomotionDetailed = 9
  cam.FreeLook(false)
  local R0 = recorder(W)
  R0.add()
  frames(W, 0.4, 60, function(i)
    W.lwTick(0, 0)
    W.fppPitch = -40 -- the vault animation's camera pitch (after the mock's tick), read back once the write is off
    R0.add()
  end)
  local _, _, worst = R0.worst(1)
  local _, p1 = shownYP(W)
  check("TRAV-4: look write OFF mid-vault -> the camera keeps its pitch on every frame (no snap onto the vault pitch)",
    count(W.lines, "game look write OFF (heading unlocked (follow))") == 1 and worst <= 0.5 and math.abs(p1 - p0) <= 0.5
    and W.active == "wc", string.format("%.2f -> %.2f worst %.2f", p0, p1, worst))
  check("TRAV-4: own pitch logged with the traversal state", count(W.lines, "own pitch ON (traversal: LocomotionDetailed 9)") == 1,
    table.concat(W.lines, " | "))
  -- the vault ends, the game's pitch stays -40 -> glide (<= 2 deg per frame), no snap
  W.psm.LocomotionDetailed = 1
  local wp, pp = 0, p1
  frames(W, 1.5, 60, function() W.lwTick(0, 0); W.fppPitch = -40 end)
  local _, p2 = shownYP(W)
  check("TRAV-4: after the vault the pitch glides onto the game's (reached)", math.abs(p2 + 40) < 0.6,
    string.format("%.2f look.pitch=%.2f rec=%s ownP=%s free=%s why=%s fpp=%.1f lw=%s src=%s", p2, cam.S.look.pitch,
      tostring(cam.S.look.recovering), tostring(cam.S.look.ownP), tostring(cam.S.look.free), tostring(cam.G.ownPitchWhy),
      W.fppPitch, tostring(cam.G.lwState), tostring(cam.cfg.pitchSource)))
  W.psm.LocomotionDetailed = 9
  cam.FreeLook(true)
  frames(W, 0.3, 60, function() W.lwTick(0, 0) end)
  cam.FreeLook(false)
  local R = recorder(W)
  frames(W, 0.3, 60, function() W.lwTick(0, 0); W.fppPitch = -10; R.add() end)
  local _, _, wpf = R.worst(1)
  check("TRAV-4: every frame of a second vault release <= 0.5 deg", wpf <= 0.5, wpf)
  local _ = wp + pp
  -- plain locomotion (no traversal): the 0.5 s after the look write OFF edge keep the own pitch, then the glide
  W.psm.LocomotionDetailed = 1
  frames(W, 1.0, 60, function() W.lwTick(0, 0); W.fppPitch = -10 end)
  cam.FreeLook(true)
  frames(W, 0.3, 60, function() W.lwTick(0, 0) end)
  local nOff = count(W.lines, "game look write OFF (heading unlocked (follow))") + 1
  cam.FreeLook(false)
  local R2 = recorder(W)
  R2.add()
  -- (the game's raw pitch is read back once the write is off; while it is on the read-back is the written echo)
  frames(W, 0.4, 60, function() W.lwTick(0, 0); if cam.G.lwState ~= "on" then W.fppPitch = -35 end; R2.add() end)
  local _, _, w2 = R2.worst(1)
  check("CAM-3 / TRAV-4: on foot, the 0.4 s after the look write OFF edge keep the camera pitch (game pitch moved 25 deg)",
    w2 <= 0.5 and count(W.lines, "game look write OFF (heading unlocked (follow))") == nOff, string.format("worst %.2f", w2))
  local R3 = recorder(W)
  R3.add()
  frames(W, 1.0, 60, function() W.lwTick(0, 0); W.fppPitch = -35; R3.add() end)
  local _, _, w3 = R3.worst(1)
  local _, p3 = shownYP(W)
  check("CAM-3 / TRAV-4: then the follow pitch glides onto the game's (<= 2 deg per frame, reached)", w3 <= 2.01
    and math.abs(p3 + 35) < 0.6, string.format("worst %.2f end %.2f", w3, p3))
  check("TRAV-4: no update errors", cam.stats.errors == 0)
  os.remove(settingsFile)
end

-- ================================================================ E) utpp_probe
do
  local W = makeWorld()
  local P
  withEnv(W, function() P = dofile(UTPP_PROBE_PATH) end)
  check("E: probe loads", type(P) == "table" and type(P.Cycle) == "function")
  P.LOG_FILE = TMP .. "utpp_probe_test.log"
  os.remove(P.LOG_FILE)
  withEnv(W, function() W.handlers.onInit(); P.Cycle(1.0) end)
  frames(W, 2.5)
  check("E: Cycle: dumps before / rep_on / rep_off", count(W.logged, "DUMP before begin") == 1 and count(W.logged, "DUMP rep_on end") == 1
        and count(W.logged, "DUMP rep_off end") == 1 and count(W.logged, "CYCLE done") == 1, #W.logged)
  check("E: component lines with class, enabled, mesh hash, chunk mask",
        count(W.logged, "D|before|1|entGarmentSkinnedMeshComponent|t0_000_pwa_fpp__torso|en=true|app=app_t0_000_pwa_fpp__torso|meshApp=default|mesh=H_t0_000_pwa_fpp__torso") == 1,
        W.logged[3])
  check("E: item lines (slot, record, appearance) and item components", count(W.logged, "I|before|Chest|Items.TestJacket|app=jacket_&FPP") == 1
        and count(W.logged, "IC|before|Chest|") == 1)
  check("E: representation events queued (on, off)", countEv(W, "gameActivateTPPRepresentationEvent") == 1
        and countEv(W, "gameDeactivateTPPRepresentationEvent") == 1)
  check("E: diff before>rep_on shows the 2 switched body parts", count(W.logged, "DIFF before>rep_on: 2 changes") == 1
        and count(W.logged, "DIFF before>rep_off: 0 changes") == 1)
  check("E: camera lines (active, camera, utppCamera)", count(W.logged, "C|before|active|") == 1
        and count(W.logged, "C|before|utppCamera|class=gameCameraComponent") == 1)
  withEnv(W, function() P.CamSpike(true) end)
  frames(W, 0.5)
  check("E: CamSpike: takeover detected", count(W.logged, "TAKEOVER OK") == 1, W.logged[#W.logged])
  withEnv(W, function() P.CamSpike(false) end)
  frames(W, 0.2)
  check("E: CamSpike off: FPP camera back", W.active == "fpp" and count(W.logged, "CamSpike OFF") == 1)
  local f = io.open(P.LOG_FILE, "r")
  local okFile = f ~= nil
  if f then f:close() end
  check("E: probe_dump.log written", okFile)
  os.remove(P.LOG_FILE)
end

-- ================================================================ 2026-11-01 NP-3 (C2): near-camera head hide while inside
-- Default eye-start blend (camera.clearanceBlend off): the start of the enter blend is inside the body clearance -> HeadGuard TPP
-- value 0.2025 there, back to 0 once the boom is out (blend timing unchanged; the edge lines only with camera.trace, which the
-- fixture sets). Wall 0.2 m behind V: the boom is placed before the wall, inside -> 0.2025 on that frame, one line; wall gone ->
-- back to 0 after releaseDelay, one line; a session reset while inside releases it; the TPP signal is not touched.
local function c2World(native, trace)
  local W = makeWorld()
  W.hg = {}
  if native == true then W.env.Game.UTPPHeadGuardSetValue = function(v) W.hg[#W.hg + 1] = v; return true end
  elseif native == false then W.env.Game.UTPPHeadGuardSetValue = function(v) W.hg[#W.hg + 1] = v; return false end end
  local C = loadCore(W)
  if trace == false then C.settings.stored["camera.trace"] = "false" end
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 0.5)
  return W, cam
end
do
  local W, cam = c2World(true)
  withEnv(W, function() cam.Toggle("key") end)
  local ws = {}
  frames(W, 2.5, 60, function() ws[#ws + 1] = string.format("%s:%.6f", cam.T.state, cam.T.w) end)
  local tppCalls = #W.tppCalls
  check("C2: eye-start enter blend: hide ON at the blend start, back to 0 once out of the clearance, TPP, trace lines",
    cam.T.state == "tpp" and #W.hg == 2 and W.hg[1] == 0.2025 and W.hg[2] == 0 and cam.insideHide.on == false
      and count(W.lines, "camera inside the body clearance (boom ") == 1 and count(W.lines, "enter / exit blend)") == 1
      and count(W.lines, "camera out of the body clearance (") == 1, table.concat(W.hg, ",") .. " | " .. table.concat(W.lines, " | "))
  -- the same enter without the native: identical blend (state and w every frame), one 'unavailable' line, no FAILED
  local W0, cam0 = c2World(nil)
  withEnv(W0, function() cam0.Toggle("key") end)
  local ws0 = {}
  frames(W0, 2.5, 60, function() ws0[#ws0 + 1] = string.format("%s:%.6f", cam0.T.state, cam0.T.w) end)
  check("C2: blend timing unchanged by the head hide (state / w per frame equal with and without the native)",
    table.concat(ws, ",") == table.concat(ws0, ","))
  check("C2: native missing (older DLL): one 'unavailable' line, no FAILED line, no update error, never retried",
    count(W0.lines, "near-camera head hide while the camera is inside V: unavailable") == 1 and count(W0.lines, "FAILED (") == 0
      and count(W0.lines, "update error") == 0 and cam0.insideHide.on == false and cam0.insideHide.missing == true,
    table.concat(W0.lines, " | "))
  -- camera.trace off: the blend edges write the value without a log line
  local Wq, camq = c2World(true, false)
  withEnv(Wq, function() camq.Toggle("key") end)
  frames(Wq, 2.5)
  check("C2: trace off: blend-only edges are silent (value written and restored, no line)", #Wq.hg == 2 and Wq.hg[2] == 0
    and count(Wq.lines, "body clearance") == 0, table.concat(Wq.lines, " | "))
  -- wall
  local n0, l0, o0 = #W.hg, count(W.lines, "body clearance"), count(W.lines, "camera out of the body clearance (boom ")
  W.wallY = W.pos.y - 0.2
  frames(W, 0.1)
  check("C2: wall 0.2 m behind V: camera before the wall, inside -> HeadGuard TPP value 0.2025 once, one line",
    W.utppWorld().y > W.wallY and #W.hg == n0 + 1 and W.hg[n0 + 1] == 0.2025 and cam.insideHide.on == true
      and count(W.lines, "body clearance") == l0 + 1, table.concat(W.lines, " | "))
  frames(W, 0.5)
  check("C2: staying inside: no further call or line", #W.hg == n0 + 1 and count(W.lines, "body clearance") == l0 + 1)
  W.wallY = nil
  frames(W, 0.1)
  check("C2: wall gone: hide still on during releaseDelay", #W.hg == n0 + 1 and cam.insideHide.on == true)
  frames(W, 0.5)
  check("C2: wall gone > releaseDelay: back to the TPP value 0, one line", #W.hg == n0 + 2 and W.hg[n0 + 2] == 0
    and cam.insideHide.on == false and count(W.lines, "camera out of the body clearance (boom ") == o0 + 1, table.concat(W.lines, " | "))
  W.wallY = W.pos.y - 0.2
  frames(W, 0.1)
  W.player = false
  frames(W, 0.1)
  check("C2: session reset (no player) while inside releases at once", #W.hg == n0 + 4 and W.hg[n0 + 3] == 0.2025
    and W.hg[n0 + 4] == 0 and cam.insideHide.on == false
    and count(W.lines, "camera out of the body clearance (session reset: no player)") == 1, table.concat(W.hg, ","))
  check("C2: the TPP signal is not touched by the head hide", #W.tppCalls == tppCalls + 1 and W.tppCalls[#W.tppCalls] == false,
    #W.tppCalls .. " vs " .. tppCalls)
  -- wall already there at the toggle: no hide while the FPP camera still renders (rep_wait), the first call with utppCamera
  -- shown, no on / off flapping through the enter
  local W3, cam3 = c2World(true)
  W3.wallY = W3.pos.y - 0.2
  withEnv(W3, function() cam3.Toggle("key") end)
  local seen, states = 0, {}
  frames(W3, 2.5, 60, function()
    if #W3.hg > seen then seen = #W3.hg; states[#states + 1] = cam3.T.state end
  end)
  check("C2: wall before the toggle: first hide with utppCamera shown (not in rep_wait), one call, no flapping",
    #W3.hg == 1 and W3.hg[1] == 0.2025 and states[1] ~= "rep_wait" and states[1] ~= "fpp" and cam3.T.state == "tpp",
    table.concat(W3.hg, ",") .. " | " .. table.concat(states, ","))
  -- mounted (the game reports a mounted vehicle) with the boom inside: no HeadGuard write (on foot only)
  local W4, cam4 = c2World(true)
  withEnv(W4, function() cam4.Toggle("key") end)
  frames(W4, 2.5)
  local n4 = #W4.hg
  W4.env.Game.GetMountedVehicle = function() return { mock = "vehicle" } end
  W4.wallY = W4.pos.y - 0.2
  frames(W4, 0.5)
  check("C2: mounted + boom inside the clearance: no HeadGuard write", #W4.hg == n4 and cam4.insideHide.on == false,
    table.concat(W4.hg, ","))
  -- a native answering false: one FAILED line, not marked on
  local W2, cam2 = c2World(false)
  withEnv(W2, function() cam2.Toggle("key") end)
  frames(W2, 2.5)
  W2.wallY = W2.pos.y - 0.2
  frames(W2, 0.5)
  check("C2: UTPPHeadGuardSetValue answering false: one FAILED line, no update error, hide not marked on",
    count(W2.lines, "near-camera head hide FAILED") == 1 and count(W2.lines, "update error") == 0 and cam2.insideHide.on == false,
    table.concat(W2.lines, " | "))
end

-- ================================================================ SK) 2026-11-09 shoulder-swap input (utpp_shoulder_input.reds
-- requests drained through Game.UTPPShoulderSwapTake / Source): missing system logged once, 1 request -> one swap with the
-- source in the log, 2 in one frame cancel out, 0 -> nothing, gates (menu, photo mode, vehicle, scanner, takedown, first
-- person, camera off) drop the request with the reason, rate-limited log; actionEdge numeric types follow RED4ext.SDK.
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local baseDefs = W.env.GetAllBlackboardDefs
  W.env.GetAllBlackboardDefs = function()
    local d = baseDefs()
    d.PlayerStateMachine.Vision = "Vision"
    return d
  end
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  cam.SHOULDER_IN_RETRY = 0.5
  frames(W, 2.0)
  check("SK: reds function missing -> one log line, no update error, Status shoulderIn=missing",
    count(W.lines, "shoulder input: Game.UTPPShoulderSwapTake unavailable") == 1 and cam.stats.errors == 0
    and string.find(cam.ShoulderInputStatus(), "shoulderIn=missing", 1, true) ~= nil, table.concat(W.lines, " | "))
  W.env.Game.UTPPShoulderSwapTake = function() return -1 end
  frames(W, 2.0)
  check("SK: system missing (-1) after the missing function: still one 'shoulder input:' line (retried, not re-logged)",
    count(W.lines, "shoulder input: ") == 1 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  W.shReq, W.shSrc = 0, ""
  W.env.Game.UTPPShoulderSwapTake = function() local n = W.shReq; W.shReq = 0; return n end
  W.env.Game.UTPPShoulderSwapTrigger = function() return W.shSrc end
  frames(W, 0.6)
  check("SK: system found after the retry -> logged once, Status shoulderIn=ok",
    count(W.lines, "shoulder input: UTPPShoulderInput found") == 1
    and string.find(cam.ShoulderInputStatus(), "shoulderIn=ok", 1, true) ~= nil, cam.ShoulderInputStatus())
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("SK: third person, right shoulder at the start", cam.T.state == "tpp" and W.active == "utpp" and cam.shoulder.target == 1)
  local function req(n, src) W.shReq, W.shSrc = n, src or "key"; frames(W, 1 / 60) end
  -- 1 request -> one swap
  req(1, "key")
  frames(W, 0.5)
  check("SK: key request -> one swap, log 'shoulder -> left (key)', boom on the left", cam.shoulder.target == -1
    and count(W.lines, "shoulder -> left (key)") == 1 and W.utpp.lp.x < -0.3, string.format("%.3f", W.utpp.lp.x))
  -- 2 requests in one frame -> no net change
  local swaps = cam.shIn.swaps
  req(2, "pad")
  check("SK: 2 requests in one frame cancel out (no swap, one line)", cam.shoulder.target == -1 and cam.shIn.swaps == swaps
    and count(W.lines, "shoulder swap: 2 requests in one frame cancel out (pad)") == 1)
  -- 0 -> nothing
  local nSh, swaps0, req0 = count(W.lines, "shoulder"), cam.shIn.swaps, cam.shIn.requests
  frames(W, 0.2)
  check("SK: no request -> nothing logged or counted by the input", count(W.lines, "shoulder") == nSh
    and cam.shIn.swaps == swaps0 and cam.shIn.requests == req0 and cam.shoulder.target == -1)
  req(1, "pad")
  frames(W, 0.5)
  check("SK: pad request -> back to the right shoulder, log 'shoulder -> right (pad)'", cam.shoulder.target == 1
    and count(W.lines, "shoulder -> right (pad)") == 1 and W.utpp.lp.x > 0.3)
  -- gates: the request is dropped with the reason, never applied later
  local function gated(label, setup, reason, teardown)
    local t = cam.shoulder.target
    local ign = cam.shIn.ignored
    setup()
    req(1, "key")
    local logged = count(W.lines, "shoulder swap ignored (" .. reason .. ") [key]")
    teardown()
    frames(W, 1.5)
    check("SK: " .. label .. " -> ignored (" .. reason .. "), not applied later", cam.shoulder.target == t
      and cam.shIn.ignored == ign + 1 and logged >= 1, table.concat(W.lines, " | "))
  end
  gated("menu", function() W.menu = true end, "menu", function() W.menu = false end)
  gated("photo mode", function() W.photo = true end, "photo mode", function() W.photo = false end)
  gated("scanner (PSM Vision 1)", function() W.psm.Vision = 1 end, "scanner", function() W.psm.Vision = 0 end)
  gated("takedown", function() W.psm.Takedown = 1 end, "takedown", function() W.psm.Takedown = 0 end)
  gated("body carry", function() W.psm.BodyCarrying = 1 end, "carrying", function() W.psm.BodyCarrying = 0 end)
  gated("swimming", function() W.psm.HighLevel = 6 end, "swimming", function() W.psm.HighLevel = 0 end)
  gated("vehicle (PSM Vehicle)", function() W.psm.Vehicle = 1 end, "vehicle", function() W.psm.Vehicle = 0 end)
  frames(W, 1.0)
  check("SK: third person again after the gate checks", cam.T.state == "tpp" and W.active == "utpp", cam.T.state .. " " .. W.active)
  -- rate limit: the same reason twice within 2 s -> one line
  local nMenu = count(W.lines, "shoulder swap ignored (menu)")
  W.menu = true
  req(1, "key"); frames(W, 0.2); req(1, "key")
  W.menu = false
  check("SK: same reason twice within 2 s -> logged once (both counted)", count(W.lines, "shoulder swap ignored (menu)") == nMenu + 1)
  frames(W, 1.0)
  -- first person: ignored, not remembered for the next third person view (O6)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  local t = cam.shoulder.target
  req(1, "pad")
  check("SK: first person -> ignored (first person)", cam.T.state ~= "tpp" and cam.shoulder.target == t
    and count(W.lines, "shoulder swap ignored (first person) [pad]") == 1, cam.T.state)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  check("SK: back in third person: the first person request was not remembered", cam.T.state == "tpp" and cam.shoulder.target == t)
  -- camera off
  withEnv(W, function() C.Set("camera.impl", "off") end)
  frames(W, 0.5)
  req(1, "key")
  check("SK: camera off -> ignored (UTPP camera off)", cam.shoulder.target == t
    and count(W.lines, "shoulder swap ignored (UTPP camera off) [key]") == 1)
  withEnv(W, function() C.Set("camera.impl", "on") end)
  -- console Swap() keeps the old log line (no source label)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  local before = count(W.lines, "shoulder -> ")
  withEnv(W, function() cam.Swap() end)
  check("SK: console Swap() logs the plain 'shoulder -> left/right' line", count(W.lines, "shoulder -> ") == before + 1
    and string.find(W.lines[#W.lines], "%(key%)") == nil and string.find(W.lines[#W.lines], "%(pad%)") == nil, W.lines[#W.lines])
  local st
  withEnv(W, function() st = cam.Status() end)
  check("SK: Status() ends with the shoulder input block", string.find(st, "shoulderIn=ok requests=", 1, true) ~= nil, st)
  check("SK: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0, table.concat(W.lines, " | "))
  os.remove(settingsFile)
end
-- actionEdge (unarmed zoom wheel, own rules): numeric gameinputActionType follows RED4ext.SDK - 8 TOGGLE_PRESSED steps,
-- 9 TOGGLE_RELEASED and 7 RELATIVE_CHANGE do not, 0 BUTTON_PRESSED steps
do
  local W = makeWorld()
  W.psm.LocomotionDetailed = 1
  local C = loadCore(W)
  local cam = loadCamera(W)
  withEnv(W, function() W.handlers.onInit() end)
  frames(W, 2.0)
  withEnv(W, function() cam.Toggle("key") end)
  frames(W, 1.0)
  W.psm.UpperBody = 6
  frames(W, 0.4)
  W.env.ListenerAction.GetType = function(a) return a.type end
  local function act(name, ty) withEnv(W, function() W.observers["PlayerPuppet.OnAction"](nil, { name = name, value = 0, type = ty }) end) end
  check("SK2: unarmed zoom on at step 0", cam.zoomU.on and cam.zoomU.k == 0)
  act("ZoomIn", 7); frames(W, 0.1)
  check("SK2: type 7 (RELATIVE_CHANGE) -> no step", cam.zoomU.k == 0)
  act("ZoomIn", 9); frames(W, 0.1)
  check("SK2: type 9 (TOGGLE_RELEASED) -> no step", cam.zoomU.k == 0)
  act("ZoomIn", 8); frames(W, 0.1)
  check("SK2: type 8 (TOGGLE_PRESSED) -> one step", cam.zoomU.k == 1)
  act("ZoomIn", 0); frames(W, 0.1)
  check("SK2: type 0 (BUTTON_PRESSED) -> one step", cam.zoomU.k == 2)
  act("ZoomIn", 1); frames(W, 0.1)
  check("SK2: type 1 (BUTTON_RELEASED) -> no step", cam.zoomU.k == 2)
  W.psm.UpperBody = 0
  frames(W, 0.5)
  check("SK2: no update errors", cam.stats.errors == 0 and count(W.lines, "update error") == 0)
  os.remove(settingsFile)
end

print = savedPrint
print(string.format("utpp_camera CET load test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_camera CET load test failed")

-- Offline tests of the utpp_core side of the DLL head look layer: the live target / release sent through Game.UTPPSetHeadLook, the
-- fallback to the game's own look-at request (UTPPHeadLook system) while the layer cannot act, and the raw (unclamped) target of
-- HL.Target. Pure logic with a mocked CET / game, no game. Runs in its own Lua state (the mocks replace engine globals).
local results = { pass = 0, fail = 0 }
local outPrint = print
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  outPrint(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end
local function near(a, b, eps) return math.abs(a - b) <= (eps or 1e-6) end

local tmp = os.tmpname()

-- ---------------------------------------------------------------- HL.Target: raw (unclamped) point
do
  local savedRegister = registerForEvent
  registerForEvent = nil
  local M = dofile(UTPP_CORE_PATH)
  registerForEvent = savedRegister
  M.sink = function() end
  local HL = M.HeadLook
  local c = HL.New({ faceCamera = false })
  local obs = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = 0.2, z = 1.62 },
                camFwd = { x = -0.2, y = -0.98, z = 0 } }
  local t, mode, clamped, raw = HL.Target(c, obs)
  check("HL.Target: camera looking backwards -> clamped point (100 deg), unclamped raw point behind V",
    clamped == true and mode == "camera" and raw ~= nil and raw.y < 0 and t.y > raw.y, string.format("raw y %.2f clamped y %.2f", raw and raw.y or 0, t.y))
  local obs2 = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = -3, z = 1.9 }, camFwd = { x = 0, y = 1, z = 0 } }
  local t2, _, clamped2, raw2 = HL.Target(c, obs2)
  check("HL.Target: ahead -> not clamped, raw equals the target", clamped2 == false and near(raw2.x, t2.x, 1e-4) and near(raw2.y, t2.y, 1e-4)
    and near(raw2.z, t2.z, 1e-4))
  -- framing dead band (camera mode only): the resting camera pitch (-15 .. 0) leaves the head level, only the excess tilts it
  local function camPitchOf(camPitchDeg, mode)
    local cc = HL.New({ faceCamera = false })
    local p = math.rad(camPitchDeg)
    local o = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = -3, z = 1.9 },
                camFwd = { x = 0, y = math.cos(p), z = math.sin(p) } }
    local _, m, _, r = HL.Target(cc, o)
    -- pitch of the target ray from the camera (the target lies along the camera ray, so this is the effective camera pitch)
    return math.deg(math.atan2(r.z - o.camPos.z, math.sqrt((r.x - o.camPos.x) ^ 2 + (r.y - o.camPos.y) ^ 2))), m, r
  end
  local p15, m15, r15 = camPitchOf(-15)
  check("dead band: camera pitch -15 (the resting framing) -> a level target ray, mode camera, ahead of V",
    near(p15, 0, 1e-4) and m15 == "camera" and r15.y > 3, string.format("%.4f", p15))
  check("dead band: camera pitch 0 and -7 -> level", near(camPitchOf(0), 0, 1e-4) and near(camPitchOf(-7), 0, 1e-4))
  check("dead band: camera pitch -30 -> only the excess, -15", near(camPitchOf(-30), -15, 1e-3), string.format("%.3f", camPitchOf(-30)))
  check("dead band: camera pitch +10 -> +10 (looking up is not reduced)", near(camPitchOf(10), 10, 1e-3), string.format("%.3f", camPitchOf(10)))
  check("dead band: neutralPitch 0 in the config -> the plain camera pitch again", (function()
    local cc = HL.New({ faceCamera = false, neutralPitch = 0 })
    local p = math.rad(-15)
    local o = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = -3, z = 1.9 },
                camFwd = { x = 0, y = math.cos(p), z = math.sin(p) } }
    local _, _, _, r = HL.Target(cc, o)
    return math.deg(math.atan2(r.z - 1.9, math.sqrt(r.x * r.x + (r.y + 3) ^ 2))) < -14.9
  end)())
  do
    local cc = HL.New({ faceCamera = false })
    local o = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = -3, z = 1.9 }, camFwd = { x = -math.sin(math.rad(40)), y = math.cos(math.rad(40)), z = 0 } }
    local _, _, _, r = HL.Target(cc, o)
    check("dead band: the heading of the camera is kept (40 deg left of the body -> the target ray 40 deg left)",
      near(math.deg(math.atan2(-(r.x - 0), r.y + 3)), 40, 1e-3), string.format("%.3f", math.deg(math.atan2(-r.x, r.y + 3))))
  end
  do
    local cf = HL.New()
    local p = math.rad(-15)
    local o = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = 3, z = 1.7 }, camFwd = { x = 0, y = -math.cos(p), z = math.sin(p) } }
    local _, fm, _, fr = HL.Target(cf, o)
    check("dead band: face mode keeps the true target (the camera position)", fm == "face" and near(fr.y, 3) and near(fr.z, 1.7))
  end
  local cl = HL.New()
  local obs3 = { pos = { x = 0, y = 0, z = 0 }, fwd = { x = 0, y = 1 }, camPos = { x = 0, y = -3, z = 1.9 }, camFwd = { x = 0, y = 1, z = 0 },
                 loot = { x = 0.5, y = 1.0, z = 0.0 } }
  local _, lmode, _, lraw = HL.Target(cl, obs3)
  check("HL.Target: loot -> raw is the item (+0.1 m)", lmode == "loot" and near(lraw.x, 0.5) and near(lraw.y, 1.0) and near(lraw.z, 0.1))
end

-- ---------------------------------------------------------------- wiring with a mocked game
-- opts.native: "ok" (returns true), "cannot" (returns false), "missing" (no such function)
local function world(opts)
  local W = { applies = {}, clears = 0, observers = {}, logs = {}, calls = {}, statusCalls = 0 }
  local player = { pos = { x = 0, y = 0, z = 0 }, vel = { x = 0, y = 0, z = 0 }, cam = { x = 0.4, y = -3, z = 1.9 },
                   camFwd = { x = 0, y = 1, z = -0.1 } }
  local psmVals = { UpperBody = 0, Vehicle = 0, IsInWorkspot = 0, HighLevel = 1, Swimming = 0, LocomotionDetailed = 1,
                    Takedown = 0, Vision = 0 }
  local defs = {}
  for k in pairs(psmVals) do defs[k] = k end
  local P = {}
  function P:GetWorldPosition() return player.pos end
  function P:GetWorldForward() return { x = 0, y = 1, z = 0 } end
  function P:GetVelocity() return player.vel end
  function P:GetEntityID() return { hash = 1 } end
  function P:FindComponentByName() return nil end
  function P:GetFPPCameraComponent() return nil end
  function P:IsPlayer() return true end
  local headSys = {}
  function headSys:Apply(t, hw, cw, sp) W.applies[#W.applies + 1] = { x = t.x, y = t.y, z = t.z, chest = cw }; return true end
  function headSys:Clear() W.clears = W.clears + 1; return true end
  local loot = { id = { hash = 77 }, pos = { x = 0.3, y = 1.0, z = 0 } }
  function loot:GetEntityID() return self.id end
  function loot:GetWorldPosition() return self.pos end
  local handlers = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  registerInput = function() end
  ObserveAfter = function(cls, fn, cb) W.observers[cls .. "." .. fn] = cb end
  Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end }
  Transform = { new = function() return {} end }
  EnumInt = function(v) return v end
  GetAllBlackboardDefs = function() return { PlayerStateMachine = defs, UI_System = { IsInMenu = "IsInMenu" } } end
  local camSys = {}
  function camSys:GetActiveCameraWorldTransform() return true, { position = player.cam } end
  function camSys:GetActiveCameraForward() return player.camFwd end
  Game = {
    GetPlayer = function() return P end,
    UTPPLogLine = function(l) W.logs[#W.logs + 1] = l end,
    GetScriptableSystemsContainer = function() return { Get = function(_, n) if n == "UTPPHeadLook" then return headSys end return nil end } end,
    GetCameraSystem = function() return camSys end,
    GetBlackboardSystem = function() return {
      GetLocalInstanced = function() return { GetInt = function(_, k) return psmVals[k] end } end,
      Get = function() return { GetBool = function() return false end } end } end,
    GetSystemRequestsHandler = function() return { IsGamePaused = function() return false end } end,
    GetTargetingSystem = function() return { GetLookAtObject = function() return nil end } end,
    FindEntityByID = function(id) if id.hash == 77 then return loot end end,
    NameToString = function(n) return n end,
    UTPPSetMMEnabled = function() end, UTPPSetMMTrace = function() end,
    UTPPSetMMFootReplant = function() return true end, UTPPSetMMFeedDump = function() return true end,
    UTPPSetMMIdleHold = function() end, UTPPSetMMKneeFix = function() return true end, UTPPSetMMAir = function() return true end,
    UTPPSetMMAirProbe = function() end, UTPPSetMMAirSteer = function() end, UTPPSetMMAirRobust = function() return true end,
    UTPPSetMMClimbRelease = function() return true end,
  }
  if opts.native ~= "missing" then
    Game.UTPPSetHeadLook = function(target, weight, mode, chest, maxYaw)
      W.calls[#W.calls + 1] = { x = target.x, y = target.y, z = target.z, w = weight, mode = mode, chest = chest, maxYaw = maxYaw }
      local res = (opts.native == "ok")
      if W.nativeResult ~= nil then res = W.nativeResult end
      return res
    end
    Game.UTPPHeadLookStatus = function() W.statusCalls = W.statusCalls + 1; return "headlook rig=1 test" end
  end
  print = function() end
  GetMod = nil
  local savedRegister = nil
  local C = dofile(UTPP_CORE_PATH)
  C.SETTINGS_FILE = tmp
  os.remove(tmp)
  GetMod = function() return nil end
  handlers.onInit()
  local function frames(n) for _ = 1, n do handlers.onUpdate(1 / 30) end end
  local function lootOn() W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 0 }) end
  local function lootOff() W.observers["gameLootObject.OnInteractionActivated"](loot, { activator = P, layerData = { tag = "QualityRange_Short" }, eventType = 1 }) end
  local function logCount(text)
    local n = 0
    for _, l in ipairs(W.logs) do if l:find(text, 1, true) then n = n + 1 end end
    return n
  end
  return { W = W, C = C, player = player, psm = psmVals, frames = frames, handlers = handlers, lootOn = lootOn, lootOff = lootOff,
           logCount = logCount, headSys = headSys, shutdown = function() handlers.onShutdown() end }
end

local function weightCalls(w, on) local n = 0 for _, c in ipairs(w.calls) do if (c.w > 0) == on then n = n + 1 end end return n end

-- native available and acting
do
  local t = world({ native = "ok" })
  local W, C = t.W, t.C
  t.frames(8) -- 0.27 s: inside the engage delay
  check("native: no call while the engage delay runs, no game request", #W.calls == 0 and #W.applies == 0)
  t.frames(30)
  local n1 = #W.calls
  check("native: engaged -> one call per tick with weight 1, mode camera 0, chest 0.5, maxYaw 100", n1 >= 25 and weightCalls(W, true) == n1 and
    W.calls[n1].mode == 0 and near(W.calls[n1].chest, 0.5) and near(W.calls[n1].maxYaw, 100), tostring(n1))
  check("native: the DLL layer took the look -> no game look-at request", #W.applies == 0 and C.hl.nativeTook == true, tostring(#W.applies))
  -- raw target: camera at (0.4,-3,1.9) looking along +Y slightly down; the target is far ahead (> 4 m beyond), x about 0.4
  local last = W.calls[n1]
  check("native: target is a point ahead along the camera ray (raw world point)", last.y > 3 and near(last.x, 0.4, 0.05), string.format("%.2f %.2f %.2f", last.x, last.y, last.z))
  -- the target is sent every tick: it follows the camera at tick rate (the game request is throttled to 0.15 s / 0.2 m)
  local before = #W.calls
  t.player.camFwd = { x = 0.2, y = 0.98, z = -0.1 }
  t.frames(1)
  check("native: a camera move reaches the DLL on the very next tick", #W.calls == before + 1 and W.calls[#W.calls].x > last.x + 0.1)
  -- settings travel with the call
  C.Set("headLook.maxYaw", 80)
  C.Set("headLook.chest", 0.25)
  t.frames(1)
  check("native: maxYaw / chest settings are sent as they change", near(W.calls[#W.calls].maxYaw, 80) and near(W.calls[#W.calls].chest, 0.25))
  C.Set("headLook.maxYaw", 100)
  C.Set("headLook.chest", 0.5)
  -- loot mode
  t.lootOn()
  t.frames(30)
  local lc = W.calls[#W.calls]
  check("native: loot view target -> mode 2 with the item's point", lc.mode == 2 and near(lc.x, 0.3, 1e-3) and near(lc.y, 1.0, 1e-3), tostring(lc.mode))
  t.lootOff()
  t.frames(30)
  -- face mode: the camera in front of V, looking at her
  t.player.cam = { x = 0, y = 3, z = 1.7 }
  t.player.camFwd = { x = 0, y = -1, z = 0 }
  t.frames(30)
  local fc = W.calls[#W.calls]
  check("native: camera in front looking at V -> mode face 1 and the camera position as target", fc.mode == 1 and near(fc.y, 3, 1e-3), tostring(fc.mode))
  t.player.cam = { x = 0.4, y = -3, z = 1.9 }
  t.player.camFwd = { x = 0, y = 1, z = -0.1 }
  t.frames(30)
  -- the status line only while the diagnostic logs are on (project rule: diagnostics off by default), then about every 5 s
  check("native: no status line while the diagnostic logs are off (over 4 s of ticks before this point and the whole run so far)",
    W.statusCalls == 0 and t.logCount("head look status:") == 0, tostring(W.statusCalls))
  C.Set("core.diagnosticLogs", "until")
  t.frames(30 * 11)
  check("native: diagnostic logs on -> status line from Game.UTPPHeadLookStatus about every 5 s (2-3 lines in 11 s, not every tick)",
    W.statusCalls >= 2 and W.statusCalls <= 3 and t.logCount("head look status:") == W.statusCalls, tostring(W.statusCalls))
  C.Set("core.diagnosticLogs", "off")
  -- release by a block (aiming): exactly one weight 0 call, then silence; the game request is cleared too
  local calls0 = weightCalls(W, false)
  t.psm.UpperBody = 6
  t.frames(2)
  local after = #W.calls
  t.frames(10)
  check("native: aiming releases with exactly one weight 0 call, then no more calls", weightCalls(W, false) == calls0 + 1 and #W.calls == after
    and W.calls[#W.calls].w == 0 and C.hl.state == "off" and C.hl.nativeTook == false)
  t.psm.UpperBody = 0
  t.frames(30)
  check("native: engages again after the block", C.hl.state == "on" and W.calls[#W.calls].w == 1)
  -- running: the Lua gate releases (the DLL must not turn the head of a sprinting V)
  local rel = weightCalls(W, false)
  t.player.vel = { x = 0, y = 4, z = 0 }
  t.frames(2)
  check("native: running releases the layer (weight 0 once)", weightCalls(W, false) == rel + 1 and C.hl.state == "off")
  t.player.vel = { x = 0, y = 0, z = 0 }
  t.frames(28) -- 0.93 s: after a release by speed the look waits HL.MOVING_ENGAGE_DELAY (1.5 s) at walking speed or below
  check("native: stopped after running -> still released before the delay", C.hl.state == "off" and W.calls[#W.calls].w == 0)
  t.frames(32)
  check("native: stopped for 2 s -> engaged again with weight 1", C.hl.state == "on" and W.calls[#W.calls].w == 1)
  -- the setting off
  local rel2 = weightCalls(W, false)
  C.Set("headLook.impl", "off")
  t.frames(3)
  check("native: head look switched off -> one weight 0 call", weightCalls(W, false) == rel2 + 1 and C.hl.state == "off")
  check("native: no call after the release", (function() local k = #W.calls; t.frames(10); return #W.calls == k end)())
  C.Set("headLook.impl", "on")
  t.frames(30)
  t.shutdown()
  check("native: shutdown sends a last weight 0 call", W.calls[#W.calls].w == 0)
  check("native: the game look-at request was never used", #W.applies == 0)
end

-- the layer cannot act (rig not resolved / MM off): the game request is the fallback, and it is removed when the layer takes over
do
  local t = world({ native = "cannot" })
  local W, C = t.W, t.C
  t.frames(40)
  check("fallback: the layer cannot act -> the game look-at request is applied (once, the camera is still)",
    #W.applies == 1 and C.hl.nativeTook == false and C.hl.gameApplied == true and weightCalls(W, true) >= 25, tostring(#W.applies))
  check("fallback: logged once", t.logCount("head look layer cannot act") == 1, tostring(t.logCount("head look layer cannot act")))
  W.nativeResult = true -- MM comes on: the layer can act now
  local clears = W.clears
  t.frames(3)
  check("fallback: the layer takes over -> the game request is cleared once and not applied again", W.clears == clears + 1 and C.hl.nativeTook == true
    and C.hl.gameApplied == false and #W.applies == 1)
  W.nativeResult = false
  t.frames(3)
  check("fallback: the layer stops acting -> the game request is applied again", #W.applies == 2 and C.hl.gameApplied == true)
  t.shutdown()
end

-- older DLL: no such native
do
  local t = world({ native = "missing" })
  local W, C = t.W, t.C
  t.frames(40)
  check("missing native: logged once, session flag native = false", C.hl.native == false and t.logCount("UTPPSetHeadLook unavailable") == 1)
  check("missing native: the game look-at request works as before (one apply, no calls to the missing native)", #W.applies == 1 and #W.calls == 0)
  t.psm.UpperBody = 6
  t.frames(2)
  check("missing native: aiming -> Clear as before", W.clears == 1 and C.hl.state == "off")
  t.shutdown()
end

os.remove(tmp)
outPrint(string.format("utpp_headlook lua: %d passed, %d failed", results.pass, results.fail))
if results.fail > 0 then error(string.format("%d head look checks failed", results.fail)) end

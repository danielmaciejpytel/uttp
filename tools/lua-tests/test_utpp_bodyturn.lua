-- UTPP offline check of cet/utpp_bodyturn/init.lua: the body turn executor (adjustTransform request per CET frame, camera
-- hold, guards, aim align, weapon-turn block mask, CET wiring). Needs UTPP_BODYTURN_PATH.
local function norm(v)
  local n = (v + 180) % 360
  if n < 0 then n = n + 360 end
  return n - 180
end
local UTPP = dofile(UTPP_BODYTURN_PATH)
assert(type(UTPP) == "table" and type(UTPP.SetExecutor) == "function", "utpp_bodyturn init.lua must return the module table")

-- ---------------------------------------------------------------- 0.6.2: body turn executor
-- World per frame: CET ExecStep (reads body / camera yaw, asks the native command, sends a request) -> the next locomotion
-- update applies the requested yaw (lag 0) -> utpp_camera renders: free look held = camera keeps its world yaw, otherwise
-- (follow mode) camera = body yaw. Native command model: the matcher's target is camera + stick; step towards it at
-- 200 deg/s (animation-like), or a fixed raw step for the clamp test.
local X = { pass = 0, fail = 0 }
local function xcheck(name, ok, detail)
  if ok then X.pass = X.pass + 1 else X.fail = X.fail + 1 end
  print(string.format("[%s] executor: %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local function freshModule(smooth)
  local m = dofile(UTPP_BODYTURN_PATH)
  -- Historical executor/guard checks exercise the explicit legacy OFF path.
  -- The policy tests below pass true and exercise the production ON path.
  m.exec.aimSmooth = smooth == true
  m.lines = {}
  m.log = function(msg) m.lines[#m.lines + 1] = msg end
  return m
end

-- opts: stick, utppCamera (true = free-look hold available), locomotion (function(frame) -> bool), cmd (function(w, cur) -> yaw),
--       arm (value returned by arm; nil = true), dt
local function newWorld(opts)
  local w = { body = 0, cam = 0, free = false, stick = opts.stick or 90, frame = 0, pending = nil,
              calls = { arm = {}, native = {}, freeMove = {}, apply = {}, command = 0, hold = {} }, cmds = {} }
  w.api = {
    arm = function(on) w.calls.arm[#w.calls.arm + 1] = on; if opts.arm ~= nil then return opts.arm end return true end,
    setNative = function(on) w.calls.native[#w.calls.native + 1] = on; w.nativeOn = on end,
    locomotionEnable = function(on) w.calls.freeMove[#w.calls.freeMove + 1] = on end,
    locomotionReasons = function() return 0 end,
    locomotionActive = function() if opts.locomotion then return opts.locomotion(w.frame) end return true end,
    active = function() return w.nativeOn == true end,
    bodyYaw = function() return w.body end,
    cameraYaw = function() return w.cam end,
    command = function(cur)
      w.calls.command = w.calls.command + 1
      local c
      if opts.cmd then
        c = opts.cmd(w, cur)
      else
        local err = norm(w.cam + w.stick - cur)
        local step = math.min(200 * (opts.dt or 1 / 60), math.abs(err))
        c = norm(cur + (err >= 0 and step or -step))
      end
      w.cmds[#w.cmds + 1] = c
      return c
    end,
    apply = function(yaw) w.calls.apply[#w.calls.apply + 1] = yaw; w.pending = yaw; return true end,
    camHeld = function() if not opts.utppCamera then return nil end return w.free end,
    camHold = function(on)
      w.calls.hold[#w.calls.hold + 1] = on
      if opts.utppCamera then w.free = on end
      return opts.utppCamera == true
    end,
  }
  return w
end

local function stepWorld(m, w, dt)
  w.frame = w.frame + 1
  local why = m.ExecStep(w.api, dt)
  if w.pending ~= nil then w.body = w.pending; w.pending = nil end
  if not w.free then w.cam = w.body end -- follow mode: camera behind V
  return why
end

-- 1) decoupled camera (utpp_camera free-look hold): the mock executor applies exactly the commanded yaw, the body reaches
--    camera + stick, the camera stays world-fixed; Locomotion off later -> native off, hold released after the linger
do
  local m = freshModule()
  local dt = 1 / 60
  local w = newWorld({ stick = 90, utppCamera = true, locomotion = function(f) return f <= 120 end })
  local line = m.SelectExecutor(w.api)
  xcheck("selection line names the executor and the kill switch",
    line:find("body turn executor", 1, true) ~= nil and line:find("SetExecutor(false)", 1, true) ~= nil, line)
  xcheck("selection switches UTPP Locomotion on, native body turn off until Locomotion is active",
    w.calls.freeMove[1] == true and w.calls.native[1] == false and m.exec.mode == "executor")
  local exact, maxCam, maxStep = true, 0, 0
  for i = 1, 120 do
    local before = w.body
    local nA0 = #w.calls.apply
    stepWorld(m, w, dt)
    maxCam = math.max(maxCam, math.abs(norm(w.cam - 0)))
    maxStep = math.max(maxStep, math.abs(norm(w.body - before)))
    if #w.calls.apply > nA0 and math.abs(norm(w.calls.apply[#w.calls.apply] - w.cmds[#w.cmds])) > 1e-9 then exact = false end
  end
  xcheck("applied yaw == commanded yaw (below the clamp)", exact and #w.calls.apply > 20, #w.calls.apply)
  xcheck("body ends at camera + stick (90)", math.abs(norm(w.body - 90)) < 0.5, string.format("body %.2f", w.body))
  xcheck("camera world-fixed under the free-look hold", maxCam < 1e-9, string.format("max camera dev %.4f", maxCam))
  xcheck("step <= native model rate (200 deg/s)", maxStep <= 200 * dt + 1e-6, string.format("max step %.3f", maxStep))
  xcheck("hold engaged once and owned", m.exec.holdOwned == true and m.execStats.holds == 1)
  xcheck("native body turn on while Locomotion is active", w.nativeOn == true)
  local applies = #w.calls.apply
  for _ = 1, 20 do stepWorld(m, w, dt) end
  xcheck("Locomotion inactive -> native off, nothing applied", w.nativeOn == false and #w.calls.apply == applies, m.exec.why)
  xcheck("hold kept during the linger", m.exec.holdOwned == true)
  for _ = 1, 20 do stepWorld(m, w, dt) end
  xcheck("hold released after the linger (0.5 s)", m.exec.holdOwned == false and w.free == false
    and w.calls.hold[#w.calls.hold] == false)
  xcheck("no guard trip with a decoupled camera", m.exec.trips == 0 and #m.lines == 0)
  -- UTPPSetMMBodyTurn resets the native controller + latch: only selection (false), Locomotion on (true), Locomotion off (false)
  xcheck("native switch called only on change (3 calls in 160 frames)", #w.calls.native == 3
    and w.calls.native[2] == true and w.calls.native[3] == false, #w.calls.native)
end

-- 2) rate clamp: a command 170 deg away is limited to maxRate * dt (900 deg/s -> 15 deg at 60 fps); deadband
do
  local m = freshModule()
  local w = newWorld({ utppCamera = true, cmd = function(_, cur) return norm(cur + 170) end })
  m.SelectExecutor(w.api)
  stepWorld(m, w, 1 / 60)
  xcheck("clamped step = 15 deg", w.calls.apply[1] ~= nil and math.abs(w.calls.apply[1] - 15) < 1e-6
    and m.execStats.clamped == 1, tostring(w.calls.apply[1]))
  local m2 = freshModule()
  local w2 = newWorld({ utppCamera = true, cmd = function(_, cur) return norm(cur + 0.01) end })
  m2.SelectExecutor(w2.api)
  stepWorld(m2, w2, 1 / 60)
  xcheck("deadband: 0.01 deg is not sent", #w2.calls.apply == 0 and m2.execStats.deadband == 1)
end

-- 3) camera turning with the body (no utpp_camera hold): the guard pauses the executor before V spins
do
  local m = freshModule()
  local dt = 1 / 60
  local w = newWorld({ stick = 90, utppCamera = false })
  m.SelectExecutor(w.api)
  local turned, firstTrip = 0, nil
  for i = 1, 60 do
    local before = w.body
    stepWorld(m, w, dt)
    turned = turned + math.abs(norm(w.body - before))
    if m.exec.trips == 1 and not firstTrip then firstTrip = i end
  end
  xcheck("guard trips while the camera follows the body", m.exec.trips == 1 and firstTrip ~= nil, tostring(firstTrip))
  xcheck("turned < 90 deg before the pause (unguarded: endless turn)", turned < 90, string.format("%.1f deg", turned))
  xcheck("paused -> native off, one log line", w.nativeOn == false and #m.lines == 1
    and m.lines[1]:find("paused 5 s", 1, true) ~= nil, m.lines[1])
  local applies = #w.calls.apply
  for _ = 1, 200 do stepWorld(m, w, dt) end -- 3.3 s
  xcheck("nothing applied during the pause", #w.calls.apply == applies)
  for _ = 1, 1200 do stepWorld(m, w, dt) end -- retries after 5 s, trips again, third trip latches
  xcheck("third trip pauses for the session", m.exec.trips == 3 and m.exec.pausedUntil == math.huge and #m.lines == 3, m.lines[3])
  m.api = w.api
  m.SetExecutor(true) -- documented re-arm after the latch
  xcheck("SetExecutor(true) re-arms after the latch", m.exec.trips == 0 and m.exec.pausedUntil == nil and m.exec.mode == "executor")
  -- negative control: without the guard (limit unreachable) the body keeps turning with the camera
  local m3 = freshModule()
  m3.exec.coupledLimit = 1e9
  local w3 = newWorld({ stick = 90, utppCamera = false })
  m3.SelectExecutor(w3.api)
  local turned3 = 0
  for _ = 1, 120 do
    local before = w3.body
    stepWorld(m3, w3, dt)
    turned3 = turned3 + math.abs(norm(w3.body - before))
  end
  xcheck("negative control: guard off -> body spins (> 300 deg in 2 s)", turned3 > 300, string.format("%.1f deg", turned3))
end

-- 3b) batch 6 guardFix (male V test 23:42:57 'moonwalk'): the game's look follows V while the look is not written, so
--     the 0.6.3 guard tripped while V ran and the pause froze V's yaw for 5 s. Now: the guard reads the shown camera
--     (viewYaw) when available, skips the game's look while it is not written, never runs while V moves (> 1 m/s) and a
--     pause does not hold while V moves; the trip line carries bd / cd / score. Off = the 0.6.3 guard (section 3).
do
  local dt = 1 / 60
  local function turnFor(m, w, n)
    local turned = 0
    for _ = 1, n do local b = w.body; stepWorld(m, w, dt); turned = turned + math.abs(norm(w.body - b)) end
    return turned
  end
  -- a) running (3.5 m/s) with the camera following the body: no trip, V keeps turning with the movement
  local m = freshModule()
  local w = newWorld({ stick = 90, utppCamera = false })
  w.api.speed = function() return 3.5 end
  m.SelectExecutor(w.api)
  local turned = turnFor(m, w, 120)
  xcheck("guardFix: no trip while V runs (camera following the body), V keeps turning", m.exec.trips == 0 and turned > 300
    and m.exec.pausedUntil == nil and #m.lines == 0, string.format("%.1f deg trips %d", turned, m.exec.trips))
  xcheck("guardFix: Status() shows the guard source 'moving'", string.find(m.Status(), "guardFix=true guardSrc=moving", 1, true) ~= nil, m.Status())
  -- the same with guardFix off (the 0.6.3 guard): trips as in section 3
  local m0 = freshModule()
  m0.exec.guardFix = false
  local w0 = newWorld({ stick = 90, utppCamera = false })
  w0.api.speed = function() return 3.5 end
  m0.SelectExecutor(w0.api)
  turnFor(m0, w0, 120)
  xcheck("guardFix off: the old guard trips while running (negative control)", m0.exec.trips == 1
    and m0.lines[1]:find("paused 5 s (trip 1)", 1, true) ~= nil and m0.lines[1]:find("[bd=", 1, true) == nil, m0.lines[1])
  -- b) standing, game look not written (world camera failed): the look follows V -> the guard is skipped
  local m2 = freshModule()
  local w2 = newWorld({ stick = 90, utppCamera = false })
  w2.api.speed = function() return 0 end
  w2.api.lookWritten = function() return false end
  m2.SelectExecutor(w2.api)
  local turned2 = turnFor(m2, w2, 120)
  xcheck("guardFix: look not written -> guard skipped (src look-unwritten), no trip", m2.exec.trips == 0 and turned2 > 300
    and string.find(m2.Status(), "guardSrc=look-unwritten", 1, true) ~= nil, string.format("%.1f deg", turned2))
  -- c) standing, look written: the real coupling still trips, the line carries bd / cd / score / src / speed
  local m3 = freshModule()
  local w3 = newWorld({ stick = 90, utppCamera = false })
  w3.api.speed = function() return 0.2 end
  w3.api.lookWritten = function() return true end
  m3.SelectExecutor(w3.api)
  local turned3 = turnFor(m3, w3, 60)
  xcheck("guardFix: look written + camera follows V while standing -> trip as before, diagnostics appended", m3.exec.trips == 1
    and turned3 < 90 and m3.lines[1]:find("paused 5 s (trip 1) [bd=", 1, true) ~= nil and m3.lines[1]:find("src=look speed=0.2]", 1, true) ~= nil
    and m3.lines[1]:find("score=6", 1, true) ~= nil, m3.lines[1])
  -- the pause does not hold while V moves (yaw never frozen while running), holds again when V stands
  local applies = #w3.calls.apply
  turnFor(m3, w3, 30)
  xcheck("guardFix: paused while standing -> nothing applied", #w3.calls.apply == applies)
  w3.api.speed = function() return 3.0 end
  local turnedMoving = turnFor(m3, w3, 60)
  xcheck("guardFix: pause suspended while V moves - V turns again, one log line", #w3.calls.apply > applies and turnedMoving > 100
    and m3.exec.pausedUntil ~= nil and m3.exec.trips == 1 and #m3.lines == 2
    and m3.lines[2]:find("camera guard pause suspended: V moves 3.0 m/s", 1, true) ~= nil, m3.lines[2])
  w3.api.speed = function() return 0 end
  local applies2 = #w3.calls.apply
  turnFor(m3, w3, 30)
  xcheck("guardFix: V stands again -> the pause holds again (no new trip, nothing applied)", #w3.calls.apply == applies2
    and m3.exec.trips == 1 and #m3.lines == 2)
  turnFor(m3, w3, 600)
  xcheck("guardFix: pause over after 5 s -> executor back", m3.exec.pausedUntil == nil or m3.exec.pausedUntil == math.huge)
  -- d) the shown camera (utpp_camera world camera) keeps its own yaw while the game's look follows V -> no trip
  local m4 = freshModule()
  local w4 = newWorld({ stick = 90, utppCamera = false })
  w4.api.speed = function() return 0 end
  w4.api.viewYaw = function() return 0 end -- world camera: fixed in the world
  m4.SelectExecutor(w4.api)
  local turned4 = turnFor(m4, w4, 120)
  xcheck("guardFix: shown camera decoupled (viewYaw fixed) while the game look follows V -> no trip", m4.exec.trips == 0
    and turned4 > 300 and string.find(m4.Status(), "guardSrc=view", 1, true) ~= nil, string.format("%.1f deg", turned4))
  -- e) the shown camera turning with V (a real coupling on the world camera) still trips
  local m5 = freshModule()
  local w5 = newWorld({ stick = 90, utppCamera = false })
  w5.api.speed = function() return 0 end
  w5.api.viewYaw = function() return w5.body end
  m5.SelectExecutor(w5.api)
  local turned5 = turnFor(m5, w5, 60)
  xcheck("guardFix: shown camera turning with V -> trip (src=view)", m5.exec.trips == 1 and turned5 < 90
    and m5.lines[1]:find("src=view", 1, true) ~= nil, m5.lines[1])
  -- f) the same while V moves (3 m/s): src=view is not corrupted by the body-following look -> still trips
  local m6 = freshModule()
  local w6 = newWorld({ stick = 90, utppCamera = false })
  w6.api.speed = function() return 3.0 end
  w6.api.viewYaw = function() return w6.body end
  m6.SelectExecutor(w6.api)
  turnFor(m6, w6, 60)
  xcheck("guardFix: shown camera turning with V while moving -> still trips (src=view)", m6.exec.trips == 1
    and m6.lines[1] ~= nil and m6.lines[1]:find("src=view", 1, true) ~= nil, tostring(m6.lines[1]))
end

-- Camera-turn regression: current-frame guard source must not feed its own pause admission.
for _,fps in ipairs({30,60,144}) do
  local dt=1/fps
  local m=freshModule();local w=newWorld({utppCamera=true,dt=dt,cmd=function(x,cur)return norm(cur+2)end})
  w.api.speed=function()return 8.2 end
  w.api.viewYaw=function()return w.body end -- deliberate correlation: user pans while actor follows
  w.api.viewIndependent=function()return true end -- confirmed world camera + free-look owner
  m.SelectExecutor(w.api);m.exec.pausedUntil=math.huge;m.exec.viewSrc=true
  for i=1,fps*2 do stepWorld(m,w,dt) end
  xcheck("moving independent view: no pause flapping at "..fps, w.nativeOn==true and #w.calls.native==2 and #w.calls.apply==fps*2 and m.exec.trips==0, #w.calls.native)
  local n=#w.calls.native;w.api.speed=function()return 0 end
  for i=1,8 do stepWorld(m,w,dt) end
  xcheck("standing retains existing guard pause at "..fps,w.nativeOn==false and #w.calls.native==n+1,#w.calls.native)
  local m2=freshModule();local w2=newWorld({utppCamera=false,dt=dt})
  w2.api.speed=function()return 8.2 end;w2.api.viewYaw=function()return w2.body end
  m2.SelectExecutor(w2.api);m2.exec.pausedUntil=math.huge;m2.exec.viewSrc=true
  for i=1,fps do stepWorld(m2,w2,dt) end
  xcheck("untrusted moving view: real pause stays disabled at "..fps,w2.nativeOn==false and #w2.calls.native==1 and #w2.calls.apply==0,#w2.calls.native)
  -- Source availability changes without borrowing last frame's source to decide admission.
  w2.api.viewYaw=function()return nil end;w2.api.lookWritten=function()return false end
  for i=1,5 do stepWorld(m2,w2,dt) end
  local n2=#w2.calls.native
  w2.api.viewYaw=function()return w2.body end
  for i=1,5 do stepWorld(m2,w2,dt) end
  xcheck("view missing then restored: one transition each at "..fps,n2==2 and #w2.calls.native==3 and w2.nativeOn==false,#w2.calls.native)
end

-- 4) Locomotion never active: native body turn stays off, nothing commanded or applied
do
  local m = freshModule()
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  m.SelectExecutor(w.api)
  for _ = 1, 60 do stepWorld(m, w, 1 / 60) end
  xcheck("Locomotion inactive: no command, no apply, native off", w.calls.command == 0 and #w.calls.apply == 0
    and w.nativeOn == false and m.exec.why == "locomotion inactive", m.exec.why)
end

-- 5) utpp_bodyturn.reds not answering (no session / not installed): disabled as before
do
  local m = freshModule()
  local w = newWorld({ utppCamera = true })
  w.api.arm = function(on) w.calls.arm[#w.calls.arm + 1] = on; return nil end
  local line = m.SelectExecutor(w.api)
  for _ = 1, 30 do stepWorld(m, w, 1 / 60) end
  xcheck("executor missing: native never on, nothing applied", #w.calls.apply == 0 and w.nativeOn == false
    and m.exec.why == "executor missing" and m.execStats.armFails == 30, m.exec.why)
  xcheck("selection line says the executor does not answer yet", line:find("answers now: not yet", 1, true) ~= nil, line)
end

-- 6) kill switch: SetExecutor(false) while driving -> disarmed, Locomotion and hold restored; before selection -> disabled
do
  local m = freshModule()
  local w = newWorld({ stick = 120, utppCamera = true })
  m.api = w.api
  m.SelectExecutor(w.api)
  for _ = 1, 10 do stepWorld(m, w, 1 / 60) end
  local applies = #w.calls.apply
  xcheck("driving before the switch", applies > 0 and m.exec.holdOwned == true)
  m.SetExecutor(false)
  xcheck("switch off: arm(false), native off, Locomotion off, hold released", w.calls.arm[#w.calls.arm] == false
    and w.nativeOn == false and w.calls.freeMove[#w.calls.freeMove] == false and w.free == false and m.exec.mode == "disabled")
  xcheck("switch off: one log line", m.lines[#m.lines] ~= nil and m.lines[#m.lines]:find("switched off", 1, true) ~= nil,
    m.lines[#m.lines])
  local lastWhy
  for _ = 1, 30 do lastWhy = stepWorld(m, w, 1 / 60) end
  xcheck("switch off: nothing applied afterwards", #w.calls.apply == applies and lastWhy == "off")
  m.SetExecutor(true)
  xcheck("switch on again: executor reselected with one line", m.exec.mode == "executor"
    and m.lines[#m.lines]:find("body turn executor", 1, true) ~= nil)

  local m2 = freshModule()
  local w2 = newWorld({ utppCamera = true })
  m2.SetExecutor(false) -- switched off before selection
  local line = m2.SelectExecutor(w2.api)
  for _ = 1, 30 do stepWorld(m2, w2, 1 / 60) end
  xcheck("switch off before selection: UTPPSetMMBodyTurn(false), no executor", w2.calls.native[1] == false
    and #w2.calls.native == 1 and #w2.calls.arm == 0 and #w2.calls.apply == 0 and #w2.calls.freeMove == 0
    and m2.exec.mode == "disabled", line)
  xcheck("switch off before selection: disabled line",
    line:find("UTPP body turn disabled (UTPPSetMMBodyTurn(false)", 1, true) ~= nil, line)
end

-- 7) CET wiring (onInit / onUpdate / onShutdown) with a Game mock: the real
--    game api (GetPlayer():GetWorldForward() yaw convention, Game.UTPPBodyTurnExecRequest, utpp_camera FreeLook) drives V
local function wired(world, opts)
  opts = opts or {}
  local handlers, lines = {}, {}
  local savedR, savedG, savedGame, savedPrint = registerForEvent, GetMod, Game, print
  local gameCalls = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  GetMod = function(name)
    if name == "utpp_camera" then return world and world.camMod or nil end
    if name == "utpp_core" then return opts.core end
    if opts.mods and opts.mods[name] ~= nil then return opts.mods[name] end
    return nil
  end
  local function rec(name, ret) return function(...) gameCalls[name] = (gameCalls[name] or 0) + 1; return ret(...) end end
  Game = {
    UTPPLogLine = function() end,
    UTPPSetMMBodyTurn = rec("UTPPSetMMBodyTurn", function(on) if world then world.native = on end end),
    UTPPLocomotionSetEnabled = rec("UTPPLocomotionSetEnabled", function() end),
    UTPPLocomotionSetBlockMask = function(mask)
      gameCalls.masks = gameCalls.masks or {}
      gameCalls.masks[#gameCalls.masks + 1] = mask
    end,
    UTPPLocomotionActive = rec("UTPPLocomotionActive", function() return true end),
    UTPPLocomotionReasons = function() return 0 end,
    UTPPMMBodyTurnActive = rec("UTPPMMBodyTurnActive", function() return world ~= nil and world.native == true end),
    UTPPMMBodyYawCommand = rec("UTPPMMBodyYawCommand", function(cur, _)
      local err = norm(world.cam + world.stick - cur)
      local step = math.min(200 / 60, math.abs(err))
      return norm(cur + (err >= 0 and step or -step))
    end),
    UTPPBodyTurnExecArm = rec("UTPPBodyTurnExecArm", function() return true end),
    UTPPBodyTurnExecRequest = rec("UTPPBodyTurnExecRequest", function(yaw) world.pending = yaw; return true end),
    UTPPBodyTurnHeldCameraYaw = function() return world and world.cam or 1000 end,
    GetPlayer = function()
      if opts.noPlayer then return nil end
      return { GetEntityID = function() return 1 end, GetWorldForward = function()
        local r = math.rad(world.body)
        return { x = -math.sin(r), y = math.cos(r), z = 0 }
      end }
    end,
  }
  for k, v in pairs(opts.gameExtra or {}) do Game[k] = v end
  print = function(...)
    local t = {}
    for i = 1, select("#", ...) do t[#t + 1] = tostring(select(i, ...)) end
    lines[#lines + 1] = table.concat(t, " ")
  end
  local ok, mod = pcall(dofile, UTPP_BODYTURN_PATH)
  local err = nil
  if ok then
    local okI, eI = pcall(handlers.onInit)
    if not okI then err = eI end
    for _ = 1, 400 do
      local okU, eU = pcall(handlers.onUpdate, 1 / 60)
      if not okU then err = eU end
      if world and world.pending then world.body = world.pending; world.pending = nil end
      if world and not world.camMod.rt.freeHeld then world.cam = world.body end
    end
    if opts.inspect then
      local okInspect, inspectErr = pcall(opts.inspect, mod)
      if not okInspect then err = inspectErr end
    end
    if handlers.onShutdown then pcall(handlers.onShutdown) end
  else
    err = mod
  end
  registerForEvent, GetMod, Game, print = savedR, savedG, savedGame, savedPrint
  return mod, lines, gameCalls, err
end

do
  local world = { body = 30, cam = 30, stick = -90, native = nil, pending = nil }
  world.camMod = { rt = { freeHeld = false } }
  world.camMod.FreeLook = function(on) world.camMod.rt.freeHeld = on and true or false; return world.camMod.rt.freeHeld end
  local mod, lines, calls, err = wired(world)
  local selLines = 0
  for _, l in ipairs(lines) do
    if l:find("body turn executor:", 1, true) then selLines = selLines + 1 end
  end
  xcheck("wired: one selection line, no errors", err == nil and selLines == 1, tostring(err))
  xcheck("wired: requests sent through Game.UTPPBodyTurnExecRequest", (calls.UTPPBodyTurnExecRequest or 0) > 20,
    tostring(calls.UTPPBodyTurnExecRequest))
  xcheck("wired: body reaches camera + stick (yaw convention of GetWorldForward)",
    math.abs(norm(world.body - (-60))) < 0.5 and math.abs(norm(world.cam - 30)) < 1e-9,
    string.format("body %.2f cam %.2f", world.body, world.cam))
  xcheck("wired: onShutdown disarms (native off, hold released)", world.native == false
    and world.camMod.rt.freeHeld == false and mod.exec.mode == "disabled")
end

-- 8) 0.6.3 body turn with a weapon: setting bodyturn.weaponTurn -> Game.UTPPLocomotionSetBlockMask(0x5FBF on / 0x7FBF off)
local function maskApi(w, hasPlayer)
  w.masks, w.hasPlayer = {}, hasPlayer ~= false
  w.api.hasPlayer = function() return w.hasPlayer end
  w.api.setBlockMask = function(mask) w.masks[#w.masks + 1] = mask end
end
local function maskLines(m)
  local n = 0
  for _, l in ipairs(m.lines) do if l:find("Locomotion block mask", 1, true) then n = n + 1 end end
  return n
end

do -- registration: key, type, default, section, label; apply routes into SetWeaponTurn
  local m = freshModule()
  local regs, sections = {}, {}
  local core = { RegisterSection = function(id, label) sections[#sections + 1] = { id, label } end,
                 Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end }
  m.RegisterSettings(core)
  local d = regs[1]
  xcheck("weaponTurn setting: key / type / default / section", #regs == 12 and d.key == "bodyturn.weaponTurn" and d.type == "bool"
    and d.default == true and d.section == "bodyturn" and sections[1][1] == "bodyturn", d and d.key)
  xcheck("smooth aim: production and registered defaults ON", UTPP.exec.aimSmooth == true and regs[10].key == "bodyturn.aimSmooth"
    and regs[10].default == true and m.exec.aimSmooth == true)
  regs[10].apply(false)
  xcheck("smooth aim: setting OFF restores the legacy policy", m.exec.aimSmooth == false)
  -- 2026-10-12 turn clip rate settings (natives from UTPP.dll 0.6.4; absent in this offline Game mock -> logged once, inert)
  local t1, t2 = regs[2], regs[3]
  xcheck("turnRate settings: keys / types / defaults", t1 and t1.key == "bodyturn.turnRate" and t1.type == "bool"
    and t1.default == true and t2 and t2.key == "bodyturn.turnRateMax" and t2.type == "float" and t2.default == 1.5
    and t2.min == 1.0 and t2.max == 2.0, t1 and t1.key, t2 and t2.key)
  xcheck("turnRate settings: natives missing -> inert, state kept", m.turnRate.on == true and m.turnRate.max == 1.5
    and m.turnRate.missing == true and m.turnRate.sent == nil)
  t2.apply(1.6)
  xcheck("turnRate settings: apply(1.6) updates the module state", m.turnRate.max == 1.6)
  xcheck("turnRate settings: Status() shows turnRate", m.Status():find("turnRate=true max=1.60 sent=missing", 1, true) ~= nil,
    m.Status())
  -- 2026-10-13 B2 aim strafe settings (natives absent in this mock -> logged once, inert)
  local s1, s2 = regs[4], regs[5]
  xcheck("aimStrafe settings: keys / types / defaults (aimStrafe ON, aimStrafeUnarmed ON: user defaults 2026-10-20)", s1 and s1.key == "bodyturn.aimStrafe"
    and s1.type == "bool" and s1.default == true and s1.section == "bodyturn" and s2 and s2.key == "bodyturn.aimStrafeUnarmed"
    and s2.type == "bool" and s2.default == true and s2.section == "bodyturn", s1 and s1.key, s2 and s2.key)
  local missingLines = 0
  for _, l in ipairs(m.lines) do if l:find("aim strafe natives missing", 1, true) then missingLines = missingLines + 1 end end
  xcheck("aimStrafe settings: natives missing -> logged once, inert, state kept", m.strafe.on == true and m.strafe.unarmed == true
    and m.strafe.missing == true and m.strafe.sent == nil and missingLines == 1, missingLines)
  s2.apply(false)
  local offOk = m.strafe.unarmed == false
  s2.apply(true)
  missingLines = 0
  for _, l in ipairs(m.lines) do if l:find("aim strafe natives missing", 1, true) then missingLines = missingLines + 1 end end
  xcheck("aimStrafe settings: apply(true) on the unarmed switch updates the state, no second missing line",
    offOk and m.strafe.unarmed == true and missingLines == 1, missingLines)
  xcheck("aimStrafe settings: Status() shows aimStrafe", m.Status():find("aimStrafe=true unarmed=true sent=missing", 1, true) ~= nil,
    m.Status())
  -- 2026-10-13 B2 aim strafe S2 chest align setting (native absent in this mock -> logged once, inert)
  local c1 = regs[6]
  xcheck("aimChestAlign setting: key / type / default ON / section", c1 and c1.key == "bodyturn.aimChestAlign" and c1.type == "bool"
    and c1.default == true and c1.section == "bodyturn", c1 and c1.key)
  local chestMissing = 0
  for _, l in ipairs(m.lines) do if l:find("chest align native missing", 1, true) then chestMissing = chestMissing + 1 end end
  xcheck("aimChestAlign setting: native missing -> logged once, inert, state kept", m.chest.on == true and m.chest.missing == true
    and m.chest.sent == nil and chestMissing == 1, chestMissing)
  c1.apply(false)
  chestMissing = 0
  for _, l in ipairs(m.lines) do if l:find("chest align native missing", 1, true) then chestMissing = chestMissing + 1 end end
  xcheck("aimChestAlign setting: apply(false) updates the state, no second missing line, Status() shows it",
    m.chest.on == false and chestMissing == 1 and m.Status():find("aimChestAlign=false sent=missing calls=0 fails=0", 1, true) ~= nil,
    m.Status())
  -- 2026-10-13 B2 aim strafe S4 jog settings (natives absent in this mock -> inert, state kept, no second missing line)
  local j1, j2 = regs[7], regs[8]
  xcheck("aimJog settings: keys / types / defaults (aimJog ON by default, aimJogSpeed 3.3 in 3.3..3.85)", j1 and j1.key == "bodyturn.aimJog"
    and j1.type == "bool" and j1.default == true and j1.section == "bodyturn" and j2 and j2.key == "bodyturn.aimJogSpeed"
    and j2.type == "float" and j2.default == 3.3 and j2.min == 3.3 and j2.max == 3.85 and j2.section == "bodyturn",
    j1 and j1.key, j2 and j2.key)
  j2.apply(3.5)
  j1.apply(false)
  missingLines = 0
  for _, l in ipairs(m.lines) do if l:find("aim strafe natives missing", 1, true) then missingLines = missingLines + 1 end end
  xcheck("aimJog settings: natives missing -> state kept (jog off, speed 3.5), jogF 0, no second missing line, Status() shows aimJog",
    m.strafe.jog == false and m.strafe.jogSpeed == 3.5 and m.strafe.jogF == 0 and missingLines == 1
    and m.Status():find("aimJog=false jogF=0.00", 1, true) ~= nil, m.Status())
  xcheck("weaponTurn setting: label and description", d.label == "Body turn with a weapon in hand (Locomotion)"
    and d.desc:find("aiming still blocks", 1, true) ~= nil, d.label)
  xcheck("weaponTurn setting: default ON (module state and applied default)", m.weapon.on == true)
  d.apply(false)
  xcheck("weaponTurn setting: apply(false) switches it off", m.weapon.on == false)
  d.apply(true)
  xcheck("weaponTurn setting: apply(true) switches it on", m.weapon.on == true)
end

do -- mask values, once-per-change log, live switching (default ON)
  local m = freshModule()
  local w = newWorld({ utppCamera = true })
  maskApi(w)
  m.api = w.api
  m.SelectExecutor(w.api)
  xcheck("init (executor selected, fresh profile = default on): 0x5FBF sent once, logged", #w.masks == 1 and w.masks[1] == 0x5FBF
    and w.masks[1] == 0x7FBF - 8192 and w.masks[1] % 8192 >= 4096 and maskLines(m) == 1
    and m.lines[#m.lines] == "[utpp_bodyturn] Locomotion block mask -> 0x5FBF (weapon turn on)", m.lines[#m.lines])
  m.SetWeaponTurn(true)
  xcheck("same value again: mask re-sent but no second log line", #w.masks == 2 and w.masks[2] == 0x5FBF and maskLines(m) == 1)
  for _ = 1, 30 do stepWorld(m, w, 1 / 60) end
  xcheck("ExecStep does not spam the mask", #w.masks == 2 and maskLines(m) == 1, #w.masks)
  m.SetWeaponTurn(false)
  xcheck("weapon turn off: 0x7FBF sent and logged", w.masks[#w.masks] == 0x7FBF and #w.masks == 3
    and m.lines[#m.lines] == "[utpp_bodyturn] Locomotion block mask -> 0x7FBF (weapon turn off)", m.lines[#m.lines])
  xcheck("Status shows weaponTurn=false", m.Status():find("weaponTurn=false mask=0x7FBF", 1, true) ~= nil)
  m.SetWeaponTurn(true)
  xcheck("weapon turn on again: 0x5FBF, logged", w.masks[#w.masks] == 0x5FBF and #w.masks == 4 and maskLines(m) == 3
    and m.lines[#m.lines] == "[utpp_bodyturn] Locomotion block mask -> 0x5FBF (weapon turn on)", m.lines[#m.lines])
  xcheck("Status shows weaponTurn=true", m.Status():find("weaponTurn=true mask=0x5FBF", 1, true) ~= nil)
  xcheck("SetWeaponTurn(nil) is off", m.SetWeaponTurn(nil) == false and w.masks[#w.masks] == 0x7FBF)

  -- setting stored as off: the init path sends the stock mask, once, logged "off"
  local m2 = freshModule()
  local w2 = newWorld({ utppCamera = true })
  maskApi(w2)
  m2.api = w2.api
  local core = { RegisterSection = function() end, Register = function(def) def.apply(false) end }
  m2.RegisterSettings(core) -- before the executor is selected: deferred, nothing sent
  xcheck("setting off before selection: nothing sent yet", #w2.masks == 0)
  m2.SelectExecutor(w2.api)
  xcheck("setting off: init sends 0x7FBF once, logged (weapon turn off)", #w2.masks == 1 and w2.masks[1] == 0x7FBF
    and maskLines(m2) == 1 and m2.lines[#m2.lines] == "[utpp_bodyturn] Locomotion block mask -> 0x7FBF (weapon turn off)",
    m2.lines[#m2.lines])
  -- fresh profile through the core: registration (apply default true) before the selection -> init sends 0x5FBF once
  local m3 = freshModule()
  local w3 = newWorld({ utppCamera = true })
  maskApi(w3)
  m3.api = w3.api
  m3.RegisterSettings({ RegisterSection = function() end, Register = function(def) def.apply(def.default) end })
  m3.SelectExecutor(w3.api)
  xcheck("fresh profile via core: init sends 0x5FBF once, logged (weapon turn on)", #w3.masks == 1 and w3.masks[1] == 0x5FBF
    and maskLines(m3) == 1 and m3.lines[#m3.lines] == "[utpp_bodyturn] Locomotion block mask -> 0x5FBF (weapon turn on)")
end

do -- no player: no call; sent by ExecStep when the player appears; failures retried after 2 s
  local m = freshModule()
  local w = newWorld({ utppCamera = true })
  maskApi(w, false)
  m.api = w.api
  m.SelectExecutor(w.api)
  m.SetWeaponTurn(true)
  xcheck("no player: no native call, no log", #w.masks == 0 and maskLines(m) == 0)
  for _ = 1, 10 do stepWorld(m, w, 1 / 60) end
  xcheck("no player: still nothing after frames", #w.masks == 0 and maskLines(m) == 0)
  w.hasPlayer = true
  stepWorld(m, w, 1 / 60)
  xcheck("player appears: ExecStep sends 0x5FBF once", #w.masks == 1 and w.masks[1] == 0x5FBF and maskLines(m) == 1)
  -- new session: onUpdate marks the mask dirty (no player), the next ExecStep with a player sends it again
  m.weapon.dirty, m.weapon.sent = true, nil
  stepWorld(m, w, 1 / 60)
  xcheck("new session: mask re-sent, no duplicate log", #w.masks == 2 and w.masks[2] == 0x5FBF and maskLines(m) == 1)
  -- native call failing (function missing): retried every 2 s, no log
  local m2 = freshModule()
  local w2 = newWorld({ utppCamera = true })
  maskApi(w2)
  local fail = true
  w2.api.setBlockMask = function(mask) if fail then error("native missing") end w2.masks[#w2.masks + 1] = mask end
  m2.api = w2.api
  m2.SelectExecutor(w2.api)
  for _ = 1, 60 do stepWorld(m2, w2, 1 / 60) end
  xcheck("native failing: no log, counted", #w2.masks == 0 and maskLines(m2) == 0 and m2.weapon.fails >= 1, m2.weapon.fails)
  fail = false
  for _ = 1, 120 do stepWorld(m2, w2, 1 / 60) end
  xcheck("native back: retried and logged once", #w2.masks == 1 and w2.masks[1] == 0x5FBF and maskLines(m2) == 1, #w2.masks)
end

do -- re-apply after Locomotion is (re)enabled
  local m = freshModule()
  local w = newWorld({ utppCamera = true, locomotion = function(f) return not (f >= 20 and f < 40) end })
  maskApi(w)
  m.api = w.api
  m.SelectExecutor(w.api)
  local n0 = #w.masks
  for _ = 1, 19 do stepWorld(m, w, 1 / 60) end
  xcheck("steady Locomotion: no re-send", #w.masks == n0, #w.masks)
  for _ = 1, 30 do stepWorld(m, w, 1 / 60) end -- inactive 20..39, active again at 40
  xcheck("Locomotion active again: mask re-sent (0x5FBF), log not repeated", #w.masks == n0 + 1 and w.masks[#w.masks] == 0x5FBF
    and maskLines(m) == 1, #w.masks)
  local n1 = #w.masks
  m.SetExecutor(false)
  xcheck("kill switch: stock mask restored silently", w.masks[#w.masks] == 0x7FBF and #w.masks == n1 + 1 and m.exec.mode == "disabled"
    and maskLines(m) == 1)
  m.SetWeaponTurn(true)
  xcheck("executor off: SetWeaponTurn sends nothing", #w.masks == n1 + 1)
  local linesBefore = maskLines(m)
  m.SetExecutor(true)
  xcheck("SetExecutor(true) (Locomotion enabled again): 0x5FBF sent again, logged as a change", w.masks[#w.masks] == 0x5FBF
    and #w.masks == n1 + 2 and maskLines(m) == linesBefore + 1, #w.masks)
end

do -- executor not selected: the mask is never touched
  local m = freshModule()
  local w = newWorld({})
  maskApi(w)
  m.api = w.api
  m.exec.mode = "disabled"
  m.SetWeaponTurn(true)
  xcheck("executor not selected: mask untouched", #w.masks == 0 and maskLines(m) == 0)
end

-- 9) CET wiring: utpp_core mock registers the setting; Game.UTPPLocomotionSetBlockMask is called with the right mask
do
  local function newWiredWorld()
    local world = { body = 30, cam = 30, stick = -90 }
    world.camMod = { rt = { freeHeld = false } }
    world.camMod.FreeLook = function(on) world.camMod.rt.freeHeld = on and true or false; return world.camMod.rt.freeHeld end
    return world
  end
  local regs = {}
  local core = { RegisterSection = function() end, Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end }
  local _, _, calls, err = wired(newWiredWorld(), { core = core })
  xcheck("wired: setting registered at onInit, default ON", err == nil and #regs == 12 and regs[1].key == "bodyturn.weaponTurn"
    and regs[1].default == true, tostring(err))
  xcheck("wired: bodyturn.guardFix registered, default on", regs[9] ~= nil and regs[9].key == "bodyturn.guardFix" and regs[9].default == true)
  xcheck("wired: fresh profile -> 0x5FBF sent once to the game after the executor was selected; onShutdown restores 0x7FBF",
    calls.masks ~= nil and #calls.masks == 2 and calls.masks[1] == 0x5FBF and calls.masks[2] == 0x7FBF,
    calls.masks and table.concat(calls.masks, ","))
  local core2 = { RegisterSection = function() end, Register = function(def) def.apply(false) end }
  local _, _, calls2, err2 = wired(newWiredWorld(), { core = core2 })
  xcheck("wired: stored weaponTurn=false -> stock 0x7FBF only (nothing to restore at shutdown)",
    err2 == nil and calls2.masks ~= nil and #calls2.masks == 1 and calls2.masks[1] == 0x7FBF,
    calls2.masks and table.concat(calls2.masks, ","))
  local _, _, calls3, err3 = wired(newWiredWorld(), { core = core, noPlayer = true })
  xcheck("wired: no player -> Game.UTPPLocomotionSetBlockMask never called", err3 == nil and calls3.masks == nil, tostring(err3))
end

do -- 2026-10-12 aim align: V turns onto the game's look while aiming (Locomotion blocked only by the aim bit 4096)
  local m = freshModule()
  local reasons = 4096
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return reasons end
  local holds = {}
  w.api.alignHold = function(on) holds[#holds + 1] = on; return true end
  m.SelectExecutor(w.api)
  w.body, w.cam, w.free = -16, 166, true -- look 182 deg away from V (user test 21:34)
  local function run(n)
    local why
    for _ = 1, n do
      w.frame = w.frame + 1
      why = m.ExecStep(w.api, 1 / 60)
      if w.pending ~= nil then w.body = w.pending; w.pending = nil end
    end
    return why
  end
  local why = run(10)
  xcheck("aim align: turning toward the look at <= 1080 deg/s (182 deg in 10 frames at 60 fps: reached or within 18)",
    math.abs(norm(w.body - (-16))) > 150 and math.abs(norm(w.body - (-16))) <= 182 + 1e-6, why .. " body " .. w.body)
  why = run(30)
  xcheck("aim align: reaches the look within 0.5 s, then idles (aligned)", math.abs(norm(w.cam - w.body)) < 2
    and why == "aim aligned", why .. " body " .. w.body)
  xcheck("aim align: FPP heading lock (utpp_camera AlignHold) on while turning, off once aligned",
    #holds == 2 and holds[1] == true and holds[2] == false, table.concat((function() local t = {} for i, v in ipairs(holds) do t[i] = tostring(v) end return t end)(), ","))
  -- not aiming (other block reason, e.g. vehicle 4 / not TPP 65536): nothing is applied
  local m2 = freshModule()
  local w2 = newWorld({ utppCamera = true, locomotion = function() return false end })
  w2.api.locomotionReasons = function() return 4096 + 65536 end
  m2.SelectExecutor(w2.api)
  w2.body, w2.cam = 0, 120
  local n0 = #w2.calls.apply
  w2.frame = 1
  local why2 = m2.ExecStep(w2.api, 1 / 60)
  xcheck("aim align: not in third person (bit 65536) -> no align", why2 == "locomotion inactive" and #w2.calls.apply == n0, why2)
  w2.api.locomotionReasons = function() return 4 end
  why2 = m2.ExecStep(w2.api, 1 / 60)
  xcheck("aim align: Locomotion blocked by something else -> no align", why2 == "locomotion inactive" and #w2.calls.apply == n0, why2)
  -- spin guard: the look turns with V -> stopped after 540 deg, logged once
  local m3 = freshModule()
  local logs = {}
  m3.log = function(msg) logs[#logs + 1] = msg end
  local w3 = newWorld({ utppCamera = true, locomotion = function() return false end })
  w3.api.locomotionReasons = function() return 4096 end
  m3.SelectExecutor(w3.api)
  w3.body, w3.cam = 0, 170
  local why3
  for _ = 1, 120 do
    w3.frame = w3.frame + 1
    why3 = m3.ExecStep(w3.api, 1 / 60)
    if w3.pending ~= nil then local d = norm(w3.pending - w3.body); w3.body = w3.pending; w3.cam = norm(w3.cam + d); w3.pending = nil end
  end
  local stopped = 0
  for _, l in ipairs(logs) do if l:find("aim align STOPPED", 1, true) then stopped = stopped + 1 end end
  xcheck("aim align: spin guard stops after 540 deg (camera turns with V), logged once", why3 == "aim align stopped" and stopped == 1,
    tostring(why3) .. " logs " .. stopped)
  -- aim ends (Locomotion active again): the guard resets for the next aim
  w3.api.locomotionReasons = function() return 0 end
  w3.frame = w3.frame + 1
  m3.ExecStep(w3.api, 1 / 60)
  xcheck("aim align: guard reset after the aim", (m3.exec.aimAlignSum or 0) == 0 and m3.exec.aimAlignStopped == false)
end

do -- 2026-10-12 round 3: utpp_camera reports the game look sync -> no spin guard (fast mouse turns > 540 deg keep aligning)
  local m = freshModule()
  local logs = {}
  m.log = function(msg) logs[#logs + 1] = msg end
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return 4096 end
  w.api.lookSynced = function() return true end
  m.SelectExecutor(w.api)
  w.body, w.cam = 0, 170
  local why
  for _ = 1, 120 do
    w.frame = w.frame + 1
    w.cam = norm(w.cam + 10) -- the user turns the look 600 deg/s
    why = m.ExecStep(w.api, 1 / 60)
    if w.pending ~= nil then w.body = w.pending; w.pending = nil end
  end
  local stopped = 0
  for _, l in ipairs(logs) do if l:find("aim align STOPPED", 1, true) then stopped = stopped + 1 end end
  xcheck("aim align: look synced -> no spin guard after 1200 deg of turning, still aligning", stopped == 0
    and (why == "aim align" or why == "aim aligned"), tostring(why))
end

do -- 2026-10-12 23:27: fast mouse turns while aiming (not synced): V keeps up, the guard restarts whenever V gets close
  local m = freshModule()
  local logs = {}
  m.log = function(msg) logs[#logs + 1] = msg end
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return 4096 end
  m.SelectExecutor(w.api)
  w.body, w.cam = 0, 0
  for k = 1, 600 do -- 10 s of flicks: 90 deg in 0.1 s, then 0.4 s still
    w.frame = w.frame + 1
    if k % 30 < 6 then w.cam = norm(w.cam + 15) end
    m.ExecStep(w.api, 1 / 60)
    if w.pending ~= nil then w.body = w.pending; w.pending = nil end
  end
  local stopped = 0
  for _, l in ipairs(logs) do if l:find("aim align STOPPED", 1, true) then stopped = stopped + 1 end end
  xcheck("aim align: 1800 deg of flicks in one aim -> no spin guard stop, V on the camera at the end",
    stopped == 0 and math.abs(norm(w.cam - w.body)) < 2, "stops " .. stopped .. " err " .. norm(w.cam - w.body))
end

do -- 0.6.5b: the game look write gets V's exact turn of the coming tick (api.lookStep after every SENT request)
  -- aim align (applyAim): one lookStep per sent request with exactly the requested step, none once aligned / rejected
  local m = freshModule()
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return 4096 end
  w.api.alignHold = function() return true end
  local looks = {}
  w.api.applyAim = function(yaw) w.calls.apply[#w.calls.apply + 1] = yaw; w.pending = yaw; return true end
  w.api.lookStep = function(step) looks[#looks + 1] = step; return true end
  m.SelectExecutor(w.api)
  w.body, w.cam, w.free = 0, 100, true
  local exact, n = true, 0
  for _ = 1, 20 do
    w.frame = w.frame + 1
    local before, nL = w.body, #looks
    m.ExecStep(w.api, 1 / 60)
    if w.pending ~= nil then
      n = n + 1
      if #looks ~= nL + 1 or math.abs(looks[#looks] - norm(w.pending - before)) > 1e-9 then exact = false end
      w.body, w.pending = w.pending, nil
    elseif #looks ~= nL then
      exact = false
    end
  end
  xcheck("0.6.5b aim align: lookStep after every sent applyAim with the exact step (18 deg at 60 fps), none once aligned",
    exact and n >= 5 and #looks == n and math.abs(looks[1] - 18) < 1e-9 and m.exec.why == "aim aligned"
    and math.abs(m.execStats.lastAimStep or 0) > 0, string.format("n %d looks %d first %s why %s", n, #looks,
    tostring(looks[1]), tostring(m.exec.why)))
  w.cam = norm(w.body + 90)
  w.api.applyAim = function() return false end
  local nL = #looks
  w.frame = w.frame + 1
  local whyR = m.ExecStep(w.api, 1 / 60)
  xcheck("0.6.5b aim align: rejected request -> no lookStep", whyR == "aim align rejected" and #looks == nL, whyR)
  -- round 2: execStats.adjSeq counts every SENT adjustTransform request (utpp_camera marks those frames: adjustPending)
  xcheck("0.6.5b round 2 aim align: adjSeq = sent applyAim requests (not the rejected one), lastAdjStep = the last step",
    m.execStats.adjSeq == n and math.abs(m.execStats.lastAdjStep - looks[#looks]) < 1e-9,
    string.format("adjSeq %s n %d last %s", tostring(m.execStats.adjSeq), n, tostring(m.execStats.lastAdjStep)))
  -- Locomotion (applied): lookStep with the same step as bodyStep after every sent request
  local m2 = freshModule()
  local w2 = newWorld({ stick = 90, utppCamera = true })
  local looks2, bodies2 = {}, {}
  w2.api.lookStep = function(s) looks2[#looks2 + 1] = s; return true end
  w2.api.bodyStep = function(s) bodies2[#bodies2 + 1] = s; return true end
  m2.SelectExecutor(w2.api)
  for _ = 1, 30 do stepWorld(m2, w2, 1 / 60) end
  local same = #looks2 > 5 and #looks2 == #bodies2 and #looks2 == m2.execStats.applied
  for i = 1, #looks2 do if looks2[i] ~= bodies2[i] then same = false end end
  xcheck("0.6.5b Locomotion applied: lookStep with the same step as bodyStep after every sent request", same,
    string.format("look %d body %d applied %d", #looks2, #bodies2, m2.execStats.applied))
  xcheck("0.6.5b round 2 Locomotion applied: adjSeq = applied requests, lastAdjStep = the last step",
    m2.execStats.adjSeq == m2.execStats.applied and m2.execStats.lastAdjStep == looks2[#looks2],
    string.format("adjSeq %s applied %d", tostring(m2.execStats.adjSeq), m2.execStats.applied))
  -- without the hook (older utpp_camera / no camera): nothing breaks
  local m3 = freshModule()
  local w3 = newWorld({ stick = 90, utppCamera = true })
  m3.SelectExecutor(w3.api)
  local okRun = pcall(function() for _ = 1, 30 do stepWorld(m3, w3, 1 / 60) end end)
  xcheck("0.6.5b no lookStep in the api: the executor runs as before", okRun and m3.execStats.applied > 5)
end

do -- 2026-10-13 aim align hysteresis: starts beyond 12 deg, then aligns down to 2 deg; small camera moves do not turn V
  local m = freshModule()
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return 4096 end
  m.SelectExecutor(w.api)
  w.body, w.cam = 0, 8
  local n0 = #w.calls.apply
  for _ = 1, 30 do w.frame = w.frame + 1; m.ExecStep(w.api, 1 / 60); if w.pending ~= nil then w.body = w.pending; w.pending = nil end end
  xcheck("aim align hysteresis: camera 8 deg away -> V not turned (no adjustTransform)", #w.calls.apply == n0 and w.body == 0, w.body)
  w.cam = 20
  for _ = 1, 30 do w.frame = w.frame + 1; m.ExecStep(w.api, 1 / 60); if w.pending ~= nil then w.body = w.pending; w.pending = nil end end
  xcheck("aim align hysteresis: 20 deg -> V turned to within 2 deg", math.abs(norm(w.cam - w.body)) < 2, w.body)
  local n1 = #w.calls.apply
  w.cam = norm(w.body + 9)
  for _ = 1, 30 do w.frame = w.frame + 1; m.ExecStep(w.api, 1 / 60); if w.pending ~= nil then w.body = w.pending; w.pending = nil end end
  xcheck("aim align hysteresis: after aligning, 9 deg more -> no new turn", #w.calls.apply == n1, #w.calls.apply - n1)
end

do -- vehicle batch (CAM-9 hand-over): the aim align waits 0.3 s after utpp_camera's vehicle hold ends
  local function run(smooth, held)
    local m = freshModule(smooth)
    local w = newWorld({ utppCamera = true, locomotion = function() return false end })
    w.api.locomotionReasons = function() return 4096 end
    w.api.alignHold = function() return true end
    w.api.aimSignals = function() return { weaponDrawn = true, upper = 6, action = 0 } end
    if held ~= "none" then w.api.vehicleHeld = function() return w.vh end end
    if held ~= "none" then w.vh = held end
    m.SelectExecutor(w.api)
    w.body, w.cam = 0, (held == true) and 0 or 90 -- hold test: aligned while held, the camera swings 90 deg at the hold end
    local function step() w.frame = w.frame + 1; local why = m.ExecStep(w.api, 1 / 60)
      if w.pending ~= nil then w.body = w.pending; w.pending = nil end; return why end
    return m, w, step
  end
  for _, smooth in ipairs({ false, true }) do
    local tag = smooth and "smooth" or "legacy"
    -- still held: the executor does not start the settle pause (no edge yet)
    local m, w, step = run(smooth, true)
    for _ = 1, 5 do step() end
    -- hold ends on this frame, aim is already requested
    w.vh, w.cam = false, 90
    local n0 = #w.calls.apply
    local why = step()
    for _ = 1, 14 do why = step() end -- 15 frames = 0.25 s after the edge
    xcheck("vehicle settle (" .. tag .. "): no body turn within 0.25 s after the hold ended", #w.calls.apply == n0 and w.body == 0
      and why == "aim align vehicle settle", tostring(#w.calls.apply - n0) .. " " .. tostring(why) .. " body " .. w.body)
    for _ = 1, 90 do step() end
    xcheck("vehicle settle (" .. tag .. "): afterwards the body turns onto the camera at the normal rate",
      #w.calls.apply > n0 and math.abs(norm(w.cam - w.body)) < 2, string.format("body %.1f", w.body))
    -- no vehicle hold ever: identical to the plain aim align (turns at once)
    local mf, wf, stepf = run(smooth, false)
    local nf = #wf.calls.apply
    for _ = 1, 6 do stepf() end
    xcheck("vehicle settle (" .. tag .. "): no hold seen (vehicleHeld false from the start) -> aim align starts at once",
      #wf.calls.apply > nf and wf.body ~= 0, #wf.calls.apply - nf)
    -- no camera API at all (vehicleHeld nil / missing)
    local mn, wn, stepn = run(smooth, "none")
    local nn = #wn.calls.apply
    for _ = 1, 6 do stepn() end
    xcheck("vehicle settle (" .. tag .. "): no VehicleHeld API -> nil-safe, aim align starts at once", #wn.calls.apply > nn and wn.body ~= 0,
      #wn.calls.apply - nn)
    -- a throwing API is also ignored
    local mt, wt, stept = run(smooth, false)
    wt.api.vehicleHeld = function() error("boom") end
    for _ = 1, 6 do stept() end
    xcheck("vehicle settle (" .. tag .. "): throwing VehicleHeld -> ignored, aim align unaffected", #wt.calls.apply > 0 and wt.body ~= 0)
  end
end

do -- 2026-10-13 B2 aim strafe: settings -> Game.UTPPSetMMStrafe(on) + UTPPSetMMStrafeParams(unarmed, 10, 30, 0.15, jogF)
  local savedGame = Game
  local calls = { on = {}, params = {} }
  local accept = true
  Game = {
    UTPPSetMMStrafe = function(on) calls.on[#calls.on + 1] = on end,
    UTPPSetMMStrafeParams = function(unarmed, rt, rw, hold, jog)
      calls.params[#calls.params + 1] = { unarmed, rt, rw, hold, jog }
      return accept
    end,
  }
  local m = freshModule()
  local regs = {}
  m.RegisterSettings({ RegisterSection = function() end,
                       Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end })
  local p = calls.params[#calls.params]
  xcheck("aimStrafe wiring: defaults applied (aimStrafe, unarmed, aimJog, aimJogSpeed) -> UTPPSetMMStrafe(true), params (true, 10, 30, "
    .. "0.15, 3.3 = S4 jog on by default; unarmed strafe on by user default 2026-10-20)",
    #calls.on == 4 and calls.on[4] == true and p and p[1] == true and p[2] == 10 and p[3] == 30 and math.abs(p[4] - 0.15) < 1e-9
    and math.abs(p[5] - 3.3) < 1e-6 and m.strafe.sent ~= nil and m.strafe.sent.jogF == 3.3 and m.strafe.missing == false, #calls.on, p and p[5])
  -- the module defaults equal the setting defaults: every params call while registering carries jogF 3.3 (no jog flash on load)
  xcheck("aimJog wiring: every params call while registering carries jogF 3.3 (module defaults = setting defaults)",
    #calls.params == 4 and calls.params[1][5] == 3.3 and calls.params[2][5] == 3.3 and calls.params[3][5] == 3.3 and calls.params[4][5] == 3.3,
    #calls.params, calls.params[1][5])
  regs[7].apply(true)
  p = calls.params[#calls.params]
  xcheck("aimJog wiring: aimJog on -> jogF 3.3 (default speed)", math.abs(p[5] - 3.3) < 1e-6 and m.strafe.jog == true
    and m.lines[#m.lines]:find("jog 3.30", 1, true) ~= nil, p[5])
  regs[7].apply(false)
  p = calls.params[#calls.params]
  xcheck("aimJog wiring: aimJog off -> jogF 0 (vanilla aim walk speed)", p[5] == 0 and m.strafe.jog == false
    and m.lines[#m.lines]:find("jog 0.00", 1, true) ~= nil, m.lines[#m.lines])
  regs[7].apply(true)
  regs[8].apply(3.85)
  p = calls.params[#calls.params]
  xcheck("aimJog wiring: aimJog on + speed 3.85 -> jogF 3.85", math.abs(p[5] - 3.85) < 1e-6 and m.strafe.jog == true, p[5])
  regs[8].apply(9.0)
  local hi = calls.params[#calls.params][5]
  regs[8].apply(0.5)
  local lo = calls.params[#calls.params][5]
  regs[8].apply(0 / 0)
  local nan = calls.params[#calls.params][5]
  regs[8].apply("x")
  local str = calls.params[#calls.params][5]
  xcheck("aimJog wiring: speed clamped to 3.3..3.85, NaN / non-number -> 3.3 (native range 3.3..6 never violated)",
    math.abs(hi - 3.85) < 1e-6 and math.abs(lo - 3.3) < 1e-6 and nan == 3.3 and str == 3.3, hi, lo, nan, str)
  regs[8].apply(3.3)
  regs[7].apply(false)

  regs[4].apply(false)
  xcheck("aimStrafe wiring: aimStrafe off -> UTPPSetMMStrafe(false)", calls.on[#calls.on] == false and m.strafe.on == false)
  regs[5].apply(false)
  local pOff = calls.params[#calls.params]
  regs[5].apply(true)
  p = calls.params[#calls.params]
  xcheck("aimStrafe wiring: unarmed on -> params unarmed=true", pOff[1] == false and p[1] == true and m.strafe.unarmed == true)
  accept = false
  local fails0 = m.strafe.fails
  regs[5].apply(false)
  xcheck("aimStrafe wiring: params rejected -> counted and logged", m.strafe.fails == fails0 + 1
    and m.lines[#m.lines]:find("aim strafe apply FAILED", 1, true) ~= nil, m.lines[#m.lines])
  Game = savedGame
end

do -- Batch6: an interrupted aim must not carry its active hysteresis into the next aim.
  local m = freshModule()
  local reasons = 4096
  local w = newWorld({ utppCamera = true, locomotion = function() return false end })
  w.api.locomotionReasons = function() return reasons end
  m.SelectExecutor(w.api)
  w.body, w.cam = 0, 90
  w.frame = 1
  m.ExecStep(w.api, 1 / 60)
  xcheck("aim episode: large error arms alignment", m.exec.aimAligning == true)
  reasons = 0
  w.pending = nil
  w.frame = 2
  m.ExecStep(w.api, 1 / 60)
  xcheck("aim episode: exit clears alignment and its one-shot log", m.exec.aimAligning == false and m.exec.aimAlignLogged == false)
  reasons = 4096
  w.body, w.cam = 0, 8
  local before = #w.calls.apply
  w.frame = 3
  local why = m.ExecStep(w.api, 1 / 60)
  xcheck("aim episode: re-entry below 12 deg does not inherit the previous turn", #w.calls.apply == before and why == "aim aligned", why)
  w.cam = 20
  w.frame = 4
  why = m.ExecStep(w.api, 1 / 60)
  xcheck("aim episode: new error beyond the start threshold still turns the body", #w.calls.apply == before + 1 and why == "aim align", why)
end

do -- Batch6 aim policy at real frame durations, including fast mouse input and interrupted episodes.
  for _, hz in ipairs({30, 60, 144}) do
    local dt = 1 / hz
    local m = freshModule(true)
    local reasons, weapon, upper = 4096, false, 0
    local w = newWorld({utppCamera = true, locomotion = function() return false end})
    w.api.locomotionReasons = function() return reasons end
    w.api.aimSignals = function() return {weaponDrawn=weapon, upper=upper} end
    w.api.lookSynced = function() return true end
    m.SelectExecutor(w.api)
    w.body, w.cam = 0, 90
    local function tick()
      w.frame = w.frame + 1
      local before = w.body
      local why = m.ExecStep(w.api, dt)
      if w.pending then w.body, w.pending = w.pending, nil end
      return why, math.abs(norm(w.body - before)) / dt
    end
    for _ = 1, math.floor(0.05 / dt) do tick() end
    xcheck("smooth aim "..hz.."Hz: short empty-hand RMB without UpperBody Aim never turns", #w.calls.apply == 0)
    upper = 6
    for _ = 1, math.max(1, math.floor(0.05 / dt)) do tick() end
    xcheck("smooth aim "..hz.."Hz: UpperBody Aim below 0.1s still waits", #w.calls.apply == 0)
    for _ = 1, math.ceil(0.1 / dt) do tick() end
    xcheck("smooth aim "..hz.."Hz: stable UpperBody Aim becomes eligible", #w.calls.apply > 0)
    reasons = 0; tick()
    reasons, upper, weapon = 4096, 0, true
    w.body, w.cam = 0, 180
    local maxRate, tailRate = 0, 0
    for _ = 1, math.ceil(1.0 / dt) do
      local _, rate = tick()
      maxRate = math.max(maxRate, rate)
      if rate > 0 then tailRate = rate end
    end
    xcheck("smooth aim "..hz.."Hz: 180deg turn respects 540deg/s and reaches within 2deg", maxRate <= 540 + 1e-6 and math.abs(norm(w.cam-w.body)) < 2, maxRate)
    xcheck("smooth aim "..hz.."Hz: final steps ease out instead of staying at the cap", tailRate > 0 and tailRate < 100, tailRate)
    reasons = 0; tick()
    reasons = 4096; w.body, w.cam = 0, 30
    local count = #w.calls.apply
    tick()
    xcheck("smooth aim "..hz.."Hz: upper-body offset absorbs 30deg without a new body turn", #w.calls.apply == count)
    w.cam = 90; tick()
    -- End this started episode on the next frame and request a new one immediately.
    w.body = w.cam; tick()
    w.cam = norm(w.cam + 90)
    count = #w.calls.apply
    tick()
    xcheck("smooth aim "..hz.."Hz: a second episode inside 0.3s does not restart", #w.calls.apply == count)
    for _ = 1, math.ceil(0.3 / dt) do tick() end
    xcheck("smooth aim "..hz.."Hz: cooldown expiry permits the new episode", #w.calls.apply > count)
    -- A missing/failed read must never be treated as a drawn weapon or stable aim.
    reasons = 0; tick(); reasons = 4096
    w.api.aimSignals = function() error("read unavailable") end
    w.body, w.cam = 0, 150
    count = #w.calls.apply
    for _ = 1, math.ceil(0.5 / dt) do tick() end
    xcheck("smooth aim "..hz.."Hz: failed signals keep body alignment inactive", #w.calls.apply == count)
    w.api.aimSignals = function() return {weaponDrawn=true, upper=6} end
    w.cam, w.body = 0, 0
    maxRate = 0
    for frame = 1, 10 * hz do
      -- 90deg in 0.1s followed by 0.4s stationary: the previous fast-mouse regression.
      if (frame - 1) % math.floor(hz * 0.5) < math.ceil(hz * 0.1) then
        w.cam = norm(w.cam + 90 / math.ceil(hz * 0.1))
      end
      local _, rate = tick()
      maxRate = math.max(maxRate, rate)
    end
    for _ = 1, math.ceil(0.5 / dt) do tick() end
    xcheck("smooth aim "..hz.."Hz: repeated fast mouse flicks never stop alignment or exceed cap; settle within 2deg",
      not m.exec.aimAlignStopped and maxRate <= 540 + 1e-6 and math.abs(norm(w.cam-w.body)) < 2,
      "rate "..maxRate.." error "..math.abs(norm(w.cam-w.body)))
    m.api = w.api
    m.exec.aimStable, m.exec.aimLastStart, m.exec.aimAligning = 0.8, m.exec.clock, true
    m.SetAimSmooth(false)
    xcheck("smooth aim "..hz.."Hz: switching policy clears admission, cooldown and active episode",
      m.exec.aimStable == 0 and m.exec.aimLastStart == nil and m.exec.aimAligning == false)
  end
end

do -- Exercise the actual CET reader, not only the policy's table-shaped mock.
  local captured = {}
  local world = {body=0, cam=0, stick=90, camMod={rt={freeHeld=false}}}
  world.camMod.FreeLook = function(on) world.camMod.rt.freeHeld = on == true end
  local _, _, _, err = wired(world, {inspect=function(mod)
    local savedDefs, savedTweak = GetAllBlackboardDefs, TweakDBID
    GetAllBlackboardDefs = function() return {PlayerStateMachine={UpperBody="upper",Weapon="weapon"}} end
    TweakDBID = {new=function(name) return name end}
    local item, badSlot, badUpper = {}, false, false
    Game.GetTransactionSystem = function() return {GetItemInSlot=function(_, player, slot)
      if badSlot then error("slot unavailable") end
      captured.slot = slot
      return item
    end} end
    Game.GetBlackboardSystem = function() return {GetLocalInstanced=function(_, entity, defs)
      if badUpper then error("PSM unavailable") end
      captured.entity = entity
      return {GetInt=function(_, key) return key == "upper" and 6 or (key == "weapon" and 8 or nil) end}
    end} end
    captured.drawn = mod.api.aimSignals()
    item = nil
    captured.empty = mod.api.aimSignals()
    badSlot = true
    captured.slotFailure = mod.api.aimSignals()
    badUpper = true
    captured.bothFailed = mod.api.aimSignals()
    Game.GetPlayer = function() return nil end
    captured.noPlayer = mod.api.aimSignals()
    GetAllBlackboardDefs, TweakDBID = savedDefs, savedTweak
  end})
  xcheck("smooth aim CET reader: drawn slot and UpperBody from the current player's PSM", err == nil and captured.drawn
    and captured.drawn.weaponDrawn and captured.drawn.upper == 6 and captured.drawn.action == 8 and captured.slot == "AttachmentSlots.WeaponRight" and captured.entity == 1, err)
  xcheck("smooth aim CET reader: empty/failed slot does not invent a weapon, UpperBody remains usable", captured.empty
    and captured.empty.weaponDrawn == false and captured.empty.upper == 6 and captured.slotFailure.weaponDrawn == false)
  xcheck("smooth aim CET reader: failed PSM and slot become absent signals", captured.bothFailed
    and captured.bothFailed.weaponDrawn == false and captured.bothFailed.upper == nil)
  xcheck("smooth aim CET reader: missing player returns no admission signals", captured.noPlayer == nil)
end

do -- 2026-10-13 B2 aim strafe S2: setting bodyturn.aimChestAlign -> Game.UTPPSetMMChestAlign(on); params stay at the native defaults
  local savedGame = Game
  local calls = { on = {}, params = 0 }
  local throw = false
  Game = {
    UTPPSetMMChestAlign = function(on) if throw then error("boom") end calls.on[#calls.on + 1] = on end,
    UTPPSetMMChestAlignParams = function() calls.params = calls.params + 1; return true end,
  }
  local m = freshModule()
  local regs = {}
  m.RegisterSettings({ RegisterSection = function() end,
                       Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end })
  xcheck("aimChestAlign wiring: default applied -> UTPPSetMMChestAlign(true) once, params never sent (native defaults 0.5 / 60)",
    #calls.on == 1 and calls.on[1] == true and calls.params == 0 and m.chest.sent ~= nil and m.chest.missing == false, #calls.on)
  regs[6].apply(false)
  xcheck("aimChestAlign wiring: off -> UTPPSetMMChestAlign(false)", calls.on[#calls.on] == false and m.chest.on == false
    and m.lines[#m.lines]:find("aim chest align -> off", 1, true) ~= nil, m.lines[#m.lines])
  throw = true
  local fails0 = m.chest.fails
  regs[6].apply(true)
  xcheck("aimChestAlign wiring: native error -> counted and logged", m.chest.fails == fails0 + 1
    and m.lines[#m.lines]:find("chest align apply FAILED", 1, true) ~= nil, m.lines[#m.lines])
  Game = savedGame
end

do -- Hipfire heading preempts Locomotion without inventing native ADS or camera zoom.
  for _, hz in ipairs({30,60,144}) do
    local m = freshModule(true)
    local w = newWorld({utppCamera=true})
    local action, drawn, reasons = 8, true, 0
    w.api.aimSignals=function()return {weaponDrawn=drawn,upper=0,action=action}end
    w.api.locomotionReasons=function()return reasons end
    w.api.lookSynced=function()return true end
    w.api.alignHold=function()return true end
    w.api.applyAim=w.api.apply
    w.api.viewYaw=function()return w.cam end
    m.SelectExecutor(w.api);w.body,w.cam=180,0
    for _=1,math.ceil(.3*hz) do
      m.ExecStep(w.api,1/hz);if w.pending then w.body=w.pending;w.pending=nil end
    end
    xcheck("hipfire "..hz.."Hz: live Shoot8 preempts movement yaw and turns180deg within0.3s",
      math.abs(norm(w.body))<2 and w.calls.command==0 and m.exec.nativeOn==false)
    m.exec.aimAlign=false;m.ExecStep(w.api,1/hz)
    xcheck("hipfire "..hz.."Hz: disabled alignment does not freeze Locomotion turn",m.exec.hipFireRemaining==0 and m.exec.nativeOn==true)
    m.exec.aimAlign=true
    for _, blocked in ipairs({1,2048,65536,131072,262144}) do
      reasons=blocked;action=8;m.ExecStep(w.api,1/hz)
      xcheck("hipfire "..hz.."Hz: unsafe reason "..blocked.." rejects action",m.exec.hipFireRemaining==0)
    end
    reasons=0;action=8;m.ExecStep(w.api,1/hz)
    action=0;m.ExecStep(w.api,1/hz)
    xcheck("hipfire "..hz.."Hz: brief shot gap retains heading lease",m.exec.hipFireRemaining>0)
    action=8;reasons=65536;m.ExecStep(w.api,1/hz)
    xcheck("hipfire "..hz.."Hz: FPP clears lease immediately",m.exec.hipFireRemaining==0)
    reasons=0;action=8;drawn=true;m.ExecStep(w.api,1/hz)
    action=2;m.ExecStep(w.api,1/hz)
    xcheck("hipfire "..hz.."Hz: reload clears lease immediately",m.exec.hipFireRemaining==0)
    action=8;drawn=true;m.ExecStep(w.api,1/hz);drawn=false;m.ExecStep(w.api,1/hz)
    xcheck("hipfire "..hz.."Hz: holster clears lease immediately",m.exec.hipFireRemaining==0)
    drawn=true;action=8;m.ExecStep(w.api,1/hz);action=0
    for _=1,math.ceil(.8*hz) do m.ExecStep(w.api,1/hz) end
    xcheck("hipfire "..hz.."Hz: action tail expires and restores movement turn",m.exec.hipFireRemaining==0 and m.exec.nativeOn==true)
  end
end

do -- Real executor: Ready gap keeps aim heading, never resumes movement turn early.
  local m=freshModule(true);local w=newWorld({utppCamera=true});local action=8
  w.api.aimSignals=function()return {weaponDrawn=true,upper=0,action=action}end
  w.api.locomotionReasons=function()return 8192 end
  w.api.lookSynced=function()return true end
  w.api.alignHold=function()return true end
  w.api.applyAim=w.api.apply;w.api.viewYaw=function()return w.cam end
  m.SelectExecutor(w.api);w.body,w.cam=90,0
  m.ExecStep(w.api,1/60);action=5
  for _=1,24 do m.ExecStep(w.api,1/60);if w.pending then w.body=w.pending;w.pending=nil end end
  xcheck("hipfire Ready gap: executor retains aim alignment and blocks movement turn",
    m.exec.hipFireRemaining>0 and m.exec.nativeOn==false and w.calls.command==0 and math.abs(norm(w.body))<2)
end

do -- Native Ready/Safe/NoAmmo/Overheat between shots must share the C++ pose hold.
  for _, hz in ipairs({30,60,144}) do
    for _, gap in ipairs({0,1,4,5,6,7}) do
      local m=freshModule(true)
      local sig={weaponDrawn=true,upper=0,action=8}
      xcheck("hipfire gap "..gap.." "..hz.."Hz: Shoot starts heading lease",m.HipFireStep(sig,8192,1/hz))
      sig.action=gap
      for _=1,math.floor(.4*hz) do m.HipFireStep(sig,8192,1/hz) end
      xcheck("hipfire gap "..gap.." "..hz.."Hz: heading survives native shot gap",m.exec.hipFireRemaining>0)
      sig.action=8;m.HipFireStep(sig,8192,1/hz);sig.action=gap
      for _=1,math.ceil(.8*hz) do m.HipFireStep(sig,8192,1/hz) end
      xcheck("hipfire gap "..gap.." "..hz.."Hz: finite tail expires",m.exec.hipFireRemaining==0)
    end
    for _, bad in ipairs({2,3,9,-1,0/0}) do
      local m=freshModule(true);local sig={weaponDrawn=true,upper=0,action=8}
      m.HipFireStep(sig,8192,1/hz);sig.action=bad
      xcheck("hipfire unknown/reload "..tostring(bad).." clears lease",not m.HipFireStep(sig,8192,1/hz))
    end
  end
end

do -- P4/P5 (recording 2026-10-21 12:15): a Shoot=8 in ADS must not lease hipfire heading. Replays the three recorded episodes
   -- "aimed shot -> RMB released 0.0-0.15 s before Shoot=8 ends -> camera turned 150-250 deg while V stands (stick 0)".
   -- Before the fix the lease aligned the capsule onto the camera at 1080 deg/s for 0.75 s (recorded: -248, +200, -254 deg),
   -- the native offset root hid it and the next reload revealed it as a 100-180 deg spin of the visible V.
  -- generated by tools/analysis/gen_lua_ads_shot_fixture.py from the 2026-10-21 12:15 recording (pose frame clock)
  -- fields: name; body0 = capsule yaw (deg) at the first sample; adsOn/adsOff = upperBody==6 window; shot0/shot1 = Shoot=8 window
  -- (s from the first sample); cam = {t0, yaw0, t1, yaw1, ...} camera yaw shown (deg), t in s from the first sample
  local P4_ADS_SHOTS = {
    { name = "v81.4", body0 = -37.4, adsOn = 0.107, adsOff = 0.802, shot0 = 0.600, shot1 = 0.951,
      cam = {0.000,-25.5,0.017,-21.1,0.033,-15.6,0.050,-9.1,0.067,-2.7,0.084,3.5,0.107,7.7,0.117,9.5,0.134,10.6,0.150,11.6,0.167,12.9,0.184,14.4,0.202,16.5,0.218,18.5,0.235,19.9,0.251,20.3,0.268,20.5,0.284,20.9,0.300,22.0,0.319,24.0,0.333,26.5,0.350,29.2,0.367,32.0,0.383,35.0,0.400,38.8,0.417,43.3,0.435,48.4,0.450,54.1,0.468,60.9,0.483,67.8,0.500,74.8,0.517,80.8,0.534,85.3,0.562,88.4,0.568,89.8,0.584,90.3,0.600,90.3,0.617,90.3,0.634,90.3,0.652,90.3,0.668,90.3,0.684,90.3,0.700,90.3,0.717,90.3,0.734,90.3,0.751,90.2,0.769,90.2,0.785,90.2,0.802,90.2,0.817,90.2,0.834,90.2,0.850,90.4,0.867,90.5,0.884,90.6,0.902,90.7,0.918,90.5,0.934,88.9,0.951,84.5,0.968,78.4,0.984,70.0,1.002,60.8,1.018,50.8,1.034,39.2,1.051,26.3,1.068,13.3,1.084,-0.9,1.102,-14.4,1.118,-26.6,1.134,-39.3,1.151,-51.2,1.168,-61.8,1.184,-72.3,1.201,-82.1,1.219,-91.0,1.234,-99.0,1.250,-105.6,1.268,-111.1,1.284,-116.4,1.301,-121.4,1.317,-126.1,1.335,-130.3,1.350,-133.7,1.367,-136.3,1.384,-139.4,1.400,-142.6,1.417,-145.9,1.434,-149.4,1.452,-152.7,1.468,-155.4,1.484,-157.8,1.501,-159.5,1.517,-160.8,1.534,-162.5,1.550,-164.2,1.569,-164.2,1.584,-164.2,1.600,-164.2,1.618,-164.2,1.634,-164.2,1.650,-164.2,1.669,-164.2,1.684,-164.2,1.700,-164.2,1.717,-164.2,1.734,-164.2,1.750,-164.2,1.767,-164.2,1.785,-164.2,1.801,-164.2,1.817,-164.0,1.834,-161.5,1.851,-159.2,1.867,-159.0,1.886,-159.4,1.900,-159.6,1.917,-159.5,1.934,-159.0,1.950,-158.6,1.967,-158.5,1.984,-158.5,2.001,-158.4,2.019,-158.3,2.040,-158.3,2.051,-158.3,2.068,-158.3,2.085,-159.0,2.101,-159.9,2.117,-160.4,2.134,-160.5,2.151,-160.6,2.167,-160.6,2.185,-160.6,2.201,-160.6,2.218,-160.6,2.234,-160.6,2.251,-160.6,2.270,-160.6,2.285,-160.6,2.302,-160.6,2.318,-160.6,2.335,-160.6,2.351,-160.6,2.368,-160.6,2.384,-160.6,2.403,-160.6,2.418,-160.6,2.434,-160.6,2.451,-160.6,2.468,-160.6,2.484,-160.6,2.501,-160.6,2.518,-160.6} },
    { name = "v85.2", body0 = -157.7, adsOn = 0.102, adsOff = 0.802, shot0 = 0.567, shot1 = 0.916,
      cam = {0.000,-156.0,0.017,-155.0,0.033,-153.4,0.050,-151.2,0.067,-148.6,0.084,-146.5,0.102,-145.3,0.116,-145.0,0.133,-145.0,0.149,-145.0,0.166,-144.9,0.183,-144.9,0.201,-144.9,0.217,-144.9,0.233,-144.9,0.249,-144.9,0.266,-144.9,0.283,-144.9,0.300,-144.9,0.318,-144.9,0.333,-144.9,0.350,-144.9,0.366,-144.9,0.384,-144.9,0.400,-144.9,0.418,-144.9,0.433,-144.9,0.450,-144.9,0.466,-144.9,0.489,-144.9,0.500,-144.9,0.518,-144.9,0.534,-144.9,0.550,-144.9,0.567,-144.9,0.584,-144.9,0.601,-144.9,0.618,-144.9,0.638,-144.9,0.650,-144.9,0.667,-144.9,0.684,-144.9,0.700,-144.9,0.717,-144.9,0.736,-144.9,0.750,-144.9,0.767,-144.9,0.785,-144.9,0.802,-144.9,0.817,-144.9,0.834,-144.9,0.850,-144.8,0.867,-144.5,0.883,-144.0,0.911,-143.5,0.916,-143.4,0.933,-143.9,0.950,-143.9,0.967,-143.9,0.983,-143.9,0.999,-143.9,1.018,-143.9,1.033,-143.9,1.050,-143.9,1.066,-143.9,1.083,-143.9,1.099,-143.9,1.117,-143.9,1.133,-143.9,1.150,-143.5,1.167,-136.9,1.183,-129.0,1.199,-120.8,1.218,-111.4,1.234,-101.5,1.250,-89.4,1.267,-75.1,1.283,-62.9,1.300,-49.7,1.316,-38.3,1.335,-28.9,1.349,-19.5,1.366,-9.9,1.384,-0.5,1.400,9.1,1.416,17.5,1.435,24.7,1.449,30.9,1.466,36.3,1.483,41.1,1.500,45.9,1.517,49.9,1.533,52.7,1.551,54.8,1.566,56.4,1.583,57.8,1.600,59.5,1.616,61.6,1.634,64.0,1.651,66.9,1.666,70.3,1.683,73.9,1.700,77.8,1.716,81.2,1.733,83.5,1.752,85.1,1.767,85.7,1.783,85.8,1.800,85.8,1.817,85.8,1.834,85.1,1.849,83.1,1.867,78.9,1.883,72.9,1.899,65.3,1.916,57.3,1.934,50.4,1.950,43.3,1.966,36.3,1.983,29.5,1.999,22.4,2.016,16.1,2.034,10.8,2.050,7.4,2.067,5.7,2.084,5.1,2.100,4.3,2.117,2.7,2.134,0.5,2.150,-2.6,2.167,-6.2,2.184,-10.8,2.199,-16.6,2.217,-23.8,2.234,-31.4,2.250,-38.4,2.266,-43.6,2.284,-46.4,2.300,-47.0,2.316,-45.5,2.333,-41.6,2.350,-35.1,2.368,-26.0,2.383,-16.5,2.400,-5.3,2.416,6.5,2.433,18.0,2.449,30.2,2.467,42.4,2.483,53.2,2.500,64.2} },
    { name = "v91.1", body0 = 54.7, adsOn = 0.102, adsOff = 0.752, shot0 = 0.450, shot1 = 0.784,
      cam = {0.000,131.1,0.018,131.1,0.033,131.1,0.050,131.1,0.067,131.1,0.084,131.1,0.102,131.1,0.117,131.1,0.134,131.1,0.150,131.3,0.167,131.3,0.183,131.3,0.201,131.3,0.217,131.3,0.233,131.3,0.250,131.3,0.267,131.3,0.283,131.3,0.301,131.3,0.317,131.3,0.333,131.3,0.350,131.3,0.367,131.3,0.383,131.3,0.402,131.3,0.417,131.3,0.433,131.3,0.450,131.3,0.467,131.3,0.484,131.3,0.500,131.3,0.518,131.3,0.533,131.3,0.550,131.3,0.567,131.3,0.584,131.3,0.600,131.3,0.618,131.3,0.633,131.3,0.650,131.3,0.667,131.3,0.684,131.3,0.700,131.3,0.717,131.3,0.736,131.3,0.752,131.3,0.767,131.3,0.784,131.3,0.800,131.3,0.817,131.3,0.833,131.3,0.852,131.3,0.867,131.1,0.885,130.5,0.900,129.2,0.917,127.1,0.934,124.5,0.952,121.3,0.967,117.3,0.983,112.2,1.000,106.8,1.017,101.2,1.034,95.7,1.050,90.4,1.069,83.7,1.084,77.3,1.101,70.5,1.118,62.6,1.134,54.0,1.151,44.0,1.169,33.7,1.184,23.0,1.200,12.3,1.222,0.8,1.234,-9.5,1.251,-19.5,1.270,-28.3,1.284,-36.1,1.302,-43.2,1.317,-50.4,1.335,-58.3,1.351,-64.4,1.368,-71.5,1.388,-78.5,1.402,-84.7,1.418,-90.9,1.434,-96.9,1.451,-102.4,1.467,-108.3,1.484,-113.7,1.504,-118.5,1.517,-123.4,1.534,-128.3,1.551,-132.4,1.567,-136.2,1.584,-139.2,1.600,-141.2,1.619,-142.5,1.634,-143.7,1.650,-145.3,1.668,-147.0,1.683,-148.2,1.700,-148.6,1.718,-148.5,1.743,-148.1,1.751,-146.8,1.767,-144.6,1.784,-141.6,1.800,-137.4,1.817,-131.7,1.834,-124.0,1.853,-114.6,1.868,-103.5,1.883,-91.0,1.900,-78.1,1.917,-66.4,1.933,-54.0,1.950,-42.1,1.967,-28.6,1.984,-14.6,2.000,-1.0,2.017,13.1,2.034,25.7,2.051,35.8,2.067,44.6,2.084,50.8,2.101,54.1,2.117,55.0,2.134,54.4,2.150,52.5,2.167,49.0,2.183,43.9,2.201,36.9,2.217,27.4,2.233,15.9,2.251,3.2,2.267,-8.9,2.284,-21.7,2.301,-33.7,2.317,-45.8,2.334,-56.7,2.351,-66.0,2.367,-74.5} },
  }
  local function camAt(ep, t)
    local c, n = ep.cam, #ep.cam / 2
    if t <= c[1] then return c[2] end
    for k = 1, n - 1 do
      local t0, t1 = c[2 * k - 1], c[2 * k + 1]
      if t <= t1 then
        local y0, y1 = c[2 * k], c[2 * k + 2]
        return norm(y0 + norm(y1 - y0) * (t - t0) / math.max(t1 - t0, 1e-9))
      end
    end
    return c[2 * n]
  end
  -- hipfire = true replays the same shot without ADS (UpperBody stays 0): there the lease must still follow the camera.
  -- drift = largest capsule excursion after the reference instant (ADS exit; hipfire: first Shoot=8 sample).
  local function replay(m, ep, hz, hipfire, fpp)
    local dt = 1 / hz
    local w = newWorld({utppCamera = true, cmd = function(_, cur) return cur end}) -- stick 0: Locomotion asks for no turn
    local ads, action, applyAim = false, 0, 0
    w.api.aimSignals = function() return {weaponDrawn = true, upper = ads and 6 or 0, action = action} end
    -- fpp: ADS in first person (Locomotion adds kNotTpp, 65536); the RMB release returns to third person
    w.api.locomotionReasons = function() return ads and (fpp and 4096 + 65536 or 4096) or 8192 end
    w.api.locomotionActive = function() return not ads end
    w.api.lookSynced = function() return true end
    w.api.alignHold = function() return true end
    w.api.viewYaw = function() return w.cam end
    w.api.applyAim = function(yaw) applyAim = applyAim + 1; return w.api.apply(yaw) end
    m.SelectExecutor(w.api)
    w.body = ep.body0
    local refT = hipfire and ep.shot0 or ep.adsOff
    local refBody, refApply, drift, leaseFrames = nil, 0, 0, 0
    for i = 0, math.floor(ep.cam[#ep.cam - 1] * hz) do
      local t = i * dt
      w.cam = camAt(ep, t)
      ads = (not hipfire) and t >= ep.adsOn and t < ep.adsOff
      action = (t >= ep.shot0 and t < ep.shot1) and 8 or 0
      w.frame = w.frame + 1
      m.ExecStep(w.api, dt)
      if w.pending then w.body, w.pending = w.pending, nil end
      if refBody == nil and t >= refT then refBody, refApply = w.body, applyAim end
      if refBody ~= nil then
        drift = math.max(drift, math.abs(norm(w.body - refBody)))
        if m.exec.hipFireRemaining > 0 then leaseFrames = leaseFrames + 1 end
      end
    end
    return {drift = drift, applyAfter = applyAim - refApply, leaseFrames = leaseFrames}
  end
  -- Mutants are made from the shipped source text: a textual edit that must be found exactly once, so the harness cannot rot.
  local function mutant(from, to)
    local f = assert(io.open(UTPP_BODYTURN_PATH, "rb"))
    local src = f:read("*a"); f:close()
    local a, b = src:find(from, 1, true)
    assert(a and not src:find(from, b + 1, true), "mutation anchor must exist exactly once: " .. from)
    local m = assert(loadstring(src:sub(1, a - 1) .. to .. src:sub(b + 1), "=utpp_bodyturn_mutant"))()
    m.exec.aimSmooth = true
    m.lines = {}
    m.log = function(msg) m.lines[#m.lines + 1] = msg end
    return m
  end
  local mutants = {
    {"ADS clause removed: lease on every Shoot=8", "elseif sig.upper == 6 then", "elseif false then"},
    {"ADS-tail attribution removed: lease starts on the Shoot=8 tail after the RMB release",
     "E.hipFireAdsTail = false, 0, 0.4", "E.hipFireAdsTail = false, 0, 0"},
  }
  for _, hz in ipairs({60, 144}) do
    for _, ep in ipairs(P4_ADS_SHOTS) do
      local r = replay(freshModule(true), ep, hz, false)
      xcheck(string.format("ADS shot %s %dHz: capsule keeps its heading after the RMB release (drift %.2f deg <= 2)", ep.name, hz, r.drift),
        r.drift <= 2, r.drift)
      xcheck(string.format("ADS shot %s %dHz: no aim-align request and no hipfire lease after the RMB release", ep.name, hz),
        r.applyAfter == 0 and r.leaseFrames == 0, string.format("applyAfter %d lease frames %d", r.applyAfter, r.leaseFrames))
      for _, mu in ipairs(mutants) do
        local rm = replay(mutant(mu[2], mu[3]), ep, hz, false)
        xcheck(string.format("mutation [%s] is caught on %s %dHz (capsule follows the camera by %.0f deg)", mu[1], ep.name, hz, rm.drift),
          rm.drift > 100 and rm.leaseFrames > 0 and rm.applyAfter > 0, rm.drift)
      end
      local h = replay(freshModule(true), ep, hz, true)
      xcheck(string.format("hipfire shot %s %dHz (no ADS): the lease still turns the capsule onto the camera (%.0f deg)", ep.name, hz, h.drift),
        h.drift > 100 and h.leaseFrames > 0 and h.applyAfter > 0, h.drift)
      -- the same recorded shots aimed in first person (perspective.aiming = fpp): the latch is armed from the FPP aim samples
      local f = replay(freshModule(true), ep, hz, false, true)
      xcheck(string.format("FPP ADS shot %s %dHz: capsule keeps its heading after the RMB release back in TPP (drift %.2f deg <= 2)", ep.name, hz, f.drift),
        f.drift <= 2 and f.applyAfter == 0 and f.leaseFrames == 0, f.drift)
      local fm = replay(mutant("if fppAds then", "if false then"), ep, hz, false, true)
      xcheck(string.format("mutation [FPP-ADS arming removed] is caught on %s %dHz (capsule follows the camera by %.0f deg)", ep.name, hz, fm.drift),
        fm.drift > 100 and fm.leaseFrames > 0, fm.drift)
    end
  end
  -- Signal-level contract of the lease (shared with the C++ HipFirePoseHold gap rules, tests above stay unchanged).
  for _, hz in ipairs({30, 60, 144}) do
    local dt = 1 / hz
    local function steps(m, sig, seconds, reasons)
      local lease = false
      for _ = 1, math.max(1, math.floor(seconds * hz + 1e-6)) do lease = m.HipFireStep(sig, reasons or 4096, dt) end
      return lease
    end
    local m = freshModule(true)
    local sig = {weaponDrawn = true, upper = 6, action = 8}
    xcheck("ADS lease " .. hz .. "Hz: Shoot=8 in ADS never starts the lease", not steps(m, sig, 0.3))
    sig.upper = 0
    xcheck("ADS lease " .. hz .. "Hz: the Shoot=8 tail after the RMB release is still ADS", not steps(m, sig, 0.15, 8192))
    sig.action = 0
    xcheck("ADS lease " .. hz .. "Hz: action 8 -> 0 ends the ADS attribution, no lease", not steps(m, sig, 0.1, 8192))
    sig.action = 8
    xcheck("ADS lease " .. hz .. "Hz: the next Shoot=8 run without ADS is hipfire again", steps(m, sig, 0.1, 8192))
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = 6, action = 8}
    steps(m, sig, 0.2); sig.upper = 0
    xcheck("ADS lease " .. hz .. "Hz: a burst held 0.3 s after the RMB release is hipfire (attribution is bounded)", not steps(m, sig, 0.3, 8192)
      and steps(m, sig, 0.3, 8192))
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = 6, action = 8}
    xcheck("ADS lease " .. hz .. "Hz: Shoot=8 in FPP ADS (reasons carry kNotTpp) never starts the lease", not steps(m, sig, 0.3, 4096 + 65536))
    sig.upper = 0
    xcheck("ADS lease " .. hz .. "Hz: the Shoot=8 tail after the FPP ADS ended (back in TPP) is still ADS", not steps(m, sig, 0.15, 8192))
    sig.action = 0; steps(m, sig, 0.1, 8192); sig.action = 8
    xcheck("ADS lease " .. hz .. "Hz: the next Shoot=8 run after an FPP ADS shot is hipfire again", steps(m, sig, 0.1, 8192))
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = 0, action = 8}
    xcheck("ADS lease " .. hz .. "Hz: FPP without ADS (UpperBody != 6) still clears the lease at once", not steps(m, sig, 0.1, 65536))
    sig.upper = 6; sig.weaponDrawn = false
    steps(m, sig, 0.1, 4096 + 65536); sig.weaponDrawn = true; sig.upper = 0
    xcheck("ADS lease " .. hz .. "Hz: an FPP aim sample without a drawn weapon does not arm the latch", steps(m, sig, 0.1, 8192))
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = 6, action = 8}
    steps(m, sig, 0.1, 4096 + 65536 + 2048); sig.upper = 0
    xcheck("ADS lease " .. hz .. "Hz: an FPP aim sample with another blocking reason bit does not arm the latch", steps(m, sig, 0.1, 8192))
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = 0, action = 8}
    xcheck("ADS lease " .. hz .. "Hz: a hipfire shot starts the lease", steps(m, sig, 0.1, 8192))
    sig.upper = 6
    xcheck("ADS lease " .. hz .. "Hz: pressing RMB clears a running hipfire lease at once", not steps(m, sig, 0.01) and m.exec.hipFireRemaining == 0)
    m = freshModule(true)
    sig = {weaponDrawn = true, upper = nil, action = 8}
    xcheck("ADS lease " .. hz .. "Hz: unreadable UpperBody behaves as before (hipfire lease)", steps(m, sig, 0.1, 8192))
  end
end

-- ================================================================ 2026-10-22 Shooter mode (bodyturn.shooterMode)
-- B1-B9 (spec acceptance_tests): registration, SelectExecutor, live ON / OFF order, Locomotion
-- re-assert, face-the-camera align (measured progress, pause, TPP gate), aim in Shooter, native missing / failing / retry,
-- v2 not enabled / not live, Status. The natives are mocked (Game global); the module code is the real one.
do
  local savedGame = Game
  local function nrm(v) return norm(v) end

  -- world: shared call sequence (S) for ordering checks; viewNil / applyOn / reasons / v2 / live are switchable
  local function fnWorld(o)
    o = o or {}
    local w = { body = o.body or 0, view = o.view or 0, reasons = o.reasons or 0, freeMove = o.freeMove == true,
                v2 = o.v2 ~= false, live = o.live, seq = {}, aimSteps = {}, bodySteps = {}, lookSteps = {}, holds = {},
                camHeld = o.camHeld == true, applyOn = o.applyOn ~= false, frame = 0, masks = {}, freeMoveCalls = {}, nativeCalls = {},
                aimCalls = 0, sig = o.sig, natives = {}, cmd = o.cmd, freeMoveAt = {} }
    local function S(x) w.seq[#w.seq + 1] = x end
    w.S = S
    w.api = {
      arm = function() return true end,
      setNative = function(on) S("setNative:" .. tostring(on)); w.nativeOn = on; w.nativeCalls[#w.nativeCalls + 1] = on end,
      locomotionEnable = function(on) S("freeMove:" .. tostring(on)); w.freeMoveCalls[#w.freeMoveCalls + 1] = on
        if on == false then w.freeMoveAt[#w.freeMoveAt + 1] = w.frame end end,
      locomotionActive = function() return w.freeMove end,
      locomotionReasons = function() return w.reasons end,
      aimSignals = function() return w.sig end,
      active = function() return w.nativeOn == true end,
      bodyYaw = function() return w.body end,
      cameraYaw = function() return w.view end,
      viewYaw = function() if w.viewNil then return nil end return w.view end,
      command = function(cur) if w.cmd then return w.cmd(cur) end return cur end,
      apply = function(yaw) S("apply"); w.pending = yaw; return true end,
      applyAim = function(yaw) S("applyAim"); w.aimCalls = w.aimCalls + 1
        w.aimSteps[#w.aimSteps + 1] = nrm(yaw - w.body); w.pending = yaw; return true end,
      camHeld = function() return w.camHeld end,
      camHold = function(on) S("camHold:" .. tostring(on)); w.holds[#w.holds + 1] = on; w.camHeld = on; return true end,
      alignHold = function(on) S("alignHold:" .. tostring(on)); return true end,
      bodyStep = function(s) w.bodySteps[#w.bodySteps + 1] = s end,
      lookStep = function(s) w.lookSteps[#w.lookSteps + 1] = s end,
      hasPlayer = function() return true end,
      setBlockMask = function(mask) S("mask:" .. mask); w.masks[#w.masks + 1] = mask end,
      motionEnabled = function() return w.v2 end,
      shooterModeLive = function() return w.live end,
    }
    return w
  end
  local function installGame(w, o)
    o = o or {}
    w.natives = {}
    Game = {}
    if not o.noNative then
      Game.UTPPSetMotionShooterMode = function(on)
        w.S("native:" .. tostring(on)); w.natives[#w.natives + 1] = on
        if w.nativeFail then error("native boom") end
        return true
      end
    end
  end
  local function fnStep(m, w, dt)
    w.frame = w.frame + 1
    local why = m.ExecStep(w.api, dt or 1 / 60)
    if w.pending ~= nil then
      if w.applyOn then w.body = w.pending end
      w.pending = nil
    end
    return why
  end
  local function fnRun(m, w, n, dt, each)
    local why
    for _ = 1, n do why = fnStep(m, w, dt); if each then each(why) end end
    return why
  end
  local function fnModule()
    local m = freshModule(true)
    m.exec.aimAlign = true
    return m
  end
  local function count(list, x) local n = 0 for _, v in ipairs(list) do if v == x then n = n + 1 end end return n end
  local function index(list, x) for i, v in ipairs(list) do if v == x then return i end end return nil end
  local function lastIndex(list, x) local r for i, v in ipairs(list) do if v == x then r = i end end return r end
  local function lines(m, pat) local n = 0 for _, l in ipairs(m.lines) do if l:find(pat, 1, true) then n = n + 1 end end return n end
  local function seqStr(w) return table.concat(w.seq, ",") end

  -- ---- B1 registration
  do
    local m = freshModule()
    local w = fnWorld()
    installGame(w)
    local regs = {}
    local core = { RegisterSection = function() end,
                   Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end }
    m.RegisterSettings(core)
    local d = regs[12]
    xcheck("B1 shooterMode: 12 registrations, aimSmooth stays #10, meleeStrikeTurn #11, bodyturn.shooterMode is the LAST one", #regs == 12
      and regs[10].key == "bodyturn.aimSmooth" and regs[11].key == "bodyturn.meleeStrikeTurn" and d ~= nil and d.key == "bodyturn.shooterMode", d and d.key)
    xcheck("B1 shooterMode: type bool, default false, section bodyturn, label exactly 'Shooter mode' (2026-10-23), English description (camera-relative)",
      d.type == "bool" and d.default == false and d.section == "bodyturn" and d.label == "Shooter mode"
      and d.desc ~= nil and d.desc:find("relative to the camera", 1, true) ~= nil
      and type(d.desc) == "string" and d.desc:find("Sprint", 1, true) ~= nil and d.desc:find("70 %", 1, true) ~= nil
      and d.desc:find("60 %", 1, true) ~= nil
      and d.desc:find("free look", 1, true) == nil and d.desc:find("key", 1, true) == nil, d.label)
    xcheck("B1 shooterMode: apply(false) at registration (mode pending) = ONE UTPPSetMotionShooterMode(false), no locomotion / setNative",
      #w.natives == 1 and w.natives[1] == false and #w.freeMoveCalls == 0 and #w.nativeCalls == 0 and seqStr(w) == "native:false",
      seqStr(w))
    xcheck("B1 shooterMode: setting off -> not effective, Status shows it", m.shooterMode.on == false and m.ShooterModeActive() == false
      and m.Status():find("shooterMode=false eff=false native=ok", 1, true) ~= nil, m.Status())
    -- native absent (older DLL): OFF registration is silent
    local m2 = freshModule()
    Game = {}
    local regs2 = {}
    m2.RegisterSettings({ RegisterSection = function() end,
                          Register = function(def) regs2[#regs2 + 1] = def; if def.apply then def.apply(def.default) end end })
    xcheck("B1 shooterMode: native missing + setting off -> no log line at all", lines(m2, "Shooter") == 0 and #regs2 == 12
      and m2.ShooterModeActive() == false)
  end

  -- ---- B2 SelectExecutor (pending -> executor)
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true })
    installGame(w)
    m.SetShooterMode(true)
    xcheck("B2 shooterMode: registration-time ON in pending mode: native(true) only, nothing else touched",
      seqStr(w) == "native:true" and m.shooterMode.effective == true and m.ShooterModeActive() == true, seqStr(w))
    m.api = w.api
    w.seq = {}
    local line = m.SelectExecutor(w.api)
    xcheck("B2 shooterMode: selection with Shooter: native body turn forced off, Locomotion off (never on), block mask sent",
      seqStr(w) == "setNative:false,freeMove:false,mask:24511" and count(w.freeMoveCalls, true) == 0 and m.exec.locomotionOwned == false
      and m.shooterMode.align == true and line:find("Locomotion off (Shooter mode)", 1, true) ~= nil, seqStr(w))
    -- Shooter OFF: today's sequence
    local m0 = fnModule()
    local w0 = fnWorld({ freeMove = true })
    installGame(w0)
    m0.SetShooterMode(false)
    w0.seq = {}
    m0.SelectExecutor(w0.api)
    xcheck("B2 shooterMode: OFF selection = today's sequence (native off, Locomotion on, mask)",
      seqStr(w0) == "setNative:false,freeMove:true,mask:24511" and m0.exec.locomotionOwned == true and m0.shooterMode.align == false,
      seqStr(w0))
  end

  -- ---- B3 live ON / OFF in executor mode
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true, view = 0, body = 0, cmd = function(cur) return norm(cur + 6) end })
    installGame(w)
    m.SetShooterMode(false)
    m.api = w.api
    m.SelectExecutor(w.api)
    fnRun(m, w, 10)
    xcheck("B3 shooterMode: before ON the executor drives (native on, own camera hold)", w.nativeOn == true and m.exec.holdOwned == true
      and m.exec.locomotionOwned == true)
    w.seq, w.reasons = {}, 0
    m.SetShooterMode(true)
    local iN, iS, iM = index(w.seq, "native:true"), index(w.seq, "setNative:false"), index(w.seq, "freeMove:false")
    xcheck("B3 shooterMode: ON order = native(true), setNative(false), locomotionEnable(false)", iN == 1 and iS == 2 and iM == 3, seqStr(w))
    xcheck("B3 shooterMode: ON releases the module's own hold, align armed, Locomotion not owned", m.exec.holdOwned == false
      and index(w.seq, "camHold:false") ~= nil and m.shooterMode.align == true and m.exec.locomotionOwned == false and w.nativeOn == false,
      seqStr(w))
    fnRun(m, w, 30)
    xcheck("B3 shooterMode: the executor never turns the native body turn back on (want &&= not effective)", count(w.nativeCalls, true) == 1
      and w.nativeOn == false)
    w.seq = {}
    m.SetShooterMode(false)
    local a, b, c = index(w.seq, "freeMove:true"), index(w.seq, "mask:24511"), index(w.seq, "native:false")
    xcheck("B3 shooterMode: OFF order = locomotionEnable(true), forced mask, THEN native(false) last", a ~= nil and b ~= nil and c ~= nil
      and a < b and b < c and c == #w.seq, seqStr(w))
    xcheck("B3 shooterMode: OFF -> Locomotion owned again, align off, mask dirty re-sent, hold released",
      m.exec.locomotionOwned == true and m.shooterMode.align == false and m.shooterMode.effective == false and m.exec.holdOwned == false)
    fnRun(m, w, 10)
    xcheck("B3 shooterMode: after OFF the native body turn returns through ExecStep", w.nativeOn == true, tostring(w.nativeOn))
    xcheck("B3 shooterMode: ON / OFF log lines, one each", lines(m, "Shooter mode ON") == 1 and lines(m, "Shooter mode OFF") == 1)
  end

  -- ---- B4 300 frames with Locomotion reported active
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    w.freeMoveCalls, w.freeMoveAt, w.nativeCalls, w.frame = {}, {}, {}, 0
    fnRun(m, w, 300)
    local gapOk = true
    for i = 2, #w.freeMoveAt do if w.freeMoveAt[i] - w.freeMoveAt[i - 1] < 59 then gapOk = false end end
    xcheck("B4 shooterMode: 300 frames with Locomotion active: setNative(true) never sent", count(w.nativeCalls, true) == 0)
    xcheck("B4 shooterMode: locomotionEnable(false) re-asserted at most once per mock second (>= 59 frames apart, <= 6 in 5 s)",
      #w.freeMoveAt >= 1 and #w.freeMoveAt <= 6 and gapOk and count(w.freeMoveCalls, true) == 0, #w.freeMoveAt)
    xcheck("B4 shooterMode: one log line for the re-assert", lines(m, "reported active") == 1, lines(m, "reported active"))
  end

  -- ---- B5 align (camera 120 deg away from V)
  do
    local m = fnModule()
    local w = fnWorld({ body = 0, view = 120 })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    w.seq = {}
    local dt = 1 / 60
    local why1 = fnStep(m, w, dt)
    xcheck("B5 shooterMode: first frame: free look hold taken, align step sent", w.holds[1] == true and why1 == "shooterMode align"
      and w.aimCalls == 1, seqStr(w))
    local maxStep, mono, prevErr, frames, lastWhy = math.abs(w.aimSteps[1]), true, 120, 1, why1
    local holdsBefore
    while frames < 400 and lastWhy == "shooterMode align" do
      lastWhy = fnStep(m, w, dt)
      frames = frames + 1
      local e = math.abs(nrm(w.view - w.body))
      if e > prevErr + 1e-9 then mono = false end
      prevErr = e
    end
    for _, s in ipairs(w.aimSteps) do maxStep = math.max(maxStep, math.abs(s)) end
    xcheck("B5 shooterMode: every step <= 540 deg/s * dt, approach monotone, ends 'shooterMode aligned' within 2 s",
      maxStep <= 540 * dt + 1e-6 and mono and lastWhy == "shooterMode aligned" and frames < 120, string.format("max %.3f frames %d %s", maxStep, frames, lastWhy))
    local same = #w.bodySteps == #w.aimSteps and #w.lookSteps == #w.aimSteps
    for i = 1, #w.aimSteps do
      if not same then break end
      if math.abs(w.bodySteps[i] - w.aimSteps[i]) > 1e-9 or math.abs(w.lookSteps[i] - w.aimSteps[i]) > 1e-9 then same = false end
    end
    xcheck("B5 shooterMode: bodyStep and lookStep get the same step as applyAim", same and #w.aimSteps > 5)
    xcheck("B5 shooterMode: |err| < 2 -> camHold(false) in the same frame (no linger), align off", w.holds[#w.holds] == false
      and m.exec.holdOwned == false and m.shooterMode.align == false and math.abs(nrm(w.view - w.body)) < 2, tostring(w.holds[#w.holds]))
    local calls = w.aimCalls
    fnRun(m, w, 30)
    xcheck("B5 shooterMode: afterwards idle 'shooterMode' - no sends, no hold", w.aimCalls == calls and m.exec.why == "shooterMode"
      and #w.holds == 2)
    -- the user already holds free look: no camHold calls, the align still converges
    local m2 = fnModule()
    local w2 = fnWorld({ body = 0, view = 120, camHeld = true })
    installGame(w2)
    m2.SetShooterMode(true)
    m2.api = w2.api
    m2.SelectExecutor(w2.api)
    fnRun(m2, w2, 90)
    xcheck("B5 shooterMode: camera already held by the user: no camHold calls, the align still converges",
      #w2.holds == 0 and math.abs(nrm(w2.view - w2.body)) < 2 and m2.shooterMode.align == false)
    -- a 720 deg chase (the camera turns with V) stops once and logs once
    local m3 = fnModule()
    local w3 = fnWorld({ body = 0, view = 120 })
    installGame(w3)
    m3.SetShooterMode(true)
    m3.api = w3.api
    m3.SelectExecutor(w3.api)
    for _ = 1, 400 do
      fnStep(m3, w3, dt)
      w3.view = w3.body + 120 -- camera turns with the body
    end
    xcheck("B5 shooterMode: a 720 deg chase stops, logs once, hold released", m3.shooterMode.align == false
      and lines(m3, "Shooter align STOPPED") == 1 and m3.exec.holdOwned == false and m3.exec.why == "shooterMode",
      m3.exec.why .. " " .. lines(m3, "Shooter align STOPPED"))
  end

  -- ---- B5b pause: the game applies no request for 120 frames (pause menu), then resumes (ingame-1)
  do
    local m = fnModule()
    local w = fnWorld({ body = 0, view = 120, applyOn = false })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    local why
    for _ = 1, 120 do why = fnStep(m, w) end
    xcheck("B5b shooterMode: 120 frames without any applied request: still armed, 'waiting', sum not running away, no stop",
      m.shooterMode.align == true and why == "shooterMode align waiting" and m.shooterMode.alignSum < 30 and m.shooterMode.alignStopped == false
      and lines(m, "Shooter align STOPPED") == 0, string.format("%s sum %.1f", why, m.shooterMode.alignSum))
    xcheck("B5b shooterMode: the camera hold is kept while waiting", m.exec.holdOwned == true and w.camHeld == true)
    w.applyOn = true
    fnRun(m, w, 90)
    xcheck("B5b shooterMode: after the game resumes the align still converges onto the camera", m.shooterMode.align == false
      and math.abs(nrm(w.view - w.body)) < 2 and lines(m, "Shooter align STOPPED") == 0,
      string.format("body %.2f view %.2f", w.body, w.view))
  end

  -- ---- B5c TPP gate (ingame-6): no look fallback, kNotTpp / kStale -> waiting without holds or sends
  do
    local function gateCase(label, mut)
      local m = fnModule()
      local w = fnWorld({ body = 0, view = 120 })
      installGame(w)
      m.SetShooterMode(true)
      m.api = w.api
      m.SelectExecutor(w.api)
      mut(w)
      local why = fnRun(m, w, 30)
      xcheck("B5c shooterMode: " .. label .. " -> 'shooterMode align waiting', no applyAim, no camHold", why == "shooterMode align waiting"
        and w.aimCalls == 0 and #w.holds == 0 and m.shooterMode.align == true, why)
    end
    gateCase("viewYaw nil (the look yaw is NOT used as a fallback)", function(w) w.viewNil = true end)
    gateCase("kNotTpp (65536)", function(w) w.reasons = 65536 end)
    gateCase("kStale (131072)", function(w) w.reasons = 131072 + 262144 end)
    gateCase("reasons unreadable", function(w) w.api.locomotionReasons = function() return nil end end)
  end

  -- ---- B6 aim in Shooter
  do
    local m = fnModule()
    local w = fnWorld({ body = 0, view = 0, sig = { weaponDrawn = true, upper = 6, action = 0 } })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    fnRun(m, w, 3) -- initial align: already aligned
    xcheck("B6 shooterMode: initial align done (aligned)", m.shooterMode.align == false and m.exec.why == "shooterMode")
    w.view, w.reasons = 120, 4096 + 262144
    local why = fnRun(m, w, 3)
    xcheck("B6 shooterMode: reasons 4096|262144 (aim + disabled): the existing aim-align path runs unchanged", why == "aim align"
      and w.aimCalls >= 1 and index(w.seq, "alignHold:true") ~= nil, why)
    w.applyOn = false -- the aim ended before V caught up
    fnRun(m, w, 3)
    w.reasons, w.sig = 262144, { weaponDrawn = true, upper = 0, action = 0 }
    w.applyOn = true
    why = fnStep(m, w)
    xcheck("B6 shooterMode: aim end with err >= 2 arms the Shooter align, the episode runs", why == "shooterMode align" and m.shooterMode.align == true,
      why)
    fnRun(m, w, 90)
    xcheck("B6 shooterMode: ... and ends aligned", m.shooterMode.align == false and math.abs(nrm(w.view - w.body)) < 2)
    -- hip-fire lease never starts (Locomotion reasons carry kDisabled)
    local n = w.aimCalls
    w.sig = { weaponDrawn = true, upper = 0, action = 8 }
    why = fnRun(m, w, 30)
    xcheck("B6 shooterMode: the hip-fire lease never starts (rest ~= 0), nothing sent", m.exec.hipFireRemaining == 0 and w.aimCalls == n
      and why == "shooterMode", why)
    -- a stale camera-guard pause must not disable the ADS align (ingame-4)
    local m2 = fnModule()
    local w2 = fnWorld({ body = 0, view = 120, freeMove = true, sig = { weaponDrawn = true, upper = 6, action = 0 } })
    installGame(w2)
    m2.SetShooterMode(false)
    m2.api = w2.api
    m2.SelectExecutor(w2.api)
    m2.exec.pausedUntil, m2.exec.trips = math.huge, 3
    m2.SetShooterMode(true)
    xcheck("B6 shooterMode: ON resets trips / pausedUntil / score", m2.exec.pausedUntil == nil and m2.exec.trips == 0 and m2.exec.score == 0)
    w2.freeMove, w2.reasons = false, 4096 + 262144
    local why2 = fnRun(m2, w2, 3)
    xcheck("B6 shooterMode: with E.pausedUntil = math.huge before ON the ADS aim align still runs", why2 == "aim align", why2)
    -- the same through SelectExecutor
    local m3 = fnModule()
    local w3 = fnWorld({ body = 0, view = 120, sig = { weaponDrawn = true, upper = 6, action = 0 } })
    installGame(w3)
    m3.SetShooterMode(true)
    m3.exec.pausedUntil, m3.exec.trips = math.huge, 3
    m3.api = w3.api
    m3.SelectExecutor(w3.api)
    w3.reasons = 4096 + 262144
    xcheck("B6 shooterMode: SelectExecutor branch resets the stale pause, ADS align runs", fnRun(m3, w3, 3) == "aim align")
  end

  -- ---- B7 native missing / failing / retry
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true })
    installGame(w, { noNative = true })
    m.SetShooterMode(true)
    m.api = w.api
    w.seq = {}
    m.SelectExecutor(w.api)
    xcheck("B7 shooterMode: native missing -> not effective, call log equals OFF", m.shooterMode.effective == false
      and seqStr(w) == "setNative:false,freeMove:true,mask:24511" and m.shooterMode.nativeState == "missing", seqStr(w))
    xcheck("B7 shooterMode: one log line when the user turns it on", lines(m, "Shooter mode native missing") == 1)
    fnRun(m, w, 150)
    m.SetShooterMode(true)
    xcheck("B7 shooterMode: the retries and a second ON add no second line", lines(m, "Shooter mode native missing") == 1
      and m.shooterMode.effective == false)
    -- a native that errors at registration is retried once per second (ingame-7), one log line on success
    local m2 = fnModule()
    local w2 = fnWorld({ freeMove = false })
    installGame(w2)
    w2.nativeFail = true
    m2.SetShooterMode(true)
    m2.api = w2.api
    m2.SelectExecutor(w2.api)
    xcheck("B7 shooterMode: native errors -> inert (failed)", m2.shooterMode.effective == false and m2.shooterMode.nativeState == "failed"
      and m2.Status():find("native=failed", 1, true) ~= nil)
    local calls0 = #w2.natives
    fnRun(m2, w2, 130)
    xcheck("B7 shooterMode: failing native retried about once per second (not every frame)", #w2.natives - calls0 >= 1
      and #w2.natives - calls0 <= 3 and m2.shooterMode.effective == false, #w2.natives - calls0)
    w2.nativeFail = false
    fnRun(m2, w2, 70)
    xcheck("B7 shooterMode: retry succeeds -> effective, Locomotion switched off, one 'accepted' line", m2.shooterMode.effective == true
      and m2.shooterMode.nativeState == "ok" and lines(m2, "accepted after a retry") == 1 and count(w2.freeMoveCalls, false) >= 1)
  end

  -- ---- B8 v2 not enabled / not live (ingame-5)
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true, v2 = false })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    w.seq = {}
    m.SelectExecutor(w.api)
    xcheck("B8 shooterMode: Motion set not enabled -> inert, Locomotion behaves as OFF", m.shooterMode.effective == false
      and seqStr(w) == "setNative:false,freeMove:true,mask:24511" and m.exec.locomotionOwned == true, seqStr(w))
    fnRun(m, w, 20)
    w.v2, w.view = true, 60 -- the camera is away from V when the mode becomes effective
    local why
    for i = 1, 70 do why = fnStep(m, w); if m.shooterMode.effective then break end end
    xcheck("B8 shooterMode: motion turns enabled -> effective within one mock second, Locomotion off, align armed", m.shooterMode.effective == true
      and w.freeMoveCalls[#w.freeMoveCalls] == false and m.shooterMode.align == true and w.frame <= 20 + 61, w.frame)
    w.live = false
    fnRun(m, w, 61)
    xcheck("B8 shooterMode: the DLL reports not live -> inert again (Locomotion back on), one OFF line", m.shooterMode.effective == false
      and w.freeMoveCalls[#w.freeMoveCalls] == true and lines(m, "Shooter mode OFF") == 1)
    local seqN = #w.seq
    fnRun(m, w, 130)
    xcheck("B8 shooterMode: ... no log / call spam while it stays not live", lines(m, "Shooter mode OFF") == 1
      and lines(m, "Shooter mode ON") == 1)
    w.live = true
    fnRun(m, w, 61)
    xcheck("B8 shooterMode: live again -> effective again", m.shooterMode.effective == true and m.ShooterModeActive() == true)
    -- live unknown (nil) counts as fine
    local m2 = fnModule()
    local w2 = fnWorld({ freeMove = true, live = nil })
    installGame(w2)
    m2.SetShooterMode(true)
    m2.api = w2.api
    m2.SelectExecutor(w2.api)
    fnRun(m2, w2, 130)
    xcheck("B8 shooterMode: live unknown (nil / older api) does not block", m2.shooterMode.effective == true)
  end

  -- ---- B9 Status
  do
    local m = fnModule()
    local w = fnWorld({})
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    local st = m.Status()
    xcheck("B9 shooterMode: Status contains 'shooterMode=true eff=true native=ok align=true motion=true'",
      st:find("shooterMode=true eff=true native=ok align=true motion=true", 1, true) ~= nil, st)
    m.SetShooterMode(false)
    xcheck("B9 shooterMode: ... and 'shooterMode=false eff=false' after OFF", m.Status():find("shooterMode=false eff=false", 1, true) ~= nil)
  end

  -- ---- OFF: no Shooter call in a long run (identical call sequences to today)
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true, cmd = function(cur) return norm(cur + 3) end })
    installGame(w)
    m.SetShooterMode(false)
    m.api = w.api
    m.SelectExecutor(w.api)
    fnRun(m, w, 300)
    xcheck("OFF shooterMode: 300 frames add no Shooter native call beyond the registration one, align never armed",
      #w.natives == 1 and w.natives[1] == false and m.shooterMode.align == false and m.shooterMode.effective == false
      and index(w.seq, "freeMove:false") == nil and w.nativeOn == true)
    m.SetShooterMode(false)
    xcheck("OFF shooterMode: an extra SetShooterMode(false) only repeats the idempotent native", #w.natives == 2
      and index(w.seq, "freeMove:false") == nil)
  end

  -- ---- kill switch with Shooter: the executor disarm resets the align
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = false })
    installGame(w)
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    m.SetExecutor(false)
    xcheck("kill switch with Shooter: align disarmed, native body turn off", m.shooterMode.align == false and w.nativeOn == false
      and m.exec.mode == "disabled")
    m.SetExecutor(true)
    xcheck("kill switch with Shooter: re-selection re-applies the mode (Locomotion off, align armed)", m.shooterMode.effective == true
      and m.shooterMode.align == true and w.freeMoveCalls[#w.freeMoveCalls] == false)
  end

  -- ---- B-1 UTPPSetMotionHipLease (round 1 review): the CET hip-fire lease tells the DLL (H2 exclusion), independent of Shooter mode
  do
    local function leaseGame(w, o)
      o = o or {}
      installGame(w)
      if not o.noLease then
        Game.UTPPSetMotionHipLease = function(on)
          w.S("lease:" .. tostring(on)); w.leases = w.leases or {}; w.leases[#w.leases + 1] = on
          if w.leaseFail then error("lease boom") end
          return true
        end
      end
    end
    local function leaseList(w) local t = {} for _, v in ipairs(w.leases or {}) do t[#t + 1] = tostring(v) end return table.concat(t, ",") end
    -- Shooter OFF (today's mode): select -> false; lease start -> true; the refresh of a running lease sends nothing; end -> false
    local m = fnModule()
    local w = fnWorld({ freeMove = true, sig = { weaponDrawn = true, upper = 0, action = 0 } })
    leaseGame(w)
    m.SetShooterMode(false)
    m.api = w.api
    m.SelectExecutor(w.api)
    xcheck("B-1 lease: SelectExecutor sends false once (re-sync)", leaseList(w) == "false", leaseList(w))
    fnRun(m, w, 30)
    xcheck("B-1 lease: idle frames send nothing", leaseList(w) == "false" and m.HipLeaseCalls() == 1, leaseList(w))
    w.sig = { weaponDrawn = true, upper = 0, action = 8 }
    fnRun(m, w, 1)
    xcheck("B-1 lease: Shoot=8 with a drawn weapon starts the lease -> true", leaseList(w) == "false,true", leaseList(w))
    w.sig = { weaponDrawn = true, upper = 0, action = 8 }
    fnRun(m, w, 40)
    xcheck("B-1 lease: refreshing shots (action 8 again) send nothing", leaseList(w) == "false,true", leaseList(w))
    w.sig = { weaponDrawn = true, upper = 0, action = 0 }
    fnRun(m, w, 30)
    xcheck("B-1 lease: still inside the 0.75 s tail -> nothing yet", leaseList(w) == "false,true", leaseList(w))
    fnRun(m, w, 30)
    xcheck("B-1 lease: the tail expired -> false (exactly one call per change)", leaseList(w) == "false,true,false", leaseList(w))
    fnRun(m, w, 60)
    xcheck("B-1 lease: afterwards silent; Shooter OFF adds only the registration-time native(false)",
      leaseList(w) == "false,true,false" and #w.natives == 1 and w.natives[1] == false and index(w.seq, "freeMove:false") == nil)
    -- lease running while the executor is switched off: false at once (kill switch), re-selection starts clean
    w.sig = { weaponDrawn = true, upper = 0, action = 8 }
    fnRun(m, w, 2)
    local n = #w.leases
    m.SetExecutor(false)
    xcheck("B-1 lease: DisarmExecutor (kill switch) sends false", w.leases[#w.leases] == false and #w.leases == n + 1 and w.leases[n] == true,
      leaseList(w))
    m.SetExecutor(true)
    xcheck("B-1 lease: re-selection sends false once more (state unknown -> re-sync)", w.leases[#w.leases] == false and #w.leases == n + 2,
      leaseList(w))
    -- new session: the sent flag is forgotten; the first ExecStep with a player re-syncs the current value
    w.sig = { weaponDrawn = true, upper = 0, action = 0 }
    fnRun(m, w, 80)
    m.exec.hipLeaseSent = nil
    n = #w.leases
    fnRun(m, w, 3)
    xcheck("B-1 lease: after a new session the first ExecStep re-sends the current value (false) once", #w.leases == n + 1
      and w.leases[#w.leases] == false, leaseList(w))
    -- Shooter ON: the lease never starts (Locomotion reasons carry kDisabled) -> no sends but the selection one
    local m2 = fnModule()
    local w2 = fnWorld({ freeMove = false, reasons = 262144, sig = { weaponDrawn = true, upper = 0, action = 8 } })
    leaseGame(w2)
    m2.SetShooterMode(true)
    m2.api = w2.api
    m2.SelectExecutor(w2.api)
    fnRun(m2, w2, 120)
    xcheck("B-1 lease: Shooter ON, Shoot=8: no lease (rest ~= 0) -> only the selection false", leaseList(w2) == "false", leaseList(w2))
    -- older DLL: native missing -> silent, no log, no error, the flag is tracked (no per-frame retry)
    local m3 = fnModule()
    local w3 = fnWorld({ freeMove = true, sig = { weaponDrawn = true, upper = 0, action = 8 } })
    leaseGame(w3, { noLease = true })
    m3.SetShooterMode(false)
    m3.api = w3.api
    m3.SelectExecutor(w3.api)
    fnRun(m3, w3, 120)
    xcheck("B-1 lease: native missing -> silent (no log line about it), nothing else changed", lines(m3, "HipLease") == 0
      and m3.HipLeaseCalls() == 0 and m3.exec.hipLeaseSent == true and index(w3.seq, "lease:true") == nil)
    -- failing native: retried next frame, silently, until it works
    local m4 = fnModule()
    local w4 = fnWorld({ freeMove = true, sig = { weaponDrawn = true, upper = 0, action = 8 } })
    leaseGame(w4)
    w4.leaseFail = true
    m4.SetShooterMode(false)
    m4.api = w4.api
    m4.SelectExecutor(w4.api)
    fnRun(m4, w4, 5)
    w4.leaseFail = false
    fnRun(m4, w4, 3)
    xcheck("B-1 lease: an erroring native is retried and then holds the right state (true)", w4.leases[#w4.leases] == true
      and m4.exec.hipLeaseSent == true and lines(m4, "HipLease") == 0)
    -- wired in game: no-player frames forget the flag (new session); selection sends false
    local world = { body = 0, cam = 0, stick = 0 }
    world.camMod = { rt = { freeHeld = false } }
    world.camMod.FreeLook = function(on) world.camMod.rt.freeHeld = on and true or false; return world.camMod.rt.freeHeld end
    local leaseWired = {}
    local _, _, _, werr = wired(world, { noPlayer = true, gameExtra = {
      UTPPSetMotionHipLease = function(on) leaseWired[#leaseWired + 1] = on; return true end },
      inspect = function(mm) leaseWired.sentAtEnd = mm.exec.hipLeaseSent end })
    xcheck("B-1 lease: wired - selection sent false, a session without player forgets the flag (re-sync on return)",
      werr == nil and leaseWired[1] == false and #leaseWired >= 1 and leaseWired.sentAtEnd == nil, tostring(werr))
  end

  -- ---- B-2 no spurious 'reported active' log at a poll-driven ON; a real re-enable by someone else is still logged
  do
    local m = fnModule()
    local w = fnWorld({ freeMove = true, live = false })
    installGame(w)
    local en = w.api.locomotionEnable
    w.api.locomotionEnable = function(on) en(on); w.freeMove = on == true end -- the mock Locomotion follows the enable calls
    m.SetShooterMode(true)
    m.api = w.api
    m.SelectExecutor(w.api)
    xcheck("B-2: not live at selection -> inert, Locomotion enabled", m.shooterMode.effective == false and w.freeMove == true)
    fnRun(m, w, 30)
    w.live = true
    local before = count(w.freeMoveCalls, false)
    for _ = 1, 70 do fnStep(m, w); if m.shooterMode.effective then break end end
    xcheck("B-2: the poll switched the mode on in this frame: exactly ONE locomotionEnable(false), no re-assert, no log",
      m.shooterMode.effective == true and count(w.freeMoveCalls, false) == before + 1 and lines(m, "reported active") == 0
      and m.shooterMode.assertLogged == false, count(w.freeMoveCalls, false) - before)
    fnRun(m, w, 100)
    xcheck("B-2: afterwards (Locomotion really off) nothing is re-asserted or logged", count(w.freeMoveCalls, false) == before + 1
      and lines(m, "reported active") == 0)
    w.freeMove = true -- another module / a CET reload switches Locomotion on again
    fnRun(m, w, 70)
    xcheck("B-2: a real re-enable is re-asserted within a second and logged once", count(w.freeMoveCalls, false) == before + 2
      and lines(m, "reported active") == 1 and w.freeMove == false, lines(m, "reported active"))
    w.freeMove = true
    fnRun(m, w, 70)
    xcheck("B-2: ... and further re-enables are re-asserted without more log lines", count(w.freeMoveCalls, false) == before + 3
      and lines(m, "reported active") == 1)
  end

  -- ---- wired in game: motionEnabled comes from GetMod("utpp_motion").want.enabled, shooterModeLive from Game.UTPPMotionShooterModeLive
  do
    local function wiredFn(o)
      local world = { body = 0, cam = 0, stick = 0 }
      world.camMod = { rt = { freeHeld = false } }
      world.camMod.FreeLook = function(on) world.camMod.rt.freeHeld = on and true or false; return world.camMod.rt.freeHeld end
      local nativeCalls = {}
      local regs = {}
      local core = { RegisterSection = function() end,
                     Register = function(def) regs[#regs + 1] = def
                       if def.apply then def.apply(def.key == "bodyturn.shooterMode" and true or def.default) end end }
      local state
      local _, _, _, err = wired(world, { core = core, mods = o.mods, gameExtra = {
        UTPPSetMotionShooterMode = function(on) nativeCalls[#nativeCalls + 1] = on; return true end,
        UTPPMotionShooterModeLive = function() return o.live end },
        inspect = function(mm) state = { eff = mm.shooterMode.effective, v2 = mm.shooterMode.motionOk, status = mm.Status(), regs = #regs } end })
      return state, err, nativeCalls
    end
    local st, err, calls = wiredFn({ mods = { utpp_motion = { want = { enabled = true } } }, live = true })
    xcheck("wired shooterMode: motion enabled + DLL live -> effective, setting is the 12th registration, native(true) sent",
      err == nil and st ~= nil and st.eff == true and st.regs == 12 and calls[1] == true, tostring(err))
    st, err = wiredFn({ mods = { utpp_motion = { want = { enabled = false } } }, live = true })
    xcheck("wired shooterMode: utpp_motion setting off -> inert", err == nil and st ~= nil and st.eff == false and st.v2 == false)
    st, err = wiredFn({ mods = {}, live = true })
    xcheck("wired shooterMode: utpp_motion module missing counts as motion off -> inert", err == nil and st ~= nil and st.eff == false)
    st, err = wiredFn({ mods = { utpp_motion = { want = { enabled = true } } }, live = false })
    xcheck("wired shooterMode: DLL not live (UTPPMotionShooterModeLive false) -> inert", err == nil and st ~= nil and st.eff == false)
  end

  Game = savedGame
end

-- ================================================================ HIP-2 (locomotion probe 2026-10-31 22:57:36.75 / 37.76): the CET
-- hipfire lease mirrors HipFirePoseHold (isolated 1.2 s, sustained 2 s within 2.5 s, sprint / unreadable sprint -> 0.75 s). The fixed
-- 0.75 s lease dropped 50 ms before the second shot of a 1.0 s cadence (HipLease 0 at 37.708, 1 at 37.758): native body turn and the
-- H2 exclusion switched for 50 ms between the shots.
do
  local function mutantModule(from, to)
    local f = assert(io.open(UTPP_BODYTURN_PATH, "rb"))
    local src = f:read("*a"); f:close()
    local a, b = src:find(from, 1, true)
    assert(a and not src:find(from, b + 1, true), "mutation anchor must exist exactly once: " .. from)
    local m = assert(loadstring(src:sub(1, a - 1) .. to .. src:sub(b + 1), "=utpp_bodyturn_hip2_mutant"))()
    m.exec.aimSmooth = true
    m.lines = {}
    m.log = function(msg) m.lines[#m.lines + 1] = msg end
    return m
  end
  local kRun = 0.217 -- Shoot=8 run of the log (36.749 -> 36.968)
  -- runs: list of Shoot run start times (s); sprint(t) -> true / false / "nil" (unreadable); returns a frame list {t, lease, act}
  local function run(m, runs, hz, total, sprint, action)
    local dt, out = 1 / hz, {}
    for f = 0, math.floor(total * hz + 0.5) - 1 do
      local t = f * dt
      local act = 0
      for _, s in ipairs(runs) do if t >= s - 1e-9 and t < s + kRun - 1e-9 then act = 8 end end
      if action then act = action(t, act) end
      local sig = {weaponDrawn = true, upper = 0, action = act, sprint = false}
      if sprint then
        local v = sprint(t)
        if v == "nil" then sig.sprint = nil else sig.sprint = v end
      end
      out[#out + 1] = {t = t, lease = m.HipFireStep(sig, 8192, dt), act = act}
    end
    return out
  end
  -- lease held over [a, b]
  local function heldThrough(frames, a, b)
    for _, fr in ipairs(frames) do if fr.t >= a and fr.t <= b and not fr.lease then return false end end
    return true
  end
  -- s from the last Shoot frame to the last lease frame
  local function tail(frames)
    local lastShot, lastLease = nil, nil
    for _, fr in ipairs(frames) do
      if fr.act == 8 then lastShot = fr.t end
      if fr.lease then lastLease = fr.t end
    end
    return (lastLease or 0) - (lastShot or 0)
  end
  local function cadence(m, hz) -- the recorded 1.0 s cadence, sprint known false
    local fr = run(m, {0, 1.009}, hz, 2.0)
    return heldThrough(fr, 0, 1.009 + 0.5)
  end
  local function sustained(m, hz) -- a third shot 1.8 s after the second: the engagement keeps the lease (2 s)
    local fr = run(m, {0, 1.009, 2.809}, hz, 3.5)
    return heldThrough(fr, 0, 2.809 + 0.5)
  end
  local function sprintCut(m, hz) -- isolated shot, sprint requested from 0.1 s after it: the former 0.75 s
    local fr = run(m, {0}, hz, 2.0, function(t) return t >= kRun + 0.1 end)
    return math.abs(tail(fr) - 0.75) <= 1.5 / hz, tail(fr)
  end
  for _, hz in ipairs({30, 60, 144}) do
    local dt = 1 / hz
    xcheck("HIP-2 lease " .. hz .. "Hz: the recorded 1.0 s cadence keeps the lease between the shots (no BodyTurn / H2 toggle)",
      cadence(freshModule(true), hz))
    local fr = run(freshModule(true), {0}, hz, 2.0)
    xcheck("HIP-2 lease " .. hz .. "Hz: an isolated shot leases 1.2 s after its last Shoot frame (HipFirePoseHold kIsolatedHoldS)",
      math.abs(tail(fr) - 1.2) <= 1.5 * dt, tail(fr))
    xcheck("HIP-2 lease " .. hz .. "Hz: a Shoot run 1.8 s after the previous one is sustained (2 s lease)", sustained(freshModule(true), hz))
    fr = run(freshModule(true), {0, 2.8}, hz, 5.0)
    xcheck("HIP-2 lease " .. hz .. "Hz: shots 2.8 s apart stay isolated (1.2 s)", math.abs(tail(fr) - 1.2) <= 1.5 * dt, tail(fr))
    local okCut, cut = sprintCut(freshModule(true), hz)
    xcheck("HIP-2 lease " .. hz .. "Hz: a sprint cuts the isolated lease to the former 0.75 s", okCut, cut)
    fr = run(freshModule(true), {0, 1.009}, hz, 3.5, function(t) return t >= 1.009 + kRun + 0.1 end)
    xcheck("HIP-2 lease " .. hz .. "Hz: a sprint ends a sustained lease 0.75 s after the last Shoot frame",
      math.abs(tail(fr) - 0.75) <= 1.5 * dt, tail(fr))
    fr = run(freshModule(true), {0, 1.009}, hz, 3.5, function() return "nil" end)
    xcheck("HIP-2 lease " .. hz .. "Hz: an unreadable sprint state keeps the former fixed 0.75 s lease",
      math.abs(tail(fr) - 0.75) <= 1.5 * dt and not heldThrough(fr, 0, 1.009), tail(fr))
    -- a reload between two shots ends the lease but not the engagement; an undrawn weapon forgets it
    local m = freshModule(true)
    fr = run(m, {0, 1.6}, hz, 1.6 + 2 * dt, nil, function(t, act) if t >= 0.4 and t < 1.4 then return 2 end return act end)
    local reloadOff = true
    for _, x in ipairs(fr) do if x.act == 2 and x.lease then reloadOff = false end end
    xcheck("HIP-2 lease " .. hz .. "Hz: a reload ends the lease, the next shot within 2.5 s is sustained (2 s)",
      reloadOff and math.abs(m.exec.hipFireRemaining - 2.0) < 1e-6, m.exec.hipFireRemaining)
    m = freshModule(true)
    m.HipFireStep({weaponDrawn = true, upper = 0, action = 8, sprint = false}, 8192, dt)
    m.HipFireStep({weaponDrawn = false, upper = 0, action = 0, sprint = false}, 8192, dt)
    m.HipFireStep({weaponDrawn = true, upper = 0, action = 8, sprint = false}, 8192, dt)
    xcheck("HIP-2 lease " .. hz .. "Hz: an undrawn weapon forgets the engagement (next shot isolated)",
      math.abs(m.exec.hipFireRemaining - 1.2) < 1e-6, m.exec.hipFireRemaining)
    -- the DLL hold's grounded rule (combat probe 2026-10-24 shoot_jump): a jump after a burst ends the lease on its first air frame,
    -- a Shoot frame in the air leases only itself, the first shot after the landing is isolated
    local function airCase(mm)
      local sig = {weaponDrawn = true, upper = 0, action = 8, sprint = false, air = false}
      for _ = 1, math.ceil(kRun * hz) do mm.HipFireStep(sig, 8192, dt) end
      sig.action = 0
      for _ = 1, math.ceil(0.3 * hz) do mm.HipFireStep(sig, 8192, dt) end
      sig.action = 8
      for _ = 1, math.ceil(kRun * hz) do mm.HipFireStep(sig, 8192, dt) end -- second shot: sustained (2 s)
      sig.action = 0
      for _ = 1, math.ceil(0.2 * hz) do mm.HipFireStep(sig, 8192, dt) end
      local before = mm.exec.hipFireRemaining > 1.0
      sig.air = true -- take-off (LocomotionDetailed Jump 18)
      local offAtTakeoff = not mm.HipFireStep(sig, 8192, dt)
      sig.action = 8
      local airShot = mm.HipFireStep(sig, 8192, dt)
      sig.action = 0
      local airAfter = not mm.HipFireStep(sig, 8192, dt)
      sig.air = false
      mm.HipFireStep(sig, 8192, dt)
      sig.action = 8
      mm.HipFireStep(sig, 8192, dt)
      return before and offAtTakeoff and airShot and airAfter and math.abs(mm.exec.hipFireRemaining - 1.2) < 1e-6
    end
    xcheck("HIP-2 lease " .. hz .. "Hz: a jump after a burst ends the lease at the take-off, an air shot leases only itself, the next shot is isolated",
      airCase(freshModule(true)))
    xcheck("HIP-2 lease " .. hz .. "Hz: mutation [no air rule] is caught", not airCase(mutantModule("elseif sig.air == true then", "elseif false then")))
    -- mutations of the shipped source must be caught
    xcheck("HIP-2 lease " .. hz .. "Hz: mutation [isolated lease 0.75 s] is caught by the 1.0 s cadence",
      not cadence(mutantModule("= 1.2, 2.0, 2.5, 0.75", "= 0.75, 2.0, 2.5, 0.75"), hz))
    xcheck("HIP-2 lease " .. hz .. "Hz: mutation [no sustained lease] is caught",
      not sustained(mutantModule("(E.hipFireSustained and E.hipFireSustainedHold or E.hipFireIsolatedHold)", "E.hipFireIsolatedHold"), hz))
    xcheck("HIP-2 lease " .. hz .. "Hz: mutation [sprint ignored] is caught",
      not sprintCut(mutantModule("if sig.sprint ~= false and", "if false and"), hz))
  end
end

-- melee strike turn (bodyturn.meleeStrikeTurn): V turns toward the direction of the game's fist hit (api.meleeStrikeYaw, the tracked
-- target) in an eased, capped turn; nothing without the signal, the melee bit, third person, the setting or an armed executor
do
  local function mutantStrike(from, to)
    local f = assert(io.open(UTPP_BODYTURN_PATH, "rb"))
    local src = f:read("*a"); f:close()
    local a, b = src:find(from, 1, true)
    assert(a and not src:find(from, b + 1, true), "mutation anchor must exist exactly once: " .. from)
    local m = assert(loadstring(src:sub(1, a - 1) .. to .. src:sub(b + 1), "=utpp_bodyturn_strike_mutant"))()
    m.exec.aimSmooth = true
    m.lines = {}
    m.log = function(msg) m.lines[#m.lines + 1] = msg end
    return m
  end
  -- world: body yaw 0, melee attack state (reasons 2048) with a hit direction of `strike` degrees.
  -- o.lag = 1: a request lands one frame late; o.drop: every second request is queued but never applied (the executor drops stale
  -- requests and ones that meet a game adjustTransform)
  local function strikeWorld(m, o)
    o = o or {}
    local w = newWorld({ utppCamera = true, locomotion = function() return false end, arm = o.arm })
    w.reasons, w.strike, w.holds, w.looks, w.queue, w.sent = 2048, o.strike, {}, {}, {}, 0
    w.lag = o.lag or 0
    w.api.locomotionReasons = function() return w.reasons end
    w.api.meleeStrikeYaw = function() return w.strike end
    w.api.alignHold = function(on) w.holds[#w.holds + 1] = on; return true end
    w.api.lookStep = function(step) w.looks[#w.looks + 1] = step; return true end
    if o.reject then
      w.api.applyAim = function() return false end
    else
      w.api.applyAim = function(yaw)
        w.calls.apply[#w.calls.apply + 1] = yaw
        w.sent = w.sent + 1
        if not (o.drop and w.sent % 2 == 0) then w.queue[#w.queue + 1] = { yaw = yaw, due = w.frame + w.lag } end
        return true
      end
    end
    m.SelectExecutor(w.api)
    m.SetStrikeTurn(o.on ~= false)
    return w
  end
  local function runStrike(m, w, n, dt)
    local why, maxStep, atHit = nil, 0, nil
    for i = 1, n do
      w.frame = w.frame + 1
      local before = w.body
      why = m.ExecStep(w.api, dt or 1 / 60)
      -- requests are applied by the next locomotion update (lag 0: this frame)
      local rest = {}
      for _, q in ipairs(w.queue) do
        if q.due <= w.frame then w.body = q.yaw else rest[#rest + 1] = q end
      end
      w.queue = rest
      maxStep = math.max(maxStep, math.abs(norm(w.body - before)))
      if i == 12 then atHit = w.body end -- 0.2 s after the strike starts (attackEffectDelay) at 60 fps
    end
    return why, maxStep, atHit
  end

  -- 1) default off: no turn although the signal is there
  do
    local m = freshModule(true)
    xcheck("strike turn: default OFF", m.exec.strikeTurn == false)
    local w = strikeWorld(m, { strike = 40, on = false })
    local why = runStrike(m, w, 20)
    xcheck("strike turn: off -> nothing applied", #w.calls.apply == 0 and why == "locomotion inactive", why)
  end
  -- 2) 40 deg: eased turn, no overshoot, <= 540 deg/s, ends within the deadband, heading lock on then off, exact look steps
  local function turnCase(m)
    local w = strikeWorld(m, { strike = 40 })
    local why, maxStep = runStrike(m, w, 18)
    local sumLook = 0
    for _, s in ipairs(w.looks) do sumLook = sumLook + s end
    local ok = w.body > 40 - m.exec.strikeDeadband - 1e-6 and w.body <= 40 + 1e-9 and maxStep <= 540 / 60 + 1e-9
      and why == "strike aligned" and #w.looks == #w.calls.apply and math.abs(sumLook - w.body) < 1e-6
      and #w.holds == 2 and w.holds[1] == true and w.holds[2] == false
    return ok, string.format("body %.2f maxStep %.2f why %s holds %d", w.body, maxStep, tostring(why), #w.holds)
  end
  do
    local ok, d = turnCase(freshModule(true))
    xcheck("strike turn: 40 deg turn is eased, <= 540 deg/s, ends within the deadband, no overshoot, heading lock on then off, exact look steps", ok, d)
  end
  -- 2b) the turn is done by the hit (frame 12 at 60 fps): ideal, one frame late, every second request dropped
  do
    local function atHit(o)
      local m = freshModule(true)
      local w = strikeWorld(m, o)
      local _, _, hit = runStrike(m, w, 30)
      return hit, w.body
    end
    local h0, f0 = atHit({ strike = 40 })
    local h1, f1 = atHit({ strike = 40, lag = 1 })
    local h2, f2 = atHit({ strike = 40, drop = true })
    xcheck("strike turn: ideal executor - within 6 deg of the hit direction at the hit (0.2 s), no overshoot", math.abs(40 - h0) <= 6 and h0 <= 40 + 1e-9
      and f0 <= 40 + 1e-9, string.format("%.2f / %.2f", h0, f0))
    xcheck("strike turn: requests landing one frame late - within 6 deg at the hit, no overshoot", math.abs(40 - h1) <= 6 and f1 <= 40 + 1e-9,
      string.format("%.2f / %.2f", h1, f1))
    xcheck("strike turn: every second request dropped - within 9 deg at the hit and 6 deg after 0.5 s, no overshoot", math.abs(40 - h2) <= 9
      and math.abs(40 - f2) <= 6 and f2 <= 40 + 1e-9, string.format("%.2f / %.2f", h2, f2))
  end
  -- 3) cap: a hit 120 deg away turns V at most strikeMax (50) from where the episode started, then holds - also with late / dropped requests
  local function capCase(m, o)
    local w = strikeWorld(m, o or { strike = 120 })
    local why = runStrike(m, w, 90)
    return w.body >= 50 - m.exec.strikeDeadband - 1e-6 and w.body <= 50 + 1e-6 and why == "strike hold" and w.holds[#w.holds] == false, w.body
  end
  do
    local ok, body = capCase(freshModule(true))
    xcheck("strike turn: a 120 deg hit direction is capped at 50 deg per episode, then holds", ok, body)
    local okL, bodyL = capCase(freshModule(true), { strike = 120, lag = 2 })
    xcheck("strike turn: the cap holds with requests landing two frames late (no overshoot past 50)", okL, bodyL)
    local okD, bodyD = capCase(freshModule(true), { strike = 120, drop = true })
    xcheck("strike turn: the cap holds with every second request dropped", okD, bodyD)
    local okMut = capCase(mutantStrike("local goal = E.strikeStart + math.max(-E.strikeMax, math.min(E.strikeMax, off))",
      "local goal = E.strikeStart + off"))
    xcheck("strike turn: mutation [no cap] is caught", not okMut)
  end
  -- 4) episode: a gap of > 0.5 s without the signal starts a new episode from V's current facing, a short gap does not
  do
    local m = freshModule(true)
    local w = strikeWorld(m, { strike = 120 })
    runStrike(m, w, 60)
    local b1 = w.body
    w.strike = nil
    runStrike(m, w, 20) -- 0.33 s: same episode
    w.strike = 170
    runStrike(m, w, 40)
    xcheck("strike turn: a 0.33 s gap keeps the episode (V stays within 50 deg of where it started)", w.body <= 50 + 1e-6 and w.body >= b1 - 1e-6, w.body)
    w.strike = nil
    runStrike(m, w, 40) -- 0.67 s without a signal
    w.strike = 170
    runStrike(m, w, 60)
    xcheck("strike turn: after > 0.5 s without a signal a new episode may turn 50 deg again", w.body - b1 >= 50 - m.exec.strikeDeadband - 1e-6
      and w.body - b1 <= 50 + 1e-6, w.body)
  end
  -- 5) gates: no signal / sentinel / NaN / no melee bit / not third person / not armed / inside the start threshold / rejected request
  do
    local function noTurn(setup, factory)
      local m = (factory or freshModule)(true)
      local w = strikeWorld(m, { strike = 40 })
      setup(w)
      runStrike(m, w, 10)
      return #w.calls.apply == 0 and #w.looks == 0 and #w.holds == 0
    end
    xcheck("strike turn: no tracked target (nil) -> no turn", noTurn(function(w) w.strike = nil end))
    xcheck("strike turn: sentinel 1000 -> no turn", noTurn(function(w) w.strike = 1000 end))
    xcheck("strike turn: NaN -> no turn", noTurn(function(w) w.strike = 0 / 0 end))
    xcheck("strike turn: no melee attack bit (aim / hold bit 4096 only) -> no turn", noTurn(function(w) w.reasons = 4096 end))
    xcheck("strike turn: not third person (bit 65536) -> no turn", noTurn(function(w) w.reasons = 2048 + 65536 end))
    xcheck("strike turn: executor not armed -> no turn", noTurn(function(w) w.api.arm = function() return false end end))
    xcheck("strike turn: inside the start threshold (7 deg) -> no turn, no heading lock", noTurn(function(w) w.strike = 7 end))
    xcheck("strike turn: V moving faster than 1 m/s -> no turn (legs are motion matching)", noTurn(function(w) w.api.speed = function() return 3.5 end end))
    local m = freshModule(true)
    local w = strikeWorld(m, { strike = 40, reject = true })
    local why = runStrike(m, w, 10)
    xcheck("strike turn: a rejected request sends no look step and does not move V", #w.looks == 0 and w.body == 0
      and why == "strike turn rejected", why)
    local function mutGate()
      return mutantStrike("math.floor(reasons / 2048) % 2 == 1 and ", "")
    end
    local gateHeld = noTurn(function(w) w.reasons = 4096 end, function() return mutGate() end)
    xcheck("strike turn: mutation [no melee bit gate] is caught", not gateHeld)
  end
  -- 5b) hysteresis: a target jittering around the 8 / 4 deg thresholds does not toggle the heading lock every frame
  do
    local m = freshModule(true)
    local w = strikeWorld(m, { strike = 6 })
    for i = 1, 60 do
      w.strike = (i % 2 == 0) and 9 or 5 -- crosses the 8 deg start only on the even frames, never the 4 deg stop
      runStrike(m, w, 1)
    end
    xcheck("strike turn: a target jittering 5 / 9 deg gives one lock engage and release at most", #w.holds <= 2, #w.holds)
  end
  -- 6) the heading lock is released when the attack state ends
  do
    local m = freshModule(true)
    local w = strikeWorld(m, { strike = 40 })
    runStrike(m, w, 5)
    w.reasons = 0
    w.strike = nil
    runStrike(m, w, 2)
    xcheck("strike turn: the heading lock is released when the attack state ends", w.holds[#w.holds] == false and m.exec.strikeHeld ~= true)
  end
  -- 6b) with the setting ON the aim align (hold / block / gun aim, bit 4096) keeps its heading lock exactly as before:
  -- one engage while turning, one release once aligned (the strike step must not release a lock it does not own)
  do
    local function aimCase(m)
      local w = strikeWorld(m, { strike = nil })
      w.reasons = 4096
      w.body, w.cam, w.free = -16, 166, true
      local why = runStrike(m, w, 40)
      local seq = {}
      for i, v in ipairs(w.holds) do seq[i] = tostring(v) end
      return math.abs(norm(w.cam - w.body)) < 2 and #w.holds == 2 and w.holds[1] == true and w.holds[2] == false,
        table.concat(seq, ",") .. " " .. tostring(why)
    end
    -- the legacy aim policy of the original aim align test (no weapon signals needed)
    local ok, d = aimCase(freshModule())
    xcheck("strike turn ON: aim align unchanged - reaches the look, lock engaged once and released once", ok, d)
    local okMut = aimCase(mutantStrike("    if E.strikeHeld then\n      if not E.alignHeld then try(api.alignHold, false) end\n      E.strikeHeld = false\n    end\n  end\n  -- melee attack state",
      "    try(api.alignHold, false); E.alignHeld = false\n  end\n  -- melee attack state"))
    xcheck("strike turn: mutation [releases the aim align's lock] is caught", not okMut)
  end
  -- 7) the setting is registered (default off) after aimSmooth and before shooterMode
  do
    local m = freshModule(true)
    local regs = {}
    m.RegisterSettings({ RegisterSection = function() end,
                         Register = function(def) regs[#regs + 1] = def; if def.apply then def.apply(def.default) end end })
    local d
    for _, r in ipairs(regs) do if r.key == "bodyturn.meleeStrikeTurn" then d = r end end
    xcheck("strike turn: registered, bool, default false, section bodyturn, English text", d ~= nil and d.type == "bool" and d.default == false
      and d.section == "bodyturn" and type(d.label) == "string" and type(d.desc) == "string" and d.desc:find("tracked target", 1, true) ~= nil)
    d.apply(true)
    local onOk = m.exec.strikeTurn == true
    d.apply(false)
    xcheck("strike turn: the setting switches the turn on and off", onOk and m.exec.strikeTurn == false)
  end
end

print(string.format("utpp_bodyturn 0.6.3 executor + weapon turn: %d passed, %d failed", X.pass, X.fail))
assert(X.fail == 0, "utpp_bodyturn executor / weapon turn test failed")

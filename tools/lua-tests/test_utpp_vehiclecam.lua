-- UTPP offline check of cet/utpp_vehiclecam/init.lua (controller on UTPP's own camera): request logic while entering /
-- driving / exiting, exit match (game camera pose sources), pull-out timing, frame math, A/B setters and Status.
-- Needs UTPP_VEHICLECAM_PATH.
local V = dofile(UTPP_VEHICLECAM_PATH)
assert(type(V) == "table" and type(V.NewController) == "function")

local pass, fail = 0, 0
local function check(name, cond, detail)
  if cond then pass = pass + 1 else fail = fail + 1; print("FAIL " .. name .. (detail and (" " .. detail) or "")) end
end

-- controller on the vehicle path: utpp_camera answers "ready" (opts.camReady == false = not held-ready)
local function newOwn(cfg, camReady)
  local c = V.NewController(cfg)
  c.own = true
  c.ownCamReady = function() return camReady ~= false end
  return c
end

-- ride(): drives Step through on foot -> entering (Transition 4) -> driving (1) -> exit (4) -> on foot; returns the
-- perspective requests and the controller.
local function ride(opts)
  opts = opts or {}
  local c = newOwn(opts.cfg, opts.camReady)
  if opts.car then c.vehCar = true; c.cfg.passengerChase = opts.chase == true end -- 2026-11-01: the passenger / late-entry requests are car only
  local t, dt = 0, 1 / 60
  local requests, exitKeep = {}, {}
  local active = opts.startActive or 0
  local function frame(obs)
    t = t + dt
    local p = c.Step(t, obs)
    if p and c.lastReqKind == "exitKeep" then -- MOTO-1 A exit keep requests are listed apart (r.exitKeep)
      exitKeep[#exitKeep + 1] = { t = t, p = p, state = obs.vehicleState }
    elseif p then
      requests[#requests + 1] = { t = t, p = p, state = obs.vehicleState }
      if opts.acceptAfter and #requests >= opts.acceptAfter then active = p end
    end
  end
  for _ = 1, 30 do frame({ vehicleState = 0, mounted = false }) end
  for _ = 1, 90 do
    frame({ vehicleState = opts.enterState or 4, mounted = true, driver = opts.driver, activePersp = active,
            persistentPersp = opts.persistent or 0, seat = opts.seat, driverSeatFree = opts.driverSeatFree })
  end
  for _ = 1, (opts.driveFrames or 120) do
    frame({ vehicleState = opts.seatState or 1, mounted = true, driver = opts.driver, activePersp = active,
            persistentPersp = opts.persistent or 0 })
  end
  local preExit = #requests
  for _ = 1, 60 do frame({ vehicleState = 4, mounted = true, driver = opts.driver, activePersp = active }) end
  local duringExit = #requests - preExit
  for _ = 1, 24 do frame({ vehicleState = 0, mounted = false }) end
  return { requests = requests, c = c, duringExit = duringExit, exitKeep = exitKeep }
end

do -- driver, the game accepts the 1st request
  local r = ride({ driver = true, acceptAfter = 1 })
  check("driver: one request", #r.requests == 1, tostring(#r.requests))
  -- 0.3.9: first request once the vehicle PSM is in Driving (state 1), like vanilla DriveEvents.OnEnter
  check("driver: first request in Driving (state 1)", r.requests[1] and r.requests[1].state == 1)
  check("driver: TPPClose by default (persistent FPP)", r.requests[1] and r.requests[1].p == 1)
  check("driver: done", r.c.stats.done == 1 and r.c.stats.failed == 0)
  check("driver: no request during exit", r.duringExit == 0)
  check("driver: mount hold, handover and exit hold happened", r.c.stats.ownMounts == 1 and r.c.stats.ownHandovers == 1
    and r.c.stats.ownExits == 1, tostring(r.c.stats.ownMounts) .. "/" .. tostring(r.c.stats.ownHandovers) .. "/"
    .. tostring(r.c.stats.ownExits))
end
do -- the game accepts only once seated (requests repeat every 0.25 s, then stop)
  local r = ride({ driver = true, acceptAfter = 7 })
  check("late accept: 7 requests", #r.requests == 7, tostring(#r.requests))
  local ok = true
  for i = 2, #r.requests do
    if r.requests[i].t - r.requests[i - 1].t < 0.25 - 1e-6 then ok = false end
  end
  check("late accept: interval >= 0.25 s", ok)
end
do -- persistent TPPFar is kept
  local r = ride({ driver = true, acceptAfter = 1, persistent = 3 })
  check("persistent TPPFar honoured", r.requests[1] and r.requests[1].p == 3)
end
do -- fixed perspective
  local r = ride({ driver = true, acceptAfter = 1, persistent = 3, cfg = { perspective = 2 } })
  check("fixed TPPMedium", r.requests[1] and r.requests[1].p == 2)
end
do -- the game never accepts: bounded, then stops
  local r = ride({ driver = true })
  check("never accepted: bounded requests", #r.requests <= V.DEFAULTS.keepMaxRequests and #r.requests > 0)
  check("never accepted: failed", r.c.stats.failed == 1)
  check("never accepted: no request during exit", r.duringExit == 0)
end
do -- already TPP (vanilla persistent) -> nothing to do
  local r = ride({ driver = true, startActive = 1 })
  check("already TPP: no request", #r.requests == 0 and r.c.stats.done == 1)
end
do -- passenger seat (the driver seat is taken, no seat switch): never armed, no mount hold; 2026-11-01: every entry starts
  -- in third person, so the passenger state (vanilla PassengerEvents.OnEnter queued FPP) gets one TPP request
  local r = ride({ driver = false, seatState = 3, seat = "seat_front_right", driverSeatFree = false, acceptAfter = 1, car = true })
  check("passenger: one TPPClose request in the passenger state (after the vanilla FPP), not armed, no mount hold",
    #r.requests == 1 and r.requests[1].state == 3 and r.requests[1].p == 1 and r.c.stats.armed == 0
    and (r.c.stats.ownMounts or 0) == 0 and (r.c.stats.passengerRequests or 0) == 1)
  local b = ride({ driver = false, seatState = 3, seat = "seat_back_left", acceptAfter = 1, car = true })
  check("passenger (back seat, never switched to the driver seat): one TPP request, not armed, no mount hold",
    #b.requests == 1 and b.requests[1].state == 3 and b.c.stats.armed == 0 and (b.c.stats.ownMounts or 0) == 0)
  local o = ride({ driver = false, seatState = 3, seat = "seat_back_left", acceptAfter = 1, cfg = { forceTppPassenger = false }, car = true })
  check("passenger, forceTppPassenger off: no request (previous behaviour)", #o.requests == 0)
  local f = ride({ driver = false, seatState = 3, seat = "seat_back_left", acceptAfter = 1, cfg = { forceTppOnEnter = false }, car = true })
  check("passenger, forceTppOnEnter off: no request", #f.requests == 0)
  local n = ride({ driver = false, seatState = 3, seat = "seat_back_left", car = true })
  check("passenger, never accepted: bounded retries (<= keepMaxRequests), none in the exit", #n.requests > 1
    and #n.requests <= V.DEFAULTS.keepMaxRequests and n.duringExit == 0, tostring(#n.requests))
  local k = ride({ driver = false, seatState = 3, seat = "seat_back_left", startActive = 2, car = true })
  check("passenger already in a TPP vehicle camera: no request", #k.requests == 0 and k.c.passengerForced ~= nil)
end
do -- driver combat state: no request in state 6
  local r = ride({ driver = true, seatState = 6 })
  local inSeat = 0
  for _, q in ipairs(r.requests) do if q.state == 6 then inSeat = inSeat + 1 end end
  check("driver combat: no request in state 6", inSeat == 0 and r.c.stats.failed == 1)
end
do -- the camera is not held-ready (e.g. first person), vehicle.forceTppOnEnter off: nothing is armed (old behaviour)
  local r = ride({ driver = true, camReady = false, cfg = { forceTppOnEnter = false } })
  check("camera not ready, forceTppOnEnter off: no request, not armed", #r.requests == 0 and r.c.stats.armed == 0)
end
do -- kill switch
  local r = ride({ driver = true, cfg = { enabled = false } })
  check("disabled: no request", #r.requests == 0 and (r.c.stats.ownMounts or 0) == 0)
end

do -- exit match: sources of the game camera pose, A/B setters (not persisted), frame math, pull-out timing, Status
  check("exit match: defaults on (exitMatch, 0.70 s)", V.DEFAULTS.exitMatch == true and V.DEFAULTS.exitMatchTime == 0.70)
  check("exit match: A/B setters", V.SetExitMatch(false) == false and V.SetExitMatch(true) == true
    and V.SetExitMatchTime(0.6) == 0.6 and V.SetExitMatchTime(nil) == V.DEFAULTS.exitMatchTime)
  local path = os.tmpname()
  V.SetExitMatch(false)
  V.SaveSettings(path)
  local f = io.open(path, "r")
  local text = f and f:read("*a") or ""
  if f then f:close() end
  os.remove(path)
  check("exit match: A/B switches not persisted", not string.find(text, "exitMatch", 1, true))
  V.SetExitMatch(true)
  local st = V.Status()
  check("exit match: Status reports the switches", string.find(st, "exitMatch=true", 1, true) ~= nil
    and string.find(st, "exitMatches=", 1, true) ~= nil)
  local fr = { px = 3, py = -2, pz = 0.5, pyaw = 37, vx = 1, vy = 4, vz = 0.2, vyaw = -120 }
  local lx, ly, lz = V.WorldToLocal(2.3, -5.2, 1.9, fr)
  local wx, wy, wz = V.LocalToWorld(lx, ly, lz, fr)
  -- the vehicle-behind point (0, -pullBack, pullHeight) of the vehicle frame in world coordinates
  local r = math.rad(fr.vyaw)
  local bwx, bwy = fr.vx + math.sin(r) * V.DEFAULTS.pullBack, fr.vy - math.cos(r) * V.DEFAULTS.pullBack
  local tx, ty, tz = V.WorldToLocal(bwx, bwy, fr.vz + V.DEFAULTS.pullHeight, fr)
  local p = V.PullPose(fr.px, fr.py, fr.pz, fr.pyaw, fr.vx, fr.vy, fr.vz, fr.vyaw, V.DEFAULTS)
  check("exit match: frame math round trip and the vehicle-behind point = PullPose",
    math.abs(wx - 2.3) < 1e-9 and math.abs(wy + 5.2) < 1e-9 and math.abs(wz - 1.9) < 1e-9
    and math.abs(tx - p.x) < 1e-9 and math.abs(ty - p.y) < 1e-9 and math.abs(tz - p.z) < 1e-9)
  -- exit start in a fresh controller seated in a kept ride; reader(allowActive) -> pose / list
  local function exitStart(reader, cfg)
    local c = newOwn(cfg or {})
    local lines = {}
    c.trace = function(msg) lines[#lines + 1] = msg end
    c.keep, c.keepReady, c.seatedActive, c.lastStepClock = "ride", false, 3, 0
    c.gameCamQuery = reader
    c.Step(0.01, { vehicleState = 4, mounted = true, driver = true, activePersp = 3, persistentPersp = 3 })
    return c, lines
  end
  local function logged(lines, needle)
    for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then return true end end
    return false
  end
  local SEAT = { x = -0.33, y = -0.30, z = 0.16, yaw = 0, pitch = 0, src = "vehicleTPPCamera" } -- 0.5.9 log
  local FAR = { x = 5.01, y = -6.84, z = 2.82, yaw = 36, pitch = -4, src = "active camera" }    -- 0.5.9 exit 2
  local c, lines = exitStart(function() return { x = 0.2, y = 0.9, z = 1.2, yaw = 0, pitch = 0, src = "vehicleTPPCamera" } end)
  check("exit match: a pose inside the vehicle (FPP / interior) is rejected", c.match == nil and c.cam.hold == "exit")
  -- the active camera (rendered vehicle TPP camera) is read before our activation
  local allowSeen = nil
  c, lines = exitStart(function(allow) allowSeen = allow; return { FAR, SEAT } end)
  check("exit match: scripted exit with the active camera pose -> matched from the active camera (logged source)",
    allowSeen == true and c.match ~= nil and c.match.src == "active camera" and c.match.x == FAR.x
    and math.abs(c.MatchWeight() - 1) < 1e-9 and c.cam.hold == "exit"
    and logged(lines, "exit match: cam=(5.01,-6.84,2.82) yaw=36 pitch=-4 (active camera) -> target in 0.70 s (exit start)"))
  c.gameCamQuery = function() error("must not be read") end
  c.Step(0.01 + 0.71, { vehicleState = 4, mounted = true, driver = true, activePersp = 0, persistentPersp = 3 })
  check("exit match: done after exitMatchTime", c.match == nil)
  c, lines = exitStart(function() return { SEAT, { x = 2.3, y = -5.2, z = 1.9, yaw = 24, pitch = -9, src = "active camera" } } end)
  check("exit match: vehicleTPPCamera inside the vehicle, active camera valid -> matched from the active camera",
    c.match ~= nil and c.match.src == "active camera"
    and logged(lines, "(active camera) -> target in 0.70 s (exit start) [skipped: vehicleTPPCamera at (-0.33,-0.30,0.16) is inside the vehicle]"))
  c, lines = exitStart(function() return { { x = -0.2, y = 0.4, z = 1.1, src = "active camera" }, SEAT } end)
  check("exit match: active camera and vehicleTPPCamera inside the vehicle -> vehicle-behind pose (both logged)",
    c.match == nil and c.cam.hold == "exit" and c.stats.exitMatchMisses == 1
    and logged(lines, "exit match: no game camera pose at exit start (active camera at (-0.20,0.40,1.10) is inside the vehicle; "
      .. "vehicleTPPCamera at (-0.33,-0.30,0.16) is inside the vehicle) - vehicle-behind pose as is"))
  c, lines = exitStart(function() return nil, "active camera unread; vehicleTPPCamera component not found" end)
  check("exit match: nothing readable -> vehicle-behind pose",
    c.match == nil and logged(lines, "(active camera unread; vehicleTPPCamera component not found) - vehicle-behind pose as is"))
  -- motorcycle pull-out over bikePullTime, cars over pullTime
  local function pullAfter(bike)
    local b = V.NewController({ blendDtCap = 0 }) -- (rate check with one long step: the stall cap is off here, see the stall-cap checks)
    b.keep, b.keepSince, b.vehBike = "mount", 0, bike
    b.UpdatePull(0.3, { vehDist = 0.5 }, 0.65)
    return b.pull
  end
  check("bike pull-out over bikePullTime (1.3 s), car pull-out over pullTime (0.9 s)",
    V.DEFAULTS.bikePullTime == 1.3 and math.abs(pullAfter(true) - 0.5) < 1e-9
    and math.abs(pullAfter(nil) - 0.65 / 0.9) < 1e-9, string.format("%.3f / %.3f", pullAfter(true), pullAfter(nil)))
  -- 2026-10-28 mount pivot continuity: the near trigger waits while utpp_camera's mount pivot blend runs
  -- (obs.camMountBlend); pullStart still starts it; nil (older camera) / false = no wait
  do
    local b = V.NewController({})
    local lines = {}
    b.trace = function(l) lines[#lines + 1] = l end
    b.keep, b.keepSince, b.vehBike = "mount", 0, true
    b.UpdatePull(0.25, { vehDist = 0.3, camMountBlend = true }, 1 / 60)
    local waited = not b.pulling and b.pull == 0
    b.UpdatePull(0.38, { vehDist = 0.3, camMountBlend = false }, 1 / 60)
    local started = b.pulling == true and b.pull > 0
    local logged = false
    for _, l in ipairs(lines) do if string.find(l, "after the camera's mount pivot blend)", 1, true) then logged = true end end
    local b2 = V.NewController({})
    b2.keep, b2.keepSince = "mount", 0
    b2.UpdatePull(0.95, { vehDist = 0.3, camMountBlend = true }, 1 / 60)
    local b3 = V.NewController({})
    b3.keep, b3.keepSince = "mount", 0
    b3.UpdatePull(0.25, { vehDist = 0.3 }, 1 / 60)
    check("mount pivot blend: near pull-out trigger waits for the camera blend, starts at its end (logged), pullStart still "
      .. "applies, no wait without the flag", waited and started and logged and b2.pulling == true and b3.pulling == true,
      string.format("%s %s %s %s %s", tostring(waited), tostring(started), tostring(logged), tostring(b2.pulling), tostring(b3.pulling)))
    b.UpdatePull(0.40, { vehDist = 0.3, camMountBlend = true }, 1 / 60)
    check("mount pivot blend: a running pull-out is not stopped by the flag", b.pulling == true)
  end
end

-- ================================================================ vehicle batch 2026-10-22 (recording 14-57-06)
-- seq(): drives Step through segments { n = frames, obs = {...}, acc = true (the vehicle reports the last accepted
-- request as its active perspective) }; collects the requests, the camera commands (drained each frame, outcomes ok)
-- and the log lines.
local function seq(c, segs)
  local out = { requests = {}, cmds = {}, lines = {}, c = c, keeps = {}, exitKeep = {} }
  c.trace = function(m) out.lines[#out.lines + 1] = m end
  local t = c.seqClock or 0 -- continues the clock of an earlier seq() on the same controller
  for _, s in ipairs(segs) do
    for _ = 1, s.n do
      t = t + 1 / 60
      local obs = {}
      for k, v in pairs(s.obs) do obs[k] = v end
      if not obs.mounted then out.accepted = nil end -- a new vehicle entry starts from its own camera
      if s.acc and out.accepted then obs.activePersp = out.accepted end
      local p = c.Step(t, obs)
      if p and c.lastReqKind == "exitKeep" then -- MOTO-1 A exit keep requests are listed apart
        out.exitKeep[#out.exitKeep + 1] = { t = t, p = p, state = obs.vehicleState, mounted = obs.mounted }
      elseif p then
        out.requests[#out.requests + 1] = { t = t, p = p, state = obs.vehicleState }
        if s.acc then out.accepted = p end
      end
      for _, cmd in ipairs(c.TakeCamCmds()) do
        out.cmds[#out.cmds + 1] = { op = cmd.op, blend = cmd.blend, why = cmd.why, t = t, keep = c.keep }
      end
      if out.keeps[#out.keeps] ~= c.keep then out.keeps[#out.keeps + 1] = c.keep end
    end
  end
  c.seqClock = t
  return out
end
local function nLines(r, needle)
  local n = 0
  for _, l in ipairs(r.lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end
local function ops(r, op)
  local l = {}
  for _, cmd in ipairs(r.cmds) do if op == nil or cmd.op == op then l[#l + 1] = cmd end end
  return l
end
local function keepPath(r) return table.concat(r.keeps, ">") end
local FOOT = { vehicleState = 0, mounted = false }
local function obsOf(vs, extra)
  local o = { vehicleState = vs, mounted = true, driver = true, activePersp = 0, persistentPersp = 0 }
  for k, v in pairs(extra or {}) do o[k] = v end
  return o
end
local FARCAM = { x = 0.4, y = -6.8, z = 2.6, yaw = 2, pitch = -12, src = "active camera" }

do -- CAM-1 T1 / CAR-8 (a): the mounting slot reads as seat_front_right for the whole mount (Quadra, log 1115-1145; the
  -- passenger-side entry of 2026-10-23): the vanilla seat switch to the free driver seat is expected -> hold at the start
  local c = newOwn({})
  local r = seq(c, { { n = 30, obs = FOOT },
                     { n = 240, obs = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = true }) },
                     { n = 150, acc = true, obs = obsOf(1) } })
  check("seat switch expected: hold at the mount start (no late hold), no passenger rule, no 'seat not final'",
    c.stats.ownMounts == 1 and (c.stats.lateHolds or 0) == 0 and (c.stats.seatSwitchHolds or 0) == 1
    and nLines(r, "passenger-side entry (slot seat_front_right, driver seat free=true)") == 1
    and nLines(r, "seat not final yet") == 0 and nLines(r, "passenger seat") == 0, table.concat(r.lines, "\n"))
  check("seat switch expected: one TPPClose request in Driving, done, handed over, seat final logged once", #r.requests == 1
    and r.requests[1].p == 1 and r.requests[1].state == 1 and c.stats.done == 1 and c.stats.ownHandovers == 1
    and c.keep == "ride" and keepPath(r) == "off>mount>ride" and nLines(r, "seat final: driver state 1") == 1
    and (c.stats.seatSwitchDrivers or 0) == 1, keepPath(r))
end
do -- CAR-8 (d): a confirmed passenger keeps the passenger rule (state 3); a non-driver slot with the driver seat taken,
  -- a back seat or switches disabled keep the decision open (no hold)
  local function pending(extra)
    local c = newOwn({}); c.vehCar = true; c.cfg.passengerChase = false
    local o4 = { driver = false }
    for k, v in pairs(extra) do o4[k] = v end
    local r = seq(c, { { n = 30, obs = FOOT }, { n = 90, obs = obsOf(4, o4) }, { n = 120, obs = obsOf(3, { driver = false }) } })
    return c, r
  end
  for _, e in ipairs({ { seat = "seat_front_right", driverSeatFree = false }, { seat = "seat_back_left" },
                       { seat = "seat_front_right", driverSeatFree = true, switchSeatsDisabled = true } }) do
    local c, r = pending(e)
    check(string.format("confirmed passenger (state 3, %s, free=%s, disabled=%s): decision kept, passenger rule once, "
      .. "no hold, TPP request in state 3 only (2026-11-01)", tostring(e.seat), tostring(e.driverSeatFree),
      tostring(e.switchSeatsDisabled)),
      nLines(r, "seat not final yet") == 1 and nLines(r, "passenger seat") == 1 and (c.stats.ownMounts or 0) == 0
      and #r.requests >= 1 and r.requests[1].state == 3 and c.keep == "off" and #r.cmds == 0, keepPath(r))
  end
  -- the switch was expected but the game kept V in the passenger seat (state 3): the hold ends there, camera's own rules
  local c3 = newOwn({}); c3.vehCar = true; c3.cfg.passengerChase = false
  local r3 = seq(c3, { { n = 30, obs = FOOT },
                       { n = 90, obs = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = true }) },
                       { n = 120, obs = obsOf(3, { driver = false, seat = "seat_front_right" }) } })
  check("seat switch expected, passenger state 3 at the end: hold ended with 'free' (keep off), TPP request in state 3 only, "
    .. "seat final logged", keepPath(r3) == "off>mount>off" and #ops(r3, "free") == 1 and #ops(r3, "togame") == 0
    and #r3.requests >= 1 and r3.requests[1].state == 3 and nLines(r3, "seat final: passenger state 3") == 1
    and (c3.stats.seatSwitchPassengers or 0) == 1, keepPath(r3))
  -- A/B SetSeatSwitchHold(false): the old behaviour (decision kept until the seat is final, late hold in state 1)
  local c4 = newOwn({ seatSwitchHold = false })
  local r4 = seq(c4, { { n = 30, obs = FOOT },
                       { n = 90, obs = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = true }) },
                       { n = 120, acc = true, obs = obsOf(1) } })
  check("seatSwitchHold off: decision kept during the mount, late hold once seated", nLines(r4, "seat not final yet") == 1
    and (c4.stats.lateHolds or 0) == 1 and c4.stats.ownMounts == 1, keepPath(r4))
end
do -- passenger-side entry, log 2026-10-23 21:17:06.9-10.4: enter + switch_seat in state 4 (3.48 s, slot seat_front_right),
  -- one state-1 frame still with the old slot, then seat_front_left; vehicle persistent TPPFar
  local function passengerSide(cfgx, ready)
    local c = newOwn(cfgx or {}, ready)
    c.ownApiMotion = true
    local r = seq(c, { { n = 30, obs = FOOT },
      { n = 209, obs = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = true, persistentPersp = 3 }) },
      { n = 1, acc = true, obs = obsOf(1, { driver = false, seat = "seat_front_right", persistentPersp = 3 }) },
      { n = 150, acc = true, obs = obsOf(1, { seat = "seat_front_left", persistentPersp = 3 }) } })
    return c, r
  end
  local c, r = passengerSide()
  check("passenger-side entry: hold from the mount start through the seat switch, never 'off' / 'game' before the "
    .. "hand-over", keepPath(r) == "off>mount>ride" and #ops(r, "free") == 0 and #ops(r, "togame") == 0
    and #ops(r, "vhold") == 1 and ops(r, "vhold")[1].t < 0.52 + 1e-6, keepPath(r))
  check("passenger-side entry: pull-out during the entry, one TPPFar request in Driving, hand-over after it",
    (c.stats.pulls or 0) == 1 and #r.requests == 1 and r.requests[1].p == 3 and r.requests[1].state == 1
    and c.stats.done == 1 and c.stats.ownHandovers == 1 and nLines(r, "pull-out start (") == 1, tostring(#r.requests))
  check("passenger-side entry: the stale slot in the first state-1 frame is not a passenger (no passenger rule, no "
    .. "'not driver seat', slot lag logged once)", nLines(r, "passenger seat") == 0 and nLines(r, "not driver seat") == 0
    and nLines(r, "slot not updated yet") <= 1 and nLines(r, "always third person on entry: no request") == 0, keepPath(r))
  -- the user's camera key in the ride: honoured, not requested again (the choice is this ride only)
  local r2 = seq(c, { { n = 90, obs = obsOf(1, { seat = "seat_front_left", activePersp = 0, persistentPersp = 0 }) },
                      { n = 30, obs = obsOf(4, { seat = "seat_front_left" }) }, { n = 40, obs = FOOT } })
  check("passenger-side entry: in-ride FPP honoured (no request), exit from the game camera, next ride base TPPFar",
    #r2.requests == 0 and c.rideBase.car == 3 and keepPath(r2) == "ride>game>exit>off", keepPath(r2))
  -- first person on foot (camera not held-ready): the passive ride is still forced to third person in Driving
  local cp, rp = passengerSide({}, false)
  check("passenger-side entry from on-foot FPP: no hold, one forced TPPFar request in the first Driving frame",
    (cp.stats.ownMounts or 0) == 0 and (cp.stats.forcedRides or 0) == 1 and #rp.requests == 1 and rp.requests[1].p == 3
    and rp.requests[1].state == 1 and nLines(rp, "no request for this ride") == 0, tostring(#rp.requests))
end
do -- CAM-1 T2 / (3): a seat state with a TPP vehicle camera during the mount hold hands the view to the game camera
  local function mountThen(vs, extra)
    local c = newOwn({}); if vs == 3 then c.vehCar = true; c.cfg.passengerChase = false end
    local r = seq(c, { { n = 30, obs = FOOT }, { n = 40, obs = obsOf(4) }, { n = 30, obs = obsOf(vs, extra) } })
    return c, r
  end
  local c, r = mountThen(7, { activePersp = 5 })
  local tg = ops(r, "togame")
  check("mount hold + scene state 7 with vehicle TPP: 'togame' 0.30 s, keep game, no 'free'", #tg == 1
    and math.abs((tg[1].blend or 0) - 0.30) < 1e-9 and #ops(r, "free") == 0 and c.keep == "game" and c.ownHeld == false,
    keepPath(r))
  c, r = mountThen(7, { activePersp = 0 })
  check("mount hold + state 7 with vehicle FPP: 'free' (camera's own rules)", #ops(r, "free") == 1 and #ops(r, "togame") == 0
    and c.keep == "off")
  c, r = mountThen(3, { activePersp = 2 })
  -- 2026-11-01: a passenger whose vehicle camera is TPP (the request of c.StepPassenger accepted / already TPP) runs as keep
  -- "game" after the hold ended with 'free': the exit is taken from the game camera, utpp_body gets the camera callback
  check("mount hold + passenger state 3 with a TPP vehicle camera: 'free', then keep game (passenger ride)",
    #ops(r, "free") == 1 and #ops(r, "togame") == 0 and c.keep == "game" and c.passengerForced == true, keepPath(r))
  c, r = mountThen(3, { activePersp = 0 })
  check("mount hold + passenger state 3 with the vehicle camera in FPP: 'free', keep off, TPP request",
    #ops(r, "free") == 1 and c.keep == "off" and #r.requests >= 1 and r.requests[1].state == 3, keepPath(r))
end
do -- MOTO-5 (a) / CAR-8 (b): driver combat (state 6) during the mount keeps the hold; hand-over in state 6
  local c = newOwn({})
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 20, acc = true, obs = obsOf(1) },
                     { n = 120, obs = obsOf(6, { activePersp = 5 }) } })
  check("mount 4 -> 1 -> 6 before the hand-over: keep stays mount, hand-over in state 6, no free / togame",
    keepPath(r) == "off>mount>ride" and c.stats.ownHandovers == 1 and #ops(r, "free") == 0 and #ops(r, "togame") == 0
    and #r.requests == 1, keepPath(r))
  local c2 = newOwn({})
  local r2 = seq(c2, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 120, obs = obsOf(6, { activePersp = 5 }) } })
  check("mount 4 -> 6 (vehicle DriverCombatMedium): done without a request, hand-over, keep ride", c2.stats.done == 1
    and c2.stats.failed == 0 and #r2.requests == 0 and c2.stats.ownHandovers == 1 and c2.keep == "ride", keepPath(r2))
end
do -- MOTO-5 (b): ride 1 -> 6 -> 1 keeps the ride
  local c = newOwn({})
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 90, acc = true, obs = obsOf(1) },
                     { n = 60, obs = obsOf(6, { activePersp = 5 }) }, { n = 60, obs = obsOf(1, { activePersp = 2 }) } })
  check("ride 1 -> 6 -> 1: keep stays ride, no togame", keepPath(r) == "off>mount>ride" and #ops(r, "togame") == 0
    and #ops(r, "free") == 0, keepPath(r))
end
do -- MOTO-5 (c): exit 4 -> 6 -> 4 = aborted exit, then a new exit
  local c = newOwn({})
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 90, acc = true, obs = obsOf(1) },
                     { n = 20, obs = obsOf(4) }, { n = 30, obs = obsOf(6, { activePersp = 5 }) }, { n = 20, obs = obsOf(4) } })
  check("exit 4 -> 6 -> 4: aborted (drop to the vehicle camera), then a new exit", c.stats.ownExits == 2
    and #ops(r, "drop") == 1 and c.keep == "exit", keepPath(r))
end
do -- CAM-2 T1: a seat state in the ride hands the view to the game camera (keep game), the exit is taken from it
  local function rideGameExit(apiMotion, extraCfg, exitQuery)
    local c = newOwn(extraCfg or {})
    c.ownApiMotion = apiMotion
    c.gameCamQuery = function() return { FARCAM } end
    if exitQuery then c.ownExitQuery = exitQuery end
    local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 90, acc = true, obs = obsOf(1) },
                       { n = 60, obs = obsOf(7, { activePersp = 5 }) }, { n = 30, obs = obsOf(4) } })
    return c, r
  end
  local c, r = rideGameExit(true)
  local gt = ops(r, "gametake")
  check("ride -> state 7 (vehicle TPP) -> exit: keep ride > game > exit, one togame, one gametake at the matched pose",
    keepPath(r) == "off>mount>ride>game>exit" and #ops(r, "togame") == 1 and #gt == 1 and c.exitPull == "hold"
    and c.match ~= nil and c.match.src == "active camera" and math.abs((gt[1].blend or 0) - 0.25) < 1e-9
    and nLines(r, "takes the view from the game camera at the matched game camera pose (vehicle camera TPP") == 1,
    keepPath(r))
  c, r = rideGameExit(false)
  check("game exit without TakeForVehicleExit (vehicle API < 2): no take-over, keep game, logged once (as before)",
    #ops(r, "gametake") == 0 and c.keep == "game" and nLines(r, "vehicle API < 2") == 1, keepPath(r))
  c, r = rideGameExit(true, nil, function() return false, "user prefers first person" end)
  check("game exit with on-foot first person wanted: no take-over, keep off", #ops(r, "gametake") == 0 and c.keep == "off",
    keepPath(r))
end
do -- CAM-2 T2: passive driver ride (hold refused at the mount) -> keep game -> the exit issues a gametake
  local ready = false
  local c = V.NewController({ forceTppOnEnter = false })
  c.own, c.ownApiMotion = true, true
  c.ownCamReady = function() return ready end
  c.gameCamQuery = function() error("must not be read from a vehicle FPP camera") end
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) } })
  ready = true
  local r2 = seq(c, { { n = 90, obs = obsOf(1) }, { n = 30, obs = obsOf(4) } })
  local gt = ops(r2, "gametake")
  check("passive driver ride from vehicle FPP: keep game while seated, exit = one gametake with a cut (blend 0), "
    .. "vehicle-behind pose, no camera read", nLines(r, "mount while the camera is not held-ready") == 1
    and keepPath(r2) == "game>exit" and #gt == 1 and gt[1].blend == 0 and c.match == nil and c.exitPull == "hold"
    and #r2.requests == 0, keepPath(r2))
end
do -- CAM-7 T1 / MOTO-5 (3): a passive driver ride requests the TPP perspective like an armed one
  local r = ride({ driver = true, camReady = false, acceptAfter = 1 })
  check("always TPP: passive driver ride with persistent FPP -> one request in Driving (TPPClose), done, none in the exit",
    #r.requests == 1 and r.requests[1].state == 1 and r.requests[1].p == 1 and r.c.stats.done == 1
    and (r.c.stats.forcedRides or 0) == 1 and r.duringExit == 0 and (r.c.stats.ownMounts or 0) == 0, tostring(#r.requests))
  local n = ride({ driver = true, camReady = false })
  check("always TPP: never accepted -> bounded, none during the exit", #n.requests > 0
    and #n.requests <= V.DEFAULTS.maxRequests and n.duringExit == 0 and n.c.stats.failed == 1, tostring(#n.requests))
  local p = ride({ driver = false, seatState = 3, camReady = false, acceptAfter = 1, car = true })
  check("always TPP: passenger (state 3) -> one request (after the vanilla FPP), not armed", #p.requests == 1
    and p.requests[1].state == 3 and p.c.stats.armed == 0)
  local cs = newOwn({}, false)
  local rs = seq(cs, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 150, obs = obsOf(1, { highLevel = 4 }) } })
  check("always TPP: scene (PSM HighLevel 4) -> no request", #rs.requests == 0 and cs.stats.armed == 0)
  local off = ride({ driver = true, camReady = false, acceptAfter = 1, cfg = { forceTppOnEnter = false } })
  check("always TPP off: the passive ride sends no request (old behaviour)", #off.requests == 0)
end
do -- CAM-7 T2 / CAR-8 (c): the next entry requests the value UTPP requested last, never the player's in-ride choice
  local c = newOwn({})
  local r = seq(c, {
    { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { persistentPersp = 2 }) },
    { n = 60, acc = true, obs = obsOf(1, { persistentPersp = 2 }) },
    { n = 90, obs = obsOf(1, { activePersp = 0, persistentPersp = 0 }) }, -- the player switches to FPP in the ride
    { n = 30, obs = obsOf(4) }, { n = 30, obs = FOOT },
    { n = 60, obs = obsOf(4) }, { n = 60, acc = true, obs = obsOf(1) } })
  check("ride 1 TPPMedium, in-ride FPP honoured (no request), ride 2 with persistent FPP requests TPPMedium again",
    #r.requests == 2 and r.requests[1].p == 2 and r.requests[2].p == 2 and c.rideBase.car == 2, string.format("%d: %s %s",
    #r.requests, tostring(r.requests[1] and r.requests[1].p), tostring(r.requests[2] and r.requests[2].p)))
  local c2 = newOwn({})
  local r2 = seq(c2, {
    { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { persistentPersp = 2 }) },
    { n = 60, acc = true, obs = obsOf(1, { persistentPersp = 2 }) },
    { n = 90, obs = obsOf(1, { activePersp = 3, persistentPersp = 3 }) }, -- the player picks TPPFar in the ride
    { n = 30, obs = obsOf(4) }, { n = 30, obs = FOOT },
    { n = 60, obs = obsOf(4, { persistentPersp = 3 }) }, { n = 60, acc = true, obs = obsOf(1, { persistentPersp = 3 }) } })
  check("in-ride TPP distance change is not carried over: ride 2 requests TPPMedium (not the persistent TPPFar)",
    #r2.requests == 2 and r2.requests[2].p == 2, tostring(r2.requests[2] and r2.requests[2].p))
  local m = ride({ driver = true, acceptAfter = 1, persistent = 5 })
  check("persistent DriverCombatMedium (5) maps to TPPMedium (2)", m.requests[1] and m.requests[1].p == 2)
  local cb = newOwn({})
  seq(cb, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { persistentPersp = 3 }) },
            { n = 60, acc = true, obs = obsOf(1, { persistentPersp = 3 }) }, { n = 30, obs = obsOf(4) }, { n = 30, obs = FOOT } })
  cb.vehBike = true
  local rb = seq(cb, { { n = 60, obs = obsOf(4) }, { n = 60, acc = true, obs = obsOf(1) } })
  check("rideBase per vehicle class: the car's TPPFar is not used for a bike (TPPClose)", rb.requests[1]
    and rb.requests[1].p == 1 and cb.rideBase.car == 3 and cb.rideBase.bike == 1)
end
do -- setting registered through utpp_core is checked in test_utpp_user_defaults.lua; module defaults / setter here
  check("forceTppOnEnter: default on, togame 0.30 s", V.DEFAULTS.forceTppOnEnter == true and V.DEFAULTS.togameBlend == 0.30)
  check("forceTppOnEnter: Status reports it", string.find(V.Status(), "forceTppOnEnter=true forcedRides=", 1, true) ~= nil)
end

-- ================================================================ vehicle batch P2
local function near(a, b, eps) return type(a) == "number" and type(b) == "number" and math.abs(a - b) <= (eps or 1e-6) end
-- a controller in a kept ride (handed over), the pull frame of a car at the origin facing +Y
local function inRide(cfg)
  local c = newOwn(cfg or {})
  local lines = {}
  c.trace = function(m) lines[#lines + 1] = m end
  c.keep, c.keepReady, c.seatedActive, c.lastStepClock, c.lastActive = "ride", false, 2, 0, 2
  c.pullFrame = { px = -0.35, py = 0.25, pz = -0.3, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  return c, lines
end
local function has(lines, needle)
  for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then return true end end
  return false
end

do -- CAM-5 T2: weight of the hand-over blend and the matched pose
  local c = inRide()
  check("CAM-5: no hand-over blend -> nil", c.HandBlend(1.0) == nil)
  c.handAt, c.handBlendDur, c.handOwnPose = 1.0, 0.45, { x = 0, y = -5.5, z = 2.4, yaw = 0, pitch = -14 }
  local gp = { x = 4, y = -6.5, z = 2.6, yaw = 30, pitch = -10 }
  local w0, wm, we = c.HandBlend(1.0), c.HandBlend(1.225), c.HandBlend(1.4499)
  local m0, mm, me = V.VPoseLerp(c.handOwnPose, gp, w0), V.VPoseLerp(c.handOwnPose, gp, wm), V.VPoseLerp(c.handOwnPose, gp, we)
  check("CAM-5: weight 0 / 0.5 / ~1 at 0 / 0.225 / 0.45 s -> pull-out pose / midpoint / game pose",
    near(w0, 0) and near(wm, 0.5) and near(we, 1, 1e-4) and c.HandBlend(1.46) == nil
    and near(m0.x, 0) and near(m0.y, -5.5) and near(mm.x, 2) and near(mm.y, -6) and near(mm.yaw, 15) and near(mm.pitch, -12)
    and near(me.x, 4, 1e-3) and near(me.yaw, 30, 1e-2), string.format("%s %s %s", tostring(w0), tostring(wm), tostring(we)))
  -- control: the hand-over blend is long over at the exit
  c.handAt = -10
  c.gameCamQuery = function() return { { x = 4, y = -6.5, z = 2.6, yaw = 30, pitch = -10, src = "active camera" } } end
  local r = seq(c, { { n = 1, obs = obsOf(4, { activePersp = 2 }) } })
  -- seq starts its clock at 0: set the hand-over relative to the first frame
  local c2, l2 = inRide()
  c2.handAt, c2.handBlendDur, c2.handOwnPose = 0.0167 - 0.18, 0.45, { x = 0, y = -5.5, z = 2.4, yaw = 0, pitch = -14 }
  c2.gameCamQuery = c.gameCamQuery
  local r2 = seq(c2, { { n = 1, obs = obsOf(4, { activePersp = 2 }) } })
  local tk = ops(r2, "take")
  local w = c2.match and string.match(c2.match.src, "w=([%d%.]+)")
  check("CAM-5 T1: exit 0.18 s after the hand-over -> 'exit during the hand-over blend', take-over blend 0, match between "
    .. "the pull-out pose and the game camera", nLines(r2, "exit during the hand-over blend (w=") == 1 and #tk == 1
    and tk[1].blend == 0 and c2.match ~= nil and c2.match.x > 0 and c2.match.x < 4 and tonumber(w) ~= nil
    and tonumber(w) > 0.2 and tonumber(w) < 0.5, c2.match and c2.match.src)
  check("CAM-5: exit after the blend -> the normal take-over blend", #ops(r, "take") == 1 and ops(r, "take")[1].blend
    == V.DEFAULTS.earlyExitBlend and nLines(r, "exit during the hand-over blend") == 0)
end
do -- CAM-5: a canonical yaw 0 / pitch 0 reading with a different alternative orientation -> the alternative
  local c = inRide()
  c.gameCamQuery = function() return { { x = 0, y = -5.25, z = 1.65, yaw = 0, pitch = 0, altYaw = 3, altPitch = -14,
    altSrc = "transform orientation", src = "active camera" } } end
  local p = c.QueryGameCam(true)
  check("CAM-5: canonical 0/0 pose -> alternative orientation (source logged)", p and near(p.yaw, 3) and near(p.pitch, -14)
    and string.find(p.src, "transform orientation", 1, true) ~= nil, p and p.src)
  c.gameCamQuery = function() return { { x = 0, y = -5.25, z = 1.65, yaw = 0, pitch = 0, altYaw = 1, altPitch = -1,
    src = "active camera" } } end
  local q = c.QueryGameCam(true)
  check("CAM-5: alternative within 2 deg -> the reading is kept", q and q.yaw == 0 and q.pitch == 0 and q.src == "active camera")
end
do -- c9 M3 (log 19-54-19 08.034-08.75): pull-out orientation from the orbit yaw / pitch frozen at the pull start
  -- ride 1 geometry: bike yaw -8.3 at the origin, orbit at -69 deg / 2.95 m / 0.88 m in the vehicle frame, orbit yaw -8.4;
  -- ride 2: -124 deg / 2.47 m / 1.41 m, orbit yaw -62.2 pitch -11. The live orbit follows V's body (+67 deg slide spin in
  -- 0.3 s, unwound over 0.6 s); V's entity yaw spins with it (pullFrame.pyaw).
  local function wrap(d) return (d + 180) % 360 - 180 end
  local function spin(t)
    if t < 0.3 then return 67 * t / 0.3 end
    if t < 0.9 then return 67 * (1 - (t - 0.3) / 0.6) end
    return 0
  end
  local function series(aDeg, r, h, oyaw0, opitch, live, keep, pullPath)
    local c = newOwn({ pullPath = pullPath })
    c.keep, c.pulling, c.vehBike = keep or "mount", true, true
    local vyaw = -8.3
    local ar = math.rad(aDeg + vyaw)
    local ox, oy = r * math.cos(ar), r * math.sin(ar)
    local out, dt = {}, 1 / 60
    local pt = V.DEFAULTS.bikePullTime
    for i = 1, math.floor(pt / dt) + 2 do
      local t = (i - 1) * dt -- the first pose is frozen at t = 0 (one pull step in)
      c.pull = math.min(1, i * dt / pt)
      c.clock = t
      local s = live and spin(t) or 0
      local f = { px = 0.1, py = -0.1, pz = 0.6, pyaw = vyaw + s, vx = 0, vy = 0, vz = 0, vyaw = vyaw }
      -- the live orbit: around V, its yaw chasing the body (old follow under the override)
      local orbit = { x = ox, y = oy, z = h, yaw = oyaw0 + s, pitch = opitch }
      out[#out + 1] = V.OwnPose(c, f, orbit)
      out[#out].f = f
    end
    return out, c
  end
  for _, g in ipairs({ { -69, 2.95, 0.88, -8.4, 0, "ride 1" }, { -124, 2.47, 1.41, -62.2, -11, "ride 2" } }) do
    for _, path in ipairs({ true, false }) do
      local tag = g[6] .. (path and "" or " (linear path)")
      local sp, c = series(g[1], g[2], g[3], g[4], g[5], true, nil, path)
      local st = series(g[1], g[2], g[3], g[4], g[5], false, nil, path)
      local p0 = sp[1]
      local ar = math.rad(g[1] - 8.3)
      local maxStep, maxDiff, first = 0, 0, math.abs(wrap(sp[2].yaw - sp[1].yaw))
      for i = 2, #sp do
        if (i - 1) / 60 <= 0.4 then maxStep = math.max(maxStep, math.abs(wrap(sp[i].yaw - sp[i - 1].yaw))) end
        maxDiff = math.max(maxDiff, math.abs(wrap(sp[i].yaw - st[i].yaw)), math.abs(sp[i].pitch - st[i].pitch),
          math.sqrt((sp[i].pos.x - st[i].pos.x) ^ 2 + (sp[i].pos.y - st[i].pos.y) ^ 2 + (sp[i].pos.z - st[i].pos.z) ^ 2))
      end
      local last = sp[#sp]
      local _, tyaw, tpitch = V.PullPose(last.f.px, last.f.py, last.f.pz, last.f.pyaw, 0, 0, 0, -8.3, V.PullCfg(V.DEFAULTS, true))
      check("c9 M3 " .. tag .. ": frame 0 = the orbit pose of the freeze frame (no jump at the pull start)",
        -- (one pull step in: smoothstep(dt / 1.3 s) = 5e-4 of the way, under 0.05 deg / 2 mm here)
        p0 and near(p0.yaw, g[4], 0.05) and near(p0.pitch, g[5], 0.05) and near(p0.pos.x, g[2] * math.cos(ar), 0.002)
        and near(p0.pos.y, g[2] * math.sin(ar), 0.002) and near(p0.pos.z, g[3], 0.002),
        p0 and string.format("%.4f/%.4f (%.4f,%.4f,%.4f) want %.4f/%.4f (%.4f,%.4f,%.4f)", p0.yaw, p0.pitch, p0.pos.x, p0.pos.y,
          p0.pos.z, g[4], g[5], g[2] * math.cos(ar), g[2] * math.sin(ar), g[3]))
      check("c9 M3 " .. tag .. ": body spin under the pull -> first pull frame yaw step < 0.1 deg, <= 4 deg/frame over 0.4 s, "
        .. "same output as without the spin", first < 0.1 and maxStep <= 4 and maxDiff <= 1e-6,
        string.format("first %.3f, max %.2f, diff %.2e", first, maxStep, maxDiff))
      check("c9 M3 " .. tag .. ": w = 1 ends on the pull-out target yaw / pitch", near(wrap(last.yaw - (tyaw + last.f.pyaw)), 0, 1e-6)
        and near(last.pitch, tpitch, 1e-6), string.format("%.3f vs %.3f", last.yaw, tyaw + last.f.pyaw))
      check("c9 M3 " .. tag .. ": the mount override asks the orbit to keep its world yaw (keepYaw)", p0.keepYaw == true)
      check("c9 M3 " .. tag .. ": pathStart keeps the frozen orientation", c.pathStart and near(c.pathStart.oyawV, wrap(g[4] + 8.3), 1e-6))
    end
  end
  local ch = series(-69, 2.95, 0.88, -8.4, 0, false, "chase", true)
  check("c9 M3: chase retake pose leaves the orbit follow unchanged (no keepYaw)", ch[1] and ch[1].keepYaw == nil)
end
do -- CAM-6 T1/T2: the close bike TPP camera is accepted, cars unchanged
  local BIKECAM = { x = 0.02, y = -1.55, z = 0.98, yaw = 0, pitch = -8, src = "active camera" }
  local function q(bike, tpp, pose)
    local c = inRide()
    c.vehBike, c.vehTppNow = bike, tpp
    c.gameCamQuery = function() return { pose or BIKECAM } end
    return c.QueryGameCam(true)
  end
  check("CAM-6 T1: bike, vehicle TPP, camera 1.55 m behind -> accepted", q(true, true) ~= nil)
  check("CAM-6 T2: car (same pose) -> rejected inside the vehicle (unchanged)", q(nil, true) == nil)
  check("CAM-6: bike without a TPP vehicle camera -> rejected", q(true, false) == nil)
  check("CAM-6: bike camera in front of the seat -> rejected", q(true, true, { x = 0.1, y = 1.5, z = 1.0, src = "active camera" }) == nil)
  check("CAM-6 (2): earlyExitBlend 0.30 s", V.DEFAULTS.earlyExitBlend == 0.30)
end
do -- CAM-6 (3): FOV of the matched camera, blended to the profile FOV with the match weight; unread -> logged
  local c, lines = inRide()
  c.gameCamQuery = function() return { { x = 0.4, y = -6.8, z = 2.6, yaw = 2, pitch = -12, fov = 70, src = "active camera" } } end
  c.Step(0.01, obsOf(4, { activePersp = 2 }))
  local orbit = { x = 0, y = -3, z = 1.8, yaw = 0, pitch = -10, fov = 56 }
  local p0 = V.OwnPose(c, c.pullFrame, orbit)
  c.clock = 0.01 + 0.35
  local pm = V.OwnPose(c, c.pullFrame, orbit)
  c.clock = 0.01 + 0.71
  local pe = V.OwnPose(c, c.pullFrame, orbit)
  check("CAM-6 (3): fov 70 at the match, between 70 and the profile 56 half way, profile (nil) after the match",
    p0 and near(p0.fov, 70) and pm and pm.fov > 56 and pm.fov < 70 and pe and pe.fov == nil
    and has(lines, "fov=70.0"), string.format("%s %s %s", tostring(p0 and p0.fov), tostring(pm and pm.fov), tostring(pe and pe.fov)))
  local c2, l2 = inRide()
  c2.gameCamQuery = function() return { { x = 0.4, y = -6.8, z = 2.6, yaw = 2, pitch = -12, src = "active camera" } } end
  c2.Step(0.01, obsOf(4, { activePersp = 2 }))
  local p2 = V.OwnPose(c2, c2.pullFrame, orbit)
  check("CAM-6 (3): no FOV reader -> 'fov unread', no fov written", has(l2, "(fov unread)") and p2 and p2.fov == nil)
end
do -- CAM-8: launch / bike distance follow triggers after the unmount; follow pitch clamp; CAR-3 (4) caps
  local function exitHold(bike, f)
    local c, lines = inRide()
    c.vehBike = bike
    c.keep, c.exitPull, c.cam.hold, c.unmountAt = "off", "hold", "exit", 1.0
    c.pullFrame = f
    c.UpdateExitFollow(1.05, {})
    return c, lines
  end
  local c = exitHold(true, { px = -1.0, py = -0.4, pz = 0.9 + 0.8, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 })
  check("CAM-8 (a): bike, V 0.8 m above the seat height after the unmount -> follow", c.exitFollow ~= nil
    and string.find(c.exitFollow.why, "above the seat height", 1, true) ~= nil, c.exitFollow and c.exitFollow.why)
  c = exitHold(true, { px = -1.3, py = -0.2, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 })
  check("CAM-8 (b): bike, V 1.3 m from the bike after the unmount -> follow", c.exitFollow ~= nil
    and string.find(c.exitFollow.why, "from the motorcycle", 1, true) ~= nil)
  c = exitHold(nil, { px = -1.3, py = -0.2, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 })
  check("CAM-8: car standing exit (V 1.3 m away, on the ground) -> no follow (unchanged)", c.exitFollow == nil)
  check("CAM-8: exitFollowBlend 0.25 s, bike hold 0.2 s after the unmount", V.DEFAULTS.exitFollowBlend == 0.25
    and V.DEFAULTS.bikeExitHoldAfterUnmount == 0.2 and V.DEFAULTS.exitHoldAfterUnmount == 0.5)
  -- follow pitch clamp and caps (orbit target without OrbitPoseAt = the orbit itself)
  c = exitHold(true, { px = -1.0, py = -0.4, pz = 2.0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 })
  c.clock = 1.05
  c.match = { x = 0, y = -3.6, z = 4.0, yaw = 0, pitch = -40, at = 1.05, dur = 10 } -- a steep hold pose
  local orbit = { x = -1.0, y = -3.6, z = 3.4, yaw = 0, pitch = -12, fov = 56 }
  local p1 = V.OwnPose(c, c.pullFrame, orbit)
  check("CAM-8: follow pitch clamped to [-20, 5] (hold pitch -40)", c.exitFollow and near(c.exitFollow.pitch, -20) and p1
    and near(p1.pitch, -20), tostring(c.exitFollow and c.exitFollow.pitch))
  -- jump: V moves 2 m sideways in one frame; the framing change relative to V is capped to 15 m/s * dt
  c.clock = 1.05 + 1 / 60
  c.pullFrame = { px = 1.0, py = -0.4, pz = 2.0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  local orbit2 = { x = 3.0, y = -3.6, z = 3.4, yaw = 90, pitch = -12 } -- the orbit around V shifts 2 m as well
  local p2 = V.OwnPose(c, c.pullFrame, orbit2)
  local o1 = { x = p1.pos.x - (-1.0), y = p1.pos.y + 0.4, z = p1.pos.z - 2.0 }
  local o2 = { x = p2.pos.x - 1.0, y = p2.pos.y + 0.4, z = p2.pos.z - 2.0 }
  local dRel = math.sqrt((o2.x - o1.x) ^ 2 + (o2.y - o1.y) ^ 2 + (o2.z - o1.z) ^ 2)
  check("CAR-3 (4): follow caps: framing change <= 15 m/s * dt (<= 0.35 m), yaw change <= 180 deg/s * dt",
    dRel <= 15 / 60 + 1e-6 and math.abs(p2.yaw - p1.yaw) <= 3 + 1e-6, string.format("%.3f m, %.2f deg", dRel, p2.yaw - p1.yaw))
end
do -- CAM-10: the bike pull-out target and hand-over blend
  local bc = V.PullCfg(V.DEFAULTS, true)
  local p = V.PullPose(0, 0, 0, 0, 0, 0, 0, 0, bc)
  local pc = V.PullPose(0, 0, 0, 0, 0, 0, 0, 0, V.PullCfg(V.DEFAULTS, false))
  check("CAM-10: bike target 3.6 m behind / 1.7 m up, car 5.5 / 2.4 (unchanged)", near(p.y, -3.6) and near(p.z, 1.7)
    and near(pc.y, -5.5) and near(pc.z, 2.4))
  local c = newOwn({})
  c.vehBike = true
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 120, acc = true, obs = obsOf(1) } })
  local rel = ops(r, "release")
  check("CAM-10: bike hand-over Deactivate(0.65)", #rel == 1 and near(rel[1].blend, 0.65), rel[1] and tostring(rel[1].blend))
  local cc = newOwn({})
  local rc = seq(cc, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 120, acc = true, obs = obsOf(1) } })
  check("CAM-10: car hand-over Deactivate(0.45) (unchanged)", #ops(rc, "release") == 1 and near(ops(rc, "release")[1].blend, 0.45))
end
do -- CAM-11 T1: keep "game", vehicle camera 0 -> 4 -> the callback once with (0, 4) in the same Step
  local calls = {}
  V.onVehiclePerspective = function(a, b) calls[#calls + 1] = { a, b } end
  local c = newOwn({}, false)
  c.ownApiMotion = true
  local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 30, obs = obsOf(1, { activePersp = 0 }) } })
  local before = #calls
  c.Step((c.seqClock or 0) + 1 / 60, obsOf(1, { activePersp = 4 }))
  local inStep = #calls - before
  seq(c, { { n = 30, obs = obsOf(1, { activePersp = 4 }) } })
  check("CAM-11 T1: keep game, active 0 -> 4: onVehiclePerspective(0, 4) once, in the same Step", c.keep == "game"
    and inStep == 1 and #calls == 1 and calls[1][1] == 0 and calls[1][2] == 4, string.format("%d %d", inStep, #calls))
  V.onVehiclePerspective = function() error("boom") end
  local okE = pcall(c.Step, (c.seqClock or 0) + 1 / 60, obsOf(1, { activePersp = 1 }))
  check("CAM-11: a failing callback is caught", okE)
  V.onVehiclePerspective = nil
  local c2 = newOwn({})
  local hits = 0
  V.onVehiclePerspective = function() hits = hits + 1 end
  seq(c2, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 60, acc = true, obs = obsOf(1) },
            { n = 30, obs = obsOf(1, { activePersp = 3 }) } })
  check("CAM-11: no callback outside keep game (ride on utppCamera)", hits == 0 and c2.keep == "ride")
  V.onVehiclePerspective = nil
  check("CAM-11: the field defaults to nil (no registration function)", V.onVehiclePerspective == nil
    and V.RegisterVehiclePerspective == nil)
end
do -- W9 safe teleport exit: pushed once a game session answers, retried while the system is missing
  local savedGame = Game
  local calls = {}
  local answer = false
  Game = { UTPPSetSafeTeleportExit = function(on) calls[#calls + 1] = on; return answer end }
  V.SetSafeTeleportExit(false)
  local logs = {}
  local lg = function(m) logs[#logs + 1] = m end
  local pl = {}
  local s1 = V.PushSafeTeleportExit(10.0, pl, lg)
  local s2 = V.PushSafeTeleportExit(10.5, pl, lg) -- before the retry time
  answer = true
  local s3 = V.PushSafeTeleportExit(11.1, pl, lg)
  local s4 = V.PushSafeTeleportExit(12.0, pl, lg)
  check("safeTeleportExit: false before a session -> retried after 1 s, then sent once (value false)", s1 == "pending"
    and s2 == "pending" and s3 == "sent" and s4 == "sent" and #calls == 2 and calls[1] == false and calls[2] == false,
    string.format("%s %s %s %s %d", s1, s2, s3, s4, #calls))
  local s5 = V.PushSafeTeleportExit(13.0, {}, lg) -- a new player object (new session) -> pushed again
  check("safeTeleportExit: a new player object pushes the value again", s5 == "sent" and #calls == 3)
  -- review: CET hands out a fresh wrapper per Game.GetPlayer call; the same entity (ID hash) must not re-push or log per frame
  local function wrapper(h) return { GetEntityID = function() return { hash = h } end } end
  V.SetSafeTeleportExit(true)
  local n0, l0 = #calls, #logs
  for i = 1, 120 do V.PushSafeTeleportExit(20 + i / 60, wrapper(77), lg) end
  check("safeTeleportExit: a fresh wrapper per frame for the same entity -> pushed and logged once (not per frame)",
    #calls == n0 + 1 and calls[#calls] == true and #logs == l0 + 1, string.format("calls %d logs %d", #calls - n0, #logs - l0))
  answer = false
  for i = 1, 120 do V.PushSafeTeleportExit(30 + i / 60, wrapper(77), lg) end
  check("safeTeleportExit: same entity, value already sent -> nothing more even when the native would refuse",
    #calls == n0 + 1)
  answer = true
  V.PushSafeTeleportExit(40, wrapper(78), lg)
  check("safeTeleportExit: another entity ID -> pushed again, the unchanged value not logged again", #calls == n0 + 2
    and #logs == l0 + 1)
  V.SafeTeleportExitPlayerGone()
  V.PushSafeTeleportExit(41, wrapper(78), lg)
  V.PushSafeTeleportExit(41.1, wrapper(78), lg)
  check("safeTeleportExit: a frame without a player (load) -> the next player gets one push", #calls == n0 + 3)
  V.SetSafeTeleportExit(false)
  V.PushSafeTeleportExit(42, wrapper(78), lg)
  check("safeTeleportExit: a setting change -> pushed and logged once with the new value", #calls == n0 + 4
    and calls[#calls] == false and #logs == l0 + 2)
  Game = {}
  V.SetSafeTeleportExit(true)
  check("safeTeleportExit: native missing -> 'unavailable' logged once", V.PushSafeTeleportExit(14, pl, lg) == "unavailable"
    and V.PushSafeTeleportExit(15, pl, lg) == "unavailable" and nLines({ lines = logs }, "unavailable") == 1)
  Game = savedGame
  check("safeTeleportExit: default on", V.DEFAULTS.safeTeleportExit == true)
end

-- ================================================================ 2026-11-01 car camera (log 07-50-12): controller checks
-- a block that raises (e.g. on the base code without the new API) is one failed check, not an abort
local function runBlock(fn)
  local ok, err = pcall(fn)
  if not ok then check("vehiclecam 2026-11-01: a test block raised an error", false, tostring(err)) end
end
runBlock(function() -- a ride whose mount hold ran into armWindow ("failed (timeout)", log :419) still gets the forced third-person request
  local function lateRide(cfg, tail)
    local c = newOwn(cfg or {}); c.vehCar = true
    local segs = { { n = 30, obs = FOOT }, { n = 60 * 9, obs = obsOf(4) } } -- keepTimeout 6 s, armWindow 8 s: failed (timeout)
    for _, s in ipairs(tail or {}) do segs[#segs + 1] = s end
    return c, seq(c, segs)
  end
  local c, r = lateRide({}, { { n = 150, acc = true, obs = obsOf(1) }, { n = 120, obs = obsOf(1, { activePersp = 0 }) } })
  check("late entry: the hold timed out and the phase failed (timeout) before Driving, the first Driving frame asks for TPP, "
    .. "armed again, done", #r.requests == 1 and r.requests[1].state == 1 and r.requests[1].p == 1 and c.stats.done == 1
    and (c.stats.lateEntryRides or 0) == 1 and nLines(r, "failed (timeout)") == 1
    and nLines(r, "driver ride without the UTPP hold (mount hold timed out) - armed") == 1, string.format("%d requests", #r.requests))
  check("late entry: a key switch to FPP later in the ride sends no further request (entry window only)", #r.requests == 1)
  local cOff, rOff = lateRide({ forceTppOnEnter = false }, { { n = 150, acc = true, obs = obsOf(1) } })
  check("late entry: forceTppOnEnter off -> no request", #rOff.requests == 0 and (cOff.stats.lateEntryRides or 0) == 0)
  local cX, rX = lateRide({}, { { n = 60, obs = obsOf(4) }, { n = 30, obs = FOOT } })
  check("late entry: never reached Driving (exit) -> no request, no repeated log line", #rX.requests == 0
    and nLines(rX, "no request for this ride") == 0 and (cX.stats.lateEntryRides or 0) == 0)
  local cP, rP = lateRide({}, { { n = 60, obs = obsOf(3, { driver = false }) } })
  check("late entry: the first seated frame is a passenger state -> no driver request, one 'no request' line",
    nLines(rP, "no request for this ride (vehicle state 3") == 1 and (cP.stats.lateEntryRides or 0) == 0)
  -- a ride that failed for another reason (no TPP after driving) is not re-armed
  local cN = newOwn({})
  local rN = seq(cN, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 240, obs = obsOf(1) } })
  check("late entry: other failures ('no TPP after driving') are not re-armed", cN.stats.failed == 1 and (cN.stats.lateEntryRides or 0) == 0
    and #rN.requests <= V.DEFAULTS.keepMaxRequests)
end)

runBlock(function() -- passengers: a TPP reading inside the settle window is not final (vanilla OnEnter: TPPClose, then FPP)
  local function passRide(segs)
    local c = newOwn({}); c.vehCar = true; c.cfg.passengerChase = false
    local all = { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { driver = false, seat = "seat_back_left" }) } }
    for _, s in ipairs(segs) do all[#all + 1] = s end
    return c, seq(c, all)
  end
  local P3 = function(active) return obsOf(3, { driver = false, seat = "seat_back_left", activePersp = active }) end
  local c, r = passRide({ { n = 12, obs = P3(1) }, { n = 120, obs = P3(0) } }) -- TPPClose at 0.15-0.2 s, then FPP
  check("passenger: TPP read first, FPP lands later inside the window -> the request is sent then (and passengerForced from the "
    .. "first TPP reading)", #r.requests >= 1 and r.requests[1].state == 3 and c.passengerForced == true
    and nLines(r, "the vehicle camera reads TPPClose") == 1 and (c.stats.passengerRequests or 0) >= 1,
    string.format("%d requests", #r.requests))
  local c2, r2 = passRide({ { n = 90, obs = P3(1) }, { n = 90, obs = P3(0) } }) -- TPP held 1.5 s, the user's FPP later
  check("passenger: a TPP vehicle camera that holds ends the request; the user's FPP switch later is honoured (no request)", #r2.requests == 0 and c2.passDone == true and c2.passengerForced == true)
  local c3, r3 = passRide({ { n = 90, obs = P3(0) } }) -- FPP the whole window, never accepted: bounded, then stops
  check("passenger: FPP read for 1.5 s -> a request every 0.25 s, the window (3 s) still open", #r3.requests >= 3
    and #r3.requests <= 6 and c3.passDone ~= true, tostring(#r3.requests))
end)

runBlock(function() -- the 2026-11-01 car behaviour needs a positively identified car: unknown class and bikes keep the previous values
  local car = V.NewController({}); car.vehCar = true
  local bike = V.NewController({}); bike.vehBike = true
  local unk = V.NewController({})
  local both = V.NewController({}); both.vehCar = true; both.vehBike = true
  check("car class: IsCar needs vehCar and not vehBike", car.IsCar() and not bike.IsCar() and not unk.IsCar() and not both.IsCar())
  local m = { x = 0, y = -9.0, z = 2.5 }
  check("exit match duration: car 3.5 m at 4 m/s = 0.88 s; clamped to 0.7 s near and 1.0 s far; bikes / unknown / scaling off "
    .. "keep exitMatchTime", math.abs(car.MatchDuration(m) - 0.875) < 0.01 and car.MatchDuration({ x = 0, y = -5.6, z = 2.4 }) == 0.7
    and car.MatchDuration({ x = 5, y = -12, z = 4 }) == 1.0 and bike.MatchDuration(m) == 0.7 and unk.MatchDuration(m) == 0.7
    and both.MatchDuration(m) == 0.7 and V.NewController({ exitMatchScale = false }).MatchDuration(m) == 0.7,
    string.format("%.3f", car.MatchDuration(m)))
  check("exit match duration: defaults 4 m/s, 0.7-1.0 s, take blend 0, unsure 0.12", V.DEFAULTS.exitMatchSpeed == 4.0
    and V.DEFAULTS.exitMatchTimeMin == 0.7 and V.DEFAULTS.exitMatchTimeMax == 1.0 and V.DEFAULTS.exitTakeBlendMatched == 0
    and V.DEFAULTS.exitTakeBlendUnsure == 0.12 and V.DEFAULTS.exitMatchTime == 0.70 and V.DEFAULTS.earlyExitBlend == 0.30)
end)

runBlock(function() -- car exit: the take blend by class / source (ride path = op "take", game path = op "gametake")
  local ACTIVE = { x = 0.4, y = -9.0, z = 2.5, yaw = 2, pitch = -9, fov = 69, src = "active camera" }
  local function copyPose(p, over) local q = {}; for k, v in pairs(p) do q[k] = v end; for k, v in pairs(over or {}) do q[k] = v end; return q end
  local function take(setup, pose, game)
    local c = newOwn({})
    local lines = {}
    c.trace = function(msg) lines[#lines + 1] = msg end
    c.keep, c.keepReady, c.seatedActive, c.lastStepClock, c.lastActive = game and "game" or "ride", false, 3, 0, 3
    if game then c.ownApiMotion = true end
    if setup then setup(c) end
    c.gameCamQuery = function() return { pose } end
    c.Step(0.01, { vehicleState = 4, mounted = true, driver = true, activePersp = 0, persistentPersp = 3 })
    local cmd = nil
    for _, k in ipairs(c.TakeCamCmds()) do if k.op == (game and "gametake" or "take") then cmd = k end end
    return c, cmd, lines
  end
  local car = function(c) c.vehCar = true end
  local bike = function(c) c.vehBike = true end
  local _, cmd = take(car, ACTIVE)
  check("take blend: car, matched from the rendered camera -> 0 (cut)", cmd and cmd.blend == 0)
  local c1 = take(car, ACTIVE)
  check("take blend: the car match duration is scaled (3.5 m -> 0.88 s)", c1.match and math.abs(c1.match.dur - 3.5 / 4.0) < 0.05,
    c1.match and string.format("%.3f", c1.match.dur))
  _, cmd = take(nil, ACTIVE)
  check("take blend: unknown class -> earlyExitBlend 0.30 as before", cmd and cmd.blend == 0.30)
  _, cmd = take(bike, ACTIVE)
  check("take blend: bike -> 0.30 as before", cmd and cmd.blend == 0.30)
  _, cmd = take(car, copyPose(ACTIVE, { yaw = 0, pitch = 0 }))
  check("take blend: car, orientation read 0 / 0 (not proven) -> 0.12 s", cmd and math.abs(cmd.blend - 0.12) < 1e-9)
  _, cmd = take(car, copyPose(ACTIVE, { yaw = 0, pitch = 0, altYaw = 3, altPitch = -9, altSrc = "transform orientation" }))
  check("take blend: car, 0 / 0 forward but the transform orientation reads -9 -> the alternative is used, cut", cmd and cmd.blend == 0)
  _, cmd = take(car, copyPose(ACTIVE, { src = "vehicleTPPCamera" }))
  check("take blend: car, match from the vehicleTPPCamera component (not the rendered camera) -> 0.30", cmd and cmd.blend == 0.30)
  _, cmd = take(function(c) c.vehCar = true; c.vehSpeed = 12 end, ACTIVE)
  check("take blend: car jump-out (12 m/s) matched from the rendered camera -> 0", cmd and cmd.blend == 0)
  _, cmd = take(function(c) c.vehCar = true; c.vehSpeed = 12 end, copyPose(ACTIVE, { yaw = 0, pitch = 0 }))
  check("take blend: car jump-out, unproven orientation -> the shorter of 0.12 and jumpExitBlend", cmd and cmd.blend == 0.12)
  local gcar, gcmd = take(function(c) c.vehCar = true; c.exitVehTpp = nil end, ACTIVE, true)
  check("game path: car matched from the rendered camera -> TakeForVehicleExit(0)", gcmd and gcmd.blend == 0, gcmd and gcmd.blend)
  _, gcmd = take(bike, ACTIVE, true)
  check("game path: bike -> gameTakeBlend 0.25 as before", gcmd and gcmd.blend == 0.25, gcmd and gcmd.blend)
  _, gcmd = take(nil, copyPose(ACTIVE, { yaw = 0, pitch = 0 }), true)
  check("game path: unknown class, 0 / 0 pose (the bike exit of log :507) -> 0.25 as before", gcmd and gcmd.blend == 0.25)
  _, gcmd = take(car, copyPose(ACTIVE, { yaw = 0, pitch = 0 }), true)
  check("game path: car, 0 / 0 pose -> 0.12", gcmd and math.abs(gcmd.blend - 0.12) < 1e-9)
  local cb = V.NewController({ exitTakeBlendMatched = 0.3, exitMatchScale = false })
  cb.own, cb.ownCamReady, cb.vehCar = true, function() return true end, true
  cb.keep, cb.keepReady, cb.seatedActive, cb.lastStepClock = "ride", false, 3, 0
  cb.gameCamQuery = function() return { ACTIVE } end
  cb.Step(0.01, { vehicleState = 4, mounted = true, driver = true, activePersp = 0, persistentPersp = 3 })
  local ab = nil
  for _, k in ipairs(cb.TakeCamCmds()) do if k.op == "take" then ab = k end end
  check("take blend: A/B exitTakeBlendMatched 0.3 and exitMatchScale off restore the previous values", ab and ab.blend == 0.3
    and cb.match.dur == 0.7)
end)

runBlock(function() -- entry glide target: learned > class default, sanity rules, car only
  local function glideCtrl(cfg)
    local c = newOwn(cfg or {})
    c.vehCar, c.vehRecKey = true, "Vehicle.v_test_car"
    c.pullFrame = { px = -1, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
    c.ownLast = { x = 0, y = -5.5, z = 2.4, yaw = 0, pitch = -14 }
    return c, c.ShownVehPose()
  end
  local c, shown = glideCtrl()
  local t, src = c.VehTppTarget(3, shown)
  check("glide target: TPPFar car default (0,-9,2.5) pitch 0 (the settled camera reads 0) FOV 69", t and src == "default" and t.y == -9 and t.z == 2.5 and t.fov == 69 and t.pitch == 0)
  local t1, why1 = c.VehTppTarget(1, shown)
  check("glide target: no default for TPPClose / TPPMedium -> nil with the reason", t1 == nil and string.find(tostring(why1), "no learned or default", 1, true) ~= nil,
    tostring(why1))
  c.vehTppLearned["Vehicle.v_test_car|1"] = { x = 0, y = -6.2, z = 2.2, yaw = 0, pitch = -8, fov = 66, src = "learned" }
  local t2, src2 = c.VehTppTarget(1, shown)
  check("glide target: a learned pose (record + perspective) is used", t2 and src2 == "learned" and t2.y == -6.2)
  local gl = glideCtrl()
  gl.vehTppLearned["Vehicle.v_test_car|1"] = { x = 0, y = -6.2, z = 2.2, yaw = 0, pitch = -8, fov = 66, src = "learned" }
  gl.keep, gl.phase, gl.phaseEndAt, gl.pulling, gl.pull = "mount", "done", 0, true, 1
  gl.TryHandGlide(1.0, { activePersp = 1 })
  local blendUnconfirmed = gl.handMatch and gl.handMatch.blend
  local gl2 = glideCtrl()
  gl2.vehTppLearned["Vehicle.v_test_car|1"] = { x = 0, y = -6.2, z = 2.2, yaw = 0, pitch = -8, fov = 66, src = "learned", confirmed = true }
  gl2.keep, gl2.phase, gl2.phaseEndAt, gl2.pulling, gl2.pull = "mount", "done", 0, true, 1
  gl2.TryHandGlide(1.0, { activePersp = 1 })
  check("glide target: a learned pose that no later reading confirmed keeps the 0.80 s residual blend, a confirmed one 0.20 s",
    blendUnconfirmed == 0.80 and gl2.handMatch and gl2.handMatch.blend == 0.20, tostring(blendUnconfirmed))
  c.vehTppLearned["Vehicle.v_test_car|3"] = { x = 0, y = 2.0, z = 1.0, yaw = 0, pitch = 0, src = "learned" }
  local t3, src3 = c.VehTppTarget(3, shown)
  check("glide target: a learned pose inside the vehicle is rejected, the default follows", t3 and src3 == "default")
  c.vehTppLearned["Vehicle.v_test_car|3"] = nil
  c.ownLast = { x = 0, y = -0.5, z = 2.4, yaw = 0, pitch = -14 }
  local t4, why4 = c.VehTppTarget(3, c.ShownVehPose())
  check("glide target: more than 8 m from the shown pose -> rejected (reason logged)", t4 == nil
    and string.find(tostring(why4), "from the shown pose", 1, true) ~= nil, tostring(why4))
  local cb = glideCtrl(); cb.vehBike = true
  local cu = glideCtrl(); cu.vehCar = nil
  local tb, whyb = cb.VehTppTarget(3, shown)
  check("glide target: a bike / an unknown class has none", tb == nil and cu.VehTppTarget(3, shown) == nil and whyb == "not a car")
  -- TryHandGlide: speed gate, once per hold, no pose -> engine blend (fallback) logged
  local g = glideCtrl()
  g.keep, g.phase, g.phaseEndAt, g.pulling, g.pull = "mount", "done", 0, true, 1
  local lines = {}
  g.trace = function(m) lines[#lines + 1] = m end
  g.vehSpeed = 5
  g.TryHandGlide(1.0, { activePersp = 3 })
  local moving = g.handMatch == nil and g.handGlideTried
  local logMoving = false
  for _, l in ipairs(lines) do if string.find(l, "handover glide: no vehicle TPP pose (vehicle moves 5.0 m/s) - engine blend 0.90 s", 1, true) then logMoving = true end end
  check("glide: a vehicle moving > 2 m/s skips the glide (logged, decided once)", moving and logMoving, table.concat(lines, "|"))
  local g2 = glideCtrl()
  g2.keep, g2.phase, g2.phaseEndAt, g2.pulling, g2.pull = "mount", "done", 0, true, 1
  g2.TryHandGlide(1.0, { activePersp = 3 })
  local hm = g2.handMatch
  check("glide: standing car -> handMatch glide 1.05 s (3.5 m at <= 5 m/s peak, 0.9 s minimum) to the default (FOV 69), residual blend 0.80 for a default",
    hm and hm.glide and near(hm.dur, 1.05, 0.01) and hm.fov == 69 and hm.blend == 0.80 and hm.src == "default" and g2.glideTarget.persp == 3)
  g2.handMatch, g2.handMatchTried = nil, false
  g2.TryHandGlide(1.1, { activePersp = 3 })
  check("glide: decided once per mount hold", g2.handMatch == nil)
  local g3 = glideCtrl()
  g3.keep, g3.phase, g3.phaseEndAt, g3.pulling, g3.pull = "mount", "done", 0, true, 0.5
  g3.TryHandGlide(0.2, { activePersp = 3 })
  local early = g3.handMatch == nil and not g3.handGlideTried
  g3.TryHandGlide(1.2, { activePersp = 3 }) -- HandoverDelay (0.53 s) after the phase end: composes with the pull-out tail
  check("glide: waits for the pull-out end, at the latest HandoverDelay after the phase end", early and g3.handMatch ~= nil)
  local g4 = glideCtrl({ handGlide = false })
  g4.keep, g4.phase, g4.phaseEndAt, g4.pulling, g4.pull = "mount", "done", 0, true, 1
  g4.TryHandGlide(1.0, { activePersp = 3 })
  check("glide: handGlide off -> nothing", g4.handMatch == nil and not g4.handGlideTried)
end)

runBlock(function() -- learning: a standing vehicle's stable camera outside the vehicle, behind it, not orbited, not closer than a stored pose
  local function learnCtrl()
    local c = newOwn({})
    c.vehCar, c.vehRecKey = true, "Vehicle.v_test_car"
    c.keep, c.phase, c.handAt, c.vehSpeed = "ride", "done", 0, 0
    c.lines = {}
    c.trace = function(m) c.lines[#c.lines + 1] = m end
    return c
  end
  local function drive(c, poseFn, seconds, persp)
    local t = c.learnClock or 0
    local n = math.floor(seconds / 0.05)
    c.gameCamQuery = function() return { poseFn(c, t) } end
    for _ = 1, n do
      t = t + 0.05
      c.clock = t
      c.learnClock = t
      c.UpdateLearn(t, { activePersp = persp or 3, mounted = true }, 1)
    end
  end
  local function lastLine(c, needle)
    local n = 0
    for _, l in ipairs(c.lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
    return n
  end
  local REST = { x = 0.01, y = -9.0, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "active camera" }
  local c = learnCtrl()
  drive(c, function() return REST end, 0.5)
  local L = c.vehTppLearned["Vehicle.v_test_car|3"]
  check("learn: a stable camera is stored once (record|perspective), pitch / FOV kept, logged", L and L.y == -9 and L.pitch == -9
    and L.fov == 69 and lastLine(c, "vehicle TPP pose learned: Vehicle.v_test_car|3 TPPFar (0.01,-9.00,2.50)") == 1
    and (c.stats.vehTppLearns or 0) == 1)
  local c2 = learnCtrl()
  drive(c2, function(_, t) return { x = 0, y = -9 + 0.4 * t, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "active camera" } end, 1.0)
  check("learn: a camera that is still moving (0.4 m/s) is not learned", next(c2.vehTppLearned) == nil)
  local c3 = learnCtrl()
  c3.vehSpeed = 3
  drive(c3, function() return REST end, 0.5)
  check("learn: a moving vehicle learns nothing", next(c3.vehTppLearned) == nil)
  local c4 = learnCtrl()
  drive(c4, function() return { x = 3.2, y = -9.0, z = 2.5, yaw = 20, pitch = -9, fov = 69, src = "active camera" } end, 0.5)
  check("learn: a camera orbited sideways (20 deg) is not stored, logged once", next(c4.vehTppLearned) == nil
    and lastLine(c4, "vehicle TPP pose not learned: Vehicle.v_test_car|3 TPPFar") == 1 and lastLine(c4, "orbited") == 1)
  local c5 = learnCtrl()
  drive(c5, function() return { x = 0, y = -0.8, z = 1.6, yaw = 0, pitch = 0, fov = 80, src = "active camera" } end, 0.5)
  check("learn: a camera inside the vehicle (first person) is never read as a pose", next(c5.vehTppLearned) == nil)
  local c6 = learnCtrl()
  c6.vehTppLearned["Vehicle.v_test_car|3"] = { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "learned" }
  drive(c6, function() return { x = 0, y = -7.0, z = 2.4, yaw = 0, pitch = -9, fov = 69, src = "active camera" } end, 0.5)
  check("learn: a settled camera 2 m closer than the stored pose (collision) does not replace it",
    c6.vehTppLearned["Vehicle.v_test_car|3"].y == -9.0 and lastLine(c6, "collision?") == 1)
  local c7 = learnCtrl()
  c7.vehTppLearned["Vehicle.v_test_car|3"] = { x = 0, y = -8.5, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "learned" }
  drive(c7, function() return { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "active camera" } end, 0.5)
  check("learn: a farther reading refines the stored pose (0.5 m apart: not confirmed)",
    c7.vehTppLearned["Vehicle.v_test_car|3"].y == -9.0 and c7.vehTppLearned["Vehicle.v_test_car|3"].confirmed == false)
  check("learn: the first reading is unconfirmed", L.confirmed == false)
  local c7b = learnCtrl()
  c7b.vehTppLearned["Vehicle.v_test_car|3"] = { x = 0, y = -8.9, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "learned", confirmed = false }
  drive(c7b, function() return REST end, 0.5)
  check("learn: a reading within 0.3 m of the stored one confirms it (logged)", c7b.vehTppLearned["Vehicle.v_test_car|3"].confirmed == true
    and lastLine(c7b, "confirmed by an earlier reading") == 1)
  local c8 = learnCtrl()
  drive(c8, function() return { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = 0, fov = 69, src = "active camera" } end, 0.5)
  local L8 = c8.vehTppLearned["Vehicle.v_test_car|3"]
  check("learn: an orientation reading 0 / 0 stores the rest pitch 0 (the settled camera reads 0), not a look-at estimate",
    L8 and L8.pitch == 0 and lastLine(c8, "rest pitch") == 1 and lastLine(c8, "orientation unread") == 0)
  -- mutation: the previous look-at formula (A/B vehTppRestPitch = nil) gives -9.2 deg, which the continuity gates below reject
  local c8m = learnCtrl()
  c8m.cfg.vehTppRestPitch = nil
  drive(c8m, function() return { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = 0, fov = 69, src = "active camera" } end, 0.5)
  local L8m = c8m.vehTppLearned["Vehicle.v_test_car|3"]
  check("learn (A/B): vehTppRestPitch nil restores the pitch to the look-at point (-9.2 deg)",
    L8m and math.abs(L8m.pitch - math.deg(math.atan2(1.05 - 2.5, 9.0))) < 0.01 and lastLine(c8m, "orientation unread") == 1)
  local c9 = learnCtrl()
  c9.vehBike = true
  drive(c9, function() return REST end, 0.5)
  local c10 = learnCtrl()
  c10.phase = "armed"
  drive(c10, function() return REST end, 0.5)
  local c11 = learnCtrl()
  drive(c11, function() return REST end, 0.5, 0) -- the vehicle camera is FPP
  check("learn: bikes, an unfinished request phase and a first-person vehicle camera learn nothing", next(c9.vehTppLearned) == nil
    and next(c10.vehTppLearned) == nil and next(c11.vehTppLearned) == nil)
  local c12 = learnCtrl()
  c12.cam.releaseUntil = 99
  drive(c12, function() return REST end, 0.5)
  check("learn: not during the hand-over release", next(c12.vehTppLearned) == nil)
end)

-- ================================================================ review round (2026-11-01): M1 bikes, M2 aborted exit, M3 passenger window
runBlock(function() -- M1: the late-entry request and the passenger request are car only (positive identification)
  local function lateRide(set)
    local c = newOwn({})
    if set then set(c) end
    local r = seq(c, { { n = 30, obs = FOOT }, { n = 60 * 9, obs = obsOf(4) }, { n = 150, acc = true, obs = obsOf(1) } })
    return c, r
  end
  local cb, rb = lateRide(function(c) c.vehBike = true end)
  check("M1: bike whose mount hold ran into armWindow: no late-entry request (bikes unchanged)", #rb.requests == 0
    and (cb.stats.lateEntryRides or 0) == 0, tostring(#rb.requests))
  local cu, ru = lateRide(nil)
  check("M1: unknown class: no late-entry request", #ru.requests == 0 and (cu.stats.lateEntryRides or 0) == 0, tostring(#ru.requests))
  local cc, rc = lateRide(function(c) c.vehCar = true end)
  check("M1: positively identified car: the late-entry request is sent", #rc.requests == 1 and (cc.stats.lateEntryRides or 0) == 1)
  local function passRide(set)
    local c = newOwn({})
    if set then set(c) end
    local r = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { driver = false, seat = "seat_back_left" }) },
                       { n = 120, obs = obsOf(3, { driver = false, seat = "seat_back_left", activePersp = 0 }) } })
    return c, r
  end
  local pb, prb = passRide(function(c) c.vehBike = true end)
  check("M1: bike in the passenger state: no request, not forced", #prb.requests == 0 and not pb.passengerForced and pb.keep == "off")
  local pu, pru = passRide(nil)
  check("M1: unknown class in the passenger state: no request", #pru.requests == 0 and not pu.passengerForced)
  local pc, prc = passRide(function(c) c.vehCar = true; c.cfg.passengerChase = false end)
  check("M1: car in the passenger state: requests", #prc.requests >= 1 and pc.passengerForced == true)
end)

runBlock(function() -- M2: an aborted car exit (exit -> ride) asks for the TPP perspective again; the native exit left the vehicle camera in FPP
  local function abortRun(set, afterExit)
    local c = newOwn({})
    if set then set(c) end
    local r1 = seq(c, { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4) }, { n = 150, acc = true, obs = obsOf(1) },
                        { n = 20, obs = obsOf(4, { activePersp = 0 }) } }) -- exit start: the native exit forces FPP
    local base = c.rideBase.car
    local r2 = seq(c, afterExit)
    return c, r1, r2, base
  end
  local backToRide = { { n = 12, acc = true, obs = obsOf(1) }, { n = 90, acc = true, obs = obsOf(1) } }
  local c, r1, r2, base = abortRun(function(c) c.vehCar = true end, backToRide)
  local kinds = {}
  for _, q in ipairs(r2.requests) do kinds[#kinds + 1] = q.p end
  check("M2: car: the aborted exit sends one TPP request (the rideBase perspective) at once, the ride stays keep ride (no game)",
    #r2.requests == 1 and r2.requests[1].state == 1 and r2.requests[1].p == base and c.keep == "ride"
    and keepPath(r2) == "ride" and nLines(r2, "aborted exit: the vehicle camera reads FPP - request") == 1 and c.rideBase.car == base,
    string.format("%d requests, keep %s, path %s", #r2.requests, c.keep, keepPath(r2)))
  local cn, _, rn = abortRun(function(c) c.vehCar = true end, { { n = 90, obs = obsOf(1, { activePersp = 3 }) } })
  check("M2: car: the vehicle camera is TPP again by itself -> no request", #rn.requests == 0 and cn.keep == "ride")
  local cs, _, rs = abortRun(function(c) c.vehCar = true end, { { n = 150, obs = obsOf(1, { activePersp = 0 }) } })
  check("M2: car: a vehicle that never answers is asked a bounded number of times inside one second only",
    #rs.requests >= 2 and #rs.requests <= 5, tostring(#rs.requests))
  local cb2, _, rb2 = abortRun(function(c) c.vehBike = true end, { { n = 12, acc = true, obs = obsOf(1) } })
  check("M2: bike: no request on an aborted exit (unchanged)", #rb2.requests == 0)
  local cu2, _, ru2 = abortRun(nil, { { n = 12, acc = true, obs = obsOf(1) } })
  check("M2: unknown class: no request on an aborted exit (unchanged)", #ru2.requests == 0)
end)

runBlock(function() -- M3: the passenger request keeps trying for 3 s while the camera reads FPP; a TPP held >= 0.5 s ends it
  local function pass(segs)
    local c = newOwn({}); c.vehCar = true; c.cfg.passengerChase = false
    local all = { { n = 30, obs = FOOT }, { n = 60, obs = obsOf(4, { driver = false, seat = "seat_back_left" }) } }
    for _, s in ipairs(segs) do all[#all + 1] = s end
    return c, seq(c, all)
  end
  local P3 = function(a) return obsOf(3, { driver = false, seat = "seat_back_left", activePersp = a }) end
  -- no answer for 1.4 s (a late vanilla FPP request / slow vehicle), then the vehicle takes the next request
  local c, r = pass({ { n = 84, obs = P3(0) }, { n = 30, acc = true, obs = P3(0) }, { n = 60, obs = P3(1) } })
  local late = 0
  for _, q in ipairs(r.requests) do if q.t - r.requests[1].t > 1.0 then late = late + 1 end end
  check("M3: FPP still read 1.4 s into the passenger state -> the request is still sent (window 3 s)", late >= 1 and c.passDone == true,
    string.format("%d requests, %d later than 1 s after the first", #r.requests, late))
  local c2, r2 = pass({ { n = 45, obs = P3(1) }, { n = 120, obs = P3(0) } }) -- TPP held 0.75 s, then the user's FPP
  check("M3: TPP held >= 0.5 s ends the request; a later deliberate FPP switch is respected", #r2.requests == 0 and c2.passDone == true)
  local c3, r3 = pass({ { n = 300, obs = P3(0) } })
  check("M3: FPP for the whole 3 s window, never answered: bounded (<= keepMaxRequests), nothing after the window",
    #r3.requests >= 8 and #r3.requests <= V.DEFAULTS.keepMaxRequests and c3.passDone == true, tostring(#r3.requests))
  local c4, r4 = pass({ { n = 12, obs = P3(1) }, { n = 120, obs = P3(0) } }) -- transient TPPClose 0.2 s (the earlier case)
  check("M3: a transient TPP reading (< 0.5 s) then FPP still gets the request", #r4.requests >= 1)
end)

-- ================================================================ passenger on the own camera (game log 10-34-22: the passenger's vehicle camera stays FPP)
runBlock(function()
  local function chaseRide(set, mountSeg, tail)
    local c = newOwn({})
    c.vehCar = true
    if set then set(c) end
    local segs = { { n = 30, obs = FOOT }, { n = 90, obs = mountSeg } }
    for _, s in ipairs(tail) do segs[#segs + 1] = s end
    return c, seq(c, segs)
  end
  local sw = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = true })  -- seat switch expected: the hold from the start
  local occ = obsOf(4, { driver = false, seat = "seat_front_right", driverSeatFree = false }) -- driver seat taken: decision kept
  local P3 = obsOf(3, { driver = false, seat = "seat_front_right", activePersp = 0, persistentPersp = 3 })
  local ex = obsOf(4, { driver = false, activePersp = 0 })
  local c, r = chaseRide(nil, sw, { { n = 120, obs = P3 }, { n = 40, obs = ex }, { n = 30, obs = FOOT } })
  check("chase: seat switch expected, the seat ends as passenger: mount -> chase (hold kept, no 'free'), no vehicle camera request, "
    .. "exit from the shown pose", keepPath(r) == "off>mount>chase>exit>off" and #ops(r, "free") == 0 and #r.requests == 0
    and nLines(r, "passenger seat (state 3): utppCamera keeps the vehicle-behind chase pose") == 1
    and nLines(r, "exit (exit start (chase)): utppCamera still shown") == 1 and (c.stats.passengerChases or 0) == 1, keepPath(r))
  local hp = 0
  for _, k in ipairs(ops(r, "hold")) do if k.why == "passenger chase" then hp = hp + 1 end end
  check("chase: the camera is set to high priority once for the chase", hp == 1)
  local c2, r2 = chaseRide(nil, occ, { { n = 120, obs = P3 }, { n = 40, obs = ex }, { n = 30, obs = FOOT } })
  check("chase: driver seat taken (no hold at the mount): the hold is taken at the passenger state, pull-out at once, chase, "
    .. "exit", keepPath(r2) == "off>chase>exit>off" and (c2.stats.lateHolds or 0) == 1 and #ops(r2, "vhold") == 1 and #r2.requests == 0
    and nLines(r2, "pull-out start at once (hold taken while seated, state 3)") == 1
    and nLines(r2, "hold taken while seated - utppCamera takes the vehicle-behind chase pose") == 1, keepPath(r2))
  local cB, rB = chaseRide(function(k) k.vehCar = nil; k.vehBike = true end, occ, { { n = 120, obs = P3 } })
  local cU, rU = chaseRide(function(k) k.vehCar = nil end, occ, { { n = 120, obs = P3 } })
  check("chase: bikes and an unreadable class never chase and send no request (unchanged)", cB.keep == "off" and cU.keep == "off"
    and (cB.stats.passengerChases or 0) == 0 and (cU.stats.passengerChases or 0) == 0 and #rB.requests == 0 and #rU.requests == 0)
  local cO, rO = chaseRide(function(k) k.cfg.passengerChase = false end, occ, { { n = 120, obs = P3 } })
  check("chase: passengerChase off -> the request path (no chase)", cO.keep == "off" and #rO.requests >= 1 and (cO.stats.passengerChases or 0) == 0)
  local cN, rN = chaseRide(function(k) k.cfg.pullOut = false end, sw, { { n = 120, obs = P3 } })
  check("chase: pullOut off -> no chase pose to hold: the previous passenger handling", (cN.stats.passengerChases or 0) == 0)
  local cS, rS = chaseRide(nil, sw, { { n = 60, obs = P3 }, { n = 60, obs = obsOf(7, { driver = false, activePersp = 0 }) } })
  check("chase: a scene state (7) during the chase leaves the view to the camera's own rules (off, 'free')",
    cS.keep == "off" and #ops(rS, "free") >= 1 and (cS.stats.passengerChases or 0) == 1, keepPath(rS))
end)


-- ================================================================ 2026-11-01 per-vehicle car camera (logs 15-00-36 / 15-20-28)
-- Settled vehicle cameras read from those logs (vehicle frame, standing vehicle, rendered camera after the release; the
-- forward pitch reads 0 on every car, the Hella TPPFar of log 07-50-12 included).
local SETTLED = {
  { rec = "Vehicle.v_standard25_villefort_columbus", name = "Columbus", persp = 3, x = 0.0, y = -9.00, z = 2.75 },
  { rec = "Vehicle.v_standard3_chevalier_emperor",   name = "Emperor",  persp = 3, x = 0.0, y = -9.50, z = 2.85 },
  { rec = "Vehicle.v_sport1_quadra_turbo_r",         name = "Turbo-R",  persp = 3, x = 0.0, y = -8.00, z = 2.20 },
  { rec = "Vehicle.v_utility4_kaukaz_bratsk",        name = "Bratsk",   persp = 3, x = 0.0, y = -11.00, z = 4.00 },
  { rec = "Vehicle.v_standard2_archer_hella_player", name = "Hella TPPFar", persp = 3, x = 0.0, y = -9.00, z = 2.50 },
  { rec = "Vehicle.v_standard2_archer_hella_player", name = "Hella TPPClose", persp = 1, x = 0.0, y = -4.75, z = 1.25 },
}
-- Handover residual gate: the pose utppCamera glides to vs the settled camera, and what that residual implies per frame at
-- 60 fps in the Deactivate blend that follows (smoothstep peak 1.5 x mean).
local GATE = { posPerFrame = 0.01, rotPerFrame = 0.10, glidePerFrame = 0.09, glideRotPerFrame = 0.6 }
-- a controller about to glide: shown pose (0,-5.5,2.4) pitch -14 = the pull-out end; target by VehTppTarget (cfg overrides =
-- the previous behaviour); runs the glide at 60 fps through V.OwnPose and measures the commanded pose
local function glideRun(rec, persp, cfg, learned)
  local c = newOwn(cfg or {})
  c.vehCar, c.vehRecKey = true, rec
  if learned then c.vehTppLearned[rec .. "|" .. persp] = learned end
  local f = { px = -1, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  c.pullFrame = f
  c.keep, c.phase, c.phaseEndAt, c.pulling, c.pull = "mount", "done", 0, true, 1
  local orbit = { x = -2, y = -3, z = 1.8, yaw = 0, pitch = -10, fov = 56 }
  local dt = 1 / 60
  c.clock = 0
  local p0 = V.OwnPose(c, f, orbit) -- the frozen pull-out end = the shown pose
  c.TryHandGlide(1.0, { activePersp = persp })
  local hm = c.handMatch
  if not hm then return nil, c end
  local prev, maxStep, maxRot = p0, 0, 0
  for i = 1, math.ceil(hm.dur / dt) + 2 do
    c.clock = hm.at + i * dt
    local p = V.OwnPose(c, f, orbit)
    maxStep = math.max(maxStep, math.sqrt((p.pos.x - prev.pos.x) ^ 2 + (p.pos.y - prev.pos.y) ^ 2 + (p.pos.z - prev.pos.z) ^ 2))
    maxRot = math.max(maxRot, math.abs(p.pitch - prev.pitch) + math.abs(p.yaw - prev.yaw))
    prev = p
  end
  return { hm = hm, last = prev, maxStep = maxStep, maxRot = maxRot, c = c }, c
end
-- gate on one settled camera: residual of the glide end vs the settled pose, the Deactivate blend's implied per-frame change,
-- and the glide's own per-frame translation / rotation
local function handoverGate(g, S)
  if not g then return false, "no glide" end
  local l, hm = g.last, g.hm
  local resPos = math.sqrt((l.pos.x - S.x) ^ 2 + (l.pos.y - S.y) ^ 2 + (l.pos.z - S.z) ^ 2)
  local resRot = math.abs(l.pitch - 0) -- the settled camera reads pitch 0
  local pp, pr = V.BlendPeak(resPos, resRot, hm.blend, 60)
  local ok = pp <= GATE.posPerFrame and pr <= GATE.rotPerFrame and g.maxStep <= GATE.glidePerFrame and g.maxRot <= GATE.glideRotPerFrame
  return ok, string.format("residual %.2f m / %.1f deg, blend %.2f s -> %.4f m/frame, %.3f deg/frame; glide max %.3f m, %.2f deg per frame",
    resPos, resRot, hm.blend, pp, pr, g.maxStep, g.maxRot)
end
-- the behaviour before 2026-11-01: only the Hella's class default (look-at pitch -9), 0.45 s blend, learned pitch from the
-- look-at formula, fixed 0.9 s glide
local OLD = { vehTppRestRecords = {}, handGlideDefaultBlend = 0.45, handGlideSpeed = 0,
              vehTppRest = { car = { [3] = { x = 0, y = -9.0, z = 2.5, yaw = 0, pitch = -9, fov = 69, src = "default" } } } }
runBlock(function()
  for _, S in ipairs(SETTLED) do
    local g = glideRun(S.rec, S.persp)
    local ok, why = handoverGate(g, S)
    check("handover gate (" .. S.name .. ", first ride of the session): glide end = the settled camera, implied handover change "
      .. "< 0.01 m / 0.1 deg per frame, glide < 0.09 m / frame", ok and g.hm.src == "record", why)
    -- a later ride: a pose learned by the settled-camera reader (rest pitch 0 since 2026-11-01)
    local g2 = glideRun(S.rec, S.persp, nil, { x = S.x, y = S.y, z = S.z, yaw = 0, pitch = 0, fov = 69, src = "learned", confirmed = true })
    local ok2, why2 = handoverGate(g2, S)
    check("handover gate (" .. S.name .. ", learned pose): same gate", ok2 and g2.hm.src == "learned", why2)
  end
  -- mutation / negative: the previous behaviour fails the gate on the new classes (first ride: class default of the Hella
  -- with a look-at pitch)
  local failed, names = 0, {}
  for _, S in ipairs(SETTLED) do
    if S.rec ~= "Vehicle.v_standard2_archer_hella_player" then
      local g = glideRun(S.rec, S.persp, OLD)
      local ok = handoverGate(g, S)
      if not ok then failed = failed + 1; names[#names + 1] = S.name end
    end
  end
  check("handover gate mutation: the previous behaviour (class default pitch -9, 0.45 s blend) fails the gate for all four new classes",
    failed == 4, table.concat(names, ","))
  -- later ride, old learner: the learned pose carried the look-at pitch (Turbo-R -8.2 deg) with a perfect position
  local lookAtPitch = math.deg(math.atan2(1.05 - 2.2, 8.0))
  local gOld = glideRun("Vehicle.v_sport1_quadra_turbo_r", 3, { vehTppRestRecords = {} },
    { x = 0, y = -8.0, z = 2.2, yaw = 0, pitch = lookAtPitch, fov = 69, src = "learned", confirmed = true })
  local okOld, whyOld = handoverGate(gOld, SETTLED[3])
  check("handover gate mutation: a position-perfect learned Turbo-R pose with the old look-at pitch (-8.2) still fails (rotation snap)",
    not okOld, whyOld)
  -- Hella TPPFar: the old default had the right position but pitch -9: the old behaviour fails the rotation gate, the new one meets it
  local gH = glideRun("Vehicle.v_standard2_archer_hella_player", 3, OLD)
  check("handover gate: the Hella TPPFar class default (look-at pitch -9) also failed the rotation gate before", not (handoverGate(gH, SETTLED[5])))
  -- record table: shipped values equal the logged settled cameras
  local allOk = true
  for _, S in ipairs(SETTLED) do
    local r = V.DEFAULTS.vehTppRestRecords[S.rec] and V.DEFAULTS.vehTppRestRecords[S.rec][S.persp]
    if not (r and near(r.x, S.x) and near(r.y, S.y) and near(r.z, S.z) and r.pitch == 0) then allOk = false end
  end
  check("record table: 4 new classes + the Hella (TPPFar, TPPClose) equal the poses read from the logs, pitch 0", allOk)
  -- the player's own variant of a record ("..._player") finds the shipped pose of the record
  local gp = glideRun("Vehicle.v_sport1_quadra_turbo_r_player", 3)
  check("record table: a '_player' variant of a shipped record uses its pose", gp and gp.hm.src == "record" and near(gp.hm.y, -8.0))
  -- an unknown record keeps the class default with the longer unverified blend
  local gu = glideRun("Vehicle.v_unknown_new_car", 3)
  check("unknown record: class default, 0.80 s residual blend (unverified)", gu and gu.hm.src == "default" and near(gu.hm.blend, 0.80))
  -- the glide time grows with the distance (peak <= 5 m/s) and stays 0.9 s for a short way
  local gB = glideRun("Vehicle.v_utility4_kaukaz_bratsk", 3)
  local gS = glideRun("Vehicle.v_standard2_archer_hella_player", 1)
  check("glide time: Bratsk (5.7 m) 1.7 s cap, short Hella TPPClose way 0.9 s", gB and near(gB.hm.dur, 1.7, 0.01) and gS and near(gS.hm.dur, 0.9, 0.01),
    gB and gS and string.format("%.2f / %.2f", gB.hm.dur, gS.hm.dur))
end)

runBlock(function() -- camTrace continuity measure: per-frame step / rotation, window summary
  local st = V.NewContinuity()
  -- a smooth handover from the Turbo-R door ride (log 15-23-21: pitch -8 -> 0 in 0.2 s, position fixed) at 60 fps
  for i = 0, 36 do
    local t = i / 60
    local w = math.min(1, math.max(0, (t - 0.1) / 0.2)); w = w * w * (3 - 2 * w)
    V.ContinuityStep(st, t, { x = 0, y = -8, z = 2.2, yaw = 0, pitch = -8 * (1 - w) }, 0.1, 0.2)
  end
  check("continuity: smooth pitch release -> max per-frame rotation < 1 deg, no translation", st.max.step == 0 and st.max.rot < 1.0 and st.frames == 36,
    string.format("%.3f deg", st.max.rot))
  local st2 = V.NewContinuity()
  -- the same with the one-frame jump of the Emperor / Columbus rides (1.15 m in one frame at 60 fps)
  for i = 0, 36 do
    V.ContinuityStep(st2, i / 60, { x = 0, y = -9 + (i >= 18 and 1.15 or 0), z = 2.7, yaw = 0, pitch = 0 }, 0.2, 0.45)
  end
  check("continuity: a one-frame 1.15 m jump is measured (max step, time, handover window)", near(st2.max.step, 1.15, 1e-6)
    and near(st2.max.at, 18 / 60, 1e-6) and near(st2.hand.step, 1.15, 1e-6) and st2.hand.frames > 0)
  check("continuity gate: max per-frame translation in the handover window < 0.12 m -> smooth series passes, the jump fails",
    st.hand.step < 0.12 and not (st2.hand.step < 0.12))
  local line = V.ContinuitySummary(st2, 1.0)
  check("continuity: summary line carries the max step, rotation and the handover window",
    string.find(line, "camera continuity", 1, true) and string.find(line, "max per-frame step 1.150 m", 1, true)
    and string.find(line, "handover window", 1, true), line)
  local st3 = V.NewContinuity()
  V.ContinuityStep(st3, 0, { x = 0, y = 0, z = 0, yaw = 0, pitch = 0 })
  local d = V.ContinuityStep(st3, 1.0, { x = 5, y = 0, z = 0, yaw = 0, pitch = 0 }) -- a window gap is not a frame
  check("continuity: frames more than 0.25 s apart are not compared", d == nil and st3.frames == 0)
end)

runBlock(function() -- pull-out duration follows the frozen path (log fixtures: Hella and the new classes)
  local P = V.DEFAULTS
  local function dur(aDeg, r, h)
    return V.PullDuration(V.PullCfg(P, false), { a = math.rad(aDeg), r = r, h = h }, P.pullTime)
  end
  -- {start angle, radius, height up} from the 'pull-out path: start frozen' lines
  local FIX = {
    { "Hella parallel (-98 deg, 3.05 m)", -98, 3.05, 1.60, false },
    { "Hella door (-159 deg, 4.34 m; 11.3 m/s before)", -159, 4.34, 1.74, true },
    { "Columbus door (-160 deg, 4.27 m)", -160, 4.27, 1.63, true },
    { "Emperor door (-163 deg, 4.21 m; 10.7 m/s before)", -163, 4.21, 1.74, true },
    { "Bratsk door (150 deg, 5.67 m; 19.7 m/s before)", 150, 5.67, 1.43, true },
  }
  for _, fx in ipairs(FIX) do
    local T, len, sweep = dur(fx[2], fx[3], fx[4])
    local vpk, wpk = 1.5 * len / T, 1.5 * sweep / T
    local capped = T >= P.pullTimeMax - 1e-9
    -- (r15: pullMaxSpeed 7 -> 6 m/s, the cap 2.2 -> 4.0 s)
    check("pull duration: " .. fx[1] .. " peak speed <= 6 m/s, turn rate <= 110 deg/s (or the 4.0 s cap)",
      (vpk <= 6.0 + 1e-6 and wpk <= 110 + 1e-6) or capped, string.format("T=%.2f s, %.1f m/s, %.0f deg/s", T, vpk, wpk))
    if not fx[5] then
      check("pull duration: " .. fx[1] .. " stays at the old 0.9 s (identical)", near(T, 0.9, 1e-9), tostring(T))
    else
      check("pull duration: " .. fx[1] .. " is slower than the old fixed 0.9 s", T > 0.9 + 1e-6, tostring(T))
    end
  end
  local Tb, Lb = dur(150, 5.67, 1.43)
  -- r15: the 2.2 s cap held the Bratsk door at about 8 m/s; at 6 m/s it is 1.5 x 11.8 m / 6 = 2.95 s, under the 4.0 s cap
  check("pull duration: Bratsk door 1.5 x length / 6 m/s, under the 4.0 s cap (peak 19.7 m/s at 0.9 s -> 6 m/s)",
    near(Tb, 1.5 * Lb / 6.0, 1e-9) and Tb < P.pullTimeMax and Tb > 2.2, string.format("T=%.2f L=%.2f", Tb, Lb))
  -- mutation: pullScale = false restores the fixed 0.9 s
  local Toff = V.PullDuration({ pullScale = false, pullTime = 0.9, pullBack = 5.5, pullHeight = 2.4 }, { a = math.rad(150), r = 5.67, h = 1.43 }, 0.9)
  check("pull duration mutation: pullScale off -> the fixed 0.9 s", near(Toff, 0.9))
  -- through the controller: the path freeze stores the duration, UpdatePull uses it
  local ar = math.rad(150)
  local f = { px = -1, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  local orbit = { x = 5.67 * math.cos(ar), y = 5.67 * math.sin(ar), z = 1.43, yaw = -68, pitch = -14 }
  local c = newOwn({ blendDtCap = 0 }) -- (rate checks with one long step: the stall cap is off here)
  c.keep, c.pulling, c.vehCar, c.keepSince, c.pull = "mount", true, true, -1, 0.01
  c.clock = 0
  V.OwnPose(c, f, orbit)
  -- r15: the duration of a car's path follows the sampled arc length of the actual path (pathStart.arcLen)
  local Tst = c.pathStart and c.pathStart.dur
  check("pull duration: the frozen path stores the duration (Bratsk: 1.5 x arc length / 6 m/s)", c.pathStart and tonumber(c.pathStart.arcLen)
    and near(Tst, math.max(P.pullTime, 1.5 * c.pathStart.arcLen / 6.0), 1e-9), string.format("dur=%s arcLen=%s", tostring(Tst), tostring(c.pathStart and c.pathStart.arcLen)))
  c.pull = 0
  c.UpdatePull(1.0, { vehDist = 0.5 }, 0.1)
  check("pull duration: UpdatePull advances by dt / the stored duration", Tst and near(c.pull, 0.1 / Tst, 1e-6), tostring(c.pull))
  -- an unknown class (IsA failed) and bikes keep the fixed pull time and the previous hand-over timing
  local cu = newOwn({ blendDtCap = 0 })
  cu.keep, cu.pulling, cu.keepSince, cu.pull = "mount", true, -1, 0.01
  cu.clock = 0
  V.OwnPose(cu, f, orbit)
  cu.pull = 0
  cu.UpdatePull(1.0, { vehDist = 0.5 }, 0.09)
  check("pull duration: an unknown class (vehCar nil) keeps the fixed 0.9 s (no stored duration, 0.09 s -> 0.1)",
    cu.pathStart and cu.pathStart.dur == nil and near(cu.pull, 0.1, 1e-6), tostring(cu.pull))
  -- max per-frame commanded translation through the whole pull-out, Bratsk door, 60 fps
  local function sweepMax(cfg)
    local k = newOwn(cfg or {})
    k.keep, k.pulling, k.vehCar, k.keepSince = "mount", true, true, -1
    local prev, mx = nil, 0
    for i = 1, 60 * 3 do
      k.clock = i / 60
      local p = V.OwnPose(k, f, orbit)
      if p and prev then mx = math.max(mx, math.sqrt((p.pos.x - prev.pos.x) ^ 2 + (p.pos.y - prev.pos.y) ^ 2 + (p.pos.z - prev.pos.z) ^ 2)) end
      prev = p
      k.UpdatePull(k.clock, { vehDist = 0.5 }, 1 / 60)
    end
    return mx
  end
  local newMax, oldMax = sweepMax({}), sweepMax({ pullScale = false })
  check("pull-out gate: Bratsk door max per-frame camera translation < 0.15 m (new), the old fixed 0.9 s exceeds it (mutation)",
    newMax < 0.15 and oldMax > 0.15, string.format("new %.3f m, old %.3f m", newMax, oldMax))
end)

runBlock(function() -- exit follow of a standing car: per-vehicle gate, around the body, duration from the path
  local function exitCtrl(extra)
    local c, lines = inRide(extra)
    c.vehCar = true
    c.keep, c.exitPull, c.cam.hold, c.keepSince = "exit", "hold", "exit", 0
    c.exitStartDist = nil
    c.vehSpeed = 0
    return c, lines
  end
  local DOOR = { px = -1.5, py = 3.4, pz = 0.9, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  -- Bratsk driver: V is 3.68 m from the origin at the exit start (log 15-20-28 :7959): no follow at the exit start
  local c = exitCtrl()
  c.pullFrame = DOOR
  c.UpdateExitFollow(0.01, { vehDist = 3.68 })
  check("exit follow: Bratsk seat 3.7 m from the origin -> no follow at the exit start (gate relative to the exit start)",
    c.exitFollow == nil and near(c.exitStartDist, 3.68))
  c.UpdateExitFollow(0.5, { vehDist = 4.5 })
  check("exit follow: 0.8 m further is still inside the gate (start + 1.2 m)", c.exitFollow == nil)
  c.UpdateExitFollow(0.8, { vehDist = 4.95 })
  check("exit follow: beyond start + 1.2 m -> follow, around the vehicle", c.exitFollow ~= nil and c.exitFollow.around == true)
  -- mutation: the old fixed 2.0 m gate fires at once (slack -100 makes the gate max(2.0, start - 100) = 2.0)
  local cm = exitCtrl({ exitFollowSlack = -100 })
  cm.pullFrame = DOOR
  cm.UpdateExitFollow(0.01, { vehDist = 3.68 })
  check("exit follow mutation: the old fixed 2.0 m gate fires at the exit start for the Bratsk seat", cm.exitFollow ~= nil)
  -- unmount of a far seat starts the around follow even without walking away; the hold is extended
  local cu = exitCtrl()
  cu.pullFrame = DOOR
  cu.UpdateExitFollow(0.01, { vehDist = 3.68 })
  cu.keep, cu.unmountAt, cu.cam.holdUntil, cu.clock = "off", 2.0, 2.5, 2.0
  cu.UpdateExitFollow(2.0, { vehDist = 3.8 })
  check("exit follow: unmount of a car seat 3.7 m from the origin starts the follow around the vehicle", cu.exitFollow ~= nil and cu.exitFollow.around
    and string.find(cu.exitFollow.why, "unmount of a car seat", 1, true) ~= nil, cu.exitFollow and cu.exitFollow.why)
  -- Turbo-R / Hella: seat 0.7 m from the origin: the 2.0 m gate is unchanged
  local ct = exitCtrl()
  ct.pullFrame = { px = -2.0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  ct.UpdateExitFollow(0.01, { vehDist = 0.7 })
  ct.UpdateExitFollow(1.0, { vehDist = 1.95 })
  local notYet = ct.exitFollow == nil
  ct.UpdateExitFollow(1.1, { vehDist = 2.05 })
  check("exit follow: short car (seat 0.7 m): the 2.0 m gate is unchanged", notYet and ct.exitFollow ~= nil)
  -- geometry: the follow from the hold pose behind a 9 m truck to the orbit at the driver's door goes around the body
  local ORBIT = { x = -3.2, y = 3.4, z = 2.5, yaw = 90, pitch = -10 } -- the orbit camera around V at the driver's door
  local function followRun(cfg)
    local k = exitCtrl(cfg)
    k.pullFrame = DOOR
    k.UpdateExitFollow(0.01, { vehDist = 3.68 })
    k.keep, k.unmountAt = "off", 1.0
    k.cam.hold, k.cam.holdUntil, k.clock = "exit", 1.6, 1.0
    k.UpdateExitFollow(1.05, { vehDist = 3.7 }) -- far seat: the follow starts at the unmount
    local ef = k.exitFollow
    if not ef then return nil, k end
    local minClear, prev, maxStep = 1e9, nil, 0
    for i = 0, 200 do
      k.clock = ef.at + i / 60
      local p = V.OwnPose(k, k.pullFrame, ORBIT)
      if not p then break end
      -- clearance of the camera from the truck body box in plan (|x| <= 1.3, |y| <= 4.8) below the roof (3.4 m)
      local dx, dy = math.max(math.abs(p.pos.x) - 1.3, 0), math.max(math.abs(p.pos.y) - 4.8, 0)
      if p.pos.z < 3.4 then minClear = math.min(minClear, math.sqrt(dx * dx + dy * dy)) end
      if prev then maxStep = math.max(maxStep, math.sqrt((p.pos.x - prev.pos.x) ^ 2 + (p.pos.y - prev.pos.y) ^ 2 + (p.pos.z - prev.pos.z) ^ 2)) end
      prev = p
    end
    return { minClear = minClear, maxStep = maxStep, dur = ef.dur, ef = ef }, k
  end
  local g, kk = followRun({})
  check("exit follow around: Bratsk driver follow keeps >= 0.3 m clear of the body box, max per-frame step < 0.16 m (9.6 m/s), 1.0-2.0 s",
    g and g.minClear >= 0.3 and g.maxStep < 0.16 and g.dur >= 1.0 and g.dur <= 2.0,
    g and string.format("clear %.2f m, step %.3f m, dur %.2f s", g.minClear, g.maxStep, g.dur))
  check("exit follow around: the hold lasts until the follow ends (holdUntil >= at + dur)", g and kk.cam.holdUntil >= g.ef.at + g.dur - 1e-6)
  -- mutation: the straight 0.25 s offset-carry blend (exitFollowAround off, the old gate) cuts through the cab
  local gm = followRun({ exitFollowAround = false, exitFollowSlack = -100 }) -- the old fixed 2.0 m gate and the straight blend
  check("exit follow mutation: the straight offset-carry blend (old behaviour) passes through the body (clearance < 0.3 m)",
    gm and gm.minClear < 0.3, gm and tostring(gm.minClear))
  -- a jump-out keeps the offset-carrying blend (not around)
  local cj = exitCtrl()
  cj.exitJump, cj.vehSpeed = true, 12
  cj.pullFrame = { px = -4, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  cj.UpdateExitFollow(0.01, { vehDist = 0.7 })
  cj.UpdateExitFollow(0.5, { vehDist = 5 })
  check("exit follow: a jump-out (moving vehicle) keeps the offset-carrying blend, not around the vehicle", cj.exitFollow ~= nil and not cj.exitFollow.around)
  -- 2026-11-04 the native jump-out launch (Game.UTPPVehicleJumpOutEpoch) arms the jump-out of a standing vehicle (double F): UpdateExitFollow polls it
  -- while the exit is up, the baseline is the value at the exit start, the speed classification is unchanged
  do
    local savedGame = Game
    local epoch = 4
    Game = { UTPPVehicleJumpOutEpoch = function() return epoch end }
    local function joCtrl(extra, lines)
      local k = exitCtrl(extra)
      if lines then k.trace = function(m) lines[#lines + 1] = m end end
      k.pullFrame = { px = -2.0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
      k.joEpoch = k.NativeJumpOutEpoch() -- as startOwnExit does
      return k
    end
    local function jumpCmds(k) local n = 0; for _, cm in ipairs(k.cam.cmds) do if cm.op == "jumpout" then n = n + 1 end end; return n end
    local kn = joCtrl()
    kn.UpdateExitFollow(0.01, { vehDist = 0.7 })
    kn.UpdateExitFollow(0.02, { vehDist = 0.7 })
    check("native launch: no new launch -> not a jump-out, no jumpout command", not kn.exitJump and jumpCmds(kn) == 0)
    epoch = 5
    kn.UpdateExitFollow(0.03, { vehDist = 0.7 })
    kn.UpdateExitFollow(0.04, { vehDist = 0.7 })
    check("native launch: a new launch arms the jump-out once (jumpout command with the native reason)", kn.exitJump == true and jumpCmds(kn) == 1
      and kn.cam.cmds[#kn.cam.cmds].why == "jump-out (native launch)")
    -- after the unmount (keep off, the exit hold still up) a later launch counts too
    epoch = 5
    local ku = joCtrl()
    ku.keep, ku.unmountAt = "off", 1.0
    ku.UpdateExitFollow(1.0, { vehDist = 0.7 })
    epoch = 6
    ku.UpdateExitFollow(1.06, { vehDist = 0.7 })
    check("native launch: reported after the unmount while the exit hold is up -> armed", ku.exitJump == true and jumpCmds(ku) == 1)
    -- no exit in progress: the native is not even called
    local calls = 0
    Game = { UTPPVehicleJumpOutEpoch = function() calls = calls + 1; return 99 end }
    local kr = inRide()
    kr.UpdateExitFollow(0.5, {})
    check("native launch: not polled outside an exit", calls == 0 and not kr.exitJump)
    -- disabled by setting / older DLL / failing native
    local kd = joCtrl({ exitJumpNative = false })
    Game = { UTPPVehicleJumpOutEpoch = function() return 100 end }
    kd.UpdateExitFollow(0.1, { vehDist = 0.7 })
    check("native launch: exitJumpNative = false -> ignored", not kd.exitJump and jumpCmds(kd) == 0)
    Game = {}
    local lines = {}
    local kf = joCtrl(nil, lines)
    kf.UpdateExitFollow(0.1, { vehDist = 0.7 })
    kf.UpdateExitFollow(0.2, { vehDist = 0.7 })
    check("native launch: older DLL (native missing) -> speed classification only, logged once", not kf.exitJump and has(lines, "UTPPVehicleJumpOutEpoch unavailable")
      and #lines == 1)
    Game = { UTPPVehicleJumpOutEpoch = function() error("boom") end }
    local ke = joCtrl(nil, lines)
    ke.UpdateExitFollow(0.1, { vehDist = 0.7 })
    check("native launch: a native that raises is caught (pcall), logged once", not ke.exitJump and has(lines, "UTPPVehicleJumpOutEpoch failed") and #lines == 2)
    -- the speed classification still arms a moving vehicle without the native
    Game = {}
    local km = joCtrl()
    km.vehSpeed = 12
    km.vehSample = { x = 0, y = 0, t = 0 }
    km.UpdateExitFollow(0.1, { vehDist = 0.7, vehX = 0, vehY = 1.2 })
    check("native launch: a moving vehicle is still armed by the speed classification", km.exitJump == true and jumpCmds(km) == 1)
    Game = savedGame
  end
  -- a standing short car: the follow lasts offset / 5 m/s (Turbo-R 2.3 m in 0.25 s before)
  local ks = exitCtrl()
  ks.pullFrame = { px = -2.0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  ks.UpdateExitFollow(0.01, { vehDist = 0.7 })
  ks.UpdateExitFollow(1.1, { vehDist = 2.05 })
  ks.clock = 1.1
  V.OwnPose(ks, ks.pullFrame, { x = -2.2, y = -1.5, z = 2.2, yaw = 80, pitch = -14 })
  check("exit follow: a standing car's follow is no longer 0.25 s (offset / 4.5 m/s, 0.25-2.0 s)", ks.exitFollow and ks.exitFollow.around and ks.exitFollow.dur > 0.3,
    ks.exitFollow and tostring(ks.exitFollow.dur))
end)

-- 2026-11-04 jump-out side push (vehicle.jumpOutSidePush): the native launch edge of a standing car queues one small outward velocity change
-- (Game.UTPPQueuePlayerImpulse) so the flight clears the fender. r13a log: V launched at car-frame x=-1.16, y=0.24, the car 1.6 m/s.
do
  local savedGame = Game
  local YAW = 37 -- any car heading: the round trip through the trace's own car frame must give a pure side axis
  local function carAt(yaw) return { x = 100, y = 200, z = 10, yaw = yaw } end
  -- world position of V from car-frame (x right, y forward) coordinates (inverse of V.JoDiagCarFrame)
  local function worldOf(car, lx, ly, lz)
    local r = car.yaw * math.pi / 180
    return { x = car.x + lx * math.cos(r) - ly * math.sin(r), y = car.y + lx * math.sin(r) + ly * math.cos(r), z = car.z + (lz or 0) }
  end
  local cfg = V.DEFAULTS
  check("jump-out side push: setting default on, 0.65 m/s, car speed gate 3 m/s", cfg.jumpOutSidePush == true and cfg.jumpOutPushSpeed == 0.65
    and cfg.jumpOutPushMaxCarSpeed == 3.0)
  -- r13a replay: the push is outward-left along the car's side axis (round trip through the trace's own frame)
  local car = carAt(YAW)
  local v = worldOf(car, -1.16, 0.24, 1.95)
  local d, why = V.JoPushDecide(cfg, car, v, 1.6, true)
  check("jump-out side push: r13a launch (x=-1.16, y=0.24, car 1.6 m/s) -> a push", d ~= nil, why)
  if d ~= nil then
    local after = { x = v.x + d.vx, y = v.y + d.vy, z = v.z }
    local bx, by = V.JoDiagCarFrame(car, v)
    local ax, ay = V.JoDiagCarFrame(car, after)
    check("jump-out side push: left side -> side -1, the car-frame x grows outward by exactly the speed, y unchanged, no vertical part",
      d.side == -1 and near(ax - bx, -0.65, 1e-6) and near(ay - by, 0, 1e-6) and d.vz == 0 and near(d.speed, 0.65, 1e-9),
      string.format("dx=%.4f dy=%.4f", ax - bx, ay - by))
  end
  local dr = V.JoPushDecide(cfg, car, worldOf(car, 1.2, -0.5, 1.5), 0, true)
  local rx = dr and select(1, V.JoDiagCarFrame(car, { x = worldOf(car, 1.2, -0.5, 1.5).x + dr.vx, y = worldOf(car, 1.2, -0.5, 1.5).y + dr.vy, z = 11.5 }))
  check("jump-out side push: right side -> side +1, outward (x 1.2 -> 1.85)", dr ~= nil and dr.side == 1 and near(rx, 1.85, 1e-6), tostring(rx))
  for _, yaw in ipairs({ 0, 90, 180, -135, 270 }) do
    local k = carAt(yaw)
    local p = V.JoPushDecide(cfg, k, worldOf(k, -1.1, 0, 1), 0, true)
    local b = worldOf(k, -1.1, 0, 1)
    local ax, ay = V.JoDiagCarFrame(k, { x = b.x + (p and p.vx or 0), y = b.y + (p and p.vy or 0), z = b.z })
    check(string.format("jump-out side push: yaw %d: pure side axis (x -1.1 -> -1.75, y 0)", yaw), p ~= nil and near(ax, -1.75, 1e-6) and near(ay, 0, 1e-6))
  end
  -- gates
  local function why_(c2, car2, v2, spd, isCar2) local r, w = V.JoPushDecide(c2, car2, v2, spd, isCar2); return r, w end
  local r1, w1 = why_(cfg, car, v, 3.5, true)
  check("jump-out side push: a moving car (3.5 m/s) -> no push", r1 == nil and w1 == "car moves", tostring(w1))
  local r2, w2 = why_(cfg, car, v, 1.6, false)
  check("jump-out side push: not a positively identified car (bike / unknown) -> no push", r2 == nil and w2 == "not a car", tostring(w2))
  local r3, w3 = why_(cfg, car, worldOf(car, -0.3, 0.2, 1.9), 0, true)
  check("jump-out side push: V near the centre line (x -0.3, side unclear) -> no push", r3 == nil and w3 == "side unclear", tostring(w3))
  local r4, w4 = why_(cfg, car, worldOf(car, -2.3, 0.2, 1.9), 0, true)
  check("jump-out side push: V already 2.3 m out -> no push", r4 == nil and w4 == "already clear of the body", tostring(w4))
  local r5, w5 = why_(cfg, car, worldOf(car, -1.1, 3.4, 1.9), 0, true)
  check("jump-out side push: V 3.4 m along the car (a launch behind / ahead of the body) -> no push", r5 == nil and w5 == "beside no part of the car", tostring(w5))
  local r6, w6 = why_(cfg, car, nil, 0, true)
  check("jump-out side push: no pose -> no push", r6 == nil and w6 == "no vehicle pose", tostring(w6))
  local off = setmetatable({ jumpOutSidePush = false }, { __index = cfg })
  local r7, w7 = why_(off, car, v, 0, true)
  check("jump-out side push: setting off -> no push", r7 == nil and w7 == "setting off", tostring(w7))
  local z = setmetatable({ jumpOutPushSpeed = 0 }, { __index = cfg })
  check("jump-out side push: speed 0 -> no push", why_(z, car, v, 0, true) == nil)
  local big = setmetatable({ jumpOutPushSpeed = 9 }, { __index = cfg })
  local rb = V.JoPushDecide(big, car, v, 0, true)
  check("jump-out side push: a huge configured speed is clamped to 1.5 m/s (the reds side refuses more than 2)", rb ~= nil and near(rb.speed, 1.5, 1e-9))

  -- controller: the native launch edge arms the push once (not without a launch, not with the setting off)
  local epoch = 4
  Game = { UTPPVehicleJumpOutEpoch = function() return epoch end }
  local function launchCtrl(extra, carKnown)
    local k = inRide(extra)
    k.vehCar = carKnown ~= false or nil
    k.keep, k.exitPull, k.cam.hold, k.keepSince, k.vehSpeed = "exit", "hold", "exit", 0, 1.6
    k.pullFrame = { px = -2.0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
    k.joEpoch = k.NativeJumpOutEpoch()
    return k
  end
  local kn = launchCtrl()
  kn.UpdateExitFollow(0.01, { vehDist = 0.7 })
  check("jump-out side push: no launch -> nothing pending", kn.joPush == nil)
  epoch = 5
  kn.UpdateExitFollow(0.03, { vehDist = 0.7 })
  check("jump-out side push: the native launch arms one push (epoch, time, car speed, car = true)", kn.joPush ~= nil and kn.joPush.epoch == 5
    and near(kn.joPush.at, 0.03) and near(kn.joPush.speed, 1.6) and kn.joPush.car == true)
  local ko = launchCtrl({ jumpOutSidePush = false })
  epoch = 6
  ko.UpdateExitFollow(0.03, { vehDist = 0.7 })
  check("jump-out side push: setting off -> the launch arms the jump-out as before but nothing is queued", ko.exitJump == true and ko.joPush == nil)
  local kb = launchCtrl(nil, false)
  epoch = 7
  kb.UpdateExitFollow(0.03, { vehDist = 0.7 })
  check("jump-out side push: an unidentified vehicle is carried as car = false", kb.joPush ~= nil and kb.joPush.car == false)

  -- run: one native call, one log line; every refusal logs its reason and calls nothing
  local calls = {}
  local function mockGame(ret)
    Game = { UTPPQueuePlayerImpulse = function(x, y, z) calls[#calls + 1] = { x = x, y = y, z = z }; if ret == "raise" then error("boom") end; if ret ~= nil then return ret end; return true end }
  end
  local function runCase(pending, sample, lg, clock)
    local k = inRide()
    k.joPush = pending
    return V.JoPushRun(k, clock or 0.05, sample, function(m) lg[#lg + 1] = m end), k
  end
  local sample = { car = car, v = v }
  mockGame()
  local lg = {}
  local res, kr = runCase({ epoch = 1, at = 0.03, speed = 1.6, car = true }, sample, lg)
  check("jump-out side push run: applied with exactly one Game.UTPPQueuePlayerImpulse call of the decided vector",
    res == "applied" and #calls == 1 and near(calls[1].z, 0) and near(math.sqrt(calls[1].x ^ 2 + calls[1].y ^ 2), 0.65, 1e-9), tostring(res) .. " calls=" .. #calls)
  check("jump-out side push run: one log line with the car-frame position, side and applied velocity",
    #lg == 1 and has(lg, "jump-out side push applied: car-frame x=-1.16 y=0.24, side left, 0.65 m/s -> world (") and has(lg, "car 1.6 m/s"), lg[1])
  check("jump-out side push run: the pending push is consumed (a second run does nothing)", kr.joPush == nil and V.JoPushRun(kr, 0.06, sample, nil) == nil and #calls == 1)
  calls, lg = {}, {}
  local rm = runCase({ epoch = 1, at = 0.03, speed = 5, car = true }, sample, lg)
  check("jump-out side push run: a moving car -> skipped with the reason, no call", rm == "skipped" and #calls == 0 and has(lg, "skipped (car moves)"), lg[1])
  calls, lg = {}, {}
  local rbk = runCase({ epoch = 1, at = 0.03, speed = 0, car = false }, sample, lg)
  check("jump-out side push run: a bike / unknown vehicle -> skipped, no call", rbk == "skipped" and #calls == 0 and has(lg, "skipped (not a car)"))
  calls, lg = {}, {}
  local rs = runCase({ epoch = 1, at = 0.03, speed = 0, car = true }, sample, lg, 0.9)
  check("jump-out side push run: a launch older than 0.5 s is dropped, no call", rs == "skipped" and #calls == 0 and has(lg, "dropped"))
  calls, lg = {}, {}
  local rn = runCase({ epoch = 1, at = 0.03, speed = 0, car = true }, nil, lg)
  check("jump-out side push run: unreadable poses -> skipped, no call", rn == "skipped" and #calls == 0 and has(lg, "no vehicle pose"))
  calls, lg = {}, {}
  mockGame(false)
  local rf = runCase({ epoch = 1, at = 0.03, speed = 0, car = true }, sample, lg)
  check("jump-out side push run: the reds side refusing (false) -> FAILED logged", rf == "failed" and #calls == 1 and has(lg, "jump-out side push FAILED") and has(lg, "returned false"))
  calls, lg = {}, {}
  mockGame("raise")
  local rr = runCase({ epoch = 1, at = 0.03, speed = 0, car = true }, sample, lg)
  check("jump-out side push run: a raising native is caught -> FAILED logged", rr == "failed" and has(lg, "jump-out side push FAILED") and has(lg, "boom"))
  calls, lg = {}, {}
  Game = {}
  local ru = runCase({ epoch = 1, at = 0.03, speed = 0, car = true }, sample, lg)
  check("jump-out side push run: an older install without utpp_vehicle_exit.reds -> unavailable, logged, no error", ru == "unavailable" and has(lg, "UTPPQueuePlayerImpulse unavailable"))
  check("jump-out side push: nothing pending -> JoPushRun does nothing", V.JoPushRun(inRide(), 0.1, sample, nil) == nil)
  -- module setter (the setting's apply callback)
  check("jump-out side push: SetJumpOutSidePush switches the live config", V.SetJumpOutSidePush(false) == false and V.SetJumpOutSidePush(true) == true)
  Game = savedGame
end

-- 2026-11-04 r13 jump-out push diagnostic ('jo diag' lines): gate, throttle, window, car frame, format
do
  local cfgOn, cfgOff = { camTrace = true, joDiagHz = 20 }, { camTrace = false, joDiagHz = 20 }
  check("jo diag gate: camTrace + jump-out armed + V out of the car", V.JoDiagActive(cfgOn, true, false) == true)
  check("jo diag gate: camTrace off -> inactive", V.JoDiagActive(cfgOff, true, false) == false)
  check("jo diag gate: no jump-out -> inactive", V.JoDiagActive(cfgOn, false, false) == false)
  check("jo diag gate: still mounted -> inactive", V.JoDiagActive(cfgOn, true, true) == false)
  check("jo diag gate: default config keeps it off", V.JoDiagActive(V.DEFAULTS, true, false) == false and V.DEFAULTS.joDiagHz == 20)

  -- car frame (the camera trace convention c * rx - s * ry, s * rx + c * ry with c, s = cos / sin(-yaw)): the car at (100, 50, 10)
  -- yawed 90 deg looks along world -x, its right is world +y
  local car = { x = 100, y = 50, z = 10, yaw = 90 }
  local lx, ly, lz = V.JoDiagCarFrame(car, { x = 99, y = 50, z = 10.8 })
  check("jo diag car frame: yaw 90, 1 m ahead of the car (world -x) = y 1", math.abs(lx - 0) < 1e-6 and math.abs(ly - 1) < 1e-6 and math.abs(lz - 0.8) < 1e-9,
    string.format("%.3f %.3f %.3f", lx, ly, lz))
  lx, ly, lz = V.JoDiagCarFrame(car, { x = 100, y = 52, z = 10 })
  check("jo diag car frame: yaw 90, 2 m to the car's right (world +y) = x 2", math.abs(lx - 2) < 1e-6 and math.abs(ly) < 1e-6, string.format("%.3f %.3f", lx, ly))
  lx, ly = V.JoDiagCarFrame({ x = 0, y = 0, z = 0, yaw = 0 }, { x = 1.5, y = -2, z = 0 })
  check("jo diag car frame: yaw 0 is the identity", math.abs(lx - 1.5) < 1e-9 and math.abs(ly + 2) < 1e-9)

  -- a stationary-car flight: rises, falls at 20 Hz (V world z), lands, is pushed 1.7 m sideways, rests
  local st = V.NewJoDiag()
  local lines, t = {}, 0
  local function tick(active, vx, vz, door)
    local line = V.JoDiagTick(st, t, active, { car = { x = 0, y = 0, z = 0, yaw = 0 }, v = { x = vx, y = 0, z = vz } }, door)
    if line then lines[#lines + 1] = line end
    return line
  end
  check("jo diag throttle: first sample is due", V.JoDiagDue(st, 0, 20))
  local z, x = 0.5, 1.5
  local vzs = { 3, 1, -3, -6, -9, -11, -4, -0.1, 0, 0, 0 } -- m/s per step, 0.05 s steps
  local firstLine
  for i = 0, #vzs do
    t = i * 0.05
    if i > 0 then z = math.max(0, z + vzs[i] * 0.05); if i >= 8 then x = x + 0.3 end end
    local due = V.JoDiagDue(st, t, 20)
    check("jo diag throttle: 20 Hz steps are due (" .. i .. ")", due)
    local l = tick(true, x, z, i == 3 and "fl:Open*" or nil)
    if i == 0 then firstLine = l end
  end
  check("jo diag throttle: a frame 16 ms after a sample is not due", not V.JoDiagDue(st, t + 0.016, 20) and V.JoDiagDue(st, t + 0.05, 20))
  check("jo diag throttle: a clock that went back is due", V.JoDiagDue(st, t - 5, 20))
  check("jo diag: a line per tick", #lines == #vzs + 1, tostring(#lines))
  check("jo diag format: prefix, t, car, vcar, onG, carV",
    firstLine:match("^%[utpp_vehiclecam%] jo diag t=0%.000 car=%(1%.50,0%.00,0%.50%) vcar=%(%-,%-,%-%) onG=0 carV=%- ") ~= nil, firstLine)
  check("jo diag format: relative velocity, onG, door, jt", lines[4]:match("^%[utpp_vehiclecam%] jo diag t=0%.150 car=%(1%.50,0%.00,[%d%.%-]+%) vcar=%(0%.00,0%.00,%-?[%d%.]+%) onG=0 carV=0%.00 vzw=%-?[%d%.]+ jt=0%.15 door=fl:Open%*$") ~= nil, lines[4])
  local landed
  for i, l in ipairs(lines) do if l:find("onG=1", 1, true) then landed = landed or i end end
  check("jo diag onG: derived 1 after the descent ends (|vzw| < 0.5)", landed ~= nil and landed >= 7, tostring(landed))
  check("jo diag push: the 0.3 m / 0.05 s lateral step shows as vcar x = 6.00 m/s", lines[#lines]:find("vcar=(6.00,", 1, true) ~= nil or lines[10]:find("vcar=(6.00,", 1, true) ~= nil, lines[10])
  -- the window closes 1.5 s after the landing: samples stay on for 1.5 s, then nothing
  local landT = (landed - 1) * 0.05
  local lastEmit = t
  local closedAt
  for i = 1, 60 do
    t = t + 0.05
    local l = tick(true, x, 0)
    if l then lastEmit = t elseif not closedAt then closedAt = t end
  end
  check("jo diag window: last line at landing + 1.5 s", math.abs(lastEmit - (landT + 1.5)) < 0.051, string.format("last %.2f landing %.2f", lastEmit, landT))
  check("jo diag window: closed after that (done) and silent", closedAt ~= nil and st.done == true and tick(true, x, 0) == nil)
  -- the next jump-out re-arms the window (exitJump false in between resets it)
  check("jo diag window: inactive resets", V.JoDiagTick(st, t, false, nil) == nil and st.done == false and st.open == false)
  t = t + 1
  check("jo diag window: re-opens for the next jump-out", tick(true, 0, 1) ~= nil and st.open == true)

  -- a false contact (stop) followed by a fall does not count as the landing; the hard cap closes a flight that never lands
  local s2 = V.NewJoDiag()
  local seq = { 0, -3, -6, 0, 0, -8, -9, 0, 0 } -- world z deltas as speeds: contact at idx 4/5, falls again at 6/7, lands at 8
  local zz, tt, lan = 5, 0, {}
  for i, v in ipairs(seq) do
    tt = i * 0.05
    zz = zz + v * 0.05
    local l = V.JoDiagTick(s2, tt, true, { car = { x = 0, y = 0, z = 0, yaw = 0 }, v = { x = 0, y = 0, z = zz } })
    lan[i] = (l and l:find("onG=1", 1, true)) and 1 or 0
  end
  check("jo diag false contact: onG=1 at the sill contact, back to 0 when V falls again, 1 at the real landing",
    lan[5] == 1 and lan[7] == 0 and lan[9] == 1, table.concat(lan, ","))
  local s3 = V.NewJoDiag()
  local n, closed = 0, nil
  for i = 0, 200 do
    local l = V.JoDiagTick(s3, i * 0.05, true, { car = { x = 0, y = 0, z = 0, yaw = 0 }, v = { x = 0, y = 0, z = 10 + i * 0.01 } })
    if l then n = n + 1 elseif not closed then closed = i * 0.05 end
  end
  check("jo diag hard cap: a flight that never lands stops after JO_DIAG_MAX", closed ~= nil and closed > V.JO_DIAG_MAX - 0.06 and closed < V.JO_DIAG_MAX + 0.1, tostring(closed))
  -- unreadable sample: no line, counted, nothing thrown; frozen pose is flagged
  local s4 = V.NewJoDiag()
  check("jo diag unreadable sample: nil line, counted", V.JoDiagTick(s4, 0, true, nil) == nil and s4.misses == 1)
  local lf = V.JoDiagTick(s4, 0.05, true, { car = { x = 0, y = 0, z = 0, yaw = 0 }, v = { x = 1, y = 0, z = 0 }, frozen = true })
  check("jo diag frozen car pose: frz=1 marker", lf ~= nil and lf:match(" frz=1$") ~= nil, tostring(lf))
  -- the car moves (moving jump-out): carV from the live pose, V's relative velocity excludes the car's own motion
  local s5 = V.NewJoDiag()
  V.JoDiagTick(s5, 0, true, { car = { x = 0, y = 0, z = 0, yaw = 0 }, v = { x = 2, y = 0, z = 1 } })
  local lm = V.JoDiagTick(s5, 0.05, true, { car = { x = 0, y = 0.4, z = 0, yaw = 0 }, v = { x = 2, y = 0.4, z = 1 } })
  check("jo diag moving car: carV=8.00, vcar y = 0 (V moves with the car)", lm:find("carV=8.00", 1, true) ~= nil and lm:find("vcar=(0.00,0.00,0.00)", 1, true) ~= nil, lm)
end

-- 2026-11-04 r13 'jo diag' in-game wiring smoke: the real module loaded a second time with CET-like globals (registerForEvent, Game, GetMod)
-- and driven through onInit / onUpdate: the car entity captured while mounted, its LIVE pose read after the unmount, the gate (camTrace),
-- the lines in the log sink, a failing entity read -> frozen pose marker
do
  local savedReg, savedGame, savedGetMod = registerForEvent, Game, GetMod
  local handlers, logged = {}, {}
  local carPos = { x = 100, y = 50, z = 10 }
  local carOk = true
  local car = {
    GetWorldPosition = function() if not carOk then error("entity gone") end return { x = carPos.x, y = carPos.y, z = carPos.z } end,
    GetWorldYaw = function() return 0 end,
  }
  local mounted = false
  local vpos = { x = 101.5, y = 50, z = 10.8 }
  local player = {
    GetEntityID = function() return { hash = 1 } end,
    GetWorldPosition = function() return { x = vpos.x, y = vpos.y, z = vpos.z } end,
    GetWorldYaw = function() return 0 end,
  }
  Game = {
    GetPlayer = function() return player end,
    GetMountedVehicle = function() return mounted and car or nil end,
    UTPPLogLine = function(m) logged[#logged + 1] = m end,
  }
  GetMod = function(name)
    if name == "utpp_camera" then
      return setmetatable({ VERSION = "test" }, { __index = function() return function() return nil end end })
    end
    return nil
  end
  registerForEvent = function(name, fn) handlers[name] = fn end
  local W = dofile(UTPP_VEHICLECAM_PATH)
  local okRun, errRun = pcall(function()
    handlers.onInit()
    for _ = 1, 305 do handlers.onUpdate(1 / 60) end -- startDelay frames, then the path is on
    for _ = 1, 5 do mounted = true; handlers.onUpdate(1 / 60) end -- the ride: the car entity is captured
    mounted = false
    W.ctrl.cfg.camTrace = false
    W.ctrl.exitJump = true
    for _ = 1, 30 do handlers.onUpdate(1 / 60) end -- camTrace off: nothing
    local quiet = 0
    for _, l in ipairs(logged) do if l:find("jo diag", 1, true) then quiet = quiet + 1 end end
    check("jo diag wiring: camTrace off -> no line", quiet == 0, tostring(quiet))
    W.ctrl.cfg.camTrace = true
    local z = 10.8
    for i = 1, 60 do -- 1 s: V falls 11 m/s while the car drives on at 6 m/s
      z = math.max(10, z - 11 / 60); vpos.z = z
      carPos.y = carPos.y + 0.1
      handlers.onUpdate(1 / 60)
    end
  end)
  check("jo diag wiring: onInit / onUpdate ran without a raised error", okRun, tostring(errRun))
  local diag = {}
  for _, l in ipairs(logged) do if l:find("jo diag", 1, true) then diag[#diag + 1] = l end end
  check("jo diag wiring: the car entity of the ride was captured", W.wiring.joVeh == car)
  check("jo diag wiring: ~20 Hz lines in 1 s (not one per frame)", #diag >= 17 and #diag <= 22, tostring(#diag))
  check("jo diag wiring: lines carry the live car pose (V 1.5 m to the right of the car, carV ~ 6 m/s)",
    diag[#diag] ~= nil and diag[#diag]:find("car=(1.50,", 1, true) ~= nil and diag[#diag]:match("carV=(%d+%.%d+)") ~= nil
      and math.abs(tonumber(diag[#diag]:match("carV=(%d+%.%d+)")) - 6) < 0.6 and not diag[#diag]:find("frz=1", 1, true), tostring(diag[#diag]))
  carOk = false -- the entity read fails: the last mounted pose is used and marked
  W.wiring.jo = W.NewJoDiag()
  logged = {}
  Game.UTPPLogLine = function(m) logged[#logged + 1] = m end
  for _ = 1, 10 do handlers.onUpdate(1 / 60) end
  local frz = nil
  for _, l in ipairs(logged) do if l:find("jo diag", 1, true) then frz = l end end
  check("jo diag wiring: unreadable car entity -> frozen pose, frz=1", frz ~= nil and frz:find("frz=1", 1, true) ~= nil, tostring(frz))
  registerForEvent, Game, GetMod = savedReg, savedGame, savedGetMod
end

do -- 2026-11-04 r14 (r13c entry review): 'from behind' mounts - the camera never goes through the body, the yaw never flips, the end pose is the ride camera
  local function wrap(d) return (d + 180) % 360 - 180 end
  local function smooth(t) t = math.max(0, math.min(1, t)); return t * t * (3 - 2 * t) end
  local VYAW, VPOS = 30, { x = 100, y = 200, z = 50 } -- the car in the world
  local function vehToWorld(x, y, z)
    local r = math.rad(VYAW)
    return VPOS.x + math.cos(r) * x - math.sin(r) * y, VPOS.y + math.sin(r) * x + math.cos(r) * y, VPOS.z + z
  end
  local function worldToVeh(wx, wy, wz)
    local r = math.rad(-VYAW)
    local dx, dy = wx - VPOS.x, wy - VPOS.y
    return math.cos(r) * dx - math.sin(r) * dy, math.sin(r) * dx + math.cos(r) * dy, wz - VPOS.z
  end
  local function polar(aDeg, r, z) return { r * math.cos(math.rad(aDeg)), r * math.sin(math.rad(aDeg)), z } end
  -- Logged starts of the r13c entries (vehicle frame; v = the mount anchor of V, cam = the orbit camera, yaw = its yaw relative to the car).
  -- behind / door / parallel = the frozen start of the log ("start frozen at A deg, R m, H up"), 'escape' = the hold camera 0.3 s after the mount (cam veh= lines).
  -- inside: the start is inside the box of the record; route: the plan takes the exit leg (false: the cylinder path leaves the box first / no start inside).
  local REC = { sport1 = "Vehicle.v_sport1_quadra_turbo_r", sport2 = "Vehicle.v_sport2_rayfield_aerondight", standard25 = "Vehicle.v_standard25_villefort_columbus_player",
                standard3 = "Vehicle.v_standard3_chevalier_emperor", utility4 = "Vehicle.v_utility4_kaukaz_bratsk" }
  local ANCH = { sport1 = { -1.60, -0.10, -0.3 }, sport2 = { -2.03, -0.79, -0.3 }, standard25 = { -1.70, -0.33, -0.41 }, standard3 = { -2.02, -0.53, -0.52 },
                 utility4 = { -2.70, 3.66, -0.89 } }
  local SEAT = { sport1 = { -0.5, 0, 0.1 }, sport2 = { -0.5, 0, 0.1 }, standard25 = { -0.5, 0, 0.1 }, standard3 = { -0.5, 0, 0.1 }, utility4 = { -0.8, 3.7, -0.4 } }
  local function case(cls, kind, cam, yaw, inside, route)
    return { name = cls .. " " .. kind, rec = REC[cls], v = ANCH[cls], seat = SEAT[cls], cam = cam, yaw = yaw, inside = inside, route = route, kind = kind }
  end
  local BEHIND = {
    case("utility4", "behind (freeze 0.9 s)", { -1.22, 4.29, 0.91 }, 93, true, false),
    case("utility4", "behind (escape 0.3 s)", { -1.28, 4.28, 0.86 }, 93, true, false),
    case("standard25", "behind (escape 0.3 s)", { -0.56, 0.07, 1.27 }, 90, true, true),
    case("standard3", "behind (freeze 0.9 s)", { -0.21, 0.09, 1.40 }, 89, true, true),
    case("standard3", "behind (escape 0.3 s)", { -0.66, -0.06, 1.25 }, 89, true, true),
    case("sport1", "behind", polar(26, 1.35, 1.49), 100, false, false),
    case("sport2", "behind", polar(-102, 3.50, 1.71), 100, false, false),
  }
  -- entries the r13c review called clean: nothing may change
  local CLEAN = {
    case("sport1", "door", polar(-167, 4.51, 1.57), -30, false, false),
    case("sport1", "parallel", polar(-101, 2.92, 1.57), -30, false, false),
    case("sport1", "parallel 2", polar(-105, 3.13, 1.43), -30, false, false),
    case("sport2", "door", polar(-159, 5.02, 1.59), -30, false, false),
    case("sport2", "parallel", polar(-108, 3.76, 1.59), -30, false, false),
    case("sport2", "parallel 2", polar(-105, 3.63, 1.52), -30, false, false),
    case("standard25", "door", polar(-164, 4.30, 1.61), -30, false, false),
    case("standard25", "parallel", polar(-114, 1.80, 1.47), -30, false, false),
    case("standard25", "parallel 2", polar(-108, 2.24, 1.49), -30, false, false),
    case("standard3", "door", polar(-159, 4.65, 1.62), -30, false, false),
    case("standard3", "parallel", polar(-103, 3.24, 1.62), -30, false, false),
    case("standard3", "parallel 2", polar(-104, 2.82, 1.56), -30, false, false),
    case("utility4", "door", polar(154, 5.91, 0.99), -30, false, false),
    case("utility4", "parallel", polar(151, 1.97, 1.47), -30, false, false),
    case("utility4", "parallel 2", polar(147, 1.87, 1.49), -30, false, false),
  }
  local function replay(case, fps, cfgOver)
    local c = newOwn(cfgOver or {})
    c.keep, c.pulling, c.vehCar, c.vehRecKey = "mount", true, true, case.rec
    local dt = 1 / fps
    local hull = V.HullFor(c.cfg, case.rec)
    local ox, oy, oz = vehToWorld(case.cam[1], case.cam[2], case.cam[3])
    local orbit = { x = ox, y = oy, z = oz, yaw = VYAW + case.yaw, pitch = -14 }
    local function frame(t)
      -- V slides 0.6 m towards the seat over 0.3 s (the mount slide of the r12 log; the live V the clearance reads)
      local dx, dy, dz = case.seat[1] - case.v[1], case.seat[2] - case.v[2], case.seat[3] - case.v[3]
      local len = math.sqrt(dx * dx + dy * dy + dz * dz)
      local k = math.max(0, math.min(1, t / 0.3)) * math.min(1, 0.6 / len)
      local vx, vy, vz = case.v[1] + dx * k, case.v[2] + dy * k, case.v[3] + dz * k
      local wx, wy, wz = vehToWorld(vx, vy, vz)
      return { px = wx, py = wy, pz = wz, pyaw = VYAW + 60, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
    end
    c.clock = 0
    c.pull = 1e-6
    local f0 = frame(0)
    c.pullFrame = f0
    local first = V.OwnPose(c, f0, orbit) -- freezes the path (w ~ 0: the orbit pose itself)
    local T = (c.pathStart and c.pathStart.dur) or 2.2
    local out, n = { first }, math.ceil(T * fps) + 2
    do
      local vx, vy, vz = worldToVeh(first.pos.x, first.pos.y, first.pos.z)
      first.vx, first.vy, first.vz, first.t, first.w = vx, vy, vz, 0, 0
      first.depth = V.HullDepth(hull, vx, vy, vz)
    end
    for i = 1, n do
      local t = i * dt
      c.clock = t
      c.pull = math.min(1, t / T)
      local f = frame(t)
      c.pullFrame = f
      local p = V.OwnPose(c, f, orbit)
      local vx, vy, vz = worldToVeh(p.pos.x, p.pos.y, p.pos.z)
      p.vx, p.vy, p.vz, p.t, p.w = vx, vy, vz, t, c.pull
      p.depth = V.HullDepth(hull, vx, vy, vz)
      out[#out + 1] = p
    end
    return out, c, hull, T
  end
  local function maxSteps(out)
    local ys, ps = 0, 0
    for i = 2, #out do
      local dt = out[i].t - out[i - 1].t
      ys = math.max(ys, math.abs(wrap(out[i].yaw - out[i - 1].yaw)) / dt)
      ps = math.max(ps, math.sqrt((out[i].vx - out[i - 1].vx) ^ 2 + (out[i].vy - out[i - 1].vy) ^ 2 + (out[i].vz - out[i - 1].vz) ^ 2) / dt)
    end
    return ys, ps
  end
  local function insideFrames(out, thr)
    local n = 0
    for i = 1, #out do if out[i].depth > thr then n = n + 1 end end
    return n
  end
  -- the old yaw of the same replay: the shortest arc of every frame (the formula of r13c), from the path the controller froze
  local function oldYaws(out, c)
    local ps, res = c.pathStart, {}
    for i = 1, #out do
      local w = smooth(out[i].w)
      if ps.arc then w = V.PullArcW(ps.arc, w) end -- r15: the same path weight as the controller (eased along the arc length)
      local f = { px = 0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = VYAW }
      local _, yaw = V.PullPathPose(ps, w, f, c.cfg)
      local oyaw = VYAW + ps.oyawV
      res[i] = oyaw + wrap(yaw - oyaw) * w
    end
    return res
  end
  for _, case in ipairs(BEHIND) do
    for _, fps in ipairs({ 30, 60, 144 }) do
      local out, c, hull = replay(case, fps)
      -- (r15: a start inside the box keeps the path weight of r14; the plain cylinder path of r13c is compared with the same weight)
      local plain = replay(case, fps, { pullHull = false, pullArcLength = false })
      local ps = c.pathStart
      local rt = ps and ps.route
      local wClear = rt and rt.wE or ((ps and ps.clampFrom) or 0)
      local d0 = V.HullDepth(hull, case.cam[1], case.cam[2], case.cam[3])
      check(string.format("r14 %s @%d fps: the start is %s the box, the plan %s", case.name, fps, case.inside and "inside" or "outside",
        case.route and "takes the exit leg" or "keeps the cylinder path"), (d0 > 0.05) == case.inside and (rt ~= nil) == case.route,
        string.format("depth0=%.2f route=%s", d0, tostring(rt ~= nil)))
      -- outside the box for the rest of the path once the exit leg / the cylinder path has left it
      local worst, legMax, worstW = -1e9, -1e9, 0
      for i = 1, #out do
        local p = out[i]
        if smooth(p.w) > wClear + 0.03 then
          if p.depth > worst then worst, worstW = p.depth, p.w end
        else
          legMax = math.max(legMax, p.depth)
        end
      end
      check(string.format("r14 %s @%d fps: the camera is outside the box for the rest of the path (deepest %.3f m at pull %.2f)", case.name, fps, worst, worstW),
        worst <= 1e-6, string.format("%.3f at pull %.2f", worst, worstW))
      -- (the clearance around V, who slides toward the camera, may push the camera a little deeper during the leg - not by more than 0.25 m)
      check(string.format("r14 %s @%d fps: before that no deeper than 0.25 m beyond the start (%.2f of %.2f m)", case.name, fps, legMax, d0),
        legMax <= math.max(d0, 0) + 0.25)
      -- never more frames inside the box than the plain cylinder path of r13c
      local nNew, nOld = insideFrames(out, 0.05), insideFrames(plain, 0.05)
      check(string.format("r14 %s @%d fps: frames inside the box %d <= the plain cylinder path's %d", case.name, fps, nNew, nOld), nNew <= nOld)
      local ys, ps2 = maxSteps(out)
      -- (the r13c cylinder paths peak at 9-10 m/s; the radius clamp lifts the path by up to 0.15 m where it passes a corner of the box, a one-frame step of
      -- about 19 m/s at 144 fps for the 11 m truck)
      check(string.format("r14 %s @%d fps: yaw rate <= 250 deg/s (%.0f), speed <= 20 m/s (%.1f)", case.name, fps, ys, ps2), ys <= 250 and ps2 <= 20,
        string.format("%.0f deg/s %.1f m/s", ys, ps2))
      -- end pose = the ride camera behind the car (PullPose: 5.5 m behind, 2.4 m up, looking forward)
      local e = out[#out]
      local tw = V.PullPose(0, 0, 0, 0, 0, 0, 0, 0, c.cfg)
      check(string.format("r14 %s @%d fps: the end pose is the ride camera (0, -5.5, 2.4) looking forward", case.name, fps),
        near(e.vx, tw.x, 1e-6) and near(e.vy, tw.y, 1e-6) and near(e.vz, tw.z, 1e-6) and near(wrap(e.yaw - VYAW), 0, 1e-6),
        string.format("(%.3f,%.3f,%.3f) yaw %.3f", e.vx, e.vy, e.vz, wrap(e.yaw - VYAW)))
    end
  end
  -- the replay reproduces the r13c log (pullHull = false, the old yaw formula): the yaw jump and where it happens
  do
    local function biggest(case)
      local out, c = replay(case, 60, { pullHull = false, pullArcLength = false }) -- (the r13c log: no r15 arc-length weight)
      local old = oldYaws(out, c)
      local best, at = 0, 0
      for i = 2, #out do
        local st = math.abs(wrap(old[i] - old[i - 1]))
        if st > best then best, at = st, out[i].w end
      end
      -- the unwrapped yaw of the controller on the same replay
      local nb = 0
      for i = 2, #out do nb = math.max(nb, math.abs(wrap(out[i].yaw - out[i - 1].yaw))) end
      return best, at, nb
    end
    local j1, a1, n1 = biggest(BEHIND[1])
    check(string.format("r14 log replay: utility4 behind - the old formula jumps %.1f deg at pull %.2f (log: frot 168.3 at pull 0.50); the new yaw %.1f deg per frame", j1, a1, n1),
      math.abs(j1 - 168.3) <= 3 and math.abs(a1 - 0.50) <= 0.04 and n1 < 6, string.format("%.1f at %.2f / %.1f", j1, a1, n1))
    local j2, a2, n2 = biggest(BEHIND[4])
    check(string.format("r14 log replay: standard3 behind - the old formula jumps %.1f deg at pull %.2f (log: frot 70.1 at pull 0.31); the new yaw %.1f deg per frame", j2, a2, n2),
      math.abs(j2 - 70.1) <= 3 and math.abs(a2 - 0.31) <= 0.04 and n2 < 6, string.format("%.1f at %.2f / %.1f", j2, a2, n2))
    local out = replay(BEHIND[1], 60, { pullHull = false, pullArcLength = false })
    local p = out[1]
    for i = 1, #out do if math.abs(out[i].w - 0.48) < math.abs(p.w - 0.48) then p = out[i] end end
    check(string.format("r14 log replay: utility4 behind at pull %.2f the camera is at veh (%.2f,%.2f,%.2f) (log: (-4.93,-0.01,1.59) at pull 0.48)", p.w, p.vx, p.vy, p.vz),
      math.abs(p.vx + 4.93) < 0.1 and math.abs(p.vy + 0.01) < 0.35 and math.abs(p.vz - 1.59) < 0.1)
  end
  -- unchanged: the entries the review called clean (door / parallel, all five classes) - no plan, no early pull-out, the same poses as without the box,
  -- the yaw of the old formula
  for _, case in ipairs(CLEAN) do
    local out, c, hull = replay(case, 60)
    local plain = replay(case, 60, { pullHull = false })
    local old = oldYaws(out, c)
    local md, myaw, pyaw, ypos = 0, 0, 0, 0
    for i = 1, #out do
      md = math.max(md, math.abs(out[i].pos.x - plain[i].pos.x), math.abs(out[i].pos.y - plain[i].pos.y), math.abs(out[i].pos.z - plain[i].pos.z))
      myaw = math.max(myaw, math.abs(wrap(out[i].yaw - old[i])))
      pyaw = math.max(pyaw, math.abs(wrap(out[i].yaw - plain[i].yaw)))
      ypos = math.max(ypos, out[i].depth)
    end
    local rt = c.pathStart and c.pathStart.route
    -- (r15: the weight follows the arc length of each path, so the <= 2 cm box push shifts the timing of the yaw against the plain path by a few
    -- thousandths of a degree; the yaw formula itself stays the old one exactly)
    check(string.format("r14 %s: no exit leg, no early pull-out flag, the poses unchanged (max %.3f m, yaw vs the old formula %.2e deg, vs the plain path %.2e deg)",
      case.name, md, myaw, pyaw), rt == nil and c.camInBox == nil and md <= 0.02 and myaw <= 1e-6 and pyaw <= 0.05,
      string.format("route=%s camInBox=%s md=%.3f yaw=%.2e plain=%.2e", tostring(rt ~= nil), tostring(c.camInBox), md, myaw, pyaw))
    local cf = newOwn({})
    cf.keep, cf.vehCar, cf.vehRecKey = "mount", true, case.rec
    local ox, oy, oz = vehToWorld(case.cam[1], case.cam[2], case.cam[3])
    local wx, wy, wz = vehToWorld(case.v[1], case.v[2], case.v[3])
    V.OwnPose(cf, { px = wx, py = wy, pz = wz, pyaw = 0, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }, { x = ox, y = oy, z = oz, yaw = 0, pitch = -14 })
    check(string.format("r14 %s: the orbit camera of the hold is not flagged for the early pull-out", case.name), cf.camInBox == nil)
  end
  -- early pull-out: the orbit camera inside the box starts the pull-out after the pivot blend (not before)
  do
    local function pullAt(clock, inBox, extra)
      local c = newOwn({})
      c.keep, c.keepSince, c.vehCar = "mount", 0, true
      c.camInBox = inBox
      c.UpdatePull(clock, extra or {}, 1 / 60)
      return c.pulling == true
    end
    check("r14 escape: orbit camera inside the box -> pull-out at 0.3 s", pullAt(0.3, 0.2))
    check("r14 escape: not before pullEscapeMin (0.1 s)", not pullAt(0.1, 0.2))
    check("r14 escape: not while the mount pivot blends", not pullAt(0.3, 0.2, { camMountBlend = true }))
    check("r14 escape: camera outside the box -> the pull-out waits for pullStart (0.9 s)", not pullAt(0.3, nil) and pullAt(0.95, nil))
    -- OwnPose raises the flag for an orbit inside the box of a car (not for a bike, not outside)
    local c = newOwn({})
    c.keep, c.vehCar, c.vehRecKey = "mount", true, "Vehicle.v_standard25_villefort_columbus"
    local f = { px = -1.65, py = -0.3, pz = -0.4, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
    V.OwnPose(c, f, { x = -0.4, y = 0.1, z = 1.2, yaw = 90, pitch = -14 })
    check("r14 escape: OwnPose flags an orbit camera inside the box", c.camInBox ~= nil and c.camInBox > 0.1, tostring(c.camInBox))
    V.OwnPose(c, f, { x = -3.0, y = 0.1, z = 1.2, yaw = 90, pitch = -14 })
    check("r14 escape: no flag for an orbit camera outside the box", c.camInBox == nil)
    c.vehCar, c.vehBike = nil, true
    V.OwnPose(c, f, { x = -0.4, y = 0.1, z = 1.2, yaw = 90, pitch = -14 })
    check("r14 escape: no flag for a motorcycle", c.camInBox == nil)
  end
  -- the log: a rendered camera inside the box is counted and warned about, a clear window says so
  do
    local hull = V.HullFor(V.DEFAULTS, "Vehicle.v_standard25_villefort_columbus")
    local st = V.NewContinuity()
    for i = 1, 30 do V.ContinuityStep(st, i / 60, { x = 0.1 + i * 0.03, y = 0.3, z = 1.2, yaw = 0, pitch = 0 }, nil, nil, hull, V.DEFAULTS.hullWarnDepth) end
    local s = V.ContinuitySummary(st, 1)
    check("r14 log: a camera inside the box is counted and warned about in the continuity summary", st.box.frames > 15
      and string.find(s, "camera inside the vehicle box WARNING", 1, true) ~= nil and string.find(s, "deepest", 1, true) ~= nil, s)
    local st2 = V.NewContinuity()
    for i = 1, 10 do V.ContinuityStep(st2, i / 60, { x = -3, y = 0.3, z = 1.4, yaw = 0, pitch = 0 }, nil, nil, hull, V.DEFAULTS.hullWarnDepth) end
    check("r14 log: a camera outside the box reads 'camera box: clear'", string.find(V.ContinuitySummary(st2, 1), "camera box: clear", 1, true) ~= nil)
    local st3 = V.NewContinuity()
    V.ContinuityStep(st3, 0.1, { x = 0, y = 0, z = 1, yaw = 0, pitch = 0 })
    check("r14 log: without a box the summary is the old one", not string.find(V.ContinuitySummary(st3, 1), "camera box", 1, true))
  end
end

do -- 2026-11-04 r15 (r13d 'behind' mounts of standard3 / utility4): the hold camera never enters the box, the
   -- pull-out peaks at <= 6 m/s on the actual path, the hand-over glide does not compose with a slowed pull-out, the WARNING line reads the frame it prints
  local VYAW, VPOS = 30, { x = 100, y = 200, z = 50 }
  local function vehToWorld(x, y, z)
    local r = math.rad(VYAW)
    return VPOS.x + math.cos(r) * x - math.sin(r) * y, VPOS.y + math.sin(r) * x + math.cos(r) * y, VPOS.z + z
  end
  local function worldToVeh(wx, wy, wz)
    local r = math.rad(-VYAW)
    local dx, dy = wx - VPOS.x, wy - VPOS.y
    return math.cos(r) * dx - math.sin(r) * dy, math.sin(r) * dx + math.cos(r) * dy, wz - VPOS.z
  end
  local R14 = { pullEscapeLead = 0, pullArcLength = false, pullMaxSpeed = 7.0, pullTimeMax = 2.2, handGlideAfterPull = false, hullPushEarly = false } -- the r13d build
  local function merged(a, b)
    local t = {}
    for k, v in pairs(a or {}) do t[k] = v end
    for k, v in pairs(b or {}) do t[k] = v end
    return t
  end
  -- the hold camera of the log ('cam veh=' lines, t from 'keepTpp off -> mount'); beyond the last line the camera goes on into the cabin (it follows the door)
  local CASES = {
    -- (climb: V's own speed 1.6 m/s for standard3, 1.3 m/s for utility4 - the camera at the clearance radius around her moves with her)
    { name = "standard3", rec = "Vehicle.v_standard3_chevalier_emperor", yaw = 89, v = { -1.98, -0.52, -0.46 }, seat = { -0.6, -0.2, 0.3 }, climb = { 0.7, 1.0 },
      keys = { { 0, -1.18, -0.22, 1.12 }, { 0.05, -1.12, -0.20, 1.13 }, { 0.10, -1.03, -0.17, 1.15 }, { 0.12, -1.00, -0.16, 1.15 }, { 0.17, -0.90, -0.14, 1.18 },
               { 0.22, -0.81, -0.11, 1.21 }, { 1.0, -0.2, 0.1, 1.5 } },
      logPull = 0.20, logDepth = 0.29 },
    { name = "utility4", rec = "Vehicle.v_utility4_kaukaz_bratsk", yaw = 93, v = { -2.69, 3.67, -0.86 }, seat = { -1.9, 3.4, 0.9 }, climb = { 0.95, 1.5 },
      keys = { { 0, -1.81, 4.06, 0.77 }, { 0.06, -1.69, 4.11, 0.79 }, { 0.11, -1.58, 4.16, 0.80 }, { 0.16, -1.47, 4.20, 0.82 }, { 0.23, -1.36, 4.25, 0.84 },
               { 1.0, -0.4, 4.5, 1.0 } },
      logPull = 0.21, logDepth = 0.15 },
  }
  local function camAt(keys, t)
    for i = 2, #keys do
      if t <= keys[i][1] or i == #keys then
        local a, b = keys[i - 1], keys[i]
        local u = (t - a[1]) / (b[1] - a[1])
        return a[2] + (b[2] - a[2]) * u, a[3] + (b[3] - a[3]) * u, a[4] + (b[4] - a[4]) * u
      end
    end
  end
  -- the mount hold frame by frame in the wiring order (Step -> UpdatePull, then OwnPose); rendered = the override of the previous frame (one frame
  -- late, the conservative reading) or the orbit camera of this frame; V climbs to the seat as in the log (climb = start, duration)
  local function hold(cs, fps, cfgOver)
    local c = newOwn(cfgOver or {})
    c.keep, c.keepSince, c.vehCar, c.vehRecKey = "mount", 0, true, cs.rec
    local hull = V.HullFor(c.cfg, cs.rec)
    local dt = 1 / fps
    local r = { deep = -9, deep0 = -9, step = 0 }
    local prevPose, prevR = nil, nil
    local i = 0
    while i * dt < 6 do
      local t = i * dt
      local k = math.max(0, math.min(1, (t - cs.climb[1]) / cs.climb[2]))
      local vx, vy, vz = cs.v[1] + (cs.seat[1] - cs.v[1]) * k, cs.v[2] + (cs.seat[2] - cs.v[2]) * k, cs.v[3] + (cs.seat[3] - cs.v[3]) * k
      local px, py, pz = vehToWorld(vx, vy, vz)
      local f = { px = px, py = py, pz = pz, pyaw = VYAW + 60, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      c.pullFrame, c.clock = f, t
      c.UpdatePull(t, {}, dt)
      if c.pulling and r.pullAt == nil then r.pullAt = t end
      local cx, cy, cz = camAt(cs.keys, t)
      local ox, oy, oz = vehToWorld(cx, cy, cz)
      local p = V.OwnPose(c, f, { x = ox, y = oy, z = oz, yaw = VYAW + cs.yaw, pitch = -14 })
      if p and r.startDepth == nil and c.pathStart then r.startDepth, r.T = V.HullDepth(hull, c.pathStart.lx, c.pathStart.ly, c.pathStart.lz), c.pathStart.dur end
      local rx, ry, rz = cx, cy, cz
      if prevPose then rx, ry, rz = worldToVeh(prevPose.pos.x, prevPose.pos.y, prevPose.pos.z) end
      local d = V.HullDepth(hull, rx, ry, rz)
      local d0 = d -- the same frame read without the one-frame delay of the override
      if p then d0 = V.HullDepth(hull, worldToVeh(p.pos.x, p.pos.y, p.pos.z)) else d0 = V.HullDepth(hull, cx, cy, cz) end
      if d0 > r.deep0 then r.deep0 = d0 end
      if d > r.deep then r.deep, r.deepAt = d, t end
      if prevR then
        local s = math.sqrt((rx - prevR[1]) ^ 2 + (ry - prevR[2]) ^ 2 + (rz - prevR[3]) ^ 2)
        if s > r.step then r.step, r.stepAt, r.stepPull = s, t, c.pull end
      end
      prevPose, prevR = p, { rx, ry, rz }
      if (c.pull or 0) >= 1 then break end
      i = i + 1
    end
    r.c = c
    local dx, dy, dz = cs.seat[1] - cs.v[1], cs.seat[2] - cs.v[2], cs.seat[3] - cs.v[3]
    r.vSpeed = math.sqrt(dx * dx + dy * dy + dz * dz) / cs.climb[2]
    return r
  end
  for _, cs in ipairs(CASES) do
    -- the r13d build on the same replay: the log's late pull-out from a start inside the box (the repro)
    local o = hold(cs, 60, R14)
    check(string.format("r15 log repro %s (r13d settings): pull-out %.2f s after the mount from a start %.2f m inside (log: %.2f s, %.2f m), the rendered camera %.2f m inside",
      cs.name, o.pullAt or -1, o.startDepth or -9, cs.logPull, cs.logDepth, o.deep), o.pullAt ~= nil and math.abs(o.pullAt - cs.logPull) <= 0.04
      and o.startDepth ~= nil and math.abs(o.startDepth - cs.logDepth) <= 0.06 and o.deep > 0.1)
    for _, fps in ipairs({ 30, 60, 144 }) do
      local r = hold(cs, fps)
      check(string.format("r15 %s @%d fps: the pull-out starts as the hold camera approaches the box (+%.3f s), the frozen start is outside (depth %.3f m)",
        cs.name, fps, r.pullAt or -1, r.startDepth or -9), r.pullAt ~= nil and r.pullAt < cs.logPull - 0.05 and r.startDepth ~= nil and r.startDepth <= 0.02)
      -- (known residual: at 30 fps the standard3 hold camera already moves into the box on the first mounted frame, where no approach can be measured yet:
      -- the frame of the freeze shows the orbit 0.01 m inside when the override is read one frame late; read in the same frame it is at the face)
      check(string.format("r15 %s @%d fps: the rendered camera never enters the box (deepest %.3f m at +%.2f s; %.3f m without the one-frame delay)", cs.name, fps,
        r.deep, r.deepAt or -1, r.deep0), r.deep0 <= 0.005 and (r.deep <= 0.005 or (fps <= 30 and r.deep <= 0.015)))
      -- 6 m/s = 0.10 m per frame at 60 fps (+1 % for the sampled arc length), plus V's own speed while she climbs past the camera at the clearance radius
      check(string.format("r15 %s @%d fps: per-frame step <= 6 m/s + V's climb (%.1f m/s) from the mount to the pull-out end (%.3f m = %.2f m/s at pull %.2f, T %.2f s)",
        cs.name, fps, r.vSpeed, r.step, r.step * fps, r.stepPull or -1, r.T or -1), r.step * fps <= 6.0 * 1.01 + r.vSpeed, string.format("%.3f m", r.step))
    end
  end
  -- the logged late-frozen starts (r13d 'pull-out path: start frozen' lines) through the pull-out alone: the r13d build reproduces the logged peaks (the
  -- clearance detour around V ran at 1.4 x the planned peak); the r15 arc-length weight holds an outside start at 6 m/s
  do
    local function pullOnly(cam, yaw, v, rec, fps, cfgOver, frozenV)
      local c = newOwn(cfgOver or {})
      c.keep, c.pulling, c.vehCar, c.vehRecKey = "mount", true, true, rec
      local ox, oy, oz = vehToWorld(cam[1], cam[2], cam[3])
      local orbit = { x = ox, y = oy, z = oz, yaw = VYAW + yaw, pitch = -14 }
      local function frame(t)
        local k = frozenV and 0 or math.max(0, math.min(1, (t - 0.75) / 1.5))
        local px, py, pz = vehToWorld(v[1] + (-1.9 - v[1]) * k, v[2] + (3.4 - v[2]) * k, v[3] + (0.9 - v[3]) * k)
        return { px = px, py = py, pz = pz, pyaw = VYAW + 60, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      end
      c.clock, c.pull = 0, 1e-6
      local f0 = frame(0)
      c.pullFrame = f0
      local p0 = V.OwnPose(c, f0, orbit)
      local T = c.pathStart.dur
      local prev, best, bestW = { worldToVeh(p0.pos.x, p0.pos.y, p0.pos.z) }, 0, 0
      for i = 1, math.ceil(T * fps) + 1 do
        local t = i / fps
        c.clock, c.pull = t, math.min(1, t / T)
        local f = frame(t)
        c.pullFrame = f
        local p = V.OwnPose(c, f, orbit)
        local x, y, z = worldToVeh(p.pos.x, p.pos.y, p.pos.z)
        local d = math.sqrt((x - prev[1]) ^ 2 + (y - prev[2]) ^ 2 + (z - prev[3]) ^ 2)
        if d > best then best, bestW = d, c.pull end
        prev = { x, y, z }
      end
      return best, bestW, T, c
    end
    local U4REC, S3REC = "Vehicle.v_utility4_kaukaz_bratsk", "Vehicle.v_standard3_chevalier_emperor"
    local b, w = pullOnly({ -1.36, 4.25, 0.84 }, 93, { -2.69, 3.67, -0.86 }, U4REC, 60, R14)
    check(string.format("r15 log repro utility4 (r13d settings, frozen 0.14 m inside): max step %.3f m at pull %.2f (log: 0.226 m = 13.5 m/s at pull 0.27)", b, w),
      math.abs(b - 0.226) <= 0.01 and math.abs(w - 0.27) <= 0.03)
    local bn = pullOnly({ -1.36, 4.25, 0.84 }, 93, { -2.69, 3.67, -0.86 }, U4REC, 60, merged(R14, { pullClearR = 0 }))
    check(string.format("r15 log repro utility4: without the clearance around V the r13d peak is the planned one (%.1f m/s, log plan 9.7 m/s)", bn * 60),
      math.abs(bn * 60 - 9.7) <= 0.3)
    local b3, w3 = pullOnly({ -0.81, -0.11, 1.21 }, 89, { -1.98, -0.52, -0.46 }, S3REC, 60, R14)
    check(string.format("r15 log repro standard3 (r13d settings, exit leg from 0.29 m inside): max step %.3f m at pull %.2f (log: 0.140 m at pull 0.29)", b3, w3),
      math.abs(b3 - 0.140) <= 0.01 and math.abs(w3 - 0.29) <= 0.04)
    -- the start the r15 hold freezes (outside, at the door): the arc-length weight and 6 m/s, V standing and V climbing as logged
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _, fz in ipairs({ true, false }) do
        local bo, _, To = pullOnly({ -1.69, 4.11, 0.79 }, 93, { -2.69, 3.67, -0.86 }, U4REC, fps, nil, fz)
        -- (V climbing at 1.3 m/s: the camera at the clearance radius around her moves with her - the excess stays below her speed)
        check(string.format("r15 utility4 outside start @%d fps (%s): peak %.2f m/s <= 6 m/s%s on the actual path (T %.2f s <= pullTimeMax)", fps,
          fz and "V standing" or "V climbing", bo * fps, fz and "" or " + V's 1.3 m/s", To), bo * fps <= 6.0 * 1.01 + (fz and 0 or 1.3) and To <= V.DEFAULTS.pullTimeMax + 1e-9)
      end
    end
    local bm = pullOnly({ -1.69, 4.11, 0.79 }, 93, { -2.69, 3.67, -0.86 }, U4REC, 60, { pullArcLength = false }, true)
    check(string.format("r15 mutation: without the arc-length weight the same outside start overshoots 6 m/s in the detour around V (%.2f m/s)", bm * 60),
      bm * 60 > 7.0)
    -- the arc-length table: monotone, the weight reaches each fraction of the length
    local _, _, _, ca = pullOnly({ -1.69, 4.11, 0.79 }, 93, { -2.69, 3.67, -0.86 }, U4REC, 60, nil, true)
    local arc = ca.pathStart.arc
    local mono = arc ~= nil
    for i = 2, (arc and arc.n + 1 or 0) do if arc.cum[i] < arc.cum[i - 1] then mono = false end end
    check("r15 arc table: cumulative length monotone, PullArcW(0) = 0, PullArcW(1) = 1, PullArcW(0.5) inside (0, 1)", mono and V.PullArcW(arc, 0) == 0
      and V.PullArcW(arc, 1) == 1 and V.PullArcW(arc, 0.5) > 0 and V.PullArcW(arc, 0.5) < 1)
    -- an inside start (the fallback when the early pull-out cannot fire: the pivot blend runs) keeps the r14 weight (no table)
    local _, _, _, ci = pullOnly({ -1.36, 4.25, 0.84 }, 93, { -2.69, 3.67, -0.86 }, U4REC, 60, nil, true)
    check("r15 arc table: a start inside the box keeps the r14 path weight (no table)", ci.pathStart.arc == nil and (ci.pathStart.clampFrom or 0) > 0)
  end
  -- the early pull-out flag: an orbit camera that moves away from or along the box is not flagged; one approaching is flagged within lead frames
  do
    local function seq(rec, p0, dir, speed, n, fps, cfgOver)
      local c = newOwn(cfgOver or {})
      c.keep, c.keepSince, c.vehCar, c.vehRecKey = "mount", 0, true, rec
      local f = { px = VPOS.x - 3, py = VPOS.y, pz = VPOS.z, pyaw = 0, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      local firstAt, depthAt = nil, nil
      local hull = V.HullFor(c.cfg, rec)
      for i = 0, n do
        local t = i / fps
        c.clock = t
        local x, y, z = p0[1] + dir[1] * speed * t, p0[2] + dir[2] * speed * t, p0[3] + dir[3] * speed * t
        local ox, oy, oz = vehToWorld(x, y, z)
        V.OwnPose(c, f, { x = ox, y = oy, z = oz, yaw = 0, pitch = -14 })
        if c.camBoxNear and not firstAt then firstAt, depthAt = t, V.HullDepth(hull, x, y, z) end
      end
      return firstAt, depthAt
    end
    local S3REC = "Vehicle.v_standard3_chevalier_emperor"
    local a1 = seq(S3REC, { -1.6, -0.3, 1.1 }, { -1, 0, 0 }, 2.4, 30, 60)
    local a2 = seq(S3REC, { -1.3, -0.3, 1.1 }, { 0, -1, 0 }, 2.4, 30, 60)
    local a3 = seq(S3REC, { -2.5, -0.3, 1.1 }, { 1, 0, 0 }, 2.4, 60, 60)
    check("r15 escape flag: an orbit camera moving away from the box or along its side is not flagged", a1 == nil and a2 == nil)
    for _, fps in ipairs({ 30, 60, 144 }) do
      local _, d = seq(S3REC, { -2.5, -0.3, 1.1 }, { 1, 0, 0 }, 2.4, fps, fps)
      -- (lead 3 / 60 s at 2.4 m/s + 0.03 m = 0.15 m from the face at any frame rate; the flag comes on the first frame inside that distance,
      -- at most one frame of approach (2.4 m/s / fps) later)
      check(string.format("r15 escape flag @%d fps: an approaching camera is flagged outside the box, within lead / 60 s + 0.03 m of its face (depth %.3f m)", fps, d or 9),
        d ~= nil and d < 0 and d >= -(3 * 2.4 / 60 + 0.03) - 1e-6 and d <= -(3 * 2.4 / 60 + 0.03) + 2.4 / fps + 1e-6)
    end
    -- uneven frame times (45 Hz +-30 %, fixed-seed Lehmer generator): the same look-ahead distance, within the longest frame of approach
    do
      local c = newOwn({})
      c.keep, c.keepSince, c.vehCar, c.vehRecKey = "mount", 0, true, S3REC
      local f = { px = VPOS.x - 3, py = VPOS.y, pz = VPOS.z, pyaw = 0, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      local hull = V.HullFor(c.cfg, S3REC)
      local t, seed, maxDt, d = 0, 12345, 0, nil
      while t < 1.0 and d == nil do
        local x, y, z = -2.5 + 2.4 * t, -0.3, 1.1
        c.clock = t
        local ox, oy, oz = vehToWorld(x, y, z)
        V.OwnPose(c, f, { x = ox, y = oy, z = oz, yaw = 0, pitch = -14 })
        if c.camBoxNear then d = V.HullDepth(hull, x, y, z) end
        seed = (seed * 16807) % 2147483647
        local dt = (1 / 45) * (1 + 0.3 * (2 * seed / 2147483647 - 1))
        maxDt = math.max(maxDt, dt)
        t = t + dt
      end
      check(string.format("r15 escape flag @45 Hz +-30 %% jitter: flagged outside the box within lead / 60 s + 0.03 m of its face (depth %.3f m)", d or 9),
        d ~= nil and d < 0 and d >= -(3 * 2.4 / 60 + 0.03) - 1e-6 and d <= -(3 * 2.4 / 60 + 0.03) + 2.4 * maxDt + 1e-6)
    end
    local off = seq(S3REC, { -2.5, -0.3, 1.1 }, { 1, 0, 0 }, 2.4, 60, 60, { pullEscapeLead = 0 })
    check("r15 escape flag mutation: pullEscapeLead = 0 never flags (r14)", off == nil and a3 ~= nil)
    -- UpdatePull: the approach flag starts the pull-out at once (no pullEscapeMin wait), not during the mount pivot blend
    local function pullAt(clock, near, extra)
      local c = newOwn({})
      c.keep, c.keepSince, c.vehCar = "mount", 0, true
      c.camBoxNear = near
      c.UpdatePull(clock, extra or {}, 1 / 60)
      return c.pulling == true
    end
    check("r15 escape: an approaching orbit camera starts the pull-out at 0.05 s (no pullEscapeMin wait)", pullAt(0.05, { d = -0.05, pred = 0.02 }))
    check("r15 escape: not while the mount pivot blends", not pullAt(0.05, { d = -0.05, pred = 0.02 }, { camMountBlend = true }))
    check("r15 escape: no flag -> the pull-out waits as before", not pullAt(0.05, nil))
  end
  -- the hand-over glide waits for a slowed pull-out (it does not compose with a path still running at 6 m/s), at the latest so that it starts before
  -- keepTimeout; the pull-out alone ends before keepTimeout
  do
    local function glide(cfgOver, pull, clock, keepSince)
      local c = newOwn(cfgOver or {})
      c.vehCar, c.vehRecKey = true, "Vehicle.v_test_car"
      c.pullFrame = { px = -1, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
      c.ownLast = { x = 0, y = -5.5, z = 2.4, yaw = 0, pitch = -14 }
      c.keep, c.keepSince, c.phase, c.phaseEndAt, c.pulling, c.pull = "mount", keepSince or 0, "done", 0, true, pull
      c.pathStart = { dur = 3.7 }
      c.TryHandGlide(clock, { activePersp = 3 })
      return c.handMatch ~= nil
    end
    check("r15 glide: a slowed pull-out (3.7 s) at pull 0.5 holds the glide after HandoverDelay", not glide(nil, 0.5, 1.0))
    check("r15 glide: it starts at the pull-out end", glide(nil, 1, 1.0))
    check("r15 glide: it starts by keepTimeout - handGlideTimeMax - 0.3 s even before the pull-out end (4.0 s)", glide(nil, 0.9, 4.05, 0) and not glide(nil, 0.9, 3.95, 0))
    check("r15 glide mutation: handGlideAfterPull = false composes with the tail again (r14)", glide({ handGlideAfterPull = false }, 0.5, 1.0))
    local P = V.DEFAULTS
    check("r15 timeline: pullStart + pullTimeMax < keepTimeout (the hand-over without a glide waits for the pull-out end inside the hold)",
      P.pullStart + P.pullTimeMax < P.keepTimeout)
    -- composition on the truck path: a glide that starts mid-way (r14) against the glide after the pull-out end (r15), 60 fps
    local function composed(glideAt)
      local c = newOwn({})
      c.keep, c.pulling, c.vehCar, c.vehRecKey = "mount", true, true, "Vehicle.v_utility4_kaukaz_bratsk"
      local ox, oy, oz = vehToWorld(-1.69, 4.11, 0.79)
      local orbit = { x = ox, y = oy, z = oz, yaw = VYAW + 93, pitch = -14 }
      local px, py, pz = vehToWorld(-2.69, 3.67, -0.86)
      local f = { px = px, py = py, pz = pz, pyaw = VYAW + 60, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      c.clock, c.pull = 0, 1e-6
      local p0 = V.OwnPose(c, f, orbit)
      local T = c.pathStart.dur
      local ga = glideAt or T
      local prev, best = { worldToVeh(p0.pos.x, p0.pos.y, p0.pos.z) }, 0
      for i = 1, math.ceil((ga + 1.7) * 60) do
        local t = i / 60
        c.clock, c.pull = t, math.min(1, t / T)
        if t >= ga and c.handMatch == nil then c.handMatch = { x = 0, y = -11, z = 4, yaw = 0, pitch = 0, at = t, dur = 1.7 } end
        local p = V.OwnPose(c, f, orbit)
        local x, y, z = worldToVeh(p.pos.x, p.pos.y, p.pos.z)
        best = math.max(best, math.sqrt((x - prev[1]) ^ 2 + (y - prev[2]) ^ 2 + (z - prev[3]) ^ 2))
        prev = { x, y, z }
      end
      return best
    end
    local after, mid = composed(nil), composed(1.5)
    check(string.format("r15 glide composition: after the pull-out end <= 0.10 m per frame (%.3f m); started mid-way it exceeds 0.15 m (%.3f m)", after, mid),
      after <= 0.101 and mid > 0.15)
  end
  -- the WARNING line of the wiring: the standard3 sequence of the log (inside 518.29 - 518.62, outside after; the r14 line at 518.81 read -0.59 m)
  do
    local hull = V.HullFor(V.DEFAULTS, "Vehicle.v_standard3_chevalier_emperor")
    local w, lines = {}, {}
    for i = 0, 40 do
      local t = 518.29 + i / 60
      local d
      if t <= 518.625 then d = 0.07 + 0.2 * math.sin(math.min(1, (t - 518.29) / 0.33) * math.pi) else d = -0.1 - (t - 518.625) * 3 end
      if V.ContinuityBoxWarnDue(w, d, t, V.DEFAULTS.hullWarnDepth) then lines[#lines + 1] = { t = t, d = d } end
    end
    local allInside = #lines > 0
    for _, l in ipairs(lines) do if l.d <= V.DEFAULTS.hullWarnDepth then allInside = false end end
    check(string.format("r15 WARNING: the logged sequence gives %d line(s), each on a frame inside the box (r14 printed the outside frame 518.81 as -0.59 m)", #lines),
      #lines == 1 and allInside and math.abs(lines[1].t - 518.29) < 1e-6)
    local w2, n2 = {}, 0
    for i = 0, 72 do if V.ContinuityBoxWarnDue(w2, 0.2, i / 60, 0.05) then n2 = n2 + 1 end end
    check(string.format("r15 WARNING: a camera inside for 1.2 s gives one line per 0.5 s (%d)", n2), n2 == 3)
    check("r15 WARNING: the logged poses against the box: (-1.69,-1.78,1.49) outside, (-1.03,-0.17,1.15) 0.07 m inside",
      V.HullDepth(hull, -1.69, -1.78, 1.49) < 0 and math.abs(V.HullDepth(hull, -1.03, -0.17, 1.15) - 0.07) < 1e-6)
  end
end

do -- 2026-11-05 r16 (r13f sport1 'behind' mount, t=900.86 - 901.78): the walking
   -- camera rested over the rear deck (the camera collision had shortened the boom to 2.17 m) and the flat box of the record (top 1.05 m over the whole car)
   -- read it 0.12 m inside on 4 frames (WARNING at 900.86; continuity: 4 of 284 frames, t=900.86 - 900.91) while the video (normal_cycle_behind-x-13.mp4)
   -- shows it clear of the body. The top profile of the record (topY, from the extracted body shell) is low over the rear deck and the bonnet and keeps
   -- the cabin height: the logged hold is outside the box, the cabin is not. Mutation: hullProfile = false (the flat box) reproduces the log.
  local REC = "Vehicle.v_sport1_quadra_turbo_r"
  local FLAT = { hullProfile = false }
  local VYAW, VPOS = 30, { x = 100, y = 200, z = 50 }
  local function vehToWorld(x, y, z)
    local r = math.rad(VYAW)
    return VPOS.x + math.cos(r) * x - math.sin(r) * y, VPOS.y + math.sin(r) * x + math.cos(r) * y, VPOS.z + z
  end
  local function worldToVeh(wx, wy, wz)
    local r = math.rad(-VYAW)
    local dx, dy = wx - VPOS.x, wy - VPOS.y
    return math.cos(r) * dx - math.sin(r) * dy, math.sin(r) * dx + math.cos(r) * dy, wz - VPOS.z
  end
  -- the 'cam veh=' lines of the hold (s after 'keepTpp off -> mount' at 900.86; camera x, y, z and V = vtpp x, y, z, vehicle frame, yaw 0 relative to
  -- the car). The car settled about 1 m right after the mount (POS v z 181.02 -> 180.02): the camera rises in the vehicle frame, replayed as logged.
  local KEYS = {
    { 0.00, -0.90, -2.18, 0.76, -1.63, -0.16, -1.04 }, { 0.05, -0.88, -2.23, 0.79, -1.63, -0.16, -0.99 }, { 0.10, -0.84, -2.34, 0.86, -1.63, -0.16, -0.92 },
    { 0.17, -0.78, -2.48, 1.01, -1.63, -0.16, -0.83 }, { 0.22, -0.74, -2.59, 1.14, -1.62, -0.16, -0.77 }, { 0.27, -0.69, -2.70, 1.29, -1.62, -0.16, -0.72 },
    { 0.34, -0.62, -2.83, 1.35, -1.59, -0.20, -0.71 }, { 0.39, -0.56, -2.93, 1.28, -1.56, -0.26, -0.69 }, { 0.44, -0.59, -2.90, 1.29, -1.58, -0.28, -0.60 },
    { 0.49, -0.65, -2.85, 1.31, -1.61, -0.26, -0.47 }, { 0.55, -0.64, -2.92, 1.35, -1.62, -0.24, -0.37 }, { 0.60, -0.62, -2.95, 1.36, -1.62, -0.23, -0.35 },
    { 0.65, -0.60, -2.97, 1.36, -1.61, -0.23, -0.36 }, { 0.70, -0.59, -2.98, 1.36, -1.60, -0.22, -0.37 }, { 0.75, -0.58, -2.97, 1.38, -1.60, -0.22, -0.39 },
    { 0.80, -0.57, -2.97, 1.41, -1.59, -0.21, -0.41 }, { 0.85, -0.56, -2.96, 1.45, -1.59, -0.21, -0.43 }, { 0.90, -0.55, -2.96, 1.49, -1.58, -0.21, -0.45 },
    { 0.95, -0.54, -2.97, 1.51, -1.56, -0.22, -0.46 },
  }
  local function at(t)
    for i = 2, #KEYS do
      if t <= KEYS[i][1] or i == #KEYS then
        local a, b = KEYS[i - 1], KEYS[i]
        local u = math.max(0, math.min(1, (t - a[1]) / (b[1] - a[1])))
        local r = {}
        for j = 2, 7 do r[j - 1] = a[j] + (b[j] - a[j]) * u end
        return r
      end
    end
  end
  local hullP, hullF = V.HullFor(V.DEFAULTS, REC), V.HullFor(V.NewController(FLAT).cfg, REC)
  -- (1) the logged poses against the box
  local worst, worstAt = -9, nil
  for _, k in ipairs(KEYS) do
    local d = V.HullDepth(hullP, k[2], k[3], k[4])
    if d > worst then worst, worstAt = d, k[1] end
  end
  local a = math.rad(-101) -- the frozen start of the log ('start frozen at -101 deg, 2.99 m from the vehicle, 1.49 m up')
  local dStart = V.HullDepth(hullP, 2.99 * math.cos(a), 2.99 * math.sin(a), 1.49)
  check(string.format("r16 sport1 behind: every logged hold pose and the frozen start are outside the profiled box (deepest %.3f m at +%.2f s, start %.2f m)",
    worst, worstAt or -1, dStart), worst <= 0 and dStart <= 0)
  local f0, f1 = V.HullDepth(hullF, -0.90, -2.18, 0.76), V.HullDepth(hullF, -0.88, -2.23, 0.79)
  check(string.format("r16 log repro (hullProfile = false, the flat box): the first two poses read %.3f / %.3f m inside (log: 0.12 m WARNING, 4 frames)", f0, f1),
    math.abs(f0 - 0.12) <= 0.005 and f1 > 0.05)
  -- (2) the hold frame by frame (Step -> UpdatePull, then OwnPose; rendered = the override of the previous frame or the orbit), the continuity box count
  -- and the WARNING lines of the wiring
  local function hold(fps, cfgOver)
    local c = newOwn(cfgOver or {})
    c.keep, c.keepSince, c.vehCar, c.vehRecKey = "mount", 0, true, REC
    local hull = V.HullFor(c.cfg, REC)
    local dt = 1 / fps
    local r = { deep = -9, deep0 = -9, warn = 0, inBoxFlag = false, nearFlag = false, logs = {} }
    c.Log = function(s) r.logs[#r.logs + 1] = s end
    local st, w = V.NewContinuity(), {}
    local prevPose
    local i = 0
    while i * dt < 4 do
      local t = i * dt
      local k = at(t)
      local px, py, pz = vehToWorld(k[4], k[5], k[6])
      local fr = { px = px, py = py, pz = pz, pyaw = VYAW + 60, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }
      c.pullFrame, c.clock = fr, t
      c.UpdatePull(t, {}, dt)
      if c.pulling and r.pullAt == nil then r.pullAt = t end
      local ox, oy, oz = vehToWorld(k[1], k[2], k[3])
      local p = V.OwnPose(c, fr, { x = ox, y = oy, z = oz, yaw = VYAW, pitch = -14 })
      if c.camInBox ~= nil then r.inBoxFlag = true end
      if c.camBoxNear ~= nil then r.nearFlag = true end
      local rx, ry, rz = k[1], k[2], k[3]
      if prevPose then rx, ry, rz = worldToVeh(prevPose.pos.x, prevPose.pos.y, prevPose.pos.z) end
      local d = V.HullDepth(hull, rx, ry, rz)
      local d0 = p and V.HullDepth(hull, worldToVeh(p.pos.x, p.pos.y, p.pos.z)) or V.HullDepth(hull, k[1], k[2], k[3])
      if d0 > r.deep0 then r.deep0 = d0 end
      if d > r.deep then r.deep, r.deepAt = d, t end
      V.ContinuityStep(st, t, { x = rx, y = ry, z = rz, yaw = 0, pitch = -14 }, nil, nil, hull, V.DEFAULTS.hullWarnDepth)
      if V.ContinuityBoxWarnDue(w, d, t, V.DEFAULTS.hullWarnDepth) then r.warn = r.warn + 1 end
      prevPose = p
      if (c.pull or 0) >= 1 then break end
      i = i + 1
    end
    r.box, r.boxDeep, r.c = st.box.frames, st.box.depth, c
    return r
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local r, m = hold(fps), hold(fps, FLAT)
    check(string.format("r16 sport1 behind @%d fps: the rendered camera never enters the profiled box through the hold and the pull-out (deepest %.3f m at +%.2f s; "
      .. "%.3f m without the one-frame delay), 0 continuity box frames (%d), no WARNING (%d), no early pull-out flag", fps, r.deep, r.deepAt or -1, r.deep0, r.box, r.warn),
      r.deep <= 0 and r.deep0 <= 0 and r.box == 0 and r.warn == 0 and not r.inBoxFlag and not r.nearFlag)
    check(string.format("r16 sport1 behind @%d fps: the pull-out starts at pullStart as logged (+%.2f s, flat box +%.2f s; log +0.92 s) and reaches the ride camera",
      fps, r.pullAt or -1, m.pullAt or -1), r.pullAt ~= nil and m.pullAt ~= nil and math.abs(r.pullAt - 0.92) <= 0.04 and math.abs(m.pullAt - r.pullAt) <= 1e-9
      and (r.c.pull or 0) >= 1)
    local pl = nil
    for _, s in ipairs(r.logs) do if string.find(s, "pull-out path: box", 1, true) then pl = s end end
    check(string.format("r16 sport1 behind @%d fps: the pull-out log line reads the start outside and names the profile (%s)", fps, tostring(pl)),
      pl ~= nil and string.find(pl, "start outside the box", 1, true) ~= nil and string.find(pl, ", top profile 8 knots (", 1, true) ~= nil)
    -- the flat box: the WARNING of the log (one line, 0.12 m) and the frames the continuity summary counted (log: 4 at about 60 fps)
    check(string.format("r16 log repro @%d fps (hullProfile = false): the flat box counts %d frame(s) inside (deepest %.3f m), %d WARNING line(s)", fps, m.box, m.boxDeep, m.warn),
      m.box >= (fps >= 60 and 3 or 2) and math.abs(m.boxDeep - 0.12) <= 0.005 and m.warn == 1)
  end
  -- (3) negative: the cabin stays inside the box (the profile is the flat top over it), the roof and the body shell are covered
  local CABIN = { { -0.36, -0.41, 0.03, "the log's vehicleTPPCamera at the driver seat" }, { 0.0, -0.4, 0.95, "under the roof, centre" },
                  { -0.5, -0.9, 0.98, "under the roof, rear of the cabin" }, { 0.4, 0.0, 0.85, "under the roof at the windscreen" },
                  { -0.6, -0.5, 0.6, "head height of the driver" } }
  for _, q in ipairs(CABIN) do
    local d, df = V.HullDepth(hullP, q[1], q[2], q[3]), V.HullDepth(hullF, q[1], q[2], q[3])
    check(string.format("r16 sport1 negative: %s (%.2f,%.2f,%.2f) is inside the profiled box as deep as in the flat one (%.3f / %.3f m)", q[4], q[1], q[2], q[3], d, df),
      d > V.DEFAULTS.hullWarnDepth and math.abs(d - df) < 1e-9)
  end
  check("r16 sport1 negative: the roof top is 1.05 m from y -1.15 to 0.05 (the flat top), above it the camera is outside",
    V.HullTop(hullP, -1.15) == 1.05 and V.HullTop(hullP, -0.5) == 1.05 and V.HullTop(hullP, 0.05) == 1.05 and V.HullDepth(hullP, 0, -0.5, 1.1) < 0)
  do -- the orbit camera of a mount hold in the cabin is still flagged for the early pull-out
    local c = newOwn({})
    c.keep, c.vehCar, c.vehRecKey = "mount", true, REC
    local ox, oy, oz = vehToWorld(-0.4, -0.4, 0.8)
    local px, py, pz = vehToWorld(-1.6, -0.1, -0.3)
    V.OwnPose(c, { px = px, py = py, pz = pz, pyaw = 0, vx = VPOS.x, vy = VPOS.y, vz = VPOS.z, vyaw = VYAW }, { x = ox, y = oy, z = oz, yaw = VYAW, pitch = -14 })
    check(string.format("r16 sport1 negative: OwnPose flags an orbit camera in the cabin (%.2f m inside)", c.camInBox or -1), c.camInBox ~= nil and c.camInBox > 0.1)
  end
  -- the body shell of the record (highest point of each 0.2 m slice along the car, rounded up to 1 cm: Extracted/game/vehicle-entry-car glb-body
  -- ext01_body_01 + the four glass meshes, surface points every 0.03 m as extract_door_data.py --body samples them but uncropped; Base-bone frame =
  -- the vehicle frame) lies at least 0.1 m below the top of the box over the whole length of the box
  local SHELL = { { -2.41, -2.2, 0.48 }, { -2.2, -2.0, 0.53 }, { -2.0, -1.8, 0.58 }, { -1.8, -1.6, 0.62 }, { -1.6, -1.4, 0.65 }, { -1.4, -1.2, 0.71 },
                  { -1.2, -1.0, 0.74 }, { -1.0, -0.8, 0.93 }, { -0.8, -0.6, 0.89 }, { -0.6, -0.4, 0.82 }, { -0.4, -0.2, 0.83 }, { -0.2, 0.0, 0.82 },
                  { 0.0, 0.2, 0.78 }, { 0.2, 0.4, 0.71 }, { 0.4, 0.6, 0.64 }, { 0.6, 0.8, 0.56 }, { 0.8, 1.0, 0.51 }, { 1.0, 1.2, 0.50 }, { 1.2, 1.4, 0.48 },
                  { 1.4, 1.6, 0.46 }, { 1.6, 1.8, 0.41 }, { 1.8, 2.0, 0.36 }, { 2.0, 2.13, 0.27 } }
  local gap, gapAt = 9, nil
  for _, s in ipairs(SHELL) do
    for j = 0, 20 do
      local y = s[1] + (s[2] - s[1]) * j / 20
      if y >= -hullP.hyB and y <= hullP.hyF then
        local g = V.HullTop(hullP, y) - s[3]
        if g < gap then gap, gapAt = g, y end
      end
    end
  end
  check(string.format("r16 sport1: the top profile is at least 0.10 m above the body shell over the length of the box (smallest gap %.3f m at y %.2f)", gap, gapAt or 0),
    gap >= 0.10 - 1e-9)
  -- (4) the profile helpers: other records and hand-made boxes keep the flat top, a malformed profile is ignored, the "_player" record reads the same
  local s3 = V.HullFor(V.DEFAULTS, "Vehicle.v_standard3_chevalier_emperor")
  local hand = { hx = 1, hyF = 2, hyB = 2, top = 1.3, bottom = -1 }
  local bad = V.HullFor({ hullRecords = { x = { hx = 1, hyF = 2, hyB = 2, top = 1.2, topY = { { 0, 1 }, { -1, 0.5 } } } } }, "x")
  local pl = V.HullFor(V.DEFAULTS, REC .. "_player")
  check("r16 profile helpers: standard3 / the default / a hand-made box have no profile (flat top), a profile out of order is ignored, the _player record has it",
    s3.topY == nil and V.HullFor(V.DEFAULTS, "Vehicle.v_unknown").topY == nil and V.HullTop(hand, -1.9) == 1.3 and bad.topY == nil and V.HullTop(bad, -1) == 1.2
    and pl.topY ~= nil and V.HullTop(pl, -2.18) == V.HullTop(hullP, -2.18) and hullF.topY == nil)
  check(string.format("r16 profile: the ends hold their values beyond the knots, linear between (top at y -2.18: %.3f, at -2.5: %.3f, at 2.5: %.3f)",
    V.HullTop(hullP, -2.18), V.HullTop(hullP, -2.5), V.HullTop(hullP, 2.5)),
    math.abs(V.HullTop(hullP, -2.18) - (0.68 + 0.04 * 0.12 / 0.3)) < 1e-9 and V.HullTop(hullP, -2.5) == 0.68 and V.HullTop(hullP, 2.5) == 0.62)
  -- the box push: a point over the rear deck between the profile and the flat top is left where it is, one just under the profile goes to it
  local fId = { px = 0, py = 0, pz = 0, pyaw = 0, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  local over = V.PullHullPush(hullP, { x = -0.5, y = -2.0, z = 0.9 }, fId, 0.05)
  local under = V.PullHullPush(hullP, { x = -0.2, y = -1.6, z = V.HullTop(hullP, -1.6) - 0.02 }, fId, 0.05)
  check(string.format("r16 box push: over the rear deck (z 0.90 > the profile %.2f) not moved; 0.02 m under the profile at y -1.6 lifted to the profile + margin (%.3f)",
    V.HullTop(hullP, -2.0), under.z), over.z == 0.9 and over.y == -2.0 and math.abs(under.z - (V.HullTop(hullP, -1.6) + 0.05)) < 1e-9)
end

print(string.format("utpp_vehiclecam offline: %d passed, %d failed", pass, fail))
assert(fail == 0, "utpp_vehiclecam offline checks failed")

-- UTPP offline check of the pure parts of utpp_vehiclecam: pull-out pose math (PullPose, the path around the vehicle) and the
-- persisted settings (keepTpp / pullOut). The controller scenarios live in test_utpp_vehiclecam.lua, the camera integration
-- in test_utpp_camera_vehicle.lua. Needs UTPP_VEHICLECAM_PATH.
local V = dofile(UTPP_VEHICLECAM_PATH)
assert(type(V.NewController) == "function")

local pass, fail = 0, 0
local function check(name, cond, detail)
  if cond then pass = pass + 1 else fail = fail + 1; print("FAIL " .. name .. (detail and (" " .. detail) or "")) end
end

-- box around the vehicle (half width 1.0, half length 2.4, height 1.9)
local function boxDist(x, y, z)
  local dx, dy, dz = math.max(math.abs(x) - 1.0, 0), math.max(math.abs(y) - 2.4, 0), math.max(z - 1.9, 0)
  return math.sqrt(dx * dx + dy * dy + dz * dz)
end

do -- PullPose math: vehicle at the origin facing +Y, player 0.5 m right of it facing +Y
  local cfg = { pullBack = 5.5, pullHeight = 2.4, pullLookHeight = 1.0, pullPitchSign = 1.0 }
  local pos, yaw, pitch = V.PullPose(0.5, 0, 0, 0, 0, 0, 0, 0, cfg)
  check("PullPose: behind and above the car in the player frame",
        math.abs(pos.x + 0.5) < 1e-6 and math.abs(pos.y + 5.5) < 1e-6 and math.abs(pos.z - 2.4) < 1e-6)
  check("PullPose: looking forward and down", math.abs(yaw) < 1e-6 and math.abs(pitch - math.deg(math.atan2(-1.4, 5.5))) < 1e-4,
        string.format("%.2f %.2f", yaw, pitch))
  local pos2, yaw2 = V.PullPose(0, 0, 0, 90, 0, 0, 0, 0, cfg) -- player faces -X: the point behind the car (0,-5.5) is on its left
  check("PullPose: player yaw handled (local frame rotates)", math.abs(pos2.x + 5.5) < 1e-5 and math.abs(pos2.y) < 1e-5 and
        math.abs(yaw2 + 90) < 1e-5, string.format("%.2f %.2f %.2f", pos2.x, pos2.y, yaw2))
end

do -- path math: frozen start, shorter arc, w = 1 equals PullPose, look-at towards the vehicle
  check("path math: M.VehCyl / M.PullPathPose available", type(V.VehCyl) == "function" and type(V.PullPathPose) == "function")
  local f = { px = -1, py = 0.3, pz = 0, pyaw = -90, vx = 0, vy = 0, vz = 0, vyaw = 0 }
  local s0 = V.VehCyl(0.4, -2.5, 1.7, f) -- 2.5 m behind V who faces the car from its left side
  check("VehCyl: left of the vehicle", math.abs(math.deg(s0.a)) > 170 and math.abs(s0.r - 3.5) < 0.01 and math.abs(s0.h - 1.7) < 1e-6,
    string.format("%.1f deg %.2f m", math.deg(s0.a), s0.r))
  local p1, y1, pi1 = V.PullPathPose(s0, 1, f, V.DEFAULTS)
  local q1, yq, pq = V.PullPose(-1, 0.3, 0, -90, 0, 0, 0, 0, V.DEFAULTS)
  check("PullPathPose: w = 1 equals PullPose", math.abs(p1.x - q1.x) < 1e-6 and math.abs(p1.y - q1.y) < 1e-6
    and math.abs(p1.z - q1.z) < 1e-6 and math.abs(y1 - yq) < 1e-6 and math.abs(pi1 - pq) < 1e-6)
  local p0 = V.PullPathPose(s0, 0, f, V.DEFAULTS)
  check("PullPathPose: w = 0 is the frozen start", math.abs(p0.x - 0.4) < 1e-6 and math.abs(p0.y + 2.5) < 1e-6)
  local ph = V.PullPathPose(s0, 0.5, f, V.DEFAULTS)
  local wx, wy = -1 + ph.y, 0.3 - ph.x -- player yaw -90: local (x, y) -> world (y, -x)
  check("PullPathPose: half way it is behind-left of the vehicle, outside the body",
    wx < -1.5 and wy < -2.0 and boxDist(wx, wy, ph.z) > 1.0, string.format("%.2f %.2f", wx, wy))
end

do -- settings persistence round trip (temp file)
  local path = os.tmpname()
  V.ctrl.cfg.keepTpp = false
  check("settings: save", V.SaveSettings(path))
  V.ctrl.cfg.keepTpp = true
  check("settings: load restores false", V.LoadSettings(path) and V.ctrl.cfg.keepTpp == false)
  V.ctrl.cfg.keepTpp = true
  os.remove(path)
  check("settings: missing file keeps the default (true)", V.LoadSettings(path) == false and V.ctrl.cfg.keepTpp == true)
  -- vehicle batch: vehicle.forceTppOnEnter in the fallback file (without utpp_core); default on
  check("settings: forceTppOnEnter default on", V.ctrl.cfg.forceTppOnEnter == true)
  V.ctrl.cfg.forceTppOnEnter = false
  V.SaveSettings(path)
  V.ctrl.cfg.forceTppOnEnter = true
  check("settings: forceTppOnEnter=false saved and loaded", V.LoadSettings(path) and V.ctrl.cfg.forceTppOnEnter == false)
  V.ctrl.cfg.forceTppOnEnter = true
  os.remove(path)
end

do -- MOTO-1 A (3): the seated vehicle TPP perspective is requested during the exit (first exit frame, every 0.10 s, max 8)
  local function exitRun(seated, cfg, tail, setup)
    local c = V.NewController(cfg or {})
    if setup then setup(c) end
    c.own = true
    c.ownCamReady = function() return true end
    local t, reqs, lines = 0, {}, {}
    c.trace = function(m) lines[#lines + 1] = m end
    local function fr(n, obs)
      for _ = 1, n do
        t = t + 1 / 60
        local p = c.Step(t, obs)
        if p and c.lastReqKind == "exitKeep" then reqs[#reqs + 1] = { t = t, p = p, vs = obs.vehicleState, mounted = obs.mounted } end
        c.TakeCamCmds()
      end
    end
    fr(30, { vehicleState = 0, mounted = false })
    fr(60, { vehicleState = 4, mounted = true, driver = true, activePersp = 0, persistentPersp = 2 })
    fr(120, { vehicleState = 1, mounted = true, driver = true, activePersp = seated, persistentPersp = 2 })
    local exitAt = t + 1 / 60
    for _, s in ipairs(tail or { { 120, { vehicleState = 4, mounted = true, driver = true, activePersp = 0 } },
                                 { 60, { vehicleState = 0, mounted = false } } }) do fr(s[1], s[2]) end
    return reqs, exitAt, lines, c
  end
  local reqs, exitAt, lines = exitRun(2)
  local spacing = true
  for i = 2, #reqs do if reqs[i].t - reqs[i - 1].t < 0.10 - 1e-6 then spacing = false end end
  local after = 0
  for _, q in ipairs(reqs) do if not q.mounted or q.vs ~= 4 then after = after + 1 end end
  local logged = false
  for _, l in ipairs(lines) do if string.find(l, "exit keep vehicle TPP: request TPPMedium, active=0", 1, true) then logged = true end end
  local result = false
  for _, l in ipairs(lines) do if string.find(l, "exit keep result: active stayed TPP=false", 1, true) then result = true end end
  check("exit keep: TPPMedium on the exit frame, every >= 0.10 s, 8 at most, none after the unmount, logged",
    #reqs == 8 and reqs[1].p == 2 and math.abs(reqs[1].t - exitAt) < 1e-6 and spacing and after == 0 and logged and result,
    string.format("%d requests, first %s", #reqs, tostring(reqs[1] and reqs[1].t - exitAt)))
  local r0 = exitRun(0)
  check("exit keep: none after a ride in vehicle FPP (seated active 0)", #r0 == 0)
  local r5 = exitRun(5)
  check("exit keep: DriverCombatMedium seated -> TPPMedium", #r5 > 0 and r5[1].p == 2)
  local rOff = exitRun(2, { exitKeepVehicleTpp = false })
  check("exit keep: A/B off -> no request", #rOff == 0)
  -- unmount transition with the vehicle state already 0 while still mounted: no request
  local rU = exitRun(2, nil, { { 3, { vehicleState = 4, mounted = true, driver = true, activePersp = 0 } },
                               { 60, { vehicleState = 0, mounted = true, driver = true, activePersp = 0 } },
                               { 30, { vehicleState = 0, mounted = false } } })
  local outside = 0
  for _, q in ipairs(rU) do if q.vs ~= 4 then outside = outside + 1 end end
  check("exit keep: only while the vehicle state is 4 (exit)", #rU >= 1 and outside == 0, string.format("%d/%d", outside, #rU))
  -- 2026-11-01: the keep requests are no-ops on cars (the native exit forces the vehicle camera to FPP 16 ms after the exit
  -- start, log 07-50-12): off for a positively identified car, the FPP-frame count stays as a passive diagnostic; bikes and
  -- an unknown class keep MOTO-1 A as it was (the cases above)
  local car = function(c) c.vehCar = true end
  local rc, _, linesC, cC = exitRun(2, nil, nil, car)
  local diag, requestLine, resultLine = 0, 0, 0
  for _, l in ipairs(linesC) do
    if string.find(l, "exit vehicle camera (no requests, diagnostic): active stayed TPP=false (", 1, true) then diag = diag + 1 end
    if string.find(l, "exit keep vehicle TPP: request", 1, true) then requestLine = requestLine + 1 end
    if string.find(l, "exit keep result", 1, true) then resultLine = resultLine + 1 end
  end
  check("exit keep: a car sends no exit keep request by default; the passive diagnostic logs the FPP frames once",
    #rc == 0 and diag == 1 and requestLine == 0 and resultLine == 0 and (cC.stats.exitKeepRequests or 0) == 0,
    string.format("%d requests, diag %d", #rc, diag))
  local rcOn = exitRun(2, { carExitKeepVehicleTpp = true }, nil, car)
  check("exit keep: carExitKeepVehicleTpp = true restores the requests for a car (A/B)", #rcOn == 8 and rcOn[1].p == 2)
  local rb = exitRun(2, nil, nil, function(c) c.vehBike = true end)
  check("exit keep: a bike keeps the requests (MOTO-1 A unchanged)", #rb == 8 and rb[1].p == 2)
  local rcb = exitRun(2, nil, nil, function(c) c.vehCar = true; c.vehBike = true end)
  check("exit keep: a vehicle marked as both car and bike is a bike", #rcb == 8)
  local rcOff = exitRun(2, { exitKeepVehicleTpp = false }, nil, function(c) c.vehBike = true end)
  check("exit keep: bike, exitKeepVehicleTpp = false -> no request, diagnostic only", #rcOff == 0)
  -- the A/B setter drives both keys
  V.SetExitKeepVehicleTpp(true)
  check("exit keep: SetExitKeepVehicleTpp(true) also restores the car requests", V.ctrl.cfg.carExitKeepVehicleTpp == true)
  V.SetExitKeepVehicleTpp(false)
  check("exit keep: SetExitKeepVehicleTpp(false) switches both off", V.ctrl.cfg.exitKeepVehicleTpp == false
    and V.ctrl.cfg.carExitKeepVehicleTpp == false)
  V.ctrl.cfg.exitKeepVehicleTpp, V.ctrl.cfg.carExitKeepVehicleTpp = true, false
  check("exit keep: defaults (bikes / unknown on, cars off)", V.DEFAULTS.exitKeepVehicleTpp == true
    and V.DEFAULTS.carExitKeepVehicleTpp == false)
  check("exit keep: A/B setter (not persisted)", V.SetExitKeepVehicleTpp(false) == false and V.SetExitKeepVehicleTpp(true) == true)
end

print(string.format("utpp vehiclecam math / settings offline: %d passed, %d failed", pass, fail))
assert(fail == 0, "utpp vehiclecam math / settings offline checks failed")

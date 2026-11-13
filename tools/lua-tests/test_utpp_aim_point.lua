-- 2026-10-30 aim point feed (barrel pitch = bullet line, plan 2.A) and the utpp_probe step 0 axis math. Offline, mocked
-- natives; no game. Run: python Source\tools\lua-tests\test_utpp_camera_run.py test_utpp_aim_point.lua
--   1. CM.AIM_POINT off / no P: the native aim / camera target org / fwd bit-identical (==) to the code before 2026-10-30
--      over a randomized grid (ADS with and without the parallax-corrected look of the last frame, hipfire).
--   2. P mode: Q = org + fwd * 60 (what Native_SetAimTarget / SetCameraTarget build) = P; a settled miss = the old ray
--      exactly; d >= max(2 m, eye + aimLookMinAhead, eye + minPastEye 2.5); after the hold a glide out to the far end, then
--      no P. CM.AIM_POINT ships disabled (review 2026-10-30).
--   3. Wall edge 60 m <-> 2 m at 60 / 30 fps, aim and hipfire boom, camera pitch -45..45: the per-frame step of the
--      chest (0, 0, 1.4) -> Q yaw / pitch (MotionDriver AimAnglesFromModel) <= 1 deg at 60 fps (<= 60 deg/s); -60 / 60 INFO.
--   4. M.AimPointUpdate: own ray past the eye plane, rayHz throttle, armed gate, reuse of a fresh look ray, enabled off.
--   5. utpp_probe P.AimAxisMath (muzzle axis in the WeaponRight frame, pitch errors) and one mocked AimProbe sample.
UTPP_CAMERA_MATH_PATH = UTPP_CAMERA_MATH_PATH or "Source/plugin/UTPP/cet/utpp_camera/camera_math.lua"
local CM = dofile(UTPP_CAMERA_MATH_PATH)
local passed, failed = 0, 0
local function check(name, ok, info)
  if ok then passed = passed + 1; print("[PASS] " .. name)
  else failed = failed + 1; print("[FAIL] " .. name .. (info and (" -- " .. tostring(info)) or "")) end
end
local function same(a, b) return a.x == b.x and a.y == b.y and a.z == b.z end
local deg, atan2, sqrt = math.deg, math.atan2, math.sqrt
local AP = CM.AIM_POINT

-- ---------------------------------------------------------------- 1. bit identity without P
-- reference = the ADS feed of utpp_camera/init.lua before 2026-10-30 (copied verbatim)
local function oldAds(res, lwPrev, view)
  local fwd = CM.forward(res.yaw, res.pitch)
  local org = res.world
  if lwPrev and view and view.pos then
    local dy = CM.wrapDeg(lwPrev.yaw - (lwPrev.camYaw or lwPrev.yaw))
    local dp = lwPrev.pitch - (lwPrev.camPitch or lwPrev.pitch)
    if math.abs(dy) > 1e-6 or math.abs(dp) > 1e-6 then
      fwd = CM.forward(res.yaw + dy, CM.clamp(res.pitch + dp, -89, 89))
      org = view.pos
    end
  end
  return org, fwd
end
math.randomseed(20261030)
local function rnd(a, b) return a + (b - a) * math.random() end
local bad, n = 0, 0
for _ = 1, 2000 do
  local res = { world = { x = rnd(-500, 500), y = rnd(-500, 500), z = rnd(-20, 80) }, yaw = rnd(-180, 180), pitch = rnd(-85, 85) }
  local view = math.random() < 0.8 and { pos = { x = res.world.x + rnd(-2, 2), y = res.world.y + rnd(-2, 2), z = res.world.z + rnd(-1, 1) } } or {}
  local lwPrev = nil
  local k = math.random(3)
  if k == 2 then lwPrev = { yaw = res.yaw + rnd(-10, 10), pitch = res.pitch + rnd(-10, 10), camYaw = res.yaw, camPitch = res.pitch }
  elseif k == 3 then lwPrev = { yaw = res.yaw, pitch = res.pitch } end
  local o1, f1 = oldAds(res, lwPrev, view)
  local o2, f2 = CM.aimTargetRay(res, lwPrev, view and view.pos or nil, nil)
  local o3, f3 = CM.aimPointRay(res.world, CM.forward(res.yaw, res.pitch), nil)
  n = n + 1
  if not (same(o1, o2) and same(f1, f2) and o3 == res.world and same(f3, CM.forward(res.yaw, res.pitch))) then bad = bad + 1 end
end
check("1: no P -> ADS / hipfire native target org / fwd bit-identical to before (2000 random poses)", bad == 0,
  string.format("%d of %d differ", bad, n))

-- ---------------------------------------------------------------- 2. P mode
do
  local res = { world = { x = 12.5, y = -7.25, z = 3.1 }, yaw = 33, pitch = -21 }
  local fwd = CM.forward(res.yaw, res.pitch)
  local org, f = CM.aimPointRay(res.world, fwd, 7.5)
  local Q = CM.add(org, CM.scale(CM.norm(f), AP.targetDist))
  local Pt = CM.add(res.world, CM.scale(fwd, 7.5))
  check("2: P mode: native Q = org + norm(fwd) * 60 = P (7.5 m along the camera ray), fwd = camera forward",
    CM.dist(Q, Pt) < 1e-9 and same(f, fwd), CM.dist(Q, Pt))
  local o60 = CM.aimPointRay(res.world, fwd, AP.targetDist)
  check("2: d = 60 m (a miss) -> org == the camera position bit for bit", same(o60, res.world))
  local oA, fA = CM.aimTargetRay(res, { yaw = res.yaw + 5, pitch = res.pitch + 3, camYaw = res.yaw, camPitch = res.pitch },
    { x = 0, y = 0, z = 0 }, 7.5)
  check("2: ADS with P ignores the parallax look correction (P is already parallax-correct)", same(oA, org) and same(fA, fwd))
  check("2: CM.AIM_POINT.targetDist = 60 (aimofs::Params::targetDistance, src/mm/AimOffset.hpp)", AP.targetDist == 60)
  check("2: CM.AIM_POINT ships disabled (until the DLL loop, seam C and step 0 land together)", AP.enabled == false)
  check("2: near clamp minPastEye 2.5 m past the eye plane (shared with BarrelPitchLoop kBarrelMinTargetDist)",
    AP.minPastEye == 2.5, AP.minPastEye)
  -- filter: a new filter starts at the far end (= the old target), glides, settles; settled miss returns farD exactly
  local st = CM.newAimPoint()
  local d0 = CM.aimPointStep(st, AP, 1 / 60, 0, 60, 1.1, 1.0, 60)
  check("2: first sample (miss) -> d = 60 exactly", d0 == 60, d0)
  st = CM.newAimPoint()
  local d1 = CM.aimPointStep(st, AP, 1 / 60, 0, 5, 1.1, 1.0, 60)
  check("2: first sample with a 5 m hit glides in from the far end (no snap)", d1 > 30, d1)
  local t, d = 0, d1
  for _ = 1, 180 do t = t + 1 / 60; d = CM.aimPointStep(st, AP, 1 / 60, t, 5, 1.1, 1.0, 60) end
  check("2: 5 m hit settled after 3 s", math.abs(d - 5) < 1e-6, d)
  for _ = 1, 240 do t = t + 1 / 60; d = CM.aimPointStep(st, AP, 1 / 60, t, 60, 1.1, 1.0, 60) end
  check("2: back to a miss: settled to 60 exactly after 4 s", d == 60, string.format("%.12f", d))
  for _ = 1, 120 do t = t + 1 / 60; d = CM.aimPointStep(st, AP, 1 / 60, t, 0.4, 1.1, 1.0, 60) end
  check("2: hit 0.4 m -> clamped to eye 1.1 + minPastEye 2.5 = 3.6 m", math.abs(d - 3.6) < 1e-6, d)
  for _ = 1, 120 do t = t + 1 / 60; d = CM.aimPointStep(st, AP, 1 / 60, t, 0.4, 0.2, 1.0, 60) end
  check("2: eye 0.2 m ahead -> clamped to 0.2 + 2.5 = 2.7 m", math.abs(d - 2.7) < 1e-6, d)
  local tHold = t
  for _ = 1, 25 do t = t + 1 / 60; d = CM.aimPointStep(st, AP, 1 / 60, t, nil, 0.2, 1.0, 60) end
  check("2: no ray result < hold (0.5 s): the last P kept", d ~= nil and math.abs(d - 2.7) < 1e-6, d)
  -- expiry = a miss: glide out to the far end at the rate limit (chest pitch step bounded), nil only once settled there
  local prevD, maxCP, glide, endD, ended = d, 0, 0, nil, false
  while t - tHold <= AP.hold + 3 do
    t = t + 1 / 60
    d = CM.aimPointStep(st, AP, 1 / 60, t, nil, 0.2, 1.0, 60)
    if d == nil then ended = true; break end
    if t - tHold > AP.hold + 1e-6 then glide = glide + 1 end
    local dx = 1 / (d - 0.2) - 1 / (prevD - 0.2)
    maxCP = math.max(maxCP, math.abs(dx))
    prevD, endD = d, d
  end
  check("2: no ray result > hold: P glides out (no one-frame jump to the old target), 1/e step <= invRate * dt",
    glide > 1 and maxCP <= AP.invRate / 60 + 1e-9, string.format("glide %d frames, max 1/e step %.5f", glide, maxCP))
  check("2: ... nil (the old targets) only after the glide settled at the far end, state cleared",
    ended and endD ~= nil and math.abs(endD - 60) < 0.5 and st.x == nil and st.d == nil and st.raw == nil,
    string.format("ended %s last d %s", tostring(ended), tostring(endD)))
  st = CM.newAimPoint()
  check("2: a fresh filter without a result -> nil", CM.aimPointStep(st, AP, 1 / 60, 0, nil, 0.2, 1.0, 60) == nil)
  st = CM.newAimPoint()
  d = CM.aimPointStep(st, AP, 1 / 60, 0, 100, 0, 1.0, 100)
  check("2: camera.aim.range 100 m: a miss = 100 m (far end = max(range, 60))", d == 100, d)
end

-- ---------------------------------------------------------------- 3. wall edge: chest -> Q step per frame
-- entity at the origin, yaw 0; pivot 1.55 m (standHeight); aim boom 1.1 / side 0.45 / height 0.05, boom pitch * 0.35;
-- hipfire Medium 2.75 / 0.50 / -0.35, full boom pitch; eye (FPP camera) at (0, 0.08, 1.62)
local function chest(Q)
  local x, y, z = Q.x, Q.y, Q.z - 1.4
  return deg(atan2(-x, y)), deg(atan2(z, sqrt(x * x + y * y)))
end
local GEOM = {
  { name = "aim", dist = 1.1, side = 0.45, height = 0.05, boomScale = 0.35 },
  { name = "hipfire", dist = 2.75, side = 0.50, height = -0.35, boomScale = 1.0 },
}
local function edgeRun(g, pitch, fps)
  local pivot, eye = { x = 0, y = 0, z = 1.55 }, { x = 0, y = 0.08, z = 1.62 }
  local cam = CM.add(pivot, CM.boomOffset(0, pitch * g.boomScale, g.side, g.dist, g.height))
  local fwd = CM.forward(0, pitch)
  local ahead = math.max(0, CM.dot(CM.sub(eye, cam), fwd))
  local st, t, dt = CM.newAimPoint(), 0, 1 / fps
  local maxStep, prevY, prevP, settled, back, missExact = 0, nil, nil, nil, nil, true
  local function frame(raw)
    t = t + dt
    local d = CM.aimPointStep(st, AP, dt, t, raw, ahead, 1.0, 60)
    local org, f = CM.aimPointRay(cam, fwd, d)
    local Q = CM.add(org, CM.scale(f, 60))
    local y, p = chest(Q)
    if prevY then maxStep = math.max(maxStep, math.abs(CM.wrapDeg(y - prevY)), math.abs(p - prevP)) end
    prevY, prevP = y, p
    return d, org
  end
  for _ = 1, fps do local _, org = frame(60); if not same(org, cam) then missExact = false end end
  local d
  for _ = 1, 3 * fps do d = frame(2.0) end
  settled = d
  -- absolute chest (0, 0, 1.4) -> P angles at the wall clamp (what the open-loop v2 aim offset turns the spine to)
  local absY, absP = chest(CM.add(cam, CM.scale(fwd, settled)))
  for _ = 1, 3 * fps do d = frame(60) end
  back = d
  return maxStep, settled, back, missExact, math.max(2.0, ahead + 1.0, ahead + AP.minPastEye), absY, absP
end
for _, g in ipairs(GEOM) do
  for _, fps in ipairs({ 60, 30 }) do
    local worst, worstP, okSettle, okBack, okMiss = 0, nil, true, true, true
    local info, absInfo, absYMax, absPMax = {}, {}, 0, 0
    for pitch = -60, 60, 15 do
      local step, settled, back, missExact, minD, absY, absP = edgeRun(g, pitch, fps)
      absYMax, absPMax = math.max(absYMax, math.abs(absY)), math.max(absPMax, math.abs(absP - pitch))
      absInfo[#absInfo + 1] = string.format("%d:%.1f/%.1f@%.2fm", pitch, absY, absP, settled)
      local strict = math.abs(pitch) <= 45
      if strict and step > worst then worst, worstP = step, pitch end
      if math.abs(settled - minD) > 1e-6 then okSettle = false end
      if back ~= 60 then okBack = false end
      if not missExact then okMiss = false end
      info[#info + 1] = string.format("%d:%.2f", pitch, step)
    end
    local limit = 60 / fps -- 1 deg per frame at 60 fps = 60 deg/s
    check(string.format("3: %s boom, %d fps, wall edge 60 m <-> clamp, pitch -45..45: chest -> Q step <= %.1f deg/frame",
      g.name, fps, limit), worst <= limit, string.format("worst %.3f deg at pitch %s", worst, tostring(worstP)))
    print(string.format("[INFO] 3: %s %d fps max chest step per camera pitch (deg): %s", g.name, fps, table.concat(info, " ")))
    -- review 2026-10-30 (high): the absolute angle, not only the step - before enabling CM.AIM_POINT these must be acceptable
    print(string.format("[INFO] 3: %s %d fps chest -> P at the wall clamp, camera pitch:yaw/pitch@d: %s; max |yaw| %.1f, "
      .. "max |pitch - camera pitch| %.1f deg", g.name, fps, table.concat(absInfo, " "), absYMax, absPMax))
    check(string.format("3: %s %d fps: miss = the old camera ray bit for bit, wall settles on the clamp, back to 60 exactly",
      g.name, fps), okSettle and okBack and okMiss)
  end
end

-- ---------------------------------------------------------------- 4. M.AimPointUpdate (init.lua)
do
  local calls = {}
  Vector4 = { new = function(x, y, z, w) return { x = x, y = y, z = z, w = w } end }
  Game = { UTPPSetCameraTarget = function(p, d, on) calls[#calls + 1] = { p = p, d = d, on = on } end,
           UTPPSetAimTarget = function() end, UTPPSetPoseWriteTpp = function() return true end }
  local M = dofile(UTPP_CAMERA_PATH or "Source/plugin/UTPP/cet/utpp_camera/init.lua")
  local apWas = M.CM.AIM_POINT.enabled
  check("4: init.lua default: aim point disabled -> no P, no ray", M.AimPointUpdate({ world = { x = 0, y = 0, z = 0 }, yaw = 0,
    pitch = 0 }, { x = 0, y = 1, z = 0 }, function() error("no ray expected") end, 1 / 60) == nil and apWas == false)
  M.CM.AIM_POINT.enabled = true -- the rest of 4 exercises the enabled path
  local logs = {}
  M.Log = function(x) logs[#logs + 1] = x end
  M.G = { clock = 0 }
  local MCM = M.CM
  local res = { world = { x = 10, y = 20, z = 2 }, yaw = 0, pitch = 0 }
  local eye = { x = 10.4, y = 21.0, z = 1.6 } -- 1.0 m ahead of the camera along +Y
  local rays = {}
  local hit = nil
  local function ray(a, b) rays[#rays + 1] = { a = a, b = b }; return hit end
  local armed = true
  hit = 10
  local d = M.AimPointUpdate(res, nil, ray, 1 / 60, function() return armed end)
  check("4: eye plane never known (no FPP camera): no ray, no P (the old targets)", d == nil and #rays == 0 and M.apt.ahead == nil)
  armed, hit = false, nil
  M.G.clock = 0.5
  d = M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: unarmed (no weapon, no aim): no ray, no P", d == nil and #rays == 0)
  armed, hit = true, 10
  M.G.clock = 1
  d = M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: armed: own ray from rayPastEye past the eye plane (1.5 m) to camera.aim.range", #rays == 1
    and math.abs(rays[1].a.y - 21.5) < 1e-9 and math.abs(rays[1].b.y - 80) < 1e-9 and d ~= nil and d > 11.5,
    rays[1] and string.format("from y %.3f to y %.3f d %s", rays[1].a.y, rays[1].b.y, tostring(d)))
  check("4: raw distance = start + hit = 11.5 m", M.apt.raw ~= nil and math.abs(M.apt.raw - 11.5) < 1e-9, M.apt.raw)
  M.G.clock = 1 + 1 / 60
  M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: throttled to rayHz (no second ray 1/60 s later)", #rays == 1)
  M.G.clock = 1 + 2 / 60 + 1e-4
  M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: next ray after 1/30 s", #rays == 2)
  M.G.lwpDist, M.G.lwpAt = 8, M.G.clock
  M.G.clock = M.G.clock + 1 / 60
  M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: fresh parallax look ray past the start reused (no own ray), raw 8 m", #rays == 2 and M.apt.raw == 8, M.apt.raw)
  M.G.lwpDist, M.G.lwpAt = 1.2, M.G.clock
  M.G.clock = M.G.clock + 2 / 60
  M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  check("4: a look ray hit before the start (maybe V herself) is not reused: own ray", #rays == 3)
  local p, dd = M.AimPoint()
  check("4: M.AimPoint() = world P on the camera ray and its distance", p ~= nil and math.abs(p.y - (20 + dd)) < 1e-9
    and p.x == 10 and p.z == 2)
  check("4: one log line when P starts", #logs == 1 and logs[1]:find("crosshair point P", 1, true) ~= nil, logs[1])
  M.CM.AIM_POINT.enabled = false
  d = M.AimPointUpdate(res, eye, ray, 1 / 60, function() return armed end)
  M.CM.AIM_POINT.enabled = true
  check("4: CM.AIM_POINT.enabled = false -> no P, state cleared, logged", d == nil and M.apt.x == nil and M.AimPoint() == nil
    and #logs == 2)
  -- FeedCameraTarget: d = nil = before; d = P
  M.cameraTargetSent = false
  M.FeedCameraTarget(res)
  local c1 = calls[#calls]
  M.FeedCameraTarget(res, nil)
  local c2 = calls[#calls]
  local fwd = MCM.forward(res.yaw, res.pitch)
  check("4: FeedCameraTarget(res, nil) == FeedCameraTarget(res) == camera position / forward (as before)",
    c1.p.x == res.world.x and c1.p.y == res.world.y and c1.p.z == res.world.z and c2.p.x == c1.p.x and c2.p.y == c1.p.y
    and c2.p.z == c1.p.z and c2.d.x == fwd.x and c2.d.y == fwd.y and c2.d.z == fwd.z)
  M.FeedCameraTarget(res, 6)
  local c3 = calls[#calls]
  check("4: FeedCameraTarget(res, 6): pos + dir * 60 = P 6 m along the camera ray",
    math.abs(c3.p.y + c3.d.y * 60 - 26) < 1e-9 and c3.p.x == 10 and c3.p.z == 2 and c3.on == true)
  M.apt.x = 0.5
  M.FeedCameraTarget(nil)
  check("4: FeedCameraTarget(nil) resets the shared filter", M.apt.x == nil and calls[#calls].on == false)
  M.CM.AIM_POINT.enabled = apWas
  -- the public switch utpp_motion drives together with the DLL barrel loop (mm.aimAtCrosshair)
  local r1, e1 = M.SetAimPoint(true), M.AimPointEnabled()
  local r2, e2 = M.SetAimPoint(false), M.AimPointEnabled()
  check("4: M.SetAimPoint(on) sets CM.AIM_POINT.enabled and returns it; AimPointEnabled reads it", r1 == true and e1 == true
    and M.CM.AIM_POINT.enabled == false and r2 == false and e2 == false)
  M.CM.AIM_POINT.enabled = apWas
  Game, Vector4 = nil, nil
end

-- ---------------------------------------------------------------- 5. utpp_probe step 0
do
  registerForEvent = function() end
  local P = dofile(UTPP_PROBE_PATH or "Source/plugin/UTPP/cet/utpp_probe/init.lua")
  local function qAxis(ax, a) -- rotation a deg about axis ax
    local h = math.rad(a) / 2
    local s = math.sin(h)
    return { i = ax.x * s, j = ax.y * s, k = ax.z * s, r = math.cos(h) }
  end
  local function qmul(a, b)
    return { i = a.r * b.i + a.i * b.r + a.j * b.k - a.k * b.j, j = a.r * b.j - a.i * b.k + a.j * b.r + a.k * b.i,
             k = a.r * b.k + a.i * b.j - a.j * b.i + a.k * b.r, r = a.r * b.r - a.i * b.i - a.j * b.j - a.k * b.k }
  end
  -- frame yawed 30 deg, muzzle = frame * 7 deg down about the frame X (pitch); muzzle 0.25 m along the frame +Y
  local fQ = qAxis({ x = 0, y = 0, z = 1 }, 30)
  local mQ = qmul(fQ, qAxis({ x = 1, y = 0, z = 0 }, -7))
  local fPos = { x = 1, y = 2, z = 1.3 }
  local fY = CM.forward(30, 0)
  local mPos = CM.add(fPos, CM.scale(fY, 0.25))
  local mFwd = CM.forward(30, -7)
  local aimP = CM.add(mPos, CM.scale(mFwd, 10)) -- P straight along the muzzle forward
  local m = P.AimAxisMath(mPos, mQ, fPos, fQ, CM.forward(30, -3), aimP)
  check("5: probe axis math: muzzle +Y vs frame +Y = 7 deg, muzzle +Y in the frame = (0, cos 7, -sin 7)",
    math.abs(m.axisOff - 7) < 1e-6 and math.abs(m.mYinF.x) < 1e-9 and math.abs(m.mYinF.y - math.cos(math.rad(7))) < 1e-9
    and math.abs(m.mYinF.z + math.sin(math.rad(7))) < 1e-9, m.axisOff)
  check("5: probe: muzzle offset in the frame = (0, 0.25, 0)", math.abs(m.ofsF.x) < 1e-9 and math.abs(m.ofsF.y - 0.25) < 1e-9
    and math.abs(m.ofsF.z) < 1e-9)
  check("5: probe pitches: muzzle -7, frame 0, camera -3, muzzle -> P -7; errM 0, errF +7, yaw errors 0",
    math.abs(m.mPitch + 7) < 1e-6 and math.abs(m.fPitch) < 1e-6 and math.abs(m.camPitch + 3) < 1e-6
    and math.abs(m.pPitch + 7) < 1e-6 and math.abs(m.errM) < 1e-6 and math.abs(m.errF - 7) < 1e-6
    and math.abs(m.yawM) < 1e-6 and math.abs(m.yawF) < 1e-6)
  local m2 = P.AimAxisMath(mPos, mQ, nil, nil, nil, nil)
  check("5: probe without frame / P: only the muzzle pitch, no error", m2.mPitch ~= nil and m2.axisOff == nil
    and m2.errM == nil and m2.ofsF == nil)
  -- one mocked sample: slot WeaponRight via SlotComponent.GetSlotTransform (returned form), muzzle orientation, P from
  -- utpp_camera AimPoint()
  local lines = {}
  P.Log = function(x) lines[#lines + 1] = tostring(x) end
  local item = {}
  function item:GetMuzzleSlotWorldTransform() return { position = mPos, orientation = mQ } end
  function item:GetItemID() return 1 end
  local comp = { slots = { { slotName = "WeaponRight", boneName = "WeaponRight", relativePosition = { x = 0, y = 0, z = 0 },
                             relativeRotation = { i = 0, j = 0, k = 0, r = 1 } } } }
  function comp:GetSlotTransform(_, xf) return true, { Position = fPos, Orientation = fQ } end
  local player = {}
  function player:GetComponents() return { comp } end
  WorldTransform = { new = function() return {} end }
  WorldPosition = { ToVector4 = function(p) return p end }
  ItemID = { GetTDBID = function() return "t" end }
  TDBID = { ToStringDEBUG = function() return "Items.Preset_Lexington_Default" end }
  TweakDBID = { new = function(s) return s end }
  GetMod = function(n) if n == "utpp_camera" then return { AimPoint = function() return aimP, 10.25 end } end end
  Game = { GetPlayer = function() return player end, NameToString = function(s) return s end,
           GetTransactionSystem = function() return { GetItemInSlot = function() return item end } end }
  P.AimProbe(true)
  P.AimProbeSample()
  P.AimProbe(false)
  local axis, summary = nil, nil
  for _, l in ipairs(lines) do
    if l:find("^aimaxis t=") then axis = l end
    if l:find("^aimaxis summary") then summary = l end
  end
  check("5: AimProbe sample logs an aimaxis line (slot frame, axisOff 7.00, errM 0.00, errF 7.00, P from utpp_camera)",
    axis ~= nil and axis:find("frame=slot", 1, true) and axis:find("axisOff=7.00", 1, true)
    and axis:find("errM=%-?0%.00 ") and axis:find("errF=7.00", 1, true) and axis:find("p=camera d=10.25", 1, true)
    and axis:find("ofsF=(0.000,0.250,0.000)", 1, true) ~= nil, axis or table.concat(lines, " | "))
  check("5: AimProbe(false) logs a per-weapon aimaxis summary",
    summary ~= nil and summary:find("item=Items.Preset_Lexington_Default samples=1 meanAxisOff=7.00", 1, true) ~= nil, summary)
  check("5: the snapshot keeps no weapon handle", P.ap.last ~= nil and P.ap.last.weapon == nil)
  Game, GetMod, WorldTransform, WorldPosition, ItemID, TDBID, TweakDBID = nil, nil, nil, nil, nil, nil, nil
end

print(string.format("utpp aim point test: %d passed, %d failed", passed, failed))
if failed > 0 then error("utpp aim point test failed") end

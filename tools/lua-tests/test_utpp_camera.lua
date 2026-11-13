-- UTPP utpp_camera 0.1.0 offline tests: camera math (camera_math.lua) - orbit, input, calibration, look modes,
-- profiles, shoulder, pivot, collision with mocked rays, transitions (state machine), solver, frame-rate independence.
-- 0.1.3 (camera round 3): capsule pivot (stand / crouch, FPP pose ignored), follow-mode yaw limiter + body-flip hold
-- (the "camera whips behind the back" report), per-transition enter time (return from aiming), lookReset.
-- 0.1.4 aim occlusion (level A): t0 / aim point P / blocked decision (tolerance, P behind the muzzle, far end, near
-- wall), throttle and hysteresis edge timing at 30 / 60 / 144 fps.
-- Globals from the runner: UTPP_CAMERA_MATH_PATH (camera_math.lua).
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, (not ok and detail) and (" -- " .. tostring(detail)) or ""))
end
local function near(a, b, eps) return math.abs(a - b) <= (eps or 1e-6) end
local function vnear(a, b, eps) return near(a.x, b.x, eps) and near(a.y, b.y, eps) and near(a.z, b.z, eps) end
-- geometry fixture for the solver / aim maths: the original "Medium" profile (the registered default profile 2 since
-- 2026-10-20 is 2.75 m / -0.35 m / FOV 55; its values are asserted in test_utpp_user_defaults.lua). These tests do not test the default.
local MEDIUM = { name = "Medium", distance = 2.10, height = 0.12, side = 0.50, fov = 56, sens = 1.0 }
local function vs(v) return string.format("(%.4f, %.4f, %.4f)", v.x, v.y, v.z) end

local path = UTPP_CAMERA_MATH_PATH or "Source/plugin/UTPP/cet/utpp_camera/camera_math.lua"
local CM = dofile(path)
check("module loads", type(CM) == "table" and CM.VERSION == "0.1.6")
local V = CM.v3

-- ================================================================ quaternions / conventions
do
  local f0 = CM.qrot(CM.qFromYawPitch(0, 0), V(0, 1, 0))
  check("yaw 0 looks along +Y", vnear(f0, V(0, 1, 0)), vs(f0))
  local f90 = CM.qrot(CM.qFromYawPitch(90, 0), V(0, 1, 0))
  check("yaw +90 turns left (to -X)", vnear(f90, V(-1, 0, 0)), vs(f90))
  local fp = CM.qrot(CM.qFromYawPitch(0, 30), V(0, 1, 0))
  check("pitch +30 looks up", vnear(fp, V(0, math.cos(math.rad(30)), 0.5)), vs(fp))
  local okAll, worst = true, 0
  for yaw = -175, 175, 35 do
    for pitch = -60, 50, 22 do
      local q = CM.qFromYawPitch(yaw, pitch)
      local a, b = CM.qrot(q, V(0, 1, 0)), CM.forward(yaw, pitch)
      local y2, p2 = CM.yawPitchFromForward(a)
      local e = math.max(CM.dist(a, b), math.abs(CM.wrapDeg(y2 - yaw)), math.abs(p2 - pitch))
      if e > 1e-6 then okAll = false end
      worst = math.max(worst, e)
    end
  end
  check("qFromYawPitch / forward / yawPitchFromForward round trip (all angles)", okAll, worst)
  local r = CM.qrot(CM.qFromYawPitch(0, 0), V(1, 0, 0))
  check("camera right is +X at yaw 0", vnear(r, V(1, 0, 0)), vs(r))
  check("wrapDeg", near(CM.wrapDeg(190), -170) and near(CM.wrapDeg(-190), 170) and near(CM.wrapDeg(180), 180)
        and near(CM.wrapDeg(540), 180) and near(CM.wrapDeg(0), 0))
  -- toLocal: camera offset expressed in the player space is independent of the player yaw
  local okL = true
  for _, py in ipairs({ 0, 37, 90, -135 }) do
    local pq = CM.qFromYawPitch(py, 0)
    local ppos = V(10, -4, 2)
    local loc = V(0.5, -2, 1.6)
    local wpos = CM.add(ppos, CM.qrot(pq, loc))
    local wq = CM.qFromYawPitch(py + 12, -8)
    local lp, lq = CM.toLocal(ppos, pq, wpos, wq)
    local f = CM.qrot(lq, V(0, 1, 0))
    local y, p = CM.yawPitchFromForward(f)
    if not (vnear(lp, loc, 1e-6) and near(y, 12, 1e-6) and near(p, -8, 1e-6)) then okL = false end
  end
  check("toLocal: position and orientation relative to the player root", okL)
  local m = { X = { x = 1, y = 0, z = 0, w = 0 }, Y = { x = 0, y = 0.8, z = 0.6, w = 0 }, Z = { x = 0, y = -0.6, z = 0.8, w = 0 },
              W = { x = 3, y = 4, z = 5, w = 1 } }
  check("matrix rows layout: forward = Y row, position = W row",
        vnear(CM.matrixForward(m, "rows"), V(0, 0.8, 0.6)) and vnear(CM.matrixPosition(m, "rows"), V(3, 4, 5)))
end

-- ================================================================ filters
do
  local function run(fps, secs, k)
    local x = 0
    for _ = 1, math.floor(fps * secs + 0.5) do x = CM.approach(x, 1, k, 1 / fps) end
    return x
  end
  check("approach is frame-rate independent (30 / 60 / 144 fps)", near(run(30, 1, 5), run(144, 1, 5), 1e-9)
        and near(run(60, 1, 5), run(144, 1, 5), 1e-9), string.format("%.9f %.9f", run(30, 1, 5), run(144, 1, 5)))
  local x, v, maxx = 0, 0, 0
  for _ = 1, 144 do x, v = CM.smoothDamp(x, 1, v, 0.12, 1 / 144); maxx = math.max(maxx, x) end
  check("smoothDamp converges without overshoot", near(x, 1, 1e-3) and maxx <= 1 + 1e-9, string.format("%.5f max %.5f", x, maxx))
end

-- ================================================================ input + calibration
do
  local cfg = CM.config()
  local dy, dp = CM.inputDelta({ mouseX = 10, mouseY = 0 }, cfg, 1 / 60, 0.1)
  check("mouse right turns right (yaw decreases)", near(dy, -1.0) and near(dp, 0), dy)
  cfg.invertY = true
  local _, dp2 = CM.inputDelta({ mouseX = 0, mouseY = 5 }, cfg, 1 / 60, 0.1)
  check("invert Y", near(dp2, -0.5), dp2)
  cfg.invertY = false
  local dzy = CM.inputDelta({ padX = 0.1, padY = 0 }, cfg, 1 / 60)
  check("pad deadzone", near(dzy, 0), dzy)
  local function padSecond(fps)
    local sum = 0
    for _ = 1, fps do sum = sum + CM.inputDelta({ padX = 1, padY = 0 }, cfg, 1 / fps) end
    return sum
  end
  check("pad orbit speed per second independent of fps", near(padSecond(30), -cfg.padDegPerSec, 1e-6)
        and near(padSecond(144), -cfg.padDegPerSec, 1e-6), padSecond(30))
  local cal = CM.newCalibration()
  for _ = 1, 20 do CM.calibrate(cal, 10, -1.5) end
  check("calibration: degrees per mouse unit from the game's body turn", near(cal.value, 0.15, 1e-9) and cal.samples == 20, cal.value)
  CM.calibrate(cal, 10, 1.5)
  check("calibration ignores opposite-sign samples", cal.samples == 20)
  local c2 = CM.newCalibration()
  for _ = 1, 5 do CM.calibrate(c2, 10, -1.0) end
  check("mouseUnit: default until 10 samples, then calibrated, override wins",
        near(CM.mouseUnit(c2, cfg, 0), cfg.degPerMouseUnit) and near(CM.mouseUnit(cal, cfg, 0), 0.15)
        and near(CM.mouseUnit(cal, cfg, 0.3), 0.3))
end

-- ================================================================ look modes
do
  local cfg = CM.config({ idleOrbit = false })
  local L = CM.newLook()
  local y, p = CM.lookStep(L, cfg, 1 / 60, 5, 3, { bodyYaw = 40, fppPitch = -12, moving = true })
  check("follow: yaw = body yaw, pitch = FPP pitch (aim aligned)", near(y, 40) and near(p, -12), y .. " " .. p)
  -- free look: camera integrates input even if the game turns V as well (heading lock missing)
  local body = 40
  for _ = 1, 60 do
    body = body + 1 -- game keeps turning V
    y, p = CM.lookStep(L, cfg, 1 / 60, 2, 0, { bodyYaw = body, fppPitch = -12, freeHeld = true, moving = true })
  end
  -- Entering free look keeps the previously shown yaw (40), including when V turns on the entry frame.
  check("free look: world yaw = shown start + input only (V turning does not double the orbit)", near(y, 40 + 120, 1e-6), y)
  check("free look: own pitch starts from the FPP pitch", near(p, -12), p)
  local function release(fps)
    local LL = CM.newLook()
    CM.lookStep(LL, cfg, 1 / fps, 0, 0, { bodyYaw = 0, fppPitch = 0, freeHeld = true })
    CM.lookStep(LL, cfg, 1 / fps, 90, 20, { bodyYaw = 0, fppPitch = 0, freeHeld = true })
    for _ = 1, math.floor(0.5 * fps + 0.5) do CM.lookStep(LL, cfg, 1 / fps, 0, 0, { bodyYaw = 0, fppPitch = 0 }) end
    return LL.yawOff, LL.pitch
  end
  local a30, p30 = release(30)
  local a144, p144 = release(144)
  check("free look release: yaw offset recenters (same at 30 and 144 fps)", near(a30, a144, 1e-6) and math.abs(a30) < 90 * 0.1,
        string.format("%.4f %.4f", a30, a144))
  check("free look release: pitch recovers to FPP pitch", math.abs(p30) < 2 and math.abs(p144) < 2, p30 .. " " .. p144)
  -- idle orbit
  local cfgI = CM.config({ idleOrbit = true, idleDelay = 0.35 })
  local LI = CM.newLook()
  local free0
  for i = 1, 30 do
    CM.lookStep(LI, cfgI, 1 / 60, 1, 0, { bodyYaw = 0, fppPitch = 0, moving = false })
    if i == 20 then free0 = LI.free end
  end
  check("idle orbit engages after the idle delay only", free0 == false and LI.free == true)
  CM.lookStep(LI, cfgI, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true })
  check("moving ends idle orbit", LI.free == false)
  -- 2026-10-22 Shooter mode (env.shooterMode from utpp_bodyturn.ShooterModeActive): no idle orbit - the camera always follows V's capsule
  local LF = CM.newLook()
  local freeF = false
  for i = 1, 60 do
    CM.lookStep(LF, cfgI, 1 / 60, 1, 0, { bodyYaw = 0, fppPitch = 0, moving = false, shooterMode = true })
    if LF.free then freeF = true end
  end
  check("Shooter: idle orbit config on, 1 s without input -> follow (L.free never set)", freeF == false and LF.free == false)
  local LN = CM.newLook()
  for i = 1, 60 do CM.lookStep(LN, cfgI, 1 / 60, 1, 0, { bodyYaw = 0, fppPitch = 0, moving = false, shooterMode = false }) end
  check("Shooter false: the same 1 s -> idle orbit free after 0.35 s (today)", LN.free == true)
  local LH = CM.newLook()
  for i = 1, 5 do CM.lookStep(LH, cfgI, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = false, shooterMode = true, freeHeld = true }) end
  check("Shooter: a free-look hold of another module (utpp_bodyturn face-the-camera align) still frees the camera", LH.free == true)
  local LD = CM.newLook()
  for i = 1, 5 do CM.lookStep(LD, cfgI, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true, shooterMode = true, decoupled = false }) end
  check("Shooter: decoupled = false (set by the adapter) keeps follow mode while moving", LD.free == false)
  -- pitch limits with own pitch source
  local cfgP = CM.config({ pitchSource = "input", idleOrbit = false })
  local LP = CM.newLook()
  local _, pp = CM.lookStep(LP, cfgP, 1 / 60, 0, 500, { bodyYaw = 0, moving = true })
  check("pitch clamped to pitchMax", near(pp, cfgP.pitchMax), pp)
  -- orbit 360 deg: continuous, no jump across +-180
  local L3 = CM.newLook()
  local prev, maxStep = nil, 0
  for _ = 1, 400 do
    local yy = CM.lookStep(L3, cfg, 1 / 60, 1, 0, { bodyYaw = 0, fppPitch = 0, freeHeld = true })
    if prev then maxStep = math.max(maxStep, math.abs(CM.wrapDeg(yy - prev))) end
    prev = yy
  end
  check("orbit 360+ deg continuous (no wrap jump)", maxStep <= 1 + 1e-6, maxStep)
end

-- ================================================================ 0.1.3 follow limiter / body flip (user report 1: idle, camera
-- turned to face V, then running backwards -> the camera whipped behind the back)
do
  -- mouse turns pass at once (aim aligned): 720 deg/s turn by the mouse
  local cfg = CM.config({ idleOrbit = false })
  local L = CM.newLook()
  local body, worst = 0, 0
  CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = body, fppPitch = 0, moving = true })
  for _ = 1, 60 do
    body = CM.wrapDeg(body + 12)
    local y = CM.lookStep(L, cfg, 1 / 60, 12, 0, { bodyYaw = body, fppPitch = 0, moving = true })
    worst = math.max(worst, math.abs(CM.wrapDeg(y - body)))
  end
  check("limiter: mouse turn (720 deg/s) passes at once, camera stays behind V", worst < 1e-6 and L.flips == 0, worst)
  -- mouse scale not calibrated yet (estimate 2x the real turn): no over-rotation, no false flip on a 360 deg flick
  local LM = CM.newLook()
  local bm, worstM = 0, 0
  CM.lookStep(LM, cfg, 1 / 60, 0, 0, { bodyYaw = bm, fppPitch = 0, moving = true })
  for _ = 1, 30 do
    bm = CM.wrapDeg(bm + 12)
    local y = CM.lookStep(LM, cfg, 1 / 60, 24, 0, { bodyYaw = bm, fppPitch = 0, moving = true })
    worstM = math.max(worstM, math.abs(CM.wrapDeg(y - bm)))
  end
  check("limiter: wrong mouse scale (2x) cannot over-rotate the camera or fake a body flip", worstM < 1e-6 and LM.flips == 0
        and LM.hold <= 0, string.format("%.3f deg, flips %d", worstM, LM.flips)) 
  -- body turn without the mouse (motion matching / target facing): rate capped, ease-in, converges
  local function autoTurn(fps, deg, dur, secs, over)
    local c = CM.config(over or { idleOrbit = false })
    local LL = CM.newLook()
    local dt = 1 / fps
    local b, prev, maxRate, firstRate, t = 0, nil, 0, nil, 0
    local samples = {}
    for i = 1, math.floor(secs * fps + 0.5) do
      t = i * dt
      b = deg * math.min(1, t / dur)
      local y = CM.lookStep(LL, c, dt, 0, 0, { bodyYaw = b, fppPitch = 0, moving = true })
      if prev then
        local r = math.abs(CM.wrapDeg(y - prev)) / dt
        maxRate = math.max(maxRate, r)
        if not firstRate and r > 0 then firstRate = r end
      end
      prev = y
      samples[#samples + 1] = { t = t, y = y }
    end
    return prev, maxRate, firstRate, LL, samples
  end
  local y90, r90, f90, L90 = autoTurn(60, 90, 0.3, 1.5)
  check("limiter: 90 deg body turn without the mouse followed at <= 240 deg/s, converged in 1.5 s",
        r90 <= 240 + 1e-6 and math.abs(CM.wrapDeg(y90 - 90)) < 0.5 and L90.flips == 0, string.format("%.1f deg/s, end %.2f", r90, y90))
  check("limiter: ease-in (first frame speed <= max / ease time * dt)", f90 and f90 <= 240 / 0.25 / 60 + 1e-6, tostring(f90))
  -- the whip scenario: idle orbit facing V (camera yaw 180 from V's back), V starts running and turns 180 deg in 0.25 s
  local function whip(fps, over)
    local c = CM.config(over or {})
    c.idleOrbit, c.idleDelay = true, 0.35
    local LL = CM.newLook()
    local dt = 1 / fps
    local n = math.floor(fps + 0.5)
    for _ = 1, n do CM.lookStep(LL, c, dt, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = false }) end -- idle orbit on
    local turnIn = math.floor(0.25 * fps + 0.5)
    for i = 1, turnIn do -- orbit 180 deg around V with the mouse (heading locked: body stays)
      CM.lookStep(LL, c, dt, 180 / turnIn, 0, { bodyYaw = 0, fppPitch = 0, moving = false })
    end
    local start = CM.wrapDeg(LL.out)
    local prev, maxRate, t, body = start, 0, 0, 0
    local heldMax, flipAt = 0, nil
    local endY
    for i = 1, math.floor(3 * fps + 0.5) do
      t = i * dt
      body = 180 * math.min(1, t / 0.25)
      local y = CM.lookStep(LL, c, dt, 0, 0, { bodyYaw = body, fppPitch = 0, moving = true })
      maxRate = math.max(maxRate, math.abs(CM.wrapDeg(y - prev)) / dt)
      if LL.flip and not flipAt then flipAt = t; LL.flip = nil end
      if flipAt and t > flipAt and t <= flipAt + c.flipHold - dt then heldMax = math.max(heldMax, math.abs(CM.wrapDeg(y - prev))) end
      prev, endY = y, y
    end
    return start, endY, maxRate, flipAt, heldMax, LL, body
  end
  local s60, e60, r60, fl60, h60, W60, b60 = whip(60)
  check("whip: idle orbit in front of V (camera yaw 180 from the body)", math.abs(CM.wrapDeg(s60 - 180)) < 1e-6, s60)
  check("whip: body flip detected (> 120 deg in 0.3 s without the mouse) -> world yaw held 0.6 s", fl60 ~= nil and fl60 <= 0.3
        and W60.flips == 1 and h60 < 1e-9, string.format("flip at %s, held step %.4f", tostring(fl60), h60))
  check("whip: yaw rate never above 240 deg/s", r60 <= 240 + 1e-6, string.format("%.1f deg/s", r60))
  check("whip: camera ends behind V's back (body yaw 180)", math.abs(CM.wrapDeg(e60 - b60)) < 1.0, string.format("%.2f vs %.2f", e60, b60))
  local okFps, detail = true, {}
  for _, fps in ipairs({ 30, 144 }) do
    local s, e, r, fl = whip(fps)
    detail[#detail + 1] = string.format("%d fps: %.1f deg/s flip %s end %.2f", fps, r, tostring(fl), e)
    if not (r <= 240 + 1e-6 and fl and math.abs(CM.wrapDeg(e - 180)) < 1.0) then okFps = false end
  end
  check("whip: same at 30 / 144 fps (rate cap, hold, end behind the back)", okFps, table.concat(detail, "; "))
  local _, eNo, rNo, flNo = whip(60, { followHoldOnFlip = false })
  check("whip: followHoldOnFlip=false -> no hold, still rate capped", flNo == nil and rNo <= 240 + 1e-6
        and math.abs(CM.wrapDeg(eNo - 180)) < 1.0, string.format("%.1f deg/s", rNo))
  local _, _, rOld = whip(60, { followMaxRate = 0, followHoldOnFlip = false })
  check("whip negative control: followMaxRate=0 (0.1.2 rigid follow) whips > 400 deg/s", rOld > 400, string.format("%.0f deg/s", rOld))
  -- teleport: the follow yaw snaps (no glide after a fast travel / load)
  local cfgT = CM.config({ idleOrbit = false })
  local LT = CM.newLook()
  CM.lookStep(LT, cfgT, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true })
  local yT = CM.lookStep(LT, cfgT, 1 / 60, 0, 0, { bodyYaw = 170, fppPitch = 0, moving = true, teleport = true })
  check("limiter: teleport snaps to the body yaw", near(yT, 170), yT)
  -- lookReset (return from aiming): yaw offset 0, shown yaw re-initialised behind the back
  local LR = CM.newLook()
  CM.lookStep(LR, cfgT, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, freeHeld = true })
  CM.lookStep(LR, cfgT, 1 / 60, 150, 0, { bodyYaw = 0, fppPitch = 0, freeHeld = true })
  CM.lookReset(LR, 20, false)
  local yR = CM.lookStep(LR, cfgT, 1 / 60, 0, 0, { bodyYaw = 20, fppPitch = 0, moving = true })
  check("lookReset: orbit yaw back behind the back at once", near(yR, 20) and LR.yawOff == 0 and LR.free == false, yR)
end

-- ================================================================ profiles and shoulder
do
  check("cycle wraps in the pool", CM.cycle(3, 3, 1) == 1 and CM.cycle(1, 3, -1) == 3 and CM.cycle(2, 5, 1) == 3)
  local P = CM.DEFAULT_PROFILES
  check("5 default profiles with all fields", #P == 5 and P[1].distance and P[5].fov and P[3].sens)
  local b = CM.newBlend(P[1])
  CM.blendTo(b, P[3], 0.5)
  local prev, maxJump, mono = CM.blendCurrent(b).distance, 0, true
  local t = 0
  while b.t < 1 do
    local c = CM.blendStep(b, 1 / 60)
    t = t + 1 / 60
    maxJump = math.max(maxJump, math.abs(c.distance - prev))
    if c.distance < prev - 1e-9 then mono = false end
    prev = c.distance
  end
  check("profile blend: monotonic, smooth, done in blend time", mono and maxJump < 0.12 and near(prev, P[3].distance)
        and t < 0.5 + 1 / 60 + 1e-9, string.format("jump %.4f t %.3f", maxJump, t))
  CM.blendTo(b, P[1], 0.5)
  for _ = 1, 10 do CM.blendStep(b, 1 / 60) end
  local mid = CM.blendCurrent(b).distance
  CM.blendTo(b, P[2], 0.5)
  check("profile switch mid-blend starts from the current value (no jump)", near(CM.blendCurrent(b).distance, mid, 1e-9))
  CM.blendStep(b, 1 / 60)
  local tBefore = b.t
  CM.blendRetarget(b, { name = "x", distance = 9, height = 0, side = 0, fov = 50, sens = 1 })
  check("live edit retargets without restarting the blend", b.t == tBefore and b.t > 0 and b.to.distance == 9)
  local st = CM.newShoulder(1)
  CM.shoulderSwap(st)
  local pv, mj = st.s, 0
  for i = 1, 30 do
    local s = CM.shoulderStep(st, 1 / 60, 0.25)
    mj = math.max(mj, math.abs(s - pv)); pv = s
    if i == 6 then CM.shoulderSwap(st) end -- swap back mid-way
  end
  check("shoulder swap smooth (also swapped back mid-way)", mj < 0.25 and near(st.s, 1), string.format("%.3f %.3f", mj, st.s))
end

-- ================================================================ pivot
do
  local cfg = CM.config({ pivotSource = "fpp" }) -- 0.1.0-0.1.2 eye-height pivot (camera.pivotSource = fpp)
  local pv = CM.newPivot()
  local pos = V(0, 0, 0)
  CM.pivotStep(pv, cfg, 1 / 60, pos, 1.6)
  local t95, over = nil, false
  for i = 1, 60 do
    local p = CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 0.3), 1.9) -- stair step 0.3 m
    if p.z > 1.9 + 1e-9 then over = true end
    if not t95 and p.z >= 1.6 + 0.3 * 0.95 then t95 = i / 60 end
  end
  check("pivot: stair step smoothed, no overshoot, 95% within 0.35 s", (not over) and t95 and t95 <= 0.35, tostring(t95))
  local p = CM.pivotStep(pv, cfg, 1 / 60, V(100, 0, 50), 51.6)
  check("pivot: teleport snaps", near(p.z, 51.6) and near(p.x, 100))
  local p2 = CM.pivotStep(CM.newPivot(), cfg, 1 / 60, V(0, 0, 0), nil)
  check("pivot: eye fallback when the FPP camera is unreadable", near(p2.z, cfg.eyeFallback))
  local p3 = CM.pivotStep(CM.newPivot(), cfg, 1 / 60, V(0, 0, 0), 9)
  check("pivot: implausible eye height rejected", near(p3.z, cfg.eyeFallback))
end
-- 0.1.3 capsule pivot (user report 6: the camera lowers as if V crouched whenever the game's FPP set plays)
do
  local cfg = CM.config()
  check("capsule pivot is the default (stand 1.55 m, crouch 1.05 m, blend 0.25 s)", cfg.pivotSource == "capsule"
        and cfg.standHeight == 1.55 and cfg.crouchHeight == 1.05 and cfg.stanceBlend == 0.25)
  local pv = CM.newPivot()
  local p0 = CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 10), 11.62)
  local minZ = 1e9
  for _ = 1, 60 do -- the FPP camera drops to 1.10 m (FPP melee / block / vehicle exit set) while V stands
    local p = CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 10), 11.10, false)
    minZ = math.min(minZ, p.z)
  end
  check("capsule: FPP camera drop ignored (pivot stays at 1.55 m)", near(p0.z, 11.55) and near(minZ, 11.55), string.format("%.3f", minZ))
  local fppCfg = CM.config({ pivotSource = "fpp" })
  local pf = CM.newPivot()
  CM.pivotStep(pf, fppCfg, 1 / 60, V(0, 0, 10), 11.62)
  local pz
  for _ = 1, 60 do pz = CM.pivotStep(pf, fppCfg, 1 / 60, V(0, 0, 10), 11.10).z end
  check("negative control: pivotSource=fpp follows the drop (the reported lowering)", pz < 11.2, string.format("%.3f", pz))
  -- crouch: 1.55 -> 1.05 blended over 0.25 s, monotonic; back up the same way
  local function crouchRun(fps)
    local pc = CM.newPivot()
    CM.pivotStep(pc, cfg, 1 / fps, V(0, 0, 0), nil, false)
    local prev, mono, doneAt = 1.55, true, nil
    for i = 1, math.floor(0.6 * fps + 0.5) do
      local z = CM.pivotStep(pc, cfg, 1 / fps, V(0, 0, 0), nil, true).z
      if z > prev + 1e-9 then mono = false end
      if not doneAt and near(z, 1.05, 1e-6) then doneAt = i / fps end
      prev = z
    end
    local up
    for _ = 1, math.floor(0.3 * fps + 0.5) do up = CM.pivotStep(pc, cfg, 1 / fps, V(0, 0, 0), nil, false).z end
    return mono, doneAt, up
  end
  local mono, doneAt, up = crouchRun(60)
  check("capsule: crouch lowers the pivot to 1.05 m in 0.25 s (monotonic), stand back up to 1.55 m", mono and doneAt
        and math.abs(doneAt - 0.25) <= 1 / 60 + 1e-9 and near(up, 1.55), string.format("%s %.3f", tostring(doneAt), up or -1))
  local _, d30 = crouchRun(30)
  local _, d144 = crouchRun(144)
  check("capsule: crouch blend time equal at 30 / 144 fps", d30 and d144 and math.abs(d30 - 0.25) <= 1 / 30 + 1e-9
        and math.abs(d144 - 0.25) <= 1 / 144 + 1e-9, string.format("%s %s", tostring(d30), tostring(d144)))
  -- stairs: the capsule height step is smoothed like before, teleport snaps
  local ps = CM.newPivot()
  CM.pivotStep(ps, cfg, 1 / 60, V(0, 0, 0), nil, false)
  local t95, over = nil, false
  for i = 1, 60 do
    local p = CM.pivotStep(ps, cfg, 1 / 60, V(0, 0, 0.3), nil, false)
    if p.z > 1.85 + 1e-9 then over = true end
    if not t95 and p.z >= 1.55 + 0.3 * 0.95 then t95 = i / 60 end
  end
  check("capsule: stair step smoothed, no overshoot, 95% within 0.35 s", (not over) and t95 and t95 <= 0.35, tostring(t95))
  local pt = CM.pivotStep(ps, cfg, 1 / 60, V(100, 0, 50), nil, false)
  check("capsule: teleport snaps", near(pt.z, 51.55) and near(pt.x, 100))
end
-- 2026-10-21 slide pivot (recording 03:39 v21.9-23.5 / 33-35 / 43-44 / 52, session log 03:37). The seated slide (PSM Locomotion
-- Slide, LocomotionDetailed 5) counted as a crouch: pivot 1.05 m, but FK of the final pose (pose CSV, root-relative) puts the
-- sliding V's neck at 0.52-0.57 m and the head at 0.62-0.66 m, 0.6 m BEHIND the root (standing: neck 1.50 / head 1.64, crouch:
-- neck 0.94 / head 1.06). The aim camera (1.58 m behind the pivot, FOV 55) then looked ~29 deg over the head: V fell out of the
-- bottom of the frame, and the 0.25 s crouch blend dropped / raised the pivot 3 m/s at the slide's edges (the fastest pivot
-- motion of the session: 2.6-4.7 m/s against a 1.1 m/s 99th percentile outside stance changes).
-- Log camTrace: v43-44 (03:39:48.7, aimW 1.00) and v52 (03:39:58, aimW 1.00) are aimed slides = the low pivot; v33-35
-- (03:39:38.5-40.5, aimW 0.00, mode free) is a free slide, whose frame was good at the crouch pivot (v33.4) - the low pivot
-- applies in proportion to the aim weight (CM.pivotStep aimW); the free slide keeps the crouch pivot over the slower blend.
do
  local unpack = table.unpack or unpack
  local cfg = CM.config()
  check("slide pivot defaults: slideHeight 0.62 m (crouch 1.05 m), slideBlend 0.5 s", cfg.slideHeight == 0.62
        and cfg.slideBlend == 0.5 and cfg.slideHeight < cfg.crouchHeight)
  -- aimW: a number or function(step, steps) -> number (nil = the free camera). The steady value comes first in the series: a
  -- jump in the very first step of a transition counts for peak() / monotone() as well.
  local function run(pv, c, fps, secs, crouch, slide, aimW)
    local zs, n = { pv.z }, math.floor(secs * fps + 0.5)
    for i = 1, n do
      local aw = type(aimW) == "function" and aimW(i, n) or aimW
      zs[#zs + 1] = CM.pivotStep(pv, c, 1 / fps, V(0, 0, 0), nil, crouch, slide, aw).z
    end
    return zs
  end
  local function settle(c, crouch, slide, aimW) -- a pivot that has been in the given stance for a while
    local pv = CM.newPivot()
    run(pv, c, 60, 1.5, crouch, slide, aimW)
    return pv
  end
  local function monotone(zs, dir) -- dir -1 = never rises, +1 = never falls
    for i = 2, #zs do if (zs[i] - zs[i - 1]) * dir < -1e-9 then return false end end
    return true
  end
  local function peak(zs, fps)
    local m = 0
    for i = 2, #zs do m = math.max(m, math.abs(zs[i] - zs[i - 1]) * fps) end
    return m
  end
  local function doneAtOf(zs, fps, z)
    for i, v in ipairs(zs) do if near(v, z, 1e-6) then return (i - 1) / fps end end
  end
  -- AIMED slide (aimW 1): stand -> slide 0.62 m after slideBlend, monotonic, never faster than the crouch's own peak (3 m/s),
  -- same at 30 / 60 / 144 fps
  for _, fps in ipairs({ 30, 60, 144 }) do
    local zs = run(settle(cfg, false, false), cfg, fps, 0.8, true, true, 1)
    local doneAt = doneAtOf(zs, fps, 0.62)
    check(string.format("slide pivot aimed (%d fps): stand -> slide: 0.62 m in 0.5 s, monotonic, peak <= 3.05 m/s", fps),
          monotone(zs, -1) and near(zs[#zs], 0.62, 1e-9) and doneAt and math.abs(doneAt - 0.5) <= 1 / fps + 1e-9 and peak(zs, fps) <= 3.05,
          string.format("done %s peak %.2f end %.3f", tostring(doneAt), peak(zs, fps), zs[#zs]))
  end
  -- crouch -> slide: from 1.05 m down, no overshoot below the slide pivot; slide -> crouch / stand: up, no overshoot
  local dn = run(settle(cfg, true, false), cfg, 60, 0.8, true, true, 1)
  check("slide pivot aimed: crouch -> slide: 1.05 -> 0.62 m monotonic, no undershoot", monotone(dn, -1) and near(dn[#dn], 0.62, 1e-9)
        and math.min(unpack(dn)) >= 0.62 - 1e-9 and dn[1] < 1.05 + 1e-9)
  for _, fps in ipairs({ 30, 60, 144 }) do
    local upC = run(settle(cfg, false, true, 1), cfg, fps, 0.8, true, false, 1) -- stand -> slide -> crouch (the crouch weight was held)
    local upS = run(settle(cfg, false, true, 1), cfg, fps, 0.8, false, false, 1)
    check(string.format("slide pivot aimed (%d fps): slide -> crouch rises to 1.05 m and slide -> stand to 1.55 m, monotonic, no overshoot", fps),
          monotone(upC, 1) and near(upC[#upC], 1.05, 1e-9) and math.max(unpack(upC)) <= 1.05 + 1e-9
          and monotone(upS, 1) and near(upS[#upS], 1.55, 1e-9) and math.max(unpack(upS)) <= 1.55 + 1e-9 and peak(upC, fps) <= 3.05,
          string.format("crouch end %.3f max %.3f / stand end %.3f", upC[#upC], math.max(unpack(upC)), upS[#upS]))
  end
  -- crouch -> slide -> crouch (the crouch weight stays 1): down to 0.62 and back to exactly 1.05, never above it
  local cc = run(settle(cfg, true, false), cfg, 60, 0.8, true, true, 1)
  local cs = settle(cfg, true, false)
  run(cs, cfg, 60, 0.8, true, true, 1)
  local ccBack = run(cs, cfg, 60, 0.8, true, false, 1)
  check("slide pivot aimed: crouch -> slide -> crouch: 0.62 m and back to 1.05 m, monotonic each way, never above 1.05",
        near(cc[#cc], 0.62, 1e-9) and monotone(ccBack, 1) and near(ccBack[#ccBack], 1.05, 1e-9) and math.max(unpack(ccBack)) <= 1.05 + 1e-9)
  -- (stance blend and crouch behave as before: no slide flag = the 0.25 s crouch of the tests above; slide = nil / false)
  local zc = run(settle(cfg, false, false), cfg, 60, 0.3, true, nil)
  check("slide pivot: the crouch without the slide flag is unchanged (1.05 m in 0.25 s)", near(zc[#zc], 1.05, 1e-6) and near(zc[16], 1.05, 1e-6))
  -- the fix off (slideHeight = crouchHeight): the slide is the crouch again (negative control of the framing test below)
  local c0 = CM.config({ slideHeight = 1.05 })
  local z0 = run(settle(c0, false, false), c0, 60, 0.5, true, true, 1)
  check("slide pivot: slideHeight = crouchHeight -> the slide pivot is the crouch pivot 1.05 m in 0.25 s (old behaviour)",
        near(z0[#z0], 1.05, 1e-6) and near(z0[16], 1.05, 1e-6))
  -- slideHeight above the crouch height (a misconfiguration) and a missing slideHeight fall back to the crouch pivot
  local cUp = CM.config({ slideHeight = 1.4 })
  local zUp = run(settle(cUp, false, false), cUp, 60, 0.5, true, true, 1)
  local cNil = CM.config(); cNil.slideHeight = nil
  local zNil = run(settle(cNil, false, false), cNil, 60, 0.8, true, true, 1)
  check("slide pivot: slideHeight above the crouch height / absent -> crouch pivot (no error)",
        near(zUp[#zUp], 1.05, 1e-6) and near(zNil[#zNil], 1.05, 1e-6))

  -- FREE camera (aimW 0 / nil): the slide keeps the crouch pivot; only the stand -> slide blend is the slower 0.5 s one
  for _, fps in ipairs({ 30, 60, 144 }) do
    local zs = run(settle(cfg, false, false), cfg, fps, 0.8, true, true, 0)
    local zn = run(settle(cfg, false, false), cfg, fps, 0.8, true, true, nil)
    local doneAt = doneAtOf(zs, fps, 1.05)
    check(string.format("slide pivot free (%d fps): stand -> slide: the crouch pivot 1.05 m (not 0.62) in 0.5 s, monotonic, peak <= 1.6 m/s (was 3)", fps),
          monotone(zs, -1) and near(zs[#zs], 1.05, 1e-9) and doneAt and math.abs(doneAt - 0.5) <= 1 / fps + 1e-9 and peak(zs, fps) <= 1.6
          and near(zn[#zn], zs[#zs], 1e-12),
          string.format("done %s peak %.2f end %.3f", tostring(doneAt), peak(zs, fps), zs[#zs]))
    local upS = run(settle(cfg, false, true, 0), cfg, fps, 0.8, false, false, 0)
    check(string.format("slide pivot free (%d fps): slide -> stand: 1.05 -> 1.55 m monotonic, no overshoot, peak <= 1.6 m/s", fps),
          monotone(upS, 1) and near(upS[1], 1.05, 1e-9) and near(upS[#upS], 1.55, 1e-9) and math.max(unpack(upS)) <= 1.55 + 1e-9 and peak(upS, fps) <= 1.6,
          string.format("start %.3f end %.3f peak %.2f", upS[1], upS[#upS], peak(upS, fps)))
    local upC = run(settle(cfg, false, true, 0), cfg, fps, 0.8, true, false, 0) -- stand -> slide -> crouch (the crouch weight was 0)
    -- (review 2026-10-21: the held crouch weight plus the falling slide weight made a +3.9 cm hump here; the exit is one blend now)
    check(string.format("slide pivot free (%d fps): slide -> crouch: stays 1.05 m exactly (no hump, no move)", fps),
          near(upC[1], 1.05, 1e-9) and near(upC[#upC], 1.05, 1e-9) and math.max(unpack(upC)) <= 1.05 + 1e-9
          and math.min(unpack(upC)) >= 1.05 - 1e-9,
          string.format("max %.4f end %.4f min %.4f", math.max(unpack(upC)), upC[#upC], math.min(unpack(upC))))
  end
  local fc = run(settle(cfg, true, false), cfg, 60, 0.8, true, true, 0)
  check("slide pivot free: crouch -> slide: the pivot does not move at all (1.05 m)", math.max(unpack(fc)) - math.min(unpack(fc)) < 1e-9
        and near(fc[1], 1.05, 1e-9))
  -- the aim weight scales the drop: half aim = half way, held
  local half = run(settle(cfg, true, true, 0.5), cfg, 60, 0.2, true, true, 0.5)
  check("slide pivot: aimW 0.5 in the slide -> 0.835 m (halfway between the crouch and the slide pivot)", near(half[#half], 0.835, 1e-9),
        half[#half])
  -- aim enters / ends during the slide (the aim tween 0..1 over 0.25 s): the pivot follows it rate-limited (slideBlend 0.5 s for the
  -- full 0.43 m = 0.86 m/s), no overshoot
  for _, fps in ipairs({ 30, 60, 144 }) do
    local zin = run(settle(cfg, false, true, 0), cfg, fps, 0.6, true, true, function(i) return math.min(1, i / fps / 0.25) end)
    local zout = run(settle(cfg, false, true, 1), cfg, fps, 0.6, true, true, function(i) return math.max(0, 1 - i / fps / 0.25) end)
    check(string.format("slide pivot (%d fps): aim in during the slide 1.05 -> 0.62 m / aim out 0.62 -> 1.05 m, monotonic, peak <= 0.9 m/s", fps),
          monotone(zin, -1) and near(zin[1], 1.05, 1e-9) and near(zin[#zin], 0.62, 1e-9) and math.min(unpack(zin)) >= 0.62 - 1e-9
          and monotone(zout, 1) and near(zout[1], 0.62, 1e-9) and near(zout[#zout], 1.05, 1e-9) and math.max(unpack(zout)) <= 1.05 + 1e-9
          and peak(zin, fps) <= 0.9 and peak(zout, fps) <= 0.9,
          string.format("in %.3f..%.3f peak %.2f / out peak %.2f", zin[1], zin[#zin], peak(zin, fps), peak(zout, fps)))
    -- aim starts together with the slide from standing (recording 03:39:48.4 -> .7): one monotonic fall, peak <= the crouch's 3 m/s
    -- (2.75 m/s; 3.3-3.6 m/s when the aim tween drove the pivot at full speed)
    local zboth = run(settle(cfg, false, false), cfg, fps, 0.8, true, true, function(i) return math.min(1, i / fps / 0.25) end)
    check(string.format("slide pivot (%d fps): aim and slide start together from standing: 1.55 -> 0.62 m monotonic, peak <= 3.05 m/s", fps),
          monotone(zboth, -1) and near(zboth[#zboth], 0.62, 1e-9) and peak(zboth, fps) <= 3.05,
          string.format("end %.3f peak %.2f", zboth[#zboth], peak(zboth, fps)))
  end

  -- aim starting at / after the end of a FREE slide (review 2026-10-21: the aim weight kept tracking in the exit blend, the pivot
  -- of the crouching / standing V dropped 3-10 cm below 1.05 m and came back, one direction change): the exit is one monotonic move
  local function dirChanges(zs)
    local n, last = 0, 0
    for i = 2, #zs do
      local d = zs[i] - zs[i - 1]
      if math.abs(d) > 1e-9 then
        local dir = d > 0 and 1 or -1
        if last ~= 0 and dir ~= last then n = n + 1 end
        last = dir
      end
    end
    return n
  end
  for _, delay in ipairs({ 0.0, 0.1, 0.2 }) do
    local pc = settle(cfg, true, false) -- crouch -> free slide (1 s) -> crouch, the aim tween (0.25 s) starts `delay` s after the slide
    run(pc, cfg, 60, 1.0, true, true, 0)
    local zc = run(pc, cfg, 60, 0.9, true, false, function(i) return math.max(0, math.min(1, (i / 60 - delay) / 0.25)) end)
    local ps = settle(cfg, false, true, 0) -- stand -> free slide -> stand (the crouch weight was 0)
    local zs = run(ps, cfg, 60, 0.9, false, false, function(i) return math.max(0, math.min(1, (i / 60 - delay) / 0.25)) end)
    check(string.format("slide pivot free: aim starts %.1f s after the slide ends: crouch exit stays 1.05 m, stand exit rises monotonically 1.05 -> 1.55", delay),
          math.min(unpack(zc)) >= 1.05 - 1e-6 and math.max(unpack(zc)) <= 1.05 + 1e-6 and dirChanges(zc) == 0
          and monotone(zs, 1) and dirChanges(zs) == 0 and near(zs[#zs], 1.55, 1e-9),
          string.format("crouch %.3f..%.3f dir %d / stand %.3f..%.3f dir %d", math.min(unpack(zc)), math.max(unpack(zc)), dirChanges(zc),
          math.min(unpack(zs)), math.max(unpack(zs)), dirChanges(zs)))
  end

  -- slide interrupted before the blend ended (0.1 / 0.2 / 0.35 s): the exit starts from the pivot of the moment (free: 1.50 / 1.38 / 1.19 m,
  -- aimed: 1.45 / 1.23 / 0.82 m) and runs monotonically to the stance of the new state (review: 2.8 cm hump / a dip and back)
  for _, aimed in ipairs({ 0, 1 }) do
    for _, secs in ipairs({ 0.1, 0.2, 0.35 }) do
      for _, toStand in ipairs({ false, true }) do
        local target = toStand and 1.55 or 1.05
        local pv = settle(cfg, false, false)
        local zin = run(pv, cfg, 60, secs, true, true, aimed)
        local zex = run(pv, cfg, 60, 0.7, not toStand, false, aimed)
        local from = zin[#zin]
        local dir = target > from and 1 or -1
        local lo, hi = math.min(from, target), math.max(from, target)
        check(string.format("slide pivot (%s, interrupted after %.2f s) -> %s: one monotonic blend from %.3f m to %.2f m, no hump, no dip",
              aimed == 1 and "aimed" or "free", secs, toStand and "stand" or "crouch", from, target),
              monotone(zex, dir) and dirChanges(zex) == 0 and near(zex[#zex], target, 1e-9)
              and math.min(unpack(zex)) >= lo - 1e-9 and math.max(unpack(zex)) <= hi + 1e-9 and peak(zex, 60) <= 3.05,
              string.format("%.3f -> %.3f (min %.3f max %.3f) dir %d peak %.2f", from, zex[#zex], math.min(unpack(zex)),
              math.max(unpack(zex)), dirChanges(zex), peak(zex, 60)))
      end
    end
  end
  -- crouch <-> stand during the exit blend and a new slide during it: a new blend from the pivot of the moment, no jump
  do
    local function stepsOk(zs, fps, vmax)
      for i = 2, #zs do if math.abs(zs[i] - zs[i - 1]) * fps > vmax then return false, (zs[i] - zs[i - 1]) * fps end end
      return true
    end
    for _, aimed in ipairs({ 0, 1 }) do
      local pv = settle(cfg, false, false)
      local zs = run(pv, cfg, 60, 0.8, true, true, aimed)             -- stand -> slide
      for _, z in ipairs(run(pv, cfg, 60, 0.2, true, false, aimed)) do zs[#zs + 1] = z end   -- exit to the crouch (0.2 s of 0.5)
      for _, z in ipairs(run(pv, cfg, 60, 0.2, false, false, aimed)) do zs[#zs + 1] = z end  -- stood up during the exit
      for _, z in ipairs(run(pv, cfg, 60, 0.15, true, true, aimed)) do zs[#zs + 1] = z end   -- slid again
      for _, z in ipairs(run(pv, cfg, 60, 1.0, false, false, aimed)) do zs[#zs + 1] = z end  -- stood up
      local ok, v = stepsOk(zs, 60, 3.05)
      check(string.format("slide pivot (%s): slide -> crouch -> stand -> slide -> stand within the blends: no jump (every step <= 3.05 m/s), ends 1.55 m",
            aimed == 1 and "aimed" or "free"), ok and near(zs[#zs], 1.55, 1e-9), tostring(v))
    end
  end
  -- random crouch / slide / aim toggles at 30 / 60 / 144 fps: no pivot speed above ~3.3 m/s (the crouch blend's own peak is 3.0; the
  -- earlier held-crouch-weight scheme reached 4.05 m/s)
  do
    local seed = 12345
    local function rnd() seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648 end
    local worst = 0
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _ = 1, 25 do
        local pv, crouch, slide, aim, prev = CM.newPivot(), false, false, 0, nil
        for _ = 1, fps * 40 do
          if rnd() < 0.01 then crouch = not crouch end
          if rnd() < 0.012 then slide = not slide end
          if rnd() < 0.02 then aim = rnd() < 0.5 and 0 or rnd() end
          local z = CM.pivotStep(pv, cfg, 1 / fps, V(0, 0, 0), nil, crouch or slide, slide, aim).z
          if prev then worst = math.max(worst, math.abs(z - prev) * fps) end
          prev = z
        end
      end
    end
    check("slide pivot: random crouch / slide / aim toggles (3 frame rates x 25 x 40 s): no pivot speed above 3.3 m/s", worst <= 3.3,
          string.format("%.2f m/s", worst))
  end
  -- no slide at all: the stance blend is the earlier 0.25 s smoothstep, bit for bit (crouch toggles at random times, aim ignored)
  do
    local seed = 777
    local function rnd() seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648 end
    local pv, w, crouch, worst = CM.newPivot(), 0, false, 0
    for _ = 1, 60 * 90 do
      if rnd() < 0.01 then crouch = not crouch end
      local step = 1 / 60 / cfg.stanceBlend
      if crouch then w = math.min(1, w + step) else w = math.max(0, w - step) end
      local want = cfg.standHeight + (cfg.crouchHeight - cfg.standHeight) * CM.smoothstep(w)
      local z = CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 0), nil, crouch, nil, rnd()).z
      worst = math.max(worst, math.abs(z - want))
    end
    check("slide pivot: without a slide the pivot is the 0.25 s smoothstep stance blend, exactly (aim weight ignored)", worst < 1e-12, worst)
  end

  -- framing (the aim setup: profile 2 FOV 55, aimDistance 1.5 / side 0.5 = the 1.58 m boom of the log, framing limit 0.45,
  -- look pitch -1.7 deg as in the log at v22): V's head (FK of the final pose: z / y behind the root) within 0.8 of half the
  -- vertical FOV from the screen centre in every stance; stand and crouch were fine, the slide was ~29 deg low (FOV half 27.5)
  local P = { name = "P2", distance = 2.75, height = -0.35, side = 0.5, fov = 55, sens = 1 }
  local heads = { stand = V(0, -0.04, 1.64), crouch = V(0, 0.03, 1.06), slide = V(0, -0.62, 0.64) }
  -- flat floor at z = 0 for the boom's collision ray (the reviewer's offline floor model)
  local function floorRay(from, to)
    if to.z >= 0 or from.z <= 0 then return nil end
    return CM.dist(from, to) * from.z / (from.z - to.z)
  end
  local function headAngle(c, stance, pitch, aimed, raycast)
    local S = CM.newSolver(c)
    local r
    for _ = 1, 120 do
      r = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = pitch, dyaw = 0,
        dpitch = 0, moving = true, profile = aimed and CM.aimProfile(P, c, nil) or P, side = 1, w = 1, aim = aimed,
        aimW = aimed and 1 or 0, crouch = stance ~= "stand", slide = stance == "slide", raycast = raycast })
    end
    local v = CM.sub(heads[stance], r.world)
    local f = CM.dot(v, CM.qrot(r.q, V(0, 1, 0)))
    return math.deg(math.atan2(CM.dot(v, CM.qrot(r.q, V(0, 0, 1))), f)), r.fov, r
  end
  local cu = CM.config({ aimDistance = 1.5, aimSide = 0.5, aimBoomFrame = 0.45 })
  local bad, txt = {}, {}
  for _, st in ipairs({ "stand", "crouch", "slide" }) do
    local a, fov, r = headAngle(cu, st, -1.7, true)
    txt[#txt + 1] = string.format("%s %.1f deg (pivot %.2f, arm %.2f)", st, a, r.pivot.z, r.arm)
    if math.abs(a) > 0.8 * fov / 2 then bad[#bad + 1] = st end
  end
  check("slide pivot: aim framing: V's head within 0.8 x half FOV in stand / crouch / slide", #bad == 0, table.concat(txt, "; "))
  local aOld = headAngle(CM.config({ aimDistance = 1.5, aimSide = 0.5, aimBoomFrame = 0.45, slideHeight = 1.05 }), "slide", -1.7, true)
  check("slide pivot: negative control (slideHeight = crouchHeight): the sliding head is out of the aim frame (> 0.8 x half FOV, 27 deg low)",
        math.abs(aOld) > 0.8 * 55 / 2 and aOld < -25, string.format("%.1f deg", aOld))
  -- aimed slide over the whole look-pitch range (WITH the flat floor: without it the camera of the +35 / +40 deg looks stood under
  -- the floor, review 2026-10-21), and the free slide framing (the crouch pivot, as before the fix)
  local worstAim, worstFree = 0, 0
  for _, pitch in ipairs({ -30, -1.7, 20, 30, 35, 40 }) do
    worstAim = math.max(worstAim, math.abs((headAngle(cu, "slide", pitch, true, floorRay))))
  end
  for _, pitch in ipairs({ -30, -1.7, 20, 40 }) do
    worstFree = math.max(worstFree, math.abs((headAngle(cu, "slide", pitch, false))))
  end
  check("slide pivot: head within 0.8 x half FOV (22 deg) at look pitch -30 .. +40 deg: aimed slide (flat floor, up to +40) and free slide",
        worstAim <= 0.8 * 27.5 and worstFree <= 0.8 * 27.5, string.format("aimed %.1f free %.1f deg", worstAim, worstFree))
  -- the aim settings (1.5 m / 0.5 / boomFrame 0.45, pitch scale 0.35) over the 0.62 m pivot: looking up the boom end must stay
  -- over the floor, not be pulled in by the floor ray (was 1.45 / 1.13 / 0.93 m at +30 / +35 / +40, the head 0.55 m away)
  local armTxt, armOk = {}, true
  for _, pitch in ipairs({ 30, 35, 40 }) do
    local _, _, r = headAngle(cu, "slide", pitch, true, floorRay)
    local _, _, rc = headAngle(cu, "crouch", pitch, true, floorRay)
    local d = CM.dist(r.world, heads.slide)
    armTxt[#armTxt + 1] = string.format("+%d: arm %.2f z %.2f head %.2f m (crouch arm %.2f)", pitch, r.arm, r.world.z, d, rc.arm)
    if r.arm < 1.5 or r.world.z < 0.28 or d < 0.95 or rc.arm < 1.5 then armOk = false end
  end
  check("slide pivot aimed: user's aim settings, flat floor, look +30 / +35 / +40: full arm (>= 1.5 m), camera over the floor (>= 0.28 m), head >= 0.95 m",
        armOk, table.concat(armTxt, "; "))
  -- the limit follows the aim weight of the slide pivot (0.25 / 0.5 / 0.75 / 1: the pivot 0.94 / 0.84 / 0.73 / 0.62 m): the boom pitch
  -- it takes off grows with it, at 0.25 well under a quarter of the full effect (no jump when the aim starts in a slide)
  do
    local function boomAt(c, aw)
      local S = CM.newSolver(c)
      local r
      for _ = 1, 150 do
        r = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = 40, dyaw = 0,
          dpitch = 0, moving = true, profile = CM.aimProfile(P, c, nil), side = 1, w = 1, aim = true, aimW = aw, crouch = true, slide = true })
      end
      return r.boomPitch
    end
    local cOff = CM.config({ aimDistance = 1.5, aimSide = 0.5, aimBoomFrame = 0.45, slideArmLimit = false })
    local eff, prev, grows = {}, 0, true
    for i, aw in ipairs({ 0.25, 0.5, 0.75, 1 }) do
      eff[i] = boomAt(cOff, aw) - boomAt(cu, aw)
      if eff[i] < prev - 1e-9 then grows = false end
      prev = eff[i]
    end
    check("slide pivot aimed: the boom pitch limit grows with the aim weight (0.25 -> 1) and is small at 0.25 (< a quarter of the full effect)",
          grows and eff[1] > 0 and eff[4] > 10 and eff[1] < 0.25 * eff[4],
          string.format("%.2f %.2f %.2f %.2f deg", eff[1], eff[2], eff[3], eff[4]))
  end
  -- Continuity of the CAMERA (r.world, r.boomPitch), not only of the pivot (review 2026-10-21: the arm limit was weighted with the
  -- blend parameter pv.slideW, which restarts with each blend - the camera jumped 0.07-0.46 m in one frame at look +40 while the pivot
  -- stayed continuous). Aimed (user's aim settings, flat floor, look +40 = the limit is working at full strength), 60 fps; the
  -- steps are measured from t = fromT on (the pose settled; the entry transient of a slide is not part of it)
  local function camSteps(fn, fromT, secs, pitch)
    local S = CM.newSolver(cu)
    local prevW, prevB, maxStep, maxBoom, prevRel, maxRel = nil, nil, 0, 0, nil, 0
    for i = 1, math.floor((secs or 5) * 60) do
      local t = i / 60
      local crouch, slide, pz = fn(t)
      local r = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, pz or 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = pitch or 40,
        dyaw = 0, dpitch = 0, moving = true, profile = CM.aimProfile(P, cu, nil), side = 1, w = 1, aim = true, aimW = 1, crouch = crouch,
        slide = slide, raycast = floorRay })
      local rel = CM.sub(r.world, r.pivot) -- the boom itself (the pivot's own motion taken out)
      if prevW ~= nil and t > fromT then
        maxStep = math.max(maxStep, CM.dist(r.world, prevW))
        maxBoom = math.max(maxBoom, math.abs(r.boomPitch - prevB))
        maxRel = math.max(maxRel, CM.dist(rel, prevRel))
      end
      prevW, prevB, prevRel = r.world, r.boomPitch, rel
    end
    return maxStep, maxBoom, maxRel
  end
  do
    local cases = {
      { "control: slide, then the normal exit to the crouch",
        function(t) return true, t >= 2.0 and t < 3.0 end },
      { "the slide flag drops for ONE frame in the full slide",
        function(t) return true, t >= 2.0 and not (t > 3.0 and t < 3.02) end },
      { "a new slide 0.25 s after the exit started (re-entry during the exit)",
        function(t) return true, (t >= 2.0 and t < 3.0) or t >= 3.25 end },
    }
    for _, c in ipairs(cases) do
      local step, boom = camSteps(c[2], 2.8, 5)
      check(string.format("slide camera: aimed, look +40, flat floor: %s: no camera jump (step <= 0.05 m, boom pitch step <= 2.5 deg)", c[1]),
            step <= 0.05 and boom <= 2.5, string.format("%.3f m / %.2f deg", step, boom))
    end
  end

  -- the same for a slide interrupted before its blend ended (the exit starts from the pivot of the moment) and a stance change in the
  -- exit (review: the weight jumped to ~0.97 at the start of such an exit, the boom 7-9 cm in one frame at +40, 3.2 deg of boom pitch)
  do
    local cases = {
      { "slide interrupted after 0.3 s -> crouch", function(t) return true, t >= 2.0 and t < 2.3 end },
      { "slide interrupted after 0.2 s -> crouch", function(t) return true, t >= 2.0 and t < 2.2 end },
      { "slide 1 s, crouch for 0.3 s, then stand up in the exit", function(t) return t < 3.3, t >= 2.0 and t < 3.0 end },
    }
    for _, pitch in ipairs({ 40, 35 }) do
      for _, c in ipairs(cases) do
        local _, boom, rel = camSteps(c[2], 2.15, 5, pitch)
        check(string.format("slide camera: aimed, look +%d, flat floor: %s: the boom does not jump (step <= 0.04 m, boom pitch step <= 1.5 deg)", pitch, c[1]),
              rel <= 0.04 and boom <= 1.5, string.format("%.3f m / %.2f deg", rel, boom))
      end
    end
  end
  -- entering the aimed slide from the crouch / from standing and leaving it: the limit grows and fades with the pivot, no step of the boom
  do
    -- (from standing the pivot falls 2.8 m/s through the crouch height and the limit takes hold at that speed: 0.045 m / 1.7 deg per frame)
    local cases = {
      { "crouch -> slide (entry) and the exit after 1 s", function(t) return true, t >= 2.0 and t < 3.0 end, 0.04, 1.5 },
      { "standing -> slide (entry) and the exit to standing", function(t) return t >= 2.0 and t < 3.0, t >= 2.0 and t < 3.0 end, 0.06, 2.2 },
    }
    for _, c in ipairs(cases) do
      local _, boom, rel = camSteps(c[2], 1.9, 5, 40)
      check(string.format("slide camera: aimed, look +40, flat floor: %s: the boom does not jump (step <= %.2f m, boom pitch step <= %.1f deg)", c[1], c[3], c[4]),
            rel <= c[3] and boom <= c[4], string.format("%.3f m / %.2f deg", rel, boom))
    end
  end
  -- the floor of the limit is the pivot's smoothed base (pivotSmooth), not the raw z of the player: a step / curb / capsule step-off
  -- of +-0.15 m in one frame must not move the camera by more than the pivot (review: 0.150 m in one frame, the pivot 0.005 m)
  do
    -- the street stays at z = 0 behind the camera; the capsule steps up onto a curb / off it (the pivot's base is smoothed, the raw z jumps)
    for _, j in ipairs({ { 0, 0.15 }, { 0.15, 0 } }) do
      local step = camSteps(function(t) return true, t >= 2.0, t >= 3.0 and j[2] or j[1] end, 2.8, 5, 40)
      check(string.format("slide camera: aimed, look +40: the capsule z jumps %+.2f m in one frame (street behind the camera unchanged): the camera follows the smoothed pivot (step <= 0.04 m)", j[2] - j[1]),
            step <= 0.04, string.format("%.3f m", step))
    end
  end
  -- stand / crouch aim and the FREE slide: the arm limit is not applied (no slide weight / no aim weight on the pivot): bit-identical
  -- with camera.slideArmLimit on and off; the aimed slide differs (the negative control)
  do
    local cOff = CM.config({ aimDistance = 1.5, aimSide = 0.5, aimBoomFrame = 0.45, slideArmLimit = false })
    local same, diff = true, false
    for _, st in ipairs({ "stand", "crouch", "slide" }) do
      for _, aimed in ipairs({ true, false }) do
        for _, pitch in ipairs({ -30, 0, 30, 40, 60, 80 }) do
          local _, _, r1 = headAngle(cu, st, pitch, aimed, floorRay)
          local _, _, r2 = headAngle(cOff, st, pitch, aimed, floorRay)
          local equal = r1.world.z == r2.world.z and r1.world.x == r2.world.x and r1.arm == r2.arm and r1.boomPitch == r2.boomPitch
          if st == "slide" and aimed then
            if not equal then diff = true end
          elseif not equal then same = false end
        end
      end
    end
    check("slide pivot: stand / crouch aim, free camera and the free slide are bit-identical with camera.slideArmLimit on / off; the aimed slide differs",
          same and diff, string.format("same %s diff %s", tostring(same), tostring(diff)))
  end
  -- free slide on the flat floor looking up (the review: pivot 0.62 m -> the camera 0.35 m over the floor, the floor ray pushed it
  -- in to 0.85 m from V's head at +10 deg, 0.47 m at +20): the free boom keeps its length / distance from the head
  local _, _, r10 = headAngle(cu, "slide", 10, false, floorRay)
  local _, _, r20 = headAngle(cu, "slide", 20, false, floorRay)
  local d10, d20 = CM.dist(r10.world, heads.slide), CM.dist(r20.world, heads.slide)
  check("slide pivot free: flat floor, looking up 10 / 20 deg: boom arm >= 2.7 m / >= 1.5 m (not pushed in to 1.40 / 0.88), head >= 2.0 m / >= 0.9 m away",
        r10.arm >= 2.7 and r20.arm >= 1.5 and d10 >= 2.0 and d20 >= 0.9,
        string.format("arm %.2f / %.2f head %.2f / %.2f pivot %.2f", r10.arm, r20.arm, d10, d20, r10.pivot.z))
  -- negative control: the low pivot in the free camera (what an unscaled slide pivot gave) is pushed in by the floor
  local cLow = CM.config({ aimDistance = 1.5, aimSide = 0.5, aimBoomFrame = 0.45, crouchHeight = 0.62, slideHeight = 0.62 })
  local _, _, l10 = headAngle(cLow, "slide", 10, false, floorRay)
  check("slide pivot free: negative control (pivot 0.62 m in the free camera): the floor ray shortens the arm at +10 deg (< 1.6 m)",
        l10.arm < 1.6 and l10.pivot.z < 0.7, string.format("arm %.2f pivot %.2f", l10.arm, l10.pivot.z))
end

-- ================================================================ collision (mocked world)
-- world: planes {axis, value, side}: a ray hits when it crosses the plane; poles: vertical cylinders (x, y, r)
local function makeWorld(def)
  local W = { planes = def.planes or {}, poles = def.poles or {}, casts = 0 }
  function W.ray(from, to)
    W.casts = W.casts + 1
    local best = nil
    local d = CM.sub(to, from)
    local len = CM.len(d)
    for _, pl in ipairs(W.planes) do
      local a, b = from[pl.axis] - pl.value, to[pl.axis] - pl.value
      if (a > 0) ~= (b > 0) then
        local t = a / (a - b)
        local dist = t * len
        if not best or dist < best then best = dist end
      end
    end
    for _, po in ipairs(W.poles) do
      -- 2D segment vs circle
      local fx, fy = from.x - po.x, from.y - po.y
      local A = d.x * d.x + d.y * d.y
      if A > 1e-12 then
        local B = 2 * (fx * d.x + fy * d.y)
        local C = fx * fx + fy * fy - po.r * po.r
        local disc = B * B - 4 * A * C
        if disc >= 0 then
          local t = (-B - math.sqrt(disc)) / (2 * A)
          if t >= 0 and t <= 1 then
            local dist = t * len
            if not best or dist < best then best = dist end
          end
        end
      end
    end
    return best
  end
  return W
end

do
  local cfg = CM.config()
  local hits = { nil, 1.0, nil, 0.7, nil }
  local allowed, n = CM.collisionLength(hits, 5, 2.5, cfg)
  check("collisionLength: nearest hit minus margin", near(allowed, 0.7 - cfg.margin) and n == 2, allowed)
  local a2 = CM.collisionLength({ 0.05 }, 1, 2.5, cfg)
  check("collisionLength: hard minimum", near(a2, cfg.hardMin), a2)
  local a3 = CM.collisionLength({}, 5, 2.5, cfg)
  check("collisionLength: no hit -> full length", near(a3, 2.5))
  local rays = CM.collisionRays(V(0, 0, 1.6), V(0, -2, 1.6), 0, 0, 0.2, true)
  check("collisionRays: center + 4 edge rays offset by the radius", #rays == 5 and vnear(rays[2].from, V(0.2, 0, 1.6))
        and vnear(rays[4].to, V(0, -2, 1.8)))
  local cs = CM.newCollision()
  local plans = {}
  for i = 1, 6 do plans[i] = CM.collisionPlan(cs, cfg); CM.collisionSmooth(cs, cfg, 1 / 60, 2, 0) end
  check("collisionPlan: full set every 3rd frame in open space", (not plans[1]) and (not plans[2]) and plans[3] and plans[6])
  CM.collisionSmooth(cs, cfg, 1 / 60, 1.0, 1)
  local f1, f2 = CM.collisionPlan(cs, cfg), CM.collisionPlan(cs, cfg)
  check("collisionPlan: full set every frame shortly after a hit", f1 and f2)
  -- closer at once, back smoothly, dead zone
  local c2 = CM.newCollision()
  CM.collisionSmooth(c2, cfg, 1 / 60, 2.0, 0)
  local l = CM.collisionSmooth(c2, cfg, 1 / 60, 0.8, 1)
  check("collision: closer immediately", near(l, 0.8))
  local prev, mono, t = l, true, 0
  while c2.len < 2.0 - cfg.deadzone - 1e-6 and t < 3 do
    local x = CM.collisionSmooth(c2, cfg, 1 / 60, 2.0, 0)
    if x < prev - 1e-12 then mono = false end
    prev = x
    t = t + 1 / 60
  end
  check("collision: back smoothly (monotonic, within ~2x return time)", mono and t <= 2.5 * cfg.returnTime + 0.05, t)
  local before = c2.len
  CM.collisionSmooth(c2, cfg, 1 / 60, before + cfg.deadzone * 0.5, 0)
  check("collision: dead zone ignores tiny changes", near(c2.len, before))
  -- 2026-11-01: inRate limits the pull-in (vehicle mount hold); nil / 0 keeps the instant pull-in
  local c3 = CM.newCollision()
  CM.collisionSmooth(c3, cfg, 1 / 60, 2.95, 0)
  local r1 = CM.collisionSmooth(c3, cfg, 1 / 60, 1.80, 1, 12)
  check("collision inRate: one frame pulls in at most rate*dt (12 m/s -> 0.2 m)", near(r1, 2.95 - 0.2), r1)
  local n = 1
  while c3.len > 1.80 + 1e-9 and n < 60 do CM.collisionSmooth(c3, cfg, 1 / 60, 1.80, 1, 12); n = n + 1 end
  check("collision inRate: reaches the allowed length, never below (1.15 m in 6 frames)", near(c3.len, 1.80) and n == 6, n)
  local c4 = CM.newCollision()
  CM.collisionSmooth(c4, cfg, 1 / 60, 2.95, 0)
  check("collision inRate 0: instant as before", near(CM.collisionSmooth(c4, cfg, 1 / 60, 1.80, 1, 0), 1.80))
  -- 2026-11-04 inDelay (game log 02-01-16, ride 7: a door swinging open crossed the boom for ~0.25 s, 2.93 -> 1.27 -> 2.83 m): the logged series replayed at
  -- several frame rates. allowed = 1.20 m (hits = 1) from 0.33 s to 0.58 s of a standing boom of 2.93 m.
  local DELAY = cfg.vehHoldColDelay
  local function replay(fps, from, to, delay, flicker)
    local cs, dt = CM.newCollision(), 1 / fps
    local t, lo, maxV, prev = 0, 1e9, 0, nil
    CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 12, delay)
    while t < 2.5 do
      t = t + dt
      local hit = t >= from and t <= to
      if hit and flicker then hit = (math.floor(t * fps + 0.5) % 2) == 0 end
      local len = CM.collisionSmooth(cs, cfg, dt, hit and 1.20 or 2.93, hit and 1 or 0, 12, delay)
      lo = math.min(lo, len)
      if prev then maxV = math.max(maxV, (prev - len) / dt) end
      prev = len
    end
    return { low = lo, maxV = maxV, final = cs.len }
  end
  check("collision inDelay cfg: vehHoldColDelay 0.35 s, vehHoldColAccel 30 m/s2", near(DELAY, 0.35) and near(cfg.vehHoldColAccel, 30))
  for _, fps in ipairs({ 30, 60, 144 }) do
    local a, b = replay(fps, 0.33, 0.58, DELAY), replay(fps, 0.33, 0.58, nil)
    check(string.format("collision inDelay %d fps: a 0.25 s pulse of an occluder does not move the boom (lowest %.2f m); without the delay (previous) it comes in to %.2f m at %.1f m/s", fps,
          a.low, b.low, b.maxV), near(a.low, 2.93) and b.low < 1.8 and b.maxV > 11, string.format("%.2f vs %.2f", a.low, b.low))
    local f = replay(fps, 0.33, 1.5, DELAY, true)
    check(string.format("collision inDelay %d fps: a flickering surface (every other frame, 1.2 s) does not pull the boom in either (lowest %.2f m)", fps, f.low), f.low > 2.5, f.low)
    local c = replay(fps, 0.33, 2.5, DELAY)
    check(string.format("collision inDelay %d fps: a surface that stays pulls the boom to 1.20 m (lowest %.2f), after the delay, at most 12 m/s (%.1f)", fps, c.low, c.maxV),
          near(c.low, 1.20, 1e-6) and c.maxV <= 12 + 1e-6, string.format("%.2f %.1f", c.low, c.maxV))
  end
  do -- the pull-in of a surface that stays starts after the delay and ramps up (no velocity step)
    local cs, dt, t, firstMove, v1 = CM.newCollision(), 1 / 60, 0, nil, nil
    CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 12, DELAY)
    local prev = cs.len
    while t < 1.0 do
      t = t + dt
      local len = CM.collisionSmooth(cs, cfg, dt, 1.20, 1, 12, DELAY)
      if firstMove == nil and len < prev - 1e-9 then firstMove, v1 = t, (prev - len) / dt end
      prev = len
    end
    check(string.format("collision inDelay: the first pull-in comes %.2f s after the surface appeared (>= 0.35) and starts at %.2f m/s (<= 30 m/s2 * dt = %.2f)", firstMove or -1, v1 or -1, 30 * dt),
          firstMove ~= nil and firstMove >= 0.35 - 1e-9 and v1 <= 30 * dt + 1e-9)
  end
  -- review r4 (medium): the press timer must not drain while the occluder is still there. The boom catches up with a surface that stays (1.20 m), sits there
  -- 1.5 s, then the surface comes closer (1.00 m, e.g. the boom turning into the car body): that is no new door pulse, the pull-in goes on at once
  for _, fps in ipairs({ 30, 60, 144 }) do
    local cs, dt = CM.newCollision(), 1 / fps
    CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 12, DELAY)
    for _ = 1, math.floor(2.0 * fps) do CM.collisionSmooth(cs, cfg, dt, 1.20, 1, 12, DELAY) end -- caught up (about 0.7 s) and sitting there
    local atRest, press = cs.len, cs.press
    local after = CM.collisionSmooth(cs, cfg, dt, 1.00, 1, 12, DELAY)
    check(string.format("collision inDelay %d fps: an occluder that stays keeps the press timer (%.2f s) and a closer surface pulls in at once (%.3f -> %.3f m in one frame)", fps,
          press, atRest, after), near(atRest, 1.20, 1e-6) and press >= DELAY - 1e-9 and after < atRest - 1e-6, string.format("%.3f %.3f %.3f", press, atRest, after))
    -- the hit clears: the timer empties (twice the fill rate), the next pulse waits the full delay again
    for _ = 1, math.floor(1.0 * fps) do CM.collisionSmooth(cs, cfg, dt, 2.93, 0, 12, DELAY) end
    local lenBack, pressGone = cs.len, cs.press
    local t, firstIn = 0, nil
    while t < 0.6 do
      t = t + dt
      local before = cs.len
      local len = CM.collisionSmooth(cs, cfg, dt, 1.20, 1, 12, DELAY)
      if firstIn == nil and len < before - 1e-9 then firstIn = t end
    end
    check(string.format("collision inDelay %d fps: once the hit cleared the timer is empty (%.2f s) and a new pulse waits the delay again (first pull-in after %.2f s)", fps, pressGone, firstIn or -1),
          pressGone < 1e-9 and lenBack > 2.5 and firstIn ~= nil and firstIn >= DELAY - 1e-9, string.format("%s %s", tostring(pressGone), tostring(firstIn)))
  end
  do -- without the delay (every non-hold use of the boom) nothing is kept: press and the pull-in speed stay 0
    local cs, dt = CM.newCollision(), 1 / 60
    CM.collisionSmooth(cs, cfg, dt, 2.93, 0, nil, nil)
    for _ = 1, 30 do CM.collisionSmooth(cs, cfg, dt, 1.20, 1, nil, nil) end
    for _ = 1, 30 do CM.collisionSmooth(cs, cfg, dt, 1.20, 1, 12, nil) end
    check("collision inDelay: without a delay the press timer and the pull-in speed stay 0 (normal behaviour)", cs.press == 0 or cs.press == nil, tostring(cs.press))
  end
end

-- 2026-11-01 NP-3: body clearance floor (minArm), never past a surface, inside flag
do
  local cfg = CM.config()
  local clr = CM.clearance(cfg)
  check("NP-3 clearance = max(hardMin, minArm) = 0.30 m by default", near(clr, 0.30) and near(CM.clearance(CM.config({ minArm = 0 })), cfg.hardMin))
  -- hit at 0.40 m: margin would give 0.28 m (inside the head); the surface is farther than clearance + gap -> 0.30 m
  local a1, n1, in1 = CM.collisionLength({ 0.40 }, 1, 2.5, cfg)
  check("NP-3 collisionLength: clearance floor (hit 0.40 -> 0.30, not 0.28), not inside", near(a1, clr) and n1 == 1 and in1 == false, a1)
  -- wall 0.25 m behind the pivot: never behind the surface (hit - clearanceWallGap), above the old 0.13, inside
  local a2, _, in2 = CM.collisionLength({ nil, 0.25 }, 2, 2.5, cfg)
  check("NP-3 collisionLength: near wall caps the floor at hit - gap (0.19), never past the surface, inside",
    near(a2, 0.25 - cfg.clearanceWallGap) and a2 <= 0.25 and a2 > 0.25 - cfg.margin and in2 == true, a2)
  -- surfaces closer than hardMin + gap: the old hard minimum
  local a3, _, in3 = CM.collisionLength({ 0.05 }, 1, 2.5, cfg)
  check("NP-3 collisionLength: very near surface -> hardMin (as before), inside", near(a3, cfg.hardMin) and in3 == true, a3)
  -- far hits and no hits: unchanged, not inside
  local a4, _, in4 = CM.collisionLength({ 0.9, 0.7 }, 2, 2.5, cfg)
  local a5, n5, in5 = CM.collisionLength({}, 5, 2.5, cfg)
  check("NP-3 collisionLength: far hit (0.7 - margin) and no hit unchanged, not inside",
    near(a4, 0.7 - cfg.margin) and in4 == false and near(a5, 2.5) and n5 == 0 and in5 == false)
  -- a boom shorter than the clearance (the default eye-start blend): no floor above the boom (collision smoothing as before), inside
  local a6, _, in6 = CM.collisionLength({ 0.9 }, 1, 0.2, cfg)
  check("NP-3 collisionLength: boom shorter than the clearance stays at its length, inside", near(a6, 0.2) and in6 == true, a6)
  -- minArm <= hardMin: exactly the previous formula over a sweep of hits
  local old = CM.config({ minArm = 0 })
  local same, worst = true, nil
  for d = 0.0, 3.0, 0.01 do
    local got = CM.collisionLength({ d, nil, d + 0.3 }, 3, 2.5, old)
    local want = CM.clamp(math.min(2.5, d - old.margin, d + 0.3 - old.margin), old.hardMin, 2.5) -- the previous return value
    if got ~= want then same, worst = false, d end
  end
  check("NP-3 collisionLength: minArm = 0 -> bit-identical to the previous floor (hardMin)", same, worst)
end

-- ================================================================ transitions
local function trRun(T, cfg, fps, secs, env, log)
  local cmdsAll = {}
  for _ = 1, math.floor(fps * secs + 0.5) do
    local c = CM.trStep(T, cfg, 1 / fps, env)
    for _, x in ipairs(c) do cmdsAll[#cmdsAll + 1] = x; if log then log[#log + 1] = { x, T.state } end end
  end
  return cmdsAll
end
do
  local cfg = CM.config()
  local env = { enabled = true, canEnter = true, repReady = true }
  local T = CM.newTransition(true)
  CM.trRequest(T, true)
  local c1 = CM.trStep(T, cfg, 1 / 60, env)
  check("enter: representation first (rep_on), camera waits", c1[1] == "rep_on" and T.state == "rep_wait")
  local waited, cOn = 0, {}
  repeat cOn = CM.trStep(T, cfg, 1 / 60, env); waited = waited + 1 until cOn[1] == "cam_on" or waited > 200
  check("enter: cam_on after the representation wait", cOn[1] == "cam_on" and T.state == "entering"
        and near(waited / 60, cfg.repWait, 1 / 60 + 1e-9), waited)
  local frames = 0
  while T.state ~= "tpp" and frames < 1000 do CM.trStep(T, cfg, 1 / 60, env); frames = frames + 1 end
  check("enter: boom grows to 1 in enterTime", near(frames / 60, cfg.enterTime, 1 / 60 + 1e-9) and T.w == 1, frames)
  -- 0.1.3 aim return: enterOverride for one entry only
  do
    local TA = CM.newTransition(false)
    CM.trRequest(TA, true)
    TA.enterOverride = 0.35
    local n = 0
    repeat CM.trStep(TA, cfg, 1 / 60, env); n = n + 1 until TA.state == "tpp" or n > 1000
    local cleared = TA.enterOverride == nil
    CM.trRequest(TA, false)
    trRun(TA, cfg, 60, 1.0, env)
    CM.trRequest(TA, true)
    local n2 = 0
    repeat CM.trStep(TA, cfg, 1 / 60, env); n2 = n2 + 1 until TA.state == "tpp" or n2 > 1000
    check("enter override (aim return 0.35 s) used once, next entry back to enterTime", math.abs(n / 60 - 0.35) <= 2 / 60 + 1e-9
          and cleared and math.abs(n2 / 60 - cfg.enterTime) <= 2 / 60 + 1e-9, string.format("%d / %d frames", n, n2))
  end
  CM.trRequest(T, false)
  local log = {}
  trRun(T, cfg, 60, 1.0, env, log)
  check("exit: fpp_on, rep_off, then cam_off one frame later", #log == 3 and log[1][1] == "fpp_on" and log[2][1] == "rep_off"
        and log[3][1] == "cam_off" and T.state == "fpp" and T.w == 0, #log)
  -- frame-rate independence of the enter time
  local function enterTime(fps)
    local TT = CM.newTransition(false)
    CM.trRequest(TT, true)
    local n = 0
    repeat CM.trStep(TT, cfg, 1 / fps, env); n = n + 1 until TT.state == "tpp" or n > 5000
    return n / fps
  end
  check("enter time equal at 30 / 60 / 144 fps (within 2 frames)", math.abs(enterTime(30) - enterTime(144)) <= 2 / 30 + 1e-9,
        string.format("%.3f %.3f", enterTime(30), enterTime(144)))
  -- blocked cut and automatic return
  local T2 = CM.newTransition(true)
  CM.trRequest(T2, true)
  trRun(T2, cfg, 60, 1.0, env)
  local c2 = CM.trStep(T2, cfg, 1 / 60, { enabled = true, canEnter = true, repReady = true, blocked = "vehicle", cut = true })
  check("blocked (vehicle): cut in the same frame", c2[1] == "fpp_on" and c2[2] == "rep_off" and T2.w == 0)
  trRun(T2, cfg, 60, 0.5, { enabled = true, canEnter = true, repReady = true, blocked = "vehicle", cut = true })
  check("blocked: stays first person", T2.state == "fpp" and T2.want == true)
  trRun(T2, cfg, 60, 1.0, env)
  check("unblocked: back to third person (want kept)", T2.state == "tpp")
  -- disabled (setting)
  local T3 = CM.newTransition(true)
  CM.trRequest(T3, true)
  local c3 = trRun(T3, cfg, 60, 1.0, { enabled = false, canEnter = true, repReady = true })
  check("disabled (setting): never enters", #c3 == 0 and T3.state == "fpp")
  local T4 = CM.newTransition(true)
  CM.trRequest(T4, true)
  trRun(T4, cfg, 60, 1.0, env)
  local c4 = CM.trStep(T4, cfg, 1 / 60, { enabled = false, canEnter = true })
  check("disabled while active: immediate cut", c4[1] == "fpp_on" and T4.state == "settle")
  -- quick FPP
  local T5 = CM.newTransition(true)
  CM.trRequest(T5, true)
  trRun(T5, cfg, 60, 1.0, env)
  CM.trQuick(T5, true)
  local c5 = CM.trStep(T5, cfg, 1 / 60, env)
  check("quick FPP down: cut", c5[1] == "fpp_on")
  trRun(T5, cfg, 60, 0.2, env)
  CM.trQuick(T5, false)
  local reachedTppAt = nil
  for i = 1, 60 do CM.trStep(T5, cfg, 1 / 60, env); if not reachedTppAt and T5.state == "tpp" then reachedTppAt = i end end
  check("quick FPP up: back without the boom animation", reachedTppAt and reachedTppAt <= math.ceil(cfg.repWait * 60) + 3,
        tostring(reachedTppAt))
  -- regression: release one frame after the press (settle -> fpp in between) still re-enters instantly
  local T5b = CM.newTransition(true)
  CM.trRequest(T5b, true)
  trRun(T5b, cfg, 60, 1.0, env)
  CM.trQuick(T5b, true)
  CM.trStep(T5b, cfg, 1 / 60, env)
  CM.trQuick(T5b, false)
  local at = nil
  for i = 1, 60 do CM.trStep(T5b, cfg, 1 / 60, env); if not at and T5b.state == "tpp" then at = i end end
  check("quick FPP released after one frame: instant re-entry kept", at and at <= math.ceil(cfg.repWait * 60) + 4, tostring(at))
  -- reverse mid-exit
  local T6 = CM.newTransition(false)
  CM.trRequest(T6, true)
  trRun(T6, cfg, 60, 1.0, env)
  CM.trRequest(T6, false)
  trRun(T6, cfg, 60, 0.1, env)
  local wMid = T6.w
  CM.trRequest(T6, true)
  CM.trStep(T6, cfg, 1 / 60, env)
  check("reverse during exit continues from the current weight", T6.state == "entering" and T6.w > wMid and T6.w < wMid + 0.1)
  -- freeze, missing component, no representation
  local T7 = CM.newTransition(true)
  CM.trRequest(T7, true)
  local c7 = trRun(T7, cfg, 60, 0.5, { enabled = true, canEnter = true, freeze = true })
  check("freeze (menu / photo mode): nothing happens", #c7 == 0 and T7.state == "fpp")
  local c8 = trRun(T7, cfg, 60, 0.5, { enabled = true, canEnter = false })
  check("component missing: stays first person", #c8 == 0 and T7.state == "fpp")
  local T9 = CM.newTransition(false)
  CM.trRequest(T9, true)
  local c9 = CM.trStep(T9, cfg, 1 / 60, env)
  check("body setting off: camera only", c9[1] == "cam_on" and #c9 == 1)
  local e0 = T9.epoch
  CM.trRequest(T9, false); CM.trRequest(T9, true)
  check("epoch increases on every change of request", T9.epoch == e0 + 2)
end

-- ================================================================ solver
local function solverRun(fps, secs, opts)
  local cfg = CM.config(opts.cfg)
  local S = CM.newSolver(cfg)
  local T = CM.newTransition(false)
  CM.trRequest(T, true)
  local blend = CM.newBlend(MEDIUM)
  local sh = CM.newShoulder(1)
  local res, maxStep, prevWorld, clipped = nil, 0, nil, false
  local ppos, body = V(0, 0, 0), 0
  local dt = 1 / fps
  for i = 1, math.floor(fps * secs + 0.5) do
    local t = i * dt
    CM.trStep(T, cfg, dt, { enabled = true, canEnter = true, repReady = true })
    if opts.move then ppos = opts.move(t, ppos, dt) end
    if opts.turn then body = opts.turn(t, body, dt) end
    local pq = CM.qFromYawPitch(body, 0)
    res = CM.solve(S, { dt = dt, playerPos = ppos, playerQ = pq, bodyYaw = body, eyeZ = ppos.z + 1.62,
      eyePos = CM.add(ppos, CM.qrot(pq, V(0, 0.1, 1.62))), fppPitch = opts.pitch or 0, fppFov = 51,
      dyaw = opts.dyaw and opts.dyaw * dt or 0, dpitch = 0, freeHeld = opts.free, moving = true,
      profile = CM.blendStep(blend, dt), side = CM.shoulderStep(sh, dt, cfg.shoulderTime), w = T.w, zoom = 0,
      raycast = opts.world and opts.world.ray or nil })
    if prevWorld then maxStep = math.max(maxStep, CM.dist(prevWorld, res.world)) end
    prevWorld = res.world
    if opts.wallY and res.world.y < opts.wallY - 1e-6 then clipped = true end
  end
  return res, S, maxStep, clipped
end
do
  local P2 = MEDIUM
  local cfg = CM.config()
  -- w = 0 with clearanceBlend = false (the previous blend): camera at the eyes, FPP orientation and FOV
  local S = CM.newSolver(CM.config({ clearanceBlend = false }))
  local eye = V(1, 2, 1.7)
  local function w0inp(w) return { dt = 1 / 60, playerPos = V(1, 2, 0), playerQ = CM.qFromYawPitch(30, 0), bodyYaw = 30, eyeZ = 1.7,
    eyePos = eye, fppPitch = -10, fppFov = 51, dyaw = 0, dpitch = 0, moving = true, profile = P2, side = 1, w = w } end
  local r0 = CM.solve(S, w0inp(0))
  check("solve w=0 (clearanceBlend = false): camera at the FPP eye point, FPP direction and FOV, inside", vnear(r0.world, eye, 1e-9)
        and near(r0.yaw, 30) and near(r0.pitch, -10) and near(r0.fov, 51) and r0.inside == true, vs(r0.world))
  -- 2026-11-01 NP-3 opt-in clearanceBlend = true: w = 0 is pivot + boom direction * clearance, outside the head; orientation / FOV
  -- still the FPP view; the boom direction is that of the full boom (w = 1)
  local cfgB = CM.config({ clearanceBlend = true }) -- opt-in since 2026-11-01 (default off: no boom jump at the blend ends)
  local clr = CM.clearance(cfgB)
  local rc0 = CM.solve(CM.newSolver(cfgB), w0inp(0))
  local rc1 = CM.solve(CM.newSolver(cfgB), w0inp(1))
  local dir0 = CM.scale(CM.sub(rc0.world, rc0.pivot), 1 / math.max(CM.dist(rc0.world, rc0.pivot), 1e-9))
  local dir1 = CM.scale(CM.sub(rc1.world, rc1.pivot), 1 / CM.dist(rc1.world, rc1.pivot))
  -- (w = 1 yaw / pitch differ from the FPP view only by the follow look, here equal: bodyYaw 30, no input)
  check("NP-3 solve w=0 (clearanceBlend): camera at pivot + boom direction * clearance, FPP direction and FOV, not inside",
    near(CM.dist(rc0.world, rc0.pivot), clr, 1e-9) and vnear(dir0, dir1, 1e-6) and near(rc0.yaw, 30) and near(rc0.pitch, -10)
      and near(rc0.fov, 51) and rc0.inside == false and CM.dist(rc0.world, eye) > 0.2,
    vs(rc0.world) .. " pivot " .. vs(rc0.pivot) .. " d=" .. CM.dist(rc0.world, rc0.pivot))
  -- boom length from the pivot over the whole blend: never below the clearance, monotonic, the full boom at w = 1
  local okMin, okMono, lastD, worst = true, true, -1, math.huge
  for i = 0, 100 do
    local r = CM.solve(CM.newSolver(cfgB), w0inp(i / 100))
    local d = CM.dist(r.world, r.pivot)
    worst = math.min(worst, d)
    if d < clr - 1e-9 or r.inside then okMin = false end
    if d < lastD - 1e-12 then okMono = false end
    lastD = d
  end
  check("NP-3 solve: enter / exit blend never inside the clearance (every w in [0, 1]), boom length monotonic", okMin and okMono, worst)
  local rcOld = CM.solve(CM.newSolver(CM.config({ clearanceBlend = false })), w0inp(1))
  check("NP-3 solve w=1: identical with and without clearanceBlend", rcOld.world.x == rc1.world.x and rcOld.world.y == rc1.world.y
    and rcOld.world.z == rc1.world.z and rcOld.arm == rc1.arm and rc1.inside == false)
  -- inside flag through the solver: a surface 0.25 m behind the pivot along the boom -> placed before it, inside; gone -> out
  local Sw = CM.newSolver(cfg)
  local wallD = 0.25
  local inpW = w0inp(1)
  inpW.raycast = function(from, to) return wallD end
  local rw = CM.solve(Sw, inpW)
  check("NP-3 solve: near surface -> boom at hit - gap (never behind it), inside", near(rw.arm, wallD - cfg.clearanceWallGap, 1e-9)
    and rw.inside == true and rw.hits > 0, rw.arm)
  wallD = nil
  local rOut = rw
  for _ = 1, 120 do rOut = CM.solve(Sw, inpW) end
  check("NP-3 solve: surface gone -> boom back out, not inside", rOut.inside == false and rOut.arm > clr, rOut.arm)
  -- w = 1 open space: boom = pivot + R(yaw, pitch)(side, -distance, height)
  local r1, S1, step = solverRun(60, 1.5, {})
  local expect = CM.add(V(0, 0, cfg.standHeight), CM.boomOffset(0, 0, P2.side, P2.distance, P2.height)) -- 0.1.3 capsule pivot
  local rf = solverRun(60, 1.5, { cfg = { pivotSource = "fpp" } })
  check("solve w=1 with pivotSource=fpp: pivot at the FPP eye height (1.62 m)", vnear(rf.world,
        CM.add(V(0, 0, 1.62), CM.boomOffset(0, 0, P2.side, P2.distance, P2.height)), 1e-6), vs(rf.world))
  check("solve w=1: boom offset behind the right shoulder", vnear(r1.world, expect, 1e-6) and near(r1.fov, P2.fov, 0.01),
        vs(r1.world) .. " vs " .. vs(expect))
  check("solve: enter transition without jumps (60 fps)", step < 0.2, step)
  check("solve: local position in the player space", vnear(r1.localPos, CM.sub(expect, V(0, 0, 0)), 1e-6))
  -- follow mode: V turned by the mouse keeps the camera fixed in the player space (rigid behind V)
  local r2 = solverRun(60, 1.5, { turn = function(t, b, dt) return b + 90 * dt end, dyaw = 90 })
  check("follow mode: local transform constant while V turns with the mouse", vnear(r2.localPos, r1.localPos, 1e-6), vs(r2.localPos))
  -- 0.1.3: a turn without the mouse lags (limiter) and catches up after the turn
  local r3 = solverRun(60, 2.0, { turn = function(t, b, dt) return t < 0.8 and b + 90 * dt or b end })
  check("follow mode: body turn without the mouse followed by the limiter (caught up 1.2 s after)", vnear(r3.localPos,
        r1.localPos, 1e-3) and r3.flip == nil, vs(r3.localPos))
  -- frame-rate independence of the whole rig (enter + free-look orbit)
  local a = solverRun(30, 1.2, { free = true, dyaw = 60 })
  local b = solverRun(144, 1.2, { free = true, dyaw = 60 })
  check("solver result equal at 30 and 144 fps (enter + orbit)", CM.dist(a.world, b.world) < 0.02,
        string.format("%.4f", CM.dist(a.world, b.world)))
  -- collision: wall 1.2 m behind V; V walks backwards into it: the camera never passes the wall
  local wallY = -1.2
  local world = makeWorld({ planes = { { axis = "y", value = wallY } } })
  local _, Sw, _, clipped = solverRun(60, 3, { world = world, wallY = wallY,
    move = function(t, p, dt) return V(p.x, math.max(p.y - 0.4 * dt, wallY + 0.3), p.z) end })
  check("collision: camera never behind the wall while backing into it", not clipped and Sw.col.hits > 0, tostring(Sw.col.hits))
  -- thin pole only hit by an edge ray: shortened on full-set frames
  -- at y = -1.2 the center ray is at x = 0.5 * 1.2 / 2.1 = 0.286; edge rays at 0.286 +- 0.18 -> the pole hits only x = 0.466
  local pole = makeWorld({ poles = { { x = 0.456, y = -1.2, r = 0.03 } } })
  local rp, Sp = solverRun(60, 2, { world = pole })
  check("collision: thin obstacle beside the center ray caught by the edge rays", Sp.col.len < CM.len(CM.boomOffset(0, 0, P2.side,
        P2.distance, P2.height)) - 0.3, string.format("%.3f", Sp.col.len))
  -- ray budget: open space uses mostly the center ray
  local open = makeWorld({})
  solverRun(60, 1, { world = open })
  check("collision cost in open space: <= 2.5 rays per frame on average", open.casts / 60 <= 2.5, open.casts / 60)
end

-- ================================================================ 0.1.4 aim (over the shoulder in third person)
do
  -- enter / exit decision
  local base = { enabled = true, tpp = true, upperBody = 6, weaponRanged = true, aimingSetting = "tpp", vehicleHeld = false,
                 poseOverride = false }
  local function with(over)
    local o = {}
    for k, v in pairs(base) do o[k] = v end
    for k, v in pairs(over) do if v == "<nil>" then o[k] = nil else o[k] = v end end
    return o
  end
  local okAll = CM.aimWanted(base)
  check("aim decision: all conditions -> aim", okAll == true)
  local cases = {
    { "not third person", { tpp = false }, "not third person" },
    { "UpperBody not Aim", { upperBody = 0 }, "UpperBody=0" },
    { "no ranged weapon", { weaponRanged = false }, "no ranged weapon" },
    { "perspective.aiming = fpp", { aimingSetting = "fpp" }, "perspective.aiming=fpp" },
    { "perspective.aiming unknown (utpp_perspective missing)", { aimingSetting = "<nil>" }, "perspective.aiming=nil" },
    { "vehicle hold wins", { vehicleHeld = true }, "vehicle hold" },
    { "pose override wins", { poseOverride = true }, "pose override" },
    { "camera.aim.enabled off", { enabled = false }, "camera.aim.enabled off" },
  }
  local okCases, bad = true, nil
  for _, c in ipairs(cases) do
    local w, r = CM.aimWanted(with(c[2]))
    if w ~= false or r ~= c[3] then okCases, bad = false, c[1] .. " -> " .. tostring(w) .. " " .. tostring(r) end
  end
  check("aim decision: every condition dropping -> no aim, with its reason", okCases, bad)
  local wv = CM.aimWanted(with({ vehicleHeld = true, poseOverride = true }))
  check("aim decision: legacy aiming=false counts as tpp, enabled=nil counts as on, vehicle hold beats everything",
        CM.aimWanted(with({ aimingSetting = false })) == true and CM.aimWanted(with({ enabled = nil })) == true and wv == false)
  check("weapon ranged: empty slot / fists / melee type / PSM melee block -> not ranged; ranged or unknown type -> ranged",
        CM.weaponRanged(false, nil, 0) == false and CM.weaponRanged(true, "", 0) == false
        and CM.weaponRanged(true, "Wea_Katana", 0) == false and CM.weaponRanged(true, "Cyb_MantisBlades", nil) == false
        and CM.weaponRanged(true, "Wea_Handgun", 7) == false and CM.weaponRanged(true, "Wea_Handgun", 0) == true
        and CM.weaponRanged(true, "Wea_AssaultRifle", nil) == true and CM.weaponRanged(true, nil, nil) == true)

  -- aim weight tween at 30 / 60 / 144 fps: in 0.12 s, out 0.20 s, smoothstep (like CM.blendTo)
  local cfg = CM.config({ aimZoomFov = true, aimOwnYaw = false, aimDistance = 0.8 }) -- legacy aim (2026-10-12 defaults: no FOV zoom, own yaw)
  local function tween(fps, secs, want, A)
    A = A or CM.newAim()
    local dt, t, evs, tFull, maxStep, prev = 1 / fps, 0, {}, nil, 0, A.w
    for _ = 1, math.floor(secs * fps + 0.5) do
      t = t + dt
      local ev, w = CM.aimStep(A, cfg, dt, want, want and "aiming" or "test")
      if ev then evs[#evs + 1] = ev end
      maxStep = math.max(maxStep, math.abs(w - prev))
      prev = w
      if tFull == nil and ((want and w >= 1) or (not want and w <= 0)) then tFull = t end
    end
    return A, evs, tFull, maxStep
  end
  local okIn, okOut, detail = true, true, {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    local A, evs, tIn = tween(fps, 0.3, true)
    -- the enter frame starts the tween (t = 0), the weight reaches 1 after 0.12 s of further frames
    if not (evs[1] == "enter" and #evs == 1 and tIn and math.abs(tIn - (1 / fps + 0.12)) <= 1 / fps + 1e-9) then okIn = false end
    local _, evs2, tOut = tween(fps, 0.4, false, A)
    if not (evs2[1] == "exit" and #evs2 == 1 and tOut and math.abs(tOut - (1 / fps + 0.20)) <= 1 / fps + 1e-9) then okOut = false end
    detail[#detail + 1] = string.format("%d fps in %.3f out %.3f", fps, tIn or -1, tOut or -1)
  end
  check("aim tween: blend in 0.12 s at 30 / 60 / 144 fps (one enter event)", okIn, table.concat(detail, ", "))
  check("aim tween: blend out 0.20 s at 30 / 60 / 144 fps (one exit event)", okOut, table.concat(detail, ", "))
  -- weight at the same time is equal at every frame rate (piecewise-linear t + smoothstep, sampled at exact times)
  local wAt = {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    local A = CM.newAim()
    CM.aimStep(A, cfg, 1 / fps, true, "aiming")
    for _ = 1, math.floor(0.06 * fps + 0.5) do CM.aimStep(A, cfg, 1 / fps, true) end
    wAt[fps] = { w = A.w, expect = CM.smoothstep(math.floor(0.06 * fps + 0.5) / fps / 0.12) }
  end
  check("aim tween: weight = smoothstep(t / 0.12) at every frame rate (same time = same weight)",
        math.abs(wAt[30].w - wAt[30].expect) < 1e-9 and math.abs(wAt[60].w - wAt[60].expect) < 1e-9
        and math.abs(wAt[144].w - wAt[144].expect) < 1e-9 and math.abs(wAt[30].w - wAt[60].w) < 1e-9, string.format("%.4f %.4f %.4f", wAt[30].w, wAt[60].w, wAt[144].w))
  -- interrupted: exit in the middle of the blend in continues from the current weight (no jump)
  local Ai = CM.newAim()
  for _ = 1, 4 do CM.aimStep(Ai, cfg, 1 / 60, true) end
  local wMid = Ai.w
  local _, wNext = CM.aimStep(Ai, cfg, 1 / 60, false, "UpperBody=0")
  check("aim tween: exit during the blend in starts from the current weight", wMid > 0.1 and wMid < 0.9 and math.abs(wNext - wMid) < 1e-9
        and Ai.from == wMid and Ai.exits == 1, string.format("%.3f -> %.3f", wMid, wNext))
  local _, _, _, maxStep60 = tween(60, 0.3, true)
  check("aim tween: no weight jump larger than one frame of a 0.12 s smoothstep (60 fps)", maxStep60 <= 1.5 * (1 / 60) / 0.12 + 1e-9,
        maxStep60)
  -- aim time / Status helpers
  local At = CM.newAim()
  CM.aimStep(At, cfg, 0.5, false)
  CM.aimStep(At, cfg, 0.1, true, "aiming")
  CM.aimStep(At, cfg, 0.25, true)
  check("aim time: seconds since the enter", math.abs(CM.aimTime(At) - 0.25) < 1e-9 and CM.aimActive(At), CM.aimTime(At))

  -- aim pose: composed over the live profile
  local P2 = MEDIUM
  local A1 = CM.newAim()
  check("aim compose: weight 0 = the profile itself", CM.aimCompose(P2, A1, cfg, 42) == P2)
  A1.w = 1
  local pa = CM.aimCompose(P2, A1, cfg, nil)
  check("aim compose: weight 1 = aim pose 0.8 m / side 0.45 m / FOV 42 (camera.aim.fov) / height 0.05",
        near(pa.distance, 0.8) and near(pa.side, 0.45) and near(pa.fov, 42) and near(pa.height, 0.05) and near(pa.sens, P2.sens))
  check("aim compose: weapon AimFOV replaces camera.aim.fov", near(CM.aimCompose(P2, A1, cfg, 55).fov, 55))
  A1.w = 0.5
  local ph = CM.aimCompose(P2, A1, cfg, 42)
  check("aim compose: half weight = halfway between the profile and the aim pose",
        near(ph.distance, (P2.distance + 0.8) / 2) and near(ph.side, (P2.side + 0.45) / 2) and near(ph.fov, (P2.fov + 42) / 2))

  -- full solver run with the aim state at 30 / 60 / 144 fps
  local function aimRun(fps, opts)
    local oc = { aimZoomFov = true, aimOwnYaw = false, aimDistance = 0.8, -- legacy aim (2026-10-12 23:40 defaults: partial boom pitch, +-85)
                 aimBoomPitchScale = 1, aimPitchMin = -70, aimPitchMax = 70 } for k, v in pairs(opts.cfg or {}) do oc[k] = v end
    local c = CM.config(oc)
    local S = CM.newSolver(c)
    local A = CM.newAim()
    local blend = CM.newBlend(P2)
    local sh = CM.newShoulder(opts.shoulder or 1)
    local ppos, body, dt = V(0, 0, 0), opts.body or 0, 1 / fps
    local res, t, log = nil, 0, { fovMaxStep = 0, yawMaxStep = 0, posMaxStep = 0 }
    local prev = nil
    for i = 1, math.floor((opts.secs or 1.0) * fps + 0.5) do
      t = i * dt
      local want = t >= (opts.aimFrom or 0.3) - 1e-6 and t < (opts.aimTo or 1e9) - 1e-6
      CM.aimStep(A, c, dt, want, want and "aiming" or "UpperBody=0")
      if opts.turn then body = opts.turn(t, body, dt) end
      local pq = CM.qFromYawPitch(body, 0)
      local fpitch = opts.pitch and opts.pitch(t) or -5
      res = CM.solve(S, { dt = dt, playerPos = ppos, playerQ = pq, bodyYaw = body, eyeZ = 1.62, fppPitch = fpitch, fppFov = 51,
        dyaw = opts.dyaw and opts.dyaw(t) * dt or 0, dpitch = 0, freeHeld = opts.free, moving = opts.moving ~= false,
        profile = CM.aimCompose(CM.blendStep(blend, dt), A, c, opts.fov), side = CM.shoulderStep(sh, dt, c.shoulderTime),
        w = 1, zoom = (opts.zoom or 0) * (1 - A.w), aim = A.on, aimW = A.w, fovDirect = CM.aimActive(A) })
      if prev then
        log.fovMaxStep = math.max(log.fovMaxStep, math.abs(res.fov - prev.fov))
        log.yawMaxStep = math.max(log.yawMaxStep, math.abs(CM.wrapDeg(res.yaw - prev.yaw)))
        log.posMaxStep = math.max(log.posMaxStep, CM.dist(res.world, prev.world))
      end
      if opts.sample and not log.sample and t >= opts.sample - 1e-6 then log.sample = res end
      prev = res
    end
    return res, A, log, body
  end
  local okPose, okFov, okFwd, poses = true, true, true, {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    local r, A = aimRun(fps, { secs = 0.8, body = 30, pitch = function() return -12 end })
    local expect = CM.add(V(0, 0, cfg.standHeight), CM.boomOffset(30, -12, 0.45, 0.8, 0.05))
    if not (A.on and vnear(r.world, expect, 1e-6)) then okPose = false end
    if not near(r.fov, 42, 1e-9) then okFov = false end
    local f = CM.qrot(r.q, V(0, 1, 0))
    if CM.dist(f, CM.forward(30, -12)) > 1e-6 then okFwd = false end
    poses[fps] = r.world
  end
  check("aim solve: camera 0.8 m behind the capsule pivot, 0.45 m right, 0.05 m up (30 / 60 / 144 fps)", okPose)
  check("aim solve: FOV exactly the aim FOV (no fovRate lag)", okFov)
  check("aim solve: camera forward = body yaw + FPP pitch (the aim direction)", okFwd)
  check("aim solve: same pose at 30 / 60 / 144 fps", CM.dist(poses[30], poses[144]) < 1e-6 and CM.dist(poses[60], poses[144]) < 1e-6)
  -- blend in at the same time equal across frame rates (sampled 0.06 s into the blend at frame-exact times)
  local s30 = select(3, aimRun(30, { secs = 0.5, sample = 0.3 + 2 / 30 })).sample
  local s60 = select(3, aimRun(60, { secs = 0.5, sample = 0.3 + 4 / 60 })).sample
  check("aim solve: blend in distance / side / FOV equal at 30 and 60 fps at the same time", s30 and s60
        and CM.dist(s30.world, s60.world) < 1e-6 and math.abs(s30.fov - s60.fov) < 1e-6,
        s30 and s60 and string.format("%.5f fov %.3f / %.3f", CM.dist(s30.world, s60.world), s30.fov, s60.fov) or "no sample")
  -- zoom masked while aiming, back after the exit
  local rz = aimRun(60, { secs = 0.8, zoom = 1.5 })
  local expectZ = CM.add(V(0, 0, cfg.standHeight), CM.boomOffset(0, -5, 0.45, 0.8, 0.05))
  check("aim solve: zoom masked while aiming", vnear(rz.world, expectZ, 1e-6), vs(rz.world))
  -- free look / idle orbit ignored while aiming (the mouse would orbit otherwise)
  local rf = aimRun(60, { secs = 1.0, free = true, moving = false, dyaw = function() return 90 end })
  check("aim solve: free look held + idle (standing) ignored: camera yaw = body yaw", near(rf.yaw, 0, 1e-6), rf.yaw)
  -- aim pitch range wider than the orbit range (camera forward stays on the aim)
  local rp = aimRun(60, { secs = 0.8, pitch = function() return 66 end })
  check("aim solve: pitch 66 deg kept (orbit max 55, aim max 70)", near(rp.pitch, 66, 1e-6), rp.pitch)
  local rp2 = aimRun(60, { secs = 0.8, pitch = function() return -85 end })
  check("aim solve: pitch clamped at the aim min -70 (no top-down view over the head)", near(rp2.pitch, -70, 1e-6), rp2.pitch)
  -- entering aim from a free-look orbit offset: no yaw jump, aligned behind V within ~0.15 s
  -- 2026-10-12 round 2: the offset shrinks with CM.alignStep (<= aimAlignRate 300 deg/s), not with the 0.12 s pose tween
  local _, Ao, lo = aimRun(60, { secs = 0.3 + 0.6, free = true, dyaw = function(t) return t < 0.3 and 200 or 0 end })
  local ro = aimRun(60, { secs = 0.3 + 0.6, free = true, dyaw = function(t) return t < 0.3 and 200 or 0 end })
  local rmid = aimRun(60, { secs = 0.3 + 0.12, free = true, dyaw = function(t) return t < 0.3 and 200 or 0 end })
  check("aim solve: from a free-look offset (60 deg) the yaw glides onto the aim (no jump, <= 300 deg/s, aligned within 0.6 s)",
        Ao.on and math.abs(ro.yaw) < 1e-6 and lo.yawMaxStep <= 300 / 60 + 1e-6 and math.abs(rmid.yaw) > 10,
        string.format("yaw %.2f maxStep %.2f yaw at the end of the pose tween %.2f", ro.yaw, lo.yawMaxStep, rmid.yaw))
  -- exit: back to the profile over 0.20 s without FOV / position jumps, FOV back on the profile
  local okExit, ex = true, {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    local r, A, l = aimRun(fps, { secs = 1.4, aimTo = 0.9 })
    local expectP = CM.add(V(0, 0, cfg.standHeight), CM.boomOffset(0, -5, P2.side, P2.distance, P2.height))
    if A.on or not vnear(r.world, expectP, 1e-6) or not near(r.fov, P2.fov, 0.05) then okExit = false end
    -- per-frame steps bounded by the tween speed (0.12 s in: FOV 56 -> 42 = 14 deg; smoothstep peak 1.5x)
    if l.fovMaxStep > 1.5 * 14 / (0.12 * fps) + 1e-6 or l.posMaxStep > 1.5 * 1.4 / (0.12 * fps) + 1e-6 then okExit = false end
    ex[#ex + 1] = string.format("%d fps fov %.2f step %.3f pos %.3f", fps, r.fov, l.fovMaxStep, l.posMaxStep)
  end
  check("aim solve: enter + exit at 30 / 60 / 144 fps: profile pose back, FOV / position steps bounded by the tween", okExit,
        table.concat(ex, "; "))

  -- shoulder auto-swap with 0.3 s hysteresis
  local function swapRun(fps, hitFn, otherFn, secs)
    local st, dt, t, swaps = CM.newAimSwap(), 1 / fps, 0, {}
    for i = 1, math.floor(secs * fps + 0.5) do
      t = i * dt
      local sw = CM.aimSwapStep(st, cfg, dt, hitFn(t), function() return otherFn and otherFn(t) or nil end)
      if sw then swaps[#swaps + 1] = t end
    end
    return swaps, st
  end
  local okSw, sd = true, {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    -- the first frame with the wall counts its dt (the wall was there during that frame): swap 0.3 s after the
    -- frame before the first hit, within one frame
    local sw = swapRun(fps, function(t) return t >= 0.1 - 1e-6 and 0.3 or nil end, nil, 0.6)
    local t1 = math.ceil(0.1 * fps - 1e-6) / fps -- first frame with the wall
    local e = t1 - 1 / fps + 0.3
    if not (#sw == 1 and sw[1] >= e - 1e-6 and sw[1] <= e + 1 / fps + 1e-6) then okSw = false end
    sd[#sd + 1] = string.format("%d fps %s", fps, sw[1] and string.format("%.3f", sw[1]) or "none")
  end
  check("aim auto-swap: wall 0.3 m on the shoulder side for 0.3 s -> one swap (30 / 60 / 144 fps)", okSw, table.concat(sd, ", "))
  local swFlick = swapRun(60, function(t) return (t % 0.25) < 0.2 and 0.3 or nil end, nil, 1.0)
  check("aim auto-swap: wall seen < 0.3 s at a time (flicker) -> no swap", #swFlick == 0, #swFlick)
  local swFar = swapRun(60, function() return 0.6 end, nil, 1.0)
  check("aim auto-swap: wall beyond 0.5 m -> no swap", #swFar == 0)
  local swBoth = swapRun(60, function() return 0.3 end, function() return 0.25 end, 1.0)
  check("aim auto-swap: other side as blocked -> stays", #swBoth == 0)
  local swAgain = swapRun(60, function() return 0.3 end, nil, 1.0)
  check("aim auto-swap: after a swap the new side needs its own 0.3 s (hysteresis, no ping-pong)", #swAgain == 3
        and swAgain[2] - swAgain[1] >= 0.3 - 1e-9, #swAgain)
end

-- ================================================================ 0.1.4 aim occlusion (level A)
do
  local cfg = CM.config()
  check("aim occlusion defaults: on, range 60 m, tolerance 0.15 m, 30 Hz / every 2nd frame, hysteresis 0.1 s, weight > 0.5",
        cfg.aimOcclusion == true and cfg.aimRange == 60 and cfg.aimOccTol == 0.15 and cfg.aimOccHz == 30
        and cfg.aimOccMinFrames == 2 and cfg.aimOccHold == 0.1 and cfg.aimOccMinW == 0.5)
  -- camera at the origin looking along +Y; muzzle 1.2 m ahead, 0.3 m left, 0.15 m down (over-the-shoulder layout)
  local cam, fwd, muz = V(0, 0, 0), V(0, 1, 0), V(-0.3, 1.2, -0.15)
  check("aim occlusion t0 = projection of the muzzle on the camera ray", near(CM.aimOccT0(cam, fwd, muz), 1.2))
  check("aim occlusion t0 = 0 when the muzzle is behind the camera", CM.aimOccT0(cam, fwd, V(0.2, -0.4, 0)) == 0)
  local a, b = CM.aimOccRay1(cam, fwd, 1.2, 60)
  check("aim occlusion ray 1 runs from the muzzle plane to the range", vnear(a, V(0, 1.2, 0)) and vnear(b, V(0, 60, 0)))
  check("aim occlusion ray 1: none when t0 >= range", CM.aimOccRay1(cam, fwd, 60, 60) == nil)
  local P1, t1 = CM.aimOccPoint(cam, fwd, 1.2, 60, nil)
  local P2, t2 = CM.aimOccPoint(cam, fwd, 1.2, 60, 8.8)
  check("aim occlusion P: no hit -> far end (range), hit -> t0 + hit", vnear(P1, V(0, 60, 0)) and near(t1, 60)
        and vnear(P2, V(0, 10, 0)) and near(t2, 10))
  -- decision with the 0.15 m tolerance (P 10 m ahead)
  local dMP = CM.dist(V(0, 10, 0), muz)
  local function dec(h) return (CM.aimOccDecide(cfg, fwd, muz, V(0, 10, 0), h)) end
  local _, dOut, rClear = CM.aimOccDecide(cfg, fwd, muz, V(0, 10, 0), nil)
  check("aim occlusion decide: no ray 2 hit -> clear", dec(nil) == false and rClear == "clear" and near(dOut, dMP))
  local _, _, rAt = CM.aimOccDecide(cfg, fwd, muz, V(0, 10, 0), dMP)
  check("aim occlusion decide: hit exactly at P -> not blocked (hit at P)", dec(dMP) == false and rAt == "hit at P")
  check("aim occlusion decide: hit 0.10 m before P (inside the 0.15 m tolerance) -> not blocked", dec(dMP - 0.10) == false)
  check("aim occlusion decide: hit 0.20 m before P -> blocked", dec(dMP - 0.20) == true)
  check("aim occlusion decide: cover 1 m in front of the muzzle -> blocked", dec(1.0) == true)
  local bBehind, _, rBehind = CM.aimOccDecide(cfg, fwd, muz, V(0, 1.15, 0), 0.01)
  check("aim occlusion decide: P behind the muzzle -> not blocked", bBehind == false and rBehind == "P behind the muzzle")
  local bNear = CM.aimOccDecide(cfg, fwd, muz, V(0, 1.25, 0), 0.01)
  check("aim occlusion decide: P only 0.05 m in front of the muzzle (< 0.1 m) -> not blocked", bNear == false)
  -- sample with a ray mock: planes y = const (walls facing the camera) + an obstacle hit only by the muzzle ray
  local function planeRay(walls, cover)
    local calls = {}
    return function(from, to)
      calls[#calls + 1] = { from = from, to = to }
      local best = nil
      for _, wy in ipairs(walls) do
        local da, db = from.y - wy, to.y - wy
        if da ~= 0 and da * db <= 0 then -- crossing or touching at the segment end (P on the wall)
          local d = CM.dist(from, to) * da / (da - db)
          if d > 1e-4 and (best == nil or d < best) then best = d end
        end
      end
      if cover and vnear(from, muz) then best = (best == nil or cover < best) and cover or best end
      return best
    end, calls
  end
  local ray, calls = planeRay({}, nil)
  local s1 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: open space -> P = far end 60 m, 2 rays, not blocked", s1.raw == false and near(s1.t, 60)
        and #calls == 2 and s1.rays == 2 and s1.dMuzzleHit == nil and vnear(calls[1].from, V(0, 1.2, 0))
        and vnear(calls[2].from, muz) and vnear(calls[2].to, V(0, 60, 0)), s1.reason)
  ray, calls = planeRay({ 20 }, nil)
  local s2 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: wall at 20 m -> P on the wall, muzzle ray hits at P (tolerance) -> not blocked",
        s2.raw == false and near(s2.t, 20, 1e-9) and s2.reason == "hit at P" and near(s2.dMuzzleHit, s2.dMuzzleP, 1e-6), s2.reason)
  ray, calls = planeRay({ 20 }, 0.8)
  local s3 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: cover 0.8 m in front of the muzzle, camera ray clear -> blocked", s3.raw == true
        and near(s3.dMuzzleHit, 0.8) and s3.reason == "blocked" and near(s3.t, 20, 1e-9))
  -- near wall: V faces a wall 1.4 m in front of the camera (0.2 m in front of the muzzle) -> P on the wall, not blocked
  ray, calls = planeRay({ 1.4 }, nil)
  local s4 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: near wall (0.2 m in front of the muzzle) -> P on the wall, not blocked", s4.raw == false
        and near(s4.t, 1.4, 1e-9) and s4.rays == 2, s4.reason)
  -- wall 1.25 m: P only 0.05 m ahead of the muzzle -> ray 2 not cast
  ray, calls = planeRay({ 1.25 }, nil)
  local s5 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: wall 0.05 m in front of the muzzle -> P behind / at the muzzle, ray 2 not cast",
        s5.raw == false and s5.reason == "P behind the muzzle" and s5.rays == 1 and #calls == 1, s5.reason)
  -- muzzle through the wall (wall between the camera and the muzzle): ray 1 starts past the wall -> far end, clear
  ray, calls = planeRay({ 1.0 }, nil)
  local s6 = CM.aimOccSample(cfg, cam, fwd, muz, ray)
  check("aim occlusion sample: wall between the camera and the muzzle -> ray 1 starts past it (P far end), not blocked",
        s6.raw == false and near(s6.t, 60) and s6.reason == "clear")
  -- muzzle beyond the range
  ray, calls = planeRay({}, nil)
  local s7 = CM.aimOccSample(CM.config({ aimRange = 1.0 }), cam, fwd, muz, ray)
  check("aim occlusion sample: muzzle plane beyond the range -> no rays, not blocked", s7.raw == false and #calls == 0
        and s7.reason == "P behind the muzzle")
  -- throttle + hysteresis at 30 / 60 / 144 fps: cover from 0.50 s to 1.50 s (raw blocked), 2 s total
  local function occRun(fps, rawFn, secs, c)
    c = c or cfg
    local st = CM.newAimOcc()
    local ev, clock, prevSample, minGapFrames, maxRate = {}, 0, nil, 1e9, 0
    local frame = 0
    for i = 1, math.floor(secs * fps + 0.5) do
      clock = i / fps
      if CM.aimOccDue(st, c, clock) then
        if prevSample then
          minGapFrames = math.min(minGapFrames, i - prevSample)
          maxRate = math.max(maxRate, 1 / ((i - prevSample) / fps))
        end
        prevSample = i
        local e = CM.aimOccStep(st, c, clock, rawFn(clock), nil)
        if e then ev[#ev + 1] = { e, clock } end
      end
    end
    return ev, st, minGapFrames, maxRate
  end
  local okEdges, okThrottle, det = true, true, {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    local ev, st, gap, rate = occRun(fps, function(t) return t > 0.5 + 1e-9 and t <= 1.5 + 1e-9 end, 2.0)
    local interval = math.max(2 / fps, math.ceil((1 / 30 - 1e-6) * fps) / fps)
    local hi = 0.1 + 2 * interval + 1 / fps + 1e-6
    local ok = #ev == 2 and ev[1][1] == "blocked" and ev[2][1] == "clear"
      and ev[1][2] - 0.5 >= 0.1 - 1e-6 and ev[1][2] - 0.5 <= hi and ev[2][2] - 1.5 >= 0.1 - 1e-6 and ev[2][2] - 1.5 <= hi
    if not ok then okEdges = false end
    if not (gap >= 2 and rate <= 30 + 1e-6) then okThrottle = false end
    det[#det + 1] = string.format("%d fps: %s %s gap %d rate %.1f samples %d", fps, ev[1] and string.format("%s %.3f", ev[1][1], ev[1][2]) or "-",
      ev[2] and string.format("%s %.3f", ev[2][1], ev[2][2]) or "-", gap, rate, st.samples)
  end
  check("aim occlusion edges: blocked / clear 0.1 s .. 0.1 s + 2 samples + 1 frame after the change (30 / 60 / 144 fps)",
        okEdges, table.concat(det, "; "))
  check("aim occlusion throttle: never 2 frames in a row, never above 30 Hz (30 / 60 / 144 fps)", okThrottle, table.concat(det, "; "))
  local _, st60 = occRun(60, function() return false end, 1.0)
  local _, st144 = occRun(144, function() return false end, 1.0)
  local _, st30 = occRun(30, function() return false end, 1.0)
  check("aim occlusion throttle: ~30 samples/s at 60 fps, <= 30 at 144 fps, 15 at 30 fps (every 2nd frame)",
        st60.samples >= 29 and st60.samples <= 31 and st144.samples >= 27 and st144.samples <= 30 and st30.samples == 15,
        string.format("%d %d %d", st60.samples, st144.samples, st30.samples))
  -- flicker shorter than the hysteresis: no edge
  local evF = occRun(60, function(t) return (t % 0.2) < 0.07 end, 2.0)
  check("aim occlusion hysteresis: raw blocked < 0.1 s at a time (flicker) -> no edge", #evF == 0, #evF)
  local evC = occRun(60, function(t) return t > 0.3 and not (t > 1.0 and t < 1.06) end, 2.0)
  check("aim occlusion hysteresis: clear gap < 0.1 s while blocked -> stays blocked (one edge)", #evC == 1 and evC[1][1] == "blocked", #evC)
  local evZ = occRun(60, function(t) return t > 0.5 + 1e-9 end, 1.0, CM.config({ aimOccHold = 0 }))
  check("aim occlusion hysteresis 0: edge at the first sample that sees the cover", #evZ == 1 and evZ[1][2] - 0.5 <= 2 / 60 + 1e-6,
        evZ[1] and evZ[1][2])
  -- reset: off at once, "clear" with the blocked time when it was blocked; next frame samples at once
  local st = CM.newAimOcc()
  CM.aimOccStep(st, cfg, 1.0, true)
  CM.aimOccStep(st, cfg, 1.1, true)
  local blockedNow = st.blocked
  local evR, durR = CM.aimOccReset(st, 1.6)
  local evR2 = CM.aimOccReset(st, 1.7)
  check("aim occlusion reset: blocked -> clear with the blocked time, second reset silent, next frame samples",
        blockedNow and evR == "clear" and near(durR, 0.5) and evR2 == nil and not st.blocked and CM.aimOccDue(st, cfg, 1.71))
end

-- ================================================================ summary
-- 2026-10-12 decoupled look (Locomotion turns V): body yaw changes do not move the camera, the mouse does
-- 2026-10-18: entering free mode must preserve the displayed world view even when the
-- previous follow limiter was stationary. A body turn on that frame is not camera input.
do
  for _, fps in ipairs({30, 60, 144}) do
    local cfg = CM.config({ idleOrbit = false })
    local L = CM.newLook()
    local first = CM.lookStep(L, cfg, 1 / fps, 0, 0, { bodyYaw = 170, fppPitch = -25, moving = true })
    local y, p = CM.lookStep(L, cfg, 1 / fps, 0, 0,
      { bodyYaw = -170, fppPitch = -79, moving = true, decoupled = true })
    check("free entry at " .. fps .. " fps: settled camera ignores simultaneous body turn and FPP pitch override",
      near(CM.wrapDeg(y - first), 0) and near(p, -25), y .. "/" .. p)
    local quiet = true
    for i = 1, fps * 2 do
      y, p = CM.lookStep(L, cfg, 1 / fps, 0, 0,
        { bodyYaw = CM.wrapDeg(i * 17), fppPitch = (i % 2 == 0) and -79 or 15,
          moving = i % 3 == 0, freeHeld = i % 5 == 0, decoupled = true })
      quiet = quiet and near(CM.wrapDeg(y - first), 0) and near(p, -25)
    end
    check("decoupled at " .. fps .. " fps: move/stop and executor hold changes cannot recenter or tilt", quiet)
    y, p = CM.lookStep(L, cfg, 1 / fps, 2, -3,
      { bodyYaw = 50, fppPitch = 15, moving = true, decoupled = true })
    check("decoupled at " .. fps .. " fps: only look input changes view", near(y, 172) and near(p, -28))
  end
end

do
  local cfg = CM.config({ idleOrbit = false, aimOwnYaw = false })
  local L = CM.newLook()
  CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true, decoupled = true })
  local y0 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true, decoupled = true })
  local y1
  for i = 1, 30 do -- the body flips back and forth every frame (strafe left / right)
    y1 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = (i % 2 == 0) and 190 or 10, fppPitch = -5, moving = true, decoupled = true })
  end
  check("decoupled: body flips do not move the camera yaw", near(y1, y0, 1e-6), tostring(y0) .. " -> " .. tostring(y1))
  local y2 = CM.lookStep(L, cfg, 1 / 60, 15, 0, { bodyYaw = 10, fppPitch = -5, moving = true, decoupled = true })
  check("decoupled: the mouse still turns the camera", near(y2, CM.wrapDeg(y1 + 15), 1e-6), tostring(y2))
  -- 2026-10-12 round 2: aiming from the decoupled camera keeps the shown yaw and glides onto the aim reference
  local y3 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true, decoupled = true, aim = true, aimW = 1 })
  check("decoupled -> aim: no snap onto the body yaw (first aim frame moves <= 300 deg/s)",
        math.abs(CM.wrapDeg(y3 - y2)) <= 300 / 60 + 1e-6, tostring(y2) .. " -> " .. tostring(y3))
  local y4
  for _ = 1, 60 do y4 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true, aim = true, aimW = 1 }) end
  check("decoupled -> aim: aligned on the body yaw (no refYaw) within 1 s", near(y4, 10, 1e-6), tostring(y4))
end

-- ================================================================ 2026-10-12 round 2: aim reference, alignment, prediction, calibration
do
  local cfg = CM.config({ idleOrbit = false, aimOwnYaw = false })
  -- alignStep: exponential at gain 10/s, >= 20 deg/s, <= 300 deg/s, never past 0, frame-rate independent end time
  local function alignTime(off, fps)
    local t, n = 0, 0
    while math.abs(off) > 0 and n < 10000 do off = CM.alignStep(off, cfg, 1 / fps); t = t + 1 / fps; n = n + 1 end
    return t, off
  end
  local t30, e30 = alignTime(180, 30)
  local t144, e144 = alignTime(180, 144)
  check("alignStep: 180 deg aligned exactly (0) within 1.2 s at 30 and 144 fps, similar end time", e30 == 0 and e144 == 0
        and t30 < 1.2 and t144 < 1.2 and math.abs(t30 - t144) < 0.1, string.format("%.3f / %.3f", t30, t144))
  check("alignStep: step capped at 300 deg/s, never past 0, sign kept", near(CM.alignStep(180, cfg, 0.1), 150)
        and CM.alignStep(0.1, cfg, 0.1) == 0 and near(CM.alignStep(-90, cfg, 0.05), -75) and CM.alignStep(0, cfg, 0.1) == 0)
  -- aim reference = the game's look yaw (refYaw), not the body yaw (the body turns under the camera when the lock ends)
  local L = CM.newLook()
  CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true, decoupled = true })
  for _ = 1, 30 do CM.lookStep(L, cfg, 1 / 60, 3, 0, { bodyYaw = 0, fppPitch = 0, moving = true, decoupled = true }) end
  local shown = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true, decoupled = true })
  local maxStep, prev, yA = 0, shown, nil
  for i = 1, 90 do -- the game turns V from 0 onto its look yaw 90 over 0.2 s; the look yaw itself stays at 90
    local body = math.min(90, 90 * i / 12)
    yA = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = body, fppPitch = 0, refYaw = 90, moving = true, aim = true, aimW = 1 })
    maxStep = math.max(maxStep, math.abs(CM.wrapDeg(yA - prev)))
    prev = yA
  end
  check("aim: shown yaw 90 (free orbit), look yaw 90, body turning 0 -> 90 under it: the camera does not move",
        near(shown, 90, 1e-6) and near(yA, 90, 1e-6) and maxStep < 1e-6, string.format("shown %.2f end %.2f maxStep %.4f", shown, yA, maxStep))
  local L2 = CM.newLook()
  CM.lookStep(L2, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = 0, moving = true, decoupled = true })
  local mx, pv, yB = 0, 0, nil
  for _ = 1, 90 do
    yB = CM.lookStep(L2, cfg, 1 / 60, 0, 0, { bodyYaw = 170, fppPitch = 0, refYaw = 170, moving = true, aim = true, aimW = 1 })
    mx = math.max(mx, math.abs(CM.wrapDeg(yB - pv)))
    pv = yB
  end
  check("aim: look yaw 170 deg away -> glide <= 300 deg/s, aligned within 1.5 s (old: 170 deg within the 0.12 s tween)",
        near(yB, 170, 1e-6) and mx <= 300 / 60 + 1e-6, string.format("end %.2f maxStep %.2f", yB, mx))
  -- aim pitch: offset to the FPP pitch shrinks the same way, then the FPP pitch is followed exactly (recoil kick)
  local L3 = CM.newLook()
  CM.lookStep(L3, cfg, 1 / 60, 0, 30, { bodyYaw = 0, fppPitch = 0, moving = true, decoupled = true })
  local _, p0 = CM.lookStep(L3, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = -10, moving = true, aim = true, aimW = 1 })
  local p1
  for _ = 1, 60 do _, p1 = CM.lookStep(L3, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = -10, moving = true, aim = true, aimW = 1 }) end
  local _, p2 = CM.lookStep(L3, cfg, 1 / 60, 0, 0, { bodyYaw = 0, fppPitch = -7, moving = true, aim = true, aimW = 1 })
  check("aim pitch: starts at the shown pitch, aligned on the FPP pitch, then follows it exactly",
        p0 > 20 and near(p1, -10, 1e-6) and near(p2, -7, 1e-6), string.format("%.2f %.2f %.2f", p0, p1, p2))
  -- prediction of the coming entity turn
  check("predictTurn: last step repeated, scaled by dt, 0 below 0.05 / above 15 deg or without dt",
        near(CM.predictTurn(6, 1 / 60, 1 / 60, 15), 6) and near(CM.predictTurn(6, 1 / 30, 1 / 60, 15), 12)
        and near(CM.predictTurn(6, 1 / 240, 1 / 60, 15), 3) and CM.predictTurn(0.01, 1 / 60, 1 / 60, 15) == 0
        and CM.predictTurn(40, 1 / 60, 1 / 60, 15) == 0 and CM.predictTurn(5, 1 / 60, nil, 15) == 0)
  -- the predicted local transform renders the intended world pose after the entity turned by the predicted step
  local ppos, pq = V(1, 2, 0), CM.qFromYawPitch(30, 0)
  local world, wq = V(0.5, -1.5, 1.7), CM.qFromYawPitch(75, -10)
  local step = 7
  local lp, lq = CM.toLocal(ppos, CM.qmul(CM.qFromYawPitch(step, 0), pq), world, wq)
  local rq = CM.qFromYawPitch(30 + step, 0) -- entity after the game tick
  local rw = CM.add(ppos, CM.qrot(rq, lp))
  local rf = CM.qrot(CM.qmul(rq, lq), V(0, 1, 0))
  local lp0 = CM.toLocal(ppos, pq, world, wq)
  local rw0 = CM.add(ppos, CM.qrot(rq, lp0))
  check("predictTurn: local transform against the predicted entity yaw -> rendered pose = intended pose (without: off)",
        vnear(rw, world, 1e-6) and CM.dist(rf, CM.forward(75, -10)) < 1e-6 and CM.dist(rw0, world) > 0.1,
        string.format("with %.6f without %.3f", CM.dist(rw, world), CM.dist(rw0, world)))
  -- calibration band: once calibrated, a heading-unlock burst (ratio 1.2) does not move the value; a persistent change re-seeds
  local cal = CM.newCalibration()
  for _ = 1, 20 do CM.calibrate(cal, 10, -1.5, 2, 20) end
  for _ = 1, 12 do CM.calibrate(cal, 10, -12, 2, 20) end
  local afterBurst = cal.value
  local calOld = CM.newCalibration()
  for _ = 1, 20 do CM.calibrate(calOld, 10, -1.5) end
  for _ = 1, 12 do CM.calibrate(calOld, 10, -12) end
  for _ = 1, 25 do CM.calibrate(cal, 10, -4.0, 2, 20) end
  check("calibration band: 12 outlier samples rejected (value 0.15 kept; old: inflated), 20 in a row re-seed",
        near(afterBurst, 0.15, 1e-9) and calOld.value > 0.7 and near(cal.value, 0.4, 1e-6) and (cal.reseeds or 0) == 1,
        string.format("band %.3f old %.3f reseeded %.3f", afterBurst, calOld.value, cal.value))
end

-- 2026-10-12 23:27 (user): aim camera a bit further by default
do
  check("aim default distance 1.1 m", CM.config().aimDistance == 1.1, CM.config().aimDistance)
end

-- 2026-10-12 23:40 (user: aiming up / down during the zoom / with a weapon is a drama): the aim / zoom boom position
-- orbits with aimBoomPitchScale of the pitch, the camera orientation keeps the full look pitch
-- 2026-10-13 (review: at 0.35 V left the frame past about -40 / +48): framing limit - the pivot stays within
-- aimBoomFrame of half the vertical FOV from the screen centre (the boom orbits with the look beyond that)
do
  local d = CM.config()
  -- 2026-10-13 round 2: weapon aim frame 0.6 -> 0.8; the unarmed zoom's own scale 0.25 without a framing limit
  -- round 3 (review: at 0.8 the weapon boom followed the look 1:1 past about -35 / +30): weapon frame 0 (opt-in limit)
  check("aim boom pitch defaults: scale 0.35, frame 0 (no limit), aim pitch -85 / 85, zoom scale 0.25 / frame 0",
        d.aimBoomPitchScale == 0.35 and d.aimBoomFrame == 0 and d.aimPitchMin == -85 and d.aimPitchMax == 85
        and d.zoomBoomPitchScale == 0.25 and d.zoomBoomFrame == 0, string.format("%s %s %s %s %s %s",
        tostring(d.aimBoomPitchScale), tostring(d.aimBoomFrame), tostring(d.aimPitchMin), tostring(d.aimPitchMax),
        tostring(d.zoomBoomPitchScale), tostring(d.zoomBoomFrame)))
  -- the framing limit (opt-in, camera.aim.boomFrame 0.8) - the 2026-10-13 rules
  local dF = CM.config({ aimBoomFrame = 0.8 })
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  -- frames at 60 fps with a fixed game look pitch; aim = true -> aim pose (aimDistance 1.1 m) or `pose`, else the profile P
  local function run(cfg, fpitch, aim, frames, S, pose)
    S = S or CM.newSolver(cfg)
    local res
    for _ = 1, frames or 120 do
      res = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fpitch,
        dyaw = 0, dpitch = 0, moving = true, profile = aim and (pose or CM.aimProfile(P, cfg, nil)) or P, side = 1, w = 1,
        aim = aim, aimW = aim and 1 or 0 })
    end
    return res, S
  end
  local function fwdPitch(r) return select(2, CM.yawPitchFromForward(CM.qrot(r.q, V(0, 1, 0)))) end
  local function rel(r) return CM.sub(r.world, r.pivot) end
  -- screen angles (deg) of a world point: vertical (+ = above the centre), horizontal; in front of the camera
  local function sang(r, pt)
    local v = CM.sub(pt, r.world)
    local f = CM.dot(v, CM.qrot(r.q, V(0, 1, 0)))
    return math.deg(math.atan2(CM.dot(v, CM.qrot(r.q, V(0, 0, 1))), f)), math.deg(math.atan2(CM.dot(v, CM.qrot(r.q, V(1, 0, 0))), f)), f
  end
  local function inFrame(r, pt) -- 16:9, vertical FOV = r.fov
    local va, ha, f = sang(r, pt)
    local vh = r.fov / 2
    return f > 0 and math.abs(va) <= vh and math.abs(ha) <= math.deg(math.atan(math.tan(math.rad(vh)) * 16 / 9))
  end
  local lim, b0 = 0.8 * 28, -math.deg(math.atan2(0.05, 1.1)) -- aim pose: FOV 56 (profile, no aim zoom), height 0.05, 1.1 m
  -- moderate pitch: the partial orbit (limit not reached)
  local rm = run(d, -20, true)
  check("aim boom: look -20 -> boom pitch -20 * 0.35 = -7 (partial orbit, limit not reached)", near(rm.boomPitch, -7, 1e-6)
        and near(fwdPitch(rm), -20, 1e-6) and vnear(rel(rm), CM.boomOffset(rm.yaw, -7, 0.45, 1.1, 0.05), 1e-6), vs(rel(rm)))
  -- steep pitch: the lag is limited, V's pivot sits at 0.8 of half the FOV off the centre (camera.aim.boomFrame 0.8)
  local rd = run(dF, -70, true)
  local hd = rel(rd).z
  check("aim boom (frame 0.8): look -70 -> lag limited: boom pitch = -70 - (b0 - 0.8 * 28)", near(rd.boomPitch, -70 - (b0 - lim), 1e-6)
        and vnear(rel(rd), CM.boomOffset(rd.yaw, rd.boomPitch, 0.45, 1.1, 0.05), 1e-6), string.format("%.3f", rd.boomPitch))
  check("aim boom (frame 0.8): look -70 -> camera forward still pitch -70 (the shot direction)", near(fwdPitch(rd), -70, 1e-6), fwdPitch(rd))
  -- (2026-10-13 frame 0.8: ~0.81 m above the pivot; 0.6 gave ~0.9 m, the full orbit ~1.05 m)
  check("aim boom (frame 0.8): look -70 -> pivot 22.4 deg above the centre, camera ~0.81 m above the pivot (full orbit ~1.05 m)",
        near((sang(rd, rd.pivot)), lim, 1e-4) and hd < 0.85 and hd > 0.78, string.format("%.3f / %.3f m", (sang(rd, rd.pivot)), hd))
  local ru = run(dF, 70, true)
  local hu = rel(ru).z
  check("aim boom (frame 0.8): look +70 -> pivot 22.4 deg below the centre, camera ~0.81 m below the pivot (full orbit ~1.0 m)",
        near((sang(ru, ru.pivot)), -lim, 1e-4) and hu > -0.85 and hu < -0.78, string.format("%.3f / %.3f m", (sang(ru, ru.pivot)), hu))
  check("aim boom (frame 0.8): look +70 -> camera forward still pitch +70", near(fwdPitch(ru), 70, 1e-6), fwdPitch(ru))
  -- round 3 defaults (frame 0): the pure 0.35 partial orbit at every pitch - the camera stays within ~0.6 m of the pivot
  -- height (frame 0.8: 0.98 m at +-85, the old full orbit 1.10 m); the camera forward = the look (the shot direction)
  local badD, hMax, hMin = {}, -1, 1
  for p = -85, 85, 5 do
    local r = run(d, p, true, 60)
    local h = rel(r).z
    hMax, hMin = math.max(hMax, h), math.min(hMin, h)
    if not near(r.boomPitch, p * 0.35, 1e-9) or not near(fwdPitch(r), p, 1e-6) or math.abs(h) > 0.6 then
      badD[#badD + 1] = string.format("p%d boom %.3f h %.3f", p, r.boomPitch, h)
    end
  end
  check("aim boom defaults (frame 0): sweep -85..85 -> boom = look * 0.35, camera within 0.6 m of the pivot height, forward = look",
        #badD == 0 and hMax < 0.6 and hMin > -0.55, string.format("h %.3f..%.3f %s", hMin, hMax, table.concat(badD, "; ")))
  -- crouched, looking up +85 while aiming (review: frame 0.8 put the camera 0.07 m above the feet): well above the floor
  local Sc = CM.newSolver(d)
  local rc
  for _ = 1, 120 do
    rc = CM.solve(Sc, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.1, fppPitch = 85,
      dyaw = 0, dpitch = 0, moving = true, profile = CM.aimProfile(P, d, nil), side = 1, w = 1, aim = true, aimW = 1,
      crouch = true })
  end
  check("aim boom defaults: crouched, look +85 -> camera >= 0.5 m above the feet (pivot 1.05 m), forward +85",
        rc.world.z >= 0.5 and near(rc.pivot.z, 1.05, 1e-3) and near(fwdPitch(rc), 85, 1e-6),
        string.format("camera z %.3f pivot %.3f", rc.world.z, rc.pivot.z))
  -- framing sweep -85..85 (camera.aim.boomFrame 0.8, weapon aim rules): the pivot within the limit for the aim pose and
  -- every unarmed zoom step pose (FOV 45..28, 0.9..0.6 m); aim pose: V's right shoulder in the 16:9 frame at every pitch
  -- (at 0.35 without the limit: lost past -40 / +48). 2026-10-13 frame 0.8: the head point (pivot + 0.12 m) may touch
  -- the top edge looking down -40..-50 (28.2-28.5 deg of 28) - the shoulder stays in
  local bad = {}
  local poses = { { "aim", CM.aimProfile(P, dF, nil) } }
  for k = 0, CM.zoomUnarmedSteps(dF) - 1 do poses[#poses + 1] = { "zoom k" .. k, CM.zoomUnarmedPose(dF, k) } end
  for _, ps in ipairs(poses) do
    for p = -85, 85, 5 do
      local r = run(dF, p, true, 60, nil, ps[2])
      local pv = (sang(r, r.pivot))
      if math.abs(pv) > 0.8 * ps[2].fov / 2 + 1e-6 or not near(fwdPitch(r), p, 1e-6) then
        bad[#bad + 1] = string.format("%s p%d pivot %.2f", ps[1], p, pv)
      end
      if ps[1] == "aim" and not inFrame(r, CM.add(r.pivot, V(0.2, 0, -0.05))) then
        bad[#bad + 1] = string.format("aim p%d R shoulder out of frame", p)
      end
    end
  end
  check("aim boom (frame 0.8): framing sweep -85..85 (aim + 4 zoom step poses, weapon rules): pivot within 0.8 x half FOV, R shoulder in frame (aim)",
        #bad == 0, table.concat(bad, "; "))
  -- the framing limit off (aimBoomFrame huge) = the pure 23:40 partial orbit
  local rf = run(CM.config({ aimBoomFrame = 99 }), -70, true)
  check("aim boom: aimBoomFrame 99 (no limit) -> boom pitch -70 * 0.35 (V's pivot ~45 deg off: out of frame)",
        near(rf.boomPitch, -24.5, 1e-6) and not inFrame(rf, rf.pivot), string.format("%.3f", rf.boomPitch))
  -- 2026-10-13 round 2: aimBoomFrame 0 = no framing limit (camera.aim.boomFrame 0): the pure partial orbit for the weapon aim
  local c0 = CM.config({ aimBoomFrame = 0 })
  local z0, z70, z85 = run(c0, 0, true), run(c0, -70, true), run(c0, 85, true)
  check("aim boom: aimBoomFrame 0 -> no limit: boom pitch = look * 0.35 (0 / -70 / +85), camera forward = look",
        z0.boomPitch == 0 and near(z70.boomPitch, -24.5, 1e-9) and near(z85.boomPitch, 85 * 0.35, 1e-9)
        and near(fwdPitch(z70), -70, 1e-6) and vnear(rel(z70), CM.boomOffset(z70.yaw, -24.5, 0.45, 1.1, 0.05), 1e-6),
        string.format("%.3f / %.3f / %.3f", z0.boomPitch, z70.boomPitch, z85.boomPitch))
  local rx = run(d, -95, true)
  check("aim boom: look -95 clamped at the aim min -85 (camera forward)", near(rx.pitch, -85, 1e-6) and near(fwdPitch(rx), -85, 1e-6), rx.pitch)
  -- not aiming: the full-pitch orbit, bit-identical
  local rn = run(d, -40, false)
  check("aim boom: not aiming -> boom = full-pitch orbit (profile P, pitch -40)", rn.boomPitch == rn.pitch and near(rn.pitch, -40, 1e-6)
        and vnear(rel(rn), CM.boomOffset(rn.yaw, rn.pitch, P.side, P.distance, P.height), 1e-9), vs(rel(rn)))
  -- exit tween (aim off, aim weight 0.5): scale blended by the weight (-40: lag 13 deg, inside the framing limit)
  local _, Se = run(d, -40, true)
  local re = CM.solve(Se, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = -40,
    dyaw = 0, dpitch = 0, moving = true, profile = CM.profileLerp(P, CM.aimProfile(P, d, nil), 0.5), side = 1, w = 1,
    aim = false, aimW = 0.5 })
  check("aim boom: exit tween (aim weight 0.5) -> boom pitch = pitch * (1 + (0.35 - 1) * 0.5)",
        near(re.boomPitch, re.pitch * (1 - 0.65 * 0.5), 1e-9) and near(fwdPitch(re), re.pitch, 1e-6),
        string.format("%.3f / %.3f", re.boomPitch, re.pitch))
  -- camera.aim.boomPitchScale 1 = the old full orbit while aiming
  local r1 = run(CM.config({ aimBoomPitchScale = 1 }), -70, true)
  check("aim boom: scale 1 -> old full orbit while aiming (~1.05 m above the pivot at -70)",
        vnear(rel(r1), CM.boomOffset(r1.yaw, -70, 0.45, 1.1, 0.05), 1e-6) and rel(r1).z > 1.0, vs(rel(r1)))

  -- 2026-10-13 round 2 (review finding 1: the framing limit at the narrow zoom FOV fell back to ~the full orbit - camera
  -- ~1 m over V's head looking straight down): the unarmed zoom (inp.zoomOnly) orbits with
  -- zoomBoomPitchScale 0.25 and no framing limit (zoomBoomFrame 0); the weapon aim keeps 0.35 (frame 0.8 = opt-in, dF)
  local function runZ(cfg, fpitch, pose, zo, S)
    S = S or CM.newSolver(cfg)
    local res
    for _ = 1, 60 do
      res = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fpitch,
        dyaw = 0, dpitch = 0, moving = true, profile = pose, side = 1, w = 1, aim = true, aimW = 1, zoomOnly = zo })
    end
    return res, S
  end
  local k3 = CM.zoomUnarmedPose(d, 3)
  local zk = runZ(d, -70, k3, true)
  check("zoom boom: step k3 (0.6 m, FOV 28) at -70 -> pure 0.25 partial orbit: boom -17.5, camera <= 0.30 m above the pivot, forward -70",
        near(k3.distance, 0.6, 1e-9) and near(k3.fov, 28, 1e-9) and near(zk.fov, 28, 1e-9) and near(zk.boomPitch, -17.5, 1e-9)
        and rel(zk).z <= 0.30 and rel(zk).z > 0 and near(zk.pitch, -70, 1e-6) and near(fwdPitch(zk), -70, 1e-6)
        and vnear(rel(zk), CM.boomOffset(zk.yaw, -17.5, 0.45, 0.6, 0.05), 1e-9),
        string.format("boom %.3f height %.3f m fwd %.3f", zk.boomPitch, rel(zk).z, fwdPitch(zk)))
  local wk = runZ(dF, -70, k3, false)
  local limK, b0K = 0.8 * 28 / 2, -math.deg(math.atan2(0.05, 0.6))
  check("zoom boom: the same pose as the weapon aim (zoomOnly false, aimBoomFrame 0.8) -> V's pivot within 0.8 x half FOV (11.2 deg)",
        math.abs((sang(wk, wk.pivot))) <= limK + 1e-6 and near(wk.boomPitch, -70 - (b0K - limK), 1e-6)
        and near(fwdPitch(wk), -70, 1e-6) and rel(wk).z > 0.45,
        string.format("pivot %.3f deg boom %.3f height %.3f m", (sang(wk, wk.pivot)), wk.boomPitch, rel(wk).z))
  local badZ = {}
  for k = 0, CM.zoomUnarmedSteps(d) - 1 do
    local pose = CM.zoomUnarmedPose(d, k)
    for pp = -85, 85, 5 do
      local r = runZ(d, pp, pose, true)
      if not near(r.boomPitch, pp * 0.25, 1e-9) or not near(fwdPitch(r), pp, 1e-6) then
        badZ[#badZ + 1] = string.format("k%d p%d boom %.3f", k, pp, r.boomPitch)
      end
    end
  end
  check("zoom boom: sweep -85..85 x 4 steps -> boom pitch = look * 0.25 (no framing limit), camera forward = look",
        #badZ == 0, table.concat(badZ, "; "))
  local zf = runZ(CM.config({ zoomBoomFrame = 0.8 }), -70, k3, true)
  check("zoom boom: zoomBoomFrame 0.8 (camera.zoomUnarmed.boomFrame) -> the framing limit applies (pivot at 0.8 x half FOV)",
        near((sang(zf, zf.pivot)), limK, 1e-4) and near(zf.boomPitch, wk.boomPitch, 1e-6), string.format("%.3f", zf.boomPitch))
  local z1 = runZ(CM.config({ zoomBoomPitchScale = 1 }), -70, k3, true)
  check("zoom boom: zoomBoomPitchScale 1 -> full orbit in the zoom (aimBoomPitchScale 0.35 not used)",
        near(z1.boomPitch, -70, 1e-9), string.format("%.3f", z1.boomPitch))
  -- zoom exit tween (aimW 0.5, zoomOnly from ZU.w > A.w in init.lua): the zoom scale blended by the weight
  local _, Sz = runZ(d, -40, k3, true)
  local rz = CM.solve(Sz, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = -40,
    dyaw = 0, dpitch = 0, moving = true, profile = CM.profileLerp(P, k3, 0.5), side = 1, w = 1, aim = false, aimW = 0.5,
    zoomOnly = true })
  check("zoom boom: exit tween (weight 0.5, zoomOnly) -> boom pitch = pitch * (1 + (0.25 - 1) * 0.5)",
        near(rz.boomPitch, rz.pitch * (1 - 0.75 * 0.5), 1e-9), string.format("%.3f / %.3f", rz.boomPitch, rz.pitch))
  -- round 3 (review: zoomOnly switched the rule set in one frame when aim / zoom tweens overlapped): inp.zoomShare blends
  -- the two lags linearly (0 = weapon rules, 1 = zoom rules = zoomOnly, bit-identical at the ends)
  local function runS(cfg, fpitch, share, zo)
    local S = CM.newSolver(cfg)
    local res
    for _ = 1, 60 do
      res = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fpitch,
        dyaw = 0, dpitch = 0, moving = true, profile = k3, side = 1, w = 1, aim = true, aimW = 1, zoomShare = share,
        zoomOnly = zo })
    end
    return res
  end
  local badS = {}
  for _, cf in ipairs({ { "defaults", d }, { "frame 0.8", dF } }) do
    for _, pp in ipairs({ -85, -70, -30, 40, 70, 85 }) do
      local bA, bZ = runS(cf[2], pp, nil, false).boomPitch, runS(cf[2], pp, nil, true).boomPitch
      if runS(cf[2], pp, 0).boomPitch ~= bA or runS(cf[2], pp, 1).boomPitch ~= bZ then
        badS[#badS + 1] = string.format("%s p%d ends", cf[1], pp)
      end
      for i = 1, 19 do
        local sh = i / 20
        local b = runS(cf[2], pp, sh).boomPitch
        if not near(b, bA + (bZ - bA) * sh, 1e-9) then badS[#badS + 1] = string.format("%s p%d s%.2f %.4f", cf[1], pp, sh, b) end
      end
    end
  end
  check("zoom boom: inp.zoomShare 0..1 -> boom pitch linear between the weapon and the zoom rules (ends = zoomOnly false / true)",
        #badS == 0, table.concat(badS, "; "))
  -- overlapping tweens at 60 fps through the real aim / zoom steps and composes: weapon aim exit + unarmed zoom enter on
  -- the same frame (and the reverse); the largest camera step per frame stays close to the same exit / enter without
  -- the overlap (the review measured 0.82 m vs 0.25 m with the on-flag switch at frame 0.8)
  local function overlap(cfg, fpitch, order, shareFn)
    local A, Z = CM.newAim(), CM.newZoomUnarmed()
    local S = CM.newSolver(cfg)
    local prev, maxStep = nil, 0
    for i = 1, 150 do
      local wantA, wantZ
      if order == "aim->zoom" then wantA, wantZ = i <= 60, i > 60
      elseif order == "zoom->aim" then wantA, wantZ = i > 60, i <= 60
      elseif order == "aim exit" then wantA, wantZ = i <= 60, false
      else wantA, wantZ = false, i > 60 end -- "zoom enter"
      CM.aimStep(A, cfg, 1 / 60, wantA, "t")
      CM.zoomUnarmedStep(Z, cfg, 1 / 60, wantZ, "t", 0)
      local p = CM.zoomUnarmedCompose(CM.aimCompose(P, A, cfg, nil), Z)
      local r = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fpitch,
        dyaw = 0, dpitch = 0, moving = true, profile = p, side = 1, w = 1, aim = A.on or Z.on, aimW = math.max(A.w, Z.w),
        fovDirect = true, zoomShare = shareFn(A, Z) })
      if prev and i > 55 then maxStep = math.max(maxStep, CM.dist(prev, r.world)) end
      prev = r.world
    end
    return maxStep
  end
  local function share(A, Z) return Z.w / math.max(A.w + Z.w, 1e-6) end
  local function flag(A, Z) return ((Z.on and not A.on) or (not A.on and not Z.on and Z.w > A.w)) and 1 or 0 end
  local badO = {}
  for _, cf in ipairs({ { "defaults", d }, { "frame 0.8", dF } }) do
    for _, pp in ipairs({ -85, -70, 70, 85 }) do
      local ref = math.max(overlap(cf[2], pp, "aim exit", share), overlap(cf[2], pp, "zoom enter", share))
      for _, ord in ipairs({ "aim->zoom", "zoom->aim" }) do
        local m = overlap(cf[2], pp, ord, share)
        if m > ref + 0.05 then badO[#badO + 1] = string.format("%s p%d %s %.3f (ref %.3f, flag %.3f)", cf[1], pp, ord, m, ref,
          overlap(cf[2], pp, ord, flag)) end
      end
    end
  end
  local flagJump = overlap(dF, 85, "aim->zoom", flag)
  check("zoom boom: overlapping aim exit / zoom enter (both orders, -85..85, frame 0 and 0.8): per-frame camera step <= the "
        .. "non-overlapping tweens + 0.05 m (the on-flag switch: " .. string.format("%.2f m", flagJump) .. " at +85, frame 0.8)",
        #badO == 0 and flagJump > overlap(dF, 85, "aim->zoom", share) + 0.2, table.concat(badO, "; "))

  -- review finding 2: the collision rays follow the boom (boomPitch), not the look pitch - the centre ray ends at
  -- pivot + boomOffset(yaw, boomPitch, ...) and the edge rays are shifted along the boom orientation's up axis
  local cfgR = CM.config({ fullEvery = 1 })
  local calls = {}
  local SR = CM.newSolver(cfgR)
  local rr
  for i = 1, 120 do
    calls = {}
    rr = CM.solve(SR, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = -70,
      dyaw = 0, dpitch = 0, moving = true, profile = CM.aimProfile(P, cfgR, nil), side = 1, w = 1, aim = true, aimW = 1,
      raycast = function(from, to) calls[#calls + 1] = { from = from, to = to }; return nil end })
  end
  local upB = CM.qrot(CM.qFromYawPitch(rr.yaw, rr.boomPitch), V(0, 0, 1))
  local upL = CM.qrot(CM.qFromYawPitch(rr.yaw, rr.pitch), V(0, 0, 1))
  local tgt = CM.add(rr.pivot, CM.boomOffset(rr.yaw, rr.boomPitch, 0.45, 1.1, 0.05))
  local rad = cfgR.radius
  check("collision rays follow the boom: centre ray pivot -> pivot + boomOffset(yaw, boomPitch, ...), boom pitch != look pitch",
        #calls == 5 and math.abs(rr.boomPitch - rr.pitch) > 20 and vnear(calls[1].from, rr.pivot, 1e-9)
        and vnear(calls[1].to, tgt, 1e-9) and vnear(rr.world, tgt, 1e-9),
        string.format("%d rays, boom %.2f look %.2f", #calls, rr.boomPitch, rr.pitch))
  check("collision rays follow the boom: up / down edge rays offset along the boom's up axis (not the look's)",
        #calls == 5 and vnear(CM.sub(calls[4].from, rr.pivot), CM.scale(upB, rad), 1e-9)
        and vnear(CM.sub(calls[4].to, tgt), CM.scale(upB, rad), 1e-9)
        and vnear(CM.sub(calls[5].from, rr.pivot), CM.scale(upB, -rad), 1e-9)
        and CM.dist(CM.sub(calls[4].from, rr.pivot), CM.scale(upL, rad)) > 0.05,
        #calls == 5 and vs(CM.sub(calls[4].from, rr.pivot)) or tostring(#calls))
end

-- 2026-10-13 parallax-correct game look (review finding 3): CM.anglesEyeToPoint(eye, point) -> yaw, pitch
do
  local eye = V(0, 0, 1.6)
  local y, p = CM.anglesEyeToPoint(eye, V(0, 10, -0.4))
  check("anglesEyeToPoint: point 10 m ahead and 2 m lower -> yaw 0, pitch -deg(atan2(2, 10))",
        y ~= nil and near(y, 0, 1e-9) and near(p, -math.deg(math.atan2(2, 10)), 1e-9), string.format("%s / %s", tostring(y), tostring(p)))
  local bad = {}
  for _, yp in ipairs({ { 30, -10 }, { -120, 25 }, { 170, -60 }, { -179, 5 }, { 90, 0 }, { 0, 80 } }) do
    local yy, pp = CM.anglesEyeToPoint(eye, CM.add(eye, CM.scale(CM.forward(yp[1], yp[2]), 7)))
    if not (yy and near(CM.wrapDeg(yy - yp[1]), 0, 1e-9) and near(pp, yp[2], 1e-9)) then
      bad[#bad + 1] = string.format("%d/%d -> %s/%s", yp[1], yp[2], tostring(yy), tostring(pp))
    end
  end
  check("anglesEyeToPoint: CM.forward round trip (yaw atan2(-x, y), pitch > 0 = up)", #bad == 0, table.concat(bad, "; "))
  check("anglesEyeToPoint: point = eye -> nil", CM.anglesEyeToPoint(eye, V(0, 0, 1.6)) == nil)
end

-- 2026-10-13 round 3 (review): CM.lookParallaxStep - the correction of the written game look is limited per axis to
-- aimLookMaxDeg (10) and moves at <= aimLookRate (100) deg/s; nil target = back to the camera
do
  local cfg = CM.config()
  check("parallax defaults: max 10 deg per axis, rate 100 deg/s, release flush <= 3 frames",
        cfg.aimLookMaxDeg == 10 and cfg.aimLookRate == 100 and cfg.aimLookFlush == 3)
  -- steep pitch, floor hit (review: weapon -80, P on the floor beside V -> raw yaw delta ~-109 deg)
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  local function aimAt(fp)
    local S = CM.newSolver(cfg)
    local r
    for _ = 1, 60 do
      r = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fp,
        dyaw = 0, dpitch = 0, moving = true, profile = CM.aimProfile(P, cfg, nil), side = 1, w = 1, aim = true, aimW = 1 })
    end
    return r
  end
  local eye = V(0, 0.1, 1.6)
  local bad, worst = {}, 0
  for _, fp in ipairs({ -60, -70, -80, -85 }) do
    local r = aimAt(fp)
    local f = CM.forward(r.yaw, r.pitch)
    local Pf = CM.add(r.world, CM.scale(f, -r.world.z / f.z)) -- the floor z = 0 under the crosshair
    local ry, rp = CM.anglesEyeToPoint(eye, Pf)
    worst = math.max(worst, math.abs(CM.wrapDeg(ry - r.yaw)))
    local st, y, p = {}, nil, nil
    local prevY, prevP, maxStep = r.yaw, r.pitch, 0
    for _ = 1, 30 do
      y, p = CM.lookParallaxStep(st, cfg, 1 / 60, r.yaw, r.pitch, ry, rp)
      maxStep = math.max(maxStep, math.abs(CM.wrapDeg(y - prevY)), math.abs(p - prevP))
      prevY, prevP = y, p
    end
    local dy, dp = CM.wrapDeg(y - r.yaw), p - r.pitch
    local ey = CM.clamp(CM.wrapDeg(ry - r.yaw), -10, 10)
    local ep = CM.clamp(rp - r.pitch, -10, 10)
    if math.abs(dy) > 10 + 1e-9 or math.abs(dp) > 10 + 1e-9 or not near(dy, ey, 1e-9)
        or not near(p, CM.clamp(r.pitch + ep, -85, 85), 1e-9) or maxStep > 100 / 60 + 1e-9 then
      bad[#bad + 1] = string.format("p%d raw %.1f/%.1f written %.2f/%.2f step %.3f", fp, CM.wrapDeg(ry - r.yaw), rp - r.pitch,
        dy, dp, maxStep)
    end
  end
  check("parallax: steep pitch floor hit (-60..-85, raw yaw delta > 45 deg) -> written correction <= 10 deg per axis, "
        .. "<= 100 deg/s per frame, pitch within -85..85", #bad == 0 and worst > 45,
        string.format("worst raw %.1f | %s", worst, table.concat(bad, "; ")))
  -- a near / far edge (review: 23.8 deg in one frame): the written look moves <= 1.67 deg per frame at 60 fps and reaches
  -- the (bounded) target exactly
  local st = {}
  local y, p
  for _ = 1, 30 do y, p = CM.lookParallaxStep(st, cfg, 1 / 60, 0, 0, -0.44, 0.1) end
  local y0 = y
  local steps, maxStep = 0, 0
  for _ = 1, 30 do
    local py = y
    y, p = CM.lookParallaxStep(st, cfg, 1 / 60, 0, 0, -24.23, 7.8)
    maxStep = math.max(maxStep, math.abs(y - py))
    if math.abs(y - py) > 1e-12 then steps = steps + 1 end
  end
  check("parallax: near / far edge -0.44 -> -24.23 deg: <= 100/60 deg per frame, ends exactly at the 10 deg bound",
        near(y0, -0.44, 1e-12) and maxStep <= 100 / 60 + 1e-9 and steps >= 5 and near(y, -10, 1e-12) and near(p, 7.8, 1e-12),
        string.format("%.3f -> %.3f (%d frames, max %.3f) pitch %.3f", y0, y, steps, maxStep, p))
  -- no target (no ray / hold over): back to the camera at the same rate, exactly 0
  for _ = 1, 30 do y, p = CM.lookParallaxStep(st, cfg, 1 / 60, 5, -20, nil, nil) end
  check("parallax: no target -> glides back to the camera yaw / pitch (exact)", y == 5 and p == -20 and st.dy == 0 and st.dp == 0,
        string.format("%s / %s", tostring(y), tostring(p)))
  -- rate 0 = at once; pitch output inside aimPitchMin..aimPitchMax; yaw wrap at +-180
  local c0 = CM.config({ aimLookRate = 0 })
  local s0 = {}
  y, p = CM.lookParallaxStep(s0, c0, 1 / 60, 178, -84, -176, -95)
  check("parallax: aimLookRate 0 -> at once; yaw across +-180 (+6 deg), pitch -84 - 10 clamped at the aim min -85",
        near(CM.wrapDeg(y - (-176)), 0, 1e-9) and near(s0.dy, 6, 1e-9) and near(p, -85, 1e-12), string.format("%.3f / %.3f", y, p))
end

-- 2026-10-13: while the game's look is written (UTPP.dll 0.6.5) the FPP pitch read back is our own target (the native
-- replaces the game's mouse delta): aiming with env.lookEcho integrates the mouse itself (no freeze, no feedback of a
-- parallax corrected target); when the echo ends the offset to the game's pitch is re-based and aligned
do
  local cfg = CM.config()
  local L = CM.newLook()
  local env = { bodyYaw = 0, fppPitch = -10, moving = true, aim = true, aimW = 1 }
  for _ = 1, 60 do CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  local p0 = L.pitch
  env.lookEcho = true
  local p
  for _ = 1, 30 do env.fppPitch = L.pitch; _, p = CM.lookStep(L, cfg, 1 / 60, 0, -0.5, env) end
  check("aim + look echo: the aim pitch integrates the mouse (-10 - 30 x 0.5 = -25), not frozen on the echo",
        near(p0, -10, 1e-6) and near(p, -25, 1e-6), string.format("%.3f -> %.3f", p0, p))
  for _ = 1, 30 do env.fppPitch = L.pitch + 4; _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("aim + look echo of a corrected pitch (+4 deg): no feedback drift", near(p, -25, 1e-6), p)
  env.lookEcho, env.fppPitch = false, -20
  _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  local p1 = p
  for _ = 1, 60 do _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("echo ended: continues from the shown pitch (no jump), then aligns onto the game's pitch -20",
        math.abs(p1 + 25) < 1 and near(p, -20, 1e-3), string.format("%.3f -> %.3f", p1, p))
  -- aim ends while echoing: the game's look keeps the last written (parallax corrected) pitch -> the follow pitch glides.
  -- Round 3 (review: the old check fed a sequence init.lua never produces): the release frame still reads our echo
  -- (init.lua: lwSent of the last aim frame, corrected = the camera pitch), the game's raw (corrected) pitch arrives one
  -- frame later - it must glide, not snap (the recovery stays on while lookEcho)
  env.lookEcho = true
  for _ = 1, 10 do env.fppPitch = L.pitch + 4; _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  local pa = p
  env.aim, env.aimW, env.lookEcho, env.fppPitch = false, 0, true, pa
  _, p1 = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  env.lookEcho, env.fppPitch = false, pa + 4
  local _, p2 = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  local maxD, pp = 0, p2
  for _ = 1, 60 do _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env); maxD = math.max(maxD, math.abs(p - pp)); pp = p end
  check("aim ended during the echo (release frame echo, raw pitch a frame later): glides onto the game's pitch, no 4 deg jump",
        near(p1, pa, 1e-9) and p2 - pa > 0.1 and p2 - pa < 1.5 and maxD < 1.5 and near(p, pa + 4, 1e-9),
        string.format("%.3f -> %.3f -> %.3f -> %.3f (max step %.3f)", pa, p1, p2, p, maxD))
  -- through CM.solve (inp.lookEcho)
  local S = CM.newSolver(cfg)
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  local res
  local function step(fp, dp, echo)
    return CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fp,
      dyaw = 0, dpitch = dp, moving = true, profile = CM.aimProfile(P, cfg, nil), side = 1, w = 1, aim = true, aimW = 1,
      lookEcho = echo })
  end
  for _ = 1, 60 do res = step(-5, 0, false) end
  for _ = 1, 20 do res = step(res.pitch + 3, 0.25, true) end
  check("solve inp.lookEcho: aim pitch = own integration (-5 + 20 x 0.25 = 0), the echo + 3 deg ignored",
        near(res.pitch, 0, 1e-6), res.pitch)
end

-- 0.6.5b (user 01:16: aiming jerked up / down / left / right with the game's look): while the game's look is written
-- (env.lookWriteOn) the aim / zoom pitch and yaw are the camera's own from the first aim frame - the game's look pitch /
-- yaw jumping every frame (no echo flag) moves nothing; the mouse still does. Write off (tripped / setting) while
-- aiming: the game's pitch again, re-based from the shown pitch (no jump), then aligned
do
  local cfg = CM.config()
  local L = CM.newLook()
  local env = { bodyYaw = 0, fppPitch = -10, moving = true, aim = false, aimW = 0 }
  for _ = 1, 30 do CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  env.aim, env.aimW, env.lookWriteOn = true, 1, true
  local y0, p0 = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  local maxDY, maxDP = 0, 0
  for i = 1, 120 do
    env.fppPitch = -10 + ((i % 2 == 0) and 30 or -30)
    env.refYaw = (i % 3 == 0) and 25 or -15
    local y, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
    maxDY = math.max(maxDY, math.abs(CM.wrapDeg(y - y0)))
    maxDP = math.max(maxDP, math.abs(p - p0))
  end
  check("lookWriteOn: own aim pitch / yaw from the first aim frame; the game's look pitch +-30 / yaw jitter moves nothing",
        near(p0, -10, 1e-9) and maxDY < 1e-9 and maxDP < 1e-9, string.format("p0 %.3f maxDY %.6f maxDP %.6f", p0, maxDY, maxDP))
  local p
  for _ = 1, 30 do env.fppPitch = 40; _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0.5, env) end
  check("lookWriteOn: the mouse moves the aim pitch (-10 + 30 x 0.5 = 5), the game's pitch 40 ignored", near(p, 5, 1e-6), p)
  env.lookWriteOn, env.fppPitch, env.refYaw = false, -20, nil
  local _, q1 = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  local q
  for _ = 1, 60 do _, q = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  -- (the 25 deg offset glides with CM.alignStep: the first frame moves 25 x (1 - e^(-10/60)) = 3.8 deg, no snap)
  check("write off while aiming: continues from the shown pitch (glide, no snap onto -20), then aligns onto -20",
        q1 < 5 and 5 - q1 < 4 and near(q, -20, 1e-3), string.format("%.3f -> %.3f", q1, q))
  -- aimOwnYaw off: without the write the aim yaw follows the game's look (refYaw); with the write it is the camera's own
  local cfgN = CM.config({ aimOwnYaw = false })
  local function yawJitter(on)
    local L2 = CM.newLook()
    local e2 = { bodyYaw = 0, fppPitch = 0, moving = true, aim = false, aimW = 0 }
    for _ = 1, 30 do CM.lookStep(L2, cfgN, 1 / 60, 0, 0, e2) end
    e2.aim, e2.aimW, e2.lookWriteOn, e2.refYaw = true, 1, on, 0
    local ya = CM.lookStep(L2, cfgN, 1 / 60, 0, 0, e2)
    local m = 0
    for i = 1, 60 do
      e2.refYaw = (i % 2 == 0) and 20 or -20
      local y = CM.lookStep(L2, cfgN, 1 / 60, 0, 0, e2)
      m = math.max(m, math.abs(CM.wrapDeg(y - ya)))
    end
    return m
  end
  local mOff, mOn = yawJitter(false), yawJitter(true)
  check("aimOwnYaw off: the game's look yaw jitter moves the aim yaw without the write, not with it (lookWriteOn)",
        mOff > 1 and mOn < 1e-9, string.format("off %.3f on %.6f", mOff, mOn))
  -- through CM.solve (inp.lookWriteOn)
  local S = CM.newSolver(cfg)
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  local res
  local function step(fp, dp, on)
    return CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fp,
      dyaw = 0, dpitch = dp, moving = true, profile = CM.aimProfile(P, cfg, nil), side = 1, w = 1, aim = true, aimW = 1,
      lookWriteOn = on })
  end
  for _ = 1, 60 do res = step(-5, 0, false) end
  local okJ = near(res.pitch, -5, 1e-3)
  for i = 1, 20 do res = step((i % 2 == 0) and 25 or -35, 0.25, true) end
  check("solve inp.lookWriteOn: aim pitch = own integration (-5 + 20 x 0.25 = 0), the game's pitch jumping +-30 ignored",
        okJ and near(res.pitch, 0, 1e-6), res.pitch)
end

-- 0.6.5b round 2 (review: a recording ran without the write; the aim pitch read the game's look pitch,
-- which utpp_bodyturn's aim align adjustTransform overrode for a frame: 52.9 -> -8.7 -> 51.1). env.adjustPending: the
-- aim pitch does not read fppPitch in such a frame (the mouse is integrated), the offset stays for the next clean frame;
-- an aim that starts in such a frame takes its offset at the first clean frame
do
  local cfg = CM.config()
  local function run(pending)
    local L = CM.newLook()
    local env = { bodyYaw = 0, fppPitch = 20, moving = true, aim = true, aimW = 1 }
    local p
    for _ = 1, 180 do _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
    local p0 = p
    local maxDev = 0
    -- the game's own pitch moves with the mouse (0.5 deg per frame); every 6th frame the override (pitch -8.7)
    local game = 20
    for i = 1, 60 do
      game = game + 0.5
      local over = i % 6 == 0
      env.fppPitch = over and -8.7 or game
      env.adjustPending = pending and over or nil
      _, p = CM.lookStep(L, cfg, 1 / 60, 0, 0.5, env)
      maxDev = math.max(maxDev, math.abs(p - game))
    end
    return p0, maxDev, p, game
  end
  local p0, devOn, pEnd, gEnd = run(true)
  local _, devOff = run(false)
  -- 2026-10-13 recheck: the override frame marks the echo, the next clean frame re-takes the offset and glides it out
  -- (aimAlignMin) - a sub-degree lag instead of the exact follow; the jerk was ~30 deg
  check("adjustPending: the aim pitch ignores the override frames (pitch -8.7) and follows the mouse / the game's pitch",
        near(p0, 20, 1e-3) and devOn < 0.5 and math.abs(pEnd - gEnd) < 0.5, string.format("p0 %.3f dev %.6f end %.3f/%.3f",
        p0, devOn, pEnd, gEnd))
  -- recheck finding: the mouse pitch and the game's pitch differ in scale during a long align (mouse 0.5, game 0.2 per
  -- frame, every frame overridden for 1 s) - the first clean frame must not jump by the accumulated difference
  do
    local L = CM.newLook()
    local env = { bodyYaw = 0, fppPitch = 10, moving = true, aim = true, aimW = 1 }
    for _ = 1, 60 do CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
    local game, prev, maxStep = 10, L.pitch, 0
    for i = 1, 90 do
      game = game + 0.2
      local over = i <= 60
      env.fppPitch = over and -8.7 or game
      env.adjustPending = over or nil
      local _, pp = CM.lookStep(L, cfg, 1 / 60, 0, i <= 60 and 0.5 or 0, env)
      maxStep = math.max(maxStep, math.abs(pp - prev)); prev = pp
    end
    check("adjustPending: mouse/game pitch scale mismatch during the align -> the accumulated 18 deg glide out (<= 4 deg/frame, align glide), no one-frame jump",
          maxStep <= 4.0, string.format("max step %.3f", maxStep))
  end
  check("adjustPending (control): without the flag the same frames drag the aim pitch by ~30 deg (the reported jerk)",
        devOff > 25, devOff)
  -- an aim that starts in an override frame: the offset is taken at the first clean frame (no glide toward a wrong pitch)
  local L = CM.newLook()
  local env = { bodyYaw = 0, fppPitch = 10, moving = true }
  for _ = 1, 60 do CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  local _, pS = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  env.aim, env.aimW, env.fppPitch, env.adjustPending = true, 1, -30, true
  local _, pA = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  env.fppPitch, env.adjustPending = 10, nil
  local ps = {}
  for i = 1, 30 do _, ps[i] = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  local maxStep = 0
  for i = 2, 30 do maxStep = math.max(maxStep, math.abs(ps[i] - ps[i - 1])) end
  check("adjustPending at the aim start: the pitch stays (10), offset taken at the first clean frame, no glide (game 10)",
        near(pS, 10, 1e-6) and near(pA, 10, 1e-6) and maxStep < 1e-6 and near(ps[30], 10, 1e-6),
        string.format("start %.3f aim %.3f end %.3f maxStep %.6f", pS, pA, ps[30], maxStep))
  -- through CM.solve (inp.adjustPending)
  local S = CM.newSolver(cfg)
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  local res
  local function step(fp, pend)
    return CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.62, fppPitch = fp,
      dyaw = 0, dpitch = 0, moving = true, profile = CM.aimProfile(P, cfg, nil), side = 1, w = 1, aim = true, aimW = 1,
      adjustPending = pend })
  end
  for _ = 1, 120 do res = step(15, nil) end
  local r1 = res.pitch
  res = step(-8.7, true)
  local r2 = res.pitch
  res = step(15, nil)
  check("solve inp.adjustPending: the override frame (-8.7) does not move the aim pitch (15)",
        near(r1, 15, 1e-3) and near(r2, r1, 1e-9) and near(res.pitch, 15, 1e-3), string.format("%.3f %.3f %.3f", r1, r2,
        res.pitch))
end

-- 2026-10-12 user test 22:23 (world camera): aiming keeps the camera's own yaw (mouse), no FOV zoom without a scope
do
  local cfg = CM.config()
  check("aim defaults: own yaw on, FOV zoom off", cfg.aimOwnYaw == true and cfg.aimZoomFov == false)
  local L = CM.newLook()
  local env = { bodyYaw = 0, refYaw = 170, fppPitch = 0, moving = false, decoupled = true }
  for _ = 1, 10 do CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  env.decoupled, env.aim, env.aimW = false, true, 1
  local y
  for _ = 1, 60 do y = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("aim own yaw: the camera keeps its yaw (no glide onto the game's look 170 deg away)", near(y, 0, 1e-6), y)
  for _ = 1, 30 do env.bodyYaw = env.bodyYaw + 6; y = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("aim own yaw: V turning (aim align) does not move the camera", near(y, 0, 1e-6), y)
  for _ = 1, 10 do y = CM.lookStep(L, cfg, 1 / 60, 2, 0, env) end
  check("aim own yaw: the mouse turns the camera", near(y, 20, 1e-6), y)
  local P = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  check("aim without FOV zoom: aim pose keeps the profile FOV (weapon AimFOV 40 ignored)", near(CM.aimProfile(P, cfg, 40).fov, 56))
  check("aim with camera.aim.zoomFov: weapon AimFOV used", near(CM.aimProfile(P, CM.config({ aimZoomFov = true }), 40).fov, 40))
end

-- 2026-10-12 user test 21:46 (30 fps): V turns 17-19 deg per frame; the old 15 deg cap left those steps uncompensated
do
  local cfg = CM.config()
  check("predict: 18 deg body step at 30 fps is compensated (cap >= 60)", near(CM.predictTurn(18, 1 / 30, 1 / 30, cfg.predictMaxStep), 18, 1e-9)
    and cfg.predictMaxStep >= 60, cfg.predictMaxStep)
end

-- 2026-10-12 user test 21:23: pitchSource=input must not drive the aim / zoom pitch; crouch caps the orbit look-up
do
  local cfg = CM.config({ pitchSource = "input" })
  local L = CM.newLook()
  local env = { bodyYaw = 0, fppPitch = -40, moving = true }
  for _ = 1, 30 do CM.lookStep(L, cfg, 1 / 60, 0, -3, env) end -- own mouse pitch runs down (input mode, follow)
  local own = L.pitch
  env.aim, env.aimW = true, 1
  local p
  for _ = 1, 60 do _, p = CM.lookStep(L, cfg, 1 / 60, 0, -3, env) end
  check("pitchSource=input: aiming follows the game's look pitch (-40), not the own mouse pitch", own < -60 and near(p, -40, 1e-3),
    string.format("own %.1f aim %.1f", own, p))
  env.aim, env.aimW = false, 0
  local _, p2 = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  check("pitchSource=input: aim exit continues from the shown pitch (no jump)", near(p2, -40, 1e-3), p2)
  local Lc = CM.newLook()
  local envc = { bodyYaw = 0, fppPitch = 0, moving = false, freeHeld = true, crouch = true }
  local pc
  for _ = 1, 60 do _, pc = CM.lookStep(Lc, cfg, 1 / 60, 0, 5, envc) end
  check("crouch: orbit look-up capped at crouchPitchMax 30", near(pc, 30, 1e-6), pc)
  envc.crouch = false
  for _ = 1, 60 do _, pc = CM.lookStep(Lc, cfg, 1 / 60, 0, 5, envc) end
  check("standing: orbit look-up keeps pitchMax 55", near(pc, 55, 1e-6), pc)
end

-- ================================================================ 0.1.6 unarmed zoom (RMB with empty hands)
do
  -- 2026-10-12: defaults 4 steps, no first person handoff; the legacy 3-step / handoff scenarios below
  -- set both explicitly
  local cfgDef = CM.config()
  check("zoomUnarmed defaults: on, 4 steps, 0.9 -> 0.6 m, FOV 45 -> 28, blend 0.15 s, enter delay 0.1 s, handoff OFF",
        cfgDef.zoomUnarmedEnabled == true and CM.zoomUnarmedSteps(cfgDef) == 4 and cfgDef.zoomUnarmedDistance == 0.9
        and cfgDef.zoomUnarmedLastDistance == 0.6 and cfgDef.zoomUnarmedFov == 45 and cfgDef.zoomUnarmedLastFov == 28
        and cfgDef.zoomUnarmedBlend == 0.15 and cfgDef.zoomUnarmedEnterDelay == 0.1 and cfgDef.zoomUnarmedFppHandoff == false)
  local pd1, pd3 = CM.zoomUnarmedPose(cfgDef, 1), CM.zoomUnarmedPose(cfgDef, 3)
  check("zoomUnarmed defaults: 4 steps = 0.90 / 0.80 / 0.70 / 0.60 m, FOV 45 / 38.4 / 32.8 / 28", near(pd1.distance, 0.8)
        and near(pd3.distance, 0.6) and near(pd1.fov, 45 * (28 / 45) ^ (1 / 3)) and near(pd3.fov, 28))
  local cfg = CM.config({ zoomUnarmedSteps = 3, zoomUnarmedFppHandoff = true })
  -- step -> distance / FOV
  local p0, p1, p2 = CM.zoomUnarmedPose(cfg, 0), CM.zoomUnarmedPose(cfg, 1), CM.zoomUnarmedPose(cfg, 2)
  check("zoomUnarmed pose: distance linear 0.90 / 0.75 / 0.60", near(p0.distance, 0.9) and near(p1.distance, 0.75)
        and near(p2.distance, 0.6), string.format("%.3f %.3f %.3f", p0.distance, p1.distance, p2.distance))
  check("zoomUnarmed pose: FOV geometric 45 / 35.5 / 28", near(p0.fov, 45) and near(p1.fov, math.sqrt(45 * 28))
        and near(p2.fov, 28), string.format("%.2f %.2f %.2f", p0.fov, p1.fov, p2.fov))
  check("zoomUnarmed pose: over the shoulder (side 0.45, aim height 0.05), name for the log", near(p0.side, 0.45)
        and near(p0.height, cfg.aimHeight) and p0.name == "ZoomUnarmed")
  local pk = CM.zoomUnarmedPose(cfg, 7)
  check("zoomUnarmed pose: k past the last step clamps to the last third-person step", near(pk.distance, 0.6) and near(pk.fov, 28))
  local c1 = CM.config({ zoomUnarmedSteps = 1 })
  check("zoomUnarmed pose: 1 step = first pose only (no division by zero)", CM.zoomUnarmedSteps(c1) == 1
        and near(CM.zoomUnarmedPose(c1, 0).distance, 0.9) and near(CM.zoomUnarmedPose(c1, 3).fov, 45))
  local c4 = CM.config({ zoomUnarmedSteps = 4 })
  local q = {}
  for k = 0, 3 do q[k] = CM.zoomUnarmedPose(c4, k) end
  check("zoomUnarmed pose: 4 steps monotonic closer and narrower", q[0].distance > q[1].distance and q[1].distance > q[2].distance
        and q[2].distance > q[3].distance and q[0].fov > q[1].fov and q[2].fov > q[3].fov and near(q[3].fov, 28))
  check("zoomUnarmed steps: invalid / zero -> at least 1, missing -> 4", CM.zoomUnarmedSteps({ zoomUnarmedSteps = 0 }) == 1
        and CM.zoomUnarmedSteps({}) == 4)

  -- decision
  local function o(t)
    local r = { tpp = true, upperBody = 6, weaponInHand = false, held = 1, enterDelay = 0.1 }
    for k, v in pairs(t or {}) do r[k] = v end
    return r
  end
  local function W(t) local w, why = CM.zoomUnarmedWanted(o(t)); return w, why end
  local w, why = W()
  check("zoomUnarmed wanted: TPP + UpperBody 6 + empty WeaponRight", w == true and why == "unarmed zoom", why)
  w, why = W({ upperBody = 0 })
  check("zoomUnarmed wanted: UpperBody 0 -> no (reason UpperBody=0)", w == false and why == "UpperBody=0", why)
  w, why = W({ weaponInHand = true })
  check("zoomUnarmed wanted: item in WeaponRight -> no (the weapon aim handles it)", w == false and why == "item in WeaponRight", why)
  w, why = CM.zoomUnarmedWanted({ tpp = true, upperBody = 6, held = 1 }) -- weaponInHand absent = read failed
  check("zoomUnarmed wanted: WeaponRight unreadable -> no", w == false and why == "WeaponRight unreadable", why)
  w, why = W({ meleeState = 8 })
  check("zoomUnarmed wanted: PSM MeleeWeapon block -> no", w == false and string.find(why, "melee block", 1, true) == 1, why)
  w, why = W({ gadget = 2 })
  check("zoomUnarmed wanted: gadget charging -> no", w == false and string.find(why, "gadget", 1, true) == 1, why)
  w, why = W({ lhcw = 5 })
  check("zoomUnarmed wanted: arm cyberware -> no", w == false and string.find(why, "arm cyberware", 1, true) == 1, why)
  w, why = W({ tpp = false })
  check("zoomUnarmed wanted: not third person (user in FPP: the game's own zoom) -> no", w == false and why == "not third person", why)
  w, why = W({ tpp = false, on = true, handoff = true })
  check("zoomUnarmed wanted: first person handoff stays without third person", w == true and why == "first person handoff", why)
  w, why = W({ tpp = false, on = true, handoff = true, upperBody = 0 })
  check("zoomUnarmed wanted: handoff ends on RMB release (UpperBody 0)", w == false and why == "UpperBody=0", why)
  w, why = W({ enabled = false })
  check("zoomUnarmed wanted: camera.zoomUnarmed.enabled off", w == false and why == "camera.zoomUnarmed.enabled off", why)
  w, why = W({ vehicleHeld = true })
  check("zoomUnarmed wanted: vehicle hold wins", w == false and why == "vehicle hold", why)
  w, why = W({ poseOverride = true })
  check("zoomUnarmed wanted: pose override wins", w == false and why == "pose override", why)
  w, why = W({ held = 0.05 })
  check("zoomUnarmed wanted: enter delay (UpperBody 6 for < 0.1 s)", w == false and why == "enter delay", why)
  w, why = W({ held = 0.05, on = true })
  check("zoomUnarmed wanted: the enter delay does not apply once on", w == true, why)
  w, why = W({ loco = 6 })
  check("zoomUnarmed wanted: no entry during a dodge (PSM Locomotion 6)", w == false and string.find(why, "dodge", 1, true) == 1, why)
  w, why = W({ loco = 9 })
  check("zoomUnarmed wanted: no entry during a slide (PSM Locomotion 9)", w == false and string.find(why, "slide", 1, true) == 1, why)
  w, why = W({ loco = 9, on = true })
  check("zoomUnarmed wanted: a slide does not end a running zoom", w == true, why)
  check("zoomUnarmed wanted: nil input", CM.zoomUnarmedWanted(nil) == false)

  -- state machine: enter, wheel steps, handoff, exit
  local Z = CM.newZoomUnarmed()
  local dt = 1 / 60
  local ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 0)
  check("zoomUnarmed step: enter event at k=0, weight starts at 0", #ev == 1 and ev[1].kind == "enter" and Z.on and Z.k == 0
        and Z.w == 0 and Z.enters == 1)
  for _ = 1, 10 do CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 0) end
  check("zoomUnarmed step: weight 1 after the 0.15 s blend", near(Z.w, 1), Z.w)
  local mid = nil
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 1)
  check("zoomUnarmed step: wheel in -> step 1", #ev == 1 and ev[1].kind == "step" and ev[1].k == 1 and ev[1].dir == 1 and Z.k == 1)
  CM.zoomUnarmedStep(Z, cfg, 0.075, true, "unarmed zoom", 0)
  mid = CM.zoomUnarmedCompose({ name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }, Z)
  check("zoomUnarmed compose: half way through the 0.15 s step blend the pose is between steps 0 and 1",
        mid.distance < 0.9 - 1e-3 and mid.distance > 0.75 + 1e-3 and mid.fov < 45 and mid.fov > 35.5 and CM.zoomUnarmedActive(Z),
        string.format("%.3f %.2f", mid.distance, mid.fov))
  for _ = 1, 10 do CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 0) end
  local full = CM.zoomUnarmedCompose({ name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1.3 }, Z)
  check("zoomUnarmed compose: step 1 pose at full weight (0.75 m, FOV 35.5), profile sensitivity kept",
        near(full.distance, 0.75) and near(full.fov, math.sqrt(45 * 28)) and near(full.sens, 1.3) and near(full.side, 0.45))
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", -1)
  check("zoomUnarmed step: wheel out -> step 0", #ev == 1 and ev[1].kind == "step" and ev[1].k == 0 and ev[1].dir == -1)
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", -1)
  check("zoomUnarmed step: wheel out at step 0 -> nothing", #ev == 0 and Z.k == 0)
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 2)
  check("zoomUnarmed step: two notches in one frame -> steps 1 and 2", #ev == 2 and ev[2].k == 2 and Z.k == 2 and not Z.fpp)
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 1)
  check("zoomUnarmed step: wheel in past the last step -> fpp handoff (k = N)", #ev == 1 and ev[1].kind == "handoff"
        and ev[1].k == 3 and Z.fpp and Z.k == 3 and Z.handoffs == 1)
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "first person handoff", 1)
  local ev2 = CM.zoomUnarmedStep(Z, cfg, dt, true, "first person handoff", -1)
  check("zoomUnarmed step: during the handoff the wheel belongs to the game (no events, k stays N)", #ev == 0 and #ev2 == 0
        and Z.k == 3 and Z.fpp)
  for _ = 1, 10 do CM.zoomUnarmedStep(Z, cfg, dt, true, "first person handoff", 0) end -- step 1 -> 2 pose blend ends
  local poseDuring = CM.zoomUnarmedCompose({ name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }, Z)
  check("zoomUnarmed compose: the handoff keeps the last third-person pose (k clamped)", near(poseDuring.distance, 0.6)
        and near(poseDuring.fov, 28))
  ev = CM.zoomUnarmedStep(Z, cfg, dt, false, "UpperBody=0", 0)
  check("zoomUnarmed step: exit event with the step, the reason and the handoff flag", #ev == 1 and ev[1].kind == "exit"
        and ev[1].k == 3 and ev[1].wasFpp == true and ev[1].reason == "UpperBody=0" and ev[1].dur > 0.2 and not Z.on
        and not Z.fpp and Z.exits == 1)
  for _ = 1, 12 do CM.zoomUnarmedStep(Z, cfg, dt, false, "UpperBody=0", 1) end
  check("zoomUnarmed step: weight 0 after the 0.15 s exit blend; wheel ignored while off", Z.w == 0 and Z.k == 3)
  local base = { name = "P", distance = 2.1, height = 0.12, side = 0.5, fov = 56, sens = 1 }
  check("zoomUnarmed compose: weight 0 returns the live profile itself", CM.zoomUnarmedCompose(base, Z) == base)
  CM.zoomUnarmedStep(Z, cfg, dt, false, "UpperBody=0", 0)
  check("zoomUnarmed active: off after the exit tween ended", not CM.zoomUnarmedActive(Z))
  ev = CM.zoomUnarmedStep(Z, cfg, dt, true, "unarmed zoom", 1)
  check("zoomUnarmed step: re-entry starts at step 0 again; a wheel step on the enter frame counts",
        #ev == 2 and ev[1].kind == "enter" and ev[2].kind == "step" and Z.k == 1 and not Z.fpp)

  -- handoff off: the wheel stops at the last step; cancel restores the last step
  local cOff = CM.config({ zoomUnarmedSteps = 3, zoomUnarmedFppHandoff = false })
  local Z2 = CM.newZoomUnarmed()
  CM.zoomUnarmedStep(Z2, cOff, dt, true, "unarmed zoom", 0)
  ev = CM.zoomUnarmedStep(Z2, cOff, dt, true, "unarmed zoom", 5)
  check("zoomUnarmed: fppHandoff off -> the wheel stops at the last step (no handoff event)", #ev == 2 and Z2.k == 2
        and not Z2.fpp and Z2.handoffs == 0)
  local Z3 = CM.newZoomUnarmed()
  CM.zoomUnarmedStep(Z3, cfg, dt, true, "unarmed zoom", 0)
  CM.zoomUnarmedStep(Z3, cfg, dt, true, "unarmed zoom", 3)
  local canceled = CM.zoomUnarmedCancelHandoff(Z3, cfg)
  check("zoomUnarmed: cancel handoff (no route) -> back at the last third-person step", Z3.fpp == false and Z3.k == 2
        and canceled == true and CM.zoomUnarmedCancelHandoff(Z3, cfg) == false)
  -- 144 fps: the same weight curve end
  local Z4 = CM.newZoomUnarmed()
  for _ = 1, 23 do CM.zoomUnarmedStep(Z4, cfg, 1 / 144, true, "unarmed zoom", 0) end
  check("zoomUnarmed step: 144 fps reaches weight 1 within 0.15 s + one frame", near(Z4.w, 1))
  -- live settings reach the current step pose
  local c5 = CM.config({ zoomUnarmedSteps = 3, zoomUnarmedFppHandoff = true })
  local Z5 = CM.newZoomUnarmed()
  for _ = 1, 20 do CM.zoomUnarmedStep(Z5, c5, dt, true, "unarmed zoom", 0) end
  c5.zoomUnarmedFov = 50
  CM.zoomUnarmedStep(Z5, c5, dt, true, "unarmed zoom", 0)
  check("zoomUnarmed: a settings change (first FOV 50) reaches the shown pose", near(CM.zoomUnarmedCompose(base, Z5).fov, 50))
end

-- 2026-10-12 round 3 game look sync: evidence (CM.lookSyncSample) and the look as the free / own aim yaw (lookStep lookYaw)
do
  local cfg = CM.config()
  check("lookSync defaults: OFF (2026-10-12: not holding in game), set VehicleCombat, 2 confirm samples, 3 carried in a row, 2 resends",
    cfg.lookSync == false and cfg.lookSyncSet == "VehicleCombat" and cfg.lookSyncConfirm == 2 and cfg.lookSyncCarried == 3
    and cfg.lookSyncResends == 2)
  -- mouse only, V still: the look follows the mouse -> ok after 2 samples
  local ls = CM.newLookSync()
  local function smp(t) return CM.lookSyncSample(ls, cfg, t) end
  local e1 = smp({ want = true, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = -0.5 })
  local e2 = smp({ want = true, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = -0.5 })
  check("lookSync: look moved with the mouse while V stood x2 -> ok", e1 == nil and e2 == "ok" and ls.ok and ls.confirm == 2)
  -- wrong direction / no lock / not wanted / stick input: no evidence
  local ls2 = CM.newLookSync()
  local n = 0
  for _ = 1, 5 do
    if CM.lookSyncSample(ls2, cfg, { want = true, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = 0.5 }) then n = n + 1 end
    if CM.lookSyncSample(ls2, cfg, { want = true, locked = false, mouseX = 10, bodyDelta = 0, lookDelta = -0.5 }) then n = n + 1 end
    if CM.lookSyncSample(ls2, cfg, { want = false, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = -0.5 }) then n = n + 1 end
    if CM.lookSyncSample(ls2, cfg, { want = true, locked = true, mouseX = 10, pad = true, bodyDelta = 0, lookDelta = -0.5 }) then n = n + 1 end
    if CM.lookSyncSample(ls2, cfg, { want = true, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = nil }) then n = n + 1 end
  end
  check("lookSync: opposite look / unlocked / not wanted / stick / no look -> no evidence", n == 0 and not ls2.ok and ls2.confirm == 0)
  -- V turned without look input, the look stayed in the world (maintained) -> ok
  local ls3 = CM.newLookSync()
  local m1 = CM.lookSyncSample(ls3, cfg, { want = true, locked = true, mouseX = 0, bodyDelta = 12, lookDelta = 0.3 })
  local m2 = CM.lookSyncSample(ls3, cfg, { want = true, locked = true, mouseX = 0, bodyDelta = -8, lookDelta = 0 })
  check("lookSync: look kept its world heading while V turned x2 -> ok", m1 == nil and m2 == "ok" and ls3.maintained == 2)
  -- carried: 3 in a row -> resend (x2), then broken (logged by the caller once)
  local evs = {}
  for _ = 1, 9 do
    local ev = CM.lookSyncSample(ls3, cfg, { want = true, locked = true, mouseX = 0, bodyDelta = 10, lookDelta = 10.2 })
    if ev then evs[#evs + 1] = ev end
  end
  check("lookSync: look turned with V -> resend after 3, again after 3, then broken", #evs == 3 and evs[1] == "resend"
    and evs[2] == "resend" and evs[3] == "broken" and ls3.broken and not ls3.ok, table.concat(evs, ","))
  local after = CM.lookSyncSample(ls3, cfg, { want = true, locked = true, mouseX = 10, bodyDelta = 0, lookDelta = -0.5 })
  for _ = 1, 6 do after = after or CM.lookSyncSample(ls3, cfg, { want = true, locked = true, mouseX = 0, bodyDelta = 10, lookDelta = 10 }) end
  check("lookSync: broken stays broken (no new ok / resend events)", after == nil and ls3.broken and not ls3.ok)
  -- a maintained sample between carried ones resets the run (a single PSM set flip is not a failure)
  local ls4 = CM.newLookSync()
  local ev4 = {}
  for i = 1, 12 do
    local carried = (i % 3) ~= 0
    local ev = CM.lookSyncSample(ls4, cfg, { want = true, locked = true, mouseX = 0, bodyDelta = 10,
      lookDelta = carried and 10 or 0 })
    if ev and ev ~= "ok" then ev4[#ev4 + 1] = ev end
  end
  check("lookSync: carried runs broken by maintained samples -> no resend", #ev4 == 0, table.concat(ev4, ","))

  -- lookStep: free look shows the game's look; the switch offset shrinks (no jump); the mouse integration is not used
  local L = CM.newLook()
  local env = { bodyYaw = 0, fppPitch = -5, moving = false, decoupled = true }
  for _ = 1, 10 do CM.lookStep(L, cfg, 1 / 60, 1, 0, env) end -- own yaw 10 deg (fallback integration)
  env.lookYaw = 40
  local y1 = CM.lookStep(L, cfg, 1 / 60, 50, 0, env)
  check("lookSync lookStep: switching to the look does not jump (offset kept, shrinking)", y1 > 10 and y1 < 20, y1)
  local y, p
  for _ = 1, 60 do y, p = CM.lookStep(L, cfg, 1 / 60, 50, 0, env) end
  check("lookSync lookStep: free yaw = the game's look after 1 s (mouse dyaw 50 deg/frame ignored)", near(y, 40, 1e-6), y)
  check("lookSync lookStep: free pitch = the game's look pitch", near(p, -5, 1e-6), p)
  for _ = 1, 30 do env.bodyYaw = env.bodyYaw + 6; y = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("lookSync lookStep: V turning under the free camera does not move it", near(y, 40, 1e-6), y)
  env.lookYaw, env.fppPitch = 47, 12
  y, p = CM.lookStep(L, cfg, 1 / 60, 0, 0, env)
  check("lookSync lookStep: the look turns -> the camera turns the same frame (yaw and pitch)", near(y, 47, 1e-6)
    and near(p, 12, 1e-6), string.format("%.3f %.3f", y, p))
  -- aim (own yaw) with the look: = look; V's aim align turn does not move it
  env.decoupled, env.aim, env.aimW, env.refYaw = false, true, 1, 47
  for _ = 1, 5 do y = CM.lookStep(L, cfg, 1 / 60, 0, 0, env) end
  check("lookSync lookStep: aim from the synced free look keeps the look (no offset)", near(y, 47, 1e-6), y)
  for _ = 1, 20 do env.bodyYaw = env.bodyYaw - 9; y = CM.lookStep(L, cfg, 1 / 60, 3, 0, env) end
  check("lookSync lookStep: aiming, V turned onto the look and the mouse ignored -> camera = look", near(y, 47, 1e-6), y)
  -- look lost (sync off / broken): own integration continues from the shown yaw, no jump
  env.lookYaw = nil
  y = CM.lookStep(L, cfg, 1 / 60, 2, 0, env)
  check("lookSync lookStep: look gone -> own yaw from the shown one (+ mouse)", near(y, 49, 1e-6), y)
  -- lookReset forgets the sync (a fresh offset next time)
  CM.lookReset(L, 0)
  check("lookSync: lookReset clears the sync state", L.sync == false and L.syncP == false)
  -- solve passes lookYaw through
  local S = CM.newSolver(cfg)
  local base = { dt = 1 / 60, playerPos = V(0, 0, 0), playerQ = CM.qFromYawPitch(0, 0), bodyYaw = 0, eyeZ = 1.6,
                 fppPitch = 0, dyaw = 0, dpitch = 0, moving = false, decoupled = true,
                 profile = { name = "P", distance = 2, height = 0.1, side = 0.4, fov = 56, sens = 1 }, side = 1, w = 1, zoom = 0 }
  local r
  for _ = 1, 3 do r = CM.solve(S, base) end
  base.lookYaw = -30
  for _ = 1, 90 do r = CM.solve(S, base) end
  check("lookSync solve: inp.lookYaw reaches the shown yaw", near(r.yaw, -30, 1e-6), r.yaw)
end

-- 2026-10-12 (UTPP.dll 0.6.5) game look write: when the game's look is set to the shown camera (CM.lookWriteWant)
do
  local cfg = CM.defaultConfig and CM.defaultConfig() or CM.newConfig and CM.newConfig() or nil
  local base = { world = true, kind = "orbit", state = "tpp", held = false, locked = true }
  local function with(k, v) local s = {} for a, b in pairs(base) do s[a] = b end s[k] = v return s end
  local on = {}
  check("lookWrite: default on (cfg.lookWrite)", CM.DEFAULTS == nil or CM.DEFAULTS.lookWrite == true,
    CM.DEFAULTS and tostring(CM.DEFAULTS.lookWrite))
  check("lookWrite: world camera written, orbit, third person, heading locked -> write",
    CM.lookWriteWant(on, base) == true)
  check("lookWrite: camera.lookWrite off -> no write", CM.lookWriteWant({ lookWrite = false }, base) == false)
  check("lookWrite: utppCamera component (world camera not written) -> no write", CM.lookWriteWant(on, with("world", false)) == false)
  check("lookWrite: heading not locked (follow mode) -> no write", CM.lookWriteWant(on, with("locked", false)) == false)
  check("lookWrite: vehicle hold / release blend -> no write", CM.lookWriteWant(on, with("held", true)) == false
    and CM.lookWriteWant(on, with("kind", "release")) == false and CM.lookWriteWant(on, with("kind", "override")) == false)
  check("lookWrite: entering / leaving (not tpp) -> no write", CM.lookWriteWant(on, with("state", "entering")) == false
    and CM.lookWriteWant(on, with("state", "leaving")) == false and CM.lookWriteWant(on, with("state", "fpp")) == false)
  check("lookWrite: lock state nil -> no write", CM.lookWriteWant(on, with("locked", nil)) == false)
  local _ = cfg
end

-- ================================================================ vehicle batch (P1b): own pitch, hold yaw, framing band
-- CAM-3 T1 (= CAR-4 / TRAV-4): follow mode, the game's look pitch jumps to -60 when the look write turns off (vault / exit
-- animation) while env.ownPitch is on -> the camera pitch stays (mouse only); ownPitch off -> a glide (<= 3 deg per frame,
-- monotonic) instead of a one-frame snap
do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local cfg = CM.config({ idleOrbit = false })
    local L = CM.newLook()
    local dt = 1 / fps
    for _ = 1, 10 do CM.lookStep(L, cfg, dt, 0, 0, { bodyYaw = 0, fppPitch = -5, moving = true }) end
    local okHold, p = true, nil
    for i = 1, math.floor(0.5 * fps) do
      local dp = (i == 3) and 0.7 or 0
      _, p = CM.lookStep(L, cfg, dt, 0, dp, { bodyYaw = 0, fppPitch = -60, moving = true, ownPitch = true })
      okHold = okHold and math.abs(p - (-5 + (i >= 3 and 0.7 or 0))) < 1e-6
    end
    check("CAM-3 T1 at " .. fps .. " fps: own pitch kept while the game's look pitch jumps to -60 (mouse still applies)",
      okHold, p)
    local maxStep, mono, last, prev = 0, true, p, p
    for _ = 1, 3 * fps do
      _, p = CM.lookStep(L, cfg, dt, 0, 0, { bodyYaw = 0, fppPitch = -60, moving = true })
      maxStep = math.max(maxStep, math.abs(p - prev))
      if p > prev + 1e-9 then mono = false end
      prev = p
    end
    local limit = (cfg.ownPitchGlideRate or 120) * dt + 1e-6
    check("CAM-3 T1 at " .. fps .. " fps: own pitch off -> glide onto the game's pitch (<= " .. string.format("%.1f", limit)
      .. " deg per frame, monotonic, reached)", maxStep <= limit and (fps ~= 60 or maxStep <= 3) and mono and math.abs(p + 60) < 1e-6
      and L.recovering == false, string.format("max %.2f last %.2f from %.2f", maxStep, p, last))
  end
end
-- CAM-3 T2 (on foot unchanged): ownPitch nil, fppPitch moving with the mouse -> L.pitch == fppPitch on every frame
do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local cfg = CM.config({ idleOrbit = false })
    local L = CM.newLook()
    local same = true
    local worst = 0
    for i = 1, 3 * fps do
      local fp = 20 * math.sin(i / fps * 3) - 10 + ((i % 37 == 0) and 15 or 0)
      local _, p = CM.lookStep(L, cfg, 1 / fps, 0.3, 0.4, { bodyYaw = i * 0.5, fppPitch = fp, moving = i % 5 ~= 0 })
      worst = math.max(worst, math.abs(p - fp))
      same = same and p == fp
    end
    check("CAM-3 T2 at " .. fps .. " fps: on foot (ownPitch nil) the follow pitch IS the game's look pitch every frame", same, worst)
    -- the same with ownPitch = false explicitly
    local L2 = CM.newLook()
    local same2 = true
    for i = 1, fps do
      local fp = -30 + i % 50
      local _, p = CM.lookStep(L2, cfg, 1 / fps, 0, 0, { bodyYaw = 0, fppPitch = fp, moving = true, ownPitch = false })
      same2 = same2 and p == fp
    end
    check("CAM-3 T2 at " .. fps .. " fps: ownPitch false = unchanged", same2)
  end
  -- aim / zoom branches unchanged by ownPitch (they come first)
  local cfg = CM.config({ idleOrbit = false })
  local LA, LB = CM.newLook(), CM.newLook()
  local same = true
  for i = 1, 120 do
    local env = { bodyYaw = 0, fppPitch = -10 + i * 0.1, moving = true, aim = i > 10, aimW = 1 }
    local _, pa = CM.lookStep(LA, cfg, 1 / 60, 0.2, 0.1, env)
    local envB = {}
    for k, v in pairs(env) do envB[k] = v end
    envB.ownPitch = i > 10 -- while aiming only (the follow frames before must match for the same aim entry offset)
    local _, pb = CM.lookStep(LB, cfg, 1 / 60, 0.2, 0.1, envB)
    if i > 10 then same = same and pa == pb end
  end
  check("CAM-3: aiming ignores ownPitch (aim pitch path unchanged)", same)
end
-- CAM-4 T3: holdYaw (vehicle hold, orbit shown) -> a body yaw jump of 90 deg does not move the shown yaw; the mouse does
do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local cfg = CM.config({ idleOrbit = false })
    local L = CM.newLook()
    local y0
    for _ = 1, 20 do y0 = CM.lookStep(L, cfg, 1 / fps, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true }) end
    local y1 = CM.lookStep(L, cfg, 1 / fps, 0, 0, { bodyYaw = 100, fppPitch = -5, moving = true, holdYaw = true })
    local still = true
    for i = 1, fps do
      local y = CM.lookStep(L, cfg, 1 / fps, 0, 0, { bodyYaw = 100 + (i % 7) * 10, fppPitch = -5, moving = true, holdYaw = true })
      still = still and math.abs(CM.wrapDeg(y - y0)) < 1e-6
    end
    local y2 = CM.lookStep(L, cfg, 1 / fps, 4, 0, { bodyYaw = 100, fppPitch = -5, moving = true, holdYaw = true })
    check("CAM-4 T3 at " .. fps .. " fps: holdYaw keeps the shown world yaw through a 90 deg body jump, the mouse still turns it",
      math.abs(CM.wrapDeg(y1 - y0)) < 1e-6 and still and math.abs(CM.wrapDeg(y2 - y0) - 4) < 1e-6,
      string.format("%.2f %.2f %.2f", y0, y1, y2))
  end
  -- negative control (the old behaviour): snap = rigid follow -> the shown yaw jumps with the body
  local cfg = CM.config({ idleOrbit = false })
  local L = CM.newLook()
  local y0
  for _ = 1, 20 do y0 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 10, fppPitch = -5, moving = true }) end
  local y1 = CM.lookStep(L, cfg, 1 / 60, 0, 0, { bodyYaw = 100, fppPitch = -5, moving = true, snap = true })
  check("CAM-4 negative control: snap (rigid follow) jumps 90 deg with the body", math.abs(CM.wrapDeg(y1 - y0) - 90) < 1e-6, y1 - y0)
end
-- CAR-3 (1): framing band - V's pivot inside [-0.35, +0.25] of the half vertical FOV
do
  local cfg = CM.config()
  local cam = V(0, -5, 2.4)
  local piv = V(0, 0, 1.4)
  local need = math.deg(math.atan2(-1.0, 5))
  local h = 28
  check("CAR-3: band keeps a pitch inside unchanged", near(CM.vehFramePitch(need, cam, piv, 56, cfg), need))
  check("CAR-3: looking too far down -> clamped so V's pivot is <= 25 % of the half FOV above the centre",
    near(CM.vehFramePitch(-60, cam, piv, 56, cfg), need - 0.25 * h))
  check("CAR-3: looking too far up -> clamped so V's pivot is <= 35 % of the half FOV below the centre",
    near(CM.vehFramePitch(30, cam, piv, 56, cfg), need + 0.35 * h))
  local up = V(0, 0, 4.4) -- V launched 3 m up
  local needUp = math.deg(math.atan2(2.0, 5))
  check("CAR-3: V launched up -> the pitch follows up", near(CM.vehFramePitch(-14, cam, up, 56, cfg), needUp - 0.25 * h))
  check("CAR-3: pivot straight above the camera / missing input -> unchanged", CM.vehFramePitch(-14, cam, V(0, -4.9, 6), 56, cfg) == -14
    and CM.vehFramePitch(-14, nil, piv, 56, cfg) == -14)
end

-- ================================================================ 2026-10-28 mount pivot continuity (CM.mountShiftStep)
-- recorded bike mount (log 15-45-08 15:48:40.72 -> 40.85): V snapped about 1.2 m and 75 deg onto the seat slot in one
-- frame, +0.68 m z; the camera yaw stayed (-8.4). Solver frames at 60 fps, V still before and after the snap.
do
  local cfg = CM.config()
  local DT = 1 / 60
  local function frameInp(pos, arm, keep, body)
    return { dt = DT, playerPos = pos, playerQ = CM.qFromYawPitch(body or 66.5, 0), bodyYaw = body or 66.5, eyeZ = pos.z + 1.62,
             fppPitch = 0, dyaw = 0, dpitch = 0, profile = MEDIUM, side = 1, w = 1, mountArm = arm, mountKeep = keep }
  end
  local before, seat = V(10, 20, 0), V(10 - 1.15, 20 + 0.35, 0.68)
  -- run: n0 frames before (at before), snap at frame n0 + 1, then V on the seat; returns the per-frame results
  local function run(armFn, keepFn, n, cfgOver, posFn)
    local S = CM.newSolver(cfgOver or cfg)
    local out = {}
    for i = 1, n do
      local p = posFn and posFn(i) or (i <= 30 and before or seat)
      out[i] = CM.solve(S, frameInp(p, armFn(i), keepFn(i))) -- body yaw constant: only the pivot is under test
    end
    return out, S
  end
  local always = function() return true end
  local never = function() return false end
  local R = run(always, always, 30 + 40)
  local snap = R[31]
  check("mount blend: start event on the snap frame (1.20 m)", snap.mountShift ~= nil and snap.mountShift.kind == "start"
    and near(snap.mountShift.d, math.sqrt(1.15 ^ 2 + 0.35 ^ 2), 1e-9), snap.mountShift and snap.mountShift.kind)
  check("mount blend: continuity at t = 0 (pivot xy of the snap frame = the last shown pivot xy, exact)",
    near(snap.pivot.x, R[30].pivot.x, 1e-9) and near(snap.pivot.y, R[30].pivot.y, 1e-9), vs(snap.pivot) .. " vs " .. vs(R[30].pivot))
  -- the shown camera position moves <= 0.12 m per frame in xy (it moved 1.2 m in one frame before the fix)
  local maxStep = 0
  for i = 31, #R do
    local a, b = R[i - 1].world, R[i].world
    maxStep = math.max(maxStep, math.sqrt((b.x - a.x) ^ 2 + (b.y - a.y) ^ 2))
  end
  check("mount blend: camera xy moves <= 0.08 m per frame after the snap (0.38 s smoothstep)", maxStep <= 0.08,
    string.format("%.3f m", maxStep))
  -- end: at t >= mountBlendTime the pivot is the live one bit for bit (offset dropped)
  local nEnd = 31 + math.ceil(cfg.mountBlendTime / DT)
  local endF
  for i = 31, #R do if R[i].mountShift and R[i].mountShift.kind == "end" then endF = i; break end end
  check("mount blend: ends after mountBlendTime with one end event", endF ~= nil and endF <= nEnd + 1 and endF >= nEnd - 1,
    tostring(endF) .. " vs " .. nEnd)
  local Rn = run(always, always, 30 + 40, CM.config({ mountBlend = false }))
  check("mount blend: pivot exactly on V from the end frame on (same as without the blend)",
    endF ~= nil and R[endF].pivot.x == Rn[endF].pivot.x and R[endF].pivot.y == Rn[endF].pivot.y
    and R[#R].world.x == Rn[#Rn].world.x and R[#R].world.y == Rn[#Rn].world.y and R[#R].world.z == Rn[#Rn].world.z)
  check("mount blend: mountBlend = false -> the old cut (pivot on V on the snap frame), no event",
    Rn[31].mountShift == nil and near(Rn[31].pivot.x, seat.x, 1e-9) and near(Rn[31].pivot.y, seat.y, 1e-9))
  check("mount blend: z untouched by the blend (same pivot z with and without)", near(R[31].pivot.z, Rn[31].pivot.z, 1e-9)
    and near(R[40].pivot.z, Rn[40].pivot.z, 1e-9))
  -- monotonic: the pivot never overshoots the line from the old to the new position
  local mono = true
  for i = 32, endF or 31 do
    local da = (R[i].pivot.x - seat.x) ^ 2 + (R[i].pivot.y - seat.y) ^ 2
    local db = (R[i - 1].pivot.x - seat.x) ^ 2 + (R[i - 1].pivot.y - seat.y) ^ 2
    if da > db + 1e-12 then mono = false end
  end
  check("mount blend: monotonic approach to V (no overshoot)", mono)
  -- no jump -> bit-identical results with the inputs on and off (normal walking speed 1.5 m/s)
  local walk = function(i) return V(10 + 0.025 * i, 20, 0) end
  local Ra = run(always, always, 60, nil, walk)
  local Rb = run(never, never, 60, nil, walk)
  local same = true
  for i = 1, 60 do
    if Ra[i].world.x ~= Rb[i].world.x or Ra[i].world.y ~= Rb[i].world.y or Ra[i].world.z ~= Rb[i].world.z
      or Ra[i].mountShift ~= nil then same = false end
  end
  check("mount blend: no jump -> results bit-identical with the gate on / off, no event", same)
  -- not armed (outside the mount window / on foot): the cut stays (no blend), identical to mountBlend = false
  local Rx = run(never, always, 40)
  check("mount blend: not armed -> no blend (pivot on V on the snap frame)", Rx[31].mountShift == nil
    and Rx[31].pivot.x == Rn[31].pivot.x and Rx[31].pivot.y == Rn[31].pivot.y)
  -- abort: keep false (unmount / exit / perspective change) during the blend -> pivot on V at once, one abort event
  local Rab = run(always, function(i) return i < 36 end, 40)
  check("mount blend: abort on keep = false (exit / perspective change) -> event, pivot on V", Rab[36].mountShift ~= nil
    and Rab[36].mountShift.kind == "abort" and Rab[36].pivot.x == Rn[36].pivot.x and Rab[36].pivot.y == Rn[36].pivot.y
    and Rab[37].mountShift == nil, Rab[36].mountShift and Rab[36].mountShift.kind)
  -- abort: switched off during the blend
  do
    local cfgT = CM.config()
    local S = CM.newSolver(cfgT)
    local last
    for i = 1, 36 do
      if i == 35 then cfgT.mountBlend = false end
      last = CM.solve(S, frameInp(i <= 30 and before or seat, true, true))
      if i == 35 then break end
    end
    check("mount blend: switched off during the blend -> abort, pivot on V", last.mountShift ~= nil and last.mountShift.kind == "abort"
      and near(last.pivot.x, seat.x, 1e-9) and S.mshift == nil)
  end
  -- teleport (> teleportDist) during the blend -> abort; a teleport never starts a blend
  local far = V(10 + 5, 20, 0)
  local Rt = run(always, always, 40, nil, function(i) if i <= 30 then return before elseif i < 35 then return seat end return far end)
  check("mount blend: teleport during the blend -> abort, pivot on V; no blend started by it", Rt[35].mountShift ~= nil
    and Rt[35].mountShift.why == "teleport" and near(Rt[35].pivot.x, far.x, 1e-9) and Rt[36].mountShift == nil)
  local Rt2 = run(always, always, 34, nil, function(i) return i <= 30 and before or far end)
  check("mount blend: a 5 m teleport is not blended", Rt2[31].mountShift == nil and near(Rt2[31].pivot.x, far.x, 1e-9))
  -- restart: a second snap during the blend starts from the shown pivot (continuous) and blends over a full mountBlendTime
  local seat2 = V(seat.x + 0.6, seat.y - 0.2, seat.z)
  local Rr = run(always, always, 70, nil, function(i) if i <= 30 then return before elseif i <= 36 then return seat end return seat2 end)
  check("mount blend: second jump during the blend -> restart from the shown pivot (exact continuity)", Rr[37].mountShift ~= nil
    and Rr[37].mountShift.kind == "restart" and near(Rr[37].pivot.x, Rr[36].pivot.x, 1e-9) and near(Rr[37].pivot.y, Rr[36].pivot.y, 1e-9))
  -- frame-rate independence: the blend duration in seconds at 30 / 144 fps
  for _, fps in ipairs({ 30, 144 }) do
    local S = CM.newSolver(cfg)
    local t, endT, maxS, prev = 0, nil, 0, nil
    for i = 1, fps do
      local p = i <= 5 and before or seat
      local inp = frameInp(p, true, true)
      inp.dt = 1 / fps
      local r = CM.solve(S, inp)
      if i > 5 then t = t + 1 / fps end
      if prev and i > 5 then maxS = math.max(maxS, math.sqrt((r.pivot.x - prev.x) ^ 2 + (r.pivot.y - prev.y) ^ 2)) end
      prev = r.pivot
      if r.mountShift and r.mountShift.kind == "end" and endT == nil then endT = t end
    end
    check(string.format("mount blend: %d fps ends %.2f s after the snap (+- 1 frame), pivot step <= 1.5x the 60 fps peak rate", fps,
      cfg.mountBlendTime), endT ~= nil and math.abs(endT - cfg.mountBlendTime - 1 / fps) <= 1 / fps + 1e-9
      and maxS <= 1.5 * 1.2 * 1.5 / cfg.mountBlendTime / fps + 1e-9, string.format("end %s, step %.3f", tostring(endT), maxS))
  end
end

-- ================================================================ 2026-11-03 fast fall: the capsule pivot follows V without a lag snap
-- Game log 16-28-47 (car jump-out, JO1 / JO2): the pivot base followed V's Z with the 0.12 s spring, whose lag grows to v * T (1 m at
-- about 8.3 m/s). Over pivotSnap (1.0 m) the base was snapped onto V (vb = 0): a 1 m camera step in one frame on the way up and 1 - 3
-- times in the last 0.25 s before the touchdown, then 0.5 - 0.8 m of extra sink after it. Ballistic fixture of the jump-out: launch vz
-- 10.4 m/s, g -16, the ground 1.93 m under the launch point, lag 0.41 m at the launch edge (log line 594), V rising for 0.055 s before
-- the edge. Every frame of the ascent, the descent and 0.5 s after the touchdown: the pivot z step differs from V's step by <= 0.15 m,
-- nothing snaps, the landing settle is monotonic and under 0.15 m within 0.3 s.
-- 2026-11-03 (jump-out landing): this block is the vehicle exit window OFF (pivotStep without jo: every other jump / drop keeps the rigid follow). The
-- jump-out itself (hold on, armed before the launch) uses CM.jumpOutStep and has its own block at the end of this file.
do
  local cfg = CM.config()
  check("fast fall cfg: new pivot constants exist (pivotFast, pivotFastVz, pivotFastArm, pivotFastSmooth, pivotFastRamp, pivotLagMax)",
        cfg.pivotFast == true and cfg.pivotFastVz == 3.0 and cfg.pivotFastArm and cfg.pivotFastSmooth and cfg.pivotFastRamp
        and cfg.pivotLagMax == cfg.pivotSnap)
  local VZ0, G, DROP, LAG0, STAND = 10.4, -16, 1.93, 0.41, 1.55
  local function fall(fps, over)
    local c = over and CM.config(over) or cfg
    local dt = 1 / fps
    local pv = CM.newPivot()
    local z0 = 100
    for _ = 1, 10 do CM.pivotStep(pv, c, dt, V(0, 0, z0 - DROP + DROP), nil, false) end
    local ground = z0 - DROP
    local function zAt(t) return math.max(ground, z0 + VZ0 * t + 0.5 * G * t * t) end
    -- V rises for 0.055 s before the launch edge (the capsule has been moving that long when the log sample shows lag 0.41)
    local pre = math.ceil(0.055 * fps)
    local t = -pre * dt
    local prevZ, prevP
    for i = 1, pre do
      t = t + dt
      prevZ = zAt(t)
      CM.pivotStep(pv, c, dt, V(0, 0, prevZ), nil, false)
    end
    pv.base, pv.vb = prevZ - LAG0, 0 -- the lag at the launch edge
    local zp = pv.base + STAND
    local maxDiff, maxAt, snaps, landed, landedZ, landT = 0, nil, 0, nil, nil, nil
    local minPz, monoAfter, travel, settleStart = 1e9, true, 0, nil
    prevP = zp
    local last
    for i = 1, math.floor(3.2 * fps) do
      t = t + dt
      local pz = zAt(t)
      local p = CM.pivotStep(pv, c, dt, V(0, 0, pz), nil, false)
      local dPlayer, dPivot = pz - prevZ, p.z - prevP
      local diff = math.abs(dPivot - dPlayer)
      if diff > maxDiff then maxDiff, maxAt = diff, t end
      if diff > 0.5 or pv.snapEv ~= nil then snaps = snaps + 1 end
      pv.snapEv = nil
      if pz <= ground + 1e-9 and landed == nil then landed, landedZ, landT = i, p.z, t end
      if landed then
        if p.z > last + 1e-9 then monoAfter = false end
        minPz = math.min(minPz, p.z)
        if t - landT <= 0.3 + 1e-9 then travel = math.max(travel, math.abs(p.z - landedZ)) end
      end
      last = p.z
      prevZ, prevP = pz, p.z
    end
    return { maxDiff = maxDiff, maxAt = maxAt, snaps = snaps, mono = monoAfter, minPz = minPz, travel = travel,
             finalZ = last, ground = ground, landT = landT, pv = pv, c = c }
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local r = fall(fps)
    check(string.format("fast fall %d fps: pivot z step within 0.15 m of V's step on the ascent and the descent (max %.3f m at t=%.3f)",
          fps, r.maxDiff, r.maxAt or -1), r.maxDiff <= 0.15, string.format("%.3f m at t=%.3f", r.maxDiff, r.maxAt or -1))
    check(string.format("fast fall %d fps: no snap (a 1 m one-frame step) and no snap event", fps), r.snaps == 0, tostring(r.snaps))
    check(string.format("fast fall %d fps: post-landing travel under 0.15 m within 0.3 s (%.3f m)", fps, r.travel), r.travel < 0.15,
          string.format("%.3f m", r.travel))
    check(string.format("fast fall %d fps: the landing settle is monotonic, no overshoot below the stance height (min %.4f, stance %.4f)", fps,
          r.minPz, r.ground + STAND), r.mono and r.minPz >= r.ground + STAND - 1e-6, string.format("min %.4f", r.minPz))
    check(string.format("fast fall %d fps: back at the stance height 0.5 s after the touchdown", fps), math.abs(r.finalZ - (r.ground + STAND)) < 1e-3,
          string.format("%.4f", r.finalZ - (r.ground + STAND)))
  end
  -- a real teleport still snaps: 50 m up in one frame, also with the fast mode armed (V in flight at 10 m/s)
  for _, fps in ipairs({ 30, 144 }) do
    local pv = CM.newPivot()
    local dt = 1 / fps
    local z = 0
    for _ = 1, 5 do CM.pivotStep(pv, cfg, dt, V(0, 0, z), nil, false) end
    for _ = 1, 3 * math.floor(fps / 10) do z = z + 10 * dt; CM.pivotStep(pv, cfg, dt, V(0, 0, z), nil, false) end
    local ev = pv.fastOn
    local p = CM.pivotStep(pv, cfg, dt, V(300, 0, z + 50), nil, false)
    check(string.format("teleport still snaps (%d fps, 50 m, fast mode %s): pivot on the new position at once, snap event", fps, ev and "armed" or "not armed"),
          near(p.z, z + 50 + STAND, 1e-9) and near(p.x, 300) and pv.snapEv ~= nil and pv.snapEv.kind == "teleport" and pv.vb == 0,
          string.format("%.4f vs %.4f", p.z, z + 50 + STAND))
  end
  -- the lag bound is continuous: a V that moves 0.8 m in one frame on a 0.5 m lag (old: snap, one 1.3 m step) is clamped to the lag bound
  do
    local pv = CM.newPivot()
    CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 0), nil, false)
    pv.base = -0.5
    local p0z = pv.base + STAND
    local p = CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 0.8), nil, false)
    check("lag clamp: a lag over pivotLagMax is clamped (continuous, vb carried), not snapped to V",
          pv.snapEv ~= nil and pv.snapEv.kind == "clamp" and math.abs(p.z - p0z) < 1.3 and p.z > 0.8 + STAND - 1.0 - 1e-6 and pv.vb ~= 0,
          string.format("z %.3f vb %.3f", p.z, pv.vb))
  end
  -- a legitimate fast fall is not a teleport: 30 m / 30 fps terminal fall (1 m per frame) - no snap event, the step follows V
  do
    local fps, dt = 30, 1 / 30
    local pv = CM.newPivot()
    for _ = 1, 10 do CM.pivotStep(pv, cfg, dt, V(0, 0, 0), nil, false) end
    local z, vz, prevZ, prevP, maxD, ev = 0, 0, 0, pv.base + STAND, 0, 0
    for _ = 1, 6 * fps do
      vz = math.max(-30, vz - 19.62 * dt)
      z = math.max(-30, z + vz * dt)
      if z <= -30 then vz = 0 end
      local p = CM.pivotStep(pv, cfg, dt, V(0, 0, z), nil, false)
      maxD = math.max(maxD, math.abs((p.z - prevP) - (z - prevZ)))
      if pv.snapEv then ev = ev + 1; pv.snapEv = nil end
      prevZ, prevP = z, p.z
    end
    check("fast fall: a 30 m/s fall at 30 fps (1 m per frame) is no teleport: no snap / clamp event, pivot step follows V (<= 0.15 m)",
          ev == 0 and maxD <= 0.15, string.format("events %d, max %.3f m", ev, maxD))
  end
  -- a one-frame seat-to-ground style drop of 1.2 m: at 144 fps (170 m/s) a snap like before, at 30 fps (36 m/s) the continuous clamp
  for _, fps in ipairs({ 144, 30 }) do
    local pv = CM.newPivot()
    for _ = 1, 5 do CM.pivotStep(pv, cfg, 1 / fps, V(0, 0, 0), nil, false) end
    local p = CM.pivotStep(pv, cfg, 1 / fps, V(0, 0, -1.2), nil, false)
    local kind = pv.snapEv and pv.snapEv.kind or "none"
    check(string.format("a one-frame 1.2 m drop at %d fps: %s", fps, fps == 144 and "snap to V (as before)" or "no snap, continuous clamp"),
          kind == (fps == 144 and "teleport" or "clamp") and (fps ~= 144 or near(p.z, -1.2 + STAND, 1e-9)), kind)
  end
  -- unchanged: stairs, a single-frame step, walking slopes and the crouch do not arm the fast mode (bit for bit equal with pivotFast = false)
  local function series(over)
    local c = over and CM.config(over) or cfg
    local out = {}
    for _, fps in ipairs({ 30, 60, 144 }) do
      local dt = 1 / fps
      local pv = CM.newPivot()
      local z = 0
      for i = 1, math.floor(0.2 * fps) do CM.pivotStep(pv, c, dt, V(0, 0, z), nil, false) end
      for i = 1, math.floor(1.0 * fps) do -- a 0.3 m stair step (one frame), then 3 m/s slope walk 0.3 s, a 0.5 m ledge drop, crouch edge
        if i == 3 then z = z + 0.3 end
        if i > 10 and i <= 10 + math.floor(0.3 * fps) then z = z + 2.5 * dt end
        if i == math.floor(0.6 * fps) then z = z - 0.5 end
        out[#out + 1] = CM.pivotStep(pv, c, dt, V(0, 0, z), nil, i > math.floor(0.7 * fps)).z
      end
    end
    return out
  end
  local sOn, sOff = series(), series({ pivotFast = false })
  local same = #sOn == #sOff
  for i = 1, #sOn do if sOn[i] ~= sOff[i] then same = false; break end end
  check("fast fall: stairs / ledge drop / slope walk / crouch edge never arm the fast mode (bit for bit equal to pivotFast = false)", same)
  -- 2026-11-03 review r2 #13: pivotFast = false is the OLD rule - a 1.5 m one-frame step at 30 fps (under 50 m/s * dt = 1.67 m, so no teleport in the
  -- new logic; the old rule snaps at any lag over pivotSnap) snaps the base to V with vb = 0; the default (fast) mode clamps it instead
  do
    local function stepOnce(over)
      local c = over and CM.config(over) or cfg
      local pv = CM.newPivot()
      for _ = 1, 5 do CM.pivotStep(pv, c, 1 / 30, V(0, 0, 0), nil, false) end
      local p = CM.pivotStep(pv, c, 1 / 30, V(0, 0, 1.5), nil, false)
      return p, pv
    end
    local pOld, pvOld = stepOnce({ pivotFast = false })
    check("pivotFast = false: a 1.5 m one-frame step at 30 fps snaps the base to V exactly as before (base = V, vb = 0)",
          near(pOld.z, 1.5 + STAND, 1e-9) and pvOld.base == 1.5 and pvOld.vb == 0, string.format("%.4f base %s vb %s", pOld.z, tostring(pvOld.base), tostring(pvOld.vb)))
    local pNew, pvNew = stepOnce()
    check("pivotFast default: the same step is clamped to the lag bound (no snap), the base keeps V's velocity",
          pvNew.snapEv ~= nil and pvNew.snapEv.kind == "clamp" and pvNew.base > 0.5 and pvNew.base < 1.5 - 1e-6 and pvNew.vb ~= 0, string.format("%.4f", pNew.z))
  end
end

-- ================================================================ 2026-11-03 jump-out landing: the governed vertical follow (CM.jumpOutStep)
-- Game log 23-23-27 (car jump-out, 9.76 m/s launch, 1.85 m below the seat, touchdown at 11.4 m/s): the rigid fast-fall follow above stopped the camera
-- dead in the touchdown frame (250 - 1400 m/s^2 of vertical camera acceleration, 8 - 10 m/s of speed change in one frame). With the vehicle exit hold on
-- (pivotStep jo.on, armed before the launch edge, kept through the apex) the pivot base follows V as an acceleration limited follower with a ground
-- estimate from a down ray; the mode ends when the hold is off and the base sat on V. The 'fast fall' block above is the window OFF (non jump-out)
-- behaviour and stays rigid. Limits are dt-normalised (speed change of the shown pivot z over the mean of two frame times), also with frame time
-- jitter (+-20 %) and 37 ms hitches on the launch / touchdown frames (a hitch next to short frames inflates the estimate: 25 % more allowed there).
do
  local cfg = CM.config()
  local GRAV, VZ0, DROP, STAND = -16, 9.76, 1.85, 1.55
  local PRE = 0.25 -- s of standstill (armed) before the launch edge
  local function lcg(seed)
    local x = seed
    return function() x = (x * 16807) % 2147483647; return x / 2147483647 end
  end
  -- o: ray = "ground" | "car" (a surface 1 m over the ground until V passed below it) | nil; jit, hitchL, hitchT, seed; armed = false (window off);
  -- over = cfg overrides; vz0 / drop / offAfter (s of ground before the hold goes off)
  -- 2026-11-04 (review): hitchL / hitchT = true (37 ms) or a frame time in s on the launch / touchdown frame; zeroL / zeroT = an extra frame with dt = 0
  -- (V unchanged) before the launch / touchdown frame; grav, dur (s simulated), offAtT (s: the hold goes off at that time), climb (m/s: V keeps
  -- rising after the touchdown, a slope); every snap / clamp event of every frame is counted (m.events)
  local function hv(x) if x == true then return 0.037 end return x end
  local function jumpOut(fps, o)
    o = o or {}
    local c = o.over and CM.config(o.over) or cfg
    local rnd = lcg(o.seed or 1)
    local vz0, drop = o.vz0 or VZ0, o.drop or DROP
    local z0, pv = 100, CM.newPivot()
    local ground = z0 - drop
    local landZ = ground + (o.curb or 0) -- 2026-11-04 r11: o.curb = V lands on a surface this high over the ground the ray sees (the centre ray missed it)
    local rejumped, rejumpT = false, nil -- r11: o.rejumpAt (s after the touchdown) / o.rejumpV: V jumps again
    local z, v, t = z0, 0, 0
    local on, launched, landed, touchT = true, false, false, nil
    local carTop = ground + 1.0
    local function ray(from, to)
      local g = ground
      if o.ray == "car" and from.z > carTop + 1e-6 then g = carTop end
      if from.z < g or to.z > g then return nil end
      return from.z - g
    end
    local rayFn = (o.ray == "ground" or o.ray == "car") and ray or nil
    for _ = 1, 10 do CM.pivotStep(pv, c, 1 / fps, V(0, 0, z0), nil, false, nil, nil, nil, nil, nil, o.armed ~= false and { on = true, ray = rayFn } or nil) end
    local rec, offT, activeAtOff, endedT = {}, nil, nil, nil
    local rays = 0
    local events, evKinds, zeroOk, zeroFrames, lastZ, lastPz = 0, {}, true, 0, z0, nil
    local grav = o.grav or GRAV
    local rayCount = function(a, b) rays = rays + 1; return ray(a, b) end
    local rf = rayFn and rayCount or nil
    while t < (o.dur or 3.0) do
      local dt = (1 / fps) * (1 + (o.jit or 0) * (2 * rnd() - 1))
      local zeroNow = false
      if not launched and t + dt >= PRE then
        launched, v = true, vz0
        if o.hitchL then dt = hv(o.hitchL) end
        zeroNow = o.zeroL == true
      end
      if landed and o.rejumpAt and not rejumped and touchT ~= nil and t + dt >= touchT + o.rejumpAt then
        rejumped, landed, v, rejumpT = true, false, o.rejumpV or 5, t + dt
      end
      if launched and not landed then
        local vn = v + grav * dt
        local zn = z + vn * dt
        if zn <= landZ then
          landed = true
          if o.hitchT then dt = hv(o.hitchT) end
          zn, vn = landZ, 0
          touchT = touchT or (t + dt)
          zeroNow = zeroNow or o.zeroT == true
        end
        z, v = zn, vn
      elseif landed and o.climb then
        z = z + o.climb * dt
        ground = z
      end
      -- a repeated frame (dt = 0, V where the previous frame left it): the pivot must not change
      if zeroNow and lastPz ~= nil then
        local pz0 = CM.pivotStep(pv, c, 0, V(0, 0, lastZ), nil, false, nil, nil, nil, nil, nil, o.armed ~= false and { on = on, ray = rf } or nil)
        zeroFrames = zeroFrames + 1
        if pz0.z ~= lastPz then zeroOk = false end
      end
      t = t + dt
      if on and ((landed and t - touchT >= (o.offAfter or 0.2)) or (o.offAtT ~= nil and t >= o.offAtT)) then on, offT = false, t end
      pv.snapEv = nil
      local p = CM.pivotStep(pv, c, dt, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, o.armed ~= false and { on = on, ray = rf } or nil)
      if pv.snapEv ~= nil then events = events + 1; evKinds[#evKinds + 1] = pv.snapEv.kind end
      lastZ, lastPz = z, p.z
      if offT ~= nil and activeAtOff == nil then activeAtOff = pv.joOn == true end
      if offT ~= nil and endedT == nil and not pv.joOn then endedT = t end
      rec[#rec + 1] = { t = t, dt = dt, pz = p.z, z = z, pb = pv.base, on = pv.joOn == true, recov = pv.joRec == true, brk = pv.joBr == true }
    end
    local final = ground + (o.curb or 0) + STAND
    local apexT = PRE + vz0 / -GRAV
    local maxLaunch, maxLand, prevV, prevDt = 0, 0, nil, nil
    local minAfter, lagTD, settle = 1e9, nil, 0
    local maxBrake, maxStep, prevAcc, recSeen, brkSeen = 0, 0, nil, false, false -- 2026-11-04: braking (upward) peak, per-frame acceleration step
    for i = 2, #rec do
      local r = rec[i]
      local vel = (r.pz - rec[i - 1].pz) / r.dt
      if prevV ~= nil then
        local acc = math.abs(vel - prevV) / ((r.dt + prevDt) / 2) -- frame mean speeds, centres of the two frames (mean of both frame times)
        if r.t < apexT then maxLaunch = math.max(maxLaunch, acc) else maxLand = math.max(maxLand, acc) end
        if r.t >= apexT then
          local sacc = (vel - prevV) / ((r.dt + prevDt) / 2) -- signed: + = braking a fall
          maxBrake = math.max(maxBrake, sacc)
          if prevAcc ~= nil then maxStep = math.max(maxStep, math.abs(sacc - prevAcc)) end
          prevAcc = sacc
        end
      end
      if r.recov then recSeen = true end
      if r.brk then brkSeen = true end
      prevV, prevDt = vel, r.dt
      if touchT ~= nil and r.t >= touchT - 1e-9 then
        minAfter = math.min(minAfter, r.pz)
        if lagTD == nil then lagTD = r.pz - final end
        if math.abs(r.pz - final) > 0.01 then settle = r.t - touchT end
      end
    end
    local gap = 0
    for i = 1, #rec do gap = math.max(gap, math.abs(rec[i].pz - rec[i].z - STAND)) end
    return { launchA = maxLaunch, landA = maxLand, under = math.max(0, final - minAfter), lagTD = lagTD or 0, settle = settle, gap = gap,
             active = activeAtOff, endedT = endedT, offT = offT, touchT = touchT, finalZ = rec[#rec].pz, final = final, pv = pv, rec = rec, rays = rays,
             events = events, evKinds = evKinds, zeroOk = zeroOk, zeroFrames = zeroFrames, brake = maxBrake, stepA = maxStep, recSeen = recSeen, brkSeen = brkSeen, rejumpT = rejumpT }
  end
  local function fmt(m) return string.format("launch %.0f, land %.0f m/s2, under %.3f, lag at TD %.2f m, settle %.2f s, gap %.2f", m.launchA, m.landA, m.under,
      m.lagTD, m.settle, m.gap) end
  check("jump-out landing cfg: the new constants exist and the mode is on",
        cfg.jumpOutGov == true and cfg.jumpOutTp == 0.10 and cfg.jumpOutTa == 0.035 and cfg.jumpOutAmax == 60 and cfg.jumpOutAup == 80
        and cfg.jumpOutAbrake == 35 and cfg.jumpOutVth == 3 and cfg.jumpOutEps == 0.10 and cfg.jumpOutRayLen == 8 and cfg.jumpOutLagMax >= 1.2
        and cfg.jumpOutJerk == 300 and cfg.jumpOutDecel == 18 and cfg.jumpOutDrop == 0.3 and cfg.jumpOutRecJerk == 900 and cfg.jumpOutRecDecel == 26)
  local worst = {}
  local sink = {}
  for _, fps in ipairs({ 30, 60, 144 }) do
    for _, kind in ipairs({ "ground", "car", "none" }) do
      local variants = { { name = "plain" },
                         { name = "hitches", hitchL = true, hitchT = true },
                         { name = "jitter+hitches", jit = 0.2, hitchL = true, hitchT = true, seed = 3 },
                         { name = "jitter", jit = 0.2, seed = 7 } }
      for _, vr in ipairs(variants) do
        local o = { ray = kind ~= "none" and kind or nil, jit = vr.jit, hitchL = vr.hitchL, hitchT = vr.hitchT, seed = vr.seed }
        local m = jumpOut(fps, o)
        local hitch = vr.hitchL or vr.hitchT
        local landLim = (kind == "none" and 85 or 70) * (hitch and 1.25 or 1)
        local launchLim = 90 * (hitch and 1.25 or 1)
        check(string.format("jump-out %d fps, ray %s, %s: landing accel <= %.0f m/s2, launch accel <= %.0f, no undershoot, settle <= 0.4 s (%s)", fps, kind,
              vr.name, landLim, launchLim, fmt(m)), m.landA <= landLim and m.launchA <= launchLim and m.under <= 0.01 and m.settle <= 0.4 and m.gap <= cfg.jumpOutLagMax + 1e-6, fmt(m))
        if vr.name == "plain" then sink[fps .. kind] = m.lagTD end
      end
    end
  end
  -- the price: the pivot is still this far above its resting height at the touchdown and sinks on for that distance after it
  print(string.format("jump-out landing: post-touchdown sink (lag at the touchdown, plain): 30 fps ray %.2f / car %.2f / none %.2f m; 60 fps %.2f / %.2f / %.2f m; "
    .. "144 fps %.2f / %.2f / %.2f m", sink["30ground"], sink["30car"], sink["30none"], sink["60ground"], sink["60car"], sink["60none"], sink["144ground"],
    sink["144car"], sink["144none"]))
  -- the mode is armed before the launch edge: the camera does not follow the launch in one frame (the previous rigid follow: 250 - 1400 m/s2)
  do
    local m = jumpOut(60, { ray = "ground" })
    local rec, maxStep = m.rec, 0
    for i = 2, #rec do maxStep = math.max(maxStep, math.abs((rec[i].pz - rec[i - 1].pz) - (rec[i].z - rec[i - 1].z))) end
    local rigid = jumpOut(60, { ray = "ground", armed = false })
    check(string.format("jump-out: armed before the launch edge the launch is limited (%.0f m/s2) where the window-off follow is rigid (%.0f m/s2)", m.launchA,
          rigid.launchA), m.launchA <= 90 and rigid.launchA > 150, string.format("%.0f vs %.0f", m.launchA, rigid.launchA))
    check(string.format("jump-out: the previous rigid follow lands at %.0f m/s2 (what the mode replaces) and this mode at %.0f m/s2", rigid.landA, m.landA),
          rigid.landA > 150 and m.landA <= 70)
  end
  -- 2026-11-04 launch (game log 02-01-16, both jump-outs): the base starts at rest 0.85 m behind V (9.7 m/s within one frame) and the catch-up command asked for
  -- V + 8.5 m/s; the acceleration limits could not brake it: the pivot rose at 13.5 m/s while V rose at 6 m/s, overshot V by 0.6 m (pivot offset +0.70 ->
  -- +2.18 -> +1.58) and braked at -50 m/s2. jumpOutCatch limits the closing speed of the base towards a rising V. Measured from the launch to the apex.
  do
    local function ascent(fps, over, o)
      o = o or {}
      local m = jumpOut(fps, { ray = "ground", over = over, vz0 = 9.73, jit = o.jit, seed = o.seed, hitchL = o.hitchL })
      local apexT = PRE + 9.73 / -GRAV
      local over0, lag0, maxClose, maxA, prevV, prevDt = 0, 0, 0, 0, nil, nil
      for i = 2, #m.rec do
        local r = m.rec[i]
        if r.t < apexT then
          local off = r.pz - r.z - STAND -- the pivot over its resting height above V (> 0: past V)
          over0 = math.max(over0, off)
          lag0 = math.max(lag0, -off)
          local vel = (r.pz - m.rec[i - 1].pz) / r.dt
          local vV = (r.z - m.rec[i - 1].z) / r.dt
          maxClose = math.max(maxClose, vel - vV)
          if prevV ~= nil then maxA = math.max(maxA, math.abs(vel - prevV) / ((r.dt + prevDt) / 2)) end
          prevV, prevDt = vel, r.dt
        end
      end
      return { over = over0, lag = lag0, close = maxClose, accel = maxA }
    end
    local off = { jumpOutCatch = 0 } -- the previous catch-up command (mutation)
    for _, fps in ipairs({ 20, 30, 60, 144 }) do
      for _, vr in ipairs({ { name = "plain" }, { name = "jitter+hitch", jit = 0.2, seed = 5, hitchL = true } }) do
        local a, b = ascent(fps, nil, vr), ascent(fps, off, vr)
        check(string.format("jump-out launch %d fps, %s: the pivot stays within 0.15 m of its resting height over V (overshoot %.2f m, lag %.2f m), at most V + 4.5 m/s "
              .. "(%.1f), launch accel <= 100 m/s2 (%.0f)", fps, vr.name, a.over, a.lag, a.close, a.accel),
              a.over <= 0.15 and a.close <= 4.5 and a.accel <= 100 and a.lag <= 1.0, string.format("overshoot %.2f close %.1f accel %.0f lag %.2f", a.over, a.close, a.accel, a.lag))
        check(string.format("jump-out launch %d fps, %s mutation (jumpOutCatch = 0, the previous catch-up): overshoots V by %.2f m (> 0.3), closes at V + %.1f m/s",
              fps, vr.name, b.over, b.close), b.over > 0.3, string.format("overshoot %.2f close %.1f accel %.0f", b.over, b.close, b.accel))
      end
    end
  end
  -- latch: the hold goes off 0.2 s after the touchdown (vehLandSettle) with the base still above V - the mode stays on, then ends on V; no step at the end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local m = jumpOut(fps, { ray = "ground" })
    check(string.format("jump-out latch %d fps: input off 0.2 s after the touchdown, mode still active (lag open), ended after %.2f s on V, final pivot on the stance height",
          fps, (m.endedT or 99) - m.offT), m.active == true and m.endedT ~= nil and m.endedT - m.offT < 0.5 and not m.pv.joOn and math.abs(m.finalZ - m.final) < 1e-6,
          string.format("active %s end %s final %.5f", tostring(m.active), tostring(m.endedT), m.finalZ - m.final))
  end
  -- window off (non jump-out): the same trajectory is bit for bit the rigid follow of the fast fall block; jo.on = false and jumpOutGov = false too
  do
    local a = jumpOut(60, { ray = "ground", armed = false })
    local b = jumpOut(60, { ray = "ground", over = { jumpOutGov = false } })
    local same = #a.rec == #b.rec
    for i = 1, #a.rec do if a.rec[i].pz ~= b.rec[i].pz then same = false; break end end
    check("jump-out: jumpOutGov = false (hold on) is bit for bit the window-off follow", same)
    -- regular jump (5 m/s, flat) and a 2.5 m ledge drop with the window off: rigid as before
    for _, sc in ipairs({ { "regular jump", { vz0 = 5, drop = 0 } }, { "2.5 m ledge drop", { vz0 = 0, drop = 2.5 } } }) do
      local o = sc[2]
      o.armed, o.ray = false, "ground"
      local m = jumpOut(60, o)
      local rigidLand = m.landA
      check(string.format("jump-out window off, %s: the rigid follow is kept (landing accel %.0f m/s2, over the mode's 70)", sc[1],
            rigidLand), rigidLand > 100)
    end
  end
  -- a 7 m drop (15 m/s at the touchdown) with the ray: no clamp event, landing limits as above
  do
    local m = jumpOut(60, { ray = "ground", drop = 7, vz0 = 0 })
    check(string.format("jump-out: a 7 m drop: landing accel <= 70, no undershoot, no snap / clamp event (%s)", fmt(m)),
          m.landA <= 70 and m.under <= 0.01 and m.pv.snapEv == nil, fmt(m))
  end
  -- 2026-11-04 landing brake (cfg.jumpOutJerk, game log 06-04-45 jumpout_stand / jumpout_move: the camera's vertical acceleration at the touchdown was
  -- 28 - 36 m/s2 in the 10 Hz trace, offline 33 - 35 m/s2 with a +47 m/s2 step in ONE frame when the stopping guard set the acceleration). With a
  -- ground estimate the base brakes with a jerk limited deceleration profile that stops exactly on the ground: the peak is about jumpOutDecel, the
  -- acceleration changes by at most jumpOutJerk * dt per frame. Geometries: the test jump and the two logged ones (launch 9.68 / 9.74 m/s, 2.06 / 2.79 m
  -- below the seat, g = 16.1 fitted from the trace). Plain runs gate the step; with frame time jitter the finite differences are noisy (wider limit).
  -- The price is a higher pivot over V at the touchdown (lag 0.6 - 0.8 m instead of 0.4 m); the head stays in the frame (CM.solve, below).
  local GEOS = { { "test jump", { vz0 = 9.76, drop = 1.85, grav = -16 } },
                 { "logged stand jump-out", { vz0 = 9.68, drop = 2.06, grav = -16.1 } },
                 { "logged move jump-out", { vz0 = 9.74, drop = 2.79, grav = -16.1 } } }
  local function withGeo(geo, extra)
    local o = { ray = "ground" }
    for k, v in pairs(geo) do o[k] = v end
    for k, v in pairs(extra or {}) do o[k] = v end
    return o
  end
  -- framing: the real default profile (2.75 m, height -0.35, FOV 55 = vertical in camera component units), CM.solve on the replayed pivot. V's head (1.75 m,
  -- lowered by a heavy-landing crouch of 0.9 m) and feet against the half FOV. NOT calibrated against the video: at lag 0.34 m the model puts the head
  -- far inside the frame where the r4 video shows it below the edge (the crouch pose and the release blend are not modelled); it bounds the lag effect only.
  local REAL = { name = "Real", distance = 2.75, height = -0.35, side = 0.50, fov = 55, sens = 1.0 }
  local function framing(m, pitch)
    local worstHead, worstFeet = 1e9, 1e9
    for _, r in ipairs(m.rec) do
      if m.touchT ~= nil and r.t >= m.touchT - 0.3 and r.t <= m.touchT + 0.6 then
        local S = CM.newSolver(CM.config())
        local res
        for _ = 1, 2 do
          res = CM.solve(S, { dt = 1 / 60, playerPos = V(0, 0, r.pz - STAND), playerQ = CM.QID, bodyYaw = 0, eyeZ = r.pz, fppPitch = pitch, dyaw = 0, dpitch = 0,
            moving = true, profile = REAL, side = 1, w = 1, zoom = 0 })
        end
        local fwd, up = CM.qrot(res.q, V(0, 1, 0)), CM.qrot(res.q, V(0, 0, 1))
        local function ang(zAbs)
          local rel = CM.sub(V(0, 0, zAbs), res.world)
          return math.deg(math.atan2(CM.dot(rel, up), CM.dot(rel, fwd)))
        end
        worstHead = math.min(worstHead, ang(r.z + 1.75 - 0.9))
        worstFeet = math.min(worstFeet, ang(r.z))
      end
    end
    return worstHead, worstFeet
  end
  local brakeWorst, stepWorst, settleWorst, lagWorst, headWorst, feetWorst = 0, 0, 0, 0, 1e9, 1e9
  for _, g in ipairs(GEOS) do
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _, jv in ipairs({ { name = "plain" }, { name = "jitter", jit = 0.2, seed = 7 } }) do
        local m = jumpOut(fps, withGeo(g[2], { jit = jv.jit, seed = jv.seed }))
        local brakeLim, stepLim = jv.jit and 24 or 22, jv.jit and 22 or 14
        check(string.format("landing brake %d fps, %s, %s: braking peak <= %d m/s2, per-frame acceleration step <= %d, settle <= 0.4 s, no undershoot, no event, no recovery (%.1f, step %.1f, %s)",
              fps, g[1], jv.name, brakeLim, stepLim, m.brake, m.stepA, fmt(m)),
              m.brake <= brakeLim and m.stepA <= stepLim and m.settle <= 0.4 and m.under <= 0.01 and m.events == 0 and not m.recSeen and m.brkSeen and m.gap <= cfg.jumpOutLagMax,
              string.format("brake %.1f step %.1f rec %s brk %s %s", m.brake, m.stepA, tostring(m.recSeen), tostring(m.brkSeen), fmt(m)))
        brakeWorst, stepWorst, settleWorst, lagWorst = math.max(brakeWorst, m.brake), math.max(stepWorst, m.stepA), math.max(settleWorst, m.settle), math.max(lagWorst, m.lagTD)
        if jv.name == "plain" and fps == 60 then
          for _, pitch in ipairs({ -4.6, -14.3 }) do
            local hd, ft = framing(m, pitch)
            headWorst, feetWorst = math.min(headWorst, hd), math.min(feetWorst, ft)
          end
        end
      end
    end
  end
  print(string.format("landing brake: worst over the matrix: braking peak %.1f m/s2, acceleration step %.1f, settle %.2f s, lag at the touchdown %.2f m; framing at 60 fps (pitch -4.6 / -14.3): "
    .. "head with a 0.9 m crouch %.1f deg, feet %.1f deg (half FOV 27.5)", brakeWorst, stepWorst, settleWorst, lagWorst, headWorst, feetWorst))
  check(string.format("landing brake framing: V's head (heavy-landing crouch) stays inside the vertical FOV with 3 deg margin through the landing (%.1f deg, limit -24.5)", headWorst), headWorst >= -24.5)
  -- mutation: without the jerk limited brake (the previous guard) the same jumps brake at 33 - 35 m/s2 with a 45 - 50 m/s2 step in one frame
  for _, g in ipairs(GEOS) do
    local m = jumpOut(60, withGeo(g[2], { over = { jumpOutJerk = 0 } }))
    check(string.format("landing brake mutation (jumpOutJerk = 0, the previous guard), 60 fps, %s: braking peak %.1f > 30 m/s2 and acceleration step %.1f > 35", g[1], m.brake, m.stepA),
          m.brake > 30 and m.stepA > 35 and not m.brkSeen)
  end
  -- a ground estimate that falls (the first hit was a car roof) hands over to the recovery brake; a plain ground never does (checked above: no recovery)
  for _, fps in ipairs({ 30, 60, 144 }) do
    local m = jumpOut(fps, { ray = "car" })
    check(string.format("landing brake %d fps, ray car: the roof estimate falls -> recovery brake (jumpOutRecJerk) entered, braking peak %.1f <= 45 m/s2, settle %.2f <= 0.4 s",
          fps, m.brake, m.settle), m.recSeen and m.brake <= 45 and m.settle <= 0.4 and m.under <= 0.01)
  end
  do
    -- mutation: without the recovery brake (jumpOutRecJerk = 0: the guard of the previous behaviour after the drop) at least one of the car runs
    -- settles late (> 0.4 s) or brakes harder than 45 m/s2
    local bad, worstS, worstB = 0, 0, 0
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _, hv in ipairs({ { name = "plain" }, { name = "hitches", hitchL = true, hitchT = true }, { name = "jitter", jit = 0.2, seed = 7 } }) do
        local mm = jumpOut(fps, { ray = "car", over = { jumpOutRecJerk = 0 }, jit = hv.jit, seed = hv.seed, hitchL = hv.hitchL, hitchT = hv.hitchT })
        if mm.settle > 0.4 or mm.brake > 45 then bad = bad + 1 end
        worstS, worstB = math.max(worstS, mm.settle), math.max(worstB, mm.brake)
      end
    end
    check(string.format("landing brake ray car mutation (jumpOutRecJerk = 0): %d of 9 runs settle late or brake harder (worst settle %.2f s, braking %.1f)", bad, worstS, worstB), bad >= 1)
  end
  -- the no-ground path is the spring as before: the same trajectory with the brake on and off, bit for bit
  for _, fps in ipairs({ 30, 60, 144 }) do
    local a, c2 = jumpOut(fps, {}), jumpOut(fps, { over = { jumpOutJerk = 0 } })
    local same = #a.rec == #c2.rec
    for i = 1, #a.rec do if a.rec[i].pz ~= c2.rec[i].pz then same = false; break end end
    check(string.format("landing brake %d fps: without a ground estimate the pivot is bit for bit the one with jumpOutJerk = 0", fps), same and not a.brkSeen)
  end
  -- 2026-11-04 (review finding 4): the follower is integrated in sub-steps, so a frame of any length is stable. A 0.25 s hitch on the touchdown frame, on
  -- the launch frame and on both; a repeated dt = 0 frame on the launch and the touchdown frame. The same limits as the 37 ms hitches (x 1.25), no
  -- clamp / snap event in any frame, no undershoot over 1 cm, the pivot ends on the stance height.
  for _, fps in ipairs({ 30, 60, 144 }) do
    for _, kind in ipairs({ "ground", "car", "none" }) do
      local variants = { { name = "0.25 s hitch on the touchdown frame", hitchT = 0.25 },
                         { name = "0.25 s hitch on the launch frame", hitchL = 0.25 },
                         { name = "0.25 s hitches on both", hitchL = 0.25, hitchT = 0.25 },
                         { name = "dt = 0 frame before the launch and the touchdown frame", zeroL = true, zeroT = true },
                         { name = "0.25 s hitches + dt = 0 frames", hitchL = 0.25, hitchT = 0.25, zeroL = true, zeroT = true } }
      for _, vr in ipairs(variants) do
        local m = jumpOut(fps, { ray = kind ~= "none" and kind or nil, hitchL = vr.hitchL, hitchT = vr.hitchT, zeroL = vr.zeroL, zeroT = vr.zeroT })
        local hitch = vr.hitchL or vr.hitchT
        local landLim = (kind == "none" and 85 or 70) * (hitch and 1.25 or 1)
        local launchLim = 90 * (hitch and 1.25 or 1)
        local zeroNeeded = (vr.zeroL or vr.zeroT) and 2 or 0
        check(string.format("jump-out %d fps, ray %s, %s: landing accel <= %.0f, launch accel <= %.0f, no clamp / snap event, undershoot <= 1 cm, settles (%s, events %d)",
              fps, kind, vr.name, landLim, launchLim, fmt(m), m.events),
              m.landA <= landLim and m.launchA <= launchLim and m.events == 0 and m.under <= 0.01 and m.settle <= 0.4 and m.gap <= cfg.jumpOutLagMax + 1e-6
              and math.abs(m.finalZ - m.final) < 1e-6 and m.zeroOk and m.zeroFrames == zeroNeeded, fmt(m) .. " events " .. table.concat(m.evKinds, ","))
      end
    end
  end
  -- 2026-11-04 r11 (review finding 3): the latched landing brake drove the base onto the stale ground estimate G and never read V again. (a) V lands on a curb /
  -- sill / roof edge the centre ray missed (stops above G): the pivot kept lowering below V's real stance and then rebounded. (b) V jumps again during the brake:
  -- the base kept descending while V rose. The latched brake now respects V (CM.jumpOutStep, jumpOutRetarget = false: the previous behaviour).
  for _, fps in ipairs({ 30, 60, 144 }) do
    for _, curb in ipairs({ 0.15, 0.30 }) do
      local m = jumpOut(fps, { ray = "ground", curb = curb })
      local b = jumpOut(fps, { ray = "ground", curb = curb, over = { jumpOutRetarget = false } })
      check(string.format("landing brake %d fps, V lands %.2f m above the ground the ray sees: the pivot never goes below V's stance (undershoot %.3f m), settles in %.2f s, landing accel %.0f <= 70, "
            .. "per-frame accel step %.1f <= 22, no event", fps, curb, m.under, m.settle, m.landA, m.stepA),
            m.under <= 0.02 and m.settle <= 0.5 and m.landA <= 70 and m.stepA <= 22 and m.events == 0 and math.abs(m.finalZ - m.final) < 1e-6,
            string.format("under %.3f settle %.2f landA %.0f stepA %.1f events %d final %.4f", m.under, m.settle, m.landA, m.stepA, m.events, m.finalZ - m.final))
      check(string.format("landing brake %d fps, curb %.2f m mutation (jumpOutRetarget = false): the pivot sinks %.3f m below V's stance and rebounds", fps, curb, b.under), b.under >= 0.9 * curb)
    end
    for _, ra in ipairs({ 0.05, 0.12, 0.2 }) do
      local o = { ray = "ground", rejumpAt = ra, rejumpV = 5, dur = 4, offAfter = 1.5 }
      local m = jumpOut(fps, o)
      o.over = { jumpOutRetarget = false }
      local b = jumpOut(fps, o)
      -- 0.45 s from the second take-off (before the second touchdown): how far the base keeps descending, and its acceleration step per frame
      local function window(mm)
        local pz0, low, stepMax, prevV, prevAcc = nil, 1e9, 0, nil, nil
        for i = 2, #mm.rec do
          local r, q = mm.rec[i], mm.rec[i - 1]
          if r.t >= mm.rejumpT and pz0 == nil then pz0 = r.pz end
          if pz0 ~= nil and r.t <= mm.rejumpT + 0.45 then
            low = math.min(low, r.pz)
            local vel = (r.pz - q.pz) / r.dt
            if prevV ~= nil then
              local acc = (vel - prevV) / ((r.dt + q.dt) / 2)
              if prevAcc ~= nil then stepMax = math.max(stepMax, math.abs(acc - prevAcc)) end
              prevAcc = acc
            end
            prevV = vel
          end
        end
        return pz0 - low, stepMax
      end
      local d1, s1 = window(m)
      local d2 = window(b)
      check(string.format("landing brake %d fps, V jumps again %.2f s after the touchdown: the base keeps descending %.3f m (previous behaviour %.3f m), accel step per frame %.1f <= 35, no event, "
            .. "the mode ends on the stance height", fps, ra, d1, d2, s1),
            d1 <= 0.6 * d2 + 0.01 and s1 <= 35 and m.events == 0 and math.abs(m.finalZ - m.final) < 1e-6 and not m.pv.joOn,
            string.format("descent %.3f vs %.3f step %.1f events %d final %.5f on %s", d1, d2, s1, m.events, m.finalZ - m.final, tostring(m.pv.joOn)))
    end
  end
  -- any frame time in one go: a constant dt of 1/30 .. 0.5 s through the whole jump (explicit integration without sub-steps overshoots and clamps)
  for _, dtc in ipairs({ 0.05, 0.1, 0.25, 0.5 }) do
    local worstUnder, ev = 0, 0
    for _, kind in ipairs({ "ground", "none" }) do
      local m = jumpOut(1 / dtc, { ray = kind ~= "none" and kind or nil, dur = 6 })
      worstUnder, ev = math.max(worstUnder, m.under), ev + m.events
    end
    check(string.format("jump-out constant frame time %.2f s through the whole jump: no clamp event, undershoot <= 1 cm (%.3f), no NaN", dtc, worstUnder),
          ev == 0 and worstUnder <= 0.01)
  end
  -- finding 10: the failsafe (jumpOutHoldMax after the hold went off) must not end the mode in the middle of a fall. A slow fall (g -3, 30 m, 4.5 s)
  -- with the hold off 0.1 s after the launch: the mode is still on at 3 s after the hold went off, and ends after the touchdown with no clamp event
  do
    local m = jumpOut(60, { ray = "ground", vz0 = 0, drop = 30, grav = -3, offAtT = PRE + 0.1, dur = 8 })
    local failT = m.offT + cfg.jumpOutHoldMax
    local midFall, offMid = 0, 0
    for _, r in ipairs(m.rec) do
      if r.t > failT + 0.05 and r.t < m.touchT - 0.05 then midFall = midFall + 1; if not r.on then offMid = offMid + 1 end end
    end
    check(string.format("jump-out failsafe: a fall longer than jumpOutHoldMax keeps the mode (%d frames after the failsafe time, %d with the mode off)", midFall, offMid),
          midFall > 20 and offMid == 0 and m.touchT > failT + 0.3)
    check(string.format("jump-out failsafe: ... no clamp event, landing accel <= 70, undershoot <= 1 cm, the mode ends after the touchdown on the stance height (%s, events %d)", fmt(m), m.events),
          m.events == 0 and m.landA <= 70 and m.under <= 0.01 and m.endedT ~= nil and m.endedT > m.touchT and not m.pv.joOn and math.abs(m.finalZ - m.final) < 1e-6)
  end
  -- finding 10: V stays slowly in motion after the touchdown (a slope, 1 m/s): the follower never settles, the failsafe ends the mode after jumpOutHoldMax s
  -- with V calm and the lag inside pivotLagMax. The base and its speed are carried over: no step, no event around the hand-over
  do
    local m = jumpOut(60, { ray = "ground", climb = 1.0, dur = 6 })
    local endT = m.endedT
    local maxStep, maxRef, at = 0, 0, nil
    for i = 2, #m.rec do
      local r, q = m.rec[i], m.rec[i - 1]
      local step = math.abs((r.pz - r.z) - (q.pz - q.z))
      if endT ~= nil and r.t >= endT - 0.2 and r.t <= endT + 0.5 then
        if step > maxStep then maxStep, at = step, r.t end
      elseif endT ~= nil and r.t > m.touchT + 0.8 and r.t < endT - 0.3 then
        maxRef = math.max(maxRef, step)
      end
    end
    check(string.format("jump-out failsafe on a slope: the mode ends %.2f s after the hold went off (jumpOutHoldMax %.1f) with no event (%d)", (endT or 99) - (m.offT or 0),
          cfg.jumpOutHoldMax, m.events), endT ~= nil and endT - m.offT >= cfg.jumpOutHoldMax - 0.02 and endT - m.offT < cfg.jumpOutHoldMax + 0.2 and m.events == 0)
    -- (the normal spring's no-overshoot rule may stall the pivot for one frame at the hand-over: at most V's own one-frame step, no jump)
    check(string.format("jump-out failsafe on a slope: the hand-over moves the shown pivot against V by at most %.4f m per frame (V's own frame step 0.0167, before the hand-over %.4f)",
          maxStep, maxRef), maxStep <= 1.0 / 60 + 1e-4 and maxStep <= maxRef + 1.0 / 60 + 1e-4)
  end
  -- the last resort: V still moves 10 s after the hold went off (here a lift at 4 m/s): the mode hands over with the lag inside pivotLagMax (one logged clamp)
  do
    local pv = CM.newPivot()
    local z = 0
    for _ = 1, 20 do CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, { on = true }) end
    local ended
    for i = 1, 60 * 12 do
      z = z + 4 / 60
      CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, { on = i <= 1 })
      if not pv.joOn then ended = i / 60; break end
    end
    check(string.format("jump-out failsafe: V rising at 4 m/s keeps the mode past jumpOutHoldMax and hands over at jumpOutHoldHard (%.2f s)", ended or -1),
          ended ~= nil and ended > cfg.jumpOutHoldMax + 1 and math.abs(ended - cfg.jumpOutHoldHard) < 0.1 and math.abs(pv.base - z) <= (cfg.pivotLagMax or 1) + 1e-6)
  end
  -- a teleport while the mode runs still snaps (and the mode re-arms on the next frame from the new base)
  do
    local pv = CM.newPivot()
    local z = 0
    for _ = 1, 20 do CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, { on = true }) end
    local p = CM.pivotStep(pv, cfg, 1 / 60, V(300, 0, 50), nil, false, nil, nil, nil, nil, nil, { on = true })
    check("jump-out: a 50 m teleport still snaps the pivot onto V at once and drops the mode state", near(p.z, 50 + STAND, 1e-9) and pv.snapEv ~= nil
          and pv.snapEv.kind == "teleport" and pv.joOn == nil)
    local p2 = CM.pivotStep(pv, cfg, 1 / 60, V(300, 0, 50), nil, false, nil, nil, nil, nil, nil, { on = true })
    check("jump-out: the mode re-arms on the frame after the teleport without a step", near(p2.z, 50 + STAND, 1e-9) and pv.joOn == true)
  end
  -- standing hold (the ride, V on the seat): the armed mode follows a 0.68 m seat step without a jump and ends on V when the hold is off
  do
    local pv = CM.newPivot()
    for _ = 1, 20 do CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, 0), nil, false, nil, nil, nil, nil, nil, { on = true }) end
    local prev, maxStep, z = pv.base, 0, 0.68
    for _ = 1, 60 do
      CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, { on = true })
      maxStep = math.max(maxStep, math.abs(pv.base - prev))
      prev = pv.base
    end
    check(string.format("jump-out: a 0.68 m seat step while armed is followed continuously (max step %.3f m) and settles on V", maxStep), maxStep < 0.68 and math.abs(pv.base - z) < 0.01)
    for _ = 1, 10 do CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, nil, nil, nil, nil, nil, { on = false }) end
    check("jump-out: with the hold off and the base on V the mode ends", pv.joOn == nil)
  end
end

print(string.format("utpp_camera math test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_camera math test failed")

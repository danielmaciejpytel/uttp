-- UTPP utpp_camera: ragdoll camera (2026-11-03; cfg.ragdollCam / rag*): the camera while V is ragdolled after a car hit and while she gets up.
--  * CM.solve with the DLL natives' inputs (inp.ragdoll = { state, knock, pelvis (model space), fresh, noLook }): scripted episode - a fast
--    tumble (0.85 s, root xy 12 m/s decaying, root z arch of 2 m = 7 m/s vertical, entity spinning 720 deg/s, pelvis in MODEL space),
--    settle, blend-out (state 2), the PSM knockdown get-up with a stale pelvis, then standing.
--  * pivot = world pelvis + lift (spring), continuity at the enter and at the exit, bounded steps and accelerations after the tumble, no
--    snap / clamp / teleport events (the capsule pivot is pinned), boom longer + pitch biased down (look state untouched), the look is
--    free (world yaw = the mouse only, the spinning body does not turn the camera), natives absent / state 0 / switch off = the old camera
--    bit for bit, a state stuck at 1 (time cap, no second episode), the PSM knockdown alone never starts an episode, a teleport inside the
--    episode, a vehicle hold (noLook) keeps its look path, CM.pivotStep pin.
--  * utpp_camera/init.lua in the mocked CET game of test_utpp_camera_load.lua (its harness is taken from that file): natives absent -> the
--    normal camera with one log line; the episode logs one ON / one OFF line, the heading lock is frozen (a lock lost to the game's
--    camera parameter set is not set again during the episode, set again after it + grace; the episode's free yaw does not set the lock), a menu
--    gap inside the episode. NOT asserted (code only, check in the game): the body-turn prediction and the mouse calibration gates of init.lua.
-- Every check that matters fails when the cfg switch is flipped (the mutation list is in the check names: [M1] ragdollCam = false, [M2]
-- ragSmoothActive = 0, [M3] ragExitBlend = 0, [M4] ragDistAdd / ragPitchBias = 0).
-- Globals from the runner: UTPP_CAMERA_MATH_PATH, UTPP_CAMERA_PATH, UTPP_CAMERA_DIR, UTPP_CORE_PATH, UTPP_LUA_FIXTURE_DIR.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, (not ok and detail ~= nil) and (" -- " .. tostring(detail)) or ""))
end
local CM = dofile(UTPP_CAMERA_MATH_PATH)
local V = CM.v3
local MEDIUM = { name = "Medium", distance = 2.10, height = 0.12, side = 0.50, fov = 56, sens = 1.0 }
local function near(a, b, eps) return math.abs(a - b) <= (eps or 1e-6) end

-- ================================================================ the scripted episode
local T0 = 1.0 -- the hit
local TUMBLE = 0.85 -- the ragdoll owns the body (state 1)
local BLENDOUT = 0.65 -- state 2 (the engine disabled the ragdoll, the get-up plays)
local KNOCK_END = 4.5 -- PSM knockdown (LocomotionDetailed 29) over
local STAND = 6.5 -- well after the episode (ragEndHold + ragExitBlend)
local function ss(x) x = math.max(0, math.min(1, x)); return x * x * (3 - 2 * x) end
local function body(t)
  local a = math.max(0, math.min(1, (t - T0) / 0.5)) -- the horizontal tumble: 12 m/s decaying to 0 within 0.5 s (2 m)
  local ua = math.max(0, math.min(1, (t - T0) / 0.4)) -- the airborne arch: 0.4 s, root 0.6 m / pelvis 1.2 m high (5 / 9 m/s vertical)
  local arch = math.sin(math.pi * ua) ^ 2
  local root = V(2.0 * (1 - (1 - a) ^ 3) + ((t < T0) and 5 * (t - T0) or 0), 0.3 * a, 0.6 * arch) -- V runs 5 m/s along +x before the hit
  -- the engine attaches the entity root to the body while the ragdoll owns it: the root z is 1.3 m high from 0.5 s on and drops back to the
  -- ground when the ragdoll ends (1.3 m in ONE frame = 78 m/s: what the capsule pivot would take for a teleport)
  if t >= T0 + 0.5 and t < T0 + TUMBLE then root.z = root.z + 1.3 end
  local pz = 0.25 + 1.2 * arch -- the pelvis lies 0.25 m over the ground when settled
  if t > T0 + TUMBLE then pz = 0.25 + 0.25 * ss((t - T0 - TUMBLE) / BLENDOUT) end -- starts to get up
  local yaw = 720 * math.max(0, math.min(t - T0, TUMBLE))
  return root, V(root.x, root.y, pz), yaw
end
local function stateAt(t)
  if t < T0 then return 0 elseif t < T0 + TUMBLE then return 1 elseif t < T0 + TUMBLE + BLENDOUT then return 2 end
  return 0
end
local function inEpisodeMotion(t) return t >= T0 end
-- one run: returns the per-frame records. o: fps, secs, cfg (overrides), natives ("full" | "none" | "idle"), mouse (deg/s over [1.2, 1.7]),
-- noLook, stuck (state 1 for ever, knock on), knockOnly, teleportAt (root moves 100 m)
local function run(o)
  o = o or {}
  local fps = o.fps or 60
  local dt = 1 / fps
  local cfg = CM.config(o.cfg or {})
  cfg.idleOrbit = false
  if (o.cfg or {}).ragFrame == nil then cfg.ragFrame = false end -- (r13b: the scripted pelvis of this episode jumps at the enter; the framing constraint has its own tests, 6d)
  local S = CM.newSolver(cfg)
  local rec = {}
  local snapEvents, prev = 0, nil
  for i = 1, math.floor((o.secs or 8) * fps + 0.5) do
    local t = i * dt
    local root, pelvisW, yaw = body(t)
    if o.teleportAt and t >= o.teleportAt then root = V(root.x + 100, root.y, root.z); pelvisW = V(pelvisW.x + 100, pelvisW.y, pelvisW.z) end
    -- the standing V of the first second and the end (root z 0, upright, yaw 0)
    local q = CM.qFromYawPitch(yaw, 0)
    local state = o.stuck and (t >= T0 and 1 or 0) or stateAt(t)
    local knock = (t >= T0 + TUMBLE and t < KNOCK_END) or (o.stuck and t >= T0) or (o.knockOnly and t >= T0)
    if o.knockOnly then state = 0 end
    local ragIn
    if o.natives ~= "none" and not (o.nativesCutAt and t >= o.nativesCutAt) then
      ragIn = { state = state, knock = knock, noLook = o.noLook == true }
      if state ~= 0 and o.natives ~= "idle" then
        local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root)) -- the pelvis in V's model space
        ragIn.pelvis, ragIn.fresh = m, true
      end
      if o.natives == "idle" then ragIn.state, ragIn.knock = 0, false end
      if o.getup then ragIn.getup = o.getup(t) end -- 2026-11-04 r13: the native get-up input { state, frac } of the frame (nil = none)
    end
    local mouse = (o.mouse and t >= 1.2 and t < 1.7) and o.mouse * dt or 0
    if o.mouseFn then mouse = o.mouseFn(t, dt) end -- r13e: a scripted look input (deg of this frame)
    local res = CM.solve(S, { dt = dt, playerPos = root, playerQ = q, bodyYaw = yaw, eyeZ = root.z + 0.3, fppPitch = 0, dyaw = mouse,
      dpitch = 0, moving = false, profile = MEDIUM, side = 1, w = 1, zoom = 0, crouch = false, ragdoll = ragIn })
    local sev = S.pivot.snapEv
    if sev ~= nil then snapEvents = snapEvents + 1; S.pivot.snapEv = nil end
    local r = { t = t, res = res, root = root, pelvisW = pelvisW, yaw = yaw, pivot = res.pivot, phase = res.ragdoll and res.ragdoll.phase,
                ev = res.ragdoll and res.ragdoll.ev, mouse = mouse, state = state }
    if prev ~= nil then r.step = CM.dist(res.pivot, prev.pivot); r.stepZ = res.pivot.z - prev.pivot.z end
    rec[i] = r
    prev = r
  end
  return rec, S, snapEvents
end
local function at(rec, t) local fps = math.floor(1 / (rec[2].t - rec[1].t) + 0.5); return rec[math.floor(t * fps + 0.5)] end
local function maxOver(rec, t0, t1, fn)
  local m, when = 0, nil
  for _, r in ipairs(rec) do
    if r.t >= t0 and r.t <= t1 then
      local v = fn(r)
      if v ~= nil and v > m then m, when = v, r.t end
    end
  end
  return m, when
end
local function evCount(rec, kind) local n = 0 for _, r in ipairs(rec) do if r.ev and r.ev.kind == kind then n = n + 1 end end return n end

-- ================================================================ 1) pin of CM.pivotStep
do
  local cfg = CM.config()
  local function fall(pin)
    local pv, ev = CM.newPivot(), 0
    local pos = V(0, 0, 0)
    for i = 1, 120 do
      local t = i / 60
      local z = (t < 1.0) and 0 or (2.0 * math.sin(math.pi * math.min(1, (t - 1.0) / 0.85)) ^ 2) -- 7 m/s arch (own arch for this unit test)
      CM.pivotStep(pv, cfg, 1 / 60, V(0, 0, z), nil, false, false, 0, false, false, pin)
      if pv.snapEv then ev = ev + 1; pv.snapEv = nil end
      pos = V(0, 0, z)
    end
    return ev, pv
  end
  local evFree, pvFree = fall(false)
  local evPin, pvPin = fall(true)
  check("pin: control - the unpinned capsule pivot meets the 7 m/s arch with fast mode / clamp events (the arch is a real test of the pin)",
        evFree > 0 or pvFree.fastOn ~= nil, evFree)
  check("pin: pinned = no snap / clamp event, base = V's height every frame, vertical velocity 0", evPin == 0 and pvPin.vb == 0
        and near(pvPin.base, 0, 1e-9) and pvPin.fastOn == false, string.format("%d events, vb %.3f base %.3f", evPin, pvPin.vb, pvPin.base))
end

-- ================================================================ 2) the episode at 60 fps
local base = run({ cfg = { ragdollCam = false }, mouse = 30 }) -- control = the old camera with the same inputs
local rec, S, snaps = run({ mouse = 30 })
do
  local enter = at(rec, T0)
  local before = at(rec, T0 - 1 / 60)
  check("enter: the episode starts at the first frame of state 1 (one enter event)", evCount(rec, "enter") == 1 and enter.ev ~= nil
        and enter.ev.kind == "enter" and rec[1].ev == nil and enter.phase == "on")
  local before2 = at(rec, T0 - 2 / 60)
  local acc = CM.len(CM.sub(CM.sub(enter.pivot, before.pivot), CM.sub(before.pivot, before2.pivot)))
  check("enter: continuity - the first ragdoll frame continues the shown motion (5 m/s run-up): the pivot's second difference <= 5 cm (spring from the shown pivot + velocity) [M2]",
        acc <= 0.05, string.format("%.3f m", acc))
  local c1 = math.abs(enter.step - before.step)
  check("enter: no step - the first ragdoll step differs from the last normal one by <= 4 cm [M2]", c1 <= 0.04, string.format("%.3f m", c1))
  -- the pivot is at the capsule pivot (root z + standHeight) before the hit
  local cap = before.root.z + CM.config().standHeight
  check("before the hit: the normal capsule pivot (root + standHeight), natives present with state 0", near(before.pivot.z, cap, 1e-6),
        before.pivot.z)
  -- tracking the world pelvis + lift while the body lies (state 1 end / state 2): within 10 cm, far from the capsule height [M1]
  local worst = 0
  for _, r in ipairs(rec) do
    if r.t >= T0 + 0.65 and r.t <= T0 + TUMBLE then
      local tgt = V(r.pelvisW.x, r.pelvisW.y, r.pelvisW.z + CM.config().ragLift)
      worst = math.max(worst, CM.dist(r.pivot, tgt))
    end
  end
  check("lying (state 1, body at rest): the pivot follows the world pelvis + lift within 10 cm (not the capsule height of the root + 1.55 m) [M1]", worst < 0.10,
        string.format("%.3f m", worst))
  local rr = at(rec, T0 + 0.8)
  check("lying: pivot z is below 1.2 m while the capsule pivot would be 1.55 m over the root [M1]", rr.pivot.z < 1.2, rr.pivot.z)
  -- the model-space convention: target = root + qrot(q, model) + lift = world pelvis + lift, and toLocal returns the model point
  local worstT, worstRT = 0, 0
  for _, r in ipairs(rec) do
    if r.state ~= 0 and r.res.ragdoll and r.res.ragdoll.target then
      local tg = r.res.ragdoll.target
      worstT = math.max(worstT, CM.dist(tg, V(r.pelvisW.x, r.pelvisW.y, r.pelvisW.z + CM.config().ragLift)))
      local q = CM.qFromYawPitch(r.yaw, 0)
      local lp = CM.toLocal(r.root, q, r.pelvisW, CM.QID)
      local back = CM.add(r.root, CM.qrot(q, lp))
      worstRT = math.max(worstRT, CM.dist(back, r.pelvisW))
    end
  end
  check("convention: the world pelvis target = entity position + entity rotation * model pelvis + lift (the entity spins 720 deg/s)",
        worstT < 1e-9, worstT)
  check("convention: CM.toLocal round trip of the pelvis through the spinning entity", worstRT < 1e-9, worstRT)
  -- steps after the tumble: speed <= 3 m/s (5 cm at 60 fps) and acceleration <= 1.5 cm per frame^2 from the end of the tumble + settle on
  local stepMax, stepAt = maxOver(rec, T0 + TUMBLE + 0.35, 8, function(r) return r.step end)
  check("steps: no pivot step above 5 cm per frame from 0.35 s after the tumble to the end (get-up, exit blend, standing) [M3]",
        stepMax <= 0.05, string.format("%.3f m at t=%.2f", stepMax, stepAt or -1))
  local accMax, accAt = maxOver(rec, T0 + TUMBLE + 0.35, 8, function(r)
    local pr = rec[math.floor(r.t * 60 + 0.5) - 1]
    return (pr and pr.step) and math.abs(r.step - pr.step) or nil
  end)
  check("steps: no pivot jerk above 1.5 cm per frame^2 from 0.35 s after the tumble on [M3]", accMax <= 0.015,
        string.format("%.4f m at t=%.2f", accMax, accAt or -1))
  local tumbleMax = maxOver(rec, T0, T0 + TUMBLE + 0.35, function(r) return r.step end)
  check("steps: the tumble itself is followed (steps up to 20 cm per frame = the body's 12 m/s), no frame above 25 cm", tumbleMax <= 0.25,
        tumbleMax)
  check("no pivot snap / clamp events in the whole episode (the root z jumps 1.3 m twice in one frame, arch of 5 m/s) [M1]", snaps == 0, snaps)
  local tele = 0
  for _, r in ipairs(rec) do if r.res.teleport then tele = tele + 1 end end
  check("no solver teleport flag in the episode", tele == 0, tele)
  -- exit: on -> out -> nil, one exit event, continuity at the exit frame and through the crossfade
  local exitRec
  for _, r in ipairs(rec) do if r.ev and r.ev.kind == "exit" then exitRec = r break end end
  check("exit: one exit event after the PSM knockdown (4.5 s) + ragEndHold, then phase out, then none", evCount(rec, "exit") == 1
        and exitRec ~= nil and exitRec.t >= KNOCK_END + 0.09 and exitRec.t <= KNOCK_END + 0.2 and exitRec.phase == "out"
        and at(rec, STAND).phase == nil, exitRec and exitRec.t)
  if exitRec then
    local prevR = rec[math.floor(exitRec.t * 60 + 0.5) - 1]
    local jumpX = CM.dist(exitRec.pivot, prevR.pivot)
    check("exit: continuity - the exit frame's pivot is within 3 cm of the last episode frame's (crossfade starts at the spring pivot) [M3]",
          jumpX <= 0.03, string.format("%.3f m", jumpX))
    local after = at(rec, exitRec.t + 0.4 + 0.05)
    check("exit: after ragExitBlend the pivot is the capsule pivot exactly", near(after.pivot.z, after.root.z + CM.config().standHeight, 1e-9)
          and after.phase == nil, string.format("%.4f vs %.4f, phase %s", after.pivot.z, after.root.z + CM.config().standHeight, tostring(after.phase)))
  end
  local fin = at(rec, STAND)
  check("standing again: pivot = root + standHeight, boom distance = the profile's", near(fin.pivot.z, fin.root.z + CM.config().standHeight, 1e-9)
        and near(fin.res.fullLen, base[#base].res.fullLen, 1e-9), fin.pivot.z)
end

-- ================================================================ 3) boom and look
do
  local lyingT = T0 + 2.0
  local a, b = at(rec, lyingT), at(base, lyingT)
  local cfg = CM.config()
  check("boom: longer while the body lies (+0.6 m at full weight) [M4]", a.res.fullLen >= b.res.fullLen + 0.54,
        string.format("%.3f vs %.3f", a.res.fullLen, b.res.fullLen))
  check("boom: pitched down at the body (boom pitch at least 8 deg lower than the normal camera's; ragPitchBias -10) [M4]",
        a.res.boomPitch <= b.res.boomPitch - 8, string.format("%.2f vs %.2f", a.res.boomPitch, b.res.boomPitch))
  check("boom: the look state is untouched - the reported look pitch equals the normal camera's (no write-back into the game's look)",
        near(a.res.pitch, b.res.pitch, 1e-9), string.format("%.3f vs %.3f", a.res.pitch, b.res.pitch))
  -- 2026-11-03 review r2 #12: res.shownPitch = the pitch the camera quaternion shows (look pitch + the boom bias); without the episode it is res.pitch
  local function qPitch(q)
    local f = CM.qrot(q, CM.v3(0, 1, 0))
    return math.deg(math.asin(math.max(-1, math.min(1, f.z))))
  end
  check("shownPitch: while the body lies it is the pitch of res.q (about 10 deg under the look pitch), not res.pitch",
        near(qPitch(a.res.q), a.res.shownPitch, 1e-6) and a.res.shownPitch <= a.res.pitch - 8, string.format("%.3f / %.3f / %.3f", qPitch(a.res.q), a.res.shownPitch or -999, a.res.pitch))
  check("shownPitch: the normal camera's equals its look pitch", near(b.res.shownPitch, b.res.pitch, 1e-12), string.format("%.3f vs %.3f", b.res.shownPitch or -999, b.res.pitch))
  -- weights: in over ragBlendIn, out over ragBlendOut, never a step
  local maxW = 0
  for i = 2, #rec do
    local w1 = rec[i].res.ragdoll and rec[i].res.ragdoll.w or 0
    local w0 = rec[i - 1].res.ragdoll and rec[i - 1].res.ragdoll.w or 0
    maxW = math.max(maxW, math.abs(w1 - w0))
  end
  check("boom weight: changes by <= 1.5/(ragBlendIn * 60) per frame (smoothstep of a linear ramp, <= 5.2 %)", maxW <= 0.052, maxW)
  local fullStep = maxOver(rec, 0.5, 8, function(r) local p = rec[math.floor(r.t * 60 + 0.5) - 1]; return p and math.abs(r.res.fullLen - p.res.fullLen) or nil end)
  check("boom length: no step above 3.5 cm per frame (blend, not a jump) [M4]", fullStep <= 0.035, fullStep)
  -- look free: the camera yaw = the mouse only while the body spins 720 deg/s (and mouse input still turns it)
  local y0, y1 = at(rec, T0 + 0.02).res.yaw, at(rec, T0 + TUMBLE).res.yaw
  local mouseSum = 0
  for _, r in ipairs(rec) do if r.t > T0 + 0.02 and r.t <= T0 + TUMBLE then mouseSum = mouseSum + r.mouse end end
  local turned = CM.wrapDeg(y1 - y0)
  check("look: free during the tumble - the world yaw turns by the mouse (30 deg/s for 0.5 s) and not with the spinning body [M1]",
        math.abs(CM.wrapDeg(turned + mouseSum)) < 1.0 or math.abs(CM.wrapDeg(turned - mouseSum)) < 1.0,
        string.format("camera %.2f deg, mouse %.2f deg", turned, mouseSum))
  check("look: the mouse still turns the camera (not locked)", math.abs(mouseSum) > 10 and math.abs(turned) > 10, turned)
  -- the same without any mouse input: the ragdoll camera holds its world yaw, the old follow camera is carried by the spinning body
  local r0 = run({})
  local b0 = run({ cfg = { ragdollCam = false } })
  local ry = CM.wrapDeg(at(r0, T0 + TUMBLE).res.yaw - at(r0, T0 + 0.02).res.yaw)
  local oy = CM.wrapDeg(at(b0, T0 + TUMBLE).res.yaw - at(b0, T0 + 0.02).res.yaw)
  check("look: no mouse - the ragdoll camera keeps its world yaw through the tumble (< 0.5 deg) [M1]", math.abs(ry) < 0.5, ry)
  check("control: no mouse - the old follow camera is carried by the spinning body (> 10 deg in the same time)", math.abs(oy) > 10, oy)
  -- no jump of the yaw at the exit: the follow mode starts from the shown yaw and recentres at <= followMaxRate (240 deg/s = 4 deg per frame)
  local cfgD = CM.config()
  local yawJump, yawAt = maxOver(rec, T0 + 2.0, 8, function(r) local p = rec[math.floor(r.t * 60 + 0.5) - 1]; return p and math.abs(CM.wrapDeg(r.res.yaw - p.res.yaw)) or nil end)
  check("look: no yaw step above followMaxRate * dt (4 deg at 60 fps) from the settle to the end (no jump at the exit)",
        yawJump <= cfgD.followMaxRate / 60 + 0.01, string.format("%.3f deg at t=%.2f", yawJump, yawAt or -1))
  local exR
  for _, r in ipairs(rec) do if r.ev and r.ev.kind == "exit" then exR = r break end end
  local exJump = exR and math.abs(CM.wrapDeg(exR.res.yaw - rec[math.floor(exR.t * 60 + 0.5) - 1].res.yaw)) or 99
  check("look: the yaw does not move at the exit frame itself (< 0.5 deg: the hold goes on through the crossfade)", exJump < 0.5, exJump)
end

-- ================================================================ 4) the old camera when the natives are absent
do
  local function same(a, b)
    if #a ~= #b then return false, "length" end
    for i = 1, #a do
      local x, y = a[i].res, b[i].res
      if x.world.x ~= y.world.x or x.world.y ~= y.world.y or x.world.z ~= y.world.z or x.q.i ~= y.q.i or x.q.j ~= y.q.j or x.q.k ~= y.q.k
          or x.q.r ~= y.q.r or x.fov ~= y.fov or x.yaw ~= y.yaw or x.pitch ~= y.pitch then return false, "frame " .. i end
    end
    return true
  end
  local none = run({ natives = "none", mouse = 30 })
  local idle = run({ natives = "idle", mouse = 30 })
  local off = run({ cfg = { ragdollCam = false }, mouse = 30 })
  local ok1, w1 = same(none, off)
  local ok2, w2 = same(idle, off)
  check("natives absent: the camera is the old one bit for bit (same transforms as ragdollCam = false) over the whole scripted run", ok1, w1)
  check("natives present, state 0 / no knockdown all the time: bit for bit the old camera (a solver with an idle ragdoll input)", ok2, w2)
  local ragNone = false
  for _, r in ipairs(none) do if r.res.ragdoll ~= nil then ragNone = true end end
  check("natives absent: no ragdoll result, no solver ragdoll state", not ragNone)
  local differs = false
  for i = 1, #rec do
    if rec[i].res.world.z ~= off[i].res.world.z then differs = true break end
  end
  check("control: with the natives the camera differs from the old one (the comparison above can fail)", differs)
  -- the PSM knockdown alone (no ragdoll state) never starts an episode
  local ko = run({ knockOnly = true, mouse = 30 })
  local okK, wK = same(ko, off)
  local kEv = evCount(ko, "enter")
  check("knockdown (PSM) without a ragdoll state: no episode, the old camera bit for bit", okK and kEv == 0, wK)
end

-- ================================================================ 5) other frame rates, stuck state, teleport, vehicle hold
do
  local r30, _, s30 = run({ fps = 30, mouse = 30 })
  local m30 = maxOver(r30, T0 + TUMBLE + 0.35, 8, function(r) return r.step end)
  check("30 fps: steps from 0.35 s after the tumble on <= 10 cm (the same 3 m/s) [M3]", m30 <= 0.10, m30)
  local ex30
  for _, r in ipairs(r30) do if r.ev and r.ev.kind == "exit" then ex30 = r break end end
  local jump30 = ex30 and CM.dist(ex30.pivot, r30[math.floor(ex30.t * 30 + 0.5) - 1].pivot) or 9
  check("30 fps: exit continuity <= 6 cm and no snap events [M3]", jump30 <= 0.06 and s30 == 0, string.format("%.3f m, %d events", jump30, s30))
  local e30 = at(r30, T0)
  local j30 = math.abs(e30.step - at(r30, T0 - 1 / 30).step)
  check("30 fps: enter step differs from the last normal one by <= 16 cm (= 4 cm * (60 / 30)^2) [M2]", j30 <= 0.16, j30)

  local r144, _, s144 = run({ fps = 144, mouse = 30 })
  local m144 = maxOver(r144, T0 + TUMBLE + 0.35, 8, function(r) return r.step end)
  check("144 fps: steps from 0.35 s after the tumble on <= 2.1 cm, no snap events", m144 <= 0.021 and s144 == 0, string.format("%.4f, %d", m144, s144))

  -- a state stuck at 1: the episode ends at the time cap and does not start again
  local st = run({ stuck = true, secs = 20 })
  local ent, ext = evCount(st, "enter"), evCount(st, "exit")
  local exr
  for _, r in ipairs(st) do if r.ev and r.ev.kind == "exit" then exr = r break end end
  check("stuck state 1: one enter, one exit at the time cap (ragMaxTime 12 s), no second episode", ent == 1 and ext == 1 and exr ~= nil
        and exr.t >= T0 + 11.9 and exr.t <= T0 + 12.2 and at(st, 18).phase == nil, string.format("%d enters, %d exits, exit at %s", ent, ext, tostring(exr and exr.t)))

  -- the natives fail inside the episode (a transient pcall error: init returns nil for 5 s): the exit is a crossfade, not a cut to the capsule pivot
  local cut = run({ nativesCutAt = T0 + 1.0, mouse = 30 })
  local cutMax = maxOver(cut, T0 + 1.0, 8, function(r) return r.step end)
  local cutEv
  for _, r in ipairs(cut) do if r.ev and r.ev.kind == "exit" then cutEv = r break end end
  check("natives lost inside the episode: the pivot crosses over to the capsule pivot (steps <= 8 cm per frame, ~1 m in 0.4 s, no cut) and the episode ends",
        cutMax <= 0.08 and cutEv ~= nil and cutEv.phase == "out", string.format("%.3f m", cutMax))
  -- a teleport inside the episode (respawn / load): the spring restarts on its target, no sweep of 100 m
  local tp = run({ teleportAt = T0 + 2.0, mouse = 30 })
  local sweep = 0
  for _, r in ipairs(tp) do if r.t >= T0 + 2.0 and r.t <= T0 + 2.5 then sweep = math.max(sweep, CM.dist(r.pivot, V(r.root.x, r.root.y, r.root.z + 1.0))) end end
  check("teleport during the episode: the pivot is next to the player at once (no spring sweep over 100 m)", sweep < 2.0, sweep)

  -- a vehicle hold (noLook): the pivot / boom are overridden, the look path is the old one bit for bit
  local nl = run({ noLook = true, mouse = 30 })
  local off = run({ cfg = { ragdollCam = false }, mouse = 30 })
  local lookSame, pivDiff = true, false
  for i = 1, #nl do
    if nl[i].res.yaw ~= off[i].res.yaw then lookSame = false end
    if nl[i].res.pivot.z ~= off[i].res.pivot.z then pivDiff = true end
  end
  check("vehicle hold (noLook): the look (yaw) is the old camera's bit for bit, the pivot is still the ragdoll's", lookSame and pivDiff)
end

-- ================================================================ 6) mutations: the checks above need the feature
do
  local function evalMut(over)
    local r, _, sn = run({ cfg = over, mouse = 30 })
    local enter = at(r, T0)
    local before = at(r, T0 - 1 / 60)
    local enterJump = math.abs(enter.step - before.step)
    local stepMax = maxOver(r, T0 + TUMBLE + 0.35, 8, function(x) return x.step end)
    local lie = at(r, T0 + 1.0)
    return { enterJump = enterJump, stepMax = stepMax, lieZ = lie.pivot.z, snaps = sn, len = at(r, T0 + 2.0).res.fullLen,
             bp = at(r, T0 + 2.0).res.boomPitch, r = r }
  end
  local m1 = evalMut({ ragdollCam = false })
  check("[M1] ragdollCam = false: the pivot does not follow the pelvis (z over 1.5 m while lying), no longer boom", m1.lieZ > 1.5
        and m1.len < 2.7, string.format("z %.2f, len %.2f", m1.lieZ, m1.len))
  local m1snap = evalMut({ ragdollCam = false })
  check("[M1] ragdollCam = false: the capsule pivot meets the root jumps (pivot snap events, steps of 1 m)", m1snap.snaps >= 2
        and maxOver(m1snap.r, T0, T0 + 2, function(x) return x.step end) > 0.9, string.format("%d events", m1snap.snaps))
  local m2 = evalMut({ ragSmoothActive = 0, ragEnterSmooth = 0 }) -- (ragEnterSmooth = 0: the r13 blend-in off too, else it is a spring of its own)
  check("[M2] ragSmoothActive = 0: the enter is a jump (> 40 cm)", m2.enterJump > 0.40, m2.enterJump)
  local m3 = evalMut({ ragExitBlend = 0 })
  local exr
  for _, x in ipairs(m3.r) do if x.t > T0 + 3 and x.step and x.step > 0.1 then exr = x break end end
  check("[M3] ragExitBlend = 0: the exit is a step over 10 cm", exr ~= nil, nil)
  local m4 = evalMut({ ragDistAdd = 0, ragPitchBias = 0 })
  check("[M4] ragDistAdd = 0 / ragPitchBias = 0: the boom is the normal one", near(m4.len, m1.len, 1e-9) and near(m4.bp, at(base, T0 + 2.0).res.boomPitch, 1e-9),
        string.format("%.3f / %.3f", m4.len, m1.len))
end

-- ================================================================ 6b) 2026-11-04 r13: blend-in, early return (native get-up), vanilla path
do
  -- blend-in: V stands still, the episode begins with the pelvis target D below the shown pivot; the pivot's step per frame
  local function runStatic(fps, over)
    local cfg = CM.config(over or {})
    cfg.idleOrbit = false
    cfg.ragFrame = false -- (the spring alone; the synthetic pelvis target jumps by 1.3 m at the enter)
    local S2 = CM.newSolver(cfg)
    local dt = 1 / fps
    local zs, ws, prevZ, prevW = {}, {}, nil, nil
    local steps, wsteps = {}, {}
    for i = 1, math.floor(1.6 * fps) do
      local t = i * dt
      local on = t >= 0.5
      local ragIn = { state = on and 1 or 0, knock = on }
      if on then ragIn.pelvis, ragIn.fresh = V(0, 0, 0.3), true end
      local res = CM.solve(S2, { dt = dt, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.6, fppPitch = 0, dyaw = 0, dpitch = 0,
        moving = false, profile = MEDIUM, side = 1, w = 1, zoom = 0, crouch = false, ragdoll = ragIn })
      local z, w = res.pivot.z, res.ragdoll and res.ragdoll.w or 0
      if prevZ ~= nil then steps[#steps + 1] = math.abs(z - prevZ); wsteps[#wsteps + 1] = math.abs(w - prevW) end
      zs[#zs + 1], prevZ, prevW = z, z, w
    end
    return zs, steps, wsteps
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local dt = 1 / fps
    local zNew, stNew, wsNew = runStatic(fps)
    local zOld, stOld = runStatic(fps, { ragEnterBlend = 0 })
    local D = zNew[math.floor(0.5 * fps)] - (0.3 + 0.35)
    local mxNew, mxOld, w = 0, 0, 0
    for i = 1, #stNew do mxNew = math.max(mxNew, stNew[i]); mxOld = math.max(mxOld, stOld[i]) end
    for i = 1, #wsNew do w = math.max(w, wsNew[i]) end
    local mono, under = true, false
    for i = math.floor(0.5 * fps) + 1, #zNew do
      if zNew[i] > zNew[i - 1] + 1e-9 then mono = false end
      if zNew[i] < 0.3 + 0.35 - 1e-6 then under = true end
    end
    check(string.format("r13 blend-in %d fps: the pivot step per frame <= 0.8 * D * dt / 0.15 (%.4f m, D %.2f m) and <= 0.75 x the old spring's peak (%.4f) [M7]",
          fps, mxNew, D, mxOld), D > 0.5 and mxNew <= 0.8 * D * dt / 0.15 and mxNew <= 0.75 * mxOld, string.format("%.4f / %.4f", mxNew, mxOld))
    check(string.format("r13 blend-in %d fps: the pivot moves monotonically onto the pelvis target without an overshoot", fps), mono and not under)
    check(string.format("r13 blend-in %d fps: the boom weight step per frame <= 3.1 * dt (smoothstep over ragBlendIn 0.5 s, %.4f)", fps, w), w <= 3.1 * dt + 1e-9, w)
  end

  -- early return
  local TG0, GLEN = T0 + TUMBLE, 2.0 -- the native get-up plays from the end of the tumble, clip length 2 s, state 3 for 0.3 s after
  local function getupFn(withFrac, cutState)
    return function(t)
      if t < TG0 then return { state = 0 } end
      local tt = t - TG0
      if tt < GLEN then return { state = cutState or 2, frac = withFrac and (tt / GLEN) or nil } end
      if tt < GLEN + 0.3 then return { state = cutState and 2 or 3 } end
      return { state = 0 }
    end
  end
  local function retEv(rec) for _, r in ipairs(rec) do if r.ev and r.ev.kind == "return" then return r end end end
  local function exitEv(rec) for _, r in ipairs(rec) do if r.ev and r.ev.kind == "exit" then return r end end end
  local plain = run({ fps = 60 }) -- no getup input: the vanilla path
  local early = run({ fps = 60, getup = getupFn(true) })
  local re = retEv(early)
  check("r13 return: state 2 + clip time >= 45 % starts the return (one 'return' event at TG0 + 0.9 s +- 1 frame, none on the plain run)",
        re ~= nil and math.abs(re.t - (TG0 + 0.45 * GLEN)) <= 1 / 60 + 1e-9 and retEv(plain) == nil and evCount(early, "return") == 1,
        re and re.t)
  check("r13 return: the episode is not cut: one enter, one exit, the exit at the plain run's time", evCount(early, "enter") == 1 and evCount(early, "exit") == 1
        and math.abs(exitEv(early).t - exitEv(plain).t) <= 1 / 60 + 1e-9, exitEv(early) and exitEv(early).t)
  -- boom weight falls to 0 long before the exit; the plain run keeps it until the exit
  local wEarly = at(early, re.t + 1.0).res.ragdoll.w
  local wPlain = at(plain, re.t + 1.0).res.ragdoll.w
  check("r13 return: the boom weight is 0 one second after the trigger (plain run still 1)", wEarly <= 1e-9 and wPlain >= 0.99, string.format("%.4f / %.4f", wEarly, wPlain))
  -- pivot: smooth, ends at the capsule pivot (V's standing pivot) before the episode ends; V's head (pelvis + 0.35 + the rise) never leaves the frame
  local stepMax = maxOver(early, re.t, re.t + 1.2, function(r) return r.step end)
  local stepPlain = maxOver(plain, re.t, re.t + 1.2, function(r) return r.step end)
  local piv = at(early, re.t + 1.0)
  check("r13 return: pivot steps <= 4 cm per frame through the return and the pivot is back at the standing pivot a second later (z 1.3 .. 1.7)",
        stepMax <= 0.04 and piv.pivot.z > 1.3 and piv.pivot.z < 1.7, string.format("max %.4f, z %.3f", stepMax, piv.pivot.z))
  local headGap = 0
  for _, r in ipairs(early) do
    if r.t >= re.t and r.t <= re.t + 1.2 then
      local head = r.root.z + 1.0 + 0.7 * ss((r.t - re.t) / 0.9) -- V's head: lying 0.25 -> standing 1.7 (scripted rise over the return time)
      headGap = math.max(headGap, math.abs(r.pivot.z - head))
    end
  end
  check("r13 return: the pivot stays within 1.1 m of V's rising head height (V stays in the frame)", headGap < 1.1, headGap)
  local exE = exitEv(early)
  local exStep = exE and CM.dist(exE.pivot, early[math.floor(exE.t * 60 + 0.5) - 1].pivot) or 9
  check("r13 return: exit after the return is continuous (<= 6 cm, no snap)", exStep <= 0.06, exStep)
  -- yaw: the body ends -108 deg from the pre-hit facing: the orbit eases onto it by the end of the return, no overshoot, step bound per frame
  for _, fps in ipairs({ 30, 60, 144 }) do
    local dt = 1 / fps
    local r2 = run({ fps = fps, getup = getupFn(true), cfg = { ragRecenter = true } }) -- (r13e: the recenter is opt-in)
    local e2 = retEv(r2)
    local target = CM.wrapDeg(at(r2, e2.t).yaw) -- the body yaw
    local y0 = at(r2, e2.t - dt).res.yaw
    local D = math.abs(CM.wrapDeg(target - y0))
    local prevY, mx, mono = y0, 0, true
    local dirSign = CM.wrapDeg(target - y0) >= 0 and 1 or -1
    for _, r in ipairs(r2) do
      if r.t > e2.t and r.t <= e2.t + 1.0 then
        local d = CM.wrapDeg(r.res.yaw - prevY)
        mx = math.max(mx, math.abs(d))
        if d * dirSign < -1e-9 then mono = false end
        prevY = r.res.yaw
      end
    end
    local yEnd = at(r2, e2.t + 1.0 + 1 / fps).res.yaw
    check(string.format("r13 return %d fps: yaw %.0f deg -> V's facing: step <= 1.5 * D * dt / 0.9 * 1.1 (%.2f deg), monotone, reaches it within 0.5 deg (%.2f) [M6]", fps, D, mx, CM.wrapDeg(yEnd - target)),
          D > 90 and mx <= 1.5 * D * dt / 0.9 * 1.1 + 1e-6 and mono and math.abs(CM.wrapDeg(yEnd - target)) <= 0.5, string.format("D %.1f mx %.3f", D, mx))
  end
  local plainYaw = at(plain, re.t + 1.0).res.yaw
  local earlyYaw = at(run({ getup = getupFn(true), cfg = { ragRecenter = true } }), re.t + 1.0).res.yaw
  check("r13 return: without the recenter (default / ragRecenter = false / the offset below ragRecenterMin) the yaw is the plain run's",
        near(at(run({ getup = getupFn(true), cfg = { ragRecenter = false } }), re.t + 1.0).res.yaw, plainYaw, 1e-9)
        and near(at(early, re.t + 1.0).res.yaw, plainYaw, 1e-9)
        and near(at(run({ getup = getupFn(true), cfg = { ragRecenter = true, ragRecenterMin = 200 } }), re.t + 1.0).res.yaw, plainYaw, 1e-9)
        and math.abs(CM.wrapDeg(earlyYaw - plainYaw)) > 90, string.format("%.1f / %.1f", earlyYaw, plainYaw))
  -- r13e (user: the view must stay free while V lies and gets up; game log 23-02-49: the yaw drifted 18 deg onto V's facing during RETURN, no input):
  -- look input applies 1:1 in every phase of the episode (lying, return, exit blend) at 30 / 60 / 144 fps, and nothing pulls it back afterwards
  do
    local exT = exitEv(early).t
    local function phaseEnd(rec) local te = 0; for _, r in ipairs(rec) do if r.phase ~= nil then te = r.t end end; return te end
    local tEnd = phaseEnd(early)
    local wins = { { "lying", T0 + 0.6, T0 + 1.1 }, { "return", re.t + 0.1, re.t + 0.6 }, { "exit blend", exT + 0.02, math.min(exT + 0.3, tEnd - 0.05) } }
    for _, fps in ipairs({ 30, 60, 144 }) do
      for _, w in ipairs(wins) do
        local rate = 40 -- deg/s of look input inside the window
        local function inWin(t) return t > w[2] - 1e-9 and t <= w[3] + 1e-9 end
        local base = run({ fps = fps, getup = getupFn(true) })
        local turn = run({ fps = fps, getup = getupFn(true), mouseFn = function(t, dt) return inWin(t) and rate * dt or 0 end })
        local sumIn, worst = 0, 0
        for i, r in ipairs(turn) do
          sumIn = sumIn + r.mouse
          if r.t >= w[2] - 1 / fps and r.phase ~= nil then
            worst = math.max(worst, math.abs(CM.wrapDeg(r.res.yaw - base[i].res.yaw) - sumIn))
          end
        end
        check(string.format("r13e look %d fps, %s: the yaw changes by exactly the input (%.1f deg) and keeps it to the episode end (no pull-back, err %.2e)",
          fps, w[1], sumIn, worst), sumIn > 5 and worst <= 1e-6, worst)
      end
    end
    -- the opt-in recenter: input during the return cancels it for the rest of the return (the view is not pulled back after the input ends)
    local rc = { ragRecenter = true }
    local inpEnd = re.t + 0.25
    local turned = run({ getup = getupFn(true), cfg = rc, mouseFn = function(t, dt) return (t > re.t + 0.15 and t <= inpEnd) and 30 * dt or 0 end })
    local y0 = at(turned, inpEnd + 1 / 60).res.yaw
    local drift = 0
    for _, r in ipairs(turned) do if r.t > inpEnd + 1 / 60 and r.phase ~= nil then drift = math.max(drift, math.abs(CM.wrapDeg(r.res.yaw - y0))) end end
    check("r13e recenter (opt-in): look input during the return cancels it; no yaw change after the input until the episode end", drift <= 1e-6, drift)
    local cfgD = CM.config({})
    check("r13e defaults: ragRecenter off, pivot / boom early return kept (ragReturnEarly on)", cfgD.ragRecenter == false and cfgD.ragReturnEarly == true)
  end
  -- a mouse turn during the return is kept (no fight): the remaining angle is measured every frame
  -- trigger variants: state 3 alone (no clip time) starts the return at the hand-off; state 2 without a clip time / below the fraction does not; -1 never
  local only3 = run({ getup = getupFn(false) })
  local r3 = retEv(only3)
  check("r13 return: state 2 without a clip time never triggers, state 3 does (at the hand-off, TG0 + 2.0 s)",
        r3 ~= nil and r3.ev.state == 3 and math.abs(r3.t - (TG0 + GLEN)) <= 1 / 60 + 1e-9, r3 and r3.t)
  local low = run({ getup = function(t) if t < TG0 then return { state = 0 } end return { state = 2, frac = 0.3 } end })
  check("r13 return: state 2 below ragReturnFrac (30 %) and no state 3: no early return, the plain camera", retEv(low) == nil
        and near(at(low, T0 + 3).res.pivot.z, at(plain, T0 + 3).res.pivot.z, 1e-9))
  local abort = run({ getup = function(t) if t < TG0 then return { state = 0 } end return { state = -1, frac = 0.9 } end })
  check("r13 return: aborted get-up (-1): no early return", retEv(abort) == nil)
  local noEnd = run({ getup = function(t) if t < TG0 then return { state = 0 } end return { state = 2, frac = 0.5 } end, stuck = true, secs = 8 })
  check("r13 return: no second enter after the trigger while the state / knock stay on (one enter)", evCount(noEnd, "enter") == 1 and retEv(noEnd) ~= nil)
  -- vanilla path: nothing changes when the native get-up does not exist / idles / the switch is off (bit for bit the run without any getup input)
  local function same(a, b)
    for i = 1, #a do
      local x, y = a[i].res, b[i].res
      if x.world.x ~= y.world.x or x.world.y ~= y.world.y or x.world.z ~= y.world.z or x.yaw ~= y.yaw or x.pivot.z ~= y.pivot.z then return false, i end
    end
    return true
  end
  local idle = run({ getup = function() return { state = 0 } end })
  local sIdle, iIdle = same(plain, idle)
  check("r13 vanilla: a get-up input that stays idle (state 0) = the plain run bit for bit [M5]", sIdle, iIdle)
  local off = run({ getup = getupFn(true), cfg = { ragReturnEarly = false } })
  local sOff, iOff = same(plain, off)
  check("r13 vanilla: ragReturnEarly = false with a playing get-up = the plain run bit for bit [M5]", sOff and retEv(off) == nil, iOff)
  local off2 = run({ getup = getupFn(true), cfg = { ragReturnTime = 0 } })
  check("r13 vanilla: ragReturnTime = 0 = the plain run bit for bit", same(plain, off2) and retEv(off2) == nil)
  -- old values restore the old camera of the first episode: the blend-in off (ragEnterBlend 0) differs from the default only in the enter frames
  local oldEnter = run({ cfg = { ragEnterBlend = 0, ragEnterSmooth = 0 }, mouse = 30 })
  local defEnter = run({ mouse = 30 })
  local late = true
  for i = 1, #oldEnter do
    if oldEnter[i].t > T0 + 1.5 and CM.dist(oldEnter[i].res.world, defEnter[i].res.world) > 0.35 then late = false end
  end
  check("r13 blend-in: with the old values (0 / 0) the camera is the old one; the default differs only through the first second of the episode (the spring has caught up)", late)
end

-- ================================================================ 6c) 2026-11-04 r13: the look handover on the ON frame (game look write OFF + own pitch)
-- The solver's shown orientation (res.q = oyaw / shownPitch) comes from its own look state; the game's look (inp.lookYaw / inp.fppPitch, synced and
-- written until the ON frame) is only an input of the frames before it. A game look 13 deg away from the shown camera (yaw and pitch) must not
-- change the shown orientation on the ON frame: the step of the shown yaw / pitch stays within the smooth boom-bias slope.
do
  for _, fps in ipairs({ 30, 60, 144 }) do
    local dt = 1 / fps
    local cfg = CM.config()
    cfg.idleOrbit = false
    local S3 = CM.newSolver(cfg)
    local onFrame = math.floor(1.0 * fps)
    local shown, prevY, prevP, maxStep, onStep, preMax = nil, nil, nil, 0, nil, 0
    for i = 1, math.floor(2.0 * fps) do
      local on = i >= onFrame
      -- before the episode: the game's look is written from the shown camera + a 13 deg mismatch in both axes (the read-back lag of the look write)
      local gy, gp = 13, -14.3 + 13
      local ragIn = { state = on and 1 or 0, knock = on }
      if on then ragIn.pelvis, ragIn.fresh = V(0, 0, 0.9), true end
      local res = CM.solve(S3, { dt = dt, playerPos = V(0, 0, 0), playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.6, fppPitch = gp, lookYaw = (not on) and gy or nil,
        freeHeld = true, dyaw = 0, dpitch = (i == 1) and -14.3 or 0, moving = false, profile = MEDIUM, side = 1, w = 1, zoom = 0, crouch = false,
        lookWriteOn = true, ragdoll = ragIn })
      local y, p = res.yaw, res.shownPitch or res.pitch
      if prevY ~= nil then
        local step = math.abs(CM.wrapDeg(y - prevY)) + math.abs(p - prevP)
        if i > 3 and i < onFrame then preMax = math.max(preMax, step) end
        if i == onFrame then onStep = step end
        if i >= onFrame and i < onFrame + math.floor(0.5 * fps) then maxStep = math.max(maxStep, step) end
      end
      prevY, prevP = y, p
    end
    check(string.format("r13 handover %d fps: the shown yaw + pitch step on the ON frame is %.3f deg (<= 1 deg x 60 / fps; before the episode up to %.3f), and %.3f deg over the first 0.5 s", fps,
          onStep or -1, preMax, maxStep), onStep ~= nil and onStep <= 1.0 * 60 / fps + 1e-6 and maxStep <= 1.0 * 60 / fps + 1e-6, string.format("%s / %s", tostring(onStep), tostring(maxStep)))
  end
end

-- ================================================================ 6d) 2026-11-04 r13b: the framing constraint - a launch (V 6 m/s away / sideways and up) keeps head and pelvis in the frame
-- (game log 18:32:48: the slower enter spring lagged the launched body, V's head left the frame at the bottom). V stands (pelvis 1.0 m over the root) for
-- 0.5 s, then the ragdoll owns the body: horizontal 6 m/s for 0.5 s decaying to 0 by 1.0 s, vertical 5 m/s up under gravity until the pelvis lies 0.25 m high.
do
  local function launch(fps, dir, over, speed)
    local dt = 1 / fps
    local cfg = CM.config(over or {})
    cfg.idleOrbit = false
    local S4 = CM.newSolver(cfg)
    local hx, hy = dir.x, dir.y
    local out = { worstV = 0, worstH = 0, maxOri = 0, maxStep = 0, behind = false }
    local prevY, prevP, prevPiv
    local tanV = math.tan(math.rad(MEDIUM.fov * 0.5)) -- vertical half angle of the profile's FOV (the solver's FOV settles on it)
    for i = 1, math.floor(3.0 * fps) do
      local t = i * dt
      local te = t - 0.5
      local on = te >= 0
      local px, py, pz = 0, 0, 1.0
      if on then
        -- horizontal travel: 6 m/s until 0.5 s (3 m), then decaying linearly to 0 at 1.0 s (another 1.5 m)
        local d = (te < 0.5) and (6 * te) or (3 + 1.5 - 1.5 * (1 - math.min(1, (te - 0.5) / 0.5)) ^ 2)
        d = d * (speed or 6) / 6
        px, py = hx * d, hy * d
        pz = math.max(0.25, 1.0 + 5 * te - 4.9 * te * te)
      end
      local root = V(px, py, 0)
      local pel = V(px, py, pz)
      local ragIn = { state = on and 1 or 0, knock = on }
      if on then ragIn.pelvis, ragIn.fresh = CM.sub(pel, root), true end
      local res = CM.solve(S4, { dt = dt, playerPos = root, playerQ = CM.QID, bodyYaw = 0, eyeZ = 1.6, fppPitch = 0, dyaw = 0, dpitch = 0, moving = false,
        profile = MEDIUM, side = 1, w = 1, zoom = 0, crouch = false, ragdoll = ragIn })
      if on then
        local right, fwd, up = CM.qrot(res.q, V(1, 0, 0)), CM.qrot(res.q, V(0, 1, 0)), CM.qrot(res.q, V(0, 0, 1))
        for _, pt in ipairs({ V(pel.x, pel.y, pel.z + 0.6), pel }) do -- the head 0.6 m over the pelvis, the pelvis
          local rel = CM.sub(pt, res.world)
          local f = CM.dot(rel, fwd)
          if f < 0.2 then out.behind = true else
            out.worstV = math.max(out.worstV, math.abs(CM.dot(rel, up)) / (f * tanV))
            out.worstH = math.max(out.worstH, math.abs(CM.dot(rel, right)) / (f * tanV * 16 / 9))
          end
        end
        local y, p = res.yaw, res.shownPitch or res.pitch
        if prevY ~= nil then out.maxOri = math.max(out.maxOri, math.abs(CM.wrapDeg(y - prevY)) + math.abs(p - prevP)) end
        if prevPiv ~= nil then out.maxStep = math.max(out.maxStep, CM.dist(res.pivot, prevPiv)) end
      end
      prevY, prevP, prevPiv = res.yaw, res.shownPitch or res.pitch, res.pivot
    end
    return out
  end
  for _, fps in ipairs({ 30, 60, 144 }) do
    local fast = launch(fps, V(0, 1, 0), nil, 10)
    check(string.format("r13b frame %d fps, launch away at 10 m/s: head + pelvis inside the band (vertical %.2f, horizontal %.2f), orientation step %.3f deg", fps, fast.worstV, fast.worstH, fast.maxOri),
          fast.worstV <= 0.70 and fast.worstH <= 0.70 and not fast.behind and fast.maxOri <= 1.0 * 60 / fps + 1e-6, string.format("%.2f / %.2f", fast.worstV, fast.worstH))
    local lim = 0.70 -- the frame band: |angle| <= 70 % of the half FOV (vertical / horizontal; the constraint's own margin is 75 % of a head 0.75 m over the pelvis)
    for _, d in ipairs({ { n = "away", v = V(0, 1, 0) }, { n = "sideways", v = V(1, 0, 0) } }) do
      local on = launch(fps, d.v)
      local off = launch(fps, d.v, { ragFrame = false })
      check(string.format("r13b frame %d fps, launch %s: head + pelvis inside the band every frame (vertical %.2f, horizontal %.2f of the half FOV; limit %.2f) [M8]", fps, d.n,
            on.worstV, on.worstH, lim), on.worstV <= lim and on.worstH <= lim and not on.behind, string.format("%.2f / %.2f", on.worstV, on.worstH))
      check(string.format("r13b frame %d fps, launch %s: the shown orientation step <= 1 deg x 60 / fps (%.3f deg) and the pivot step per frame <= 0.22 m x 60 / fps (%.3f m)", fps, d.n, on.maxOri, on.maxStep),
            on.maxOri <= 1.0 * 60 / fps + 1e-6 and on.maxStep <= 0.22 * 60 / fps + 1e-6, string.format("%.3f / %.3f", on.maxOri, on.maxStep))
      if fps == 60 then
        check(string.format("r13b frame %d fps, launch %s: control - without the constraint the body leaves the band (vertical %.2f / horizontal %.2f) [M8]", fps, d.n, off.worstV, off.worstH),
              math.max(off.worstV, off.worstH) > lim, string.format("%.2f / %.2f", off.worstV, off.worstH))
      end
    end
  end
end

-- ================================================================ 7) utpp_camera/init.lua in the mocked CET game
-- the harness (makeWorld, withEnv, loadCore, loadCamera, frames, count) of test_utpp_camera_load.lua is compiled from that file
local H
do
  local path = (UTPP_LUA_FIXTURE_DIR or ".") .. "/../test_utpp_camera_load.lua"
  local f = io.open(path, "rb")
  local src = f and f:read("*a") or nil
  if f then f:close() end
  if src then
    local s1 = string.find(src, "local function makeWorld(opts)", 1, true)
    local s2, e2 = string.find(src, "local function action(W, name, value)", 1, true)
    local eol = e2 and string.find(src, "\n", e2, true)
    if s1 and eol then
      local chunk = "local CM, check, TMP = ...\n" .. string.sub(src, s1, eol)
        .. "\nreturn { makeWorld = makeWorld, withEnv = withEnv, loadCore = loadCore, loadCamera = loadCamera, frames = frames, count = count }"
      local fn, err = loadstring(chunk, "=load-harness")
      if fn then
        local okR, tbl = pcall(fn, CM, check, (os.getenv("TEMP") or os.getenv("TMP") or ".") .. "\\")
        if okR then H = tbl else print("harness failed: " .. tostring(tbl)) end
      else
        print("harness compile failed: " .. tostring(err))
      end
    end
  end
  check("init: the mocked CET harness of test_utpp_camera_load.lua is available", H ~= nil)
end
if H then
  package.path = UTPP_CAMERA_DIR .. "\\?.lua;" .. UTPP_CAMERA_DIR .. "/?.lua;" .. package.path
  -- a mocked world with the ragdoll natives (nil natives = a DLL without them). The scripted body: root path of the episode above.
  local function world(natives)
    local W = H.makeWorld({})
    W.rag = { state = 0, pelvis = { x = 0, y = 0, z = 0.9, w = 1 } }
    if natives then
      W.env.Game.UTPPRagdollState = function() W.ragStateCalls = (W.ragStateCalls or 0) + 1; return W.rag.state end
      W.env.Game.UTPPRagdollPelvis = function() W.ragPelvisCalls = (W.ragPelvisCalls or 0) + 1; return W.rag.pelvis end
    end
    return W
  end
  local function start(W)
    local C = H.loadCore(W)
    local cam = H.loadCamera(W)
    H.withEnv(W, function() W.handlers.onInit() end)
    C.Set("camera.idleOrbit", false)
    H.frames(W, 1.0)
    H.withEnv(W, function() cam.Toggle("key") end)
    H.frames(W, 2.0)
    return C, cam
  end
  -- drive the scripted episode (root path of body() shifted to the mock world's start position); returns the shown poses
  local function drive(W, cam, secs, withNatives)
    local p0 = { x = W.pos.x, y = W.pos.y, z = W.pos.z }
    local shown, t = {}, 0
    local logs0 = #W.lines
    H.frames(W, secs, 60, function(i)
      t = i / 60
      local root, pelvisW, yaw = body(t + T0 - 1.0) -- t = 1.0 s is the hit
      local standing = t + T0 - 1.0 < T0
      W.pos = CM.v3(p0.x + root.x, p0.y + root.y, p0.z + (standing and 0 or root.z))
      W.yaw = standing and 0 or yaw
      local st = stateAt(t + T0 - 1.0)
      if withNatives then
        W.rag.state = st
        local q = CM.qFromYawPitch(W.yaw, 0)
        local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root))
        W.rag.pelvis = { x = m.x, y = m.y, z = m.z, w = (st ~= 0) and 1 or 0 }
      end
      local tt = t + T0 - 1.0
      W.psm.LocomotionDetailed = (tt >= T0 + TUMBLE and tt < KNOCK_END) and 29 or 1
    end)
    local last = cam.G.last
    return last, logs0
  end
  -- a: natives absent
  do
    local W = world(false)
    local C, cam = start(W)
    local last = drive(W, cam, 7.0, false)
    check("init: natives absent - the camera runs through the whole script, the missing native is logged once, no ragdoll state",
          cam.G.last ~= nil and H.count(W.lines, "ragdoll camera: Game.UTPPRagdollState unavailable") == 1
          and H.count(W.lines, "ragdoll camera ON") == 0 and cam.G.ragPhase == nil and cam.stats.errors == 0, table.concat(W.lines, " | "))
  end
  -- b: natives present. Scripted episode in the mocked game; while the PSM runs the knockdown the game's camera parameter set drops the heading
  -- lock every frame (the log "heading lock lost (the game applied a camera parameter set: PSM Locomotion 0, LocomotionDetailed 1)")
  local function episode(W, cam, dropLock)
    local p0 = { x = W.pos.x, y = W.pos.y, z = W.pos.z }
    local out = { relocksDuring = 0, lockedDuring = false, sawOn = false, lines0 = #W.lines, base = cam.stats.lockRelocks or 0 }
    local atPhaseEnd
    H.frames(W, 7.0, 60, function(i)
      local tt = i / 60 + T0 - 1.0
      local root, pelvisW, yaw = body(tt)
      local standing = tt < T0
      W.pos = CM.v3(p0.x + root.x, p0.y + root.y, p0.z + (standing and 0 or root.z))
      W.yaw = standing and 0 or yaw
      W.look = W.yaw
      local st = stateAt(tt)
      W.rag.state = st
      local q = CM.qFromYawPitch(W.yaw, 0)
      local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root))
      W.rag.pelvis = { x = m.x, y = m.y, z = m.z, w = (st ~= 0) and 1 or 0 }
      W.psm.LocomotionDetailed = (tt >= T0 + TUMBLE and tt < KNOCK_END) and 29 or 1
      if dropLock and tt >= T0 and tt < KNOCK_END + 0.05 then W.fpp.headingLocked = false end -- the game's parameter set, scripted by time
      if cam.G.ragPhase ~= nil then
        out.sawOn = true
        out.relocksDuring = math.max(out.relocksDuring, (cam.stats.lockRelocks or 0) - out.base)
        if W.fpp.headingLocked == true and not dropLock then out.lockedDuring = true end
      elseif out.sawOn and atPhaseEnd == nil then
        atPhaseEnd = (cam.stats.lockRelocks or 0) - out.base
      end
    end)
    out.after = (cam.stats.lockRelocks or 0) - out.base
    out.relocksAtEnd = atPhaseEnd
    return out
  end
  do
    local W = world(true)
    local C, cam = start(W)
    cam.FreeLook(true)
    H.frames(W, 0.5)
    check("init: free look ON before the episode: the heading lock is set", W.fpp.headingLocked == true)
    local o = episode(W, cam, true)
    check("init: natives present - one ragdoll camera ON line and one OFF line over the episode", o.sawOn
          and H.count(W.lines, "ragdoll camera ON") == 1 and H.count(W.lines, "ragdoll camera OFF") == 1, table.concat(W.lines, " | "))
    check("init: the heading lock dropped by the game's parameter set during the episode (knockdown) is NOT set again inside it [M1]",
          o.relocksDuring == 0, o.relocksDuring)
    check("init: ... and it is set again after the episode + grace (the free look is still on: 'heading lock lost ... set again')",
          o.after >= 1 and W.fpp.headingLocked == true and H.count(W.lines, "heading lock lost (the game applied a camera parameter set") >= 1,
          string.format("%d relocks after, lock %s", o.after, tostring(W.fpp.headingLocked)))
    check("init: no camera update errors through the episode", cam.stats.errors == 0, cam.stats.errors)
    check("init: ragdoll flags cleared after the episode (no stale freeze)", cam.G.ragPhase == nil and cam.G.ragEp == false and not cam.G.ragBoom)
  end
  -- b2: no lock before the episode: the free yaw of the episode must not set it (wrapper around setHeadingLock)
  do
    local W = world(true)
    local C, cam = start(W)
    check("init: follow mode before the episode: heading not locked", W.fpp.headingLocked == false)
    local o = episode(W, cam, false)
    check("init: the episode's free yaw does not write the heading lock (never locked during the episode) [M1]", o.sawOn and not o.lockedDuring)
  end
  -- b3: frames without a camera solve inside the episode (first person / menu) drop the episode state, no sweep, no error
  do
    local W = world(true)
    local C, cam = start(W)
    local p0 = { x = W.pos.x, y = W.pos.y, z = W.pos.z }
    local prev, maxStep = nil, 0
    H.frames(W, 3.0, 60, function(i)
      local tt = i / 60 + T0 - 1.0
      local root, pelvisW, yaw = body(tt)
      W.pos = CM.v3(p0.x + root.x, p0.y + root.y, p0.z + ((tt < T0) and 0 or root.z))
      W.yaw = (tt < T0) and 0 or yaw
      local st = stateAt(tt)
      W.rag.state = st
      local q = CM.qFromYawPitch(W.yaw, 0)
      local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root))
      W.rag.pelvis = { x = m.x, y = m.y, z = m.z, w = (st ~= 0) and 1 or 0 }
      W.menu = (tt >= T0 + 0.5 and tt < T0 + 0.6) -- the menu freezes the camera for 6 frames inside the tumble
      local g = cam.G.last
      if tt >= T0 + 1.0 and g and prev then maxStep = math.max(maxStep, CM.dist(g.pivot, prev)) end
      prev = g and g.pivot or nil
    end)
    check("init: smoke - a menu gap inside the episode: the camera runs on without errors, no pivot step above 8 cm after the settle (does not discriminate the gap drop)",
          cam.stats.errors == 0 and maxStep <= 0.08, string.format("%d errors, %.3f m", cam.stats.errors, maxStep))
  end
  -- d: 2026-11-04 r11 (game log 09:13:44): a car hit at a jump-out landing started a UTPP ragdoll episode whose vanilla knockdown effect never arrived (aborted
  -- after 0.44 s 'knockdown effect missing'); the camera used to turn ON at native state 1 and dipped the view. Game.UTPPRagdollConfirmed() (true while an episode
  -- runs / blends out AND its knockdown effect was seen) gates the camera ON. Unconfirmed state 1 for 0.44 s, then 0: no ON line, the pivot identical to a run
  -- without the natives. Confirmed 0.05 s after the begin: ON as before, and once on the later answers do not end it (state 2, the vanilla get-up).
  do
    local function world2(natives, conf)
      local W = world(natives)
      if natives and conf ~= nil then W.env.Game.UTPPRagdollConfirmed = function() W.confirmedCalls = (W.confirmedCalls or 0) + 1; return W.rag.confirmed end end
      return W
    end
    -- stFn(tt) -> state, confFn(tt) -> bool|nil; returns the shown pivots per frame, the first frame with a ragdoll phase and the world
    local function play(W, cam, stFn, confFn)
      local p0 = { x = W.pos.x, y = W.pos.y, z = W.pos.z }
      local piv, firstOn = {}, nil
      H.frames(W, 7.0, 60, function(i)
        local tt = i / 60 + T0 - 1.0
        local root, pelvisW, yaw = body(tt)
        local standing = tt < T0
        W.pos = CM.v3(p0.x + root.x, p0.y + root.y, p0.z + (standing and 0 or root.z))
        W.yaw = standing and 0 or yaw
        W.look = W.yaw
        local st = stFn(tt)
        W.rag.state = st
        W.rag.confirmed = confFn(tt)
        local q = CM.qFromYawPitch(W.yaw, 0)
        local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root))
        W.rag.pelvis = { x = m.x, y = m.y, z = m.z, w = (st ~= 0) and 1 or 0 }
        W.psm.LocomotionDetailed = (tt >= T0 + TUMBLE and tt < KNOCK_END) and 29 or 1
        local g = cam.G.last
        piv[i] = g and { x = g.pivot.x, y = g.pivot.y, z = g.pivot.z } or nil
        if firstOn == nil and cam.G.ragPhase ~= nil then firstOn = tt - T0 end
      end)
      return piv, firstOn
    end
    local abort = function(tt) return (tt >= T0 and tt < T0 + 0.44) and 1 or 0 end -- state 1 for 0.44 s, then the episode is gone
    -- unconfirmed
    local Wa = world2(true, true)
    local Ca, camA = start(Wa)
    local pivA, onA = play(Wa, camA, abort, function() return false end)
    local Wc = world2(false)
    local Cc, camC = start(Wc)
    local pivC = play(Wc, camC, abort, function() return false end)
    local worstDiff, n = 0, 0
    for i = 1, #pivC do
      if pivA[i] and pivC[i] then worstDiff = math.max(worstDiff, CM.dist(V(pivA[i].x, pivA[i].y, pivA[i].z), V(pivC[i].x, pivC[i].y, pivC[i].z))); n = n + 1 end
    end
    check("init: [r11] state 1 for 0.44 s, never confirmed, then state 0: the ragdoll camera never turns ON (no ON line, no phase, the native asked)",
          onA == nil and H.count(Wa.lines, "ragdoll camera ON") == 0 and camA.G.ragPhase == nil and (Wa.confirmedCalls or 0) > 0 and camA.stats.errors == 0,
          table.concat(Wa.lines, " | "))
    check(string.format("init: [r11] ... and the pivot is the one of a run without the natives in all %d frames (largest difference %.2e m)", n, worstDiff), n > 300 and worstDiff < 1e-9)
    -- mutation: the native answers true at once (the old behaviour: state 1 turns it ON)
    local Wm = world2(true, true)
    local Cm, camM = start(Wm)
    local pivM, onM = play(Wm, camM, abort, function() return true end)
    local mDiff = 0
    for i = 1, #pivC do
      if pivM[i] and pivC[i] then mDiff = math.max(mDiff, CM.dist(V(pivM[i].x, pivM[i].y, pivM[i].z), V(pivC[i].x, pivC[i].y, pivC[i].z))) end
    end
    check(string.format("init: [r11] mutation (confirmed from the first frame = the old behaviour): the camera turns ON at the begin (+%.2f s) and the pivot dips by %.2f m",
          onM or -1, mDiff), onM ~= nil and onM <= 0.1 and H.count(Wm.lines, "ragdoll camera ON") >= 1 and mDiff > 0.05, string.format("%s %.3f", tostring(onM), mDiff))
    -- confirmed after 0.05 s, withdrawn again 0.25 s later: ON at +0.05 s (within 2 frames), once; the later 'false' does not end it
    local Wb = world2(true, true)
    local Cb, camB = start(Wb)
    local full = function(tt) return stateAt(tt) end
    local pivB, onB = play(Wb, camB, full, function(tt) return tt >= T0 + 0.05 and tt < T0 + 0.30 end)
    check(string.format("init: [r11] confirmed 0.05 s after the begin: the camera turns ON at +%.3f s as before, one ON and one OFF line, a later 'false' does not end the episode",
          onB or -1), onB ~= nil and onB >= 0.05 - 1e-6 and onB <= 0.05 + 2 / 60 and H.count(Wb.lines, "ragdoll camera ON") == 1 and H.count(Wb.lines, "ragdoll camera OFF") == 1
          and camB.stats.errors == 0, table.concat(Wb.lines, " | "))
    -- a DLL / reds without the native: the old behaviour (ON at the begin)
    local Wn = world2(true, nil)
    local Cn, camN = start(Wn)
    local _, onN = play(Wn, camN, full, function() return nil end)
    check(string.format("init: [r11] Game.UTPPRagdollConfirmed unavailable: the old behaviour - ON at the begin (+%.3f s)", onN or -1), onN ~= nil and onN <= 2 / 60 + 1e-6)
  end
  -- r13: the native get-up input of M.RagdollRead (Game.UTPPRagdollGetUpState / Status) and the RETURN line of the episode
  do
    local W = world(true)
    local C, cam = start(W)
    local gu = { state = 2, status = "getup setting=1 avail=ok phase=play clip=getup_front t=0.90/2.00 w=1.00 rms=0.020 lift=0.000 owns=1" }
    W.env.Game.UTPPRagdollGetUpState = function() W.guCalls = (W.guCalls or 0) + 1; return gu.state end
    W.env.Game.UTPPRagdollGetUpStatus = function() W.guStatusCalls = (W.guStatusCalls or 0) + 1; return gu.status end
    local function read(G) local r; H.withEnv(W, function() r = cam.RagdollRead(G, 1, false) end); return r end
    W.rag.state = 2
    local r1 = read({ clock = 100, ragEp = true })
    check("init r13: episode on, get-up state 2, 't=0.90/2.00' -> getup { state 2, frac 0.45 }", r1 and r1.getup and r1.getup.state == 2 and near(r1.getup.frac or -1, 0.45, 1e-9))
    local r0 = read({ clock = 100 })
    check("init r13: no episode (G.ragEp unset): the get-up natives are not read", r0 and r0.getup == nil and (W.guCalls or 0) == 1)
    gu.status = "getup t=0.90/0.00 w=1"
    local rz = read({ clock = 100, ragEp = true })
    check("init r13: clip length 0 -> no frac (no division), state kept", rz and rz.getup and rz.getup.state == 2 and rz.getup.frac == nil)
    gu.state = 3
    local callsBefore = W.guStatusCalls
    local r3 = read({ clock = 100, ragEp = true })
    check("init r13: state 3 -> state only, the status string is not read", r3 and r3.getup and r3.getup.state == 3 and W.guStatusCalls == callsBefore)
    cam.cfg.ragReturnEarly = false
    local rOff = read({ clock = 100, ragEp = true })
    check("init r13: ragReturnEarly = false -> no getup input", rOff and rOff.getup == nil)
    cam.cfg.ragReturnEarly = true
    W.env.Game.UTPPRagdollGetUpState = nil
    local G2 = { clock = 200, ragEp = true }
    local rM = read(G2)
    local rM2 = read({ clock = 201, ragEp = true, ragGuNaAt = G2.ragGuNaAt })
    check("init r13: native missing -> no getup input (the old return), retried in 5 s", rM and rM.getup == nil and G2.ragGuNaAt == 205 and rM2 and rM2.getup == nil)
  end
  -- the scripted episode with the native get-up: one RETURN line between ON and OFF, one ON, one OFF
  do
    local W = world(true)
    local C, cam = start(W)
    W.env.Game.UTPPRagdollGetUpState = function()
      local tt = W.epT or 0
      if tt < T0 + TUMBLE then return 0 end
      if tt < T0 + TUMBLE + 2.0 then return 2 end
      if tt < T0 + TUMBLE + 2.3 then return 3 end
      return 0
    end
    W.env.Game.UTPPRagdollGetUpStatus = function()
      local len = 2.0
      return string.format("getup phase=play clip=x t=%.2f/%.2f w=1", math.max(0, (W.epT or 0) - T0 - TUMBLE), len)
    end
    local p0 = { x = W.pos.x, y = W.pos.y, z = W.pos.z }
    H.frames(W, 7.0, 60, function(i)
      local tt = i / 60 + T0 - 1.0
      W.epT = tt
      local root, pelvisW, yaw = body(tt)
      local standing = tt < T0
      W.pos = CM.v3(p0.x + root.x, p0.y + root.y, p0.z + (standing and 0 or root.z))
      W.yaw = standing and 0 or yaw
      W.look = W.yaw
      local st = stateAt(tt)
      W.rag.state = st
      local q = CM.qFromYawPitch(W.yaw, 0)
      local m = CM.qrot(CM.qconj(q), CM.sub(pelvisW, root))
      W.rag.pelvis = { x = m.x, y = m.y, z = m.z, w = (st ~= 0) and 1 or 0 }
      W.psm.LocomotionDetailed = (tt >= T0 + TUMBLE and tt < KNOCK_END) and 29 or 1
    end)
    local lines = table.concat(W.lines, "\n")
    local iOn = lines:find("ragdoll camera ON", 1, true)
    local iRet = lines:find("ragdoll camera RETURN", 1, true)
    local iOff = lines:find("ragdoll camera OFF", 1, true)
    check("init r13: the episode with a native get-up logs ON, RETURN (clip 45 %), OFF once each, in this order, no camera errors",
          iOn and iRet and iOff and iOn < iRet and iRet < iOff and H.count(W.lines, "ragdoll camera RETURN") == 1 and H.count(W.lines, "ragdoll camera ON") == 1
          and H.count(W.lines, "ragdoll camera OFF") == 1 and cam.stats.errors == 0 and cam.G.ragPhase == nil, lines)
  end
  -- c: switch off: the natives are never called, the old camera
  do
    local W = world(true)
    local C, cam = start(W)
    cam.cfg.ragdollCam = false
    W.ragStateCalls, W.ragPelvisCalls = 0, 0
    drive(W, cam, 7.0, true)
    check("init: ragdollCam = false: the natives are not called, no ON line", (W.ragStateCalls or 0) == 0 and H.count(W.lines, "ragdoll camera ON") == 0,
          tostring(W.ragStateCalls))
  end
end

print(string.format("utpp_camera ragdoll test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_camera ragdoll test failed")
